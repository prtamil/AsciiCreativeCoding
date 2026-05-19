/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lattice_gas.c — FHP-I lattice gas automaton (Frisch-Hasslacher-Pomeau, 1986)
 *
 * DEMO: A two-step rule on a hexagonal grid produces REAL FLUID
 *       behaviour.  Particles flow in from the left, collide with
 *       each other, bounce off obstacles, and the macroscopic
 *       picture matches the Navier-Stokes equations.  Six preset
 *       SCENES (1..6) — flow past a cylinder, jets through slits,
 *       Poiseuille channel, free gas equilibrium.  Five colour
 *       THEMES.  All emergent: there is no "fluid equation"
 *       anywhere in the code, just six bits per cell + a lookup
 *       table.
 *
 *       Each cell holds ONE BYTE — six bits, one per hexagonal
 *       direction (E, NE, NW, W, SW, SE).  A bit is "1" if a
 *       particle is moving in that direction, else "0."  Each
 *       physics tick:
 *
 *           COLLIDE  — for each cell, replace its 6-bit state
 *                       with the post-collision state from a
 *                       precomputed table.  Mass + momentum
 *                       conserved; head-on pairs scatter ±60°.
 *           STREAM   — every particle hops one cell in its
 *                       direction.  Walls reflect particles back
 *                       (bounce-back boundary).
 *
 *       That's it.  Average over a 3×3 patch and you see the
 *       macroscopic FLOW FIELD — wakes, jets, boundary layers,
 *       Bernoulli pressure drops.
 *
 * Study alongside:
 *   fluid/navier_stokes.c   — Eulerian grid solver (the OPPOSITE
 *                              philosophy: solve the PDE directly,
 *                              don't simulate particles).
 *   fluid/fluid_sph.c       — Lagrangian particles WITHOUT a lattice
 *                              (continuous positions).  Read T1 of
 *                              that file to see the spectrum.
 *   procedural/cellular_automata/   — other CA examples (Conway,
 *                              Wireworld, etc.).
 *
 * Section map:
 *   §1  config              — every tunable constant
 *   §2  clock               — monotonic ns timer + sleep
 *   §3  rng                 — xorshift32 (no global lock)
 *   §4  hex_directions      — 6-direction unit-vector tables
 *   §5  collision rules     — CollisionRules + g_collisions lookup, built at startup
 *   §6  momentum_extract    — bit-popcount density + signed mx
 *   §7  themes              — 5 momentum-to-colour palettes
 *   §8  colors              — pair init + theme apply
 *   §9  grid_state          — main + double-buffer + wall mask
 *   §10 step_inject_inlet   — drive flow from left edge
 *   §11 step_collide        — collision phase (table lookup)
 *   §12 step_stream         — streaming + bounce-back
 *   §13 step                — full physics tick
 *   §14 presets             — 6 preset definitions
 *   §15 scene_load          — apply preset to walls + state
 *   §16 cell_neighbourhood  — 3×3 spatial average
 *   §17 render              — fluid + walls + glyph ramp
 *   §18 hud                 — top status + bottom hint
 *   §19 screen              — ncurses init / cleanup
 *   §20 app                 — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   p / space        pause / resume
 *   r                reset (reload current preset)
 *   n / N            next / prev preset (1..6)
 *   t / T            next / prev theme  (1..5)
 *   i / I            inlet probability up / down
 *   + / -            physics steps per render frame up / down
 *   ] / [            simulation Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/lattice_gas.c -o lattice_gas \
 *       -lncurses
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      T1-T2 establish CA-fluid worldview + WHY HEX.  T3 explains
 *      the bit-packed cell.  T4 derives the collision rules.  T5
 *      covers streaming & walls.  T6-T7 describe the visualisation.
 *      T8 closes with the H-theorem and the link to Navier-Stokes.
 *   2. §4 hex_directions + §5 g_collisions.lookup — the entire physics
 *      lookup data.  Print the tables, study them: that's the model.
 *   3. §13 step — three lines: inject, collide, stream.
 *   4. §11 step_collide — read AFTER tutorial T4.
 *   5. §12 step_stream — read AFTER tutorial T5.
 *   6. §17 render + §16 cell_neighbourhood — read AFTER tutorial T6.
 *   7. §15 scene_load + §14 presets — orchestration.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   g_scene.particle_grid[r][c]            uint8_t per cell, 6 bits used
 *   g_scene.stream_buffer[r][c]            scratch for double-buffered streaming
 *   g_scene.wall_mask[r][c]                true if cell is solid obstacle
 *
 *   g_collisions.lookup[parity][bits]  post-collision state lookup
 *   parity                          (row + col) & 1 — alternating
 *                                    chirality cancels global rotation
 *
 *   hex_direction_drow[parity][d]  Δrow for direction d in row parity
 *   hex_direction_dcol[parity][d]  Δcol  ... same convention
 *
 *   g_scene.physics_tick_count             monotonic counter
 *   xorshift_state                 PRNG state
 *
 *   g_scene.inlet_probability_per_cell     per-cell probability that left edge
 *                                    gets an east-going particle
 *   g_scene.physics_steps_per_frame        time-lapse multiplier
 *   g_scene.sim_steps_per_second           physics tick rate
 *
 *   density_estimate               bit-count avg over 3×3 (∈ [0, 6])
 *   momentum_x_doubled             integer mx*2 from bits (∈ [-2, +2])
 *
 * Background you need
 * ───────────────────
 *   - Bitwise ops: |, &, <<, >> and that 6 bits fit in a byte.
 *   - Cellular automata in general (Conway's Life as the cliché).
 *   - Hexagonal grid offset coordinates (T2 explains the convention
 *     used here; you don't need to know it ahead of time).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Navier-Stokes derivation.  We just OBSERVE that the macroscopic
 *     flow looks like fluid — the H-theorem and Chapman-Enskog
 *     expansion that prove it (T8) are sketched, not derived.
 *   - Statistical mechanics formalism.  Local conservation laws are
 *     enough.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm     : Frisch-Hasslacher-Pomeau (FHP-I) Lattice Gas
 *                 Cellular Automaton.  Each cell of a HEXAGONAL grid
 *                 holds 6 BITS — one per direction (E, NE, NW, W, SW,
 *                 SE).  At most one particle per direction per cell.
 *
 *                 Each tick has TWO PHASES:
 *
 *                   1. COLLISION (local).  For each cell, look up
 *                      the 6-bit state in a 64-entry table and
 *                      replace it with the post-collision state.
 *                      The table conserves mass (popcount) and
 *                      momentum (vector sum).  Only 5 of the 64
 *                      patterns trigger a non-trivial scatter; the
 *                      other 59 are "identity" (pass through).
 *
 *                   2. STREAMING (transport).  Every particle hops
 *                      one cell in its direction.  Walls reflect
 *                      particles back the way they came (bounce-
 *                      back boundary, gives no-slip walls).
 *
 *                 The same lookup table is used at every cell every
 *                 tick.  No PDEs, no floats in the hot loop.  Yet
 *                 the AVERAGE behaviour is incompressible Navier-
 *                 Stokes — which is the whole point.
 *
 * Per tick:     INJECT INLET   — add east-going particles to the
 *                                 left edge with probability p (this
 *                                 is the "wind" driving the flow).
 *               COLLIDE        — table lookup per cell.
 *               STREAM         — bit-shift particle to neighbour cell.
 *
 * Visualisation: Cells are too NOISY to draw raw — at best 6 bits per
 *                cell, often 0 or 1 particle.  We average over a 3×3
 *                neighbourhood:
 *                   density ρ   ← popcount sum ÷ 9 (≈ 0..6)
 *                   momentum mx ← signed bit-direction sum ÷ 9
 *                The DENSITY picks a glyph from a sparse-to-dense
 *                ramp: '. , o O 0 @'.  The MOMENTUM picks a colour
 *                pair from a 9-step ramp (deep blue = strong west,
 *                grey = still, deep red = strong east).
 *
 * Performance   : Each cell is ONE BYTE.  Collision = one table
 *                 lookup.  Streaming = bit shifts.  Whole thing
 *                 fits in L1 cache for grids up to ~256² ≈ 64KB.
 *                 Easy 60 fps on multi-million-cell grids.
 *
 * References
 * ──────────
 *   ── Lattice-gas / FHP foundations ─────────────────────────────────
 *   [1] Hardy, J., Pomeau, Y. & de Pazzis, O. (1973), "Time evolution
 *       of a two-dimensional model system. I. Invariant states and
 *       time correlation functions", J. Math. Phys. 14, pp. 1746-1759
 *       — the HPP precursor (square lattice; broken Galilean
 *       invariance).  Shows why FHP needs HEXAGONAL geometry.
 *   [2] Frisch, U., Hasslacher, B. & Pomeau, Y. (1986), "Lattice-gas
 *       automata for the Navier-Stokes equation", Phys. Rev. Lett.
 *       56(14), pp. 1505-1508 — THE foundational FHP paper.  Proof
 *       that a 6-velocity hexagonal lattice recovers isotropic
 *       Navier-Stokes in the continuum limit.
 *   [3] d'Humières, D., Lallemand, P. & Frisch, U. (1986), "Lattice
 *       gas models for 3D hydrodynamics", Europhys. Lett. 2, pp.
 *       291-297 — the FCHC extension to 3D.
 *
 *   ── Textbook / pedagogy ───────────────────────────────────────────
 *   [4] Wolf-Gladrow, D. A. (2000), "Lattice-Gas Cellular Automata and
 *       Lattice Boltzmann Models", Springer LNM 1725 — standard
 *       textbook covering both the original FHP and its successor
 *       (lattice Boltzmann, which replaced LGCA in modern practice).
 *   [5] Toffoli, T. & Margolus, N. (1987), "Cellular Automata Machines",
 *       MIT Press — the broader CA-as-physics worldview.  Ch. 9 covers
 *       LGCA explicitly.
 *
 *   ── Collision tables ──────────────────────────────────────────────
 *   [6] Hénon, M. (1987), "Viscosity of a lattice gas", Complex Systems
 *       1, pp. 763-789 — derives effective viscosity from the
 *       collision table; useful when tuning the §collision pass.
 *
 *   ── Visualisation / rendering ─────────────────────────────────────
 *   [7] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the velocity-magnitude glyph ramp.
 *   [8] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; init_pair,
 *       use_default_colors, newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ────────────────────────────────────────
 *   [9] https://en.wikipedia.org/wiki/FHP_model
 *  [10] https://en.wikipedia.org/wiki/Lattice_gas_automaton
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A FLUID is a HUGE NUMBER of tiny particles bumping into each other.
 * If you keep mass + momentum conserved at every collision, and let
 * everything flow on a regular grid, the AVERAGE BEHAVIOUR over many
 * particles and many ticks IS A FLUID.  No PDE solver, no continuous
 * variables — just a 6-bit cell, one lookup table, one shift.  This
 * is one of the most surprising results in 1980s computational
 * physics: complex emergent macroscopic behaviour from THE SIMPLEST
 * possible local rule.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a HONEYCOMB grid.  At each hexagonal cell, draw arrows
 * pointing OUT toward the 6 neighbours.  At every tick:
 *
 *   - Look at the cell's set of arrows.
 *   - If they make a "head-on collision pattern" (2 arrows opposite,
 *     no others), rotate them ±60° — the collision scatters the pair.
 *   - If they make a 3-arrow symmetric pattern, rotate by 60° — same
 *     reason.
 *   - Otherwise leave them alone.
 *   - Then SLIDE every arrow one cell forward in its direction.
 *
 * That's the whole simulator.  Repeat a million times and you'll see
 * vortices behind a cylinder, jets through gaps in a wall, parabolic
 * channel flow.  All emergent, no fluid equations involved.
 *
 *      ┌─────────────────────────────────────────────────────────────┐
 *      │                                                             │
 *      │   Hex cell with 6 direction bits:                           │
 *      │                                                             │
 *      │            NW          NE                                   │
 *      │              ╲        ╱                                     │
 *      │               ╲      ╱                                      │
 *      │       W ─── [bits]──── E                                    │
 *      │               ╱      ╲                                      │
 *      │              ╱        ╲                                     │
 *      │            SW          SE                                   │
 *      │                                                             │
 *      │   bit 0 = E, bit 1 = NE, bit 2 = NW,                        │
 *      │   bit 3 = W, bit 4 = SW, bit 5 = SE                         │
 *      │                                                             │
 *      │   Opposite of bit d  =  bit (d + 3) mod 6                   │
 *      │                                                             │
 *      └─────────────────────────────────────────────────────────────┘
 *
 * KEY FORMULAS
 * ────────────
 *
 *   CELL STATE (one byte, 6 bits used):
 *     state = bit_0 | bit_1 | bit_2 | bit_3 | bit_4 | bit_5
 *           = E    | NE   | NW   | W    | SW   | SE
 *
 *   PARTICLE COUNT in cell ("local density"):
 *     n(state) = popcount(state)        [0..6]
 *
 *   HORIZONTAL MOMENTUM × 2 (integer; double the actual mx so we can
 *   stay in integers — the unit-vector x-components are ±½ for the
 *   four diagonal bits):
 *     mx2(state) =  2·E + NE − NW − 2·W − SW + SE
 *
 *   COLLISION (table lookup):
 *     parity = (row + col) & 1
 *     state' = g_collisions.lookup[parity][state]
 *     The table is precomputed once at startup; 64 entries, only 5
 *     non-identity:
 *
 *         0x09 (E + W)        → 0x12 or 0x24    head-on E-W pair
 *         0x12 (NE + SW)      → 0x24 or 0x09    head-on NE-SW pair
 *         0x24 (NW + SE)      → 0x09 or 0x12    head-on NW-SE pair
 *         0x15 (E + NW + SW)  → 0x2A             3-particle Y
 *         0x2A (NE + W + SE)  → 0x15             3-particle Y rotated
 *
 *   STREAMING (per particle):
 *     for each set bit d in state:
 *       (nr, nc) = (r + Δrow[parity][d], c + Δcol[parity][d])
 *       if wall[nr][nc]:  set bit (d+3 mod 6) in g_scene.stream_buffer[r][c]
 *                         (bounce-back: stay in place, reverse direction)
 *       else:             set bit d in g_scene.stream_buffer[nr][nc]
 *
 *   3×3 NEIGHBOURHOOD AVERAGE (visualisation):
 *     ρ̄  = (Σ popcount over 9 cells) / 9_excluding_walls
 *     m̄x = (Σ mx2 over 9 cells) / 9_excluding_walls
 *
 *   MOMENTUM → COLOUR (9 bins):
 *     idx = clamp(round((m̄x + 2) / 4 · 8), 0, 8)
 *
 *   DENSITY → GLYPH (7 bins):
 *     glyph = " .,oO0@"[round(ρ̄)]
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - PARITY MATTERS for hex offset coordinates.  The Δrow/Δcol
 *     tables have TWO ROWS, indexed by row & 1.  Forgetting to use
 *     parity gives anisotropic flow.
 *   - GLOBAL CHIRALITY: the FHP-I head-on collision is AMBIGUOUS
 *     (two valid post-states: +60° or -60°).  Always picking the
 *     same one introduces a global rotation bias.  Solution:
 *     alternate by parity — each cell randomises CW/CCW
 *     deterministically by its checkerboard colour.
 *   - WALL BOUNCE-BACK: when a particle would land in a wall, it
 *     stays at the source cell with its direction REVERSED.  This
 *     gives no-slip boundary — the velocity AT the wall is zero
 *     after averaging.  If you don't reverse the direction, you
 *     get free-slip walls instead.
 *   - INLET TIMING: we inject AT THE START of every tick, before
 *     collision.  If you inject AFTER streaming, you'd need a
 *     guard cycle to avoid double-counting.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Preset 1 (cylinder): a BLUE WAKE forms behind the obstacle.
 *     If the inlet probability is high enough, the wake develops
 *     a Kármán vortex street (alternating eddies).
 *   - Preset 5 (channel): the velocity profile across the channel
 *     is PARABOLIC (Poiseuille flow) — fastest in the centre, zero
 *     at the walls.  Visible as red gradient.
 *   - Preset 6 (free): a random initial gas relaxes to a UNIFORM
 *     density grey field — H-theorem in action.
 *   - Toggle inlet OFF and watch the flow gradually equilibrate to
 *     stationary grey.  No spurious global rotation should appear.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 *   T1  CA fluids — emergence from local rules
 *   T2  Hexagonal lattice — why hex and not square
 *   T3  Bit-packed cells — six directions in one byte
 *   T4  Collision table — FHP-I rules step by step
 *   T5  Streaming + bounce-back walls
 *   T6  Spatial averaging — micro is noisy, macro is smooth
 *   T7  Reading the visualisation — density to glyph, momentum to colour
 *   T8  H-theorem — why this gives Navier-Stokes
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  CA FLUIDS — EMERGENCE FROM LOCAL RULES
 * ──────────────────────────────────────────
 * In 1986 Frisch, Hasslacher, and Pomeau proved that a tiny
 * cellular-automaton model on a hexagonal grid produces THE
 * NAVIER-STOKES EQUATIONS as its macroscopic limit.  This was
 * astonishing.  No floating-point, no PDEs, no integration
 * scheme — just bits, bit-shifts, and a 64-entry lookup table.
 * Yet zoom out far enough (in space and time) and you see the
 * exact same equations that govern weather, oceans, blood, and
 * smoke.
 *
 * The recipe has three parts:
 *
 *   1. A CRYSTAL LATTICE.  Square doesn't work (T2 explains why).
 *      Hexagonal does.  Each cell has 6 neighbours.
 *
 *   2. A CELL STATE that lists which DIRECTIONS particles are
 *      going in (T3).  At most ONE particle per direction per cell.
 *
 *   3. A LOCAL UPDATE RULE — collide, then stream — that conserves:
 *        - MASS         (number of particles)
 *        - MOMENTUM     (sum of direction unit vectors)
 *        - ENERGY       (in FHP-I, all particles same speed → mass
 *                        conservation implies energy conservation)
 *      And nothing else.  No special "fluid forces" added.
 *
 * Run this for ~1000 ticks on a 100×100 grid.  Average each 5×5
 * patch over 100 ticks.  You'll see vortices, jets, boundary
 * layers, eddies — all the qualitative behaviour of real fluids.
 *
 * It's the fluid version of "ANT BEHAVIOUR FROM ANT NEIGHBOUR
 * RULES" — emergence is the right word.
 *
 * Why does it work?  Because the conservation laws are EXACTLY
 * the same locally as in real fluids.  The Chapman-Enskog
 * expansion (T8) shows that the LARGE-SCALE LIMIT of any
 * mass+momentum+energy-conserving local rule on an isotropic
 * lattice IS Navier-Stokes (with some specific viscosity).
 *
 * Lattice gas was largely superseded in the 1990s by LATTICE
 * BOLTZMANN methods (LBM), which use fractional populations
 * instead of bits — smoother, less noise, more knobs.  But the
 * BIT version is far more elegant and educationally valuable:
 * it shows that the hard part isn't FLOATS, it's GETTING THE
 * SYMMETRIES RIGHT.
 *
 * T2  HEXAGONAL LATTICE — WHY HEX AND NOT SQUARE
 * ──────────────────────────────────────────────
 * The first lattice-gas model (HPP, 1973) used a SQUARE grid
 * with 4 directions per cell.  It failed at fluid behaviour.
 * The reason: square grids are NOT ISOTROPIC.
 *
 * In Navier-Stokes, the second-rank stress tensor must be
 * INVARIANT under rotation.  In tensor algebra, this requires
 * the lattice's velocity vectors to satisfy:
 *
 *     Σᵢ cᵢα cᵢβ cᵢγ cᵢδ  ∝  δαβ δγδ + δαγ δβδ + δαδ δβγ
 *
 * (i.e. the 4-vector sum is a fully symmetric isotropic tensor).
 * SQUARE has 4 directions: this 4-vector sum is NOT isotropic —
 * it depends on the orientation of the square.  Result: HPP
 * fluid has DIFFERENT viscosity along x vs along the diagonal.
 * Not a fluid; a square-grid artefact.
 *
 * HEX has 6 directions.  The same 4-vector sum IS isotropic.
 * Result: FHP fluid is rotationally symmetric.  The macroscopic
 * limit really is Navier-Stokes, no anisotropy.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   SQUARE (HPP, FAILS):     HEX (FHP-I, WORKS):   │
 *      │                                                  │
 *      │       ↑                       ↗     ↖             │
 *      │   ←   ●   →                  ●  ←     →           │
 *      │       ↓                       ↘     ↙             │
 *      │                                                  │
 *      │   4 directions             6 directions          │
 *      │   anisotropic              isotropic              │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Three-dimensional fluid is harder still: 6 directions in 3D
 * isn't enough.  d'Humières, Lallemand & Frisch (1986) showed
 * you need 24 directions, on a face-centred hypercubic (FCHC)
 * lattice projected to 3D.  Modern LBM uses D2Q9 (2D, 9
 * velocities including a rest particle) or D3Q19, etc.
 *
 * Symmetry beats arithmetic.
 *
 * T3  BIT-PACKED CELLS — SIX DIRECTIONS IN ONE BYTE
 * ─────────────────────────────────────────────────
 * Each cell stores SIX BOOLEAN BITS in a uint8_t:
 *
 *     bit 0  — particle moving E   (east)
 *     bit 1  — particle moving NE  (60° above east)
 *     bit 2  — particle moving NW  (120° above east)
 *     bit 3  — particle moving W   (west)
 *     bit 4  — particle moving SW  (240° above east)
 *     bit 5  — particle moving SE  (300° above east)
 *
 * "Particle moving X" means: at the next streaming step, this
 * bit will hop into the X neighbour cell and become bit X there.
 *
 * EXCLUSION PRINCIPLE: at most ONE particle per direction per
 * cell.  This is what makes the model finite-state — there are
 * exactly 64 possible cell states (2⁶), so the collision lookup
 * table fits in 64 bytes.
 *
 * Why not allow MULTIPLE particles per direction?  Because the
 * collision rules become combinatorially explosive, and you
 * lose the simple table lookup.  (Lattice Boltzmann gives up
 * the bits and uses real-valued populations instead — that's
 * the key generalization.)
 *
 * COMPACTNESS: 100×100 grid = 10000 bytes = 10 KB.  Fits in L1
 * cache.  All hot-loop arithmetic is bitwise, integer, branch-
 * predictor-friendly.  This is what made FHP fast on 1980s
 * hardware.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  Cell state byte:                                │
 *      │    7 6 5 4 3 2 1 0                               │
 *      │    │ │ │ │ │ │ │ └── E   particle going east      │
 *      │    │ │ │ │ │ │ └──── NE  particle going NE        │
 *      │    │ │ │ │ │ └────── NW                           │
 *      │    │ │ │ │ └──────── W                            │
 *      │    │ │ │ └────────── SW                           │
 *      │    │ │ └──────────── SE                           │
 *      │    └─┴────────────── unused (always 0)            │
 *      └──────────────────────────────────────────────────┘
 *
 * Helpful identities:
 *
 *   "is there a particle going west?"     →    state & (1 << 3)
 *   "add a particle going east"           →    state |= (1 << 0)
 *   "remove the SE particle"              →    state &= ~(1 << 5)
 *   "all directions"                      →    0x3F (= 63)
 *   "popcount = local particle count"     →    __builtin_popcount(state)
 *
 * T4  COLLISION TABLE — FHP-I RULES STEP BY STEP
 * ──────────────────────────────────────────────
 * "Collision" means: given a cell state, what's the post-collision
 * state, BEFORE streaming?  Most of the 64 states are "leave alone"
 * (no collision happens).  Only FIVE states have a non-trivial rule.
 *
 * RULE #1 — Head-on E-W pair (state = 0x09 = 0b001001):
 *
 *     Two particles, opposite directions, no others.  Conserves mass
 *     and momentum (both vanish).  Collide → scatter ±60° from the
 *     E-W axis.  TWO valid outcomes:
 *
 *         NE+SW  pair  (state 0x12 = 0b010010)        +60° rotation
 *         NW+SE  pair  (state 0x24 = 0b100100)        -60° rotation
 *
 *     Always picking +60° introduces a GLOBAL CHIRALITY — the fluid
 *     would slowly rotate counter-clockwise even with no input.
 *     FHP-I removes this by ALTERNATING by parity:
 *
 *         parity 0:  0x09 → 0x12     (CCW, +60°)
 *         parity 1:  0x09 → 0x24     (CW,  -60°)
 *
 * RULE #2 — Head-on NE-SW (0x12) and NW-SE (0x24):
 *
 *     Same logic as Rule #1, just rotated.  Each parity rotates
 *     consistently in its own chirality, so no global bias.  The
 *     full cycles are:
 *
 *         parity 0 (CCW):  0x09 → 0x12 → 0x24 → 0x09 ...
 *         parity 1 (CW):   0x09 → 0x24 → 0x12 → 0x09 ...
 *
 * RULE #3 — Three-particle "Y" (state = 0x15 = 0b010101):
 *
 *     Three particles at 120° spacing (E + NW + SW).  Mass = 3,
 *     momentum = 0.  Only ONE other state has the same totals: the
 *     opposite Y (NE + W + SE = 0x2A).  Both parities rotate by
 *     60°, no chirality issue:
 *
 *         0x15 ↔ 0x2A    (both parities)
 *
 * The other 59 states either have unique mass+momentum totals
 * (so no scatter is possible — would violate conservation) or
 * have only ONE valid outcome that equals the input (trivial).
 *
 * The g_collisions.lookup[2][64] is built ONCE at startup with all
 * 64 entries initialised to identity (table[parity][s] = s),
 * then the 5 non-trivial entries are overwritten.  At runtime
 * collision is a SINGLE LOOKUP per cell:
 *
 *     state' = g_collisions.lookup[(r + c) & 1][state]
 *
 * T5  STREAMING + BOUNCE-BACK WALLS
 * ─────────────────────────────────
 * After collision, every set bit must HOP one cell in its
 * direction.  The hop is the same for every cell:
 *
 *     for each set bit d in state[r][c]:
 *       new_state[r + Δrow[parity][d]][c + Δcol[parity][d]] |= (1 << d)
 *
 * Because we read AND write the grid, we use a DOUBLE BUFFER:
 *   - Read from g_scene.particle_grid (current state, just collided)
 *   - Write into g_scene.stream_buffer (next state, all zeros initially)
 *   - At end, swap pointers (or memcpy if fixed buffers)
 *
 * Otherwise particles would step twice in one tick.
 *
 * WALL BOUNCE-BACK
 * ────────────────
 * When a particle would hop into a wall cell, instead it STAYS
 * in its source cell with its direction REVERSED:
 *
 *     for each set bit d in state[r][c]:
 *       (nr, nc) = neighbour
 *       if wall[nr][nc]:
 *         g_scene.stream_buffer[r][c] |= (1 << ((d + 3) % 6))   ← reversed
 *       else:
 *         g_scene.stream_buffer[nr][nc] |= (1 << d)             ← normal hop
 *
 * Why "stay in source"?  Real-fluid no-slip walls have ZERO
 * velocity at the wall.  After many bounce-backs, the average
 * velocity of cells touching the wall is zero (incoming particles
 * are reflected back with opposite direction).  Conservation
 * is local: the bit count is unchanged, momentum is reversed at
 * the wall (pressure transmitted to wall, like a real boundary).
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  Particle hits a wall:                           │
 *      │                                                  │
 *      │   Before:                After:                  │
 *      │     [E→][WALL]            [←W][WALL]             │
 *      │                                                  │
 *      │  bit 0 (E) of source cell                        │
 *      │  becomes                                         │
 *      │  bit 3 (W) of source cell                        │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Variants exist (specular reflection, free-slip), but bounce-back
 * is the simplest and most physically correct for solid walls.
 *
 * T6  SPATIAL AVERAGING — MICRO IS NOISY, MACRO IS SMOOTH
 * ───────────────────────────────────────────────────────
 * If you draw the raw g_scene.particle_grid, you get TV STATIC.  Each
 * cell has ~3 particles on average; popcount gives noisy 0..6
 * values; momentum jumps wildly between adjacent cells.  No
 * wakes, no jets visible.  Just noise.
 *
 * The solution: AVERAGE over a 3×3 neighbourhood.  For each
 * draw cell (r, c):
 *
 *     ρ̄(r, c)  = (Σ over 3×3 of popcount(s)) / count_excluding_walls
 *     m̄x(r, c) = (Σ over 3×3 of mx2(s))      / count_excluding_walls
 *
 * The 9-cell window smooths out single-particle fluctuations.
 * What's left after averaging IS the macroscopic flow field —
 * the same field you'd compute from Navier-Stokes.
 *
 * Why 3×3 and not 5×5 or 9×9?  Bigger windows = smoother but
 * blurrier.  3×3 is the smallest that visibly removes noise
 * while preserving fine structure (jets, boundary layers).  At
 * a typical terminal of 200×60 cells, 3×3 averaging gives ~70
 * "macroscopic patches" across — enough resolution for jet
 * fringes but not so much that the noise comes back.
 *
 * In practice, REAL LATTICE-GAS RESEARCH used much larger
 * averaging windows (in time AND space) to get clean
 * measurements.  We sacrifice quantitative accuracy for visual
 * immediacy.
 *
 * T7  READING THE VISUALISATION — DENSITY TO GLYPH, MOMENTUM TO COLOUR
 * ────────────────────────────────────────────────────────────────────
 * The averaged ρ̄ and m̄x feed two independent visual channels:
 *
 *   ρ̄ → GLYPH (sparse to dense):
 *     " .,oO0@" indexed by round(ρ̄) ∈ [0, 6]
 *     Empty cells stay blank; dense cells get '@'.  This shows
 *     where the FLUID IS — gaps, voids, dense waves.
 *
 *   m̄x → COLOUR (deep blue to deep red):
 *     normalise to [-2, +2], scale to 9 buckets.  Theme palettes
 *     map these 9 buckets to 9 ncurses pairs:
 *       bucket 0 (most westward)  : strongest "left" colour
 *       bucket 4 (still)           : neutral (grey / pale)
 *       bucket 8 (most eastward)   : strongest "right" colour
 *     This shows where the FLUID IS GOING — wakes are blue (left
 *     of inlet direction = backflow), jets are red.
 *
 * Result: you see DENSITY AS BRIGHTNESS-LIKE and DIRECTION AS
 * COLOUR.  Both channels are independent — a dense wake (lots
 * of '@') can be blue (going backward) at the same time.
 *
 * Five THEMES re-shade the colour axis but always preserve the
 * "left = cool, still = neutral, right = warm" semantics:
 *
 *   Classic — blue/cyan/grey/orange/red    (canonical)
 *   Ocean   — navy → cyan → white          (surface waves)
 *   Plasma  — purple → magenta → gold      (high-energy feel)
 *   Matrix  — dark green → bright lime     (cyber-aesthetic)
 *   Heat    — dark red → orange → yellow   (warm only)
 *
 * The glyph ramp doesn't change with theme — it's universal.
 *
 * T8  H-THEOREM — WHY THIS GIVES NAVIER-STOKES
 * ────────────────────────────────────────────
 * The H-THEOREM (Boltzmann, 1872) says that any system with
 * conservative collisions evolves toward MAXIMUM ENTROPY (the
 * Maxwell-Boltzmann equilibrium distribution).
 *
 * For FHP-I:
 *   - Collisions conserve mass and momentum locally.
 *   - The lattice is rotationally isotropic at 4-vector level (T2).
 *   - The Chapman-Enskog expansion (a series in mean-free-path /
 *     macroscopic length) gives:
 *
 *         ∂ρ/∂t  + ∇·(ρu)        = 0                ← continuity
 *         ∂(ρu)/∂t + ∇·(ρuu)    = -∇p + ν∇²(ρu)   ← Navier-Stokes
 *
 *     where ν is a CALCULABLE function of the lattice rules.
 *
 * Concretely: at FHP-I parameters, ν ≈ 0.166 lattice units²/tick.
 * Reynolds number Re = U·L/ν is whatever your simulation produces;
 * for typical inlet velocity 0.1 cells/tick on a width L = 100,
 * Re ≈ 60 — laminar but interesting.
 *
 * What does this mean PRACTICALLY?  It means that if you turn off
 * gravity, the fluid SETTLES TO uniform density with zero net
 * velocity (preset 6 demonstrates this).  And at any instant, the
 * AVERAGED flow field obeys real fluid dynamics — vortices behind
 * cylinders, parabolic profiles in channels, exactly what
 * experimentalists measure with real water.
 *
 * The H-theorem PROOFS work for any local conservative rule on
 * an isotropic lattice — FHP-I just happens to be the SIMPLEST
 * 2D example.  More complex models (FHP-III, lattice Boltzmann)
 * give more accurate viscosity values, more exotic equations of
 * state, multiphase flow, magnetohydrodynamics.  The principle
 * stays the same: GET THE SYMMETRIES RIGHT, GET THE PHYSICS FREE.
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
/* §1  config — every constant in one place                              */
/* ===================================================================== */
/*
 * No magic numbers below this section.  Every literal in code is a
 * named constant from §1.
 */

/* ── GRID SIZE LIMITS ── */
#define GRID_ROWS_MAX 128
#define GRID_COLS_MAX 512

/* ── PHYSICS TIMING ── */

/* Physics ticks per render frame (time-lapse multiplier). */
#define STEPS_PER_FRAME_DEFAULT 8
#define STEPS_PER_FRAME_MIN 1
#define STEPS_PER_FRAME_MAX 32

/* Inlet (left edge) injection rate. */
#define INLET_PROB_DEFAULT 0.55f
#define INLET_PROB_MIN 0.10f
#define INLET_PROB_MAX 0.95f
#define INLET_PROB_STEP 0.05f

/* Simulation tick rate (Hz). */
#define SIM_HZ_DEFAULT 20
#define SIM_HZ_MIN 5
#define SIM_HZ_MAX 60
#define SIM_HZ_STEP 5

/* Render frame rate cap. */
#define RENDER_FPS_CAP 60

/* Aspect-ratio compensation for circular obstacles in 2:1-tall
 * terminal cells.  Multiply Δrow by this when computing distances
 * so a "circle of N cells wide" stays visually round. */
#define ASPECT_FACTOR_CELL_TO_VISUAL 2.0f

/* ── COLLISION-TABLE CONSTANTS ── */

/* The 5 non-identity FHP-I collision states (T4): */
#define COLLISION_HEADON_EW 0x09    /* E + W                 */
#define COLLISION_HEADON_NE_SW 0x12 /* NE + SW               */
#define COLLISION_HEADON_NW_SE 0x24 /* NW + SE               */
#define COLLISION_THREEY_EVEN 0x15  /* E + NW + SW           */
#define COLLISION_THREEY_ODD 0x2A   /* NE + W + SE           */

#define COLLISION_TABLE_SIZE 64 /* 2^6 — six bits per cell */
#define HEX_DIRECTIONS 6
#define HEX_DIRECTION_BITS_MASK 0x3F /* lowest 6 bits          */

/* Direction bit indices.  Order matters — collision-table magic
 * numbers above assume this exact layout. */
enum {
  DIR_E = 0,  /* east       */
  DIR_NE = 1, /* north-east */
  DIR_NW = 2, /* north-west */
  DIR_W = 3,  /* west       */
  DIR_SW = 4, /* south-west */
  DIR_SE = 5, /* south-east */
};

/* ── VISUALISATION ── */

/* 9 momentum buckets → 9 colour pairs (deep west … grey … deep east). */
#define MOMENTUM_BUCKET_COUNT 9

/* Density-to-glyph ramp.  [0] = blank, [1..6] = sparse..dense. */
#define DENSITY_GLYPH_COUNT 7

/* 3×3 neighbourhood for spatial averaging. */
#define AVG_WINDOW_HALF 1

/* Density threshold below which a draw cell is left blank. */
#define DENSITY_BLANK_THRESHOLD 0.25f

/* Density threshold for A_BOLD on the glyph (denser = brighter). */
#define DENSITY_BOLD_THRESHOLD 3.5f

/* ── COLOUR PAIRS ── */
enum {
  PAIR_MOMENTUM_FIRST = 1, /* +0..+8 */
  PAIR_WALL = 1 + MOMENTUM_BUCKET_COUNT,
  PAIR_HUD,
  PAIR_HINT,
};

/* ── PRESETS / THEMES ── */
#define PRESET_COUNT 6
#define THEME_COUNT 5

/* ── HUD reserved rows ── */
#define HUD_RESERVED_ROWS 2

/* ── TIME UNITS ── */
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
/* §3  rng — xorshift32 (no global lock)                                 */
/* ===================================================================== */
/*
 * A 32-bit xorshift PRNG.  Period 2³² - 1, fast, no
 * thread-safety guarantees (we don't need any — single thread).
 * Used only by inlet injection and "free" preset's initial gas;
 * the physics is deterministic given an RNG seed.
 */

static uint32_t xorshift_state = 12345u;

static inline uint32_t xorshift_next(void) {
  uint32_t s = xorshift_state;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  xorshift_state = s;
  return s;
}

/* Random float in [0, 1).  24-bit precision. */
static inline float xorshift_unit_float(void) {
  return (float)(xorshift_next() >> 8) / (float)(1u << 24);
}

/* ===================================================================== */
/* §4  hex_directions — 6-direction unit-vector tables                   */
/* ===================================================================== */
/*
 * Hex coordinates use OFFSET COORDINATES with row-parity (T2).
 * Each row's neighbours are at slightly different (Δrow, Δcol) than
 * the next row.  This keeps things in a rectangular array indexed by
 * (row, col), at the cost of needing two tables.
 *
 * EVEN ROW (parity 0) layout:
 *
 *       NW    NE
 *         \  /
 *      W -- ● -- E         dr  dc
 *         /  \             ─── ───
 *       SW    SE      E    0   +1
 *                     NE  -1   0
 *                     NW  -1  -1
 *                     W    0   -1
 *                     SW  +1   -1
 *                     SE  +1   0
 *
 * ODD ROW (parity 1) layout — diagonals shifted by +1 col:
 *
 *                            dr  dc
 *                            ─── ───
 *                     E    0   +1
 *                     NE  -1   +1
 *                     NW  -1   0
 *                     W    0   -1
 *                     SW  +1   0
 *                     SE  +1   +1
 *
 * Direction bit indices:
 *   E=0  NE=1  NW=2  W=3  SW=4  SE=5
 * Opposite of d:  (d + 3) mod 6
 */

static const int hex_direction_drow[2][HEX_DIRECTIONS] = {
    {0, -1, -1, 0, +1, +1}, /* even row */
    {0, -1, -1, 0, +1, +1}, /* odd  row */
};

static const int hex_direction_dcol[2][HEX_DIRECTIONS] = {
    {+1, 0, -1, -1, -1, 0}, /* even row */
    {+1, +1, 0, -1, 0, +1}, /* odd  row */
};

/* Opposite-direction lookup: bounce_back[d] = (d + 3) % 6. */
static const int direction_bounce_back[HEX_DIRECTIONS] = {
    DIR_W,  /* opposite of E  */
    DIR_SW, /* opposite of NE */
    DIR_SE, /* opposite of NW */
    DIR_E,  /* opposite of W  */
    DIR_NE, /* opposite of SW */
    DIR_NW, /* opposite of SE */
};

/* ===================================================================== */
/* §5  collision rules — FHP-I lookup table, precomputed at startup      */
/* ===================================================================== */
/*
 * CollisionRules — the precomputed FHP-I collision lookup.
 *
 * Intent
 *   The FHP-I collision rules transform a 6-bit cell state (one bit
 *   per hex direction, true = particle present) into a post-collision
 *   6-bit state.  In principle this is a FUNCTION; in practice we
 *   precompute the full 64-entry lookup at startup and read it as an
 *   ARRAY in the inner-loop collision pass.  Trade: 128 bytes of BSS
 *   for an O(1) per-cell collision step.  See [2] Frisch, Hasslacher
 *   & Pomeau 1986 §III for the original rules.
 *
 * Why TWO parities (not just one table)
 *   The 2-particle head-on collision has TWO possible outcomes — both
 *   rotate the input by ±60°.  Picking one direction globally would
 *   inject a uniform CHIRALITY into the flow (everything would rotate
 *   the same way).  Splitting by cell parity — (row + col) & 1 — and
 *   using opposite rotations on the two parities cancels the bias on
 *   average.  See [2] §III B and [4] Wolf-Gladrow §3.5.
 *
 * Layout
 *   lookup[parity][6-bit input] = post-collision 6-bit state.
 *   - Default rows are the IDENTITY (no collision → state passes through).
 *   - Only 5 entries per parity get overwritten with real rules:
 *       3 head-on rotations + 2 three-particle Y exchanges.
 *
 * Why a struct (not a bare 2-D global)
 *   Wrapping the array in a named type makes its purpose explicit at
 *   use sites: `g_collisions.lookup[parity][s]` reads as "look up the
 *   FHP collision rule" — a bare `collision_table[parity][s]` says
 *   nothing about what kind of table it is.  Same memory footprint,
 *   real documentation value.
 *
 * Built ONCE in main() via collision_table_build() before sim starts.
 */
typedef struct {
    /* [parity][6-bit input] → post-collision 6-bit state.  parity is
     * (row + col) & 1; the 6 bits index DIR_E .. DIR_NE in the canonical
     * hex order from §4. */
    uint8_t lookup[2][COLLISION_TABLE_SIZE];
} CollisionRules;

static CollisionRules g_collisions;

static void collision_table_build(void) {
    /* Default: identity (no collision; pass-through). */
    for (int s = 0; s < COLLISION_TABLE_SIZE; s++) {
        g_collisions.lookup[0][s] = (uint8_t)s;
        g_collisions.lookup[1][s] = (uint8_t)s;
    }

    /* Parity 0 — CCW (+60°) head-on cycle. */
    g_collisions.lookup[0][COLLISION_HEADON_EW]    = COLLISION_HEADON_NE_SW;
    g_collisions.lookup[0][COLLISION_HEADON_NE_SW] = COLLISION_HEADON_NW_SE;
    g_collisions.lookup[0][COLLISION_HEADON_NW_SE] = COLLISION_HEADON_EW;

    /* Parity 1 — CW (-60°) head-on cycle. */
    g_collisions.lookup[1][COLLISION_HEADON_EW]    = COLLISION_HEADON_NW_SE;
    g_collisions.lookup[1][COLLISION_HEADON_NE_SW] = COLLISION_HEADON_EW;
    g_collisions.lookup[1][COLLISION_HEADON_NW_SE] = COLLISION_HEADON_NE_SW;

    /* 3-particle Y — symmetric, both parities. */
    g_collisions.lookup[0][COLLISION_THREEY_EVEN] = COLLISION_THREEY_ODD;
    g_collisions.lookup[1][COLLISION_THREEY_EVEN] = COLLISION_THREEY_ODD;
    g_collisions.lookup[0][COLLISION_THREEY_ODD]  = COLLISION_THREEY_EVEN;
    g_collisions.lookup[1][COLLISION_THREEY_ODD]  = COLLISION_THREEY_EVEN;
}

/* ===================================================================== */
/* §6  momentum_extract — bit-popcount density + signed mx               */
/* ===================================================================== */
/*
 * cell_particle_count(state):
 *   How many particles are in this cell.  popcount of bits 0..5.
 *
 * cell_momentum_x_doubled(state):
 *   Sum of horizontal unit-vector components of all set bits, ×2 to
 *   stay in integers.  Hex unit-vector x components:
 *     E=+1   NE=+½   NW=-½   W=-1   SW=-½   SE=+½
 *   ×2 to drop fractions:
 *     E=+2   NE=+1   NW=-1   W=-2   SW=-1   SE=+1
 *
 * The "doubled" momentum is later renormalised in mx_to_color_index.
 */

static inline int cell_particle_count(uint8_t state) {
  return __builtin_popcount((unsigned)state);
}

static inline int cell_momentum_x_doubled(uint8_t state) {
  int e = (state >> DIR_E) & 1;
  int ne = (state >> DIR_NE) & 1;
  int nw = (state >> DIR_NW) & 1;
  int w = (state >> DIR_W) & 1;
  int sw = (state >> DIR_SW) & 1;
  int se = (state >> DIR_SE) & 1;
  return 2 * e + ne - nw - 2 * w - sw + se;
}

/* ===================================================================== */
/* §7  themes — 5 momentum-to-colour palettes                            */
/* ===================================================================== */
/*
 * Each theme has 9 entries:
 *
 *   index 0 — strongest WESTWARD flow (e.g. deep blue)
 *   index 4 — neutral / still         (e.g. grey)
 *   index 8 — strongest EASTWARD flow (e.g. deep red)
 *
 * Two palettes per theme:
 *   palette_256[]: xterm-256 colour codes for 256-colour terminals
 *   palette_8[] : 8-colour fallback
 *
 * All values lie in the BRIGHT half of the 256 cube (≥ 19) or are
 * standard terminal-bright colours so they remain visible on
 * default-black background.
 */

/*
 * ColorTheme — palette bundle for one named look.
 *
 * Intent
 *   Each cell's local momentum is bucketed into MOMENTUM_BUCKET_COUNT
 *   tiers (typically 9) and the bucket index selects a colour pair.
 *   A theme is exactly two parallel palettes — one 256-cube, one
 *   8-colour fallback — plus a short name shown in HUD.
 *
 * Why two parallel palette tracks
 *   palette_256[i] is the index used when COLORS >= 256.  palette_8[i]
 *   is the fallback for low-COLORS terminals; on those we lose tier
 *   resolution but keep sign-meaningful colour.  Picking by terminal
 *   capability at init keeps the renderer hot path identical.
 *
 * Why bucket-by-momentum (and not, e.g., density)
 *   FHP-style LGCA conserves momentum exactly at each cell.  Local
 *   momentum is the natural physical observable to colour by — high
 *   |p| highlights flow fronts and vortex cores.  Density is nearly
 *   uniform after a few collision passes (incompressibility limit)
 *   and would give a flat, low-contrast image.  See [6] Hénon for the
 *   relation between LGCA momentum and macroscopic velocity.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   All palette_256 values are kept ≥ 19 so even the dimmest bucket
 *   stays visible against the default-black background.  Cube 16-23
 *   are technically borderline but the existing themes were tuned by
 *   hand to look right; future themes should target ≥ 24.
 *
 * Reference [8] Raymond's NCURSES HOWTO §6 — init_pair semantics that
 *   turn these palette arrays into live colour pairs.
 */
typedef struct {
    short       palette_256[MOMENTUM_BUCKET_COUNT];  /* 256-colour cube */
    short       palette_8  [MOMENTUM_BUCKET_COUNT];  /* 8-colour fallback */
    const char *name;                                /* HUD label       */
} ColorTheme;

static const ColorTheme color_theme_table[THEME_COUNT] = {
    /* 0 Classic — deep blue → cyan → grey → orange → red */
    {{21, 33, 51, 87, 244, 208, 202, 196, 160},
     {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_RED, COLOR_RED},
     "Classic"},
    /* 1 Ocean — midnight blue → teal → cyan → white */
    {{19, 27, 39, 51, 87, 123, 159, 195, 231},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     "Ocean"},
    /* 2 Plasma — deep purple → magenta → gold */
    {{91, 129, 165, 201, 207, 213, 214, 220, 226},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_RED, COLOR_RED,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     "Plasma"},
    /* 3 Matrix — dark green → bright lime */
    {{28, 34, 40, 46, 82, 118, 154, 190, 226},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE},
     "Matrix"},
    /* 4 Heat — dark red → orange → bright yellow */
    {{52, 88, 124, 166, 172, 178, 214, 220, 226},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     "Heat"},
};

/* ===================================================================== */
/* §8  colors — pair init + theme apply                                  */
/* ===================================================================== */

static bool terminal_has_256_colors = false;

static void colors_apply_theme(int theme_index) {
  const ColorTheme *theme = &color_theme_table[theme_index];
  for (int i = 0; i < MOMENTUM_BUCKET_COUNT; i++) {
    short fg =
        terminal_has_256_colors ? theme->palette_256[i] : theme->palette_8[i];
    init_pair((short)(PAIR_MOMENTUM_FIRST + i), fg, -1);
  }

  if (terminal_has_256_colors) {
    init_pair(PAIR_WALL, 244, -1);
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_WALL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();
  terminal_has_256_colors = (COLORS >= 256);
  colors_apply_theme(theme_index);
}

/* Map 3×3-averaged horizontal momentum (signed, roughly in [-2, +2])
 * to a colour-pair index 0..8.  Linear bin, clamped. */
static inline int momentum_to_color_bucket(float m_avg_doubled) {
  float t = (m_avg_doubled + 2.0f) / 4.0f; /* → [0, 1] */
  if (t < 0.0f)
    t = 0.0f;
  if (t > 1.0f)
    t = 1.0f;
  int idx = (int)(t * 8.49f); /* → [0, 8] */
  if (idx < 0)
    idx = 0;
  if (idx >= MOMENTUM_BUCKET_COUNT)
    idx = MOMENTUM_BUCKET_COUNT - 1;
  return idx;
}

/* Density-to-glyph ramp.  Indexed by round(ρ̄). */
static const char density_glyph_ramp[DENSITY_GLYPH_COUNT] = {' ', '.', ',', 'o',
                                                             'O', '0', '@'};

/* ===================================================================== */
/* §9  scene — the single owner of all simulation + render state         */
/* ===================================================================== */
/*
 * Scene — the single owner of this demo's live state.
 *
 * Intent
 *   Earlier versions of this file kept the scene state as flat
 *   file-scope globals (every physics pass needs all of it, so it
 *   used to be unbundled).  This Scene struct groups the same fields
 *   together so the sim/render/control split is enforced by the
 *   STRUCT LAYOUT, not just by convention.  Helpers continue to
 *   reach the state through `g_scene.<field>` so the inner-loop math
 *   stays free of pointer threading.
 *
 * Locality (sim vs render)
 *   Fields are GROUPED EXPLICITLY so a reader can tell at a glance
 *   which subsystem touches each one:
 *     - collision / stream / inlet passes read it  → simulation
 *     - paint_field / hud_paint_* / theme reads it → rendering
 *     - both sides bound their loops (g_scene.grid_active_rows / _cols)
 *                                                  → shared geometry
 *     - g_scene.simulation_paused gates the tick AND the HUD "PAUSED" tag
 *                                                  → control state
 *
 *   Mis-classifying a field is a real source of bugs: a render-only
 *   value (g_scene.active_theme_index) accidentally read by the tick would
 *   couple physics to a visual choice, defeating the engine-toggle
 *   invariant that "the same scenario with the same Re must evolve
 *   identically regardless of which palette is displayed".
 *
 * Why one big struct (not split across Scene + App + Field)
 *   Every physics pass needs the whole grid (g_scene.particle_grid + stream_
 *   buffer + g_scene.wall_mask + bounds).  Splitting would force pointer
 *   chains in the inner loop — the loop already runs the equivalent
 *   of 13 multiplies per cell, and the additional indirection would
 *   double that.  Keeping one Scene struct + one g_scene instance
 *   matches CLAUDE.md's "no dynamic allocation after init": ~48 KB
 *   in BSS, no malloc anywhere.
 *
 * Why these specific fields and no others
 *   - g_scene.particle_grid       the FHP field, 6 bit-channels per cell — the
 *                          LARGEST piece of state (~48 KB at max size).
 *   - g_scene.stream_buffer       scratch for the streaming pass; double-
 *                          buffered so streaming is single-pass.
 *   - g_scene.wall_mask           per-cell solid-obstacle flag (preset-
 *                          configured; static during simulation).
 *   - g_scene.grid_active_rows / _cols  active subregion bounds; both sim and
 *                          render loops use these as iteration limits.
 *   - g_scene.active_preset_index user's preset choice (Cylinder / 2-Slit / …).
 *   - g_scene.simulation_paused   gate for the physics passes + HUD tag.
 *   - g_scene.physics_steps_per_frame  substeps per render frame (visual speed).
 *   - g_scene.sim_steps_per_second     overall sim tick rate (Hz).
 *   - g_scene.inlet_enabled       whether the left-edge inlet drives flow.
 *   - g_scene.inlet_probability_per_cell  per-cell bit-injection chance.
 *   - g_scene.active_theme_index  pure render — palette selector.  Changing
 *                          it MUST NOT touch any sim field.
 *   - g_scene.physics_tick_count  monotonic counter for HUD display.
 *
 * Things that DO NOT live here
 *   - The 6 FHP collision tables / 10 themes / 5 presets  → file-scope
 *                                                            constants
 *   - Render-frame timing / FPS accumulator               → locals in
 *                                                            main()
 *   - Signal flags (SIGINT, SIGWINCH)                     → file-scope
 *                                                            volatile
 *
 * Reference [8] Raymond NCURSES HOWTO on the scene-paint pipeline
 *   that consumes the render fields below.
 */
typedef struct {
    /* ── Simulation state (read by collision / stream / inlet) ─── */
    uint8_t particle_grid[GRID_ROWS_MAX][GRID_COLS_MAX]; /* FHP bits */
    uint8_t stream_buffer[GRID_ROWS_MAX][GRID_COLS_MAX]; /* scratch  */
    bool    wall_mask    [GRID_ROWS_MAX][GRID_COLS_MAX]; /* solid?   */

    /* ── Shared geometry (sim AND render) ────────────────────────── */
    int grid_active_rows;
    int grid_active_cols;

    /* ── Control state (gates tick / drives HUD) ─────────────────── */
    int  active_preset_index;
    bool simulation_paused;

    /* ── Simulation tuning ──────────────────────────────────────── */
    int   physics_steps_per_frame;     /* substeps per render frame  */
    int   sim_steps_per_second;        /* sim tick rate (Hz)         */
    bool  inlet_enabled;               /* drive flow from left edge? */
    float inlet_probability_per_cell;  /* per-cell bit-injection p   */

    /* ── Pure render state (theme picker; must not touch sim) ───── */
    int active_theme_index;

    /* ── Diagnostics / introspection ────────────────────────────── */
    long physics_tick_count;           /* monotonic step counter     */
} Scene;

static Scene g_scene = {
    .physics_steps_per_frame    = STEPS_PER_FRAME_DEFAULT,
    .sim_steps_per_second       = SIM_HZ_DEFAULT,
    .inlet_enabled              = true,
    .inlet_probability_per_cell = INLET_PROB_DEFAULT,
};

/* ===================================================================== */
/* §10  step_inject_inlet — drive flow from left edge                    */
/* ===================================================================== */
/*
 * For each non-wall cell on column 0, with probability p, set bit 0
 * (= particle moving E).  This is the "wind" that drives the flow.
 *
 * If the inlet bit is already set, "|=" leaves it alone.
 */

static void step_inject_inlet(void) {
  if (!g_scene.inlet_enabled)
    return;
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    if (g_scene.wall_mask[r][0])
      continue;
    if (xorshift_unit_float() < g_scene.inlet_probability_per_cell)
      g_scene.particle_grid[r][0] |= (uint8_t)(1u << DIR_E);
  }
}

/* ===================================================================== */
/* §11  step_collide — collision phase (table lookup)                    */
/* ===================================================================== */
/*
 * For each non-wall cell, replace its 6-bit state with the post-
 * collision state from g_collisions.lookup[parity][state].
 *
 * Parity = (row + col) & 1 — alternating CW/CCW chirality cancels
 * global rotation bias (T4).
 *
 * MASKING with HEX_DIRECTION_BITS_MASK belt-and-braces guard against
 * any spurious upper-bit corruption.
 */
static void step_collide(void) {
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      if (g_scene.wall_mask[r][c])
        continue;
      int parity = (r + c) & 1;
      uint8_t s = g_scene.particle_grid[r][c] & HEX_DIRECTION_BITS_MASK;
      g_scene.particle_grid[r][c] = g_collisions.lookup[parity][s];
    }
  }
}

/* ===================================================================== */
/* §12  step_stream — streaming + bounce-back                            */
/* ===================================================================== */
/*
 * Double-buffered streaming.  Every set bit in g_scene.particle_grid[r][c]
 * hops one cell in its direction.  At wall encounters, the bit
 * STAYS at (r, c) but flipped to the opposite direction (bounce-back).
 *
 * Toroidal wrap: indices outside the grid wrap around (modulo).  This
 * makes left/right edges connect (so flow that exits east re-enters
 * west of column 0).
 */

static void step_stream(void) {
  memset(g_scene.stream_buffer, 0, sizeof g_scene.stream_buffer);

  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    int parity = r & 1;
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      if (g_scene.wall_mask[r][c])
        continue;
      uint8_t state = g_scene.particle_grid[r][c];
      if (state == 0)
        continue;

      for (int d = 0; d < HEX_DIRECTIONS; d++) {
        if (!((state >> d) & 1))
          continue;

        int nr = (r + hex_direction_drow[parity][d] + g_scene.grid_active_rows) %
                 g_scene.grid_active_rows;
        int nc = (c + hex_direction_dcol[parity][d] + g_scene.grid_active_cols) %
                 g_scene.grid_active_cols;

        if (g_scene.wall_mask[nr][nc]) {
          /* Bounce-back: stay at source, flip direction. */
          g_scene.stream_buffer[r][c] |= (uint8_t)(1u << direction_bounce_back[d]);
        } else {
          /* Normal hop: deposit bit d at neighbour cell. */
          g_scene.stream_buffer[nr][nc] |= (uint8_t)(1u << d);
        }
      }
    }
  }

  memcpy(g_scene.particle_grid, g_scene.stream_buffer, sizeof g_scene.particle_grid);
}

/* ===================================================================== */
/* §13  step — full physics tick                                         */
/* ===================================================================== */
/*
 *   1. INJECT INLET — add east-going particles to left edge
 *   2. COLLIDE      — table lookup per cell
 *   3. STREAM       — bit-shift particles to neighbour cells
 *
 * The whole physics in three lines.
 */
static void physics_step(void) {
  step_inject_inlet();
  step_collide();
  step_stream();
  g_scene.physics_tick_count++;
}

/* ===================================================================== */
/* §14  presets — 6 preset definitions                                   */
/* ===================================================================== */
/*
 * Each preset specifies:
 *   inlet            — whether to drive flow from left edge
 *   initial_density  — fraction of bits randomly seeded at start
 *                      (only the "Free" preset uses > 0)
 *   name / desc      — strings shown in the HUD
 */

/*
 * Preset — one named scene configuration the user can flip to.
 *
 * Intent
 *   Each preset is a SCENARIO that demonstrates a different fluid
 *   phenomenon: cylinder wake (von Kármán street), double-slit
 *   diffraction analogue, free decay of a random soup, etc.  A
 *   preset specifies WHAT FLOWS (inlet or none) and WHAT'S THERE TO
 *   FLOW THROUGH (the obstacle pattern, drawn separately by the
 *   preset's apply function).
 *
 * Why two-line description
 *   `name` is the short label in HUD ("Cylinder").  `desc` is the
 *   one-line story shown after preset switch ("Particles hit
 *   cylinder — BLUE wake forms and grows behind").  Keeping them
 *   separate lets the HUD stay compact while still teaching the
 *   user what to look for.
 *
 * Why initial_density is here (not a global)
 *   Most scenarios START FROM REST and rely on the inlet to inject
 *   particles; only "Free" needs a soup of random bits.  Per-preset
 *   field avoids a "if (preset is Free)" check at init time.
 *
 * Reference [2] Frisch-Hasslacher-Pomeau 1986 for the cylinder /
 *   Karmán-street setup; [5] Toffoli & Margolus ch. 9 for the
 *   "lattice gas as cellular-automaton machine" perspective behind
 *   the scenario-as-bit-pattern design.
 */
typedef struct {
    bool        inlet;            /* drive flow from left edge?       */
    float       initial_density;  /* fraction of bits seeded at start *
                                   * (0 except "Free" preset)         */
    const char *name;             /* short HUD label                  */
    const char *desc;             /* one-line "what to watch for"     */
} Preset;

static const Preset preset_table[PRESET_COUNT] = {
    {true, 0.0f, "Cylinder",
     "Particles hit cylinder -- BLUE wake forms and grows behind"},
    {true, 0.0f, "2-Slit",
     "Two gaps in a wall -- jets spread out and meet in the middle"},
    {true, 0.0f, "3-Slit",
     "Three gaps -- three jets spread and interfere downstream"},
    {true, 0.0f, "4-Slit",
     "Four gaps -- tight jets, rich interference pattern"},
    {true, 0.0f, "Channel",
     "Parallel walls top & bottom -- flow fastest at centre (Poiseuille)"},
    {false, 0.40f, "Free",
     "No inlet, no walls -- random gas relaxes toward equilibrium"},
};

/* ===================================================================== */
/* §15  scene_load — apply preset to walls + state                       */
/* ===================================================================== */
/*
 * Wipe the grid, then build the preset-specific obstacle layout and
 * (optionally) seed initial particles.
 */

static void scene_seed_random_particles(float density_per_bit) {
  if (density_per_bit <= 0.0f)
    return;
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      uint8_t bits = 0;
      for (int b = 0; b < HEX_DIRECTIONS; b++)
        if (xorshift_unit_float() < density_per_bit)
          bits |= (uint8_t)(1u << b);
      g_scene.particle_grid[r][c] = bits;
    }
  }
}

static void scene_build_cylinder(void) {
  int centre_col = g_scene.grid_active_cols * 2 / 5;
  int centre_row = g_scene.grid_active_rows / 2;
  int radius = g_scene.grid_active_cols / 12;
  if (radius < 3)
    radius = 3;

  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      float dx = (float)(c - centre_col);
      float dy = (float)(r - centre_row) * ASPECT_FACTOR_CELL_TO_VISUAL;
      if (dx * dx + dy * dy < (float)(radius * radius)) {
        g_scene.wall_mask[r][c] = true;
        g_scene.particle_grid[r][c] = 0;
      }
    }
  }
}

static void scene_build_slit_wall(int slit_count) {
  int wall_col = g_scene.grid_active_cols / 3;
  int gap_height = g_scene.grid_active_rows / (slit_count * 3);
  if (gap_height < 2)
    gap_height = 2;

  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    bool inside_slit = false;
    for (int s = 0; s < slit_count; s++) {
      int slit_centre_row = g_scene.grid_active_rows * (s + 1) / (slit_count + 1);
      if (r >= slit_centre_row - gap_height / 2 &&
          r < slit_centre_row + gap_height / 2) {
        inside_slit = true;
        break;
      }
    }
    if (inside_slit)
      continue;

    /* Make the wall TWO cells thick so particles can't tunnel. */
    for (int dc = 0; dc < 2; dc++) {
      int wc = wall_col + dc;
      if (wc >= g_scene.grid_active_cols)
        break;
      g_scene.wall_mask[r][wc] = true;
      g_scene.particle_grid[r][wc] = 0;
    }
  }
}

static void scene_build_channel(void) {
  int wall_thickness = (g_scene.grid_active_rows > 8) ? 2 : 1;
  for (int c = 0; c < g_scene.grid_active_cols; c++) {
    for (int i = 0; i < wall_thickness; i++) {
      g_scene.wall_mask[i][c] = true;
      g_scene.wall_mask[g_scene.grid_active_rows - 1 - i][c] = true;
      g_scene.particle_grid[i][c] = 0;
      g_scene.particle_grid[g_scene.grid_active_rows - 1 - i][c] = 0;
    }
  }
}

static void scene_load(int preset_index) {
  if (preset_index < 0 || preset_index >= PRESET_COUNT)
    preset_index = 0;
  g_scene.active_preset_index = preset_index;
  g_scene.inlet_enabled = preset_table[preset_index].inlet;
  g_scene.physics_tick_count = 0;

  memset(g_scene.wall_mask, 0, sizeof g_scene.wall_mask);
  memset(g_scene.particle_grid, 0, sizeof g_scene.particle_grid);
  memset(g_scene.stream_buffer, 0, sizeof g_scene.stream_buffer);

  scene_seed_random_particles(preset_table[preset_index].initial_density);

  switch (preset_index) {
  case 0:
    scene_build_cylinder();
    break;
  case 1:
    scene_build_slit_wall(2);
    break;
  case 2:
    scene_build_slit_wall(3);
    break;
  case 3:
    scene_build_slit_wall(4);
    break;
  case 4:
    scene_build_channel();
    break;
  case 5: /* Free — no obstacles */
    break;
  default:
    break;
  }
}

/* ===================================================================== */
/* §16  cell_neighbourhood — 3×3 spatial average                         */
/* ===================================================================== */
/*
 * Average particle-count and momentum-x-doubled over the 3×3 block
 * centred on (r, c), excluding wall cells.  Returns the averaged
 * density and momentum.  Used by the renderer (§17).
 */

static void cell_neighbourhood_average(int r, int c, float *density_out,
                                       float *momentum_x_doubled_out) {
  float density_sum = 0.0f;
  float momentum_sum = 0.0f;
  int counted = 0;

  for (int dr = -AVG_WINDOW_HALF; dr <= AVG_WINDOW_HALF; dr++) {
    for (int dc = -AVG_WINDOW_HALF; dc <= AVG_WINDOW_HALF; dc++) {
      int nr = r + dr;
      int nc = c + dc;
      if (nr < 0 || nr >= g_scene.grid_active_rows)
        continue;
      if (nc < 0 || nc >= g_scene.grid_active_cols)
        continue;
      if (g_scene.wall_mask[nr][nc])
        continue;

      uint8_t s = g_scene.particle_grid[nr][nc];
      density_sum += (float)cell_particle_count(s);
      momentum_sum += (float)cell_momentum_x_doubled(s);
      counted++;
    }
  }

  if (counted > 0) {
    density_sum /= (float)counted;
    momentum_sum /= (float)counted;
  }
  *density_out = density_sum;
  *momentum_x_doubled_out = momentum_sum;
}

/* ===================================================================== */
/* §17  render — fluid + walls + glyph ramp                              */
/* ===================================================================== */
/*
 * For each visible cell:
 *   if WALL              — draw '#' in wall colour
 *   else compute (ρ̄, m̄x) from §16
 *     if ρ̄ < BLANK_THRESHOLD  → leave blank
 *     else                     → glyph from ramp[round(ρ̄)],
 *                                 colour from momentum bucket,
 *                                 A_BOLD if dense.
 *
 * The fluid's appearance is driven entirely by these two mappings;
 * see tutorial T7.
 */

static void render_fluid_field(int draw_rows, int draw_cols) {
  for (int r = 0; r < draw_rows; r++) {
    for (int c = 0; c < draw_cols; c++) {

      if (g_scene.wall_mask[r][c]) {
        attron(COLOR_PAIR(PAIR_WALL) | A_BOLD);
        mvaddch(r, c, '#');
        attroff(COLOR_PAIR(PAIR_WALL) | A_BOLD);
        continue;
      }

      float density, momentum_doubled;
      cell_neighbourhood_average(r, c, &density, &momentum_doubled);

      if (density < DENSITY_BLANK_THRESHOLD) {
        mvaddch(r, c, ' ');
        continue;
      }

      int glyph_index = (int)(density + 0.5f);
      if (glyph_index < 1)
        glyph_index = 1;
      if (glyph_index >= DENSITY_GLYPH_COUNT)
        glyph_index = DENSITY_GLYPH_COUNT - 1;

      int bucket = momentum_to_color_bucket(momentum_doubled);
      int pair = PAIR_MOMENTUM_FIRST + bucket;
      attr_t bold = (density >= DENSITY_BOLD_THRESHOLD) ? A_BOLD : 0;

      attron(COLOR_PAIR(pair) | bold);
      mvaddch(r, c, (chtype)(unsigned char)density_glyph_ramp[glyph_index]);
      attroff(COLOR_PAIR(pair) | bold);
    }
  }
}

/* ===================================================================== */
/* §18  hud — top status + bottom hint                                   */
/* ===================================================================== */
/*
 * Two HUD elements per CLAUDE.md spec:
 *   - STATUS at top-right: PAIR_HUD bright yellow + A_BOLD
 *   - HINT   at bottom:    PAIR_HINT bright cyan   + A_BOLD
 *
 * The PRESET DESCRIPTION lives at row 1 (dimmed-yellow A_NORMAL) so
 * the bright top status row doesn't fight with descriptive text.
 */

static void hud_paint_status(int term_cols) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " [%d] %-9s tick:%-6ld inlet:%3.0f%% steps:%2d sim:%2dHz "
           "theme:%-7s %s ",
           g_scene.active_preset_index + 1, preset_table[g_scene.active_preset_index].name,
           g_scene.physics_tick_count, g_scene.inlet_probability_per_cell * 100.0f,
           g_scene.physics_steps_per_frame, g_scene.sim_steps_per_second,
           color_theme_table[g_scene.active_theme_index].name,
           g_scene.simulation_paused ? "PAUSED" : "running");
  int len = (int)strlen(buf);
  int x = term_cols - len;
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_description(int term_rows, int term_cols) {
  /* Preset description on the second row from bottom (above hint). */
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(term_rows - 2, 0, "  %-*s", term_cols - 2,
           preset_table[g_scene.active_preset_index].desc);
  attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_paint_hint(int term_rows) {
  const char *hint = " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme  "
                     "i/I:inlet  +/-:steps  ]/[:simHz ";
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0, "%s", hint);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §19  screen — ncurses init / cleanup                                  */
/* ===================================================================== */

/*
 * Screen — terminal extent record.  ncurses owns the buffers; we
 * keep only cell dimensions for HUD placement and field clipping.
 *
 * Render pipeline (one frame): erase → paint_field → hud_paint_*
 *   → wnoutrefresh(stdscr) → doupdate().  Diff-only writes to the
 *   terminal — no flicker.  See [8] Raymond §11.
 */
typedef struct {
    int rows;   /* terminal height in cells (getmaxyx)             */
    int cols;   /* terminal width  in cells (getmaxyx)             */
} Screen;

static void screen_init(Screen *s, int theme_index) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init(theme_index);
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);

  g_scene.grid_active_rows = (s->rows - HUD_RESERVED_ROWS < GRID_ROWS_MAX)
                         ? (s->rows - HUD_RESERVED_ROWS)
                         : GRID_ROWS_MAX;
  g_scene.grid_active_cols = (s->cols < GRID_COLS_MAX) ? s->cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;
}

static void screen_present_frame(Screen *s) {
  erase();
  int draw_rows = (g_scene.grid_active_rows < s->rows - HUD_RESERVED_ROWS)
                      ? g_scene.grid_active_rows
                      : s->rows - HUD_RESERVED_ROWS;
  int draw_cols = (g_scene.grid_active_cols < s->cols) ? g_scene.grid_active_cols : s->cols;
  render_fluid_field(draw_rows, draw_cols);
  hud_paint_status(s->cols);
  hud_paint_description(s->rows, s->cols);
  hud_paint_hint(s->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §20  app — main loop + signals + input                                */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int ch, Screen *s) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case 'p':
  case ' ':
    g_scene.simulation_paused = !g_scene.simulation_paused;
    break;

  case 'r':
  case 'R':
    scene_load(g_scene.active_preset_index);
    break;

  case 'n':
    scene_load((g_scene.active_preset_index + 1) % PRESET_COUNT);
    break;
  case 'N':
    scene_load((g_scene.active_preset_index + PRESET_COUNT - 1) % PRESET_COUNT);
    break;

  case 't':
    g_scene.active_theme_index = (g_scene.active_theme_index + 1) % THEME_COUNT;
    colors_apply_theme(g_scene.active_theme_index);
    break;
  case 'T':
    g_scene.active_theme_index = (g_scene.active_theme_index + THEME_COUNT - 1) % THEME_COUNT;
    colors_apply_theme(g_scene.active_theme_index);
    break;

  case 'i':
    if (g_scene.inlet_probability_per_cell + INLET_PROB_STEP <= INLET_PROB_MAX)
      g_scene.inlet_probability_per_cell += INLET_PROB_STEP;
    break;
  case 'I':
    if (g_scene.inlet_probability_per_cell - INLET_PROB_STEP >= INLET_PROB_MIN)
      g_scene.inlet_probability_per_cell -= INLET_PROB_STEP;
    break;

  case '+':
  case '=':
    if (g_scene.physics_steps_per_frame < STEPS_PER_FRAME_MAX)
      g_scene.physics_steps_per_frame++;
    break;
  case '-':
    if (g_scene.physics_steps_per_frame > STEPS_PER_FRAME_MIN)
      g_scene.physics_steps_per_frame--;
    break;

  case ']':
    if (g_scene.sim_steps_per_second + SIM_HZ_STEP <= SIM_HZ_MAX)
      g_scene.sim_steps_per_second += SIM_HZ_STEP;
    break;
  case '[':
    if (g_scene.sim_steps_per_second - SIM_HZ_STEP >= SIM_HZ_MIN)
      g_scene.sim_steps_per_second -= SIM_HZ_STEP;
    break;

  default:
    break;
  }
  (void)s;
  return true;
}

int main(void) {
  xorshift_state = (uint32_t)time(NULL) ^ 0xFACEB00Cu;

  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  collision_table_build();

  Screen screen;
  screen_init(&screen, g_scene.active_theme_index);

  g_scene.grid_active_rows = (screen.rows - HUD_RESERVED_ROWS < GRID_ROWS_MAX)
                         ? (screen.rows - HUD_RESERVED_ROWS)
                         : GRID_ROWS_MAX;
  g_scene.grid_active_cols =
      (screen.cols < GRID_COLS_MAX) ? screen.cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;

  scene_load(g_scene.active_preset_index);

  int64_t next_physics_at_ns = clock_now_ns();
  const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

  while (!g_should_quit) {
    int64_t frame_start = clock_now_ns();

    /* ── input ── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch, &screen)) {
        g_should_quit = 1;
        break;
      }
    }

    /* ── resize ── */
    if (g_resize_pending) {
      g_resize_pending = 0;
      screen_resize(&screen);
      scene_load(g_scene.active_preset_index);
      next_physics_at_ns = clock_now_ns();
    }

    /* ── physics ── */
    int64_t now_ns = clock_now_ns();
    if (!g_scene.simulation_paused && now_ns >= next_physics_at_ns) {
      for (int s = 0; s < g_scene.physics_steps_per_frame; s++)
        physics_step();
      next_physics_at_ns = now_ns + TICK_NS(g_scene.sim_steps_per_second);
    }

    /* ── render ── */
    screen_present_frame(&screen);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start;
    if (spent < frame_cap_ns)
      clock_sleep_ns(frame_cap_ns - spent);
  }

  return 0;
}
