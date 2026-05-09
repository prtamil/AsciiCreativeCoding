/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * flowfield.c — animated 2-D flow visualisation in ASCII
 *
 * DEMO: Hundreds of TRACER particles drift across the screen as if
 *       riding wind, smoke, or ocean currents.  The wind itself is
 *       INVISIBLE (toggle with 'a' to see the field arrows behind
 *       the particles); the tracer trails are what reveal it.  The
 *       wind evolves slowly over time so the pattern keeps
 *       changing.  The result reads as smoke wisps, sand drifting,
 *       or aurora — depending on theme.
 *
 *       Everything you see emerges from THREE ideas:
 *
 *         1. PERLIN NOISE — a smooth pseudo-random function on a 2-D
 *            grid that LOOKS organic (not white noise).
 *         2. NOISE → ANGLE — two independent noise samples give the
 *            (vx, vy) of a vector at every grid cell; atan2 turns
 *            them into a single angle.
 *         3. PARTICLE ADVECTION — at every tick each tracer reads
 *            the local angle and walks one step in that direction.
 *
 *       Tutorials T2-T6 build these from scratch.  Press 'a' to
 *       see the wind arrows underneath; the connection between the
 *       arrows and the tracer trails is the visual proof of
 *       advection.
 *
 * Study alongside:
 *   procedural/fields/flow_field_particles.c — same idea, different
 *                                                visual aesthetic
 *   procedural/fields/perin_noise_flow_showcase.c — Perlin noise
 *                                                visualisation alone
 *   procedural/fields/curl_noise_vector_field.c — divergence-free
 *                                                noise (better for
 *                                                "real" fluid look)
 *   fluid/navier_stokes.c — the OPPOSITE approach: a real fluid
 *                                                solver, not a
 *                                                noise lookup
 *
 * Section map:
 *   §1  config       — every tunable constant
 *   §2  clock        — monotonic ns time + sleep
 *   §3  rng          — seeded shuffle for the perlin permutation table
 *   §4  perlin       — single-octave 2-D Perlin gradient noise
 *   §5  fbm          — multi-octave fractional Brownian motion
 *   §6  field        — 2-D angle grid: build, sample, evolve
 *   §7  arrows       — direction → ASCII arrow + hue table
 *   §8  themes       — rainbow + three mono palettes (data only)
 *   §9  colors       — ncurses pair setup
 *   §10 tracer       — one particle: pose, trail ring buffer
 *   §11 tracer_step  — Lagrangian advection (one tick of one tracer)
 *   §12 tracer_paint — fading-trail rendering
 *   §13 scene        — pool + field + tick orchestrator
 *   §14 hud          — top status + bottom key-hint strip
 *   §15 screen       — ncurses init / cleanup
 *   §16 app          — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r                respawn all tracers at random positions
 *   a                toggle field-arrow background ON/OFF
 *   t                cycle theme  (rainbow → cyan → green → white)
 *   + / =            more tracers
 *   -                fewer tracers
 *   ] / [            sim FPS up / down
 *   f / F            field evolves faster / slower (time-axis speed)
 *   s / S            longer / shorter trail
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/flowfield.c \
 *       -o flowfield -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose.  T1 motivates vector fields; T2-T4 build Perlin
 *      noise; T5 turns noise into a flow direction; T6 advects a
 *      tracer along it.  After T6 you can write a minimal
 *      flowfield from scratch.
 *   2. §4 perlin + §5 fbm — the math underpinning everything.
 *      Read AFTER tutorials T2-T4.
 *   3. §6 field — assembles a per-cell angle from FBM noise.
 *   4. §11 tracer_step — the heart of the simulation: ONE tracer's
 *      one-tick update.  Six lines.  Read AFTER tutorial T6.
 *   5. §13 scene + §16 app — orchestration.  Standard framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   tracer_pos_col, tracer_pos_row    a particle's continuous (x, y) position
 *   tracer_step_cells                  per-particle speed (cells/tick)
 *   tracer_angle                       last sampled flow angle (radians)
 *   trail_col[i], trail_row[i]         past-position ring buffer
 *   trail_write_index                  next slot to write
 *   trail_filled_count                 how many slots are populated (0..len)
 *   trail_length                       active trail length (slots used)
 *   ticks_until_respawn                lifetime countdown
 *   trail_pair_base                    the brightest colour pair for this trail
 *   tracer_alive                       slot is in use
 *
 *   flow_angle_at(field, c, r)         radians at integer cell (c, r)
 *   flow_angle_bilinear(field, x, y)   bilinearly interpolated, sub-cell
 *   field_evolution_speed              how fast the noise time-axis advances
 *
 *   perlin_value(x, y)                 single-octave Perlin in [-1, +1]
 *   fbm_value(x, y, octaves)           fractional Brownian motion sum
 *
 * Background you need
 * ───────────────────
 *   - 2-D arrays, ring buffers.
 *   - atan2(y, x).
 *   - Bilinear interpolation (lerp + lerp).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - The Navier-Stokes equations.  This file does NOT solve fluid
 *     dynamics — the field is a noise lookup, not a physical
 *     simulation.  See fluid/navier_stokes.c for the real thing.
 *   - Curl-free / divergence-free noise.  Curl noise is the
 *     improvement that makes this look more like real fluid; out
 *     of scope here.  See procedural/fields/curl_noise_vector_field.c.
 *   - Streamline integration with adaptive step size.  We do plain
 *     forward Euler — accurate enough for visualisation.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : LAGRANGIAN PARTICLE ADVECTION on a procedurally-
 *                 generated vector field.  Each tracer particle reads
 *                 the local flow angle and steps one cell in that
 *                 direction every tick.  The field is computed from
 *                 fractional Brownian motion (FBM) — a sum of three
 *                 octaves of 2-D Perlin noise.  Two independent FBM
 *                 samples (offset spatially) form the (vx, vy) of a
 *                 vector at each grid cell; atan2(vy, vx) gives the
 *                 angle.  The field's "time" axis advances slowly so
 *                 the pattern evolves smoothly.
 *
 * Math          : Perlin noise (Ken Perlin, 1985) — interpolates a
 *                 random GRADIENT vector at each integer corner of a
 *                 grid.  At a query point inside a cell:
 *
 *                     dot_product_at_corner_k = grad_k · offset_to_corner_k
 *                     value = bilinear_lerp(those four dot products)
 *                     value = smoothstep_interpolated  (3t² - 2t³)
 *
 *                 Result: a smooth function in [-1, +1] that looks
 *                 random but has continuous derivatives.
 *
 *                 FBM:  Σ (perlin(x · 2^i, y · 2^i) · 0.5^i)
 *                      i=0..octaves-1
 *
 *                 → broad swirls (low octave) plus fine detail
 *                 (high octaves), mimicking turbulent flow.
 *
 *                 Forward Euler advection:
 *                     pos_new = pos + (cos(angle), sin(angle)) · step
 *
 * Data layout   : - perlin permutation table (256 bytes, doubled to
 *                   512 to avoid modulo)
 *                 - 2-D angle field, one float per cell
 *                 - tracer pool: each tracer has (pos, speed, life,
 *                   trail ring buffer of up to MAX_TRAIL positions)
 *                 - rendering: erase → optional field arrows →
 *                   tracer trails → HUD
 *
 * Performance   : O(W·H · OCTAVES) for field rebuild every tick.
 *                 O(N_TRACERS · trail_length) for drawing.  At W=160,
 *                 H=40, OCTAVES=3, N_TRACERS=300, trail=14 — that's
 *                 ~19k noise evaluations + ~4200 cell paints per
 *                 tick.  Sub-millisecond on any modern CPU.
 *
 * References
 * ──────────
 *   Perlin, "An image synthesizer" (SIGGRAPH 1985) — the original
 *     Perlin-noise paper.
 *   Perlin, "Improving noise" (SIGGRAPH 2002) — the gradient hash
 *     + smoothstep⁵ refinement, which we use here in simplified form.
 *   Reynolds, "Steering Behaviors for Autonomous Characters" (1999) —
 *     same particle-following pattern; we just use noise instead of
 *     a target seek.
 *   Inigo Quilez, "Painting a 2D animation with noise":
 *     https://iquilezles.org/articles/warp/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The screen is FILLED with an INVISIBLE WIND.  At every cell of the
 * terminal grid there's a wind direction (and implicitly a unit
 * speed).  We can't draw the wind directly — that'd be too busy.
 * Instead, we drop hundreds of tiny TRACER particles into the wind
 * and let each one drift at the local speed in the local direction.
 * The trails the tracers leave are what reveal the wind's shape.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a calm field of grass on a hilly day.  The wind blows in
 * complicated swirls — eddies behind hills, smooth flows down
 * valleys.  You can't SEE the wind, but if you sprinkle thousands
 * of tiny seed-fluffs onto the field, they DRIFT — and the patterns
 * they trace as they drift make the wind visible.  Slow swirls
 * become slow circling fluffs.  Sharp gusts become straight streaks.
 * Eventually each fluff lands somewhere or blows off the edge; new
 * fluffs spawn to keep the field populated.
 *
 * This file is exactly that:
 *
 *   - The "wind" is a 2-D angle field generated from PERLIN NOISE
 *     (a smooth random function, T2).
 *   - The "fluffs" are TRACERS — particles that drift one cell per
 *     tick in the direction the local wind tells them.
 *   - The "trails" are the last N positions of each tracer, drawn
 *     fading from bright (recent) to dim (old).
 *
 * THE THREE-LAYER STRUCTURE
 * ─────────────────────────
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   Layer 3: TRACERS (visible)                     │
 *      │      hundreds of moving particles                │
 *      │      with fading trails ' . , + ~ * '            │
 *      │                                                  │
 *      │   Layer 2: VECTOR FIELD (invisible by default,   │
 *      │            press 'a' to see)                     │
 *      │      one angle per cell, drives tracers          │
 *      │      (also represented by ASCII arrow chars)     │
 *      │                                                  │
 *      │   Layer 1: PERLIN NOISE (mathematical, never     │
 *      │            drawn directly)                       │
 *      │      smooth pseudo-random scalar field           │
 *      │      sampled by FBM to give the flow angles      │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Each layer is a CLEAN ABSTRACTION over the layer below.  Tracers
 * don't know about Perlin noise; they read the field.  The field
 * doesn't know about advection; it just supplies an angle when
 * asked.  Perlin noise doesn't know about anything else; it just
 * returns a smooth scalar at any (x, y).
 *
 * GEOMETRY OF ONE TRACER STEP
 * ───────────────────────────
 *
 *      tick t                    tick t + 1
 *
 *      ●               →         '─→  ●
 *      tracer at                  trail glyph    tracer
 *      (x, y)                     where it       at
 *                                 was             (x + cos θ · v,
 *                                                  y + sin θ · v)
 *
 *      The angle θ comes from sampling the field at (x, y).
 *      The speed v is per-tracer (with jitter), in cells/tick.
 *
 * KEY FORMULAS
 * ────────────
 *   Perlin gradient hash + smoothstep:
 *     value(x, y) = bilinear_smoothstep(
 *                    grad(corner) · (x - corner.x, y - corner.y) )
 *
 *   FBM (3 octaves):
 *     fbm(x, y) = perlin(x, y)              · 1.0
 *               + perlin(x · 2, y · 2)      · 0.5
 *               + perlin(x · 4, y · 4)      · 0.25
 *
 *   Flow angle from two noise samples:
 *     vx = fbm(x · sx, y · sy + t)
 *     vy = fbm(x · sx + 100, y · sy + 200 + t)    ← offset → independent
 *     angle = atan2(vy, vx)
 *
 *   Tracer advection (forward Euler):
 *     pos = pos + (cos angle, sin angle) · speed_cells
 *
 *   Wrap-around boundary (toroidal):
 *     pos.x = (pos.x + cols) mod cols
 *     pos.y = (pos.y + rows) mod rows
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - Trail wrap on toroidal field.  When a tracer crosses the
 *     screen edge, its NEW position is on the OPPOSITE side, but
 *     the trail's PREVIOUS position is on the FAR side.  Drawing a
 *     line between them would draw across the screen.  We just
 *     stamp each trail position individually (no line connect),
 *     so the wrap looks natural.
 *   - "Stuck" tracers.  If two adjacent cells have nearly opposite
 *     angles, a tracer can oscillate between them.  We give each
 *     tracer a finite LIFETIME so it gets respawned to a fresh
 *     position eventually.
 *   - Field rebuilt every tick.  Cheaper alternative: rebuild only
 *     when time has advanced by some Δ.  We rebuild every tick for
 *     visual smoothness; the cost is negligible.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Default theme (rainbow): tracers should clearly form streamlines
 *     — long curving paths, not random dots.  If trails look
 *     scattered, FIELD_EVOLVE_SPEED is too high (field changes
 *     faster than tracers can follow).
 *   - Press 'a': field arrows visible behind the tracers.  Each
 *     trail should run ALONG an arrow direction (visual proof of
 *     advection — T6).
 *   - Press 'F' to slow the field evolution: the wind looks
 *     "frozen," tracers form long stable streamlines.
 *   - Press 'r' to respawn: tracers vanish and reappear randomly;
 *     trails grow over the next ~14 ticks.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials that build the flowfield from first principles.
 *
 *   T1  What's a vector field, and why "flow"?
 *   T2  Why Perlin noise — smooth random functions
 *   T3  Perlin noise step by step (gradient + smoothstep + lerp)
 *   T4  FBM — layering octaves for organic detail
 *   T5  From noise to flow direction (atan2 trick)
 *   T6  Lagrangian advection — tracers riding the flow
 *   T7  Trail ring buffer — fixed memory, dynamic content
 *   T8  Why this isn't real fluid (and what it would take)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT'S A VECTOR FIELD, AND WHY "FLOW"?
 * ──────────────────────────────────────────
 * A SCALAR FIELD assigns a NUMBER to every point of space:
 *   - temperature in a room: T(x, y) → degrees
 *   - elevation on a map: H(x, y) → metres
 *
 * A VECTOR FIELD assigns a VECTOR (an arrow with direction +
 * magnitude) to every point:
 *   - wind in a weather map: V(x, y) → (vx, vy) in m/s
 *   - water current under a ship: U(x, y) → flow velocity
 *   - electric field around a charge: E(x, y) → V/m
 *
 * "FLOW" is the technical term for following a vector field by
 * stepping along its direction.  If at every moment you walk in
 * the local flow direction at your speed, you trace a STREAMLINE
 * — a curve tangent to the field everywhere.  This is what tracer
 * particles in this file do.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   Vector field (sparse arrows):                  │
 *      │                                                  │
 *      │     ↗   ↗   →   →   ↘                            │
 *      │   ↗   →   →   →   ↘   ↘                          │
 *      │     →   →   →   →   →                            │
 *      │   ↘   →   →   →   ↗   ↗                          │
 *      │     ↘   →   →   ↗                                │
 *      │                                                  │
 *      │   Streamline (one tracer's path):                │
 *      │                                                  │
 *      │              .─→─.                               │
 *      │             /     \                              │
 *      │        ●─→─●       \                             │
 *      │                     ●─→─                          │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * In this file, the "wind" is procedurally generated from Perlin
 * noise (T2-T5), and "tracers" are the fluffs we follow (T6).
 *
 * T2  WHY PERLIN NOISE — SMOOTH RANDOM FUNCTIONS
 * ──────────────────────────────────────────────
 * Two ways to generate "random-looking" 2-D scalar fields:
 *
 *   WHITE NOISE — rand() at every cell.
 *   Pure salt-and-pepper.  Adjacent cells are UNCORRELATED.  Looks
 *   like TV static.  Bad for flow:  a tracer that lands on a cell
 *   pointing east, then walks 1 cell east, finds an unrelated
 *   random direction.  No streamlines emerge.
 *
 *   PERLIN NOISE — gradient hash + interpolation (Perlin 1985).
 *   Adjacent cells are HIGHLY CORRELATED.  Looks like a smooth
 *   landscape.  Adjacent angles vary continuously, so streamlines
 *   are smooth curves.  Looks like wind, smoke, or marble.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   White noise:           Perlin noise:           │
 *      │                                                  │
 *      │   ░▓░▓▒▒░▓▒░             ░░▒▒▓▒▒░░░              │
 *      │   ▓▒░▓▓░▓▒░▓             ░▒▓▓▓▒▒░░░              │
 *      │   ▒░▓░▒▓░▓▓░             ▒▓▓▓▓▒▒░░░              │
 *      │   ▓░▒░▓░▒░▒▓             ▒▓▓▒▒░░░░░              │
 *      │   ░▓▒░▓▒░▒░▓             ▒▒▒░░░░░░░              │
 *      │                                                  │
 *      │   uncorrelated           organic blobs           │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Perlin noise is the workhorse of procedural generation:
 *   - Minecraft terrain
 *   - Cloud textures in 3-D games
 *   - Fire / smoke shaders
 *   - Procedural marble, wood, lava
 *   - Demoscene effects
 *
 * The simplest usable variant is what we implement next.
 *
 * T3  PERLIN NOISE STEP BY STEP (GRADIENT + SMOOTHSTEP + LERP)
 * ────────────────────────────────────────────────────────────
 * Goal: a function `perlin(x, y)` returning a value in [-1, +1]
 * that varies SMOOTHLY (continuous derivatives) but UNPREDICTABLY.
 *
 *   Step 1: AT EVERY INTEGER GRID POINT, assign a random GRADIENT
 *           vector.  We use a permutation table (256 bytes,
 *           shuffled) hashed by integer (xi, yi):
 *
 *               hash[xi, yi] = perm[ perm[xi] + yi ]      ∈ [0, 255]
 *
 *           From the hash, derive a 2-D unit gradient (we use 4
 *           directions: ±x, ±y combos — Perlin's "improved"
 *           variation uses 4 such vectors per grid cell).
 *
 *   Step 2: At a query point (x, y), find which grid cell contains
 *           it.  The cell has 4 corners c00, c10, c01, c11.
 *
 *               (xi, yi)   (xi+1, yi)
 *                  ┌─────────┐
 *                  │         │
 *                  │   ●     │   ← query point (x, y)
 *                  │         │
 *                  └─────────┘
 *               (xi, yi+1)  (xi+1, yi+1)
 *
 *   Step 3: At each corner, compute the DOT PRODUCT of the gradient
 *           vector with the OFFSET FROM CORNER TO QUERY POINT:
 *
 *               d00 = grad(c00) · (x - xi    , y - yi    )
 *               d10 = grad(c10) · (x - xi - 1, y - yi    )
 *               d01 = grad(c01) · (x - xi    , y - yi - 1)
 *               d11 = grad(c11) · (x - xi - 1, y - yi - 1)
 *
 *           Each d is a SCALAR.  At a corner exactly, the dot
 *           product is 0 (the offset is zero).  Slightly off the
 *           corner, it's a small linear deviation.
 *
 *   Step 4: BILINEAR INTERPOLATE the four dot products by the
 *           query's fractional offsets within the cell:
 *
 *               u = smoothstep(x - xi)
 *               v = smoothstep(y - yi)
 *               value = lerp( lerp(d00, d10, u),
 *                             lerp(d01, d11, u), v )
 *
 *           SMOOTHSTEP (3t² - 2t³) is the magic.  Plain linear
 *           interpolation makes the noise look CRYSTALLINE — you
 *           can see the grid.  Smoothstep makes it look SMOOTH
 *           because its first derivative is continuous at corners.
 *
 *   Step 5: Result is in [-1, +1].  Done.
 *
 * Pseudocode:
 *
 *     perlin_value(x, y):
 *       xi = floor(x);  yi = floor(y)
 *       fx = x - xi;    fy = y - yi
 *       u  = smoothstep(fx);  v = smoothstep(fy)
 *       g00 = grad(hash[xi   , yi   ])
 *       g10 = grad(hash[xi+1 , yi   ])
 *       g01 = grad(hash[xi   , yi+1 ])
 *       g11 = grad(hash[xi+1 , yi+1 ])
 *       d00 = g00 · (fx,   fy  )
 *       d10 = g10 · (fx-1, fy  )
 *       d01 = g01 · (fx,   fy-1)
 *       d11 = g11 · (fx-1, fy-1)
 *       return lerp( lerp(d00, d10, u), lerp(d01, d11, u), v )
 *
 * Implemented in §4 perlin.
 *
 * T4  FBM — LAYERING OCTAVES FOR ORGANIC DETAIL
 * ─────────────────────────────────────────────
 * Single-octave Perlin noise has ONE characteristic length scale —
 * roughly the spatial scale you queried at.  Real-world flow has
 * MANY scales: huge weather systems, regional eddies, local gusts,
 * tiny micro-turbulence.  To synthesise a flow with such richness,
 * SUM SEVERAL OCTAVES of Perlin noise at increasing frequencies and
 * decreasing amplitudes:
 *
 *     fbm(x, y) = Σ  perlin(x · 2^i, y · 2^i) · (0.5)^i
 *               i=0..N-1
 *
 * - Octave 0 contributes BIG SWIRLS (low frequency, high amplitude).
 * - Octave 1 adds smaller details at half amplitude.
 * - Octave 2 adds even smaller wobbles at quarter amplitude.
 * - ...
 *
 * The result has visible structure at every scale — a FRACTAL
 * pattern.  Mandelbrot-like in spirit, but generated cheaply.
 *
 * Three octaves is plenty for visual flow (more octaves cost more
 * noise evaluations per cell and give diminishing returns).
 *
 * Implemented in §5 fbm.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   1 octave         2 octaves        3 octaves    │
 *      │                                                  │
 *      │   smooth           smooth +         organic      │
 *      │   blobs            wobbles          detail       │
 *      │                                                  │
 *      │   ▒▒░░             ▒▓▒░░░          ░▒▓▒░░░       │
 *      │   ▓▓▒▒             ▒▓▓▒░▒          ▒▓▓▓▒░▒       │
 *      │   ▓▓▒              ▓▓▓▒▒░          ▓▓▒▒░░▒       │
 *      │   ▒░░              ▓▒░░░░          ▒▒░░░░▒       │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * T5  FROM NOISE TO FLOW DIRECTION (THE ATAN2 TRICK)
 * ──────────────────────────────────────────────────
 * FBM gives us a SCALAR field.  We need a VECTOR field — two
 * numbers per cell, the (vx, vy) of the local flow.
 *
 * Trick: sample TWO INDEPENDENT FBM functions, one for vx and one
 * for vy.  We make them independent by SAMPLING AT OFFSET POINTS:
 *
 *     vx = fbm(x · scale_x,        y · scale_y       + time)
 *     vy = fbm(x · scale_x + 100,  y · scale_y + 200 + time · 1.1)
 *
 * The "+ 100" and "+ 200" shifts move the second sample to a
 * faraway region of noise space — far enough that the two values
 * are uncorrelated.  The "+ time" terms drift the noise space over
 * time so the field evolves smoothly (T2 of the file: the scene
 * keeps changing).
 *
 * Now (vx, vy) is a 2-D vector.  Convert to an ANGLE for storage
 * and easy interpolation:
 *
 *     angle = atan2(vy, vx)            ∈ (-π, +π]
 *
 * (Storing only the angle discards the magnitude, but for a
 * visualisation we want all tracers to move at similar speeds, so
 * a unit-length advection is what we want anyway.)
 *
 * The field becomes one float per cell — angle in radians.  At
 * any sub-cell point we BILINEAR-INTERPOLATE the four corners (T6
 * uses this).  Bilinear-interpolating angles across the
 * −π↔+π wrap is technically wrong (you can get tracer wobble at
 * the seam) but for visual purposes it's invisible.
 *
 * T6  LAGRANGIAN ADVECTION — TRACERS RIDING THE FLOW
 * ──────────────────────────────────────────────────
 * EULERIAN view of fluid: stand still and watch the field at fixed
 * grid points.  The state is V(x, y, t).
 *
 * LAGRANGIAN view: ride along with a tracer.  Track ONE PARTICLE
 * over time as it moves through the flow.
 *
 * We use Lagrangian here because TRACERS ARE WHAT WE DRAW.  At
 * each tick, for each tracer:
 *
 *     1. Sample the flow angle at the tracer's CURRENT position
 *        (using bilinear interpolation between the four
 *        surrounding grid cells).
 *     2. Step the tracer by `step_cells` in the direction angle:
 *
 *          tracer.x += cos(angle) · step
 *          tracer.y += sin(angle) · step
 *
 *     3. Wrap the tracer toroidally (off the right edge → on the
 *        left edge, etc.).
 *
 * That's it.  Each tracer is a 6-line update.  Hundreds of tracers
 * later, you've got a beautiful flowfield visualisation.
 *
 * Pseudocode:
 *
 *     for each tracer t:
 *       angle = field_sample_bilinear(t.col, t.row)
 *       t.col += cos(angle) · t.step
 *       t.row += sin(angle) · t.step
 *       wrap_toroidal(&t.col, &t.row)
 *       record_in_trail_buffer(t)
 *
 * Implemented in §11 tracer_step.
 *
 * NOTE: this is FORWARD EULER integration of the ODE
 * dx/dt = V(x, t).  Higher-order integrators (RK2, RK4) give
 * smoother streamlines but cost more.  For visualisation, Euler
 * is fine because the visible "error" is just slight curve
 * deviation — invisible.  Real fluid simulators (e.g. Stam's
 * stable fluids) often use SEMI-LAGRANGIAN with RK2; not needed
 * here.
 *
 * T7  TRAIL RING BUFFER — FIXED MEMORY, DYNAMIC CONTENT
 * ─────────────────────────────────────────────────────
 * Each tracer needs to remember its LAST N positions to draw a
 * fading trail.  We don't want to allocate per-tracer; instead
 * each tracer owns a FIXED-SIZE ARRAY used as a CIRCULAR
 * (RING) BUFFER:
 *
 *     trail_col[MAX_TRAIL]
 *     trail_row[MAX_TRAIL]
 *     trail_write_index    — slot to write next
 *     trail_filled_count   — how many slots are populated (≤ MAX)
 *
 * Per tick:
 *
 *     trail_col[trail_write_index] = current_col_int
 *     trail_row[trail_write_index] = current_row_int
 *     trail_write_index = (trail_write_index + 1) % trail_length
 *     if trail_filled_count < trail_length:
 *         trail_filled_count++
 *
 * To draw, walk the trail in time order — OLDEST first, NEWEST
 * last — applying a BRIGHTNESS RAMP along the way:
 *
 *     for i in 0..trail_filled_count-1:
 *       slot = (trail_write_index − trail_filled_count + i + 2*L) % L
 *       brightness = i / (trail_filled_count − 1)
 *       paint(trail_col[slot], trail_row[slot], glyph_for(brightness))
 *
 * The "+ 2*L" before mod handles negative indices (C's mod can
 * return negative for negative dividends — classic gotcha).
 *
 * Why a ring buffer?  Because we add ONE position per tick and
 * remove ONE.  A naïve linked list would alloc/free every tick;
 * a shifting array would be O(N).  The ring buffer is O(1) per
 * push, O(N) only when DRAWING (one walk).
 *
 * Same trick is used in audio buffers, network packet queues,
 * keyboard typing-history buffers, and the trail buffers in
 * fk_centipede.c, snake_forward_kinematics.c, etc.
 *
 * T8  WHY THIS ISN'T REAL FLUID (AND WHAT IT WOULD TAKE)
 * ──────────────────────────────────────────────────────
 * The result LOOKS like fluid flow but isn't.  Differences:
 *
 *   This file:
 *     - field is a Perlin/FBM lookup; doesn't satisfy any
 *       conservation law.
 *     - field has no DIVERGENCE-FREE property.  Real
 *       incompressible fluid has ∇ · V = 0 (mass conserved); our
 *       field does not.  Visual symptom: tracers can pile up in
 *       "sinks" (cells where the surrounding angles converge
 *       inward) or rapidly empty out of "sources."
 *     - tracers are PASSIVE — they don't push back on the field.
 *       Real fluid flows respond to obstacles, walls, viscosity.
 *
 *   Real fluid simulator (e.g. fluid/navier_stokes.c):
 *     - solves the Navier-Stokes equations on the grid.
 *     - enforces ∇ · V = 0 every step via a pressure projection.
 *     - tracers (or smoke density, or temperature) are advected
 *       by the live solved velocity field, not a fixed lookup.
 *     - obstacles cleanly induce vortex shedding, boundary layers,
 *       turbulent wakes — all from the equations, no hand-tuning.
 *
 * To turn this file into a real fluid:
 *   1. Replace the Perlin field with a velocity grid initialised
 *      to zero.
 *   2. Each tick: solve the NS equations on the grid (advect →
 *      diffuse → project).
 *   3. Use the resulting (vx, vy) for the tracer advection.
 *
 * Step 2 is the substantial work; see fluid/navier_stokes.c for
 * Stam's stable-fluid algorithm in ~700 lines.
 *
 * BETTER MIDDLE GROUND: CURL NOISE.
 *
 *   Take a Perlin SCALAR field ψ(x, y).  Define the velocity as
 *   the CURL:
 *
 *     vx = ∂ψ/∂y       vy = -∂ψ/∂x
 *
 *   This is automatically divergence-free!  No tracer pile-ups.
 *   Looks dramatically more like real fluid than plain noise.
 *   See procedural/fields/curl_noise_vector_field.c.
 *
 * For "look like wind" without the cost of NS, curl noise is the
 * standard choice.  This file uses plain noise for pedagogical
 * clarity (you only need ONE noise function + atan2).
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
/* §1  config — every constant in one place                              */
/* ===================================================================== */
/*
 * What's in this section
 * ----------------------
 * Tunable knobs grouped by purpose.  Anything literal in the rest of
 * the file should be a NAME from §1 (project rule: no magic numbers
 * scattered through the code).
 */

enum {
    /* Render frame cap. */
    TARGET_FPS_HZ        = 60,

    /* Simulation step rate (independent of render rate). */
    SIM_HZ_MIN           =  5,
    SIM_HZ_DEFAULT       = 30,
    SIM_HZ_MAX           = 60,
    SIM_HZ_STEP          =  5,

    /* HUD recompute cadence. */
    FPS_RECOMPUTE_MS     = 500,

    /* Tracer pool — fixed-size pre-allocated array. */
    TRACERS_MIN          =  50,
    TRACERS_DEFAULT      = 300,
    TRACERS_MAX          = 800,
    TRACERS_STEP         =  50,

    /* Trail buffer per tracer. */
    TRAIL_LEN_MIN        =  3,
    TRAIL_LEN_DEFAULT    = 14,
    TRAIL_LEN_MAX        = 20,

    /* FBM octaves — 3 is the sweet spot for organic-looking flow. */
    FBM_OCTAVES          =  3,

    /* Number of distinct hue pairs around the angle wheel (T5). */
    HUE_WHEEL_PAIRS      =  8,

    /* Number of themes (rainbow + 3 monos). */
    THEME_COUNT          =  4,

    /* Maximum trail length compiled in (sized once for static arrays). */
    TRAIL_LEN_HARD_MAX   = 20,
};

/* PHYSICAL / VISUAL CONSTANTS — units in the comment. */

/* Per-tracer step distance (cells per tick).  Tracers get a per-particle
 * jitter so they don't all move in lockstep. */
#define TRACER_STEP_BASE_CPT     0.9f
#define TRACER_STEP_JITTER_CPT   0.4f          /* uniform [-J/2, +J/2] */

/* How fast the noise time-axis advances per tick.  Smaller = slower
 * field evolution = longer-lived streamlines. */
#define FIELD_EVOLUTION_DEFAULT  0.008f
#define FIELD_EVOLUTION_MIN      0.001f
#define FIELD_EVOLUTION_MAX      0.080f
#define FIELD_EVOLUTION_FACTOR   1.4f          /* multiplier per F/f keypress */

/* Spatial scale of the noise (smaller = larger swirls).  Different
 * x/y scales make the flow gently anisotropic — looks like wind that
 * prefers horizontal direction. */
#define NOISE_SCALE_X            0.04f
#define NOISE_SCALE_Y            0.07f

/* Tracer lifetime in ticks before forced respawn (with jitter). */
#define TRACER_LIFE_BASE_TICKS   100
#define TRACER_LIFE_JITTER_TICKS  60

/* Time helpers. */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL

/* Color pair IDs.  Reserved blocks: */
enum {
    PAIR_TRAIL_BASE = 1,                              /* 1..HUE_WHEEL_PAIRS */
    PAIR_HUD        = PAIR_TRAIL_BASE + HUE_WHEEL_PAIRS,
    PAIR_HINT,
};

/* ===================================================================== */
/* §2  clock — monotonic ns timer                                        */
/* ===================================================================== */
/*
 * CLOCK_MONOTONIC + clock_gettime + nanosleep is the POSIX standard
 * for soft real-time animation.  Wall-clock changes (NTP, manual
 * adjust) don't perturb monotonic time.
 */

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
/* §3  rng — Fisher-Yates shuffle for the perlin permutation table       */
/* ===================================================================== */
/*
 * The perlin permutation table is the SINGLE source of pseudo-randomness
 * for the noise function.  We seed rand() once at startup and shuffle
 * a 256-byte identity array.  The shuffled array is then doubled to
 * 512 bytes so we can index without a modulo (perm[i & 255]).
 */

static uint8_t perlin_perm_table[512];

static void perlin_perm_init(void)
{
    uint8_t identity[256];
    for (int i = 0; i < 256; i++) identity[i] = (uint8_t)i;

    /* Fisher-Yates: walk from end to start, swap with random earlier. */
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t tmp     = identity[i];
        identity[i]     = identity[j];
        identity[j]     = tmp;
    }

    /* Double the table so perm[a + b] is always valid for a, b ∈ [0, 255]. */
    for (int i = 0; i < 512; i++)
        perlin_perm_table[i] = identity[i & 255];
}

/* ===================================================================== */
/* §4  perlin — single-octave 2-D Perlin gradient noise                  */
/* ===================================================================== */
/*
 * Computes a smooth pseudo-random scalar in [-1, +1] at any (x, y).
 * Adjacent calls give CORRELATED values (the whole point — T2).
 *
 * Implementation (Perlin 1985 / improved 2002, simplified):
 *
 *   1. Find the integer cell containing (x, y).
 *   2. At each of the 4 cell corners, compute a "gradient dot" — the
 *      dot product of a gradient vector (selected from the perm
 *      table by hashing the corner coords) with the offset from
 *      that corner to (x, y).
 *   3. Smoothstep-interpolate the 4 corner dots back to (x, y)
 *      (bilinear with a 3t² - 2t³ ease so first derivatives are
 *      continuous at corners).
 */

/* Smoothstep: 3t² - 2t³.  C¹ continuous; C² has a discontinuity at
 * t=0 and t=1, which is why some implementations use the "improved"
 * 6t⁵ - 15t⁴ + 10t³.  For visualisation 3t² - 2t³ is plenty. */
static inline float smoothstep_cubic(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

static inline float lerp_scalar(float a, float b, float t)
{
    return a + t * (b - a);
}

/* Gradient hash — turn an 8-bit hash into a unit-vector dot product
 * with the offset (x, y) to the corner.  Perlin's "improved" version
 * uses 4 vectors {(±1, 0), (0, ±1)} or 8 around the wheel; we use 4. */
static inline float perlin_gradient_dot(int hash, float x, float y)
{
    int   bits  = hash & 3;
    float term1 = (bits < 2) ? x : y;
    float term2 = (bits < 2) ? y : x;
    return ((hash & 1) ? -term1 : term1) + ((hash & 2) ? -term2 : term2);
}

/* perlin_value — smooth pseudo-random scalar in [-1, +1] at (x, y). */
static float perlin_value(float x, float y)
{
    /* Integer cell containing (x, y). */
    int xi = (int)floorf(x) & 255;
    int yi = (int)floorf(y) & 255;

    /* Fractional offsets within the cell. */
    float fx = x - floorf(x);
    float fy = y - floorf(y);

    /* Eased fractional offsets (smoothstep). */
    float ux = smoothstep_cubic(fx);
    float uy = smoothstep_cubic(fy);

    /* Hash the four corners to gradient indices. */
    int h00 = perlin_perm_table[perlin_perm_table[xi    ] + yi    ];
    int h10 = perlin_perm_table[perlin_perm_table[xi + 1] + yi    ];
    int h01 = perlin_perm_table[perlin_perm_table[xi    ] + yi + 1];
    int h11 = perlin_perm_table[perlin_perm_table[xi + 1] + yi + 1];

    /* Dot product at each corner of (gradient, offset-to-query). */
    float d00 = perlin_gradient_dot(h00, fx,        fy       );
    float d10 = perlin_gradient_dot(h10, fx - 1.0f, fy       );
    float d01 = perlin_gradient_dot(h01, fx,        fy - 1.0f);
    float d11 = perlin_gradient_dot(h11, fx - 1.0f, fy - 1.0f);

    /* Bilinear interpolation with the smoothstepped offsets. */
    float row_top    = lerp_scalar(d00, d10, ux);
    float row_bottom = lerp_scalar(d01, d11, ux);
    return lerp_scalar(row_top, row_bottom, uy);
}

/* ===================================================================== */
/* §5  fbm — fractional Brownian motion                                  */
/* ===================================================================== */
/*
 * Sum multiple Perlin octaves at increasing frequency / decreasing
 * amplitude.  See T4 for the motivation.  The result has visible
 * structure at every scale — exactly what real-world flow looks
 * like.
 *
 * Pseudocode:
 *
 *     fbm_value(x, y, octaves):
 *         result = 0;  amp = 1;  freq = 1
 *         for i in 0 .. octaves-1:
 *             result += perlin_value(x · freq, y · freq) · amp
 *             amp *= 0.5;  freq *= 2
 *         return result
 *
 * Result range: roughly [-2, +2] for 3 octaves with the 1, 0.5, 0.25
 * amplitude weights.  We don't normalise — the magnitude doesn't
 * matter because we use atan2 to extract the angle (T5).
 */

static float fbm_value(float x, float y, int octaves)
{
    float result    = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    for (int oct = 0; oct < octaves; oct++) {
        result    += perlin_value(x * frequency, y * frequency) * amplitude;
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }
    return result;
}

/* ===================================================================== */
/* §6  field — 2-D angle grid: build, sample, evolve                     */
/* ===================================================================== */
/*
 * The field stores ONE FLOAT per cell — the flow angle at that cell.
 * (Two floats per cell would let us also store a magnitude, but for
 * this visualisation all tracers move at unit speed, so we only need
 * the direction.)
 *
 * Two TIME-OFFSET fbm samples produce the (vx, vy) of the local
 * flow vector; atan2 turns it into an angle.  The "time" axis
 * advances slowly each tick so the pattern keeps changing.
 *
 * Static storage (no malloc): a 200×60 max grid uses 48 KB of
 * floats — tiny.
 */

#define FIELD_COLS_MAX 256
#define FIELD_ROWS_MAX  80

typedef struct {
    int   active_cols;
    int   active_rows;
    float time_axis;         /* seconds-ish along noise time */
    float evolution_speed;   /* per-tick advance of time_axis */
    float angle[FIELD_ROWS_MAX][FIELD_COLS_MAX];
} flow_field;

static void field_init(flow_field *f, int cols, int rows)
{
    if (cols > FIELD_COLS_MAX) cols = FIELD_COLS_MAX;
    if (rows > FIELD_ROWS_MAX) rows = FIELD_ROWS_MAX;
    f->active_cols     = cols;
    f->active_rows     = rows;
    f->time_axis       = 0.0f;
    f->evolution_speed = FIELD_EVOLUTION_DEFAULT;
    memset(f->angle, 0, sizeof f->angle);
}

/* field_evolve_and_rebuild — advance time, recompute every cell.
 *
 * Cost: rows × cols × OCTAVES Perlin evaluations.  At 200×60×3 =
 * 36 000 perlin_value calls per tick.  Sub-millisecond. */
static void field_evolve_and_rebuild(flow_field *f)
{
    f->time_axis += f->evolution_speed;
    float t = f->time_axis;
    for (int r = 0; r < f->active_rows; r++) {
        for (int c = 0; c < f->active_cols; c++) {
            float vx = fbm_value(c * NOISE_SCALE_X +         t,
                                 r * NOISE_SCALE_Y + t * 0.7f,
                                 FBM_OCTAVES);
            float vy = fbm_value(c * NOISE_SCALE_X + 100.3f + t * 1.1f,
                                 r * NOISE_SCALE_Y + 200.7f + t * 0.5f,
                                 FBM_OCTAVES);
            f->angle[r][c] = atan2f(vy, vx);
        }
    }
}

/* flow_angle_at_cell — exact angle at integer cell.  Used by the
 * field-arrow background renderer. */
static inline float flow_angle_at_cell(const flow_field *f, int c, int r)
{
    if (c < 0)                    c = 0;
    if (c >= f->active_cols)      c = f->active_cols - 1;
    if (r < 0)                    r = 0;
    if (r >= f->active_rows)      r = f->active_rows - 1;
    return f->angle[r][c];
}

/* flow_angle_bilinear — interpolated angle at sub-cell position.
 *
 * NOTE: bilinearly interpolating angles across the −π↔+π wrap is
 * technically wrong (you'd want to interpolate (cos, sin) pairs and
 * re-atan2).  For visualisation the seam is invisible because Perlin
 * values change slowly across cells; the wrap rarely happens between
 * neighbours. */
static float flow_angle_bilinear(const flow_field *f, float col, float row)
{
    int c0 = (int)floorf(col);
    int r0 = (int)floorf(row);
    int c1 = c0 + 1;
    int r1 = r0 + 1;

    if (c0 < 0)                c0 = 0;
    if (c0 >= f->active_cols)  c0 = f->active_cols - 1;
    if (c1 < 0)                c1 = 0;
    if (c1 >= f->active_cols)  c1 = f->active_cols - 1;
    if (r0 < 0)                r0 = 0;
    if (r0 >= f->active_rows)  r0 = f->active_rows - 1;
    if (r1 < 0)                r1 = 0;
    if (r1 >= f->active_rows)  r1 = f->active_rows - 1;

    float fx = col - floorf(col);
    float fy = row - floorf(row);

    float a00 = f->angle[r0][c0];
    float a10 = f->angle[r0][c1];
    float a01 = f->angle[r1][c0];
    float a11 = f->angle[r1][c1];

    float row_top    = lerp_scalar(a00, a10, fx);
    float row_bottom = lerp_scalar(a01, a11, fx);
    return lerp_scalar(row_top, row_bottom, fy);
}

/* ===================================================================== */
/* §7  arrows — direction → ASCII glyph + hue index                      */
/* ===================================================================== */
/*
 * Map a flow angle (in radians) to:
 *   - one of 8 ASCII arrow glyphs (for the field-arrow overlay)
 *   - one of 8 hue pair indices (for the rainbow theme)
 *
 * The mapping divides the full circle into 8 octants, each π/4 wide.
 * Octant 0 starts at the EAST (angle = 0) and increments clockwise
 * in screen space (which is counter-clockwise in math coords because
 * y points down on screens — matches the physical convention "wind
 * from east" = arrow pointing east).
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │           octant 2 (N, '^')                      │
 *      │                                                  │
 *      │   octant 3   ↖    │    ↗   octant 1              │
 *      │   (NW, '\')  '\'  │  '/'   (NE, '/')             │
 *      │                                                  │
 *      │   octant 4 ─ '<' ─ ●  ─ '>' ─ octant 0 (E, '>')  │
 *      │                                                  │
 *      │   octant 5   ↙    │    ↘   octant 7              │
 *      │   (SW, '/')  '/'  │  '\'   (SE, '\')             │
 *      │                                                  │
 *      │           octant 6 (S, 'v')                      │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 */

#define ARROW_TABLE_LEN 8
static const char arrow_glyph_table[ARROW_TABLE_LEN] = {
    '>',   /* 0  E   */
    '/',   /* 1  NE  */
    '^',   /* 2  N   */
    '\\',  /* 3  NW  */
    '<',   /* 4  W   */
    '/',   /* 5  SW  */
    'v',   /* 6  S   */
    '\\',  /* 7  SE  */
};

static int angle_to_octant(float angle_radians)
{
    float a = angle_radians;
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    int octant = (int)(a / (2.0f * (float)M_PI) * ARROW_TABLE_LEN + 0.5f)
               % ARROW_TABLE_LEN;
    return octant;
}

static inline char arrow_glyph_for_angle(float angle_radians)
{
    return arrow_glyph_table[angle_to_octant(angle_radians)];
}

static inline int hue_pair_for_angle(float angle_radians)
{
    return PAIR_TRAIL_BASE + angle_to_octant(angle_radians);
}

/* ===================================================================== */
/* §8  themes — rainbow + 3 mono palettes (data only)                    */
/* ===================================================================== */
/*
 * All theme data lives in TABLES so adding a new theme is just one
 * row of numbers.  Palettes target 256-colour terminals; the
 * 8-colour fallback is a separate table.
 *
 * RAINBOW: the 8 hue indices map to an evenly-spaced rainbow.
 * MONO:    all 8 indices map to one hue with brightness ramps,
 *          giving a "trail fade" effect within each particle's
 *          lifetime.
 *
 * Pair index 1 is BRIGHTEST (fresh trail / particle head).
 * Pair index 8 is DIMMEST   (oldest trail cell).
 */

static const char *theme_name_table[THEME_COUNT] = {
    "rainbow", "cyan", "green", "white"
};

static const int theme_palette_256[THEME_COUNT][HUE_WHEEL_PAIRS] = {
    /* RAINBOW — 8 hues around the wheel (matches octants in §7). */
    { 196, 208, 226,  46,  51,  33, 129, 201 },
    /* CYAN ramp — bright → dim. */
    {  51,  45,  39,  33,  27,  26,  25,  25 },
    /* GREEN ramp — bright → dim. */
    {  82,  46,  40,  34,  28,  28,  28,  28 },
    /* WHITE / GREY ramp — bright → dim. */
    { 255, 250, 247, 245, 243, 241, 240, 240 },
};

static const int theme_palette_8[THEME_COUNT][HUE_WHEEL_PAIRS] = {
    { COLOR_RED, COLOR_YELLOW, COLOR_GREEN, COLOR_CYAN,
      COLOR_BLUE, COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE },
    { COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
      COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE },
    { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN },
    { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
};

/* ===================================================================== */
/* §9  colors — ncurses pair init                                        */
/* ===================================================================== */
/*
 * Per CLAUDE.md HUD spec:
 *   PAIR_HUD  = bright yellow on default bg, used with A_BOLD.
 *   PAIR_HINT = bright cyan   on default bg, used with A_BOLD.
 * Other pairs use default bg too (-1) so the user's terminal
 * background shows through.
 */

static void colors_init(int theme_index)
{
    start_color();
    use_default_colors();

    bool has_256 = (COLORS >= 256);
    const int *palette = has_256
        ? theme_palette_256[theme_index]
        : theme_palette_8  [theme_index];

    for (int i = 0; i < HUE_WHEEL_PAIRS; i++)
        init_pair(PAIR_TRAIL_BASE + i, palette[i], -1);

    if (has_256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §10  tracer — one particle: pose + trail ring buffer                  */
/* ===================================================================== */
/*
 * State of one tracer:
 *
 *   tracer_alive            slot in use?
 *   pos_col, pos_row        continuous (x, y) in grid units
 *   step_cells              per-tracer speed (cells/tick) with jitter
 *   last_angle              flow angle from the most recent tick
 *   ticks_until_respawn     lifetime countdown
 *   trail_pair_base         brightest colour pair (rainbow updates per tick)
 *   trail_col[i], trail_row[i]    ring buffer of past integer positions
 *   trail_write_index       next slot to write
 *   trail_filled_count      how many slots are populated (≤ active_length)
 *   trail_active_length     current length of the trail (for s/S keys)
 */

typedef struct {
    bool   tracer_alive;
    float  pos_col;
    float  pos_row;
    float  step_cells;
    float  last_angle;
    int    ticks_until_respawn;
    int    trail_pair_base;
    int    trail_col[TRAIL_LEN_HARD_MAX];
    int    trail_row[TRAIL_LEN_HARD_MAX];
    int    trail_write_index;
    int    trail_filled_count;
    int    trail_active_length;
} tracer;

/* tracer_respawn — give a tracer fresh position + speed + lifetime.
 * Called at init, when a tracer dies, and when the user presses 'r'. */
static void tracer_respawn(tracer *t, int active_cols, int active_rows,
                           int trail_active_length)
{
    t->tracer_alive        = true;
    t->pos_col             = (float)(rand() % active_cols);
    t->pos_row             = (float)(rand() % active_rows);
    /* Per-tracer step in [BASE - JITTER/2, BASE + JITTER/2]. */
    t->step_cells          = TRACER_STEP_BASE_CPT
                           - TRACER_STEP_JITTER_CPT * 0.5f
                           + TRACER_STEP_JITTER_CPT
                             * ((float)rand() / (float)RAND_MAX);
    t->last_angle          = 0.0f;
    t->ticks_until_respawn = TRACER_LIFE_BASE_TICKS
                           + rand() % TRACER_LIFE_JITTER_TICKS;
    t->trail_pair_base     = 1 + rand() % HUE_WHEEL_PAIRS;
    t->trail_write_index   = 0;
    t->trail_filled_count  = 0;
    t->trail_active_length = trail_active_length;
    /* Pre-fill trail with the spawn position so the first frame
     * doesn't draw a gap. */
    for (int i = 0; i < TRAIL_LEN_HARD_MAX; i++) {
        t->trail_col[i] = (int)t->pos_col;
        t->trail_row[i] = (int)t->pos_row;
    }
}

/* ===================================================================== */
/* §11  tracer_step — one tick of one tracer                             */
/* ===================================================================== */
/*
 * THE HEART OF THE SIMULATION.  Six lines of physics:
 *
 *   1. Push current position into the trail ring buffer.
 *   2. Sample the field angle at the tracer's position.
 *   3. Step in that direction at the tracer's per-particle speed.
 *   4. Wrap toroidally if the tracer left the grid.
 *   5. Update colour (rainbow theme tracks the angle).
 *   6. Decrement lifetime; mark dead when expired.
 *
 * Implements forward-Euler advection (T6).
 */

static void tracer_advance_one_tick(tracer *t, const flow_field *f,
                                    int active_cols, int active_rows,
                                    int theme_index)
{
    if (!t->tracer_alive) return;

    /* 1. Record current position in the ring buffer. */
    t->trail_col[t->trail_write_index] = (int)t->pos_col;
    t->trail_row[t->trail_write_index] = (int)t->pos_row;
    t->trail_write_index = (t->trail_write_index + 1) % t->trail_active_length;
    if (t->trail_filled_count < t->trail_active_length)
        t->trail_filled_count++;

    /* 2. Sample the flow angle (bilinear). */
    float angle = flow_angle_bilinear(f, t->pos_col, t->pos_row);
    t->last_angle = angle;

    /* 3. Step in the flow direction. */
    t->pos_col += cosf(angle) * t->step_cells;
    t->pos_row += sinf(angle) * t->step_cells;

    /* 4. Toroidal wrap. */
    if (t->pos_col < 0.0f)                  t->pos_col += (float)active_cols;
    if (t->pos_col >= (float)active_cols)   t->pos_col -= (float)active_cols;
    if (t->pos_row < 0.0f)                  t->pos_row += (float)active_rows;
    if (t->pos_row >= (float)active_rows)   t->pos_row -= (float)active_rows;

    /* 5. Rainbow theme: hue follows angle.  Mono themes keep
     *    the spawn-time pair; the brightness ramp at draw time
     *    creates the trail fade. */
    if (theme_index == 0)
        t->trail_pair_base = hue_pair_for_angle(angle);

    /* 6. Age. */
    t->ticks_until_respawn--;
    if (t->ticks_until_respawn <= 0) t->tracer_alive = false;
}

/* ===================================================================== */
/* §12  tracer_paint — fading-trail render                               */
/* ===================================================================== */
/*
 * Walk the trail ring buffer from OLDEST to NEWEST (so the head
 * draws LAST and wins overlaps).  Pick a glyph based on age and a
 * colour pair that fades from `trail_pair_base` at the head down to
 * pair 1 at the tail tip.  The head gets the directional arrow
 * glyph (T7-T6 connection — visual proof of the angle).
 */

#define TRAIL_RAMP_LEN 5
static const char trail_ramp_glyph[TRAIL_RAMP_LEN] = {
    '.', ',', '+', '~', '*'
};

/* Direction glyph for the head — picks the same 8-octant arrow as §7. */
static char trail_head_glyph_for_angle(float angle_radians)
{
    static const char head_dir_glyph[ARROW_TABLE_LEN] = {
        '-', '/', '|', '\\', '-', '/', '|', '\\'
    };
    return head_dir_glyph[angle_to_octant(angle_radians)];
}

static void tracer_paint(const tracer *t, WINDOW *win,
                         int active_cols, int active_rows)
{
    if (!t->tracer_alive)            return;
    if (t->trail_filled_count == 0)  return;

    int filled = t->trail_filled_count;
    int len    = t->trail_active_length;

    for (int i = 0; i < filled; i++) {
        /* OLDEST entry is at (write_index - filled + i) modulo length.
         * The "+ 2*len" before mod handles negative dividends — C's
         * % returns negative for negative left operand. */
        int slot = (t->trail_write_index - filled + i + 2 * len) % len;
        int col  = t->trail_col[slot];
        int row  = t->trail_row[slot];

        if (col < 0 || col >= active_cols) continue;
        if (row < 0 || row >= active_rows) continue;

        bool is_head = (i == filled - 1);

        /* Glyph: head gets a direction arrow; trail uses age ramp. */
        char glyph;
        if (is_head) {
            glyph = trail_head_glyph_for_angle(t->last_angle);
        } else {
            int ramp_index = (i * (TRAIL_RAMP_LEN - 1))
                           / (filled > 1 ? filled - 1 : 1);
            if (ramp_index >= TRAIL_RAMP_LEN - 1)
                ramp_index = TRAIL_RAMP_LEN - 2;   /* head's '*' reserved */
            glyph = trail_ramp_glyph[ramp_index];
        }

        /* Colour: head = base; tail tip = pair 1; linear in between. */
        int pair = is_head
            ? t->trail_pair_base
            : t->trail_pair_base
              - ((filled - 1 - i) * (t->trail_pair_base - 1))
                / (filled > 1 ? filled - 1 : 1);
        if (pair < 1)                  pair = 1;
        if (pair > HUE_WHEEL_PAIRS)    pair = HUE_WHEEL_PAIRS;

        attr_t a = COLOR_PAIR(pair);
        if (is_head) a |= A_BOLD;

        wattron(win, a);
        mvwaddch(win, row, col, (chtype)(unsigned char)glyph);
        wattroff(win, a);
    }
}

/* ===================================================================== */
/* §13  scene — pool + field + tick orchestrator                         */
/* ===================================================================== */

typedef struct {
    flow_field flow;
    tracer     pool[TRACERS_MAX];
    int        active_tracer_count;
    int        trail_active_length;
    int        theme_index;
    bool       show_field_arrows;
    bool       paused;
} scene_state;

static void scene_init(scene_state *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->active_tracer_count = TRACERS_DEFAULT;
    s->trail_active_length = TRAIL_LEN_DEFAULT;
    s->theme_index         = 0;
    s->show_field_arrows   = false;
    s->paused              = false;

    field_init(&s->flow, cols, rows);
    field_evolve_and_rebuild(&s->flow);

    for (int i = 0; i < s->active_tracer_count; i++)
        tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_resize(scene_state *s, int cols, int rows)
{
    field_init(&s->flow, cols, rows);
    field_evolve_and_rebuild(&s->flow);
    for (int i = 0; i < TRACERS_MAX; i++)
        if (s->pool[i].tracer_alive)
            tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_respawn_all(scene_state *s, int cols, int rows)
{
    for (int i = 0; i < s->active_tracer_count; i++)
        tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
    for (int i = s->active_tracer_count; i < TRACERS_MAX; i++)
        s->pool[i].tracer_alive = false;
}

static void scene_set_trail_length(scene_state *s, int new_length)
{
    if (new_length < TRAIL_LEN_MIN) new_length = TRAIL_LEN_MIN;
    if (new_length > TRAIL_LEN_MAX) new_length = TRAIL_LEN_MAX;
    s->trail_active_length = new_length;
    for (int i = 0; i < TRACERS_MAX; i++)
        s->pool[i].trail_active_length = new_length;
}

static void scene_tick(scene_state *s, int cols, int rows)
{
    if (s->paused) return;

    field_evolve_and_rebuild(&s->flow);

    for (int i = 0; i < s->active_tracer_count; i++) {
        if (!s->pool[i].tracer_alive)
            tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
        tracer_advance_one_tick(&s->pool[i], &s->flow,
                                cols, rows, s->theme_index);
    }
}

/* Paint the field-arrow background (only when 'a' is toggled on).
 * Each cell gets its arrow glyph in the rainbow-mapped colour, NOT
 * dimmed (per CLAUDE.md anti-A_DIM rule).  Using a normal-weight
 * arrow keeps it visible without competing with bold tracer heads. */
static void scene_paint_field_arrows(const scene_state *s, WINDOW *win,
                                     int cols, int rows)
{
    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            float angle = flow_angle_at_cell(&s->flow, c, r);
            char  glyph = arrow_glyph_for_angle(angle);
            int   pair  = (s->theme_index == 0)
                        ? hue_pair_for_angle(angle)
                        : 1;
            wattron(win, COLOR_PAIR(pair));
            mvwaddch(win, r, c, (chtype)(unsigned char)glyph);
            wattroff(win, COLOR_PAIR(pair));
        }
    }
}

static void scene_paint(const scene_state *s, WINDOW *win, int cols, int rows)
{
    if (s->show_field_arrows)
        scene_paint_field_arrows(s, win, cols, rows);

    for (int i = 0; i < s->active_tracer_count; i++)
        tracer_paint(&s->pool[i], win, cols, rows);
}

/* ===================================================================== */
/* §14  hud — top status + bottom hint strip (CLAUDE.md spec)            */
/* ===================================================================== */

static void hud_paint_status(WINDOW *win, int cols,
                             double fps, int sim_hz,
                             const scene_state *s)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%2dHz  tracers:%3d  trail:%2d  "
             "theme:%-7s  field-spd:%.4f  %s ",
             fps, sim_hz, s->active_tracer_count, s->trail_active_length,
             theme_name_table[s->theme_index],
             (double)s->flow.evolution_speed,
             s->paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    int x   = cols - len;
    if (x < 0) x = 0;
    wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvwprintw(win, 0, x, "%s", buf);
    wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hints(WINDOW *win, int rows)
{
    const char *hint =
        " q:quit  spc:pause  r:respawn  a:arrows  t:theme  +/-:tracers  "
        "s/S:trail  ]/[:simHz  f/F:field-spd ";
    wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvwprintw(win, rows - 1, 0, "%s", hint);
    wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §15  screen — ncurses init / cleanup                                  */
/* ===================================================================== */

static void screen_init(int theme_index)
{
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    colors_init(theme_index);
}

static void screen_cleanup(void)
{
    endwin();
}

/* ===================================================================== */
/* §16  app — main loop + signals + input                                */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit = 1;
}

/* Returns false if the user wants to quit. */
static bool app_handle_key(int key, scene_state *s, int *sim_hz,
                           int cols, int rows)
{
    switch (key) {
        case 'q': case 'Q': case 27:    /* 27 = ESC */
            return false;

        case ' ':
            s->paused = !s->paused;
            break;

        case 'r': case 'R':
            scene_respawn_all(s, cols, rows);
            break;

        case 'a': case 'A':
            s->show_field_arrows = !s->show_field_arrows;
            break;

        case 't': case 'T':
            s->theme_index = (s->theme_index + 1) % THEME_COUNT;
            colors_init(s->theme_index);
            break;

        case '+': case '=':
            if (s->active_tracer_count + TRACERS_STEP <= TRACERS_MAX) {
                int old_count = s->active_tracer_count;
                s->active_tracer_count += TRACERS_STEP;
                for (int i = old_count; i < s->active_tracer_count; i++)
                    tracer_respawn(&s->pool[i], cols, rows,
                                   s->trail_active_length);
            }
            break;
        case '-': case '_':
            if (s->active_tracer_count - TRACERS_STEP >= TRACERS_MIN) {
                s->active_tracer_count -= TRACERS_STEP;
                for (int i = s->active_tracer_count;
                     i < s->active_tracer_count + TRACERS_STEP; i++)
                    s->pool[i].tracer_alive = false;
            }
            break;

        case ']':
            if (*sim_hz + SIM_HZ_STEP <= SIM_HZ_MAX) *sim_hz += SIM_HZ_STEP;
            break;
        case '[':
            if (*sim_hz - SIM_HZ_STEP >= SIM_HZ_MIN) *sim_hz -= SIM_HZ_STEP;
            break;

        case 'f':
            s->flow.evolution_speed *= FIELD_EVOLUTION_FACTOR;
            if (s->flow.evolution_speed > FIELD_EVOLUTION_MAX)
                s->flow.evolution_speed = FIELD_EVOLUTION_MAX;
            break;
        case 'F':
            s->flow.evolution_speed /= FIELD_EVOLUTION_FACTOR;
            if (s->flow.evolution_speed < FIELD_EVOLUTION_MIN)
                s->flow.evolution_speed = FIELD_EVOLUTION_MIN;
            break;

        case 's':
            scene_set_trail_length(s, s->trail_active_length + 1);
            break;
        case 'S':
            scene_set_trail_length(s, s->trail_active_length - 1);
            break;

        default:
            break;
    }
    return true;
}

int main(void)
{
    /* Seed PRNG from monotonic time, then build the perlin permutation. */
    srand((unsigned int)clock_now_ns());
    perlin_perm_init();

    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    static scene_state scene;
    int                sim_hz = SIM_HZ_DEFAULT;

    screen_init(0);
    atexit(screen_cleanup);

    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    scene_init(&scene, cols, rows);

    /* Fixed-step accumulator (Glenn Fiedler "Fix Your Timestep!"). */
    int64_t prev_ns      = clock_now_ns();
    int64_t sim_accum_ns = 0;

    /* Sliding-window FPS counter. */
    int     frames_in_window = 0;
    int64_t window_accum_ns  = 0;
    double  fps_display      = 0.0;

    const int64_t frame_cap_ns = NS_PER_SEC / TARGET_FPS_HZ;

    while (!g_should_quit) {
        int64_t frame_start = clock_now_ns();

        /* Resize handling (signal-driven, processed at frame top). */
        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&scene, cols, rows);
            sim_accum_ns = 0;
        }

        /* dt + spiral-of-death guard (cap at 100 ms). */
        int64_t dt_ns = frame_start - prev_ns;
        prev_ns       = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;

        /* Drive simulation at fixed sim_hz, regardless of render rate. */
        int64_t tick_ns = NS_PER_SEC / sim_hz;
        sim_accum_ns += dt_ns;
        while (sim_accum_ns >= tick_ns) {
            scene_tick(&scene, cols, rows);
            sim_accum_ns -= tick_ns;
        }

        /* Draw frame. */
        erase();
        scene_paint(&scene, stdscr, cols, rows);
        hud_paint_status(stdscr, cols, fps_display, sim_hz, &scene);
        hud_paint_hints (stdscr, rows);
        wnoutrefresh(stdscr);
        doupdate();

        /* FPS readout — refresh every ~500 ms. */
        frames_in_window++;
        window_accum_ns += dt_ns;
        if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
            fps_display = (double)frames_in_window
                        / ((double)window_accum_ns / (double)NS_PER_SEC);
            frames_in_window = 0;
            window_accum_ns  = 0;
        }

        /* Drain input queue. */
        int key;
        while ((key = getch()) != ERR) {
            if (!app_handle_key(key, &scene, &sim_hz, cols, rows)) {
                g_should_quit = 1;
                break;
            }
        }

        /* Sleep to cap render rate. */
        int64_t spent = clock_now_ns() - frame_start;
        if (spent < frame_cap_ns) clock_sleep_ns(frame_cap_ns - spent);
    }

    return 0;
}
