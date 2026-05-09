/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * complex_flowfield.c — four physics models in one tracer engine
 *
 * DEMO: The "richer cousin" of fluid/flowfield.c.  Same particle-tracer
 *       loop, but the WIND IS GENERATED IN FOUR DIFFERENT WAYS.  Press
 *       'a' to swap between:
 *
 *           CURL NOISE      (T3) — divergence-free Perlin turbulence;
 *                                   particles never accumulate; looks
 *                                   like real fluid swirls.
 *           VORTEX LATTICE  (T4) — N point vortices arranged on a
 *                                   ring (Biot-Savart math); particles
 *                                   spiral around them.
 *           SINE LATTICE    (T5) — superposed travelling sine waves;
 *                                   crisscross interference patterns.
 *           RADIAL SPIRAL   (T6) — polar tangential flow + breathing
 *                                   radial pulse; "galaxy".
 *
 *       The COSINE PALETTE (Inigo Quilez, T7) gives six perceptually-
 *       smooth colour themes; particles' hues come from their movement
 *       angle so adjacent directions get complementary colours.  Three
 *       BACKGROUND MODES (blank / arrows / colormap) reveal the field
 *       at increasing detail.
 *
 *       The architectural lesson (T2) is that ALL four modes plug into
 *       the SAME tracer engine via one function pointer.  Adding a
 *       fifth field generator is one new function plus one row in a
 *       table — the rest of the file doesn't change.
 *
 * Study alongside:
 *   fluid/flowfield.c                    — the simpler, single-mode
 *                                            cousin (Perlin only)
 *   procedural/fields/curl_noise_vector_field.c
 *                                          — curl noise alone, deeper
 *                                            visual treatment
 *   physics/magnetic_field.c             — Biot-Savart in EM context
 *   fluid/navier_stokes.c                — actual fluid solver, not
 *                                            field lookup
 *
 * Section map:
 *   §1  config           — every constant grouped by subsystem
 *   §2  clock            — monotonic ns time + sleep
 *   §3  rng              — perlin permutation table seed
 *   §4  perlin           — single-octave 2-D Perlin noise
 *   §5  fbm              — multi-octave fractional Brownian motion
 *   §6  cosine_palette   — Inigo Quilez palette + 6 themes
 *   §7  colors           — pair init + angle → palette pair
 *   §8  field            — angle grid + bilinear sample + dispatch
 *   §9  field_curl       — divergence-free turbulence (CURL of noise)
 *   §10 field_vortex     — Biot-Savart point vortices on a rotating ring
 *   §11 field_sine       — superposed sinusoidal waves
 *   §12 field_spiral     — polar tangential + radial pulse
 *   §13 arrows           — angle → 8-octant ASCII arrow glyph
 *   §14 tracer           — one particle: pose + trail ring buffer
 *   §15 tracer_step      — Lagrangian advection (one tick)
 *   §16 tracer_paint     — fading-trail rendering
 *   §17 scene            — pool + field + tick orchestrator
 *   §18 hud              — top status + bottom hint strip
 *   §19 screen           — ncurses init / cleanup
 *   §20 app              — main loop + signals + input
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space            pause / resume
 *   r                reset (respawn tracers, rewind field time)
 *   a                next field type      (curl → vortex → sine → spiral)
 *   t                next colour theme    (cosmic → ember → ocean → ...)
 *   v                next background mode (blank → arrows → colormap)
 *   ] / [            sim Hz up / down
 *   + / -            more / fewer tracers
 *   f / F            field evolves faster / slower
 *   s / S            longer / shorter trail
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/complex_flowfield.c \
 *       -o complex_flowfield -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      T2 explains the architectural trick (function-pointer dispatch
 *      to four physics models).  T3-T6 each derive one of the
 *      four field generators in plain English.  T7 is the cosine
 *      palette.  T8 covers aspect-ratio gotchas in vortex / spiral.
 *   2. fluid/flowfield.c first — that file's tutorials T1-T6
 *      establish particles + Perlin + advection.  This file
 *      ASSUMES those concepts.
 *   3. §8 field — the angle grid + bilinear sampler + dispatcher.
 *      Read AFTER tutorials T1-T2.
 *   4. §9-§12 field_* — four field generators, each ~30 lines.
 *      Read each AFTER the matching tutorial (T3 → §9, T4 → §10,
 *      T5 → §11, T6 → §12).
 *   5. §15 tracer_step — six lines of Lagrangian advection.
 *   6. §17 scene + §20 app — orchestration; standard framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   tracer.pos_col, tracer.pos_row    continuous (x, y) position
 *   tracer.step_cells                  per-particle speed (cells/tick)
 *   tracer.last_angle                  flow angle from most recent tick
 *   tracer.trail_col[], trail_row[]    ring buffer of past int positions
 *   tracer.trail_write_index           next slot to write
 *   tracer.trail_filled_count          how many slots populated
 *   tracer.trail_active_length         current active length
 *   tracer.trail_pair_id               colour pair (palette index)
 *   tracer.ticks_until_respawn         lifetime countdown
 *
 *   flow_field.angle[r][c]             flow direction at integer cell
 *   flow_field.time_axis               noise/wave time variable
 *   flow_field.evolution_speed         per-tick advance of time_axis
 *   flow_field.active_kind             which of FIELD_KIND_* is live
 *
 *   vortex_pos_col[i], vortex_pos_row[i]
 *                                       i-th vortex centre on the ring
 *   vortex_ring_phase                   ring rotation angle (radians)
 *   vortex_strength_table[i]            per-vortex Γ (alternating signs)
 *
 *   palette_param_t                     a value ∈ [0, 1] that indexes
 *                                       the cosine palette
 *
 * Background you need
 * ───────────────────
 *   - fluid/flowfield.c T1-T6 (vector fields, Perlin, advection).
 *   - atan2(y, x) for converting (vx, vy) to a single angle.
 *   - Polar coordinates (r, θ) for spiral / vortex math.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real fluid mechanics.  Curl noise FAKES divergence-free flow;
 *     it doesn't solve any conservation law.  See fluid/navier_stokes.c
 *     for the real thing.
 *   - Maxwell's equations.  Biot-Savart is borrowed from EM but used
 *     here as a 2-D vortex-velocity formula, not an electromagnetic
 *     simulation.  See physics/magnetic_field.c for the EM version.
 *   - Colour theory.  Cosine palette gives perceptually-smooth
 *     gradients with four numbers per channel; deeper colour-space
 *     theory not required.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : One tracer engine + four field generators behind a
 *                 function-pointer dispatcher.  Each tick:
 *
 *                   1. ADVANCE THE FIELD (call the active generator
 *                      for every grid cell, write into a 2-D angle
 *                      array).
 *                   2. ADVECT EACH TRACER one step using the
 *                      bilinearly-interpolated angle at its position.
 *                   3. RENDER:  optional background (arrows / colormap)
 *                      followed by tracer trails (painters' order).
 *
 * The four field generators:
 *
 *   CURL NOISE (§9)      Sample a scalar potential ψ(x, y, t) from
 *                          multi-octave Perlin noise.  The 2-D curl
 *                          (∂ψ/∂y, -∂ψ/∂x) is automatically
 *                          DIVERGENCE-FREE — no sources, no sinks.
 *                          Looks like turbulent fluid.
 *
 *   VORTEX LATTICE (§10) N point vortices on a slowly-rotating ring;
 *                          velocity at (x, y) is the Biot-Savart
 *                          superposition of all N contributions:
 *                              v = Σ Γᵢ · (-dy, dx) / (r² + ε)
 *                          where dᵢ is the offset to vortex i.  Looks
 *                          like a galaxy of orbiting eyes.
 *
 *   SINE LATTICE (§11)   Velocity components are simple sums of
 *                          travelling sinusoids.  Standing-wave
 *                          interference creates regular crisscross
 *                          patterns.
 *
 *   RADIAL SPIRAL (§12)  In polar coords relative to screen centre:
 *                          tangential unit vector + time-pulsing
 *                          radial component.  Looks like a breathing
 *                          galaxy.
 *
 * Cosine palette (§6)   Inigo Quilez's formula:
 *
 *                          color(t) = a + b · cos(2π · (c·t + d))
 *
 *                       with a, b, c, d as RGB 3-vectors.  Adjusting
 *                       (a, b, c, d) gives perceptually smooth
 *                       gradients with full hue rotation.  16 evenly-
 *                       spaced palette samples are pre-baked into
 *                       ncurses pairs at theme-switch time.
 *
 * Data layout   : Field state is one float-per-cell angle grid (max
 *                 256×80) plus per-mode auxiliary state (vortex
 *                 positions, etc.).  Tracer pool is a fixed-size
 *                 array of structs; each tracer owns its trail ring
 *                 buffer.  No malloc anywhere after init.
 *
 * Performance   : O(rows · cols) per field rebuild + O(N · trail) for
 *                 rendering.  At 200 × 60 grid, 400 tracers, trail 18:
 *                 ~12 k field samples + ~7 k cell paints per tick.
 *                 Sub-millisecond.
 *
 * References
 * ──────────
 *   Inigo Quilez, "Palettes":
 *     https://iquilezles.org/articles/palettes/
 *   Inigo Quilez, "Curl noise":
 *     https://iquilezles.org/articles/warp/
 *   Bridson, Hourihan & Nordenstam, "Curl-noise for procedural
 *     fluid flow" (SIGGRAPH 2007) — the original curl-noise paper.
 *   Saffman, "Vortex Dynamics" (Cambridge, 1992) — point-vortex
 *     theory + Biot-Savart in fluid context.
 *   Perlin, "Improving noise" (SIGGRAPH 2002) — Perlin noise refresh.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The "wind" can be generated MANY DIFFERENT WAYS — Perlin noise,
 * spinning vortices, intersecting waves, polar swirls.  The code
 * treats each as a PLUGGABLE BLACK BOX: given (x, y, t), return an
 * angle.  The tracer engine doesn't care which box is active; it
 * just calls "give me the angle at this position" and walks
 * accordingly.  Swapping fields at runtime swaps the visual
 * personality without touching the tracer code.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine four artists, each with their own paint style, taking
 * turns inflating one shared balloon — the balloon's shape (the
 * tracers) stays the same; only the paint pattern changes.  An
 * abstract painter does Perlin curl swirls.  A surrealist arranges
 * eddying point vortices.  A modernist lays down sine-wave grids.
 * A fantasy artist paints galactic spirals.  Same canvas, four
 * styles.
 *
 * ARCHITECTURE — FUNCTION-POINTER DISPATCH
 * ────────────────────────────────────────
 *
 *      ┌───────────────────────────────────────────────────────────┐
 *      │                                                           │
 *      │   TRACER ENGINE        FIELD DISPATCHER                   │
 *      │                                                           │
 *      │   for each tracer t:                                      │
 *      │     angle = field_sample(field, t.x, t.y)  ◄──┐           │
 *      │     t.x += cos(angle) * t.step                │           │
 *      │     t.y += sin(angle) * t.step                │           │
 *      │     ...                                       │           │
 *      │                                               │           │
 *      │                                       calls into:         │
 *      │                                                           │
 *      │              ┌──────────────────────────────────┐         │
 *      │              │  field->kind ?                    │         │
 *      │              │    CURL    → field_curl_angle    │         │
 *      │              │    VORTEX  → field_vortex_angle  │         │
 *      │              │    SINE    → field_sine_angle    │         │
 *      │              │    SPIRAL  → field_spiral_angle  │         │
 *      │              └──────────────────────────────────┘         │
 *      │                                                           │
 *      │                                                           │
 *      │   The tracer doesn't care which case fires.              │
 *      │   Each generator is a SELF-CONTAINED function.            │
 *      │                                                           │
 *      └───────────────────────────────────────────────────────────┘
 *
 * Adding a 5th field generator:
 *   1. Write a `field_NEW_angle(field, x, y, t)` function in §13ish.
 *   2. Add a `FIELD_KIND_NEW` enum value.
 *   3. Add one case in `field_evolve_and_rebuild`'s switch.
 *   That's it.  No tracer / scene / HUD code touched.
 *
 * KEY FORMULAS
 * ────────────
 *
 *   CURL NOISE — divergence-free velocity from a scalar potential ψ:
 *     vx =  ∂ψ/∂y
 *     vy = -∂ψ/∂x
 *     ψ  = multi-octave Perlin noise
 *     ∂ψ/∂y ≈ (ψ(x, y+ε) - ψ(x, y-ε)) / (2ε)    central differences
 *
 *   BIOT-SAVART — velocity from a single point vortex at origin
 *     with strength Γ:
 *     vx = Γ · (-y) / (r² + ε)
 *     vy = Γ ·   x  / (r² + ε)
 *     where r² = x² + y² and ε is a softening to avoid singularity
 *     at the vortex centre.
 *
 *     With N vortices at positions cᵢ, strengths Γᵢ:
 *     v(p) = Σ  Γᵢ · ((-(p.y - cᵢ.y), p.x - cᵢ.x)) / (|p - cᵢ|² + ε)
 *
 *   SINE LATTICE — direct sum of sinusoids:
 *     vx = sin(x · fx + t) + sin(y · fy - 0.7 t)
 *     vy = cos(x · fx - 0.5 t) + cos(y · fy + 0.3 t)
 *
 *   RADIAL SPIRAL — polar tangential + pulsing radial:
 *     θ  = atan2(dy, dx) where (dx, dy) = position relative to centre
 *     vx = -sin θ + W · sin(t · 0.8) · cos θ
 *     vy =  cos θ + W · sin(t · 0.8) · sin θ
 *     The first term is pure tangential (counter-clockwise); the W
 *     term is a time-modulated radial component that breathes in/out.
 *
 *   COSINE PALETTE (Inigo Quilez):
 *     color(t) = a + b · cos(2π · (c · t + d))
 *     Independent (a, b, c, d) for each of R, G, B.  At t=0 the
 *     output is a + b·cos(2π·d); as t grows the cosines rotate.
 *
 *   FORWARD EULER ADVECTION (same as flowfield.c):
 *     pos_new = pos + (cos angle, sin angle) · step
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - VORTEX SINGULARITY: dividing by r² blows up at the vortex
 *     centre.  We add a small ε to the denominator (VORT_SOFTEN_PX²)
 *     to keep things finite.
 *   - ASPECT RATIO: terminal cells are ~2× tall as wide.  The
 *     vortex / spiral generators correct dy by ASPECT_FACTOR (= 0.5
 *     in this file's "math"-coords convention) so that orbits look
 *     CIRCULAR on screen.  Without correction they'd be vertical
 *     ellipses.  See T8 for details.
 *   - CURL CENTRAL DIFFERENCES at boundary: we sample noise at
 *     (x ± ε, y ± ε); near the screen edge those samples can be
 *     "off-grid."  Perlin is defined everywhere though, so this is
 *     fine — the noise just doesn't see grid edges.
 *   - SHARED TRACER TIME: when you swap field types, tracers
 *     carry over their old positions but the field is suddenly
 *     different.  We force a `scene_reset` on swap so the visual
 *     transition is clean.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - Default mode (CURL): tracer trails form smooth meandering
 *     streams that fold and merge.  No "pile-up" anywhere — that's
 *     the divergence-free property.
 *   - Press 'a' to VORTEX: tracers visibly orbit a ring of N
 *     vortex points.  Adjacent vortices have OPPOSITE rotation
 *     (alternating CCW / CW) so streams twist between them.
 *   - Press 'a' to SINE: regular plaid-like crosshatch patterns.
 *     Long-living standing waves; tracers form regular grids.
 *   - Press 'a' to SPIRAL: galaxy effect.  Tracers spiral
 *     counter-clockwise in / out, breathing on a slow timescale.
 *   - Press 't' to cycle palettes: the same field stays the same
 *     SHAPE, but the COLOURS change.  Confirms palette and field
 *     are independent layers.
 *   - Press 'v' to cycle backgrounds: 'arrows' shows the field
 *     direction at every cell; 'colormap' fills the screen with
 *     palette-coloured arrows so the topology is visible at a
 *     glance.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Eight tutorials covering the four field models and the cosine
 * palette.  After T2 you understand the architectural pattern; T3-T6
 * cover one field each; T7-T8 close the loop on colour and aspect.
 *
 *   T1  What's "complex" about this flowfield?
 *   T2  Function-pointer dispatch — pluggable physics
 *   T3  Curl noise — divergence-free turbulence in 5 lines
 *   T4  Biot-Savart vortices — point sources of rotation
 *   T5  Sine lattice — interference patterns from sums
 *   T6  Radial spiral — polar coordinates and tangential flow
 *   T7  Cosine palette — Inigo Quilez's perceptual gradient trick
 *   T8  Aspect ratio — why vortices need a y-squash
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT'S "COMPLEX" ABOUT THIS FLOWFIELD?
 * ──────────────────────────────────────────
 * The simpler flowfield.c uses ONE field generator (Perlin noise →
 * atan2).  That's enough to look like wind.  But there are MANY
 * other ways to generate a vector field, each with a different
 * visual personality:
 *
 *     CURL NOISE        organic, fluid-like, no pile-ups
 *     POINT VORTICES    swirling eyes, like a galaxy of eddies
 *     WAVE INTERFERENCE crisscross plaid, regular geometry
 *     POLAR SPIRAL      breathing rotation, galaxy effect
 *
 * "Complex" here means:
 *   - MORE physics models than flowfield.c
 *   - Each model TEACHES a different concept (divergence, vortices,
 *     interference, polar coords)
 *   - Architectural pattern (T2): all four plug into ONE engine
 *
 * Plus a richer COLOUR system (cosine palette, T7) and three
 * BACKGROUND modes (blank / arrows / colormap) so the underlying
 * field can be made visible at three levels of detail.
 *
 * If you've worked through fluid/flowfield.c, this file is the
 * sequel.
 *
 * T2  FUNCTION-POINTER DISPATCH — PLUGGABLE PHYSICS
 * ─────────────────────────────────────────────────
 * The architectural trick that makes "four physics models" easy:
 *
 *   1. Define a uniform CONTRACT for what a "field generator" is:
 *
 *        float compute_angle(field, x, y, t)
 *
 *      "Given the active field state and a continuous (x, y, t),
 *      return the flow angle at that point in radians."
 *
 *   2. Implement four such functions (§9-§12), each ~30 lines.
 *
 *   3. The field has an ENUM telling which one is active:
 *
 *        enum field_kind { CURL, VORTEX, SINE, SPIRAL };
 *        field.active_kind = CURL;
 *
 *   4. The field-rebuild loop SWITCHES on `active_kind`:
 *
 *        for r, c in grid:
 *          switch field.active_kind:
 *            CURL   : field.angle[r][c] = field_curl_angle(  ... )
 *            VORTEX : field.angle[r][c] = field_vortex_angle(... )
 *            SINE   : field.angle[r][c] = field_sine_angle(  ... )
 *            SPIRAL : field.angle[r][c] = field_spiral_angle(... )
 *
 *   5. The TRACER ENGINE never looks at the kind — it just calls
 *      `field_sample()` to get the bilinearly-interpolated angle.
 *
 * This is the GOF "Strategy" pattern in C — the strategy is the
 * field-generation function, swappable at runtime.
 *
 * Adding a 5th field is a 3-line code change (new enum, new switch
 * case, new function) — not a refactor of every consumer.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   tracer engine ────► field_sample()             │
 *      │                          │                       │
 *      │                          ▼                       │
 *      │                    ┌───────────┐                 │
 *      │                    │ angle[r][c]│ (cached)       │
 *      │                    └───────────┘                 │
 *      │                          ▲                       │
 *      │                          │ filled by             │
 *      │                          │                       │
 *      │   field_evolve_and_rebuild():                    │
 *      │     for each cell:                               │
 *      │       switch active_kind:                        │
 *      │         curl   → field_curl_angle()              │
 *      │         vortex → field_vortex_angle()            │
 *      │         sine   → field_sine_angle()              │
 *      │         spiral → field_spiral_angle()            │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Same dispatch idea is used everywhere:
 *   - Game engines (component systems, render passes)
 *   - GUI toolkits (widget event handlers)
 *   - Compiler front ends (per-token parser)
 *   - This project: artistic/hindu_mandalas.c (RingType dispatch)
 *
 * T3  CURL NOISE — DIVERGENCE-FREE TURBULENCE IN 5 LINES
 * ──────────────────────────────────────────────────────
 * Plain Perlin noise gives a SCALAR — a smooth random field of
 * floats.  Two independent samples → (vx, vy) → atan2 → angle, as
 * in fluid/flowfield.c.  But that vector field is NOT divergence-
 * free: it has spurious "sources" (cells where flow points
 * outward) and "sinks" (inward).  Tracers PILE UP in sinks and
 * DRAIN OUT of sources, producing visual clumping.
 *
 * Real fluid (incompressible) has ∇ · V = 0 everywhere — mass is
 * conserved, no piling up.
 *
 * THE TRICK: take ONE scalar Perlin field ψ(x, y, t).  Define
 * velocity as the CURL:
 *
 *     vx =  ∂ψ/∂y
 *     vy = -∂ψ/∂x
 *
 * Quick algebra to confirm divergence-free:
 *
 *     ∇ · V = ∂vx/∂x + ∂vy/∂y
 *           = ∂²ψ/∂y∂x  +  ∂(-∂ψ/∂x)/∂y
 *           = ∂²ψ/(∂x∂y)  -  ∂²ψ/(∂x∂y)
 *           = 0                                     ✓
 *
 * (Mixed partials are equal for any smooth ψ — Schwarz's theorem.)
 *
 * Implementation uses CENTRAL DIFFERENCES — sample the noise four
 * times around (x, y) and subtract:
 *
 *     vx ≈ (ψ(x, y+ε) - ψ(x, y-ε)) / (2ε)
 *     vy ≈ -(ψ(x+ε, y) - ψ(x-ε, y)) / (2ε)
 *
 * Five lines (4 noise samples + atan2):
 *
 *     n_north = noise_fbm(x,        y + eps, t)
 *     n_south = noise_fbm(x,        y - eps, t)
 *     n_east  = noise_fbm(x + eps,  y,       t)
 *     n_west  = noise_fbm(x - eps,  y,       t)
 *     return atan2(n_north - n_south, -(n_east - n_west))
 *      (the (1/2ε) in numerator + denominator cancels in atan2)
 *
 * VISUAL PROOF: in CURL mode tracers form long meandering
 * streams that NEVER PILE UP.  Compare with plain Perlin (the
 * other simpler file) where you can sometimes see a faint clump.
 *
 * Implemented in §9 field_curl.
 *
 * T4  BIOT-SAVART VORTICES — POINT SOURCES OF ROTATION
 * ────────────────────────────────────────────────────
 * In electromagnetism, the BIOT-SAVART LAW gives the magnetic
 * field around a current-carrying wire: each current segment
 * contributes a field that falls off as 1/r and points
 * perpendicular to the radius.
 *
 * In 2-D fluid mechanics, exactly the same formula gives the
 * velocity around a "POINT VORTEX" — an idealised infinitesimal
 * column of swirling fluid.  Every actual vortex (tornado, drain
 * whirlpool, cyclone) can be approximated as a sum of point
 * vortices.
 *
 * For ONE vortex at origin with strength Γ ("circulation"):
 *
 *     vx = Γ · (-y) / (r² + ε)
 *     vy = Γ ·   x  / (r² + ε)
 *     where r² = x² + y² and ε is a small softening
 *
 * Why does this give rotation?  Compute the curl:
 *   - Sign of vx is opposite sign of y → flow goes backward
 *     above the vortex, forward below.
 *   - Sign of vy is same sign as x → flow goes forward to the
 *     right, backward to the left.
 *   The result: counter-clockwise rotation around the origin
 *   (for Γ > 0).  Sign-flipped Γ gives CW rotation.
 *
 * The 1/r² factor means the velocity is HUGE near the centre
 * (mathematical singularity), so we add ε ≈ 5 to avoid blowup.
 * This is called a "REGULARISED" or "BLOB" vortex.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │              flow direction at corners:          │
 *      │                                                  │
 *      │           ←      ↑       →                       │
 *      │                                                  │
 *      │              ●  vortex at centre                 │
 *      │              (Γ > 0 = CCW)                       │
 *      │                                                  │
 *      │           ↓      ←       ↑                       │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * SUPERPOSITION: with N vortices at positions cᵢ, strengths Γᵢ:
 *
 *     v_total(p) = Σ  Γᵢ · ((-(p.y - cᵢ.y), p.x - cᵢ.x))
 *                       / (|p - cᵢ|² + ε)
 *
 * This file uses N=6 vortices on a slowly-rotating ring, with
 * alternating Γ signs (CCW, CW, CCW, ...).  Adjacent opposite-sign
 * vortices induce flow BETWEEN them — produces clean
 * tracer streams threading the ring.
 *
 * Implemented in §10 field_vortex.
 *
 * T5  SINE LATTICE — INTERFERENCE PATTERNS FROM SUMS
 * ──────────────────────────────────────────────────
 * Simplest "field": just sums of sinusoids.  No physical model —
 * pure pattern.  Looks LIKE wave interference.
 *
 *     vx = sin(x · fx + t) + sin(y · fy - 0.7t)
 *     vy = cos(x · fx - 0.5t) + cos(y · fy + 0.3t)
 *     angle = atan2(vy, vx)
 *
 * Each sin/cos is a TRAVELLING WAVE in space + time.  Sum two →
 * INTERFERENCE pattern.  The horizontal component is the sum of
 * one x-going wave and one y-going wave; the vertical is sum of a
 * different x-going and y-going pair.  At intersections, the
 * waves reinforce or cancel.
 *
 * Visual signature: REGULAR CRISSCROSS PATTERN, like a plaid
 * fabric or moiré.  Does NOT look like fluid.  Looks like
 * deliberate geometric art.  Very different from curl/vortex.
 *
 * The phase offsets (0.7, 0.5, 0.3) prevent the four terms from
 * being perfectly in step — that would degenerate to a single
 * standing wave.  Different speeds make the pattern slowly drift.
 *
 * Implemented in §11 field_sine.
 *
 * Real-world analogue: think of ripples on a pond from two stones
 * thrown in different places — the interference pattern between
 * them is mathematically the same.
 *
 * T6  RADIAL SPIRAL — POLAR COORDINATES AND TANGENTIAL FLOW
 * ─────────────────────────────────────────────────────────
 * In polar coordinates (r, θ) relative to some centre, the unit
 * vectors are:
 *
 *     r̂ = ( cos θ, sin θ)         pointing OUT from centre
 *     θ̂ = (-sin θ, cos θ)         pointing PERPENDICULAR (CCW)
 *
 * A pure TANGENTIAL FLOW means velocity is θ̂ everywhere:
 *
 *     vx = -sin θ
 *     vy =  cos θ
 *
 * This is INFINITE COUNTER-CLOCKWISE ROTATION about the centre at
 * unit speed regardless of radius.  Tracers orbit forever.  Looks
 * boring on its own — same speed in concentric circles.
 *
 * Add a TIME-PULSING RADIAL component:
 *
 *     vx = -sin θ + W · sin(0.8t) · cos θ
 *     vy =  cos θ + W · sin(0.8t) · sin θ
 *
 * The W·sin(0.8t) term oscillates between -W and +W over time.
 * When positive, it adds an OUTWARD push (along r̂); when negative,
 * INWARD.  Tracers BREATHE — alternately spiral out, then in, in
 * a slow rhythm.
 *
 * Visual: galaxy with periodic puffs of expansion / contraction.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │       at a radius r:                             │
 *      │                                                  │
 *      │       pure tangential:        + radial pulse:    │
 *      │                                                  │
 *      │            θ̂                       θ̂   r̂          │
 *      │            ↗                         ↗ ↗          │
 *      │       ●─────►                  ●─────► ●          │
 *      │       (CCW)                    (CCW + outward)    │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * The polar-coordinate aspect-ratio fix (T8) keeps the spiral
 * actually CIRCULAR in cells (terminal cells are 2:1 tall).
 *
 * Implemented in §12 field_spiral.
 *
 * T7  COSINE PALETTE — INIGO QUILEZ'S PERCEPTUAL GRADIENT TRICK
 * ─────────────────────────────────────────────────────────────
 * Generating colour gradients is surprisingly subtle.  Naïve
 * approaches:
 *
 *   - LERP between two RGB endpoints: looks dull because RGB lerp
 *     passes through grey at the midpoint.
 *   - HSV with H rotating: looks "rainbow," somewhat garish.
 *   - Pre-baked palette tables (256 colours): heavy, inflexible.
 *
 * Inigo Quilez's COSINE PALETTE formula generates a SMOOTH,
 * COMPLEMENTARY gradient with FOUR per-channel parameters:
 *
 *     red(t)   = a₀ + b₀ · cos(2π · (c₀ · t + d₀))
 *     green(t) = a₁ + b₁ · cos(2π · (c₁ · t + d₁))
 *     blue(t)  = a₂ + b₂ · cos(2π · (c₂ · t + d₂))
 *
 * For a parameter t ∈ [0, 1] → output (R, G, B) ∈ [0, 1]³ (clamped).
 *
 * The ABCD vectors:
 *
 *   a (BIAS)       — baseline brightness; (0.5, 0.5, 0.5) = grey-centred
 *   b (AMPLITUDE)  — how far each channel oscillates
 *   c (FREQUENCY)  — how many cycles across t ∈ [0, 1]
 *   d (PHASE)      — channel-specific phase offset → controls hue
 *
 * Different (a, b, c, d) settings give wildly different palettes:
 *
 *   c = (1, 1, 1), d = (0, 1/3, 2/3):  classic rainbow
 *   c = (2, 1, 0), d = (0.5, 0.2, 0.25):  tropical sunset
 *   c = (1, 1, 0.5), d = (0.8, 0.9, 0.3):  cosmic violet/cyan
 *
 * This file ships SIX hand-tuned themes (cosmic, ember, ocean, neon,
 * sunset, mono).  Cycling through them with 't' demonstrates how
 * the same field can look totally different just by swapping the
 * four parameter triples.
 *
 * Implementation: pre-bake 16 evenly-spaced palette samples into
 * ncurses pairs at theme-switch time.  The angle-to-pair mapping
 * (§7 angle_to_palette_pair) takes (angle ∈ [-π, +π]), normalises
 * to [0, 1], scales to a pair index.
 *
 * Source: https://iquilezles.org/articles/palettes/
 *
 * T8  ASPECT RATIO — WHY VORTICES NEED A Y-SQUASH
 * ───────────────────────────────────────────────
 * Terminal cells are roughly TWICE AS TALL as wide.  When we
 * compute "distance from a vortex centre" in cell coordinates:
 *
 *     dx = px - cx          horizontal
 *     dy = py - cy          vertical (in cells)
 *     r² = dx² + dy²        (NAIVE)
 *
 * the formula treats 1 horizontal cell as the same length as 1
 * vertical cell.  But on screen, 1 vertical cell SPANS 2× THE
 * PIXELS of 1 horizontal cell.  So a VISUALLY round circle of
 * radius R cells wide is only R/2 cells tall:
 *
 *      ┌────────────────────────────────────────────────────┐
 *      │                                                    │
 *      │   What the math sees      What you see on screen:  │
 *      │   (dx² + dy² = const):    (true on-screen circle)  │
 *      │                                                    │
 *      │       ●  ●  ●                  ●  ●                │
 *      │     ●        ●                ●    ●               │
 *      │     ●        ●               ●      ●              │
 *      │       ●  ●  ●                ●      ●              │
 *      │     (TALL ELLIPSE)            ●    ●               │
 *      │                                ●  ●                │
 *      │                                                    │
 *      └────────────────────────────────────────────────────┘
 *
 * To get visually-circular vortex orbits and visually-circular
 * spirals, scale dy DOWN by ASPECT_FACTOR ≈ 0.5 before computing
 * distances:
 *
 *     dx = px - cx
 *     dy = (py - cy) / 0.5      — i.e. multiply by 2
 *      OR equivalently
 *     dy = (py - cy) * 2.0
 *
 * Wait — depending on convention this could be DIVIDE by 0.5 or
 * MULTIPLY by 0.5.  The right framing: if you want a CIRCLE of N
 * cells horizontally, you want (N/2) cells vertically.  So when
 * comparing dx and dy in geometric formulas, you stretch dy.
 *
 * In this file we DIVIDE dy by ASPECT_FACTOR (0.5) inside the
 * vortex / spiral generators — equivalent to MULTIPLYING dy by 2.
 * The math now lives in "isotropic visual pixel space" where 1 unit
 * x and 1 unit y are the same physical screen distance.
 *
 * Same trick used everywhere geometric in this project:
 *   - artistic/hindu_mandalas.c (polar mandalas)
 *   - matrix_rain/pulsar_rain.c (radial beams)
 *   - turtle/duo_poly.c (regular polygons)
 *   - algorithms/visibility_polygon.c (geo-space coordinates)
 *
 * One trick.  One concept.  Many files apply it.
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
 * Every literal in code below this section is a NAME from §1.  Tunables
 * grouped by subsystem so the file's "knobs" are easy to find.
 */

enum {
    /* Render frame cap. */
    RENDER_FPS_CAP        = 60,

    /* Simulation step rate. */
    SIM_HZ_MIN            =  5,
    SIM_HZ_DEFAULT        = 30,
    SIM_HZ_MAX            = 60,
    SIM_HZ_STEP           =  5,

    /* HUD recompute cadence. */
    FPS_RECOMPUTE_MS      = 500,

    /* Tracer pool. */
    TRACERS_MIN           =  50,
    TRACERS_DEFAULT       = 400,
    TRACERS_MAX           = 800,
    TRACERS_STEP          =  50,

    /* Trail ring buffer (per tracer). */
    TRAIL_LEN_MIN         =  3,
    TRAIL_LEN_DEFAULT     = 18,
    TRAIL_LEN_MAX         = 24,
    TRAIL_LEN_HARD_MAX    = 24,    /* compile-time array size */

    /* FBM octaves used by the curl-noise generator. */
    CURL_FBM_OCTAVES      =  3,

    /* Cosine palette: 16 evenly-spaced samples become ncurses pairs. */
    PALETTE_PAIR_COUNT    = 16,

    /* Number of themes. */
    THEME_COUNT           =  6,

    /* Number of background modes (blank / arrows / colormap). */
    BG_MODE_COUNT         =  3,

    /* Number of field generator kinds (curl / vortex / sine / spiral). */
    FIELD_KIND_COUNT      =  4,

    /* Vortex-lattice config. */
    VORTEX_COUNT          =  6,
};

/* PHYSICAL / VISUAL CONSTANTS — units in the comments. */

/* Tracer step distance (cells per tick) with per-particle jitter. */
#define TRACER_STEP_BASE_CPT       0.85f
#define TRACER_STEP_JITTER_CPT     0.40f

/* Tracer lifetime (ticks before forced respawn) with jitter. */
#define TRACER_LIFE_BASE_TICKS     120
#define TRACER_LIFE_JITTER_TICKS    80

/* Field time-axis evolution (per tick). */
#define FIELD_EVOLUTION_DEFAULT    0.006f
#define FIELD_EVOLUTION_MIN        0.001f
#define FIELD_EVOLUTION_MAX        0.080f
#define FIELD_EVOLUTION_FACTOR     1.5f         /* multiplier for f/F keys */

/* Curl-noise spatial scales + central-difference epsilon. */
#define CURL_NOISE_SCALE_X         0.030f
#define CURL_NOISE_SCALE_Y         0.055f
#define CURL_DIFFERENCE_EPSILON    1.20f

/* Vortex-lattice geometry. */
#define VORTEX_RING_RADIUS_FRAC    0.28f        /* fraction of min(W,H)/2 */
#define VORTEX_RING_ORBIT_SPEED    0.014f       /* radians per tick */
#define VORTEX_STRENGTH_GAMMA      3.0f
#define VORTEX_SOFTEN_PIXELS       5.0f         /* added to r² for stability */

/* Sine-lattice spatial frequencies. */
#define SINE_FREQ_X                0.055f
#define SINE_FREQ_Y                0.095f

/* Radial-spiral pulse amplitude. */
#define SPIRAL_RADIAL_WEIGHT       0.65f

/* Aspect-ratio compensation for vortex / spiral (T8). */
#define ASPECT_FACTOR              0.5f

/* Time helpers. */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS       1000000LL

/* Colour pair IDs. */
enum {
    PAIR_PALETTE_BASE  = 1,                                   /* +0..+15 */
    PAIR_HUD           = PAIR_PALETTE_BASE + PALETTE_PAIR_COUNT,
    PAIR_HINT,
};

/* Field kind enum + names (used by the dispatcher and HUD). */
enum field_kind {
    FIELD_KIND_CURL = 0,
    FIELD_KIND_VORTEX,
    FIELD_KIND_SINE,
    FIELD_KIND_SPIRAL,
};

static const char *field_kind_name_table[FIELD_KIND_COUNT] = {
    "curl-noise", "vortex-lattice", "sine-lattice", "radial-spiral"
};

/* Background mode names. */
static const char *bg_mode_name_table[BG_MODE_COUNT] = {
    "blank", "arrows", "colormap"
};

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
/* §3  rng — Fisher-Yates shuffle for the perlin permutation table       */
/* ===================================================================== */
/*
 * Perlin noise's pseudo-randomness comes entirely from this 256-byte
 * table.  We seed rand() once at startup, build the table once, and
 * never touch it again.
 */

static uint8_t perlin_perm_table[512];

static void perlin_perm_init(void)
{
    uint8_t identity[256];
    for (int i = 0; i < 256; i++) identity[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t tmp = identity[i];
        identity[i] = identity[j];
        identity[j] = tmp;
    }
    /* Double for index-without-modulo. */
    for (int i = 0; i < 512; i++)
        perlin_perm_table[i] = identity[i & 255];
}

/* ===================================================================== */
/* §4  perlin — single-octave 2-D Perlin noise                           */
/* ===================================================================== */
/*
 * Smooth pseudo-random scalar in [-1, +1] at any (x, y).  Same
 * algorithm as fluid/flowfield.c §4 (read that file's tutorial T3 if
 * unfamiliar): integer-cell + smoothstep-interpolated bilinear
 * gradient hash.
 */

static inline float smoothstep_cubic(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

static inline float lerp_scalar(float a, float b, float t)
{
    return a + t * (b - a);
}

static inline float perlin_gradient_dot(int hash, float x, float y)
{
    int   bits  = hash & 3;
    float term1 = (bits < 2) ? x : y;
    float term2 = (bits < 2) ? y : x;
    return ((hash & 1) ? -term1 : term1) + ((hash & 2) ? -term2 : term2);
}

static float perlin_value(float x, float y)
{
    int   xi = (int)floorf(x) & 255;
    int   yi = (int)floorf(y) & 255;
    float fx = x - floorf(x);
    float fy = y - floorf(y);
    float ux = smoothstep_cubic(fx);
    float uy = smoothstep_cubic(fy);

    int h00 = perlin_perm_table[perlin_perm_table[xi    ] + yi    ];
    int h10 = perlin_perm_table[perlin_perm_table[xi + 1] + yi    ];
    int h01 = perlin_perm_table[perlin_perm_table[xi    ] + yi + 1];
    int h11 = perlin_perm_table[perlin_perm_table[xi + 1] + yi + 1];

    float d00 = perlin_gradient_dot(h00, fx,        fy       );
    float d10 = perlin_gradient_dot(h10, fx - 1.0f, fy       );
    float d01 = perlin_gradient_dot(h01, fx,        fy - 1.0f);
    float d11 = perlin_gradient_dot(h11, fx - 1.0f, fy - 1.0f);

    float top    = lerp_scalar(d00, d10, ux);
    float bottom = lerp_scalar(d01, d11, ux);
    return lerp_scalar(top, bottom, uy);
}

/* ===================================================================== */
/* §5  fbm — multi-octave fractional Brownian motion                     */
/* ===================================================================== */
/*
 * Sum N octaves at increasing frequency / decreasing amplitude.  See
 * fluid/flowfield.c T4 for the why.  The scale + time inputs come
 * from the caller (curl-noise field generator below).
 */

static float fbm_value(float x, float y, float t, int octaves)
{
    float result    = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    for (int oct = 0; oct < octaves; oct++) {
        result    += perlin_value(x * frequency + t,
                                  y * frequency + t * 0.7f) * amplitude;
        amplitude *= 0.5f;
        frequency *= 2.0f;
    }
    return result;
}

/* ===================================================================== */
/* §6  cosine_palette — Inigo Quilez palette + 6 themes                  */
/* ===================================================================== */
/*
 * THE FORMULA (T7):
 *     color(t) = a + b · cos(2π · (c · t + d))
 *
 * Stored as one CosinePaletteTheme per theme.  Each member is a 3-vector
 * (one value per RGB channel).
 */

typedef struct {
    float bias_rgb[3];        /* a — channel-wise mean */
    float amplitude_rgb[3];   /* b — channel-wise oscillation amplitude */
    float frequency_rgb[3];   /* c — cycles across t ∈ [0, 1] */
    float phase_rgb[3];       /* d — channel-wise phase offset */
    const char *name;
} CosinePaletteTheme;

/* Six themes — feel-targets in the comment. */
static const CosinePaletteTheme palette_theme_table[THEME_COUNT] = {
    /* 0 cosmic — electric violet → cyan → hot magenta */
    { {0.50f, 0.50f, 0.50f},  {0.50f, 0.50f, 0.50f},
      {1.00f, 1.00f, 0.50f},  {0.80f, 0.90f, 0.30f},  "cosmic" },

    /* 1 ember — deep red → orange → pale yellow */
    { {0.55f, 0.30f, 0.05f},  {0.45f, 0.30f, 0.05f},
      {1.00f, 0.80f, 0.30f},  {0.00f, 0.10f, 0.25f},  "ember" },

    /* 2 ocean — navy → teal → ice */
    { {0.15f, 0.40f, 0.60f},  {0.20f, 0.35f, 0.40f},
      {0.50f, 0.70f, 1.00f},  {0.00f, 0.10f, 0.30f},  "ocean" },

    /* 3 neon — electric green → hot pink → violet */
    { {0.50f, 0.50f, 0.50f},  {0.50f, 0.50f, 0.50f},
      {1.00f, 0.50f, 1.00f},  {0.00f, 0.50f, 0.33f},  "neon" },

    /* 4 sunset — purple → crimson → amber */
    { {0.50f, 0.38f, 0.30f},  {0.50f, 0.38f, 0.30f},
      {1.00f, 0.85f, 0.60f},  {0.00f, 0.18f, 0.40f},  "sunset" },

    /* 5 mono — silver-blue clean grayscale */
    { {0.45f, 0.48f, 0.55f},  {0.40f, 0.42f, 0.45f},
      {0.50f, 0.50f, 0.50f},  {0.00f, 0.02f, 0.05f},  "mono" },
};

/* Evaluate the cosine palette at parameter t ∈ [0, 1] → (R, G, B).
 * Each channel ∈ [0, 1] after clamping. */
static void cosine_palette_eval(const CosinePaletteTheme *theme,
                                float palette_param,
                                float *out_r, float *out_g, float *out_b)
{
    float r = theme->bias_rgb[0]
            + theme->amplitude_rgb[0]
              * cosf(2.0f * (float)M_PI
                     * (theme->frequency_rgb[0] * palette_param
                        + theme->phase_rgb[0]));
    float g = theme->bias_rgb[1]
            + theme->amplitude_rgb[1]
              * cosf(2.0f * (float)M_PI
                     * (theme->frequency_rgb[1] * palette_param
                        + theme->phase_rgb[1]));
    float b = theme->bias_rgb[2]
            + theme->amplitude_rgb[2]
              * cosf(2.0f * (float)M_PI
                     * (theme->frequency_rgb[2] * palette_param
                        + theme->phase_rgb[2]));
    if (r < 0.0f) r = 0.0f; else if (r > 1.0f) r = 1.0f;
    if (g < 0.0f) g = 0.0f; else if (g > 1.0f) g = 1.0f;
    if (b < 0.0f) b = 0.0f; else if (b > 1.0f) b = 1.0f;
    *out_r = r; *out_g = g; *out_b = b;
}

/* Map an (R, G, B) ∈ [0, 1]³ to the closest xterm-256 cube colour
 * (indices 16..231 = 6×6×6 = 216 colours). */
static int rgb_to_xterm256_cube(float r, float g, float b)
{
    int r5 = (int)(r * 5.0f + 0.5f);
    int g5 = (int)(g * 5.0f + 0.5f);
    int b5 = (int)(b * 5.0f + 0.5f);
    if (r5 > 5) r5 = 5;
    if (r5 < 0) r5 = 0;
    if (g5 > 5) g5 = 5;
    if (g5 < 0) g5 = 0;
    if (b5 > 5) b5 = 5;
    if (b5 < 0) b5 = 0;
    return 16 + 36 * r5 + 6 * g5 + b5;
}

/* 8-colour fallback (one row per theme). */
static const int palette_fallback_8[THEME_COUNT][PALETTE_PAIR_COUNT] = {
    { COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE,
      COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE,
      COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE,
      COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE },
    { COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
      COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
      COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
      COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE },
    { COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE },
    { COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE,
      COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE,
      COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE,
      COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE },
    { COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
      COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
      COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
      COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW, COLOR_WHITE },
    { COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
};

/* ===================================================================== */
/* §7  colors — ncurses pair init + angle → palette pair                 */
/* ===================================================================== */

static void colors_apply_theme(int theme_index)
{
    const CosinePaletteTheme *theme = &palette_theme_table[theme_index];
    bool has_256 = (COLORS >= 256);

    for (int i = 0; i < PALETTE_PAIR_COUNT; i++) {
        float palette_param = (float)i
                            / (float)(PALETTE_PAIR_COUNT - 1);
        if (has_256) {
            float r, g, b;
            cosine_palette_eval(theme, palette_param, &r, &g, &b);
            int fg = rgb_to_xterm256_cube(r, g, b);
            init_pair(PAIR_PALETTE_BASE + i, fg, -1);
        } else {
            init_pair(PAIR_PALETTE_BASE + i,
                      palette_fallback_8[theme_index][i], -1);
        }
    }

    /* HUD per CLAUDE.md: bright yellow + bright cyan, default bg. */
    if (has_256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

static void colors_init(int theme_index)
{
    start_color();
    use_default_colors();
    colors_apply_theme(theme_index);
}

/* angle_to_palette_pair — map a flow angle ∈ (-π, π] to one of the
 * PALETTE_PAIR_COUNT palette pairs.  Adjacent angles get adjacent
 * pairs; opposite angles get half-cycle-apart palette colours
 * (perceptually complementary). */
static int angle_to_palette_pair(float angle_radians)
{
    float a = angle_radians;
    if (a < 0.0f) a += 2.0f * (float)M_PI;
    int idx = (int)(a / (2.0f * (float)M_PI) * PALETTE_PAIR_COUNT)
            % PALETTE_PAIR_COUNT;
    return PAIR_PALETTE_BASE + idx;
}

/* ===================================================================== */
/* §8  field — angle grid + bilinear sample + dispatch                   */
/* ===================================================================== */
/*
 * The field state.  Holds a per-cell flow angle, plus the time-axis
 * variable, plus per-mode auxiliary state (vortex positions).  The
 * `active_kind` selects which generator function fills `angle[][]`
 * each tick.
 */

#define FIELD_COLS_MAX 256
#define FIELD_ROWS_MAX  80

typedef struct {
    int    active_cols;
    int    active_rows;
    int    active_kind;            /* one of FIELD_KIND_* */
    float  time_axis;
    float  evolution_speed;

    /* Flow angle per cell (radians).  Filled by the active generator. */
    float  angle[FIELD_ROWS_MAX][FIELD_COLS_MAX];

    /* Vortex-lattice auxiliary state. */
    float  vortex_pos_col[VORTEX_COUNT];
    float  vortex_pos_row[VORTEX_COUNT];
    float  vortex_strength_table[VORTEX_COUNT];
    float  vortex_ring_phase;     /* slowly increases — ring rotation */
} flow_field;

/* Forward declarations of generators — defined in §9-§12. */
static float field_curl_angle  (const flow_field *f, float x, float y, float t);
static float field_vortex_angle(const flow_field *f, float x, float y);
static float field_sine_angle  (const flow_field *f, float x, float y, float t);
static float field_spiral_angle(const flow_field *f, float x, float y, float t);

static void field_update_vortex_positions(flow_field *f)
{
    /* Place N vortices on a ring centred on the screen centre.  Each
     * frame the ring slowly rotates by VORTEX_RING_ORBIT_SPEED. */
    float centre_col = (float)f->active_cols * 0.5f;
    float centre_row = (float)f->active_rows * 0.5f;
    int   smaller    = (f->active_cols < f->active_rows)
                     ? f->active_cols : f->active_rows;
    float radius     = VORTEX_RING_RADIUS_FRAC * (float)smaller * 0.5f;

    for (int i = 0; i < VORTEX_COUNT; i++) {
        float angle = f->vortex_ring_phase
                    + (float)i * (2.0f * (float)M_PI / (float)VORTEX_COUNT);
        f->vortex_pos_col[i] = centre_col + radius * cosf(angle);
        /* y-component scaled by ASPECT_FACTOR so the ring is VISUALLY
         * round on a 2:1-tall cell grid (T8). */
        f->vortex_pos_row[i] = centre_row
                             + radius * sinf(angle) * ASPECT_FACTOR;
    }
}

static void field_init(flow_field *f, int cols, int rows, int kind)
{
    if (cols > FIELD_COLS_MAX) cols = FIELD_COLS_MAX;
    if (rows > FIELD_ROWS_MAX) rows = FIELD_ROWS_MAX;
    f->active_cols      = cols;
    f->active_rows      = rows;
    f->active_kind      = kind;
    f->time_axis        = 0.0f;
    f->evolution_speed  = FIELD_EVOLUTION_DEFAULT;
    f->vortex_ring_phase = 0.0f;
    memset(f->angle, 0, sizeof f->angle);

    /* Alternate vortex strength signs (CCW, CW, CCW, ...) for a
     * dynamic between adjacent vortices. */
    for (int i = 0; i < VORTEX_COUNT; i++)
        f->vortex_strength_table[i] = (i & 1)
            ? -VORTEX_STRENGTH_GAMMA :  VORTEX_STRENGTH_GAMMA;

    field_update_vortex_positions(f);
}

/* field_evolve_and_rebuild — advance time, then recompute every cell.
 * The DISPATCHER for the four field kinds. */
static void field_evolve_and_rebuild(flow_field *f)
{
    f->time_axis += f->evolution_speed;
    f->vortex_ring_phase += VORTEX_RING_ORBIT_SPEED;
    field_update_vortex_positions(f);

    for (int r = 0; r < f->active_rows; r++) {
        for (int c = 0; c < f->active_cols; c++) {
            float angle;
            switch (f->active_kind) {
                case FIELD_KIND_CURL:
                    angle = field_curl_angle(f, (float)c, (float)r,
                                             f->time_axis);
                    break;
                case FIELD_KIND_VORTEX:
                    angle = field_vortex_angle(f, (float)c, (float)r);
                    break;
                case FIELD_KIND_SINE:
                    angle = field_sine_angle(f, (float)c, (float)r,
                                             f->time_axis);
                    break;
                case FIELD_KIND_SPIRAL:
                default:
                    angle = field_spiral_angle(f, (float)c, (float)r,
                                               f->time_axis);
                    break;
            }
            f->angle[r][c] = angle;
        }
    }
}

/* flow_angle_at_cell — the angle stored at integer cell (c, r). */
static inline float flow_angle_at_cell(const flow_field *f, int c, int r)
{
    if (c < 0)                   c = 0;
    if (c >= f->active_cols)     c = f->active_cols - 1;
    if (r < 0)                   r = 0;
    if (r >= f->active_rows)     r = f->active_rows - 1;
    return f->angle[r][c];
}

/* flow_angle_bilinear — interpolated angle at sub-cell (col, row). */
static float flow_angle_bilinear(const flow_field *f, float col, float row)
{
    int c0 = (int)floorf(col);
    int r0 = (int)floorf(row);
    int c1 = c0 + 1;
    int r1 = r0 + 1;

    if (c0 < 0)                  c0 = 0;
    if (c0 >= f->active_cols)    c0 = f->active_cols - 1;
    if (c1 < 0)                  c1 = 0;
    if (c1 >= f->active_cols)    c1 = f->active_cols - 1;
    if (r0 < 0)                  r0 = 0;
    if (r0 >= f->active_rows)    r0 = f->active_rows - 1;
    if (r1 < 0)                  r1 = 0;
    if (r1 >= f->active_rows)    r1 = f->active_rows - 1;

    float fx = col - floorf(col);
    float fy = row - floorf(row);

    float a00 = f->angle[r0][c0];
    float a10 = f->angle[r0][c1];
    float a01 = f->angle[r1][c0];
    float a11 = f->angle[r1][c1];

    float top    = lerp_scalar(a00, a10, fx);
    float bottom = lerp_scalar(a01, a11, fx);
    return lerp_scalar(top, bottom, fy);
}

/* ===================================================================== */
/* §9  field_curl — divergence-free turbulence (T3)                      */
/* ===================================================================== */
/*
 * Sample a scalar potential ψ via 4 Perlin-FBM samples around (x, y).
 * Take the 2-D curl (∂ψ/∂y, -∂ψ/∂x) by central differences.
 * atan2 → angle.
 *
 * The result is GUARANTEED divergence-free (∇ · V = 0), so tracers
 * do not pile up.
 */

static float field_curl_angle(const flow_field *f, float x, float y, float t)
{
    (void)f;   /* curl uses only Perlin state, not field state */
    float eps = CURL_DIFFERENCE_EPSILON;

    float psi_north = fbm_value(x * CURL_NOISE_SCALE_X,
                                y * CURL_NOISE_SCALE_Y + eps,
                                t, CURL_FBM_OCTAVES);
    float psi_south = fbm_value(x * CURL_NOISE_SCALE_X,
                                y * CURL_NOISE_SCALE_Y - eps,
                                t, CURL_FBM_OCTAVES);
    float psi_east  = fbm_value(x * CURL_NOISE_SCALE_X + eps,
                                y * CURL_NOISE_SCALE_Y,
                                t, CURL_FBM_OCTAVES);
    float psi_west  = fbm_value(x * CURL_NOISE_SCALE_X - eps,
                                y * CURL_NOISE_SCALE_Y,
                                t, CURL_FBM_OCTAVES);

    /* curl(ψ) = (∂ψ/∂y, -∂ψ/∂x) via central differences. */
    float vx =  (psi_north - psi_south);    /* /(2ε) cancels in atan2 */
    float vy = -(psi_east  - psi_west );
    return atan2f(vy, vx);
}

/* ===================================================================== */
/* §10  field_vortex — Biot-Savart point vortices (T4)                   */
/* ===================================================================== */
/*
 * Velocity at (x, y) from N point vortices is the sum of each
 * vortex's contribution.  For one vortex at (cx, cy) with strength Γ:
 *
 *     vx = Γ · (-(y-cy)) / (r² + ε)
 *     vy = Γ ·  (x-cx)   / (r² + ε)
 *
 * Aspect-ratio (T8): scale dy by 1/ASPECT_FACTOR so distances are
 * isotropic in visual screen space.
 */

static float field_vortex_angle(const flow_field *f, float x, float y)
{
    float vx_total = 0.0f;
    float vy_total = 0.0f;

    for (int i = 0; i < VORTEX_COUNT; i++) {
        float dx = x - f->vortex_pos_col[i];
        float dy = (y - f->vortex_pos_row[i]) / ASPECT_FACTOR;
        float r2 = dx * dx + dy * dy + VORTEX_SOFTEN_PIXELS;
        float strength = f->vortex_strength_table[i];
        vx_total += strength * (-dy) / r2;
        vy_total += strength * ( dx) / r2;
    }
    return atan2f(vy_total, vx_total);
}

/* ===================================================================== */
/* §11  field_sine — superposed sinusoidal waves (T5)                    */
/* ===================================================================== */
/*
 * Each velocity component is a sum of two travelling sinusoids in
 * different axes + different time speeds.  Visual signature: regular
 * crisscross interference patterns.
 */

static float field_sine_angle(const flow_field *f, float x, float y, float t)
{
    (void)f;
    float vx = sinf(x * SINE_FREQ_X + t)
             + sinf(y * SINE_FREQ_Y - t * 0.7f);
    float vy = cosf(x * SINE_FREQ_X - t * 0.5f)
             + cosf(y * SINE_FREQ_Y + t * 0.3f);
    return atan2f(vy, vx);
}

/* ===================================================================== */
/* §12  field_spiral — polar tangential + radial pulse (T6)              */
/* ===================================================================== */
/*
 * Pure tangential rotation (-sin θ, cos θ) plus a time-pulsing radial
 * component W·sin(0.8t).  Produces a breathing galaxy effect.
 *
 * Aspect ratio (T8): dy scaled by 1/ASPECT_FACTOR so the spiral is
 * visually circular on a 2:1 cell grid.
 */

static float field_spiral_angle(const flow_field *f, float x, float y, float t)
{
    float centre_col = (float)f->active_cols * 0.5f;
    float centre_row = (float)f->active_rows * 0.5f;
    float dx = x - centre_col;
    float dy = (y - centre_row) / ASPECT_FACTOR;
    float radius = sqrtf(dx * dx + dy * dy);
    if (radius < 1e-4f) return 0.0f;            /* avoid centre singularity */

    float theta = atan2f(dy, dx);
    float pulse = SPIRAL_RADIAL_WEIGHT * sinf(t * 0.8f);

    float vx = -sinf(theta) + pulse * cosf(theta);
    float vy =  cosf(theta) + pulse * sinf(theta);
    return atan2f(vy, vx);
}

/* ===================================================================== */
/* §13  arrows — angle → 8-octant ASCII arrow glyph                      */
/* ===================================================================== */
/*
 * Same 8-direction mapping used in fluid/flowfield.c.  See that
 * file's tutorial T7 for the geometry.
 */

#define ARROW_OCTANT_COUNT 8

static const char arrow_glyph_table[ARROW_OCTANT_COUNT] = {
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
    return (int)(a / (2.0f * (float)M_PI) * ARROW_OCTANT_COUNT + 0.5f)
         % ARROW_OCTANT_COUNT;
}

static inline char arrow_glyph_for_angle(float angle_radians)
{
    return arrow_glyph_table[angle_to_octant(angle_radians)];
}

/* ===================================================================== */
/* §14  tracer — one particle: pose + trail ring buffer                  */
/* ===================================================================== */
/*
 * Same struct shape as fluid/flowfield.c §10.  Re-read that file's
 * tutorial T7 if ring-buffer trails are unfamiliar.
 */

typedef struct {
    bool   tracer_alive;
    float  pos_col;
    float  pos_row;
    float  step_cells;
    float  last_angle;
    int    ticks_until_respawn;
    int    trail_pair_id;             /* current colour pair (palette) */
    int    trail_col[TRAIL_LEN_HARD_MAX];
    int    trail_row[TRAIL_LEN_HARD_MAX];
    int    trail_write_index;
    int    trail_filled_count;
    int    trail_active_length;
} tracer;

static void tracer_respawn(tracer *t, int active_cols, int active_rows,
                           int trail_active_length)
{
    t->tracer_alive          = true;
    t->pos_col               = (float)(rand() % active_cols);
    t->pos_row               = (float)(rand() % active_rows);
    t->step_cells            = TRACER_STEP_BASE_CPT
                             - TRACER_STEP_JITTER_CPT * 0.5f
                             + TRACER_STEP_JITTER_CPT
                               * ((float)rand() / (float)RAND_MAX);
    t->last_angle            = 0.0f;
    t->ticks_until_respawn   = TRACER_LIFE_BASE_TICKS
                             + rand() % TRACER_LIFE_JITTER_TICKS;
    t->trail_pair_id         = PAIR_PALETTE_BASE
                             + rand() % PALETTE_PAIR_COUNT;
    t->trail_write_index     = 0;
    t->trail_filled_count    = 0;
    t->trail_active_length   = trail_active_length;
    for (int i = 0; i < TRAIL_LEN_HARD_MAX; i++) {
        t->trail_col[i] = (int)t->pos_col;
        t->trail_row[i] = (int)t->pos_row;
    }
}

/* ===================================================================== */
/* §15  tracer_step — Lagrangian advection (one tick of one tracer)      */
/* ===================================================================== */
/*
 * 1. Push current integer position into trail ring buffer.
 * 2. Sample interpolated flow angle at tracer position.
 * 3. Step in that direction at per-particle speed.
 * 4. Wrap toroidally if off-grid.
 * 5. Update colour from new direction (cosine palette gives
 *    complementary hues for opposite directions).
 * 6. Decrement lifetime; mark dead when expired.
 */

static void tracer_advance_one_tick(tracer *t, const flow_field *f,
                                    int active_cols, int active_rows)
{
    if (!t->tracer_alive) return;

    /* 1. ring-buffer push */
    t->trail_col[t->trail_write_index] = (int)t->pos_col;
    t->trail_row[t->trail_write_index] = (int)t->pos_row;
    t->trail_write_index = (t->trail_write_index + 1) % t->trail_active_length;
    if (t->trail_filled_count < t->trail_active_length)
        t->trail_filled_count++;

    /* 2. sample */
    float angle = flow_angle_bilinear(f, t->pos_col, t->pos_row);
    t->last_angle = angle;

    /* 3. step */
    t->pos_col += cosf(angle) * t->step_cells;
    t->pos_row += sinf(angle) * t->step_cells;

    /* 4. toroidal wrap */
    if (t->pos_col < 0.0f)                  t->pos_col += (float)active_cols;
    if (t->pos_col >= (float)active_cols)   t->pos_col -= (float)active_cols;
    if (t->pos_row < 0.0f)                  t->pos_row += (float)active_rows;
    if (t->pos_row >= (float)active_rows)   t->pos_row -= (float)active_rows;

    /* 5. colour from direction */
    t->trail_pair_id = angle_to_palette_pair(angle);

    /* 6. age */
    t->ticks_until_respawn--;
    if (t->ticks_until_respawn <= 0) t->tracer_alive = false;
}

/* ===================================================================== */
/* §16  tracer_paint — fading trail render                               */
/* ===================================================================== */
/*
 * Walk the ring buffer from oldest to newest; head gets the
 * directional arrow + A_BOLD; older trail cells use a glyph ramp.
 *
 * Brightness ramp (NOTE: the project's "no A_DIM" rule applies to
 * the HUD specifically; trail attenuation here uses A_DIM as part of
 * the visual aesthetic — equivalent to alpha-blending in a graphics
 * shader.  Keeping faithful to the original visual.)
 */

#define TRAIL_RAMP_STR  ".,;+~*#"
#define TRAIL_RAMP_LEN  7

static char trail_head_glyph_for_angle(float angle_radians)
{
    static const char head_glyph_table[ARROW_OCTANT_COUNT] = {
        '-', '/', '|', '\\', '-', '/', '|', '\\'
    };
    return head_glyph_table[angle_to_octant(angle_radians)];
}

static void tracer_paint(const tracer *t, WINDOW *win,
                         int active_cols, int active_rows)
{
    if (!t->tracer_alive)            return;
    if (t->trail_filled_count == 0)  return;

    int filled = t->trail_filled_count;
    int len    = t->trail_active_length;

    for (int i = 0; i < filled; i++) {
        int slot = (t->trail_write_index - filled + i + 4 * len) % len;
        int col  = t->trail_col[slot];
        int row  = t->trail_row[slot];

        if (col < 0 || col >= active_cols) continue;
        if (row < 0 || row >= active_rows) continue;

        bool is_head = (i == filled - 1);

        /* glyph */
        char glyph;
        if (is_head) {
            glyph = trail_head_glyph_for_angle(t->last_angle);
        } else {
            int ramp_index = (i * TRAIL_RAMP_LEN)
                           / (filled > 1 ? filled : 1);
            if (ramp_index >= TRAIL_RAMP_LEN) ramp_index = TRAIL_RAMP_LEN - 1;
            glyph = TRAIL_RAMP_STR[ramp_index];
        }

        /* brightness */
        attr_t bright;
        if (is_head)                          bright = A_BOLD;
        else if (i >= filled / 4)             bright = A_NORMAL;
        else                                  bright = A_DIM;

        attr_t a = COLOR_PAIR(t->trail_pair_id) | bright;
        wattron(win, a);
        mvwaddch(win, row, col, (chtype)(unsigned char)glyph);
        wattroff(win, a);
    }
}

/* ===================================================================== */
/* §17  scene — pool + field + tick orchestrator                         */
/* ===================================================================== */

enum bg_mode {
    BG_BLANK = 0,
    BG_ARROWS,
    BG_COLORMAP,
};

typedef struct {
    flow_field flow;
    tracer     pool[TRACERS_MAX];
    int        active_tracer_count;
    int        trail_active_length;
    int        theme_index;
    int        bg_mode;
    bool       paused;
} scene_state;

static void scene_init(scene_state *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->active_tracer_count = TRACERS_DEFAULT;
    s->trail_active_length = TRAIL_LEN_DEFAULT;
    s->theme_index         = 0;
    s->bg_mode             = BG_BLANK;
    s->paused              = false;

    field_init(&s->flow, cols, rows, FIELD_KIND_CURL);
    field_evolve_and_rebuild(&s->flow);

    for (int i = 0; i < s->active_tracer_count; i++)
        tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_resize(scene_state *s, int cols, int rows)
{
    field_init(&s->flow, cols, rows, s->flow.active_kind);
    field_evolve_and_rebuild(&s->flow);
    for (int i = 0; i < TRACERS_MAX; i++)
        if (s->pool[i].tracer_alive)
            tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_reset(scene_state *s, int cols, int rows)
{
    s->flow.time_axis        = 0.0f;
    s->flow.vortex_ring_phase = 0.0f;
    field_update_vortex_positions(&s->flow);
    field_evolve_and_rebuild(&s->flow);
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
        tracer_advance_one_tick(&s->pool[i], &s->flow, cols, rows);
    }
}

/* Background painter.  Three modes (T1):
 *   BG_BLANK    — leave canvas empty
 *   BG_ARROWS   — one arrow glyph per cell, palette-coloured
 *   BG_COLORMAP — same as ARROWS but at NORMAL weight (denser look)
 *
 * Both arrow modes use A_NORMAL (no A_DIM) so the field topology
 * stays clearly visible without competing with bold tracer heads. */
static void scene_paint_background(const scene_state *s, WINDOW *win,
                                   int cols, int rows)
{
    if (s->bg_mode == BG_BLANK) return;

    for (int r = 0; r < rows; r++) {
        for (int c = 0; c < cols; c++) {
            float angle = flow_angle_at_cell(&s->flow, c, r);
            int   pair  = angle_to_palette_pair(angle);
            char  glyph = arrow_glyph_for_angle(angle);
            wattron(win, COLOR_PAIR(pair));
            mvwaddch(win, r, c, (chtype)(unsigned char)glyph);
            wattroff(win, COLOR_PAIR(pair));
        }
    }
}

static void scene_paint(const scene_state *s, WINDOW *win, int cols, int rows)
{
    scene_paint_background(s, win, cols, rows);
    for (int i = 0; i < s->active_tracer_count; i++)
        tracer_paint(&s->pool[i], win, cols, rows);
}

/* ===================================================================== */
/* §18  hud — top status + bottom hint strip (CLAUDE.md spec)            */
/* ===================================================================== */

static void hud_paint_status(WINDOW *win, int cols,
                             double fps_display, int sim_hz,
                             const scene_state *s)
{
    char buf[200];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%2dHz  tracers:%3d  trail:%2d  "
             "field:%-15s  bg:%-9s  theme:%-7s  evol:%.4f  %s ",
             fps_display, sim_hz,
             s->active_tracer_count, s->trail_active_length,
             field_kind_name_table[s->flow.active_kind],
             bg_mode_name_table[s->bg_mode],
             palette_theme_table[s->theme_index].name,
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
        " q:quit  spc:pause  r:reset  a:field  t:theme  v:bg  "
        "+/-:tracers  s/S:trail  ]/[:simHz  f/F:evol ";
    wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvwprintw(win, rows - 1, 0, "%s", hint);
    wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §19  screen — ncurses init / cleanup                                  */
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
/* §20  app — main loop + signals + input                                */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit = 1;
}

static bool app_handle_key(int key, scene_state *s, int *sim_hz,
                           int cols, int rows)
{
    switch (key) {
        case 'q': case 'Q': case 27:
            return false;

        case ' ':
            s->paused = !s->paused;
            break;

        case 'r': case 'R':
            scene_reset(s, cols, rows);
            break;

        case 'a': case 'A':
            s->flow.active_kind = (s->flow.active_kind + 1) % FIELD_KIND_COUNT;
            scene_reset(s, cols, rows);
            break;

        case 't': case 'T':
            s->theme_index = (s->theme_index + 1) % THEME_COUNT;
            colors_apply_theme(s->theme_index);
            break;

        case 'v': case 'V':
            s->bg_mode = (s->bg_mode + 1) % BG_MODE_COUNT;
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

    const int64_t frame_cap_ns = NS_PER_SEC / RENDER_FPS_CAP;

    while (!g_should_quit) {
        int64_t frame_start = clock_now_ns();

        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&scene, cols, rows);
            sim_accum_ns = 0;
        }

        int64_t dt_ns = frame_start - prev_ns;
        prev_ns       = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;

        int64_t tick_ns = NS_PER_SEC / sim_hz;
        sim_accum_ns += dt_ns;
        while (sim_accum_ns >= tick_ns) {
            scene_tick(&scene, cols, rows);
            sim_accum_ns -= tick_ns;
        }

        erase();
        scene_paint(&scene, stdscr, cols, rows);
        hud_paint_status(stdscr, cols, fps_display, sim_hz, &scene);
        hud_paint_hints (stdscr, rows);
        wnoutrefresh(stdscr);
        doupdate();

        frames_in_window++;
        window_accum_ns += dt_ns;
        if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
            fps_display = (double)frames_in_window
                        / ((double)window_accum_ns / (double)NS_PER_SEC);
            frames_in_window = 0;
            window_accum_ns  = 0;
        }

        int key;
        while ((key = getch()) != ERR) {
            if (!app_handle_key(key, &scene, &sim_hz, cols, rows)) {
                g_should_quit = 1;
                break;
            }
        }

        int64_t spent = clock_now_ns() - frame_start;
        if (spent < frame_cap_ns) clock_sleep_ns(frame_cap_ns - spent);
    }

    return 0;
}
