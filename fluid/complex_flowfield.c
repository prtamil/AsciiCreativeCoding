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
 *   ── Procedural noise (§4 perlin, §5 fbm) ──────────────────────────
 *   [1] Perlin, K. (1985), "An Image Synthesizer", SIGGRAPH 1985, pp.
 *       287-296 — THE original gradient-noise paper.  §4 implements
 *       his lattice-hash + smooth-interpolation scheme.
 *   [2] Perlin, K. (2002), "Improving noise", SIGGRAPH 2002 — the
 *       quintic-fade refresh used as our smoothstep.  Read this
 *       before §4 to see why we use 6t⁵−15t⁴+10t³ instead of the
 *       earlier cubic 3t²−2t³.
 *   [3] Mandelbrot, B. B. (1982), "The Fractal Geometry of Nature",
 *       W. H. Freeman — formalises the FBM idea our §5 uses (geometric
 *       octave summation gives a 1/f power spectrum).
 *   [4] Lagae, A. et al. (2010), "A Survey of Procedural Noise
 *       Functions", Comput. Graph. Forum 29(8), pp. 2579-2600 —
 *       comprehensive survey if you want alternatives (simplex,
 *       wavelet, sparse-convolution noise).
 *
 *   ── Curl noise / divergence-free fields (§9 field_curl) ───────────
 *   [5] Bridson, R., Hourihan, J. & Nordenstam, M. (2007), "Curl-Noise
 *       for Procedural Fluid Flow", SIGGRAPH 2007 — THE curl-noise
 *       paper.  Defines u = ∇×ψ.  Why our particles look like real
 *       fluid (incompressibility ⇒ no piling-up).
 *
 *   ── Vortex dynamics / Biot-Savart (§10 field_vortex) ──────────────
 *   [6] Helmholtz, H. (1858), "On Integrals of the Hydrodynamical
 *       Equations which Express Vortex-Motion", Phil. Mag. 33 —
 *       historical origin of the vortex theorems behind §10.
 *   [7] Saffman, P. G. (1992), "Vortex Dynamics", Cambridge UP — point-
 *       vortex theory + Biot-Savart in fluid context.  Reference for
 *       the 1/r velocity field around each vortex in §10.
 *   [8] Aref, H. (1983), "Integrable, Chaotic, and Turbulent Vortex
 *       Motion in 2D Flows", Annu. Rev. Fluid Mech. 15, pp. 345-389
 *       — N-vortex behaviour as the count grows past 3.
 *
 *   ── Wave superposition (§11 field_sine) ───────────────────────────
 *   [9] Crawford, F. S. (1968), "Waves", Berkeley Physics Course Vol.
 *       III, ch. 2 — sine-wave superposition basics; explains why
 *       crossed sines in §11 produce a moiré-like interference grid.
 *
 *   ── Palettes & glyph design (§6 cosine_palette, §13 arrows) ───────
 *  [10] Quilez, I., "Palettes" — iquilezles.org/articles/palettes;
 *       the cosine-palette formula color = a + b·cos(2π(c·t + d))
 *       implemented in §6.  Six themes are six (a,b,c,d) rows.
 *  [11] Quilez, I., "Domain Warping" — iquilezles.org/articles/warp;
 *       the noise-warp technique that inspired the time-varying
 *       phase wobble used in §9.
 *  [12] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the 8-octant ASCII arrow ramp in §13.
 *
 *   ── ncurses rendering ─────────────────────────────────────────────
 *  [13] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the newscr/curscr diff pipeline used
 *       in scene_paint() → wnoutrefresh() → doupdate().
 *
 *   ── Online quick reference ────────────────────────────────────────
 *  [14] https://en.wikipedia.org/wiki/Perlin_noise
 *  [15] https://en.wikipedia.org/wiki/Biot%E2%80%93Savart_law
 *  [16] https://en.wikipedia.org/wiki/Helmholtz_decomposition
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
#define M_PI 3.14159265358979323846
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
  RENDER_FPS_CAP = 60,

  /* Simulation step rate. */
  SIM_HZ_MIN = 5,
  SIM_HZ_DEFAULT = 30,
  SIM_HZ_MAX = 60,
  SIM_HZ_STEP = 5,

  /* HUD recompute cadence. */
  FPS_RECOMPUTE_MS = 500,

  /* Tracer pool. */
  TRACERS_MIN = 50,
  TRACERS_DEFAULT = 400,
  TRACERS_MAX = 800,
  TRACERS_STEP = 50,

  /* Trail ring buffer (per tracer). */
  TRAIL_LEN_MIN = 3,
  TRAIL_LEN_DEFAULT = 18,
  TRAIL_LEN_MAX = 24,
  TRAIL_LEN_HARD_MAX = 24, /* compile-time array size */

  /* FBM octaves used by the curl-noise generator. */
  CURL_FBM_OCTAVES = 3,

  /* Cosine palette: 16 evenly-spaced samples become ncurses pairs. */
  PALETTE_PAIR_COUNT = 16,

  /* Number of themes. */
  THEME_COUNT = 6,

  /* Number of background modes (blank / arrows / colormap). */
  BG_MODE_COUNT = 3,

  /* Number of field generator kinds (curl / vortex / sine / spiral). */
  FIELD_KIND_COUNT = 4,

  /* Vortex-lattice config. */
  VORTEX_COUNT = 6,
};

/* PHYSICAL / VISUAL CONSTANTS — units in the comments. */

/* Tracer step distance (cells per tick) with per-particle jitter. */
#define TRACER_STEP_BASE_CPT 0.85f
#define TRACER_STEP_JITTER_CPT 0.40f

/* Tracer lifetime (ticks before forced respawn) with jitter. */
#define TRACER_LIFE_BASE_TICKS 120
#define TRACER_LIFE_JITTER_TICKS 80

/* Field time-axis evolution (per tick). */
#define FIELD_EVOLUTION_DEFAULT 0.006f
#define FIELD_EVOLUTION_MIN 0.001f
#define FIELD_EVOLUTION_MAX 0.080f
#define FIELD_EVOLUTION_FACTOR 1.5f /* multiplier for f/F keys */

/* Curl-noise spatial scales + central-difference epsilon. */
#define CURL_NOISE_SCALE_X 0.030f
#define CURL_NOISE_SCALE_Y 0.055f
#define CURL_DIFFERENCE_EPSILON 1.20f

/* Vortex-lattice geometry. */
#define VORTEX_RING_RADIUS_FRAC 0.28f  /* fraction of min(W,H)/2 */
#define VORTEX_RING_ORBIT_SPEED 0.014f /* radians per tick */
#define VORTEX_STRENGTH_GAMMA 3.0f
#define VORTEX_SOFTEN_PIXELS 5.0f /* added to r² for stability */

/* Sine-lattice spatial frequencies. */
#define SINE_FREQ_X 0.055f
#define SINE_FREQ_Y 0.095f

/* Radial-spiral pulse amplitude. */
#define SPIRAL_RADIAL_WEIGHT 0.65f

/* Aspect-ratio compensation for vortex / spiral (T8). */
#define ASPECT_FACTOR 0.5f

/* Time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* Colour pair IDs. */
enum {
  PAIR_PALETTE_BASE = 1, /* +0..+15 */
  PAIR_HUD = PAIR_PALETTE_BASE + PALETTE_PAIR_COUNT,
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
    "curl-noise", "vortex-lattice", "sine-lattice", "radial-spiral"};

/* Background mode names. */
static const char *bg_mode_name_table[BG_MODE_COUNT] = {"blank", "arrows",
                                                        "colormap"};

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
/* §3  rng — Fisher-Yates shuffle for the perlin permutation table       */
/* ===================================================================== */
/*
 * Perlin noise's pseudo-randomness comes entirely from this 256-byte
 * table.  We seed rand() once at startup, build the table once, and
 * never touch it again.
 */

static uint8_t perlin_perm_table[512];

static void perlin_perm_init(void) {
  uint8_t identity[256];
  for (int i = 0; i < 256; i++)
    identity[i] = (uint8_t)i;
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

static inline float smoothstep_cubic(float t) {
  return t * t * (3.0f - 2.0f * t);
}

static inline float lerp_scalar(float a, float b, float t) {
  return a + t * (b - a);
}

static inline float perlin_gradient_dot(int hash, float x, float y) {
  int bits = hash & 3;
  float term1 = (bits < 2) ? x : y;
  float term2 = (bits < 2) ? y : x;
  return ((hash & 1) ? -term1 : term1) + ((hash & 2) ? -term2 : term2);
}

static float perlin_value(float x, float y) {
  int xi = (int)floorf(x) & 255;
  int yi = (int)floorf(y) & 255;
  float fx = x - floorf(x);
  float fy = y - floorf(y);
  float ux = smoothstep_cubic(fx);
  float uy = smoothstep_cubic(fy);

  int h00 = perlin_perm_table[perlin_perm_table[xi] + yi];
  int h10 = perlin_perm_table[perlin_perm_table[xi + 1] + yi];
  int h01 = perlin_perm_table[perlin_perm_table[xi] + yi + 1];
  int h11 = perlin_perm_table[perlin_perm_table[xi + 1] + yi + 1];

  float d00 = perlin_gradient_dot(h00, fx, fy);
  float d10 = perlin_gradient_dot(h10, fx - 1.0f, fy);
  float d01 = perlin_gradient_dot(h01, fx, fy - 1.0f);
  float d11 = perlin_gradient_dot(h11, fx - 1.0f, fy - 1.0f);

  float top = lerp_scalar(d00, d10, ux);
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

static float fbm_value(float x, float y, float t, int octaves) {
  float result = 0.0f;
  float amplitude = 1.0f;
  float frequency = 1.0f;
  for (int oct = 0; oct < octaves; oct++) {
    result +=
        perlin_value(x * frequency + t, y * frequency + t * 0.7f) * amplitude;
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

/*
 * CosinePaletteTheme — one row of the cosine-palette table (§6).
 *
 * Intent
 *   Quilez's cosine palette [10] computes ONE rgb colour from a
 *   scalar t with one formula:
 *
 *       color(t) = bias + amplitude · cos( 2π · (frequency · t + phase) )
 *
 *   Each row of the table is FOUR 3-vectors (a, b, c, d) — one set
 *   of bias / amplitude / frequency / phase per rgb channel.  By
 *   varying these four vectors we get qualitatively different colour
 *   journeys (cosmic, ember, ocean, ...) from the same math.
 *
 * Why this shape (4 separate float[3] arrays + a name)
 *   Quilez expresses the palette as 4 rgb vectors, so a parallel
 *   layout makes the table read like the formula 1:1.  Alternative
 *   `float coeffs[4][3]` would compile to identical code but be
 *   harder to author by hand and harder to diff.
 *
 * Reference [10] Quilez, "Palettes" — iquilezles.org/articles/palettes
 *   gives the geometric intuition (rgb cosine sweeps trace out smooth
 *   perceptual gradients) and tabulates sample (a, b, c, d) vectors
 *   that we adapt for the 6 themes below.
 */
typedef struct {
    float       bias_rgb[3];      /* a — channel-wise mean colour       */
    float       amplitude_rgb[3]; /* b — channel-wise oscillation amp   */
    float       frequency_rgb[3]; /* c — cycles across t ∈ [0, 1]       */
    float       phase_rgb[3];     /* d — channel-wise phase offset 0..1 */
    const char *name;             /* short ASCII label shown in HUD     */
} CosinePaletteTheme;

/* Six themes — feel-targets in the comment. */
static const CosinePaletteTheme palette_theme_table[THEME_COUNT] = {
    /* 0 cosmic — electric violet → cyan → hot magenta */
    {{0.50f, 0.50f, 0.50f},
     {0.50f, 0.50f, 0.50f},
     {1.00f, 1.00f, 0.50f},
     {0.80f, 0.90f, 0.30f},
     "cosmic"},

    /* 1 ember — deep red → orange → pale yellow */
    {{0.55f, 0.30f, 0.05f},
     {0.45f, 0.30f, 0.05f},
     {1.00f, 0.80f, 0.30f},
     {0.00f, 0.10f, 0.25f},
     "ember"},

    /* 2 ocean — navy → teal → ice */
    {{0.15f, 0.40f, 0.60f},
     {0.20f, 0.35f, 0.40f},
     {0.50f, 0.70f, 1.00f},
     {0.00f, 0.10f, 0.30f},
     "ocean"},

    /* 3 neon — electric green → hot pink → violet */
    {{0.50f, 0.50f, 0.50f},
     {0.50f, 0.50f, 0.50f},
     {1.00f, 0.50f, 1.00f},
     {0.00f, 0.50f, 0.33f},
     "neon"},

    /* 4 sunset — purple → crimson → amber */
    {{0.50f, 0.38f, 0.30f},
     {0.50f, 0.38f, 0.30f},
     {1.00f, 0.85f, 0.60f},
     {0.00f, 0.18f, 0.40f},
     "sunset"},

    /* 5 mono — silver-blue clean grayscale */
    {{0.45f, 0.48f, 0.55f},
     {0.40f, 0.42f, 0.45f},
     {0.50f, 0.50f, 0.50f},
     {0.00f, 0.02f, 0.05f},
     "mono"},
};

/* Evaluate the cosine palette at parameter t ∈ [0, 1] → (R, G, B).
 * Each channel ∈ [0, 1] after clamping. */
static void cosine_palette_eval(const CosinePaletteTheme *theme,
                                float palette_param, float *out_r, float *out_g,
                                float *out_b) {
  float r =
      theme->bias_rgb[0] +
      theme->amplitude_rgb[0] *
          cosf(2.0f * (float)M_PI *
               (theme->frequency_rgb[0] * palette_param + theme->phase_rgb[0]));
  float g =
      theme->bias_rgb[1] +
      theme->amplitude_rgb[1] *
          cosf(2.0f * (float)M_PI *
               (theme->frequency_rgb[1] * palette_param + theme->phase_rgb[1]));
  float b =
      theme->bias_rgb[2] +
      theme->amplitude_rgb[2] *
          cosf(2.0f * (float)M_PI *
               (theme->frequency_rgb[2] * palette_param + theme->phase_rgb[2]));
  if (r < 0.0f)
    r = 0.0f;
  else if (r > 1.0f)
    r = 1.0f;
  if (g < 0.0f)
    g = 0.0f;
  else if (g > 1.0f)
    g = 1.0f;
  if (b < 0.0f)
    b = 0.0f;
  else if (b > 1.0f)
    b = 1.0f;
  *out_r = r;
  *out_g = g;
  *out_b = b;
}

/* Map an (R, G, B) ∈ [0, 1]³ to the closest xterm-256 cube colour
 * (indices 16..231 = 6×6×6 = 216 colours). */
static int rgb_to_xterm256_cube(float r, float g, float b) {
  int r5 = (int)(r * 5.0f + 0.5f);
  int g5 = (int)(g * 5.0f + 0.5f);
  int b5 = (int)(b * 5.0f + 0.5f);
  if (r5 > 5)
    r5 = 5;
  if (r5 < 0)
    r5 = 0;
  if (g5 > 5)
    g5 = 5;
  if (g5 < 0)
    g5 = 0;
  if (b5 > 5)
    b5 = 5;
  if (b5 < 0)
    b5 = 0;
  return 16 + 36 * r5 + 6 * g5 + b5;
}

/* 8-colour fallback (one row per theme). */
static const int palette_fallback_8[THEME_COUNT][PALETTE_PAIR_COUNT] = {
    {COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_CYAN, COLOR_BLUE, COLOR_WHITE, COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE,
     COLOR_WHITE, COLOR_MAGENTA, COLOR_CYAN, COLOR_BLUE, COLOR_WHITE},
    {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_RED, COLOR_RED,
     COLOR_YELLOW, COLOR_WHITE, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE,
     COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
    {COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_BLUE, COLOR_CYAN,
     COLOR_CYAN, COLOR_WHITE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
     COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
    {COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE, COLOR_GREEN,
     COLOR_MAGENTA, COLOR_BLUE, COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA,
     COLOR_BLUE, COLOR_WHITE, COLOR_GREEN, COLOR_MAGENTA, COLOR_BLUE,
     COLOR_WHITE},
    {COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_RED, COLOR_YELLOW, COLOR_WHITE, COLOR_MAGENTA, COLOR_RED,
     COLOR_YELLOW, COLOR_WHITE, COLOR_MAGENTA, COLOR_RED, COLOR_YELLOW,
     COLOR_WHITE},
    {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE},
};

/* ===================================================================== */
/* §7  colors — ncurses pair init + angle → palette pair                 */
/* ===================================================================== */

static void colors_apply_theme(int theme_index) {
  const CosinePaletteTheme *theme = &palette_theme_table[theme_index];
  bool has_256 = (COLORS >= 256);

  for (int i = 0; i < PALETTE_PAIR_COUNT; i++) {
    float palette_param = (float)i / (float)(PALETTE_PAIR_COUNT - 1);
    if (has_256) {
      float r, g, b;
      cosine_palette_eval(theme, palette_param, &r, &g, &b);
      int fg = rgb_to_xterm256_cube(r, g, b);
      init_pair(PAIR_PALETTE_BASE + i, fg, -1);
    } else {
      init_pair(PAIR_PALETTE_BASE + i, palette_fallback_8[theme_index][i], -1);
    }
  }

  /* HUD per CLAUDE.md: bright yellow + bright cyan, default bg. */
  if (has_256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();
  colors_apply_theme(theme_index);
}

/* angle_to_palette_pair — map a flow angle ∈ (-π, π] to one of the
 * PALETTE_PAIR_COUNT palette pairs.  Adjacent angles get adjacent
 * pairs; opposite angles get half-cycle-apart palette colours
 * (perceptually complementary). */
static int angle_to_palette_pair(float angle_radians) {
  float a = angle_radians;
  if (a < 0.0f)
    a += 2.0f * (float)M_PI;
  int idx =
      (int)(a / (2.0f * (float)M_PI) * PALETTE_PAIR_COUNT) % PALETTE_PAIR_COUNT;
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
#define FIELD_ROWS_MAX 80

/*
 * flow_field — the vector field that drives every tracer.
 *
 * Intent
 *   Every generator (§9 curl, §10 vortex, §11 sine, §12 spiral) emits
 *   the SAME observable: an angle θ(x, y, t) at every grid cell.
 *   Tracers don't know which generator filled angle[][] — they just
 *   bilinearly sample it and step in that direction.  This decoupling
 *   is the architectural lesson (T2): the field is a function-pointer-
 *   selectable black box.
 *
 * Why angle is precomputed on a grid (not evaluated per-tracer)
 *   With N tracers and a W·H grid, grid evaluation costs O(W·H)
 *   while per-tracer evaluation costs O(N).  For our defaults
 *   (W·H ≈ 16k cells, N ≈ 400 tracers) the grid wins by ~40× — AND
 *   bilinear sampling gives sub-cell smoothness for free.
 *
 * Why vortex auxiliary state lives here
 *   §10 needs N vortices' positions + strengths over time.  Keeping
 *   them on the field makes the generator stateless from the
 *   tracer's point of view, while still letting the vortices drift /
 *   rotate as the field clock advances.  See [7] Saffman §1.2 for
 *   the underlying point-vortex theory.
 *
 * References [5] Bridson curl-noise; [7] Saffman vortex dynamics.
 */
typedef struct {
    /* ── Active subregion (≤ FIELD_*_MAX, clamped on resize) ────── */
    int active_cols;
    int active_rows;

    /* ── Which generator currently fills angle[][] ──────────────── *
     * One of FIELD_KIND_CURL / VORTEX / SINE / SPIRAL.  Changed by  *
     * the 'a' key; scene_tick reads the new generator next tick.   */
    int active_kind;

    /* ── Field clock + evolution speed ──────────────────────────── *
     * time_axis advances by evolution_speed each tick.  Generators *
     * use it as the `t` in f(x, y, t).  Independent of frame-time  *
     * so 'f'/'F' can speed up / slow down evolution without        *
     * affecting tracer integration.                                */
    float time_axis;
    float evolution_speed;

    /* ── Per-cell flow angle, radians (filled each tick by active *
     * generator).  Tracers read this via bilinear sampling — they *
     * never call generators directly.                              */
    float angle[FIELD_ROWS_MAX][FIELD_COLS_MAX];

    /* ── Vortex-lattice auxiliary state (consumed by §10 only) ──── *
     * Other generators ignore these.  Kept on the field rather    *
     * than file-scope so vortex strength + phase survive resize    *
     * and theme/field switches.                                    */
    float vortex_pos_col      [VORTEX_COUNT];
    float vortex_pos_row      [VORTEX_COUNT];
    float vortex_strength_table[VORTEX_COUNT];
    float vortex_ring_phase;       /* slowly increases — ring rotation */
} flow_field;

/* Forward declarations of generators — defined in §9-§12. */
static float field_curl_angle(const flow_field *f, float x, float y, float t);
static float field_vortex_angle(const flow_field *f, float x, float y);
static float field_sine_angle(const flow_field *f, float x, float y, float t);
static float field_spiral_angle(const flow_field *f, float x, float y, float t);

static void field_update_vortex_positions(flow_field *f) {
  /* Place N vortices on a ring centred on the screen centre.  Each
   * frame the ring slowly rotates by VORTEX_RING_ORBIT_SPEED. */
  float centre_col = (float)f->active_cols * 0.5f;
  float centre_row = (float)f->active_rows * 0.5f;
  int smaller =
      (f->active_cols < f->active_rows) ? f->active_cols : f->active_rows;
  float radius = VORTEX_RING_RADIUS_FRAC * (float)smaller * 0.5f;

  for (int i = 0; i < VORTEX_COUNT; i++) {
    float angle = f->vortex_ring_phase +
                  (float)i * (2.0f * (float)M_PI / (float)VORTEX_COUNT);
    f->vortex_pos_col[i] = centre_col + radius * cosf(angle);
    /* y-component scaled by ASPECT_FACTOR so the ring is VISUALLY
     * round on a 2:1-tall cell grid (T8). */
    f->vortex_pos_row[i] = centre_row + radius * sinf(angle) * ASPECT_FACTOR;
  }
}

static void field_init(flow_field *f, int cols, int rows, int kind) {
  if (cols > FIELD_COLS_MAX)
    cols = FIELD_COLS_MAX;
  if (rows > FIELD_ROWS_MAX)
    rows = FIELD_ROWS_MAX;
  f->active_cols = cols;
  f->active_rows = rows;
  f->active_kind = kind;
  f->time_axis = 0.0f;
  f->evolution_speed = FIELD_EVOLUTION_DEFAULT;
  f->vortex_ring_phase = 0.0f;
  memset(f->angle, 0, sizeof f->angle);

  /* Alternate vortex strength signs (CCW, CW, CCW, ...) for a
   * dynamic between adjacent vortices. */
  for (int i = 0; i < VORTEX_COUNT; i++)
    f->vortex_strength_table[i] =
        (i & 1) ? -VORTEX_STRENGTH_GAMMA : VORTEX_STRENGTH_GAMMA;

  field_update_vortex_positions(f);
}

/* field_evolve_and_rebuild — advance time, then recompute every cell.
 * The DISPATCHER for the four field kinds. */
static void field_evolve_and_rebuild(flow_field *f) {
  f->time_axis += f->evolution_speed;
  f->vortex_ring_phase += VORTEX_RING_ORBIT_SPEED;
  field_update_vortex_positions(f);

  for (int r = 0; r < f->active_rows; r++) {
    for (int c = 0; c < f->active_cols; c++) {
      float angle;
      switch (f->active_kind) {
      case FIELD_KIND_CURL:
        angle = field_curl_angle(f, (float)c, (float)r, f->time_axis);
        break;
      case FIELD_KIND_VORTEX:
        angle = field_vortex_angle(f, (float)c, (float)r);
        break;
      case FIELD_KIND_SINE:
        angle = field_sine_angle(f, (float)c, (float)r, f->time_axis);
        break;
      case FIELD_KIND_SPIRAL:
      default:
        angle = field_spiral_angle(f, (float)c, (float)r, f->time_axis);
        break;
      }
      f->angle[r][c] = angle;
    }
  }
}

/* flow_angle_at_cell — the angle stored at integer cell (c, r). */
static inline float flow_angle_at_cell(const flow_field *f, int c, int r) {
  if (c < 0)
    c = 0;
  if (c >= f->active_cols)
    c = f->active_cols - 1;
  if (r < 0)
    r = 0;
  if (r >= f->active_rows)
    r = f->active_rows - 1;
  return f->angle[r][c];
}

/* Clamp an integer cell index to the active field bounds.  Used
 * everywhere the bilinear stencil reaches one cell past the edge. */
static inline int clamp_to_active(int idx, int max_count) {
    if (idx < 0)              return 0;
    if (idx >= max_count)     return max_count - 1;
    return idx;
}

/* The fractional offsets within the cell at floor(col), floor(row).
 * Both are in [0, 1) — they become the bilinear blend weights. */
static inline void compute_subcell_offsets(float col, float row,
                                           float *out_fx, float *out_fy) {
    *out_fx = col - floorf(col);
    *out_fy = row - floorf(row);
}

/* Bilinear interpolation of a scalar at fractional position (fx, fy)
 * given the four corner values (top-left, top-right, bottom-left,
 * bottom-right).  Two-stage lerp: blend horizontally on each row, then
 * blend vertically.  Output is a weighted average of the four
 * corners, hence between their min and max. */
static inline float bilinear_lerp_4corners(float tl, float tr,
                                            float bl, float br,
                                            float fx, float fy) {
    float top    = lerp_scalar(tl, tr, fx);
    float bottom = lerp_scalar(bl, br, fx);
    return lerp_scalar(top, bottom, fy);
}

/*
 * flow_angle_bilinear — interpolated flow angle at sub-cell (col, row).
 *
 * Pseudocode:
 *   c0 = clamp(floor(col)),       c1 = clamp(c0 + 1)        ← cell corners
 *   r0 = clamp(floor(row)),       r1 = clamp(r0 + 1)
 *   (fx, fy) = compute_subcell_offsets(col, row)            ← weights
 *   return bilinear_lerp_4corners(angle[r0][c0], angle[r0][c1],
 *                                 angle[r1][c0], angle[r1][c1],
 *                                 fx, fy)
 */
static float flow_angle_bilinear(const flow_field *f, float col, float row) {
    int   c0 = clamp_to_active((int)floorf(col),     f->active_cols);
    int   c1 = clamp_to_active((int)floorf(col) + 1, f->active_cols);
    int   r0 = clamp_to_active((int)floorf(row),     f->active_rows);
    int   r1 = clamp_to_active((int)floorf(row) + 1, f->active_rows);

    float fx, fy;
    compute_subcell_offsets(col, row, &fx, &fy);

    return bilinear_lerp_4corners(f->angle[r0][c0], f->angle[r0][c1],
                                   f->angle[r1][c0], f->angle[r1][c1],
                                   fx, fy);
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

static float field_curl_angle(const flow_field *f, float x, float y, float t) {
  (void)f; /* curl uses only Perlin state, not field state */
  float eps = CURL_DIFFERENCE_EPSILON;

  float psi_north =
      fbm_value(x * CURL_NOISE_SCALE_X, y * CURL_NOISE_SCALE_Y + eps, t,
                CURL_FBM_OCTAVES);
  float psi_south =
      fbm_value(x * CURL_NOISE_SCALE_X, y * CURL_NOISE_SCALE_Y - eps, t,
                CURL_FBM_OCTAVES);
  float psi_east = fbm_value(x * CURL_NOISE_SCALE_X + eps,
                             y * CURL_NOISE_SCALE_Y, t, CURL_FBM_OCTAVES);
  float psi_west = fbm_value(x * CURL_NOISE_SCALE_X - eps,
                             y * CURL_NOISE_SCALE_Y, t, CURL_FBM_OCTAVES);

  /* curl(ψ) = (∂ψ/∂y, -∂ψ/∂x) via central differences. */
  float vx = (psi_north - psi_south); /* /(2ε) cancels in atan2 */
  float vy = -(psi_east - psi_west);
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

static float field_vortex_angle(const flow_field *f, float x, float y) {
  float vx_total = 0.0f;
  float vy_total = 0.0f;

  for (int i = 0; i < VORTEX_COUNT; i++) {
    float dx = x - f->vortex_pos_col[i];
    float dy = (y - f->vortex_pos_row[i]) / ASPECT_FACTOR;
    float r2 = dx * dx + dy * dy + VORTEX_SOFTEN_PIXELS;
    float strength = f->vortex_strength_table[i];
    vx_total += strength * (-dy) / r2;
    vy_total += strength * (dx) / r2;
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

static float field_sine_angle(const flow_field *f, float x, float y, float t) {
  (void)f;
  float vx = sinf(x * SINE_FREQ_X + t) + sinf(y * SINE_FREQ_Y - t * 0.7f);
  float vy =
      cosf(x * SINE_FREQ_X - t * 0.5f) + cosf(y * SINE_FREQ_Y + t * 0.3f);
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

static float field_spiral_angle(const flow_field *f, float x, float y,
                                float t) {
  float centre_col = (float)f->active_cols * 0.5f;
  float centre_row = (float)f->active_rows * 0.5f;
  float dx = x - centre_col;
  float dy = (y - centre_row) / ASPECT_FACTOR;
  float radius = sqrtf(dx * dx + dy * dy);
  if (radius < 1e-4f)
    return 0.0f; /* avoid centre singularity */

  float theta = atan2f(dy, dx);
  float pulse = SPIRAL_RADIAL_WEIGHT * sinf(t * 0.8f);

  float vx = -sinf(theta) + pulse * cosf(theta);
  float vy = cosf(theta) + pulse * sinf(theta);
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
    '>',  /* 0  E   */
    '/',  /* 1  NE  */
    '^',  /* 2  N   */
    '\\', /* 3  NW  */
    '<',  /* 4  W   */
    '/',  /* 5  SW  */
    'v',  /* 6  S   */
    '\\', /* 7  SE  */
};

static int angle_to_octant(float angle_radians) {
  float a = angle_radians;
  if (a < 0.0f)
    a += 2.0f * (float)M_PI;
  return (int)(a / (2.0f * (float)M_PI) * ARROW_OCTANT_COUNT + 0.5f) %
         ARROW_OCTANT_COUNT;
}

static inline char arrow_glyph_for_angle(float angle_radians) {
  return arrow_glyph_table[angle_to_octant(angle_radians)];
}

/* ===================================================================== */
/* §14  tracer — one particle: pose + trail ring buffer                  */
/* ===================================================================== */
/*
 * Same struct shape as fluid/flowfield.c §10.  Re-read that file's
 * tutorial T7 if ring-buffer trails are unfamiliar.
 */

/*
 * tracer — one particle: pose + ring-buffer trail.
 *
 * Intent
 *   A tracer is a PASSIVE LAGRANGIAN PARTICLE: each tick it samples
 *   the field at its current position, advances in that direction by
 *   step_cells, and pushes its old position onto a fixed-length
 *   ring-buffer trail.  scene_paint reads the ring back into a
 *   fading-glow rendering (newest = brightest, oldest = faintest).
 *
 * Why a fixed-length ring buffer
 *   Trails have a maximum visible length, so static
 *   trail_col[]/trail_row[] arrays avoid ALL allocation in the hot
 *   path.  trail_write_index walks circularly mod trail_active_length;
 *   trail_filled_count saturates once the buffer is full.  Static
 *   sizing matches CLAUDE.md "no dynamic allocation after init".
 *
 * Why ticks_until_respawn exists
 *   Tracers that walk off-grid don't teleport — they FADE, then wait
 *   a small random number of ticks (per-tracer), then respawn at a
 *   fresh random position.  Random offsets prevent all respawns from
 *   happening on the same frame ("popping" patterns).
 *
 * Why trail_pair_id is captured at respawn (not per-tick)
 *   Each tracer keeps the SAME palette pair for its entire life.
 *   Re-sampling the palette every tick would make adjacent particles
 *   strobe.  Locking the colour to the spawn-angle gives the
 *   pleasing "streams of coherent colour" look.
 */
typedef struct {
    /* ── Life cycle ──────────────────────────────────────────────── */
    bool  tracer_alive;
    int   ticks_until_respawn;     /* counts down once tracer dies     */

    /* ── Current pose (continuous; bilinear-sampled in field) ────── */
    float pos_col;
    float pos_row;
    float step_cells;              /* per-tick advance (cells/tick)    */
    float last_angle;              /* last sampled θ (drives glyph)    */

    /* ── Trail ring buffer (fixed-size; no allocation in hot path) ─ */
    int   trail_pair_id;           /* palette pair LOCKED at respawn   */
    int   trail_col[TRAIL_LEN_HARD_MAX];
    int   trail_row[TRAIL_LEN_HARD_MAX];
    int   trail_write_index;       /* head index, mod active length    */
    int   trail_filled_count;      /* up to trail_active_length        */
    int   trail_active_length;     /* current window into the ring     */
} tracer;

/* Uniform random spawn point inside the active field rect. */
static inline void pick_random_spawn_position(int active_cols, int active_rows,
                                              float *out_col, float *out_row) {
    *out_col = (float)(rand() % active_cols);
    *out_row = (float)(rand() % active_rows);
}

/* Step speed in cells/tick with ±50% jitter around the base.  Jitter
 * desynchronises adjacent tracers so the flow looks textured rather
 * than gridded. */
static inline float pick_jittered_step_speed_cpt(void) {
    return TRACER_STEP_BASE_CPT
         - TRACER_STEP_JITTER_CPT * 0.5f
         + TRACER_STEP_JITTER_CPT * ((float)rand() / (float)RAND_MAX);
}

/* Lifetime in ticks: BASE + uniform[0, JITTER).  Random offsets prevent
 * all tracers from dying on the same frame ("popping" artefact). */
static inline int pick_random_lifetime_ticks(void) {
    return TRACER_LIFE_BASE_TICKS + rand() % TRACER_LIFE_JITTER_TICKS;
}

/* Pick a fresh colour pair for the tracer's whole life — locked at
 * respawn so adjacent particles don't strobe (see tracer struct doc). */
static inline int pick_random_palette_pair(void) {
    return PAIR_PALETTE_BASE + rand() % PALETTE_PAIR_COUNT;
}

/* Reset the ring buffer so the tracer's INITIAL trail is a row of
 * spawn-position duplicates (no stale data from a previous life).
 * trail_active_length is the WINDOW size; the underlying buffer is
 * the hard-max array. */
static inline void reset_trail_ring_at(tracer *t,
                                       int initial_col, int initial_row,
                                       int trail_active_length) {
    t->trail_write_index   = 0;
    t->trail_filled_count  = 0;
    t->trail_active_length = trail_active_length;
    for (int i = 0; i < TRAIL_LEN_HARD_MAX; i++) {
        t->trail_col[i] = initial_col;
        t->trail_row[i] = initial_row;
    }
}

/*
 * tracer_respawn — give a dead tracer a fresh life.
 *
 * Pseudocode:
 *   alive ← true
 *   (pos_col, pos_row)    ← pick_random_spawn_position(...)
 *   step_cells            ← pick_jittered_step_speed_cpt()
 *   last_angle            ← 0
 *   ticks_until_respawn   ← pick_random_lifetime_ticks()
 *   trail_pair_id         ← pick_random_palette_pair()
 *   reset the ring buffer to be a row of spawn-position duplicates
 */
static void tracer_respawn(tracer *t, int active_cols, int active_rows,
                           int trail_active_length) {
    t->tracer_alive        = true;
    pick_random_spawn_position(active_cols, active_rows,
                               &t->pos_col, &t->pos_row);
    t->step_cells          = pick_jittered_step_speed_cpt();
    t->last_angle          = 0.0f;
    t->ticks_until_respawn = pick_random_lifetime_ticks();
    t->trail_pair_id       = pick_random_palette_pair();
    reset_trail_ring_at(t, (int)t->pos_col, (int)t->pos_row,
                        trail_active_length);
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

/* Push the tracer's CURRENT integer position into the ring buffer.
 * trail_write_index advances mod trail_active_length; trail_filled_
 * count saturates at trail_active_length.  See tracer struct doc. */
static inline void push_position_into_trail(tracer *t) {
    t->trail_col[t->trail_write_index] = (int)t->pos_col;
    t->trail_row[t->trail_write_index] = (int)t->pos_row;
    t->trail_write_index = (t->trail_write_index + 1) % t->trail_active_length;
    if (t->trail_filled_count < t->trail_active_length)
        t->trail_filled_count++;
}

/* Sample the underlying vector field θ(x, y) at the tracer's current
 * position with bilinear interpolation.  Caches the result on the
 * tracer so the head glyph (paint pass) reads the SAME angle the
 * particle just travelled. */
static inline float sample_field_at_tracer(tracer *t, const flow_field *f) {
    float angle = flow_angle_bilinear(f, t->pos_col, t->pos_row);
    t->last_angle = angle;
    return angle;
}

/* Lagrangian advection: advance the tracer one step_cells in the
 * direction (cos θ, sin θ).  This is the forward-Euler integration
 * of dx/dt = v, dy/dt = v with v = step_cells · (cos θ, sin θ).
 * See [7] Witkin 1991 §3 on Lagrangian particles in prescribed flow. */
static inline void step_tracer_along_angle(tracer *t, float angle) {
    t->pos_col += cosf(angle) * t->step_cells;
    t->pos_row += sinf(angle) * t->step_cells;
}

/* Toroidal wrap-around: particles leaving the right edge re-enter
 * from the left, etc.  Equivalent to a 2-D torus topology; avoids the
 * "tracer evaporated" failure mode that would happen with a hard
 * boundary + no respawn check. */
static inline void wrap_position_toroidally(tracer *t,
                                            int active_cols, int active_rows) {
    if (t->pos_col <  0.0f)              t->pos_col += (float)active_cols;
    if (t->pos_col >= (float)active_cols) t->pos_col -= (float)active_cols;
    if (t->pos_row <  0.0f)              t->pos_row += (float)active_rows;
    if (t->pos_row >= (float)active_rows) t->pos_row -= (float)active_rows;
}

/* Recolour the tracer's TRAIL from its current direction.  Using
 * direction-based colour means opposite-flowing particles get
 * complementary hues (the cosine palette wraps angle → colour); see §6
 * cosine palette doc. */
static inline void recolour_tracer_from_angle(tracer *t, float angle) {
    t->trail_pair_id = angle_to_palette_pair(angle);
}

/* Age the tracer by one tick and mark it dead when its lifetime
 * expires.  scene_tick respawns dead tracers via tracer_respawn. */
static inline void age_tracer_and_check_death(tracer *t) {
    t->ticks_until_respawn--;
    if (t->ticks_until_respawn <= 0)
        t->tracer_alive = false;
}

/*
 * tracer_advance_one_tick — one Lagrangian-advection step of one tracer.
 *
 * Pseudocode:
 *   if not alive: skip
 *   push_position_into_trail(t)                        (ring-buffer)
 *   angle = sample_field_at_tracer(t, field)           (bilinear)
 *   step_tracer_along_angle(t, angle)                  (forward Euler)
 *   wrap_position_toroidally(t, cols, rows)            (torus topology)
 *   recolour_tracer_from_angle(t, angle)               (cosine palette)
 *   age_tracer_and_check_death(t)                      (lifetime)
 *
 * References [7] Witkin 1991 for the Lagrangian particle pattern;
 *   [5] Bridson curl-noise for the divergence-free regime where this
 *   particle never piles up.
 */
static void tracer_advance_one_tick(tracer *t, const flow_field *f,
                                    int active_cols, int active_rows) {
    if (!t->tracer_alive)
        return;

    push_position_into_trail(t);
    float angle = sample_field_at_tracer(t, f);
    step_tracer_along_angle(t, angle);
    wrap_position_toroidally(t, active_cols, active_rows);
    recolour_tracer_from_angle(t, angle);
    age_tracer_and_check_death(t);
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

#define TRAIL_RAMP_STR ".,;+~*#"
#define TRAIL_RAMP_LEN 7

static char trail_head_glyph_for_angle(float angle_radians) {
  static const char head_glyph_table[ARROW_OCTANT_COUNT] = {
      '-', '/', '|', '\\', '-', '/', '|', '\\'};
  return head_glyph_table[angle_to_octant(angle_radians)];
}

/* Resolve ring-buffer slot index `i ∈ [0, filled)` to the actual
 * buffer position.  i=0 is the OLDEST slot, i=filled-1 the NEWEST
 * (the current head).  The `+ 4*len` keeps the result non-negative
 * across the modulo when trail_write_index − filled goes negative. */
static inline int trail_slot_index(const tracer *t, int i, int filled, int len) {
    return (t->trail_write_index - filled + i + 4 * len) % len;
}

/* Is this slot a position inside the visible field rect? */
static inline bool position_in_field(int col, int row,
                                     int active_cols, int active_rows) {
    return col >= 0 && col < active_cols && row >= 0 && row < active_rows;
}

/* Glyph for trail slot i out of filled.
 *   head (i == filled-1)  → directional arrow from last_angle
 *   body                  → ramp '.,;+~*#' mapped by age (oldest = '.')
 * The ramp is short enough to stay readable in any theme.            */
static inline char pick_trail_glyph(const tracer *t, int i, int filled,
                                    bool is_head) {
    if (is_head)
        return trail_head_glyph_for_angle(t->last_angle);
    int ramp_index = (i * TRAIL_RAMP_LEN) / (filled > 1 ? filled : 1);
    if (ramp_index >= TRAIL_RAMP_LEN)
        ramp_index = TRAIL_RAMP_LEN - 1;
    return TRAIL_RAMP_STR[ramp_index];
}

/* Brightness for trail slot i out of filled.
 *   head            → A_BOLD     (focal point)
 *   newer 3/4 body  → A_NORMAL
 *   oldest 1/4 body → A_DIM      (fading tail; alpha-blend analogue)
 * Note: A_DIM is forbidden by CLAUDE.md for HUD but allowed here as
 * part of the visual aesthetic — see tracer_paint section preamble. */
static inline attr_t pick_trail_brightness(int i, int filled, bool is_head) {
    if (is_head)                  return A_BOLD;
    if (i >= filled / 4)          return A_NORMAL;
    return A_DIM;
}

/* Paint ONE trail cell at the given screen position. */
static inline void paint_trail_cell(WINDOW *win, int row, int col,
                                    char glyph, int pair_id, attr_t bright) {
    attr_t a = COLOR_PAIR(pair_id) | bright;
    wattron(win, a);
    mvwaddch(win, row, col, (chtype)(unsigned char)glyph);
    wattroff(win, a);
}

/*
 * tracer_paint — fading-trail render for one tracer.
 *
 * Pseudocode:
 *   if dead or no trail yet: skip
 *   for i = 0 .. filled-1     (oldest → newest):
 *     slot = trail_slot_index(t, i, filled, len)
 *     (col, row) = trail buffer[slot]
 *     if not position_in_field(col, row, ...): skip
 *     is_head    = (i == filled - 1)
 *     glyph      = pick_trail_glyph(t, i, filled, is_head)
 *     brightness = pick_trail_brightness(i, filled, is_head)
 *     paint_trail_cell(win, row, col, glyph, t->trail_pair_id, brightness)
 */
static void tracer_paint(const tracer *t, WINDOW *win, int active_cols,
                         int active_rows) {
    if (!t->tracer_alive)               return;
    if (t->trail_filled_count == 0)     return;

    int filled = t->trail_filled_count;
    int len    = t->trail_active_length;

    for (int i = 0; i < filled; i++) {
        int slot = trail_slot_index(t, i, filled, len);
        int col  = t->trail_col[slot];
        int row  = t->trail_row[slot];

        if (!position_in_field(col, row, active_cols, active_rows))
            continue;

        bool   is_head    = (i == filled - 1);
        char   glyph      = pick_trail_glyph(t, i, filled, is_head);
        attr_t brightness = pick_trail_brightness(i, filled, is_head);

        paint_trail_cell(win, row, col, glyph, t->trail_pair_id, brightness);
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

/*
 * scene_state — the single owner of this demo's live state.
 *
 * Intent
 *   scene_state composes the field (the WHAT — simulation) with the
 *   tracer pool (the WITNESSES — also simulation, since they advect
 *   in the field) and the user's visual choices (theme, bg_mode —
 *   pure rendering).  The hot path reads from here every tick;
 *   everything else (palette tables, glyph tables, key bindings)
 *   lives at file scope as immutable constants.
 *
 * Locality (sim vs render)
 *   Fields below are GROUPED EXPLICITLY so a reader can tell at a
 *   glance which subsystem touches each one:
 *     - scene_tick / tracer_step / field generators read it
 *                                          → simulation
 *     - scene_paint / arrows / hud_draw reads it
 *                                          → rendering
 *     - both sides bound their loops by it (active_tracer_count,
 *       trail_active_length)               → shared bounds
 *     - paused gates the tick AND drives the HUD "PAUSED" tag
 *                                          → control state
 *
 *   Mis-classifying a field is a real source of bugs: a render-only
 *   value (theme, bg_mode) accidentally read by the tick would
 *   couple the simulation to a visual choice, defeating the
 *   architectural lesson (T2) that generators are independent of
 *   the renderer.
 *
 * Why these specific fields and no others
 *   - flow                  the angle grid + vortex auxiliary state;
 *                            largest member by far.
 *   - pool[]                tracer particles.  Active up to
 *                            active_tracer_count; the rest are unused
 *                            slots kept allocated for SIGWINCH.
 *   - active_tracer_count   user-adjustable via +/-; both tick and
 *                            paint iterate up to this bound.
 *   - trail_active_length   user-adjustable via s/S; affects both
 *                            the ring buffer's effective window (sim)
 *                            AND the fading-trail paint (render).
 *   - theme_index           pure render — selects which cosine
 *                            palette row maps angle → colour pair.
 *   - bg_mode               pure render — BLANK / ARROWS / COLORMAP.
 *   - paused                gate for scene_tick + HUD "PAUSED" tag.
 *
 * Things that DO NOT live here
 *   - The 4 field generators (§9..§12)        → file-scope functions
 *   - 6 cosine palette themes (§6)            → file-scope table
 *   - 8 arrow glyphs (§13)                    → file-scope table
 *   - Render-frame timing                     → locals in main()
 *   - Signal flags (SIGINT, SIGWINCH)         → file-scope volatile
 *
 * Reference [13] Raymond, NCURSES HOWTO §11 — the scene-paint
 *   newscr/curscr diff pipeline this struct feeds into.
 */
typedef struct {
    /* ── Simulation state (read by scene_tick + tracer_step) ────── *
     * The field plus the tracer particles that move through it.    */
    flow_field flow;
    tracer     pool[TRACERS_MAX];

    /* ── Shared sim+render bounds (read by both subsystems) ─────── *
     * Both the tick loop (iterating live tracers) and the paint    *
     * loop (drawing trails) bound their for-loops by these.        *
     * Adjusted by +/- (tracer count) and s/S (trail length).       */
    int active_tracer_count;
    int trail_active_length;

    /* ── Pure render state (read by scene_paint only) ───────────── *
     * Changing these MUST NOT touch simulation — they only re-pick *
     * which palette row / background painter is consulted.         */
    int theme_index;        /* 0 .. THEME_COUNT-1; indexes palette  */
    int bg_mode;            /* BG_BLANK / BG_ARROWS / BG_COLORMAP   */

    /* ── Control state ──────────────────────────────────────────── *
     * Gates the tick AND drives the HUD "PAUSED" indicator.        */
    bool paused;
} scene_state;

static void scene_init(scene_state *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->active_tracer_count = TRACERS_DEFAULT;
  s->trail_active_length = TRAIL_LEN_DEFAULT;
  s->theme_index = 0;
  s->bg_mode = BG_BLANK;
  s->paused = false;

  field_init(&s->flow, cols, rows, FIELD_KIND_CURL);
  field_evolve_and_rebuild(&s->flow);

  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_resize(scene_state *s, int cols, int rows) {
  field_init(&s->flow, cols, rows, s->flow.active_kind);
  field_evolve_and_rebuild(&s->flow);
  for (int i = 0; i < TRACERS_MAX; i++)
    if (s->pool[i].tracer_alive)
      tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
}

static void scene_reset(scene_state *s, int cols, int rows) {
  s->flow.time_axis = 0.0f;
  s->flow.vortex_ring_phase = 0.0f;
  field_update_vortex_positions(&s->flow);
  field_evolve_and_rebuild(&s->flow);
  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
  for (int i = s->active_tracer_count; i < TRACERS_MAX; i++)
    s->pool[i].tracer_alive = false;
}

static void scene_set_trail_length(scene_state *s, int new_length) {
  if (new_length < TRAIL_LEN_MIN)
    new_length = TRAIL_LEN_MIN;
  if (new_length > TRAIL_LEN_MAX)
    new_length = TRAIL_LEN_MAX;
  s->trail_active_length = new_length;
  for (int i = 0; i < TRACERS_MAX; i++)
    s->pool[i].trail_active_length = new_length;
}

static void scene_tick(scene_state *s, int cols, int rows) {
  if (s->paused)
    return;

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
static void scene_paint_background(const scene_state *s, WINDOW *win, int cols,
                                   int rows) {
  if (s->bg_mode == BG_BLANK)
    return;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float angle = flow_angle_at_cell(&s->flow, c, r);
      int pair = angle_to_palette_pair(angle);
      char glyph = arrow_glyph_for_angle(angle);
      wattron(win, COLOR_PAIR(pair));
      mvwaddch(win, r, c, (chtype)(unsigned char)glyph);
      wattroff(win, COLOR_PAIR(pair));
    }
  }
}

static void scene_paint(const scene_state *s, WINDOW *win, int cols, int rows) {
  scene_paint_background(s, win, cols, rows);
  for (int i = 0; i < s->active_tracer_count; i++)
    tracer_paint(&s->pool[i], win, cols, rows);
}

/* ===================================================================== */
/* §18  hud — top status + bottom hint strip (CLAUDE.md spec)            */
/* ===================================================================== */

static void hud_paint_status(WINDOW *win, int cols, double fps_display,
                             int sim_hz, const scene_state *s) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " %5.1f fps  sim:%2dHz  tracers:%3d  trail:%2d  "
           "field:%-15s  bg:%-9s  theme:%-7s  evol:%.4f  %s ",
           fps_display, sim_hz, s->active_tracer_count, s->trail_active_length,
           field_kind_name_table[s->flow.active_kind],
           bg_mode_name_table[s->bg_mode],
           palette_theme_table[s->theme_index].name,
           (double)s->flow.evolution_speed, s->paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  int x = cols - len;
  if (x < 0)
    x = 0;
  wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvwprintw(win, 0, x, "%s", buf);
  wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hints(WINDOW *win, int rows) {
  const char *hint = " q:quit  spc:pause  r:reset  a:field  t:theme  v:bg  "
                     "+/-:tracers  s/S:trail  ]/[:simHz  f/F:evol ";
  wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvwprintw(win, rows - 1, 0, "%s", hint);
  wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §19  screen — ncurses init / cleanup                                  */
/* ===================================================================== */

static void screen_init(int theme_index) {
  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init(theme_index);
}

static void screen_cleanup(void) { endwin(); }

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

static bool app_handle_key(int key, scene_state *s, int *sim_hz, int cols,
                           int rows) {
  switch (key) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 'r':
  case 'R':
    scene_reset(s, cols, rows);
    break;

  case 'a':
  case 'A':
    s->flow.active_kind = (s->flow.active_kind + 1) % FIELD_KIND_COUNT;
    scene_reset(s, cols, rows);
    break;

  case 't':
  case 'T':
    s->theme_index = (s->theme_index + 1) % THEME_COUNT;
    colors_apply_theme(s->theme_index);
    break;

  case 'v':
  case 'V':
    s->bg_mode = (s->bg_mode + 1) % BG_MODE_COUNT;
    break;

  case '+':
  case '=':
    if (s->active_tracer_count + TRACERS_STEP <= TRACERS_MAX) {
      int old_count = s->active_tracer_count;
      s->active_tracer_count += TRACERS_STEP;
      for (int i = old_count; i < s->active_tracer_count; i++)
        tracer_respawn(&s->pool[i], cols, rows, s->trail_active_length);
    }
    break;
  case '-':
  case '_':
    if (s->active_tracer_count - TRACERS_STEP >= TRACERS_MIN) {
      s->active_tracer_count -= TRACERS_STEP;
      for (int i = s->active_tracer_count;
           i < s->active_tracer_count + TRACERS_STEP; i++)
        s->pool[i].tracer_alive = false;
    }
    break;

  case ']':
    if (*sim_hz + SIM_HZ_STEP <= SIM_HZ_MAX)
      *sim_hz += SIM_HZ_STEP;
    break;
  case '[':
    if (*sim_hz - SIM_HZ_STEP >= SIM_HZ_MIN)
      *sim_hz -= SIM_HZ_STEP;
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

int main(void) {
  srand((unsigned int)clock_now_ns());
  perlin_perm_init();

  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  static scene_state scene;
  int sim_hz = SIM_HZ_DEFAULT;

  screen_init(0);
  atexit(screen_cleanup);

  int cols, rows;
  getmaxyx(stdscr, rows, cols);
  scene_init(&scene, cols, rows);

  /* Fixed-step accumulator (Glenn Fiedler "Fix Your Timestep!"). */
  int64_t prev_ns = clock_now_ns();
  int64_t sim_accum_ns = 0;

  /* Sliding-window FPS counter. */
  int frames_in_window = 0;
  int64_t window_accum_ns = 0;
  double fps_display = 0.0;

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
    prev_ns = frame_start;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    int64_t tick_ns = NS_PER_SEC / sim_hz;
    sim_accum_ns += dt_ns;
    while (sim_accum_ns >= tick_ns) {
      scene_tick(&scene, cols, rows);
      sim_accum_ns -= tick_ns;
    }

    erase();
    scene_paint(&scene, stdscr, cols, rows);
    hud_paint_status(stdscr, cols, fps_display, sim_hz, &scene);
    hud_paint_hints(stdscr, rows);
    wnoutrefresh(stdscr);
    doupdate();

    frames_in_window++;
    window_accum_ns += dt_ns;
    if (window_accum_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      fps_display = (double)frames_in_window /
                    ((double)window_accum_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      window_accum_ns = 0;
    }

    int key;
    while ((key = getch()) != ERR) {
      if (!app_handle_key(key, &scene, &sim_hz, cols, rows)) {
        g_should_quit = 1;
        break;
      }
    }

    int64_t spent = clock_now_ns() - frame_start;
    if (spent < frame_cap_ns)
      clock_sleep_ns(frame_cap_ns - spent);
  }

  return 0;
}
