/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * epicycles.c — draw any 2-D closed curve as a chain of rotating arms.
 *
 * DEMO
 *   The screen shows one rotating arm pinned at the centre.  At its
 *   tip is pinned another, smaller arm that spins faster.  At THAT
 *   tip a third, still smaller arm spins faster again.  And so on.
 *   The very last tip leaves a glowing trail that — given enough
 *   arms — traces a recognisable shape: heart, star, butterfly,
 *   trefoil knot, whatever you pick from the catalogue.
 *
 *   The "magic" is that every closed 2-D curve in the world can be
 *   built this way — by choosing the right radius, angular speed,
 *   and starting angle for each arm.  The recipe machine that turns
 *   a chosen shape into a list of (radius, speed, phase) triples is
 *   the DISCRETE FOURIER TRANSFORM.  This program runs the recipe
 *   live for 20 sample shapes and lets you watch the reconstruction
 *   improve as more arms are added to the chain.
 *
 * Study alongside
 *   signal/fft_visualizer.c   the same maths in 1-D, used to draw
 *                              an audio spectrum.
 *   procedural/lissajous*.c   two-circle epicycles where the speeds
 *                              are not necessarily integer.
 *   raymarcher/spinning_*.c   2-D rotation generalised to 3-D
 *                              spatial rotation matrices.
 *
 * Section map
 *   §1   config            tunable constants (every magic number lives here)
 *   §2   clock             monotonic timer + nanosecond sleep
 *   §3   complex           a tiny complex-number arithmetic library
 *   §4   themes            six colour palettes selectable at runtime
 *   §5   colors            ncurses colour-pair init + apply_theme
 *   §6   shapes            20 parametric curves + vertex-interp helpers
 *   §7   sample            evaluate a curve at N equally-spaced t values
 *   §8   dft               the recipe machine — sum of N×N projections
 *   §9   epicycle          the per-arm record + amplitude-descending sort
 *   §10  build             sample → DFT → decode → sort, all in one call
 *   §11  trail             ring buffer remembering the tip's recent path
 *   §12  chain             walk the arms each frame, store joint positions
 *   §13  scene             frame-level animation state + tick orchestrator
 *   §14  bresenham         integer-only line rasteriser (for arm segments)
 *   §15  ellipse           pixel-circle → cell-aspect-corrected ellipse
 *   §16  render_orbits     back layer: faint orbit guides
 *   §17  render_trail      mid  layer: fading tip path
 *   §18  render_chain      arm-segment layer
 *   §19  render_markers    pivot dot + bright tip bob
 *   §20  render_debug      'd' amplitude bar chart (sorted), 'D' arm-info table
 *   §21  hud               top status line + bottom key hint
 *   §22  screen            ncurses init / cleanup / resize / present
 *   §23  app               main loop, signal handlers, key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   n / p            next / previous shape  (20 to choose from)
 *   t / T            next / previous theme  (6 palettes)
 *   + / -            grow / shrink chain by one arm
 *   a                toggle auto-grow (one new arm every ~0.4 s)
 *   c                toggle orbit-circle overlay
 *   d                toggle amplitude bar chart, sorted by rank (debug)
 *   D                toggle per-arm numeric panel (debug)
 *   r                reset (back to 1 arm, restart angle, clear trail)
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/epicycles.c -o epicycles \
 *       -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, total newcomer to Fourier:
 *     1. CONCEPTS   — what we're doing and why anyone would care.
 *     2. MENTAL MODEL — pictures + invariants you need in your head.
 *     3. GUIDED TUTORIAL T1..T12 — derive the algorithm in twelve
 *        small steps; each step ends with a piece of pseudocode
 *        that maps 1:1 to a function in §6..§23.
 *     4. §1 config   — the knobs.  Now you know what they control.
 *     5. §6 shapes   — concrete examples of "input curves".
 *     6. §7..§10     — sampling, DFT, sort.  Read after T5..T9.
 *     7. §11..§13    — animation state + per-frame tick.  After T10.
 *     8. §14..§20    — rendering layers + debug overlays.  After T12.
 *     9. §21..§23    — HUD, ncurses, main loop + key dispatch.
 *
 *   Reader who already knows DFT and just wants the code:
 *     §1 config, §6 shapes, §8 dft, §10 build, §13 scene, §23 app.
 *     The rest is teaching scaffolding.
 *
 * Naming convention
 *   Long, descriptive identifiers throughout — no `vr`, `T_view`, `dt`.
 *     total_epicycle_count        N in the literature (= DFT_SAMPLE_COUNT)
 *     animation_phase_radians     phi (the global animation angle)
 *     amplitude_normalised        |Z[n]| / N  (in unit-disc terms)
 *     phase_offset_radians        arg(Z[n])
 *     frequency_signed            integer harmonic (T8 explains the sign)
 *     active_epicycle_count       arms currently animated (1..N)
 *     joint_pixel_x[i]            x of the i-th joint in pixel space
 *
 *   Short names are kept only for tight loop counters (`i`, `k`, `n`)
 *   where the surrounding line provides full context.
 *
 * Background you NEED
 *   Basic complex numbers (a + bi, |z|, arg(z)).  T2 reviews them.
 *   Trigonometry (cos and sin as projections of a unit-circle point).
 *
 * Background you do NOT need
 *   The Fast Fourier Transform.  We use the textbook O(N²) DFT;
 *   N = 256 is small enough that even a slow laptop does it in
 *   under 5 ms, and we run it ONCE per shape change, not per frame.
 *
 *   Continuous integrals.  We never integrate; everything is a sum.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm
 *   Pick a closed 2-D curve and sample it at N equally-spaced
 *   parameter values to get N complex numbers z[0..N-1].  Compute
 *   the discrete Fourier transform Z[n] = Σ_k z[k] · exp(-i·2π·n·k/N).
 *   Each output bin Z[n] tells us the radius, starting angle, and
 *   integer rotation rate of one rotating arm; the input curve is
 *   the sum of all those arms attached end-to-end.  Sort the arms
 *   from largest to smallest radius and animate the first M of
 *   them; M can be raised until the visual reconstruction is good
 *   enough.
 *
 * Data structure
 *   A flat array of `Epicycle` records.  Each record stores three
 *   numbers (amplitude, phase, signed integer frequency) — that is
 *   all an arm needs to be animated for any phase φ ∈ [0, 2π).
 *   The chain is just walking this array left-to-right and adding
 *   the i-th arm's tip displacement to the previous joint position.
 *   No trees, no graphs, no spatial hashes.
 *
 * Rendering
 *   Pixel-space simulation, cell-space draw.  The chain math runs
 *   in continuous (sub-cell) coordinates so that arms don't snap
 *   to the grid as they rotate; only at the final draw step do we
 *   round each joint to a (col, row) and rasterise the connecting
 *   line with Bresenham.  Orbit guides are drawn as point-sampled
 *   ellipses because terminal cells are roughly twice as tall as
 *   wide — a true pixel-space circle becomes a cell-space ellipse.
 *
 * Performance
 *   The DFT is O(N²) but it runs once per shape change.  Per-frame
 *   work is O(M) for the chain walk, O(L) for the trail (L =
 *   trail-buffer length), and O(W·H) worst case for the screen
 *   blit.  At N = 256, M ≤ 256, L = 600, the program holds 30 fps
 *   on any terminal that can keep up with 30 redraws per second.
 *   The one optimisation worth knowing — the twiddle-factor
 *   recurrence — is documented in T6.
 *
 * References
 *   3Blue1Brown, "But what is a Fourier series? From heat flow to
 *     drawing with circles" — the YouTube essay that popularised
 *     the epicycle visualisation.  Watch this first.
 *   Cormen, Leiserson, Rivest, Stein, "Introduction to Algorithms"
 *     3rd ed., Chapter 30 — DFT and FFT in textbook prose.
 *   Press et al., "Numerical Recipes in C" 2nd ed., Chapter 12 —
 *     practical DFT/FFT with code.
 *   Smith, "The Scientist and Engineer's Guide to DSP", Chapter 8
 *     — DFT explained with no calculus prerequisite.
 *   https://en.wikipedia.org/wiki/Discrete_Fourier_transform
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   A closed 2-D curve and a set of rotating arms attached
 *   end-to-end are TWO REPRESENTATIONS OF THE SAME OBJECT.  Going
 *   from curve to arms is a transform (the DFT).  Going from arms
 *   back to curve is animation (sum of rotations evaluated for
 *   each phase).  Both directions are exact; neither loses
 *   information.
 *
 * HOW TO THINK ABOUT IT
 *   Imagine a wooden pendulum with a pen at its tip.  The pendulum
 *   is hinged at the ceiling and swings in a circle, drawing a
 *   circle on the floor.  Now hang another, smaller pendulum from
 *   the tip of the first one — it can swing on its own circle
 *   while the big arm carries it around.  The pen, now at the tip
 *   of the second pendulum, no longer draws a clean circle but an
 *   intricate looping shape.  Add a third pendulum to the second
 *   tip; the pattern grows even more intricate.  Continue until
 *   you have a hundred pendulums in a chain — the pen at the very
 *   tip can be made to trace ANY closed 2-D shape if you choose
 *   each pendulum's length and rotation rate correctly.
 *
 *   The DFT is the calculation that figures out the lengths and
 *   rates.  It looks at your target shape, asks "how much energy
 *   is at each integer rotation rate?", and returns a table.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   pivot                                                 │
 *      │     +─── arm 0 (biggest, slowest)                       │
 *      │            ╲                                            │
 *      │             ●─── arm 1 (smaller, faster)                │
 *      │                   ╲                                     │
 *      │                    ●─── arm 2 (smaller still, faster)   │
 *      │                          ╲                              │
 *      │                           ● ←── pen tip leaves a trail  │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 *   ONCE per shape change:
 *     1. Sample the chosen parametric curve at N evenly-spaced
 *        parameter values t_k = 2πk/N, giving N complex numbers
 *        z[0..N-1] where z[k] = x(t_k) + i·y(t_k).
 *     2. Compute the DFT: Z[n] = Σ_k z[k]·exp(-i·2π·n·k/N) for
 *        every output bin n ∈ [0, N).
 *     3. For each Z[n], extract amplitude_n = |Z[n]|/N,
 *        phase_n = arg(Z[n]), and signed integer frequency
 *        freq_n = (n ≤ N/2) ? n : n - N.  These three numbers
 *        are everything one arm needs.
 *     4. Sort the N records by amplitude_n descending, so that
 *        epicycle_table[0] is the dominant arm and the table
 *        tail is just decorative high-frequency wiggle.
 *
 *   EVERY frame:
 *     5. Advance the global animation phase φ by 2π/360 radians.
 *     6. Walk the first M = active_epicycle_count records, each
 *        time adding amplitude_i·(cos α_i, sin α_i) to the
 *        running joint position, where α_i = freq_i·φ + phase_i.
 *     7. Push the final tip position into the trail ring buffer.
 *     8. Erase the screen, then paint orbits, trail, arm lines,
 *        and markers in back-to-front order, then HUD on top.
 *
 * KEY FORMULAS
 *
 *   Euler — the bridge between rotation and arithmetic
 *     exp(i·θ) = cos θ + i·sin θ
 *
 *   Forward DFT (we use this)
 *     Z[n] = Σ_{k=0..N-1} z[k] · exp(-i·2π·n·k/N)
 *
 *   Inverse DFT (we don't run it, but it justifies the chain walk)
 *     z[k] = (1/N) · Σ_{n=0..N-1} Z[n] · exp(+i·2π·n·k/N)
 *
 *   Per-arm decoding
 *     amplitude_n = |Z[n]| / N
 *     phase_n     = arg(Z[n])
 *     freq_n      = (n ≤ N/2) ? n : n - N      (T8 derives the sign)
 *
 *   Tip position at animation phase φ
 *     z(φ) = Σ_{i=0..M-1} amplitude_i · exp(i·(freq_i·φ + phase_i))
 *
 *   Twiddle recurrence (one cos+sin per output bin, T6)
 *     W = exp(-i·2π·n/N)
 *     twiddle = 1; sum = 0
 *     for k in 0..N-1:
 *       sum += z[k] · twiddle
 *       twiddle *= W
 *     Z[n] = sum
 *
 *   Animation phase step (1 degree per frame at 360-frame cycle)
 *     φ ← φ + 2π / ANIMATION_FRAMES_PER_CYCLE
 *
 *   Pixel-circle to cell-ellipse aspect compensation
 *     semi_axis_x_cells = radius_pixels / CELL_PIXEL_WIDTH
 *     semi_axis_y_cells = radius_pixels / CELL_PIXEL_HEIGHT
 *
 * EDGE CASES TO WATCH
 *   - The DC bin Z[0] is the curve's CENTROID times N.  Decoded as
 *     an arm with frequency 0, it doesn't rotate; it just shifts
 *     the chain origin from the screen centre to wherever the
 *     curve's mean point sits.  If a shape is centred (Heart,
 *     Star) the DC arm is tiny; if it's off-centre (Cardioid)
 *     the DC arm is large and stationary.
 *
 *   - Upper-half DFT bins (n > N/2) encode NEGATIVE frequencies
 *     by the periodicity of complex exponentials.  Without the
 *     n ↦ n-N remap in §10, all arms would spin clockwise and
 *     the reconstruction would come out as the mirror image of
 *     the input curve.  T8 has the derivation.
 *
 *   - Sharp corners (Star, Triangle, Square) produce the GIBBS
 *     phenomenon: even with hundreds of arms, the reconstruction
 *     overshoots the corner by a fraction that does not vanish.
 *     This is intrinsic to Fourier reconstruction of discontinuous
 *     derivatives, not a bug in our DFT.  T11 has the picture.
 *
 *   - The trail ring buffer must be cleared at every full cycle
 *     (φ wraps to 0); otherwise multiple traces stack and the
 *     screen accumulates leftover dots.  Done in §13 animation_tick.
 *
 *   - Resize triggers a full scene_init, which resets the chain to
 *     1 arm.  Acceptable on a UI program, but a surprise — flag
 *     this in any error report ("did the terminal resize?").
 *
 * HOW TO VERIFY
 *   - Press 'r' with the Heart shape loaded; with M = 1 you should
 *     see one slow orbit + a stationary off-centre arm (DC).  Hit
 *     '+' to grow M; by ~10 arms the trail traces a heart cleanly.
 *
 *   - Switch to Star (5pt) and reduce M to ~30; the trail's points
 *     should look round-shouldered, not sharp.  Around M = 80 the
 *     points become recognisably pointy.  At M = 200 they're crisp.
 *     This visualises Gibbs convergence.
 *
 *   - Switch to Circle.  M = 1 should already be perfect (all
 *     other Z[n] are zero).  M = 2 unchanged.
 *
 *   - Switch to Ellipse.  M = 2 should already be perfect (only
 *     the +1 and -1 bins are non-zero).  M = 1 traces a circle
 *     because we drop the negative-frequency conjugate partner.
 *
 *   - Cycle themes with 't'.  Arm/trail/orbit colours change; HUD
 *     colour does not (theme-invariant by HUD Standard).
 *
 *   - Press 'd' — a bar chart appears at bottom-left showing the
 *     amplitudes of the first 30 arms.  Sharp shapes have a long,
 *     slow tail; smooth shapes have a tall first bar then nothing.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   Wheels on wheels — the visual idea
 *   T2   Complex numbers as 2-D vectors with a multiplication
 *   T3   Forward problem: chain → curve
 *   T4   Inverse problem: curve → chain (we need a recipe machine)
 *   T5   The DFT formula, term by term
 *   T6   Twiddle recurrence — saving 99.6% of the trig calls
 *   T7   Reading DFT output — amplitude, phase, frequency
 *   T8   Negative frequencies and the upper-half remap
 *   T9   Greedy reconstruction by sorting amplitudes
 *   T10  Per-frame animation — what the main loop actually does
 *   T11  Convergence — why sharp corners need many more arms (Gibbs)
 *   T12  Drawing in the terminal — Bresenham + cell-aspect ellipse
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHEELS ON WHEELS — THE VISUAL IDEA
 * ──────────────────────────────────────
 * Take a single circle of radius r centred on a fixed point P.  Put
 * a marker on its rim; let the marker rotate at angular speed ω.
 * Over time the marker traces — a circle.  Boring.
 *
 * Now CARRY a second circle on the rim of the first.  The second
 * circle has its own radius r₂ and rotates at its own speed ω₂
 * around its own moving centre.  A marker on its rim no longer
 * traces a circle: it traces a curlicue, called an EPITROCHOID
 * in the literature.  Spirograph toys are built on exactly this
 * mechanism with two circles.
 *
 * Add a third circle on the rim of the second.  Add a fourth.
 * Add a hundred.  Each new wheel adds a finer-grained wobble to
 * the marker's path.  After enough wheels the path can be ANY
 * closed 2-D shape — a heart, a butterfly, a portrait of Homer
 * Simpson, anything.
 *
 * The choreography is:
 *   - Each wheel has a fixed RADIUS (size of its arm).
 *   - Each wheel has a fixed ANGULAR SPEED (an integer number of
 *     turns per "outer cycle").
 *   - Each wheel has a fixed STARTING ANGLE.
 *
 * Three numbers per wheel.  No animation needed to specify it —
 * just a static table.  Run the chain forward in time and a curve
 * comes out.
 *
 * The natural question now: GIVEN a target curve (say, a heart),
 * what radii / speeds / starting angles do I pick to make the
 * marker trace it?  That's the inverse problem.  It has a
 * beautiful answer (the DFT), but first we need a tidy way to
 * write the maths.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   one wheel               three wheels                  │
 *      │                                                         │
 *      │      ___                  ___    ___                    │
 *      │     /   \                /   \  /   \                   │
 *      │    | ●  |   →           |   ● ●● ●  |   → curlicue path │
 *      │     \___/                \___/  \___/                   │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * T2  COMPLEX NUMBERS AS 2-D VECTORS WITH A MULTIPLICATION
 * ─────────────────────────────────────────────────────────
 * A 2-D point (x, y) and a complex number z = x + i·y are the same
 * object viewed two ways.  The "+ i" part feels mysterious if you
 * think of i as "the square root of -1", but the geometric story
 * is concrete:
 *
 *   - ADDITION of complex numbers is vector addition (head-to-tail).
 *     This is the same as saying "add x's, add y's".
 *
 *   - MULTIPLICATION of complex numbers is "rotate-and-scale" in
 *     the plane.  Specifically, multiplying by a complex number
 *     of magnitude r and angle θ rotates the other operand by θ
 *     and scales it by r.
 *
 * Example: multiplying any z by `i` rotates z by 90° counter-clockwise
 * (because |i| = 1, arg(i) = 90°).  Try z = 2 + 0i; multiply by i;
 * get 0 + 2i = (0, 2), which is (2, 0) rotated 90° CCW.  ✓
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │              imaginary axis                             │
 *      │                  ▲                                      │
 *      │                  │                                      │
 *      │              z·i = -1+2i  (rotated 90°, |z|=√5)         │
 *      │                  │                                      │
 *      │      ────────────┼──────────────► real axis             │
 *      │                  │       z = 2+i                        │
 *      │                  │      (|z| = √5, angle ≈ 26.6°)       │
 *      │                  │                                      │
 *      └─────────────────────────────────────────────────────────┘
 *
 * The bridge between angles and arithmetic is EULER'S FORMULA:
 *
 *     exp(i·θ) = cos θ + i·sin θ                  (unit circle)
 *
 * In words: walk θ radians around the unit circle CCW from the
 * positive real axis.  exp(i·θ) is just the (cos, sin) point you
 * end up at, but written as a complex number so that you can do
 * arithmetic with it.
 *
 * Why we like this for epicycles: an arm of length r at starting
 * angle φ that has rotated through an additional angle ψ since
 * t=0 has its tip at displacement r·exp(i·(φ+ψ)) from its base.
 * Add a chain of these and the tip is just a sum.  Rotation
 * becomes multiplication; chaining becomes addition.  No trig
 * identities needed at the high level.
 *
 * In code (§3) we don't use <complex.h>; a 2-float struct named
 * `ComplexNumber` plus five helpers (add, mul, exp_unit, magnitude,
 * argument) is enough and is easy to print in a debugger.
 *
 * T3  FORWARD PROBLEM: CHAIN → CURVE
 * ──────────────────────────────────
 * Suppose someone hands us a table of M arms:
 *
 *     i:   amplitude_i    frequency_i    phase_i
 *     0:        0.50            1            0.00
 *     1:        0.30            2            π/4
 *     2:        0.20           -1            π/2
 *
 * The "tip position at time φ" is the sum of the individual arm
 * tips, each treated as a complex displacement from the previous
 * joint:
 *
 *     z(φ) = Σ_{i=0..M-1}  amplitude_i · exp(i·(frequency_i·φ + phase_i))
 *
 * Read the i-th term: the arm's length is amplitude_i; its tip
 * sits at angle frequency_i·φ + phase_i from the +x axis (relative
 * to its own base), so its displacement is the polar-to-Cartesian
 * formula written as a complex number.  Sum all terms — that's
 * the tip position relative to the pivot.
 *
 * The variable φ ("phi") is the GLOBAL animation clock.  At φ = 0
 * the chain is "fresh" — arm i is at its starting angle phase_i.
 * One frame later φ has advanced by a small step.  Arm i has
 * rotated through frequency_i times that step (because it spins
 * at integer multiples of the global rate).  The TIP traces a
 * closed loop because all arms return to their starting angles
 * exactly when φ has advanced by 2π — the integer frequencies
 * guarantee that.
 *
 * In code this becomes a simple loop (§12 chain_compute):
 *
 *     joint[0] = pivot
 *     for i in 0..M-1:
 *       angle = freq_i · φ + phase_i
 *       dx    = amp_i · cos(angle)
 *       dy    = amp_i · sin(angle)
 *       joint[i+1] = joint[i] + (dx, dy)
 *
 * The forward problem is THIS easy.  The hard part is finding
 * (amplitude, frequency, phase) for an arbitrary input curve.
 *
 * T4  INVERSE PROBLEM: CURVE → CHAIN  (we need a recipe machine)
 * ──────────────────────────────────────────────────────────────
 * Goal: given N complex sample points z[0..N-1] (the target curve
 * sampled evenly along its parameter), produce the table of arms
 * (amplitude, frequency, phase) such that the chain's tip at
 * phase φ_k = 2πk/N exactly equals z[k].
 *
 * Restrict to INTEGER frequencies — without this the problem is
 * underdetermined.  With integer frequencies and N arms we have
 * exactly N degrees of freedom in the table (well, 3N parameters
 * but the magnitude/phase of each pin them down), and N
 * constraints (the N sample points must be hit).  The system is
 * square; a unique solution exists, and the DISCRETE FOURIER
 * TRANSFORM is the closed-form way to compute it.
 *
 * Intuition for why integer frequencies suffice: if any arm spun
 * at a non-integer rate (say 1.3 turns per outer cycle), it would
 * never return to its starting angle when φ = 2π, so the curve
 * wouldn't close.  Since our target curve IS closed, only integer
 * harmonics contribute — and the DFT happens to give us exactly
 * the coefficients of those harmonics.
 *
 * The DFT outputs N complex numbers Z[0..N-1].  Each Z[n] encodes
 * one arm as |Z[n]|/N (amplitude), arg(Z[n]) (starting angle),
 * and the integer n itself as the frequency.  T7 covers the
 * decoding; T5 derives the formula.
 *
 * T5  THE DFT FORMULA, TERM BY TERM
 * ─────────────────────────────────
 *
 *     Z[n] = Σ_{k=0..N-1}  z[k] · exp(-i·2π·n·k/N)
 *
 * Read this as a measurement.  We are asking: "How much of the
 * pure n-cycles-per-period rotation exp(i·2π·n·t/N) is present
 * in the input z[k]?"  The standard way to extract a component
 * from a vector is to project it onto the basis direction.  In
 * function space, projection is "multiply by the basis function's
 * conjugate and integrate" — for discrete data, sum.  That's all
 * the formula is doing, summed over N samples.
 *
 *   - z[k]                      the input curve point at sample k.
 *   - exp(-i·2π·n·k/N)          the conjugate of the n-th basis
 *                                rotation.  As k advances 0 → N-1,
 *                                this winds exactly n full turns
 *                                around the unit circle CW (the
 *                                minus sign).  Acts as a "lock-in
 *                                detector" tuned to frequency n.
 *   - z[k] · exp(...)           each sample, twisted by however
 *                                much it'd take to undo the n-th
 *                                rotation.  If z does contain that
 *                                rotation, the twisted samples line
 *                                up; otherwise they cancel.
 *   - Σ_k                       sum the twisted samples.  Aligned
 *                                ones add (constructive interference);
 *                                random-phase ones cancel.
 *
 * Result: Z[n] is large only if z contains energy at the n-th
 * harmonic.  The MAGNITUDE of Z[n] tells you how much; the ARGUMENT
 * tells you the starting phase.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   Lock-in detector picture:                             │
 *      │                                                         │
 *      │     z[k]    *=    exp(-i·2π·n·k/N)    →   accumulator   │
 *      │   (input)        (n-th basis CCW⁻¹)                     │
 *      │                                                         │
 *      │   If z has component at freq n: terms align → big sum   │
 *      │   If z is "orthogonal" to freq n: terms cancel → 0      │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * The inverse DFT is similar, with the sign of the exponent
 * flipped and a 1/N normaliser.  We don't run the inverse — but
 * its existence is what guarantees the chain (§12) reconstructs
 * z[k] exactly when M = N.
 *
 * T6  TWIDDLE RECURRENCE — SAVING 99.6% OF THE TRIG CALLS
 * ───────────────────────────────────────────────────────
 * The naive DFT calls cos AND sin once for every (n, k) pair —
 * that's 2N² trig calls per transform.  At N = 256 that's
 * 131,072 calls.  Each cos/sin on x86 takes roughly 30-50 ns;
 * total ≈ 5-7 ms.  Not crippling, but not free.
 *
 * THE INSIGHT: in the inner loop, we don't need cos/sin of
 * arbitrary angles — we need successive powers of one fixed
 * complex number W = exp(-i·2π·n/N).  And complex multiplication
 * IS "advance by one power":
 *
 *     W^0 = 1
 *     W^1 = W
 *     W^2 = W · W
 *     W^k = W · W^(k-1)            ← maintain incrementally
 *
 * So compute W ONCE per output bin (one cos + one sin), then walk
 * the inner loop multiplying a running `twiddle_power` by W each
 * iteration.  Total trig calls per DFT drops from 2N² to 2N — at
 * N = 256, from 131,072 to 512.  ~256× fewer.
 *
 * The cost is one extra complex multiply per inner-loop iteration
 * (the `twiddle *= W` step), but that's much cheaper than a
 * hardware trig call.  Documented in §8 dft_compute_with_twiddle.
 *
 * Numerical caveat: floating-point error accumulates in
 * `twiddle_power` over many multiplications; |twiddle| drifts
 * away from 1.  At N = 256 the drift is well below single-
 * precision noise.  At N > ~10,000 you'd want to refresh the
 * twiddle every few hundred steps.  Since we're at N = 256 we
 * ignore it.
 *
 * If we cared about even bigger N, the FAST FOURIER TRANSFORM
 * computes the same Z[] in O(N log N) by exploiting the recursive
 * structure of the DFT matrix.  At N = 256 the FFT would be ~10×
 * faster than our recurrence-DFT, but our DFT runs once per
 * shape change — not in the per-frame hot path — so the win is
 * irrelevant here.
 *
 * T7  READING DFT OUTPUT — AMPLITUDE, PHASE, FREQUENCY
 * ────────────────────────────────────────────────────
 * Each bin Z[n] is a complex number; decompose it into three
 * arm-table fields:
 *
 *     amplitude_n  = |Z[n]| / N       arm radius (in shape units)
 *     phase_n      = arg(Z[n])         starting angle of the arm
 *     frequency_n  = signed-int(n)     integer rotations per cycle
 *
 * The 1/N divisor in amplitude_n comes from the inverse DFT's
 * normalisation factor.  Without it, the reconstructed tip would
 * be N times too far from the pivot.
 *
 * Sanity-check at φ = 0: the chain's tip should be z[0] (the first
 * input sample).  Plug φ = 0 into the chain formula:
 *
 *     z(0) = Σ_n amp_n · exp(i·phase_n)
 *          = (1/N) · Σ_n Z[n]                    (definition of amp,phase)
 *          = (1/N) · Σ_n Σ_k z[k]·exp(-i·2π·n·k/N)
 *          = z[0]                                 (orthogonality)  ✓
 *
 * Common bin highlights:
 *   - Z[0] is the SUM of all samples.  After /N it's the curve's
 *     CENTROID.  Decoded as a frequency-0 arm, it's stationary
 *     and just shifts the chain origin to the centroid.
 *   - Z[1] is the projection onto the +1 harmonic.  Usually large
 *     for any one-loop curve.  Our shape_sample_circle (a pure CCW
 *     unit-speed rotation) has Z[1] = N exactly and every other
 *     bin equal to zero — only one arm is needed.
 *   - Z[N/2] is the NYQUIST bin — fastest representable rotation.
 *   - Z[N-1] occupies signed frequency -1 (the "other-handed"
 *     once-around rotation).  It is the conjugate of Z[1] for any
 *     real-axis-symmetric curve (e.g. our Ellipse: Z[1] = 0.8 N,
 *     Z[N-1] = 0.2 N, all others zero).  For asymmetric curves
 *     the two halves carry independent magnitudes and phases.
 *
 * For SMOOTH shapes (Heart, Trefoil) most energy sits in low
 * harmonics.  For SHARP shapes (Star, Triangle) energy decays only
 * polynomially with n — that's why they need many more arms (T11).
 *
 * T8  NEGATIVE FREQUENCIES AND THE UPPER-HALF REMAP
 * ─────────────────────────────────────────────────
 * The DFT outputs N bins indexed 0..N-1.  But mathematically, the
 * complex Fourier series of a 2-D curve uses BOTH POSITIVE AND
 * NEGATIVE frequencies — arms can spin clockwise as well as
 * counter-clockwise.  Where do the negative frequencies hide?
 *
 * In the upper half of the DFT output.  The reason is the
 * periodicity of complex exponentials: multiplying the basis
 * angle by N is the same as multiplying by 0, because
 * exp(-i·2π·k) = 1 for any integer k.  Concretely:
 *
 *     exp(-i·2π·(N-1)·k/N) = exp(-i·2π·k) · exp(+i·2π·k/N)
 *                          = 1 · exp(+i·2π·k/N)
 *
 * So the (N-1) bin's basis function is identical to the (-1) bin's
 * basis function.  In the algorithm we REMAP:
 *
 *     for n = 0       to N/2:    signed_freq = n
 *     for n = N/2 + 1 to N - 1:  signed_freq = n - N
 *
 * After the remap, frequencies live in {-N/2+1, ..., -1, 0, 1,
 * ..., N/2}.  Negative-frequency arms spin CLOCKWISE; positive-
 * frequency arms spin CCW.
 *
 * If we forgot the remap and treated all frequencies as positive,
 * every arm would spin the same direction, and the reconstructed
 * curve would come out as the COMPLEX CONJUGATE of the original
 * (mirror-image across the real axis).  Easy to spot if you ever
 * see it — the trail traces a left-right-flipped version of the
 * shape.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │  Raw DFT bin index   n: 0  1  2 ... N/2 ... N-2 N-1     │
 *      │                                                         │
 *      │  Signed frequency      0  1  2 ... N/2 ...  -2  -1      │
 *      │                       ─positive─►       ◄─negative─     │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * T9  GREEDY RECONSTRUCTION BY SORTING AMPLITUDES
 * ───────────────────────────────────────────────
 * After §10 builds N arm records, we SORT THEM BY AMPLITUDE
 * DESCENDING.  Why?  Because at any cutoff M < N, we want the M
 * arms that contribute the most to the visual.  The L² norm
 * (squared error between target and reconstruction) is minimised
 * by keeping the largest-amplitude bins — this is a standard
 * result, sometimes called the "best M-term Fourier
 * approximation".
 *
 * Visual benefit: the user can dial M up from 1 and see the chain
 * go from "biggest overall sweep" to "biggest two sweeps" to
 * "best 3-term", etc.  Each step adds the next-most-important
 * detail rather than a random extra bit.
 *
 * For very smooth curves the sort puts bins +1 and -1 (the
 * fundamental and its conjugate) at the top, then +2 and -2, etc.
 * — i.e. small frequencies first, exactly as you'd expect.  For
 * sharper curves, higher harmonics get larger and the sorted
 * order may interleave them.
 *
 * In code this is just a qsort with a comparator that returns -1
 * when amp_a > amp_b (§9).  The comparator is two lines.
 *
 * T10 PER-FRAME ANIMATION — WHAT THE MAIN LOOP ACTUALLY DOES
 * ──────────────────────────────────────────────────────────
 * Every frame, in order:
 *
 *   1. Read keys; possibly toggle pause / change shape / etc.
 *   2. Honour any pending SIGWINCH (terminal resize).
 *   3. animation_tick (§13):
 *        - If not paused, advance φ by one frame's worth.
 *        - Maybe auto-grow the chain by one arm.
 *        - At φ wrap (cycle boundary), clear the trail buffer.
 *        - Recompute joint positions from arm table + φ.
 *        - Push the tip into the trail ring.
 *   4. Erase screen, paint orbit guides, paint trail (oldest-to-
 *      newest so newest dots paint over older), paint arm chain
 *      lines, paint pivot + tip markers, paint debug overlay if
 *      toggled, paint HUD, present.
 *   5. Sleep until 1/30 s has elapsed since the frame started.
 *
 * The split between "tick" (state update) and "render" (read-only
 * scene drawing) is tiny here — render touches no simulation
 * state — but it's worth keeping the boundary because the same
 * pattern scales to fixed-timestep accumulator loops in physics
 * sims.
 *
 * T11 CONVERGENCE — WHY SHARP CORNERS NEED MANY MORE ARMS (GIBBS)
 * ───────────────────────────────────────────────────────────────
 * Not all shapes converge at the same speed.
 *
 * SMOOTH curves (curvature continuous, all derivatives bounded):
 * Fourier coefficients decay EXPONENTIALLY with n.  By M ≈ 10 the
 * reconstruction is already pixel-perfect at terminal resolution.
 * Examples: Circle, Ellipse, Heart, Trefoil, Bean.
 *
 * SHARP curves (corners, cusps, anywhere the derivative is
 * discontinuous): coefficients decay only POLYNOMIALLY (typically
 * 1/n or 1/n²).  You may need M ≈ 100+ for crisp corners.  Below
 * that, you see GIBBS RINGING — small overshoots and oscillations
 * near each corner that don't go away as M grows.
 *
 * The mathematical statement of the Gibbs phenomenon (Wilbraham
 * 1848, Gibbs 1898) is that the maximum overshoot at a step
 * discontinuity approaches ~9% of the jump height in the limit
 * M → ∞ AND PERSISTS.  In our setting "step discontinuity" maps
 * to "discontinuous direction at a corner": near the corner the
 * trail bulges outward by a small fixed fraction even with
 * arbitrarily many arms.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   target square corner:    M = 30:        M = 200:      │
 *      │                                                         │
 *      │       │                       __,──         _.--        │
 *      │       │                      /              /           │
 *      │       └────                ─/             ─/            │
 *      │                                                         │
 *      │       crisp 90°            rings + smear   tighter,     │
 *      │                                            still rings  │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * Workarounds exist (Lanczos sigma factor, Hamming/Hann windows)
 * — but they smear the corners.  We don't apply them.  Instead we
 * make Gibbs visible: the user can see ringing on Star, Triangle,
 * Square at low M, and watch it get smaller (but never disappear)
 * as they raise M.  That's a genuine fact about Fourier maths
 * that anyone working with FFTs eventually meets.
 *
 * T12 DRAWING IN THE TERMINAL — BRESENHAM + CELL-ASPECT ELLIPSE
 * ─────────────────────────────────────────────────────────────
 * Two specialised drawing routines.
 *
 * BRESENHAM LINE (§14).  Given two integer cell coordinates
 * (x0, y0) and (x1, y1), trace a one-cell-wide line between them
 * using only integer arithmetic.  At each step the algorithm
 * picks whether to advance the x-axis, y-axis, or both, based on
 * the sign of an accumulated error term.  Originally invented at
 * IBM (Bresenham 1965) for plotting on raster CRTs.
 *
 * Glyph choice: we pick a glyph that approximates the local slope.
 *
 *   horizontal moves  → '-'
 *   vertical   moves  → '|'
 *   diagonal NE→SW    → '/'
 *   diagonal NW→SE    → '\'
 *
 * Reasonable for a quick visualisation; not pixel-perfect.
 *
 * CELL-ASPECT ELLIPSE (§15).  In our PIXEL space (where the chain
 * math lives) terminal cells are CELL_PIXEL_WIDTH × CELL_PIXEL_HEIGHT
 * sub-pixels each.  Currently 8 × 16, so cells are 2× taller than
 * wide.  A pixel-space CIRCLE of radius r therefore appears in
 * cell space as an ELLIPSE with semi-axes:
 *
 *     a_cells = r / CELL_PIXEL_WIDTH
 *     b_cells = r / CELL_PIXEL_HEIGHT
 *
 * If we drew a "circle" of constant cell radius we'd get a
 * vertically squashed oval.  By scaling x and y separately we
 * compensate.  We sample the parametric (a·cos θ, b·sin θ)
 * ellipse at enough θ values (proportional to the ellipse's
 * cell-space circumference) and stamp '.' at each.
 *
 * BACK-TO-FRONT layer order (later layers paint over earlier):
 *
 *     orbit guides  (faint)
 *     trail dots    (red → orange → yellow with age)
 *     arm segments  (cyan + white, A_BOLD)
 *     markers       (pivot + tip, A_BOLD)
 *     debug         (if toggled)
 *     HUD           (always on top)
 *
 * Because we erase the whole screen each frame and repaint, layer
 * compositing is "last writer wins per cell" — no blending.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config — every constant + enum lives here                         */
/* ===================================================================== */
/*
 * Naming convention: SCREAMING_SNAKE_CASE for compile-time #defines,
 * units are spelled in the name (PIXELS, FRAMES, NS = nanoseconds).
 *
 * Every literal in the rest of the file should refer to one of these
 * names.  When you see a bare number elsewhere, ask whether it should
 * be promoted to a named constant here.
 */

/* §1.1 — DFT + animation lengths -------------------------------------- */

#define DFT_SAMPLE_COUNT             256   /* N — input samples & max arms */
#define ANIMATION_FRAMES_PER_CYCLE   360   /* one full curve trace = 12 s @ 30 fps */
#define AUTO_GROW_INTERVAL_FRAMES     12   /* +1 arm every 12 frames */

/* §1.2 — trail ring buffer -------------------------------------------- */

#define TIP_TRAIL_LENGTH             600   /* ~1.5 cycles of recent tip path */

/* §1.3 — frame-rate cap ------------------------------------------------ */

#define NS_PER_SEC          1000000000LL
#define NS_PER_MS              1000000LL
#define RENDER_FPS_CAP                30
#define RENDER_TICK_NS         (NS_PER_SEC / RENDER_FPS_CAP)

/* §1.4 — terminal cell aspect ratio ----------------------------------- */
/*
 * Sub-pixel resolution for the chain math.  Cell-space coordinates are
 * derived by integer divide.  Standard monospace fonts make a cell
 * about twice as tall as it is wide — we encode that 2:1 ratio here.
 */

#define CELL_PIXEL_WIDTH               8
#define CELL_PIXEL_HEIGHT             16

/* §1.5 — visual layout ------------------------------------------------ */

#define SHAPE_PIXEL_SCALE_FRAC      0.38f   /* fraction of min(W,H) */
#define ORBIT_DRAW_COUNT_MAX           6    /* draw at most this many orbit ellipses */
#define ORBIT_DRAW_MIN_RADIUS_PX     ((float)CELL_PIXEL_WIDTH * 0.5f)
#define ARM_RADIUS_THRESHOLD_LARGE  0.10f   /* fraction of overall scale */
#define ARM_RADIUS_THRESHOLD_MID    0.02f

/* §1.6 — colour pair IDs ---------------------------------------------- */
/*
 * Pair 0 is reserved by ncurses for "default on default".  Our pairs
 * start at 1 and are bound to actual colours by §5 colors_init +
 * apply_theme.  HUD/HINT pairs are theme-INVARIANT (CLAUDE.md HUD
 * Standard: bright yellow / bright cyan + A_BOLD always).
 */

enum {
    PAIR_ARM_LARGE   = 1,    /* biggest arms (top of sorted table)       */
    PAIR_ARM_MID,            /* medium arms                              */
    PAIR_ARM_SMALL,          /* small / tail arms                         */
    PAIR_ORBIT,              /* orbit guide ellipses                     */
    PAIR_TRAIL_NEW,          /* most recent tip-path points              */
    PAIR_TRAIL_MID,          /* middle-aged tip-path points              */
    PAIR_TRAIL_OLD,          /* oldest still-visible tip-path points     */
    PAIR_TIP_BOB,            /* the tip marker '@'                        */
    PAIR_PIVOT,              /* the pivot marker '+'                      */
    PAIR_HUD,                /* top status line — theme-invariant        */
    PAIR_HINT,               /* bottom key hint — theme-invariant         */
};

/* §1.7 — shape catalogue size (table in §6) --------------------------- */

#define SHAPE_COUNT                   20

/* §1.8 — theme catalogue size  (table in §4) -------------------------- */

#define THEME_COUNT                    6

/* ===================================================================== */
/* §2  clock — monotonic ns timer + nanosecond sleep                     */
/* ===================================================================== */
/*
 * We use CLOCK_MONOTONIC (not CLOCK_REALTIME) because we care about
 * elapsed-time deltas and don't want NTP jumps to confuse the frame
 * pacer.  Resolution on Linux is ~1 ns; easily enough for a 30 fps
 * cap.
 */

static int64_t clock_now_ns(void)
{
    /* Purpose         : return the current monotonic clock in nanoseconds.
     * Pseudocode      : ts = clock_gettime; return ts.sec*1e9 + ts.nsec.
     * Mental model    : a wall clock that only moves forward, never
     *                   resets, never jumps.
     * Inputs / outputs: none / int64 nanoseconds since some epoch.
     * Why it exists   : the main loop subtracts `now()` values to
     *                   measure frame time and decide how long to sleep.
     */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    /* Purpose         : block the calling thread for exactly `ns` ns.
     * Pseudocode      : if ns <= 0 return; nanosleep((sec, nsec), NULL).
     * Mental model    : the main loop's "sit still until next frame slot".
     * Inputs / outputs: int64 nanoseconds / void.
     * Why it exists   : without this the loop would spin a CPU at 100%.
     */
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  complex — minimal complex-number arithmetic                       */
/* ===================================================================== */
/*
 * The C standard library's <complex.h> would do, but a 2-float struct
 * is more honest about what's happening (no compiler-magic types) and
 * prints cleanly in a debugger.  We need only:
 *
 *     z + w        (vector addition)
 *     z * w        (rotate-and-scale)
 *     exp(i·θ)     (point on the unit circle)
 *     |z|, arg(z)  (radius, angle in polar form)
 *
 * That's it.  Anything else needed by the algorithm is built from
 * these five operations.  See T2 for why complex multiplication is
 * the same as 2-D rotation.
 */

typedef struct {
    float re;   /* real part      */
    float im;   /* imaginary part */
} ComplexNumber;

static inline ComplexNumber complex_make(float re, float im)
{
    return (ComplexNumber){ re, im };
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b)
{
    /* Purpose : (a.re + b.re) + i·(a.im + b.im). */
    return complex_make(a.re + b.re, a.im + b.im);
}

static inline ComplexNumber complex_mul(ComplexNumber a, ComplexNumber b)
{
    /* Purpose : multiply two complex numbers (rotate-and-scale).
     * Derivation:
     *     (a.re + i·a.im) · (b.re + i·b.im)
     *   =  a.re·b.re + i·a.re·b.im + i·a.im·b.re + i²·a.im·b.im
     *   = (a.re·b.re − a.im·b.im) + i·(a.re·b.im + a.im·b.re)
     *     ↑ real part                ↑ imaginary part
     */
    return complex_make(a.re * b.re - a.im * b.im,
                        a.re * b.im + a.im * b.re);
}

static inline ComplexNumber complex_exp_unit(float theta_radians)
{
    /* Purpose : Euler — return cos(θ) + i·sin(θ).  A point on the unit
     *           circle at angle θ measured CCW from the +real axis.
     * Inputs  : θ in radians.
     * Outputs : ComplexNumber with magnitude 1 and argument θ.
     */
    return complex_make(cosf(theta_radians), sinf(theta_radians));
}

static inline float complex_magnitude(ComplexNumber z)
{
    /* |z| = sqrt(re² + im²).  The Pythagorean theorem on the (re, im) point. */
    return sqrtf(z.re * z.re + z.im * z.im);
}

static inline float complex_argument(ComplexNumber z)
{
    /* arg(z) = atan2(im, re).  Returns the angle in (-π, π].
     * atan2 (not plain atan) so that all four quadrants are handled. */
    return atan2f(z.im, z.re);
}

/* ===================================================================== */
/* §4  themes — six colour palettes, swappable at runtime                */
/* ===================================================================== */
/*
 * Each theme provides nine 256-colour codes, one per animation pair
 * (PAIR_ARM_LARGE through PAIR_PIVOT).  The HUD/HINT pairs are not
 * touched by themes — they stay bright yellow / bright cyan in every
 * theme so the status bar is always legible.
 *
 * All colour codes sit in the BRIGHT half of xterm's 256-colour
 * space (>= 24 in the 6×6×6 cube, >= 240 in the greyscale ramp).
 * Codes 16-23 in the cube and 232-239 in the ramp are visually
 * indistinguishable from black with A_DIM and would disappear.
 *
 * Layout of `theme_table`:
 *
 *   ─────── name      ARM_L ARM_M ARM_S ORB   T_NEW T_MID T_OLD TIP   PIV
 *   "Classic"           231    51   246  244   226   208   196  231  220
 *
 * To add a theme: bump THEME_COUNT, add a row, that's it.
 */

typedef struct {
    const char *display_name;
    short arm_large_color;
    short arm_mid_color;
    short arm_small_color;
    short orbit_color;
    short trail_new_color;
    short trail_mid_color;
    short trail_old_color;
    short tip_color;
    short pivot_color;
} ThemePalette;

static const ThemePalette theme_table[THEME_COUNT] = {
    /* Classic — bright white arms, gold/orange/red trail, yellow pivot. */
    { "Classic ",  231,  51, 246, 244, 226, 208, 196, 231, 220 },
    /* Ocean — pale blues + cyan, gold trail for warm contrast. */
    { "Ocean   ",  195, 117,  39,  31, 226, 220, 208, 195,  51 },
    /* Sunset — peach-to-red palette, magenta trail. */
    { "Sunset  ",  223, 215, 209, 174, 201, 213, 198, 231, 220 },
    /* Matrix — green spectrum on default-black. */
    { "Matrix  ",  120,  46,  34,  28, 154,  76,  34, 231, 226 },
    /* Mono — high-contrast greyscale. */
    { "Mono    ",  231, 250, 244, 240, 252, 247, 242, 231, 246 },
    /* Neon — purple/cyan/pink, club-flyer aesthetic. */
    { "Neon    ",  213,  51, 207, 240, 226, 207, 197, 231, 226 },
};

/* ===================================================================== */
/* §5  colors — pair init + apply_theme                                  */
/* ===================================================================== */

static void apply_theme(int theme_index)
{
    /* Purpose         : rebind the nine animation pairs to a theme's
     *                   palette without touching HUD/HINT pairs.
     * Pseudocode      : clamp index → look up row → init_pair for each
     *                   of the nine animation pairs.  8-colour fallback
     *                   ignores the theme and uses ANSI defaults.
     * Mental model    : a stylesheet swap.  ncurses caches the pair→
     *                   colour binding, so re-painting the same chars
     *                   immediately shows the new colours.
     * Inputs / outputs: int theme_index ∈ [0, THEME_COUNT) / void.
     * Why it exists   : called from colors_init at startup AND from
     *                   the 't'/'T' key handler at runtime.  Single
     *                   place to change the policy.
     */
    if (theme_index < 0 || theme_index >= THEME_COUNT) theme_index = 0;
    const ThemePalette *t = &theme_table[theme_index];

    if (COLORS >= 256) {
        init_pair(PAIR_ARM_LARGE,   t->arm_large_color, -1);
        init_pair(PAIR_ARM_MID,     t->arm_mid_color,   -1);
        init_pair(PAIR_ARM_SMALL,   t->arm_small_color, -1);
        init_pair(PAIR_ORBIT,       t->orbit_color,     -1);
        init_pair(PAIR_TRAIL_NEW,   t->trail_new_color, -1);
        init_pair(PAIR_TRAIL_MID,   t->trail_mid_color, -1);
        init_pair(PAIR_TRAIL_OLD,   t->trail_old_color, -1);
        init_pair(PAIR_TIP_BOB,     t->tip_color,       -1);
        init_pair(PAIR_PIVOT,       t->pivot_color,     -1);
    } else {
        /* 8-colour terminal: themes don't have enough resolution to
         * matter.  Use a single legible fallback. */
        init_pair(PAIR_ARM_LARGE,   COLOR_WHITE,  -1);
        init_pair(PAIR_ARM_MID,     COLOR_CYAN,   -1);
        init_pair(PAIR_ARM_SMALL,   COLOR_WHITE,  -1);
        init_pair(PAIR_ORBIT,       COLOR_WHITE,  -1);
        init_pair(PAIR_TRAIL_NEW,   COLOR_YELLOW, -1);
        init_pair(PAIR_TRAIL_MID,   COLOR_YELLOW, -1);
        init_pair(PAIR_TRAIL_OLD,   COLOR_RED,    -1);
        init_pair(PAIR_TIP_BOB,     COLOR_WHITE,  -1);
        init_pair(PAIR_PIVOT,       COLOR_YELLOW, -1);
    }
}

static void colors_init(void)
{
    /* Bind theme-invariant pairs (HUD + HINT) once at startup, then
     * apply the default theme.  Subsequent theme changes go through
     * apply_theme() and leave HUD/HINT alone. */
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,   51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    apply_theme(0);
}

/* ===================================================================== */
/* §6  shapes — 20 parametric curves + vertex helpers                    */
/* ===================================================================== */
/*
 * Each shape is a function t ↦ (x, y) for t ∈ [0, 2π).  The curve is
 * closed (the t=0 and t=2π endpoints coincide).  All shapes are
 * SCALE-NORMALISED to fit roughly inside the unit disc; the actual
 * pixel-space size happens later in §12 chain_compute via the
 * `shape_pixel_scale` multiplier.
 *
 * The shape table at the bottom maps integer shape indices → display
 * name + sampler function.  'n' / 'p' cycle through the table.
 *
 * GROUPING: indices 0..9 are SMOOTH (Fourier converges in ~10-20
 * arms); indices 10..19 are SHARP (need 50-200 arms for crisp
 * features; classic Gibbs ringing visible below).
 *
 * Helpers `polygon_sample` and `star_sample` factor out the
 * vertex-interpolation logic shared by the polygon and star samplers.
 */

/* ── §6.1  polygon vertex interpolation ─────────────────────────────── */

static void polygon_sample(float t, int n_sides,
                           float *out_x, float *out_y)
{
    /* Purpose         : sample a regular n-gon's perimeter at parameter t.
     * Pseudocode      :
     *     normalised = t / (2π) · n_sides           // 0..n_sides
     *     segment    = floor(normalised) mod n_sides
     *     fraction   = normalised - floor(normalised)
     *     v_a        = (cos, sin) at angle for `segment`
     *     v_b        = (cos, sin) at angle for `segment + 1`
     *     return v_a + (v_b - v_a) · fraction       // linear interp
     * Mental model    : march around the polygon at constant arc-rate.
     *                   When t = 0 we're at vertex 0 (top); after going
     *                   1/n_sides of the way around in t, we're at
     *                   vertex 1; in between we walk the chord.
     * Inputs / outputs: t ∈ [0, 2π), n_sides >= 3 / out_x, out_y ∈ [-1,1].
     * Why it exists   : Triangle, Square, Pentagon all reduce to one call.
     */
    float normalised = t / (2.0f * (float)M_PI) * (float)n_sides;
    int   segment    = (int)floorf(normalised) % n_sides;
    float fraction   = normalised - floorf(normalised);
    float angle_base = -(float)M_PI / 2.0f;          /* start at the top */
    float step_angle = 2.0f * (float)M_PI / (float)n_sides;
    float angle_a    = angle_base + (float)segment       * step_angle;
    float angle_b    = angle_base + (float)(segment + 1) * step_angle;
    float x_a = cosf(angle_a),  y_a = sinf(angle_a);
    float x_b = cosf(angle_b),  y_b = sinf(angle_b);
    *out_x = x_a + (x_b - x_a) * fraction;
    *out_y = y_a + (y_b - y_a) * fraction;
}

/* ── §6.2  star (alternating outer/inner radius) ─────────────────────── */

static void star_sample(float t, int n_points, float inner_ratio,
                        float *out_x, float *out_y)
{
    /* Purpose         : sample an n-pointed star: 2·n_points vertices
     *                   alternating between radius 1 (outer) and
     *                   radius `inner_ratio` (inner).
     * Mental model    : same as polygon_sample but with a different
     *                   radius on every other vertex.
     */
    int   total_segs = 2 * n_points;
    float normalised = t / (2.0f * (float)M_PI) * (float)total_segs;
    int   segment    = (int)floorf(normalised) % total_segs;
    float fraction   = normalised - floorf(normalised);
    float angle_base = -(float)M_PI / 2.0f;
    float step_angle = (float)M_PI / (float)n_points;
    float angle_a    = angle_base + (float)segment       * step_angle;
    float angle_b    = angle_base + (float)(segment + 1) * step_angle;
    float radius_a   = (segment % 2 == 0) ? 1.00f : inner_ratio;
    float radius_b   = (segment % 2 == 0) ? inner_ratio : 1.00f;
    float x_a = radius_a * cosf(angle_a),  y_a = radius_a * sinf(angle_a);
    float x_b = radius_b * cosf(angle_b),  y_b = radius_b * sinf(angle_b);
    *out_x = x_a + (x_b - x_a) * fraction;
    *out_y = y_a + (y_b - y_a) * fraction;
}

/* ── §6.3  smooth shapes (indices 0..9) ──────────────────────────────── */

static void shape_sample_circle(float t, float *out_x, float *out_y)
{
    /* The simplest possible target.  ONE Fourier harmonic suffices:
     * |Z[+1]| = N/2 (and its conjugate at signed freq -1); all other
     * bins are zero.  Use this to convince yourself the rest of the
     * pipeline works. */
    *out_x = cosf(t);
    *out_y = sinf(t);
}

static void shape_sample_ellipse(float t, float *out_x, float *out_y)
{
    /* TWO Fourier harmonics suffice: bins +1 and -1 with unequal
     * magnitudes.  The +1 and -1 amplitudes encode the average and
     * difference of the semi-axes. */
    *out_x = cosf(t);
    *out_y = 0.6f * sinf(t);
}

static void shape_sample_cardioid(float t, float *out_x, float *out_y)
{
    /* Polar r = (1 - cos t).  Heart-shaped and very smooth — a few
     * harmonics give visually exact reconstruction.  Same family as
     * the boundary of the Mandelbrot set's main cardioid. */
    float r = (1.0f - cosf(t)) * 0.5f;     /* /2 fits the unit disc */
    *out_x = r * cosf(t);
    *out_y = r * sinf(t);
}

static void shape_sample_limacon(float t, float *out_x, float *out_y)
{
    /* Polar r = 0.5 + 0.5·cos t — Pascal's "snail" curve.  Cousin
     * of the cardioid; the (a, b) parameters tune the dimple. */
    float r = 0.5f + 0.5f * cosf(t);
    *out_x = r * cosf(t);
    *out_y = r * sinf(t);
}

static void shape_sample_heart(float t, float *out_x, float *out_y)
{
    /* The famous "Wolfram heart": x = 16 sin³t, y = 13 cos t - 5 cos 2t
     * - 2 cos 3t - cos 4t.  The y-component already IS a Fourier sum
     * of four harmonics, which is why the DFT recovers it cleanly. */
    *out_x =  16.0f * sinf(t) * sinf(t) * sinf(t) / 17.0f;
    *out_y = -(13.0f * cosf(t)
             - 5.0f  * cosf(2.0f * t)
             - 2.0f  * cosf(3.0f * t)
             - 1.0f  * cosf(4.0f * t)) / 17.0f;
}

static void shape_sample_trefoil(float t, float *out_x, float *out_y)
{
    /* Planar projection of a (2, 3) torus knot.  Three lobes, smooth
     * everywhere; needs ~20 harmonics for visual accuracy. */
    *out_x = (sinf(t) + 2.0f * sinf(2.0f * t)) / 3.2f;
    *out_y = (cosf(t) - 2.0f * cosf(2.0f * t)) / 3.2f;
}

static void shape_sample_figure_eight(float t, float *out_x, float *out_y)
{
    /* Lemniscate of Gerono: x = sin t, y = sin t · cos t = ½ sin 2t.
     * Both x and y are pure sinusoids → very few harmonics needed. */
    *out_x = sinf(t);
    *out_y = sinf(t) * cosf(t);
}

static void shape_sample_lemniscate(float t, float *out_x, float *out_y)
{
    /* Lemniscate of Bernoulli — a smoother infinity-symbol than
     * Gerono's, parameterised so the centre crossing is well-behaved. */
    float denom = 1.0f + sinf(t) * sinf(t);
    *out_x = cosf(t)            / denom;
    *out_y = sinf(t) * cosf(t)  / denom;
}

static void shape_sample_egg(float t, float *out_x, float *out_y)
{
    /* Asymmetric oval: narrower at one end via a (1 + 0.18 cos t)
     * radial modulator. */
    *out_x = cosf(t) * (1.0f + 0.18f * cosf(t)) * 0.85f;
    *out_y = sinf(t);
}

static void shape_sample_bean(float t, float *out_x, float *out_y)
{
    /* Bean / kidney — y = sin t · (1 + cos t) / 2 squashes the lower
     * half asymmetrically inward.  Smooth but lopsided. */
    *out_x = cosf(t);
    *out_y = sinf(t) * (1.0f + cosf(t)) * 0.5f;
}

/* ── §6.4  sharp / complex shapes (indices 10..19) ──────────────────── */

static void shape_sample_triangle(float t, float *x, float *y) { polygon_sample(t, 3, x, y); }
static void shape_sample_square  (float t, float *x, float *y) { polygon_sample(t, 4, x, y); }
static void shape_sample_pentagon(float t, float *x, float *y) { polygon_sample(t, 5, x, y); }

static void shape_sample_star  (float t, float *x, float *y) { star_sample(t, 5, 0.42f, x, y); }
static void shape_sample_star_7(float t, float *x, float *y) { star_sample(t, 7, 0.50f, x, y); }
static void shape_sample_hexagram(float t, float *x, float *y)
{
    /* Star of David: 6 points with inner radius √3/3 ≈ 0.577 keeps
     * the two interlocking triangles equilateral. */
    star_sample(t, 6, 0.577f, x, y);
}

static void shape_sample_astroid(float t, float *out_x, float *out_y)
{
    /* Hypocycloid traced by a point on a small circle of radius 1/4
     * rolling inside a unit circle.  Closed form: x = cos³t, y = sin³t.
     * Four cusps on the axes — classic cusp-convergence test case. */
    float c = cosf(t), s = sinf(t);
    *out_x = c * c * c;
    *out_y = s * s * s;
}

static void shape_sample_rose5(float t, float *out_x, float *out_y)
{
    /* Polar r = cos(5t) — five-petal rose.  Each petal touches the
     * origin with a cusp; a tough Fourier target. */
    float r = cosf(5.0f * t);
    *out_x = r * cosf(t);
    *out_y = r * sinf(t);
}

static void shape_sample_maltese(float t, float *out_x, float *out_y)
{
    /* Disc with four concave indentations on the axes.  The
     * 1 - 0.7·sin²(2t) factor reaches 1 at the diagonals (t = π/4
     * etc.) and 0.3 at the axes (t = 0, π/2 etc.). */
    float r = 1.0f - 0.7f * sinf(2.0f * t) * sinf(2.0f * t);
    *out_x = r * cosf(t);
    *out_y = r * sinf(t);
}

static void shape_sample_butterfly(float t, float *out_x, float *out_y)
{
    /* Temple H. Fay (1989), "The Butterfly Curve", American Math
     * Monthly.  Combines exp(cos t), cos(4t), and sin⁵(t/12).  The
     * Fourier coefficients fall off slowly — this is intentionally
     * a hard-to-converge shape. */
    float envelope = expf(cosf(t))
                   - 2.0f * cosf(4.0f * t)
                   - powf(sinf(t / 12.0f), 5.0f);
    *out_x =  sinf(t) * envelope / 5.0f;
    *out_y = -cosf(t) * envelope / 5.0f;
}

/* ── §6.5  dispatch table ───────────────────────────────────────────── */

typedef struct {
    const char *display_name;
    void (*sampler)(float t, float *out_x, float *out_y);
} ShapeEntry;

static const ShapeEntry shape_table[SHAPE_COUNT] = {
    /* 0..9   smooth  — Fourier converges in ~10-20 arms */
    { "Circle      ",  shape_sample_circle       },
    { "Ellipse     ",  shape_sample_ellipse      },
    { "Cardioid    ",  shape_sample_cardioid     },
    { "Limacon     ",  shape_sample_limacon      },
    { "Heart       ",  shape_sample_heart        },
    { "Trefoil     ",  shape_sample_trefoil      },
    { "Figure-8    ",  shape_sample_figure_eight },
    { "Lemniscate  ",  shape_sample_lemniscate   },
    { "Egg         ",  shape_sample_egg          },
    { "Bean        ",  shape_sample_bean         },
    /* 10..19 sharp   — corners and cusps need 50+ arms (Gibbs visible) */
    { "Triangle    ",  shape_sample_triangle     },
    { "Square      ",  shape_sample_square       },
    { "Pentagon    ",  shape_sample_pentagon     },
    { "Star (5pt)  ",  shape_sample_star         },
    { "Star (7pt)  ",  shape_sample_star_7       },
    { "Hexagram    ",  shape_sample_hexagram     },
    { "Astroid     ",  shape_sample_astroid      },
    { "Rose (5pet) ",  shape_sample_rose5        },
    { "Maltese     ",  shape_sample_maltese      },
    { "Butterfly   ",  shape_sample_butterfly    },
};

/* ===================================================================== */
/* §7  sample — evaluate a chosen shape at N equally-spaced t values     */
/* ===================================================================== */

static void sample_shape_into(int shape_index,
                              ComplexNumber *out_samples,
                              int sample_count)
{
    /* Purpose         : write N complex samples z[k] = x(t_k) + i·y(t_k)
     *                   where t_k = 2π·k/N.
     * Pseudocode      : guard index → look up entry → for k in 0..N-1:
     *                   t = 2π·k/N; entry.sampler(t, &x, &y);
     *                   out[k] = (x, y).
     * Mental model    : sample the curve at N "tick marks" evenly
     *                   spaced around its parameter, packing each
     *                   (x, y) into a complex number for the DFT.
     * Inputs / outputs: shape_index, output array, count / void
     *                   (writes `sample_count` ComplexNumbers).
     * Why it exists   : the DFT operates on samples — this is the
     *                   bridge from "continuous parametric formula"
     *                   to "discrete vector".
     */
    if (shape_index < 0 || shape_index >= SHAPE_COUNT) shape_index = 0;
    const ShapeEntry *entry = &shape_table[shape_index];
    for (int k = 0; k < sample_count; k++) {
        float t = (float)k / (float)sample_count * 2.0f * (float)M_PI;
        float x, y;
        entry->sampler(t, &x, &y);
        out_samples[k] = complex_make(x, y);
    }
}

/* ===================================================================== */
/* §8  dft — O(N²) DFT with twiddle-factor recurrence (T6)               */
/* ===================================================================== */

static void dft_compute_with_twiddle(const ComplexNumber *input_samples,
                                     ComplexNumber       *output_bins,
                                     int                  sample_count)
{
    /* Purpose         : compute Z[n] = Σ_k z[k]·exp(-i·2π·n·k/N) for
     *                   every output bin n ∈ [0, N).
     * Pseudocode      :
     *     for n in 0..N-1:
     *       W           = exp(-i·2π·n/N)        // ONE cos+sin per bin
     *       twiddle     = 1                     // = W^0
     *       sum         = 0
     *       for k in 0..N-1:
     *         sum     += z[k] · twiddle
     *         twiddle *= W                      // advance to W^(k+1)
     *       Z[n] = sum
     *
     * Mental model    : N parallel "lock-in detectors", each tuned to
     *                   one integer harmonic.  Twiddle recurrence
     *                   replaces N² trig calls with 2N (T6).
     * Inputs / outputs: const z[N] / Z[N].  Both arrays caller-allocated.
     * Why it exists   : this is the recipe machine.  It runs ONCE per
     *                   shape change, not per frame.
     */
    for (int n = 0; n < sample_count; n++) {
        float twiddle_angle =
            -2.0f * (float)M_PI * (float)n / (float)sample_count;
        ComplexNumber twiddle_step  = complex_exp_unit(twiddle_angle);
        ComplexNumber twiddle_power = complex_make(1.0f, 0.0f);
        ComplexNumber accumulator   = complex_make(0.0f, 0.0f);

        for (int k = 0; k < sample_count; k++) {
            accumulator   = complex_add(accumulator,
                                        complex_mul(input_samples[k],
                                                    twiddle_power));
            twiddle_power = complex_mul(twiddle_power, twiddle_step);
        }
        output_bins[n] = accumulator;
    }
}

/* ===================================================================== */
/* §9  epicycle — per-arm record + sort comparator                       */
/* ===================================================================== */
/*
 * Three numbers per arm — that's all the chain walker needs.  We
 * normalise amplitude here (divide by N) so that the chain-walking
 * code in §12 doesn't need to know about the DFT's N factor.
 */

typedef struct {
    float amplitude_normalised;   /* |Z[n]| / N — in shape units, ~[0, 1] */
    float phase_offset_radians;   /* arg(Z[n])                             */
    int   frequency_signed;       /* signed integer harmonic (T8)          */
} Epicycle;

static Epicycle epicycle_table[DFT_SAMPLE_COUNT];
static int      total_epicycle_count  = 0;
static int      active_epicycle_count = 1;

static int epicycle_compare_descending(const void *a, const void *b)
{
    /* Purpose : qsort comparator for amplitude DESCENDING sort.
     * Returns : negative if `a` should come first (a has larger amp).
     * Why it exists : §10 sorts the table so that the user can
     *                 dial M up from 1 and always get the best
     *                 M-arm approximation (T9).
     */
    float amp_a = ((const Epicycle *)a)->amplitude_normalised;
    float amp_b = ((const Epicycle *)b)->amplitude_normalised;
    if (amp_a < amp_b) return  1;
    if (amp_a > amp_b) return -1;
    return 0;
}

/* ===================================================================== */
/* §10  build — sample → DFT → decode → sort                             */
/* ===================================================================== */

static void build_sorted_epicycle_table(int shape_index)
{
    /* Purpose         : turn a chosen shape into a sorted Epicycle[]
     *                   ready for animation.
     * Pseudocode      :
     *     samples = sample_shape_into(shape_index)
     *     bins    = dft_compute_with_twiddle(samples)
     *     for n in 0..N-1:
     *       table[n] = decode_bin(bins[n], n, N)         // T7 + T8
     *     qsort(table, descending by amplitude)
     *
     * Mental model    : run the recipe machine once, then sort the
     *                   recipe so the most important ingredients are
     *                   listed first.
     * Inputs / outputs: shape_index / fills global epicycle_table.
     * Why it exists   : called at startup, on shape change ('n'/'p'),
     *                   and on reset ('r').  Keeps the DFT cost out
     *                   of the hot path.
     */
    static ComplexNumber input_samples[DFT_SAMPLE_COUNT];
    static ComplexNumber dft_bins     [DFT_SAMPLE_COUNT];

    sample_shape_into(shape_index, input_samples, DFT_SAMPLE_COUNT);
    dft_compute_with_twiddle(input_samples, dft_bins, DFT_SAMPLE_COUNT);

    float inverse_n = 1.0f / (float)DFT_SAMPLE_COUNT;
    for (int n = 0; n < DFT_SAMPLE_COUNT; n++) {
        Epicycle e;
        e.amplitude_normalised = complex_magnitude(dft_bins[n]) * inverse_n;
        e.phase_offset_radians = complex_argument(dft_bins[n]);
        /* Upper-half remap (T8): bins above N/2 represent negative
         * frequencies (clockwise rotations).  Without this, all arms
         * spin CCW and the reconstruction comes out mirrored. */
        e.frequency_signed     = (n <= DFT_SAMPLE_COUNT / 2)
                               ? n
                               : n - DFT_SAMPLE_COUNT;
        epicycle_table[n] = e;
    }
    qsort(epicycle_table, DFT_SAMPLE_COUNT, sizeof(Epicycle),
          epicycle_compare_descending);
    total_epicycle_count = DFT_SAMPLE_COUNT;
}

/* ===================================================================== */
/* §11  trail — ring buffer for the tip's recent positions               */
/* ===================================================================== */
/*
 * A fixed-size circular buffer.  `write_head` is the next index to
 * write.  `filled_count` saturates at TIP_TRAIL_LENGTH.  The oldest
 * filled element is at (write_head - filled_count) mod LENGTH.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   capacity = 8                                          │
 *      │   write_head = 3, filled_count = 6                      │
 *      │                                                         │
 *      │   indices:    0  1  2  3  4  5  6  7                    │
 *      │   contents:  [N][N][N][_][O][O][O][O]                   │
 *      │                                                         │
 *      │   N = newer than, O = older than write_head             │
 *      │   oldest at index (3 - 6 + 8) % 8 = 5                   │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 */

typedef struct {
    float pixel_x[TIP_TRAIL_LENGTH];
    float pixel_y[TIP_TRAIL_LENGTH];
    int   write_head;
    int   filled_count;
} TipTrail;

static TipTrail tip_trail;

static void trail_push(TipTrail *trail, float pixel_x, float pixel_y)
{
    /* Append one (x, y) to the ring; advance head; saturate count. */
    trail->pixel_x[trail->write_head] = pixel_x;
    trail->pixel_y[trail->write_head] = pixel_y;
    trail->write_head = (trail->write_head + 1) % TIP_TRAIL_LENGTH;
    if (trail->filled_count < TIP_TRAIL_LENGTH) trail->filled_count++;
}

static void trail_clear(TipTrail *trail)
{
    /* Logically empty the ring without zeroing the data array. */
    trail->write_head   = 0;
    trail->filled_count = 0;
}

/* ===================================================================== */
/* §12  chain — walk the arms, record joint positions                    */
/* ===================================================================== */

/* Joint positions in pixel space.  joint[0] is the pivot; joint[i+1]
 * is the tip of arm i.  We allocate one extra slot so that joint
 * accesses by `active_epicycle_count` are always in bounds. */
static float joint_pixel_x[DFT_SAMPLE_COUNT + 1];
static float joint_pixel_y[DFT_SAMPLE_COUNT + 1];

/* Pivot + chain scale, set by scene_init based on terminal size. */
static float pivot_pixel_x         = 0.0f;
static float pivot_pixel_y         = 0.0f;
static float shape_pixel_scale     = 1.0f;

/* Global animation phase φ.  Advances each frame in animation_tick. */
static float animation_phase_radians = 0.0f;

static void compute_chain_joint_positions(void)
{
    /* Purpose         : evaluate the chain at the current φ; record
     *                   joint positions in `joint_pixel_x/y[]`.
     * Pseudocode      :
     *     (x, y) = (pivot_x, pivot_y)
     *     joint[0] = (x, y)
     *     for i in 0..M-1:
     *       angle  = freq_i · φ + phase_i
     *       radius = amp_i · shape_pixel_scale
     *       x += radius · cos(angle)
     *       y += radius · sin(angle)
     *       joint[i+1] = (x, y)
     *
     * Mental model    : add the i-th arm's tip displacement to the
     *                   running joint position.  Each arm is a polar-
     *                   to-Cartesian conversion (T3).
     * Inputs / outputs: globals (epicycle_table, animation_phase_radians,
     *                   pivot, scale, active_epicycle_count) / fills
     *                   joint_pixel_x[0..M] and joint_pixel_y[0..M].
     * Why it exists   : the rendering layers consume joint positions,
     *                   not arm parameters.  Computing joints once per
     *                   frame avoids redundant work.
     */
    float x = pivot_pixel_x;
    float y = pivot_pixel_y;
    joint_pixel_x[0] = x;
    joint_pixel_y[0] = y;
    for (int i = 0; i < active_epicycle_count; i++) {
        const Epicycle *e = &epicycle_table[i];
        float angle = (float)e->frequency_signed * animation_phase_radians
                    + e->phase_offset_radians;
        float radius_pixels = e->amplitude_normalised * shape_pixel_scale;
        x += radius_pixels * cosf(angle);
        y += radius_pixels * sinf(angle);
        joint_pixel_x[i + 1] = x;
        joint_pixel_y[i + 1] = y;
    }
}

/* ===================================================================== */
/* §13  scene — animation state + per-frame tick                         */
/* ===================================================================== */
/*
 * Globals here are user-toggleable knobs.  Keeping them as named
 * file-scope variables (rather than a Scene struct) is a deliberate
 * teaching choice — the reader can grep for "auto_grow_enabled" and
 * find every place it's read or written.
 */

static int     screen_rows           = 0;
static int     screen_cols           = 0;
static int     active_shape_index    = 0;
static int     active_theme_index    = 0;
static bool    simulation_paused     = false;
static bool    auto_grow_enabled     = true;
static bool    show_orbits_enabled   = true;
static bool    show_debug_spectrum   = false;     /* 'd' */
static bool    show_debug_arm_table  = false;     /* 'D' */
static int     auto_grow_counter     = 0;

static void scene_reset_for_shape(int shape_index)
{
    /* Pick a new shape (or the same one); rebuild the arm table; rewind. */
    active_shape_index      = shape_index;
    animation_phase_radians = 0.0f;
    auto_grow_counter       = 0;
    active_epicycle_count   = 1;
    trail_clear(&tip_trail);
    build_sorted_epicycle_table(active_shape_index);
    compute_chain_joint_positions();
}

static void scene_init(int rows, int cols)
{
    /* Bind the chain to the current terminal size.  Called at startup
     * and on every SIGWINCH (terminal resize). */
    screen_rows = rows;
    screen_cols = cols;

    /* Centre the pivot in pixel space. */
    pivot_pixel_x = (float)(cols * CELL_PIXEL_WIDTH ) * 0.5f;
    pivot_pixel_y = (float)(rows * CELL_PIXEL_HEIGHT) * 0.5f;

    /* Choose the chain's overall pixel size as a fraction of the
     * smaller terminal dimension, so the curve fits even when the
     * terminal is portrait-oriented. */
    float min_pixels = fminf((float)(cols * CELL_PIXEL_WIDTH),
                             (float)(rows * CELL_PIXEL_HEIGHT));
    shape_pixel_scale = min_pixels * SHAPE_PIXEL_SCALE_FRAC;

    simulation_paused    = false;
    auto_grow_enabled    = true;
    show_orbits_enabled  = true;
    show_debug_spectrum  = false;
    show_debug_arm_table = false;
    scene_reset_for_shape(active_shape_index);
}

static void animation_tick(void)
{
    /* Purpose : advance animation state by ONE frame.
     * What changes:
     *   - active_epicycle_count, if auto-grow elapsed.
     *   - animation_phase_radians.
     *   - tip_trail (cleared at cycle wrap; pushed every frame).
     *   - joint_pixel_x/y[] (recomputed from new φ).
     * Why it exists : the main loop calls this once per frame.  All
     *                 simulation-state updates live here, not in
     *                 render code.
     */
    if (simulation_paused) return;

    /* Auto-grow the chain by one arm every AUTO_GROW_INTERVAL_FRAMES
     * frames, so the user can watch reconstruction quality improve
     * without touching the keyboard. */
    if (auto_grow_enabled
        && active_epicycle_count < total_epicycle_count) {
        auto_grow_counter++;
        if (auto_grow_counter >= AUTO_GROW_INTERVAL_FRAMES) {
            auto_grow_counter = 0;
            active_epicycle_count++;
        }
    }

    /* Advance φ by 2π / FRAMES_PER_CYCLE = exactly one degree per
     * frame at the default 360-frame cycle. */
    animation_phase_radians += 2.0f * (float)M_PI
                             / (float)ANIMATION_FRAMES_PER_CYCLE;

    /* On full-cycle wrap, clear the trail so the next traversal of
     * the curve paints onto a fresh canvas (otherwise stale dots
     * accumulate where the trail never gets re-overwritten). */
    if (animation_phase_radians >= 2.0f * (float)M_PI) {
        animation_phase_radians -= 2.0f * (float)M_PI;
        trail_clear(&tip_trail);
    }

    compute_chain_joint_positions();
    trail_push(&tip_trail,
               joint_pixel_x[active_epicycle_count],
               joint_pixel_y[active_epicycle_count]);
}

/* ===================================================================== */
/* §14  bresenham — integer-only line rasteriser                         */
/* ===================================================================== */

static inline int pixel_to_cell_col(float pixel_x)
{
    /* +0.5 rounds nearest; integer divide rounds toward zero only
     * AFTER shifting to a non-negative range, which we don't bother
     * with because pivot+arm pixels are always non-negative on a
     * sane terminal. */
    return (int)(pixel_x / (float)CELL_PIXEL_WIDTH + 0.5f);
}

static inline int pixel_to_cell_row(float pixel_y)
{
    return (int)(pixel_y / (float)CELL_PIXEL_HEIGHT + 0.5f);
}

static char bresenham_glyph_for_step(int err, int dx, int dy,
                                     int sx, int sy)
{
    /* Pick a glyph approximating the local slope.
     *   advancing both x and y this step → diagonal: '/' or '\'
     *   advancing only x → horizontal: '-'
     *   advancing only y → vertical:   '|'
     */
    int e2 = 2 * err;
    bool advance_x = (e2 > -dy);
    bool advance_y = (e2 <  dx);
    if (advance_x && advance_y)
        return (sx == sy) ? '\\' : '/';
    if (advance_x) return '-';
    return '|';
}

static void bresenham_line(int x0, int y0, int x1, int y1, attr_t attr)
{
    /* Purpose : draw a line of cells from (x0,y0) to (x1,y1).
     * Uses Bresenham's 1965 integer-only algorithm: at each step
     * an `err` accumulator decides whether to bump x, y, or both.
     * Glyph chosen per step by `bresenham_glyph_for_step`.
     */
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    for (;;) {
        if (x0 >= 0 && x0 < screen_cols && y0 >= 0 && y0 < screen_rows) {
            char g = bresenham_glyph_for_step(err, dx, dy, sx, sy);
            attron(attr);
            mvaddch(y0, x0, (chtype)(unsigned char)g);
            attroff(attr);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/* ===================================================================== */
/* §15  ellipse — pixel-circle → cell-aspect ellipse                     */
/* ===================================================================== */
/*
 * In PIXEL space the orbit is a circle of `radius_pixels`.  In CELL
 * space, since cells are CELL_PIXEL_HEIGHT/CELL_PIXEL_WIDTH = 2× as
 * tall as wide, that circle appears as an ellipse with semi-axes:
 *
 *     a_cells = radius_pixels / CELL_PIXEL_WIDTH
 *     b_cells = radius_pixels / CELL_PIXEL_HEIGHT
 *
 * We approximate the ellipse by stamping '.' at parametric samples
 * (a·cos θ, b·sin θ) for θ stepping evenly around 2π.  The number of
 * samples scales with the larger semi-axis so we don't overdraw small
 * ellipses or under-draw big ones.
 */

static void render_orbit_ellipse(float pivot_pixel_x_local,
                                 float pivot_pixel_y_local,
                                 float radius_pixels)
{
    if (radius_pixels < ORBIT_DRAW_MIN_RADIUS_PX) return;

    float semi_axis_x = radius_pixels / (float)CELL_PIXEL_WIDTH;
    float semi_axis_y = radius_pixels / (float)CELL_PIXEL_HEIGHT;
    int   centre_col  = pixel_to_cell_col(pivot_pixel_x_local);
    int   centre_row  = pixel_to_cell_row(pivot_pixel_y_local);

    /* Sample count proportional to circumference (≈ 2π · max(a, b)).
     * +4 keeps very small ellipses from collapsing to a single dot. */
    int sample_count = (int)(2.0f * (float)M_PI
                       * fmaxf(semi_axis_x, semi_axis_y)) + 4;

    attr_t attr = COLOR_PAIR(PAIR_ORBIT);
    attron(attr);
    for (int i = 0; i < sample_count; i++) {
        float theta = 2.0f * (float)M_PI * (float)i / (float)sample_count;
        int x = centre_col + (int)(semi_axis_x * cosf(theta) + 0.5f);
        int y = centre_row + (int)(semi_axis_y * sinf(theta) + 0.5f);
        if (x >= 0 && x < screen_cols && y >= 0 && y < screen_rows)
            mvaddch(y, x, '.');
    }
    attroff(attr);
}

/* ===================================================================== */
/* §16  render_orbits — back layer: faint orbit guides                   */
/* ===================================================================== */

static void render_orbit_layer(void)
{
    /* Draw at most ORBIT_DRAW_COUNT_MAX orbit ellipses, one centred
     * on each joint.  We cap the count because drawing every arm's
     * orbit becomes visual clutter past ~6 arms and the smaller
     * orbits collapse to single cells anyway. */
    if (!show_orbits_enabled) return;
    int draw_count = (active_epicycle_count < ORBIT_DRAW_COUNT_MAX)
                   ? active_epicycle_count : ORBIT_DRAW_COUNT_MAX;
    for (int i = 0; i < draw_count; i++) {
        float radius_pixels = epicycle_table[i].amplitude_normalised
                            * shape_pixel_scale;
        render_orbit_ellipse(joint_pixel_x[i], joint_pixel_y[i],
                             radius_pixels);
    }
}

/* ===================================================================== */
/* §17  render_trail — middle layer: fading tip path                     */
/* ===================================================================== */
/*
 * Walk the trail OLDEST → NEWEST so newer dots paint over older ones.
 * Map normalised age fraction [0, 1] to one of three colour pairs:
 *   0.00..0.35  → TRAIL_OLD  (dimmest, e.g. red)
 *   0.35..0.70  → TRAIL_MID  (e.g. orange)
 *   0.70..1.00  → TRAIL_NEW  (brightest, e.g. yellow)
 *
 * Three discrete tiers (instead of a continuous gradient) is fine
 * for terminal output; finer gradation would require 256-colour
 * grayscale ramps and look noisy.
 */

static void render_trail_layer(void)
{
    int filled = tip_trail.filled_count;
    if (filled == 0) return;

    int oldest_index = (tip_trail.write_head - filled + TIP_TRAIL_LENGTH)
                       % TIP_TRAIL_LENGTH;
    for (int i = 0; i < filled; i++) {
        int index = (oldest_index + i) % TIP_TRAIL_LENGTH;
        int col = pixel_to_cell_col(tip_trail.pixel_x[index]);
        int row = pixel_to_cell_row(tip_trail.pixel_y[index]);
        if (col < 0 || col >= screen_cols) continue;
        if (row < 0 || row >= screen_rows) continue;

        float age_fraction = (float)i / (float)filled;
        int   pair_id = (age_fraction > 0.70f) ? PAIR_TRAIL_NEW
                      : (age_fraction > 0.35f) ? PAIR_TRAIL_MID
                                                : PAIR_TRAIL_OLD;
        attr_t attr = COLOR_PAIR(pair_id) | A_BOLD;
        attron(attr);
        mvaddch(row, col, '*');
        attroff(attr);
    }
}

/* ===================================================================== */
/* §18  render_chain — arm segments                                      */
/* ===================================================================== */

static int arm_pair_for_radius(float radius_pixels)
{
    /* Three-tier classification by radius.  Lets the eye distinguish
     * the dominant arms (bright white) from the tail of tiny corrective
     * arms (dim grey) — useful at high M. */
    if (radius_pixels > shape_pixel_scale * ARM_RADIUS_THRESHOLD_LARGE)
        return PAIR_ARM_LARGE;
    if (radius_pixels > shape_pixel_scale * ARM_RADIUS_THRESHOLD_MID)
        return PAIR_ARM_MID;
    return PAIR_ARM_SMALL;
}

static void render_chain_layer(void)
{
    /* Draw a Bresenham line for every arm: from joint[i] to joint[i+1]. */
    for (int i = 0; i < active_epicycle_count; i++) {
        float radius_pixels = epicycle_table[i].amplitude_normalised
                            * shape_pixel_scale;
        int   pair = arm_pair_for_radius(radius_pixels);
        attr_t attr = COLOR_PAIR(pair) | A_BOLD;
        bresenham_line(
            pixel_to_cell_col(joint_pixel_x[i]),
            pixel_to_cell_row(joint_pixel_y[i]),
            pixel_to_cell_col(joint_pixel_x[i + 1]),
            pixel_to_cell_row(joint_pixel_y[i + 1]),
            attr);
    }
}

/* ===================================================================== */
/* §19  render_markers — pivot dot + tip bob                             */
/* ===================================================================== */

static void render_markers_layer(void)
{
    /* Two single-cell glyphs:
     *   '@' at the very tip (the pen tracing the curve)
     *   '+' at the pivot (where the chain attaches)
     * Both bright + bold so they pop against the trail and arms. */

    int tip_col = pixel_to_cell_col(joint_pixel_x[active_epicycle_count]);
    int tip_row = pixel_to_cell_row(joint_pixel_y[active_epicycle_count]);
    if (tip_col >= 0 && tip_col < screen_cols
     && tip_row >= 0 && tip_row < screen_rows) {
        attr_t attr = COLOR_PAIR(PAIR_TIP_BOB) | A_BOLD;
        attron(attr);
        mvaddch(tip_row, tip_col, '@');
        attroff(attr);
    }

    int pivot_col = pixel_to_cell_col(pivot_pixel_x);
    int pivot_row = pixel_to_cell_row(pivot_pixel_y);
    if (pivot_col >= 0 && pivot_col < screen_cols
     && pivot_row >= 0 && pivot_row < screen_rows) {
        attr_t attr = COLOR_PAIR(PAIR_PIVOT) | A_BOLD;
        attron(attr);
        mvaddch(pivot_row, pivot_col, '+');
        attroff(attr);
    }
}

/* ===================================================================== */
/* §20  render_debug — 'd' rank-sorted amplitude chart, 'D' arm table    */
/* ===================================================================== */
/*
 * Two optional teaching overlays.  Neither is needed to run the
 * simulation; both visualise intermediate state that makes the
 * algorithm easier to understand.
 *
 *   'd' — bottom-left bar chart of the first ~30 sorted arm
 *         amplitudes.  Shape of the chart is the Fourier coefficient
 *         decay rate: tall first bar with nothing after for a
 *         circle; a long slowly-decaying tail for a star.
 *
 *   'D' — top-left numeric table for the first 8 arms, showing
 *         (i, freq_signed, amplitude_normalised, phase_offset).
 *         Useful for verifying the DFT-decode logic by hand.
 *
 * Debug overlays use the existing ARM_LARGE/MID/SMALL pairs so they
 * automatically follow the active theme.
 */

#define DEBUG_SPECTRUM_BAR_COUNT     30
#define DEBUG_SPECTRUM_BAR_HEIGHT     6
#define DEBUG_ARM_TABLE_ROWS          8

static void render_debug_spectrum(void)
{
    if (!show_debug_spectrum) return;

    int num_bars = (active_epicycle_count < DEBUG_SPECTRUM_BAR_COUNT)
                 ? active_epicycle_count : DEBUG_SPECTRUM_BAR_COUNT;
    if (num_bars <= 0) return;

    float max_amp = epicycle_table[0].amplitude_normalised;
    if (max_amp <= 0.0f) return;

    int chart_left   = 1;
    int chart_bottom = screen_rows - 3;   /* leave room for hint line */

    /* Label row sits one below the bottom of the chart. */
    if (chart_bottom + 1 < screen_rows) {
        attron(COLOR_PAIR(PAIR_HINT));
        mvprintw(chart_bottom + 1, chart_left, "amplitude (sorted by rank)");
        attroff(COLOR_PAIR(PAIR_HINT));
    }

    for (int i = 0; i < num_bars; i++) {
        float frac = epicycle_table[i].amplitude_normalised / max_amp;
        int   h    = (int)(frac * (float)DEBUG_SPECTRUM_BAR_HEIGHT + 0.5f);
        if (h < 1 && frac > 0.001f) h = 1;
        for (int row = 0; row < h; row++) {
            int x = chart_left + i;
            int y = chart_bottom - row;
            if (x < 0 || x >= screen_cols) continue;
            if (y < 1 || y >= screen_rows) continue;
            int pair = (i < 1) ? PAIR_ARM_LARGE
                     : (i < 6) ? PAIR_ARM_MID
                               : PAIR_ARM_SMALL;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(y, x, '#');
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

static void render_debug_arm_table(void)
{
    if (!show_debug_arm_table) return;

    int rows_to_show = (active_epicycle_count < DEBUG_ARM_TABLE_ROWS)
                     ? active_epicycle_count : DEBUG_ARM_TABLE_ROWS;
    int x = 2;
    int y = 2;
    if (y + rows_to_show + 1 >= screen_rows) return;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, "  i  freq    amp     phase ");
    attroff(COLOR_PAIR(PAIR_HINT));

    for (int i = 0; i < rows_to_show; i++) {
        const Epicycle *e = &epicycle_table[i];
        int pair = (i < 1) ? PAIR_ARM_LARGE
                 : (i < 6) ? PAIR_ARM_MID
                           : PAIR_ARM_SMALL;
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvprintw(y + 1 + i, x, " %2d  %4d  %5.3f  %+5.2f",
                 i,
                 e->frequency_signed,
                 (double)e->amplitude_normalised,
                 (double)e->phase_offset_radians);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* ===================================================================== */
/* §21  hud — top status line + bottom key hint                          */
/* ===================================================================== */
/*
 * HUD Standard (CLAUDE.md):
 *   - bright yellow + A_BOLD top-right status, never A_DIM.
 *   - bright cyan + A_BOLD bottom-left key hint.
 * Both pairs are theme-invariant so they stay legible against any
 * combination of arm/trail colours.
 */

static void hud_paint_status(int term_cols, double measured_fps)
{
    char buf[200];
    snprintf(buf, sizeof buf,
             " Epicycles  %s  thm:%s  arms:%3d/%-3d  %s  %s  %5.1f fps  %s ",
             shape_table[active_shape_index].display_name,
             theme_table[active_theme_index].display_name,
             active_epicycle_count, total_epicycle_count,
             auto_grow_enabled    ? "auto"    : "manual",
             show_orbits_enabled  ? "orbits"  : "no-orbits",
             measured_fps,
             simulation_paused    ? "PAUSED " : "running");
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
             " q:quit  spc:pause  n/p:shape  t/T:theme  +/-:arms  "
             "r:reset  a:auto  c:circles  d/D:debug ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §22  screen — ncurses init / cleanup / resize / present               */
/* ===================================================================== */

typedef struct { int rows, cols; } Screen;

static void screen_init(Screen *screen)
{
    initscr();
    noecho();
    cbreak();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);          /* don't let stdin peeks interrupt blits */
    colors_init();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_cleanup(void)
{
    endwin();
}

static void screen_resize(Screen *screen)
{
    /* On SIGWINCH, ncurses needs an endwin/refresh cycle so it re-
     * reads the new terminal size before getmaxyx tells us anything. */
    endwin();
    refresh();
    getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, double measured_fps)
{
    /* Layer order matters: each layer paints over the previous in
     * any cells they share.  HUD is always last (always on top). */
    erase();
    render_orbit_layer();
    render_trail_layer();
    render_chain_layer();
    render_markers_layer();
    render_debug_spectrum();
    render_debug_arm_table();
    hud_paint_status(screen->cols, measured_fps);
    hud_paint_hint  (screen->rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §23  app — main loop, signal handlers, key dispatch                   */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    /* Signal handlers MUST stay async-signal-safe: only set flags;
     * the main loop does the real work. */
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit    = 1;
}

static bool app_handle_key(int ch)
{
    /* Returns false if the program should quit.  Quiet on unknown keys. */
    switch (ch) {
        case 'q': case 'Q': case 27:    /* 27 = ESC */
            return false;

        case ' ':
            simulation_paused = !simulation_paused;
            break;

        case 'n': case 'N':
            scene_reset_for_shape((active_shape_index + 1) % SHAPE_COUNT);
            break;
        case 'p': case 'P':
            scene_reset_for_shape((active_shape_index + SHAPE_COUNT - 1)
                                  % SHAPE_COUNT);
            break;

        case 't':
            active_theme_index = (active_theme_index + 1) % THEME_COUNT;
            apply_theme(active_theme_index);
            break;
        case 'T':
            active_theme_index = (active_theme_index + THEME_COUNT - 1)
                                 % THEME_COUNT;
            apply_theme(active_theme_index);
            break;

        case '+': case '=':
            if (active_epicycle_count < total_epicycle_count)
                active_epicycle_count++;
            break;
        case '-':
            if (active_epicycle_count > 1) {
                active_epicycle_count--;
                trail_clear(&tip_trail);
            }
            break;

        case 'r': case 'R':
            scene_reset_for_shape(active_shape_index);
            break;

        case 'a': case 'A':
            auto_grow_enabled = !auto_grow_enabled;
            break;

        case 'c': case 'C':
            show_orbits_enabled = !show_orbits_enabled;
            break;

        case 'd':
            show_debug_spectrum = !show_debug_spectrum;
            break;
        case 'D':
            show_debug_arm_table = !show_debug_arm_table;
            break;

        default: break;
    }
    return true;
}

int main(void)
{
    atexit(screen_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    Screen screen;
    screen_init(&screen);
    scene_init(screen.rows, screen.cols);

    int64_t prev_frame_ns    = clock_now_ns();
    int64_t fps_window_ns    = 0;
    int     frames_in_window = 0;
    double  measured_fps     = 0.0;

    while (!g_should_quit) {
        int64_t frame_start_ns = clock_now_ns();

        /* ── input ────────────────────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(ch)) {
                g_should_quit = 1;
                break;
            }
        }

        /* ── resize ───────────────────────────────────────────────── */
        if (g_resize_pending) {
            g_resize_pending = 0;
            screen_resize(&screen);
            scene_init(screen.rows, screen.cols);
            prev_frame_ns = clock_now_ns();
        }

        /* ── dt + rolling-fps measurement ─────────────────────────── */
        int64_t dt_ns = frame_start_ns - prev_frame_ns;
        prev_frame_ns = frame_start_ns;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;

        frames_in_window++;
        fps_window_ns += dt_ns;
        if (fps_window_ns >= 500 * NS_PER_MS) {
            measured_fps = (double)frames_in_window
                         / ((double)fps_window_ns / (double)NS_PER_SEC);
            frames_in_window = 0;
            fps_window_ns    = 0;
        }

        /* ── animation + render ──────────────────────────────────── */
        animation_tick();
        screen_present_frame(&screen, measured_fps);

        /* ── frame cap (sleep BEFORE next iteration's I/O) ───────── */
        int64_t spent = clock_now_ns() - frame_start_ns;
        if (spent < RENDER_TICK_NS) clock_sleep_ns(RENDER_TICK_NS - spent);
    }

    return 0;
}
