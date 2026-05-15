/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dft_helloworld.c — animated, minimum-viable Discrete Fourier Transform
 *                    demo.  No FFT, no twiddle recurrence — just the
 *                    textbook double-loop DFT, run live every frame on
 *                    a moving cosine, with the spectrum painted as a
 *                    bar chart.
 *
 * DEMO
 *   Two stacked panels.  TOP: the input signal x[n] = cos(2*pi*f*n/N)
 *   as a vertical-bar waveform.  BOTTOM: the magnitude spectrum |X[k]|
 *   computed by the DFT, also as vertical bars.  By default the input
 *   frequency f auto-sweeps between bin 1 and bin N/2-1; you watch
 *   the spike in the spectrum panel walk left and right while the
 *   wave shape morphs.  Press 'a' to switch to manual mode where
 *   '+' / '-' steps the frequency by 1 bin and ',' / '.' by 0.1 bin.
 *
 *   Two debug overlays add depth without adding clutter:
 *     'd' shows a phase panel — arg(X[k]) for each bin, color-coded
 *         by sign.  Magnitude alone discards phase; this shows what
 *         the spectrum panel throws away.
 *     'D' shows a numeric table of the eight loudest bins
 *         (k, |X[k]|, arg(X[k])).  Useful for verifying decode
 *         by hand.
 *
 *   The whole point of this file is to make ONE sentence physically
 *   undeniable: a Discrete Fourier Transform is a bank of "lock-in
 *   detectors", one per integer frequency.  When the input contains
 *   that frequency, the corresponding detector lights up; otherwise
 *   it stays dark.  Sweep the input frequency and you see the lit
 *   detector chase the input — proof, in motion, that the DFT does
 *   exactly what the formula says.
 *
 * Study alongside
 *   signal/fft_helloworld.c   the same demo and verification UX, but
 *                              the spectrum is computed by the FFT
 *                              (and verified bin-by-bin against this
 *                              file's DFT every frame).  Read this
 *                              file FIRST, then move on for the
 *                              "same answer, faster" story.
 *   signal/fft_vis.c          the same DFT/FFT scaled up: 7 signal
 *                              modes, 4 windows, spectrogram waterfall.
 *   signal/epicycles.c        the same DFT machinery applied to closed
 *                              2-D curves (epicycle chain animation).
 *   ncurses_basics/framework.c the canonical animation framework that
 *                              this file follows section-by-section.
 *
 * Section map
 *   §1   config            constants (N, FPS, sweep, freq range)
 *   §2   clock             monotonic-ns timer + sleep
 *   §3   complex           ComplexNumber type + 6 named helpers
 *   §4   colors            palette pairs (HUD/HINT theme-invariant)
 *   §5   dft_naive         the textbook O(N^2) DFT (THE function here)
 *   §6   signal            generate cosine at current frequency
 *   §7   scene_state       App-level globals + reset
 *   §8   scene_tick        per-frame update: sweep -> generate -> DFT
 *   §9   scene_input       handle manual freq-adjust keys
 *   §10  draw_bar          vertical-bar primitive
 *   §11  draw_signal       TIME panel renderer
 *   §12  draw_spectrum     FREQ panel (magnitude) renderer
 *   §13  draw_phase        debug 'd' phase panel
 *   §14  draw_arm_table    debug 'D' top-N bin-info table
 *   §15  hud               status top-right + hint bottom + PAUSED chip
 *   §16  app               signal handlers + main loop + key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   a                toggle auto-sweep <-> manual mode
 *   + / -            (manual) change freq by 1 bin
 *   , / .            (manual) change freq by 0.1 bin (fine — for leakage)
 *   d                toggle phase panel (debug)
 *   D                toggle bin-info numeric table top-left (debug)
 *   r                reset (auto-sweep on, freq = 1.0)
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/dft_helloworld.c \
 *       -o dft_helloworld -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no Fourier background:
 *     1. CONCEPTS   — the elevator pitch with references.
 *     2. MENTAL MODEL — analogies, diagrams, invariants.
 *     3. GUIDED TUTORIAL T1..T11 — eleven mini-lessons that derive
 *        the DFT from first principles.  Each ends in pseudocode that
 *        maps 1:1 to a function in §5..§9.  T5 walks an N=4 worked
 *        example you can step through by hand.
 *     4. §1 config — the knobs (now you know what they do).
 *     5. §3 complex — the tiny abstraction that lets the algorithm
 *        code read like the math.  Read this BEFORE §5.
 *     6. §5 dft_naive — THE function in this file.  Every other line
 *        of code exists to set it up, run it, or visualise its output.
 *     7. §7..§9 scene — how the animation wires the algorithm into
 *        a live ncurses demo.
 *     8. §10..§15 draw + hud — rendering plumbing.
 *     9. §16 app — main loop + signal handlers + key dispatch.
 *
 *   Reader who already knows what a DFT is, just wants the code:
 *     §1 config, §3 complex, §5 dft_naive, §8 scene_tick.  The rest
 *     is ncurses scaffolding.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     output_bin_index, k         the loop counter for output bin k
 *     input_sample_index, n       the loop counter for input sample n
 *     twist_angle_radians         the angle inside the DFT's exp(-i*theta)
 *     twist                       exp(i * twist_angle) — unit-circle complex
 *     twisted                     x[n] * twist (one rotated arrow)
 *     running_sum                 the DFT bin's accumulator
 *     g_input_signal              N real samples (the "what we measured")
 *     g_dft_spectrum              N complex outputs (the DFT result)
 *     g_magnitude                 |X[k]| for k = 0..N/2 (one-sided)
 *     g_animation_phase_radians   the AUTO-SWEEP phase (NOT the DFT phase)
 *
 *   Inside tight loops where context is one line away, single-letter
 *   counters (`i`, `j`, `k`, `n`) stay short.
 *
 * Background you NEED
 *   - Basic complex arithmetic at the level of "z = a + bi, |z| =
 *     sqrt(a^2 + b^2), exp(i*theta) = cos + i*sin".  §3 reviews the
 *     handful of operations we use; T2 covers the geometry.
 *   - High-school trigonometry (cos, sin as projections of a unit-
 *     circle point).
 *
 * Background you do NOT need
 *   - The Fast Fourier Transform.  This file uses the textbook
 *     O(N^2) DFT.  signal/fft_helloworld.c is the next step up.
 *   - Continuous calculus.  Everything is a finite sum, indexed by
 *     integers.  No integrals appear anywhere.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * What is the DFT, in one sentence?
 *   Take N evenly-spaced samples of any signal, get back N "bin
 *   strengths" — each bin tells you how much of one specific
 *   frequency was present in the signal.
 *
 * Algorithm
 *   Direct evaluation of the DFT formula by two nested for-loops.
 *   The outer loop iterates over OUTPUT bins (k = 0..N-1); the
 *   inner loop iterates over INPUT samples (n = 0..N-1).  Each
 *   inner iteration computes one cos and one sin for the angle
 *   -2*pi*k*n/N, multiplies by the input sample, and adds to a
 *   running complex sum.  The sum at the end of the inner loop IS
 *   the bin's value.
 *
 *   Computed every frame so the spectrum panel is always live.
 *   No optimisations — this is the slowest possible DFT, written
 *   to read 1:1 with the math.  See signal/fft_helloworld.c for
 *   the same answer in O(N log N) time.
 *
 * Math
 *   The forward DFT formula:
 *
 *     X[k] = sum from n=0 to N-1 of  x[n] * exp(-i * 2*pi * k * n / N)
 *
 *   Decompose the complex exponential via Euler:
 *
 *     exp(-i * theta) = cos(theta) - i * sin(theta)
 *
 *   For real-valued input x[n] (always the case here), each X[k]
 *   decomposes into separate real and imaginary sums.  The §3
 *   ComplexNumber abstraction packages this so the code reads as
 *   one line per math step instead of one line per real/imaginary
 *   half-step.
 *
 *   Output structure for our test signal:
 *     If x[n] = cos(2*pi * k0 * n / N) (unit-amplitude cosine at
 *     bin k0), then |X[k0]| = N/2 (twin spike), |X[N - k0]| = N/2,
 *     and every other bin is zero (within float noise).  The twin
 *     spike comes from the conjugate symmetry of REAL inputs:
 *     X[N - k] = conjugate(X[k]).  Our visible spectrum panel
 *     shows only bins 0..N/2 (the unique half).
 *
 * Performance / boundaries
 *   - Cost per frame: O(N^2).  N = 32 → 1024 multiplies/frame at
 *     30 fps = 30k/sec.  Negligible at this size.
 *   - At N = 1024 the DFT becomes measurably slow at 30 fps.  At
 *     N = 65536 it's infeasible — that is where the FFT becomes
 *     essential, not optional (see fft_helloworld.c, T8).
 *   - All math is single-precision float; "should-be-zero" bins
 *     land around 1e-6, not exactly zero.  Use double for tighter
 *     precision.
 *   - frequency_bin can be FRACTIONAL.  Off-integer frequencies
 *     leak into neighbouring bins (a "hill" instead of a clean
 *     spike).  See T8 for why and signal/fft_vis.c for the cure
 *     (window functions).
 *
 * Rendering
 *   Two stacked vertical-bar panels (input waveform on top, magnitude
 *   spectrum on bottom) plus HUD.  Both panels auto-scale each frame
 *   to their own peak, so the visual is always full-height regardless
 *   of input amplitude.
 *
 *   Two optional debug overlays (toggled by 'd' and 'D') add a phase
 *   panel and a numeric bin-info table — both reveal information
 *   that magnitude alone discards.
 *
 * References
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapter 8.  Free online; the gentlest
 *     DFT introduction I know of.
 *   Brigham, E. O. (1988), "The Fast Fourier Transform and Its
 *     Applications".  The standard practitioner's reference.
 *   3Blue1Brown, "But what is the Fourier Transform? A visual
 *     introduction", YouTube — the animated intuition warm-up.
 *   https://en.wikipedia.org/wiki/Discrete_Fourier_transform
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   The DFT is a bank of N "lock-in detectors", one per integer
 *   frequency.  Each detector returns a complex number whose
 *   magnitude tells you "how much of this frequency is in the input".
 *   A pure tone lights up exactly one detector (and its conjugate
 *   twin); everything else stays dark.
 *
 *   This program animates the test input by sweeping the tone's
 *   frequency.  As the tone moves, the lit detector moves with it —
 *   you see the spike walk left and right across the spectrum panel.
 *   That walking spike is the simplest possible animated proof that
 *   the DFT is a "find me the frequency" operation.
 *
 * HOW THE DETECTOR WORKS — THE "TWIST AND SUM" PICTURE
 *   Inside the inner loop, each input sample x[n] is multiplied by
 *   a UNIT-CIRCLE complex number called the "twist":
 *
 *       twist = exp(-i * 2*pi * k * n / N)
 *             = cos(angle) - i * sin(angle)
 *
 *   That factor is a pure ROTATION in the complex plane — it spins
 *   x[n] backward by an angle that depends on n and k.  Picture each
 *   sample as a vector of length |x[n]|, all initially pointing along
 *   the real axis (since x[n] is a real number, no imaginary part).
 *   After multiplying by the twist, each sample's vector has been
 *   rotated to a NEW angle.  We then sum all N rotated vectors,
 *   head to tail.
 *
 *   When the input frequency MATCHES the detector's k, the rotation
 *   exactly UNDOES the input's natural spinning, and all the rotated
 *   vectors point the same direction.  Their sum is large.
 *
 *   When the input frequency does NOT match, the rotated vectors
 *   point every-which-way and cancel.  The sum is ~0.
 *
 *   That is the entire intuition.  The DFT is a bank of these
 *   detectors, one for each integer k.
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │   MATCHED (input freq = detector k):                   │
 *      │     all rotated vectors align  →  large sum            │
 *      │                                                        │
 *      │     →→→→→→→→→  (all pointing same way)                 │
 *      │                                                        │
 *      │   MISMATCHED (input freq ≠ detector k):                │
 *      │     rotated vectors point everywhere  →  sum cancels   │
 *      │                                                        │
 *      │     ↗  ↘  ←  ↑  ↓  →  ↖  ↘                            │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * EULER'S FORMULA (THE BRIDGE BETWEEN ANGLES AND COMPLEX NUMBERS)
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │    imag                                                │
 *      │     ▲                                                  │
 *      │     │   exp(i * theta)                                 │
 *      │     │      ●  on unit circle                           │
 *      │     │     /                                            │
 *      │     │    /                                             │
 *      │     │   /  theta                                       │
 *      │     │  /                                               │
 *      │     │ /                                                │
 *      │     +──────────────────────►  real                     │
 *      │                                                        │
 *      │   exp(i * theta) = cos(theta) + i * sin(theta)         │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (per frame)
 *   1. (If auto-sweep) advance the sweep phase, recompute g_freq_bin.
 *   2. Generate the cosine at g_freq_bin into g_input_signal[N].
 *   3. Run compute_dft_naive() — the textbook nested loop.
 *   4. Compute |X[k]| for k = 0..N/2 (one-sided magnitude spectrum).
 *   5. Render: input panel, spectrum panel, optional phase overlay,
 *      optional bin-table overlay, HUD top-right, hint bottom-left.
 *
 * KEY FORMULAS
 *
 *   The input we feed:
 *     x[n] = cos(2 * pi * frequency_bin * n / N)
 *
 *   The DFT we compute:
 *     X[k] = sum_{n=0..N-1}  x[n] * exp(-i * 2*pi * k * n / N)
 *
 *   For real input, output is conjugate-symmetric:
 *     X[N - k] = conjugate(X[k])
 *     |X[N - k]| = |X[k]|       ← twin spikes for any pure tone
 *
 *   Magnitude spectrum:
 *     |X[k]| = sqrt(re^2 + im^2)
 *
 *   Auto-sweep frequency (smooth there-and-back motion via sin):
 *     animation_phase += 2*pi / SWEEP_PERIOD_FRAMES
 *     g_freq_bin = FREQ_LO + (sin(animation_phase) + 1) / 2
 *                          * (FREQ_HI - FREQ_LO)
 *
 * EDGE CASES TO WATCH
 *   - Single-precision noise: bins that should be zero land around
 *     1e-6, not exactly zero.  The 'D' overlay's amplitude column
 *     shows this; use double if you need tighter precision.
 *   - Off-integer frequencies cause spectral leakage — the spike
 *     spreads across nearby bins.  This is the DFT's "your N samples
 *     don't fit an integer number of cycles" misery; window functions
 *     cure it (signal/fft_vis.c demonstrates).
 *   - Time-domain phase shifts leave |X[k]| INVARIANT.  Press 'd'
 *     and watch — magnitude bars stay still while phase bars
 *     respond to the shifting wave.  This is what magnitude throws
 *     away.
 *   - The DC bin X[0] is the sum of all samples.  For a pure cosine
 *     X[0] is zero (cos integrates to zero over an integer number
 *     of cycles).  If you change the input to add a constant offset
 *     you'd see X[0] light up.
 *
 * HOW TO VERIFY
 *   - Press 'a' to enter manual mode, then '+' until freq = 5.  The
 *     spike should land at bin 5.  Magnitude should be near N/2 = 16.
 *   - Press '+' more — spike walks rightward one bin per press.
 *   - In auto-sweep, the spike walks back and forth between bin 1
 *     and bin N/2 - 1 over ~3 seconds.
 *   - Hit ',' to nudge to freq = 4.9.  The spike spreads across
 *     bins 4..6 — spectral leakage in action.
 *   - Press 'd' for the phase panel.  In auto-sweep the magnitude
 *     bars are stationary while the phase bars wiggle — magnitude
 *     is invariant under time-shift; phase is not.
 *   - Press 'D' for the numeric bin table.  Top row should be the
 *     spike's bin with magnitude ~16; row 2 should be the conjugate
 *     twin at bin (N - spike's bin).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What problem does the DFT solve?  (decomposition into tones)
 *   T2   Complex numbers as 2-D rotations (Euler's formula)
 *   T3   The DFT formula, term by term
 *   T4   The "twist and sum" picture — why it works geometrically
 *   T5   A worked example — compute X[1] for N=4 by hand
 *   T6   Reading the output — magnitude and phase
 *   T7   Twin spikes for real input — conjugate symmetry
 *   T8   Spectral leakage — what happens at fractional bins
 *   T9   Cost analysis: O(N^2) and where it stops being acceptable
 *   T10  The animated demo — sweep, manual, leakage in motion
 *   T11  Pixel space vs cell space (terminal rendering)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT PROBLEM DOES THE DFT SOLVE?
 * ────────────────────────────────────
 * Suppose someone hands you a list of N numbers — say, microphone
 * samples, or pixel intensities along a line, or stock prices for
 * 32 trading days.  You want to know: WHAT FREQUENCIES are present?
 * Is there a repeating pattern every 8 samples?  Every 11?  Both?
 *
 * The DFT answers this question.  It takes N input samples and
 * returns N "bin strengths", one per integer frequency.  Bin 0 is
 * the DC (constant) component.  Bin 1 represents "completes 1 cycle
 * over the N-sample window".  Bin 2 is "2 cycles".  Bin k is "k
 * cycles".  The strength of bin k tells you how much of that
 * frequency is actually in your input.
 *
 * For a pure cosine at bin k0, only bins k0 and (N - k0) light up
 * (T7 explains the twin spikes); every other bin is essentially
 * zero.  For a sum of two pure cosines, two pairs of bins light up.
 * For a square wave, many bins light up at the harmonics of the
 * fundamental.  For random noise, every bin glows weakly with no
 * particular pattern.
 *
 * Real-world use cases for this single algorithm:
 *   - Audio analysis (spectrum analysers, music tuners, MP3
 *     compression).
 *   - Image compression (JPEG uses a related transform, the DCT).
 *   - Radar (find the doppler shift of a moving target).
 *   - Solving partial differential equations (heat conduction,
 *     wave propagation).
 *   - Optical microscopy (Fourier ptychography).
 *
 * Same one algorithm — just applied to different signals.
 *
 * T2  COMPLEX NUMBERS AS 2-D ROTATIONS (EULER'S FORMULA)
 * ──────────────────────────────────────────────────────
 * A complex number z = a + bi can be thought of either algebraically
 * (real part `a`, imaginary part `b`) or as a 2-D point / arrow at
 * coordinates (a, b).  The two views agree on every operation.
 *
 * Three operations are central to the DFT:
 *
 *   1. ADDITION.  (a + bi) + (c + di) = (a+c) + (b+d)i.  Geometrically
 *      this is just head-to-tail vector addition.
 *
 *   2. MULTIPLY by a real scalar.  s * (a + bi) = (s*a) + (s*b)i.
 *      Geometrically: stretch the arrow uniformly without rotating.
 *
 *   3. EULER'S FORMULA.  exp(i * theta) = cos(theta) + i * sin(theta).
 *      The result has magnitude 1 and angle theta — a point on the
 *      UNIT CIRCLE.
 *
 * Multiplying any complex z by exp(i * theta) ROTATES z by theta
 * radians counter-clockwise without changing its length.  This is
 * the deep reason the DFT works: the "twist" factor inside the
 * formula is exactly such a unit-circle complex, and multiplying by
 * it rotates each input sample by an angle determined by the bin k
 * and the sample index n.
 *
 * In code (§3) we use a 2-float `ComplexNumber` struct and a handful
 * of named helpers (complex_add, complex_scale_by_real,
 * complex_from_angle, complex_magnitude).  The DFT body (§5) reads
 * as ONE named operation per math step instead of two real/imaginary
 * lines of float arithmetic.
 *
 * T3  THE DFT FORMULA, TERM BY TERM
 * ─────────────────────────────────
 * Here's the formula:
 *
 *     X[k] = sum_{n=0..N-1}  x[n] * exp(-i * 2*pi * k * n / N)
 *
 * Read each piece:
 *
 *   sum_{n=0..N-1}        we compute this once per output bin k.
 *                         Inside, we walk n from 0 to N-1.
 *
 *   x[n]                  the n-th input sample (real number).
 *
 *   exp(-i * angle)       a unit-circle complex by Euler.  The
 *                         negative sign in -i * 2pi*k*n/N makes it
 *                         a CLOCKWISE rotation (the FORWARD DFT
 *                         convention; the inverse DFT uses +i).
 *
 *   2 * pi * k * n / N    the angle.  For k = 0 the angle is always
 *                         0 (no twist; sum equals sum of x).  For
 *                         k = 1, as n walks 0..N-1 the angle walks
 *                         one full revolution.  For k = 2, two full
 *                         revolutions.  Etc.
 *
 *   x[n] * exp(-i*angle)  the "twisted" sample — x[n] rotated by
 *                         the angle.  Since x[n] is real, this is
 *                         a complex_scale_by_real(x[n], twist).
 *
 *   sum                   add up all the twisted samples.  When they
 *                         align (matched freq), the sum is large.
 *                         When they don't (mismatched), they cancel.
 *
 * The result X[k] is a complex number.  Its magnitude tells you "how
 * much of frequency k is in the input"; its argument (atan2 of imag
 * over real) tells you "where in the cycle the frequency starts".
 * The spectrum panel plots magnitude.  The 'd' debug overlay plots
 * argument.
 *
 * T4  THE "TWIST AND SUM" PICTURE — WHY IT WORKS GEOMETRICALLY
 * ────────────────────────────────────────────────────────────
 * Set aside the formula for a moment.  Picture this:
 *
 *   - You have N arrows in the plane.  Each starts pointing along
 *     the +x axis (the real axis).  The lengths are signed, so
 *     "length -1" actually points along -x.  These are your input
 *     samples x[n].
 *
 *   - You pick a bin k.  You want to compute X[k].
 *
 *   - For each n, you ROTATE the n-th arrow by the angle
 *     -2*pi*k*n/N.  As n goes 0..N-1, that angle walks k full
 *     revolutions clockwise.  Different arrows get rotated by
 *     different amounts.
 *
 *   - You ADD all N rotated arrows head-to-tail.  The final tip
 *     position is X[k] — a complex number.
 *
 * Now the punchline: WHEN does the sum of those rotated arrows end
 * up far from the origin (large |X[k]|)?
 *
 *   - WHEN the input itself was a cosine at frequency k0, AND k = k0,
 *     the rotation EXACTLY UNDOES the input's natural rotation.  All
 *     N rotated arrows end up pointing the same direction.  Their
 *     sum stretches that direction N/2 units (the factor of 2 comes
 *     from the cosine being half the sum of two opposite rotations
 *     — see T7).
 *
 *   - WHEN k != k0, the arrows get rotated by amounts that don't
 *     match the input's pattern.  They point all over the place.
 *     They cancel.  Sum is ~0.
 *
 * THIS is why the spectrum spike sits exactly at the input's
 * frequency — the geometric construction is engineered to align
 * arrows for the matched bin and scatter them for everyone else.
 *
 * T5  A WORKED EXAMPLE — COMPUTE X[1] FOR N=4 BY HAND
 * ───────────────────────────────────────────────────
 * Take N = 4 and a cosine at bin 1 over 4 samples:
 *
 *     n:        0      1      2      3
 *     x[n]:  +1.0    0.0   -1.0    0.0
 *
 *   (cos(0) = 1, cos(pi/2) = 0, cos(pi) = -1, cos(3pi/2) = 0.)
 *
 * Now compute X[1].  The twist angle for each n is -2*pi * 1 * n / 4
 * = -pi*n/2.
 *
 *     n = 0:  twist = exp(0)        = 1 + 0i
 *             contribution = 1 * (1 + 0i)   = 1 + 0i
 *
 *     n = 1:  twist = exp(-i*pi/2)  = 0 - 1i
 *             contribution = 0 * (0 - 1i)   = 0 + 0i
 *
 *     n = 2:  twist = exp(-i*pi)    = -1 + 0i
 *             contribution = -1 * (-1 + 0i) = 1 + 0i
 *
 *     n = 3:  twist = exp(-i*3pi/2) = 0 + 1i
 *             contribution = 0 * (0 + 1i)   = 0 + 0i
 *
 *     X[1] = (1+0i) + 0 + (1+0i) + 0 = 2 + 0i
 *     |X[1]| = 2 = N/2.   ✓
 *
 * Now do X[2]:
 *
 *     n = 0:  twist = exp(0)        = 1 + 0i
 *             contribution = 1 * 1            = 1
 *     n = 1:  contribution = 0.
 *     n = 2:  twist = exp(-i*2pi)   = 1 + 0i
 *             contribution = -1 * 1            = -1
 *     n = 3:  contribution = 0.
 *
 *     X[2] = 1 + 0 + (-1) + 0 = 0.   ✓ (no energy at bin 2)
 *
 * The cancellation in X[2] is the "MISMATCHED" case from the
 * MENTAL MODEL diagram — the twisted contributions point opposite
 * directions and cancel.
 *
 * Now do X[3] (which should equal conjugate(X[1]) by T7):
 *
 *     n = 0:  twist = 1 + 0i;  contribution = 1
 *     n = 1:  contribution = 0
 *     n = 2:  twist = exp(-i*3pi)  = -1 + 0i;  contribution = 1
 *     n = 3:  contribution = 0
 *
 *     X[3] = 1 + 0 + 1 + 0 = 2 + 0i.   |X[3]| = 2.
 *
 * X[1] = 2 + 0i, X[3] = 2 + 0i — they're complex conjugates of each
 * other (conjugate of a real number is itself).  This is the twin
 * spike.  T7 explains why this happens for all real inputs.
 *
 * T6  READING THE OUTPUT — MAGNITUDE AND PHASE
 * ────────────────────────────────────────────
 * Each X[k] is a complex number — two floats.  Two scalar views:
 *
 *     magnitude_k = |X[k]|       = sqrt(re^2 + im^2)     "loudness"
 *     phase_k     = arg(X[k])    = atan2(im, re)         "where in cycle"
 *
 * The spectrum panel of this program (and almost every audio
 * spectrum analyser you'll ever see) plots magnitude only.  This has
 * a beautiful invariance: SHIFTING A SIGNAL IN TIME LEAVES THE
 * MAGNITUDE SPECTRUM UNCHANGED.  Press 'd' to flip on the phase
 * panel — in auto-sweep, the magnitude bars are completely
 * stationary while the phase bars wiggle every frame.  THAT WIGGLE
 * is the time-shift information that magnitude throws away.
 *
 * Phase is NOT junk, however.  To RECONSTRUCT a signal from its
 * spectrum (inverse DFT) you need both magnitude AND phase.  Audio
 * compression (MP3) keeps both, with very clever compression of the
 * phase data.  Throwing phase away is a one-way operation — fine
 * for visualisation, fatal for round-trip reconstruction.
 *
 * T7  TWIN SPIKES FOR REAL INPUT — CONJUGATE SYMMETRY
 * ───────────────────────────────────────────────────
 * For any REAL-valued input (no imaginary part, which is always the
 * case in this file), the DFT output is CONJUGATE-SYMMETRIC:
 *
 *     X[N - k] = conjugate(X[k])
 *
 * which immediately implies:
 *
 *     |X[N - k]| = |X[k]|
 *
 * So a pure cosine produces TWO equally-tall spikes — at bin k0
 * (the "positive frequency") AND at bin N - k0 (the "negative
 * frequency").  The worked example in T5 made this concrete:
 * x[n] = cos(2*pi*1*n/4) gave |X[1]| = |X[3]| = 2 at N = 4.
 *
 * Why does this happen?  The cosine identity says:
 *
 *     cos(theta) = (exp(+i*theta) + exp(-i*theta)) / 2
 *
 * That is, every cosine at angular frequency theta is the AVERAGE of
 * two complex exponentials — one rotating CCW (positive frequency),
 * one rotating CW (negative frequency).  The DFT bin at "negative
 * frequency -k" lives at index N - k by the periodicity of complex
 * exponentials.  So the two halves of the cosine show up at bins
 * k and N - k, each with magnitude N/2.
 *
 * The visible spectrum panel SHOWS only bins 0..N/2.  We don't show
 * the upper half because it carries no new information — it's just
 * the mirror.  When you see a spike at bin k0 in the panel, there's
 * an identical spike at bin N - k0 that we deliberately hide.
 *
 * The 'D' debug overlay reveals this clearly: select an integer freq,
 * toggle 'D', and the top two rows of the table will be your bin
 * and the (hidden) twin.  Their magnitudes will be identical.
 *
 * T8  SPECTRAL LEAKAGE — WHAT HAPPENS AT FRACTIONAL BINS
 * ──────────────────────────────────────────────────────
 * The DFT is built on the fiction that the N input samples are ONE
 * PERIOD of an infinitely-repeating signal.  If the actual frequency
 * fits an integer number of cycles in N samples (e.g. freq = 4 in
 * a buffer of N = 32 samples — exactly 4 cycles), the fiction holds:
 * the sample at n = 0 smoothly continues into n = N, the periodic
 * extension is continuous, and the DFT shows a clean spike at bin 4.
 *
 * If the frequency is, say, 4.5, the fiction's seam at n = N has a
 * sudden jump.  That discontinuity contains LOTS of high-frequency
 * components, so the energy of the original tone "leaks" across many
 * bins — bins 3, 4, 5, 6, 7, ... all light up partially in a sin-x/x
 * pattern, instead of just bin 4 alone.
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │  freq = 4.0  (integer)        freq = 4.5  (fractional) │
 *      │                                                        │
 *      │   |                              |                     │
 *      │   |                              | |                   │
 *      │   |                            | | | |                 │
 *      │  _|________________            | | | | |  ___          │
 *      │  3 4 5 6 7  bins               2 3 4 5 6 7 8  bins     │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * Try it live:
 *   - Press 'a' to enter manual mode.
 *   - Press '+' four times — freq = 5, single clean spike.
 *   - Press ',' five times — freq = 4.5, the spike is now a wide
 *     hill spread across 4 bins.
 *
 * The cure is WINDOW FUNCTIONS — you multiply the input by a smooth
 * taper that goes to zero at both ends, so the periodic extension
 * has no discontinuity.  This is what audio FFT analysers do all the
 * time.  signal/fft_vis.c demonstrates four window functions live.
 *
 * T9  COST ANALYSIS: O(N^2) AND WHERE IT STOPS BEING ACCEPTABLE
 * ─────────────────────────────────────────────────────────────
 * Per output bin: N inner iterations, each calling cos and sin once
 * plus doing a complex add/scale.  Total per DFT: N outer × N inner =
 * N^2 inner iterations.  Plus 2 * N^2 trig calls.
 *
 * Plug in some N values and per-frame costs at 30 fps:
 *
 *     N      ops/frame          ops/sec      Verdict
 *     ----   -----------        --------     -------
 *     8      64                 1,920        instant
 *     32     1,024              30,720       this demo
 *     128    16,384             491,520      still instant
 *     1024   ~1,000,000         ~30,000,000  measurable lag
 *     65536  ~4,300,000,000     128 G/sec    infeasible
 *
 * The Fast Fourier Transform (FFT) computes the SAME spectrum in
 * O(N log N) instead of O(N^2).  At N = 65536 that's ~1 million
 * operations per frame instead of 4.3 BILLION — about 4000x faster.
 *
 * For this hello-world demo, N = 32 and the naive DFT is more than
 * adequate.  When you outgrow it, signal/fft_helloworld.c is the
 * next file to read.
 *
 * T10  THE ANIMATED DEMO — SWEEP, MANUAL, LEAKAGE IN MOTION
 * ─────────────────────────────────────────────────────────
 * The demo loop has two modes:
 *
 *   AUTO-SWEEP (default).  An internal sweep phase advances by
 *   2*pi / SWEEP_PERIOD_FRAMES each frame.  The frequency oscillates
 *   smoothly between FREQ_LO (= 1) and FREQ_HI (= N/2 - 1) following
 *   sin(sweep_phase).  Spike walks across the spectrum panel; wave
 *   shape morphs accordingly.
 *
 *   MANUAL ('a' to toggle).  The sweep is frozen.  '+' / '-' steps
 *   the frequency by 1.0 bin (lands on integer bins; clean spike).
 *   ',' / '.' steps by 0.1 bin (lands on fractional bins; the spike
 *   spreads — see T8 leakage demo).
 *
 * The animation phase (the SWEEP's own phase, not the DFT's phase
 * — we have two unrelated "phases" in this file, named distinctly to
 * avoid confusion) advances every frame; reset with 'r'.
 *
 * When paused, scene_tick is a no-op — the spectrum panel stays
 * frozen on whatever was last computed.  Useful for the 'D' table
 * when you want to inspect specific bin values without them moving.
 *
 * T11  PIXEL SPACE VS CELL SPACE  (terminal rendering)
 * ────────────────────────────────────────────────────
 * The screen has cells (rows × cols).  In a typical monospace font,
 * cells are about TWICE AS TALL as wide.  For this hello-world demo
 * we never draw circles, so we stay in CELL space throughout.  But
 * for orientation:
 *
 *   - The TIME panel maps one input sample n to one column of cells.
 *     The sample value sets the height of a vertical bar from the
 *     panel's midline.  Positive samples grow upward as '|',
 *     negative grow downward as '.'.
 *
 *   - The FREQ panel maps one bin k to one or two columns (depending
 *     on terminal width).  The magnitude is the height of a vertical
 *     bar from the panel's bottom edge.
 *
 *   - The (optional) PHASE panel below FREQ does the same with phase
 *     instead of magnitude, except phase can be negative so bars grow
 *     up (magenta) or down (sky blue) from a midline.
 *
 * Per-frame layout (computed in §15 render_frame):
 *
 *     row 0           HUD top-right + PAUSED chip (when paused)
 *     row 1           panel labels
 *     rows 2..mid     TIME panel (signal waveform)
 *     row mid+1       FREQ panel label
 *     rows ..bot      FREQ panel (magnitude spectrum)
 *     rows ..         PHASE panel if 'd' is on
 *     row LINES-1     bottom-left HINT
 *
 * The PHASE panel "steals" rows from the FREQ panel when toggled on,
 * so the overall layout always fits inside the screen.
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
/* §1  config — every constant + enum lives here                         */
/* ===================================================================== */

#define N 32           /* number of samples                */
#define N_HALF (N / 2) /* bins shown on the spectrum panel */
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 90 /* one full sweep ≈ 3 seconds       */
#define FREQ_LO 1.0f           /* sweep / clamp lower bound         */
#define FREQ_HI ((float)N * 0.5f - 1.0f) /* upper bound     */
#define ARM_TABLE_ROWS 8                 /* 'D' overlay: top-N bins          */

/* Colour pair IDs.  HUD / HINT / PHASE pairs are theme-invariant per
 * CLAUDE.md HUD Standard. */
enum {
  PAIR_SIG = 1,       /* time-domain wave bars                      */
  PAIR_SPEC = 2,      /* spectrum bars (medium magnitude)           */
  PAIR_SPIKE = 3,     /* spectrum bar at the dominant bin           */
  PAIR_LABEL = 4,     /* panel labels                               */
  PAIR_HUD = 5,       /* HUD top status (bright yellow + bold)      */
  PAIR_HINT = 6,      /* bottom hint  (bright cyan + bold)          */
  PAIR_PHASE_POS = 7, /* phase panel positive bars (magenta-pink)   */
  PAIR_PHASE_NEG = 8, /* phase panel negative bars (sky blue)       */
};

/* ===================================================================== */
/* §2  clock — monotonic ns timer + nanosecond sleep                     */
/* ===================================================================== */

static long long clock_now_ns(void) {
  /* Purpose : monotonic clock in ns.  Used by the main loop to
   *           figure out how long to sleep before the next frame. */
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  /* Block for `ns` nanoseconds.  Without this the main loop would
   * spin a CPU at 100% just to redraw 30 times a second. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  complex — the ComplexNumber abstraction                           */
/* ===================================================================== */
/*
 *  WHY THIS EXISTS
 *  ───────────────
 *  Without it, every complex operation in the DFT inner loop would be
 *  TWO lines of C — one for the real part, one for the imaginary.
 *  That makes the DFT noisy and the algebra hard to see:
 *
 *      // Without the abstraction:
 *      float twisted_real = sample * cosf(angle);
 *      float twisted_imag = sample * sinf(angle);
 *      running_sum_real  += twisted_real;
 *      running_sum_imag  += twisted_imag;
 *
 *      // With the abstraction:
 *      ComplexNumber twist   = complex_from_angle(angle);
 *      ComplexNumber twisted = complex_scale_by_real(sample, twist);
 *      running_sum = complex_add(running_sum, twisted);
 *
 *  Same arithmetic, half the syntactic noise, and now every line
 *  reads like English: "the twist at this angle, scaled by the
 *  sample, added to the running sum".
 *
 *  Why not <complex.h>?  The standard library has `float complex`
 *  and uses operators directly, but the type and operators feel like
 *  compiler magic.  A 2-float struct with named helpers is more
 *  honest about what's happening, prints cleanly in a debugger, and
 *  is portable to any C11 compiler.  The six names below are the
 *  entire complex-number library this file needs.
 *
 *  fft_helloworld.c §3 has a slightly LARGER toolkit (adds
 *  complex_sub, complex_mul, complex_one) because the FFT butterfly
 *  multiplies two complex numbers.  We don't need those here.
 */

typedef struct {
  float re; /* real part      (the "x" coordinate)  */
  float im; /* imaginary part (the "y" coordinate)  */
} ComplexNumber;

/* Identity: the complex number 0 + 0i.  Initial value for running
 * sums (the DFT accumulator starts here). */
static const ComplexNumber complex_zero = {0.0f, 0.0f};

static inline ComplexNumber complex_make(float real_part, float imag_part) {
  return (ComplexNumber){real_part, imag_part};
}

/*
 * complex_from_angle — Euler's formula:  exp(i * theta) = cos + i * sin.
 *   Result always sits ON the unit circle (magnitude 1).  As
 *   `angle_radians` walks 0 -> 2*pi the returned complex walks once
 *   around the unit circle counter-clockwise.  This is THE bridge
 *   between an angle and a complex multiplication.
 */
static inline ComplexNumber complex_from_angle(float angle_radians) {
  return complex_make(cosf(angle_radians), sinf(angle_radians));
}

/*
 * complex_add — vector addition (head-to-tail).
 */
static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re + b.re, a.im + b.im);
}

/*
 * complex_scale_by_real — multiply by a real scalar (uniform stretch
 *   without rotation).  The DFT's "twisted = x[n] * twist" step:
 *   x[n] is real, twist is unit-circle complex, twisted is the
 *   rotated arrow scaled by x[n]'s magnitude.
 */
static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z) {
  return complex_make(scale * z.re, scale * z.im);
}

/*
 * complex_magnitude — length of the arrow:  |z| = sqrt(re^2 + im^2).
 *   Used by the spectrum panel ("loudness" of each bin) and by the
 *   'D' table (numeric magnitudes).  Phase is discarded.
 */
static inline float complex_magnitude(ComplexNumber z) {
  return sqrtf(z.re * z.re + z.im * z.im);
}

/* That's six names.  No complex_sub, no complex_mul — the DFT never
 * subtracts or multiplies two complex numbers (only x[n] × twist,
 * which is real × complex = scale_by_real).  When the FFT version
 * needs a fuller library, it lives in fft_helloworld.c. */

/* ===================================================================== */
/* §4  colors — palette pair init                                        */
/* ===================================================================== */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_SIG, 51, -1);        /* bright cyan    */
    init_pair(PAIR_SPEC, 154, -1);      /* yellow-green   */
    init_pair(PAIR_SPIKE, 46, -1);      /* bright green   */
    init_pair(PAIR_LABEL, 244, -1);     /* mid grey       */
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow  */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan    */
    init_pair(PAIR_PHASE_POS, 213, -1); /* magenta-pink   */
    init_pair(PAIR_PHASE_NEG, 117, -1); /* sky blue       */
  } else {
    init_pair(PAIR_SIG, COLOR_CYAN, -1);
    init_pair(PAIR_SPEC, COLOR_GREEN, -1);
    init_pair(PAIR_SPIKE, COLOR_GREEN, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_PHASE_POS, COLOR_MAGENTA, -1);
    init_pair(PAIR_PHASE_NEG, COLOR_BLUE, -1);
  }
}

/* ===================================================================== */
/* §5  dft_naive — the textbook O(N^2) DFT (THE function in this file)   */
/* ===================================================================== */
/*
 *  Purpose         : compute the discrete Fourier transform exactly
 *                    by direct evaluation of the formula.  Every
 *                    other line in this file exists to set this up
 *                    or visualise its output.
 *
 *  Pseudocode      :
 *      for each output bin k in [0, N):
 *        running_sum = 0 + 0i
 *        for each input sample n in [0, N):
 *          twist_angle = -2*pi * k * n / N
 *          twist       = exp(i * twist_angle)            ← unit circle
 *          twisted     = x[n] * twist                    ← real × complex
 *          running_sum = running_sum + twisted
 *        output[k]   = running_sum
 *
 *  Mental model    : N parallel "lock-in detectors", each tuned to
 *                    one integer harmonic.  When the input contains
 *                    that harmonic, the twists undo its rotation and
 *                    the sum is large; otherwise the twisted samples
 *                    point every-which-way and cancel.
 *
 *                    See MENTAL MODEL block above for the full
 *                    "twist and sum" picture, including the
 *                    matched/mismatched diagram.
 *
 *  Inputs/outputs  : input_signal[N]    — real samples.
 *                    output_spectrum[N] — complex bins (caller-allocated).
 *
 *                    For our test signal (unit cosine at integer bin
 *                    k0): |X[k0]| = N/2, |X[N - k0]| = N/2 (twin
 *                    spike), all others ≈ 0.
 *
 *  Why it exists   : this IS the DFT, written as 1:1 with the math.
 *                    No twiddle recurrence, no FFT trickery.  Run
 *                    every frame so the spectrum panel is live.
 *
 *  Cost            : O(N^2) inner iterations, each with 2 trig calls
 *                    (one cos, one sin).  At N = 32 → 1024 iterations
 *                    per frame, totally negligible.  T9 covers what
 *                    happens at large N.
 */
static void compute_dft_naive(const float *input_signal,
                              ComplexNumber *output_spectrum) {
  /* Pseudocode line: "for each output bin k in [0, N)" */
  for (int output_bin_index = 0; output_bin_index < N; output_bin_index++) {

    /* Pseudocode line: "start with running_sum = 0 + 0i". */
    ComplexNumber running_sum = complex_zero;

    /* Pseudocode line: "for each input sample n in [0, N)" */
    for (int input_sample_index = 0; input_sample_index < N;
         input_sample_index++) {

      /* ── STEP 1 — compute the twist angle ──────────────────
       *
       * As n walks 0..N-1, this angle walks k full revolutions
       * BACKWARD around the unit circle (negative because this
       * is the FORWARD DFT — see T3).
       *   k = 0:  angle is always 0 (no twist; sum = sum of x).
       *   k = 1:  angle makes one full backward revolution.
       *   k = 2:  two revolutions.  Etc.
       */
      float twist_angle_radians = -2.0f * (float)M_PI *
                                  (float)output_bin_index *
                                  (float)input_sample_index / (float)N;

      /* ── STEP 2 — twist x[n] by that angle ────────────────
       *
       * The twist is a unit-circle complex (magnitude 1) at the
       * angle above — Euler's formula (§3 complex_from_angle).
       * Multiplying x[n] by it scales the unit-circle vector
       * by x[n]'s magnitude (signed), giving the rotated arrow.
       *
       * This is the only place we call cos/sin (inside §3).
       * Two trig calls per inner iteration × N^2 iterations is
       * what makes this O(N^2).
       */
      ComplexNumber twist = complex_from_angle(twist_angle_radians);
      ComplexNumber twisted =
          complex_scale_by_real(input_signal[input_sample_index], twist);

      /* ── STEP 3 — accumulate the twisted sample ───────────
       *
       * Add the rotated arrow to the running sum.  By the end
       * of the inner loop we have summed N rotated arrows;
       * that sum IS the bin's value.
       */
      running_sum = complex_add(running_sum, twisted);
    }

    /* Pseudocode line: "store running_sum as X[k]" */
    output_spectrum[output_bin_index] = running_sum;
  }
}

/* ===================================================================== */
/* §6  signal — generate the test input                                  */
/* ===================================================================== */
/*
 *  We feed the DFT a single pure cosine every frame:
 *
 *      x[n] = cos(2 * pi * frequency_bin * n / N)
 *
 *  This wave completes EXACTLY `frequency_bin` cycles over the N
 *  sample window:
 *
 *      frequency_bin = 1   → 1 cycle in N samples
 *      frequency_bin = 2   → 2 cycles in N samples
 *      frequency_bin = N/2 → fastest representable (Nyquist limit)
 *
 *  Auto-sweep changes frequency_bin smoothly; manual mode steps it
 *  in 0.1 / 1.0 increments.  The wave shape on screen morphs to
 *  match.
 *
 *  Boundary: frequency_bin can be FRACTIONAL.  Off-integer values
 *  cause spectral leakage — the spike spreads across nearby bins.
 *  T8 covers why; signal/fft_vis.c demonstrates the cure (window
 *  functions).
 */
static void generate_cosine(float frequency_bin, float *output_signal) {
  /* Pseudocode:
   *   for n in 0..N-1:
   *     phase = 2 * pi * frequency_bin * n / N
   *     output_signal[n] = cos(phase)
   */
  for (int sample_index = 0; sample_index < N; sample_index++) {
    float phase_radians =
        2.0f * (float)M_PI * frequency_bin * (float)sample_index / (float)N;
    output_signal[sample_index] = cosf(phase_radians);
  }
}

/* ===================================================================== */
/* §7  scene_state — globals + reset                                     */
/* ===================================================================== */

static float g_input_signal[N];         /* time-domain wave */
static ComplexNumber g_dft_spectrum[N]; /* X[k] (full N)    */
static float g_magnitude[N_HALF + 1];   /* |X[k]| (one-sided)*/
static float g_magnitude_peak = 1.0f;   /* for autoscale    */

static float g_freq_bin = FREQ_LO;
static bool g_simulation_paused = false;
static bool g_auto_sweep_enabled = true;
static float g_animation_phase_radians = 0.0f; /* sweep's own phase  */

/* Debug overlay toggles (see §13, §14). */
static bool g_show_phase_panel = false;
static bool g_show_arm_table = false;

static void scene_reset(void) {
  g_freq_bin = FREQ_LO;
  g_auto_sweep_enabled = true;
  g_animation_phase_radians = 0.0f;
  g_simulation_paused = false;
}

/* ===================================================================== */
/* §8  scene_tick — per-frame update orchestrator                        */
/* ===================================================================== */
/*
 *  Purpose : advance state by ONE frame.  Five numbered steps that
 *            match the ALGORITHM IN STEPS section of the MENTAL
 *            MODEL block.  No render work; that lives in §11..§15
 *            and only READS this state.
 *
 *  Steps:
 *    1. (auto-sweep) advance sweep phase, recompute g_freq_bin.
 *    2. Generate the cosine at g_freq_bin into g_input_signal[].
 *    3. Run compute_dft_naive() — fills g_dft_spectrum[].
 *    4. Compute |X[k]| for k = 0..N/2 + the running peak (autoscale).
 *
 *  All four steps run only when not paused; pause freezes the
 *  spectrum at its last computed state.
 */
static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* ── Step 1.  auto-sweep frequency ─────────────────────────── */
  if (g_auto_sweep_enabled) {
    g_animation_phase_radians +=
        2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
    if (g_animation_phase_radians > 2.0f * (float)M_PI)
      g_animation_phase_radians -= 2.0f * (float)M_PI;

    /* sin in [-1, +1] → unit interval → [FREQ_LO, FREQ_HI].
     * sin (not a sawtooth) gives smooth there-and-back motion. */
    float sin_normalised_to_unit =
        (sinf(g_animation_phase_radians) + 1.0f) * 0.5f;
    g_freq_bin = FREQ_LO + sin_normalised_to_unit * (FREQ_HI - FREQ_LO);
  }

  /* ── Step 2.  generate the test cosine ──────────────────────── */
  generate_cosine(g_freq_bin, g_input_signal);

  /* ── Step 3.  run the DFT (the heart of this file) ─────────── */
  compute_dft_naive(g_input_signal, g_dft_spectrum);

  /* ── Step 4.  one-sided magnitude spectrum + autoscale peak ── */
  g_magnitude_peak = 1e-6f; /* tiny non-zero to avoid divide-by-zero */
  for (int bin_index = 0; bin_index <= N_HALF; bin_index++) {
    g_magnitude[bin_index] = complex_magnitude(g_dft_spectrum[bin_index]);
    if (g_magnitude[bin_index] > g_magnitude_peak)
      g_magnitude_peak = g_magnitude[bin_index];
  }
}

/* ===================================================================== */
/* §9  scene_input — manual frequency adjustment                         */
/* ===================================================================== */

static void scene_adjust_freq(float delta_bins) {
  /* No-op while auto-sweep is on; otherwise clamp to [FREQ_LO, FREQ_HI]. */
  if (g_auto_sweep_enabled)
    return;
  g_freq_bin += delta_bins;
  if (g_freq_bin < FREQ_LO)
    g_freq_bin = FREQ_LO;
  if (g_freq_bin > FREQ_HI)
    g_freq_bin = FREQ_HI;
}

/* ===================================================================== */
/* §10  draw_bar — vertical-bar primitive                                */
/* ===================================================================== */

static void draw_bar(int column, int baseline_row, int bar_height_cells,
                     bool growing_upward, int colour_pair) {
  /* Stamp a vertical column of glyphs starting at baseline_row,
   * growing either upward or downward.  Used by every panel. */
  if (bar_height_cells <= 0)
    return;
  attron(COLOR_PAIR(colour_pair) | A_BOLD);
  for (int dy = 0; dy < bar_height_cells; dy++) {
    int row = growing_upward ? (baseline_row - dy) : (baseline_row + dy + 1);
    if (row < 0 || row >= LINES)
      continue;
    if (column < 0 || column >= COLS)
      continue;
    mvaddch(row, column, growing_upward ? '|' : '.');
  }
  attroff(COLOR_PAIR(colour_pair) | A_BOLD);
}

/* ===================================================================== */
/* §11  draw_signal — TIME panel renderer                                */
/* ===================================================================== */

static void draw_signal_panel(int top_row, int height_rows) {
  /* Per sample: vertical bar from the panel midline.  Positive
   * samples grow upward as '|', negative grow downward as '.'.
   * Signal is bounded by ±1 so we scale half-height directly. */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int columns_to_draw = (COLS < N) ? COLS : N;
  int spacing_per_sample = (COLS / N >= 2) ? 2 : 1;

  for (int sample_index = 0; sample_index < columns_to_draw; sample_index++) {

    float v = g_input_signal[sample_index];
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);

    for (int s = 0; s < spacing_per_sample; s++)
      draw_bar(sample_index * spacing_per_sample + s, midline_row, h, pos,
               PAIR_SIG);
  }
}

/* ===================================================================== */
/* §12  draw_spectrum — FREQ panel (magnitude) renderer                  */
/* ===================================================================== */

static void draw_spectrum_panel(int top_row, int height_rows,
                                int *out_dominant_bin) {
  /* Bins 0..N/2, each as an upward bar from the panel bottom.
   * Spike bin (the loudest one) is highlighted in bright green. */
  int baseline_row = top_row + height_rows - 1;
  int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
  int max_bins = COLS / bin_w;
  if (max_bins > N_HALF + 1)
    max_bins = N_HALF + 1;

  int dominant_bin = 0;
  for (int bin_index = 1; bin_index <= N_HALF; bin_index++)
    if (g_magnitude[bin_index] > g_magnitude[dominant_bin])
      dominant_bin = bin_index;
  *out_dominant_bin = dominant_bin;

  for (int bin_index = 0; bin_index < max_bins; bin_index++) {
    float v = g_magnitude[bin_index] / (g_magnitude_peak + 1e-6f);
    int h = (int)(v * (float)height_rows + 0.5f);
    int pair = (bin_index == dominant_bin) ? PAIR_SPIKE : PAIR_SPEC;

    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(bin_index * bin_w + bx, baseline_row, h, true, pair);
  }
}

/* ===================================================================== */
/* §13  draw_phase — debug 'd' phase panel                               */
/* ===================================================================== */
/*
 *  Show arg(X[k]) for each bin as a vertical bar from a midline.
 *  Positive phase grows up (magenta), negative grows down (sky blue).
 *  Magnitude alone discards phase — toggle this overlay to see what
 *  the magnitude panel throws away.
 */

static void draw_phase_panel(int top_row, int height_rows) {
  if (height_rows < 3)
    return;

  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
  int max_bins = COLS / bin_w;
  if (max_bins > N_HALF + 1)
    max_bins = N_HALF + 1;

  for (int bin_index = 0; bin_index < max_bins; bin_index++) {
    ComplexNumber z = g_dft_spectrum[bin_index];
    /* Skip phase for near-zero magnitude bins (noise). */
    if (complex_magnitude(z) < 1e-3f)
      continue;

    float phase_radians = atan2f(z.im, z.re);
    float fraction = phase_radians / (float)M_PI; /* in (-1, 1] */
    bool pos = (phase_radians >= 0.0f);
    int h = (int)(fabsf(fraction) * (float)half_height + 0.5f);

    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(bin_index * bin_w + bx, midline_row, h, pos,
               pos ? PAIR_PHASE_POS : PAIR_PHASE_NEG);
  }

  /* Midline marker so the user can see "zero phase". */
  attron(COLOR_PAIR(PAIR_LABEL));
  for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

/* ===================================================================== */
/* §14  draw_arm_table — debug 'D' top-N bin info table                  */
/* ===================================================================== */
/*
 *  Top-left numeric panel.  Sorts bins by descending magnitude and
 *  prints (bin, |X[k]|, arg(X[k])) for the first ARM_TABLE_ROWS.
 *  Useful for verifying decode by hand.
 */

static void draw_arm_table(void) {
  /* Selection-sort the first N/2 bin indices by descending
   * magnitude — only the top ARM_TABLE_ROWS positions matter. */
  int sorted_bin_indices[N_HALF + 1];
  for (int i = 0; i <= N_HALF; i++)
    sorted_bin_indices[i] = i;
  for (int i = 0; i < ARM_TABLE_ROWS && i <= N_HALF; i++) {
    int max_at = i;
    for (int j = i + 1; j <= N_HALF; j++)
      if (g_magnitude[sorted_bin_indices[j]] >
          g_magnitude[sorted_bin_indices[max_at]])
        max_at = j;
    int tmp = sorted_bin_indices[i];
    sorted_bin_indices[i] = sorted_bin_indices[max_at];
    sorted_bin_indices[max_at] = tmp;
  }

  int x = 2, y = 2;
  if (y + ARM_TABLE_ROWS + 1 >= LINES - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, " bin   |X[k]|     arg(rad)");
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int row = 0; row < ARM_TABLE_ROWS && row <= N_HALF; row++) {
    int bin_index = sorted_bin_indices[row];
    ComplexNumber z = g_dft_spectrum[bin_index];
    float phase = atan2f(z.im, z.re);
    attron(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
    mvprintw(y + 1 + row, x, "  %2d   %7.3f   %+6.3f", bin_index,
             (double)g_magnitude[bin_index], (double)phase);
    attroff(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
  }
}

/* ===================================================================== */
/* §15  hud — status top-right + hint bottom + frame composer            */
/* ===================================================================== */

static void draw_hud(int dominant_bin) {
  char status[160];
  snprintf(status, sizeof status,
           " DFT helloworld  N=%d  freq=%5.2f (peak bin=%d)  %s  %s ", N,
           (double)g_freq_bin, dominant_bin,
           g_auto_sweep_enabled ? "AUTO  " : "MANUAL",
           g_simulation_paused ? "PAUSED" : "      ");
  int x = COLS - (int)strlen(status);
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hint(void) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(LINES - 1, 0,
           " q:quit  spc:pause  a:auto/manual  +/-:freq  ,/.:fine "
           " d:phase  D:bins  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Reserved rows: 1 hud + 2 panel labels + 1 hint = 4. */
  int rows_for_panels = LINES - 4;
  if (rows_for_panels < 6)
    rows_for_panels = 6;

  /* Phase panel "steals" a chunk of vertical space when toggled. */
  int phase_h = g_show_phase_panel ? rows_for_panels / 4 : 0;
  int main_h = rows_for_panels - phase_h;
  int sig_h = main_h / 2;
  int spec_h = main_h - sig_h;

  int sig_label_row = 1;
  int sig_top_row = 2;
  int spec_label_row = 2 + sig_h;
  int spec_top_row = 3 + sig_h;
  int phase_top_row = 3 + sig_h + spec_h; /* used only if show_phase */

  /* Panel labels. */
  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(sig_label_row, 0, "Input x[n] = cos(2*pi * freq * n / N)");
  mvprintw(spec_label_row, 0, "DFT magnitude |X[k]|  (bins 0..%d)", N_HALF);
  if (g_show_phase_panel)
    mvprintw(phase_top_row - 1, 0,
             "DFT phase arg(X[k])  (magenta = +, sky = -)");
  attroff(COLOR_PAIR(PAIR_LABEL));

  /* Panels. */
  draw_signal_panel(sig_top_row, sig_h);
  int dominant_bin = 0;
  draw_spectrum_panel(spec_top_row, spec_h, &dominant_bin);
  if (g_show_phase_panel)
    draw_phase_panel(phase_top_row, phase_h);

  /* Debug overlay (drawn on top of panels but under HUD). */
  if (g_show_arm_table)
    draw_arm_table();

  /* HUD always last. */
  draw_hud(dominant_bin);
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §16  app — signal handlers + main loop + key dispatch                 */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* Async-signal-safe: only set flags; main loop handles them. */
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static void cleanup_screen(void) { endwin(); }

static bool app_handle_key(int ch) {
  /* Returns true on quit. */
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return true;
  case ' ':
    g_simulation_paused = !g_simulation_paused;
    break;
  case 'a':
  case 'A':
    g_auto_sweep_enabled = !g_auto_sweep_enabled;
    break;
  case '+':
  case '=':
    scene_adjust_freq(+1.0f);
    break;
  case '-':
    scene_adjust_freq(-1.0f);
    break;
  case '.':
    scene_adjust_freq(+0.1f);
    break;
  case ',':
    scene_adjust_freq(-0.1f);
    break;
  case 'd':
    g_show_phase_panel = !g_show_phase_panel;
    break;
  case 'D':
    g_show_arm_table = !g_show_arm_table;
    break;
  case 'r':
  case 'R':
    scene_reset();
    break;
  default:
    break;
  }
  return false;
}

int main(void) {
  atexit(cleanup_screen);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init();

  scene_reset();

  long long next_frame_ns = clock_now_ns();

  while (!g_should_quit) {
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
    }

    int ch;
    while ((ch = getch()) != ERR) {
      if (app_handle_key(ch)) {
        g_should_quit = 1;
        break;
      }
    }

    long long now = clock_now_ns();
    if (now >= next_frame_ns) {
      scene_tick();
      render_frame();
      next_frame_ns += RENDER_TICK_NS;
      /* Snap forward if we fell badly behind (e.g. terminal hidden). */
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
