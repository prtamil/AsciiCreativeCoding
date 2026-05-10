/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * idft_helloworld.c — animated, minimum-viable INVERSE DFT demo with
 *                     spectrum manipulation.  Five modes: round-trip
 *                     verification, low-pass-via-spectrum, high-pass-
 *                     via-spectrum, magnitude-only reconstruction,
 *                     phase-only reconstruction.  The IDFT itself
 *                     reads as pseudocode so the math and the C are
 *                     line-for-line identical.
 *
 * DEMO
 *   Three stacked panels.
 *     TOP        original input signal x[n]
 *     MIDDLE     magnitude spectrum |X[k]| (with dim-grey bars showing
 *                bins killed by the active mode + a yellow ':' cutoff
 *                marker for filter modes)
 *     BOTTOM     reconstructed signal y[n] = IDFT(modified X)
 *
 *   Press 'm' to cycle through five modes:
 *
 *     0 ROUND-TRIP   y = IDFT(DFT(x)).  Verifies the perfect
 *                    inversion identity bit-for-bit.  HUD prints
 *                    "round-trip err=~1e-6 [PASS]".
 *
 *     1 LOW-PASS     Zero out X[k] for k in (cutoff, N - cutoff),
 *                    then IDFT.  Result is a smoothed version of
 *                    the input.  Cutoff auto-sweeps.
 *
 *     2 HIGH-PASS    Zero out X[k] for k <= cutoff and k >= N - cutoff,
 *                    then IDFT.  Result keeps only the rapid changes.
 *
 *     3 MAG-ONLY     Set every X[k]'s phase to zero (keep magnitude).
 *                    Reconstruction loses ALL time-shift info — it's
 *                    symmetric around the buffer centre.
 *
 *     4 PHASE-ONLY   Set every X[k]'s magnitude to 1 (keep phase).
 *                    Surprise: the result still RESEMBLES the
 *                    original's shape!  This is the famous
 *                    Oppenheim & Lim 1981 "phase carries shape"
 *                    pedagogy made visible.
 *
 *   Two debug overlays:
 *     'd' shows the PHASE PANEL — arg(X[k]) for each bin, drawn
 *         around a midline (magenta = positive, sky = negative).
 *         Magnitude alone discards phase; this overlay shows what
 *         the spectrum panel throws away.
 *     'D' shows a numeric BIN-INFO TABLE for the eight loudest
 *         bins (k, |X[k]|, arg(X[k])) — useful for verifying
 *         decode by hand.
 *
 *   The headline lesson: the DFT is a perfect change of basis (mode 0
 *   proves it), modifying the spectrum is just filtering by another
 *   name (modes 1-2), and magnitude vs phase is a pedagogical
 *   landmine that mode 3 + 4 detonate visibly.
 *
 * Study alongside
 *   signal/dft_helloworld.c          The forward DFT.  Read it first.
 *   signal/fft_helloworld.c          The same DFT, but O(N log N)
 *                                     via the Cooley-Tukey FFT.
 *   signal/convolution_helloworld.c  Filtering in the time domain.
 *                                     Modes 1-2 here are the same
 *                                     operation in the frequency
 *                                     domain (T9 derives the duality).
 *   signal/fft_vis.c                 Window functions = spectrum-
 *                                     domain tricks done at FFT
 *                                     input time.
 *
 * Section map
 *   §1   config            constants
 *   §2   clock             monotonic-ns timer + sleep
 *   §3   complex           ComplexNumber + 7 helpers
 *   §4   colors            palette pairs (HUD/HINT theme-invariant)
 *   §5   dft               forward DFT (textbook double loop)
 *   §6   idft              INVERSE DFT (the headline new function)
 *   §7   signal_kinds      enum + names + 5 generators
 *   §8   mode_kinds        enum + names for the 5 modes
 *   §9   spectrum_mod      modes 1-4 dispatched (mode 0 = no-op)
 *   §10  scene_state       per-frame state + reset
 *   §11  scene_tick        per-frame update orchestrator
 *   §12  scene_input       handle keys
 *   §13  draw_bar          vertical-bar primitive
 *   §14  draw_signals      input + reconstruction panels
 *   §15  draw_spectrum     magnitude + phase + debug table
 *   §16  hud               status top-right + hint bottom + frame composer
 *   §17  app               signal handlers + main loop + key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   m / M            next / previous mode
 *   s / S            next / previous input signal
 *   a                toggle auto-sweep cutoff (modes 1 / 2)
 *   + / -            (manual) change cutoff by 1 bin
 *   , / .            (manual) change cutoff by 1 bin (alt keys)
 *   d                toggle phase panel (debug)
 *   D                toggle bin-info numeric table top-left (debug)
 *   r                reset (mode 0, default signal, fresh state)
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/idft_helloworld.c \
 *       -o idft_helloworld -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no IDFT background:
 *     1. CONCEPTS — the elevator pitch with references.
 *     2. MENTAL MODEL — analogies, diagrams, the worked example.
 *     3. GUIDED TUTORIAL T1..T11 — eleven mini-lessons that derive
 *        the IDFT from first principles, then walk through each
 *        of the five modes in turn.  Each ends in pseudocode that
 *        maps 1:1 to a function in §5..§9.
 *     4. §1 config — the knobs.
 *     5. §3 complex — the tiny abstraction that lets the algorithm
 *        code read like the math.  Read this BEFORE §6.
 *     6. §6 idft — THE function in this file.  Every other line of
 *        code exists to set this up, run it, or visualise its output.
 *     7. §9 spectrum_mod — the four mode mutations.  Each is one
 *        switch case; each is a different lesson.
 *     8. §10..§17 — scene + rendering plumbing + main loop.
 *
 *   Reader who already knows DFT/IDFT, just wants the code:
 *     §1, §3, §6 idft, §9 spectrum_mod, §11 scene_tick.  The rest
 *     is ncurses scaffolding.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     g_input_signal[N]              real-valued time-domain samples
 *     g_spectrum[N]                  forward DFT of input (complex)
 *     g_spectrum_modified[N]         after spectrum_mod (complex)
 *     g_reconstruction[N]            IDFT of modified spectrum (complex)
 *     g_magnitude[N/2 + 1]           |X[k]| for the visible spectrum panel
 *     output_sample_index, n         IDFT outer-loop counter
 *     input_bin_index, k             IDFT inner-loop counter
 *     twist_angle_radians            angle inside exp(+i * angle)
 *     twist                          exp(+i * twist_angle) — unit-circle complex
 *     twisted                        X[k] * twist (one rotated arrow)
 *     running_sum                    IDFT accumulator
 *     normalisation_one_over_N       the 1/N factor (made explicit)
 *     g_animation_phase_radians      sweep's own phase (NOT the DFT phase)
 *     g_round_trip_error             |y - x| max bin-by-bin (mode 0)
 *
 *   Inside tight loops single-letter counters (`n`, `k`, `i`) stay
 *   short where context is one line away.
 *
 * Background you NEED
 *   - The forward DFT formula.  See signal/dft_helloworld.c CONCEPTS
 *     for a self-contained intro.
 *   - Basic complex arithmetic (add, multiply, magnitude, phase).
 *     §3 reviews the operations we use; T2 of dft_helloworld covers
 *     the geometry.
 *
 * Background you do NOT need
 *   - The Fast Fourier Transform.  We use the textbook O(N^2) DFT
 *     and IDFT.  At N = 32 the cost is trivial.
 *   - Continuous calculus.  Everything is finite sums.
 *   - Linear algebra (the DFT/IDFT are matrix operations but we
 *     never need to write the matrix explicitly).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * What is the IDFT, in one sentence?
 *   The mathematical inverse of the DFT.  Take N complex frequency-
 *   domain samples X[0..N-1], get back the N time-domain samples
 *   x[0..N-1] that produced them.  Together DFT and IDFT form a
 *   PERFECT change of basis between time and frequency domains.
 *
 * Algorithm
 *   Direct evaluation of the IDFT formula by two nested loops, just
 *   like the forward DFT but with TWO sign changes:
 *
 *     x[n] = (1/N) * sum_{k=0..N-1} X[k] * exp(+i * 2*pi * k * n / N)
 *
 *   Differences from the forward DFT (T2 derives the WHY):
 *     (1) The exponent has +i (not -i).  CCW rotation, not CW.
 *     (2) There's a 1/N normalisation factor at the front.
 *     (3) Inputs are COMPLEX (X[k]), so the per-sample multiplication
 *         is full complex × complex (not real × complex like in DFT).
 *
 *   For real-valued time-domain signals (always the case here) the
 *   imaginary part of the IDFT output is essentially zero (machine
 *   epsilon).  We display only the real part.
 *
 * Round-trip property (mode 0)
 *   x = IDFT(DFT(x))  exactly, within float epsilon.  Mode 0 of
 *   this demo verifies it every frame and prints the max bin-by-bin
 *   error in the HUD (typically ~1e-6 in single precision at N = 32).
 *   This is the operational definition of "DFT and IDFT are
 *   inverses".
 *
 * Spectrum manipulation as filtering
 *   Modifying X before running IDFT is the EASIEST way to filter.
 *   Zero out high bins  →  low-pass.
 *   Zero out low bins   →  high-pass.
 *   Multiply X[k] by H[k] for any H[k]  →  arbitrary filter.
 *   This works because of the convolution-multiplication duality
 *   (T9): modifying X is mathematically identical to convolving x
 *   with the IDFT of the modification mask.  Both are valid; spectrum-
 *   domain filtering is often easier to design.
 *
 * Magnitude vs phase
 *   The DFT output X[k] has both magnitude |X[k]| and phase arg(X[k]).
 *   Conventional spectrum analysers display only magnitude — but
 *   that's a 50% lossy view.
 *
 *     MAGNITUDE  carries the AMPLITUDE of each frequency component.
 *     PHASE      carries the relative TIMING of those components.
 *
 *   Drop magnitude  →  all frequencies normalised; the result still
 *   roughly RESEMBLES the original's shape because phase encodes
 *   "where the features sit in time".
 *
 *   Drop phase  →  all frequency components aligned at t = 0; the
 *   result is symmetric around index 0 ("zero-phase reconstruction").
 *   For most signals this looks like a centred blob bearing little
 *   resemblance to the original.
 *
 *   Modes 3 (mag-only) and 4 (phase-only) make the magnitude/phase
 *   split physically visible.  T7-T8 cover the pedagogy.
 *
 * Performance / boundaries
 *   - N = 32 samples.  Forward DFT + spectrum mod + IDFT ≈ 2100
 *     multiplies per frame.  Trivial.
 *   - Both DFT and IDFT are O(N^2) here.  For larger N you'd switch
 *     to the FFT/IFFT (signal/fft_helloworld.c).
 *   - All math is single-precision float; round-trip error lands
 *     around 1e-6.  Use double for tighter precision.
 *
 * Rendering
 *   Three stacked vertical-bar panels (input, magnitude spectrum
 *   with kill mask, reconstruction) plus HUD with PAUSED chip and
 *   bottom-line key hints.  Two optional debug overlays (toggled
 *   by 'd' and 'D') add a phase panel and a numeric bin-info
 *   table.  All theme-invariant per CLAUDE.md HUD Standard.
 *
 * References
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapter 8 (DFT) and Chapter 12 (FFT).
 *     Free online; the gentlest in-depth intro I know of.
 *   Oppenheim, A. V., & Lim, J. S. (1981), "The importance of phase
 *     in signals", Proceedings of the IEEE, 69(5).  The classic
 *     paper that demonstrated phase carries the shape — modes 3 and
 *     4 of this demo are direct echoes of its experiments.
 *   Bracewell, R. N., "The Fourier Transform and Its Applications".
 *     Textbook reference for the convolution-multiplication duality.
 *   https://en.wikipedia.org/wiki/Discrete_Fourier_transform
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   The DFT and IDFT are inverse operations.  Together they form a
 *   PERFECT change of basis between the time domain (samples in time)
 *   and the frequency domain (amplitudes + phases at integer
 *   frequencies).  No information lost in either direction.
 *
 *   Filtering becomes trivial in this view: go to the frequency
 *   domain, throw away whatever you don't want, come back.  Modes
 *   1 and 2 of this demo show low-pass and high-pass via this
 *   technique — the cleanest possible filter implementation.
 *
 *   Modes 3 and 4 carve the spectrum apart in a different way: keep
 *   ONLY magnitude, or keep ONLY phase, and show that the
 *   reconstruction looks dramatically different in each case.  The
 *   "phase wins for shape" demo (mode 4) is the classical pedagogy
 *   from Oppenheim & Lim's 1981 paper.
 *
 * HOW TO THINK ABOUT IT — THE PRISM PICTURE
 *   Imagine the DFT as a glass prism and the IDFT as a reverse
 *   prism.  Light goes through the first one and gets split into a
 *   rainbow (the spectrum).  The reverse prism collects the rainbow
 *   back into white light (the original signal).  No energy lost,
 *   provided you do nothing to the rainbow in between.
 *
 *   What if you BLOCK part of the rainbow?  You filtered the light.
 *   What if you make all colours equally bright (magnitude
 *   normalisation)?  You normalised by colour.  What if you only
 *   keep the brightnesses but throw away the colours' relative
 *   positions (phase)?  You destroyed the focus.  Each operation
 *   has a meaning, and IDFT lets you see the result.
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │   Mode 0 ROUND-TRIP                                    │
 *      │     x → DFT → X → IDFT → y                             │
 *      │     y == x  (perfect identity, |err| ~ 1e-6)           │
 *      │                                                        │
 *      │   Mode 1 LOW-PASS                                      │
 *      │     x → DFT → X → zero high bins → IDFT → y            │
 *      │     y is a smoothed x (rounded edges)                  │
 *      │                                                        │
 *      │   Mode 2 HIGH-PASS                                     │
 *      │     x → DFT → X → zero low bins → IDFT → y             │
 *      │     y has only the rapid changes (edges + transients)  │
 *      │                                                        │
 *      │   Mode 3 MAG-ONLY                                      │
 *      │     x → DFT → X → set arg(X) = 0 → IDFT → y            │
 *      │     y is symmetric around N/2 (centred blob)           │
 *      │     Phase carries TIMING — discarded → no time info    │
 *      │                                                        │
 *      │   Mode 4 PHASE-ONLY                                    │
 *      │     x → DFT → X → set |X| = 1 → IDFT → y               │
 *      │     y looks like x's SHAPE, normalised amplitude       │
 *      │     Phase carries SHAPE — preserved → shape preserved  │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * WORKED EXAMPLE — ROUND-TRIP FOR x = COSINE AT BIN 2, N = 8
 *
 *   Input:    x[n] = cos(2*pi*2*n/8) = cos(pi*n/2)
 *             x = [1, 0, -1, 0, 1, 0, -1, 0]
 *
 *   Forward DFT (computed by §5):
 *             |X[2]| = N/2 = 4   at bin 2
 *             |X[6]| = N/2 = 4   at bin 6 (= N - 2; conjugate twin)
 *             All other |X[k]| = 0  (within float epsilon)
 *
 *   Mode 0 — IDFT(X) using §6:
 *             For each n in [0, N):
 *               y[n] = (1/8) * (X[2] * exp(+i*2pi*2*n/8)
 *                             + X[6] * exp(+i*2pi*6*n/8))
 *
 *             X[2] = 4 + 0i  (real number, at bin 2)
 *             X[6] = 4 + 0i  (its conjugate twin)
 *
 *             For n = 0:  y[0] = (1/8)*(4*1 + 4*1) = 1.0   ✓
 *             For n = 1:  y[1] = (1/8)*(4*exp(+i*pi/2)  + 4*exp(+i*3pi/2))
 *                              = (1/8)*(4i + (-4i)) = 0.0   ✓
 *             For n = 2:  y[2] = (1/8)*(4*exp(+i*pi)   + 4*exp(+i*3pi))
 *                              = (1/8)*(-4 + (-4)) = -1.0   ✓
 *
 *             y matches x exactly.  Try the rest by hand if you doubt.
 *
 *   This is the round-trip property in action.  At single-precision
 *   the actual error per sample is around 1e-6 (float epsilon
 *   compounded over N multiplications).
 *
 * ALGORITHM IN STEPS  (per frame)
 *
 *   1. Generate the active signal x[n] into g_input_signal[N].
 *   2. Run forward DFT: X = DFT(x), stored in g_spectrum[].
 *   3. Copy X into g_spectrum_modified[], then apply spectrum
 *      modification per the active mode:
 *        Mode 0: no modification (pure round-trip).
 *        Mode 1: zero out X[k] for k > cutoff (and conjugate twins).
 *        Mode 2: zero out X[k] for k < cutoff (and conjugate twins).
 *        Mode 3: set X[k].im = 0 (force phase to 0).
 *        Mode 4: replace X[k] with X[k] / |X[k]| (force magnitude to 1).
 *   4. Run IDFT: y = IDFT(modified X), stored in g_reconstruction[].
 *   5. Compute |X[k]| for the visible spectrum panel + max bin-by-bin
 *      error |y - x| (mode 0 only, for the HUD).
 *   6. Render: input panel, spectrum panel, reconstruction panel,
 *      optional debug overlays, HUD.
 *
 * KEY FORMULAS
 *
 *   Forward DFT (computed in §5):
 *     X[k] = sum_{n=0..N-1} x[n] * exp(-i * 2*pi * k * n / N)
 *
 *   Inverse DFT (computed in §6 — note +i and the 1/N):
 *     x[n] = (1/N) * sum_{k=0..N-1} X[k] * exp(+i * 2*pi * k * n / N)
 *
 *   Round-trip (mode 0):
 *     x' = IDFT(DFT(x))
 *     |x' - x| = O(machine epsilon) ~ 1e-6 in single precision
 *
 *   Spectrum-domain low-pass (mode 1):
 *     mask[k] = (k <= cutoff || k >= N - cutoff) ? 1 : 0
 *     y = IDFT(X * mask)
 *
 *   Magnitude-only (mode 3):
 *     X'[k] = (|X[k]|, 0)             ← keep magnitude, force imag = 0
 *     y = IDFT(X')
 *
 *   Phase-only (mode 4):
 *     X'[k] = X[k] / |X[k]|           ← unit-magnitude, original phase
 *     y = IDFT(X')
 *
 * EDGE CASES TO WATCH
 *
 *   - The IDFT of a real signal's spectrum (after we modify it) might
 *     not be perfectly real — small imaginary parts appear due to
 *     machine epsilon and our modifications breaking conjugate
 *     symmetry.  We display the REAL part only and ignore the imag.
 *
 *   - Modes 1 and 2 (filters) preserve CONJUGATE SYMMETRY by zeroing
 *     X[k] and X[N - k] together.  Without that symmetry the IDFT
 *     output would have a non-trivial imaginary component — i.e.
 *     the "low-pass filtered version" would not be a real signal.
 *     T6 covers conjugate symmetry in detail.
 *
 *   - Mode 3 (magnitude-only) sets X[k].im = 0, which is the same
 *     as forcing phase to 0 (or pi for negative reals).  This
 *     destroys time-shift information — the reconstruction is
 *     symmetric around the centre of the buffer.
 *
 *   - Mode 4 (phase-only) divides by |X[k]| which can be tiny for
 *     bins where the signal has no energy.  We guard with a small
 *     epsilon (1e-6) to avoid divide-by-zero.
 *
 *   - Cutoff range: 1 to N/2 - 1.  At cutoff = N/2 - 1 the low-pass
 *     keeps essentially everything; at cutoff = 1 it keeps only DC
 *     and the fundamental.
 *
 * HOW TO VERIFY
 *   - Mode 0 should display "round-trip err: ~1e-6 [PASS]" in the
 *     HUD.  The reconstruction panel should match the input panel
 *     exactly (ignoring the floor-of-noise).
 *
 *   - Mode 1 with cutoff = 4: input "Square" should turn into a
 *     smooth sinusoidal-looking wave.  Bumping cutoff to 16 should
 *     restore the square corners.
 *
 *   - Mode 2 with cutoff = 4: input "Square" should show only the
 *     vertical edges as positive/negative spikes.
 *
 *   - Mode 3 with input "Square": the reconstruction is a CENTRED
 *     spike — all the square's harmonics interfere constructively
 *     at index N/2 and destructively elsewhere.  This is the visual
 *     proof that magnitude alone discards time-shift info.
 *
 *   - Mode 4 with input "Square": the reconstruction LOOKS LIKE A
 *     SQUARE, just with smaller amplitude.  Phase preserved the
 *     shape!  This is the Oppenheim & Lim 1981 classic.
 *
 *   - Press 'd' for the phase panel — see arg(X[k]) drawn as bars
 *     above/below a midline.  In modes 1-2 the phase is preserved
 *     for surviving bins; in mode 3 it's zeroed; in mode 4 it's
 *     the only thing left.
 *
 *   - Press 'D' for the bin-info table — top 8 bins by magnitude,
 *     with their (k, |X[k]|, arg(X[k])) printed numerically.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What is the IDFT and why does it matter?
 *   T2   The IDFT formula vs the DFT formula (the two sign changes)
 *   T3   Why is there a 1/N? — the normalisation derivation
 *   T4   The round-trip identity x = IDFT(DFT(x))  (mode 0)
 *   T5   Spectrum manipulation as filtering  (modes 1-2)
 *   T6   Conjugate symmetry — why filter modes preserve realness
 *   T7   Magnitude alone discards time-shift info  (mode 3)
 *   T8   Phase carries the shape  (mode 4 — the Oppenheim & Lim demo)
 *   T9   The convolution-multiplication duality
 *   T10  Per-frame pipeline + how each mode mutates the spectrum
 *   T11  How the demo works — signal/mode cycling, cutoff sweep
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS THE IDFT AND WHY DOES IT MATTER?
 * ────────────────────────────────────────────
 * The forward DFT (signal/dft_helloworld.c) takes a time-domain
 * signal x[n] and returns its frequency-domain spectrum X[k] —
 * a list of complex amplitudes telling you "how much of frequency
 * k is in the signal".
 *
 * The IDFT does the OPPOSITE.  Given a spectrum X[k] (any spectrum,
 * not just one produced by the DFT), it returns the time-domain
 * signal x[n] that would have produced that spectrum.
 *
 * Why does this matter?
 *
 *   1. The DFT is INVERTIBLE.  No information is lost in the
 *      forward direction — the IDFT can always recover the original.
 *      This is what makes spectral analysis non-destructive: you can
 *      look at the spectrum and then reconstruct the signal exactly.
 *
 *   2. SPECTRUM MANIPULATION = FILTERING.  Going to the frequency
 *      domain, modifying the spectrum, and coming back is the
 *      cleanest possible filter.  Want to remove all frequencies
 *      above some cutoff?  Zero out those bins, IDFT, done.
 *      Modes 1-2 of this demo do exactly that.
 *
 *   3. CONSTRUCTING SIGNALS BY SPECTRUM.  If you know you want a
 *      signal that has specific frequency content (e.g. a tone at
 *      440 Hz with no harmonics), you can build the spectrum
 *      directly and IDFT it to get the time-domain samples.  This
 *      is how additive audio synthesis works.
 *
 *   4. ROUND-TRIP VERIFICATION.  Computing IDFT(DFT(x)) and checking
 *      it equals x is the standard test for "does my DFT
 *      implementation work?".  Mode 0 of this demo does this every
 *      frame and shows the error.
 *
 * T2  THE IDFT FORMULA VS THE DFT FORMULA  (the two sign changes)
 * ───────────────────────────────────────────────────────────────
 * The forward DFT (dft_helloworld.c §5) is:
 *
 *     X[k] = sum_{n=0..N-1} x[n] * exp(-i * 2*pi * k * n / N)
 *
 * The inverse DFT (this file's §6) is:
 *
 *     x[n] = (1/N) * sum_{k=0..N-1} X[k] * exp(+i * 2*pi * k * n / N)
 *
 * Two sign changes go from DFT to IDFT:
 *
 *   (1) The exponent has +i (CCW rotation) instead of -i (CW).
 *       The forward DFT "untwists" each input sample by rotating
 *       it backward; the IDFT "re-twists" each frequency by rotating
 *       it forward.  Together they undo each other exactly.
 *
 *   (2) There's a 1/N normalisation factor at the FRONT of the IDFT
 *       formula.  T3 derives why.
 *
 * Read both formulas side by side:
 *
 *     DFT  : X[k] = sum_n  x[n] * exp(-i * angle)        (no 1/N)
 *     IDFT : x[n] = sum_k  X[k] * exp(+i * angle) / N    (with 1/N)
 *
 *     where angle = 2*pi * k * n / N is the same in both.
 *
 * Notice the symmetric structure.  Some textbook conventions split
 * the 1/N differently (e.g. 1/sqrt(N) on each side, making both
 * "unitary").  We use the asymmetric convention because it matches
 * the most common engineering codebase (numpy, scipy, etc.).
 *
 * T3  WHY IS THERE A 1/N? — THE NORMALISATION DERIVATION
 * ──────────────────────────────────────────────────────
 * Plug DFT into IDFT and simplify.  Goal: prove IDFT(DFT(x)) = x.
 *
 *     IDFT(DFT(x))[n']
 *   = (1/N) * sum_k DFT(x)[k] * exp(+i*2*pi*k*n'/N)
 *   = (1/N) * sum_k (sum_n x[n] * exp(-i*2*pi*k*n/N))
 *                  * exp(+i*2*pi*k*n'/N)
 *   = (1/N) * sum_n x[n] * sum_k exp(+i*2*pi*k*(n'-n)/N)
 *
 * The inner sum sum_k exp(+i*2*pi*k*(n' - n)/N) is a finite
 * geometric series.  It equals:
 *   - N        if n' == n  (every term is 1, summed N times)
 *   - 0        otherwise   (geometric series sums to 0)
 *
 * So:
 *
 *     IDFT(DFT(x))[n'] = (1/N) * x[n'] * N = x[n']     ✓
 *
 * The 1/N is exactly what cancels the N from the inner-sum
 * orthogonality property.  Without it we'd get IDFT(DFT(x)) = N*x.
 * With it we get the perfect inversion.
 *
 * T4  THE ROUND-TRIP IDENTITY  x = IDFT(DFT(x))   (mode 0)
 * ────────────────────────────────────────────────────────
 * Mode 0 of this demo computes IDFT(DFT(x)) and compares it sample-
 * by-sample to x.  The HUD shows the maximum bin-by-bin difference:
 *
 *     g_round_trip_error = max over n of |y[n] - x[n]|
 *
 * In single-precision floats this number lands around 1e-6 to 1e-5
 * — float epsilon compounded over N multiplications.  The HUD
 * declares PASS if it's below 1e-4 (a generous threshold).
 *
 * If you ever see FAIL with a noticeable error, your DFT or IDFT
 * implementation has a bug.  This is the #1 unit test for any new
 * Fourier code.
 *
 * Press 'r' to reset to mode 0 and watch it pass on every input
 * signal.  Cycle 's' through Sum-sines, Square, Sawtooth, Impulse,
 * Noise — round-trip works for ALL of them, because the DFT is a
 * change of basis and changes of basis are bijective by definition.
 *
 * T5  SPECTRUM MANIPULATION AS FILTERING  (modes 1-2)
 * ───────────────────────────────────────────────────
 * Mode 1 (LOW-PASS) zeroes out the high-frequency bins of the
 * spectrum, then runs IDFT.  Result: a smoothed signal.
 *
 * Mode 2 (HIGH-PASS) zeroes out the low-frequency bins, then runs
 * IDFT.  Result: a signal with only the rapid changes (edges,
 * transitions).
 *
 * In code, the mask for low-pass with cutoff K is:
 *
 *   mask[k] = (k <= cutoff || k >= N - cutoff) ? 1 : 0
 *   spectrum_modified[k] = spectrum[k] * mask[k]
 *   reconstruction = IDFT(spectrum_modified)
 *
 * The "k >= N - cutoff" part is critical — see T6.
 *
 * Why does this filter the signal?  Because each spectrum bin X[k]
 * represents the AMPLITUDE of one specific frequency in the signal.
 * Zero out a bin → that frequency is completely removed.  Keep a
 * bin → that frequency passes through unchanged.
 *
 * This is the FREQUENCY-DOMAIN filter design technique.  It's
 * mathematically equivalent to time-domain convolution (T9) but
 * conceptually much simpler — you just decide which bins to keep
 * and which to throw away.
 *
 * Try it: press 'm' to mode 1, 's' to "Square".  Watch as the
 * cutoff auto-sweeps: at low cutoffs the square becomes a sine; at
 * high cutoffs it sharpens back into a square.  The dim grey bars
 * in the spectrum panel show which bins are killed at each moment.
 *
 * T6  CONJUGATE SYMMETRY — WHY FILTER MODES PRESERVE REALNESS
 * ───────────────────────────────────────────────────────────
 * For any REAL-valued time-domain signal x[n] (always the case in
 * this demo), the DFT output has CONJUGATE SYMMETRY:
 *
 *     X[N - k] = conjugate(X[k])   for k in [1, N-1]
 *
 * This means that if X[k] = a + bi, then X[N - k] = a - bi.  The
 * upper half of the spectrum is the mirror image of the lower half.
 *
 * Why does this matter for our filters?  Because IDFT preserves
 * realness ONLY IF the input spectrum has conjugate symmetry.  If
 * we zero out X[k] but not X[N - k], the symmetry is broken and
 * the IDFT output has a non-zero imaginary component — i.e., the
 * "filtered signal" is no longer real.
 *
 * To keep the filtered signal real, we ALWAYS zero X[k] and X[N - k]
 * together.  In the low-pass mask:
 *
 *     mask[k]      = 1 for k <= cutoff       (keep low positive-freq)
 *     mask[N - k]  = 1 for N - k >= N - cutoff (keep their twins)
 *     mask[k]      = 0 otherwise              (zero everything else)
 *
 * The condition "k <= cutoff || k >= N - cutoff" implements this
 * with a single test.
 *
 * If you want to FORCE non-real output for some reason (say, an
 * analytic signal in radar), break the symmetry deliberately.  But
 * for normal filtering, always preserve it.
 *
 * Modes 3 and 4 deliberately break symmetry — but the result is
 * still approximately real for symmetric input signals (small
 * imaginary part from float epsilon).  We display .re and ignore
 * .im.
 *
 * T7  MAGNITUDE ALONE DISCARDS TIME-SHIFT INFO  (mode 3)
 * ──────────────────────────────────────────────────────
 * Mode 3 (MAG-ONLY) sets every X[k]'s phase to zero, keeping only
 * the magnitude:
 *
 *     X'[k] = (|X[k]|, 0)          ← real number, magnitude preserved
 *
 * Then IDFT.  The result is ALWAYS SYMMETRIC around index N/2.
 *
 * Why?  Because X' has the property X'[k] = X'[N - k] (both equal
 * the magnitude).  This is "real and symmetric" in the spectrum.
 * The IDFT of a real symmetric spectrum is a SYMMETRIC TIME-DOMAIN
 * SIGNAL.
 *
 * In information-theoretic terms: phase encodes WHEN something
 * happens.  Strip the phase → you've thrown away all timing info →
 * the signal becomes a centred blob with no preferred direction.
 *
 * This is why audio "spectrograms" (which show |X[k]| over time)
 * don't let you reconstruct the audio — phase is missing.  Two
 * sounds with identical magnitude spectra can be distinct to the
 * ear because of phase.
 *
 * Try mode 3 with input "Square" — you'll see a single tall spike
 * at the buffer centre, instead of a square.  All the square's
 * harmonics constructively interfere at the centre and destructively
 * everywhere else.
 *
 * T8  PHASE CARRIES THE SHAPE  (mode 4 — the Oppenheim & Lim demo)
 * ────────────────────────────────────────────────────────────────
 * Mode 4 (PHASE-ONLY) sets every X[k]'s magnitude to 1, keeping
 * only the phase:
 *
 *     X'[k] = X[k] / |X[k]|         ← unit-magnitude complex number
 *
 * Then IDFT.  Surprise: the result LOOKS LIKE THE ORIGINAL'S SHAPE,
 * just with normalised amplitude (all frequencies equally loud).
 *
 * This was the headline finding of Oppenheim & Lim's 1981 paper
 * "The importance of phase in signals".  They showed that for
 * signals with concentrated time-domain energy (like images and
 * speech), phase alone can be enough to recognise the original.
 *
 * Try mode 4 with input "Square" — you'll see a square-ish wave
 * (approximate, but recognisable as a square).  Try mode 4 with
 * "Sawtooth" — you'll see a sawtooth-ish wave.  Phase preserved
 * the SHAPE; magnitude carried the AMPLITUDE.
 *
 * Practical implication: when designing audio compression, you can
 * be aggressive about magnitude quantisation but must preserve
 * phase carefully.  When designing image compression, the same.
 * MP3 keeps both magnitude AND phase, but allocates more bits to
 * magnitude in regions where the ear is less sensitive to phase
 * (controlled by psychoacoustic models).
 *
 * T9  THE CONVOLUTION-MULTIPLICATION DUALITY
 * ──────────────────────────────────────────
 * One of the most important identities in all of mathematics:
 *
 *     CONVOLUTION IN TIME  =  MULTIPLICATION IN FREQUENCY
 *
 * Formally:
 *
 *     y[n]  =  (h * x)[n]              ← time-domain convolution
 *     Y[k]  =  H[k] * X[k]             ← frequency-domain product
 *
 *     where Y = DFT(y), X = DFT(x), H = DFT(h)
 *
 * This file's modes 1 and 2 are spectrum-domain filtering: we set
 * H[k] = 1 in the passband and H[k] = 0 in the stopband, multiply,
 * and IDFT.  signal/convolution_helloworld.c does the time-domain
 * version: it convolves with a kernel directly.
 *
 * Both produce the same answer.  Two implementations of one
 * mathematical operation.  Choose based on:
 *
 *   FREQUENCY-DOMAIN (this file's approach):
 *     - Easier to design — just decide which bins to keep
 *     - Naturally circular (wraps around buffer boundaries)
 *     - Requires DFT + multiply + IDFT
 *
 *   TIME-DOMAIN (convolution_helloworld.c approach):
 *     - Easier to understand intuitively
 *     - Naturally linear (no wrap-around)
 *     - Just slide-and-multiply, no FFT needed for short kernels
 *
 * For long signals + long kernels, frequency-domain wins (FFT-based
 * convolution is O(N log N) vs O(N*K) direct).  For short kernels,
 * time-domain is cheaper (no FFT setup overhead).
 *
 * T10  PER-FRAME PIPELINE + HOW EACH MODE MUTATES THE SPECTRUM
 * ────────────────────────────────────────────────────────────
 * Per frame, scene_tick (§11) does six steps:
 *
 *   Step 1.  generate_signal(g_signal_kind, g_input_signal)
 *            Fills g_input_signal[N] with the chosen test signal.
 *
 *   Step 2.  compute_dft(g_input_signal, g_spectrum)
 *            Forward DFT.  g_spectrum holds the full N complex bins.
 *
 *   Step 3.  Copy g_spectrum into g_spectrum_modified, then
 *            apply_spectrum_modification(g_spectrum_modified,
 *                                        g_mode, g_cutoff_bin)
 *            This is where the four modes diverge:
 *              MODE_ROUND_TRIP — no-op (verify perfect inversion)
 *              MODE_LOW_PASS   — zero high bins + their twins
 *              MODE_HIGH_PASS  — zero low bins + their twins
 *              MODE_MAG_ONLY   — set X[k].im = 0 for all k
 *              MODE_PHASE_ONLY — divide every X[k] by |X[k]|
 *
 *   Step 4.  compute_idft(g_spectrum_modified, g_reconstruction)
 *            Inverse DFT.  g_reconstruction holds the result of
 *            running the modified spectrum through IDFT.
 *
 *   Step 5.  Compute |X[k]| for the visible spectrum panel and
 *            (for mode 0 only) the round-trip error |y - x| for HUD.
 *
 *   Step 6.  Render — handled by render_frame in §16.
 *
 * The spectrum modification is the entire artistic difference between
 * the modes.  Everything else is identical.  In code (§9):
 *
 *   apply_spectrum_modification(spectrum, mode, cutoff):
 *     switch (mode):
 *       case ROUND_TRIP: do nothing
 *       case LOW_PASS:   zero bins where (k > cutoff && k < N - cutoff)
 *       case HIGH_PASS:  zero bins where (k < cutoff || k > N - cutoff)
 *       case MAG_ONLY:   for each k: X[k] = (|X[k]|, 0)
 *       case PHASE_ONLY: for each k: X[k] = X[k] / |X[k]|
 *
 * Five modes, five tiny case bodies, five different lessons.
 *
 * T11  HOW THE DEMO WORKS
 * ───────────────────────
 * Default state: mode 0 (round-trip), input = Sum sines, auto-sweep
 * cutoff enabled.  HUD shows "round-trip err=~1e-6 [PASS]".  The
 * reconstruction panel matches the input panel exactly.
 *
 * Press 'm' to advance the mode.  Each press changes how the spectrum
 * is modified:
 *   m → mode 1 (low-pass)   the cutoff sweeps; reconstruction smooths
 *   m → mode 2 (high-pass)  the cutoff sweeps; reconstruction edges
 *   m → mode 3 (mag-only)   reconstruction becomes centred blob
 *   m → mode 4 (phase-only) reconstruction looks like input's shape
 *   m → mode 0 (round-trip) (wraps around)
 *
 * Press 's' to cycle the input signal (Sum sines → Square → Sawtooth
 * → Impulse → Noise).  Each signal interacts with each mode
 * differently — try Square + mode 4 for the Oppenheim & Lim demo.
 *
 * Press 'd' for the phase panel (debug overlay).  Shows arg(X[k])
 * for each bin.  In modes 3-4 this overlay tells you what's
 * preserved/discarded.
 *
 * Press 'D' for the bin-info table.  Top 8 bins by magnitude with
 * their (k, |X[k]|, arg) printed numerically.
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
/* §1  config                                                            */
/* ===================================================================== */

#define N                  32
#define N_HALF             (N / 2)
#define RENDER_FPS         30
#define RENDER_TICK_NS     (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 90       /* one full cutoff sweep ≈ 3 seconds  */
#define ARM_TABLE_ROWS      8        /* 'D' overlay shows top N bins      */

enum {
    PAIR_INPUT       = 1,    /* time-domain input wave bars              */
    PAIR_SPEC        = 2,    /* spectrum bars                            */
    PAIR_SPEC_KILL   = 3,    /* spectrum bars zeroed by spectrum_mod      */
    PAIR_RECON       = 4,    /* reconstructed signal bars                */
    PAIR_LABEL       = 5,    /* panel labels                             */
    PAIR_HUD         = 6,    /* HUD top status                           */
    PAIR_HINT        = 7,    /* bottom hint                              */
    PAIR_PHASE_POS   = 8,    /* phase panel positive bars                */
    PAIR_PHASE_NEG   = 9,    /* phase panel negative bars                */
};

/* ===================================================================== */
/* §2  clock                                                             */
/* ===================================================================== */

static long long clock_now_ns(void)
{
    /* Purpose : monotonic clock in ns.  Used by the main loop to
     *           figure out how long to sleep before the next frame. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    /* Block for `ns` nanoseconds.  Without this the main loop would
     * spin a CPU at 100% to redraw 30 times a second. */
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  complex — ComplexNumber + 7 helpers                               */
/* ===================================================================== */
/*
 *  Same shape as fft_helloworld.c §3.  Includes complex_mul because
 *  IDFT multiplies X[k] (complex) by exp(+i * angle) (complex) — a
 *  full complex × complex multiplication.
 *
 *  Why this exists: without it, every complex operation would be
 *  TWO lines of C — one for the real part, one for the imaginary.
 *  With it, every line of the IDFT reads exactly like the math:
 *
 *      ComplexNumber t = complex_mul(X[k], twist);
 *      running_sum     = complex_add(running_sum, t);
 *
 *  rather than four lines of float arithmetic.
 */

typedef struct {
    float re;
    float im;
} ComplexNumber;

static const ComplexNumber complex_zero = { 0.0f, 0.0f };

static inline ComplexNumber complex_make(float real_part, float imag_part)
{
    return (ComplexNumber){ real_part, imag_part };
}

/*
 * complex_from_angle — Euler's formula:  exp(i * theta) = cos + i * sin.
 *   Result always sits ON the unit circle (magnitude 1).  This is the
 *   bridge between an angle and a complex multiplication.
 */
static inline ComplexNumber complex_from_angle(float angle_radians)
{
    return complex_make(cosf(angle_radians), sinf(angle_radians));
}

/*
 * complex_add — vector addition (head-to-tail).
 */
static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b)
{
    return complex_make(a.re + b.re, a.im + b.im);
}

/*
 * complex_sub — vector subtraction.
 */
static inline ComplexNumber complex_sub(ComplexNumber a, ComplexNumber b)
{
    return complex_make(a.re - b.re, a.im - b.im);
}

/*
 * complex_mul — full complex multiplication (rotate-AND-stretch).
 *   Result magnitude = product of magnitudes; angle = sum of angles.
 *   Multiplying by a UNIT-CIRCLE complex (magnitude 1) is pure
 *   rotation by that complex's angle — exactly what the IDFT's
 *   X[k] * exp(+i*angle) does.
 */
static inline ComplexNumber complex_mul(ComplexNumber a, ComplexNumber b)
{
    return complex_make(a.re * b.re - a.im * b.im,
                        a.re * b.im + a.im * b.re);
}

/*
 * complex_scale_by_real — multiply by a real scalar (uniform stretch
 *   without rotation).  Used by the IDFT's 1/N normalisation.
 */
static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z)
{
    return complex_make(scale * z.re, scale * z.im);
}

/*
 * complex_magnitude — length of the arrow:  |z| = sqrt(re^2 + im^2).
 */
static inline float complex_magnitude(ComplexNumber z)
{
    return sqrtf(z.re * z.re + z.im * z.im);
}

/* ===================================================================== */
/* §4  colors                                                            */
/* ===================================================================== */

static void colors_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_INPUT,      51, -1);   /* bright cyan      */
        init_pair(PAIR_SPEC,      154, -1);   /* yellow-green     */
        init_pair(PAIR_SPEC_KILL, 240, -1);   /* dim grey         */
        init_pair(PAIR_RECON,     213, -1);   /* magenta-pink     */
        init_pair(PAIR_LABEL,     244, -1);   /* mid grey         */
        init_pair(PAIR_HUD,       226, -1);   /* bright yellow    */
        init_pair(PAIR_HINT,       51, -1);   /* bright cyan      */
        init_pair(PAIR_PHASE_POS, 213, -1);   /* magenta-pink     */
        init_pair(PAIR_PHASE_NEG, 117, -1);   /* sky blue         */
    } else {
        init_pair(PAIR_INPUT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SPEC,       COLOR_GREEN,   -1);
        init_pair(PAIR_SPEC_KILL,  COLOR_WHITE,   -1);
        init_pair(PAIR_RECON,      COLOR_MAGENTA, -1);
        init_pair(PAIR_LABEL,      COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,        COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,    -1);
        init_pair(PAIR_PHASE_POS,  COLOR_MAGENTA, -1);
        init_pair(PAIR_PHASE_NEG,  COLOR_BLUE,    -1);
    }
}

/* ===================================================================== */
/* §5  dft — forward DFT                                                 */
/* ===================================================================== */
/*
 *  Same algorithm as dft_helloworld.c §5.  We could use the FFT here
 *  but at N = 32 the naive DFT is plenty fast (1024 multiplies/frame)
 *  and the code reads as 1:1 with the math.
 *
 *  Pseudocode:
 *      for each output bin k in [0, N):
 *        running_sum = 0
 *        for each input sample n in [0, N):
 *          twist_angle = -2*pi * k * n / N            ← negative for forward
 *          twist       = exp(i * twist_angle)
 *          twisted     = x[n] * twist                  ← real * complex
 *          running_sum += twisted
 *        X[k] = running_sum
 */
static void compute_dft(const float   *input_signal,
                        ComplexNumber *output_spectrum)
{
    for (int output_bin_index = 0;
         output_bin_index < N;
         output_bin_index++) {

        ComplexNumber running_sum = complex_zero;

        for (int input_sample_index = 0;
             input_sample_index < N;
             input_sample_index++) {

            float twist_angle_radians =
                -2.0f * (float)M_PI
                * (float)output_bin_index
                * (float)input_sample_index
                / (float)N;

            ComplexNumber twist =
                complex_from_angle(twist_angle_radians);
            ComplexNumber twisted =
                complex_scale_by_real(input_signal[input_sample_index],
                                      twist);

            running_sum = complex_add(running_sum, twisted);
        }

        output_spectrum[output_bin_index] = running_sum;
    }
}

/* ===================================================================== */
/* §6  idft — INVERSE DFT (the headline new function, reads as pseudocode) */
/* ===================================================================== */
/*
 *  THE FUNCTION YOU ARE ABOUT TO READ
 *  ──────────────────────────────────
 *  compute_idft() is THIS file's punchline.  Every other line of code
 *  exists to set it up (forward DFT + spectrum_mod), call it, or
 *  visualise its output.  Read this section carefully — everything
 *  else is plumbing around this 25-line core.
 *
 *  WHAT GOES IN, WHAT COMES OUT
 *  ────────────────────────────
 *    INPUT
 *      input_spectrum[0..N-1]    N complex frequency-domain samples
 *
 *    OUTPUT  (caller provides storage; we fill it)
 *      output_signal[0..N-1]     N complex time-domain samples.  For
 *                                 spectra of real input signals (always
 *                                 the case in this demo) the imaginary
 *                                 part is essentially zero (machine
 *                                 epsilon); we display only .re.
 *
 *  THE MATH WE ARE TRANSLATING TO CODE
 *  ───────────────────────────────────
 *    x[n] = (1/N) * sum_{k=0..N-1} X[k] * exp(+i * 2*pi * k * n / N)
 *
 *    Plain English: each output sample x[n] is the sum, over all N
 *    input bins, of (X[k] times a TWIST).  The twist is a unit-circle
 *    complex at angle +2*pi*k*n/N.  Note the +i (not -i — see T2)
 *    and the 1/N normalisation factor (T3 derives why).
 *
 *  PSEUDOCODE  (this maps line-by-line to the code below)
 *  ──────────
 *    for each output sample n in [0, N):                ← outer loop
 *      start with running_sum = 0 + 0i                  ← init
 *      for each input bin k in [0, N):                  ← inner loop
 *        Step 1.  twist_angle = +2*pi * k * n / N       ← +i, NOT -i
 *        Step 2.  twist = exp(i * twist_angle)          ← unit circle
 *        Step 3.  twisted = X[k] * twist                ← complex × complex
 *        Step 4.  running_sum += twisted
 *      Step 5.  output[n] = (1/N) * running_sum         ← normalisation
 *
 *  WHY THIS COMPUTES "THE INVERSE OF THE DFT"
 *  ──────────────────────────────────────────
 *    The forward DFT untwists each input sample by a CW rotation
 *    (exp(-i * angle)) and sums.  The IDFT re-twists each frequency
 *    bin by the SAME rotation but in the CCW direction (exp(+i *
 *    angle)) and sums.  Together they undo each other exactly,
 *    courtesy of the orthogonality property of complex exponentials
 *    (T3 derivation).
 *
 *    The 1/N at the end of IDFT is the constant of integration that
 *    makes the round-trip exact.  Without it, IDFT(DFT(x)) = N*x.
 *
 *  BOUNDARIES & CONTEXT
 *  ────────────────────
 *    - Cost: O(N^2).  At N = 32 that's 1024 inner iterations per
 *      call, totally negligible.  At N = 1024 it's a million per
 *      call — switch to IFFT (signal/fft_helloworld.c).
 *    - Trig calls: 2 per inner iteration (cos + sin via §3
 *      complex_from_angle).  Same as the forward DFT.
 *    - This function does NOT mutate its input array.  It writes
 *      only to output_signal.  No allocation.
 *    - For real-valued signals (always the case here) the output's
 *      imaginary part is essentially zero (< 1e-6).  For arbitrary
 *      complex spectra the output is genuinely complex.
 */

static void compute_idft(const ComplexNumber *input_spectrum,
                         ComplexNumber       *output_signal)
{
    /* The 1/N normalisation factor.  Computed once, applied per
     * output sample.  Made explicit so a reader can see the constant. */
    float normalisation_one_over_N = 1.0f / (float)N;

    /* Pseudocode line: "for each output sample n in [0, N)" */
    for (int output_sample_index = 0;
         output_sample_index < N;
         output_sample_index++) {

        /* Pseudocode line: "start with running_sum = 0 + 0i". */
        ComplexNumber running_sum = complex_zero;

        /* Pseudocode line: "for each input bin k in [0, N)" */
        for (int input_bin_index = 0;
             input_bin_index < N;
             input_bin_index++) {

            /* ── STEP 1 — compute the twist angle ──────────────────
             * Note +2*pi (NOT -2*pi).  This is the SIGN CHANGE that
             * makes IDFT the inverse of DFT — see T2.  The rotation
             * walks CCW (counter-clockwise) instead of CW.
             */
            float twist_angle_radians =
                +2.0f * (float)M_PI
                * (float)input_bin_index
                * (float)output_sample_index
                / (float)N;

            /* ── STEP 2 — twist = exp(i * angle) on the unit circle ──
             * Same Euler-formula construction as the forward DFT.
             * Only the SIGN of the angle differs (CCW vs CW).
             */
            ComplexNumber twist =
                complex_from_angle(twist_angle_radians);

            /* ── STEP 3 — twisted = X[k] * twist ──────────────────
             * Both operands are complex now.  This is a full complex
             * × complex multiply — see §3 complex_mul.
             *
             * In the forward DFT the input was real (the time-domain
             * sample x[n]) so we used scale_by_real.  Here X[k] is
             * itself complex so we need the full multiplication.
             */
            ComplexNumber twisted =
                complex_mul(input_spectrum[input_bin_index], twist);

            /* ── STEP 4 — accumulate ──────────────────────────────
             * Add the twisted bin to the running sum.  By the end of
             * the inner loop we've summed N twisted bins; that sum
             * (after normalisation) is the time-domain sample.
             */
            running_sum = complex_add(running_sum, twisted);
        }

        /* ── STEP 5 — apply the 1/N normalisation ─────────────────
         * The constant of integration that closes the round-trip
         * identity x = IDFT(DFT(x)).  Without it the output would be
         * N times too large.  T3 derives why this is exactly the
         * right factor.
         */
        output_signal[output_sample_index] =
            complex_scale_by_real(normalisation_one_over_N, running_sum);
    }
}

/* ===================================================================== */
/* §7  signal_kinds — enum + names + 5 generators                        */
/* ===================================================================== */

typedef enum {
    SIG_SINES = 0,    /* sum of two cosines                             */
    SIG_SQUARE,       /* periodic square wave                           */
    SIG_SAWTOOTH,     /* periodic sawtooth                              */
    SIG_IMPULSE,      /* one tall sample at the centre                  */
    SIG_NOISE,        /* repeatable pseudo-random                       */
    SIG_COUNT
} SignalKind;

static const char *signal_name[SIG_COUNT] = {
    "Sum sines", "Square   ", "Sawtooth ", "Impulse  ", "Noise    "
};

/*
 * generate_signal — fill `output_signal[0..N-1]` with the chosen
 *   test signal.  All signals are bounded by ±1 so the panel
 *   auto-scaling is consistent.
 *
 * Each case is one or two lines; the SIGNAL choice is what makes
 * each mode demo look different.  Square + mode 4 is the
 * Oppenheim & Lim "phase wins" demo; Impulse + mode 0 is the
 * minimal round-trip test; etc.
 */
static void generate_signal(SignalKind kind, float *output_signal)
{
    switch (kind) {
        case SIG_SINES: {
            /* Two cosines added: 3 cycles + 7 cycles in the buffer.
             * Simple, clean spectrum — good default. */
            for (int n = 0; n < N; n++) {
                float t = (float)n / (float)N;
                output_signal[n] =
                    0.6f * cosf(2.0f * (float)M_PI * 3.0f * t)
                  + 0.4f * cosf(2.0f * (float)M_PI * 7.0f * t);
            }
            break;
        }
        case SIG_SQUARE: {
            /* Periodic square wave at 4 cycles per buffer.  Sharp
             * vertical edges → many high harmonics → showcases
             * filtering modes 1-2 dramatically. */
            for (int n = 0; n < N; n++) {
                float t = (float)n / (float)N;
                float arg = 2.0f * (float)M_PI * 4.0f * t;
                output_signal[n] = sinf(arg) >= 0.0f ? 0.7f : -0.7f;
            }
            break;
        }
        case SIG_SAWTOOTH: {
            /* Linear ramp repeated 4 times across the buffer.  Like
             * square but with all harmonics (not just odd). */
            for (int n = 0; n < N; n++) {
                float t = (float)n / (float)N;
                float u = 4.0f * t - floorf(4.0f * t);
                output_signal[n] = 1.4f * u - 0.7f;
            }
            break;
        }
        case SIG_IMPULSE: {
            /* Zero everywhere except a single 1 at the centre.  The
             * IDFT of an impulse spectrum is a sinc; the DFT of a
             * time-domain impulse is a flat spectrum (mode 0
             * round-trips perfectly). */
            for (int n = 0; n < N; n++) output_signal[n] = 0.0f;
            output_signal[N / 2] = 1.0f;
            break;
        }
        case SIG_NOISE: {
            /* Repeatable pseudo-random using a linear congruential
             * generator with seed 1.  Same noise every frame so the
             * mode demos are stable. */
            unsigned int seed = 1;
            for (int n = 0; n < N; n++) {
                seed = seed * 1664525u + 1013904223u;
                float r = (float)(seed >> 16) / 65535.0f;
                output_signal[n] = (r - 0.5f) * 1.6f;
            }
            break;
        }
        default: break;
    }
}

/* ===================================================================== */
/* §8  mode_kinds — enum + display names                                 */
/* ===================================================================== */

typedef enum {
    MODE_ROUND_TRIP = 0,    /* y = IDFT(DFT(x)) — perfect inversion test */
    MODE_LOW_PASS,          /* zero high bins → smoothing                */
    MODE_HIGH_PASS,         /* zero low bins → edges only                 */
    MODE_MAG_ONLY,          /* discard phase → centred blob               */
    MODE_PHASE_ONLY,        /* discard magnitude → original SHAPE         */
    MODE_COUNT
} Mode;

static const char *mode_name[MODE_COUNT] = {
    "round-trip ",
    "low-pass   ",
    "high-pass  ",
    "mag-only   ",
    "phase-only "
};

/* ===================================================================== */
/* §9  spectrum_mod — modes 1-4 dispatched (mode 0 = no-op)              */
/* ===================================================================== */
/*
 *  Each mode is one switch case below.  The ENTIRE artistic
 *  difference between the modes lives here; everything else is
 *  identical.  Five cases, five different lessons.
 *
 *  Modes 1 and 2 (filters) preserve CONJUGATE SYMMETRY by zeroing
 *  X[k] and X[N - k] together.  Without this, the IDFT output would
 *  have a non-trivial imaginary component — i.e., the filtered
 *  signal would not be real.  See T6.
 *
 *  Modes 3 and 4 (mag-only / phase-only) deliberately break that
 *  symmetry — but the result is still real (or close to it) for
 *  symmetric signals.  Imag part is small machine epsilon.
 */

/* Kill mask — true if the spectrum-mod step zeroed out this bin.
 * Used by the spectrum panel to draw killed bins in dim grey,
 * visualising the filter mask. */
static bool g_bin_killed[N];

static void apply_spectrum_modification(ComplexNumber *spectrum,
                                        Mode mode, int cutoff_bin)
{
    /* Reset the kill mask. */
    for (int k = 0; k < N; k++) g_bin_killed[k] = false;

    switch (mode) {
        case MODE_ROUND_TRIP: {
            /* No modification — verify perfect round-trip identity.
             * After this, IDFT(spectrum) should equal the original
             * input within float epsilon (~1e-6). */
            break;
        }

        case MODE_LOW_PASS: {
            /* Zero out X[k] for k in (cutoff, N - cutoff).  Keep
             * low frequencies (k <= cutoff) AND their conjugate
             * twins (k >= N - cutoff) so the IDFT result is real.
             * See T6 for why both halves must be zeroed together. */
            for (int k = 0; k < N; k++) {
                if (k > cutoff_bin && k < N - cutoff_bin) {
                    spectrum[k] = complex_zero;
                    g_bin_killed[k] = true;
                }
            }
            break;
        }

        case MODE_HIGH_PASS: {
            /* Zero out X[k] for k in [0, cutoff) and (N - cutoff, N).
             * Keep high frequencies (cutoff <= k <= N - cutoff). */
            for (int k = 0; k < N; k++) {
                if (k < cutoff_bin || k > N - cutoff_bin) {
                    spectrum[k] = complex_zero;
                    g_bin_killed[k] = true;
                }
            }
            break;
        }

        case MODE_MAG_ONLY: {
            /* Force every X[k] to (|X[k]|, 0).  Phase set to 0;
             * magnitude preserved.  Reconstruction is symmetric
             * around index N/2 — see T7. */
            for (int k = 0; k < N; k++) {
                float magnitude = complex_magnitude(spectrum[k]);
                spectrum[k] = complex_make(magnitude, 0.0f);
            }
            break;
        }

        case MODE_PHASE_ONLY: {
            /* Force every X[k] to a unit-magnitude complex with the
             * same phase.  Magnitude set to 1 (for non-zero bins);
             * phase preserved.  Reconstruction has the original's
             * SHAPE but normalised amplitude — see T8 (Oppenheim &
             * Lim 1981). */
            for (int k = 0; k < N; k++) {
                float magnitude = complex_magnitude(spectrum[k]);
                if (magnitude > 1e-6f) {
                    spectrum[k] = complex_scale_by_real(
                        1.0f / magnitude, spectrum[k]);
                } else {
                    spectrum[k] = complex_zero;
                }
            }
            break;
        }

        default: break;
    }
}

/* ===================================================================== */
/* §10  scene_state                                                      */
/* ===================================================================== */

static float         g_input_signal[N];
static ComplexNumber g_spectrum[N];                /* DFT(input)         */
static ComplexNumber g_spectrum_modified[N];       /* after spectrum_mod */
static ComplexNumber g_reconstruction[N];          /* IDFT of modified   */
static float         g_magnitude[N_HALF + 1];      /* for the panel       */
static float         g_magnitude_peak = 1.0f;
static float         g_round_trip_error = 0.0f;
static float         g_signal_peak = 1.0f;
static float         g_recon_peak = 1.0f;

static SignalKind  g_signal_kind            = SIG_SINES;
static Mode        g_mode                   = MODE_ROUND_TRIP;
static int         g_cutoff_bin             = 4;     /* low/high-pass     */
static bool        g_simulation_paused      = false;
static bool        g_auto_sweep_cutoff      = true;
static float       g_sweep_phase_radians    = 0.0f;

/* Debug overlay toggles. */
static bool        g_show_phase_panel       = false;
static bool        g_show_arm_table         = false;

static void scene_reset(void)
{
    g_signal_kind            = SIG_SINES;
    g_mode                   = MODE_ROUND_TRIP;
    g_cutoff_bin             = 4;
    g_simulation_paused      = false;
    g_auto_sweep_cutoff      = true;
    g_sweep_phase_radians    = 0.0f;
}

/* ===================================================================== */
/* §11  scene_tick — per-frame update                                    */
/* ===================================================================== */
/*
 *  Six numbered steps that match T10 of the GUIDED TUTORIAL.
 */
static void scene_tick(void)
{
    if (g_simulation_paused) return;

    /* Auto-sweep the cutoff in modes 1 / 2.  sin(sweep_phase) gives
     * smooth there-and-back motion between cutoff = 1 and N/2 - 1. */
    bool mode_uses_cutoff =
        (g_mode == MODE_LOW_PASS) || (g_mode == MODE_HIGH_PASS);
    if (g_auto_sweep_cutoff && mode_uses_cutoff) {
        g_sweep_phase_radians +=
            2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
        if (g_sweep_phase_radians > 2.0f * (float)M_PI)
            g_sweep_phase_radians -= 2.0f * (float)M_PI;
        float s = (sinf(g_sweep_phase_radians) + 1.0f) * 0.5f;
        g_cutoff_bin = 1 + (int)(s * ((float)N * 0.5f - 2.0f) + 0.5f);
    }

    /* ── Step 1.  generate input signal ───────────────────────── */
    generate_signal(g_signal_kind, g_input_signal);

    /* ── Step 2.  forward DFT ──────────────────────────────────── */
    compute_dft(g_input_signal, g_spectrum);

    /* ── Step 3.  copy + apply spectrum modification ─────────── */
    for (int k = 0; k < N; k++) g_spectrum_modified[k] = g_spectrum[k];
    apply_spectrum_modification(g_spectrum_modified, g_mode, g_cutoff_bin);

    /* ── Step 4.  inverse DFT (THE function in this file) ─────── */
    compute_idft(g_spectrum_modified, g_reconstruction);

    /* ── Step 5.  magnitude spectrum + peaks for autoscale ────── */
    g_magnitude_peak = 1e-6f;
    for (int k = 0; k <= N_HALF; k++) {
        g_magnitude[k] = complex_magnitude(g_spectrum_modified[k]);
        if (g_magnitude[k] > g_magnitude_peak)
            g_magnitude_peak = g_magnitude[k];
    }
    g_signal_peak = 1e-6f;
    for (int n = 0; n < N; n++) {
        float a = fabsf(g_input_signal[n]);
        if (a > g_signal_peak) g_signal_peak = a;
    }
    g_recon_peak = 1e-6f;
    for (int n = 0; n < N; n++) {
        float a = fabsf(g_reconstruction[n].re);
        if (a > g_recon_peak) g_recon_peak = a;
    }

    /* ── Step 6.  round-trip error (mode 0 only) for HUD ─────── */
    g_round_trip_error = 0.0f;
    if (g_mode == MODE_ROUND_TRIP) {
        for (int n = 0; n < N; n++) {
            float diff = g_reconstruction[n].re - g_input_signal[n];
            float err  = fabsf(diff);
            if (err > g_round_trip_error) g_round_trip_error = err;
        }
    }
}

/* ===================================================================== */
/* §12  scene_input — handle keys                                        */
/* ===================================================================== */

static void scene_cycle_mode(int direction)
{
    g_mode = (Mode)((g_mode + direction + MODE_COUNT) % MODE_COUNT);
}

static void scene_cycle_signal(int direction)
{
    g_signal_kind = (SignalKind)((g_signal_kind + direction + SIG_COUNT)
                                 % SIG_COUNT);
}

static void scene_adjust_cutoff(int delta)
{
    if (g_auto_sweep_cutoff) return;
    g_cutoff_bin += delta;
    if (g_cutoff_bin < 1) g_cutoff_bin = 1;
    if (g_cutoff_bin > N_HALF - 1) g_cutoff_bin = N_HALF - 1;
}

/* ===================================================================== */
/* §13  draw_bar                                                         */
/* ===================================================================== */

static void draw_bar(int column, int baseline_row, int bar_height_cells,
                     bool growing_upward, int colour_pair)
{
    if (bar_height_cells <= 0) return;
    attron(COLOR_PAIR(colour_pair) | A_BOLD);
    for (int dy = 0; dy < bar_height_cells; dy++) {
        int row = growing_upward ? (baseline_row - dy)
                                 : (baseline_row + dy + 1);
        if (row < 0 || row >= LINES) continue;
        if (column < 0 || column >= COLS)  continue;
        mvaddch(row, column, growing_upward ? '|' : '.');
    }
    attroff(COLOR_PAIR(colour_pair) | A_BOLD);
}

/* ===================================================================== */
/* §14  draw_signals — input + reconstruction panels                     */
/* ===================================================================== */

static void draw_signal_panel(const float *signal, int top_row,
                              int height_rows, float peak,
                              int colour_pair)
{
    /* Render a real-valued signal as ± vertical bars from the
     * panel midline.  Positive grows up, negative grows down. */
    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    int spacing = (COLS / N >= 2) ? 2 : 1;
    int columns = (COLS / spacing < N) ? COLS / spacing : N;

    for (int n = 0; n < columns; n++) {
        float v   = signal[n] / (peak + 1e-6f);
        bool  pos = (v >= 0.0f);
        int   h   = (int)(fabsf(v) * (float)half_height + 0.5f);
        for (int s = 0; s < spacing; s++)
            draw_bar(n * spacing + s, midline_row, h, pos, colour_pair);
    }
}

static void draw_complex_real_panel(const ComplexNumber *signal,
                                    int top_row, int height_rows,
                                    float peak, int colour_pair)
{
    /* Like draw_signal_panel but reads .re from a complex array.
     * For real inputs (always the case here) the .im part is
     * essentially zero (machine epsilon) and we display only .re. */
    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    int spacing = (COLS / N >= 2) ? 2 : 1;
    int columns = (COLS / spacing < N) ? COLS / spacing : N;

    for (int n = 0; n < columns; n++) {
        float v   = signal[n].re / (peak + 1e-6f);
        bool  pos = (v >= 0.0f);
        int   h   = (int)(fabsf(v) * (float)half_height + 0.5f);
        for (int s = 0; s < spacing; s++)
            draw_bar(n * spacing + s, midline_row, h, pos, colour_pair);
    }
}

/* ===================================================================== */
/* §15  draw_spectrum — magnitude + phase + debug table                  */
/* ===================================================================== */

static void draw_spectrum_panel(int top_row, int height_rows)
{
    /* Bins 0..N/2 as upward bars from panel bottom.  Killed bins
     * (from spectrum_mod) drawn in dim grey to visualise the mask. */
    int baseline_row = top_row + height_rows - 1;
    int bin_w        = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
    int max_bins     = COLS / bin_w;
    if (max_bins > N_HALF + 1) max_bins = N_HALF + 1;

    for (int k = 0; k < max_bins; k++) {
        float v = g_magnitude[k] / (g_magnitude_peak + 1e-6f);
        int   h = (int)(v * (float)height_rows + 0.5f);
        int   pair = g_bin_killed[k] ? PAIR_SPEC_KILL : PAIR_SPEC;

        for (int bx = 0; bx < bin_w - 1; bx++)
            draw_bar(k * bin_w + bx, baseline_row, h, true, pair);
    }

    /* Cutoff marker (filter modes only): vertical ":" at the cutoff
     * column. */
    bool mode_uses_cutoff =
        (g_mode == MODE_LOW_PASS) || (g_mode == MODE_HIGH_PASS);
    if (mode_uses_cutoff && g_cutoff_bin <= max_bins) {
        attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
        for (int dy = 0; dy < height_rows; dy++) {
            int row = top_row + dy;
            int col = g_cutoff_bin * bin_w + bin_w / 2;
            if (col < COLS && row < LINES)
                mvaddch(row, col, ':');
        }
        attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    }
}

static void draw_phase_panel(int top_row, int height_rows)
{
    /* Phase panel ('d' overlay).  Shows arg(X[k]) for each bin
     * (after spectrum_mod) as ± bars from a midline.  Magenta-pink
     * = positive phase; sky blue = negative phase.  Skipped for
     * near-zero magnitude bins (would be noise). */
    if (height_rows < 3) return;

    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    int bin_w    = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
    int max_bins = COLS / bin_w;
    if (max_bins > N_HALF + 1) max_bins = N_HALF + 1;

    for (int bin_index = 0; bin_index < max_bins; bin_index++) {
        ComplexNumber z = g_spectrum_modified[bin_index];
        if (complex_magnitude(z) < 1e-3f) continue;

        float phase_radians = atan2f(z.im, z.re);
        float fraction      = phase_radians / (float)M_PI;
        bool  pos           = (phase_radians >= 0.0f);
        int   h             = (int)(fabsf(fraction)
                                  * (float)half_height + 0.5f);

        for (int bx = 0; bx < bin_w - 1; bx++)
            draw_bar(bin_index * bin_w + bx, midline_row,
                     h, pos,
                     pos ? PAIR_PHASE_POS : PAIR_PHASE_NEG);
    }

    /* Midline marker. */
    attron(COLOR_PAIR(PAIR_LABEL));
    for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
        mvaddch(midline_row, x, '-');
    attroff(COLOR_PAIR(PAIR_LABEL));
}

static void draw_arm_table_overlay(void)
{
    /* 'D' overlay.  Selection-sort the bins by descending magnitude
     * and print the top ARM_TABLE_ROWS as a numeric table. */
    if (!g_show_arm_table) return;

    int sorted_bin_indices[N_HALF + 1];
    for (int i = 0; i <= N_HALF; i++) sorted_bin_indices[i] = i;
    for (int i = 0; i < ARM_TABLE_ROWS && i <= N_HALF; i++) {
        int max_at = i;
        for (int j = i + 1; j <= N_HALF; j++)
            if (g_magnitude[sorted_bin_indices[j]]
              > g_magnitude[sorted_bin_indices[max_at]])
                max_at = j;
        int tmp                    = sorted_bin_indices[i];
        sorted_bin_indices[i]      = sorted_bin_indices[max_at];
        sorted_bin_indices[max_at] = tmp;
    }

    int x = 2, y = 2;
    if (y + ARM_TABLE_ROWS + 1 >= LINES - 1) return;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, " bin   |X[k]|     arg(rad)");
    attroff(COLOR_PAIR(PAIR_HINT));

    for (int row = 0; row < ARM_TABLE_ROWS && row <= N_HALF; row++) {
        int bin_index   = sorted_bin_indices[row];
        ComplexNumber z = g_spectrum_modified[bin_index];
        float phase     = atan2f(z.im, z.re);
        attron(COLOR_PAIR(PAIR_SPEC) | A_BOLD);
        mvprintw(y + 1 + row, x,
                 "  %2d   %7.3f   %+6.3f",
                 bin_index, (double)g_magnitude[bin_index], (double)phase);
        attroff(COLOR_PAIR(PAIR_SPEC) | A_BOLD);
    }
}

/* ===================================================================== */
/* §16  hud — status + hint + frame composer                             */
/* ===================================================================== */

static void draw_hud(void)
{
    char status[200];
    bool mode_uses_cutoff =
        (g_mode == MODE_LOW_PASS) || (g_mode == MODE_HIGH_PASS);

    if (g_mode == MODE_ROUND_TRIP) {
        snprintf(status, sizeof status,
                 " IDFT helloworld  N=%d  signal:%s  mode:%s  "
                 "round-trip err=%.1e %s  %s ",
                 N, signal_name[g_signal_kind],
                 mode_name[g_mode],
                 (double)g_round_trip_error,
                 (g_round_trip_error < 1e-4f) ? "[PASS]" : "[FAIL]",
                 g_simulation_paused  ? "PAUSED" : "      ");
    } else if (mode_uses_cutoff) {
        snprintf(status, sizeof status,
                 " IDFT helloworld  N=%d  signal:%s  mode:%s  "
                 "cutoff=%2d  %s  %s ",
                 N, signal_name[g_signal_kind],
                 mode_name[g_mode], g_cutoff_bin,
                 g_auto_sweep_cutoff ? "AUTO  " : "MANUAL",
                 g_simulation_paused ? "PAUSED" : "      ");
    } else {
        snprintf(status, sizeof status,
                 " IDFT helloworld  N=%d  signal:%s  mode:%s  %s ",
                 N, signal_name[g_signal_kind],
                 mode_name[g_mode],
                 g_simulation_paused  ? "PAUSED" : "      ");
    }

    int x = COLS - (int)strlen(status);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hint(void)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(LINES - 1, 0,
             " q:quit  spc:pause  m/M:mode  s/S:signal  a:auto/manual "
             " +/-:cutoff  d:phase  D:bins  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void)
{
    erase();

    /* Layout: 1 hud + 3 labels + 3 panels + 1 hint = 5 reserved.
     * Phase panel "steals" some vertical space when toggled on. */
    int rows_for_panels = LINES - 5;
    if (rows_for_panels < 9) rows_for_panels = 9;
    int phase_h = g_show_phase_panel ? rows_for_panels / 5 : 0;
    int main_h  = rows_for_panels - phase_h;
    int input_h = main_h / 3;
    int spec_h  = main_h / 3;
    int recon_h = main_h - input_h - spec_h;

    int input_label_row = 1;
    int input_top_row   = 2;
    int spec_label_row  = 2 + input_h;
    int spec_top_row    = 3 + input_h;
    int recon_label_row = 3 + input_h + spec_h;
    int recon_top_row   = 4 + input_h + spec_h;
    int phase_top_row   = 4 + input_h + spec_h + recon_h;

    /* Panel labels. */
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(input_label_row, 0, "Input x[n]");
    mvprintw(spec_label_row,  0,
             "Spectrum |X[k]|  (dim grey = killed by mode, ':' = cutoff)");
    mvprintw(recon_label_row, 0,
             "Reconstruction y[n] = IDFT(modified X)");
    if (g_show_phase_panel)
        mvprintw(phase_top_row - 1, 0,
                 "Phase arg(X[k])  (magenta = +, sky = -)");
    attroff(COLOR_PAIR(PAIR_LABEL));

    /* Panels. */
    draw_signal_panel(g_input_signal, input_top_row, input_h,
                      g_signal_peak, PAIR_INPUT);
    draw_spectrum_panel(spec_top_row, spec_h);
    draw_complex_real_panel(g_reconstruction, recon_top_row, recon_h,
                            g_recon_peak, PAIR_RECON);
    if (g_show_phase_panel)
        draw_phase_panel(phase_top_row, phase_h);

    /* Debug overlays (drawn over panels but under HUD). */
    draw_arm_table_overlay();

    /* HUD always last. */
    draw_hud();
    draw_hint();

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §17  app — signal handlers + main loop + key dispatch                 */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    /* Async-signal-safe: only set flags; main loop handles them. */
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit    = 1;
}

static void cleanup_screen(void) { endwin(); }

static bool app_handle_key(int ch)
{
    switch (ch) {
        case 'q': case 'Q': case 27:  return true;
        case ' ':                     g_simulation_paused = !g_simulation_paused; break;
        case 'm':                     scene_cycle_mode(+1); break;
        case 'M':                     scene_cycle_mode(-1); break;
        case 's':                     scene_cycle_signal(+1); break;
        case 'S':                     scene_cycle_signal(-1); break;
        case 'a': case 'A':           g_auto_sweep_cutoff = !g_auto_sweep_cutoff; break;
        case '+': case '=': case '.': scene_adjust_cutoff(+1); break;
        case '-':           case ',': scene_adjust_cutoff(-1); break;
        case 'd':                     g_show_phase_panel = !g_show_phase_panel; break;
        case 'D':                     g_show_arm_table   = !g_show_arm_table;   break;
        case 'r': case 'R':           scene_reset(); break;
        default: break;
    }
    return false;
}

int main(void)
{
    atexit(cleanup_screen);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
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
            if (app_handle_key(ch)) { g_should_quit = 1; break; }
        }

        long long now = clock_now_ns();
        if (now >= next_frame_ns) {
            scene_tick();
            render_frame();
            next_frame_ns += RENDER_TICK_NS;
            if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
                next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
        } else {
            clock_sleep_ns(next_frame_ns - now);
        }
    }

    return 0;
}
