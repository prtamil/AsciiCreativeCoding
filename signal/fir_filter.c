/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fir_filter.c — animated, minimum-viable Finite Impulse Response
 *                filter demo.  Build a low-pass, high-pass, or band-
 *                pass filter from a windowed-sinc impulse response,
 *                apply it to a multi-tone test signal via
 *                convolution, and watch the filtered output emerge
 *                live.  Every function reads as pseudocode so the
 *                math and the C are line-for-line identical.
 *
 * DEMO
 *   Four stacked panels.
 *     TOP        input signal x[n]   (multi-tone, chirp, square, ...)
 *     2nd        filter impulse response h[i]   (the K filter taps,
 *                drawn centered, magenta)
 *     3rd        filter magnitude response |H[k]|   (gold bars,
 *                yellow ':' at the cutoff position)
 *     BOTTOM     filtered output y[n] = (h * x)[n]   (yellow-green)
 *
 *   Press 'f' to cycle filter type (low-pass / high-pass / band-pass),
 *   'w' to cycle the window function applied to the sinc (Rectangular
 *   / Hann / Hamming / Blackman), 's' to cycle input signal kinds.
 *   Press '+' / '-' to change the filter cutoff (auto-sweep on by
 *   default; press 'a' to switch to manual).
 *
 *   The FIR filter is BUILT inside the program, not picked from a
 *   table.  The recipe (windowed-sinc design):
 *     1. Take the IDEAL impulse response of the chosen filter (a
 *        sinc function for low-pass).
 *     2. TRUNCATE it to K = 21 taps symmetric around the center.
 *     3. Apply SPECTRAL INVERSION (low-pass → high-pass) or
 *        SPECTRAL DIFFERENCE (low-pass → band-pass) if requested.
 *     4. MULTIPLY by a window function that tapers the truncation
 *        edges to zero (Hann, Hamming, Blackman).
 *
 *   Two debug overlays:
 *     'd' shows the PHASE RESPONSE arg(H[k]) as a fifth panel.
 *         For these symmetric kernels the phase is LINEAR in k —
 *         that's the famous "linear phase FIR" property that makes
 *         FIR filters preferable to IIR for many audio applications.
 *     'D' shows the kernel as a numeric table (all K weights as
 *         floats).  Useful for understanding what the active filter
 *         actually IS and for copying the design into other code.
 *
 *   The headline lesson: filter design is a RECIPE, not a lookup
 *   table.  Three lines of math (sinc + truncation + window) buys you
 *   any low-pass, high-pass, or band-pass filter you can describe.
 *   Apply via convolution.  Done.
 *
 * Study alongside
 *   signal/convolution_helloworld.c    The "slide kernel, multiply,
 *                                       sum" operation that this file
 *                                       runs to apply the filter.
 *                                       Read it first.
 *   signal/idft_helloworld.c           Filtering by zeroing spectrum
 *                                       bins.  Same effect, different
 *                                       implementation.  T9 derives
 *                                       the convolution-multiplication
 *                                       duality that connects them.
 *   signal/dft_helloworld.c            The DFT we use to compute the
 *                                       magnitude (and phase) response.
 *   signal/fft_vis.c                   Where window functions are
 *                                       introduced for FFT analysis.
 *                                       Same windows used here.
 *
 * Section map
 *   §1   config            constants
 *   §2   clock             monotonic-ns timer + sleep
 *   §3   complex           ComplexNumber + 6 helpers (for the DFT)
 *   §4   colors            palette pairs (HUD/HINT theme-invariant)
 *   §5   sinc_helper       the sin(pi*x)/(pi*x) helper alone
 *   §6   windows           4 window functions (Rect / Hann / Hamming / Blackman)
 *   §7   filter_design     low-pass sinc, then design_filter (the recipe)
 *   §8   convolve          apply filter to signal
 *   §9   dft_response      DFT of zero-padded filter for |H[k]| + arg(H[k])
 *   §10  signals           input signal generators
 *   §11  scene_state       per-frame state + reset
 *   §12  scene_tick        per-frame update orchestrator
 *   §13  scene_input       handle keys
 *   §14  draw_bar          vertical-bar primitive
 *   §15  draw_signal       signal panel renderer
 *   §16  draw_filter       impulse + magnitude + phase response panels
 *   §17  draw_debug        'D' kernel numeric table
 *   §18  hud               status top-right + hint bottom + frame composer
 *   §19  app               signal handlers + main loop + key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   f / F            next / previous filter type
 *   w / W            next / previous window function
 *   s / S            next / previous input signal
 *   a                toggle auto-sweep cutoff
 *   + / -            (manual) change cutoff by 1 bin
 *   , / .            (manual) change cutoff by 1 bin (alt keys)
 *   d                toggle phase response panel (debug)
 *   D                toggle kernel numeric table (debug)
 *   r                reset to defaults
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/fir_filter.c \
 *       -o fir_filter -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no FIR background:
 *     1. CONCEPTS — the elevator pitch with references.
 *     2. MENTAL MODEL — analogies, diagrams, the windowed-sinc recipe.
 *     3. GUIDED TUTORIAL T1..T11 — eleven mini-lessons that derive
 *        windowed-sinc filter design from first principles.
 *     4. §1 config — the knobs.
 *     5. §5 sinc_helper → §6 windows → §7 filter_design.  THE three
 *        sections that build the filter.  §7 is the headline.
 *     6. §8 convolve — applies the filter to the input.  Same code
 *        as convolution_helloworld.c §8.
 *     7. §9 dft_response — runs DFT on the kernel for the
 *        magnitude/phase response panels.
 *     8. §10..§17 — scene + rendering plumbing.
 *     9. §18..§19 — main loop + key dispatch.
 *
 *   Reader who already knows FIR design:
 *     §1 config, §7 filter_design, §8 convolve, §12 scene_tick.
 *     The rest is ncurses scaffolding.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     g_input_signal[N]           the source signal, real-valued
 *     g_kernel_taps[K]            the K filter taps (the impulse response)
 *     g_output_signal[N]          the convolved (filtered) result
 *     g_magnitude_response[N/2+1] |H[k]| computed from DFT of kernel
 *     g_phase_response[N/2+1]     arg(H[k]) (debug overlay)
 *     output_position, n          outer-loop counter (which output sample)
 *     kernel_tap_index, i         inner-loop counter (which kernel tap)
 *     weighted_sum                inner-loop accumulator
 *     cutoff_bin, fc_bin          cutoff frequency in bins (0..N/2)
 *     cutoff_normalised, fc       cutoff in cycles per sample (= bin / N)
 *     center                      (K - 1) / 2, the kernel's centre tap index
 *
 *   Inside tight loops single-letter counters (`n`, `i`, `k`) stay
 *   short where context is one line away.
 *
 * Background you NEED
 *   - Convolution at the level of "slide kernel, multiply, sum".
 *     See signal/convolution_helloworld.c CONCEPTS for a 2-minute
 *     intro.  This file APPLIES filters via convolution.
 *   - Basic complex arithmetic (we use ComplexNumber for the DFT
 *     that computes the magnitude/phase response panels).
 *   - The DFT formula at the level of "X[k] is a weighted sum of
 *     inputs".  Optional — only relevant for the response panels.
 *
 * Background you do NOT need
 *   - The Fast Fourier Transform.  We use the textbook O(N^2) DFT
 *     for the response panels because N = 64 is small.
 *   - IIR filter theory, z-transforms, transfer functions.  All
 *     covered in DSP textbooks; not needed for FIR design.
 *   - Continuous calculus.  Everything is finite sums.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * What is an FIR filter, in one sentence?
 *   A short array of weights (the impulse response) that you
 *   convolve with your input signal to remove unwanted frequencies.
 *
 * What does "FIR" mean?
 *   FINITE IMPULSE RESPONSE.  When you feed the filter a single
 *   impulse (a 1 at one position, zeros elsewhere), the output is
 *   non-zero for a FINITE number of samples — exactly K of them
 *   (the K taps of the kernel).  After K samples the output is
 *   permanently zero.  This is the defining property — and what
 *   distinguishes FIR from IIR (Infinite Impulse Response) filters,
 *   whose impulse responses ring forever.
 *
 *   Practically: FIR filters are always STABLE (no feedback to
 *   blow up), have LINEAR PHASE if the kernel is symmetric (no
 *   group-delay distortion), and are easy to design.  Trade-off:
 *   they need MORE taps than IIR for the same frequency selectivity.
 *
 * Algorithm
 *   FIR filtering is convolution: y[n] = sum_i h[i] * x[n + i].
 *   The impulse response h[i] is short (K = 21 taps in this demo)
 *   so direct convolution is cheap (O(N * K) = ~1300 multiplies
 *   per frame at N = 64).
 *
 *   The KEY question is: HOW DO YOU CHOOSE h[i]?  This is where
 *   filter DESIGN comes in.  We use the WINDOWED-SINC technique:
 *     1. The IDEAL impulse response of a low-pass filter with
 *        cutoff fc cycles/sample is h_ideal[i] = 2*fc * sinc(2*fc*
 *        (i - center)).  The catch: this ideal h has INFINITE
 *        length.
 *     2. TRUNCATE it to K samples symmetric around the center.
 *     3. MULTIPLY by a WINDOW function that tapers the truncation
 *        edges to zero (Hann, Hamming, Blackman).  This trades a
 *        slightly wider transition band for much less ringing.
 *
 *   High-pass and band-pass filters are derived from low-pass by
 *   SPECTRAL INVERSION (high-pass = identity - low-pass) or
 *   SPECTRAL DIFFERENCE (band-pass = low-pass(f_high) -
 *   low-pass(f_low)).  T5 / T6 derive these.
 *
 * Math
 *   sinc(x) = sin(pi*x) / (pi*x), with sinc(0) = 1.
 *
 *   Ideal low-pass impulse response (centered, cutoff fc in cycles/sample):
 *     h_lp[i] = 2 * fc * sinc(2 * fc * (i - center))     for all i
 *
 *   Window functions (applied to truncated h):
 *     Rectangular  w[i] = 1
 *     Hann         w[i] = 0.5  - 0.5  * cos(2*pi*i/(K-1))
 *     Hamming      w[i] = 0.54 - 0.46 * cos(2*pi*i/(K-1))
 *     Blackman     w[i] = 0.42 - 0.5  * cos(2*pi*i/(K-1)) +
 *                          0.08 * cos(4*pi*i/(K-1))
 *
 *   Spectral inversion (low-pass → high-pass):
 *     h_hp[i] = identity[i] - h_lp[i]
 *     where identity[i] = 1 at i = center, 0 elsewhere
 *
 *   Spectral difference (low-pass → band-pass):
 *     h_bp[i] = h_lp(f_high)[i] - h_lp(f_low)[i]
 *
 *   Convolution (the filter operation):
 *     y[n] = sum_{i=0..K-1}  h[i] * x[n + i]
 *
 *   Magnitude response (for visualization):
 *     pad h with zeros to length N
 *     H = DFT(h_padded)
 *     |H[k]| = sqrt(re^2 + im^2)
 *
 * Convolution-multiplication duality
 *   Convolving with h in the time domain is the same as multiplying
 *   by H = DFT(h) in the frequency domain.  signal/idft_helloworld.c
 *   demonstrates the spectrum-domain version (zero out bins to
 *   filter).  This file does the time-domain version (convolve with
 *   the designed kernel).  Same effect, two implementations.
 *
 * Performance
 *   Filter design: O(K) = 21 multiplies per filter recompute.
 *   Convolution: O(N * K) = 64 * 21 = 1344 multiplies per frame.
 *   DFT for response: O(N * N) since we zero-pad h to length N and
 *   DFT.  All trivial at this size.
 *
 *   For long signals + long kernels (audio at K = 1024+), direct
 *   convolution becomes slow and you switch to FFT-based filtering
 *   (multiply spectra) at O(N log N).
 *
 * References
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapters 14-16.  Free online.  The most
 *     accessible introduction to FIR filter design.
 *   Oppenheim, A. V. & Schafer, R. W., "Discrete-Time Signal
 *     Processing", Chapter 7 — the textbook reference.
 *   Harris, F. J. (1978), "On the use of windows for harmonic
 *     analysis with the discrete Fourier transform", Proc. IEEE
 *     66(1).  The authoritative reference on window functions.
 *   https://en.wikipedia.org/wiki/Finite_impulse_response
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   To remove unwanted frequencies from a signal, you need an
 *   "impulse response" h that, when convolved with the signal,
 *   passes the frequencies you want and rejects the ones you don't.
 *
 *   Filter DESIGN is the question of "which weights h[0..K-1] do I
 *   pick?".  The answer for FIR filters is the WINDOWED-SINC RECIPE:
 *   start with the ideal infinite sinc, truncate to K samples, then
 *   taper the truncated edges with a window function.
 *
 *   Filter APPLICATION is then trivial: just convolve.
 *
 *   This file shows all four pieces of the picture at once: input
 *   (top), kernel (middle), magnitude response (gives a frequency-
 *   domain view of the kernel), output (bottom).  Adjust cutoff and
 *   you can watch all four panels respond live.
 *
 * HOW TO THINK ABOUT IT — THE RADIO TUNER PICTURE
 *   Imagine you have a noisy radio receiver picking up many stations.
 *   You want to listen to one station only.  The TUNING KNOB selects
 *   the station; it's a band-pass filter centered on that station's
 *   frequency.  Internally the filter is a small array of numbers
 *   (the impulse response) that get multiplied with the incoming
 *   signal samples and summed — exactly the convolution from
 *   signal/convolution_helloworld.c.
 *
 *   This demo is a visual radio tuner.  Pick a filter (tuning knob),
 *   pick an input (the radio band's signal), watch what comes out.
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │   FOUR-PANEL STORY:                                    │
 *      │                                                        │
 *      │   1. Input          mix of sines at bins 3, 11, 23     │
 *      │      x[n]                                              │
 *      │                                                        │
 *      │   2. Impulse h[i]   the filter's "DNA" — short sinc    │
 *      │                     centered, K = 21 taps              │
 *      │                                                        │
 *      │   3. Magnitude      |H[k]| — passband peak, stop band  │
 *      │      response       valleys (frequency-domain view)    │
 *      │                                                        │
 *      │   4. Output y[n]    only the input frequencies that    │
 *      │                     fall inside the passband survive   │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * THE WINDOWED-SINC RECIPE  (in one ASCII diagram)
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │   ideal h_lp(t)            — infinite sinc:            │
 *      │                                                        │
 *      │      ___                                               │
 *      │     /   \                                              │
 *      │    /     \      ___      ___     (oscillating tails    │
 *      │  --       --   /   \    /   \    extending forever)    │
 *      │              \/     \--/                               │
 *      │                                                        │
 *      │                ↓ TRUNCATE to K = 21 taps               │
 *      │                                                        │
 *      │   truncated h[0..K-1]      — approximate sinc:         │
 *      │                                                        │
 *      │      ___                                               │
 *      │     /   \                                              │
 *      │  ___                ___ ←— sharp edges at i = 0, K-1  │
 *      │                                                        │
 *      │                ↓ APPLY WINDOW (Hann, Hamming, ...)     │
 *      │                                                        │
 *      │   windowed h[0..K-1]       — taper smooths edges:      │
 *      │                                                        │
 *      │      ___                                               │
 *      │     /   \                                              │
 *      │  ../     \..  ←— smooth taper to ~0 at edges          │
 *      │                                                        │
 *      │   This windowed kernel IS our filter.                  │
 *      │   Apply via convolution.  Done.                        │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 *   The sharp edges of the truncated (un-windowed) kernel cause
 *   ugly ripples in the magnitude response (Gibbs phenomenon for
 *   filter design).  The window's smooth taper trades sharper
 *   transitions for smaller ripples — a classic time/frequency
 *   trade-off (T4 covers it in detail).
 *
 * ALGORITHM IN STEPS  (per frame)
 *   1. (Auto-sweep) advance cutoff if enabled.
 *   2. Generate the input signal x[n].
 *   3. DESIGN the filter:
 *      a. Build ideal low-pass: h[i] = 2*fc * sinc(2*fc*(i - center))
 *      b. If high-pass: spectral inversion h = identity - h
 *      c. If band-pass: spectral difference h = low_pass(f_high) -
 *                                              low_pass(f_low)
 *      d. Multiply h[i] by window[i] for i in [0, K).
 *   4. APPLY: convolve y[n] = sum_i h[i] * x[n + i]
 *   5. Compute magnitude (and phase if 'd' is on) response of h
 *      via DFT for the visualisation panels.
 *   6. Render: input panel, impulse response panel, magnitude
 *      response panel, output panel, debug overlays, HUD.
 *
 * KEY FORMULAS
 *
 *   sinc(x) = sin(pi*x) / (pi*x), sinc(0) = 1.
 *
 *   Ideal low-pass impulse response:
 *     h_lp[i] = 2 * fc * sinc(2 * fc * (i - center))
 *     where fc is cutoff in cycles/sample, center is (K-1)/2.
 *
 *   Spectral inversion (low-pass → high-pass):
 *     h_hp[i] = (i == center) ? 1 : 0    minus    h_lp[i]
 *     i.e. impulse at center minus the low-pass impulse response
 *
 *   Spectral difference (low-pass → band-pass):
 *     h_bp[i] = h_lp(f_high)[i] - h_lp(f_low)[i]
 *
 *   Convolution:
 *     y[n] = sum_{i=0..K-1} h[i] * x[n + i]
 *
 *   Magnitude response (for visualization):
 *     pad h with zeros to length N
 *     H = DFT(h_padded)
 *     |H[k]| = sqrt(re^2 + im^2)
 *
 *   Phase response (debug overlay):
 *     arg(H[k]) = atan2(im(H[k]), re(H[k]))
 *     For symmetric kernels, arg(H[k]) is LINEAR in k → linear-phase
 *     FIR filter → no group-delay distortion.
 *
 * EDGE CASES TO WATCH
 *   - The cutoff fc is in CYCLES PER SAMPLE.  Range is (0, 0.5).
 *     At fc = 0.5 you're at Nyquist (no filtering); at fc = 0 you'd
 *     kill everything.  In bins (out of N), the equivalent integer
 *     cutoff is fc * N — we work in bins for the user-facing
 *     controls.
 *
 *   - Truncation introduces "Gibbs ringing" in the frequency
 *     response.  Rectangular window has the worst ringing (~21 dB
 *     sidelobes); Blackman window has the best (~74 dB) but a wider
 *     main lobe.  Hann (~44 dB) and Hamming (~53 dB) are good
 *     general-purpose middle ground.  Try cycling 'w' and watching
 *     the magnitude response panel — sidelobe heights change visibly.
 *
 *   - Boundary effects: the convolution loses K-1 output samples at
 *     the edges (where the filter would run off the end of the input).
 *     We zero those samples in the output panel — visible as flat
 *     regions at the right edge.
 *
 *   - Band-pass with f_low close to f_high: the passband becomes very
 *     narrow.  Energy in the output drops dramatically.  This is the
 *     practical limit of FIR design — narrow band-passes need many
 *     more taps than the K = 21 we use here.
 *
 *   - LINEAR PHASE.  Because our windowed-sinc kernel is symmetric
 *     (h[i] = h[K-1-i]), the phase response is exactly linear in k.
 *     Press 'd' to see — it's a straight line that wraps at ±pi.
 *     Linear phase means all frequencies are delayed by exactly the
 *     same number of samples (= K/2), so the filter introduces no
 *     SHAPE distortion, only a delay.  This is the #1 reason audio
 *     engineers prefer FIR over IIR for high-quality filtering.
 *
 * HOW TO VERIFY
 *   - Default: low-pass, Hann window, cutoff bin ~5, input "Sum of
 *     sines" at bins 3, 11, 23.  Output should show only the bin-3
 *     sine; the bin-11 and bin-23 components should be killed.
 *
 *   - Press 'f' to high-pass, cutoff bin 17.  Output should show
 *     only the bin-23 sine (highest frequency component).
 *
 *   - Press 'f' to band-pass, cutoff bin 11.  Output should show
 *     only the bin-11 sine (the middle component).
 *
 *   - Press 's' to "Impulse" input.  The output IS the filter's
 *     impulse response — that's the DEFINITION of impulse response,
 *     made visible.  Try it with each filter type.
 *
 *   - Press 'w' to cycle through windows.  The magnitude response
 *     panel's sidelobes get visibly smaller from Rectangular to
 *     Blackman; the main lobe gets wider.
 *
 *   - Press 'd' for the phase response panel.  Should be a clean
 *     linear ramp (with ±pi wraparounds).
 *
 *   - Press 'D' for the kernel numeric table — see the actual K
 *     filter weights as floats.  Useful for copying the design into
 *     other code.
 *
 *   - Auto-sweep cutoff (default on): the magnitude response slides
 *     left/right and the output passes different input components
 *     in turn.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What is an FIR filter?  (the impulse response IS the filter)
 *   T2   The ideal low-pass — why the impulse response is a sinc
 *   T3   Truncation + windowing — why we can't use the ideal directly
 *   T4   Four windows and the sidelobe vs main-lobe trade-off
 *   T5   Spectral inversion (low-pass → high-pass)
 *   T6   Spectral difference (low-pass → band-pass)
 *   T7   Applying the filter — convolution recap
 *   T8   Reading the magnitude response panel
 *   T9   Linear phase — why FIR filters preserve waveform shape
 *   T10  Cost analysis: O(N*K) direct vs O(N log N) FFT-based
 *   T11  Practical filter design — picking K, cutoff, window
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS AN FIR FILTER?
 * ──────────────────────────
 * Filter is a fancy word for "convolve with a kernel".  An FIR
 * filter is a SHORT array of weights (the kernel, or "impulse
 * response", or "taps") that you slide across your signal,
 * multiplying and summing — exactly the convolution operation
 * from signal/convolution_helloworld.c.
 *
 * "FIR" stands for FINITE IMPULSE RESPONSE.  The defining property:
 * if you feed the filter a single impulse (one 1, otherwise zeros),
 * the output is non-zero for exactly K samples and then permanently
 * zero.  In other words, the filter "remembers" only the last K
 * inputs; older inputs are forgotten.
 *
 * Compare to IIR (Infinite Impulse Response) filters, which use
 * feedback (output samples loop back into the input).  IIR filters'
 * impulse response rings forever, decaying exponentially.  Pros:
 * fewer multiplies for the same selectivity.  Cons: can be
 * unstable, have non-linear phase, harder to design.
 *
 * For pedagogical purposes (this file) and many practical
 * applications (audio, image processing, scientific computing),
 * FIR is the easier place to start.
 *
 * The art of FIR filtering is in CHOOSING THE KERNEL.  All the
 * theory in this file is about that choice.
 *
 * T2  THE IDEAL LOW-PASS — WHY THE IMPULSE RESPONSE IS A SINC
 * ──────────────────────────────────────────────────────────
 * Suppose we want a PERFECT low-pass filter — one that passes
 * frequencies below cutoff fc unchanged and completely rejects
 * frequencies above fc.  In the frequency domain, the desired
 * transfer function H(f) is a RECTANGLE:
 *
 *     H(f) = 1 for |f| <= fc
 *     H(f) = 0 for |f| > fc
 *
 * What does this filter's impulse response h(t) look like in the
 * time domain?  By the inverse Fourier transform:
 *
 *     h(t) = inverse_FT(H(f))
 *
 * The Fourier transform pair RECT ↔ SINC says:
 *
 *     FT(rect) = sinc      and     FT(sinc) = rect
 *
 * So h(t) of the ideal low-pass is a SINC FUNCTION.  Specifically,
 * for cutoff fc in cycles per second and unit sample rate:
 *
 *     h(t) = 2 * fc * sinc(2 * fc * t)
 *     where sinc(x) = sin(pi*x) / (pi*x)
 *
 * In discrete time (sampling rate = 1 sample per unit time, so
 * "samples" and "seconds" are interchangeable):
 *
 *     h[i] = 2 * fc * sinc(2 * fc * (i - center))
 *
 * The CENTER offset shifts the sinc so its peak is at the middle
 * of the kernel rather than at i = 0.  This makes the truncated
 * kernel symmetric around its centre — important for LINEAR PHASE
 * (T9).
 *
 * The §7 lowpass_sinc() function is THIS formula, written in C.
 *
 * T3  TRUNCATION + WINDOWING — WHY WE CAN'T USE THE IDEAL DIRECTLY
 * ────────────────────────────────────────────────────────────────
 * The ideal sinc has INFINITE LENGTH.  We can't store an infinite
 * array of taps; we have to TRUNCATE to a finite K samples.
 *
 * Truncation is mathematically equivalent to MULTIPLYING the ideal
 * sinc by a RECTANGULAR WINDOW (1 inside [0, K), 0 outside).  By the
 * convolution-multiplication duality (signal/idft_helloworld.c T9),
 * multiplying in time = convolving in frequency.
 *
 *     time domain:   h_truncated = h_ideal * rect_window
 *     freq domain:   H_truncated = H_ideal * RECT_WINDOW_FT
 *                  = ideal_response * sinc
 *
 * The frequency response of a rectangular window is a sinc itself
 * (FT(rect) = sinc).  Convolving the ideal flat passband with a
 * sinc creates RIPPLES around the cutoff — the GIBBS PHENOMENON
 * applied to filter design.
 *
 * To reduce these ripples, we taper the truncated kernel with a
 * SMOOTH window function whose own frequency response has smaller
 * sidelobes.  Hann, Hamming, Blackman — all do this.  Their
 * frequency-domain shapes are smoother than rect → smaller ripples
 * in the resulting filter response.
 *
 * Trade-off (T4): smoother window → smaller ripples in H, but ALSO
 * a WIDER MAIN LOBE in H → wider transition band → less sharp
 * cutoff.  There's no free lunch.
 *
 * T4  FOUR WINDOWS AND THE SIDELOBE vs MAIN-LOBE TRADE-OFF
 * ────────────────────────────────────────────────────────
 * The four window functions cycled by 'w' have different trade-offs:
 *
 *     window         main-lobe width    first sidelobe (dB below)
 *     -----------    ----------------    -------------------------
 *     Rectangular    narrow (1 bin)      -13 dB
 *     Hann           wider (1.5 bins)    -32 dB
 *     Hamming        like Hann            -42 dB
 *     Blackman       widest (2 bins)      -58 dB
 *
 * "Main-lobe width" determines how SHARP the cutoff is — narrower
 * main lobe = steeper transition from passband to stopband.
 *
 * "Sidelobe height" determines how clean the stopband is —
 * smaller sidelobes = better rejection of unwanted frequencies.
 *
 * The two are inversely related.  You can't have both.  Pick a
 * window based on what matters more for your application:
 *
 *   - DETECTING a known weak signal near a strong one (radar,
 *     bird-song): pick a window with small sidelobes (Blackman).
 *   - RESOLVING two closely-spaced frequencies: pick a window with
 *     a narrow main lobe (Hann or Rectangular).
 *   - RECONSTRUCTING the signal exactly: Rectangular (no window).
 *
 * Try cycling 'w' in this demo and watch the magnitude response
 * panel.  Going Rect → Hann → Hamming → Blackman, the sidelobes
 * shrink visibly while the main lobe widens.
 *
 * T5  SPECTRAL INVERSION  (LOW-PASS → HIGH-PASS)
 * ──────────────────────────────────────────────
 * Once you know how to design a low-pass, you can derive any other
 * filter type from it.
 *
 * High-pass = "all frequencies EXCEPT those below cutoff".  In
 * frequency domain:
 *
 *     H_hp(f)  =  1 - H_lp(f)
 *
 * The constant "1" in the frequency domain corresponds to an
 * IMPULSE in the time domain (FT of an impulse is a constant).
 * So in time domain:
 *
 *     h_hp[i]  =  identity_at_center[i] - h_lp[i]
 *
 * where identity_at_center is 1 at the centre tap and 0 elsewhere.
 *
 * This is the SPECTRAL INVERSION trick.  It works for any
 * symmetric low-pass kernel.  In code (§7):
 *
 *     for i in 0..K-1:
 *       h[i] = -h_lp[i]
 *     h[center] += 1
 *
 * Negate every tap, then add 1 to the centre tap.  Done.  The
 * resulting kernel is a high-pass filter with the same cutoff and
 * same window as the original low-pass.
 *
 * T6  SPECTRAL DIFFERENCE  (LOW-PASS → BAND-PASS)
 * ───────────────────────────────────────────────
 * Band-pass = "frequencies BETWEEN f_low and f_high".  In frequency
 * domain:
 *
 *     H_bp(f)  =  H_lp(f_high) - H_lp(f_low)
 *
 * The first term passes everything below f_high; the second
 * subtracts everything below f_low.  Net result: passes (f_low,
 * f_high).  In time domain:
 *
 *     h_bp[i]  =  h_lp(f_high)[i] - h_lp(f_low)[i]
 *
 * In code (§7):
 *
 *     lowpass_sinc(f_high, h_high)
 *     lowpass_sinc(f_low,  h_low)
 *     for i in 0..K-1:
 *       h[i] = h_high[i] - h_low[i]
 *
 * Two low-passes, subtracted.  The cutoff knob controls the centre
 * of the band; we use a fixed half-width of 3 bins for a
 * moderately narrow passband.
 *
 * T7  APPLYING THE FILTER — CONVOLUTION RECAP
 * ───────────────────────────────────────────
 * Once you have the kernel, applying the filter is just convolution:
 *
 *     y[n] = sum_{i=0..K-1} h[i] * x[n + i]
 *
 * Same formula as signal/convolution_helloworld.c §6.  In code (§8):
 *
 *     for output_position in 0..N-K:
 *       weighted_sum = 0
 *       for kernel_tap_index in 0..K-1:
 *         weighted_sum += h[kernel_tap_index]
 *                       * x[output_position + kernel_tap_index]
 *       y[output_position] = weighted_sum
 *
 * For boundary handling we zero the K-1 outputs at the right edge
 * (where the kernel would run off the end of the input).  See
 * convolution_helloworld.c T6 for alternatives (zero-padding,
 * circular).
 *
 * T8  READING THE MAGNITUDE RESPONSE PANEL
 * ────────────────────────────────────────
 * The magnitude response panel shows |H[k]| — how much energy at
 * each frequency bin k passes through the filter.  Three regions
 * to recognize:
 *
 *     PASSBAND     |H[k]| ≈ 1.  Frequencies in this region pass
 *                  through unchanged.
 *     STOPBAND     |H[k]| ≈ 0.  Frequencies in this region are
 *                  blocked.
 *     TRANSITION   |H[k]| smoothly rolls from 1 to 0.  No real
 *                  filter has an instantaneous transition.
 *
 * For a low-pass, the passband is on the left (low bins), stopband
 * on the right (high bins).  For high-pass, reversed.  For band-
 * pass, passband is a hill in the middle, stopbands on both sides.
 *
 * The cutoff marker (yellow ':' in the panel) shows where the
 * "official" cutoff sits — typically defined as the bin where
 * |H[k]| has dropped to ~0.5 (the -6 dB point) or 0.707 (the -3 dB
 * point).
 *
 * Sidelobes (T4) are the small wiggles in the stopband.  Watch
 * them shrink as you cycle through windows.
 *
 * T9  LINEAR PHASE — WHY FIR FILTERS PRESERVE WAVEFORM SHAPE
 * ──────────────────────────────────────────────────────────
 * For a SYMMETRIC kernel (h[i] = h[K-1-i], which all our designed
 * kernels are), the phase response arg(H[k]) is exactly LINEAR in k.
 *
 * Why does this matter?  Because phase ↔ time-shift.  A LINEAR
 * phase means every frequency is delayed by EXACTLY THE SAME
 * NUMBER OF SAMPLES.  Specifically, the delay is K/2 samples, which
 * is the centre of the kernel.
 *
 * Consequence: the filter does not distort the SHAPE of the input
 * — it just delays it.  A square wave going in comes out as a
 * smoothed square wave (low-pass filtered) but still recognizable
 * as a square — the relative timing between its harmonics is
 * preserved.
 *
 * Compare to an IIR filter, where the phase response is in general
 * NOT linear.  Different frequencies get different delays, and the
 * input's shape is distorted (the harmonics arrive at different
 * times so the wave changes shape, not just amplitude).
 *
 * This is a HUGE deal in audio:
 *   - High-end audio EQs use LINEAR-PHASE FIR filters specifically
 *     to avoid shape distortion — preserving "transient response".
 *   - Mastering plugins (e.g. iZotope Ozone) offer linear-phase
 *     mode for the same reason.
 *   - The cost: more taps (longer kernel) → more CPU + more delay.
 *     IIR is cheaper but sacrifices linear phase.
 *
 * Press 'd' to toggle the phase response panel and see the linear
 * ramp.  It wraps at ±pi, so you'll see a sawtooth pattern, but
 * within each "tooth" the slope is constant.
 *
 * T10  COST ANALYSIS — DIRECT vs FFT-BASED
 * ────────────────────────────────────────
 * Direct convolution: y[n] = sum_i h[i] * x[n+i].
 *
 *   Cost: N inner iterations × N outer iterations = O(N * K) total.
 *   At N = 64, K = 21: 1344 multiplies per signal.  Negligible.
 *
 * FFT-based convolution:
 *   1. FFT(x) → X            O(N log N)
 *   2. FFT(h zero-padded) → H  O(N log N)
 *   3. Multiply Y[k] = X[k] * H[k] for all k  O(N)
 *   4. IFFT(Y) → y           O(N log N)
 *   Total: O(N log N) regardless of K.
 *
 * Crossover point (where FFT becomes faster):
 *   - At small K (< ~50), direct is faster (less overhead).
 *   - At medium K (50 to 500), comparable.
 *   - At large K (> 500), FFT-based wins decisively.
 *
 * For audio applications:
 *   - Speech coding (K ≈ 32):  direct
 *   - EQ filtering (K ≈ 64-256): direct
 *   - Convolution reverb (K ≈ 10000+): FFT-based, partitioned
 *
 * For this demo K = 21 so direct is the right choice.  If you ever
 * find yourself with a huge K, partition the kernel and use the
 * "overlap-add" or "overlap-save" technique to keep the FFT size
 * reasonable.
 *
 * T11  PRACTICAL FILTER DESIGN — PICKING K, CUTOFF, WINDOW
 * ────────────────────────────────────────────────────────
 * Three knobs, three decisions:
 *
 *   K (NUMBER OF TAPS).  Bigger K → narrower transition band, more
 *   precise cutoff.  Doubling K halves the transition band width
 *   (approximately).  Trade-off: linear cost in CPU and in delay
 *   (delay = K/2 samples).  Common values:
 *     - 21..51 for casual filtering, gentle slopes.
 *     - 100..500 for serious audio EQ.
 *     - 1000+ for sharp brick-wall filters or convolution reverb.
 *
 *   CUTOFF.  Where you want the transition.  Express in cycles per
 *   sample (range (0, 0.5)) or normalized by sample rate.  In this
 *   demo we use bins out of N for ease.
 *
 *   WINDOW.  Pick based on your priority (T4):
 *     - Sharp transition, small ripples in passband: Hann or Hamming
 *     - Strong stopband suppression: Blackman or Kaiser
 *     - Quick prototyping, don't care about ripples: Rectangular
 *
 * For most jobs the recipe is: K = 51 or 101, Hann or Hamming
 * window, cutoff at the desired -6 dB point.  Iterate by ear or by
 * checking the magnitude response in a tool like Octave/Matlab.
 *
 * For more advanced design (equiripple via the Parks-McClellan
 * algorithm, minimum-phase filters, etc.) see the references.
 * But windowed-sinc with Hann gets you 90% of the way for 10% of
 * the complexity.
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

#define N                  64        /* input signal length             */
#define K                  21        /* filter length (must be odd for
                                       * symmetric centered design)     */
#define K_CENTER           (K / 2)
#define N_HALF             (N / 2)
#define RENDER_FPS         30
#define RENDER_TICK_NS     (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 120      /* one full cutoff sweep ≈ 4 seconds */

enum {
    PAIR_INPUT     = 1,    /* input wave bars                              */
    PAIR_OUTPUT    = 2,    /* output (filtered) wave bars                  */
    PAIR_FILTER    = 3,    /* filter impulse response bars                 */
    PAIR_MAGRESP   = 4,    /* magnitude response bars                      */
    PAIR_PHRESP    = 5,    /* phase response bars (debug)                  */
    PAIR_LABEL     = 6,    /* panel labels                                 */
    PAIR_HUD       = 7,    /* HUD top status                               */
    PAIR_HINT      = 8,    /* bottom hint                                  */
};

/* ===================================================================== */
/* §2  clock                                                             */
/* ===================================================================== */

static long long clock_now_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  complex — minimal toolkit for the magnitude/phase response DFT    */
/* ===================================================================== */
/*
 *  Only used for the visualization DFT in §9.  The convolution
 *  itself uses real arithmetic (kernel and signal are both real).
 */

typedef struct {
    float re;
    float im;
} ComplexNumber;

static const ComplexNumber complex_zero = { 0.0f, 0.0f };

static inline ComplexNumber complex_make(float re, float im)
{
    return (ComplexNumber){ re, im };
}

static inline ComplexNumber complex_from_angle(float angle_radians)
{
    return complex_make(cosf(angle_radians), sinf(angle_radians));
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b)
{
    return complex_make(a.re + b.re, a.im + b.im);
}

static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z)
{
    return complex_make(scale * z.re, scale * z.im);
}

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
        init_pair(PAIR_INPUT,    51, -1);   /* bright cyan      */
        init_pair(PAIR_OUTPUT,  154, -1);   /* yellow-green     */
        init_pair(PAIR_FILTER,  213, -1);   /* magenta-pink     */
        init_pair(PAIR_MAGRESP, 220, -1);   /* gold             */
        init_pair(PAIR_PHRESP,  117, -1);   /* sky blue         */
        init_pair(PAIR_LABEL,   244, -1);   /* mid grey         */
        init_pair(PAIR_HUD,     226, -1);   /* bright yellow    */
        init_pair(PAIR_HINT,     51, -1);   /* bright cyan      */
    } else {
        init_pair(PAIR_INPUT,    COLOR_CYAN,    -1);
        init_pair(PAIR_OUTPUT,   COLOR_GREEN,   -1);
        init_pair(PAIR_FILTER,   COLOR_MAGENTA, -1);
        init_pair(PAIR_MAGRESP,  COLOR_YELLOW,  -1);
        init_pair(PAIR_PHRESP,   COLOR_BLUE,    -1);
        init_pair(PAIR_LABEL,    COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,      COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,     COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §5  sinc_helper — the sin(pi*x) / (pi*x) function                     */
/* ===================================================================== */
/*
 *  sinc(x) is the impulse response of an ideal low-pass filter.  The
 *  Fourier transform pair RECT ↔ SINC: a rectangle in the frequency
 *  domain (the ideal flat passband) inverse-transforms to a sinc in
 *  the time domain.  See T2 for the derivation.
 *
 *  Key values:
 *    sinc(0) = 1           (limit, taken to avoid 0/0)
 *    sinc(integer) = 0     (for any non-zero integer argument)
 *    The function decays as 1/x for large x.
 *
 *  We use float epsilon to detect x ≈ 0 and return 1 directly,
 *  avoiding the divide-by-zero.
 */
static float sinc(float x)
{
    if (fabsf(x) < 1e-7f) return 1.0f;
    float pi_x = (float)M_PI * x;
    return sinf(pi_x) / pi_x;
}

/* ===================================================================== */
/* §6  windows — 4 window functions                                      */
/* ===================================================================== */

typedef enum {
    WIN_RECT = 0,    /* no taper — worst ringing (-13 dB sidelobes)   */
    WIN_HANN,        /* good general-purpose         (-32 dB)         */
    WIN_HAMMING,     /* slightly better sidelobes    (-42 dB)         */
    WIN_BLACKMAN,    /* best sidelobes, widest main  (-58 dB)         */
    WIN_COUNT
} WindowKind;

static const char *window_name[WIN_COUNT] = {
    "Rectangular", "Hann       ", "Hamming    ", "Blackman   "
};

/*
 * window_coefficient — return w[i] for filter tap i in [0, K-1].
 *   Symmetric windows centered at (K-1)/2.  Each window is a
 *   smooth taper that goes to (or near) zero at the edges, which
 *   reduces the Gibbs ringing caused by truncating the ideal sinc.
 *
 *   See T4 for the sidelobe-vs-main-lobe trade-off between windows.
 */
static float window_coefficient(WindowKind kind, int i)
{
    float two_pi_over_K_minus_1 = 2.0f * (float)M_PI / (float)(K - 1);
    switch (kind) {
        case WIN_RECT:     return 1.0f;
        case WIN_HANN:
            return 0.5f - 0.5f * cosf(two_pi_over_K_minus_1 * (float)i);
        case WIN_HAMMING:
            return 0.54f - 0.46f * cosf(two_pi_over_K_minus_1 * (float)i);
        case WIN_BLACKMAN:
            return 0.42f
                 - 0.5f  * cosf(two_pi_over_K_minus_1 * (float)i)
                 + 0.08f * cosf(2.0f * two_pi_over_K_minus_1 * (float)i);
        default: return 1.0f;
    }
}

/* ===================================================================== */
/* §7  filter_design — windowed-sinc low/high/band-pass                  */
/* ===================================================================== */

typedef enum {
    FILT_LOW_PASS = 0,
    FILT_HIGH_PASS,
    FILT_BAND_PASS,
    FILT_COUNT
} FilterKind;

static const char *filter_name[FILT_COUNT] = {
    "low-pass  ", "high-pass ", "band-pass "
};

/*
 *  lowpass_sinc — fill h[0..K-1] with the IDEAL (un-windowed) low-
 *  pass impulse response with cutoff fc cycles/sample.
 *
 *  Used as a building block for high-pass (spectral inversion) and
 *  band-pass (spectral difference) — see design_filter().
 *
 *  Pseudocode:
 *    center = (K - 1) / 2          (in floats)
 *    for i in 0..K-1:
 *      h[i] = 2 * fc * sinc(2 * fc * (i - center))
 *
 *  Read it: peak at i = center (where sinc = 1, scaled by 2*fc),
 *  then sin/x oscillations on either side.  See T2 for why this
 *  particular formula is the ideal low-pass impulse response.
 */
static void lowpass_sinc(float fc, float *h)
{
    float center = (float)(K - 1) * 0.5f;     /* (K-1)/2 in floats */
    for (int i = 0; i < K; i++) {
        float x = 2.0f * fc * ((float)i - center);
        h[i] = 2.0f * fc * sinc(x);
    }
}

/*
 *  design_filter — build the filter h[0..K-1] for the chosen filter
 *  type, cutoff, and window.  THE recipe of this file.
 *
 *  Pseudocode:
 *    fc = cutoff_bin / N                             (cycles per sample)
 *
 *    Step 1.  Build the IDEAL impulse response.
 *      For low-pass:   h = lowpass_sinc(fc)
 *      For high-pass:  h = lowpass_sinc(fc), then negate; add 1 at center
 *                      (= identity - low_pass; see T5)
 *      For band-pass:  h = lowpass_sinc(fc + half_width) -
 *                          lowpass_sinc(fc - half_width)
 *                      (= spectral difference; see T6)
 *
 *    Step 2.  Apply the WINDOW (taper truncation edges).
 *      For each i in 0..K-1:
 *        h[i] *= window_coefficient(window, i)
 *
 *  In code each step is one block in the switch + a final loop.
 */
static void design_filter(FilterKind kind, int cutoff_bin,
                          WindowKind window, float *h)
{
    /* Convert cutoff bin to normalized cutoff frequency in cycles
     * per sample.  bin / N is the standard normalisation. */
    float cutoff_normalised = (float)cutoff_bin / (float)N;

    /* ── Step 1.  Build the ideal impulse response ─────────────── */
    switch (kind) {
        case FILT_LOW_PASS: {
            /* Ideal low-pass = sinc.  See T2. */
            lowpass_sinc(cutoff_normalised, h);
            break;
        }
        case FILT_HIGH_PASS: {
            /* Spectral inversion: h_hp = identity - h_lp.  See T5.
             *   First compute h_lp.
             *   Then negate every tap.
             *   Then add 1 at the centre (identity at centre, 0 elsewhere).
             */
            lowpass_sinc(cutoff_normalised, h);
            for (int i = 0; i < K; i++) h[i] = -h[i];
            h[K_CENTER] += 1.0f;
            break;
        }
        case FILT_BAND_PASS: {
            /* Spectral difference: h_bp = h_lp(f_high) - h_lp(f_low).
             * See T6.  We use a fixed half-width of 3 bins for a
             * moderately narrow passband.  The cutoff knob acts as
             * the CENTER of the band. */
            float band_half_bins = 3.0f;
            float fc_lo = ((float)cutoff_bin - band_half_bins) / (float)N;
            float fc_hi = ((float)cutoff_bin + band_half_bins) / (float)N;
            if (fc_lo < 0.01f) fc_lo = 0.01f;
            if (fc_hi > 0.49f) fc_hi = 0.49f;
            float h_lo[K], h_hi[K];
            lowpass_sinc(fc_lo, h_lo);
            lowpass_sinc(fc_hi, h_hi);
            for (int i = 0; i < K; i++) h[i] = h_hi[i] - h_lo[i];
            break;
        }
        default: break;
    }

    /* ── Step 2.  Apply the window function ─────────────────────
     * Tapers the truncation edges.  Trades sharper transition for
     * smaller sidelobes.  See T3 / T4. */
    for (int i = 0; i < K; i++)
        h[i] *= window_coefficient(window, i);
}

/* ===================================================================== */
/* §8  convolve — apply filter to input signal                           */
/* ===================================================================== */
/*
 *  Purpose         : compute y[n] = sum_i h[i] * x[n + i] for n in
 *                    [0, N-K].  Output samples beyond that range
 *                    are zeroed (boundary).
 *
 *  Pseudocode      :
 *      last_valid_position = N - K
 *      for output_position in 0..last_valid_position:
 *        weighted_sum = 0
 *        for kernel_tap_index in 0..K-1:
 *          Step 1.  Read kernel weight: kernel_taps[kernel_tap_index]
 *          Step 2.  Read input sample : input_signal[output_position
 *                                                  + kernel_tap_index]
 *          Step 3.  Multiply and accumulate
 *        output_signal[output_position] = weighted_sum
 *      for output_position in last_valid_position + 1..N-1:
 *        output_signal[output_position] = 0    (boundary)
 *
 *  Mental model    : slide K-tap kernel across input, write one
 *                    output per slide position.  Same algorithm as
 *                    signal/convolution_helloworld.c §8.
 *
 *  Why it exists   : THIS is filter APPLICATION.  Given a designed
 *                    kernel (from §7) and an input (from §10), this
 *                    function produces the filtered output that the
 *                    bottom panel displays.
 */
static void convolve(const float *input_signal,
                     const float *kernel_taps,
                     float *output_signal)
{
    /* Largest valid output position.  Beyond this the kernel would
     * read past the end of the input. */
    int last_valid_output_position = N - K;

    for (int output_position = 0;
         output_position <= last_valid_output_position;
         output_position++) {

        float weighted_sum = 0.0f;

        for (int kernel_tap_index = 0;
             kernel_tap_index < K;
             kernel_tap_index++) {

            /* ── STEP 1 — read the kernel weight ───────────────── */
            float kernel_weight = kernel_taps[kernel_tap_index];

            /* ── STEP 2 — read the corresponding input sample ── */
            float input_sample = input_signal[output_position
                                              + kernel_tap_index];

            /* ── STEP 3 — multiply and accumulate ─────────────── */
            weighted_sum += kernel_weight * input_sample;
        }

        output_signal[output_position] = weighted_sum;
    }

    /* Boundary: outputs past N - K are not well-defined.  Zero them. */
    for (int output_position = last_valid_output_position + 1;
         output_position < N;
         output_position++) {
        output_signal[output_position] = 0.0f;
    }
}

/* ===================================================================== */
/* §9  dft_response — DFT of zero-padded filter for |H[k]| + arg(H[k])   */
/* ===================================================================== */
/*
 *  Purpose : compute |H[k]| (magnitude response) and arg(H[k])
 *            (phase response, debug overlay) for k in [0, N/2].
 *            Used by the visualisation panels — NOT in the filter
 *            application path.
 *
 *  We zero-pad h from K to N samples (so the DFT has more frequency
 *  resolution than just N/K bins) and run the textbook DFT.
 *
 *  Cost: O(N * N) DFT = 4096 multiplies.  At N = 64 negligible.
 *  We could use the FFT but the helloworld vibes call for the
 *  textbook DFT.
 */
static void compute_filter_response(const float *h,
                                    float *out_magnitude,
                                    float *out_phase)
{
    /* Zero-pad. */
    float padded[N];
    for (int i = 0; i < N; i++)
        padded[i] = (i < K) ? h[i] : 0.0f;

    /* DFT (real input). */
    for (int k = 0; k <= N_HALF; k++) {
        ComplexNumber acc = complex_zero;
        for (int n = 0; n < N; n++) {
            float angle = -2.0f * (float)M_PI * (float)k * (float)n
                        / (float)N;
            ComplexNumber twist   = complex_from_angle(angle);
            ComplexNumber twisted = complex_scale_by_real(padded[n], twist);
            acc = complex_add(acc, twisted);
        }
        out_magnitude[k] = complex_magnitude(acc);
        out_phase[k]     = atan2f(acc.im, acc.re);
    }
}

/* ===================================================================== */
/* §10  signals — input signal generators                                */
/* ===================================================================== */

typedef enum {
    SIG_SINES = 0,    /* sum of three sines at bins 3, 11, 23           */
    SIG_CHIRP,        /* frequency sweeps across the buffer             */
    SIG_SQUARE,       /* periodic square wave                           */
    SIG_IMPULSE,      /* one tall sample early in the buffer            */
    SIG_NOISE,        /* repeatable pseudo-random                       */
    SIG_COUNT
} SignalKind;

static const char *signal_name[SIG_COUNT] = {
    "Sum sines", "Chirp    ", "Square   ", "Impulse  ", "Noise    "
};

/*
 *  generate_signal — fill output_signal[0..N-1] with the chosen test
 *    signal.  All signals are bounded by ±1 so the panel auto-
 *    scaling is consistent.
 */
static void generate_signal(SignalKind kind, float *output_signal)
{
    switch (kind) {
        case SIG_SINES: {
            /* Sum of three pure sines at well-separated bins.  The
             * default test signal.  Cycle through filter types and
             * watch which sines survive. */
            for (int n = 0; n < N; n++) {
                float t = (float)n / (float)N;
                output_signal[n] =
                    0.5f * cosf(2.0f * (float)M_PI * 3.0f  * t)
                  + 0.4f * cosf(2.0f * (float)M_PI * 11.0f * t)
                  + 0.3f * cosf(2.0f * (float)M_PI * 23.0f * t);
            }
            break;
        }
        case SIG_CHIRP: {
            /* Linear chirp: instantaneous frequency varies linearly
             * from bin 1 to bin N/2 - 1 across the buffer.  Phase is
             * the integral of frequency, so it carries an n^2 term. */
            for (int n = 0; n < N; n++) {
                float t = (float)n / (float)N;
                float lo = 1.0f, hi = (float)N * 0.5f - 1.0f;
                float phase_cycles = lo * t + (hi - lo) * t * t * 0.5f;
                output_signal[n] = sinf(2.0f * (float)M_PI * phase_cycles);
            }
            break;
        }
        case SIG_SQUARE: {
            /* Periodic square wave at 3 cycles per buffer.  Sharp
             * edges → many high harmonics → showcases low-pass
             * smoothing dramatically. */
            for (int n = 0; n < N; n++) {
                float t = (float)n / (float)N;
                float arg = 2.0f * (float)M_PI * 3.0f * t;
                output_signal[n] = sinf(arg) >= 0.0f ? 0.7f : -0.7f;
            }
            break;
        }
        case SIG_IMPULSE: {
            /* Zero everywhere except a single 1 at index N/4.  The
             * output of any filter on an impulse IS that filter's
             * impulse response (definition demo). */
            for (int n = 0; n < N; n++) output_signal[n] = 0.0f;
            output_signal[N / 4] = 1.0f;
            break;
        }
        case SIG_NOISE: {
            /* Repeatable pseudo-random.  Useful for seeing how the
             * filter shapes broadband noise — output will have only
             * the frequencies in the filter's passband. */
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
/* §11  scene_state — globals + reset                                    */
/* ===================================================================== */

static float g_input_signal[N];
static float g_kernel_taps[K];
static float g_magnitude_response[N_HALF + 1];
static float g_phase_response[N_HALF + 1];
static float g_magnitude_peak = 1.0f;
static float g_output_signal[N];
static float g_input_peak = 1.0f;
static float g_output_peak = 1.0f;

static SignalKind g_signal_kind   = SIG_SINES;
static FilterKind g_filter_kind   = FILT_LOW_PASS;
static WindowKind g_window_kind   = WIN_HANN;
static int        g_cutoff_bin    = 5;
static bool       g_simulation_paused   = false;
static bool       g_auto_sweep_cutoff   = true;
static float      g_sweep_phase_radians = 0.0f;

/* Debug overlay toggles. */
static bool       g_show_phase_panel    = false;
static bool       g_show_kernel_table   = false;

static void scene_reset(void)
{
    g_signal_kind         = SIG_SINES;
    g_filter_kind         = FILT_LOW_PASS;
    g_window_kind         = WIN_HANN;
    g_cutoff_bin          = 5;
    g_simulation_paused   = false;
    g_auto_sweep_cutoff   = true;
    g_sweep_phase_radians = 0.0f;
}

/* ===================================================================== */
/* §12  scene_tick — per-frame update orchestrator                       */
/* ===================================================================== */
/*
 *  Six numbered steps that match T10 of the GUIDED TUTORIAL.
 */
static void scene_tick(void)
{
    if (g_simulation_paused) return;

    /* Auto-sweep cutoff.  sin gives smooth there-and-back motion. */
    if (g_auto_sweep_cutoff) {
        g_sweep_phase_radians +=
            2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
        if (g_sweep_phase_radians > 2.0f * (float)M_PI)
            g_sweep_phase_radians -= 2.0f * (float)M_PI;
        float s = (sinf(g_sweep_phase_radians) + 1.0f) * 0.5f;
        /* Cutoff in [3, N/2 - 3] so band-pass has room on both sides. */
        g_cutoff_bin = 3 + (int)(s * ((float)N * 0.5f - 6.0f) + 0.5f);
    }

    /* ── Step 1.  generate input ──────────────────────────────── */
    generate_signal(g_signal_kind, g_input_signal);

    /* ── Step 2.  DESIGN filter (the headline of this file) ──── */
    design_filter(g_filter_kind, g_cutoff_bin, g_window_kind,
                  g_kernel_taps);

    /* ── Step 3.  APPLY filter (convolve) ─────────────────────── */
    convolve(g_input_signal, g_kernel_taps, g_output_signal);

    /* ── Step 4.  compute filter response (for visualization) ── */
    compute_filter_response(g_kernel_taps, g_magnitude_response,
                            g_phase_response);

    /* ── Step 5.  peaks for autoscale ─────────────────────────── */
    g_input_peak = 1e-6f;
    for (int n = 0; n < N; n++) {
        float a = fabsf(g_input_signal[n]);
        if (a > g_input_peak) g_input_peak = a;
    }
    g_output_peak = 1e-6f;
    for (int n = 0; n < N; n++) {
        float a = fabsf(g_output_signal[n]);
        if (a > g_output_peak) g_output_peak = a;
    }
    g_magnitude_peak = 1e-6f;
    for (int k = 0; k <= N_HALF; k++) {
        if (g_magnitude_response[k] > g_magnitude_peak)
            g_magnitude_peak = g_magnitude_response[k];
    }
}

/* ===================================================================== */
/* §13  scene_input — handle keys                                        */
/* ===================================================================== */

static void scene_cycle_filter(int direction)
{
    g_filter_kind = (FilterKind)((g_filter_kind + direction + FILT_COUNT)
                                 % FILT_COUNT);
}

static void scene_cycle_window(int direction)
{
    g_window_kind = (WindowKind)((g_window_kind + direction + WIN_COUNT)
                                 % WIN_COUNT);
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
    if (g_cutoff_bin < 3) g_cutoff_bin = 3;
    if (g_cutoff_bin > N_HALF - 3) g_cutoff_bin = N_HALF - 3;
}

/* ===================================================================== */
/* §14  draw_bar                                                         */
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
/* §15  draw_signal — input + output panels                              */
/* ===================================================================== */

static void draw_signal_panel(const float *signal, int top_row,
                              int height_rows, float peak,
                              int colour_pair)
{
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

/* ===================================================================== */
/* §16  draw_filter — impulse + magnitude + phase response panels        */
/* ===================================================================== */

static void draw_impulse_response_panel(int top_row, int height_rows)
{
    /* Impulse response h[0..K-1] as bars centered horizontally on
     * the panel.  Positive grows up, negative grows down. */
    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    /* Find filter peak for normalisation. */
    float peak = 0.0f;
    for (int i = 0; i < K; i++) {
        float a = fabsf(g_kernel_taps[i]);
        if (a > peak) peak = a;
    }
    if (peak < 1e-6f) peak = 1.0f;

    /* Center the K-tap kernel on the screen.  3 cells per tap. */
    int tap_w = 3;
    int total_w = K * tap_w;
    int left = (COLS - total_w) / 2;
    if (left < 0) left = 0;

    for (int i = 0; i < K; i++) {
        float v   = g_kernel_taps[i] / peak;
        bool  pos = (v >= 0.0f);
        int   h   = (int)(fabsf(v) * (float)half_height + 0.5f);
        for (int s = 0; s < tap_w - 1; s++) {
            int col = left + i * tap_w + s;
            draw_bar(col, midline_row, h, pos, PAIR_FILTER);
        }
    }

    /* Midline marker. */
    attron(COLOR_PAIR(PAIR_LABEL));
    int mid_left = left;
    int mid_right = left + total_w;
    if (mid_left < 0) mid_left = 0;
    if (mid_right > COLS) mid_right = COLS;
    for (int x = mid_left; x < mid_right; x++)
        mvaddch(midline_row, x, '-');
    attroff(COLOR_PAIR(PAIR_LABEL));
}

static void draw_magnitude_response_panel(int top_row, int height_rows)
{
    /* |H[k]| for k=0..N/2 as upward bars from panel bottom. */
    int baseline_row = top_row + height_rows - 1;
    int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
    int max_bins = COLS / bin_w;
    if (max_bins > N_HALF + 1) max_bins = N_HALF + 1;

    for (int k = 0; k < max_bins; k++) {
        float v = g_magnitude_response[k] / (g_magnitude_peak + 1e-6f);
        int   h = (int)(v * (float)height_rows + 0.5f);
        for (int bx = 0; bx < bin_w - 1; bx++)
            draw_bar(k * bin_w + bx, baseline_row, h, true, PAIR_MAGRESP);
    }

    /* Cutoff marker — vertical ':' at the cutoff column. */
    if (g_cutoff_bin <= max_bins) {
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

static void draw_phase_response_panel(int top_row, int height_rows)
{
    /* arg(H[k]) for k=0..N/2 as ± bars from a midline.  For
     * symmetric kernels the phase is LINEAR in k → ramp pattern
     * with ±pi wraparounds (T9). */
    if (height_rows < 3) return;

    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
    int max_bins = COLS / bin_w;
    if (max_bins > N_HALF + 1) max_bins = N_HALF + 1;

    for (int k = 0; k < max_bins; k++) {
        /* Skip near-zero magnitude bins (phase would be noise). */
        if (g_magnitude_response[k] < 1e-3f * g_magnitude_peak) continue;

        float fraction = g_phase_response[k] / (float)M_PI;
        bool  pos      = (g_phase_response[k] >= 0.0f);
        int   h        = (int)(fabsf(fraction) * (float)half_height + 0.5f);
        for (int bx = 0; bx < bin_w - 1; bx++)
            draw_bar(k * bin_w + bx, midline_row, h, pos, PAIR_PHRESP);
    }

    /* Midline marker. */
    attron(COLOR_PAIR(PAIR_LABEL));
    for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
        mvaddch(midline_row, x, '-');
    attroff(COLOR_PAIR(PAIR_LABEL));
}

/* ===================================================================== */
/* §17  draw_debug — 'D' kernel numeric table                            */
/* ===================================================================== */
/*
 *  Print the K filter taps as a numeric table, top-left.  Useful for
 *  reading off the actual kernel weights.
 */
static void draw_kernel_table_overlay(void)
{
    if (!g_show_kernel_table) return;

    int x = 2, y = 2;
    if (y + K + 2 >= LINES - 1) return;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, "Kernel taps  %s  (K = %d, win = %s)",
             filter_name[g_filter_kind], K,
             window_name[g_window_kind]);
    attroff(COLOR_PAIR(PAIR_HINT));

    /* Two columns of K/2 taps for compactness. */
    int rows_per_col = (K + 1) / 2;
    for (int i = 0; i < K; i++) {
        int row_offset = i % rows_per_col;
        int col_offset = (i / rows_per_col) * 22;
        attron(COLOR_PAIR(PAIR_FILTER) | A_BOLD);
        mvprintw(y + 1 + row_offset, x + col_offset,
                 "  k[%2d] = %+8.5f", i, (double)g_kernel_taps[i]);
        attroff(COLOR_PAIR(PAIR_FILTER) | A_BOLD);
    }
}

/* ===================================================================== */
/* §18  hud — status + hint + paused chip                                */
/* ===================================================================== */

static void draw_hud(void)
{
    char status[200];
    snprintf(status, sizeof status,
             " FIR filter  N=%d K=%d  signal:%s  filter:%s  win:%s  "
             "cutoff:%2d  %s  %s ",
             N, K, signal_name[g_signal_kind],
             filter_name[g_filter_kind], window_name[g_window_kind],
             g_cutoff_bin,
             g_auto_sweep_cutoff ? "AUTO  " : "MANUAL",
             g_simulation_paused ? "PAUSED" : "      ");
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
             " q:quit  spc:pause  f:filter  w:window  s:signal "
             " a:auto/manual  +/-:cutoff  d:phase  D:taps  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void)
{
    erase();

    /* Layout: 1 hud + 4 labels + 4 panels + 1 hint = 6 reserved.
     * Phase panel "steals" some vertical space when toggled on. */
    int rows_for_panels = LINES - 6;
    if (rows_for_panels < 12) rows_for_panels = 12;

    int phase_h    = g_show_phase_panel ? rows_for_panels / 5 : 0;
    int main_h     = rows_for_panels - phase_h;
    int input_h    = main_h / 4;
    int filter_h   = main_h / 4;
    int magresp_h  = main_h / 4;
    int output_h   = main_h - input_h - filter_h - magresp_h;

    int input_label_row    = 1;
    int input_top_row      = 2;
    int filter_label_row   = 2 + input_h;
    int filter_top_row     = 3 + input_h;
    int magresp_label_row  = 3 + input_h + filter_h;
    int magresp_top_row    = 4 + input_h + filter_h;
    int output_label_row   = 4 + input_h + filter_h + magresp_h;
    int output_top_row     = 5 + input_h + filter_h + magresp_h;
    int phase_top_row      = output_top_row + output_h
                           + (g_show_phase_panel ? 1 : 0);

    /* Panel labels. */
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(input_label_row,   0, "Input x[n]");
    mvprintw(filter_label_row,  0,
             "Filter h[i]  (K = %d taps, centered)", K);
    mvprintw(magresp_label_row, 0,
             "Magnitude response |H[k]|  (cutoff marked with ':')");
    mvprintw(output_label_row,  0, "Output y[n] = h * x");
    if (g_show_phase_panel)
        mvprintw(phase_top_row - 1, 0,
                 "Phase response arg(H[k])  (linear ramp = linear phase)");
    attroff(COLOR_PAIR(PAIR_LABEL));

    draw_signal_panel(g_input_signal, input_top_row, input_h,
                      g_input_peak, PAIR_INPUT);
    draw_impulse_response_panel(filter_top_row, filter_h);
    draw_magnitude_response_panel(magresp_top_row, magresp_h);
    draw_signal_panel(g_output_signal, output_top_row, output_h,
                      g_output_peak, PAIR_OUTPUT);
    if (g_show_phase_panel)
        draw_phase_response_panel(phase_top_row, phase_h);

    /* Debug overlay. */
    draw_kernel_table_overlay();

    /* HUD always last. */
    draw_hud();
    draw_hint();

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §19  app — signal handlers + main loop + key dispatch                 */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit    = 1;
}

static void cleanup_screen(void) { endwin(); }

static bool app_handle_key(int ch)
{
    switch (ch) {
        case 'q': case 'Q': case 27:  return true;
        case ' ':                     g_simulation_paused = !g_simulation_paused; break;
        case 'f':                     scene_cycle_filter(+1); break;
        case 'F':                     scene_cycle_filter(-1); break;
        case 'w':                     scene_cycle_window(+1); break;
        case 'W':                     scene_cycle_window(-1); break;
        case 's':                     scene_cycle_signal(+1); break;
        case 'S':                     scene_cycle_signal(-1); break;
        case 'a': case 'A':           g_auto_sweep_cutoff = !g_auto_sweep_cutoff; break;
        case '+': case '=': case '.': scene_adjust_cutoff(+1); break;
        case '-':           case ',': scene_adjust_cutoff(-1); break;
        case 'd':                     g_show_phase_panel  = !g_show_phase_panel; break;
        case 'D':                     g_show_kernel_table = !g_show_kernel_table; break;
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
