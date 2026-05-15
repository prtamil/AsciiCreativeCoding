/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * convolution_helloworld.c — animated, minimum-viable 1-D convolution
 *                            demo, written so the algorithm reads as
 *                            pseudocode.  A small kernel slides across
 *                            an input signal; you watch each output
 *                            sample being computed live.
 *
 * DEMO
 *   Three stacked panels.
 *     TOP        input signal x[n]; samples currently under the kernel
 *                are highlighted in bright yellow.
 *     MIDDLE     kernel weights k[i], drawn at the kernel's current
 *                sliding position above the input panel's wave.
 *                Positive weights grow up, negative grow down.
 *     BOTTOM     output signal y[n], filled in by the convolution.
 *
 *   Per frame the kernel auto-slides one position rightward; the
 *   yellow highlight shows you exactly which input samples are
 *   being multiplied this frame.  Press 'a' for manual mode where
 *   '<' / '>' step the kernel one position at a time.
 *
 *   Six built-in kernels (cycle with 'k'):
 *     Identity   {0, 0, 1, 0, 0}     output equals input (no change)
 *     Box        {1, 1, 1, 1, 1}/5   moving average — smooths
 *     Gaussian   {1, 4, 6, 4, 1}/16  smoother smoother
 *     Edge       {-1, 0, +1}         derivative — highlights changes
 *     Sharpen    {-1,-1, 5,-1,-1}    high-pass — amplifies edges
 *     Inverse    {0, 0,-1, 0, 0}     negation
 *
 *   Five built-in input signals (cycle with 's'):
 *     Impulse    one tall sample at the centre, otherwise zero
 *     Step       half negative, half positive
 *     Sum sines  two cosines added (3 cycles + 11 cycles)
 *     Square     periodic square wave
 *     Noise      repeatable pseudo-random
 *
 *   The headline lesson: every smoothing filter, edge detector, blur
 *   effect, echo, reverb, and FIR filter at the bottom of the call
 *   stack is THIS operation — slide kernel, multiply element-wise,
 *   sum.  That's the whole thing.  Different kernels, same operation.
 *
 *   Two debug overlays:
 *     'd' shows the LIVE ARITHMETIC for the currently-highlighted
 *         output sample — see the actual multiply-and-sum
 *         computation that produced it, written out as numbers.
 *     'D' shows the kernel as a numeric table (taps + weights).
 *
 * Study alongside
 *   signal/fir_filter.c          Practical filtering — same operation,
 *                                  with kernels designed in the
 *                                  frequency domain (windowed sinc).
 *   signal/idft_helloworld.c     Filtering by zeroing spectrum bins.
 *                                  Same effect via the convolution-
 *                                  multiplication duality (see T8).
 *   signal/dft_helloworld.c      The DFT is the OTHER bedrock of DSP.
 *                                  Convolution + Fourier are the two
 *                                  foundational operations.
 *
 * Section map
 *   §1   config             constants
 *   §2   clock              monotonic-ns timer + sleep
 *   §3   colors             palette pairs (HUD/HINT theme-invariant)
 *   §4   signal_kinds       enum + display names for the 5 signals
 *   §5   signal_generators  the 5 generators (Impulse, Step, ...)
 *   §6   kernel_kinds       enum + display names for the 6 kernels
 *   §7   kernel_generators  the 6 generators (Identity, Box, ...)
 *   §8   convolve           the headline function (reads as pseudocode)
 *   §9   scene_state        per-frame state + reset
 *   §10  scene_tick         per-frame update orchestrator
 *   §11  scene_input        handle keys
 *   §12  draw_bar           vertical-bar primitive
 *   §13  draw_signal_panel  input + output panel renderer
 *   §14  draw_kernel_panel  kernel panel (centered on current slide)
 *   §15  draw_debug         'd' live-arithmetic + 'D' kernel table
 *   §16  hud                status top-right + hint bottom + PAUSED chip
 *   §17  app                signal handlers + main loop + key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   a                toggle auto-slide <-> manual mode
 *   < / >            (manual) step kernel left / right by 1 sample
 *   k / K            next / previous kernel type
 *   s / S            next / previous input signal
 *   d                toggle live-arithmetic overlay (debug)
 *   D                toggle kernel numeric table top-left (debug)
 *   r                reset (slide back to the start)
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/convolution_helloworld.c \
 *       -o convolution_helloworld -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no DSP background:
 *     1. CONCEPTS — the elevator pitch with references.
 *     2. MENTAL MODEL — analogies, diagrams, the worked example.
 *     3. GUIDED TUTORIAL T1..T9 — nine mini-lessons that derive
 *        convolution from first principles.  Each ends in pseudocode
 *        that maps 1:1 to a function in §5..§8.
 *     4. §1 config — the knobs.
 *     5. §8 convolve — THE function.  Every other line of code in
 *        this file exists to set this up or visualise its output.
 *     6. §5 signal_generators + §7 kernel_generators — concrete
 *        examples of "what goes in" and "what slides over it".
 *     7. §10..§16 — scene tick + rendering plumbing.
 *     8. §17 app — main loop + signal handlers + key dispatch.
 *
 *   Reader who already knows convolution, just wants the code:
 *     §1 config, §8 convolve, §10 scene_tick.  The rest is ncurses
 *     scaffolding.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     g_input_signal[N]            the source signal, real-valued
 *     g_kernel_weights[K]          the K filter taps, real-valued
 *     g_output_signal[N]           the convolved result
 *     g_kernel_position            current sliding position (0..N-K)
 *     output_position, n           outer-loop counter (which output sample)
 *     kernel_tap_index, i          inner-loop counter (which kernel tap)
 *     weighted_sum                 the inner-loop accumulator
 *
 *   Inside tight loops single-letter counters (`n`, `i`, `k`) stay
 *   short where context is one line away.
 *
 * Background you NEED
 *   - Basic arithmetic: addition, multiplication, the meaning of "sum".
 *     Convolution doesn't require any complex numbers, calculus, or
 *     deep maths.
 *   - For the duality teasers (T8): a passing familiarity with the
 *     DFT (signal/dft_helloworld.c).  Not strictly required.
 *
 * Background you do NOT need
 *   - Linear algebra, complex analysis, or differential equations.
 *   - The Fourier transform.
 *   - Calculus.  Convolution is a finite sum, not an integral.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * What is convolution, in one sentence?
 *   Slide a small "kernel" k[0..K-1] across an input signal x[n], and
 *   at each position compute the sum of element-wise products.  The
 *   sum is one output sample y[n].  Repeat for every position.
 *
 * Algorithm
 *   Two nested loops.  The OUTER loop walks the kernel's left edge
 *   across the input (n = 0, 1, 2, ..., N - K).  The INNER loop
 *   computes one output sample by summing K element-wise products.
 *
 *   This program animates the outer loop one position per frame so
 *   you can SEE convolution happen — the kernel's bars slide rightward
 *   one position at a time, with the input samples currently under
 *   the kernel highlighted in yellow.
 *
 *   Cost per output sample: K multiplies + K adds.  Cost per full
 *   convolution: (N - K + 1) * K = O(N * K) operations.  For long
 *   signals and long kernels this becomes the bottleneck — at which
 *   point the FFT-based approach wins (O(N log N) total).  See
 *   tutorial T8 for the convolution-multiplication duality that
 *   makes the FFT approach work.
 *
 * Math
 *   What this file computes (called CROSS-CORRELATION, not strict
 *   mathematical convolution):
 *
 *     y[n]  =  sum over i in [0, K)  of  k[i] * x[n + i]
 *
 *   Strict mathematical convolution flips the kernel:
 *
 *     y[n]  =  sum over i in [0, K)  of  k[i] * x[n - i]
 *
 *   For SYMMETRIC kernels (Identity, Box, Gaussian) the two are
 *   identical.  For ASYMMETRIC kernels (Edge, Sharpen) they produce
 *   mirror-image outputs.  T3 covers the difference; we use cross-
 *   correlation throughout because it's slightly more intuitive to
 *   visualize ("the kernel slides across the input as drawn").
 *
 *   The convolution-multiplication duality (T8):
 *
 *     time-domain convolution     <=>   frequency-domain multiplication
 *     time-domain multiplication  <=>   frequency-domain convolution
 *
 *   This is WHY the FFT is so important — it turns expensive O(N*K)
 *   convolution into cheap O(N log N) FFT + O(N) multiply +
 *   O(N log N) inverse FFT.  But understand the time-domain version
 *   first.  signal/idft_helloworld.c demonstrates filtering done in
 *   the frequency domain (zero out spectrum bins → IDFT) for the
 *   exact same effect.
 *
 * Performance / boundaries
 *   - N = 64 input samples, K = 5..15 kernel taps.  Per-frame cost:
 *     trivial (~1000 multiplies).
 *   - We compute the FULL output array every frame; the animation
 *     only changes which output sample is currently highlighted.
 *   - Boundary handling: when the kernel runs off the right edge of
 *     the input, output samples in that region are not defined.  We
 *     zero them.
 *   - All math is real-valued (no complex numbers).  Convolution is
 *     defined for complex sequences too but we never need it.
 *
 * Rendering
 *   Three vertical-bar panels stacked top-to-bottom.  The kernel
 *   panel's bars are positioned ABOVE the input samples they are
 *   currently multiplying — so the eye can connect "kernel weight,
 *   input sample, product" in one glance.
 *
 *   Two optional debug overlays show the live arithmetic numerically
 *   ('d') and the kernel weights as a table ('D').
 *
 * References
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapters 6-7 (convolution).  Free online;
 *     the gentlest introduction I know of.
 *   Press et al., "Numerical Recipes in C", Chapter 13 (Fourier and
 *     spectral applications, including FFT-based convolution).
 *   Bracewell, R. N., "The Fourier Transform and Its Applications".
 *     The textbook reference for the convolution-multiplication
 *     duality.
 *   https://en.wikipedia.org/wiki/Convolution
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   Convolution is the answer to "what does this filter do to my
 *   signal?".  The filter is a small array of weights (the kernel);
 *   the operation is "slide the weights across the signal,
 *   multiply, sum".  Different kernels do different things, but the
 *   operation is identical for every kernel ever invented.
 *
 *   Smoothing filters, edge detectors, blurs, sharpening, echo,
 *   reverb, FIR audio filters, convolutional neural networks — at
 *   the bottom of the call stack, all of them are THIS operation.
 *   This file makes one slide per frame so you can WATCH it happen
 *   on a tiny example (N = 64 input samples, K = 3..15 kernel taps).
 *
 * HOW TO THINK ABOUT IT — THE WOODEN-RULER PICTURE
 *   Imagine you're sliding a small wooden ruler across a sheet of
 *   paper covered in numbers.  At each ruler position you read off
 *   K numbers under the ruler, multiply each by a fixed weight
 *   written ON the ruler, add the products, and write the sum on
 *   a third sheet of paper at the position directly below the
 *   ruler's left edge.
 *
 *   Move the ruler one position to the right.  Repeat.  Move,
 *   repeat.  Continue until the ruler runs off the right edge.
 *   The third sheet now contains the convolution.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   THE WORKED EXAMPLE (do this on paper to confirm):     │
 *      │                                                         │
 *      │   input   x  : [ 1   2   3   4   3   2   1   0 ]         │
 *      │                                                         │
 *      │   kernel k   : [ 1   2   1 ] / 4    (= [0.25, 0.5, 0.25])│
 *      │                                                         │
 *      │   slide n=0  :  k[0]*x[0] + k[1]*x[1] + k[2]*x[2]       │
 *      │              =  0.25*1 + 0.5*2 + 0.25*3                 │
 *      │              =  0.25 + 1.0 + 0.75                       │
 *      │              =  2.00.   →  y[0] = 2.00                  │
 *      │                                                         │
 *      │   slide n=1  :  k[0]*x[1] + k[1]*x[2] + k[2]*x[3]       │
 *      │              =  0.25*2 + 0.5*3 + 0.25*4                 │
 *      │              =  0.5 + 1.5 + 1.0                         │
 *      │              =  3.00.   →  y[1] = 3.00                  │
 *      │                                                         │
 *      │   slide n=2  :  k[0]*x[2] + k[1]*x[3] + k[2]*x[4]       │
 *      │              =  0.25*3 + 0.5*4 + 0.25*3                 │
 *      │              =  0.75 + 2.0 + 0.75                       │
 *      │              =  3.50.   →  y[2] = 3.50                  │
 *      │                                                         │
 *      │   ...continue: y[3] = 3.0, y[4] = 2.0, y[5] = 1.0       │
 *      │                                                         │
 *      │   output y  : [ 2.00  3.00  3.50  3.00  2.00  1.00  ]  │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 *   The output is a SMOOTHED version of the input (this kernel is a
 *   weighted moving average — bigger weight in the middle, smaller
 *   weights on the sides).  Try a different kernel and the output
 *   changes character entirely.  Same OPERATION, different OUTPUT.
 *
 * ALGORITHM IN STEPS  (per frame)
 *   1. (If auto-slide) advance the kernel position by 1.  At the
 *      end of the input, wrap back to 0.
 *   2. Compute the FULL output array y[0..N-K] by walking n = 0..N-K
 *      and summing k[i] * x[n+i] for each n.
 *   3. Render: input panel (with current-window highlight), kernel
 *      panel (positioned at slide), output panel, HUD, debug
 *      overlays if toggled.
 *
 * KEY FORMULAS
 *
 *   The operation we run (cross-correlation):
 *     y[n]  =  sum over i in [0, K)  of  k[i] * x[n + i]
 *
 *   Strict mathematical convolution (kernel flipped):
 *     y[n]  =  sum over i in [0, K)  of  k[i] * x[n - i]
 *
 *   The IMPULSE RESPONSE identity (T5):
 *     If x = impulse (one 1 at index n0, zeros elsewhere), then
 *     y[n0 - i] = k[i].  In other words, convolving any kernel with
 *     an impulse PRODUCES THE KERNEL.  This is the DEFINITION of
 *     "impulse response".
 *
 *   Convolution-multiplication duality (T8):
 *     y = x * h     <=>     Y[k] = X[k] * H[k]
 *     (where * is convolution, X/Y/H are Fourier transforms of x/y/h)
 *
 *   Cost:
 *     Direct convolution:    O(N * K) operations.
 *     FFT-based convolution: O(N log N) when K is large.
 *
 * EDGE CASES TO WATCH
 *
 *   - BOUNDARIES.  Output samples y[N-K+1..N-1] are undefined because
 *     the kernel would run off the right end of the input.  Three
 *     common ways to handle this:
 *       (a) Don't compute them (set to 0).  We do this.
 *       (b) Pad the input with zeros.  Gives "linear" convolution
 *           of length N+K-1.
 *       (c) Wrap the input (use x[(n+i) % N]).  Gives "circular"
 *           convolution — what FFT-based convolution naturally
 *           computes.
 *     For visualisation purposes (a) is cleanest; in production
 *     audio code (b) is most common.
 *
 *   - KERNEL NORMALISATION.  Box and Gaussian here are normalised so
 *     the weights sum to 1.  This means the output's overall
 *     amplitude is preserved (low-pass with unity DC gain).  Edge
 *     and Sharpen are NOT normalised — Edge sums to 0 (kills the
 *     constant component) and Sharpen sums to 1 (preserves it but
 *     amplifies high frequencies).
 *
 *   - SIGN MATTERS.  The Edge kernel {-1, 0, +1} is asymmetric; the
 *     cross-correlation we compute gives the negative of the strict
 *     convolution.  Visually this means rising input edges produce
 *     POSITIVE output spikes in our cross-correlation, NEGATIVE in
 *     strict convolution.  We use cross-correlation throughout.
 *
 *   - ASYMMETRIC KERNELS LOOK MIRRORED.  If you take a strictly
 *     asymmetric kernel like {1, 2, 3} and want strict convolution,
 *     you must flip it before passing to a cross-correlation routine.
 *     Equivalently, write the cross-correlation routine and load
 *     the kernel reversed.  The two are interchangeable.
 *
 * HOW TO VERIFY
 *   - Press 'k' until kernel = IDENTITY.  The output panel should
 *     equal the input panel exactly (just shifted by K/2 samples).
 *
 *   - Press 'k' to BOX.  The output is a smoothed version of the
 *     input — rising and falling edges become ramps.
 *
 *   - Press 'k' to EDGE, then 's' to STEP.  The output is a single
 *     spike at the step location.  This is "differentiation by
 *     convolution".
 *
 *   - Press 's' to IMPULSE.  The output of ANY kernel on an impulse
 *     IS THE KERNEL ITSELF (mirrored if cross-correlation, true
 *     identity if strict convolution).  This is the "impulse
 *     response" identity (T5).
 *
 *   - Press 'd' for live arithmetic overlay.  At any frame you'll
 *     see the actual sum being computed: e.g. "y[12] = 0.25*0.7 +
 *     0.50*0.3 + 0.25*0.1 = 0.350" — the multiplication and addition
 *     written out as the program performs it.
 *
 *   - Press 'D' for the kernel table — shows kernel name, length,
 *     and each weight in numeric form.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What convolution is, in plain English
 *   T2   The slide-and-multiply picture (with the worked example)
 *   T3   Strict convolution vs cross-correlation
 *   T4   Five canonical kernels and what each one does
 *   T5   The impulse response identity — the most beautiful identity in DSP
 *   T6   Boundary handling (three ways, why it matters)
 *   T7   O(N * K) cost — when to switch to FFT-based convolution
 *   T8   Convolution-multiplication duality (preview)
 *   T9   How the demo works — auto-slide, manual mode, debug overlays
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT CONVOLUTION IS, IN PLAIN ENGLISH
 * ─────────────────────────────────────────
 * Convolution is "slide a small weight-array across a signal and at
 * each position write down the weighted sum of the values you
 * covered".  That's it.  Nine words.
 *
 * The weight-array is called the KERNEL (or filter, or impulse
 * response — three words for the same thing).  It's typically much
 * shorter than the signal: a 3-tap kernel slides across thousands
 * of samples to produce thousands of output samples.
 *
 * What does the output represent?  It depends entirely on the
 * kernel:
 *
 *     If kernel weights are all positive and sum to 1: the output
 *     is a SMOOTHED version of the input.
 *
 *     If kernel weights have a positive group and a negative group
 *     that sum to 0: the output is a DERIVATIVE-like version that
 *     responds to changes (edges).
 *
 *     If kernel weights amplify the centre and dampen the sides:
 *     the output SHARPENS the input.
 *
 *     If the kernel is a single 1 surrounded by zeros: the output
 *     equals the input (the IDENTITY kernel).
 *
 * Real-world examples that are ALL convolution:
 *   - Audio EQ:           kernel = the EQ's impulse response
 *   - Echo / reverb:      kernel = the room's impulse response
 *   - Image blur:         kernel = a Gaussian (in 2-D)
 *   - Image edge-detect:  kernel = a Sobel or Laplacian (in 2-D)
 *   - CNN convolutional layer:  kernel = the LEARNED filter weights
 *
 * Same operation, different kernels.
 *
 * T2  THE SLIDE-AND-MULTIPLY PICTURE  (with worked example)
 * ─────────────────────────────────────────────────────────
 * The worked example in MENTAL MODEL above showed every step of
 * convolving an 8-sample input with a 3-tap kernel.  Repeat it here
 * with annotation:
 *
 *     position n=0:   k[0]*x[0] + k[1]*x[1] + k[2]*x[2]  →  y[0]
 *                     ↑           ↑           ↑
 *                     |           |           |
 *                     leftmost    middle      rightmost
 *                     of kernel   of kernel   of kernel
 *
 *     position n=1:   k[0]*x[1] + k[1]*x[2] + k[2]*x[3]  →  y[1]
 *                     (every input index incremented by 1)
 *
 *     ...
 *
 *     position n=N-K: k[0]*x[N-K] + k[1]*x[N-K+1] + k[2]*x[N-K+2] → y[N-K]
 *
 * Three observations:
 *
 *   1. THE KERNEL DOESN'T MOVE relative to itself — k[0] is always
 *      the first weight, k[K-1] is always the last.
 *
 *   2. THE INPUT WINDOW SLIDES — at position n we read x[n..n+K-1].
 *
 *   3. ONE OUTPUT PER POSITION — y[n] is the sum of K element-wise
 *      products at this position.
 *
 * In code (§8 convolve):
 *
 *     for output_position in 0..N - K:
 *       weighted_sum = 0
 *       for kernel_tap_index in 0..K - 1:
 *         weighted_sum += kernel[kernel_tap_index]
 *                       * input[output_position + kernel_tap_index]
 *       output[output_position] = weighted_sum
 *
 * Two for-loops.  No complex numbers, no calculus.  This is the
 * entire algorithm.
 *
 * T3  STRICT CONVOLUTION VS CROSS-CORRELATION
 * ───────────────────────────────────────────
 * The math textbook formula for convolution flips the kernel:
 *
 *     y_strict[n]  =  sum over i in [0, K)  of  k[i] * x[n - i]
 *
 * The formula we use (cross-correlation):
 *
 *     y_xcorr[n]   =  sum over i in [0, K)  of  k[i] * x[n + i]
 *
 * The difference is the SIGN of i in the input subscript.  For
 * SYMMETRIC kernels (k[i] = k[K-1-i]) the two operations produce
 * IDENTICAL output.  This file's Identity, Box, Gaussian, Sharpen,
 * and Inverse kernels are symmetric — convolve or cross-correlate,
 * the output is the same.
 *
 * For ASYMMETRIC kernels the outputs are mirror images of each
 * other.  Our Edge kernel {-1, 0, +1} is asymmetric.  In strict
 * convolution it produces NEGATIVE spikes at rising edges; in our
 * cross-correlation it produces POSITIVE spikes.  Both are correct
 * answers to slightly different questions.
 *
 * Why does the textbook flip the kernel?  Because in continuous-
 * time convolution (the integral version), the kernel is mirrored
 * by the variable change t → -t inside the integrand.  Discrete
 * convolution inherits the convention.
 *
 * Why do we use cross-correlation?  Because it's slightly more
 * intuitive to visualize — the kernel slides "as drawn", left to
 * right, with k[0] reading the leftmost input sample.  No mental
 * mirroring required.  And for symmetric kernels it doesn't matter.
 *
 * Practical advice: when reading code in real DSP libraries, check
 * the docs for which convention they use.  numpy.convolve and
 * scipy.signal.convolve do strict convolution; OpenCV's filter2D
 * and most image-processing libraries do cross-correlation.
 *
 * T4  FIVE CANONICAL KERNELS AND WHAT EACH ONE DOES
 * ─────────────────────────────────────────────────
 * The five kernels in this demo are pedagogical archetypes — every
 * useful FIR filter is a variation on these.
 *
 *   IDENTITY   {0, 0, 1, 0, 0}
 *     The single 1 in the centre means the output equals the input
 *     (shifted by K/2 samples).  Useful as a sanity check: convolve
 *     with identity, output should match input.
 *
 *   BOX        {1, 1, 1, 1, 1} / 5         (a "moving average")
 *     Equal weights, normalised to sum to 1.  Computes the average
 *     of K neighbouring input samples.  Smooths the input by
 *     averaging out high-frequency wiggles.  Side-effect: ringing
 *     near sharp edges (the Gibbs phenomenon for box filters).
 *
 *   GAUSSIAN   {1, 4, 6, 4, 1} / 16
 *     Bell-curve weights, normalised to sum to 1.  Smooths like Box
 *     but with LESS ringing — the weighted sum gives more importance
 *     to the centre, less to the edges, so abrupt jumps in the input
 *     produce gentler transitions in the output.  The standard
 *     image-blur kernel; in 2-D, this becomes the bivariate Gaussian
 *     used by Photoshop's Gaussian Blur.
 *
 *   EDGE       {-1, 0, +1}
 *     Subtract left from right.  Output is approximately the
 *     DERIVATIVE of the input — flat where the input is flat, large
 *     positive where the input is rising, large negative where the
 *     input is falling.  In 2-D this becomes the Sobel operator,
 *     used in classical computer vision for edge detection.
 *
 *   SHARPEN    {-1, -1, +5, -1, -1}
 *     Centre weight large positive, side weights small negative.
 *     Sums to 1 (preserves DC), but amplifies any deviation from
 *     "average of neighbours".  Net effect: edges become more
 *     pronounced, smooth regions stay the same.  Mathematically:
 *     sharpen ≈ identity + lambda * (identity - blur), the unsharp
 *     mask formula.
 *
 *   INVERSE    {0, 0, -1, 0, 0}
 *     Identity, but negated.  Output = -input.  Just a sanity check
 *     and demonstration that kernels can be negative everywhere.
 *
 * Try cycling through them with 'k' on different signals.  Each
 * kernel produces a different output character — but the OPERATION
 * (slide, multiply, sum) is identical.
 *
 * T5  THE IMPULSE RESPONSE IDENTITY
 * ─────────────────────────────────
 * Here is the most beautiful identity in DSP:
 *
 *     CONVOLVING ANY KERNEL WITH AN IMPULSE GIVES THE KERNEL.
 *
 * An impulse is a signal that's 1 at one specific index and 0
 * everywhere else: x = [..., 0, 0, 1, 0, 0, ...].  Convolve any
 * kernel k with this and the output is:
 *
 *     y[n] = sum_i k[i] * x[n + i]
 *
 * Only one term in the sum is non-zero (the term where x[n + i] = 1,
 * i.e. n + i = impulse_index, i.e. i = impulse_index - n).  So:
 *
 *     y[n] = k[impulse_index - n]
 *
 * In other words, the output is the kernel's mirror image positioned
 * at the impulse location.  (Strict convolution would give the
 * kernel un-mirrored.)
 *
 * Try it: press 's' to Impulse, then press 'k' to cycle kernels.
 * Each kernel's output is the kernel itself, drawn at the impulse's
 * location.  This is the LITERAL DEFINITION of "impulse response":
 * a filter's impulse response IS the impulse response, by definition.
 *
 * Why does this matter?  Because impulses are easy to generate and
 * easy to measure.  If you record an empty cathedral and clap your
 * hands once, the recording captures the cathedral's impulse
 * response — and convolving any audio with that recording adds the
 * cathedral's reverb to the audio.  This is how convolution reverb
 * works.  Real product, real money, exactly this identity.
 *
 * T6  BOUNDARY HANDLING
 * ─────────────────────
 * Convolution outputs are well-defined for n in [0, N - K].  Beyond
 * that, the kernel would need to read x[n + i] for indices >= N,
 * which don't exist.  Three common ways to handle this:
 *
 *   (a) ZERO BOUNDARY — set undefined outputs to 0.
 *       Simplest.  What this demo does.  Output panel shows
 *       flat-zero region near the right edge.  No artifacts but
 *       no useful output for those samples either.
 *
 *   (b) ZERO PADDING — pretend x[N], x[N+1], ... = 0.
 *       Lets you compute outputs all the way to N + K - 1 — the
 *       "linear" convolution length.  In the output, the right edge
 *       has a "ramp-down" caused by the trailing zeros being
 *       weighted in.  This is what numpy.convolve(mode='full') does.
 *
 *   (c) CIRCULAR / WRAP-AROUND — pretend x[N + i] = x[i].
 *       The signal wraps around so the kernel never runs off the
 *       edge.  This is what FFT-based convolution naturally
 *       computes.  The right and left edges of the output mix
 *       together — desired for periodic signals (audio loops),
 *       weird for non-periodic ones.
 *
 * Real-world choice: audio plugins use (c) inside the FFT loop,
 * then often patch up the seams.  Image processing uses (a) or
 * (b) depending on whether boundary artifacts are tolerable.
 *
 * T7  O(N * K) COST — WHEN TO SWITCH TO FFT-BASED CONVOLUTION
 * ───────────────────────────────────────────────────────────
 * Direct convolution costs N inner iterations × N outer iterations
 * = N * K multiplies + N * K adds.  Tabulated:
 *
 *     N        K       multiplies       comment
 *     ----     -----   ----------       -------
 *     64       5       320              this demo, instant
 *     1024     21      21,504           still trivial
 *     65536    1024    67 million       half a second
 *     65536    65536   ~4 billion       impossible
 *
 * Past about K = 64, FFT-based convolution becomes faster:
 *
 *     1. FFT(input)        → O(N log N)
 *     2. FFT(kernel)       → O(N log N)
 *     3. Multiply spectra  → O(N)
 *     4. Inverse-FFT       → O(N log N)
 *     Total:                 O(N log N)
 *
 * For long kernels this is hugely faster.  At K = 1024, N = 65536:
 *   Direct:   67 million multiplies
 *   FFT-based: 4 * 65536 * 16 ≈ 4 million multiplies (16x faster)
 *
 * The crossover point depends on the SIMD instruction set and the
 * specific CPU but is usually somewhere between K = 64 and K = 256.
 * Below that, direct is faster (less overhead); above, FFT wins.
 *
 * For this hello-world demo, K is at most 15, so direct convolution
 * is the right choice.  signal/fir_filter.c also uses direct
 * convolution since K = 21.  signal/fft_vis.c is where FFT-based
 * processing lives.
 *
 * T8  CONVOLUTION-MULTIPLICATION DUALITY (PREVIEW)
 * ────────────────────────────────────────────────
 * One of the most important identities in all of mathematics:
 *
 *     CONVOLUTION IN TIME = MULTIPLICATION IN FREQUENCY
 *
 * Formally:
 *
 *     y[n]   =  (h * x)[n]              ← time-domain convolution
 *     Y[k]   =  H[k] * X[k]             ← frequency-domain product
 *
 *     where Y = DFT(y), X = DFT(x), H = DFT(h)
 *
 * Two ways to filter a signal:
 *
 *     APPROACH 1 (time domain): convolve with h.  This file.
 *     APPROACH 2 (freq domain): DFT, multiply by H, IDFT.
 *                                signal/idft_helloworld.c demonstrates
 *                                a special case (zeroing high bins =
 *                                low-pass = convolution with sinc).
 *
 * Both produce the same answer.  Approach 1 is conceptually simpler;
 * approach 2 is computationally faster for long kernels (T7).
 *
 * The duality is also the REASON window functions exist in fft_vis.c
 * — a window in time = convolution with the window's spectrum in
 * frequency, which is why windowed FFTs have widened spectral peaks.
 *
 * T9  HOW THE DEMO WORKS
 * ──────────────────────
 * Per frame:
 *
 *   1. Maybe advance kernel position by 1 (auto-slide mode).
 *   2. Compute the FULL output array (cheap: < 1000 multiplies).
 *   3. Render three panels:
 *        Input panel    — bars + yellow highlight on the K samples
 *                         currently under the kernel
 *        Kernel panel   — bars positioned over the input samples
 *                         being multiplied
 *        Output panel   — bars showing the full output array
 *
 * Press 'k' / 's' to cycle kernel / signal.  Press 'a' to switch
 * between auto-slide and manual; in manual, '<' / '>' step by 1.
 *
 * Two debug overlays:
 *
 *   'd' shows the LIVE ARITHMETIC for the current slide position.
 *       You see the actual numbers being multiplied and summed,
 *       written out as text.  Pause ('space') to inspect at leisure.
 *
 *   'D' shows the kernel as a numeric table — name, length, each
 *       weight as a float.  Useful for understanding what the
 *       active filter actually IS.
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
/* §1  config                                                            */
/* ===================================================================== */

#define N 64          /* input signal length              */
#define KERNEL_MAX 15 /* max kernel length                 */
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define AUTO_SLIDE_FRAMES 3 /* frames between auto-slide steps   */

/* Colour pair IDs.  HUD/HINT are theme-invariant per CLAUDE.md
 * HUD Standard. */
enum {
  PAIR_INPUT = 1,     /* input wave bars                              */
  PAIR_OUTPUT = 2,    /* output wave bars                             */
  PAIR_KERNEL = 3,    /* kernel weight bars                           */
  PAIR_KERNEL_HI = 4, /* highlight on input samples under the kernel  */
  PAIR_LABEL = 5,     /* panel labels                                 */
  PAIR_HUD = 6,       /* HUD top status                               */
  PAIR_HINT = 7,      /* bottom hint                                  */
};

/* ===================================================================== */
/* §2  clock                                                             */
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
   * spin a CPU at 100% to redraw 30 times a second. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  colors                                                            */
/* ===================================================================== */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_INPUT, 51, -1);      /* bright cyan      */
    init_pair(PAIR_OUTPUT, 154, -1);    /* yellow-green     */
    init_pair(PAIR_KERNEL, 213, -1);    /* magenta-pink     */
    init_pair(PAIR_KERNEL_HI, 226, -1); /* bright yellow    */
    init_pair(PAIR_LABEL, 244, -1);     /* mid grey         */
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow    */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan      */
  } else {
    init_pair(PAIR_INPUT, COLOR_CYAN, -1);
    init_pair(PAIR_OUTPUT, COLOR_GREEN, -1);
    init_pair(PAIR_KERNEL, COLOR_MAGENTA, -1);
    init_pair(PAIR_KERNEL_HI, COLOR_YELLOW, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* ===================================================================== */
/* §4  signal_kinds — enum + display names                               */
/* ===================================================================== */

typedef enum {
  SIG_IMPULSE = 0, /* one tall sample at the centre                */
  SIG_STEP,        /* zero, then jumps to 1 at the centre          */
  SIG_SINES,       /* sum of two sines                             */
  SIG_SQUARE,      /* periodic square wave                         */
  SIG_NOISE,       /* repeatable pseudo-random                     */
  SIG_COUNT
} SignalKind;

static const char *signal_name[SIG_COUNT] = {
    "Impulse  ", "Step     ", "Sum sines", "Square   ", "Noise    "};

/* ===================================================================== */
/* §5  signal_generators — fill the input signal                         */
/* ===================================================================== */

/*
 * generate_signal — fill `output_signal[0..N-1]` with the chosen
 *   test signal.  All signals are bounded by ±1 so the panel
 *   auto-scaling is consistent.
 *
 * Each case in the switch is a one-liner formula or a tight loop;
 * the CONCEPTS / MENTAL MODEL above explains why we picked these
 * particular signals (Impulse and Step are the "test patterns" that
 * define the impulse and step response of any filter).
 */
static void generate_signal(SignalKind kind, float *output_signal) {
  switch (kind) {
  case SIG_IMPULSE: {
    /* Zero everywhere except a single 1 at the centre.
     * Convolving any kernel with this gives the kernel back —
     * see T5 (the impulse response identity). */
    for (int n = 0; n < N; n++)
      output_signal[n] = 0.0f;
    output_signal[N / 2] = 1.0f;
    break;
  }
  case SIG_STEP: {
    /* Two-level signal: -0.5 for the first half, +0.5 for the
     * second.  Used to demonstrate the "step response" of
     * each kernel — Box smoothes the step into a ramp; Edge
     * produces a single spike at the step location. */
    for (int n = 0; n < N; n++)
      output_signal[n] = (n < N / 2) ? -0.5f : 0.5f;
    break;
  }
  case SIG_SINES: {
    /* Two cosines added: 3 cycles + 11 cycles in the buffer.
     * Demonstrates that low-pass kernels keep the slow 3-cycle
     * component and kill the fast 11-cycle one. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      output_signal[n] = 0.6f * sinf(2.0f * (float)M_PI * 3.0f * t) +
                         0.4f * sinf(2.0f * (float)M_PI * 11.0f * t);
    }
    break;
  }
  case SIG_SQUARE: {
    /* Periodic square wave at 5 cycles per buffer.  Sharp
     * vertical edges expose Gibbs ringing in low-pass
     * kernels and produce strong responses from Edge. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      float arg = 2.0f * (float)M_PI * 5.0f * t;
      output_signal[n] = sinf(arg) >= 0.0f ? 0.7f : -0.7f;
    }
    break;
  }
  case SIG_NOISE: {
    /* Repeatable pseudo-random using a linear congruential
     * generator with seed 1.  Same noise every frame so the
     * convolution result is stable (not jittering). */
    unsigned int seed = 1;
    for (int n = 0; n < N; n++) {
      seed = seed * 1664525u + 1013904223u;
      float r = (float)(seed >> 16) / 65535.0f;
      output_signal[n] = (r - 0.5f) * 1.6f;
    }
    break;
  }
  default:
    break;
  }
}

/* ===================================================================== */
/* §6  kernel_kinds — enum + display names                               */
/* ===================================================================== */

typedef enum {
  KERN_IDENTITY = 0, /* {0, 0, 1, 0, 0}     identity (no change)   */
  KERN_BOX,          /* {1, 1, 1, 1, 1}/5   moving average         */
  KERN_GAUSSIAN,     /* {1, 4, 6, 4, 1}/16  Gaussian-ish blur      */
  KERN_EDGE,         /* {-1, 0, 1}          derivative              */
  KERN_SHARPEN,      /* {-1,-1, 5,-1,-1}    high-pass amplifier     */
  KERN_INVERSE,      /* {0, 0,-1, 0, 0}     negation                 */
  KERN_COUNT
} KernelKind;

static const char *kernel_name[KERN_COUNT] = {"Identity ", "Box      ",
                                              "Gaussian ", "Edge     ",
                                              "Sharpen  ", "Inverse  "};

/* ===================================================================== */
/* §7  kernel_generators — fill the kernel weights                       */
/* ===================================================================== */

/*
 * generate_kernel — fill `kernel_weights` with the chosen kernel and
 *   write the kernel length into *out_length.
 *
 * Each kernel is hand-coded for clarity.  T4 in the GUIDED TUTORIAL
 * walks through what each one does and why.
 */
static void generate_kernel(KernelKind kind, float *kernel_weights,
                            int *out_length) {
  switch (kind) {
  case KERN_IDENTITY: {
    /* Single 1 at the centre, zeros elsewhere.  Output of
     * convolving with this equals the input (shifted by K/2). */
    *out_length = 5;
    kernel_weights[0] = 0.0f;
    kernel_weights[1] = 0.0f;
    kernel_weights[2] = 1.0f;
    kernel_weights[3] = 0.0f;
    kernel_weights[4] = 0.0f;
    break;
  }
  case KERN_BOX: {
    /* Equal weights summing to 1.  Computes the average of
     * K neighbouring samples → smoothing low-pass filter. */
    *out_length = 5;
    for (int i = 0; i < 5; i++)
      kernel_weights[i] = 0.2f;
    break;
  }
  case KERN_GAUSSIAN: {
    /* Bell-curve weights {1, 4, 6, 4, 1} / 16.  Smoother
     * smoother than Box — less ringing on sharp edges. */
    *out_length = 5;
    const float w[] = {1.0f, 4.0f, 6.0f, 4.0f, 1.0f};
    for (int i = 0; i < 5; i++)
      kernel_weights[i] = w[i] / 16.0f;
    break;
  }
  case KERN_EDGE: {
    /* {-1, 0, +1}.  Subtracts left from right → discrete
     * derivative.  Output is large where the input changes
     * rapidly, ~0 where the input is constant. */
    *out_length = 3;
    kernel_weights[0] = -1.0f;
    kernel_weights[1] = 0.0f;
    kernel_weights[2] = +1.0f;
    break;
  }
  case KERN_SHARPEN: {
    /* {-1, -1, +5, -1, -1}.  Centre amplifies, sides
     * subtract.  Sums to 1 (preserves DC) but amplifies
     * deviations from local average → edges get sharper. */
    *out_length = 5;
    kernel_weights[0] = -1.0f;
    kernel_weights[1] = -1.0f;
    kernel_weights[2] = +5.0f;
    kernel_weights[3] = -1.0f;
    kernel_weights[4] = -1.0f;
    break;
  }
  case KERN_INVERSE: {
    /* Identity but negated.  Output = -input.  Mostly a
     * sanity check that negative kernels work. */
    *out_length = 5;
    kernel_weights[0] = 0.0f;
    kernel_weights[1] = 0.0f;
    kernel_weights[2] = -1.0f;
    kernel_weights[3] = 0.0f;
    kernel_weights[4] = 0.0f;
    break;
  }
  default:
    *out_length = 1;
    kernel_weights[0] = 1.0f;
    break;
  }
}

/* ===================================================================== */
/* §8  convolve — the headline function (reads as pseudocode)            */
/* ===================================================================== */
/*
 *  THE FUNCTION YOU ARE ABOUT TO READ
 *  ──────────────────────────────────
 *  convolve() is THIS file's punchline.  Every other line of code
 *  exists to set it up (generate signal + kernel), call it, or
 *  visualise its output.  Read this section carefully — everything
 *  else is plumbing around this 15-line core.
 *
 *  WHAT GOES IN, WHAT COMES OUT
 *  ────────────────────────────
 *    INPUT
 *      input_signal[0..N-1]    real-valued signal samples
 *      kernel_weights[0..K-1]  the K filter taps
 *      kernel_length           K (3 to 15 in this demo)
 *
 *    OUTPUT  (caller provides storage; we fill it)
 *      output_signal[0..N-1]   the convolution result.  Samples
 *                              [0, N-K] are computed; samples beyond
 *                              that are zeroed (boundary handling
 *                              option (a) from T6).
 *
 *  THE MATH WE ARE TRANSLATING TO CODE
 *  ───────────────────────────────────
 *    y[n]  =  sum over i in [0, K)  of  k[i] * x[n + i]
 *
 *    Plain English: each output sample y[n] is the sum, over all K
 *    kernel taps, of (kernel weight × input sample) products.  The
 *    input window slides one position rightward for each output.
 *
 *  PSEUDOCODE  (this maps line-by-line to the code below)
 *  ──────────
 *    for each output position n in [0, N - K]:           ← outer loop
 *      start with weighted_sum = 0                       ← init
 *      for each kernel tap i in [0, K):                  ← inner loop
 *        Step 1.  Read the kernel weight:
 *                 weight = kernel_weights[i]
 *        Step 2.  Read the corresponding input sample:
 *                 sample = input_signal[n + i]
 *        Step 3.  Multiply and add to running sum:
 *                 weighted_sum += weight * sample
 *      store weighted_sum as output_signal[n]            ← output
 *
 *    for each undefined output position past N - K:      ← boundary
 *      output_signal[that_position] = 0                  ← zero it
 *
 *  WHY THIS COMPUTES "FILTERED SIGNAL"
 *  ───────────────────────────────────
 *    Each output sample is a WEIGHTED COMBINATION of K consecutive
 *    input samples.  The weights come from the kernel.  Different
 *    kernels weight differently → different filters.
 *
 *    A "low-pass" kernel weights neighbours equally (or smoothly) to
 *    average out high-frequency wiggles.  A "high-pass" kernel uses
 *    sign-alternating weights to detect rapid changes.  The OPERATION
 *    is the same; the WEIGHTS make it a specific filter.
 *
 *  BOUNDARIES & CONTEXT
 *  ────────────────────
 *    - Cost: O(N * K).  At N = 64, K = 5..15, that's 320..960
 *      multiplies per call.  Negligible at this size; at production
 *      audio sizes (N = 65536, K = 1024) becomes ~67 million per
 *      call and you switch to FFT-based convolution (T7).
 *    - We do NOT mutate the input array.  output_signal is the only
 *      thing written.
 *    - We do NOT allocate.  Caller provides all storage.
 *    - Boundary samples y[N-K+1..N-1] are zeroed.  See T6 for other
 *      options (zero-padding, circular).
 */

static void convolve(const float *input_signal, const float *kernel_weights,
                     int kernel_length, float *output_signal) {
  /* Largest valid output position.  Beyond this the kernel would
   * read past the end of the input. */
  int last_valid_output_position = N - kernel_length;

  /* Pseudocode line: "for each output position n in [0, N - K]" */
  for (int output_position = 0; output_position <= last_valid_output_position;
       output_position++) {

    /* Pseudocode line: "start with weighted_sum = 0". */
    float weighted_sum = 0.0f;

    /* Pseudocode line: "for each kernel tap i in [0, K)" */
    for (int kernel_tap_index = 0; kernel_tap_index < kernel_length;
         kernel_tap_index++) {

      /* ── STEP 1 — read the kernel weight ──────────────────
       * The kernel doesn't move relative to itself: tap 0 is
       * always the leftmost weight, tap K-1 is the rightmost.
       */
      float kernel_weight = kernel_weights[kernel_tap_index];

      /* ── STEP 2 — read the corresponding input sample ────
       * The input WINDOW slides: at output position n we read
       * x[n + 0], x[n + 1], ..., x[n + K - 1].  Each output
       * position reads K consecutive input samples; the window
       * advances by 1 for the next output position. */
      float input_sample = input_signal[output_position + kernel_tap_index];

      /* ── STEP 3 — multiply and accumulate ─────────────────
       * The product (weight × sample) is one term of the
       * weighted sum.  We add K such terms; the running total
       * after the inner loop is the value of this output
       * sample. */
      weighted_sum += kernel_weight * input_sample;
    }

    /* Pseudocode line: "store weighted_sum as output_signal[n]". */
    output_signal[output_position] = weighted_sum;
  }

  /* Boundary: outputs past N - K are not well-defined because the
   * kernel would run off the end of the input.  Zero them.
   * See T6 for alternative boundary-handling strategies. */
  for (int output_position = last_valid_output_position + 1;
       output_position < N; output_position++) {
    output_signal[output_position] = 0.0f;
  }
}

/* ===================================================================== */
/* §9  scene_state                                                       */
/* ===================================================================== */

static float g_input_signal[N];
static float g_output_signal[N];
static float g_kernel_weights[KERNEL_MAX];
static int g_kernel_length;
static int g_kernel_position = 0; /* current slide position    */

static SignalKind g_signal_kind = SIG_SINES;
static KernelKind g_kernel_kind = KERN_GAUSSIAN;

static bool g_simulation_paused = false;
static bool g_auto_slide_enabled = true;
static int g_auto_slide_counter = 0;

/* Debug overlay toggles (preserved across kernel/signal cycling). */
static bool g_show_live_arithmetic = false; /* 'd' overlay */
static bool g_show_kernel_table = false;    /* 'D' overlay */

static void scene_reset(void) {
  g_kernel_position = 0;
  g_auto_slide_counter = 0;
  g_simulation_paused = false;
  g_auto_slide_enabled = true;
  generate_signal(g_signal_kind, g_input_signal);
  generate_kernel(g_kernel_kind, g_kernel_weights, &g_kernel_length);
}

/* ===================================================================== */
/* §10  scene_tick — per-frame update                                    */
/* ===================================================================== */
/*
 *  Three numbered steps that match the per-frame portion of
 *  ALGORITHM IN STEPS in the MENTAL MODEL block.
 */
static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* ── Step 1.  auto-slide kernel position by 1 ──────────────── */
  if (g_auto_slide_enabled) {
    g_auto_slide_counter++;
    if (g_auto_slide_counter >= AUTO_SLIDE_FRAMES) {
      g_auto_slide_counter = 0;
      g_kernel_position++;
      if (g_kernel_position > N - g_kernel_length)
        g_kernel_position = 0; /* wrap to start */
    }
  }

  /* ── Step 2.  compute the FULL output via convolution ───── */
  convolve(g_input_signal, g_kernel_weights, g_kernel_length, g_output_signal);

  /* (Step 3 — render — happens in the main loop after this.) */
}

/* ===================================================================== */
/* §11  scene_input — handle keys                                        */
/* ===================================================================== */

static void scene_step_kernel(int delta) {
  /* Manual slide.  No-op while auto-slide is on. */
  if (g_auto_slide_enabled)
    return;
  g_kernel_position += delta;
  if (g_kernel_position < 0)
    g_kernel_position = 0;
  if (g_kernel_position > N - g_kernel_length)
    g_kernel_position = N - g_kernel_length;
}

static void scene_cycle_kernel(int direction) {
  g_kernel_kind =
      (KernelKind)((g_kernel_kind + direction + KERN_COUNT) % KERN_COUNT);
  generate_kernel(g_kernel_kind, g_kernel_weights, &g_kernel_length);
  if (g_kernel_position > N - g_kernel_length)
    g_kernel_position = N - g_kernel_length;
}

static void scene_cycle_signal(int direction) {
  g_signal_kind =
      (SignalKind)((g_signal_kind + direction + SIG_COUNT) % SIG_COUNT);
  generate_signal(g_signal_kind, g_input_signal);
}

/* ===================================================================== */
/* §12  draw_bar — vertical-bar primitive                                */
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
/* §13  draw_signal_panel — input + output panel renderer                */
/* ===================================================================== */

static void draw_signal_panel(const float *signal, int top_row, int height_rows,
                              int colour_pair, int highlight_left,
                              int highlight_right) {
  /* Per sample: vertical bar from the panel midline.  Positive
   * grows up, negative grows down.  Samples in the half-open
   * range [highlight_left, highlight_right) get an alternate
   * pair (used by the input panel to show which samples the
   * kernel is currently multiplying). */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int spacing = (COLS / N >= 2) ? 2 : 1;
  int columns_to_draw = (COLS / spacing < N) ? COLS / spacing : N;

  for (int n = 0; n < columns_to_draw; n++) {
    float v = signal[n];
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    int pair = (n >= highlight_left && n < highlight_right) ? PAIR_KERNEL_HI
                                                            : colour_pair;
    for (int s = 0; s < spacing; s++)
      draw_bar(n * spacing + s, midline_row, h, pos, pair);
  }
}

/* ===================================================================== */
/* §14  draw_kernel_panel — kernel weights at current slide position    */
/* ===================================================================== */

static void draw_kernel_panel(int top_row, int height_rows) {
  /* Draw the kernel's K bars positioned over the input samples
   * the kernel is currently multiplying.  Bar height encodes the
   * kernel weight (signed: positive grows up, negative grows down).
   * Weights are normalised by the kernel's peak so all kernels
   * fit the panel uniformly. */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int spacing = (COLS / N >= 2) ? 2 : 1;

  /* Find the kernel's weight peak so all weights normalise to fit. */
  float peak = 0.0f;
  for (int i = 0; i < g_kernel_length; i++) {
    float a = fabsf(g_kernel_weights[i]);
    if (a > peak)
      peak = a;
  }
  if (peak < 1e-6f)
    peak = 1.0f;

  for (int i = 0; i < g_kernel_length; i++) {
    int n = g_kernel_position + i;
    if (n >= N)
      continue;
    float v = g_kernel_weights[i] / peak;
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    for (int s = 0; s < spacing; s++)
      draw_bar(n * spacing + s, midline_row, h, pos, PAIR_KERNEL);
  }

  /* Midline marker so the user can see "zero weight". */
  attron(COLOR_PAIR(PAIR_LABEL));
  int mid_w = N * spacing;
  for (int x = 0; x < mid_w && x < COLS; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

/* ===================================================================== */
/* §15  draw_debug — 'd' live arithmetic + 'D' kernel table              */
/* ===================================================================== */
/*
 *  Two non-invasive teaching overlays.  Both READ algorithm state
 *  without modifying it.
 *
 *    'd'  LIVE ARITHMETIC.  Show the actual multiply-and-sum
 *         computation that produced y[g_kernel_position].  Pause
 *         to inspect at leisure.
 *
 *    'D'  KERNEL TABLE.  Print kernel name, length, and each
 *         weight as a numeric value.
 */

static void draw_live_arithmetic_overlay(void) {
  if (!g_show_live_arithmetic)
    return;
  int x = 2, y = 2;
  if (y + g_kernel_length + 2 >= LINES - 1)
    return;

  /* Header. */
  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Live arithmetic for y[%d]:", g_kernel_position);
  attroff(COLOR_PAIR(PAIR_HINT));

  /* Per-tap line: weight * sample = product. */
  float running_sum = 0.0f;
  for (int i = 0; i < g_kernel_length; i++) {
    float w = g_kernel_weights[i];
    float s = g_input_signal[g_kernel_position + i];
    float p = w * s;
    running_sum += p;
    attron(COLOR_PAIR(PAIR_KERNEL_HI) | A_BOLD);
    mvprintw(y + 1 + i, x, "  k[%d]*x[%2d] = %+.3f * %+.3f = %+.4f", i,
             g_kernel_position + i, (double)w, (double)s, (double)p);
    attroff(COLOR_PAIR(PAIR_KERNEL_HI) | A_BOLD);
  }

  /* Total. */
  attron(COLOR_PAIR(PAIR_OUTPUT) | A_BOLD);
  mvprintw(y + 1 + g_kernel_length, x, "  ── sum = %+.4f  =  y[%d]",
           (double)running_sum, g_kernel_position);
  attroff(COLOR_PAIR(PAIR_OUTPUT) | A_BOLD);
}

static void draw_kernel_table_overlay(void) {
  if (!g_show_kernel_table)
    return;
  int x = 2, y = 2;
  /* Push table down if the live-arithmetic overlay is on so they
   * don't collide. */
  if (g_show_live_arithmetic)
    y += g_kernel_length + 3;
  if (y + g_kernel_length + 2 >= LINES - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Kernel table:  %s  (K = %d)", kernel_name[g_kernel_kind],
           g_kernel_length);
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int i = 0; i < g_kernel_length; i++) {
    attron(COLOR_PAIR(PAIR_KERNEL) | A_BOLD);
    mvprintw(y + 1 + i, x, "  k[%2d] = %+.4f", i, (double)g_kernel_weights[i]);
    attroff(COLOR_PAIR(PAIR_KERNEL) | A_BOLD);
  }
}

/* ===================================================================== */
/* §16  hud — status top-right + hint bottom + frame composer            */
/* ===================================================================== */

static void draw_hud(void) {
  char status[200];
  snprintf(
      status, sizeof status,
      " Convolution  N=%d K=%d  signal:%s  kernel:%s  pos:%2d/%-2d  %s  %s ", N,
      g_kernel_length, signal_name[g_signal_kind], kernel_name[g_kernel_kind],
      g_kernel_position, N - g_kernel_length,
      g_auto_slide_enabled ? "AUTO  " : "MANUAL",
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
           " q:quit  spc:pause  a:auto/manual  </>:slide  k:kernel "
           " s:signal  d:arith  D:table  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Layout: 1 HUD + 3 labels + 3 panels + 1 hint = 5 reserved
   * rows.  Split the rest evenly into 3 panels. */
  int rows_for_panels = LINES - 5;
  if (rows_for_panels < 9)
    rows_for_panels = 9;
  int input_h = rows_for_panels / 3;
  int kernel_h = rows_for_panels / 3;
  int output_h = rows_for_panels - input_h - kernel_h;

  int input_label_row = 1;
  int input_top_row = 2;
  int kernel_label_row = 2 + input_h;
  int kernel_top_row = 3 + input_h;
  int output_label_row = 3 + input_h + kernel_h;
  int output_top_row = 4 + input_h + kernel_h;

  /* Panel labels. */
  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(input_label_row, 0,
           "Input x[n]   (yellow = under the kernel right now)");
  mvprintw(kernel_label_row, 0,
           "Kernel k[i]  (positive grows up, negative grows down)");
  mvprintw(output_label_row, 0, "Output y[n]  =  sum_i  k[i] * x[n + i]");
  attroff(COLOR_PAIR(PAIR_LABEL));

  /* Panels. */
  draw_signal_panel(g_input_signal, input_top_row, input_h, PAIR_INPUT,
                    g_kernel_position, g_kernel_position + g_kernel_length);
  draw_kernel_panel(kernel_top_row, kernel_h);
  draw_signal_panel(g_output_signal, output_top_row, output_h, PAIR_OUTPUT, -1,
                    -1); /* no highlight */

  /* Debug overlays (drawn over panels but under HUD). */
  draw_live_arithmetic_overlay();
  draw_kernel_table_overlay();

  /* HUD + hint always last. */
  draw_hud();
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §17  app — signal handlers + main loop + key dispatch                 */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* Async-signal-safe: only set flags; main loop does the work. */
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
    g_auto_slide_enabled = !g_auto_slide_enabled;
    break;
  case '<':
  case ',':
    scene_step_kernel(-1);
    break;
  case '>':
  case '.':
    scene_step_kernel(+1);
    break;
  case 'k':
    scene_cycle_kernel(+1);
    break;
  case 'K':
    scene_cycle_kernel(-1);
    break;
  case 's':
    scene_cycle_signal(+1);
    break;
  case 'S':
    scene_cycle_signal(-1);
    break;
  case 'd':
    g_show_live_arithmetic = !g_show_live_arithmetic;
    break;
  case 'D':
    g_show_kernel_table = !g_show_kernel_table;
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
