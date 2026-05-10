/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fft_helloworld.c — animated, minimum-viable Cooley-Tukey FFT demo.
 *                    Runs the FFT on a moving cosine, displays the
 *                    spectrum live, and verifies its output against
 *                    the textbook O(N^2) DFT every single frame.
 *
 * DEMO
 *   Two stacked panels.  TOP: the input signal x[n] = cos(2*pi*f*n/N)
 *   as a vertical-bar waveform.  BOTTOM: the magnitude spectrum |X[k]|
 *   computed by the FFT, also as vertical bars.  By default the input
 *   frequency f auto-sweeps between bin 1 and bin N/2 - 1; you watch
 *   the spike in the spectrum panel walk left and right while the
 *   wave shape morphs.  Press 'a' to switch to manual mode where
 *   '+' / '-' steps the frequency by 1 bin and ',' / '.' by 0.1 bin.
 *
 *   Every frame, the same input is also processed by the naive O(N^2)
 *   DFT.  The HUD shows the maximum bin-by-bin magnitude difference
 *   between the two spectra; the label "FFT == DFT" appears when the
 *   error stays below 1e-4 (which is essentially always at N = 32 in
 *   single precision, ~1e-6 noise floor).  This is the LIVE PROOF
 *   that the FFT is mathematically equivalent to the DFT — the FFT is
 *   just a faster way to compute the same numbers.
 *
 *   Two debug overlays add depth without clutter:
 *     'd' shows a phase panel — arg(X[k]) for each bin, color-coded
 *         by sign.  Magnitude alone discards phase; this shows what
 *         got thrown away.
 *     'D' shows a numeric table of the eight loudest bins
 *         (k, |X[k]|, arg(X[k])).  Useful for verifying decode by hand.
 *
 * Study alongside
 *   signal/dft_helloworld.c   the DFT-only version of this demo —
 *                              same animated UX, no FFT.  Read it first
 *                              if "what is a DFT?" is the open question.
 *   signal/fft_vis.c          the same FFT scaled up to feature-rich
 *                              demo: 7 signal modes, 4 windows,
 *                              spectrogram waterfall.
 *   signal/epicycles.c        the same DFT machinery applied to closed
 *                              2-D curves (epicycle chain animation).
 *   ncurses_basics/framework.c the canonical animation-framework
 *                              template that this file follows.
 *
 * Section map
 *   §1   config            constants (N, LOG2_N, FPS, sweep, freq range)
 *   §2   clock             monotonic-ns timer + sleep
 *   §3   complex           ComplexNumber type + 8 named helpers
 *   §4   colors            palette pairs (HUD/HINT theme-invariant)
 *   §5   dft_naive         reference O(N^2) DFT (used to verify FFT)
 *   §6   bit_helpers       reverse_low_bits — flip the low bits of an int
 *   §7   bit_reverse       in-place bit-reversal permutation of buffer
 *   §8   butterfly         one Cooley-Tukey radix-2 stage
 *   §9   fft               three-line driver: bit-reverse + log2(N) stages
 *   §10  signal            generate cosine at current frequency
 *   §11  scene_state       App-level state globals + reset
 *   §12  scene_tick        per-frame update: sweep -> generate -> FFT -> verify
 *   §13  scene_input       handle manual freq-adjust keys
 *   §14  draw_bar          vertical-bar primitive
 *   §15  draw_signal       TIME panel renderer
 *   §16  draw_spectrum     FREQ panel renderer
 *   §17  draw_phase        debug 'd' phase panel
 *   §18  draw_arm_table    debug 'D' top-N bin-info table
 *   §19  hud               status top-right + hint bottom + PAUSED chip
 *   §20  app               signal handlers + main loop + key dispatch
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
 *   gcc -std=c11 -O2 -Wall -Wextra signal/fft_helloworld.c \
 *       -o fft_helloworld -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no FFT background:
 *     1. CONCEPTS   — the elevator pitch with references to dig deeper.
 *     2. MENTAL MODEL — analogies, diagrams, invariants in your head.
 *     3. GUIDED TUTORIAL T1..T11 — eleven mini-lessons that derive the
 *        FFT from first principles.  Each ends in pseudocode that
 *        maps 1:1 to a function body in §5..§9.
 *     4. §3 complex — the tiny abstraction that lets the algorithm
 *        code read like the math.  Read this BEFORE §5..§9.
 *     5. §5 dft_naive (the textbook reference) → §6..§9 (the FFT
 *        broken into four small files-within-the-file).
 *     6. §10..§13 scene — how the demo wires the algorithm into a
 *        live ncurses animation.
 *     7. §14..§19 draw + hud — rendering plumbing.
 *     8. §20 app — main loop + signal handlers + key dispatch.
 *
 *   Reader who already knows the FFT and just wants the code:
 *     §1 config, §3 complex, §5 dft_naive, §7-§9 (bit_reverse +
 *     butterfly + fft), §12 scene_tick.  The rest is ncurses scaffolding.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     fft_workspace_buffer        N complex numbers, mutated in place by FFT
 *     dft_reference_buffer        N complex numbers, written by naive DFT
 *     output_bin_index, k         the loop counter for bin k
 *     input_sample_index, n       the loop counter for sample n
 *     twist_angle_radians         the angle inside the DFT's exp(-i*theta)
 *     twiddle, twiddle_step       the running factor + per-step multiplier
 *     even_slot_index             upper slot of a butterfly pair
 *     odd_slot_index              lower slot (gets the twiddle multiply)
 *     stage_width                 size of one butterfly group (2, 4, 8, ..., N)
 *     animation_phase_radians     the AUTO-SWEEP phase (not the FFT phase!)
 *
 *   Inside tight loops where context is one line away, single-letter
 *   counters (`i`, `j`, `k`, `n`) stay short.
 *
 * Background you NEED
 *   - Basic complex arithmetic at the level of "z = a + bi, |z| =
 *     sqrt(a^2 + b^2), exp(i*theta) = cos + i*sin".  §3 reviews the
 *     operations we use; T2 covers the geometry.
 *   - The DFT formula at the level of "X[k] is a weighted sum of
 *     inputs".  signal/dft_helloworld.c is the self-contained intro.
 *
 * Background you do NOT need
 *   - Recursion.  The FFT is presented iteratively; we never recurse.
 *   - Continuous calculus.  Everything is a finite sum, indexed by
 *     integers.
 *   - The proof of why the divide-and-conquer split works at general
 *     N.  Tutorial T3 derives it for radix-2 (N a power of 2) and
 *     that's the only case we implement.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm
 *   COOLEY-TUKEY radix-2 DIT (Decimation-In-Time) FFT.  Splits the
 *   N-point DFT into two N/2-point DFTs of even- and odd-indexed
 *   samples, then recombines the two halves using the BUTTERFLY
 *   operation.  Applied recursively, the splits bottom out after
 *   log_2(N) levels and the total work telescopes from O(N^2) to
 *   O(N log N).
 *
 *   Implemented iteratively, not recursively: we (1) permute the
 *   input by bit-reversal so adjacent-pair butterflies operate on
 *   the right (E, O) slots, then (2) walk log_2(N) stages, each
 *   running N/2 butterflies of doubling width.  The "twiddle" factor
 *   W = exp(-i * 2*pi / stage_width) is updated incrementally inside
 *   each stage (one cos + one sin per stage instead of per
 *   butterfly).
 *
 * Data structure
 *   ONE complex array of length N — the `fft_workspace_buffer`.  It
 *   starts holding the input samples (in the `re` slots, with `im`
 *   zeroed).  Bit-reversal permutes its entries.  Each butterfly
 *   stage mutates it in place.  After log_2(N) stages it holds the
 *   final spectrum X[0..N-1].  No extra array is needed.
 *
 *   We use a 2-float `ComplexNumber` struct (§3) instead of two
 *   parallel float arrays — every operation in the algorithm reads
 *   as ONE named line of code instead of two real/imaginary lines.
 *
 * Rendering
 *   Two stacked vertical-bar panels (input waveform on top, FFT
 *   magnitude spectrum on bottom) plus HUD.  Both panels auto-scale
 *   each frame to their own peak, so the visual is always full-height.
 *
 *   Two optional debug overlays (toggled by 'd' and 'D') add a phase
 *   panel and a numeric bin-info table — both show what magnitude
 *   alone discards.
 *
 * Performance
 *   N = 32 makes both algorithms instant.  We deliberately run BOTH
 *   the FFT and the naive DFT every frame so the HUD can verify the
 *   FFT bin-by-bit against the reference.  The verification cost
 *   (1024 multiplies for the naive DFT) dominates this demo — which
 *   is fine, since the whole point is "look, they agree".
 *
 *   At realistic FFT sizes (N >= 1024) the naive DFT becomes
 *   prohibitively slow and the FFT's O(N log N) is the only viable
 *   option.  See the op-count derivation in tutorial T7.
 *
 * References
 *   Cooley, J. W. & Tukey, J. W. (1965), "An Algorithm for the Machine
 *     Calculation of Complex Fourier Series", Math. Comp. 19, 297-301.
 *     The original five-page paper that started the FFT era.
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapter 12.  Free online; the gentlest
 *     in-depth FFT tutorial I know of.
 *   Press et al., "Numerical Recipes in C", Chapter 12.  The standard
 *     practitioner's reference; less pedagogical but more thorough.
 *   3Blue1Brown, "But what is the Fourier Transform? A visual
 *     introduction", YouTube — the warm-up if Smith's chapters are
 *     too dense.
 *   https://en.wikipedia.org/wiki/Cooley%E2%80%93Tukey_FFT_algorithm
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   The naive DFT computes a weighted sum for every output bin.
 *   That's N outputs × N input contributions per output = N^2 work.
 *
 *   The FFT exploits one observation: bins which differ by N/2 share
 *   most of their work, bins which differ by N/4 share a bit less,
 *   and so on recursively.  Cooley-Tukey is the algorithm that
 *   climbs that sharing tree in log_2(N) iterative stages.  Each
 *   stage does O(N) work; total is O(N log N).
 *
 *   This program runs the FFT on a moving cosine and shows the
 *   spectrum live.  It also runs the naive DFT every frame and
 *   verifies the FFT bin-by-bin.  The agreement is the whole point:
 *   the FFT is the SAME mathematical transform, computed faster.
 *
 * HOW TO THINK ABOUT IT
 *   You want to add up your year's grocery receipts.  Two approaches:
 *     (a) ONE giant pile of 365 receipts — slow, error-prone.
 *     (b) Twelve monthly sub-totals, then sum the twelve.
 *
 *   Same answer either way.  But if you also wanted MONTHLY averages,
 *   QUARTERLY averages, and the YEARLY total, the monthly sub-totals
 *   from (b) are reusable across all three.  You did the arithmetic
 *   once and got many sums for free.
 *
 *   The FFT is exactly this: it pre-computes "even-index sub-DFTs"
 *   and "odd-index sub-DFTs" and combines them with a complex twist
 *   (the twiddle factor) to produce many of the final bin values
 *   at once.
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │  THE BUTTERFLY (one step inside the FFT):              │
 *      │                                                        │
 *      │      in[k]       --o-----------o--    out[k]           │
 *      │                     \\       //                        │
 *      │                       X    (* W^k inside)              │
 *      │                     //       \\                        │
 *      │      in[k+half]  --o-----------o--    out[k+half]      │
 *      │                                                        │
 *      │      out[k]       =  in[k] + W^k * in[k+half]          │
 *      │      out[k+half]  =  in[k] - W^k * in[k+half]          │
 *      │                                                        │
 *      │  Two complex numbers in, two complex numbers out, ONE  │
 *      │  complex multiply does the work.  Drawn as a graph it  │
 *      │  really does look like a butterfly's wings.            │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (per frame)
 *   1. (If auto-sweep) advance the sweep phase, recompute g_freq_bin.
 *   2. Generate the cosine at g_freq_bin into g_signal[N].
 *   3. Promote real signal into the FFT workspace: copy g_signal[n]
 *      into g_fft_workspace_buffer[n].re; zero the imag parts.
 *   4. Bit-reverse-permute the workspace (one O(N log N) loop).
 *   5. Run log_2(N) butterfly stages.  Stage s has butterflies of
 *      width 2^s = 2, 4, 8, ..., N.
 *   6. Compute |X[k]| for k = 0..N/2 (the unique half of the spectrum).
 *   7. Run the naive DFT on the same input into g_dft_reference_buffer.
 *   8. Compute the maximum bin-by-bin error |fft[k] - dft[k]| for HUD.
 *   9. Render: input panel, spectrum panel, optional phase panel,
 *      optional bin table, HUD top-right, hint bottom-left.
 *
 * KEY FORMULAS
 *
 *   The DFT (what we are computing):
 *     X[k] = sum_{n=0..N-1}  x[n] * exp(-i * 2*pi * k * n / N)
 *
 *   The even/odd split (T3 derives this):
 *     X[k]      = E[k] + W^k * O[k]                k = 0..N/2-1
 *     X[k+N/2]  = E[k] - W^k * O[k]                k = 0..N/2-1
 *     where W = exp(-i * 2*pi / N), E and O are N/2-point DFTs
 *     of even-indexed and odd-indexed samples respectively.
 *
 *   The butterfly (the inner-loop shape):
 *     t          = twiddle * buffer[odd_slot]
 *     buffer[odd_slot]  = buffer[even_slot] - t
 *     buffer[even_slot] = buffer[even_slot] + t
 *     twiddle    = twiddle * twiddle_step       (advance W^j to W^(j+1))
 *
 *   Op count (T7 derives this):
 *     Naive DFT:  N^2                    (1024 at N = 32)
 *     FFT:        (N/2) * log_2(N)       (80 at N = 32)
 *
 *   Auto-sweep frequency:
 *     animation_phase_radians += 2*pi / SWEEP_PERIOD_FRAMES
 *     g_freq_bin = FREQ_LO + (sin(animation_phase) + 1) / 2 * (FREQ_HI - FREQ_LO)
 *     (sin gives a smooth there-and-back motion, not a sawtooth jump)
 *
 * EDGE CASES TO WATCH
 *   - N MUST be a power of 2.  Bit-reversal and butterfly stages both
 *     depend on it.  We hard-code N = 32 and LOG2_N = 5.
 *   - Single-precision noise: FFT vs DFT difference is ~1e-7 typical
 *     for N = 32; we declare PASS if it stays below 1e-4 (a generous
 *     margin against any realistic deviation).
 *   - Off-integer frequencies cause spectral leakage — the spike
 *     spreads across nearby bins.  Both the FFT and the DFT show the
 *     SAME leakage; the FFT is not the source of it (the finite
 *     buffer is).  Window functions cure leakage; signal/fft_vis.c
 *     demonstrates them.
 *   - The bit-reversal swap rule "swap only if reversed_index > i"
 *     is essential: without it we would swap each pair twice and
 *     undo the swap.
 *   - Time-domain phase shifts leave |X[k]| INVARIANT.  In auto-
 *     sweep mode the wave shape moves while the spectrum panel
 *     stays stationary at any one frequency.  Press 'd' to see the
 *     phase changes that magnitude throws away.
 *
 * HOW TO VERIFY
 *   - Compile and run.  HUD should show "FFT == DFT" with err
 *     well below 1e-4 (typically ~1e-7).
 *   - Press 'a' to enter manual mode, then '+' until freq = 5.  The
 *     spike should land at bin 5; the magnitude at the spike should
 *     be near N/2 = 16.
 *   - Hit ',' to nudge freq to 4.9.  The single spike should spread
 *     into a 3-bin hill — spectral leakage in action.  This appears
 *     in BOTH the FFT and the (hidden) DFT identically; the err in
 *     the HUD stays tiny.
 *   - Press 'd' to toggle the phase panel.  In auto-sweep, watch the
 *     phase bars wiggle while magnitude bars stay still — magnitude
 *     is invariant under time-shift; phase is not.
 *   - The op-count chip in the HUD reads "FFT 80 vs DFT 1024 ops".
 *     For real-world sizes (N = 65536) the ratio jumps to ~4000x.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What does the FFT actually compute?  (the same DFT, faster)
 *   T2   Complex numbers as 2-D rotations
 *   T3   The naive DFT and its O(N^2) cost
 *   T4   The divide-and-conquer split — even/odd DFTs
 *   T5   Bit reversal — what the recursion leaves behind (worked N=8)
 *   T6   The butterfly — the smallest unit of work
 *   T7   Iterative implementation + the twiddle recurrence
 *   T8   O(N log N) accounting and a benchmark table
 *   T9   Magnitude vs phase — what we display, what we throw away
 *   T10  The animated demo — sweep mode, manual mode, leakage
 *   T11  Pixel space vs cell space (terminal rendering)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT DOES THE FFT ACTUALLY COMPUTE?  (THE SAME DFT, FASTER)
 * ───────────────────────────────────────────────────────────────
 * The Fast Fourier Transform computes the SAME result as the
 * Discrete Fourier Transform.  Same inputs, same outputs, bit-for-
 * bit identical (within floating-point noise).  The "Fast" in FFT
 * refers to running time, not to the math.  The DFT formula is:
 *
 *     X[k] = sum_{n=0..N-1}  x[n] * exp(-i * 2*pi * k * n / N)
 *
 * The naive way to evaluate this is two nested loops: outer over k,
 * inner over n.  For each (k, n) pair you call cos and sin once,
 * multiply by x[n], add to a running sum.  Total: N inner iterations
 * × N outer iterations = N^2 operations.  At N = 8 that's 64; at
 * N = 1024 it's a million; at N = 65536 (a typical audio FFT size)
 * it's over 4 BILLION operations per transform.
 *
 * The FFT cuts this to ~N * log_2(N) operations.  At N = 65536 that
 * is about a million — a 4000x speedup over the naive DFT.  The
 * trick (T3 onward) is divide-and-conquer.
 *
 * This file PROVES the equivalence empirically.  Every frame it
 * computes the spectrum twice — once with the FFT, once with the
 * naive DFT — and compares the two outputs bin by bin.  The HUD
 * prints the maximum difference; it stays around 1e-7 (single-
 * precision noise floor).  The label "FFT == DFT" appears as long
 * as the agreement is below 1e-4.
 *
 * T2  COMPLEX NUMBERS AS 2-D ROTATIONS
 * ────────────────────────────────────
 * A complex number z = a + bi can be thought of as either an algebraic
 * object (real part `a`, imaginary part `b`) or as a 2-D point /
 * arrow at coordinates (a, b).  The two views agree on every
 * operation.  Two facts make complex arithmetic ideal for FFT code:
 *
 *   1. ADDITION is vector addition.  (a + bi) + (c + di) = (a+c) +
 *      (b+d)i.  Geometrically: head-to-tail addition of two arrows.
 *
 *   2. MULTIPLICATION is "rotate-and-stretch".  A complex of
 *      magnitude r and angle theta, multiplied by another, produces
 *      a new complex whose magnitude is the PRODUCT of magnitudes and
 *      whose angle is the SUM of angles.
 *
 * The bridge between angles and complex numbers is EULER'S FORMULA:
 *
 *     exp(i * theta) = cos(theta) + i * sin(theta)
 *
 * The result has magnitude 1 and angle theta — a point on the unit
 * circle.  The FFT (and DFT) is built on this: the "twist" factor
 * exp(-i * 2*pi * k * n / N) is just a unit-circle complex at the
 * angle inside the exponent.  Multiplying a sample by it ROTATES the
 * sample by that angle.
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
 *      └────────────────────────────────────────────────────────┘
 *
 * In code (§3) we use a 2-float struct named `ComplexNumber` and a
 * handful of helpers: complex_add, complex_sub, complex_mul,
 * complex_scale_by_real, complex_from_angle, complex_magnitude.
 * The FFT body (§8) reads as 4 named operations per inner iteration
 * instead of 8 lines of float arithmetic.
 *
 * T3  THE NAIVE DFT AND ITS O(N^2) COST
 * ─────────────────────────────────────
 * Direct evaluation of the DFT formula:
 *
 *   for each output bin k in [0, N):
 *     running_sum = 0 + 0i
 *     for each input sample n in [0, N):
 *       angle    = -2*pi * k * n / N
 *       twist    = exp(i * angle)        ← cos + i*sin
 *       twisted  = x[n] * twist          ← real * complex
 *       running_sum += twisted
 *     X[k] = running_sum
 *
 * Two nested loops, N iterations each, two trig calls per inner
 * iteration.  Total: 2 * N^2 trig calls plus N^2 complex multiplies.
 * Code is in §5; same algorithm as signal/dft_helloworld.c.
 *
 * For N = 32 (this demo) that is 1024 trig calls per frame.  At
 * 30 fps it is 30,720 trig calls per second — completely trivial
 * on any CPU built since 1990.
 *
 * For N = 1024 it is 2 million trig calls per frame — measurably
 * slow at 30 fps.
 *
 * For N = 65536 it is 8.5 BILLION trig calls per frame — completely
 * infeasible.  This is where the FFT becomes essential, not optional.
 *
 * T4  THE DIVIDE-AND-CONQUER SPLIT — EVEN/ODD DFTS
 * ────────────────────────────────────────────────
 * Start with the DFT for an N-point sequence:
 *
 *     X[k] = sum_{n=0..N-1}  x[n] * W^(k*n)        W = exp(-i*2*pi/N)
 *
 * Group the sum into even-index and odd-index terms.  For even n
 * write n = 2m; for odd n write n = 2m+1.  Both ranges 0..N/2-1.
 *
 *     X[k] = sum_m x[2m]   * W^(k*2m)               (even part)
 *          + sum_m x[2m+1] * W^(k*(2m+1))           (odd  part)
 *
 *     X[k] = sum_m x[2m]   * (W^2)^(k*m)
 *          + W^k * sum_m x[2m+1] * (W^2)^(k*m)
 *
 * Now the magic: W^2 = exp(-i*2*pi/N * 2) = exp(-i*2*pi/(N/2)).
 * That is the twiddle for an N/2-point DFT!  Define:
 *
 *     E[k] = sum_m x[2m]   * (W^2)^(k*m)    ← N/2-point DFT of evens
 *     O[k] = sum_m x[2m+1] * (W^2)^(k*m)    ← N/2-point DFT of odds
 *
 * Then:
 *
 *     X[k] = E[k] + W^k * O[k]              for k = 0..N/2-1   (★)
 *
 * What about k in N/2..N-1?  The N/2-point DFTs E and O are only
 * defined for k = 0..N/2-1, but they are naturally PERIODIC with
 * period N/2.  And W^(k+N/2) = W^k * W^(N/2) = W^k * (-1).  So:
 *
 *     X[k+N/2] = E[k] + W^(k+N/2) * O[k]
 *              = E[k] - W^k * O[k]          for k = 0..N/2-1   (★★)
 *
 * (★) and (★★) together are the BUTTERFLY (T6).  They turn one
 * pair (E[k], O[k]) into one pair (X[k], X[k+N/2]).
 *
 * Cost accounting: E and O each cost (N/2)^2 = N^2/4 work.
 * Recombining via (★)/(★★) is N/2 multiplies and N adds.  Total:
 *     2 * (N^2/4) + O(N) = N^2/2 + O(N).
 * Already half the naive cost — and we can apply the same trick
 * recursively to E and to O.  After log_2(N) levels of splitting,
 * total work telescopes to O(N log N).
 *
 * T5  BIT REVERSAL — WHAT THE RECURSION LEAVES BEHIND  (worked N = 8)
 * ───────────────────────────────────────────────────────────────────
 * Imagine running the recursion to its base case for N = 8.  At each
 * level we partition the sequence into evens and odds:
 *
 *   level 0:  x[0]  x[1]  x[2]  x[3]  x[4]  x[5]  x[6]  x[7]
 *   level 1:  (x[0] x[2] x[4] x[6])      (x[1] x[3] x[5] x[7])
 *               evens                       odds
 *   level 2:  ((x[0] x[4]) (x[2] x[6]))   ((x[1] x[5]) (x[3] x[7]))
 *               e-e        e-o            o-e        o-o
 *   level 3:  x[0] x[4] x[2] x[6] x[1] x[5] x[3] x[7]
 *
 * The leaves come out in BIT-REVERSED order.  Compare:
 *
 *     leaf position  : 0    1    2    3    4    5    6    7
 *     leaf binary    : 000  001  010  011  100  101  110  111
 *     element here   : x[0] x[4] x[2] x[6] x[1] x[5] x[3] x[7]
 *     orig binary    : 000  100  010  110  001  101  011  111
 *
 * Each leaf's element index, in binary, is the BIT REVERSAL of the
 * leaf's own position.  Why: at level 1 we picked elements by bit 0
 * (even = bit 0 zero, odd = bit 0 one), so bit 0 of the original
 * index ends up as the HIGHEST bit of the leaf position.  At level 2
 * we split by bit 1, so bit 1 of the original becomes the second-
 * highest bit of the leaf position.  And so on.
 *
 * The ITERATIVE FFT does the recursion's work BOTTOM-UP.  Treating
 * adjacent pairs in the bit-reversed order as "1-point DFTs", it
 * runs the butterfly to combine them into 2-point DFTs (stage 1),
 * then 4-point DFTs (stage 2), and so on, log_2(N) stages total.
 *
 * The bit-reverse permutation at the start (§7) is what rearranges
 * the input so adjacent slots are the (E, O) pairs the butterfly
 * expects.  Without it, the butterfly would multiply the wrong
 * sub-DFTs together.
 *
 * For N = 32 (this demo) the bit-reverse swaps 12 pairs:
 *   (1, 16) (2, 8) (3, 24) (5, 20) (6, 12) (7, 28)
 *   (9, 18) (11, 26) (13, 22) (15, 30) (19, 25) (23, 29).
 *
 * T6  THE BUTTERFLY — THE SMALLEST UNIT OF WORK
 * ─────────────────────────────────────────────
 * The butterfly takes one (E[k], O[k]) pair and produces one
 * (X[k], X[k+half]) pair, in place:
 *
 *     t                = W^k * X[k+half]    ← was O[k] in that slot
 *     X[k+half]        = X[k] - t           ← was E[k] in that slot
 *     X[k]             = X[k] + t
 *
 * Three lines.  ONE complex multiply (W^k * input), one complex
 * add, one complex subtract.  The variable `t` holds the twisted
 * O[k] term; both outputs use it.
 *
 *      ┌────────────────────────────────────────────────────────┐
 *      │                                                        │
 *      │    in[k]      ──●─────────●──── out[k]      = in[k]+t  │
 *      │                  \\     //                             │
 *      │                    ✕   (* W^k inside)                  │
 *      │                  //     \\                             │
 *      │    in[k+half] ──●─────────●──── out[k+half] = in[k]-t  │
 *      │                                                        │
 *      │    where t = W^k * in[k+half]                          │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * Stage s of the iterative algorithm contains exactly N/2 butterflies
 * — N/2 in the FIRST stage (paired adjacent slots) and N/2 in EVERY
 * subsequent stage too.  The groupings just get larger each stage:
 * stage 1 has groups of width 2, stage 2 width 4, ..., stage log_2(N)
 * width N (one giant group).  Per-stage work is always O(N).
 *
 * T7  ITERATIVE IMPLEMENTATION + THE TWIDDLE RECURRENCE
 * ─────────────────────────────────────────────────────
 * The whole iterative outer loop is two lines of pseudocode:
 *
 *     bit_reverse_permute(buffer)
 *     for stage_width in [2, 4, 8, ..., N]:
 *         run_butterfly_stage(buffer, stage_width)
 *
 * Each stage processes the N input slots in groups of `stage_width`.
 * Inside one group the first half holds E values and the second
 * half holds O values.  We pair them up and butterfly.
 *
 *     run_butterfly_stage(buffer, stage_width):
 *       half          = stage_width / 2
 *       twiddle_step  = exp(-i * 2*pi / stage_width)        ← compute ONCE
 *       for group_start in range(0, N, stage_width):
 *         twiddle = 1                                        ← W^0
 *         for j in range(half):
 *           even_slot = group_start + j
 *           odd_slot  = even_slot + half
 *           t                  = twiddle * buffer[odd_slot]
 *           buffer[odd_slot]   = buffer[even_slot] - t
 *           buffer[even_slot] += t
 *           twiddle           *= twiddle_step                ← W^j → W^(j+1)
 *
 * THE TWIDDLE RECURRENCE: the inner-loop variable `twiddle` is the
 * running value of W^j.  Naively we could compute it per j as
 * cos(angle) + i*sin(angle), but cos/sin are expensive.  Instead we
 * compute W = twiddle_step once per stage and multiply twiddle by W
 * each iteration.  Pay one cos+sin per stage; never call trig in
 * the inner loop.
 *
 * For N = 32 that is 5 trig calls total per FFT.  The naive DFT
 * version of this same operation would call trig 2 * 32^2 = 2048
 * times.  ~400x fewer trig calls.
 *
 * Numerical caveat: floating-point error accumulates in `twiddle`
 * over many multiplications.  At N = 32 the drift is far below
 * single-precision noise.  At N > 100,000 a smart implementation
 * would refresh `twiddle` from a fresh trig call every few hundred
 * iterations; we do not bother here.
 *
 * T8  O(N LOG N) ACCOUNTING AND A BENCHMARK TABLE
 * ───────────────────────────────────────────────
 * Each stage processes exactly N/2 butterflies.  There are log_2(N)
 * stages.  Total butterflies: (N/2) * log_2(N).  Each butterfly costs
 * one complex multiply + one complex add + one complex subtract +
 * one complex multiply for the twiddle update.  Lump those into
 * "complex multiplies":
 *
 *     FFT total ≈ N * log_2(N)
 *
 * For comparison the naive DFT is N^2.  Tabulating both:
 *
 *     N         DFT ops          FFT ops         Speedup
 *     ----      -------          -------         -------
 *     8         64               24              ~3x
 *     32        1024             80              13x   ← this demo
 *     128       16,384           896             18x
 *     1,024     1,048,576        10,240          102x
 *     4,096     16,777,216       49,152          341x
 *     65,536    4,294,967,296    1,048,576       4096x
 *
 * The ratio FFT_ops / DFT_ops = log_2(N) / N.  So the FFT becomes
 * proportionally faster as N grows.  At N = 32 the speedup is
 * modest (13x); the demo runs both algorithms every frame just to
 * prove the equivalence.  At realistic audio sizes the ratio is
 * more than 4000 and only the FFT is feasible.
 *
 * The HUD displays the N = 32 numbers live: "FFT 80 vs DFT 1024 ops".
 *
 * T9  MAGNITUDE VS PHASE — WHAT WE DISPLAY, WHAT WE THROW AWAY
 * ────────────────────────────────────────────────────────────
 * Each output bin X[k] is a complex number (two floats).  Two
 * standard scalar views:
 *
 *     magnitude_k = |X[k]|       = sqrt(re^2 + im^2)     "loudness"
 *     phase_k     = arg(X[k])    = atan2(im, re)         "where in cycle"
 *
 * The default spectrum panel of this program (and almost every
 * audio-spectrum analyser) plots magnitude only.  This has a
 * beautiful invariance: SHIFTING A SIGNAL IN TIME CHANGES THE
 * PHASES BUT LEAVES THE MAGNITUDES UNCHANGED.  When auto-sweep is
 * frozen at one frequency, the input wave still drifts a little
 * each frame (because the auto-sweep computes a fresh wave per
 * frame); the magnitude bars stay rock-steady; the phase bars
 * (visible only with 'd') move.
 *
 * Phase is NOT junk, however.  To RECONSTRUCT a signal from its
 * spectrum (inverse FFT) you need both magnitude AND phase.  Audio
 * compression (MP3) keeps both, with very clever compression of the
 * phase data.  Throwing phase away is a one-way operation — fine
 * for visualisation, fatal for round-trip reconstruction.
 *
 * Press 'd' to toggle the phase panel.  Each bin's phase is shown
 * as a vertical bar from a midline: positive phase grows up, negative
 * grows down.  In auto-sweep you'll see the phase bars wiggle while
 * the magnitude bars stay still — that wiggle IS the time-shift
 * information that magnitude discards.
 *
 * T10  THE ANIMATED DEMO — SWEEP MODE, MANUAL MODE, LEAKAGE
 * ─────────────────────────────────────────────────────────
 * The demo loop has two modes:
 *
 *   AUTO-SWEEP (default).  An internal sweep phase advances by
 *   2*pi / SWEEP_PERIOD_FRAMES each frame.  The frequency oscillates
 *   smoothly between FREQ_LO (= 1) and FREQ_HI (= N/2 - 1) following
 *   sin(sweep_phase).  Spike walks across the spectrum panel.
 *
 *   MANUAL ('a' to toggle).  The sweep is frozen.  '+' / '-' steps
 *   the frequency by 1.0 bin (lands on integer bins, clean spike).
 *   ',' / '.' steps by 0.1 bin (lands on fractional bins; the spike
 *   spreads across nearby bins — spectral leakage).
 *
 * Spectral leakage explained quickly: the DFT assumes its N samples
 * are ONE PERIOD of an infinitely-repeating signal.  If the actual
 * signal's frequency does NOT fit an integer number of cycles in N
 * samples, the implicit periodic extension has a discontinuity at
 * the wrap-around point.  That discontinuity contains many high-
 * frequency components, so the energy of the original tone "leaks"
 * across nearby bins.  signal/fft_vis.c demonstrates the cure
 * (window functions) interactively.
 *
 * T11  PIXEL SPACE VS CELL SPACE  (terminal rendering)
 * ────────────────────────────────────────────────────
 * The screen has cells (rows × cols).  In a typical monospace font,
 * cells are about TWICE AS TALL as wide, so a "circle" drawn in cell
 * coordinates would look squashed.
 *
 * For this hello-world demo we never draw circles, so we stay in
 * CELL space throughout.  But for orientation:
 *
 *   - The TIME panel maps one input sample n to one column of cells.
 *     The sample value is the height of a vertical bar from the
 *     panel's midline.  Positive samples grow upward as '|', negative
 *     grow downward as '.'.
 *
 *   - The FREQ panel maps one bin k to one or two columns (depending
 *     on terminal width).  The magnitude is the height of a vertical
 *     bar from the panel's bottom edge.
 *
 *   - The (optional) PHASE panel below FREQ does the same with phase
 *     instead of magnitude, except phase can be negative so bars
 *     grow up or down from a midline.
 *
 * Frame layout per frame (computed in §19):
 *
 *     row 0           HUD top-right + PAUSED chip
 *     row 1           panel labels
 *     rows 2..mid     TIME panel (signal waveform)
 *     row mid+1       FREQ panel label
 *     rows ..bot      FREQ panel (magnitude spectrum)
 *     rows ..         PHASE panel if 'd' is on
 *     row LINES-1     bottom-left HINT
 *
 * The PHASE panel "steals" rows from the FREQ panel when toggled
 * on, so the overall layout stays inside the screen.
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
/* §1  config — every constant + enum lives here                         */
/* ===================================================================== */

#define N                  32        /* MUST be a power of 2.  Both the   */
                                     /* bit-reverse and the butterfly     */
                                     /* loop depend on this.              */
#define LOG2_N              5        /* log_2(N).  HARD-CODED — keep      */
                                     /* in sync with N above.             */
#define N_HALF             (N / 2)
#define RENDER_FPS         30
#define RENDER_TICK_NS     (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 90       /* one full sweep ≈ 3 seconds        */
#define FREQ_LO            1.0f
#define FREQ_HI            ((float)N * 0.5f - 1.0f)
#define ARM_TABLE_ROWS      8        /* 'D' overlay shows top N bins      */

/* Colour pair IDs.  PAIR_HUD/HINT are theme-invariant per CLAUDE.md
 * HUD Standard.  PAIR_GHOST is reserved for the phase panel's midline. */
enum {
    PAIR_SIG    = 1,     /* time-domain wave              */
    PAIR_SPEC   = 2,     /* spectrum bars (medium)        */
    PAIR_SPIKE  = 3,     /* spectrum bar at the peak bin  */
    PAIR_LABEL  = 4,     /* panel labels                  */
    PAIR_HUD    = 5,     /* HUD top status                */
    PAIR_HINT   = 6,     /* bottom hint                   */
    PAIR_PHASE_POS = 7,  /* phase panel positive bars     */
    PAIR_PHASE_NEG = 8,  /* phase panel negative bars     */
};

/* ===================================================================== */
/* §2  clock — monotonic ns timer + nanosecond sleep                     */
/* ===================================================================== */

static long long clock_now_ns(void)
{
    /* Purpose : return monotonic clock in nanoseconds.
     * Why     : the main loop subtracts now() values to figure out
     *           how long to sleep before the next frame slot. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    /* Block until `ns` nanoseconds have elapsed.  Without this, the
     * main loop would spin a CPU at 100% just to redraw 30 times a
     * second. */
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  complex — the ComplexNumber abstraction (the entire library)      */
/* ===================================================================== */
/*
 *  WHY THIS EXISTS
 *  ───────────────
 *  Without the abstraction, every complex operation in the FFT
 *  butterfly would be 8 lines of float arithmetic per inner-loop
 *  iteration — one for each real/imaginary component of each step.
 *  With it, the same butterfly is FOUR named operations.  Same
 *  arithmetic, much less syntactic noise, and now every line of the
 *  butterfly reads exactly like the math.
 *
 *      // Without the abstraction:
 *      float t_re = twiddle_re * re[dn] - twiddle_im * im[dn];
 *      float t_im = twiddle_re * im[dn] + twiddle_im * re[dn];
 *      re[dn] = re[up] - t_re;
 *      im[dn] = im[up] - t_im;
 *      re[up] += t_re;
 *      im[up] += t_im;
 *      ... and twiddle update is another 3 lines ...
 *
 *      // With the abstraction:
 *      ComplexNumber t = complex_mul(twiddle, buffer[dn]);
 *      buffer[dn]      = complex_sub(buffer[up], t);
 *      buffer[up]      = complex_add(buffer[up], t);
 *      twiddle         = complex_mul(twiddle, twiddle_step);
 *
 *  Why not <complex.h>?  The standard library has `float complex`
 *  and uses the operators `+` `-` `*` directly on it, but the type
 *  and operators feel like compiler magic.  A 2-float struct with
 *  named helpers is more honest about what's happening, prints
 *  cleanly in a debugger, and is portable to any C11 compiler.  The
 *  eight names below are the entire complex-number library this
 *  file needs.
 */

typedef struct {
    float re;     /* real part      (the "x" coordinate)  */
    float im;     /* imaginary part (the "y" coordinate)  */
} ComplexNumber;

/* Identities. */
static const ComplexNumber complex_zero = { 0.0f, 0.0f };
static const ComplexNumber complex_one  = { 1.0f, 0.0f };

static inline ComplexNumber complex_make(float real_part, float imag_part)
{
    return (ComplexNumber){ real_part, imag_part };
}

/*
 * complex_from_angle — Euler's formula:  exp(i * theta) = cos + i * sin.
 *   Result always sits ON the unit circle (magnitude 1).  This is the
 *   bridge between an angle and a complex multiplication; the FFT uses
 *   it everywhere a "twiddle" is needed.
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
 * complex_sub — vector subtraction.  a - b is the arrow from b to a.
 */
static inline ComplexNumber complex_sub(ComplexNumber a, ComplexNumber b)
{
    return complex_make(a.re - b.re, a.im - b.im);
}

/*
 * complex_mul — full complex multiplication (rotate-AND-stretch).
 *   Result's magnitude = product of magnitudes; angle = sum of angles.
 *   Multiplying any z by a UNIT-CIRCLE complex (magnitude 1) is pure
 *   rotation by that complex's angle — exactly what the butterfly's
 *   twiddle multiplication does.
 *
 *   Derivation:
 *     (a.re + i*a.im) * (b.re + i*b.im)
 *   = a.re*b.re + i*a.re*b.im + i*a.im*b.re + i^2*a.im*b.im
 *   = (a.re*b.re - a.im*b.im) + i*(a.re*b.im + a.im*b.re)
 */
static inline ComplexNumber complex_mul(ComplexNumber a, ComplexNumber b)
{
    return complex_make(a.re * b.re - a.im * b.im,
                        a.re * b.im + a.im * b.re);
}

/*
 * complex_scale_by_real — multiply by a real scalar (uniform stretch
 *   without rotation).  Used by the naive DFT (since the input signal
 *   is real-valued).
 */
static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z)
{
    return complex_make(scale * z.re, scale * z.im);
}

/*
 * complex_magnitude — length of the arrow:  |z| = sqrt(re^2 + im^2).
 *   Used by the spectrum panel ("loudness" of each bin) and by the
 *   verification step (max bin-by-bin difference between FFT and DFT).
 *   Phase is discarded.
 */
static inline float complex_magnitude(ComplexNumber z)
{
    return sqrtf(z.re * z.re + z.im * z.im);
}

/* ===================================================================== */
/* §4  colors — palette pair init                                        */
/* ===================================================================== */

static void colors_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_SIG,        51, -1);   /* bright cyan    */
        init_pair(PAIR_SPEC,      154, -1);   /* yellow-green   */
        init_pair(PAIR_SPIKE,      46, -1);   /* bright green   */
        init_pair(PAIR_LABEL,     244, -1);   /* mid grey       */
        init_pair(PAIR_HUD,       226, -1);   /* bright yellow  */
        init_pair(PAIR_HINT,       51, -1);   /* bright cyan    */
        init_pair(PAIR_PHASE_POS, 213, -1);   /* magenta-pink   */
        init_pair(PAIR_PHASE_NEG, 117, -1);   /* sky blue       */
    } else {
        init_pair(PAIR_SIG,        COLOR_CYAN,    -1);
        init_pair(PAIR_SPEC,       COLOR_GREEN,   -1);
        init_pair(PAIR_SPIKE,      COLOR_GREEN,   -1);
        init_pair(PAIR_LABEL,      COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,        COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,    -1);
        init_pair(PAIR_PHASE_POS,  COLOR_MAGENTA, -1);
        init_pair(PAIR_PHASE_NEG,  COLOR_BLUE,    -1);
    }
}

/* ===================================================================== */
/* §5  dft_naive — the textbook O(N^2) DFT (used to verify the FFT)      */
/* ===================================================================== */
/*
 *  Purpose         : compute the discrete Fourier transform exactly
 *                    by direct evaluation of the formula.  Every frame
 *                    we run BOTH this and the FFT, then compare bin
 *                    by bin to verify they agree.
 *
 *  Pseudocode      :
 *      for each output bin k in [0, N):
 *        running_sum = 0 + 0i
 *        for each input sample n in [0, N):
 *          twist_angle = -2*pi * k * n / N
 *          twist       = exp(i * twist_angle)            ← unit circle
 *          twisted     = x[n] * twist                    ← real * complex
 *          running_sum = running_sum + twisted
 *        output[k]   = running_sum
 *
 *  Mental model    : N parallel "lock-in detectors", one per integer
 *                    frequency.  When the input contains that
 *                    frequency, the twists undo its rotation and the
 *                    sum is large; otherwise the twisted samples
 *                    point every-which-way and cancel.
 *
 *  Inputs/outputs  : input_signal[N] (real samples) /
 *                    output_spectrum[N] (complex bins).
 *
 *  Why it exists   : as the GROUND TRUTH against which we compare the
 *                    FFT output.  Every frame's HUD displays the
 *                    maximum bin-by-bin difference; "FFT == DFT"
 *                    label appears as long as the agreement is below
 *                    1e-4 (typically ~1e-7).
 */
static void compute_dft_naive(const float   *input_signal,
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
/* §6  bit_helpers — flip the low bits of an int                         */
/* ===================================================================== */
/*
 *  Why this exists
 *  ───────────────
 *  The iterative FFT requires the INPUT to be permuted by bit
 *  reversal before any butterfly stages run.  See T5 for the
 *  derivation; in short, the recursive even/odd splits leave the
 *  base-case 1-point DFTs in bit-reversed order, and the iterative
 *  version starts from those.
 *
 *  reverse_low_bits() flips the low `bit_count` bits of `value` —
 *  the elemental operation §7 needs once per index.
 */

static int reverse_low_bits(int value, int bit_count)
{
    /* For value = 13 (binary 01101), bit_count = 5:
     *   bit 0 of value is 1 → goes to position 4 in result
     *   bit 1 of value is 0 → goes to position 3
     *   bit 2 of value is 1 → goes to position 2
     *   bit 3 of value is 1 → goes to position 1
     *   bit 4 of value is 0 → goes to position 0
     *   result = 10110 = 22.
     * O(bit_count) work per call; called N times during permutation.
     */
    int reversed_value = 0;
    for (int bit_position = 0; bit_position < bit_count; bit_position++)
        reversed_value |=
            ((value >> bit_position) & 1) << (bit_count - 1 - bit_position);
    return reversed_value;
}

/* ===================================================================== */
/* §7  bit_reverse — in-place bit-reversal permutation of the buffer     */
/* ===================================================================== */
/*
 *  Purpose         : permute the buffer so that adjacent slots hold
 *                    the (E, O) pairs the iterative butterfly expects.
 *
 *  Pseudocode      :
 *      for i in 0..N-1:
 *        reversed_index = reverse_low_bits(i, log_2(N))
 *        if reversed_index > i:                ← important!
 *          swap(buffer[i], buffer[reversed_index])
 *
 *  Mental model    : a one-pass UNSORTING.  Every pair of indices
 *                    that are bit-reverses of each other (and are
 *                    not the same index — like 13 ↔ 22 above) needs
 *                    to be swapped exactly once.  The "rev > i" rule
 *                    enforces that "exactly once" — without it we
 *                    would visit each pair twice and undo the swap.
 *
 *  Inputs/outputs  : ComplexNumber buffer[N], mutated in place.
 *
 *  Why it exists   : the DFT/FFT correspondence (T5) requires this
 *                    permutation as the FFT's first step.  Without it
 *                    the butterflies would multiply the wrong
 *                    sub-DFTs together.  Cost: O(N log N) bit flips
 *                    + O(N) struct copies.  At N = 32 that is 160
 *                    bit operations + 12 swaps — instant.
 */

static void bit_reverse_permute(ComplexNumber *buffer)
{
    for (int original_index = 0; original_index < N; original_index++) {
        int reversed_index = reverse_low_bits(original_index, LOG2_N);
        if (reversed_index > original_index) {
            ComplexNumber temp           = buffer[original_index];
            buffer[original_index]       = buffer[reversed_index];
            buffer[reversed_index]       = temp;
        }
    }
}

/* ===================================================================== */
/* §8  butterfly — one Cooley-Tukey radix-2 stage                        */
/* ===================================================================== */
/*
 *  Purpose         : run one of the FFT's log_2(N) butterfly stages
 *                    in place on the workspace buffer.
 *
 *  Pseudocode      :
 *      half          = stage_width / 2
 *      twiddle_step  = exp(-i * 2*pi / stage_width)        ← compute ONCE
 *      for each group of stage_width consecutive elements:
 *        twiddle = 1                                        ← W^0
 *        for j in 0..half-1:
 *          even_slot = group_start + j           (first half of group)
 *          odd_slot  = group_start + j + half    (second half of group)
 *          t                  = twiddle * buffer[odd_slot]   ← complex MUL
 *          buffer[odd_slot]   = buffer[even_slot] - t        ← complex SUB
 *          buffer[even_slot]  = buffer[even_slot] + t        ← complex ADD
 *          twiddle            = twiddle * twiddle_step       ← complex MUL
 *
 *  Mental model    : draw it as a graph and every (even_slot, odd_slot)
 *                    pair forms a butterfly's wings (two crossing
 *                    diagonals).  See the diagram in MENTAL MODEL.
 *
 *  Inputs/outputs  : ComplexNumber buffer[N], in-place mutation.
 *                    stage_width = 2, 4, 8, ..., N (one of the log_2(N)
 *                    stages).
 *
 *  Why it exists   : the heart of the FFT.  Every "share work between
 *                    bins" claim from T1 happens inside this function
 *                    body.  Cost: N/2 butterflies × O(1) per butterfly
 *                    = O(N) per stage.  log_2(N) stages → O(N log N)
 *                    overall.
 *
 *  Twiddle recurrence note (T7): we update `twiddle` by multiplication
 *  rather than calling cos+sin per iteration.  This is the single
 *  practical optimisation worth understanding; without it the inner
 *  loop would call trig 2 * (N/2) = N times per stage, vs the 2 we
 *  pay (in `complex_from_angle`).
 */

static void run_butterfly_stage(ComplexNumber *buffer, int stage_width)
{
    int half = stage_width / 2;

    ComplexNumber twiddle_step =
        complex_from_angle(-2.0f * (float)M_PI / (float)stage_width);

    for (int group_start = 0; group_start < N; group_start += stage_width) {

        /* Twiddle starts at W^0 = 1 + 0i, advances by twiddle_step. */
        ComplexNumber twiddle = complex_one;

        for (int j = 0; j < half; j++) {
            int even_slot_index = group_start + j;
            int odd_slot_index  = even_slot_index + half;

            /* t          = twiddle * buffer[odd_slot]   (twist of O[k])  */
            ComplexNumber t = complex_mul(twiddle, buffer[odd_slot_index]);

            /* buffer[odd_slot]  = buffer[even_slot] - t  (lower output) */
            buffer[odd_slot_index] =
                complex_sub(buffer[even_slot_index], t);

            /* buffer[even_slot] = buffer[even_slot] + t  (upper output) */
            buffer[even_slot_index] =
                complex_add(buffer[even_slot_index], t);

            /* Advance W^j → W^(j+1) — see T7 for the recurrence. */
            twiddle = complex_mul(twiddle, twiddle_step);
        }
    }
}

/* ===================================================================== */
/* §9  fft — the three-line driver                                       */
/* ===================================================================== */
/*
 *  All of the FFT's complexity is hidden inside §7 + §8.  The driver
 *  itself is THREE lines of work:
 *
 *      1. bit-reverse the input
 *      2. run one butterfly stage of width 2
 *      3. run one butterfly stage of width 4
 *      ...
 *      log_2(N) + 1.  return — buffer holds X[0..N-1].
 *
 *  Read it once and you have read the whole FFT.  Everything
 *  interesting is in §7 bit_reverse_permute and §8 run_butterfly_stage.
 */

static void fft(ComplexNumber *buffer)
{
    bit_reverse_permute(buffer);
    for (int stage_width = 2; stage_width <= N; stage_width <<= 1)
        run_butterfly_stage(buffer, stage_width);
}

/* ===================================================================== */
/* §10  signal — generate the test input                                 */
/* ===================================================================== */

static void generate_cosine(float frequency_bin, float *output_signal)
{
    /* x[n] = cos(2*pi * frequency_bin * n / N).
     *
     * frequency_bin can be FRACTIONAL.  Off-integer values cause
     * spectral leakage: the spike spreads across nearby bins.  This
     * is normal DFT behaviour, not a bug.  See T10 in the tutorial.
     *
     * Pseudocode:
     *   for n in 0..N-1:
     *     phase = 2 * pi * frequency_bin * n / N
     *     output_signal[n] = cos(phase)
     */
    for (int sample_index = 0; sample_index < N; sample_index++) {
        float phase_radians = 2.0f * (float)M_PI
                            * frequency_bin
                            * (float)sample_index
                            / (float)N;
        output_signal[sample_index] = cosf(phase_radians);
    }
}

/* ===================================================================== */
/* §11  scene_state — globals + reset                                    */
/* ===================================================================== */

static float         g_signal[N];                    /* time-domain wave   */
static ComplexNumber g_fft_workspace_buffer[N];      /* X[k] from FFT      */
static ComplexNumber g_dft_reference_buffer[N];      /* X[k] from DFT      */
static float         g_magnitude[N_HALF + 1];        /* |X[k]| (one-sided) */
static float         g_magnitude_peak = 1.0f;        /* for autoscale      */
static float         g_max_verification_error = 0.0f;

static float g_freq_bin              = FREQ_LO;
static bool  g_simulation_paused     = false;
static bool  g_auto_sweep_enabled    = true;
static float g_animation_phase_radians = 0.0f;       /* sweep's own phase  */

/* Debug overlay toggles. */
static bool  g_show_phase_panel      = false;
static bool  g_show_arm_table        = false;

static void scene_reset(void)
{
    g_freq_bin                 = FREQ_LO;
    g_auto_sweep_enabled       = true;
    g_animation_phase_radians  = 0.0f;
    g_simulation_paused        = false;
}

/* ===================================================================== */
/* §12  scene_tick — per-frame update orchestrator                       */
/* ===================================================================== */
/*
 *  Purpose : advance state by ONE frame.  All the per-frame work,
 *            in seven numbered steps that match the ALGORITHM IN
 *            STEPS section of the MENTAL MODEL block.
 *
 *  Steps:
 *    1. (auto-sweep) advance sweep phase, recompute g_freq_bin.
 *    2. Generate the cosine at g_freq_bin into g_signal[].
 *    3. Promote real signal into g_fft_workspace_buffer.
 *    4. Run fft() in place.
 *    5. Compute |X[k]| for k = 0..N/2 + the running peak.
 *    6. Run the naive DFT into g_dft_reference_buffer for comparison.
 *    7. Compute max bin-by-bin difference (HUD displays this).
 *
 *  No render work happens here — render functions live in §15-§19
 *  and only READ this state.
 */
static void scene_tick(void)
{
    if (g_simulation_paused) return;

    /* ── Step 1.  auto-sweep frequency ─────────────────────────── */
    if (g_auto_sweep_enabled) {
        g_animation_phase_radians +=
            2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
        if (g_animation_phase_radians > 2.0f * (float)M_PI)
            g_animation_phase_radians -= 2.0f * (float)M_PI;

        /* sin in [-1, +1] → unit interval → [FREQ_LO, FREQ_HI]. */
        float sin_normalised_to_unit =
            (sinf(g_animation_phase_radians) + 1.0f) * 0.5f;
        g_freq_bin = FREQ_LO
                   + sin_normalised_to_unit * (FREQ_HI - FREQ_LO);
    }

    /* ── Step 2.  generate test cosine ─────────────────────────── */
    generate_cosine(g_freq_bin, g_signal);

    /* ── Step 3.  promote real signal into complex workspace ──── */
    for (int sample_index = 0; sample_index < N; sample_index++)
        g_fft_workspace_buffer[sample_index] =
            complex_make(g_signal[sample_index], 0.0f);

    /* ── Step 4.  run the FFT in place ─────────────────────────── */
    fft(g_fft_workspace_buffer);

    /* ── Step 5.  one-sided magnitude spectrum + autoscale peak ── */
    g_magnitude_peak = 1e-6f;
    for (int bin_index = 0; bin_index <= N_HALF; bin_index++) {
        g_magnitude[bin_index] =
            complex_magnitude(g_fft_workspace_buffer[bin_index]);
        if (g_magnitude[bin_index] > g_magnitude_peak)
            g_magnitude_peak = g_magnitude[bin_index];
    }

    /* ── Step 6.  reference DFT for verification ───────────────── */
    compute_dft_naive(g_signal, g_dft_reference_buffer);

    /* ── Step 7.  max bin-by-bin error |fft - dft| ─────────────── */
    g_max_verification_error = 0.0f;
    for (int bin_index = 0; bin_index < N; bin_index++) {
        ComplexNumber difference =
            complex_sub(g_fft_workspace_buffer[bin_index],
                        g_dft_reference_buffer[bin_index]);
        float bin_error = complex_magnitude(difference);
        if (bin_error > g_max_verification_error)
            g_max_verification_error = bin_error;
    }
}

/* ===================================================================== */
/* §13  scene_input — manual frequency adjustment                        */
/* ===================================================================== */

static void scene_adjust_freq(float delta_bins)
{
    /* No-op while auto-sweep is on; otherwise clamp to [FREQ_LO, FREQ_HI]. */
    if (g_auto_sweep_enabled) return;
    g_freq_bin += delta_bins;
    if (g_freq_bin < FREQ_LO) g_freq_bin = FREQ_LO;
    if (g_freq_bin > FREQ_HI) g_freq_bin = FREQ_HI;
}

/* ===================================================================== */
/* §14  draw_bar — vertical-bar primitive                                */
/* ===================================================================== */

static void draw_bar(int column, int baseline_row, int bar_height_cells,
                     bool growing_upward, int colour_pair)
{
    /* Stamp a vertical column of glyphs starting at baseline_row,
     * growing either upward or downward.  Used by every panel. */
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
/* §15  draw_signal — TIME panel renderer                                */
/* ===================================================================== */

static void draw_signal_panel(int top_row, int height_rows)
{
    /* Per sample: vertical bar from the panel midline.  Positive
     * samples grow upward as '|', negative grow downward as '.'. */
    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    int columns_to_draw = (COLS < N) ? COLS : N;
    int spacing_per_sample = (COLS / N >= 2) ? 2 : 1;

    for (int sample_index = 0;
         sample_index < columns_to_draw;
         sample_index++) {

        float v   = g_signal[sample_index];
        bool  pos = (v >= 0.0f);
        int   h   = (int)(fabsf(v) * (float)half_height + 0.5f);

        for (int s = 0; s < spacing_per_sample; s++)
            draw_bar(sample_index * spacing_per_sample + s,
                     midline_row, h, pos, PAIR_SIG);
    }
}

/* ===================================================================== */
/* §16  draw_spectrum — FREQ panel renderer                              */
/* ===================================================================== */

static void draw_spectrum_panel(int top_row, int height_rows,
                                int *out_dominant_bin)
{
    /* Magnitude per bin as upward bars from panel bottom.  Spike bin
     * (the loudest one) is highlighted in bright green. */
    int baseline_row = top_row + height_rows - 1;
    int bin_w        = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
    int max_bins     = COLS / bin_w;
    if (max_bins > N_HALF + 1) max_bins = N_HALF + 1;

    int dominant_bin = 0;
    for (int bin_index = 1; bin_index <= N_HALF; bin_index++)
        if (g_magnitude[bin_index] > g_magnitude[dominant_bin])
            dominant_bin = bin_index;
    *out_dominant_bin = dominant_bin;

    for (int bin_index = 0; bin_index < max_bins; bin_index++) {
        float v    = g_magnitude[bin_index]
                   / (g_magnitude_peak + 1e-6f);
        int   h    = (int)(v * (float)height_rows + 0.5f);
        int   pair = (bin_index == dominant_bin) ? PAIR_SPIKE : PAIR_SPEC;

        for (int bx = 0; bx < bin_w - 1; bx++)
            draw_bar(bin_index * bin_w + bx, baseline_row,
                     h, true, pair);
    }
}

/* ===================================================================== */
/* §17  draw_phase — debug 'd' phase panel                               */
/* ===================================================================== */
/*
 *  Show arg(X[k]) for each bin as a vertical bar from a midline.
 *  Positive phase grows up (magenta), negative grows down (sky blue).
 *  Magnitude alone discards phase — toggle this overlay to see what
 *  got thrown away.
 */
static void draw_phase_panel(int top_row, int height_rows)
{
    if (height_rows < 3) return;

    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    int bin_w    = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
    int max_bins = COLS / bin_w;
    if (max_bins > N_HALF + 1) max_bins = N_HALF + 1;

    for (int bin_index = 0; bin_index < max_bins; bin_index++) {
        ComplexNumber z = g_fft_workspace_buffer[bin_index];
        /* Skip phase for near-zero magnitude bins (noise). */
        if (complex_magnitude(z) < 1e-3f) continue;

        float phase_radians = atan2f(z.im, z.re);
        float fraction      = phase_radians / (float)M_PI;   /* in (-1, 1] */
        bool  pos           = (phase_radians >= 0.0f);
        int   h             = (int)(fabsf(fraction) * (float)half_height + 0.5f);

        for (int bx = 0; bx < bin_w - 1; bx++)
            draw_bar(bin_index * bin_w + bx, midline_row,
                     h, pos,
                     pos ? PAIR_PHASE_POS : PAIR_PHASE_NEG);
    }

    /* Midline marker so the user can see "zero phase". */
    attron(COLOR_PAIR(PAIR_LABEL));
    for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
        mvaddch(midline_row, x, '-');
    attroff(COLOR_PAIR(PAIR_LABEL));
}

/* ===================================================================== */
/* §18  draw_arm_table — debug 'D' top-N bin info                        */
/* ===================================================================== */
/*
 *  Top-left numeric panel.  Sorts bins by descending magnitude and
 *  prints (bin, |X[k]|, arg(X[k])) for the first ARM_TABLE_ROWS.
 *  Useful for verifying decode by hand.
 */
static void draw_arm_table(void)
{
    /* Selection-sort the first N/2 bin indices by descending
     * magnitude — only the top ARM_TABLE_ROWS positions matter. */
    int sorted_bin_indices[N_HALF + 1];
    for (int i = 0; i <= N_HALF; i++) sorted_bin_indices[i] = i;
    for (int i = 0; i < ARM_TABLE_ROWS && i <= N_HALF; i++) {
        int max_at = i;
        for (int j = i + 1; j <= N_HALF; j++)
            if (g_magnitude[sorted_bin_indices[j]]
              > g_magnitude[sorted_bin_indices[max_at]])
                max_at = j;
        int tmp                 = sorted_bin_indices[i];
        sorted_bin_indices[i]   = sorted_bin_indices[max_at];
        sorted_bin_indices[max_at] = tmp;
    }

    int x = 2, y = 2;
    if (y + ARM_TABLE_ROWS + 1 >= LINES - 1) return;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, " bin   |X[k]|     arg(rad)");
    attroff(COLOR_PAIR(PAIR_HINT));

    for (int row = 0; row < ARM_TABLE_ROWS && row <= N_HALF; row++) {
        int bin_index   = sorted_bin_indices[row];
        ComplexNumber z = g_fft_workspace_buffer[bin_index];
        float phase     = atan2f(z.im, z.re);
        attron(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
        mvprintw(y + 1 + row, x,
                 "  %2d   %7.3f   %+6.3f",
                 bin_index, (double)g_magnitude[bin_index], (double)phase);
        attroff(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
    }
}

/* ===================================================================== */
/* §19  hud — status top-right + hint bottom + frame composer            */
/* ===================================================================== */

static void draw_hud(int dominant_bin)
{
    int dft_ops = N * N;
    int fft_ops = (N / 2) * LOG2_N;

    char status[200];
    snprintf(status, sizeof status,
             " FFT helloworld  N=%d  freq=%5.2f (peak bin=%d)  "
             "FFT %d vs DFT %d ops  err=%.1e %s  %s  %s ",
             N, (double)g_freq_bin, dominant_bin,
             fft_ops, dft_ops,
             (double)g_max_verification_error,
             (g_max_verification_error < 1e-4f) ? "[FFT == DFT]" : "[FFT != DFT]",
             g_auto_sweep_enabled ? "AUTO  " : "MANUAL",
             g_simulation_paused  ? "PAUSED" : "      ");
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
             " q:quit  spc:pause  a:auto/manual  +/-:freq  ,/.:fine "
             " d:phase  D:bins  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void)
{
    erase();

    /* Reserved rows: 1 hud + 2 panel labels + 1 hint = 4. */
    int rows_for_panels = LINES - 4;
    if (rows_for_panels < 6) rows_for_panels = 6;

    /* Phase panel "steals" a chunk of vertical space when toggled. */
    int phase_h = g_show_phase_panel ? rows_for_panels / 4 : 0;
    int main_h  = rows_for_panels - phase_h;
    int sig_h   = main_h / 2;
    int spec_h  = main_h - sig_h;

    int sig_label_row    = 1;
    int sig_top_row      = 2;
    int spec_label_row   = 2 + sig_h;
    int spec_top_row     = 3 + sig_h;
    int phase_top_row    = 3 + sig_h + spec_h;   /* used only if show_phase */

    /* Panel labels. */
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(sig_label_row,  0, "Input x[n] = cos(2*pi * freq * n / N)");
    mvprintw(spec_label_row, 0, "FFT magnitude |X[k]|  (bins 0..%d)", N_HALF);
    if (g_show_phase_panel)
        mvprintw(phase_top_row - 1, 0,
                 "FFT phase arg(X[k])  (magenta = +, sky = -)");
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
/* §20  app — signal handlers + main loop + key dispatch                 */
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
        case 'a': case 'A':           g_auto_sweep_enabled = !g_auto_sweep_enabled; break;
        case '+': case '=':           scene_adjust_freq(+1.0f); break;
        case '-':                     scene_adjust_freq(-1.0f); break;
        case '.':                     scene_adjust_freq(+0.1f); break;
        case ',':                     scene_adjust_freq(-0.1f); break;
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
            /* Snap forward if we fell badly behind (e.g. terminal hidden). */
            if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
                next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
        } else {
            clock_sleep_ns(next_frame_ns - now);
        }
    }

    return 0;
}
