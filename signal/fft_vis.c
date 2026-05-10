/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fft_vis.c — Cooley-Tukey radix-2 DIT FFT visualiser, three live panels
 *             (time waveform / magnitude spectrum / spectrogram) over
 *             seven signal modes and four window functions.
 *
 * DEMO
 *   The screen splits into three stacked panels.  The TOP panel shows
 *   the input waveform sample by sample.  The MIDDLE panel shows the
 *   magnitude spectrum |X[k]| computed by the FFT — bar heights are
 *   the strength of each frequency.  The BOTTOM panel is a scrolling
 *   waterfall: a strip of "the spectrum we just saw" pushed in at the
 *   bottom each frame, with older spectra floating up.  When the
 *   waveform changes, the spike pattern in the middle changes; when
 *   the changes happen over time, they paint a story in the waterfall.
 *
 *   You can switch between seven signal modes (sum of three sines,
 *   square, sawtooth, triangle, frequency chirp, AM modulation,
 *   periodic impulses) and four window functions (rectangular, Hann,
 *   Hamming, Blackman).  The point of the window selector is that
 *   when a sine wave's frequency doesn't fit an integer number of
 *   cycles into the buffer, the rectangular window produces SPECTRAL
 *   LEAKAGE — the spike smears across many bins.  Other windows tame
 *   the smear at the cost of a slightly broader main lobe.
 *
 * Study alongside
 *   signal/epicycles.c    The same DFT/FFT mathematics applied to a
 *                          two-dimensional closed curve (x + i·y per
 *                          sample).  Coefficients become rotating arms
 *                          rather than frequency-bin spikes.  If you
 *                          have read that file, the algorithm part of
 *                          THIS file (§3..§5) will feel familiar.
 *
 * Section map
 *   §1   config             tunable constants — every magic number lives here
 *   §2   clock              monotonic-ns timer + nanosecond sleep
 *   §3   bit_reverse        the FFT input permutation (and why it exists)
 *   §4   butterfly_stage    one stage of the Cooley-Tukey butterfly
 *   §5   fft                three-line driver: permute then run log₂N stages
 *   §6   windows            Rectangular / Hann / Hamming / Blackman tapers
 *   §7   signal_helpers     periodic_phase — fractional-position helper
 *   §8   signal_modes       per-mode sample generators (square, AM, chirp …)
 *   §9   signal_dispatch    build_signal — wraps the mode generators
 *   §10  themes             palette table for the live colour-cycling
 *   §11  colors             ncurses colour-pair init + apply_theme
 *   §12  draw_bar           low-level vertical-bar primitive
 *   §13  spectrogram        ring buffer + draw_waterfall
 *   §14  layout             panel-size computation per terminal size
 *   §15  draw_time          the TIME panel
 *   §16  draw_freq          the FREQ panel
 *   §17  draw_debug         'd' window-envelope overlay, 'D' phase table
 *   §18  draw_hud           HUD top-right + bottom hint
 *   §19  app_state          App struct + defaults + adjust_freq
 *   §20  app_compute        per-frame orchestration
 *   §21  app_input          key dispatch
 *   §22  app_main           signal handlers + main loop
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   1 / 2 / 3        toggle sine component 1 / 2 / 3   (sine-sum mode only)
 *   j / k            select component                  (sine-sum mode only)
 *   + / -            adjust selected freq by 1.0 bin
 *   , / .            adjust selected freq by 0.1 bin   (fine — for leakage)
 *   m / M            next / previous signal mode
 *   w / W            next / previous window function
 *   g                toggle spectrogram on/off
 *   n                inject one-shot noise burst (~1 s decay)
 *   t / T            next / previous colour theme
 *   d                toggle window-envelope overlay on TIME panel  (debug)
 *   D                toggle bin-info phase table top-left           (debug)
 *   r                reset all parameters to defaults
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/fft_vis.c -o fft_vis \
 *       -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no FFT background:
 *     1. CONCEPTS — the elevator pitch, including references to dig deeper.
 *     2. MENTAL MODEL — the analogies + invariants you need in mind.
 *     3. GUIDED TUTORIAL T1..T12 — twelve mini-lessons that build the
 *        algorithm from first principles, ending with pseudocode that
 *        maps line-for-line to a function in §3..§9.
 *     4. §1 config — the knobs (now you know what they do).
 *     5. §3 bit_reverse → §4 butterfly_stage → §5 fft — the algorithm,
 *        each in its own bite-sized section.
 *     6. §6 windows → §7..§9 signal — the supporting maths and the
 *        seven waveform generators that exercise the FFT.
 *     7. §10..§18 — the colour, layout, and rendering layers.  Algorithm-
 *        wise these are uninteresting; pedagogically they show how raw
 *        numbers become a watchable picture.
 *     8. §19..§22 — the App struct, key dispatch, main loop.
 *
 *   Reader who already knows FFT and just wants the code:
 *     §1, §3, §4, §5, §8, §13, §20.  The rest is teaching scaffolding.
 *
 * Naming convention
 *   Long descriptive identifiers throughout — even inside the FFT inner
 *   loop where the textbook usually writes `wr`, `wi`, `tr`, `ti`.
 *   Spelling these out makes the algebra visible:
 *
 *     twiddle_re, twiddle_im        the running twiddle factor W^j
 *     twiddle_step_re, _im          W^1 — the per-iteration multiplier
 *     t_re, t_im                    the butterfly's `t = W^j · X[j+half]`
 *     stage_width, half             butterfly group sizes
 *     fft_real_buffer, _imag        FFT input/output arrays
 *     signal_pre_window             time-domain signal BEFORE windowing
 *     magnitude_normalised          |X[k]| / (N/2) — one-sided spectrum
 *     animation_phase_radians       slow drift on time wave (no effect on |X|)
 *
 *   Short loop counters (`i`, `j`, `k`, `n`) are fine where context is
 *   one line away.  Anything carried across multiple lines gets a
 *   meaningful name.
 *
 * Background you NEED
 *   The DFT formula at the level of "X[k] is a weighted sum of the
 *   inputs" — see signal/epicycles.c CONCEPTS or any DSP textbook.
 *
 *   Basic complex arithmetic.  We treat each complex number as a pair
 *   of floats `(re, im)` and do multiplication by hand:
 *     (a + ib)·(c + id) = (ac - bd) + i(ad + bc)
 *
 * Background you do NOT need
 *   Continuous Fourier transforms / integrals.  Everything here is
 *   discrete, finite, and indexed by integers.
 *
 *   Recursion.  The textbook FFT is usually presented as recursive,
 *   but the iterative version (which we use) requires only a couple
 *   of nested for-loops and is easier to step through in a debugger.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm
 *   COOLEY-TUKEY radix-2 DIT (Decimation-In-Time) FFT.  Splits the
 *   N-point DFT into two N/2-point DFTs of the even- and odd-indexed
 *   samples, then recombines via the BUTTERFLY operation.  Applied
 *   recursively, the splitting bottoms out after log₂N levels and the
 *   total work is O(N log₂N) instead of the naive O(N²).
 *
 *   Implemented iteratively, not recursively: we (1) permute the input
 *   in place by bit-reversing each index, then (2) walk log₂N "stages",
 *   each running N/2 small butterflies of doubling width.  The twiddle
 *   factor W = exp(-i·2π/stage_width) is multiplied in incrementally,
 *   so we pay one cos+sin per stage rather than one per butterfly.
 *
 * Data structure
 *   Two parallel float arrays — `fft_real_buffer[N]` and
 *   `fft_imag_buffer[N]` — hold the complex sample sequence.  No
 *   `complex` struct, because the butterfly formulas read better when
 *   the real and imaginary parts are spelled out explicitly (and there
 *   is no advantage to packing them).
 *
 *   The spectrogram is an SPEC_HISTORY_LEN × N/2 ring buffer of
 *   quantised intensities (0..4 per cell).  We push one row per frame
 *   when not paused; older rows scroll up the visible waterfall.
 *
 * Rendering
 *   Three vertically stacked panels of vertical bars.  Heights are
 *   normalised per frame against each panel's running peak so the
 *   display auto-scales.  Two debug overlays paint over the panels
 *   when toggled: the window envelope on top of the time wave, and
 *   a numeric phase table for the loudest bins in the corner.
 *
 * Performance
 *   N_FFT is hard-coded to 128 (a power of two so the radix-2
 *   butterfly works).  Per-frame cost: O(N log N) FFT + O(N) signal
 *   generation + O(N · waterfall_height) render.  Holds 30 fps on
 *   any terminal that can keep up with 30 redraws per second.
 *
 *   The op-count comparison printed in the HUD ("FFT 896 ops vs DFT
 *   16384 ops") is exact: 128 × log₂128 = 128 × 7 = 896, and 128² =
 *   16,384.  The factor of 18 grows quickly: at N = 1024 it's 100×.
 *
 * References
 *   Cooley, J. W. & Tukey, J. W. (1965), "An Algorithm for the Machine
 *     Calculation of Complex Fourier Series", Math. Comp. 19, 297-301.
 *     The original five-page paper.  The "discovery" was earlier (Gauss
 *     ~1805) but Cooley-Tukey is the popular re-discovery.
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapters 8 & 12.  Free online.
 *   Harris, F. J. (1978), "On the use of windows for harmonic analysis
 *     with the discrete Fourier transform", Proc. IEEE 66(1).  The
 *     authoritative reference on window functions.
 *   3Blue1Brown, "But what is the Fourier Transform? A visual
 *     introduction", YouTube — the warm-up if Smith's chapters feel
 *     dense.
 *   https://en.wikipedia.org/wiki/Cooley%E2%80%93Tukey_FFT_algorithm
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   The DFT computes a weighted sum for every output bin.  Naively
 *   that costs O(N²) — N output bins × N input contributions per bin.
 *   The FFT is a way to REUSE intermediate sums between bins so that
 *   the total cost drops to O(N log N).  The reuse comes from the
 *   fact that bins which differ by N/2 share most of their work,
 *   bins which differ by N/4 share a bit less, and so on, in a
 *   fractal pattern.  Cooley-Tukey is the algorithm that climbs that
 *   fractal in log₂N levels.
 *
 * HOW TO THINK ABOUT IT
 *   Imagine you want to total up every receipt for a year and you can
 *   either (a) sum them in a pile of 365 (one per day), or (b) sum
 *   them in piles of 31 per month (12 sub-totals) and then sum the
 *   12.  Both produce the same answer; (b) is no faster for a single
 *   sum, but if you wanted MANY sums of OVERLAPPING subsets of days,
 *   the monthly sub-totals would be reusable.  The FFT extends this
 *   idea: it pre-computes "even-index sub-DFTs" and "odd-index sub-
 *   DFTs" once, then combines them with a twist (the twiddle factor)
 *   to produce many of the final bin values at once.
 *
 *   The BUTTERFLY is the smallest unit of that combination.  Two
 *   numbers go in; two numbers come out, computed as sum and
 *   difference (with one of them rotated by the twiddle).  Drawn as
 *   a graph it really does look like a butterfly — a pair of
 *   crossing diagonals on top of two horizontals.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │    E[j] ──●──●──── X[j]      = E[j] + W^j · O[j]        │
 *      │            \\//                                         │
 *      │             X    (W^j multiplier on the O input)        │
 *      │            //\\                                         │
 *      │    O[j] ──●──●──── X[j+N/2]  = E[j] - W^j · O[j]        │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS (per frame)
 *
 *    1. Generate N samples for the chosen signal mode.
 *    2. Save the raw signal so the time panel can show it.
 *    3. Multiply the raw signal by the chosen window function.
 *       Copy into the FFT real-input array; zero the imag-input.
 *    4. Bit-reverse-permute the FFT input arrays in place.
 *    5. Run log₂N butterfly stages.  Stage s has butterflies of
 *       width 2^s; each stage updates the arrays in place.
 *    6. Compute |X[k]| / (N/2) for k = 0..N/2-1 (one-sided magnitude
 *       spectrum; bin N/2 is the Nyquist limit).
 *    7. Push that magnitude row into the spectrogram ring buffer if
 *       not paused.
 *    8. Render: time panel, freq panel, optional waterfall, optional
 *       debug overlays, HUD on top.  doupdate() flushes to terminal.
 *
 * KEY FORMULAS
 *
 *   Forward DFT (definition we are computing efficiently)
 *     X[k] = Σ_{n=0..N-1} x[n] · exp(-i · 2π · k · n / N)
 *
 *   Even/odd split (T4 derives this)
 *     X[k]      = E[k] + W_N^k · O[k]      k = 0 .. N/2-1
 *     X[k+N/2]  = E[k] - W_N^k · O[k]      (because W_N^(k+N/2) = -W_N^k)
 *
 *   Twiddle factor and recurrence (T7 explains why we update incrementally)
 *     W = exp(-i · 2π / stage_width)
 *     twiddle_after  =  twiddle_before  ·  W
 *
 *   Window functions (M = N - 1 in the symmetric formulation)
 *     Rectangular  w[n] = 1
 *     Hann         w[n] = 0.5  - 0.5  · cos(2π · n / M)
 *     Hamming      w[n] = 0.54 - 0.46 · cos(2π · n / M)
 *     Blackman     w[n] = 0.42 - 0.5  · cos(2π · n / M) + 0.08 · cos(4π · n / M)
 *
 *   Magnitude spectrum (one-sided, scale chosen so unit sine → unit peak)
 *     |X[k]| / (N/2)         for k = 0 .. N/2
 *
 *   Operation count (T8)
 *     FFT ≈ N · log₂ N        128 · 7 = 896
 *     DFT ≈ N²                128² = 16,384
 *
 * EDGE CASES TO WATCH
 *   - N_FFT must be a power of two; the bit-reverse and butterfly
 *     stages both assume it.  Changing it to 100 silently corrupts the
 *     output.  We therefore expose `LOG2_N_FFT` next to it and bake
 *     the constant in.
 *
 *   - Magnitude is INVARIANT under time shifts.  The animation phase
 *     drifts the time panel left/right but the freq panel is rock-
 *     steady.  This is a feature: it shows you what `|X[k]|` throws
 *     away (the phase information).  Toggle 'D' to see the discarded
 *     phases as numbers.
 *
 *   - Window functions reduce overall amplitude (their average is < 1).
 *     We do not compensate.  The user should see both the leakage
 *     reduction AND the amplitude reduction so the trade-off is real.
 *
 *   - Rectangular window + integer-bin frequency = perfectly clean
 *     spike.  Move the frequency by 0.5 bin (half-press of `,` or `.`)
 *     and the spike turns into a wide hill.  This is leakage; flip to
 *     Hann to see it tighten.
 *
 *   - The spectrogram is paused-aware: we don't push when the
 *     simulation is paused, so you can study a transient at leisure.
 *     A mode change clears the spectrogram so you don't see the old
 *     mode's history bleeding in.
 *
 *   - Resize calls `endwin(); refresh()` and clears the spectrogram
 *     because column counts may have changed.  Layout recomputes from
 *     `LINES`/`COLS` every frame.
 *
 * HOW TO VERIFY
 *
 *   - Defaults (Sine sum, comp 0/1/2 at bins 4/11/23 with amps
 *     1.0/0.6/0.4): the freq panel shows three peaks at exactly bins
 *     4, 11, 23 with proportional heights.
 *
 *   - Toggle '2' off — bin 11 spike vanishes; the time waveform
 *     simplifies.  Press '2' again — it returns.
 *
 *   - With component 0 selected, press `+`, then `,`, `,`, `,`, `,`
 *     to land on freq = 4.6.  With Rectangular window the bin-4
 *     spike turns into a fat hill across bins 3..6.  Press `w` to
 *     switch to Hann — the hill narrows; press `w` for Hamming, then
 *     Blackman — sidelobes drop further (main lobe widens slightly).
 *
 *   - `m` to Square: spectrum should show odd-bin spikes (4, 12, 20,
 *     28, 36, …) for base freq 4 — the textbook Fourier series of a
 *     square wave (1, 1/3, 1/5, …).
 *
 *   - `m` to Sawtooth: ALL bins (4, 8, 12, 16, …) — Fourier series
 *     of a sawtooth has 1/n decay and no odd/even restriction.
 *
 *   - `m` to Chirp: the freq panel sweeps continuously; the waterfall
 *     paints a clean diagonal stripe from chirp_lo to chirp_hi.
 *
 *   - `m` to AM (carrier 16, mod 3 by default): three spikes appear
 *     at bins 13, 16, 19 — carrier ± modulator, the broadcast-radio
 *     three-spike picture.
 *
 *   - `m` to Impulse with period 16: BOTH panels show evenly-spaced
 *     spikes at multiples of N/period = 8 in the freq panel (a Dirac
 *     comb in time has a Dirac comb in frequency).
 *
 *   - `g` toggles the waterfall.  Time + freq grow into the vacated
 *     rows.
 *
 *   - `d` toggles the WINDOW ENVELOPE OVERLAY: a faint dotted curve
 *     traces the active window function on top of the time panel.
 *     Try Hann ('w' from default) then `d` — you'll see the bell
 *     shape clearly.
 *
 *   - `D` toggles the PHASE TABLE: a numeric grid in the top-left
 *     listing the eight loudest bins with magnitude and arg(X[k]) in
 *     radians.  Pause ('space') and watch the phases for a sine wave
 *     — they sit at the values predicted by the sine's offset.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What we are computing — the DFT formula in plain English
 *   T2   Why it is interesting — pure tones become single bins
 *   T3   Naive cost: O(N²).  The number-theoretic fix.
 *   T4   The even/odd split derivation
 *   T5   Bit reversal — what the recursion leaves behind
 *   T6   The butterfly: turning (E, O) into (X[k], X[k+N/2])
 *   T7   Iterative implementation + the twiddle recurrence
 *   T8   O(N log N) accounting and a quick benchmark
 *   T9   Reading FFT output — magnitude vs phase
 *   T10  Spectral leakage — the price of a finite buffer
 *   T11  Window functions — the leakage cure with a trade-off
 *   T12  Spectrogram + signal-mode fingerprints
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT WE ARE COMPUTING — THE DFT FORMULA IN PLAIN ENGLISH
 * ────────────────────────────────────────────────────────────
 * We have N input samples x[0], x[1], ..., x[N-1] — a string of
 * numbers, each one a snapshot of "the signal at this moment".  We
 * want N output bins X[0], X[1], ..., X[N-1] — also complex numbers
 * — that tell us, in some useful way, what frequencies are present.
 *
 * The discrete Fourier transform defines the relationship:
 *
 *     X[k] = Σ_{n=0..N-1}  x[n] · exp(-i · 2π · k · n / N)
 *
 * For each output bin k we sum the inputs, but each input is FIRST
 * twisted by the complex factor exp(-i · 2π · k · n / N).  As n
 * ranges from 0 to N-1, that factor walks k full turns clockwise
 * around the unit circle.  Bins with k = 0 leave each input
 * untwisted (sum is the average DC level); bins with k = 1 walk the
 * factor once around (catches signals that complete one cycle in N
 * samples); bin k = 2 walks twice around (two cycles); and so on.
 *
 * If a particular frequency is NOT present in the input, the
 * twisted samples have random-looking angles and tend to cancel
 * when summed.  If the frequency IS present, the twists exactly
 * undo the input's rotation, so the twisted samples ALL line up
 * and the sum is large.  Bin k is therefore a "lock-in detector"
 * for the k-th harmonic.
 *
 * Since each input contributes to each output, the naive cost of
 * the DFT is N × N = O(N²).  For N = 128 that's 16,384 complex
 * multiplications per transform.  The FFT (T3..T8) gets the same
 * answer in 896.
 *
 * T2  WHY IT IS INTERESTING — PURE TONES BECOME SINGLE BINS
 * ─────────────────────────────────────────────────────────
 * Take a pure cosine x[n] = cos(2π · k₀ · n / N) for some integer
 * k₀ ∈ [1, N/2-1].  Plug it into the DFT formula and use Euler's
 * identity cos θ = (e^(iθ) + e^(-iθ)) / 2:
 *
 *     X[k] = Σ_n cos(2π·k₀·n/N) · exp(-i·2π·k·n/N)
 *          = ½ Σ_n exp( i·2π·k₀·n/N) · exp(-i·2π·k·n/N)
 *          + ½ Σ_n exp(-i·2π·k₀·n/N) · exp(-i·2π·k·n/N)
 *          = ½ Σ_n exp( i·2π·(k₀-k)·n/N)
 *          + ½ Σ_n exp(-i·2π·(k₀+k)·n/N)
 *
 * The first sum is N when k = k₀ and 0 for all other k in [0, N).
 * The second sum is N when k = N - k₀ (i.e., bin k₀ "from the top")
 * and 0 elsewhere.  So the DFT of a pure cosine has exactly two
 * non-zero bins: at k₀ and at N - k₀, each with magnitude N/2.
 *
 * A SUM of pure cosines produces a SUM of such spike pairs, by
 * linearity of the DFT.  This is what makes the spectrum panel
 * useful: each ingredient frequency appears as its own clean spike.
 *
 * The freq panel in this program shows only bins 0..N/2.  The
 * upper half (N/2+1..N-1) is the conjugate mirror of the lower
 * half for any real input, so it carries no new information and
 * we throw it away.
 *
 * T3  NAIVE COST: O(N²).  THE NUMBER-THEORETIC FIX.
 * ─────────────────────────────────────────────────
 * The naive DFT for one bin is N multiplications + N adds.  For all
 * N bins it's N² multiplications + N² adds.  For N = 128 that's
 * 16,384 complex multiplies — every frame, 30 times a second.  Not
 * crippling on a modern CPU but real.
 *
 * The Cooley-Tukey FFT brings the count down to (N/2) · log₂N
 * complex multiplies.  For N = 128 that's 448, a 36× win.  At
 * N = 1024 it's 5,120 vs over a million — a 200× win.  At
 * N = 65,536 (a typical audio FFT) the FFT is ~4,000× faster than
 * naive DFT.  This is why almost every modern signal-processing
 * algorithm assumes "we can FFT in real time".
 *
 * The TRICK is decomposition.  If N is even, the N-point DFT can be
 * written exactly as TWO N/2-point DFTs of related sub-sequences,
 * combined cheaply with N/2 butterflies.  If N/2 is also even, each
 * sub-DFT can be split again.  And again.  When N is a power of
 * two, the splitting goes log₂N levels deep until we reach 1-point
 * DFTs (which are no-ops: X[0] = x[0]).  The total work telescopes
 * to O(N log N).  T4 derives the split.
 *
 * T4  THE EVEN/ODD SPLIT DERIVATION
 * ─────────────────────────────────
 * Start with the DFT of an N-point sequence:
 *
 *     X[k] = Σ_{n=0..N-1}  x[n] · W^(k·n)        W = exp(-i·2π/N)
 *
 * Group the sum into even-n and odd-n terms.  For even n, write
 * n = 2m where m ranges 0..N/2-1.  For odd n, write n = 2m+1 with
 * m the same range:
 *
 *     X[k] = Σ_m  x[2m]   · W^(k·2m)       (even part)
 *          + Σ_m  x[2m+1] · W^(k·(2m+1))   (odd  part)
 *
 *     X[k] = Σ_m  x[2m]   · (W²)^(k·m)
 *          + W^k · Σ_m  x[2m+1] · (W²)^(k·m)
 *
 * Now the magic: W² = exp(-i·2π/N · 2) = exp(-i·2π/(N/2)) = the
 * twiddle for an N/2-point DFT.  Let W' = W² and:
 *
 *     E[k] = Σ_m x[2m]   · (W')^(k·m)    ← N/2-point DFT of evens
 *     O[k] = Σ_m x[2m+1] · (W')^(k·m)    ← N/2-point DFT of odds
 *
 * Then:
 *
 *     X[k] = E[k] + W^k · O[k]            for k = 0..N/2-1     (★)
 *
 * What about k in N/2..N-1?  The N/2-point DFTs E and O are only
 * defined for index 0..N/2-1, but they're naturally PERIODIC with
 * period N/2.  And W^(k+N/2) = W^k · W^(N/2) = W^k · (-1).  So:
 *
 *     X[k+N/2] = E[k] + W^(k+N/2) · O[k]
 *              = E[k] - W^k · O[k]        for k = 0..N/2-1     (★★)
 *
 * (★) and (★★) together are the BUTTERFLY (T6).  They turn one
 * pair (E[k], O[k]) into one pair (X[k], X[k+N/2]).
 *
 * Cost accounting: computing E and O is two (N/2)² = N²/2 work.
 * Combining them via (★) and (★★) is N/2 multiplies and N adds.
 * Total: N²/2 + O(N) — already half the naive O(N²).  And we can
 * apply the same trick recursively to E and to O, halving again
 * each time.
 *
 * T5  BIT REVERSAL — WHAT THE RECURSION LEAVES BEHIND
 * ───────────────────────────────────────────────────
 * Imagine running the recursion on N = 8.  At each level we split
 * even-indices from odd-indices:
 *
 *   level 0:  x[0] x[1] x[2] x[3] x[4] x[5] x[6] x[7]
 *   level 1: (x[0] x[2] x[4] x[6]) (x[1] x[3] x[5] x[7])
 *               evens                odds
 *   level 2: ((x[0] x[4]) (x[2] x[6])) ((x[1] x[5]) (x[3] x[7]))
 *               e-e        e-o          o-e        o-o
 *   level 3: x[0] x[4] x[2] x[6] x[1] x[5] x[3] x[7]
 *
 * The leaves come out in BIT-REVERSED order.  Look at the index
 * binary representations:
 *
 *     leaf:     0     1     2     3     4     5     6     7
 *     leaf_bin: 000   001   010   011   100   101   110   111
 *     above   : x[0] x[4] x[2] x[6] x[1] x[5] x[3] x[7]
 *     orig_idx: 0     4     2     6     1     5     3     7
 *     orig_bin: 000   100   010   110   001   101   011   111
 *
 * Each leaf-position's index, in binary, is the bit-reversal of
 * the original sample's index.  Make sense: at level 1 we picked
 * elements by bit 0 (even = bit 0 zero, odd = bit 0 one), so bit 0
 * of the original ends up as the highest bit of the leaf position.
 * At level 2 we split by bit 1, so bit 1 of the original becomes
 * the second-highest position bit.  And so on.
 *
 * The ITERATIVE FFT does the recursion's work bottom-up, treating
 * adjacent pairs in the bit-reversed order as the "1-point DFTs"
 * to combine.  The BIT-REVERSE PERMUTATION at the start of fft()
 * is what aligns the data so that "adjacent pairs" really are the
 * (E, O) pairs the butterfly expects.  See §3 for the code.
 *
 * T6  THE BUTTERFLY: TURNING (E, O) INTO (X[k], X[k+N/2])
 * ───────────────────────────────────────────────────────
 * One butterfly is the operation:
 *
 *     X[k]      = E[k] + W^k · O[k]
 *     X[k+N/2]  = E[k] - W^k · O[k]
 *
 * In code we don't keep separate E and O arrays — they LIVE in the
 * place where the X[k] outputs will end up, and we overwrite them
 * in place.  Reading the formulas as "in place updates":
 *
 *     t            = W^k · X[k+N/2]      ← was O[k]
 *     X[k+N/2]     = X[k] - t             ← was E[k] in the slot
 *     X[k]         = X[k] + t
 *
 * Three lines.  One complex multiplication, one complex addition,
 * one complex subtraction.  The variable `t` holds the twisted
 * O[k] term; both outputs use it.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │    in[k]      ──●─────────●──── out[k]      = in[k] + t │
 *      │                  \\     //                              │
 *      │                    ✕   (× W^k inside)                   │
 *      │                  //     \\                              │
 *      │    in[k+half] ──●─────────●──── out[k+half] = in[k] - t │
 *      │                                                         │
 *      │    where t = W^k · in[k+half]                           │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * Stage s of the iterative algorithm runs N/(2·2^s) such butterflies
 * — N/2 in the first stage, N/4 in the second (with the per-
 * butterfly width doubling), etc.  Always N/2 per stage TOTAL — the
 * groupings just get larger.
 *
 * T7  ITERATIVE IMPLEMENTATION + THE TWIDDLE RECURRENCE
 * ─────────────────────────────────────────────────────
 * The iterative outer loop is two lines of pseudocode:
 *
 *     bit_reverse_permute(input)
 *     for stage_width in [2, 4, 8, ..., N]:
 *         run_butterfly_stage(input, stage_width)
 *
 * Each stage processes the N input slots in groups of `stage_width`.
 * Inside one group, half elements at the start are the "E" slots and
 * half at the end are the "O" slots.  We pair them up and butterfly.
 *
 *     run_butterfly_stage(buf, W):
 *       half = W / 2
 *       step = exp(-i · 2π / W)
 *       for group_start in range(0, N, W):
 *         twiddle = 1                             # = step^0
 *         for j in range(half):
 *           t = twiddle · buf[group_start + j + half]
 *           buf[group_start + j + half] = buf[group_start + j] - t
 *           buf[group_start + j]       += t
 *           twiddle *= step                       # advance the W^j
 *
 * The inner-loop variable `twiddle` is the running W^j.  Naively we
 * could compute it as cos(angle) + i·sin(angle) for each j — but
 * cos/sin are expensive.  The TWIDDLE RECURRENCE replaces them with
 * one complex multiply per step, since W^(j+1) = W^j · W.  We pay
 * one cos+sin per stage to set up `step`, then never touch trig
 * again inside the group.  For N = 128, that's 7 stages × 2 trig
 * = 14 trig calls total per FFT; the naive version would be
 * 2 · N² = 32,768.
 *
 * The numerical cost of the recurrence is that floating-point error
 * accumulates in `twiddle` over many multiplications.  At N = 128
 * the drift is well below single-precision noise; at N > 100,000
 * a smart implementation refreshes `twiddle` every few hundred
 * steps from a fresh trig call.  We do not bother here.
 *
 * T8  O(N log N) ACCOUNTING AND A QUICK BENCHMARK
 * ───────────────────────────────────────────────
 * Each stage processes exactly N/2 butterflies.  There are log₂N
 * stages.  Total butterflies = (N/2) · log₂N.  Each butterfly
 * costs one complex multiply, one complex add, one complex
 * subtract, and roughly one complex multiply for the twiddle
 * update.  Lump all of those into "complex multiplies":
 *
 *     FFT total ≈ N · log₂N
 *
 * For N = 128: 128 · 7 = 896.
 * For N = 1024: 1024 · 10 = 10,240.
 * For N = 65536: 65536 · 16 ≈ 1 million.
 *
 * The naive DFT for the same N values would cost N² = 16,384,
 * 1,048,576, and 4,294,967,296 respectively.  The ratio FFT/DFT is
 * log₂N / N — the FFT becomes proportionally faster for larger N.
 *
 * The HUD on screen shows this comparison live as
 * "FFT 896 ops vs DFT 16384 ops" — pop calc.exe and verify.
 *
 * T9  READING FFT OUTPUT — MAGNITUDE VS PHASE
 * ───────────────────────────────────────────
 * Each output bin X[k] is a complex number — two floats per bin.
 * The standard way to look at it:
 *
 *     magnitude_k = |X[k]|       = √(re² + im²)     "how loud"
 *     phase_k     = arg(X[k])    = atan2(im, re)    "where in the cycle"
 *
 * The FREQ panel of this program plots magnitude only.  This is
 * what almost every audio analyser shows you, because the human ear
 * is far more sensitive to amplitude than to phase.  Magnitude has
 * a beautiful invariance: SHIFTING A SIGNAL IN TIME LEAVES THE
 * MAGNITUDE SPECTRUM UNCHANGED.  Only the phases change.  You can
 * see this live: the time wave drifts left/right (because of the
 * `animation_phase_radians` advance), but the freq panel sits
 * still.  Toggle 'D' to see the phases changing as the wave drifts.
 *
 * Phase is NOT junk, however.  If you want to RECONSTRUCT the
 * original signal from its spectrum (inverse FFT), you need both
 * magnitude AND phase.  Throwing away phase is a one-way operation
 * — convenient for visualisation, fatal for compression of audio
 * waveforms.  (JPEG and MP3 keep both, with very clever compression
 * of the phase data.)
 *
 * One-sided convention: for real-valued input, X[N-k] is the complex
 * conjugate of X[k].  So |X[N-k]| = |X[k]| — the upper half of the
 * spectrum is a mirror image of the lower half and carries no new
 * information.  We display only bins 0..N/2-1.  We also divide by
 * N/2 (not N) so that a unit cosine input produces a peak of
 * magnitude ≈ 1.0 (the energy from both bins +k and -k folds into
 * the visible bin).
 *
 * T10  SPECTRAL LEAKAGE — THE PRICE OF A FINITE BUFFER
 * ────────────────────────────────────────────────────
 * The DFT is built on the fiction that our N samples are ONE PERIOD
 * of an infinitely-repeating signal.  If the actual signal frequency
 * fits an integer number of cycles in the buffer (say, 4 cycles for
 * a buffer of 128 samples), the fiction holds: the sample at n = 0
 * smoothly continues into the sample at n = N, and the DFT returns
 * a clean spike.  But if the frequency is, say, 4.5 cycles per
 * buffer, the fiction's seam at n = N creates a step discontinuity
 * — the implicit periodic extension has a sudden jump every 128
 * samples.
 *
 * A step discontinuity is a fast-changing thing in the time domain.
 * In the frequency domain that translates to ENERGY SPREAD ACROSS
 * MANY BINS.  The expected spike at bin 4.5 is unrepresentable
 * (bins are integer-only), so the energy bleeds into bins 3, 4, 5,
 * 6, 7, ... in a sin-x/x pattern.  This is SPECTRAL LEAKAGE.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │  freq = 4.0  (integer)        freq = 4.5  (fractional)  │
 *      │                                                         │
 *      │   |                              |                      │
 *      │   |                              | |                    │
 *      │   |                            | | | |                  │
 *      │  _|________________            | | | | |  ___           │
 *      │  3 4 5 6 7  bins               2 3 4 5 6 7 8  bins      │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * The leakage shape (sinc, the Fourier transform of a rectangular
 * window) has a tall main lobe and slowly-decaying sidelobes.  The
 * sidelobes can be 13 dB below the main lobe — loud enough to mask
 * a real signal at a different frequency.  This is the primary
 * reason real-world spectrum analysers always apply a window (T11).
 *
 * Try it live: select component 0, press '+' four times to land at
 * freq = 8, see the clean spike.  Then press ',' five times to
 * land at 7.5, see the smear.
 *
 * T11  WINDOW FUNCTIONS — THE LEAKAGE CURE WITH A TRADE-OFF
 * ──────────────────────────────────────────────────────────
 * Multiply the input signal by a smooth taper that goes to (or near)
 * zero at the buffer boundaries before running the FFT.  The
 * tapered signal does not have a step at n = N, so the implicit
 * periodic extension is smooth, so leakage drops dramatically.
 *
 * Trade-off: the taper REDUCES the signal in the middle of the
 * buffer too, so the main lobe of every spike is now wider (worse
 * frequency resolution) and quieter (lower peak).  Different window
 * shapes pick different points on this trade-off curve:
 *
 *     window         main-lobe width    first sidelobe (dB below)
 *     Rectangular    narrow (1 bin)     -13 dB
 *     Hann           wider (1.5 bins)   -32 dB
 *     Hamming        like Hann          -42 dB
 *     Blackman       wider (2 bins)     -58 dB
 *
 * In this program we offer all four windows (key 'w') and let you
 * see the difference live.  At freq = 4.5:
 *   - Rectangular: tall hill spreading 4 bins wide, ugly sidelobes
 *     out to bin 12+
 *   - Hann: shorter hill spreading ~3 bins wide, sidelobes barely
 *     visible past bin 8
 *   - Blackman: wider hill again but sidelobes invisible to the eye
 *
 * Pick a window based on what matters more:
 *   - For DETECTING a known weak signal near a strong one (radar,
 *     bird-song): Blackman.  Sidelobes < signal floor.
 *   - For RESOLVING two closely-spaced frequencies: Hann or
 *     Rectangular.  Narrow main lobe.
 *   - For RECONSTRUCTING the signal exactly: Rectangular (no taper).
 *
 * T12  SPECTROGRAM + SIGNAL-MODE FINGERPRINTS
 * ───────────────────────────────────────────
 * The SPECTROGRAM (waterfall) is the same FFT applied to a sliding
 * window of the signal over time — except in this demo there's no
 * "sliding window"; we just stack the per-frame spectrum into a
 * scrolling history buffer.  The vertical axis is time (newest at
 * bottom, oldest at top); the horizontal axis is frequency (same
 * bin layout as the freq panel above).  Glyph density and colour
 * encode magnitude.
 *
 * Each signal mode paints its own characteristic fingerprint in
 * both the freq panel and the waterfall:
 *
 *   SINE SUM:  one to three vertical stripes, each at its component's
 *              bin, perfectly stationary.
 *
 *   SQUARE:    odd-indexed harmonics (1, 3, 5, 7, ...) of the base
 *              frequency, with magnitudes decaying as 1/n.  This is
 *              the textbook Fourier series of a square wave.
 *
 *   SAWTOOTH:  ALL harmonics, magnitudes 1/n.  Looks like a comb in
 *              both panels, denser than the square wave's.
 *
 *   TRIANGLE:  odd harmonics, magnitudes 1/n² — much faster decay.
 *              You'll see a strong fundamental and barely-visible
 *              third / fifth harmonics.
 *
 *   CHIRP:     a clean diagonal stripe in the waterfall as the
 *              instantaneous frequency sweeps from chirp_lo to
 *              chirp_hi.  In radar this is "linear FM"; in audio
 *              it's a glissando.
 *
 *   AM:        three spikes — at the carrier and at carrier ± mod.
 *              The waterfall is three stationary stripes.  This is
 *              the spectrum of a multiplication: x_AM(t) = (1 + ½ cos
 *              ω_m·t) · cos ω_c·t = cos ω_c·t + ¼ cos(ω_c+ω_m)t +
 *              ¼ cos(ω_c-ω_m)t.
 *
 *   IMPULSE:   a Dirac comb in time has a Dirac comb in frequency.
 *              Period P samples → spikes at every N/P bins.  At the
 *              default period 16 you'll see spikes at bins 0, 8, 16,
 *              24, 32, 40, 48, 56 (= multiples of 128/16).
 *
 * The waterfall is most striking on the chirp and on the noise
 * burst (press 'n').  Try them.
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
/* §1  config — tunable constants                                        */
/* ===================================================================== */
/*
 * Everything sized at compile time.  No allocator is touched after
 * startup.  The naming convention SCREAMING_SNAKE_CASE indicates a
 * #define so a reader can immediately tell "this is a constant" from
 * "this is a variable".
 */

/* §1.1 — FFT sizing -------------------------------------------------- */
/*
 * N_FFT must be a power of two; the bit-reverse + butterfly stages
 * both depend on it.  LOG2_N_FFT is hard-coded to keep §3 free of a
 * dynamic log calculation (which would also be a runtime trap if the
 * two constants got out of sync).
 */
#define N_FFT             128
#define LOG2_N_FFT          7    /* = log2(128); MUST equal log2(N_FFT)  */

/* §1.2 — sine-sum mode component count ------------------------------ */
#define N_SINE_COMPONENTS   3

/* §1.3 — render frame rate ------------------------------------------ */
#define RENDER_FPS         30
#define RENDER_TICK_NS    (1000000000LL / RENDER_FPS)

/* §1.4 — noise-burst envelope ('n' key) ----------------------------- */
#define NOISE_AMP_PEAK    0.30f      /* burst amplitude              */
#define NOISE_DECAY_RATE  0.03f      /* per-frame decay; ~1 s to fade */

/* §1.5 — animation drift step (advances per frame when not paused) - */
#define ANIMATION_PHASE_STEP_RAD   0.06f

/* §1.6 — spectrogram history depth (rows) --------------------------- */
#define SPEC_HISTORY_LEN  120

/* §1.7 — selectable-table sizes ------------------------------------- */
#define N_THEMES            5
#define N_WINDOWS           4
#define N_SIGNAL_MODES      7

/* §1.8 — colour-pair IDs (bound by §11 init_colors) ----------------- *
 * Pair 0 is reserved by ncurses for the default background.  Our
 * pairs start at 1.  HUD/HINT pairs are theme-INVARIANT per CLAUDE.md
 * HUD Standard.
 */
enum {
    PAIR_BG       = 1,
    PAIR_TIME_POS,                /* time-domain positive lobe       */
    PAIR_TIME_NEG,                /* time-domain negative lobe       */
    PAIR_FREQ_LOW,                /* freq bar — small magnitude      */
    PAIR_FREQ_HIGH,               /* freq bar — large magnitude      */
    PAIR_HUD,                     /* yellow + bold; theme-invariant  */
    PAIR_HINT,                    /* cyan   + bold; theme-invariant  */
};

/* ===================================================================== */
/* §2  clock — monotonic ns timer + nanosecond sleep                     */
/* ===================================================================== */

static long long clock_now_ns(void)
{
    /* Purpose         : return monotonic clock in nanoseconds.
     * Pseudocode      : ts = clock_gettime; return ts.sec*1e9 + ts.nsec.
     * Mental model    : a wall clock that only moves forward, never
     *                   resets, never jumps backward (CLOCK_MONOTONIC
     *                   is immune to NTP and DST).
     * Inputs / outputs: none / int64_t-as-long-long ns.
     * Why it exists   : the main loop subtracts now() values to figure
     *                   out how long to sleep before the next frame.
     */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    /* Purpose : block the calling thread for `ns` nanoseconds.
     * Mental model : "sit still until next frame slot".
     * Why it exists : without this the main loop would spin a CPU at
     *                 100% just to repeat the FFT 30,000 times a sec.
     */
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  bit_reverse — the FFT input permutation                           */
/* ===================================================================== */
/*
 * Why does this function exist?  Because the recursive FFT picks
 * even-then-odd, then even-then-odd within each, and so on, for
 * log₂N levels.  The leaves of that recursion (the 1-point DFTs) sit
 * in BIT-REVERSED index order.  T5 walks through this with an N=8
 * worked example.
 *
 * Iterative implementations don't recurse; they walk the recursion
 * tree bottom-up.  But the iterative algorithm reads adjacent pairs
 * and assumes "those are the (E, O) pairs the butterfly expects".
 * For that assumption to hold, the input array must already be in
 * the bit-reversed order that the recursion would have produced.
 * This function permutes the input in place to that order.
 *
 *      ┌──────── N = 8 worked example ──────────────────────────┐
 *      │                                                        │
 *      │   index   binary    bit-reversed binary    new index   │
 *      │     0      000              000                 0      │
 *      │     1      001              100                 4      │
 *      │     2      010              010                 2      │
 *      │     3      011              110                 6      │
 *      │     4      100              001                 1      │
 *      │     5      101              101                 5      │
 *      │     6      110              011                 3      │
 *      │     7      111              111                 7      │
 *      │                                                        │
 *      │   So we swap (1, 4), (3, 6), and leave (0,2,5,7) put.  │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 *
 * Cost: O(N · log₂N) bit-shifts.  For N = 128 that's 896 shifts —
 * cheap, called once per FFT.
 */

static int reverse_low_bits(int value, int bit_count)
{
    /* Purpose         : flip the low `bit_count` bits of `value`.
     * Pseudocode      : reversed = 0
     *                   for b in 0..bit_count-1:
     *                     bit = (value >> b) & 1
     *                     reversed |= bit << (bit_count - 1 - b)
     *                   return reversed
     * Mental model    : write the binary digits down, then read them
     *                   right-to-left.
     * Inputs / outputs: value, bit_count / int.
     * Why it exists   : called once per index by bit_reverse_permute.
     */
    int reversed = 0;
    for (int b = 0; b < bit_count; b++)
        reversed |= ((value >> b) & 1) << (bit_count - 1 - b);
    return reversed;
}

static void bit_reverse_permute(float *real_buffer,
                                float *imag_buffer,
                                int    sample_count)
{
    /* Purpose         : swap each (real, imag) pair at index i with
     *                   the pair at bit-reversed index, IF the swap
     *                   moves an element to a higher slot — otherwise
     *                   we'd undo a previous swap.
     * Pseudocode      : for i in 0..N-1:
     *                     j = reverse_low_bits(i, log2 N)
     *                     if j > i: swap(buf[i], buf[j])
     * Mental model    : a one-pass unsorting.  The "if j > i" rule
     *                   ensures each pair is touched exactly once.
     * Inputs / outputs: real/imag arrays, length / mutates them.
     * Why it exists   : preconditions the input for the butterfly.
     */
    for (int i = 0; i < sample_count; i++) {
        int j = reverse_low_bits(i, LOG2_N_FFT);
        if (j > i) {
            float tmp;
            tmp = real_buffer[i]; real_buffer[i] = real_buffer[j]; real_buffer[j] = tmp;
            tmp = imag_buffer[i]; imag_buffer[i] = imag_buffer[j]; imag_buffer[j] = tmp;
        }
    }
}

/* ===================================================================== */
/* §4  butterfly_stage — one stage of the Cooley-Tukey FFT               */
/* ===================================================================== */
/*
 * Stage s (s = 1 .. log₂N) does N/2 butterflies of width 2^s.  The
 * input array is divided into N/(2^s) groups of 2^s consecutive
 * elements; within each group the first half holds the running
 * "E" values and the second half holds the running "O" values
 * (after the bit-reverse permutation made this true at the start).
 *
 * Each butterfly inside a group consumes one (E[j], O[j]) pair and
 * writes one (X[j], X[j+half]) pair, in place:
 *
 *     t           = twiddle · in[group_start + j + half]      // = W^j · O[j]
 *     in[j+half]  = in[j] - t
 *     in[j]       = in[j] + t                                  // X[j] = E[j] + t
 *
 * The twiddle starts at W^0 = 1 for j = 0 and is multiplied by the
 * stage's W = exp(-i · 2π / stage_width) at every iteration.  This
 * is the TWIDDLE RECURRENCE — see T7 for why we maintain it
 * incrementally instead of calling cos/sin per butterfly.
 *
 * The BUTTERFLY DIAGRAM:
 *
 *      in[j]      ──●─────────●──── out[j]      = in[j] + W^j · in[j+half]
 *                    \\     //
 *                      ✕   (multiply by W^j inside)
 *                    //     \\
 *      in[j+half] ──●─────────●──── out[j+half] = in[j] - W^j · in[j+half]
 */

static void run_butterfly_stage(float *real_buffer,
                                float *imag_buffer,
                                int    sample_count,
                                int    stage_width)
{
    /* Purpose         : run one Cooley-Tukey radix-2 DIT stage in place.
     * Pseudocode      :
     *     half       = stage_width / 2
     *     step_angle = -2π / stage_width
     *     step       = exp(i · step_angle)
     *     for i in [0, N) step stage_width:
     *       twiddle = 1
     *       for j in [0, half):
     *         t        = twiddle · buf[i+j+half]
     *         buf[i+j+half] = buf[i+j] - t
     *         buf[i+j]      = buf[i+j] + t
     *         twiddle  = twiddle · step
     *
     * Mental model    : N/(2·stage_width) groups, each running half
     *                   butterflies with twiddles W^0, W^1, ... W^(half-1).
     * Inputs / outputs: real/imag arrays, length, stage_width / mutates.
     * Why it exists   : split out from `fft()` so this section can be
     *                   read on its own and the comments can teach the
     *                   butterfly without a wall of code around them.
     */
    int   half = stage_width >> 1;
    float twiddle_step_angle = -2.0f * (float)M_PI / (float)stage_width;
    float twiddle_step_re    = cosf(twiddle_step_angle);
    float twiddle_step_im    = sinf(twiddle_step_angle);

    for (int i = 0; i < sample_count; i += stage_width) {
        float twiddle_re = 1.0f;        /* W^0 = 1 + 0i */
        float twiddle_im = 0.0f;

        for (int j = 0; j < half; j++) {
            /* t = W^j · in[i + j + half]  (complex multiplication
             *   spelled out; (a+bi)(c+di) = (ac - bd) + (ad + bc)i) */
            float t_re = twiddle_re * real_buffer[i + j + half]
                       - twiddle_im * imag_buffer[i + j + half];
            float t_im = twiddle_re * imag_buffer[i + j + half]
                       + twiddle_im * real_buffer[i + j + half];

            /* in[i + j + half] = in[i + j] - t */
            real_buffer[i + j + half] = real_buffer[i + j] - t_re;
            imag_buffer[i + j + half] = imag_buffer[i + j] - t_im;

            /* in[i + j] += t */
            real_buffer[i + j] += t_re;
            imag_buffer[i + j] += t_im;

            /* Advance twiddle: W^(j+1) = W^j · W^1.  Save next-real in
             * a temp because we'd otherwise clobber `twiddle_re` before
             * `twiddle_im` is updated.  Same shape as complex_mul. */
            float next_twiddle_re = twiddle_re * twiddle_step_re
                                  - twiddle_im * twiddle_step_im;
            twiddle_im            = twiddle_re * twiddle_step_im
                                  + twiddle_im * twiddle_step_re;
            twiddle_re            = next_twiddle_re;
        }
    }
}

/* ===================================================================== */
/* §5  fft — driver: bit-reverse then run log₂N butterfly stages         */
/* ===================================================================== */

static void fft(float *real_buffer, float *imag_buffer, int sample_count)
{
    /* Purpose         : in-place forward FFT of a complex sequence.
     * Pseudocode      : bit_reverse(buf)
     *                   for stage_width in [2, 4, 8, ..., N]:
     *                     run_butterfly_stage(buf, stage_width)
     * Mental model    : permute the input into "iterative-friendly"
     *                   order, then climb the log₂N stages combining
     *                   pairs into 4-tuples into 8-tuples ... into N.
     * Inputs / outputs: real/imag input arrays, length / arrays now
     *                   hold the FFT output.
     * Why it exists   : tiny three-line driver makes the algorithm
     *                   structure visible at a glance.
     */
    bit_reverse_permute(real_buffer, imag_buffer, sample_count);
    for (int stage_width = 2; stage_width <= sample_count; stage_width <<= 1)
        run_butterfly_stage(real_buffer, imag_buffer, sample_count, stage_width);
}

/* ===================================================================== */
/* §6  windows — Rectangular / Hann / Hamming / Blackman                 */
/* ===================================================================== */
/*
 * A window function is a smooth taper applied to the signal BEFORE
 * the FFT.  It kills spectral leakage by ensuring the implicit
 * periodic extension of the buffer is continuous at the boundary.
 * The price is wider main lobes (worse frequency resolution).  T10
 * sets up the problem; T11 explains the fix.
 *
 * Each window is defined by a one-line cosine-sum formula.  All
 * three non-rectangular variants below use the parameter
 * M = N - 1 (the "symmetric" form), so the corner samples
 * (n = 0 and n = N - 1) hit zero exactly for Hann/Blackman.
 */

typedef enum {
    WINDOW_RECT     = 0,
    WINDOW_HANN,
    WINDOW_HAMMING,
    WINDOW_BLACKMAN,
} WindowKind;

static const char *window_name[N_WINDOWS] = {
    "Rectangular", "Hann       ", "Hamming    ", "Blackman   ",
};

/* Compute one window coefficient at index `n` for buffer length `N`.
 * Used both by apply_window (in §6) and by the debug overlay in §17. */
static float window_coefficient(WindowKind kind, int n, int N)
{
    if (kind == WINDOW_RECT) return 1.0f;
    float two_pi_over_M = 2.0f * (float)M_PI / (float)(N - 1);
    switch (kind) {
        case WINDOW_HANN:
            return 0.5f - 0.5f * cosf(two_pi_over_M * (float)n);
        case WINDOW_HAMMING:
            return 0.54f - 0.46f * cosf(two_pi_over_M * (float)n);
        case WINDOW_BLACKMAN:
            return 0.42f
                 - 0.5f  * cosf(two_pi_over_M * (float)n)
                 + 0.08f * cosf(2.0f * two_pi_over_M * (float)n);
        default:
            return 1.0f;
    }
}

static void apply_window(float *signal, int N, WindowKind kind)
{
    /* Purpose         : multiply signal[0..N-1] by the chosen window
     *                   coefficient in place.
     * Pseudocode      : for n in 0..N-1: signal[n] *= w_kind(n).
     * Mental model    : a smooth bell-curve mask laid over the signal.
     * Inputs / outputs: signal array, length, kind / mutates signal.
     * Why it exists   : called once per frame in app_compute on the
     *                   FFT input array, AFTER copying from the raw
     *                   signal so the time panel can still show the
     *                   un-windowed waveform.
     */
    if (kind == WINDOW_RECT) return;
    for (int n = 0; n < N; n++)
        signal[n] *= window_coefficient(kind, n, N);
}

/* ===================================================================== */
/* §7  signal_helpers — periodic_phase                                   */
/* ===================================================================== */

static inline float periodic_phase(int sample_index,
                                   float frequency_bin,
                                   float animation_phase_radians)
{
    /* Purpose         : given a sample index n and a frequency in
     *                   cycles-per-buffer, return the fractional
     *                   position within one cycle, ∈ [0, 1).
     * Mental model    : "if the wave repeats every k samples, where
     *                   are we within the current cycle?"  Used by
     *                   square / sawtooth / triangle generators.
     * Inputs / outputs: n, freq, anim_phase / float in [0, 1).
     * Why it exists   : factors out the same arithmetic from three
     *                   periodic-waveform samplers.
     */
    float t = frequency_bin * (float)sample_index / (float)N_FFT
            + animation_phase_radians * frequency_bin
              / (2.0f * (float)M_PI);
    return t - floorf(t);
}

/* ===================================================================== */
/* §8  signal_modes — per-mode sample generators                         */
/* ===================================================================== */
/*
 * One function per signal mode.  Each takes a sample index and
 * whatever parameters that mode needs, and returns one float in
 * roughly [-1, 1].  See T12 for the spectral fingerprint each mode
 * paints in the freq panel.
 */

static float square_sample(int n, float freq, float anim_phase)
{
    /* +1 for the first half of each cycle, -1 for the second.
     * Spectrum: odd harmonics, magnitudes ∝ 1/n. */
    return periodic_phase(n, freq, anim_phase) < 0.5f ? 1.0f : -1.0f;
}

static float sawtooth_sample(int n, float freq, float anim_phase)
{
    /* Linear ramp from -1 to +1, repeated.  Spectrum: all harmonics,
     * magnitudes ∝ 1/n. */
    return 2.0f * periodic_phase(n, freq, anim_phase) - 1.0f;
}

static float triangle_sample(int n, float freq, float anim_phase)
{
    /* /\/\/\/\ shape.  Spectrum: odd harmonics, magnitudes ∝ 1/n²
     * — much faster decay than square. */
    float p = periodic_phase(n, freq, anim_phase);
    return 4.0f * fabsf(p - 0.5f) - 1.0f;
}

static float chirp_sample(int n, float lo, float hi)
{
    /* Linear chirp: instantaneous frequency varies linearly from lo
     * (at n=0) to hi (at n=N-1).  Phase is the integral of frequency
     * over time, which carries an n² term:
     *   φ(n) = 2π · (lo · t + (hi - lo) · t² / 2)   where t = n / N
     * Spectrum: a smear from lo to hi.  Spectrogram: a clean
     * diagonal line. */
    float t = (float)n / (float)N_FFT;
    float phase_cycles = lo * t + (hi - lo) * t * t * 0.5f;
    return sinf(2.0f * (float)M_PI * phase_cycles);
}

static float am_sample(int n, float carrier_freq, float mod_freq, float anim_phase)
{
    /* Amplitude-modulated sine:
     *   x(t) = (1 + ½ cos ω_m t) · cos ω_c t
     * Algebraic identity: x = cos ω_c t + ¼ cos(ω_c+ω_m)t + ¼ cos(ω_c-ω_m)t.
     * Spectrum: three spikes — carrier and ± modulator sidebands. */
    float t          = (float)n / (float)N_FFT;
    float modulator  = cosf(2.0f * (float)M_PI * mod_freq * t
                          + anim_phase * mod_freq);
    float carrier_s  = cosf(2.0f * (float)M_PI * carrier_freq * t
                          + anim_phase * carrier_freq);
    return (1.0f + 0.5f * modulator) * carrier_s * 0.55f;
}

static float impulse_sample(int n, int period)
{
    /* Periodic delta function: non-zero every `period` samples.
     * The Fourier transform of a Dirac comb is itself a Dirac comb
     * — spikes every N/period bins in the freq panel. */
    if (period <= 0) period = 1;
    return (n % period == 0) ? 0.85f : 0.0f;
}

/* ===================================================================== */
/* §9  signal_dispatch — build_signal                                    */
/* ===================================================================== */

typedef struct {
    float frequency_bin;     /* in [1, N/2-1]  */
    float amplitude;         /* in [0, 1]       */
    bool  enabled;
} SineComponent;

typedef enum {
    MODE_SINE_SUM = 0,
    MODE_SQUARE,
    MODE_SAWTOOTH,
    MODE_TRIANGLE,
    MODE_CHIRP,
    MODE_AM,
    MODE_IMPULSE,
} SignalMode;

static const char *mode_name[N_SIGNAL_MODES] = {
    "Sine sum", "Square  ", "Sawtooth", "Triangle",
    "Chirp   ", "AM      ", "Impulse ",
};

typedef struct {
    SignalMode    mode;
    SineComponent sine_components[N_SINE_COMPONENTS];
    float         frequency_single_bin;   /* used by square/saw/triangle */
    float         chirp_lo_bin;
    float         chirp_hi_bin;
    float         am_carrier_bin;
    float         am_modulator_bin;
    int           impulse_period_samples;
} SignalParams;

/* One-shot noise burst.  When the user presses 'n' we fill this with
 * fresh random values and set the decay envelope to 1.0; build_signal
 * adds this scaled by the decay envelope on every call. */
static float g_noise_buffer[N_FFT];
static float g_noise_decay_envelope = 0.0f;

static void build_signal(const SignalParams *p,
                         float *out_signal,
                         float anim_phase)
{
    /* Purpose : fill out_signal[0..N-1] with one buffer of the chosen
     *           waveform plus the active noise burst.
     * Pseudocode :
     *     for n in [0, N):
     *       v = mode_specific_sample(p, n)
     *       v += g_noise_buffer[n] * g_noise_decay_envelope
     *       out_signal[n] = v
     * Mental model : a switch on mode picks the right waveform formula.
     *                The noise term is added on top so noise+signal
     *                experiments work regardless of mode.
     * Inputs / outputs : params struct, output array, animation phase
     *                    / mutates output array.
     */
    for (int n = 0; n < N_FFT; n++) {
        float v = 0.0f;
        switch (p->mode) {
            case MODE_SINE_SUM: {
                for (int c = 0; c < N_SINE_COMPONENTS; c++) {
                    if (!p->sine_components[c].enabled) continue;
                    float freq = p->sine_components[c].frequency_bin;
                    float t    = (float)n / (float)N_FFT;
                    v += p->sine_components[c].amplitude
                       * sinf(2.0f * (float)M_PI * freq * t
                            + anim_phase * freq);
                }
                break;
            }
            case MODE_SQUARE:
                v = 0.7f * square_sample  (n, p->frequency_single_bin, anim_phase);
                break;
            case MODE_SAWTOOTH:
                v = 0.7f * sawtooth_sample(n, p->frequency_single_bin, anim_phase);
                break;
            case MODE_TRIANGLE:
                v = 0.8f * triangle_sample(n, p->frequency_single_bin, anim_phase);
                break;
            case MODE_CHIRP:
                v = 0.8f * chirp_sample(n, p->chirp_lo_bin, p->chirp_hi_bin);
                break;
            case MODE_AM:
                v = am_sample(n, p->am_carrier_bin, p->am_modulator_bin, anim_phase);
                break;
            case MODE_IMPULSE:
                v = impulse_sample(n, p->impulse_period_samples);
                break;
        }
        v += g_noise_buffer[n] * g_noise_decay_envelope;
        out_signal[n] = v;
    }
}

/* ===================================================================== */
/* §10  themes — palette table for the live colour-cycling               */
/* ===================================================================== */
/*
 * Five palettes, each providing five 256-colour codes for the
 * animation pairs.  All entries chosen from the bright half of the
 * 256-colour cube so the "small magnitude" tier remains visible
 * against default-black even at low intensity.  Switch live with
 * the 't' key.
 */

typedef struct {
    const char *display_name;
    short time_pos_color;
    short time_neg_color;
    short freq_low_color;
    short freq_high_color;
    short reserved_color;        /* future-proof slot, unused for now */
} ThemePalette;

static const ThemePalette theme_table[N_THEMES] = {
    { "Cyan-Green",  51,  39,  46,  82,  82 },
    { "Fire      ", 196, 202, 208, 226, 226 },
    { "Purple    ", 201, 171, 141, 231, 231 },
    { "Mono      ", 252, 245, 244, 255, 255 },
    { "Ocean     ", 195, 117,  75,  39,  39 },
};

/* ===================================================================== */
/* §11  colors — ncurses colour-pair init + apply_theme                  */
/* ===================================================================== */

static void apply_theme(int theme_index)
{
    /* Purpose : rebind the four animation colour pairs to a theme's
     *           palette.  HUD/HINT pairs are NOT touched.
     * Mental model : a stylesheet swap.
     */
    if (theme_index < 0 || theme_index >= N_THEMES) theme_index = 0;
    const ThemePalette *t = &theme_table[theme_index];
    init_pair(PAIR_BG,        -1,                 -1);
    init_pair(PAIR_TIME_POS,  t->time_pos_color,  -1);
    init_pair(PAIR_TIME_NEG,  t->time_neg_color,  -1);
    init_pair(PAIR_FREQ_LOW,  t->freq_low_color,  -1);
    init_pair(PAIR_FREQ_HIGH, t->freq_high_color, -1);
}

static void colors_init(int initial_theme_index)
{
    /* Bind theme-invariant pairs once at startup, then apply the
     * default theme.  Subsequent theme changes go through apply_theme
     * and leave HUD/HINT alone. */
    start_color();
    use_default_colors();
    init_pair(PAIR_HUD,  226, -1);    /* bright yellow */
    init_pair(PAIR_HINT,  51, -1);    /* bright cyan   */
    apply_theme(initial_theme_index);
}

/* ===================================================================== */
/* §12  draw_bar — vertical-bar primitive                                */
/* ===================================================================== */

static void draw_bar(int column, int baseline_row,
                     int bar_height_cells, bool growing_upward,
                     int colour_pair)
{
    /* Purpose : stamp a vertical column of glyphs starting at the
     *           baseline row, growing either upward or downward.
     * Pseudocode :
     *     if h <= 0: return
     *     for dy in 0..h-1:
     *       row = upward ? baseline - dy : baseline + dy + 1
     *       mvaddch(row, col, upward ? '|' : '.')
     * Mental model : positive-going lobe uses '|' growing UP from the
     *                baseline; negative-going lobe uses '.' growing
     *                DOWN.  Two glyphs let the eye distinguish sign
     *                without needing a colour difference (though we
     *                supply colour anyway).
     */
    if (bar_height_cells <= 0) return;
    attron(COLOR_PAIR(colour_pair));
    for (int dy = 0; dy < bar_height_cells; dy++) {
        int row = growing_upward ? (baseline_row - dy)
                                 : (baseline_row + dy + 1);
        if (row < 0 || row >= LINES) continue;
        if (column < 0 || column >= COLS) continue;
        mvaddch(row, column, growing_upward ? '|' : '.');
    }
    attroff(COLOR_PAIR(colour_pair));
}

/* ===================================================================== */
/* §13  spectrogram — ring buffer + draw_waterfall                       */
/* ===================================================================== */
/*
 * The spectrogram stores one quantised intensity tier (0..4) per
 * (time, frequency_bin) cell.  Quantisation is done at push time
 * rather than at render time so that the render stays a fast
 * lookup-and-stamp.
 *
 *      ┌──────────── ring-buffer indexing ──────────────────────┐
 *      │                                                        │
 *      │   capacity = 8                                         │
 *      │   spec_head = 3, spec_count = 5                        │
 *      │                                                        │
 *      │   slots:   0  1  2  3  4  5  6  7                      │
 *      │   data:   [N][N][N][_][_][_][O][O]                     │
 *      │                                                        │
 *      │   N = newest data; O = older data; _ = empty           │
 *      │                                                        │
 *      │   newest spectrum sits at slot (spec_head - 1) = 2     │
 *      │   oldest at (spec_head - spec_count + LEN) % LEN       │
 *      │                                                        │
 *      └────────────────────────────────────────────────────────┘
 */

#define DEBUG_NOTE_STUB_so_section_below_is_clean  1

static unsigned char spec_history_buffer[SPEC_HISTORY_LEN][N_FFT / 2];
static int           spec_history_head_index   = 0;
static int           spec_history_filled_count = 0;

static unsigned char quantise_intensity(float fraction_of_max)
{
    /* Map a normalised magnitude [0, 1] to a 5-tier glyph index.
     * Hard thresholds; could be made gamma-aware but the simple
     * version is plenty pedagogical. */
    if (fraction_of_max < 0.10f) return 0;
    if (fraction_of_max < 0.30f) return 1;
    if (fraction_of_max < 0.55f) return 2;
    if (fraction_of_max < 0.80f) return 3;
    return 4;
}

static void spec_history_push(const float *magnitude, float magnitude_peak)
{
    /* Push one column of quantised magnitudes into the ring at
     * spec_head; advance head; saturate count. */
    for (int k = 0; k < N_FFT / 2; k++) {
        float frac = magnitude[k] / (magnitude_peak + 1e-6f);
        spec_history_buffer[spec_history_head_index][k]
            = quantise_intensity(frac);
    }
    spec_history_head_index = (spec_history_head_index + 1) % SPEC_HISTORY_LEN;
    if (spec_history_filled_count < SPEC_HISTORY_LEN)
        spec_history_filled_count++;
}

static void spec_history_clear(void)
{
    spec_history_head_index   = 0;
    spec_history_filled_count = 0;
    memset(spec_history_buffer, 0, sizeof spec_history_buffer);
}

static void draw_waterfall(int top_row, int left_col,
                           int height_rows, int width_cols)
{
    /* Purpose : draw the spectrogram in [top_row, top_row + height_rows)
     *           with bin k in column (left_col + k).  Newest spectrum
     *           at the BOTTOM of the panel; older spectra above.
     * Pseudocode :
     *     for offset_from_newest in 0 .. min(height, count) - 1:
     *       idx = (head - 1 - offset + LEN) % LEN
     *       y   = top + height - 1 - offset
     *       for k in 0 .. width:
     *         glyph, pair = lookup_tier(spec_history[idx][k])
     *         if pair: mvaddch(y, x, glyph)
     */
    if (height_rows <= 0) return;
    if (width_cols  >  N_FFT / 2) width_cols = N_FFT / 2;

    int rows_to_draw = (height_rows < spec_history_filled_count)
                     ? height_rows : spec_history_filled_count;

    for (int offset = 0; offset < rows_to_draw; offset++) {
        int idx = (spec_history_head_index - 1 - offset + SPEC_HISTORY_LEN)
                  % SPEC_HISTORY_LEN;
        int y = top_row + height_rows - 1 - offset;
        if (y < 0 || y >= LINES) continue;

        for (int k = 0; k < width_cols; k++) {
            int x = left_col + k;
            if (x < 0 || x >= COLS) continue;
            unsigned char tier = spec_history_buffer[idx][k];
            if (tier == 0) continue;
            char glyph; int pair;
            switch (tier) {
                default: continue;
                case 1: glyph = '.'; pair = PAIR_FREQ_LOW;  break;
                case 2: glyph = '+'; pair = PAIR_FREQ_LOW;  break;
                case 3: glyph = 'o'; pair = PAIR_FREQ_HIGH; break;
                case 4: glyph = '#'; pair = PAIR_FREQ_HIGH; break;
            }
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(y, x, glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §14  layout — panel-size computation per terminal size                */
/* ===================================================================== */
/*
 * The screen is divided into:
 *   row 0           HUD top-right (and PAUSED indicator top-left)
 *   row 1           TIME panel label
 *   rows 2..        TIME panel
 *   ...             FREQ panel label
 *   ...             FREQ panel
 *   ...             SPECTROGRAM label  (if enabled)
 *   ...             SPECTROGRAM panel  (if enabled)
 *   row LINES-2     status (component readout or mode summary)
 *   row LINES-1     bottom-line key hint
 *
 * Heights of the three panels are recomputed every frame from
 * `LINES`, so resize works without explicit handling.  When the
 * spectrogram is OFF, time and freq each get half the available
 * space; when ON, they each get a quarter and the spectrogram gets
 * the remaining half.
 */

typedef struct {
    int time_label_row,  time_top_row,  time_height_rows,  time_baseline_row;
    int freq_label_row,  freq_top_row,  freq_height_rows,  freq_baseline_row;
    int water_label_row, water_top_row, water_height_rows;
    int status_row, hint_row;
} PanelLayout;

static PanelLayout compute_layout(bool show_waterfall)
{
    /* Reserved rows: 1 HUD top + 1 status + 1 hint + 1 time-label
     * + 1 freq-label + (1 if waterfall else 0) for waterfall-label. */
    int reserved = 5 + (show_waterfall ? 1 : 0);
    int avail    = LINES - reserved;
    if (avail < 6) avail = 6;

    int time_h, freq_h, water_h;
    if (show_waterfall) {
        time_h  = avail / 4; if (time_h  < 3) time_h  = 3;
        freq_h  = avail / 4; if (freq_h  < 3) freq_h  = 3;
        water_h = avail - time_h - freq_h;
        if (water_h < 3) water_h = 3;
    } else {
        time_h  = avail / 2; if (time_h < 3) time_h = 3;
        freq_h  = avail - time_h;
        water_h = 0;
    }

    PanelLayout L;
    L.time_label_row    = 1;
    L.time_top_row      = L.time_label_row + 1;
    L.time_height_rows  = time_h;
    L.time_baseline_row = L.time_top_row + time_h - 1;

    L.freq_label_row    = L.time_baseline_row + 1;
    L.freq_top_row      = L.freq_label_row + 1;
    L.freq_height_rows  = freq_h;
    L.freq_baseline_row = L.freq_top_row + freq_h - 1;

    if (show_waterfall) {
        L.water_label_row    = L.freq_baseline_row + 1;
        L.water_top_row      = L.water_label_row + 1;
        L.water_height_rows  = water_h;
    } else {
        L.water_label_row    = 0;
        L.water_top_row      = 0;
        L.water_height_rows  = 0;
    }
    L.status_row = LINES - 2;
    L.hint_row   = LINES - 1;
    return L;
}

/* ===================================================================== */
/* §15  draw_time — TIME panel                                           */
/* ===================================================================== */

static void draw_time_panel(const float *signal_pre_window,
                            float        signal_peak,
                            const PanelLayout *L)
{
    /* Purpose : render the time-domain waveform as ± vertical bars
     *           around a horizontal midline at the panel's centre.
     * Pseudocode :
     *     half_height = panel_h / 2
     *     midline = top + half_height
     *     for n in 0 .. min(N, COLS):
     *       v = signal[n] / peak   (auto-scaled)
     *       h = round(|v| · half_height)
     *       draw_bar(n, midline, h, v >= 0, sign-coloured)
     * Mental model : positive samples grow as '|' upward; negative
     *                samples grow as '.' downward.
     */
    int half_height = L->time_height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = L->time_top_row + half_height;

    int columns_to_draw = COLS < N_FFT ? COLS : N_FFT;
    for (int n = 0; n < columns_to_draw; n++) {
        float v   = signal_pre_window[n] / (signal_peak + 1e-6f);
        bool  pos = v >= 0.0f;
        int   h   = (int)(fabsf(v) * (float)half_height + 0.5f);
        draw_bar(n, midline_row, h, pos, pos ? PAIR_TIME_POS : PAIR_TIME_NEG);
    }
}

/* ===================================================================== */
/* §16  draw_freq — FREQ panel                                           */
/* ===================================================================== */

static int draw_freq_panel(const float *magnitude,
                           float        magnitude_peak,
                           const PanelLayout *L)
{
    /* Purpose : render the magnitude spectrum as upward bars from a
     *           baseline at the bottom of the panel.  Each bin uses
     *           1 or 2 columns depending on terminal width.
     * Returns : the maximum bin index actually drawn (so the
     *           waterfall can match its width).
     */
    int bin_width_cells = (COLS / (N_FFT / 2)) >= 2 ? 2 : 1;
    int max_bins        = COLS / bin_width_cells;
    if (max_bins > N_FFT / 2) max_bins = N_FFT / 2;

    for (int k = 0; k < max_bins; k++) {
        float v       = magnitude[k] / (magnitude_peak + 1e-6f);
        int   h       = (int)(v * (float)L->freq_height_rows + 0.5f);
        int   pair_id = (h > L->freq_height_rows / 2)
                      ? PAIR_FREQ_HIGH : PAIR_FREQ_LOW;
        for (int bx = 0; bx < bin_width_cells; bx++)
            draw_bar(k * bin_width_cells + bx, L->freq_baseline_row,
                     h, true, pair_id);
    }
    return max_bins;
}

/* ===================================================================== */
/* §17  draw_debug — 'd' window-envelope, 'D' bin phase table            */
/* ===================================================================== */
/*
 * Two non-invasive teaching overlays.  Neither modifies any state
 * the algorithm uses; both just READ algorithm outputs.
 *
 *   'd' WINDOW-ENVELOPE  Draw the active window function as a faint
 *                        dotted curve on top of the time panel.
 *                        Visualises what the window does to the
 *                        signal — the bell shape of Hann is unmistakable.
 *
 *   'D' BIN PHASE TABLE  In the top-left corner, list the eight
 *                        loudest bins with magnitude and phase.
 *                        Pause and watch how phase advances when you
 *                        un-pause (showing what magnitude throws away).
 */

#define DEBUG_PHASE_TABLE_ROWS  8

static void draw_debug_window_overlay(WindowKind kind, const PanelLayout *L)
{
    /* Trace y = window_coefficient(n) over the time panel as '*'
     * marks at the height that the value would reach if it were a
     * positive-going time sample. */
    if (kind == WINDOW_RECT) return;   /* nothing to show */
    int half_height = L->time_height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = L->time_top_row + half_height;

    int columns = COLS < N_FFT ? COLS : N_FFT;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int n = 0; n < columns; n++) {
        float w = window_coefficient(kind, n, N_FFT);
        int   h = (int)(w * (float)half_height + 0.5f);
        int   y = midline_row - h;
        if (y >= 0 && y < LINES) mvaddch(y, n, '*');
    }
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void draw_debug_phase_table(const float *fft_real,
                                   const float *fft_imag,
                                   const float *magnitude)
{
    /* Sort bin indices by descending magnitude (selection sort over
     * the first N/2 bins is fine — N/2 = 64).  Print the top
     * DEBUG_PHASE_TABLE_ROWS entries. */
    int sorted_bin_indices[N_FFT / 2];
    for (int i = 0; i < N_FFT / 2; i++) sorted_bin_indices[i] = i;
    for (int i = 0; i < DEBUG_PHASE_TABLE_ROWS && i < N_FFT / 2; i++) {
        int max_at = i;
        for (int j = i + 1; j < N_FFT / 2; j++)
            if (magnitude[sorted_bin_indices[j]]
              > magnitude[sorted_bin_indices[max_at]])
                max_at = j;
        int tmp = sorted_bin_indices[i];
        sorted_bin_indices[i]      = sorted_bin_indices[max_at];
        sorted_bin_indices[max_at] = tmp;
    }

    int x = 2, y = 2;
    if (y + DEBUG_PHASE_TABLE_ROWS + 1 >= LINES) return;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, "  bin   |X[k]|     arg(rad)");
    attroff(COLOR_PAIR(PAIR_HINT));

    for (int i = 0; i < DEBUG_PHASE_TABLE_ROWS && i < N_FFT / 2; i++) {
        int k = sorted_bin_indices[i];
        float phase_radians = atan2f(fft_imag[k], fft_real[k]);
        attron(COLOR_PAIR(PAIR_FREQ_HIGH) | A_BOLD);
        mvprintw(y + 1 + i, x, " %4d  %7.4f   %+6.3f",
                 k, (double)magnitude[k], (double)phase_radians);
        attroff(COLOR_PAIR(PAIR_FREQ_HIGH) | A_BOLD);
    }
}

/* ===================================================================== */
/* §18  draw_hud — HUD top-right + bottom hint                           */
/* ===================================================================== */

static void draw_hud_top(int theme_index, bool paused)
{
    char buf[160];
    snprintf(buf, sizeof buf,
             " FFT  N=%d  FFT %d ops vs DFT %d ops  thm:%s ",
             N_FFT, N_FFT * LOG2_N_FFT, N_FFT * N_FFT,
             theme_table[theme_index].display_name);
    int x = COLS - (int)strlen(buf);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    if (paused) {
        attron(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
        mvprintw(0, 0, " PAUSED ");
        attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
    }
}

static void draw_hud_hint(int hint_row)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(hint_row, 0,
             " q:quit spc:pause m/M:mode w/W:win 1-3 j/k +/- ,/. n:noise"
             " g:waterfall t/T:theme d:wenv D:phase r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §19  app_state — App struct + defaults + adjust_freq                  */
/* ===================================================================== */

typedef struct {
    SignalParams sig_params;
    int          selected_component_index;
    WindowKind   active_window;
    int          active_theme_index;
    bool         paused;
    bool         show_waterfall;
    bool         show_debug_window_overlay;
    bool         show_debug_phase_table;

    float        animation_phase_radians;

    /* Working buffers */
    float        signal_pre_window [N_FFT];
    float        fft_real_buffer   [N_FFT];
    float        fft_imag_buffer   [N_FFT];
    float        magnitude         [N_FFT / 2];
    float        magnitude_peak;
    float        signal_peak;
} App;

static void app_set_defaults(App *a)
{
    a->sig_params.mode                    = MODE_SINE_SUM;
    a->sig_params.sine_components[0]      = (SineComponent){  4.0f, 1.0f, true };
    a->sig_params.sine_components[1]      = (SineComponent){ 11.0f, 0.6f, true };
    a->sig_params.sine_components[2]      = (SineComponent){ 23.0f, 0.4f, true };
    a->sig_params.frequency_single_bin    = 5.0f;
    a->sig_params.chirp_lo_bin            = 2.0f;
    a->sig_params.chirp_hi_bin            = 30.0f;
    a->sig_params.am_carrier_bin          = 16.0f;
    a->sig_params.am_modulator_bin        = 3.0f;
    a->sig_params.impulse_period_samples  = 16;

    a->selected_component_index           = 0;
    a->active_window                      = WINDOW_RECT;
    a->active_theme_index                 = 0;
    a->paused                             = false;
    a->show_waterfall                     = true;
    a->show_debug_window_overlay          = false;
    a->show_debug_phase_table             = false;
    a->animation_phase_radians            = 0.0f;
    spec_history_clear();
}

static void app_adjust_freq(App *a, float delta_bins)
{
    /* Coarse and fine '+'/'-'/','/'.' keys both come through here.
     * Which parameter we adjust depends on the active mode. */
    float lo = 1.0f, hi = (float)N_FFT * 0.5f - 1.0f;
    switch (a->sig_params.mode) {
        case MODE_SINE_SUM: {
            SineComponent *c =
                &a->sig_params.sine_components[a->selected_component_index];
            c->frequency_bin += delta_bins;
            if (c->frequency_bin < lo) c->frequency_bin = lo;
            if (c->frequency_bin > hi) c->frequency_bin = hi;
            break;
        }
        case MODE_CHIRP:
            a->sig_params.chirp_hi_bin += delta_bins;
            if (a->sig_params.chirp_hi_bin < a->sig_params.chirp_lo_bin + 1.0f)
                a->sig_params.chirp_hi_bin = a->sig_params.chirp_lo_bin + 1.0f;
            if (a->sig_params.chirp_hi_bin > hi)
                a->sig_params.chirp_hi_bin = hi;
            break;
        case MODE_AM:
            a->sig_params.am_carrier_bin += delta_bins;
            if (a->sig_params.am_carrier_bin
              < a->sig_params.am_modulator_bin + 1.0f)
                a->sig_params.am_carrier_bin
                  = a->sig_params.am_modulator_bin + 1.0f;
            if (a->sig_params.am_carrier_bin > hi)
                a->sig_params.am_carrier_bin = hi;
            break;
        case MODE_IMPULSE: {
            int step = (delta_bins >=  0.5f) ?  1
                     : (delta_bins <= -0.5f) ? -1 : 0;
            a->sig_params.impulse_period_samples += step;
            if (a->sig_params.impulse_period_samples < 2)
                a->sig_params.impulse_period_samples = 2;
            if (a->sig_params.impulse_period_samples > N_FFT / 2)
                a->sig_params.impulse_period_samples = N_FFT / 2;
            break;
        }
        default:
            a->sig_params.frequency_single_bin += delta_bins;
            if (a->sig_params.frequency_single_bin < lo)
                a->sig_params.frequency_single_bin = lo;
            if (a->sig_params.frequency_single_bin > hi)
                a->sig_params.frequency_single_bin = hi;
            break;
    }
}

static void mode_param_summary(const App *a, char *buf, size_t buflen)
{
    /* One-line summary of the active mode's frequency parameter,
     * shown next to the TIME panel label. */
    const SignalParams *p = &a->sig_params;
    switch (p->mode) {
        case MODE_SINE_SUM:
            snprintf(buf, buflen, "[c%d f=%.1f]",
                     a->selected_component_index + 1,
                     (double)p->sine_components[a->selected_component_index]
                       .frequency_bin);
            break;
        case MODE_CHIRP:
            snprintf(buf, buflen, "lo=%.1f hi=%.1f",
                     (double)p->chirp_lo_bin, (double)p->chirp_hi_bin);
            break;
        case MODE_AM:
            snprintf(buf, buflen, "carrier=%.1f mod=%.1f",
                     (double)p->am_carrier_bin, (double)p->am_modulator_bin);
            break;
        case MODE_IMPULSE:
            snprintf(buf, buflen, "period=%d", p->impulse_period_samples);
            break;
        default:
            snprintf(buf, buflen, "f=%.1f", (double)p->frequency_single_bin);
            break;
    }
}

/* ===================================================================== */
/* §20  app_compute — per-frame orchestration                            */
/* ===================================================================== */

static void app_compute(App *a)
{
    /* Purpose : run one full frame of the signal pipeline.
     * Pseudocode :
     *     build_signal           → signal_pre_window
     *     copy + apply_window    → fft_real_buffer (imag := 0)
     *     fft                    → fft_real_buffer + fft_imag_buffer
     *     magnitude_normalised   → magnitude[]
     *     spec_history_push      (only when not paused)
     *     compute peaks for auto-scaling time + freq panels
     */
    build_signal(&a->sig_params, a->signal_pre_window,
                 a->animation_phase_radians);

    for (int n = 0; n < N_FFT; n++) {
        a->fft_real_buffer[n] = a->signal_pre_window[n];
        a->fft_imag_buffer[n] = 0.0f;
    }
    apply_window(a->fft_real_buffer, N_FFT, a->active_window);

    fft(a->fft_real_buffer, a->fft_imag_buffer, N_FFT);

    a->magnitude_peak = 1e-6f;
    for (int k = 0; k < N_FFT / 2; k++) {
        a->magnitude[k] = sqrtf(a->fft_real_buffer[k] * a->fft_real_buffer[k]
                              + a->fft_imag_buffer[k] * a->fft_imag_buffer[k])
                        / ((float)N_FFT * 0.5f);
        if (a->magnitude[k] > a->magnitude_peak)
            a->magnitude_peak = a->magnitude[k];
    }

    a->signal_peak = 1e-6f;
    for (int n = 0; n < N_FFT; n++) {
        float av = fabsf(a->signal_pre_window[n]);
        if (av > a->signal_peak) a->signal_peak = av;
    }

    if (!a->paused && a->show_waterfall)
        spec_history_push(a->magnitude, a->magnitude_peak);
}

static void app_draw(const App *a)
{
    /* Compose the frame: erase, panels, debug overlays, HUD, present.
     * Layer order matters — later layers paint over earlier in any
     * cells they share.  HUD/HINT always last. */
    erase();
    PanelLayout L = compute_layout(a->show_waterfall);

    /* Panel labels */
    char param_buf[64];
    mode_param_summary(a, param_buf, sizeof param_buf);
    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(L.time_label_row, 0,
             "TIME x[n]   mode:%s   win:%s   %s",
             mode_name[a->sig_params.mode],
             window_name[a->active_window],
             param_buf);
    mvprintw(L.freq_label_row, 0,
             "FREQ |X[k]|   bins 0..%d   peak=%.3f",
             N_FFT / 2 - 1, (double)a->magnitude_peak);
    if (a->show_waterfall) {
        mvprintw(L.water_label_row, 0,
                 "SPECTROGRAM  newest=bottom  history=%d",
                 spec_history_filled_count);
    }
    attroff(COLOR_PAIR(PAIR_HINT));

    /* Panels */
    draw_time_panel(a->signal_pre_window, a->signal_peak, &L);
    int max_bins_drawn = draw_freq_panel(a->magnitude, a->magnitude_peak, &L);
    if (a->show_waterfall && L.water_height_rows > 0)
        draw_waterfall(L.water_top_row, 0,
                       L.water_height_rows, max_bins_drawn);

    /* Debug overlays */
    if (a->show_debug_window_overlay)
        draw_debug_window_overlay(a->active_window, &L);
    if (a->show_debug_phase_table)
        draw_debug_phase_table(a->fft_real_buffer,
                               a->fft_imag_buffer,
                               a->magnitude);

    /* Status row */
    attron(COLOR_PAIR(PAIR_HUD));
    if (a->sig_params.mode == MODE_SINE_SUM) {
        mvprintw(L.status_row, 0, "Components: ");
        for (int c = 0; c < N_SINE_COMPONENTS; c++) {
            if (c == a->selected_component_index) attron(A_REVERSE);
            if (a->sig_params.sine_components[c].enabled) attron(A_BOLD);
            printw("[%d] f=%.1f a=%.1f%s  ", c + 1,
                   (double)a->sig_params.sine_components[c].frequency_bin,
                   (double)a->sig_params.sine_components[c].amplitude,
                   a->sig_params.sine_components[c].enabled ? "" : " OFF");
            attroff(A_BOLD); attroff(A_REVERSE);
        }
    } else {
        mvprintw(L.status_row, 0, "Mode: %s   %s",
                 mode_name[a->sig_params.mode], param_buf);
    }
    attroff(COLOR_PAIR(PAIR_HUD));

    /* HUD top + hint bottom */
    draw_hud_top(a->active_theme_index, a->paused);
    draw_hud_hint(L.hint_row);

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §21  app_input — key dispatch                                         */
/* ===================================================================== */

static bool app_handle_key(App *a, int ch)
{
    /* Returns true if the program should quit, false otherwise. */
    switch (ch) {
        case 'q': case 'Q': case 27:
            return true;

        case ' ':
            a->paused = !a->paused;
            break;

        /* ── sine-sum component controls (no-op outside MODE_SINE_SUM) ── */
        case '1':
            if (a->sig_params.mode == MODE_SINE_SUM)
                a->sig_params.sine_components[0].enabled =
                    !a->sig_params.sine_components[0].enabled;
            break;
        case '2':
            if (a->sig_params.mode == MODE_SINE_SUM)
                a->sig_params.sine_components[1].enabled =
                    !a->sig_params.sine_components[1].enabled;
            break;
        case '3':
            if (a->sig_params.mode == MODE_SINE_SUM)
                a->sig_params.sine_components[2].enabled =
                    !a->sig_params.sine_components[2].enabled;
            break;
        case 'j':
            if (a->sig_params.mode == MODE_SINE_SUM)
                a->selected_component_index =
                    (a->selected_component_index + 1) % N_SINE_COMPONENTS;
            break;
        case 'k':
            if (a->sig_params.mode == MODE_SINE_SUM)
                a->selected_component_index =
                    (a->selected_component_index + N_SINE_COMPONENTS - 1)
                    % N_SINE_COMPONENTS;
            break;

        /* ── frequency adjustment ── */
        case '+': case '=':  app_adjust_freq(a, +1.0f); break;
        case '-':            app_adjust_freq(a, -1.0f); break;
        case '.':            app_adjust_freq(a, +0.1f); break;
        case ',':            app_adjust_freq(a, -0.1f); break;

        /* ── mode + window cycling ── */
        case 'm':
            a->sig_params.mode = (SignalMode)((a->sig_params.mode + 1)
                                              % N_SIGNAL_MODES);
            spec_history_clear();
            break;
        case 'M':
            a->sig_params.mode = (SignalMode)((a->sig_params.mode
                                              + N_SIGNAL_MODES - 1)
                                              % N_SIGNAL_MODES);
            spec_history_clear();
            break;
        case 'w':
            a->active_window = (WindowKind)((a->active_window + 1)
                                            % N_WINDOWS);
            break;
        case 'W':
            a->active_window = (WindowKind)((a->active_window + N_WINDOWS - 1)
                                            % N_WINDOWS);
            break;

        /* ── overlays + misc ── */
        case 'g': case 'G':
            a->show_waterfall = !a->show_waterfall;
            spec_history_clear();
            break;
        case 'd':
            a->show_debug_window_overlay = !a->show_debug_window_overlay;
            break;
        case 'D':
            a->show_debug_phase_table = !a->show_debug_phase_table;
            break;
        case 'n': case 'N':
            for (int i = 0; i < N_FFT; i++)
                g_noise_buffer[i] = ((float)rand() / (float)RAND_MAX
                                     * 2.0f - 1.0f) * NOISE_AMP_PEAK;
            g_noise_decay_envelope = 1.0f;
            break;
        case 't':
            a->active_theme_index = (a->active_theme_index + 1) % N_THEMES;
            apply_theme(a->active_theme_index);
            break;
        case 'T':
            a->active_theme_index = (a->active_theme_index + N_THEMES - 1)
                                    % N_THEMES;
            apply_theme(a->active_theme_index);
            break;
        case 'r': case 'R':
            app_set_defaults(a);
            apply_theme(a->active_theme_index);
            break;

        default: break;
    }
    return false;
}

/* ===================================================================== */
/* §22  app_main — signal handlers + main loop                           */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    /* Async-signal-safe: only set flags; the main loop handles them. */
    if (sig == SIGWINCH) g_resize_pending = 1;
    else                 g_should_quit    = 1;
}

static void cleanup_screen(void)
{
    endwin();
}

int main(void)
{
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
    atexit(cleanup_screen);

    initscr();
    cbreak();
    noecho();
    curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);                /* don't let stdin peek interrupt blits */

    App a;
    app_set_defaults(&a);
    colors_init(a.active_theme_index);

    long long next_frame_ns = clock_now_ns();

    while (!g_should_quit) {
        /* ── resize handling ── */
        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            spec_history_clear();   /* widths may have changed */
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (app_handle_key(&a, ch)) {
                g_should_quit = 1;
                break;
            }
        }

        /* ── tick + render at the frame slot ── */
        long long now = clock_now_ns();
        if (now >= next_frame_ns) {
            if (!a.paused) {
                a.animation_phase_radians += ANIMATION_PHASE_STEP_RAD;
                if (a.animation_phase_radians > 2.0f * (float)M_PI)
                    a.animation_phase_radians -= 2.0f * (float)M_PI;
                if (g_noise_decay_envelope > 0.0f)
                    g_noise_decay_envelope -= NOISE_DECAY_RATE;
                if (g_noise_decay_envelope < 0.0f)
                    g_noise_decay_envelope = 0.0f;
            }
            app_compute(&a);
            app_draw(&a);
            next_frame_ns += RENDER_TICK_NS;
            /* Snap forward if the terminal was suspended/hidden so we
             * don't spend the next several frames "catching up". */
            if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
                next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
        } else {
            clock_sleep_ns(next_frame_ns - now);
        }
    }

    return 0;
}
