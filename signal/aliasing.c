/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aliasing.c — animated, minimum-viable Nyquist / aliasing demo.
 *              A "true" continuous sine wave is shown alongside its
 *              SAMPLED version.  As the true frequency sweeps past
 *              the Nyquist limit fs/2, the sampled version appears
 *              to FOLD BACK to a lower frequency — the visual proof
 *              of the sampling theorem.  The fold-back math itself
 *              reads as pseudocode so the formula and the code are
 *              line-for-line identical.
 *
 * DEMO
 *   Two stacked panels.
 *     TOP        "continuous" signal x(t), drawn at high resolution
 *                (HIGH_RES_FACTOR = 8 sub-samples per actual sample)
 *                so it looks smooth.  The GROUND TRUTH.
 *     BOTTOM     what the sampler sees:
 *                  - bright '*' markers at the N sample positions
 *                  - green Bresenham line segments connecting samples
 *                    (linear-interp reconstruction — what you would
 *                    see if you reconstructed from samples)
 *                  - faint grey dotted GHOST of the continuous signal
 *                    underneath, so you can compare directly
 *
 *   Auto-sweep walks the true frequency from 0.5 cycles/buffer up to
 *   2 * fs over a few seconds.  As frequency rises past Nyquist
 *   (fs/2 = N/2 = 16 cycles/buffer here), the sampled wave stops
 *   matching the ghost and starts resembling a LOWER frequency.
 *   Past 2 * fs/2 = N it folds back and starts climbing again.
 *
 *   The HUD prints the "perceived" frequency live.  When true_f >
 *   fs/2 it goes red and shows "ALIASED".
 *
 *   Press 'a' for manual mode: '+' / '-' steps by 0.5 cycle/buffer;
 *   ',' / '.' for 0.1 fine motion.
 *
 *   The headline lesson: digital sampling at rate fs CANNOT
 *   distinguish frequencies above fs/2 from frequencies below it.
 *   Hidden frequency information masquerades as something else.
 *   This is why anti-aliasing filters exist — you HAVE to remove
 *   above-Nyquist content BEFORE sampling, because afterwards
 *   there is no way to tell what was real and what was an alias.
 *
 *   Two debug overlays:
 *     'd' shows the FOLD-BACK ARITHMETIC table — for the current
 *         true frequency, prints the derivation of the alias
 *         (f mod fs, then conditional fold).
 *     'D' shows a numeric FREQUENCY TABLE — true freq, alias freq,
 *         Nyquist, sample rate, normalised cutoff, etc.
 *
 * Study alongside
 *   signal/dft_helloworld.c    The DFT can only EXPRESS frequencies
 *                               in [0, fs/2].  Above that maps to
 *                               negative frequencies via conjugate
 *                               symmetry.  Same fold-back, viewed
 *                               from the spectrum side.
 *   signal/fft_vis.c           Spectral leakage — the related
 *                               phenomenon when frequency falls
 *                               between bins (off-integer cycles
 *                               per buffer).  Different from aliasing
 *                               (which is above fs/2).
 *   signal/fir_filter.c        Anti-aliasing filters in practice are
 *                               low-pass FIR filters applied BEFORE
 *                               sampling.
 *
 * Section map
 *   §1   config            constants
 *   §2   clock             monotonic-ns timer + sleep
 *   §3   colors            palette pairs (HUD/HINT theme-invariant)
 *   §4   aliased_freq      THE fold-back arithmetic (pseudocode-style)
 *   §5   signal_continuous high-res cosine for the top panel
 *   §6   signal_sampled    N sample points for the bottom panel
 *   §7   scene_state       per-frame state + reset
 *   §8   scene_tick        per-frame update orchestrator
 *   §9   scene_input       handle keys
 *   §10  draw_continuous   top panel renderer
 *   §11  draw_ghost        ghost overlay helper
 *   §12  draw_recon        Bresenham segment helper
 *   §13  draw_sampled      bottom panel: samples + lines + ghost
 *   §14  draw_debug        'd' fold-back overlay + 'D' frequency table
 *   §15  hud               status + hint + paused chip + frame composer
 *   §16  app               signal handlers + main loop + key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   a                toggle auto-sweep <-> manual mode
 *   + / -            (manual) change frequency by 0.5 cycle/buffer
 *   , / .            (manual) change frequency by 0.1 cycle/buffer
 *   d                toggle fold-back arithmetic overlay (debug)
 *   D                toggle frequency numeric table (debug)
 *   r                reset (auto-sweep on, freq = 0.5)
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/aliasing.c \
 *       -o aliasing -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no DSP background:
 *     1. CONCEPTS — the elevator pitch with references.
 *     2. MENTAL MODEL — the wagon-wheel analogy + fold-back diagram.
 *     3. GUIDED TUTORIAL T1..T11 — eleven mini-lessons that derive
 *        the Nyquist-Shannon theorem from first principles, with
 *        worked examples at f = fs/2, f = fs, f = 1.5*fs.
 *     4. §1 config — the knobs.
 *     5. §4 aliased_freq — THE function.  Two lines of math, one
 *        teaching block.  Read this, then everything else clicks.
 *     6. §5..§6 signal generators — what the two panels show.
 *     7. §8 scene_tick — per-frame pipeline.
 *     8. §10..§15 — rendering plumbing.
 *     9. §16 app — main loop + key dispatch.
 *
 *   Reader who already knows aliasing, just wants the code:
 *     §1, §4, §8 scene_tick.  The rest is ncurses scaffolding.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     g_true_frequency_cycles_per_buffer    the actual signal frequency
 *     g_aliased_frequency_cycles_per_buffer the perceived frequency
 *                                            (after sampling)
 *     g_continuous[HIGH_RES_N]              high-res ground-truth signal
 *     g_sampled[N]                          discrete sample values
 *     g_simulation_paused                   pause flag
 *     g_auto_sweep_enabled                  auto-sweep flag
 *     g_animation_phase_radians             sweep's own phase
 *
 *   Inside tight loops counters (`n`, `c`, `i`) stay short.
 *
 * Background you NEED
 *   - Basic trigonometry: cos and sin as projections of a unit-
 *     circle point.
 *   - Concept of sampling: taking discrete snapshots of a continuous
 *     signal at regular intervals.
 *
 * Background you do NOT need
 *   - Continuous calculus or integral transforms.  Everything here
 *     is finite arithmetic.
 *   - The DFT or FFT.  The aliasing phenomenon predates them and
 *     can be understood from first principles.
 *   - z-transforms, complex analysis, signal-processing background.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * What is aliasing, in one sentence?
 *   When you sample a sinusoid at rate fs, frequencies above fs/2
 *   become INDISTINGUISHABLE from lower frequencies — they fold
 *   back into the [0, fs/2] range and masquerade as something else.
 *
 * Algorithm
 *   1. Build a "continuous" signal at high temporal resolution
 *      (HIGH_RES_FACTOR sub-samples per actual sample).  This is
 *      our ground-truth sine wave.
 *   2. Build the "sampled" signal at the actual sample rate (N
 *      samples per buffer).  Same cosine formula, but evaluated at
 *      only N positions instead of HIGH_RES_N.
 *   3. Display both panels.  In the bottom panel, draw line segments
 *      connecting consecutive samples (linear-interp reconstruction)
 *      and a faint ghost of the continuous signal.
 *   4. Compute the ALIASED frequency the sampled signal *appears*
 *      to have.  HUD prints true vs perceived.
 *
 * Math
 *   The Nyquist-Shannon sampling theorem (1928, 1949):
 *     A signal can be perfectly reconstructed from samples taken at
 *     rate fs only if it contains no frequencies above fs/2.
 *
 *   Aliased frequency formula (for a single sinusoid at frequency f
 *   sampled at rate fs):
 *     f_mod = f mod fs
 *     if f_mod > fs/2: f_alias = fs - f_mod
 *     else:            f_alias = f_mod
 *
 *   In our case fs = N = 32, so fs/2 = 16.  The fold-back happens
 *   at f = 16, then again at f = N = 32, then at f = 48, etc.
 *
 *   Why the fold?  A discrete sinusoid at f cycles/buffer is
 *   identical to one at (f + k*N) cycles/buffer for any integer k,
 *   AND identical (modulo a sign flip on the imaginary part) to
 *   one at (k*N - f) cycles/buffer.  Sampling THROWS AWAY the
 *   integer part of f/fs and only keeps the fractional position
 *   relative to the [0, fs/2] "fundamental zone".
 *
 * Performance / boundaries
 *   - Continuous signal at HIGH_RES_FACTOR = 8 sub-samples per
 *     actual sample → 256 cosines per frame.  Trivial.
 *   - Sampled signal: N = 32 cosines per frame.  Trivial.
 *   - All signals are simple sinusoids (one frequency at a time).
 *     Real-world aliasing in audio / images is more complex but
 *     the fundamental phenomenon is what this demo shows.
 *
 * Rendering
 *   Two stacked vertical-bar panels.  The bottom panel additionally
 *   draws connecting line segments between samples (using simple
 *   linear interpolation) and a faint dotted ghost of the
 *   continuous signal underneath, so the eye can compare "what we
 *   would reconstruct from samples" vs "what the signal really was".
 *
 * References
 *   Nyquist, H. (1928), "Certain topics in telegraph transmission
 *     theory", Trans. AIEE, vol. 47.  The original paper.
 *   Shannon, C. E. (1949), "Communication in the presence of noise",
 *     Proc. IRE, vol. 37.  The full proof of the sampling theorem.
 *   Smith, S. W., "The Scientist and Engineer's Guide to Digital
 *     Signal Processing", Chapter 3.  Free online; the gentlest
 *     introduction to sampling theory.
 *   https://en.wikipedia.org/wiki/Nyquist%E2%80%93Shannon_sampling_theorem
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   You can't represent an arbitrarily fast oscillation with N
 *   samples per second.  At sampling rate fs, the FASTEST oscillation
 *   you can faithfully capture is one that completes a half-cycle
 *   between samples — frequency fs/2 (the Nyquist limit).  Anything
 *   faster wraps back and APPEARS to be a lower frequency.
 *
 *   The wagon-wheel effect: a wagon wheel filmed at 24 fps appears
 *   to spin backward when its true rotation rate exceeds 12
 *   revolutions per second (= 24/2 = Nyquist).  The camera (the
 *   sampler) has thrown away the high-frequency rotation; what's
 *   left is the lower-frequency apparent motion.
 *
 *   In digital audio: anything you record above fs/2 doesn't just
 *   disappear — it FOLDS BACK and corrupts your low-frequency
 *   content.  This is why every ADC in the world has an
 *   anti-aliasing filter (low-pass at fs/2) sitting in front of it.
 *
 * HOW TO THINK ABOUT IT — THE PICKET-FENCE ANALOGY
 *   Imagine standing next to a very tall picket fence and trying to
 *   describe a bird flying past.  If you can blink only once per
 *   second and the bird flaps its wings 5 times per second, each
 *   blink catches the wings in a different position — but you have
 *   no way to know if the wings flapped once between blinks or 6
 *   times.  ANY frequency that completes an integer number of wing-
 *   flaps between your blinks looks the same as 0 flaps.
 *
 *   That's the fold-back.  Your "blinks per second" is fs.  Wing
 *   flaps faster than fs/2 alias to slower-looking flaps.
 *
 *      ┌──────────────────────────────────────────────────────────┐
 *      │                                                          │
 *      │   FOLD-BACK ARITHMETIC  (fs = 32):                       │
 *      │                                                          │
 *      │   true f      perceived f      status                    │
 *      │     0..16        =  true       fine, below Nyquist       │
 *      │      16          =  16         AT Nyquist                │
 *      │     16..32       =  32 - true  ALIASED, decreasing       │
 *      │      32          =  0          full alias to DC          │
 *      │     32..48       =  true - 32  ALIASED, increasing       │
 *      │      48          =  16         AT Nyquist again          │
 *      │     48..64       =  64 - true  ALIASED, decreasing       │
 *      │     ...                                                  │
 *      │                                                          │
 *      │   Pattern: the alias is a SAWTOOTH function of true f,   │
 *      │   bouncing between 0 and fs/2 with each fs increment.    │
 *      │                                                          │
 *      └──────────────────────────────────────────────────────────┘
 *
 * THE CONTINUOUS-VS-SAMPLED COMPARISON
 *
 *      ┌──────────────────────────────────────────────────────────┐
 *      │                                                          │
 *      │   At true_f = 24, fs = 32:  alias = 32 - 24 = 8          │
 *      │                                                          │
 *      │   Continuous (top panel):                                │
 *      │     ___ ___ ___ ___ ___ ___ ___ ___ ___ ___              │
 *      │    /   X   X   X   X   X   X   X   X   X   \             │
 *      │     ~24 cycles in the buffer                             │
 *      │                                                          │
 *      │   Sampled (bottom panel):                                │
 *      │      *           *           *           *               │
 *      │     / \         / \         / \         / \              │
 *      │    /   \       /   \       /   \       /   \             │
 *      │            *           *           *                    │
 *      │     8 cycles visible (the alias)                         │
 *      │                                                          │
 *      │   The faint ghost of the 24-cycle truth is drawn under   │
 *      │   the bright 8-cycle reconstruction so the divergence    │
 *      │   is immediate.                                          │
 *      │                                                          │
 *      └──────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (per frame)
 *   1. (Auto-sweep) advance the true frequency.
 *   2. Compute the continuous signal at HIGH_RES_FACTOR sub-samples
 *      per actual sample.  Store in g_continuous[].
 *   3. Compute the sampled signal at N positions.  Store in
 *      g_sampled[].
 *   4. Compute the aliased (perceived) frequency for the HUD via
 *      §4 aliased_frequency().
 *   5. Render: continuous panel, sampled panel (with ghost overlay
 *      and connecting lines), HUD, debug overlays.
 *
 * KEY FORMULAS
 *
 *   Continuous signal (high-res view of the cosine):
 *     g_continuous[i] = cos(2*pi * f * i / HIGH_RES_N)
 *     for i in [0, HIGH_RES_N), HIGH_RES_N = N * HIGH_RES_FACTOR
 *
 *   Sampled signal (the actual ADC output):
 *     g_sampled[n] = cos(2*pi * f * n / N)
 *     for n in [0, N)
 *
 *   Aliased frequency:
 *     f_mod   = f mod fs              (fs = N here)
 *     f_alias = (f_mod > fs/2) ? fs - f_mod : f_mod
 *
 *   The reconstructed wave (linear interp between samples):
 *     Connect g_sampled[n] to g_sampled[n+1] with a straight line.
 *
 * EDGE CASES TO WATCH
 *   - At f = 0: signal is constant DC.  Both panels show flat lines.
 *   - At f = fs/2 (Nyquist): the sampled signal is exactly +1, -1,
 *     +1, -1, ... (alternating).  Visually: a perfect sawtooth in
 *     the sampled panel.  The continuous panel shows the actual
 *     fast-oscillating cosine.  This is the boundary case.
 *   - At f = fs: the signal completes one cycle PER SAMPLE.  Every
 *     sample is at the same phase → all samples equal.  Sampled
 *     panel is constant.  HUD shows alias = 0.
 *   - At f = 1.5 * fs: alias = fs/2 = N/2.  Maximum visible alias
 *     before folding back down.
 *   - The line-segment reconstruction in the bottom panel is
 *     LINEAR interpolation, not the perfect sinc reconstruction the
 *     Shannon theorem prescribes.  Even so, it's enough to make
 *     the alias visible to the eye.
 *
 * HOW TO VERIFY
 *   - At default freq = 0.5 (well below Nyquist), the sampled wave
 *     in the bottom panel should match the ghost almost exactly.
 *
 *   - Sweep up to f = 8 (a quarter of N): ~4 cycles in the buffer.
 *     Both panels show ~4 cycles.  Sampled tracks ghost.
 *
 *   - At f = N/2 = 16 (Nyquist): the sampled wave is +/-/+/-/+/-...
 *     The ghost shows a fast oscillation.  Beyond this point,
 *     aliasing begins.
 *
 *   - At f = 18: alias = 32 - 18 = 14.  The sampled wave looks like
 *     a freq-14 cosine, NOT freq-18.  The ghost still shows freq-18.
 *
 *   - At f = 24: alias = 32 - 24 = 8.  Sampled wave looks like
 *     freq 8.  HUD prints "true=24, alias=8".  Wagon-wheel made
 *     visible.
 *
 *   - At f = 32 (= fs): alias = 0.  Sampled wave is constant.
 *
 *   - Press 'd' for the fold-back arithmetic overlay — see the
 *     formula evaluated step by step for the current frequency.
 *
 *   - Press 'D' for the numeric frequency table — true, alias,
 *     Nyquist, sample rate all printed.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What is aliasing?  (the picket-fence analogy)
 *   T2   The Nyquist-Shannon sampling theorem
 *   T3   Why fs/2 is the limit (the math derivation)
 *   T4   The fold-back arithmetic — derivation by example
 *   T5   The wagon-wheel effect (real-world manifestation)
 *   T6   How the demo visualizes aliasing (continuous vs sampled)
 *   T7   Edge case: AT Nyquist (f = fs/2)
 *   T8   Edge case: AT sample rate (f = fs, alias to DC)
 *   T9   Above fs — the second fold
 *   T10  Anti-aliasing filters — the cure (BEFORE the ADC)
 *   T11  Practical implications (audio, video, scientific instruments)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS ALIASING?  (THE PICKET-FENCE ANALOGY)
 * ─────────────────────────────────────────────────
 * Aliasing happens whenever you take periodic snapshots of a
 * periodic signal: the snapshots can't tell you the difference
 * between two signals whose frequencies are related by your
 * snapshot rate.
 *
 * Classic everyday example: in old movies (filmed at 24 frames per
 * second) wagon wheels often appear to spin BACKWARD even though
 * everyone knows they're going forward.  The film is sampling the
 * rotating wheel at 24 fps.  When the wheel rotates close to (but
 * just under) 24 revolutions per second, each frame catches it just
 * before completing a full rotation — so frame-to-frame the wheel
 * appears to have rotated backward by a small amount.
 *
 * Picket-fence analogy: stand next to a fence and try to count
 * how many times a bird flaps its wings as it flies past.  If you
 * can blink only once per second and the bird flaps 5 times per
 * second, each blink catches the wings in a different position —
 * but you have no way to know if 1 or 6 flaps happened between
 * blinks.  ANY frequency that completes an integer number of flaps
 * between blinks looks identical to 0 flaps to you.
 *
 * In digital signal processing the "blinks per second" is the
 * SAMPLING RATE fs.  Frequencies above fs/2 alias to lower-looking
 * frequencies.  This file makes the aliasing visible by drawing
 * the true continuous signal alongside the sampled version.
 *
 * T2  THE NYQUIST-SHANNON SAMPLING THEOREM
 * ────────────────────────────────────────
 * Nyquist (1928) and Shannon (1949) proved a precise statement:
 *
 *     A continuous signal x(t) can be PERFECTLY RECONSTRUCTED from
 *     samples taken at rate fs IF AND ONLY IF x(t) contains no
 *     frequencies at or above fs/2.
 *
 * The fs/2 limit is called the NYQUIST FREQUENCY (or Nyquist limit
 * or Nyquist rate, depending on context — usage varies).
 *
 * Practical implications:
 *   - To capture audio with frequencies up to 20 kHz (the upper
 *     limit of human hearing), you need fs >= 40 kHz.  CD-quality
 *     audio uses fs = 44.1 kHz.
 *   - To capture an HD video signal you need to sample at 2x the
 *     highest spatial frequency you want to preserve.  This is why
 *     1080p video looks better than 720p — more samples per row.
 *   - To digitize a 1 GHz radio signal you need fs >= 2 GHz, which
 *     is why high-frequency sampling ADCs are expensive.
 *
 * The "if and only if" is critical.  If your signal contains
 * frequencies above fs/2, sampling DOES NOT just throw them away —
 * it ALIASES them into the [0, fs/2] band, corrupting your data.
 * That's why anti-aliasing filters (T10) are mandatory.
 *
 * T3  WHY fs/2 IS THE LIMIT  (THE MATH DERIVATION)
 * ────────────────────────────────────────────────
 * Consider a pure cosine at frequency f, sampled at rate fs.  The
 * n-th sample is:
 *
 *     x[n] = cos(2*pi * f * n / fs)
 *
 * Now consider another cosine at frequency (f + k*fs) for any
 * integer k:
 *
 *     y[n] = cos(2*pi * (f + k*fs) * n / fs)
 *          = cos(2*pi * f * n / fs + 2*pi * k * n)
 *
 * But cos(x + 2*pi*k*n) = cos(x) for any integer k*n.  So:
 *
 *     y[n] = x[n]   for all n
 *
 * The two signals (at frequencies f and f + k*fs) are SAMPLE-FOR-
 * SAMPLE IDENTICAL.  Sampling at rate fs cannot distinguish them.
 *
 * Similarly for cos(2*pi * (k*fs - f) * n / fs) — flipping the
 * sign of f gives the same samples too (cos is an even function).
 *
 * Combine the two facts:
 *   - f and f + fs sample identically (frequency wrap)
 *   - f and -f sample identically (sign flip)
 *
 * The unique frequencies in [0, fs] reduce to [0, fs/2] by symmetry
 * around fs/2.  Anything above fs/2 has an EQUIVALENT lower-
 * frequency partner that produces the same samples.
 *
 * T4  THE FOLD-BACK ARITHMETIC — DERIVATION BY EXAMPLE
 * ────────────────────────────────────────────────────
 * Given a true frequency f, what frequency does it appear to be
 * after sampling at rate fs?
 *
 * Step 1: take f modulo fs.  This handles the "frequency wrap"
 * symmetry (f and f + fs are identical).
 *
 *     f_mod = f mod fs    →  f_mod is in [0, fs)
 *
 * Step 2: if f_mod is in [0, fs/2], we're below Nyquist and the
 * apparent frequency IS f_mod.  If f_mod is in (fs/2, fs), the
 * "sign flip" symmetry kicks in and the apparent frequency is
 * fs - f_mod (which is in (0, fs/2)).
 *
 *     if f_mod > fs/2:  f_alias = fs - f_mod
 *     else:             f_alias = f_mod
 *
 * Worked examples (fs = 32):
 *
 *     f = 5    → f_mod = 5,    below fs/2 → alias = 5
 *     f = 14   → f_mod = 14,   below fs/2 → alias = 14
 *     f = 16   → f_mod = 16,   AT fs/2    → alias = 16 (edge case)
 *     f = 18   → f_mod = 18,   above fs/2 → alias = 32 - 18 = 14
 *     f = 24   → f_mod = 24,   above fs/2 → alias = 32 - 24 = 8
 *     f = 32   → f_mod = 0,    below fs/2 → alias = 0 (DC!)
 *     f = 40   → f_mod = 8,    below fs/2 → alias = 8
 *     f = 48   → f_mod = 16,   AT fs/2    → alias = 16
 *     f = 56   → f_mod = 24,   above fs/2 → alias = 32 - 24 = 8
 *
 * Notice the SAWTOOTH pattern: alias goes 0..16 (as f goes 0..16),
 * then 16..0 (as f goes 16..32), then 0..16 again (32..48), etc.
 *
 * This is implemented in §4 aliased_frequency() in two lines:
 *
 *     f_mod = fmodf(fabsf(true_freq), sample_rate);
 *     if (f_mod > sample_rate * 0.5f) f_mod = sample_rate - f_mod;
 *
 * T5  THE WAGON-WHEEL EFFECT  (REAL-WORLD MANIFESTATION)
 * ──────────────────────────────────────────────────────
 * Stage-coach films from the early 20th century have a famously
 * weird visual artifact: as a wagon picks up speed, its wheels
 * appear to slow down, stop, and even spin BACKWARD.  No physics
 * was violated.  The aliasing was happening in the camera.
 *
 * Film at 24 fps captures one image of the rotating wheel every
 * 1/24 second.  If the wheel rotates exactly 24 revs/second, each
 * frame catches it at the same angle → wheel APPEARS stationary.
 * If the wheel rotates 23 revs/second, each frame catches it just
 * BEFORE one full rotation → wheel APPEARS to spin backward
 * slowly.  If 25 revs/second, each frame catches it just AFTER one
 * full rotation → wheel appears to spin forward slowly.
 *
 * This is exactly the alias formula at work, applied to the wheel's
 * angular position rather than to a sinusoid.  fs = 24, true rate
 * varies; alias = fold(true_rate) gives the apparent rate.
 *
 * Modern computer animation (e.g. games, VFX) goes to lengths to
 * AVOID the wagon-wheel effect by adding motion blur — which is
 * essentially low-pass filtering before sampling, removing the
 * high-frequency components that would alias.
 *
 * T6  HOW THE DEMO VISUALIZES ALIASING
 * ────────────────────────────────────
 * Two stacked panels:
 *
 *   TOP — CONTINUOUS PANEL.  Cosine evaluated at HIGH_RES_FACTOR =
 *   8 sub-samples per actual sample, giving 256 sample values
 *   plotted across the buffer width.  This is the GROUND TRUTH —
 *   what the signal actually looks like.
 *
 *   BOTTOM — SAMPLED PANEL.  Three layers:
 *     1. Faint grey '.' GHOST of the continuous signal (every other
 *        column to keep density readable).
 *     2. Green Bresenham LINE SEGMENTS connecting the N actual
 *        sample points with linear interpolation.  This is the
 *        "reconstruction" — what you'd see if you reconstructed the
 *        signal from samples using simple linear interpolation.
 *     3. Bright yellow '*' MARKERS at the N sample points.  These
 *        are the actual samples — the only thing the digital system
 *        knows about.
 *
 * When the bright sampled wave matches the grey ghost: you're below
 * Nyquist, no aliasing.  When they diverge: aliasing is happening
 * — the sampled version represents a LOWER frequency than the truth.
 *
 * T7  EDGE CASE: AT NYQUIST  (f = fs/2)
 * ─────────────────────────────────────
 * At exactly the Nyquist limit, the sampled signal alternates
 * between +1 and -1 every sample (for a cosine starting at phase 0).
 * The reconstructed wave looks like a sharp triangle wave or
 * sawtooth, not a smooth cosine.
 *
 * This is the BOUNDARY case.  The Nyquist-Shannon theorem says
 * "below" fs/2 (strictly), not "at or below".  Exactly at fs/2 the
 * theorem doesn't apply — for a cosine, the samples don't even
 * uniquely identify the amplitude (depends on the starting phase).
 *
 * Real-world ADC designers leave a margin: they set their anti-
 * aliasing filter cutoff well below fs/2 (typically 0.45 * fs) to
 * avoid this edge case.
 *
 * Try: in manual mode (press 'a'), set frequency to exactly 16
 * (= N/2).  HUD will show "AT NYQUIST".  Sampled panel shows the
 * triangle/sawtooth pattern.
 *
 * T8  EDGE CASE: AT SAMPLE RATE  (f = fs, ALIAS TO DC)
 * ────────────────────────────────────────────────────
 * At exactly the sample rate, the cosine completes one full cycle
 * between samples.  Every sample catches the cosine at the same
 * point in its cycle → every sample has the same value → the
 * sampled signal is CONSTANT (a "DC" signal).
 *
 * The aliased frequency is 0 (DC).  HUD shows alias = 0.00.  The
 * fast-oscillating ghost shows ~32 cycles in the buffer; the bright
 * sampled wave is a flat line.  This is the most dramatic possible
 * aliasing — a high-frequency signal completely disappearing into
 * a constant.
 *
 * Try: set frequency to 32 (= N).  Watch the bright wave become
 * a flat line while the ghost spins frantically.
 *
 * T9  ABOVE fs — THE SECOND FOLD
 * ──────────────────────────────
 * Past f = fs (= 32 in our case), the alias formula's "f mod fs"
 * step kicks in.  At f = 33, f_mod = 1, alias = 1.  At f = 40,
 * f_mod = 8, alias = 8.  At f = 48 (= 1.5 * fs), f_mod = 16,
 * alias = 16 (back at Nyquist).
 *
 * As f continues climbing past 1.5 * fs, the second fold-back
 * begins.  At f = 56, f_mod = 24, alias = 32 - 24 = 8.  And so on.
 *
 * This is the "infinite zoo" of aliases: every TRUE frequency has
 * infinitely many other frequencies that sample identically.  The
 * sampler can only see the [0, fs/2] image; everything above is
 * folded down into that range, layer upon layer.
 *
 * In our auto-sweep mode the frequency walks from 0.5 to 2 * fs
 * (= 64), so you see the sweep happen TWICE — first time (f =
 * 0..32) the alias goes 0..16..0, second time (f = 32..64) it goes
 * 0..16..0 again.
 *
 * T10  ANTI-ALIASING FILTERS — THE CURE  (BEFORE THE ADC)
 * ───────────────────────────────────────────────────────
 * If aliasing is unavoidable for above-Nyquist content, how does
 * any digital system work in practice?  Answer: KILL the
 * above-Nyquist content BEFORE sampling.
 *
 * An ANTI-ALIASING FILTER is a low-pass filter (typically analogue
 * or in continuous-time signal-processing hardware) placed BEFORE
 * the ADC's sample-and-hold.  It removes everything above fs/2 so
 * there's nothing to alias.
 *
 * In a typical CD-quality audio chain:
 *
 *     mic → analogue low-pass filter at 22 kHz → ADC at 44.1 kHz → bits
 *
 * The 22 kHz filter is the anti-aliasing filter.  Without it,
 * ultrasonic content (above 22 kHz) would fold down into the audible
 * band and corrupt the audio.
 *
 * In hardware-design terms this is hugely important:
 *   - If you sample with a wide-bandwidth ADC and forget the
 *     anti-aliasing filter, you'll get noise from EVERYWHERE in
 *     the spectrum folding into your useful band.  Tracking down
 *     "where did this 1 kHz hum come from?" can be a nightmare —
 *     it might be a 43 kHz switching power supply aliasing down.
 *
 *   - If you OVERSAMPLE (use fs much higher than the highest signal
 *     of interest), you can use a gentler anti-aliasing filter.
 *     This is why audio gear sometimes runs at 192 kHz — not because
 *     anyone can hear it, but because the gentler filter has nicer
 *     passband behaviour.
 *
 * signal/fir_filter.c demonstrates how to design a low-pass filter
 * that could serve as a digital anti-aliasing filter (post-sampling
 * for downsampling, not pre-sampling for the ADC).
 *
 * T11  PRACTICAL IMPLICATIONS  (AUDIO, VIDEO, SCIENTIFIC INSTRUMENTS)
 * ───────────────────────────────────────────────────────────────────
 * Aliasing affects every digital sampling system in existence:
 *
 *   AUDIO.  CD-quality at 44.1 kHz can capture everything below
 *   22.05 kHz.  Anti-aliasing filter handles the rest.  High-resolution
 *   audio (96 kHz, 192 kHz) extends the bandwidth but mostly serves
 *   to make the anti-aliasing filter easier to design.
 *
 *   VIDEO.  Frame rate (24, 30, 60 fps) limits the temporal
 *   bandwidth.  Spatial sampling (pixel grid) limits the spatial
 *   bandwidth.  Both can alias — temporal aliasing is the wagon
 *   wheel effect; spatial aliasing creates jagged edges and Moire
 *   patterns when fine textures are under-sampled.  Anti-aliasing
 *   in graphics rendering (MSAA, FXAA, etc.) addresses the spatial
 *   side; motion blur addresses the temporal side.
 *
 *   SCIENTIFIC INSTRUMENTS.  Oscilloscope, spectrum analyser,
 *   vibration sensor — all sample analogue signals at some rate.
 *   All have anti-aliasing filters baked into the front-end
 *   electronics.  When debugging mysterious low-frequency artifacts,
 *   "is something high-frequency aliasing?" is one of the first
 *   questions.
 *
 *   RADAR / SONAR.  Sample a returning echo at rate fs; ranges
 *   beyond fs * c / 2 (where c is the wave speed) alias to closer
 *   ranges, making targets appear to be where they aren't.
 *
 *   PHOTOGRAPHY.  Pixel grid is the sampling lattice.  Fine textures
 *   that exceed the pixel resolution alias into low-frequency Moire
 *   patterns (hence the "anti-aliasing filter" — really a slight
 *   blur — in front of camera sensors, until very recently).
 *
 *   GENOMICS.  DNA sequencing reads a chemical signal at some rate;
 *   above-Nyquist content (rare bases, modifications) can alias to
 *   look like more common bases, causing read errors.
 *
 * Aliasing is everywhere there are samples.  Understanding it is
 * NOT optional if you build any system that samples a signal.
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

#define N                  32        /* sample rate (samples / buffer)  */
#define HIGH_RES_FACTOR     8        /* sub-samples per actual sample    */
#define HIGH_RES_N          (N * HIGH_RES_FACTOR)
#define RENDER_FPS         30
#define RENDER_TICK_NS     (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 240      /* one full freq sweep ≈ 8 seconds  */
#define FREQ_LO            0.5f
#define FREQ_HI            ((float)N * 2.0f)   /* sweep to 2 * sample rate */

enum {
    PAIR_CONTINUOUS = 1,    /* continuous signal bars                     */
    PAIR_SAMPLE     = 2,    /* sample-point markers                       */
    PAIR_RECON      = 3,    /* reconstructed line segments                */
    PAIR_GHOST      = 4,    /* faint ghost of continuous in bottom panel */
    PAIR_LABEL      = 5,    /* panel labels                               */
    PAIR_HUD        = 6,    /* HUD top status                             */
    PAIR_HINT       = 7,    /* bottom hint                                */
    PAIR_NYQUIST    = 8,    /* "ALIASED" warning text                     */
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
/* §3  colors                                                            */
/* ===================================================================== */

static void colors_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_CONTINUOUS, 51, -1);   /* bright cyan      */
        init_pair(PAIR_SAMPLE,    226, -1);   /* bright yellow    */
        init_pair(PAIR_RECON,     154, -1);   /* yellow-green     */
        init_pair(PAIR_GHOST,     244, -1);   /* mid grey         */
        init_pair(PAIR_LABEL,     244, -1);   /* mid grey         */
        init_pair(PAIR_HUD,       226, -1);   /* bright yellow    */
        init_pair(PAIR_HINT,       51, -1);   /* bright cyan      */
        init_pair(PAIR_NYQUIST,   196, -1);   /* bright red       */
    } else {
        init_pair(PAIR_CONTINUOUS, COLOR_CYAN,    -1);
        init_pair(PAIR_SAMPLE,     COLOR_YELLOW,  -1);
        init_pair(PAIR_RECON,      COLOR_GREEN,   -1);
        init_pair(PAIR_GHOST,      COLOR_WHITE,   -1);
        init_pair(PAIR_LABEL,      COLOR_WHITE,   -1);
        init_pair(PAIR_HUD,        COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,    -1);
        init_pair(PAIR_NYQUIST,    COLOR_RED,     -1);
    }
}

/* ===================================================================== */
/* §4  aliased_freq — THE fold-back arithmetic (pseudocode-style)        */
/* ===================================================================== */
/*
 *  THE FUNCTION YOU ARE ABOUT TO READ
 *  ──────────────────────────────────
 *  aliased_frequency() is THIS file's punchline.  Two lines of math
 *  that capture the entire Nyquist-Shannon sampling theorem in
 *  practical form: given a true frequency f and a sample rate fs,
 *  return the frequency that the sampled signal will APPEAR to have.
 *
 *  See T4 in the GUIDED TUTORIAL for the derivation by example.
 *
 *  WHAT GOES IN, WHAT COMES OUT
 *  ────────────────────────────
 *    INPUT
 *      true_freq      the actual (continuous) signal frequency,
 *                      in cycles per buffer
 *      sample_rate    the sampling rate, in samples per buffer
 *                      (in this demo, sample_rate = N = 32)
 *
 *    OUTPUT
 *      The frequency the sampled signal appears to have.  Always in
 *      the range [0, sample_rate / 2].  Equals true_freq if true_freq
 *      < sample_rate / 2; otherwise folded back per the recipe.
 *
 *  THE MATH WE ARE TRANSLATING TO CODE
 *  ───────────────────────────────────
 *    f_mod = f mod fs                         (handle frequency wrap)
 *    if f_mod > fs/2: f_alias = fs - f_mod    (handle sign-flip symmetry)
 *    else:            f_alias = f_mod
 *
 *    Plain English: take f modulo fs to get its position within the
 *    "fundamental zone" [0, fs).  If that position is in the upper
 *    half (above fs/2), fold it back down by subtracting from fs.
 *
 *  PSEUDOCODE  (this maps line-by-line to the code below)
 *  ──────────
 *    Step 1.  f_mod = |true_freq| mod sample_rate
 *    Step 2.  if f_mod > sample_rate / 2:
 *               return sample_rate - f_mod   (folded back)
 *             else:
 *               return f_mod                 (no folding needed)
 *
 *  WORKED EXAMPLES  (sample_rate = 32):
 *    true_freq = 5    → Step 1: 5      → Step 2: 5 (below fs/2)
 *    true_freq = 14   → Step 1: 14     → Step 2: 14
 *    true_freq = 16   → Step 1: 16     → Step 2: 16 (AT fs/2, edge case)
 *    true_freq = 18   → Step 1: 18     → Step 2: 32 - 18 = 14 (folded)
 *    true_freq = 24   → Step 1: 24     → Step 2: 32 - 24 = 8
 *    true_freq = 32   → Step 1: 0      → Step 2: 0  (full alias to DC)
 *    true_freq = 48   → Step 1: 16     → Step 2: 16 (AT fs/2 again)
 *    true_freq = 56   → Step 1: 24     → Step 2: 32 - 24 = 8
 *
 *  BOUNDARIES & CONTEXT
 *  ────────────────────
 *    - Cost: O(1).  Two operations.  Called once per frame.
 *    - We use fabsf() before fmodf() so negative frequencies (which
 *      arise in some signal-processing contexts) are handled
 *      correctly — the alias for -f is the same as for +f.
 *    - This function does not allocate, does not have side effects,
 *      and is pure mathematical.
 *
 *  WHY IT EXISTS
 *  ────────────
 *    THIS IS THE ENTIRE LESSON OF THE FILE.  The HUD displays the
 *    return value; the bottom panel's apparent frequency matches it;
 *    every other line of code in the file exists to set this up,
 *    call it, or visualise the result.
 */

static float aliased_frequency(float true_freq, float sample_rate)
{
    /* ── STEP 1 — handle frequency wrap (f and f + k*fs alias) ──── */
    float f_mod = fmodf(fabsf(true_freq), sample_rate);

    /* ── STEP 2 — handle sign-flip symmetry (fold upper half down) ─ */
    if (f_mod > sample_rate * 0.5f) f_mod = sample_rate - f_mod;

    return f_mod;
}

/* ===================================================================== */
/* §5  signal_continuous — high-res cosine for the top panel             */
/* ===================================================================== */

static float g_continuous[HIGH_RES_N];

static void generate_continuous_signal(float frequency_cycles_per_buffer)
{
    /*
     * Fill g_continuous[0..HIGH_RES_N - 1] with cos(2*pi * f * t)
     * where t is the position within the buffer normalised to [0, 1].
     * HIGH_RES_FACTOR sub-samples per actual sample give us a smooth
     * curve at terminal resolution.
     *
     * Pseudocode:
     *   for i in 0..HIGH_RES_N - 1:
     *     t = i / HIGH_RES_N
     *     g_continuous[i] = cos(2*pi * f * t)
     */
    for (int i = 0; i < HIGH_RES_N; i++) {
        float t = (float)i / (float)HIGH_RES_N;
        g_continuous[i] = cosf(2.0f * (float)M_PI
                             * frequency_cycles_per_buffer * t);
    }
}

/* ===================================================================== */
/* §6  signal_sampled — N sample points for the bottom panel             */
/* ===================================================================== */

static float g_sampled[N];

static void generate_sampled_signal(float frequency_cycles_per_buffer)
{
    /*
     * Fill g_sampled[0..N-1] with the ACTUAL sample values at sample
     * rate fs = N.  Same formula as continuous but t = n/N, so we
     * get exactly N values across the buffer.
     *
     * Pseudocode:
     *   for n in 0..N-1:
     *     t = n / N
     *     g_sampled[n] = cos(2*pi * f * t)
     *
     * This is what the digital system "sees" — only these N values,
     * nothing in between.  The aliasing happens because these N
     * values can be EXPLAINED by many different continuous signals,
     * and the system has no way to choose between them.
     */
    for (int n = 0; n < N; n++) {
        float t = (float)n / (float)N;
        g_sampled[n] = cosf(2.0f * (float)M_PI
                          * frequency_cycles_per_buffer * t);
    }
}

/* ===================================================================== */
/* §7  scene_state                                                       */
/* ===================================================================== */

static float g_true_frequency_cycles_per_buffer    = FREQ_LO;
static float g_aliased_frequency_cycles_per_buffer = 0.0f;
static bool  g_simulation_paused                   = false;
static bool  g_auto_sweep_enabled                  = true;
static float g_animation_phase_radians             = 0.0f;

/* Debug overlay toggles. */
static bool  g_show_foldback_overlay               = false;   /* 'd' */
static bool  g_show_frequency_table                = false;   /* 'D' */

static void scene_reset(void)
{
    g_true_frequency_cycles_per_buffer = FREQ_LO;
    g_simulation_paused                = false;
    g_auto_sweep_enabled               = true;
    g_animation_phase_radians          = 0.0f;
}

/* ===================================================================== */
/* §8  scene_tick — per-frame update orchestrator                        */
/* ===================================================================== */
/*
 *  Five numbered steps that match the ALGORITHM IN STEPS section
 *  of the MENTAL MODEL block.
 */
static void scene_tick(void)
{
    if (g_simulation_paused) return;

    /* ── Step 1.  auto-sweep frequency ──────────────────────────
     * sin gives smooth there-and-back motion between FREQ_LO and
     * FREQ_HI.  Sweep covers below + above Nyquist + above sample
     * rate so the demo shows multiple folds. */
    if (g_auto_sweep_enabled) {
        g_animation_phase_radians +=
            2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
        if (g_animation_phase_radians > 2.0f * (float)M_PI)
            g_animation_phase_radians -= 2.0f * (float)M_PI;
        float s = (sinf(g_animation_phase_radians) + 1.0f) * 0.5f;
        g_true_frequency_cycles_per_buffer =
            FREQ_LO + s * (FREQ_HI - FREQ_LO);
    }

    /* ── Step 2.  generate "continuous" signal (ground truth) ── */
    generate_continuous_signal(g_true_frequency_cycles_per_buffer);

    /* ── Step 3.  generate sampled signal (the ADC output) ──── */
    generate_sampled_signal(g_true_frequency_cycles_per_buffer);

    /* ── Step 4.  compute the aliased frequency for the HUD ── */
    g_aliased_frequency_cycles_per_buffer =
        aliased_frequency(g_true_frequency_cycles_per_buffer,
                          (float)N);

    /* (Step 5 — render — happens in the main loop after this.) */
}

/* ===================================================================== */
/* §9  scene_input — handle keys                                         */
/* ===================================================================== */

static void scene_adjust_freq(float delta)
{
    /* No-op while auto-sweep is on; otherwise clamp to [FREQ_LO, FREQ_HI]. */
    if (g_auto_sweep_enabled) return;
    g_true_frequency_cycles_per_buffer += delta;
    if (g_true_frequency_cycles_per_buffer < FREQ_LO)
        g_true_frequency_cycles_per_buffer = FREQ_LO;
    if (g_true_frequency_cycles_per_buffer > FREQ_HI)
        g_true_frequency_cycles_per_buffer = FREQ_HI;
}

/* ===================================================================== */
/* §10  draw_continuous — top panel renderer                             */
/* ===================================================================== */

static void draw_continuous_panel(int top_row, int height_rows)
{
    /*
     * Draw the high-res continuous signal as ONE column per cell.
     * For each terminal column c we pick the closest sub-sample
     * and use its value as the bar height.  Positive grows up
     * from the midline, negative grows down.
     */
    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    for (int c = 0; c < COLS; c++) {
        /* Map column c to a sub-sample index.  At HIGH_RES_FACTOR=8
         * and N=32, HIGH_RES_N = 256.  COLS varies; we pick the
         * sub-sample closest to fraction c/COLS of the buffer. */
        int idx = (int)((float)c / (float)COLS * (float)HIGH_RES_N);
        if (idx < 0) idx = 0;
        if (idx >= HIGH_RES_N) idx = HIGH_RES_N - 1;
        float v   = g_continuous[idx];
        bool  pos = (v >= 0.0f);
        int   h   = (int)(fabsf(v) * (float)half_height + 0.5f);

        attron(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
        for (int dy = 0; dy < h; dy++) {
            int row = pos ? (midline_row - dy) : (midline_row + dy + 1);
            if (row < 0 || row >= LINES) continue;
            mvaddch(row, c, pos ? '|' : '.');
        }
        attroff(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
    }
}

/* ===================================================================== */
/* §11  draw_ghost — ghost overlay helper                                */
/* ===================================================================== */

static void draw_ghost_at_column(int col, int midline_row, int half_height)
{
    /* Draw one cell of the faint ghost (continuous signal) at the
     * given column.  Used by the sampled panel as a comparison
     * reference. */
    int idx = (int)((float)col / (float)COLS * (float)HIGH_RES_N);
    if (idx < 0 || idx >= HIGH_RES_N) return;
    float v = g_continuous[idx];
    int   row = midline_row - (int)(v * (float)half_height + 0.5f);
    if (row >= 0 && row < LINES) {
        attron(COLOR_PAIR(PAIR_GHOST));
        mvaddch(row, col, '.');
        attroff(COLOR_PAIR(PAIR_GHOST));
    }
}

/* ===================================================================== */
/* §12  draw_recon — Bresenham segment helper                            */
/* ===================================================================== */

static void draw_recon_segment(int col0, int row0, int col1, int row1)
{
    /* Bresenham line segment between (col0, row0) and (col1, row1).
     * Used to draw the linear-interpolation reconstruction of the
     * sampled signal — the "what you'd reconstruct from samples"
     * version, drawn as connecting line segments. */
    int dx = abs(col1 - col0), dy = abs(row1 - row0);
    int sx = col0 < col1 ? 1 : -1, sy = row0 < row1 ? 1 : -1;
    int err = dx - dy;
    attron(COLOR_PAIR(PAIR_RECON) | A_BOLD);
    for (;;) {
        if (col0 >= 0 && col0 < COLS && row0 >= 0 && row0 < LINES) {
            int   e2 = 2 * err;
            bool  bx = e2 > -dy, by = e2 < dx;
            chtype ch = (bx && by) ? (sx == sy ? '\\' : '/')
                       : bx ? '-' : '|';
            mvaddch(row0, col0, ch);
        }
        if (col0 == col1 && row0 == row1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; col0 += sx; }
        if (e2 <  dx) { err += dx; row0 += sy; }
    }
    attroff(COLOR_PAIR(PAIR_RECON) | A_BOLD);
}

/* ===================================================================== */
/* §13  draw_sampled — bottom panel: samples + lines + ghost              */
/* ===================================================================== */
/*
 *  Three-layer renderer.  Each layer adds context the eye needs:
 *    Layer 1.  Faint grey GHOST (the truth) — every other column.
 *    Layer 2.  Green LINE SEGMENTS connecting samples (linear
 *              reconstruction).
 *    Layer 3.  Bright yellow MARKERS at the sample positions
 *              (the actual data the digital system has).
 */
static void draw_sampled_panel(int top_row, int height_rows)
{
    int half_height = height_rows / 2;
    if (half_height < 1) half_height = 1;
    int midline_row = top_row + half_height;

    /* Layer 1.  Faint continuous ghost (every 2nd column). */
    for (int c = 0; c < COLS; c += 2)
        draw_ghost_at_column(c, midline_row, half_height);

    /* Layer 2.  Linear-interpolation reconstruction line segments
     * between consecutive samples.  Pre-compute all sample
     * positions, then draw N-1 segments. */
    int sample_columns[N];
    int sample_rows[N];
    for (int n = 0; n < N; n++) {
        sample_columns[n] = (int)(((float)n + 0.5f) / (float)N
                                  * (float)COLS);
        if (sample_columns[n] < 0) sample_columns[n] = 0;
        if (sample_columns[n] >= COLS) sample_columns[n] = COLS - 1;
        sample_rows[n] = midline_row
                       - (int)(g_sampled[n] * (float)half_height + 0.5f);
    }
    for (int n = 0; n + 1 < N; n++) {
        draw_recon_segment(sample_columns[n], sample_rows[n],
                           sample_columns[n + 1], sample_rows[n + 1]);
    }

    /* Layer 3.  Bright sample markers on top of the lines. */
    for (int n = 0; n < N; n++) {
        int col = sample_columns[n];
        int row = sample_rows[n];
        if (row >= 0 && row < LINES && col >= 0 && col < COLS) {
            attron(COLOR_PAIR(PAIR_SAMPLE) | A_BOLD);
            mvaddch(row, col, '*');
            attroff(COLOR_PAIR(PAIR_SAMPLE) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §14  draw_debug — 'd' fold-back overlay + 'D' frequency table         */
/* ===================================================================== */
/*
 *  Two non-invasive teaching overlays.  Both READ algorithm state
 *  without modifying it.
 *
 *    'd' FOLD-BACK OVERLAY.  Shows the fold-back arithmetic from §4
 *        evaluated step by step for the current true frequency.  The
 *        formula in motion: see f_mod and f_alias computed live.
 *
 *    'D' FREQUENCY TABLE.  Numeric values for true_f, alias_f,
 *        Nyquist, sample rate, normalised cutoff.  Useful for
 *        precise readings when in manual mode.
 */

static void draw_foldback_overlay(void)
{
    if (!g_show_foldback_overlay) return;
    int x = 2, y = 2;
    if (y + 8 >= LINES - 1) return;

    float true_f = g_true_frequency_cycles_per_buffer;
    float fs     = (float)N;
    float f_mod  = fmodf(fabsf(true_f), fs);
    bool  folded = (f_mod > fs * 0.5f);
    float alias  = folded ? (fs - f_mod) : f_mod;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, "Fold-back arithmetic for true_f = %.2f:", (double)true_f);
    attroff(COLOR_PAIR(PAIR_HINT));

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(y + 1, x, "  Step 1:  f_mod = %.2f mod %.0f = %.2f",
             (double)fabsf(true_f), (double)fs, (double)f_mod);
    if (folded) {
        mvprintw(y + 2, x,
                 "  Step 2:  f_mod (%.2f) > fs/2 (%.0f), so:",
                 (double)f_mod, (double)(fs * 0.5f));
        mvprintw(y + 3, x,
                 "           alias = %.0f - %.2f = %.2f",
                 (double)fs, (double)f_mod, (double)alias);
    } else {
        mvprintw(y + 2, x,
                 "  Step 2:  f_mod (%.2f) <= fs/2 (%.0f), so:",
                 (double)f_mod, (double)(fs * 0.5f));
        mvprintw(y + 3, x,
                 "           alias = f_mod = %.2f",
                 (double)alias);
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(folded ? PAIR_NYQUIST : PAIR_RECON) | A_BOLD);
    mvprintw(y + 5, x, "  → Apparent frequency: %.2f cycles/buffer",
             (double)alias);
    if (folded)
        mvprintw(y + 6, x, "  → ALIASED (true content lost)");
    else
        mvprintw(y + 6, x, "  → Faithful (below Nyquist)");
    attroff(COLOR_PAIR(folded ? PAIR_NYQUIST : PAIR_RECON) | A_BOLD);
}

static void draw_frequency_table(void)
{
    if (!g_show_frequency_table) return;
    /* Push table down if the fold-back overlay is on so they don't
     * collide. */
    int x = 2, y = 2;
    if (g_show_foldback_overlay) y += 8;
    if (y + 6 >= LINES - 1) return;

    float fs           = (float)N;
    float nyquist      = fs * 0.5f;
    float true_f       = g_true_frequency_cycles_per_buffer;
    float alias_f      = g_aliased_frequency_cycles_per_buffer;
    float true_norm    = true_f / fs;     /* cycles per sample */
    float alias_norm   = alias_f / fs;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, "Frequency table:");
    attroff(COLOR_PAIR(PAIR_HINT));

    attron(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
    mvprintw(y + 1, x, "  sample rate fs       %6.2f cyc/buf  (= N)", (double)fs);
    mvprintw(y + 2, x, "  Nyquist limit fs/2   %6.2f cyc/buf", (double)nyquist);
    mvprintw(y + 3, x, "  true frequency       %6.2f cyc/buf  (= %.4f cyc/sample)",
             (double)true_f, (double)true_norm);
    mvprintw(y + 4, x, "  alias frequency      %6.2f cyc/buf  (= %.4f cyc/sample)",
             (double)alias_f, (double)alias_norm);
    attroff(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
}

/* ===================================================================== */
/* §15  hud — status + hint + paused chip + frame composer               */
/* ===================================================================== */

static void draw_hud(void)
{
    char status[200];
    bool above_nyquist = (g_true_frequency_cycles_per_buffer
                          > (float)N * 0.5f + 1e-3f);
    bool at_nyquist    = (fabsf(g_true_frequency_cycles_per_buffer
                              - (float)N * 0.5f) < 0.01f);

    const char *status_label =
        at_nyquist    ? "AT NYQUIST"
        : above_nyquist ? "ALIASED   "
        :                 "BELOW NYQ ";

    snprintf(status, sizeof status,
             " Aliasing  N=%d  fs/2=%.1f  true_f=%5.2f  alias_f=%5.2f  %s  %s  %s ",
             N, (double)((float)N * 0.5f),
             (double)g_true_frequency_cycles_per_buffer,
             (double)g_aliased_frequency_cycles_per_buffer,
             status_label,
             g_auto_sweep_enabled ? "AUTO  " : "MANUAL",
             g_simulation_paused  ? "PAUSED" : "      ");
    int x = COLS - (int)strlen(status);
    if (x < 0) x = 0;
    int pair = above_nyquist ? PAIR_NYQUIST : PAIR_HUD;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(0, x, "%s", status);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void draw_hint(void)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(LINES - 1, 0,
             " q:quit  spc:pause  a:auto/manual  +/-:freq  ,/.:fine "
             " d:foldback  D:table  r:reset ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void)
{
    erase();

    /* Layout: 1 hud + 2 labels + 2 panels + 1 hint = 4 reserved.
     * Split rest evenly into 2 panels. */
    int rows_for_panels = LINES - 4;
    if (rows_for_panels < 8) rows_for_panels = 8;
    int top_h    = rows_for_panels / 2;
    int bottom_h = rows_for_panels - top_h;

    int top_label_row    = 1;
    int top_panel_top    = 2;
    int bottom_label_row = 2 + top_h;
    int bottom_panel_top = 3 + top_h;

    /* Panel labels. */
    attron(COLOR_PAIR(PAIR_LABEL));
    mvprintw(top_label_row, 0,
             "Continuous signal x(t)  (the truth)");
    mvprintw(bottom_label_row, 0,
             "Sampled at fs=%d   '*'=sample point  green=reconstruction  "
             "grey=true ghost", N);
    attroff(COLOR_PAIR(PAIR_LABEL));

    draw_continuous_panel(top_panel_top, top_h);
    draw_sampled_panel(bottom_panel_top, bottom_h);

    /* Debug overlays (drawn over panels but under HUD). */
    draw_foldback_overlay();
    draw_frequency_table();

    /* HUD always last. */
    draw_hud();
    draw_hint();

    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §16  app — signal handlers + main loop + key dispatch                 */
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
        case 'a': case 'A':           g_auto_sweep_enabled = !g_auto_sweep_enabled; break;
        case '+': case '=':           scene_adjust_freq(+0.5f); break;
        case '-':                     scene_adjust_freq(-0.5f); break;
        case '.':                     scene_adjust_freq(+0.1f); break;
        case ',':                     scene_adjust_freq(-0.1f); break;
        case 'd':                     g_show_foldback_overlay = !g_show_foldback_overlay; break;
        case 'D':                     g_show_frequency_table  = !g_show_frequency_table;  break;
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
