/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fourier_shapes.c — animated DFT epicycle reconstruction over a
 *                    catalog of 21 preset parametric shapes, with a
 *                    PARSEVAL'S-THEOREM ENERGY BAR showing how much
 *                    of the signal's total power the active arms have
 *                    captured.  10 colour themes, 3 convergence tiers.
 *
 * DEMO
 *   Pick one of 21 preset shapes (Square, Lissajous variants,
 *   spirograph rosettes, hypocycloids, gears, knots, ...).  The
 *   program samples N = 256 points along the shape, runs an O(N²)
 *   DFT, sorts the output bins by amplitude, and animates the chain
 *   of rotating arms whose tip retraces the shape.  Add or remove
 *   arms with '+' / '-' or let auto-add ramp them up; the trail
 *   converges visibly toward the ghost overlay of the source.
 *
 *   The HEADLINE FEATURE is the energy bar at the top of the screen:
 *   it shows the cumulative fraction of total signal power captured
 *   by the M active arms (Parseval's theorem applied to a sorted,
 *   finite spectrum).  Because we sort by amplitude, this is the
 *   energy-OPTIMAL partial reconstruction at every M.  The bar's
 *   colour signals the convergence tier:
 *       red    < 50%    "we have only the broad strokes"
 *       yellow 50..80%  "shape is recognisable, details missing"
 *       green  >= 80%   "reconstruction is essentially correct"
 *
 *   Each shape carries a TIER label (Fast / Med / Slow) shown in the
 *   HUD alongside its in-tier position (e.g. "[Fast 5/9]").  Cycling
 *   through with 'n' / 'p' is a guided tour through three different
 *   convergence stories: smooth shapes hit 100% in 4 arms; cusped
 *   shapes need 10-15; sharp polygons trickle to 95% only past 50.
 *
 * Study alongside
 *   signal/fourier_draw.c    The interactive sibling — same epicycle
 *                             chain machinery, but the user TRACES a
 *                             path with arrow keys instead of choosing
 *                             a preset shape.
 *   signal/epicycles.c       The deepest pedagogical treatment of the
 *                             DFT + epicycle chain, with 20 different
 *                             parametric shapes and signed frequencies
 *                             explained in detail.
 *   signal/fft_vis.c         The 1-D version: same Fourier maths
 *                             applied to a scalar audio-style signal,
 *                             with windowing and a spectrogram waterfall.
 *   signal/dft_helloworld.c  The minimum-viable DFT intro.  Read it
 *                             first if "what is a DFT?" is the open
 *                             question.
 *
 * Section map
 *   §1   config             constants (N, FPS, scale, theme/shape counts)
 *   §2   clock              monotonic-ns timer + sleep
 *   §3   themes             palette table (10 themes) + apply_theme + init
 *   §4   shape_helpers      gcd_int, sample_poly, hypotrochoid, epitrochoid
 *   §5   shape_samplers     21 parametric shape generators (4 sub-groups)
 *   §6   shape_dispatch     sample_path — the big switch
 *   §7   tier_classify      tier table + shape_tier_position helper
 *   §8   dft                O(N^2) DFT with twiddle recurrence
 *   §9   epicycle_table     Epicycle struct + sort + cumulative energy
 *   §10  trail              tip-path ring buffer
 *   §11  scene_state        per-frame globals + reset/init
 *   §12  scene_chain        compute joint positions from epicycle table
 *   §13  scene_tick         per-frame orchestrator (auto-add + advance phi)
 *   §14  draw_helpers       Bresenham line + cell-aspect orbit ellipse
 *   §15  draw_layers        ghost / orbits / trail / chain / markers
 *   §16  hud_energy         status top-right + energy bar + bottom hint
 *   §17  screen             ncurses init / cleanup / present
 *   §18  app                signal handlers + main loop + key dispatch
 *
 * Keys
 *   q | Q | ESC      quit
 *   space            pause / resume
 *   n / p            next / previous shape  (21 to choose from)
 *   t / T            next / previous theme  (10 palettes)
 *   + / -            grow / shrink chain by one arm
 *   a                toggle auto-add (one new arm every ~0.27 s)
 *   c                toggle orbit-circle overlay
 *   g                toggle ghost overlay (faint trace of source samples)
 *   r                reset to current shape (M = 1, fresh trail)
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/fourier_shapes.c \
 *       -o fourier_shapes -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no Fourier background:
 *     1. CONCEPTS — the elevator pitch.
 *     2. MENTAL MODEL — analogies, diagrams, invariants.  The energy-
 *        bar diagram is the headline picture for THIS file.
 *     3. GUIDED TUTORIAL T1..T12 — twelve mini-lessons centred on
 *        Parseval's theorem and the convergence-tier story.  Each
 *        ends in pseudocode that maps to a function in §4..§13.
 *     4. §1 config — the knobs.
 *     5. §8 dft → §9 epicycle_table → §11 scene_state → §13 scene_tick
 *        — the algorithm pipeline, in execution order.
 *     6. §5 shape_samplers — the 21 parametric formulas.  Browse,
 *        don't read linearly; group preambles tell you what each
 *        family teaches.
 *     7. §15 draw_layers + §16 hud_energy — rendering plumbing.
 *     8. §17..§18 — ncurses + main loop.
 *
 *   Reader who already knows DFT and epicycle chains:
 *     §1, §5 (skim), §9 (cumulative energy table is the unique bit),
 *     §13 scene_tick, §16 hud_energy.  The rest is scaffolding
 *     shared with epicycles.c.
 *
 * Naming convention
 *   File-scope globals carry the `g_` prefix.  Variable names spell
 *   out the math whenever a short name would hide what's happening:
 *
 *     g_epicycle_table[i]            sorted (amp, phase, freq) records
 *     g_total_epicycle_count         always = N_SAMPLES after build
 *     g_active_epicycle_count        arms currently animated (1..N)
 *     g_cumulative_energy_fraction[k] Parseval power captured by first k arms
 *     g_total_signal_energy          sum of |Z[n]|^2 (Parseval invariant)
 *     g_animation_phase_radians      phi — global animation angle
 *     g_pivot_pixel_x / _y           chain origin in pixel space
 *     g_pixel_scale                  normalised → pixel multiplier
 *     g_joint_pixel_x[i]             i-th chain joint in pixel space
 *     g_resampled_source[N]          N complex samples used by DFT (= ghost)
 *
 *   Inside tight loops (`i`, `j`, `k`, `n` for counters; `dx`, `dy`
 *   for pixel deltas) single letters stay short.
 *
 * Background you NEED
 *   - The DFT formula at the level of "X[k] is a weighted sum of
 *     inputs".  See dft_helloworld.c CONCEPTS or any DSP textbook.
 *   - Basic complex arithmetic.  We use a 2-float Cplx struct and
 *     spell out re/im operations directly (no <complex.h>).
 *   - The "epicycle chain" idea: each output bin Z[n] decodes into
 *     one rotating arm.  Detailed in epicycles.c T7-T8.
 *
 * Background you do NOT need
 *   - The Fast Fourier Transform.  We use the textbook O(N^2) DFT
 *     with the twiddle recurrence; N = 256 keeps the per-shape cost
 *     under a millisecond and we only run it ONCE per shape change.
 *   - Continuous calculus.  Everything is finite sums.
 *   - Detailed signed-frequency derivation.  See epicycles.c T8.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * What this file demonstrates, in one sentence
 *   PARSEVAL'S THEOREM, made visceral: take any closed 2-D shape,
 *   decompose it into rotating arms via DFT, sort by amplitude, and
 *   watch the cumulative energy fraction climb to 100% as you add
 *   arms one at a time.  Different shapes climb at different rates;
 *   the rate IS the shape's complexity.
 *
 * Algorithm
 *   ONCE per shape change (n / p / r):
 *     1. Sample the chosen parametric formula at N evenly-spaced
 *        parameter values to get N complex points.
 *     2. Compute the O(N^2) DFT with the twiddle recurrence (one
 *        cos+sin per output bin, then complex multiplies).
 *     3. Decode each Z[n] into (amplitude, phase, signed_frequency).
 *     4. qsort by amplitude DESCENDING — this is what makes the
 *        partial sums ENERGY-OPTIMAL.
 *     5. Pre-compute the cumulative-energy table:
 *          cumulative[k] = (sum_{i <= k} |amp_i|^2) / total_energy
 *        The HUD's energy bar reads cumulative[active_count - 1].
 *
 *   EVERY frame (60 times a second):
 *     6. Maybe auto-add one arm (every AUTO_ADD_FRAMES frames).
 *     7. Advance global animation phase phi by 2*pi / CYCLE_FRAMES.
 *     8. Walk the active arms, accumulating tip position from pivot.
 *     9. Push the final tip into the trail ring buffer.
 *    10. Render layered scene: ghost → orbits → trail → chain →
 *        markers → energy bar → HUD.
 *
 * Math
 *   PARSEVAL'S THEOREM (the unique pedagogy here):
 *     sum_{n=0..N-1} |x[n]|^2  =  (1/N) * sum_{n=0..N-1} |Z[n]|^2
 *   In words: total signal power equals total spectrum power.  After
 *   normalising amp_n = |Z[n]| / N this becomes:
 *     average sample power = sum_n amp_n^2
 *   So sorting epicycles by amp DESCENDING and summing amp^2 gives
 *   the largest possible captured energy at every cutoff M — which is
 *   exactly what the energy bar plots.
 *
 * Performance / boundaries
 *   - DFT cost: O(N^2) ONCE per shape change.  At N = 256 → ~65k
 *     complex multiplies, well under a millisecond.
 *   - Per-frame cost: O(M) chain walk + O(L) trail render.  Holds
 *     30 fps trivially.
 *   - No malloc on the hot path.  All buffers sized at compile time.
 *   - 21 shapes × 10 themes = 210 visual variants, all from one demo.
 *
 * Rendering
 *   Pixel-space simulation, cell-space draw.  The chain math runs in
 *   sub-cell pixel coordinates so arm endpoints don't snap to grid;
 *   conversion happens at one point per draw call (px_cx / px_cy).
 *   Six layered passes per frame: ghost → orbit guides → trail →
 *   arm chain → pivot/tip markers → HUD with energy bar.
 *
 * References
 *   Parseval, M.-A. (1799), "Mémoire sur les séries et sur
 *     l'intégration complète d'une équation aux différences partielles
 *     linéaire du second ordre, à coefficients constants."  The
 *     original paper introducing what we now call Parseval's theorem.
 *   3Blue1Brown, "But what is a Fourier series? From heat flow to
 *     drawing with circles", YouTube — the popular intro that put
 *     epicycles into the mainstream.  Watch first if new to this.
 *   Cooley & Tukey (1965), "An Algorithm for the Machine Calculation
 *     of Complex Fourier Series", Math. Comp. 19, 297-301.
 *   Smith, S. W., "The Scientist and Engineer's Guide to DSP",
 *     Chapter 8 (DFT) and Chapter 12 (FFT).  Free online.
 *   https://en.wikipedia.org/wiki/Parseval%27s_theorem
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   Every closed 2-D shape is a sum of N integer-rate rotations
 *   (the DFT).  Parseval's theorem says the TOTAL ENERGY of the
 *   shape equals the SUM of |amplitude|^2 over those rotations.
 *   Sort the rotations by amplitude descending and you have the
 *   energy-optimal partial reconstruction at every cutoff M.
 *
 *   This file animates that partial reconstruction, with a literal
 *   energy bar that ratchets from 0% to 100% as M grows.  The bar's
 *   speed-of-fill IS the shape's complexity.  Smooth shapes
 *   (Lissajous, hypotrochoid) fill the bar to 100% by M = 4.  Sharp
 *   shapes (Square, Lightning) trickle for 60+ arms before the bar
 *   reaches 95%.  Cycle through the 21 shapes and you are taking
 *   a guided tour through "what kinds of curves are easy or hard
 *   for Fourier reconstruction".
 *
 * HOW TO THINK ABOUT IT
 *   Imagine a sculptor blocking out a marble statue.  The biggest
 *   chisel does the rough cut — gets you to "vaguely human-shaped"
 *   in one stroke.  Each smaller chisel adds finer detail.  The
 *   energy bar is "how much marble has been carved".
 *
 *     A SMOOTH PEBBLE: one big chisel does 100% of the work.  Bar
 *     pops to green at M = 1 or 2.
 *
 *     A FOREARM AND HAND: the rough shape is most of the silhouette;
 *     fingers are fine details.  Bar reaches yellow (~70%) by M = 5
 *     and creeps green (~95%) by M = 30.
 *
 *     A LACE DOILY: every chisel matters.  Bar is still red (< 50%)
 *     at M = 30 and only reaches green at M = 100+.
 *
 *   The Fourier basis (rotations sorted by amplitude) is the
 *   GREEDY-OPTIMAL choice of chisels — at every cutoff M, no other
 *   M-rotation chain captures more energy.  Parseval guarantees this.
 *
 *      ┌────────────────────────────────────────────────────────────┐
 *      │                                                            │
 *      │   ENERGY BAR  (the headline visual):                       │
 *      │                                                            │
 *      │     [================================------]   97.4%       │
 *      │      ▲ green = >= 80%                                       │
 *      │                                                            │
 *      │     [============================------------]   65.2%     │
 *      │      ▲ yellow = 50..80%                                     │
 *      │                                                            │
 *      │     [====================----------------------]   38.1%   │
 *      │      ▲ red = < 50%                                          │
 *      │                                                            │
 *      │     Filled portion = sum(amp^2)/total for first M arms.    │
 *      │                                                            │
 *      └────────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS  (bird's-eye view)
 *
 *   At shape change ('n' / 'p' / 'r'):
 *     1. Sample the shape's parametric formula at N=256 points.
 *     2. Run O(N^2) DFT.
 *     3. Decode each Z[n] into Epicycle{amp, phase, freq_signed}.
 *     4. qsort the table by amp DESCENDING.
 *     5. Pre-compute g_cumulative_energy_fraction[k] for every k.
 *
 *   At every frame:
 *     6. Maybe auto-add one arm.
 *     7. Advance phi.
 *     8. Walk active arms, write joint positions in pixel space.
 *     9. Push tip into trail ring.
 *    10. Render layers + HUD + energy bar.
 *
 * KEY FORMULAS
 *
 *   Sampling the chosen shape:
 *     z[k] = (x(t_k), y(t_k))  for k = 0..N-1, t_k = 2*pi*k/N
 *     (some shapes use a longer period; see hypotrochoid §4)
 *
 *   Forward DFT (computed by §8):
 *     Z[n] = sum_{k=0..N-1}  z[k] * exp(-i * 2*pi * n * k / N)
 *
 *   Per-arm decoding:
 *     amp_n   = |Z[n]| / N
 *     phase_n = arg(Z[n])
 *     freq_n  = (n <= N/2) ? n : n - N    (signed frequency)
 *
 *   Tip position at animation phase phi:
 *     z(phi) = sum_{i=0..M-1}  amp_i * exp(i * (freq_i * phi + phase_i))
 *
 *   Parseval's theorem (the unique formula here):
 *     sum_{n=0..N-1} |x[n]|^2  =  (1/N) * sum_{n=0..N-1} |Z[n]|^2
 *     equivalently:
 *     mean_sample_power  =  sum_n amp_n^2
 *
 *   Cumulative energy fraction (the HUD energy bar):
 *     g_total_signal_energy        = sum_n amp_n^2
 *     g_cumulative_energy_fraction[k] = (sum_{i <= k} amp_i^2)
 *                                     / g_total_signal_energy
 *     The bar reads g_cumulative_energy_fraction[active_count - 1].
 *
 *   Animation step (one degree per frame at default 360-frame cycle):
 *     phi <- phi + 2*pi / CYCLE_FRAMES
 *
 *   Cell-aspect orbit ellipse (rendering):
 *     a_cells = radius_pixels / CELL_W
 *     b_cells = radius_pixels / CELL_H
 *     CELL_H = 2 * CELL_W (terminal cells are twice as tall as wide)
 *
 * EDGE CASES TO WATCH
 *   - The DC bin Z[0] is the sum of samples.  After dividing by N
 *     it's the centroid of the curve.  If the centroid is far from
 *     the origin (e.g. cardioid) Z[0] is large and dominates the
 *     sorted order — its amp^2 alone can be >50% of total energy.
 *   - For real-valued samples (always the case here?  No — z[k] is
 *     COMPLEX because each shape is a 2-D point packed as x + i*y),
 *     the DFT output is NOT conjugate-symmetric.  Both positive and
 *     negative signed frequencies carry independent information.
 *     This is why we draw arms at signed frequencies (T8 in
 *     epicycles.c).
 *   - Auto-add timing: AUTO_ADD_FRAMES = 8 frames at 30 fps = ~0.27 s
 *     per arm.  256 arms total → ~68 seconds for the full ramp.
 *   - Resize triggers a full scene_init which calls scene_reset(0).
 *     You LOSE your current shape selection on resize.
 *   - Energy bar at M = 0 reads 0% (we explicitly clamp to avoid
 *     accessing g_cumulative_energy_fraction[-1]).
 *
 * HOW TO VERIFY
 *   - Default shape is Square (slow tier).  Energy bar at M = 1
 *     should be small (~50%); at M = 10 it grows to maybe ~85%; at
 *     M = 50 it reaches green (~95%).  At M = 256 it's exactly 100%.
 *   - Press 'n' eight times to reach Lissajous 7:3.  Energy bar
 *     should slam to green at M = 4 — the "4 dominant bins" claim
 *     in T8 made physical.
 *   - Press 'g' to flip ghost overlay off.  The ":" lattice
 *     disappears.  The trail and chain remain.
 *   - Press 'c' to flip orbit circles off.  Faint dotted ellipses
 *     disappear.
 *   - Press 't' to cycle themes.  Arm/trail/orbit colours change;
 *     HUD stays bright yellow (theme-invariant).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   What is the headline?  (Parseval = greedy-optimal energy)
 *   T2   Parseval's theorem — total energy is preserved
 *   T3   Sorting by amplitude is the energy-optimal greedy choice
 *   T4   The energy bar — what it shows, what it means
 *   T5   Convergence tiers — the catalog as PEDAGOGY
 *   T6   Polygonal shapes (Square, Arrow, Star, Lightning) and Gibbs
 *   T7   Smooth shapes (Cardioid, Lissajous, Hypotrochoid)
 *   T8   Cusped shapes (Deltoid, Hypocycloid, Rose) and medium decay
 *   T9   Compound curves and DFT linearity
 *   T10  Closed spiral — periodicity violation made visible
 *   T11  Per-frame pipeline (when shape changes vs every frame)
 *   T12  Layered rendering — ghost / orbits / trail / chain / energy bar
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT IS THE HEADLINE?
 * ─────────────────────────
 * Three sibling files in this directory animate the SAME idea:
 * "decompose a 2-D shape into rotating arms via DFT".  What makes
 * this file unique is the ENERGY BAR — a literal progress meter that
 * reads "how complete is the reconstruction, from a power-spectral
 * point of view".
 *
 * The bar is grounded in Parseval's theorem (T2): the sum of
 * |amplitude|^2 over all arms equals the total signal energy.  Sort
 * the arms by amplitude descending (T3) and the partial sum at
 * every cutoff M is the LARGEST POSSIBLE — no other M-arm chain
 * captures more energy.  So the bar's reading is mathematically
 * meaningful: it's "you've accomplished X percent of what's
 * physically possible with M arms".
 *
 * Cycle the 21 shapes ('n' / 'p') and the bar tells different
 * stories.  Some shapes ('Lissajous 7:3') slam the bar to 100% at
 * M = 4.  Others ('Lightning', 'Square') keep the bar in the red
 * zone past M = 30.  The catalog is engineered to span all three
 * tiers — Fast (green by M = 6), Med (~85% by M = 10), Slow (still
 * crawling at M = 30).
 *
 * T2  PARSEVAL'S THEOREM — TOTAL ENERGY IS PRESERVED
 * ──────────────────────────────────────────────────
 * The discrete Fourier transform is unitary up to a scale factor.
 * Concretely:
 *
 *   sum_{n=0..N-1} |x[n]|^2  =  (1/N) * sum_{n=0..N-1} |Z[n]|^2
 *
 * Read it: total power computed in the time domain (left side)
 * equals total power computed in the frequency domain (right side,
 * with a 1/N normaliser).  The DFT is just a CHANGE OF BASIS — it
 * rearranges the energy, doesn't destroy or create any.
 *
 * In our notation we use amp_n = |Z[n]| / N, so:
 *
 *   total_energy = sum_n amp_n^2 = mean_sample_power
 *
 * Pre-computing this once per shape gives us the denominator for
 * the energy bar.  Pre-computing the running sum gives us the
 * numerator at every M.  The ratio is what the bar plots.
 *
 * Why does the energy bar matter?  Because magnitude alone does
 * not tell you "how good is my reconstruction".  Two shapes can
 * both have a "first arm" of amp = 0.5 — but for one that arm
 * captures 90% of total energy (smooth), for the other it captures
 * 30% (sharp).  The bar gives you the absolute scale.
 *
 * T3  SORTING BY AMPLITUDE IS THE ENERGY-OPTIMAL GREEDY CHOICE
 * ────────────────────────────────────────────────────────────
 * Suppose we have N candidate arms, each with energy amp_i^2, and
 * we can pick exactly M of them.  Question: which M arms maximise
 * captured energy?
 *
 * Answer: the M with the largest amp values.  This is trivial to
 * prove — the sum of top-M is by construction the largest M-element
 * subset sum of any selection.  The greedy "pick the biggest first"
 * is provably optimal here.
 *
 * For Fourier reconstruction this becomes: sort arms by amp
 * DESCENDING, then animate the first M.  The chain at every M is
 * the energy-optimal M-arm reconstruction of the shape.  Adding
 * the (M+1)-th arm is guaranteed to capture the next-largest
 * contribution.
 *
 * Note: "energy-optimal" is not the same as "VISUALLY closest".
 * For some shapes a small high-frequency arm matters more for the
 * eye than a larger low-frequency one.  But for the headline metric
 * (total power captured), greedy by amp wins, and that's what
 * Parseval lets us track exactly.
 *
 * T4  THE ENERGY BAR — WHAT IT SHOWS, WHAT IT MEANS
 * ─────────────────────────────────────────────────
 * The bar at the top of the screen is a fixed-width horizontal
 * meter.  Its filled portion shows the captured energy fraction:
 *
 *     g_cumulative_energy_fraction[active_count - 1]
 *
 * which equals
 *
 *     (sum_{i = 0..active_count - 1} amp_i^2) / total_signal_energy
 *
 * Range: [0.0, 1.0].  The bar's COLOUR encodes the value:
 *
 *     >= 80%   green   "essentially complete"
 *     50..80%  yellow  "shape is recognisable"
 *      < 50%   red     "broad strokes only"
 *
 * The bar updates LIVE as you press '+' / '-' or as auto-add
 * inches active_count up.
 *
 * Practical reading:
 *   - If the bar JUMPS to green at M = 1 or 2, the shape is
 *     dominated by one or two pure tones.  Lissajous 3:2 does this.
 *   - If the bar takes 30+ arms to reach green, the shape has
 *     significant high-frequency content (sharp corners, cusps,
 *     discontinuities).  Square, Lightning, Gear are like this.
 *
 * The bar is the cleanest answer to "have I added enough arms?".
 *
 * T5  CONVERGENCE TIERS — THE CATALOG AS PEDAGOGY
 * ───────────────────────────────────────────────
 * The 21 shapes are deliberately partitioned into three tiers:
 *
 *   FAST  (9 shapes, energy bar fills to ~100% in <= 6 arms):
 *     Cardioid, Lissajous 3:2, 5:4, 7:3, 11:7, Hypotrochoid 5:3,
 *     Hypotrochoid 7:4, Epitrochoid 3:1, Compound spiro.
 *
 *   MED   (7 shapes, ~85% by M = 10, slow tail of cusps):
 *     Star-7, Deltoid, Hypocycloid 5, Crescent, Rose r=cos(2t),
 *     Beaded circle, Torus knot 2:5.
 *
 *   SLOW  (5 shapes, energy keeps trickling for many arms):
 *     Square, Arrow, Gear 8-tooth, Lightning, Closed spiral.
 *
 * The HUD tier chip — e.g. "[Fast 5/9]" — tells you which tier the
 * current shape belongs to and where you are within that tier.
 * Cycling 'n' walks through the catalog in numbered order; cycling
 * with the tier chip in mind lets you sample one shape from each
 * tier and compare bar speeds directly.
 *
 * Why the tiers?  Because Fourier convergence rate is a real
 * mathematical property of a curve — smooth-everywhere curves have
 * EXPONENTIALLY decaying coefficients, curves with cusps decay only
 * POLYNOMIALLY, curves with discontinuities (Gibbs phenomenon)
 * decay even slower with persistent ringing.  The tiers materialise
 * this hierarchy as something you can SEE.
 *
 * T6  POLYGONAL SHAPES AND GIBBS
 * ──────────────────────────────
 * Square, Arrow, Star-7, Lightning are all defined as polygons —
 * sequences of vertices connected by straight chords.  These have
 * SHARP CORNERS where the curve's direction changes discontinuously.
 *
 * Sharp corners are the worst case for Fourier reconstruction.  A
 * partial Fourier sum near a corner OVERSHOOTS by ~9% of the jump
 * height (the universal Gibbs constant), regardless of how many
 * terms you add — the overshoot does not vanish in the limit.
 * Visible as little bumps just outside each corner of the trail.
 *
 * In energy terms: corner energy is spread across many high-
 * frequency bins.  Each bin's amp is small but there are many of
 * them.  The energy bar climbs slowly as you add arms.  At M = 30
 * you might be at 85%; the remaining 15% lives in the long tail
 * of small bins.
 *
 * Try:
 *   - Press 'n' to Square.  At M = 5 the trail looks like a
 *     rounded blob; the energy bar reads ~85%.  At M = 30 the
 *     corners are visible but with Gibbs ringing; bar reads ~92%.
 *     At M = 100+ the corners are crisp; bar reads ~98%.
 *   - 'n' to Lightning.  Even slower convergence — the 7-vertex
 *     zigzag has many sharp corners.  Bar at M = 30 might still
 *     be in the yellow zone.
 *
 * T7  SMOOTH SHAPES AND RAPID CONVERGENCE
 * ───────────────────────────────────────
 * Cardioid, Lissajous variants, hypotrochoid, epitrochoid are all
 * SMOOTH everywhere — infinitely differentiable, no corners.  Their
 * Fourier coefficients decay exponentially.  The energy bar slams
 * to green almost immediately.
 *
 * Special cases worth knowing:
 *
 *   LISSAJOUS curves are parametric pure tones:
 *     x(t) = sin(a*t + phase)
 *     y(t) = sin(b*t)
 *   With INTEGER a and b the curve closes after one parameter
 *   period.  The DFT spectrum has just FOUR non-zero bins — at
 *   signed frequencies ±a and ±b.  Energy bar goes from 0 to 100%
 *   at M = 4, full stop.  Try Lissajous 11:7 — visually intricate,
 *   spectrally trivial.
 *
 *   HYPOTROCHOIDS / EPITROCHOIDS are spirograph rosettes:
 *     x(t) = (R±r) cos(t) ∓ d cos((R±r)/r * t)
 *     y(t) = (R±r) sin(t) -  d sin((R±r)/r * t)
 *   These are SUMS of two circular motions.  Spectrum has just TWO
 *   non-zero rotation rates (the two integer ratios).  Energy bar
 *   reaches 100% at M = 4 (positive and negative pairs of each).
 *   The intricate visual rosette is a sum of two simple rotations.
 *
 * T8  CUSPED SHAPES AND MEDIUM DECAY
 * ──────────────────────────────────
 * Deltoid, Hypocycloid 5, Rose r=cos(2t), Crescent are SMOOTH
 * almost everywhere but have CUSPS — points where the curve
 * touches itself or comes to a sharp point with zero "smoothness"
 * in the local geometry.
 *
 * Cusps are intermediate between corners (worst case) and smooth
 * curves (best case).  Fourier coefficients decay polynomially,
 * typically as 1/n^2.  Energy reaches ~85% in 10 arms but the last
 * 10-15% comes slowly.
 *
 * Special: SHAPES WITH N-FOLD ROTATIONAL SYMMETRY have spectra that
 * are non-zero only at multiples of N.  Deltoid (3-fold symmetry)
 * has dominant energy at signed bins ±2 and ±4.  Hypocycloid 5
 * (5-fold) has dominant bins at ±4 and ±6.  This is why the energy
 * bar climbs in BIG STEPS instead of smoothly — every time you
 * add a "useful" bin, the bar jumps up; in between are bins with
 * essentially zero amp and the bar barely moves.
 *
 * T9  COMPOUND CURVES AND DFT LINEARITY
 * ─────────────────────────────────────
 * Compound spiro is the SUM of two hypotrochoids that share the
 * same closing period.  Each individually is a 2-bin spectrum;
 * their sum is a 4-bin spectrum.
 *
 *   DFT(x_A + x_B) = DFT(x_A) + DFT(x_B)
 *
 * This is "DFT is a linear operator" — adding two curves in the
 * time domain is the same as adding their spectra.  The energy bar
 * for compound spiro reaches 100% at M = 4 (the union of the two
 * hypotrochoids' bin sets).
 *
 * Beaded circle is similar: a circle (2 bins) plus a small high-
 * frequency wobble (2 more bins).  Total: 4 bins.  Energy bar
 * jumps to 100% at M = 4.
 *
 * Torus knot 2:5 is a non-trivial CLOSED knot in 3-D, projected to
 * 2-D.  Its parametric form r(t)*(cos(qt), sin(qt)) where r(t) =
 * p + cos(qt) blends a p-revolution carrier with q-fold modulation.
 * Spectrum: just three signed bins (±p, ±(q-p), ±(q+p)).
 *
 * T10 CLOSED SPIRAL — PERIODICITY VIOLATION
 * ─────────────────────────────────────────
 * The Archimedean spiral r(t) = a*t is NOT a closed curve.  We
 * close it artificially by appending a straight chord from the
 * outer end back to the origin.  The result is "smooth + a sharp
 * corner where the closing chord meets the spiral end".
 *
 * This violates the DFT's implicit periodicity assumption (the N
 * samples should look like one period of an infinitely-repeating
 * signal, but the closing chord is a discontinuity).  Energy bar
 * climbs slowly — it's a smooth-looking shape that fools the eye
 * into expecting fast convergence, but the closing chord behaves
 * like a sharp corner spectrally.
 *
 * This is the "always close your curves smoothly" lesson made
 * concrete.  Compare to Cardioid (genuinely smooth, periodic; bar
 * jumps to 100% at M = 5) — same visual quietness, opposite
 * convergence story.
 *
 * T11 PER-FRAME PIPELINE (WHEN SHAPE CHANGES VS EVERY FRAME)
 * ──────────────────────────────────────────────────────────
 * The DFT and energy table are computed ONCE per shape change, not
 * per frame.  This separation is critical for performance — at
 * N = 256 the DFT alone is ~65,000 complex multiplies, which would
 * eat the frame budget if done every render cycle.
 *
 * SHAPE-CHANGE PATH (n / p / r):
 *   1. sample_path(shape_idx, ghost) — fill 256 complex samples
 *   2. compute_dft(ghost, dft) — O(N^2)
 *   3. Decode Z[n] -> Epicycle{amp, phase, freq} for n = 0..N-1
 *   4. qsort by amp descending
 *   5. Compute g_cumulative_energy_fraction[k] for k = 0..N-1
 *   6. Reset trail, set active_count = 1, phi = 0
 *
 * PER-FRAME PATH (every render cycle):
 *   1. Maybe auto-add (every AUTO_ADD_FRAMES frames)
 *   2. phi += 2*pi / CYCLE_FRAMES
 *   3. Walk active arms, compute joint positions
 *   4. Push tip into trail ring
 *   5. Render ghost / orbits / trail / chain / markers / energy bar
 *
 * The shape change feels instant because the DFT runs in well
 * under a millisecond at N = 256.  At larger N (say N = 4096) it
 * would feel laggy — and that's where the FFT (signal/fft_vis.c)
 * becomes essential.
 *
 * T12 LAYERED RENDERING
 * ─────────────────────
 * Each frame paints six layers in back-to-front order.  Each layer
 * overwrites the previous in any cells they share — last writer
 * wins per cell, no blending:
 *
 *   1. GHOST overlay      faint ':' at projected source samples
 *   2. ORBIT circles      faint '.' at orbit guide ellipses
 *   3. TRAIL              fading '*' from oldest to newest tip path
 *   4. ARM CHAIN          Bresenham '-', '|', '/', '\' along arms
 *   5. MARKERS            '@' at tip, '+' at pivot
 *   6. HUD + ENERGY BAR   always last; bright yellow + bold
 *
 * Layer 1 (ghost) sits at the BACK because it's the static "target"
 * — every other layer is allowed to obscure it.  The energy bar is
 * the LAST element rendered, on the fixed top-right HUD line, never
 * obscured by anything.  This ordering means the most informative
 * content is always on top.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config — every constant + enum lives here                         */
/* ===================================================================== */

#define N_SAMPLES         256     /* DFT input size; also max epicycles    */
#define TRAIL_LEN         512     /* tip-trail ring buffer                  */
#define RENDER_FPS         30
#define RENDER_NS         (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES      360     /* frames per full reconstruction cycle   */
#define AUTO_ADD_FRAMES     8     /* frames between auto-adding one arm     */
#define N_CIRCLES           5     /* max orbit-guide rings drawn            */
#define SHAPE_SCALE        0.38f  /* fraction of min(pw, ph)/2 used as size */
#define CELL_W              8     /* sub-pixels per terminal column         */
#define CELL_H             16     /* sub-pixels per terminal row            */

/* Colour pair IDs.  HUD/HINT/ENERGY tiers are theme-INVARIANT per
 * CLAUDE.md HUD Standard, so the status bar and Parseval bar are
 * always legible regardless of the active animation theme. */
enum {
    /* themable animation pairs (10) */
    CP_ARM_HI     =  1,   /* large-amplitude arm                          */
    CP_ARM_MID    =  2,   /* medium arm                                    */
    CP_ARM_LO     =  3,   /* tiny arm                                      */
    CP_CIRCLE     =  4,   /* orbit ring dots                               */
    CP_TRAIL_1    =  5,   /* trail newest                                  */
    CP_TRAIL_2    =  6,   /* trail mid                                     */
    CP_TRAIL_3    =  7,   /* trail oldest                                  */
    CP_BOB        =  8,   /* tip dot                                       */
    CP_PIVOT      =  9,   /* pivot marker                                  */
    CP_GHOST      = 10,   /* ghost source-sample dots                      */
    /* theme-invariant pairs (5) */
    CP_HUD        = 11,   /* HUD top status — bright yellow + bold         */
    CP_HINT       = 12,   /* bottom hint    — bright cyan   + bold         */
    CP_ENERGY     = 13,   /* energy bar tier — green (>= 80%)               */
    CP_ENERGY_MID = 14,   /* energy bar tier — yellow (50..80%)             */
    CP_ENERGY_LO  = 15,   /* energy bar tier — red    (< 50%)               */
};

/* ===================================================================== */
/* §2  clock — monotonic ns timer + sleep                                */
/* ===================================================================== */

static long long clock_ns(void)
{
    /* Purpose : monotonic clock in ns.  Used by the main loop to
     *           figure out how long to sleep before the next frame. */
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    /* Block for `ns` nanoseconds.  Without this, the main loop would
     * spin a CPU at 100% just to redraw 30 times a second. */
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  themes — palette table (10 themes) + apply_theme + init           */
/* ===================================================================== */
/*
 * Ten palettes, ten themable animation pairs each.  HUD/HINT/ENERGY
 * pairs are theme-invariant and bound separately in color_init.
 *
 * All entries chosen from the BRIGHT half of the 256-colour cube
 * (>= 24 in the cube, >= 240 in the greyscale ramp).  Codes 16-23
 * in the cube and 232-239 in greyscale are visually indistinguishable
 * from black with A_DIM and would disappear.
 *
 * Each theme also carries 8-colour fallbacks for terminals without
 * extended palette support.
 */

typedef struct {
    const char *name;
    /* 256-colour foregrounds */
    short arm_hi, arm_mid, arm_lo;
    short circle;
    short trail_1, trail_2, trail_3;
    short bob, pivot;
    short ghost;
    /* 8-colour fallbacks */
    short arm_hi8, arm_mid8, arm_lo8;
    short circle8;
    short trail_1_8, trail_2_8, trail_3_8;
    short bob8, pivot8;
    short ghost8;
} Theme;

#define N_THEMES 10

static const Theme k_themes[N_THEMES] = {
    /*  0  Classic   — bright white arms, gold/orange/red trail */
    { "Classic   ",
      231,  87, 246,  243,  226, 208, 196,  231, 220,  240,
      COLOR_WHITE, COLOR_CYAN,  COLOR_WHITE,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE },
    /*  1  Fire      — red/orange spectrum, peach guides */
    { "Fire      ",
      208, 202, 130,  215,  226, 214, 196,  231, 208,  240,
      COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_RED, COLOR_WHITE },
    /*  2  Neon      — magenta/violet/pink, lavender guides */
    { "Neon      ",
      207, 165, 105,  141,  219, 213, 201,  231, 207,  240,
      COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_WHITE },
    /*  3  Ocean     — teal/blue spectrum */
    { "Ocean     ",
      123,  45,  31,  110,  159,  87,  51,  231,  45,  240,
      COLOR_CYAN, COLOR_BLUE, COLOR_BLUE,
      COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
      COLOR_WHITE, COLOR_CYAN, COLOR_WHITE },
    /*  4  Matrix    — green spectrum, classic terminal look */
    { "Matrix    ",
      118,  46,  28,   64,  154, 118,  46,  231, 118,  240,
      COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_WHITE },
    /*  5  Sunset    — peach/pink/magenta gradient */
    { "Sunset    ",
      215, 209, 174,  217,  226, 213, 198,  231, 220,  240,
      COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_MAGENTA, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE },
    /*  6  Nova      — cosmic stellar: bright white core, orange explosion */
    { "Nova      ",
      231, 220, 175,  145,  226, 208, 197,  231, 175,  240,
      COLOR_WHITE, COLOR_YELLOW, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_YELLOW, COLOR_RED, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_WHITE },
    /*  7  Mono      — high-contrast greyscale */
    { "Mono      ",
      231, 252, 245,  240,  255, 250, 244,  231, 246,  240,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE },
    /*  8  Forest    — green/olive/brown earthy palette */
    { "Forest    ",
      154,  70,  28,  100,  226, 178, 130,  231, 178,  240,
      COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_WHITE },
    /*  9  Cyberpunk — electric cyan + hot pink high-contrast */
    { "Cyberpunk ",
       51, 207, 165,  141,  226, 207, 197,  231, 207,  240,
      COLOR_CYAN, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE, COLOR_YELLOW, COLOR_MAGENTA, COLOR_RED,
      COLOR_WHITE, COLOR_MAGENTA, COLOR_MAGENTA },
};

static int g_active_theme_index = 0;

/*
 * apply_theme — rebind the 10 themable animation pairs to a theme's
 * palette.  HUD / HINT / ENERGY pairs are NOT touched here so the
 * status bar and Parseval bar stay legible across all themes.
 */
static void apply_theme(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &k_themes[idx];
    if (COLORS >= 256) {
        init_pair(CP_ARM_HI,  t->arm_hi,  -1);
        init_pair(CP_ARM_MID, t->arm_mid, -1);
        init_pair(CP_ARM_LO,  t->arm_lo,  -1);
        init_pair(CP_CIRCLE,  t->circle,  -1);
        init_pair(CP_TRAIL_1, t->trail_1, -1);
        init_pair(CP_TRAIL_2, t->trail_2, -1);
        init_pair(CP_TRAIL_3, t->trail_3, -1);
        init_pair(CP_BOB,     t->bob,     -1);
        init_pair(CP_PIVOT,   t->pivot,   -1);
        init_pair(CP_GHOST,   t->ghost,   -1);
    } else {
        init_pair(CP_ARM_HI,  t->arm_hi8,   -1);
        init_pair(CP_ARM_MID, t->arm_mid8,  -1);
        init_pair(CP_ARM_LO,  t->arm_lo8,   -1);
        init_pair(CP_CIRCLE,  t->circle8,   -1);
        init_pair(CP_TRAIL_1, t->trail_1_8, -1);
        init_pair(CP_TRAIL_2, t->trail_2_8, -1);
        init_pair(CP_TRAIL_3, t->trail_3_8, -1);
        init_pair(CP_BOB,     t->bob8,      -1);
        init_pair(CP_PIVOT,   t->pivot8,    -1);
        init_pair(CP_GHOST,   t->ghost8,    -1);
    }
}

/*
 * color_init — bind theme-invariant pairs once at startup, then
 * apply default theme.  Subsequent theme changes go through
 * apply_theme and leave HUD/HINT/ENERGY tiers alone.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_HUD,        226, -1);   /* bright yellow */
        init_pair(CP_HINT,        51, -1);   /* bright cyan   */
        init_pair(CP_ENERGY,      46, -1);   /* green  (>= 80%) */
        init_pair(CP_ENERGY_MID, 226, -1);   /* yellow (50..80%) */
        init_pair(CP_ENERGY_LO,  196, -1);   /* red    (< 50%) */
    } else {
        init_pair(CP_HUD,        COLOR_YELLOW, -1);
        init_pair(CP_HINT,       COLOR_CYAN,   -1);
        init_pair(CP_ENERGY,     COLOR_GREEN,  -1);
        init_pair(CP_ENERGY_MID, COLOR_YELLOW, -1);
        init_pair(CP_ENERGY_LO,  COLOR_RED,    -1);
    }
    apply_theme(0);
}

/* ===================================================================== */
/* §4  shape_helpers — gcd, polygon, hypotrochoid, epitrochoid           */
/* ===================================================================== */

typedef struct { float re, im; } Cplx;

/*
 * gcd_int — Euclid's algorithm.  Used by hypotrochoid / epitrochoid
 * to compute their closing period: 2*pi * r / gcd(R, r).
 */
static int gcd_int(int a, int b)
{
    while (b) { int t = b; b = a % b; a = t; }
    return a;
}

/*
 * sample_poly — linearly interpolate around a polygon with `nv`
 * vertices, producing N evenly-spaced complex points along the
 * boundary.
 *
 * Pseudocode:
 *   for i in 0..N-1:
 *     t        = i / N * nv
 *     seg      = floor(t) mod nv
 *     fraction = t - floor(t)
 *     out[i]   = lerp(v[seg], v[seg+1 mod nv], fraction)
 *
 * Sharp vertices in the polygon will produce Gibbs ringing in the
 * Fourier reconstruction — see T6.  Used by Square, Arrow, Star-7,
 * Lightning.
 */
static void sample_poly(const float *vx, const float *vy, int nv,
                        Cplx *out, int N)
{
    for (int i = 0; i < N; i++) {
        float t        = (float)i / (float)N * (float)nv;
        int   seg      = (int)t % nv;
        float fraction = t - floorf(t);
        int   nxt      = (seg + 1) % nv;
        out[i].re = vx[seg] + (vx[nxt] - vx[seg]) * fraction;
        out[i].im = vy[seg] + (vy[nxt] - vy[seg]) * fraction;
    }
}

/*
 * sample_hypotrochoid(R, r, d) — tracer point on a small circle of
 * radius `r` rolling INSIDE a fixed circle of radius `R`, with the
 * pen at distance `d` from the rolling-circle centre.  The classic
 * spirograph parametric formula.
 *
 *   x(t) = (R - r) cos(t) + d cos((R - r)/r * t)
 *   y(t) = (R - r) sin(t) - d sin((R - r)/r * t)
 *
 * Closing period: 2*pi * r / gcd(R, r).  We sample N points over
 * the full closed loop; output normalised to fit the unit disc.
 *
 * Spectrum: dominated by just two signed bins (the two integer
 * rotation rates), regardless of how intricate the on-screen
 * rosette looks.  Energy bar fills to ~100% by M = 4.
 */
static void sample_hypotrochoid(int R, int r, float d, Cplx *out, int N)
{
    int   reps   = r / gcd_int(R, r);
    float period = 2.f * (float)M_PI * (float)reps;
    float diff   = (float)(R - r);
    float scale  = ((float)(R - r) + d);
    if (scale <= 0.001f) scale = 1.f;
    for (int k = 0; k < N; k++) {
        float t   = period * (float)k / (float)N;
        out[k].re = (diff * cosf(t) + d * cosf(diff / (float)r * t)) / scale;
        out[k].im = (diff * sinf(t) - d * sinf(diff / (float)r * t)) / scale;
    }
}

/*
 * sample_epitrochoid(R, r, d) — same family but rolling on the
 * OUTSIDE of the fixed circle.
 *
 *   x(t) = (R + r) cos(t) - d cos((R + r)/r * t)
 *   y(t) = (R + r) sin(t) - d sin((R + r)/r * t)
 *
 * Same 2-bin spectral fingerprint as hypotrochoid — yet visually
 * distinct (limaçon-like loops instead of inner rosettes).
 */
static void sample_epitrochoid(int R, int r, float d, Cplx *out, int N)
{
    int   reps   = r / gcd_int(R, r);
    float period = 2.f * (float)M_PI * (float)reps;
    float sum    = (float)(R + r);
    float scale  = sum + d;
    if (scale <= 0.001f) scale = 1.f;
    for (int k = 0; k < N; k++) {
        float t   = period * (float)k / (float)N;
        out[k].re = (sum * cosf(t) - d * cosf(sum / (float)r * t)) / scale;
        out[k].im = (sum * sinf(t) - d * sinf(sum / (float)r * t)) / scale;
    }
}

/* ===================================================================== */
/* §5  shape_samplers — 21 parametric shape generators                   */
/* ===================================================================== */
/*
 *  The 21 shapes live inside a single big switch in §6 sample_path.
 *  This section is just the dispatcher's body, organised by family
 *  so a learner can read them in convergence-tier order.
 *
 *  GROUPS (and the lesson each teaches):
 *    POLYGONAL   — Square, Arrow, Star-7, Lightning.  Sharp corners
 *                  → Gibbs ringing → slow energy convergence.
 *    SMOOTH      — Cardioid, Lissajous variants, Hypotrochoid,
 *                  Epitrochoid.  Smooth-everywhere → exponential
 *                  decay → energy bar slams to green at M = 4-6.
 *    CUSPED      — Deltoid, Hypocycloid 5, Rose, Crescent.  Smooth
 *                  almost everywhere with isolated cusps →
 *                  polynomial decay.  N-fold symmetry → spectral
 *                  combs.
 *    COMPOUND    — Compound spiro, Beaded circle, Torus knot 2:5.
 *                  Sums of simpler curves → DFT linearity → 4-bin
 *                  spectra.
 *    SPECIAL     — Closed spiral.  Periodicity violation made
 *                  visible.
 *    GEAR        — Gear 8-tooth.  Radial AM → many sidebands →
 *                  slow convergence.
 */

/* ===================================================================== */
/* §6  shape_dispatch — sample_path                                      */
/* ===================================================================== */
/*
 *  Purpose         : fill out[0..N-1] with the chosen shape's complex
 *                    samples.  Called ONCE per shape change by §9
 *                    build_epicycles.
 *  Mental model    : a directory of 21 parametric formulas.  Each case
 *                    body is the mathematical definition of one shape,
 *                    transcribed to C.
 *  Why it exists   : isolating the shape catalog from the DFT keeps
 *                    the algorithm reusable — adding a 22nd shape is
 *                    one new case, zero changes elsewhere.
 */

static const char *k_shape_names[] = {
    "Square          ",  "Arrow           ",  "Star-7          ",
    "Cardioid        ",  "Lissajous 3:2   ",  "Rose r=cos(2t)  ",
    "Lissajous 5:4   ",  "Lissajous 7:3   ",  "Hypotrochoid 5:3",
    "Hypotrochoid 7:4",  "Epitrochoid 3:1 ",  "Deltoid (3-cusp)",
    "Hypocycloid 5   ",  "Gear 8-tooth    ",  "Crescent        ",
    "Lightning       ",
    /* +5 patterns each carrying a distinct NEW lesson */
    "Lissajous 11:7  ",  "Beaded circle   ",  "Compound spiro  ",
    "Closed spiral   ",  "Torus knot 2:5  ",
};
#define N_SHAPES 21

static void sample_path(int si, Cplx *out, int N)
{
    switch (si) {

    case 0: {
        /* SQUARE — 4 sharp corners.  Polygon family.
         * Spectrum: Gibbs ringing.  Energy bar climbs slowly. */
        static const float sx[] = {-1.f,  1.f,  1.f, -1.f};
        static const float sy[] = {-1.f, -1.f,  1.f,  1.f};
        sample_poly(sx, sy, 4, out, N);
        break;
    }

    case 1: {
        /* ARROW — 7-vertex polygon (notched tail adds re-entrant
         * corners).  More corners than Square → slower convergence. */
        static const float ax[] = { 0.f,  .65f,  .30f,  .30f, -.30f, -.30f, -.65f};
        static const float ay[] = { 1.f,  .10f,  .10f, -.85f, -.85f,  .10f,  .10f};
        sample_poly(ax, ay, 7, out, N);
        break;
    }

    case 2: {
        /* 7-POINTED STAR — alternating outer (r=1) and inner (r=0.40)
         * radii at 14 vertices.  Spiky → many corners → slow convergence. */
        float vx[14], vy[14];
        for (int i = 0; i < 14; i++) {
            float r = (i % 2 == 0) ? 1.f : 0.40f;
            float a = -(float)M_PI/2.f + (float)i * (float)M_PI / 7.f;
            vx[i] = r * cosf(a);
            vy[i] = r * sinf(a);
        }
        sample_poly(vx, vy, 14, out, N);
        break;
    }

    case 3: {
        /* CARDIOID — r = 1 - cos(t) recentered.  Smooth.
         * Centroid is offset; we subtract it numerically.
         * Energy bar to 100% at M ≈ 5. */
        float cx = 0.f;
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            float r   = 0.5f * (1.f - cosf(t));
            out[k].re = r * cosf(t);
            out[k].im = r * sinf(t);
            cx       += out[k].re;
        }
        cx /= (float)N;
        for (int k = 0; k < N; k++) out[k].re -= cx;
        break;
    }

    case 4: {
        /* LISSAJOUS 3:2 with pi/4 phase offset.  Pure 4-bin spectrum
         * at signed frequencies ±3 and ±2.  Energy → 100% at M = 4. */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = sinf(3.f*t + (float)M_PI/4.f);
            out[k].im = sinf(2.f*t);
        }
        break;
    }

    case 5: {
        /* 4-PETAL ROSE — r = cos(2t).  r goes negative (back-projection);
         * curve traces all 4 petals over [0, 2*pi].  Cusps at origin. */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            float r   = cosf(2.f*t);
            out[k].re = r * cosf(t);
            out[k].im = r * sinf(t);
        }
        break;
    }

    case 6: {
        /* LISSAJOUS 5:4 — denser weaving than 3:2.  Spectrum: 4
         * dominant bins (±5, ±4) → energy 100% at M = 4. */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = sinf(5.f*t + (float)M_PI/3.f);
            out[k].im = sinf(4.f*t);
        }
        break;
    }

    case 7: {
        /* LISSAJOUS 7:3 — wider ratio, "flower-like" intersections.
         * Spectrum: 4 dominant bins (±7, ±3) → energy 100% at M = 4. */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = sinf(7.f*t);
            out[k].im = sinf(3.f*t);
        }
        break;
    }

    case 8: {
        /* HYPOTROCHOID (R=5, r=3, d=5) — spirograph rosette.
         * 2-bin spectrum (the two integer rotation rates). */
        sample_hypotrochoid(5, 3, 5.f, out, N);
        break;
    }

    case 9: {
        /* HYPOTROCHOID (R=7, r=4, d=6) — different gear ratio,
         * more intricate but still 2-bin spectrum. */
        sample_hypotrochoid(7, 4, 6.f, out, N);
        break;
    }

    case 10: {
        /* EPITROCHOID (R=3, r=1, d=2) — limaçon-like outer-rolling
         * curve.  2-bin spectrum. */
        sample_epitrochoid(3, 1, 2.f, out, N);
        break;
    }

    case 11: {
        /* DELTOID — 3-cusp hypocycloid.  R=3, r=1, d=1.
         * Cusp at every 120° → 3-fold symmetric spectral comb.
         *   x(t) = (2 cos t + cos 2t) / 3
         *   y(t) = (2 sin t - sin 2t) / 3                      */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = (2.f * cosf(t) + cosf(2.f*t)) / 3.f;
            out[k].im = (2.f * sinf(t) - sinf(2.f*t)) / 3.f;
        }
        break;
    }

    case 12: {
        /* 5-CUSP HYPOCYCLOID — R=5, r=1, d=1.  5-fold spectral comb.
         *   x(t) = (4 cos t + cos 4t) / 5
         *   y(t) = (4 sin t - sin 4t) / 5                      */
        for (int k = 0; k < N; k++) {
            float t   = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = (4.f * cosf(t) + cosf(4.f*t)) / 5.f;
            out[k].im = (4.f * sinf(t) - sinf(4.f*t)) / 5.f;
        }
        break;
    }

    case 13: {
        /* GEAR 8-TOOTH — radial square wave.  Radius alternates
         * sharply between R_outer and R_inner with period 2*pi/8.
         * Discontinuous derivative at every tooth → many high-freq
         * harmonics → slow energy convergence.  Bar still climbing
         * past M = 30. */
        for (int k = 0; k < N; k++) {
            float t = 2.f*(float)M_PI*(float)k/(float)N;
            float r = 0.6f + 0.3f * (sinf(8.f*t) > 0.f ? 1.f : -1.f);
            out[k].re = r * cosf(t);
            out[k].im = r * sinf(t);
        }
        break;
    }

    case 14: {
        /* CRESCENT — outer right semicircle + inner left semicircle
         * (offset).  Smooth except at the two join points → energy
         * converges fast initially then has a slow tail from the
         * corner singularities. */
        for (int k = 0; k < N; k++) {
            float t = 2.f*(float)M_PI*(float)k/(float)N;
            if (t < (float)M_PI) {
                /* outer semicircle, top → bottom on right side */
                float theta = (float)M_PI/2.f - t;
                out[k].re = cosf(theta);
                out[k].im = sinf(theta);
            } else {
                /* inner semicircle, bottom → top, offset right */
                float theta = -(float)M_PI/2.f + (t - (float)M_PI);
                out[k].re = 0.30f + 0.70f * cosf(theta);
                out[k].im = 0.70f * sinf(theta);
            }
        }
        break;
    }

    case 15: {
        /* LIGHTNING BOLT — piecewise-linear zigzag through 7 vertices.
         * Sharp angle at every vertex → many harmonics.  Slowest
         * convergence in the catalog: ~50% at M = 5, ~85% at M = 30,
         * 95% at M = 60+. */
        static const float lx[] = { 0.00f,  0.55f, -0.20f,
                                    0.30f, -0.40f,  0.20f,  0.00f };
        static const float ly[] = { 1.00f,  0.40f,  0.10f,
                                   -0.30f, -0.60f, -1.00f,  0.00f };
        sample_poly(lx, ly, 7, out, N);
        break;
    }

    case 16: {
        /* LISSAJOUS 11:7 — close-to-irrational ratio that visually
         * packs the screen with intersecting curves.
         *
         * NEW LESSON: visual complexity does NOT imply spectral
         * complexity.  Spectrum is still ONLY 4 dominant bins
         * (±11, ±7) — energy bar reaches 100% at M = 4 just like
         * the simpler Lissajous 3:2. */
        for (int k = 0; k < N; k++) {
            float t = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = sinf(11.f*t + (float)M_PI/4.f);
            out[k].im = sinf(7.f*t);
        }
        break;
    }

    case 17: {
        /* BEADED CIRCLE — circle of radius 1 with a small high-freq
         * VECTOR offset added per sample.  Looks like a string of
         * 8 beads on a necklace.
         *
         * NEW LESSON: vector AM places spectral energy at the TWO
         * carrier frequencies (±1 and ±8), NOT at sum/diff.
         * Compare with Gear 8-tooth: gear's RADIAL modulation
         * creates many sidebands; beaded's VECTOR modulation
         * creates just 4 bins.  Same "8" parameter, totally
         * different spectra. */
        for (int k = 0; k < N; k++) {
            float t = 2.f*(float)M_PI*(float)k/(float)N;
            out[k].re = cosf(t) + 0.25f*cosf(8.f*t);
            out[k].im = sinf(t) + 0.25f*sinf(8.f*t);
        }
        break;
    }

    case 18: {
        /* COMPOUND SPIROGRAPH — sum of TWO hypotrochoids that share
         * the same period.  Each by itself has a 2-bin spectrum;
         * the sum has 4.
         *
         * NEW LESSON: the DFT is LINEAR.  Coefficients of (curve_A +
         * curve_B) are exactly (coefs_A + coefs_B). */
        Cplx tmp[N_SAMPLES];
        sample_hypotrochoid(4, 2, 2.f, out, N);
        sample_hypotrochoid(6, 3, 2.f, tmp, N);
        for (int k = 0; k < N; k++) {
            out[k].re = 0.5f * (out[k].re + tmp[k].re);
            out[k].im = 0.5f * (out[k].im + tmp[k].im);
        }
        break;
    }

    case 19: {
        /* CLOSED ARCHIMEDEAN SPIRAL — r(t) = a*t for t in [0, 4*pi],
         * then a straight closing chord from spiral end to origin.
         *
         * NEW LESSON: PERIODICITY MATTERS.  The closing chord is a
         * sharp DISCONTINUITY in derivative; the spiral itself is
         * not periodic in 2*pi (it grows).  Energy spreads across
         * many bins.  Bar climbs slowly even though the visible
         * shape looks smooth. */
        int spiral_end = N * 9 / 10;     /* 90% of samples on spiral */
        for (int k = 0; k < spiral_end; k++) {
            float t = 4.f * (float)M_PI * (float)k / (float)spiral_end;
            float r = 0.95f * t / (4.f * (float)M_PI);
            out[k].re = r * cosf(t);
            out[k].im = r * sinf(t);
        }
        float end_re = out[spiral_end - 1].re;
        float end_im = out[spiral_end - 1].im;
        for (int k = spiral_end; k < N; k++) {
            float frac = (float)(k - spiral_end) / (float)(N - spiral_end);
            out[k].re = end_re * (1.f - frac);
            out[k].im = end_im * (1.f - frac);
        }
        break;
    }

    case 20: {
        /* TORUS KNOT (2, 5) — a closed curve that, in 3-D, is a
         * non-trivial knot wrapping a torus 5 times the short way
         * and 2 times the long way.  Projected to 2-D:
         *   x(t) = (2 + cos(5t)) * cos(2t) / 3
         *   y(t) = (2 + cos(5t)) * sin(2t) / 3
         *
         * NEW LESSON: knotted/torus closed curves have multi-frequency
         * spectra structured around their (p, q) winding parameters.
         * 5-fold modulation atop 2-rev carrier puts dominant energy
         * at signed bins ±2, ±3 (= 2-5), ±7 (= 2+5). */
        for (int k = 0; k < N; k++) {
            float t = 2.f*(float)M_PI*(float)k/(float)N;
            float r = 2.f + cosf(5.f*t);
            out[k].re = r * cosf(2.f*t) / 3.f;
            out[k].im = r * sinf(2.f*t) / 3.f;
        }
        break;
    }

    }
}

/* ===================================================================== */
/* §7  tier_classify — convergence tier table + helper                   */
/* ===================================================================== */
/*
 * Every shape belongs to one of three convergence tiers based on
 * its Fourier-coefficient decay rate.  The HUD prints
 * "[Fast 5/9]" etc so the user can navigate the catalog with intent.
 *
 *   FAST  : energy bar reaches ~100% by M = 6 (typically 2 or 4
 *           dominant bins; everything else is noise)
 *   MED   : ~85% by M = 10, slow tail (cusps or composites)
 *   SLOW  : need 50+ arms for 95% (sharp corners, discontinuities)
 */
typedef enum { TIER_FAST = 0, TIER_MED, TIER_SLOW, TIER_COUNT } Tier;

static const Tier k_shape_tier[N_SHAPES] = {
    /*  0 Square           */ TIER_SLOW,
    /*  1 Arrow            */ TIER_SLOW,
    /*  2 Star-7           */ TIER_MED,
    /*  3 Cardioid         */ TIER_FAST,
    /*  4 Lissajous 3:2    */ TIER_FAST,
    /*  5 Rose r=cos(2t)   */ TIER_MED,
    /*  6 Lissajous 5:4    */ TIER_FAST,
    /*  7 Lissajous 7:3    */ TIER_FAST,
    /*  8 Hypotrochoid 5:3 */ TIER_FAST,
    /*  9 Hypotrochoid 7:4 */ TIER_FAST,
    /* 10 Epitrochoid 3:1  */ TIER_FAST,
    /* 11 Deltoid          */ TIER_MED,
    /* 12 Hypocycloid 5    */ TIER_MED,
    /* 13 Gear 8-tooth     */ TIER_SLOW,
    /* 14 Crescent         */ TIER_MED,
    /* 15 Lightning        */ TIER_SLOW,
    /* 16 Lissajous 11:7   */ TIER_FAST,
    /* 17 Beaded circle    */ TIER_MED,
    /* 18 Compound spiro   */ TIER_FAST,
    /* 19 Closed spiral    */ TIER_SLOW,
    /* 20 Torus knot 2:5   */ TIER_MED,
};

static const char *k_tier_name[TIER_COUNT] = { "Fast", "Med ", "Slow" };

/*
 * shape_tier_position — for the active shape, compute its tier name,
 * its position within that tier (1-indexed), and the total count
 * of shapes in that tier.  HUD prints "[Fast 5/9]" using this.
 */
static void shape_tier_position(int shape_idx,
                                const char **out_tier_name,
                                int *out_pos, int *out_total)
{
    Tier t = k_shape_tier[shape_idx];
    *out_tier_name = k_tier_name[t];
    *out_pos = 0;
    *out_total = 0;
    for (int i = 0; i < N_SHAPES; i++) {
        if (k_shape_tier[i] != t) continue;
        (*out_total)++;
        if (i <= shape_idx) *out_pos = *out_total;
    }
}

/* ===================================================================== */
/* §8  dft — O(N^2) DFT with twiddle recurrence                          */
/* ===================================================================== */
/*
 *  Purpose         : compute the discrete Fourier transform of N
 *                    complex samples.  Exactly the same algorithm
 *                    as dft_helloworld.c §5 (no trickery).
 *  Optimisation    : the TWIDDLE RECURRENCE — for each output bin we
 *                    pre-compute W = exp(-2*pi*i*n/N) ONCE, then
 *                    multiply a running `w` by W per inner iteration
 *                    instead of calling cos/sin every time.  Cuts
 *                    trig calls from 2*N^2 to 2*N.
 *  Mental model    : N parallel "lock-in detectors", each tuned to
 *                    one integer harmonic.  See dft_helloworld.c for
 *                    the full intuition.
 *  Inputs/outputs  : in[N] complex / out[N] complex (caller-allocated)
 *  Why it exists   : runs ONCE per shape change inside §9
 *                    build_epicycles.  Not in the per-frame hot path.
 *  Cost            : O(N^2) — at N = 256, ~65k complex multiplies.
 *                    Sub-millisecond.
 */
static void compute_dft(const Cplx *in, Cplx *out, int N)
{
    for (int n = 0; n < N; n++) {
        float twiddle_step_re = cosf(-2.f * (float)M_PI * (float)n / (float)N);
        float twiddle_step_im = sinf(-2.f * (float)M_PI * (float)n / (float)N);
        float twiddle_re = 1.f, twiddle_im = 0.f;
        float acc_re     = 0.f, acc_im     = 0.f;

        for (int k = 0; k < N; k++) {
            /* acc += in[k] * twiddle  (complex mul spelled out) */
            acc_re += in[k].re * twiddle_re - in[k].im * twiddle_im;
            acc_im += in[k].re * twiddle_im + in[k].im * twiddle_re;

            /* twiddle = twiddle * twiddle_step  (advance W^k -> W^(k+1)) */
            float next_re = twiddle_re * twiddle_step_re
                          - twiddle_im * twiddle_step_im;
            twiddle_im    = twiddle_re * twiddle_step_im
                          + twiddle_im * twiddle_step_re;
            twiddle_re    = next_re;
        }
        out[n].re = acc_re;
        out[n].im = acc_im;
    }
}

/* ===================================================================== */
/* §9  epicycle_table — struct + sort + cumulative energy table          */
/* ===================================================================== */

typedef struct {
    float amp;    /* normalised arm length = |Z[n]| / N           */
    float phase;  /* arg(Z[n])                                    */
    int   freq;   /* signed rotation rate (cycles per shape trace)*/
} Epicycle;

/*
 * epic_cmp — qsort comparator: amplitude DESCENDING.  Used by
 * build_epicycles to sort the arm table.  Sorting descending is
 * what makes the partial sums energy-OPTIMAL (T3).
 */
static int epic_cmp(const void *a, const void *b)
{
    float da = ((const Epicycle *)a)->amp;
    float db = ((const Epicycle *)b)->amp;
    return (da < db) - (da > db);
}

static Epicycle g_epicycle_table[N_SAMPLES];
static int      g_total_epicycle_count = 0;
static int      g_active_epicycle_count = 0;

/*
 * g_cumulative_energy_fraction[k] — fraction of total signal power
 *   captured by the first k+1 epicycles (sorted by amplitude).
 *   The HUD's energy bar reads
 *   g_cumulative_energy_fraction[active_count - 1].
 *
 * Parseval connection: total_signal_energy = sum(amp^2) =
 *   (1/N^2) * sum |Z[n]|^2 = (1/N) * sum |x[k]|^2.
 */
static float g_cumulative_energy_fraction[N_SAMPLES];
static float g_total_signal_energy;

/* Source samples preserved for the ghost overlay. */
static Cplx g_resampled_source[N_SAMPLES];

/*
 * build_epicycles — turn a chosen shape into a sorted Epicycle[]
 * with a pre-computed cumulative-energy table.
 *
 * Pseudocode:
 *   sample_path(shape_idx, ghost)
 *   compute_dft(ghost, dft_bins)
 *   for n in 0..N-1:
 *     amp   = |dft_bins[n]| / N
 *     phase = arg(dft_bins[n])
 *     freq  = (n <= N/2) ? n : n - N           // signed
 *     table[n] = {amp, phase, freq}
 *   qsort(table, descending by amp)
 *   total_energy = sum(amp^2)
 *   running = 0
 *   for k in 0..N-1:
 *     running += amp_k^2
 *     cumulative_energy_fraction[k] = running / total_energy
 *
 * Why it exists : called at startup, on shape change ('n'/'p'),
 *                 and on reset ('r').  Keeps the DFT cost out of
 *                 the per-frame hot path.
 */
static void build_epicycles(int shape_idx)
{
    Cplx dft[N_SAMPLES];
    sample_path(shape_idx, g_resampled_source, N_SAMPLES);
    compute_dft(g_resampled_source, dft, N_SAMPLES);

    float inv_N = 1.f / (float)N_SAMPLES;
    for (int n = 0; n < N_SAMPLES; n++) {
        float re = dft[n].re, im = dft[n].im;
        int   f  = (n <= N_SAMPLES/2) ? n : n - N_SAMPLES;   /* signed freq */
        g_epicycle_table[n] = (Epicycle){
            sqrtf(re*re + im*im) * inv_N, atan2f(im, re), f
        };
    }
    qsort(g_epicycle_table, N_SAMPLES, sizeof(Epicycle), epic_cmp);
    g_total_epicycle_count = N_SAMPLES;

    /* Build cumulative energy table (Parseval). */
    g_total_signal_energy = 0.f;
    for (int n = 0; n < N_SAMPLES; n++)
        g_total_signal_energy +=
            g_epicycle_table[n].amp * g_epicycle_table[n].amp;

    float acc = 0.f;
    for (int n = 0; n < N_SAMPLES; n++) {
        acc += g_epicycle_table[n].amp * g_epicycle_table[n].amp;
        g_cumulative_energy_fraction[n] =
            (g_total_signal_energy > 0.f)
            ? acc / g_total_signal_energy : 0.f;
    }
}

/* ===================================================================== */
/* §10  trail — tip-path ring buffer                                     */
/* ===================================================================== */

typedef struct {
    float pixel_x[TRAIL_LEN];
    float pixel_y[TRAIL_LEN];
    int   write_head;
    int   filled_count;
} Trail;

static void trail_push(Trail *t, float pixel_x, float pixel_y)
{
    /* Append (x, y); advance head; saturate count at TRAIL_LEN. */
    t->pixel_x[t->write_head] = pixel_x;
    t->pixel_y[t->write_head] = pixel_y;
    t->write_head = (t->write_head + 1) % TRAIL_LEN;
    if (t->filled_count < TRAIL_LEN) t->filled_count++;
}

static void trail_clear(Trail *t)
{
    /* Logically empty without zeroing the data array. */
    t->write_head = 0;
    t->filled_count = 0;
}

/* ===================================================================== */
/* §11  scene_state — globals + reset/init                               */
/* ===================================================================== */

static int   g_screen_rows, g_screen_cols;
static float g_pivot_pixel_x, g_pivot_pixel_y;   /* chain origin in pixel space */
static float g_pixel_scale;                       /* normalised → pixel        */
static float g_animation_phase_radians;           /* phi ∈ [0, 2*pi)            */
static int   g_auto_add_counter;
static bool  g_simulation_paused;
static bool  g_auto_add_enabled;
static bool  g_show_orbit_circles;
static bool  g_show_ghost_overlay;
static int   g_active_shape_index;
static Trail g_tip_trail;

/* Joint positions in pixel space; joint[0] = pivot, joint[i+1] is
 * the tip of arm i (and pivot of arm i+1). */
static float g_joint_pixel_x[N_SAMPLES + 1];
static float g_joint_pixel_y[N_SAMPLES + 1];

static int px_cx(float pixel_x) { return (int)(pixel_x / (float)CELL_W + 0.5f); }
static int px_cy(float pixel_y) { return (int)(pixel_y / (float)CELL_H + 0.5f); }

/* ===================================================================== */
/* §12  scene_chain — compute joint positions from epicycle table        */
/* ===================================================================== */
/*
 *  Purpose         : evaluate the chain at the current phi; record
 *                    joint positions in g_joint_pixel_x/y[].
 *  Pseudocode      :
 *      (x, y) = (pivot_x, pivot_y)
 *      joint[0] = (x, y)
 *      for i in 0..M-1:
 *        angle  = freq_i * phi + phase_i
 *        radius = amp_i * pixel_scale
 *        x += radius * cos(angle)
 *        y += radius * sin(angle)
 *        joint[i+1] = (x, y)
 *  Mental model    : add the i-th arm's tip displacement to the
 *                    running joint position.  Each arm is a polar-
 *                    to-Cartesian conversion.
 *  Inputs/outputs  : reads g_epicycle_table, g_animation_phase_radians,
 *                    g_pivot_pixel_x/y, g_pixel_scale, g_active_epicycle_count
 *                    / writes g_joint_pixel_x/y[0..M].
 *  Why it exists   : rendering layers consume joint positions, not
 *                    arm parameters.  Computing joints once per frame
 *                    avoids redundant work.
 */
static void scene_compute_chain(void)
{
    float x = g_pivot_pixel_x, y = g_pivot_pixel_y;
    g_joint_pixel_x[0] = x;
    g_joint_pixel_y[0] = y;
    for (int i = 0; i < g_active_epicycle_count; i++) {
        float ang = (float)g_epicycle_table[i].freq
                  * g_animation_phase_radians
                  + g_epicycle_table[i].phase;
        float r   = g_epicycle_table[i].amp * g_pixel_scale;
        x += r * cosf(ang);
        y += r * sinf(ang);
        g_joint_pixel_x[i+1] = x;
        g_joint_pixel_y[i+1] = y;
    }
}

/* ===================================================================== */
/* §13  scene_tick — per-frame orchestrator                              */
/* ===================================================================== */
/*
 *  Purpose : advance state by one frame.  Three numbered steps that
 *            match the per-frame portion of ALGORITHM IN STEPS.
 *
 *  Steps:
 *    1. (auto-add) maybe bump g_active_epicycle_count by 1.
 *    2. Advance phi; on wrap, clear the trail.
 *    3. Recompute chain joints; push tip into trail.
 */
static void scene_reset(int shape_idx)
{
    g_active_shape_index = shape_idx;
    g_animation_phase_radians = 0.f;
    g_auto_add_counter = 0;
    g_active_epicycle_count = 1;
    trail_clear(&g_tip_trail);
    build_epicycles(g_active_shape_index);
    scene_compute_chain();
}

static void scene_init(int rows, int cols)
{
    g_screen_rows = rows;
    g_screen_cols = cols;
    g_pivot_pixel_x = (float)(cols * CELL_W) * 0.5f;
    g_pivot_pixel_y = (float)(rows * CELL_H) * 0.5f;
    float min_pixels = fminf((float)(cols * CELL_W),
                             (float)(rows * CELL_H));
    g_pixel_scale = min_pixels * SHAPE_SCALE;
    g_simulation_paused   = false;
    g_auto_add_enabled    = true;
    g_show_orbit_circles  = true;
    g_show_ghost_overlay  = true;
    scene_reset(0);
}

static void scene_tick(void)
{
    if (g_simulation_paused) return;

    /* ── Step 1 — auto-add one arm at intervals ───────────────── */
    if (g_auto_add_enabled
     && g_active_epicycle_count < g_total_epicycle_count) {
        g_auto_add_counter++;
        if (g_auto_add_counter >= AUTO_ADD_FRAMES) {
            g_auto_add_counter = 0;
            g_active_epicycle_count++;
        }
    }

    /* ── Step 2 — advance phi; clear trail at cycle wrap ────── */
    g_animation_phase_radians += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
    if (g_animation_phase_radians >= 2.f * (float)M_PI) {
        g_animation_phase_radians -= 2.f * (float)M_PI;
        trail_clear(&g_tip_trail);
    }

    /* ── Step 3 — recompute chain; record tip ─────────────────── */
    scene_compute_chain();
    trail_push(&g_tip_trail,
               g_joint_pixel_x[g_active_epicycle_count],
               g_joint_pixel_y[g_active_epicycle_count]);
}

/* ===================================================================== */
/* §14  draw_helpers — Bresenham line + cell-aspect orbit ellipse        */
/* ===================================================================== */

/*
 * draw_line_seg — Bresenham line with direction-aware glyph choice.
 *   Mostly-horizontal segments → '-'
 *   Mostly-vertical   segments → '|'
 *   Diagonal (\ or /)         → '\\' / '/'
 */
static void draw_line_seg(int x0, int y0, int x1, int y1, attr_t attr)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        if (x0 >= 0 && x0 < g_screen_cols
         && y0 >= 0 && y0 < g_screen_rows) {
            int  e2 = 2 * err;
            bool bx = e2 > -dy, by = e2 < dx;
            chtype ch = (bx && by) ? (sx == sy ? '\\' : '/')
                      :  bx ? '-' : '|';
            attron(attr);
            mvaddch(y0, x0, ch);
            attroff(attr);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * draw_orbit — dotted ellipse around (piv_px, piv_py) with pixel-
 * space radius r_px.  In CELL space the orbit is an ellipse because
 * cells are CELL_H/CELL_W = 2x as tall as wide.
 */
static void draw_orbit(float piv_px, float piv_py, float r_px)
{
    if (r_px < (float)CELL_W * 0.5f) return;   /* skip sub-cell orbits */
    float rx    = r_px / (float)CELL_W;
    float ry    = r_px / (float)CELL_H;
    int   pcx   = px_cx(piv_px), pcy = px_cy(piv_py);
    int   steps = (int)(2.f * (float)M_PI * fmaxf(rx, ry)) + 4;
    for (int i = 0; i < steps; i++) {
        float a = 2.f * (float)M_PI * (float)i / (float)steps;
        int x   = pcx + (int)(rx * cosf(a) + 0.5f);
        int y   = pcy + (int)(ry * sinf(a) + 0.5f);
        if (x >= 0 && x < g_screen_cols
         && y >= 0 && y < g_screen_rows) {
            attron(COLOR_PAIR(CP_CIRCLE));
            mvaddch(y, x, '.');
            attroff(COLOR_PAIR(CP_CIRCLE));
        }
    }
}

/* ===================================================================== */
/* §15  draw_layers — ghost / orbits / trail / chain / markers           */
/* ===================================================================== */

static void scene_draw(void)
{
    /* ── Layer 1.  GHOST overlay — source samples as ':' dots ──── */
    /*
     * Every other sample is drawn (i += 2) to keep density readable.
     * Shows where the "true" shape lies so the viewer can compare
     * the reconstruction's accuracy directly against the target.
     */
    if (g_show_ghost_overlay) {
        attron(COLOR_PAIR(CP_GHOST));
        for (int k = 0; k < N_SAMPLES; k += 2) {
            float px = g_pivot_pixel_x + g_resampled_source[k].re * g_pixel_scale;
            float py = g_pivot_pixel_y + g_resampled_source[k].im * g_pixel_scale;
            int   cx = px_cx(px), cy = px_cy(py);
            if (cx >= 0 && cx < g_screen_cols
             && cy >= 0 && cy < g_screen_rows)
                mvaddch(cy, cx, ':');
        }
        attroff(COLOR_PAIR(CP_GHOST));
    }

    /* ── Layer 2.  ORBIT circles for the largest arms ───────────── */
    if (g_show_orbit_circles) {
        int nc = g_active_epicycle_count < N_CIRCLES
               ? g_active_epicycle_count : N_CIRCLES;
        for (int i = 0; i < nc; i++)
            draw_orbit(g_joint_pixel_x[i], g_joint_pixel_y[i],
                       g_epicycle_table[i].amp * g_pixel_scale);
    }

    /* ── Layer 3.  TRAIL — fading dots from oldest to newest ───── */
    {
        int draw  = g_tip_trail.filled_count;
        int start = (g_tip_trail.write_head - draw + TRAIL_LEN) % TRAIL_LEN;
        for (int i = 0; i < draw; i++) {
            int   idx = (start + i) % TRAIL_LEN;
            int   cx  = px_cx(g_tip_trail.pixel_x[idx]);
            int   cy  = px_cy(g_tip_trail.pixel_y[idx]);
            if (cx < 0 || cx >= g_screen_cols
             || cy < 0 || cy >= g_screen_rows) continue;
            float age = (float)i / (float)draw;   /* 0 = oldest, 1 = newest */
            int   cp  = age > 0.70f ? CP_TRAIL_1
                      : age > 0.35f ? CP_TRAIL_2 : CP_TRAIL_3;
            attron(COLOR_PAIR(cp) | A_BOLD);
            mvaddch(cy, cx, '*');
            attroff(COLOR_PAIR(cp) | A_BOLD);
        }
    }

    /* ── Layer 4.  ARM CHAIN — Bresenham segments per arm ──────── */
    for (int i = 0; i < g_active_epicycle_count; i++) {
        float r_px = g_epicycle_table[i].amp * g_pixel_scale;
        int   cp   = r_px > g_pixel_scale * 0.10f ? CP_ARM_HI
                   : r_px > g_pixel_scale * 0.02f ? CP_ARM_MID : CP_ARM_LO;
        draw_line_seg(px_cx(g_joint_pixel_x[i]),   px_cy(g_joint_pixel_y[i]),
                      px_cx(g_joint_pixel_x[i+1]), px_cy(g_joint_pixel_y[i+1]),
                      COLOR_PAIR(cp) | A_BOLD);
    }

    /* ── Layer 5a.  TIP BOB — '@' at chain end ──────────────────── */
    {
        int bx = px_cx(g_joint_pixel_x[g_active_epicycle_count]);
        int by = px_cy(g_joint_pixel_y[g_active_epicycle_count]);
        if (bx >= 0 && bx < g_screen_cols
         && by >= 0 && by < g_screen_rows) {
            attron(COLOR_PAIR(CP_BOB) | A_BOLD);
            mvaddch(by, bx, '@');
            attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
        }
    }

    /* ── Layer 5b.  PIVOT MARKER — '+' at chain origin ─────────── */
    {
        int pcx = px_cx(g_pivot_pixel_x), pcy = px_cy(g_pivot_pixel_y);
        if (pcx >= 0 && pcx < g_screen_cols
         && pcy >= 0 && pcy < g_screen_rows) {
            attron(COLOR_PAIR(CP_PIVOT) | A_BOLD);
            mvaddch(pcy, pcx, '+');
            attroff(COLOR_PAIR(CP_PIVOT) | A_BOLD);
        }
    }
}

/* ===================================================================== */
/* §16  hud_energy — status top-right + ENERGY BAR + bottom hint         */
/* ===================================================================== */
/*
 *  HUD layout (CLAUDE.md HUD Standard):
 *    row 0 top-left  : " PAUSED " chip when paused, otherwise blank
 *    row 0 top-right : status line (bright yellow + bold) including
 *                      shape name, tier chip, arm count, energy %,
 *                      theme, toggles
 *    row 1           : ENERGY BAR with colour tier (red < 50%,
 *                      yellow 50..80%, green >= 80%) — THE headline
 *                      visual of this file
 *    row LINES-1     : bottom-left hint (bright cyan + bold)
 */

static void scene_draw_hud(void)
{
    float efrac = (g_active_epicycle_count > 0 && g_total_signal_energy > 0.f)
                ? g_cumulative_energy_fraction[g_active_epicycle_count - 1]
                : 0.f;

    /* ── status (top-right) ── */
    {
        const char *tier_name;
        int tier_pos = 0, tier_total = 0;
        shape_tier_position(g_active_shape_index, &tier_name, &tier_pos, &tier_total);

        char buf[220];
        snprintf(buf, sizeof buf,
                 " FourierShapes  %s [%s %d/%d]  arms:%3d/%d  energy:%5.1f%%  thm:%s  %s%s%s ",
                 k_shape_names[g_active_shape_index],
                 tier_name, tier_pos, tier_total,
                 g_active_epicycle_count, g_total_epicycle_count,
                 efrac * 100.f,
                 k_themes[g_active_theme_index].name,
                 g_auto_add_enabled    ? "auto "    : "       ",
                 g_show_orbit_circles  ? "circles " : "        ",
                 g_show_ghost_overlay  ? "ghost"    : "     ");
        int x = g_screen_cols - (int)strlen(buf);
        if (x < 0) x = 0;
        attron(COLOR_PAIR(CP_HUD) | A_BOLD);
        mvprintw(0, x, "%s", buf);
        attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
    }

    /* ── PAUSED chip (top-left) ── */
    if (g_simulation_paused) {
        attron(COLOR_PAIR(CP_HUD) | A_BOLD | A_REVERSE);
        mvprintw(0, 0, " PAUSED ");
        attroff(COLOR_PAIR(CP_HUD) | A_BOLD | A_REVERSE);
    }

    /* ── ENERGY BAR with colour tier (row 1) ── */
    {
        int bar_col = 1, bar_row = 1, bar_w = 50;
        if (bar_col + bar_w + 16 >= g_screen_cols)
            bar_w = g_screen_cols - bar_col - 17;
        if (bar_w < 12) bar_w = 12;

        /* Pick colour tier by energy fraction. */
        int bar_pair = (efrac >= 0.80f) ? CP_ENERGY
                     : (efrac >= 0.50f) ? CP_ENERGY_MID
                                        : CP_ENERGY_LO;

        /* "[" border + filled "=" + empty "-" + "]" border + label */
        attron(COLOR_PAIR(CP_HUD));
        mvaddch(bar_row, bar_col,             '[');
        mvaddch(bar_row, bar_col + bar_w - 1, ']');
        attroff(COLOR_PAIR(CP_HUD));

        int filled = (int)(efrac * (float)(bar_w - 2));
        attron(COLOR_PAIR(bar_pair) | A_BOLD);
        for (int i = 0; i < filled; i++)
            mvaddch(bar_row, bar_col + 1 + i, '=');
        attroff(COLOR_PAIR(bar_pair) | A_BOLD);

        attron(COLOR_PAIR(CP_HUD));
        for (int i = filled; i < bar_w - 2; i++)
            mvaddch(bar_row, bar_col + 1 + i, '-');
        mvprintw(bar_row, bar_col + bar_w + 1, "Parseval power");
        attroff(COLOR_PAIR(CP_HUD));
    }

    /* ── bottom-left hint ── */
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(g_screen_rows - 1, 0,
             " q:quit  spc:pause  n/p:shape  +/-:arms  r:reset  a:auto  c:circles  g:ghost  t/T:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §17  screen — ncurses init / cleanup / present                        */
/* ===================================================================== */

static void screen_init(void)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §18  app — signal handlers + main loop + key dispatch                 */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int s)
{
    if (s == SIGINT  || s == SIGTERM) g_should_quit    = 1;
    if (s == SIGWINCH)                g_resize_pending = 1;
}

static void cleanup(void) { endwin(); }

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    scene_init(rows, cols);

    long long last_ns = clock_ns();

    while (!g_should_quit) {

        /* ── resize ────────────────────────────────────────────── */
        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_init(rows, cols);
            last_ns = clock_ns();
            continue;
        }

        /* ── input ─────────────────────────────────────────────── */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27:
            g_should_quit = 1; break;
        case ' ':
            g_simulation_paused = !g_simulation_paused; break;
        case 'n': case 'N':
            scene_reset((g_active_shape_index + 1) % N_SHAPES); break;
        case 'p': case 'P':
            scene_reset((g_active_shape_index + N_SHAPES - 1) % N_SHAPES);
            break;
        case '+': case '=':
            if (g_active_epicycle_count < g_total_epicycle_count)
                g_active_epicycle_count++;
            break;
        case '-':
            if (g_active_epicycle_count > 1) {
                g_active_epicycle_count--;
                trail_clear(&g_tip_trail);
            }
            break;
        case 'r': case 'R':
            scene_reset(g_active_shape_index); break;
        case 'a': case 'A':
            g_auto_add_enabled = !g_auto_add_enabled; break;
        case 'c': case 'C':
            g_show_orbit_circles = !g_show_orbit_circles; break;
        case 'g': case 'G':
            g_show_ghost_overlay = !g_show_ghost_overlay; break;
        case 't':
            g_active_theme_index = (g_active_theme_index + 1) % N_THEMES;
            apply_theme(g_active_theme_index);
            break;
        case 'T':
            g_active_theme_index =
                (g_active_theme_index + N_THEMES - 1) % N_THEMES;
            apply_theme(g_active_theme_index);
            break;
        default: break;
        }

        /* ── tick ──────────────────────────────────────────────── */
        long long now_ns = clock_ns();
        long long dt     = now_ns - last_ns;
        last_ns = now_ns;
        if (dt > 100000000LL) dt = 100000000LL;   /* pause-guard */
        (void)dt;

        scene_tick();

        /* ── draw + present ────────────────────────────────────── */
        erase();
        scene_draw();
        scene_draw_hud();
        screen_present();

        /* ── frame cap ─────────────────────────────────────────── */
        clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
    }

    return 0;
}
