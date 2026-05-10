/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fourier_draw.c — interactive Fourier drawing.  Scribble any path with
 *                 arrow keys, press ENTER, and watch a chain of rotating
 *                 circles redraw it from scratch.
 *
 * DEMO
 *   The program lives in two modes that swap with one keypress.
 *
 *   In DRAW mode, a cursor sits on the screen.  Arrow keys move it,
 *   leaving a continuous trail behind it that says "this is the path
 *   you have drawn so far."  When you press ENTER, the program looks at
 *   that path, computes a recipe of rotating arms that — when chained
 *   end to end — would trace the same path, and switches to PLAY mode.
 *
 *   In PLAY mode, that arm chain is animated from the centre of the
 *   screen.  The tip of the chain leaves a glowing trail that
 *   reproduces your drawing.  The dimmed-down "ghost" of your original
 *   path is laid underneath so you can see, frame by frame, how
 *   accurately the chain is following what you scribbled.  Press '+' or
 *   '-' to add or remove arms; press 'o' to flip the auto-close toggle
 *   and watch the chain's wrap-around seam appear and disappear; press
 *   'r' to draw something new.
 *
 *   The lesson, made visceral by being interactive: ANY closed two-
 *   dimensional path is the sum of a finite chain of integer-rate
 *   rotations.  And the recipe machine — the discrete Fourier
 *   transform — is the same one that runs inside MP3 encoders, audio
 *   spectrum analysers, JPEG compressors, and a thousand other places.
 *
 * Study alongside
 *   signal/fourier_shapes.c  The same epicycle-chain animation, but the
 *                             input is one of 21 preset parametric shapes
 *                             instead of a user-drawn path.  Adds an
 *                             energy bar (Parseval's theorem).
 *   signal/epicycles.c       The same algorithm with PRESET parametric
 *                             shapes (heart, star, butterfly, ...).  That
 *                             file is the deep dive on DFT, twiddle
 *                             recurrence, signed frequencies, Gibbs
 *                             convergence, and the chain animation.  This
 *                             file is the interactive sibling that
 *                             substitutes "shape function" with "the user".
 *   signal/fft_vis.c         The 1-D version: same Fourier maths applied
 *                             to a scalar audio-style signal, with
 *                             windowing and a spectrogram waterfall.
 *
 * Section map
 *   §1   config             tunable constants (no magic numbers elsewhere)
 *   §2   clock              monotonic-ns timer + nanosecond sleep
 *   §3   dft                O(N²) DFT with twiddle recurrence
 *   §4   resample           the unique algorithm: arc-length resampling
 *   §5   epicycle_table     decode + sort the DFT into a flat arm table
 *   §6   themes             palette table for the 't' cycler
 *   §7   colors             apply_theme + theme-invariant pair init
 *   §8   trail              tip-path ring buffer
 *   §9   draw_primitives    px↔cell, Bresenham line, Bresenham mark, orbit
 *   §10  draw_state         DRAW-mode state, cursor, recording, bbox
 *   §11  draw_compute       resample → build → set up scale & pivot
 *   §12  play_state         PLAY-mode state + per-tick simulation update
 *   §13  project            raw pixel → screen cell transform (ghost overlay)
 *   §14  render_layers      ghost / orbits / trail / chain / markers
 *   §15  render_debug       'D' arm-info table (top-N arms)
 *   §16  hud                HUD top-right + hint bottom + PAUSED chip
 *   §17  draw_mode_render   DRAW-mode painter (orchestrates §14 etc.)
 *   §18  play_mode_render   PLAY-mode painter
 *   §19  screen             ncurses init / cleanup
 *   §20  app                signal handlers + main loop + key dispatch
 *
 * Keys
 *   DRAW mode:
 *     arrows | WASD     move cursor and record path
 *     ENTER  | g        compute DFT → switch to PLAY
 *     c                 clear path and start over
 *     o                 toggle auto-close (synthetic closing segment)
 *     t / T             next / previous theme
 *     q | Q | ESC       quit
 *
 *   PLAY mode:
 *     r                 back to DRAW (path is preserved)
 *     space | p         pause / resume
 *     + / -             add / remove one epicycle
 *     c                 toggle orbit-circle overlay
 *     o                 toggle auto-close — re-runs DFT live so you can
 *                        see the seam appear and disappear
 *     d                 toggle ghost overlay (faint trace of original path)
 *     D                 toggle 'D' arm-info numeric table top-left
 *     a                 toggle auto-add (one new arm every ~0.27 s)
 *     t / T             next / previous theme
 *     q | Q | ESC       quit
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra signal/fourier_draw.c -o fourier_draw \
 *       -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ────────────────────────────────────────────── *
 *
 * Reading order
 *   First-time reader, no Fourier background:
 *     1. CONCEPTS — what we're doing and why a learner should care.
 *     2. MENTAL MODEL — pictures + invariants you need in mind.
 *     3. GUIDED TUTORIAL T1..T11 — eleven mini-lessons, each ending in
 *        a piece of pseudocode that maps to a function in §3..§14.
 *        T3 (arc-length resampling) and T4 (the closure problem) are
 *        the two that don't appear in epicycles.c — they are the
 *        contributions unique to this file.
 *     4. §1 config — the knobs (now you know what they do).
 *     5. §3 dft → §4 resample → §5 epicycle_table — the algorithm core.
 *     6. §10 draw_state → §11 draw_compute → §12 play_state — the
 *        two modes wired together.
 *     7. §13..§18 — rendering layers (least interesting algorithmically,
 *        but they show how to put a watchable picture on a terminal).
 *     8. §19..§20 — ncurses + main loop + key dispatch.
 *
 *   Reader who already knows DFT and just wants the new algorithm:
 *     §1 config, §4 resample, §11 draw_compute, §12 play_state.
 *     The rest is plumbing or shared with epicycles.c.
 *
 * Naming convention
 *   File-scope globals are prefixed `g_` to distinguish them from local
 *   variables.  Their names are spelled out so you can grep-find every
 *   read and write:
 *
 *     g_raw_pixel_x, g_raw_pixel_y        raw cursor path in pixel space
 *     g_raw_point_count                   how many raw points are recorded
 *     g_active_epicycle_count             arms currently animated (1..N)
 *     g_total_epicycle_count              full table size after build
 *     g_animation_phase_radians           φ — the global animation clock
 *     g_screen_pivot_pixel_x/y            chain origin in pixel space
 *     g_pixel_scale_per_unit              normalised → pixel multiplier
 *     g_max_radius_pixels                 max |z| of centred path
 *     g_path_centroid_pixel_x/y           mean of resampled samples
 *     g_joint_pixel_x[i], _y[i]           i-th joint of the arm chain
 *     g_close_path_enabled                'o' toggle
 *     g_show_ghost_overlay                'd' toggle
 *     g_show_arm_table                    'D' toggle
 *
 *   Short single-letter names (`i`, `k`, `n`, `dx`, `dy`) are kept
 *   inside tight loops where context is one line away.
 *
 * Background you NEED
 *   The DFT formula at the level of "X[k] is a weighted sum of inputs".
 *   See signal/epicycles.c CONCEPTS, or any DSP textbook.
 *
 *   Basic complex arithmetic.  We treat (x, y) points as complex numbers
 *   x + iy and run a 1-D DFT on them — see T5.
 *
 * Background you do NOT need
 *   The Fast Fourier Transform.  We use an O(N²) DFT; N = 256 is small
 *   enough that the transform runs once at ENTER-press time in well
 *   under a millisecond.
 *
 *   Continuous calculus.  Arc-length resampling is presented in T3 as
 *   a finite sum, not an integral.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm
 *   PATH RECORDING → ARC-LENGTH RESAMPLING → O(N²) DFT → AMPLITUDE
 *   SORTING → CHAIN ANIMATION.  Each step has a clear job.  The
 *   recording captures whatever the user types.  The arc-length step
 *   converts that into uniform-time samples, regardless of how fast
 *   or slow the user drew.  The DFT decomposes the path into N integer-
 *   frequency rotations.  Sorting puts the dominant rotations first so
 *   that "draw with M arms" always means "draw with the M most
 *   important arms".  The animation walks that table N times a second
 *   to render the live chain.
 *
 * Data structure
 *   Three flat arrays, sized at compile time:
 *     g_raw_pixel_x/y[RAW_MAX]              free-form draw recording
 *     g_epics[N_SAMPLES]                    sorted (amp, phase, freq) records
 *     g_joint_pixel_x/y[N_SAMPLES + 1]      per-frame chain joint positions
 *
 *   No malloc on the hot path.  The only dynamic allocation is a
 *   scratch arc-length buffer inside resample (free'd on return).
 *
 * Rendering
 *   Pixel-space simulation, cell-space draw — each terminal cell is
 *   CELL_W × CELL_H sub-pixels, and the chain math runs in continuous
 *   sub-cell coordinates so arm endpoints don't snap to the grid.
 *   Conversion happens at one point per draw call (px_to_cell_*).
 *
 *   PLAY-mode rendering is layered: ghost (faint original) → orbits →
 *   trail → arm chain → markers → debug → HUD.  Later layers paint
 *   over earlier ones in any cells they share.
 *
 * Performance
 *   The DFT is O(N²) but runs ONCE per ENTER press (or 'o' toggle in
 *   PLAY) — about 65k multiplies at N=256, well under a millisecond.
 *   Per-frame work is the chain walk (O(M)) and the trail render
 *   (O(L)).  Holds 30 fps on any terminal that can keep up with 30
 *   redraws per second.  No malloc except the scratch arc buffer.
 *
 * References
 *   Cooley & Tukey (1965), "An Algorithm for the Machine Calculation
 *     of Complex Fourier Series", Math. Comp. 19, 297-301.
 *   3Blue1Brown, "But what is a Fourier series? From heat flow to
 *     drawing with circles", YouTube — the popular intro.  Watch this
 *     before reading the code if you've never seen epicycles.
 *   Press et al., "Numerical Recipes in C", Chapter 12 (DFT).
 *   Foley & van Dam, "Computer Graphics: Principles and Practice"
 *     §3.2.1 — Bresenham's line algorithm.
 *   https://en.wikipedia.org/wiki/Discrete_Fourier_transform
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 *   The Fourier transform is a recipe machine that takes any closed
 *   2-D path as input and returns a list of (radius, phase, integer
 *   speed) triples.  Each triple describes one rotating arm.  When
 *   the arms are chained tip-to-base and the global clock φ ticks
 *   forward, the very last tip traces the original path exactly.
 *
 *   We give the user the ability to FEED the recipe machine their
 *   own scribble and SEE the recipe play back.  The educational
 *   payoff is: "I drew this with five seconds of arrow-key presses
 *   and the program turned it into circles within a millisecond."
 *
 * HOW TO THINK ABOUT IT
 *   Imagine a sheet of graph paper, a pencil that can only spin in
 *   integer-revolution-per-cycle circles, and a robot arm that can
 *   stack pencils.  You hand the robot a closed loop you've drawn.
 *   It thinks for a moment, and then says: "OK — I need a ten-cm
 *   circle spinning once per cycle, a four-cm circle on the tip of
 *   that one spinning twice per cycle, a two-cm circle on the tip of
 *   that one spinning three times per cycle, ..."  Stack the circles
 *   end to end, give them all a starting angle, and let φ advance.
 *   The pen at the very tip retraces your loop.
 *
 *   The DFT is the robot's "thinking" step.  It's an O(N²) sum that
 *   happens once when you press ENTER.  Everything else is just
 *   spinning the circles.
 *
 *      ┌──────────────────────────────────────────────────────────┐
 *      │                                                          │
 *      │   DRAW mode                  PLAY mode                   │
 *      │  ┌─────────────┐            ┌─────────────────────────┐  │
 *      │  │   *****     │            │       o   o             │  │
 *      │  │  *  @  *    │            │     o       o           │  │
 *      │  │  *     * ▶ENTER ▶        │   o     +     o   ←chain│  │
 *      │  │  *     *    │            │     o       o   spinning│  │
 *      │  │   *****     │            │       o   o             │  │
 *      │  │ pts:42      │            │ arms:8/256              │  │
 *      │  └─────────────┘            └─────────────────────────┘  │
 *      │                                                          │
 *      │   you traced this           machine reconstructs         │
 *      │                                                          │
 *      └──────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 *
 *   DRAW mode (one tick of the loop):
 *     1. Wait for an arrow / WASD key.
 *     2. Bump the cell-space cursor by ±1 in the chosen direction.
 *     3. Bresenham-fill the cells between old and new cursor positions
 *        in the visualisation grid (so the line on screen is continuous
 *        even if cells were skipped).
 *     4. Append (cursor_col · CELL_W, cursor_row · CELL_H) to the raw
 *        pixel-space point list (skipping if it duplicates the previous).
 *     5. Re-paint cells + cursor + HUD + hint.
 *
 *   ENTER (one-shot transition):
 *     6. If g_close_path_enabled, append a synthetic point equal to
 *        raw[0] so the path forms a closed loop.
 *     7. Compute cumulative chord-length arc[i] over the raw points.
 *     8. Resample N = 256 uniformly-arc-spaced points by linear
 *        interpolation between neighbouring raw points.
 *     9. Subtract the centroid; record max |z| as max_radius_pixels.
 *    10. Run O(N²) DFT with twiddle recurrence (see §3 / epicycles.c T6).
 *    11. Decode each bin into (amp = |Z[n]|/N, phase = arg(Z[n]),
 *        freq = signed n) and qsort by amp descending.
 *    12. Compute pixel scale so the reconstruction fits 40% of screen.
 *    13. Switch state to PLAY.
 *
 *   PLAY mode (one tick):
 *    14. (Optional) auto-add: bump g_active_epicycle_count by 1.
 *    15. Advance φ by 2π / CYCLE_FRAMES.  On wrap, clear the trail.
 *    16. Walk the active arms, accumulating joint positions from pivot.
 *    17. Push the final tip into the trail ring.
 *    18. Render layered scene (ghost + orbits + trail + chain + markers
 *        + debug + HUD).
 *
 * KEY FORMULAS
 *
 *   Cumulative chord length  arc[i] = arc[i-1] + √((Δx)² + (Δy)²)
 *
 *   Arc-length resample      s_k    = (k / N) · arc[n-1]
 *                            find j such that arc[j] ≤ s_k < arc[j+1]
 *                            t      = (s_k - arc[j]) / (arc[j+1] - arc[j])
 *                            p_k    = p[j] + t · (p[j+1] - p[j])
 *
 *   Centroid                 μ      = (1/N) Σ z[k]
 *
 *   DFT bin                  Z[n]   = Σ z[k] · exp(-2πi · n · k / N)
 *
 *   Per-arm decoding         amp_n  = |Z[n]| / N
 *                            phase_n= arg(Z[n])
 *                            freq_n = n if n ≤ N/2 else n - N
 *
 *   Tip position at φ        z(φ)   = Σ_i amp_i · exp(i·(freq_i·φ + phase_i))
 *
 *   Pixel scale              g_pixel_scale_per_unit
 *                            = 0.40 · min(pixel_w, pixel_h) / max_radius_pixels
 *
 *   Animation step           φ ← φ + 2π / CYCLE_FRAMES
 *
 *   Ghost projection         screen_x = (raw_x - centroid_x) · scale + pivot_x
 *                            screen_y = (raw_y - centroid_y) · scale + pivot_y
 *
 * EDGE CASES TO WATCH
 *   - draw_compute() refuses to play with fewer than 4 raw points; sets
 *     g_draw_error_too_few and stays in DRAW mode.  Trying to fit a
 *     curve to a one-cell scribble is meaningless.
 *
 *   - Open paths.  The DFT treats its input as one period of an
 *     infinitely-repeating signal.  If the user's path is open
 *     (raw[0] ≠ raw[n-1]), the implicit periodic extension has a step
 *     discontinuity at the wrap-around point and the chain "teleports"
 *     from end to start every cycle.  The 'o' toggle appends a synthetic
 *     point equal to raw[0] so the path becomes a closed loop.  In
 *     PLAY mode 'o' re-runs the DFT live so the user can see the seam
 *     appear and disappear (T4 covers this in detail).
 *
 *   - Total arc length is clamped to ≥ 0.001 to avoid divide-by-zero
 *     on a one-cell scribble.
 *
 *   - Repeated arrow presses at the same cell don't append duplicate
 *     raw points (the "skip if identical to last" guard).  Otherwise
 *     a zero-length chord segment would show up as arc[j+1] == arc[j]
 *     and break the linear-interpolation step.
 *
 *   - Bresenham fill in DRAW mode marks cells in g_drawn_cells[][]
 *     for visualisation only.  The raw point list still gets exactly
 *     one entry per keypress so the resampling input is unchanged
 *     by the visual fill.
 *
 *   - Ghost overlay (toggle 'd', default ON) projects the raw points
 *     through the SAME (subtract centroid → scale → add pivot)
 *     transform as the reconstruction.  At full M they should overlap
 *     pixel-for-pixel; at low M they differ — exactly the convergence
 *     story we want to teach.
 *
 *   - Resize during PLAY recomputes the pixel scale from the saved
 *     g_max_radius_pixels so the reconstruction stays "40% of screen"
 *     through any number of resizes.
 *
 *   - '+' / '-' clear the trail because partial-arm previews would
 *     otherwise blend visually with the full-arm trail and confuse
 *     the reading.
 *
 * HOW TO VERIFY
 *   - Draw a small circle (~30 cells around).  After ENTER with
 *     M = 1 you should see a near-perfect ellipse (single arm).  By
 *     M = 3 the reconstruction overlays your ghost almost exactly.
 *
 *   - Draw a "+" cross — sharp corners need 50+ arms to crisp up;
 *     around M = 10 you'll see Gibbs ringing (oscillation near the
 *     cross-arms).  Same Fourier story as epicycles.c's Star.
 *
 *   - Draw an OPEN curve (e.g., the letter "U").  Without 'o' the
 *     chain has a visible jump from the open end back to the start
 *     each cycle.  Press 'o' — the jump disappears, replaced by a
 *     synthetic chord closing the loop.  The visual change is the
 *     point of the toggle.
 *
 *   - DRAW HUD shows pts:N/8192 and bbox:WxH growing as you draw.
 *     The bbox is in cells; verify it roughly matches the size of
 *     the drawing on screen.
 *
 *   - In PLAY, press '-' down to M = 1: only the largest arm rotates,
 *     tip traces an ellipse approximating your shape's overall size
 *     and orientation.  Press '+' a few times — detail piles on.
 *
 *   - Press 'd' to flip the ghost overlay off.  The trail then becomes
 *     the only curve visible; turn it back on to compare against your
 *     original.
 *
 *   - Press 'D' to show the numeric arm table.  The first row (i = 0)
 *     is the dominant arm — usually freq = +1 or -1 with the largest
 *     amplitude.  Verify the amp column decreases monotonically (the
 *     table is sorted descending).
 *
 *   - Auto-add cycles all 256 arms in 256 · 8 / 30 ≈ 68 seconds.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ──────────────────────────────────────────────────── *
 *
 *   T1   The two-mode pedagogy: DRAW and PLAY
 *   T2   Recording cursor presses into a raw point list
 *   T3   Arc-length resampling — the unique algorithm here
 *   T4   The closure problem — open paths and the 'o' toggle
 *   T5   From 2-D path to 1-D complex DFT
 *   T6   The arm chain (one-line summary; deep dive in epicycles.c)
 *   T7   Centring + scaling so the reconstruction always fits the screen
 *   T8   Pixel space vs cell space
 *   T9   The ghost overlay — proving the math works visually
 *   T10  Bresenham for visualisation continuity
 *   T11  Layer order and debug overlays
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE TWO-MODE PEDAGOGY: DRAW AND PLAY
 * ────────────────────────────────────────
 * Most Fourier demos pick a shape for you (a heart, a star, a
 * butterfly), run the DFT, and animate the chain.  That works as a
 * proof-of-concept but it leaves the user as a passive viewer.
 *
 * This file flips the script.  YOU draw the shape.  When you press
 * ENTER, the program proves to you that ITS algorithm works on YOUR
 * shape.  And then it asks: how few arms do you need before you
 * recognise your own drawing?  How many before it's perfect?
 *
 * The implementation is a state machine with two states:
 *
 *     STATE_DRAW              STATE_PLAY
 *     ─────────                ─────────
 *     Move cursor              Animate chain
 *     Record pixel points      Add/remove arms
 *     Fill visualisation       Toggle overlays
 *
 *     ─── ENTER (compute DFT) ───►
 *
 *     ◄─── 'r' (back to draw) ───
 *
 * The state variable `g_app_state` lives in §10.  The transition
 * happens in `draw_compute()` (§11) and on the 'r' key in §20.
 *
 * Because of this two-mode design, the algorithm core (§3 dft, §4
 * resample, §5 epicycle_table) sits between two thin UI shells: a
 * recorder on one side, a chain animator on the other.  Each shell
 * is small and replaceable.  If you wanted to feed the DFT a
 * signal from a microphone or from a parametric formula, you'd
 * write a new "shell" and reuse §3-§5 unchanged.
 *
 * T2  RECORDING CURSOR PRESSES INTO A RAW POINT LIST
 * ──────────────────────────────────────────────────
 * The recorder is a single function called for every arrow press:
 *
 *     move_cursor_with_bresenham_fill(dc, dr):
 *       new_col, new_row = clamp_to_screen(cur_col + dc, cur_row + dr)
 *       if (new == cur) return                        # no movement
 *       mark_bresenham_path_in_grid(cur, new)         # visual fill
 *       cur = new
 *       record_current_cell_as_raw_point()            # data for DFT
 *
 * Two outputs per move:
 *   1. The visualisation grid `g_drawn_cells[row][col]` gets cells
 *      filled in along the path from old cursor to new cursor (so
 *      the on-screen line is continuous even if successive cursor
 *      events skipped some cells).
 *   2. The raw pixel-space point list `g_raw_pixel_x/y` gets ONE
 *      new entry equal to the new cursor's pixel coordinates.  We
 *      don't add intermediate points to the raw list, because
 *      arc-length resampling (T3) handles arbitrary chord lengths
 *      gracefully — feeding it more samples would just repeat work.
 *
 * The "skip if duplicates the previous" guard exists to protect the
 * arc-length step from zero-length chord segments (which would
 * divide by zero).
 *
 * RAW_MAX = 8192 is the upper bound on raw points.  At one point
 * per arrow press, that's room for ~8000 keystrokes before the
 * recorder silently stops appending.  The HUD shows pts:N/8192 so
 * the user sees the count grow.
 *
 * T3  ARC-LENGTH RESAMPLING — THE UNIQUE ALGORITHM HERE
 * ─────────────────────────────────────────────────────
 * This is the single piece of code that exists nowhere else in this
 * project, and it's worth dwelling on.
 *
 * The DFT consumes EVENLY-SAMPLED data.  But the user's drawing is
 * not evenly sampled — it has one raw point per arrow press, and the
 * user might press fast in some places and slow in others.  Density
 * along the path is therefore non-uniform.  If we fed those raw
 * points straight into the DFT, the resulting spectrum would be
 * BIASED toward whatever direction the user drew slowly: the
 * algorithm would think the path "spent more time" there.
 *
 * The fix is ARC-LENGTH RESAMPLING.  The idea: instead of "one
 * sample every key-press", produce "one sample every fixed fraction
 * of total path length".  The mathematical recipe:
 *
 *   1. Compute cumulative chord lengths.  arc[0] = 0; for i ≥ 1,
 *      arc[i] = arc[i-1] + √((x[i]-x[i-1])² + (y[i]-y[i-1])²).
 *      arc[i] is the distance from the start to point i along the
 *      polyline.  arc[n-1] is the total path length.
 *
 *   2. For k = 0..N-1, pick the target arc length s_k = (k/N) · arc[n-1].
 *      This places N points evenly along the path, regardless of
 *      raw-point density.
 *
 *   3. For each s_k, find the polyline segment [j, j+1] containing
 *      it (i.e., arc[j] ≤ s_k < arc[j+1]).  Linearly interpolate:
 *
 *        t   = (s_k - arc[j]) / (arc[j+1] - arc[j])
 *        p_k = p[j] + t · (p[j+1] - p[j])
 *
 *      This gives the (x, y) of the new uniform sample point.
 *
 *   4. Repeat for k = 0..N-1.  Result: N points, evenly spaced along
 *      the original polyline by arc-length.
 *
 * Worked example with N = 5 and a 3-segment polyline:
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │  raw points:        p0 ───── p1 ────── p2 ── p3         │
 *      │  arc lengths:        0       3        7    8.5          │
 *      │  total = 8.5                                            │
 *      │                                                         │
 *      │  resample at fractions k/N = 0, 0.2, 0.4, 0.6, 0.8      │
 *      │  s_k                = 0   1.7  3.4  5.1  6.8            │
 *      │                                                         │
 *      │  s = 0    → segment p0–p1, t=0     → exactly p0          │
 *      │  s = 1.7  → segment p0–p1, t=0.57  → 57% along p0–p1    │
 *      │  s = 3.4  → segment p1–p2, t=0.10  → 10% along p1–p2    │
 *      │  s = 5.1  → segment p1–p2, t=0.53  → 53% along p1–p2    │
 *      │  s = 6.8  → segment p1–p2, t=0.95  → 95% along p1–p2    │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * Note how the resampled points cluster in the middle (longest)
 * segment — exactly what we want, because that segment carries the
 * most arc length.  The original raw points might have been
 * clustered near p0 (slow drawing) or near p3 (fast drawing); the
 * resampled output ignores that bias entirely.
 *
 * Complexity: O(n + N) since the inner search is monotone (j only
 * advances, never resets).  For our sizes (n ≤ 8192, N = 256) it's
 * ~8500 floating-point ops, dominated by the sqrt's in step 1.
 *
 * The implementation is in §4.  After resampling we also subtract
 * the path's centroid (mean of the resampled samples) so that the
 * shape sits at the origin in complex space — this avoids the DC
 * bin Z[0] dominating everything (T7 picks this thread back up).
 *
 * T4  THE CLOSURE PROBLEM — OPEN PATHS AND THE 'O' TOGGLE
 * ───────────────────────────────────────────────────────
 * The DFT is built on the assumption that its input is one period of
 * a periodic signal.  If your input does not naturally repeat
 * (sample[0] ≠ sample[N-1]'s neighbour), the implicit periodic
 * extension has a step discontinuity at the wrap-around point —
 * sample[N-1] jumps to sample[0] every cycle.  This is the SAME
 * issue that makes window functions necessary in fft_vis.c (T10).
 *
 * Here, the symptom is not just spectral leakage.  The chain
 * animation shows the discontinuity LITERALLY — at the moment φ
 * wraps to 0, the tip of the chain moves to the location of
 * sample[0] in one frame, regardless of where it was a frame ago.
 * Visually it looks like a teleport.  An ugly one.
 *
 * The fix: append one synthetic point equal to sample[0] at the end
 * of the raw list.  Now the path is a closed loop and the periodic
 * extension is smooth.  The toggle key is 'o'; the boolean is
 * `g_close_path_enabled`.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │  closed:N (open path "U")        closed:Y                │
 *      │                                                         │
 *      │   .─────────.                     .─────────.           │
 *      │   |        |                     |         |            │
 *      │   |        |   ◄ teleport       |         |   ◄ smooth  │
 *      │   *─        *─        each       *─       ─*            │
 *      │     ↓                cycle         \       /            │
 *      │     ↓                               \     /             │
 *      │   start                              \---/              │
 *      │                                  synthetic closing seg  │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * In DRAW mode 'o' just flips the boolean; the new value applies the
 * next time the user presses ENTER.  In PLAY mode 'o' flips the
 * boolean AND re-runs draw_compute() immediately so the user can
 * watch the seam appear and disappear without re-entering DRAW mode.
 * That live A/B is the most direct way to see what closure does.
 *
 * T5  FROM 2-D PATH TO 1-D COMPLEX DFT
 * ────────────────────────────────────
 * The classical DFT is defined on a sequence of complex numbers.
 * Our raw data is pairs of floats (x, y).  These are the same thing:
 * pack each (x, y) as the complex number x + i·y and the DFT of the
 * complex sequence is exactly the 2-D Fourier decomposition of the
 * curve.
 *
 *     z[k] = x[k] + i·y[k]
 *     Z[n] = Σ_k z[k] · exp(-2πi · n · k / N)
 *
 * No 2-D DFT, no tensor products — just the textbook 1-D formula
 * applied to a complex-valued input.  Because the DFT is linear,
 * the real and imaginary parts of Z[n] capture the x and y components
 * of the n-th rotation respectively.
 *
 * Each output bin Z[n] decodes into one rotating arm:
 *
 *     amp_n   = |Z[n]| / N      (radius in normalised units)
 *     phase_n = arg(Z[n])       (starting angle)
 *     freq_n  = (n ≤ N/2) ? n : n - N    (signed integer rotation rate)
 *
 * The signed-frequency remap is identical to epicycles.c T8: bins in
 * the upper half of the DFT output represent NEGATIVE frequencies
 * (clockwise rotations).  Without the remap, all arms would spin
 * the same direction and reconstruction would come out mirrored.
 *
 * T6  THE ARM CHAIN
 * ─────────────────
 * Per frame: φ advances by 2π / CYCLE_FRAMES.  Walk arms 0..M-1,
 * accumulating tip position from the screen pivot:
 *
 *     joint[0] = pivot
 *     for i in 0..M-1:
 *       angle = freq_i · φ + phase_i
 *       dx    = amp_i · scale · cos(angle)
 *       dy    = amp_i · scale · sin(angle)
 *       joint[i+1] = joint[i] + (dx, dy)
 *
 * `joint[M]` is the final tip; we push it into the trail ring buffer.
 *
 * For the deep dive on why this animation produces the original
 * curve at any φ, see signal/epicycles.c T7 and T8.  Same maths,
 * same code shape, different driver.
 *
 * T7  CENTRING + SCALING SO THE RECONSTRUCTION ALWAYS FITS THE SCREEN
 * ───────────────────────────────────────────────────────────────────
 * Two related but separate concerns:
 *
 *   CENTRING — after resampling we subtract the centroid (mean) of
 *   the samples.  This makes the shape sit at the origin in complex
 *   coordinates.  Why?  Because the DC bin Z[0] is N times the
 *   centroid; if the centroid isn't at the origin, Z[0] is huge and
 *   becomes the dominant arm — except it doesn't rotate (it's
 *   frequency 0), so it just shifts the whole chain by a constant.
 *   We don't WANT a stationary giant arm; we want the rotating arms
 *   to do the work.  Centring eliminates the issue at the source.
 *
 *   SCALING — the centred path lives in pixel coordinates whose range
 *   depends on how big the user drew.  We need it to fit roughly
 *   40% of the screen so the reconstruction is visible without
 *   filling the whole window.  The recipe:
 *
 *     screen_r = 0.40 · min(screen_pixel_w, screen_pixel_h)
 *     scale    = screen_r / max_radius_pixels
 *
 *   where `max_radius_pixels` is the maximum |z| of the centred path.
 *   This ensures the most-distant point on the curve sits at exactly
 *   40% of the smaller screen dimension.  The chain is then rendered
 *   from the screen centre, scaled by `scale`.
 *
 *   This whole scheme runs in `draw_compute()` (§11).  Pivot stays
 *   at screen centre; if the terminal resizes, we recompute pivot +
 *   scale from the saved `g_max_radius_pixels` so the reconstruction
 *   stays "40% of screen" through any number of resizes.
 *
 * T8  PIXEL SPACE VS CELL SPACE
 * ─────────────────────────────
 * The screen has cells (rows × cols).  Cells are about TWICE AS TALL
 * AS WIDE in a typical monospace font, which means a chain rendered
 * in cell coordinates would look horizontally squashed.
 *
 * We dodge this by running the chain math in PIXEL space — sub-cell
 * coordinates with CELL_W = 8 sub-pixels per column and CELL_H = 16
 * per row.  Conversion to cell space happens at one point per draw
 * call:
 *
 *     px_to_cell_x(px) = (int)(px / CELL_W + 0.5)
 *     px_to_cell_y(py) = (int)(py / CELL_H + 0.5)
 *
 * Because CELL_H / CELL_W = 2, a pixel-space CIRCLE of radius r
 * draws as a cell-space ELLIPSE with semi-axes (r/8, r/16) — a
 * vertical squash by 2 — but the eye perceives it as round because
 * the cells themselves are tall.  Net effect: circles look round.
 *
 * Cursor moves in DRAW mode are in CELL space (one cell per arrow
 * press).  But the recorded raw points are converted to pixel space
 * (cur_col · CELL_W, cur_row · CELL_H) so all subsequent maths
 * (resampling, DFT, animation) stays in pixel units.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │   one cell = 8 px wide × 16 px tall                     │
 *      │                                                         │
 *      │   ┌────────┐ ←── 8 pixels wide                          │
 *      │   │        │                                            │
 *      │   │        │ ←── 16 pixels tall                         │
 *      │   │        │                                            │
 *      │   └────────┘                                            │
 *      │                                                         │
 *      │   pixel (x, y)              cell (col, row)             │
 *      │   continuous, no grid       integer indices              │
 *      │   chain math here           draws here                  │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * T9  THE GHOST OVERLAY — PROVING THE MATH WORKS VISUALLY
 * ───────────────────────────────────────────────────────
 * When you press ENTER, your drawing has been transformed twice
 * before the chain renders:
 *
 *   1. Centroid subtracted → centred at origin in pixel space.
 *   2. Multiplied by `scale` and added `pivot` → centred at the
 *      screen middle, sized to fit 40% of screen.
 *
 * The chain's tip operates in this transformed coordinate system.
 * If we want to compare what you drew with what the chain produces,
 * we need to render your ORIGINAL raw points in the SAME transformed
 * system.  That's what `project_raw_pixel_to_cell()` (§13) does:
 *
 *     screen_x = (raw_x - centroid_x) · scale + pivot_x
 *     screen_y = (raw_y - centroid_y) · scale + pivot_y
 *
 * Apply this to every raw point and stamp '.' at the resulting cell.
 * The result is a faint dotted underlay of your drawing in exactly
 * the same place where the chain is rendering.  When M is small, the
 * chain trail will diverge from the ghost; as you raise M, they
 * converge.  At M = N (full chain) they overlap pixel-for-pixel.
 *
 * This is the most visually convincing demonstration of "Fourier
 * reconstruction works".  You're not just told it works; you're
 * watching it happen on top of your own drawing.
 *
 * Toggle 'd' turns the ghost on and off.  Default is ON, because
 * the comparison is the point of this file.
 *
 * T10  BRESENHAM FOR VISUALISATION CONTINUITY
 * ───────────────────────────────────────────
 * Cursor moves in DRAW mode are 1 cell per arrow press.  In a
 * perfect world, every press would advance to a single adjacent
 * cell, all cells along the cursor's path would be marked, and the
 * resulting on-screen line would be continuous.
 *
 * In the real world, terminal key-repeat can fire multiple events
 * per main-loop iteration; on slow systems those events get drained
 * in one input pass, so the cursor advances 2-3 cells in a single
 * tick.  Without intervention the visualisation grid would have
 * gaps.
 *
 * Bresenham's line algorithm (Foley & van Dam §3.2.1) draws an
 * integer-pixel line between two endpoints in O(steps) time using
 * only addition and subtraction.  We use it twice:
 *
 *   - `mark_bresenham_path_in_grid(x0, y0, x1, y1)` — fills cells
 *     in the visualisation grid g_drawn_cells[][] without writing
 *     to the screen.  Used in DRAW mode for cursor movement.
 *
 *   - `render_bresenham_line(x0, y0, x1, y1, attr)` — same algorithm
 *     but writes glyphs ('-', '|', '/', '\') chosen by slope to
 *     stdscr.  Used in PLAY mode for chain segments.
 *
 * Critical detail: `mark_bresenham_path_in_grid` does NOT touch the
 * raw point list.  We add only ONE raw point per keypress, so
 * arc-length resampling sees the user's actual key-press cadence,
 * not interpolated extras.  The Bresenham fill is purely cosmetic.
 *
 * T11  LAYER ORDER AND DEBUG OVERLAYS
 * ───────────────────────────────────
 * PLAY-mode rendering paints six layers in back-to-front order.
 * Each layer overwrites the previous in any cells they share —
 * "last writer wins" per cell, no blending.  Order matters because
 * we want the most informative content on top:
 *
 *   1. GHOST overlay      faint '.' at projected raw points
 *   2. ORBIT circles      faint '.' at orbit guide ellipses
 *   3. TRAIL              fading '*' from oldest to newest tip path
 *   4. ARM CHAIN          Bresenham '-', '|', '/', '\' along arms
 *   5. MARKERS            '@' at tip, '+' at pivot
 *   6. DEBUG arm table    'D' overlay top-left if enabled
 *   7. HUD                always last; status top-right, hint bottom
 *
 * This ordering means the bright tip marker '@' always wins over
 * the trail it just left; the arm chain is visible against both
 * trail and ghost; the ghost recedes whenever any other layer
 * touches its cell.
 *
 * Two debug overlays:
 *
 *   'd' — ghost overlay (T9).  Default ON.  Faint dotted underlay
 *         of the original raw path so you can verify reconstruction.
 *
 *   'D' — arm table.  Numeric panel top-left listing the eight
 *         loudest arms with (i, signed freq, amp, phase).  Useful
 *         for verifying DFT decode by hand or for picking out which
 *         arms drive the rough-shape vs which contribute fine detail.
 *
 * Both overlays read algorithm state without modifying it; they
 * cost a few hundred mvaddch calls per frame and have no impact on
 * the simulation.
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

/* ===================================================================== */
/* §1  config — every constant + enum lives here                         */
/* ===================================================================== */
/*
 * Naming convention: SCREAMING_SNAKE_CASE for #defines.  Units are
 * spelled in the constant name when not obvious from context.  When
 * you see a bare number elsewhere in the file, ask whether it should
 * be promoted to a named constant here.
 */

/* §1.1 — DFT + path sizing ------------------------------------------ */

#define N_SAMPLES                256    /* DFT resolution = max epicycles */
#define RAW_MAX                 8192    /* upper bound on raw cursor points */

/* §1.2 — animation + frame rate ------------------------------------- */

#define RENDER_FPS                30
#define RENDER_TICK_NS    (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES             300    /* frames per full reconstruction cycle */
#define AUTO_ADD_FRAMES            8    /* frames between auto-adding one arm */

/* §1.3 — trail + visual layout -------------------------------------- */

#define TRAIL_LEN                600    /* tip-path ring buffer length */
#define N_CIRCLES_DRAWN            6    /* orbit circles drawn (largest arms) */
#define ARM_TABLE_ROWS             8    /* 'D' overlay top-N arm count */

/* §1.4 — terminal cell aspect (sub-pixels per cell) ----------------- */
/*
 * Standard monospace fonts: each cell is roughly twice as tall as
 * wide.  We encode that 1:2 aspect here so the chain math runs in
 * pixel space without horizontal squash.
 */
#define CELL_W                     8
#define CELL_H                    16

/* §1.5 — visualisation grid bounds ---------------------------------- */

#define ROWS_MAX                 128    /* upper bound on g_drawn_cells rows */
#define COLS_MAX                 512    /* upper bound on g_drawn_cells cols */

/* §1.6 — themes ----------------------------------------------------- */

#define N_THEMES                   5

/* §1.7 — colour pair IDs (bound by §7) ------------------------------ */
/*
 * Animation pairs (1..11) are theme-cycled by apply_theme().  HUD,
 * HINT, GHOST pairs are theme-INVARIANT per CLAUDE.md HUD Standard
 * so the status bar and the comparison ghost stay legible against
 * any animation colour.
 */

enum {
    PAIR_ARM_HI    = 1,    /* large arm                             */
    PAIR_ARM_MID,          /* medium arm                            */
    PAIR_ARM_LO,           /* tiny arm                              */
    PAIR_CIRCLE,           /* orbit-guide ellipses                  */
    PAIR_TRAIL_NEW,        /* trail newest                          */
    PAIR_TRAIL_MID,        /* trail middle                          */
    PAIR_TRAIL_OLD,        /* trail oldest                          */
    PAIR_TIP,              /* tip '@' marker                        */
    PAIR_PIVOT,            /* centre '+' marker                     */
    PAIR_CURSOR,           /* DRAW-mode cursor '@'                  */
    PAIR_PATH,             /* DRAW-mode path cells                  */
    PAIR_HUD,              /* HUD status — invariant bright yellow  */
    PAIR_HINT,             /* hint bottom — invariant bright cyan   */
    PAIR_GHOST,            /* ghost overlay — invariant medium grey */
};

/* ===================================================================== */
/* §2  clock — monotonic ns timer + nanosecond sleep                     */
/* ===================================================================== */

static long long clock_now_ns(void)
{
    /* Purpose         : return the current monotonic clock in nanoseconds.
     * Mental model    : a wall clock that only moves forward, never
     *                   resets (CLOCK_MONOTONIC is immune to NTP and DST).
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
     * Why it exists : without this the main loop would spin a CPU at
     *                 100% just to repeat the FFT 30,000 times a sec. */
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  dft — O(N²) DFT with twiddle recurrence                           */
/* ===================================================================== */

typedef struct { float re, im; } Cplx;

/*
 * compute_dft_with_twiddle — direct DFT, but the inner loop avoids
 * cos/sin calls by maintaining the running power of a "twiddle factor"
 * W = exp(-2πi · n / N) incrementally via complex multiplication.
 * Trig calls per output bin: 2 (one cos + one sin to set W up).  Total
 * trig calls per DFT: 2N, vs 2N² for the naive version.  At N = 256
 * that's 512 vs 131,072.
 *
 * For the deep derivation see signal/epicycles.c T6 or signal/fft_vis.c
 * T6.  This file just uses it.
 */
static void compute_dft_with_twiddle(const Cplx *input,
                                     Cplx *output,
                                     int N)
{
    /* Purpose         : compute the discrete Fourier transform.
     * Pseudocode      :
     *     for n in 0..N-1:
     *       W = exp(-i · 2π · n / N)
     *       twiddle = 1
     *       acc = 0
     *       for k in 0..N-1:
     *         acc     += in[k] · twiddle
     *         twiddle *= W
     *       out[n] = acc
     * Mental model    : N parallel "lock-in detectors", each tuned to
     *                   one integer harmonic.  Largest output where the
     *                   input contains energy at that harmonic.
     * Inputs / outputs: const input array, output array, length / fills out[].
     * Why it exists   : runs once per draw_compute() call (ENTER or 'o').
     *                   Not in the per-frame hot path.
     */
    for (int n = 0; n < N; n++) {
        float twiddle_step_re = cosf(-2.f * (float)M_PI * (float)n / (float)N);
        float twiddle_step_im = sinf(-2.f * (float)M_PI * (float)n / (float)N);
        float twiddle_re      = 1.f, twiddle_im = 0.f;
        float acc_re          = 0.f, acc_im     = 0.f;

        for (int k = 0; k < N; k++) {
            acc_re += input[k].re * twiddle_re - input[k].im * twiddle_im;
            acc_im += input[k].re * twiddle_im + input[k].im * twiddle_re;
            float next_twiddle_re = twiddle_re * twiddle_step_re
                                  - twiddle_im * twiddle_step_im;
            twiddle_im            = twiddle_re * twiddle_step_im
                                  + twiddle_im * twiddle_step_re;
            twiddle_re            = next_twiddle_re;
        }
        output[n].re = acc_re;
        output[n].im = acc_im;
    }
}

/* ===================================================================== */
/* §4  resample — arc-length resampling (the unique algorithm here)      */
/* ===================================================================== */
/*
 * See T3 for the conceptual derivation.  Implementation notes:
 *
 *   - The arc[] cumulative chord-length array is malloc'd on the stack
 *     of the heap (caller's choice via this function); we use the heap
 *     because n can be up to RAW_MAX = 8192 and 32 KB on the stack
 *     is plausible-but-not-great.
 *
 *   - The inner loop is monotone in `j`: we never reset j, just advance
 *     it as s_k grows.  Total inner iterations across all k is bounded
 *     by O(n + N), so the function is linear-time after the sqrt'd
 *     arc-length pass.
 *
 *   - On a degenerate one-cell scribble (all raw points identical),
 *     total = 0.  We clamp it to 1.0 to avoid divide-by-zero; the
 *     resulting samples will all collapse to a single point and the
 *     DFT will produce a Z[0] = N · point, all other bins zero.
 */
static float resample_path_to_uniform_arc_length(const float *raw_pixel_x,
                                                 const float *raw_pixel_y,
                                                 int          raw_point_count,
                                                 Cplx        *output_samples,
                                                 int          output_count,
                                                 float       *out_centroid_x,
                                                 float       *out_centroid_y)
{
    /* Purpose         : resample a polyline to N uniformly arc-length-
     *                   spaced points; subtract centroid; return max |z|.
     * Pseudocode      :
     *     arc[0] = 0
     *     for i in 1..n-1: arc[i] = arc[i-1] + |Δp[i]|
     *     total = arc[n-1]
     *
     *     j = 0
     *     for k in 0..N-1:
     *       s_k = (k/N) · total
     *       while arc[j+1] < s_k: j++
     *       t   = (s_k - arc[j]) / (arc[j+1] - arc[j])
     *       p_k = p[j] + t · (p[j+1] - p[j])
     *       out[k] = p_k
     *
     *     centroid = mean(out)
     *     subtract centroid; record max |z|
     *
     * Mental model    : a "chord-length yardstick" walks along the
     *                   polyline taking samples every (total / N)
     *                   units of distance.
     * Inputs / outputs: raw points + count, output array + count,
     *                   centroid out-pointers / fills output, returns
     *                   max radius for screen-fit scaling.
     * Why it exists   : ensures the DFT receives uniform-time samples
     *                   regardless of how fast the user drew (T3).
     */
    float *arc = malloc((size_t)raw_point_count * sizeof(float));
    if (!arc) return 1.f;

    /* Cumulative chord lengths. */
    arc[0] = 0.f;
    for (int i = 1; i < raw_point_count; i++) {
        float dx = raw_pixel_x[i] - raw_pixel_x[i - 1];
        float dy = raw_pixel_y[i] - raw_pixel_y[i - 1];
        arc[i] = arc[i - 1] + sqrtf(dx * dx + dy * dy);
    }
    float total = arc[raw_point_count - 1];
    if (total < 0.001f) total = 1.f;   /* one-cell scribble guard */

    /* Resample at uniform arc-length steps via linear interpolation. */
    int j = 0;
    for (int k = 0; k < output_count; k++) {
        float s_k = (float)k / (float)output_count * total;
        while (j < raw_point_count - 2 && arc[j + 1] < s_k) j++;
        float span = arc[j + 1] - arc[j];
        float t    = (span > 0.001f) ? (s_k - arc[j]) / span : 0.f;
        output_samples[k].re = raw_pixel_x[j]
                             + t * (raw_pixel_x[j + 1] - raw_pixel_x[j]);
        output_samples[k].im = raw_pixel_y[j]
                             + t * (raw_pixel_y[j + 1] - raw_pixel_y[j]);
    }
    free(arc);

    /* Compute centroid; record for caller; subtract from samples. */
    float mean_re = 0.f, mean_im = 0.f;
    for (int k = 0; k < output_count; k++) {
        mean_re += output_samples[k].re;
        mean_im += output_samples[k].im;
    }
    mean_re /= (float)output_count;
    mean_im /= (float)output_count;

    if (out_centroid_x) *out_centroid_x = mean_re;
    if (out_centroid_y) *out_centroid_y = mean_im;

    float max_radius = 0.f;
    for (int k = 0; k < output_count; k++) {
        output_samples[k].re -= mean_re;
        output_samples[k].im -= mean_im;
        float r = sqrtf(output_samples[k].re * output_samples[k].re
                      + output_samples[k].im * output_samples[k].im);
        if (r > max_radius) max_radius = r;
    }
    return (max_radius > 0.001f) ? max_radius : 1.f;
}

/* ===================================================================== */
/* §5  epicycle_table — decode + sort                                    */
/* ===================================================================== */

typedef struct {
    float amplitude_normalised;    /* |Z[n]| / N (unitless, in [0, 1])     */
    float phase_offset_radians;    /* arg(Z[n])                             */
    int   frequency_signed;        /* signed integer harmonic (T5)          */
} Epicycle;

static int epicycle_compare_descending(const void *a, const void *b)
{
    /* qsort comparator: amplitude descending. */
    float aa = ((const Epicycle *)a)->amplitude_normalised;
    float bb = ((const Epicycle *)b)->amplitude_normalised;
    return (aa < bb) - (aa > bb);
}

static Epicycle g_epicycle_table[N_SAMPLES];
static int      g_total_epicycle_count  = 0;
static int      g_active_epicycle_count = 0;

static void build_sorted_epicycle_table(const Cplx *samples, int N)
{
    /* Purpose         : run DFT, decode each bin, qsort by amp desc.
     * Pseudocode      :
     *     dft = compute_dft_with_twiddle(samples)
     *     for n in 0..N-1:
     *       table[n] = (|Z[n]|/N, arg(Z[n]), signed_freq(n))
     *     qsort(table, descending by amplitude)
     * Mental model    : turn the raw DFT output into a sorted catalog
     *                   of arms, dominant arm first.
     * Why it exists   : called from draw_compute() (§11).  Output
     *                   feeds the per-frame chain walker in §12.
     */
    Cplx dft[N_SAMPLES];
    compute_dft_with_twiddle(samples, dft, N);
    float inv_N = 1.f / (float)N;
    for (int n = 0; n < N; n++) {
        Epicycle e;
        e.amplitude_normalised = sqrtf(dft[n].re * dft[n].re
                                     + dft[n].im * dft[n].im) * inv_N;
        e.phase_offset_radians = atan2f(dft[n].im, dft[n].re);
        /* Signed-frequency remap: bins above N/2 represent NEGATIVE
         * frequencies (clockwise rotations) — see epicycles.c T8. */
        e.frequency_signed     = (n <= N / 2) ? n : n - N;
        g_epicycle_table[n] = e;
    }
    qsort(g_epicycle_table, N, sizeof(Epicycle),
          epicycle_compare_descending);
    g_total_epicycle_count  = N;
    g_active_epicycle_count = 1;
}

/* ===================================================================== */
/* §6  themes — palette table for the live colour-cycler                 */
/* ===================================================================== */
/*
 * Five palettes, eleven animation pairs each.  HUD, HINT, GHOST pairs
 * are theme-invariant and bound separately in §7 colors_init.
 *
 * All entries chosen from the BRIGHT half of the 256-colour cube
 * (≥ 24 in the cube, ≥ 240 in greyscale).  CLAUDE.md flags codes 16-23
 * and 232-239 as invisible against default-black; the original file
 * had several such entries that this rebuild replaced.
 *
 * Each theme also carries 8-colour fallbacks for terminals without
 * extended palette support.
 */

typedef struct {
    /* 256-colour foreground codes for each animation role */
    short arm_hi, arm_mid, arm_lo;
    short circle, trail_new, trail_mid, trail_old;
    short tip, pivot, cursor, path;
    /* 8-colour fallbacks (same roles, indices 0..7) */
    short arm_hi8, arm_mid8, arm_lo8;
    short circle8, trail_new8, trail_mid8, trail_old8;
    short tip8, pivot8, cursor8, path8;
    const char *display_name;
} ThemePalette;

static const ThemePalette theme_table[N_THEMES] = {
    /* Classic — bright white arms, gold/orange/red trail */
    { 231,  51, 245,   244,  226, 208, 196,   231, 220, 226, 250,
      COLOR_WHITE, COLOR_CYAN,   COLOR_WHITE,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE,
      "Classic" },
    /* Fire — red/orange arms, peach orbit guides */
    { 208, 196,  88,   215,  226, 214, 196,   231, 208, 226, 202,
      COLOR_RED,    COLOR_RED,    COLOR_RED,
      COLOR_WHITE,  COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
      COLOR_WHITE,  COLOR_RED,    COLOR_YELLOW, COLOR_RED,
      "Fire" },
    /* Neon — magenta/violet arms, lavender guides */
    { 207, 165,  93,   141,  219, 213, 201,   231, 207, 219, 201,
      COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE,   COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_WHITE,   COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      "Neon" },
    /* Ocean — teal/blue arms, steel-blue guides */
    { 123,  45,  31,   110,  159,  87,  51,   231,  45, 123,  39,
      COLOR_CYAN, COLOR_BLUE, COLOR_BLUE,
      COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
      COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
      "Ocean" },
    /* Matrix — green spectrum, olive-green guides */
    { 118,  46,  28,    64,  154, 118,  46,   231, 118, 154,  82,
      COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_WHITE, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      "Matrix" },
};

static int g_active_theme_index = 0;

/* ===================================================================== */
/* §7  colors — apply_theme + theme-invariant pair init                  */
/* ===================================================================== */

static void apply_theme(int theme_index)
{
    /* Purpose : rebind ONLY the eleven animation pairs to a theme's
     *           palette.  HUD/HINT/GHOST are theme-invariant.
     */
    if (theme_index < 0 || theme_index >= N_THEMES) theme_index = 0;
    const ThemePalette *t = &theme_table[theme_index];
    if (COLORS >= 256) {
        init_pair(PAIR_ARM_HI,    t->arm_hi,    -1);
        init_pair(PAIR_ARM_MID,   t->arm_mid,   -1);
        init_pair(PAIR_ARM_LO,    t->arm_lo,    -1);
        init_pair(PAIR_CIRCLE,    t->circle,    -1);
        init_pair(PAIR_TRAIL_NEW, t->trail_new, -1);
        init_pair(PAIR_TRAIL_MID, t->trail_mid, -1);
        init_pair(PAIR_TRAIL_OLD, t->trail_old, -1);
        init_pair(PAIR_TIP,       t->tip,       -1);
        init_pair(PAIR_PIVOT,     t->pivot,     -1);
        init_pair(PAIR_CURSOR,    t->cursor,    -1);
        init_pair(PAIR_PATH,      t->path,      -1);
    } else {
        init_pair(PAIR_ARM_HI,    t->arm_hi8,    -1);
        init_pair(PAIR_ARM_MID,   t->arm_mid8,   -1);
        init_pair(PAIR_ARM_LO,    t->arm_lo8,    -1);
        init_pair(PAIR_CIRCLE,    t->circle8,    -1);
        init_pair(PAIR_TRAIL_NEW, t->trail_new8, -1);
        init_pair(PAIR_TRAIL_MID, t->trail_mid8, -1);
        init_pair(PAIR_TRAIL_OLD, t->trail_old8, -1);
        init_pair(PAIR_TIP,       t->tip8,       -1);
        init_pair(PAIR_PIVOT,     t->pivot8,     -1);
        init_pair(PAIR_CURSOR,    t->cursor8,    -1);
        init_pair(PAIR_PATH,      t->path8,      -1);
    }
}

static void colors_init(void)
{
    /* Bind theme-invariant pairs once at startup, then apply default
     * theme.  Subsequent theme changes go through apply_theme and
     * leave HUD/HINT/GHOST alone. */
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,   51, -1);   /* bright cyan   */
        init_pair(PAIR_GHOST, 244, -1);   /* medium grey   */
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_GHOST, COLOR_WHITE,  -1);
    }
    apply_theme(0);
}

/* ===================================================================== */
/* §8  trail — tip-path ring buffer                                      */
/* ===================================================================== */

typedef struct {
    float pixel_x[TRAIL_LEN];
    float pixel_y[TRAIL_LEN];
    int   write_head;
    int   filled_count;
} TipTrail;

static TipTrail g_tip_trail;

static void trail_push(TipTrail *t, float pixel_x, float pixel_y)
{
    /* Append (x, y); advance head; saturate count at TRAIL_LEN. */
    t->pixel_x[t->write_head] = pixel_x;
    t->pixel_y[t->write_head] = pixel_y;
    t->write_head = (t->write_head + 1) % TRAIL_LEN;
    if (t->filled_count < TRAIL_LEN) t->filled_count++;
}

static void trail_clear(TipTrail *t)
{
    /* Logically empty without zeroing the data array. */
    t->write_head   = 0;
    t->filled_count = 0;
}

/* ===================================================================== */
/* §9  draw_primitives — px↔cell, Bresenham line, mark, orbit ellipse    */
/* ===================================================================== */

static int g_screen_rows = 0;     /* updated by main loop on resize */
static int g_screen_cols = 0;

static int px_to_cell_x(float pixel_x) { return (int)(pixel_x / CELL_W + 0.5f); }
static int px_to_cell_y(float pixel_y) { return (int)(pixel_y / CELL_H + 0.5f); }

/*
 * render_bresenham_line — draw a one-cell-wide line in cell space using
 * Bresenham's 1965 integer-only algorithm.  Glyph chosen by local slope:
 *   horizontal moves → '-'
 *   vertical   moves → '|'
 *   diagonals        → '/' or '\\' depending on direction
 * Clips against the viewport (rows-1 reserved for the hint line).
 */
static void render_bresenham_line(int x0, int y0, int x1, int y1, attr_t attr)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        if (x0 >= 0 && x0 < g_screen_cols
         && y0 >= 0 && y0 < g_screen_rows - 1) {
            int   e2 = 2 * err;
            bool  bx = e2 > -dy, by = e2 < dx;
            chtype glyph = (bx && by) ? (sx == sy ? '\\' : '/')
                         : bx ? '-' : '|';
            attron(attr);
            mvaddch(y0, x0, glyph);
            attroff(attr);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * mark_bresenham_path_in_grid — same Bresenham walk, but writes 1's
 * into the visualisation grid g_drawn_cells[][] instead of glyphs to
 * stdscr.  The DRAW-mode line stays continuous even when key-repeat
 * fires events that advance the cursor 2+ cells per drained event.
 *
 * Forward declaration — we need g_drawn_cells declared first; the
 * actual array lives in §10.
 */
static uint8_t g_drawn_cells[ROWS_MAX][COLS_MAX];

static void mark_bresenham_path_in_grid(int x0, int y0, int x1, int y1)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        if (y0 >= 0 && y0 < ROWS_MAX && x0 >= 0 && x0 < COLS_MAX)
            g_drawn_cells[y0][x0] = 1;
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * render_orbit_ellipse — draw the cell-space ellipse corresponding to
 * a pixel-space circle of radius `radius_pixels` centred at
 * (pivot_pixel_x, pivot_pixel_y).  See T8 for the cell-aspect story.
 *
 * Sample count proportional to circumference so we don't overdraw
 * tiny ellipses or under-draw big ones.
 */
static void render_orbit_ellipse(float pivot_pixel_x,
                                 float pivot_pixel_y,
                                 float radius_pixels)
{
    if (radius_pixels < (float)CELL_W * 0.5f) return;

    float semi_axis_x = radius_pixels / (float)CELL_W;
    float semi_axis_y = radius_pixels / (float)CELL_H;
    int   center_col  = px_to_cell_x(pivot_pixel_x);
    int   center_row  = px_to_cell_y(pivot_pixel_y);
    int   sample_count = (int)(2.f * (float)M_PI
                          * fmaxf(semi_axis_x, semi_axis_y)) + 4;

    attr_t attr = COLOR_PAIR(PAIR_CIRCLE);
    attron(attr);
    for (int i = 0; i < sample_count; i++) {
        float theta = 2.f * (float)M_PI * (float)i / (float)sample_count;
        int x = center_col + (int)(semi_axis_x * cosf(theta) + 0.5f);
        int y = center_row + (int)(semi_axis_y * sinf(theta) + 0.5f);
        if (x >= 0 && x < g_screen_cols
         && y >= 0 && y < g_screen_rows - 1)
            mvaddch(y, x, '.');
    }
    attroff(attr);
}

/* ===================================================================== */
/* §10  draw_state — DRAW-mode state, cursor, recording, bbox            */
/* ===================================================================== */

typedef enum { STATE_DRAW, STATE_PLAY } AppState;

static AppState g_app_state = STATE_DRAW;

/* Cursor in cell space (one cell per arrow press). */
static int     g_cursor_col = 0;
static int     g_cursor_row = 0;

/* Raw recorded path in pixel space (one entry per keypress). */
static float   g_raw_pixel_x[RAW_MAX];
static float   g_raw_pixel_y[RAW_MAX];
static int     g_raw_point_count = 0;

/* Visualisation grid: 1 if cell is on the recorded path.  Note the
 * underlying array is declared in §9 (we needed it for the forward
 * declaration of mark_bresenham_path_in_grid). */

static bool    g_draw_error_too_few = false;

static void reset_draw_state(void)
{
    /* Centre the cursor; clear all recordings. */
    g_cursor_col         = g_screen_cols / 2;
    g_cursor_row         = (g_screen_rows - 1) / 2;
    g_raw_point_count    = 0;
    g_draw_error_too_few = false;
    memset(g_drawn_cells, 0, sizeof g_drawn_cells);
}

static void record_current_cell_as_raw_point(void)
{
    /* Append the current cursor's pixel coordinates as a raw point,
     * unless it duplicates the previous (the duplicate guard protects
     * arc-length resampling from zero-length chord segments). */
    if (g_raw_point_count >= RAW_MAX) return;
    float pixel_x = (float)g_cursor_col * CELL_W;
    float pixel_y = (float)g_cursor_row * CELL_H;
    if (g_raw_point_count > 0
     && g_raw_pixel_x[g_raw_point_count - 1] == pixel_x
     && g_raw_pixel_y[g_raw_point_count - 1] == pixel_y)
        return;
    g_raw_pixel_x[g_raw_point_count] = pixel_x;
    g_raw_pixel_y[g_raw_point_count] = pixel_y;
    g_raw_point_count++;
    if (g_cursor_row >= 0 && g_cursor_row < ROWS_MAX
     && g_cursor_col >= 0 && g_cursor_col < COLS_MAX)
        g_drawn_cells[g_cursor_row][g_cursor_col] = 1;
}

static void move_cursor_with_bresenham_fill(int delta_col, int delta_row)
{
    /* Purpose : move cursor by (dc, dr); fill cells between old and
     *           new positions; record the new cursor as a raw point.
     * Mental model : the cursor leaves a trail.  Bresenham fill is the
     *                visual continuity (so fast key-repeat doesn't
     *                produce gaps); the raw point is the data the DFT
     *                will eventually consume.
     */
    int new_col = g_cursor_col + delta_col;
    int new_row = g_cursor_row + delta_row;
    if (new_col < 0)                  new_col = 0;
    if (new_col >= g_screen_cols)     new_col = g_screen_cols - 1;
    if (new_row < 0)                  new_row = 0;
    if (new_row >= g_screen_rows - 1) new_row = g_screen_rows - 2;
    if (new_col == g_cursor_col && new_row == g_cursor_row) return;

    mark_bresenham_path_in_grid(g_cursor_col, g_cursor_row, new_col, new_row);
    g_cursor_col = new_col;
    g_cursor_row = new_row;
    record_current_cell_as_raw_point();
}

static bool compute_bbox_in_cells(int *out_w_cells, int *out_h_cells)
{
    /* Cell-space bounding box of the raw path; used by the DRAW HUD. */
    if (g_raw_point_count == 0) {
        *out_w_cells = 0; *out_h_cells = 0; return false;
    }
    float min_x = g_raw_pixel_x[0], max_x = g_raw_pixel_x[0];
    float min_y = g_raw_pixel_y[0], max_y = g_raw_pixel_y[0];
    for (int i = 1; i < g_raw_point_count; i++) {
        if (g_raw_pixel_x[i] < min_x) min_x = g_raw_pixel_x[i];
        if (g_raw_pixel_x[i] > max_x) max_x = g_raw_pixel_x[i];
        if (g_raw_pixel_y[i] < min_y) min_y = g_raw_pixel_y[i];
        if (g_raw_pixel_y[i] > max_y) max_y = g_raw_pixel_y[i];
    }
    *out_w_cells = (int)((max_x - min_x) / CELL_W + 0.5f) + 1;
    *out_h_cells = (int)((max_y - min_y) / CELL_H + 0.5f) + 1;
    return true;
}

/* ===================================================================== */
/* §11  draw_compute — resample → build → set up scale & pivot           */
/* ===================================================================== */

/* PLAY-mode geometry, computed in draw_compute and used in §12-§14. */
static float g_screen_pivot_pixel_x   = 0.f;
static float g_screen_pivot_pixel_y   = 0.f;
static float g_pixel_scale_per_unit   = 1.f;
static float g_max_radius_pixels      = 1.f;
static float g_path_centroid_pixel_x  = 0.f;
static float g_path_centroid_pixel_y  = 0.f;

/* Per-frame chain joints (joint[0] = pivot; joint[i+1] = tip of arm i). */
static float g_joint_pixel_x[N_SAMPLES + 1];
static float g_joint_pixel_y[N_SAMPLES + 1];

/* User toggles. */
static bool  g_close_path_enabled = false;

static bool switch_from_draw_to_play(void)
{
    /* Purpose : the ENTER-key transition.  Resample, DFT, sort, set
     *           up screen geometry, switch state.
     * Returns false (and stays in DRAW) if the path has too few points.
     */
    if (g_raw_point_count < 4) {
        g_draw_error_too_few = true;
        return false;
    }
    g_draw_error_too_few = false;

    /* Working copy in case we need to append a closing point.  Keeps
     * the user's raw path untouched so toggling 'o' is reversible. */
    static float local_pixel_x[RAW_MAX + 1];
    static float local_pixel_y[RAW_MAX + 1];
    int n = g_raw_point_count;
    memcpy(local_pixel_x, g_raw_pixel_x, (size_t)n * sizeof(float));
    memcpy(local_pixel_y, g_raw_pixel_y, (size_t)n * sizeof(float));
    if (g_close_path_enabled) {
        local_pixel_x[n] = local_pixel_x[0];
        local_pixel_y[n] = local_pixel_y[0];
        n++;
    }

    /* Resample (T3) → centroid + max_radius out-pointers. */
    Cplx samples[N_SAMPLES];
    g_max_radius_pixels = resample_path_to_uniform_arc_length(
        local_pixel_x, local_pixel_y, n,
        samples, N_SAMPLES,
        &g_path_centroid_pixel_x, &g_path_centroid_pixel_y);

    /* DFT → sorted arm table (T5, T6). */
    build_sorted_epicycle_table(samples, N_SAMPLES);

    /* Pivot at screen centre; scale fits 40% of screen (T7). */
    g_screen_pivot_pixel_x = (float)g_screen_cols * CELL_W * 0.5f;
    g_screen_pivot_pixel_y = (float)(g_screen_rows - 1) * CELL_H * 0.5f;
    float screen_radius_pixels =
        fminf((float)g_screen_cols * CELL_W,
              (float)(g_screen_rows - 1) * CELL_H) * 0.40f;
    g_pixel_scale_per_unit = screen_radius_pixels / g_max_radius_pixels;

    /* Hand off to PLAY. */
    g_app_state = STATE_PLAY;
    return true;
}

/* ===================================================================== */
/* §12  play_state — PLAY-mode state + per-tick simulation update        */
/* ===================================================================== */

static float g_animation_phase_radians = 0.f;
static int   g_auto_add_counter        = 0;
static bool  g_simulation_paused       = false;
static bool  g_auto_add_enabled        = true;
static bool  g_show_orbit_circles      = true;

/* Debug overlay toggles (preserved across draw↔play switches). */
static bool  g_show_ghost_overlay      = true;
static bool  g_show_arm_table          = false;

static void compute_chain_joint_positions(void)
{
    /* Purpose         : evaluate the chain at the current φ.
     * Pseudocode      :
     *     joint[0] = pivot
     *     for i in 0..M-1:
     *       angle  = freq_i · φ + phase_i
     *       radius = amp_i · scale
     *       joint[i+1] = joint[i] + (radius·cos angle, radius·sin angle)
     * Mental model    : add the i-th arm's tip displacement to the
     *                   running joint position.  Each arm is a polar-
     *                   to-Cartesian conversion.
     */
    float x = g_screen_pivot_pixel_x;
    float y = g_screen_pivot_pixel_y;
    g_joint_pixel_x[0] = x;
    g_joint_pixel_y[0] = y;
    for (int i = 0; i < g_active_epicycle_count; i++) {
        const Epicycle *e = &g_epicycle_table[i];
        float angle  = (float)e->frequency_signed * g_animation_phase_radians
                     + e->phase_offset_radians;
        float radius = e->amplitude_normalised * g_pixel_scale_per_unit;
        x += radius * cosf(angle);
        y += radius * sinf(angle);
        g_joint_pixel_x[i + 1] = x;
        g_joint_pixel_y[i + 1] = y;
    }
}

static void play_simulation_tick(void)
{
    /* Purpose : advance simulation state by one frame.  No rendering;
     *           render_layers in §14 reads the state we just updated.
     */
    if (g_simulation_paused) return;

    /* Optional auto-grow. */
    if (g_auto_add_enabled
     && g_active_epicycle_count < g_total_epicycle_count) {
        g_auto_add_counter++;
        if (g_auto_add_counter >= AUTO_ADD_FRAMES) {
            g_auto_add_counter = 0;
            g_active_epicycle_count++;
        }
    }

    /* Advance φ.  On wrap, clear the trail so the next cycle paints
     * onto a clean canvas instead of stacking on the previous one. */
    g_animation_phase_radians += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
    if (g_animation_phase_radians >= 2.f * (float)M_PI) {
        g_animation_phase_radians -= 2.f * (float)M_PI;
        trail_clear(&g_tip_trail);
    }

    compute_chain_joint_positions();
    trail_push(&g_tip_trail,
               g_joint_pixel_x[g_active_epicycle_count],
               g_joint_pixel_y[g_active_epicycle_count]);
}

/* ===================================================================== */
/* §13  project — raw pixel → screen cell transform (ghost overlay)      */
/* ===================================================================== */

static void project_raw_pixel_to_cell(float raw_pixel_x, float raw_pixel_y,
                                      int *out_col, int *out_row)
{
    /* Apply the SAME affine transform the chain uses:
     *   1. subtract centroid (so the path is centred at origin)
     *   2. multiply by scale
     *   3. add pivot (so the centred shape sits at the screen middle)
     *   4. round to cells
     * After this, original raw points and reconstructed chain occupy
     * the same coordinate system and can be visually compared.
     */
    float screen_pixel_x = (raw_pixel_x - g_path_centroid_pixel_x)
                          * g_pixel_scale_per_unit
                          + g_screen_pivot_pixel_x;
    float screen_pixel_y = (raw_pixel_y - g_path_centroid_pixel_y)
                          * g_pixel_scale_per_unit
                          + g_screen_pivot_pixel_y;
    *out_col = px_to_cell_x(screen_pixel_x);
    *out_row = px_to_cell_y(screen_pixel_y);
}

/* ===================================================================== */
/* §14  render_layers — ghost / orbits / trail / chain / markers         */
/* ===================================================================== */

static void render_ghost_overlay(void)
{
    /* Layer 1.  Faint trace of the ORIGINAL raw path through the
     * play-mode transform.  See T9. */
    if (!g_show_ghost_overlay || g_raw_point_count < 2) return;
    attron(COLOR_PAIR(PAIR_GHOST));
    for (int i = 0; i < g_raw_point_count; i++) {
        int col, row;
        project_raw_pixel_to_cell(g_raw_pixel_x[i], g_raw_pixel_y[i],
                                  &col, &row);
        if (col >= 0 && col < g_screen_cols
         && row >= 0 && row < g_screen_rows - 1)
            mvaddch(row, col, '.');
    }
    attroff(COLOR_PAIR(PAIR_GHOST));
}

static void render_orbit_layer(void)
{
    /* Layer 2.  Up to N_CIRCLES_DRAWN orbit ellipses, one per joint
     * for the largest arms.  Capped because more than ~6 becomes
     * visual clutter and the smallest orbits collapse to single cells. */
    if (!g_show_orbit_circles) return;
    int draw_count = (g_active_epicycle_count < N_CIRCLES_DRAWN)
                   ? g_active_epicycle_count : N_CIRCLES_DRAWN;
    for (int i = 0; i < draw_count; i++) {
        float radius_pixels = g_epicycle_table[i].amplitude_normalised
                            * g_pixel_scale_per_unit;
        render_orbit_ellipse(g_joint_pixel_x[i], g_joint_pixel_y[i],
                             radius_pixels);
    }
}

static void render_trail_layer(void)
{
    /* Layer 3.  Tip path stamped oldest-to-newest so newer cells
     * paint over older ones.  Three discrete colour tiers approximate
     * a fade from old (red) to new (yellow). */
    int filled = g_tip_trail.filled_count;
    if (filled == 0) return;
    int oldest_index = (g_tip_trail.write_head - filled + TRAIL_LEN)
                       % TRAIL_LEN;
    for (int i = 0; i < filled; i++) {
        int idx = (oldest_index + i) % TRAIL_LEN;
        int col = px_to_cell_x(g_tip_trail.pixel_x[idx]);
        int row = px_to_cell_y(g_tip_trail.pixel_y[idx]);
        if (col < 0 || col >= g_screen_cols) continue;
        if (row < 0 || row >= g_screen_rows - 1) continue;
        float age_fraction = (float)i / (float)(filled > 1 ? filled : 1);
        int   pair_id = (age_fraction > 0.70f) ? PAIR_TRAIL_NEW
                      : (age_fraction > 0.35f) ? PAIR_TRAIL_MID
                                                : PAIR_TRAIL_OLD;
        attron(COLOR_PAIR(pair_id) | A_BOLD);
        mvaddch(row, col, '*');
        attroff(COLOR_PAIR(pair_id) | A_BOLD);
    }
}

static int arm_pair_for_radius(float radius_pixels)
{
    if (radius_pixels > g_pixel_scale_per_unit * 0.08f) return PAIR_ARM_HI;
    if (radius_pixels > g_pixel_scale_per_unit * 0.02f) return PAIR_ARM_MID;
    return PAIR_ARM_LO;
}

static void render_chain_layer(void)
{
    /* Layer 4.  Bresenham-line each arm from joint[i] to joint[i+1].
     * Line colour by amplitude tier — large arms stand out, the long
     * tail of high-frequency arms recedes. */
    for (int i = 0; i < g_active_epicycle_count; i++) {
        float radius_pixels = g_epicycle_table[i].amplitude_normalised
                            * g_pixel_scale_per_unit;
        attr_t attr = COLOR_PAIR(arm_pair_for_radius(radius_pixels)) | A_BOLD;
        render_bresenham_line(
            px_to_cell_x(g_joint_pixel_x[i]),
            px_to_cell_y(g_joint_pixel_y[i]),
            px_to_cell_x(g_joint_pixel_x[i + 1]),
            px_to_cell_y(g_joint_pixel_y[i + 1]),
            attr);
    }
}

static void render_markers_layer(void)
{
    /* Layer 5.  '@' at the chain tip (the "pen"), '+' at the pivot
     * (the chain origin).  Both bright + bold so they pop against
     * the trail and chain. */
    int tip_col = px_to_cell_x(g_joint_pixel_x[g_active_epicycle_count]);
    int tip_row = px_to_cell_y(g_joint_pixel_y[g_active_epicycle_count]);
    if (tip_col >= 0 && tip_col < g_screen_cols
     && tip_row >= 0 && tip_row < g_screen_rows - 1) {
        attron(COLOR_PAIR(PAIR_TIP) | A_BOLD);
        mvaddch(tip_row, tip_col, '@');
        attroff(COLOR_PAIR(PAIR_TIP) | A_BOLD);
    }
    int pivot_col = px_to_cell_x(g_screen_pivot_pixel_x);
    int pivot_row = px_to_cell_y(g_screen_pivot_pixel_y);
    if (pivot_col >= 0 && pivot_col < g_screen_cols
     && pivot_row >= 0 && pivot_row < g_screen_rows - 1) {
        attron(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
        mvaddch(pivot_row, pivot_col, '+');
        attroff(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
    }
}

/* ===================================================================== */
/* §15  render_debug — 'D' arm-info numeric table                        */
/* ===================================================================== */

static void render_arm_table_overlay(void)
{
    /* Top-N rows of the sorted arm table, printed as text.  Useful
     * for verifying DFT decode by hand, and for picking out which arms
     * dominate the rough shape vs which contribute fine detail. */
    if (!g_show_arm_table) return;
    int rows_to_show = (g_active_epicycle_count < ARM_TABLE_ROWS)
                     ? g_active_epicycle_count : ARM_TABLE_ROWS;
    int x = 2, y = 2;
    if (y + rows_to_show + 1 >= g_screen_rows - 1) return;

    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(y, x, "  i  freq    amp     phase");
    attroff(COLOR_PAIR(PAIR_HINT));

    for (int i = 0; i < rows_to_show; i++) {
        const Epicycle *e = &g_epicycle_table[i];
        int pair = (i < 1) ? PAIR_ARM_HI
                 : (i < 6) ? PAIR_ARM_MID : PAIR_ARM_LO;
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvprintw(y + 1 + i, x, " %2d  %4d  %5.3f  %+5.2f",
                 i, e->frequency_signed,
                 (double)e->amplitude_normalised,
                 (double)e->phase_offset_radians);
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

/* ===================================================================== */
/* §16  hud — HUD top-right + hint bottom + PAUSED chip                  */
/* ===================================================================== */

static void hud_paint_top_right(const char *text)
{
    int x = g_screen_cols - (int)strlen(text);
    if (x < 0) x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, x, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_paused_chip(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
    mvprintw(0, 0, " PAUSED ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
}

static void hud_paint_hint(const char *text)
{
    move(g_screen_rows - 1, 0);
    clrtoeol();
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g_screen_rows - 1, 0, "%s", text);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §17  draw_mode_render — DRAW-mode painter                             */
/* ===================================================================== */

static void render_draw_mode(void)
{
    /* Path cells (Bresenham-filled) painted as '*'. */
    for (int r = 0; r < g_screen_rows - 1 && r < ROWS_MAX; r++) {
        for (int c = 0; c < g_screen_cols && c < COLS_MAX; c++) {
            if (!g_drawn_cells[r][c]) continue;
            attron(COLOR_PAIR(PAIR_PATH));
            mvaddch(r, c, '*');
            attroff(COLOR_PAIR(PAIR_PATH));
        }
    }

    /* Cursor on top of path. */
    if (g_cursor_row < g_screen_rows - 1) {
        attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
        mvaddch(g_cursor_row, g_cursor_col, '@');
        attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    }

    /* Top HUD: status + bbox + closed-toggle + theme. */
    int bw = 0, bh = 0;
    compute_bbox_in_cells(&bw, &bh);
    char buf[180];
    if (g_draw_error_too_few) {
        snprintf(buf, sizeof buf,
                 " DRAW  need >= 4 pts  pts:%d/%d  cur:(%d,%d)  bbox:%dx%d  closed:%s  thm:%s ",
                 g_raw_point_count, RAW_MAX, g_cursor_col, g_cursor_row,
                 bw, bh, g_close_path_enabled ? "Y" : "N",
                 theme_table[g_active_theme_index].display_name);
    } else {
        snprintf(buf, sizeof buf,
                 " DRAW  pts:%d/%d  cur:(%d,%d)  bbox:%dx%d  closed:%s  thm:%s ",
                 g_raw_point_count, RAW_MAX, g_cursor_col, g_cursor_row,
                 bw, bh, g_close_path_enabled ? "Y" : "N",
                 theme_table[g_active_theme_index].display_name);
    }
    hud_paint_top_right(buf);
    hud_paint_hint(" arrows/WASD:draw  ENTER/g:play  c:clear  o:close  t/T:theme  q:quit ");
}

/* ===================================================================== */
/* §18  play_mode_render — PLAY-mode painter                             */
/* ===================================================================== */

static void render_play_mode(void)
{
    /* Layer 1..5 — main scene. */
    render_ghost_overlay();
    render_orbit_layer();
    render_trail_layer();
    render_chain_layer();
    render_markers_layer();

    /* Layer 6 — debug overlays. */
    render_arm_table_overlay();

    /* Layer 7 — HUD top + hint bottom + optional PAUSED chip. */
    char buf[180];
    snprintf(buf, sizeof buf,
             " PLAY  arms:%d/%d  thm:%s  closed:%s  ghost:%s ",
             g_active_epicycle_count, g_total_epicycle_count,
             theme_table[g_active_theme_index].display_name,
             g_close_path_enabled ? "Y"  : "N",
             g_show_ghost_overlay ? "ON" : "off");
    hud_paint_top_right(buf);
    if (g_simulation_paused) hud_paint_paused_chip();
    hud_paint_hint(" r:redraw  p:pause  +/-:arms  c:circles  o:close  d:ghost  D:arms  t/T:theme  a:auto  q:quit ");
}

/* ===================================================================== */
/* §19  screen — ncurses init / cleanup                                  */
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
    colors_init();
}

static void screen_cleanup(void)
{
    endwin();
}

/* ===================================================================== */
/* §20  app — signal handlers, main loop, key dispatch                   */
/* ===================================================================== */

static volatile sig_atomic_t g_should_quit    = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig)
{
    if (sig == SIGINT || sig == SIGTERM) g_should_quit    = 1;
    if (sig == SIGWINCH)                 g_resize_pending = 1;
}

static void handle_play_mode_close_toggle(void)
{
    /* 'o' in PLAY: flip the closed-path toggle and re-run the DFT
     * live so the user sees the seam appear/disappear without leaving
     * PLAY.  Preserve the user's chosen arm count across the re-run. */
    g_close_path_enabled = !g_close_path_enabled;
    int saved_active = g_active_epicycle_count;
    if (switch_from_draw_to_play()) {
        /* switch_from_draw_to_play resets g_active_epicycle_count to 1
         * via build_sorted_epicycle_table; restore the saved count for
         * a smooth A/B comparison. */
        g_active_epicycle_count = saved_active;
        if (g_active_epicycle_count > g_total_epicycle_count)
            g_active_epicycle_count = g_total_epicycle_count;
        g_app_state = STATE_PLAY;
    }
}

int main(void)
{
    atexit(screen_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);

    screen_init();
    getmaxyx(stdscr, g_screen_rows, g_screen_cols);
    reset_draw_state();

    long long last_frame_ns = clock_now_ns();

    while (!g_should_quit) {
        /* ── resize ── */
        if (g_resize_pending) {
            g_resize_pending = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, g_screen_rows, g_screen_cols);
            if (g_app_state == STATE_DRAW) {
                reset_draw_state();
            } else {
                /* Recompute pivot + scale from saved g_max_radius_pixels
                 * so the reconstruction stays "40% of screen" through
                 * any number of resizes. */
                g_screen_pivot_pixel_x =
                    (float)g_screen_cols * CELL_W * 0.5f;
                g_screen_pivot_pixel_y =
                    (float)(g_screen_rows - 1) * CELL_H * 0.5f;
                float screen_radius_pixels =
                    fminf((float)g_screen_cols * CELL_W,
                          (float)(g_screen_rows - 1) * CELL_H) * 0.40f;
                g_pixel_scale_per_unit =
                    screen_radius_pixels / g_max_radius_pixels;
                trail_clear(&g_tip_trail);
            }
            last_frame_ns = clock_now_ns();
            continue;
        }

        /* ── input ── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (g_app_state == STATE_DRAW) {
                switch (ch) {
                case 'q': case 'Q': case 27:
                    g_should_quit = 1; break;
                case KEY_UP:    case 'w': case 'W': move_cursor_with_bresenham_fill( 0, -1); break;
                case KEY_DOWN:  case 's': case 'S': move_cursor_with_bresenham_fill( 0,  1); break;
                case KEY_LEFT:  case 'a': case 'A': move_cursor_with_bresenham_fill(-1,  0); break;
                case KEY_RIGHT: case 'd': case 'D': move_cursor_with_bresenham_fill( 1,  0); break;
                case '\n': case '\r': case 'g': case 'G':
                    switch_from_draw_to_play();
                    break;
                case 'c': case 'C':
                    reset_draw_state();
                    break;
                case 'o': case 'O':
                    g_close_path_enabled = !g_close_path_enabled;
                    break;
                case 't':
                    g_active_theme_index = (g_active_theme_index + 1) % N_THEMES;
                    apply_theme(g_active_theme_index);
                    break;
                case 'T':
                    g_active_theme_index = (g_active_theme_index + N_THEMES - 1) % N_THEMES;
                    apply_theme(g_active_theme_index);
                    break;
                }
            } else { /* STATE_PLAY */
                switch (ch) {
                case 'q': case 'Q': case 27:
                    g_should_quit = 1; break;
                case 'r': case 'R':
                    g_app_state = STATE_DRAW;
                    g_draw_error_too_few = false;
                    break;
                case ' ': case 'p': case 'P':
                    g_simulation_paused = !g_simulation_paused;
                    break;
                case '+': case '=':
                    if (g_active_epicycle_count < g_total_epicycle_count) {
                        g_active_epicycle_count++;
                        trail_clear(&g_tip_trail);
                    }
                    break;
                case '-':
                    if (g_active_epicycle_count > 1) {
                        g_active_epicycle_count--;
                        trail_clear(&g_tip_trail);
                    }
                    break;
                case 'c': case 'C':
                    g_show_orbit_circles = !g_show_orbit_circles;
                    break;
                case 'o': case 'O':
                    handle_play_mode_close_toggle();
                    break;
                case 'd':
                    g_show_ghost_overlay = !g_show_ghost_overlay;
                    break;
                case 'D':
                    g_show_arm_table = !g_show_arm_table;
                    break;
                case 'a': case 'A':
                    g_auto_add_enabled = !g_auto_add_enabled;
                    break;
                case 't':
                    g_active_theme_index = (g_active_theme_index + 1) % N_THEMES;
                    apply_theme(g_active_theme_index);
                    break;
                case 'T':
                    g_active_theme_index = (g_active_theme_index + N_THEMES - 1) % N_THEMES;
                    apply_theme(g_active_theme_index);
                    break;
                }
            }
        }

        /* ── tick ── */
        if (g_app_state == STATE_PLAY) play_simulation_tick();

        /* ── draw ── */
        erase();
        if (g_app_state == STATE_DRAW) render_draw_mode();
        else                           render_play_mode();
        wnoutrefresh(stdscr);
        doupdate();

        /* ── FPS cap ── */
        long long now_ns = clock_now_ns();
        clock_sleep_ns(RENDER_TICK_NS - (now_ns - last_frame_ns));
        last_frame_ns = clock_now_ns();
    }
    return 0;
}
