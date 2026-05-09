/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * pulsar_rain.c — rotating pulsar beams with matrix-rain shimmer
 *
 * DEMO: A neutron-star pulsar in your terminal. N beams sweep around
 *       a single '@' core like a lighthouse, each leaving an angular
 *       wake of fading characters behind it. The wake glyphs are
 *       random ASCII letters/digits/punctuation that reroll every
 *       frame — the same matrix-rain shimmer trick from
 *       matrix_rain.c, applied to a rotating beam instead of a
 *       falling stream.
 *
 *           SINGLE-BEAM SCHEMATIC (one frame, beam pointing east)
 *           ─────────────────────────────────────────────────────
 *
 *                       k=WAKE_LEN  (oldest, FADE/DIM)
 *                       │
 *                       ▼
 *                       . . . . .
 *                      . . . . . .
 *                     . . . . . . .
 *                    . . . . . . . .
 *                   . . . . . . . . .         ←  N_RADII radial samples
 *                  . . . . . . . . . .            walk outward from core
 *                 @ . . . . . . . . . X
 *                  ▲   ↑           ↑    ↑
 *                core  HOT (k=1)   |    HEAD (k=0)
 *                                  MID (k≈WAKE_LEN/2)
 *
 *           The wake spans WAKE_LEN · WAKE_STEP = 16 · 0.05 ≈ 46°
 *           behind the head. Drawing dim slots first lets the
 *           bright HEAD always win at any cell where multiple
 *           angular slots round to the same column/row (which
 *           happens at small radius).
 *
 *           Default n_beams = 2 (classic pulsar pair, 180° apart).
 *           Press ']' to add beams (3=tri-blade, 4=cross, ...).
 *           Press '[' to remove. 1..16 evenly spaced at 360°/N.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — same shimmer trick on plain
 *                                     vertical falling streams.
 *   matrix_rain/fireworks_rain.c    — shimmer on arc trails.
 *   matrix_rain/matrix_snowflake.c  — rain accumulating into snow.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed 5-shade palettes
 *   §4  pulsar       — Pulsar state, tick, beam draw, core draw
 *   §5  screen       — ncurses init / present / HUD
 *   §6  app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset (zero angle, default beam count)
 *   = / +            spin faster
 *   -                spin slower
 *   ]                add a beam (1 → 16)
 *   [                remove a beam
 *   t                cycle theme (5 themes)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/pulsar_rain.c \
 *       -o pulsar_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Polar-coordinate sweep with shimmer cache.
 *
 *                 (A) ROTATION — a single float `angle` advances by
 *                     `omega · dt` each frame, where omega = spin_rps
 *                     · 2π. No fixed-step accumulator, no alpha
 *                     interpolation: the float angle gives smooth
 *                     sub-cell rotation at any frame rate. Variable-
 *                     dt is unconditionally stable here because the
 *                     simulation is non-stiff (no springs, no fast
 *                     oscillators, no contact constraints).
 *
 *                 (B) BEAM   — for each beam b in 0..n_beams-1, the
 *                     base angle is `angle + b · 2π/n_beams`. A
 *                     beam consists of N_RADII radial samples
 *                     (steps outward from the core) crossed with
 *                     WAKE_LEN+1 angular wake slots (each step
 *                     trailing the head by WAKE_STEP radians). The
 *                     direction vectors for the wake_len+1 slots
 *                     are pre-computed once per beam (only 17 trig
 *                     calls per beam regardless of N_RADII), then
 *                     reused at every radial sample.
 *
 *                 (C) SHIMMER — a 2-D char cache `glyphs[N_RADII]
 *                     [WAKE_LEN+1]` holds one ASCII glyph per
 *                     (radius, wake-slot) pair. Every frame ~75% of
 *                     cells reroll, so the same beam geometry
 *                     re-renders with constantly-changing characters
 *                     — the matrix shimmer.
 *
 * Data-structure: Pulsar struct holds the angle, spin rate (in
 *                 rotations/sec), beam count, theme index, paused
 *                 flag, geometry (cx, cy, max_r, r_step), and the
 *                 N_RADII × (WAKE_LEN+1) glyph cache. Everything
 *                 inline; no heap allocation post-init.
 *
 * Rendering     : For each beam, pre-compute cw[k]/sw[k] direction
 *                 vectors once, then walk N_RADII radial samples.
 *                 At each ri walk angular slots k = WAKE_LEN..0
 *                 (DIM FIRST) so the bright HEAD slot wins overlap
 *                 at small radius where multiple slots round to the
 *                 same cell. Beams paint in any order; all share
 *                 the same brightness ramp at each k. Core '@' paints
 *                 LAST so it always sits on top of every beam.
 *
 * Performance   : Per frame: O(beams · (WAKE_LEN+1) trig + beams ·
 *                 N_RADII · (WAKE_LEN+1) cells). With defaults (2
 *                 beams, 80 radii, 17 wake slots) that's 34 trig +
 *                 ~2700 mvaddch — microseconds. ncurses redraw is
 *                 the dominant cost.
 *
 * References    :
 *   Wikipedia, "Pulsar" — rotating neutron stars, the lighthouse
 *     model, beam radiation geometry. Real pulsars have one or two
 *     beams; the >2 multi-beam variants in this demo are stylised.
 *     https://en.wikipedia.org/wiki/Pulsar
 *   Wikipedia, "Lighthouse model (pulsar)" — the analogy that gives
 *     the demo its visual.
 *     https://en.wikipedia.org/wiki/Pulsar#Lighthouse_model
 *   "The Matrix" (1999, Wachowski) — visual inspiration for the
 *     shimmering ASCII glyphs riding the rotating beams.
 *   This project, matrix_rain/matrix_rain.c — the same shimmer-
 *     cache pattern on plain vertical streams.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The pulsar is a SHEAF OF DIRECTIONS, not a pixel buffer. At any
 * instant the simulation owns N angular sweep directions (`angle`,
 * `angle + 2π/N`, ...). Each direction emits a beam consisting of
 * N_RADII radial samples (going outward from the core) and a
 * WAKE_LEN-slot angular tail (going back behind the head). Rendering
 * one beam is therefore TWO indices: ri (how far along the beam) and
 * k (how far behind the head). The head k=0 is white-bold; growing k
 * fades the colour through HOT → BRIGHT → MID → DARK → FADE while
 * also rotating the cell back through the wake. No off-screen pixel
 * buffer for the rotation — the angle alone advances; everything
 * else is recomputed every frame from cos/sin.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * A lighthouse sweeping a foggy night. The beam itself is bright and
 * narrow; behind it, glowing fog particles linger in the air for a
 * moment before fading. Now spin the lighthouse fast enough that you
 * see N beams at once (one or two for a real pulsar; more if you
 * want stylised flowers). Each frame you snap a photo of the beams
 * plus their fog wakes. The matrix-rain shimmer is just the foggy
 * particles being random ASCII chars that re-pick themselves every
 * frame — the FOG IS THE TEXT.
 *
 * BEAM GEOMETRY DIAGRAM
 * ─────────────────────
 *
 *   The core '@' sits at (cx, cy). For ONE beam at base_angle:
 *
 *      k=0 = head; k=1 trails by WAKE_STEP rad; k=2 by 2·WAKE_STEP; ...
 *      k=WAKE_LEN trails by WAKE_LEN · WAKE_STEP ≈ 46°.
 *
 *      For ri = 0..N_RADII-1, the cell at (ri, k) is at:
 *
 *          θ_k  = base_angle - k · WAKE_STEP
 *          r    = (ri + 1) · r_step
 *          col  = cx + round(r · cos θ_k)
 *          row  = cy + round(r · sin θ_k · ASPECT)
 *
 *      ASPECT (~0.45) compresses sin to compensate for terminal
 *      cells being roughly 2× tall as they are wide.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │     . . . .                                             │
 *      │   .  .  .  .  .                                         │
 *      │  .   .   .   .   X    ← head (k=0)                      │
 *      │  .   .   .   .  /                                       │
 *      │   .  .  .  .  /                                         │
 *      │     . . .  ./       beam direction (base_angle)         │
 *      │            @ ───────────────►                           │
 *      │            ▲                                            │
 *      │           core (cx, cy)                                 │
 *      │                                                         │
 *      └─────────────────────────────────────────────────────────┘
 *
 * BRIGHTNESS RAMP
 * ───────────────
 *
 *      k=0           HEAD     white     BOLD
 *      k=1           HOT      theme[4]  BOLD
 *      k=2           BRIGHT   theme[3]  BOLD
 *      k=3..N/2      MID      theme[2]  normal
 *      k=N/2+1..N-2  DARK     theme[1]  normal
 *      k=N-1..N      FADE     theme[0]  DIM
 *
 *      WAKE_LEN+1 = 17 slots in total (k = 0..16).
 *
 * MULTI-BEAM LAYOUT
 * ─────────────────
 *
 *      n_beams beams evenly spaced around the circle:
 *
 *          beam b at base_angle = angle + b · (2π / n_beams)
 *
 *      n=1 :  single sweeping beam
 *      n=2 :  classic pulsar pair (0°, 180°)        ← default
 *      n=3 :  tri-blade            (0°, 120°, 240°)
 *      n=4 :  cross                (0°, 90°, 180°, 270°)
 *      n=8 :  star
 *      n=16:  flower (wakes overlap at 22.5°)
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. Measure dt (wall-clock seconds since last frame, capped).
 *  2. pulsar_tick(dt):
 *       angle += spin_rps · 2π · dt          (advance rotation)
 *       wrap angle into [0, 2π)
 *       pulsar_shimmer(): reroll ~75% of glyphs in the cache
 *                         (1 in SHIMMER_KEEP_ONE_IN survives)
 *  3. erase()
 *  4. pulsar_draw():
 *       For each beam b in 0..n_beams-1:
 *           base = angle + b · (2π / n_beams)
 *           pulsar_draw_beam(base):
 *               Pre-compute cw[k] = cos(base − k·WAKE_STEP)
 *                          sw[k] = sin(base − k·WAKE_STEP) · ASPECT
 *                                                         (k = 0..WAKE_LEN)
 *               For ri = 0..N_RADII-1:
 *                   r = (ri+1) · r_step
 *                   For k = WAKE_LEN..0:        ← DIM-FIRST is load-bearing
 *                       col = cx + round(r · cw[k])
 *                       row = cy + round(r · sw[k])
 *                       paint glyphs[ri][k] with wake_attr(k)
 *       pulsar_draw_core():
 *           paint '@' at (cx, cy) — drawn LAST, always on top
 *  5. HUD: yellow status row 0; cyan hint strip on bottom row.
 *  6. doupdate; sleep to 60-fps cap.
 *
 * KEY FORMULAS
 * ────────────
 *   omega       = spin_rps · 2π                    rad / sec
 *   angle(t+dt) = angle(t) + omega · dt
 *
 *   For each beam b:
 *     base_b = angle + b · (2π / n_beams)
 *
 *   For each wake slot k (0..WAKE_LEN):
 *     θ_k = base_b - k · WAKE_STEP
 *     cw[k] = cos θ_k
 *     sw[k] = sin θ_k · ASPECT
 *
 *   For each radial sample ri (0..N_RADII-1):
 *     r = (ri + 1) · r_step
 *     col = cx + round(r · cw[k])
 *     row = cy + round(r · sw[k])
 *
 *   Wake arc      = WAKE_LEN · WAKE_STEP = 16 · 0.05 = 0.80 rad ≈ 46°
 *   Shimmer rate  = 1 − 1/SHIMMER_KEEP_ONE_IN = 1 − 1/4 = 0.75
 *   Coverage      = WAKE_STEP · r ≈ 1 cell at r ≈ 1/STEP = 20 cells
 *                   (sets the minimum radius for solid-arc wake density)
 *   max_r         = √((cols/2)² + (rows/(2·ASPECT))²) · MAX_R_OVERSHOOT
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • At small radius, multiple wake slots round to the same cell.
 *    Drawing slots in DESCENDING order (WAKE_LEN→0) ensures the
 *    bright HEAD wins. Reverse the loop and the head hides behind
 *    its own dim fog.
 *
 *  • Beam overlap with n_beams=16 — adjacent wakes (22.5° apart)
 *    merge. Order doesn't matter between beams since they share the
 *    same brightness ramp at each k.
 *
 *  • Glyph cache rerolls per FRAME at ~75%. Lower rate (KEEP_ONE_IN
 *    larger) feels frozen; higher rate (smaller) becomes noisy.
 *
 *  • ASPECT is BAKED into sw[k] (sw[k] = sin·ASPECT). Never apply
 *    it again at draw time or the wake compresses vertically twice.
 *
 *  • Resize calls pulsar_init which fully resets the cache and
 *    recomputes max_r/r_step/cx/cy. The angle and spin survive so
 *    the visual continuity is preserved across resize.
 *
 *  • spin_rps = 0 (perfectly stationary) is allowed by SPIN_MIN_RPS=0;
 *    the shimmer continues so the beams don't look frozen.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Default 2 beams: at any frozen frame (`space` to pause) the
 *    two beams are exactly 180° apart. Press `]` to add a beam:
 *    3 at 120°, 4 at 90°, ...
 *
 *  • Spin sanity: HUD reads "rps" (rotations per second). At
 *    SPIN_DEFAULT_RPS = 0.50, one beam returns to its starting
 *    cell every 2 seconds — time it.
 *
 *  • Slow with `-` to SPIN_MIN_RPS = 0.05 (0.05 rev/sec ≈ 18° per
 *    second); the rotation is barely visible but the shimmer
 *    continues.
 *
 *  • Inspect the wake: head cell is white BOLD, then 1 hot, 1
 *    bright, a stretch of mid, a stretch of dark, finally fade
 *    DIM — for a total of WAKE_LEN+1 = 17 angular slots.
 *
 *  • The '@' core never disappears regardless of beam count or
 *    spin. It paints last.
 *
 *  • Theme cycle: `t` runs through 5 hue palettes (green / amber /
 *    blue / plasma / fire); head and core stay white in every
 *    theme; HUD stays bright yellow; hint strip stays bright cyan.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read matrix_rain.c first; the SHIMMER and BAND
 *      mechanics (T3-T4 there) are reused. The NEW LESSON here is
 *      polar-coordinate beams instead of vertical streams.
 *   2. §4 pulsar — THE HEART. Read AFTER tutorials T1-T5 below.
 *      Sub-sections:
 *        - pulsar_tick      ← angle += omega · dt  (T1)
 *        - draw_beam        ← polar walk + wake fading (T2-T4)
 *        - draw_core        ← '@' at the centre, painted last
 *   3. §3 color — 5 themed palettes + HUD pairs.
 *   4. §1 + §2 + §5 + §6 — config / clock / screen / app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   angle              float radians, advances by omega · dt.
 *   spin_rps           rotations PER SECOND (physical units).
 *   omega              spin_rps · 2π — radians per second.
 *   n_beams            number of evenly-spaced beams (1..16).
 *   base_angle         angle + b · (2π / n_beams) for beam b.
 *   ri                 radial index along a beam (0..N_RADII-1).
 *   k                  wake slot index (0 = head, larger = older).
 *   WAKE_LEN+1         total wake slots.
 *   WAKE_STEP          radians of trail per slot.
 *   r_step             radial step size (pixels per ri).
 *   ASPECT             vertical squash factor (~0.45) so a circle
 *                      looks circular in cells, not stretched.
 *   glyphs[ri][k]      cached glyph at radial sample ri and wake
 *                      slot k. Same shimmer cache idea as
 *                      matrix_rain T3.
 *
 * Background you need
 * ───────────────────
 *   - matrix_rain T1-T5 (especially T3 shimmer cache).
 *   - Polar coordinates: (r, θ) → (r cos θ, r sin θ).
 *   - Terminal aspect ratio: cells are ~2× as tall as wide.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Pulsar physics (general relativity, magnetic dipoles). The
 *     "rotating beam" is purely a visual analogy.
 *   - Anti-aliasing. We deliberately rely on cell snapping.
 *   - Sub-pixel rotation. The angle advances continuously; the
 *     visual look is fine without ANY anti-aliasing.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a rotating Matrix-style pulsar from
 * first principles.
 *
 *   T1  Polar geometry — what it means to "rotate" a stream
 *   T2  Wake — extending the beam BACKWARDS in angle
 *   T3  ASPECT correction — why circles need a y-squash factor
 *   T4  Render order — DIM FIRST so the head wins overlaps
 *   T5  Multiple beams — evenly spaced, no extra state
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  POLAR GEOMETRY — WHAT IT MEANS TO "ROTATE" A STREAM
 * ───────────────────────────────────────────────────────
 * matrix_rain has streams at FIXED COLUMNS that fall over time.
 * The simulation owns one float per column (head_y).
 *
 * pulsar_rain has BEAMS at FIXED RADIAL DEPTHS that rotate
 * over time. The simulation owns ONE float (angle) shared by
 * every beam.
 *
 *     angle += omega · dt
 *     where omega = spin_rps · 2π
 *
 * For beam b, base direction is:
 *     base_angle = angle + b · (2π / n_beams)
 *
 * For radial sample ri at radius r = (ri + 1) · r_step:
 *     col = cx + round(r · cos(base_angle))
 *     row = cy + round(r · sin(base_angle) · ASPECT)
 *
 * That's how a "beam" gets drawn — walk OUTWARD from the centre
 * in the base_angle direction, painting a glyph at each radial
 * sample.
 *
 * Compare with matrix_rain:
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  matrix_rain    pulsar_rain                      │
 *      │                                                  │
 *      │  vertical fall  rotational sweep                 │
 *      │  head_y ↑       angle ↑                          │
 *      │  one per col    one for whole sim                │
 *      │  trail BACK     wake BACK in angle               │
 *      │  at fixed col   at fixed radius                  │
 *      └──────────────────────────────────────────────────┘
 *
 * Both files OWN a single float per visual element and DERIVE
 * the screen position by trig. matrix_rain uses identity (head_y
 * IS row); pulsar_rain uses cos/sin.
 *
 * T2  WAKE — EXTENDING THE BEAM BACKWARDS IN ANGLE
 * ────────────────────────────────────────────────
 * A bare beam (just the leading edge) reads as a "spinning
 * line." A WAKE — angularly trailing copies of the beam — turns
 * it into a rotating glow, like a lighthouse beam through fog.
 *
 * Implementation: at each radial sample, paint multiple slots
 * at slightly DIFFERENT base_angles:
 *
 *     for k in 0 .. WAKE_LEN:
 *       theta_k = base_angle - k · WAKE_STEP
 *       paint cell at (cx + r·cos theta_k, cy + r·sin theta_k · ASPECT)
 *               in band(k) colour with shimmer glyph
 *
 * Slot k = 0 is the HEAD (brightest). Slot k = WAKE_LEN is the
 * trailing TIP (faintest). The angular span behind the head is:
 *
 *     wake_span = WAKE_LEN · WAKE_STEP
 *
 * For WAKE_LEN = 16 and WAKE_STEP = 0.05 rad: span ≈ 46°. Tune
 * larger for a long lazy trail; smaller for a sharp blade.
 *
 * Brightness banding (HEAD / HOT / BRIGHT / MID / DARK / FADE) is
 * the same six-band ramp from matrix_rain T4 — only the
 * INDEX is the angular slot k instead of the radial dist.
 *
 * Optimisation: pre-compute cos(theta_k) and sin(theta_k) ONCE
 * per beam (17 trig calls), reuse them at every ri. Without
 * that, the inner loop would do 17 × 80 = 1360 trig calls per
 * beam.
 *
 * T3  ASPECT CORRECTION — WHY CIRCLES NEED A Y-SQUASH FACTOR
 * ──────────────────────────────────────────────────────────
 * Terminal cells are NOT SQUARE. A typical monospace cell is
 * roughly TWICE AS TALL as it is wide. If we render a circle
 * naively:
 *
 *     col = cx + round(r · cos θ)
 *     row = cy + round(r · sin θ)
 *
 * the result LOOKS LIKE A VERTICAL ELLIPSE — taller than wide
 * by ~2×.
 *
 * Fix: multiply the y-component by a squash factor < 1:
 *
 *     row = cy + round(r · sin θ · ASPECT)
 *
 * Where ASPECT ≈ cell_width / cell_height ≈ 8/16 = 0.5. Tune
 * empirically — 0.45 looks right on most terminals.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  no aspect correction:    with ASPECT = 0.5:     │
 *      │                                                  │
 *      │       . . .                       .              │
 *      │     .       .                  .  .  .           │
 *      │    .         .                .       .          │
 *      │    .         .                .       .          │
 *      │     .       .                  .  .  .           │
 *      │       . . .                       .              │
 *      │                                                  │
 *      │  vertical egg                  round-ish         │
 *      └──────────────────────────────────────────────────┘
 *
 * Same trick is used by every cell-space radial sim: spirals,
 * sun_rain, mandala demos.
 *
 * T4  RENDER ORDER — DIM FIRST SO THE HEAD WINS OVERLAPS
 * ──────────────────────────────────────────────────────
 * Two slots at SMALL RADIUS may round to the SAME terminal
 * cell. If we paint k = 0 (HEAD, bright) FIRST and then k = 16
 * (FADE, dim) on top of it at the same cell, the dim slot
 * overwrites the head — visual artifact, head looks fragmented.
 *
 * Fix: paint slots from DIM TO BRIGHT (k = WAKE_LEN down to 0).
 * The bright HEAD always lands LAST, winning every overlap.
 *
 *     for ri in 0 .. N_RADII-1:
 *       for k in WAKE_LEN .. 0 (decreasing):
 *         paint slot k
 *
 * Same painter's-algorithm reasoning as snowflake T4 (snow
 * underneath, rain on top). Always paint the FAR / DIM /
 * background layer first; bring the BRIGHT focal layer last.
 *
 * Core '@' painted absolute LAST so it always wins the centre
 * cell, regardless of how many beams happen to converge there.
 *
 * T5  MULTIPLE BEAMS — EVENLY SPACED, NO EXTRA STATE
 * ──────────────────────────────────────────────────
 * The simulation owns ONE float (`angle`). All n_beams beams
 * are derived from it:
 *
 *     for b in 0 .. n_beams-1:
 *       base_angle_b = angle + b · (2π / n_beams)
 *       draw_beam(base_angle_b)
 *
 * Adding more beams costs only RENDER time — no extra state.
 * The user can change n_beams interactively (']' / '[' keys)
 * with no allocation, no disruption.
 *
 * This is the matrix_rain "stream-per-column" decomposition
 * (T1 there) generalised: stream-per-(beam-index) where
 * indices map to evenly spaced angles.
 *
 * Visual interpretations of n_beams:
 *
 *     1   single sweeping beam (rotating searchlight)
 *     2   classic pulsar pair (180° apart)        ← default
 *     3   tri-blade (120° apart)
 *     4   cross (90° apart)
 *     8   8-pointed star
 *     16  flower with overlapping wakes — visually busy
 *
 * As n_beams grows, the angular gaps between beams shrink. At
 * n_beams = 16, adjacent wakes overlap (each spans 46°,
 * gap is 22.5°), creating a near-solid ring. Pure aesthetic
 * exploration.
 *
 * Same trick (one float + index → many copies) reappears in
 * fireworks_rain (one explosion fires N particles around a
 * circle) and sun_rain (the sun emits N rays).
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
/* §1  config                                                             */
/* ===================================================================== */

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum {
    TARGET_FPS = 60,                /* render and physics rate */
};

/* ── §1.2 beam geometry ───────────────────────────────────────────── */
enum {
    /* N_RADII — number of radial samples per beam. 80 is enough for
     * a continuous beam from core to far corner of any reasonable
     * terminal; smaller leaves gaps at the rim. */
    N_RADII   = 80,

    /* WAKE_LEN — angular wake depth. Each beam has WAKE_LEN+1 slots
     * (head plus tail) trailing behind it. 16 spans about 46° of arc. */
    WAKE_LEN  = 16,
};

/*
 * WAKE_STEP — angular gap between consecutive wake slots (radians).
 *   Rationale: at radius r, one slot spans r · WAKE_STEP cells.
 *   With WAKE_STEP = 0.05, slots are 1 cell wide at r = 20 cells.
 *   Smaller WAKE_STEP = thinner wake; larger = chunkier.
 *   Total wake arc = WAKE_LEN · WAKE_STEP = 16 · 0.05 = 0.8 rad ≈ 46°.
 */
#define WAKE_STEP   0.05f

/*
 * ASPECT — terminal cell height/width ratio correction.
 *   Baked into sw[k] (sin · ASPECT) so every position formula is:
 *       col = cx + r · cos θ
 *       row = cy + r · sin θ · ASPECT
 *   Without ASPECT, beams look stretched vertically because terminal
 *   cells are ~2× tall as they are wide.
 */
#define ASPECT   0.45f

/*
 * MAX_R_OVERSHOOT — fraction past the screen-corner distance that
 * we walk N_RADII radial samples to. > 1 ensures the beam reaches
 * just past the corners so no visible gap appears at the edges.
 */
#define MAX_R_OVERSHOOT  1.05f

/* ── §1.3 spin (rotations per second — physical units) ────────────── */
#define SPIN_DEFAULT_RPS  0.50f
#define SPIN_MIN_RPS      0.00f
#define SPIN_MAX_RPS      4.00f
#define SPIN_STEP_RPS     0.10f

/* ── §1.4 beam count range ────────────────────────────────────────── */
enum {
    BEAMS_MIN     =  1,
    BEAMS_DEFAULT =  2,
    BEAMS_MAX     = 16,
};

/* ── §1.5 shimmer ──────────────────────────────────────────────────── */
/* Per-frame, each glyph has a 1-in-KEEP_ONE_IN chance of surviving
 * (the rest reroll). KEEP_ONE_IN = 4 → reroll fraction = 0.75, the
 * classic Matrix-rain shimmer rate. Lower the value (smaller) to
 * make the shimmer noisier; raise it to feel frozen. */
#define SHIMMER_KEEP_ONE_IN  4

/* ── §1.6 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..5 — wake bands (theme-controlled) */
    SHADE_FADE     = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,

    /* 6 — beam head, always white (theme-independent) */
    SHADE_HEAD,

    /* 7 — pulsar core '@', always white (theme-independent) */
    SHADE_CORE,

    /* 8..9 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

/* ── §1.7 dt cap (spiral-of-death guard) ──────────────────────────── */
#define DT_CAP_SEC  0.10f

/* ── §1.8 timing primitives ───────────────────────────────────────── */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* ── §1.9 HUD layout ──────────────────────────────────────────────── */
#define HUD_BUF_LEN  96

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Theme — 5-colour wake ramp. Pairs SHADE_FADE..SHADE_HOT are
 * remapped per theme; SHADE_HEAD and SHADE_CORE are always white.
 *
 * Every entry is in the bright half of the 256-colour space
 * (CLAUDE.md brightness rule: cube ≥ 24, with 24-29 as the lowest
 * ramp tier only). Values 16-23 are deliberately excluded.
 *
 * Themes:
 *   green   — classic Matrix
 *   amber   — orange / sodium-vapour
 *   blue    — cool, cyber-noir
 *   plasma  — purple / magenta — the eponymous pulsar plasma
 *   fire    — red / orange — supernova hue
 */
typedef struct {
    const char *name;
    int         fg  [5];     /* 256-colour                              */
    int         fg_8[5];     /* 8-colour fallback                       */
} Theme;

static const Theme k_themes[] = {
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "amber",
      {  94, 130, 172, 214, 220 },
      { COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW,  COLOR_YELLOW  } },
    { "blue",
      {  24,  33,  39,  45,  51 },
      { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN    } },
    { "plasma",
      {  53,  57,  93, 129, 201 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
    { "fire",
      {  52,  88, 124, 160, 196 },
      { COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED,     COLOR_RED     } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — bind wake bands and head/core for the chosen theme.
 * PAIR_HUD and PAIR_HINT are NEVER touched here — they carry semantic
 * meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
    init_pair(SHADE_CORE,   COLOR_WHITE, COLOR_BLACK);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD sits on the
 * user's real background instead of a forced black box.
 */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/*
 * wake_attr — map angular distance k from beam head to an ncurses
 * attr (colour pair + bold/normal/dim).
 *
 *   k=0           HEAD     white     BOLD
 *   k=1           HOT      theme[4]  BOLD
 *   k=2           BRIGHT   theme[3]  BOLD
 *   k=3..N/2      MID      theme[2]  NORMAL
 *   k=N/2+1..N-2  DARK     theme[1]  NORMAL
 *   k=N-1..N      FADE     theme[0]  DIM
 */
static attr_t wake_attr(int k)
{
    if (k == 0)              return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (k == 1)              return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (k == 2)              return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (k <= WAKE_LEN / 2)   return COLOR_PAIR(SHADE_MID);
    if (k <= WAKE_LEN - 2)   return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* ===================================================================== */
/* §4  pulsar — state, tick, beam draw, core draw                         */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utility ─────────────────────────── */

static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }

/* ── §4.2 Pulsar type ─────────────────────────────────────────────── */

/*
 * Pulsar — full simulation state.
 *
 *   Rotation:
 *     angle      current beam angle, radians, wrapped into [0, 2π).
 *                Advances by spin_rps · 2π · dt every frame.
 *     spin_rps   spin rate, rotations per second. Physical unit you
 *                can verify with a stopwatch.
 *     n_beams    number of beams, evenly spaced at 2π/n_beams.
 *
 *   Geometry (set in pulsar_init from screen size; used by drawing):
 *     cx, cy     screen-cell centre.
 *     max_r     farthest radius any beam reaches; corner of the
 *                screen plus MAX_R_OVERSHOOT padding.
 *     r_step     max_r / N_RADII — gap between radial samples.
 *
 *   Shimmer cache:
 *     glyphs[ri][k]   one ASCII char per (radius, wake-slot). All
 *                     beams share the same cache (n_beams beams ×
 *                     N_RADII × WAKE_LEN+1 cells would be wasteful;
 *                     identical glyphs across beams is acceptable
 *                     because the eye can't track which beam shows
 *                     which character).
 *
 *   UI flags:
 *     theme_idx  0..THEME_COUNT-1.
 *     paused     when true, pulsar_tick returns early.
 */
typedef struct {
    /* rotation */
    float angle;
    float spin_rps;
    int   n_beams;

    /* geometry */
    int   cx, cy;
    float max_r;
    float r_step;

    /* shimmer cache */
    char  glyphs[N_RADII][WAKE_LEN + 1];

    /* UI */
    int   theme_idx;
    bool  paused;
} Pulsar;

/* ── §4.3 pulsar_init / pulsar_reset ──────────────────────────────── */

/*
 * pulsar_init — full reset: geometry + cache + zero angle. Called at
 * startup AND on resize.  pulsar_reset is the lighter version called
 * by the user's 'r' key — keeps user-tuned spin_rps and theme_idx.
 */
static void pulsar_init(Pulsar *p, int cols, int rows)
{
    p->angle    = 0.0f;
    p->spin_rps = SPIN_DEFAULT_RPS;
    p->n_beams  = BEAMS_DEFAULT;
    p->paused   = false;
    p->theme_idx = 0;

    p->cx       = cols / 2;
    p->cy       = rows / 2;

    /* Distance from centre to the farthest screen corner, in
     * isotropic cell units (ASPECT undoes the cell-aspect squish). */
    float hx = (float)cols * 0.5f;
    float hy = (float)rows * 0.5f / ASPECT;
    p->max_r  = sqrtf(hx * hx + hy * hy) * MAX_R_OVERSHOOT;
    p->r_step = p->max_r / (float)N_RADII;

    /* Seed the entire glyph cache. */
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            p->glyphs[ri][k] = rand_glyph();
}

static void pulsar_reset(Pulsar *p)
{
    p->angle  = 0.0f;
    p->n_beams = BEAMS_DEFAULT;
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            p->glyphs[ri][k] = rand_glyph();
}

/* ── §4.4 pulsar_shimmer — reroll most cache cells ────────────────── */

/*
 * Reroll most glyphs in the cache (1-in-SHIMMER_KEEP_ONE_IN survive).
 * Every cell that has its glyph swapped will visibly change next
 * frame as the beams sweep past it — the matrix shimmer.
 */
static void pulsar_shimmer(Pulsar *p)
{
    for (int ri = 0; ri < N_RADII; ri++)
        for (int k = 0; k <= WAKE_LEN; k++)
            if (rand() % SHIMMER_KEEP_ONE_IN != 0)
                p->glyphs[ri][k] = rand_glyph();
}

/* ── §4.5 pulsar_tick — advance angle, then shimmer ───────────────── */

/*
 * One per-frame physics step. With spin_rps in rotations per second,
 * the radians-per-second rate is 2π · spin_rps; we add omega · dt
 * radians to `angle` and wrap into [0, 2π). When paused, both the
 * angle and the cache freeze.
 */
static void pulsar_tick(Pulsar *p, float dt)
{
    if (p->paused) return;

    float omega = p->spin_rps * 2.0f * (float)M_PI;
    p->angle += omega * dt;

    /* Wrap into [0, 2π). while-loop handles arbitrarily large dt. */
    while (p->angle >= 2.0f * (float)M_PI) p->angle -= 2.0f * (float)M_PI;
    while (p->angle <  0.0f)               p->angle += 2.0f * (float)M_PI;

    pulsar_shimmer(p);
}

/* ── §4.6 pulsar_draw_beam — one beam + its angular wake ─────────── */

/*
 * Draw one rotating beam (head + WAKE_LEN trailing slots) at
 * `base_angle`.
 *
 *   Step 1: pre-compute direction vectors cw[k]/sw[k] for the
 *           WAKE_LEN+1 angular slots. Only 17 trig pairs needed per
 *           beam regardless of N_RADII.
 *
 *   Step 2: walk N_RADII radial samples outward. For each ri, paint
 *           wake slots in DESCENDING order (k = WAKE_LEN..0). The
 *           descending order is load-bearing: at small radius
 *           multiple slots can round to the same cell, and we want
 *           the BRIGHT HEAD to win — so it's painted LAST.
 */
static void pulsar_draw_beam(const Pulsar *p, float base_angle,
                             int cols, int rows)
{
    /* Step 1 — direction vectors per angular slot. */
    float cw[WAKE_LEN + 1], sw[WAKE_LEN + 1];
    for (int k = 0; k <= WAKE_LEN; k++) {
        float wa = base_angle - (float)k * WAKE_STEP;
        cw[k] = cosf(wa);
        sw[k] = sinf(wa) * ASPECT;       /* ASPECT BAKED IN */
    }

    /* Step 2 — radial samples × angular slots, dim-first per ri. */
    for (int ri = 0; ri < N_RADII; ri++) {
        float r = (float)(ri + 1) * p->r_step;

        for (int k = WAKE_LEN; k >= 0; k--) {
            int col = p->cx + (int)roundf(r * cw[k]);
            int row = p->cy + (int)roundf(r * sw[k]);
            if (col < 0 || col >= cols || row < 0 || row >= rows) continue;

            attr_t attr = wake_attr(k);
            attron(attr);
            mvaddch(row, col, (chtype)(unsigned char)p->glyphs[ri][k]);
            attroff(attr);
        }
    }
}

/* ── §4.7 pulsar_draw_core — the '@' on top of everything ─────────── */

static void pulsar_draw_core(const Pulsar *p, int cols, int rows)
{
    if (p->cx < 0 || p->cx >= cols || p->cy < 0 || p->cy >= rows) return;
    attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
    mvaddch(p->cy, p->cx, '@');
    attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* ── §4.8 pulsar_draw — orchestrator: N beams + core last ─────────── */

static void pulsar_draw(const Pulsar *p, int cols, int rows)
{
    /* N beams evenly spaced at 2π/N. Order between beams doesn't
     * matter because they share the same brightness ramp at each k. */
    float step = 2.0f * (float)M_PI / (float)p->n_beams;
    for (int b = 0; b < p->n_beams; b++)
        pulsar_draw_beam(p, p->angle + (float)b * step, cols, rows);

    /* Core LAST — always wins overlap. */
    pulsar_draw_core(p, cols, rows);
}

/* ── §4.9 input helpers (used by app_handle_key) ─────────────────── */

static void pulsar_change_spin(Pulsar *p, float delta_rps)
{
    p->spin_rps += delta_rps;
    if (p->spin_rps < SPIN_MIN_RPS) p->spin_rps = SPIN_MIN_RPS;
    if (p->spin_rps > SPIN_MAX_RPS) p->spin_rps = SPIN_MAX_RPS;
}

static void pulsar_change_beams(Pulsar *p, int delta)
{
    p->n_beams += delta;
    if (p->n_beams < BEAMS_MIN) p->n_beams = BEAMS_MIN;
    if (p->n_beams > BEAMS_MAX) p->n_beams = BEAMS_MAX;
}

static void pulsar_cycle_theme(Pulsar *p)
{
    p->theme_idx = (p->theme_idx + 1) % THEME_COUNT;
    theme_apply(p->theme_idx);
}

/* ===================================================================== */
/* §5  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use -1 background       */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0       PAIR_HUD  + A_BOLD  (yellow) — fps + state + params
 *   Bottom row  PAIR_HINT + A_BOLD  (cyan)   — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
static void screen_draw_hud(const Screen *sc, double fps, const Pulsar *p)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %5.2f rps  %d beam%s  [%s] %s ",
             fps, (double)p->spin_rps, p->n_beams,
             p->n_beams == 1 ? " " : "s",
             k_themes[p->theme_idx].name,
             p->paused ? "PAUSED " : "running");

    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  +/-:spin  []:beams  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §6  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

typedef struct {
    Pulsar                pulsar;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — recompute geometry + cache for the new screen
 * size, but preserve user-tuned spin_rps, n_beams, theme_idx, and
 * paused flag across the resize.
 */
static void app_do_resize(App *app)
{
    float saved_spin   = app->pulsar.spin_rps;
    int   saved_beams  = app->pulsar.n_beams;
    int   saved_theme  = app->pulsar.theme_idx;
    bool  saved_paused = app->pulsar.paused;

    screen_resize(&app->screen);
    pulsar_init(&app->pulsar, app->screen.cols, app->screen.rows);

    app->pulsar.spin_rps  = saved_spin;
    app->pulsar.n_beams   = saved_beams;
    app->pulsar.theme_idx = saved_theme;
    app->pulsar.paused    = saved_paused;
    app->need_resize      = 0;
}

/* Map one keypress to an action. Returns false on quit. */
static bool app_handle_key(App *app, int ch)
{
    Pulsar *p = &app->pulsar;
    switch (ch) {

    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ': case 'p': case 'P':
        p->paused = !p->paused;
        break;

    case 'r': case 'R':
        pulsar_reset(p);
        break;

    case '=': case '+':
        pulsar_change_spin(p, +SPIN_STEP_RPS);
        break;

    case '-':
        pulsar_change_spin(p, -SPIN_STEP_RPS);
        break;

    case ']':
        pulsar_change_beams(p, +1);
        break;

    case '[':
        pulsar_change_beams(p, -1);
        break;

    case 't': case 'T':
        pulsar_cycle_theme(p);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    pulsar_init(&app->pulsar, app->screen.cols, app->screen.rows);
    theme_apply(app->pulsar.theme_idx);
    hud_pairs_init();

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (4) advance pulsar (rotation + shimmer) */
        pulsar_tick(&app->pulsar, dt);

        /* (5) rolling fps display */
        fps_accum_ns += dt_ns;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC / 2) {
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        /* (6) draw + present */
        erase();
        pulsar_draw(&app->pulsar, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, fps_display, &app->pulsar);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
