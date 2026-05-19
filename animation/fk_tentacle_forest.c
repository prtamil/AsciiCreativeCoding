/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_tentacle_forest.c — eight ocean tentacles swaying in a current
 *
 * DEMO: Eight seaweed strands rooted along the bottom of the terminal
 *       sway left and right in a simulated underwater current. Each
 *       strand has its own phase and a tiny frequency detuning so the
 *       forest never lapses into mechanical lockstep. Motion is pure
 *       stateless forward kinematics — every frame is recomputed from
 *       a closed-form sine of wave_time alone.
 *
 * Study alongside: snake_forward_kinematics.c (path-following FK contrast)
 *                  fk_centipede.c             (same variable-timestep loop)
 *
 * Section map:
 *   §1  config       — all tunables in one place
 *   §2  clock        — monotonic clock + sleep (verbatim from framework)
 *   §3  color        — deep-sea 7-step palette + spec HUD/hint pairs
 *   §4  coords       — pixel↔cell aspect-ratio helpers
 *   §5  entity       — Tentacle: stateless FK chain + two-pass render
 *       §5a  tentacle_tick   — closed-form joint placement
 *       §5b  rendering helpers (seg_pair, seg_attr, seg_glyph)
 *       §5c  draw_segment_dense
 *       §5d  render_tentacle (orchestrator)
 *   §6  scene        — Scene wrapping N_TENTACLES, scene_init/tick/draw
 *   §7  screen       — ncurses double-buffer display layer
 *   §8  app          — signals, resize, main game loop
 *
 * Keys:  q / ESC      quit                space   pause / resume
 *        w / ↑        wave freq + 0.15    s / ↓   wave freq − 0.15
 *        d / →        amplitude + 0.10    a / ←   amplitude − 0.10
 *        [ / ]        time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/fk_tentacle_forest.c \
 *       -o fk_tentacle_forest -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Stateless forward kinematics for fixed-root chains.
 *                 For each tentacle, every joint position is recomputed
 *                 from scratch each frame by accumulating per-segment
 *                 bend angles up the chain:
 *                     δθᵢ              = amp · sin(ω·t + ph + i·PSEG)
 *                     cumulative_angle += δθᵢ
 *                     joint[i+1]       = joint[i]
 *                                      + seg_len · (cos cumul, sin cumul)
 *                 No history is needed because re-evaluating the formula
 *                 at the same wave_time yields identical positions —
 *                 the chain is fully determined by wave_time alone.
 *                 Per-strand root_phase + tiny freq_offset prevent the
 *                 eight chains from re-synchronising into lockstep.
 *
 * Data-structure: Tentacle holds the fixed root, two per-strand wave
 *                 constants (root_phase, freq_offset), and a single
 *                 joint[] array of (N_SEGS+1) pixel positions. Scene
 *                 owns N_TENTACLES of them plus shared wave parameters.
 *                 No trail buffer, no prev/cur snapshots, no alpha
 *                 interpolation — variable timestep makes them unneeded.
 *
 * Rendering     : Two-pass per tentacle. Pass 1 stamps direction glyphs
 *                 ('-' '/' '|' '\\') along each segment via dense bead-
 *                 stepping; the colour-pair gradient runs deep blue at
 *                 the root → bright yellow-green at the tip with depth-
 *                 cued attributes (A_DIM / A_NORMAL / A_BOLD). Pass 2
 *                 stamps bold node glyphs (#, O, o, ., *) at every joint
 *                 on top of the lines, giving the chain its knuckled
 *                 silhouette. A dim '~' seabed row anchors the scene.
 *
 * Performance   : Variable timestep at render rate — no fixed-step
 *                 accumulator, no alpha lerp, no prev_joint snapshots.
 *                 The whole simulation is closed-form trigonometry
 *                 (non-stiff), so variable dt is provably safe and
 *                 gives motion as smooth as the terminal can render.
 *                 Per frame: O(N_TENTACLES · N_SEGS) — 8 · 16 = 128
 *                 sinf/cosf calls, microseconds total.
 *
 * References
 * ──────────
 *   ── Forward kinematics (cumulative-angle chains) ─────────────────
 *   [1] Craig, J. J. (2005), "Introduction to Robotics: Mechanics
 *       and Control" (3rd ed.), Pearson — the canonical FK textbook;
 *       Ch. 3 derives the link-by-link composition that
 *       tentacle_tick() implements in 2-D.
 *   [2] Spong, M. W., Hutchinson, S. & Vidyasagar, M. (2005), "Robot
 *       Modeling and Control", Wiley — alternative canonical text;
 *       Ch. 3 is the cleanest derivation of cumulative-angle FK.
 *   [3] Denavit, J. & Hartenberg, R. S. (1955), "A kinematic
 *       notation for lower-pair mechanisms based on matrices",
 *       J. Appl. Mech. 22, pp. 215-221 — the DH formalism every
 *       multi-link FK algorithm follows; this file uses a
 *       restricted 2-D version (one DOF per joint, no twist).
 *
 *   ── Procedural / stateless motion ────────────────────────────────
 *   [4] Reynolds, C. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference — framework for
 *       stateless analytic motion. The "wander" behaviour is
 *       conceptually the per-segment δθᵢ used here.
 *       https://www.red3d.com/cwr/steer/
 *   [5] Perlin, K. (1995), "Real Time Responsive Animation with
 *       Personality", IEEE TVCG 1(1) — argues for closed-form
 *       sinusoidal animation layers; the per-strand `root_phase` +
 *       `freq_offset` design follows Perlin's "noise + phase
 *       stagger" recipe for non-synchronising groups.
 *
 *   ── Biological inspiration (continuum mechanics) ─────────────────
 *   [6] Sumbre, G. et al. (2001), "Control of octopus arm extension
 *       by a peripheral motor program", Science 293, pp. 1845-1848
 *       — biological reference for sinusoidal-wave propagation as a
 *       locomotor primitive in soft-bodied appendages.
 *   [7] Hirose, S. (1993), "Biologically Inspired Robots: Snake-like
 *       Locomotors and Manipulators", Oxford — engineering treatment
 *       of travelling-wave control; the i·PHASE_PER_SEG offset is a
 *       discrete version of Hirose's "serpenoid curve".
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *   [8] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com —
 *       articulates when fixed-step matters and when variable-step
 *       is the better tool; non-stiff sin-based sims fall in the
 *       latter camp.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *   [9] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       integer line drawing; we use the simpler parametric
 *       oversample (draw_segment_dense §5c) because float math is
 *       already paid for in tentacle_tick.
 *  [10] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the depth-cued glyph ramp.
 *  [11] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the newscr/curscr diff pipeline
 *       used in screen_present().
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [12] https://en.wikipedia.org/wiki/Forward_kinematics
 *  [13] https://en.wikipedia.org/wiki/Phase_(waves)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each tentacle is a chain of stiff segments hinged end-to-end, rooted
 * at a fixed point on the sea floor. At any moment, every segment has
 * a small bend δθᵢ relative to its parent, computed from a sine of
 * (wave_time, segment index, per-strand phase). The world-space
 * direction of segment i is the SUM of all bends from 0 through i, so
 * a wave that ripples through the bend values appears to ripple up the
 * physical chain. Nothing is stored between frames except wave_time.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a marionette with strings attached to every joint, and one
 * sinusoidal hand controls all the strings together — but with a small
 * delay (i·PSEG) for each joint up the chain. The lower joints jerk
 * first, the upper ones follow. That delay is what makes the wave
 * appear to *travel* along the tentacle rather than just shake the
 * whole rod. Per-strand `root_phase` shifts each marionette's hand
 * by a different starting angle; per-strand `freq_offset` shakes each
 * one a hair faster or slower so the eight marionettes never quite re-
 * synchronise.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. wave_time += dt   (the only persistent simulation state).
 *  3. For each tentacle k:
 *       a. joint[0] = (root_px, root_py)              (fixed anchor)
 *       b. cumulative_angle = −π/2                    (straight up)
 *       c. for i in 0..N_SEGS−1:
 *            δθᵢ = amp · sin((freq + freq_offset[k])·wave_time
 *                            + root_phase[k] + i·PHASE_PER_SEG)
 *            cumulative_angle += δθᵢ
 *            joint[i+1] = joint[i] + seg_len ·
 *                                    (cos cumulative_angle,
 *                                     sin cumulative_angle)
 *  4. Render two-pass: segment dense-fill glyphs first, then bold
 *     node markers on top. Repeat for all tentacles.
 *
 * KEY FORMULAS
 * ────────────
 *  Per-segment bend  : δθᵢ = amp · sin((freq + freq_offset)·wave_time
 *                                        + root_phase + i·PHASE_PER_SEG)
 *  Cumulative angle  : ang_i = base + Σ_{j<i} δθⱼ      (FK accumulation)
 *  Segment placement : joint[i+1] = joint[i]
 *                                 + seg_len · (cos ang_i, sin ang_i)
 *  Wave travel speed : freq / PHASE_PER_SEG  segments per second
 *                      (the same phase appears at increasing i over time)
 *  Total spatial span: N_SEGS · PHASE_PER_SEG = 7.2 rad ≈ 1.15 cycles
 *                      → roughly one S-curve visible from root to tip
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - High amplitude (above ~0.5 rad/seg) lets the cumulative angle
 *    wrap, so the chain can fold back on itself. Visible as a tight
 *    spiral when AMP is cranked. Not a bug — but the `AMP_MAX = 1.2`
 *    ceiling is intentionally just shy of the spiral threshold.
 *
 *  - Pause: scene_tick() returns early so wave_time stops advancing.
 *    Joints stay at the moment of pause. No prev/cur lerp needed.
 *
 *  - Resize: roots and seg_len_px must be recomputed for the new
 *    geometry. wave_time MUST be preserved so the animation continues
 *    from the same point in its cycle, not snapped back to t=0.
 *
 *  - Suspend / lid-close: dt can grow huge between frames; capped at
 *    100 ms so the tentacles don't suddenly flail through ~hundreds of
 *    radians of accumulated wave_time.
 *
 *  - Glyph aliasing: the four ASCII line glyphs sample a continuous
 *    angle. Near boundaries (22.5°, 67.5°, ...) the glyph flickers as
 *    the segment angle crosses. Folding to [0°, 180°) halves the
 *    boundary crossings; residual flicker visible only at very low
 *    time_scale.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: each tentacle shows roughly one visible S-curve
 *    (~1.15 cycles); the eight chains are clearly out of phase, never
 *    all bending the same direction at once.
 *  - Press space → tentacles freeze instantly. Un-pause → motion
 *    resumes exactly from the freeze frame (no jitter).
 *  - Crank amplitude with `d` → curves get more dramatic. The cumulative
 *    angle never makes the chain stretch (segments stay seg_len_px
 *    apart) regardless of amp.
 *  - Set frequency to 0 → all chains hold their last shape (sin
 *    argument stops advancing); they remain in whatever pose wave_time
 *    last placed them.
 *  - Press `[` to slow time (4× slow-mo) → motion is buttery; press
 *    `]` to fast-forward to 4× → still smooth (variable timestep
 *    guarantees this).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read fk_helloworld.c first if FK basics aren't
 *      automatic. fk_centipede.c is its closest cousin (also uses
 *      sinusoidal FK), but the centipede ALSO has a path-following
 *      body; this tentacle has ONLY the sinusoidal-FK part.
 *   2. §5 entity — the entire algorithm is one function:
 *        §5a tentacle_tick           ← THE HEART (read AFTER T1-T4)
 *        §5b-§5d rendering helpers   ← painter's two-pass
 *      The simulation has ONE persistent state variable:
 *      wave_time. Everything else is a closed-form derivation.
 *   3. §6 scene — orchestrator: spawn 8 tentacles with detuned
 *      phases and frequencies, tick them all, draw them all.
 *   4. §1-§4 + §7-§8 — config / clock / colour / screen / app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   joint[i]              i-th joint position (Vec2). 0 = root,
 *                         N_SEGS = tip.
 *   wave_time             master clock. INCREMENTS BY DT per frame.
 *                         Only persistent state.
 *   freq, amp             shared wave parameters (Hz-ish, radians).
 *   root_phase[k]         per-tentacle phase offset (radians).
 *   freq_offset[k]        per-tentacle frequency detune (Hz).
 *   PHASE_PER_SEG         per-segment phase advance — controls
 *                         how the wave PROPAGATES along the chain.
 *
 * Background you need
 * ───────────────────
 *   - The FK primitive (fk_helloworld T2 + T3).
 *   - Sinusoid arguments: sin(ω·t + φ) at given t shifts left/right
 *     with φ.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Fluid dynamics. The "current" is fake — there's no fluid
 *     simulation, just sin().
 *   - Mass-spring chains, Verlet, soft-body physics. The chain is
 *     RIGID with closed-form bend angles, not a deformable body.
 *   - Per-frame state. wave_time is enough.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a stateless FK tentacle from first
 * principles.
 *
 *   T1  Why STATELESS — the entire chain from one number (wave_time)
 *   T2  Per-segment bend angles — the sinusoid that travels
 *   T3  Cumulative-angle FK — generalising fk_helloworld T3 to N links
 *   T4  Phase detuning — eight tentacles, no synchronisation
 *   T5  When to fold in trail-buffer FK (and why this file doesn't)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY STATELESS — THE ENTIRE CHAIN FROM ONE NUMBER (wave_time)
 * ────────────────────────────────────────────────────────────────
 * "Stateful" simulation: each frame derives the next frame from
 * the previous frame's positions. Verlet integration, mass-spring,
 * trail buffers — all stateful.
 *
 * "Stateless" simulation: each frame's positions are computed
 * directly from a single time variable, ignoring whatever state
 * was visible last frame. tentacle_tick is stateless.
 *
 *     joint[0..N_SEGS] = pure_function(wave_time, per-tentacle phase)
 *
 * Properties:
 *
 *   + No accumulation drift. Re-evaluating at the same wave_time
 *     produces bit-identical results.
 *   + Variable timestep is automatically safe — there's no
 *     numerical-integration scheme to destabilise.
 *   + Fast forward / fast backward (set wave_time = anything)
 *     is trivial. Pause = stop incrementing wave_time.
 *   + No initial condition required: at t = 0 the formula has
 *     a defined output for every joint.
 *
 *   − The motion has to be EXPRESSIBLE as a closed form. If you
 *     want behaviour that responds to obstacles, contacts, or
 *     external forces, stateless is too constrained.
 *
 * Anything that LOOKS like a wave (oscillation, sway, breathing,
 * orbit) is best done stateless. Anything that NEEDS history
 * (trail, collision response, locomotion) needs state.
 *
 * T2  PER-SEGMENT BEND ANGLES — THE SINUSOID THAT TRAVELS
 * ───────────────────────────────────────────────────────
 * The simulation drives ONE thing per segment: its bend angle
 * relative to its parent.
 *
 *     δθ_i = amp · sin( freq · wave_time
 *                     + root_phase
 *                     + i · PHASE_PER_SEG )
 *
 * Three things in the sin argument:
 *
 *   freq · wave_time      ADVANCE of the wave over time
 *   root_phase            per-tentacle starting offset (T4)
 *   i · PHASE_PER_SEG     spatial phase shift along the chain
 *
 * The third term is what makes the wave TRAVEL up the chain.
 * At wave_time = 0 segment i has bend amp · sin(i · PHASE_PER_SEG)
 * — a SPATIAL pattern. As wave_time grows, that pattern shifts:
 * the value previously at segment i appears at segment i ± Δi
 * later. The wave RIDES UP the chain.
 *
 * Wave speed in segments per second:
 *   d(arg)/dt = freq             (advance per second)
 *   d(arg)/d(i) = PHASE_PER_SEG  (advance per segment)
 *   speed = freq / PHASE_PER_SEG (segments per second)
 *
 * Increase freq → wave runs faster up the chain. Increase
 * PHASE_PER_SEG → wave compresses (more bends per chain).
 * Decrease either → wave stretches.
 *
 * T3  CUMULATIVE-ANGLE FK — GENERALISING fk_helloworld T3 TO N LINKS
 * ──────────────────────────────────────────────────────────────────
 * Recall fk_helloworld T3 — for a 2-link arm:
 *
 *   joint[1] = joint[0] + L · (cos θ₁,         sin θ₁)
 *   joint[2] = joint[1] + L · (cos(θ₁ + θ₂),   sin(θ₁ + θ₂))
 *
 * The pattern: each joint's ABSOLUTE angle is the SUM of all
 * previous bends. For N links:
 *
 *     ang_0 = base direction (here: −π/2, "straight up")
 *     ang_i = ang_(i-1) + δθ_i              ← accumulate
 *     joint[i+1] = joint[i] + seg_len · (cos ang_i, sin ang_i)
 *
 * That's tentacle_tick in §5a. One running cumulative_angle
 * variable, walked forward through every segment.
 *
 * In code:
 *
 *     joint[0] = root_pos
 *     ang = -π/2                       ← upward
 *     for i in 0..N_SEGS-1:
 *       δθ = amp · sin(...)
 *       ang += δθ
 *       joint[i+1] = joint[i] + seg_len · (cos ang, sin ang)
 *
 * No matter how long the chain, FK is one loop and one running
 * sum. snake_forward_kinematics.c uses the same trick at 80
 * segments.
 *
 * T4  PHASE DETUNING — EIGHT TENTACLES, NO SYNCHRONISATION
 * ────────────────────────────────────────────────────────
 * Eight tentacles with IDENTICAL frequency would all reach the
 * same point in their cycle simultaneously, producing visible
 * lockstep ("seaweed flashmob"). Two cheap defences:
 *
 *   PHASE OFFSET (root_phase[k])
 *     Each tentacle starts at a different point in its cycle.
 *     Eight evenly-spaced offsets ∈ [0, 2π) give all eight a
 *     different starting bend without changing the SHAPE of
 *     each tentacle's individual motion.
 *
 *   FREQUENCY DETUNING (freq_offset[k])
 *     Each tentacle uses a SLIGHTLY different frequency
 *     (freq + freq_offset[k]). Without detuning, two tentacles
 *     that start out of phase will RE-SYNCHRONISE after enough
 *     time because they have the same period — the lockstep
 *     would just be delayed, not prevented. Detuning ensures
 *     no two tentacles ever return to the same relative phase.
 *
 * The freq offsets are TINY (~1% of base freq) so each tentacle
 * still looks like the same SPECIES — they're just slightly
 * different individuals. Same trick as in detuning oscillator
 * banks in audio synthesis.
 *
 * T5  WHEN TO FOLD IN TRAIL-BUFFER FK (AND WHY THIS FILE DOESN'T)
 * ───────────────────────────────────────────────────────────────
 * fk_centipede.c uses TWO FK techniques:
 *
 *   - Path-following body (trail buffer + arc-length sampling)
 *   - Sinusoidal legs (this file's technique)
 *
 * Why two? Because a centipede needs to MOVE, and stateless FK
 * can't drive locomotion — there's no concept of "where I am"
 * other than the deterministic formula. The trail buffer
 * handles position; the sinusoid handles articulation.
 *
 * fk_tentacle_forest doesn't move. The roots are bolted to the
 * sea floor; only the body articulates. So statefulness isn't
 * needed — sinusoidal FK alone is enough.
 *
 * Decision tree:
 *
 *   needs to MOVE in space?           → add trail buffer (centipede)
 *   needs RESPONSE to environment?    → add IK or steering (boids)
 *   needs DEFORMATION (squish)?       → add Verlet (ragdoll)
 *   none of those — pure articulation → stateless FK (this file)
 *
 * The dumbest thing that works wins. Tentacles in seaweed:
 * sinusoidal FK is plenty.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/* M_PI is a POSIX extension, not standard C99/C11 — provide a fallback
 * so the build never fails on strict-conformance toolchains. */
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
/* §1  config                                                             */
/* ===================================================================== */

/* All magic numbers live here. Never scatter literals through the code. */
enum {
    /* Render frame-rate target. Variable-timestep simulation, so the
     * only thing this controls is the sleep cap at end of frame. */
    TARGET_FPS    = 60,

    /* HUD layout. */
    HUD_COLS      = 96,    /* status-bar byte budget        */
    FPS_UPDATE_MS = 500,   /* fps display refresh interval  */

    /* ncurses pair IDs. Pairs 1..N_PAIRS = body gradient (root→tip);
     * PAIR_HUD/PAIR_HINT are reserved per CLAUDE.md HUD spec. */
    N_PAIRS   = 7,
    PAIR_HUD  = 8,
    PAIR_HINT = 9,

    /*
     * N_TENTACLES — strands rooted along the sea floor. 8 fills an
     * 80–220-column terminal comfortably with even spacing.
     *
     * N_SEGS — rigid segments per chain. More segments → smoother
     * curvature; 16 at the dynamic seg_len_px gives one visible S-curve
     * with no visible faceting at typical terminal sizes.
     */
    N_TENTACLES = 8,
    N_SEGS      = 16,
};

/*
 * DRAW_STEP_PX — pixel stride for dense glyph stamping in §5c. Must be
 * < CELL_W (8) so a near-horizontal segment never skips a column.
 *
 * 5 px ≈ 0.625 cell widths — guarantees every cell the segment crosses
 * is sampled at least once, while leaving texture sparse enough that
 * the §5d node markers can read through the fill clearly.
 */
#define DRAW_STEP_PX   5.0f

/*
 * PHASE_PER_SEG — per-segment phase advance (radians). Determines how
 * fast the wave appears to travel up the chain.
 *
 * Geometric interpretation:
 *   0.0   → all segments in phase → chain bends as a rigid rod
 *   π/2   → quarter wavelength per segment → tight zigzag
 *   0.45  → ~1/14 wavelength per segment → one gentle S-curve over
 *           N_SEGS=16 (16·0.45 = 7.2 rad ≈ 1.15 cycles per chain)
 */
#define PHASE_PER_SEG  0.45f

/*
 * Wave amplitude and frequency — user-tunable at runtime.
 *
 *   AMP   ≈ 0.28 rad ≈ 16° peak per-segment bend at default. With 16
 *         segments and PHASE_PER_SEG distributing the bends, the typical
 *         tip deflection is around ±2 rad (≈115° from vertical) — vigorous
 *         but non-spiralling sway.
 *
 *   FREQ  ≈ 0.8 rad/s → period 2π/0.8 ≈ 7.9 s per oscillation, matching
 *         the leisurely pace of real seaweed in a gentle tidal current.
 */
#define AMP_DEFAULT    0.28f
#define AMP_MIN        0.0f
#define AMP_MAX        1.2f
#define FREQ_DEFAULT   0.8f
#define FREQ_MIN       0.1f
#define FREQ_MAX       5.0f

/* Per-strand frequency detuning amplitude. Each tentacle gets a tiny
 * additive offset on the global frequency so they never re-synchronise.
 * Magnitude 0.04 → adjacent strands drift ~0.32 rad apart per cycle. */
#define FREQ_OFFSET_MAG  0.04f

/*
 * Time scale — user-controlled simulation speed multiplier.
 * 0.25× to 4×, default 1×. Stepped by ×/÷ TIME_SCALE_STEP via [/].
 */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions — the aspect-ratio bridge between physics
 * and display. 1 px represents the same physical distance in x and y;
 * a typical cell is ~2× taller than wide. */
#define CELL_W   8
#define CELL_H  16

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);    /* never goes backward */
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;                   /* over-budget frame: skip */
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color — deep-sea 7-step palette + spec-fixed HUD pairs            */
/* ===================================================================== */

/*
 * The body gradient runs deep blue at the root → bright yellow-green
 * at the tip, evoking water depth (cold/dark below, sunlit/warm above).
 *
 * All entries sit in the bright half of the 256-colour space (>= 24 in
 * the cube) per CLAUDE.md theme palette brightness rule, so even the
 * dim root segments stay visible under A_DIM rendering.
 *
 * HUD/HINT pairs are theme-independent (CLAUDE.md HUD spec) — bright
 * yellow status, bright cyan hint, both on default background so they
 * stay readable against any animation behind them.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(1,  24, -1);   /* deep blue       — root, sea floor    */
        init_pair(2,  27, -1);   /* medium blue                          */
        init_pair(3,  33, -1);   /* cyan-blue                            */
        init_pair(4,  51, -1);   /* bright cyan     — mid-body           */
        init_pair(5,  86, -1);   /* cyan-green                           */
        init_pair(6, 118, -1);   /* yellow-green                         */
        init_pair(7, 154, -1);   /* bright yellow-green — tip, sunlit    */
    } else {
        /* 8-color fallback — coarser gradient, still directionally right. */
        init_pair(1, COLOR_BLUE,  -1);
        init_pair(2, COLOR_BLUE,  -1);
        init_pair(3, COLOR_CYAN,  -1);
        init_pair(4, COLOR_CYAN,  -1);
        init_pair(5, COLOR_CYAN,  -1);
        init_pair(6, COLOR_GREEN, -1);
        init_pair(7, COLOR_GREEN, -1);
    }

    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * All entity positions live in square pixel space (1 unit = 1 px).
 * Only at draw time do these helpers convert to cell coordinates,
 * undoing the 8:16 cell aspect ratio.
 *   cell = floor(px / CELL_DIM + 0.5)    — nearest-integer rounding
 */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Tentacle: stateless FK chain                             */
/* ===================================================================== */

/*
 * Vec2 — a 2-D position in PIXEL space (sub-cell precision).
 *
 * Intent
 *   All §5 FK math lives in pixel space. Each character cell is
 *   CELL_W × CELL_H sub-pixels (8 × 16), giving sub-cell resolution
 *   so a swaying tentacle reads as continuous motion instead of
 *   jumping cell-to-cell. The conversion to cell space happens only
 *   inside draw_segment_dense (§5c), per the project's "one
 *   conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → down the screen).
 *
 *   Angles are MATH-CONVENTION (positive CCW from +X). Because y
 *   points DOWN on screen, a math-CCW rotation displays as CW; this
 *   matters only when reading the −π/2 seed in tentacle_tick (which
 *   points UP on screen), nowhere else.
 *
 * Why a value type
 *   Sized 8 bytes; passes in registers on every modern ABI. Each
 *   tentacle stores (N_SEGS+1) joints by value — no allocations,
 *   cache-friendly.
 *
 * Why not just two floats inline in Tentacle
 *   Named (x, y) gives compile-time type-checking against accidental
 *   (col, row) confusion and lets tentacle_tick read like the math.
 *
 * References [1] Craig §2 "Spatial descriptions".
 */
typedef struct {
    float x;   /* eastward  pixel coordinate (positive → right) */
    float y;   /* southward pixel coordinate (positive → down)  */
} Vec2;

/*
 * Tentacle — one seaweed strand: a stateless FK chain.
 *
 * Intent
 *   A tentacle is a closed-form function of (wave_time, shared
 *   amplitude, shared frequency). Each frame the chain is rebuilt
 *   FROM SCRATCH; no integrator state, no prev/cur snapshots, no
 *   trail buffer. The math:
 *
 *       δθᵢ              = amp · sin((ω + freq_offset)·t
 *                                    + root_phase + i·PHASE_PER_SEG)
 *       cumulative_angle = −π/2 + Σᵢ δθᵢ          (−π/2 = straight up)
 *       joint[i+1]       = joint[i] + seg_len·(cos, sin)(cumulative)
 *
 *   The two per-strand wave constants (root_phase, freq_offset) are
 *   what BREAK lockstep — without them, eight identical chains would
 *   sway in mechanical synchrony.
 *
 * Why "fixed at init" + "computed each frame" split
 *   The first four fields (root_px, root_py + two wave constants)
 *   are set ONCE in scene_init and never mutate. The joint[] array
 *   is the OUTPUT of tentacle_tick() — rewritten in full each frame.
 *   This split makes the closed-form nature obvious: no state
 *   crosses frames except wave_time, which lives in Scene.
 *
 * Why joint[] stores N_SEGS+1 entries
 *   A chain with N segments has N+1 joints (one per segment end +
 *   the root). joint[0] is the fixed root; joint[N_SEGS] is the tip.
 *
 * Why freq_offset is symmetric around 0 (set in scene_init)
 *   So the AVERAGE wave frequency across the forest equals the
 *   user-tunable `Scene::frequency`. Asymmetric offsets would drift
 *   the average and confuse the HUD readout.
 *
 * Why root_phase is uniformly distributed over [0, 2π)
 *   Maximises initial desynchronisation (any two strands start
 *   max-separated in phase). See [5] Perlin §"Noise + phase stagger".
 *
 * References [1] Craig §3 (chain FK); [4] Reynolds "wander" for the
 *   stateless analytic motion pattern; [7] Hirose for the
 *   travelling-wave (serpenoid) inspiration behind i·PHASE_PER_SEG.
 */
typedef struct {
    /* ── Fixed-at-init: anchor + per-strand wave constants ─────── *
     * Set ONCE in scene_init; never mutated thereafter. The two   *
     * wave constants are what give each strand its individual     *
     * "personality" and prevent forest-wide lockstep.             */
    float root_px;        /* root x in pixel space (fixed anchor)    */
    float root_py;        /* root y in pixel space (fixed anchor)    */
    float root_phase;     /* phase stagger (rad), uniform on [0,2π)  */
    float freq_offset;    /* tiny freq detuning (rad/s), symmetric   *
                           * about 0 across the forest               */

    /* ── Computed each frame: the FK chain output ──────────────── *
     * Rewritten in full by tentacle_tick(); no value carries       *
     * across frames. [0] = root (pinned), [N_SEGS] = tip.          */
    Vec2  joint[N_SEGS + 1];
} Tentacle;

/* ── §5a  tentacle_tick ─────────────────────────────────────────────── */

/*
 * tentacle_tick — recompute all joint positions from wave_time.
 *
 * The cumulative-angle FK trick: each segment's WORLD direction is the
 * SUM of all upstream bends. By seeding cumulative_angle at −π/2
 * ("up" in pixel space) and adding δθᵢ each iteration, every segment
 * inherits all the bends from segments below it — that's how a wave at
 * the root propagates to the tip without any explicit force model.
 *
 * Per-strand freq_offset and root_phase enter the sine argument so
 * each chain oscillates at a slightly different rate and starting phase.
 */
static void tentacle_tick(Tentacle *t,
                          float wave_time,
                          float amplitude,
                          float frequency,
                          float seg_len_px)
{
    t->joint[0].x = t->root_px;
    t->joint[0].y = t->root_py;

    float cumulative_angle = -(float)M_PI * 0.5f;   /* start pointing up */

    for (int i = 0; i < N_SEGS; i++) {
        float delta = amplitude
                    * sinf((frequency + t->freq_offset) * wave_time
                           + t->root_phase
                           + (float)i * PHASE_PER_SEG);
        cumulative_angle += delta;

        t->joint[i + 1].x = t->joint[i].x
                          + seg_len_px * cosf(cumulative_angle);
        t->joint[i + 1].y = t->joint[i].y
                          + seg_len_px * sinf(cumulative_angle);
    }
}

/* ── §5b  rendering helpers ─────────────────────────────────────────── */

/* seg_pair — body gradient pair for segment i.
 *   i = 0           → pair 1       (root, deep blue)
 *   i = N_SEGS - 1  → pair N_PAIRS (tip, bright yellow-green)
 * Integer linear interpolation across the gradient. */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/* seg_attr — depth-cued attribute for segment i.
 *   root quarter : A_DIM    (deep water absorbs light)
 *   middle half  : A_NORMAL (neutral)
 *   tip quarter  : A_BOLD   (sunlit shallows, vibrant tip)         */
static attr_t seg_attr(int i)
{
    if (i <     N_SEGS / 4) return A_DIM;
    if (i > 3 * N_SEGS / 4) return A_BOLD;
    return A_NORMAL;
}

/* node_marker — bold joint glyph by position along chain.
 *   0          '#'  root anchor — thick, grounded
 *   upper qtr  'O'  lower body  — wide, fat node
 *   middle     'o'  mid body    — medium node
 *   lower qtr  '.'  upper body  — small, wispy
 *   N_SEGS     '*'  tip         — bright spark, free end           */
static chtype node_marker(int i)
{
    if (i == 0)                  return (chtype)(unsigned char)'#';
    if (i <=     N_SEGS / 4)     return (chtype)(unsigned char)'O';
    if (i <= 3 * N_SEGS / 4)     return (chtype)(unsigned char)'o';
    if (i <  N_SEGS)             return (chtype)(unsigned char)'.';
    return                              (chtype)(unsigned char)'*';
}

/*
 * seg_glyph — best ASCII direction glyph for vector (dx, dy).
 *
 * Glyphs partition the angular circle, folded to [0°, 180°) since each
 * glyph is symmetric under 180° rotation:
 *   '-'  near-horizontal     ( 0°,  22.5° ) ∪ (157.5°, 180°)
 *   '\'  down-right diagonal ( 22.5°,  67.5° )
 *   '|'  near-vertical       ( 67.5°, 112.5° )
 *   '/'  down-left diagonal  (112.5°, 157.5° )
 *
 * dy is negated before atan2f so the angle matches visual direction
 * (terminal y grows down; math y grows up).
 */
static chtype seg_glyph(float dx, float dy)
{
    float ang = atan2f(-dy, dx);
    float deg = ang * (180.0f / (float)M_PI);
    if (deg <    0.0f) deg += 360.0f;
    if (deg >= 180.0f) deg -= 180.0f;

    if (deg < 22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg < 67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                  return (chtype)(unsigned char)'|';
    return                             (chtype)(unsigned char)'/';
}

/* ── §5c  draw_segment_dense ────────────────────────────────────────── */

/*
 * draw_segment_dense — stamp a direction glyph at every cell the line
 * a→b passes through.
 *
 * WHY DENSE STEPPING? A cell is CELL_W=8 px wide. Drawing only at the
 * endpoints leaves visible gaps on segments longer than one cell.
 * Stepping every DRAW_STEP_PX=5 px (< CELL_W) guarantees at least one
 * sample per cell along the segment.
 *
 * DEDUP CURSOR (prev_cx, prev_cy): caller-owned so it persists across
 * the segments of one tentacle. Initialise to a sentinel (-9999) before
 * the first segment so the first cell always draws. Sharing across
 * segments prevents double-stamping at segment boundaries.
 *
 * Rows 0 and rows-1 are skipped — those are the HUD bars.
 */
static void draw_segment_dense(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows,
                                int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;       /* degenerate: nothing to draw */

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx;
        *prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* ── §5d  render_tentacle ───────────────────────────────────────────── */

/* draw_tentacle_lines — pass 1: dense direction-glyph fill, root → tip.
 * Drawing root → tip means upper segments overwrite lower ones where
 * the chain curves back on itself (tight curls at high amplitude). */
static void draw_tentacle_lines(const Tentacle *t, WINDOW *w,
                                int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;   /* dedup cursor */
    for (int i = 0; i < N_SEGS; i++) {
        draw_segment_dense(w, t->joint[i], t->joint[i + 1],
                           seg_pair(i), seg_attr(i),
                           cols, rows, &prev_cx, &prev_cy);
    }
}

/* draw_tentacle_nodes — pass 2: bold node glyph at each joint, on top
 * of the line fill. Marker shape encodes position along the chain (#
 * grounded root → * bright tip), reinforcing the size-gradient look. */
static void draw_tentacle_nodes(const Tentacle *t, WINDOW *w,
                                int cols, int rows)
{
    for (int i = 0; i <= N_SEGS; i++) {
        int cx = px_to_cell_x(t->joint[i].x);
        int cy = px_to_cell_y(t->joint[i].y);
        if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1) continue;

        int    pair  = seg_pair(i < N_SEGS ? i : N_SEGS - 1);
        chtype glyph = node_marker(i);

        wattron(w, COLOR_PAIR(pair) | A_BOLD);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | A_BOLD);
    }
}

/* render_tentacle — orchestrator. Painter's order: lines first (so node
 * markers always overwrite line glyphs at joint cells), nodes on top. */
static void render_tentacle(const Tentacle *t, WINDOW *w,
                            int cols, int rows)
{
    draw_tentacle_lines(t, w, cols, rows);
    draw_tentacle_nodes(t, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — the tentacle forest.
 *
 * Intent
 *   The eight tentacles share three wave parameters (wave_time,
 *   amplitude, frequency) plus a screen-derived seg_len_px. Per-
 *   strand variation comes only from Tentacle::{root_phase,
 *   freq_offset} — see Tentacle struct doc. The Scene IS the
 *   forest: one composition root, scene_init / scene_tick /
 *   scene_draw all take Scene* and nothing else of the world.
 *
 * Why one master clock for the forest
 *   wave_time is the SINGLE persistent simulation state — all eight
 *   tentacles read the same clock value each frame. This is what
 *   makes the simulation closed-form: same wave_time ⇒ same forest,
 *   regardless of how it got there. Compare with fk_centipede.c
 *   where each segment has its own trail history.
 *
 * Simulation vs Rendering locality
 *   ── Simulation state ── read+written by scene_tick:
 *      t[]            N_TENTACLES strands; each holds its FK chain.
 *      wave_time      Master clock; the ONLY value that crosses
 *                     frames. Frozen when paused.
 *      amplitude      Peak per-segment bend (rad). User-tunable
 *                     via a/d; broadcast to every tentacle each tick.
 *      frequency      Base oscillation rate (rad/s). User-tunable
 *                     via w/s; broadcast similarly.
 *      seg_len_px     Segment length (pixels). Recomputed in
 *                     scene_init + on resize so tentacles stay
 *                     proportional to terminal height (~55%).
 *
 *   ── Control / render-gate state ──
 *      paused         Freezes scene_tick (wave_time stops). Render
 *                     still runs — the forest just sits frozen.
 *
 * Why seg_len_px is SIM state (not render state)
 *   Because it changes the SHAPE the tentacle takes — joint
 *   positions multiply by it. It's geometry the solver uses, not a
 *   camera choice. Resize triggers a recompute (in scene_init).
 *
 * Why no theme_idx / colour state
 *   Themes are baked at compile time via §3 color_init(); no
 *   runtime cycling in this demo. If a future variant adds theme
 *   cycling, it would slot in next to `paused` as render state.
 *
 * Things that DO NOT live here
 *   - Per-strand wave constants (root_phase, freq_offset) → Tentacle.
 *   - Tunable defaults (AMP_DEFAULT, FREQ_DEFAULT, FREQ_OFFSET_MAG)
 *     → §1 config (compile-time constants).
 *   - fps / time_scale → §8 App (frame-timing concern, not part of
 *     the world being simulated).
 *   - Terminal extent (cols, rows) → §7 Screen.
 *   - Signal flags → §8 App.
 *
 * References [1] Craig §3 (chain FK); [4] Reynolds for the
 *   stateless-motion framework; [8] Fiedler for the timestep choice.
 */
typedef struct {
    /* ── Simulation state ─────────────────────────────────────── */
    Tentacle t[N_TENTACLES];   /* the forest                        */
    float wave_time;           /* master sim clock (s)              */
    float amplitude;           /* peak per-segment bend (rad)       */
    float frequency;           /* base oscillation rate (rad/s)     */
    float seg_len_px;          /* segment length (px); from screen  */

    /* ── Control / render-gate ────────────────────────────────── */
    bool  paused;              /* freezes scene_tick; render unchanged */
} Scene;

/*
 * scene_init — distribute roots evenly along the sea floor and assign
 * per-strand phase + frequency offsets.
 *
 * Init-site choices (rationale for per-strand wave constants lives
 * in the Tentacle struct doc):
 *  - Root spacing uses (N_TENTACLES + 1) divisor so no strand sits at
 *    the screen edge — fractions 1/9, 2/9, ... 8/9 give equal margins.
 *  - root_py = bottom − 4 px places roots inside the seabed '~' row.
 *  - seg_len_px = 55% of screen height divided across N_SEGS — tips
 *    visually terminate near mid-screen even at full sway amplitude.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    sc->amplitude  = AMP_DEFAULT;
    sc->frequency  = FREQ_DEFAULT;
    sc->wave_time  = 0.0f;
    sc->paused     = false;
    sc->seg_len_px = (float)(rows * CELL_H) * 0.55f / (float)N_SEGS;

    float screen_wpx = (float)(cols * CELL_W);
    float root_py    = (float)(rows * CELL_H) - 4.0f;

    for (int i = 0; i < N_TENTACLES; i++) {
        Tentacle *t = &sc->t[i];

        t->root_px     = (float)(i + 1) * screen_wpx
                       / (float)(N_TENTACLES + 1);
        t->root_py     = root_py;
        t->root_phase  = (float)i * 2.0f * (float)M_PI
                       / (float)N_TENTACLES;
        t->freq_offset = ((float)i - (float)N_TENTACLES * 0.5f)
                       * FREQ_OFFSET_MAG;

        /* Seed all joints straight up — overwritten on first tick. */
        for (int k = 0; k <= N_SEGS; k++) {
            t->joint[k].x = t->root_px;
            t->joint[k].y = t->root_py - (float)k * sc->seg_len_px;
        }
    }
}

/*
 * scene_tick — one variable-timestep update. dt is the wall-clock
 * delta scaled by the caller's time_scale. When paused, do nothing —
 * wave_time is frozen, so the next render produces an identical frame.
 */
static void scene_tick(Scene *sc, float dt)
{
    if (sc->paused) return;
    sc->wave_time += dt;
    for (int i = 0; i < N_TENTACLES; i++) {
        tentacle_tick(&sc->t[i], sc->wave_time,
                      sc->amplitude, sc->frequency, sc->seg_len_px);
    }
}

/* draw_seabed — dim '~' row beneath the tentacles for atmosphere.
 * The '~' is the traditional ASCII wave glyph. A_DIM keeps it visually
 * receding behind the bright tentacle roots. */
static void draw_seabed(WINDOW *w, int cols, int rows)
{
    int seabed_row = rows - 2;     /* rows-1 is the hint bar */
    if (seabed_row < 1) return;

    wattron(w, COLOR_PAIR(1) | A_DIM);
    for (int c = 0; c < cols; c++)
        mvwaddch(w, seabed_row, c, (chtype)(unsigned char)'~');
    wattroff(w, COLOR_PAIR(1) | A_DIM);
}

/* scene_draw — read-only render: tentacles + seabed atmosphere. */
static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_TENTACLES; i++)
        render_tentacle(&sc->t[i], w, cols, rows);
    draw_seabed(w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal-extent snapshot in CHARACTER CELLS.
 *
 * Intent
 *   Caches the current terminal size so every §6 entry point reads
 *   (cols, rows) as plain ints rather than re-querying ncurses each
 *   frame. Refreshed only when SIGWINCH sets App::need_resize, then
 *   propagated to scene_init via app_do_resize.
 *
 * Why a separate struct (not just two ints in App)
 *   Resize logic (endwin + refresh + getmaxyx) touches NOTHING in App
 *   except this struct. Carving it out makes screen_resize pure and
 *   isolates the ncurses dependency from the simulation layer.
 *
 * Why cells, not pixels
 *   ncurses' coordinate system is cells. Pixel space (CELL_W ×
 *   CELL_H sub-pixels per cell) lives only inside §5 — converted at
 *   the draw boundary, per the project's "one conversion point" rule.
 *
 * References [11] Raymond, NCURSES Programming HOWTO.
 */
typedef struct {
    int cols;   /* terminal width  in CHARACTER CELLS */
    int rows;   /* terminal height in CHARACTER CELLS */
} Screen;

/* The non-obvious call here is typeahead(-1): without it, ncurses peeks
 * at stdin during output writes, which can tear frames mid-update. */
static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* SIGWINCH path: endwin()+refresh() forces ncurses to re-read LINES/COLS
 * from the kernel; we then sample the fresh dimensions. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — compose one full frame:
 *   erase → tentacles + seabed → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any animation
 * behind them. NEVER use A_DIM on the hint strip — it disappears on
 * bright frames.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %.1ffps  %.2fx  amp:%.2f  freq:%.2f  %s ",
             fps, time_scale, sc->amplitude, sc->frequency,
             sc->paused ? "PAUSED" : "swaying");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:freq  a/d:amp  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level container; everything outside the forest.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the loop-control flags into one record so main() reads as
 *   four-line phases: init / service signals / step+draw / shutdown.
 *   Declared file-scope (g_app) so signal handlers — which cannot
 *   take a user argument — can write `running` and `need_resize`
 *   without globals scattered through the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated (§6)
 *      screen      — the terminal extent it draws to (§7)
 *
 *   ── Loop control ─── verbs the loop reads each frame
 *      time_scale  — wall-clock dt multiplier ([ / ] adjust)
 *      running     — clear → loop exits; set by SIGINT/SIGTERM
 *      need_resize — set by SIGWINCH; cleared after Screen refresh
 *
 * Why volatile sig_atomic_t (not bool, not int)
 *   `volatile`    : the compiler must not cache the flag across a
 *                   signal-handler write — every loop iteration must
 *                   re-read it from memory.
 *   `sig_atomic_t`: POSIX-guaranteed atomic with respect to async
 *                   signals; a plain `int` could be observed half-
 *                   written on architectures where stores are split.
 *   See [11] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no other
 *     code path needs them.
 *   - Forest tuning values — those are user-state in Scene.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the forest (§6)                       */
    Screen screen;             /* terminal extent (§7)                  */

    /* ── Loop control ─────────────────────────────────────────── */
    float                 time_scale;   /* dt multiplier; 1.0 = realtime */
    volatile sig_atomic_t running;      /* main loop predicate            */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending               */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize — handle a pending SIGWINCH.
 *
 * Geometry (root_px, root_py, seg_len_px) is recomputed for the new
 * screen size, but wave_time + user-tunable params (amplitude,
 * frequency) are PRESERVED so animation continues from the same point
 * in its cycle rather than snapping back to t=0.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    float saved_wave_time = app->scene.wave_time;
    float saved_amp       = app->scene.amplitude;
    float saved_freq      = app->scene.frequency;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    app->scene.wave_time = saved_wave_time;
    app->scene.amplitude = saved_amp;
    app->scene.frequency = saved_freq;

    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Letter aliases (w/s, a/d) are provided because some terminals
 * swallow arrow-key escape sequences.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sc->paused = !sc->paused; break;

    case KEY_UP:   case 'w': case 'W':
        sc->frequency += 0.15f;
        if (sc->frequency > FREQ_MAX) sc->frequency = FREQ_MAX;
        break;
    case KEY_DOWN: case 's': case 'S':
        sc->frequency -= 0.15f;
        if (sc->frequency < FREQ_MIN) sc->frequency = FREQ_MIN;
        break;

    case KEY_RIGHT: case 'd': case 'D':
        sc->amplitude += 0.10f;
        if (sc->amplitude > AMP_MAX) sc->amplitude = AMP_MAX;
        break;
    case KEY_LEFT:  case 'a': case 'A':
        sc->amplitude -= 0.10f;
        if (sc->amplitude < AMP_MIN) sc->amplitude = AMP_MIN;
        break;

    case ']':
        app->time_scale *= TIME_SCALE_STEP;
        if (app->time_scale > TIME_SCALE_MAX) app->time_scale = TIME_SCALE_MAX;
        break;
    case '[':
        app->time_scale /= TIME_SCALE_STEP;
        if (app->time_scale < TIME_SCALE_MIN) app->time_scale = TIME_SCALE_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * main — variable-timestep render loop. Per-frame phases:
 *   ① INPUT      drain getch() — a press takes effect on this frame
 *   ② RESIZE     handle pending SIGWINCH before touching ncurses
 *   ③ MEASURE dt wall-clock ns since last frame; capped at 100 ms
 *                (see EDGE CASES "Suspend / lid-close")
 *   ④ TICK       one simulation step at exactly dt · time_scale
 *   ⑤ FPS        smoothed over a 500 ms window
 *   ⑥ RENDER     erase → draw → wnoutrefresh → doupdate
 *   ⑦ FRAME CAP  sleep so total frame ≈ 1/TARGET_FPS
 */
int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app        = &g_app;
    app->running    = 1;
    app->time_scale = TIME_SCALE_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    const int64_t target_ns = NS_PER_SEC / TARGET_FPS;

    int64_t last_time   = clock_ns();
    int64_t fps_accum   = 0;
    int     fps_frames  = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① drain input */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
        if (!app->running) break;

        /* ② resize */
        if (app->need_resize) {
            app_do_resize(app);
            last_time = clock_ns();
        }

        /* ③ measure dt */
        int64_t dt_ns = frame_start - last_time;
        last_time     = frame_start;
        if (dt_ns > 100 * NS_PER_MS) dt_ns = 100 * NS_PER_MS;
        float dt = (float)dt_ns / (float)NS_PER_SEC;

        /* ④ tick */
        scene_tick(&app->scene, dt * app->time_scale);

        /* ⑤ fps counter */
        fps_frames++;
        fps_accum += dt_ns;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)fps_frames
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_frames = 0;
            fps_accum  = 0;
        }

        /* ⑥ render */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->time_scale);
        screen_present();

        /* ⑦ frame cap — sleep so total frame ≈ target_ns. The math is
         *    just (target − elapsed); no spurious +dt terms. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
