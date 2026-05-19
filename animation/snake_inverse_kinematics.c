/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snake_inverse_kinematics.c — IK snake chasing a target that bounces inside
 *
 * DEMO: A 32-segment snake chases a wandering target that carves organic
 *       terrain-like paths driven by three incommensurable sine harmonics.
 *       When the target nears any screen edge it reflects inward; the head
 *       follows it back, so the snake always stays on-screen.
 *
 * Study alongside: snake_forward_kinematics.c (same body, no target — wave only)
 *
 * Section map:
 *   §1 config   — tunables: speeds, wander harmonics, edge bounce, geometry
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 10 themes × 7 body pairs + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect-ratio bridge
 *   §5 entity   — Snake: trail buffer, IK head, bead renderer
 *       §5a trail helpers      — push, index, arc-length sampler
 *       §5b move_head          — wander + bounce + IK seek + clamp + record
 *       §5c compute_joints     — body placement from trail
 *       §5d render helpers     — pair, attribute, glyphs
 *       §5e mark_cell          — central glyph stamp helper
 *       §5f draw_segment_beads — bead fill for one segment
 *       §5g render_chain       — full frame composition
 *   §6 scene    — scene_init / scene_tick / scene_draw
 *   §7 screen   — ncurses double-buffer display layer
 *   §8 app      — signals, resize, main game loop
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause / resume
 *   ↑ / w         move speed faster
 *   ↓ / s         move speed slower
 *   + / =         target wander speed faster
 *   -             target wander speed slower
 *   t / T         next / previous theme
 *   r / R         reset simulation (theme preserved)
 *   ] / [         sim Hz + / -
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       snake_inverse_kinematics.c -o snake_ik -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : IK goal-seeking head + path-following FK body.  The
 *                  head steers each tick toward a smoothly-tracked wander
 *                  target (atan2 → cos/sin step).  The wander target
 *                  drives itself like a mini-snake: a sum of three
 *                  incommensurable sine harmonics is integrated into a
 *                  heading angle, which then drives a constant-speed step.
 *                  When the target nears any screen edge, its heading is
 *                  reflected through the edge normal — a billiard-ball
 *                  bounce that keeps it on-screen and naturally redirects
 *                  the chasing head.  The body follows by sampling the
 *                  head's trail at fixed arc-length offsets — same FK as
 *                  the sibling FK snake.
 *
 * Data-structure : trail[TRAIL_CAP] is a Vec2 ring buffer of head
 *                  positions (newest at trail[trail_head]).  joint[] +
 *                  prev_joint[] are the current chain and its sub-tick
 *                  alpha-lerp anchor.  tgt_pos / tgt_dir hold the
 *                  wander target's pose; actual_target is its low-pass-
 *                  filtered version (lerps toward tgt_pos at 8×/s).
 *                  tgt_trail[] is a short ring of recent actual_target
 *                  positions for the dim ghost trail rendering.
 *
 * Rendering      : Two-pass bead style.  Pass 1: 'o' fill along every
 *                  segment.  Pass 2: graded node markers ('0' head third,
 *                  'o' middle, '.' tail third) over-stamp at every joint.
 *                  The wander-target ghost trail and the bright '+' cursor
 *                  render BEFORE the snake so the body draws on top.
 *                  Alpha interpolation between prev_joint and joint keeps
 *                  motion smooth at any sim/render rate.
 *
 * Performance    : Fixed-step accumulator decouples physics Hz from render
 *                  Hz.  The trail walk is O(distance/speed) per joint;
 *                  with 32 joints at 60 Hz this is well under 1 µs/frame.
 *                  ncurses doupdate transmits only changed cells.
 *
 * References
 * ──────────
 *   ── Inverse kinematics (the §5b head solver) ─────────────────────
 *   [1] Craig, J. J. (2005), "Introduction to Robotics: Mechanics
 *       and Control" (3rd ed.), Pearson — Ch. 4 derives analytical
 *       IK; the 1-link case used by analytic_one_link_ik_step() is
 *       the trivial subset (atan2 + step).
 *   [2] Spong, M. W., Hutchinson, S. & Vidyasagar, M. (2005), "Robot
 *       Modeling and Control", Wiley — alternative canonical text;
 *       Ch. 4 covers the 1-/2-link solvability conditions cleanly.
 *
 *   ── Iterative IK alternatives (the contrast) ─────────────────────
 *   [3] Aristidou, A. & Lasenby, J. (2011), "FABRIK: a fast,
 *       iterative solver for the inverse kinematics problem",
 *       Graphical Models 73(5) — iterative IK for long chains.
 *       This file does NOT use FABRIK; the body follows the head's
 *       PATH (FK trail-sampling) while the HEAD does 1-link IK.
 *       Two-stage hybrid: pure analytic at the head, FK for the body.
 *   [4] Buss, S. R. (2004), "Introduction to Inverse Kinematics with
 *       Jacobian Transpose, Pseudoinverse and Damped Least Squares",
 *       UCSD course notes — the Jacobian-iterative family we
 *       DON'T need here.
 *
 *   ── Steering / autonomous wander (the target's motion) ──────────
 *   [5] Reynolds, C. (1999), "Steering Behaviors for Autonomous
 *       Characters", GDC — wander + arrival + containment patterns;
 *       multi_harmonic_steering_turn() implements §"Wander" with
 *       three sinusoids instead of one.
 *       https://www.red3d.com/cwr/steer/
 *   [6] Reynolds, C. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 — earlier work
 *       establishing the autonomous-character motion pattern this
 *       file's target follows.
 *
 *   ── Central pattern generators (biology) ────────────────────────
 *   [7] Manton, S. M. (1952), "The evolution of arthropodan
 *       locomotory mechanisms", J. Linn. Soc. Zoology — biological
 *       reference for CPGs: gait timing from an internal oscillator,
 *       not sensory feedback. The wander target's CPG is a
 *       computer-science analogue.
 *
 *   ── Path / arc-length parameterisation (the §5a trail walk) ─────
 *   [8] Foley, J. D., van Dam, A. et al. (1995), "Computer Graphics:
 *       Principles and Practice" (2nd ed.) — §11.2 spline / curve
 *       arc-length parameterisation; trail_sample applies the same
 *       walk-and-interpolate technique to a polyline.
 *   [9] Khoshrou, S. (2009), "Snake Robot Locomotion: Theory and
 *       Simulation" — analysis of serpentine FK on a moving curve.
 *
 *   ── Signal processing (the §5b IIR low-pass) ────────────────────
 *  [10] Oppenheim, A. V. & Schafer, R. W. (2010), "Discrete-Time
 *       Signal Processing" (3rd ed.), Pearson — §3.6 first-order
 *       IIR (leaky integrator); first_order_low_pass() is the
 *       discrete-time form of that filter.
 *
 *   ── Classical mechanics (the wall reflections) ─────────────────
 *  [11] Goldstein, H. (1980), "Classical Mechanics" (2nd ed.),
 *       Addison-Wesley — §3.6 elastic collisions with axis-aligned
 *       walls; basis for reflect_direction_about_{vertical,
 *       horizontal}_wall() in bounce_target().
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *  [12] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com —
 *       the fixed-step accumulator pattern this file uses; alpha
 *       interpolation between prev_joint and joint comes from here.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *  [13] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       the integer line algorithm; §5d draw_segment_beads uses the
 *       simpler parametric oversample because float math is already
 *       paid for in trail_sample.
 *  [14] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — basis for
 *       the head-to-tail brightness gradient.
 *  [15] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the diff pipeline §7 relies on.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [16] https://en.wikipedia.org/wiki/Inverse_kinematics
 *  [17] https://en.wikipedia.org/wiki/Forward_kinematics
 *  [18] https://en.wikipedia.org/wiki/Infinite_impulse_response
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two animals coexist on the screen: a wander target that flies around
 * along smooth pseudo-random curves, and a snake whose head chases the
 * target while its 32-segment body follows by arc-length sampling of
 * the head's recorded trail.  When the target hits a wall it bounces
 * (heading reflects through the edge normal); the head follows it
 * back, so the snake stays on-screen forever without wrapping.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a fly buzzing inside a glass tank, bouncing off the walls,
 * and a 32-bead string trying to keep up.  Each tick the lead bead
 * (the head) takes one short hop toward the fly's current spot; every
 * other bead places itself at a fixed bead-spacing back along the
 * actual ink line that the head has been drawing.  The fly's flight
 * path is the seed of every shape the body takes; the rest is just
 * "where was the head at distance i·SEG_LEN_PX in the past?"
 *
 * DRAWING METHOD / ALGORITHM IN STEPS
 * ───────────────────────────────────
 *   1. Save prev_joint = joint                    (anchor for alpha lerp)
 *   2. Advance tgt_time by dt
 *   3. turn  = A1·sin(f1·t) + A2·sin(f2·t+φ2) + A3·sin(f3·t+φ3)
 *      tgt_dir += turn · dt                       (curvature → heading)
 *      tgt_pos += tgt_speed · (cos, sin)(tgt_dir) · dt
 *   4. If tgt_pos crossed an edge: reflect tgt_dir through the edge
 *      normal AND clamp tgt_pos to inside the bounded region.
 *   5. Lerp actual_target toward tgt_pos at rate min(dt · 8, 1).
 *   6. heading = atan2(actual_target − head)      (analytic 1-link IK)
 *      head   += min(move_speed · dt, dist) · (cos, sin)(heading)
 *   7. Hard-clamp head into the screen box (safety net).
 *   8. trail_push(head); tgt_push(actual_target)
 *   9. For i in 1..N_SEGS: joint[i] = trail_sample(i · SEG_LEN_PX)
 *  10. Render: lerp prev_joint → joint by alpha; ghost trail + cursor +
 *      bead body + node markers + head arrow (painter's order).
 *
 * KEY FORMULAS
 * ────────────
 *   Wander target heading update:
 *     turn = Σ Aᵢ · sin(fᵢ · t + φᵢ)              [rad/s]
 *     tgt_dir += turn · dt
 *
 *   Edge reflection (target only — billiard bounce):
 *     left/right wall: tgt_dir → π − tgt_dir       (flip cos)
 *     top/bottom wall: tgt_dir → −tgt_dir          (flip sin)
 *     followed by hard clamp tgt_pos to inside.
 *
 *   IK head step (analytic 1-link):
 *     v       = actual_target − head
 *     dist    = |v|
 *     heading = atan2(v.y, v.x)
 *     step    = min(move_speed · dt, dist)         (no overshoot)
 *     head   += (v / dist) · step
 *
 *   Trail arc-length sampling (place joint i):
 *     walk trail newest→oldest, accumulating |trail[k+1] − trail[k]|;
 *     when accum ≥ i · SEG_LEN_PX, lerp into the segment that crossed it.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • If dist < 0.5 px, skip the head step.  Otherwise (v / dist) blows up
 *     and the head jitters when it sits on top of the target.
 *   • Reflect-and-clamp must be done together.  Reflecting alone can leave
 *     tgt_pos still outside (e.g. pushed several pixels past the wall by
 *     a single tick at high speed); without the clamp, the next reflection
 *     fires again, oscillating in place.
 *   • actual_target follows tgt_pos with a one-tick lag, so right after a
 *     bounce the head still aims at the OLD smoothed target for ~one tick.
 *     That is intentional: it makes the corner turn read as "thoughtful"
 *     rather than instant.
 *   • Two coincident trail samples → divide-by-zero in trail_sample's lerp.
 *     Guard with `seg = max(seg, 1e-4f)`.
 *   • Frame cap: never `elapsed = clock_ns() − frame_time + dt` — the +dt
 *     cancels the cap, sleep is always 0, CPU pegs at 100 %.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Crank tgt_speed (+ key) until the target is sprinting; the head
 *     should always end up inside the screen box no matter how often the
 *     target hits walls.
 *   • Crank move_speed (UP) to MAX; even if the head briefly overshoots
 *     during a sharp target turn, the hard clamp should keep it inside.
 *   • Pause (space): both target and snake freeze; no perceptible glitch
 *     on resume.
 *   • Cycle themes (t/T): every theme entry is legible against a default
 *     terminal background under A_DIM (no segment "disappears" at the
 *     tail-quarter).  Confirms cube ≥ 24 brightness rule.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read snake_forward_kinematics.c first — same body
 *      mechanics, but with FK autopilot instead of IK seek.
 *   2. §5 entity — THE HEART of this file. In sub-section order:
 *        §5a trail helpers       ← circular buffer (T1 in FK partner)
 *        §5b move_head           ← READ THIS, T1-T5 below
 *           ‣ wander target update
 *           ‣ edge reflection (billiard bounce)
 *           ‣ low-pass filter on actual_target
 *           ‣ 1-link IK step toward actual_target
 *        §5c compute_joints      ← arc-length body placement
 *        §5d-§5g rendering
 *   3. §6 scene — orchestrator with prev/cur snapshot.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   trail[]               head's recent positions (circular).
 *   tgt_pos               wander target's current position.
 *   tgt_dir               wander target's heading (rad).
 *   actual_target         smoothed version of tgt_pos that the
 *                         head actually chases.
 *   tgt_trail[]           ghost trail of actual_target positions.
 *   joint[i]              snake body joint i (Vec2). 0 = head,
 *                         N_SEGS = tail.
 *   prev_joint[i]         snapshot for sub-tick alpha lerp.
 *   move_speed            head's speed in pixels/sec (key-tuned).
 *   tgt_speed             wander target's speed (key-tuned).
 *
 * Background you need
 * ───────────────────
 *   - Path-following FK (snake_forward_kinematics T1-T2). The body
 *     mechanics are IDENTICAL.
 *   - 1-link IK reduces to atan2 — that's the only "IK" math here.
 *   - Reynolds-style WANDER (random heading turn rate, integrated).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - FABRIK or Jacobian. The "IK" in this file is exactly one
 *     joint (the head); analytical solution is one atan2.
 *   - PID controllers. The IK step is just "step toward target,
 *     don't overshoot."
 *   - Joint limits. The body is FK-driven from the trail, no
 *     joint angles to constrain.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a goal-seeking snake from first
 * principles.
 *
 *   T1  IK on a 1-link chain — atan2 is enough
 *   T2  Why a separate WANDER TARGET (and not just a fixed goal)
 *   T3  Billiard-ball bounce — reflect heading through edge normal
 *   T4  Low-pass filter on the chase target — same trick as tentacle
 *   T5  No-overshoot guarantee — clamp the step to remaining distance
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  IK ON A 1-LINK CHAIN — atan2 IS ENOUGH
 * ──────────────────────────────────────────
 * The snake's BODY is FK (snake_forward_kinematics T1-T2). The
 * snake's HEAD is the only IK joint — and a 1-link IK has a
 * closed-form trivial solution:
 *
 *     v       = target − head
 *     heading = atan2(v.y, v.x)
 *     head   += (v / |v|) · step_size
 *
 * "Point at target, walk toward target." That's it. No FABRIK,
 * no law of cosines — they're not needed because there's only
 * one joint.
 *
 * Once the head moves, the trail records its new position, and
 * the body follows by arc-length sampling (T2 in the FK partner
 * file). So the file is structurally:
 *
 *     1-link IK on the head + 32-link FK on the body
 *
 * Most "IK" demos in this folder follow that pattern: a small
 * IK problem at the front, FK propagating the rest.
 *
 * T2  WHY A SEPARATE WANDER TARGET (AND NOT JUST A FIXED GOAL)
 * ────────────────────────────────────────────────────────────
 * If the user moved the target manually with arrow keys, this
 * file would just be "1-link IK chasing user input." Boring as
 * a demo because the head response is dominated by user
 * intention, not by emergent behaviour.
 *
 * Instead, the target itself is autonomous — a tiny
 * "self-wandering" agent. Its motion comes from a sum of three
 * incommensurable sine waves driving its turn rate:
 *
 *     turn = A₁ sin(f₁ t) + A₂ sin(f₂ t + φ₂) + A₃ sin(f₃ t + φ₃)
 *     tgt_dir += turn · dt
 *     tgt_pos += tgt_speed · (cos tgt_dir, sin tgt_dir) · dt
 *
 * Three frequencies with NO rational ratio means the resulting
 * curve never exactly repeats — quasi-periodic motion. The
 * wander target traces organic-looking curves that resemble
 * terrain ridges or rivers without being any specific shape.
 *
 * Reynolds (1999) called this WANDER. He used random noise as
 * the turn-rate driver; we use the deterministic sine sum
 * because it's REPRODUCIBLE — running the demo twice gives the
 * same path, which makes debugging easier and the visual
 * predictable enough to compare across themes.
 *
 * T3  BILLIARD-BALL BOUNCE — REFLECT HEADING THROUGH EDGE NORMAL
 * ──────────────────────────────────────────────────────────────
 * The wander target needs to STAY ON SCREEN. The autopilot
 * alone won't do it — the heading random walk will eventually
 * drift past any boundary.
 *
 * Snake-FK uses a soft fence (edge bias). For the wander target
 * we use a HARD bounce — billiard physics:
 *
 *     if tgt_pos.x < 0 or > screen_width:
 *       tgt_dir = π − tgt_dir            ← flip horizontal component
 *       clamp tgt_pos.x inside
 *     if tgt_pos.y < 0 or > screen_height:
 *       tgt_dir = −tgt_dir               ← flip vertical component
 *       clamp tgt_pos.y inside
 *
 * Why hard bounce instead of soft fence?
 *
 *   - Visually distinct from the snake. The target snaps off
 *     walls; the snake curves smoothly because it's chasing
 *     the (now smoothed) target, not the wall directly.
 *   - Cheaper: just heading flip + position clamp.
 *   - Quasi-periodic motion + reflective walls = chaotic
 *     billiard-like trajectory. Looks lively without any
 *     randomness.
 *
 * The CLAMP is essential alongside the flip: a single fast
 * tick can push tgt_pos several pixels past the wall, so the
 * flip alone doesn't restore in-bounds. Clamp to the wall
 * AFTER flipping the direction.
 *
 * T4  LOW-PASS FILTER ON THE CHASE TARGET — SAME TRICK AS TENTACLE
 * ────────────────────────────────────────────────────────────────
 * The target's motion is autonomous and includes velocity
 * reversals at every wall bounce. If the head chased tgt_pos
 * directly, every bounce would be a discontinuous heading
 * change — the snake would VISUALLY SNAP at the moment of
 * bounce.
 *
 * Same fix as ik_tentacle_seek T5: low-pass filter the
 * chase target.
 *
 *     rate = clamp(dt · 8, 0, 1)
 *     actual_target += (tgt_pos − actual_target) · rate
 *     head chases ACTUAL_TARGET, not tgt_pos
 *
 * Time constant ≈ 125 ms. Fast enough to track the wander
 * comfortably; slow enough that bounces appear as a
 * THOUGHTFUL TURN rather than a snap.
 *
 * The ghost-trail dots are positions of actual_target (not
 * tgt_pos), so the user sees what the snake is actually
 * chasing — useful for debugging the smoothing tuning.
 *
 * T5  NO-OVERSHOOT GUARANTEE — CLAMP THE STEP TO REMAINING DISTANCE
 * ─────────────────────────────────────────────────────────────────
 * A naïve IK step always moves move_speed · dt. If the head is
 * already very close to the target (less than that step), it
 * OVERSHOOTS, lands on the far side, then overshoots back, then
 * forward — visible jitter when the snake catches up.
 *
 * Standard arrival behaviour: clamp the step to the remaining
 * distance.
 *
 *     dist = |actual_target − head|
 *     step = min(move_speed · dt, dist)
 *     head += (v / dist) · step
 *
 * Now the head can NEVER pass through the target — it lands on
 * top of it and waits for the target to drift away. The snake
 * settles smoothly when paused or when the target loiters.
 *
 * Edge case: dist == 0 makes (v / dist) divide by zero. Guard:
 *
 *     if dist < 0.5 px: skip the step
 *
 * Without the guard, even after the snake catches the target,
 * NaN would propagate through the head's heading. Half a pixel
 * is well below visibility, so freezing in place there is
 * indistinguishable from following exactly.
 *
 * Same arrival pattern is used in ik_arm_reach (CONV_TOL early-
 * out in the FABRIK loop). Different solver, same idea: STOP
 * when you're already there.
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
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    N_PAIRS          =   7,   /* gradient color pairs for snake body        */
    PAIR_HUD         =   8,   /* bright yellow — top status bar             */
    PAIR_HINT        =   9,   /* bright cyan   — bottom key hint            */
    PAIR_GHOST       =  10,   /* dim grey      — wander-target ghost trail  */
    N_THEMES         =  10,

    N_SEGS           =  32,   /* rigid body segments                        */
    TRAIL_CAP        = 4096,  /* circular head-position history capacity    */

    /*
     * TARGET_TRAIL_CAP — ghost trail length for the wandering target.
     * The last 200 actual_target positions are drawn as dim dots, showing
     * enough of the winding path that the terrain-like character is visible.
     * At tgt_speed=80 px/s and 60 Hz: 200 ticks × 80/60 ≈ 267 px of trail
     * = ~33 columns — enough to see several hills and valleys.
     */
    TARGET_TRAIL_CAP = 200,
};

/* Segment and bead step dimensions */
#define SEG_LEN_PX     18.0f   /* pixel length of each rigid body segment   */
#define DRAW_STEP_PX    5.0f   /* bead fill step (larger = sparser beads)   */

/* Head translation speed (px/s) */
#define MOVE_SPEED_DEFAULT  150.0f
#define MOVE_SPEED_MIN       20.0f
#define MOVE_SPEED_MAX      600.0f

/*
 * Wandering target parameters.
 *
 * The target steers itself via a superposition of three sine waves applied
 * to its heading (tgt_dir).  This models the kind of curvature a river or
 * mountain path makes — mostly smooth, with irregular undulations.
 *
 * TGT_WANDER_SPEED — target translation speed in px/s.
 *   At ~80 px/s the target crosses a 640 px screen in 8 s.  Slow enough
 *   for the snake to chase; fast enough to stay ahead of it.
 *   +/- keys adjust this at runtime in ×/÷ 1.25 steps.
 *
 * TGT_TURN_AMPn / TGT_TURN_FREQn — amplitude and angular frequency for
 *   each of three sine harmonics.
 *   Frequencies are mutually irrational (0.29, 0.71, 1.13) so the combined
 *   turn-rate waveform never exactly repeats.
 *   Amp1 (large, slow) → wide sweeping hills.
 *   Amp2 (medium)      → mid-scale wiggles overlaid on the hills.
 *   Amp3 (small, fast) → fine tremors for organic texture.
 *
 * TGT_TURN_PHASEn — initial phase offsets so the three waves start
 *   out of sync and the path is interesting from frame one.
 *
 * TGT_SMOOTH_RATE — lerp coefficient for actual_target → tgt_pos.
 *   Filters out wrap discontinuities; 8×/s is fast enough to track the
 *   target closely while still softening sharp turns.
 */
#define TGT_WANDER_SPEED_DEFAULT  80.0f
#define TGT_WANDER_SPEED_MIN       5.0f
#define TGT_WANDER_SPEED_MAX     500.0f

#define TGT_TURN_AMP1   1.40f   /* wide sweeping curves (rad/s)            */
#define TGT_TURN_FREQ1  0.29f   /* ~21 s period                            */
#define TGT_TURN_AMP2   0.80f   /* medium wiggles                          */
#define TGT_TURN_FREQ2  0.71f   /* ~8.9 s period                           */
#define TGT_TURN_AMP3   0.40f   /* fine tremors                            */
#define TGT_TURN_FREQ3  1.13f   /* ~5.6 s period                           */
#define TGT_TURN_PHASE2 1.10f   /* phase offset for harmonic 2 (radians)   */
#define TGT_TURN_PHASE3 2.40f   /* phase offset for harmonic 3             */

#define TGT_SMOOTH_RATE  8.00f  /* actual_target lerp rate toward tgt_pos  */

/*
 * EDGE_MARGIN_PX — gap between the wander target's bounce wall and the
 * physical screen edge, in pixels.
 *
 * The target reflects off an inner wall positioned EDGE_MARGIN_PX from
 * each screen edge.  Setting margin > 0 (rather than bouncing off the
 * screen edge itself) keeps the target — and the snake's head chasing it —
 * fully visible at the corner, instead of drawing partly behind the HUD
 * bars or against the very edge.  64 px ≈ 8 columns / 4 rows of breathing
 * room on every side.
 */
#define EDGE_MARGIN_PX   64.0f

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Terminal cell dimensions (physics↔display bridge, see §4) */
#define CELL_W   8
#define CELL_H  16

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
/* §3  color / themes                                                     */
/* ===================================================================== */

/*
 * Theme — one named body palette.
 *
 *   name   — displayed in the HUD status bar.
 *   body[] — 7 xterm-256 foreground indices for pairs 1..7 (head→tail).
 *
 * Brightness rule: every body[] entry is in the bright half of the
 * 256-colour cube (≥ 24).  Indices 16–23 (cube near-blacks) and 232–239
 * (gray near-blacks) become invisible under A_DIM at the tail-quarter,
 * so they are avoided.
 *
 * PAIR_HUD / PAIR_HINT / PAIR_GHOST are theme-independent and configured
 * once in color_init().
 */
/*
 * Theme — one named multi-colour body palette (xterm-256 fg indices).
 *
 * Intent
 *   Each theme rebinds the seven body pairs (1..N_PAIRS) via init_pair
 *   in one shot (see theme_apply). HUD/HINT/GHOST pairs are NOT
 *   theme-bound — they stay bright yellow / bright cyan / faint
 *   target-ghost across every theme so the status bar + target trail
 *   remain legible against any palette (CLAUDE.md HUD spec).
 *
 * Slot semantics (body[0..6] → pairs 1..7; head → tail gradient)
 *   [0] head      — joint[0] + first body bead (brightest)
 *   [1..5] body   — mid-body region (smooth gradient)
 *   [6] tail tip  — joint[N_SEGS] (dimmest, A_DIM-attenuated)
 *
 *   The head → tail brightness gradient lets the eye trace which way
 *   the snake is swimming. Ref [14] Bourke.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every entry sits in the BRIGHT HALF of the 256-colour cube:
 *     - cube colours: ≥ 24  (avoid 16-23; invisible under A_DIM)
 *     - grayscale  : ≥ 240 (avoid 232-239; same reason)
 *   The tail quarter renders with A_DIM, so without the brightness
 *   rule the tail would simply disappear on dark terminals.
 *
 * References [14] Bourke for the gradient-as-depth pattern;
 *   [15] Raymond §init_pair for the rebind mechanism.
 */
typedef struct {
    const char *name;            /* HUD-displayable theme name        */
    int         body[N_PAIRS];   /* xterm-256 fg per pair slot;       *
                                  * pair p = body[p-1] at apply time  */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      head ←─────────────────────→ tail */
    {"Medusa", { 57,  63,  93,  99, 105, 111, 159}},
    {"Matrix", { 28,  34,  40,  76,  46,  82, 118}},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}},
    {"Ocean",  { 24,  25,  31,  33,  39,  45,  51}},
    {"Nova",   { 54,  55,  56,  57,  93, 129, 165}},
    {"Toxic",  { 28,  58,  64,  70,  76,  82, 118}},
    {"Lava",   { 52,  88, 124, 160, 196, 202, 208}},
    {"Ghost",  {244, 245, 247, 249, 251, 253, 255}},
    {"Aurora", { 28,  34,  64,  71,  78, 121, 159}},
    {"Neon",   {201, 165, 129,  93,  57,  51,  45}},
};

/*
 * theme_apply() — register the body palette with ncurses.
 *
 * Background = -1 (terminal default) so demos respect the user's theme.
 * 8-colour fallback approximates the warm-to-cool feel.
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, th->body[p], -1);
    } else {
        static const int fb8[N_PAIRS] = {
            COLOR_YELLOW, COLOR_YELLOW, COLOR_GREEN,
            COLOR_GREEN,  COLOR_CYAN,   COLOR_CYAN, COLOR_BLUE
        };
        for (int p = 0; p < N_PAIRS; p++)
            init_pair(p + 1, fb8[p], -1);
    }
}

/*
 * color_init() — one-time colour system setup.
 *
 *   start_color()        — initialise ncurses colour support.
 *   use_default_colors() — allow background = -1 to mean "terminal default".
 *   theme_apply()        — register the initial body palette.
 *   PAIR_HUD/HINT/GHOST  — bright yellow / cyan / dim grey, theme-independent.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_GHOST, 244, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_GHOST, COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the aspect-ratio bridge                      */
/* ===================================================================== */

/*
 * All physics positions are in square pixel space (1 px = 1 physical pixel
 * in both axes).  Drawing converts to cell coordinates so the snake looks
 * isotropic regardless of terminal cell aspect ratio (CELL_H = 2×CELL_W).
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
/* §5  entity — Snake: trail buffer + IK head + bead renderer            */
/* ===================================================================== */

/*
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   Every §5 simulation quantity — wander target position, head
 *   position, trail samples, body joints — lives in pixel space.
 *   Each character cell is CELL_W × CELL_H sub-pixels (8 × 16),
 *   giving sub-cell precision so a chasing snake reads as continuous
 *   motion instead of jumping cell-to-cell. The conversion to cell
 *   space happens only inside §5d rendering helpers via
 *   px_to_cell_x/y — the project's "one conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → DOWN the screen).
 *
 *   Angles are MATH-CONVENTION (positive CCW from +X). Because y
 *   points DOWN on screen, a math-CCW rotation displays as CW.
 *   reflect_direction_about_horizontal_wall (θ' = −θ) handles the
 *   y-axis-flipped reflection correctly in this convention because
 *   sin(−θ) = −sin θ regardless of whether y points up or down.
 *
 * Why a value type
 *   8 bytes; passes in registers on every modern ABI. The §5b helpers
 *   (polyline_segment_length, lerp_between_points, first_order_low_pass)
 *   all take/return Vec2 by value — -O2 inlines them into straight-
 *   line code with no allocation.
 *
 * Why named (x, y) instead of two loose floats
 *   Type-checking catches (col, row) confusion at compile time
 *   rather than via visual debugging.
 *
 * References [1] Craig §2 "Spatial descriptions".
 */
typedef struct {
    float x;   /* eastward  pixel coordinate (positive → right) */
    float y;   /* southward pixel coordinate (positive → down)  */
} Vec2;

/*
 * Snake — the full state of the IK demo.
 *
 * Two animals coexist on screen, both stored in this one record:
 *
 *   1. WANDER TARGET — an autonomous mover driven by a three-harmonic
 *      CPG. State: (tgt_time, tgt_pos, tgt_dir, tgt_speed). Bounces
 *      off the bounded region. Refs [5][6] Reynolds, [7] Manton.
 *
 *   2. SNAKE — chases the smoothed target via 1-link analytical IK
 *      at the head; the body follows the head's path via FK trail-
 *      sampling. The architecture is a "two-stage hybrid": analytic
 *      IK at the head, FK arc-length sampling for the body.
 *      Refs [1][9] Craig + Khoshrou.
 *
 *   The smoothing layer (actual_target, first_order_low_pass) sits
 *   BETWEEN them: it reads tgt_pos (the wander target's raw output)
 *   and produces actual_target (what the head actually chases). The
 *   IIR low-pass rounds bounce-corners so the head's pursuit looks
 *   deliberate, not jittery. Ref [10] Oppenheim & Schafer.
 *
 * Data-flow arrow (one direction, no feedback loop):
 *
 *     wander CPG          → tgt_pos  (raw target, bounces off walls)
 *     first_order_low_pass → actual_target  (IIR-smoothed target)
 *     analytic_one_link_ik → head position + heading
 *     trail_push           → trail buffer
 *     compute_joints       → body joints (arc-length sample of trail)
 *
 * Why trail is a CIRCULAR buffer (not a regular array)
 *   Trail samples are PUSHED at the head every tick and CONSUMED at
 *   every arc-length along the body. Circular indexing means no
 *   O(N) shift per frame; trail_head walks mod TRAIL_CAP. Once
 *   filled, oldest entries are silently overwritten.
 *
 * Why prev_joint AND joint (two body-position arrays)
 *   - joint     : the CURRENT body pose; rewritten each tick.
 *   - prev_joint: pose at the START of the current tick; anchor for
 *                  sub-tick render interpolation.
 *   render_chain lerps prev_joint → joint by alpha ∈ [0, 1) for
 *   smooth motion at any sim/render Hz mismatch. Ref [12] Fiedler.
 *
 * Why tgt_pos AND actual_target (two target positions)
 *   - tgt_pos      : the RAW wander-CPG output. Jumps sharply when
 *                     bounce_target reflects off a wall.
 *   - actual_target: IIR-smoothed version that the IK chases.
 *   Without the smoothing layer, the head would snap-pursue every
 *   bounce corner — visually unpleasant. Smoothing converts each
 *   sharp bounce into a smooth quarter-circle curve around the
 *   reflected corner.
 *
 * Why tgt_trail (separate ghost-trail buffer)
 *   Renders as a faint dotted breadcrumb path behind the target
 *   so the viewer can see where the head is HEADING. Shorter cap
 *   than the body trail (TARGET_TRAIL_CAP < TRAIL_CAP) because we
 *   only need a few seconds of memory for the visual hint.
 *
 * References [1] Craig §4 (1-link IK); [5] Reynolds wander;
 *   [7] Manton CPG; [8] Foley & van Dam arc-length sampling;
 *   [10] Oppenheim & Schafer IIR low-pass; [12] Fiedler.
 */
typedef struct {
    /* ── Trail buffer (FK source-of-truth for body) ──────────── *
     * Circular log of past head positions; compute_joints arc-  *
     * length-samples this to place body joints.                 */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;                 /* index of newest entry, mod TRAIL_CAP*/
    int  trail_count;                /* valid entries, saturates at TRAIL_CAP*/

    /* ── Body joints (FK output) ─────────────────────────────── *
     * joint[0] = head (set by move_head's IK step).             *
     * joint[1..N_SEGS] = body+tail (set by compute_joints).     *
     * prev_joint = snapshot at tick start (sub-tick lerp anchor).*/
    Vec2 joint     [N_SEGS + 1];
    Vec2 prev_joint[N_SEGS + 1];

    /* ── Head steering (output of 1-link IK) ─────────────────── *
     * heading is the CURRENT direction of travel; written by    *
     * analytic_one_link_ik_step from atan2(target − head).      *
     * move_speed is the constant head pursuit speed (px/s).     */
    float heading;
    float move_speed;

    /* ── Wander target (input to the IK system) ──────────────── *
     * tgt_pos/dir/speed are the WANDER CPG's state. tgt_time is *
     * the CPG clock; tgt_pos jumps sharply at wall bounces.     */
    Vec2  tgt_pos;
    float tgt_time;                  /* CPG clock (s)                       */
    float tgt_speed;                 /* wander target speed (px/s)          */
    float tgt_dir;                   /* wander target heading (rad)         */

    /* ── Smoothing layer (between raw target and head) ──────── *
     * IIR low-pass output that the head's IK actually chases.   *
     * Decouples the sharp wander bounces from the head's chase. */
    Vec2  actual_target;

    /* ── Target ghost trail (renderer-visible breadcrumbs) ──── *
     * Short circular buffer of recent actual_target positions; *
     * rendered as a faint dotted hint of where the head will go.*/
    Vec2 tgt_trail[TARGET_TRAIL_CAP];
    int  tgt_head;                   /* index of newest entry             */
    int  tgt_count;                  /* valid entries, saturates at CAP   */

    /* ── UI / control state ─────────────────────────────────── *
     * paused gates scene_tick (renderer still runs);            *
     * theme_idx selects the §3 palette.                         */
    int  theme_idx;                  /* index into THEMES[]; t/T cycles    */
    bool paused;
} Snake;

/* ── §5a  trail helpers ─────────────────────────────────────────────── */

static void trail_push(Snake *s, Vec2 pos)
{
    s->trail_head = (s->trail_head + 1) % TRAIL_CAP;
    s->trail[s->trail_head] = pos;
    if (s->trail_count < TRAIL_CAP) s->trail_count++;
}

static inline Vec2 trail_at(const Snake *s, int k)
{
    return s->trail[(s->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * polyline_segment_length — Euclidean distance between two trail points.
 *
 *     |b − a| = √((b.x − a.x)² + (b.y − a.y)²)
 *
 *   The trail is a POLYLINE (piecewise-linear curve through stored
 *   samples). Its arc length is the sum of these segment lengths.
 *   trail_sample walks the polyline summing this primitive until the
 *   target arc-length is bracketed.
 */
static inline float polyline_segment_length(Vec2 a, Vec2 b)
{
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrtf(dx * dx + dy * dy);
}

/*
 * lerp_between_points — linear interpolation a + t·(b − a).
 *
 *     t = 0  →  a            (start of segment)
 *     t = 1  →  b            (end   of segment)
 *     t ∈ (0,1)  →  intermediate point on the line segment a→b
 *
 *   Used to land a body joint at exactly the requested arc length,
 *   not at the nearest stored sample — sub-sample precision is what
 *   keeps the chain looking smooth as the head curves.
 */
static inline Vec2 lerp_between_points(Vec2 a, Vec2 b, float t)
{
    return (Vec2){
        a.x + (b.x - a.x) * t,
        a.y + (b.y - a.y) * t,
    };
}

/*
 * trail_sample — arc-length parameterisation along the trail polyline.
 *
 *   Walk newest → oldest accumulating polyline_segment_length until
 *   the running total brackets the requested arc length `dist`. Then
 *   linearly interpolate WITHIN that bracketing segment.
 *
 *   Pseudocode:
 *     accum ← 0
 *     a     ← trail[0]                       (newest = current head)
 *     for k = 1 .. trail_count − 1:
 *         b   ← trail[k]                     (one tick older)
 *         seg ← polyline_segment_length(a, b)
 *         if accum + seg ≥ dist:             (target inside [a, b])
 *             t ← (dist − accum) / seg       (fractional position)
 *             return lerp_between_points(a, b, t)
 *         accum ← accum + seg
 *         a     ← b
 *     return trail[trail_count − 1]          (trail exhausted)
 *
 *   This is the standard ARC-LENGTH SAMPLING pattern used in
 *   tessellators and motion-capture playback: parameterise the
 *   polyline by ARC LENGTH (even spacing along the curve), not by
 *   INDEX (uneven spacing that depends on how fast samples arrived).
 *
 *   Numerical guard: divide-by-zero on seg ≈ 0 — clamp the
 *   denominator to 1e-4 so t collapses to "start of segment".
 *
 *   Reference: Foley & van Dam, "Computer Graphics: Principles and
 *   Practice" §11.2 on spline / curve arc-length parameterisation.
 */
static Vec2 trail_sample(const Snake *s, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(s, 0);              /* newest = current head */

    for (int k = 1; k < s->trail_count; k++) {
        Vec2  b   = trail_at(s, k);            /* one tick older than a */
        float seg = polyline_segment_length(a, b);

        if (accum + seg >= dist) {             /* target inside [a, b] */
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return lerp_between_points(a, b, t);
        }

        accum += seg;                          /* keep walking back    */
        a      = b;
    }

    return trail_at(s, s->trail_count - 1);    /* trail exhausted      */
}

/* ── §5b  move_head — IK goal-seeking ──────────────────────────────── */

/*
 * tgt_push() — record actual_target into the ghost trail buffer.
 */
static void tgt_push(Snake *s, Vec2 pos)
{
    s->tgt_head = (s->tgt_head + 1) % TARGET_TRAIL_CAP;
    s->tgt_trail[s->tgt_head] = pos;
    if (s->tgt_count < TARGET_TRAIL_CAP) s->tgt_count++;
}

/*
 * bounce_target() — reflect tgt_dir off any wall the target has crossed,
 * and clamp tgt_pos to the bounded region.
 *
 * Bounded region: [margin, wpx − margin] × [margin, hpx − margin], where
 * margin = EDGE_MARGIN_PX.  Reflection rules (terminal +y is downward):
 *   left/right wall  → tgt_dir' = π − tgt_dir   (flip the cos component)
 *   top/bottom wall  → tgt_dir' = −tgt_dir      (flip the sin component)
 *
 * The clamp is essential — a single tick at high tgt_speed can push the
 * target several pixels past the wall.  Without the clamp the next tick
 * would still find tgt_pos outside, reflect tgt_dir again, and the
 * target would oscillate in place at the wall.
 *
 * After a bounce, actual_target lerps toward the new tgt_pos at 8×/s, so
 * the head's chase smoothly rounds the corner rather than snapping.
 */
/*
 * reflect_direction_about_vertical_wall — left/right wall reflection.
 *
 *   A wall parallel to the y-axis (i.e. vertical: x = constant) has
 *   normal along ±x. Reflecting a direction θ about that normal flips
 *   the x component of (cos θ, sin θ) while leaving y untouched:
 *
 *       (cos θ, sin θ)   →   (−cos θ, sin θ)
 *
 *   In angle form: θ' = π − θ. (Substitute and verify:
 *   cos(π − θ) = −cos θ, sin(π − θ) = sin θ. ✓)
 *
 *   Reference: any classical mechanics text on elastic collisions
 *   with axis-aligned walls; e.g. Goldstein §3.6.
 */
static inline float reflect_direction_about_vertical_wall(float dir)
{
    return (float)M_PI - dir;
}

/*
 * reflect_direction_about_horizontal_wall — top/bottom wall reflection.
 *
 *   A wall parallel to the x-axis has normal along ±y. Reflection
 *   flips the y component of (cos θ, sin θ):
 *
 *       (cos θ, sin θ)   →   (cos θ, −sin θ)
 *
 *   In angle form: θ' = −θ. (cos(−θ) = cos θ, sin(−θ) = −sin θ. ✓)
 */
static inline float reflect_direction_about_horizontal_wall(float dir)
{
    return -dir;
}

/*
 * bounce_target — reflect tgt_dir off any wall the target has crossed,
 * AND clamp tgt_pos back inside the bounded region.
 *
 *   Bounded region: [m, wpx−m] × [m, hpx−m] where m = EDGE_MARGIN_PX.
 *
 *   For each axis-aligned wall the target crossed this tick:
 *     1. Clamp the position back to the wall (so the next tick
 *        doesn't keep finding the target outside).
 *     2. Reflect the direction about the wall's normal.
 *
 *   THE CLAMP IS ESSENTIAL — at high tgt_speed a single tick can
 *   carry the target several pixels past the wall. Without the
 *   clamp, the next tick would still find tgt_pos outside, reflect
 *   AGAIN, and the target would oscillate in place at the wall.
 *
 *   After a bounce, actual_target lerps toward the new tgt_pos at
 *   8× per second (first_order_low_pass below), so the head's chase
 *   smoothly rounds the corner rather than snapping.
 */
static void bounce_target(Snake *s, float wpx, float hpx)
{
    float m    = EDGE_MARGIN_PX;
    float lo_x = m, hi_x = wpx - m;
    float lo_y = m, hi_y = hpx - m;

    if (s->tgt_pos.x < lo_x) {
        s->tgt_pos.x = lo_x;
        s->tgt_dir   = reflect_direction_about_vertical_wall(s->tgt_dir);
    } else if (s->tgt_pos.x > hi_x) {
        s->tgt_pos.x = hi_x;
        s->tgt_dir   = reflect_direction_about_vertical_wall(s->tgt_dir);
    }
    if (s->tgt_pos.y < lo_y) {
        s->tgt_pos.y = lo_y;
        s->tgt_dir   = reflect_direction_about_horizontal_wall(s->tgt_dir);
    } else if (s->tgt_pos.y > hi_y) {
        s->tgt_pos.y = hi_y;
        s->tgt_dir   = reflect_direction_about_horizontal_wall(s->tgt_dir);
    }
}

/*
 * multi_harmonic_steering_turn — three-harmonic CPG output (rad/s).
 *
 *     ω(t) = A₁·sin(f₁·t) + A₂·sin(f₂·t + φ₂) + A₃·sin(f₃·t + φ₃)
 *
 *   A SINGLE sinusoid produces a strictly periodic wander pattern —
 *   the target traces the same Lissajous figure forever and the eye
 *   reads the repetition. Summing three sinusoids with INCOMMENSURATE
 *   frequencies (f₁, f₂, f₃ chosen so no two are rational multiples)
 *   produces a QUASI-PERIODIC signal that never exactly repeats over
 *   any human-scale time window.
 *
 *   This is a discrete-time central pattern generator (CPG): the
 *   composite output drives autonomous motion without sensory
 *   feedback. Three is the minimum that gives convincing wander —
 *   one is too periodic, two often beat audibly.
 *
 *   Reference: Manton (1952) on biological CPGs; Reynolds (1999)
 *   §"Wander" — the multi-harmonic stagger pattern.
 */
static inline float multi_harmonic_steering_turn(float t)
{
    return TGT_TURN_AMP1 * sinf(TGT_TURN_FREQ1 * t)
         + TGT_TURN_AMP2 * sinf(TGT_TURN_FREQ2 * t + TGT_TURN_PHASE2)
         + TGT_TURN_AMP3 * sinf(TGT_TURN_FREQ3 * t + TGT_TURN_PHASE3);
}

/*
 * advance_wander_target — semi-implicit Euler step on the target's
 * (direction, position) state.
 *
 *     dir' = dir + turn · dt          (integrate heading first)
 *     pos' = pos + v · (cos dir', sin dir') · dt
 *
 *   "Semi-implicit" because we use the NEW dir to integrate position.
 *   Equivalent to explicit Euler at first order but reads "turn then
 *   move", which matches the physical intuition: the target rotates
 *   first, then walks in the rotated direction.
 */
static inline void advance_wander_target(Snake *s, float turn, float dt)
{
    s->tgt_dir += turn * dt;
    s->tgt_pos.x += s->tgt_speed * cosf(s->tgt_dir) * dt;
    s->tgt_pos.y += s->tgt_speed * sinf(s->tgt_dir) * dt;
}

/*
 * first_order_low_pass — exponential-moving-average smoother.
 *
 *     y_new = y_old + (x_raw − y_old) · α
 *
 *   Discrete-time form of the first-order linear IIR filter
 *   y' + (1/τ)·y = (1/τ)·x with α = dt/τ:
 *     α → 0  : output frozen (infinite smoothing)
 *     α → 1  : output follows input exactly (no smoothing)
 *
 *   α is clamped to [0, 1] so a large dt (e.g. paused frame) can't
 *   overshoot. Used here to round the target's sharp bounce-corners
 *   so the head's chase looks deliberate, not jittery.
 *
 *   Reference: Oppenheim & Schafer, "Discrete-Time Signal Processing"
 *   §3.6 — first-order IIR (leaky integrator).
 */
static inline Vec2 first_order_low_pass(Vec2 current, Vec2 raw, float alpha)
{
    if (alpha > 1.0f) alpha = 1.0f;
    if (alpha < 0.0f) alpha = 0.0f;
    return (Vec2){
        current.x + (raw.x - current.x) * alpha,
        current.y + (raw.y - current.y) * alpha,
    };
}

/*
 * analytic_one_link_ik_step — closed-form IK for a single rigid link.
 *
 *   The "arm" here has ONE link: the snake head. Goal-seeking it is
 *   trivial — no triangle, no law of cosines, no iteration:
 *
 *       1. Δ      = target − head            (displacement)
 *       2. d      = |Δ|                       (distance to target)
 *       3. if d < ε: already there, do nothing.
 *       4. heading = atan2(Δ.y, Δ.x)         (face the target)
 *       5. step = min(max_step, d)            (clip to not overshoot)
 *       6. head ← head + (Δ / d) · step      (walk forward)
 *
 *   The step CLIP is the IK-specific guard: a 1-link arm can land
 *   exactly on its target every frame, so without the clip the head
 *   would overshoot and bounce-back jitter at the apex of every
 *   pursuit. Clipping to `d` makes the head SETTLE on the target
 *   when it catches up.
 *
 *   Reference: Craig "Introduction to Robotics" §4 (analytical IK,
 *   1-link case is the trivial subset of the 2-link law of cosines).
 */
static inline void analytic_one_link_ik_step(Snake *s, Vec2 target,
                                             float max_step)
{
    float dx   = target.x - s->joint[0].x;
    float dy   = target.y - s->joint[0].y;
    float dist = sqrtf(dx * dx + dy * dy);
    if (dist <= 0.5f) return;             /* already at target */

    s->heading = atan2f(dy, dx);
    float step = (max_step < dist) ? max_step : dist;
    s->joint[0].x += (dx / dist) * step;
    s->joint[0].y += (dy / dist) * step;
}

/*
 * clamp_head_to_pixel_bounds — defensive screen-rect clip.
 *
 *   The analytic IK clip + target bouncing normally keep the head
 *   inside the screen. But a single large-dt frame (debugger pause,
 *   suspend + resume) can fling the head past the wall before the
 *   target's wall-reflection has a chance to redirect it. This is
 *   the safety net for that worst case.
 */
static inline void clamp_head_to_pixel_bounds(Snake *s, float wpx, float hpx)
{
    if (s->joint[0].x < 0.0f) s->joint[0].x = 0.0f;
    if (s->joint[0].x > wpx)  s->joint[0].x = wpx;
    if (s->joint[0].y < 0.0f) s->joint[0].y = 0.0f;
    if (s->joint[0].y > hpx)  s->joint[0].y = hpx;
}

/*
 * move_head — IK goal-seek pipeline: the head chases a wall-bouncing
 * wander target.
 *
 *   PHASE 1 — wander target self-propels:
 *     1. tgt_time += dt
 *     2. turn  = multi_harmonic_steering_turn(tgt_time)   (CPG)
 *     3. advance_wander_target(turn, dt)                  (Euler)
 *     4. bounce_target                                    (walls)
 *
 *   PHASE 2 — smooth raw target into actual_target:
 *     5. actual_target ← first_order_low_pass(actual_target, tgt_pos,
 *                                             dt · TGT_SMOOTH_RATE)
 *
 *   PHASE 3 — head pursues actual_target via 1-link IK:
 *     6. analytic_one_link_ik_step(head, actual_target,
 *                                  move_speed · dt)
 *     7. clamp_head_to_pixel_bounds                       (safety)
 *
 *   PHASE 4 — record:
 *     8. trail_push(head)        — FK chain follows this trail
 *     9. tgt_push(actual_target) — ghost-trail behind the target
 *
 *   The wander target is the INPUT to the IK system; the head is its
 *   OUTPUT. The head's chase is purely analytical 1-link IK (atan2 +
 *   clipped step) — no Jacobian, no iteration, no overshoot.
 *
 *   References: Manton + Reynolds for the CPG; Oppenheim & Schafer
 *   for the IIR low-pass; Craig §4 for the IK math.
 */
static void move_head(Snake *s, float dt, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    /* PHASE 1 — wander target self-propels */
    s->tgt_time += dt;
    float turn = multi_harmonic_steering_turn(s->tgt_time);
    advance_wander_target(s, turn, dt);
    bounce_target(s, wpx, hpx);

    /* PHASE 2 — IIR low-pass smooths bounce corners */
    s->actual_target = first_order_low_pass(s->actual_target, s->tgt_pos,
                                            dt * TGT_SMOOTH_RATE);

    /* PHASE 3 — head pursues smoothed target via 1-link IK */
    analytic_one_link_ik_step(s, s->actual_target, s->move_speed * dt);
    clamp_head_to_pixel_bounds(s, wpx, hpx);

    /* PHASE 4 — record into trails */
    trail_push(s, s->joint[0]);
    tgt_push  (s, s->actual_target);
}

/* ── §5c  compute_joints ────────────────────────────────────────────── */

/*
 * compute_joints() — place body joints by arc-length sampling of the trail.
 * Identical to snake_forward_kinematics.c: joint[i] is placed at distance
 * i × SEG_LEN_PX behind the head, measured along the actual path taken.
 */
static void compute_joints(Snake *s)
{
    for (int i = 1; i <= N_SEGS; i++)
        s->joint[i] = trail_sample(s, (float)i * SEG_LEN_PX);
}

/* ── §5d  bead rendering helpers ────────────────────────────────────── */

/*
 * seg_pair() — color pair index for body segment i (head=1, tail=N_PAIRS).
 * Linear interpolation: pair 1 (head) → pair N_PAIRS (tail tip).
 */
static int seg_pair(int i)
{
    return 1 + (i * (N_PAIRS - 1)) / (N_SEGS - 1);
}

/*
 * seg_attr() — ncurses attribute for body segment i.
 * Head quarter: A_BOLD (bright, draws the eye).
 * Tail quarter: A_DIM (fades into background, emphasises tail-end).
 */
static attr_t seg_attr(int i)
{
    if (i < N_SEGS / 4)       return A_BOLD;
    if (i > 3 * N_SEGS / 4)   return A_DIM;
    return A_NORMAL;
}

/*
 * joint_node_char() — bead marker at joint position i.
 * Head third:  '0' (thick, prominent node)
 * Middle:      'o' (standard bead)
 * Tail third:  '.' (small, receding)
 */
static chtype joint_node_char(int i)
{
    if (i <= (N_SEGS - 1) / 3)    return '0';
    if (i >= (N_SEGS - 1) * 2 / 3) return '.';
    return 'o';
}

/*
 * head_glyph() — directional arrow at the snake's head.
 * Maps heading (radians) to >, v, <, ^ based on 90° quadrants.
 */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;

    if (deg <  45.0f || deg >= 315.0f) return (chtype)'>';
    if (deg < 135.0f)                  return (chtype)'v';
    if (deg < 225.0f)                  return (chtype)'<';
    return                             (chtype)'^';
}

/* ── §5e  mark_cell — central glyph stamp helper ────────────────────── */

/*
 * mark_cell() — stamp one ASCII glyph at terminal cell (cx,cy).
 *
 * Centralises the (chtype)(unsigned char) cast, bounds-check, and the
 * wattron/wattroff sandwich so callers stay short.  The double cast
 * prevents sign-extension on character values > 127.  Off-screen cells
 * are silently dropped.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5f  draw_segment_beads ────────────────────────────────────────── */

/*
 * draw_segment_beads() — fill segment a→b with 'o' at DRAW_STEP_PX steps.
 *
 * Per-call dedup (prev_cx/prev_cy) prevents stamping a cell twice, which
 * would cause attribute flicker.  Off-screen cells are silently skipped
 * by mark_cell.  Pass 2 (in render_chain) over-stamps joint positions
 * with '0' / 'o' / '.' graded markers.
 */
static void draw_segment_beads(WINDOW *w,
                                Vec2 a, Vec2 b,
                                int pair, attr_t attr,
                                int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx;  prev_cy = cy;
        mark_cell(w, cx, cy, 'o', pair, attr, cols, rows);
    }
}

/* ── §5g  render_chain ──────────────────────────────────────────────── */

/*
 * lerp_joints() — fill rj[] with alpha-interpolated render positions.
 *
 * rj[i] = prev_joint[i] + (joint[i] − prev_joint[i]) · alpha
 */
static void lerp_joints(const Snake *s, float alpha, Vec2 rj[N_SEGS + 1])
{
    for (int i = 0; i <= N_SEGS; i++) {
        rj[i].x = s->prev_joint[i].x
                + (s->joint[i].x - s->prev_joint[i].x) * alpha;
        rj[i].y = s->prev_joint[i].y
                + (s->joint[i].y - s->prev_joint[i].y) * alpha;
    }
}

/*
 * draw_target_ghost() — render the wander target's recent path as dim dots.
 *
 * Walks the ghost ring buffer from oldest to newest so newer entries
 * over-stamp older on overlap.  PAIR_GHOST (244 grey) is theme-independent
 * and stays subtle against any body palette.
 */
static void draw_target_ghost(WINDOW *w, const Snake *s, int cols, int rows)
{
    int n = s->tgt_count;
    for (int k = n - 1; k >= 1; k--) {
        int idx = (s->tgt_head + TARGET_TRAIL_CAP - k) % TARGET_TRAIL_CAP;
        int cx  = px_to_cell_x(s->tgt_trail[idx].x);
        int cy  = px_to_cell_y(s->tgt_trail[idx].y);
        mark_cell(w, cx, cy, '.', PAIR_GHOST, A_DIM, cols, rows);
    }
}

/*
 * draw_target_cursor() — bright '+' at the actual_target so the user can
 * see what the head is chasing.  Drawn before the body so the snake
 * over-stamps it when the head catches up.
 */
static void draw_target_cursor(WINDOW *w, const Snake *s, int cols, int rows)
{
    int cx = px_to_cell_x(s->actual_target.x);
    int cy = px_to_cell_y(s->actual_target.y);
    mark_cell(w, cx, cy, '+', PAIR_HUD, A_BOLD, cols, rows);
}

/*
 * draw_body_fill() — bead-fill every segment, tail → head.  Tail-first
 * means warmer head-end colours win at any cell where two segments cross.
 */
static void draw_body_fill(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                           int cols, int rows)
{
    for (int i = N_SEGS - 1; i >= 0; i--) {
        draw_segment_beads(w,
                           rj[i + 1], rj[i],
                           seg_pair(i), seg_attr(i),
                           cols, rows);
    }
}

/*
 * draw_body_nodes() — over-stamp graded node markers ('0'/'o'/'.') at
 * every joint, tail → head so warmer colours win on overlap.
 */
static void draw_body_nodes(WINDOW *w, const Vec2 rj[N_SEGS + 1],
                            int cols, int rows)
{
    for (int i = N_SEGS; i >= 1; i--) {
        int cx = px_to_cell_x(rj[i].x);
        int cy = px_to_cell_y(rj[i].y);
        char ch = (char)joint_node_char(i);
        mark_cell(w, cx, cy, ch,
                  seg_pair(i - 1), seg_attr(i - 1),
                  cols, rows);
    }
}

/*
 * draw_head() — directional arrow at joint[0], drawn last so it always
 * reads above the body.
 */
static void draw_head(WINDOW *w, const Vec2 *head_pos, float heading,
                      int cols, int rows)
{
    int cx = px_to_cell_x(head_pos->x);
    int cy = px_to_cell_y(head_pos->y);
    char ch = (char)head_glyph(heading);
    mark_cell(w, cx, cy, ch, 1 /* PAIR_HEAD */, A_BOLD, cols, rows);
}

/*
 * render_chain() — orchestrate one frame in painter's order.
 *
 *   1. lerp_joints       — sub-tick interpolation for all 33 joints.
 *   2. draw_target_ghost — dim ghost trail of recent target positions.
 *   3. draw_target_cursor — bright '+' marker at actual_target.
 *   4. draw_body_fill    — bead fill, tail → head.
 *   5. draw_body_nodes   — graded node markers over-stamp the fill.
 *   6. draw_head         — directional arrow, always on top.
 */
static void render_chain(const Snake *s, WINDOW *w,
                          int cols, int rows, float alpha)
{
    Vec2 rj[N_SEGS + 1];
    lerp_joints(s, alpha, rj);

    draw_target_ghost (w, s,         cols, rows);
    draw_target_cursor(w, s,         cols, rows);
    draw_body_fill    (w, rj,        cols, rows);
    draw_body_nodes   (w, rj,        cols, rows);
    draw_head         (w, &rj[0], s->heading, cols, rows);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — composition root for §6.
 *
 * Intent
 *   In this demo the simulation IS one snake chasing one wander
 *   target (both stored inside the Snake struct). We keep a Scene
 *   wrapper anyway so the framework loop (scene_init / scene_tick /
 *   scene_draw) reads identically to every other file in the repo.
 *   If a future variant adds e.g. obstacles, secondary targets,
 *   or a swarm, they slot in here as siblings of `snake` without
 *   changing the loop.
 *
 * Simulation vs Rendering locality
 *   The Snake struct itself already separates trail / body / IK head
 *   / wander target / smoothing / ghost trail / UI. Within Scene
 *   there is no further split needed yet — one field, one concern.
 *   When adding a new member, place it as follows:
 *
 *     ── Simulation state ──   things scene_tick READS+WRITES
 *                              (prey[], obstacles[], goal switcher)
 *     ── Render-only state ──  things scene_draw READS, never the
 *                              physics path (camera, shake, FX)
 *
 * Things that DO NOT live here
 *   - fps / sim_fps counters → §8 App (frame-timing concern, not
 *     part of the world being simulated).
 *   - terminal extents (cols, rows) → §7 Screen.
 *   - signal flags (running, need_resize) → §8 App.
 *   - Theme table → file-scope `static const` in §3.
 *
 * One Scene per program; passed by pointer to every §6 entry point.
 */
typedef struct {
    /* ── Simulation state ─────────────────────────────────────── */
    Snake snake;               /* world: trail + body + IK + target */
} Scene;

/*
 * scene_init() — initialise snake to a clean, immediately-animated state.
 *
 * theme_idx is preserved across reset: saved before memset, restored after,
 * so r/R doesn't jump back to theme 0 unexpectedly.
 *
 * The trail is pre-populated (TRAIL_CAP entries, 1 px apart, extending
 * behind the head) so body joints are valid from frame one.
 *
 * actual_target and tgt_pos are initialised to the head position so
 * there is no initial snap; the wander target diverges naturally from
 * frame one as the harmonic steering accumulates.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    int saved_theme = sc->snake.theme_idx;
    memset(sc, 0, sizeof *sc);
    Snake *s = &sc->snake;
    s->theme_idx = saved_theme;

    s->move_speed = MOVE_SPEED_DEFAULT;
    s->tgt_speed  = TGT_WANDER_SPEED_DEFAULT;
    s->tgt_time   = 0.0f;
    s->tgt_dir    = (float)M_PI / 6.0f;   /* start heading slightly SE */
    s->heading    = 0.0f;
    s->paused     = false;

    /* Head at screen centre */
    s->joint[0].x = (float)(cols * CELL_W) * 0.5f;
    s->joint[0].y = (float)(rows * CELL_H) * 0.5f;

    /* Wander target starts near head (offset slightly so it leads) */
    s->tgt_pos       = s->joint[0];
    s->actual_target = s->joint[0];

    /*
     * Pre-populate trail: extend behind head pointing west (heading 0 → east,
     * so backward is west: bx = cos(π) = -1, by = 0).
     * 1 px spacing covers the full snake body length from frame one.
     */
    float bx = -1.0f;   /* unit vector pointing west */
    float by =  0.0f;
    for (int k = 0; k < TRAIL_CAP; k++) {
        s->trail[k].x = s->joint[0].x + (float)k * bx;
        s->trail[k].y = s->joint[0].y + (float)k * by;
    }
    s->trail_head  = 0;
    s->trail_count = TRAIL_CAP;

    compute_joints(s);
    memcpy(s->prev_joint, s->joint, sizeof s->joint);

    /* Seed target ghost trail with head position */
    for (int k = 0; k < TARGET_TRAIL_CAP; k++)
        s->tgt_trail[k] = s->joint[0];
    s->tgt_head  = 0;
    s->tgt_count = TARGET_TRAIL_CAP;
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Snake *s = &sc->snake;
    memcpy(s->prev_joint, s->joint, sizeof s->joint);   /* save for α lerp */
    if (s->paused) return;
    move_head(s, dt, cols, rows);
    compute_joints(s);
}

static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_chain(&sc->snake, w, cols, rows, alpha);
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
 *   ncurses' coordinate system is cells. Pixel space (CELL_W × CELL_H
 *   sub-pixels per cell) lives only inside §5 — converted at the
 *   draw boundary, per the project's "one conversion point" rule.
 *
 * References [15] Raymond, NCURSES Programming HOWTO.
 */
typedef struct {
    int cols;   /* terminal width  in CHARACTER CELLS */
    int rows;   /* terminal height in CHARACTER CELLS */
} Screen;

static void screen_init(Screen *s, int initial_theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init(initial_theme);
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
 * screen_draw() — compose the full frame: scene + HUD bars.
 *
 * Top-right HUD:  fps · simHz · speed · liss_speed · theme · state
 * Bottom hint:    keyboard reference for all controls
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    const Snake *sn = &sc->snake;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  spd:%.0f  tgt:%.0f  [%s]  %s ",
             fps, sim_fps,
             sn->move_speed,
             sn->tgt_speed,
             THEMES[sn->theme_idx].name,
             sn->paused ? "PAUSED " : "chasing");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  UD/ws:spd  +/-:tgt-spd  t/T:theme  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level container; everything outside the world.
 *
 * Intent
 *   Bundles the simulated world (Scene), the host terminal (Screen),
 *   and the session-level loop-control flags into one record so
 *   main() reads as four-line phases: init / service signals /
 *   step+draw / shutdown. Declared file-scope (g_app) so signal
 *   handlers — which cannot take a user argument — can write
 *   `running` and `need_resize` without globals scattered through
 *   the file.
 *
 * Locality of concern
 *   ── Owned subsystems ── nouns the app composes
 *      scene       — the world being simulated (§6)
 *      screen      — the terminal extent it draws to (§7)
 *
 *   ── Session state ── user settings that survive a reset
 *      sim_fps     — physics tick rate (cycled with [ / ])
 *
 *   ── Loop control ── verbs the loop reads each frame
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
 *   See [15] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Snake tuning values (tgt_speed, move_speed, theme_idx) —
 *     simulation/render state in §5 Snake.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

    /* ── Session state ────────────────────────────────────────── */
    int    sim_fps;            /* physics tick rate (Hz)               */

    /* ── Loop control ─────────────────────────────────────────── */
    volatile sig_atomic_t running;      /* main loop predicate            */
    volatile sig_atomic_t need_resize;  /* SIGWINCH pending               */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Snake *s   = &app->scene.snake;
    float  wpx = (float)(app->screen.cols * CELL_W);
    float  hpx = (float)(app->screen.rows * CELL_H);
    if (s->joint[0].x >= wpx) s->joint[0].x = wpx - 1.0f;
    if (s->joint[0].y >= hpx) s->joint[0].y = hpx - 1.0f;
    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit
 *   space         toggle paused
 *   ↑ / w / W     move_speed × 1.20
 *   ↓ / s / S     move_speed ÷ 1.20  (clamped [MOVE_SPEED_MIN, MAX])
 *   + / =         liss_speed × 1.25
 *   -             liss_speed ÷ 1.25  (clamped [LISS_SPEED_MIN, MAX])
 *   t             next theme (wraps)
 *   T             previous theme (wraps)
 *   r / R         reset simulation (theme preserved)
 *   ]             sim_fps + step     (clamped [SIM_FPS_MIN, MAX])
 *   [             sim_fps - step
 */
static bool app_handle_key(App *app, int ch)
{
    Snake *s = &app->scene.snake;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        s->paused = !s->paused;
        break;

    case KEY_UP: case 'w': case 'W':
        s->move_speed *= 1.20f;
        if (s->move_speed > MOVE_SPEED_MAX) s->move_speed = MOVE_SPEED_MAX;
        break;
    case KEY_DOWN: case 's': case 'S':
        s->move_speed /= 1.20f;
        if (s->move_speed < MOVE_SPEED_MIN) s->move_speed = MOVE_SPEED_MIN;
        break;

    case '+': case '=':
        s->tgt_speed *= 1.25f;
        if (s->tgt_speed > TGT_WANDER_SPEED_MAX) s->tgt_speed = TGT_WANDER_SPEED_MAX;
        break;
    case '-':
        s->tgt_speed /= 1.25f;
        if (s->tgt_speed < TGT_WANDER_SPEED_MIN) s->tgt_speed = TGT_WANDER_SPEED_MIN;
        break;

    case 't':
        s->theme_idx = (s->theme_idx + 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;
    case 'T':
        s->theme_idx = (s->theme_idx + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme_idx);
        break;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/* ─────────────────────────────────────────────────────────────────────
 * main() — game loop (structure identical to framework.c §8)
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);

    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen, 0);   /* theme 0 = Medusa on startup */
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ── ① resize ────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── ② dt ────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ─────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ───────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ──────────────────── *
         * Budget = 1/60 s.  elapsed is wall time spent on physics +
         * accounting since frame_start; sleep the remainder so the
         * render rate sits at 60 fps regardless of sim Hz.            */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain input ───────────────────────────────────────── */
        int ch;
        while ((ch = getch()) != ERR) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }
    }

    screen_free(&app->screen);
    return 0;
}
