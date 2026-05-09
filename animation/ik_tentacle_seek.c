/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_tentacle_seek.c — 16-link FABRIK tentacle with joint angle constraints
 *
 * DEMO: A 16-segment tentacle anchored at the screen centre uses FABRIK
 *       inverse kinematics with per-joint angle limits (~63° each) to
 *       track a target tracing a 1:1.7-ratio Lissajous figure. The
 *       angle clamp prevents kinking; a low-pass filter smooths the
 *       target's velocity so the tentacle curls fluidly through the
 *       path's internal crossings. A bright dotted ghost trail in the
 *       tip-colour glows along the figure being traced.
 *
 * Study alongside: ik_arm_reach.c   (FABRIK without joint constraints)
 *                  ik_spider.c      (analytical 2-joint IK contrast)
 *
 * Section map:
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — 10 themes + spec HUD/hint pairs
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Tentacle: FABRIK + constraints + Lissajous + trail
 *       §5a  vec2 helpers (arithmetic, dot, cross, rotate)
 *       §5b  joint constraint (per-joint angle clamp)
 *       §5c  FABRIK solver (forward/backward/orchestrator)
 *       §5d  Lissajous target + smooth tracking
 *       §5e  trail ring buffer
 *       §5f  rendering helpers
 *       §5g  render_tentacle (orchestrator)
 *   §6  scene         — thin Scene wrapper
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 *
 * Keys:  q / ESC     quit                 space   pause / resume
 *        w/+/=       speed scale × 1.25   s/-     speed scale ÷ 1.25
 *        t / T       cycle theme forward / backward
 *        [ / ]       time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_tentacle_seek.c \
 *       -o ik_tentacle_seek -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : FABRIK iterative IK with per-joint angle constraints.
 *
 *                 BACKWARD pass: snap tip to target; walk root-ward
 *                 sliding each parent joint along the child→parent
 *                 unit direction so the link length is restored. After
 *                 each slide, clamp the bend angle at the just-moved
 *                 joint. After this pass, all link lengths are correct
 *                 and the tip is at the target, but the root has
 *                 drifted from its anchor.
 *
 *                 FORWARD pass: snap root back to anchor; walk tip-ward
 *                 sliding each child joint to restore the link length.
 *                 After this pass, root anchored and all links correct;
 *                 the tip has drifted from target — by less than before.
 *
 *                 Each iteration strictly reduces |tip − target| (proven
 *                 monotonic convergence). Convergence in 3–8 iterations
 *                 for smooth target motion; MAX_ITER=20 safety cap.
 *
 *                 JOINT CONSTRAINT (the addition over plain FABRIK):
 *                 after the backward pass repositions joint i, compute
 *                 the signed angle between the incoming and outgoing
 *                 link directions via atan2(cross, dot). If |angle| >
 *                 MAX_JOINT_BEND, rotate the outgoing direction by the
 *                 correction and reposition the child joint. This
 *                 prevents kinks at fast target reversals.
 *
 *                 LISSAJOUS TARGET: parametric figure with frequency
 *                 ratio 1:1.7 (a = 10:17 rational that gives a 62.8 s
 *                 quasi-period with internal crossings).
 *                 ACTUAL_TARGET tracks this via low-pass lerp at rate
 *                 TARGET_SMOOTH = 8 1/s (τ ≈ 125 ms) so the FABRIK
 *                 solver never sees a discontinuous velocity.
 *
 * Data-structure: Tentacle holds joint position array pos[N_JOINTS=13],
 *                 fixed link_len[N_LINKS=12] (tapered), the actual_target
 *                 lerp state, and a TRAIL_POINTS=120 ring buffer of
 *                 recent target positions for the ghost-dot trail.
 *
 * Rendering     : Painter's order — ghost trail dots → bead-fill links
 *                 with root-to-tip color gradient → bold node markers
 *                 ('0' root third, 'o' middle, '.' tip third) on top of
 *                 fill → target marker ('*' converged or '#' at-limit) →
 *                 anchor '0'. Two-pass body (fill + nodes) gives the
 *                 articulated bead-on-string look.
 *
 * Performance   : Variable timestep at render rate. Per frame:
 *                 worst-case 20 FABRIK iterations × 17 joints × 2 passes
 *                 ≈ 680 vector ops + 16 constraint clamps. Microseconds
 *                 total. ncurses doupdate sends only changed cells.
 *
 * References    :
 *   Aristidou & Lasenby, "FABRIK: A fast, iterative solver for the
 *     inverse kinematics problem" (Graphical Models 2011) — original
 *     paper including the convergence proof.
 *     https://www.andreasaristidou.com/FABRIK.html
 *   Aristidou & Lasenby, "Inverse kinematics solutions using conformal
 *     geometric algebra" (2011) — extension showing joint constraints
 *     can bolt onto FABRIK without breaking convergence.
 *   Wikipedia, "Lissajous curve" — derives why a non-integer frequency
 *     ratio gives a quasi-periodic figure.
 *   Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) — case
 *     for fixed-step (stiff sims); we don't qualify, hence variable.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A long whip pinned at one end, told where its tip should be. FABRIK
 * is the trick that makes the whip put its tip there: snap tip, walk
 * back fixing lengths; snap root, walk forward fixing lengths;
 * repeat. The new piece in this file is that each joint has a maximum
 * bend — after each backward step, if the joint just exceeded its
 * angle limit, rotate the outgoing direction back into bounds.
 * Result: a long fluid tentacle that reaches anywhere it can but
 * never folds back on itself.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Whip   : 16 stiff links pinned end-to-end with hinge joints. Each
 *          joint can rotate freely except for a ±63° limit on its bend.
 *
 * FABRIK : a coin-flip between two constraints. "Tip must reach target"
 *          (snap). "Root must stay anchored" (snap). Each round violates
 *          the OTHER less; 3–8 rounds converges to satisfying both.
 *
 * Limit  : after each backward step at joint i, look at the angle between
 *          its incoming and outgoing link directions. If too sharp, rotate
 *          the outgoing direction back into the allowed range. The chain
 *          can curl but cannot kink.
 *
 * Target : a planet on a quasi-periodic orbit. The orbit has internal
 *          crossings where the planet's velocity reverses sharply — a
 *          low-pass filter on the actual_target absorbs those reversals
 *          so the whip never sees a discontinuous goal.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. Update target:
 *       a. scene_time += dt · speed_scale
 *       b. lis = (anchor.x + Ax cos(ω_x · t), anchor.y + Ay sin(ω_y · t + φ))
 *       c. actual_target += (lis − actual_target) · clamp(dt · TARGET_SMOOTH, 0, 1)
 *       d. push actual_target into the trail ring buffer
 *  3. Reachability: if |target − anchor| ≥ Σ link_len, stretch chain
 *     straight at the target, set at_limit, return.
 *  4. Else FABRIK loop (up to MAX_ITER):
 *       a. BACKWARD pass: pos[N−1] = target.
 *          For i = N−2 downto 0:
 *            slide pos[i] toward pos[i+1] to restore link_len[i];
 *            if i > 0, apply joint angle clamp.
 *       b. FORWARD pass: pos[0] = anchor.
 *          For i = 0 to N−2:
 *            slide pos[i+1] away from pos[i] to restore link_len[i].
 *       c. If |pos[N−1] − target| < CONV_TOL, break.
 *  5. Render: trail dots → link fill (gradient) → joint markers → target
 *     marker → anchor marker.
 *
 * KEY FORMULAS
 * ────────────
 *  Lissajous       : x(t) = anchor.x + Ax · cos(ω_x · t)
 *                    y(t) = anchor.y + Ay · sin(ω_y · t + φ)
 *                    ω_x : ω_y = 1 : 1.7 → quasi-periodic (period 62.8 s)
 *
 *  Smooth tracking : rate = clamp(dt · TARGET_SMOOTH, 0, 1)
 *                    actual_target += (lis − actual_target) · rate
 *                    (first-order IIR low-pass; τ = 1/TARGET_SMOOTH)
 *
 *  Backward pass   : for i = N−2 downto 0:
 *                      d      = norm(pos[i] − pos[i+1])
 *                      pos[i] = pos[i+1] + d · link_len[i]
 *
 *  Forward pass    : for i = 0 to N−2:
 *                      d        = norm(pos[i+1] − pos[i])
 *                      pos[i+1] = pos[i] + d · link_len[i]
 *
 *  Joint angle     : dir_in  = norm(pos[i] − pos[i−1])
 *                    dir_out = norm(pos[i+1] − pos[i])
 *                    angle   = atan2(cross(dir_in, dir_out),
 *                                    dot  (dir_in, dir_out))
 *                    if |angle| > MAX_JOINT_BEND:
 *                      delta   = clamp(angle, ±MAX_JOINT_BEND) − angle
 *                      new_dir = rotate(dir_out, delta)
 *                      pos[i+1] = pos[i] + new_dir · link_len[i]
 *
 *  Tapered links   : link_len[i] = max(BASE_LINK_LEN − i · TAPER, 4 px)
 *                    (root longest; tip shortest — biomechanical look)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - Coincident joints. norm() guards against zero-length input by
 *    returning (1, 0). If two consecutive joints land at the same
 *    pixel, the next iteration separates them; safe.
 *
 *  - Target out of reach. Skip iteration; stretch chain collinearly
 *    toward target. Without the early-out FABRIK would oscillate.
 *
 *  - Constraint placement. The clamp must run during the BACKWARD
 *    pass, immediately after each joint is repositioned (not in the
 *    forward pass). Putting it in forward would let the backward pass
 *    create kinks that the forward pass can't unwind without breaking
 *    convergence.
 *
 *  - Smoothing rate vs. FABRIK iteration count. If TARGET_SMOOTH is
 *    too high (target jumps fast), the joint clamp fires aggressively
 *    and the chain stiffens. 8 1/s is tuned so the 12-link chain
 *    tracks at default Lissajous speed without visible stiffening.
 *
 *  - Suspend / lid-close. dt clamped to 100 ms in main() so the
 *    tentacle doesn't snap to a wildly future Lissajous point.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: tentacle traces the 1:1.7 Lissajous, taking ~63 s
 *    to revisit any point. Tip never quite reaches the target during
 *    fast direction changes — the smoothing visibly lags ~125 ms.
 *
 *  - Crank speed with `+` repeatedly: tip increasingly lags; HUD's
 *    iteration count climbs toward MAX_ITER as the solver works harder.
 *
 *  - At extreme speeds the tentacle visibly "stiffens" near tight
 *    crossings — that's the joint angle clamp firing on consecutive
 *    joints. Slow time with `[` to study the effect.
 *
 *  - When the target wanders to a point beyond Σ link_len from anchor,
 *    HUD shows "at-limit" and the tentacle stretches straight at the
 *    target (target glyph changes from '*' to '#').
 *
 *  - Cycle themes with `t`/`T` → arm gradient changes; HUD stays
 *    bright yellow regardless.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read ik_arm_reach.c first if FABRIK is new — that
 *      file teaches the basic two-pass algorithm. This file ADDS
 *      per-joint angle limits + low-pass target smoothing.
 *   2. §5 entity — THE HEART of this file. In sub-section order:
 *        §5b joint constraint     ← angle clamp helper (T3)
 *        §5c FABRIK solver        ← read AFTER tutorials T1-T4
 *        §5d Lissajous + smooth   ← target velocity filter (T5)
 *        §5e trail ring buffer
 *        §5f-§5g rendering
 *   3. §6 scene — thin wrapper.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if you've seen the
 *      framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pos[i]                joint i position (Vec2). 0 = root, N-1 = tip.
 *   link_len[i]           constant length of link between i and i+1.
 *                         Tapered root → tip.
 *   actual_target         smoothed version of the Lissajous point;
 *                         what the solver actually chases.
 *   TARGET_SMOOTH         IIR low-pass rate (1/s). Higher = snappier;
 *                         lower = laggier.
 *   MAX_JOINT_BEND        per-joint angle limit, radians. Symmetric
 *                         (±63°).
 *   dir_in, dir_out       unit vectors INTO and OUT OF a joint.
 *   angle                 signed angle between dir_in and dir_out
 *                         via atan2(cross, dot).
 *
 * Background you need
 * ───────────────────
 *   - FABRIK (ik_arm_reach T2-T6).
 *   - atan2(y, x) — four-quadrant arctan.
 *   - 2-D rotation: rotate(v, δ) = (v.x cos δ − v.y sin δ,
 *                                    v.x sin δ + v.y cos δ).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Quaternions / 3-D twist constraints. We're planar.
 *   - Inverse-time / reverse-mode IK ("CCD with ageing"). Plain
 *     FABRIK + clamp is plenty for this scale.
 *   - Soft / penalty-based constraints. Hard clamp is enough.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a constrained, smoothed FABRIK
 * tentacle from first principles.
 *
 *   T1  Why plain FABRIK kinks on a 16-link chain
 *   T2  Joint angle limits — what to clamp and where
 *   T3  Computing the signed angle at a joint
 *   T4  Why constraint runs in BACKWARD only (not forward)
 *   T5  Target smoothing — why the chain needs a low-pass filter
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY PLAIN FABRIK KINKS ON A 16-LINK CHAIN
 * ─────────────────────────────────────────────
 * Run ik_arm_reach.c with N_LINKS = 16 instead of 4 and trace
 * a fast Lissajous. You'll see the chain SHARPLY KINK at random
 * joints — adjacent links making 170° angles, the chain folding
 * back on itself in a hairpin.
 *
 * Where does the kink come from? FABRIK preserves LINK LENGTHS
 * but says NOTHING about the angle between consecutive links.
 * As long as |pos[i] − pos[i+1]| = link_len[i], the algorithm
 * is happy. It will gladly fold the chain into a 180° hairpin
 * if the geometry asks for it.
 *
 * For a robot arm with 4 hinges that's fine — physical hinges
 * have travel limits and you wouldn't ask them to fold. For a
 * tentacle with 16 segments, the chain has 16 DEGREES OF
 * FREEDOM and any unconstrained one of them can flip without
 * the others noticing. Result: kinks during fast direction
 * changes.
 *
 * Real tentacles don't kink. We need to ADD an angle constraint
 * that rejects the kinking poses.
 *
 * T2  JOINT ANGLE LIMITS — WHAT TO CLAMP AND WHERE
 * ────────────────────────────────────────────────
 * The constraint is: the angle between consecutive link
 * directions cannot exceed MAX_JOINT_BEND.
 *
 *   dir_in  = normalize(pos[i]   − pos[i-1])    incoming link
 *   dir_out = normalize(pos[i+1] − pos[i])      outgoing link
 *   angle   = signed angle from dir_in to dir_out
 *
 *   if |angle| > MAX_JOINT_BEND:
 *     correction = sign(angle) · MAX_JOINT_BEND − angle
 *     new_dir_out = rotate(dir_out, correction)
 *     pos[i+1] = pos[i] + new_dir_out · link_len[i]
 *
 * Where do we run this clamp? After EACH joint is repositioned
 * during the FABRIK pass. So as we walk the chain we both
 * preserve length AND clamp angle joint-by-joint.
 *
 * Note we move POS[i+1], not POS[i]. The constraint at joint i
 * affects the OUTGOING link (i, i+1), so the joint that needs
 * to move is the FAR END of the outgoing link.
 *
 * T3  COMPUTING THE SIGNED ANGLE AT A JOINT
 * ─────────────────────────────────────────
 * "Signed angle from u to v" — radians, ∈ (−π, +π], positive
 * for counter-clockwise rotation. Standard 2-D recipe:
 *
 *     angle = atan2( cross(u, v), dot(u, v) )
 *
 *     where cross(u, v) = u.x · v.y − u.y · v.x       (scalar)
 *           dot  (u, v) = u.x · v.x + u.y · v.y       (scalar)
 *
 * Reasoning:
 *   - dot(u, v)   = |u| · |v| · cos(angle)
 *   - cross(u, v) = |u| · |v| · sin(angle)
 *
 *   atan2(sin, cos) recovers the angle without sign loss. (Plain
 *   acos(dot) would lose sign info — it's always non-negative.)
 *
 * The "rotate by δ" used to apply the correction:
 *
 *     rotate(v, δ) = (v.x · cos δ − v.y · sin δ,
 *                     v.x · sin δ + v.y · cos δ)
 *
 * Standard 2-D rotation matrix applied to a vector. This is in
 * §5a vec2 helpers.
 *
 * Limit choice: MAX_JOINT_BEND = ~63° (1.1 rad). Bent further,
 * even consecutive joints can't reach the target geometry; the
 * chain visibly stiffens but doesn't kink. Tune up for a more
 * flexible tentacle, down for a stiffer one.
 *
 * T4  WHY CONSTRAINT RUNS IN BACKWARD ONLY (NOT FORWARD)
 * ──────────────────────────────────────────────────────
 * Tempting symmetry: clamp during BOTH passes. But that breaks
 * convergence.
 *
 * The reason: each FABRIK pass must EITHER restore the boundary
 * condition (tip / root snap) OR adjust intermediate joints —
 * not both. A pass that does both can introduce non-shrinking
 * adjustments and the algorithm oscillates.
 *
 * Aristidou & Lasenby's recommendation: clamp ONLY during the
 * pass that ALREADY moves intermediate joints — that is, the
 * BACKWARD pass (the one that walks tip-to-root in this file's
 * orientation, root-to-tip in others). The FORWARD pass just
 * restores root and re-stretches; no clamp.
 *
 * If you reverse the convention (forward = tip-to-root), swap
 * which pass clamps. The general rule: clamp on the pass that
 * starts from a "free" end (the end you just snapped, not the
 * anchored end). That ensures the constraint adjustment doesn't
 * fight the boundary condition restoration.
 *
 * In this file the BACKWARD pass starts at the tip (snaps to
 * target) and walks rootward; that's where clamping happens.
 * The FORWARD pass starts at the root (snaps to anchor) and
 * walks tipward; no clamping.
 *
 * T5  TARGET SMOOTHING — WHY THE CHAIN NEEDS A LOW-PASS FILTER
 * ────────────────────────────────────────────────────────────
 * The Lissajous target moves SMOOTHLY — its position is a
 * cosine. But the target's VELOCITY can change abruptly at
 * the figure's internal crossings (where two strands of the
 * curve cross at near-perpendicular).
 *
 * If the FABRIK solver chases the raw Lissajous point, the
 * sudden velocity reversal forces the chain into a "snap"
 * pose — and at fast Lissajous rates, the joint clamp fires
 * cascadingly across all 16 joints. Visually: stiffness pops.
 *
 * The fix is a low-pass filter on the chase target:
 *
 *     rate = clamp(dt · TARGET_SMOOTH, 0, 1)
 *     actual_target += (lissajous − actual_target) · rate
 *
 * That's a first-order IIR (infinite impulse response) low-pass
 * filter. Properties:
 *   - Time constant τ = 1 / TARGET_SMOOTH.
 *   - When the target jumps abruptly, actual_target follows
 *     exponentially with time-constant τ.
 *   - When the target moves slowly, actual_target tracks
 *     near-perfectly (the lerp coefficient is small but
 *     consistent).
 *
 * TARGET_SMOOTH = 8 1/s → τ ≈ 125 ms — visible lag, but no
 * sharp velocity discontinuities. The chain reads as
 * elastic-following rather than instantaneous tracking.
 *
 * This filter is independent of FABRIK; it just makes the
 * SOLVER's job easier by feeding it smooth motion. Same trick
 * is used in real animation rigs (often called "chase damping"
 * or "follow lag").
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
    /* Render frame-rate target. Variable-timestep simulation. */
    TARGET_FPS    = 60,

    /* HUD layout. */
    HUD_COLS      = 96,
    FPS_UPDATE_MS = 500,

    /* ncurses pair IDs.
     *   1..N_PAIRS  body gradient (root dark → tip bright), themed
     *   8 PAIR_HUD   bright yellow status (theme-independent)
     *   9 PAIR_HINT  bright cyan hint     (theme-independent) */
    N_PAIRS       = 7,
    PAIR_HUD      = 8,
    PAIR_HINT     = 9,

    N_THEMES      = 10,    /* cycled with `t`/`T` */

    /* Joint count: 17 joints = 16 links. Tip = pos[16]; root = pos[0].
     * Longer chain than the canonical 12-link FABRIK demo so the joint
     * angle constraint visibly fires across more bends — easier to see
     * the curl-without-kink behaviour in action. */
    N_JOINTS      = 17,
    N_LINKS       = 16,

    /* MAX_ITER: FABRIK convergence cap. Typical 16-link chains converge
     * in 4–10 iterations; 20 is a safety ceiling. Cost: 20 · 17 · 2 = 680
     * vector ops per tick — negligible. */
    MAX_ITER      = 20,

    /* Trail buffer capacity. At ~60 fps this is ~2 s of target history;
     * every 3rd point rendered → 40 visible dots. */
    TRAIL_POINTS  = 120,
};

/* CONV_TOL — FABRIK convergence threshold (pixel space). 2.0 px is
 * sub-cell on both axes (CELL_W=8, CELL_H=16) — further iterations
 * produce no visible improvement. */
#define CONV_TOL        2.0f

/* MAX_JOINT_BEND — per-joint bend angle limit (radians). 1.1 rad ≈ 63°.
 * Below 45° the chain is too stiff to reach close targets. Above 90°
 * visible kinking appears at fast direction reversals. 1.1 is the
 * biological sweet spot. */
#define MAX_JOINT_BEND  1.1f

/*
 * Link geometry — tapered chain.
 *   link_len[i] = max(BASE_LINK_LEN − i · TAPER, 4 px)
 *   i=0  (root):  22.0 px,  i=8 (mid): 16.4 px,  i=15 (tip): 11.5 px
 *   Σ = 16 · 22 − 0.7 · 120 = 268 px total reach
 * Tapering is biomechanical: root link long (lever arm), tip short (nimble).
 * The 0.7 taper (gentler than the typical 0.8) keeps even tip links
 * substantial so the tip is articulated, not vestigial.
 */
#define BASE_LINK_LEN   22.0f
#define TAPER            0.7f

/* DRAW_STEP_PX — bead fill stride. < CELL_W (8) so dense stamping never
 * skips a column. */
#define DRAW_STEP_PX    5.0f

/*
 * Lissajous target parameters.
 *
 *   x(t) = anchor.x + LIS_AX · cos(LIS_OMEGA_X · scene_time)
 *   y(t) = anchor.y + LIS_AY · sin(LIS_OMEGA_Y · scene_time + LIS_PHASE_Y)
 *
 * LIS_OMEGA_X : LIS_OMEGA_Y = 1 : 1.7 = 10 : 17 — large LCM gives a
 * 62.8 s quasi-period with internal crossings that exercise the IK
 * solver with varied directional demands.
 *
 * LIS_AX/AY scaled to 55%/38% of screen — fits within tentacle reach.
 * LIS_PHASE_Y = π/3 avoids a tangent cusp at the figure's start.
 */
#define LIS_AX_FACTOR   0.55f
#define LIS_AY_FACTOR   0.38f
#define LIS_OMEGA_X     1.0f
#define LIS_OMEGA_Y     1.7f
#define LIS_PHASE_Y     ((float)M_PI / 3.0f)

/*
 * TARGET_SMOOTH — first-order low-pass rate (1/s). At 60 fps,
 * dt · TARGET_SMOOTH ≈ 0.133 → time constant τ = 1/8 s ≈ 125 ms.
 * Absorbs sudden target velocity reversals at Lissajous crossings so
 * the joint clamp never fires aggressively.
 */
#define TARGET_SMOOTH   8.0f

/* Speed scale — user-controlled Lissajous time multiplier on +/-/w/s. */
#define SPEED_DEFAULT   1.0f
#define SPEED_MIN       0.05f
#define SPEED_MAX       8.0f

/* Time scale — user-controlled simulation time multiplier on `[/]`. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions (aspect-ratio bridge). */
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
/* §3  color — 10 themes + spec-fixed HUD pairs                          */
/* ===================================================================== */

/*
 * Per-theme body palette (pairs 1..N_PAIRS, root → tip). HUD/HINT
 * pairs are theme-independent (CLAUDE.md HUD spec).
 *
 * All entries sit in the bright half of the 256-colour space:
 *   - cube colours: ≥ 24 (brightness rule)
 *   - grayscale:    ≥ 240 (the 232-239 zone vanishes under A_DIM)
 */
typedef struct {
    const char *name;
    int body[N_PAIRS];   /* pairs 1..7 */
} Theme;

static const Theme THEMES[N_THEMES] = {
    {"Medusa", { 57,  63,  93,  99, 105, 111, 159}},
    {"Matrix", { 24,  28,  34,  40,  46,  82, 118}},
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}},
    {"Ocean",  { 24,  25,  27,  33,  39,  45,  51}},
    {"Nova",   { 54,  55,  56,  57,  93, 129, 165}},
    {"Toxic",  { 24,  58,  64,  70,  76,  82, 118}},
    {"Lava",   { 52,  88, 124, 160, 196, 202, 208}},
    {"Ghost",  {240, 244, 246, 248, 250, 252, 255}},
    {"Aurora", { 24,  28,  64,  71,  78, 121, 159}},
    {"Neon",   {201, 165, 129,  93,  57,  51,  45}},
};

/* theme_apply — re-bind body pairs to the chosen theme. HUD/HINT
 * pairs are NEVER touched here. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_PAIRS; p++)
        init_pair(p + 1, t->body[p], -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-color fallback — coarse purple→blue→cyan→white gradient. */
        init_pair(1, COLOR_MAGENTA, -1);
        init_pair(2, COLOR_MAGENTA, -1);
        init_pair(3, COLOR_BLUE,    -1);
        init_pair(4, COLOR_BLUE,    -1);
        init_pair(5, COLOR_CYAN,    -1);
        init_pair(6, COLOR_CYAN,    -1);
        init_pair(7, COLOR_WHITE,   -1);
    }

    /* HUD pairs are theme-independent — bright yellow status, bright
     * cyan hint, both on default bg so they overlay any theme. */
    init_pair(PAIR_HUD,  COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLORS >= 256 ?  51 : COLOR_CYAN,   -1);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio helpers                          */
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
/* §5  entity — Tentacle                                                  */
/* ===================================================================== */

/* Vec2 — 2-D position vector in pixel space. */
typedef struct { float x, y; } Vec2;

/* ── §5a  vec2 helpers ──────────────────────────────────────────────── */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline Vec2 vec2_add  (Vec2 a, Vec2 b)
{ return (Vec2){ a.x + b.x, a.y + b.y }; }

static inline Vec2 vec2_sub  (Vec2 a, Vec2 b)
{ return (Vec2){ a.x - b.x, a.y - b.y }; }

static inline Vec2 vec2_scale(Vec2 a, float s)
{ return (Vec2){ a.x * s, a.y * s }; }

static inline Vec2 vec2_lerp (Vec2 a, Vec2 b, float t)
{ return (Vec2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t }; }

static inline float vec2_len (Vec2 a)
{ return sqrtf(a.x * a.x + a.y * a.y); }

static inline float vec2_dist(Vec2 a, Vec2 b)
{ return vec2_len(vec2_sub(b, a)); }

/* vec2_norm — unit vector. Zero-length input returns (1, 0); the
 * specific direction is arbitrary but lets callers avoid NaN. */
static inline Vec2 vec2_norm(Vec2 a)
{
    float len = vec2_len(a);
    if (len < 1e-6f) return (Vec2){ 1.0f, 0.0f };
    return (Vec2){ a.x / len, a.y / len };
}

/* dot, cross — for unit vectors, cos(θ) and sin(θ). Together with
 * atan2 they give the signed angle in [−π, π], which is what the
 * joint-angle clamp needs (acos would lose the sign). */
static inline float vec2_dot  (Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }
static inline float vec2_cross(Vec2 a, Vec2 b) { return a.x*b.y - a.y*b.x; }

/* vec2_rotate — rotate v by `angle` rad. Used by the joint clamp to
 * rotate the outgoing link direction by the correction. */
static inline Vec2 vec2_rotate(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ── Tentacle state ─────────────────────────────────────────────────── */

typedef struct {
    /* joint chain (pixel space) */
    Vec2  pos[N_JOINTS];
    float link_len[N_LINKS];

    /* fixed root */
    Vec2  anchor;

    /* Lissajous target oscillator + smoothing */
    Vec2  actual_target;       /* low-pass filtered target tracked by IK   */
    float scene_time;          /* phase accumulator (s)                     */
    float speed_scale;         /* user-tunable Lissajous time multiplier    */

    /* trail ring buffer — recent actual_target positions */
    Vec2  trail_pts[TRAIL_POINTS];
    int   trail_write;          /* next-write index                          */
    int   trail_fill;           /* valid entries, ≤ TRAIL_POINTS             */

    /* state flags */
    int   last_iter;            /* iterations the last fabrik_solve used    */
    bool  at_limit;             /* target out of reach this tick            */
    bool  paused;
    int   theme_idx;
} Tentacle;

/* ── §5b  joint angle constraint ───────────────────────────────────── */

/*
 * apply_joint_constraint — clamp bend angle at joint i.
 *
 * Called inside the FABRIK BACKWARD pass immediately after pos[i] is
 * repositioned. At this point pos[i+1] is the (still-valid) child
 * position from the prior iteration; we may rotate the outgoing
 * direction to bring the bend within bounds, then reposition pos[i+1].
 *
 * The signed angle from dir_in to dir_out comes from atan2(sin θ, cos θ)
 * = atan2(cross, dot) — this gives the correct sign whereas acos(dot)
 * would lose it.
 *
 * Endpoints have nothing to clamp: joint 0 has no incoming link, joint
 * N−1 has no outgoing link. Caller guards i ∈ [1, N−2].
 */
static void apply_joint_constraint(Tentacle *t, int i)
{
    if (i < 1 || i >= N_JOINTS - 1) return;

    Vec2 dir_in  = vec2_norm(vec2_sub(t->pos[i],     t->pos[i - 1]));
    Vec2 dir_out = vec2_norm(vec2_sub(t->pos[i + 1], t->pos[i]));

    float cr    = vec2_cross(dir_in, dir_out);   /* sin θ */
    float dt    = vec2_dot  (dir_in, dir_out);   /* cos θ */
    float angle = atan2f(cr, dt);                /* signed θ in [−π, π] */

    if (fabsf(angle) <= MAX_JOINT_BEND) return;

    float clamped = clampf(angle, -MAX_JOINT_BEND, MAX_JOINT_BEND);
    float delta   = clamped - angle;
    Vec2  new_dir = vec2_rotate(dir_out, delta);
    t->pos[i + 1] = vec2_add(t->pos[i],
                             vec2_scale(new_dir, t->link_len[i]));
}

/* ── §5c  FABRIK solver ─────────────────────────────────────────────── */

/* total_link_length — sum of all link lengths. Constant after init. */
static float total_link_length(const Tentacle *t)
{
    float total = 0.0f;
    for (int i = 0; i < N_LINKS; i++) total += t->link_len[i];
    return total;
}

/* stretch_collinear — out-of-reach posture. Lay all joints along the
 * unit vector from anchor toward target. */
static void stretch_collinear(Tentacle *t, Vec2 anchor, Vec2 target)
{
    Vec2  dir   = vec2_norm(vec2_sub(target, anchor));
    float accum = 0.0f;
    t->pos[0]   = anchor;
    for (int i = 0; i < N_LINKS; i++) {
        accum += t->link_len[i];
        t->pos[i + 1] = vec2_add(anchor, vec2_scale(dir, accum));
    }
}

/* fabrik_backward_pass — pin tip to target, walk root-ward restoring
 * link lengths. Apply joint clamp after each interior repositioning. */
static void fabrik_backward_pass(Tentacle *t, Vec2 target)
{
    t->pos[N_JOINTS - 1] = target;
    for (int i = N_JOINTS - 2; i >= 0; i--) {
        Vec2 dir  = vec2_norm(vec2_sub(t->pos[i], t->pos[i + 1]));
        t->pos[i] = vec2_add(t->pos[i + 1],
                             vec2_scale(dir, t->link_len[i]));
        if (i > 0) apply_joint_constraint(t, i);
    }
}

/* fabrik_forward_pass — pin root to anchor, walk tip-ward restoring
 * link lengths. */
static void fabrik_forward_pass(Tentacle *t, Vec2 anchor)
{
    t->pos[0] = anchor;
    for (int i = 0; i < N_JOINTS - 1; i++) {
        Vec2 dir      = vec2_norm(vec2_sub(t->pos[i + 1], t->pos[i]));
        t->pos[i + 1] = vec2_add(t->pos[i],
                                 vec2_scale(dir, t->link_len[i]));
    }
}

/*
 * fabrik_solve — orchestrator. Reachability check first; if reachable,
 * iterate backward + forward passes until the tip is within CONV_TOL
 * of target or MAX_ITER iterations have run.
 *
 * Returns the iteration count actually used (for the HUD). Updates
 * t->at_limit.
 */
static int fabrik_solve(Tentacle *t, Vec2 target, Vec2 anchor)
{
    float total = total_link_length(t);
    t->at_limit = (vec2_dist(anchor, target) >= total);

    if (t->at_limit) {
        stretch_collinear(t, anchor, target);
        return 1;
    }

    int iter = 0;
    for (iter = 0; iter < MAX_ITER; iter++) {
        fabrik_backward_pass(t, target);
        fabrik_forward_pass (t, anchor);
        if (vec2_dist(t->pos[N_JOINTS - 1], target) < CONV_TOL) {
            iter++;
            break;
        }
    }
    return iter;
}

/* ── §5d  Lissajous target + smooth tracking ────────────────────────── */

/* trail_push — append actual_target to the ring buffer (overwrites
 * oldest when full). */
static void trail_push(Tentacle *t, Vec2 pos)
{
    t->trail_pts[t->trail_write] = pos;
    t->trail_write = (t->trail_write + 1) % TRAIL_POINTS;
    if (t->trail_fill < TRAIL_POINTS) t->trail_fill++;
}

/*
 * lissajous_at — raw Lissajous position at given scene_time.
 * Pure helper: given anchor + screen size + time, returns target.
 */
static Vec2 lissajous_at(Vec2 anchor, int cols, int rows, float t)
{
    float ax = (float)(cols * CELL_W) * LIS_AX_FACTOR;
    float ay = (float)(rows * CELL_H) * LIS_AY_FACTOR;
    return (Vec2){
        anchor.x + ax * cosf(LIS_OMEGA_X * t),
        anchor.y + ay * sinf(LIS_OMEGA_Y * t + LIS_PHASE_Y),
    };
}

/*
 * update_target — advance scene_time, lerp actual_target toward the
 * raw Lissajous, push into trail buffer.
 *
 * The exponential lerp `actual += (raw − actual) · rate` is a first-
 * order IIR low-pass: sudden velocity changes in the raw target are
 * absorbed into smooth curves the FABRIK solver can track without
 * the joint clamp firing aggressively.
 */
static void update_target(Tentacle *t, float dt, int cols, int rows)
{
    t->scene_time += dt * t->speed_scale;

    Vec2  raw  = lissajous_at(t->anchor, cols, rows, t->scene_time);
    float rate = clampf(dt * TARGET_SMOOTH, 0.0f, 1.0f);
    t->actual_target = vec2_lerp(t->actual_target, raw, rate);

    trail_push(t, t->actual_target);
}

/* ── §5e  trail ring buffer ────────────────────────────────────────── */

/* trail_get — retrieve a trail point k steps back from newest.
 * +TRAIL_POINTS before % avoids C's negative-modulo trap. */
static inline Vec2 trail_get(const Tentacle *t, int k)
{
    int idx = (t->trail_write - 1 - k + TRAIL_POINTS) % TRAIL_POINTS;
    return t->trail_pts[idx];
}

/* ── §5f  rendering helpers ────────────────────────────────────────── */

/* joint_pair — body gradient pair for joint/link i.
 *   i = 0          → pair 1 (root, deepest)
 *   i = N_JOINTS-1 → pair N_PAIRS (tip, brightest)  */
static int joint_pair(int i)
{
    int p = 1 + (i * (N_PAIRS - 1)) / (N_JOINTS - 1);
    if (p < 1)       p = 1;
    if (p > N_PAIRS) p = N_PAIRS;
    return p;
}

/* joint_attr — depth-cued attribute for joint/link i.
 *   root third : A_DIM    (heavy, anchored)
 *   middle     : A_NORMAL
 *   tip third  : A_BOLD   (glowing, active)         */
static attr_t joint_attr(int i)
{
    if (i <= 2)              return A_DIM;
    if (i >= N_JOINTS - 4)   return A_BOLD;
    return A_NORMAL;
}

/* joint_marker — bead marker character by joint index.
 *   '0' root third, 'o' middle, '.' tip third — taper reinforces
 *   the visual hierarchy from anchor to seeking tip. */
static chtype joint_marker(int i)
{
    if (i <= (N_JOINTS - 1) / 3)        return (chtype)(unsigned char)'0';
    if (i >= (N_JOINTS - 1) * 2 / 3)    return (chtype)(unsigned char)'.';
    return                              (chtype)(unsigned char)'o';
}

/*
 * draw_link_beads — bead-fill 'o' along link a→b at DRAW_STEP_PX
 * intervals (pass 1 of the two-pass body renderer).
 *
 * Dedup cursor is local — each link's stamping is independent.
 */
static void draw_link_beads(WINDOW *w, Vec2 a, Vec2 b,
                            int pair, attr_t attr, int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/* mark_cell — stamp one glyph at a pixel position with bounds check.
 * Pure helper used for trail dots, joint markers, target, anchor. */
static void mark_cell(WINDOW *w, Vec2 p, chtype glyph,
                      int pair, attr_t attr, int cols, int rows)
{
    int cx = px_to_cell_x(p.x);
    int cy = px_to_cell_y(p.y);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, glyph);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* ── §5g  render_tentacle ──────────────────────────────────────────── */

/* draw_trail — pass A: bright dotted ghost trail of the Lissajous path.
 * Every 3rd stored point so the trail reads as dashed dots, not a line.
 * Drawn in pair 7 (the brightest gradient slot — same colour family as
 * the tentacle's tip) at A_NORMAL — vivid enough to read as a glowing
 * breadcrumb path, not just a faint hint. */
static void draw_trail(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    for (int k = 0; k < t->trail_fill; k += 3) {
        mark_cell(w, trail_get(t, k), (chtype)(unsigned char)'.',
                  7, A_NORMAL, cols, rows);
    }
}

/* draw_link_fill — pass B: bead fill along each link with the gradient
 * pair and depth-cued attribute. Drawn root → tip so tip overwrites
 * root where the chain folds back on itself. */
static void draw_link_fill(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LINKS; i++) {
        draw_link_beads(w, t->pos[i], t->pos[i + 1],
                        joint_pair(i), joint_attr(i), cols, rows);
    }
}

/* draw_link_nodes — pass C: bold bead marker at each joint position,
 * stamped on top of the fill. Marker shape encodes joint-along-chain. */
static void draw_link_nodes(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_JOINTS; i++) {
        mark_cell(w, t->pos[i], joint_marker(i),
                  joint_pair(i), joint_attr(i), cols, rows);
    }
}

/* draw_target_marker — pass D: '*' converged or '#' at-limit, in
 * bright red bold. Pair 6 reused for the converged marker; pair 2 for
 * out-of-reach to give it a different colour. */
static void draw_target_marker(const Tentacle *t, WINDOW *w,
                               int cols, int rows)
{
    if (t->at_limit) {
        mark_cell(w, t->actual_target, (chtype)(unsigned char)'#',
                  2, A_BOLD, cols, rows);
    } else {
        mark_cell(w, t->actual_target, (chtype)(unsigned char)'*',
                  6, A_BOLD, cols, rows);
    }
}

/* draw_anchor_marker — pass E: chunky '0' at the fixed root anchor.
 * Drawn last so nothing overwrites it. */
static void draw_anchor_marker(const Tentacle *t, WINDOW *w,
                               int cols, int rows)
{
    mark_cell(w, t->anchor, (chtype)(unsigned char)'0',
              1, A_BOLD, cols, rows);
}

/*
 * render_tentacle — orchestrator. Painter's order so each pass overlays
 * the previous: trail (deepest, dim) → link fill → joint markers →
 * target marker → anchor marker (topmost).
 */
static void render_tentacle(const Tentacle *t, WINDOW *w, int cols, int rows)
{
    draw_trail        (t, w, cols, rows);
    draw_link_fill    (t, w, cols, rows);
    draw_link_nodes   (t, w, cols, rows);
    draw_target_marker(t, w, cols, rows);
    draw_anchor_marker(t, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — thin wrapper around Tentacle                              */
/* ===================================================================== */

typedef struct { Tentacle tentacle; } Scene;

/*
 * scene_init — place tentacle anchored at screen centre with all joints
 * laid out in a horizontal line. Pre-seed the trail with the initial
 * actual_target so the first frame already has a visible trail. Run
 * one FABRIK solve before the first render so HUD values are valid.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Tentacle *t = &sc->tentacle;

    t->anchor.x    = (float)(cols * CELL_W) * 0.50f;
    t->anchor.y    = (float)(rows * CELL_H) * 0.50f;
    t->speed_scale = SPEED_DEFAULT;
    t->scene_time  = 0.0f;
    t->paused      = false;
    t->theme_idx   = 0;

    /* Tapered link lengths, root longest. */
    for (int i = 0; i < N_LINKS; i++) {
        t->link_len[i] = BASE_LINK_LEN - (float)i * TAPER;
        if (t->link_len[i] < 4.0f) t->link_len[i] = 4.0f;
    }

    /* Initial pose: straight line right from anchor. */
    float x = t->anchor.x;
    t->pos[0] = t->anchor;
    for (int i = 0; i < N_LINKS; i++) {
        x += t->link_len[i];
        t->pos[i + 1] = (Vec2){ x, t->anchor.y };
    }

    /* Initial target = Lissajous at scene_time = 0 (max +x displacement). */
    t->actual_target = lissajous_at(t->anchor, cols, rows, 0.0f);

    /* Pre-seed the trail buffer so the first frame is fully populated. */
    for (int k = 0; k < TRAIL_POINTS; k++) t->trail_pts[k] = t->actual_target;
    t->trail_fill  = TRAIL_POINTS;
    t->trail_write = 0;

    /* One solve so HUD shows valid iteration count and at_limit flag. */
    t->last_iter = fabrik_solve(t, t->actual_target, t->anchor);
}

/*
 * scene_tick — one variable-timestep simulation step. dt is wall-clock
 * delta scaled by the caller's time_scale.
 *
 * Order: target update first (so FABRIK sees the freshly smoothed
 * target), then FABRIK solve.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Tentacle *t = &sc->tentacle;
    if (t->paused) return;

    update_target(t, dt, cols, rows);
    t->last_iter = fabrik_solve(t, t->actual_target, t->anchor);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    render_tentacle(&sc->tentacle, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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
    color_init(0);
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
 * screen_draw — compose one full frame:
 *   erase → tentacle → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any theme.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Tentacle *t = &sc->tentacle;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-FABRIK  N=%d  iter:%2d  %s  spd:%.2fx  theme:%s  %.2ftime  %.1ffps  %s ",
             N_LINKS, t->last_iter,
             t->at_limit ? "at-limit " : "converged",
             t->speed_scale,
             THEMES[t->theme_idx].name,
             time_scale, fps,
             t->paused ? "PAUSED" : "seeking");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:speed  t/T:theme  [/]:time ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    float                 time_scale;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/* atexit safety net — endwin() called on every exit path. */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize — handle a pending SIGWINCH. Re-centres the anchor to
 * the new screen dimensions; the chain follows on the next solve.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Tentacle *t = &app->scene.tentacle;
    t->anchor.x = (float)(app->screen.cols * CELL_W) * 0.50f;
    t->anchor.y = (float)(app->screen.rows * CELL_H) * 0.50f;
    t->pos[0]   = t->anchor;
    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Letter aliases (w/s for +/-) for ergonomics.
 */
static bool app_handle_key(App *app, int ch)
{
    Tentacle *t = &app->scene.tentacle;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': t->paused = !t->paused; break;

    case 'w': case '=': case '+':
        t->speed_scale *= 1.25f;
        if (t->speed_scale > SPEED_MAX) t->speed_scale = SPEED_MAX;
        break;
    case 's': case '-':
        t->speed_scale /= 1.25f;
        if (t->speed_scale < SPEED_MIN) t->speed_scale = SPEED_MIN;
        break;

    case 't':
        t->theme_idx = (t->theme_idx + 1) % N_THEMES;
        theme_apply(t->theme_idx);
        break;
    case 'T':
        t->theme_idx = (t->theme_idx - 1 + N_THEMES) % N_THEMES;
        theme_apply(t->theme_idx);
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
 *   ④ TICK       one simulation step at exactly dt · time_scale
 *   ⑤ FPS        smoothed over a 500 ms window
 *   ⑥ RENDER     erase → draw → wnoutrefresh → doupdate
 *   ⑦ FRAME CAP  sleep so total frame ≈ 1/TARGET_FPS
 */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFFU));
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
        scene_tick(&app->scene, dt * app->time_scale,
                   app->screen.cols, app->screen.rows);

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
