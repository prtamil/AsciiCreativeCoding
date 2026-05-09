/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_arm_reach.c — FABRIK robotic arm tracking a Lissajous figure-8
 *
 * DEMO: A 4-link robotic arm anchored at screen centre tracks a target
 *       that traces a Lissajous figure-8 (∞) path autonomously. The
 *       FABRIK solver iteratively bends the chain to follow; when the
 *       target wanders beyond the arm's reach, the chain stretches
 *       straight at it and a yellow horizon circle appears showing
 *       exactly how far the arm can extend.
 *
 * Study alongside: snake_forward_kinematics.c (FK contrast)
 *                  hexpod_tripod.c            (analytical 2-joint IK contrast)
 *
 * Section map:
 *   §1  config       — all tunables in one place
 *   §2  clock        — monotonic clock + sleep (verbatim from framework)
 *   §3  color        — 10 themes + spec HUD/hint pairs
 *   §4  coords       — pixel↔cell aspect-ratio helpers
 *   §5  entity       — Arm = FABRIK chain + Lissajous target
 *       §5a  vec2 helpers
 *       §5b  FABRIK solver (forward / backward / orchestrator)
 *       §5c  target motion (Lissajous + trail)
 *       §5d  rendering helpers
 *       §5e  render_arm orchestrator
 *   §6  scene        — thin Scene wrapper
 *   §7  screen       — ncurses double-buffer display layer
 *   §8  app          — signals, resize, main game loop
 *
 * Keys:  q / ESC      quit              space   pause / resume
 *        + / -        target speed × / ÷ 1.25
 *        t            cycle theme       [ / ]   time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_arm_reach.c \
 *       -o ik_arm_reach -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : FABRIK iterative inverse kinematics. Two geometric
 *                 passes per iteration:
 *                   FORWARD  — snap tip to target; walk root-ward
 *                              re-stretching each segment to its fixed
 *                              link length. After this pass the tip
 *                              exactly equals target but the root has
 *                              drifted away from its anchor.
 *                   BACKWARD — snap root back to anchor; walk tip-ward
 *                              re-stretching each segment. After this
 *                              pass the root is fixed and every link
 *                              is correct, but the tip has drifted —
 *                              by less than before, provably.
 *                 Each iteration strictly reduces the tip-to-target
 *                 error; convergence in 3–5 iterations for a typical
 *                 4-link chain. MAX_ITER = 15 caps degenerate-config
 *                 cost. No matrix inverse, no Jacobian, no singularities
 *                 — only positions and elementary trig.
 *
 *                 A reachability check fires once per tick: if the
 *                 target is beyond the chain's total length, no IK
 *                 solution exists. Skip iteration, stretch the arm
 *                 straight at the target, raise at_limit so the
 *                 reach-horizon circle is drawn.
 *
 *                 The autonomous target traces a Lissajous curve
 *                 x(t) = Ax · cos(t), y(t) = Ay · sin(2t + π/4). The
 *                 1:2 frequency ratio produces exactly one self-
 *                 intersection — the classic figure-8 ∞ shape. The
 *                 π/4 phase shift makes the crossing a clean X
 *                 rather than a tangent cusp.
 *
 * Data-structure: Arm holds joint positions pos[N_JOINTS=5] (4 links),
 *                 link_len[N_LINKS=4] tapered root→tip, current target
 *                 with a TRAIL_SIZE=60 ring-buffer history, Lissajous
 *                 parameters (root_px/py, lis_ax/ay, scene_time,
 *                 speed_scale), and the at_limit flag. No prev/cur
 *                 snapshots and no alpha-lerp scaffolding — variable
 *                 timestep makes them unnecessary.
 *
 * Rendering     : Painter's order — faint dotted Lissajous trail behind
 *                 → yellow reach-horizon circle (only at_limit) →
 *                 arm link bead-fill (root-dark to tip-bright gradient)
 *                 → joint node markers (size-coded '0' '0' 'o' 'o' '.')
 *                 → bright red target marker ('+' tracking, 'X' out
 *                 of reach). 48-sample sparse-dot circle approximates
 *                 the reach envelope without overlapping cells.
 *
 * Performance   : Variable timestep at render rate. Per frame:
 *                 max 15 FABRIK iterations × 4 link traversals × 2
 *                 passes ≈ 120 simple ops; typically converges in 3–5
 *                 iterations so ~30 ops. Microseconds total.
 *
 * References    :
 *   Aristidou & Lasenby, "FABRIK: A fast, iterative solver for the
 *     Inverse Kinematics problem" (Graphical Models 2011) — the
 *     foundational paper, including the convergence proof.
 *     https://www.andreasaristidou.com/FABRIK.html
 *   Wikipedia, "Inverse kinematics" — broader context including the
 *     Jacobian alternatives that FABRIK avoids.
 *   Wikipedia, "Lissajous curve" — derivation of the figure-8 shape
 *     for fx:fy = 1:2 frequency ratio with phase offset.
 *   Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com) — the
 *     case for fixed-step (stiff sims); we don't qualify, hence
 *     variable dt.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two clean halves: a TARGET that drives itself along a closed-form
 * figure-8 path, and an ARM that does whatever it has to do to put its
 * tip on that target. The arm has no will of its own — every frame it
 * solves "given root anchor, given target, find joint positions that
 * make all segments stay link_len apart" with the FABRIK iterative
 * geometric trick. When the target escapes the arm's reach sphere, the
 * "trick" is simpler: stretch straight at it and accept defeat (the
 * tip can't get there).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Target  : a planet on a fixed elliptical orbit. It does not negotiate.
 *
 * Arm     : a multi-jointed pointer chasing a laser dot. With more
 *           joints, the pointer can curl around obstacles and trace
 *           non-straight paths — the wider freedom of multi-link IK
 *           over a plain 2-bar linkage.
 *
 * FABRIK  : an iterative coin-flip between two constraints. Cycle 1:
 *           "tip must reach target" — done, by snapping it. Cycle 2:
 *           "root must stay anchored" — done, by snapping it. Each
 *           cycle violates the OTHER constraint by a smaller amount
 *           than before, so 3–5 cycles converges to satisfying both
 *           within sub-pixel tolerance.
 *
 * Reach   : the disc of radius Σ link_len centred on the root. Outside
 *           the disc, no joint configuration can place the tip there —
 *           geometric impossibility. The arm just points at the target
 *           and the dashed yellow circle visualises that boundary.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. Update target:
 *       a. scene_time += dt · speed_scale
 *       b. target.x = root.x + Ax · cos(LIS_FX · scene_time)
 *          target.y = root.y + Ay · sin(LIS_FY · scene_time + LIS_PHASE)
 *       c. push target into the trail ring buffer
 *  3. Reachability: if |target − root| > Σ link_len, stretch the arm
 *     straight at the target, set at_limit = true, return.
 *  4. Else FABRIK loop (up to MAX_ITER iterations):
 *       a. Forward pass: pos[N−1] = target. For i = N−2 down to 0,
 *          slide pos[i] toward pos[i+1] until they're link_len[i] apart.
 *       b. Backward pass: pos[0] = root. For i = 0 up to N−2, slide
 *          pos[i+1] away from pos[i] until they're link_len[i] apart.
 *       c. If |pos[N−1] − target| < CONV_TOL: converged, break.
 *  5. Render painter's order: trail → reach circle (if at_limit) →
 *     link beads → joint markers → target marker.
 *
 * KEY FORMULAS
 * ────────────
 *  Lissajous     : x = root.x + Ax · cos(LIS_FX · t)
 *                  y = root.y + Ay · sin(LIS_FY · t + LIS_PHASE)
 *                  fx:fy = 1:2 → one self-crossing → figure-8 shape.
 *
 *  Forward pass  : pos[N−1] = target
 *                  for i = N−2 .. 0:
 *                    f      = pos[i] − pos[i+1]
 *                    r      = link_len[i] / |f|
 *                    pos[i] = pos[i+1] + f · r
 *
 *  Backward pass : pos[0] = root
 *                  for i = 0 .. N−2:
 *                    b        = pos[i+1] − pos[i]
 *                    r        = link_len[i] / |b|
 *                    pos[i+1] = pos[i] + b · r
 *
 *  Convergence   : break when |pos[N−1] − target| < CONV_TOL (≈ 1.5 px)
 *                  — sub-cell accuracy, further iterations invisible.
 *
 *  Reach circle  : 48 dots at angle k · 2π/48, at distance Σ link_len
 *                  from root. Sparse enough to read as dashed; dense
 *                  enough that the eye reads it as circular.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - Coincident joints. If two joints land at the exact same pixel,
 *    |f| or |b| → 0 and division blows up. Guarded with a 1e−6 floor;
 *    the solver self-corrects on the next iteration as one joint is
 *    flung far apart and pulled back into normal range.
 *
 *  - Target out of reach. Skip iteration entirely and stretch the arm
 *    straight. Without the early-out, FABRIK would oscillate without
 *    converging (the tip can never reach the target).
 *
 *  - Tip already at target. CONV_TOL early-out lets the solver finish
 *    in one iteration when the target hasn't moved; otherwise we'd
 *    waste 14 more iterations doing nothing visible.
 *
 *  - Resize. scene_init recomputes root anchor, link lengths, and
 *    Lissajous amplitudes for the new screen. Saved scene_time,
 *    speed_scale, and theme_idx are preserved so animation continues.
 *
 *  - Suspend / lid-close. dt clamped to 100 ms in main() so the arm
 *    doesn't catastrophically catch up after a long pause.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: target traces ∞ in ~9 s; arm tracks smoothly. The
 *    reach-horizon circle blinks on at the horizontal extremes of the
 *    figure-8 (because Ax is clipped to 90% of total reach, the lobes
 *    just barely escape).
 *
 *  - Pause: scene_tick returns; arm freezes at current pose.
 *
 *  - Crank `+` repeatedly: target speeds up; the tip increasingly lags
 *    behind during fast motion — the FABRIK solver only does 15 iters
 *    per tick.
 *
 *  - Press `[` to slow time: motion stays smooth at any time scale.
 *
 *  - Cycle themes: arm gradient changes; HUD stays bright yellow on
 *    default bg regardless of theme.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read ik_helloworld.c first if 2-link IK isn't
 *      automatic. This file handles a 4-link arm where the
 *      closed-form approach STOPS WORKING (ik_helloworld T6) —
 *      so we switch to FABRIK iteration.
 *   2. §5 entity — THE HEART of this file. In sub-section order:
 *        §5b FABRIK solver           ← read AFTER tutorials T1-T5
 *           - fabrik_forward
 *           - fabrik_backward
 *           - fabrik_solve (orchestrator)
 *        §5c target Lissajous + trail
 *        §5d-§5e renderers
 *      The two passes are tiny (≤15 lines each); the algorithm's
 *      brilliance is in their COMPOSITION.
 *   3. §6 scene — thin wrapper.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if you've seen the
 *      framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pos[i]              joint i position (Vec2). 0 = root, N_JOINTS-1
 *                       = tip.
 *   link_len[i]         constant length of link between pos[i] and
 *                       pos[i+1]. Tapered root-to-tip.
 *   target              where the user (here: the Lissajous) wants
 *                       the tip.
 *   anchor              root pos[0] origin — this NEVER MOVES.
 *   at_limit            bool — true iff target is outside the reach
 *                       sphere; renderer draws the dashed circle.
 *   CONV_TOL            ~1.5 px — once tip is this close to target,
 *                       declare convergence.
 *   MAX_ITER            iteration cap (15) — degenerate-config bound.
 *
 * Background you need
 * ───────────────────
 *   - 2-link analytical IK (ik_helloworld T3-T5). FABRIK is the
 *     successor for chains where closed-form fails.
 *   - Vector subtract / normalise / scale.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Jacobian / pseudoinverse IK. FABRIK avoids matrix algebra
 *     entirely; positions in, positions out.
 *   - Cyclic Coordinate Descent (CCD). FABRIK is its successor —
 *     similar idea but converges faster.
 *   - Joint limits, ball joints, twist constraints. We have a
 *     plain planar chain; angle limits are mentioned in T7 only.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Seven tutorials that build FABRIK from first principles.
 *
 *   T1  Why iteration past 2 links — closed-form's limit
 *   T2  FABRIK in one sentence — alternate two snaps
 *   T3  The forward pass — chase the tip down to the root
 *   T4  The backward pass — anchor the root, push back to the tip
 *   T5  Why this CONVERGES — error bounded by the convex hull
 *   T6  Reachability — the disc, the horizon circle, the early-out
 *   T7  Joint limits, multi-effector, smoothing — what FABRIK adds
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  WHY ITERATION PAST 2 LINKS — CLOSED-FORM'S LIMIT
 * ────────────────────────────────────────────────────
 * 2-link IK has a closed-form solution (law of cosines —
 * ik_helloworld T3-T5). 3-link IK is UNDERDETERMINED; the
 * 2+1 hybrid (fk_ik_helloworld T4) picks one specific
 * configuration by adding a constraint.
 *
 * What about 4 links? 5? 30 (a tentacle)? The CONFIG space is
 * a manifold of dimension N − 2 (target gives 2 equations to
 * pin down N angles). Tabulating all of those by hand is
 * intractable.
 *
 * Iterative IK side-steps the algebraic blowup: instead of
 * SOLVING for joint angles, ADJUST the chain repeatedly until
 * it satisfies the constraints. Two flavours dominate:
 *
 *   JACOBIAN  Linearise around the current pose, take a step
 *             toward the target. Used in robotics.
 *
 *   FABRIK    Geometric only. Two passes. No matrix math.
 *             Used in animation and games.
 *
 * FABRIK is what this file implements. It scales to ANY chain
 * length, converges in 3-5 iterations for typical configurations,
 * and uses only positions + Euclidean lengths.
 *
 * T2  FABRIK IN ONE SENTENCE — ALTERNATE TWO SNAPS
 * ────────────────────────────────────────────────
 * "Forward And Backward Reaching Inverse Kinematics."
 *
 * Each iteration is two passes:
 *
 *   FORWARD   pretend the TIP is at the target. Walk root-ward
 *             along the chain, re-stretching each segment to
 *             its link_len. The tip ends exactly at target;
 *             the root has DRIFTED.
 *
 *   BACKWARD  pretend the ROOT is at its anchor. Walk tip-ward
 *             along the chain, re-stretching each segment to
 *             its link_len. The root ends exactly at anchor;
 *             the tip has drifted.
 *
 * Iterate. The tip-drift after each backward pass is provably
 * smaller than after the previous backward pass. After ~5 cycles
 * the tip lands within sub-pixel tolerance.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │  before:    @──●──●──●──●  current pose          │
 *      │             (root)        (tip)                  │
 *      │                                          *  target│
 *      │                                                  │
 *      │  forward:        ●──●──●──●──*                   │
 *      │             ╲ ↑ root drifted               (tip)  │
 *      │                                                  │
 *      │  backward:  @──●──●──●──●                        │
 *      │             (root anchored)  ↑ tip drifted (less) │
 *      │                                                  │
 *      │  iterate ~5 times → both ends correct            │
 *      └──────────────────────────────────────────────────┘
 *
 * That is FABRIK. The rest is implementation.
 *
 * T3  THE FORWARD PASS — CHASE THE TIP DOWN TO THE ROOT
 * ─────────────────────────────────────────────────────
 * The forward pass walks the chain from tip to root.
 *
 *     pos[N-1] = target           ← snap the tip to target
 *     for i = N-2 down to 0:
 *       direction = pos[i] − pos[i+1]      ← vector from i+1 to i
 *       d = |direction|
 *       pos[i] = pos[i+1] + (direction / d) · link_len[i]
 *
 * "Slide pos[i] along the line from pos[i+1] until they're
 * link_len[i] apart." The new pos[i] sits in the SAME DIRECTION
 * from pos[i+1] as before, but at the correct distance.
 *
 * Why does this preserve link length? Because we explicitly
 * scaled the direction vector to length link_len[i] and
 * placed pos[i] at that distance from pos[i+1]. By
 * construction.
 *
 * After the loop, every segment has its correct length and
 * pos[N-1] is on the target. Only pos[0] is wrong — the root
 * has drifted from its anchor.
 *
 * T4  THE BACKWARD PASS — ANCHOR THE ROOT, PUSH BACK TO THE TIP
 * ─────────────────────────────────────────────────────────────
 * Same pattern, opposite direction:
 *
 *     pos[0] = anchor               ← snap the root to anchor
 *     for i = 0 up to N-2:
 *       direction = pos[i+1] − pos[i]      ← vector from i to i+1
 *       d = |direction|
 *       pos[i+1] = pos[i] + (direction / d) · link_len[i]
 *
 * "Slide pos[i+1] along the line from pos[i] until they're
 * link_len[i] apart." After the loop, every segment has its
 * correct length and pos[0] is at anchor — but pos[N-1] has
 * drifted from target.
 *
 * Forward fixed the tip and broke the root.
 * Backward fixed the root and broke the tip.
 *
 * Both passes preserve segment lengths exactly — only the
 * endpoint constraints get violated, alternately. The
 * MAGNITUDE of violation shrinks each iteration.
 *
 * T5  WHY THIS CONVERGES — ERROR BOUNDED BY THE CONVEX HULL
 * ─────────────────────────────────────────────────────────
 * Why doesn't FABRIK oscillate forever between two bad poses?
 *
 * Aristidou & Lasenby's proof: the BACKWARD pass moves pos[i]
 * toward where pos[i] should ideally sit. The chain's tip-end
 * error after backward pass is BOUNDED ABOVE by the chain's
 * tip-end error after forward pass scaled by a contraction
 * factor < 1. Each FULL iteration (forward + backward) shrinks
 * the error by a factor that depends on the geometry but is
 * always < 1 for non-degenerate configs.
 *
 * Convergence is GEOMETRIC: typical 4-link arm reaches sub-
 * pixel error in 3-5 iterations. Our MAX_ITER = 15 is just a
 * safety cap — for adversarial geometries (chain folded
 * tightly) it might need a few more.
 *
 * Failure modes:
 *   - Coincident joints (|direction| → 0). Guard with a 1e-6
 *     floor; the next iteration flings the joint apart again.
 *   - Target unreachable (T6). Forward pass never converges
 *     because the tip cannot get to the target. Detect early
 *     and stretch the arm straight.
 *
 * T6  REACHABILITY — THE DISC, THE HORIZON CIRCLE, THE EARLY-OUT
 * ──────────────────────────────────────────────────────────────
 * The arm can reach any point at distance ≤ Σ link_len from
 * the anchor — a DISC. Outside, no IK solution exists.
 *
 * If we pretend it does and start FABRIK iterating, the tip
 * keeps trying to reach the target and the root keeps trying
 * to stay anchored — the algorithm oscillates without
 * converging. MAX_ITER fires; the result is a fully extended
 * arm in some semi-random direction.
 *
 * Better: detect the unreachable case BEFORE iterating:
 *
 *     d = |target − anchor|
 *     R = Σ link_len
 *     if d > R:
 *       at_limit = true
 *       stretch chain straight at target
 *       return
 *
 * "Stretch the chain straight" means each pos[i] sits at
 * distance Σ_{j<i} link_len[j] from anchor along the
 * anchor → target direction. The arm visibly POINTS at the
 * target it can't reach.
 *
 * The reach circle (yellow dashed) is drawn ONLY when at_limit;
 * its appearance teaches the user that the arm has hit a real
 * geometric limit, not a buggy solver.
 *
 * T7  JOINT LIMITS, MULTI-EFFECTOR, SMOOTHING — WHAT FABRIK ADDS
 * ──────────────────────────────────────────────────────────────
 * The basic FABRIK loop in this file is the SIMPLEST form. The
 * algorithm scales gracefully:
 *
 *   JOINT LIMITS  After each pass, clamp every joint's angle
 *                 (relative to its parent) to a min/max range.
 *                 The next pass may violate other constraints
 *                 but on average the chain settles into a
 *                 limit-respecting pose.
 *
 *   MULTI-EFFECTOR  Multiple targets — solve each in turn,
 *                 average the resulting positions per joint.
 *                 Used for character rigs where two hands need
 *                 to grab two objects simultaneously.
 *
 *   BRANCHING     The chain forks into a tree (a torso with two
 *                 arms). At the branching joint, the forward
 *                 passes from each branch coexist via a centroid
 *                 average.
 *
 *   SMOOTHING     Damp the tip's motion toward target by
 *                 weight ∈ [0, 1]; the chain follows the target
 *                 elastically rather than instantly.
 *
 * For our 4-link Lissajous-tracking arm, none of these are
 * needed. The basic loop with reachability early-out is enough.
 * ik_tentacle_seek.c uses the same basic loop on a 16-link chain;
 * snake_inverse_kinematics.c on a much longer chain. All three
 * files share the §5b FABRIK code structurally.
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
    HUD_COLS      = 96,
    FPS_UPDATE_MS = 500,

    /* ncurses pair IDs.
     *   1..N_ARM_COLORS  arm gradient (root dark → tip bright), themed
     *   6                target marker — fixed bright red (semantic)
     *   7                reach horizon — fixed yellow    (semantic)
     *   8 PAIR_HUD       status bar — bright yellow on default bg
     *   9 PAIR_HINT      key hint   — bright cyan  on default bg */
    N_ARM_COLORS = 5,
    PAIR_HUD     = 8,
    PAIR_HINT    = 9,

    N_THEMES     = 10,    /* cycled with `t` */

    /* Joint chain sizing. N_JOINTS = N_LINKS + 1.
     *   pos[0]      root anchor (never moved by solver)
     *   pos[N-1]    end effector / tip (must reach target)
     *   pos[1..N-2] interior joints, freely moved by FABRIK */
    N_JOINTS     = 5,
    N_LINKS      = 4,

    /* MAX_ITER: FABRIK convergence cap. Typical 4-link chains converge
     * in 3-5 iterations; 15 is a safety ceiling for degenerate configs.
     * Cost: 15 · 5 · 2 = 150 simple ops — negligible per tick. */
    MAX_ITER     = 15,

    /* Trail buffer capacity. At ~60 ticks/s this is roughly 1 second of
     * target history — about 1/9 of the figure-8 cycle, enough to read
     * the curve shape without clutter. */
    TRAIL_SIZE   = 60,

    /* Sparse reach-circle samples. 48 dots ≈ 7.5° spacing — visually
     * dashed without overlapping cells at typical arm sizes. */
    REACH_CIRCLE_SAMPLES = 48,
};

/*
 * CONV_TOL — FABRIK convergence tolerance (pixel space). 1.5 px is
 * sub-cell on both axes (CELL_W=8, CELL_H=16), so further iterations
 * produce no visible improvement.
 */
#define CONV_TOL    1.5f

/*
 * Lissajous figure-8 parameters.
 *
 *   x(t) = root.x + lis_ax · cos(LIS_FX · scene_time)
 *   y(t) = root.y + lis_ay · sin(LIS_FY · scene_time + LIS_PHASE)
 *
 * fx : fy = 1 : 2 → exactly one self-intersection → figure-8 shape.
 * φ = π/4         → clean X-crossing at the centre (no tangent cusp).
 * Amplitudes lis_ax/ay are computed at scene_init from terminal size
 * and clipped to ≤ 90% of total arm reach so the at_limit state
 * triggers naturally at the figure-8 extremes.
 */
#define LIS_FX               1.0f
#define LIS_FY               2.0f
#define LIS_PHASE            0.785f       /* ≈ π/4 */

/* Lissajous speed multiplier (user-tunable on +/-).
 *   default 0.7 → ~9 s per figure-8 cycle, leisurely.
 *   range [0.05, 5.0] → ~125 s to ~1.3 s per cycle. */
#define LIS_SPEED_DEFAULT    0.7f
#define LIS_SPEED_MIN        0.05f
#define LIS_SPEED_MAX        5.0f

/*
 * DRAW_STEP_PX — bead fill stride along each link. Must be < CELL_W (8)
 * so the dense stamping never skips a column. 5 px gives a slightly
 * sparse "beaded chain" texture so the joint markers (drawn in pass 2)
 * read clearly through the fill.
 */
#define DRAW_STEP_PX   5.0f

/* Time scale — user-controlled simulation speed multiplier on `[/]`. */
#define TIME_SCALE_DEFAULT  1.0f
#define TIME_SCALE_MIN      0.25f
#define TIME_SCALE_MAX      4.0f
#define TIME_SCALE_STEP     1.5f

/* Timing primitives — verbatim from framework.c. */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL

/* Terminal cell dimensions — the aspect-ratio bridge. */
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
 * Theme — body gradient (pairs 1..N_ARM_COLORS, root → tip).
 * HUD/HINT pairs are theme-independent (CLAUDE.md HUD spec) — they
 * stay readable against any animation behind them.
 *
 * All entries sit in the bright half of the 256-colour space:
 *   - cube colours: ≥ 24 (brightness rule)
 *   - grayscale  : ≥ 240 (the 232-239 zone vanishes under A_DIM)
 */
typedef struct {
    const char *name;
    int arm[N_ARM_COLORS];   /* pairs 1..5: root → tip */
} Theme;

static const Theme THEMES[N_THEMES] = {
    { "Steel",  {240, 244, 248, 252,  51} },
    { "Matrix", { 24,  28,  34,  40,  46} },
    { "Fire",   { 52,  88, 124, 160, 196} },
    { "Ocean",  { 24,  25,  27,  33,  51} },
    { "Nova",   { 54,  93, 129, 165, 201} },
    { "Toxic",  { 58,  64,  70,  76,  82} },
    { "Lava",   { 52,  94, 130, 166, 202} },
    { "Ghost",  {240, 244, 246, 250, 254} },
    { "Aurora", { 24,  35,  71, 107, 143} },
    { "Neon",   { 57,  93, 129, 165, 201} },
};

/* theme_apply — re-bind arm pairs (1..N_ARM_COLORS) to the chosen
 * theme. Pairs 6 (target red) and 7 (reach yellow) are NEVER
 * touched here — they carry semantic meaning that must not change
 * with the theme. HUD/HINT (8/9) are also theme-independent. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_ARM_COLORS; p++)
        init_pair(p + 1, t->arm[p], -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
        init_pair(6, 196, -1);   /* bright red — target marker (semantic) */
        init_pair(7, 226, -1);   /* yellow     — reach horizon (semantic) */
    } else {
        /* 8-color fallback */
        init_pair(1, COLOR_WHITE,  -1);
        init_pair(2, COLOR_WHITE,  -1);
        init_pair(3, COLOR_WHITE,  -1);
        init_pair(4, COLOR_WHITE,  -1);
        init_pair(5, COLOR_CYAN,   -1);
        init_pair(6, COLOR_RED,    -1);
        init_pair(7, COLOR_YELLOW, -1);
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
/* §5  entity — Arm                                                       */
/* ===================================================================== */

/* Vec2 — 2-D position vector in pixel space. */
typedef struct { float x, y; } Vec2;

/* ── §5a  vec2 helpers ──────────────────────────────────────────────── */

static inline float vec2_len(Vec2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}

/*
 * vec2_norm — unit vector. Degenerate guard: zero-length input returns
 * (1, 0) so callers can use the result without NaN propagation. The
 * specific direction (east) is arbitrary; any unit vector is OK because
 * the next FABRIK iteration immediately separates coincident joints.
 */
static inline Vec2 vec2_norm(Vec2 v)
{
    float l = vec2_len(v);
    if (l < 1e-6f) return (Vec2){1.0f, 0.0f};
    return (Vec2){v.x / l, v.y / l};
}

/* Arm — full IK arm state. */
typedef struct {
    /* joint chain */
    Vec2  pos[N_JOINTS];           /* current positions (pixels)         */
    float link_len[N_LINKS];       /* fixed lengths, set in scene_init   */
    float total_len;               /* Σ link_len — reachability test     */

    /* root anchor (Lissajous centre too) */
    float root_px, root_py;

    /* Lissajous target oscillator */
    Vec2  target;                  /* current Lissajous output (pixels)  */
    float scene_time;              /* phase accumulator (s)              */
    float speed_scale;              /* +/- multiplier on scene_time      */
    float lis_ax, lis_ay;           /* amplitudes (pixels)               */

    /* trail ring buffer — recent target positions */
    Vec2  trail[TRAIL_SIZE];
    int   trail_head;              /* newest entry index                 */
    int   trail_count;             /* valid entries, ≤ TRAIL_SIZE        */

    /* state flags */
    bool  at_limit;                /* target out of reach this tick      */
    bool  paused;
    int   theme_idx;
} Arm;

/* ── §5b  FABRIK solver ─────────────────────────────────────────────── */

/*
 * stretch_arm_straight — out-of-reach posture. Lay every joint along
 * the unit vector from root toward target. Cannot reach the target
 * but presents the arm at maximum extension in the right direction.
 */
static void stretch_arm_straight(Arm *a, Vec2 root, Vec2 target)
{
    Vec2 dir   = vec2_norm((Vec2){ target.x - root.x, target.y - root.y });
    a->pos[0]  = root;
    for (int i = 0; i < N_LINKS; i++) {
        a->pos[i + 1].x = a->pos[i].x + dir.x * a->link_len[i];
        a->pos[i + 1].y = a->pos[i].y + dir.y * a->link_len[i];
    }
}

/*
 * fabrik_forward_pass — pin tip to target, walk back to root preserving
 * link lengths. After this pass: tip equals target exactly; all link
 * lengths correct; root has drifted from anchor.
 */
static void fabrik_forward_pass(Arm *a, Vec2 target)
{
    a->pos[N_JOINTS - 1] = target;
    for (int i = N_JOINTS - 2; i >= 0; i--) {
        float fx   = a->pos[i].x - a->pos[i + 1].x;
        float fy   = a->pos[i].y - a->pos[i + 1].y;
        float flen = sqrtf(fx * fx + fy * fy);
        if (flen < 1e-6f) flen = 1e-6f;     /* coincident-joint guard */
        float r    = a->link_len[i] / flen;
        a->pos[i].x = a->pos[i + 1].x + fx * r;
        a->pos[i].y = a->pos[i + 1].y + fy * r;
    }
}

/*
 * fabrik_backward_pass — pin root to anchor, walk forward to tip
 * preserving link lengths. After this pass: root anchored; all links
 * correct; tip has drifted from target (by less than before).
 */
static void fabrik_backward_pass(Arm *a, Vec2 root)
{
    a->pos[0] = root;
    for (int i = 0; i < N_JOINTS - 1; i++) {
        float bx   = a->pos[i + 1].x - a->pos[i].x;
        float by   = a->pos[i + 1].y - a->pos[i].y;
        float blen = sqrtf(bx * bx + by * by);
        if (blen < 1e-6f) blen = 1e-6f;
        float r    = a->link_len[i] / blen;
        a->pos[i + 1].x = a->pos[i].x + bx * r;
        a->pos[i + 1].y = a->pos[i].y + by * r;
    }
}

/* tip_target_distance — |pos[N−1] − target|, used for convergence test. */
static float tip_target_distance(const Arm *a, Vec2 target)
{
    float tdx = a->pos[N_JOINTS - 1].x - target.x;
    float tdy = a->pos[N_JOINTS - 1].y - target.y;
    return sqrtf(tdx * tdx + tdy * tdy);
}

/*
 * fabrik_solve — orchestrator. Reachability check first; if reachable,
 * iterate forward + backward passes until tip-to-target error drops
 * below CONV_TOL or MAX_ITER is hit. See ALGORITHM IN STEPS §3-§4.
 */
static void fabrik_solve(Arm *a)
{
    Vec2 root   = { a->root_px, a->root_py };
    Vec2 target = a->target;

    Vec2  drt   = { target.x - root.x, target.y - root.y };
    float dist  = vec2_len(drt);

    if (dist > a->total_len) {
        a->at_limit = true;
        stretch_arm_straight(a, root, target);
        return;
    }

    a->at_limit = false;
    for (int iter = 0; iter < MAX_ITER; iter++) {
        fabrik_forward_pass (a, target);
        fabrik_backward_pass(a, root);
        if (tip_target_distance(a, target) < CONV_TOL) break;
    }
}

/* ── §5c  target motion — Lissajous figure-8 ───────────────────────── */

/* trail_push — append current target to ring buffer (overwrites oldest). */
static void trail_push(Arm *a, Vec2 pos)
{
    a->trail_head = (a->trail_head + 1) % TRAIL_SIZE;
    a->trail[a->trail_head] = pos;
    if (a->trail_count < TRAIL_SIZE) a->trail_count++;
}

/*
 * update_target — advance the Lissajous clock and recompute target
 * position. See KEY FORMULAS for the parametric equations.
 */
static void update_target(Arm *a, float dt)
{
    a->scene_time += dt * a->speed_scale;

    a->target.x = a->root_px
                + a->lis_ax * cosf(LIS_FX * a->scene_time);
    a->target.y = a->root_py
                + a->lis_ay * sinf(LIS_FY * a->scene_time + LIS_PHASE);

    trail_push(a, a->target);
}

/* ── §5d  rendering helpers ─────────────────────────────────────────── */

/*
 * draw_link_beads — stamp 'o' along segment a→b at DRAW_STEP_PX
 * intervals (pass 1 of the two-pass arm renderer).
 *
 * WHY DENSE STEPPING? A cell is CELL_W=8 px wide. Drawing only at
 * endpoints leaves visible gaps on segments longer than one cell.
 * Stepping every DRAW_STEP_PX=5 px (< CELL_W) guarantees at least
 * one sample per cell along the segment.
 *
 * The dedup cursor is local: each link's stamping is independent.
 */
static void draw_link_beads(WINDOW *w, Vec2 a, Vec2 b,
                            int pair, attr_t attr,
                            int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
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

/* mark_cell — stamp one glyph at a pixel position with bounds check. */
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

/*
 * joint_marker — size-coded glyph by joint index.
 *   root anchor (i=0)         → '0' big
 *   tip / end effector        → '.' small (precision)
 *   near-root interior        → '0' big
 *   near-tip interior         → 'o' medium
 * The taper from big to small visually emphasises the root→tip
 * hierarchy and makes the tip read as a precision gripper.
 */
static chtype joint_marker(int i)
{
    if (i == 0)              return (chtype)(unsigned char)'0';   /* root */
    if (i == N_JOINTS - 1)   return (chtype)(unsigned char)'.';   /* tip  */
    if (i <=     N_JOINTS / 2) return (chtype)(unsigned char)'0'; /* near root */
    return                          (chtype)(unsigned char)'o';   /* near tip  */
}

/*
 * joint_pair — colour pair by joint index. Maps to the link below
 * each joint, so each joint's marker matches its femur colour. Pair 4
 * is intentionally skipped (tip uses pair 5 directly) for visual
 * contrast between the bright tip and middle links.
 */
static int joint_pair(int i)
{
    if (i == 0)             return 1;   /* root */
    if (i == N_JOINTS - 1)  return 5;   /* tip  */
    return i + 1;                       /* interior — 2, 3 */
}

/* ── §5e  render_arm orchestrator ───────────────────────────────────── */

/* draw_trail — pass A: faint dotted Lissajous path, oldest → newest. */
static void draw_trail(const Arm *a, WINDOW *w, int cols, int rows)
{
    for (int k = 0; k < a->trail_count; k++) {
        int idx = (a->trail_head + TRAIL_SIZE - a->trail_count + 1 + k)
                  % TRAIL_SIZE;
        mark_cell(w, a->trail[idx], (chtype)(unsigned char)'.',
                  6, A_DIM, cols, rows);
    }
}

/*
 * draw_reach_circle — pass B (only when at_limit): sparse dot circle
 * showing the reach horizon. 48 angular samples — see MENTAL MODEL.
 */
static void draw_reach_circle(const Arm *a, WINDOW *w, int cols, int rows)
{
    if (!a->at_limit) return;

    Vec2  root = { a->root_px, a->root_py };
    float pi2  = 2.0f * (float)M_PI;

    for (int k = 0; k < REACH_CIRCLE_SAMPLES; k++) {
        float angle = (float)k * pi2 / (float)REACH_CIRCLE_SAMPLES;
        Vec2  p = { root.x + a->total_len * cosf(angle),
                    root.y + a->total_len * sinf(angle) };
        mark_cell(w, p, (chtype)(unsigned char)'.', 7, A_DIM, cols, rows);
    }
}

/*
 * draw_links — pass C: bead fill along each link with the gradient
 * pair. Pair 4 is intentionally skipped between mid-links and tip
 * (link_pairs[3] = 5) to widen the visual gap to the bright tip.
 */
static void draw_links(const Arm *a, WINDOW *w, int cols, int rows)
{
    static const int link_pairs[N_LINKS] = { 1, 2, 3, 5 };
    for (int i = 0; i < N_LINKS; i++) {
        attr_t attr = (i == 0 || i == N_LINKS - 1) ? A_BOLD : A_NORMAL;
        draw_link_beads(w, a->pos[i], a->pos[i + 1],
                        link_pairs[i], attr, cols, rows);
    }
}

/* draw_joints — pass D: bold node glyph at each joint, sitting on top
 * of the link bead fill. Marker shape encodes hierarchy via
 * joint_marker(); pair via joint_pair(). */
static void draw_joints(const Arm *a, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_JOINTS; i++)
        mark_cell(w, a->pos[i], joint_marker(i),
                  joint_pair(i), A_BOLD, cols, rows);
}

/* draw_target — pass E: '+' when tracking, 'X' when out of reach.
 * Bright red bold so it always stands out against any theme. */
static void draw_target(const Arm *a, WINDOW *w, int cols, int rows)
{
    chtype glyph = a->at_limit
                 ? (chtype)(unsigned char)'X'
                 : (chtype)(unsigned char)'+';
    mark_cell(w, a->target, glyph, 6, A_BOLD, cols, rows);
}

/*
 * render_arm — orchestrator. Painter's order so each pass overlays
 * the previous: trail (deepest) → reach circle → links → joints →
 * target marker (topmost).
 */
static void render_arm(const Arm *a, WINDOW *w, int cols, int rows)
{
    draw_trail        (a, w, cols, rows);
    draw_reach_circle (a, w, cols, rows);
    draw_links        (a, w, cols, rows);
    draw_joints       (a, w, cols, rows);
    draw_target       (a, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — thin wrapper around Arm                                   */
/* ===================================================================== */

typedef struct { Arm arm; } Scene;

/*
 * scene_init — place the arm at screen centre with all geometry sized
 * to the current terminal dimensions.
 *
 * Non-obvious bits:
 *  - link_len weights {0.32, 0.27, 0.23, 0.18} sum to 1.0 so total_len
 *    equals arm_reach exactly. Tapered root → tip mirrors animal-limb
 *    biomechanics (upper arm > forearm > hand) and gives visual hierarchy.
 *  - lis_ax / lis_ay clipped to 0.9 · total_len so the figure-8
 *    extremes are JUST out of reach — at_limit briefly fires at each
 *    horizontal lobe, making the reach-horizon circle blink visibly.
 *  - Initial pose: arm laid out straight to the right. FABRIK reshapes
 *    it on the first tick to track the initial Lissajous target.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Arm *a = &sc->arm;

    float sw = (float)(cols * CELL_W);
    float sh = (float)(rows * CELL_H);

    /* Link lengths: arm spans 60% of shorter screen dimension.
     * Weights sum to 1.0 so total_len = arm_reach exactly. */
    float arm_reach = (sw < sh ? sw : sh) * 0.60f;
    a->link_len[0]  = arm_reach * 0.32f;   /* longest — root link  */
    a->link_len[1]  = arm_reach * 0.27f;
    a->link_len[2]  = arm_reach * 0.23f;
    a->link_len[3]  = arm_reach * 0.18f;   /* shortest — tip link  */
    a->total_len    = arm_reach;

    a->root_px = sw * 0.50f;
    a->root_py = sh * 0.50f;

    /* Lissajous amplitudes: 40% of each axis, clipped to 90% of reach. */
    float max_amp = a->total_len * 0.90f;
    a->lis_ax     = sw * 0.40f;
    a->lis_ay     = sh * 0.40f;
    if (a->lis_ax > max_amp) a->lis_ax = max_amp;
    if (a->lis_ay > max_amp) a->lis_ay = max_amp;

    a->scene_time  = 0.0f;
    a->speed_scale = LIS_SPEED_DEFAULT;

    /* Initial pose: straight line to the right from root. */
    a->pos[0] = (Vec2){ a->root_px, a->root_py };
    for (int i = 1; i < N_JOINTS; i++) {
        a->pos[i].x = a->pos[i - 1].x + a->link_len[i - 1];
        a->pos[i].y = a->pos[i - 1].y;
    }

    /* Initial target at scene_time = 0. */
    a->target.x = a->root_px + a->lis_ax;
    a->target.y = a->root_py + a->lis_ay * sinf(LIS_PHASE);

    a->trail_head  = 0;
    a->trail_count = 0;
    a->at_limit    = false;
    a->paused      = false;
    a->theme_idx   = 0;
}

/*
 * scene_tick — one variable-timestep update. dt is the wall-clock
 * delta scaled by the caller's time_scale.
 */
static void scene_tick(Scene *sc, float dt)
{
    Arm *a = &sc->arm;
    if (a->paused) return;

    update_target(a, dt);    /* advance Lissajous, push trail */
    fabrik_solve (a);        /* IK: bring tip to new target   */
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    render_arm(&sc->arm, w, cols, rows);
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

/* SIGWINCH path: endwin()+refresh() forces ncurses to re-read LINES/COLS. */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — compose one full frame:
 *   erase → arm → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any theme.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Arm *a = &sc->arm;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-FABRIK  reach:%s  spd:%.2f  theme:%s  %.2fx  %.1ffps  %s ",
             a->at_limit ? "LIMIT" : "NEAR ",
             a->speed_scale,
             THEMES[a->theme_idx].name,
             time_scale, fps,
             a->paused ? "PAUSED" : "tracking");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  +/-:speed  t:theme  [/]:time ");
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
 * app_do_resize — handle a pending SIGWINCH.
 *
 * Geometry (root, link lengths, Lissajous amplitudes) is recomputed for
 * the new screen, but scene_time, speed_scale, theme_idx are PRESERVED
 * so animation continues from the same point in its cycle.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    int   saved_theme = app->scene.arm.theme_idx;
    float saved_speed = app->scene.arm.speed_scale;
    float saved_time  = app->scene.arm.scene_time;

    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    app->scene.arm.theme_idx   = saved_theme;
    app->scene.arm.speed_scale = saved_speed;
    app->scene.arm.scene_time  = saved_time;

    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Arm *a = &app->scene.arm;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': a->paused = !a->paused; break;

    case '+': case '=':
        a->speed_scale *= 1.25f;
        if (a->speed_scale > LIS_SPEED_MAX) a->speed_scale = LIS_SPEED_MAX;
        break;
    case '-':
        a->speed_scale /= 1.25f;
        if (a->speed_scale < LIS_SPEED_MIN) a->speed_scale = LIS_SPEED_MIN;
        break;

    case 't': case 'T':
        a->theme_idx = (a->theme_idx + 1) % N_THEMES;
        theme_apply(a->theme_idx);
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
