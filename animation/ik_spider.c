/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_spider.c — six-legged crawler with trail-buffer body and IK legs
 *
 * DEMO: A six-legged crawler walks across the terminal under autonomous
 *       steering. Its body curves through space along a trail buffer
 *       (FK), while each leg uses analytical 2-joint IK to reach a
 *       computed step target. Each leg steps independently when its foot
 *       drifts too far from its ideal position, with at most three legs
 *       airborne at once for stability.
 *
 * Study alongside: hexpod_tripod.c            (rigid-body chassis contrast)
 *                  snake_forward_kinematics.c (trail-buffer body sibling)
 *
 * Section map:
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — 10 themes + spec HUD/hint pairs
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Spider: body FK + IK legs + step gait
 *       §5a  vec2 + small helpers
 *       §5b  trail (push / at / sample)
 *       §5c  body motion (steer + translate + wrap)
 *       §5d  body joints (trail-buffer FK)
 *       §5e  hip placement (legs attach to body)
 *       §5f  2-joint analytical IK (law of cosines)
 *       §5g  step gait (drift detect, swing animate, autonomous)
 *       §5h  rendering helpers (line/bead/marker primitives)
 *       §5i  render_spider (orchestrator)
 *   §6  scene         — thin Scene wrapper
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 *
 * Keys:  q / ESC     quit                 space   pause / resume
 *        ↑ ↓ ← →     steer in 4 directions
 *        w / s       speed × / ÷ 1.25
 *        t           cycle theme          [ / ]   time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_spider.c \
 *       -o ik_spider -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three sub-systems share one body.
 *
 *                 BODY uses path-following (trail-buffer) FK, identical
 *                 to snake_forward_kinematics.c. Every tick the head's
 *                 position is pushed into a circular trail; each body
 *                 joint is placed at arc-length i*BODY_SEG_LEN backward
 *                 along the recorded path. The body curves naturally
 *                 wherever the head went — no per-segment angle math.
 *
 *                 LEGS use 2-joint analytical IK via law of cosines:
 *                     cos(θ_hip) = (d² + U² − L²) / (2·d·U)
 *                 with the hip→target distance d clamped into the
 *                 reachable annulus to keep acos in [-1, 1]. Left and
 *                 right legs use opposite signs of θ_hip so knees splay
 *                 outward from the body centre line.
 *
 *                 GAIT is per-leg autonomous: each leg checks its own
 *                 drift from ideal foot position and over-stretch from
 *                 the hip, and triggers a swing when ready — gated by
 *                 a global "no more than N_LEGS/2 legs in the air at
 *                 once" stability cap. This is fluid and asymmetric vs
 *                 the lockstep tripod gait of hexpod_tripod.c.
 *
 *                 STEERING interpolates heading toward target_heading
 *                 at TURN_RATE rad/s, taking the short arc through ±π.
 *
 * Data-structure: Spider holds the trail buffer + body joints (4
 *                 segments curving through space), per-leg state
 *                 (hip / knee / foot / step animation), heading +
 *                 target_heading, and ui state. Static tables LEG_ANGLE
 *                 and HIP_BODY_T encode each leg's angular bias and
 *                 attachment point along the body.
 *
 * Rendering     : Painter's order — leg lines (femur + tibia
 *                 alternating direction-glyph and `.` for chain look)
 *                 → knee 'o' markers + foot '*'/'o' markers → body
 *                 bead-fill (tail→head) → body 'O'/'o' joint markers
 *                 → eye cluster `:>:` at the head perpendicular to
 *                 heading. The eye cluster + chunky 'O' abdomen tip
 *                 are the spider's visual signature, distinct from
 *                 hexpod_tripod's plain rectangle chassis.
 *
 * Performance   : Variable timestep at render rate. Per frame: 6 IK
 *                 solves (each one acos + atan2 + sin/cos), 6 step
 *                 animation updates, body motion + trail push +
 *                 4-joint trail sample. Microseconds total.
 *
 * References
 * ──────────
 *   ── Analytical 2-joint IK (the §5f solver) ───────────────────────
 *   [1] Craig, J. J. (2005), "Introduction to Robotics: Mechanics
 *       and Control" (3rd ed.), Pearson — Ch. 4 derives the law-of-
 *       cosines 2-link solution that solve_ik() implements verbatim;
 *       §4.4 "Solvability" justifies the reachable-annulus clamp.
 *   [2] Spong, M. W., Hutchinson, S. & Vidyasagar, M. (2005), "Robot
 *       Modeling and Control", Wiley — alternative canonical text;
 *       Ch. 4 covers the knee-up/knee-down configuration ambiguity.
 *   [3] al-Kāshī, J. (1427), "Miftāḥ al-Hisāb" — earliest known
 *       trigonometric form of the law of cosines (Euclid II.12-13
 *       has the geometric form ~300 BCE); both drive solve_ik.
 *
 *   ── Iterative IK alternatives (the contrast) ─────────────────────
 *   [4] Aristidou, A. & Lasenby, J. (2011), "FABRIK: a fast,
 *       iterative solver for the inverse kinematics problem",
 *       Graphical Models 73(5), pp. 243-260 — iterative IK for
 *       longer chains; for 2-joint legs the closed-form wins on
 *       simplicity and exactness. See ik_arm_reach.c for a FABRIK
 *       application.
 *   [5] Buss, S. R. (2004), "Introduction to Inverse Kinematics with
 *       Jacobian Transpose, Pseudoinverse and Damped Least Squares",
 *       UCSD course notes — Jacobian-iterative family that
 *       solve_ik AVOIDS by staying analytical at 2 joints.
 *
 *   ── Procedural motion / steering ─────────────────────────────────
 *   [6] Reynolds, C. (1999), "Steering Behaviors for Autonomous
 *       Characters", GDC — the slew-rate-limited heading-toward-
 *       target pattern in steer_heading.
 *       https://www.red3d.com/cwr/steer/
 *
 *   ── Biological gait (the autonomous step-trigger model) ─────────
 *   [7] Wilson, D. M. (1966), "Insect Walking", Annual Review of
 *       Entomology 11, pp. 103-122 — basis for the alternating
 *       support pattern; the "≤ N_LEGS/2 airborne" guard in
 *       update_steps follows this rule.
 *   [8] Cruse, H. (1990), "What mechanisms coordinate leg movement
 *       in walking arthropods?", Trends in Neurosciences 13(1) —
 *       inter-leg coordination rules behind step triggering.
 *   [9] Full, R. J. & Tu, M. S. (1991), "Mechanics of a rapid
 *       running insect: two-, four- and six-legged locomotion",
 *       J. Exp. Biol. 156, pp. 215-231 — quantitative reference
 *       for leg-pair phase + duty-cycle in legged locomotion.
 *
 *   ── Differential geometry (2-D Frenet–Serret) ────────────────────
 *  [10] do Carmo, M. P. (1976), "Differential Geometry of Curves
 *       and Surfaces", Prentice-Hall — Ch. 1 §6: the Frenet frame.
 *       lateral_normal_at_spine is the 2-D normal of the spine
 *       polyline (rotate tangent by π/2).
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *  [11] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com —
 *       when fixed-step matters; this file uses variable-step
 *       because solve_ik is closed-form and the gait is stable.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *  [12] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       integer line drawing; §5h draw_leg_line uses the simpler
 *       parametric oversample because float math is already paid
 *       for in the FK / IK pipeline.
 *  [13] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the high-contrast glyph ramp.
 *  [14] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the diff pipeline §7 relies on.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [15] https://en.wikipedia.org/wiki/Inverse_kinematics
 *  [16] https://en.wikipedia.org/wiki/Law_of_cosines
 *  [17] https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The spider is a snake-like body with six pendulum legs hanging off
 * it. Body: head walks autonomously, body trails along the path the
 * head carved (trail-buffer FK). Legs: each one is a 2-bar linkage
 * solved by trig; given the hip's current world position and the foot's
 * planted target, the knee falls out of the law of cosines. Gait: a
 * leg autonomously decides when to swing forward — when its foot has
 * drifted too far from where it should be, OR the leg is stretched
 * past its IK reach. A stability cap keeps at most three legs airborne
 * at any instant so the support tripod is always intact.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Body : a four-bead chain that follows the trail of the head. Turning
 *        bends the body just like a snake — the curving body is the
 *        spider's identity vs hexpod_tripod's rigid rectangular chassis.
 *
 * Legs : six 2-bar linkages with hips anchored to the body. Each hip
 *        attaches at a parametric position along the body (front pair
 *        near the head, mid pair in the middle, rear pair near the
 *        abdomen) with a lateral offset to either the left or right.
 *
 * Knees: knee position is a closed-form function of (hip, foot,
 *        UPPER_LEN, LOWER_LEN, side) — left legs bend knee outward to
 *        the left, right legs to the right. No iteration.
 *
 * Gait : an autonomous timer per leg. Each leg watches its own foot's
 *        drift from the ideal step target; when it triggers, the foot
 *        animates from old position to new along a smoothstep ease.
 *        Stability cap: at most N_LEGS/2 = 3 legs may be airborne, so
 *        ground contact is always at least 3-point.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. Steer: heading interpolates toward target_heading at TURN_RATE
 *     rad/s, taking the short arc through ±π.
 *  3. Translate body_joint[0] (head) along heading at move_speed; wrap
 *     toroidally. Push the new head position into the circular trail.
 *  4. Body: for each i in 1..N_BODY_SEGS, body_joint[i] = trail_sample
 *     at arc-length i·BODY_SEG_LEN backward from the head.
 *  5. Hips: each hip's world position = attachment point along the
 *     body (interpolated from HIP_BODY_T) + lateral hip_dist on the
 *     correct side (left/right by leg parity).
 *  6. Stretch-snap: any foot now beyond IK reach (hip just moved a
 *     screen-wrap teleport) gets snapped to its rest position.
 *  7. Step gait: each non-stepping leg checks drift and stretch; if
 *     above thresholds AND fewer than N_LEGS/2 legs already airborne,
 *     trigger swing. Each stepping leg animates its foot via
 *     smoothstep until step_t hits 1.0 and lands.
 *  8. IK: solve law-of-cosines for every leg → knee position.
 *  9. Render painter's order: leg lines → leg joint markers → body
 *     bead-fill → body markers → head eye cluster + arrow.
 *
 * KEY FORMULAS
 * ────────────
 *  Trail sample (s): walk trail backward summing segment distances
 *                    until total ≥ s; interpolate inside the bracketing
 *                    segment.
 *
 *  Heading lerp    : turn = clamp(target − heading, ±TURN_RATE · dt)
 *                    (with target − heading wrapped to [−π, π])
 *
 *  Hip placement   : attach = lerp(body_joint[k], body_joint[k+1], frac)
 *                    forward = norm(body_joint[k] − body_joint[k+1])
 *                    left_normal = (−forward.y, forward.x)
 *                    hip = attach + side · hip_dist · left_normal
 *                    ( side = +1 left, −1 right )
 *
 *  2-joint IK      : dist  = clamp(|T − H|, |U − L| + 1, U + L − 1)
 *                    base  = atan2(Ty − Hy, Tx − Hx)
 *                    cos_h = (dist² + U² − L²) / (2 · dist · U)
 *                    θ_hip = acos(clamp(cos_h, −1, 1))
 *                    knee_angle = base ± θ_hip   ( + left, − right )
 *                    knee  = H + U · (cos knee_angle, sin knee_angle)
 *
 *  Ideal foot      : forward = (cos heading, sin heading)
 *                    dir     = rotate2d(forward, LEG_ANGLE[i])
 *                    foot    = hip + dir · (UPPER + LOWER) · STEP_REACH_FACTOR
 *
 *  Step swing      : ease = smoothstep(step_t)
 *                    foot = lerp(foot_old, step_target, ease)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - IK clamp. The hip↔target distance is bounded into the reachable
 *    annulus so acos never sees a value outside [−1, 1]. Without the
 *    clamp, an over-stretched leg yields NaN and the chain vanishes.
 *
 *  - Toroidal wrap during walk. The body wraps with the trail buffer,
 *    but planted feet do NOT — they were anchored to old coordinates.
 *    The stretch-snap pass detects feet beyond reach and snaps them to
 *    rest. The visible glitch is one frame of teleporting feet, then
 *    the gait recovers.
 *
 *  - Stability cap. The "n_air < N_LEGS/2" gate prevents all legs from
 *    swinging at once. At very high speeds, drift may exceed the
 *    threshold for many legs simultaneously; they queue up and step in
 *    turn, which makes the gait visibly hurry but never tip.
 *
 *  - Suspend / lid-close. dt clamped to 100 ms in main() so the body
 *    doesn't teleport across the screen on resume.
 *
 *  - Glyph aliasing. The four ASCII line glyphs ('-', '\', '|', '/')
 *    sample a continuous angle; near boundaries the glyph flickers
 *    when a leg rotates through the threshold. Folding to [0°, 180°)
 *    halves the boundary crossings.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: spider walks rightward at 45 px/s. The body
 *    visibly curves when steering (vs hexpod's rigid rotation). At any
 *    frozen frame, exactly 3-or-fewer legs have '.' (swinging) and the
 *    rest have '*' (planted) — never all six in the air.
 *
 *  - Press arrows → heading interpolates toward target; a 90° turn
 *    takes ~0.6 s. Body bends through the turn following the head.
 *
 *  - Press space → everything freezes. Un-pause → motion resumes
 *    exactly from where it was.
 *
 *  - Crank speed with `w` → step lookahead grows so feet land further
 *    ahead of the hips. Gait keeps up via the per-leg drift trigger.
 *
 *  - Press `[` for slow time → motion stays smooth (variable timestep
 *    guarantees this); press `]` for fast-forward.
 *
 *  - Cycle themes with `t` → spider colour changes; HUD stays bright
 *    yellow regardless.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read hexpod_tripod.c first; this file uses the
 *      same per-leg IK (T1 there) but on a CURVED FK body and
 *      with an asymmetric per-leg gait instead of locked tripods.
 *   2. §5 entity — THE HEART. In sub-section order:
 *        §5b trail (push / sample)        ← from snake FK
 *        §5c-§5d body motion + joints     ← path-following FK
 *        §5e hip placement                ← T2 below
 *        §5f 2-joint IK                   ← same as hexpod T1
 *        §5g step gait                    ← T3-T5 — the new lesson
 *        §5h-§5i rendering
 *   3. §6 scene — thin wrapper.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   trail[]                    head's recent positions (circular).
 *   body_joint[i]              i-th body joint world position.
 *                              0 = head, N_BODY_SEGS = abdomen tip.
 *   HIP_BODY_T[i]              parametric position along the body
 *                              for hip i (0 = head, 1 = tail).
 *   LEG_ANGLE[i]               per-leg body-relative angular bias
 *                              (sets each leg's "natural" reach
 *                              direction).
 *   hip[i], knee[i], foot[i]   per-leg joint world positions.
 *   stepping[i]                bool — leg i mid-swing.
 *   step_t[i]                  swing progress ∈ [0, 1].
 *   step_old[i]                foot position at start of swing.
 *   step_target[i]             foot position at end of swing.
 *   N_LEGS                     6.
 *   N_LEGS/2                   stability cap — max legs in air.
 *
 * Background you need
 * ───────────────────
 *   - Path-following FK / trail buffer (snake_forward_kinematics
 *     T1-T2).
 *   - 2-link analytical IK (hexpod_tripod T1 / ik_helloworld T3-T5).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Centre-of-mass dynamics. Same as hexpod — the body never
 *     tips because it's not gravity-loaded.
 *   - Coordinated gait state machines. THIS FILE has DECENTRALISED
 *     gait: each leg decides for itself when to step.
 *   - Iterative IK. Closed-form 2-link is exact.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a spider with curved body + per-leg
 * gait from first principles.
 *
 *   T1  Curved body vs rigid chassis — spider vs hexpod
 *   T2  Sliding hips along a curving body — the parametric attach
 *   T3  Per-leg autonomous gait — drift triggers a swing
 *   T4  Stability cap — no more than half the legs airborne
 *   T5  Why this gait READS LIVELY — emergence from local rules
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  CURVED BODY VS RIGID CHASSIS — SPIDER VS HEXPOD
 * ───────────────────────────────────────────────────
 * hexpod_tripod.c uses a RIGID body — a rectangle that
 * translates and rotates as a single unit. The body's mass is
 * pinned in one piece; only the legs move relative to it.
 *
 * This file uses a CURVING body — four bead joints connected
 * by trail-buffer FK (snake_forward_kinematics T1-T2). When
 * the spider turns, the body bends THROUGH the turn rather
 * than spinning rigidly.
 *
 *      ┌─────────────────────────────────────────────────┐
 *      │  hexpod (rigid):   ▭ → rotates as one piece     │
 *      │                                                 │
 *      │  spider (curved):  ●─●─●─● bends through turns  │
 *      └─────────────────────────────────────────────────┘
 *
 * Why curved? Aesthetics — real spiders have flexible
 * abdomen segments; rigid would look like a tin toy.
 *
 * Cost: hips are now ATTACHED TO A MOVING CURVE, not a fixed
 * rigid offset. T2 explains how to compute hip world positions
 * in that setting.
 *
 * T2  SLIDING HIPS ALONG A CURVING BODY — THE PARAMETRIC ATTACH
 * ─────────────────────────────────────────────────────────────
 * Each hip is attached at a parametric position along the body
 * curve, NOT at a fixed body-local offset:
 *
 *     HIP_BODY_T[6] = { 0.18, 0.18,    leg pair 1 — front
 *                       0.50, 0.50,    leg pair 2 — middle
 *                       0.82, 0.82 }   leg pair 3 — rear
 *
 * Each value is a LERP PARAMETER along the body chain (0 =
 * head, 1 = tail). To find the hip's world position:
 *
 *     T_along = HIP_BODY_T[i]
 *     k       = floor(T_along · N_BODY_SEGS)
 *     frac    = (T_along · N_BODY_SEGS) − k
 *     attach  = lerp(body_joint[k], body_joint[k+1], frac)
 *     forward = normalize(body_joint[k] − body_joint[k+1])
 *     left_normal = (−forward.y, forward.x)
 *     hip = attach + side · hip_dist · left_normal
 *
 * Three steps:
 *   1. Find the attach point on the body curve (linear
 *      interpolation between two adjacent body joints).
 *   2. Compute the body's LOCAL forward direction at that
 *      point (the direction of the body segment the attach
 *      sits on).
 *   3. Offset perpendicular to that direction by hip_dist on
 *      the correct side (left vs right).
 *
 * As the body curves, the perpendicular direction CHANGES along
 * the body, so left hips "steer" with the body's curvature.
 * That's why the spider's leg arrangement looks organic when it
 * turns — the hips track the body's spine.
 *
 * Hexpod uses a FIXED body-local offset rotated by a single
 * heading value. Spider uses a PARAMETRIC LOCATION along a
 * curve plus a local perpendicular at that location. Same idea
 * generalised to a non-rigid body.
 *
 * T3  PER-LEG AUTONOMOUS GAIT — DRIFT TRIGGERS A SWING
 * ────────────────────────────────────────────────────
 * Hexpod uses a CENTRAL gait state machine — phases A and B
 * swap at the drum-beat of a global timer. Spider does it the
 * other way: each leg decides for itself.
 *
 * Per-leg trigger condition:
 *
 *     drift = |foot − ideal_foot_for_current_hip|
 *     stretch = |foot − hip|
 *
 *     should_step = (drift > DRIFT_TRIGGER)
 *                OR (stretch > MAX_REACH)
 *
 * "Should step" means the foot is too far from where it ought
 * to be relative to the body's current pose, OR the foot is so
 * far that the leg can't physically reach it.
 *
 * When should_step fires AND the global stability cap (T4) is
 * not exceeded, the leg launches a swing:
 *
 *     stepping[i]    = true
 *     step_t[i]      = 0
 *     step_old[i]    = foot[i]
 *     step_target[i] = ideal_foot_for_current_hip + lookahead
 *
 * During the swing:
 *
 *     step_t[i] += SWING_RATE · dt
 *     ease = smoothstep(step_t[i])
 *     foot[i] = lerp(step_old[i], step_target[i], ease)
 *     when step_t[i] reaches 1.0: stepping[i] = false
 *
 * No central coordinator. Each leg is its own state machine.
 *
 * T4  STABILITY CAP — NO MORE THAN HALF THE LEGS AIRBORNE
 * ───────────────────────────────────────────────────────
 * If every leg followed its own trigger independently, ALL SIX
 * could swing simultaneously when drift conditions align — the
 * spider lifts all feet and falls.
 *
 * Stability cap: at most N_LEGS / 2 = 3 legs may be in the air
 * at the same time. Implementation:
 *
 *     n_air = count(stepping[*] == true)
 *     for each leg i:
 *       if not stepping[i] and should_step(i) and n_air < N_LEGS / 2:
 *         launch_swing(i)
 *         n_air += 1
 *
 * The check happens inside the leg-decision loop. Three
 * triggered legs go first; the others have to wait until one
 * lands and frees a slot.
 *
 * Two consequences:
 *   - At any instant ≥ 3 feet are planted → support polygon
 *     always exists → spider never tips.
 *   - At very high speeds, more than 3 legs want to step
 *     simultaneously → they QUEUE in the iteration order
 *     (legs 0, 1, 2 go first, 3-5 wait). Visually the gait
 *     looks "hurried" without breaking stability.
 *
 * The stability cap is the ONLY centralised piece of gait
 * logic. Everything else is per-leg autonomous.
 *
 * T5  WHY THIS GAIT READS LIVELY — EMERGENCE FROM LOCAL RULES
 * ───────────────────────────────────────────────────────────
 * Hexpod's tripod gait is HIGHLY VISUAL — the alternating
 * tripods are obvious to the eye. But it's also REGULAR; once
 * you see the pattern, it's clear what comes next.
 *
 * Spider's per-leg gait is IRREGULAR by design. Each leg
 * triggers on its own conditions, so the order in which legs
 * step depends on:
 *   - body curvature (faster turns drift outer-side hips
 *     more than inner-side)
 *   - body speed (faster speed = more drift per second)
 *   - what the body did 100 ms ago (legs lag the body)
 *
 * No two seconds look exactly alike. The result READS as a
 * living organism rather than a clockwork mechanism.
 *
 * This is "EMERGENT BEHAVIOUR" — complex, lifelike motion
 * arising from simple per-agent rules with no central
 * coordinator. The same principle drives boids (flocking from
 * three local rules), Reynolds steering (autonomous characters
 * from local sensors), and reaction-diffusion patterns (animal
 * coats from chemical kinetics).
 *
 * Decision tree for new locomotion creatures:
 *
 *   regular, mechanical look           → central gait machine
 *                                        (hexpod_tripod)
 *   organic, irregular look            → per-leg autonomous +
 *                                        stability cap (this file)
 *   need ground-contact dynamics       → add Verlet (ragdoll_*)
 *   simple wave undulation             → stateless FK
 *                                        (fk_tentacle_forest)
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
     *   1..N_PAIRS  body + legs gradient (themed)
     *   8 PAIR_HUD   bright yellow status bar (theme-independent)
     *   9 PAIR_HINT  bright cyan key hint    (theme-independent) */
    N_PAIRS       = 7,
    PAIR_HUD      = 8,
    PAIR_HINT     = 9,

    N_THEMES      = 10,    /* cycled with `t` */

    /* Anatomy.
     *   N_LEGS       = 6 (3 per side, three pairs at 60° angular spacing)
     *   N_BODY_SEGS  = 4 → body has 5 joints, 80 px end-to-end at
     *                  BODY_SEG_LEN = 20, matching hexpod_tripod's
     *                  BODY_LEN. The body's identity is that it CURVES
     *                  (multi-segment chain) rather than being rigid. */
    N_LEGS        = 6,
    N_BODY_SEGS   = 4,
    TRAIL_CAP     = 1024,
};

/* Body geometry (px). Body curves through space — multi-segment chain. */
#define BODY_SEG_LEN      20.0f   /* px between consecutive body joints     */
#define BODY_SPEED        45.0f   /* head translation speed, px/s           */
#define BODY_SPEED_MIN    10.0f
#define BODY_SPEED_MAX   200.0f
#define TURN_RATE          2.5f   /* rad/s — heading interpolation rate     */

/* Leg geometry (px). Legs are 4× body length to read as long-limbed. */
#define UPPER_LEN         56.0f   /* femur (hip → knee)                     */
#define LOWER_LEN         50.0f   /* tibia (knee → foot)                    */

/* Hip lateral offset from body centerline as a fraction of screen height.
 * 4% gives ~2 cells of body width on a typical 30-row terminal — narrow
 * silhouette, legs do most of the visual work. */
#define HIP_DIST_FACTOR   0.04f

/* Step gait parameters.
 *   STEP_REACH_FACTOR — ideal foot reach as fraction of (UPPER + LOWER).
 *   STEP_TRIGGER_DIST — px drift before a step is triggered.
 *   MAX_STRETCH       — absolute hip→foot distance that forces a step.
 *   STEP_DURATION     — seconds for one swing arc to complete. */
#define STEP_REACH_FACTOR 0.68f
#define STEP_TRIGGER_DIST 28.0f
#define MAX_STRETCH       65.0f
#define STEP_DURATION     0.22f

/* Direction-glyph step sizes. DRAW_STEP_PX < CELL_W (8) so the dense
 * stamping never skips a column. */
#define DRAW_STEP_PX      5.0f    /* body bead fill                         */
#define DRAW_LEG_STEP_PX  8.0f    /* leg direction-char lines               */

/* Eye cluster — perpendicular distance from head joint, in pixel space.
 * 10 px ≈ 1.25 cells; eyes flank the head arrow at all four directions. */
#define EYE_OFFSET_PX    10.0f

/* Time scale — user-controlled simulation speed multiplier on `[/]`. */
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

/*
 * LEG_ANGLE[i] — angle (radians) added to body forward to give leg i's
 * ideal step direction in body-local space.
 *
 * Three pairs at 60° angular spacing — each pair clearly distinct, no
 * overlap. Left legs (even index) get positive angle; right legs the
 * negative mirror.
 */
static const float LEG_ANGLE[N_LEGS] = {
     0.6f,    /* 0 front-LEFT   ~ 34° forward + outward */
    -0.6f,    /* 1 front-RIGHT  */
     1.57f,   /* 2 mid-LEFT     90° straight out        */
    -1.57f,   /* 3 mid-RIGHT    */
     2.5f,    /* 4 rear-LEFT    ~143° rear + outward    */
    -2.5f,    /* 5 rear-RIGHT   */
};

/*
 * HIP_BODY_T[i] — parametric position along the body where each hip
 * attaches (0 = head joint, 1 = tail joint). Three pairs spread evenly
 * along the (4-segment) body so the front/mid/rear anchor points are
 * spatially distinct — the eye reads three legs per side, not all six
 * stacked at one point.
 */
static const float HIP_BODY_T[N_LEGS] = {
    0.20f, 0.20f,   /* front pair — near the head */
    0.50f, 0.50f,   /* mid pair   — body centre   */
    0.80f, 0.80f,   /* rear pair  — near the tail */
};

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
 * Theme — one spider colour palette (xterm-256 fg indices).
 *
 * Intent
 *   Each theme rebinds the seven body-rendering pairs (1..7) in
 *   one shot via init_pair (see theme_apply). HUD/HINT pairs are
 *   theme-independent (PAIR_HUD = bright yellow, PAIR_HINT = bright
 *   cyan) so the status bar stays readable against any animation
 *   behind it — CLAUDE.md HUD spec.
 *
 * Slot semantics (col[0..6] map to pairs 1..7)
 *   [0] body gradient — tail end (dimmest)
 *   [1] body gradient — mid
 *   [2] body gradient — head end (main spider colour)
 *   [3] upper leg segment (femur)
 *   [4] lower leg segment (tibia)
 *   [5] BRIGHT accent — planted-foot '*' (in ground contact)
 *   [6] DIM accent    — swinging-foot '.' (in flight)
 *
 *   The tail→head gradient ([0]→[2]) lets the eye trace the body
 *   without joint-numbering overlay. The bright/dim split on feet
 *   [5]/[6] encodes ground contact at a glance: planted reads
 *   brighter than airborne. Ref [13] Bourke.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every entry sits in the BRIGHT HALF of the 256-colour space:
 *     - cube colours: ≥ 24  (avoid 16-23; invisible under A_DIM)
 *     - grayscale  : ≥ 240 (avoid 232-239; same reason)
 *   Theme character comes from the RELATIVE gradient, not absolute
 *   darkness.
 *
 * References [13] Bourke for the gradient-as-depth pattern;
 *   [14] Raymond §init_pair for the rebind mechanism.
 */
typedef struct {
    const char *name;            /* HUD-displayable theme name        */
    int         col[N_PAIRS];    /* xterm-256 fg index per pair slot; *
                                  * pair p = col[p-1] at apply time   */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        body gradient   legs      foot   ghost */
    {"Arachnid",{ 52,  88, 124,  58,  64,  70, 240}},
    {"Scarlet", { 88, 124, 160,  52,  88, 196, 240}},
    {"Toxic",   { 24,  28,  34,  28,  34,  82, 240}},
    {"Ocean",   { 24,  25,  27,  33,  39,  51, 240}},
    {"Nova",    { 54,  93, 129,  93, 129, 165, 240}},
    {"Ember",   { 52,  94, 130, 130, 130, 208, 240}},
    {"Aurora",  { 24,  29,  35,  35,  71, 221, 240}},
    {"Ghost",   {240, 244, 248, 244, 248, 254, 246}},
    {"Fire",    { 52,  88, 196,  88, 124, 226, 240}},
    {"Neon",    { 57,  93, 201,  93, 129, 201, 240}},
};

/* theme_apply — re-bind body/leg pairs (1..N_PAIRS) to chosen theme.
 * HUD/HINT pairs are NEVER touched — they're theme-independent. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS < 256) return;
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_PAIRS; p++)
        init_pair(p + 1, t->col[p], -1);
}

static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
    } else {
        /* 8-color fallback */
        init_pair(1, COLOR_RED,    -1);
        init_pair(2, COLOR_RED,    -1);
        init_pair(3, COLOR_RED,    -1);
        init_pair(4, COLOR_GREEN,  -1);
        init_pair(5, COLOR_GREEN,  -1);
        init_pair(6, COLOR_GREEN,  -1);
        init_pair(7, COLOR_WHITE,  -1);
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
/* §5  entity — Spider                                                    */
/* ===================================================================== */

/* Vec2 — 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention). */
/*
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   Body kinematics, IK, hip placement, and gait interpolation all
 *   live in pixel space. Each character cell is CELL_W × CELL_H
 *   sub-pixels (8 × 16), so a walking spider reads as continuous
 *   motion instead of jumping cell-to-cell. The conversion to cell
 *   space happens only inside §5h rendering helpers via
 *   px_to_cell_x/y — the project's "one conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → down the screen).
 *
 *   Angles are MATH-CONVENTION (positive CCW from +X). Because y
 *   points DOWN on screen, a math-CCW rotation displays as CW —
 *   this matters in solve_ik's port/starboard sign convention and
 *   in compute_hips's lateral-normal rotation (90° CCW = port).
 *
 * Why a value type
 *   8 bytes; passes in registers. solve_ik takes/returns Vec2 by
 *   value, vec2_add/sub/scale/lerp/norm all by value — -O2 inlines
 *   them into straight-line code with no allocation.
 *
 * Why named (x, y) instead of two loose floats
 *   Type-checking catches (col, row) confusion at compile time
 *   rather than via visual debugging.
 *
 * References [1] Craig §2 "Spatial descriptions"; [10] do Carmo for
 *   the 2-D vector formalism used in compute_hips.
 */
typedef struct {
    float x;   /* eastward  pixel coordinate (positive → right) */
    float y;   /* southward pixel coordinate (positive → down)  */
} Vec2;

/* ── §5a  vec2 + small helpers ──────────────────────────────────────── */

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

static inline float vec2_len (Vec2 v)
{ return sqrtf(v.x * v.x + v.y * v.y); }

static inline float vec2_dist(Vec2 a, Vec2 b)
{ return vec2_len(vec2_sub(a, b)); }

/* vec2_norm — unit vector. Zero-length input returns (1, 0); the
 * specific direction is arbitrary but lets callers use the result
 * without NaN propagation. */
static inline Vec2 vec2_norm(Vec2 v)
{
    float len = vec2_len(v);
    if (len < 1e-6f) return (Vec2){ 1.0f, 0.0f };
    return (Vec2){ v.x / len, v.y / len };
}

/* smoothstep — cubic ease-in/ease-out on [0, 1]. Used for foot swing. */
static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* rotate2d — rotate v by `angle` rad (screen space: +y = down). */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/*
 * Spider — the full creature state. Three interlocked subsystems
 * sharing one record:
 *
 *   1. BODY — trail-buffer FK. A circular log of past head positions;
 *             body_joint[1..N] sample the trail at arc-lengths
 *             i·BODY_SEG_LEN backward, so the body follows the path
 *             the head carved. No per-segment angle math, no IK.
 *             The same primitive as fk_centipede.c. Refs [1] Craig §3.
 *
 *   2. LEGS — 2-joint analytical IK (law of cosines) + alternating
 *             gait state machine. Per leg: (hip, foot_pos) drive an
 *             IK solve that places (knee). Stepping flag + step_t
 *             run a smoothstep-eased swing between foot_old and
 *             step_target. The "≤ N_LEGS/2 airborne at once" guard
 *             gives biological tripod-like stability. Refs [1][7][8].
 *
 *   3. STEERING — short-arc heading lerp toward target_heading,
 *                 rate-limited to TURN_RATE rad/s. Ref [6] Reynolds.
 *
 *   Subsystems are UNCOUPLED in time: body integrates from heading;
 *   hips read body_joint; gait reads hip; IK reads (hip, foot_pos).
 *   No feedback loop — scene_tick walks the dependency chain
 *   top-to-bottom once per frame.
 *
 * Why per-leg parallel arrays (SoA), not array-of-structs
 *   N_LEGS is small (≤ 8) and most loops walk ONE field across all
 *   legs at a time (renderer touches all foot_pos[]; gait touches
 *   all stepping[]). SoA matches that access pattern; static config
 *   tables (LEG_ANGLE, HIP_BODY_T) are parallel arrays too, so the
 *   convention is consistent.
 *
 * Why trail is a CIRCULAR buffer (not a regular array)
 *   Trail samples are pushed at the head and consumed at every
 *   distance along the body. Circular indexing means no O(N) shift
 *   per frame; trail_head walks mod TRAIL_CAP.
 *
 * Why knee[] is CACHED (recomputed each tick)
 *   solve_ik is cheap but not free. The renderer reads knee
 *   multiple times per frame (legs, joint markers); computing once
 *   in update_steps lets the rest of the frame skip the math.
 *
 * Why hip_dist is DERIVED (not a compile-time constant)
 *   Scaled to terminal height so legs spread proportionally.
 *   Recomputed on SIGWINCH (in scene_init/app_do_resize).
 *
 * Why foot_old + step_target + step_t together describe ONE swing
 *   A swing is a parametric arc from foot_old → step_target
 *   indexed by step_t ∈ [0, 1] with smoothstep easing. Decomposing
 *   it into three fields lets the gait state machine pause/resume
 *   a swing without re-solving the trajectory each tick.
 *
 * References [1] Craig §3-§4 for FK + IK; [6] Reynolds for steering;
 *   [7][8] Wilson + Cruse for the gait stability rule;
 *   [10] do Carmo for the lateral-normal math in compute_hips.
 */
typedef struct {
    /* ── Body trail-buffer FK ─────────────────────────────────── *
     * Circular log of recent head positions; body_joint[] is the *
     * arc-length-sampled output, rewritten each tick.            */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;                  /* index of newest entry, mod TRAIL_CAP */
    int  trail_count;                 /* valid entries, saturates at TRAIL_CAP*/
    Vec2 body_joint[N_BODY_SEGS + 1]; /* [0]=head, [N]=tail                   */

    /* ── Body kinematics (rigid-body in world pixel space) ───── *
     * heading is integrated from a slew-limited turn toward     *
     * target_heading; body_joint[0] integrates along heading.   */
    float heading;                    /* current direction (rad), 0 = +x      */
    float target_heading;             /* steered toward this each tick        */
    float move_speed;                 /* magnitude (px/s) along heading       */

    /* ── Per-leg state (SoA, length N_LEGS) ──────────────────── *
     * hip and knee are derived (output of compute_hips/solve_ik). *
     * foot_pos is in/out of the gait state machine; foot_old +   *
     * step_target + step_t + stepping describe an active swing. */
    Vec2  hip        [N_LEGS];        /* derived: world hip per tick          */
    Vec2  knee       [N_LEGS];        /* derived: IK output per tick          */
    Vec2  foot_pos   [N_LEGS];        /* current IK target (planted or arcing)*/
    Vec2  foot_old   [N_LEGS];        /* foot at swing start (lerp anchor)    */
    Vec2  step_target[N_LEGS];        /* landing point this swing aims at     */
    bool  stepping   [N_LEGS];        /* in-flight flag                       */
    float step_t     [N_LEGS];        /* swing progress in [0, 1]             */

    /* ── Resize-derived geometry ─────────────────────────────── *
     * Scaled at init+resize so legs spread proportionally to    *
     * terminal size.                                            */
    float hip_dist;                   /* lateral offset of hip from spine (px)*/

    /* ── UI / control state ─────────────────────────────────── *
     * paused gates scene_tick (render still runs);              *
     * theme_idx selects the §3 palette.                         */
    bool  paused;
    int   theme_idx;
} Spider;

/* ── §5b  trail (push / at / sample) ────────────────────────────────── */

static void trail_push(Spider *sp, Vec2 pos)
{
    sp->trail_head = (sp->trail_head + 1) % TRAIL_CAP;
    sp->trail[sp->trail_head] = pos;
    if (sp->trail_count < TRAIL_CAP) sp->trail_count++;
}

static inline Vec2 trail_at(const Spider *sp, int k)
{
    return sp->trail[(sp->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

/*
 * trail_sample — interpolated position at arc-length `dist` from head.
 * Walks the trail accumulating Euclidean distance until the running
 * total crosses `dist`, then linearly interpolates inside the
 * bracketing segment. The body literally retraces the head's path.
 */
static Vec2 trail_sample(const Spider *sp, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(sp, 0);

    for (int k = 1; k < sp->trail_count; k++) {
        Vec2  b   = trail_at(sp, k);
        float dx  = b.x - a.x;
        float dy  = b.y - a.y;
        float seg = sqrtf(dx * dx + dy * dy);

        if (accum + seg >= dist) {
            float t = (dist - accum) / (seg > 1e-4f ? seg : 1e-4f);
            return (Vec2){ a.x + dx * t, a.y + dy * t };
        }
        accum += seg;
        a      = b;
    }
    return trail_at(sp, sp->trail_count - 1);
}

/* ── §5c  body motion ──────────────────────────────────────────────── */

/*
 * shortest_signed_angle — wrap a heading difference to [-π, π].
 *
 *   Heading lives on a circle; raw subtraction can give +5π/3 when
 *   the true short rotation is -π/3. Wrapping makes the spider
 *   always turn the SHORT way round the circle — critical when
 *   target_heading and current heading straddle the π / -π
 *   discontinuity.
 */
static float shortest_signed_angle(float diff)
{
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return diff;
}

/*
 * clamp_to_max_turn_per_dt — rate limit on angular velocity.
 *
 *   The slew-rate limiter from Reynolds steering: |dθ/dt| ≤ TURN_RATE.
 *   No matter how far away target_heading is, the heading can only
 *   change by TURN_RATE · dt radians per tick — producing a smooth
 *   constant-angular-velocity turn instead of an instant snap.
 */
static float clamp_to_max_turn_per_dt(float diff, float dt)
{
    float max_step = TURN_RATE * dt;
    return clampf(diff, -max_step, max_step);
}

/*
 * steer_heading — rate-limited rotation toward target_heading.
 *
 *   1. shortest_signed_angle    : take the SHORT direction.
 *   2. clamp_to_max_turn_per_dt : slew-rate limit (max angular vel).
 *   3. heading += clamped step.
 *
 * Reference: Reynolds (1999) "Steering Behaviors for Autonomous
 * Characters", §"Wander / Seek" — the rate-limited heading-
 * correction pattern.
 */
static void steer_heading(Spider *sp, float dt)
{
    float diff    = shortest_signed_angle(sp->target_heading - sp->heading);
    float clamped = clamp_to_max_turn_per_dt(diff, dt);
    sp->heading  += clamped;
}

/*
 * integrate_position_along_heading — explicit Euler step.
 *
 *   x(t+dt) = x(t) + v · cos(θ) · dt
 *   y(t+dt) = y(t) + v · sin(θ) · dt
 *
 *   move_speed is constant magnitude; (cos θ, sin θ) is the unit
 *   direction. Explicit Euler is unconditionally stable here because
 *   there is no acceleration term — body is purely kinematic.
 */
static void integrate_position_along_heading(Spider *sp, float dt)
{
    sp->body_joint[0].x += sp->move_speed * cosf(sp->heading) * dt;
    sp->body_joint[0].y += sp->move_speed * sinf(sp->heading) * dt;
}

/*
 * wrap_position_to_toroidal_world — modular boundary.
 *
 *   The screen is treated as a TORUS — exiting the right edge re-
 *   enters from the left, exiting the bottom re-enters from the
 *   top. Mathematically:  pos ≡ pos  (mod world_size)
 *
 *   Subtract/add one world span instead of `fmodf` so a one-tick
 *   overshoot resolves in O(1) without floating-point modulus
 *   precision loss. Assumes |step| < world_size, which holds for
 *   realistic dt + move_speed combinations.
 */
static void wrap_position_to_toroidal_world(Spider *sp, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (sp->body_joint[0].x <  0.0f) sp->body_joint[0].x += wpx;
    if (sp->body_joint[0].x >= wpx)  sp->body_joint[0].x -= wpx;
    if (sp->body_joint[0].y <  0.0f) sp->body_joint[0].y += hpx;
    if (sp->body_joint[0].y >= hpx)  sp->body_joint[0].y -= hpx;
}

/*
 * translate_body — advance head one tick and record the trail.
 *
 *   1. integrate_position_along_heading  : Euler step in heading dir.
 *   2. wrap_position_to_toroidal_world   : modular boundary.
 *   3. trail_push                        : FK source-of-truth update —
 *                                          body_joints follow the head.
 */
static void translate_body(Spider *sp, float dt, int cols, int rows)
{
    integrate_position_along_heading  (sp, dt);
    wrap_position_to_toroidal_world   (sp, cols, rows);
    trail_push                        (sp, sp->body_joint[0]);
}

/* ── §5d  body joints (trail-buffer FK) ──────────────────────────── */

/* compute_body_joints — place body_joint[1..N] by arc-length sampling
 * along the trail. body_joint[0] is set by translate_body(). */
static void compute_body_joints(Spider *sp)
{
    for (int i = 1; i <= N_BODY_SEGS; i++)
        sp->body_joint[i] = trail_sample(sp, (float)i * BODY_SEG_LEN);
}

/* ── §5e  hip placement ──────────────────────────────────────────── */

/* body_local_forward — local body forward direction at parametric
 * position t along the body. Smoothed by a single-segment difference. */
static Vec2 body_local_forward(const Spider *sp, int seg_idx)
{
    if (seg_idx + 1 > N_BODY_SEGS)
        return (Vec2){ cosf(sp->heading), sinf(sp->heading) };
    return vec2_norm(vec2_sub(sp->body_joint[seg_idx],
                              sp->body_joint[seg_idx + 1]));
}

/*
 * attachment_point_along_spine — world-space point at fractional
 * arc-length t along the body-joint polyline.
 *
 *   The spine is a polyline through body_joint[0..N_BODY_SEGS].
 *   HIP_BODY_T[i] ∈ [0,1] is leg i's normalised position along it.
 *   Multiply by N_BODY_SEGS to get a real-valued index, split into
 *   integer (segment) + fractional (interpolation weight) parts,
 *   then linearly interpolate between the two bracketing joints.
 *
 *   This is PIECEWISE-LINEAR ARC-LENGTH PARAMETERISATION — the same
 *   trick a tessellator uses to walk a curve at constant rate.
 *
 *   Outputs the spine point AND the segment index whose tangent the
 *   caller will need (so compute_hips doesn't recompute it).
 */
static Vec2 attachment_point_along_spine(const Spider *sp, float t_norm,
                                         int *seg_idx_out)
{
    float t_body  = t_norm * (float)N_BODY_SEGS;
    int   seg_idx = (int)t_body;
    if (seg_idx >= N_BODY_SEGS) seg_idx = N_BODY_SEGS - 1;
    float frac    = t_body - (float)seg_idx;

    *seg_idx_out = seg_idx;
    return vec2_lerp(sp->body_joint[seg_idx],
                     sp->body_joint[seg_idx + 1], frac);
}

/*
 * lateral_normal_at_spine — 90° rotation of the local body tangent.
 *
 *   In 2-D the LEFT-HAND NORMAL of a unit tangent (tx, ty) is
 *   (-ty, tx) — a +π/2 rotation. Multiplying by `side` ∈ {+1, -1}
 *   gives port (left) or starboard (right) lateral direction.
 *
 *   This is the Frenet–Serret normal for a 2-D planar curve; in 3-D
 *   it becomes one of two principal normals and the math gets harder.
 *   In 2-D we just rotate by 90°.
 *
 *   Reference: do Carmo, "Differential Geometry of Curves and
 *   Surfaces" Ch. 1 §6 — the Frenet frame.
 */
static Vec2 lateral_normal_at_spine(const Spider *sp, int seg_idx, float side)
{
    Vec2 fwd       = body_local_forward(sp, seg_idx);
    Vec2 left_norm = (Vec2){ -fwd.y, fwd.x };           /* +π/2 rotation */
    return vec2_scale(left_norm, side);
}

/*
 * compute_hips — place all N_LEGS hips by stepping laterally off the
 * spine.
 *
 *   For each leg i:
 *     1. attachment_point_along_spine : where on the body chain this
 *                                       leg hangs from.
 *     2. lateral_normal_at_spine      : ± Frenet normal (port/starboard).
 *     3. hip = attachment + hip_dist · normal.
 *
 *   side = +1 for even i (port), −1 for odd (starboard). hip_dist is
 *   scaled to terminal height so legs spread proportionally.
 */
static void compute_hips(Spider *sp)
{
    for (int i = 0; i < N_LEGS; i++) {
        int  seg_idx;
        Vec2 attach = attachment_point_along_spine(sp, HIP_BODY_T[i], &seg_idx);

        float side  = (i % 2 == 0) ? 1.0f : -1.0f;
        Vec2  out   = lateral_normal_at_spine(sp, seg_idx, side);

        sp->hip[i]  = vec2_add(attach, vec2_scale(out, sp->hip_dist));
    }
}

/* ── §5f  2-joint analytical IK ──────────────────────────────────── */

/*
 * clamp_to_reachable_annulus — pull distance into the IK-solvable range.
 *
 *   A 2-link chain (lengths U, L) can reach exactly the points whose
 *   distance from the hip lies in the closed interval [|U-L|, U+L]:
 *     - INNER boundary |U-L|: legs folded flat against each other.
 *     - OUTER boundary  U+L : legs fully extended in a straight line.
 *   Outside this annulus there is no real triangle and acos's argument
 *   leaves [-1, 1]. The ±1 px margin keeps cos_h strictly interior so
 *   floating-point round-off can't push it onto the singular boundary.
 *
 *   Geometric reference: the triangle inequality.
 */
static float clamp_to_reachable_annulus(float dist)
{
    return clampf(dist,
                  fabsf(UPPER_LEN - LOWER_LEN) + 1.0f,
                  UPPER_LEN + LOWER_LEN        - 1.0f);
}

/*
 * law_of_cosines_apex_angle — angle at the hip in the triangle
 * (hip, knee, foot).
 *
 *   Sides:  U = hip→knee, L = knee→foot, d = hip→foot.
 *   Law of cosines at the HIP vertex (opposite side L):
 *
 *       L² = U² + d² − 2·U·d·cos(α)
 *       cos(α) = (d² + U² − L²) / (2·d·U)
 *       α      = acos(cos(α))
 *
 *   α is the angle between the U-side (upper leg) and the d-side
 *   (line to target). Always positive (acos ∈ [0, π]); the
 *   knee-up vs knee-down ambiguity is resolved in the caller by
 *   adding or subtracting α from `base`.
 *
 *   Reference: law of cosines — geometric form in Euclid's Elements
 *   II.12-13 (~300 BCE), trigonometric form in al-Kāshī's "Miftāḥ
 *   al-Hisāb" (1427).
 */
static float law_of_cosines_apex_angle(float dist)
{
    float cos_h = (dist * dist + UPPER_LEN * UPPER_LEN
                                - LOWER_LEN * LOWER_LEN)
                  / (2.0f * dist * UPPER_LEN);
    return acosf(clampf(cos_h, -1.0f, 1.0f));
}

/*
 * place_knee_at_angle — walk UPPER_LEN from hip along a given angle.
 *
 *   knee = hip + U · (cos θ, sin θ)
 *
 *   The FK PRIMITIVE (one rigid link from a base) applied once.
 *   After this, the lower leg is implicit — it runs straight from
 *   knee to foot (and is exactly LOWER_LEN long iff dist was inside
 *   the reachable annulus, guaranteed by the clamp above).
 */
static Vec2 place_knee_at_angle(Vec2 hip, float angle)
{
    return (Vec2){ hip.x + UPPER_LEN * cosf(angle),
                   hip.y + UPPER_LEN * sinf(angle) };
}

/*
 * solve_ik — analytical 2-joint IK by law of cosines.
 *
 *   Geometry — triangle (hip, knee, foot):
 *
 *       hip *────────── U ──────────* knee
 *            \                      /
 *             \                    /
 *               d                 L
 *                \              /
 *                 \           /
 *                  *────────* foot (= target)
 *
 *   1. d = |target − hip|
 *   2. clamp_to_reachable_annulus(d)  — keep the triangle valid.
 *   3. base = atan2(target − hip)     — direction from hip to target.
 *   4. α    = law_of_cosines_apex_angle(d)
 *        offset of the knee from the d-line.
 *   5. knee_angle = base ± α          — sign picks knee-up / knee-down.
 *      Left-side legs: +α (knee on port side).
 *      Right-side legs: -α (knee on starboard side).
 *   6. place_knee_at_angle(hip, knee_angle).
 *
 * Closed-form, branchless except for the L/R sign and the safety
 * clamps. References: Wikipedia "Inverse kinematics" §"Analytical
 * 2-link"; Craig "Introduction to Robotics" Ch. 4 "Solvability".
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = clamp_to_reachable_annulus(sqrtf(dx * dx + dy * dy));

    float base       = atan2f(dy, dx);
    float alpha      = law_of_cosines_apex_angle(dist);
    float knee_angle = is_left ? (base + alpha) : (base - alpha);

    *knee_out = place_knee_at_angle(hip, knee_angle);
}

/* ── §5g  step gait ──────────────────────────────────────────────── */

/*
 * leg_facing_direction — unit vector pointing where leg i naturally
 * splays.
 *
 *   Start with the body's heading unit vector (cos θ, sin θ), then
 *   rotate by LEG_ANGLE[i] — a per-leg constant that bends the
 *   forward direction outward for front legs and rearward for hind
 *   legs. The result is the leg's "rest azimuth" in world space.
 *
 *   This is a 2-D rigid rotation (SO(2) matrix) applied to a unit
 *   vector — the same operation used to derive leg orientations
 *   from a body pose in any articulated kinematics scheme.
 */
static Vec2 leg_facing_direction(const Spider *sp, int i)
{
    Vec2 body_forward = (Vec2){ cosf(sp->heading), sinf(sp->heading) };
    return rotate2d(body_forward, LEG_ANGLE[i]);
}

/*
 * rest_reach_distance — how far from the hip the ideal foot lands.
 *
 *   Total leg length (U + L) scaled by STEP_REACH_FACTOR ∈ (0, 1).
 *   A factor < 1 ensures the foot lands STRICTLY INSIDE the
 *   reachable annulus, so a small step or body wobble can't push
 *   the foot to the IK boundary where law_of_cosines would clip.
 *   STEP_REACH_FACTOR is the engineering "comfort margin" knob.
 */
static float rest_reach_distance(void)
{
    return (UPPER_LEN + LOWER_LEN) * STEP_REACH_FACTOR;
}

/*
 * compute_ideal_foot — desired foot position when leg i is not stepping.
 *
 *   ideal = hip + reach · direction
 *
 *   1. leg_facing_direction(i) : unit vector in the leg's rest azimuth.
 *   2. rest_reach_distance()   : magnitude (inside the IK annulus).
 *   3. hip + reach · direction : translate.
 *
 *   This is the gait's TARGET — when the planted foot has drifted
 *   too far from this ideal (or is over-stretched), maybe_trigger_step
 *   launches a swing toward it.
 */
static Vec2 compute_ideal_foot(const Spider *sp, int i)
{
    Vec2  dir   = leg_facing_direction(sp, i);
    float reach = rest_reach_distance();
    return vec2_add(sp->hip[i], vec2_scale(dir, reach));
}

/* count_airborne — number of legs currently in swing phase. */
static int count_airborne(const Spider *sp)
{
    int n = 0;
    for (int i = 0; i < N_LEGS; i++)
        if (sp->stepping[i]) n++;
    return n;
}

/* snap_overstretched_foot — recover after toroidal wrap or sharp turn.
 * If a foot is now beyond IK reach (e.g., because the body just wrapped
 * across the screen), force it to its rest position. Returns true if
 * the leg was airborne and got snapped (caller must decrement n_air). */
static bool snap_overstretched_foot(Spider *sp, int i)
{
    if (vec2_dist(sp->foot_pos[i], sp->hip[i]) <= UPPER_LEN + LOWER_LEN - 2.0f)
        return false;

    bool was_airborne   = sp->stepping[i];
    sp->foot_pos[i]     = compute_ideal_foot(sp, i);
    sp->foot_old[i]     = sp->foot_pos[i];
    sp->step_target[i]  = sp->foot_pos[i];
    sp->stepping[i]     = false;
    sp->step_t[i]       = 0.0f;
    return was_airborne;
}

/* maybe_trigger_step — for a non-stepping leg, decide whether to swing.
 * Triggers if foot has drifted from ideal OR is over-stretched. Gated
 * by the n_air < N_LEGS/2 stability cap. Returns true if a swing started. */
static bool maybe_trigger_step(Spider *sp, int i, int n_air)
{
    if (sp->stepping[i] || n_air >= N_LEGS / 2) return false;

    Vec2  ideal   = compute_ideal_foot(sp, i);
    float drift   = vec2_dist(sp->foot_pos[i], ideal);
    float stretch = vec2_dist(sp->foot_pos[i], sp->hip[i]);
    if (drift <= STEP_TRIGGER_DIST && stretch <= MAX_STRETCH) return false;

    sp->stepping[i]    = true;
    sp->step_t[i]      = 0.0f;
    sp->foot_old[i]    = sp->foot_pos[i];
    sp->step_target[i] = ideal;
    return true;
}

/* advance_swing — for a stepping leg, animate the foot along its arc.
 * Returns true if the swing finished this frame (caller decrements n_air). */
static bool advance_swing(Spider *sp, int i, float dt)
{
    sp->step_t[i] += dt / STEP_DURATION;
    if (sp->step_t[i] >= 1.0f) {
        sp->step_t[i]   = 1.0f;
        sp->foot_pos[i] = sp->step_target[i];
        sp->stepping[i] = false;
        return true;
    }
    float ease     = smoothstep(sp->step_t[i]);
    sp->foot_pos[i] = vec2_lerp(sp->foot_old[i], sp->step_target[i], ease);
    return false;
}

/*
 * update_steps — one gait-tick orchestrator.
 *
 *   Per-leg dispatch (mutually exclusive): each leg is in exactly
 *   one of three states this tick, with airborne-count book-keeping
 *   to enforce the "≤ N_LEGS/2 in flight at once" stability rule
 *   (≥ 4 feet planted on an 8-legged spider = support polygon
 *   wide enough to keep the centre of mass over the contact patch).
 *
 *     STATE                  ACTION                       n_air delta
 *     ─────                  ──────                       ───────────
 *     OVERSTRETCHED          snap_overstretched_foot      -1 (lands)
 *     IN-FLIGHT              advance_swing                -1 if lands
 *     PLANTED + drift/strain maybe_trigger_step           +1 if takes off
 *
 *   The stability rule is the same constraint real arthropods
 *   follow — see Wilson "Insect Walking" 1966 and Cruse "What
 *   mechanisms coordinate leg movement" 1990.
 *
 *   After dispatch, run analytical 2-joint IK on every leg so the
 *   knee always matches the current foot position. is_left =
 *   (i%2==0) mirrors solve_ik's port/starboard sign convention.
 */
static void update_steps(Spider *sp, float dt)
{
    int n_air = count_airborne(sp);

    /* PHASE 1 — advance per-leg state machine */
    for (int i = 0; i < N_LEGS; i++) {
        if      (snap_overstretched_foot(sp, i))    n_air--;
        else if (sp->stepping[i]) {
            if (advance_swing(sp, i, dt))           n_air--;
        }
        else if (maybe_trigger_step(sp, i, n_air))  n_air++;
    }

    /* PHASE 2 — recompute knee positions from updated feet */
    for (int i = 0; i < N_LEGS; i++)
        solve_ik(sp->hip[i], sp->foot_pos[i], (i % 2 == 0), &sp->knee[i]);
}

/* ── §5h  rendering helpers ──────────────────────────────────────── */

/* head_glyph — directional arrow ('>' '<' '^' 'v') for heading (rad). */
static chtype head_glyph(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    if (deg <  45.0f || deg >= 315.0f) return (chtype)(unsigned char)'>';
    if (deg < 135.0f)                  return (chtype)(unsigned char)'v';
    if (deg < 225.0f)                  return (chtype)(unsigned char)'<';
    return                             (chtype)(unsigned char)'^';
}

/*
 * seg_glyph — best ASCII direction glyph for vector (dx, dy).
 * dy negated before atan2f so the angle matches visual direction.
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

/*
 * draw_leg_line — segmented-limb line for a leg segment.
 *
 * Alternates direction-glyph and '.' so each segment reads as a chain
 * (e.g., '-.-.-.-' horizontal, '|.|.|.|' vertical). This gives the
 * arthropod-limb feel that distinguishes legs from body bead-fill.
 */
static void draw_leg_line(WINDOW *w, Vec2 a, Vec2 b,
                          int pair, attr_t attr, int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph   = seg_glyph(dx, dy);
    int    nsteps  = (int)ceilf(len / DRAW_LEG_STEP_PX) + 1;
    int    prev_cx = -9999, prev_cy = -9999;
    int    phase   = 0;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        chtype ch = (phase & 1) ? (chtype)(unsigned char)'.' : glyph;
        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, ch);
        wattroff(w, COLOR_PAIR(pair) | attr);
        phase++;
    }
}

/*
 * draw_body_beads — stamp 'o' along the body centerline. Used for
 * body bead-fill (joint markers drawn on top in a second pass). Caller
 * supplies the dedup cursor so it persists across all body segments.
 */
static void draw_body_beads(WINDOW *w, Vec2 a, Vec2 b,
                            int pair, attr_t attr, int cols, int rows,
                            int *prev_cx, int *prev_cy)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == *prev_cx && cy == *prev_cy) continue;
        *prev_cx = cx; *prev_cy = cy;
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

/* ── §5i  render_spider ──────────────────────────────────────────── */

/* draw_legs — pass 1: femur and tibia direction-glyph lines for all 6 legs. */
static void draw_legs(const Spider *sp, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg_line(w, sp->hip[i],  sp->knee[i],     3, A_BOLD, cols, rows);
        draw_leg_line(w, sp->knee[i], sp->foot_pos[i], 3, A_BOLD, cols, rows);
    }
}

/* draw_leg_joints — pass 2: knee 'o' + foot ('*' planted, '.' airborne). */
static void draw_leg_joints(const Spider *sp, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        mark_cell(w, sp->knee[i], (chtype)(unsigned char)'o',
                  3, A_BOLD, cols, rows);

        if (sp->stepping[i])
            mark_cell(w, sp->foot_pos[i], (chtype)(unsigned char)'.',
                      7, A_DIM, cols, rows);
        else
            mark_cell(w, sp->foot_pos[i], (chtype)(unsigned char)'*',
                      6, A_BOLD, cols, rows);
    }
}

/* draw_body_lines — pass 3: bead fill along body centerline, tail→head.
 * Tail-to-head order means head-area glyphs win where the body
 * crosses itself in tight curves. */
static void draw_body_lines(const Spider *sp, WINDOW *w, int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = N_BODY_SEGS - 1; i >= 0; i--) {
        draw_body_beads(w, sp->body_joint[i + 1], sp->body_joint[i],
                        3, A_BOLD, cols, rows, &prev_cx, &prev_cy);
    }
}

/* draw_body_nodes — pass 4: chunky 'O' at the abdomen tip, slimmer 'o'
 * at mid-body joints. The size taper distinguishes the spider's body
 * from hexpod_tripod's uniform '+' rectangle markers. */
static void draw_body_nodes(const Spider *sp, WINDOW *w, int cols, int rows)
{
    for (int i = N_BODY_SEGS; i >= 1; i--) {
        chtype glyph = (i == N_BODY_SEGS)
                     ? (chtype)(unsigned char)'O'   /* abdomen tip */
                     : (chtype)(unsigned char)'o';  /* mid-body    */
        mark_cell(w, sp->body_joint[i], glyph, 3, A_BOLD, cols, rows);
    }
}

/*
 * draw_head — pass 5: eye cluster ': arrow :' at the head joint.
 *
 * Eyes sit at perpendicular offset (-sinθ, cosθ) · EYE_OFFSET_PX from
 * the head, so they always flank the directional arrow correctly:
 *   east/west motion → eyes above and below
 *   north/south motion → eyes left and right
 * The 3-glyph cluster is the spider's visual signature, distinct from
 * hexpod_tripod's plain '@' body centre.
 */
static void draw_head(const Spider *sp, WINDOW *w, int cols, int rows)
{
    Vec2  head   = sp->body_joint[0];
    float perp_x = -sinf(sp->heading) * EYE_OFFSET_PX;
    float perp_y =  cosf(sp->heading) * EYE_OFFSET_PX;

    Vec2 eye_l = { head.x - perp_x, head.y - perp_y };
    Vec2 eye_r = { head.x + perp_x, head.y + perp_y };

    /* Dim ':' eyes — read as eyes, not as solid markers. */
    mark_cell(w, eye_l, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);
    mark_cell(w, eye_r, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);

    /* Bold directional arrow dominates between the eyes. */
    mark_cell(w, head, head_glyph(sp->heading), 3, A_BOLD, cols, rows);
}

/*
 * render_spider — orchestrator. Painter's order back-to-front so node
 * markers always sit on top of line glyphs:
 *   legs (lines)     →  leg joints  →  body fill
 *   body nodes       →  head cluster
 */
static void render_spider(const Spider *sp, WINDOW *w, int cols, int rows)
{
    draw_legs       (sp, w, cols, rows);
    draw_leg_joints (sp, w, cols, rows);
    draw_body_lines (sp, w, cols, rows);
    draw_body_nodes (sp, w, cols, rows);
    draw_head       (sp, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — thin wrapper around Spider                                */
/* ===================================================================== */

/*
 * Scene — composition root for §6.
 *
 * Intent
 *   In this demo the simulation IS one spider. We keep a Scene
 *   wrapper anyway so the framework loop (scene_init / scene_tick /
 *   scene_draw) reads identically to every other file in the repo.
 *   If a future variant adds prey, web strands, or a swarm, they
 *   slot in here as siblings of `spider` without changing the loop.
 *
 * Simulation vs Rendering locality
 *   The Spider struct itself already separates Body / Legs / Steering
 *   / UI. Within Scene there is no further split needed yet — one
 *   field, one concern. When adding a new member, place it as
 *   follows:
 *
 *     ── Simulation state ──   things scene_tick READS+WRITES
 *                              (prey[], web_strands[], pheromones, …)
 *     ── Render-only state ──  things scene_draw READS, never the
 *                              physics path (camera, shake, FX layers)
 *
 * Things that DO NOT live here
 *   - fps / sim_fps counters → §8 App (frame-timing concern, not
 *     part of the world being simulated).
 *   - terminal extents (cols, rows) → §7 Screen.
 *   - signal flags (running, need_resize) → §8 App.
 *   - Per-leg geometry tables HIP_BODY_T, LEG_ANGLE → §5a
 *     (file-scope `static const`; never mutates).
 *
 * One Scene per program; passed by pointer to every §6 entry point.
 */
typedef struct {
    /* ── Simulation state ─────────────────────────────────────── */
    Spider spider;             /* the world: body + 8 legs + gait    */

    /* (no render-only state yet — theme_idx + paused live inside
     *  Spider because they're user-toggled in the same keymap)    */
} Scene;

/*
 * scene_init — place spider at screen centre with a pre-populated trail
 * (so the body is fully extended on frame 1) and feet planted at their
 * ideal rest positions.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Spider *sp = &sc->spider;

    sp->move_speed     = BODY_SPEED;
    sp->heading        = 0.0f;
    sp->target_heading = 0.0f;
    sp->hip_dist       = (float)(rows * CELL_H) * HIP_DIST_FACTOR;
    sp->paused         = false;
    sp->theme_idx      = 0;

    sp->body_joint[0].x = (float)(cols * CELL_W) * 0.50f;
    sp->body_joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /* Pre-fill trail one pixel at a time straight backward so the body
     * appears fully extended on the very first rendered frame. */
    float bx = cosf(sp->heading + (float)M_PI);
    float by = sinf(sp->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        sp->trail[k].x = sp->body_joint[0].x + (float)k * bx;
        sp->trail[k].y = sp->body_joint[0].y + (float)k * by;
    }
    sp->trail_head  = 0;
    sp->trail_count = TRAIL_CAP;

    compute_body_joints(sp);
    compute_hips(sp);

    /* Plant all feet at their ideal positions and solve IK. */
    for (int i = 0; i < N_LEGS; i++) {
        sp->foot_pos[i]    = compute_ideal_foot(sp, i);
        sp->foot_old[i]    = sp->foot_pos[i];
        sp->step_target[i] = sp->foot_pos[i];
        sp->stepping[i]    = false;
        sp->step_t[i]      = 0.0f;
        solve_ik(sp->hip[i], sp->foot_pos[i], (i % 2 == 0), &sp->knee[i]);
    }
}

/*
 * scene_tick — one variable-timestep simulation step. dt is wall-clock
 * delta scaled by the caller's time_scale.
 *
 * Order: steer, translate, body joints, hips, gait + IK. Each step
 * depends on the previous (hips need updated body, IK needs hips, etc).
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Spider *sp = &sc->spider;
    if (sp->paused) return;

    steer_heading      (sp, dt);
    translate_body     (sp, dt, cols, rows);
    compute_body_joints(sp);
    compute_hips       (sp);
    update_steps       (sp, dt);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    render_spider(&sc->spider, w, cols, rows);
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
 * References [14] Raymond, NCURSES Programming HOWTO.
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

/* heading_arrow — '>' '<' '^' 'v' for the HUD readout. */
static const char *heading_arrow(float heading)
{
    float deg = heading * (180.0f / (float)M_PI);
    while (deg <    0.0f) deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    if (deg <  45.0f || deg >= 315.0f) return ">";
    if (deg < 135.0f)                   return "v";
    if (deg < 225.0f)                   return "<";
    return                              "^";
}

/*
 * screen_draw — compose one full frame:
 *   erase → spider → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any theme.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Spider *sp = &sc->spider;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-SPIDER  dir:%s  spd:%.0f  theme:%s  %.2fx  %.1ffps  %s ",
             heading_arrow(sp->heading), sp->move_speed,
             THEMES[sp->theme_idx].name,
             time_scale, fps,
             sp->paused ? "PAUSED" : "crawling");

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  arrows:steer  w/s:speed  t:theme  [/]:time ");
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
 *   See [14] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Spider tuning values (move_speed, theme_idx) — user-state in
 *     §5 Spider.
 */
typedef struct {
    /* ── Owned subsystems ─────────────────────────────────────── */
    Scene  scene;              /* the world (§6)                       */
    Screen screen;             /* terminal extent (§7)                 */

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
 * Body position is clamped into the new bounds; hip_dist is recomputed
 * for the new screen height. Theme is re-applied because some
 * terminals re-derive COLORS on resize.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Spider *sp  = &app->scene.spider;
    float   wpx = (float)(app->screen.cols * CELL_W);
    float   hpx = (float)(app->screen.rows * CELL_H);
    if (sp->body_joint[0].x >= wpx) sp->body_joint[0].x = wpx - 1.0f;
    if (sp->body_joint[0].y >= hpx) sp->body_joint[0].y = hpx - 1.0f;
    sp->hip_dist = hpx * HIP_DIST_FACTOR;
    theme_apply(sp->theme_idx);
    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Arrow keys set target_heading; the body interpolates toward it at
 * TURN_RATE rad/s in steer_heading().
 */
static bool app_handle_key(App *app, int ch)
{
    Spider *sp = &app->scene.spider;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sp->paused = !sp->paused; break;

    case KEY_RIGHT: sp->target_heading =  0.0f;                break;
    case KEY_DOWN:  sp->target_heading =  (float)M_PI * 0.5f;  break;
    case KEY_LEFT:  sp->target_heading =  (float)M_PI;         break;
    case KEY_UP:    sp->target_heading = -(float)M_PI * 0.5f;  break;

    case 'w': case 'W':
        sp->move_speed *= 1.25f;
        if (sp->move_speed > BODY_SPEED_MAX) sp->move_speed = BODY_SPEED_MAX;
        break;
    case 's': case 'S':
        sp->move_speed /= 1.25f;
        if (sp->move_speed < BODY_SPEED_MIN) sp->move_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        sp->theme_idx = (sp->theme_idx + 1) % N_THEMES;
        theme_apply(sp->theme_idx);
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
