/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fk_ik_helloworld.c — three-link arm with two modes: FK (angles) and IK (target)
 *
 * DEMO: A 3-link arm — shoulder → elbow → wrist → hand — with two
 *       interactive modes that you toggle with 'm':
 *
 *         FK MODE   you control three joint angles directly.
 *                   Press a/d, w/s, z/x to rotate θ1, θ2, θ3.
 *                   Hand moves wherever the angles place it.
 *
 *         IK MODE   you move a target with the arrow keys, and
 *                   the solver bends the arm so the hand lands
 *                   on it. Press 'f' to flip the elbow side.
 *
 *           S ─────── E ─────── W ─────── H        + (IK only)
 *           @   L1    O   L2    o   L3    *
 *        shoulder  elbow     wrist     hand     target
 *         (fixed)
 *
 *       This is the third file in a graduated series:
 *         ik_helloworld.c  — 2-link IK   (law of cosines)
 *         fk_helloworld.c  — 2-link FK   (forward propagation)
 *         this file        — 3-link arm with BOTH modes
 *
 * The 3-link insight (why this matters)
 *   2-link IK is exact: triangle SET, two solutions (elbow up/down).
 *   3-link IK is UNDERDETERMINED — infinitely many configurations
 *   reach any target. To pick one we have to add a constraint.
 *   This file picks the simplest possible: the wrist always points
 *   AT the target. That fixes the third link's orientation and
 *   reduces the rest to plain 2-link IK on a virtual target one
 *   link's length back along the S→T direction.
 *
 *   This 2+1 split — IK for the gross "reach", FK-like aim for the
 *   final link — is exactly what real animation rigs do. IK puts
 *   the wrist where it needs to be; FK controls the hand
 *   orientation, finger curl, and so on.
 *
 * Symbol meanings — see how '*' and '+' partition input vs output:
 *
 *               FK mode                 IK mode
 *     '@'  S    fixed (input config)    fixed (input config)
 *     'O'  E    computed (FK output)    computed (IK output)
 *     'o'  W    computed (FK output)    computed (IK output)
 *     '*'  H    computed (FK output)    computed (IK output)
 *     '+'  T    not drawn               drawn — user input
 *
 * Study alongside:
 *   animation/ik_helloworld.c    — 2-link IK
 *   animation/fk_helloworld.c    — 2-link FK
 *   animation/ik_arm_reach.c     — FABRIK on a 4-link arm
 *
 * Section map:
 *   §1  config       — every tunable in one place
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-link / per-marker pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  ik_fk        — Vec2 + compute_fk + solve_ik2 + solve_ik3
 *   §6  scene        — Scene state, input dispatch, draw
 *   §7  screen       — ncurses init / present / HUD
 *   §8  app          — signals, resize, main loop
 *
 * Keys (always active):
 *   q / ESC      quit
 *   space        pause / resume input
 *   r            reset angles & target to defaults
 *   m            toggle mode (FK ↔ IK)
 *
 * Keys (FK mode):
 *   a / d        rotate θ1 (shoulder)
 *   w / s        rotate θ2 (elbow,  relative to upper arm)
 *   z / x        rotate θ3 (wrist,  relative to forearm)
 *
 * Keys (IK mode):
 *   ↑ ↓ ← →      move target T
 *   f            flip elbow to OTHER valid 2-link solution
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/fk_ik_helloworld.c \
 *       -o fk_ik_helloworld -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two solvers, one toggle.
 *
 *                  FK : forward propagation of cumulative angles.
 *                       E = S + L1·(cos θ1,         sin θ1)
 *                       W = E + L2·(cos(θ1+θ2),     sin(θ1+θ2))
 *                       H = W + L3·(cos(θ1+θ2+θ3),  sin(θ1+θ2+θ3))
 *                       O(1), no branching.
 *
 *                  IK : 2+1 hybrid. Three steps:
 *                       (1) shrink the target by L3 along S→T to
 *                           get a virtual 2-link target T2;
 *                       (2) run plain 2-link analytical IK
 *                           (law of cosines) from S to T2 to get
 *                           E and W;
 *                       (3) place H at L3 from W, pointing at T.
 *                       Result: hand lands on T, wrist points at T,
 *                       and the arm has a unique configuration up
 *                       to the elbow-up/down flip.
 *
 * Data-structure : One Scene struct holding the fixed shoulder,
 *                  the three FK angles, the IK target, the
 *                  elbow_up flag, the current Mode (FK or IK), and
 *                  the three cached output positions (elbow, wrist,
 *                  hand). The two solvers are pure functions
 *                  returning an ArmPose; the main loop chooses one
 *                  based on Mode and writes the result back into
 *                  the Scene cache so renderer and HUD read it
 *                  without recomputing.
 *
 * Rendering      : Three coloured lines (S→E cyan, E→W orange,
 *                  W→H magenta) plus four arm markers — '@'
 *                  shoulder, 'O' elbow, 'o' wrist, '*' hand —
 *                  plus a '+' target marker drawn ONLY in IK mode.
 *                  Lines first, markers on top, '@' last so the
 *                  pinned shoulder always wins the cell contest.
 *
 * Performance    : O(1) per frame in either mode. FK does six
 *                  trig calls; IK does one acos, one atan2, two
 *                  cosf/sinf, plus a couple of normalisations.
 *                  Microseconds.
 *
 * Why a hybrid IK and not a full 3-link solver?
 *   Full 3-link analytical IK is genuinely hard — the solution
 *   set is a 1-parameter family (the "redundancy"), not a discrete
 *   pair. Iterative solvers like FABRIK handle it cleanly but at
 *   the cost of an iteration loop. The 2+1 hybrid pins the third
 *   link's direction (always points at the target) so the
 *   redundancy collapses and the rest becomes the 2-link analytical
 *   IK we already understand. It is the simplest design that
 *   teaches "more joints = more ambiguity = pick a strategy".
 *
 * References
 * ──────────
 *   ── Canonical robotics textbooks (FK + IK side-by-side) ──────────
 *   [1] Craig, J. J. (2005), "Introduction to Robotics: Mechanics
 *       and Control" (3rd ed.), Pearson — the standard textbook;
 *       Ch. 3 (FK), Ch. 4 (IK casework). The 2-link analytical IK
 *       in §5 solve_ik2 follows Craig's law-of-cosines derivation
 *       directly.
 *   [2] Spong, M. W., Hutchinson, S. & Vidyasagar, M. (2005),
 *       "Robot Modeling and Control", Wiley — alternative canonical
 *       text; Ch. 3 (FK), Ch. 4 (IK) cover the same ground with
 *       cleaner notation.
 *   [3] Murray, R. M., Li, Z. & Sastry, S. S. (1994), "A Mathematical
 *       Introduction to Robotic Manipulation", CRC — rigorous
 *       Lie-group treatment; read once you want to see FK/IK as
 *       SE(2)/SE(3) maps.
 *
 *   ── Inverse kinematics: solver families ──────────────────────────
 *   [4] Buss, S. R. (2004), "Introduction to Inverse Kinematics with
 *       Jacobian Transpose, Pseudoinverse and Damped Least Squares",
 *       UCSD course notes — the Jacobian-iterative family of
 *       solvers, contrast with the analytical 2-link IK used here.
 *   [5] Aristidou, A. & Lasenby, J. (2011), "FABRIK: a fast,
 *       iterative solver for the inverse kinematics problem",
 *       Graphical Models 73(5), pp. 243-260 — the iterative
 *       alternative to the 2+1 hybrid used here; FABRIK scales
 *       cleanly to many-link chains where analytical IK does not.
 *   [6] Tolani, D., Goswami, A. & Badler, N. I. (2000), "Real-time
 *       inverse kinematics techniques for anthropomorphic limbs",
 *       Graphical Models 62(5), pp. 353-388 — surveys closed-form
 *       solutions for 3-DOF chains; the 2+1 hybrid is one entry.
 *
 *   ── Formal notation (multi-joint scaling) ────────────────────────
 *   [7] Denavit, J. & Hartenberg, R. S. (1955), "A kinematic
 *       notation for lower-pair mechanisms based on matrices",
 *       J. Appl. Mech. 22, pp. 215-221 — the DH formalism every
 *       multi-link FK algorithm follows.
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *   [8] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com
 *       — when fixed-step matters; we use variable-step because
 *       both solvers are unconditionally stable.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *   [9] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       integer line drawing; we use the parametric oversample
 *       (draw_line_px in §6) because float math is already paid for.
 *  [10] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the high-contrast glyph choice ('@', 'O', 'o', '*', '+').
 *  [11] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the newscr/curscr diff pipeline.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [12] https://en.wikipedia.org/wiki/Forward_kinematics
 *  [13] https://en.wikipedia.org/wiki/Inverse_kinematics
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Same arm, two ways to tell it what to do.
 *
 *   FK ("puppeteer mode"): you rotate every joint by hand. The
 *     hand follows wherever the math says. Total control over
 *     pose, no goal.
 *
 *   IK ("goal mode"): you point at a spot in space. The arm
 *     contorts itself to put the hand there. Total control over
 *     destination, no direct say in the joint angles.
 *
 * These are the two halves of a real animation system. FK is what
 * a 3-D modeller uses to pose a character frame by frame; IK is
 * what a game uses to make the character's hand grab a doorknob.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Sit at a desk and try both modes with your own arm.
 *
 *   FK : focus on your shoulder, elbow, wrist one at a time.
 *        Rotate one joint, leave the other two. Watch where your
 *        hand ends up — wherever it goes, it goes. You did not
 *        steer it; you steered the JOINTS.
 *
 *   IK : pick a coffee cup. Reach for it. You commanded one thing:
 *        the cup. Your shoulder, elbow, and wrist all bent
 *        automatically. You did not feel any of those bends as
 *        deliberate decisions; they were all consequences.
 *
 * FK is the easy direction: angles → positions, one matrix
 * multiplication. IK is the hard inverse: positions → angles, a
 * problem with multiple solutions in general. The 2+1 hybrid in
 * §5 is the simplest answer to the 3-link version.
 *
 * GEOMETRY
 * ────────
 *   shoulder    @        ← S, fixed
 *                ╲ θ1
 *                 ╲  L1
 *                  ╲
 *           elbow   O    ← E
 *                   ╲ θ2 (relative to upper arm)
 *                    ╲ L2
 *                     ╲
 *           wrist      o ← W
 *                       ╲ θ3 (relative to forearm)
 *                        ╲ L3
 *                         ╲
 *                hand      *   ← H
 *                              ← + (only in IK mode: the target T)
 *
 * INVARIANTS
 * ──────────
 *   • In IK mode, T is clamped to the reach disc of radius
 *     L1 + L2 + L3 around S, so |T − S| ≤ L1 + L2 + L3 always
 *     and the hybrid solver always succeeds.
 *
 *   • Angles in FK mode are wrapped into (−π, π] every frame —
 *     cosmetic only; the trig calls don't care.
 *
 * FK ALGORITHM
 * ────────────
 *   E = S + L1·(cos θ1,         sin θ1)
 *   W = E + L2·(cos(θ1+θ2),     sin(θ1+θ2))
 *   H = W + L3·(cos(θ1+θ2+θ3),  sin(θ1+θ2+θ3))
 *
 * Each link inherits the cumulative angle of its predecessor
 * — that is what makes FK a CHAIN. Read compute_fk in §5
 * line-by-line; it matches.
 *
 * IK ALGORITHM (the 2+1 hybrid)
 * ─────────────────────────────
 *   (1) Compute a virtual 2-link target by pulling T back along
 *       the S→T direction by L3:
 *           dir_st = (T − S) / |T − S|
 *           T2     = T − L3 · dir_st
 *
 *   (2) Solve plain 2-link IK for the segment S → E → W with W
 *       landing on T2. Use the law of cosines exactly as in
 *       ik_helloworld.c:
 *           cos α       = (L1² + d² − L2²) / (2·L1·d)   where d = |T2 − S|
 *           φ           = atan2(T2.y − S.y, T2.x − S.x)
 *           side        = elbow_up ? −1 : +1
 *           elbow_angle = φ + side · α
 *           E           = S + L1·(cos elbow_angle, sin elbow_angle)
 *           W           = E + L2·normalize(T2 − E)        // = T2 if reachable
 *
 *   (3) Aim the third link from W toward the REAL target T:
 *           dir_wt = (T − W) / |T − W|
 *           H      = W + L3 · dir_wt
 *
 * When |T − S| ≤ L1 + L2 + L3 (which our clamp guarantees) step 2
 * always succeeds and W lands exactly on T2; substituting back
 * gives H = T exactly. The hand always reaches the target.
 *
 * KEY FORMULAS
 * ────────────
 *   FK link step    : tip = base + L · (cos θ_total, sin θ_total)
 *                     where θ_total is the SUM of all upstream
 *                     joint angles. This is THE FK primitive,
 *                     applied once per link.
 *
 *   Law of cosines  : cos α = (L1² + d² − L2²) / (2·L1·d)
 *                     used inside step 2 of the IK hybrid.
 *
 *   Reach disc      : R = L1 + L2 + L3. clamp_to_reach forces the
 *                     IK target onto this disc; outside it, the
 *                     hybrid solver would still produce something
 *                     plausible (2-link IK clipping at the rim)
 *                     but |H − T| would be non-zero.
 *
 *   Wrist invariant : the third link always points from W toward
 *                     T, so the angle the wrist makes with the
 *                     forearm IS NOT free — it is locked by the
 *                     2+1 design choice. That single design
 *                     choice is what kills the 1-parameter
 *                     redundancy of full 3-link IK.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • Target on top of the shoulder (|T − S| ≈ 0). The S→T
 *     direction is undefined; we substitute a fallback unit vector
 *     so dir_st stays sane and the chain doesn't NaN out.
 *
 *   • Target on top of the wrist after step 2 (|T − W| ≈ 0).
 *     Same fallback in step 3 keeps the third link visible.
 *
 *   • acos input drift on boundary configurations. The cosine
 *     value is clamped to [−1, 1] before the acos call.
 *
 *   • Mode toggle while paused. Allowed — switching modes is a
 *     "structural" change, not a continuous nudge. Ditto 'r'
 *     reset and 'f' elbow flip.
 *
 *   • Resize. SIGWINCH re-anchors S to the new screen centre and
 *     re-clamps T into both the new bounds and the reach disc.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press 'r'. Default state: FK mode, all angles zero, arm
 *     extended horizontally to the right. Hand at distance
 *     L1+L2+L3 from the shoulder.
 *
 *   • Hold 'd' (FK mode). θ1 increases; the entire arm rotates
 *     around the shoulder rigidly because θ2 and θ3 are zero.
 *     Both elbow and wrist trace concentric arcs.
 *
 *   • In FK mode, set θ1 = θ2 = θ3 = 0 (press 'r'), then hold 's'.
 *     θ2 increases; only the FOREARM and beyond rotate. The
 *     upper arm stays put.
 *
 *   • Press 'm' to switch to IK mode. The target '+' appears.
 *     Move the target and watch the arm contort — but the wrist
 *     always POINTS at the target, because of step 3 of the
 *     hybrid.
 *
 *   • Press 'f' in IK mode. The elbow snaps to the OTHER valid
 *     2-link solution, mirrored across S→T2. The wrist position
 *     stays on T2 and the hand stays on T.
 *
 *   • Drag the target far from the shoulder. clamp_to_reach
 *     stops it at distance L1+L2+L3 (= reach). The HUD's
 *     |H−T| readout stays at 0 because the hand follows the
 *     clamped target exactly.
 *
 *   • Sanity check: in FK mode the cell distances S→E, E→W, W→H
 *     should be constant (≈10 cells each). In IK mode the same
 *     three distances should still be constant — link lengths
 *     never change, no matter which solver computed the pose.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read fk_helloworld.c AND ik_helloworld.c first;
 *      this file is the 3-link version with a switchable solver,
 *      so it presupposes both halves of the 2-link case.
 *   2. §5 ik_fk — TWO solvers + a target clamp. Read AFTER tutorials
 *      T1-T6:
 *        compute_fk     (3-link FK propagation, 6 trig calls)
 *        solve_ik3      (the 2+1 hybrid)
 *        clamp_to_reach (keeps the IK target inside the reach disc)
 *   3. §6 scene — input dispatcher: which key handler runs depends
 *      on the current Mode. Mode toggle is 'm'.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if you've seen the
 *      framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   theta1, 2, 3       FK joint angles. theta1 absolute; theta2 and
 *                      theta3 RELATIVE to their predecessor link.
 *   L1, L2, L3         link lengths. Constant.
 *   S, E, W, H         shoulder, elbow, wrist, hand (Vec2). S fixed.
 *   T                  target (Vec2) — IK INPUT only.
 *   T2                 virtual 2-link target inside the IK hybrid:
 *                      T pulled back by L3 along the S→T direction.
 *   elbow_up           bool, picks the 2-link branch (elbow above or
 *                      below the S→T2 line).
 *   Mode               enum { MODE_FK, MODE_IK }; toggled by 'm'.
 *
 * Background you need
 * ───────────────────
 *   - Both helloworld solvers (fk_helloworld T2-T3 and
 *     ik_helloworld T3-T5).
 *   - Vector normalisation: dir = v / |v|.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - General N-link IK (FABRIK, Jacobian). The 2+1 hybrid here
 *     is a clean way to TEACH 3-link IK without those iterative
 *     solvers; ik_arm_reach.c covers FABRIK on a longer chain.
 *   - Quaternions / rotation matrices. We're still in 2-D.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that progress from the 2-link case into 3-link IK
 * via the simplest design choice that breaks the redundancy.
 *
 *   T1  Two modes, one arm — FK and IK as DUAL viewpoints
 *   T2  3-link FK is just 2-link FK with one more step
 *   T3  3-link IK is UNDERDETERMINED — what changes
 *   T4  The 2+1 hybrid — pin the wrist direction
 *   T5  Why the 2+1 split mirrors real animation rigs
 *   T6  When the hybrid is wrong (and what to do then)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  TWO MODES, ONE ARM — FK AND IK AS DUAL VIEWPOINTS
 * ─────────────────────────────────────────────────────
 * One geometry: shoulder + 3 links + 4 markers. TWO ways to
 * specify a pose:
 *
 *   FK ("puppeteer"):  user gives you angles. You compute positions.
 *                      Always succeeds. One unique answer.
 *
 *   IK ("goal"):       user gives you a target. You compute angles.
 *                      Can fail if unreachable. Multiple answers if
 *                      it succeeds.
 *
 * Same struct holds the result either way:
 *
 *     struct ArmPose { Vec2 elbow, wrist, hand; }
 *
 * Whichever solver runs writes into that struct; the renderer
 * doesn't know which one ran. That's the LESSON of having both
 * modes in one file: the OUTPUT representation is the same. FK
 * and IK are just different ways to ARRIVE at the same pose.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │   FK input         shared output     IK input    │
 *      │                                                  │
 *      │   θ1, θ2, θ3   →   ArmPose       ←   target T    │
 *      │     ↑                                  ↑         │
 *      │  arrow keys                       arrow keys     │
 *      └──────────────────────────────────────────────────┘
 *
 * Toggling 'm' switches which input pipeline is alive. The arm
 * keeps drawing through the same code regardless.
 *
 * T2  3-LINK FK IS JUST 2-LINK FK WITH ONE MORE STEP
 * ──────────────────────────────────────────────────
 * The FK primitive (fk_helloworld T2) generalises directly:
 *
 *     E = S + L₁ · (cos θ₁,            sin θ₁)
 *     W = E + L₂ · (cos(θ₁+θ₂),        sin(θ₁+θ₂))
 *     H = W + L₃ · (cos(θ₁+θ₂+θ₃),     sin(θ₁+θ₂+θ₃))
 *
 * Three lines, six trig calls. Notice the cumulative angle —
 * each link sees the SUM of all previous joint angles. That's
 * why fk_centipede.c can extend this to 30 segments: just keep
 * adding link steps, each with one more angle in the sum.
 *
 * The compute_fk in §5 mirrors these three lines exactly.
 *
 * T3  3-LINK IK IS UNDERDETERMINED — WHAT CHANGES
 * ───────────────────────────────────────────────
 * Pulling the same trick from ik_helloworld T3 — three sides of
 * a triangle — does NOT generalise. With three links there's no
 * single triangle; there's a quadrilateral S → E → W → H, and
 * the user has only specified two corners (S, T = H).
 *
 * Counting unknowns and equations:
 *
 *     unknowns:  E.x, E.y, W.x, W.y           (4 numbers)
 *     equations: |E−S| = L₁                   (1)
 *                |W−E| = L₂                   (1)
 *                |H−W| = L₃ AND H = T          (3 — H pinned)
 *
 * Wait — let's count carefully. H is given (= T). We need E and W
 * (4 unknowns). We have:
 *     |E−S| = L₁          1 equation
 *     |W−E| = L₂          1 equation
 *     |T−W| = L₃          1 equation
 *
 * 3 equations, 4 unknowns. ONE DEGREE OF FREEDOM is unconstrained
 * — there's a 1-parameter family of poses where the hand lands
 * on T.
 *
 * Visually: with two links the elbow had only two valid spots
 * (up/down). With three links the elbow can slide along an
 * entire CURVE, with the wrist adjusting to keep the hand on T.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │  three valid 3-link poses for the SAME target:   │
 *      │                                                  │
 *      │      O──o            O──o──*                     │
 *      │     ╱    ╲          ╱                            │
 *      │    @      *        @                             │
 *      │                                                  │
 *      │           O                                      │
 *      │          ╱╲                                      │
 *      │         @  o                                     │
 *      │             ╲                                    │
 *      │              *                                   │
 *      └──────────────────────────────────────────────────┘
 *
 * To pick ONE pose, the solver needs an EXTRA CONSTRAINT.
 *
 * T4  THE 2+1 HYBRID — PIN THE WRIST DIRECTION
 * ────────────────────────────────────────────
 * The simplest extra constraint: "the third link always points
 * AT the target." That fixes the third link's orientation,
 * which kills the redundancy.
 *
 * Geometric construction:
 *
 *   1. The wrist W must be exactly L₃ away from T, on the S→T
 *      ray. So W = T − L₃ · normalize(T − S).
 *      Call that point T₂ — the "virtual 2-link target."
 *
 *   2. With W = T₂ pinned, the segment S → E → W is a 2-LINK arm
 *      with target T₂. Run plain 2-link IK exactly as in
 *      ik_helloworld (law of cosines).
 *
 *   3. Place H = W + L₃ · normalize(T − W) — this just confirms
 *      H lands on T, since W is L₃ along the S→T direction
 *      already.
 *
 * 2+1 split, hence "the 2+1 hybrid." Step 1 is one normalisation
 * + one subtraction. Step 2 is the existing 2-link solver. Step 3
 * is one normalisation + one addition.
 *
 * The solver is exactly as expensive as 2-link IK plus a few
 * extra ops. No iteration loop, no convergence check.
 *
 * Cost of the design choice: the wrist NEVER bends sideways
 * relative to the forearm. The user can't tell the arm to "wave"
 * via the IK target. They'd need to enter FK mode and rotate θ₃
 * directly.
 *
 * T5  WHY THE 2+1 SPLIT MIRRORS REAL ANIMATION RIGS
 * ─────────────────────────────────────────────────
 * "IK for the gross reach, FK for the final orientation" is a
 * standard pattern in production rigs. Examples:
 *
 *   - Reaching for a doorknob: IK targets the doorknob; FK
 *     controls the wrist twist that rotates your hand to grip.
 *
 *   - Holding a sword: IK puts the wrist where the user wants;
 *     FK controls the blade angle independently.
 *
 *   - Walking: IK pins the foot to a contact point on the
 *     ground; FK animates the toes / ankle roll.
 *
 * Real rigs use ARMS THAT END at the wrist for IK, and then a
 * FK chain from wrist to fingertip. Same shape as our 2+1.
 *
 * The 2+1 hybrid in this file is the SIMPLEST INSTANCE of that
 * pattern: one IK joint pair (elbow), one FK aim direction
 * (wrist→hand). Adding more FK joints past the wrist just keeps
 * extending the FK chain.
 *
 * T6  WHEN THE HYBRID IS WRONG (AND WHAT TO DO THEN)
 * ──────────────────────────────────────────────────
 * The 2+1 hybrid produces ONE specific pose for any reachable
 * target. Sometimes that pose is wrong:
 *
 *   - You wanted the elbow tucked in (pose touching the body),
 *     but the hybrid gave you elbow-out. The 'f' flip handles
 *     this — switch to the OTHER 2-link branch.
 *
 *   - You wanted the wrist bent (e.g. a salute pose). The hybrid
 *     locks wrist→hand to the S→T direction; you can't ask for
 *     a bent wrist via IK alone. Switch to FK or compose.
 *
 *   - You wanted the arm to AVOID an obstacle — slide the elbow
 *     along its valid curve to thread between obstacles. The
 *     hybrid doesn't expose that DOF; you'd need a real
 *     redundant-IK solver (FABRIK, Jacobian + null-space).
 *
 *   - You wanted the elbow to track a SECONDARY GOAL ("look
 *     elegant"). Same answer — needs a redundant solver.
 *
 * For longer chains (4+ links) the redundancy becomes too
 * useful to ignore, and animation/ moves to FABRIK:
 *
 *   ik_arm_reach.c           4-link arm with FABRIK
 *   ik_tentacle_seek.c       6+ link tentacle with FABRIK
 *   snake_inverse_kinematics.c  long chain with FABRIK
 *
 * FABRIK is the natural successor: like the hybrid here, it
 * picks a pose without requiring you to specify every joint;
 * unlike the hybrid, the choice is informed by the previous
 * frame's pose, giving smooth animation. Read it next.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    TARGET_FPS = 60,

    /* ncurses pair IDs. PAIR_HUD/PAIR_HINT carry the project HUD
     * convention (bright yellow / bright cyan, both A_BOLD). */
    PAIR_HUD          = 1,
    PAIR_HINT         = 2,
    PAIR_LINK_UPPER   = 3,
    PAIR_LINK_FORE    = 4,
    PAIR_LINK_HAND    = 5,
    PAIR_JOINT        = 6,
    PAIR_SHOULDER     = 7,
    PAIR_HAND         = 8,
    PAIR_TARGET       = 9,
};

/* Three equal-length links → maximum reach 3·L = 240 px = 30 cells.
 * Equal lengths keep both solvers' edge cases trivial: any d in
 * [0, 3·L] is reachable by the IK hybrid, and FK can fold the arm
 * back onto itself when all three angles point opposite ways. */
#define L1_PX  80.0f                  /* upper arm length */
#define L2_PX  80.0f                  /* forearm  length */
#define L3_PX  80.0f                  /* hand link length */

/* Angle conversions. */
#define DEG_TO_RAD      ((float)M_PI / 180.0f)
#define RAD_TO_DEG      (180.0f / (float)M_PI)

/* 2° per FK keypress — comfortable sweep at typical key repeat. */
#define ANGLE_STEP_RAD  (2.0f * DEG_TO_RAD)

/* 4 px per IK arrow keypress = half a cell on the wide axis. */
#define KEY_STEP_PX     4.0f

/* Initial IK target — 70% of full reach, on the rest line. */
#define INITIAL_TARGET_REACH_FRAC  0.7f

/* Terminal cell dimensions — the aspect-ratio bridge. CELL_W=8
 * sub-pixels per column, CELL_H=16 per row. All math is in pixel
 * space (isotropic); cell conversion happens only at draw time. */
#define CELL_W   8
#define CELL_H  16

#define NS_PER_SEC 1000000000LL

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
 * One fixed semantic colour per visual role:
 *   upper arm     cyan     (L1 — closest to shoulder)
 *   forearm       orange   (L2 — middle link)
 *   hand link     magenta  (L3 — closest to hand)
 *   joints        white    (elbow, wrist — solver output)
 *   shoulder      lime     (anchor — never moves)
 *   hand          red      (end of the chain)
 *   target        gold     (only drawn in IK mode)
 */
static void color_init(void)
{
    if (!has_colors()) return;
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,         226, -1);
        init_pair(PAIR_HINT,         51, -1);
        init_pair(PAIR_LINK_UPPER,   45, -1);
        init_pair(PAIR_LINK_FORE,   208, -1);
        init_pair(PAIR_LINK_HAND,   201, -1);
        init_pair(PAIR_JOINT,       255, -1);
        init_pair(PAIR_SHOULDER,    118, -1);
        init_pair(PAIR_HAND,        196, -1);
        init_pair(PAIR_TARGET,      220, -1);
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_UPPER,  COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_FORE,   COLOR_RED,     -1);
        init_pair(PAIR_LINK_HAND,   COLOR_MAGENTA, -1);
        init_pair(PAIR_JOINT,       COLOR_WHITE,   -1);
        init_pair(PAIR_SHOULDER,    COLOR_GREEN,   -1);
        init_pair(PAIR_HAND,        COLOR_RED,     -1);
        init_pair(PAIR_TARGET,      COLOR_YELLOW,  -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio bridge                           */
/* ===================================================================== */

/*
 * Positions live in square pixel space. These helpers convert to
 * cell coordinates only at draw time, undoing the 8:16 cell aspect
 * ratio. Doing geometry in pixel space means perpendicular vectors
 * are genuinely perpendicular — a property that would break in
 * cell space.
 */
static inline int   px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int   px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }
static inline float cells_to_px_w(int cols){ return (float)cols * CELL_W; }
static inline float cells_to_px_h(int rows){ return (float)rows * CELL_H; }

/* ===================================================================== */
/* §5  ik_fk — Vec2 + compute_fk + solve_ik2 + solve_ik3                  */
/* ===================================================================== */

/*
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   Everything in §5 (FK + IK solvers) does arithmetic in pixel space,
 *   not character cells. Each cell is CELL_W × CELL_H sub-pixels
 *   (8 × 16 by default), so a swinging arm reads as continuous motion
 *   instead of jumping cell-to-cell. The conversion to cell space
 *   happens at exactly ONE place — px_to_cell_x/y in §6 draw_line_px
 *   — per the project's "one conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → down the screen).
 *
 *   Angles are MATH-CONVENTION (positive CCW around +X), so a CCW
 *   rotation in the math becomes CW on screen because y points DOWN.
 *   acosf in solve_ik2 yields the same angle either way — only the
 *   HUD readout ever needs to think about it.
 *
 * Why a value type (not a pointer)
 *   Sized 8 bytes; passed in registers on every modern ABI. The
 *   solvers return ArmPose / TwoLinkPose BY VALUE — -O2 lowers the
 *   call to the same straight-line code as if the caller wrote the
 *   trig inline. Encapsulation costs nothing here.
 *
 * Why not just two loose floats
 *   Named fields catch a whole class of bugs at compile time:
 *   passing (col, row) where (x, y) is expected becomes a type error
 *   rather than a visual debugging session.
 *
 * References [1] Craig §2 "Spatial descriptions"; [3] Murray et al.
 *   for the formal R² convention.
 */
typedef struct {
    float x;   /* eastward  pixel coordinate (positive → right) */
    float y;   /* southward pixel coordinate (positive → down)  */
} Vec2;

static inline Vec2  vec2(float x, float y)    { return (Vec2){x, y}; }
static inline Vec2  vec2_add(Vec2 a, Vec2 b)  { return (Vec2){a.x+b.x, a.y+b.y}; }
static inline Vec2  vec2_sub(Vec2 a, Vec2 b)  { return (Vec2){a.x-b.x, a.y-b.y}; }
static inline Vec2  vec2_scl(Vec2 a, float s) { return (Vec2){a.x*s, a.y*s}; }
static inline float vec2_len(Vec2 v)          { return sqrtf(v.x*v.x + v.y*v.y); }
static inline float vec2_dist(Vec2 a, Vec2 b) { return vec2_len(vec2_sub(a, b)); }

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/*
 * vec2_normalize_or — unit vector, with a fallback for the
 * degenerate zero-length input. Used wherever the IK solvers
 * divide by a length that could momentarily collapse to zero.
 */
static inline Vec2 vec2_normalize_or(Vec2 v, Vec2 fallback)
{
    float len = vec2_len(v);
    if (len < 1e-6f) return fallback;
    return vec2_scl(v, 1.0f / len);
}

/* wrap_pi — fold an angle into (−π, π]. Cosmetic only — keeps the
 * HUD readout stable under sustained key holds. */
static inline float wrap_pi(float a)
{
    while (a >  (float)M_PI) a -= 2.0f * (float)M_PI;
    while (a < -(float)M_PI) a += 2.0f * (float)M_PI;
    return a;
}

/*
 * ArmPose — the three computed joint positions of one arm evaluation.
 *
 * Intent
 *   Both solvers (compute_fk in FK mode, solve_ik3 in IK mode) return
 *   the SAME shape, so the main loop can dispatch on Mode and write
 *   to one cache:
 *
 *       ArmPose pose = (scene.mode == MODE_FK)
 *                    ? compute_fk(S, θ1, θ2, θ3)
 *                    : solve_ik3(S, T, elbow_up);
 *       scene.elbow = pose.elbow; scene.wrist = pose.wrist; scene.hand = pose.hand;
 *
 *   This is the file's TEACHING POINT: FK and IK are two algorithms
 *   that produce the same RECORD (joint positions) from different
 *   INPUTS (angles vs target). Same output schema, opposite data flow.
 *
 * Why ONE struct for both solvers
 *   If FK and IK returned different shapes, the call site would need
 *   to branch on Mode TWICE (once to pick the solver, once to read
 *   the result). One shared output type collapses the second branch
 *   into a single set of field assignments.
 *
 * Field origin (in pixel space)
 *   elbow : E — joint 1, derived from S and the first angle/length pair
 *   wrist : W — joint 2, derived from E and the second angle/length pair
 *   hand  : H — joint 3 (end-effector), derived from W and the third
 *
 *   In FK these are sequential cosf/sinf walks of cumulative angles
 *   (θ1, θ1+θ2, θ1+θ2+θ3). In IK they fall out of the 2+1 hybrid
 *   (Step 2 places E and W via law of cosines; Step 3 aims H at T).
 *   See CONCEPTS §"Algorithm" and [1] Craig §3-§4.
 */
typedef struct {
    Vec2 elbow;   /* E — first  joint, end of upper arm (L1 from S) */
    Vec2 wrist;   /* W — second joint, end of forearm   (L2 from E) */
    Vec2 hand;    /* H — third  joint / end-effector    (L3 from W) */
} ArmPose;

/*
 * compute_fk — three-link forward kinematics.
 *
 * Each link inherits the cumulative angle of its predecessors:
 * the upper arm uses θ1, the forearm uses θ1+θ2, and the hand link
 * uses θ1+θ2+θ3. Walk L away from the previous joint along that
 * cumulative direction and you land on the next joint.
 *
 * Pure function — no branches, no iteration, six trig calls.
 */
static ArmPose compute_fk(Vec2 S, float th1, float th2, float th3)
{
    float a1 = th1;
    float a2 = th1 + th2;
    float a3 = th1 + th2 + th3;

    Vec2 E = vec2(S.x + L1_PX * cosf(a1), S.y + L1_PX * sinf(a1));
    Vec2 W = vec2(E.x + L2_PX * cosf(a2), E.y + L2_PX * sinf(a2));
    Vec2 H = vec2(W.x + L3_PX * cosf(a3), W.y + L3_PX * sinf(a3));

    return (ArmPose){E, W, H};
}

/*
 * TwoLinkPose — internal return type of solve_ik2.
 *
 * Intent
 *   solve_ik2 is the analytical 2-link IK kernel — the same law of
 *   cosines you would use for ik_helloworld.c. It returns BOTH the
 *   computed elbow position AND the 2-link tip:
 *
 *     elbow : E — solved by the law of cosines from S, T, L1, L2.
 *     tip   : W — placed at L2 from E along E→T. EQUALS T exactly
 *                 when T is reachable (within [|L1-L2|, L1+L2] of S);
 *                 otherwise points AT T but stays at length L2 from E.
 *
 *   Why a struct and not two out-parameters? solve_ik3 needs BOTH
 *   the elbow (to draw S→E) AND the tip (to place the third link's
 *   base); returning by value bundles them into one assignment.
 *
 * Why "tip" instead of "wrist"
 *   In the 2-link sub-problem there is no wrist — the second segment
 *   ENDS at the tip. The name `tip` keeps the 2-link kernel
 *   independent of the 3-link context that consumes it. solve_ik3
 *   THEN treats this tip as the wrist of the larger arm.
 *
 * Why this is a separate type from ArmPose
 *   ArmPose has three joints (E, W, H); TwoLinkPose has two (E, tip).
 *   Conflating them would force the 2-link kernel to invent a fake
 *   third joint, hiding the algorithm. Two types make the dataflow
 *   explicit: solve_ik2 → TwoLinkPose, solve_ik3 → ArmPose.
 *
 * References [1] Craig §4.4 "Solvability"; the law-of-cosines
 *   derivation is in [2] Spong et al. §4.2.
 */
typedef struct {
    Vec2 elbow;   /* E — law-of-cosines solution at the shoulder vertex */
    Vec2 tip;    /* W — end of the second link; = T when reachable     */
} TwoLinkPose;

/*
 * solve_ik2 — analytical 2-link IK, the same law of cosines as
 * ik_helloworld.c. Returns the elbow position and the 2-link tip
 * (which lands on the target T whenever it is reachable).
 *
 * Used internally by solve_ik3 to handle the first two links of
 * the 3-link arm; the third link is then aimed at the real target.
 */
static TwoLinkPose solve_ik2(Vec2 S, Vec2 T, bool elbow_up,
                             float L1, float L2)
{
    Vec2  d_vec = vec2_sub(T, S);
    float d     = vec2_len(d_vec);

    /* Degenerate: target on top of shoulder. */
    if (d < 1e-6f)
        return (TwoLinkPose){ vec2(S.x + L1, S.y), S };

    /* Law of cosines at the shoulder vertex of triangle SET. */
    float cos_a       = (L1*L1 + d*d - L2*L2) / (2.0f * L1 * d);
    float alpha       = acosf(clampf(cos_a, -1.0f, 1.0f));
    float phi         = atan2f(d_vec.y, d_vec.x);
    float side        = elbow_up ? -1.0f : +1.0f;
    float elbow_angle = phi + side * alpha;

    Vec2 E = vec2(S.x + L1 * cosf(elbow_angle),
                  S.y + L1 * sinf(elbow_angle));

    /* Tip = E + L2 toward T. Lands on T exactly when reachable. */
    Vec2 dir = vec2_normalize_or(vec2_sub(T, E), vec2(1.0f, 0.0f));
    Vec2 W   = vec2_add(E, vec2_scl(dir, L2));

    return (TwoLinkPose){E, W};
}

/*
 * solve_ik3 — three-link IK via the 2+1 hybrid. Three steps,
 * matching the ALGORITHM block in MENTAL MODEL exactly:
 *
 *   1. Pull the target back along S→T by L3 to get T2 — the
 *      virtual target for the first two links.
 *   2. Run analytical 2-link IK from S to T2 to place E and W.
 *   3. Aim the third link from W toward the real target T.
 *
 * Pure function. Returns all three computed positions.
 */
static ArmPose solve_ik3(Vec2 S, Vec2 T, bool elbow_up)
{
    /* Step 1 — virtual 2-link target T2 = T − L3 · dir(S → T). */
    Vec2 dir_st = vec2_normalize_or(vec2_sub(T, S), vec2(1.0f, 0.0f));
    Vec2 T2     = vec2_sub(T, vec2_scl(dir_st, L3_PX));

    /* Step 2 — 2-link IK places E and the wrist W (= T2 if reachable). */
    TwoLinkPose tl = solve_ik2(S, T2, elbow_up, L1_PX, L2_PX);

    /* Step 3 — aim the hand link from W toward the real target. */
    Vec2 dir_wt = vec2_normalize_or(vec2_sub(T, tl.tip), vec2(1.0f, 0.0f));
    Vec2 H      = vec2_add(tl.tip, vec2_scl(dir_wt, L3_PX));

    return (ArmPose){tl.elbow, tl.tip, H};
}

/* ===================================================================== */
/* §6  scene — Scene state, input dispatch, draw                          */
/* ===================================================================== */

/*
 * Mode — which solver runs this frame.
 *
 *   MODE_FK : compute_fk(S, θ1, θ2, θ3) → ArmPose
 *             User edits ANGLES; positions FALL OUT.
 *   MODE_IK : solve_ik3(S, T, elbow_up) → ArmPose
 *             User edits the TARGET; angles FALL OUT (implicit).
 *
 * One toggle, two data-flow directions. The keymap and the HUD
 * also branch on Mode — see scene_input + screen_hud. The Mode
 * value lives in Scene so signal-handler-free code (scene_input)
 * can switch by mutating it.
 */
typedef enum { MODE_FK, MODE_IK } Mode;

/*
 * Scene — all per-frame state of the FK/IK demo.
 *
 * Intent
 *   The file's whole purpose is to put FK and IK back-to-back. Scene
 *   therefore holds the inputs for BOTH solvers simultaneously, plus
 *   ONE shared output cache (elbow, wrist, hand) that the renderer
 *   reads. Mode picks which input set the current frame consumes;
 *   the OTHER set is preserved across the toggle so flipping back is
 *   instant.
 *
 *   Read direction (what the data-flow arrow looks like):
 *
 *     FK mode:  theta1/2/3 ──compute_fk──► elbow,wrist,hand
 *     IK mode:  target     ──solve_ik3──► elbow,wrist,hand
 *
 *   Same OUTPUT slots, different INPUT slots. The renderer never
 *   needs to know which solver ran — it just reads the cache.
 *
 * Simulation vs Rendering locality
 *   ── Simulation state ── read+written by scene_input + solvers:
 *      shoulder              S, pinned origin (resets on SIGWINCH).
 *      theta1, theta2, theta3 FK INPUTS — a/d, w/s, z/x edit (in FK mode).
 *      target                IK INPUT — arrow keys edit (in IK mode).
 *      elbow_up              IK CONFIG — 'f' selects one of two valid
 *                            elbow configurations (Craig §4.4).
 *      elbow, wrist, hand    Cached SOLVER OUTPUTS — written each
 *                            frame, read by scene_draw + HUD.
 *
 *   ── Rendering / control state ── read only by scene_draw + HUD:
 *      mode                  Which solver runs (also gates the '+'
 *                            target glyph: drawn only in IK mode).
 *      paused                Frozen frame; solver path skipped,
 *                            render unchanged.
 *      rows, cols            Terminal extent in CELLS (resize cache).
 *
 *   Why mode counts as RENDER-CONTROL not SIM
 *     Because it doesn't change WORLD state — it changes WHICH
 *     pipeline transforms world state. The simulation that runs
 *     under MODE_FK and the one under MODE_IK both produce the
 *     same kind of output; the Mode bit just routes to one of them.
 *
 *   Why elbow_up is SIM (not RENDER-CONTROL)
 *     It changes the SOLVED CONFIGURATION (elbow up vs down). Two
 *     valid mathematical solutions, only one is realised — that's
 *     part of the math, not of the camera. Compare with `mode`,
 *     which doesn't change the answer, only the question.
 *
 * Things that DO NOT live here
 *   - Link lengths L1_PX, L2_PX, L3_PX → §1 config (compile-time).
 *   - Movement step sizes ANGLE_STEP_RAD, TARGET_STEP_PX → §1.
 *   - fps counter → main() local (display-only, not per-frame state).
 *   - Signal flags g_running / g_need_resize → §8 file-scope (must
 *     be writable from a signal handler that takes no user pointer).
 *
 * References [1] Craig §3 (FK) + §4 (IK casework); [4] Buss for the
 *   Jacobian alternative this file's IK is NOT.
 */
typedef struct {
    /* ── Simulation state: anchor ─────────────────────────────── */
    Vec2  shoulder;     /* S — pinned origin; resets on SIGWINCH    */

    /* ── Simulation state: FK inputs (active in MODE_FK) ──────── */
    float theta1;       /* shoulder angle (rad), abs from +X        */
    float theta2;       /* elbow    angle (rad), REL to upper arm   */
    float theta3;       /* wrist    angle (rad), REL to forearm     */

    /* ── Simulation state: IK inputs (active in MODE_IK) ──────── */
    Vec2  target;       /* T — IK only; drawn as '+'                */
    bool  elbow_up;     /* IK only; selects 1 of 2 valid solutions  */

    /* ── Simulation state: cached solver outputs (both modes) ─── *
     * Written each frame by whichever solver Mode selects; read   *
     * by scene_draw and the HUD without recomputing.              */
    Vec2  elbow;        /* E                                        */
    Vec2  wrist;        /* W                                        */
    Vec2  hand;         /* H — also the IK end-effector             */

    /* ── Rendering / control state ────────────────────────────── *
     * Read only by scene_draw and the HUD. mode also routes the   *
     * solver in the main loop, but does not itself BECOME state.  */
    Mode  mode;         /* MODE_FK or MODE_IK                       */
    bool  paused;       /* freezes solver path; render unchanged    */
    int   rows, cols;   /* terminal extent in CHARACTER CELLS       */
} Scene;

static Vec2 clamp_to_screen(Vec2 p, int rows, int cols)
{
    return vec2(clampf(p.x, 0.0f, cells_to_px_w(cols) - 1.0f),
                clampf(p.y, 0.0f, cells_to_px_h(rows) - 1.0f));
}

static Vec2 clamp_to_reach(Vec2 t, Vec2 shoulder, float reach)
{
    Vec2  d   = vec2_sub(t, shoulder);
    float len = vec2_len(d);
    if (len <= reach) return t;
    return vec2_add(shoulder, vec2_scl(d, reach / len));
}

/*
 * scene_recompute — run whichever solver matches the current mode
 * and write the result into the scene's position cache. Called at
 * init, on resize, and once per frame in main.
 */
static void scene_recompute(Scene *s)
{
    ArmPose p = (s->mode == MODE_FK)
        ? compute_fk(s->shoulder, s->theta1, s->theta2, s->theta3)
        : solve_ik3 (s->shoulder, s->target, s->elbow_up);
    s->elbow = p.elbow;
    s->wrist = p.wrist;
    s->hand  = p.hand;
}

/*
 * scene_reset — restore default angles, target, and elbow side.
 * Mode is preserved on purpose: pressing 'r' should not yank the
 * user out of the mode they were experimenting in.
 */
static void scene_reset(Scene *s)
{
    s->theta1   = 0.0f;
    s->theta2   = 0.0f;
    s->theta3   = 0.0f;
    s->elbow_up = false;
    s->paused   = false;

    float reach = L1_PX + L2_PX + L3_PX;
    s->target   = vec2(s->shoulder.x + reach * INITIAL_TARGET_REACH_FRAC,
                       s->shoulder.y);

    scene_recompute(s);
}

/*
 * scene_init — anchor S at screen centre, set mode = FK, then
 * reset all per-mode values.
 */
static void scene_init(Scene *s, int rows, int cols)
{
    s->rows     = rows;
    s->cols     = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);
    s->mode     = MODE_FK;
    scene_reset(s);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows = rows;
    s->cols = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);
    s->target   = clamp_to_screen(s->target, rows, cols);
    s->target   = clamp_to_reach (s->target, s->shoulder,
                                  L1_PX + L2_PX + L3_PX);
    scene_recompute(s);
}

/*
 * scene_input — translate one keypress.
 *
 * Always-on keys (work even when paused): space, r, m, f.
 * Mode-specific value-changing keys are ignored when paused.
 *
 * FK keys live on the keyboard's left half (a/d, w/s, z/x) so the
 * three angle pairs sit in a natural row stack. IK keys are the
 * arrow cluster, mirroring ik_helloworld.c.
 */
static void scene_input(Scene *s, int ch)
{
    switch (ch) {
        case ' ': s->paused   = !s->paused;        return;
        case 'r': scene_reset(s);                  return;
        case 'm': s->mode     = (s->mode == MODE_FK) ? MODE_IK : MODE_FK; return;
        case 'f': s->elbow_up = !s->elbow_up;      return;
        default: break;
    }

    if (s->paused) return;

    if (s->mode == MODE_FK) {
        const float a = ANGLE_STEP_RAD;
        switch (ch) {
            case 'a': s->theta1 -= a; break;
            case 'd': s->theta1 += a; break;
            case 'w': s->theta2 -= a; break;
            case 's': s->theta2 += a; break;
            case 'z': s->theta3 -= a; break;
            case 'x': s->theta3 += a; break;
            default: break;
        }
        s->theta1 = wrap_pi(s->theta1);
        s->theta2 = wrap_pi(s->theta2);
        s->theta3 = wrap_pi(s->theta3);
    } else {
        const float k = KEY_STEP_PX;
        switch (ch) {
            case KEY_LEFT:  s->target.x -= k; break;
            case KEY_RIGHT: s->target.x += k; break;
            case KEY_UP:    s->target.y -= k; break;
            case KEY_DOWN:  s->target.y += k; break;
            default: break;
        }
        s->target = clamp_to_screen(s->target, s->rows, s->cols);
        s->target = clamp_to_reach (s->target, s->shoulder,
                                    L1_PX + L2_PX + L3_PX);
    }
}

/* ---- rendering ------------------------------------------------------- */

static inline bool in_screen(int row, int col, int rows, int cols)
{
    return row >= 0 && row < rows && col >= 0 && col < cols;
}

/*
 * draw_line_px — parametric line from a to b in pixel space, sampled
 * at half-cell spacing along the dominant axis so no cell on the
 * path is skipped at any angle.
 */
static void draw_line_px(Vec2 a, Vec2 b, chtype glyph, int rows, int cols)
{
    float dx = b.x - a.x, dy = b.y - a.y;
    float steps_f = fmaxf(fabsf(dx) / (CELL_W * 0.5f),
                          fabsf(dy) / (CELL_H * 0.5f));
    int   steps   = (int)steps_f + 1;
    for (int i = 0; i <= steps; i++) {
        float t   = (float)i / (float)steps;
        float px  = a.x + t * dx;
        float py  = a.y + t * dy;
        int   col = px_to_cell_x(px);
        int   row = px_to_cell_y(py);
        if (in_screen(row, col, rows, cols))
            mvaddch(row, col, glyph);
    }
}

static void draw_line(Vec2 a, Vec2 b, int color_pair, int rows, int cols)
{
    chtype glyph = (chtype)((unsigned char)'#') | COLOR_PAIR(color_pair);
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    draw_line_px(a, b, glyph, rows, cols);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

static void draw_point(Vec2 p, char glyph, int color_pair, int rows, int cols)
{
    int row = px_to_cell_y(p.y);
    int col = px_to_cell_x(p.x);
    if (!in_screen(row, col, rows, cols)) return;
    attron(COLOR_PAIR(color_pair) | A_BOLD);
    mvaddch(row, col, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | A_BOLD);
}

/*
 * scene_draw — three lines + four arm markers + one target marker
 * (only in IK mode). Lines first, markers on top; '@' shoulder
 * paints last so the pinned anchor always wins the cell contest.
 */
static void scene_draw(const Scene *s)
{
    Vec2 S = s->shoulder, E = s->elbow, W = s->wrist, H = s->hand;

    /* three link lines */
    draw_line(S, E, PAIR_LINK_UPPER, s->rows, s->cols);
    draw_line(E, W, PAIR_LINK_FORE,  s->rows, s->cols);
    draw_line(W, H, PAIR_LINK_HAND,  s->rows, s->cols);

    /* IK target — painted before joints so any overlap is hidden
     * by the joint markers (no flicker if hand sits on target). */
    if (s->mode == MODE_IK)
        draw_point(s->target, '+', PAIR_TARGET, s->rows, s->cols);

    /* hand, wrist, elbow, shoulder — distal first, proximal last. */
    draw_point(H, '*', PAIR_HAND,     s->rows, s->cols);
    draw_point(W, 'o', PAIR_JOINT,    s->rows, s->cols);
    draw_point(E, 'O', PAIR_JOINT,    s->rows, s->cols);
    draw_point(S, '@', PAIR_SHOULDER, s->rows, s->cols);
}

/* ===================================================================== */
/* §7  screen — ncurses init / present / HUD                              */
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

static void screen_cleanup(void)
{
    if (!isendwin()) endwin();
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * screen_hud — required HUD: status row 0, hint row last. Row 1 is
 * mode-aware: in FK it shows the three angles in degrees; in IK it
 * shows target distance, hand-to-target error, and the elbow side.
 * The bottom hint strip changes its key list with the mode too.
 */
static void screen_hud(const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    char buf[128];

    /* Row 0 — mode + paused state + fps. */
    snprintf(buf, sizeof buf, " %5.1f fps  mode:%s  %s ",
             fps, s->mode == MODE_FK ? "FK" : "IK",
             s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — mode-specific parameters. */
    if (s->mode == MODE_FK) {
        snprintf(buf, sizeof buf,
                 " L:%.0f,%.0f,%.0f  th1:%+7.1f°  th2:%+7.1f°  th3:%+7.1f° ",
                 L1_PX, L2_PX, L3_PX,
                 s->theta1 * RAD_TO_DEG,
                 s->theta2 * RAD_TO_DEG,
                 s->theta3 * RAD_TO_DEG);
    } else {
        float reach = L1_PX + L2_PX + L3_PX;
        float h_err = vec2_dist(s->hand, s->target);
        snprintf(buf, sizeof buf,
                 " reach:%.0fpx  |T-S|:%5.1f  |H-T|:%5.1f  elbow:%s ",
                 reach,
                 vec2_dist(s->target, s->shoulder),
                 h_err,
                 s->elbow_up ? "up  " : "down");
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint — keys differ by mode. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    if (s->mode == MODE_FK)
        mvprintw(rows - 1, 0,
                 " q:quit  spc:pause  r:reset  m:mode  a/d:th1  w/s:th2  z/x:th3 ");
    else
        mvprintw(rows - 1, 0,
                 " q:quit  spc:pause  r:reset  m:mode  arrows:target  f:flip elbow ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §8  app — signals, resize, main loop                                   */
/* ===================================================================== */

static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_need_resize = 1;
    else                 g_running     = 0;
}

static void install_signals(void)
{
    struct sigaction sa = {0};
    sa.sa_handler = on_signal;
    sigaction(SIGINT,   &sa, NULL);
    sigaction(SIGTERM,  &sa, NULL);
    sigaction(SIGWINCH, &sa, NULL);
}

int main(void)
{
    install_signals();
    atexit(screen_cleanup);
    screen_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    Scene scene;
    scene_init(&scene, rows, cols);

    int64_t       t_prev       = clock_ns();
    int64_t       fps_accum_ns = 0;
    int           fps_frames   = 0;
    float         fps          = 0.0f;
    const int64_t TICK_NS      = NS_PER_SEC / TARGET_FPS;

    while (g_running) {
        int64_t t_now = clock_ns();
        fps_accum_ns += t_now - t_prev;
        t_prev = t_now;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC) {
            fps          = (float)fps_frames * 1e9f / (float)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        if (g_need_resize) {
            g_need_resize = 0;
            endwin();
            refresh();
            getmaxyx(stdscr, rows, cols);
            scene_resize(&scene, rows, cols);
        }

        /* (1) input — angles in FK mode, target in IK mode */
        for (int ch; (ch = getch()) != ERR; ) {
            if (ch == 'q' || ch == 27 /*ESC*/) { g_running = 0; break; }
            scene_input(&scene, ch);
        }

        /* (2) run the right solver and cache its output positions */
        scene_recompute(&scene);

        /* (3) clear / draw / present */
        erase();
        scene_draw(&scene);
        screen_hud(&scene, fps);
        screen_present();

        int64_t elapsed = clock_ns() - t_now;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    return 0;
}
