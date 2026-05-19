/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_helloworld.c — two-link arm reaches a target via analytical IK
 *
 * DEMO: The textbook hello-world of inverse kinematics. A two-link
 *       arm hangs off a fixed shoulder. The user moves ONE target
 *       with the arrow keys, and every frame the solver computes
 *       where the elbow must bend so the hand (= the target) sits
 *       L2 from the elbow and L1 from the shoulder.
 *
 *           S ──────────────── E ──────────────── T
 *           @       L1         O       L2         *
 *        shoulder            elbow              target
 *         (fixed)         (computed)          (you move)
 *
 *       The target is clamped to the reach disc (radius L1 + L2)
 *       so it can never leave — both link lengths therefore stay
 *       visibly constant forever. Press 'f' to flip the elbow
 *       to the OTHER valid solution: there are always exactly
 *       two for any reachable target, and that ambiguity is the
 *       conceptual heart of IK.
 *
 * Study alongside:
 *   animation/ik_arm_reach.c              — FABRIK on a 4-link arm
 *   animation/snake_inverse_kinematics.c  — FABRIK on a long chain
 *   animation/snake_forward_kinematics.c  — FK contrast
 *
 * Section map:
 *   §1  config       — every tunable in one place
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus per-link / per-marker pairs
 *   §4  coords       — pixel↔cell aspect-ratio bridge
 *   §5  ik           — Vec2 + solve_ik(S, T) → E
 *   §6  scene        — Scene state, input, draw
 *   §7  screen       — ncurses init / present / HUD
 *   §8  app          — signals, resize, main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume target input
 *   r            reset target to default
 *   f            flip elbow to the OTHER valid solution
 *   ↑ ↓ ← →      move target
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_helloworld.c \
 *       -o ik_helloworld -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytical 2-link inverse kinematics via the law
 *                  of cosines. Shoulder S, elbow E, and target T form
 *                  a triangle with sides L1 (constant), L2 (constant),
 *                  and d = |T − S|. All three sides are known, so the
 *                  law of cosines gives the angle α at the shoulder
 *                  vertex in one expression. The direction φ from S
 *                  to T is one atan2 call. The elbow then sits L1
 *                  from S in direction (φ ± α). The ± selects
 *                  between the two valid solutions. No iteration, no
 *                  convergence loop — pure trigonometry.
 *
 * Data-structure : Three Vec2 points (S, T, E) and two scalar link
 *                  lengths, all in pixel space. solve_ik(S, T,
 *                  elbow_up) is a pure function returning E; the
 *                  main loop caches the result as scene.elbow so
 *                  renderer and HUD read it without re-solving.
 *
 * Rendering      : Two coloured lines (S→E cyan, E→T orange) plus
 *                  three markers — '@' shoulder, 'O' elbow, '*'
 *                  target. Lines first, markers on top, '@' last so
 *                  it always wins the cell contest at S.
 *
 * Performance    : O(1) per frame. One sqrt, one acos, one atan2,
 *                  two cosf/sinf. Below the noise floor of one
 *                  ncurses redraw — frame budget is dominated by
 *                  terminal I/O, not math.
 *
 * Why closed-form, not iterative? With two links the law of cosines
 * is exact and one-shot — no iteration loop, no convergence test.
 * Iterative solvers (FABRIK, Jacobian) become attractive only past
 * two links, when the analytical solution branches into multi-page
 * case analyses. For a hello-world, the closed form IS the lesson.
 *
 * References
 * ──────────
 *   ── Canonical robotics textbooks (analytical IK) ─────────────────
 *   [1] Craig, J. J. (2005), "Introduction to Robotics: Mechanics
 *       and Control" (3rd ed.), Pearson — Ch. 4 derives the law-of-
 *       cosines 2-link solution that solve_ik() implements verbatim.
 *       Also §4.4 "Solvability" — the reachability discussion that
 *       clamp_to_reach() enforces.
 *   [2] Spong, M. W., Hutchinson, S. & Vidyasagar, M. (2005), "Robot
 *       Modeling and Control", Wiley — alternative canonical text;
 *       Ch. 4 covers the same derivation with cleaner notation.
 *   [3] Murray, R. M., Li, Z. & Sastry, S. S. (1994), "A Mathematical
 *       Introduction to Robotic Manipulation", CRC — rigorous Lie-
 *       group treatment; read once you want to see IK as solving a
 *       map on SE(2)/SE(3).
 *
 *   ── Iterative IK alternatives (what we DON'T need) ────────────────
 *   [4] Buss, S. R. (2004), "Introduction to Inverse Kinematics with
 *       Jacobian Transpose, Pseudoinverse and Damped Least Squares",
 *       UCSD course notes — the Jacobian-iterative family. Read to
 *       see what closed-form 2-link IK lets you SKIP.
 *   [5] Aristidou, A. & Lasenby, J. (2011), "FABRIK: a fast,
 *       iterative solver for the inverse kinematics problem",
 *       Graphical Models 73(5), pp. 243-260 — the iterative
 *       successor for longer chains; see ik_arm_reach.c.
 *   [6] Wang, L.-C. T. & Chen, C. C. (1991), "A combined
 *       optimization method for solving the inverse kinematics
 *       problem of mechanical manipulators", IEEE TRA 7(4) — CCD
 *       (Cyclic Coordinate Descent), one of the iteration schemes
 *       FABRIK improved on.
 *
 *   ── The math driving the solver ──────────────────────────────────
 *   [7] Heath, T. L. (1908), "The Thirteen Books of Euclid's
 *       Elements", Cambridge — Book II Proposition 12-13: the
 *       geometric law of cosines, 300 BCE, still the load-bearing
 *       theorem in §5.
 *   [8] al-Kāshī, J. (1427), "Miftāḥ al-Hisāb" ("Key of Arithmetic")
 *       — earliest known trigonometric form of the law of cosines
 *       used here.
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *   [9] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com
 *       — when fixed-step matters; this file uses variable-step
 *       because solve_ik is one closed-form call, unconditionally
 *       stable.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *  [10] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       the integer line algorithm; we use the simpler parametric
 *       oversample (draw_line_px in §6) because float math is
 *       already paid for in solve_ik.
 *  [11] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the high-contrast glyph choice ('@', 'O', '*', '#').
 *  [12] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the diff pipeline §7 relies on.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [13] https://en.wikipedia.org/wiki/Inverse_kinematics
 *  [14] https://en.wikipedia.org/wiki/Law_of_cosines
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Three points form a triangle. If you know all three side lengths,
 * the law of cosines gives every angle in closed form. That is ALL
 * 2-link IK is. Solve the triangle SET (sides L1, L2, d), place the
 * elbow at the angle the law of cosines gave you, done. The hand is
 * the target — when the triangle exists, |E − T| is L2 by
 * construction, so the forearm always has its correct length.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Reach for a coffee cup with your right hand. Your shoulder didn't
 * move — it can't, it's bolted to your torso. Your hand IS the cup
 * — you commanded it there. So WHERE did your elbow go? It bent
 * automatically to whatever angle was required for an upper arm of
 * fixed length plus a forearm of fixed length to reach from your
 * stationary shoulder to your chosen hand position. Your nervous
 * system solved a triangle.
 *
 *   shoulder    @          ← S, fixed, never moves
 *                \
 *                 \  L1    ← upper arm (constant length)
 *                  \
 *           elbow   O      ← E, solver picks this
 *                    \
 *                     \  L2 ← forearm (constant length)
 *                      \
 *                target  *  ← T, you move this; this IS the hand
 *
 * One nail. One target. The elbow is the unknown. One triangle.
 *
 * INVARIANT
 * ─────────
 * The input layer (clamp_to_reach, §6) pulls T radially onto the
 * reach disc whenever the user pushes past the rim, so d = |T − S|
 * is always ≤ L1 + L2 and the triangle SET is always valid. The
 * solver never has to handle unreachable cases — they cannot
 * happen.
 *
 * ALGORITHM
 * ─────────
 *   d_vec       = T − S
 *   d           = |d_vec|
 *   cos α       = (L1² + d² − L2²) / (2·L1·d)         // law of cosines
 *   α           = acos(cos α)
 *   φ           = atan2(d_vec.y, d_vec.x)             // direction S→T
 *   side        = elbow_up ? −1 : +1                  // pick a solution
 *   elbow_angle = φ + side · α
 *   E           = S + L1 · (cos elbow_angle, sin elbow_angle)
 *
 * Eight lines of arithmetic, every frame. Read solve_ik in §5
 * line-by-line and it matches this exactly.
 *
 * KEY FORMULAS
 * ────────────
 *   Law of cosines : cos α = (L1² + d² − L2²) / (2·L1·d)
 *                    Domain: |L1−L2| ≤ d ≤ L1+L2 (always satisfied
 *                    here thanks to clamp_to_reach).
 *
 *   atan2          : four-quadrant arctan returning the right angle
 *                    for any direction; ordinary atan loses sign
 *                    information.
 *
 *   Reach disc     : the set of points within R = L1 + L2 of the
 *                    shoulder. clamp_to_reach forces T back to the
 *                    rim if it ever leaves, so |E − T| stays exactly
 *                    L2 every frame.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • User pushes past the rim. clamp_to_reach pulls T back radially;
 *     the visible forearm length never grows.
 *
 *   • Target on top of shoulder (d ≈ 0). Triangle degenerates;
 *     cos α divides by zero. We check d < ε and pick a default
 *     elbow direction (+X) so the chain stays visible.
 *
 *   • acos input drift. Floating-point error can push the cosine
 *     just outside [−1, 1] on boundaries; we clamp before acos to
 *     avoid NaN.
 *
 *   • Resize. SIGWINCH re-anchors S to the new screen centre and
 *     re-clamps T into both the new screen bounds AND the reach
 *     disc, so the invariant survives.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press 'r'. Arm in horizontal rest pose: '@' at centre, '*'
 *     to the right at 70% of full reach, 'O' folded slightly.
 *
 *   • Drag the target around. Both line lengths stay visibly
 *     constant; only their angles change. The HUD shows |E − T|
 *     pinned at L2.
 *
 *   • Hold an arrow key. The target glides until it touches the
 *     rim of the reach disc, then stops there.
 *
 *   • Press 'f'. The elbow snaps to the OTHER valid position —
 *     mirrored across the S→T line. The hand stays on the target.
 *
 *   • Sanity-check by eye: the cell distance from '@' to 'O'
 *     should be constant (≈10 cells), and the same for 'O' to '*'.
 *     Any visible growth or shrinkage would mean a bug.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read animation/fk_helloworld.c FIRST so FK is in
 *      your head; this file is the inverse problem on the same
 *      geometry.
 *   2. §5 ik — the entire algorithm is one function: solve_ik.
 *      Read AFTER tutorials T1-T6. Eight lines mirror the formula
 *      in MENTAL MODEL exactly.
 *   3. §6 scene — input pipeline (arrow keys → target T, clamped to
 *      the reach disc) and rendering. The clamp is what makes the
 *      INVARIANT |E−T| = L₂ hold every frame.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if you've seen the
 *      framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   S, T, E         shoulder / target / elbow positions (Vec2).
 *                   S fixed; T is INPUT (you move it); E is OUTPUT.
 *   L1, L2          link lengths (constant).
 *   d_vec, d        T − S, and its length. The "third side" of
 *                   the triangle.
 *   alpha (α)       angle at the shoulder VERTEX of the (S, E, T)
 *                   triangle, opposite the L₂ side.
 *   phi (φ)         direction S → T, in world coordinates.
 *   elbow_angle     final absolute angle of the upper arm,
 *                   = φ + side · α.
 *   side            ±1 selector — picks elbow-up vs. elbow-down.
 *   elbow_up        bool form of `side`, exposed to the user via
 *                   the 'f' key.
 *
 * Background you need
 * ───────────────────
 *   - The FK primitive (fk_helloworld T2). solve_ik finds θ₁
 *     such that the FK formula lands on the target.
 *   - Law of cosines: in any triangle with sides a, b, c, the
 *     angle opposite c is acos((a² + b² − c²) / (2ab)).
 *   - atan2(y, x): four-quadrant arctan returning angle of vector
 *     (x, y) from +X. Plain atan loses sign info.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Iterative solvers (Jacobian, FABRIK). Two-link IK is
 *     closed-form; iteration only matters past two links.
 *   - Quaternions. 2-D angles compose by addition.
 *   - DH parameters or full robotics kinematics — the closed-form
 *     2-link case predates and undercuts that machinery.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build analytical 2-link IK from first principles.
 *
 *   T1  The IK problem statement
 *   T2  Reachability — the disc and the rim
 *   T3  Three sides of a triangle, all known
 *   T4  Law of cosines — angle from sides
 *   T5  Two valid solutions — elbow up or down
 *   T6  Why this approach STOPS WORKING past two links
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE IK PROBLEM STATEMENT
 * ────────────────────────────
 * Given:
 *   - a fixed shoulder S
 *   - link lengths L₁, L₂
 *   - a target T (where the user wants the hand)
 *
 * Find:
 *   - joint angles θ₁, θ₂ such that the FK output coincides with T.
 *
 * Concretely we want the elbow E such that:
 *   |E − S| = L₁          (upper arm has correct length)
 *   |E − T| = L₂          (forearm has correct length)
 *
 * Two equations, two unknowns (E.x, E.y). Solving them is what
 * the rest of this tutorial unfolds.
 *
 * Note we never directly solve for θ₁, θ₂. Once E is known, the
 * angles fall out trivially: θ₁ is the angle of E − S; θ₂ is the
 * angle from (E − S) to (T − E). For rendering, knowing E is
 * already enough — we just need to draw two line segments.
 *
 * T2  REACHABILITY — THE DISC AND THE RIM
 * ───────────────────────────────────────
 * Before solving, ask: is the target REACHABLE? With link lengths
 * L₁, L₂ the hand can reach exactly the points within distance:
 *
 *     d_min ≤ |T − S| ≤ d_max
 *     d_min = |L₁ − L₂|     (forearm folded back maximally)
 *     d_max = L₁ + L₂       (arm fully extended)
 *
 * For L₁ = L₂ that's [0, 2L]. The set of reachable hand positions
 * is a DISC of radius L₁ + L₂ centred on S, with a hole of radius
 * |L₁ − L₂| in the middle (only relevant when links differ).
 *
 *      ┌──────────────────────────────────────────────┐
 *      │         outer rim (full extension)           │
 *      │              ─────────                       │
 *      │           ─                ─                 │
 *      │         /                    \               │
 *      │        /  reach disc           \             │
 *      │       │       @  shoulder       │            │
 *      │        \                       /             │
 *      │         \                     /              │
 *      │           ─                ─                 │
 *      │              ─────────                       │
 *      └──────────────────────────────────────────────┘
 *
 * If the user pushes T outside the disc, IK has NO solution. Two
 * options:
 *   - REPORT failure and stop drawing the arm. Honest but jarring.
 *   - CLAMP the target onto the disc rim. The arm fully extends
 *     toward the off-disc target, and the user simply can't push
 *     past the rim. Visual — the link lengths stay constant, the
 *     "target" '*' is the same as the "hand."
 *
 * We chose CLAMP. clamp_to_reach() in §6 does it. Once it runs,
 * the solver is guaranteed reachable input — a robust invariant.
 *
 * T3  THREE SIDES OF A TRIANGLE, ALL KNOWN
 * ────────────────────────────────────────
 * (S, E, T) are the corners of a triangle:
 *
 *     SE side has length L₁          (constant)
 *     ET side has length L₂          (constant)
 *     ST side has length d = |T − S| (computed each frame)
 *
 * SAS (side-angle-side), SSS (side-side-side), and other
 * configurations have classical "triangle solving" recipes. We
 * have SSS — three sides given. The question is the SHAPE of
 * the triangle and where to place E given S and T are pinned.
 *
 *      ┌──────────────────────────────────────────────┐
 *      │                                              │
 *      │       elbow E                                │
 *      │           O                                  │
 *      │          / \                                 │
 *      │     L₁  /   \  L₂                            │
 *      │        /     \                               │
 *      │       /   α   \                              │
 *      │      @─────────*                             │
 *      │      S    d    T                             │
 *      └──────────────────────────────────────────────┘
 *
 * α is the angle at the shoulder vertex, between the S→E and
 * S→T directions.
 *
 * T4  LAW OF COSINES — ANGLE FROM SIDES
 * ─────────────────────────────────────
 * In any triangle with sides a, b, c and angle γ opposite side c:
 *
 *     c² = a² + b² − 2ab · cos γ
 *
 * That's the LAW OF COSINES. Solving for γ:
 *
 *     cos γ = (a² + b² − c²) / (2ab)
 *
 * Plug in our triangle: at the shoulder vertex, the angle α is
 * opposite the L₂ side (the side OPPOSITE α is the one not
 * touching the α-vertex). So a = L₁, b = d, c = L₂:
 *
 *     cos α = (L₁² + d² − L₂²) / (2 · L₁ · d)
 *     α     = acos(...)
 *
 * That ONE equation gives the offset angle of the upper arm
 * from the S→T direction.
 *
 * Then the absolute angle of the upper arm is:
 *
 *     φ = atan2(T.y − S.y, T.x − S.x)         ← direction of T
 *     elbow_angle = φ ± α                     ← rotate from φ
 *
 * Why ±? See T5.
 *
 * Drift defence: floating-point can push the cosine just outside
 * [-1, +1] on boundary configurations (target exactly on the rim).
 * acos returns NaN there. Clamp the input to [-1, +1] before acos.
 *
 * T5  TWO VALID SOLUTIONS — ELBOW UP OR DOWN
 * ──────────────────────────────────────────
 * α is unsigned: acos returns a value in [0, π]. But the elbow
 * can sit on EITHER side of the S→T line. So there are two
 * triangles consistent with our SSS:
 *
 *      ┌──────────────────────────────────────────────┐
 *      │                                              │
 *      │       O  ← elbow-up                          │
 *      │      / \                                     │
 *      │     /   \                                    │
 *      │    /     \                                   │
 *      │   @-------*                                  │
 *      │    \     /                                   │
 *      │     \   /                                    │
 *      │      \ /                                     │
 *      │       O  ← elbow-down                        │
 *      └──────────────────────────────────────────────┘
 *
 * Mathematically: the elbow_angle = φ + α and the elbow_angle =
 * φ − α both satisfy the IK equations.
 *
 * The solver must PICK ONE. We expose the choice as a boolean
 * 'elbow_up'; the 'f' key flips it. Most real applications pick
 * elbow_up = true (anatomically natural for human arms reaching
 * forward).
 *
 * Reachability EDGE CASE: when the target is exactly on the rim
 * (d = L₁ + L₂), α = 0 — the two solutions COINCIDE. The arm is
 * fully extended; there's no elbow bend either way. When the
 * target is at the far inner-rim limit (d = |L₁ − L₂|, only if
 * L₁ ≠ L₂), α = π and the two solutions also coincide, with the
 * forearm folded entirely along the upper arm.
 *
 * Both edge cases are CONTINUOUS (no discontinuity in elbow
 * position as you approach them); the 'f' flip simply has no
 * visible effect there.
 *
 * T6  WHY THIS APPROACH STOPS WORKING PAST TWO LINKS
 * ──────────────────────────────────────────────────
 * Closed-form 2-link IK is BEAUTIFUL. Eight lines of trig, exact,
 * one frame, no iteration. Why doesn't every IK solver work this
 * way?
 *
 * For 3 links the problem becomes UNDERDETERMINED. The user
 * supplies one target (2 equations: T.x, T.y). The solver has
 * three unknowns (θ₁, θ₂, θ₃). Infinitely many configurations
 * satisfy the target — one extra DOF (degree of freedom) is
 * unconstrained. Picking ONE solution requires extra criteria:
 * naturalness, joint limits, smoothness over time.
 *
 * For 4+ links the under-determination grows further. There's no
 * clean closed-form; the field moved to ITERATIVE solvers:
 *
 *   FABRIK     "forward and backward reaching IK" — sweeps the
 *              chain twice per iteration. See ik_arm_reach.c
 *              and ik_tentacle_seek.c.
 *
 *   CCD        "cyclic coordinate descent" — adjusts one joint
 *              at a time, repeats. Older but simpler.
 *
 *   Jacobian   linearise around current pose, take a step toward
 *              the target. Used in physics / robotics.
 *
 * All of those work for any chain length, but they ITERATE — the
 * solver runs many passes per frame to converge. The cost goes
 * up; the trade is generality.
 *
 * Lesson: the elegance of this file's solve_ik EXPIRES at three
 * links. Use it for 2-link arms (knees, elbows, animal legs in
 * planar projection) and reach for FABRIK once you have a snake
 * or tentacle.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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
    PAIR_JOINT        = 5,
    PAIR_SHOULDER     = 6,
    PAIR_TARGET       = 7,
};

/* Equal upper arm and forearm: the inner-reach degenerate case
 * |L1 − L2| = 0 is gone, so any d ∈ [0, L1+L2] has a valid triangle.
 * 80 px = 10 cells per link → total reach 20 cells. */
#define L1_PX  80.0f                  /* upper arm length, pixels */
#define L2_PX  80.0f                  /* forearm  length, pixels */

/* 4 px per arrow keypress = half a cell on the wide axis. */
#define KEY_STEP_PX                4.0f

/* Initial target position: 70% of full reach along the rest line. */
#define INITIAL_TARGET_REACH_FRAC  0.7f

/* Terminal cell dimensions — the aspect-ratio bridge. CELL_W=8
 * sub-pixels per column, CELL_H=16 per row, giving the 2:1 ratio
 * that matches a typical terminal font. All math is in pixel space
 * (isotropic); cell conversion happens only at draw time. */
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
 *   upper arm   cyan
 *   forearm     orange
 *   elbow       white
 *   shoulder    lime    (anchor — never moves)
 *   target      red     (the only thing the user controls)
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
        init_pair(PAIR_JOINT,       255, -1);
        init_pair(PAIR_SHOULDER,    118, -1);
        init_pair(PAIR_TARGET,      196, -1);
    } else {
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_UPPER,  COLOR_CYAN,    -1);
        init_pair(PAIR_LINK_FORE,   COLOR_RED,     -1);
        init_pair(PAIR_JOINT,       COLOR_WHITE,   -1);
        init_pair(PAIR_SHOULDER,    COLOR_GREEN,   -1);
        init_pair(PAIR_TARGET,      COLOR_RED,     -1);
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
/* §5  ik — Vec2 + solve_ik(S, T) → E                                     */
/* ===================================================================== */

/*
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   The §5 IK algebra lives in pixel space, not cells. Each cell is
 *   CELL_W × CELL_H sub-pixels (8 × 16 by default), giving the math
 *   sub-cell resolution so a moving target reads as continuous
 *   motion instead of jumping cell-to-cell. The conversion to cell
 *   space happens at the §6 render boundary (px_to_cell_x/y) — the
 *   project's "one conversion point" rule.
 *
 * Convention
 *   x : EASTWARD pixel coordinate (positive → right of screen).
 *   y : SOUTHWARD pixel coordinate (positive → down the screen).
 *
 *   atan2f(d_vec.y, d_vec.x) in solve_ik returns φ in screen-space
 *   conventions; the `elbow_up` sign flip (−1 vs +1) chooses which
 *   side of the S→T baseline the elbow lands on. Because y points
 *   DOWN on screen, "elbow_up = true" actually pushes the elbow
 *   toward smaller y (which appears UP on screen). The convention
 *   matches the visual, not the math y-axis.
 *
 * Why a value type (not a pointer)
 *   Sized 8 bytes; passes in registers on every modern ABI.
 *   solve_ik returns Vec2 by value; -O2 lowers it to the same
 *   straight-line code as direct field writes.
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
 * solve_ik — analytical 2-link inverse kinematics. Pure function:
 * given a fixed shoulder S, a target T (assumed within the reach
 * disc), and a side selector, return the elbow position E.
 * The body matches the ALGORITHM block in MENTAL MODEL line for line.
 */
static Vec2 solve_ik(Vec2 S, Vec2 T, bool elbow_up)
{
    Vec2  d_vec = vec2_sub(T, S);
    float d     = vec2_len(d_vec);

    /* Degenerate: T on top of S. Triangle collapses to a point;
     * cos α would divide by zero. Pick a default elbow direction
     * so the chain stays visible instead of NaN-ing. */
    if (d < 1e-6f) return vec2(S.x + L1_PX, S.y);

    float cos_a       = (L1_PX*L1_PX + d*d - L2_PX*L2_PX) / (2.0f * L1_PX * d);
    float alpha       = acosf(clampf(cos_a, -1.0f, 1.0f));
    float phi         = atan2f(d_vec.y, d_vec.x);
    float side        = elbow_up ? -1.0f : +1.0f;
    float elbow_angle = phi + side * alpha;

    return vec2(S.x + L1_PX * cosf(elbow_angle),
                S.y + L1_PX * sinf(elbow_angle));
}

/* ===================================================================== */
/* §6  scene — Scene state, input, draw                                   */
/* ===================================================================== */

/*
 * Scene — all per-frame state of the IK demo.
 *
 * Intent
 *   The scene holds three KINDS of data, kept separate so a reader
 *   can see the data-flow direction this file exists to teach:
 *
 *     ANCHOR   (fixed per resize) : shoulder              ← SIGWINCH
 *     INPUTS   (user-controlled)  : target, elbow_up      ← arrow keys / 'f'
 *     OUTPUT   (derived each frame): elbow                 ← solve_ik
 *
 *   The main loop's order is exactly: read INPUTS → run solve_ik →
 *   write OUTPUT → render. That arrow ALWAYS points the same way;
 *   no field ever flows backwards. Compare with fk_helloworld.c,
 *   where the user controls theta1/theta2 (angles) and elbow falls
 *   out — same struct shape, opposite data-flow direction. THAT is
 *   the IK/FK contrast.
 *
 * Simulation vs Rendering locality
 *   ── Simulation state ── read+written by scene_input + solve_ik:
 *      shoulder        S — pinned origin per resize.
 *      target          T — user-controlled via arrow keys; constrained
 *                          to lie inside the reach disc (see [1]
 *                          Craig §4.4 "Solvability" and clamp_to_reach).
 *      elbow_up        Selects 1 of 2 valid elbow positions
 *                          (knee-up / knee-down — the analytical
 *                          ambiguity that Craig §4 enumerates).
 *      elbow           E — cached solver output; refreshed each frame.
 *
 *   ── Rendering / control state ── read only by scene_draw + HUD:
 *      paused          Frozen frame; solver skipped, render unchanged.
 *      rows, cols      Terminal extent in CELLS (resize cache).
 *
 *   Note: rows/cols live HERE rather than in a separate Screen struct
 *   (cf. fk_centipede.c / hexpod_tripod.c) because this demo is small
 *   enough that one Scene argument carries everything draw needs. No
 *   App struct either — main() holds the signal flags directly. Trade-
 *   off: simpler reading vs the canonical multi-struct framework.
 *
 * Why elbow_up is SIM (not RENDER-CONTROL)
 *   It changes the SOLVED CONFIGURATION (knee up vs down). Two valid
 *   mathematical solutions, only one is realised — that's part of
 *   the math, not of the camera.
 *
 * Things that DO NOT live here
 *   - Link lengths L1_PX, L2_PX → §1 config (compile-time constants).
 *   - Target step size KEY_STEP_PX → §1 config.
 *   - fps counter → main() local (display-only, not part of the scene).
 *   - Signal flags g_running / g_need_resize → §8 file-scope (must
 *     be writable from a signal handler that takes no user pointer).
 *
 * References [1] Craig §4 (IK casework + solvability); [4] Buss for
 *   the Jacobian alternative this file is the foil for.
 */
typedef struct {
    /* ── Simulation state ─────────────────────────────────────── *
     * Anchor + inputs + cached IK output. scene_input writes T   *
     * and elbow_up; solve_ik reads (S, T, elbow_up) and writes E.*/
    Vec2 shoulder;       /* S — pinned origin; resets on SIGWINCH    */
    Vec2 target;         /* T — user-controlled; clamped to reach    */
    Vec2 elbow;          /* E — cached IK output, refreshed per frame*/
    bool elbow_up;       /* selects one of two valid solutions       */

    /* ── Rendering / control state ────────────────────────────── *
     * Read only by scene_draw and the HUD. paused gates the sim   *
     * but not the renderer; rows/cols clip draws to terminal.     */
    bool paused;         /* user-toggled freeze; sim skipped, draw same */
    int  rows, cols;     /* terminal extent in CHARACTER CELLS         */
} Scene;

static Vec2 clamp_to_screen(Vec2 p, int rows, int cols)
{
    return vec2(clampf(p.x, 0.0f, cells_to_px_w(cols) - 1.0f),
                clampf(p.y, 0.0f, cells_to_px_h(rows) - 1.0f));
}

/*
 * clamp_to_reach — pull T back onto the reach disc if it has drifted
 * outside. Keeping T inside the disc means the law of cosines always
 * returns a valid triangle and the forearm |E − T| is always exactly
 * L2 — the visible arm length never grows.
 */
static Vec2 clamp_to_reach(Vec2 t, Vec2 shoulder, float reach)
{
    Vec2  d   = vec2_sub(t, shoulder);
    float len = vec2_len(d);
    if (len <= reach) return t;
    return vec2_add(shoulder, vec2_scl(d, reach / len));
}

/*
 * scene_init — anchor S at screen centre, spawn T inside the reach
 * disc, default elbow-down, run solve_ik once so scene.elbow is
 * valid on frame zero.
 */
static void scene_init(Scene *s, int rows, int cols)
{
    s->rows     = rows;
    s->cols     = cols;
    s->paused   = false;
    s->elbow_up = false;

    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);

    float reach = L1_PX + L2_PX;
    s->target   = vec2(s->shoulder.x + reach * INITIAL_TARGET_REACH_FRAC,
                       s->shoulder.y);

    s->elbow = solve_ik(s->shoulder, s->target, s->elbow_up);
}

static void scene_resize(Scene *s, int rows, int cols)
{
    s->rows     = rows;
    s->cols     = cols;
    s->shoulder = vec2(cells_to_px_w(cols) * 0.5f,
                       cells_to_px_h(rows) * 0.5f);
    s->target   = clamp_to_screen(s->target, rows, cols);
    s->target   = clamp_to_reach (s->target, s->shoulder, L1_PX + L2_PX);
    s->elbow    = solve_ik(s->shoulder, s->target, s->elbow_up);
}

/*
 * scene_input — translate one keypress.
 *   arrows  nudge T (ignored when paused)
 *   f       flip elbow side
 *   space   toggle pause
 *   r       reset
 * There is no key for moving E — E is solver output, not user input.
 */
static void scene_input(Scene *s, int ch)
{
    const float k = KEY_STEP_PX;

    if (!s->paused) {
        switch (ch) {
            case KEY_LEFT:  s->target.x -= k; break;
            case KEY_RIGHT: s->target.x += k; break;
            case KEY_UP:    s->target.y -= k; break;
            case KEY_DOWN:  s->target.y += k; break;
            default: break;
        }
        s->target = clamp_to_screen(s->target, s->rows, s->cols);
        s->target = clamp_to_reach (s->target, s->shoulder, L1_PX + L2_PX);
    }

    switch (ch) {
        case 'f': s->elbow_up = !s->elbow_up; break;
        case ' ': s->paused   = !s->paused;   break;
        case 'r': scene_init(s, s->rows, s->cols); break;
        default: break;
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
 * scene_draw — the five draw calls from the pseudocode skeleton.
 * Lines first, markers on top; '@' last so the pinned shoulder
 * always wins the cell contest at S.
 */
static void scene_draw(const Scene *s)
{
    Vec2 S = s->shoulder, E = s->elbow, T = s->target;

    draw_line (S, E, PAIR_LINK_UPPER, s->rows, s->cols);
    draw_line (E, T, PAIR_LINK_FORE,  s->rows, s->cols);

    draw_point(T, '*', PAIR_TARGET,   s->rows, s->cols);
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
    typeahead(-1);              /* don't let stdin interrupt frame writes */
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
 * screen_hud — required HUD: status row 0, hint row last. Row 1
 * shows |E − T|, which should remain pinned at exactly L2 every
 * frame. If it ever drifts, the solver has a bug.
 */
static void screen_hud(const Scene *s, float fps)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);

    char  buf[96];
    float fore_len = vec2_dist(s->elbow, s->target);

    snprintf(buf, sizeof buf, " %5.1f fps  closed-form IK  %s ",
             fps, s->paused ? "PAUSED " : "running");
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    snprintf(buf, sizeof buf,
             " L1:%.0f  L2:%.0f  reach:%.0fpx  |E-T|:%5.1fpx  elbow:%s ",
             L1_PX, L2_PX, L1_PX + L2_PX, fore_len,
             s->elbow_up ? "up  " : "down");
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cols - (int)strlen(buf), "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc:pause  r:reset  arrows:move target  f:flip elbow ");
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

        /* T = read_input() */
        for (int ch; (ch = getch()) != ERR; ) {
            if (ch == 'q' || ch == 27 /*ESC*/) { g_running = 0; break; }
            scene_input(&scene, ch);
        }

        /* E = solveIK(S, T)  — cached in scene.elbow */
        scene.elbow = solve_ik(scene.shoulder, scene.target, scene.elbow_up);

        /* clear / draw / present */
        erase();
        scene_draw(&scene);
        screen_hud(&scene, fps);
        screen_present();

        int64_t elapsed = clock_ns() - t_now;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    return 0;
}
