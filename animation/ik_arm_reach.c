/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_arm_reach.c — FABRIK Robotic Arm with Lissajous Figure-8 Target
 *
 * DEMO: A 4-link robotic arm is anchored at screen centre.  Its end
 *       effector tracks a target that traces a Lissajous figure-8 (∞)
 *       path autonomously.  When the target is reachable, the FABRIK
 *       iterative IK solver bends the chain each tick to track it.
 *       When the target moves beyond the total chain reach, the arm
 *       straightens toward it (maximum-extension configuration) and a
 *       yellow limit circle appears showing exactly how far the arm can
 *       extend.  A faint red trail of recent target positions reveals
 *       the figure-8 shape being traced.
 *
 * STUDY ALONGSIDE: snake_forward_kinematics.c (FK reference — the path-
 *                  following complement to IK)
 *                  framework.c (canonical loop / timing template)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — themed palettes + dedicated HUD pair
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Arm: FABRIK IK solver, Lissajous target, renderer
 *       §5a  vec2 helpers       — length, normalise
 *       §5b  FABRIK solver      — reachability, forward pass, backward pass
 *       §5c  target motion      — Lissajous figure-8 update + trail push
 *       §5d  rendering helpers  — bead filler, reach-limit circle
 *       §5e  render_arm         — full frame composition
 *   §6  scene         — scene_init / scene_tick / scene_draw wrappers
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 1: FABRIK INVERSE KINEMATICS
 * ═══════════════════════════════════════════════════════════════════════
 *
 * FORWARD KINEMATICS (FK) vs INVERSE KINEMATICS (IK)
 * ────────────────────────────────────────────────────
 * In FK (see snake_forward_kinematics.c) you specify the joint angles
 * and the algorithm computes where the tip ends up.  The direction is
 * angles → tip position.  FK has a closed-form solution and always
 * terminates in O(N) time.
 *
 * IK is the reverse problem: given where you WANT the tip to be, find
 * the joint angles that place it there.  The direction is
 * desired tip position → joint angles.
 *
 * WHY IK IS HARDER THAN FK (the Jacobian problem)
 * ─────────────────────────────────────────────────
 * The classical analytical solution to IK for a 2-link chain uses the
 * law of cosines and produces two closed-form solutions (elbow-up,
 * elbow-down).  For N > 2 links the system is underdetermined: there
 * are infinitely many angle configurations that place the tip at the
 * same point.  The standard algebraic approach uses the Jacobian matrix
 * (J) of partial derivatives ∂tip/∂θᵢ and solves:
 *
 *   Δθ = J⁺ · Δtip
 *
 * where J⁺ is the Moore-Penrose pseudo-inverse.  This requires a matrix
 * inversion (O(N³)) every frame — expensive, numerically fragile near
 * singularities, and hard to implement robustly.
 *
 * THE FABRIK APPROACH — Forward And Backward Reaching Inverse Kinematics
 * ────────────────────────────────────────────────────────────────────────
 * FABRIK (Aristidou & Lasenby, 2011) solves IK without any matrix
 * computation at all.  It operates purely on joint POSITIONS (not angles)
 * and uses two simple geometric passes each iteration:
 *
 *  ┌─ REACHABILITY CHECK (run once, before any iteration) ──────────────┐
 *  │  dist = |target − root|                                            │
 *  │  If dist > Σ link_len:                                             │
 *  │    The target is outside the arm's reach sphere.  The best the arm │
 *  │    can do is point straight at the target (all links collinear).   │
 *  │    Straighten: pos[i+1] = pos[i] + dir × link_len[i]              │
 *  │    where dir = normalise(target − root).                           │
 *  │    Return immediately — no FABRIK iterations needed.               │
 *  └────────────────────────────────────────────────────────────────────┘
 *
 *  ┌─ FORWARD PASS — pull tip to target, preserve link lengths ─────────┐
 *  │  The TIP is moved TO the target:                                   │
 *  │    pos[N-1] = target                                               │
 *  │  Then, from tip toward root (i = N-2 down to 0):                  │
 *  │    fx, fy = pos[i] − pos[i+1]          (direction from tip joint) │
 *  │    r      = link_len[i] / |fx, fy|      (scale to restore length) │
 *  │    pos[i] = pos[i+1] + (fx, fy) × r    (slide pos[i] onto arc)   │
 *  │  After this pass the tip is exactly at target and every consecutive│
 *  │  pair is exactly link_len[i] apart.  But pos[0] (the root) has    │
 *  │  moved away from its fixed anchor — the root constraint is broken. │
 *  └────────────────────────────────────────────────────────────────────┘
 *
 *  ┌─ BACKWARD PASS — restore root, propagate the fix forward ──────────┐
 *  │  The ROOT is snapped back to its fixed anchor:                     │
 *  │    pos[0] = root                                                   │
 *  │  Then, from root toward tip (i = 0 to N-2):                       │
 *  │    bx, by = pos[i+1] − pos[i]          (direction toward tip)     │
 *  │    r      = link_len[i] / |bx, by|     (scale to restore length)  │
 *  │    pos[i+1] = pos[i] + (bx, by) × r   (slide pos[i+1] onto arc)  │
 *  │  After this pass the root is exactly at anchor and every consecutive│
 *  │  pair is exactly link_len[i] apart.  The tip may have moved a     │
 *  │  little away from target — but it is now CLOSER than before.      │
 *  └────────────────────────────────────────────────────────────────────┘
 *
 *  ┌─ CONVERGENCE CHECK ─────────────────────────────────────────────────┐
 *  │  err = |pos[N-1] − target|                                         │
 *  │  If err < CONV_TOL (1.5 px): DONE — the tip is within one pixel   │
 *  │  of the target; further iteration is sub-cell and imperceptible.   │
 *  │  Otherwise: repeat forward + backward passes (up to MAX_ITER=15). │
 *  └────────────────────────────────────────────────────────────────────┘
 *
 * WHY FABRIK CONVERGES
 * ─────────────────────
 * Each forward pass brings the tip closer to target (by definition: we
 * move tip exactly to target).  Each backward pass restores the root
 * while preserving all link lengths — this can only push the tip away
 * from target by a bounded amount determined by the chain geometry.
 *
 * Because the target is within reach (reachability check passed), a
 * valid solution exists.  FABRIK is provably monotonically convergent:
 * each full iteration (forward + backward) strictly decreases the
 * distance |tip − target|.  In practice, a 4-link chain converges to
 * sub-pixel accuracy in 3-5 iterations — never 15.  MAX_ITER=15 is a
 * safety cap for degenerate configurations (e.g., links nearly collinear
 * pointed away from target) where convergence is slower.
 *
 * FABRIK vs JACOBIAN: why FABRIK wins for real-time animation
 *   • No matrix inverse: O(N) per iteration vs O(N³)
 *   • No singularity issues: geometric ops are always well-defined
 *     (only degenerate case is coincident joints, handled with guard)
 *   • Intuitive, easy to add joint angle constraints later
 *   • Converges in very few iterations for typical configurations
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 2: LISSAJOUS FIGURE-8 TARGET PATH
 * ═══════════════════════════════════════════════════════════════════════
 *
 * The target's position is given by a Lissajous curve:
 *
 *   x(t) = root_px + Ax · cos(fx · t)
 *   y(t) = root_py + Ay · sin(fy · t + φ)
 *
 * WHY TWO DIFFERENT FREQUENCIES CREATE A FIGURE-8
 * ─────────────────────────────────────────────────
 * With fx = 1, fy = 2:  the x-axis completes one full oscillation
 * while the y-axis completes two.  This 1:2 frequency ratio means the
 * x-component passes through its midpoint (cos=0) exactly TWICE per
 * cycle — once going right, once going left.  Each time x crosses the
 * centre, y has returned to the centre coming from opposite vertical
 * directions.  The path therefore crosses itself once at the centre,
 * creating two lobes — the classic figure-8 (∞ shape).
 *
 * General rule: a Lissajous figure with frequency ratio p:q has at most
 * (p + q − 2) self-intersections.  For 1:2: (1 + 2 − 2) = 1 crossing,
 * giving exactly the single-crossing figure-8 shape.
 *
 * The phase φ = π/4 (LIS_PHASE = 0.785 rad) rotates the figure
 * slightly so the loop entry and exit are not tangent (which would make
 * the crossing a cusp rather than a clean X intersection).  This gives
 * the arm's reach transitions a visually appealing diagonal entry.
 *
 * PERIOD OF THE FIGURE-8
 * ───────────────────────
 * The full figure-8 repeats when BOTH x and y return to their starting
 * positions simultaneously.  x repeats with period Tx = 2π/fx = 2π.
 * y repeats with period Ty = 2π/fy = π.  The joint period (LCM of Tx
 * and Ty) is Tx = 2π.  With LIS_SPEED_DEFAULT = 0.7:
 *   scene_time advances at 0.7 × wall_seconds
 *   One full figure-8 = 2π / 0.7 ≈ 8.98 seconds of wall time.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 3: LINK LENGTH TAPERING
 * ═══════════════════════════════════════════════════════════════════════
 *
 * The four links have lengths proportional to weights {0.32, 0.27, 0.23, 0.18}
 * applied to arm_reach = 60% of the shorter screen dimension.
 * Sum of weights = 1.0, so total_len = arm_reach exactly.
 *
 * ROOT LINK (link 0, weight 0.32) is the longest.
 * TIP LINK  (link 3, weight 0.18) is the shortest — 56% of root.
 *
 * WHY TAPER?
 *   Constant-length links look robotic and uniform.  Tapering toward the
 *   tip mimics the biomechanical structure of animal limbs (upper arm >
 *   forearm > hand > finger) and of industrial robot arms (large base
 *   actuator > wrist > gripper).  Visually, it draws the eye toward the
 *   tip and gives the arm a sense of weight and hierarchy.
 *
 *   Mathematically, tapering also makes the FABRIK solver converge
 *   faster: because the tip link is shortest, the final adjustment step
 *   has less distance to correct, so the tip reaches tolerance faster.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 4: REACH LIMIT CIRCLE
 * ═══════════════════════════════════════════════════════════════════════
 *
 * When |target − root| > total_len (arm fully stretched cannot reach),
 * a circle of radius = total_len is drawn centred on the root joint.
 * This visualises the reach envelope — the boundary beyond which the
 * arm cannot place its tip.
 *
 * RENDERING APPROACH: Instead of drawing every pixel on the circle
 * (expensive, and many adjacent pixels map to the same terminal cell),
 * 48 evenly-spaced angular samples are taken:
 *   angle_k = k × 2π / 48,  k = 0…47
 *   px = root_px + total_len × cos(angle_k)
 *   py = root_py + total_len × sin(angle_k)
 * Each sample is converted to a cell and stamped with '.' (dim yellow).
 * 48 points on a circle of radius R px covers the circle at spacing
 * R × 2π/48 ≈ R/7.6 px.  For a typical arm reach of 300 px this gives
 * ~39 px between consecutive dots — about 5 cell-widths — producing a
 * clearly dashed circle without overlapping dots.
 *
 * Keys:
 *   q / ESC         quit
 *   space           pause / resume
 *   + / -           Lissajous speed faster / slower (×1.25 per press)
 *   t               cycle color theme
 *   [ / ]           simulation Hz down / up
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ik_arm_reach.c -o ik_arm_reach -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : FABRIK iterative IK.  Two geometric passes per
 *                  iteration: (1) FORWARD — tip snapped to target,
 *                  link lengths restored root→tip direction reversed;
 *                  (2) BACKWARD — root re-anchored, link lengths restored
 *                  root→tip.  Each iteration strictly reduces tip error.
 *                  Converges in 3–5 iterations for a 4-link chain;
 *                  MAX_ITER=15 caps degenerate-configuration cost.
 *                  No Jacobian matrix, no trigonometry, no singularities.
 *                  Reachability check (|target−root| vs Σ link_len)
 *                  fires once per tick and short-circuits to a straight
 *                  stretch posture when the target is out of reach.
 *
 * Data-structure : Vec2 pos[N_JOINTS] — current joint positions (N=5
 *                  joints, 4 links).  Vec2 prev_pos[N_JOINTS] — snapshot
 *                  at tick start for sub-tick alpha lerp.  Lissajous trail
 *                  ring buffer (TRAIL_SIZE=60 entries) stores recent target
 *                  positions for the faint figure-8 path display.
 *                  All positions in square pixel space; converted to
 *                  cell coordinates only at draw time (see §4).
 *
 * Rendering      : Two-pass bead render per link: pass 1 stamps 'o' at
 *                  DRAW_STEP_PX=5 px intervals (dense fill); pass 2
 *                  overlays '0'/'o'/'.' joint markers.  Colour gradient
 *                  runs root (dark) → tip (cyan/bright).  Target drawn as
 *                  '+' (reachable) or 'X' (out of reach) in bright red.
 *                  Reach limit circle: 48 sampled '.' dots in dim yellow.
 *                  Alpha lerp between prev/current positions gives smooth
 *                  sub-tick motion regardless of render vs physics Hz.
 *
 * Performance    : Fixed-step accumulator (§8) decouples physics (60 Hz
 *                  default) from render (capped at 60 fps).  FABRIK
 *                  worst-case: MAX_ITER=15 × N_JOINTS=5 × 2 passes =
 *                  150 simple arithmetic ops per tick — trivial at 60 Hz.
 *                  ncurses doupdate() sends only changed cells to the
 *                  terminal fd; typically <50 cells change per frame.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * _POSIX_C_SOURCE 200809L exposes it on most systems, but if a toolchain
 * omits it we define our own so the build never fails.
 */
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

/*
 * All magic numbers live here.  Change behaviour by editing this block
 * only — never scatter literals through the code.
 */
enum {
    /*
     * SIM_FPS_DEFAULT — target physics tick rate in Hz.
     * The fixed-step accumulator in §8 fires scene_tick() exactly this
     * many times per wall-clock second, regardless of render frame rate.
     *
     * Raising sim Hz makes IK tracking smoother (more FABRIK solves per
     * second → tip follows target more closely in fast motion) but costs
     * more CPU.  [10, 120] is the user-adjustable range; 60 Hz default
     * is comfortable and keeps physics aligned with the 60-fps render cap
     * so one physics tick fires per rendered frame under normal load.
     */
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,   /* step size for [ and ] keys              */

    /*
     * HUD_COLS — byte budget for the status bar string.
     * The longest possible HUD line (all values at max) fits in 96 bytes.
     * snprintf(buf, sizeof buf, ...) truncates safely if ever exceeded.
     *
     * FPS_UPDATE_MS — how often the displayed fps value is recalculated.
     * 500 ms gives a stable reading without flickering each frame.
     * The fps counter accumulates frame_count and elapsed ns over this
     * window, then divides: fps = frame_count / (elapsed_ns / 1e9).
     */
    HUD_COLS         =  96,
    FPS_UPDATE_MS    = 500,

    /*
     * N_ARM_COLORS — number of gradient color pairs for the arm links
     * (ncurses pairs 1–5, one per link plus tip glow).
     *
     * Color pair semantics:
     *   pairs 1–5  : arm gradient (root=dark → tip=bright), set by theme
     *   pair 6     : target marker (bright red — semantic; never changes)
     *   pair 7     : reach limit circle (yellow — semantic; never changes)
     *   pair 8     : HUD status bar (varies per theme via HUD_PAIR)
     *
     * Separating semantic pairs (6, 7) from theme pairs (1–5, 8) ensures
     * that cycling themes never accidentally changes the target or reach-
     * circle colors, which carry specific meaning to the viewer.
     */
    N_ARM_COLORS     =   5,
    HUD_PAIR         =   8,   /* status bar color pair index             */

    /*
     * N_THEMES — number of selectable color palettes.
     * Press 't' to cycle forward through all ten.
     */
    N_THEMES         =  10,

    /*
     * N_JOINTS — total joint positions in the arm chain.
     *   pos[0]       — root anchor (fixed; never moved by solver)
     *   pos[1..N-2]  — interior joints (moved by FABRIK each iteration)
     *   pos[N-1]     — end effector / tip (must reach target)
     *
     * N_LINKS = N_JOINTS − 1 — number of rigid segments connecting them.
     * With N_JOINTS=5, N_LINKS=4: four links, five joint positions.
     * Each link has its own length (see link_len[] in Arm struct).
     */
    N_JOINTS         =   5,
    N_LINKS          =   4,

    /*
     * MAX_ITER — maximum FABRIK iterations per physics tick.
     *
     * FABRIK converges monotonically: each iteration strictly reduces
     * the tip-to-target error.  For a 4-link chain at a reachable target,
     * empirical testing shows convergence to CONV_TOL in 3–5 iterations
     * under typical motion.  15 is a safety cap for near-singular configs
     * (e.g., target near the workspace boundary, arm nearly fully extended)
     * where convergence is slower.  Cost is bounded: 15 × 5 × 2 passes =
     * 150 simple arithmetic ops, which is negligible at 60 Hz.
     */
    MAX_ITER         =  15,

    /*
     * TRAIL_SIZE — capacity of the Lissajous target position trail buffer.
     *
     * At SIM_FPS_DEFAULT=60 Hz and TRAIL_SIZE=60, the trail holds exactly
     * 1 second of target history.  This shows roughly 1/9 of the full
     * figure-8 cycle (period ≈ 9 s at LIS_SPEED_DEFAULT=0.7), which is
     * enough to see the curve shape without cluttering the screen.
     *
     * Increasing TRAIL_SIZE shows more of the figure-8 path at the cost
     * of more '.' dots on screen.  The trail is stored as a ring buffer
     * so the memory cost is fixed: TRAIL_SIZE × sizeof(Vec2) = 60 × 8 = 480 B.
     */
    TRAIL_SIZE       =  60,
};

/*
 * CONV_TOL — FABRIK convergence tolerance in pixel space.
 *
 * The FABRIK loop terminates early when |tip − target| < CONV_TOL.
 * 1.5 px is deliberately chosen to be sub-cell in both axes:
 *   Horizontal: 1.5 / CELL_W (8 px) = 0.19 of a cell column.
 *   Vertical:   1.5 / CELL_H (16 px) = 0.09 of a cell row.
 * Once the tip is within 1.5 px of target, the terminal display shows
 * them in the same cell — further solver iterations produce no visible
 * improvement and waste CPU.
 */
#define CONV_TOL    1.5f

/*
 * Lissajous figure-8 frequency and phase parameters.
 *
 * The target traces:
 *   x(t) = root_px + lis_ax × cos(LIS_FX × scene_time)
 *   y(t) = root_py + lis_ay × sin(LIS_FY × scene_time + LIS_PHASE)
 *
 * LIS_FX = 1.0 — x oscillates once per period T = 2π/1 = 2π seconds
 *   of scene_time.  cos(t) moves the target smoothly left→right→left.
 *
 * LIS_FY = 2.0 — y oscillates TWICE per period: sin(2t + φ) completes
 *   two full vertical oscillations while x completes one horizontal.
 *   This 1:2 frequency ratio is the geometric cause of the figure-8:
 *   every time the x-component passes through the centre (cos=0,
 *   two times per cycle), the y-component has returned to the centre
 *   from opposite vertical directions.  The path therefore crosses
 *   itself exactly once at the origin, forming two lobes.
 *
 * LIS_PHASE = π/4 ≈ 0.785 rad — initial phase offset for the y sinusoid.
 *   Without phase offset (φ=0): both components start at their extremes
 *   simultaneously, creating a tangent self-crossing (a figure-8 with a
 *   cusp) that looks like two separate ovals touching.
 *   With φ=π/4: the crossing is a clean X intersection with equal entry/
 *   exit angles, giving the classic smooth ∞ shape.
 *
 * Amplitudes lis_ax, lis_ay are NOT constants — they are computed at
 * scene_init() from terminal size (40% of screen width/height in pixels)
 * so the figure-8 scales proportionally to any terminal size.
 */
#define LIS_FX               1.0f   /* x-axis angular frequency (rad / s of scene_time) */
#define LIS_FY               2.0f   /* y-axis angular frequency — exactly 2× LIS_FX     */
#define LIS_PHASE            0.785f /* y phase offset ≈ π/4; ensures clean X crossing   */

/*
 * Lissajous speed parameters — control how fast scene_time advances.
 *
 * scene_time += dt × speed_scale each physics tick.
 * One complete figure-8 requires scene_time to advance by 2π (the period
 * of the x-component, which sets the longest period in the 1:2 ratio).
 *
 * LIS_SPEED_DEFAULT = 0.7:
 *   One full figure-8 = 2π / 0.7 ≈ 8.98 wall-clock seconds.
 *   At 60 Hz that is 8.98 × 60 = 539 ticks per cycle — smooth enough
 *   that the arm tracks without visible stepping, slow enough to watch.
 *
 * LIS_SPEED_MIN = 0.05: ~125 s per cycle — extremely slow, good for
 *   watching the FABRIK solver in near-static detail.
 *
 * LIS_SPEED_MAX = 5.0: ~1.26 s per cycle — fast enough that the arm
 *   oscillates visibly but the IK solver keeps up at 60 Hz.
 *
 * The + and - keys multiply/divide speed_scale by 1.25 per press,
 * giving 9 doublings from min to max (5.0 / 0.05 = 100 ≈ 2^6.6).
 */
#define LIS_SPEED_DEFAULT    0.7f    /* comfortable tracking speed          */
#define LIS_SPEED_MIN        0.05f   /* near-static — study solver detail   */
#define LIS_SPEED_MAX        5.0f    /* fast oscillation — stress-test IK   */

/*
 * DRAW_STEP_PX — bead fill step size along each arm link segment (pixels).
 *
 * The link renderer (draw_link_beads) walks each link from joint[i] to
 * joint[i+1] in increments of DRAW_STEP_PX, stamping 'o' at each sample.
 * The step must be small enough that no terminal cell is ever skipped.
 *
 * Critical constraint: DRAW_STEP_PX < CELL_W (8 px).
 * With 5 px: at worst a near-horizontal link steps 5/8 = 0.625 cells
 * per sample — every cell is covered multiple times.  For near-vertical
 * links, CELL_H=16 px is the relevant constraint: 5/16 = 0.3 cells/step
 * — still dense enough to fill every row the link passes through.
 *
 * Larger DRAW_STEP_PX → sparser beads (gaps between 'o' chars visible).
 * Smaller DRAW_STEP_PX → denser beads, but more mvwaddch calls per frame.
 * 5 px is a deliberate choice to show discrete beads for the IK chain
 * (unlike snake_forward_kinematics which uses 3 px for a solid-line look).
 */
#define DRAW_STEP_PX   5.0f

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS: unit-conversion constants for nanosecond math.
 *   NS_PER_SEC = 1 000 000 000 (10⁹)
 *   NS_PER_MS  =     1 000 000 (10⁶)
 *
 * TICK_NS(f): converts frequency f (Hz) to tick period in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 ≈ 16 666 667 ns = 16.67 ms.
 * Used in the fixed-step accumulator (see §8 main()) to know how many
 * nanoseconds to drain from sim_accum per physics tick.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between pixel space
 * and display (see §4 for full explanation of why this matters).
 *
 * CELL_W = 8 px:  physical pixels per terminal column.
 * CELL_H = 16 px: physical pixels per terminal row.
 *
 * Terminal cells are not square — a typical cell is exactly twice as tall
 * as it is wide.  Physics (FABRIK, Lissajous) runs in a square pixel
 * space where 1 unit = 1 physical pixel in both x and y.  Converting to
 * cell coordinates (col = px_x / CELL_W, row = px_y / CELL_H) applies
 * this 2:1 correction so distances and angles look correct on screen.
 */
#define CELL_W   8    /* physical pixels per terminal column */
#define CELL_H  16    /* physical pixels per terminal row    */

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC is guaranteed never to go backward (unlike
 * CLOCK_REALTIME, which can jump on NTP corrections or DST changes).
 * Subtracting two consecutive clock_ns() values gives the true elapsed
 * wall time regardless of system load or clock adjustments.
 *
 * Returns int64_t (signed 64-bit) so dt differences are naturally
 * signed; negative values (from the pause-guard cap) do not wrap around.
 * int64_t holds up to ±9.2 × 10¹⁸ ns = ±292 years — no overflow risk.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep for exactly ns nanoseconds.
 *
 * Called BEFORE render in §8 to cap the frame rate at 60 fps.  Sleeping
 * before (not after) terminal I/O means the terminal write cost is not
 * charged against the next frame's physics budget.
 *
 * If ns ≤ 0 the frame is already over-budget (FABRIK + other work took
 * longer than one frame): skip the sleep so we recover as fast as possible
 * rather than sleeping a negative or zero duration (nanosleep(0) is
 * harmless but wastes a syscall).
 */
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
/* §3  color — themed palettes + dedicated HUD pair                       */
/* ===================================================================== */

/*
 * Theme — one named color palette for the arm.
 *
 *   name       : display name shown in the HUD.
 *   arm[0..4]  : xterm-256 foreground color indices for ncurses pairs 1–5.
 *                arm[0] = root link (darkest), arm[4] = tip link (brightest).
 *                The gradient from dark to bright visually emphasises
 *                the root→tip hierarchy and makes the tip stand out.
 *   hud        : xterm-256 foreground index for HUD_PAIR (pair 8).
 *
 * Pairs 6 (target, red=196) and 7 (reach circle, yellow=226) are NOT
 * part of the theme — they are fixed semantic colors applied in color_init()
 * and never overwritten by theme_apply().
 */
typedef struct {
    const char *name;
    int arm[N_ARM_COLORS];   /* pairs 1–5: root link → tip glow */
    int hud;                 /* pair 8: HUD status bar           */
} Theme;

/*
 * THEMES[10] — ten selectable palettes.  Press 't' to cycle forward.
 *
 * Each palette is designed to give the arm a coherent industrial, natural,
 * or dramatic appearance.  The darkest color goes on pair 1 (root link)
 * and the brightest on pair 5 (tip link / end-effector glow):
 *
 *   0 Steel  — dark grey → cyan: industrial robot default
 *   1 Matrix — phosphor green gradient: classic terminal aesthetic
 *   2 Fire   — deep red → orange: hot forge aesthetic
 *   3 Ocean  — deep navy → aqua: deep-sea submersible
 *   4 Nova   — deep violet → pink: cosmic/plasma
 *   5 Toxic  — dark olive → neon lime: biohazard glow
 *   6 Lava   — dark red → amber: molten metal
 *   7 Ghost  — dark grey → white: minimal monochrome
 *   8 Aurora — teal → gold: northern lights
 *   9 Neon   — blue-violet → hot pink: synthwave/retro
 */
static const Theme THEMES[N_THEMES] = {
    { "Steel",  {240, 244, 248, 252,  51}, 226 },
    { "Matrix", { 22,  28,  34,  40,  46},  46 },
    { "Fire",   { 52,  88, 124, 160, 196}, 208 },
    { "Ocean",  { 17,  20,  27,  33,  51},  51 },
    { "Nova",   { 54,  93, 129, 165, 201}, 213 },
    { "Toxic",  { 58,  64,  70,  76,  82}, 118 },
    { "Lava",   { 52,  94, 130, 166, 202}, 208 },
    { "Ghost",  {234, 238, 242, 246, 250}, 252 },
    { "Aurora", { 23,  35,  71, 107, 143}, 221 },
    { "Neon",   { 57,  93, 129, 165, 201}, 197 },
};

/*
 * theme_apply() — rebind arm color pairs (1–5) and HUD pair (8) live.
 *
 * Called by 't' keypress and during init.  Pairs 6 and 7 are never
 * touched here — they retain their fixed semantic colors (red, yellow).
 *
 * Falls back to no-op on 8-color terminals (COLORS < 256) because the
 * 256-color xterm palette is not available; color_init() installs basic
 * fallback pairs for those terminals instead.
 */
static void theme_apply(int idx)
{
    if (COLORS < 256) return;   /* fallback pairs set by color_init(); leave them */
    const Theme *t = &THEMES[idx];
    for (int p = 0; p < N_ARM_COLORS; p++)
        init_pair(p + 1, t->arm[p], COLOR_BLACK);
    init_pair(HUD_PAIR, t->hud, COLOR_BLACK);
}

/*
 * color_init() — one-time ncurses color setup.
 *
 * On 256-color terminals: applies the initial theme (pairs 1–5, 8) and
 * sets fixed semantic pairs:
 *   pair 6 = foreground 196 (bright red) on black  — target marker
 *   pair 7 = foreground 226 (yellow)     on black  — reach limit circle
 *
 * On 8-color terminals: installs basic COLOR_* approximations for all
 * pairs so the simulation is still usable (just less beautiful).
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        theme_apply(initial_theme);
        init_pair(6, 196, COLOR_BLACK);   /* bright red  — target marker     */
        init_pair(7, 226, COLOR_BLACK);   /* yellow      — reach limit circle */
    } else {
        /* 8-color fallback: no 256-color xterm palette available */
        init_pair(1, COLOR_WHITE,  COLOR_BLACK);
        init_pair(2, COLOR_WHITE,  COLOR_BLACK);
        init_pair(3, COLOR_WHITE,  COLOR_BLACK);
        init_pair(4, COLOR_WHITE,  COLOR_BLACK);
        init_pair(5, COLOR_CYAN,   COLOR_BLACK);
        init_pair(6, COLOR_RED,    COLOR_BLACK);
        init_pair(7, COLOR_YELLOW, COLOR_BLACK);
        init_pair(8, COLOR_YELLOW, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — pixel↔cell; the one aspect-ratio fix                     */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide
 * in physical pixels (CELL_H=16 vs CELL_W=8).  If arm joint positions
 * were stored directly in cell coordinates, a displacement of (dx, dy)
 * cells would travel twice as far vertically as horizontally in physical
 * space.  The Lissajous figure-8 would appear squashed, and the FABRIK
 * link lengths would be wrong (a nominally 100-px link in cell coords
 * would be 100×8=800 px wide but 100×16=1600 px tall physically).
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions (pos[], prev_pos[], target, trail[]) are in pixel
 *   space where 1 unit = 1 physical pixel.  The pixel grid is square and
 *   isotropic: moving 1 unit in x or y covers the same physical distance.
 *   Only at draw time does §5d convert to cell coordinates via these helpers.
 *
 * px_to_cell_x / px_to_cell_y — round to nearest cell.
 *
 * Formula: cell = floor(px / CELL_DIM + 0.5)
 *   Adding 0.5 before flooring = "round half up" — deterministic and
 *   symmetric.  roundf() uses "round half to even" (banker's rounding)
 *   which can oscillate when px lands exactly on a cell boundary, causing
 *   a one-cell flicker.  Truncation ((int)(px/CELL_W)) always rounds down,
 *   giving asymmetric dwell at boundaries.  floor + 0.5 avoids both.
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

/*
 * Vec2 — lightweight 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention).
 * All physics calculations use this type so no coordinate confusion arises.
 */
typedef struct { float x, y; } Vec2;

/* ── §5a  vec2 helpers ──────────────────────────────────────────────── */

/*
 * vec2_len() — Euclidean magnitude of a 2-D vector.
 *
 * Used in the FABRIK solver to measure link lengths and compute the
 * scale factor r = link_len / current_distance.  Inlined for speed;
 * called hundreds of times per tick in the solver loop.
 */
static inline float vec2_len(Vec2 v)
{
    return sqrtf(v.x * v.x + v.y * v.y);
}

/*
 * vec2_norm() — unit-length normalisation of a 2-D vector.
 *
 * Returns (v.x / |v|, v.y / |v|), a vector of length 1.0 pointing
 * in the same direction as v.
 *
 * DEGENERATE GUARD: if |v| < 1e-6 (effectively zero-length), return
 * (1, 0) — a unit vector pointing east.  This handles the edge case
 * where two joints are at the same pixel position (e.g., on the very
 * first tick before FABRIK runs).  Returning any unit vector is
 * acceptable; (1,0) keeps the arm pointing right until the solver
 * separates the coincident joints on the next iteration.
 */
static inline Vec2 vec2_norm(Vec2 v)
{
    float l = vec2_len(v);
    if (l < 1e-6f) return (Vec2){1.0f, 0.0f};   /* degenerate: default east */
    return (Vec2){v.x / l, v.y / l};
}

/*
 * Arm — complete simulation state for the IK robotic arm.
 *
 * ── JOINT POSITIONS ────────────────────────────────────────────────────
 *
 *   pos[0]          Root anchor — the fixed shoulder joint.  Its pixel
 *                   position is set at scene_init() and NEVER modified
 *                   by the FABRIK solver (the backward pass snaps pos[0]
 *                   back to root each iteration — effectively read-only).
 *
 *   pos[1..N-2]     Interior joints — freely moved by FABRIK each tick.
 *                   Their positions emerge from the solver; there are no
 *                   explicit angle variables.  FABRIK works purely on
 *                   positions, converting implicitly to angles at draw time
 *                   by the direction of each segment.
 *
 *   pos[N-1]        End effector (tip) — the point that must reach target.
 *                   After convergence: |pos[N-1] − target| < CONV_TOL.
 *
 *   prev_pos[]      Snapshot of pos[] taken at the START of each tick
 *                   (before FABRIK runs).  The renderer lerps between
 *                   prev_pos and pos using alpha ∈ [0,1) to produce
 *                   sub-tick smooth motion at any render frame rate.
 *                   Without prev_pos, joints would visibly jump at each
 *                   tick boundary when render Hz > sim Hz.
 *
 * ── LINK GEOMETRY ──────────────────────────────────────────────────────
 *
 *   link_len[i]     Length of link i in pixel space (from joint i to
 *                   joint i+1).  Set at scene_init() from terminal size;
 *                   constant thereafter.  Decreasing root→tip (tapering).
 *
 *   total_len       Σ link_len[i] — the maximum reach of the arm.
 *                   If |target − root| > total_len, the arm is fully
 *                   extended and cannot reach.  Recomputed at init.
 *
 * ── LISSAJOUS TARGET ───────────────────────────────────────────────────
 *
 *   target          Current target pixel position (output of Lissajous).
 *                   Updated by update_target() every physics tick.
 *
 *   prev_target     Snapshot at tick start; used for alpha lerp of the
 *                   target marker ('+' or 'X') so it slides smoothly.
 *
 *   scene_time      Accumulated simulation time driving the Lissajous
 *                   formulas.  Advances at dt × speed_scale each tick.
 *                   Separated from wall time so speed_scale can stretch
 *                   or compress the figure-8 without affecting FABRIK.
 *
 *   speed_scale     Multiplier on scene_time advancement.  1.0 = default;
 *                   > 1 = faster figure-8; < 1 = slower.  Adjusted by +/-.
 *
 *   root_px, root_py  Pixel position of the root joint (Lissajous centre).
 *                   Also used as the FABRIK root anchor each iteration.
 *
 *   lis_ax, lis_ay  Lissajous amplitudes in pixel space.  Set at init
 *                   from terminal size (40% of screen width/height).
 *                   Clipped to ≤ total_len × 0.9 so part of the figure-8
 *                   path is always reachable (making the at_limit state
 *                   appear naturally when the target swings to the far lobe).
 *
 * ── STATE FLAGS ────────────────────────────────────────────────────────
 *
 *   at_limit        Set true by fabrik_solve() when |target − root| >
 *                   total_len.  When true: arm is fully extended and the
 *                   reach limit circle is drawn.  Cleared when the target
 *                   moves back inside the reach sphere.
 *
 *   paused          When true: scene_tick() skips update_target() and
 *                   fabrik_solve(); prev_pos is still saved so alpha lerp
 *                   freezes cleanly (prev = curr → lerp = identity).
 *
 * ── LISSAJOUS TRAIL ────────────────────────────────────────────────────
 *
 *   trail[]         Ring buffer of recent target pixel positions.
 *                   Entry pushed each physics tick by update_target().
 *                   Rendered as faint '.' dots (pair 6, A_DIM) to reveal
 *                   the figure-8 shape being traced.
 *
 *   trail_head      Write cursor (index of most recently pushed entry).
 *                   Advances modulo TRAIL_SIZE on each push.
 *
 *   trail_count     Number of valid entries, ≤ TRAIL_SIZE.
 *                   Reaches TRAIL_SIZE after TRAIL_SIZE ticks (~1 s at
 *                   60 Hz) and stays there for the life of the simulation.
 */
typedef struct {
    Vec2  pos[N_JOINTS];        /* current joint positions (pixels)         */
    Vec2  prev_pos[N_JOINTS];   /* snapshot for sub-tick alpha lerp         */
    Vec2  target;               /* current Lissajous target (pixels)        */
    Vec2  prev_target;          /* snapshot for alpha lerp of target marker */
    float link_len[N_LINKS];    /* link lengths in pixels (tapering root→tip)*/
    float scene_time;           /* accumulated time driving Lissajous       */
    float speed_scale;          /* Lissajous time multiplier (+ / - keys)   */
    float root_px, root_py;     /* root anchor pixel position               */
    float lis_ax, lis_ay;       /* Lissajous amplitudes (pixels)            */
    float total_len;            /* Σ link_len — reachability threshold       */
    bool  at_limit;             /* true when target is out of reach         */
    bool  paused;               /* physics frozen when true                 */
    int   theme_idx;            /* current palette index into THEMES[]      */

    /* Lissajous trail ring buffer for faint figure-8 path display */
    Vec2  trail[TRAIL_SIZE];
    int   trail_head;           /* write cursor (most recently pushed slot) */
    int   trail_count;          /* valid entries, ≤ TRAIL_SIZE              */
} Arm;

/* ── §5b  FABRIK solver ─────────────────────────────────────────────── */

/*
 * fabrik_solve() — run the FABRIK IK algorithm to move tip toward target.
 *
 * This is the computational heart of the simulation.  It is called once
 * per physics tick (from scene_tick()) and modifies a->pos[] in place.
 * a->target is read-only here; it was set by update_target() earlier in
 * the same tick.
 *
 * ─── STEP 0: REACHABILITY CHECK ────────────────────────────────────────
 *   Compute dist = |target − root|.
 *   If dist > total_len:
 *     No configuration of link angles can place the tip at target —
 *     the arm simply isn't long enough.  The best posture is to point
 *     straight at the target (all links collinear):
 *       dir = normalise(target − root)
 *       pos[0] = root
 *       pos[i+1] = pos[i] + dir × link_len[i]   for i = 0..N_LINKS-1
 *     Set at_limit = true and return.  No FABRIK iterations needed.
 *
 * ─── STEP 1: FORWARD PASS ──────────────────────────────────────────────
 *   Move the TIP to target exactly:
 *     pos[N_JOINTS-1] = target
 *   Then walk from tip toward root (i = N_JOINTS-2 downto 0):
 *     fx = pos[i].x − pos[i+1].x   (vector from i+1 toward i)
 *     fy = pos[i].y − pos[i+1].y
 *     flen = |(fx, fy)|
 *     r = link_len[i] / flen        (scale factor to restore link length)
 *     pos[i].x = pos[i+1].x + fx × r    (slide pos[i] to be exactly
 *     pos[i].y = pos[i+1].y + fy × r     link_len[i] from pos[i+1])
 *   After this pass: every consecutive pair is exactly link_len[i] apart,
 *   and pos[N_JOINTS-1] == target.  BUT: pos[0] has moved from the root
 *   anchor — the root constraint is violated.
 *
 * ─── STEP 2: BACKWARD PASS ─────────────────────────────────────────────
 *   Snap the ROOT back to the anchor:
 *     pos[0] = {root_px, root_py}
 *   Then walk from root toward tip (i = 0 to N_JOINTS-2):
 *     bx = pos[i+1].x − pos[i].x   (vector from i toward i+1)
 *     by = pos[i+1].y − pos[i].y
 *     blen = |(bx, by)|
 *     r = link_len[i] / blen
 *     pos[i+1].x = pos[i].x + bx × r    (slide pos[i+1] to be exactly
 *     pos[i+1].y = pos[i].y + by × r     link_len[i] from pos[i])
 *   After this pass: pos[0] == root and every pair is link_len[i] apart.
 *   The tip has moved slightly away from target — but closer than before.
 *
 * ─── STEP 3: CONVERGENCE CHECK ─────────────────────────────────────────
 *   tdx = pos[N-1].x − target.x
 *   tdy = pos[N-1].y − target.y
 *   If sqrt(tdx² + tdy²) < CONV_TOL: break — sub-pixel accuracy reached.
 *   Otherwise: repeat steps 1–2 (up to MAX_ITER total iterations).
 *
 * DIVISION GUARD: all division by flen/blen is guarded with a minimum
 * of 1e-6f to prevent division-by-zero when two joints are at the same
 * pixel (degenerate configuration).  The 1e-6 minimum produces a very
 * large r (≈ link_len / 1e-6), pushing the joint far apart — the solver
 * self-corrects on the next iteration.
 */
static void fabrik_solve(Arm *a)
{
    Vec2 root   = { a->root_px, a->root_py };
    Vec2 target = a->target;

    /* Step 0: reachability — can the chain span the distance to target? */
    float dx_rt = target.x - root.x;
    float dy_rt = target.y - root.y;
    float dist  = sqrtf(dx_rt * dx_rt + dy_rt * dy_rt);

    if (dist > a->total_len) {
        /* Target unreachable — stretch arm straight toward target */
        a->at_limit = true;
        Vec2 dir  = vec2_norm((Vec2){ dx_rt, dy_rt });
        a->pos[0] = root;
        for (int i = 0; i < N_LINKS; i++) {
            a->pos[i + 1].x = a->pos[i].x + dir.x * a->link_len[i];
            a->pos[i + 1].y = a->pos[i].y + dir.y * a->link_len[i];
        }
        return;
    }

    a->at_limit = false;

    /* FABRIK iteration loop */
    for (int iter = 0; iter < MAX_ITER; iter++) {

        /* Step 1: forward pass — snap tip to target, restore lengths backward */
        a->pos[N_JOINTS - 1] = target;
        for (int i = N_JOINTS - 2; i >= 0; i--) {
            float fx   = a->pos[i].x - a->pos[i + 1].x;
            float fy   = a->pos[i].y - a->pos[i + 1].y;
            float flen = sqrtf(fx * fx + fy * fy);
            if (flen < 1e-6f) flen = 1e-6f;   /* degenerate-joint guard */
            float r    = a->link_len[i] / flen;
            a->pos[i].x = a->pos[i + 1].x + fx * r;
            a->pos[i].y = a->pos[i + 1].y + fy * r;
        }

        /* Step 2: backward pass — restore root anchor, propagate forward */
        a->pos[0] = root;
        for (int i = 0; i < N_JOINTS - 1; i++) {
            float bx   = a->pos[i + 1].x - a->pos[i].x;
            float by   = a->pos[i + 1].y - a->pos[i].y;
            float blen = sqrtf(bx * bx + by * by);
            if (blen < 1e-6f) blen = 1e-6f;   /* degenerate-joint guard */
            float r    = a->link_len[i] / blen;
            a->pos[i + 1].x = a->pos[i].x + bx * r;
            a->pos[i + 1].y = a->pos[i].y + by * r;
        }

        /* Step 3: convergence — stop when tip is within sub-pixel tolerance */
        float tdx = a->pos[N_JOINTS - 1].x - target.x;
        float tdy = a->pos[N_JOINTS - 1].y - target.y;
        if (sqrtf(tdx * tdx + tdy * tdy) < CONV_TOL) break;
    }
}

/* ── §5c  target motion — Lissajous figure-8 ───────────────────────── */

/*
 * update_target() — advance scene_time and compute the new Lissajous target.
 *
 * STEP 1 — advance scene_time:
 *   scene_time += dt × speed_scale
 *   scene_time is a pure accumulator used only as the argument to cos/sin.
 *   Multiplying by speed_scale stretches or compresses the figure-8 in
 *   time without affecting link geometry or the FABRIK solver.
 *
 * STEP 2 — compute target position:
 *   target.x = root_px + lis_ax × cos(LIS_FX × scene_time)
 *   target.y = root_py + lis_ay × sin(LIS_FY × scene_time + LIS_PHASE)
 *
 *   LIS_FX=1, LIS_FY=2, LIS_PHASE=π/4: produces the figure-8 (∞ shape).
 *   Full explanation of why the 1:2 ratio gives a figure-8 is in the
 *   HOW IT WORKS section at the top of this file.
 *
 * STEP 3 — push into trail ring buffer:
 *   trail_head advances modulo TRAIL_SIZE; current target is stored there.
 *   trail_count increments until it reaches TRAIL_SIZE (then stays there).
 *   The renderer walks trail[] from oldest to newest to display the faint
 *   '.' path that reveals the figure-8 shape to the viewer.
 *
 * NOTE: update_target() must be called BEFORE fabrik_solve() each tick
 * so the FABRIK solver uses the freshly computed target position.
 */
static void update_target(Arm *a, float dt)
{
    /* Step 1: advance the Lissajous clock */
    a->scene_time += dt * a->speed_scale;

    /* Step 2: Lissajous parametric equations */
    a->target.x = a->root_px + a->lis_ax * cosf(LIS_FX * a->scene_time);
    a->target.y = a->root_py + a->lis_ay * sinf(LIS_FY * a->scene_time + LIS_PHASE);

    /* Step 3: push into trail ring buffer (overwrites oldest when full) */
    a->trail_head = (a->trail_head + 1) % TRAIL_SIZE;
    a->trail[a->trail_head] = a->target;
    if (a->trail_count < TRAIL_SIZE) a->trail_count++;
}

/* ── §5d  rendering helpers ─────────────────────────────────────────── */

/*
 * draw_link_beads() — stamp 'o' along a link segment at DRAW_STEP_PX intervals.
 *
 * This is pass 1 of the two-pass arm renderer.  It fills the segment from
 * pixel position a to b with equally-spaced bead characters, then §5e's
 * render_arm() overlays joint markers in pass 2 (overwriting the beads at
 * joint positions with '0'/'o'/'.' for a cleaner articulated look).
 *
 * ALGORITHM:
 *   Compute the direction vector (dx, dy) = b − a and its length len.
 *   Divide len by DRAW_STEP_PX and ceil to get nsteps.
 *   Walk t from 0 to nsteps (inclusive):
 *     u = t / nsteps   (parameter in [0, 1])
 *     px = a.x + dx × u,  py = a.y + dy × u   (lerp along segment)
 *     Convert to cell (cx, cy); skip if same as previous (dedup).
 *     Skip if outside [0,cols) × [0,rows) (out-of-bounds clip).
 *     Otherwise: mvwaddch(cy, cx, 'o') with the given color pair/attr.
 *
 * DEDUPLICATION: prev_cx / prev_cy start at −9999 (off-screen sentinel).
 *   Comparing the new cell to the previous one prevents stamping the
 *   same terminal cell twice for adjacent t-values that round to the same
 *   cell.  Without this, the 'o' would be written once but attributes
 *   set/unset twice, causing a subtle colour flicker on some terminals.
 *
 * OUT-OF-BOUNDS CLIPPING: necessary during the alpha-lerp window when
 *   an arm joint is transitioning between tick positions (particularly
 *   for near-boundary positions).  Silent skip keeps rendering safe.
 */
static void draw_link_beads(WINDOW *w,
                             Vec2 a, Vec2 b,
                             int pair, attr_t attr,
                             int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;   /* degenerate segment (joints coincident) */

    int nsteps  = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;   /* dedup */
        prev_cx = cx;
        prev_cy = cy;

        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;   /* clip */

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, (chtype)'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/*
 * draw_reach_circle() — draw a sparse dotted approximation of the reach envelope.
 *
 * The arm's reach sphere in 2-D is a circle of radius = total_len centred
 * on the root joint.  When at_limit is true (target beyond reach), this
 * circle is drawn as a reminder of the arm's physical constraint.
 *
 * WHY SPARSE (48 SAMPLES)?
 *   Drawing every pixel on the circle would stamp thousands of 'o' chars
 *   in adjacent cells — most mapping to the same terminal cell — which is
 *   both wasteful and noisy (overdraw artefacts).  Instead, 48 evenly-
 *   spaced angular samples give a dashed circle:
 *     angle_k = k × 2π/48 = k × 7.5°,   k = 0…47
 *     dot at (root_px + R×cos(angle_k),  root_py + R×sin(angle_k))
 *
 *   Arc between consecutive dots = R × 2π/48 ≈ R/7.6 px.
 *   For a typical arm reach R ≈ 300 px: ~39 px ≈ 5 cell-widths per gap.
 *   This gives a clearly recognisable dashed circle without dot overlap.
 *
 * The circle is drawn with pair 7 (yellow) A_DIM — dim so it doesn't
 * compete with the arm's own rendering, but bright enough to be noticed
 * as a warning boundary when the target escapes the reach envelope.
 *
 * NOTE: Only called from render_arm() when a->at_limit is true.
 */
static void draw_reach_circle(WINDOW *w, Vec2 root, float radius,
                               int cols, int rows)
{
    /* 48 angular samples = 7.5° per step — see comment above for sizing */
    static const int N_CIRCLE_PTS = 48;
    float pi2 = 2.0f * (float)M_PI;

    for (int k = 0; k < N_CIRCLE_PTS; k++) {
        float angle = (float)k * pi2 / (float)N_CIRCLE_PTS;   /* 0..2π */
        float px    = root.x + radius * cosf(angle);
        float py    = root.y + radius * sinf(angle);
        int   cx    = px_to_cell_x(px);
        int   cy    = px_to_cell_y(py);

        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(7) | A_DIM);
        mvwaddch(w, cy, cx, (chtype)'.');
        wattroff(w, COLOR_PAIR(7) | A_DIM);
    }
}

/* ── §5e  render_arm ────────────────────────────────────────────────── */

/*
 * render_arm() — compose the complete arm frame into WINDOW *w.
 *
 * Called once per render frame by scene_draw().  This function only READS
 * arm state (const Arm *a) — it never mutates simulation data.  The
 * separation of "tick mutates state" and "draw reads state" is enforced
 * by the const qualifier here.
 *
 * ─── STEP 1: SUB-TICK ALPHA INTERPOLATION (joint positions) ────────────
 *   rj[i] = prev_pos[i] + (pos[i] − prev_pos[i]) × alpha
 *   alpha ∈ [0, 1) is the fractional time elapsed since the last physics
 *   tick.  When render Hz > sim Hz, alpha smoothly interpolates between
 *   two physics snapshots so joint positions slide frame-by-frame rather
 *   than jumping discretely.
 *
 * ─── STEP 2: SUB-TICK ALPHA INTERPOLATION (target position) ────────────
 *   rt = prev_target + (target − prev_target) × alpha
 *   The target marker ('+' or 'X') also slides smoothly between ticks.
 *
 * ─── STEP 3: FAINT LISSAJOUS TRAIL ────────────────────────────────────
 *   Walk trail[] from oldest to newest.
 *   The trail ring buffer stores entries with trail_head = newest.
 *   Oldest entry index = (trail_head − trail_count + 1 + k) % TRAIL_SIZE.
 *   Each entry is drawn as a '.' dot in pair 6 (red) A_DIM — dim so the
 *   trail doesn't obscure the arm, but visible enough to show the ∞ shape.
 *
 * ─── STEP 4: REACH LIMIT CIRCLE (conditional) ─────────────────────────
 *   Only drawn when a->at_limit is true (target out of reach).
 *   draw_reach_circle() renders the dashed yellow boundary circle.
 *
 * ─── STEP 5: ARM LINK BEAD FILL (pass 1) ───────────────────────────────
 *   For each link i (i = 0..N_LINKS-1): call draw_link_beads() to fill
 *   the segment from rj[i] to rj[i+1] with 'o' beads.
 *   Color pair assignment:
 *     link 0 (root):  pair 1 (darkest), A_BOLD — prominent base
 *     link 1:         pair 2,            A_NORMAL
 *     link 2:         pair 3,            A_NORMAL
 *     link 3 (tip):   pair 5 (brightest),A_BOLD — glowing tip
 *   (link_pairs[] maps link index to pair number; pair 4 is skipped
 *    intentionally to leave visual headroom for the tip's extra brightness.)
 *
 * ─── STEP 6: JOINT NODE MARKERS (pass 2) ───────────────────────────────
 *   Overwrite bead fill at each joint position with a size-coded marker:
 *     pos[0]       — '0' large node,  pair 1, A_BOLD  (root anchor)
 *     pos[1..N/2]  — '0' large node,  pair i+1, A_BOLD (near-root joints)
 *     pos[N/2+1..]— 'o' medium node, pair i+1, A_BOLD (distal joints)
 *     pos[N-1]     — '.' small node,  pair 5, A_BOLD  (end effector tip)
 *   Decreasing marker size root→tip reinforces the visual hierarchy and
 *   makes the tip look like a precision gripper (small, fast, bright).
 *
 * ─── STEP 7: TARGET MARKER ─────────────────────────────────────────────
 *   At the interpolated target position rt:
 *     If a->at_limit: draw 'X' — target out of reach (arm can't get there)
 *     Else:           draw '+' — target being tracked
 *   Both in pair 6 (bright red), A_BOLD — stands out against the arm.
 */
static void render_arm(const Arm *a, WINDOW *w,
                        int cols, int rows, float alpha)
{
    /* Step 1 — interpolated joint positions */
    Vec2 rj[N_JOINTS];
    for (int i = 0; i < N_JOINTS; i++) {
        rj[i].x = a->prev_pos[i].x + (a->pos[i].x - a->prev_pos[i].x) * alpha;
        rj[i].y = a->prev_pos[i].y + (a->pos[i].y - a->prev_pos[i].y) * alpha;
    }

    /* Step 2 — interpolated target position */
    Vec2 rt;
    rt.x = a->prev_target.x + (a->target.x - a->prev_target.x) * alpha;
    rt.y = a->prev_target.y + (a->target.y - a->prev_target.y) * alpha;

    /* Step 3 — faint Lissajous trail: oldest→newest entry */
    for (int k = 0; k < a->trail_count; k++) {
        /* Compute index of the k-th oldest entry in the ring buffer */
        int idx = (a->trail_head + TRAIL_SIZE - a->trail_count + 1 + k) % TRAIL_SIZE;
        int cx  = px_to_cell_x(a->trail[idx].x);
        int cy  = px_to_cell_y(a->trail[idx].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(6) | A_DIM);
        mvwaddch(w, cy, cx, (chtype)'.');
        wattroff(w, COLOR_PAIR(6) | A_DIM);
    }

    /* Step 4 — reach limit circle (only when arm is fully stretched) */
    if (a->at_limit) {
        Vec2 root = { a->root_px, a->root_py };
        draw_reach_circle(w, root, a->total_len, cols, rows);
    }

    /* Step 5 — arm link bead fill (pass 1):
     * link_pairs[i] maps link index to ncurses color pair.
     * Pair 4 is deliberately skipped to create extra contrast between
     * the middle links (pairs 2, 3) and the bright tip link (pair 5). */
    static const int link_pairs[N_LINKS] = { 1, 2, 3, 5 };
    for (int i = 0; i < N_LINKS; i++) {
        attr_t attr = (i == 0 || i == N_LINKS - 1) ? A_BOLD : A_NORMAL;
        draw_link_beads(w, rj[i], rj[i + 1], link_pairs[i], attr, cols, rows);
    }

    /* Step 6 — joint node markers (pass 2, overwriting bead fill at joints):
     * Marker choice encodes position in hierarchy: '0' > 'o' > '.' */
    for (int i = 0; i < N_JOINTS; i++) {
        int jx = px_to_cell_x(rj[i].x);
        int jy = px_to_cell_y(rj[i].y);
        if (jx < 0 || jx >= cols || jy < 0 || jy >= rows) continue;

        chtype marker;
        int    pair;
        if (i == 0) {
            /* Root anchor: largest marker, darkest pair */
            marker = (chtype)'0';  pair = 1;
        } else if (i == N_JOINTS - 1) {
            /* End effector: smallest marker, brightest pair */
            marker = (chtype)'.';  pair = 5;
        } else if (i <= N_JOINTS / 2) {
            /* Near-root interior joints: large marker */
            marker = (chtype)'0';  pair = i + 1;
        } else {
            /* Near-tip interior joints: medium marker */
            marker = (chtype)'o';  pair = i + 1;
        }
        wattron(w, COLOR_PAIR(pair) | A_BOLD);
        mvwaddch(w, jy, jx, marker);
        wattroff(w, COLOR_PAIR(pair) | A_BOLD);
    }

    /* Step 7 — target marker: '+' tracking, 'X' out-of-reach */
    int tx = px_to_cell_x(rt.x);
    int ty = px_to_cell_y(rt.y);
    if (tx >= 0 && tx < cols && ty >= 0 && ty < rows) {
        wattron(w, COLOR_PAIR(6) | A_BOLD);
        mvwaddch(w, ty, tx, a->at_limit ? (chtype)'X' : (chtype)'+');
        wattroff(w, COLOR_PAIR(6) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene — top-level simulation container.
 * Currently holds one Arm.  The scene abstraction matches the framework
 * pattern (framework.c §6) so this file can serve as a template for
 * multi-entity scenes (e.g., two arms, or an arm with an obstacle field).
 */
typedef struct { Arm arm; } Scene;

/*
 * scene_init() — initialise the arm to a clean, immediately-animated state.
 *
 * Called once at startup and again on terminal resize (SIGWINCH).  On
 * resize, theme_idx, speed_scale, and scene_time are preserved so the
 * animation continues smoothly from where it was.
 *
 * ─── ROOT POSITION ─────────────────────────────────────────────────────
 *   root_px = sw × 0.50  (screen centre horizontally)
 *   root_py = sh × 0.50  (screen centre vertically)
 *   where sw = cols × CELL_W, sh = rows × CELL_H (pixel dimensions).
 *   Centring the root ensures the figure-8 target path is visible in all
 *   four quadrants from the starting position.
 *
 * ─── LINK LENGTHS (TAPERING) ───────────────────────────────────────────
 *   arm_reach = min(sw, sh) × 0.60
 *     Using 60% of the shorter screen dimension ensures the arm fits on
 *     any terminal without clipping.  On a typical 220×50 terminal:
 *       sw = 220 × 8 = 1760 px,  sh = 50 × 16 = 800 px
 *       arm_reach = 800 × 0.60 = 480 px  (total chain length)
 *
 *   link_len[i] = arm_reach × weight[i]:
 *     weight = {0.32, 0.27, 0.23, 0.18}   (sum = 1.0)
 *     link_len[0] (root) = 480 × 0.32 = 153.6 px   ← longest
 *     link_len[1]        = 480 × 0.27 = 129.6 px
 *     link_len[2]        = 480 × 0.23 = 110.4 px
 *     link_len[3] (tip)  = 480 × 0.18 =  86.4 px   ← shortest (56% of root)
 *
 *   The descending weights mirror the biomechanical structure of animal
 *   limbs (upper arm > forearm > hand) and give the arm visual hierarchy.
 *
 * ─── LISSAJOUS AMPLITUDES ──────────────────────────────────────────────
 *   lis_ax = sw × 0.40  (40% of screen pixel width)
 *   lis_ay = sh × 0.40  (40% of screen pixel height)
 *   On the example terminal: lis_ax = 704 px,  lis_ay = 320 px.
 *
 *   Clip both to ≤ total_len × 0.90:
 *     max_amp = 480 × 0.90 = 432 px
 *     lis_ax is clipped from 704 to 432 px  (too large → clip)
 *     lis_ay = 320 px < 432  → unchanged
 *   After clipping, the x-lobe of the figure-8 extends to 432 px from
 *   centre — just inside the arm's 480 px reach.  The arm can ALMOST
 *   reach the x-extremes, causing at_limit to briefly trigger on each
 *   horizontal loop, which makes the reach circle appear and disappear
 *   rhythmically — a visually interesting IK boundary demonstration.
 *
 * ─── INITIAL ARM CONFIGURATION ─────────────────────────────────────────
 *   Joints placed in a straight horizontal line to the right of root:
 *     pos[0]   = root
 *     pos[i+1] = pos[i] + (link_len[i], 0)   for i = 0..N_LINKS-1
 *   This is a valid non-degenerate configuration; FABRIK will smoothly
 *   reconfigure it on the first tick to track the initial target position.
 *
 * ─── INITIAL TARGET ────────────────────────────────────────────────────
 *   Computed at scene_time = 0:
 *     target.x = root_px + lis_ax × cos(0)      = root_px + lis_ax
 *     target.y = root_py + lis_ay × sin(LIS_PHASE) ≈ root_py + lis_ay × 0.707
 *   This places the target to the right of and slightly below root —
 *   compatible with the initial straight-right arm configuration.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Arm *a = &sc->arm;

    /* Pixel dimensions of the terminal in square pixel space */
    float sw = (float)(cols * CELL_W);   /* total screen width  in pixels */
    float sh = (float)(rows * CELL_H);   /* total screen height in pixels */

    /* Link lengths: arm spans 60% of shorter screen dimension.
     * Weights {0.32, 0.27, 0.23, 0.18} sum to 1.0 → total_len = arm_reach. */
    float arm_reach  = (sw < sh ? sw : sh) * 0.60f;
    a->link_len[0]   = arm_reach * 0.32f;   /* root link  — longest  */
    a->link_len[1]   = arm_reach * 0.27f;   /* second link           */
    a->link_len[2]   = arm_reach * 0.23f;   /* third link            */
    a->link_len[3]   = arm_reach * 0.18f;   /* tip link   — shortest */
    a->total_len     = a->link_len[0] + a->link_len[1]
                     + a->link_len[2] + a->link_len[3];

    /* Root anchor: screen centre in pixel space */
    a->root_px = sw * 0.50f;
    a->root_py = sh * 0.50f;

    /* Lissajous amplitudes: 40% of each screen axis in pixels.
     * Clipped to 90% of total arm reach so at least part of the figure-8
     * is always reachable — the at_limit state occurs naturally at the extremes. */
    a->lis_ax = sw * 0.40f;
    a->lis_ay = sh * 0.40f;
    float max_amp = a->total_len * 0.90f;
    if (a->lis_ax > max_amp) a->lis_ax = max_amp;
    if (a->lis_ay > max_amp) a->lis_ay = max_amp;

    /* Lissajous timing: start at t=0, default speed */
    a->scene_time  = 0.0f;
    a->speed_scale = LIS_SPEED_DEFAULT;

    /* Initial arm: straight line to the right from root */
    a->pos[0].x = a->root_px;
    a->pos[0].y = a->root_py;
    for (int i = 1; i < N_JOINTS; i++) {
        a->pos[i].x = a->pos[i - 1].x + a->link_len[i - 1];
        a->pos[i].y = a->pos[i - 1].y;   /* all on the same horizontal line */
    }

    /* Initial target: Lissajous at scene_time=0 */
    a->target.x = a->root_px + a->lis_ax * cosf(0.0f);
    a->target.y = a->root_py + a->lis_ay * sinf(LIS_PHASE);

    /* Copy to prev arrays so the first frame's alpha lerp is a no-op */
    memcpy(a->prev_pos, a->pos, sizeof a->pos);
    a->prev_target = a->target;

    /* Trail starts empty; fills over the first TRAIL_SIZE ticks */
    a->trail_head  = 0;
    a->trail_count = 0;

    a->at_limit  = false;
    a->paused    = false;
    a->theme_idx = 0;
}

/*
 * scene_tick() — one fixed-step physics update.
 *
 * Called by the main loop accumulator (§8) exactly sim_fps times per
 * wall-clock second.  dt = 1.0 / sim_fps seconds (the fixed tick duration).
 * This function is the sole writer of Arm state each tick.
 *
 * ORDER IS CRITICAL:
 *   1. Save prev_pos[] and prev_target FIRST — before any physics runs.
 *      These are the interpolation anchors for render_arm(); they must
 *      hold the state from the END of the PREVIOUS tick.  Saving after
 *      update_target() would produce a lerp that begins from a wrong state.
 *
 *   2. Return early if paused — prev snapshots are saved regardless, so
 *      the alpha lerp produces a clean freeze (prev == curr → lerp == identity).
 *
 *   3. update_target() — advance scene_time, compute new Lissajous target,
 *      push into trail buffer.  Must run before fabrik_solve() so the
 *      solver uses the freshly updated target.
 *
 *   4. fabrik_solve() — run FABRIK IK: moves pos[] to track the new target.
 *      Reads target (set in step 3) and root_px/root_py (constant).
 *      Writes pos[] and at_limit.
 *
 * cols/rows unused this tick (arm position is pixel-space; screen bounds
 * only matter for rendering, not physics).  The parameters are kept to
 * match the scene_tick() signature used by the framework template.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    (void)cols; (void)rows;   /* arm physics is screen-size-independent */
    Arm *a = &sc->arm;

    /* Step 1: snapshot previous state for sub-tick alpha lerp */
    memcpy(a->prev_pos, a->pos, sizeof a->pos);
    a->prev_target = a->target;

    /* Step 2: skip physics if paused (lerp freezes cleanly) */
    if (a->paused) return;

    /* Step 3: advance Lissajous target */
    update_target(a, dt);

    /* Step 4: FABRIK IK solve — bring tip toward new target */
    fabrik_solve(a);
}

/*
 * scene_draw() — render the scene into WINDOW *w; called once per render frame.
 *
 * alpha ∈ [0, 1) is the sub-tick interpolation factor computed by the
 * main loop: alpha = sim_accum / tick_ns.  Passed to render_arm() for
 * sub-tick smooth motion.  dt_sec is unused (the arm renderer needs only
 * prev/current positions and alpha, not dt directly).
 *
 * This function only reads scene state (const Scene *sc) — it never
 * mutates simulation data.  The const enforces the "tick mutates; draw
 * reads" contract that keeps the sim and render paths cleanly separated.
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_arm(&sc->arm, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows), queried after each
 * resize.  All rendering passes cols/rows to scene_draw() so renderers can
 * clip out-of-bounds writes without segfaulting ncurses.
 *
 * See framework.c §7 for the full double-buffer architecture:
 *   erase() → draw into newscr → wnoutrefresh() → doupdate()
 * doupdate() sends only the cells that changed since the last frame —
 * typically fewer than 100 cells for an arm that moves a few pixels per tick.
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure the terminal for animation.
 *
 *   initscr()         initialise ncurses; must be first.
 *   noecho()          do not echo typed characters to the display.
 *   cbreak()          pass keys immediately without line buffering.
 *   curs_set(0)       hide the blinking hardware cursor (cleaner animation).
 *   nodelay(TRUE)     getch() returns ERR immediately when no key is pending;
 *                     without this the render loop stalls up to 1 s waiting
 *                     for input, killing the animation frame rate.
 *   keypad(TRUE)      decode multi-byte escape sequences (arrow keys, F-keys)
 *                     into single KEY_* constants.
 *   typeahead(-1)     disable ncurses' internal read-ahead during output;
 *                     without this, ncurses may call read() mid-write to
 *                     check for pending input, causing frame tearing on
 *                     slow terminals.
 */
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

/*
 * screen_free() — restore the terminal to its pre-animation state.
 * endwin() re-enables echo, restores the cursor, and resets the scroll region.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a SIGWINCH terminal resize event.
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from the
 * kernel and resize its internal virtual screens (curscr/newscr) to match
 * the new terminal dimensions.  Without this two-step, stdscr retains the
 * old size and mvwaddch() at coordinates in the newly valid area silently
 * fails — the arm may partially disappear after resize.
 *
 * After screen_resize(), app_do_resize() calls scene_init() to recompute
 * arm geometry for the new screen size, then resets frame_time and
 * sim_accum to prevent a physics time avalanche from the resize latency.
 */
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw() — compose the full frame into stdscr (ncurses' newscr).
 *
 * Frame composition order (must not change):
 *   1. erase()        — write spaces over entire newscr, clearing stale
 *                       content from the previous frame.  No terminal I/O.
 *   2. scene_draw()   — write arm, trail, markers into newscr.
 *   3. HUD top bar    — status string right-aligned on row 0, drawn LAST
 *                       so it is always on top of any arm glyph there.
 *   4. Hint bar       — key reference on the bottom row.
 *
 * Nothing reaches the terminal until screen_present() calls doupdate().
 *
 * HUD content: "IK-FABRIK  reach:NEAR/LIMIT  spd:X.XX  [Theme]  fps  Hz"
 *   reach:NEAR  — target within arm reach; FABRIK tracking actively.
 *   reach:LIMIT — target beyond reach; arm stretched, circle visible.
 *   spd         — current Lissajous speed multiplier (speed_scale).
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Arm *a = &sc->arm;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-FABRIK  reach:%s  spd:%.2f  [%s]  %.1ffps  %dHz ",
             a->at_limit ? "LIMIT" : "NEAR ",   /* reach state            */
             a->speed_scale,                     /* Lissajous time scale   */
             THEMES[a->theme_idx].name,          /* current palette name   */
             fps, sim_fps);

    /* Right-align HUD on row 0 (hx = leftmost column of the string) */
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    /* Key reference on the bottom row — left-aligned */
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  +/-:speed  t:theme  [/]:Hz ");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal atomically.
 *
 * wnoutrefresh(stdscr) copies the in-memory newscr model into ncurses'
 *   internal "pending update" structure.  No terminal I/O yet.
 * doupdate() diffs newscr against curscr (what is physically on screen),
 *   generates minimal terminal escape sequences for just the changed cells,
 *   writes them to stdout in one shot, then sets curscr = newscr.
 *
 * This two-step is the canonical ncurses flush.  For a single window,
 * wrefresh(stdscr) is equivalent but less explicit.  Using the two-step
 * makes the intent clear and matches the multi-window pattern in framework.c
 * where multiple WINDOW*s are batched before a single doupdate().
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state, accessible from signal handlers.
 *
 * Declared as a file-scope global (g_app) because POSIX signal handlers
 * receive no user-defined argument — they can only communicate with the
 * main loop through global variables.
 *
 * volatile sig_atomic_t for running and need_resize:
 *   volatile     — prevents the compiler from caching the value in a
 *                  register across signal-handler writes; ensures the main
 *                  loop always reads the latest value from memory.
 *   sig_atomic_t — the only C type POSIX guarantees can be read and
 *                  written atomically from a signal handler context.
 *                  Using int or bool here would be technically undefined
 *                  behaviour (signal/main data race without atomics).
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;   /* file-scope so signal handlers can reach it */

/*
 * Signal handlers — set flags only.
 * POSIX rule: signal handlers may not call any function that is not
 * async-signal-safe.  ncurses functions and malloc are NOT async-signal-safe.
 * Setting a sig_atomic_t flag is the correct pattern; the main loop checks
 * the flag at the top of each iteration and handles the action there.
 */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 * Registered with atexit() so endwin() is always called even if the
 * program exits via an unhandled signal path or early return that bypasses
 * screen_free().  Calling endwin() twice is safe (idempotent).
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * Sequence:
 *   1. screen_resize() re-reads LINES/COLS from the kernel via
 *      endwin() + refresh().
 *   2. Save theme_idx, speed_scale, and scene_time from the current arm
 *      state so they survive the upcoming scene_init().
 *   3. scene_init() resets ALL arm geometry (root position, link lengths,
 *      Lissajous amplitudes) to values appropriate for the new terminal size.
 *   4. Restore saved values so animation continues from the same phase and
 *      theme, just rescaled to the new screen.
 *   5. need_resize = 0 clears the flag.
 *
 * After returning, the main loop resets frame_time = clock_ns() and
 * sim_accum = 0 to prevent a physics avalanche from the large dt that
 * accumulated while the terminal was being resized (resize can take 10–100 ms
 * on some terminals, which would inject 1–6 phantom physics ticks if not reset).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);

    /* Preserve user-adjustable state across scene reinitialisation */
    int   saved_theme = app->scene.arm.theme_idx;
    float saved_speed = app->scene.arm.speed_scale;
    float saved_time  = app->scene.arm.scene_time;

    /* Re-initialise: arm root and Lissajous amplitudes depend on screen size */
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    /* Restore preserved state */
    app->scene.arm.theme_idx   = saved_theme;
    app->scene.arm.speed_scale = saved_speed;
    app->scene.arm.scene_time  = saved_time;

    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * All adjustments are to simulation parameters — there is no manual arm
 * steering.  The arm's joint configuration is driven entirely by FABRIK
 * tracking the autonomous Lissajous target.
 *
 * KEY MAP:
 *   q / Q / ESC   quit (return false → main loop exits)
 *   space         toggle arm->paused
 *   + / =         speed_scale × 1.25, clamped to [LIS_SPEED_MIN, MAX]
 *                 (= is included because + requires Shift on many keyboards)
 *   -             speed_scale ÷ 1.25, clamped to [LIS_SPEED_MIN, MAX]
 *   t / T         cycle theme forward (theme_idx + 1) % N_THEMES
 *                 and re-apply the new palette via theme_apply()
 *   ]             sim_fps + SIM_FPS_STEP, clamped to [SIM_FPS_MIN, MAX]
 *   [             sim_fps − SIM_FPS_STEP, clamped to [SIM_FPS_MIN, MAX]
 *
 * Speed scale notes:
 *   × 1.25 per press = 6 presses to double.
 *   From LIS_SPEED_MIN (0.05) to LIS_SPEED_MAX (5.0) = 100× range = ~27 presses.
 *   The 1.25 factor gives fine-grained control without needing too many presses.
 */
static bool app_handle_key(App *app, int ch)
{
    Arm *a = &app->scene.arm;

    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': a->paused = !a->paused; break;

    /* Lissajous target speed: faster / slower */
    case '+': case '=':
        a->speed_scale *= 1.25f;
        if (a->speed_scale > LIS_SPEED_MAX) a->speed_scale = LIS_SPEED_MAX;
        break;
    case '-':
        a->speed_scale /= 1.25f;
        if (a->speed_scale < LIS_SPEED_MIN) a->speed_scale = LIS_SPEED_MIN;
        break;

    /* Color theme: cycle forward (t and T both advance; no reverse in this sim) */
    case 't': case 'T':
        a->theme_idx = (a->theme_idx + 1) % N_THEMES;
        theme_apply(a->theme_idx);
        break;

    /* Simulation Hz: raise / lower physics tick rate */
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
 * main() — the game loop  (structure identical to framework.c §8)
 *
 * The loop body executes eight steps every frame:
 *
 *  ① RESIZE CHECK
 *     If need_resize is set (from SIGWINCH handler), call app_do_resize().
 *     Then reset frame_time and sim_accum to prevent the dt spike that
 *     accumulated during the resize from injecting phantom physics ticks.
 *
 *  ② MEASURE dt
 *     dt = clock_ns() − frame_time   (nanoseconds since last frame start).
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger,
 *     swap) and resumed, an uncapped dt would fire many physics ticks in
 *     one frame — a "spiral of death" where the physics backlog keeps
 *     growing.  100 ms cap limits the maximum tick debt to 6 ticks at 60 Hz.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates wall-clock dt each frame.
 *     tick_ns = 1 000 000 000 / sim_fps nanoseconds per physics tick.
 *     While sim_accum ≥ tick_ns:
 *       call scene_tick(dt_sec);    (one physics step)
 *       sim_accum -= tick_ns;
 *     This guarantees physics runs at exactly sim_fps Hz on average,
 *     regardless of how fast or slow the render loop is.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds fractional leftover time:
 *       alpha = sim_accum / tick_ns  ∈ [0.0, 1.0)
 *     Passed to render_arm() so joint positions are lerped between the
 *     last two physics snapshots, eliminating micro-stutter at any
 *     combination of render Hz and sim Hz.
 *
 *  ⑤ FPS COUNTER
 *     frame_count and fps_accum accumulate over a 500 ms window.
 *     Every 500 ms: fps_display = frame_count / (fps_accum / 1e9).
 *     500 ms averaging gives a stable reading; per-frame division would
 *     oscillate 30–120 fps on every frame even at steady rate.
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     elapsed = time spent on physics since this frame started.
 *     budget  = NS_PER_SEC / 60  (16.67 ms for a 60-fps frame).
 *     sleep   = budget − elapsed.
 *     Sleeping BEFORE the terminal write means I/O cost is not charged
 *     against the next frame's physics budget.  If elapsed > budget
 *     (over-budget frame), clock_sleep_ns(negative) returns immediately.
 *
 *  ⑦ DRAW + PRESENT
 *     screen_draw() → erase, scene_draw, HUD, hint bar — all into newscr.
 *     screen_present() → wnoutrefresh + doupdate — one atomic terminal write.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued key event.
 *     Looping (not single-call) ensures burst key-repeat events are
 *     consumed within the same frame, keeping parameter adjustments
 *     responsive when keys are held.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed RNG from monotonic clock so each run starts at a different time */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Safety net: endwin() even if we exit via an unhandled path */
    atexit(cleanup);

    /* SIGINT / SIGTERM: Ctrl-C or `kill` → graceful exit */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH: terminal resize → set need_resize flag for next iteration */
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();   /* timestamp at start of last frame  */
    int64_t sim_accum   = 0;            /* nanoseconds in the physics bucket  */
    int64_t fps_accum   = 0;            /* ns elapsed in current fps window   */
    int     frame_count = 0;            /* frames rendered in fps window      */
    double  fps_display = 0.0;          /* smoothed fps shown in HUD          */

    while (app->running) {

        /* ── ① resize ─────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset: avoid dt spike from resize  */
            sim_accum  = 0;
        }

        /* ── ② dt ─────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per physics tick   */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha — sub-tick interpolation factor ─────────────── */
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

        /* ── ⑥ frame cap — sleep before render ──────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ──────────────────────────── */
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
