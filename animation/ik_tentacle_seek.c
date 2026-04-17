/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_tentacle_seek.c — 12-Segment FABRIK Tentacle with Joint Constraints
 *                       Seeking a Lissajous Target
 *
 * DEMO: A 12-segment tentacle is anchored at the centre of the terminal.
 *       It uses the FABRIK (Forward And Backward Reaching Inverse Kinematics)
 *       algorithm to smoothly reach toward a target that traces a complex
 *       Lissajous path (frequency ratio 1:1.7, giving a quasi-periodic figure
 *       with internal crossings).  Per-joint angle constraints limit how
 *       sharply any joint can bend, preventing the tentacle from folding in
 *       on itself.  A ghost trail of '.' dots shows the recent target path.
 *       Ten selectable bioluminescent colour themes cycle with 't'/'T'.
 *
 * STUDY ALONGSIDE: snake_forward_kinematics.c (trail buffer / rendering style)
 *                  ik_arm_reach.c (FABRIK without joint constraints)
 *                  framework.c (canonical fixed-step loop / timing template)
 *
 * ─────────────────────────────────────────────────────────────────────────
 *  Section map
 * ─────────────────────────────────────────────────────────────────────────
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — 10-theme bioluminescent palette + HUD pair
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Tentacle: FABRIK solver, constraints, target, render
 *       §5a  vec2 helpers       — arithmetic, length, normalise, rotate, dot/cross
 *       §5b  FABRIK solver      — reachability, backward pass, forward pass
 *       §5c  joint constraints  — per-joint angle clamp via atan2(cross, dot)
 *       §5d  Lissajous target   — quasi-periodic path + smooth lerp tracking
 *       §5e  ghost trail        — ring buffer: push, retrieve, draw
 *       §5f  rendering          — bead fill, node markers, target, anchor
 *   §6  scene         — scene_init / scene_tick / scene_draw
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, fixed-step main loop
 * ─────────────────────────────────────────────────────────────────────────
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 1: FABRIK INVERSE KINEMATICS
 * ═══════════════════════════════════════════════════════════════════════
 *
 * FORWARD KINEMATICS (FK) vs INVERSE KINEMATICS (IK)
 * ────────────────────────────────────────────────────
 * FK (see snake_forward_kinematics.c) specifies joint angles and computes
 * where the tip ends up.  Direction: angles → tip position.  Closed form,
 * always O(N).
 *
 * IK is the reverse: given where you WANT the tip, find the joint angles
 * that place it there.  Direction: desired tip position → joint angles.
 *
 * For N > 2 links the system is underdetermined.  The classical approach
 * uses the Jacobian matrix J (partial derivatives ∂tip/∂θᵢ) and solves
 *   Δθ = J⁺ · Δtip   (J⁺ = Moore-Penrose pseudo-inverse)
 * This is O(N³), numerically fragile near singularities, and hard to
 * extend with joint-angle constraints.
 *
 * THE FABRIK APPROACH
 * ────────────────────
 * FABRIK (Aristidou & Lasenby, 2011) needs no matrix math at all.  It
 * works purely on joint POSITIONS and runs two geometric passes per
 * iteration.  p[0] = anchor (fixed root), p[N−1] = end effector (tip).
 *
 * ┌─ REACHABILITY CHECK (once per tick, before iterating) ─────────────┐
 * │  total_len = Σ link_len[i]                                         │
 * │  If |target − anchor| ≥ total_len:                                 │
 * │    Target is outside the reach sphere.  Best posture: all links    │
 * │    collinear, pointing toward target.                               │
 * │    dir = normalise(target − anchor)                                 │
 * │    p[i+1] = p[i] + dir × link_len[i],  for i = 0 … N_LINKS−1     │
 * │    Set at_limit = true; return without iterating.                  │
 * └────────────────────────────────────────────────────────────────────┘
 *
 * ┌─ BACKWARD PASS — pull tip to target, slide each joint tip→root ────┐
 * │  p[N−1] = target            (snap tip to target)                   │
 * │  for i = N−2 downto 0:                                             │
 * │    dir  = normalise(p[i] − p[i+1])    (direction child→parent)    │
 * │    p[i] = p[i+1] + dir × link_len[i] (slide parent along dir)     │
 * │    if i > 0: apply_joint_constraint(i)  (clamp bend angle)         │
 * │  After this pass: link lengths restored, tip at target, BUT root   │
 * │  has drifted away from anchor — root constraint is violated.       │
 * └────────────────────────────────────────────────────────────────────┘
 *
 * ┌─ FORWARD PASS — restore root, propagate fix toward tip ────────────┐
 * │  p[0] = anchor              (snap root back to anchor)             │
 * │  for i = 0 to N−2:                                                 │
 * │    dir    = normalise(p[i+1] − p[i])  (direction parent→child)    │
 * │    p[i+1] = p[i] + dir × link_len[i] (slide child along dir)      │
 * │  After this pass: root at anchor, all link lengths exact.  Tip is  │
 * │  now slightly off target — but CLOSER than before this iteration.  │
 * └────────────────────────────────────────────────────────────────────┘
 *
 * ┌─ CONVERGENCE CHECK ─────────────────────────────────────────────────┐
 * │  err = |p[N−1] − target|                                           │
 * │  If err < CONV_TOL (2.0 px): DONE.                                 │
 * │  Otherwise repeat backward + forward (up to MAX_ITER = 20 times). │
 * └────────────────────────────────────────────────────────────────────┘
 *
 * WHY FABRIK CONVERGES
 * ─────────────────────
 * Each backward pass snaps the tip exactly to the target (err = 0
 * immediately after that step).  The forward pass can only displace the
 * tip by a bounded amount (determined by chain geometry and root distance).
 * Because the target is within reach, a valid solution exists and FABRIK
 * is provably monotonically convergent — each iteration strictly reduces
 * tip error.  A 12-link chain at a reachable, slowly-moving target typically
 * converges in 3–8 iterations.  MAX_ITER = 20 is a safety cap for near-
 * singular configurations (arm nearly collinear, target near reach boundary)
 * where convergence is slower.  Cost is bounded: 20 × 13 joints × 2 passes
 * = 520 simple arithmetic ops per tick — negligible at 60 Hz.
 *
 * FABRIK vs JACOBIAN for real-time tentacle animation:
 *   • No matrix inverse: O(N·iter) vs O(N³)
 *   • No singularities: all ops are simple vector arithmetic
 *   • Joint constraints bolt on naturally (after each backward step)
 *   • Handles long chains (N=12) with the same code as short ones (N=4)
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 2: PER-JOINT ANGLE CONSTRAINTS
 * ═══════════════════════════════════════════════════════════════════════
 *
 * A bare FABRIK solver allows any joint to bend to any angle, including
 * 180° (complete fold-back).  Real tentacles cannot do this — muscle and
 * tissue limit each joint to ±~60°.  Without constraints the chain can
 * develop unnatural kinks when the target moves rapidly.
 *
 * THE CONSTRAINT (applied to joint i after the backward pass step):
 *
 *   dir_in  = normalise(p[i]   − p[i−1])   (direction of incoming link)
 *   dir_out = normalise(p[i+1] − p[i])     (direction of outgoing link)
 *
 *   The signed angle from dir_in to dir_out:
 *     cross = dir_in.x × dir_out.y − dir_in.y × dir_out.x
 *                               (2-D cross product = z-component)
 *     dot   = dir_in.x × dir_out.x + dir_in.y × dir_out.y
 *                               (= cos θ)
 *     angle = atan2(cross, dot)
 *                               (signed angle; + = CCW, − = CW)
 *
 *   The 2-D cross product gives the sine of the angle between the vectors
 *   (times their lengths, but since both are unit vectors, = sin θ).
 *   atan2(sin θ, cos θ) = θ  exactly, with the correct sign.
 *
 *   If |angle| > MAX_JOINT_BEND (1.1 rad ≈ 63°):
 *     clamped = clamp(angle, −MAX_JOINT_BEND, +MAX_JOINT_BEND)
 *     delta   = clamped − angle           (rotation correction)
 *     new_dir = rotate(dir_out, delta)    (rotate outgoing direction)
 *     p[i+1]  = p[i] + new_dir × link_len[i]  (reposition child joint)
 *
 * WHY 1.1 RAD (≈ 63°)?
 *   63° per joint for a 12-segment chain gives a maximum total curvature
 *   of 12 × 63° = 756°, so the tentacle can spiral more than twice.
 *   Values below ~45° make the tentacle stiff and slow to reach targets.
 *   Values above ~90° allow visible kinking at rapid direction reversals.
 *   1.1 rad is the biological sweet spot: flexible enough to be expressive,
 *   constrained enough to look like a real animal appendage.
 *
 * WHERE IN THE LOOP:
 *   The constraint is applied inside the backward pass, immediately AFTER
 *   repositioning p[i] (before moving on to p[i−1]).  Applying it during
 *   the backward pass (not the forward pass) ensures the constraint is
 *   respected in the direction that most affects convergence: pulling the
 *   chain from the tip toward the root.  The forward pass then propagates
 *   the constrained positions back outward.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 3: LISSAJOUS TARGET + SMOOTH TRACKING
 * ═══════════════════════════════════════════════════════════════════════
 *
 * LISSAJOUS PATH (quasi-periodic figure)
 * ────────────────────────────────────────
 * The "raw" target traces:
 *   lx = anchor.x + LIS_AX × cos(LIS_OMEGA_X × scene_time)
 *   ly = anchor.y + LIS_AY × sin(LIS_OMEGA_Y × scene_time + LIS_PHASE_Y)
 *
 *   LIS_AX = screen_width_px  × LIS_AX_FACTOR   (0.55 × width)
 *   LIS_AY = screen_height_px × LIS_AY_FACTOR   (0.38 × height)
 *
 * FREQUENCY RATIO 1:1.7 (= 10:17)
 *   If LIS_OMEGA_X and LIS_OMEGA_Y were in an integer ratio p:q with
 *   small p and q (e.g., 1:2 for a figure-8), the path would close after
 *   lcm(Tx, Ty) seconds and repeat identically forever.
 *
 *   The ratio 10:17 is non-integer (irrational in spirit, though rational):
 *   lcm period = 2π × 10 / LIS_OMEGA_X = 20π ≈ 62.8 s of scene_time.
 *   At default speed_scale=1, that is 62.8 wall-clock seconds — the tentacle
 *   visits every part of the figure before it repeats.  The internal crossings
 *   produce rapid target direction changes that stress-test the IK solver and
 *   the joint constraints, keeping the motion visually complex.
 *
 * SMOOTH TRACKING (low-pass lerp)
 * ─────────────────────────────────
 * actual_target does NOT jump directly to the Lissajous point each tick.
 * Instead it lerps toward it:
 *   rate           = clamp(dt × TARGET_SMOOTH, 0, 1)
 *   actual_target += (lissajous_target − actual_target) × rate
 *
 *   TARGET_SMOOTH = 8.0 (1/s): at 60 Hz, dt = 1/60, rate ≈ 0.133.
 *   This is a first-order low-pass filter with time constant τ = 1/8 s.
 *   The actual_target lags the raw Lissajous by ~125 ms, smoothing sudden
 *   direction reversals at the loop crossings into gentle curves.
 *
 *   WHY THIS MATTERS FOR THE TENTACLE:
 *   Without smoothing, the target jumps by multiple pixels each tick near
 *   the crossing points.  FABRIK can track that, but the joint constraints
 *   would fire aggressively, causing stiffening artefacts.  The lerp keeps
 *   the effective target velocity bounded so the tentacle always looks fluid.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 4: LINK LENGTH TAPERING
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Link lengths are computed as:
 *   link_len[i] = BASE_LINK_LEN − i × TAPER  =  20.0 − i × 0.8
 *   (minimum clamped to 4.0 px to avoid degenerate zero-length links)
 *
 *   i=0  (root link):  20.0 px
 *   i=5  (mid link):   16.0 px
 *   i=11 (tip link):   11.2 px
 *   Total reach: Σᵢ₌₀¹¹ (20 − i×0.8) = 12×20 − 0.8×(0+1+…+11) = 240 − 52.8 = 187.2 px
 *
 * WHY TAPER?
 *   Real tentacles (squid, octopus) are thicker and stiffer at the root
 *   and thin at the tip.  Constant-length links would look mechanical and
 *   uniform — all joints would contribute equally to bending, which is not
 *   biologically correct.
 *
 *   With tapered links:
 *   • Root joints swing fewer degrees per pixel of tip travel (longer lever).
 *   • Tip joints swing more degrees per pixel (shorter lever) — the tip
 *     is nimble while the base is stable.
 *   • FABRIK converges slightly faster: the final correction step (tip link)
 *     is shorter, so the residual error it introduces after the forward pass
 *     is smaller, reaching CONV_TOL in fewer iterations.
 *
 * ═══════════════════════════════════════════════════════════════════════
 *  HOW IT WORKS — PART 5: BEAD RENDERING (TWO-PASS SYSTEM)
 * ═══════════════════════════════════════════════════════════════════════
 *
 * Each rendered frame uses two draw passes over the tentacle chain:
 *
 *   PASS 1 — BEAD FILL (draw_link_beads)
 *     For each link i (joint i → joint i+1):
 *       Walk from joint i to joint i+1 in steps of DRAW_STEP_PX (5 px).
 *       At each step, convert (px_x, px_y) → (cell_col, cell_row).
 *       Stamp 'o' in the link's gradient colour pair.
 *       A dedup cursor (prev_cx, prev_cy) prevents stamping the same cell
 *       twice when two adjacent steps land on the same terminal cell.
 *
 *   PASS 2 — NODE MARKERS (over-stamp joint positions)
 *     For each joint i: look up joint_node_char(i):
 *       Root quarter  (i ≤ (N−1)/3):    '0'  — thick, anchored look
 *       Middle third:                    'o'  — standard bead
 *       Tip quarter   (i ≥ (N−1)×2/3):  '.'  — thin, nimble tip
 *     Stamp at the joint's cell position, overwriting the fill bead.
 *     The joint markers give the chain an articulated, segmented look
 *     (beads on a string) rather than a smooth filled line.
 *
 * The ghost trail of prior target positions is drawn BEFORE both passes
 * so that the tentacle body always renders on top of the trail dots.
 *
 * Keys:
 *   q / ESC       quit
 *   space         pause / resume
 *   w / + / =     target speed faster (speed_scale × 1.25)
 *   s / -         target speed slower (speed_scale ÷ 1.25)
 *   t             next colour theme (cycles 0 … N_THEMES−1)
 *   T             previous colour theme
 *   ] / [         simulation Hz up / down (±SIM_FPS_STEP)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       ik_tentacle_seek.c -o ik_tentacle_seek -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : FABRIK iterative IK with per-joint angle constraints.
 *                  Backward pass: tip snapped to target, each parent joint
 *                  slid along the child-to-parent direction to restore link
 *                  length; constraint applied immediately after each slide.
 *                  Forward pass: root re-anchored, each child slid along
 *                  parent-to-child direction to restore link length.
 *                  Repeat until |tip − target| < CONV_TOL or MAX_ITER hit.
 *                  No Jacobian, no trigonometry in the solver, no singularities.
 *                  Reachability check (|target−anchor| vs Σ link_len) fires
 *                  once per tick and short-circuits to a straight stretch
 *                  posture when the target is out of reach.
 *
 * Joint constraints: After each backward-pass repositioning of joint i,
 *                  compute the signed angle between the incoming link
 *                  direction (p[i]−p[i−1]) and outgoing link direction
 *                  (p[i+1]−p[i]) using angle = atan2(cross, dot) where
 *                  cross = dir_in × dir_out (2-D scalar cross product) and
 *                  dot = dir_in · dir_out.  If |angle| > MAX_JOINT_BEND,
 *                  rotate dir_out by (clamped − angle) and reposition p[i+1].
 *                  Prevents kinking; enforces biological bend limit (~63°).
 *
 * Target motion  : Lissajous figure with frequency ratio 1:1.7 (= 10:17).
 *                  The ratio is rational but with a large period (≈62.8 s
 *                  of scene_time) so the path appears quasi-periodic.
 *                  Internal crossings produce rapid direction changes that
 *                  exercise both the solver and the joint constraints.
 *                  actual_target low-pass lerps toward the Lissajous point
 *                  at rate TARGET_SMOOTH = 8 s⁻¹, smoothing jumps near
 *                  crossings so the tentacle body curves naturally.
 *
 * Data-structure : Vec2 pos[N_JOINTS] — joint positions in pixel space.
 *                  Vec2 prev_pos[N_JOINTS] — tick-start snapshot for alpha lerp.
 *                  float link_len[N_LINKS] — tapered lengths (root longest).
 *                  Vec2 trail_pts[TRAIL_POINTS] ring buffer — ghost trail of
 *                  recent actual_target positions, rendered as '.' dots.
 *                  All positions in isotropic pixel space; converted to
 *                  cell coordinates only at draw time (see §4).
 *
 * Rendering      : Two-pass bead system: pass 1 fills each link with 'o'
 *                  at DRAW_STEP_PX = 5 px intervals; pass 2 over-stamps
 *                  joint node markers ('0'/'o'/'.').  Seven-pair colour
 *                  gradient: root pair 1 (deepest, A_DIM) → tip pair 7
 *                  (brightest, A_BOLD).  Ghost trail: pair 6, A_DIM, every
 *                  3rd stored point.  Alpha lerp between prev_pos and pos
 *                  gives smooth sub-tick motion at any render frame rate.
 *
 * Performance    : Fixed-step accumulator (§8) decouples physics (60 Hz)
 *                  from render (capped at 60 fps).  FABRIK worst-case:
 *                  MAX_ITER=20 × N_JOINTS=13 × 2 passes = 520 ops/tick.
 *                  Constraint check: 12 joints × 5 ops each = 60 ops/tick.
 *                  Both are trivial at 60 Hz.  ncurses doupdate() sends only
 *                  changed cells to the terminal; typically ~150 per frame.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

/*
 * M_PI is a POSIX extension, not standard C99/C11.
 * _POSIX_C_SOURCE 200809L exposes it on most systems, but if a toolchain
 * omits it we define our own constant so the build never fails.
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
     * At 60 Hz, one FABRIK solve fires per rendered frame (since the render
     * is also capped at 60 fps), giving maximally smooth IK tracking.
     * Lowering sim Hz reduces CPU cost but makes the tentacle appear to
     * "step" rather than glide.  [10, 120] is the user-adjustable range.
     */
    SIM_FPS_MIN     =  10,
    SIM_FPS_DEFAULT =  60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    =  10,   /* step size for [ and ] keys                */

    /*
     * HUD_COLS — byte budget for the status bar string.
     * snprintf(buf, sizeof buf, ...) truncates safely if this is exceeded.
     * 96 bytes covers the longest possible HUD string at all max values.
     *
     * FPS_UPDATE_MS — how often the displayed fps value is recalculated.
     * 500 ms gives a stable reading without flickering each frame.
     */
    HUD_COLS        =  96,
    FPS_UPDATE_MS   = 500,

    /*
     * HUD_PAIR — ncurses color pair reserved for both HUD bars.
     * Kept separate from the tentacle gradient pairs (1–7) so that
     * cycling themes never accidentally changes the HUD colour.
     */
    HUD_PAIR        =   8,

    /*
     * N_JOINTS — total joint positions in the tentacle chain.
     *   pos[0]            — root anchor (fixed; the FABRIK forward pass always
     *                       snaps this back to anchor, making it effectively read-only)
     *   pos[1 .. N−2]     — interior joints (freely moved by FABRIK each iteration)
     *   pos[N_JOINTS−1]   — end effector / tip (tracks actual_target)
     *
     * N_LINKS = N_JOINTS − 1 — number of rigid segments.
     * With N_JOINTS=13, N_LINKS=12: thirteen positions, twelve links.
     * The HUD labels this "N=12" (the link count, which is more intuitive).
     */
    N_JOINTS        =  13,
    N_LINKS         =  12,

    /*
     * TRAIL_POINTS — ring buffer capacity for the ghost target trail.
     *
     * At SIM_FPS_DEFAULT=60 Hz, 120 entries = 2 seconds of target history.
     * The trail is drawn every 3rd point (40 visible dots) to show the
     * recent Lissajous path without cluttering the screen.
     *
     * Memory cost: 120 × sizeof(Vec2) = 120 × 8 = 960 bytes.  Lives in
     * the Tentacle struct inside g_app → Scene, so it is BSS not stack.
     */
    TRAIL_POINTS    = 120,

    /*
     * N_PAIRS — number of gradient colour pairs for the tentacle body.
     * Pairs 1–7 map root (pair 1, deepest/dimmest) → tip (pair 7, brightest).
     * The colour gradient gives an immediate visual reading of which end
     * is anchored and which is the active, seeking tip.
     */
    N_PAIRS         =   7,

    /*
     * N_THEMES — number of selectable bioluminescent colour palettes.
     * Press 't' to cycle forward, 'T' to cycle backward.
     * The "Medusa" theme (index 0) is the startup default: deep purple → cyan,
     * evoking a bioluminescent deep-sea creature.
     */
    N_THEMES        =  10,
};

/*
 * MAX_ITER — maximum FABRIK iterations per physics tick.
 *
 * FABRIK is provably monotonically convergent for reachable targets: each
 * full iteration (backward + forward pass) strictly reduces |tip − target|.
 * Empirical testing on a 12-link chain shows convergence to CONV_TOL in
 * 3–8 iterations for smooth target motion.  Near-singular configurations
 * (target near the reach boundary, chain nearly collinear) converge more
 * slowly; 20 iterations handles all observed cases with margin.
 *
 * At the iteration limit (target not yet converged), the tentacle holds
 * the best posture achievable in 20 iterations — visually correct.  The
 * HUD shows the actual iteration count each tick so you can see when
 * the limit is approached (look for last_iter close to 20 when moving
 * the target very fast with the 'w'/'=' keys).
 *
 * Cost: 20 × 13 × 2 passes = 520 vector ops per tick.  At 60 Hz this is
 * 31 200 ops/second — negligible on any modern processor.
 */
#define MAX_ITER        20

/*
 * CONV_TOL — FABRIK convergence threshold in pixel space (pixels).
 *
 * The FABRIK loop terminates early when |tip − target| < CONV_TOL.
 * 2.0 px is sub-cell in both axes:
 *   Horizontal: 2.0 / CELL_W (8 px) = 0.25 of a column.
 *   Vertical:   2.0 / CELL_H (16 px) = 0.125 of a row.
 * At this error the tip and target occupy the same terminal cell — further
 * iterations produce no visible improvement.  Setting it smaller (e.g., 0.5)
 * would rarely fire early termination, wasting iterations on imperceptible
 * sub-pixel corrections.
 */
#define CONV_TOL        2.0f

/*
 * MAX_JOINT_BEND — per-joint bend angle limit (radians).
 *
 * Each joint is constrained to rotate at most this far from the direction
 * of the preceding link.  1.1 rad ≈ 63°.
 *
 * WHY 63°?
 *   A single 12-link tentacle with 63° per joint can achieve a maximum
 *   total curvature of 12 × 63° = 756° — enough to spiral more than twice.
 *   This is biologically plausible for a cephalopod tentacle, which can
 *   coil tightly but cannot fold back on itself.
 *
 *   Values below 45° make the tentacle stiff and unable to track targets
 *   at close range (the chain cannot curl tightly enough).
 *   Values above 90° allow visible kinking at rapid direction reversals,
 *   where two consecutive joints bend in opposite extremes and the chain
 *   looks broken rather than biological.
 *   1.1 rad is the sweet spot: maximum flexibility while preventing kinking.
 */
#define MAX_JOINT_BEND  1.1f

/*
 * Link geometry — BASE_LINK_LEN and TAPER define the tapered chain.
 *
 * link_len[i] = BASE_LINK_LEN − i × TAPER  (minimum clamped to 4.0 px)
 *
 * Computed values:
 *   i=0  (root link):  20.0 − 0 × 0.8 = 20.0 px
 *   i=5  (mid link):   20.0 − 5 × 0.8 = 16.0 px
 *   i=11 (tip link):   20.0 − 11 × 0.8 = 11.2 px
 *   Total reach: Σᵢ₌₀¹¹(20 − 0.8i) = 240 − 0.8 × 66 = 240 − 52.8 = 187.2 px
 *
 * The Lissajous amplitudes (LIS_AX_FACTOR × screen_width ≈ 0.55 × width)
 * are chosen so the target path fits well within this reach envelope.
 */
#define BASE_LINK_LEN   20.0f   /* root link length in pixels              */
#define TAPER            0.8f   /* length reduction per link toward tip     */

/*
 * DRAW_STEP_PX — pixel step size for the bead fill renderer.
 *
 * draw_link_beads() walks each link from joint[i] to joint[i+1] in steps
 * of DRAW_STEP_PX, stamping 'o' at each step.  This must satisfy:
 *   DRAW_STEP_PX < CELL_W (8 px)  — so no column is skipped horizontally
 *   DRAW_STEP_PX < CELL_H (16 px) — so no row is skipped vertically
 *
 * With 5 px: near-horizontal link (21 px root) → ceil(21/5)+1 = 6 samples,
 * spanning 21/8 = 2.6 columns — every column covered.  Near-vertical:
 * 5/16 = 0.31 rows/sample — dense enough.
 *
 * 5 px (vs snake's 3 px) gives slightly sparser beads — individual 'o'
 * characters are more visible between joints, emphasising the articulated
 * bead-chain look of the tentacle rather than the solid-line look of a snake.
 */
#define DRAW_STEP_PX    5.0f

/*
 * Lissajous target parameters.
 *
 * The "raw" Lissajous target:
 *   lx = anchor.x + LIS_AX × cos(LIS_OMEGA_X × scene_time)
 *   ly = anchor.y + LIS_AY × sin(LIS_OMEGA_Y × scene_time + LIS_PHASE_Y)
 *
 * LIS_AX_FACTOR = 0.55 — x amplitude as fraction of screen pixel width.
 * LIS_AY_FACTOR = 0.38 — y amplitude as fraction of screen pixel height.
 *   These give the Lissajous figure roughly 55% of the terminal width and
 *   76% of the height (38% × 2 for the full vertical swing), keeping the
 *   target visible on any typical terminal size.
 *
 * LIS_OMEGA_X = 1.0 rad/s — x oscillates with period 2π/1 ≈ 6.28 s.
 * LIS_OMEGA_Y = 1.7 rad/s — y oscillates with period 2π/1.7 ≈ 3.70 s.
 *   Frequency ratio 1:1.7 = 10:17.  The figure closes after LCM of the
 *   two periods = 2π × 10 ≈ 62.8 s of scene_time.  The irrational-feeling
 *   ratio creates a path with many internal crossings (unlike a simple 1:2
 *   figure-8) that exercises the IK solver with varied directional demands.
 *
 * LIS_PHASE_Y = π/3 ≈ 1.047 rad — initial phase offset for the y component.
 *   Without phase offset, both components start at their extreme positions
 *   simultaneously, which causes the Lissajous path to start at a cusp.
 *   π/3 gives a clean, non-tangent starting position.
 */
#define LIS_AX_FACTOR   0.55f
#define LIS_AY_FACTOR   0.38f
#define LIS_OMEGA_X     1.0f
#define LIS_OMEGA_Y     1.7f
#define LIS_PHASE_Y     ((float)M_PI / 3.0f)

/*
 * TARGET_SMOOTH — low-pass filter rate for actual_target tracking (1/s).
 *
 * Each tick:
 *   rate = clamp(dt × TARGET_SMOOTH, 0, 1)
 *   actual_target += (lissajous_target − actual_target) × rate
 *
 * This is an exponential moving average (first-order IIR low-pass filter).
 * At 60 Hz, dt = 1/60 ≈ 0.0167 s:
 *   rate ≈ 0.0167 × 8 ≈ 0.133
 *   Time constant τ = 1 / TARGET_SMOOTH = 0.125 s
 *
 * The actual_target lags the raw Lissajous by ~125 ms.  This is barely
 * perceptible to the eye, but it converts the Lissajous velocity's sudden
 * reversals (sharp direction changes at loop crossings) into smooth curves.
 * Without this, the tentacle would need to respond to near-instantaneous
 * target direction changes, which stress the joint constraints and can
 * cause brief stiffening artefacts.
 */
#define TARGET_SMOOTH   8.0f

/*
 * Timing primitives — verbatim from framework.c.
 *
 * NS_PER_SEC / NS_PER_MS: unit-conversion constants.
 *   NS_PER_SEC = 1 000 000 000 (10⁹)
 *   NS_PER_MS  =     1 000 000 (10⁶)
 *
 * TICK_NS(f): converts frequency f (Hz) to period in nanoseconds.
 *   e.g. TICK_NS(60) = 1 000 000 000 / 60 ≈ 16 666 667 ns ≈ 16.67 ms.
 * Used in the fixed-step accumulator (§8) to know how many nanoseconds to
 * drain from sim_accum per physics tick.
 */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Terminal cell dimensions — the aspect-ratio bridge between pixel space
 * and display (see §4 for full explanation).
 *
 * CELL_W = 8 px:  physical pixels per terminal column.
 * CELL_H = 16 px: physical pixels per terminal row.
 *
 * A typical terminal cell is exactly twice as tall as it is wide.  All
 * simulation positions (Vec2) live in a square pixel space where 1 unit
 * = 1 physical pixel in both x and y.  Only at draw time do §4 helpers
 * convert to cell coordinates, applying the 2:1 correction.
 */
#define CELL_W   8
#define CELL_H  16

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — monotonic wall-clock in nanoseconds.
 *
 * CLOCK_MONOTONIC never goes backward (unlike CLOCK_REALTIME, which can
 * jump on NTP corrections or DST changes).  Subtracting two consecutive
 * clock_ns() calls gives the true elapsed wall time under any system load.
 *
 * Returns int64_t (signed 64-bit) so dt differences are naturally signed;
 * negative values (from the pause-guard cap) do not wrap.
 * int64_t holds ±9.2 × 10¹⁸ ns = ±292 years — no overflow risk.
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
 * before (not after) terminal I/O means the I/O write cost is not charged
 * against the next frame's physics budget.
 *
 * If ns ≤ 0, the frame is already over-budget (FABRIK + other work took
 * longer than one frame period): skip the sleep and recover immediately
 * rather than sleeping zero or a negative duration.
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
/* §3  color — 10-theme bioluminescent palette + HUD pair                */
/* ===================================================================== */

/*
 * Theme — one named colour palette for the tentacle.
 *
 *   name     : display string shown in the HUD; identifies the theme.
 *   body[]   : seven xterm-256 foreground colour indices for ncurses
 *              pairs 1–7.  body[0] = root link (darkest), body[6] = tip
 *              (brightest).  The dark-to-bright gradient emphasises the
 *              root→tip hierarchy and makes the seeking tip glow.
 *   hud      : xterm-256 foreground index for HUD_PAIR (pair 8).
 *              Each theme provides its own HUD colour for visual coherence.
 *
 * theme_apply() calls init_pair() live, so switching themes takes effect
 * on the very next rendered frame without restarting ncurses.
 *
 * The 10 themes are organised as bioluminescent deep-sea aesthetics.
 * The startup default is "Medusa" (index 0): deep violet → pale cyan,
 * evoking the bioluminescent glow of a jellyfish tentacle.
 */
typedef struct {
    const char *name;
    int body[N_PAIRS]; /* 7 colour indices for pairs 1–7 (root → tip)     */
    int hud;           /* colour index for pair 8 (HUD bars)               */
} Theme;

/*
 * THEMES[10] — ten selectable palettes.
 *
 * Index  Name     Colour journey (root → tip)         HUD
 * ─────────────────────────────────────────────────────────
 *   0    Medusa   deep violet → indigo → pale cyan     yellow  (default)
 *   1    Matrix   dark green  → mid-green → neon lime  green
 *   2    Fire     deep red    → orange → bright yellow yellow
 *   3    Ocean    dark navy   → royal blue → aqua      aqua
 *   4    Nova     deep purple → violet → hot pink      magenta
 *   5    Toxic    dark olive  → grass → neon lime      lime
 *   6    Lava     dark red    → rust  → amber          red
 *   7    Ghost    dark grey   → mid-grey → white       white
 *   8    Aurora   dark teal   → seafoam → gold         seafoam
 *   9    Neon     hot pink    → violet → blue-cyan     magenta
 *
 * The body[] array maps directly to ncurses pairs 1–7:
 *   pair p = init_pair(p, body[p−1], COLOR_BLACK)
 * So body[0] is the foreground colour for the root link (pair 1) and
 * body[6] is the foreground colour for the tip (pair 7).
 */
static const Theme THEMES[N_THEMES] = {
    {"Medusa", {57,  63,  93,  99,  105, 111, 159}, 226},
    {"Matrix", {22,  28,  34,  40,  46,  82,  118}, 46 },
    {"Fire",   {196, 202, 208, 214, 220, 226, 227}, 226},
    {"Ocean",  {17,  18,  19,  20,  21,  27,  51 }, 123},
    {"Nova",   {54,  55,  56,  57,  93,  129, 165}, 201},
    {"Toxic",  {22,  58,  64,  70,  76,  82,  118}, 82 },
    {"Lava",   {52,  88,  124, 160, 196, 202, 208}, 196},
    {"Ghost",  {237, 238, 239, 240, 241, 250, 255}, 255},
    {"Aurora", {22,  28,  64,  71,  78,  121, 159}, 159},
    {"Neon",   {201, 165, 129, 93,  57,  51,  45 }, 201},
};

/*
 * theme_apply() — rebind tentacle pairs (1–7) and HUD pair (8) live.
 *
 * On 256-colour terminals: calls init_pair() for each of the 8 pairs
 * using the xterm-256 indices from THEMES[idx].  Effective immediately.
 *
 * On 8-colour terminals: falls back to basic COLOR_* approximations so
 * the simulation is still usable (just without the bioluminescent gradient).
 * The 8-colour fallback uses a purple→blue→cyan→white sequence that
 * roughly approximates the Medusa theme.
 */
static void theme_apply(int idx)
{
    const Theme *th = &THEMES[idx];
    if (COLORS >= 256) {
        for (int p = 1; p <= N_PAIRS; p++)
            init_pair(p, th->body[p-1], COLOR_BLACK);
        init_pair(HUD_PAIR, th->hud, COLOR_BLACK);
    } else {
        /* 8-colour fallback: approximate Medusa gradient */
        init_pair(1, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(2, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(3, COLOR_BLUE,    COLOR_BLACK);
        init_pair(4, COLOR_BLUE,    COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_CYAN,    COLOR_BLACK);
        init_pair(7, COLOR_WHITE,   COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);
    }
}

/*
 * color_init() — one-time ncurses colour setup.
 *
 * start_color()        — enables colour support in ncurses.
 * use_default_colors() — allows COLOR_BLACK background to use the
 *                        terminal's actual background (usually true black),
 *                        avoiding a forced grey background on some terminals.
 * theme_apply()        — installs the initial theme's colour pairs.
 */
static void color_init(int initial_theme)
{
    start_color();
    use_default_colors();
    theme_apply(initial_theme);
}

/* ===================================================================== */
/* §4  coords — pixel↔cell aspect-ratio helpers                          */
/* ===================================================================== */

/*
 * WHY TWO COORDINATE SPACES
 * ─────────────────────────
 * Terminal cells are not square.  A typical cell is ~2× taller than wide
 * in physical pixels (CELL_H=16 vs CELL_W=8).  If joint positions were
 * stored in cell coordinates, a FABRIK link of "100 cells" would be 800 px
 * wide but 1600 px tall physically.  The Lissajous figure would appear
 * squashed vertically; link lengths would be geometrically wrong.
 *
 * THE FIX — physics in pixel space, drawing in cell space:
 *   All Vec2 positions are in pixel space: 1 unit = 1 physical pixel.
 *   Moving 1 unit in x or y covers the same physical distance.
 *   Only at draw time do these helpers convert to cell coordinates.
 *
 * px_to_cell_x / px_to_cell_y — round px to nearest cell coordinate.
 *
 * Formula: cell = floor(px / CELL_DIM + 0.5)
 *   Adding 0.5 before floor() = "round half up" — deterministic and
 *   symmetric.  roundf() uses banker's rounding (round half to even)
 *   which can oscillate at cell boundaries.  Truncation ((int)(px/CELL_W))
 *   always rounds down, giving asymmetric dwell.  floor+0.5 avoids both.
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

/* ── §5a  vec2 helpers ─────────────────────────────────────────────── */

/*
 * Vec2 — lightweight 2-D vector in pixel space.
 * x increases eastward; y increases downward (terminal convention).
 * All simulation positions, directions, and velocities use this type.
 */
typedef struct { float x, y; } Vec2;

/*
 * clampf() — clamp a float value to [lo, hi].
 * Used in the joint-constraint angle clamping and the lerp rate guard.
 */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/*
 * Basic Vec2 arithmetic — inlined for zero-overhead use in the FABRIK loop.
 */
static inline Vec2 vec2_add(Vec2 a, Vec2 b)    { return (Vec2){ a.x+b.x, a.y+b.y }; }
static inline Vec2 vec2_sub(Vec2 a, Vec2 b)    { return (Vec2){ a.x-b.x, a.y-b.y }; }
static inline Vec2 vec2_scale(Vec2 a, float s) { return (Vec2){ a.x*s, a.y*s }; }

/*
 * vec2_lerp() — linear interpolation between a and b.
 *   t = 0 → returns a exactly
 *   t = 1 → returns b exactly
 *   t = 0.5 → returns midpoint
 * Used for sub-tick alpha interpolation in the renderer and for the
 * actual_target low-pass smoothing in update_target().
 */
static inline Vec2 vec2_lerp(Vec2 a, Vec2 b, float t)
{
    return (Vec2){ a.x + (b.x-a.x)*t, a.y + (b.y-a.y)*t };
}

/*
 * vec2_len() — Euclidean magnitude: sqrt(x² + y²).
 * Used to measure distances between joints and to compute scale factors
 * in the FABRIK solver.  Inlined since it is called hundreds of times/tick.
 */
static inline float vec2_len(Vec2 a)
{
    return sqrtf(a.x*a.x + a.y*a.y);
}

/*
 * vec2_dist() — Euclidean distance between two points.
 * vec2_dist(a, b) = vec2_len(b − a).
 * Used in the convergence check: if vec2_dist(tip, target) < CONV_TOL → done.
 */
static inline float vec2_dist(Vec2 a, Vec2 b)
{
    return vec2_len(vec2_sub(b, a));
}

/*
 * vec2_norm() — normalise v to unit length.
 *
 * Returns v / |v|, a vector of magnitude 1.0 pointing in the same direction.
 *
 * DEGENERATE GUARD: if |v| < 1e-6 (effectively zero-length), return (1, 0).
 * This handles the case where two joints are at the same pixel (e.g., on the
 * very first tick).  Any unit vector would be acceptable; (1, 0) keeps the
 * tentacle pointing right until the solver separates the coincident joints.
 */
static inline Vec2 vec2_norm(Vec2 a)
{
    float len = vec2_len(a);
    if (len < 1e-6f) return (Vec2){1.0f, 0.0f};
    return (Vec2){ a.x/len, a.y/len };
}

/*
 * vec2_dot() — 2-D dot product: a·b = ax×bx + ay×by.
 *
 * For two unit vectors: a·b = cos(θ) where θ is the angle between them.
 * Used together with vec2_cross() to compute signed angles via atan2.
 */
static inline float vec2_dot(Vec2 a, Vec2 b) { return a.x*b.x + a.y*b.y; }

/*
 * vec2_cross() — 2-D cross product (scalar z-component): ax×by − ay×bx.
 *
 * For two unit vectors: a×b = sin(θ) where θ is the signed angle from
 * a to b (positive = counter-clockwise rotation, negative = clockwise).
 *
 * WHY THIS IS USEFUL:
 *   In 3-D, cross(a, b) = |a||b|sin(θ) × n̂  (n̂ = outward normal).
 *   In 2-D, both a and b lie in the xy-plane so the cross product
 *   reduces to a scalar: the z-component of the 3-D result.
 *   combined with dot = cos(θ):
 *     atan2(cross, dot) = atan2(sin θ, cos θ) = θ  (exact, correct sign).
 *   This avoids acos() (which loses sign and is numerically unstable near
 *   ±1) and correctly distinguishes clockwise from counter-clockwise bends.
 */
static inline float vec2_cross(Vec2 a, Vec2 b) { return a.x*b.y - a.y*b.x; }

/*
 * vec2_rotate() — rotate vector v by angle θ (radians).
 *
 * Applies the 2-D rotation matrix:
 *   [cos θ  −sin θ] [v.x]   [v.x cos θ − v.y sin θ]
 *   [sin θ   cos θ] [v.y] = [v.x sin θ + v.y cos θ]
 *
 * Used in apply_joint_constraint() to rotate the outgoing link direction
 * by the correction angle (clamped − original), enforcing the bend limit
 * without changing the link length.
 */
static inline Vec2 vec2_rotate(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x*c - v.y*s, v.x*s + v.y*c };
}

/* ── §5b + §5c  Tentacle struct, FABRIK solver, joint constraint ─────── */

/*
 * Tentacle — complete simulation state for the IK tentacle.
 *
 * ── JOINT POSITIONS ────────────────────────────────────────────────────
 *
 *   pos[N_JOINTS]       Current joint positions in pixel space.
 *                       pos[0] = anchor (fixed root).  The forward pass
 *                       snaps this back to anchor every FABRIK iteration,
 *                       making it effectively read-only during solving.
 *                       pos[N_JOINTS−1] = end effector (tracks actual_target).
 *
 *   prev_pos[N_JOINTS]  Snapshot taken at the START of each physics tick,
 *                       before FABRIK runs.  The renderer lerps between
 *                       prev_pos and pos using alpha ∈ [0, 1) to produce
 *                       sub-tick smooth motion at any render frame rate.
 *                       Without prev_pos, joints would visibly jump at each
 *                       tick boundary when render Hz > sim Hz.
 *
 * ── LINK GEOMETRY ──────────────────────────────────────────────────────
 *
 *   link_len[N_LINKS]   Length of each link in pixel space (joint i to i+1).
 *                       Computed once at scene_init() via BASE_LINK_LEN −
 *                       i × TAPER, then held constant.  Decreasing root→tip
 *                       (tapering) gives the biomechanical look of a tentacle.
 *
 * ── TARGET ─────────────────────────────────────────────────────────────
 *
 *   anchor              Fixed root position (screen centre).  Set at init
 *                       and updated on resize.  Never modified by FABRIK.
 *
 *   actual_target       The smoothly tracked target position.  Lags the raw
 *                       Lissajous position by ~125 ms (τ = 1/TARGET_SMOOTH).
 *                       This is the point that FABRIK tries to reach each tick.
 *
 *   prev_target         Snapshot of actual_target at tick start.  Used for
 *                       alpha lerp of the target marker ('*' or '#') so it
 *                       slides smoothly between frames.
 *
 *   scene_time          Accumulated simulation time (seconds × speed_scale).
 *                       Drives the Lissajous parameter.  Advances at
 *                       dt × speed_scale each tick.
 *
 *   speed_scale         Multiplier on scene_time advancement.  1.0 = default;
 *                       > 1 → faster Lissajous path; < 1 → slower.
 *                       Adjusted by 'w'/'s' or '+'/'-' keys.
 *
 * ── STATE FLAGS ────────────────────────────────────────────────────────
 *
 *   last_iter           FABRIK iterations used on the most recent tick.
 *                       Shown in the HUD.  Values near MAX_ITER indicate
 *                       a near-singular configuration or fast target motion.
 *
 *   at_limit            True when |actual_target − anchor| ≥ total reach.
 *                       Causes fabrik_solve() to stretch straight and
 *                       the HUD to show "at-limit" instead of "converged".
 *
 *   paused              When true, scene_tick() skips target update and
 *                       FABRIK solve.  prev_pos is still saved so alpha lerp
 *                       produces a clean freeze (prev = curr → identity lerp).
 *
 * ── GHOST TRAIL ────────────────────────────────────────────────────────
 *
 *   trail_pts[]         Ring buffer of recent actual_target pixel positions.
 *                       One entry appended per physics tick by update_target().
 *                       Every 3rd point is rendered as a '.' (pair 6, A_DIM)
 *                       to show the Lissajous path the tentacle is following.
 *
 *   trail_write         Write cursor — index of the next slot to be filled.
 *                       Advances mod TRAIL_POINTS on each push.
 *
 *   trail_fill          Number of valid entries, ≤ TRAIL_POINTS.
 *                       Reaches TRAIL_POINTS after TRAIL_POINTS ticks
 *                       (~2 s at 60 Hz) and stays there permanently.
 *
 * ── THEME ──────────────────────────────────────────────────────────────
 *
 *   theme_idx           Index into THEMES[].  'T'/'t' keys cycle it.
 *                       Stored in the struct so scene_init() can preserve
 *                       the current theme across resets.
 */
typedef struct {
    Vec2  pos[N_JOINTS];
    Vec2  prev_pos[N_JOINTS];
    float link_len[N_LINKS];
    Vec2  anchor;
    Vec2  actual_target;
    Vec2  prev_target;
    float scene_time;
    float speed_scale;
    int   last_iter;
    bool  at_limit;
    bool  paused;
    Vec2  trail_pts[TRAIL_POINTS];
    int   trail_write;
    int   trail_fill;
    int   theme_idx;
} Tentacle;

/* ── §5c  joint angle constraint ──────────────────────────────────────── */

/*
 * apply_joint_constraint() — clamp the bend angle at joint i.
 *
 * Called inside the FABRIK backward pass immediately after repositioning
 * joint i.  At this point:
 *   p[i−1] holds the previous (still-valid) position
 *   p[i]   has just been repositioned by the backward pass step
 *   p[i+1] holds the current child position (not yet updated this pass)
 *
 * DERIVATION:
 *
 *   Step 1 — compute the direction of the incoming link (parent → joint i):
 *     dir_in = normalise(pos[i] − pos[i−1])
 *
 *   Step 2 — compute the direction of the outgoing link (joint i → child):
 *     dir_out = normalise(pos[i+1] − pos[i])
 *
 *   Step 3 — measure the signed angle between them:
 *     cross = dir_in.x × dir_out.y − dir_in.y × dir_out.x
 *               (z-component of 3-D cross product; = sin θ for unit vectors)
 *     dot   = dir_in.x × dir_out.x + dir_in.y × dir_out.y
 *               (= cos θ for unit vectors)
 *     angle = atan2f(cross, dot)
 *               (signed angle in [−π, π]; + = CCW, − = CW)
 *
 *   Step 4 — if constraint is violated:
 *     clamped = clamp(angle, −MAX_JOINT_BEND, +MAX_JOINT_BEND)
 *     delta   = clamped − angle          (how much to rotate dir_out)
 *     new_dir = rotate(dir_out, delta)   (apply correction)
 *     pos[i+1] = pos[i] + new_dir × link_len[i]  (reposition child)
 *
 * GUARD: joint 0 (root) has no incoming link; joint N−1 (tip) has no
 * outgoing link.  Only joints 1 … N−2 can have a valid constraint.
 */
static void apply_joint_constraint(Tentacle *t, int i)
{
    if (i < 1 || i >= N_JOINTS - 1) return;

    Vec2 dir_in  = vec2_norm(vec2_sub(t->pos[i],   t->pos[i-1]));
    Vec2 dir_out = vec2_norm(vec2_sub(t->pos[i+1], t->pos[i]));

    float cr    = vec2_cross(dir_in, dir_out);   /* sin θ (unit vectors)   */
    float dt    = vec2_dot(dir_in, dir_out);     /* cos θ (unit vectors)   */
    float angle = atan2f(cr, dt);                /* signed angle in [−π, π]*/

    if (fabsf(angle) > MAX_JOINT_BEND) {
        float clamped  = clampf(angle, -MAX_JOINT_BEND, MAX_JOINT_BEND);
        float delta    = clamped - angle;            /* rotation correction */
        Vec2  new_dir  = vec2_rotate(dir_out, delta);
        t->pos[i+1] = vec2_add(t->pos[i],
                                vec2_scale(new_dir, t->link_len[i]));
    }
}

/* ── §5b  FABRIK solver ─────────────────────────────────────────────── */

/*
 * fabrik_solve() — run the FABRIK IK algorithm; return iteration count.
 *
 * Modifies t->pos[] in place.  t->at_limit is set/cleared here.
 *
 * Parameters:
 *   t       — the Tentacle (pos[] modified in place)
 *   target  — desired pixel position for pos[N_JOINTS−1] (end effector)
 *   anchor  — fixed pixel position for pos[0] (root)
 *
 * Returns: number of iterations performed (stored in t->last_iter by caller).
 *
 * ─── STEP 0: REACHABILITY CHECK ─────────────────────────────────────────
 *   total_len = Σᵢ link_len[i]
 *   dist = |target − anchor|
 *   If dist ≥ total_len:
 *     Chain cannot reach target.  Best posture: all links collinear,
 *     pointing from anchor toward target.
 *       dir = normalise(target − anchor)
 *       pos[0] = anchor
 *       pos[i+1] = pos[i] + dir × link_len[i]   (for i = 0..N_LINKS−1)
 *     Set at_limit = true; return 1 (one stretch operation, not an iteration).
 *
 * ─── STEP 1: BACKWARD PASS (tip → root) ─────────────────────────────────
 *   pos[N_JOINTS−1] = target         (snap tip to target)
 *   for i = N_JOINTS−2 downto 0:
 *     dir    = normalise(pos[i] − pos[i+1])   (parent-of-i direction)
 *     pos[i] = pos[i+1] + dir × link_len[i]  (slide parent to restore length)
 *     if i > 0: apply_joint_constraint(t, i)  (clamp joint i's bend angle)
 *   After this pass: all link lengths restored, tip at target,
 *   root displaced from anchor (root constraint temporarily violated).
 *
 * ─── STEP 2: FORWARD PASS (root → tip) ──────────────────────────────────
 *   pos[0] = anchor                            (snap root back to anchor)
 *   for i = 0 to N_JOINTS−2:
 *     dir      = normalise(pos[i+1] − pos[i]) (child direction)
 *     pos[i+1] = pos[i] + dir × link_len[i]  (slide child to restore length)
 *   After this pass: root at anchor, all link lengths exact, tip may have
 *   moved slightly away from target — but CLOSER than before this iteration.
 *
 * ─── CONVERGENCE CHECK ───────────────────────────────────────────────────
 *   if |pos[N_JOINTS−1] − target| < CONV_TOL: break (tip is close enough).
 *   Otherwise: repeat backward + forward passes (up to MAX_ITER total).
 */
static int fabrik_solve(Tentacle *t, Vec2 target, Vec2 anchor)
{
    /* Compute total reach once per tick */
    float total_len = 0.0f;
    for (int i = 0; i < N_LINKS; i++) total_len += t->link_len[i];

    float dist_to_target = vec2_dist(anchor, target);
    t->at_limit = (dist_to_target >= total_len);

    if (t->at_limit) {
        /*
         * Target unreachable: stretch chain collinearly toward target.
         * Each joint is placed at cumulative link-length offsets along
         * the anchor→target unit direction.
         */
        Vec2  dir   = vec2_norm(vec2_sub(target, anchor));
        float accum = 0.0f;
        t->pos[0] = anchor;
        for (int i = 0; i < N_LINKS; i++) {
            accum += t->link_len[i];
            t->pos[i+1] = vec2_add(anchor, vec2_scale(dir, accum));
        }
        return 1;
    }

    /* Iterative FABRIK — up to MAX_ITER backward+forward pass pairs */
    int iter = 0;
    for (iter = 0; iter < MAX_ITER; iter++) {

        /* ── BACKWARD PASS: drag tip to target, slide each joint tip→root ── */
        t->pos[N_JOINTS - 1] = target;
        for (int i = N_JOINTS - 2; i >= 0; i--) {
            Vec2 dir  = vec2_norm(vec2_sub(t->pos[i], t->pos[i+1]));
            t->pos[i] = vec2_add(t->pos[i+1], vec2_scale(dir, t->link_len[i]));
            /*
             * Apply joint angle constraint immediately after repositioning
             * joint i.  This is the key addition over a bare FABRIK solver:
             * before moving on to joint i−1, we check whether joint i bends
             * more than MAX_JOINT_BEND and rotate the outgoing link to clamp.
             */
            if (i > 0) apply_joint_constraint(t, i);
        }

        /* ── FORWARD PASS: restore root, propagate fix toward tip ── */
        t->pos[0] = anchor;
        for (int i = 0; i < N_JOINTS - 1; i++) {
            Vec2 dir    = vec2_norm(vec2_sub(t->pos[i+1], t->pos[i]));
            t->pos[i+1] = vec2_add(t->pos[i], vec2_scale(dir, t->link_len[i]));
        }

        /* Convergence check: is the tip now within CONV_TOL of target? */
        if (vec2_dist(t->pos[N_JOINTS - 1], target) < CONV_TOL) {
            iter++;   /* count this successful iteration */
            break;
        }
    }

    return iter;
}

/* ── §5d  Lissajous target + smooth tracking ───────────────────────── */

/*
 * update_target() — advance scene_time and smooth-track the Lissajous target.
 *
 * Called once per physics tick from scene_tick().
 *
 * STEP 1 — advance scene_time:
 *   scene_time += dt × speed_scale
 *   Separating scene_time from wall time lets speed_scale stretch/compress
 *   the Lissajous path without touching FABRIK or joint constraints.
 *
 * STEP 2 — compute raw Lissajous position:
 *   LIS_AX = screen_width_px  × LIS_AX_FACTOR   (55% of pixel width)
 *   LIS_AY = screen_height_px × LIS_AY_FACTOR   (38% of pixel height)
 *   lx = anchor.x + LIS_AX × cos(LIS_OMEGA_X × scene_time)
 *   ly = anchor.y + LIS_AY × sin(LIS_OMEGA_Y × scene_time + LIS_PHASE_Y)
 *
 * STEP 3 — lerp actual_target toward the Lissajous point:
 *   rate = clamp(dt × TARGET_SMOOTH, 0, 1)
 *   actual_target = lerp(actual_target, (lx, ly), rate)
 *
 *   This is the key smoothing step.  At 60 Hz, rate ≈ 0.133, giving a
 *   time constant of ~125 ms.  Without it, actual_target would jump by the
 *   full Lissajous velocity each tick; the tentacle would snap to sudden
 *   direction changes and the joint constraints would fire aggressively.
 *   With the lerp, the effective target velocity is bounded so the
 *   tentacle body always forms smooth curves.
 *
 * STEP 4 — record actual_target in the ghost trail ring buffer:
 *   trail_pts[trail_write] = actual_target
 *   trail_write = (trail_write + 1) mod TRAIL_POINTS
 *   trail_fill++  (capped at TRAIL_POINTS)
 *   This overwrites the oldest entry once the buffer is full.  At 60 Hz,
 *   TRAIL_POINTS=120 gives exactly 2 seconds of target history.
 */
static void update_target(Tentacle *t, float dt, int cols, int rows)
{
    t->scene_time += dt * t->speed_scale;

    float lis_ax = (float)(cols * CELL_W) * LIS_AX_FACTOR;
    float lis_ay = (float)(rows * CELL_H) * LIS_AY_FACTOR;

    Vec2 lis_target = {
        t->anchor.x + lis_ax * cosf(LIS_OMEGA_X * t->scene_time),
        t->anchor.y + lis_ay * sinf(LIS_OMEGA_Y * t->scene_time + LIS_PHASE_Y)
    };

    float rate = clampf(dt * TARGET_SMOOTH, 0.0f, 1.0f);
    t->actual_target = vec2_lerp(t->actual_target, lis_target, rate);

    /* Record smoothed target in the ghost trail ring buffer */
    t->trail_pts[t->trail_write] = t->actual_target;
    t->trail_write = (t->trail_write + 1) % TRAIL_POINTS;
    if (t->trail_fill < TRAIL_POINTS) t->trail_fill++;
}

/* ── §5e  ghost trail ring buffer ──────────────────────────────────── */

/*
 * trail_get() — retrieve a trail point k steps back from the most recent.
 *
 *   k = 0 → most recently written (current actual_target)
 *   k = 1 → one tick earlier
 *   k = trail_fill−1 → oldest valid entry
 *
 * The expression (trail_write − 1 − k + TRAIL_POINTS) % TRAIL_POINTS
 * converts from "how many ticks back" to the physical ring buffer index
 * without ever computing a negative modulo (C's % is implementation-defined
 * for negative operands before C99 on some compilers; adding TRAIL_POINTS
 * before subtracting k keeps the operand positive as long as k < TRAIL_POINTS,
 * which is always true from the callers).
 */
static inline Vec2 trail_get(const Tentacle *t, int k)
{
    int idx = (t->trail_write - 1 - k + TRAIL_POINTS) % TRAIL_POINTS;
    return t->trail_pts[idx];
}

/* ── §5f  rendering ─────────────────────────────────────────────────── */

/*
 * joint_pair() — ncurses colour pair index for joint / link i.
 *
 * Maps i linearly from root (i=0, pair 1, deepest) to tip
 * (i=N_JOINTS−1, pair 7, brightest):
 *
 *   pair = 1 + (i × 6) / (N_JOINTS − 1)
 *
 * With N_JOINTS = 13:
 *   i=0  → 1 + 0/12  = 1   (deep root colour)
 *   i=2  → 1 + 1/2   = 2
 *   i=6  → 1 + 3     = 4   (mid-body)
 *   i=12 → 1 + 6     = 7   (bright tip colour)
 *
 * The gradient gives an immediate visual reading of which end is anchored.
 */
static int joint_pair(int i)
{
    int p = 1 + (i * 6) / (N_JOINTS - 1);
    if (p < 1) p = 1;
    if (p > 7) p = 7;
    return p;
}

/*
 * joint_attr() — ncurses text attribute for joint / link i.
 *
 * Root 3 links (i ≤ 2):           A_DIM    — heavy, anchored, darker.
 * Tip 3 joints (i ≥ N_JOINTS−4):  A_BOLD   — glowing, active.
 * Middle body:                      A_NORMAL — standard brightness.
 *
 * Combined with the colour gradient, this creates a subtle depth effect:
 * the root looks deeply embedded; the tip glows with urgency.
 */
static attr_t joint_attr(int i)
{
    if (i <= 2)              return A_DIM;
    if (i >= N_JOINTS - 4)  return A_BOLD;
    return A_NORMAL;
}

/*
 * joint_node_char() — bead marker character for joint position i.
 *
 * The three character sizes reinforce the root→tip taper visually:
 *
 *   Root quarter  (i ≤ (N−1)/3):    '0'  — thick, dense, anchored look.
 *   Tip quarter   (i ≥ (N−1)×2/3):  '.'  — tiny, light, nimble tip.
 *   Middle body:                     'o'  — standard medium bead.
 *
 * With N_JOINTS=13: N−1=12.
 *   '0' for i ≤ 4    (root third: joints 0–4)
 *   'o' for i = 5,6,7 (middle third)
 *   '.' for i ≥ 8    (tip third: joints 8–12)
 *
 * These glyphs are stamped in Pass 2 of the bead renderer, overwriting
 * the 'o' fill characters placed in Pass 1.  The result is a chain of
 * beads that gradually transitions from bold segments near the anchor
 * to delicate dots near the seeking tip.
 */
static chtype joint_node_char(int i)
{
    if (i <= (N_JOINTS - 1) / 3)      return (chtype)'0';
    if (i >= (N_JOINTS - 1) * 2 / 3)  return (chtype)'.';
    return (chtype)'o';
}

/*
 * draw_link_beads() — fill link a→b with 'o' beads at DRAW_STEP_PX intervals.
 *
 * This is Pass 1 of the two-pass bead renderer.  It walks from joint a to
 * joint b in increments of DRAW_STEP_PX pixels, converting each sample to
 * a terminal cell and stamping 'o' there.
 *
 * DEDUPLICATION: prev_cx/prev_cy track the most recently stamped cell.
 * If the current sample maps to the same cell, it is skipped.  This avoids
 * double-stamping (which would cause a flicker artefact where two identical
 * wattron/mvwaddch/wattroff calls fight over the same cell).
 *
 * OUT-OF-BOUNDS CLIPPING: samples outside [0,cols)×[0,rows) are silently
 * skipped so no special handling is needed if the tentacle partially exits
 * the terminal boundary.
 *
 * The 'o' characters placed here will be partially overwritten by Pass 2
 * (joint node markers), giving the chain its articulated bead-on-string look.
 */
static void draw_link_beads(WINDOW *w, Vec2 a, Vec2 b,
                             int pair, attr_t attr, int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx*dx + dy*dy);
    if (len < 0.1f) return;

    int nsteps = (int)ceilf(len / DRAW_STEP_PX) + 1;
    int prev_cx = -9999, prev_cy = -9999;

    for (int s = 0; s <= nsteps; s++) {
        float u  = (float)s / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx*u);
        int   cy = px_to_cell_y(a.y + dy*u);
        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, 'o');
        wattroff(w, COLOR_PAIR(pair) | attr);
    }
}

/*
 * render_tentacle() — compose the complete tentacle frame for one render tick.
 *
 * alpha ∈ [0, 1) is the sub-tick interpolation factor from the fixed-step
 * accumulator in §8.  All positions are lerped between their prev_* snapshot
 * (start of last physics tick) and their current value (end of last tick).
 * This eliminates micro-stutter at any combination of sim Hz and render Hz.
 *
 * DRAW ORDER (must not change — later draws overwrite earlier ones):
 *
 *   1. Ghost trail ('.' dots, pair 6, A_DIM)
 *      Every 3rd stored target position, oldest→newest, drawn first so
 *      the tentacle body always appears on top of the trail.
 *
 *   2. Pass 1 — bead fill (draw_link_beads)
 *      Each link i draws 'o' fill beads from rp[i] to rp[i+1] using the
 *      gradient pair and attribute for that link.  Drawn root→tip so that
 *      tip links overwrite root links where they cross.
 *
 *   3. Pass 2 — joint node markers (joint_node_char)
 *      Stamps '0'/'o'/'.' at each joint cell, overwriting the fill beads
 *      at joint positions.  Same draw order (root→tip).
 *
 *   4. Target marker ('*' pair 6 A_BOLD if converged, '#' pair 2 if at_limit)
 *      The '#' signals the target is out of reach so the arm is stretched.
 *
 *   5. Anchor marker ('0' pair 1 A_BOLD)
 *      Drawn last so the anchor glyph is never overwritten by any link.
 */
static void render_tentacle(const Tentacle *t, WINDOW *w,
                             int cols, int rows, float alpha)
{
    /* Build interpolated joint positions for this render frame */
    Vec2 rp[N_JOINTS];
    for (int i = 0; i < N_JOINTS; i++) {
        rp[i] = vec2_lerp(t->prev_pos[i], t->pos[i], alpha);
    }

    /* Interpolate target marker position */
    Vec2 rt = vec2_lerp(t->prev_target, t->actual_target, alpha);

    /* 1. Ghost trail — every 3rd point, pair 6, A_DIM */
    for (int k = 0; k < t->trail_fill; k += 3) {
        Vec2 pt = trail_get(t, k);
        int  cx = px_to_cell_x(pt.x);
        int  cy = px_to_cell_y(pt.y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(6) | A_DIM);
        mvwaddch(w, cy, cx, '.');
        wattroff(w, COLOR_PAIR(6) | A_DIM);
    }

    /* 2. Pass 1: fill each link with 'o' beads (gradient root→tip) */
    for (int i = 0; i < N_LINKS; i++) {
        int    pair = joint_pair(i);
        attr_t attr = joint_attr(i);
        draw_link_beads(w, rp[i], rp[i+1], pair, attr, cols, rows);
    }

    /* 3. Pass 2: joint node markers stamped over the fill beads */
    for (int i = 0; i < N_JOINTS; i++) {
        int cx = px_to_cell_x(rp[i].x);
        int cy = px_to_cell_y(rp[i].y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        int    pair  = joint_pair(i);
        attr_t attr  = joint_attr(i);
        chtype glyph = joint_node_char(i);
        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
        wattroff(w, COLOR_PAIR(pair) | attr);
    }

    /* 4. Target marker: '*' (converged) or '#' (at reach limit) */
    int tx = px_to_cell_x(rt.x);
    int ty = px_to_cell_y(rt.y);
    if (tx >= 0 && tx < cols && ty >= 0 && ty < rows) {
        if (t->at_limit) {
            wattron(w, COLOR_PAIR(2) | A_BOLD);
            mvwaddch(w, ty, tx, '#');
            wattroff(w, COLOR_PAIR(2) | A_BOLD);
        } else {
            wattron(w, COLOR_PAIR(6) | A_BOLD);
            mvwaddch(w, ty, tx, '*');
            wattroff(w, COLOR_PAIR(6) | A_BOLD);
        }
    }

    /* 5. Anchor marker — always drawn last so nothing overwrites it */
    int ax = px_to_cell_x(t->anchor.x);
    int ay = px_to_cell_y(t->anchor.y);
    if (ax >= 0 && ax < cols && ay >= 0 && ay < rows) {
        wattron(w, COLOR_PAIR(1) | A_BOLD);
        mvwaddch(w, ay, ax, '0');
        wattroff(w, COLOR_PAIR(1) | A_BOLD);
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Tentacle tentacle; } Scene;

/*
 * scene_init() — initialise the tentacle to a clean, immediately-animated state.
 *
 * ANCHOR POSITION:
 *   Placed at 50% × screen pixel width, 50% × screen pixel height — screen
 *   centre.  This maximises the Lissajous path's visibility: the target
 *   can swing in all four directions symmetrically.
 *
 * INITIAL JOINT POSITIONS:
 *   All joints initialised in a straight horizontal line extending rightward
 *   from the anchor, at cumulative link-length offsets.  This gives FABRIK
 *   a clean, non-degenerate starting configuration on the first tick.
 *
 * LINK LENGTHS (tapered):
 *   link_len[i] = BASE_LINK_LEN − i × TAPER  (min clamped to 4.0 px).
 *   Computed once here and held constant for the simulation's lifetime.
 *
 * INITIAL TARGET:
 *   Set to the Lissajous starting position (scene_time=0, phase 0):
 *   actual_target = (anchor.x + LIS_AX, anchor.y) — far right of anchor.
 *   prev_target = actual_target (so the first frame's alpha lerp is identity).
 *
 * TRAIL PRE-SEEDING:
 *   All TRAIL_POINTS entries are initialised to actual_target so the trail
 *   is full from frame one.  Without pre-seeding, the first TRAIL_POINTS
 *   ticks would show an ugly cluster of dots all at the initial position.
 *   trail_fill = TRAIL_POINTS, trail_write = 0 (next write at index 0).
 *
 * INITIAL FABRIK SOLVE:
 *   One fabrik_solve() call before the first frame ensures last_iter and
 *   at_limit are valid for the HUD display from tick zero.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Tentacle *t = &sc->tentacle;

    t->anchor.x   = (float)(cols * CELL_W) * 0.50f;
    t->anchor.y   = (float)(rows * CELL_H) * 0.50f;
    t->speed_scale = 1.0f;
    t->scene_time  = 0.0f;
    t->paused      = false;
    t->theme_idx   = 0;

    /* Tapered link lengths: root longest, tip shortest */
    for (int i = 0; i < N_LINKS; i++) {
        t->link_len[i] = BASE_LINK_LEN - (float)i * TAPER;
        if (t->link_len[i] < 4.0f) t->link_len[i] = 4.0f;
    }

    /* Initial joint positions: straight horizontal line from anchor */
    float x = t->anchor.x;
    t->pos[0] = t->anchor;
    for (int i = 0; i < N_LINKS; i++) {
        x += t->link_len[i];
        t->pos[i+1] = (Vec2){ x, t->anchor.y };
    }

    /* Initial target: Lissajous starting position (t=0, max x displacement) */
    float lis_ax = (float)(cols * CELL_W) * LIS_AX_FACTOR;
    t->actual_target = (Vec2){ t->anchor.x + lis_ax, t->anchor.y };
    t->prev_target   = t->actual_target;

    /* One solve before frame 1 so HUD values are valid immediately */
    t->last_iter = fabrik_solve(t, t->actual_target, t->anchor);
    memcpy(t->prev_pos, t->pos, sizeof t->pos);

    /* Pre-seed trail so it is fully populated from frame one */
    for (int k = 0; k < TRAIL_POINTS; k++) {
        t->trail_pts[k] = t->actual_target;
    }
    t->trail_fill  = TRAIL_POINTS;
    t->trail_write = 0;
}

/*
 * scene_tick() — one fixed-step physics update.
 *
 * dt is the fixed tick duration in seconds (= 1.0 / sim_fps).
 * cols and rows are the current terminal dimensions (used by update_target
 * to compute the Lissajous amplitudes from screen size).
 *
 * ORDER IS IMPORTANT:
 *   1. Save prev_pos[] and prev_target FIRST — before any physics runs.
 *      These are the interpolation anchors for render_tentacle(); they must
 *      hold the state from the END of the PREVIOUS tick.
 *      Saving after the update would produce a lerp that overshoots.
 *
 *   2. Return early if paused — prev_pos is still saved so the alpha lerp
 *      produces a clean freeze (prev = curr → lerp is identity → no motion).
 *
 *   3. update_target() — advances scene_time, computes the Lissajous point,
 *      lerps actual_target toward it, records it in the ghost trail.
 *
 *   4. fabrik_solve() — runs the FABRIK IK solver on actual_target; stores
 *      iteration count in t->last_iter; updates pos[].
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    Tentacle *t = &sc->tentacle;

    /* Step 1 — save snapshots for alpha lerp */
    memcpy(t->prev_pos, t->pos, sizeof t->pos);
    t->prev_target = t->actual_target;

    if (t->paused) return;   /* Step 2 — frozen: lerp is identity */

    update_target(t, dt, cols, rows);                              /* Step 3 */
    t->last_iter = fabrik_solve(t, t->actual_target, t->anchor);  /* Step 4 */
}

/*
 * scene_draw() — render the scene; called once per render frame by §7.
 *
 * alpha ∈ [0, 1) is the sub-tick interpolation factor (see §8 main()).
 * dt_sec is the physics tick duration; unused here (the tentacle needs only
 * position data, not velocity, for rendering).
 */
static void scene_draw(const Scene *sc, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)dt_sec;
    render_tentacle(&sc->tentacle, w, cols, rows, alpha);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — ncurses display layer.
 *
 * Holds the current terminal dimensions (cols, rows) read after each
 * SIGWINCH resize.  See framework.c §7 for the full double-buffer
 * rationale (erase → draw → wnoutrefresh → doupdate).
 */
typedef struct { int cols, rows; } Screen;

/*
 * screen_init() — configure the terminal for animation.
 *
 *   initscr()         initialise ncurses; must be first call.
 *   noecho()          do not echo typed characters to the screen.
 *   cbreak()          pass keys immediately, without line buffering.
 *   curs_set(0)       hide the hardware cursor (stops blinking cursor).
 *   nodelay(TRUE)     getch() returns ERR immediately if no key available;
 *                     the render loop never stalls waiting for input.
 *   keypad(TRUE)      decode arrow keys and function keys into single
 *                     KEY_* constants rather than multi-byte escape sequences.
 *   typeahead(-1)     disable ncurses' mid-output read() calls to check for
 *                     typeahead; without this, output can be interrupted and
 *                     frames arrive visually torn.
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
 * endwin() re-enables echo, shows the cursor, and restores the scroll region.
 */
static void screen_free(Screen *s) { (void)s; endwin(); }

/*
 * screen_resize() — handle a pending SIGWINCH event.
 *
 * endwin() + refresh() forces ncurses to re-read LINES and COLS from the
 * kernel and resize its internal virtual screens (curscr/newscr) to match.
 * Without this, stdscr retains the old size and mvwaddch at newly-valid
 * coordinates silently fails, producing rendering artefacts.
 *
 * Called from app_do_resize() which also re-anchors the tentacle.
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
 *   1. erase()        — clear newscr in memory; NO terminal I/O yet.
 *   2. scene_draw()   — write tentacle, trail, target, anchor glyphs.
 *   3. Top HUD bar    — written last so it always appears on top of the body.
 *   4. Bottom hint bar — key reference; also on top.
 *
 * Nothing reaches the terminal until screen_present() calls doupdate().
 *
 * HUD content: N=12, FABRIK iteration count, converged/at-limit state,
 *              speed scale, current theme name, fps, sim Hz.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps,
                        float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Tentacle *t = &sc->tentacle;
    const char *state_str = t->at_limit ? "at-limit" : "converged";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-FABRIK  N=12  iter:%2d  %s  spd:%.2fx  [%s]  %.1ffps  %dHz ",
             t->last_iter,
             state_str,
             t->speed_scale,
             THEMES[t->theme_idx].name,
             fps,
             sim_fps);

    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);

    attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  w/s:speed  t:theme  [/]:Hz  %s ",
             t->paused ? "PAUSED" : "seeking");
    attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

/*
 * screen_present() — flush the composed frame to the terminal in one write.
 *
 * wnoutrefresh(stdscr) copies newscr into ncurses' "pending update" queue.
 *   No terminal I/O yet.
 * doupdate() diffs newscr against curscr (what is physically on screen),
 *   sends only the changed cells to the terminal fd, then sets curscr=newscr.
 *
 * This two-step sequence batches all window updates into one diff write,
 * which is more efficient than wrefresh(stdscr) (= wnoutrefresh + doupdate
 * in a single call) because it allows multiple windows to be combined.
 * For this program (single window) the result is identical but the pattern
 * matches framework.c and is correct for future multi-window extensions.
 */
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level application state, accessible from signal handlers.
 *
 * Declared as a file-scope global (g_app) because POSIX signal handlers
 * receive no user-defined argument; they can only communicate via globals.
 *
 * running and need_resize are volatile sig_atomic_t because:
 *   volatile     — prevents the compiler from caching the value in a register
 *                  across the signal-handler write (optimiser must re-read).
 *   sig_atomic_t — the only integer type POSIX guarantees can be written
 *                  atomically from a signal handler on all conforming platforms.
 *                  Using int here would technically be UB on some architectures.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* Signal handlers — set flags only; never call ncurses or malloc here */
static void on_exit_signal(int sig)   { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }

/*
 * cleanup() — atexit safety net.
 * Registered with atexit() so endwin() is always called even if the process
 * exits via an unhandled signal path that bypasses screen_free().  Without
 * this, a crash or unhandled signal would leave the terminal in raw/no-echo
 * mode (the "scrambled terminal" problem).
 */
static void cleanup(void) { endwin(); }

/*
 * app_do_resize() — handle a pending SIGWINCH terminal resize.
 *
 * screen_resize() re-reads LINES/COLS from the kernel.  After the resize,
 * the anchor is re-centred to 50% of the new screen dimensions.  The rest
 * of the chain follows naturally on the next FABRIK solve.
 *
 * frame_time and sim_accum are reset in the main loop after this returns
 * to prevent a large dt from the resize pause from firing a physics
 * avalanche (many FABRIK solves in a single frame).
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Tentacle *t  = &app->scene.tentacle;
    float     wpx = (float)(app->screen.cols * CELL_W);
    float     hpx = (float)(app->screen.rows * CELL_H);

    /* Re-centre the anchor to the new screen proportions */
    t->anchor.x = wpx * 0.50f;
    t->anchor.y = hpx * 0.50f;
    t->pos[0]   = t->anchor;

    app->need_resize = 0;
}

/*
 * app_handle_key() — process one keypress; return false to quit.
 *
 * KEY MAP:
 *   q / Q / ESC   quit (running = 0; screen_free() called in main)
 *   space         toggle paused (scene_tick skips physics while paused)
 *
 *   w / = / +     speed_scale × 1.25  (target moves faster; max 8.0)
 *   s / -         speed_scale ÷ 1.25  (target moves slower; min 0.05)
 *     speed_scale multiplies scene_time advancement, not FABRIK frequency.
 *     8.0 maximum: at this rate the Lissajous path runs 8× real-time;
 *     the joint constraints fire nearly every iteration, visible as stiffening.
 *     0.05 minimum: so slow the path barely moves — useful for studying the
 *     FABRIK solver in near-static detail.
 *
 *   t             next theme (theme_idx + 1 mod N_THEMES)
 *   T             previous theme (theme_idx − 1 mod N_THEMES)
 *     theme_apply() is called immediately, taking effect on the next frame.
 *     theme_idx is stored in the Tentacle struct (not App) so it survives
 *     hypothetical future resets.
 *
 *   ] / [         sim_fps ± SIM_FPS_STEP  (clamped to [SIM_FPS_MIN, MAX])
 *     Raising sim Hz fires FABRIK more often per second → smoother tracking
 *     at fast target speeds.  Lowering it reduces CPU cost at the price of
 *     visible "stepping" between physics ticks (compensated by alpha lerp
 *     in the renderer, but the lerp cannot compensate for large physics gaps).
 */
static bool app_handle_key(App *app, int ch)
{
    Tentacle *t = &app->scene.tentacle;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': t->paused = !t->paused; break;

    case 'w': case '=': case '+':
        t->speed_scale *= 1.25f;
        if (t->speed_scale > 8.0f) t->speed_scale = 8.0f;
        break;
    case 's': case '-':
        t->speed_scale /= 1.25f;
        if (t->speed_scale < 0.05f) t->speed_scale = 0.05f;
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
 * main() — fixed-step accumulator game loop
 *
 * Structure is identical to framework.c §8.  The loop body executes
 * these eight steps every frame:
 *
 *  ① RESIZE CHECK
 *     If need_resize is set (by SIGWINCH handler), call app_do_resize()
 *     before touching any ncurses state.  Reset frame_time and sim_accum
 *     so the large dt that accumulated during the resize does not inject
 *     a physics avalanche (many FABRIK solves in one frame).
 *
 *  ② MEASURE dt
 *     Wall-clock nanoseconds since the previous frame start.
 *     Capped at 100 ms: if the process was suspended (Ctrl-Z, debugger
 *     breakpoint) and resumed, uncapped dt would fire ~6 FABRIK solves
 *     at 60 Hz — visible as a sudden position jump.
 *
 *  ③ FIXED-STEP ACCUMULATOR
 *     sim_accum accumulates raw wall-clock dt each frame.
 *     tick_ns = NS_PER_SEC / sim_fps  (nanoseconds per physics tick).
 *     While sim_accum ≥ tick_ns: fire scene_tick(), drain tick_ns.
 *     Result: FABRIK runs at exactly sim_fps Hz regardless of render rate.
 *     Normally one tick fires per frame (when sim Hz = render Hz = 60).
 *     If a frame takes 30 ms, two ticks fire to catch up.
 *
 *  ④ ALPHA — sub-tick interpolation factor
 *     After draining, sim_accum holds the fractional leftover — how far
 *     into the next unfired tick we are currently.
 *       alpha = sim_accum / tick_ns  ∈ [0, 1)
 *     Passed to render_tentacle() so all positions are lerped between
 *     the previous tick (alpha=0) and the current tick (alpha→1).
 *     This eliminates the micro-stutter that would otherwise appear
 *     when render Hz is not an exact multiple of sim Hz.
 *
 *  ⑤ FPS COUNTER
 *     Frames are counted over a 500 ms sliding window.
 *     fps = frame_count / (elapsed_ns / 1e9) — division happens only
 *     every 500 ms (not every frame), keeping the HUD value stable.
 *
 *  ⑥ FRAME CAP — sleep BEFORE render
 *     elapsed = time spent on physics since frame_time was captured.
 *     budget  = NS_PER_SEC / 60  (one 60-fps frame = 16.67 ms).
 *     sleep   = budget − elapsed.
 *     Sleeping before the terminal write means I/O cost is not charged
 *     against the next frame's physics budget.  If elapsed > budget
 *     (FABRIK took longer than one frame), clock_sleep_ns() returns
 *     immediately without sleeping.
 *
 *  ⑦ DRAW + PRESENT
 *     screen_draw(): erase → scene_draw (tentacle + HUD bars) into newscr.
 *     screen_present(): wnoutrefresh + doupdate → atomic diff write to terminal.
 *     Only cells that changed from the previous frame are transmitted,
 *     typically ~150 cells for a moving tentacle on a dark background.
 *
 *  ⑧ DRAIN INPUT
 *     Loop getch() until ERR, processing every queued key event.
 *     Looping (not single getch) ensures all key-repeat events are handled
 *     within the same frame they arrive, keeping controls responsive.
 * ───────────────────────────────────────────────────────────────────── */
int main(void)
{
    /* Seed RNG from monotonic clock so each run looks visually different */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));

    /* Safety net: endwin() called even if we exit via an unhandled path */
    atexit(cleanup);

    /* SIGINT / SIGTERM — graceful exit from Ctrl-C or system kill */
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);

    /* SIGWINCH — terminal resize; processed at top of next iteration */
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
    double  fps_display = 0.0;          /* smoothed fps value for HUD         */

    while (app->running) {

        /* ── ① resize ─────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset: don't accumulate resize pause */
            sim_accum  = 0;
        }

        /* ── ② dt ─────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;   /* suspend guard */

        /* ── ③ fixed-step accumulator ─────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);   /* ns per physics tick   */
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── ④ alpha ──────────────────────────────────────────────── */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── ⑤ fps counter ────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── ⑥ frame cap — sleep before render ───────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── ⑦ draw + present ─────────────────────────────────────── */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps,
                    alpha, dt_sec);
        screen_present();

        /* ── ⑧ drain all pending input ───────────────────────────── */
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
