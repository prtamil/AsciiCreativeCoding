/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hexpod_tripod.c — 2-D hexapod walker with tripod gait + 2-joint IK
 *
 * DEMO: A 6-legged rigid-body robot walks across the terminal in an
 *       alternating tripod gait — three feet always planted in a
 *       stable support triangle while the other three swing forward.
 *       Each leg is a 2-joint chain solved each frame by law-of-cosines
 *       IK; arrow keys steer the heading.
 *
 * Study alongside: ik_spider.c        (2-joint IK + per-leg gait)
 *                  ragdoll_figure.c   (framework structure)
 *
 * Section map:
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep
 *   §3  color         — 8 robot themes + spec HUD/hint pairs
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Hexapod = rigid body + 6 legs
 *       §5a  vec helpers + per-leg static tables
 *       §5b  solve_ik         — analytical 2-joint IK
 *       §5c  hip_world_pos    — body-local → world transform
 *       §5d  rest_target      — ideal foot rest position
 *       §5e  gait_tick        — tripod state machine
 *       §5f  hexapod_tick     — body + gait + IK orchestrator
 *       §5g  rendering helpers (seg_glyph, draw_leg_line)
 *       §5h  hexapod_draw     — full-frame compositor
 *   §6  scene         — thin Scene wrapper
 *   §7  screen        — ncurses double-buffer display layer
 *   §8  app           — signals, resize, main game loop
 *
 * Keys:  q / ESC      quit                  space  pause / resume
 *        ↑ ↓ ← →      steer heading         w / s  speed × / ÷ 1.25
 *        t            cycle theme           [ / ]  time scale (0.25× .. 4×)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra animation/hexpod_tripod.c \
 *       -o hexpod_tripod -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Two interlocked sub-systems share one body.
 *
 *                 GAIT — a clock that keeps the robot upright. The six
 *                 legs are split into two interlocked tripods:
 *                     A = {left-front 0, right-mid 3, left-rear 4}
 *                     B = {right-front 1, left-mid 2, right-rear 5}
 *                 At all times, exactly one tripod's three feet are
 *                 PLANTED (forming a stable support triangle under the
 *                 body) while the other three SWING forward in
 *                 parabolic arcs to new rest targets. Phases swap when
 *                 the timer expires AND every swinging foot has
 *                 finished its arc — never mid-stride.
 *
 *                 IK — a law-of-cosines solver placing knee joints.
 *                 For each leg the femur (U) and tibia (L) form a
 *                 triangle with the hip→foot vector; the knee angle
 *                 satisfies cos(ah) = (d² + U² − L²)/(2·d·U). Left and
 *                 right legs use opposite signs of ah so left knees
 *                 break toward −y and right knees toward +y.
 *
 *                 STEERING — heading interpolates toward target_heading
 *                 at TURN_RATE rad/s (short-arc through ±π so a flip
 *                 takes the shorter path).
 *
 * Data-structure: Hexapod owns body kinematics (body_x, body_y, heading,
 *                 target_heading, body_speed), per-leg state (foot_pos,
 *                 foot_old, step_target, stepping flag, swing progress
 *                 step_t), per-leg derived (hip, knee) recomputed each
 *                 frame, and the gait state machine (gait_phase,
 *                 phase_timer). Static tables HIP_LOCAL_X/Y, REST_*,
 *                 TRIPOD_A/B encode the per-leg geometry.
 *
 * Rendering     : Painter's order — leg lines first (femur pair 2,
 *                 tibia pair 3), then foot markers ('*' planted bold,
 *                 'o' swinging dim), then knee markers, then the body
 *                 rectangle (4 edges + 2 cross-braces), then hip
 *                 attachment markers '+', and finally the body center
 *                 '@'. Direction-glyph line rasterisation uses the
 *                 same dense-stepping pattern as the FK demos.
 *
 * Performance   : Variable timestep at render rate — no fixed-step
 *                 accumulator, no alpha lerp, no prev/cur snapshots.
 *                 Per frame: 6 IK solves (each ≈ 2 sqrtf + 1 atan2f +
 *                 1 acosf), gait state machine, body integration,
 *                 ~30 cell stamps for legs + body. Microseconds total.
 *
 * References    :
 *   Wikipedia, "Tripod gait" — biological reference for the alternating
 *     three-leg pattern used by insects and 6-legged robots.
 *   Wikipedia, "Inverse kinematics" — derives the law-of-cosines
 *     2-joint solver used in solve_ik().
 *   Hirose, "Biologically Inspired Robots: Snake-Like Locomotors and
 *     Manipulators" (1993) — foundational hexapod gait references.
 *   Reynolds, "Steering Behaviors for Autonomous Characters" (1999) —
 *     the heading-toward-target interpolation pattern used here.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A rigid body slides along its heading; six legs hang off it. The
 * legs play a duet: three are always PLANTED (anchored to the world,
 * forming a stable triangle), three are SWINGING (lifting off, arcing
 * forward, landing at a new spot). Every tick: re-derive hips from the
 * body's pose, advance any swinging feet, then solve law-of-cosines IK
 * to place the knees. The body never tips because the planted tripod
 * is always there.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Body : a brick on rails. You tell it which way to face; it slides
 *        forward at body_speed. Gravity is irrelevant — the legs
 *        aren't bearing weight, they're a visual.
 *
 * Legs : six 2-bar linkages (femur + tibia). Given the hip's world
 *        position and the foot's target, the knee falls out of
 *        elementary trigonometry. No state stored on the leg itself
 *        beyond "is it stepping right now, and how far through the
 *        step?"
 *
 * Gait : a metronome with two phases. Phase A: legs A swing, legs B
 *        plant. Phase B: legs B swing, legs A plant. Phase swap
 *        happens after PHASE_DURATION elapses AND all swinging feet
 *        have landed — whichever is later. This guarantees the
 *        support tripod is always complete.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. Steer: heading interpolates toward target_heading at TURN_RATE
 *     rad/s, taking the short arc through ±π.
 *  3. Translate body along heading by body_speed · dt; wrap toroidally.
 *  4. Recompute every world-space hip from the body's rotated pose.
 *  5. Stretch-snap: if any foot is now beyond IK reach (e.g. body just
 *     wrapped around the edge), snap that foot to its rest target.
 *  6. Gait tick:
 *       a. For each leg in the currently swinging tripod, advance
 *          step_t; place the foot at lerp(foot_old, step_target,
 *          smoothstep(step_t)) plus a parabolic Y-arc.
 *       b. If phase_timer ≥ PHASE_DURATION AND every swinging foot
 *          has landed (step_t = 1), swap tripods: the just-landed
 *          tripod plants, the other launches with new step_targets.
 *  7. For every leg, solve law-of-cosines IK to find knee position.
 *  8. Render painter's order: leg lines → foot markers → knee markers
 *     → body box → hips → center.
 *
 * KEY FORMULAS
 * ────────────
 *  Heading lerp  : turn = clamp(target − heading, ±TURN_RATE · dt)
 *                  (with target − heading wrapped to [−π, π])
 *
 *  2-joint IK    : dist  = clamp(|T − H|, |U − L| + 1, U + L − 1)
 *                  base  = atan2(Ty − Hy, Tx − Hx)
 *                  cos_h = (dist² + U² − L²) / (2 · dist · U)
 *                  ah    = acos(clamp(cos_h, −1, 1))
 *                  knee_angle = base ± ah    ( − for left, + for right )
 *                  knee  = H + U · (cos knee_angle, sin knee_angle)
 *
 *  Foot swing    : ease  = smoothstep(step_t)
 *                  hz    = lerp(foot_old, step_target, ease)
 *                  arc_y = −STEP_HEIGHT · sin(π · step_t)
 *                  foot  = (hz.x, hz.y + arc_y)
 *                  ( smoothstep on the lerp, raw step_t on the arc, so
 *                    the foot decelerates into landing while the arc
 *                    peaks symmetrically at step_t = 0.5 )
 *
 *  Step target   : T = hip_world + rotate(REST_offset
 *                                         + (body_speed · LOOKAHEAD, 0),
 *                                         heading)
 *
 *  Tripod groups : A = {0, 3, 4},  B = {1, 2, 5}
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - IK clamp. The hip↔target distance is bounded into
 *    [|U − L| + 1, U + L − 1] so acos(cos_h) never sees a value
 *    outside [−1, 1]. Without the clamp, an over-stretched leg
 *    would yield NaN and the chain would vanish.
 *
 *  - Toroidal wrap during walk. The body moves N pixels per tick,
 *    but planted feet don't wrap with it — they were anchored to
 *    the OLD coordinates. The stretch-snap pass detects feet beyond
 *    reach and forces them to rest. The visible glitch is one frame
 *    of teleporting feet, then the gait recovers naturally.
 *
 *  - Phase swap timing. The metronome fires when phase_timer expires
 *    AND all swinging feet have landed. If swing time > phase time,
 *    the metronome waits. Without this guard, the next tripod could
 *    launch while the current one is still mid-air → robot tips.
 *
 *  - Sharp 180° steer. heading interpolates at TURN_RATE rad/s, so a
 *    π-rad flip takes π/TURN_RATE ≈ 1.26 s. During the turn, planted
 *    feet stay where they were planted; you'll see the body spin
 *    while the gait keeps stepping in body-local coordinates.
 *
 *  - Suspend / lid-close. dt grows large; capped at 100 ms in main()
 *    so the body never teleports across the screen on resume.
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: robot walks rightward at 40 px/s. At any
 *    frozen frame, exactly 3 feet are '*' (planted) and 3 are 'o'
 *    (swinging). The 3 planted feet form a triangle: a left/right
 *    pair at front-or-rear plus the opposite-side mid leg.
 *
 *  - Press arrow keys → heading interpolates toward the new target;
 *    a 90° turn takes ~0.6 s. Body visibly spins; legs keep stepping.
 *
 *  - Press space → everything freezes: gait, body, knees. Un-pause
 *    → motion resumes from exactly where it was.
 *
 *  - Crank speed with `w` → step lookahead grows so feet land further
 *    ahead of the hips. The gait keeps up automatically because step
 *    targets are recomputed every phase swap.
 *
 *  - Press `[` to slow time → motion stays smooth (variable timestep
 *    guarantees this); press `]` for fast-forward.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read ik_helloworld.c first; the IK here is the
 *      same 2-link law-of-cosines, applied 6 times per frame.
 *      The NEW lesson is the GAIT state machine.
 *   2. §5 entity — THE HEART. In sub-section order:
 *        §5a static tables       ← per-leg geometry
 *        §5b solve_ik            ← 2-link IK (T1)
 *        §5c hip_world_pos       ← body→world transform (T2)
 *        §5d rest_target         ← per-leg ideal foot rest
 *        §5e gait_tick           ← TRIPOD GAIT state machine (T3-T5)
 *        §5f hexapod_tick        ← top-level orchestrator
 *        §5g-§5h rendering
 *      Read AFTER tutorials T1-T5.
 *   3. §6 scene — thin wrapper.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   body_x, body_y         body center pixel position.
 *   heading                body facing angle. INTERPOLATES toward
 *                          target_heading at TURN_RATE.
 *   target_heading         what the user just set with arrow keys.
 *   HIP_LOCAL_X/Y[i]       hip i's offset from body in BODY LOCAL
 *                          coordinates (constant per leg).
 *   hip[i]                 hip i's WORLD position (recomputed each
 *                          frame from body pose).
 *   foot_pos[i]            foot i's world position.
 *   foot_old[i]            foot i's position at the start of its
 *                          current swing (the lerp anchor).
 *   step_target[i]         where the foot is heading during swing.
 *   stepping[i]            bool — true iff this leg is currently
 *                          swinging.
 *   step_t[i]              progress through swing ∈ [0, 1].
 *   gait_phase             0 (tripod A swings) or 1 (tripod B).
 *   phase_timer            seconds since last phase swap.
 *   TRIPOD_A, TRIPOD_B     the two leg-index sets.
 *
 * Background you need
 * ───────────────────
 *   - 2-link analytical IK (ik_helloworld T3-T5).
 *   - Body-local → world transform via 2-D rotation +
 *     translation. We'll cover this in T2.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Centre-of-mass / dynamic balance. The body NEVER tips —
 *     it's not gravity-loaded; we just animate it sliding on
 *     rails. Tripod gait is purely AESTHETIC stability.
 *   - 3-D leg geometry. We're planar; femur and tibia all lie
 *     in the screen plane.
 *   - Iterative IK. Closed-form 2-link is exact; no FABRIK.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a tripod-gait hexapod from first
 * principles.
 *
 *   T1  Six 2-link arms — IK applied per leg, every frame
 *   T2  Body-local hips — the rigid-body transform we apply 6 times
 *   T3  The tripod gait — three legs always planted, stable triangle
 *   T4  Step phase machine — time-bounded swings with landing wait
 *   T5  Step target lookahead — feet land where the body is going
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  SIX 2-LINK ARMS — IK APPLIED PER LEG, EVERY FRAME
 * ─────────────────────────────────────────────────────
 * Each leg is a 2-link arm hanging off a hip:
 *
 *     hip ─── femur ─── knee ─── tibia ─── foot
 *      │              (U)               (L)
 *      └─ pinned to the body
 *      └─ foot pinned to the world (planted) or in flight (swinging)
 *
 * For each leg every frame:
 *   1. Compute hip's WORLD position from body pose (T2).
 *   2. The foot is given (either planted at last-frame's pos or
 *      mid-swing as computed by the gait T4).
 *   3. solve_ik(hip, foot, U, L) → knee position.
 *
 * The IK is the SAME law-of-cosines from ik_helloworld T4 — but
 * applied to a triangle (hip, knee, foot) where hip and foot are
 * both KNOWN. This is "place the elbow given the shoulder and
 * the hand position."
 *
 * Sign convention: left legs use elbow-DOWN (knee below the
 * hip-foot line); right legs use elbow-UP. Without that, the
 * left and right legs would visually mirror the wrong way and
 * the body would look broken.
 *
 * The IK is COMPLETELY STATELESS — it doesn't care if the foot
 * is planted or swinging. It just places the knee given the
 * current pair of endpoints. State lives in the GAIT (T3-T5),
 * not the IK.
 *
 * T2  BODY-LOCAL HIPS — THE RIGID-BODY TRANSFORM WE APPLY 6 TIMES
 * ───────────────────────────────────────────────────────────────
 * Hips are pinned to the body. As the body moves and rotates,
 * the hips move and rotate WITH it. Per-leg static tables hold
 * the BODY-LOCAL offset of each hip:
 *
 *     HIP_LOCAL_X[6] = { -2.0, +2.0, -2.0, +2.0, -2.0, +2.0 }
 *     HIP_LOCAL_Y[6] = { -1.5, -1.5,  0.0,  0.0, +1.5, +1.5 }
 *
 * "x = ±2 from body center, y = front / mid / rear." These are
 * constants — the legs never move relative to the body.
 *
 * Every frame, transform LOCAL → WORLD:
 *
 *     for each leg i:
 *       local_x, local_y = HIP_LOCAL_X[i], HIP_LOCAL_Y[i]
 *       cos_h = cos(heading); sin_h = sin(heading)
 *       hip[i].x = body_x + local_x · cos_h − local_y · sin_h
 *       hip[i].y = body_y + local_x · sin_h + local_y · cos_h
 *
 * That's the standard 2-D rigid-body transform: rotate the
 * local offset by the heading, then translate by the body's
 * world position.
 *
 * Same trick is used by every multi-body system: each part's
 * world pose is parent_pose composed with local_offset.
 * Hierarchical animation, scene graphs, and physics engines
 * all operate this way.
 *
 * T3  THE TRIPOD GAIT — THREE LEGS ALWAYS PLANTED, STABLE TRIANGLE
 * ────────────────────────────────────────────────────────────────
 * Real insects use ALTERNATING TRIPOD: divide six legs into two
 * sets of three, where each set forms a TRIANGLE under the body.
 * At any moment, ONE set is planted (forming the support
 * triangle) and the other set is swinging.
 *
 *   TRIPOD A = {LF, RM, LR}   (left-front, right-mid, left-rear)
 *   TRIPOD B = {RF, LM, RR}   (right-front, left-mid, right-rear)
 *
 *      ┌────────────────────────────────────┐
 *      │      LF *           RF o           │
 *      │           ╲ A     B ╱              │
 *      │            ▭body▭                  │
 *      │           ╱ B     A ╲              │
 *      │      LM o           RM *           │
 *      │           ╲ A     B ╱              │
 *      │            ▭body▭                  │
 *      │           ╱ B     A ╲              │
 *      │      LR *           RR o           │
 *      │                                    │
 *      │  '*' planted (tripod A here),      │
 *      │  'o' swinging (tripod B here)      │
 *      └────────────────────────────────────┘
 *
 * Notice each tripod has a leg in EACH front/mid/rear row
 * AND mixes left + right. This guarantees that when those
 * three feet are planted, their support triangle ENCLOSES
 * the body's centre of mass — kinematically stable.
 *
 * Real insects use this gait at speed ~3-5 body lengths per
 * second; slower they switch to wave gait (one leg swinging
 * at a time, five planted).
 *
 * T4  STEP PHASE MACHINE — TIME-BOUNDED SWINGS WITH LANDING WAIT
 * ──────────────────────────────────────────────────────────────
 * The gait is a STATE MACHINE with two phases:
 *
 *     phase 0:   tripod A swings,   tripod B planted
 *     phase 1:   tripod A planted,  tripod B swings
 *
 * Transition rule: SWAP after PHASE_DURATION elapsed AND every
 * swinging foot has reached step_t == 1 (finished its arc).
 *
 *     phase_timer += dt
 *     if phase_timer >= PHASE_DURATION:
 *       all_landed = (every swinging leg has step_t == 1)
 *       if all_landed:
 *         swap phases
 *         phase_timer = 0
 *
 * Why both conditions? Time alone isn't enough — at very high
 * step heights or low swing speeds, a foot may still be
 * mid-arc when the timer expires. Swapping then would launch
 * a new tripod while the previous one is still in the air →
 * three feet up + three feet up = no support triangle = robot
 * tips. The "all landed" guard prevents that.
 *
 * Foot trajectory during swing:
 *
 *     ease     = smoothstep(step_t)                   ∈ [0, 1]
 *     ground   = lerp(foot_old, step_target, ease)
 *     arc_y    = −STEP_HEIGHT · sin(π · step_t)
 *     foot     = ground + (0, arc_y)
 *
 * The smoothstep on horizontal lerp gives soft acceleration /
 * deceleration. The sin-based vertical arc peaks at step_t =
 * 0.5 and lands cleanly at step_t = 1 (the foot kisses the
 * ground at landing — no slam).
 *
 * T5  STEP TARGET LOOKAHEAD — FEET LAND WHERE THE BODY IS GOING
 * ─────────────────────────────────────────────────────────────
 * Where SHOULD a stepping foot land? Not at its current rest
 * position relative to the hip — by the time it lands, the
 * body has moved forward, and the foot would be too far back.
 *
 * Solution: LOOKAHEAD by some amount in the body's heading
 * direction:
 *
 *     T = hip_world + rotate(REST_offset + (body_speed · LOOKAHEAD, 0),
 *                            heading)
 *
 * In words: from the hip's CURRENT world position, walk to
 * the leg's body-local rest offset PLUS a forward push
 * proportional to body_speed. At zero speed the lookahead is
 * zero — feet plant exactly under their rest. At higher
 * speed, feet plant further ahead of the rest, so by the time
 * they land they're at the rest under the moving body.
 *
 * LOOKAHEAD is in seconds: it's "how far ahead the body will
 * be when this foot lands." For a half-second swing,
 * LOOKAHEAD ≈ 0.5 · body_speed predicts almost exactly where
 * the body will be at landing.
 *
 * Without lookahead, the body would visually outpace its own
 * legs at speed — feet permanently dragging behind the body's
 * actual position. Lookahead keeps the legs centred under the
 * moving body.
 *
 * Same trick is used by real bipedal walking robots (foot
 * placement based on velocity prediction) and by 4-legged
 * animation rigs.
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
     *   1 body frame (rectangle + cross-braces + hip markers + center)
     *   2 femur (hip → knee)
     *   3 tibia (knee → foot)
     *   4 planted foot '*'  (bright accent)
     *   5 swinging foot 'o' (dim, in flight)
     *   6 knee joint 'o'    (bold)
     *   7 reserved
     *   8 PAIR_HUD  (status bar — bright yellow on default bg, A_BOLD)
     *   9 PAIR_HINT (key hint — bright cyan  on default bg, A_BOLD) */
    N_PAIRS  = 7,
    PAIR_HUD = 8,
    PAIR_HINT = 9,

    N_THEMES = 8,
    N_LEGS   = 6,
};

/* Body geometry (px). Body always faces +x in body-local space; the
 * heading rotates the whole frame about (body_x, body_y). */
#define BODY_LEN         80.0f   /* total length along body axis           */
#define BODY_HALF_W      20.0f   /* half-width — legs attach at ±this      */

/* Leg geometry (px). Femur + tibia must be > max reach for IK to solve. */
#define UPPER_LEN        40.0f   /* femur (hip → knee)                     */
#define LOWER_LEN        36.0f   /* tibia (knee → foot)                    */

/* Walking speed (px/s). User-tunable in [MIN, MAX] via w/s. */
#define BODY_SPEED_DEFAULT  40.0f
#define BODY_SPEED_MIN      10.0f
#define BODY_SPEED_MAX     200.0f

/* Gait timing.
 *   PHASE_DURATION : minimum time one tripod stays in air before swap.
 *   STEP_DURATION  : single foot's swing arc duration.
 *   STEP_HEIGHT    : peak Y-lift during arc (pixels, +y = down).
 *   STEP_LOOKAHEAD : seconds-ahead foot lands; multiplied by body_speed
 *                    so faster walking → longer strides automatically. */
#define PHASE_DURATION    0.35f
#define STEP_DURATION     0.28f
#define STEP_HEIGHT       12.0f
#define STEP_LOOKAHEAD    0.18f

/* Heading interpolation rate (rad/s). 2.5 → 90° turn in ~0.6 s. */
#define TURN_RATE         2.5f

/* Direction-glyph line rasteriser step. Must be < CELL_W (8) so the
 * line never skips a column. */
#define DRAW_LEG_STEP_PX  8.0f

/* Time scale — user-controlled simulation speed multiplier.
 * 0.25× to 4×, default 1×. Stepped by ×/÷ TIME_SCALE_STEP via [/]. */
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
/* §3  color — 8 robot themes + spec-fixed HUD pairs                     */
/* ===================================================================== */

/*
 * Per-theme body palette (pairs 1..N_PAIRS). HUD/HINT pairs are theme-
 * independent (CLAUDE.md HUD spec) — they stay readable against any
 * theme.
 *
 * All entries sit in the bright half of the 256-colour space:
 *   - cube colours: ≥ 24 (brightness rule)
 *   - grayscale  : ≥ 240 (the dim-grayscale 232-239 zone is invisible
 *                  under A_DIM and therefore avoided here)
 */
typedef struct {
    const char *name;
    int col[N_PAIRS];   /* pairs 1..7 */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name      body  femur tibia plant step  knee   rsvd   */
    {"Steel",  {245,  67,  75,  46, 214, 231,  244}},
    {"Cobalt", {244,  27,  39,  46, 208, 231,  246}},
    {"Copper", {242, 130, 172,  46, 214, 231,  244}},
    {"Toxin",  {244,  34,  40,  46,  82, 231,  246}},
    {"Ember",  {244, 130, 136,  46, 208, 231,  246}},
    {"Ghost",  {248, 251, 254,  46, 252, 255,  246}},
    {"Neon",   {245,  93, 201,  46, 226, 255,  244}},
    {"Ocean",  {244,  27,  51,  46,  51, 231,  244}},
};

/* theme_apply — re-bind body pairs to the chosen theme.
 * HUD/HINT are NOT re-bound here because they're theme-independent. */
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
        /* 8-color fallback — coarser but directionally correct. */
        init_pair(1, COLOR_WHITE,   -1);
        init_pair(2, COLOR_CYAN,    -1);
        init_pair(3, COLOR_CYAN,    -1);
        init_pair(4, COLOR_GREEN,   -1);
        init_pair(5, COLOR_YELLOW,  -1);
        init_pair(6, COLOR_WHITE,   -1);
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
/* §5  entity — Hexapod                                                   */
/* ===================================================================== */

/* ── §5a  vec helpers + per-leg static tables ─────────────────────── */

typedef struct { float x, y; } Vec2;

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline Vec2 vec2_lerp(Vec2 a, Vec2 b, float t)
{
    return (Vec2){ a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
}

static inline float smoothstep(float t)
{
    t = clampf(t, 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* rotate2d — rotate v by `angle` radians (screen space: +y = down). */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/*
 * Coordinate convention in body-local space:
 *   body faces +x  (walk direction)
 *   +y = down on screen → "left" side of body is −y, "right" side +y
 *
 * Leg index map:
 *   0 left-front   1 right-front
 *   2 left-mid     3 right-mid
 *   4 left-rear    5 right-rear
 */

/* Hip attachment offsets from body center (body-local). */
static const float HIP_LOCAL_X[N_LEGS] = {
    BODY_LEN * 0.40f,    /* 0 left-front  */
    BODY_LEN * 0.40f,    /* 1 right-front */
    0.0f,                /* 2 left-mid    */
    0.0f,                /* 3 right-mid   */
   -BODY_LEN * 0.40f,    /* 4 left-rear   */
   -BODY_LEN * 0.40f,    /* 5 right-rear  */
};
static const float HIP_LOCAL_Y[N_LEGS] = {
   -BODY_HALF_W,   /* 0 left  side */
    BODY_HALF_W,   /* 1 right side */
   -BODY_HALF_W,
    BODY_HALF_W,
   -BODY_HALF_W,
    BODY_HALF_W,
};

/*
 * Rest foot offsets from hip (body-local).
 *   REST_FORWARD — along body axis; front legs reach forward, rear back.
 *   REST_SIDE    — lateral; left legs to −y, right legs to +y.
 *
 * Reach check: max |offset| ≈ √(32² + 50²) ≈ 60 px < UPPER+LOWER = 76 px.
 */
static const float REST_FORWARD[N_LEGS] = {
     32.0f, 32.0f,    /* front */
      0.0f,  0.0f,    /* mid   */
    -32.0f,-32.0f,    /* rear  */
};
static const float REST_SIDE[N_LEGS] = {
   -45.0f, 45.0f,     /* front: closer to body */
   -50.0f, 50.0f,     /* mid:   spread further */
   -45.0f, 45.0f,     /* rear */
};

/*
 * Tripod groups — two interlocked support triangles.
 *   A = {left-front, right-mid, left-rear}
 *   B = {right-front, left-mid, right-rear}
 * When A swings, B forms a stable triangle (and vice versa).
 */
static const int TRIPOD_A[3] = { 0, 3, 4 };
static const int TRIPOD_B[3] = { 1, 2, 5 };

/* Hexapod — full robot state. */
typedef struct {
    /* body kinematics */
    float body_x, body_y;       /* center in pixel space                 */
    float body_speed;           /* px/s along heading                    */
    float heading;              /* current direction (rad, 0 = +x)       */
    float target_heading;       /* desired direction (steered toward)    */

    /* per-leg state */
    Vec2  foot_pos[N_LEGS];     /* current IK target (planted or arcing) */
    Vec2  foot_old[N_LEGS];     /* foot at step start — lerp anchor      */
    Vec2  step_target[N_LEGS];  /* where this foot is stepping to        */
    bool  stepping[N_LEGS];     /* true while leg is in flight           */
    float step_t[N_LEGS];       /* swing progress in [0, 1]              */

    /* per-leg derived (recomputed each frame) */
    Vec2  hip[N_LEGS];          /* world hip from rotated body pose      */
    Vec2  knee[N_LEGS];         /* IK-computed mid-joint                 */

    /* gait state machine */
    int   gait_phase;           /* 0 = group A swings, 1 = group B       */
    float phase_timer;          /* time elapsed in current half-cycle    */

    /* ui state */
    bool  paused;
    int   theme_idx;
} Hexapod;

/* ── §5b  solve_ik ─────────────────────────────────────────────────── */

/*
 * solve_ik — 2-joint IK via law of cosines.
 *
 * The hip→target distance is clamped just inside the reachable annulus
 * |U − L| < d < U + L so acos never receives a value outside [−1, 1]
 * (which would yield NaN and silently break the chain).
 *
 * Left vs right sign convention: even-indexed legs are LEFT (−y side),
 * so their knees break toward −y (knee_angle = base − ah). Right legs
 * mirror this with base + ah. This is opposite to ik_spider.c because
 * here the legs extend perpendicular to the walk axis.
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = sqrtf(dx * dx + dy * dy);

    dist = clampf(dist,
                  fabsf(UPPER_LEN - LOWER_LEN) + 1.0f,
                  UPPER_LEN + LOWER_LEN - 1.0f);

    float base  = atan2f(dy, dx);
    float cos_h = (dist * dist + UPPER_LEN * UPPER_LEN
                                - LOWER_LEN * LOWER_LEN)
                  / (2.0f * dist * UPPER_LEN);
    float ah    = acosf(clampf(cos_h, -1.0f, 1.0f));
    float ka    = is_left ? (base - ah) : (base + ah);

    knee_out->x = hip.x + UPPER_LEN * cosf(ka);
    knee_out->y = hip.y + UPPER_LEN * sinf(ka);
}

/* ── §5c  hip_world_pos ────────────────────────────────────────────── */

/* hip_world_pos — leg i's hip in world coords, accounting for heading. */
static Vec2 hip_world_pos(const Hexapod *h, int i)
{
    Vec2 local   = { HIP_LOCAL_X[i], HIP_LOCAL_Y[i] };
    Vec2 rotated = rotate2d(local, h->heading);
    return (Vec2){ h->body_x + rotated.x, h->body_y + rotated.y };
}

/* ── §5d  rest_target ──────────────────────────────────────────────── */

/*
 * rest_target — ideal foot landing spot for leg i.
 * The (body_speed · STEP_LOOKAHEAD) forward bias plants the foot ahead
 * of the hip so the leg is already reaching forward when it lands —
 * faster walking → longer strides automatically.
 */
static Vec2 rest_target(const Hexapod *h, int i)
{
    Vec2 hip     = hip_world_pos(h, i);
    Vec2 offset  = { REST_FORWARD[i] + h->body_speed * STEP_LOOKAHEAD,
                     REST_SIDE[i] };
    Vec2 rotated = rotate2d(offset, h->heading);
    return (Vec2){ hip.x + rotated.x, hip.y + rotated.y };
}

/* ── §5e  gait_tick ────────────────────────────────────────────────── */

/* swing_foot — advance one leg's swing by dt; place foot.
 * Pure helper called by gait_tick for each leg in the stepping group. */
static void swing_foot(Hexapod *h, int i, float dt)
{
    h->step_t[i] += dt / STEP_DURATION;
    if (h->step_t[i] >= 1.0f) {
        h->step_t[i]   = 1.0f;
        h->foot_pos[i] = h->step_target[i];
        h->stepping[i] = false;
        return;
    }

    float ease  = smoothstep(h->step_t[i]);
    Vec2  hz    = vec2_lerp(h->foot_old[i], h->step_target[i], ease);
    /* Parabolic Y-arc — raw step_t (not eased) so the peak is centred. */
    float arc_y = -STEP_HEIGHT * sinf((float)M_PI * h->step_t[i]);
    h->foot_pos[i] = (Vec2){ hz.x, hz.y + arc_y };
}

/* launch_tripod — start a new step for every leg in the given group. */
static void launch_tripod(Hexapod *h, const int group[3])
{
    for (int k = 0; k < 3; k++) {
        int i = group[k];
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = rest_target(h, i);
        h->stepping[i]    = true;
        h->step_t[i]      = 0.0f;
    }
}

/*
 * gait_tick — one tick of the tripod state machine. Advances any
 * swinging legs, then checks whether the phase can swap.
 *
 * The phase swaps only when BOTH conditions hold:
 *   - phase_timer ≥ PHASE_DURATION  (rhythmic minimum)
 *   - every swinging leg has landed (no leg in mid-air)
 *
 * Without the second guard, the next tripod could launch while the
 * current one is still mid-arc → the body would briefly have only a
 * 2-foot or 1-foot support pattern → it would visibly tip.
 */
static void gait_tick(Hexapod *h, float dt)
{
    const int *swing_grp = (h->gait_phase == 0) ? TRIPOD_A : TRIPOD_B;

    /* Advance any in-flight legs. */
    for (int k = 0; k < 3; k++) {
        int i = swing_grp[k];
        if (h->stepping[i]) swing_foot(h, i, dt);
    }

    h->phase_timer += dt;
    if (h->phase_timer < PHASE_DURATION) return;

    /* Wait until every swinging leg has landed before swapping. */
    for (int k = 0; k < 3; k++)
        if (h->stepping[swing_grp[k]]) return;

    /* Swap: the just-landed tripod plants, the other launches. */
    h->gait_phase  = (h->gait_phase + 1) % 2;
    h->phase_timer = 0.0f;
    launch_tripod(h, (h->gait_phase == 0) ? TRIPOD_A : TRIPOD_B);
}

/* ── §5f  hexapod_tick ─────────────────────────────────────────────── */

/* steer_heading — interpolate heading toward target_heading at TURN_RATE.
 * The diff is wrapped to [−π, π] so a 180° flip takes the short arc. */
static void steer_heading(Hexapod *h, float dt)
{
    float diff = h->target_heading - h->heading;
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    float turn = clampf(diff, -TURN_RATE * dt, TURN_RATE * dt);
    h->heading += turn;
}

/* translate_body — advance position along heading; wrap toroidally.
 * Wrapping in all 4 directions matches the snake/spider convention
 * and makes the world feel infinite at any screen size. */
static void translate_body(Hexapod *h, float dt, int cols, int rows)
{
    h->body_x += h->body_speed * cosf(h->heading) * dt;
    h->body_y += h->body_speed * sinf(h->heading) * dt;

    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (h->body_x <  0.0f) h->body_x += wpx;
    if (h->body_x >= wpx)  h->body_x -= wpx;
    if (h->body_y <  0.0f) h->body_y += hpx;
    if (h->body_y >= hpx)  h->body_y -= hpx;
}

/*
 * stretch_snap — recover after toroidal wrap or sharp turn.
 *
 * After the body wraps, planted feet didn't wrap with it — they're
 * still anchored to the OLD coordinates, now possibly across the
 * screen. The IK solver clamps distance, but a foot that's a full
 * screen away looks wrong. Snap any over-reach foot to its rest
 * target so the gait recovers within one frame.
 */
static void stretch_snap(Hexapod *h)
{
    const float max_reach = UPPER_LEN + LOWER_LEN - 2.0f;
    for (int i = 0; i < N_LEGS; i++) {
        float dx = h->foot_pos[i].x - h->hip[i].x;
        float dy = h->foot_pos[i].y - h->hip[i].y;
        if (sqrtf(dx*dx + dy*dy) <= max_reach) continue;

        h->foot_pos[i]    = rest_target(h, i);
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = h->foot_pos[i];
        h->stepping[i]    = false;
        h->step_t[i]      = 0.0f;
    }
}

/* hexapod_tick — orchestrator. See ALGORITHM IN STEPS for the order. */
static void hexapod_tick(Hexapod *h, float dt, int cols, int rows)
{
    if (h->paused) return;

    steer_heading  (h, dt);
    translate_body (h, dt, cols, rows);

    /* Recompute hips from new body pose before any IK or stretch check. */
    for (int i = 0; i < N_LEGS; i++)
        h->hip[i] = hip_world_pos(h, i);

    stretch_snap(h);
    gait_tick   (h, dt);

    for (int i = 0; i < N_LEGS; i++)
        solve_ik(h->hip[i], h->foot_pos[i], (i % 2 == 0), &h->knee[i]);
}

/* ── §5g  rendering helpers ─────────────────────────────────────────── */

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

    if (deg <  22.5f || deg >= 157.5f) return (chtype)(unsigned char)'-';
    if (deg <  67.5f)                   return (chtype)(unsigned char)'\\';
    if (deg < 112.5f)                   return (chtype)(unsigned char)'|';
    return                              (chtype)(unsigned char)'/';
}

/*
 * draw_leg_line — rasterise a→b with direction glyphs.
 * Steps every DRAW_LEG_STEP_PX, dedups same-cell samples. The dedup
 * cursor is local: each call sweeps independently because consecutive
 * segments do not share intermediate cells in this simulation.
 */
static void draw_leg_line(WINDOW *w,
                          Vec2 a, Vec2 b,
                          int pair, attr_t attr,
                          int cols, int rows)
{
    float dx  = b.x - a.x;
    float dy  = b.y - a.y;
    float len = sqrtf(dx * dx + dy * dy);
    if (len < 0.1f) return;

    chtype glyph  = seg_glyph(dx, dy);
    int    nsteps = (int)ceilf(len / DRAW_LEG_STEP_PX) + 1;
    int    prev_cx = -9999, prev_cy = -9999;

    for (int t = 0; t <= nsteps; t++) {
        float u  = (float)t / (float)nsteps;
        int   cx = px_to_cell_x(a.x + dx * u);
        int   cy = px_to_cell_y(a.y + dy * u);

        if (cx == prev_cx && cy == prev_cy) continue;
        prev_cx = cx; prev_cy = cy;
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        wattron(w, COLOR_PAIR(pair) | attr);
        mvwaddch(w, cy, cx, glyph);
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

/* ── §5h  hexapod_draw ─────────────────────────────────────────────── */

/* draw_legs — pass 1: femur and tibia line glyphs for all 6 legs. */
static void draw_legs(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_leg_line(w, h->hip[i],  h->knee[i],     2, A_NORMAL, cols, rows);
        draw_leg_line(w, h->knee[i], h->foot_pos[i], 3, A_NORMAL, cols, rows);
    }
}

/* draw_feet — pass 2: '*' bold for planted, 'o' dim for swinging. */
static void draw_feet(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        if (h->stepping[i])
            mark_cell(w, h->foot_pos[i], (chtype)(unsigned char)'o',
                      5, A_DIM, cols, rows);
        else
            mark_cell(w, h->foot_pos[i], (chtype)(unsigned char)'*',
                      4, A_BOLD, cols, rows);
    }
}

/* draw_knees — pass 3: bold 'o' marker at each knee joint. */
static void draw_knees(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++)
        mark_cell(w, h->knee[i], (chtype)(unsigned char)'o',
                  6, A_BOLD, cols, rows);
}

/* draw_body_box — pass 4: rectangle edges + cross-braces, rotated. */
static void draw_body_box(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    float bhl = BODY_LEN  * 0.5f;
    float bhw = BODY_HALF_W;

    /* Four corners in body-local then rotated to world. */
    Vec2 tl_l = { -bhl, -bhw }, tr_l = {  bhl, -bhw };
    Vec2 bl_l = { -bhl,  bhw }, br_l = {  bhl,  bhw };
    Vec2 tl   = rotate2d(tl_l, h->heading);
    Vec2 tr   = rotate2d(tr_l, h->heading);
    Vec2 bl   = rotate2d(bl_l, h->heading);
    Vec2 br   = rotate2d(br_l, h->heading);
    tl.x += h->body_x;  tl.y += h->body_y;
    tr.x += h->body_x;  tr.y += h->body_y;
    bl.x += h->body_x;  bl.y += h->body_y;
    br.x += h->body_x;  br.y += h->body_y;

    /* Edges bold, cross-braces dim — gives the body weight visually. */
    draw_leg_line(w, tl, tr, 1, A_BOLD, cols, rows);
    draw_leg_line(w, bl, br, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tl, bl, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tr, br, 1, A_BOLD, cols, rows);
    draw_leg_line(w, tl, br, 1, A_DIM,  cols, rows);
    draw_leg_line(w, tr, bl, 1, A_DIM,  cols, rows);
}

/* draw_hips_and_center — pass 5+6: hip '+' markers and body '@' centre. */
static void draw_hips_and_center(const Hexapod *h, WINDOW *w,
                                 int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++)
        mark_cell(w, h->hip[i], (chtype)(unsigned char)'+',
                  1, A_NORMAL, cols, rows);

    Vec2 center = { h->body_x, h->body_y };
    mark_cell(w, center, (chtype)(unsigned char)'@', 1, A_BOLD, cols, rows);
}

/*
 * hexapod_draw — orchestrator. Painter's order back-to-front so joint
 * markers and the body box always sit on top of leg lines.
 */
static void hexapod_draw(const Hexapod *h, WINDOW *w, int cols, int rows)
{
    draw_legs           (h, w, cols, rows);
    draw_feet           (h, w, cols, rows);
    draw_knees          (h, w, cols, rows);
    draw_body_box       (h, w, cols, rows);
    draw_hips_and_center(h, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — thin wrapper around Hexapod                               */
/* ===================================================================== */

typedef struct { Hexapod hexapod; } Scene;

/*
 * scene_init — place the robot at screen centre with all feet at rest,
 * then immediately launch group A into a step so the gait begins on
 * frame 1. Without that initial launch, both tripods would be planted
 * and gait_tick would have to wait PHASE_DURATION before the first step.
 */
static void scene_init(Scene *sc, int cols, int rows)
{
    memset(sc, 0, sizeof *sc);
    Hexapod *h = &sc->hexapod;

    h->body_speed     = BODY_SPEED_DEFAULT;
    h->body_x         = (float)(cols * CELL_W) * 0.5f;
    h->body_y         = (float)(rows * CELL_H) * 0.5f;
    h->heading        = 0.0f;
    h->target_heading = 0.0f;
    h->gait_phase     = 0;
    h->phase_timer    = 0.0f;
    h->paused         = false;
    h->theme_idx      = 0;

    for (int i = 0; i < N_LEGS; i++)
        h->hip[i] = hip_world_pos(h, i);

    /* Plant all feet at rest (no lookahead — robot is stationary). */
    for (int i = 0; i < N_LEGS; i++) {
        Vec2 hip = h->hip[i];
        h->foot_pos[i]    = (Vec2){ hip.x + REST_FORWARD[i],
                                    hip.y + REST_SIDE[i] };
        h->foot_old[i]    = h->foot_pos[i];
        h->step_target[i] = h->foot_pos[i];
        h->stepping[i]    = false;
        h->step_t[i]      = 0.0f;
        solve_ik(h->hip[i], h->foot_pos[i], (i % 2 == 0), &h->knee[i]);
    }

    /* Launch group A so step animation begins on frame 1. */
    launch_tripod(h, TRIPOD_A);
}

static void scene_tick(Scene *sc, float dt, int cols, int rows)
{
    hexapod_tick(&sc->hexapod, dt, cols, rows);
}

static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows)
{
    hexapod_draw(&sc->hexapod, w, cols, rows);
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
 *   erase → hexapod → status (top right) → key hint (bottom).
 *
 * HUD pairs are spec-fixed (PAIR_HUD = bright yellow, PAIR_HINT = bright
 * cyan, both A_BOLD on default bg) so they read against any animation
 * behind them.
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);

    const Hexapod *h = &sc->hexapod;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " HEXAPOD  dir:%s  spd:%.0f  phase:%s  theme:%s  %.2fx  %.1ffps  %s ",
             heading_arrow(h->heading), h->body_speed,
             h->gait_phase == 0 ? "A" : "B",
             THEMES[h->theme_idx].name,
             time_scale, fps,
             h->paused ? "PAUSED" : "walking");

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
 * Body position is clamped into the new bounds; theme is re-applied
 * because some terminals re-derive COLORS on resize.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Hexapod *h   = &app->scene.hexapod;
    float    wpx = (float)(app->screen.cols * CELL_W);
    float    hpx = (float)(app->screen.rows * CELL_H);
    if (h->body_x < 0.0f || h->body_x >= wpx) h->body_x = wpx * 0.5f;
    if (h->body_y < 0.0f || h->body_y >= hpx) h->body_y = hpx * 0.5f;
    theme_apply(h->theme_idx);
    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch one keypress; return false to quit.
 * Arrow keys set target_heading; the body interpolates toward it at
 * TURN_RATE rad/s in steer_heading().
 */
static bool app_handle_key(App *app, int ch)
{
    Hexapod *h = &app->scene.hexapod;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': h->paused = !h->paused; break;

    case KEY_RIGHT: h->target_heading =  0.0f;                break;
    case KEY_DOWN:  h->target_heading =  (float)M_PI * 0.5f;  break;
    case KEY_LEFT:  h->target_heading =  (float)M_PI;         break;
    case KEY_UP:    h->target_heading = -(float)M_PI * 0.5f;  break;

    case 'w': case 'W':
        h->body_speed *= 1.25f;
        if (h->body_speed > BODY_SPEED_MAX) h->body_speed = BODY_SPEED_MAX;
        break;
    case 's': case 'S':
        h->body_speed /= 1.25f;
        if (h->body_speed < BODY_SPEED_MIN) h->body_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        h->theme_idx = (h->theme_idx + 1) % N_THEMES;
        theme_apply(h->theme_idx);
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
