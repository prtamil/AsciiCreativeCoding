/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * ik_scorpin.c — six-legged scorpion with curving FK tail
 *
 * DEMO: A scorpion crawls across the terminal under autonomous steering.
 *       Body uses trail-buffer FK; six legs use 2-joint analytical IK
 *       with a per-leg autonomous gait. The signature feature: a
 *       seven-segment tail anchored at the abdomen curves up and over
 *       the body in the iconic scorpion silhouette, with a sin-wave
 *       perturbation traveling along its length so the stinger visibly
 *       sways as the scorpion moves.
 *
 * Study alongside: ik_spider.c       (same body + leg model, no tail)
 *                  hexpod_tripod.c   (rigid-body chassis contrast)
 *
 * Section map:
 *   §1  config        — all tunables in one place
 *   §2  clock         — monotonic clock + sleep (verbatim from framework)
 *   §3  color         — 10 themes + spec HUD/hint pairs
 *   §4  coords        — pixel↔cell aspect-ratio helpers
 *   §5  entity        — Scorpion: body FK + IK legs + curving FK tail
 *       §5a  vec2 + small helpers
 *       §5b  trail (push / at / sample)
 *       §5c  body motion (steer + translate + wrap)
 *       §5d  body joints (trail-buffer FK)
 *       §5e  hip placement
 *       §5f  2-joint analytical IK
 *       §5g  step gait
 *       §5h  tail FK (cumulative angle + sin wave)
 *       §5i  rendering helpers
 *       §5j  render_scorpion (orchestrator)
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
 *   gcc -std=c11 -O2 -Wall -Wextra animation/ik_scorpin.c \
 *       -o ik_scorpin -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Four sub-systems share one body.
 *
 *                 BODY uses path-following (trail-buffer) FK. Every
 *                 tick the head pushes its position into a circular
 *                 trail; each body joint is placed at arc-length
 *                 i*BODY_SEG_LEN backward along the trail. The body
 *                 curves naturally wherever the head went.
 *
 *                 LEGS use 2-joint analytical IK via law of cosines:
 *                     cos(θ_hip) = (d² + U² − L²) / (2·d·U)
 *                 with the hip→target distance d clamped into the
 *                 reachable annulus. Left and right legs use opposite
 *                 signs of θ_hip so knees splay outward.
 *
 *                 GAIT is per-leg autonomous: each leg watches its own
 *                 foot's drift from the ideal step target and triggers
 *                 a swing when ready, gated by an "n_air < N_LEGS/2"
 *                 stability cap so ≥3 feet are always planted.
 *
 *                 TAIL is a stateless FK chain anchored at the abdomen.
 *                 Cumulative angle starts at (heading + π) and adds
 *                 TAIL_BASE_CURL rad per segment so the tail sweeps up
 *                 and over the body in the classic scorpion silhouette.
 *                 A small sin perturbation per segment, with phase
 *                 offset i*TAIL_PHASE_PER_SEG, makes a wave roll along
 *                 the tail — the stinger visibly sways as wave_time
 *                 advances. No iteration, no contact constraints.
 *
 * Data-structure: Scorpion holds the trail buffer + body joints,
 *                 per-leg state, heading + steering, AND a tail joint
 *                 array `tail[N_TAIL_SEGS+1]` plus a `wave_time`
 *                 accumulator that drives the tail oscillation.
 *
 * Rendering     : Painter's order — leg lines → leg joints → body
 *                 fill → body markers → tail (with stinger) → head
 *                 cluster. The tail is drawn AFTER the body so where
 *                 the curve passes over the body, the tail wins. The
 *                 head cluster is drawn last so it always overlays.
 *
 * Performance   : Variable timestep at render rate. Per frame: body
 *                 trail push + 4 trail samples + 6 IK solves + 7 tail
 *                 FK iterations. Microseconds total.
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
 *   ── Procedural motion / steering ─────────────────────────────────
 *   [4] Reynolds, C. (1999), "Steering Behaviors for Autonomous
 *       Characters", GDC — the slew-rate-limited heading-toward-
 *       target pattern in steer_heading.
 *       https://www.red3d.com/cwr/steer/
 *
 *   ── Biological gait + travelling-wave bodies ────────────────────
 *   [5] Wilson, D. M. (1966), "Insect Walking", Annual Review of
 *       Entomology 11, pp. 103-122 — basis for the alternating
 *       support-leg pattern; the "≤ N_LEGS/2 airborne" guard in
 *       §5g update_steps follows this rule.
 *   [6] Cruse, H. (1990), "What mechanisms coordinate leg movement
 *       in walking arthropods?", Trends in Neurosciences 13(1) —
 *       inter-leg coordination rules behind step triggering.
 *   [7] Hirose, S. (1993), "Biologically Inspired Robots: Snake-Like
 *       Locomotors and Manipulators", Oxford — the serpenoid curve
 *       and travelling-wave body articulation; tail_bend_at_segment
 *       implements a discrete version.
 *
 *   ── Differential geometry (2-D Frenet–Serret) ────────────────────
 *   [8] do Carmo, M. P. (1976), "Differential Geometry of Curves
 *       and Surfaces", Prentice-Hall — Ch. 1 §6: the Frenet frame.
 *       lateral_normal_at_spine is the 2-D normal of the spine
 *       polyline (rotate tangent by π/2).
 *
 *   ── Anatomy reference ──────────────────────────────────────────
 *   [9] Polis, G. A., ed. (1990), "The Biology of Scorpions",
 *       Stanford — Ch. 1 morphology + locomotion; basis for the
 *       6-leg + curving cauda silhouette.
 *
 *   ── Timestep / animation ─────────────────────────────────────────
 *  [10] Fiedler, G. (2004), "Fix Your Timestep!", gafferongames.com
 *       — when fixed-step matters; this file uses variable-step
 *       because solve_ik is closed-form and the tail FK is stateless.
 *
 *   ── Rendering / ncurses ─────────────────────────────────────────
 *  [11] Bresenham, J. E. (1965), "Algorithm for computer control of
 *       a digital plotter", IBM Systems Journal 4(1), pp. 25-30 —
 *       the integer line algorithm; §5i draw_chain_line uses the
 *       simpler parametric oversample because float math is already
 *       paid for in the FK pipeline.
 *  [12] Bourke, P. (1997), "Character representation of grayscale
 *       images", paulbourke.net/dataformats/asciiart — design basis
 *       for the high-contrast glyph ramp.
 *  [13] Raymond, E. S., "NCURSES Programming HOWTO" —
 *       tldp.org/HOWTO/NCURSES-Programming-HOWTO; covers init_pair,
 *       use_default_colors, and the diff pipeline §7 relies on.
 *
 *   ── Online quick reference ───────────────────────────────────────
 *  [14] https://en.wikipedia.org/wiki/Inverse_kinematics
 *  [15] https://en.wikipedia.org/wiki/Law_of_cosines
 *  [16] https://en.wikipedia.org/wiki/Frenet%E2%80%93Serret_formulas
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The scorpion is a snake-like body with six pendulum legs and one
 * curving tail-pendulum. Body, legs, and gait are identical to
 * ik_spider.c — the tail is the new piece. The tail is just a chain
 * of stiff segments hinged end-to-end, anchored at the abdomen and
 * with each segment's angle = (previous angle) + (base curl + sin
 * wave). Cumulative angle accumulates the curl, so 7 segments × 0.34
 * rad each = 2.4 rad ≈ 136° total arch — the iconic scorpion shape.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Body : same trail-buffer chain as the spider — curves through space
 *        as the head walks.
 *
 * Legs : six 2-bar linkages, IK-solved, autonomous gait.
 *
 * Tail : one extra chain hinged at the abdomen. Imagine a snake
 *        attached to the back end of a flat lizard, but with each
 *        joint curling the same direction at the same rate — the
 *        whole snake naturally arcs over the lizard's back. Now add
 *        a slow sine wave to the angles and the snake breathes.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Measure dt = wall-clock since last frame; multiply by time_scale.
 *  2. wave_time += dt — single accumulator drives the tail wave.
 *  3. Steer: heading interpolates toward target_heading at TURN_RATE.
 *  4. Translate body, push trail, sample body joints.
 *  5. Compute hips along the body; run gait + IK for legs.
 *  6. Compute tail: starting at body_joint[N_BODY_SEGS] and angle
 *     (heading + π), accumulate angle += TAIL_BASE_CURL +
 *     TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ + i · TAIL_PHASE_PER_SEG)
 *     and place tail[i+1] from the new angle.
 *  7. Render painter's order: legs → body → tail → head.
 *
 * KEY FORMULAS
 * ────────────
 *  Tail FK     : ang_0   = heading + π        (rearward from abdomen)
 *                δᵢ      = TAIL_BASE_CURL
 *                        + TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ
 *                                              + i · TAIL_PHASE_PER_SEG)
 *                ang_i+1 = ang_i + δᵢ
 *                tail_i+1 = tail_i + TAIL_SEG_LEN · (cos ang_i+1, sin ang_i+1)
 *
 *  Total curl  : N_TAIL_SEGS · TAIL_BASE_CURL ≈ 2.4 rad ≈ 136°
 *                stinger ends pointing forward-up (over the body).
 *
 *  All other formulas (trail_sample, hip placement, IK, ideal foot,
 *  step swing) are unchanged from ik_spider.c.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  - Tail base anchor moves with the body. Because body_joint[N] comes
 *    from trail_sample, the tail base curves naturally with the body.
 *    No extra work — the tail "just follows" the abdomen.
 *
 *  - Tail tip can poke off-screen. Body wraps toroidally; the tail
 *    extends from the wrapped abdomen and may extend past edges. The
 *    line-drawer clips out-of-bounds cells silently.
 *
 *  - Wave amplitude vs base curl. TAIL_SWAY_AMP must stay well below
 *    TAIL_BASE_CURL so the tail never reverses direction at any
 *    segment (which would cause the chain to fold back on itself).
 *    0.06 << 0.34 — safe by 5×.
 *
 *  - IK clamp / wrap recovery: same as the spider — over-stretched
 *    feet snap to ideal positions on the next tick.
 *
 *  - Suspend / lid-close: dt clamped to 100 ms in main().
 *
 * HOW TO VERIFY
 * ─────────────
 *  - Default config: scorpion crawls rightward at 45 px/s. The tail
 *    visibly arches up and over the body, ending in a '#' stinger
 *    that points forward-and-up. The stinger sways slowly side-to-side
 *    in a ~6 s cycle.
 *
 *  - Press arrows → body curves through the turn AND the tail follows,
 *    re-arching from the new body heading.
 *
 *  - Press space → tail freezes mid-curve. Un-pause → wave continues
 *    from the same phase (wave_time stops advancing during pause).
 *
 *  - Press `[` for slow time → tail wave slows visibly. Press `]` for
 *    fast — wave speeds up. Smooth at any time scale.
 *
 *  - Cycle themes with `t` → all parts (body / legs / tail / stinger)
 *    update colour together; HUD stays bright yellow.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read ik_spider.c first; the body + legs + per-leg
 *      gait are STRUCTURALLY IDENTICAL. The only NEW lesson here is
 *      §5h tail FK.
 *   2. §5 entity — focus on §5h tail (T1-T4 below). Everything else
 *      (§5b-§5g) is the spider's code, unchanged.
 *   3. §6 scene — thin wrapper.
 *   4. §1-§4 + §7-§8 — infrastructure. Skim if seen.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   tail[i]                   tail joint i (Vec2). 0 = base (at the
 *                             abdomen), N_TAIL_SEGS = stinger.
 *   wave_time                 master clock for the tail oscillation.
 *   TAIL_BASE_CURL            angle ADDED to the cumulative angle at
 *                             every segment — the "default arch."
 *   TAIL_SWAY_AMP             amplitude of the per-segment sin
 *                             perturbation.
 *   TAIL_FREQ                 Hz of the sway.
 *   TAIL_PHASE_PER_SEG        spatial-phase advance per segment;
 *                             makes the wave TRAVEL up the tail.
 *
 * Background you need
 * ───────────────────
 *   - Cumulative-angle FK (fk_tentacle_forest T3 / fk_helloworld T3).
 *     The tail uses this directly.
 *   - Per-leg gait, IK, body curve from ik_spider.c.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Tail-mass dynamics. The tail is purely kinematic — no Verlet,
 *     no spring forces. It just LOOKS like it's swaying.
 *   - Stinger / venom logic. The '#' marker is decorative.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Four tutorials that build the curving scorpion tail on top of
 * the spider chassis.
 *
 *   T1  The constant-curl trick — bake the arch into FK
 *   T2  Adding sway — sin perturbation that travels up the tail
 *   T3  Anchoring the tail to the moving abdomen
 *   T4  Why this isn't a separate state machine
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE CONSTANT-CURL TRICK — BAKE THE ARCH INTO FK
 * ───────────────────────────────────────────────────
 * A real scorpion tail is a chain of 7 segments that ARCHES
 * UP AND OVER the body. The arch isn't a separate posing
 * decision — it's the tail's NATURAL shape.
 *
 * Cumulative-angle FK (fk_tentacle_forest T3) places joint i+1
 * by:
 *
 *     ang_(i+1) = ang_i + δ_i
 *     tail[i+1] = tail[i] + L · (cos ang_(i+1), sin ang_(i+1))
 *
 * If we set δ_i = TAIL_BASE_CURL (constant per segment),
 * after N segments the cumulative angle has advanced by
 * N · TAIL_BASE_CURL — the tail describes a CIRCULAR ARC.
 *
 * Pick the constants so the total arc is the desired arch:
 *
 *     N_TAIL_SEGS = 7
 *     TAIL_BASE_CURL = 0.34 rad     (≈ 19.5° per segment)
 *     total = 7 × 0.34 = 2.38 rad   (≈ 136°)
 *
 * Starting angle: heading + π — pointing OPPOSITE the body's
 * forward direction. Adding 136° rotates the tail to point
 * UP and FORWARD (over the back) — the iconic scorpion shape.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │             ╱  ━╲                                │
 *      │           ╱    ╮ ╲                               │
 *      │          │      ╲ ╲ ← tail arc                   │
 *      │          │       ╲                               │
 *      │       ↗  ●━━━━━●  │                              │
 *      │       head      abdomen ← tail base anchor       │
 *      │                                                  │
 *      │   each tail segment adds 19.5° to the cumulative │
 *      │   angle → 7 segments produce 136° total arc      │
 *      └──────────────────────────────────────────────────┘
 *
 * Same trick is reused in any "arched" creature: deer antler
 * curves, fish anal fins, animal tails. Constant-curl-per-
 * segment is the cheapest expressive arch.
 *
 * T2  ADDING SWAY — SIN PERTURBATION THAT TRAVELS UP THE TAIL
 * ──────────────────────────────────────────────────────────
 * A still tail looks frozen. Adding a small SIN PERTURBATION
 * to each segment's bend angle gives the stinger life:
 *
 *     δ_i = TAIL_BASE_CURL
 *         + TAIL_SWAY_AMP · sin(wave_time · TAIL_FREQ
 *                                + i · TAIL_PHASE_PER_SEG)
 *
 * Three parts:
 *   - TAIL_BASE_CURL       constant arch (T1)
 *   - · sin(...)           time-varying perturbation
 *   - i · ... in the sin   spatial phase shift → wave travels
 *
 * Same shape as fk_tentacle_forest T2: arg = (time term) +
 * (spatial term). The wave RIDES UP the tail at speed
 * TAIL_FREQ / TAIL_PHASE_PER_SEG segments per second.
 *
 * Amplitude tuning: TAIL_SWAY_AMP MUST be well below
 * TAIL_BASE_CURL. If the perturbation overcame the base curl
 * (sway > curl), the tail's local angle would REVERSE at some
 * segments, folding the chain on itself. The tuned ratio
 * 0.06 / 0.34 ≈ 1:5.7 keeps the curl always positive.
 *
 * T3  ANCHORING THE TAIL TO THE MOVING ABDOMEN
 * ────────────────────────────────────────────
 * The tail base must follow the abdomen as the scorpion
 * moves. Trick: anchor at the LAST body joint, which is
 * already trail-sampled to be exactly N_BODY_SEGS · BODY_SEG_LEN
 * behind the head:
 *
 *     tail[0] = body_joint[N_BODY_SEGS]
 *     ang_0 = heading + π                 ← pointing rearward
 *     for i = 0..N_TAIL_SEGS-1:
 *       δ = TAIL_BASE_CURL + sway_perturbation
 *       ang_(i+1) = ang_i + δ
 *       tail[i+1] = tail[i] + L · (cos ang_(i+1), sin ang_(i+1))
 *
 * Two bonuses from anchoring at body_joint[N]:
 *   - As the body curves through turns, the abdomen position
 *     (and therefore the tail base) AUTOMATICALLY tracks the
 *     curved body without any extra computation.
 *   - As the body wraps toroidally, the tail wraps with it
 *     because its base is one of the wrapped body joints.
 *
 * The "starting angle = heading + π" trick: the tail should
 * start pointing OPPOSITE the body's facing. heading is the
 * body's forward direction; heading + π is its rearward
 * direction. From there the cumulative curl rotates upward
 * and over.
 *
 * For the body's curving motion, this means: as heading turns,
 * the tail's STARTING DIRECTION rotates with the body, but
 * the cumulative arch shape stays the same. Visually: the
 * scorpion spins, and the tail spins with it.
 *
 * T4  WHY THIS ISN'T A SEPARATE STATE MACHINE
 * ───────────────────────────────────────────
 * The tail has NO STATE outside wave_time. No locomotion, no
 * gait, no contact. Pure stateless FK derived from
 * (wave_time, body pose).
 *
 * Why? Because the tail doesn't need to do anything except
 * LOOK alive. We don't simulate venom strikes, posing
 * differences, or threat displays. The tail is decoration.
 *
 * Anywhere in this folder where the body has a "decorative"
 * appendage (centipede antennae, scorpion tail, lizard tongue
 * if we had one), use the same pattern:
 *
 *   1. Anchor at the appropriate body joint.
 *   2. Cumulative-angle FK along the appendage.
 *   3. Per-segment δ = (constant) + (sin perturbation).
 *   4. wave_time advances every frame; everything else is
 *      derived.
 *
 * If the appendage SHOULD respond to the environment (avoid
 * walls, swat at bugs), THEN you'd add per-segment IK or a
 * physical sim. For pure visual articulation, stateless FK is
 * always the cheaper, simpler choice.
 *
 * Decision tree for adding parts to a creature:
 *
 *   purely visual articulation (tail, antennae)
 *     → cumulative-angle FK + wave_time
 *
 *   articulation that REACTS to environment
 *     → IK + targeting code
 *
 *   articulation that PHYSICALLY follows
 *     → Verlet + constraints (ragdoll family)
 *
 * The scorpion tail picks the first. Everything in this file's
 * §5h is the dumbest version of "make the chain look alive"
 * that works. That's why it's only ~30 lines added on top of
 * the spider chassis.
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
     *   1..N_PAIRS  body + legs + tail gradient (themed)
     *   8 PAIR_HUD   bright yellow status bar (theme-independent)
     *   9 PAIR_HINT  bright cyan key hint    (theme-independent) */
    N_PAIRS       = 7,
    PAIR_HUD      = 8,
    PAIR_HINT     = 9,

    N_THEMES      = 10,    /* cycled with `t` */

    /* Anatomy.
     *   N_LEGS       = 6 (3 per side, 3 angular pairs at 60° spacing)
     *   N_BODY_SEGS  = 4 → 5 joints, 80 px end-to-end
     *   N_TAIL_SEGS  = 7 → 8 tail joints, ~77 px curving up over body */
    N_LEGS        = 6,
    N_BODY_SEGS   = 4,
    N_TAIL_SEGS   = 7,
    TRAIL_CAP     = 1024,
};

/* Body geometry (px). */
#define BODY_SEG_LEN      20.0f
#define BODY_SPEED        45.0f
#define BODY_SPEED_MIN    10.0f
#define BODY_SPEED_MAX   200.0f
#define TURN_RATE          2.5f

/* Leg geometry (px). */
#define UPPER_LEN         56.0f
#define LOWER_LEN         50.0f
#define HIP_DIST_FACTOR   0.04f

/* Step gait parameters. */
#define STEP_REACH_FACTOR 0.68f
#define STEP_TRIGGER_DIST 28.0f
#define MAX_STRETCH       65.0f
#define STEP_DURATION     0.22f

/*
 * Tail geometry and oscillation parameters.
 *
 *   TAIL_SEG_LEN      = 11 px per segment → 7×11 = 77 px ≈ 10 cells of tail.
 *
 *   TAIL_BASE_CURL    = 0.34 rad/seg. Cumulative: 7 × 0.34 = 2.38 rad ≈ 136°
 *                       total curl. Tail starts pointing rearward (heading
 *                       + π) and after the cumulative curl ends pointing
 *                       forward-and-up (over the body) — the iconic
 *                       scorpion silhouette.
 *
 *   TAIL_SWAY_AMP     = 0.06 rad — small per-segment perturbation. Must
 *                       stay well below TAIL_BASE_CURL so the chain never
 *                       folds back on itself (0.06 << 0.34, safe by 5×).
 *
 *   TAIL_FREQ         = 1.0 rad/s → period 2π/1.0 ≈ 6.3 s — slow elegant
 *                       sway. The stinger swings ±~10° over each cycle.
 *
 *   TAIL_PHASE_PER_SEG= 0.5 rad — wave traveling rate along the tail.
 *                       7 × 0.5 = 3.5 rad ≈ 0.56 of a full cycle, so a
 *                       half-wave is visible rolling from base to tip.
 */
#define N_TAIL_SEGS_F       7.0f
#define TAIL_SEG_LEN       11.0f
#define TAIL_BASE_CURL      0.34f
#define TAIL_SWAY_AMP       0.06f
#define TAIL_FREQ           1.0f
#define TAIL_PHASE_PER_SEG  0.5f

/* Direction-glyph step sizes. DRAW_STEP_PX < CELL_W (8) so the dense
 * stamping never skips a column. */
#define DRAW_STEP_PX      5.0f
#define DRAW_LEG_STEP_PX  8.0f

/* Eye cluster — perpendicular distance from head joint, in pixel space. */
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
 * ideal step direction in body-local space. Three pairs at 60° spacing.
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
 * attaches (0 = head joint, 1 = tail joint).
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
 * Theme — one scorpion colour palette (xterm-256 fg indices).
 *
 * Intent
 *   Each theme rebinds the seven body-rendering pairs (1..7) in one
 *   shot via init_pair (see theme_apply). HUD/HINT pairs are
 *   theme-independent (PAIR_HUD = bright yellow, PAIR_HINT = bright
 *   cyan) so the status bar stays readable against any animation
 *   behind it — CLAUDE.md HUD spec.
 *
 * Slot semantics (col[0..6] map to pairs 1..7)
 *   [0] body gradient — tail end (dimmest)
 *   [1] body gradient — mid
 *   [2] body gradient — head end (main scorpion colour)
 *   [3] upper leg segment (femur)
 *   [4] lower leg segment (tibia)
 *   [5] BRIGHT accent — tail spine + planted-foot '*' + stinger '#'
 *   [6] DIM accent    — swinging-foot '.' (visual cue: foot is in flight)
 *
 *   The tail-to-head gradient ([0]→[2]) lets the eye trace the body
 *   without joint-numbering overlay. The bright/dim split on feet
 *   [5]/[6] encodes ground contact: planted (in-contact) reads
 *   brighter than swinging (in-flight). Ref [12] Bourke.
 *
 * Brightness rule (CLAUDE.md "Theme Palette Brightness")
 *   Every entry sits in the BRIGHT HALF of the 256-colour space:
 *     - cube colours: ≥ 24  (avoid 16-23; invisible under A_DIM)
 *     - grayscale  : ≥ 240 (avoid 232-239; same reason)
 *   Theme character comes from the RELATIVE gradient, not absolute
 *   darkness.
 *
 * References [12] Bourke for the gradient-as-depth pattern;
 *   [13] Raymond §init_pair for the rebind mechanism.
 */
typedef struct {
    const char *name;            /* HUD-displayable theme name        */
    int         col[N_PAIRS];    /* xterm-256 fg index per pair slot; *
                                  * pair p = col[p-1] at apply time   */
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* name        body gradient   legs      foot   ghost */
    {"Desert",  {130, 136, 142,  58,  64,  46, 240}},
    {"Black",   { 52,  88, 124,  52,  88, 196, 240}},
    {"Toxic",   { 24,  28,  34,  28,  34,  82, 240}},
    {"Ocean",   { 24,  25,  27,  33,  39,  51, 240}},
    {"Nova",    { 54,  93, 129,  93, 129, 165, 240}},
    {"Ember",   { 52,  94, 130, 130, 130, 208, 240}},
    {"Aurora",  { 24,  29,  35,  35,  71, 221, 240}},
    {"Ghost",   {240, 244, 248, 244, 248, 254, 246}},
    {"Fire",    { 52,  88, 196,  88, 124, 226, 240}},
    {"Neon",    { 57,  93, 201,  93, 129, 201, 240}},
};

/* theme_apply — re-bind body/leg/tail pairs to chosen theme. HUD/HINT
 * pairs are NEVER touched — they're theme-independent. */
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

static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ===================================================================== */
/* §5  entity — Scorpion                                                  */
/* ===================================================================== */

/* Vec2 — 2-D position vector in pixel space.
 * x increases eastward; y increases downward (terminal convention). */
/*
 * Vec2 — a 2-D point in PIXEL space (sub-cell precision).
 *
 * Intent
 *   Body kinematics, IK, hip placement, gait interpolation, and the
 *   tail FK chain all live in pixel space. Each character cell is
 *   CELL_W × CELL_H sub-pixels (8 × 16), so a walking scorpion reads
 *   as continuous motion instead of jumping cell-to-cell. The
 *   conversion to cell space happens only inside §5i rendering
 *   helpers via px_to_cell_x/y — the project's "one conversion
 *   point" rule.
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
 * References [1] Craig §2 "Spatial descriptions"; [8] do Carmo for
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

/* rotate2d — rotate v by `angle` rad. */
static inline Vec2 rotate2d(Vec2 v, float angle)
{
    float c = cosf(angle), s = sinf(angle);
    return (Vec2){ v.x * c - v.y * s, v.x * s + v.y * c };
}

/* ── Scorpion state ──────────────────────────────────────────────────── */

/*
 * Scorpion — the full creature state. Four interlocked subsystems
 * sharing one record:
 *
 *   1. BODY  — trail-buffer FK. A circular log of past head positions;
 *              body_joint[1..N] sample the trail at arc-lengths
 *              i·BODY_SEG_LEN backward, so the body follows the path
 *              the head carved. No per-segment angle math, no IK.
 *              The same primitive as fk_centipede.c. Refs [1] Craig §3.
 *
 *   2. LEGS  — 2-joint analytical IK (law of cosines) + alternating
 *              gait state machine. Per leg: (hip, foot_pos) drive an
 *              IK solve that places (knee). Stepping flag + step_t
 *              run a smoothstep-eased swing between foot_old and
 *              step_target. The "≤ N_LEGS/2 airborne at once" guard
 *              gives biological tripod-like stability. Refs [1][5][6].
 *
 *   3. TAIL  — stateless cumulative-angle FK chain anchored at the
 *              abdomen joint. Each segment adds a base curl plus a
 *              travelling-wave sin perturbation; total integrated
 *              curl sweeps the stinger over the back (the iconic
 *              scorpion silhouette). Refs [7] Hirose, [9] Polis.
 *
 *   4. STEERING — short-arc heading lerp toward target_heading,
 *                 rate-limited to TURN_RATE rad/s. Ref [4] Reynolds.
 *
 *   Subsystems are UNCOUPLED in time: body integrates from heading;
 *   hips read body_joint; gait reads hip; IK reads (hip, foot_pos);
 *   tail reads body_joint[N]. No feedback loop — scene_tick walks
 *   the dependency chain top-to-bottom once per frame.
 *
 * Why per-leg parallel arrays (SoA), not array-of-structs
 *   N_LEGS is small (≤8) and most loops walk ONE field across all
 *   legs at a time (renderer touches all foot_pos[]; gait touches
 *   all stepping[]). SoA matches that access pattern; static config
 *   tables (LEG_ANGLE, HIP_BODY_T) are parallel arrays too, so the
 *   convention is consistent.
 *
 * Why trail is a CIRCULAR buffer (not a regular array)
 *   Trail samples are pushed at the head and consumed at every
 *   distance along the body. Circular indexing means no O(N) shift
 *   per frame; trail_head walks mod TRAIL_CAP. Same idea as
 *   fk_centipede.c's body chain.
 *
 * Why knee[] is CACHED (recomputed each tick)
 *   solve_ik is cheap but not free. The renderer (legs, joint
 *   markers) reads knee multiple times per frame; computing once
 *   in update_steps lets the rest of the frame skip the math.
 *
 * Why hip_dist is DERIVED (not a compile-time constant)
 *   Scaled to terminal height so legs spread proportionally. Recomputed
 *   on SIGWINCH (in scene_init/app_do_resize).
 *
 * Why wave_time is SEPARATE from any frame-counter
 *   It's the master clock for the tail's CLOSED-FORM FK — same
 *   wave_time ⇒ same tail pose, regardless of how it got there.
 *   Pausing freezes wave_time so the tail resumes in phase. Refs [7].
 *
 * References [1] Craig §3-§4 for FK + IK; [4] Reynolds for steering;
 *   [5][6] Wilson + Cruse for the gait stability rule; [7] Hirose
 *   for the serpenoid wave used in compute_tail; [9] Polis for the
 *   anatomy that motivates the layout.
 */
typedef struct {
    /* ── Body trail-buffer FK ─────────────────────────────────── *
     * Circular log of recent head positions; body_joint[] is the *
     * arc-length-sampled output, rewritten each tick.            */
    Vec2 trail[TRAIL_CAP];
    int  trail_head;                  /* index of newest entry, mod TRAIL_CAP */
    int  trail_count;                 /* valid entries, saturates at TRAIL_CAP*/
    Vec2 body_joint[N_BODY_SEGS + 1]; /* [0]=head joint, [N]=abdomen end      */

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
    Vec2  foot_old   [N_LEGS];        /* foot at step start (lerp anchor)     */
    Vec2  step_target[N_LEGS];        /* landing point this swing aims at     */
    bool  stepping   [N_LEGS];        /* in-flight flag                       */
    float step_t     [N_LEGS];        /* swing progress in [0, 1]             */

    /* ── Tail (closed-form cumulative-angle FK) ──────────────── *
     * Stateless: tail[] is a pure function of (body_joint[N],   *
     * heading, wave_time). wave_time is the ONLY persistent     *
     * state — freezing it freezes the tail in phase.            */
    Vec2  tail[N_TAIL_SEGS + 1];      /* [0]=anchor, [N]=stinger              */
    float wave_time;                  /* phase accumulator (s) for tail sway  */

    /* ── Resize-derived geometry ─────────────────────────────── *
     * Scaled at init+resize so legs spread proportionally to    *
     * terminal size.                                            */
    float hip_dist;                   /* lateral offset of hip from spine (px)*/

    /* ── UI / control state ─────────────────────────────────── *
     * paused gates scene_tick (render still runs);              *
     * theme_idx selects the §3 palette.                         */
    bool  paused;
    int   theme_idx;
} Scorpion;

/* ── §5b  trail (push / at / sample) ────────────────────────────────── */

static void trail_push(Scorpion *sc, Vec2 pos)
{
    sc->trail_head = (sc->trail_head + 1) % TRAIL_CAP;
    sc->trail[sc->trail_head] = pos;
    if (sc->trail_count < TRAIL_CAP) sc->trail_count++;
}

static inline Vec2 trail_at(const Scorpion *sc, int k)
{
    return sc->trail[(sc->trail_head + TRAIL_CAP - k) % TRAIL_CAP];
}

static Vec2 trail_sample(const Scorpion *sc, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(sc, 0);

    for (int k = 1; k < sc->trail_count; k++) {
        Vec2  b   = trail_at(sc, k);
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
    return trail_at(sc, sc->trail_count - 1);
}

/* ── §5c  body motion ──────────────────────────────────────────────── */

/*
 * shortest_signed_angle — wrap a heading difference to [-π, π].
 *
 *   Heading lives on a circle; raw subtraction can give a delta of
 *   e.g. +5π/3 when the true short rotation is -π/3. Wrapping makes
 *   the scorpion always turn the SHORT way round the circle — critical
 *   when target_heading and current heading straddle the π / -π
 *   discontinuity.
 */
static float shortest_signed_angle(float diff)
{
    while (diff >  (float)M_PI) diff -= 2.0f * (float)M_PI;
    while (diff < -(float)M_PI) diff += 2.0f * (float)M_PI;
    return diff;
}

/*
 * clamp_to_max_turn_per_dt — apply rate limit on angular velocity.
 *
 *   This is a SLEW-RATE LIMITER (Reynolds steering): the heading
 *   can change by at most TURN_RATE · dt radians per tick, no
 *   matter how far away target_heading is. Produces a smooth
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
 *   1. shortest_signed_angle    : pick the SHORT direction.
 *   2. clamp_to_max_turn_per_dt : slew-rate limit (max angular vel).
 *   3. heading += clamped step.
 *
 * Reference: Reynolds (1999) "Steering Behaviors for Autonomous
 * Characters", §"Wander / Seek" — the rate-limited heading-
 * correction pattern.
 */
static void steer_heading(Scorpion *sc, float dt)
{
    float diff    = shortest_signed_angle(sc->target_heading - sc->heading);
    float clamped = clamp_to_max_turn_per_dt(diff, dt);
    sc->heading  += clamped;
}

/*
 * integrate_position_along_heading — explicit Euler step.
 *
 *   x(t+dt) = x(t) + v · cos(θ) · dt
 *   y(t+dt) = y(t) + v · sin(θ) · dt
 *
 *   Move_speed is constant magnitude; heading vector (cos, sin) is the
 *   unit direction. Explicit Euler is unconditionally stable here
 *   because there is no acceleration term — body is kinematic.
 */
static void integrate_position_along_heading(Scorpion *sc, float dt)
{
    sc->body_joint[0].x += sc->move_speed * cosf(sc->heading) * dt;
    sc->body_joint[0].y += sc->move_speed * sinf(sc->heading) * dt;
}

/*
 * wrap_position_to_toroidal_world — modular boundary.
 *
 *   The screen is treated as a TORUS — exiting the right edge
 *   re-enters from the left, exiting the bottom re-enters from the
 *   top. Mathematically:  pos ≡ pos  (mod world_size)
 *
 *   Subtract/add one world span instead of `fmodf` so a one-tick
 *   overshoot resolves in O(1) without floating-point modulus
 *   precision loss. Assumes |step| < world_size, which holds for
 *   realistic dt + move_speed combinations.
 */
static void wrap_position_to_toroidal_world(Scorpion *sc, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);
    if (sc->body_joint[0].x <  0.0f) sc->body_joint[0].x += wpx;
    if (sc->body_joint[0].x >= wpx)  sc->body_joint[0].x -= wpx;
    if (sc->body_joint[0].y <  0.0f) sc->body_joint[0].y += hpx;
    if (sc->body_joint[0].y >= hpx)  sc->body_joint[0].y -= hpx;
}

/*
 * translate_body — advance head one tick and record the trail.
 *
 *   1. integrate_position_along_heading  : Euler step in heading dir.
 *   2. wrap_position_to_toroidal_world   : modular boundary.
 *   3. trail_push                        : FK source-of-truth update —
 *                                          body_joints follow the head.
 */
static void translate_body(Scorpion *sc, float dt, int cols, int rows)
{
    integrate_position_along_heading  (sc, dt);
    wrap_position_to_toroidal_world   (sc, cols, rows);
    trail_push                        (sc, sc->body_joint[0]);
}

/* ── §5d  body joints (trail-buffer FK) ──────────────────────────── */

static void compute_body_joints(Scorpion *sc)
{
    for (int i = 1; i <= N_BODY_SEGS; i++)
        sc->body_joint[i] = trail_sample(sc, (float)i * BODY_SEG_LEN);
}

/* ── §5e  hip placement ──────────────────────────────────────────── */

static Vec2 body_local_forward(const Scorpion *sc, int seg_idx)
{
    if (seg_idx + 1 > N_BODY_SEGS)
        return (Vec2){ cosf(sc->heading), sinf(sc->heading) };
    return vec2_norm(vec2_sub(sc->body_joint[seg_idx],
                              sc->body_joint[seg_idx + 1]));
}

/*
 * attachment_point_along_spine — find world-space point at fractional
 * arc-length t along the body-joint chain.
 *
 *   The spine is a polyline through body_joint[0..N_BODY_SEGS].
 *   HIP_BODY_T[i] ∈ [0,1] is leg i's normalised position along it.
 *   Multiply by N_BODY_SEGS to get a real-valued index, split into
 *   integer (segment) + fractional (interpolation weight) parts,
 *   then linearly interpolate between the two bracketing joints.
 *
 *   This is PIECEWISE-LINEAR ARC-LENGTH PARAMETERISATION — the same
 *   trick a tessellator uses to walk along a curve at constant rate.
 *
 *   Outputs the spine point AND the segment index whose tangent the
 *   caller will need (so compute_hips doesn't recompute it).
 */
static Vec2 attachment_point_along_spine(const Scorpion *sc, float t_norm,
                                         int *seg_idx_out)
{
    float t_body  = t_norm * (float)N_BODY_SEGS;
    int   seg_idx = (int)t_body;
    if (seg_idx >= N_BODY_SEGS) seg_idx = N_BODY_SEGS - 1;
    float frac    = t_body - (float)seg_idx;

    *seg_idx_out = seg_idx;
    return vec2_lerp(sc->body_joint[seg_idx],
                     sc->body_joint[seg_idx + 1], frac);
}

/*
 * lateral_normal_at_spine — 90° rotation of the local body tangent.
 *
 *   In 2-D the LEFT-HAND NORMAL of a unit tangent (tx, ty) is
 *   (-ty, tx) — a +π/2 rotation. Multiplying by `side` ∈ {+1,-1}
 *   gives port (left) or starboard (right) lateral direction.
 *
 *   This is the Frenet–Serret normal for a 2-D planar curve; in 3-D
 *   it becomes one of two principal normals and the math gets harder.
 *   In 2-D we just rotate by 90°.
 */
static Vec2 lateral_normal_at_spine(const Scorpion *sc, int seg_idx, float side)
{
    Vec2 fwd       = body_local_forward(sc, seg_idx);
    Vec2 left_norm = (Vec2){ -fwd.y, fwd.x };           /* +π/2 rotation */
    return vec2_scale(left_norm, side);
}

/*
 * compute_hips — place all N_LEGS hips by stepping laterally off the
 * spine.
 *
 *   For each leg i:
 *     1. attachment_point_along_spine : where on the body chain this
 *                                       leg pair hangs from.
 *     2. lateral_normal_at_spine      : ± Frenet normal (port/starboard).
 *     3. hip = attachment + hip_dist · normal.
 *
 *   side = +1 for even i (port), −1 for odd (starboard). hip_dist is
 *   scaled to terminal height so legs spread proportionally.
 */
static void compute_hips(Scorpion *sc)
{
    for (int i = 0; i < N_LEGS; i++) {
        int  seg_idx;
        Vec2 attach = attachment_point_along_spine(sc, HIP_BODY_T[i], &seg_idx);

        float side  = (i % 2 == 0) ? 1.0f : -1.0f;
        Vec2  out   = lateral_normal_at_spine(sc, seg_idx, side);

        sc->hip[i]  = vec2_add(attach, vec2_scale(out, sc->hip_dist));
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
 *   Geometric reference: triangle inequality.
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
 *   (line to target). It is ALWAYS positive (acos ∈ [0, π]); the
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
 *   This is the FK PRIMITIVE (one rigid link from a base) applied
 *   once. After this, the lower leg is implicit — it runs straight
 *   from knee to foot (and is LOWER_LEN long iff dist was inside
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
 * 2-link" derivation; Craig "Introduction to Robotics" Ch. 4
 * "Solvability".
 */
static void solve_ik(Vec2 hip, Vec2 target, bool is_left, Vec2 *knee_out)
{
    float dx   = target.x - hip.x;
    float dy   = target.y - hip.y;
    float dist = clamp_to_reachable_annulus(sqrtf(dx * dx + dy * dy));

    float base        = atan2f(dy, dx);
    float alpha       = law_of_cosines_apex_angle(dist);
    float knee_angle  = is_left ? (base + alpha) : (base - alpha);

    *knee_out = place_knee_at_angle(hip, knee_angle);
}

/* ── §5g  step gait ──────────────────────────────────────────────── */

static Vec2 compute_ideal_foot(const Scorpion *sc, int i)
{
    Vec2  fwd   = (Vec2){ cosf(sc->heading), sinf(sc->heading) };
    Vec2  dir   = rotate2d(fwd, LEG_ANGLE[i]);
    float reach = (UPPER_LEN + LOWER_LEN) * STEP_REACH_FACTOR;
    return vec2_add(sc->hip[i], vec2_scale(dir, reach));
}

static int count_airborne(const Scorpion *sc)
{
    int n = 0;
    for (int i = 0; i < N_LEGS; i++)
        if (sc->stepping[i]) n++;
    return n;
}

static bool snap_overstretched_foot(Scorpion *sc, int i)
{
    if (vec2_dist(sc->foot_pos[i], sc->hip[i]) <= UPPER_LEN + LOWER_LEN - 2.0f)
        return false;

    bool was_airborne   = sc->stepping[i];
    sc->foot_pos[i]     = compute_ideal_foot(sc, i);
    sc->foot_old[i]     = sc->foot_pos[i];
    sc->step_target[i]  = sc->foot_pos[i];
    sc->stepping[i]     = false;
    sc->step_t[i]       = 0.0f;
    return was_airborne;
}

static bool maybe_trigger_step(Scorpion *sc, int i, int n_air)
{
    if (sc->stepping[i] || n_air >= N_LEGS / 2) return false;

    Vec2  ideal   = compute_ideal_foot(sc, i);
    float drift   = vec2_dist(sc->foot_pos[i], ideal);
    float stretch = vec2_dist(sc->foot_pos[i], sc->hip[i]);
    if (drift <= STEP_TRIGGER_DIST && stretch <= MAX_STRETCH) return false;

    sc->stepping[i]    = true;
    sc->step_t[i]      = 0.0f;
    sc->foot_old[i]    = sc->foot_pos[i];
    sc->step_target[i] = ideal;
    return true;
}

static bool advance_swing(Scorpion *sc, int i, float dt)
{
    sc->step_t[i] += dt / STEP_DURATION;
    if (sc->step_t[i] >= 1.0f) {
        sc->step_t[i]   = 1.0f;
        sc->foot_pos[i] = sc->step_target[i];
        sc->stepping[i] = false;
        return true;
    }
    float ease     = smoothstep(sc->step_t[i]);
    sc->foot_pos[i] = vec2_lerp(sc->foot_old[i], sc->step_target[i], ease);
    return false;
}

/*
 * update_steps — one gait-tick orchestrator.
 *
 *   Per-leg dispatch (mutually exclusive): each leg is in exactly one
 *   of three states this tick, with airborne-count book-keeping to
 *   enforce the "≤ N_LEGS / 2 in flight at once" stability rule.
 *
 *     STATE                  ACTION                       n_air delta
 *     ─────                  ──────                       ───────────
 *     OVERSTRETCHED          snap_overstretched_foot      -1 (lands)
 *     IN-FLIGHT              advance_swing                -1 if lands
 *     PLANTED + drift/strain maybe_trigger_step           +1 if takes off
 *
 *   The stability rule (≤ half airborne) is the same constraint
 *   real arthropods follow — see Wilson "Insect Walking" 1966 and
 *   Cruse "What mechanisms coordinate leg movement" 1990.
 *
 *   After dispatch, run analytical 2-joint IK on every leg so the
 *   knee always matches the current foot position. is_left = (i%2==0)
 *   mirrors solve_ik's port/starboard sign convention.
 */
static void update_steps(Scorpion *sc, float dt)
{
    int n_air = count_airborne(sc);

    /* PHASE 1 — advance per-leg state machine */
    for (int i = 0; i < N_LEGS; i++) {
        if      (snap_overstretched_foot(sc, i))    n_air--;
        else if (sc->stepping[i]) {
            if (advance_swing(sc, i, dt))           n_air--;
        }
        else if (maybe_trigger_step(sc, i, n_air))  n_air++;
    }

    /* PHASE 2 — recompute knee positions from updated feet */
    for (int i = 0; i < N_LEGS; i++)
        solve_ik(sc->hip[i], sc->foot_pos[i], (i % 2 == 0), &sc->knee[i]);
}

/* ── §5h  tail FK ────────────────────────────────────────────────── */

/*
 * tail_bend_at_segment — per-segment bend angle δᵢ for the tail FK.
 *
 *     δᵢ = BASE_CURL + SWAY_AMP · sin(ω·t + i·PSEG)
 *
 *   BASE_CURL : constant additive bend (each segment curls a bit
 *               further than its predecessor → cumulative curve).
 *   SWAY_AMP  : amplitude of the sinusoidal travelling-wave sway.
 *   PSEG      : per-segment phase offset; with PSEG > 0 the wave
 *               propagates FROM ROOT TO TIP along the tail (the
 *               serpenoid pattern — see Hirose 1993 §"Wave
 *               propagation in articulated bodies").
 *
 *   Reference: Frisch-Hasslacher-Pomeau showed wave-propagation
 *   patterns can be reproduced by simple per-cell rules; the same
 *   intuition applies here to per-segment angles.
 */
static float tail_bend_at_segment(const Scorpion *sc, int i)
{
    float wave = TAIL_SWAY_AMP * sinf(sc->wave_time * TAIL_FREQ
                                      + (float)i * TAIL_PHASE_PER_SEG);
    return TAIL_BASE_CURL + wave;
}

/*
 * compute_tail — stateless cumulative-angle FK chain for the tail.
 *
 *   1. ANCHOR    tail[0] = body_joint[N_BODY_SEGS]   (abdomen end)
 *   2. SEED      cumulative_angle = heading + π     (point rearward)
 *   3. FOR each segment i ∈ [0, N_TAIL_SEGS):
 *        a. δᵢ = tail_bend_at_segment(i)
 *        b. cumulative_angle += δᵢ                  (chain-of-rotations)
 *        c. tail[i+1] = tail[i] + L · (cos, sin)(cumulative_angle)
 *
 *   Total integrated curl ≈ N_TAIL_SEGS · TAIL_BASE_CURL ≈ 2.4 rad ≈ 136°
 *   sweeps the tail UP and OVER the body, ending with the stinger
 *   pointing forward-and-up — the iconic scorpion silhouette.
 *
 *   STATELESS: closed-form function of (body pose, wave_time). Same
 *   inputs ⇒ same tail; no integrator state crosses frames.
 *
 *   This is the same cumulative-angle FK used in fk_tentacle_forest.c;
 *   here it's anchored to a moving abdomen rather than a fixed root.
 */
static void compute_tail(Scorpion *sc)
{
    sc->tail[0] = sc->body_joint[N_BODY_SEGS];   /* PHASE 1: anchor */

    float cumulative_angle = sc->heading + (float)M_PI;  /* PHASE 2: seed */

    for (int i = 0; i < N_TAIL_SEGS; i++) {              /* PHASE 3: walk */
        cumulative_angle += tail_bend_at_segment(sc, i);

        sc->tail[i + 1].x = sc->tail[i].x + TAIL_SEG_LEN * cosf(cumulative_angle);
        sc->tail[i + 1].y = sc->tail[i].y + TAIL_SEG_LEN * sinf(cumulative_angle);
    }
}

/* ── §5i  rendering helpers ──────────────────────────────────────── */

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
 * draw_chain_line — segmented chain line: alternating direction-glyph
 * and '.' so each segment reads as a chained limb. Used for legs AND
 * for the tail — both are articulated chains in this creature.
 */
static void draw_chain_line(WINDOW *w, Vec2 a, Vec2 b,
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

/* ── §5j  render_scorpion ────────────────────────────────────────── */

static void draw_legs(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        draw_chain_line(w, sc->hip[i],  sc->knee[i],     3, A_BOLD, cols, rows);
        draw_chain_line(w, sc->knee[i], sc->foot_pos[i], 3, A_BOLD, cols, rows);
    }
}

static void draw_leg_joints(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_LEGS; i++) {
        mark_cell(w, sc->knee[i], (chtype)(unsigned char)'o',
                  3, A_BOLD, cols, rows);

        if (sc->stepping[i])
            mark_cell(w, sc->foot_pos[i], (chtype)(unsigned char)'.',
                      7, A_DIM, cols, rows);
        else
            mark_cell(w, sc->foot_pos[i], (chtype)(unsigned char)'*',
                      6, A_BOLD, cols, rows);
    }
}

static void draw_body_lines(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    int prev_cx = -9999, prev_cy = -9999;
    for (int i = N_BODY_SEGS - 1; i >= 0; i--) {
        draw_body_beads(w, sc->body_joint[i + 1], sc->body_joint[i],
                        3, A_BOLD, cols, rows, &prev_cx, &prev_cy);
    }
}

static void draw_body_nodes(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = N_BODY_SEGS; i >= 1; i--) {
        chtype glyph = (i == N_BODY_SEGS)
                     ? (chtype)(unsigned char)'O'   /* abdomen tip */
                     : (chtype)(unsigned char)'o';
        mark_cell(w, sc->body_joint[i], glyph, 3, A_BOLD, cols, rows);
    }
}

/*
 * draw_tail — render the curving tail with stinger.
 * Same chained-line style as the legs (direction-glyph + '.'), drawn
 * in the brighter accent pair (6) so it visually pops over the body.
 * Stinger '#' at the tip — chunky barb, A_BOLD for emphasis.
 */
static void draw_tail(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < N_TAIL_SEGS; i++) {
        draw_chain_line(w, sc->tail[i], sc->tail[i + 1],
                        6, A_BOLD, cols, rows);
    }
    /* Stinger — distinct glyph at the tail tip. */
    mark_cell(w, sc->tail[N_TAIL_SEGS], (chtype)(unsigned char)'#',
              6, A_BOLD, cols, rows);
}

/*
 * draw_head — eye cluster ': arrow :' at the head joint, perpendicular
 * to heading. Same as ik_spider — a small visual signature so the
 * scorpion's facing direction is unambiguous.
 */
static void draw_head(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    Vec2  head   = sc->body_joint[0];
    float perp_x = -sinf(sc->heading) * EYE_OFFSET_PX;
    float perp_y =  cosf(sc->heading) * EYE_OFFSET_PX;

    Vec2 eye_l = { head.x - perp_x, head.y - perp_y };
    Vec2 eye_r = { head.x + perp_x, head.y + perp_y };

    mark_cell(w, eye_l, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);
    mark_cell(w, eye_r, (chtype)(unsigned char)':', 3, A_DIM, cols, rows);
    mark_cell(w, head, head_glyph(sc->heading), 3, A_BOLD, cols, rows);
}

/*
 * render_scorpion — orchestrator. Painter's order back-to-front:
 *   legs (lines)  →  leg joints  →  body fill  →  body nodes
 *   tail (curves over body)  →  head cluster
 *
 * Tail is drawn AFTER body nodes so where it crosses over the body
 * (which it does at the tail's apex), the tail wins. Head is drawn
 * last so it always overlays everything else.
 */
static void render_scorpion(const Scorpion *sc, WINDOW *w, int cols, int rows)
{
    draw_legs       (sc, w, cols, rows);
    draw_leg_joints (sc, w, cols, rows);
    draw_body_lines (sc, w, cols, rows);
    draw_body_nodes (sc, w, cols, rows);
    draw_tail       (sc, w, cols, rows);
    draw_head       (sc, w, cols, rows);
}

/* ===================================================================== */
/* §6  scene — thin wrapper around Scorpion                              */
/* ===================================================================== */

/*
 * Scene — composition root for §6.
 *
 * Intent
 *   In this demo the simulation IS one scorpion. We keep a Scene
 *   wrapper anyway so the framework loop (scene_init / scene_tick /
 *   scene_draw) reads identically to every other file in the repo.
 *   If a future variant adds prey, obstacles, pheromone trails, or
 *   a swarm of scorpions, they slot in here as siblings of `scorpion`
 *   without changing the loop.
 *
 * Simulation vs Rendering locality
 *   The Scorpion struct itself already separates Body / Legs / Tail /
 *   Steering / UI. Within Scene there is no further split needed yet
 *   — one field, one concern. When adding a new member, place it
 *   as follows:
 *
 *     ── Simulation state ──   things scene_tick READS+WRITES
 *                              (prey[], obstacle_grid, pheromones, …)
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
    Scorpion scorpion;         /* the world: body + 6 legs + tail   */

    /* (no render-only state yet — theme_idx + paused live inside
     *  Scorpion because they're user-toggled in the same keymap)  */
} Scene;

/*
 * scene_init — place scorpion at screen centre with a pre-populated
 * trail (so the body is fully extended on frame 1) and feet planted
 * at their ideal rest positions.
 */
static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    Scorpion *sc = &s->scorpion;

    sc->move_speed     = BODY_SPEED;
    sc->heading        = 0.0f;
    sc->target_heading = 0.0f;
    sc->hip_dist       = (float)(rows * CELL_H) * HIP_DIST_FACTOR;
    sc->wave_time      = 0.0f;
    sc->paused         = false;
    sc->theme_idx      = 0;

    sc->body_joint[0].x = (float)(cols * CELL_W) * 0.50f;
    sc->body_joint[0].y = (float)(rows * CELL_H) * 0.50f;

    /* Pre-fill trail backward so the body is fully extended on frame 1. */
    float bx = cosf(sc->heading + (float)M_PI);
    float by = sinf(sc->heading + (float)M_PI);
    for (int k = 0; k < TRAIL_CAP; k++) {
        sc->trail[k].x = sc->body_joint[0].x + (float)k * bx;
        sc->trail[k].y = sc->body_joint[0].y + (float)k * by;
    }
    sc->trail_head  = 0;
    sc->trail_count = TRAIL_CAP;

    compute_body_joints(sc);
    compute_hips(sc);
    compute_tail(sc);

    for (int i = 0; i < N_LEGS; i++) {
        sc->foot_pos[i]    = compute_ideal_foot(sc, i);
        sc->foot_old[i]    = sc->foot_pos[i];
        sc->step_target[i] = sc->foot_pos[i];
        sc->stepping[i]    = false;
        sc->step_t[i]      = 0.0f;
        solve_ik(sc->hip[i], sc->foot_pos[i], (i % 2 == 0), &sc->knee[i]);
    }
}

/*
 * scene_tick — one variable-timestep simulation step. dt is wall-clock
 * delta scaled by the caller's time_scale.
 *
 * Order: wave_time, steer, translate, body joints, hips, gait+IK, tail.
 * Tail comes LAST so it sees the new abdomen position from this tick's
 * body update.
 */
static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    Scorpion *sc = &s->scorpion;
    if (sc->paused) return;

    sc->wave_time += dt;

    steer_heading      (sc, dt);
    translate_body     (sc, dt, cols, rows);
    compute_body_joints(sc);
    compute_hips       (sc);
    update_steps       (sc, dt);
    compute_tail       (sc);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    render_scorpion(&s->scorpion, w, cols, rows);
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
 * References [13] Raymond, NCURSES Programming HOWTO.
 */
typedef struct {
    int cols;   /* terminal width  in CHARACTER CELLS */
    int rows;   /* terminal height in CHARACTER CELLS */
} Screen;

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

static void screen_draw(Screen *s, const Scene *sn,
                        double fps, float time_scale)
{
    erase();
    scene_draw(sn, stdscr, s->cols, s->rows);

    const Scorpion *sc = &sn->scorpion;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " IK-SCORPION  dir:%s  spd:%.0f  theme:%s  %.2fx  %.1ffps  %s ",
             heading_arrow(sc->heading), sc->move_speed,
             THEMES[sc->theme_idx].name,
             time_scale, fps,
             sc->paused ? "PAUSED" : "crawling");

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
 *   See [13] Raymond §"Signal handling".
 *
 * Things that DO NOT live here
 *   - Wall-clock timestamps / fps counters — main() locals; no
 *     other code path needs them.
 *   - Scorpion tuning values (move_speed, theme_idx) — user-state
 *     in §5 Scorpion.
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

static void cleanup(void) { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scorpion *sc = &app->scene.scorpion;
    float wpx = (float)(app->screen.cols * CELL_W);
    float hpx = (float)(app->screen.rows * CELL_H);
    if (sc->body_joint[0].x >= wpx) sc->body_joint[0].x = wpx - 1.0f;
    if (sc->body_joint[0].y >= hpx) sc->body_joint[0].y = hpx - 1.0f;
    sc->hip_dist = hpx * HIP_DIST_FACTOR;
    theme_apply(sc->theme_idx);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scorpion *sc = &app->scene.scorpion;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ': sc->paused = !sc->paused; break;

    case KEY_RIGHT: sc->target_heading =  0.0f;                break;
    case KEY_DOWN:  sc->target_heading =  (float)M_PI * 0.5f;  break;
    case KEY_LEFT:  sc->target_heading =  (float)M_PI;         break;
    case KEY_UP:    sc->target_heading = -(float)M_PI * 0.5f;  break;

    case 'w': case 'W':
        sc->move_speed *= 1.25f;
        if (sc->move_speed > BODY_SPEED_MAX) sc->move_speed = BODY_SPEED_MAX;
        break;
    case 's': case 'S':
        sc->move_speed /= 1.25f;
        if (sc->move_speed < BODY_SPEED_MIN) sc->move_speed = BODY_SPEED_MIN;
        break;

    case 't': case 'T':
        sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
        theme_apply(sc->theme_idx);
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

        /* ⑦ frame cap */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(target_ns - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
