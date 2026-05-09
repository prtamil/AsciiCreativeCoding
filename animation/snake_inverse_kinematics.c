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
 * References     : Reynolds, "Steering Behaviors for Autonomous Characters,"
 *                    1999 — the canonical wander + arrival + containment.
 *                  Wikipedia: "Inverse kinematics" — analytic form for the
 *                    one-link case used here (atan2 + step).
 *                  Stam, "Real-Time Stable Cloth and Hair," 2002 — same
 *                    trail-following FK applied to per-strand chains.
 *                  Khoshrou, "Snake Robot Locomotion," 2009 — a survey of
 *                    serpentine FK on a moving curve, the body model used.
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
typedef struct {
    const char *name;
    int         body[N_PAIRS];
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

typedef struct { float x, y; } Vec2;

/*
 * Snake — complete simulation state.
 *
 * TRAIL BUFFER — same as snake_forward_kinematics.c:
 *   Circular buffer of head positions; trail_at(k) returns the entry
 *   k ticks back from newest.  Body joints sampled from this by
 *   arc-length (see trail_sample, compute_joints).
 *
 * IK FIELDS:
 *   tgt_time       simulation time accumulator for the wander harmonics
 *   tgt_speed      wander target translation speed (px/s); adjusted by +/-
 *   tgt_dir        current heading of the wander target (radians)
 *   tgt_pos        current pixel position of the wander target
 *   actual_target  smoothly-tracked position (lerp toward tgt_pos)
 *   heading        head's current travel direction (radians); head arrow
 *
 * TARGET TRAIL:
 *   tgt_trail[]    circular buffer of recent actual_target positions
 *   tgt_head       write pointer
 *   tgt_count      valid entries
 *
 * theme_idx — index into THEMES[]; adjusted by t/T keys.
 */
typedef struct {
    /* Trail buffer */
    Vec2  trail[TRAIL_CAP];
    int   trail_head;
    int   trail_count;

    /* Body joints */
    Vec2  joint[N_SEGS + 1];
    Vec2  prev_joint[N_SEGS + 1];

    /* IK / wander target */
    Vec2  actual_target;
    Vec2  tgt_pos;
    float tgt_time;
    float tgt_speed;
    float tgt_dir;
    float heading;
    float move_speed;

    /* Target ghost trail */
    Vec2  tgt_trail[TARGET_TRAIL_CAP];
    int   tgt_head;
    int   tgt_count;

    int   theme_idx;
    bool  paused;
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
 * trail_sample() — interpolated position at arc-length dist from head.
 * Walks the trail from newest entry, accumulating distances, until the
 * cumulative arc equals dist, then linearly interpolates.
 */
static Vec2 trail_sample(const Snake *s, float dist)
{
    float accum = 0.0f;
    Vec2  a     = trail_at(s, 0);

    for (int k = 1; k < s->trail_count; k++) {
        Vec2  b   = trail_at(s, k);
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

    return trail_at(s, s->trail_count - 1);
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
static void bounce_target(Snake *s, float wpx, float hpx)
{
    float m  = EDGE_MARGIN_PX;
    float lo_x = m, hi_x = wpx - m;
    float lo_y = m, hi_y = hpx - m;

    if (s->tgt_pos.x < lo_x) {
        s->tgt_pos.x = lo_x;
        s->tgt_dir   = (float)M_PI - s->tgt_dir;
    } else if (s->tgt_pos.x > hi_x) {
        s->tgt_pos.x = hi_x;
        s->tgt_dir   = (float)M_PI - s->tgt_dir;
    }
    if (s->tgt_pos.y < lo_y) {
        s->tgt_pos.y = lo_y;
        s->tgt_dir   = -s->tgt_dir;
    } else if (s->tgt_pos.y > hi_y) {
        s->tgt_pos.y = hi_y;
        s->tgt_dir   = -s->tgt_dir;
    }
}

/*
 * move_head() — IK goal-seek: head chases a wall-bouncing wander target.
 *
 * STEP 1 — advance wander target via three-harmonic steering.
 *          turn_rate = ΣAᵢ·sin(fᵢ·t + φᵢ); integrate into tgt_dir; step.
 * STEP 2 — bounce target off the bounded region's walls.  Reflecting the
 *          heading AND clamping the position prevents oscillation.
 * STEP 3 — low-pass filter actual_target → tgt_pos at TGT_SMOOTH_RATE.
 *          Smooths the bounce corner so the head's chase looks deliberate.
 * STEP 4 — analytic 1-link IK: heading = atan2(target − head); step at
 *          move_speed, clamped so the head cannot overshoot the target.
 * STEP 5 — hard-clamp head position to the screen box (safety net at the
 *          highest speeds where dt × move_speed could overshoot).
 * STEP 6 — push the new head position into the trail; push actual_target
 *          into the short ghost-trail buffer for rendering.
 */
static void move_head(Snake *s, float dt, int cols, int rows)
{
    float wpx = (float)(cols * CELL_W);
    float hpx = (float)(rows * CELL_H);

    /* Step 1: advance wander target via multi-harmonic steering */
    s->tgt_time += dt;

    float turn = TGT_TURN_AMP1 * sinf(TGT_TURN_FREQ1 * s->tgt_time)
               + TGT_TURN_AMP2 * sinf(TGT_TURN_FREQ2 * s->tgt_time + TGT_TURN_PHASE2)
               + TGT_TURN_AMP3 * sinf(TGT_TURN_FREQ3 * s->tgt_time + TGT_TURN_PHASE3);
    s->tgt_dir += turn * dt;

    s->tgt_pos.x += s->tgt_speed * cosf(s->tgt_dir) * dt;
    s->tgt_pos.y += s->tgt_speed * sinf(s->tgt_dir) * dt;

    /* Step 2: bounce target off bounded region (no toroidal wrap) */
    bounce_target(s, wpx, hpx);

    /* Step 3: smooth actual_target toward tgt_pos */
    float k = dt * TGT_SMOOTH_RATE;
    if (k > 1.0f) k = 1.0f;
    s->actual_target.x += (s->tgt_pos.x - s->actual_target.x) * k;
    s->actual_target.y += (s->tgt_pos.y - s->actual_target.y) * k;

    /* Step 4: steer head toward actual_target */
    float dx   = s->actual_target.x - s->joint[0].x;
    float dy   = s->actual_target.y - s->joint[0].y;
    float dist = sqrtf(dx * dx + dy * dy);

    if (dist > 0.5f) {
        s->heading  = atan2f(dy, dx);
        float step  = s->move_speed * dt;
        if (step > dist) step = dist;   /* don't overshoot */
        s->joint[0].x += (dx / dist) * step;
        s->joint[0].y += (dy / dist) * step;
    }

    /* Step 5: hard-clamp head to screen box (safety net) */
    if (s->joint[0].x < 0.0f) s->joint[0].x = 0.0f;
    if (s->joint[0].x > wpx)  s->joint[0].x = wpx;
    if (s->joint[0].y < 0.0f) s->joint[0].y = 0.0f;
    if (s->joint[0].y > hpx)  s->joint[0].y = hpx;

    /* Step 6: record into trail and target ghost trail */
    trail_push(s, s->joint[0]);
    tgt_push(s, s->actual_target);
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

typedef struct { Snake snake; } Scene;

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

typedef struct { int cols, rows; } Screen;

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

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
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
