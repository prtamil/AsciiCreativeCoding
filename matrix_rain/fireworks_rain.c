/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fireworks_rain.c — fireworks with matrix-rain arc trails
 *
 * DEMO: Rockets rise from the bottom of the screen, decelerate to apex,
 *       and explode into 72 sparks per burst. Every spark grows a
 *       16-character matrix-rain trail that follows the exact arc
 *       it traces under gravity — head bright, body dimming, glyphs
 *       rerolling every frame for the classic Matrix shimmer.
 *
 *           ASCII sketch of one spark in flight:
 *
 *               head (white, bold)
 *                ↓
 *                A   ← cache[0]   (newest, BOLD, TRAIL_HOT band)
 *                 q  ← cache[1]   BOLD
 *                  W ← cache[2]   BOLD
 *                   z              normal (TRAIL_WARM band)
 *                    7             normal
 *                     R            DIM   (TRAIL_COOL band)
 *                      e           DIM
 *                       %          DIM   (oldest)
 *
 *       The trail is the LAST 16 head positions kept in a per-spark
 *       ring buffer. Each frame: slide the buffer down, push the new
 *       head into slot [0], integrate physics, reroll most cache
 *       glyphs. The same trajectory therefore re-renders with
 *       constantly-changing characters — the Matrix-rain shimmer.
 *
 * Study alongside:
 *   matrix_rain/matrix_rain.c       — same shimmer-cache trick on
 *                                     plain vertical rain (read first
 *                                     if the cache reroll is unfamiliar).
 *   particle_systems/fireworks.c    — the rocket-and-burst skeleton
 *                                     without trails.
 *   matrix_rain/pulsar_rain.c       — the same shimmer-cache trick
 *                                     on rotating beams.
 *   matrix_rain/matrix_snowflake.c  — rain accumulating into snow.
 *
 * Section map:
 *   §1  config       — every tunable in one place, grouped by concept
 *   §2  clock        — monotonic timer + sleep
 *   §3  color        — HUD/HINT plus 5 themed spark palettes
 *   §4  spark        — Vec2, Spark, burst, tick, draw
 *   §5  rocket       — IDLE → RISING → EXPLODED state machine
 *   §6  show         — fixed rocket pool + tick + draw + pause
 *   §7  screen       — ncurses init / present / HUD
 *   §8  app          — signals, resize, variable-dt main loop
 *
 * Keys:
 *   q / Q / ESC      quit
 *   space / p        pause / resume
 *   r                reset the show
 *   ]   [            speed up / slow down
 *   = / +            more rockets
 *   -                fewer rockets
 *   t                cycle theme (vivid / matrix / fire / ice / plasma)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/fireworks_rain.c \
 *       -o fireworks_rain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Three independent ideas stacked on top of each other.
 *
 *                 (A) ROCKET   — a 3-state machine. IDLE counts down a
 *                                fuse (in seconds). RISING climbs under
 *                                gravity until vy ≥ 0 (apex) or y < 2
 *                                (top edge). EXPLODED ticks all sparks
 *                                until none are alive, then returns to
 *                                IDLE with a fresh random fuse.
 *
 *                 (B) SPARK    — one explosion particle with a head
 *                                position + velocity, a 16-slot trail
 *                                history of past head positions, and
 *                                a parallel cache of random ASCII
 *                                glyphs. Each frame: slide history,
 *                                integrate physics, shimmer cache,
 *                                decay life.
 *
 *                 (C) SHIMMER  — for each cache slot, with probability
 *                                1 − 1/SHIMMER_KEEP_ONE_IN, reroll the
 *                                glyph. KEEP_ONE_IN = 4 → 75 % rerolled
 *                                each frame, the classic Matrix-rain
 *                                shimmer rate.
 *
 * Data-structure: Show ⊃ Rocket[16] ⊃ Spark[72]. All inline; no heap
 *                 allocation post-init. Each Spark owns: a Vec2 head,
 *                 a Vec2 velocity, a Vec2[TRAIL_LEN] history, a
 *                 char[TRAIL_LEN] glyph cache, plus life/decay/color
 *                 /active. The trail history is laid out newest-at-[0]
 *                 oldest-at-[N−1] so painter's-order drawing reads as
 *                 a normal for-loop.
 *
 * Rendering     : Painter's order is load-bearing. For each spark,
 *                 paint oldest trail slot first, newer over older,
 *                 then a single white head on top. Drawing dim
 *                 entries first lets the brighter head overwrite them
 *                 at cells where multiple positions map to the same
 *                 column/row (common near burst centres where sparks
 *                 are tightly packed). trail_attr() maps slot index
 *                 → BOLD / NORMAL / DIM bands; if life < FADING_THRESH
 *                 every slot goes DIM (death fade).
 *
 * Performance   : O(R · P · T) per tick where R = active rockets ≤ 16,
 *                 P = sparks per burst = 72, T = trail slots = 16 —
 *                 about 18k vec2 ops worst case. At 60 fps that's
 *                 ~1.1 M ops/sec, microseconds. Trail history uses a
 *                 literal shift-down (O(T) per push) instead of a ring
 *                 buffer with head index because T = 16 makes the
 *                 constant factor invisible and the shift is easier
 *                 to read.
 *
 * References    :
 *   Reeves, "Particle Systems — A Technique for Modeling a Class of
 *     Fuzzy Objects", SIGGRAPH 1983 — foundational paper introducing
 *     particle systems for explosions, fire, smoke, and clouds.
 *   Wikipedia, "Particle system" — broader context on the technique.
 *     https://en.wikipedia.org/wiki/Particle_system
 *   "The Matrix" (1999, Wachowski) — visual inspiration for the
 *     rerolling-glyph rain effect, here decorating each spark's
 *     trajectory rather than falling vertically.
 *   This project, matrix_rain/matrix_rain.c — the same shimmer-cache
 *     pattern applied to plain vertical rain. Read that file first
 *     if the cache reroll trick is unfamiliar.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A normal firework spark traces a parabolic arc, but its visible body
 * is just one bright point. Here we keep the last 16 positions per
 * spark in a buffer, redraw all of them, and randomly reshuffle the
 * GLYPHS at those positions every frame. The result: the spark is no
 * longer a dot but a 16-character snake of jumping ASCII that traces
 * its own trajectory, brightest at the head, dim at the tail.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a sparkler in the dark, photographed with long exposure: you
 * see the arc as a glowing curve. Now overlay The Matrix's flickering
 * character rain on that curve, head bright, body fading, glyphs
 * rerolling every frame. That's the visual contract. The rocket-and-
 * burst skeleton is standard fireworks (rise, gravity, apex, explode);
 * the novelty is the per-spark trail buffer + shimmer cache.
 *
 * ROCKET STATE MACHINE
 * ────────────────────
 *
 *      ┌──────┐  fuse_sec ≤ 0    ┌────────┐  vy ≥ 0  OR    ┌──────────┐
 *      │ IDLE │ ────────────────►│ RISING │────────────────│ EXPLODED │
 *      └──────┘                  └────────┘  y < 2 row     └──────────┘
 *         ▲                                                      │
 *         │  all sparks dead → uniform(FUSE_MIN..MIN+VAR) sec    │
 *         └──────────────────────────────────────────────────────┘
 *
 *      IDLE     fuse_sec -= dt; transition when fuse hits 0.
 *      RISING   y += vy·dt; vy += GRAVITY·dt; explode at apex.
 *      EXPLODED tick all sparks; return to IDLE when none alive.
 *
 * TRAIL BUFFER LAYOUT
 * ───────────────────
 *
 *      live head (white, BOLD)
 *      trail[0]   ← newest    ┐
 *      trail[1]               │ HOT band  (BOLD, indices 0..TRAIL_HOT_END)
 *      trail[2]               ┘
 *      trail[3]               ┐
 *      trail[4]               │ WARM band (NORMAL,
 *      trail[5]               │           indices ..TRAIL_WARM_END-1)
 *      trail[6]               │
 *      trail[7]               ┘
 *      trail[8]               ┐
 *      trail[9]               │
 *      ...                    │ COOL band (DIM, the rest)
 *      trail[15]  ← oldest    ┘
 *
 *      If life < FADING_LIFE_THRESHOLD: every band → DIM (death fade).
 *
 * ALGORITHM IN STEPS  (per frame, per spark — read with spark_tick)
 * ──────────────────
 *  1. ADVANCE TRAIL
 *       slide trail[i] = trail[i-1] for i = N-1 down to 1
 *       trail[0] = head            (newest snapshot)
 *  2. INTEGRATE PHYSICS
 *       g          = SPARK_GRAVITY · uniform(1−J, 1+J)
 *       head      += vel · dt
 *       vel.y     += g · dt
 *  3. SHIMMER CACHE
 *       for each k in 0..N-1:
 *         if rand() % SHIMMER_KEEP_ONE_IN ≠ 0:
 *           cache[k] = rand_glyph()
 *  4. LIFE DECAY
 *       life -= decay_rate · dt
 *       if life ≤ 0: active = false
 *
 * KEY FORMULAS
 * ────────────
 *   angle_i  = 2π·i/N + jitter             even fan-out around centre
 *   (vx,vy)  = (cos a · s, sin a · s)      polar→cartesian initial vel
 *   head    += vel · dt                    cells/sec × seconds = cells
 *   vy      += g · dt                      cells/sec² × seconds = cells/sec
 *   trail[0] = head                        most recent position; index = age
 *   shimmer  = 1 − 1/KEEP_ONE_IN           fraction of cache rerolled/frame
 *   life(t)  = life(0) − decay_rate · t    linear decay
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • The trail-shift loop must run in REVERSE (i = N-1 down to 1) or
 *    you smear the newest position across the whole buffer.
 *  • Drawing newest-first instead of oldest-first looks visually wrong:
 *    trails get overwritten by their own dim tails. Order is load-bearing.
 *  • trail_fill caps at TRAIL_LEN — drawing trail_fill-1 down to 0 avoids
 *    rendering uninitialised slots in the first 16 frames of a spark's life.
 *  • At apex detection, vy ≥ 0 only fires once because vy grows
 *    monotonically; the y < 2 clause covers the rare too-fast launch.
 *  • Resize calls show_init which clears all rockets — bursts mid-flight
 *    are lost. By design.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press '-' until rkt:1. The single rocket clearly shows 72 trailing
 *    snakes per burst, all curving downward under gravity.
 *  • The head glyph stays white regardless of theme.
 *  • Press 't' through themes; spark hues change but the head stays
 *    white and the HUD stays bright yellow on the default-bg row.
 *  • Press space to pause: rockets and sparks freeze in place; resume
 *    and motion picks up exactly where it left off.
 *  • Press r to reset: all rockets respawn on the bottom edge with
 *    their original staggered fuses.
 *  • Stopwatch test: at default speeds, time from launch to explosion
 *    is ~1 sec for a slow rocket, ~1.5 sec for a fast one. Spark
 *    lifetimes are 0.3–1.3 sec. Verify with a real timer if curious.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read matrix_rain.c first if the shimmer cache is
 *      new (its T3-T4). The NEW LESSONS here are: physics-driven
 *      arcs (gravity), trail history buffer (per-spark ring), and
 *      a 3-state rocket lifecycle.
 *   2. §4 spark — THE HEART. spark_tick + spark_burst + spark_draw.
 *      Read AFTER tutorials T1-T5.
 *   3. §5 rocket — IDLE / RISING / EXPLODED state machine (T5).
 *   4. §6 show — orchestrator: pool of rockets + per-frame tick.
 *   5. §1-§3, §7-§8 — config / colour / screen / app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   spark.head           current Vec2 position (newest, brightest).
 *   spark.vel            Vec2 velocity (cells/sec).
 *   spark.trail[i]       past head position i frames ago.
 *                        i = 0 is newest, i = TRAIL_LEN-1 oldest.
 *   spark.cache[i]       glyph at trail position i.
 *   spark.life           remaining lifetime in seconds; spark
 *                        deactivates when life ≤ 0.
 *   trail_fill           how many trail slots are populated;
 *                        guard against rendering uninit slots.
 *   FADING_LIFE_THRESHOLD
 *                        below this remaining life, every band
 *                        falls to DIM (death fade).
 *   rocket.state         IDLE | RISING | EXPLODED.
 *   GRAVITY              per-rocket downward acceleration.
 *   SPARK_GRAVITY        per-spark downward acceleration (with
 *                        small jitter for variety).
 *
 * Background you need
 * ───────────────────
 *   - matrix_rain T3-T4 (shimmer cache + 6-band brightness ramp).
 *   - Newton's laws: pos += vel · dt; vel += g · dt.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Air drag / wind. We use plain gravity, no aerodynamic
 *     forces. The arcs are exact parabolas.
 *   - Particle-particle collision. Sparks pass through each other.
 *   - Sound or smoke effects.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build matrix-trailed fireworks from first
 * principles.
 *
 *   T1  Physics-driven motion vs scripted rotation
 *   T2  Trail HISTORY buffer — past positions, not future
 *   T3  Polar burst — N sparks fanned around a circle
 *   T4  Death fade — life threshold collapses bands to DIM
 *   T5  Rocket lifecycle — IDLE / RISING / EXPLODED
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  PHYSICS-DRIVEN MOTION VS SCRIPTED ROTATION
 * ──────────────────────────────────────────────
 * The earlier matrix_rain variants drive motion by SCRIPT:
 *
 *   matrix_rain    head_y += speed · dt          (constant speed)
 *   pulsar_rain    angle  += omega · dt          (constant ω)
 *   sun_rain       r_off  += speed · dt          (constant outward)
 *
 * fireworks_rain drives motion by PHYSICS — Newtonian
 * integration with gravity:
 *
 *   spark.head += spark.vel · dt
 *   spark.vel.y += GRAVITY · dt
 *
 * The vertical velocity DECREASES as gravity pulls down (positive
 * y is screen-down). The spark slows, stops, reverses. The
 * resulting trajectory is a PARABOLIC ARC — exactly the path
 * a real firework spark traces.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │       . . .                                      │
 *      │     .       .   apex (vy ≈ 0)                    │
 *      │    .         .                                   │
 *      │   .           .                                  │
 *      │  .             .                                 │
 *      │ .               .                                │
 *      │.                 .                               │
 *      │                                                  │
 *      └──────────────────────────────────────────────────┘
 *
 * No scripting of the apex. No "now turn around at the top."
 * The integrator handles ascent, apex, and descent uniformly.
 *
 * Each spark gets a SLIGHTLY JITTERED gravity (1 ± J) so 72
 * sparks from the same burst follow 72 SLIGHTLY DIFFERENT
 * arcs. Without the jitter the burst looks symmetrical and
 * mechanical; with it, organic.
 *
 * T2  TRAIL HISTORY BUFFER — PAST POSITIONS, NOT FUTURE
 * ─────────────────────────────────────────────────────
 * matrix_rain's trail is computed BACKWARD from the head's
 * current position by reading old glyphs at row = head_y - dist.
 * That works because matrix_rain streams move STRAIGHT DOWN —
 * the past trail position is just (col, row - dist).
 *
 * fireworks_rain's spark CURVES — its past positions don't lie
 * on a straight line. The trail at frame k must record where
 * the spark WAS k frames ago, not derive it from the current
 * head.
 *
 * Solution: an explicit HISTORY BUFFER per spark:
 *
 *     trail[0] = position now
 *     trail[1] = position 1 frame ago
 *     trail[2] = position 2 frames ago
 *     ...
 *     trail[TRAIL_LEN-1] = position TRAIL_LEN-1 frames ago
 *
 * Each frame:
 *
 *     1. SLIDE the buffer:  trail[i] = trail[i-1] for i descending
 *     2. STORE the new head: trail[0] = head_after_step
 *
 * (The slide MUST be in REVERSE order — i = TRAIL_LEN-1 down to
 * 1 — or you smear the newest entry across the whole buffer.
 * That's a classic "off-by-direction" bug.)
 *
 * The result: rendering the trail is just walking trail[0] to
 * trail[TRAIL_LEN-1] and painting each at its recorded position
 * with the appropriate band colour. The arc renders correctly
 * because every position was actually computed by physics.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │              X    ← trail[0] = current head       │
 *      │             /                                    │
 *      │            q     ← trail[1]                      │
 *      │             ╲                                    │
 *      │              z   ← trail[2]                      │
 *      │              │                                    │
 *      │              R   ← trail[3]                      │
 *      │              │                                    │
 *      │              %   ← trail[4..]  (oldest = dimmest) │
 *      └──────────────────────────────────────────────────┘
 *
 * This is the same structure as a Verlet old_pos (ragdoll T2),
 * extended to TRAIL_LEN history slots instead of just 1.
 *
 * T3  POLAR BURST — N SPARKS FANNED AROUND A CIRCLE
 * ─────────────────────────────────────────────────
 * When a rocket reaches its apex, it EXPLODES — emits 72 sparks
 * radially. How to assign each spark its initial velocity?
 *
 *     for i in 0 .. N_SPARKS-1:
 *       angle_i = 2π · i / N_SPARKS + jitter
 *       speed_i = uniform(SPEED_MIN, SPEED_MAX)
 *       spark[i].vel = (cos angle_i · speed_i, sin angle_i · speed_i)
 *
 * Same evenly-spaced-angles trick as pulsar_rain T5 / sun_rain
 * (fixed-angle rays). Each spark fans out in a different
 * direction; gravity then bends each parabola downward.
 *
 * The "+ jitter" on each angle (small uniform noise) breaks the
 * perfect symmetry — without it the explosion looks like a
 * mechanical 72-pointed star. With it, a natural-looking burst.
 *
 * Each spark's speed is also randomised slightly so some sparks
 * fly farther (lasting longer in the air) than others, giving
 * the burst a sense of depth.
 *
 * T4  DEATH FADE — LIFE THRESHOLD COLLAPSES BANDS TO DIM
 * ──────────────────────────────────────────────────────
 * Each spark has a `life` counter that decreases over time.
 * When life reaches 0, the spark dies. But sparks shouldn't
 * just POP off the screen — that looks abrupt.
 *
 * Death fade: when life drops below FADING_LIFE_THRESHOLD,
 * EVERY trail band collapses to DIM (regardless of its
 * normal HOT/WARM/COOL band):
 *
 *     trail_attr(i, life):
 *       if life < FADING_LIFE_THRESHOLD: return DIM
 *       else:                            return normal band(i)
 *
 * Visually the spark gradually loses its brightness uniformly
 * across the trail before disappearing. Like an ember cooling.
 *
 * The threshold is tuned so the fade lasts roughly 0.3 sec —
 * long enough to register as "fading away," short enough not
 * to leave dim ghosts on screen.
 *
 * Compare with matrix_rain's "respawn" model: there, streams
 * either FALL OFF the bottom or DEACTIVATE at the end. They
 * never fade in place because they always exit the screen.
 * fireworks sparks die in mid-air, so they need an explicit
 * fade-out.
 *
 * T5  ROCKET LIFECYCLE — IDLE / RISING / EXPLODED
 * ───────────────────────────────────────────────
 * A rocket is a 3-state machine:
 *
 *      ┌──────┐  fuse_sec ≤ 0    ┌────────┐  vy ≥ 0  OR    ┌──────────┐
 *      │ IDLE │ ────────────────►│ RISING │────────────────│ EXPLODED │
 *      └──────┘                  └────────┘  y < 2 row     └──────────┘
 *         ▲                                                      │
 *         │  all sparks dead → uniform(FUSE_MIN..MAX) sec        │
 *         └──────────────────────────────────────────────────────┘
 *
 *   IDLE:     waiting on the ground. Fuse counts down.
 *             Transition: fuse hits 0 → RISING.
 *
 *   RISING:   physics-driven climb. Initial vy is negative
 *             (upward); GRAVITY pulls it back. Transition
 *             to EXPLODED when vy ≥ 0 (apex reached) or
 *             y < 2 (top edge — too-fast rockets).
 *
 *   EXPLODED: 72 sparks tick under their own physics + trail.
 *             Transition: every spark inactive → IDLE with a
 *             fresh random fuse.
 *
 * Each rocket runs INDEPENDENTLY. The pool has 16 rockets, all
 * sharing the same logic. With FUSE_MIN/MAX = 1/3 sec, the
 * pool produces roughly one rocket-burst every 0.5 sec — a
 * continuous show.
 *
 * The state-machine pattern is identical to matrix_snowflake's
 * FALL ↔ FLASH (T5 there). Both files have a finite-state loop:
 * an "active" state that runs physics, a transition condition,
 * and a "transition" state (FLASH / EXPLODED) that ends and
 * resets back to "active." Same structural template.
 *
 * Decision tree for adding state machines:
 *
 *   simulation has a clear LIFECYCLE (warm-up → active →
 *     fade → reset)?                                     → state machine
 *
 *   stateless / continuous (always renderable)?          → no state needed,
 *                                                          just integrate
 *
 *   state has BRANCHES (e.g. spark sometimes splits)?    → tree-state machine
 *                                                          or hierarchical FSM
 *
 * Most of matrix_rain/ uses simple linear state (active /
 * inactive); only matrix_snowflake and fireworks_rain need
 * proper state machines because their lifecycle has distinct
 * phases requiring different code paths.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#ifndef M_PI
#  define M_PI 3.14159265358979323846
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

/* ── §1.1 frame rate ──────────────────────────────────────────────── */
enum {
    TARGET_FPS = 60,
};

/* ── §1.2 rocket pool ──────────────────────────────────────────────── */
enum {
    ROCKETS_MIN     =  1,
    ROCKETS_DEFAULT =  5,
    ROCKETS_MAX     = 16,
    MAX_ROCKETS     = ROCKETS_MAX,
};

/* Initial fuse stagger — rocket i starts at i × STAGGER seconds after
 * launch. Without this they all fire at t = 0 and bunch up at apex. */
#define INITIAL_FUSE_STAGGER_SEC  0.13f

/* ── §1.3 rocket physics (cells/sec, cells/sec²) ──────────────────── */
/*
 * Launch velocity range. Negative = upward in screen-row coordinates
 * (which increase downward). Slow rocket explodes ~1 sec after launch
 * about 5 rows up; fast rocket ~1.5 sec, ~30 rows up.
 */
#define ROCKET_LAUNCH_VY_MIN_CPS  -18.0f   /* slow rocket             */
#define ROCKET_LAUNCH_VY_MAX_CPS  -48.0f   /* fast rocket             */

/*
 * ROCKET_GRAVITY_CPS2 — downward acceleration on rockets. Higher
 * values → rockets explode lower on screen. 30 cells/sec² puts
 * explosions throughout the upper half.
 */
#define ROCKET_GRAVITY_CPS2       30.0f

/*
 * ROCKET_TOP_EXPLODE_ROW — safety net: explode early if a rocket
 * hits the top edge (rare for too-fast launches).
 */
#define ROCKET_TOP_EXPLODE_ROW    2.0f

/* Random fuse range after a rocket finishes (seconds). 0.5 .. 2.5 s
 * before the same rocket relaunches. */
#define FUSE_MIN_SEC      0.5f
#define FUSE_VAR_SEC      2.0f

/* ── §1.4 spark physics (cells/sec, cells/sec²) ───────────────────── */
enum { PARTICLES_PER_BURST = 72 };

/*
 * Spark initial-velocity magnitude range. Each spark fires off the
 * burst origin at a polar angle with this speed.
 */
#define SPARK_SPEED_MIN_CPS    12.0f
#define SPARK_SPEED_MAX_CPS    40.0f

/*
 * SPARK_GRAVITY_CPS2 — downward acceleration on every spark.
 * SPARK_GRAVITY_JITTER — per-spark variance (±20 % at default 0.20)
 *   so sparks don't all trace the SAME parabola. Without jitter
 *   the whole burst falls in lock-step.
 */
#define SPARK_GRAVITY_CPS2     32.0f
#define SPARK_GRAVITY_JITTER    0.20f

/*
 * Spark life — initial value uniform([MIN, MIN+VAR]) on [0, 1] scale.
 * Decay rate uniform([MIN_RPS, MIN+VAR_RPS]) per second. Lifespan
 * = life / decay → 0.33 s at fastest decay, 1.33 s at slowest.
 */
#define SPARK_LIFE_MIN          0.6f
#define SPARK_LIFE_VAR          0.4f
#define SPARK_DECAY_MIN_RPS     0.75f
#define SPARK_DECAY_VAR_RPS     1.05f

/*
 * BURST_ANGLE_JITTER — radians of random offset added to evenly-
 * spaced burst angles. Without this 72 sparks form a perfect ring;
 * with it the cloud reads as a sphere, not a circle.
 */
#define BURST_ANGLE_JITTER      0.30f

/* ── §1.5 trail + shimmer ─────────────────────────────────────────── */
enum {
    /* TRAIL_LEN — historic positions kept per spark.
     * At 60 fps a trail of 16 spans ~267 ms of arc history. */
    TRAIL_LEN       = 16,

    /* Brightness band boundaries (used by trail_attr). */
    TRAIL_HOT_END   = 2,                /* [0..2]      → BOLD            */
    TRAIL_WARM_END  = TRAIL_LEN / 2,    /* [3..N/2-1]  → NORMAL          */
                                        /* [N/2..N-1]  → DIM             */

    /* Per-frame, each cache slot has a 1-in-KEEP_ONE_IN chance of
     * surviving; the rest reroll. KEEP_ONE_IN = 4 → reroll fraction
     * = 0.75, the classic 75 % Matrix-rain shimmer. */
    SHIMMER_KEEP_ONE_IN = 4,
};

/* ── §1.6 fade threshold ──────────────────────────────────────────── */
/* When life drops below this, every trail slot is forced DIM —
 * the spark visibly fades out before disappearing. */
#define FADING_LIFE_THRESHOLD  0.25f

/* ── §1.7 speed scale (rain global multiplier, [/] keys) ─────────── */
#define SPEED_SCALE_DEFAULT    1.0f
#define SPEED_SCALE_MIN        0.25f
#define SPEED_SCALE_MAX        4.0f
#define SPEED_SCALE_STEP       1.25f

/* ── §1.8 ncurses pair IDs ────────────────────────────────────────── */
enum {
    /* 1..7 — spark colours, theme-controlled */
    CP_RED          = 1,
    CP_ORANGE,
    CP_YELLOW,
    CP_GREEN,
    CP_CYAN,
    CP_BLUE,
    CP_MAGENTA,

    /* 8 — trail head, always white */
    CP_TRAIL_HEAD,

    /* 9..10 — HUD spec, theme-independent */
    PAIR_HUD,
    PAIR_HINT,
};

#define N_SPARK_COLORS  7

/* ── §1.9 dt cap + timing primitives ─────────────────────────────── */
#define DT_CAP_SEC    0.10f
#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define HUD_BUF_LEN   96

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
 * Color pairs.
 *
 *   1..7  CP_RED..CP_MAGENTA  spark colour SLOTS, remapped per theme.
 *                             The names refer to the default vivid
 *                             theme; under e.g. the matrix theme
 *                             CP_RED is a dark green. Spark code
 *                             never relies on the literal hue.
 *   8     CP_TRAIL_HEAD       always white (theme-independent).
 *   9     PAIR_HUD            yellow on default bg (CLAUDE.md HUD).
 *   10    PAIR_HINT           cyan on default bg (CLAUDE.md HINT).
 */

/*
 * Theme — a 7-colour palette for the spark slots. Each theme's
 * 8-colour fallback covers terminals without 256-colour support.
 *
 * Every entry is in the bright half of the 256-colour cube
 * (CLAUDE.md brightness rule: ≥ 24 in cube; 24-29 only as the lowest
 * ramp tier). Values 16-23 are deliberately excluded.
 *
 * Themes:
 *   vivid  — multi-hue (red/orange/yellow/green/cyan/blue/magenta)
 *   matrix — green ramp; trails look like Matrix rain arcs
 *   fire   — reds/oranges/yellows; every burst is a flame arc
 *   ice    — blues/cyans; cold, crystalline trails
 *   plasma — purples/magentas; electric neon arcs
 */
typedef struct {
    const char *name;
    int         colors  [N_SPARK_COLORS];
    int         fallback[N_SPARK_COLORS];
} Theme;

static const Theme k_themes[] = {
    { "vivid",
      { 196, 208, 226,  46,  51,  33, 201 },
      { COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA } },
    { "matrix",
      {  28,  34,  40,  46,  82, 118, 154 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN } },
    { "fire",
      { 196, 160, 202, 208, 214, 220,  88 },
      { COLOR_RED, COLOR_RED, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED } },
    { "ice",
      {  24,  33,  39,  45,  51,  87, 153 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE } },
    { "plasma",
      {  53,  57,  93, 129, 165, 201, 207 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply — re-bind spark pairs 1..7 plus trail-head pair 8 for
 * the chosen theme. PAIR_HUD and PAIR_HINT are NEVER touched here —
 * they carry semantic meaning that must not change with theme.
 */
static void theme_apply(int idx)
{
    const Theme *t = &k_themes[idx];
    for (int i = 0; i < N_SPARK_COLORS; i++) {
        int fg = g_has_256 ? t->colors[i] : t->fallback[i];
        init_pair(i + 1, fg, COLOR_BLACK);
    }
    init_pair(CP_TRAIL_HEAD, g_has_256 ? 255 : COLOR_WHITE, COLOR_BLACK);
}

/*
 * hud_pairs_init — bind PAIR_HUD and PAIR_HINT once at startup. Both
 * use the default terminal background (-1) so the HUD row sits on
 * whatever background the user actually has, never a forced black box.
 */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

static int color_rand(void) { return 1 + rand() % N_SPARK_COLORS; }

/* ===================================================================== */
/* §4  spark — the heart of the matrix-rain effect                        */
/* ===================================================================== */

/* ── §4.1 ASCII glyph pool + tiny utilities ─────────────────────────── */

/* The shimmer pool — same letters/digits/punctuation as matrix_rain.c. */
static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01   (void) { return (float)rand() / (float)RAND_MAX; }

/* ── §4.2 Vec2 + Spark types ────────────────────────────────────────── */

/* Vec2 — cell-space 2-D position used by head, velocity, and trail. */
typedef struct { float x, y; } Vec2;

/*
 * Spark — one explosion particle with a matrix-rain arc trail.
 *
 *   head          live position in cell-space floats.
 *   vel           velocity in cells per second.
 *
 *   trail[]       last TRAIL_LEN head positions. Newest at [0],
 *                 oldest at [TRAIL_LEN-1]. Slid down by
 *                 spark_advance_trail every frame.
 *   trail_fill    how many slots are populated (ramps 0 → TRAIL_LEN
 *                 over the spark's first TRAIL_LEN frames).
 *
 *   cache[]       parallel array of random ASCII glyphs. Each frame
 *                 ~75 % of slots are rerolled — the shimmer effect.
 *
 *   life          1.0 (fresh) → 0.0 (dead). Dimensionless.
 *   decay_rps     per-second life drop. Varied per spark so a burst
 *                 fades gradually rather than collapsing in unison.
 *   color         hue assigned at burst time (theme-dependent pair).
 *   active        false once life ≤ 0.
 */
typedef struct {
    Vec2  head;
    Vec2  vel;

    Vec2  trail[TRAIL_LEN];
    int   trail_fill;

    char  cache[TRAIL_LEN];

    float life;
    float decay_rps;
    int   color;
    bool  active;
} Spark;

/* ── §4.3 spark_burst_spawn — factory for a full explosion ──────────── */

/*
 * Spawn `count` sparks at `origin`. Angles are evenly spaced around
 * 2π with BURST_ANGLE_JITTER added so the ring isn't perfectly
 * regular. Speed and life and decay are uniform-jittered so the
 * burst fades out gradually rather than collapsing all at once.
 */
static void spark_burst_spawn(Spark *pool, int count, Vec2 origin)
{
    for (int i = 0; i < count; i++) {
        float angle = ((float)i / (float)count) * 2.0f * (float)M_PI
                      + urand01() * BURST_ANGLE_JITTER;
        float speed = SPARK_SPEED_MIN_CPS
                    + urand01() * (SPARK_SPEED_MAX_CPS - SPARK_SPEED_MIN_CPS);

        pool[i].head        = origin;
        pool[i].vel.x       = cosf(angle) * speed;
        pool[i].vel.y       = sinf(angle) * speed;
        pool[i].trail_fill  = 0;
        pool[i].life        = SPARK_LIFE_MIN  + urand01() * SPARK_LIFE_VAR;
        pool[i].decay_rps   = SPARK_DECAY_MIN_RPS
                            + urand01() * SPARK_DECAY_VAR_RPS;
        pool[i].color       = color_rand();
        pool[i].active      = true;

        for (int k = 0; k < TRAIL_LEN; k++)
            pool[i].cache[k] = rand_glyph();
    }
}

/* ── §4.4 per-frame helpers — one named function per ALGORITHM step ── */

/*
 * Step 1 — push current head into trail[0], slide older entries down.
 *
 * The shift loop must run in REVERSE (i = N-1 down to 1). Running it
 * forward would copy the same value across the whole buffer because
 * trail[i] would be overwritten before being read into trail[i+1].
 */
static void spark_advance_trail(Spark *p)
{
    for (int i = TRAIL_LEN - 1; i > 0; i--)
        p->trail[i] = p->trail[i - 1];
    p->trail[0] = p->head;
    if (p->trail_fill < TRAIL_LEN) p->trail_fill++;
}

/*
 * Step 2 — explicit Euler integration with per-spark gravity variance.
 *
 * Each spark's effective gravity is SPARK_GRAVITY_CPS2 scaled by
 * uniform(1−J, 1+J), so sparks fan out instead of all tracing the
 * same parabola. Velocities are in cells/sec; multiplying by dt
 * gives the cell-space displacement directly — no magic scale factor.
 */
static void spark_integrate(Spark *p, float dt)
{
    float jitter = 1.0f - SPARK_GRAVITY_JITTER + urand01() * 2.0f * SPARK_GRAVITY_JITTER;
    float g      = SPARK_GRAVITY_CPS2 * jitter;
    p->head.x += p->vel.x * dt;
    p->head.y += p->vel.y * dt;
    p->vel.y  += g * dt;
}

/*
 * Step 3 — for each cache slot, with probability 1 − 1/KEEP_ONE_IN
 * reroll the glyph. KEEP_ONE_IN = 4 → 75 % rerolled per frame —
 * the classic Matrix-rain shimmer.
 */
static void spark_shimmer(Spark *p)
{
    for (int k = 0; k < TRAIL_LEN; k++)
        if (rand() % SHIMMER_KEEP_ONE_IN != 0)
            p->cache[k] = rand_glyph();
}

/* ── §4.5 spark_tick — orchestrator: one of each helper, in order ───── */

/*
 * One simulation step. Reads as the recipe in MENTAL MODEL → ALGORITHM
 * IN STEPS, line for line: advance trail, integrate, shimmer, decay.
 * Drop the spark from the live pool when life reaches zero.
 */
static void spark_tick(Spark *p, float dt)
{
    if (!p->active) return;

    spark_advance_trail(p);                        /* 1. slide history */
    spark_integrate    (p, dt);                    /* 2. physics       */
    spark_shimmer      (p);                        /* 3. reroll cache  */

    p->life -= p->decay_rps * dt;                  /* 4. life decay    */
    if (p->life <= 0.0f) p->active = false;
}

/* ── §4.6 trail_attr — band the brightness gradient ─────────────────── */

/*
 * Map a trail slot index to its ncurses attribute.
 *
 *   i in [0..TRAIL_HOT_END]    → BOLD   (hot, near-head)
 *   i in [..TRAIL_WARM_END-1]  → NORMAL (mid-fade)
 *   else (deep tail)           → DIM
 *
 *   life < FADING_LIFE_THRESHOLD overrides everything to DIM,
 *   producing a clean death fade as the spark dies.
 */
static attr_t trail_attr(int i, int cp, bool fading)
{
    if (fading)              return COLOR_PAIR(cp) | A_DIM;
    if (i <= TRAIL_HOT_END)  return COLOR_PAIR(cp) | A_BOLD;
    if (i <  TRAIL_WARM_END) return COLOR_PAIR(cp);
    return COLOR_PAIR(cp) | A_DIM;
}

/* ── §4.7 spark_draw — paint trail (oldest first) then head on top ──── */

/*
 * Painter's-algorithm render. Drawing dim entries first lets the
 * brighter head and near-head slots overwrite them at cells where
 * multiple positions map to the same column/row — common near burst
 * centres where many sparks are tightly packed.
 */
static void spark_draw(const Spark *p, int cols, int rows)
{
    if (!p->active) return;

    bool fading = (p->life < FADING_LIFE_THRESHOLD);

    /* Trail — oldest first so newer slots paint on top. */
    for (int i = p->trail_fill - 1; i >= 0; i--) {
        int x = (int)roundf(p->trail[i].x);
        int y = (int)roundf(p->trail[i].y);
        if (x < 0 || x >= cols || y < 0 || y >= rows) continue;

        attr_t attr = trail_attr(i, p->color, fading);
        attron(attr);
        mvaddch(y, x, (chtype)(unsigned char)p->cache[i]);
        attroff(attr);
    }

    /* Live head — drawn last, always wins overlap. */
    int hx = (int)roundf(p->head.x);
    int hy = (int)roundf(p->head.y);
    if (hx >= 0 && hx < cols && hy >= 0 && hy < rows) {
        attr_t attr = fading ? (COLOR_PAIR(p->color)      | A_DIM)
                             : (COLOR_PAIR(CP_TRAIL_HEAD) | A_BOLD);
        attron(attr);
        mvaddch(hy, hx, (chtype)(unsigned char)p->cache[0]);
        attroff(attr);
    }
}

/* ===================================================================== */
/* §5  rocket — IDLE → RISING → EXPLODED state machine                    */
/* ===================================================================== */

/* ── §5.1 RocketState enum + Rocket type ─────────────────────────── */

typedef enum {
    RS_IDLE     = 0,    /* counting down a fuse, then launches */
    RS_RISING   = 1,    /* climbing under gravity until apex   */
    RS_EXPLODED = 2,    /* burst alive; sparks ticking         */
} RocketState;

/*
 * Rocket — one ascending streak plus its inline pool of sparks.
 *
 *   x, y         current cell-space position (y in cells, fractional).
 *   vy           vertical velocity (cells/sec, negative = climbing).
 *   color        rocket-body hue while RISING (not used after burst —
 *                each spark picks its own).
 *   state        current RocketState.
 *   fuse_sec     seconds remaining before next launch (RS_IDLE only).
 *   particles    inline pool of PARTICLES_PER_BURST Sparks (never
 *                allocated/freed — just deactivated and overwritten
 *                on the next burst).
 */
typedef struct {
    float        x, y;
    float        vy;
    int          color;
    RocketState  state;
    float        fuse_sec;
    Spark        particles[PARTICLES_PER_BURST];
} Rocket;

/* ── §5.2 rocket_launch — fresh rising rocket from the bottom ─────── */

static void rocket_launch(Rocket *r, int cols, int rows)
{
    r->x     = (float)(rand() % cols);
    r->y     = (float)(rows - 1);
    r->vy    = ROCKET_LAUNCH_VY_MIN_CPS
             + urand01() * (ROCKET_LAUNCH_VY_MAX_CPS - ROCKET_LAUNCH_VY_MIN_CPS);
    r->color = color_rand();
    r->state = RS_RISING;

    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        r->particles[i].active = false;
}

/* ── §5.3 per-state tick functions — one per RocketState ─────────── */

/*
 * RS_IDLE — count down the fuse; launch when it hits zero.
 *
 * cols/rows are forwarded to rocket_launch which uses them to pick
 * a random column for the new rocket.
 */
static void rocket_tick_idle(Rocket *r, float dt, int cols, int rows)
{
    r->fuse_sec -= dt;
    if (r->fuse_sec <= 0.0f)
        rocket_launch(r, cols, rows);
}

/*
 * RS_RISING — integrate position and gravity; explode at apex
 * (vy ≥ 0) or if the rocket clears the top edge (rare too-fast
 * launch). When exploding, spawn PARTICLES_PER_BURST sparks at
 * the rocket's current position.
 */
static void rocket_tick_rising(Rocket *r, float dt)
{
    r->y  += r->vy * dt;
    r->vy += ROCKET_GRAVITY_CPS2 * dt;

    if (r->vy >= 0.0f || r->y < ROCKET_TOP_EXPLODE_ROW) {
        Vec2 origin = { r->x, r->y };
        spark_burst_spawn(r->particles, PARTICLES_PER_BURST, origin);
        r->state = RS_EXPLODED;
    }
}

/*
 * RS_EXPLODED — tick every spark; when none remain alive, return
 * to RS_IDLE with a fresh random fuse.
 */
static void rocket_tick_exploded(Rocket *r, float dt)
{
    bool any_alive = false;
    for (int i = 0; i < PARTICLES_PER_BURST; i++) {
        spark_tick(&r->particles[i], dt);
        if (r->particles[i].active) any_alive = true;
    }
    if (!any_alive) {
        r->fuse_sec = FUSE_MIN_SEC + urand01() * FUSE_VAR_SEC;
        r->state    = RS_IDLE;
    }
}

/* ── §5.4 rocket_tick — dispatch on state ───────────────────────── */

static void rocket_tick(Rocket *r, float dt, int cols, int rows)
{
    switch (r->state) {
    case RS_IDLE:     rocket_tick_idle    (r, dt, cols, rows); break;
    case RS_RISING:   rocket_tick_rising  (r, dt);             break;
    case RS_EXPLODED: rocket_tick_exploded(r, dt);             break;
    }
}

/* ── §5.5 rocket_draw — body while rising, sparks when exploded ──── */

/*
 * Body while rising; particle trails while exploded. IDLE rockets
 * draw nothing — they're "below ground" waiting on the fuse.
 */
static void rocket_draw(const Rocket *r, int cols, int rows)
{
    if (r->state == RS_RISING) {
        int x = (int)r->x;
        int y = (int)r->y;
        if (x >= 0 && x < cols && y >= 0 && y < rows) {
            attron(COLOR_PAIR(r->color) | A_BOLD);
            mvaddch(y, x, '|');
            attroff(COLOR_PAIR(r->color) | A_BOLD);

            if (y + 1 < rows) {
                attron(COLOR_PAIR(r->color));
                mvaddch(y + 1, x, '\'');
                attroff(COLOR_PAIR(r->color));
            }
        }
    }

    if (r->state == RS_EXPLODED) {
        for (int i = 0; i < PARTICLES_PER_BURST; i++)
            spark_draw(&r->particles[i], cols, rows);
    }
}

/* ===================================================================== */
/* §6  show — fixed rocket pool + tick + draw + pause                     */
/* ===================================================================== */

typedef struct {
    Rocket rockets[MAX_ROCKETS];
    int    active_rockets;
    float  speed_scale;       /* global multiplier on dt ([ / ] keys)    */
    bool   paused;
} Show;

static void show_init(Show *s, int cols, int rows, int rocket_count)
{
    s->active_rockets = rocket_count;
    s->speed_scale    = SPEED_SCALE_DEFAULT;
    s->paused         = false;

    for (int i = 0; i < MAX_ROCKETS; i++) {
        if (i < rocket_count) {
            rocket_launch(&s->rockets[i], cols, rows);
            /* Stagger so they don't all fire at once. */
            s->rockets[i].fuse_sec = (float)i * INITIAL_FUSE_STAGGER_SEC;
            s->rockets[i].state    = RS_IDLE;
        } else {
            s->rockets[i].state    = RS_IDLE;
            s->rockets[i].fuse_sec = 1e9f;            /* effectively never */
            for (int j = 0; j < PARTICLES_PER_BURST; j++)
                s->rockets[i].particles[j].active = false;
        }
    }
}

static void show_free(Show *s) { memset(s, 0, sizeof *s); }

static void show_tick(Show *s, float dt, int cols, int rows)
{
    if (s->paused) return;
    float scaled = dt * s->speed_scale;
    for (int i = 0; i < s->active_rockets; i++)
        rocket_tick(&s->rockets[i], scaled, cols, rows);
}

static void show_draw(const Show *s, int cols, int rows)
{
    for (int i = 0; i < s->active_rockets; i++)
        rocket_draw(&s->rockets[i], cols, rows);
}

/* Show input helpers — used by app_handle_key. */

static void show_change_rockets(Show *s, int delta, int cols, int rows)
{
    int n = s->active_rockets + delta;
    if (n < ROCKETS_MIN) n = ROCKETS_MIN;
    if (n > ROCKETS_MAX) n = ROCKETS_MAX;
    if (n > s->active_rockets) {
        /* Activate the next slot with a quick fuse so it fires soon. */
        int i = s->active_rockets;
        rocket_launch(&s->rockets[i], cols, rows);
        s->rockets[i].fuse_sec = INITIAL_FUSE_STAGGER_SEC;
        s->rockets[i].state    = RS_IDLE;
    }
    s->active_rockets = n;
}

static void show_scale_speed(Show *s, float factor)
{
    s->speed_scale *= factor;
    if (s->speed_scale < SPEED_SCALE_MIN) s->speed_scale = SPEED_SCALE_MIN;
    if (s->speed_scale > SPEED_SCALE_MAX) s->speed_scale = SPEED_SCALE_MAX;
}

/* ===================================================================== */
/* §7  screen — ncurses init / present / HUD                              */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);              /* don't let stdin interrupt frame writes */
    start_color();
    use_default_colors();       /* lets HUD pairs use -1 background       */
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
 * screen_draw_hud — required HUD per CLAUDE.md spec.
 *
 *   Row 0       PAIR_HUD  + A_BOLD  (yellow) — fps + state + params
 *   Bottom row  PAIR_HINT + A_BOLD  (cyan)   — full key list
 *
 * Both pairs sit on default background (-1) so they stay legible
 * regardless of theme. theme_apply() never touches them.
 */
static void screen_draw_hud(const Screen *sc, double fps,
                            const Show *show, int theme_idx)
{
    char buf[HUD_BUF_LEN];
    snprintf(buf, sizeof buf,
             " %5.1f fps  spd:%.2fx  rkt:%d  [%s] %s ",
             fps, show->speed_scale, show->active_rockets,
             k_themes[theme_idx].name,
             show->paused ? "PAUSED " : "running");

    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  []:speed  +/-:rockets  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app — signals, resize, variable-dt main loop                       */
/* ===================================================================== */

typedef struct {
    Show                  show;
    Screen                screen;
    int                   theme_idx;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_do_resize — full re-init of the show on the new screen size,
 * preserving user-tuned theme and rocket count.
 */
static void app_do_resize(App *app)
{
    int   saved_n     = app->show.active_rockets;
    float saved_speed = app->show.speed_scale;

    show_free(&app->show);
    screen_resize(&app->screen);
    show_init(&app->show, app->screen.cols, app->screen.rows, saved_n);
    app->show.speed_scale = saved_speed;
    app->need_resize      = 0;
}

/* Map one keypress to an action. Returns false on quit. */
static bool app_handle_key(App *app, int ch)
{
    Show *s = &app->show;
    switch (ch) {

    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ': case 'p': case 'P':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        show_init(s, app->screen.cols, app->screen.rows, s->active_rockets);
        break;

    case ']':
        show_scale_speed(s, SPEED_SCALE_STEP);
        break;

    case '[':
        show_scale_speed(s, 1.0f / SPEED_SCALE_STEP);
        break;

    case '=': case '+':
        show_change_rockets(s, +1, app->screen.cols, app->screen.rows);
        break;

    case '-':
        show_change_rockets(s, -1, app->screen.cols, app->screen.rows);
        break;

    case 't': case 'T':
        app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
        theme_apply(app->theme_idx);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app       = &g_app;
    app->running   = 1;
    app->theme_idx = 0;

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    theme_apply(app->theme_idx);
    hud_pairs_init();
    show_init(&app->show, app->screen.cols, app->screen.rows, ROCKETS_DEFAULT);

    int64_t last_ns      = clock_ns();
    int64_t fps_accum_ns = 0;
    int     fps_frames   = 0;
    double  fps_display  = 0.0;
    const int64_t TICK_NS = NS_PER_SEC / TARGET_FPS;

    while (app->running) {

        /* (1) handle resize first so subsequent steps see the new size */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* (2) measure dt, capped to prevent spiral-of-death */
        int64_t now_ns = clock_ns();
        int64_t dt_ns  = now_ns - last_ns;
        last_ns        = now_ns;
        float   dt     = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        /* (3) drain input */
        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        /* (4) advance the show */
        show_tick(&app->show, dt, app->screen.cols, app->screen.rows);

        /* (5) rolling fps display */
        fps_accum_ns += dt_ns;
        fps_frames++;
        if (fps_accum_ns >= NS_PER_SEC / 2) {
            fps_display = (double)fps_frames * 1e9
                        / (double)fps_accum_ns;
            fps_accum_ns = 0;
            fps_frames   = 0;
        }

        /* (6) draw + present */
        erase();
        show_draw(&app->show, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, fps_display, &app->show, app->theme_idx);
        screen_present();

        /* (7) frame cap — sleep before the NEXT frame's I/O */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(TICK_NS - elapsed);
    }

    show_free(&app->show);
    screen_free(&app->screen);
    return 0;
}
