/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shepherd.c — autonomous border collie herds scattered sheep back into a pen
 *
 * DEMO: A flock of sheep grazes calmly inside a circular pen drawn in
 *       the centre of the screen.  Press SPACE: every sheep gets an
 *       outward impulse and runs for the boundary.  A single border
 *       collie outside the pen runs to whichever sheep has strayed
 *       furthest, positions itself behind that sheep relative to the
 *       pen, and the sheep — fleeing the dog — runs back through the
 *       boundary on its own.  The dog repeats with the next outlier
 *       until every sheep is back inside.
 *
 * Study alongside: flocking/flocking.c (the boid forces sheep use)
 *                  flocking/crowd.c    (steering primer + Person model)
 *
 * Section map:
 *   §1 config   — pen geometry, sheep/dog speeds, force weights
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — pen / sheep / dog pairs + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect-ratio bridge + Vec2 helpers
 *   §5 sheep    — Sheep struct, spawn, four steering forces
 *   §6 dog      — Dog struct + Strömbom collect/patrol controller
 *   §7 scene    — Pen + flock + dog; tick + scatter; render with mark_cell
 *   §8 app      — signals, resize, main game loop
 *
 * Keys:
 *   q / ESC    quit                       space    SCATTER (panic the herd)
 *   S          mega-scatter (2x impulse)  c        toggle continuous chaos
 *   p          pause / resume             r        reset (regather in pen)
 *   + / -      add / remove 5 sheep
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/shepherd.c -o shepherd -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two interlocking pieces share the screen.
 *
 *                  SHEEP — each sheep is a steering-behaviour particle
 *                  (Reynolds 1987 style) with four forces summed each
 *                  tick: separation from close neighbours, soft cohesion
 *                  toward the local centroid, flee from the dog (only
 *                  when within DOG_FLEE_RADIUS), and a gentle inward
 *                  pull whenever the sheep is outside the pen.  Forces
 *                  are integrated into velocity with a per-tick damping
 *                  factor so motion settles to rest when no force
 *                  applies (sheep can actually stop and graze).
 *
 *                  DOG — a Strömbom-style controller (Strömbom et al.
 *                  2014, "Solving the shepherding problem").  Each tick
 *                  the dog picks one of two modes:
 *                    1. COLLECT — if any sheep is more than
 *                       (pen_radius + tolerance) from the pen centre,
 *                       find the worst outlier and place the dog at
 *                       sheep_pos + APPROACH_OFFSET · (sheep − pen)/|...|.
 *                       I.e. directly OUTSIDE the sheep relative to the
 *                       pen.  The sheep, fleeing the dog, runs toward
 *                       the pen interior — the goal direction emerges
 *                       from positioning, not from a goal-aware sheep.
 *                    2. PATROL — when all sheep are inside the pen
 *                       (with tolerance), the dog walks a slow circle
 *                       around the pen at radius pen_r + PATROL_OFFSET.
 *                       Patrolling rather than parking keeps the dog
 *                       visible and reactive when the user triggers a
 *                       new scatter event.
 *
 *                  The dog has no path planner — it simply steers
 *                  toward its target position at DOG_SPEED.  The
 *                  emergent herding comes from the two-mode position
 *                  selection.
 *
 * Data-structure : Scene owns a Pen (centre + radius), a fixed-capacity
 *                  Sheep pool (only first n_sheep slots active), a
 *                  single Dog, and the current world dimensions
 *                  (refreshed each tick from terminal cols/rows).
 *                  Sheep carry pos/prev_pos/vel + a `fleeing` flag set
 *                  each tick by the flee force.  Dog carries pos +
 *                  target + the controller's mode + the sheep index it
 *                  is currently collecting.
 *
 * Rendering      : Painter's order — pen ring (`*` and `.` alternated
 *                  for a dashed look) → sheep (`o` calm, `O` panicking)
 *                  → dog (`&`, A_BOLD).  Sub-tick alpha lerp of pos
 *                  smooths motion at any sim-rate / render-rate combo.
 *                  Every glyph stamp goes through a `mark_cell()`
 *                  helper that carries the (chtype)(unsigned char) cast
 *                  and the bounds-check.
 *
 * Performance    : Sheep separation/cohesion are O(N²); dog mode is
 *                  O(N) (one scan to find the worst outlier).  At
 *                  N = 30 sheep this is 30·29 = 870 distance checks
 *                  per tick — under 0.1 ms on any modern CPU.
 *
 * References     : Strömbom, Mann, Wilson, Hailes, Morton, Sumpter, &
 *                    King, "Solving the shepherding problem: heuristics
 *                    for herding autonomous, interacting agents,"
 *                    J. R. Soc. Interface 11 (2014).  The collect/drive
 *                    decomposition used here.
 *                  Reynolds, "Flocks, Herds, and Schools: A Distributed
 *                    Behavioral Model," SIGGRAPH 1987 — the boid forces
 *                    that drive the sheep.
 *                  Reynolds, "Steering Behaviors for Autonomous
 *                    Characters," 1999 — the unified seek/flee form
 *                    used by sheep_flee_dog and sheep_pen_pull.
 *                  Wikipedia: "Sheepdog trial" — context for the
 *                    real-world task being simulated.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The dog never tells a sheep where to go.  The dog picks WHERE TO
 * STAND, and a fleeing sheep does the rest.  If the dog stands on the
 * far side of a sheep relative to the pen, the sheep — running away
 * from the dog — happens to run toward the pen.  Repeat for every
 * outlier and the herd collapses back inward.  The whole "intelligence"
 * is one geometric placement rule applied each tick.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a real sheepdog trial: the dog runs wide around the flock,
 * approaches the strays from the *outside*, and lets the sheep's own
 * panic do the work of pushing them back.  A bad dog charges straight
 * at the flock and just scatters them further.  A good dog uses
 * geometry — pick a position, hold the line, let the sheep flee
 * toward where you want them.
 *
 * Now picture the algorithm as a flowchart:
 *
 *      every tick
 *          │
 *          ▼
 *    pick the sheep furthest from pen centre
 *          │
 *          ├── if its distance > pen_r + tolerance:
 *          │       dog target = sheep_pos + offset · (away-from-pen)
 *          │       state      = COLLECT
 *          │
 *          └── else (everyone home):
 *                  dog target = pen_centre + (pen_r + offset) ·
 *                               (cos(patrol_phase), sin(patrol_phase))
 *                  state      = PATROL
 *
 * That's it.  No targets for individual sheep, no goal direction in
 * any sheep's head — just a moving repulsor (the dog) placed where
 * its repulsion happens to point inward.
 *
 * ALGORITHM IN STEPS  (per tick)
 * ──────────────────────────────
 *   1. Read inputs: SPACE → impulse every sheep outward; 'c' → toggle
 *      continuous random nudges; arrow keys not used (autonomous).
 *   2. Dog controller:
 *        a. scan sheep, find argmax(distance from pen centre).
 *        b. if worst distance > pen_r + PEN_TOLERANCE:
 *               mode = COLLECT
 *               target = sheep[worst].pos
 *                      + DOG_APPROACH_OFFSET · normalize(sheep[worst].pos − pen_centre)
 *           else:
 *               mode = PATROL
 *               patrol_phase += DOG_PATROL_OMEGA · dt
 *               target = pen_centre
 *                      + (pen_r + DOG_PATROL_OFFSET) · (cos phase, sin phase)
 *        c. dog.vel = DOG_SPEED · normalize(target − dog.pos)
 *           dog.pos += dog.vel · dt
 *   3. Sheep update (two-stage so all sheep see the OLD positions):
 *        a. for each sheep i compute force = Σ weights · (sep, coh, flee, pen_pull)
 *        b. for each sheep i: vel = (vel + force·dt) · DAMPING ; clamp |vel|
 *        c. for each sheep i: prev_pos = pos ; pos += vel · dt ; clamp to world.
 *   4. Render: pen ring → sheep (alpha-lerped) → dog (alpha-lerped, on top).
 *
 * KEY FORMULAS
 * ────────────
 *   Sheep separation (per neighbour with 0 < d < SHEEP_SEP_RADIUS):
 *     strength = (SHEEP_SEP_RADIUS − d) / SHEEP_SEP_RADIUS
 *     force   += normalize(self − neighbour) · strength · SHEEP_GRAZE_SPEED
 *
 *   Sheep flee from dog (only if within DOG_FLEE_RADIUS):
 *     strength = (DOG_FLEE_RADIUS − d) / DOG_FLEE_RADIUS
 *     force   += normalize(self − dog) · strength · SHEEP_FLEE_SPEED
 *
 *   Sheep pen pull (only if outside pen):
 *     force   += normalize(pen_centre − self) · SHEEP_GRAZE_SPEED
 *
 *   Dog COLLECT target (the geometric trick):
 *     target  = sheep[worst].pos
 *             + DOG_APPROACH_OFFSET · normalize(sheep[worst].pos − pen_centre)
 *
 *     i.e. the dog stands directly outside the sheep, on the line from
 *     pen-through-sheep, at offset DOG_APPROACH_OFFSET further out.
 *     The sheep, fleeing radially away from the dog, ends up moving
 *     along the SAME line but in the OPPOSITE direction — back toward
 *     the pen.
 *
 *   Dog PATROL target:
 *     target  = pen_centre + (pen_r + DOG_PATROL_OFFSET) · (cos φ, sin φ)
 *     φ      += DOG_PATROL_OMEGA · dt
 *
 *   Pixel→cell aspect bridge:
 *     cx = round(px / CELL_W)        (CELL_W = 8)
 *     cy = round(py / CELL_H)        (CELL_H = 16)
 *
 * WORKED EXAMPLE  (defaults: 30 sheep, 80x24 terminal)
 * ────────────────────────────────────────────────────
 *   World box       : 80 cols × 24 rows = 640 × 384 pixels.
 *   Pen             : centre = (320, 192), radius = 0.18 · 384 ≈ 69 px
 *                     = 8.6 cols × 4.3 rows in cell space.  Sheep
 *                     spawn uniformly in a disc of radius 0.7 · 69 ≈
 *                     48 px so they appear inside the visual ring.
 *   Per tick (1/60 s):
 *     a calm sheep at SHEEP_GRAZE_SPEED (20 px/s) drifts 20/60 ≈
 *       0.33 px — visibly stationary.
 *     a fleeing sheep at SHEEP_FLEE_SPEED (140 px/s) covers
 *       140/60 ≈ 2.3 px ≈ one new cell every 3-4 ticks.
 *     the dog at DOG_SPEED (180 px/s) crosses 3 px per tick — fast
 *       enough to outflank a fleeing sheep (180 > 140 px/s).
 *   SCATTER          : SPACE adds 220 px/s outward radial impulse to
 *     every sheep.  Sheep at the centre with no outward direction get
 *     a random direction.  Each sheep also gets ±0.6 rad of jitter on
 *     its scatter heading so they fan out, not all in straight lines.
 *   Time to herd back: from a single SPACE press, ~30 sheep, default
 *     speeds — visually about 5-15 seconds depending on how far the
 *     scatter sent them and which order the dog picks the outliers.
 *   Steering cost   : sheep separation + cohesion are O(N²) inside one
 *     flock.  At N = 30: 30·29 = 870 checks per tick × 60 Hz ≈ 52 000
 *     checks/second — microseconds total.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - Sheep at pen centre with v ≈ 0 has no "outward" direction for
 *     the scatter impulse.  scene_scatter falls back to a uniform
 *     random angle for any sheep whose distance to centre is below
 *     1 px.
 *   - All sheep already inside pen: dog_decide_target returns PATROL
 *     mode; the dog circles the pen at constant speed instead of
 *     parking (parking would make resume from a fresh scatter feel
 *     laggy because the dog has to accelerate from rest).
 *   - Sheep at corner of world: clamped to [0, w] × [0, h] hard.
 *     Without the clamp, a strong dog flee could push a sheep off-
 *     screen and the renderer would silently drop it; clamping keeps
 *     every sheep visible.
 *   - Two-stage update: writing into the same array we are reading
 *     from causes index-order drift.  scene_tick fills new_vel[]
 *     fully before writing any back.
 *   - Dog overshoots target: dog_step uses a simple desired-velocity
 *     P-controller (no overshoot guard).  At default speeds the dog
 *     gets within a few pixels of target each tick; if you crank
 *     DOG_SPEED much higher, the dog can oscillate around target —
 *     fix would be a min(DOG_SPEED, dist/dt) clamp.
 *   - Frame cap: never `elapsed = clock_ns() − frame_time + dt` —
 *     adding dt cancels the cap.  Use a `frame_start` snapshot.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - At startup: 30 sheep mill calmly inside the dashed circle; dog
 *     walks a slow patrol arc around the outside.  The HUD reads
 *     "sheep:30/30  dog:PATROL".
 *   - Press SPACE: every sheep glyph turns 'O' (bold, red), velocities
 *     point radially outward, the dog immediately switches to
 *     "dog:COLLECT" and runs to the worst outlier.  Within 5-15 s the
 *     sheep are all back inside; dog returns to PATROL.
 *   - Press 'c' (continuous chaos): random small kicks every tick.  The
 *     dog never gets to PATROL — it cycles between outliers
 *     indefinitely.  The herd never quite settles, which is the point.
 *   - Reduce sheep count with '-': 5 sheep, one SPACE press — the dog
 *     should sweep them up one by one in clearly visible order.
 *   - Hold SPACE (auto-repeat): each scatter restarts the chase; the
 *     dog re-targets between presses without lag.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read flocking.c first for boid mechanics; read
 *      crowd.c for steering primitives library. The NEW LESSONS
 *      here are: heterogeneous agents (sheep + dog), Strömbom
 *      collect-or-patrol controller, and "geometry replaces goal-
 *      awareness" trick.
 *   2. §6 dog — THE HEART of this file. Strömbom's collect / patrol
 *      controller. Read AFTER tutorials T1-T5 below.
 *   3. §5 sheep — sheep struct + four steering forces (separation,
 *      cohesion, flee dog, pen pull).
 *   4. §7 scene — orchestrator: pen + flock + dog ticks.
 *   5. §1-§4, §8 — config / clock / colour / coords / app loop.
 *      Skim if you've seen the framework.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   pen.centre, pen.radius      where the sheep belong.
 *   PEN_TOLERANCE               extra slack beyond pen radius
 *                               before the dog starts collecting.
 *   sheep.pos, vel, prev_pos    standard particle state.
 *   sheep.fleeing               bool — set per tick when within
 *                               DOG_FLEE_RADIUS. Drives glyph
 *                               choice.
 *   dog.pos, target             dog's position + where it's
 *                               heading.
 *   dog.mode                    COLLECT or PATROL.
 *   dog.target_idx              which sheep is the current
 *                               outlier (during COLLECT).
 *   patrol_phase                radians around the pen for patrol
 *                               orbit.
 *   DOG_APPROACH_OFFSET         the geometric magic — distance
 *                               the dog stands BEHIND the sheep
 *                               relative to the pen.
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T7 (boid + Reynolds rules + 2-stage update).
 *   - crowd.c T1-T2 (steering primitives — seek/flee).
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Reinforcement learning / trained dog policies. Strömbom's
 *     algorithm is a HAND-CRAFTED rule that works without
 *     training.
 *   - Goal-aware sheep with mental maps of the pen. Sheep are
 *     PURELY REACTIVE — they don't even know the pen exists in
 *     a meaningful sense (only that "outside is bad, inside is
 *     fine").
 *   - Multi-dog coordination. We have ONE dog; multi-dog
 *     herding adds a coordination layer Strömbom (2014) covers
 *     but we don't.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Five tutorials that build a Strömbom shepherd from first
 * principles.
 *
 *   T1  Heterogeneous agents — sheep ≠ dog
 *   T2  The geometric trick — dog stands BEHIND the sheep
 *   T3  Strömbom's two modes — COLLECT vs PATROL
 *   T4  Why this is "intelligence without intelligence"
 *   T5  Damping and rest — sheep that actually stop grazing
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  HETEROGENEOUS AGENTS — SHEEP ≠ DOG
 * ──────────────────────────────────────
 * Up until this file every flocking demo had ONE KIND of agent.
 * Reynolds boids: all boids same. Crowd: all people same. Even
 * murmuration with the hawk: hawk is a DECORATIVE state-machine
 * predator with its own simple controller.
 *
 * shepherd.c has TWO kinds of agents with INVERSE behaviours:
 *
 *   SHEEP   reactive: separation, cohesion, flee dog, pen pull
 *           passive: graze speed (low), no goal awareness
 *
 *   DOG     deliberative: scans entire flock, picks outlier,
 *           positions to evict outlier toward pen
 *           active: high speed, picks targets
 *
 * Two heterogeneous agents in one simulation. Each ticks its
 * own logic. The interaction is asymmetric:
 *
 *   - SHEEP sees DOG: as a flee target.
 *   - DOG sees SHEEP: as a goal (which one is furthest from pen?)
 *
 * Same simulation, different roles. Real predator-prey,
 * herding, swarm-robotics scenarios all need this asymmetry.
 *
 * Implementation: separate Sheep + Dog structs, separate tick
 * functions in §5 / §6, but they share the World (positions are
 * in the same coordinate system, distance computations are the
 * same).
 *
 * T2  THE GEOMETRIC TRICK — DOG STANDS BEHIND THE SHEEP
 * ─────────────────────────────────────────────────────
 * The crucial insight from Strömbom et al. (2014):
 *
 *   "To herd a sheep TOWARD some destination, position the dog
 *    at the OPPOSITE side of the sheep relative to the
 *    destination. The sheep flees radially from the dog. Result:
 *    sheep moves toward the destination."
 *
 * Geometrically:
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   dog target                                     │
 *      │       &                                          │
 *      │       │   ← place dog HERE                       │
 *      │       │                                          │
 *      │       o   ← sheep, fleeing                       │
 *      │       │                                          │
 *      │       │   ← sheep flees DOWN (away from dog)     │
 *      │       │                                          │
 *      │       •   ← pen centre (destination)             │
 *      │                                                  │
 *      │  Line: dog → sheep → pen                         │
 *      │  Dog position is sheep + offset · (sheep − pen)/ │
 *      │                                |sheep − pen|     │
 *      │  i.e. extending the (pen → sheep) ray by `offset`│
 *      └──────────────────────────────────────────────────┘
 *
 * In code:
 *
 *     away_dir = normalize(sheep.pos - pen.centre)
 *     dog.target = sheep.pos + DOG_APPROACH_OFFSET · away_dir
 *
 * The sheep doesn't know about the pen. The dog doesn't push
 * the sheep. The dog STANDS IN THE RIGHT PLACE, and the sheep
 * does the rest by panicking.
 *
 * This is GEOMETRY REPLACING GOAL-AWARENESS. The dog has a
 * goal (sheep should be in pen); the sheep doesn't know that
 * goal. The dog encodes the goal as a POSITIONING RULE so the
 * sheep's purely reactive behaviour produces the desired
 * movement.
 *
 * T3  STRÖMBOM'S TWO MODES — COLLECT vs PATROL
 * ────────────────────────────────────────────
 * The dog runs ONE controller per tick. The controller picks
 * one of two modes based on the sheep state:
 *
 *     find worst outlier:
 *       worst = argmax_i |sheep[i].pos - pen.centre|
 *
 *     if |sheep[worst].pos - pen.centre| > pen.radius + tolerance:
 *       MODE = COLLECT
 *       target = position behind sheep[worst] (T2 formula)
 *     else:
 *       MODE = PATROL
 *       phase += omega · dt
 *       target = pen.centre + (pen.radius + offset) ·
 *                (cos phase, sin phase)
 *
 * Then the dog steers toward target at DOG_SPEED.
 *
 * Why PATROL when all sheep are home? Two reasons:
 *
 *   - Visual: the dog stays ON SCREEN moving, signalling
 *     readiness. Without patrol, the dog would freeze somewhere,
 *     hard to read.
 *
 *   - Reactive readiness: when SPACE is pressed and sheep
 *     scatter, the dog's PATROL position is already near the
 *     edge of the pen — it can switch to COLLECT instantly
 *     without first running halfway across the screen.
 *
 * Mode selection runs every tick: as soon as one sheep escapes
 * the pen (during PATROL), the controller switches to COLLECT
 * and the dog redirects. As soon as all sheep are home (during
 * COLLECT), it switches back to PATROL.
 *
 * No "while" loops, no plan. Just decide, every tick, what to
 * do RIGHT NOW.
 *
 * T4  WHY THIS IS "INTELLIGENCE WITHOUT INTELLIGENCE"
 * ───────────────────────────────────────────────────
 * Both agents are SIMPLE:
 *
 *   - Sheep: 4 force terms summed + damping. Maybe 30 lines.
 *   - Dog: argmax + position formula + steering. Maybe 40 lines.
 *
 * Yet the result LOOKS LIKE coordinated, intentional behaviour
 * — a real sheepdog herding a real flock back to a pen.
 *
 * That's because the INTELLIGENCE LIVES IN THE INTERACTION,
 * not in either agent. The dog's positioning rule + the sheep's
 * flee response together produce GOAL-DIRECTED movement that
 * NEITHER party planned alone.
 *
 * Same principle in:
 *
 *   - flocking emergence (no boid plans the swarm shape)
 *   - ant trails (no ant plans the path; pheromones do)
 *   - termite mounds (each termite drops sand; the mound
 *                     emerges)
 *   - market price discovery (no trader knows the equilibrium;
 *                             trades discover it)
 *
 * SHEPHERD's specific lesson: when designing multi-agent
 * systems, push as much logic into AGENT INTERACTIONS and as
 * LITTLE into AGENT INTROSPECTION as possible. Reactive
 * agents + clever positioning beat goal-aware agents
 * every time for emergent behaviour.
 *
 * T5  DAMPING AND REST — SHEEP THAT ACTUALLY STOP GRAZING
 * ───────────────────────────────────────────────────────
 * Reynolds boids in flocking.c clamp speed to [MIN_SPEED,
 * MAX_SPEED]. MIN_SPEED > 0 means boids NEVER stop — they
 * always cruise.
 *
 * For sheep that doesn't fit. A real flock at rest GRAZES —
 * sheep mostly stand still, occasionally take a step, mostly
 * still again. We want sheep that STOP when no force applies.
 *
 * Solution: replace MIN_SPEED clamp with VELOCITY DAMPING:
 *
 *     vel.x *= DAMPING        (e.g. 0.92 per tick)
 *     vel.y *= DAMPING
 *
 * Each tick velocity decays toward zero. With no applied force,
 * a sheep at vel = 50 px/s drops to 50 · 0.92 = 46 next tick,
 * 42 next, ... down to ~0 in a couple of seconds.
 *
 * Combined with the still-applied separation force (which
 * pushes sheep apart slightly when they bump), the net behaviour
 * is: sheep drift apart slowly, slow to a stop, drift again
 * when crowded. Looks exactly like grazing.
 *
 * Damping is also why the SCATTER impulse is finite-duration —
 * sheep accelerate up to flee speed, then decelerate as the
 * impulse + force_dt sum stays smaller than the damping
 * subtraction. Without damping, sheep would keep fleeing
 * forever from the impulse.
 *
 * Same trick is used in any "agents that should sometimes
 * rest" simulation: sleeping NPCs, idle pedestrians,
 * grazing animals. Damping is the simplest way to encode
 * "if no force, settle to zero" without adding an explicit
 * REST state.
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_DEFAULT = 60,
    TARGET_FPS      = 60,

    HUD_COLS        = 96,
    FPS_UPDATE_MS   = 500,

    /* Colour pair IDs — see §3 color_init for the actual colour values. */
    PAIR_PEN_RING    = 1,   /* dashed pen boundary circle               */
    PAIR_SHEEP_CALM  = 2,   /* sheep grazing inside pen                 */
    PAIR_SHEEP_FLEE  = 3,   /* sheep panicking from dog or scatter      */
    PAIR_DOG         = 4,   /* the border collie                        */
    PAIR_HUD         = 8,   /* bright yellow — top status               */
    PAIR_HINT        = 9,   /* bright cyan   — bottom key hint          */

    SHEEP_MIN        = 5,
    SHEEP_MAX        = 80,
    SHEEP_DEFAULT    = 30,
    SHEEP_STEP       = 5,
};

/* Cell dimensions — physics is in pixel space (CELL_W × CELL_H sub-pixels
 * per terminal cell) so motion is isotropic regardless of the terminal's
 * taller-than-wide cells. */
#define CELL_W   8
#define CELL_H  16

/* ── pen geometry ────────────────────────────────────────────────────── */
/*
 * PEN_RADIUS_FRAC — pen radius as a fraction of min(world_w, world_h).
 *   0.18 → on an 80x24 terminal (640x384 px), pen radius ≈ 69 px ≈ 9
 *   columns / 4.3 rows.  Big enough to contain 30 sheep with comfortable
 *   spacing; small enough that the dog has visible room outside it.
 *
 * PEN_TOLERANCE  — extra slack added to pen_radius before a sheep counts
 *   as "outlier" (i.e. dog should COLLECT it).  16 px ≈ 2 columns:
 *   sheep teetering exactly on the boundary do not trigger a fresh
 *   collect cycle.
 */
#define PEN_RADIUS_FRAC  0.18f
#define PEN_TOLERANCE   16.0f

/* ── sheep speeds (px/s) ─────────────────────────────────────────────── */
/*
 * SHEEP_GRAZE_SPEED — relaxed milling speed when no forces apply.
 * SHEEP_FLEE_SPEED  — speed scaling on flee force (note: this is the
 *                     desired-flee MAGNITUDE used in the formula, not
 *                     a hard cap; max actual speed is SHEEP_MAX_SPEED).
 * SHEEP_MAX_SPEED   — hard cap on |vel| each tick after force integration.
 *                     Set above SHEEP_FLEE_SPEED so flee can saturate
 *                     but below DOG_SPEED so the dog can outflank.
 */
#define SHEEP_GRAZE_SPEED   20.0f
#define SHEEP_FLEE_SPEED   140.0f
#define SHEEP_MAX_SPEED    180.0f

/* ── sheep flocking radii (px) ───────────────────────────────────────── */
/*
 * SHEEP_SEP_RADIUS — personal-space bubble.  At 24 px ≈ 3 cols, two
 *   adjacent sheep on the same row (8 px apart) are well inside it and
 *   gently push apart.
 *
 * SHEEP_COH_RADIUS — local cohesion sensing range.  At 96 px ≈ 12 cols,
 *   a sheep sees most of its flockmates without seeing the entire screen.
 *
 * DOG_FLEE_RADIUS  — sheep flee disc around the dog.  At 120 px ≈ 15
 *   cols, the dog's "scary aura" extends well past the pen tolerance,
 *   so sheep at the boundary still feel pressure.
 */
#define SHEEP_SEP_RADIUS    24.0f
#define SHEEP_COH_RADIUS    96.0f
#define DOG_FLEE_RADIUS    120.0f

/* ── sheep force weights ─────────────────────────────────────────────── */
/*
 * Order of magnitude reflects priority:
 *   FLEE   (2.6) overrides COH (0.4) so panic disperses, not clumps.
 *   SEP    (1.6) is firm enough to prevent overlap during the panic
 *                  but does not dominate FLEE.
 *   PEN_PULL (0.3) is the weakest — only relevant when outside the pen
 *                  AND the dog is far away.  It is a *gentle* nudge, not
 *                  an attractor that competes with the dog's repulsion.
 */
#define W_SHEEP_SEP    1.6f
#define W_SHEEP_COH    0.4f
#define W_SHEEP_FLEE   2.6f
#define W_PEN_PULL     0.3f

/* SHEEP_DAMPING — per-tick velocity scale.
 *   0.95 at 60 Hz → 0.95^60 ≈ 4.6 % retained after 1 second of zero force.
 *   That is, a sheep with no forces decays from 100 px/s to ~5 px/s in
 *   a second — looks like grazing, not a perpetual-motion glide. */
#define SHEEP_DAMPING  0.95f

/* ── scatter event ───────────────────────────────────────────────────── */
/*
 * SCATTER_IMPULSE — radial outward push (px/s) applied instantaneously
 *   to every sheep on SPACE.  220 px/s is well above SHEEP_FLEE_SPEED so
 *   the impulse is the dominant motion for the first ~0.5 s after press.
 *
 * SCATTER_JITTER  — angular randomisation per sheep, in radians.
 *   ±0.6 rad ≈ ±34° so sheep fan out radially rather than firing along
 *   parallel rays from the same direction.
 */
#define SCATTER_IMPULSE 220.0f
#define SCATTER_JITTER    0.6f

/* ── dog ─────────────────────────────────────────────────────────────── */
/*
 * DOG_SPEED              — dog's cruise speed.  Must exceed SHEEP_FLEE_SPEED
 *                          (140) so the dog can outflank a fleeing sheep
 *                          rather than chase from behind.
 *
 * DOG_APPROACH_OFFSET    — distance from sheep to dog when COLLECTing.
 *                          Too small (< sheep flee radius) and the sheep
 *                          panics in random directions because the flee
 *                          force saturates; too large and the dog takes
 *                          forever to influence the sheep.  60 px sits
 *                          comfortably inside DOG_FLEE_RADIUS (120 px).
 *
 * DOG_PATROL_OFFSET      — gap between pen ring and dog's patrol path.
 *                          40 px keeps the dog visible just outside the
 *                          ring without risking accidentally entering it.
 *
 * DOG_PATROL_OMEGA       — patrol angular speed (rad/s).
 *                          0.5 rad/s → ~12.6 s for one full lap; slow
 *                          and watchful, not frantic.
 */
#define DOG_SPEED              180.0f
#define DOG_APPROACH_OFFSET     60.0f
#define DOG_PATROL_OFFSET       40.0f
#define DOG_PATROL_OMEGA         0.5f

/* Continuous-chaos mode ('c' key): tiny random nudges per tick.
 * RANDOM_NUDGE in px/s², applied as Δv = RANDOM_NUDGE · dt each tick. */
#define RANDOM_NUDGE  120.0f

/* Timing primitives (verbatim from framework.c). */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/* clock_ns — monotonic wall-clock in nanoseconds.  CLOCK_MONOTONIC never
 * goes backward (unlike CLOCK_REALTIME), so dt = clock_ns() - prev is
 * always non-negative regardless of NTP or DST jumps. */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* clock_sleep_ns — sleep exactly ns nanoseconds; ns ≤ 0 returns
 * immediately so the caller never has to bounds-check the budget. */
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
 * color_init — register all ncurses colour pairs.
 *
 * Background = -1 (terminal default) so the demo respects the user's
 * theme.  All foreground tints sit in the bright half of the 256-colour
 * cube (≥ 33) per the project palette-brightness rule.  HUD pairs use
 * the canonical bright yellow + cyan combo regardless of theme.
 *
 *   PAIR_PEN_RING   220 — warm yellow ring (visible against grass/black)
 *   PAIR_SHEEP_CALM 255 — near-white grazing sheep
 *   PAIR_SHEEP_FLEE 196 — pure red panicking sheep
 *   PAIR_DOG         33 — dodger blue border collie
 *   PAIR_HUD        226 — bright yellow status
 *   PAIR_HINT        51 — bright cyan key hint
 */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_PEN_RING,    220, -1);
        init_pair(PAIR_SHEEP_CALM,  255, -1);
        init_pair(PAIR_SHEEP_FLEE,  196, -1);
        init_pair(PAIR_DOG,          33, -1);
        init_pair(PAIR_HUD,         226, -1);
        init_pair(PAIR_HINT,         51, -1);
    } else {
        init_pair(PAIR_PEN_RING,    COLOR_YELLOW,  -1);
        init_pair(PAIR_SHEEP_CALM,  COLOR_WHITE,   -1);
        init_pair(PAIR_SHEEP_FLEE,  COLOR_RED,     -1);
        init_pair(PAIR_DOG,         COLOR_CYAN,    -1);
        init_pair(PAIR_HUD,         COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,        COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords + Vec2                                                      */
/* ===================================================================== */

/*
 * Two coordinate spaces:
 *   PIXEL space — physics; isotropic; 1 unit = 1 sub-pixel.
 *   CELL space  — terminal rows/cols where drawing happens.
 * Only the draw step converts pixel→cell via px_to_cell_x/y.
 */
static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/*
 * px_to_cell_x/y — round to nearest cell.  Adding 0.5 before flooring is
 * "round half up" — deterministic and symmetric, unlike roundf() which
 * uses banker's rounding and can flicker on exact half-boundaries.
 */
static inline int px_to_cell_x(float px) { return (int)floorf(px / (float)CELL_W + 0.5f); }
static inline int px_to_cell_y(float py) { return (int)floorf(py / (float)CELL_H + 0.5f); }

/* Vec2 — 2-D float vector in pixel space.  Tiny inline helpers for
 * readability; the compiler folds them to register-only ops at -O2. */
typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)        { return (Vec2){x, y}; }
static inline Vec2  v2add(Vec2 a, Vec2 b)        { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)        { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)     { return v2(v.x*s, v.y*s); }
static inline float v2len(Vec2 v)                { return sqrtf(v.x*v.x + v.y*v.y); }

/* v2norm — unit vector; returns zero vector if input is near-zero so
 * coincident-point divides never blow up. */
static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0, 0);
}

/* v2clamp_len — cap magnitude to max_len, preserving direction. */
static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* randf — uniform float in [0, 1].  Used for sheep spawn jitter and
 * the scatter angle perturbation. */
static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/* ===================================================================== */
/* §5  sheep                                                              */
/* ===================================================================== */

/*
 * Sheep — one prey agent.
 *
 * Field groups:
 *   - kinematic state (pos, prev_pos, vel) is rewritten every tick.
 *   - the `fleeing` flag is a cheap render hint set each tick by the
 *     flee force.  It encodes the dog's influence on this sheep so
 *     scene_draw can pick the right glyph + colour without recomputing
 *     the flee distance.
 */
typedef struct {
    /* kinematic state — updated every tick */
    Vec2 pos;          /* current position, pixel space         */
    Vec2 prev_pos;     /* position at start of tick — alpha lerp */
    Vec2 vel;          /* velocity, pixels per second           */

    /* render hint — set each tick by sheep_step's flee force */
    bool fleeing;
} Sheep;

/*
 * sheep_spawn_in_pen — uniform random spawn inside a disc of radius
 * 0.7 · pen_r centred at pen_centre.  Rejection sampling on a unit
 * square ensures uniform density (a polar sample r·(cos θ, sin θ)
 * with uniform r, θ would cluster sheep at the pen centre).
 *
 * The 0.7 factor leaves a ~30 % safety margin so spawned sheep are
 * comfortably inside the visible ring on frame one — not riding the
 * boundary where a tiny separation force could push them outside.
 */
static void sheep_spawn_in_pen(Sheep *s, Vec2 pen_centre, float pen_r)
{
    float dx, dy;
    do {
        dx = randf() * 2.0f - 1.0f;       /* uniform in [-1, 1] */
        dy = randf() * 2.0f - 1.0f;
    } while (dx*dx + dy*dy > 1.0f);       /* keep only points inside unit disc */

    s->pos      = v2(pen_centre.x + dx * pen_r * 0.7f,
                     pen_centre.y + dy * pen_r * 0.7f);
    s->prev_pos = s->pos;

    /* Small initial graze velocity in a random direction so the spawn
     * doesn't look frozen on frame one. */
    float ang = randf() * 2.0f * (float)M_PI;
    s->vel    = v2(cosf(ang) * SHEEP_GRAZE_SPEED * 0.5f,
                   sinf(ang) * SHEEP_GRAZE_SPEED * 0.5f);
    s->fleeing = false;
}

/* ── sheep steering forces ──────────────────────────────────────────── *
 * Each force returns a Vec2 in px/s² (logical acceleration).  scene_tick
 * sums them with weights and integrates into vel.
 */

/*
 * sheep_separate — push this sheep away from each neighbour within
 * SHEEP_SEP_RADIUS.  Linear falloff: touching → strong push, edge → ~0.
 *
 * Why this shape: linear falloff makes sheep settle into a comfortable
 * spacing rather than a hard exclusion shell.  At inner-shell distances
 * the push is small enough that other forces (cohesion, pen pull) can
 * still influence the sheep.
 */
static Vec2 sheep_separate(const Sheep *flock, int n, int self)
{
    Vec2 force = v2(0, 0);
    Vec2 me    = flock[self].pos;
    for (int i = 0; i < n; i++) {
        if (i == self) continue;
        Vec2  away = v2sub(me, flock[i].pos);
        float d    = v2len(away);
        if (d < SHEEP_SEP_RADIUS && d > 0.001f) {
            float strength = (SHEEP_SEP_RADIUS - d) / SHEEP_SEP_RADIUS;
            force = v2add(force, v2scale(v2norm(away), strength * SHEEP_GRAZE_SPEED));
        }
    }
    return force;
}

/*
 * sheep_cohere — gentle pull toward the mean position of neighbours
 * within SHEEP_COH_RADIUS.  Returned force has magnitude
 * SHEEP_GRAZE_SPEED so cohesion competes on a graze-scale, not a
 * panic-scale.
 *
 * Returns zero when the sheep is alone in its perception disc.
 */
static Vec2 sheep_cohere(const Sheep *flock, int n, int self)
{
    Vec2 sum   = v2(0, 0);
    int  count = 0;
    Vec2 me    = flock[self].pos;
    for (int i = 0; i < n; i++) {
        if (i == self) continue;
        if (v2len(v2sub(flock[i].pos, me)) < SHEEP_COH_RADIUS) {
            sum = v2add(sum, flock[i].pos);
            count++;
        }
    }
    if (count == 0) return v2(0, 0);
    Vec2 centroid = v2scale(sum, 1.0f / (float)count);
    Vec2 desired  = v2norm(v2sub(centroid, me));
    return v2scale(desired, SHEEP_GRAZE_SPEED);
}

/*
 * sheep_flee_dog — repulsion from the dog with linear falloff inside
 * DOG_FLEE_RADIUS, zero outside.  Magnitude scales toward
 * SHEEP_FLEE_SPEED at the wall (d → 0).
 *
 * Returns zero magnitude when the dog is outside the disc — the caller
 * uses |force| > 0 as the trigger to set sheep->fleeing for rendering.
 */
static Vec2 sheep_flee_dog(Vec2 sheep_pos, Vec2 dog_pos)
{
    Vec2  away = v2sub(sheep_pos, dog_pos);
    float d    = v2len(away);
    if (d > DOG_FLEE_RADIUS || d < 0.001f) return v2(0, 0);
    float strength = (DOG_FLEE_RADIUS - d) / DOG_FLEE_RADIUS;
    return v2scale(v2norm(away), strength * SHEEP_FLEE_SPEED);
}

/*
 * sheep_pen_pull — gentle inward force, zero when the sheep is inside
 * the pen.  Magnitude is SHEEP_GRAZE_SPEED (graze-scale, not panic) so
 * this never overrides the dog flee force; it just keeps sheep that
 * have wandered out from drifting forever.
 */
static Vec2 sheep_pen_pull(Vec2 sheep_pos, Vec2 pen_centre, float pen_r)
{
    Vec2  inward = v2sub(pen_centre, sheep_pos);
    float d      = v2len(inward);
    if (d < pen_r) return v2(0, 0);
    return v2scale(v2norm(inward), SHEEP_GRAZE_SPEED);
}

/* ===================================================================== */
/* §6  dog — Strömbom-style controller                                    */
/* ===================================================================== */

/*
 * DogMode — which behaviour the controller picked this tick.
 *
 *   DOG_PATROL  : every sheep is inside (pen_r + tolerance).  Walk a
 *                 slow circle around the pen so we stay visible and
 *                 reactive.
 *   DOG_COLLECT : at least one sheep is beyond pen_r + tolerance.
 *                 Position the dog directly outside the WORST outlier
 *                 relative to the pen so the sheep flees inward.
 */
typedef enum {
    DOG_PATROL = 0,
    DOG_COLLECT,
} DogMode;

/*
 * Dog — one border collie.
 *
 * Field groups:
 *   - kinematic state (pos, prev_pos, vel) is rewritten every tick.
 *   - controller output (target) is rewritten every tick by
 *     dog_decide_target; dog_step then steers toward it.
 *   - controller introspection (mode, target_sheep, patrol_phase) is
 *     read by the HUD and the renderer; not consumed by physics.
 */
typedef struct {
    /* kinematic state */
    Vec2 pos;
    Vec2 prev_pos;
    Vec2 vel;

    /* controller output: where the dog wants to be this tick */
    Vec2 target;

    /* controller introspection (for HUD) */
    DogMode mode;
    int     target_sheep;   /* sheep index in COLLECT mode; -1 in PATROL */
    float   patrol_phase;   /* PATROL mode angle around pen, radians     */
} Dog;

/*
 * dog_decide_target — Strömbom-style mode selection.
 *
 * Step 1: scan every sheep, find the index with the largest distance
 *         from pen centre.
 * Step 2: if that distance exceeds pen_r + PEN_TOLERANCE, the dog must
 *         COLLECT this sheep.  Place the dog at
 *           target = sheep_pos + DOG_APPROACH_OFFSET · (sheep − pen) / |sheep − pen|
 *         i.e. on the same line as pen→sheep, but DOG_APPROACH_OFFSET
 *         further out from the pen.  The sheep, fleeing the dog
 *         radially, will move along that line in the pen-ward direction.
 * Step 3: otherwise, switch to PATROL.  Advance patrol_phase by
 *         DOG_PATROL_OMEGA · dt and place the dog at
 *           target = pen_centre + (pen_r + DOG_PATROL_OFFSET) · (cos φ, sin φ)
 *
 * Reads only from flock[]; never writes back.
 */
static void dog_decide_target(Dog *d, const Sheep *flock, int n,
                              Vec2 pen_centre, float pen_r, float dt)
{
    /* Step 1: find worst outlier */
    float worst_dist = -1.0f;
    int   worst_i    = -1;
    for (int i = 0; i < n; i++) {
        float dist = v2len(v2sub(flock[i].pos, pen_centre));
        if (dist > worst_dist) {
            worst_dist = dist;
            worst_i    = i;
        }
    }

    /* Step 2: COLLECT if the worst outlier is meaningfully outside pen */
    if (worst_i >= 0 && worst_dist > pen_r + PEN_TOLERANCE) {
        d->mode         = DOG_COLLECT;
        d->target_sheep = worst_i;

        Vec2 from_pen = v2sub(flock[worst_i].pos, pen_centre);
        Vec2 dir      = v2norm(from_pen);
        d->target     = v2add(flock[worst_i].pos,
                              v2scale(dir, DOG_APPROACH_OFFSET));
        return;
    }

    /* Step 3: all sheep home — patrol the perimeter */
    d->mode         = DOG_PATROL;
    d->target_sheep = -1;
    d->patrol_phase += DOG_PATROL_OMEGA * dt;

    Vec2 patrol_dir = v2(cosf(d->patrol_phase), sinf(d->patrol_phase));
    d->target       = v2add(pen_centre,
                            v2scale(patrol_dir, pen_r + DOG_PATROL_OFFSET));
}

/*
 * dog_step — move the dog one tick toward d->target at DOG_SPEED.
 *
 * No acceleration, no inertia: the dog snaps its velocity to point at
 * the target each tick.  This is acceptable because (a) the target
 * itself moves smoothly (sheep move smoothly, patrol phase is C¹),
 * and (b) it keeps the algorithm visibly "geometric" — the dog goes
 * where the algorithm tells it to go, no internal dynamics to debug.
 */
static void dog_step(Dog *d, float dt)
{
    Vec2  to_target = v2sub(d->target, d->pos);
    float dist      = v2len(to_target);

    Vec2 desired = (dist > 0.001f)
                 ? v2scale(v2norm(to_target), DOG_SPEED)
                 : v2(0, 0);

    d->vel      = desired;
    d->prev_pos = d->pos;
    d->pos      = v2add(d->pos, v2scale(d->vel, dt));
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Scene — pen + sheep pool + dog + world dimensions.
 *
 * Field groups:
 *   - pen geometry (set by scene_init / app_do_resize).
 *   - sheep pool (first n_sheep slots active; rest pre-spawned for
 *     instant '+' reveal).
 *   - the dog (singleton).
 *   - user mode flags (paused, continuous_chaos).
 *   - HUD-only derived state (in_pen_count, recomputed each tick).
 *   - world dimensions in pixels (refreshed each tick from cols/rows).
 */
typedef struct {
    /* pen geometry */
    Vec2  pen_centre;
    float pen_radius;

    /* sheep pool — first `n_sheep` slots are active */
    Sheep sheep[SHEEP_MAX];
    int   n_sheep;

    /* the dog */
    Dog   dog;

    /* user mode flags */
    bool  paused;
    bool  continuous_chaos;

    /* HUD-only derived state — recomputed in scene_tick */
    int   in_pen_count;

    /* world dimensions in pixels — refreshed each tick */
    float world_w, world_h;
} Scene;

/*
 * scene_init — fresh scene with sheep evenly scattered inside the pen
 * and the dog parked at the right edge of the patrol ring.
 *
 * Pen centre = world centre.  Pen radius = PEN_RADIUS_FRAC × min(w, h).
 * Using min(w, h) (not max, not w·h) keeps the pen circular in pixel
 * space regardless of terminal aspect ratio.
 */
static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->world_w = pw(cols);
    s->world_h = ph(rows);
    s->n_sheep = SHEEP_DEFAULT;

    s->pen_centre = v2(s->world_w * 0.5f, s->world_h * 0.5f);
    float min_dim = (s->world_w < s->world_h) ? s->world_w : s->world_h;
    s->pen_radius = min_dim * PEN_RADIUS_FRAC;

    /* Pre-spawn the entire pool so '+' reveals already-placed sheep
     * without stuttering the simulation while they randomise. */
    for (int i = 0; i < SHEEP_MAX; i++)
        sheep_spawn_in_pen(&s->sheep[i], s->pen_centre, s->pen_radius);

    /* Dog starts at the east edge of the patrol circle. */
    s->dog.pos          = v2(s->pen_centre.x + s->pen_radius + DOG_PATROL_OFFSET,
                             s->pen_centre.y);
    s->dog.prev_pos     = s->dog.pos;
    s->dog.target       = s->dog.pos;
    s->dog.vel          = v2(0, 0);
    s->dog.mode         = DOG_PATROL;
    s->dog.target_sheep = -1;
    s->dog.patrol_phase = 0.0f;
}

/*
 * scene_scatter — apply an outward radial impulse to every sheep, with
 * per-sheep angular jitter so the herd fans out instead of firing along
 * straight rays.
 *
 * For each sheep:
 *   1. Compute outward direction = normalise(pos − pen_centre).
 *      If the sheep is at (or very near) the centre, fall back to a
 *      uniform random direction so divide-by-zero doesn't strand it.
 *   2. Rotate the direction by a random angle in [-SCATTER_JITTER,
 *      +SCATTER_JITTER] radians.
 *   3. Add SCATTER_IMPULSE × strength_mul × jittered_dir to vel.
 *
 * strength_mul: 1.0 for normal SPACE, 2.0 for 'S' mega-scatter.
 */
static void scene_scatter(Scene *s, float strength_mul)
{
    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh = &s->sheep[i];

        /* Step 1: outward direction (or random if at centre) */
        Vec2  outward = v2sub(sh->pos, s->pen_centre);
        float d       = v2len(outward);
        Vec2  dir;
        if (d > 0.001f) {
            dir = v2norm(outward);
        } else {
            float ang = randf() * 2.0f * (float)M_PI;
            dir = v2(cosf(ang), sinf(ang));
        }

        /* Step 2: rotate by random jitter */
        float jitter = (randf() * 2.0f - 1.0f) * SCATTER_JITTER;
        float ca = cosf(jitter), sa = sinf(jitter);
        Vec2  jdir = v2(dir.x * ca - dir.y * sa,
                        dir.x * sa + dir.y * ca);

        /* Step 3: apply impulse */
        sh->vel = v2add(sh->vel,
                        v2scale(jdir, SCATTER_IMPULSE * strength_mul));
    }
}

/*
 * scene_apply_chaos — sprinkle a tiny random velocity nudge into every
 * sheep this tick.  Used when the user has toggled continuous-chaos
 * mode ('c' key); the nudge is small enough that a single tick is
 * imperceptible, but accumulated over hundreds of ticks the herd
 * never quite settles — the dog stays in COLLECT mode indefinitely,
 * which is the visual point of the mode.
 *
 * Magnitude per tick: |Δv| = RANDOM_NUDGE · dt = 120 · 1/60 = 2 px/s.
 * Direction: uniform random angle.
 */
static void scene_apply_chaos(Scene *s, float dt)
{
    for (int i = 0; i < s->n_sheep; i++) {
        float ang = randf() * 2.0f * (float)M_PI;
        float mag = RANDOM_NUDGE * dt;
        s->sheep[i].vel = v2add(s->sheep[i].vel,
                                v2(cosf(ang) * mag, sinf(ang) * mag));
    }
}

/*
 * scene_step_sheep — two-stage sheep update.
 *
 * The two-stage pattern is essential:
 *   Stage A reads ALL sheep positions, computes new_vel[i] for every i.
 *   Stage B writes new_vel back, integrates pos, clamps to world.
 * Without two stages, sheep[5] would react to sheep[0..4]'s NEW
 * positions but sheep[0..4] would react to sheep[5..29]'s OLD
 * positions — a subtle index-order drift visible over many ticks.
 *
 * Side effect: also sets each sheep's `fleeing` flag (rendering hint)
 * and counts how many sheep ended up inside the pen this tick (HUD).
 */
static void scene_step_sheep(Scene *s, float dt)
{
    /* Stage A: read OLD positions, compute new velocities. */
    Vec2 new_vel[SHEEP_MAX];
    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh    = &s->sheep[i];
        Vec2   force = v2(0, 0);

        force = v2add(force, v2scale(sheep_separate(s->sheep, s->n_sheep, i),
                                     W_SHEEP_SEP));
        force = v2add(force, v2scale(sheep_cohere  (s->sheep, s->n_sheep, i),
                                     W_SHEEP_COH));

        Vec2 flee = sheep_flee_dog(sh->pos, s->dog.pos);
        sh->fleeing = (v2len(flee) > 0.001f);
        force = v2add(force, v2scale(flee, W_SHEEP_FLEE));

        force = v2add(force,
                      v2scale(sheep_pen_pull(sh->pos, s->pen_centre, s->pen_radius),
                              W_PEN_PULL));

        Vec2 v = v2add(sh->vel, v2scale(force, dt));
        v = v2scale(v, SHEEP_DAMPING);              /* per-tick drag */
        v = v2clamp_len(v, SHEEP_MAX_SPEED);        /* hard speed cap */
        new_vel[i] = v;
    }

    /* Stage B: commit velocities, integrate position, clamp; count in-pen. */
    int in_pen = 0;
    for (int i = 0; i < s->n_sheep; i++) {
        Sheep *sh = &s->sheep[i];
        sh->vel      = new_vel[i];
        sh->prev_pos = sh->pos;
        sh->pos      = v2add(sh->pos, v2scale(sh->vel, dt));

        if (sh->pos.x < 0.0f)        sh->pos.x = 0.0f;
        if (sh->pos.x >= s->world_w) sh->pos.x = s->world_w - 1.0f;
        if (sh->pos.y < 0.0f)        sh->pos.y = 0.0f;
        if (sh->pos.y >= s->world_h) sh->pos.y = s->world_h - 1.0f;

        if (v2len(v2sub(sh->pos, s->pen_centre)) <= s->pen_radius)
            in_pen++;
    }
    s->in_pen_count = in_pen;
}

/*
 * scene_step_dog — run the controller, integrate the dog, clamp to world.
 *
 * Decomposed out of scene_tick so the orchestrator stays linear and
 * each step's purpose is named.  The clamp at the end is a safety net
 * for the rare case where a single tick of DOG_SPEED could push the
 * dog past the screen edge (e.g. a chased sheep escapes to a corner).
 */
static void scene_step_dog(Scene *s, float dt)
{
    dog_decide_target(&s->dog, s->sheep, s->n_sheep,
                      s->pen_centre, s->pen_radius, dt);
    dog_step(&s->dog, dt);

    if (s->dog.pos.x < 0.0f)        s->dog.pos.x = 0.0f;
    if (s->dog.pos.x >= s->world_w) s->dog.pos.x = s->world_w - 1.0f;
    if (s->dog.pos.y < 0.0f)        s->dog.pos.y = 0.0f;
    if (s->dog.pos.y >= s->world_h) s->dog.pos.y = s->world_h - 1.0f;
}

/*
 * scene_tick — one fixed-step physics update; thin orchestrator.
 *
 * Order matters:
 *   1. Continuous chaos (if enabled) — perturb sheep velocities.
 *   2. Dog: controller picks target, then dog steps.  Done BEFORE
 *      sheep step so sheep_flee sees the dog's NEW position
 *      (sheep flee from where the dog IS, not where it WAS).
 *   3. Sheep: two-stage update.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    if (s->continuous_chaos) scene_apply_chaos(s, dt);
    scene_step_dog  (s, dt);
    scene_step_sheep(s, dt);
}

/* ── render ──────────────────────────────────────────────────────────── */

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double cast
 * prevents sign-extension on character values > 127 (per CLAUDE.md
 * "Common ncurses Bugs").  Off-screen cells are silently dropped.
 */
static void mark_cell(WINDOW *w, int cx, int cy, char ch,
                      int pair, attr_t attr, int cols, int rows)
{
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) return;
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, cy, cx, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * draw_pen — render the pen boundary as a dashed circle.
 *
 * Sample N points evenly around the circle in PIXEL space, convert
 * each sample to a cell, alternate '*' and '.' by sample index for a
 * dashed look.  Drawing in pixel space (not cell space) means the
 * circle stays circular regardless of terminal aspect ratio — a
 * cell-space circle would appear as a vertically-squished oval.
 *
 * N samples chosen so adjacent samples are ~4 px apart along the arc:
 *   N = ceil(2π · r / 4)
 * For r = 69 px (default), N ≈ 109.
 */
static void draw_pen(WINDOW *w, Vec2 c, float r, int cols, int rows)
{
    int n = (int)ceilf(2.0f * (float)M_PI * r / 4.0f);
    if (n < 16) n = 16;
    for (int i = 0; i < n; i++) {
        float a   = (float)i / (float)n * 2.0f * (float)M_PI;
        float px  = c.x + r * cosf(a);
        float py  = c.y + r * sinf(a);
        char  ch  = (i & 1) ? '.' : '*';   /* alternate for dashed look */
        mark_cell(w, px_to_cell_x(px), px_to_cell_y(py),
                  ch, PAIR_PEN_RING, A_BOLD, cols, rows);
    }
}

/*
 * scene_draw — paint the frame in painter's order:
 *   1. Pen ring (bottom layer).
 *   2. Sheep at alpha-interpolated positions ('o' calm / 'O' fleeing).
 *   3. Dog last so it always reads above the sheep.
 *
 * alpha ∈ [0, 1) is the fixed-step accumulator's leftover fraction; it
 * lerps prev_pos → pos so motion stays smooth at any sim/render rate
 * combination.
 */
static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha)
{
    /* 1. pen ring */
    draw_pen(w, s->pen_centre, s->pen_radius, cols, rows);

    /* 2. sheep */
    for (int i = 0; i < s->n_sheep; i++) {
        const Sheep *sh = &s->sheep[i];
        Vec2 dp = v2add(sh->prev_pos,
                        v2scale(v2sub(sh->pos, sh->prev_pos), alpha));

        char   ch   = sh->fleeing ? 'O' : 'o';
        int    pair = sh->fleeing ? PAIR_SHEEP_FLEE : PAIR_SHEEP_CALM;
        attr_t attr = sh->fleeing ? A_BOLD : A_NORMAL;
        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  ch, pair, attr, cols, rows);
    }

    /* 3. dog */
    {
        const Dog *d = &s->dog;
        Vec2 dp = v2add(d->prev_pos,
                        v2scale(v2sub(d->pos, d->prev_pos), alpha));
        mark_cell(w, px_to_cell_x(dp.x), px_to_cell_y(dp.y),
                  '&', PAIR_DOG, A_BOLD, cols, rows);
    }
}

/* ===================================================================== */
/* §8  app — screen + signals + main loop                                 */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);    /* getch returns ERR if no key — non-blocking */
    keypad(stdscr, TRUE);
    typeahead(-1);             /* prevent input polling from interrupting output */
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();                 /* force ncurses to re-read LINES/COLS */
    getmaxyx(stdscr, s->rows, s->cols);
}

static const char *DOG_MODE_NAMES[] = { "PATROL ", "COLLECT" };

/*
 * screen_draw — compose one frame: scene + HUD bars.
 *
 * Order matters: scene_draw first so HUD always over-stamps any glyph
 * that landed on the same cell.  Top-right uses PAIR_HUD (bright
 * yellow) with A_BOLD; bottom-left uses PAIR_HINT (bright cyan) with
 * A_BOLD.  The hint bar uses A_BOLD (NEVER A_DIM) so it stays readable
 * against any animation behind it (per the project HUD spec).
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    /* Top-right status — PAIR_HUD bright yellow, A_BOLD */
    char buf[HUD_COLS + 1];
    const char *suffix = sc->paused           ? "  PAUSED"
                       : sc->continuous_chaos ? "  CHAOS"
                                              : "";
    snprintf(buf, sizeof buf,
             " %5.1f fps  sim:%3d Hz  sheep:%d/%d in pen  dog:%s%s ",
             fps, sim_fps,
             sc->in_pen_count, sc->n_sheep,
             DOG_MODE_NAMES[sc->dog.mode],
             suffix);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:scatter  S:mega-scatter  c:chaos  p:pause  r:reset  +/-:sheep ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App + signal handlers ──────────────────────────────────────────── */

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

/*
 * app_do_resize — handle a pending SIGWINCH.
 *
 * Re-read terminal dimensions, recompute pen geometry from the new
 * size (so the pen stays centred and proportional), clamp every sheep
 * and the dog into the new world box.
 */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    Scene *sc = &app->scene;

    sc->world_w    = pw(app->screen.cols);
    sc->world_h    = ph(app->screen.rows);
    sc->pen_centre = v2(sc->world_w * 0.5f, sc->world_h * 0.5f);
    float min_dim  = (sc->world_w < sc->world_h) ? sc->world_w : sc->world_h;
    sc->pen_radius = min_dim * PEN_RADIUS_FRAC;

    /* Clamp every active sheep into the new world bounds. */
    for (int i = 0; i < sc->n_sheep; i++) {
        Sheep *sh = &sc->sheep[i];
        if (sh->pos.x >= sc->world_w) sh->pos.x = sc->world_w - 1.0f;
        if (sh->pos.y >= sc->world_h) sh->pos.y = sc->world_h - 1.0f;
        sh->prev_pos = sh->pos;
    }
    if (sc->dog.pos.x >= sc->world_w) sc->dog.pos.x = sc->world_w - 1.0f;
    if (sc->dog.pos.y >= sc->world_h) sc->dog.pos.y = sc->world_h - 1.0f;
    sc->dog.prev_pos = sc->dog.pos;

    app->need_resize = 0;
}

/*
 * app_handle_key — dispatch a single keypress; return false to quit.
 *
 *   q / Q / ESC    quit
 *   space          single scatter pulse
 *   S              mega-scatter (2x impulse)
 *   c / C          toggle continuous-chaos sprinkle mode
 *   p / P          pause / resume
 *   r / R          reset (regather all sheep into the pen)
 *   + / =          add SHEEP_STEP sheep (cap SHEEP_MAX)
 *   -              remove SHEEP_STEP sheep (floor SHEEP_MIN)
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':  scene_scatter(sc, 1.0f);                 break;
    case 'S':  scene_scatter(sc, 2.0f);                 break;
    case 'c': case 'C': sc->continuous_chaos = !sc->continuous_chaos; break;
    case 'p': case 'P': sc->paused           = !sc->paused;           break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);
        break;
    case '+': case '=':
        sc->n_sheep += SHEEP_STEP;
        if (sc->n_sheep > SHEEP_MAX) sc->n_sheep = SHEEP_MAX;
        break;
    case '-':
        sc->n_sheep -= SHEEP_STEP;
        if (sc->n_sheep < SHEEP_MIN) sc->n_sheep = SHEEP_MIN;
        break;
    default: break;
    }
    return true;
}

/*
 * main — the game loop.  Structure identical to the project framework:
 *   ① resize check → ② measure dt → ③ fixed-step physics accumulator →
 *   ④ alpha → ⑤ fps counter → ⑥ frame cap (sleep BEFORE render) →
 *   ⑦ draw + present → ⑧ drain input.
 *
 * Frame cap uses `clock_ns() − frame_start` (no `+ dt`).  Adding dt
 * back into elapsed cancels the cap; sleep is always 0 and CPU pegs
 * at 100 %.
 */
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

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        int64_t frame_start = clock_ns();

        /* ① resize */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ② dt */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ③ fixed-step accumulator */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* ④ alpha */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ⑤ fps counter (500 ms window) */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ⑥ frame cap — sleep before render to keep terminal I/O off
         * the next frame's budget.  elapsed is wall time spent on
         * physics + accounting since frame_start; sleep the remainder. */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ⑦ draw + present */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ⑧ drain input */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
