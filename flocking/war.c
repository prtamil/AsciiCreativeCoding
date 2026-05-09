/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * war.c — two-faction battle: melee + archers + 6 switchable strategies
 *
 * DEMO: Two armies — GONDOR (right side, cyan letters + green '@'
 *       archers) and MORDOR (left side, red letters + orange '%'
 *       archers) — clash in the middle of the terminal.  Each warrior
 *       runs a four-state machine (ADVANCE / COMBAT / FLEE / DEAD).
 *       Archers fire real '-' projectile arrows that travel across
 *       the screen and deal damage on contact.  Press 1-6 to live-
 *       switch between six battle strategies (Standard, Berserker,
 *       Shield Wall, Guerrilla, Archer Focus, Chaos), each tuning
 *       all engagement / rout / shoot parameters.
 *
 * Study alongside: flocking/flocking.c (the steering forces both
 *                                       unit types use)
 *                  flocking/shepherd.c (Strömbom-style state machine)
 *
 * Section map:
 *   §1 config   — strategy presets + global combat constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — faction palette + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect bridge + Vec2 helpers
 *   §5 entity   — Warrior + Arrow structs, spawn, integrator
 *   §6 combat   — steering forces, melee/archer state machines
 *   §7 scene    — Scene = pool + arrows; tick + arrow physics + draw
 *   §8 app      — signals, resize, main game loop
 *
 * Keys:
 *   q / ESC    quit                       space    pause / resume
 *   r / R      reset                      g / m    add Gondor / Mordor
 *   1-6        switch strategy (live, no reset needed)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/war.c -o war -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two cooperating systems share the screen.
 *
 *                  STEERING — every warrior is a Reynolds-style
 *                  particle.  Each tick it sums a weighted force
 *                  vector (seek toward target / centroid, separate
 *                  from same-faction allies, flee from threat),
 *                  integrates it into velocity, clamps the
 *                  magnitude, and integrates position.
 *
 *                  STATE MACHINE — each warrior is in one of four
 *                  states: ADVANCE (march toward enemy mass),
 *                  COMBAT (engaged: brawl or shoot), FLEE (route),
 *                  or DEAD (corpse, briefly visible).  Transitions
 *                  fire on engage range, HP threshold, target
 *                  death, or rally timer expiry.  Both melee and
 *                  archer use the same four states; the per-state
 *                  forces differ.
 *
 *                  ARROW PROJECTILES — archers don't deal instant
 *                  damage.  Each shot spawns an Arrow in a flat
 *                  pool: pos, vel, target_idx, faction.  The
 *                  arrows_tick step advances every active arrow,
 *                  detects hit (within ARROW_HIT_DIST of target),
 *                  marks miss (off-screen), and compacts the pool.
 *
 *                  STRATEGIES — six StrategyParams presets each
 *                  configure ~16 parameters (ranges, speeds, force
 *                  weights, attack intervals).  All combat code
 *                  reads through `*g_sp`, so a key-press switches
 *                  the active preset and the next tick fights
 *                  under the new rules — no reset needed.
 *
 * Data-structure : Scene owns one flat pool[] of Warriors (both
 *                  factions interleaved by spawn order; dead
 *                  warriors keep their slot for simple corpse
 *                  rendering) and one arrow[] pool that compacts
 *                  inactive entries each tick.  Warrior carries
 *                  pos / vel / faction / unit_type / hp / state /
 *                  the per-unit timers; Arrow carries just the
 *                  projectile pose + its target.
 *
 * Rendering      : Painter's order — arrows '-' (under) → corpses
 *                  '.' dim → living warriors with HP-driven
 *                  attributes (A_BOLD full HP, A_DIM last HP,
 *                  A_BLINK while routing) → '*' arrow-hit flash
 *                  briefly overrides the warrior glyph → optional
 *                  victory banner.  Sub-tick alpha lerp on
 *                  prev_pos → pos for smooth motion.  Every glyph
 *                  stamp goes through a mark_cell() helper that
 *                  performs the (chtype)(unsigned char) cast and
 *                  bounds-check.
 *
 * Performance    : Steering and target-search are O(N²); at
 *                  N = 120 warriors per faction (POOL_MAX = 160
 *                  total cap including corpses), 120·119 = 14 280
 *                  distance checks per warrior per tick is ~0.25 M
 *                  per tick, ~15 M/s at 60 Hz — sub-millisecond.
 *                  Arrow loop is O(N_arrows), capped at
 *                  ARROW_POOL_MAX = 80.
 *
 * References     : Reynolds, "Steering Behaviors for Autonomous
 *                    Characters," 1999 (red3d.com/cwr/steer/) —
 *                    the source of every steering force used here.
 *                  Reynolds, "Flocks, Herds, and Schools: A
 *                    Distributed Behavioral Model," SIGGRAPH 1987 —
 *                    the original boid-style separation/seek.
 *                  Wikipedia, "Game artificial intelligence (state
 *                    machines)" — survey of the ADVANCE/COMBAT/
 *                    FLEE/DEAD pattern used by every behaviour
 *                    branch in §6.
 *                  Tolkien, *The Lord of the Rings* — flavour for
 *                    the Gondor / Mordor faction names; gameplay
 *                    is fictional Reynolds-AI, not lore-accurate.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every warrior is a particle with a brain that picks one of four
 * verbs each tick: ADVANCE (head toward enemy mass), COMBAT (fight
 * the locked target or shoot at the nearest), FLEE (sprint away),
 * DEAD (be a corpse).  The "brain" is just a switch on the warrior's
 * current state plus a few distance/HP checks; the "body" sums
 * steering forces, clamps to max_speed, and integrates.  Arrows are
 * separate particles that fly between archer and victim — damage
 * lands on hit, not on shoot.  Strategies just rebind the constants.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a chess piece that has its own little finite-state
 * controller and steering rig stapled on.  At every tick it asks
 * itself: "where is the enemy mass, and how close is the nearest
 * one?", then reads its own HP, then picks one of four moods, then
 * moves accordingly.  All 70+ pieces do this in parallel; no
 * central commander, no formation orders.  Archers add a layer:
 * they shoot arrows that travel as their own little flying
 * particles, with their own trajectories and hit-detection.  A
 * strategy preset is just "give every brain a different set of
 * parameter values and watch what emerges."
 *
 * ALGORITHM IN STEPS  (per tick, per warrior)
 * ───────────────────────────────────────────
 *   1. DEAD?            decrement corpse timer, return.
 *   2. hp ≤ 0?          enter DEAD; credit kill to enemy faction.
 *   3. Stale target?    drop target_idx if locked enemy is now dead.
 *   4. Dispatch by unit_type:
 *        UNIT_MELEE  → melee_logic
 *        UNIT_ARCHER → archer_logic
 *
 *   melee_logic switch:
 *     ADVANCE → seek enemy centroid; if any enemy enters
 *               engage_range, lock on, transition to COMBAT.
 *     COMBAT  → seek locked target slowly; deal ATK_DAMAGE every
 *               atk_interval s.  If target dies → ADVANCE.  If hp
 *               ≤ flee_hp → FLEE.
 *     FLEE    → flee from nearest enemy.  If d ≥ safe_range, count
 *               rally_timer; on rally_time → ADVANCE.
 *
 *   archer_logic distance ladder:
 *     d < archer_flee_range   → FLEE (panic).
 *     d ≤ arrow_range         → COMBAT: drift to standoff, fire
 *                                arrow every shoot_interval s.
 *     otherwise               → ADVANCE: head toward standoff
 *                                position behind enemy centroid.
 *
 *   After every warrior, run arrows_tick: advance each arrow by
 *   vel·dt, hit-test against its target (within ARROW_HIT_DIST →
 *   ATK_DAMAGE + flash), miss if off-screen, then compact the pool.
 *
 * KEY FORMULAS
 * ────────────
 *   Seek:        desired = normalize(target − pos) · speed
 *                force   = desired − vel
 *
 *   Flee:        force   = − seek(threat)
 *
 *   Separate (per same-faction ally with 0 < d < sep_radius):
 *                strength = (sep_radius − d) / sep_radius
 *                force   += normalize(self − ally) · strength · speed_advance
 *
 *   Arrow flight:
 *                arrow.pos += arrow.vel · dt
 *                hit if |target.pos − arrow.pos| < ARROW_HIT_DIST
 *                miss if arrow exits world bounds
 *                arrow.vel = normalize(target.pos@spawn − archer.pos)
 *                          · ARROW_TRAVEL_SPD       (set once at shoot)
 *
 *   Pixel→cell aspect bridge:
 *                cx = round(px / CELL_W)        (CELL_W = 8)
 *                cy = round(py / CELL_H)        (CELL_H = 16)
 *
 * WORKED EXAMPLE  (defaults: 35 melee + 12 archers per faction, 80x24)
 * ──────────────────────────────────────────────────────────────────
 *   World box       : 80 × 24 cells = 640 × 384 pixels.
 *   Spawn zones     : Gondor melee in right 60-82 % of world width;
 *                     Gondor archers in 84-97 %.  Mordor mirror.
 *   Per tick (1/60 s):
 *     a STANDARD melee at speed_advance = 55 px/s travels 0.92 px;
 *     a STANDARD archer at archer_speed = 48 px/s travels 0.80 px;
 *     an arrow at ARROW_TRAVEL_SPD = 220 px/s covers 3.67 px ≈ ½ col.
 *   Engagement      : two STANDARD lines closing at 55+55 = 110 px/s
 *                     bridge ~640 px in ≈ 5.8 s — gives archers ~3-4
 *                     volleys before melee contact.
 *   Steering cost   : separation O(N²) per warrior at 47 alive per
 *                     side ≈ 94 total → 94·93 = 8 742 checks/tick
 *                     per warrior, 821 K/s total — sub-millisecond.
 *   Arrows in flight: capped at ARROW_POOL_MAX = 80; over-spawned
 *                     shots are silently dropped.  At
 *                     shoot_interval ≈ 1.8 s × 12 archers/side × 2
 *                     sides ≈ 13 shots/s, well under 80.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   - Mutual annihilation: if both factions reach 0 alive in the
 *     same tick (last warriors trade kills via in-flight arrows),
 *     scene_tick declares MORDOR winner by tiebreak (line 962-963).
 *     Not a fairness concern — it's a vanishingly rare frame.
 *   - Stale target: between melee_logic deciding to attack and the
 *     attack landing, the locked enemy might have died from another
 *     warrior's blow.  warrior_tick clears target_idx when the
 *     locked enemy is DEAD before dispatching to melee_logic.
 *   - Arrow target dies in flight: arrows_tick deactivates the arrow
 *     when target.state == DEAD.  No "homing past death" weirdness.
 *   - Pool grows without compaction: dead warriors keep their slots.
 *     POOL_MAX = 160 caps total spawn count.  Hitting the cap
 *     silently drops further reinforcements.
 *   - Frame cap: do NOT add dt back into elapsed — that cancels the
 *     cap.  Use a `frame_start = clock_ns()` snapshot at the top of
 *     the loop and `elapsed = clock_ns() − frame_start`.
 *
 * HOW TO VERIFY
 * ─────────────
 *   - At startup the two armies stream toward the centre, archers
 *     stop ~110 px short and start firing '-' arrows.  HUD on row 0
 *     shows live counts and kill totals per faction.
 *   - Press 2 (BERSERKER): everyone sprints into the middle, very
 *     dense melee scrum, no fleeing.  Press 3 (SHIELD WALL): ranks
 *     spread out, slow march, archers hammer from distance.  The
 *     visual change should be unmistakable on the next tick.
 *   - Press g / m: a small clump of reinforcements appears at the
 *     spawn zone of that faction.
 *   - Watch for arrow-hit flashes: a warrior briefly shows '*'
 *     (HIT_FLASH_TIME = 0.15 s) when struck.  Arrows visibly travel
 *     across the screen at constant speed regardless of strategy.
 *   - Eventually one side reaches 0 alive and the centre shows the
 *     victory banner; reset with 'r'.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read flocking.c first for steering basics; read
 *      shepherd.c for "agent state machine" pattern. The NEW
 *      LESSONS here are: factions, four-state warrior FSM,
 *      projectile pool, and parameter-driven strategy presets.
 *   2. §6 combat — THE HEART of this file. Read AFTER tutorials
 *      T1-T6 below. Sub-sections:
 *        - steering primitives (seek, flee, separate)
 *        - melee_logic (4-state FSM)
 *        - archer_logic (4-state FSM with shooting)
 *   3. §7 scene — orchestrator + arrows_tick + draw.
 *   4. §5 entity — Warrior, Arrow structs.
 *   5. §1 — strategy presets (six tables of constants).
 *   6. §2-§4, §8 — clock / colour / coords / app loop.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   warrior.faction        F_GONDOR or F_MORDOR.
 *   warrior.unit_type      UNIT_MELEE or UNIT_ARCHER.
 *   warrior.state          ADVANCE / COMBAT / FLEE / DEAD.
 *   warrior.target_idx     index of locked enemy (in COMBAT).
 *   warrior.hp             current hit points; ≤ 0 → DEAD.
 *   warrior.atk_timer      countdown to next melee swing.
 *   warrior.shoot_timer    countdown to next arrow.
 *   warrior.rally_timer    accumulator while in FLEE — when high
 *                          enough, rally back to ADVANCE.
 *   arrow.pos, vel         projectile pose.
 *   arrow.target_idx       which enemy this arrow is heading to
 *                          (index captured at shoot time).
 *   g_sp                   pointer to active StrategyParams. All
 *                          combat code reads constants through
 *                          this so 1-6 keys swap rules live.
 *
 * Background you need
 * ───────────────────
 *   - flocking.c T1-T7 (boid + steering forces).
 *   - shepherd.c T1-T3 (heterogeneous agents + state machines).
 *   - Object pool with ACTIVE flag — the arrow pool compacts
 *     in place rather than allocating.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Real RTS combat math (range/armour/morale models).
 *     Simplified to HP + flat damage.
 *   - Pathfinding / navmesh. Warriors steer reactively; no
 *     A*; no obstacles.
 *   - Networked multiplayer. Single-process simulation.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build a two-faction battle from first
 * principles.
 *
 *   T1  Factions — heterogeneous agents with mirrored behaviour
 *   T2  The 4-state warrior FSM — ADVANCE / COMBAT / FLEE / DEAD
 *   T3  Steering, parameterised — same forces, different weights
 *   T4  Archers — adding a SHOOT action and a projectile pool
 *   T5  Strategy presets — global parameter tables behind a pointer
 *   T6  Why mutual annihilation matters (and how to handle it)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  FACTIONS — HETEROGENEOUS AGENTS WITH MIRRORED BEHAVIOUR
 * ───────────────────────────────────────────────────────────
 * shepherd.c had two AGENT TYPES (sheep, dog). war.c has two
 * FACTIONS — same agent type, opposing allegiance.
 *
 *     enum Faction { F_GONDOR, F_MORDOR };
 *     warrior.faction;
 *
 * Faction tag is just a colour-and-target marker:
 *
 *   - For a Gondor warrior: ALLIES = other Gondor; ENEMIES =
 *     all Mordor. Reverse for Mordor.
 *   - When computing forces, separation only counts ALLIES (so
 *     friendly lines stay tight). Seeking centroid only counts
 *     ENEMIES (so warriors advance toward the enemy mass).
 *
 * Implementation: two faction-filtered loops:
 *
 *     for each ally w of self.faction:
 *       sep += separation_force(self, w)
 *
 *     enemy_centroid = mean({w.pos : w.faction != self.faction})
 *     advance_force = seek(enemy_centroid)
 *
 * Both faction sides run THE SAME CODE — the faction tag just
 * selects which agents go in "allies" and "enemies." Mirroring.
 *
 * The two factions could be three (alliances), four (free-for-
 * all), or any number — same machinery scales. We hard-code 2
 * for the visual clarity of "this side vs that side."
 *
 * T2  THE 4-STATE WARRIOR FSM — ADVANCE / COMBAT / FLEE / DEAD
 * ────────────────────────────────────────────────────────────
 * Warriors are NOT pure boids — they have INTENTIONS that
 * change with context:
 *
 *      ┌─────────┐  enemy in range   ┌─────────┐  hp ≤ flee_hp  ┌──────┐
 *      │ ADVANCE │ ──────────────────│ COMBAT  │ ──────────────►│ FLEE │
 *      │ (seek   │                   │ (fight) │                │      │
 *      │  enemy  │                   └─────────┘                └──┬───┘
 *      │  mass)  │                       ▲                          │
 *      └────┬────┘                       │                          │
 *           │                            │                          │
 *           │ rally_time elapsed         │ target died             ↓
 *           └────────────────────────────┘                       ┌──────┐
 *                                                                │ DEAD │
 *                                                                │      │
 *                                                                └──────┘
 *
 *                                              ↑ hp ≤ 0 from any state
 *
 *   ADVANCE:   seek enemy centroid; if any enemy enters
 *              engage_range, lock target → COMBAT.
 *
 *   COMBAT:    seek locked target slowly; deal damage every
 *              atk_interval seconds. If target dies → ADVANCE
 *              (look for new target). If hp ≤ flee_hp → FLEE.
 *
 *   FLEE:      flee from nearest enemy. Increment rally_timer
 *              each tick; when rally_timer ≥ rally_time AND
 *              far enough from enemies → ADVANCE.
 *
 *   DEAD:      no forces, becomes a corpse glyph for a brief
 *              window (corpse_timer), then disappears.
 *
 * Each state has its own STEERING FORCE recipe. Code shape:
 *
 *     switch (warrior.state):
 *       case ADVANCE: force = seek_enemy_mass(warrior); ...
 *       case COMBAT:  force = seek_target(warrior); deal_damage()
 *       case FLEE:    force = flee_nearest_enemy(warrior); ...
 *       case DEAD:    return; // no movement
 *
 * Mid-state TRANSITIONS happen at the top of each tick before
 * the steering computation, by checking conditions (hp,
 * distance, target alive, timer expired). The FSM is the
 * "agent's mind"; the steering is the "agent's body."
 *
 * Same pattern reappears in any agent-based game AI: enemies
 * in shooters, units in RTS, NPCs in RPGs. State machine for
 * intent + steering for motion.
 *
 * T3  STEERING, PARAMETERISED — SAME FORCES, DIFFERENT WEIGHTS
 * ────────────────────────────────────────────────────────────
 * Each warrior steering recipe uses the same primitives we
 * built in crowd.c (T1-T2 there): seek, flee, separate. The
 * differences are:
 *
 *   - WEIGHTS — how strong each force is.
 *   - TARGETS — what to seek / flee from.
 *
 * Per-state recipes:
 *
 *     ADVANCE:  force = W_advance · seek(enemy_centroid)
 *                     + W_separate · separate(allies)
 *
 *     COMBAT:   force = W_combat · seek(locked_target)
 *                     + W_separate · separate(allies)
 *
 *     FLEE:     force = W_flee · flee(nearest_enemy)
 *                     + W_separate · separate(allies)
 *
 * The same boid-style sum drives all three states; only the
 * coefficients and targets differ. After force is computed:
 *
 *     vel += force · dt
 *     vel = clamp(vel, max_speed)        ← state-dependent cap
 *     pos += vel · dt
 *
 * max_speed is also state-dependent: warriors RUN faster
 * when fleeing than when advancing. Pulling speed cap from a
 * per-state lookup gives panic the right urgent feel.
 *
 * T4  ARCHERS — ADDING A SHOOT ACTION AND A PROJECTILE POOL
 * ─────────────────────────────────────────────────────────
 * Archers are warriors with EXTENDED LOGIC:
 *
 *     archer_logic distance ladder:
 *       d < archer_flee_range:    FLEE (close-range panic)
 *       d ≤ arrow_range:           COMBAT — shoot every shoot_interval
 *       d > arrow_range:           ADVANCE — close to standoff range
 *
 * The "shoot every interval" action is what's new. Each archer
 * has a shoot_timer:
 *
 *     archer.shoot_timer -= dt
 *     if archer.state == COMBAT and archer.shoot_timer ≤ 0:
 *       fire_arrow(archer, archer.target)
 *       archer.shoot_timer = shoot_interval
 *
 * Firing an arrow:
 *
 *     fire_arrow(archer, target):
 *       slot = first inactive slot in arrows[ARROW_POOL_MAX]
 *       if no slot: silently drop the shot
 *       slot.pos = archer.pos
 *       slot.vel = normalize(target.pos - archer.pos) · ARROW_TRAVEL_SPD
 *       slot.target_idx = target_idx
 *       slot.faction = archer.faction
 *       slot.active = true
 *
 * arrows_tick advances each arrow:
 *
 *     for each active arrow:
 *       arrow.pos += arrow.vel · dt
 *       if |arrow.pos - arrows.target.pos| < ARROW_HIT_DIST:
 *         deal damage; deactivate arrow
 *       elif arrow off-screen:
 *         deactivate
 *     compact arrows[] to keep active entries dense
 *
 * Three notable design choices:
 *
 *   - Arrow has its own struct + pool; not part of the warrior.
 *     Arrows can OUTLIVE the archer that fired them.
 *
 *   - Damage is on HIT, not on FIRE. The arrow can MISS (target
 *     moves, arrow exits world).
 *
 *   - Arrow is dumb-fire — velocity captured at shoot time.
 *     No homing. The target may walk out of the way.
 *
 * Same pattern works for any projectile system in any game:
 *  bullet, magic missile, thrown rock. Pool + tick + hit detect.
 *
 * T5  STRATEGY PRESETS — GLOBAL PARAMETER TABLES BEHIND A POINTER
 * ───────────────────────────────────────────────────────────────
 * Six battle "strategies" — STANDARD, BERSERKER, SHIELD WALL,
 * GUERRILLA, ARCHER FOCUS, CHAOS — each tune ~16 different
 * parameters (engage range, flee threshold, attack interval,
 * archer speed, shoot interval, etc.).
 *
 * Naive: switch (strategy) { case BERSERKER: engage_range = 80;
 * ...; flee_hp = 0; ... } scattered through the combat code.
 *
 * Better: a STRUCT of parameters + a global pointer:
 *
 *     typedef struct {
 *       float engage_range;
 *       float flee_hp;
 *       float atk_interval;
 *       float archer_speed;
 *       ...
 *     } StrategyParams;
 *
 *     StrategyParams STANDARD  = { ... };
 *     StrategyParams BERSERKER = { ... };
 *     ...
 *
 *     const StrategyParams *g_sp = &STANDARD;     // active strategy
 *
 * Combat code reads through `g_sp->engage_range` etc. Pressing
 * 1-6 just changes g_sp. Next tick, every warrior fights under
 * the new rules — no reset, no flag, no warrior-level state to
 * change.
 *
 * Why this works: the parameters drive STEERING + FSM
 * transitions, NOT the agent's structure. Same pool of
 * warriors; same FSM logic; different constants. Turn the dials
 * and the war changes character live.
 *
 * Generalisation: any "config preset" UI uses this pattern.
 * Audio mixers, IDE colour themes, scientific simulations
 * with parameter sweeps. Encapsulate the parameters; expose
 * them through a pointer; switch the pointer to switch the
 * config.
 *
 * T6  WHY MUTUAL ANNIHILATION MATTERS (AND HOW TO HANDLE IT)
 * ──────────────────────────────────────────────────────────
 * In a battle simulation, what if BOTH sides reach 0 alive
 * simultaneously?
 *
 *   - Last Gondor warrior fires an arrow at last Mordor.
 *   - Last Mordor warrior fires an arrow at last Gondor.
 *   - Both arrows hit the same tick.
 *   - Both warriors die simultaneously.
 *
 * Without explicit handling, the win-condition check sees
 * "no Gondor alive AND no Mordor alive" and may declare both
 * sides winners, neither, or NaN.
 *
 * Solution: deterministic tiebreak. We arbitrarily pick MORDOR
 * to win on simultaneous extinction. Code:
 *
 *     if g_alive == 0 and m_alive == 0:
 *       winner = F_MORDOR
 *     elif g_alive == 0:
 *       winner = F_MORDOR
 *     elif m_alive == 0:
 *       winner = F_GONDOR
 *     else:
 *       winner = NONE
 *
 * The choice (Mordor wins ties) is visual flavour, not balance.
 *
 * General pattern for ANY simultaneous outcome:
 *   - Define a tiebreak rule (any deterministic choice — by
 *     faction order, by who has more arrows in flight, by
 *     RNG seed).
 *   - Document it.
 *   - Don't pretend it can't happen.
 *
 * Mutual annihilation is rare but happens reproducibly. A
 * battle sim that crashes or shows undefined victory text on
 * those frames is worse than one that shows a slightly
 * arbitrary winner.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <float.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                            */
/* ===================================================================== */

/*
 * StrategyParams — all tuneable combat constants in one struct.
 *
 * g_sp always points to one of the six presets.  All combat logic reads
 * through g_sp so a key-press takes effect on the very next tick.
 *
 * Melee fields
 * ────────────
 *   engage_range   distance at which a melee warrior locks on → COMBAT
 *   flee_hp        HP threshold that triggers melee rout (0 = never)
 *   atk_interval   seconds between melee damage ticks
 *   speed_advance  march speed toward enemy mass (px/s)
 *   speed_flee     routing sprint speed; must exceed speed_advance
 *   sep_radius     ally personal-space radius (px); lower = denser
 *   safe_range     distance a routing warrior must reach before rallying
 *   rally_time     seconds spent safe before returning to ADVANCE
 *   melee_speed    slow footwork while brawling; keeps fighters near target
 *
 * Archer fields
 * ─────────────
 *   archer_flee_hp    HP at which archers panic (0 = never flee from HP)
 *   arrow_range       max shooting range; archers advance until inside
 *   archer_flee_range panic-flee if any enemy closes this far
 *   stand_off_dist    preferred gap from nearest enemy while shooting
 *   shoot_interval    seconds between arrow shots
 *   archer_speed      archer movement speed (slower than melee)
 *
 * Steering weights  (higher = stronger influence)
 *   w_seek   drive toward chosen target
 *   w_sep    push away from allies (low = dense pack, high = spread)
 *   w_flee   urgency of routing sprint
 */
typedef struct {
    const char *name;
    float engage_range;
    int   flee_hp;
    float atk_interval;
    float speed_advance;
    float speed_flee;
    float sep_radius;
    float safe_range;
    float rally_time;
    float melee_speed;
    int   archer_flee_hp;
    float arrow_range;
    float archer_flee_range;
    float stand_off_dist;
    float shoot_interval;
    float archer_speed;
    float w_seek;
    float w_sep;
    float w_flee;
} StrategyParams;

/*
 * Six presets — dramatically different combat rhythms:
 *
 *  STANDARD      Deliberate advance, engage near contact, flee at 1 HP.
 *                Archers hold mid-range.  Good baseline.
 *
 *  BERSERKER     Wide engage, fast attacks, nobody routs.  Dense packing
 *                (low sep) → chaotic melee pile in the centre.
 *
 *  SHIELD WALL   Slow tight march (high sep keeps ranks orderly).
 *                Archers shoot from very long range.  Hard to break.
 *
 *  GUERRILLA     Skirmish: hit fast, flee at 2 HP, rally in 1.2 s,
 *                re-engage.  Archers very jumpy and highly mobile.
 *
 *  ARCHER FOCUS  Ranged dominance: 220 px range, ~1 s fire rate,
 *                archers stay deep in their half (160 px standoff).
 *
 *  CHAOS         Everyone sprints, personal space collapses, no rout.
 *                Produces one big scrum at the centre.
 */
static const StrategyParams g_presets[6] = {
    /*            engage flee atk   adv    flee  sep   safe  rally melee */
    /*            afleehp  arange  aflee standoff shoot aspd  seek  sep   flee */

    { "STANDARD",
      40.0f, 1, 1.4f, 55.0f,105.0f, 30.0f,140.0f, 2.5f, 20.0f,
      1, 160.0f, 48.0f,110.0f, 1.8f, 48.0f,
      1.0f, 1.8f, 1.5f },

    { "BERSERKER",
      60.0f, 0, 0.9f, 75.0f,120.0f, 18.0f, 80.0f, 1.0f, 30.0f,
      0, 140.0f, 20.0f, 80.0f, 1.2f, 60.0f,
      1.5f, 0.8f, 0.5f },

    { "SHIELD WALL",
      28.0f, 1, 1.8f, 35.0f, 90.0f, 16.0f,160.0f, 3.5f, 12.0f,
      1, 180.0f, 60.0f,140.0f, 2.2f, 36.0f,
      0.8f, 2.5f, 1.8f },

    { "GUERRILLA",
      40.0f, 2, 0.8f, 65.0f,135.0f, 35.0f,100.0f, 1.2f, 25.0f,
      2, 150.0f, 70.0f,120.0f, 1.4f, 70.0f,
      1.2f, 1.5f, 2.2f },

    { "ARCHER FOCUS",
      35.0f, 1, 1.6f, 50.0f,100.0f, 28.0f,130.0f, 2.0f, 18.0f,
      2, 220.0f, 35.0f,160.0f, 1.0f, 55.0f,
      1.0f, 2.0f, 1.6f },

    { "CHAOS",
      80.0f, 0, 1.0f, 90.0f,115.0f,  8.0f, 60.0f, 0.8f, 40.0f,
      0, 120.0f, 15.0f, 60.0f, 2.5f, 80.0f,
      2.0f, 0.3f, 0.3f },
};

static const StrategyParams *g_sp        = &g_presets[0];
static int                   g_strat_idx = 0;

/* ── army sizes + colour pair IDs — fixed across all strategies ── */
enum {
    MELEE_DEFAULT    =  35,
    ARCHER_DEFAULT   =  12,
    WARRIORS_MAX     =  70,    /* cap per faction */
    POOL_MAX         = 160,    /* 2 × WARRIORS_MAX + headroom */
    REINFORCE_MELEE  =   6,
    REINFORCE_ARCHER =   2,
    SIM_FPS_DEFAULT  =  60,
    TARGET_FPS       =  60,
    FPS_UPDATE_MS    = 500,

    /*
     * Colour-pair IDs.  Pairs 1-7 paint the world (one per faction
     * unit type, plus shared corpse/banner yellow).  PAIR_HUD and
     * PAIR_HINT are theme-independent per the project HUD spec —
     * the HUD stays readable against any animation behind it.
     *
     * The HUD uses three pairs total:
     *   - faction sides (left/right counts) keep their faction colour
     *     (5 cyan for Gondor, 1 red for Mordor) so the eye reads
     *     "this number belongs to that side" instantly;
     *   - the centre strategy title uses PAIR_HUD bright yellow;
     *   - the bottom key-hint bar uses PAIR_HINT bright cyan.
     */
    N_COLORS         =   7,
    PAIR_HUD         =   8,    /* bright yellow — top-centre status */
    PAIR_HINT        =   9,    /* bright cyan   — bottom key hint   */
};

/* Cell dimensions — physics in px, draw in cells; convert only at render */
#define CELL_W   8
#define CELL_H  16

#define GONDOR  0   /* right side — cyan melee, green archers  */
#define MORDOR  1   /* left side  — red melee,  orange archers */

/* Fixed constants (identical across all strategies) */
#define ATK_DAMAGE       1
#define HP_MAX           3
#define CORPSE_LIFETIME  4.0f
#define HIT_FLASH_TIME   0.15f

/* Arrow projectile */
#define ARROW_POOL_MAX   80
#define ARROW_TRAVEL_SPD 220.0f   /* px/s, fixed regardless of strategy */
#define ARROW_HIT_DIST    14.0f   /* px, hit radius around target */

/* Timing */
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                            */
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
/* §3  color                                                            */
/* ===================================================================== */

/*
 * War palette — semantic, not theme-driven.  Each colour is tied to
 * a specific game role: faction sides + arrow visibility + corpse
 * fade + HUD bars.  Background = -1 (terminal default) so the demo
 * respects the user's theme.
 *
 *   Pair  256-col  Role
 *   ───────────────────────────────────────────────────────────
 *     1     196    Mordor melee — pure red
 *     2     208    Mordor archers + arrows — orange
 *     3     226    corpses + victory banner — yellow
 *     4      46    Gondor archers + arrows — matrix green
 *     5      51    Gondor melee — bright cyan
 *     6      33    spare (dodger blue, currently unused)
 *     7     201    spare (magenta, currently unused)
 *     8 PAIR_HUD   bright yellow 226 — top-centre status, A_BOLD
 *     9 PAIR_HINT  bright cyan   51  — bottom key hint, A_BOLD
 *
 * Every foreground sits in the bright half of the 256-colour cube
 * (≥ 33) per the project palette-brightness rule, so even routing
 * (A_BLINK) and last-HP (A_DIM) warriors stay legible.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, -1);
        init_pair(2, 208, -1);
        init_pair(3, 226, -1);
        init_pair(4,  46, -1);
        init_pair(5,  51, -1);
        init_pair(6,  33, -1);
        init_pair(7, 201, -1);
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(1, COLOR_RED,     -1);
        init_pair(2, COLOR_RED,     -1);
        init_pair(3, COLOR_YELLOW,  -1);
        init_pair(4, COLOR_GREEN,   -1);
        init_pair(5, COLOR_CYAN,    -1);
        init_pair(6, COLOR_BLUE,    -1);
        init_pair(7, COLOR_MAGENTA, -1);
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/* ===================================================================== */
/* §4  coords & vec2                                                    */
/* ===================================================================== */

static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/* Round-half-up avoids oscillation at exact half-pixel boundaries. */
static inline int px_to_cell_x(float px)
{
    return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py)
{
    return (int)floorf(py / (float)CELL_H + 0.5f);
}

typedef struct { float x, y; } Vec2;

static inline Vec2  v2(float x, float y)      { return (Vec2){x, y};          }
static inline Vec2  v2add(Vec2 a, Vec2 b)      { return v2(a.x+b.x, a.y+b.y); }
static inline Vec2  v2sub(Vec2 a, Vec2 b)      { return v2(a.x-b.x, a.y-b.y); }
static inline Vec2  v2scale(Vec2 v, float s)   { return v2(v.x*s,   v.y*s);   }
static inline float v2len(Vec2 v)              { return sqrtf(v.x*v.x+v.y*v.y); }
static inline float v2len2(Vec2 v)             { return v.x*v.x + v.y*v.y;    }

static inline Vec2 v2norm(Vec2 v)
{
    float l = v2len(v);
    return (l > 0.001f) ? v2scale(v, 1.0f/l) : v2(0,0);
}

static inline Vec2 v2clamp_len(Vec2 v, float max_len)
{
    float l = v2len(v);
    return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* Elastic wall bounce: velocity component flips on contact. */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh)
{
    if (pos->x <  0)  { pos->x = 0;    vel->x =  fabsf(vel->x); }
    if (pos->x >= ww) { pos->x = ww-1; vel->x = -fabsf(vel->x); }
    if (pos->y <  0)  { pos->y = 0;    vel->y =  fabsf(vel->y); }
    if (pos->y >= wh) { pos->y = wh-1; vel->y = -fabsf(vel->y); }
}

/* ===================================================================== */
/* §5  entity                                                           */
/* ===================================================================== */

typedef enum { UNIT_MELEE = 0, UNIT_ARCHER } UnitType;

typedef enum {
    STATE_ADVANCE = 0,   /* marching / repositioning */
    STATE_COMBAT,        /* engaged: brawling (melee) or shooting (archer) */
    STATE_FLEE,          /* routing */
    STATE_DEAD,          /* HP == 0; showing corpse */
} WarriorState;

/*
 * Warrior — complete per-entity state.
 *
 * Field groups, by what touches them:
 *   - kinematic state — written every tick by warrior_step; read by
 *     scene_draw (alpha-lerped between prev_pos and pos).
 *   - identity — set once at spawn (warrior_spawn), never changes.
 *   - HP + state machine — written by combat code; read by HUD,
 *     scene_draw (HP-driven attributes, BLINK on FLEE).
 *   - timers — countdown clocks owned by combat.
 *   - render-only hints — derived state the renderer reads.
 */
typedef struct {
    /* kinematic state — alpha-lerped on draw */
    Vec2         pos;
    Vec2         prev_pos;
    Vec2         vel;

    /* identity — set once at spawn */
    int          faction;          /* GONDOR or MORDOR             */
    UnitType     unit_type;        /* UNIT_MELEE or UNIT_ARCHER    */
    char         glyph;            /* ASCII glyph drawn for unit   */
    int          color_pair;       /* ncurses pair for this unit   */

    /* HP + state machine */
    int          hp;               /* 0..HP_MAX                    */
    WarriorState state;            /* ADVANCE/COMBAT/FLEE/DEAD     */
    int          target_idx;       /* pool index of locked enemy
                                    * (-1 = none; archers ignore)  */

    /* timers (seconds) */
    float        atk_timer;        /* until next melee hit / shot  */
    float        rally_timer;      /* time at safe distance         */
    float        dead_timer;       /* corpse remaining lifetime     */
    float        hit_timer;        /* arrow-strike '*' flash        */
} Warrior;

/*
 * Arrow — a '-' projectile fired by an archer.
 *
 * Travels at ARROW_TRAVEL_SPD px/s toward target_idx.
 * Hit: within ARROW_HIT_DIST of target → ATK_DAMAGE + '*' flash.
 * Out of bounds → deactivated (miss).
 * Compacted out of the pool each tick.
 */
typedef struct {
    Vec2 pos;
    Vec2 vel;
    int  target_idx;
    int  faction;
    bool active;
} Arrow;

static const char GONDOR_MELEE_GLYPHS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const char MORDOR_MELEE_GLYPHS[]  = "abcdefghijklmnopqrstuvwxyz";

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * warrior_spawn — initialise one pool slot.
 *
 * Spawn zones: melee outer 30%, archers deeper 15% of their half.
 * atk_timer is randomised to stagger first attacks — organic rhythm.
 */
static void warrior_spawn(Warrior *w, int id, int faction,
                          UnitType unit_type, float ww, float wh)
{
    float x_lo, x_hi;
    if (faction == GONDOR) {
        x_lo = (unit_type == UNIT_ARCHER) ? ww * 0.84f : ww * 0.60f;
        x_hi = (unit_type == UNIT_ARCHER) ? ww * 0.97f : ww * 0.82f;
    } else {
        x_lo = (unit_type == UNIT_ARCHER) ? ww * 0.03f : ww * 0.18f;
        x_hi = (unit_type == UNIT_ARCHER) ? ww * 0.16f : ww * 0.40f;
    }

    w->pos         = v2(x_lo + randf()*(x_hi - x_lo), randf()*wh);
    w->prev_pos    = w->pos;
    w->vel         = v2(0, 0);
    w->faction     = faction;
    w->unit_type   = unit_type;
    w->hp          = HP_MAX;
    w->atk_timer   = (unit_type == UNIT_ARCHER
                      ? g_sp->shoot_interval : g_sp->atk_interval) * randf();
    w->rally_timer = 0.0f;
    w->target_idx  = -1;
    w->state       = STATE_ADVANCE;
    w->dead_timer  = 0.0f;
    w->hit_timer   = 0.0f;

    if (unit_type == UNIT_ARCHER) {
        w->glyph      = (faction == GONDOR) ? '@' : '%';
        w->color_pair = (faction == GONDOR) ?  4  :  2;
    } else {
        w->glyph      = (faction == GONDOR)
                        ? GONDOR_MELEE_GLYPHS[id % (int)(sizeof(GONDOR_MELEE_GLYPHS)-1)]
                        : MORDOR_MELEE_GLYPHS[id % (int)(sizeof(MORDOR_MELEE_GLYPHS)-1)];
        w->color_pair = (faction == GONDOR) ? 5 : 1;
    }
}

/*
 * warrior_step — shared Euler integration for every live state.
 *
 *   vel     += accel × dt  then clamped to max_speed
 *   prev_pos = pos          (renderer lerps between prev and pos)
 *   pos     += vel × dt
 */
static void warrior_step(Warrior *w, Vec2 accel, float max_speed, float dt)
{
    w->vel      = v2clamp_len(v2add(w->vel, v2scale(accel, dt)), max_speed);
    w->prev_pos = w->pos;
    w->pos      = v2add(w->pos, v2scale(w->vel, dt));
}

/* ===================================================================== */
/* §6  combat — steering forces, melee logic, archer logic              */
/* ===================================================================== */

/*
 * steer_seek — force steering toward (target) at (speed).
 *
 *   desired = normalise(target − pos) × speed
 *   force   = desired − vel
 *
 * Subtracting current velocity gives smooth deceleration: when moving at
 * full speed toward the target, force → 0 naturally.
 */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed)
{
    Vec2 desired = v2scale(v2norm(v2sub(target, pos)), speed);
    return v2sub(desired, vel);
}

/* steer_flee — steer AWAY from threat; negated seek. */
static Vec2 steer_flee(Vec2 pos, Vec2 vel, Vec2 threat, float speed)
{
    return v2scale(steer_seek(pos, vel, threat, speed), -1.0f);
}

/*
 * steer_separate — repulsion from same-faction allies within sep_radius.
 *
 * Repulsion strength = (sep_radius − dist) / sep_radius ∈ (0,1].
 * Scaled by speed_advance so force is in velocity-compatible units.
 * Only same-faction warriors push each other (fight through enemies).
 */
static Vec2 steer_separate(const Warrior *pool, int n_total, int self)
{
    Vec2           force = v2(0, 0);
    const Warrior *me    = &pool[self];
    for (int i = 0; i < n_total; i++) {
        if (i == self)                       continue;
        if (pool[i].faction != me->faction)  continue;
        if (pool[i].state   == STATE_DEAD)   continue;
        Vec2  away = v2sub(me->pos, pool[i].pos);
        float d    = v2len(away);
        if (d < g_sp->sep_radius && d > 0.001f) {
            float strength = (g_sp->sep_radius - d) / g_sp->sep_radius;
            force = v2add(force, v2scale(v2norm(away),
                                         strength * g_sp->speed_advance));
        }
    }
    return force;
}

/*
 * enemy_centroid — average position of all living enemies.
 * Warriors march toward the mass (not one target) to form a battle line.
 * Falls back to world centre when no enemies remain.
 */
static Vec2 enemy_centroid(const Warrior *pool, int n_total,
                            int faction, float ww, float wh)
{
    int  efac = 1 - faction;
    Vec2 sum  = v2(0, 0);
    int  n    = 0;
    for (int i = 0; i < n_total; i++) {
        if (pool[i].faction != efac)       continue;
        if (pool[i].state   == STATE_DEAD) continue;
        sum = v2add(sum, pool[i].pos);
        n++;
    }
    return n ? v2scale(sum, 1.0f/n) : v2(ww*0.5f, wh*0.5f);
}

/* nearest_enemy_idx — pool index of the closest living enemy; -1 if none. */
static int nearest_enemy_idx(const Warrior *pool, int n_total, int self)
{
    int   efac       = 1 - pool[self].faction;
    int   best       = -1;
    float best_dist2 = FLT_MAX;
    for (int i = 0; i < n_total; i++) {
        if (pool[i].faction != efac)       continue;
        if (pool[i].state   == STATE_DEAD) continue;
        float d2 = v2len2(v2sub(pool[i].pos, pool[self].pos));
        if (d2 < best_dist2) { best_dist2 = d2; best = i; }
    }
    return best;
}

/*
 * Steer — output of every per-state behaviour helper:
 *   force    weighted force vector to integrate this tick (px/s²)
 *   max_spd  hard cap on |vel| after integration (px/s)
 *
 * Returning a struct (rather than two output pointers) keeps the
 * call sites short and makes the contract explicit: every helper
 * MUST set both fields.
 */
typedef struct { Vec2 force; float max_spd; } Steer;

/* ── melee state machine ─────────────────────────────────────────── *
 *
 * Shared contract: each per-state helper takes the warrior and the
 * pool, reads/writes w->state and w->target_idx + timers as needed,
 * and returns the Steer for this tick.  The dispatcher applies the
 * result via warrior_step + bounce_pos exactly once.
 */

/*
 * melee_advance — march toward enemy mass; transition to COMBAT
 * when any enemy enters engage_range.  Sets target_idx on lock.
 */
static Steer melee_advance(Warrior *w, const Warrior *pool, int n_total,
                           int self, float ww, float wh)
{
    Vec2 centroid = enemy_centroid(pool, n_total, w->faction, ww, wh);
    int  ne       = nearest_enemy_idx(pool, n_total, self);

    if (ne >= 0 && v2len(v2sub(pool[ne].pos, w->pos)) < g_sp->engage_range) {
        /* lock on → COMBAT next tick.  No movement this tick;
         * atk_timer reset prevents an instant-hit on transition. */
        w->state      = STATE_COMBAT;
        w->target_idx = ne;
        w->atk_timer  = g_sp->atk_interval;
        return (Steer){ v2(0, 0), g_sp->speed_advance };
    }

    Vec2 force = v2add(
        v2scale(steer_seek    (w->pos, w->vel, centroid, g_sp->speed_advance),
                g_sp->w_seek),
        v2scale(steer_separate(pool, n_total, self),
                g_sp->w_sep)
    );
    return (Steer){ force, g_sp->speed_advance };
}

/*
 * melee_combat — slow footwork toward the locked target; deal
 * ATK_DAMAGE every atk_interval seconds.  Transitions:
 *   target died    → ADVANCE
 *   hp ≤ flee_hp   → FLEE (drop target)
 */
static Steer melee_combat(Warrior *w, Warrior *pool, int n_total,
                          int self, float dt)
{
    if (w->target_idx < 0) {
        w->state = STATE_ADVANCE;
        return (Steer){ v2(0, 0), g_sp->speed_advance };
    }
    if (w->hp <= g_sp->flee_hp) {
        w->state      = STATE_FLEE;
        w->target_idx = -1;
        return (Steer){ v2(0, 0), g_sp->speed_flee };
    }

    /* Damage tick: consume the timer; on rollover, hit + reset. */
    w->atk_timer -= dt;
    if (w->atk_timer <= 0.0f) {
        w->atk_timer            = g_sp->atk_interval;
        pool[w->target_idx].hp -= ATK_DAMAGE;
    }

    Vec2 tgt   = pool[w->target_idx].pos;
    Vec2 force = v2add(
        v2scale(steer_seek    (w->pos, w->vel, tgt, g_sp->melee_speed),
                g_sp->w_seek),
        v2scale(steer_separate(pool, n_total, self),
                g_sp->w_sep * 0.4f)   /* reduced: stay near target */
    );
    return (Steer){ force, g_sp->melee_speed * 1.5f };
}

/*
 * melee_flee — sprint away from the nearest enemy.  Once at
 * safe_range, count rally_timer; on rally_time → ADVANCE.
 */
static Steer melee_flee(Warrior *w, const Warrior *pool, int n_total,
                        int self, float dt)
{
    int ne = nearest_enemy_idx(pool, n_total, self);
    if (ne < 0) {
        w->state = STATE_ADVANCE;
        return (Steer){ v2(0, 0), g_sp->speed_advance };
    }

    float d = v2len(v2sub(pool[ne].pos, w->pos));
    if (d >= g_sp->safe_range) {
        /* Safe — count rally time and gently bleed velocity. */
        w->rally_timer += dt;
        w->vel = v2scale(w->vel, 0.92f);
        if (w->rally_timer >= g_sp->rally_time) {
            w->state       = STATE_ADVANCE;
            w->rally_timer = 0.0f;
        }
        return (Steer){ v2(0, 0), g_sp->speed_flee };
    }

    /* Still in danger — flee under full force. */
    w->rally_timer = 0.0f;
    Vec2 force = v2add(
        v2scale(steer_flee    (w->pos, w->vel, pool[ne].pos,
                               g_sp->speed_flee), g_sp->w_flee),
        v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
    );
    return (Steer){ force, g_sp->speed_flee };
}

/*
 * melee_logic — dispatcher.  Picks the per-state helper, applies its
 * Steer through warrior_step + bounce_pos.  All combat code reads
 * g_sp, so strategy changes take effect on the very next tick.
 */
static void melee_logic(Warrior *pool, int n_total, int self,
                        float ww, float wh, float dt)
{
    Warrior *w = &pool[self];
    Steer s    = { v2(0, 0), g_sp->speed_advance };

    switch (w->state) {
    case STATE_ADVANCE: s = melee_advance(w, pool, n_total, self, ww, wh); break;
    case STATE_COMBAT:  s = melee_combat (w, pool, n_total, self, dt);    break;
    case STATE_FLEE:    s = melee_flee   (w, pool, n_total, self, dt);    break;
    default:            break;
    }

    warrior_step(w, s.force, s.max_spd, dt);
    bounce_pos  (&w->pos, &w->vel, ww, wh);
}

/* ── archer state machine ────────────────────────────────────────── *
 *
 * Archers are distance-driven; no target lock.  The dispatcher picks
 * one of three behaviours per tick based on dist-to-nearest-enemy
 * vs. the strategy's range thresholds, plus an HP panic check.
 */

/*
 * archer_shoot — spawn one '-' arrow from this archer toward target
 * pool[ne].  Silently dropped if the arrow pool is at capacity.
 *
 *   pos     = archer's current position
 *   vel     = ARROW_TRAVEL_SPD · normalize(target − archer)
 *   target  = pool index of the locked enemy at shoot time
 *
 * The arrow flies in a straight line at constant speed; it does not
 * track the target if the target moves after the shot.  Hits are
 * detected by arrows_tick on a proximity check.
 */
static void archer_shoot(const Warrior *w, const Warrior *pool, int ne,
                         Arrow *arrows, int *n_arrows)
{
    if (*n_arrows >= ARROW_POOL_MAX) return;
    Vec2 to_tgt = v2norm(v2sub(pool[ne].pos, w->pos));
    arrows[*n_arrows] = (Arrow){
        .pos        = w->pos,
        .vel        = v2scale(to_tgt, ARROW_TRAVEL_SPD),
        .target_idx = ne,
        .faction    = w->faction,
        .active     = true,
    };
    (*n_arrows)++;
}

/*
 * archer_flee_force — flee force from nearest enemy + ally separation.
 * Used by both the close-range panic path and the HP-flee carry-over
 * path; the caller picks which.
 */
static Steer archer_flee_force(const Warrior *w, const Warrior *pool,
                               int n_total, int self, int ne)
{
    Vec2 force = v2add(
        v2scale(steer_flee    (w->pos, w->vel, pool[ne].pos,
                               g_sp->speed_flee), g_sp->w_flee),
        v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
    );
    return (Steer){ force, g_sp->speed_flee };
}

/*
 * archer_combat — within arrow_range: hold standoff (stand_off_dist
 * from the nearest enemy) and shoot every shoot_interval seconds.
 */
static Steer archer_combat(Warrior *w, const Warrior *pool, int n_total,
                           int self, int ne, Vec2 away_from_enemy,
                           float dt, Arrow *arrows, int *n_arrows)
{
    w->state      = STATE_COMBAT;
    w->atk_timer -= dt;
    if (w->atk_timer <= 0.0f) {
        w->atk_timer = g_sp->shoot_interval;
        archer_shoot(w, pool, ne, arrows, n_arrows);
    }

    Vec2 standoff = v2add(pool[ne].pos,
                          v2scale(away_from_enemy, g_sp->stand_off_dist));
    Vec2 force    = v2add(
        v2scale(steer_seek    (w->pos, w->vel, standoff,
                               g_sp->archer_speed * 0.4f), g_sp->w_seek),
        v2scale(steer_separate(pool, n_total, self), g_sp->w_sep * 0.5f)
    );
    return (Steer){ force, g_sp->archer_speed * 0.5f };
}

/*
 * archer_advance — beyond arrow_range: head toward a standoff point
 * BEHIND the enemy centroid (i.e. on this archer's own side of the
 * field).  This keeps archers in their half rather than charging in.
 */
static Steer archer_advance(Warrior *w, const Warrior *pool, int n_total,
                            int self, float ww, float wh)
{
    w->state = STATE_ADVANCE;
    Vec2 centroid    = enemy_centroid(pool, n_total, w->faction, ww, wh);
    Vec2 safe_dir    = v2norm(v2sub(w->pos, centroid));
    Vec2 advance_tgt = v2add(centroid,
                             v2scale(safe_dir, g_sp->stand_off_dist));
    Vec2 force = v2add(
        v2scale(steer_seek    (w->pos, w->vel, advance_tgt,
                               g_sp->archer_speed), g_sp->w_seek),
        v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
    );
    return (Steer){ force, g_sp->archer_speed };
}

/*
 * archer_logic — dispatcher.  Order of decisions (matters):
 *   1. No enemies → coast to a stop and return.
 *   2. HP panic   → set FLEE (one-shot transition; doesn't return).
 *   3. Close range (dist < archer_flee_range) → FLEE force.
 *   4. Already in FLEE state (HP-flee carry-over) → keep fleeing.
 *   5. Within arrow_range → COMBAT (shoot + drift to standoff).
 *   6. Otherwise → ADVANCE toward standoff behind centroid.
 *   7. After integrate, run rally tick if still in FLEE.
 */
static void archer_logic(Warrior *pool, int n_total, int self,
                         float ww, float wh, float dt,
                         Arrow *arrows, int *n_arrows)
{
    Warrior *w  = &pool[self];
    int      ne = nearest_enemy_idx(pool, n_total, self);

    /* (1) no enemies left */
    if (ne < 0) {
        w->state = STATE_ADVANCE;
        w->vel   = v2scale(w->vel, 0.92f);
        warrior_step(w, v2(0, 0), g_sp->archer_speed, dt);
        bounce_pos  (&w->pos, &w->vel, ww, wh);
        return;
    }

    /* (2) HP panic — doesn't return; flee force chosen below */
    if (g_sp->archer_flee_hp > 0 && w->hp <= g_sp->archer_flee_hp
        && w->state != STATE_FLEE) {
        w->state       = STATE_FLEE;
        w->rally_timer = 0.0f;
    }

    float dist            = v2len(v2sub(pool[ne].pos, w->pos));
    Vec2  away_from_enemy = v2norm(v2sub(w->pos, pool[ne].pos));
    Steer s;

    if (dist < g_sp->archer_flee_range) {
        /* (3) close-range panic */
        w->state       = STATE_FLEE;
        w->rally_timer = 0.0f;
        s = archer_flee_force(w, pool, n_total, self, ne);
    } else if (w->state == STATE_FLEE) {
        /* (4) HP-flee carry-over, enemy not yet in melee range */
        s = archer_flee_force(w, pool, n_total, self, ne);
    } else if (dist <= g_sp->arrow_range) {
        /* (5) shoot mode */
        s = archer_combat(w, pool, n_total, self, ne, away_from_enemy,
                          dt, arrows, n_arrows);
    } else {
        /* (6) too far — march to standoff behind centroid */
        s = archer_advance(w, pool, n_total, self, ww, wh);
    }

    warrior_step(w, s.force, s.max_spd, dt);
    bounce_pos  (&w->pos, &w->vel, ww, wh);

    /* (7) rally tick */
    if (w->state == STATE_FLEE) {
        float post_dist = v2len(v2sub(pool[ne].pos, w->pos));
        if (post_dist >= g_sp->safe_range) {
            w->rally_timer += dt;
            if (w->rally_timer >= g_sp->rally_time) {
                w->state       = STATE_ADVANCE;
                w->rally_timer = 0.0f;
            }
        } else {
            w->rally_timer = 0.0f;
        }
    }
}

/* ── top-level warrior tick ──────────────────────────────────────── */

/*
 * warrior_tick — run one warrior through the full update pipeline.
 *
 * Shared preamble (both unit types):
 *   1. Tick hit_timer down (arrow-strike flash duration).
 *   2. DEAD: count down corpse timer; return.
 *   3. hp ≤ 0: die, credit kill to enemy faction.
 *   4. Invalidate stale target (enemy died last tick).
 *
 * Then dispatch to melee_logic or archer_logic.
 * Kill-credit bookkeeping lives here so neither logic function needs it.
 */
static void warrior_tick(Warrior *pool, int n_total, int self,
                         float ww, float wh, float dt, int kills[2],
                         Arrow *arrows, int *n_arrows)
{
    Warrior *w = &pool[self];

    if (w->hit_timer > 0.0f) w->hit_timer -= dt;

    if (w->state == STATE_DEAD) { w->dead_timer -= dt; return; }

    if (w->hp <= 0) {
        w->state      = STATE_DEAD;
        w->dead_timer = CORPSE_LIFETIME;
        kills[1 - w->faction]++;
        return;
    }

    if (w->target_idx >= 0 && pool[w->target_idx].state == STATE_DEAD)
        w->target_idx = -1;

    if (w->unit_type == UNIT_ARCHER)
        archer_logic(pool, n_total, self, ww, wh, dt, arrows, n_arrows);
    else
        melee_logic (pool, n_total, self, ww, wh, dt);
}

/* ===================================================================== */
/* §7  scene                                                            */
/* ===================================================================== */

/*
 * Scene — complete simulation state.
 *
 * Field groups, by who reads/writes them:
 *   - warrior pool — new warriors always append at pool[n_total++];
 *     dead warriors keep their slot (no compaction) so corpse
 *     rendering is trivial and indices stay stable.
 *   - arrow pool — inactive arrows compacted each tick by
 *     arrows_tick(); spare slots reused on next shoot.
 *   - per-faction tallies — recomputed each tick from the pool;
 *     drive HUD display and victory detection.
 *   - world dimensions — refreshed each tick from cols/rows.
 *   - user mode flags — toggled by app_handle_key.
 */
typedef struct {
    /* warrior pool — both factions interleaved by spawn order */
    Warrior pool[POOL_MAX];
    int     n_total;             /* used slots; never decreases */

    /* arrow pool — compacted each tick */
    Arrow   arrows[ARROW_POOL_MAX];
    int     n_arrows;            /* active arrow count          */

    /* per-faction tallies — recomputed in scene_tick */
    int     n_alive[2];
    int     n_archers[2];
    int     kills[2];            /* cumulative kills credited   */
    int     winner;              /* -1 ongoing, 0 GONDOR, 1 MORDOR */

    /* world dimensions in pixels — refreshed each tick */
    float   world_w, world_h;

    /* user mode flags */
    bool    paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);   /* zeros arrows[], n_arrows, pool[], etc. */
    s->world_w = pw(cols);
    s->world_h = ph(rows);
    s->winner  = -1;

    /* Spawn order: Gondor melee → Gondor archers → Mordor melee → archers */
    for (int i = 0; i < MELEE_DEFAULT; i++) {
        warrior_spawn(&s->pool[s->n_total], s->n_total,
                      GONDOR, UNIT_MELEE, s->world_w, s->world_h);
        s->n_total++;
    }
    for (int i = 0; i < ARCHER_DEFAULT; i++) {
        warrior_spawn(&s->pool[s->n_total], s->n_total,
                      GONDOR, UNIT_ARCHER, s->world_w, s->world_h);
        s->n_total++;
    }
    for (int i = 0; i < MELEE_DEFAULT; i++) {
        warrior_spawn(&s->pool[s->n_total], s->n_total,
                      MORDOR, UNIT_MELEE, s->world_w, s->world_h);
        s->n_total++;
    }
    for (int i = 0; i < ARCHER_DEFAULT; i++) {
        warrior_spawn(&s->pool[s->n_total], s->n_total,
                      MORDOR, UNIT_ARCHER, s->world_w, s->world_h);
        s->n_total++;
    }
}

/*
 * scene_add_warriors — append reinforcements to the pool.
 *
 * Always appends at pool[n_total++]: no faction-offset arithmetic, no
 * aliasing bugs regardless of the order in which factions are reinforced.
 */
static void scene_add_warriors(Scene *s, int faction)
{
    if (s->winner >= 0) return;

    int fac_alive = 0;
    for (int i = 0; i < s->n_total; i++)
        if (s->pool[i].faction == faction && s->pool[i].state != STATE_DEAD)
            fac_alive++;
    if (fac_alive >= WARRIORS_MAX) return;

    for (int i = 0; i < REINFORCE_MELEE && s->n_total < POOL_MAX; i++) {
        warrior_spawn(&s->pool[s->n_total], s->n_total,
                      faction, UNIT_MELEE, s->world_w, s->world_h);
        s->n_total++;
    }
    for (int i = 0; i < REINFORCE_ARCHER && s->n_total < POOL_MAX; i++) {
        warrior_spawn(&s->pool[s->n_total], s->n_total,
                      faction, UNIT_ARCHER, s->world_w, s->world_h);
        s->n_total++;
    }
}

/*
 * arrows_tick — move all active arrows and detect hits.
 *
 * Each arrow advances pos += vel × dt.
 * Out of bounds → miss (deactivated).
 * Target dead   → deactivated.
 * Within ARROW_HIT_DIST of target → deal ATK_DAMAGE + HIT_FLASH_TIME.
 * After processing, compact inactive slots to the front of the pool.
 */
static void arrows_tick(Arrow *arrows, int *n_arrows, Warrior *pool,
                        float ww, float wh, float dt)
{
    for (int i = 0; i < *n_arrows; i++) {
        Arrow *a = &arrows[i];
        if (!a->active) continue;

        a->pos = v2add(a->pos, v2scale(a->vel, dt));

        if (a->pos.x < 0 || a->pos.x >= ww ||
            a->pos.y < 0 || a->pos.y >= wh) {
            a->active = false; continue;
        }
        if (a->target_idx < 0) { a->active = false; continue; }

        Warrior *tgt = &pool[a->target_idx];
        if (tgt->state == STATE_DEAD) { a->active = false; continue; }

        if (v2len(v2sub(tgt->pos, a->pos)) < ARROW_HIT_DIST) {
            tgt->hp       -= ATK_DAMAGE;
            tgt->hit_timer = HIT_FLASH_TIME;
            a->active = false;
        }
    }

    /* Compact: shift active arrows to the front */
    int j = 0;
    for (int i = 0; i < *n_arrows; i++)
        if (arrows[i].active) arrows[j++] = arrows[i];
    *n_arrows = j;
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    s->world_w = pw(cols);
    s->world_h = ph(rows);
    if (s->paused || s->winner >= 0) return;

    for (int i = 0; i < s->n_total; i++)
        warrior_tick(s->pool, s->n_total, i,
                     s->world_w, s->world_h, dt, s->kills,
                     s->arrows, &s->n_arrows);

    arrows_tick(s->arrows, &s->n_arrows, s->pool,
                s->world_w, s->world_h, dt);

    s->n_alive[0] = s->n_alive[1] = 0;
    s->n_archers[0] = s->n_archers[1] = 0;
    for (int i = 0; i < s->n_total; i++) {
        if (s->pool[i].state == STATE_DEAD) continue;
        int f = s->pool[i].faction;
        s->n_alive[f]++;
        if (s->pool[i].unit_type == UNIT_ARCHER) s->n_archers[f]++;
    }

    if      (s->n_alive[GONDOR] == 0 && s->n_alive[MORDOR] == 0)
        s->winner = MORDOR;   /* mutual annihilation */
    else if (s->n_alive[GONDOR] == 0) s->winner = MORDOR;
    else if (s->n_alive[MORDOR] == 0) s->winner = GONDOR;
}

/*
 * mark_cell — stamp one ASCII glyph at terminal cell (cx, cy).
 *
 * Centralises the (chtype)(unsigned char) cast plus bounds-check that
 * would otherwise be repeated at every mvwaddch site.  The double
 * cast prevents sign-extension on character values > 127 (per
 * CLAUDE.md "Common ncurses Bugs").  Off-screen cells are silently
 * dropped.
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
 * warrior_attr — compute the ncurses attribute bundle for a living
 * warrior, encoding HP and state into A_BOLD / A_DIM / A_BLINK.
 *
 *   hp == HP_MAX  → A_BOLD       (full strength)
 *   hp == 1       → A_DIM        (last hit)
 *   state==FLEE   → |= A_BLINK   (panic)
 */
static attr_t warrior_attr(const Warrior *wr)
{
    attr_t attr = A_NORMAL;
    if      (wr->hp >= HP_MAX)   attr |= A_BOLD;
    else if (wr->hp <= 1)        attr |= A_DIM;
    if (wr->state == STATE_FLEE) attr |= A_BLINK;
    return attr;
}

/*
 * draw_arrows — pass 0: every active arrow as a coloured '-'
 * (green for Gondor, orange for Mordor).  No alpha lerp — arrows
 * are short-lived and move fast, so the 16 ms sub-tick smoothing
 * doesn't help.
 */
static void draw_arrows(const Scene *s, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < s->n_arrows; i++) {
        const Arrow *a = &s->arrows[i];
        if (!a->active) continue;
        int cpair = (a->faction == GONDOR) ? 4 : 2;
        mark_cell(w, px_to_cell_x(a->pos.x), px_to_cell_y(a->pos.y),
                  '-', cpair, A_BOLD, cols, rows);
    }
}

/*
 * draw_corpses — pass 1: dim '.' at every dead warrior whose corpse
 * timer has not yet expired.  Drawn underneath living warriors so a
 * fresh kill on the same cell doesn't show two glyphs.
 */
static void draw_corpses(const Scene *s, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < s->n_total; i++) {
        const Warrior *wr = &s->pool[i];
        if (wr->state != STATE_DEAD || wr->dead_timer <= 0.0f) continue;
        mark_cell(w, px_to_cell_x(wr->pos.x), px_to_cell_y(wr->pos.y),
                  '.', 3, A_DIM, cols, rows);
    }
}

/*
 * draw_living — pass 2: every living warrior with HP-driven attrs.
 * Arrow-strike flash ('*' in standout+bold) briefly overrides the
 * warrior's own glyph during HIT_FLASH_TIME after being hit.  Draws
 * at the alpha-interpolated position prev_pos → pos.
 */
static void draw_living(const Scene *s, WINDOW *w,
                        int cols, int rows, float alpha)
{
    for (int i = 0; i < s->n_total; i++) {
        const Warrior *wr = &s->pool[i];
        if (wr->state == STATE_DEAD) continue;

        Vec2 dp = v2add(wr->prev_pos,
                        v2scale(v2sub(wr->pos, wr->prev_pos), alpha));
        int cx = px_to_cell_x(dp.x);
        int cy = px_to_cell_y(dp.y);

        if (wr->hit_timer > 0.0f) {
            mark_cell(w, cx, cy, '*',
                      wr->color_pair, A_STANDOUT | A_BOLD, cols, rows);
        } else {
            mark_cell(w, cx, cy, wr->glyph,
                      wr->color_pair, warrior_attr(wr), cols, rows);
        }
    }
}

/*
 * draw_victory_banner — pass 3: blinking centred ribbon when one
 * faction has been wiped out.  Drawn last so it overlays everything.
 */
static void draw_victory_banner(const Scene *s, WINDOW *w, int cols, int rows)
{
    if (s->winner < 0) return;
    static const char *win_msg[2] = {
        "  === GONDOR WINS — FOR FRODO ===  ",
        "  === MORDOR WINS — THE EYE SEES ALL ===  ",
    };
    const char *msg = win_msg[s->winner];
    int mx = (cols - (int)strlen(msg)) / 2;
    if (mx < 0) mx = 0;
    wattron(w, COLOR_PAIR(3) | A_BOLD | A_BLINK);
    mvwprintw(w, rows/2, mx, "%s", msg);
    wattroff(w, COLOR_PAIR(3) | A_BOLD | A_BLINK);
}

/*
 * scene_draw — paint one frame in painter's order:
 *
 *   1. arrows '-' under everything (so fired arrows look "behind"
 *      the warriors they were shot at).
 *   2. corpses '.' dim under living warriors.
 *   3. living warriors at alpha-interpolated positions.
 *   4. victory banner overlaid last when battle is decided.
 *
 * Each pass is its own helper so a reader can find any pass
 * without scanning a 100-line orchestrator.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows, float alpha)
{
    draw_arrows         (s, w, cols, rows);
    draw_corpses        (s, w, cols, rows);
    draw_living         (s, w, cols, rows, alpha);
    draw_victory_banner (s, w, cols, rows);
}

/* ===================================================================== */
/* §8  app — screen, input, main loop                                  */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw — assemble a complete frame: scene, then HUD on top.
 *
 * HUD layout:
 *   Row 0 left:    GONDOR alive (melee + archers) and kills
 *   Row 0 centre:  [ WAR: <strategy> ] or [ WAR: PAUSED ]
 *   Row 0 right:   MORDOR alive and kills
 *   Row 1 right:   fps / sim Hz
 *   Last row:      key hints
 */
static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha);

    int gm = sc->n_alive[GONDOR] - sc->n_archers[GONDOR];
    int ga = sc->n_archers[GONDOR];
    int mm = sc->n_alive[MORDOR] - sc->n_archers[MORDOR];
    int ma = sc->n_archers[MORDOR];

    char lbuf[56], rbuf[56];
    snprintf(lbuf, sizeof lbuf, " GONDOR %dm %da  K:%d ",
             gm, ga, sc->kills[GONDOR]);
    snprintf(rbuf, sizeof rbuf, " K:%d  %dm %da MORDOR ",
             sc->kills[MORDOR], mm, ma);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 0, "%s", lbuf);
    attroff(COLOR_PAIR(5) | A_BOLD);

    int rx = s->cols - (int)strlen(rbuf);
    if (rx < 0) rx = 0;
    attron(COLOR_PAIR(1) | A_BOLD);
    mvprintw(0, rx, "%s", rbuf);
    attroff(COLOR_PAIR(1) | A_BOLD);

    /* Centre: active strategy name (or PAUSED) — PAIR_HUD bright yellow */
    char title_buf[48];
    snprintf(title_buf, sizeof title_buf, "[ WAR: %-12s ]",
             sc->paused ? "PAUSED" : g_sp->name);
    int tx = (s->cols - (int)strlen(title_buf)) / 2;
    if (tx < 0) tx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, tx, "%s", title_buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 right-aligned: fps + sim Hz — PAIR_HUD as well */
    char fps_buf[40];
    snprintf(fps_buf, sizeof fps_buf, " %.0f fps  sim:%d Hz ", fps, sim_fps);
    int fx = s->cols - (int)strlen(fps_buf);
    if (fx < 0) fx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, fx, "%s", fps_buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom-left key hint — PAIR_HINT bright cyan, A_BOLD per spec */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
        " q:quit  spc:pause  r:reset  g:+gondor  m:+mordor  1-6:strategy ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── App ── */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/*
 * app_handle_key — process one keystroke.
 *
 * Keys 1–6 switch g_sp to the corresponding preset.  The change is live:
 * all combat logic reads g_sp on the next tick — no reset required.
 * In-flight arrows continue unaffected (ARROW_TRAVEL_SPD is constant).
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *sc = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ':  sc->paused = !sc->paused;                     break;
    case 'r': case 'R':
        scene_init(sc, app->screen.cols, app->screen.rows);  break;
    case 'g': case 'G': scene_add_warriors(sc, GONDOR);      break;
    case 'm': case 'M': scene_add_warriors(sc, MORDOR);      break;
    case '1': case '2': case '3': case '4': case '5': case '6':
        g_strat_idx = ch - '1';
        g_sp        = &g_presets[g_strat_idx];
        break;
    default: break;
    }
    return true;
}

/*
 * main — game loop (fixed-step accumulator; see framework.c §8).
 *
 * ① dt: wall-clock elapsed since last frame, capped at 100 ms.
 * ② Drain sim accumulator: scene_tick at fixed dt_sec until empty.
 * ③ alpha = leftover ns / tick_ns ∈ [0,1) — sub-tick render offset.
 * ④ Sleep remaining TARGET_FPS budget before render.
 * ⑤ erase → scene_draw → HUD → doupdate.
 * ⑥ Non-blocking getch.
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

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns(); sim_accum = 0;
        }

        /* ① dt */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ② fixed-step accumulator */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ③ alpha */
        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0; fps_accum = 0;
        }

        /* ④ frame cap — sleep BEFORE render to keep terminal I/O
         * off the next frame's budget.  elapsed = wall time spent
         * on physics + accounting since frame_start.  Do NOT add
         * dt back into elapsed — that cancels the cap, sleep is
         * always 0, CPU pegs at 100 %.                            */
        int64_t elapsed = clock_ns() - frame_start;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

        /* ⑤ */
        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha);
        screen_present();

        /* ⑥ */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
