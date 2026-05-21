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
 *   §1 config   — StrategyParams + g_presets[N_STRATEGIES] table +
 *                  global combat constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — faction palette + PAIR_HUD/PAIR_HINT
 *   §4 coords   — pixel↔cell aspect bridge + Vec2 helpers
 *   §5 entity   — Warrior + Arrow structs, warrior_spawn, integrator
 *   §6 combat   — steering forces; melee FSM (advance/combat/flee
 *                  helpers); archer FSM as 5-step pipeline
 *                  (archer_handle_no_enemies / _maybe_panic_on_low_hp
 *                  / _pick_force / _tick_rally); all take
 *                  const StrategyParams *sp — no file-scope state
 *   §7 scene    — World + SimControls + Strategy + FactionStats[2]
 *                  sub-structs; Scene root; spawn_warriors +
 *                  count_alive_faction; scene_set_strategy,
 *                  scene_init, scene_add_warriors, arrows_tick (with
 *                  arrow_out_of_bounds + arrow_apply_hit +
 *                  compact_arrows), scene_tick, scene_draw
 *   §8 app      — FpsCounter + App; signals, resize, key dispatch,
 *                  HUD helpers (hud_paint_text + draw_faction_roster_*
 *                  + draw_hud_title / _fps / _hint), main game loop
 *                  (8 numbered phases)
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
 *                  weights, attack intervals).  All combat helpers
 *                  take `const StrategyParams *sp` from
 *                  scene.strategy.params, so a key-press switches
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
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 *   ── Canonical steering / flocking ──────────────────────────────
 *   [1] Reynolds, C. W. (1999), "Steering Behaviors for Autonomous
 *       Characters", Game Developers Conference 1999.  Online at
 *       https://www.red3d.com/cwr/steer/ — THE source of every
 *       steering force used here.  §6 steer_seek / _flee /
 *       _separate are 1:1 implementations of Reynolds's reference
 *       algorithms; the "Pursuit" / "Evasion" primitives are what
 *       warriors do in COMBAT / FLEE states.
 *   [2] Reynolds, C. W. (1987), "Flocks, Herds, and Schools: A
 *       Distributed Behavioral Model", SIGGRAPH '87 Computer Graphics
 *       21(4), pp. 25-34 — the original three-rule boid paper.
 *       Melee warriors use a subset (separation + arrive); archers
 *       add a flee-from-enemy term unique to ranged units.
 *
 *   ── Game-AI textbooks (steering + FSM combined) ────────────────
 *   [3] Buckland, M. (2005), "Programming Game AI by Example",
 *       Wordware — the most relevant single reference for this
 *       file.  Chapter 3 covers Reynolds steering with C++
 *       implementations; chapter 2 covers FSM-driven AI agents
 *       (Buckland calls them "West World" cowboys).  Combine the
 *       two → exactly this file's architecture.
 *   [4] Millington, I. & Funge, J. (2009), "Artificial Intelligence
 *       for Games" (2nd ed.), Morgan Kaufmann — the comprehensive
 *       game-AI textbook.  Part II ch. 3 ("Movement") is the
 *       steering-behaviour reference; Part IV ch. 11 ("Tactical
 *       and Strategic AI") covers squad/army-level behaviour
 *       beyond per-agent steering.
 *
 *   ── Finite state machines ──────────────────────────────────────
 *   [5] Rabin, S. (ed.) (2002), "AI Game Programming Wisdom",
 *       Charles River Media — the canonical FSM-in-games reference;
 *       Section 2 ("Movement and Pathfinding") and Section 3 ("AI
 *       Architectures") cover the ADVANCE / COMBAT / FLEE / DEAD
 *       pattern used by every behaviour branch in §6.
 *
 *   ── Combat attrition math ──────────────────────────────────────
 *   [6] Lanchester, F. W. (1916), "Aircraft in Warfare: The Dawn of
 *       the Fourth Arm" — Lanchester's Square Law: under modern
 *       (aimed-fire) combat, the rate of attrition is proportional
 *       to the square of force size:
 *
 *           dN_A/dt = -k · N_B,  dN_B/dt = -k · N_A
 *
 *       The n_alive[] curves this simulation produces match the
 *       Lanchester model when force imbalance is large — equal
 *       numbers produce a roughly proportional drain; doubled
 *       numbers produce a 4× advantage in attrition rate.
 *
 *   ── Game-loop / fixed-step physics ─────────────────────────────
 *   [7] Fiedler, G. (2004, updated 2014), "Fix Your Timestep!",
 *       https://gafferongames.com/post/fix_your_timestep/ — the
 *       fixed-step accumulator + sub-tick alpha-lerp pattern
 *       implemented in §8 main.  Caps dt at 100 ms to prevent
 *       the avalanche spiral on slow terminals.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *   [8] Wikipedia: "Boids", "Lanchester's laws", "Finite-state
 *       machine", "Steering behaviors".  Useful one-paragraph
 *       summaries that link out to the primary literature.
 *
 *   ── Flavour ────────────────────────────────────────────────────
 *   [9] Tolkien, J. R. R., *The Lord of the Rings* — flavour for
 *       the Gondor / Mordor faction names.  Gameplay is fictional
 *       Reynolds-AI [1] + Lanchester attrition [6], not lore-
 *       accurate.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:
 *     flocking/flocking.c    — Reynolds-style boids; the same
 *       steering primitives at a smaller scale, no FSM.
 *     flocking/crowd.c       — single-agent steering primer
 *       (seek/flee/wander) on a Person model.
 *     flocking/swarm_gen_numbers.c — 10 strategy presets dispatched
 *       by index; same StrategyParams + dispatch table pattern.
 *     flocking/shepherd.c    — Strömbom shepherd-and-sheep model;
 *       herd dynamics are the inverse of war's combat dynamics.
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
 *   melee_logic — switch on state → per-state Steer helper:
 *     ADVANCE → melee_advance (seek enemy centroid; if any enemy
 *               enters engage_range, lock on → COMBAT).
 *     COMBAT  → melee_combat (seek locked target slowly; deal
 *               ATK_DAMAGE every atk_interval s.  Target dies →
 *               ADVANCE.  hp ≤ flee_hp → FLEE).
 *     FLEE    → melee_flee (flee from nearest enemy; if d ≥
 *               safe_range, count rally_timer; rally_time → ADVANCE).
 *
 *   archer_logic — 5-step pipeline:
 *     (1) archer_handle_no_enemies — if none, glide to halt + return.
 *     (2) archer_maybe_panic_on_low_hp — transition to FLEE on
 *         hp ≤ archer_flee_hp.
 *     (3) archer_pick_force — distance ladder:
 *           d < archer_flee_range  → archer_flee_force (panic-close
 *                                     transition into FLEE)
 *           state == FLEE          → archer_flee_force (carry-over)
 *           d ≤ arrow_range        → archer_combat (shoot arrow
 *                                     every shoot_interval s + drift
 *                                     to standoff)
 *           otherwise              → archer_advance (head toward
 *                                     standoff behind enemy centroid)
 *     (4) integrate + bounce.
 *     (5) archer_tick_rally — accumulate timer at safe_range; rally
 *         to ADVANCE after rally_time.
 *
 *   After every warrior, run arrows_tick: integrate each arrow by
 *   vel·dt, hit-test against its target (within ARROW_HIT_DIST →
 *   arrow_apply_hit deals ATK_DAMAGE + HIT_FLASH_TIME), miss if
 *   off-screen, then compact_arrows shifts the active slots to the
 *   front of the pool (O(N) stable, no free-list).
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
 * StrategyParams — all tuneable combat constants for one steering
 * preset (one strategy = one row of the g_presets[] table).
 *
 * Intent
 *   Every combat helper in §6 takes `const StrategyParams *sp` and
 *   reads sp->* for its coefficients.  The Scene's `strategy`
 *   sub-struct (in §7) caches the pointer to the active preset so a
 *   key-press takes effect on the very next tick without re-indexing
 *   the table.  No file-scope strategy state.
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
  int flee_hp;
  float atk_interval;
  float speed_advance;
  float speed_flee;
  float sep_radius;
  float safe_range;
  float rally_time;
  float melee_speed;
  int archer_flee_hp;
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
#define N_STRATEGIES 6   /* number of presets in g_presets[] below */
static const StrategyParams g_presets[N_STRATEGIES] = {
    /*            engage flee atk   adv    flee  sep   safe  rally melee */
    /*            afleehp  arange  aflee standoff shoot aspd  seek  sep   flee
     */

    {"STANDARD", 40.0f, 1, 1.4f, 55.0f, 105.0f, 30.0f, 140.0f, 2.5f, 20.0f, 1,
     160.0f, 48.0f, 110.0f, 1.8f, 48.0f, 1.0f, 1.8f, 1.5f},

    {"BERSERKER", 60.0f, 0, 0.9f, 75.0f, 120.0f, 18.0f, 80.0f, 1.0f, 30.0f, 0,
     140.0f, 20.0f, 80.0f, 1.2f, 60.0f, 1.5f, 0.8f, 0.5f},

    {"SHIELD WALL", 28.0f, 1, 1.8f, 35.0f, 90.0f, 16.0f, 160.0f, 3.5f, 12.0f, 1,
     180.0f, 60.0f, 140.0f, 2.2f, 36.0f, 0.8f, 2.5f, 1.8f},

    {"GUERRILLA", 40.0f, 2, 0.8f, 65.0f, 135.0f, 35.0f, 100.0f, 1.2f, 25.0f, 2,
     150.0f, 70.0f, 120.0f, 1.4f, 70.0f, 1.2f, 1.5f, 2.2f},

    {"ARCHER FOCUS", 35.0f, 1, 1.6f, 50.0f, 100.0f, 28.0f, 130.0f, 2.0f, 18.0f,
     2, 220.0f, 35.0f, 160.0f, 1.0f, 55.0f, 1.0f, 2.0f, 1.6f},

    {"CHAOS", 80.0f, 0, 1.0f, 90.0f, 115.0f, 8.0f, 60.0f, 0.8f, 40.0f, 0,
     120.0f, 15.0f, 60.0f, 2.5f, 80.0f, 2.0f, 0.3f, 0.3f},
};

/* g_presets[] is the const table of all strategy presets, indexed by
 * Strategy.index in §9 Scene.  Strategy.params on Scene caches the
 * pointer to the active preset (= &g_presets[index]) so the combat
 * helpers in §6 avoid the array index on every read. */

/* ── army sizes + colour pair IDs — fixed across all strategies ── */
enum {
  MELEE_DEFAULT = 35,
  ARCHER_DEFAULT = 12,
  WARRIORS_MAX = 70, /* cap per faction */
  POOL_MAX = 160,    /* 2 × WARRIORS_MAX + headroom */
  REINFORCE_MELEE = 6,
  REINFORCE_ARCHER = 2,
  SIM_FPS_DEFAULT = 60,
  TARGET_FPS = 60,
  FPS_UPDATE_MS = 500,

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
  N_COLORS = 7,
  PAIR_HUD = 8,  /* bright yellow — top-centre status */
  PAIR_HINT = 9, /* bright cyan   — bottom key hint   */
};

/* Cell dimensions — physics in px, draw in cells; convert only at render */
#define CELL_W 8
#define CELL_H 16

#define GONDOR 0 /* right side — cyan melee, green archers  */
#define MORDOR 1 /* left side  — red melee,  orange archers */

/* Fixed constants (identical across all strategies) */
#define ATK_DAMAGE 1
#define HP_MAX 3
#define CORPSE_LIFETIME 4.0f
#define HIT_FLASH_TIME 0.15f

/* Arrow projectile */
#define ARROW_POOL_MAX 80
#define ARROW_TRAVEL_SPD 220.0f /* px/s, fixed regardless of strategy */
#define ARROW_HIT_DIST 14.0f    /* px, hit radius around target */

/* Timing */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                            */
/* ===================================================================== */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
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
static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(1, 196, -1);
    init_pair(2, 208, -1);
    init_pair(3, 226, -1);
    init_pair(4, 46, -1);
    init_pair(5, 51, -1);
    init_pair(6, 33, -1);
    init_pair(7, 201, -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(1, COLOR_RED, -1);
    init_pair(2, COLOR_RED, -1);
    init_pair(3, COLOR_YELLOW, -1);
    init_pair(4, COLOR_GREEN, -1);
    init_pair(5, COLOR_CYAN, -1);
    init_pair(6, COLOR_BLUE, -1);
    init_pair(7, COLOR_MAGENTA, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* ===================================================================== */
/* §4  coords & vec2                                                    */
/* ===================================================================== */

static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/* Round-half-up avoids oscillation at exact half-pixel boundaries. */
static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/*
 * Vec2 — 2-D float vector used everywhere physics or geometry shows
 * up: position, velocity, force, offset, centroid.
 *
 * Intent
 *   Functions return Vec2 BY VALUE (no out-params); the v2* helpers
 *   below compose into terse chains that mirror the math notation
 *   they implement:
 *
 *       Vec2 force = v2add(v2scale(seek_f, sp->w_seek),
 *                          v2scale(sep_f,  sp->w_sep));
 *
 *   reads as `force = w_seek·seek + w_sep·sep` to a math-fluent reader.
 *
 * Why a struct (not float[2] or two scalars)
 *   • Named type makes every signature read in the algorithm's
 *     vocabulary: `steer_seek(pos, vel, target, speed)` rather than
 *     `steer_seek(px, py, vx, vy, tx, ty, speed)`.
 *   • Return by value works without spilling to memory on x86-64 +
 *     ARM ABIs (8 bytes = one register pair); no perf penalty.
 *   • Eliminates two-out-parameter idioms in every steering helper.
 *
 * Members
 *   x, y    Cartesian components in PIXEL space (units = px).  Cell-
 *           space conversion happens only inside scene_draw via
 *           px_to_cell_x/y.  Physics never sees cells.
 *
 * References
 *   [1] Reynolds 1999 — every steering primitive returns a Vec2
 *       force; this file's §6 mirrors that API shape.
 *   [3] Buckland *Programming Game AI by Example* Ch. 1 — the
 *       Vector2D class this struct is the C analogue of.
 */
typedef struct {
  float x, y;
} Vec2;

static inline Vec2 v2(float x, float y) { return (Vec2){x, y}; }
static inline Vec2 v2add(Vec2 a, Vec2 b) { return v2(a.x + b.x, a.y + b.y); }
static inline Vec2 v2sub(Vec2 a, Vec2 b) { return v2(a.x - b.x, a.y - b.y); }
static inline Vec2 v2scale(Vec2 v, float s) { return v2(v.x * s, v.y * s); }
static inline float v2len(Vec2 v) { return sqrtf(v.x * v.x + v.y * v.y); }
static inline float v2len2(Vec2 v) { return v.x * v.x + v.y * v.y; }

static inline Vec2 v2norm(Vec2 v) {
  float l = v2len(v);
  return (l > 0.001f) ? v2scale(v, 1.0f / l) : v2(0, 0);
}

static inline Vec2 v2clamp_len(Vec2 v, float max_len) {
  float l = v2len(v);
  return (l > max_len) ? v2scale(v2norm(v), max_len) : v;
}

/* Elastic wall bounce: velocity component flips on contact. */
static void bounce_pos(Vec2 *pos, Vec2 *vel, float ww, float wh) {
  if (pos->x < 0) {
    pos->x = 0;
    vel->x = fabsf(vel->x);
  }
  if (pos->x >= ww) {
    pos->x = ww - 1;
    vel->x = -fabsf(vel->x);
  }
  if (pos->y < 0) {
    pos->y = 0;
    vel->y = fabsf(vel->y);
  }
  if (pos->y >= wh) {
    pos->y = wh - 1;
    vel->y = -fabsf(vel->y);
  }
}

/* ===================================================================== */
/* §5  entity                                                           */
/* ===================================================================== */

/*
 * UnitType — pick which behaviour state machine drives this warrior.
 *
 * Intent
 *   Every Warrior runs ONE of two distinct combat loops:
 *
 *     UNIT_MELEE  : ADVANCE toward nearest enemy → COMBAT (brawl)
 *                   → FLEE on low HP → DEAD.  No ranged action.
 *
 *     UNIT_ARCHER : ADVANCE toward standoff range → COMBAT (shoot
 *                   arrows at intervals) → FLEE if enemy too close
 *                   or HP low → DEAD.  Never engages in melee.
 *
 *   warrior_tick dispatches to melee_logic or archer_logic based on
 *   this enum.  The two units share Steer + WarriorState but have
 *   different attack distance, attack interval, and flee triggers.
 *
 * Why a flat enum (not a behaviour vtable or class hierarchy)
 *   Only two unit types and they're branching cleanly on a single
 *   bit — a vtable is overkill.  Adding a third unit type (cavalry,
 *   mage, …) would warrant rethinking; at two, the if-else is
 *   clearer than an indirection.
 */
typedef enum { UNIT_MELEE = 0, UNIT_ARCHER } UnitType;

/*
 * WarriorState — Buckland [3] / Rabin [5] style finite-state machine
 * driving each Warrior's behaviour.
 *
 * State diagram
 *
 *                  enemy in engage_range
 *      ┌─────────┐ ─────────────────────► ┌────────┐
 *      │ ADVANCE │                        │ COMBAT │
 *      └─────────┘ ◄───── rally ───────── └────────┘
 *           │                                   │
 *           │       hp ≤ flee_hp                │
 *           └────────────────┐  ┌───────────────┘
 *                            ▼  ▼
 *                          ┌──────┐    hp ≤ 0    ┌──────┐
 *                          │ FLEE │ ───────────► │ DEAD │
 *                          └──────┘              └──────┘
 *                                                (corpse: CORPSE_LIFETIME)
 *
 * Members
 *   STATE_ADVANCE  Marching or repositioning.  Force = seek(centroid
 *                  of nearest 3 enemies) + separation.  HUD glyph
 *                  rendered at full HP-driven intensity.
 *
 *   STATE_COMBAT   Locked on `target_idx`.  Melee: brawl in place,
 *                  apply ATK_DAMAGE every atk_interval.  Archer: stay
 *                  in arrow_range, shoot every shoot_interval.
 *
 *   STATE_FLEE     Routing.  Force = flee(centroid of nearest enemies)
 *                  at speed_flee (must exceed speed_advance to be
 *                  useful).  Renderer adds A_BLINK; HP-low warriors
 *                  flash red.  Rallies back to ADVANCE after
 *                  rally_time at distance ≥ safe_range.
 *
 *   STATE_DEAD     HP == 0.  Body stays on screen for CORPSE_LIFETIME
 *                  seconds as a '%' glyph, then is excluded from the
 *                  per-tick pool scan.  Slot is NEVER freed — n_total
 *                  is monotonic, simpler than holey-pool bookkeeping
 *                  and the pool is sized for worst-case spawns.
 *
 * Invariants
 *   STATE_ADVANCE = 0 deliberately, so memset-zeroed warriors start
 *   in ADVANCE without an explicit initialiser (used by scene_init).
 *
 * References
 *   [3] Buckland *Programming Game AI by Example* Ch. 2 — the
 *       canonical "West World" cowboy FSM that this design follows.
 *   [5] Rabin (ed.) *AI Game Programming Wisdom* §3 — survey of
 *       agent FSM architectures and state-transition idioms.
 */
typedef enum {
  STATE_ADVANCE = 0, /* marching / repositioning */
  STATE_COMBAT,      /* engaged: brawling (melee) or shooting (archer) */
  STATE_FLEE,        /* routing */
  STATE_DEAD,        /* HP == 0; showing corpse */
} WarriorState;

/*
 * Warrior — one soldier in the battle.  The atomic unit of war.c.
 *
 * Intent
 *   N_AGENTS warriors share the pool[] array, interleaved by spawn
 *   order (Gondor melee, Gondor archers, Mordor melee, Mordor archers,
 *   reinforcements …).  Each tick warrior_tick runs the appropriate
 *   logic (melee or archer) which drives the FSM in `state`, sums a
 *   weighted Reynolds force (steer_seek + steer_flee + steer_separate
 *   from §6), and integrates via warrior_step.  The renderer reads
 *   prev_pos + pos and alpha-lerps for smooth motion.
 *
 * Why prev_pos (a snapshot, not just current pos)
 *   Physics ticks at sim_fps (60 Hz default); the renderer can draw at
 *   any rate.  scene_draw lerps prev_pos → pos by sub-tick alpha
 *   (Fiedler [7]).  Without prev_pos, fast warriors would jitter
 *   visibly between tick boundaries.
 *
 * Why fields are GROUPED by lifecycle (not by type)
 *   • IDENTITY (faction, unit_type, glyph, color_pair) is set ONCE at
 *     warrior_spawn and is read-only thereafter.
 *   • KINEMATIC STATE (pos/prev_pos/vel) is rewritten EVERY tick by
 *     warrior_step.
 *   • FSM STATE (hp, state, target_idx) is written by combat code in
 *     §6, read by the HUD and renderer.
 *   • TIMERS are countdown clocks, ticked down by warrior_tick.
 *   Grouping by lifecycle makes "who writes this field" answerable at
 *   a glance — the most common debugging question.
 *
 * Why slot never frees on death
 *   STATE_DEAD warriors stay in the pool for CORPSE_LIFETIME (visible
 *   '%' glyph for narrative effect) and stay forever in `n_total`
 *   (never decremented).  Holey-pool bookkeeping would add a free-list
 *   and a compaction pass for no real win; POOL_MAX is sized for
 *   worst-case spawn counts so we just live with the dead slots.
 *
 * Members
 *   ── kinematic state (rewritten every tick) ──────────────────────
 *   pos          Current position in PIXEL space (units = px).
 *                Toroidally bounced at world edges via bounce_pos.
 *   prev_pos     Position at start of this tick — snapshotted before
 *                pos is integrated, for the renderer's alpha lerp.
 *   vel          Velocity in PIXELS PER SECOND.  Capped by the
 *                Steer.max_spd from the active state helper.
 *
 *   ── identity (set once at warrior_spawn) ────────────────────────
 *   faction      GONDOR (0) or MORDOR (1) — never changes.
 *   unit_type    UNIT_MELEE or UNIT_ARCHER — picks the state machine.
 *   glyph        ASCII character for rendering this warrior.
 *                Gondor melee: A..Z; Gondor archers: lowercase letters
 *                cycled.  Mordor: matching but tinted with PAIR_MORDOR.
 *   color_pair   ncurses pair (1..) for this warrior's faction tint.
 *
 *   ── FSM state (written by combat helpers in §6) ─────────────────
 *   hp           Current health in [0, HP_MAX].  hp ≤ 0 → STATE_DEAD.
 *   state        Current WarriorState (see enum doc above).
 *   target_idx   Pool index of the locked enemy in COMBAT, or −1
 *                when no target (ADVANCE/FLEE, or archer between shots).
 *
 *   ── timers (countdown seconds, ticked by warrior_tick) ──────────
 *   atk_timer    Seconds until next melee hit / arrow shot.
 *   rally_timer  Seconds at safe distance ≥ safe_range; once it
 *                crosses rally_time the warrior rallies back to
 *                ADVANCE.
 *   dead_timer   Seconds of corpse remaining (counts down from
 *                CORPSE_LIFETIME on death).
 *   hit_timer    Seconds of '*' flash overlay remaining after an
 *                arrow strike (visual feedback only).
 *
 * Invariants
 *   0 ≤ hp ≤ HP_MAX.
 *   state == STATE_DEAD  iff  hp == 0.
 *   target_idx ∈ [0, n_total)  OR  target_idx == −1.
 *   0 ≤ faction ≤ 1.
 *
 * References
 *   [1] Reynolds 1999 — every steering force a warrior applies is a
 *       Reynolds primitive (seek, flee, separate).
 *   [3] Buckland *Programming Game AI by Example* — the entity's
 *       (kinematic state + identity + FSM state + timers) layout
 *       follows Buckland's autonomous-agent design pattern (Ch. 1-3).
 *   [7] Fiedler 2014 — the prev_pos / alpha-lerp pattern.
 */
typedef struct {
  /* kinematic state — alpha-lerped on draw */
  Vec2 pos;
  Vec2 prev_pos;
  Vec2 vel;

  /* identity — set once at spawn */
  int faction;        /* GONDOR or MORDOR             */
  UnitType unit_type; /* UNIT_MELEE or UNIT_ARCHER    */
  char glyph;         /* ASCII glyph drawn for unit   */
  int color_pair;     /* ncurses pair for this unit   */

  /* HP + state machine */
  int hp;             /* 0..HP_MAX                    */
  WarriorState state; /* ADVANCE/COMBAT/FLEE/DEAD     */
  int target_idx;     /* pool index of locked enemy
                       * (-1 = none; archers ignore)  */

  /* timers (seconds) */
  float atk_timer;   /* until next melee hit / shot  */
  float rally_timer; /* time at safe distance         */
  float dead_timer;  /* corpse remaining lifetime     */
  float hit_timer;   /* arrow-strike '*' flash        */
} Warrior;

/*
 * Arrow — one '-' projectile fired by an archer.
 *
 * Intent
 *   Archers in STATE_COMBAT shoot one Arrow per shoot_interval at
 *   their locked target.  The Arrow flies straight (constant velocity,
 *   no gravity — this is ASCII, not realistic ballistics) and tests
 *   proximity to its target_idx each tick.  On hit, the target takes
 *   ATK_DAMAGE and a '*' flash overlays the strike point.
 *
 * Why a separate pool (not embedded in Warrior)
 *   • Arrows OUTLIVE the warrior that fired them — an archer can die
 *     mid-shot and the arrow still arrives.  Decoupling arrow life
 *     from warrior life keeps both lifecycles simple.
 *   • Arrows are SPARSE — most warriors don't have one in flight at
 *     any given moment.  A dense arrows[] array compacted each tick
 *     (active flag → memmove on the C side) avoids paying for empty
 *     arrow slots in every warrior struct.
 *
 * Why pin the target_idx (rather than re-acquire each tick)
 *   The shot is committed to its original target at fire time.  If
 *   the target dies mid-flight, the arrow continues to its dead
 *   warrior's last position and harmlessly expires off-screen.
 *   This matches the physical metaphor and avoids the "homing arrow"
 *   behaviour that re-targeting would produce.
 *
 * Members
 *   pos          Current position in PIXEL space.
 *   vel          Velocity in PIXELS PER SECOND; magnitude is exactly
 *                ARROW_TRAVEL_SPD at the moment of firing, direction
 *                = unit(target_pos − fire_pos).  Constant thereafter.
 *   target_idx   Pool index of the warrior this arrow is aimed at.
 *                Proximity-checked each tick for hit detection.
 *   faction      Faction of the FIRING archer (so an arrow doesn't
 *                damage its own side on a stray miss).
 *   active       true while in flight; cleared on hit / out-of-bounds.
 *                arrows_tick compacts the inactive slots away.
 *
 * Invariants
 *   active → target_idx ∈ [0, n_total).
 *   |vel| ≈ ARROW_TRAVEL_SPD while active.
 */
typedef struct {
  Vec2 pos;
  Vec2 vel;
  int target_idx;
  int faction;
  bool active;
} Arrow;

static const char GONDOR_MELEE_GLYPHS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ";
static const char MORDOR_MELEE_GLYPHS[] = "abcdefghijklmnopqrstuvwxyz";

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

/*
 * warrior_spawn — initialise one pool slot.
 *
 * Spawn zones: melee outer 30%, archers deeper 15% of their half.
 * atk_timer is randomised to stagger first attacks — organic rhythm.
 */
static void warrior_spawn(Warrior *w, int id, int faction, UnitType unit_type,
                          const StrategyParams *sp, float ww, float wh) {
  float x_lo, x_hi;
  if (faction == GONDOR) {
    x_lo = (unit_type == UNIT_ARCHER) ? ww * 0.84f : ww * 0.60f;
    x_hi = (unit_type == UNIT_ARCHER) ? ww * 0.97f : ww * 0.82f;
  } else {
    x_lo = (unit_type == UNIT_ARCHER) ? ww * 0.03f : ww * 0.18f;
    x_hi = (unit_type == UNIT_ARCHER) ? ww * 0.16f : ww * 0.40f;
  }

  w->pos = v2(x_lo + randf() * (x_hi - x_lo), randf() * wh);
  w->prev_pos = w->pos;
  w->vel = v2(0, 0);
  w->faction = faction;
  w->unit_type = unit_type;
  w->hp = HP_MAX;
  w->atk_timer =
      (unit_type == UNIT_ARCHER ? sp->shoot_interval : sp->atk_interval) *
      randf();
  w->rally_timer = 0.0f;
  w->target_idx = -1;
  w->state = STATE_ADVANCE;
  w->dead_timer = 0.0f;
  w->hit_timer = 0.0f;

  if (unit_type == UNIT_ARCHER) {
    w->glyph = (faction == GONDOR) ? '@' : '%';
    w->color_pair = (faction == GONDOR) ? 4 : 2;
  } else {
    w->glyph =
        (faction == GONDOR)
            ? GONDOR_MELEE_GLYPHS[id % (int)(sizeof(GONDOR_MELEE_GLYPHS) - 1)]
            : MORDOR_MELEE_GLYPHS[id % (int)(sizeof(MORDOR_MELEE_GLYPHS) - 1)];
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
static void warrior_step(Warrior *w, Vec2 accel, float max_speed, float dt) {
  w->vel = v2clamp_len(v2add(w->vel, v2scale(accel, dt)), max_speed);
  w->prev_pos = w->pos;
  w->pos = v2add(w->pos, v2scale(w->vel, dt));
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
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed) {
  Vec2 desired = v2scale(v2norm(v2sub(target, pos)), speed);
  return v2sub(desired, vel);
}

/* steer_flee — steer AWAY from threat; negated seek. */
static Vec2 steer_flee(Vec2 pos, Vec2 vel, Vec2 threat, float speed) {
  return v2scale(steer_seek(pos, vel, threat, speed), -1.0f);
}

/*
 * steer_separate — repulsion from same-faction allies within sep_radius.
 *
 * Repulsion strength = (sep_radius − dist) / sep_radius ∈ (0,1].
 * Scaled by speed_advance so force is in velocity-compatible units.
 * Only same-faction warriors push each other (fight through enemies).
 */
static Vec2 steer_separate(const Warrior *pool, int n_total, int self,
                            const StrategyParams *sp) {
  Vec2 force = v2(0, 0);
  const Warrior *me = &pool[self];
  for (int i = 0; i < n_total; i++) {
    if (i == self)
      continue;
    if (pool[i].faction != me->faction)
      continue;
    if (pool[i].state == STATE_DEAD)
      continue;
    Vec2 away = v2sub(me->pos, pool[i].pos);
    float d = v2len(away);
    if (d < sp->sep_radius && d > 0.001f) {
      float strength = (sp->sep_radius - d) / sp->sep_radius;
      force =
          v2add(force, v2scale(v2norm(away), strength * sp->speed_advance));
    }
  }
  return force;
}

/*
 * enemy_centroid — average position of all living enemies.
 * Warriors march toward the mass (not one target) to form a battle line.
 * Falls back to world centre when no enemies remain.
 */
static Vec2 enemy_centroid(const Warrior *pool, int n_total, int faction,
                           float ww, float wh) {
  int efac = 1 - faction;
  Vec2 sum = v2(0, 0);
  int n = 0;
  for (int i = 0; i < n_total; i++) {
    if (pool[i].faction != efac)
      continue;
    if (pool[i].state == STATE_DEAD)
      continue;
    sum = v2add(sum, pool[i].pos);
    n++;
  }
  return n ? v2scale(sum, 1.0f / n) : v2(ww * 0.5f, wh * 0.5f);
}

/* nearest_enemy_idx — pool index of the closest living enemy; -1 if none. */
static int nearest_enemy_idx(const Warrior *pool, int n_total, int self) {
  int efac = 1 - pool[self].faction;
  int best = -1;
  float best_dist2 = FLT_MAX;
  for (int i = 0; i < n_total; i++) {
    if (pool[i].faction != efac)
      continue;
    if (pool[i].state == STATE_DEAD)
      continue;
    float d2 = v2len2(v2sub(pool[i].pos, pool[self].pos));
    if (d2 < best_dist2) {
      best_dist2 = d2;
      best = i;
    }
  }
  return best;
}

/*
 * Steer — the (force, max_speed) result returned by every per-state
 * behaviour helper in §6.
 *
 * Intent
 *   Each helper (melee_advance, melee_combat, melee_flee,
 *   archer_advance, archer_combat, archer_flee_force) is a pure
 *   function of `(Warrior *, pool, sp, world, dt)` returning a
 *   Steer.  The dispatcher (melee_logic / archer_logic) then calls
 *   warrior_step with steer.force at steer.max_spd and bounces off
 *   walls.
 *
 *   Returning a struct (vs two output pointers) keeps call sites
 *   one-line and makes the contract explicit at the type level:
 *   every helper MUST set both fields.  STATE_ADVANCE returns
 *   speed_advance; STATE_FLEE returns speed_flee (higher); STATE_
 *   COMBAT returns 0 (warrior stands still while attacking).
 *
 * Why TWO fields (not just force)
 *   Speed cap varies by state: a fleeing warrior sprints (speed_flee
 *   may be 1.5× the advance speed), an engaged warrior stands still.
 *   Embedding the cap in the Steer result lets each per-state helper
 *   pick its own without the dispatcher knowing the state.
 *
 * Members
 *   force      Weighted force vector to integrate this tick (px/s²).
 *              Computed as a Reynolds [1] weighted sum of the
 *              steering primitives needed by this state.
 *   max_spd    Hard cap on |vel| after integration (px/s).
 *
 * References
 *   [1] Reynolds 1999 — the "vehicle.steer_*()" methods return a
 *       force vector; this struct adds the max_speed component
 *       because state-dependent speed caps weren't part of Reynolds's
 *       single-vehicle assumption.
 *   [3] Buckland *Programming Game AI by Example* — Buckland's
 *       SteeringBehavior class returns Vector2D; the addition of
 *       max_spd here is the project-specific extension.
 */
typedef struct {
  Vec2 force;
  float max_spd;
} Steer;

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
                           int self, const StrategyParams *sp,
                           float ww, float wh) {
  Vec2 centroid = enemy_centroid(pool, n_total, w->faction, ww, wh);
  int ne = nearest_enemy_idx(pool, n_total, self);

  if (ne >= 0 && v2len(v2sub(pool[ne].pos, w->pos)) < sp->engage_range) {
    /* lock on → COMBAT next tick.  No movement this tick;
     * atk_timer reset prevents an instant-hit on transition. */
    w->state = STATE_COMBAT;
    w->target_idx = ne;
    w->atk_timer = sp->atk_interval;
    return (Steer){v2(0, 0), sp->speed_advance};
  }

  Vec2 force =
      v2add(v2scale(steer_seek(w->pos, w->vel, centroid, sp->speed_advance),
                    sp->w_seek),
            v2scale(steer_separate(pool, n_total, self, sp), sp->w_sep));
  return (Steer){force, sp->speed_advance};
}

/*
 * melee_combat — slow footwork toward the locked target; deal
 * ATK_DAMAGE every atk_interval seconds.  Transitions:
 *   target died    → ADVANCE
 *   hp ≤ flee_hp   → FLEE (drop target)
 */
static Steer melee_combat(Warrior *w, Warrior *pool, int n_total, int self,
                          const StrategyParams *sp, float dt) {
  if (w->target_idx < 0) {
    w->state = STATE_ADVANCE;
    return (Steer){v2(0, 0), sp->speed_advance};
  }
  if (w->hp <= sp->flee_hp) {
    w->state = STATE_FLEE;
    w->target_idx = -1;
    return (Steer){v2(0, 0), sp->speed_flee};
  }

  /* Damage tick: consume the timer; on rollover, hit + reset. */
  w->atk_timer -= dt;
  if (w->atk_timer <= 0.0f) {
    w->atk_timer = sp->atk_interval;
    pool[w->target_idx].hp -= ATK_DAMAGE;
  }

  Vec2 tgt = pool[w->target_idx].pos;
  Vec2 force = v2add(
      v2scale(steer_seek(w->pos, w->vel, tgt, sp->melee_speed), sp->w_seek),
      v2scale(steer_separate(pool, n_total, self, sp),
              sp->w_sep * 0.4f) /* reduced: stay near target */
  );
  return (Steer){force, sp->melee_speed * 1.5f};
}

/*
 * melee_flee — sprint away from the nearest enemy.  Once at
 * safe_range, count rally_timer; on rally_time → ADVANCE.
 */
static Steer melee_flee(Warrior *w, const Warrior *pool, int n_total, int self,
                        const StrategyParams *sp, float dt) {
  int ne = nearest_enemy_idx(pool, n_total, self);
  if (ne < 0) {
    w->state = STATE_ADVANCE;
    return (Steer){v2(0, 0), sp->speed_advance};
  }

  float d = v2len(v2sub(pool[ne].pos, w->pos));
  if (d >= sp->safe_range) {
    /* Safe — count rally time and gently bleed velocity. */
    w->rally_timer += dt;
    w->vel = v2scale(w->vel, 0.92f);
    if (w->rally_timer >= sp->rally_time) {
      w->state = STATE_ADVANCE;
      w->rally_timer = 0.0f;
    }
    return (Steer){v2(0, 0), sp->speed_flee};
  }

  /* Still in danger — flee under full force. */
  w->rally_timer = 0.0f;
  Vec2 force =
      v2add(v2scale(steer_flee(w->pos, w->vel, pool[ne].pos, sp->speed_flee),
                    sp->w_flee),
            v2scale(steer_separate(pool, n_total, self, sp), sp->w_sep));
  return (Steer){force, sp->speed_flee};
}

/*
 * melee_logic — dispatcher.  Picks the per-state helper, applies its
 * Steer through warrior_step + bounce_pos.  All combat coefficients
 * come through `const StrategyParams *sp`, so a strategy change takes
 * effect on the very next tick.
 */
static void melee_logic(Warrior *pool, int n_total, int self,
                        const StrategyParams *sp, float ww, float wh, float dt) {
  Warrior *w = &pool[self];
  Steer s = {v2(0, 0), sp->speed_advance};

  switch (w->state) {
  case STATE_ADVANCE:
    s = melee_advance(w, pool, n_total, self, sp, ww, wh);
    break;
  case STATE_COMBAT:
    s = melee_combat(w, pool, n_total, self, sp, dt);
    break;
  case STATE_FLEE:
    s = melee_flee(w, pool, n_total, self, sp, dt);
    break;
  default:
    break;
  }

  warrior_step(w, s.force, s.max_spd, dt);
  bounce_pos(&w->pos, &w->vel, ww, wh);
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
                         Arrow *arrows, int *n_arrows) {
  if (*n_arrows >= ARROW_POOL_MAX)
    return;
  Vec2 to_tgt = v2norm(v2sub(pool[ne].pos, w->pos));
  arrows[*n_arrows] = (Arrow){
      .pos = w->pos,
      .vel = v2scale(to_tgt, ARROW_TRAVEL_SPD),
      .target_idx = ne,
      .faction = w->faction,
      .active = true,
  };
  (*n_arrows)++;
}

/*
 * archer_flee_force — flee force from nearest enemy + ally separation.
 * Used by both the close-range panic path and the HP-flee carry-over
 * path; the caller picks which.
 */
static Steer archer_flee_force(const Warrior *w, const Warrior *pool,
                               int n_total, int self,
                               const StrategyParams *sp, int ne) {
  Vec2 force =
      v2add(v2scale(steer_flee(w->pos, w->vel, pool[ne].pos, sp->speed_flee),
                    sp->w_flee),
            v2scale(steer_separate(pool, n_total, self, sp), sp->w_sep));
  return (Steer){force, sp->speed_flee};
}

/*
 * archer_combat — within arrow_range: hold standoff (stand_off_dist
 * from the nearest enemy) and shoot every shoot_interval seconds.
 */
static Steer archer_combat(Warrior *w, const Warrior *pool, int n_total,
                           int self, const StrategyParams *sp,
                           int ne, Vec2 away_from_enemy, float dt,
                           Arrow *arrows, int *n_arrows) {
  w->state = STATE_COMBAT;
  w->atk_timer -= dt;
  if (w->atk_timer <= 0.0f) {
    w->atk_timer = sp->shoot_interval;
    archer_shoot(w, pool, ne, arrows, n_arrows);
  }

  Vec2 standoff =
      v2add(pool[ne].pos, v2scale(away_from_enemy, sp->stand_off_dist));
  Vec2 force = v2add(
      v2scale(steer_seek(w->pos, w->vel, standoff, sp->archer_speed * 0.4f),
              sp->w_seek),
      v2scale(steer_separate(pool, n_total, self, sp), sp->w_sep * 0.5f));
  return (Steer){force, sp->archer_speed * 0.5f};
}

/*
 * archer_advance — beyond arrow_range: head toward a standoff point
 * BEHIND the enemy centroid (i.e. on this archer's own side of the
 * field).  This keeps archers in their half rather than charging in.
 */
static Steer archer_advance(Warrior *w, const Warrior *pool, int n_total,
                            int self, const StrategyParams *sp,
                            float ww, float wh) {
  w->state = STATE_ADVANCE;
  Vec2 centroid = enemy_centroid(pool, n_total, w->faction, ww, wh);
  Vec2 safe_dir = v2norm(v2sub(w->pos, centroid));
  Vec2 advance_tgt = v2add(centroid, v2scale(safe_dir, sp->stand_off_dist));
  Vec2 force =
      v2add(v2scale(steer_seek(w->pos, w->vel, advance_tgt, sp->archer_speed),
                    sp->w_seek),
            v2scale(steer_separate(pool, n_total, self, sp), sp->w_sep));
  return (Steer){force, sp->archer_speed};
}

#define ARCHER_IDLE_DAMPING  0.92f   /* per-tick velocity decay when no enemies */

/*
 * archer_handle_no_enemies — STATE_ADVANCE + glide-to-stop.  Used
 * when nearest_enemy_idx returns −1 (the last enemy of the other
 * faction is dead).
 */
static void archer_handle_no_enemies(Warrior *w, const StrategyParams *sp,
                                      float ww, float wh, float dt) {
  w->state = STATE_ADVANCE;
  w->vel   = v2scale(w->vel, ARCHER_IDLE_DAMPING);
  warrior_step(w, v2(0, 0), sp->archer_speed, dt);
  bounce_pos(&w->pos, &w->vel, ww, wh);
}

/*
 * archer_maybe_panic_on_low_hp — HP-driven FLEE transition.  Doesn't
 * return a force; the caller picks the flee force below based on the
 * (now-set) state.  Resets rally_timer so the warrior must EARN the
 * way back to ADVANCE by spending rally_time at safe_range.
 */
static void archer_maybe_panic_on_low_hp(Warrior *w, const StrategyParams *sp) {
  bool flee_enabled       = sp->archer_flee_hp > 0;
  bool hp_below_threshold = w->hp <= sp->archer_flee_hp;
  bool not_already_fleeing = w->state != STATE_FLEE;

  if (flee_enabled && hp_below_threshold && not_already_fleeing) {
    w->state       = STATE_FLEE;
    w->rally_timer = 0.0f;
  }
}

/*
 * archer_tick_rally — once per tick, if the warrior is FLEEING and far
 * enough from its last threat (≥ safe_range), accumulate rally_timer.
 * After rally_time consecutive seconds at safe distance, return to
 * STATE_ADVANCE.  Any tick inside safe_range RESETS the timer — so a
 * partial rally is wasted if the warrior dips back into danger.
 */
static void archer_tick_rally(Warrior *w, const Warrior *enemy,
                                const StrategyParams *sp, float dt) {
  if (w->state != STATE_FLEE) return;

  float distance_to_threat = v2len(v2sub(enemy->pos, w->pos));
  bool  at_safe_range      = distance_to_threat >= sp->safe_range;

  if (!at_safe_range) {
    w->rally_timer = 0.0f;        /* dipped back into danger — restart timer */
    return;
  }

  w->rally_timer += dt;
  if (w->rally_timer >= sp->rally_time) {
    w->state       = STATE_ADVANCE;
    w->rally_timer = 0.0f;
  }
}

/*
 * archer_pick_force — choose which combat-tier helper produces this
 * tick's steering force, based on (state, distance to nearest enemy).
 *
 *   panic-close  : within archer_flee_range  → archer_flee_force
 *   carry-over   : still in STATE_FLEE       → archer_flee_force
 *   shoot range  : within arrow_range        → archer_combat
 *   otherwise                                → archer_advance
 *
 * Also sets state to STATE_FLEE on the panic-close transition (the
 * other branches don't change state).
 */
static Steer archer_pick_force(Warrior *w, Warrior *pool, int n_total,
                                int self, const StrategyParams *sp,
                                int ne, Vec2 away_from_enemy, float dist,
                                float ww, float wh, float dt,
                                Arrow *arrows, int *n_arrows) {
  bool enemy_inside_panic_zone = dist < sp->archer_flee_range;
  bool enemy_inside_arrow_zone = dist <= sp->arrow_range;

  if (enemy_inside_panic_zone) {
    /* close-range panic — transition INTO flee state */
    w->state       = STATE_FLEE;
    w->rally_timer = 0.0f;
    return archer_flee_force(w, pool, n_total, self, sp, ne);
  }

  if (w->state == STATE_FLEE) {
    /* HP-flee carry-over: keep fleeing even outside the panic zone */
    return archer_flee_force(w, pool, n_total, self, sp, ne);
  }

  if (enemy_inside_arrow_zone) {
    return archer_combat(w, pool, n_total, self, sp, ne, away_from_enemy, dt,
                         arrows, n_arrows);
  }

  return archer_advance(w, pool, n_total, self, sp, ww, wh);
}

/*
 * archer_logic — dispatcher.  Order of decisions (matters):
 *
 *   (1) No enemies left → coast to a halt and bail.
 *   (2) Low-HP panic → transition into STATE_FLEE (state mutation only,
 *       force is picked below).
 *   (3) Pick the appropriate steering force based on (state, distance).
 *   (4) Integrate + bounce off walls.
 *   (5) If still FLEEING, advance the rally clock; rally back to
 *       ADVANCE if we've spent rally_time at safe_range.
 */
static void archer_logic(Warrior *pool, int n_total, int self,
                         const StrategyParams *sp, float ww,
                         float wh, float dt, Arrow *arrows, int *n_arrows) {
  Warrior *w = &pool[self];

  /* (1) no enemies left */
  int ne = nearest_enemy_idx(pool, n_total, self);
  if (ne < 0) { archer_handle_no_enemies(w, sp, ww, wh, dt); return; }

  /* (2) HP panic — state mutation; force chosen below */
  archer_maybe_panic_on_low_hp(w, sp);

  /* (3) pick steering force */
  float dist             = v2len(v2sub(pool[ne].pos, w->pos));
  Vec2  away_from_enemy  = v2norm(v2sub(w->pos, pool[ne].pos));
  Steer s = archer_pick_force(w, pool, n_total, self, sp, ne, away_from_enemy,
                               dist, ww, wh, dt, arrows, n_arrows);

  /* (4) integrate + bounce */
  warrior_step(w, s.force, s.max_spd, dt);
  bounce_pos(&w->pos, &w->vel, ww, wh);

  /* (5) rally clock (no-op unless still FLEEING) */
  archer_tick_rally(w, &pool[ne], sp, dt);
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
                         const StrategyParams *sp,
                         float ww, float wh, float dt,
                         int kills_per_faction[2],
                         Arrow *arrows, int *n_arrows) {
  Warrior *w = &pool[self];

  if (w->hit_timer > 0.0f)
    w->hit_timer -= dt;

  if (w->state == STATE_DEAD) {
    w->dead_timer -= dt;
    return;
  }

  if (w->hp <= 0) {
    w->state = STATE_DEAD;
    w->dead_timer = CORPSE_LIFETIME;
    kills_per_faction[1 - w->faction]++;  /* enemy of dying warrior gains a kill */
    return;
  }

  if (w->target_idx >= 0 && pool[w->target_idx].state == STATE_DEAD)
    w->target_idx = -1;

  if (w->unit_type == UNIT_ARCHER)
    archer_logic(pool, n_total, self, sp, ww, wh, dt, arrows, n_arrows);
  else
    melee_logic(pool, n_total, self, sp, ww, wh, dt);
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
/*
 * World — pixel-space simulation extent.
 *
 * Intent
 *   Bundles the (width, height) pair that every combat helper needs
 *   for boundary clamping and centroid computations.  Before this
 *   struct, helpers carried `float ww, float wh` as two parameters —
 *   ~30 call sites in this file.  Bundling makes the data flow
 *   reachable through one Scene field.
 *
 * Members
 *   width   World width in pixels (= cols × CELL_W).
 *   height  World height in pixels (= rows × CELL_H).
 *
 * Invariants
 *   width > 0, height > 0.  Refreshed in scene_tick + scene_init.
 */
typedef struct {
  float width;
  float height;
} World;

/*
 * SimControls — user-facing playback knobs.
 *
 * Intent
 *   Currently just one flag, but bundled into a named sub-struct for
 *   symmetry with the other files in this project (slime_mold.c,
 *   shepherd.c, etc.) and to give the future home for any additional
 *   user toggles (slow-motion, gore enable, etc.) a clear place.
 *
 * Members
 *   paused   true → scene_tick early-returns; HUD shows "PAUSED".
 *            Toggled by SPACE.
 */
typedef struct {
  bool paused;
} SimControls;

/*
 * Strategy — which of the N_STRATEGIES presets is currently active.
 *
 * Intent
 *   The strategy choice is TWO pieces of state that MUST stay in
 *   lockstep: an index (for cycling with 1..6 keys and for HUD
 *   display) and a pointer to the active StrategyParams (read by
 *   every combat helper in §6).  Bundling them prevents the historical
 *   bug of the index pointing one place while the params pointer
 *   points somewhere else after a key dispatch.
 *
 * Members
 *   index    Currently active preset in [0, N_STRATEGIES).  1..6 keys
 *            set it directly.
 *   params   Pointer to &g_presets[index] — the const table of all
 *            combat coefficients (speed_advance, atk_interval, …).
 *            Always re-set together with `index` via
 *            scene_set_strategy.
 *
 * Invariants
 *   0 ≤ index < N_STRATEGIES.
 *   params == &g_presets[index].
 */
typedef struct {
  int                   index;
  const StrategyParams *params;
} Strategy;

/*
 * FactionStats — per-faction tallies recomputed each tick.
 *
 * Intent
 *   Scene holds `faction[2]`, indexed by the GONDOR / MORDOR enum
 *   constants.  Replaces three parallel `int x[2]` arrays
 *   (n_alive[2], n_archers[2], kills[2]) with one struct array —
 *   `scene.faction[GONDOR].kills` reads like the domain language
 *   instead of `scene.kills[0]`.
 *
 * Members
 *   n_alive    living warriors of this faction this tick
 *              (excludes STATE_DEAD corpses)
 *   n_archers  living archers of this faction this tick
 *              (subset of n_alive; melee = n_alive − n_archers)
 *   kills     CUMULATIVE kills credited to this faction across the
 *              whole battle (NOT reset each tick).  Incremented in
 *              warrior_tick when a warrior of the OTHER faction dies.
 *
 * Invariants
 *   n_archers ≤ n_alive.  kills ≥ 0; monotonically increasing.
 *
 * References
 *   None directly — standard ABM "per-team aggregate" pattern.
 */
typedef struct {
  int n_alive;
  int n_archers;
  int kills;
} FactionStats;

/*
 * Scene — owns ALL simulation state for one battle.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── pool[POOL_MAX]   : Warrior[]    ← warrior pool (both factions)
 *       ├── n_total          : int          ← used slots (monotonic)
 *       ├── arrows[ARROW_…]  : Arrow[]      ← arrow pool (compacted/tick)
 *       ├── n_arrows         : int          ← active arrow count
 *       ├── faction[2]       : FactionStats ← per-side tallies + kills
 *       ├── winner           : int          ← −1 ongoing, else GONDOR/MORDOR
 *       ├── world            : World        ← pixel-space extent
 *       ├── strategy         : Strategy     ← index + active params pointer
 *       └── sim              : SimControls  ← paused
 *
 *   Every persistent simulation value is reachable from one Scene*.
 *   No file-scope globals carry simulation state; g_presets[] is a
 *   const lookup, not state.
 */
typedef struct {
  /* warrior pool — both factions interleaved by spawn order */
  Warrior pool[POOL_MAX];
  int n_total; /* used slots; never decreases */

  /* arrow pool — compacted each tick */
  Arrow arrows[ARROW_POOL_MAX];
  int n_arrows; /* active arrow count          */

  /* per-faction tallies — recomputed in scene_tick (kills is cumulative) */
  FactionStats faction[2];
  int          winner; /* -1 ongoing, 0 GONDOR, 1 MORDOR */

  /* pixel-space extent — refreshed each tick */
  World world;

  /* active steering preset — index + cached params pointer */
  Strategy strategy;

  /* user-facing controls */
  SimControls sim;
} Scene;

/*
 * scene_set_strategy — switch the active steering preset.  Keeps
 * Strategy.index and Strategy.params in lockstep so a key-press takes
 * effect on the very next tick.  Wraps modularly over N_STRATEGIES.
 */
static void scene_set_strategy(Scene *s, int new_index) {
  new_index = ((new_index % N_STRATEGIES) + N_STRATEGIES) % N_STRATEGIES;
  s->strategy.index  = new_index;
  s->strategy.params = &g_presets[new_index];
}

/*
 * spawn_warriors — append `count` warriors of one (faction, unit_type)
 * onto the pool, bumping n_total each time.
 *
 * The cap argument (POOL_MAX for init, or POOL_MAX min WARRIORS_MAX
 * for reinforcements) prevents pool overflow.  Used by scene_init's
 * four spawn waves AND by scene_add_warriors's two reinforcement
 * waves — six identical loop bodies collapsed into one helper.
 */
static void spawn_warriors(Scene *s, int faction, UnitType unit_type,
                            int count, int pool_cap) {
  for (int i = 0; i < count && s->n_total < pool_cap; i++) {
    warrior_spawn(&s->pool[s->n_total], s->n_total, faction, unit_type,
                  s->strategy.params, s->world.width, s->world.height);
    s->n_total++;
  }
}

/* count_alive_faction — how many warriors of `faction` are not corpses.
 * Used by scene_add_warriors to enforce the WARRIORS_MAX cap on
 * reinforcements (so a side can't grow unboundedly via repeated 'g'/'m'). */
static int count_alive_faction(const Scene *s, int faction) {
  int n = 0;
  for (int i = 0; i < s->n_total; i++)
    if (s->pool[i].faction == faction && s->pool[i].state != STATE_DEAD)
      n++;
  return n;
}

/*
 * scene_init — bring a Scene to a fresh start.
 *
 *   (1) zero everything, set the World extent from cols/rows
 *   (2) point Strategy at preset 0 (BALANCED) — index+params in lockstep.
 *       MUST come BEFORE warrior_spawn so it can read sp->atk_interval
 *       / shoot_interval for the initial atk_timer randomisation.
 *   (3) spawn the four starting waves in fixed order
 *       (Gondor melee → Gondor archers → Mordor melee → Mordor archers)
 */
static void scene_init(Scene *s, int cols, int rows) {
  /* (1) world extent */
  memset(s, 0, sizeof *s);
  s->world.width  = pw(cols);
  s->world.height = ph(rows);
  s->winner = -1;

  /* (2) default strategy (must precede spawning) */
  scene_set_strategy(s, 0);

  /* (3) four starting waves; spawn order matters only for pool index
   * layout, the simulation is symmetric in faction. */
  spawn_warriors(s, GONDOR, UNIT_MELEE,  MELEE_DEFAULT,  POOL_MAX);
  spawn_warriors(s, GONDOR, UNIT_ARCHER, ARCHER_DEFAULT, POOL_MAX);
  spawn_warriors(s, MORDOR, UNIT_MELEE,  MELEE_DEFAULT,  POOL_MAX);
  spawn_warriors(s, MORDOR, UNIT_ARCHER, ARCHER_DEFAULT, POOL_MAX);
}

/*
 * scene_add_warriors — append reinforcements to the pool ('g' / 'm' keys).
 *
 * Always appends at pool[n_total++]: no faction-offset arithmetic, no
 * aliasing bugs regardless of the order in which factions are reinforced.
 *
 * Bails early if:
 *   - battle already decided (winner ≥ 0), so reinforcements can't
 *     resurrect the loser
 *   - this faction already has WARRIORS_MAX living, so the side can't
 *     grow unboundedly via repeated key presses
 */
static void scene_add_warriors(Scene *s, int faction) {
  if (s->winner >= 0) return;
  if (count_alive_faction(s, faction) >= WARRIORS_MAX) return;

  spawn_warriors(s, faction, UNIT_MELEE,  REINFORCE_MELEE,  POOL_MAX);
  spawn_warriors(s, faction, UNIT_ARCHER, REINFORCE_ARCHER, POOL_MAX);
}

/* arrow_out_of_bounds — true if the arrow has flown off the world. */
static inline bool arrow_out_of_bounds(const Arrow *a, float ww, float wh) {
  return a->pos.x < 0.0f || a->pos.x >= ww
      || a->pos.y < 0.0f || a->pos.y >= wh;
}

/*
 * arrow_apply_hit — deal damage + flash on a successful proximity hit.
 *
 *   target_hp     -= ATK_DAMAGE         (may drop ≤ 0; resolved next
 *                                         warrior_tick)
 *   target_flash   = HIT_FLASH_TIME      (visual '*' overlay seconds)
 *   arrow.active   = false               (slot recycled by compact pass)
 */
static void arrow_apply_hit(Arrow *a, Warrior *target) {
  target->hp        -= ATK_DAMAGE;
  target->hit_timer  = HIT_FLASH_TIME;
  a->active          = false;
}

/* compact_arrows — Stable single-pass compaction.  Walks the arrows
 * pool, copying active entries down to fill any gaps left by
 * deactivated arrows.  *n_arrows is rewritten with the live count.
 *
 * Pattern: standard "two-finger" array compaction in O(N), used in
 * many ECS-style systems for sparse pools without a free-list. */
static void compact_arrows(Arrow *arrows, int *n_arrows) {
  int write_idx = 0;
  for (int read_idx = 0; read_idx < *n_arrows; read_idx++)
    if (arrows[read_idx].active)
      arrows[write_idx++] = arrows[read_idx];
  *n_arrows = write_idx;
}

/*
 * arrows_tick — move every active arrow, resolve hits, compact pool.
 *
 *   (1) for each active arrow:
 *         integrate position (pos += vel · dt)
 *         deactivate if off-screen / target dead / no-target
 *         if within ARROW_HIT_DIST of target → arrow_apply_hit
 *   (2) compact: shift active arrows to the front of the array
 */
static void arrows_tick(Arrow *arrows, int *n_arrows, Warrior *pool, float ww,
                        float wh, float dt) {
  for (int i = 0; i < *n_arrows; i++) {
    Arrow *a = &arrows[i];
    if (!a->active) continue;

    /* (1a) integrate */
    a->pos = v2add(a->pos, v2scale(a->vel, dt));

    /* (1b) early outs */
    if (arrow_out_of_bounds(a, ww, wh)) { a->active = false; continue; }
    if (a->target_idx < 0)              { a->active = false; continue; }

    Warrior *target = &pool[a->target_idx];
    if (target->state == STATE_DEAD)    { a->active = false; continue; }

    /* (1c) proximity hit test */
    float distance_to_target = v2len(v2sub(target->pos, a->pos));
    if (distance_to_target < ARROW_HIT_DIST)
      arrow_apply_hit(a, target);
  }

  /* (2) compact out inactive slots */
  compact_arrows(arrows, n_arrows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
  s->world.width = pw(cols);
  s->world.height = ph(rows);
  if (s->sim.paused || s->winner >= 0)
    return;

  /* warrior_tick increments a tiny `int[2]` kills array; copy current
   * cumulative totals in, let warrior_tick add to them, then write back. */
  int kills_per_faction[2] = { s->faction[GONDOR].kills, s->faction[MORDOR].kills };
  for (int i = 0; i < s->n_total; i++)
    warrior_tick(s->pool, s->n_total, i, s->strategy.params,
                 s->world.width, s->world.height, dt,
                 kills_per_faction, s->arrows, &s->n_arrows);
  s->faction[GONDOR].kills = kills_per_faction[GONDOR];
  s->faction[MORDOR].kills = kills_per_faction[MORDOR];

  arrows_tick(s->arrows, &s->n_arrows, s->pool, s->world.width, s->world.height, dt);

  /* Recount per-faction tallies from the pool (kills is cumulative, not reset). */
  s->faction[GONDOR].n_alive   = s->faction[MORDOR].n_alive   = 0;
  s->faction[GONDOR].n_archers = s->faction[MORDOR].n_archers = 0;
  for (int i = 0; i < s->n_total; i++) {
    if (s->pool[i].state == STATE_DEAD)
      continue;
    int f = s->pool[i].faction;
    s->faction[f].n_alive++;
    if (s->pool[i].unit_type == UNIT_ARCHER)
      s->faction[f].n_archers++;
  }

  if (s->faction[GONDOR].n_alive == 0 && s->faction[MORDOR].n_alive == 0)
    s->winner = MORDOR; /* mutual annihilation */
  else if (s->faction[GONDOR].n_alive == 0)
    s->winner = MORDOR;
  else if (s->faction[MORDOR].n_alive == 0)
    s->winner = GONDOR;
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
static void mark_cell(WINDOW *w, int cx, int cy, char ch, int pair, attr_t attr,
                      int cols, int rows) {
  if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
    return;
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
static attr_t warrior_attr(const Warrior *wr) {
  attr_t attr = A_NORMAL;
  if (wr->hp >= HP_MAX)
    attr |= A_BOLD;
  else if (wr->hp <= 1)
    attr |= A_DIM;
  if (wr->state == STATE_FLEE)
    attr |= A_BLINK;
  return attr;
}

/*
 * draw_arrows — pass 0: every active arrow as a coloured '-'
 * (green for Gondor, orange for Mordor).  No alpha lerp — arrows
 * are short-lived and move fast, so the 16 ms sub-tick smoothing
 * doesn't help.
 */
static void draw_arrows(const Scene *s, WINDOW *w, int cols, int rows) {
  for (int i = 0; i < s->n_arrows; i++) {
    const Arrow *a = &s->arrows[i];
    if (!a->active)
      continue;
    int cpair = (a->faction == GONDOR) ? 4 : 2;
    mark_cell(w, px_to_cell_x(a->pos.x), px_to_cell_y(a->pos.y), '-', cpair,
              A_BOLD, cols, rows);
  }
}

/*
 * draw_corpses — pass 1: dim '.' at every dead warrior whose corpse
 * timer has not yet expired.  Drawn underneath living warriors so a
 * fresh kill on the same cell doesn't show two glyphs.
 */
static void draw_corpses(const Scene *s, WINDOW *w, int cols, int rows) {
  for (int i = 0; i < s->n_total; i++) {
    const Warrior *wr = &s->pool[i];
    if (wr->state != STATE_DEAD || wr->dead_timer <= 0.0f)
      continue;
    mark_cell(w, px_to_cell_x(wr->pos.x), px_to_cell_y(wr->pos.y), '.', 3,
              A_DIM, cols, rows);
  }
}

/*
 * draw_living — pass 2: every living warrior with HP-driven attrs.
 * Arrow-strike flash ('*' in standout+bold) briefly overrides the
 * warrior's own glyph during HIT_FLASH_TIME after being hit.  Draws
 * at the alpha-interpolated position prev_pos → pos.
 */
static void draw_living(const Scene *s, WINDOW *w, int cols, int rows,
                        float alpha) {
  for (int i = 0; i < s->n_total; i++) {
    const Warrior *wr = &s->pool[i];
    if (wr->state == STATE_DEAD)
      continue;

    Vec2 dp = v2add(wr->prev_pos, v2scale(v2sub(wr->pos, wr->prev_pos), alpha));
    int cx = px_to_cell_x(dp.x);
    int cy = px_to_cell_y(dp.y);

    if (wr->hit_timer > 0.0f) {
      mark_cell(w, cx, cy, '*', wr->color_pair, A_STANDOUT | A_BOLD, cols,
                rows);
    } else {
      mark_cell(w, cx, cy, wr->glyph, wr->color_pair, warrior_attr(wr), cols,
                rows);
    }
  }
}

/*
 * draw_victory_banner — pass 3: blinking centred ribbon when one
 * faction has been wiped out.  Drawn last so it overlays everything.
 */
static void draw_victory_banner(const Scene *s, WINDOW *w, int cols, int rows) {
  if (s->winner < 0)
    return;
  static const char *win_msg[2] = {
      "  === GONDOR WINS — FOR FRODO ===  ",
      "  === MORDOR WINS — THE EYE SEES ALL ===  ",
  };
  const char *msg = win_msg[s->winner];
  int mx = (cols - (int)strlen(msg)) / 2;
  if (mx < 0)
    mx = 0;
  wattron(w, COLOR_PAIR(3) | A_BOLD | A_BLINK);
  mvwprintw(w, rows / 2, mx, "%s", msg);
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
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha) {
  draw_arrows(s, w, cols, rows);
  draw_corpses(s, w, cols, rows);
  draw_living(s, w, cols, rows, alpha);
  draw_victory_banner(s, w, cols, rows);
}

/* ===================================================================== */
/* §8  app — screen, input, main loop                                  */
/* ===================================================================== */

/*
 * Screen — terminal cell extent + ncurses lifecycle wrapper.
 *
 * Intent
 *   Owns the TERMINAL side of the world.  Where Scene.world tracks
 *   the pixel-space simulation box, this struct tracks the cell-space
 *   terminal grid that ncurses paints onto.  The two are linked but
 *   live in DIFFERENT spaces:
 *
 *      World.width  = Screen.cols × CELL_W      (CELL_W = 8 px)
 *      World.height = Screen.rows × CELL_H      (CELL_H = 16 px)
 *
 *   Keeping them in SEPARATE structs prevents the class of "drew in
 *   cell-space when I meant pixel-space" aspect-ratio bugs — physics
 *   never sees cells, the renderer never sees pixels except through
 *   the one px_to_cell_x/y bridge in §4.
 *
 * Why a tiny 2-field struct (not flat ints on App)
 *   • Lifecycle isolation: only screen_init / screen_resize /
 *     screen_free / screen_draw touch ncurses' initscr / endwin /
 *     mvprintw.  They all take `Screen *` to make this layer
 *     explicit at the type level.
 *   • Symmetry with World: simulation and rendering each get one
 *     struct named for the space they live in.
 *
 * Members
 *   cols   Terminal width in CELLS (from getmaxyx).
 *   rows   Terminal height in CELLS.  Bottom-row index is rows − 1.
 *
 * Invariants
 *   cols > 0, rows > 0.
 *   Scene.world.width  == cols × CELL_W,
 *   Scene.world.height == rows × CELL_H,
 *   maintained in lockstep by scene_init / scene_tick (refreshed
 *   each tick from the live Screen).
 *
 * References
 *   None directly — terminal extent is a rendering substrate concern.
 *   See CLAUDE.md §"Coordinate / Physics" for aspect-ratio compensation.
 */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
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
/* hud_paint_text — attron / mvprintw / attroff sandwich for one HUD
 * row.  Centralises the colour-pair setup that every HUD region uses. */
static void hud_paint_text(int row, int col, int pair, const char *text) {
  attron (COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_faction_roster_left — top-left: GONDOR roster (melee + archer
 * counts + cumulative kills).  Painted in PAIR_GONDOR (5). */
static void draw_faction_roster_left(const Scene *sc) {
  int melee_count   = sc->faction[GONDOR].n_alive - sc->faction[GONDOR].n_archers;
  int archer_count  = sc->faction[GONDOR].n_archers;
  char buf[56];
  snprintf(buf, sizeof buf, " GONDOR %dm %da  K:%d ",
           melee_count, archer_count, sc->faction[GONDOR].kills);
  hud_paint_text(0, 0, 5 /* PAIR_GONDOR */, buf);
}

/* draw_faction_roster_right — top-right: MORDOR roster, mirror of the
 * left panel.  Painted in PAIR_MORDOR (1). */
static void draw_faction_roster_right(const Screen *s, const Scene *sc) {
  int melee_count  = sc->faction[MORDOR].n_alive - sc->faction[MORDOR].n_archers;
  int archer_count = sc->faction[MORDOR].n_archers;
  char buf[56];
  snprintf(buf, sizeof buf, " K:%d  %dm %da MORDOR ",
           sc->faction[MORDOR].kills, melee_count, archer_count);
  int right_col = s->cols - (int)strlen(buf);
  if (right_col < 0) right_col = 0;
  hud_paint_text(0, right_col, 1 /* PAIR_MORDOR */, buf);
}

/* draw_hud_title — centred: active strategy name (or "PAUSED").
 * Bright yellow PAIR_HUD so it dominates any battle activity behind. */
static void draw_hud_title(const Screen *s, const Scene *sc) {
  char buf[48];
  snprintf(buf, sizeof buf, "[ WAR: %-12s ]",
           sc->sim.paused ? "PAUSED" : sc->strategy.params->name);
  int centre_col = (s->cols - (int)strlen(buf)) / 2;
  if (centre_col < 0) centre_col = 0;
  hud_paint_text(0, centre_col, PAIR_HUD, buf);
}

/* draw_hud_fps — row 1, right-aligned: fps + sim Hz indicator. */
static void draw_hud_fps(const Screen *s, double fps, int sim_fps) {
  enum { FPS_ROW = 1 };
  char buf[40];
  snprintf(buf, sizeof buf, " %.0f fps  sim:%d Hz ", fps, sim_fps);
  int right_col = s->cols - (int)strlen(buf);
  if (right_col < 0) right_col = 0;
  hud_paint_text(FPS_ROW, right_col, PAIR_HUD, buf);
}

/* draw_hud_hint — bottom-row key bindings strip (PAIR_HINT cyan). */
static void draw_hud_hint(const Screen *s) {
  static const char *KEY_HINT =
      " q:quit  spc:pause  r:reset  g:+gondor  m:+mordor  1-6:strategy ";
  hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha) {
  /* (1) clear offscreen buffer */
  erase();
  /* (2) paint the battle (warriors + arrows + corpses) */
  scene_draw(sc, stdscr, s->cols, s->rows, alpha);
  /* (3) HUD on top: 3-segment top row + fps indicator + bottom keys */
  draw_faction_roster_left (sc);
  draw_faction_roster_right(s, sc);
  draw_hud_title           (s, sc);
  draw_hud_fps             (s, fps, sim_fps);
  draw_hud_hint            (s);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── App ── */

/*
 * FpsCounter — rolling-window frame-rate estimator.
 *
 * Per-frame fps would jitter; accumulate frame_count + elapsed
 * nanoseconds over FPS_WINDOW_NS (500 ms) and emit a smoothed
 * `display` value each time the window fills.  Same shape as the
 * FpsCounter on every other file in this project.
 */
typedef struct {
  int     frame_count;
  int64_t window_ns;
  double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns   = 0;
  f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt) {
  const int64_t FPS_WINDOW_NS = (int64_t)NS_PER_SEC / 2;     /* 500 ms */
  f->frame_count++;
  f->window_ns += dt;
  if (f->window_ns < FPS_WINDOW_NS) return;
  f->display     = (double)f->frame_count
                 * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns   = 0;
}

/*
 * App — top-level container for every persistent value.
 *
 *   scene        simulation state (sub-structs reachable from here)
 *   screen       terminal cell extent + ncurses lifecycle
 *   fps          rolling-window fps estimator for the HUD
 *   sim_fps      target physics tick rate (config; not user-tweakable)
 *   running      sig_atomic_t flag cleared by SIGINT/TERM + 'q' key
 *   need_resize  sig_atomic_t flag set by SIGWINCH; main reacts on
 *                the next iteration
 */
typedef struct {
  Scene                 scene;
  Screen                screen;
  FpsCounter            fps;
  int                   sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  app->need_resize = 0;
}

/*
 * app_handle_key — process one keystroke.
 *
 * Keys 1–6 invoke scene_set_strategy with the corresponding index.
 * The change is live: every combat helper takes `const StrategyParams
 * *sp` from `scene.strategy.params` on the next tick — no reset
 * required.  In-flight arrows continue unaffected (ARROW_TRAVEL_SPD
 * is constant).
 */
static bool app_handle_key(App *app, int ch) {
  Scene *sc = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    sc->sim.paused = !sc->sim.paused;
    break;
  case 'r':
  case 'R':
    scene_init(sc, app->screen.cols, app->screen.rows);
    break;
  case 'g':
  case 'G':
    scene_add_warriors(sc, GONDOR);
    break;
  case 'm':
  case 'M':
    scene_add_warriors(sc, MORDOR);
    break;
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
    scene_set_strategy(sc, ch - '1');
    break;
  default:
    break;
  }
  return true;
}

/*
 * main — game loop (fixed-step accumulator; Fiedler "Fix Your Timestep!").
 *
 *   (1) handle pending SIGWINCH + reset timers
 *   (2) measure dt since last frame, capped at DT_CAP_NS
 *   (3) drain sim_accum: scene_tick at fixed dt_sec until caught up
 *   (4) sub-tick alpha for the renderer (= leftover ns / TICK_LEN_NS)
 *   (5) rolling-window fps via fps_counter_tick
 *   (6) sleep BEFORE render so terminal I/O stays inside FRAME_BUDGET_NS
 *   (7) draw + present
 *   (8) drain non-blocking input via app_handle_key
 */
int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT,   on_exit_signal);
  signal(SIGTERM,  on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app    = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;
  fps_counter_init(&app->fps);

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  const int64_t DT_CAP_NS       = 100 * NS_PER_MS;            /* avalanche guard */
  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;    /* render cadence  */
  const int64_t TICK_LEN_NS     = TICK_NS(app->sim_fps);
  const float   TICK_LEN_SEC    = (float)TICK_LEN_NS / (float)NS_PER_SEC;

  int64_t frame_time = clock_ns();
  int64_t sim_accum  = 0;

  while (app->running) {
    int64_t frame_start = clock_ns();

    /* (1) handle SIGWINCH */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum  = 0;
    }

    /* (2) measure dt, cap to avoid avalanche */
    int64_t now = clock_ns();
    int64_t dt  = now - frame_time;
    frame_time  = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;

    /* (3) drain accumulator: fixed-step physics until caught up */
    sim_accum += dt;
    while (sim_accum >= TICK_LEN_NS) {
      scene_tick(&app->scene, TICK_LEN_SEC,
                 app->screen.cols, app->screen.rows);
      sim_accum -= TICK_LEN_NS;
    }

    /* (4) sub-tick alpha for renderer */
    float alpha = (float)sim_accum / (float)TICK_LEN_NS;

    /* (5) rolling-window fps counter */
    fps_counter_tick(&app->fps, dt);

    /* (6) sleep BEFORE render so I/O stays inside the budget */
    int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
    clock_sleep_ns(budget_left);

    /* (7) draw + present */
    screen_draw(&app->screen, &app->scene,
                app->fps.display, app->sim_fps, alpha);
    screen_present();

    /* (8) drain input */
    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
