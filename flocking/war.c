/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * war.c — two armies of steering "warriors" clash in the terminal.
 *
 * Each warrior is a moving dot with a tiny brain that picks one of four
 * actions: advance, fight, flee, or lie dead.  Archers shoot real '-'
 * arrows that fly across the screen and damage on contact.  Press 1-6 to
 * swap battle strategies live.  Steering forces follow Reynolds 1999
 * (red3d.com/cwr/steer); the four-state brain follows Buckland's
 * "Programming Game AI by Example".  Gondor/Mordor names are Tolkien
 * flavour, not lore.
 */

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

/* ── §1 config — strategy presets + fixed combat constants ── */

/*
 * StrategyParams — one full set of combat tuning knobs.  Each of the six
 * strategies is one row of g_presets[] below, and the active row drives
 * every warrior.  Combat code reads these through a `const StrategyParams
 * *sp` pointer, so swapping the active row takes effect on the next tick.
 *
 * Melee knobs:
 *   engage_range   how close an enemy must be before a melee fighter
 *                  commits to fighting it (pixels)
 *   flee_hp        run away once health drops to this; 0 = never run
 *   atk_interval   seconds between melee hits
 *   speed_advance  marching speed toward the enemy (px/s)
 *   speed_flee     running-away speed; should beat speed_advance
 *   sep_radius     personal-space bubble between allies (px); smaller
 *                  packs them tighter
 *   safe_range     how far a fleer must get before it can regroup (px)
 *   rally_time     seconds spent safe before rejoining the advance
 *   melee_speed    slow shuffle while brawling, to stay on the target
 *
 * Archer knobs:
 *   archer_flee_hp    health at which archers panic; 0 = never from HP
 *   arrow_range       firing range; archers close in until within it (px)
 *   archer_flee_range run if any enemy gets this close (px)
 *   stand_off_dist    preferred gap to keep from the enemy while firing
 *   shoot_interval    seconds between shots
 *   archer_speed      archer move speed (slower than melee)
 *
 * Steering weights (bigger = stronger pull):
 *   w_seek  toward the chosen target
 *   w_sep   away from crowding allies (low = dense, high = spread out)
 *   w_flee  urgency of the run-away
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
 * The six presets, each with its own feel:
 *   STANDARD      balanced baseline; cautious advance, archers mid-range
 *   BERSERKER     wide engage, fast hits, nobody flees, packs tight
 *   SHIELD WALL   slow orderly ranks, very long-range archers, hard to break
 *   GUERRILLA     skirmishers: hit, run, regroup, re-engage; jumpy archers
 *   ARCHER FOCUS  ranged dominance; archers hang far back in their half
 *   CHAOS         everyone sprints, no personal space, no fleeing — one scrum
 */
#define N_STRATEGIES 6
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

/* Army sizes and colour-pair IDs — the same no matter which strategy is on. */
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

  /* Colour-pair IDs.  Pairs 1-7 paint the battle (faction units + shared
   * corpse/banner yellow).  PAIR_HUD/PAIR_HINT stay bright so the overlay
   * reads against any battle behind it. */
  N_COLORS = 7,
  PAIR_HUD = 8,  /* bright yellow — top-centre status */
  PAIR_HINT = 9, /* bright cyan   — bottom key hint   */
};

/* Physics runs in pixels; the screen is cells.  One cell is this many
 * pixels wide/tall — terminal cells are taller than wide, so x and y differ. */
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

/* ── §2 clock — monotonic timer + sleep ── */

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

/* ── §3 color — faction palette + HUD pairs ── */

/*
 * Each colour pair stands for one game role, not a theme tier:
 *   1 Mordor melee (red)   2 Mordor archers/arrows (orange)
 *   3 corpses + banner (yellow)   4 Gondor archers/arrows (green)
 *   5 Gondor melee (cyan)   6,7 spare
 *   8 PAIR_HUD   9 PAIR_HINT
 * Background is -1 (the terminal default) so we sit on the user's theme.
 * Every colour is in the bright half of the palette so even dim (last-HP)
 * and blinking (fleeing) warriors stay visible.
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

/* ── §4 coords & vec2 — pixel↔cell bridge + 2-D vector helpers ── */

static inline float pw(int cols) { return (float)(cols * CELL_W); }
static inline float ph(int rows) { return (float)(rows * CELL_H); }

/* Rounding half-up here keeps a dot from flickering between two cells when
 * it sits exactly on the boundary. */
static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/*
 * Vec2 — a 2-D point or direction.  Used for everything spatial: position,
 * velocity, force, offsets.  The helpers below return new Vec2s (no output
 * pointers) so steering math reads like the formula it implements.
 *
 *   x, y   the two components, always in PIXEL space.  Only scene_draw
 *          converts to screen cells; the physics never sees cells.
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

/* Keep a warrior inside the world: on hitting an edge, pin it and flip the
 * velocity so it bounces back in. */
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

/* ── §5 entity — Warrior + Arrow data and movement ── */

/*
 * UnitType — which kind of fighter, which decides its behaviour:
 *   UNIT_MELEE   charges in and brawls; no ranged attack
 *   UNIT_ARCHER  hangs back and shoots arrows; runs if enemies close in
 * warrior_tick branches to melee_logic or archer_logic on this.
 */
typedef enum { UNIT_MELEE = 0, UNIT_ARCHER } UnitType;

/*
 * WarriorState — the one thing a warrior is doing right now.  The four
 * states form a little brain: a warrior advances, picks a fight, may break
 * and flee, and eventually dies.  An enemy in engage_range moves ADVANCE to
 * COMBAT; low health moves COMBAT to FLEE; reaching safety long enough
 * rallies FLEE back to ADVANCE; zero health goes to DEAD.
 *
 *   STATE_ADVANCE  marching toward the enemy mass
 *   STATE_COMBAT   fighting: brawling (melee) or shooting (archer)
 *   STATE_FLEE     running away; the renderer makes it blink
 *   STATE_DEAD     a corpse, shown briefly then ignored.  The slot is
 *                  never reused — simpler than tracking free slots, and
 *                  the pool is sized for the worst case.
 *
 * ADVANCE is 0 on purpose: a freshly zeroed warrior starts out advancing.
 */
typedef enum {
  STATE_ADVANCE = 0,
  STATE_COMBAT,
  STATE_FLEE,
  STATE_DEAD,
} WarriorState;

/*
 * Warrior — one soldier.  All warriors live in one shared pool[] array.
 * Fields are grouped by who writes them: identity is set once at spawn;
 * the position/velocity block is rewritten every tick; the health/state
 * block is driven by combat code; the timers count down.
 *
 * prev_pos is last tick's position.  Physics steps at a fixed rate but the
 * screen may redraw faster, so the renderer blends prev_pos toward pos to
 * keep motion smooth instead of jumpy (Fiedler, "Fix Your Timestep!").
 *
 * Members:
 *   pos         current position, pixels; bounced off world edges
 *   prev_pos    position at the start of this tick, for smooth drawing
 *   vel         velocity, pixels per second; capped per state
 *   faction     GONDOR (0) or MORDOR (1); never changes
 *   unit_type   melee or archer; picks which brain runs
 *   glyph       the ASCII character drawn for this warrior
 *   color_pair  ncurses colour for this warrior's side
 *   hp          health, 0..HP_MAX; reaching 0 means dead
 *   state       what it's doing now (see WarriorState)
 *   target_idx  pool index of the enemy it's locked onto, or -1 for none
 *   atk_timer   seconds until the next hit or shot
 *   rally_timer seconds spent safe; crossing rally_time ends a flee
 *   dead_timer  seconds the corpse stays on screen
 *   hit_timer   seconds left of the '*' flash after being struck
 *
 * Always true: 0 <= hp <= HP_MAX; state is DEAD exactly when hp is 0;
 * target_idx is a valid pool index or -1.
 */
typedef struct {
  /* rewritten every tick */
  Vec2 pos;
  Vec2 prev_pos;
  Vec2 vel;

  /* set once at spawn */
  int faction;
  UnitType unit_type;
  char glyph;
  int color_pair;

  /* health + behaviour */
  int hp;
  WarriorState state;
  int target_idx;     /* -1 when locked onto no one */

  /* countdown clocks, in seconds */
  float atk_timer;
  float rally_timer;
  float dead_timer;
  float hit_timer;
} Warrior;

/*
 * Arrow — one '-' projectile in flight.  An archer fires it once; it then
 * flies in a straight line at a fixed speed and damages its target if it
 * gets close enough.  Arrows live in their own pool, separate from
 * warriors, because an arrow can outlive the archer who shot it, and most
 * warriors have no arrow in flight at any moment.
 *
 * The target is fixed at fire time and never re-aimed: if the target moves
 * or dies, the arrow keeps going to where it was headed and misses.  No
 * homing.
 *
 * Members:
 *   pos         current position, pixels
 *   vel         velocity, pixels per second; speed and direction set at
 *               firing and constant after
 *   target_idx  pool index of the warrior aimed at
 *   faction     the firing archer's side (so it won't hit its own team)
 *   active      true while flying; cleared on a hit or when off-screen,
 *               then the slot is swept away by arrows_tick
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
 * warrior_spawn — fill in one fresh warrior.  Each side starts on its own
 * half: melee toward the front, archers further back.  The first attack
 * timer is randomised so they don't all swing on the same frame.
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
 * warrior_step — move one warrior for this tick.  Apply the steering force
 * to its velocity (capped at max_speed), remember where it was, then slide
 * it to the new position.  Remembering prev_pos lets the renderer draw
 * smooth motion between ticks.
 */
static void warrior_step(Warrior *w, Vec2 accel, float max_speed, float dt) {
  w->vel = v2clamp_len(v2add(w->vel, v2scale(accel, dt)), max_speed);
  w->prev_pos = w->pos;
  w->pos = v2add(w->pos, v2scale(w->vel, dt));
}

/* ── §6 combat — steering forces, melee logic, archer logic ── */

/*
 * steer_seek — a steering force that pulls a warrior toward a point at a
 * given speed.  It aims for the velocity it *wants* and returns the
 * difference from its current velocity, so the harder it is already moving
 * the right way, the gentler the nudge.  Reynolds 1999.
 */
static Vec2 steer_seek(Vec2 pos, Vec2 vel, Vec2 target, float speed) {
  Vec2 desired = v2scale(v2norm(v2sub(target, pos)), speed);
  return v2sub(desired, vel);
}

/* steer_flee — the opposite of seek: a force that pushes away from a threat. */
static Vec2 steer_flee(Vec2 pos, Vec2 vel, Vec2 threat, float speed) {
  return v2scale(steer_seek(pos, vel, threat, speed), -1.0f);
}

/*
 * steer_separate — a push away from nearby allies so they don't pile onto
 * the same cell.  The closer an ally is (within sep_radius), the stronger
 * the push.  Only same-side warriors repel each other; enemies don't, so
 * fighters can press into the enemy line.
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
 * enemy_centroid — the average position of all living enemies, i.e. the
 * "centre of mass" of the other army.  Warriors march toward this rather
 * than one foe, which makes them form a battle line.  No enemies left
 * means the world centre.
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

/* nearest_enemy_idx — find the closest living enemy; returns its pool index,
 * or -1 if none are left. */
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
 * Steer — what one behaviour decides this tick: a force to apply and a top
 * speed.  Each per-state helper returns one of these, and the dispatcher
 * feeds it to warrior_step.  The speed cap travels with the force because
 * it depends on the state — a fleeing warrior sprints, a brawling one
 * barely moves.
 *
 *   force    the steering force to apply this tick
 *   max_spd  the hard ceiling on speed afterward (px/s)
 */
typedef struct {
  Vec2 force;
  float max_spd;
} Steer;

/*
 * The melee state machine.  Each helper below handles one state: it reads
 * the warrior and the pool, may change the warrior's state/target/timers,
 * and returns the Steer for this tick.  melee_logic picks the right one.
 */

/*
 * melee_advance — march toward the enemy army.  The moment any enemy comes
 * within engage_range, lock onto it and switch to fighting.
 */
static Steer melee_advance(Warrior *w, const Warrior *pool, int n_total,
                           int self, const StrategyParams *sp,
                           float ww, float wh) {
  Vec2 centroid = enemy_centroid(pool, n_total, w->faction, ww, wh);
  int ne = nearest_enemy_idx(pool, n_total, self);

  if (ne >= 0 && v2len(v2sub(pool[ne].pos, w->pos)) < sp->engage_range) {
    /* Lock on and stand still this tick; resetting atk_timer stops a free
     * hit the instant combat begins. */
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
 * melee_combat — brawl: shuffle slowly toward the locked target and land a
 * hit every atk_interval.  If the target dies, go back to advancing; if our
 * own health drops too low, break and flee.
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

  /* Count down to the next swing; when it fires, deal damage and reset. */
  w->atk_timer -= dt;
  if (w->atk_timer <= 0.0f) {
    w->atk_timer = sp->atk_interval;
    pool[w->target_idx].hp -= ATK_DAMAGE;
  }

  Vec2 tgt = pool[w->target_idx].pos;
  Vec2 force = v2add(
      v2scale(steer_seek(w->pos, w->vel, tgt, sp->melee_speed), sp->w_seek),
      v2scale(steer_separate(pool, n_total, self, sp),
              sp->w_sep * 0.4f) /* weaker push so it stays on the target */
  );
  return (Steer){force, sp->melee_speed * 1.5f};
}

/*
 * melee_flee — sprint away from the nearest enemy.  Once far enough away
 * (safe_range), start a countdown; survive that countdown and the warrior
 * regains its nerve and advances again.
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
 * melee_logic — run one melee warrior for a tick: pick the helper for its
 * current state, then apply the resulting move and bounce off the edges.
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

/*
 * The archer state machine.  Archers don't lock a single target; each tick
 * they react to how far the nearest enemy is plus a panic check on low
 * health, and pick one of: flee, shoot, or close the distance.
 */

/*
 * archer_shoot — fire one arrow at enemy pool[ne].  The arrow starts at the
 * archer and heads straight for where the target is right now, at a fixed
 * speed; it won't follow the target afterward.  Dropped silently if the
 * arrow pool is full.
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
 * archer_flee_force — the run-away force: push from the nearest enemy plus
 * the usual spacing from allies.  Shared by the two reasons an archer flees
 * (an enemy got too close, or it's already fleeing).
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
 * archer_combat — in firing range: keep a comfortable gap from the enemy
 * (stand_off_dist) and loose an arrow every shoot_interval.
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
 * archer_advance — too far to shoot: move closer, but aim for a spot on the
 * archer's own side of the enemy rather than into them, so archers hang
 * back in their half instead of charging.
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

#define ARCHER_IDLE_DAMPING  0.92f   /* how fast it coasts to a stop, per tick */

/*
 * archer_handle_no_enemies — nothing left to shoot at, so coast to a halt
 * and stand down.
 */
static void archer_handle_no_enemies(Warrior *w, const StrategyParams *sp,
                                      float ww, float wh, float dt) {
  w->state = STATE_ADVANCE;
  w->vel   = v2scale(w->vel, ARCHER_IDLE_DAMPING);
  warrior_step(w, v2(0, 0), sp->archer_speed, dt);
  bounce_pos(&w->pos, &w->vel, ww, wh);
}

/*
 * archer_maybe_panic_on_low_hp — if health is low, switch into fleeing.
 * It only flips the state (the caller chooses the actual force) and resets
 * the rally clock, so the archer has to spend the full safe time before it
 * can advance again.
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
 * archer_tick_rally — while fleeing, count how long it's been safely away
 * from the threat (at least safe_range).  Stay safe long enough and it
 * advances again.  Dipping back into danger restarts the count, so a
 * half-finished rally is wasted.
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
 * archer_pick_force — decide what an archer does this tick from how close
 * the nearest enemy is and whether it's already fleeing:
 *   enemy too close      -> flee (and switch into the flee state)
 *   already fleeing      -> keep fleeing
 *   within firing range  -> hold position and shoot
 *   otherwise            -> move closer
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
 * archer_logic — run one archer for a tick.  Order matters: stand down if
 * no enemies remain; panic-flee if hurt; pick and apply a move; bounce off
 * walls; then advance the rally clock if it's still fleeing.
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

/*
 * warrior_tick — one full update for one warrior, before handing off to the
 * melee or archer brain.  First it handles the shared bookkeeping: tick down
 * the hit flash, age corpses, turn the warrior into a corpse (and credit the
 * kill to the other side) if its health is gone, and forget a target that
 * just died.
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
    kills_per_faction[1 - w->faction]++;  /* the other side scores the kill */
    return;
  }

  if (w->target_idx >= 0 && pool[w->target_idx].state == STATE_DEAD)
    w->target_idx = -1;

  if (w->unit_type == UNIT_ARCHER)
    archer_logic(pool, n_total, self, sp, ww, wh, dt, arrows, n_arrows);
  else
    melee_logic(pool, n_total, self, sp, ww, wh, dt);
}

/* ── §7 scene — world state, spawning, ticking, drawing ── */

/*
 * World — how big the battlefield is, in pixels (width = cols x CELL_W,
 * height = rows x CELL_H).  Bundled so combat helpers can take one value
 * instead of two.  Refreshed each tick from the terminal size.
 */
typedef struct {
  float width;
  float height;
} World;

/*
 * SimControls — the playback knobs the user can flip.
 *   paused   when true the battle freezes and the HUD shows "PAUSED"
 *            (toggled with SPACE)
 */
typedef struct {
  bool paused;
} SimControls;

/*
 * Strategy — which preset is active.  Held as two things kept in lockstep:
 * an index (for the 1-6 keys and the HUD label) and a pointer straight to
 * that preset's knobs (what the combat code actually reads).  Keeping the
 * pointer means combat helpers skip the array lookup every tick.  Always
 * change both together via scene_set_strategy.
 *
 *   index    which preset, 0..N_STRATEGIES-1
 *   params   pointer to &g_presets[index]
 */
typedef struct {
  int                   index;
  const StrategyParams *params;
} Strategy;

/*
 * FactionStats — a side's running totals.  Scene keeps one per faction.
 *   n_alive    living warriors right now (corpses excluded)
 *   n_archers  how many of those are archers (so melee = n_alive - n_archers)
 *   kills      total enemies this side has killed, across the whole battle;
 *              only ever grows
 * n_alive and n_archers are recounted every tick; kills is not reset.
 */
typedef struct {
  int n_alive;
  int n_archers;
  int kills;
} FactionStats;

/*
 * Scene — all the state for one battle, reachable from a single Scene*.
 * Both armies share one warrior pool and there is one arrow pool; the rest
 * is bookkeeping (tallies, who won, world size, active strategy, controls).
 *
 *   pool/n_total    every warrior; new ones append, dead ones keep their
 *                   slot, so indices stay stable and n_total only grows
 *   arrows/n_arrows arrows in flight; swept clean each tick
 *   faction[2]      per-side totals, indexed by GONDOR/MORDOR
 *   winner          -1 while ongoing, else the winning faction
 *   world           battlefield size in pixels
 *   strategy        the active preset
 *   sim             user controls (paused)
 */
typedef struct {
  Warrior pool[POOL_MAX];
  int n_total;

  Arrow arrows[ARROW_POOL_MAX];
  int n_arrows;

  FactionStats faction[2];
  int          winner;

  World world;
  Strategy strategy;
  SimControls sim;
} Scene;

/*
 * scene_set_strategy — switch to another preset, keeping the index and the
 * params pointer in sync.  The math wraps the index around so out-of-range
 * values land back in 0..N_STRATEGIES-1.
 */
static void scene_set_strategy(Scene *s, int new_index) {
  new_index = ((new_index % N_STRATEGIES) + N_STRATEGIES) % N_STRATEGIES;
  s->strategy.index  = new_index;
  s->strategy.params = &g_presets[new_index];
}

/*
 * spawn_warriors — add `count` new warriors of one side and type to the
 * pool, stopping early if the pool would pass pool_cap.  Used both for the
 * opening armies and for reinforcements.
 */
static void spawn_warriors(Scene *s, int faction, UnitType unit_type,
                            int count, int pool_cap) {
  for (int i = 0; i < count && s->n_total < pool_cap; i++) {
    warrior_spawn(&s->pool[s->n_total], s->n_total, faction, unit_type,
                  s->strategy.params, s->world.width, s->world.height);
    s->n_total++;
  }
}

/* count_alive_faction — how many of a side's warriors are still alive.
 * Used to cap reinforcements so a side can't grow forever by mashing g/m. */
static int count_alive_faction(const Scene *s, int faction) {
  int n = 0;
  for (int i = 0; i < s->n_total; i++)
    if (s->pool[i].faction == faction && s->pool[i].state != STATE_DEAD)
      n++;
  return n;
}

/*
 * scene_init — start a fresh battle: clear everything, size the world, pick
 * the first strategy, then spawn both armies.  The strategy must be set
 * before spawning because warrior_spawn reads its attack timing.
 */
static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->world.width  = pw(cols);
  s->world.height = ph(rows);
  s->winner = -1;

  scene_set_strategy(s, 0);

  spawn_warriors(s, GONDOR, UNIT_MELEE,  MELEE_DEFAULT,  POOL_MAX);
  spawn_warriors(s, GONDOR, UNIT_ARCHER, ARCHER_DEFAULT, POOL_MAX);
  spawn_warriors(s, MORDOR, UNIT_MELEE,  MELEE_DEFAULT,  POOL_MAX);
  spawn_warriors(s, MORDOR, UNIT_ARCHER, ARCHER_DEFAULT, POOL_MAX);
}

/*
 * scene_add_warriors — drop in reinforcements for one side (the g/m keys).
 * Does nothing if the battle is already won (no reviving the loser) or if
 * the side is already at its living-warrior cap.
 */
static void scene_add_warriors(Scene *s, int faction) {
  if (s->winner >= 0) return;
  if (count_alive_faction(s, faction) >= WARRIORS_MAX) return;

  spawn_warriors(s, faction, UNIT_MELEE,  REINFORCE_MELEE,  POOL_MAX);
  spawn_warriors(s, faction, UNIT_ARCHER, REINFORCE_ARCHER, POOL_MAX);
}

/* arrow_out_of_bounds — true once the arrow has flown off the battlefield. */
static inline bool arrow_out_of_bounds(const Arrow *a, float ww, float wh) {
  return a->pos.x < 0.0f || a->pos.x >= ww
      || a->pos.y < 0.0f || a->pos.y >= wh;
}

/*
 * arrow_apply_hit — an arrow landed: damage the target, start its hit
 * flash, and retire the arrow.  The target may drop to 0 health; that's
 * turned into a death on its next tick.
 */
static void arrow_apply_hit(Arrow *a, Warrior *target) {
  target->hp        -= ATK_DAMAGE;
  target->hit_timer  = HIT_FLASH_TIME;
  a->active          = false;
}

/* compact_arrows — squeeze the dead arrows out of the pool by copying the
 * live ones down to fill the gaps, in one pass, and update the count. */
static void compact_arrows(Arrow *arrows, int *n_arrows) {
  int write_idx = 0;
  for (int read_idx = 0; read_idx < *n_arrows; read_idx++)
    if (arrows[read_idx].active)
      arrows[write_idx++] = arrows[read_idx];
  *n_arrows = write_idx;
}

/*
 * arrows_tick — advance every arrow one tick: move it, drop it if it flew
 * off-screen or its target is gone, and damage the target if it's close
 * enough.  Then sweep the dead arrows out of the pool.
 */
static void arrows_tick(Arrow *arrows, int *n_arrows, Warrior *pool, float ww,
                        float wh, float dt) {
  for (int i = 0; i < *n_arrows; i++) {
    Arrow *a = &arrows[i];
    if (!a->active) continue;

    a->pos = v2add(a->pos, v2scale(a->vel, dt));

    /* gone for good: off the field, no target, or target already dead */
    if (arrow_out_of_bounds(a, ww, wh)) { a->active = false; continue; }
    if (a->target_idx < 0)              { a->active = false; continue; }

    Warrior *target = &pool[a->target_idx];
    if (target->state == STATE_DEAD)    { a->active = false; continue; }

    /* close enough to count as a strike? */
    float distance_to_target = v2len(v2sub(target->pos, a->pos));
    if (distance_to_target < ARROW_HIT_DIST)
      arrow_apply_hit(a, target);
  }

  compact_arrows(arrows, n_arrows);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
  s->world.width = pw(cols);
  s->world.height = ph(rows);
  if (s->sim.paused || s->winner >= 0)
    return;

  /* warrior_tick bumps a plain two-element kill count; seed it with the
   * running totals, let the loop add to it, then store it back. */
  int kills_per_faction[2] = { s->faction[GONDOR].kills, s->faction[MORDOR].kills };
  for (int i = 0; i < s->n_total; i++)
    warrior_tick(s->pool, s->n_total, i, s->strategy.params,
                 s->world.width, s->world.height, dt,
                 kills_per_faction, s->arrows, &s->n_arrows);
  s->faction[GONDOR].kills = kills_per_faction[GONDOR];
  s->faction[MORDOR].kills = kills_per_faction[MORDOR];

  arrows_tick(s->arrows, &s->n_arrows, s->pool, s->world.width, s->world.height, dt);

  /* Recount how many of each side are alive (kills stays, it's a running total). */
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
 * mark_cell — draw one character at a terminal cell, skipping it if it
 * falls off screen.  The double cast on ch keeps ncurses from mangling
 * characters above 127 (it would otherwise sign-extend them).
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
 * warrior_attr — how a living warrior should look: bold at full health,
 * faint on its last hit, and blinking while it's fleeing.
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
 * draw_arrows — draw each arrow in flight as a coloured '-'.  No smoothing:
 * arrows are quick and short-lived, so it wouldn't help.
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
 * draw_corpses — draw a faint '.' for each recent corpse.  Drawn before the
 * living so a warrior standing on a corpse hides it.
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
 * draw_living — draw every living warrior at its smoothed position.  A
 * warrior that was just hit flashes as a '*' for a moment instead of its
 * normal glyph.
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
 * draw_victory_banner — once a side has won, blink a centred banner over
 * everything else.
 */
static void draw_victory_banner(const Scene *s, WINDOW *w, int cols, int rows) {
  if (s->winner < 0)
    return;
  static const char *win_msg[2] = {
      "  === GONDOR WINS - FOR FRODO ===  ",
      "  === MORDOR WINS - THE EYE SEES ALL ===  ",
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
 * scene_draw — paint one frame, back to front: arrows, then corpses, then
 * living warriors, then the victory banner on top.
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha) {
  draw_arrows(s, w, cols, rows);
  draw_corpses(s, w, cols, rows);
  draw_living(s, w, cols, rows, alpha);
  draw_victory_banner(s, w, cols, rows);
}

/* ── §8 app — screen, input, main loop ── */

/*
 * Screen — the terminal's size, in cells (cols x rows), plus the ncurses
 * setup/teardown.  This is the cell-space counterpart to Scene.world's
 * pixel space; keeping them apart keeps physics out of cells and the
 * renderer out of pixels except at the one conversion point.
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

/* hud_paint_text — write one bold coloured string at a spot on screen.
 * Wraps the attribute on/off so every HUD piece doesn't repeat it. */
static void hud_paint_text(int row, int col, int pair, const char *text) {
  attron (COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw_faction_roster_left — top-left: Gondor's melee/archer counts and
 * total kills, in Gondor's colour. */
static void draw_faction_roster_left(const Scene *sc) {
  int melee_count   = sc->faction[GONDOR].n_alive - sc->faction[GONDOR].n_archers;
  int archer_count  = sc->faction[GONDOR].n_archers;
  char buf[56];
  snprintf(buf, sizeof buf, " GONDOR %dm %da  K:%d ",
           melee_count, archer_count, sc->faction[GONDOR].kills);
  hud_paint_text(0, 0, 5 /* PAIR_GONDOR */, buf);
}

/* draw_faction_roster_right — top-right: the same for Mordor, mirrored. */
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

/* draw_hud_title — centred: the active strategy's name, or "PAUSED". */
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
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha);
  /* HUD goes on top of the battle */
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
 * FpsCounter — a smoothed frame-rate readout.  Measuring fps frame by frame
 * jumps around, so it counts frames over a half-second window and reports
 * the average.
 *   frame_count  frames seen so far this window
 *   window_ns    time elapsed this window, in nanoseconds
 *   display      the last smoothed fps value, shown in the HUD
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
 * App — everything that lives for the whole program.
 *   scene        the battle state
 *   screen       terminal size + ncurses
 *   fps          the HUD's frame-rate readout
 *   sim_fps      physics ticks per second (fixed)
 *   running      cleared to stop the loop; set by 'q' and by exit signals
 *   need_resize  set by a terminal-resize signal; handled next loop
 * The two flags are sig_atomic_t because signal handlers write them.
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
 * app_handle_key — act on one keypress; returns false only for quit.
 * Keys 1-6 switch strategy live, taking effect on the next tick with no
 * reset; arrows already flying are unaffected.
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
 * main — the game loop.  Physics runs at a fixed rate while the screen
 * draws as often as it can: each pass measures real time elapsed, steps the
 * simulation in fixed chunks until it's caught up, then draws (blending
 * between steps for smooth motion) and reads input.  Pattern from Fiedler,
 * "Fix Your Timestep!".
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

    /* react to a terminal resize and restart the timing */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum  = 0;
    }

    /* time since last frame; capped so a long stall can't trigger a flood
     * of catch-up steps */
    int64_t now = clock_ns();
    int64_t dt  = now - frame_time;
    frame_time  = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;

    /* run as many fixed physics steps as the elapsed time bought us */
    sim_accum += dt;
    while (sim_accum >= TICK_LEN_NS) {
      scene_tick(&app->scene, TICK_LEN_SEC,
                 app->screen.cols, app->screen.rows);
      sim_accum -= TICK_LEN_NS;
    }

    /* how far between two physics steps we are, for smooth drawing (0..1) */
    float alpha = (float)sim_accum / (float)TICK_LEN_NS;

    fps_counter_tick(&app->fps, dt);

    /* sleep before drawing so terminal writes stay inside the frame budget */
    int64_t budget_left = FRAME_BUDGET_NS - (clock_ns() - frame_start);
    clock_sleep_ns(budget_left);

    screen_draw(&app->screen, &app->scene,
                app->fps.display, app->sim_fps, alpha);
    screen_present();

    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
