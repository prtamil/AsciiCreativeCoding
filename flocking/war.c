/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * war.c — ASCII Two-Faction Battle Simulator  (archers + strategies)
 *
 * GONDOR (cyan uppercase + green '@' archers)
 *   vs
 * MORDOR (red lowercase + orange '%' archers)
 *
 * Archers fire real '-' projectile arrows that travel across the screen
 * and deal damage on contact with their target.
 *
 * Six battle strategies — press 1–6, live-switch (no reset needed)
 * ────────────────────────────────────────────────────────────────────
 *   1  STANDARD        balanced advance, engage near contact, flee at low HP
 *   2  BERSERKER       no routing, fast attacks, dense melee rush
 *   3  SHIELD WALL     slow tight ranks, high sep, long archer range
 *   4  GUERRILLA       hit-and-run, flee early, quick rally, mobile archers
 *   5  ARCHER FOCUS    extended range, rapid fire, archers stay deep
 *   6  CHAOS           sprint, ignore formation, brawl to the death
 *
 * Unit types
 * ──────────
 *   UNIT_MELEE   advance → lock on → brawl → flee if HP low
 *   UNIT_ARCHER  hold stand_off_dist, fire '-' arrows on timer,
 *                panic-flee if enemy closes or HP drops too low
 *
 * State machine (four states; both unit types use same states)
 * ─────────────────────────────────────────────────────────────
 *   ADVANCE → enemy in range    → COMBAT
 *   COMBAT  → target dies       → ADVANCE
 *   COMBAT  → HP too low        → FLEE
 *   FLEE    → safe + rested     → ADVANCE
 *   Any     → HP == 0           → DEAD
 *
 * Visual feedback
 * ───────────────
 *   A_BOLD   full HP      A_DIM    last HP
 *   A_BLINK  routing      '*'      arrow-hit flash
 *   '-'      arrow in flight (green = Gondor, orange = Mordor)
 *
 * Keys
 * ────
 *   q / ESC    quit       space   pause/resume     r  reset
 *   g          +gondor    m       +mordor
 *   1–6        switch battle strategy (live, no reset)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra flocking/war.c -o war -lncurses -lm
 *
 * §1 config   §2 clock   §3 color   §4 coords & vec2
 * §5 entity   §6 combat  §7 scene   §8 app
 */

/* ── ARCHER BEHAVIOUR PRIMER ─────────────────────────────────────── *
 *
 * Archers are purely distance-driven — no target lock.
 *
 *   dist > arrow_range         → ADVANCE: seek stand_off_dist behind
 *                                 the enemy centroid on our own side.
 *
 *   archer_flee_range < dist   → COMBAT: hold standoff, fire every
 *       ≤ arrow_range            shoot_interval seconds.  Each shot
 *                                 spawns a '-' Arrow projectile that
 *                                 travels at ARROW_TRAVEL_SPD px/s.
 *
 *   dist < archer_flee_range   → FLEE: sprint away; rally once safe.
 *   OR  hp ≤ archer_flee_hp
 *
 * POOL LAYOUT
 *   All warriors share one flat pool[].  New warriors always append at
 *   pool[n_total++].  Dead warriors keep their slot (no compaction).
 * ──────────────────────────────────────────────────────────────────── */

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

/* ── army sizes — fixed, not strategy-dependent ── */
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
    N_COLORS         =   7,
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
 * Seven color pairs:  1=red  2=orange  3=yellow  4=green  5=cyan
 *                     6=blue  7=magenta
 *
 * War assignments:
 *   Gondor melee   → 5 cyan        Mordor melee   → 1 red
 *   Gondor archers → 4 green       Mordor archers → 2 orange
 *   Gondor arrows  → 4 green       Mordor arrows  → 2 orange
 *   Corpses/HUD    → 3 yellow
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);
        init_pair(2, 208, COLOR_BLACK);
        init_pair(3, 226, COLOR_BLACK);
        init_pair(4,  46, COLOR_BLACK);
        init_pair(5,  51, COLOR_BLACK);
        init_pair(6,  21, COLOR_BLACK);
        init_pair(7, 201, COLOR_BLACK);
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
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
 * pos / prev_pos  pixel-space position this tick and last tick.
 *                 Renderer lerps between them by alpha for smooth draw.
 * vel             velocity in pixels per second.
 * target_idx      pool index of the locked enemy (melee only).
 *                 -1 = no target.  Archers always retarget nearest enemy.
 * hit_timer       > 0: draw '*' flash (just struck by an arrow).
 */
typedef struct {
    Vec2         pos;
    Vec2         prev_pos;
    Vec2         vel;
    int          faction;
    UnitType     unit_type;
    int          hp;
    float        atk_timer;
    float        rally_timer;
    int          target_idx;
    WarriorState state;
    float        dead_timer;
    float        hit_timer;
    char         glyph;
    int          color_pair;
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

/* ── melee state machine ─────────────────────────────────────────── */

/*
 * melee_logic — one melee warrior for one physics tick.
 *
 * ADVANCE  seek enemy centroid + separate from allies.
 *          Nearest enemy enters engage_range → lock on → COMBAT.
 *
 * COMBAT   slow footwork toward locked target; deal ATK_DAMAGE every
 *          atk_interval seconds.
 *          Target dies → ADVANCE.   HP ≤ flee_hp → FLEE.
 *
 * FLEE     sprint away from nearest enemy.
 *          Past safe_range for rally_time seconds → ADVANCE.
 *
 * All constants read from g_sp, so strategy changes take effect instantly.
 */
static void melee_logic(Warrior *pool, int n_total, int self,
                        float ww, float wh, float dt)
{
    Warrior *w = &pool[self];
    Vec2 force = v2(0, 0);

    switch (w->state) {

    case STATE_ADVANCE: {
        Vec2 centroid = enemy_centroid(pool, n_total, w->faction, ww, wh);
        int  ne       = nearest_enemy_idx(pool, n_total, self);
        if (ne >= 0 &&
            v2len(v2sub(pool[ne].pos, w->pos)) < g_sp->engage_range) {
            w->state      = STATE_COMBAT;
            w->target_idx = ne;
            w->atk_timer  = g_sp->atk_interval; /* reset: no instant first hit */
            break;
        }
        force = v2add(
            v2scale(steer_seek    (w->pos, w->vel, centroid, g_sp->speed_advance),
                    g_sp->w_seek),
            v2scale(steer_separate(pool, n_total, self),
                    g_sp->w_sep)
        );
        warrior_step(w, force, g_sp->speed_advance, dt);
        bounce_pos(&w->pos, &w->vel, ww, wh);
        return;
    }

    case STATE_COMBAT: {
        if (w->target_idx < 0)          { w->state = STATE_ADVANCE; break; }
        if (w->hp <= g_sp->flee_hp)     { w->state = STATE_FLEE;
                                           w->target_idx = -1; break; }
        w->atk_timer -= dt;
        if (w->atk_timer <= 0.0f) {
            w->atk_timer = g_sp->atk_interval;
            pool[w->target_idx].hp -= ATK_DAMAGE;
        }
        Vec2 tgt = pool[w->target_idx].pos;
        force = v2add(
            v2scale(steer_seek    (w->pos, w->vel, tgt, g_sp->melee_speed),
                    g_sp->w_seek),
            v2scale(steer_separate(pool, n_total, self),
                    g_sp->w_sep * 0.4f)  /* reduced: stay near target */
        );
        warrior_step(w, force, g_sp->melee_speed * 1.5f, dt);
        bounce_pos(&w->pos, &w->vel, ww, wh);
        return;
    }

    case STATE_FLEE: {
        int ne = nearest_enemy_idx(pool, n_total, self);
        if (ne < 0) { w->state = STATE_ADVANCE; break; }
        float d = v2len(v2sub(pool[ne].pos, w->pos));
        if (d >= g_sp->safe_range) {
            w->rally_timer += dt;
            w->vel = v2scale(w->vel, 0.92f);  /* gentle friction while safe */
            if (w->rally_timer >= g_sp->rally_time) {
                w->state = STATE_ADVANCE; w->rally_timer = 0.0f;
            }
        } else {
            w->rally_timer = 0.0f;
            force = v2add(
                v2scale(steer_flee    (w->pos, w->vel, pool[ne].pos,
                                       g_sp->speed_flee), g_sp->w_flee),
                v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
            );
        }
        warrior_step(w, force, g_sp->speed_flee, dt);
        bounce_pos(&w->pos, &w->vel, ww, wh);
        return;
    }

    default: break;
    }

    /* State transition occurred mid-switch: integrate with zero force */
    warrior_step(w, v2(0,0), g_sp->speed_advance, dt);
    bounce_pos(&w->pos, &w->vel, ww, wh);
}

/* ── archer state machine ────────────────────────────────────────── */

/*
 * archer_logic — one archer for one physics tick.
 *
 * Archers are distance-driven; no target lock.  Nearest enemy selected
 * each tick.  Shooting produces a '-' Arrow in the pool rather than
 * dealing instant damage.  The arrow travels and hits on contact.
 *
 * HP rout: if hp ≤ archer_flee_hp, immediately enter FLEE (same idea
 * as melee's flee_hp check — archers are fragile).  archer_flee_hp = 0
 * disables this (BERSERKER, CHAOS strategies).
 *
 * Rally logic mirrors melee: once past safe_range for rally_time seconds
 * the archer returns to ADVANCE.
 */
static void archer_logic(Warrior *pool, int n_total, int self,
                         float ww, float wh, float dt,
                         Arrow *arrows, int *n_arrows)
{
    Warrior *w  = &pool[self];
    int      ne = nearest_enemy_idx(pool, n_total, self);

    if (ne < 0) {
        /* No enemies left — coast to a stop */
        w->state = STATE_ADVANCE;
        w->vel   = v2scale(w->vel, 0.92f);
        warrior_step(w, v2(0,0), g_sp->archer_speed, dt);
        bounce_pos(&w->pos, &w->vel, ww, wh);
        return;
    }

    /* HP-based panic: archer routes immediately when too wounded */
    if (g_sp->archer_flee_hp > 0 && w->hp <= g_sp->archer_flee_hp
        && w->state != STATE_FLEE) {
        w->state       = STATE_FLEE;
        w->rally_timer = 0.0f;
    }

    float dist            = v2len(v2sub(pool[ne].pos, w->pos));
    Vec2  away_from_enemy = v2norm(v2sub(w->pos, pool[ne].pos));
    Vec2  force;
    float max_spd;

    if (dist < g_sp->archer_flee_range) {
        /* Distance panic: enemy entered melee range of the archer */
        w->state       = STATE_FLEE;
        w->rally_timer = 0.0f;   /* reset: still in immediate danger */
        force   = v2add(
            v2scale(steer_flee    (w->pos, w->vel, pool[ne].pos,
                                   g_sp->speed_flee), g_sp->w_flee),
            v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
        );
        max_spd = g_sp->speed_flee;

    } else if (w->state == STATE_FLEE) {
        /* HP-triggered flee still active; enemy not yet in melee range */
        force   = v2add(
            v2scale(steer_flee    (w->pos, w->vel, pool[ne].pos,
                                   g_sp->speed_flee), g_sp->w_flee),
            v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
        );
        max_spd = g_sp->speed_flee;

    } else if (dist <= g_sp->arrow_range) {
        /* COMBAT (shoot mode): hold standoff position, fire on timer */
        w->state      = STATE_COMBAT;
        w->atk_timer -= dt;
        if (w->atk_timer <= 0.0f) {
            w->atk_timer = g_sp->shoot_interval;
            /* spawn '-' projectile toward target instead of instant damage */
            if (*n_arrows < ARROW_POOL_MAX) {
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
        }
        /* Drift to standoff point: STAND_OFF_DIST behind nearest enemy */
        Vec2 standoff = v2add(pool[ne].pos,
                               v2scale(away_from_enemy, g_sp->stand_off_dist));
        force   = v2add(
            v2scale(steer_seek    (w->pos, w->vel, standoff,
                                   g_sp->archer_speed * 0.4f), g_sp->w_seek),
            v2scale(steer_separate(pool, n_total, self), g_sp->w_sep * 0.5f)
        );
        max_spd = g_sp->archer_speed * 0.5f;

    } else {
        /* ADVANCE: too far to shoot — close toward stand_off_dist */
        w->state = STATE_ADVANCE;
        Vec2 centroid    = enemy_centroid(pool, n_total, w->faction, ww, wh);
        /* Direction from centroid toward this archer → point behind centroid */
        Vec2 safe_dir    = v2norm(v2sub(w->pos, centroid));
        Vec2 advance_tgt = v2add(centroid,
                                  v2scale(safe_dir, g_sp->stand_off_dist));
        force   = v2add(
            v2scale(steer_seek    (w->pos, w->vel, advance_tgt,
                                   g_sp->archer_speed), g_sp->w_seek),
            v2scale(steer_separate(pool, n_total, self), g_sp->w_sep)
        );
        max_spd = g_sp->archer_speed;
    }

    warrior_step(w, force, max_spd, dt);
    bounce_pos(&w->pos, &w->vel, ww, wh);

    /* Rally from FLEE: count time at safe distance → return to ADVANCE */
    if (w->state == STATE_FLEE) {
        float post_dist = v2len(v2sub(pool[ne].pos, w->pos));
        if (post_dist >= g_sp->safe_range) {
            w->rally_timer += dt;
            if (w->rally_timer >= g_sp->rally_time) {
                w->state = STATE_ADVANCE; w->rally_timer = 0.0f;
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
 * pool[]      flat warrior pool for both factions; new warriors always
 *             append at pool[n_total++]; dead warriors keep their slot.
 * arrows[]    flat arrow pool; inactive arrows compacted each tick.
 * n_alive[]   recomputed each tick; drives victory detection.
 * kills[]     cumulative faction kill counts; drives HUD.
 * winner      -1=ongoing  0=GONDOR  1=MORDOR
 */
typedef struct {
    Warrior pool[POOL_MAX];
    int     n_total;
    int     n_alive[2];
    int     n_archers[2];
    int     kills[2];
    int     winner;
    float   world_w, world_h;
    bool    paused;
    Arrow   arrows[ARROW_POOL_MAX];
    int     n_arrows;
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
 * scene_draw — render everything into WINDOW *w.
 *
 * Draw order (later passes overwrite earlier):
 *   pass 0  arrows '-' in flight (green = Gondor, orange = Mordor)
 *   pass 1  corpses '.' dim
 *   pass 2  living warriors with HP-driven attributes; '*' flash on hit
 *   pass 3  victory banner when battle is decided
 *
 * Living warriors use sub-tick alpha interpolation:
 *   draw_pos = prev_pos + (pos − prev_pos) × alpha
 * Arrows use current pos (no prev_pos stored; they move fast enough).
 */
static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows, float alpha)
{
    /* ── pass 0: arrows in flight ── */
    for (int i = 0; i < s->n_arrows; i++) {
        const Arrow *a = &s->arrows[i];
        if (!a->active) continue;
        int cx = px_to_cell_x(a->pos.x), cy = px_to_cell_y(a->pos.y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        int cpair = (a->faction == GONDOR) ? 4 : 2;
        wattron(w, COLOR_PAIR(cpair) | A_BOLD);
        mvwaddch(w, cy, cx, '-');
        wattroff(w, COLOR_PAIR(cpair) | A_BOLD);
    }

    /* ── pass 1: corpses ── */
    for (int i = 0; i < s->n_total; i++) {
        const Warrior *wr = &s->pool[i];
        if (wr->state != STATE_DEAD || wr->dead_timer <= 0.0f) continue;
        int cx = px_to_cell_x(wr->pos.x), cy = px_to_cell_y(wr->pos.y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;
        wattron(w, COLOR_PAIR(3) | A_DIM);
        mvwaddch(w, cy, cx, '.');
        wattroff(w, COLOR_PAIR(3) | A_DIM);
    }

    /* ── pass 2: living warriors ── */
    for (int i = 0; i < s->n_total; i++) {
        const Warrior *wr = &s->pool[i];
        if (wr->state == STATE_DEAD) continue;

        Vec2 dp = v2add(wr->prev_pos,
                        v2scale(v2sub(wr->pos, wr->prev_pos), alpha));
        int cx = px_to_cell_x(dp.x), cy = px_to_cell_y(dp.y);
        if (cx < 0 || cx >= cols || cy < 0 || cy >= rows) continue;

        /* Arrow-strike flash overrides normal glyph briefly */
        if (wr->hit_timer > 0.0f) {
            wattron(w, COLOR_PAIR(wr->color_pair) | A_STANDOUT | A_BOLD);
            mvwaddch(w, cy, cx, '*');
            wattroff(w, COLOR_PAIR(wr->color_pair) | A_STANDOUT | A_BOLD);
            continue;
        }

        attr_t attr = (attr_t)COLOR_PAIR(wr->color_pair);
        if      (wr->hp >= HP_MAX)       attr |= A_BOLD;
        else if (wr->hp <= 1)            attr |= A_DIM;
        if (wr->state == STATE_FLEE)     attr |= A_BLINK;

        wattron(w, attr);
        mvwaddch(w, cy, cx, (chtype)(unsigned char)wr->glyph);
        wattroff(w, attr);
    }

    /* ── victory banner ── */
    if (s->winner >= 0) {
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

    /* Centre: show active strategy name (or PAUSED) */
    char title_buf[48];
    snprintf(title_buf, sizeof title_buf, "[ WAR: %-12s ]",
             sc->paused ? "PAUSED" : g_sp->name);
    int tx = (s->cols - (int)strlen(title_buf)) / 2;
    if (tx < 0) tx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, tx, "%s", title_buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    char fps_buf[40];
    snprintf(fps_buf, sizeof fps_buf, " %.0ffps sim:%dHz ", fps, sim_fps);
    int fx = s->cols - (int)strlen(fps_buf);
    if (fx < 0) fx = 0;
    attron(COLOR_PAIR(3));
    mvprintw(1, fx, "%s", fps_buf);
    attroff(COLOR_PAIR(3));

    attron(COLOR_PAIR(3));
    mvprintw(s->rows-1, 0,
        " q:quit  spc:pause  r:reset  g:+gondor  m:+mordor"
        "  1-6:strategy ");
    attroff(COLOR_PAIR(3));
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
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns(); sim_accum = 0;
        }

        /* ① */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ② */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ③ */
        float alpha = (float)sim_accum / (float)tick_ns;

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0; fps_accum = 0;
        }

        /* ④ */
        int64_t elapsed = clock_ns() - frame_time + dt;
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
