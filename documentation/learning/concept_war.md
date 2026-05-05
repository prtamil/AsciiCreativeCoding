# Pass 1 — war.c: Two-Faction ASCII Battle with Decomposed State Machines

## Core Idea

GONDOR (cyan) vs MORDOR (red). Two unit types — melee warriors and archers. Every unit runs a 4-state FSM: ADVANCE toward enemies → COMBAT (melee brawl or ranged fire) → FLEE (low HP) → back to ADVANCE when safe. Archers fire real `-` projectile arrows that travel across the screen and deal damage on contact.

The combat state machines `melee_logic` and `archer_logic` are now **thin dispatchers** that call per-state helper functions returning a `Steer { force, max_spd }` struct. Each state's behaviour lives in its own ≤30-line function (`melee_advance`, `melee_combat`, `melee_flee`, `archer_advance`, `archer_combat`, `archer_flee_force`, plus a helper `archer_shoot` for projectile spawn). The dispatcher just picks the helper, applies the result through `warrior_step + bounce_pos`, and runs the rally-timer post-step where applicable.

## Mental Model

Think of each unit as a simple autonomous agent governed by distance thresholds. The `engage_range` parameter is the trigger radius — inside it, combat begins. `flee_hp` is the cowardice threshold. `stand_off_dist` is where archers prefer to hover. The 6 strategies just change these threshold values, instantly reshaping every unit's behaviour on the next tick.

The decomposition gives a clean reading order: open the file, find the FSM section, see one helper per state. Read each helper independently — none is longer than ~30 lines. The dispatcher is 16 lines (melee) or 61 lines (archer, which has the distance ladder + rally tick); both are at or under the 60-line orchestration ceiling.

## Key Equations

**Seek steering:** `desired = normalise(target − pos) × speed`, `force = desired − vel`. Used for ADVANCE, FLEE, and archer repositioning.

**Arrow travel:** `arrow.pos += arrow.vel × dt`. `arrow.vel = normalise(target_pos − archer_pos) × ARROW_TRAVEL_SPD` (set once at spawn, no homing). Hit detection: when `|target.pos − arrow.pos| < ARROW_HIT_DIST` deal `ATK_DAMAGE` + flash; miss when arrow exits world bounds.

**Separation:** Per same-faction ally with `0 < d < sep_radius`: `force += unit(self − ally) · ((sep_radius − d)/sep_radius) · speed_advance`. Only same-faction warriors push each other (fight through enemies).

**HP-driven attributes (renderer):**
- `hp == HP_MAX` → `A_BOLD` (full strength)
- `hp == 1` → `A_DIM` (last hit)
- `state == FLEE` → `|= A_BLINK` (panic)

## Data Structures

### Warrior — grouped fields
```
/* kinematic state — alpha-lerped on draw */
pos / prev_pos / vel

/* identity — set once at spawn */
faction, unit_type, glyph, color_pair

/* HP + state machine */
hp, state, target_idx

/* timers (seconds) */
atk_timer, rally_timer, dead_timer, hit_timer
```

### Scene — grouped fields
```
/* warrior pool — both factions interleaved by spawn order */
pool[POOL_MAX], n_total

/* arrow pool — compacted each tick */
arrows[ARROW_POOL_MAX], n_arrows

/* per-faction tallies — recomputed in scene_tick */
n_alive[2], n_archers[2], kills[2], winner

/* world dimensions in pixels — refreshed each tick */
world_w, world_h

/* user mode flags */
paused
```

### Steer (combat helper output)
```
typedef struct { Vec2 force; float max_spd; } Steer;
```
Returned by every per-state behaviour helper; consumed by the dispatcher's `warrior_step + bounce_pos`.

### StrategyParams (g_presets[6], cycled with 1-6)
```
StrategyParams {
  name,
  engage_range, flee_hp, atk_interval, speed_advance, speed_flee,
  sep_radius, safe_range, rally_time, melee_speed,
  archer_flee_hp, arrow_range, archer_flee_range, stand_off_dist,
  shoot_interval, archer_speed,
  w_seek, w_sep, w_flee
}
```

### Arrow
```
Arrow { pos, vel, target_idx, faction, active }
```

## Non-Obvious Design Decisions

- **Per-state helpers return `Steer`, not write through outparams.** A struct return makes the contract explicit — every helper MUST set both `force` and `max_spd` — and keeps call sites readable (`s = melee_advance(...)`, then `warrior_step(w, s.force, s.max_spd, dt)`). C return-value optimization makes this as cheap as outparams at -O2.
- **Arrows are NOT instant.** Projectiles travel at `ARROW_TRAVEL_SPD = 220 px/s`. An archer at 160 px range takes ~0.7 s for the arrow to land, giving the target time to move out of the way — archers need to lead moving targets slightly.
- **Flat append-only arrow pool (size 80) with per-tick compaction.** When a slot is deactivated, the next `arrows_tick` shifts all active arrows to the front. No linked list, no free-list — cheap at 80 entries, avoids pointer aliasing bugs.
- **Painter's-order rendering decomposed:** arrows → corpses → living warriors → victory banner. Each pass is its own helper (`draw_arrows`, `draw_corpses`, `draw_living`, `draw_victory_banner`); a `warrior_attr` helper extracts the HP/state-driven attribute logic. `scene_draw` is a 4-line orchestrator.
- **`g_sp` global pointer pattern.** `const StrategyParams *g_sp = &g_presets[idx]` — switching strategy updates one pointer; all logic reads through `g_sp`, so the change takes effect on the very next tick with no reset needed.
- **Rally timer prevents flip-flopping.** After fleeing, a unit must wait `rally_time` seconds before re-entering ADVANCE. Without this, a unit at exactly the `flee_hp` threshold oscillates between COMBAT and FLEE every tick.
- **HUD pairs split per CLAUDE.md spec.** Faction-coloured side counts (cyan/red) carry semantic information ("this number belongs to that side"); the centre strategy title uses `PAIR_HUD` (bright yellow + A_BOLD); the bottom hint uses `PAIR_HINT` (bright cyan + A_BOLD). All glyph stamps go through a `mark_cell()` helper that performs the `(chtype)(unsigned char)` cast and bounds-check.

## Open Questions to Explore

1. What if arrows could miss? Add a spread angle to the arrow velocity at spawn.
2. Can you implement friendly fire? Arrows currently only damage their intended `target_idx`.
3. What does the battle look like with 3 factions instead of 2?
4. CHAOS strategy ignores formation — what formation emerges naturally from SHIELD_WALL?

---

# Pass 2 — Pseudocode & Data Flow

## Module Map (matches §1 … §8 source)
```
§1 config     StrategyParams, g_presets[6], g_sp pointer,
              PAIR_HUD/PAIR_HINT enum constants
§2 clock      clock_ns(), clock_sleep_ns()
§3 color      color_init() — faction palette + PAIR_HUD/PAIR_HINT,
              bg = -1, theme-independent HUD pairs
§4 vec2       Vec2 + helpers, bounce_pos()
§5 entity     Warrior + Arrow (grouped fields), warrior_spawn(),
              warrior_step()
§6 combat     steer_seek/flee/separate, enemy_centroid,
              nearest_enemy_idx,
              Steer struct,
              melee_advance/combat/flee + melee_logic dispatcher,
              archer_shoot, archer_flee_force, archer_combat,
              archer_advance + archer_logic dispatcher,
              warrior_tick (kill credit + dispatch by unit type)
§7 scene      Scene (grouped fields), scene_init, scene_add_warriors,
              arrows_tick, scene_tick,
              mark_cell helper, warrior_attr helper,
              draw_arrows / draw_corpses / draw_living /
              draw_victory_banner, scene_draw orchestrator
§8 app        screen_*, app_handle_key, main()
```

## Core Loop
```
main():
  scene_init()
  loop:
    frame_start = clock_ns()
    if need_resize: app_do_resize(); reset frame_time, sim_accum
    dt = clock_ns() - frame_time;  cap 100 ms
    sim_accum += dt
    while sim_accum >= tick_ns:
      scene_tick(dt_sec):
        for each warrior alive: warrior_tick (handles death,
                                  dispatches to melee_logic or
                                  archer_logic)
        arrows_tick: advance + hit-test + miss-test + compact
        recompute n_alive, n_archers, kills, winner
      sim_accum -= tick_ns
    alpha = sim_accum / tick_ns
    fps_counter (500 ms window)
    /* frame cap — `elapsed = clock_ns() - frame_start` (no +dt!) */
    sleep(NS_PER_SEC/TARGET_FPS - elapsed)
    screen_draw -> erase, scene_draw, HUD, doupdate
    getch loop -> app_handle_key:
      1-6 -> g_strat_idx, g_sp = &g_presets[idx]
      space -> paused, r -> reset, g/m -> add reinforcements,
      q -> exit
```

## melee_logic — thin dispatcher
```
melee_logic(pool, n, self, ww, wh, dt):
  w = &pool[self]
  s = { v2(0,0), g_sp->speed_advance }
  switch w->state:
    ADVANCE: s = melee_advance(w, pool, n, self, ww, wh)
    COMBAT:  s = melee_combat (w, pool, n, self, dt)
    FLEE:    s = melee_flee   (w, pool, n, self, dt)
  warrior_step(w, s.force, s.max_spd, dt)
  bounce_pos(&w->pos, &w->vel, ww, wh)
```

## archer_logic — distance-ladder dispatcher
```
archer_logic(pool, n, self, ww, wh, dt, arrows, n_arrows):
  ne = nearest_enemy_idx(...)
  if ne < 0: coast and return
  if HP panic and not FLEE: state = FLEE
  dist = |pool[ne].pos - w->pos|
  away = unit(w->pos - pool[ne].pos)
  if dist < archer_flee_range:    s = archer_flee_force(...)  state = FLEE
  elif state == FLEE:              s = archer_flee_force(...)
  elif dist <= arrow_range:        s = archer_combat(... arrows, n_arrows ...)
  else:                            s = archer_advance(...)    state = ADVANCE
  warrior_step + bounce_pos
  rally_tick if state == FLEE
```

## FSM Transitions
```
ADVANCE: nearest enemy within engage_range? → COMBAT
COMBAT (melee): target dead? → ADVANCE
               hp ≤ flee_hp? → FLEE
               else: seek target, attack on timer
COMBAT (archer): enemy within archer_flee_range? → FLEE
                 hp ≤ archer_flee_hp? → FLEE
                 dist > arrow_range? → ADVANCE
                 else: hold stand_off_dist, shoot on timer
FLEE: dist to all enemies > safe_range AND rally_timer elapsed? → ADVANCE
DEAD: terminal (corpse for CORPSE_LIFETIME, then ignored)
```
