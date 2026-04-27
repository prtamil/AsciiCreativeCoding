# Pass 1 — war.c: Two-Faction ASCII Battle Simulator

## Core Idea
GONDOR (cyan) vs MORDOR (red). Two unit types — melee warriors and archers. Every unit runs a 4-state FSM: ADVANCE toward enemies → COMBAT (melee brawl or ranged fire) → FLEE (low HP) → back to ADVANCE when safe. Archers fire real `-` projectile arrows that travel across the screen and deal damage on contact.

## Mental Model
Think of each unit as a simple autonomous agent governed by distance thresholds. The `engage_range` parameter is the trigger radius — inside it, combat begins. `flee_hp` is the cowardice threshold. `stand_off_dist` is where archers prefer to hover. The 6 strategies just change these threshold values, instantly reshaping every unit's behaviour on the next tick.

## Key Equations

**Seek steering:** `desired = normalise(target − pos) × speed`, `force = desired − vel`
Used for ADVANCE, FLEE, and archer repositioning.

**Arrow travel:** `arrow.pos += arrow.vel × dt`
`arrow.vel = normalise(target_pos − archer_pos) × ARROW_TRAVEL_SPD`
Arrow spawns at archer position; each tick updates position until within ARROW_HIT_DIST of target.

**Separation:** Same fixed-base pattern as swarm_gen_numbers.c — prevents units stacking.

## Data Structures
```
Warrior { pos, vel, hp, faction, type (MELEE/ARCHER), state (ADVANCE/COMBAT/FLEE/DEAD),
          atk_timer, rally_timer, glyph, color_pair }
Arrow   { pos, vel, target_idx, faction, active }
StrategyParams { name, engage_range, flee_hp, atk_interval, speed_advance,
                 speed_flee, sep_radius, safe_range, rally_time, melee_speed,
                 archer_flee_hp, arrow_range, archer_flee_range,
                 stand_off_dist, shoot_interval, archer_speed,
                 w_seek, w_sep, w_flee }
```

## Non-Obvious Design Decisions
- **Arrows are NOT instant.** Projectiles travel at ARROW_TRAVEL_SPD=220 px/s. This means an archer at 160 px range takes ~0.7 s for the arrow to land, giving the target time to move out of the way — archers need to lead moving targets slightly.
- **Flat append-only arrow pool (size 80).** No linked list or free-list. When a slot is deactivated, it stays in the array. Pool scans the full 80 entries each frame — cheap at 80 entries, avoids pointer aliasing bugs.
- **Two-pass rendering order:** arrows → corpses → living warriors → HUD. Arrows render below unit glyphs so a `-` projectile is never occluded by its target until it hits.
- **g_sp global pointer pattern.** `const StrategyParams *g_sp = &g_presets[idx]` — switching strategy updates one pointer; all logic reads through g_sp, so the change takes effect on the very next tick with no reset needed.
- **Rally timer prevents flip-flopping.** After fleeing, a unit must wait `rally_time` seconds before re-entering ADVANCE. Without this, a unit at exactly the flee_hp threshold oscillates between COMBAT and FLEE every tick.

## Open Questions to Explore
1. What if arrows could miss? Add a spread angle to the arrow velocity at spawn.
2. Can you implement friendly fire? Arrows currently only damage their intended target_idx.
3. What does the battle look like with 3 factions instead of 2?
4. CHAOS strategy ignores formation — what formation emerges naturally from SHIELD_WALL?

---

# Pass 2 — Pseudocode & Data Flow

## Module Map
```
§1 config     StrategyParams, g_presets[6], g_sp pointer
§2 clock      clock_ns(), clock_sleep_ns()
§3 color      color_init() — faction color pairs
§4 vec2       Vec2, v2(), steering helpers
§5 entity     Warrior, Arrow structs; warrior_spawn()
§6 combat     melee_logic(), archer_logic(), arrow_tick()
§7 scene      BattleScene, scene_init(), scene_tick(), scene_draw()
§8 app        screen_*, app_handle_key(), main()
```

## Core Loop
```
main():
  scene_init()   # spawn warriors for both factions
  loop:
    dt = elapsed (capped 100ms)
    scene_tick(dt):
      for each arrow: arrow.pos += arrow.vel * dt
                      if dist(arrow, arrow.target) < HIT_DIST: deal damage, deactivate
      for each warrior (alive):
        if MELEE: melee_logic()   # seek enemy, brawl, flee
        if ARCHER: archer_logic() # seek stand-off, shoot arrow on timer, flee
    screen_draw():
      pass 1: draw active arrows as '-'
      pass 2: draw corpses as '.' (dim)
      pass 3: draw living warriors
      pass 4: HUD banner with strategy name
    getch(): 1-6 switch strategy, r reset, g/m add units
```

## FSM Transitions
```
ADVANCE: nearest enemy within engage_range? → COMBAT
COMBAT (melee): target dead? → ADVANCE
               hp <= flee_hp? → FLEE
               else: seek target, attack on timer
COMBAT (archer): enemy within archer_flee_range? → FLEE
                 hp <= archer_flee_hp? → FLEE
                 dist > arrow_range? → ADVANCE
                 else: hold stand_off_dist, shoot on timer
FLEE: dist to all enemies > safe_range AND rally_timer elapsed? → ADVANCE
DEAD: (terminal, no transitions)
```
