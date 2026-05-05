# Pass 1 — crowd.c: Reynolds Steering Crowd Simulator

## Core Idea

N agents (5–150 people) moving under six switchable crowd behaviours. All behaviours share the same Euler-integrated physics: accumulate weighted steering forces, clamp speed, integrate position. Pressing 1–6 switches behaviour live without resetting positions.

## Mental Model

Each person is a steered particle with momentum. The "force" is not a physical force — it's the difference between where you want to go and where you're currently going. The magic of Reynolds steering is that this single formula `force = desired_velocity − current_velocity` produces naturally smooth, momentum-based motion without any explicit interpolation.

## Key Equations

**Seek:** `desired = normalise(target − pos) × speed`, `force = desired − vel`

**Flee:** `desired = normalise(pos − threat) × speed`, `force = desired − vel` (direction reversed)

**Separation:** `for neighbours within SEP_RADIUS: force += (1 − d/SEP_RADIUS) × normalise(away)` — pushes agents apart; stronger the closer they are.

**Cohesion:** `desired = normalise(local_centroid − pos) × speed` — pulls toward the average position of nearby agents.

**Alignment:** `force = avg_velocity_of_neighbours − vel` — steers toward the average velocity of nearby agents.

**Pixel→cell aspect bridge:** `cx = round(px / CELL_W)` (CELL_W = 8), `cy = round(py / CELL_H)` (CELL_H = 16).

## Six Behaviours

| Behaviour | Forces |
|---|---|
| WANDER | Seek random wandering target + separation |
| FLOCK | Classic boids: separation + alignment + cohesion |
| PANIC | Flee roaming threat `!` + separation (flee overrides cohesion) |
| GATHER | Seek screen centre + separation; speed scales down inside slow radius |
| FOLLOW | Each agent seeks the next agent in an index chain; index 0 leads |
| QUEUE | Each agent claims a slot in a 3-row staggered line ending at the right-edge counter `>>|` |

## Worked Example (defaults: 60 people, 80×24 terminal, 60 Hz)

- World box: 200 cols × 60 rows = 1600 × 960 pixels.
- Per tick (1/60 s):
  - person at `SPEED_BASE = 80 px/s` advances ~1.3 px (one new cell every ~6 ticks ≈ 100 ms — visibly smooth)
- Personal space: `SEP_RADIUS = 40 px = 5 columns`. Two people in adjacent cells (8 px apart) are well inside the bubble: `strength = (40−8)/40 = 0.8`, so each pushes the other with `0.8 · SPEED_BASE = 64 px/s²` of acceleration — they're two cells apart within ~3 ticks.
- Steering cost: separation alone is O(N²). At 60 people → 60·59 = 3540 distance checks/tick, ~210 000/s — microseconds total on any modern CPU.
- Boundary handling per behaviour: WANDER/FLOCK/GATHER/FOLLOW wrap toroidally (a person leaving x = 1599 reappears at x = 0); PANIC bounces (vel.x flips at the wall) so the threat traps the crowd; QUEUE clamps so people stop at the line.

## Data Structures

### Person — grouped fields
```
/* kinematic state — written by physics every tick */
pos / prev_pos / vel / target

/* render identity — written once at spawn, read by scene_draw only */
glyph, color
```
The two groups never cross — the renderer never reads `vel`; the physics never reads `glyph`. The split makes the data flow visible at the type level.

### Scene — grouped fields
```
/* agent pool — first `count` Person slots are active; the rest are
 * pre-spawned so '+' reveals already-placed people without a fresh
 * randomise pass */
people[CROWD_MAX], count

/* user mode flags — written by app_handle_key, read by scene_tick */
behaviour, paused

/* PANIC threat — only tick_panic and the '!' renderer touch these */
threat_pos, threat_vel, threat_target

/* world dimensions in pixels — refreshed each tick from cols/rows */
world_w, world_h
```

## Non-Obvious Design Decisions

- **PANIC flee overrides cohesion.** When frightened, staying with the group (cohesion) conflicts with running away. Panic gives flee force high weight and cohesion weight = 0. This causes the crowd to scatter, which looks realistic.
- **FOLLOW uses index chain, not nearest-neighbor.** Agent `i` follows agent `i-1`. This produces a snake-like trail rather than everyone chasing the nearest person.
- **Agent count is runtime-adjustable (`+/-` keys).** The pool uses a live `count` counter; pre-spawned slots beyond that index are simply not ticked or drawn — `+` reveals them instantly with no randomise pass.
- **`mark_cell` helper.** All glyph stamps in `scene_draw` route through a single `mark_cell(w, cx, cy, ch, pair, attr, cols, rows)` that performs the `(chtype)(unsigned char)` cast and bounds-check. The QUEUE counter `>>|` is rendered as three individual `mark_cell` calls instead of a multi-byte `mvwprintw`, so each glyph is independently clip-safe.
- **HUD pairs split per CLAUDE.md spec.** Top-right status uses `PAIR_HUD = 8` (bright yellow, A_BOLD); bottom-left key hint uses `PAIR_HINT = 9` (bright cyan, A_BOLD). The hint is `A_BOLD` (NEVER `A_DIM`) so it stays readable against any animation behind it.
- **Background `-1` (terminal default).** All seven agent palette colours sit in the bright half of the cube (≥ 33), legal under `A_DIM` so wandering agents stay readable.
- **Frame-cap fix.** The classic `+ dt` bug was removed: `elapsed = clock_ns() − frame_start` (no `+ dt`). Adding `dt` cancels the cap and pegs CPU at 100 %.

## Open Questions to Explore

1. What happens if FLOCK agents also have a weak goal-seek force? Does the flock move toward a target?
2. In PANIC mode, make the threat `!` seek the densest cluster. Does it create a cat-and-mouse chase?
3. FOLLOW with only 5 agents vs 150 — how does the snake length change the motion?
4. Can you add a LEADER behaviour where one agent has a goal and the rest flock around it?

---

# Pass 2 — Pseudocode & Data Flow

## Module Map
```
§1 config     behaviour enum, speed limits, weights, queue geometry,
              PAIR_HUD/PAIR_HINT pair IDs
§2 clock      clock_ns(), clock_sleep_ns()
§3 color      color_init() — bg=-1, PAIR_HUD/PAIR_HINT registered
§4 coords     pw/ph, px_to_cell_x/y, Vec2 helpers, bounce_pos
§5 entity     Person (grouped), person_spawn(), person_step()
§6 steering   steer_seek(), steer_flee(), steer_separate(),
              steer_align(), steer_cohere()
§7 scene      Scene (grouped), scene_init, scene_tick() with
              behaviour dispatch, mark_cell() helper, scene_draw()
§8 app        screen_*, app_handle_key, main()
```

## Core Loop
```
main():
  scene_init()
  loop:
    frame_start = clock_ns()
    if need_resize: app_do_resize()
    dt = clock_ns() - frame_time;  cap 100 ms
    sim_accum += dt
    while sim_accum >= tick_ns:
      scene_tick(dt_sec, cols, rows):
        switch behaviour:
          WANDER → tick_wander
          FLOCK  → tick_flock
          PANIC  → tick_panic   (advances threat first, then bounces all)
          GATHER → tick_gather
          FOLLOW → tick_follow  (leader=people[0])
          QUEUE  → tick_queue   (slots sorted by index)
      sim_accum -= tick_ns
    alpha = sim_accum / tick_ns
    fps_counter (500 ms window)
    /* frame cap — `elapsed = clock_ns() - frame_start` (no +dt!) */
    sleep(NS_PER_SEC/TARGET_FPS - elapsed)
    screen_draw -> erase, scene_draw, HUD top + bottom hint, doupdate
    getch -> app_handle_key:
      1-6 -> behaviour, +/- adjust count, r -> reset, space -> pause,
      q -> exit
```

## Behaviour Dispatch
```
switch(behaviour):
  WANDER:  seek(target[i]) + separate()
           on arrive: target[i] = random pos
  FLOCK:   separate() + align(neighbours) + cohesion(neighbours)
  PANIC:   flee(threat_pos) · HIGH_WEIGHT + separate()
           threat_pos itself wanders toward its own target
  GATHER:  seek(screen_centre, decel_in_slow_radius) + separate()
  FOLLOW:  for i ≥ 1: seek(people[i-1].pos) + separate()
           people[0] wanders independently
  QUEUE:   slot_x = sx − i · QUEUE_SLOT_W
           slot_y = sy + ((i % 3) − 1) · QUEUE_SLOT_H
           seek(slot, decel_near_slot) + separate · 0.5
```

## scene_draw Painter's Order
```
scene_draw(alpha):
  /* per-behaviour decorations under the crowd */
  if PANIC:  stamp '!' at threat_pos        (PAIR 1, BOLD+BLINK)
  if QUEUE:  stamp '|' bar + '>>|' counter  (PAIR 3, BOLD)
  if FOLLOW: stamp '@' leader               (PAIR 3, BOLD+UNDERLINE)

  /* the crowd */
  for each active person (skipping leader if FOLLOW):
    dp = lerp(prev_pos, pos, alpha)
    mark_cell(cx, cy, glyph, color, A_NORMAL)
```
