# Pass 1 — swarm_gen_numbers.c: 160-Agent Reynolds Steering Swarm Forming ASCII Digits

## Core Idea

160 ASCII agents use Reynolds steering behaviours to coordinate into the pixel shapes of digits 0–9. The digit is a 5×7 bitmap; each `#` cell is **subdivided into a 3×3 grid of mini-slots** (`SLOT_GRID = 3`), so each `#` becomes 9 target points and a digit has 99–153 slots total. `N_AGENTS = 160` matches `SLOTS_MAX` so every slot of every digit can be filled — including the largest digits (5 and 8 with 17 `#` × 9 = 153 slots). Each agent is greedily assigned its nearest unoccupied slot and steered toward it using one of 10 switchable strategies. **10 colour themes** cycle live with `t` / `T`.

## Mental Model

Think of it as a hire/fire system in continuous space. Slots are job sites (a 3×3 cluster per `#`); agents are workers. Each worker moves toward their assigned job using a strategy (jog, sprint, spiral, bounce…). The `arrive` steering behaviour gives a smooth deceleration so workers don't oscillate around the job site. Separation force prevents workers from stacking on each other.

Because `N_AGENTS == SLOTS_MAX = 160`, the largest digits get **fully populated**: every mini-slot has its own agent, and the `#` cell reads as a dense 3×3 cluster of glyphs rather than a sparse outline. On the smallest digit (7, with 99 slots) the surplus 61 agents wander in `A_DIM` glyphs around the formed shape — visually fine because the digit silhouette is still clear.

## Key Equations

**Sub-slot placement (the "no rounding leak" trick).** Each `#` is subdivided into `SLOT_GRID × SLOT_GRID` mini-slots placed at exact terminal-cell corners (multiples of `CELL_W`/`CELL_H`):

```
stride_x = (DIGIT_CELL_W − 1) / (SLOT_GRID − 1)   /* (5−1)/(3−1) = 2 */
stride_y = (DIGIT_CELL_H − 1) / (SLOT_GRID − 1)   /* (3−1)/(3−1) = 1 */

px = ox + (c · DIGIT_CELL_W + sc · stride_x) · CELL_W
py = oy + (r · DIGIT_CELL_H + sr · stride_y) · CELL_H
```

`stride_x = 2` puts sub-cols at terminal cols 0, 2, 4 of each `#`'s 5-col allocation; `stride_y = 1` puts sub-rows at rows 0, 1, 2 of the 3-row allocation. **`DIGIT_CELL_H` is 3, not 2** — the previous value of 2 left a sub-slot leak at fractional fy positions where `floor(py/16 + 0.5)` would round into the **next** bitmap row's allocated terminal rows, blurring vertical structure between adjacent bitmap rows. That bug had digit 6 rendering as digit 5; matching `DIGIT_CELL_H` to `SLOT_GRID` fixed it.

**Seek:** `desired = normalise(target − pos) × speed`, `force = desired − vel`. Subtracting `vel` means force → 0 when already moving at full speed toward target — smooth deceleration for free.

**Arrive (decelerate near goal):** `desired_speed = max_speed × min(1, dist / slow_radius)`. Prevents the overshoot oscillation raw seek produces.

**Separation:** `force += (sep_radius − d) / sep_radius × SEP_BASE_FORCE × normalise(away)`. Fixed base force (60 px/s) — NOT proportional to `arrive_speed`. Tying it to `arrive_speed` caused RUSH (arrive=180) to produce separation strong enough to overpower slot attraction entirely.

**Spring (SPRING strategy):** `F = k × (slot − pos) − damping × vel`. With k=3.5, damping=2.0: ζ = damping/(2√k) ≈ 0.53 → underdamped → agent oscillates 1–2 cycles before settling.

**Wander fade:** `wander_strength *= min(dist_to_slot / FADE_DIST, 1.0)`. Without fading, wander keeps kicking agents out of their slots after arrival.

## Data Structures

```
Agent { pos, prev_pos, vel, wander_angle, slot_idx, glyph, color_pair }
Slot  { pos, occupied }
StrategyParams { name, max_speed, arrive_speed, slow_radius, slot_weight,
                 wander_strength, sep_radius, sep_weight,
                 cohesion_weight, align_weight, neighbor_radius }
Theme { name, body[7] }   /* 10 themes; t/T cycle */
```

### SwarmScene field groups
```
agent + slot pools — first n_slots slots active per digit
  agents[N_AGENTS=160], slots[SLOTS_MAX=160], n_slots

active selection
  current_digit, strategy, theme_idx     /* 0..9, 0..9, 0..9 */

simulation clock + world dimensions
  sim_time, world_w, world_h

user mode flags
  paused, auto_cycle, cycle_timer
```

## Non-Obvious Design Decisions

- **`DIGIT_CELL_H = 3`, matching `SLOT_GRID = 3`.** The fundamental fix for the "6 looks like 5" bug. With `DIGIT_CELL_H = 2` and `SLOT_GRID = 3`, each `#` allocates only 2 terminal rows but the renderer placed 3 sub-slot rows — the third leaked into the next bitmap row's allocated terminal rows. Setting both to 3 makes the mapping exact.
- **Sub-slot stride formula `(DIGIT_CELL_X − 1) / (SLOT_GRID − 1)`.** Distributes `SLOT_GRID` sub-slots evenly across `DIGIT_CELL_X` terminal cells with first at offset 0 and last at the cell's far edge. For our defaults: 3 sub-cols at cols 0, 2, 4 of a 5-col cell; 3 sub-rows at rows 0, 1, 2 of a 3-row cell.
- **`N_AGENTS = SLOTS_MAX = 160`.** Sized so every slot of every digit can be filled. Largest digit (5/8) has 17 `#` × 9 = 153 slots; 160 leaves 7 wandering. Smallest digit (7) has 99 slots, leaving 61 wandering — but the digit silhouette stays clear because the 99 occupied slots fill the visible shape.
- **`SEP_BASE_FORCE` is a constant (60), not a strategy param.** Without this, changing `arrive_speed` between strategies (70 → 200) scaled separation by 3× and destroyed force balance.
- **Greedy slot assignment (O(N·S)) not Hungarian (O(N³)).** Globally optimal assignment would look unnatural — agents far from their optimal slot cross paths. Greedy nearest-available produces organic-looking routes.
- **`WANDER_FADE_DIST = 55 px`.** Empirically chosen so wander is near zero when agents reach `AT_SLOT_DIST = 14 px`. Below 14 px the agent is "at slot" and rendered `A_BOLD`.
- **VORTEX fades by `min(dist / VORTEX_FADE_DIST, 1.0)`.** Without fading, the agent orbits its slot forever at small radius instead of landing.
- **PULSE uses centroid→slot direction for push, NOT agent→slot.** When the agent sits exactly on its slot, `agent − slot` is a zero vector (degenerate). `centroid − slot` is always well-defined.
- **Theme system added (10 themes).** Body palette is per-theme; `PAIR_HUD` (yellow) and `PAIR_HINT` (cyan) are theme-independent so the HUD stays readable against any backdrop.

## Worked Example (defaults: 160 agents, 80x24 terminal)

- World box: 80 × 24 cells = 640 × 384 pixels.
- Digit footprint: 5 cols × 5 cells/bitmap-col = **25 cell-cols** wide; 7 rows × 3 cells/bitmap-row = **21 cell-rows** tall. Fills ~31 % of screen width, ~88 % of height — leaves 1 row top + 1 row bottom for HUD bars.
- Slot count per digit: 0/6/9 → 144; 1 → 108; 2/3 → 135; 4 → 126; **5/8 → 153 (max)**; 7 → 99 (min).
- Per tick (1/60 s):
  - RUSH agent at `arrive_speed = 180 px/s` travels 3 px ≈ less than half a column — visibly smooth.
  - DRIFT agent at `arrive_speed = 70 px/s` travels 1.2 px — slow meander.
- Steering cost: separation O(N²): 160·159 = 25 440 checks/tick × 60 Hz ≈ 1.5 M/s. Sub-millisecond at -O2.
- Slot assignment: O(N·S) = 160·153 ≈ 24 500 ops, runs only on digit change (key `0`-`9` or auto-cycle tick).

## Open Questions to Explore

1. What happens with Hungarian (optimal) assignment instead of greedy? Does the digit form faster?
2. Can agents reassign slots dynamically (live rebalancing) as positions change?
3. What if `SLOT_GRID = 4` (16 sub-slots per `#`)? `DIGIT_CELL_W` and `DIGIT_CELL_H` would need to scale to 4 too — whole digit becomes 20 cell-cols × 28 cell-rows, taller than 24-row terminal.
4. SPRING strategy: what happens at critical damping (ζ = 1)? At ζ = 2 (overdamped)?

---

# Pass 2 — Pseudocode & Data Flow

## Module Map
```
§1 config     StrategyParams, g_presets[10], constants, SLOT_GRID,
              N_AGENTS=160, SLOTS_MAX=160, DIGIT_CELL_H=3, PAIR_HUD,
              PAIR_HINT, N_THEMES
§2 clock      clock_ns(), clock_sleep_ns()
§3 color      Theme struct, THEMES[10], theme_apply(), color_init()
§4 vec2       Vec2 + helpers + bounce_pos()
§5 entity     Agent, Slot, agent_spawn(), agent_step()
§6 steering   steer_seek/arrive/separate/wander/cohesion/align/spring()
§7 strategy   strategy_drift/rush/flow/orbit/flock/pulse/vortex/gravity/
              spring/wave() + agent_tick() dispatch
§8 digit      digit_load() — sub-slot subdivision; digit_centroid();
              assign_slots() — greedy nearest-available
§9 scene      SwarmScene, scene_init/scatter/set_digit/tick/draw,
              mark_cell helper
§10 app       screen_*, app_handle_key (incl. t/T theme), main()
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
      scene_tick(dt_sec, cols, rows)
      sim_accum -= tick_ns
    alpha = sim_accum / tick_ns
    fps_counter (500 ms window)
    /* frame cap — `elapsed = clock_ns() - frame_start` (no +dt!) */
    sleep(NS_PER_SEC/TARGET_FPS - elapsed)
    screen_draw(alpha) -> erase, scene_draw, HUD, doupdate
    getch loop -> app_handle_key:
      0-9 -> scene_set_digit
      n/p -> g_strat_idx ± 1 (mod 10), g_sp = &g_presets[idx]
      t/T -> theme_idx ± 1 (mod 10), theme_apply()
      space -> paused
      r -> scene_scatter
      a -> auto_cycle
      q -> exit
```

## agent_tick dispatch
```
switch(strategy):
  0 DRIFT:   wander(fades near slot) + arrive · slot_weight + sep
  1 RUSH:    arrive · slot_weight + sep
  2 FLOW:    rightward bias if x far left of slot, else arrive + wander
  3 ORBIT:   tangent(pos-centroid, 90°) · ORBIT_STR + arrive + sep
  4 FLOCK:   cohesion + alignment + sep + arrive + wander(fades)
  5 PULSE:   arrive(oscillating_target) · slot_weight + sep
  6 VORTEX:  tangent(pos-slot, 90°) · VORTEX_STR · fade + arrive + sep
  7 GRAVITY: (0, GRAVITY_PULL) + arrive · slot_weight + sep
  8 SPRING:  k·(slot − pos) − damp·vel + sep
  9 WAVE:    arrive · slot_weight + perp · amp · sin(2π·freq·t) + sep
```

## digit_load with sub-slot subdivision
```
digit_load(slots, digit, cols, rows):
  cell_w_px = DIGIT_CELL_W * CELL_W   /* = 5*8 = 40 */
  cell_h_px = DIGIT_CELL_H * CELL_H   /* = 3*16 = 48 */
  digit_px_w = DIGIT_NCOLS * cell_w_px    /* 200 */
  digit_px_h = DIGIT_NROWS * cell_h_px    /* 336 */
  ox = (pw(cols) - digit_px_w) * 0.5
  oy = (ph(rows) - digit_px_h) * 0.5

  stride_x = (DIGIT_CELL_W - 1) / (SLOT_GRID - 1)   /* = 2 */
  stride_y = (DIGIT_CELL_H - 1) / (SLOT_GRID - 1)   /* = 1 */

  n = 0
  for r in 0..DIGIT_NROWS:
    for c in 0..DIGIT_NCOLS:
      if bitmap[r][c] != '#': continue
      cell_col0 = c * DIGIT_CELL_W
      cell_row0 = r * DIGIT_CELL_H
      for sr in 0..SLOT_GRID:
        for sc in 0..SLOT_GRID:
          sub_col = cell_col0 + sc * stride_x
          sub_row = cell_row0 + sr * stride_y
          slots[n].pos = (ox + sub_col*CELL_W, oy + sub_row*CELL_H)
          n++
  return n
```
