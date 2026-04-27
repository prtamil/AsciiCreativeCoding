# Pass 1 — swarm_gen_numbers.c: Reynolds Steering Swarm Forming ASCII Digits

## Core Idea
25 ASCII agents use Reynolds steering behaviours to coordinate into the pixel shapes of digits 0–9. The digit is a 5×7 bitmap; each `#` cell becomes a target Slot in pixel space. Each agent is greedily assigned its nearest unoccupied slot and steered toward it using one of 10 switchable strategies.

## Mental Model
Think of it as a hire/fire system in continuous space. Slots are job sites; agents are workers. Each worker moves toward their assigned job using a strategy (jog, sprint, spiral, bounce…). The `arrive` steering behaviour gives a smooth deceleration so workers don't oscillate around the job site. Separation force prevents workers from stacking on each other.

## Key Equations

**Seek:** `desired = normalise(target − pos) × speed`, `force = desired − vel`
(Subtracting vel means force→0 when already moving at full speed toward target — smooth deceleration for free)

**Arrive (decelerate near goal):** `desired_speed = max_speed × min(1, dist / slow_radius)`
Prevents the overshoot oscillation that raw seek produces.

**Separation:** `force += (sep_radius − d) / sep_radius × SEP_BASE_FORCE × normalise(away)`
Fixed base force (60 px/s) — NOT proportional to arrive_speed. Tying it to arrive_speed caused RUSH strategy (arrive=180) to produce separation forces strong enough to overpower slot attraction entirely.

**Spring (SPRING strategy):** `F = k × (slot − pos) − damping × vel`
With k=3.5, damping=2.0: ζ = damping/(2√k) ≈ 0.53 → underdamped → agent oscillates 1-2 cycles before settling.

**Wander fade:** `wander_strength *= min(dist_to_slot / FADE_DIST, 1.0)`
Without fading, wander keeps kicking agents out of their slots after arrival.

## Data Structures
```
Agent { pos, prev_pos, vel, wander_angle, slot_idx, glyph, color_pair }
Slot  { pos, occupied }
StrategyParams { name, max_speed, arrive_speed, slow_radius, slot_weight,
                 wander_strength, sep_radius, sep_weight,
                 cohesion_weight, align_weight, neighbor_radius }
```

## Non-Obvious Design Decisions
- **SEP_BASE_FORCE is a constant (60), not a strategy param.** Without this, changing `arrive_speed` between strategies (70→200) scaled separation by 3× and destroyed force balance.
- **Greedy slot assignment (O(N×S)) not Hungarian algorithm (O(N³)).** Globally optimal assignment would look unnatural — agents far from their optimal slot cross paths with each other. Greedy nearest-available produces organic-looking routes.
- **WANDER_FADE_DIST = 55 px.** Empirically chosen so wander is already near zero when agents reach AT_SLOT_DIST (14 px). Below 14 px the agent is "at slot" and rendered bold.
- **VORTEX fades by min(dist/VORTEX_FADE_DIST, 1.0).** Without fading, the agent orbits its slot forever at a small radius instead of landing.
- **PULSE uses centroid→slot direction for push, NOT agent→slot.** When the agent sits exactly on its slot, agent→slot is a zero vector (degenerate). Centroid→slot is always well-defined.

## Open Questions to Explore
1. What happens if you use Hungarian (optimal) assignment instead of greedy? Does the digit form faster?
2. Can you make agents reassign slots dynamically (live rebalancing) as positions change?
3. What would happen with more agents than slots? Try N_AGENTS=50 with digit 7 (11 slots).
4. SPRING strategy: what happens at critical damping (ζ=1)? At ζ=2 (overdamped)?

---

# Pass 2 — Pseudocode & Data Flow

## Module Map
```
§1 config     StrategyParams, g_presets[10], constants
§2 clock      clock_ns(), clock_sleep_ns()
§3 color      color_init() — 7 color pairs
§4 vec2       Vec2, v2(), v2add(), v2sub(), v2scale(), v2len(), v2norm(), bounce_pos()
§5 entity     Agent, Slot, agent_spawn(), agent_step()
§6 steering   steer_seek(), steer_arrive(), steer_separate(), steer_wander(),
              steer_cohesion(), steer_align(), steer_spring()
§7 strategy   strategy_drift/rush/flow/orbit/flock/pulse/vortex/gravity/spring/wave()
              agent_tick() — dispatch switch
§8 digit      digit_load(), digit_centroid(), assign_slots()
§9 scene      SwarmScene, scene_init(), scene_scatter(), scene_set_digit(), scene_tick(), scene_draw()
§10 app       screen_*, app_handle_key(), main()
```

## Core Loop
```
main():
  scene_init()          # spawn agents at random positions, load digit 0, assign slots
  loop:
    dt = wall_clock_elapsed (capped 100ms)
    sim_accum += dt
    while sim_accum >= tick_ns:
      scene_tick(dt_sec)   # one physics step for all agents
      sim_accum -= tick_ns
    alpha = sim_accum / tick_ns   # sub-tick interpolation factor
    screen_draw(alpha)            # lerp draw_pos = prev + (pos-prev)*alpha
    getch()                       # handle keys: n/p strategy, 0-9 digit, r scatter
```

## agent_tick dispatch
```
switch(strategy):
  0 DRIFT:   wander(fades near slot) + arrive * slot_weight + sep
  1 RUSH:    arrive * slot_weight + sep
  2 FLOW:    rightward bias if x far left of slot, else arrive + wander
  3 ORBIT:   tangent(pos-centroid, 90°) * ORBIT_STRENGTH + arrive * slot_weight + sep
  4 FLOCK:   cohesion + alignment + sep + arrive * slot_weight + wander(fades)
  5 PULSE:   arrive(oscillating_target) * slot_weight + sep
  6 VORTEX:  tangent(pos-slot, 90°) * VORTEX_STR * fade + arrive * slot_weight + sep
  7 GRAVITY: (0, GRAVITY_PULL) + arrive * slot_weight + sep
  8 SPRING:  k*(slot-pos) - damp*vel + sep
  9 WAVE:    arrive * slot_weight + perp * amp * sin(2π*freq*t) + sep
```
