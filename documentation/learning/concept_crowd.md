# Pass 1 — crowd.c: Reynolds Steering Crowd Simulator

## Core Idea
N agents (5–150 people) moving under six switchable crowd behaviours. All behaviours share the same Euler-integrated physics: accumulate weighted steering forces, clamp speed, integrate position. Pressing 1–6 switches behaviour live without resetting positions.

## Mental Model
Each person is a steered particle with momentum. The "force" is not a physical force — it's the difference between where you want to go and where you're currently going. The magic of Reynolds steering is that this single formula `force = desired_velocity − current_velocity` produces naturally smooth, momentum-based motion without any explicit interpolation.

## Key Equations

**Seek:** `desired = normalise(target − pos) × speed`, `force = desired − vel`

**Flee:** `desired = normalise(pos − threat) × speed`, `force = desired − vel` (direction reversed)

**Separation:** `for neighbours within SEP_RADIUS: force += (1 − d/SEP_RADIUS) × normalise(away)`
Pushes agents apart; stronger the closer they are.

**Cohesion:** `desired = normalise(local_centroid − pos) × speed`
Pulls toward the average position of nearby agents.

**Alignment:** `force = avg_velocity_of_neighbours − vel`
Steers toward the average velocity of nearby agents.

## Six Behaviours

| Behaviour | Forces |
|-----------|--------|
| WANDER | Seek random wandering target + separation |
| FLOCK | Classic boids: separation + alignment + cohesion |
| PANIC | Flee roaming threat `!` + separation (flee overrides cohesion) |
| GATHER | Seek screen centre + separation |
| FOLLOW | Each agent seeks the next agent in an index chain; index 0 leads |
| QUEUE | Agents seek a right-edge counter in sorted y-order; orderly line forms |

## Non-Obvious Design Decisions
- **PANIC flee overrides cohesion.** When frightened, staying with the group (cohesion) conflicts with running away. Panic gives flee force high weight and cohesion weight = 0. This causes the crowd to scatter, which looks realistic.
- **FOLLOW uses index chain, not nearest-neighbor.** Agent `i` follows agent `i-1`. This produces a snake-like trail rather than everyone chasing the nearest person.
- **Agent count is runtime-adjustable (`+/-` keys).** The pool uses a live `n_people` counter; agents beyond that index are simply not ticked or drawn.

## Open Questions to Explore
1. What happens if FLOCK agents also have a weak goal-seek force? Does the flock move toward a target?
2. In PANIC mode, make the threat `!` seek the densest cluster. Does it create a cat-and-mouse chase?
3. FOLLOW with only 5 agents vs 150 — how does the snake length change the motion?
4. Can you add a LEADER behaviour where one agent has a goal and the rest flock around it?

---

# Pass 2 — Pseudocode & Data Flow

## Module Map
```
§1 config     behaviour constants, speed limits
§2 clock      clock_ns(), clock_sleep_ns()
§3 color      color_init()
§4 vec2       Vec2 helpers, bounce_pos()
§5 entity     Person struct, person_spawn()
§6 steering   seek(), flee(), separate(), cohesion(), align(), wander()
§7 scene      Scene, scene_tick() — behaviour dispatch, scene_draw()
§8 app        screen_*, app_handle_key(), main()
```

## Core Loop
```
main():
  scene_init()   # spawn N agents at random positions
  loop:
    dt = elapsed (capped 100ms)
    scene_tick(dt):
      for each agent i in [0, n_people):
        force = behaviour_force(i)   # switch on current behaviour
        vel = clamp(vel + force*dt, SPEED_MAX)
        pos += vel * dt
        bounce_pos()
    screen_draw()
    getch(): 1-6 behaviour, +/- agent count, r reset, space pause
```

## Behaviour Dispatch
```
switch(behaviour):
  WANDER:  seek(random_target[i]) + separate()
  FLOCK:   separate() + align(neighbours) + cohesion(neighbours)
  PANIC:   flee(threat_pos) * HIGH_WEIGHT + separate()
  GATHER:  seek(screen_centre) + separate()
  FOLLOW:  seek(agents[i-1].pos) + separate()   # agent 0 wanders
  QUEUE:   seek(queue_slot[i]) + separate()     # slots sorted by y
```
