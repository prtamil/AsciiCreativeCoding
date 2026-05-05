# Pass 1 — shepherd.c: Autonomous Border Collie Herds Scattered Sheep Back into a Pen

> **Note**: This file was completely redesigned. The previous version had a *user-controlled* shepherd `#` driven by arrow keys. The current version is **autonomous** — a single border collie (`&`) decides where to stand, and the sheep do the herding by fleeing the dog. The user no longer steers the dog; SPACE triggers a scatter event for the user to challenge the collie.

## From the Source

**Algorithm:** Two interlocking pieces share the screen.

**SHEEP** — each is a steering-behaviour particle (Reynolds 1987 style) with four forces summed each tick: separation from close neighbours, soft cohesion toward the local centroid, **flee from the dog** (only when within `DOG_FLEE_RADIUS = 120 px`), and a gentle inward pull whenever the sheep is outside the pen. Forces are integrated into velocity with a per-tick `SHEEP_DAMPING = 0.95` factor, so motion settles to grazing rest when no force applies (`0.95^60 ≈ 0.046` retained per second — sheep actually *stop and graze*).

**DOG** — a Strömbom-style controller (Strömbom et al. 2014, *"Solving the shepherding problem"*). Each tick the dog picks one of two modes:
1. **COLLECT** — if any sheep is more than `pen_radius + PEN_TOLERANCE` from the pen centre, find the worst outlier and place the dog at:
   ```
   target = sheep_pos + DOG_APPROACH_OFFSET · normalize(sheep_pos − pen_centre)
   ```
   The dog stands directly *outside* the sheep relative to the pen; the sheep, fleeing the dog radially, runs *toward* the pen interior. The goal direction emerges from positioning, not from a goal-aware sheep.
2. **PATROL** — when all sheep are inside the pen (with tolerance), the dog walks a slow circle around the pen at radius `pen_r + DOG_PATROL_OFFSET`. Patrolling rather than parking keeps the dog visible and reactive when a fresh scatter event fires.

The dog has no path planner — it simply steers toward its target position at `DOG_SPEED = 180 px/s`. The emergent herding comes from the two-mode position selection.

**Math:**
- Sheep separation: `strength = (R − d)/R; force += unit(self − nb) · strength · SHEEP_GRAZE_SPEED` for each neighbour with `d < SHEEP_SEP_RADIUS`.
- Sheep flee: `strength = (R − d)/R; force += unit(self − dog) · strength · SHEEP_FLEE_SPEED` if `d < DOG_FLEE_RADIUS`.
- Sheep pen pull: `force += unit(pen_centre − self) · SHEEP_GRAZE_SPEED` if outside pen.
- Dog COLLECT target (the geometric trick): `target = sheep[worst].pos + DOG_APPROACH_OFFSET · unit(sheep − pen)`.
- Dog PATROL target: `target = pen_centre + (pen_r + DOG_PATROL_OFFSET) · (cos φ, sin φ); φ += DOG_PATROL_OMEGA · dt`.

**Performance:** Sheep separation/cohesion are O(N²); dog mode is O(N) (one scan to find worst outlier). At N = 30 sheep this is 30·29 = 870 distance checks/tick — under 0.1 ms on any modern CPU.

**Data-structure:** Scene owns a Pen (centre + radius), a fixed-capacity Sheep pool (only first `n_sheep` slots active; rest pre-spawned for instant `+` reveal), a single Dog, current world dimensions (refreshed each tick from `cols/rows`), and `in_pen_count` (recomputed each tick for the HUD). Sheep carry `pos / prev_pos / vel` + a `fleeing` flag set by the flee force; Dog carries pos + target + the controller's `mode` + the sheep index it is collecting + `patrol_phase`.

## Core Idea

The dog never tells a sheep where to go. The dog picks **WHERE TO STAND**, and a fleeing sheep does the rest. If the dog stands on the far side of a sheep relative to the pen, the sheep — running away from the dog — happens to run toward the pen. Repeat for every outlier and the herd collapses back inward. The whole "intelligence" is one geometric placement rule applied each tick.

## The Mental Model

Picture a real sheepdog trial: the dog runs *wide* around the flock, approaches the strays from the *outside*, and lets the sheep's own panic do the work of pushing them back. A bad dog charges straight at the flock and just scatters them further. A good dog uses geometry — pick a position, hold the line, let the sheep flee toward where you want them.

The algorithm is one flowchart:

```
every tick
    │
    ▼
pick the sheep furthest from pen centre
    │
    ├── if its distance > pen_r + tolerance:
    │       dog target = sheep_pos + offset · (away-from-pen)
    │       state      = COLLECT
    │
    └── else (everyone home):
            dog target = pen_centre + (pen_r + offset) ·
                         (cos(patrol_phase), sin(patrol_phase))
            state      = PATROL
```

That's it. No targets for individual sheep, no goal direction in any sheep's head — just a moving repulsor (the dog) placed where its repulsion happens to point inward.

## Algorithm in Steps (per tick)

1. Read inputs: SPACE → impulse every sheep outward; 'c' → toggle continuous chaos sprinkle.
2. Dog controller: scan sheep, find argmax(distance from pen centre); if worst > `pen_r + PEN_TOLERANCE` set `mode = COLLECT` and target outside the worst sheep; else `mode = PATROL` and orbit.
3. Dog step: `dog.vel = DOG_SPEED · normalize(target − dog.pos); dog.pos += dog.vel · dt`. Hard-clamp to world bounds.
4. Sheep update — two-stage to avoid index-order drift:
   - **Stage A**: for each sheep `i` compute `force = Σ weights · (sep, coh, flee, pen_pull)`.
   - **Stage B**: for each sheep `i`: `vel = (vel + force·dt) · DAMPING`; clamp `|vel|`; `prev_pos = pos`; `pos += vel · dt`; clamp to world.
5. Recompute `in_pen_count` for the HUD.
6. Render: pen ring (dashed `*` and `.`) → sheep (`o` calm / `O` panicking) → dog (`&`) — painter's order.

## Key Formulas

```
Sheep separate (per neighbour with 0 < d < SHEEP_SEP_RADIUS):
  strength = (SHEEP_SEP_RADIUS − d) / SHEEP_SEP_RADIUS
  force   += unit(self − neighbour) · strength · SHEEP_GRAZE_SPEED

Sheep flee dog (only if within DOG_FLEE_RADIUS):
  strength = (DOG_FLEE_RADIUS − d) / DOG_FLEE_RADIUS
  force   += unit(self − dog) · strength · SHEEP_FLEE_SPEED

Sheep pen pull (only if outside pen):
  force   += unit(pen_centre − self) · SHEEP_GRAZE_SPEED

Dog COLLECT target (the geometric trick):
  target  = sheep[worst].pos
          + DOG_APPROACH_OFFSET · unit(sheep[worst].pos − pen_centre)

Dog PATROL target:
  target  = pen_centre + (pen_r + DOG_PATROL_OFFSET) · (cos φ, sin φ)
  φ      += DOG_PATROL_OMEGA · dt
```

## Worked Example (defaults: 30 sheep, 80x24 terminal)

- World box: 80 × 24 cells = 640 × 384 pixels.
- Pen: centre `(320, 192)`, radius `0.18 · 384 ≈ 69 px ≈ 8.6 cols × 4.3 rows`. Sheep spawn uniformly in a disc of radius `0.7 · 69 ≈ 48 px` so they appear inside the visible ring.
- Per tick (1/60 s):
  - calm sheep at `SHEEP_GRAZE_SPEED = 20 px/s` drifts 0.33 px (visibly stationary)
  - fleeing sheep at `SHEEP_FLEE_SPEED = 140 px/s` covers 2.3 px (~one new cell every 3-4 ticks)
  - dog at `DOG_SPEED = 180 px/s` crosses 3 px per tick (faster than sheep flee, 180 > 140 px/s, so the dog can outflank)
- SCATTER: SPACE adds 220 px/s outward radial impulse to every sheep (mid-disc sheep get a random direction).
- Time to herd back from one SPACE press, 30 sheep, default speeds: 5–15 seconds depending on how far the scatter sent the sheep and which order the dog picks the outliers.
- Steering cost: 30·29 = 870 distance checks per tick × 60 Hz ≈ 52 000 checks/sec — microseconds total.

## Data Structures

### Sheep
```
/* kinematic state — updated every tick */
pos / prev_pos / vel   — position (alpha-lerp anchor) and velocity

/* render hint — set each tick by sheep_step's flee force */
fleeing                — true if dog is within DOG_FLEE_RADIUS
```

### Dog
```
/* kinematic state */
pos / prev_pos / vel

/* controller output — where the dog wants to be this tick */
target

/* controller introspection (for HUD) */
mode                   — DOG_PATROL or DOG_COLLECT
target_sheep           — sheep index in COLLECT; -1 in PATROL
patrol_phase           — angle around pen, radians (PATROL only)
```

### Scene
```
pen_centre / pen_radius   — set in scene_init / app_do_resize
sheep[SHEEP_MAX]          — pool; first n_sheep active; rest pre-spawned
n_sheep                   — currently active count (5..80)
dog                       — singleton border collie
paused                    — physics frozen when true
continuous_chaos          — random nudge each tick when true ('c' toggles)
in_pen_count              — HUD-only derived state, recomputed each tick
world_w / world_h         — pixel dimensions, refreshed each tick
```

## The Main Loop

Each iteration:

1. Snapshot `frame_start = clock_ns()`.
2. **Resize check.** SIGWINCH → recompute pen geometry from new size, clamp every sheep + dog into new bounds.
3. **Measure dt.** `CLOCK_MONOTONIC` since previous frame; capped at 100 ms (suspend guard).
4. **Fixed-step accumulator.** Drain `sim_accum` in `tick_ns` chunks; each fires `scene_tick(dt_sec)` (chaos sprinkle if enabled → dog controller + step → two-stage sheep update).
5. **Alpha.** `sim_accum / tick_ns ∈ [0, 1)` — sub-tick interpolation factor for smooth render.
6. **FPS counter.** 500 ms sliding window.
7. **Frame cap.** Sleep `(NS_PER_SEC/60 − elapsed)` where `elapsed = clock_ns() − frame_start`. Critically NOT `+ dt` — adding dt cancels the cap.
8. **Draw + present.** `erase()` → pen ring → sheep at alpha-lerped positions → dog at alpha-lerped position → HUD bars → `wnoutrefresh + doupdate`.
9. **Drain input.** `getch()` until `ERR`.

## Edge Cases to Watch

- **Sheep at pen centre with v ≈ 0** has no "outward" direction for the scatter impulse. `scene_scatter` falls back to a uniform random angle for any sheep whose distance to centre is below 1 px.
- **All sheep already inside pen**: `dog_decide_target` returns PATROL mode; the dog circles the pen at constant speed instead of parking. Parking would make resume from a fresh scatter feel laggy because the dog has to accelerate from rest.
- **Sheep at corner of world**: clamped to `[0, w] × [0, h]` hard. Without the clamp, a strong dog flee could push a sheep off-screen and the renderer would silently drop it.
- **Two-stage update**: writing into the same array we are reading from causes index-order drift. `scene_tick` fills `new_vel[]` fully before writing any back.
- **Dog overshoots target**: `dog_step` uses a simple desired-velocity P-controller (no overshoot guard). At default speeds the dog gets within a few pixels of target each tick; if you crank `DOG_SPEED` much higher the dog can oscillate.
- **Frame cap**: never `elapsed = clock_ns() − frame_time + dt` — adding dt cancels the cap. Use a `frame_start` snapshot.

## How to Verify

- At startup: 30 sheep mill calmly inside the dashed circle; dog walks a slow patrol arc around the outside. HUD: `sheep:30/30  dog:PATROL`.
- Press SPACE: every sheep glyph turns `O` (bold, red), velocities point radially outward, the dog immediately switches to `dog:COLLECT` and runs to the worst outlier. Within 5–15 s the sheep are all back inside; dog returns to PATROL.
- Press `c` (continuous chaos): random small kicks every tick. The dog never gets to PATROL — it cycles between outliers indefinitely.
- Reduce sheep count with `-`: 5 sheep, one SPACE press — the dog should sweep them up one by one in clearly visible order.
- Hold SPACE (auto-repeat): each scatter restarts the chase; the dog re-targets between presses without lag.

## Non-Obvious Decisions

**Why position the dog *outside* the worst sheep (not at its centre)?** The flee force radiates *away* from the dog. If the dog stood ON the sheep, the flee direction would be undefined. If the dog stood between the sheep and the pen, the sheep would flee *outward*. The only correct placement is on the line from pen-through-sheep, *further out from the pen than the sheep* — then the sheep flees inward.

**Why pre-pin the dog at PATROL even when all sheep are in?** Parking the dog at rest would make the response to a fresh scatter laggy: the dog has to accelerate from zero. Patrolling keeps the dog at constant speed, so when the scatter fires the controller just changes the target and the dog is already moving.

**Why `DOG_SPEED > SHEEP_FLEE_SPEED`?** `DOG_SPEED = 180`, `SHEEP_FLEE_SPEED = 140`. The dog must be faster than fleeing sheep so it can *outflank* them — otherwise it would chase from behind forever. With the dog faster, it can drive around to the far side of the worst sheep.

**Why one sheep at a time, not all outliers simultaneously?** The Strömbom paper's original "drive" mode pushes the herd centroid as a unit, which works when the herd is already cohesive. Our scatter event explodes the herd into many isolated agents, so picking the *worst* outlier and collecting it one at a time gives the cleaner narrative — you watch the dog visit each stray in turn.

**Why use `(sheep − pen)` and not `(sheep − dog)` for the target offset direction?** The pen is the *goal*; the sheep is the *obstacle*. The line that matters is pen→sheep (extended outward), not dog→sheep (which would just chase). This is the signature insight of the Strömbom paper: predator placement is goal-relative, not predator-relative.

## References

- Strömbom, Mann, Wilson, Hailes, Morton, Sumpter, & King, "Solving the shepherding problem: heuristics for herding autonomous, interacting agents," *J. R. Soc. Interface* 11 (2014). The collect/drive decomposition used here.
- Reynolds, "Flocks, Herds, and Schools: A Distributed Behavioral Model," SIGGRAPH 1987 — the boid forces that drive the sheep.
- Reynolds, "Steering Behaviors for Autonomous Characters," 1999 — the unified seek/flee form used by `sheep_flee_dog` and `sheep_pen_pull`.
- Wikipedia: "Sheepdog trial" — context for the real-world task being simulated.
