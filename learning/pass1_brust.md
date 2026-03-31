# Pass 1 — brust: Random Burst Field with Scorch Marks

## Core Idea

Random explosion bursts appear at random positions on screen. No rockets — explosions appear directly. Each burst has 48 particles spread in waves (staggered delays), a one-tick bright flash at ignition, and leaves a persistent scorch mark (`.`) where it occurred. The scorch map accumulates across all bursts and persists until cleared.

---

## The Mental Model

Imagine a field of firecrackers that go off at random positions. Each burst:
1. **FLASH**: one tick — draws a bright `*` center + `+` on the four cardinal neighbors
2. **LIVE**: particles fly outward with friction (velocity × 0.82 each tick), fading as life drains
3. When all particles dead: mark the birth position with `.` in the scorch map, wait for a random fuse, then ignite again at a new random position

The scorch layer is a separate flat array (`char scorch[cols*rows]`). It's drawn first (dim orange), then active bursts are drawn on top.

Particles have a `delay` field — the 48 particles are divided into 4 waves, each delayed by a fixed number of ticks. This creates the staggered ripple effect where particles don't all start at tick 0.

---

## Data Structures

### `Particle` struct (brust version)
| Field | Meaning |
|---|---|
| `cx, cy` | Burst center (fixed — never changes for this particle's lifetime) |
| `rx, ry` | Displacement from center (grows each tick) |
| `vx, vy` | Velocity; multiplied by 0.82 each tick (friction) |
| `life` | 0.8–1.0 at birth; decremented by decay |
| `decay` | Random per particle |
| `delay` | Ticks before this particle starts moving (wave stagger) |
| `sym` | Random ASCII symbol |
| `hue` | Random color |
| `alive` | False when dead |

Draw position = `cx + rx * ASPECT`, `cy + ry`. ASPECT=2.0 compensates for non-square cells.

### `Burst` struct
| Field | Meaning |
|---|---|
| `cx, cy` | Explosion center |
| `state` | BS_IDLE / BS_FLASH / BS_LIVE |
| `ticks` | How many LIVE ticks have elapsed |
| `fuse` | Countdown before next ignition |
| `parts[48]` | Particle array |

### `Field` struct
| Field | Meaning |
|---|---|
| `bursts[BURSTS_MAX]` | Pool of all burst slots |
| `scorch[cols*rows]` | Flat char array; `'.'` where a burst died, else 0 |
| `active_bursts` | How many slots are actually ticked |

---

## The Main Loop

Standard fixed-timestep. Field-level operations:
1. `field_tick()` — tick all active bursts
2. `screen_draw_field()` → `erase()` + `field_draw()` → scorch layer first (dim orange), then bursts
3. HUD

---

## Non-Obvious Decisions

### Why cx/cy + rx/ry instead of absolute position?
The center stays fixed; only the displacement changes. This makes it easy to draw relative to the burst origin and to compute the scorch mark (always at cx, cy).

### ASPECT = 2.0
Terminal cells are approximately 2× taller than wide. Multiplying horizontal displacement by 2 makes circular bursts appear circular (not elliptical) on screen.

### Friction per tick: velocity × 0.82
No gravity in brust — particles spread out from center and slow down until they die. The 0.82 multiplier per tick is exponential decay: after N ticks, speed = original_speed × 0.82^N. At 24fps, this gives ~1.3s before particles stop visibly moving.

### Wave delay for staggered bursts
`wave = i % 4`, `delay = (wave * MAX_DELAY) / (waves-1)`. Particles in wave 0 start immediately, wave 3 starts after MAX_DELAY=5 ticks. This creates a ripple — the burst doesn't all appear at once.

### BS_FLASH state is exactly one tick
The flash draws `*` + `+` neighbors but immediately transitions to BS_LIVE on the next tick. This single-frame bright flash gives the impression of ignition.

### Scorch persistence
When a burst finishes (all particles dead OR ticks >= BURST_TICKS), it calls `scorch_cb(cx, cy)` which writes `'.'` to `scorch[y*cols+x]`. Scorch marks accumulate across all bursts and persist until the user presses `r`.

### Callback pattern for scorch
`burst_tick()` takes a `scorch_cb(int x, int y, void *ud)` function pointer. This keeps burst decoupled from the Field — burst doesn't need to know about Field's scorch array.

---

## State Machine

```
IDLE
  │
  └── fuse reaches 0
        ↓
      burst_ignite():
        pick random position
        spawn 48 particles in 4 waves
        state = BS_FLASH

BS_FLASH (exactly 1 tick)
  │
  └── always transitions
        ↓
      state = BS_LIVE, ticks = 0

BS_LIVE
  │
  ├── tick all particles
  └── when (no alive particles OR ticks >= BURST_TICKS):
        scorch_cb(cx, cy)
        fuse = FUSE_MIN + rand() % FUSE_RANGE
        state = BS_IDLE
```

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|---|---|---|
| `PARTICLES` | 48 | Particles per burst |
| `BURST_TICKS` | 22 | Max live duration; burst ends early if all particles die |
| `FUSE_MIN` | 8 | Minimum ticks between bursts |
| `FUSE_RANGE` | 20 | Random extra wait |
| `BURSTS_DEFAULT` | 5 | Simultaneous bursts |
| `ASPECT` | 2.0 | Horizontal stretch; 2.0 = square on ~2:1 aspect terminal |
| friction | 0.82/tick | Lower = particles stop sooner |

---

## Open Questions for Pass 3

1. What happens if you remove the `delay` field and all particles start at tick 0?
2. Try `ASPECT = 1.0` — what happens to the burst shape?
3. Can you add gravity to brust so particles arc downward?
4. The scorch callback pattern — when would you want this vs directly accessing the Field struct?
5. What determines how wide the burst spreads? (speed + ticks + friction + ASPECT all contribute)
