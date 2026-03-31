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

---

# Pass 2 — brust: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS, BURSTS_MIN/DEFAULT/MAX, PARTICLES=48, BURST_TICKS=22, FUSE_MIN/RANGE |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 7 Hue pairs (same as fireworks); `hue_rand()` |
| §4 particle | `Particle` struct (cx/cy center + rx/ry displacement + delay); `particle_spawn()`, `particle_tick()` (friction), `particle_draw()` |
| §5 burst | `Burst` struct + 3-state machine; `burst_ignite()`, `burst_tick()`, `burst_draw()` |
| §6 field | `Field` owns bursts + scorch map; `field_init()`, `field_tick()`, `field_draw()` |
| §7 screen | ncurses init/resize/present/HUD |
| §8 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
field_tick():
  for each active burst:
    burst_tick(burst, cols, rows, scorch_cb, field)
      │
      ├── BS_IDLE: fuse--; if fuse<=0: burst_ignite()
      │
      ├── BS_FLASH: state = BS_LIVE, ticks = 0  (1 tick only)
      │
      └── BS_LIVE:
            for each particle: particle_tick(p, cols, rows)
              if delay > 0: delay--; return
              vx *= 0.82; vy *= 0.82   ← friction
              rx += vx; ry += vy
              life -= decay
              sx = cx + rx * ASPECT; sy = cy + ry
              if life<=0 or out-of-bounds: alive=false
            ticks++
            if no alive OR ticks >= BURST_TICKS:
              scorch_cb(cx, cy, field)   ← writes '.' to scorch[y*cols+x]
              fuse = FUSE_MIN + rand()%FUSE_RANGE
              state = BS_IDLE

field_draw():
  scorch layer (dim orange A_DIM):
    for each cell in scorch[]:
      if scorch[i] != 0: mvwaddch(y, x, '.')

  burst layer:
    for each active burst:
      burst_draw(burst, w, cols, rows)
        │
        ├── BS_FLASH:
        │     mvwaddch(cy, cx, '*') + A_BOLD + C_YELLOW
        │     mvwaddch(cy, cx±1, '+') + mvwaddch(cy±1, cx, '+')
        │
        └── BS_LIVE:
              for each particle:
                x = (int)(cx + rx * ASPECT)
                y = (int)(cy + ry)
                if life > 0.65: A_BOLD
                mvwaddch(y, x, sym) with COLOR_PAIR(hue)
```

---

## Function Breakdown

### burst_ignite(burst, cols, rows)
Purpose: reset burst for a new explosion at a random position
Steps:
1. cx = rand in [2, cols-4]
2. cy = rand in [1, rows-2]
3. ticks = 0, state = BS_FLASH
4. For i in 0..PARTICLES-1:
   a. angle = (i/PARTICLES) * 2π + rand()*0.2 (even spread + jitter)
   b. speed = 1.8 + rand()*2.8
   c. wave = i % 4
   d. delay = (wave * MAX_DELAY) / (waves-1)  [0,1,2,5 for waves 0,1,2,3]
   e. particle_spawn(p, cx, cy, angle, speed, delay)

---

### particle_spawn(p, cx, cy, angle, speed, delay)
Purpose: initialize one particle
Steps:
1. `p.cx = cx, p.cy = cy` (center — fixed for life)
2. `p.rx = p.ry = 0` (displacement starts at zero)
3. `p.vx = cos(angle)*speed, p.vy = sin(angle)*speed`
4. `p.life = 0.8 + rand()*0.2`
5. `p.decay = 0.05 + rand()*0.04`
6. `p.delay = delay_ticks`
7. `p.sym = random from k_syms`
8. `p.hue = hue_rand()`
9. `p.alive = true`

---

### particle_tick(p, cols, rows)
Purpose: advance one particle (friction model, no gravity)
Steps:
1. If not alive: return
2. If delay > 0: delay--; return (still in wave delay)
3. `vx *= 0.82; vy *= 0.82` (exponential friction)
4. `rx += vx; ry += vy` (displacement grows)
5. `life -= decay`
6. Check screen position: `sx = cx + rx*ASPECT, sy = cy + ry`
7. If life <= 0 or sx/sy out of bounds: alive = false

---

### burst_tick(burst, cols, rows, scorch_cb, ud)
Purpose: advance burst state machine one tick
Steps:
- **BS_IDLE**: `fuse--`; if `fuse <= 0`: `burst_ignite()`
- **BS_FLASH**: `state = BS_LIVE`; `ticks = 0`
- **BS_LIVE**:
  1. any_alive = false
  2. For each particle: `particle_tick()`; track if any alive
  3. `ticks++`
  4. If `!any_alive || ticks >= BURST_TICKS`:
     - `scorch_cb(cx, cy, ud)` — marks scorch map
     - `fuse = FUSE_MIN + rand() % FUSE_RANGE`
     - `state = BS_IDLE`

---

### field_scorch_cb(x, y, ud)
Purpose: write `.` to scorch map at burst death position
Steps:
1. Cast `ud` to `Field*`
2. If in bounds: `field->scorch[y*cols+x] = '.'`

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  field_init(cols, rows, BURSTS_DEFAULT)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       field_free()   ← frees scorch malloc
       field_init(new cols, rows, active_bursts)
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       field_tick()
       sim_accum -= tick_ns

  4. FPS counter

  5. frame cap: sleep to 60fps

  6. draw:
     erase()
     field_draw(stdscr)   ← scorch first, then bursts
     HUD
     wnoutrefresh + doupdate

  7. input:
     q/ESC → quit
     ] / [ → sim_fps ± 4
     = / - → active_bursts ± 1 (clamped; stagger new bursts)
     r     → memset scorch to 0
```

---

## Interactions Between Modules

```
App
 └── owns Field (burst pool + scorch map)

Field
 ├── owns bursts[BURSTS_MAX]
 │    └── each Burst owns parts[48]
 └── owns scorch[cols*rows] (heap, malloc in field_init)

burst_tick receives scorch_cb function pointer
  → field_scorch_cb(x, y, field_ptr) writes to field->scorch
  → keeps Burst decoupled from Field

§4 particle — no knowledge of Field or screen
```

---

## Key Design Contrasts vs fireworks.c

| Aspect | fireworks.c | brust.c |
|---|---|---|
| Launch | IDLE → RISING → EXPLODED | IDLE → FLASH → LIVE |
| Rocket | Yes (rising streak) | No (instant explosion) |
| Physics | Gravity, no friction | Friction, no gravity |
| Position tracking | Absolute (x,y) | Center + displacement (cx+rx, cy+ry) |
| Persistence | None | Scorch map accumulates |
| Flash | No separate flash state | 1-tick BS_FLASH with * + + cross |
| Aspect correction | vy * 0.5 at spawn | rx * ASPECT=2.0 at draw |
