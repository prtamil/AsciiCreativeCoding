# Pass 1 — fireworks: Rocket Fireworks with State Machine

## Core Idea

Each rocket is an independent state machine that cycles through three states: waiting to launch (IDLE), rising toward its apex (RISING), then exploding into a burst of particles (EXPLODED). Multiple rockets run in parallel. The Show struct owns a pool of rocket slots; each rocket owns its own particle array.

---

## The Mental Model

Imagine a pool of `N` rockets. Each rocket:
1. **IDLE**: waits for a fuse countdown. Staggered initial fuses prevent all rockets from launching simultaneously.
2. **RISING**: moves upward each tick, with gravity slowing it. When vertical velocity crosses zero (apex reached), it explodes — spawning 80 particles in all directions.
3. **EXPLODED**: waits for all particles to die. Each particle is drawn with brightness based on its remaining life (bold = fresh, dim = fading). When all particles dead, sets a new random fuse and returns to IDLE.

The rocket is drawn as `|` with an `'` exhaust trail below it. Particles are drawn as random ASCII symbols (`*+.,` etc.), each with a random color.

---

## Data Structures

### `Particle` struct
| Field | Meaning |
|---|---|
| `x, y` | Current position in terminal columns/rows |
| `vx, vy` | Velocity; vy increased by GRAVITY each tick |
| `life` | 0.6–1.0 at birth, decremented by decay each tick |
| `decay` | How fast life drains (random per particle) |
| `symbol` | Random ASCII character from k_particle_symbols |
| `color` | Random ColorID (each particle has its own color) |
| `active` | False when life <= 0 |

### `Rocket` struct
| Field | Meaning |
|---|---|
| `x, y` | Position in terminal coordinates |
| `vy` | Vertical velocity (negative = upward) |
| `color` | Color for the rocket body |
| `state` | RS_IDLE / RS_RISING / RS_EXPLODED |
| `fuse` | Countdown ticks until next launch from IDLE |
| `particles[80]` | Fixed array — no heap allocation |

### `Show` struct
| Field | Meaning |
|---|---|
| `rockets[MAX_ROCKETS]` | Fixed pool of all rocket slots |
| `active_rockets` | How many slots are actually ticked |

---

## The Main Loop

Standard fixed-timestep:
1. `dt` → `sim_accum`
2. While `sim_accum >= tick_ns`: `show_tick()` (ticks each active rocket)
3. Render: `erase()` → `show_draw()` → HUD → `doupdate()`
4. Input: add/remove rockets, speed, quit

---

## Non-Obvious Decisions

### Staggered initial fuses
At init, rocket `i` gets `fuse = i * 8`. Without this, all rockets would launch at tick 0 — one big simultaneous blast instead of staggered shows.

### vy >= 0 as apex detection
Gravity adds to vy each tick. The rocket starts with negative vy (upward). When gravity has accumulated enough, vy crosses zero — this is exactly the apex. No need to track max height.

### Vertical velocity squash: `vy = sinf(angle) * speed * 0.5f`
The particle spread is a 2D circle in physics, but the terminal is non-square (cells are taller than wide). Squashing vy by 0.5 makes the burst look circular on screen.

### Split gravity: ROCKET_DRAG vs GRAVITY
Originally a single `GRAVITY = 9.8f` constant controlled both rocket deceleration (how high the rocket climbs) and particle fall after explosion. The problem: lowering GRAVITY to give particles more upward spread also made rockets decelerate too slowly, causing every rocket to reach the very top of the screen before exploding — all bursts appeared in one narrow strip at the top.

The fix is two separate constants:
- `ROCKET_DRAG = 9.8f` — applied as `r->vy += ROCKET_DRAG * dt_sec * 0.5` in rocket_tick. Controls apex height independently of particle physics.
- `GRAVITY = 4.0f` — applied to each particle after explosion. Lower value gives upward sparks enough hang time to visibly rise, producing a near-spherical burst rather than a downward triangle.

This decoupling is a general principle: simulation layers that share a physical concept (gravity) may still need independent tuning constants because they operate at different scales and timescales.

### Multi-color bursts
Every particle gets an independent random color. This is different from having one color per rocket — the multi-color explosion looks richer.

### Brightness from life
`life > 0.6f → A_BOLD` (freshly born, glowing bright)
`life < 0.2f → A_DIM` (dying embers, fading out)
Middle range: normal brightness

### No heap allocation
Each rocket owns `particles[PARTICLES_PER_BURST]` directly. `Show` owns `rockets[MAX_ROCKETS]` directly. Everything is in the App struct on the (conceptual) stack. No malloc/free in the hot path.

---

## State Machine

```
IDLE ──(fuse reaches 0)──→ RISING
         ↑                     │
         │                     │ vy crosses 0 (apex) or y < 2
         │                     ↓
         └──(all particles dead) EXPLODED
                (set new random fuse)
```

---

## From the Source

**Algorithm:** Apex detection uses `vy >= 0.0f` (vertical velocity crossing zero). Rocket ascent: `r->vy += ROCKET_DRAG * dt_sec * 0.5` — the 0.5 factor halves the drag so the rocket reaches higher before apex. `ROCKET_DRAG = 9.8` and `GRAVITY = 4.0` are separate constants: ROCKET_DRAG controls apex height; GRAVITY controls how fast sparks fall after explosion.

**Physics:** Spark speeds are Gaussian-distributed (not uniform). Integration uses explicit Euler: `vel.y += GRAVITY × dt`, `pos += vel × dt` each tick. This is the first-order forward-Euler method — simple and sufficient for short-lived particles. Vertical velocity squashed: `vy = sin(angle)*speed*0.5` to compensate for non-square terminal cells.

**Data-structure:** `particles[PARTICLES_PER_BURST]` is a fixed array embedded directly in each `Rocket` struct. `Show` owns `rockets[MAX_ROCKETS]` as a flat array. Both are in the global `g_app` in BSS — no heap allocation in the simulation hot path. Inactive rocket slots beyond `active_rockets` are parked with `fuse = INT32_MAX/2`.

**Rendering:** Brightness from `life`: `> 0.6 → A_BOLD`, `< 0.2 → A_DIM`, middle → normal. Rocket body: `|` with A_BOLD; exhaust trail: `'` one row below. Staggered initial fuses: rocket `i` gets `fuse = i × 8` ticks so launches spread out from the start.

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `PARTICLES_PER_BURST` | 80 | More sparks per explosion; heavier CPU |
| `ROCKETS_DEFAULT` | 6 | More simultaneous rockets |
| `LAUNCH_SPEED_MIN/MAX` | 3–8 rows/s | Higher = rockets reach greater heights |
| `ROCKET_DRAG` | 9.8 rows/s² | Higher = quicker apex, rockets explode lower on screen |
| `GRAVITY` | 4.0 rows/s² | Higher = particles fall faster, downward triangle shape; lower = near-spherical burst |
| fuse range | 0.5–2.5s | Longer wait between launches |

---

## Open Questions for Pass 3

1. What happens if you set `GRAVITY = 0`? Particles would fly in straight lines.
2. Can you make all particles in one explosion share one color (matching the rocket) but vary in brightness?
3. What would the stagger look like with `fuse = rand() % 30` instead of `i * 8`?
4. How would you add a rocket that doesn't explode but instead transforms into a falling star?
5. The physics uses terminal rows/cols directly — no pixel space. How does this compare to bounce_ball's approach?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `rockets[MAX_ROCKETS]` | `Rocket[20]` | ~48 KB | object pool of ascending rockets |
| `rockets[i].particles[PARTICLES_PER_BURST]` | `Particle[80]` | 2.4 KB each | per-rocket spark pool for one explosion |

# Pass 2 — fireworks: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS, ROCKETS_MIN/DEFAULT/MAX, PARTICLES_PER_BURST=80, ROCKET_DRAG=9.8, GRAVITY=4.0, LAUNCH_SPEED_MIN/MAX |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 7 ColorID enum pairs (RED..MAGENTA); `color_rand()` |
| §4 particle | `Particle` struct; `particle_burst()` — spawn radial burst; `particle_tick()` — physics; `particle_draw()` — life→brightness |
| §5 rocket | `Rocket` struct + state machine; `rocket_launch()`, `rocket_tick()`, `rocket_draw()` |
| §6 show | `Show` owns rocket pool; `show_init()`, `show_tick()`, `show_draw()` |
| §7 screen | ncurses init/resize/present/HUD |
| §8 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
show_tick():
  for each active rocket:
    rocket_tick(rocket, dt_sec, cols, rows)
      │
      ├── IDLE:
      │     fuse--
      │     if fuse <= 0: rocket_launch()
      │
      ├── RISING:
      │     y += vy * dt_sec * 6.0
      │     vy += ROCKET_DRAG * dt_sec * 0.5   ← rocket deceleration
      │     if vy >= 0 or y < 2:
      │       particle_burst(particles, 80, x, y)
      │       state = EXPLODED
      │
      └── EXPLODED:
            for each particle: particle_tick()
              x += vx * dt_sec * 8
              y += vy * dt_sec * 8
              vy += GRAVITY * dt_sec   ← particle gravity (4.0, not ROCKET_DRAG)
              life -= decay
              if life <= 0: active = false
            if no particles alive:
              fuse = 0.5s..2.5s in ticks
              state = IDLE

show_draw():
  erase()
  for each active rocket:
    rocket_draw(rocket, stdscr, cols, rows)
      │
      ├── RISING: draw '|' at (x,y), '\'' at (x,y+1)
      └── EXPLODED:
            for each particle:
              particle_draw()
                x = (int)px, y = (int)py
                if life > 0.6: A_BOLD
                if life < 0.2: A_DIM
                mvaddch(y, x, symbol) with COLOR_PAIR(color)
  HUD
  wnoutrefresh + doupdate
```

---

## Function Breakdown

### particle_burst(particles, count, x, y)
Purpose: spawn `count` particles at position (x, y) in a radial burst
Steps:
1. For i in 0..count-1:
   a. `angle = (i / count) * 2π + rand()*0.3` — evenly spread + small jitter
   b. `speed = 1.5 + rand()*3.5` — varied radii
   c. `vx = cos(angle) * speed`
   d. `vy = sin(angle) * speed * 0.5` ← squash vertical for non-square cells
   e. `life = 0.6 + rand()*0.4` — varied lifetimes
   f. `decay = 0.03 + rand()*0.04` — varied fade rates
   g. `symbol = random from k_particle_symbols`
   h. `color = color_rand()` — each particle its own random color
   i. `active = true`

---

### particle_tick(particle, dt_sec)
Purpose: advance one particle one step
Steps:
1. If not active: return
2. `x += vx * dt_sec * 8.0`
3. `y += vy * dt_sec * 8.0`
4. `vy += GRAVITY * dt_sec` (gravity pulls down)
5. `life -= decay`
6. If life <= 0: active = false
Edge cases: no bounds check here — particle_draw handles that

---

### particle_draw(particle, window, cols, rows)
Purpose: draw particle with life-based brightness
Steps:
1. If not active: return
2. `x = (int)px`, `y = (int)py`
3. If out of bounds: return
4. `attr = COLOR_PAIR(color)`
5. If life > 0.6: attr |= A_BOLD
6. If life < 0.2: attr |= A_DIM
7. `mvwaddch(w, y, x, symbol)`

---

### rocket_launch(rocket, cols, rows)
Purpose: reset rocket for a new flight from the bottom
Steps:
1. x = rand() % cols
2. y = rows - 1 (bottom)
3. vy = -(LAUNCH_SPEED_MIN + rand() % range) — upward velocity
4. color = color_rand()
5. state = RS_RISING
6. Deactivate all particles

---

### rocket_tick(rocket, dt_sec, cols, rows)
Purpose: advance rocket one step through its state machine
Steps:
- **RS_IDLE**: fuse--; if fuse <= 0: rocket_launch()
- **RS_RISING**:
  1. y += vy * dt_sec * 6.0
  2. vy += ROCKET_DRAG * dt_sec * 0.5 (separate from particle GRAVITY)
  3. If vy >= 0 or y < 2:
     - particle_burst(particles, 80, x, y)
     - state = RS_EXPLODED
- **RS_EXPLODED**:
  1. Tick all particles
  2. If no particles active:
     - fuse = (0.5s..2.5s) in ticks
     - state = RS_IDLE

---

### show_init(show, cols, rows, rocket_count)
Purpose: initialize rocket pool with staggered fuses
Steps:
1. active_rockets = rocket_count
2. For i in 0..MAX_ROCKETS-1:
   - If i < rocket_count:
     - rocket_launch() then set state=IDLE and fuse = i*8 (stagger)
   - Else:
     - state = IDLE, fuse = INT32_MAX/2 (parked)
     - all particles inactive

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  show_init(cols, rows, ROCKETS_DEFAULT)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       show_init(new cols, rows, active_rockets)  ← rebuild with new bounds
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       show_tick(dt_sec, cols, rows)
       sim_accum -= tick_ns

  4. FPS counter (every 500ms)

  5. frame cap: sleep to 60fps

  6. draw:
     erase()
     show_draw(stdscr)   ← all rockets + particles
     HUD: mvprintw (fps, sim_fps, active_rockets)
     wnoutrefresh + doupdate

  7. input:
     q/ESC  → quit
     ] / [  → sim_fps ± SIM_FPS_STEP
     = / -  → active_rockets ± 1 (clamped to MIN/MAX)
```

---

## Interactions Between Modules

```
App
 └── owns Show (rocket pool)

Show
 └── owns rockets[MAX_ROCKETS]
      └── each Rocket owns particles[PARTICLES_PER_BURST]

rocket_tick → particle_burst (at explosion)
rocket_tick → particle_tick × 80 (each EXPLODED tick)
rocket_draw → particle_draw × 80 (each EXPLODED frame)

§3 color
 └── color_rand() called by particle_burst (once per particle)
     and rocket_launch (once per rocket)
```

---

## State Machine Summary

```
State     What happens each tick        Transition condition
──────────────────────────────────────────────────────────────
IDLE      fuse--                        fuse <= 0 → RISING
RISING    move up + gravity             vy >= 0 or y < 2 → EXPLODED
EXPLODED  tick all 80 particles         all particles dead → IDLE
                                        (set new random fuse first)
```
