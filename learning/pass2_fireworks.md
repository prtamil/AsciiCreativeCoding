# Pass 2 — fireworks: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS, ROCKETS_MIN/DEFAULT/MAX, PARTICLES_PER_BURST=80, GRAVITY=9.8, LAUNCH_SPEED_MIN/MAX |
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
      │     vy += GRAVITY * dt_sec * 0.5   ← gentle drag
      │     if vy >= 0 or y < 2:
      │       particle_burst(particles, 80, x, y)
      │       state = EXPLODED
      │
      └── EXPLODED:
            for each particle: particle_tick()
              x += vx * dt_sec * 8
              y += vy * dt_sec * 8
              vy += GRAVITY * dt_sec
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
  2. vy += GRAVITY * dt_sec * 0.5 (gentler than particle gravity)
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
