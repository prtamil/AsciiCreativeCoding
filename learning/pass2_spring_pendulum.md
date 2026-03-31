# Pass 2 — spring_pendulum: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | Constants: SIM_FPS, CELL_W/H, GRAVITY_PX, SPRING_K, DAMPING, N_COILS, COIL_SPREAD |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` — monotonic nanosecond timer |
| §3 color | 5 pairs: bar (white), wire (grey), spring (yellow), ball (white), HUD (cyan) |
| §4 coords | `pw/ph`, `px_to_cell_x/y` — single conversion point for aspect ratio |
| §5 pendulum | `Pendulum` struct; `pendulum_init()`; `pendulum_tick()` — symplectic Euler |
| §6 scene | `Scene` owns pendulum; `scene_tick()`; `scene_draw(alpha)` — coil + bob rendering |
| §7 screen | ncurses init/draw/present/resize |
| §8 app | Signal handlers, main dt loop, key handling |

---

## Data Flow Diagram

```
CLOCK_MONOTONIC
    │
    ▼
clock_ns() → dt (ns)
    │
    ▼
sim_accum += dt
    │
    while sim_accum >= tick_ns:
    │
    ▼
pendulum_tick(dt_sec)  [symplectic Euler]
    │
    ├── save prev_r, prev_theta
    ├── compute r̈ = r·θ̇² + g·cosθ − k·(r−r₀) − d·ṙ
    ├── compute θ̈ = −[g·sinθ + 2·ṙ·θ̇]/r − d·θ̇
    ├── r_dot  += r_ddot * dt   ← velocities first
    ├── th_dot += th_ddot * dt
    ├── r      += r_dot * dt    ← positions use new velocities
    └── theta  += th_dot * dt

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns
    │
    ▼
scene_draw(alpha)
    │
    ├── draw_r     = lerp(prev_r, r, alpha)
    ├── draw_theta = lerp(prev_theta, theta, alpha)
    │
    ├── bob pixel = pivot + (draw_r·sinθ, draw_r·cosθ)
    │
    ├── spring axis unit vector: (sinθ, cosθ)
    ├── perp unit vector:        (−cosθ, sinθ)
    │
    ├── N_COILS*2 node positions:
    │     for i in 0..N_NODES-1:
    │       t = (i+1) / (N_NODES+1)
    │       base = pivot + t * draw_r * axis
    │       sign = +1 if even, −1 if odd
    │       node[i] = base + sign * COIL_SPREAD * perp
    │
    └── draw: bar → wire stub → coil lines → coil nodes → bob
             (each overwrites previous — painter's order)
    │
    ▼
doupdate()  ← atomic write to terminal
```

---

## Function Breakdown

### pendulum_init(pend, cols, rows)
Purpose: set initial conditions for spring-pendulum
Steps:
1. pivot at (cols/2 * CELL_W, 1 * CELL_H) — center-top in pixel space
2. `r0 = ph(rows) * REST_LEN_FRAC` — natural length is 40% of screen height
3. `r = r0 * INIT_R_STRETCH` — start slightly stretched (15% above rest)
4. `theta = 40° in radians` — start at 40° from vertical
5. `r_dot = th_dot = 0` — starting from rest
6. `damping = DAMPING_DEF`
7. Copy r and theta into prev_r, prev_theta (no lerp artifact on first frame)

---

### pendulum_tick(pend, dt)
Purpose: advance physics one step using symplectic Euler
Steps:
1. Save `prev_r = r`, `prev_theta = theta`
2. Read current state: r, theta, r_dot (ṙ), th_dot (θ̇), damping (d)
3. Compute accelerations:
   ```
   r_ddot  = r * th_dot² + GRAVITY_PX * cos(theta)
             - SPRING_K * (r - r0) - d * r_dot
   th_ddot = -(GRAVITY_PX * sin(theta) + 2 * r_dot * th_dot) / r
             - d * th_dot
   ```
4. Update velocities FIRST (symplectic):
   ```
   r_dot  += r_ddot * dt
   th_dot += th_ddot * dt
   ```
5. Update positions using NEW velocities:
   ```
   r     += r_dot * dt
   theta += th_dot * dt
   ```
6. Clamp r to [r0*0.05, r0*3.5] — prevent numerical collapse or explosion
Edge cases: if r → 0, th_ddot has division by r → blow up; clamp prevents this

---

### draw_line(x0, y0, x1, y1, cols, rows, attr)
Purpose: Bresenham line with angle-appropriate characters
Steps:
1. Compute dx, dy, step directions sx, sy
2. Standard Bresenham error accumulation loop
3. At each step, choose character based on which direction was taken:
   - Both x and y advance: `\` if sx==sy, `/` otherwise (diagonal)
   - Only x advances: `-` (horizontal)
   - Only y advances: `|` (vertical)
4. `attron(attr)` → `mvaddch(y0, x0, ch)` → `attroff(attr)`
Edge cases: bounds check before every mvaddch

---

### scene_draw(scene, alpha)
Purpose: render complete frame with render interpolation
Steps:
1. Compute draw state:
   ```
   draw_r     = prev_r + (r - prev_r) * alpha
   draw_theta = prev_theta + (theta - prev_theta) * alpha
   ```
2. Compute pivot cell: `px_to_cell_x/y(pivot_px/py)`
3. Compute bob pixel: `(pivot_px + draw_r*sin(draw_theta), pivot_py + draw_r*cos(draw_theta))`
4. Compute spring axis and perp unit vectors from draw_theta
5. Compute all N_COILS*2 node pixel positions (evenly spaced, alternating sides)
6. Draw in order:
   a. Top bar: row 0, all columns = `=`, pivot column = `v` (CP_BAR, A_BOLD)
   b. Wire stub: pivot cell → first node (CP_WIRE, draw_line)
   c. Coil connecting lines: node[i] → node[i+1] for all i (CP_SPRING, draw_line)
   d. Wire stub: last node → bob cell (CP_WIRE, draw_line)
   e. Coil nodes: `*` at each node cell (CP_SPRING, A_BOLD)
   f. Bob: `(@)` three characters at bob cell (CP_BALL, A_BOLD)

---

### screen_draw(screen, scene, fps, alpha)
Purpose: build full frame in ncurses back buffer
Steps:
1. `erase()` — blank newscr
2. `scene_draw(scene, alpha)` — spring + bob
3. Build HUD: `"fps  θ:±deg  Δr:±pct  d:value"`
4. Place HUD right-aligned on row 0 (overlaps the bar)

---

## Pseudocode — Core Loop

```
setup:
  atexit(cleanup) → endwin()
  signal(SIGINT/SIGTERM) → running = 0
  signal(SIGWINCH) → need_resize = 1
  screen_init() — initscr, noecho, cbreak, curs_set(0), nodelay, keypad, typeahead(-1), color_init
  scene_init(cols, rows) — pendulum_init inside
  frame_time = clock_ns()
  sim_accum = 0
  sim_fps = SIM_FPS_DEFAULT (120)

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize() — endwin+refresh+getmaxyx
       scene_init(new cols, rows) — recompute r0 for new terminal size
       reset frame_time, sim_accum
       need_resize = 0

  2. dt:
     now = clock_ns()
     dt = now − frame_time
     frame_time = now
     cap dt at 100ms

  3. fixed-step physics:
     tick_ns = 1_000_000_000 / sim_fps
     dt_sec  = tick_ns / 1_000_000_000.0
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec)    ← calls pendulum_tick if not paused
       sim_accum -= tick_ns

  4. render alpha:
     alpha = sim_accum / tick_ns

  5. FPS counter (update every 500ms wall time)

  6. frame cap:
     sleep(1_000_000_000/60 − elapsed)

  7. draw + present:
     screen_draw(fps, alpha)
     screen_present() → wnoutrefresh + doupdate

  8. input:
     ch = getch()
     q/ESC → quit
     space → toggle paused
     r     → pendulum_init (reset)
     ]     → damping -= 0.02 (less friction)
     [     → damping += 0.02 (more friction)
```

---

## Interactions Between Modules

```
App
 ├── owns Scene (Pendulum + paused flag)
 ├── owns Screen (cols, rows)
 └── main loop: clock → dt → scene_tick → alpha → screen_draw

Scene
 └── owns Pendulum
      ├── pendulum_tick(): equations of motion + symplectic Euler
      └── scene_draw(): lerp prev→cur, coil layout, draw_line calls

§4 coords
 └── px_to_cell_x/y called by scene_draw only

Signal handlers
 ├── running (SIGINT/SIGTERM)
 └── need_resize (SIGWINCH) — both volatile sig_atomic_t
```

---

## The Symplectic Euler Rule

Always remember: **velocities before positions**.

```
WRONG (regular Euler — energy drifts upward):
  r += r_dot * dt        ← position uses OLD velocity
  r_dot += r_ddot * dt

CORRECT (symplectic Euler — energy conserved):
  r_dot += r_ddot * dt   ← velocity updated first
  r += r_dot * dt        ← position uses NEW velocity
```

This one swap is the difference between a simulation that runs forever and one that explodes.
