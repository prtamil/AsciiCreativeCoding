# Pass 1 — spring_pendulum: Lagrangian Spring-Pendulum Simulation

## Core Idea

A mass hangs from a fixed pivot by a spring that can both rotate (pendulum) and stretch/compress (spring). When the spring frequency is exactly 2× the pendulum frequency, the two modes of motion exchange energy — the bob traces complex rosette paths and the motion looks organic and chaotic. This is the **2:1 resonance** condition.

---

## The Mental Model

Imagine holding a spring with a weight at the end. You can swing it like a pendulum (rotation), or the spring itself can stretch and compress (radial oscillation). Normally these two motions happen independently. But if you tune the spring stiffness so the spring bounces exactly twice as fast as it swings, energy flows back and forth between the two modes — the bob traces big looping rosette shapes, then tightens to a nearly vertical bounce, then spreads out again.

The program simulates this in polar coordinates (angle + length) rather than (x, y) because the physics equations are cleaner in polar form. It then converts to pixel coords to draw a:
- Top bar with a pivot marker
- Spring coil (zigzag computed from the pivot-to-bob axis)
- Iron bob `(@)` at the end

---

## Data Structures

### `Pendulum` struct
| Field | Meaning |
|---|---|
| `r` | Current spring length in pixels |
| `theta` | Current angle from vertical (radians, positive = right) |
| `r_dot` | Radial velocity (how fast r is changing) |
| `th_dot` | Angular velocity (how fast theta is changing) |
| `prev_r` | Spring length at start of previous tick (for lerp) |
| `prev_theta` | Angle at start of previous tick (for lerp) |
| `pivot_px/py` | Fixed anchor point in pixel space (top center) |
| `r0` | Natural rest length of spring (40% of pixel height) |
| `damping` | Friction coefficient — reduces velocities over time |

### `Scene` struct
Owns one `Pendulum` plus terminal dimensions and paused flag.

### `Screen` struct
Terminal columns and rows — just dimensions, no ncurses state.

---

## The Main Loop

1. Check for resize signal → reinit scene to new dimensions
2. Compute `dt` (wall time since last frame, capped at 100ms)
3. Add `dt` to `sim_accum`
4. While `sim_accum >= tick_ns`: run `pendulum_tick(dt_sec)`, subtract `tick_ns`
5. Compute `alpha = sim_accum / tick_ns`
6. Update FPS display every 500ms
7. Sleep to cap render at 60fps
8. `screen_draw(alpha)` → `screen_present()`
9. Handle keyboard input

---

## Non-Obvious Decisions

### Why polar coordinates?
The spring pendulum equations of motion look simple in polar (r, θ):
```
r̈  =  r·θ̇²  +  g·cos θ  −  k·(r − r₀)  −  d·ṙ
θ̈  =  −[g·sin θ  +  2·ṙ·θ̇] / r  −  d·θ̇
```
The `2·ṙ·θ̇` term is the Coriolis effect (energy exchange between modes). In Cartesian (x, y) these equations become coupled in a messier way.

### Why symplectic Euler?
Regular Euler integration adds energy over time — the spring grows larger each step. Symplectic Euler updates velocities first, then positions using the new velocities. This preserves the symplectic structure of Hamiltonian systems — energy stays bounded long-term. No energy drift even over thousands of steps.

Update order is critical:
```
r̈, θ̈ computed from current state
r_dot += r_ddot * dt    ← velocity first
th_dot += th_ddot * dt
r += r_dot * dt         ← position uses NEW velocity
theta += th_dot * dt
```

### Why 2:1 frequency ratio?
- Pendulum frequency: `ω_pend = √(g / r₀)`
- Spring frequency: `ω_spring = √(k / m)`
- At exactly 2:1, energy transfers completely between modes every half-period
- This is where the interesting rosette shapes appear
- The constants are tuned: with `g=2000`, `k=25`, `r₀ = 0.4 × rows × CELL_H`, ratio ≈ 2.0

### Why `N_COILS * 2` nodes for the spring?
The spring is drawn as a zigzag. Each "coil" has two nodes — one offset left, one offset right of the spring axis. The nodes are evenly spaced along the pivot-to-bob line in pixel space. By computing perpendicular unit vectors from the spring axis, the zigzag automatically rotates as the pendulum swings.

### Why draw order: wire → coil lines → coil nodes → bob?
Later draws overwrite earlier ones. Coil nodes (`*`) should overwrite the lines connecting them. The bob `(@)` should overwrite any coil that overlaps it at the end.

---

## State Machines

No state machine — the pendulum is purely continuous physics. There is one boolean flag:
```
paused = false  ←→  paused = true
        space key toggles
```

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `GRAVITY_PX` | 2000 | Faster swing, higher pendulum frequency |
| `SPRING_K` | 25 | Faster spring oscillation |
| `DAMPING_DEF` | 0.12 | More friction → motion dies faster |
| `REST_LEN_FRAC` | 0.40 | Longer spring → slower pendulum frequency |
| `INIT_THETA_DEG` | 40° | Larger initial swing angle |
| `INIT_R_STRETCH` | 1.15 | Bob starts 15% above rest length |
| `N_COILS` | 8 | More coils in the spring visual |
| `COIL_SPREAD` | 2 cells | Wider spring zigzag |
| `SIM_FPS_DEFAULT` | 120 | Higher physics rate → more accurate ODEs |

**Tuning for resonance:** if `GRAVITY_PX` changes, `SPRING_K` must change to maintain 2:1. Formula: `SPRING_K = 4 * GRAVITY_PX / r₀`. With `r₀ = REST_LEN_FRAC × rows × CELL_H`.

---

## Open Questions for Pass 3

1. What happens visually if you break the 2:1 ratio? Try `SPRING_K = 10` vs `SPRING_K = 100`.
2. What does removing the `2·ṙ·θ̇` Coriolis term do to the motion?
3. What goes wrong if you use regular Euler instead of symplectic — how fast does energy drift?
4. The spring coil zigzag — can you reproduce the exact node positions from scratch?
5. Why does `r` need clamping? What would happen at `r → 0`?
6. Try `DAMPING = 0` (no friction) — does the motion stay bounded?

## From the Source

**Algorithm:** Symplectic (semi-implicit) Euler integration. Update velocities ṙ, θ̇ from accelerations r̈, θ̈ first, then update positions r, θ using the new velocities. This "leapfrog" ordering conserves a modified Hamiltonian — no secular energy drift unlike explicit Euler.

**Physics:** Spring pendulum — two coupled oscillators. Pendulum mode `ω_pend = √(g/r₀)` (like a rigid pendulum). Spring mode `ω_spring = √(k/m)`. At ω_spring ≈ 2·ω_pend (the 2:1 parametric resonance): energy flows periodically between the two modes — the bob traces a spirograph-like rosette before reversing.

**Math:** Polar-coordinate Lagrangian equations of motion. Coriolis-like term `2·ṙ·θ̇/r` in θ̈ comes from the non-inertial polar frame (centripetal acceleration).

**Performance:** Sim runs at 120 Hz; display at ~60 Hz. Between sim steps the angles/lengths are linearly interpolated by alpha, giving smooth motion without running physics at 120 Hz (render-interpolation).

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~1 KB | top-level container: scene + screen + control flags |
| `g_app.scene.pend` | `Pendulum` | ~48 B | polar state (r, θ, ṙ, θ̇), prev-tick snapshot, pivot, rest length, damping |
| `g_app.scene` | `Scene` | ~56 B | owns Pendulum + terminal size + paused flag |

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
