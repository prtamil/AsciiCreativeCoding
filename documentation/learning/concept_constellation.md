# Pass 1 — constellation: Live Star Constellation Map

## Core Idea

Stars drift slowly across the screen with random wander acceleration. When any two stars come within a threshold distance of each other, a line is drawn between them using slope-matched ASCII characters. The line style (bold/normal/stippled) depends on how close the stars are. The result is a living constellation map that evolves as stars drift.

---

## The Mental Model

Imagine 30 stars floating on a dark sky, each slowly curving around (due to random wander). When two stars get close, a connection line appears between them. Close stars get a bright solid line; far-apart-but-still-connected stars get a faint dotted line.

The trick is that lines are drawn as Bresenham line segments where each cell's character is chosen based on the local step direction — so lines at different angles use `|`, `-`, `/`, `\` appropriately. This makes the star map look like handdrawn star charts.

A `cell_used[][]` boolean grid ensures that when multiple connection lines overlap, only the first writer wins — no messy overprinting.

---

## Data Structures

### `Star` struct
| Field | Meaning |
|---|---|
| `px, py` | Current position in pixel space |
| `prev_px, prev_py` | Position at previous tick (for true lerp interpolation) |
| `vx, vy` | Velocity in pixels per second |
| `color` | Color pair index (1..N_STAR_COLORS) |
| `ch` | Symbol character from `"*+o@."` |

### `Scene` struct
| Field | Meaning |
|---|---|
| `stars[STARS_MAX]` | Pool of all stars |
| `n` | Active star count |
| `paused` | Flag |
| `connect_preset` | Index into k_connect_presets (tight/normal/wide) |

### `cell_used[rows][cols]`
A VLA (variable-length array) allocated on the stack each draw frame. Tracks which cells have already been written by connection lines. First writer wins — no double-writing.

---

## The Main Loop

Standard fixed-timestep with render interpolation:
1. `scene_tick()` — move each star (wander + speed cap + wall bounce)
2. `alpha = sim_accum / tick_ns`
3. `scene_draw(alpha)` — lerp star positions, draw connections, draw stars

---

## Non-Obvious Decisions

### True lerp, not forward extrapolation
`bounce_ball.c` uses forward extrapolation: `draw_px = px + vx * alpha * dt`

`constellation.c` uses true lerp: `draw_px = prev_px + (px - prev_px) * alpha`

Why the difference? Bounce_ball has constant velocity between ticks — extrapolation is exact. But stars have wander acceleration that changes velocity each tick. You cannot predict where the star will be "alpha ticks into the future" from current velocity alone without integrating the acceleration. Lerp between the two known states (prev and current) is always exact.

### Wander acceleration + speed cap
Each tick, stars receive a small random acceleration in any direction. This makes paths gently curve instead of moving in straight lines forever. Without the speed cap, wander would accumulate and stars would eventually move too fast. The cap rescales velocity to SPEED_CAP when exceeded.

### Stippled Bresenham for far connections
Lines are drawn with a `stipple` parameter (every Nth cell):
- `ratio < 0.50` (close): stipple=1 (every cell), A_BOLD
- `ratio < 0.75` (medium): stipple=1 (every cell), normal
- `ratio < 1.00` (far): stipple=2 (every other cell), normal

Far connections appear dotted, giving a visual sense of distance.

### Character selection from local step direction
Standard Bresenham produces runs of the same character — a very flat line is all `-`, a very steep line is all `|`. But between those extremes, the character changes based on whether the next step is diagonal or straight. This is computed from the **Bresenham error value** per step, not the overall angle. Result: smooth-looking lines without extra math.

### `cell_used[][]` VLA deduplication
When stars A-B and stars A-C have overlapping connection lines, without deduplication both would write to the same cells and produce garbled characters. `cell_used` prevents any cell from being written by a second connection line, so the first line drawn "claims" each cell.

Being a VLA allocated on the stack (`bool cell_used[rows][cols]`), it's automatically zeroed via `memset` and freed when `scene_draw()` returns — no malloc needed.

---

## State Machines

No state machine. Purely continuous physics. One boolean: paused.

---

## From the Source

**Algorithm:** Star motion uses an Ornstein-Uhlenbeck-like wander — each tick velocity is perturbed by a small random `δvx, δvy` bounded at `WANDER_FORCE`, with speed capped at `SPEED_MAX`. This is the same mean-reverting bounded random walk used to model Brownian motion with a restoring force.

**Math:** Slope character selection uses pixel-space angle thresholds: 0–22.5° → `─`; 22.5–67.5° → `╲` or `╱`; 67.5–90° → `│`. Connection distance `CONNECT_DIST` is measured in pixels and aspect-corrected before comparison. The O(n²) pair loop for connection lines is the performance bottleneck.

**Rendering:** Staircase rule intentionally relaxed for drift speed (SPEED_MIN=50 px/s is below the rule's 240 px/s threshold). Wander force keeps motion non-axis-aligned to reduce staircase visibility; lerp interpolation eliminates sub-cell jitter between ticks.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of increasing |
|---|---|---|
| `STARS_DEFAULT` | 30 | More stars = more connections = denser map |
| `SPEED_MIN/MAX` | 50–120 px/s | Faster stars = rapidly evolving map |
| `WANDER_ACCEL` | 20 px/s² | Higher = more erratic paths |
| `SPEED_CAP` | 130 px/s | Lower = slower stars even with high wander |
| `k_connect_presets` | 120/200/280 px | Larger = more connections visible |

---

## Open Questions for Pass 3

1. Remove `cell_used` — what does the visual look like when lines overwrite each other?
2. Change the stipple condition to use random sampling instead of fixed-period stepping — how does it look?
3. What happens if you use forward extrapolation instead of lerp? (Start with no wander to see they're equivalent, then add wander to see the difference.)
4. Try connecting stars in order of distance (closest first) vs arbitrary order — does it change what `cell_used` blocks?
5. The VLA `cell_used[rows][cols]` is allocated on the stack every frame — what's the maximum safe terminal size before this risks a stack overflow?

---

# Pass 2 — constellation: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | CELL_W=8, CELL_H=16, SPEED_MIN/MAX, WANDER_ACCEL, SPEED_CAP, connect_presets[3], STARS_DEFAULT/MIN/MAX |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 6 star pairs (1-6), connection pair (7), HUD pair (8) |
| §4 coords | `pw/ph`, `px_to_cell_x/y` — single aspect-ratio conversion point |
| §5 star | `Star` struct (prev_px/py for lerp); `star_spawn()`, `star_tick()` (wander + cap + bounce) |
| §6 scene | `Scene` owns star pool; `scene_tick()`, `scene_draw(alpha)` — connection lines + star dots |
| §7 screen | ncurses init/resize/present/HUD |
| §8 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
star_tick():
  prev_px = px, prev_py = py      ← save for lerp
  ax = rand in [-WANDER_ACCEL, +WANDER_ACCEL]
  ay = same
  vx += ax * dt; vy += ay * dt   ← accumulate wander
  spd = sqrt(vx² + vy²)
  if spd > SPEED_CAP: scale vx,vy down to SPEED_CAP
  px += vx * dt; py += vy * dt
  elastic bounce off pixel-space walls

scene_draw(alpha):
  lerp all star positions:
    draw_px[i] = prev_px + (px - prev_px) * alpha
    draw_py[i] = prev_py + (py - prev_py) * alpha
    dcx[i] = px_to_cell_x(draw_px[i])
    dcy[i] = px_to_cell_y(draw_py[i])

  bool cell_used[rows][cols] = {false}  ← VLA on stack

  for each pair (i, j) where i < j:
    dx_px = draw_px[j] - draw_px[i]
    dy_px = draw_py[j] - draw_py[i]
    dist² = dx_px² + dy_px²
    if dist² >= connect_dist²: skip

    ratio = sqrt(dist²) / connect_dist
    if ratio < 0.50: stipple=1, attr = CONN_PAIR + A_BOLD
    elif ratio < 0.75: stipple=1, attr = CONN_PAIR
    else: stipple=2, attr = CONN_PAIR  ← every other cell

    draw_line(w, dcx[i], dcy[i], dcx[j], dcy[j], attr, stipple, cols, rows, cell_used)

  for each star i:
    mvwaddch(dcy[i], dcx[i], star.ch) with COLOR_PAIR(star.color) + A_BOLD
```

---

## Function Breakdown

### star_spawn(star, idx, cols, rows)
Purpose: initialize star at random position with isotropic velocity
Steps:
1. Random pixel position within one-cell margin of edges
2. Rejection-sample unit disk for isotropic direction
3. Scale to random speed in [SPEED_MIN, SPEED_MAX]
4. color = (idx % N_STAR_COLORS) + 1
5. ch = k_star_chars[idx % k_n_star_chars]
6. prev_px = px, prev_py = py (no lerp artifact on first frame)

---

### star_tick(star, dt, max_px, max_py)
Purpose: advance one star with wander acceleration
Steps:
1. prev_px = px; prev_py = py
2. ax = rand[-WANDER_ACCEL, +WANDER_ACCEL]; ay = same
3. vx += ax*dt; vy += ay*dt
4. Speed cap: if sqrt(vx²+vy²) > SPEED_CAP, rescale
5. px += vx*dt; py += vy*dt
6. Elastic bounce: flip velocity and clamp position at each wall

---

### draw_line(w, x0, y0, x1, y1, attr, stipple, cols, rows, cell_used)
Purpose: Bresenham line with angle-appropriate chars, stipple, and dedup
Steps:
1. Compute adx, ady, step directions sx, sy
2. `diag = (sx*sy > 0) ? '\\' : '/'`  ← fixed for whole line
3. If adx >= ady (shallow — one cell per column):
   - Standard Bresenham error accumulation
   - At each x step: `next_err = err - ady`
   - If `next_err < 0`: next step will move y → char = diag
   - Else: stays horizontal → char = '-'
   - If `step % stipple == 0` and cell not used and in bounds: draw
4. Else (steep — one cell per row):
   - Same but: `next_err < 0` → char = diag, else → char = '|'
5. Each drawn cell: `cell_used[y][x] = true` (first-writer-wins)

---

### scene_draw(scene, window, cols, rows, alpha)
Purpose: render frame with connections first, then stars on top
Steps:
1. Lerp all star positions using alpha
2. Convert to cell coordinates, clamp to screen bounds
3. Zero-initialize `cell_used[rows][cols]` VLA
4. Connection pass (all pairs i<j):
   a. Compute pixel-space distance between draw positions
   b. If distance >= connect_dist: skip
   c. Compute ratio = dist / connect_dist → determine style
   d. `draw_line()` with style
5. Star pass (all stars):
   a. `mvwaddch(dcy, dcx, ch)` + A_BOLD
   b. Stars are drawn AFTER lines — overwrites any line at star position
   c. Stars are NOT checked against cell_used (stars always visible)

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  scene_init(cols, rows)   ← spawn STARS_DEFAULT stars
  frame_time = clock_ns()
  sim_accum = 0
  sim_fps = SIM_FPS_DEFAULT (60)

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       scene_init(new cols, rows)   ← respawn stars in new bounds
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     tick_ns = 1_000_000_000 / sim_fps
     dt_sec = tick_ns / 1e9
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows)
       sim_accum -= tick_ns

  4. alpha = sim_accum / tick_ns

  5. FPS counter (every 500ms)

  6. frame cap: sleep to 60fps

  7. draw:
     erase()
     scene_draw(alpha, stdscr, cols, rows)
     HUD: fps, n stars, connect threshold name
     wnoutrefresh + doupdate

  8. input:
     q/ESC  → quit
     space  → toggle paused
     =      → add star (up to STARS_MAX)
     -      → remove star (down to STARS_MIN)
     r      → respawn all stars randomly
     ]      → sim_fps += SIM_FPS_STEP
     [      → sim_fps -= SIM_FPS_STEP
     c      → connect_preset = (connect_preset+1) % N_CONNECT_PRESETS
```

---

## Interactions Between Modules

```
App
 └── owns Scene (star pool + paused + connect_preset)

Scene
 ├── stars[]: each star_tick saves prev_px/py then advances
 └── scene_draw():
      lerp prev→cur with alpha
      O(n²) pair loop for connection lines
      VLA cell_used[rows][cols] per frame (stack, auto-freed)
      draw_line() per connection
      mvaddch per star

§4 coords
 └── px_to_cell_x/y called in scene_draw only
     pw/ph called in scene_tick to compute pixel boundaries
```

---

## True Lerp vs Forward Extrapolation

```
bounce_ball.c (forward extrapolation — works because velocity is constant):
  draw_px = px + vx * alpha * dt_sec

constellation.c (true lerp — required because wander changes velocity):
  draw_px = prev_px + (px - prev_px) * alpha

Why lerp is exact:
  alpha=0 → draw at prev_px (start of tick)
  alpha=1 → draw at px (end of tick)
  alpha=0.5 → draw halfway between — physically correct for any acceleration profile

Why extrapolation would drift:
  With wander, vx at tick end ≠ vx at tick start
  draw_px = px + vx_new * alpha * dt_sec ≠ actual in-between position
  This creates small but visible jumps at each tick boundary
```

---

## Connection Line Styles Summary

```
Pixel distance / connect_dist    Style
──────────────────────────────────────────────────
< 50% of threshold              Bold, every cell
50%–75% of threshold            Normal, every cell
75%–100% of threshold           Normal, every other cell (stippled)
> 100% of threshold             No line drawn
```
