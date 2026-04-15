# Pass 1 — bounce_ball: Reference implementation of smooth terminal physics animation

## From the Source

**Algorithm:** Physics runs in square pixel space, not cell space. Two coordinate spaces with one conversion point: PIXEL SPACE (physics lives here — square grid, one unit = one physical pixel approximately; Width = cols × CELL_W, Height = rows × CELL_H) and CELL SPACE (drawing happens here — `cell_x = pixel_x / CELL_W`, `cell_y = pixel_y / CELL_H`). With this, 1 pixel/tick is the same physical distance in X and Y.

**Math:** Render interpolation (alpha): `alpha = sim_accum / tick_ns ∈ [0.0, 1.0)`. Draw position: `draw_px = ball.px + ball.vx * alpha * dt_sec`. This is technically extrapolation (predict forward from last known state). For constant-velocity balls with elastic wall bounces, forward extrapolation is numerically identical to interpolation and requires no extra storage. If you add acceleration or non-linear forces, switch to storing prev_px/prev_py and lerp between them.

**Performance:** Fixed-timestep accumulator. Physics always runs at exactly `sim_fps` Hz regardless of render frame timing. After draining the accumulator, `sim_accum` holds leftover time. `alpha = sim_accum / tick_ns` gives the render interpolation factor.

**Data-structure:** Ball struct contains only pixel-space data (px, py, vx, vy, color, ch) — zero knowledge of columns or rows. Scene is a flat pool with no dynamic allocation.

## Core Idea
Multiple balls bounce around the terminal with physically correct isotropic motion. The central insight is that physics must never run in terminal cell coordinates — cells are not square, so cell-space physics produces distorted motion. Instead, physics runs in a square "pixel space" and only the final drawing step converts to cell coordinates. The program also interpolates between physics ticks so rendered positions smoothly match wall-clock time, eliminating micro-stutter.

## The Mental Model

Imagine you are animating balls on a sheet of graph paper. If that graph paper has rectangles (tall cells) instead of squares, and you move a ball "one square right" and "one square up" simultaneously, the ball appears to move farther horizontally than vertically, so diagonal motion looks wrong.

The fix: pretend the graph paper is made of tiny squares (8 horizontal sub-divisions, 16 vertical sub-divisions per visible cell). All your physics math uses these tiny squares. When it is time to draw, convert back to the visible cells by dividing: column = tiny_x / 8, row = tiny_y / 16.

The second problem: physics runs at a fixed rate (60 ticks per second). Rendering might fire between ticks. Without correction, the ball's drawn position is always slightly behind "now" — up to one tick late. The fix is to project each ball slightly forward in time by the fraction of a tick that has elapsed since the last physics update.

The program is the reference that every subsequent animation in this codebase copies. The same clock, the same accumulator loop, the same pixel↔cell convention, and the same draw-layer architecture are reused in spring_pendulum.c and bonsai.c.

## Data Structures

### Ball
```
px, py   — position in pixel space (float, sub-pixel precision)
vx, vy   — velocity in pixels per second (float)
color    — which ncurses color pair index (1..7)
ch       — the ASCII character to draw ('o', '*', 'O', '@', '+')
```
Everything here is in pixel space. The struct has zero knowledge of columns or rows. `px` and `py` are floats so a ball can sit between two cells and gradually drift through them.

### Scene
```
balls[]  — array of Ball, capacity BALLS_MAX (20)
n        — how many are currently active (1..20)
paused   — if true, scene_tick does nothing
```
A flat pool — no dynamic allocation. Removing a ball just decrements `n`. Adding a ball calls ball_spawn on the next slot.

### Screen
```
cols     — terminal column count (updated on resize)
rows     — terminal row count
```
A thin wrapper that exists mainly to bundle the two dimensions together. It holds no ncurses window pointer because there is only stdscr.

### App
```
scene        — the ball simulation
screen       — terminal dimensions
sim_fps      — current simulation tick rate (10–60 Hz, user-adjustable)
running      — signal-safe flag: 0 = exit main loop
need_resize  — signal-safe flag: SIGWINCH sets this
```
`running` and `need_resize` are declared `volatile sig_atomic_t` because signal handlers write to them from outside the main thread of control.

## The Main Loop

Each iteration of the main loop:

1. **Check for resize.** If `need_resize` is set (SIGWINCH fired), call `endwin` + `refresh` to re-read the terminal size, clamp any balls that are now outside the new boundary, and reset the timing state so no phantom ticks accumulate during the resize.

2. **Measure elapsed time (dt).** Read the monotonic clock. Subtract the timestamp from the previous frame. This is the wall-clock time in nanoseconds since the last loop iteration. Clamp dt to 100 ms to prevent a "spiral of death" — if the program is suspended and resumed, we don't want to simulate hundreds of missed ticks all at once.

3. **Run the physics accumulator.** Add dt to `sim_accum`. While `sim_accum` is at least one tick's worth of time (`NS_PER_SEC / sim_fps`), run one physics tick (advance all balls, handle wall collisions) and subtract one tick duration from `sim_accum`. This guarantees physics always runs at exactly `sim_fps` Hz regardless of render frame timing.

4. **Compute alpha.** After draining the accumulator, `sim_accum` holds the leftover nanoseconds — how far into the next tick we are. Dividing by `tick_ns` gives alpha ∈ [0, 1). This is the render interpolation factor.

5. **Update FPS counter.** Count frames and accumulated time; every 500 ms compute frames/time for the displayed FPS.

6. **Sleep for frame cap.** Sleep until 1/60 second has elapsed since the start of this frame. This caps render rate at ~60 Hz without burning CPU. The sleep happens before drawing so the terminal I/O latency does not compound with physics lag.

7. **Draw and present.** Call `erase()` to clear the back buffer. Call `scene_draw()` to write each ball at its interpolated position. Call `mvprintw()` to overlay the HUD. Call `wnoutrefresh(stdscr)` + `doupdate()` for a single atomic terminal write.

8. **Handle input.** Call `getch()` in non-blocking mode (nodelay is set). If a key was pressed, dispatch it. If 'q'/ESC, set `running = 0` which ends the loop.

## Non-Obvious Decisions

**Why pixel space, not cell space?**
Terminal cells are typically about 2× taller than wide in physical pixels. If physics ran in cell units, `vx = vy = 1` would move a ball farther horizontally than vertically per tick. A ball launched at 45° would curve visually. Speeds would feel uneven depending on direction. By using a square pixel space (CELL_W=8, CELL_H=16), one pixel unit is the same physical distance in all directions.

**Why CELL_W=8 and CELL_H=16 specifically?**
These are not real screen pixel dimensions — they are the number of sub-pixel steps per cell. The ratio (16/8 = 2.0) must match the terminal cell aspect ratio. The magnitude determines physics precision: higher values give more discrete positions within each cell, making motion appear smoother for slow-moving objects. CELL_W=8 means a ball traveling at 1 pixel/tick needs 8 ticks to cross one column; CELL_H=16 means 16 ticks to cross one row. With render interpolation this is fine. Without it, a ball moving at 1 pixel/tick would appear to jump a full cell every 8 or 16 frames — staircase motion.

**Why `floorf(px/CELL_W + 0.5f)` for coordinate conversion instead of `roundf` or `(int)`?**
- `(int)(px/CELL_W)` always truncates toward zero — a ball near a cell boundary spends asymmetric time in each cell, creating a visible staircase.
- `roundf(px/CELL_W)` has a "round half to even" edge case. When a ball sits exactly on a cell boundary, `roundf` can round one direction one call and the other direction next call depending on floating-point state — the ball flickers between two cells.
- `floorf(px/CELL_W + 0.5f)` is "round half up" — always rounds to the nearest cell, always ties go up. Deterministic every call. No flicker.

**Why fixed-timestep simulation with accumulator?**
If you just multiply velocity by the actual elapsed time each frame, physics results depend on frame timing. Two runs at different frame rates produce different trajectories. A frame spike causes a ball to tunnel through a wall. The fixed timestep ensures physics is deterministic: wall bounces happen at the same ball positions every time, regardless of frame rate. The accumulator bridges the gap — real time advances by dt, and we catch up by running as many fixed-size ticks as needed.

**Why forward extrapolation instead of lerp between prev and current?**
For constant-velocity balls with elastic wall bounces, projecting forward from the current state is mathematically identical to interpolating between the previous-tick position and the current-tick position. Forward extrapolation (`draw_px = px + vx * alpha * dt_sec`) requires no extra storage. The code comments explicitly note: if you add gravity or non-linear forces, switch to storing `prev_px/prev_py` and lerping.

**Why `erase()` not `clear()`?**
`clear()` marks every cell dirty in ncurses' diff engine, forcing a full terminal repaint. `erase()` just zeroes the virtual screen buffer — the diff on the next `doupdate()` only sends characters that actually changed from the previous frame. For a terminal at 80×24 with only 5 balls, `erase()` means ~5 changed cells per frame instead of 1920.

**Why `wnoutrefresh` + `doupdate` instead of just `refresh`?**
`refresh()` on stdscr is equivalent to `wnoutrefresh(stdscr)` + `doupdate()` in a single-window program. But `wnoutrefresh` marks the window ready without writing to the terminal; `doupdate` sends the single diff. This pattern becomes important with multiple windows (avoiding partial frame visibility between window refreshes). Using it here enforces the habit.

**Why `typeahead(-1)`?**
By default, ncurses interrupts `refresh()`/`doupdate()` if input is waiting on stdin, to reduce latency. `typeahead(-1)` disables this check. Without it, heavy typing can interrupt screen updates mid-frame, causing visual tearing on slow terminals.

**Why signal handlers write only to `volatile sig_atomic_t` flags?**
Signal handlers can interrupt any instruction. If a signal handler called ncurses functions or modified complex state, it could corrupt data structures mid-operation. By only setting a single integer flag, the handler is async-signal-safe. The main loop checks these flags at known-safe points each iteration.

**Why seed with `clock_ns()` instead of `time(NULL)`?**
`time(NULL)` has 1-second resolution. If you start two instances within the same second, they get the same seed and produce identical ball spawns. `clock_ns()` has nanosecond resolution, so consecutive runs always produce different patterns.

**Why rejection-sampling for the unit vector in `ball_spawn`?**
If you use `vx = random_in_[-1,1]`, `vy = random_in_[-1,1]` and normalize, the angle distribution is not uniform — diagonal directions (45°, 135°, etc.) are over-represented because the unit square has more area near corners relative to the inscribed circle. Rejection sampling (pick a random point in the unit square, discard if outside the unit circle, then normalize) produces a uniform angle distribution.

**Why `max_px = pw(cols) - 1`?**
The pixel space runs from 0 to `pw(cols) - 1` inclusive, not `pw(cols)`. When a ball is at `max_px` it is at the rightmost pixel, which maps to column `(max_px) / CELL_W = cols - 1` (the last visible column). If you used `pw(cols)` as the boundary, a ball could sit at a pixel that maps to column `cols` — one past the right edge — and `mvaddch` would fail or wrap.

**Why clamp ball position when the terminal is resized?**
When the terminal shrinks, some balls may be at pixel positions that are now outside the new smaller screen. Before resuming physics, those balls are clamped to the new boundary. Without clamping, a ball could be at `px = 1500` when `pw(new_cols) = 1200` — its next reflection would be computed at the wrong boundary, producing a teleport.

## State Machines

### App-level state
```
        ┌───────────────────────────────────────┐
        │              RUNNING                  │
        │                                       │
        │  [physics ticks] ←── accumulator     │
        │  [render + present] each frame        │
        │  [input dispatch]                     │
        │                                       │
        │  SIGWINCH ──────────────→ need_resize │
        │       need_resize ──────→ do_resize() │
        │                                       │
        │  SIGINT/SIGTERM ────────→ running = 0 │
        └───────────────────────────────────────┘
                          │
               'q'/ESC/signal
                          │
                          v
                        EXIT
                      (endwin)
```

### Per-ball state
Balls have no discrete states — they are always either moving (vx, vy non-zero) or reflected at a wall boundary. The "paused" flag is on Scene, not Ball: when paused, ball_tick is never called, but balls still exist with their current velocity ready to resume.

### Input actions
| Key | Effect |
|-----|--------|
| q / Q / ESC | running = 0 → exit |
| space | toggle scene.paused |
| r / R | re-spawn all balls with new random positions/velocities |
| = / + | add one ball (if < BALLS_MAX) |
| - | remove one ball (if > BALLS_MIN) |
| ] | increase sim_fps by SIM_FPS_STEP |
| [ | decrease sim_fps by SIM_FPS_STEP |

## Key Constants and What Tuning Them Does
| Constant | Default | Effect of changing |
|----------|---------|-------------------|
| CELL_W | 8 | Sub-pixel X resolution per cell. Increase: more precision, but must keep CELL_H/CELL_W = ~2.0. |
| CELL_H | 16 | Sub-pixel Y resolution per cell. Ratio to CELL_W must match cell aspect ratio. |
| SPEED_MIN | 300.0 px/s | Minimum ball speed. Too low → staircase motion (ball moves less than 1 cell per many frames). Must be ≥ CELL_H * SIM_FPS / 4. |
| SPEED_MAX | 600.0 px/s | Maximum ball speed. Too high → ball tunnels through walls (crosses more than a full cell per tick). |
| SIM_FPS_DEFAULT | 60 Hz | Physics tick rate. Higher = more accurate wall-bounce timing. |
| BALLS_DEFAULT | 5 | Starting ball count. |
| BALLS_MAX | 20 | Pool size cap. |
| FPS_UPDATE_MS | 500 ms | How often the HUD FPS display updates. Lower = more flickery but more responsive. |
| HUD_COLS | 40 | Width reserved for the status bar in the top-right corner. |
| dt clamp | 100 ms | Maximum dt allowed per frame. Prevents physics runaway after OS suspend/resume. |
| frame cap | 1/60 s | Render rate ceiling. Prevents burning CPU on fast machines. |

## Open Questions for Pass 3
- If you add gravity (vy increases each tick), the forward-extrapolation in scene_draw is wrong — the ball's draw position will be above where it actually is. How do you fix it? (Answer: store prev_px/prev_py and lerp between prev and current.)
- What happens to alpha when the simulation is paused? Does the ball creep slightly forward? (Yes — velocities are still non-zero, alpha still applies. See the main loop comment about zeroing alpha on pause.)
- How does `typeahead(-1)` interact with getch() in nodelay mode? Does it matter for a non-blocking getch?
- What is the exact rounding behavior of `floorf(px/CELL_W + 0.5f)` for negative pixel values? Is this ever possible?
- Why does the frame cap sleep with `elapsed = clock_ns() - frame_time + dt` (using `frame_time` from the start of the loop, not the actual render start)? What would happen if you just measured from the render start?
- What happens if a ball is added (via '=') while the simulation is paused? Is its velocity meaningful?
- If SIM_FPS is lowered to 10, does interpolation still produce smooth motion, or is there a visual artifact from extrapolating across a large tick?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~1.5 KB | top-level container: scene + screen + control flags |
| `g_app.scene.balls[BALLS_MAX]` | `Ball[20]` | ~480 B | pixel position, velocity, color, character per ball |

# Pass 2 — bounce_ball: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | All tunable constants: CELL_W/H, SPEED_MIN/MAX, BALLS_DEFAULT/MIN/MAX, N_COLORS, tick rate |
| §2 clock | `clock_ns()` — monotonic nanosecond timestamp; `clock_sleep_ns()` — precise sleep |
| §3 color | `color_init()` — allocate 7 color pairs (256-color + 8-color fallback) |
| §4 coords | `pw/ph` — pixel space dimensions; `px_to_cell_x/y` — the ONE place aspect ratio is handled |
| §5 ball | `Ball` struct; `ball_spawn()` — random position + isotropic velocity; `ball_tick()` — move + reflect |
| §6 scene | `Scene` owns ball pool; `scene_tick()` — advance all balls; `scene_draw(alpha)` — draw at interpolated positions |
| §7 screen | `screen_init/draw/present/resize` — ncurses setup + HUD + atomic flush |
| §8 app | `App` struct; signal handlers; main dt loop |

---

## Data Flow Diagram

```
CLOCK_MONOTONIC
    │
    ▼
clock_ns() → dt (nanoseconds)
    │
    ▼
sim_accum += dt
    │
    while sim_accum >= tick_ns:
    │
    ▼
scene_tick(dt_sec)
    │
    ├── pw(cols), ph(rows)   [cell → pixel boundary, ONLY here]
    │
    └── ball_tick(ball, dt, max_px, max_py)
            │
            ├── px += vx * dt
            ├── py += vy * dt
            └── reflect if wall hit

sim_accum -= tick_ns
    │
    ▼
alpha = sim_accum / tick_ns   [0.0 .. 1.0)
    │
    ▼
scene_draw(alpha)
    │
    ├── draw_px = px + vx * alpha * dt_sec   [forward extrapolation]
    ├── draw_py = py + vy * alpha * dt_sec
    ├── cell_x = px_to_cell_x(draw_px)
    ├── cell_y = px_to_cell_y(draw_py)
    └── mvaddch(cell_y, cell_x, ch) with color pair

    ▼
screen_draw() → erase → scene_draw → HUD → wnoutrefresh
    │
    ▼
screen_present() → doupdate()   [ONE atomic write: diff → terminal]
```

---

## Function Breakdown

### clock_ns() → int64_t
Purpose: read CLOCK_MONOTONIC and return nanoseconds since epoch
Steps:
1. Call `clock_gettime(CLOCK_MONOTONIC, &t)`
2. Return `t.tv_sec * 1e9 + t.tv_nsec`
Edge cases: none (CLOCK_MONOTONIC never fails on Linux)

---

### clock_sleep_ns(ns)
Purpose: sleep for exactly ns nanoseconds
Steps:
1. If ns <= 0, return immediately
2. Fill `timespec` with sec + nsec parts
3. Call `nanosleep()`
Edge cases: early wakeup from signal — not retried (acceptable drift)

---

### color_init()
Purpose: allocate 7 color pairs for ball colors
Steps:
1. `start_color()`
2. If COLORS >= 256: assign 256-color palette (red=196, orange=208, yellow=226, green=46, cyan=51, blue=21, magenta=201)
3. Else: 8-color fallback
Edge cases: no fallback for COLORS < 8 (terminal too old)

---

### pw(cols) → int
Purpose: pixel space width
Steps: return `cols * CELL_W`

### ph(rows) → int
Purpose: pixel space height
Steps: return `rows * CELL_H`

---

### px_to_cell_x(px) → int
Purpose: convert pixel x to terminal column
Steps:
1. `return (int)floorf(px / CELL_W + 0.5f)`
Why `floorf(x + 0.5)` not `roundf()`:
- `roundf()` uses banker's rounding — rounds 0.5 to even, which alternates between 0 and 1 at cell boundaries → visible flicker
- `floorf(x + 0.5)` always rounds up at 0.5 — deterministic, no flicker

### px_to_cell_y(py) → int
Purpose: convert pixel y to terminal row
Steps: same as px_to_cell_x but divide by CELL_H

---

### ball_spawn(ball, index, cols, rows)
Purpose: place ball at random position with random isotropic velocity
Steps:
1. Random pixel position within one cell margin of the edges
2. Generate isotropic direction:
   - pick random (dx, dy) in [-1, 1]²
   - reject if outside unit circle (rejection sampling)
   - normalize to unit vector
3. Scale by random speed in [SPEED_MIN, SPEED_MAX]
4. Assign color = (index % N_COLORS) + 1
5. Assign character from `"o*O@+"` by index
Why rejection sampling: using separate random vx, vy creates more diagonal balls than axis-aligned (non-uniform angle distribution). Unit circle rejection gives uniform angle distribution.

---

### ball_tick(ball, dt, max_px, max_py)
Purpose: advance one ball by dt seconds; reflect off walls
Steps:
1. `px += vx * dt`
2. `py += vy * dt`
3. If px < 0: set px = 0, negate vx
4. If px > max_px: set px = max_px, negate vx
5. If py < 0: set py = 0, negate vy
6. If py > max_py: set py = max_py, negate vy
Edge cases: none — clamping + flip is elastic reflection

---

### scene_tick(scene, dt, cols, rows)
Purpose: advance all balls one physics step
Steps:
1. If paused, return
2. Compute pixel boundaries: max_px = pw(cols)-1, max_py = ph(rows)-1
3. For each active ball: call ball_tick(ball, dt, max_px, max_py)
Note: this is the ONLY function that calls pw/ph — the single conversion point

---

### scene_draw(scene, alpha)
Purpose: draw all balls at render-interpolated positions
Steps:
1. For each active ball:
   a. Compute draw position:
      - `draw_px = px + vx * alpha * dt_sec`
      - `draw_py = py + vy * alpha * dt_sec`
   b. Convert: `cell_x = px_to_cell_x(draw_px)`, `cell_y = px_to_cell_y(draw_py)`
   c. Clamp to screen bounds
   d. `attron(COLOR_PAIR(ball.color))` → `mvaddch(cell_y, cell_x, ball.ch)` → `attroff`
Edge cases: cells outside screen bounds skipped

---

### screen_init(screen)
Purpose: initialize ncurses
Steps:
1. `initscr()` — enter ncurses mode
2. `noecho()` — don't echo keystrokes
3. `cbreak()` — read keys immediately (no line buffering)
4. `curs_set(0)` — hide cursor
5. `nodelay(stdscr, TRUE)` — getch() returns ERR immediately if no key
6. `keypad(stdscr, TRUE)` — enable arrow keys
7. `typeahead(-1)` — disable typeahead detection → atomic doupdate writes
8. `color_init()` — set up color pairs
9. `getmaxyx(stdscr, rows, cols)` — read terminal dimensions

---

### screen_draw(screen, scene, fps, alpha)
Purpose: build complete frame in ncurses back buffer
Steps:
1. `erase()` — clear back buffer (newscr), no terminal I/O yet
2. `scene_draw(scene, alpha)` — write all balls
3. Build HUD string: fps + ball count + sim fps
4. `attron(CP_HUD | A_BOLD)` → `mvprintw(0, hud_x, buf)` → `attroff` — HUD drawn on top

---

### screen_present()
Purpose: flush back buffer to terminal
Steps:
1. `wnoutrefresh(stdscr)` — mark stdscr ready, still no I/O
2. `doupdate()` — diff newscr vs curscr → write ONLY changed cells to terminal
Why two calls: `wnoutrefresh` + `doupdate` = atomic batch write, no partial frame visible

---

### app_handle_key(app, ch) → bool
Purpose: dispatch key to correct action; return false to quit
Steps:
1. 'q' / ESC → return false
2. space → toggle scene.paused
3. '=' → add ball if under BALLS_MAX, spawn it
4. '-' → remove ball if above BALLS_MIN
5. 'r' → respawn all balls randomly
6. ']' → increase sim_fps up to SIM_FPS_MAX
7. '[' → decrease sim_fps down to SIM_FPS_MIN
8. default → ignore, return true

---

## Pseudocode — Core Loop

```
setup:
  register atexit(cleanup)
  register signal(SIGINT/SIGTERM) → running = 0
  register signal(SIGWINCH) → need_resize = 1
  screen_init()
  scene_init()
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. handle resize:
     if need_resize:
       endwin() + refresh() to flush terminal resize
       getmaxyx() for new dimensions
       reinit scene with new dimensions
       reset frame_time and sim_accum (avoid spike)
       need_resize = 0

  2. compute dt:
     now = clock_ns()
     dt = now - frame_time
     frame_time = now
     cap dt at 100ms  ← prevents "spiral of death" if tab was backgrounded

  3. physics ticks (fixed timestep):
     tick_ns = 1_000_000_000 / sim_fps
     dt_sec = tick_ns / 1_000_000_000.0
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec, cols, rows)
       sim_accum -= tick_ns

  4. compute alpha (render interpolation):
     alpha = sim_accum / tick_ns    [0.0 .. 1.0)

  5. update FPS counter:
     frame_count++
     fps_accum += dt
     if fps_accum >= 500ms:
       fps_display = frame_count / (fps_accum in seconds)
       reset frame_count and fps_accum

  6. frame cap:
     elapsed = clock_ns() - frame_time + dt
     sleep(1_000_000_000/60 - elapsed)
     WHY sleep BEFORE render: keeps render rate stable regardless of terminal I/O time

  7. draw + present:
     screen_draw(fps_display, alpha)   ← erase + scene + HUD into newscr
     screen_present()                  ← doupdate() → atomic write to terminal

  8. input:
     ch = getch()    ← non-blocking (nodelay), returns ERR if no key
     if ch != ERR:
       if !app_handle_key(ch): running = 0

cleanup:
  endwin()   ← called by atexit even if crash
```

---

## Interactions Between Modules

```
App
 ├── owns Screen (ncurses state, cols/rows)
 ├── owns Scene (ball pool, paused flag)
 ├── reads sim_fps (adjustable by key)
 └── main loop drives everything

Screen
 ├── calls color_init() at startup
 ├── calls scene_draw(alpha) inside screen_draw()
 └── calls screen_present() after screen_draw()

Scene
 ├── calls ball_spawn() at init and on '+' key
 ├── calls ball_tick() via scene_tick() each physics step
 └── calls px_to_cell_x/y() inside scene_draw()

§4 coords (pw, ph, px_to_cell_x/y)
 └── called ONLY by scene_tick() and scene_draw() — nowhere else

§2 clock (clock_ns, clock_sleep_ns)
 └── called only by main loop — physics and render code never touch the clock

Signal handlers
 ├── write running = 0 (SIGINT/SIGTERM)
 └── write need_resize = 1 (SIGWINCH)
 └── both flags are volatile sig_atomic_t — safe to write from signal context
```

---

## Key Patterns to Internalize

**Fixed timestep accumulator:**
Physics always advances by exactly `tick_ns` nanoseconds per step. The sim rate is decoupled from the render rate. You can run physics at 120 Hz and render at 60 Hz.

**Render interpolation (forward extrapolation):**
After draining the accumulator, `sim_accum` holds leftover time. `alpha = sim_accum / tick_ns` tells you how far you are into the *next* tick. You extrapolate position forward by `alpha * tick_duration`. This eliminates micro-stutter at all render rates.

**Single conversion point:**
`px_to_cell_x/y` are the only functions that touch cell coordinates. All physics is in pixel space. This is the entire fix for non-square terminal cells.

**ncurses double buffer:**
`erase()` clears newscr (back buffer). Drawing functions write to newscr. `wnoutrefresh()` marks it ready. `doupdate()` performs ONE atomic diff of newscr vs curscr and writes only changed cells. No partial frames, no tearing.
