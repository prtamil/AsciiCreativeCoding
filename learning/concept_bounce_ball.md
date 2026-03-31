# Pass 1 — bounce_ball: Reference implementation of smooth terminal physics animation

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
