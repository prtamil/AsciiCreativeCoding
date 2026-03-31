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
