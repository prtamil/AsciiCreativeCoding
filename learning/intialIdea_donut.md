# Pass 2 — donut.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | All tunable constants: FPS bounds, rotation speeds, torus geometry (R1, R2, K2), angle steps, size scale, timing |
| §2 clock | `clock_ns()` — nanosecond monotonic timestamp; `clock_sleep_ns()` — nanosecond sleep |
| §3 color | `color_init()` — set up 8 grey ncurses color pairs; `lumi_attr()` — map luminance level 0..7 to ncurses attribute |
| §4 torus | The core: geometry constants, zbuffer, framebuffer, `torus_init`, `torus_tick`, `torus_render`, `torus_draw` |
| §5 screen | `screen_init/free/resize/draw/present` — thin wrapper around ncurses stdscr |
| §6 app | `App` struct, signal handlers, resize handler, key handler, `main` dt-loop |

## Data Flow Diagram

```
  KEY INPUT
      |
      v
  app_handle_key()
      |
      +---> torus.rot_a, torus.rot_b (speed)
      +---> torus.k1_scale (size)
      +---> torus.paused

  CLOCK
      |
      v
  delta_time (ns)
      |
      v
  sim_accum += dt
  while sim_accum >= tick_ns:
      torus_tick(dt_sec)   <---  torus.A += rot_a * dt_sec
                                 torus.B += rot_b * dt_sec

  torus.A, torus.B
      |
      v
  torus_render()
      |
      for each (theta, phi) on torus surface:
          surface_point(theta, phi, R1, R2)
              |
              v
          rotate(A, B)    --> world_xyz
              |
              v
          perspective_divide(K1, K2) --> screen_xy
              |
              v
          luminance_formula(A, B, theta, phi) --> L
              |
              v
          zbuffer_test(ooz)
              |
              v
          outbuf[yp*cols + xp] = k_lumi[L * 12]
          zbuf[yp*cols + xp]   = ooz

  outbuf, zbuf
      |
      v
  torus_draw()
      |
      for each non-space cell in outbuf:
          luminance_index --> color_pair
          mvwaddch(y, x, char) with color attr
      |
      v
  screen_present()  -->  terminal
```

## Function Breakdown

### clock_ns() -> int64_t
Purpose: Return current time in nanoseconds using CLOCK_MONOTONIC (not wall clock).
Steps:
  1. Call clock_gettime(CLOCK_MONOTONIC).
  2. Return seconds * 1e9 + nanoseconds.
Edge cases: None; CLOCK_MONOTONIC is always available on POSIX.

### clock_sleep_ns(ns)
Purpose: Sleep for exactly ns nanoseconds.
Steps:
  1. If ns <= 0, return immediately.
  2. Build timespec from ns.
  3. Call nanosleep().

### color_init()
Purpose: Register 8 ncurses color pairs for the grey luminance ramp.
Steps:
  1. Call start_color().
  2. If terminal supports 256 colors: map pairs 1..8 to xterm grey indices {235,238,241,244,247,250,253,255} on black background.
  3. Otherwise: map all 8 pairs to white-on-black (brightness differentiated by A_DIM/A_BOLD later).

### lumi_attr(l) -> attr_t
Purpose: Convert luminance level 0..7 to an ncurses drawing attribute.
Steps:
  1. Clamp l to [0, 7].
  2. Base attribute = COLOR_PAIR(l+1).
  3. If 8-color terminal: add A_DIM for l < 3; add A_BOLD for l >= 6.
  4. Return attribute.

### torus_k1(t) -> float
Purpose: Compute perspective scaling K1 so the torus fills ~42% of the screen.
Steps:
  1. half_w = cols * 0.5; half_h = rows.
  2. target = min(half_w, half_h) * 0.42.
  3. k1 = target * (K2 + R2 + R1) / (R2 + R1).
  4. Return k1 * t->k1_scale.
Edge cases: Must be recalculated whenever terminal size or k1_scale changes.

### torus_init(t, cols, rows)
Purpose: Zero-initialise the Torus struct with default parameters.
Steps:
  1. memset to zero.
  2. Set A=B=0, rot_a=ROT_A_DEFAULT, rot_b=ROT_B_DEFAULT, k1_scale=1, paused=false.
  3. Set cols and rows.

### torus_tick(t, dt_sec)
Purpose: Advance rotation angles by one simulation step.
Steps:
  1. If paused, return.
  2. A += rot_a * dt_sec.
  3. B += rot_b * dt_sec.
Edge cases: Angles grow unboundedly; this is fine because sin/cos are periodic.

### torus_render(t)
Purpose: Fill outbuf and zbuf with the current frame's shaded torus.
Steps:
  1. Compute K1 = torus_k1(t).
  2. memset zbuf to 0, outbuf to space.
  3. Precompute sinA, cosA, sinB, cosB from t->A and t->B.
  4. For theta in [0, 2*pi) stepping THETA_STEP:
     a. Compute costh, sinth.
     b. For phi in [0, 2*pi) stepping PHI_STEP:
        i.   Compute cosph, sinph.
        ii.  cx = R2 + R1 * costh;  cy = R1 * sinth.
        iii. Rotate: apply combined A/B rotation matrix to get world (x, y, z).
        iv.  z += K2  (translate into camera frustum).
        v.   ooz = 1 / z.
        vi.  xp = cols/2 + K1 * ooz * x  (integer).
        vii. yp = rows/2 - K1 * ooz * y * 0.5  (integer; *0.5 = aspect correction).
        viii.Bounds check; skip if off screen.
        ix.  Compute luminance L (dot of normal with light direction).
        x.   If L <= 0: back-facing, skip.
        xi.  If ooz <= zbuf[yp*cols+xp]: farther away, skip.
        xii. zbuf[yp*cols+xp] = ooz.
        xiii.li = (int)(L * 12); outbuf[yp*cols+xp] = k_lumi[li].
Edge cases: Large terminals are handled because buffers are pre-sized to TORUS_MAX_COLS*TORUS_MAX_ROWS.

### torus_draw(t, window)
Purpose: Write the rendered outbuf to an ncurses WINDOW.
Steps:
  1. For each cell in outbuf (row-major order):
     a. If char is space, skip (background is already black).
     b. Find char's position in k_lumi to get luminance index li.
     c. Map li to color band ci = (li * 8) / 12.
     d. Get attr = lumi_attr(ci).
     e. mvwaddch(win, y, x, char) with attr on/off.

### screen_draw(s, t, fps)
Purpose: Compose the full frame: torus + HUD.
Steps:
  1. erase() clears stdscr.
  2. torus_draw(t, stdscr).
  3. Format HUD string: fps, A, B, rot_a.
  4. Draw HUD in top-right corner with luminance pair 5 + A_BOLD.

### app_handle_key(app, ch) -> bool
Purpose: Dispatch a keypress to the correct action. Returns false to quit.
Steps:
  1. q/Q/ESC: return false.
  2. space: toggle paused.
  3. ]: rot_a *= SPEED_SCALE; rot_b *= SPEED_SCALE; clamp to SPEED_MAX.
  4. [: rot_a /= SPEED_SCALE; rot_b /= SPEED_SCALE; clamp to SPEED_MIN.
  5. = or +: k1_scale *= SIZE_SCALE; clamp to SIZE_MAXX.
  6. -: k1_scale /= SIZE_SCALE; clamp to SIZE_MIN.
  7. Default: no-op.

## Pseudocode — Core Loop

```
PROGRAM START:
  register atexit(cleanup)
  register signal(SIGINT, SIGTERM) -> set running=0
  register signal(SIGWINCH) -> set need_resize=1

  screen_init()           -- initscr, noecho, cbreak, curs_set(0), nodelay, color_init
  torus_init(cols, rows)  -- zeroes struct, sets defaults

  frame_time = clock_ns()
  sim_accum = 0

  LOOP while running:

    -- Handle terminal resize
    if need_resize:
      endwin(); refresh(); getmaxyx()
      update torus.cols, torus.rows
      frame_time = clock_ns(); sim_accum = 0

    -- Measure elapsed time
    now = clock_ns()
    dt  = now - frame_time        -- nanoseconds since last iteration
    frame_time = now
    cap dt at 100ms               -- prevents spiral of death on wakeup

    -- Fixed-timestep simulation
    tick_ns = NS_PER_SEC / sim_fps
    dt_sec  = tick_ns / NS_PER_SEC as float
    sim_accum += dt
    WHILE sim_accum >= tick_ns:
      torus_tick(dt_sec)          -- advance A, B by rot_a/b * dt_sec
      sim_accum -= tick_ns

    -- Render the current state
    torus_render(torus)
      -- For each (theta, phi) point on torus surface:
      --   1. Compute surface point: cx=R2+R1*cos(theta), cy=R1*sin(theta)
      --   2. Apply rotation matrix (A around X, B around Z):
      --        x = cx*(cosB*cosph + sinA*sinB*sinph) - cy*cosA*sinB
      --        y = cx*(sinB*cosph - sinA*cosB*sinph) + cy*cosA*cosB
      --        z = K2 + cosA*cx*sinph + cy*sinA
      --   3. ooz = 1/z
      --   4. xp = cols/2 + K1*ooz*x
      --      yp = rows/2 - K1*ooz*y*0.5   (0.5 = aspect ratio fix)
      --   5. L = cosph*costh*sinB - cosA*costh*sinph - sinA*sinth
      --        + cosB*(cosA*sinth - costh*sinA*sinph)
      --   6. Skip if L <= 0 (back-facing) or ooz <= zbuf[idx]
      --   7. zbuf[idx]=ooz; outbuf[idx]=k_lumi[(int)(L*12)]

    -- Update FPS display every 500ms
    fps_accum += dt; frame_count++
    if fps_accum >= 500ms: compute fps_display

    -- Frame rate cap at 60fps
    sleep(16.67ms - time_already_elapsed)

    -- Draw to terminal
    erase()
    FOR each non-space cell in outbuf:
      draw char with color from luminance band
    draw HUD
    doupdate()

    -- Read input (non-blocking)
    ch = getch()
    if ch != ERR: app_handle_key(ch)
```

## Interactions Between Modules

```
main (§6)
 |
 +-- owns: App { Torus (§4), Screen (§5) }
 |
 +-- calls screen_init  (§5)  --> initscr, color_init (§3)
 +-- calls torus_init   (§4)
 |
 +-- each frame:
 |    torus_tick(§4)   reads/writes Torus.A, B
 |    torus_render(§4) reads Torus.{A,B,R1,R2,K2,K1_scale,cols,rows}
 |                     writes Torus.{zbuf, outbuf}
 |    screen_draw(§5)  calls torus_draw(§4) which reads Torus.{outbuf,cols,rows}
 |                     then draws HUD directly into stdscr
 |    screen_present(§5) --> doupdate()
 |
 +-- clock_ns / clock_sleep_ns (§2)  used only in main loop
 |
 +-- lumi_attr (§3)  called from torus_draw and screen_draw
```

Shared state:
- `g_app` is a global App, accessed by signal handlers `on_exit_signal` and
  `on_resize_signal` which only write `running` and `need_resize` (both
  `volatile sig_atomic_t`).
- No other globals. All state passes through the App pointer.
