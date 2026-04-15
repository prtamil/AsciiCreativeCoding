# Pass 1 — donut.c: Trigonometrically-projected spinning ASCII torus

## Core Idea

donut.c renders a spinning torus (donut shape) in the terminal using ASCII
characters and color. The key insight is that it does NOT use raymarching.
Instead, it directly parameterises every point on the torus surface using two
angles, manually computes a rotation, applies a perspective divide, and
scatter-plots those points into a z-buffered 2D grid. The shading comes from a
dot-product formula evaluated analytically at the same time.

This is sometimes called "projection-based" or "forward rendering" — you push
points from 3D space onto the screen — as opposed to raymarching which pulls
rays from the camera outward into the scene.

## The Mental Model

Imagine peeling the surface of a donut into a fine grid of sample points. For
each point you:

1. Know its 3D position in unrotated space from two angles (theta for going
   around the tube cross-section, phi for going around the main ring).
2. Spin the donut by rotating that position using two accumulated angles A (X
   rotation) and B (Z rotation).
3. Project the rotated 3D position onto the 2D terminal screen by dividing
   X and Y by the Z distance (perspective).
4. Compute a brightness for that point by dotting its surface normal with a
   fixed light direction. This is also done analytically using the same angle
   variables — no separate normal-estimation needed.
5. If this point is closer to the camera than any previous point that projected
   to the same terminal cell (z-buffer test), record its character and
   brightness there.

At the end, the terminal grid is a shaded image of the torus. The whole process
runs 30 times per second, with A and B incremented a little each tick, so the
torus appears to spin.

## Data Structures

### `Torus` struct

The central state object. Fields:

- `A`, `B` — current rotation angles in radians. A rotates around the X axis
  (tumble), B around the Z axis (spin). They grow continuously each tick.
- `rot_a`, `rot_b` — how fast A and B grow, in radians per second. The user
  controls these with `]` and `[`.
- `k1_scale` — a user-controlled multiplier on the perspective scaling factor K1.
  Changing it makes the torus appear larger or smaller without altering the
  geometry.
- `paused` — when true, tick() does nothing and A/B freeze.
- `cols`, `rows` — current terminal dimensions. Needed to compute the
  projection centre and to size the buffers.
- `zbuf[TORUS_CELLS]` — flat array of floats, one per terminal cell, storing
  the maximum `1/z` seen so far for that cell. Using `1/z` instead of `z` means
  larger values are closer. Initialised to zero each frame.
- `outbuf[TORUS_CELLS]` — flat array of chars, one per terminal cell, storing
  the ASCII character for the closest torus point found so far. Initialised to
  space each frame.

The buffers are sized to the maximum supported terminal (512x256), which is
131072 cells. This avoids heap allocation and reallocation on resize.

### The luminance ramp `k_lumi[]`

`".,-~:;=!*#$@"` — 12 characters ordered from dimmest to brightest by ink
density. Index 0 is a dot (dim surface), index 11 is @ (very bright surface).
The luminance scalar L (0 to 1) is multiplied by 12 to index into this string.

## The Main Loop

Each iteration of the `while (app->running)` loop does the following in order:

1. **Resize check** — if the terminal was resized (SIGWINCH), update cols/rows
   and reset the accumulator.
2. **Delta time** — record how many nanoseconds have passed since last frame.
   Cap at 100ms to avoid spiral of death after a pause/sleep.
3. **Simulation accumulator** — add dt to a running total. While the total
   exceeds one tick's worth of time, call `torus_tick()` and subtract one tick.
   This decouples simulation rate from render rate.
4. `torus_tick()` — adds `rot_a * dt_sec` to A and `rot_b * dt_sec` to B.
5. `torus_render()` — runs the two nested loops over theta and phi, filling
   `zbuf` and `outbuf`.
6. **FPS counting** — every 500ms, divide frame count by elapsed seconds.
7. **Frame cap sleep** — sleep whatever time remains to hit 60 fps.
8. `screen_draw()` — erases ncurses screen, calls `torus_draw()` to write
   non-space cells, then draws the HUD line.
9. `screen_present()` — flushes ncurses double buffer to the terminal.
10. **Input** — read one keypress with `getch()`, dispatch to `app_handle_key()`.

### Inside `torus_render()`

For each `(theta, phi)` pair sampled at THETA_STEP and PHI_STEP intervals:

- Compute the surface point `(cx, cy)` in 2D cross-section space.
- Apply the A/B rotation matrix (precomputed as sinA, cosA, sinB, cosB for
  efficiency) to get the rotated 3D world position `(x, y, z)`.
- Add K2 (viewer distance) to z. Compute `ooz = 1/z` (one over z).
- Project to screen: `xp = cols/2 + K1 * ooz * x`, `yp = rows/2 - K1 * ooz * y * 0.5`.
  The 0.5 on y compensates for the 2:1 terminal cell aspect ratio.
- Bounds check. If off-screen, skip.
- Compute luminance L using the surface normal dotted with the light direction.
  The light direction is implicitly `(0, 1, -1)` baked into the formula.
  If L <= 0 the surface faces away from the light — skip (back-face cull).
- Z-buffer test: if `ooz <= zbuf[idx]`, a closer point already owns this cell, skip.
- Otherwise update `zbuf[idx] = ooz` and set `outbuf[idx]` to the character
  for this brightness.

## Non-Obvious Decisions

### Why 1/z in the z-buffer?

Standard z-buffers store the actual depth z and keep the minimum (nearest).
Here `ooz = 1/z` is stored and the test keeps the maximum (largest `ooz` = smallest z = nearest).
This is mathematically equivalent but also happens to be the value you already computed for perspective division (`K1 * ooz * x`), so no extra division is needed.

### Why theta and phi angle stepping instead of a uniform mesh?

The torus is parameterised naturally by two angles. Stepping them uniformly is
the simplest approach. PHI_STEP (0.02) is much smaller than THETA_STEP (0.07)
because the phi loop generates points along the outer circumference, which is
larger than the tube cross-section (R2 > R1). Roughly, you need PHI points
proportional to 2*pi*R2 and THETA points proportional to 2*pi*R1, and R2/R1 =
2/1, so PHI needs about twice the density.

### Why the y-component gets multiplied by 0.5 in projection?

Terminal cells are approximately twice as tall as they are wide in physical
pixels. If you projected without correction, the torus would look squashed into
an ellipse. Halving the projected y coordinate stretches it back to a circle.

### Why K1 depends on terminal size?

K1 is the perspective scaling factor. If it were fixed, the torus would appear
small on a large terminal and large on a small one. The formula:

    K1 = 0.42 * min(cols/2, rows) * (K2 + R2 + R1) / (R2 + R1)

is derived from requiring the outermost ring of the torus to project to 42% of
the smaller terminal dimension. When the terminal is resized (SIGWINCH), the
cols and rows in the Torus struct are updated and K1 is recalculated each frame
via `torus_k1()`.

### Why precompute sin/cos of A and B outside the loops?

Every surface point in the double loop uses sinA, cosA, sinB, cosB. These are
expensive transcendental functions. Calling them once before the loops instead
of once per surface point is a significant performance win. The inner
trigonometric calls (sin/cos of theta and phi) cannot be precomputed the same
way because they vary with the loop variable.

### Analytical luminance formula

The luminance L is computed as:

    L = cosph*costh*sinB - cosA*costh*sinph - sinA*sinth
      + cosB*(cosA*sinth - costh*sinA*sinph)

This is the dot product of the torus surface normal (in world space after
rotation by A and B) with the light direction `(0, 1, -1)` normalised. The
derivation involves cross-multiplying the rotation matrix with the analytic
normal formula. It is evaluated at zero cost because all terms are already
computed for the position.

### The color system

The 12-character luminance ramp is split into 8 color bands (`LUMI_LEVELS`).
In 256-color terminals, each band maps to a different grey in the xterm grey
ramp (indices 235–255). In 8-color terminals, the same bands map to the same
white pair but with A_DIM / normal / A_BOLD attributes to simulate three levels.
The character choice AND the color both carry brightness information, giving
smooth shading even on limited terminals.

## State Machines

There is one effective state machine: the `paused` flag in the Torus struct.

```
         space keypress
[RUNNING] ──────────────> [PAUSED]
    ^                         |
    |      space keypress     |
    +─────────────────────────+

In RUNNING: torus_tick() increments A and B each tick.
In PAUSED:  torus_tick() returns immediately. Rendering still happens,
            so the last frame remains visible.
```

Signal states:

```
[ALIVE] ──SIGINT/SIGTERM──> sets running=0 ──> [EXITING]
[ALIVE] ──SIGWINCH──> sets need_resize=1 ──> handled next frame start
```

## Key Constants and What Tuning Them Does

| Constant | Default | Effect of changing |
|---|---|---|
| `ROT_A_DEFAULT` | 1.2 rad/s | Faster/slower tumble around X axis |
| `ROT_B_DEFAULT` | 0.6 rad/s | Faster/slower spin around Z axis |
| `TORUS_R1` | 1.0 | Tube gets fatter/thinner. Too large relative to R2 = torus self-intersects |
| `TORUS_R2` | 2.0 | Ring radius. Smaller = fatter donut hole. R2 must exceed R1 |
| `TORUS_K2` | 5.0 | Viewer distance. Larger = more orthographic (less perspective distortion). Smaller = fisheye |
| `THETA_STEP` | 0.07 | Larger = fewer cross-section samples = visible gaps in tube. Smaller = smoother but slower |
| `PHI_STEP` | 0.02 | Larger = visible gaps along the ring. Should be roughly THETA_STEP * (R1/R2) or smaller |
| `SIZE_SCALE` | 1.15 | How much each = / - keypress changes the size. Closer to 1.0 = finer control |
| `SIM_FPS_DEFAULT` | 30 | Tick rate. Does NOT change visual speed (dt-based), only smoothness of rotation |

## Open Questions for Pass 3

- The rotation is encoded as two sequential Euler-angle rotations (X then Z).
  Euler angles suffer from gimbal lock. Does the donut ever exhibit gimbal lock
  at certain A/B combinations? Try to find a combination where one axis of
  rotation appears frozen.
- The z-buffer is reset to 0 each frame, not to a large negative number. This
  works because ooz is always positive (viewer is never behind the torus), but
  what would break if the torus moved to z < 0?
- Why is THETA_STEP larger than PHI_STEP when both sample the same angular
  range [0, 2*pi]? Prove to yourself that the density is roughly correct by
  computing the arc length each step covers.
- The frame cap targets 60 fps but SIM_FPS_DEFAULT is 30. This means rendering
  and simulation are intentionally decoupled. What would you need to add to
  interpolate between simulation ticks for smoother visual motion at 60fps?
- The luminance formula is computed per-sample but the light direction is
  fixed. What would you need to change to allow the user to move the light
  with keyboard keys?

---

## From the Source

**Algorithm:** Analytic torus rasterization (not raymarching). For each point on the torus surface, the 3D position is computed from (θ, φ) parameters, rotated by two Euler angles, then projected to screen space. The depth buffer (z-buffer) resolves visibility between overlapping surface points.

**Math:** Torus parametric equation:
```
x = (R + r·cos φ) cos θ
y = (R + r·cos φ) sin θ
z =  r · sin φ
```
Surface normal: `n = (cos φ cos θ, cos φ sin θ, sin φ)`. Lighting: L·N dot product with Phong specular: `(R·V)^shininess`. Luminance mapped to ASCII ramp `".,:;+=xX$&@#"`.

**Rendering:** Z-buffer (depth buffer): each pixel stores the depth of the nearest surface point. Rasterization proceeds by scanning (θ, φ) — further points overwrite only if closer than current stored depth.

**Performance:** O(N_THETA × N_PHI) per frame — proportional to the number of surface samples, not terminal resolution. Terminal aspect (CELL_H / CELL_W ≈ 2) compensates in the projection.

**References:** Original algorithm by Andy Sloane (donut.c / a1k0n.net); this file is a rewrite in the ncurses dt-based framework.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app.torus.zbuf[TORUS_CELLS]` | `float[512×256]` | ~512 KB | Per-cell reciprocal depth (1/z) for visibility resolution |
| `g_app.torus.outbuf[TORUS_CELLS]` | `char[512×256]` | ~128 KB | Per-cell ASCII luminance character for blitting to stdscr |

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
