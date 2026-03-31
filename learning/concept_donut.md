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
