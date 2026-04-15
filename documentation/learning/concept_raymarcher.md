# Pass 1 — raymarcher: Raymarched ASCII Sphere

## Core Idea

A 3D Phong-shaded sphere is rendered using raymarching into a virtual canvas, then upscaled to terminal cells as a CELL_W×CELL_H block. Each canvas pixel fires a single ray that sphere-traces to the nearest surface point, computes Phong shading, and writes a luminance index into a character ramp. This is the simplest possible raymarching demo — one sphere, one SDF, one analytical normal.

---

## The Mental Model

Imagine the terminal as a window looking at a glowing sphere. For every pixel on the screen, you fire a ray from the camera through that pixel into the 3D scene. The sphere is described by a Signed Distance Function (SDF): a formula that tells you how far any point in space is from the sphere surface. You advance the ray by the SDF distance at each step — this is sphere tracing / raymarching. When you get close enough to the surface (distance < epsilon), you compute Phong lighting and convert the brightness to an ASCII character.

The result: a sphere that appears as a gradient of characters from dim (space) to bright (#, @), with a specular highlight that moves as the light orbits.

---

## Data Structures

### `Canvas` struct
| Field | Meaning |
|---|---|
| `w, h` | Canvas dimensions (cols/1, rows/1 = terminal size) |
| `pixels[h*w]` | Intensity index (0..RAMP_N-1) or CANVAS_MISS (-1) for background |

### `Scene` struct
| Field | Meaning |
|---|---|
| `canvas` | The virtual framebuffer |
| `time` | Animation time in seconds |
| `light_spd` | Light orbit speed in radians/second |
| `sphere_r` | Sphere radius |
| `paused` | Animation paused flag |

---

## The Main Loop

1. `scene_tick(dt_sec)` — advance time (if not paused), compute light position
2. `canvas_render(canvas, sphere_r, light)` — fire a ray per pixel, fill pixels[]
3. `canvas_draw(canvas)` — map each pixel to a terminal cell with character + luminance color

---

## Non-Obvious Decisions

### CELL_W=1, CELL_H=1 — full terminal resolution
The sphere raymarcher renders one canvas pixel per terminal cell (unlike the block-upscale approach). This maximizes detail. Aspect correction lives inside the ray direction calculation, not in block size.

### Aspect ratio correction in ray direction
Terminal cells are ~2× taller than wide in physical pixels. If you fire rays on a square grid (u and v both in [-1,1]) the sphere appears squashed vertically. Fix: multiply the vertical ray component by `phys_aspect = (canvas_h * CELL_ASPECT) / canvas_w`. This stretches the vertical field of view to match physical screen space.

### Analytical normal for sphere
For a sphere centered at the origin: `N = normalize(hit_point)`. No numerical gradient estimation needed. This is exact and fast.

### Phong with Blinn reflection (not half-vector)
The sphere uses the classic reflection form: `R = 2*(N·L)*N - L`, then `spec = (R·V)^shininess`. This gives a slightly different specular shape than Blinn-Phong (half-vector method). Both are approximations.

### Light orbits with independent Y oscillation
Light position: `(cos(t)*3, sin(t*0.7)*1 + 1.5, 2.5)`. The Y frequency (0.7) is different from X (1.0), making the light orbit an ellipse that never repeats exactly — so you always see a slightly different view.

---

## State Machines

No state machine. Continuous animation. One boolean: `paused`.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|---|---|---|
| `SPHERE_R_DEFAULT` | 1.1 | Larger = bigger sphere, may clip near edges |
| `RM_MAX_STEPS` | 80 | More = won't miss close surfaces, slower |
| `RM_HIT_EPS` | 0.002 | Smaller = more accurate but more steps needed |
| `CAM_Z` | 4.0 | Camera distance — larger = smaller sphere |
| `FOV_HALF_TAN` | 0.7 | Larger = wider FOV, more perspective distortion |
| `KS` | 0.55 | Specular weight — higher = shinier |
| `SHIN` | 40 | Shininess — higher = tighter specular spot |
| `CELL_ASPECT` | 2.0 | Terminal cell height/width physical ratio |

---

## Open Questions for Pass 3

1. Add a second sphere — add `min(sdf_sphere_a, sdf_sphere_b)` to the scene SDF. What do two spheres look like?
2. Replace the sphere SDF with `sdf_sphere - 0.1*sin(5*theta)*sin(5*phi)` (bumpy sphere). Does the analytical normal still work?
3. Reduce RM_MAX_STEPS to 10 — do you see artifacts where the sphere clips through?
4. Remove the CELL_ASPECT multiplication from the ray — does the sphere look squashed?
5. Try making the light orbit at different rates on X vs Y vs Z — does the lighting feel more interesting?

---

## From the Source

**Algorithm:** Sphere tracing (SDF raymarching) — safe stepping. A ray is cast from the camera through each pixel. At each step: evaluate the scene SDF at the current ray position p. The SDF returns the exact distance to the nearest surface. Advance the ray by this distance. This guarantees no surface overshoot (sphere tracing).

**Math:** For a sphere at origin with radius R: `SDF(p) = |p| − R`. Normal at surface: `N = normalise(∇SDF) = p/|p|` (exact for sphere). Phong shading: `I = kd·(N·L) + ks·(R·V)^n + ka`. The ray terminates when `SDF(p) < HIT_EPS` (surface hit) or total distance exceeds `MAX_DIST` (miss).

**Rendering:** Virtual canvas at low resolution (`VIRT_W × VIRT_H`) is block-upscaled to the terminal — each virtual pixel maps to a small block of terminal cells. This keeps the sphere round despite non-square terminal cells.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app.scene.canvas.pixels` | `int*` (heap, cols×rows) | ~varies | Per-pixel ramp index or CANVAS_MISS for each frame |

# Pass 2 — raymarcher: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | SIM_FPS_DEFAULT=24, CELL_W=1, CELL_H=1, CELL_ASPECT=2.0, RM_MAX_STEPS=80, RM_HIT_EPS=0.002, RM_MAX_DIST=20, CAM_Z=4, FOV_HALF_TAN=0.7, KA/KD/KS/SHIN |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 8 luminance pairs (grey ramp 235..255); `lumi_attr(l)` |
| §4 vec3 | `Vec3` value type; v3add/sub/mul/dot/len/norm |
| §5 raymarch | `sdf_sphere()`, `rm_march()`, `rm_shade()`, `rm_cast_pixel()` — pure math, no ncurses |
| §6 canvas | `Canvas` heap array; `canvas_alloc/render/draw` |
| §7 scene | `Scene` owns canvas + time + light_spd + sphere_r + paused |
| §8 screen | ncurses init/resize/present/HUD |
| §9 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
canvas_render(canvas, sphere_r, light):
  canvas_clear()   ← fill all pixels with CANVAS_MISS (-1)
  for py in 0..h-1:
    for px in 0..w-1:
      intensity = rm_cast_pixel(px, py, w, h, sphere_r, light)
      if intensity < 0:
        canvas.pixels[py*w+px] = CANVAS_MISS   ← background
      else:
        idx = round(intensity * (RAMP_N-1))
        canvas.pixels[py*w+px] = idx

rm_cast_pixel(px, py, cw, ch, sphere_r, light):
  // NDC coordinates
  u =  (px + 0.5) / cw * 2 - 1      ← [-1, +1] horizontal
  v = -(py + 0.5) / ch * 2 + 1      ← [+1, -1] vertical (Y up)

  // Aspect correction — stretch v to match physical screen height
  phys_aspect = (ch * CELL_ASPECT) / cw

  ro = (0, 0, CAM_Z)
  rd = normalize(u * FOV_HALF_TAN,
                 v * FOV_HALF_TAN * phys_aspect,
                 -1.0)

  t = rm_march(ro, rd, sphere_r)
  if t < 0: return -1 (miss)

  hit = ro + rd * t
  return rm_shade(hit, ro, light)

rm_march(ro, rd, r) → float t (or -1):
  t = 0
  for i in 0..RM_MAX_STEPS-1:
    p = ro + rd * t
    d = sdf_sphere(p, r)      ← |p| - r
    if d < RM_HIT_EPS: return t
    if t > RM_MAX_DIST: return -1
    t += d   ← sphere trace: step by SDF distance (guaranteed safe)
  return -1

rm_shade(hit, cam, light) → float [0,1]:
  N = normalize(hit)          ← sphere normal = position (sphere at origin)
  L = normalize(light - hit)
  V = normalize(cam - hit)
  ndl = max(0, N·L)
  R = 2*ndl*N - L             ← reflect L about N
  spec = max(0, R·V) ^ SHIN
  I = KA + KD*ndl + KS*spec
  return min(I, 1.0)

canvas_draw(canvas, term_cols, term_rows):
  // Centre canvas in terminal
  off_x = (term_cols - w*CELL_W) / 2
  off_y = (term_rows - h*CELL_H) / 2
  for vy in 0..h-1:
    for vx in 0..w-1:
      idx = pixels[vy*w+vx]
      if idx == MISS: continue
      ch = k_ramp[idx]            ← " .,:;+*oxOX#@"
      attr = lumi_attr((idx*LUMI_N)/RAMP_N)   ← grey color pair
      // fill CELL_W × CELL_H terminal block for this pixel
      for by in 0..CELL_H-1:
        for bx in 0..CELL_W-1:
          mvaddch(off_y + vy*CELL_H + by, off_x + vx*CELL_W + bx, ch)
```

---

## Function Breakdown

### sdf_sphere(p, r) → float
Purpose: signed distance from point p to unit sphere at origin
Formula: `|p| - r`
- Result > 0: outside sphere
- Result = 0: on surface
- Result < 0: inside sphere

---

### rm_march(ro, rd, r) → float t
Purpose: sphere-trace ray until it hits the sphere or escapes
Steps:
1. t = 0; start at camera origin
2. For i in 0..RM_MAX_STEPS:
   - p = ro + rd * t
   - d = sdf_sphere(p, r)
   - If d < RM_HIT_EPS: return t  (hit)
   - If t > RM_MAX_DIST: return -1 (escaped)
   - t += d  (step by SDF — guaranteed no overshoot)
3. return -1 (max steps exceeded)

---

### rm_shade(hit, cam, light) → float
Purpose: Phong illumination at surface hit point
Steps:
1. N = normalize(hit)  (analytical normal for sphere at origin)
2. L = normalize(light - hit)
3. V = normalize(cam - hit)
4. ndl = max(0, N·L)  (Lambertian diffuse)
5. R = 2*ndl*N - L    (reflected light direction)
6. spec = max(0, R·V)^SHIN
7. I = KA + KD*ndl + KS*spec
8. return clamp(I, 0, 1)

---

### scene_light(scene) → Vec3
Purpose: compute orbiting light position
Formula:
```
t = scene.time * scene.light_spd
light = (cos(t) * 3, sin(t * 0.7) * 1 + 1.5, 2.5)
```
Note: Y frequency = 0.7 (different from X=1.0) → non-repeating elliptical orbit

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize:
     if need_resize:
       screen_resize()
       scene_resize(new cols, rows)   ← realloc canvas
       reset sim_accum, frame_time

  2. dt (capped at 100ms)

  3. physics ticks:
     sim_accum += dt
     while sim_accum >= tick_ns:
       scene_tick(dt_sec)   ← time += dt_sec (if not paused)
       sim_accum -= tick_ns

  4. canvas_render(canvas, sphere_r, scene_light())

  5. FPS counter

  6. frame cap: sleep to 60fps

  7. draw:
     erase()
     canvas_draw(canvas, term_cols, term_rows)
     HUD: fps, sphere_r, light_spd
     wnoutrefresh + doupdate

  8. input:
     q/ESC   → quit
     space   → toggle paused
     ] / [   → light_spd *= / /= LIGHT_SPD_STEP (clamped)
     = or +  → sphere_r *= SIZE_STEP (clamped to SIZE_MAX)
     -       → sphere_r /= SIZE_STEP (clamped to SIZE_MIN)
```

---

## Interactions Between Modules

```
App
 └── owns Scene (canvas + animation state)

Scene
 ├── canvas: CANVAS_MISS = background, idx = character ramp index
 └── canvas_render() calls rm_cast_pixel() per canvas pixel
      → rm_march() sphere traces
      → rm_shade() computes Phong intensity
      → maps intensity → char ramp index

§5 raymarch — pure math
 └── knows nothing about ncurses or terminal
     all terminal logic is in §6 canvas_draw()
```

---

## Sphere Tracing Safety Proof

```
At each step:
  d = sdf_sphere(p, r) = |p| - r

If d > 0: point is outside sphere.
  The sphere surface is exactly d units away.
  Moving t += d along the ray cannot hit the sphere.
  → No overshoot: sphere tracing is always safe for convex SDFs.

If d < RM_HIT_EPS: point is inside or very close to sphere.
  → Declare hit.

Contrast with Newton's method or bisection:
  Sphere tracing requires no derivative and no bracketing.
  Convergence is linear near the surface (each step halves the remaining gap).
```

---

## Character Ramp

```
k_ramp = " .,:;+*oxOX#@"   (13 characters)
RAMP_N = 13

intensity 0.0 → idx 0 → ' '  (background / black)
intensity 0.5 → idx 6 → '*'  (mid brightness)
intensity 1.0 → idx 12 → '@' (specular highlight)

lumi_attr mapping:
  idx * LUMI_N / RAMP_N → color pair (grey ramp 235..255)
  low idx → dark grey, high idx → white
```
