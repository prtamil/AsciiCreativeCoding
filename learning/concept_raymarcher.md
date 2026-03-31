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
