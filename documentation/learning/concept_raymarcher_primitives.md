# Pass 1 — raymarcher_primitives: 17 IQ SDF Primitives Viewer

## Core Idea

A raymarching viewer for 17 Inigo Quilez (IQ) SDF primitives. Each primitive is a pure C function that returns the signed distance from a query point to a specific 3D shape. A single primitive fills the screen at a time. Tab cycles through them. The object tumbles on two axes (slow X tilt + fast Y spin) so every face, edge, and vertex is seen over time.

---

## The Mental Model

Each of the 17 primitives is a different SDF recipe — a mathematical formula that can tell you: "how far am I from this shape's surface?" All share the same rendering pipeline: sphere tracing, tetrahedron normals, Phong shading, character ramp output.

What makes this program interesting is the variety of SDFs — they go from trivial (`sdf_sphere = |p| - r`) to moderately complex (`sdf_octahedron`, `sdf_triangle`). Each one illuminates a different SDF construction technique: symmetry folding, dimension reduction (2D SDF extruded to 3D), union of sub-primitives.

The quasi-periodic tumble (ROT_X_RATIO = 0.37, irrational relative to ROT_Y) means the motion never repeats exactly — you always see the shape from a slightly different angle.

---

## Data Structures

### `Prim` struct
| Field | Meaning |
|---|---|
| `name` | Display name (e.g., "sphere", "torus", "cone") |
| `sdf` | Function pointer: `float (*sdf)(V3 p, float s)` |

`s` is the user-controlled size parameter. Each wrapper scales the SDF parameters from `s` to get good proportions.

### `Scene` struct
- `canvas` — same CELL_W=1, CELL_H=1 pattern as sphere/cube
- `prim_idx` — which of 17 primitives is active
- `rot_y` — fast Y spin angle
- `rot_x = rot_y * ROT_X_RATIO` — slow X tilt
- `rot_spd` — rotation speed in rad/s
- `prim_size` — size multiplier

---

## The Main Loop

1. `scene_tick(dt_sec)` — advance rot_y, compute rot_x
2. `canvas_render()` — per pixel: `tumble(p, rot_y, rot_x)` before SDF evaluation
3. `canvas_draw()` — same block upscale as sphere/cube

---

## Non-Obvious Decisions

### Two-axis quasi-periodic tumble
`rot_x = rot_y * ROT_X_RATIO` where `ROT_X_RATIO = 0.37` (irrational). This makes the orbit trace a Lissajous-like path on the sphere of viewing directions that never repeats. Every face, edge, and vertex is visited over time without any manual camera movement.

### Fixed light, tumbling object
The light is always at (-3, 3.5, 2.5) — upper-left of camera. The object rotates, not the light. This means the highlight moves as the object tumbles, showing its shape from different angles with different lighting at each moment.

### Wrapper functions scale `s` to good proportions
Each wrapper maps a single `s` parameter (user size, default 1.0) to the specific parameters of that SDF. For example:
- Box: `sdf_box(p, (s*0.7, s*0.7, s*0.7))` — 70% of s as half-extent
- Torus: `sdf_torus(rot_x(p, π/2), s*0.62, s*0.21)` — ring tilted 90° to face camera

### Flat shapes tilted for initial orientation
Plane/triangle/quad would be edge-on at zero rotation. Their wrappers pre-tilt them ~30° so the face is visible from the start.

### Vec2 alongside Vec3
Many 2.5D SDFs (cone, cylinder, capped torus) work by reducing a 3D point to 2D (e.g., `(sqrt(x²+z²), y)` for revolving shapes) and computing a 2D SDF. The code includes a `V2` type with its own vector math for this.

---

## The 17 SDF Primitives at a Glance

| # | Name | Key Technique |
|---|---|---|
| 1 | Sphere | `|p| - r` (trivial) |
| 2 | Box | Component-wise `|p|-h`, outside+inside |
| 3 | Round Box | Box SDF minus rounding radius `r` |
| 4 | Box Frame | Union of 3 box faces (edge-only slab) |
| 5 | Torus | 2D SDF in revolution (√(√(x²+z²)-R)² + y²) - r |
| 6 | Capped Torus | Torus with angular cap |
| 7 | Link | Chain link: two curved end caps + straight tube |
| 8 | Cone | IQ exact cone SDF, quadratic formula |
| 9 | Plane (disc) | 2D circle extruded to thin disc |
| 10 | Hex Prism | Hexagonal cross-section prism |
| 11 | Capsule | Segment SDF: project point onto segment + sphere radius |
| 12 | Cylinder | 2D round-rect: `(√(x²+z²), y)` vs `(r, h)` |
| 13 | Round Cone | Two-sphere cone with rounded ends |
| 14 | Octahedron | Fold symmetry + Chebyshev |
| 15 | Pyramid | Complex fold with corner/edge cases |
| 16 | Triangle | 2D triangle SDF extruded along Y |
| 17 | Quad | 2D rectangle SDF extruded along Y |

---

## State Machine

No state machine. Continuous tumble animation. One boolean: `paused`.

---

## From the Source

**Algorithm:** SDF primitive gallery — 17 analytical SDFs from Inigo Quilez. Each SDF is an exact (or near-exact) signed distance function for a geometric primitive. Common building blocks:
- Sphere: `sdf = |p| − R`
- Box: `sdf = length(max(|p|−b, 0)) + min(max_component(|p|−b), 0)`
- Torus: `sdf = length(vec2(length(p.xz)−R, p.y)) − r`
- Capsule: `sdf = distance_to_line_segment(p, a, b) − r`

**Math:** SDF composition via smooth-min and Boolean operations — these SDFs are the "atoms" of constructive solid geometry. Any scene can be described as a tree of SDF operations. The SDF of a union is `min(d1, d2)`; the **Lipschitz constant** of `min(f, g)` is `max(Lip(f), Lip(g)) ≤ 1`, so raymarching is still convergent for Boolean combinations — this is the mathematical reason Boolean SDF composition is safe to march without reducing step size.

**Rendering:** Two-axis continuous rotation ensures every face, edge, and vertex is visible without user interaction. All 17 primitives use the **same shading pipeline** — only the SDF function pointer differs between primitives.

---

## Key Constants

| Constant | Default | Effect |
|---|---|---|
| `ROT_Y_SPD_DEFAULT` | 0.60 | Spin speed |
| `ROT_X_RATIO` | 0.37 | X/Y ratio — irrational for quasi-periodic orbit |
| `PRIM_SIZE_DEFAULT` | 1.0 | Scales all SDF parameters uniformly |
| `N_PRIMS` | 17 | Total primitives in table |

---

## Open Questions for Pass 3

1. Add `sdf_smooth_union(d1, d2, k)`: smooth min to blend two primitives. What does it look like when two shapes merge?
2. What happens with `prim_size = 0.01`? Does raymarching still work?
3. Try `sdf_box_frame` with very small frame thickness `e` — does it approach a wireframe cube?
4. Why does sdf_octahedron use `m*0.57735` instead of `m/sqrt(3)`?
5. Compare `sdf_cone` with `sdf_cylinder` — what makes the cone SDF harder?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app.scene.canvas.intensity` | `float*` (heap, cols×rows) | ~varies | Raw per-pixel intensity from the march loop |
| `g_app.scene.canvas.pixels` | `int*` (heap, cols×rows) | ~varies | Per-pixel ramp index after tone mapping (or CANVAS_MISS) |
| `k_prims[N_PRIMS]` | `Prim[17]` | ~varies | SDF function pointer + name + parameter defaults for each primitive |

# Pass 2 — raymarcher_primitives: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | N_PRIMS=17, SIM_FPS_DEFAULT=24, CELL_ASPECT=2.0, RM_MAX_STEPS=100, RM_HIT_EPS=0.002, CAM_Z=4.5, FOV_HALF_TAN=0.65, LIGHT=(-3,3.5,2.5), KA/KD/KS/SHIN, ROT_Y_SPD_DEFAULT=0.60, ROT_X_RATIO=0.37 |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color | 8 luminance grey pairs; `lumi_attr()` |
| §4 vec2/vec3 | `V2`, `V3` + full vector math; `rot_y()`, `rot_x()`, `tumble()` |
| §5 sdf | 17 IQ SDF functions (pure math) |
| §6 wrappers | `Prim` table with name + function pointer; wrapper scales `s` → SDF params |
| §7 raymarch | `rm_cast_pixel()`, `rm_march()`, `rm_normal()`, `rm_shade()` |
| §8 canvas | same as raymarcher.c |
| §9 scene | `Scene` owns canvas + prim_idx + rot_y + rot_spd + prim_size |
| §10 screen | ncurses init/resize/present/HUD |
| §11 app | Main dt loop, input, signals |

---

## Data Flow Diagram

```
scene_tick(dt_sec):
  if paused: return
  rot_y += rot_spd * dt_sec
  rot_x  = rot_y * ROT_X_RATIO

canvas_render():
  for each pixel (px, py):
    intensity = rm_cast_pixel(px, py, prim_idx, prim_size, rot_y, rot_x)
    pixels[py*w+px] = (intensity >= 0) ? round(intensity*(RAMP_N-1)) : MISS

rm_cast_pixel(px, py, prim_idx, s, rot_y, rot_x):
  u, v = NDC coordinates
  phys_aspect = (ch * CELL_ASPECT) / cw
  ro = (0, 0, CAM_Z)
  rd = normalize(u*FOV, v*FOV*phys_aspect, -1)

  // SDF evaluator with rotation applied to query point
  sdf = lambda(p): k_prims[prim_idx].sdf(tumble(p, rot_y, rot_x), s)

  t = rm_march(ro, rd, sdf)
  if t < 0: return -1
  hit = ro + rd*t
  N = rm_normal(hit, sdf)   ← tetrahedron method
  return rm_shade(N, hit, ro, LIGHT)

tumble(p, ry, rx) → V3:
  return rot_x(rot_y(p, ry), rx)
  // Apply Y rotation first (fast spin), then X rotation (slow tilt)

rm_march(ro, rd, sdf) → float t:
  t = 0
  for i in 0..RM_MAX_STEPS-1:
    p = ro + rd*t
    d = sdf(p)
    if d < RM_HIT_EPS: return t
    if t > RM_MAX_DIST: return -1
    t += d
  return -1

rm_normal(p, sdf) → V3 (tetrahedron):
  e = RM_NORM_EPS
  k0=(+e,-e,-e), k1=(-e,-e,+e), k2=(-e,+e,-e), k3=(+e,+e,+e)
  n = k0*sdf(p+k0) + k1*sdf(p+k1) + k2*sdf(p+k2) + k3*sdf(p+k3)
  return normalize(n)
```

---

## Key SDF Implementations

```
sdf_sphere(p, r):
  return v3len(p) - r

sdf_box(p, b):      ← b is half-extents vector (V3)
  q = v3abs(p) - b
  return v3len(v3max0(q)) + min(max(q.x,q.y,q.z), 0)

sdf_round_box(p, b, r):
  return sdf_box(p, b) - r

sdf_torus(p, R, r):
  return v2len(v2(v2len(v2(p.x,p.z))-R, p.y)) - r
  // = sqrt( (sqrt(x²+z²) - R)² + y² ) - r

sdf_cylinder(p, h, r):
  d = v2abs( v2(v2len(v2(p.x,p.z)), p.y) ) - v2(r, h)
  return min(max(d.x,d.y), 0) + v2len(v2max0(d))
  // = 2D rounded-rect SDF in (radial, height) space

sdf_capsule(p, a, b, r):
  pa = p - a; ba = b - a
  h = clamp(dot(pa,ba)/dot(ba,ba), 0, 1)
  return len(pa - ba*h) - r
  // = distance to line segment, offset by r
```

---

## Prim Table Structure

```
Prim k_prims[17] = {
  {"sphere",       w_sphere},
  {"box",          w_box},
  {"round_box",    w_round_box},
  {"box_frame",    w_box_frame},
  {"torus",        w_torus},
  {"capped_torus", w_cap_torus},
  {"link",         w_link},
  {"cone",         w_cone},
  {"plane",        w_plane},
  {"hex_prism",    w_hex_prism},
  {"capsule",      w_capsule},
  {"cylinder",     w_cylinder},
  {"round_cone",   w_round_cone},
  {"octahedron",   w_octahedron},
  {"pyramid",      w_pyramid},
  {"triangle",     w_triangle},
  {"quad",         w_quad},
}

Each wrapper (e.g. w_box):
  static float w_box(V3 p, float s) {
    return sdf_box(p, v3(s*0.70, s*0.70, s*0.70));
  }
```

---

## Pseudocode — Core Loop

```
setup:
  screen_init()
  scene_init(cols, rows)
  frame_time = clock_ns()
  sim_accum = 0

main loop (while running):

  1. resize → realloc canvas, reset accum

  2. dt (capped at 100ms)

  3. physics ticks:
     while sim_accum >= tick_ns:
       scene_tick(dt_sec)   ← rot_y += rot_spd*dt, rot_x = rot_y*RATIO
       sim_accum -= tick_ns

  4. canvas_render()

  5. FPS counter, frame cap

  6. draw:
     erase()
     canvas_draw(canvas, cols, rows)
     HUD: fps, prim name, size, rot_spd
     wnoutrefresh + doupdate

  7. input:
     q/ESC       → quit
     space       → toggle paused
     Tab / t     → prim_idx = (prim_idx+1) % N_PRIMS
     T           → prim_idx = (prim_idx-1+N_PRIMS) % N_PRIMS
     ] / [       → rot_spd *= / /= ROT_SPD_STEP (clamped)
     = or + / -  → prim_size *= / /= PRIM_SIZE_STEP (clamped)
     r / R       → rot_spd *= / /= ROT_SPD_STEP (same as ] / [)
```

---

## Interactions Between Modules

```
App
 └── owns Scene

Scene
 ├── canvas: ramp index or MISS per pixel
 └── canvas_render():
      for each pixel:
        rm_cast_pixel() passes sdf = lambda(p): k_prims[idx].sdf(tumble(p,...), s)
        rm_march() sphere traces with this sdf
        rm_normal() tetrahedron gradient
        rm_shade() fixed Phong (LIGHT is constant)

§6 prim table
 └── 17 function pointers, each wraps one §5 SDF
     wrapper scales s → good-looking proportions for that primitive
```

---

## V2 Dimension Reduction Pattern

```
Many 3D SDFs reduce to 2D for revolving or extruded shapes:

Cylinder (revolved around Y):
  → measure (radial_dist, y) instead of (x, y, z)
  → sdf_2d_rounded_rect(v2(len(v2(p.x,p.z)), p.y), v2(r, h))

Torus (revolved loop):
  → measure (radial_dist - major_R, y)
  → sdf_2d_circle(v2(len_xz - R, y), r)

Capsule (extruded endpoint-to-endpoint):
  → project onto segment
  → subtract tube radius

This pattern: "project to 2D, apply 2D SDF, subtract tube radius"
is how most curved 3D primitives are built.
```
