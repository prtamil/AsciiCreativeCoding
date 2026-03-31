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
