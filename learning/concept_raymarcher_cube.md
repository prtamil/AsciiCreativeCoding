# Pass 1 — raymarcher_cube: Raymarched ASCII Spinning Cube

## Core Idea

Same raymarching approach as the sphere, but the SDF is now `sdf_box` — Inigo Quilez's exact box SDF. The cube rotates by rotating the *query point* by the inverse orientation before evaluating the SDF. Surface normals are estimated numerically using the tetrahedron technique (4 SDF samples) because the box SDF has sharp edges where analytical normals are discontinuous.

---

## The Mental Model

Same as sphere: fire a ray per pixel, sphere-trace to the surface using the SDF, shade with Phong. The differences:

1. **SDF**: Instead of `|p| - r`, the box SDF gives exact distance to an axis-aligned cube. Points outside get the distance to the nearest face/edge/corner; points inside get negative distance to the nearest face.

2. **Rotation via query point**: The cube is always defined as an AABB at the origin in the SDF. To make it appear to rotate, we rotate the *query point* by the *inverse* of the desired cube orientation before calling sdf_box. This is the standard "rotate the world, not the object" trick.

3. **Numeric normal**: At sphere-surface, N = normalize(hit) works because the sphere is smooth everywhere. The cube has sharp edges where the SDF gradient jumps discontinuously. We use the tetrahedron technique (4 SDF evaluations at tetrahedral offsets) to estimate the normal numerically — this gives better results than central differences at edges.

---

## Data Structures

Same as raymarcher.c:
- `Canvas` — CELL_W=1, CELL_H=1, character ramp " .,:;+*oxOX#@"
- `Scene` — canvas + time + cube rotation angles (rx, ry) + rotation speeds (rot_x, rot_y) + cube_h + light_spd

---

## The Main Loop

1. `scene_tick(dt_sec)` — rx += rot_x*dt, ry += rot_y*dt, orbit light
2. `canvas_render()` — for each pixel: cast ray, march against sdf_scene(p, rx, ry, cube_h), shade with rm_shade using tetrahedron normal
3. `canvas_draw()` — same block upscale as sphere

---

## Non-Obvious Decisions

### sdf_box — the IQ exact formula
```c
q = |p| - h         // vector to surface in each dimension
outside = |max(q, 0)|    // distance if outside (nearest face/edge/corner)
inside  = min(max(q.x, q.y, q.z), 0)  // distance if inside (nearest face, negative)
return outside + inside
```
This single formula handles all cases: point above a face (one q component > 0), point at an edge (two > 0), point at a corner (all three > 0), and point inside (all ≤ 0).

### Query-point rotation for cube orientation
Instead of building a rotation matrix and applying it to the SDF geometry (which is complex for an implicit surface), we rotate the *query point* by the inverse rotation. For orthogonal rotation matrices, inverse = transpose. So `rm_rotate_yx(p, rx, ry)` applies the rotation that undoes the cube's orientation.

### Tetrahedron normal estimator
4 SDF samples at offsets:
```
k0 = (+e, -e, -e)
k1 = (-e, -e, +e)
k2 = (-e, +e, -e)
k3 = (+e, +e, +e)
```
Normal = normalize(k0*d0 + k1*d1 + k2*d2 + k3*d3)
This is more stable than central differences for sharp-edged shapes.

### Starting t at 0.5 in rm_march
The sphere raymarcher starts at t=0. The cube raymarcher starts at t=0.5 (slightly away from the camera). This prevents false hits at t≈0 when the camera is close to the surface.

### Higher KS for cube
KS=0.65 (vs sphere's 0.55). The cube's flat faces create more dramatic specular highlights on the face pointing toward the light.

---

## State Machine

No state machine. Continuous animation. One boolean: `paused`.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|---|---|---|
| `CUBE_H_DEFAULT` | 0.9 | Cube half-size — radius of the inscribed sphere |
| `ROT_X_DEFAULT` | 0.7 | X-axis tumble speed |
| `ROT_Y_DEFAULT` | 1.1 | Y-axis spin speed |
| `RM_NORM_EPS` | 0.001 | Normal estimation step — smaller = more precise but noisy |
| `KS` | 0.65 | Higher than sphere: flat faces need stronger spec |
| `SHIN` | 50 | Tighter specular spot |

---

## Open Questions for Pass 3

1. Replace tetrahedron normals with central differences — does the quality degrade at edges?
2. Try `sdf_scene = sdf_box - 0.05` (round the cube) — does the SDF remain exact?
3. What happens at very small RM_NORM_EPS (1e-6)? Why does it get noisy?
4. Make the light stationary — rotate the cube faster. Does the specular highlight follow correctly?
5. Add a floor plane: `scene_sdf = min(sdf_box(lp, h), p.y + 2.0)`. Do you get a ground shadow?
