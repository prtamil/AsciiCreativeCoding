# Pass 1 — cube_raytrace.c: Analytic ray-traced cube (AABB slab method)

## Core Idea

Intersects a ray with an axis-aligned bounding box (AABB) using the slab method —
the fastest and most robust algorithm for ray-box intersection.  The cube tumbles
via a rotation matrix; instead of rotating the geometry, the ray is transformed into
object space, intersection is solved there, and the result is transformed back.

The wireframe mode is unique to ray tracing: face edges are detected analytically
from the hit point's coordinates on the face — no rasterization artifacts,
pixel-perfect sharp lines.

---

## The Slab Method

An AABB is the intersection of three pairs of parallel planes ("slabs").  For a box
`[−s, s]³`:

For each axis i:
```
t_near_i = (−s − ro_i) / rd_i      ← entry plane
t_far_i  = ( s − ro_i) / rd_i      ← exit plane
```
If `rd_i < 0`, swap near and far.

```
t_near = max(t_near_x, t_near_y, t_near_z)
t_far  = min(t_far_x,  t_far_y,  t_far_z)

hit if t_near ≤ t_far  AND  t_far > 0
t_hit = t_near > 0 ? t_near : t_far
```

**Why this works:** The ray is inside all three slabs simultaneously only between
`t_near` and `t_far`.  The max of the entry times ensures the ray has entered all
three; the min of exit times ensures it hasn't left any.

**Normal from which axis gave `t_near`:**
The axis that set `t_near` is the one the ray entered last — that's the face it
actually hit.  Sign of the normal: opposite to `rd_i` on that axis (ray comes from
outside, normal points outward).
```
n_sign = (rd_i > 0) ? −1 : +1
```

---

## Object Rotation via Inverse Ray Transform

Instead of building a new mesh each frame, the rotation is applied to the **ray**:

```
ro_os = M^T × ro_ws        ← ray origin in object space
rd_os = M^T × rd_ws        ← ray direction in object space

intersect(ro_os, rd_os)    ← solve in object space (box stays axis-aligned)

N_ws = M × N_os            ← normal back to world space
P_ws = cam + t × rd_ws     ← hit point always in world space
```

For an orthogonal rotation matrix M, `M^T = M^{-1}`.  The construction:
```
M = Rx(angle_x) · Ry(angle_y)

M rows:
  r₀ = (cy,       0,   sy    )
  r₁ = (sx·sy,    cx, −sx·cy )
  r₂ = (−cx·sy,   sx,  cx·cy )

M · v   = (dot(r₀,v), dot(r₁,v), dot(r₂,v))       [object → world]
M^T · v = transpose multiply using columns of M     [world → object]
```

This approach works for any rigid transformation.  The box stays axis-aligned in
object space forever — only the ray changes frame.

---

## Wireframe Mode

After intersection, the hit point `P_os` lies on one face of the AABB.  Two
coordinates are "free" on that face:

| Hit face | Face normal | Free coordinates |
|----------|-------------|-----------------|
| ±X face | (±1, 0, 0) | P_os.y, P_os.z |
| ±Y face | (0, ±1, 0) | P_os.x, P_os.z |
| ±Z face | (0, 0, ±1) | P_os.x, P_os.y |

Edge distance = distance to nearest face boundary in the free coordinates:
```
du = s − |u|,  dv = s − |v|
edge_dist = min(du, dv) / s       ← 0 at edge, 1 at face centre
```

If `edge_dist < WIRE_THRESH (0.055)`: draw edge with face-normal colour.
If `edge_dist ≥ WIRE_THRESH`: skip (interior is transparent).

This produces pixel-perfect lines with no aliasing — analytically clean in a way
rasterization cannot match at terminal resolution.

---

## Shading

Same 3-point Phong framework as sphere_raytrace.c (key + fill + rim).

The cube's flat faces create sharp lighting boundaries — each face is uniformly
lit by any single light, so transitions between faces are abrupt steps.  This is
physically correct and makes the cube look distinctly different from smooth-shaded
objects.

**Normal mode** maps each face to a distinct colour:
- ±X faces: red variants
- ±Y faces: green variants
- ±Z faces: blue variants

Six faces × 6 colours is the most informative view of which face the ray hits.

---

## Non-Obvious Decisions

### Why object-space intersection instead of transforming the box?
Transforming a box by a non-axis-aligned rotation turns it into an oriented bounding
box (OBB).  OBB intersection requires a separate algorithm (separating-axis
theorem).  Transforming the ray and keeping the box axis-aligned preserves the
simple slab method — no new code needed.

### Why track `near_ax` and `near_sg` during slab loop?
The normal is determined by which axis produced `t_near`, which is the face the ray
actually hit.  If you compute the normal after the loop from the hit point
coordinates, you get floating-point noise at face boundaries.  Recording `near_ax`
during the loop is exact.

### Why `t_near > t_far` as the miss test, not `<`?
`t_near == t_far` is a grazing hit — the ray touches exactly one edge.  This should
still render (produces a single pixel line on the edge).  The condition
`t_near > t_far` correctly discards only rays that miss entirely.

### Why not back-face cull?
Ray tracing can trivially cull back faces: `dot(N, rd) > 0` means the ray comes
from inside the face.  But for a solid opaque cube the camera is always outside, so
back faces are never visible — culling adds a branch with no benefit.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|----------|---------|--------|
| `CUBE_S` | 0.80 | half-extent; 1.0 = unit box filling ±1 |
| `WIRE_THRESH` | 0.055 | wireframe line width as fraction of half-extent |
| `ROT_Y` | 0.52 rad/s | Y rotation speed |
| `ROT_X` | 0.35 rad/s | X rotation speed (different rate = tumble, not spin) |
| `SHININESS` | 40 | lower than sphere; cube faces have broader highlights |
| `FOV_DEG` | 55° | wider than sphere to show cube corners |

---

## Open Questions

1. Extend to oriented bounding box (OBB): instead of transforming the ray, add a
   rotation field to the box struct and update the slab algorithm with the
   separating axis theorem.  Compare correctness and performance with the
   inverse-ray approach.

2. Add a second cube.  Two intersections per ray — sort by t, draw closest.  What
   do you need to change to handle occlusion correctly?

3. The wireframe threshold `0.055` is constant in object space.  In screen space,
   this projects to different pixel widths depending on face orientation and
   distance.  Derive a screen-space-constant line width formula.

4. Implement soft shadows: for each lit point, shoot a secondary ray toward each
   light.  If it intersects the box before reaching the light, that point is in
   shadow.  How does this change the shadow boundary appearance vs hard shadows?

---

## From the Source

**Algorithm:** The hit face is identified as whichever axis produced the largest t_near — recorded during the slab loop, not inferred post-hoc from the hit point coordinates (which would cause floating-point noise at face boundaries). The normal sign is determined as `(rd_i > 0) ? −1 : +1` on that axis.

**Rendering:** Wireframe edge condition in face UV space uses an OR test: `|u| > WIRE_THRESH || |v| > WIRE_THRESH` — pixels near *either* free-coordinate boundary are drawn as edges. Depth mode maps hit distance to a **cool → warm colour ramp** (hue-encoded, not greyscale).

---

# Pass 2 — cube_raytrace.c: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | CUBE_S, WIRE_THRESH, ROT_Y/X, CAM_*, FOV_DEG, SHININESS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 V3/Mat3 | V3 ops + `mat3_rot`, `mat3_mul`, `mat3_mulT` |
| §4 color | 6 themes; 216 pre-init pairs; `draw_color` |
| §5 intersection | `ray_aabb(ro, rd, s)` → t, N_os; `face_edge_dist` |
| §6 shading | `shade_phong`, `shade_normal`, `shade_depth` |
| §7 render | fixed camera → inverse-transform ray → shade → output |
| §8 screen | `screen_hud()` |
| §9 main | angle_x/y accumulation, input |

## Data Flow

```
each frame:
  angle_y += ROT_Y × dt
  angle_x += ROT_X × dt
  M = mat3_rot(angle_x, angle_y)

  render(cols, rows, angle_x, angle_y, cam_dist, theme, mode):
    cam = {0, 0, −cam_dist}                    ← fixed camera on Z axis
    ray basis: fwd=(0,0,1), rgt=(1,0,0), up=(0,1,0)

    for each (row, col):
      rd_ws = normalize(fwd + u×rgt + v×up)
      ro_os = mat3_mulT(M, cam)                 ← world → object
      rd_os = mat3_mulT(M, rd_ws)

      (t, N_os) = ray_aabb(ro_os, rd_os, CUBE_S)
      if miss: continue

      P_os = ro_os + t×rd_os
      P_ws = cam   + t×rd_ws
      N_ws = mat3_mul(M, N_os)                  ← object → world
      V    = normalize(cam − P_ws)

      MODE_PHONG:   color = shade_phong(P_ws, N_ws, V, theme)
      MODE_NORMAL:  color = shade_normal(N_ws)
      MODE_WIRE:    ed = face_edge_dist(P_os, N_os, CUBE_S)
                    if ed > WIRE_THRESH: continue
                    color = shade_normal(N_ws)
      MODE_DEPTH:   color = shade_depth(t, cam_dist×2, theme)

      draw_color(row, col, color, lum)
```

## ray_aabb

```
tmin = −∞,  tmax = +∞
near_ax = −1

for i in {x, y, z}:
  if |rd[i]| < ε:
    if ro[i] outside [−s,s]: return miss
    continue
  inv = 1 / rd[i]
  t0 = (−s − ro[i]) × inv
  t1 = ( s − ro[i]) × inv
  tn, tf = sort(t0, t1)
  if tn > tmin:
    tmin = tn
    near_ax = i
    near_sg = rd[i] > 0 ? −1 : +1    ← outward normal sign
  if tf < tmax: tmax = tf
  if tmin > tmax: return miss

if tmax < ε: return miss
t = tmin > ε ? tmin : tmax
N_os = axis_vector(near_ax) × near_sg
return (t, N_os)
```

## face_edge_dist

```
select u,v = free coords on hit face (depends on |N.x|, |N.y|, |N.z|)
du = s − |u|
dv = s − |v|
return min(du, dv) / s
```
