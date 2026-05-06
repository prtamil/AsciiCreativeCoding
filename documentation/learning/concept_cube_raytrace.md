# Pass 1 — cube_raytrace.c: Analytic ray-traced cube via the slab method

## Core Idea

Where sphere uses a quadratic, cube uses a CHAIN OF 1D INTERVAL TESTS.
A box is the intersection of three pairs of parallel planes ("slabs").
For each axis, find the time interval the ray is between the two walls.
The ray is INSIDE the box only during the overlap of all three intervals.

Three 1D problems combined by max-of-entries / min-of-exits — interval
overlap is two arithmetic comparisons. No square roots.

## Slab Method

For axis i (X, Y, or Z) with box span [-s, +s] on that axis:

```
slab_test(ro_i, rd_i, s):
  if |rd_i| ≈ 0:
    if ro_i in [-s, +s] → slab imposes no t-bound (ray runs parallel & inside)
    else                → permanent miss (ray runs parallel & outside)
  else:
    t0 = (-s - ro_i) / rd_i
    t1 = (+s - ro_i) / rd_i
    return tn = min(t0, t1), tf = max(t0, t1), enter_sign = -sign(rd_i)
```

Combining three slabs:

```
ray_aabb:
  tmin = -∞, tmax = +∞, near_axis = -1
  for each axis i ∈ {0, 1, 2}:
    (tn, tf, esign) = slab_test(ro_i, rd_i, s)
    if tn > tmin:
      tmin = tn
      near_axis = i
      near_sign = esign
    tmax = min(tmax, tf)
    if tmin > tmax: return MISS
  if tmax < ε: return MISS                  (whole interval behind us)
  if tmin > ε:
    t = tmin                                (ordinary entry hit)
  else:
    t = tmax                                (camera INSIDE box → exit hit)
  N = ±axis_basis[near_axis]                with the recorded sign
```

The face that produced `tmin` is the entry face. Its normal is the basis
vector of that axis with sign chosen to face the incoming ray.

## §5 split into 3 named sub-functions

```
§5.1 slab_test         single-axis 1D slab solve (atomic unit)
§5.2 ray_aabb          dispatcher: combine 3 slabs into hit decision
§5.3 face_edge_dist    fraction-to-edge for wireframe mode
```

## Shade modes (cycled with `s`)

| Mode | Visual |
|---|---|
| PHONG       | Three-point Phong (KEY + FILL + RIM) |
| NORMAL      | RGB-encoded normal — each face is one flat colour |
| WIREFRAME   | Only edge cells render; face interior skipped |
| DEPTH       | Closer = brighter |

## §6 lighting split (matches sphere/capsule/torus)

```
§6.1 light_key       KEY warm diffuse + sharp specular
§6.2 light_fill      FILL cool diffuse only
§6.3 light_rim       RIM accent specular at silhouette
§6.4 shade_phong     orchestrator
§6.5 shade_normal    diagnostic
§6.6 shade_wire      wireframe brightness (inverse distance to edge)
     shade_depth     hit-distance encoding
```

## Worked Example (verify by hand)

Cube [-0.8, +0.8]³. Camera at (0, 0, -3.2), ray rd = (0, 0, +1) (head-on):

```
X-slab: rd.x = 0  AND  ro.x = 0 ∈ [-0.8, 0.8]    → no t-bound
Y-slab: rd.y = 0  AND  ro.y = 0 ∈ [-0.8, 0.8]    → no t-bound
Z-slab: t0 = (-0.8 - (-3.2))/1 = +2.4
        t1 = (+0.8 - (-3.2))/1 = +4.0
        tn_z = 2.4, tf_z = 4.0

tmin = 2.4 (from Z),  tmax = 4.0 (from Z)
tmin ≤ tmax ✓ and tmax > 0 ✓ → HIT at t = 2.4
Setting axis = Z; sign(rd.z) = +1 → outward normal sign = -1
⇒ N_obj = (0, 0, -1)        (front face, faces camera) ✓
```

## HUD spec

- Yellow status row 0 (fps, dist, theme, run/paused) + mode label top-left
- Cyan hint bottom row

## Inverse-rotation trick

Cube is fixed AABB at origin in object space (so slab tests use simple
plane coordinates). Rotation is applied to the RAY by `M^T = M⁻¹`. Costs
ONE matrix×ray multiply per pixel; the alternative (rotating 8 vertices +
6 face equations) would be vastly more expensive AND would force the
slab method to use general-plane equations (3 dot-products per slab
instead of 1 scalar). Keeping the box AABB in object space lets us
divide instead of dot.

## Edge cases

- **rd_i ≈ 0** (parallel to slab pair): slab_test handles this — if origin
  is inside [-s, +s], slab imposes no bound; else permanent miss.
- **Inside the box** (`tmin < 0 < tmax`): return EXIT face normal
  (`sign(rd_i)`), not entry. Code recovers this by re-scanning slab
  contributions for the axis whose `tf` matches `tmax`.
- **Corner / edge ambiguity**: two slabs produce equal tn — the dispatcher
  picks one (whichever's tn was strictly larger). On terminal grid, the
  discontinuity is invisible because corner pixels are single cells.
- **Tone-map at paint**: same as sphere — quantising linear HDR puts
  every pixel in one cube cell. Reinhard inside paint_cell first.

## How to verify

- Static cube head-on: silhouette is a SQUARE (one visible face). NORMAL
  mode paints it one flat colour: (0,0,−1) face → (0.5, 0.5, 0).
- Cube viewed from a corner (rotated): silhouette is HEXAGON; three
  visible faces meet at the central vertex forming a tri-radial Y.
  NORMAL mode shows three different flat colours.
- Worked example: head-on ray to a 1.6-unit cube at distance 3.2 hits at
  t = 2.4. Compare with code output.
- WIREFRAME shows 9 visible edges (3 per visible face).

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, FOV, cube half-extent, camera, wire threshold |
| §2 clock        | monotonic timer + sleep |
| §3 math         | V3, Mat3 (with inverse-rotation explanation) |
| §4 color        | themes (6) + 256-colour cube + paint |
| §5 cube         | THE CORE — slab method (5.1 single, 5.2 dispatch, 5.3 wire) |
| §6 shading      | KEY/FILL/RIM split + 4 modes |
| §7 render       | inverse-rotation, ray-per-cell |
| §8 screen + HUD | yellow status + cyan hint |
| §9 app          | signals, resize, main loop |

## Data flow

```
keys → main → angle_x, angle_y, theme, mode, paused, cam_dist
                              │
                            render
                              per cell:
                                M = mat3_rot(angle_x, angle_y)
                                rd_ws = camera_ray
                                ro_os = M^T·cam; rd_os = M^T·rd_ws
                                ray_aabb(ro_os, rd_os, s) → t, N_os
                                P_os = ro_os + t·rd_os
                                N_ws = M·N_os
                                V_dir = normalize(cam − P_ws)
                                shade by mode → draw_color
```

## Key patterns to internalise

**Slab method = 3 one-axis interval problems.** Each is trivially
solvable; combining them is just max/min of the entry/exit bounds.

**Inverse-rotation = transform RAY not GEOMETRY.** Same trick as sphere/
capsule/torus. Keeps intersection math in its simplest possible form.

**Constraint by construction.** The "axis-aligned" property is encoded in
the slab math itself; we don't need a general plane equation.

**Same paint pipeline as the entire folder.** RGB → Reinhard → gamma →
6×6×6 cube + 92-char ramp.
