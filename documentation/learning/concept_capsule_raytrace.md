# Pass 1 — capsule_raytrace.c: Analytic ray-capsule via decomposition

## Core Idea

A capsule is the set of points within distance r of a finite line segment AB.
A point lies on the surface iff the closest point on segment AB to it is
exactly radius r away.

To ray-test a capsule, DECOMPOSE the surface into two parts:

1. **Cylinder body** — points where the closest segment-point is INTERIOR
   to AB. A 2D ray-circle problem after projecting out the axial component.
2. **Hemisphere cap** — points where the closest segment-point is endpoint
   A or B. A standard sphere quadratic at that endpoint.

Solve cylinder first; if hit lies axially OUTSIDE [0, |AB|²], retry the
appropriate end-cap. At most ONE sphere test is ever needed.

## §5 split into 3 named sub-functions

```
§5.1 cylinder_test    infinite cylinder along ba + axial bound check
§5.2 cap_test         single sphere quadratic at endpoint
§5.3 ray_capsule      dispatcher — cylinder first, fall through to nearer cap
```

## Cylinder Quadratic (Quílez form, no division)

With ba = B − A, oa = ro − A, baba = |ba|²:

```
a = baba − (ba·rd)²                ← ray length² minus axial projection²
b = baba·(rd·oa) − (ba·oa)·(ba·rd)
c = baba·(|oa|² − r²) − (ba·oa)²
disc = b² − a·c
t_body = (-b - √disc) / a
y_body = (ba·oa) + t_body·(ba·rd)   ← signed axial coordinate, range [0, baba]
```

If `0 < y_body < baba` → BODY HIT. Normal: P − A minus axial projection,
normalised:

```
N_body = normalize((oa + t·rd) − (y/baba)·ba)
```

If hit is axially out of range, dispatcher tries the appropriate cap as a
sphere at that endpoint.

## Cap Sphere Quadratic

Caller passes oc = ro − cap_centre. Standard ray-sphere:

```
b' = rd · oc
c' = oc · oc − r²
disc' = b'² − c'
t = -b' − √disc'                    (front face)
N_cap = normalize(oc + t·rd)        (away from cap centre)
```

## Worked Example (verify by hand)

Capsule A=(0, −0.65, 0), B=(0, +0.65, 0), r=0.35.
Camera ro=(0, 0, −3.4), rd=(0, 0, +1) (head-on along +Z).

```
ba   = (0, 1.30, 0)         baba = 1.69
oa   = (0, 0.65, -3.4)       oaoa = 11.98
bard = ba·rd = 0
baoa = ba·oa = 0.845
rdoa = rd·oa = -3.4

a = 1.69 − 0          = 1.69
b = 1.69·(-3.4) − 0   = -5.746
c = 1.69·(11.98 − 0.1225) − 0.714 = 19.33
h = 33.02 − 32.67     = 0.35       ← > 0, ray hits the band

t = (5.746 − 0.59)/1.69 ≈ 3.05
y = 0.845 + 3.05·0    = 0.845      ← 0 < y < 1.69, BODY HIT
P_obj = ro + t·rd = (0, 0, -0.35)  ← lies on the front of the tube
N_body = normalize((0, 0.65, -0.35) − (0.845/1.69)·(0,1.3,0))
       = normalize((0, 0, -0.35)) = (0, 0, -1)  ← faces camera ✓
```

## Shade modes (cycled with `s`)

PHONG · NORMAL · FRESNEL · DEPTH — same set as sphere/cube/torus.

## §6 lighting split (matches sphere/cube/torus)

```
§6.1 light_key       KEY warm diffuse + sharp specular
§6.2 light_fill      FILL cool diffuse only
§6.3 light_rim       RIM accent specular at silhouette
§6.4 shade_phong     orchestrator
§6.5 shade_normal    diagnostic
§6.6 shade_fresnel   Schlick
     shade_depth     hit-distance encoding
```

## HUD spec

- Yellow status row 0 + mode label top-left
- Cyan hint bottom row

## Edge cases

- **Ray parallel to axis** (`bard² → baba`, `a → 0`): cylinder formula
  divides by zero. Mathematically the ray sees only caps then; in practice
  rays exactly parallel are zero measure for a rotating capsule.
- **t > 1e-4 epsilon**: avoids self-intersection at t≈0.
- **Body/cap normal continuity**: at y=0 (and y=baba), the body normal
  and cap normal AGREE — both are radially outward from the same circle.
  No discontinuity at the seam.
- **Inside-out cap**: cap_test falls back from t = -b' − √disc to
  t = -b' + √disc when the front face is behind the camera.
- **Cylinder full miss + uninitialised y**: when h<0 the cylinder fully
  misses; ray_capsule defaults `y = 0` so the cap-test fall-through
  picks cap A. cap_test will return 0 (a ray that misses the cylinder
  of radius r can't hit a cap of the same radius), so the overall
  result is correctly MISS.

## How to verify

- A static capsule viewed head-on along its axis looks like a DISC (the
  front cap silhouette), not a pill — the cylinder is end-on.
- A capsule viewed perpendicular to its axis shows the classic pill
  silhouette: rectangle of width 2r flanked by two arcs of radius r.
- Worked example: head-on ray to unit-tall capsule at distance 3.4
  should hit at t ≈ 3.05 with N=(0,0,−1). Verify code output matches.
- MODE_FRESNEL on the cylinder band: the band is dark head-on, bright at
  the silhouette edge — strong fresnel transition because the band's
  normal sweeps through 0..90° relative to the view ray.

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, FOV, capsule half-height + radius, camera, ramp |
| §2 clock        | monotonic timer + sleep |
| §3 math         | V3, Mat3 (with inverse-rotation explanation) |
| §4 color        | themes (6) + 256-colour cube + paint |
| §5 capsule      | THE CORE — 5.1 cylinder, 5.2 cap, 5.3 dispatcher |
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
                                M = mat3_rot
                                ro_os = M^T·cam; rd_os = M^T·rd_ws
                                ray_capsule(ro_os, rd_os, A, B, r) → t, N_os
                                  cylinder_test (Quílez form)
                                  if axial out of range:
                                    pick cap A or B by sign(y)
                                    cap_test (sphere quadratic)
                                P_ws = cam + t·rd_ws
                                N_ws = M·N_os
                                shade by mode → draw_color
```

## Key patterns to internalise

**Decomposition wins.** A capsule is a sphere swept along a line —
solving it as "cylinder OR sphere" is much simpler than a single
combined equation.

**Quílez form (no division).** Keeping coefficients multiplied by `baba`
instead of dividing by it lets the projection-onto-plane-perpendicular
math run with no expensive `1 / sqrt(baba)`.

**Body normal = radial outward.** Subtract axial component from (P−A)
to get the perpendicular component → the cylinder normal.

**Same paint + lighting pipeline as the entire folder.** RGB → Reinhard
→ gamma → 6×6×6 cube + 92-char ramp.
