# Pass 1 — capsule_raytrace.c: Analytic ray-traced capsule

## Core Idea

A capsule is a cylinder capped at each end with a hemisphere.  The ray is
transformed into object space (same inverse-rotation trick as the cube and
torus), then intersected in two stages:

1. **Cylinder body** — quadratic after removing the axial component.
2. **Hemisphere caps** — sphere quadratic at the nearer endpoint if the body
   test misses.

The capsule axis is always Y in object space.  The object tumbles via a
rotation matrix; the camera stays fixed on the Z axis.  This is the simplest
shape that combines two distinct normal types (radial body vs. hemispherical
cap) in one analytic intersection.

---

## Capsule Geometry

Two endpoints in object space:
```
ca = (0, −HALF_H, 0)      ← bottom cap centre
cb = (0, +HALF_H, 0)      ← top cap centre
r  = CAP_R                ← radius shared by body and caps
```

Default values: `HALF_H = 0.65`, `CAP_R = 0.35`.
Total visible height ≈ `2*(HALF_H + CAP_R) = 2.0`.

The capsule is the set of all points within distance `r` from the line segment
`[ca, cb]`:
```
dist(P, segment) ≤ r
```

---

## Ray–Capsule Intersection (Íñigo Quílez method)

Ray: `P(t) = ro + t·rd`, `|rd| = 1`.

Let:
```
ba   = cb − ca          ← axis vector
oa   = ro − ca          ← ray origin relative to bottom cap
baba = dot(ba, ba)      ← |ba|²
bard = dot(ba, rd)      ← axial component of ray direction
baoa = dot(ba, oa)      ← signed axial offset of ray origin
```

**Key idea:** to intersect an infinite cylinder with axis `ba` and radius `r`,
project out the axial component from both the ray origin and direction.  The
residual is a 2D problem in the plane perpendicular to `ba`.

```
a = baba − bard²
b = baba·(rd·oa) − baoa·bard
c = baba·(|oa|² − r²) − baoa²
```

Discriminant: `h = b² − a·c`

- `h < 0`: ray misses the infinite cylinder entirely → skip both caps.
- `h ≥ 0`: candidate body hit at `t = (−b − √h) / a`.

### Body hit condition

```
y = baoa + t·bard        ← dot(ba, P − ca): signed axial position
```

Hit the body if `t > ε` and `0 < y < baba`.  This checks that the hit point
is between the two cap planes.

### Cap fallback

If `y ≤ 0`: try the bottom hemisphere centred at `ca`.  Use `oc = oa` (origin
relative to `ca`).

If `y ≥ baba`: try the top hemisphere centred at `cb`.  Use `oc = oa − ba`
(origin relative to `cb`).

Solve as an ordinary sphere:
```
b' = dot(rd, oc)
c' = dot(oc, oc) − r²
h' = b'² − c'
t  = −b' − √h'
```

Take `t` if positive, else `−b' + √h'`.

---

## Surface Normals

### Body normal

The nearest point on the axis to a surface point `P` is:
```
axis_pt = ca + (y / baba) · ba
N_body  = normalize(P − axis_pt)
        = normalize( (oa + t·rd) − (y/baba)·ba )
```

This strips the axial component from `(P − ca)`, leaving the radially outward
vector.  It is perpendicular to `ba` everywhere on the cylinder.

### Cap normal

`N_cap = normalize(oc + t·rd)`

where `oc` is the ray origin relative to the cap centre.  Equivalent to
`normalize(P − cap_centre)` — the same formula as a standalone sphere normal.

The transition between body and cap normals is `C¹` continuous: at the seam
the body normal and the cap normal are equal (both point radially from the
axis, which at the cap equator equals pointing from the endpoint).

---

## Object Rotation

Same inverse-ray-transform approach used in `cube_raytrace.c` and
`torus_raytrace.c`:

```
ro_os = M^T · ro_ws       ← camera origin in object space
rd_os = M^T · rd_ws       ← ray direction in object space

intersect(ro_os, rd_os)   ← capsule stays axis-aligned in object space

N_ws  = M · N_os          ← normal back to world space
P_ws  = cam + t · rd_ws   ← hit point always in world space
```

For an orthogonal rotation matrix `M = Rx(rx)·Ry(ry)`, `M^T = M^{-1}`.
The capsule never actually moves — only the ray changes frame each frame.

---

## Shading

Same 3-point Phong framework as sphere/cube/torus:

| Light | Position  | Role |
|-------|-----------|------|
| Key   | (3,4,−2)  | warm, primary diffuse + sharp specular |
| Fill  | (−4,1,−1) | cool, lifts shadow side |
| Rim   | (0.5,−1,5)| backlight, separates silhouette |

**Fresnel on a capsule:** especially visible at the cylinder seam.  The flat
band of the cylinder body facing the camera shows the dark core; the rounded
caps and the silhouette edges glow.  The cap–body seam (where normals
transition from radial to hemispherical) creates a subtle bright ring that
matches the geometric crease.

**Normal mode:** body shows gradients in X and Z; caps show hemispherical
gradients.  The cap–body boundary appears as a sudden colour shift because the
normal changes character (radial → spherical) even though the surface is smooth.

---

## Non-Obvious Decisions

### Why decompose into cylinder + sphere caps rather than using the SDF?

The capsule SDF (`max(|P−segment|−r, 0)`) tells you the distance but not the
parametric `t`.  To get `t` from SDF you would need ray marching (hundreds of
steps).  The analytic method gives `t` directly in 2–4 operations — exact, no
iteration.

### Why check `h < 0` before the cap test?

If the infinite cylinder is missed (`h < 0`), the ray cannot possibly hit
either cap.  The caps are inside the cylinder's lateral extent, so any ray
that hits a cap must first pass through the cylinder's "shadow" in the lateral
plane.  Skipping the cap test early saves two square roots on the common
miss path.

### Why `y = baoa + t·bard` and not normalize?

`y` is in units of `|ba|²`, not units of length.  The body-hit condition
`0 < y < baba` is equivalent to `0 < dot(ba,P−ca)/|ba| < |ba|`, which checks
that the axial projection of the hit point is between the endpoints.  No
normalization needed — comparing `y` to `0` and `baba` is exact.

### Why a single `oc` variable for both caps?

`oc = oa` when trying the bottom cap, and `oc = oa − ba` when trying the top.
Both make `oc = ro − cap_centre`, so the sphere quadratic formula is
identical for both.  One code path, two semantics.

### Why ROT_X ≠ ROT_Y?

Different angular speeds on the two axes produce a tumbling motion where no
two-second interval looks the same.  Equal speeds produce a circular
(Lissajous 1:1) orbit that repeats every revolution.

---

## From the Source

**Algorithm:** Analytic ray-capsule intersection — decomposed into two sub-problems: (1) infinite cylinder test (quadratic in t after projecting out the axial component of the ray/capsule vectors); (2) hemisphere cap tests (sphere quadratics at each endpoint). The minimum positive t among valid hits is the surface hit.

**Math:** Cylinder intersection: project ray and cylinder axis onto the plane perpendicular to the axis. The resulting 2D ray-circle problem is a standard quadratic. Axial bounds check: the body t is only valid if the hit point's axial projection falls within `[0, height]`. Cap normals: `(p − endpoint) / r` (sphere normal at each cap).

**Rendering:** Mode cycling (phong / normals / fresnel / depth) shows how the same intersection test feeds different shading algorithms. Normal mode renders N as an RGB vector — a diagnostic tool for verifying correct normal orientation at body/cap boundaries.

---

## Key Constants and What Tuning Them Does

| Constant   | Default   | Effect |
|------------|-----------|--------|
| `CAP_HALF_H` | 0.65    | cylinder body length; 0 = pure sphere, large = long rod |
| `CAP_R`      | 0.35    | tube radius; larger = fatter capsule |
| `ROT_Y`      | 0.45 r/s| spin speed around Y |
| `ROT_X`      | 0.22 r/s| tilt speed; set to 0 for pure spin |
| `SHININESS`  | 48      | specular exponent; cylinder faces show elongated highlights |
| `FOV_DEG`    | 55°     | wider shows more of the capsule at close range |

**Aspect ratio HALF_H / CAP_R:** `0.65 / 0.35 ≈ 1.86`.  Below 1 it looks
squat/pill-like; above 3 it looks like a rod.  The 1.86 ratio gives a
classical pharmaceutical capsule shape that reads clearly at terminal
resolution.

---

## Open Questions

1. The current code returns only the front surface.  Implement a second hit
   (back of body or back of cap) and verify the back-face normal is correct.
   What does the Fresnel effect look like when applied to both surfaces?

2. Derive the condition under which a ray hits both caps without hitting the
   body.  Is this possible?  Sketch the geometry.

3. Extend to an oriented capsule: replace the fixed axis `ba = (0,2·HALF_H,0)`
   with an arbitrary unit vector stored in the object.  How does the `oc`
   formula for cap selection change?

4. Add a second capsule and implement correct depth ordering (closest-hit
   wins).  How do you handle the case where the two capsules overlap?

5. The normal mode shows a visible colour discontinuity at the cap–body seam.
   The surface is geometrically `C¹` (tangent-plane continuous) — so why does
   the normal mode show a jump?  Is the discontinuity in the normal field or in
   the colour mapping?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_themes[]` | `const Theme[6]` | ~288 B | Per-theme diffuse, specular, and three light colour vectors |

# Pass 2 — capsule_raytrace.c: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | CAP_HALF_H, CAP_R, ROT_Y/X, CAM_*, FOV_DEG, SHININESS |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 V3/Mat3 | same as cube_raytrace; `mat3_rot`, `mat3_mul`, `mat3_mulT` |
| §4 color | 6 themes (bronze/frost/ember/pine/dusk/pearl); 216 pre-init pairs |
| §5 intersection | `ray_capsule(ro, rd, ca, cb, r)` → t, N_os |
| §6 shading | `shade_phong`, `shade_normal`, `shade_fresnel`, `shade_depth` |
| §7 render | fixed camera → inverse-transform ray → shade |
| §8 screen | `screen_hud()` |
| §9 main | angle accumulation, input |

## Data Flow

```
each frame:
  angle_y += ROT_Y × dt
  angle_x += ROT_X × dt
  M = mat3_rot(angle_x, angle_y)

  render(cols, rows, angle_x, angle_y, cam_dist, theme, mode):
    cam = (0, 0, −cam_dist)
    fwd = (0,0,1), rgt = (1,0,0), up = (0,1,0)

    ca_os = (0, −HALF_H, 0)
    cb_os = (0, +HALF_H, 0)

    for each (row, col):
      rd_ws = normalize(fwd + u×rgt + v×up)
      ro_os = mat3_mulT(M, cam)
      rd_os = mat3_mulT(M, rd_ws)

      (t, N_os) = ray_capsule(ro_os, rd_os, ca_os, cb_os, CAP_R)
      if miss: continue

      P_ws  = cam + t×rd_ws
      N_ws  = mat3_mul(M, N_os)
      V     = normalize(cam − P_ws)

      color = shade_*(P_ws, N_ws, V, theme)
      draw_color(row, col, color, lum)
```

## ray_capsule

```
ba   = cb − ca
oa   = ro − ca
baba = dot(ba,ba)
bard = dot(ba,rd)
baoa = dot(ba,oa)
rdoa = dot(rd,oa)
oaoa = dot(oa,oa)

a = baba − bard²
b = baba·rdoa − baoa·bard
c = baba·(oaoa − r²) − baoa²
h = b² − a·c

if h < 0: miss

// body
t = (−b − √h) / a
y = baoa + t·bard
if t > ε and 0 < y < baba:
  N = normalize( (oa + t·rd) − (y/baba)·ba )
  return (t, N)

// cap
oc = (y ≤ 0) ? oa : oa − ba
b' = dot(rd, oc)
c' = dot(oc,oc) − r²
h' = b'² − c'
if h' < 0: miss
t  = −b' − √h'
if t < ε: t = −b' + √h'; if t < ε: miss
N  = normalize(oc + t·rd)
return (t, N)
```
