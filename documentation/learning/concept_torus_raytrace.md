# Pass 1 — torus_raytrace.c: Analytic ray-traced torus (quartic intersection)

## Core Idea

Intersects a ray with a torus by substituting the ray parametrically into the torus
implicit equation, which produces a **quartic** (degree-4 polynomial) in t.  The
smallest positive root gives the front surface hit.

The torus lies in the XZ plane (the ring is horizontal).  The camera is elevated
so both the outer ring surface and the hole through the centre are visible.  The
same object-space ray transform used in cube_raytrace.c handles rotation.

---

## The Torus Implicit Equation

Torus centred at origin, major radius R (ring centre to tube centre), minor radius r
(tube cross-section), ring in XZ plane:

```
(√(x² + z²) − R)² + y² = r²
```

This is the set of all points at distance r from the ring circle of radius R in the
XZ plane.

---

## Deriving the Quartic

Ray: `P(t) = ro + t · rd`,  `|rd| = 1`.

Let:
- `po² = |ro|²` = dot(ro, ro)
- `rod = rd · ro`
- `rxz² = ro.x² + ro.z²`  (XZ projection squared)
- `rdxz_d = rd.x·ro.x + rd.z·ro.z`  (XZ dot product)
- `rdxz² = rd.x² + rd.z²`

Define `C₀ = po² + R² − r²`.

The torus equation expanded and rearranged gives:
```
(|P|² + R² − r²)² = 4R² (P.x² + P.z²)
```

Both sides are polynomials in t.  Left side squared, right side linear in t²:

```
Left  = (t² + 2·rod·t + C₀)²
Right = 4R²(rdxz²·t² + 2·rdxz_d·t + rxz²)
```

Expanding and collecting by power of t:
```
t⁴  +  A·t³  +  B·t²  +  C·t  +  D  =  0

A = 4·rod
B = 4·rod² + 2·C₀ − 4R²·rdxz²
C = 4·rod·C₀ − 8R²·rdxz_d
D = C₀² − 4R²·rxz²
```

This is exact — the quartic perfectly describes all intersections of the ray with
the torus.

---

## Root Finding: Sampling + Bisection

A quartic has at most 4 real roots.  We need the smallest positive one (front face).

**Why not Ferrari's formula?**  Ferrari's analytic quartic solver involves nested
square roots and cube roots.  It is numerically unstable near degenerate cases
(very thin torus, grazing rays) and produces complex branches that need careful
case-handling.  Numerical root finding is simpler, robust, and fast enough for a
terminal renderer.

**Algorithm:**

1. Evaluate the quartic at 256 equally-spaced t values in `[ε, 18]` using
   Horner's method: `f(t) = t(t(t(t + A) + B) + C) + D`.

2. When `f(t₀) × f(t₁) < 0` (sign change), a root lies in `[t₀, t₁]`.

3. Bisect 40 times (~12 decimal digits of precision):
```
for j in 0..40:
    mid  = (lo + hi) / 2
    fmid = eval(mid)
    if f(lo) × fmid < 0: hi = mid
    else:                 lo = mid, f_lo = fmid
```

4. Return `(lo + hi) / 2` as the root.

**Why 256 samples?**  The torus's near and far intersection points on the tube are
separated by ~2r = 0.56.  With a scan step of 18/256 ≈ 0.07, every sign change
is detected.  False negatives (two roots within one step) cannot happen for our
parameter ranges.

**Horner's method:**
```
t⁴ + At³ + Bt² + Ct + D = t(t(t(t + A) + B) + C) + D
```
4 multiplications and 4 additions vs 10 multiplications for naive expansion.

---

## Torus Surface Normal

The outward normal at hit point P is the gradient of the implicit function,
normalized.  For a torus in XZ plane:

```
P_xz    = (P.x, 0, P.z)              ← projection onto ring plane
rho     = |P_xz|                      ← distance from Y axis
ring_pt = (R / rho) × P_xz           ← nearest point on ring centreline
N       = normalize(P − ring_pt)
```

Geometric meaning: the surface normal points away from the nearest point on the ring
circle, which is exactly the "tube axis" direction at that location.

Derivation check: gradient of `(√(x²+z²) − R)² + y²` at P gives components
proportional to `(P − ring_pt)`. ✓

---

## Camera Placement and Torus Orientation

The ring is in the XZ plane (horizontal).  Camera at `(0, CAM_HEIGHT=1.8, −cam_dist)`:
- Elevated by 1.8 above the ring plane → sees both the top of the tube and the hole
- In front on −Z → the nearer part of the ring is at the bottom of the view

```
cam  = (0, 1.8, −3.4)
fwd  = normalize(origin − cam)     ← slightly downward and forward
```

The camera is **fixed**; the torus rotates.  Y rotation `(ROT_Y = 0.40 rad/s)`
spins the ring.  Slow X tilt `(ROT_X = 0.18 rad/s)` gradually tips the ring plane,
revealing the inside of the hole from different angles.

---

## Fresnel Mode on Torus

The Fresnel effect is especially striking on a torus because:
- The outer rim of the tube (facing the camera directly) is dark at head-on angles
- The inner rim of the hole (surface curving away) glows bright
- The tube silhouette edges are bright

This creates a ghost-donut appearance — the torus seems transparent in the middle
and glowing at the edges.

---

## Non-Obvious Decisions

### Why XZ plane instead of XY?
With the ring in XY (the usual mathematical convention), the camera must be
elevated on Y to see the hole — the ring appears nearly edge-on from the standard
view angle.  XZ plane + elevated camera gives a natural "looking down at a ring"
perspective without extreme camera angles.

### Why 256 samples, not adaptive?
Adaptive sampling (finer near expected roots) requires knowing where roots are,
which is what we're finding.  Uniform sampling is simpler, predictable, and fast:
256 multiplications at 4 FLOPs each = 1024 FLOPs/ray.  At 80×24 cells, that's
~2M FLOPs per frame — negligible.

### Why 40 bisection steps?
40 iterations give `(t_max / 2⁴⁰) ≈ 18 / 10¹² ≈ 1.6e-11` precision — well below
floating-point epsilon for our t values.  Fewer iterations visibly coarsen the
silhouette at grazing angles.

### Why not Newton's method for refinement?
Newton's method on a quartic requires evaluating the derivative `4t³ + 3At² + 2Bt + C`.
Bisection is cheaper per iteration (one evaluation vs two) and is unconditionally
convergent within the bracket.  For 40 iterations convergence speed is irrelevant.

---

## From the Source

**Math:** The torus normal formula written in the source as `N = normalise(p − R · normalise(p.xz × (0,1)))` is equivalent to the ring-point subtraction form in the concept file. The `p.xz × (0,1)` term produces the XZ projection of p, giving `normalise(p.xz)` as the direction to the ring centre — the cross-product notation is an alternative way to write the same operation.

**Rendering:** The shading pipeline (Phong + Fresnel) is **identical** to sphere_raytrace.c — the quartic root finder is the only code that is torus-specific. Everything else (lighting, modes, rotation transform) is shared. This makes the torus a clean extension exercise: replace one function, get a new shape.

---

## Key Constants and What Tuning Them Does

| Constant | Default | Effect |
|----------|---------|--------|
| `TORUS_R` | 0.68 | major radius; larger = wider ring, bigger hole |
| `TORUS_r` | 0.28 | minor radius; larger = fatter tube |
| `ROT_Y` | 0.40 rad/s | ring spin speed |
| `ROT_X` | 0.18 rad/s | tilt speed; set to 0 for pure spin |
| `CAM_HEIGHT` | 1.8 | elevation; 0 = edge-on (ring appears as line) |
| `Q_SAMPLES` | 256 | scan density; reduce for speed, increase for grazing safety |
| `Q_BISECT` | 40 | precision; 20 is usually sufficient |
| `Q_T_MAX` | 18 | scan range; must exceed max hit distance |

**Aspect ratio R/r:**  `0.68/0.28 ≈ 2.4`.  Below ~2 the tube touches itself
(self-intersecting torus); above ~4 the tube becomes very thin and fragile-looking.
The 2.4 ratio matches the Interstellar torus aesthetic.

---

## Open Questions

1. The quartic has up to 4 real roots.  The current code returns only the first
   (front surface).  Add logic to find the second root (back of tube) and verify
   it gives the correct back-surface normal.  This enables transparency effects.

2. Implement the exact Durand-Kerner or Ferrari quartic solver and compare
   numerical accuracy against bisection at grazing incidence angles.  Which
   misses roots near the silhouette?

3. A Villarceau circle is a circle traced on a torus at a 45° angle.  Derive
   which rays through the torus (as a function of origin and direction) produce
   a single tangent intersection (disc = 0), and visualise the set of such rays
   as a curve on screen.

4. Extend the XZ-plane normal formula to an arbitrarily oriented torus: the ring
   axis direction is stored as a unit vector `axis`.  Derive the general formula
   for `ring_pt` and `N` in terms of `axis` and `P`.

---

# Pass 2 — torus_raytrace.c: Pseudocode

## Module Map

| Section | Purpose |
|---------|---------|
| §1 config | TORUS_R/r, ROT_Y/X, CAM_*, Q_SAMPLES/BISECT/T_MAX |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 V3/Mat3 | same as cube_raytrace |
| §4 color | 6 themes; 216 pre-init pairs; `draw_color` |
| §5 intersection | `q_eval` (Horner); `ray_torus` (scan+bisect); `torus_normal` |
| §6 shading | `shade_phong`, `shade_normal`, `shade_fresnel`, `shade_depth` |
| §7 render | elevated fixed camera → inverse-transform ray → shade |
| §8 screen | `screen_hud()` |
| §9 main | angle accumulation, input |

## Data Flow

```
each frame:
  angle_y += ROT_Y × dt
  angle_x += ROT_X × dt
  M = mat3_rot(angle_x, angle_y)

  render(cols, rows, angle_x, angle_y, cam_dist, theme, mode):
    cam = (0, CAM_HEIGHT, −cam_dist)
    build (fwd, rgt, up) pointing toward origin

    for each (row, col):
      rd_ws = normalize(fwd + u×rgt + v×up)
      ro_os = mat3_mulT(M, cam)
      rd_os = mat3_mulT(M, rd_ws)

      t = ray_torus(ro_os, rd_os, TORUS_R, TORUS_r)
      if miss: continue

      P_os = ro_os + t×rd_os
      P_ws = cam   + t×rd_ws
      N_os = torus_normal(P_os, TORUS_R)
      N_ws = mat3_mul(M, N_os)
      V    = normalize(cam − P_ws)

      color = shade_*(P_ws, N_ws, V, theme)
      draw_color(row, col, color, lum)
```

## ray_torus

```
C0 = dot(ro,ro) + R² − r²
A  = 4·dot(rd,ro)
B  = 4·dot(rd,ro)² + 2·C0 − 4R²·(rdx²+rdz²)
C  = 4·dot(rd,ro)·C0 − 8R²·(rdx·rox+rdz·roz)
D  = C0² − 4R²·(rox²+roz²)

dt = Q_T_MAX / Q_SAMPLES
t0 = ε,  f0 = q_eval(t0, A,B,C,D)

for i in 1..Q_SAMPLES:
  t1 = i × dt
  f1 = q_eval(t1, A,B,C,D)
  if f0 × f1 < 0:
    bisect [t0,t1] Q_BISECT times → root
    return root
  t0 = t1,  f0 = f1

return miss
```

## q_eval (Horner)

```
f(t) = t·(t·(t·(t + A) + B) + C) + D
```

## torus_normal

```
P_xz    = (P.x, 0, P.z)
rho     = sqrt(P.x² + P.z²)
ring_pt = (R/rho) × P_xz
N       = normalize(P − ring_pt)
```
