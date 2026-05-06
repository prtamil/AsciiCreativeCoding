# Pass 1 — torus_raytrace.c: Analytic ray-torus via quartic root-finding

## Core Idea

A torus is the set of points at distance r from a horizontal CIRCLE of
radius R. Substituting the parametric ray P(t) = ro + t·rd into the torus
implicit equation `(√(x² + z²) − R)² + y² = r²` and squaring twice (to
remove the square root) collapses into a degree-4 polynomial in t.

Sphere is degree 2; cube is linear per axis. Torus is degree 4 because
its centreline is a CURVE, not a single point or plane.

We solve the quartic numerically (sample + bisect), not by Ferrari's
closed-form formula — the closed form is mathematically beautiful but
notoriously unstable near tangent rays.

## Quartic Coefficients

Substituting and simplifying gives `t⁴ + A·t³ + B·t² + C·t + D = 0` with:

```
let rod = rd · ro
let C0  = |ro|² + R² − r²
A = 4·rod
B = 4·rod² + 2·C0 − 4·R²·(rdx² + rdz²)
C = 4·rod·C0 − 8·R²·(rdx·rox + rdz·roz)
D = C0² − 4·R²·(rox² + roz²)
```

Evaluated efficiently via Horner's method (3 mults + 4 adds vs 6 mults
naïve):

```
q(t) = ((((t + A)·t + B)·t + C)·t + D
```

## Scan + Bisect (numerical solver)

```
1. Sample q(t) at Q_SAMPLES uniformly-spaced t-values in [ε, T_MAX].
2. Wherever consecutive samples have OPPOSITE SIGNS, a real root lies
   in that interval (intermediate value theorem).
3. On the FIRST sign change, bisect inside that bracket Q_BISECT times
   to refine the root to ~10⁻¹² precision.
4. Return that t — it's the smallest positive real root, hence the
   front-face hit.
```

Why scan-bisect over Ferrari?
- Ferrari requires a resolvent cubic + several square roots; tangent
  rays produce coefficients that send the discriminant near zero,
  blowing small input errors into large output errors.
- Scan-bisect never divides by anything potentially zero. Cost ~300 ops
  vs ~50 for Ferrari — but completely stable.

## §5 split into 3 named sub-functions

```
§5.1 q_eval         Horner-form polynomial evaluation
§5.2 ray_torus      derive coefficients + scan-bisect for smallest root
§5.3 torus_normal   closest-point geometric formula
```

## Closest-Point Normal

Outward normal at hit point P (avoiding the implicit gradient's
`1/√(x²+z²)` term):

```
P_xz    = (P.x, 0, P.z)
ρ       = |P_xz|
ring_pt = (R / ρ) · P_xz             ← closest point on ring centreline
N       = normalize(P − ring_pt)
```

If `ρ ≈ 0` (P on Y axis — physically impossible on torus surface, but
defensive), default to (R, 0, 0).

## Worked Example (verify by hand)

Torus R=0.68, r=0.28 in XZ plane. Head-on equatorial ray:

```
ro = (0, 0, -3.4)        rd = (0, 0, +1)

|ro|²    = 11.56
rod      = -3.4
rxz²     = 11.56
rdxz_d   = -3.4
rdxz²    = 1.0
C0       = 11.56 + 0.4624 − 0.0784 = 11.944

A = -13.6
B = 4·11.56 + 2·11.944 − 4·0.4624·1.0 = 68.28
C = 4·(-3.4)·11.944 − 8·0.4624·(-3.4) = -149.9
D = 11.944² − 4·0.4624·11.56          = 121.27
```

Geometrically: with x=y=0, the ray hits the torus at z = ±(R±r):
{−0.96, −0.40, +0.40, +0.96}. Mapping z = −3.4 + t:

```
t1 ≈ 2.44   front-near (z = -0.96)
t2 ≈ 3.00   front-far  (z = -0.40)
t3 ≈ 3.80   back-near  (z = +0.40)
t4 ≈ 4.36   back-far   (z = +0.96)
```

Verify q(2.44) ≈ 0.1 ✓ — it's a root. Solver finds t1 first → smallest
positive front-face hit.

## Shade modes (cycled with `s`)

PHONG · NORMAL · FRESNEL · DEPTH — same set as sphere/cube/capsule.

## §6 lighting split (matches sphere/cube/capsule)

```
§6.1 light_key       KEY warm diffuse + sharp specular
§6.2 light_fill      FILL cool diffuse only
§6.3 light_rim       RIM accent specular at silhouette
§6.4 shade_phong     orchestrator
§6.5 shade_normal    diagnostic — torus shows TWO colour belts (one per tube hemisphere)
§6.6 shade_fresnel   Schlick — both inner and outer silhouette glow
     shade_depth     hit-distance encoding
```

## HUD spec

- Yellow status row 0 + mode label top-left
- Cyan hint bottom row

## Inverse-rotation trick

Torus is fixed in object space (ring in XZ plane, centred at origin) —
this is what keeps the quartic coefficients clean. Rotation is applied
to the RAY by `M^T = M⁻¹`. Cost: ONE matrix×ray multiply per pixel.

The alternative (rotating the torus) would force every coefficient to
be recomputed with cross-axis terms — orders of magnitude more code.

## Edge cases

- **Ray missing through hole** — camera elevated looking at hole centre,
  the polynomial has no real roots; no sign change in the scan window.
- **Tangent ray (double root)** — polynomial dips to zero and returns
  WITHOUT crossing → no sign change → typically lost. Acceptable for
  visual quality; a more robust solver would also find local extrema.
- **Coefficient magnitude** ~200 at typical distances. Single-precision
  float retains ~7 digits → fine for 10⁻⁶ precision after 40 bisections.
- **Scan step size** Q_SAMPLES=256 gives Δt≈0.07. Two roots within 0.07
  can be missed (a near-tangent pass).

## How to verify

- Static torus, no rotation, camera elevated: silhouette is donut shape
  — outer circle of radius R+r=0.96 and inner hole of radius R-r=0.40.
- MODE_NORMAL: rainbow ring with TWO distinct colour belts (one per
  tube hemisphere — top of tube +Y → green, bottom −Y → magenta, etc.).
- MODE_FRESNEL: both inner AND outer silhouette glow brightly.
- Worked example: head-on equatorial ray (ro=(0,0,−3.4), rd=(0,0,1))
  produces 4 roots t ≈ 2.44, 3.00, 3.80, 4.36. Solver returns the
  first one, t ≈ 2.44.

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, FOV, torus R/r, camera, scan params |
| §2 clock        | monotonic timer + sleep |
| §3 math         | V3, Mat3 |
| §4 color        | themes (6) + 256-colour cube + paint |
| §5 torus        | THE CORE — 5.1 q_eval, 5.2 ray_torus, 5.3 torus_normal |
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
                                ray_torus(ro_os, rd_os, R, r):
                                  derive A, B, C, D
                                  scan q(t) for sign change
                                  bisect to refine
                                P_os = ro_os + t·rd_os
                                N_os = torus_normal(P_os, R)
                                N_ws = M · N_os
                                shade by mode → draw_color
```

## Key patterns to internalise

**Higher-degree implicit surfaces solve to higher-degree polynomials.**
Sphere centreline = point → quadratic. Torus centreline = circle →
quartic. The square root in the distance-to-curve formula doubles the
polynomial degree when squared away.

**Numerical solver beats closed-form for stability.** Ferrari's quartic
formula exists but is unstable. Scan + bisect is slower per pixel but
never NaN-s.

**Closest-point normal beats implicit gradient.** The gradient has a
1/√(x²+z²) term; the geometric construction (project to XZ, scale to
R, vector to P) is cheaper and stable.

**Inverse-rotation = cleaner coefficients.** Same trick as sphere/cube/
capsule. Keeping the torus fixed in object space is what makes the
quartic coefficients have closed-form expressions in dot products.
