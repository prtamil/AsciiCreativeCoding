# Concept — `saturn_with_rings.c`: Sphere + Annular Ring with Shadow Cast

## Core Idea

For every screen cell, fire ONE view ray. Two things might be in the way: a SPHERE (the planet) or a flat ANNULUS (the rings, defined as the y=0 plane clipped to a radial band). Whichever is closer wins; the other is hidden behind it. To know if a point on the ring is in the planet's shadow, fire a SECOND ray from that ring point toward the sun and check whether the sphere is in the way. **That is all the math** — everything else is colour and glyph mapping.

The visual cues that scream "3-D" come from the algorithm itself:
1. The ring **passes BEHIND** the planet (depth-sort occlusion).
2. The planet casts a **dark SHADOW BAND** on the back of the ring (shadow ray hits sphere).

---

## The Mental Model

Picture a chocolate-coated marble (the planet) sitting on a paper plate (the ring), photographed from one corner of the room with a flashlight (the sun) on the other side:

- The marble's lit half faces the flashlight; the other half is dark — that's the **Lambertian terminator**.
- The marble blocks light from reaching the far side of the paper plate, casting a dark stripe — that's the **shadow ray hitting the sphere**.
- The plate dips behind the marble where the marble is between the camera and the back of the plate — that's the **depth sort**. Ring in front: ring wins. Sphere in front: sphere wins.

We do this test for every pixel.

---

## Algorithm in Steps

1. **CAMERA.** Place camera at `(0, CAM_HEIGHT, -CAM_DIST)` looking at origin. Compute basis `(forward, right, up)`. The slight upward offset gives the rings their characteristic ELLIPSE shape on screen (rings are circles in the y=0 plane, foreshortened).

2. **PER CELL `(sx, sy)`:**
   - Compute view-ray direction:
     ```
     u   = (2·sx + 1 − cols) / cols · fov_h
     v   = −(2·sy + 1 − rows) / rows · fov_v
     dir = normalize(forward + u·right + v·up)
     ```
   - Ray-sphere intersection (planet) → `t_sphere`.
   - Ray-plane intersection at `y=0`, clipped to annulus `[R_IN, R_OUT]` → `t_ring`.
   - **Depth sort**: smaller t wins.

3. **SHADE THE WINNER:**
   - **Sphere**: `N = normalize(P − O)`; `lambert = max(0, N · sun_dir)`; apply latitude bands (SATURN, EXO) or fBm continent map (RINGED-EARTH).
   - **Ring**: per-radius density modulation + Cassini Division dimming + **shadow ray** to sun tested against sphere → dark stripe across rings.

4. **NO HIT.** Render space cell — dark background, optional hash-gated star.

5. **INTENSITY → GLYPH.** Map intensity ∈ [0, 1] to `RAMP_GLYPHS[(int)(I·8)]`; pick theme ramp index by primitive (planet ramp slots vs ring ramp slots).

6. Sun rotates with `2π / 30 s` azimuth; the same `sun_dir` drives both the planet's terminator AND the ring's shadow band, so they stay perfectly synchronised.

---

## Key Formulas

**Sun direction** (azimuth ω·t, fixed elevation):
```
ω        = 2π / ROTATION_PERIOD             // 30 s default
sun_az   = ω · t + φ
sun_dir  = normalize( (cos sun_az, SUN_ELEV_Y, sin sun_az) )
```

**Ray-sphere intersection** (sphere at `center` with radius `r`):
```
oc    = ray.origin − center
b     = oc · ray.dir
c     = oc·oc − r²
disc  = b² − c
if disc < 0:        miss
t     = −b − √disc                          // front face
if t < ε: t = −b + √disc                    // try far face
if t < ε:           miss
```

**Ray-plane annulus intersection** (plane y = 0):
```
if |dir.y| < ε:                miss         // parallel
t  = −origin.y / dir.y
if t < ε:                      miss
hit = origin + t·dir
r² = hit.x² + hit.z²
if r² < R_IN² or r² > R_OUT²:  miss
```

**Lambert (diffuse) on sphere:**
```
N        = normalize(P − O)
lambert  = max(0, N · sun_dir)
shade    = AMBIENT + (1 − AMBIENT) · lambert
```

**Latitude bands** (SATURN / EXO):
```
band     = 1 + BAND_AMP · sin(N.y · BAND_FREQ + φ)
shade   *= band
```

**Continent map** (RINGED-EARTH):
```
u        = atan2(N.x, N.z) / π             // longitude [-1, 1]
v        = N.y                              // latitude  [-1, 1]
land     = fbm(u·LAND_FREQ + φ, v·LAND_FREQ) > LAND_THRESH
ramp     = land ? PAIR_RAMP_BASE : PAIR_RING_BASE     // sea reuses ring ramp
```

**Ring density** (per-radius modulation + Cassini gap):
```
r        = √(P_ring.x² + P_ring.z²)
θ        = atan2(P_ring.z, P_ring.x)
density  = 0.6 + 0.4 · sin(r · RING_BAND_FREQ + θ · 0.4 + φ)
if |r − CASSINI_R| < CASSINI_W: density *= CASSINI_DIM       // 0.18 → bright gap dims
```

**Ring shadow test** (ray from ring point toward sun):
```
shadow_orig = P_ring + ε · sun_dir          // bias to avoid self-hit
in_shadow   = sphere_intersect(shadow_orig, sun_dir).hit
shade      *= in_shadow ? 0.18 : 1.0
```

**Ring lambert** (rings are flat, normal = (0,1,0); use absolute value because rings are double-sided):
```
ring_lambert = AMBIENT + (1 − AMBIENT) · |sun_dir.y|
```

---

## Edge Cases and Pitfalls

- **DEPTH SORT MUST INCLUDE BOTH HITS.** If only the sphere is tested where it hits, the ring is invisible everywhere the sphere doesn't cover. If only the ring is tested where it annulus-clips, the ring would show THROUGH the sphere. Always compute both, then compare `t_sphere` vs `t_ring`.

- **RAY-PLANE GRAZING.** When `|dir.y|` is tiny (camera near edge-on to ring), `t` blows up and rounding produces flicker. Reject `|dir.y| < 1e-5` to skip the test entirely; edge-on rings just don't render those cells (which is correct — edge-on rings are infinitely thin).

- **SHADOW RAY SELF-HIT.** Shadow ray starts ON the ring plane at the hit point. Without an `ε` bias along `sun_dir`, the sphere test may "hit" the ring point itself due to floating-point round-off. Bias by `ε = 1e-3`.

- **SUN BEHIND RING.** Use `|sun_dir.y|` not `max(0, sun_dir.y)` for the ring lambert — rings are double-sided; both top and bottom faces should look lit when the sun is on either side. Without the abs, the bottom of the ring goes black when the sun rises above the equator.

- **CAMERA TOO HIGH OR TOO LOW.** `CAM_HEIGHT` controls the ring tilt on screen. At `CAM_HEIGHT = 0`, rings collapse to a horizontal line (no ellipse, hard to read). At `CAM_HEIGHT > 3`, rings look nearly circular but the planet sits very low. **`CAM_HEIGHT ≈ 1.0–1.5` is the sweet spot.**

- **ASPECT RATIO.** Terminal cells are 2× taller than wide. `fov_v = fov_h · rows · ASPECT_Y / cols` keeps the planet round on screen.

---

## How to Verify

- **PAUSE** (space). Sun freezes; the terminator on the planet and the shadow band on the ring stay aligned. Resume: both advance together.

- **SATURN** pattern. The Cassini Division is visible as a thin DARKER ring within the broader bright ring. Planet shows 4–6 horizontal latitude BANDS of slightly different brightness.

- **Watch the SHADOW BAND.** As the sun rotates, the dark stripe the planet casts on the ring sweeps from the back of the ring around to the front. It is always anti-sun-direction from the planet's centre.

- **RINGED-EARTH** pattern. Press `r` a few times. Continent layouts change but stay structurally Earth-like — irregular landmasses over an ocean. The terminator falls across both land and sea.

- **URANUS** pattern. The rings tilt nearly edge-on; thin near-horizontal LINE crossing in front of and behind the planet.

- **Theme cycle** (`t`/`T`). Planet ramp, ring ramp, and space background all change tint while structure stays identical.

- **Speed** (`+`/`−`). Doubling speed should approximately halve the time for the sun to complete one full azimuthal sweep.

---

## References

- Shirley, P. — *Ray Tracing in One Weekend*, https://raytracing.github.io/.
- Wikipedia — [Ring system (astronomy)](https://en.wikipedia.org/wiki/Ring_system_(astronomy)).
- Wikipedia — [Saturn](https://en.wikipedia.org/wiki/Saturn) — the iconic ringed planet.
- Inigo Quilez — Sphere intersection, https://iquilezles.org/articles/intersectors/.

---

*Source: `raytracing/saturn_with_rings.c`*
