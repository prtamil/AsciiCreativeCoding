# Pass 1 — raymarcher/metaballs.c: SDF Metaballs + Smooth-Min Blending

## Core Idea

Six spheres move on independent Lissajous orbits. Their signed-distance fields are blended together using a smooth-min function instead of a hard minimum. This causes them to merge smoothly into a single organic blob and pull apart again as they orbit. The surface is raymarched, Phong-shaded with soft shadows, and colored by surface curvature — flat merged regions are cool-toned, sharp sphere peaks are warm-toned.

## The Mental Model

Imagine soap bubbles. Two separate bubbles are distinct spheres. When they touch they grow a shared neck — the surface smoothly connects the two. That neck is what smooth-min creates mathematically. Unlike a hard `min(sdf_a, sdf_b)` which produces a sharp crease at the junction, smooth-min rounds the crease into a smooth skin.

The `k` parameter (blend radius) controls neck width. At `k ≈ 0` the neck vanishes — distinct spheres. At `k ≈ 3.0` the neck is so wide that all 6 balls fuse into one amorphous blob.

## Smooth-Min (Polynomial, C¹)

```c
static float smin(float a, float b, float k) {
    float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
    return fminf(a, b) - h * h * k * 0.25f;
}
```

- When `|a - b| > k`: `h = 0`, `smin = min(a, b)`. No blending — the surfaces are too far apart.
- When `a ≈ b` (balls at equal distance): `h = 1`, `smin = min(a,b) - k/4`. Maximum blending — the isosurface extends k/4 units beyond either ball, creating the shared neck.
- C¹ continuity: the function and its first derivative are continuous. This is required for correct normals via finite differences — a discontinuous blend would produce faceting artifacts.

## Normal Estimation: Tetrahedron Method

Standard central differences require 6 SDF evaluations. The tetrahedron technique achieves the same accuracy with only 4:

```c
float e  = NORM_EPS;
/* Vertices: k0=(+,-,-), k1=(-,+,-), k2=(-,-,+), k3=(+,+,+) */
float f0 = sdf_scene(v3(p.x+e, p.y-e, p.z-e), ...);
float f1 = sdf_scene(v3(p.x-e, p.y+e, p.z-e), ...);
float f2 = sdf_scene(v3(p.x-e, p.y-e, p.z+e), ...);
float f3 = sdf_scene(v3(p.x+e, p.y+e, p.z+e), ...);
return v3norm(v3( f0-f1-f2+f3, -f0+f1-f2+f3, -f0-f1+f2+f3));
```

Each output component sums exactly 2 positive and 2 negative terms from the 4 samples — no redundant evaluations.

## Curvature Coloring

Mean curvature ≈ Laplacian of the SDF. Computed via 6-neighbor central differences:

```c
float lap = (sdf(p+ex) + sdf(p-ex) + sdf(p+ey) + sdf(p-ey)
           + sdf(p+ez) + sdf(p-ez)) - 6.0f * sdf(p);
float curvature = lap / (eps * eps);
```

For a sphere of radius r, the Laplacian = 2/r. For r=0.55: ~3.6. With `CURV_SCALE = 0.25`, this maps to band 7 (highest = warm). Flat merged saddle regions have curvature ≈ 0 → band 0 (lowest = cool). The color gradient shows the 3D shape of the blob even on flat terminal characters.

## Soft Shadows

```c
float res = 1.0f, t = tmin;
for (int i = 0; i < SHADOW_STEPS && t < tmax; i++) {
    float h = sdf_scene(ray_point, ...);
    if (h < eps) return 0.0f;      /* occluded */
    res = fminf(res, sk * h / t);  /* penumbra estimate */
    t += fmaxf(h, eps);
}
```

`sk * h / t` is the ratio of clearance to distance marched. A ray that passes close to an occluder accumulates a small `res` (dark penumbra). A ray with wide clearance keeps `res ≈ 1` (fully lit). The parameter `sk = SHADOW_K = 8.0` controls sharpness — higher = harder shadow edge.

## Canvas Downsampling (2×2 Block)

```c
#define CELL_W  2
#define CELL_H  2
```

Each canvas pixel renders as a 2×2 block of terminal cells. This halves both dimensions, reducing the pixel count by 4×. For a 200×50 terminal: 2500 canvas pixels instead of 10000. The raymarching SDF per pixel costs ~600 evaluations (64 march + 4 normal + 7 curvature + 16 shadow, each looping over 6 balls). At 24fps, 2500 pixels × 600 × 6 ≈ 216M sqrt operations/second — well within a modern CPU.

Aspect correction (for round circles):
```c
float phys_aspect = ((float)ch * (float)CELL_H * CELL_ASPECT) / ((float)cw * (float)CELL_W);
```

`CELL_ASPECT = 2.0` is the physical height-to-width ratio of one terminal character. Multiplied into the vertical ray direction so the blob appears as a sphere, not a squashed ellipsoid.

## Lissajous Orbits

Each of the 6 balls traces an independent 3D Lissajous figure:
```c
centers[i] = v3(
    ORBIT_RX * sinf(BALL_A[i] * t + BALL_PX[i]),
    ORBIT_RY * sinf(BALL_B[i] * t + BALL_PY[i]),
    ORBIT_RZ * cosf(BALL_C[i] * t)
);
```

Different A/B/C frequencies for each ball prevent synchronization — they approach and recede in complex, non-repeating patterns. Phase offsets `BALL_PX[i] = i × 2π/6` spread balls evenly around the orbit at t=0 so the animation doesn't start from a collapsed state.

## Four Color Themes × 8 Curvature Bands

```c
static const int k_theme_colors[N_THEMES][N_CURV_BANDS] = {
    { 27,  33,  38,  44,  130, 166, 202, 214 },  /* Classic: blue→orange */
    { 17,  18,  19,  20,   26,  32,  38,  51 },  /* Ocean:   navy→cyan   */
    { 52,  88, 124, 160,  196, 202, 208, 228 },  /* Ember:   red→yellow  */
    {201, 165, 129,  93,   57,  82, 118, 155 },  /* Neon:    magenta→green */
};
```

32 color pairs (4 themes × 8 bands) are initialised once at startup. Switching themes is instantaneous — the `theme` integer selects the base `CP_IDX(theme, band)` without re-registering pairs.

## Non-Obvious Decisions

### Why polynomial smin rather than exponential?
The exponential smin `−log(e^−ka + e^−kb)/k` has C∞ continuity but requires `exp`/`log` — ~10× slower than the polynomial version. The polynomial version is C¹ (continuous first derivative), which is sufficient for correct normals and smooth shading. Higher-order continuity is not visually distinguishable in ASCII rendering.

### Why NORM_EPS = 0.004 but CURV_EPS = 0.06?
Normals need small `eps` for accuracy — too large blurs sharp features. Curvature (the Laplacian) uses larger `eps` because: (1) it's measuring a second-order quantity and needs a larger sampling footprint, (2) it only determines color band selection (8 discrete levels), so sub-band precision is wasted.

### Why is the light path not keyframe-animated?
A slowly orbiting light on a smooth sinusoidal path `(4cos(t), 2sin(0.45t)+2.5, 3.5)` keeps the shading dynamic without the distraction of sharp directional changes. The 0.45 frequency mismatch on the vertical component prevents the light from returning to the same position too quickly.

## From the Source

**Algorithm:** SDF raymarching with smooth-min blending. Each metaball is a sphere SDF; the scene SDF is `smin(sdf_0, smin(sdf_1, ... sdf_n))` applied iteratively. The polynomial smooth-min (Quilez, 2013) blends surfaces near contact without the C∞ smooth-min's `exp`/`log` cost (~10× slower).

**Math:** Polynomial smooth-min (Quilez, 2013): `smin(a, b, k) = a − h²·k/4` when `h = clamp(0.5 + (b−a)/(2k), 0, 1)`. At k=0: exact minimum (hard Boolean union). At k=4.0: fully merged. Normal estimation uses central finite differences: `n ≈ (sdf(p+ε,p,p) − sdf(p−ε,p,p), ...) / (2ε)`. Phong shading: `colour = ambient + diffuse·(N·L) + specular·(R·V)^shininess` where `R = 2(N·L)N−L`. Curvature coloring uses Laplacian of SDF (6-neighbor central difference, CURV_EPS=0.06).

**Performance:** NORM_EPS=0.004 (small for accuracy); CURV_EPS=0.06 (larger — second-order quantity, only 8 discrete color bands needed). Each canvas pixel: RM_MAX_STEPS=64 march steps + 4 normal samples + curvature + SHADOW_STEPS=16. CELL_W=CELL_H=2 → 4× fewer pixels.

## Key Constants

| Constant | Effect |
|---|---|
| K_DEFAULT | Starting blend radius (0.8 = moderate neck width) |
| K_MIN / K_MAX | Blend range (0.05 = nearly separate, 4.0 = fully merged) |
| CELL_W / CELL_H | Canvas downsampling (2 = quarter pixel count) |
| CURV_SCALE | Laplacian → [0,1] normalisation factor |
| SHADOW_K | Soft shadow sharpness (higher = harder edge) |
| ORBIT_RX/RY/RZ | Lissajous orbit radii in world space |

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `BALL_A/B/C[6]` | `const float[6]` | ~72 B | Lissajous frequency coefficients per axis per ball |
| `BALL_PX/PY[6]` | `const float[6]` | ~48 B | initial phase offsets to spread balls at t=0 |
| `BALL_R[6]` | `const float[6]` | ~24 B | SDF sphere radius per ball |
| `k_theme_colors[4][8]` | `const int[4][8]` | ~128 B | curvature band → color pair index per theme |
