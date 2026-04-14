# Pass 1 — mandelbulb_explorer.c: 3D Mandelbulb Fractal Raymarcher

## Core Idea

The **Mandelbulb** is a 3D analogue of the Mandelbrot set defined by iterating a spherical power formula on 3D points. It is rendered by **sphere marching** a distance estimator (DE) that gives the minimum distance to the fractal surface at any point in space. When the march gets close enough (`d < HIT_EPS`) the point is on the surface; we then compute shading using SDF-gradient normals, ambient occlusion, and soft shadows.

---

## The Distance Estimator

Standard Mandelbrot: `z ← z² + c`. The Mandelbulb generalises the squaring to arbitrary power `p` in spherical coordinates:

```
z = (r, θ, φ) in spherical
z^p: r' = r^p,  θ' = p·θ,  φ' = p·φ
```

Then back to Cartesian:
```
z_new = r^p · (sin(p·θ)·cos(p·φ),  sin(p·θ)·sin(p·φ),  cos(p·θ))
z_new += c
```

The **distance estimator** (Iñigo Quílez formula):

```
DE = 0.5 · log(r) · r / dr
```

where `dr` tracks the derivative magnitude: `dr = p · r^(p-1) · dr + 1`.

When `dr` is large (fast divergence), the surface is distant. When it is small (slow divergence), we are near the surface.

```c
static float mb_de(Vec3 pos, float power, int max_iter, float *smooth) {
    Vec3 z = pos; float dr = 1.f, r;
    for (int i = 0; i < max_iter; i++) {
        r = v3_len(z);
        if (r > BAIL) { *smooth = i - logf(logf(r)/logf(BAIL))/logf(power); break; }
        float theta = acosf(z.z / r);
        float phi   = atan2f(z.y, z.x);
        dr = powf(r, power-1.f) * power * dr + 1.f;
        float rp = powf(r, power);
        z = v3_add(v3_scale(v3(sinf(power*theta)*cosf(power*phi),
                               sinf(power*theta)*sinf(power*phi),
                               cosf(power*theta)), rp), pos);
    }
    return 0.5f * logf(r) * r / dr;
}
```

---

## Smooth Coloring (Continuous Escape Time)

Integer escape count produces hard color bands. The smooth formula:

```
mu = iter + 1 − log( log(|z|) / log(bail) ) / log(power)
```

This removes the staircase discontinuity by linearly interpolating between escape counts based on how far past the bailout radius the orbit was. `mu` is a float used to index into the color palette with sub-band precision.

---

## Orbit Trap

During iteration, track the minimum distance from any orbit point to a geometric object (e.g., the XY plane, origin, or a sphere):

```c
trap = fminf(trap, fabsf(z.y));   /* distance to XY plane */
```

`trap ∈ [0, 1]` modulates color: orbit points that stayed near the trap object get a different hue than points that swung far away, creating the characteristic "tentacle" coloring on the bulb surface.

---

## Progressive Rendering (ROWS_PER_TICK)

Raymarching the full screen per frame at 60 fps is too slow. Instead, render a sliding window of `ROWS_PER_TICK=4` rows per frame:

```c
for (int r = g_scan_row; r < g_scan_row + ROWS_PER_TICK; r++) {
    for (int c = 0; c < cols; c++) {
        g_pixels[r][c] = mb_cast_pixel(r, c, cols, rows, s);
    }
}
g_scan_row = (g_scan_row + ROWS_PER_TICK) % rows;
```

The `g_stable` buffer holds the last completed full frame. While scanning, `g_stable` is what gets displayed — the radar-sweep scan effect shows new results row by row.

Key: only set `g_dirty = true` (which resets `g_scan_row`) on user input. Power morphing mode deliberately avoids setting dirty so the scan creates a continuous sweep animation.

---

## Ambient Occlusion

Cast 5 sample rays along the surface normal at geometric steps `{0.01, 0.02, 0.04, 0.08, 0.16}`:

```c
float ao = 0.f, s = 1.f;
for (int i = 1; i <= 5; i++) {
    float d = i * AO_STEP;
    ao += s * (d - mb_de(P + d*N, power, AO_ITERS, &dummy));
    s *= 0.5f;     /* geometric weight: closer samples matter more */
}
luma -= AO_STR * ao;
```

Where `d - DE(P + d·N) > 0` means the sample point is closer to the surface than the step distance — it is inside a cavity. The geometric weight `s *= 0.5` ensures near samples have 2× the weight of far samples.

---

## Soft Shadows

March a ray from the surface toward the light, tracking `min_t/d` as a penumbra factor:

```c
float shadow = 1.f;
for (float t = SHADOW_TMIN; t < SHADOW_TMAX; ) {
    float d = mb_de(P + t*L, power, SHADOW_ITERS, &dummy);
    if (d < 0.0002f) { shadow = 0.f; break; }
    shadow = fminf(shadow, SHADOW_K * d / t);
    t += d;
}
```

`SHADOW_K` controls hardness. Large K = hard shadow (only fully occluded). Small K = soft shadow (penumbra based on proximity). Typical value: K=8–16.

---

## Multi-Hue Color Themes

Each theme defines 8 xterm-256 color indices spanning multiple hue families. The smooth escape value `mu ∈ [0, 1]` indexes into the 8-entry palette with linear interpolation between adjacent entries.

Bad: single-hue gradient (e.g., 8 shades of blue) — only brightness varies, no depth contrast.  
Good: jump across the hue wheel (red → yellow → green → cyan → blue → violet → back).

The `Prism` theme uses only light/medium colors (xterm cube entries where all components ≥ 2) so the entire image is bright and saturated rather than sinking into darkness.

---

## Lighting Modes

| Mode | Formula | Effect |
|---|---|---|
| Phong | `KA + KD·(N·L)·sh·ao + KS·spec + RIM` | Full 3-light shading |
| NV (normal-view) | `KA + KD·(N·V)·ao` | Edge detection, silhouette emphasis |

Key: `KA` (ambient constant) is never multiplied by AO so pixels deep in cavities never collapse to the space character. The floor `KA = 0.08` ensures a minimum brightness that keeps the fractal visible even in occluded crevices.

---

## Non-Obvious Decisions

### Why `powf(r, power-1)` not `powf(r, power)`?

The derivative update formula is `dr = p · r^(p-1) · dr + 1`. Using `r^p` instead of `r^(p-1)` would give wrong DE values and cause the march to be too conservative (stopping too far from the surface) or too aggressive (stepping through thin features).

### Power = 8 vs other values

Power 2 gives a smooth blob (all real values, Mandelbrot limit). Power 8 gives the classic Mandelbulb with 8-fold rotational symmetry and the characteristic "tentacles" and pods. Powers 3–7 and 9–12 give different symmetries; power morphing (interpolating between 2 and 12) shows a smooth deformation between simple sphere and full fractal complexity.

### Why 24 auxiliary iterations for normals?

Normal computation calls `mb_de` 6 times (central differences). Using the full `MAX_ITER=100` iterations for each would cost 600 iterations per pixel hit for normals alone. 24 iterations captures the coarse structure well enough for a smooth normal while keeping frame time reasonable. For thin features, higher iteration count would be needed for accuracy.
