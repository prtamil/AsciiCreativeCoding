# Pass 1 — path_tracer.c: Monte Carlo Path Tracer — Cornell Box

## Core Idea

A **path tracer** simulates light by following photon paths backwards from the camera. For each pixel, one ray is fired into the scene. When it hits a surface it bounces in a random direction sampled from the surface's BRDF. The process repeats until the path hits a light source (contributing energy) or gets terminated by Russian roulette. Many such paths are **averaged** (progressive accumulation) to recover the ground-truth image.

This is *the* algorithm behind photo-realistic renders. The terminal version runs at ~30 fps while converging over ~512 samples, each frame adding a noisy layer that gradually resolves into soft shadows, color bleeding, and accurate indirect illumination.

---

## The Mental Model

### The Rendering Equation

Everything in a path tracer is an approximation of:

```
L_o(x, ω_o) = L_e(x, ω_o) + ∫ L_i(x, ω_i) · f_r(x, ω_i, ω_o) · (N · ω_i) dω_i
```

- `L_o` = outgoing radiance from point x toward camera
- `L_e` = emitted radiance (non-zero only for light sources)
- `f_r` = BRDF (how the surface scatters light)
- `N · ω_i` = cosine of incidence angle (Lambert's law)

The integral is over all incoming directions. A Monte Carlo estimator approximates it with a single random sample and an importance weight.

### Lambertian BRDF + Cosine Sampling: The Perfect Cancellation

For a Lambertian (matte) surface with albedo `ρ`:
- BRDF: `f_r = ρ / π`
- Cosine-weighted hemisphere PDF: `p(ω) = cosθ / π`

Monte Carlo weight = `f_r · cosθ / p(ω)` = `(ρ/π) · cosθ / (cosθ/π)` = **ρ**

The π and cosθ terms cancel exactly. The code `throughput *= albedo` is correct — no division, no cosine computation needed. This is why cosine-weighted hemisphere sampling is the standard choice for Lambertian surfaces.

```c
/* BRDF = albedo/π, PDF = cosθ/π → weight = albedo */
throughput = v3mul(throughput, m->albedo);
rd = cos_sample_hemi(h.N, rng);
```

### Cosine-Weighted Hemisphere Sampling (Malley's Method)

Sample uniformly from a disk, then project up to the hemisphere:

```
r1 = uniform[0,1)   → azimuth φ = 2π·r1
r2 = uniform[0,1)   → sinθ = √r2,  cosθ = √(1-r2)
```

The resulting direction in the local frame: `(cosφ·sinθ, sinφ·sinθ, cosθ)`.
Transform to world space using an ONB (orthonormal basis) around the surface normal.

```c
static void onb(V3 n, V3 *u, V3 *v) {
    V3 up = fabsf(n.x) < .9f ? v3(1,0,0) : v3(0,1,0);
    *u = v3norm(v3cross(up, n));
    *v = v3cross(n, *u);
}
```

Why not `cross(n, up)` for both? Because if `n` is nearly parallel to `up`, the cross product is near-zero. Switching `up` when `|n.x| ≥ 0.9` avoids the degenerate case.

### Russian Roulette — Unbiased Termination

Paths must terminate; fixed depth is biased (missing deep indirect light). Russian roulette terminates at any depth with probability `1 - p` where `p = max(throughput channel)`. Surviving paths are boosted by `1/p` to maintain an unbiased estimator:

```c
if (depth >= RR_DEPTH) {
    float p = v3maxc(throughput);
    if (rng_f(rng) > p) break;       /* terminate */
    throughput = v3s(1.f/p, throughput);  /* compensate */
}
```

Key insight: `E[boost × survive] = (1/p) × p = 1` — the expected value is unchanged. Dim paths (low p) are more likely to be killed, which is exactly right: they contribute little energy even if they survive.

### Progressive Accumulator

Per-pixel float array stores running sum of radiance:

```
accum[y][x][c] += path_trace(pixel_ray)
display[y][x][c] = accum[y][x][c] / num_samples
```

- Early frames: very noisy (1-4 samples per pixel)
- After 64 samples: basic scene structure visible
- After 512 samples: shadows, bleeding, and soft indirect light converge

On resize or user reset: `memset(accum, 0) + samples = 0`. The visual transition from noise to clarity *is* the animation.

### xorshift32 — Decorrelated Per-pixel RNG

Each pixel × frame gets an independent RNG state seeded from its coordinate and frame index:

```c
static Rng rng_seed(int px, int py, int frame) {
    uint32_t s = (uint32_t)(px * 1973 + py * 9277 + frame * 26699 + 1);
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;   /* warm up */
    return s ? s : 1u;
}
```

Why per-pixel independent states instead of one global RNG? Adjacent pixels sharing a RNG state creates correlated noise — artifacts like bands or streaks. Per-pixel seeds ensure each pixel's samples are uncorrelated with its neighbors, giving proper white noise that averages out.

### Reinhard Tone Mapping

Raw path-traced radiance is unbounded (a small bright light can give values of 10, 100, or more). Reinhard compresses `[0, ∞)` → `[0, 1)` per channel:

```
L_display = L / (1 + L)
```

After tone-mapping, apply gamma encoding `L^(1/2.2)` for perceptual sRGB. The light emission of 15 W·sr⁻¹ is compressed to `15/16 ≈ 0.94` — near-white, as desired.

---

## Cornell Box Scene

The Cornell Box is the standard path tracing benchmark, designed to isolate key lighting effects:

| Surface | Material | Effect tested |
|---|---|---|
| Left wall | Red diffuse | Color bleeding onto white floor |
| Right wall | Green diffuse | Color bleeding onto white spheres |
| Floor/ceiling/back | White diffuse | Indirect illumination |
| Area light (y=0.98) | Emissive (15,14,11) | Soft shadows, warm indirect |
| Gold sphere | (0.80,0.58,0.18) | Saturated color bleeding |
| Indigo sphere | (0.22,0.28,0.82) | Cool color contrast |

### Axis-Aligned Quad Intersection

Each wall is an axis-aligned rectangle. Intersection is a one-dimensional solve:

```
axis=Y, pos=−1 (floor):
  t = (−1 − ray.y) / ray.dy
  hit.x = ray.x + t·ray.dx    check ∈ [lo[0], hi[0]]
  hit.z = ray.z + t·ray.dz    check ∈ [lo[1], hi[1]]
```

Normal always faces the incoming ray: if `rd[axis] > 0`, flip normal to `-axis`. This ensures hemisphere sampling is always away from the surface.

### Light Placement

The area light at `y=0.98` sits slightly below the ceiling at `y=1.0`. A ray going upward through the light's XZ footprint `x∈[-0.36,0.36], z∈[0.62,1.38]` hits the light first (`t_light < t_ceiling`). Rays outside the footprint skip the light and hit the white ceiling — correct behavior.

---

## Non-Obvious Decisions

### Why 1e-4f offset after bounce?

```c
ro = v3add(h.P, v3s(1e-4f, h.N));
```

Without offset, the next ray's `t_min=1e-4` test might accept a hit on the same surface (self-intersection) due to floating point rounding. The offset pushes the new ray origin slightly away from the surface so it can't immediately re-hit the same quad/sphere.

### Why terminate at emission instead of continuing?

When a path hits an emissive surface, it adds `throughput × emission` and terminates. This is correct for pure emissive geometry (a light source has no BRDF contribution — it *is* the contribution). Continuing would require subtracting to avoid double-counting.

### Why does convergence stall at ACCUM_CAP?

Path tracing converges as `1/√N` — halving noise requires 4× more samples. At 8192 samples the image is essentially converged for terminal resolution (ASCII chars hide noise below a certain level). The cap prevents wasting CPU on imperceptible improvements.

### SPP tradeoff

- High SPP/frame: converges faster visually but fps drops; terminal becomes unresponsive at 8+ SPP
- Low SPP/frame: 30 fps maintained, image visibly noisy for longer  
- Default 2 SPP: reasonable balance — 60 samples/sec, converges in ~8 seconds
