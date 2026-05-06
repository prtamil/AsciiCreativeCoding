# Pass 1 — path_tracer.c: Progressive Monte Carlo path tracer (Cornell Box)

## Core Idea

A path tracer simulates light by following photon paths BACKWARDS from
the camera. For each pixel, fire a ray into the scene. At each surface
hit:
1. If the surface is emissive (a light), accumulate `throughput · emission`
   and stop.
2. Otherwise, multiply `throughput *= albedo`, sample a new direction
   from the cosine-weighted hemisphere around the normal, recurse.

Termination by Russian roulette at depth ≥ RR_DEPTH: survive with
probability `p = max(throughput)`; on survival multiply throughput by
`1/p` (preserves unbiasedness) — finite expected depth, no truncation.

Per-pixel contributions accumulate ACROSS frames. Each frame adds
SPP (samples per pixel) more paths. The image converges from noisy
to clean over hundreds of samples — watching it settle is the demo.

## Cornell Box

The canonical test scene since Goral 1984:
- 5 walls (white floor/ceiling/back, RED left, GREEN right)
- 1 area light (warm) just below the ceiling
- 2 spheres (gold left, indigo right) above the floor

Color bleeding: red wall tints the gold sphere on its left flank;
green wall tints the indigo sphere on its right. This is the visual
signature of multi-bounce global illumination.

## Cosine-weighted hemisphere (Malley's method)

```
r1, r2 ∈ [0,1)
φ      = 2π · r1
local  = (cosφ · √r2, sinφ · √r2, √(1−r2))
world  = onb(N) · local
```

PDF = cosθ/π. Paired with Lambertian BRDF f_r = albedo/π:
```
weight = f_r · cosθ / pdf = (albedo/π) · cosθ / (cosθ/π) = albedo
```
The cosθ and π cancel — that's why we use cosine-weighted sampling.

## Russian roulette (Veach)

```
if depth ≥ RR_DEPTH:
  p = max(throughput.r, .g, .b)
  if rng > p: kill (return)
  else: throughput /= p
```

Unbiased: E[contribution] is unchanged because the survival probability
exactly compensates for the throughput inflation.

## RGB cube paint pipeline

Same as the entire raytracer folder:
1. Reinhard tone-map `L/(1+L)` per channel
2. Gamma encode `^(1/2.2)`
3. Quantise to 216 ncurses pairs (6×6×6 RGB cube)
4. Pick density char from 92-char Bourke ramp
5. A_BOLD on bright cells; the path tracer also keeps A_BOLD always for
   the colour cube to stay vivid

## §-section structure

```
§1 config       (sub-sectioned 1.1-1.6: frame rate, view, PT consts, scene coords, ramp, ncurses)
§2 clock
§3 vec3
§4 rng          xorshift32 + per-pixel-per-frame seed
§5 scene        5.1 Material · 5.2 Quad · 5.3 Sphere · 5.4 lookup helpers
§6 intersection 6.1 ray_quad · 6.2 ray_sphere · 6.3 scene_hit
§7 path trace   7.1 onb · 7.2 cos_sample_hemi · 7.3 path_trace (iterative)
§8 framebuffer  8.1 accumulator · 8.2 add_frame · 8.3 tone-map+draw
§9 screen       9.1 color_init · 9.2 progress bar · 9.3 hud_draw
§10 app         signals, resize, main loop
```

## Worked Example (verify by hand, color bleeding)

Path: eye → floor → red wall → light.

```
Materials:
  floor   albedo (0.73, 0.73, 0.73)
  red     albedo (0.65, 0.05, 0.05)
  light   emission (15, 14, 11)

throughput evolution:
  start:        (1.00, 1.00, 1.00)
  after floor:  (0.73, 0.73, 0.73)
  after wall:   (0.4745, 0.0365, 0.0365)
  light hit, contribution = throughput · emission:
                = (0.4745·15, 0.0365·14, 0.0365·11)
                = (7.12, 0.51, 0.40)
```

Bright RED — that's color bleeding from the wall, the visual signature
of multi-bounce global illumination.

## HUD spec

- Yellow status row 0: fps, spp, samples, "tracing/PAUSED/CONVERGED"
- Yellow row 1: progress bar showing convergence (filled green when
  inside ACCUM_CAP)
- Cyan hint bottom row

## How to verify

- Press 'r' to reset, then watch:
  - sample 1     ≈ solid noise, faint scene
  - sample 32    ≈ silhouettes visible, lots of grain
  - sample 256   ≈ clean walls, slight grain on spheres
  - sample 2048  ≈ near-converged
- COLOUR BLEED: gold sphere has slight reddish tint on LEFT flank
  (red wall bouncing) and slight greenish on RIGHT (green wall).
- Soft shadow under each sphere (less direct illumination there).
- '+' to bump SPP — convergence visibly faster but fps drops.

## Edge cases

- **Self-intersection**: offset ray origin by 1e-4 · N each bounce.
  Without the push, the next intersection finds the surface we just
  left at t ≈ 0 → path stops one bounce short.
- **Double-sided normal**: for quads we pick normal facing incoming
  ray; for spheres we flip outward normal if pointing away.
- **RR probability**: max(channels), NOT mean — preserves paths whose
  energy is concentrated in a single band (deep red etc).
- **Tone-map AFTER divide**: divide accum by samples FIRST (linear
  average), THEN Reinhard. Tone-mapping a sum of N samples is not
  the same as N times the tone-map of one sample.

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, view, MAX_DEPTH, RR_DEPTH, SPP, scene coords |
| §2 clock        | monotonic timer + sleep |
| §3 vec3         | V3 helpers |
| §4 rng          | xorshift32 + decorrelated per-pixel-per-frame seed |
| §5 scene        | Cornell box materials + quads + spheres |
| §6 intersection | ray_quad · ray_sphere · scene_hit |
| §7 path trace   | iterative random-walk with RR termination |
| §8 framebuffer  | progressive accumulator |
| §9 screen       | 6×6×6 cube + 92-char ramp + HUD |
| §10 app         | signals, resize, main loop |

## Data flow

```
main → dt → if !paused && samples < ACCUM_CAP:
              accum_add_frame(spp):
                per pixel:
                  per sample s in 0..spp-1:
                    rng = rng_seed(col, row, frame · spp + s)
                    jitter pixel position
                    rd = camera_ray(jittered)
                    c = path_trace(cam_pos, rd, rng)
                    accumulate
                  g_accum[row][col] += sum
                g_samples += spp

           accum_draw:
             per pixel:
               linear_avg = g_accum / g_samples
               Reinhard + gamma
               cube quantise + ramp char
               paint
```

## Key patterns to internalise

**Photon paths backwards from camera.** Same image as forward sim,
trillion times less wasted work — random walks that LAND on lights
contribute, others die.

**Cosine-weighted sampling cancels the cosθ in the integrand.** That's
why we sample directions with that PDF — algebraic simplification of
the Monte Carlo estimator down to `weight = albedo`.

**Russian roulette = unbiased termination.** Compensate killed paths
by inflating survivors' throughput. E[contribution] unchanged.

**Progressive accumulator + tone-map at draw time.** Keep accum in
linear HDR; divide and tone-map only at paint time.

**Same paint pipeline as the entire raytracer folder.** RGB → Reinhard
→ gamma → 6×6×6 cube + 92-char Bourke ramp.
