# Concept — `mandelbulb_explorer.c`: 3-D Mandelbulb Raymarcher

## Core Idea

The **Mandelbulb** is a 3-D analogue of the Mandelbrot set defined by iterating a spherical-power formula on 3-D points. It is rendered by **sphere marching** a distance estimator (DE) that gives the minimum distance to the fractal surface at any point in space. When the march gets close enough (`d < HIT_EPS`) the point is on the surface; we then compute shading using SDF-gradient normals, ambient occlusion, and soft shadows.

A **bounding-sphere early-out** in the marcher rejects ~70 % of screen cells (background) in a single ray-sphere test instead of walking 60 expensive DE steps. This is what lets us render the WHOLE FRAME at sharp 1:1 resolution every tick — no progressive seam, smooth orbit at 30 fps.

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

## Bounding-Sphere Early-Out (the perf trick)

The Mandelbulb (n ≥ 2) fits entirely inside a sphere of radius **`BB_RADIUS = 1.30`**. Any view ray that misses this bounding sphere cannot possibly hit the fractal surface — so we return MISS without a single DE evaluation. For typical camera distances ~70 % of screen cells fall outside the bounding sphere; this turns those cells from ~25 expensive DE iterations × 12 inner iter = 300 mb_de iters into ONE ray-sphere test.

```c
float bb_b    = v3dot(ro, rd);
float bb_c    = v3dot(ro, ro) - BB_RADIUS * BB_RADIUS;
float bb_disc = bb_b * bb_b - bb_c;
if (bb_disc < 0.0f) {
    *out_glow_str = 0.0f;
    *out_trap     = 1.0f;
    *out_smooth   = 0.1f;
    return -1.0f;       /* MISS — no DE call needed */
}
float t_enter = -bb_b - sqrtf(bb_disc);
float t_exit  = -bb_b + sqrtf(bb_disc);
if (t_enter < 0.02f) t_enter = 0.02f;
float t = t_enter;
/* ...march until t > t_exit OR hit found */
```

The remaining 30 % of cells (rays that actually cross the bulb's volume) march normally, but only between `t_enter` and `t_exit` — no wasted steps past the bulb. **This is what lets us drop progressive rendering and render full-frame each tick.**

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

## Ambient Occlusion

Cast 3 sample rays along the surface normal at geometric steps. Where `d − DE(P + d·N) > 0` the sample point is closer to the surface than the step distance — it is inside a cavity. Reduced from 5 to 3 samples for performance; visual difference is negligible at terminal resolution.

---

## Soft Shadows

March a ray from the surface toward the light, tracking `min(K · d / t)` as a penumbra factor. `SHADOW_K = 8` controls hardness. Reduced from 24 to **16 march steps** for performance.

---

## Multi-Hue Color Themes

Each theme defines 8 xterm-256 color indices spanning multiple hue families. Smooth escape value `mu ∈ [0, 1]` indexes the 8-entry palette.

Bad: single-hue gradient (e.g., 8 shades of blue) — only brightness varies, no depth contrast.
Good: jump across the hue wheel (red → yellow → green → cyan → blue → violet).

The default theme is **Plasma** (dark magenta → coral → amber → cream): a vibrant arc that pops immediately. Subtle palette rotation (`COLOR_PHASE_SPD = 0.5`) drifts the colour bands over time so static frames stay alive.

---

## Lighting Modes & Default Pose

| Mode | Formula | Effect |
|---|---|---|
| Phong | `KA + KD·(N·L)·sh·ao + KS·spec + RIM` | Full 3-light shading |
| NV (normal-view) | `KA + KD·(N·V)·ao` | Edge detection, silhouette emphasis |

**Default key light** is at `(2.2, 1.0, 1.4)` — strongly side-dominant. Without strong side-light, the bulb just looks spherical; the lobe geometry is invisible. **Default camera angle** `cam_theta = 0.55` (≈ 30° above equator) shows polar cap + equator simultaneously, the canonical recognisable pose.

**Higher contrast** `KA = 0.18`, `KD = 0.82` (was `0.25` / `0.72`) — dark side actually dark; lobe shadows visible. The lobe geometry only READS as 3-D structure if the dark side is genuinely dark.

---

## Brightness-Driven Attribute (the readability trick)

A previous version applied `A_BOLD` to every hit pixel. Result: the dark side was bright too, washing the lobe geometry into a colour-blob. The fix: the attribute now follows the brightness (ramp-index `ri`):

```c
attr = COLOR_PAIR(pair);
if      (ri >= 6) attr |= A_BOLD;     /* highlights pop */
else if (ri <= 2) attr |= A_DIM;      /* shadows truly dark */
/* else normal */
```

Lighting now drives the 3-D read; the palette only colours it.

---

## Glyph Ramp

Project-standard airy ramp: `' '`, `'.'`, `','`, `':'`, `';'`, `'-'`, `'+'`, `'*'`. Brightest cells are `+` / `*` instead of the blocky `#` / `@` in the original — the fractal silhouette no longer reads as an `@` cluster but as a glowing, defined object.

---

## Non-Obvious Decisions

### Why `CELL_W = CELL_H = 1` (sharp pixels)?
At 2:1 block scaling the bulb covered only ~50 × 20 rendered cells — too coarse for the lobe geometry to read. With the bounding-sphere early-out making sharp 1:1 affordable, we keep full resolution.

### Why `MB_MAX_ITER = 12` not 16?
Outer silhouette is unchanged at iter 10; only the deepest cavity self-similar detail (invisible at terminal granularity) is sacrificed. 12 is the sweet spot.

### Why default to TRANSIT-style auto-orbit at 0.18 rad/s?
At smooth fps the orbit looks fluid; faster orbit + any framerate dip looks stuttery. Slow-and-smooth always reads better than fast-and-jerky.

---

## Structure

| Symbol | Type | Role |
|--------|------|------|
| `g_fbuf[CANVAS_MAX_H][CANVAS_MAX_W]` | `PixCell[]` | Framebuffer (luma + colour pair). Filled FULL each tick. |
| `g_stable[CANVAS_MAX_H][CANVAS_MAX_W]` | `PixCell[]` | Last complete frame. Vestigial under full-frame rendering. |
| `BB_RADIUS` | `float` | Bounding sphere of the fractal (1.30) — early-out radius. |
| `MB_MAX_STEPS_FULL` | `int` | Max march steps per ray (60 in fast mode, 80 full). |
| `MB_HIT_EPS` | `float` | Surface hit threshold (0.002); tighter = sharper silhouette. |
| `MB_MAX_ITER` | `int` | Mandelbulb DE iterations (12). |
| `ROWS_PER_TICK` | `int` | 240 → full frame each tick. (Was 8 = progressive seam.) |
| `CAM_ORBIT_SPD` | `float` | 0.18 rad/s — slow enough for any framerate. |

---

## How to Verify

- **Press space** to pause; the bulb freezes at the current orientation. The lobes should be clearly visible — bulbous knobs around the equator with smoother caps at the poles.
- **Arrow keys** orbit the camera. Look at the polar cap from above (`KEY_UP` to raise theta) and notice the radial symmetry.
- **`p`/`P`** ramp the power. n=2 gives a near-sphere; n=8 is the iconic Mandelbulb; n=16 has very fine lobes.
- **`m`** toggles morph mode (n oscillates 2↔8) — shows the family of fractals smoothly.
- **`c`** cycles themes. Plasma is the default; Mono is best for studying the geometry without colour distractions.
- **`o`** toggles AO. Off → cavities flatten out and the bulb looks like a smooth sphere; on → cavities go dark and the lobe structure reveals itself.

---

## References

- Quílez, I. — "Mandelbulb distance estimator", https://iquilezles.org/articles/mandelbulb/.
- White, D. & Nylander, P. (2009) — "The Unravelling of the Real 3D Mandelbrot Fractal". The original Mandelbulb formulation.
- Hart, J. C. (1996) — "Sphere tracing: a geometric method for the antialiased ray tracing of implicit surfaces", *The Visual Computer* 12(10):527–545. The bounding-sphere optimisation generalises ideas from this paper.
- Quílez, I. — "Distance functions", https://iquilezles.org/articles/distfunctions/.

---

*Source: `raymarcher/mandelbulb_explorer.c`*
