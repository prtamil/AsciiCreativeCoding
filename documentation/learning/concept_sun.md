# Pass 1 — raymarcher/sun.c: 3-D Solar Simulation

## From the Source

**Algorithm:** Sphere-tracing on a noise-displaced sphere SDF. Each terminal
cell shoots one ray; the marcher advances by `f(p)` (the SDF value, which is
also the guaranteed-safe step) until either `f(p) < HIT_EPS` or the ray
escapes. On a hit, sample a domain-warped fBm noise as a temperature value and
map through a 256-colour palette; on a miss, accumulate exponential corona
glow.

**Physics/References:** Inigo Quilez sphere-tracing notes
(https://iquilezles.org/articles/raymarchingdf/); Perlin "Improving Noise"
SIGGRAPH 2002 for the quintic fade in value-noise; limb-darkening law
(Eddington/Hestroffer, https://en.wikipedia.org/wiki/Limb_darkening) for the
edge falloff. Flares are capsule SDFs smooth-unioned (`smin` polynomial blend)
into the sphere so the foot has no seam.

**Math:** SDF for the sun is `sphere_radius − fbm(p_normalised, t) × amp`.
fBm = sum over octaves `i` of `noise(p · 2^i) / 2^i`; domain warping evaluates
`fbm(p + warp_vec)` where `warp_vec` is itself fBm of `p`, producing the
swirling cauliflower granulation that pure fBm cannot.

---

## Core Idea

Renders an animated sun: a churning lava sphere with eight independent flares,
each in a state machine (BLAST → magnetic ARCH → DECAY). The display is
temperature-mapped 256-colour ASCII; cooler edges show limb darkening.

The trick is that the entire visual emerges from one SDF function (`sdf_sun`)
plus one noise function (`warped_fbm`). All complexity is composition:

```
sdf_sun(p, t) = sphere_radius − warped_fbm(p, t) × NOISE_AMP
              ⊕  smin( capsule(p, flare_a, flare_b, r) , ... )    over 8 flares
```

The `smin` (smooth minimum) polynomial blend
`smin(a,b,k) = min(a,b) − k·h²/4` (with `h = max(k − |a−b|, 0) / k`) makes the
flare foot melt into the sphere instead of butting up against it.

---

## Pipeline (one frame)

```
sim_tick()
  └─ advance time, step each flare's state machine

renderer_render()
  └─ for every terminal cell:
       1. compute ray direction from camera through (col, row)
       2. sphere-march: t += sdf_sun(p, t)
            terminate on hit (sdf < HIT_EPS) or miss (t > MAX_T)
       3. accumulate corona along the way: corona += exp(-near · CORONA_SCALE)
       4. on hit: finite-difference normal, limb darkening, fbm temperature
          → write Pixel { hit, temp, flare_t, corona_acc } to canvas
       5. on miss: write Pixel { hit=false, corona_acc, star_hash }

canvas_draw()
  └─ each Pixel → ncurses char + 256-colour pair via the active theme
```

---

## Sphere Tracing — Why It Works

Given a signed distance function `f(p)`, the value at any point is the
guaranteed-shortest distance to the surface. Stepping along a ray by exactly
`f(p)` therefore can never overshoot. The march terminates when `f(p) < ε`
(hit) or the accumulated `t` exceeds `MAX_T` (miss).

Adaptive cost: in empty space the steps are large; near surfaces they shrink
to ε. Average rays cost ~25 evaluations even at the silhouette.

---

## Flare State Machine

Each flare has three states and runs its own clock:

| State | Duration | Visual |
|---|---|---|
| DORMANT | random | invisible |
| BLAST | ~0.5 s | white-hot point + scattered sparks fading in (`smoothstep(0, 0.25, phase)`) |
| ARCH | ~3 s | Bézier-arched magnetic loop reaching from foot_a to foot_b, peak intensity |
| DECAY | ~2 s | fade to dormant (`smoothstep(1, 0, phase)`) |

The arch is a quadratic Bézier curve from `foot_a` to `foot_b` with apex height
`h`. The capsule SDF along this arc is approximated by sampling
`FLARE_ARC_SEGS = 8` segments and taking the minimum:

```c
float bezier_tube_dist(p, fa, fb, h, segs):
    best = ∞
    for seg in 0..segs:
        pa = bezier_arc(fa, fb, h, seg/segs)
        pb = bezier_arc(fa, fb, h, (seg+1)/segs)
        d  = sdf_capsule(p, pa, pb, arc_radius((seg + 0.5) / segs))
        if d < best: best = d
    return best
```

The analytic Bézier-tube SDF would need a degree-5 polynomial root finder —
8 capsule segments are visually identical and one tenth the code.

---

## Limb Darkening

Real stars look cooler at the limb because photons from there traverse more of
the atmosphere on their way to the observer. The 1-coefficient linear law is

```
T_visible = T_centre · (1 − u₁ · (1 − μ))     with μ = N · V
```

Setting `u₁ = 0.20` gives a 20 % cooler edge — visually convincing and one
fma instruction. Used at the shading site:

```c
float ndv  = fabsf(v3dot(N, V));      /* μ = N · V */
float temp = sh.temp * (0.80f + 0.20f * ndv);
```

Reference: https://en.wikipedia.org/wiki/Limb_darkening

---

## Corona — Beer-Lambert Glow

The corona is accumulated along every ray (hit or miss) by an exponential
falloff from the surface:

```
corona_acc += exp(-near · CORONA_SCALE) · CORONA_BRIGHT · CORONA_WEIGHT
```

`near = max(0, sdf_value)` is the distance from the current ray point to the
sun surface; close points contribute heavily, far points fall off
exponentially. The accumulator naturally produces the soft halo without any
post-process step.

---

## Themes (4 total)

Each theme has paired hot/cold ramps for the sun temperature and flare colour:

| # | Name | Sun ramp | Flare colour |
|---|---|---|---|
| 0 | Solar (default) | white → yellow → orange → red → maroon | white-hot tips |
| 1 | Plasma | cyan → magenta → purple → black | bright pink |
| 2 | Toxic | bright lime → forest → black | acid yellow |
| 3 | Arctic | white → ice → steel-blue → deep blue | bright cyan |

`t` cycles the active theme; `init_pair` re-binds the same colour pair indices
so the hot loop in `canvas_draw` never branches.

---

## Non-Obvious Decisions

### Why one-sided FD normals?

Standard finite-difference normal needs 6 evaluations of the SDF. Reusing
`sh.dist` (the centre value already known from the hit test) lets us compute
the normal with only 3 extra evaluations — halves the most expensive per-pixel
cost.

### Why approximate the Bézier tube with 8 capsules?

The exact analytic distance from a point to a quadratic Bézier curve requires
solving a degree-5 polynomial. 8 capsule segments are visually
indistinguishable, far simpler to reason about, and trivially adjustable for
quality vs. speed.

### Why temperature instead of RGB?

A single float per pixel feeds the colour table; the same field naturally
interpolates flare temperature, fbm temperature, and limb darkening with one
multiplier per stage. RGB would triple the bookkeeping.

### Why `smin` for flare-to-sphere blend?

A sharp boolean union would create a visible crease at the flare foot. `smin`
polynomial blend with `k = 0.05` rounds the crease over a few cells — the
flare looks like it grew from the surface, not landed on it.

---

## Open Questions for Pass 3

1. Replace value-noise with simplex noise for the displacement. Does it look
   different at this resolution? Reference: Perlin's simplex noise paper.
2. Add a magnetic-field line tracer that follows the actual potential between
   foot pairs (not just a fixed Bézier). Compare.
3. Make the flare colour temperature-dependent — currently it's a fixed
   palette. Use Planck radiation curve to map `T → RGB`.
4. Parallelise the per-row raymarch with pthreads. What speedup at 60 fps?

---

# Pass 2 — raymarcher/sun.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | CAM_Z, FOV, NOISE_FREQ/AMP, FLARE_ARC_SEGS, CORONA_SCALE, theme palettes |
| §2 clock | `clock_ns()`, `clock_sleep_ns()` |
| §3 color/theme | 4 themes × 16 pairs; `theme_apply()` |
| §4 vec3 math | dot, cross, len, norm, add, sub, scale (inline) |
| §5 noise/fBm | hash1, value-noise, fbm, warped_fbm |
| §6 SDF | sphere, capsule, smin |
| §7 flares | Flare struct, state machine, `bezier_arc`, `bezier_tube_dist`, `flare_sdf` |
| §8 simulation | Simulation struct (time + flares), `sim_tick` |
| §9 raymarcher | `rm_cast` — sphere-march, hit shading, corona accumulator |
| §10 canvas | Pixel buffer, `canvas_draw` (Pixel → ncurses) |
| §11 renderer | bridges sim → canvas; `renderer_render`, `renderer_draw` |
| §12 screen | ncurses init/teardown/resize |
| §13 app | main loop |

## Data Flow

```
Frame N:
  sim_tick(dt):
    sim.time += dt
    for each flare:
      flare.phase += dt / state_duration
      if flare.phase >= 1: advance state, reset phase

  renderer_render(rend, &sim):
    for row in 0..rows:
      for col in 0..cols:
        dir = camera_ray(col, row)
        Pixel px = rm_cast(cam, dir, sim.time, ry, sim.flares, sim.n_flares)
        canvas[row * cols + col] = px

  renderer_draw(rend, cols, rows, hud):
    for px in canvas:
      char = brightness_char(px.temp + px.corona_acc)
      pair = theme_pair(px.temp, px.flare_t)
      mvaddch(row, col, char | pair)

rm_cast(ro, rd, t, ry, flares, n):
  t_acc = 0
  corona = 0
  for step in 0..MAX_STEPS:
    p = ro + rd * t_acc
    sh = sdf_sun(p, t, ry, flares, n)
    near = max(0, sh.dist)
    corona += exp(-near · CORONA_SCALE) · CORONA_BRIGHT · CORONA_WEIGHT
    if sh.dist < HIT_EPS:
      N = normalize( one_sided_fd_normal(p, sh.dist) )
      V = normalize(ro - p)
      ndv = |dot(N, V)|
      temp = sh.temp * (0.80 + 0.20 * ndv)         /* limb darkening */
      return Pixel { hit=true, temp, flare_t=sh.flare_t, corona }
    t_acc += sh.dist
    if t_acc > MAX_T: break
  return Pixel { hit=false, corona, star_hash=hash(rd) }
```
