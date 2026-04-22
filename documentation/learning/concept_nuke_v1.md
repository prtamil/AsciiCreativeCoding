# Pass 1 — raymarcher/nuke_v1.c: Volumetric Mushroom Cloud

## From the Source

**Algorithm:** Volumetric raymarching with Beer–Lambert front-to-back
integration (no SDFs — soft Gaussian density). The mushroom is one anisotropic
Gaussian blob that morphs through time: `rx` grows monotonically while `ry`
grows then compresses, taking the shape from sphere → cap. 2× vertical
supersampling reads two density samples per terminal cell and uses a glyph
picker to recover sub-cell detail.

**Physics/References:**
- Beer–Lambert law: PBR-Book chapter on volume scattering
  (https://www.pbr-book.org/4ed/Volume_Scattering)
- Domain warping: https://iquilezles.org/articles/warp/
- Quintic smootherstep `6t⁵−15t⁴+10t³` with zero 1st AND 2nd derivative at
  endpoints — Perlin "Improving Noise" SIGGRAPH 2002
- Air drag on ash: terminal velocity from gafferongames.com/post/integration_basics

**Math:** Optical depth per step `dτ = density · step · EXTINCTION` attenuates
remaining transmittance by `exp(−dτ)`. Emitted heat at the step is weighted by
`(1 − exp(−dτ)) · T` so front-most cloud occludes back cloud correctly. Bail
when `T < 0.07` — invisible contribution.

---

## Core Idea

A nuclear detonation as one continuous animation, no scene switches:

```
t = 0      detonate, fireball Gaussian blob spawns
t = 0–3    blob rises (advection bias on noise lookup)
t = 3–8    spread + vortex + skirt (overlapping smoothstep windows)
t = 5–9    pyrocumulus turbulence layer ramps in
t = 9–12   plateau — full mushroom holds
t = 12–17  fall — cloud_fade multiplier kills density, ash particles rain
t = 20+    deactivate
```

The blob has independently scaled radii: `rx` (horizontal) grows monotonic;
`ry` (vertical) grows then compresses. The ratio gives the squashed-cap shape
without ever switching from one primitive to another.

---

## Pipeline (one frame)

```
sim_tick(dt):
  advance time, recompute blob_rx, blob_ry, blob_y, stem_h, vortex/skirt/pyro/disp scales
  apply fall: cloud_fade *= (1 − fall); blob_y -= fall · 1.6; etc.
  spawn ash particles when fall > 0
  particles_tick(dt)

renderer_render():
  for each terminal cell:
    for ss in 0..SS_Y:                       /* 2× vertical supersampling */
      ro, rd = camera_ray(col, row + ss / SS_Y)
      Pixel = rm_cast_one(ro, rd, sim.*)
        T = 1
        heat = 0
        smoke = 0
        for step in 0..MAX_STEPS:
          d = cloud_density(p, t, blob_rx, blob_ry, blob_y, stem_h,
                            vortex, skirt, pyro, disp, cloud_fade)
          h = cloud_heat (p, blob_rx, blob_ry, blob_y, blob_heat, stem_h)
          dτ = d · step · EXTINCTION
          a  = 1 − exp(−dτ)
          heat  += a · h · T
          smoke += a · T
          T *= exp(−dτ)
          if T < 0.07: break
          p += rd · step
      pair[ss] = Pixel { heat, smoke, hit }
    glyph_picker(pair)  →  top-weighted | bottom-weighted | full | middle char
  fill canvas

canvas_draw():
  per cell → (smoke or fire) colour pair via active theme
```

---

## The Single Morphing Blob

`cloud_density` reads the blob with anisotropic distance:

```c
dn² = (dx/rx)² + (dy/ry)² + (dz/rz)²
gauss = exp(−dn² · k)
```

`rx`, `ry`, `rz` are independent. Setting `ry < rx` flattens the sphere into a
cap; setting `ry > rx` makes a bullet. Modulating both by `smoothstep` over
overlapping time windows produces sphere → bullet → cap continuously, no
switch.

`cloud_heat` reuses the same anisotropic distance with a tighter falloff to
mark the molten core that drives the fire colour.

---

## Continuous-Time Morph

Every animated parameter is `smoothstep(t_beg, t_end, time)` over an
overlapping window. Quintic `smootherstep` (zero second derivative at
endpoints) on the spread parameter eliminates the visible acceleration kink
at the start of the cap formation:

```c
static inline float smootherstep(float e0, float e1, float x) {
    float t = clmpf((x - e0) / (e1 - e0), 0.f, 1.f);
    return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}
```

Key time markers:

| Marker | Time (s) | Event |
|---|---|---|
| `T_RISE_END` | 3.6 | fireball stops accelerating up |
| `T_SPREAD_END` | 8.0 | mushroom cap fully spread |
| `T_VORTEX_END` | 8.4 | vortex ring at peak |
| `T_PLATEAU_BEG` | 9.0 | mushroom held |
| `T_FALL_BEG` | 12.0 | gravity wins, cloud_fade starts |
| `T_FALL_END` | 16.5 | cloud collapsed |
| `T_DEACTIVATE` | 20.5 | scene reset eligible |

---

## Three Particle Classes — One Struct

```c
typedef enum { PART_DEBRIS, PART_EMBER, PART_ASH } PartType;
```

| Type | When | Forces |
|---|---|---|
| DEBRIS | t ≈ 0 | full gravity, no drag, fast radial |
| EMBER | rise phase | gravity, slight upward bias from heat |
| ASH | t > T_FALL_BEG | gravity × 0.35; horizontal exp drag (1.4/s); terminal velocity −1.6 cells/s |

The terminal velocity clamp on ash is what makes the fall look like real
particulate suspension instead of Newtonian rocks.

---

## 2× Vertical Supersampling + Glyph Picker

Two rays per terminal cell — one through the top half, one through the bottom.
The picker chooses a glyph that approximates the pair:

| (top, bottom) | Glyph |
|---|---|
| (low,  low)  | space |
| (high, low)  | `'` (top-weighted) |
| (low,  high) | `,` (bottom-weighted) |
| (high, high) | `█`-class full |
| (mid,  mid)  | `:` (middle) |

This recovers vertical sub-cell detail without doubling the column resolution.

---

## Themes (5 total)

| # | Name | Smoke palette | Fire palette |
|---|---|---|---|
| 0 | Realistic | greys | white → yellow → red |
| 1 | Matrix | dark green → bright lime | white → green |
| 2 | Ocean | indigo → teal | white → cyan |
| 3 | Nova | violet → magenta | white → pink |
| 4 | Toxic | dark olive → lime | white → yellow |

---

## Non-Obvious Decisions

### Why one blob and not fireball + cap?

The dual-shape model produced a visible "pop" between states — the eye
read it as lerping. One blob with two independent radii gives a single
continuous shape, no transition artifact.

### Why ash uses terminal velocity?

Without a `vy` clamp, ash accelerates to unrealistic fall speeds. Real
particulate hits terminal velocity within a second; the `−1.6 cells/s` clamp
matches the visual cadence.

### Why bail on `T < 0.07`?

At 7 % transmittance the contribution of further steps is below the colour
quantization threshold. Continuing wastes ~20 % of the per-pixel budget for
no visible change.

### Why noise displacement uses an anti-rise bias?

The cloud rises with time. Without subtracting `time × RISE_BIAS` from the
noise lookup `y`, the granulation pattern would slide visibly downward
relative to the cloud — looking like the cloud is falling through static
noise. The bias glues the texture to the cloud.

---

## Open Questions for Pass 3

1. Replace the anisotropic Gaussian with a true SDF-based torus for the cap
   ring. Does it still feel volumetric?
2. Add wind: a horizontal velocity field that tilts the column. How much
   wind before the mushroom shape breaks?
3. Couple flash brightness to the central blob heat so the screen flashes
   white on detonation — currently flash and heat are separate.
4. Multi-scattering approximation: instead of pure absorption, add a single
   diffuse term so the cloud bottom is lit by the molten core. Visible?

---

# Pass 2 — raymarcher/nuke_v1.c: Pseudocode

## Module Map

| Section | Purpose |
|---|---|
| §1 config | EXTINCTION, MAX_STEPS, SS_Y, particle counts, theme palettes, T_* timing |
| §2 clock | `clock_ns`, `clock_sleep_ns` |
| §3 color | 5 themes × (32 smoke + 16 fire) pairs |
| §4 vec3 | dot/cross/len/norm/add/sub/scale (inline) |
| §5 noise/fBm | hash1, value-noise, fbm, fbm2, warped_fbm |
| §6 density | `ellipsoid_dn2`, `noise_displace`, `cloud_density`, `cloud_heat` |
| §7 particles | Particle struct (type), spawn debris/ember/ash, `particles_tick` |
| §8 simulation | NukeState, `sim_update_geometry`, phase HUD label |
| §9 raymarcher | `rm_cast_one` — Beer–Lambert integrator |
| §10 canvas | Pixel pair buffer, glyph picker |
| §11 renderer | bridges sim → screen, draws HUD |
| §12 screen | ncurses init/teardown |
| §13 app | main loop |

## Data Flow

```
sim_tick(dt):
  sim.time += dt
  sim_update_geometry(sim)            /* recompute blob_rx, blob_ry, fade, ... */
  if fall > 0: spawn_ash(sim)
  particles_tick(sim, dt)

renderer_render(rend, sim):
  for row in 0..rows:
    for col in 0..cols:
      Pixel top    = rm_cast_one(ro, rd_top,    sim)
      Pixel bottom = rm_cast_one(ro, rd_bottom, sim)
      canvas[row*cols+col] = glyph_pick(top, bottom)

renderer_draw():
  for px in canvas:
    pair = (px.heat > THRESHOLD) ? fire_pair(px.heat) : smoke_pair(px.smoke)
    mvaddch(row, col, px.glyph | pair)
  draw_particles()
  draw_hud(sim.time, phase_label(sim.time))
```
