# Concept: Nebula / Star Nursery

## Pass 1 — Understanding

### Core Idea
A multi-octave **fBm** (fractional Brownian motion) scalar field defines glowing gas density across the screen. Two parallax layers — near (high frequency, scrolls fast) and far (low frequency, scrolls slow) — create depth. A static catalogue of background **stars** sparkles via sinusoidal twinkle. Periodically a **shock event** (star birth) ignites in the densest gas: a bright dot, then an expanding ring of extra brightness for several seconds — illuminating the surrounding gas before fading.

### Mental Model
Three layered subsystems:
- **Gas**: fBm sampled per cell each frame at two scrolled offsets → gradient density.
- **Stars**: fixed `(x, y)` catalogue, twinkle by `sin(t · 1.5 + phase)`, drawn as `.` `+` `*` by brightness × twinkle.
- **Shocks**: when a shock fires at `(x, y)`, expanding radius `r = SHOCK_SPEED · age`. Cells whose distance to `(x, y)` is near `r` get extra brightness via Gaussian-of-distance-from-ring. Shocks fade over `SHOCK_LIFE` seconds.

The whole scene reads as a deep-space photograph: still and slow, with rare dramatic moments.

### Key Equations
```
fBm(x, y):
    sum  = 0
    amp  = 1, freq = base_freq
    for o in 0..OCTAVES:
        sum += amp · value_noise(x · freq, y · freq)
        amp *= GAIN, freq *= LACUNARITY
    return sum / Σ amplitudes

gas brightness at (sx, sy):
    f = 0.55·fBm(near) + 0.45·fBm(far)              /* parallax mix */
    f += 0.5 · shock_brightness_at(sx, sy)            /* shock illuminates gas */

shock_brightness_at(sx, sy):
    for each alive shock s:
        r_now = SHOCK_SPEED · s.age
        d     = distance from (sx, sy) to s.position
        ring  = d - r_now
        gauss = exp(-ring² / SHOCK_THICKNESS²)
        fade  = 1 - s.age / SHOCK_LIFE
        accumulate gauss · fade
```

### Non-Obvious Decisions
- **Hash-based value noise**: `hash01(x, y) = mix(x · prime1 ^ y · prime2)` — no permutation table needed, runs at every cell every frame without state.
- **Gas threshold cutoff**: cells with `f < threshold` simply skipped — most of the dim periphery isn't drawn, keeping the image sparse and starlit.
- **Two scroll speeds, not one**: the near layer scrolls 3× faster than the far. Without the difference, parallax disappears.
- **Shock spawn picks the densest cell** out of 12 random candidates: gives the impression that stars form *inside* the gas rather than at random.
- **Shock illuminates surrounding gas (not just a ring)**: the Gaussian of `(d - r_now)` adds brightness gradient outward as well as inward of the ring — feels like radiation pressure heating.

### Key Constants
| Name | Role |
|------|------|
| `FBM_OCTAVES` | 4 octaves of value noise |
| `FBM_FREQ_NEAR / FAR` | 0.10 / 0.04 — parallax frequencies |
| `SCROLL_NEAR_DEFAULT` | 0.6 cells/sec |
| `N_STARS_DEFAULT` | 180 background stars |
| `SHOCK_INTERVAL_MIN/MAX` | 4–9 seconds between auto births |
| `SHOCK_SPEED` | 5 cells/sec — radius growth |
| `SHOCK_LIFE` | 5 seconds — fade duration |

### Open Questions
- Could you Doppler-shift the gas hue based on motion direction (red side / blue side)?
- What if shocks could spawn new stars inside the catalogue at the shock centre?
- How does the visual change with 6 octaves of fBm vs 3?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — fBm params, parallax speeds, star count, shock cadence
§5 noise     — hash01 + value_noise + fbm
§6 star      — Star + Shock structs + draw
§7 nebula    — Nebula state + reseed + tick (advances scroll, ages shocks)
§8 scene     — fBm raster + star draw + shock centre marker + HUD
§10 app      — signals, dt + world_time, key handling
```

### Data Flow
```
init: catalogue N stars; clear shocks; randomize next-shock countdown
tick: world_time += dt
      scroll_near/far += rates · dt
      shocks: age += dt; recycle on age > LIFE
      next_shock_in -= dt; on ≤ 0: shock_spawn at densest cell + new countdown
draw: erase
      for each cell: fbm_near + fbm_far + shock_brightness → bucket → glyph
      stars (twinkle)
      shock centre dots (bright while alive)
      HUD
```

### References
- Perlin, "An Image Synthesizer" SIGGRAPH (1985).
- Mandelbrot, "The Fractal Geometry of Nature" (1982) — fBm.
- Musgrave, Kolb & Mace, "The synthesis and rendering of eroded fractal terrains" SIGGRAPH (1989).
