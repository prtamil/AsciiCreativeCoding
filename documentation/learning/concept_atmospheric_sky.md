# Concept — `atmospheric_sky.c`: Procedural Sky over Mountain Silhouette

## Core Idea

A view ray points somewhere in the sky. The sky's colour at that point depends on TWO things: HOW HIGH the ray points (horizon vs zenith) and WHERE THE SUN IS (just risen, overhead, setting, gone). A real sky derives this from per-wavelength Rayleigh scattering integrated along the ray. We approximate it with two GRADIENT LOOKUPS into the active theme's 8-step ramp + a sun-proximity brightness boost. **No geometry at all** — every cell is a pure function of `(sx, sy, time, pattern, seed, theme)`.

---

## The Mental Model

Read every theme's 8-step ramp as a HORIZON→ZENITH gradient at SUNSET: `ramp[7]` is fierce horizon orange, `ramp[0]` is darkest twilight zenith. To render at noon, slide the gradient: zenith uses a HIGHER index (lighter blue) and horizon uses a slightly lower index. To render at night, both ends collapse to `ramp[0..1]` and stars overlay on top. The sun is just a bright glyph plopped at the sun's screen position; the mountain is just a dark glyph below a sin-of-x curve. Three layers, no shaders.

---

## Algorithm in Steps

1. **SUN POSITION.** Pattern selects a fixed altitude (DAWN, DAY, DUSK, NIGHT) or animates it (TRANSIT: `alt = sin(t · ω)` over a 60 s cycle).

2. **PER FRAME, PER CELL `(sx, sy)`:**
   ```
   a. silhouette_y(sx) → procedural mountain row at this column
   b. if sy >= silhouette_y(sx): render dark earth, continue
   c. view_y_factor = (silhouette_y(sx) - sy) / silhouette_y(sx)   // 0=horizon, 1=zenith
   d. day_factor = smoothstep(-0.10, +0.05, sin(sun_alt))         // night→day
   e. dusk_factor = smoothstep(0.30, 0.05, sin(sun_alt))           // 1 at low sun
   f. horizon_idx = lerp(NIGHT_HORIZON_IDX, lerp(DAY_HORIZON_IDX, DUSK_HORIZON_IDX, dusk_factor), day_factor)
      zenith_idx  = lerp(NIGHT_ZENITH_IDX,  lerp(DAY_ZENITH_IDX,  DUSK_ZENITH_IDX,  dusk_factor), day_factor)
      ramp_idx    = lerp(horizon_idx, zenith_idx, view_y_factor)
   g. sun_prox = exp(-dist²/σ²)                                     // gaussian halo
      ramp_idx += SUN_BOOST · sun_prox · day_factor                 // warm near sun
   h. if cell within sun disc: render bright sun glyph, continue
   i. if cell in cloud band AND fbm(sx,sy + wind) > THRESH: render cloud glyph, continue
   j. if night AND hash3(sx, sy, seed) % STAR_DENS == 0 AND twinkle > THRESH: render star, continue
   k. plain sky: glyph = RAMP_GLYPHS[clamp(intensity·8)], colour = ramp[clamp(ramp_idx)]
   ```

3. HUD on bottom row.

---

## Key Formulas

**Sun direction** (TRANSIT mode):
```
ω        = 2π / TRANSIT_PERIOD_S            // 60 s default
sun_alt  = (π/2) · 0.95 · sin(ω·t + φ)      // [-π/2, π/2]
sun_az   = -(π/2) · 0.9  · cos(ω·t + φ)     // sun arcs east → west
sun_y    = sin(sun_alt)                      // height factor
```

**Mountain silhouette** (per column):
```
silhouette_y(sx) = base_y
                 + amp        · sin(sx·k₁ + φ₁)
                 + amp · 0.4  · sin(sx·k₂ + φ₂·1.7)
                 + amp · 0.2  · sin(sx·k₂·2.3 + φ₂·0.5)
```
Where `base_y = avail · 0.78`, `amp = avail · 0.10`, `k₁ = 0.05`, `k₂ = 0.13`. The phase `φ₁`/`φ₂` is reseeded by `r`.

**Day / dusk smooth fades:**
```
day_factor   = smoothstep(-0.10, +0.05, sun_y)
dusk_factor  = smoothstep( 0.30,  0.05, sun_y)
```

**Sun proximity (gaussian falloff in screen cells):**
```
dx          = sx - sun_sx
dy          = (sy - sun_sy) · ASPECT_Y          // aspect correction
prox        = exp( -(dx² + dy²) / σ² )
```

**Final ramp index lerp:**
```
ramp_f       = lerp(horizon_idx, zenith_idx, view_y_factor)
ramp_f      += SUN_BOOST · prox · day_factor
ramp_idx     = clamp(round(ramp_f), 0, 7)
```

**Glyph density from local intensity:**
```
intensity   = day_factor · (0.4 + 0.6 · (1 - view_y_factor))
            + day_factor · 1.2 · prox
glyph       = RAMP_GLYPHS[clamp(⌊intensity · 8⌋, 0, 7)]
```
Where `RAMP_GLYPHS = " .,:;-+*"` (airy ramp — no blocky `#`/`@`).

**Cloud sampler:**
```
wind   = t · CLOUD_WIND
c      = fbm(sx · 0.04 + wind, sy · 0.10)
if c > 0.55 AND view_y_factor < 0.55: render cloud
```

**Star sampler (night only):**
```
if (1 - day_factor) > 0.5 AND hash3(sx, sy, seed) % STAR_DENS == 0:
  twinkle = 0.5 + 0.5 · sin(2π · t · STAR_TWINKLE_HZ + per_star_phase)
  bright  = (1 - day_factor) · twinkle
  if bright > THRESH: render '*' or '.' in PAIR_STAR
```

---

## Edge Cases and Pitfalls

- **SILHOUETTE LANDS ON HUD ROW.** Cap `silhouette_y` to `rows − 2` so the HUD bottom row is never overwritten.

- **SUN_BELOW_HORIZON SUN BOOST.** Even when the sun sets (sun_alt < 0), the formula still produces a small `prox` near the horizon for above-horizon cells — that's exactly what we want for sunset AFTERGLOW. No special-casing needed.

- **NIGHT FALLBACK.** When `day_factor < 0.05`, the sun-proximity term vanishes (multiplied by `day_factor`). The whole sky collapses to `ramp[0..1]` (darkest theme tints). Stars overlay on top.

- **RAMP IDX OVERFLOW.** After all the lerps and boosts, `ramp_f` can exceed 7 or go below 0. Always clamp to `[0, 7]` before indexing.

- **CLOUD ALTITUDE BAND.** Clouds should appear in the LOWER HALF of the sky (close to horizon), not at the zenith. Restrict the cloud sampler to `view_y_factor < 0.55`.

- **GLYPH PALETTE FOR AIRY LOOK.** Brightest cells use `+`/`*` instead of `#`/`@`. The blocky chars made bright sky cells look like solid pixel walls; the airy chars read as light atmosphere.

- **ASPECT IN SUN PROXIMITY.** Multiply screen-y delta by `ASPECT_Y = 2` so the sun disc is round, not vertically stretched.

---

## How to Verify

- **PAUSE** (space). Sun freezes mid-arc; clouds and twinkles freeze too. Resume: animation continues from where it stopped.

- **DAWN** pattern. Sun visible just above horizon; warm peach belt; blue zenith; mountains in clear silhouette below.

- **DAY** pattern. Sun visible high; uniform blue sky; minimal horizon glow.

- **DUSK** pattern. Sun grazing horizon (or just below); fierce orange band along horizon; sky deepens to twilight purple at zenith.

- **NIGHT** pattern. Black sky; visible stars twinkling; thin red afterglow on horizon for the first few seconds, then darkness.

- **TRANSIT** pattern. Watch the sun arc from low east → high overhead → low west → below horizon over ~60 s. Horizon colour smoothly morphs through the cycle.

- **Theme cycle** (`t`/`T`). Each theme produces a recognisably different atmosphere at the same sun position.

---

## References

- Wikipedia — [Rayleigh scattering](https://en.wikipedia.org/wiki/Rayleigh_scattering).
- O'Neil, S. (2005) — "Accurate Atmospheric Scattering", *GPU Gems 2*, ch. 16.
- Inigo Quilez — Atmosphere demo, https://www.shadertoy.com/view/lslXDr.
- Wikipedia — [Smoothstep](https://en.wikipedia.org/wiki/Smoothstep).

---

*Source: `raytracing/atmospheric_sky.c`*
