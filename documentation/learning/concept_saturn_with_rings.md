# Pass 1 — saturn_with_rings.c: Two-primitive raytrace + RGB realism layers

## Core Idea

Per pixel: ray vs SPHERE (planet) + ray vs ANNULUS (ring plane y=0
clipped to [R_in, R_out]). Depth-sort. Shade in continuous RGB with
five additive layers stacked on top of the geometry:

| Layer | Effect |
|---|---|
| LAMBERT      | direct sunlight on planet (N·L) |
| LIMB DARKEN  | atmospheric absorption near silhouette (`pow(NdotV, 0.5)`) |
| ATMOSPHERE RIM | warm halo on lit silhouette edge |
| FORWARD-SCATTER | ring sections glow when sun is BEHIND them (Cassini look) |
| SOFT SHADOW  | 8-sample penumbra cast by planet onto rings |

## RGB Pipeline

Every shader returns a `V3` colour. Final paint:
1. Reinhard tone-map `L/(1+L)` per channel
2. Gamma encode 1/2.2
3. Quantise to 6×6×6 RGB cube (216 ncurses pairs)
4. Pick density glyph from 92-char Bourke ramp
5. A_BOLD on bright cells, A_DIM on dark

Effective resolution: ~20 000 distinct visual states per cell.

## Themes (RGB triplets, not pair indices)

5 themes — SATURN, MARS, OCEAN, FOREST, FIRE, ARCTIC, VIOLET, GOLD —
each provides RGB triplets per role: planet_base, planet_band_tint,
ring_base, sun_col, fill_col, rim_col, spec_col, sky_col, land_col,
sea_col.

## Three shade modes (cycled with `m`)

| Mode | Visual |
|---|---|
| LIT     | Full pipeline (default) |
| FLAT    | Raw albedo, NO lighting — bands + continents still show |
| NORMAL  | RGB-encoded surface normal (diagnostic) |

FLAT is the "see geometry without lighting" mode; NORMAL is the
debugging mode for verifying normal orientation.

## §9 shaders (sub-sectioned)

```
§9.1 limb_darken + atmospheric_rim   helpers
§9.2 shade_planet                    Lambert + fill + spec + limb + rim
                                      Bands (SATURN/EXO) or continent fBm (EARTH)
                                      Diagnostic-mode early returns for FLAT/NORMAL
§9.3 shade_ring                       Smooth Cassini gap (smoothstep, not hard)
                                      Soft shadow (8 cone samples)
                                      Forward-scattering glow (backlight^3)
                                      Diagnostic returns for FLAT/NORMAL
§9.4 shade_space                      Sky + warm/cool stars by hash hue
```

## Forward-scattering ring glow

```
backlight = max(0, -sun_dir · view_dir)        ← > 0 when sun behind
glow      = backlight^3 · (1 − density) · GAIN
col       += glow · sun_col
```

Sparser ring sections glow MOST brightly when sun is on the opposite
side — exactly matching Cassini's iconic backlit photographs.

## Soft shadow (8 samples)

Tangent basis around `sun_dir`; 8 jittered samples within a small
angular cone (3° = 0.05 rad). Returns fraction unobstructed → smooth
penumbra. Deterministic seed `hash3(sx, sy, frame, sample_i)` so the
shadow doesn't "boil" between frames.

## Sub-pixel anti-aliasing

`s` key cycles SPP=1/2/4. Sub-pixel jitter from a deterministic hash;
N samples averaged in linear RGB BEFORE tone-mapping.

## Patterns + Star presets

- **Patterns** (n / N): SATURN, URANUS, EARTH-RING, EXO (per-seed)
- **Star presets** are coupled into themes via the palette structure

## Worked numerical sample (lit equator pixel, SATURN theme)

```
N        = (0.95, 0, 0.31)
sun_dir  = (1, 0.18, 0)
view_dir = (0, 0.22, -0.97)
albedo   = (0.95, 0.86, 0.65)        cream
sun_col  = (1.00, 0.92, 0.78)        warm white

NdotL = 0.95, NdotV = 0.30
limb  = pow(0.30, 0.5) ≈ 0.55

diffuse_RGB = albedo · NdotL · sun_col ≈ (0.903, 0.752, 0.482)
col_after_limb ≈ (0.523, 0.435, 0.281)

Reinhard:    (0.34, 0.30, 0.22)
Gamma 2.2:   (0.61, 0.57, 0.49)
Cube:        r5=3, g5=3, b5=2 → pair 130 (warm cream-gold)
Luma 0.57 → ramp index 52 → 'k'
```

## HUD spec

- Yellow status row 0 with mode label, theme, SPP, sun-azimuth
- Cyan hint bottom row

## How to verify

- Static planet at full lit phase: limb darkening makes the silhouette
  noticeably DIMMER than the centre. With limb factor = 1.0 (no
  darkening), the disc looks flat.
- At totality-equivalent (sun behind the rings from camera POV): ring
  sections away from the moon should GLOW brightest, especially in
  the Cassini-gap region (low density).
- Shadow penumbra: the dark stripe on the rings should have a SMOOTH
  edge (gradient), not a stair-step boundary.
- Sub-pixel AA: `s` cycles SPP=1/2/4. SPP=1 shows visible jaggies on
  the planet silhouette; SPP=4 smooths them.
- Cycle MODE: FLAT shows pure albedo (bands and continents visible
  uniformly); NORMAL shows rainbow ball with ring-angle hue rainbow.

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, geometry, granulation, corona, beads |
| §2 clock        | monotonic timer + sleep |
| §3 math         | V3, RGB, blackbody helpers |
| §4 noise        | Perlin + fBm + RNG |
| §5 themes       | RGB-triplet palettes per role (per planet/ring/sun) |
| §6 color        | 216-cube + paint_cell (Reinhard + gamma + quantise) |
| §7 patterns     | pattern → params mapping |
| §8 raytrace     | 8.1 ray_sphere · 8.2 ray_ring · 8.3 hard_shadow · 8.4 soft_shadow |
| §9 shading      | 9.1 limb · 9.2 planet · 9.3 ring · 9.4 space |
| §10 scene       | Scene state, sun motion, init/reseed/tick |
| §11 render      | per-cell ray + depth-sort + shading + paint |
| §12 hud         | yellow row 0 + cyan bottom |
| §13 app         | signals, resize, fixed-step main loop |

## Data flow (LIT mode)

```
keys → app_handle_key → scene state
                              │
clock_ns → dt → scene_tick
                              │
                       scene_draw (per cell):
                          - depth-sort sphere vs ring
                          - sphere → shade_planet (or albedo/normal in FLAT/NORMAL)
                          - ring   → shade_ring (or albedo/normal-encoded)
                          - else   → shade_space
                          - SPP averaged
                          - paint_cell (RGB → cube + ramp)
```

## Key patterns to internalise

**Continuous-RGB over preset ramps.** Each shader returns a V3 RGB; the
paint function quantises ONCE at draw time. ~20 000 effective shades
per cell, no banding.

**Layer additively, gate physically.** Each effect is a small additive
contribution to the RGB. Limb darkening MULTIPLIES at the end (the
atmosphere absorbs everything proportionally), but shadow gating runs
INSIDE the diffuse term (because shadows block direct sun, not ambient).

**Soft shadow = N samples in a small angular cone.** Deterministic
hash-based offsets so the penumbra is stable frame-to-frame.

**Forward-scatter is single dot product.** `max(0, -sun·view)^p` is
the entire physics — but it's the single biggest "wow" effect.

**Same paint pipeline as the entire raytracer folder.**
