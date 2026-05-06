# Pass 1 — god_rays_silhouette.c: Volumetric god rays via screen-space ray-march

## Core Idea

Each cell asks the sun: "is there a clear line from you to me, through
the fog?" Walk N small steps from the cell toward the sun. Each step
"lit" iff outside the silhouette, weighted by Beer-Lambert `exp(-σ·d)`.
The weighted fraction of unblocked steps is the cell's visibility ∈ [0, 1].
Bright shafts emerge where the line threads a CLEAR channel.

Five additive layers stacked on top of the visibility computation make
the demo cinematic:

| Layer | Effect |
|---|---|
| COLOUR    | Continuous RGB from blackbody temperature (no preset themes) |
| SUN       | Cinematic disc = CORE + CORONA halo + 4 LENS-FLARE STREAKS |
| BLOOM     | Bright cells leak warm light into 5×5 neighbours |
| DUST      | Sparkle particles drift through shafts with motion-blur trails, gusted by Perlin wind |
| RAMP      | Curated airy 16-glyph density ramp (no closed blocks like # or @) |

## Three shade modes (cycled with `m`)

| Mode | Visual |
|---|---|
| LIT   | Full pipeline (default): RGB + bloom + dust |
| MASK  | Silhouette black + flat fog tint + sun marker — see the SHAPE |
| VIS   | Grayscale visibility map — see the algorithm OUTPUT |

## Blackbody Palette (no preset themes)

A single Kelvin temperature drives the entire scene chromaticity via
Tanner Helland's piecewise blackbody approximation. 8 presets:

| Name | K | Description |
|---|---|---|
| EMBER  | 1500 | glowing-coal red |
| SUNSET | 2000 | orange sunset |
| CANDLE | 2500 | warm candle flame |
| TORCH  | 3500 | tungsten bulb |
| WARM   | 4500 | golden hour |
| DAY    | 5500 | midday sun (D55) |
| NOON   | 6500 | clear noon (D65) |
| BLUE   | 8500 | overcast / blue hour |

`palette_from_kelvin(K)` derives shaft, fog, dust, sky colours from the
sun colour with small offsets for cool ambient blue. There are no
"theme" tunings — chromaticity is physically grounded.

## Cinematic Sun (§7.3 sun_disc)

Three additive components:

```
core   = exp(-r²/CORE_FALLOFF²)                       sharp Gaussian
corona = exp(-r²/CORONA_FALLOFF²) · CORONA_GAIN        wider warm halo
flare  = Σ_{s=0..3} exp(-Δa²/ANG_SIGMA²)·exp(-r²/R_FALLOFF²)
                                                       4 angular streaks
```

Flare angles 0°, 45°, 90°, 135° (bidirectional → 8 visible spokes).
Skip flare contribution when r < 0.5 (atan2(0,0) undefined; core
dominates anyway).

## Dust particles (§9.2 dust)

220 particles with 4-sample motion trails:

```
position drift  : (DUST_SPEED_X, DUST_SPEED_Y) cells/sec, gusted by wind
trail recording : distance-spaced (every TRAIL_SPACING = 0.6 cells)
brightness      : sin(π·age/life) envelope · DUST_BRIGHTNESS · visibility
trail falloff   : 100% / 55% / 32% / 18% / 10%
visibility gate : g_vis[gy][gx] — particles in shadow contribute nothing
```

Distance-spaced (not frame-spaced) trail recording keeps trail length
visually consistent across frame rates.

## Wind gusts (§9.2 wind_gust)

```
gust = 1 + WIND_GUST_AMP · perlin2d(time · WIND_GUST_FREQ_HZ)   ≈[0.5, 1.5]
dust velocity *= gust
```

10-second gust period; particles ebb and flow organically.

## Bloom (§9.3 bloom_apply)

5×5 bright-pass Gaussian (`luma > 0.55` threshold). Two-pass with
SEPARATE g_bloom buffer (single-pass would compound asymmetrically as
later cells read already-bloomed earlier cells). After the pass, fold
g_bloom into g_buf.

## Five silhouette patterns (§5)

ARCHWAY · MOUNTAIN · COLUMN · WINDOWS · TREE — each is a pure
point-in-shape function `silhouette_at(p, u, v) → bool`.

## Curated airy ramp

Old Bourke 92-char ramp included alphabetic chars (`g`, `B`, `M`, etc.)
that read as TEXT NOISE. New ramp ` .'`,-_:;~=+*oO0` (16 chars,
symbol-only) ends at OPEN circles (`o`, `O`, `0`) — luminous-feeling.
Closed blocks `#` `@` deliberately excluded.

## §-section structure

```
§1 config       (sub-sectioned 1.1-1.13)
§2 clock
§3 math + palette (V3, RGB, blackbody, palette_from_kelvin)
§4 ncurses paint (216-cube + airy ramp + paint_cell)
§5 silhouette (5.1-5.6: 5 patterns + dispatcher)
§6 noise + RNG
§7 scene + sun motion
§8 raymarch (8.1-8.5: cell_to_uv, march_visibility, sun_disc, fog_jitter, shade_lit_rgb)
§9 buffer + post-processing (9.1 buffer, 9.2 dust, 9.3 bloom)
§10 screen + scene_draw (4-pass) + HUD
§11 app
```

## 4-pass scene_draw (LIT mode)

```
PASS 1  per cell → V3 colour into g_buf
                   ALSO: stash visibility into g_vis (for dust gating)
PASS 2  dust_apply → fold particles into g_buf (gated by g_vis)
PASS 3  bloom_apply → 5×5 bright-pass Gaussian
PASS 4  paint each g_buf[r][c] via paint_cell
```

MASK and VIS short-circuit before PASS 1.

## HUD spec

- Yellow status row 0: fps, Hz, pattern, "DAY 5500K" temperature label,
  mode, bloom on/off, dust on/off, speed
- Cyan hint bottom row

## Worked numerical sample

GOLDEN-equivalent at 5778K, shaft-core cell with vis=0.85, sun_term=0.10,
jitter=1.05:

```
palette.fog   = (0.18, 0.14, 0.10)
palette.shaft = (1.00, 0.78, 0.42)
palette.sun   = (1.00, 0.95, 0.70)

base = lerp(fog, shaft, 0.85·1.20) ≈ shaft (clamp at 1.0)
sun  = sun_col · 0.10 · 1.50 = (0.150, 0.142, 0.105)
col  ≈ (1.05, 0.84, 0.48) HDR

Reinhard:    (0.51, 0.46, 0.32)
Gamma 2.2:   (0.74, 0.71, 0.59)
Cube:        r5=4, g5=4, b5=3 → pair 172
Luma 0.71 → ramp idx 11 → '+' glyph
```

## How to verify

- Toggle BLOOM (`b`): bright shaft cores noticeably "glow into"
  neighbours when bloom is on.
- Toggle DUST (`d`): tiny warm sparkles drift across the scene, only
  visible in shafts. Watch a single particle to see motion-blur trail.
- Cycle TEMPERATURE (`t`/`T`): scene chromaticity shifts continuously.
  EMBER 1500K → deep red; NOON 6500K → near-white; BLUE 8500K → cool.
- Sun has visible CORE + CORONA + 4 SPARKLY STREAKS, not just a
  Gaussian dot.
- Wind gusts: at 10+ sec timescales, dust speed visibly ebbs and flows.

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, march, sun, bloom, dust |
| §2 clock        | monotonic timer + sleep |
| §3 math+palette | V3, RGB, blackbody, palette_from_kelvin |
| §4 ncurses paint | 6×6×6 cube + airy ramp + paint_cell |
| §5 silhouette   | 5 patterns + dispatcher |
| §6 noise+RNG    | Perlin + fBm + xorshift + hash3 |
| §7 scene+sun    | Scene state, sun motion |
| §8 raymarch     | THE CORE — visibility + sun_disc + fog_jitter + shade_lit_rgb |
| §9 buffer+post  | V3 buf, dust trails, wind, bloom |
| §10 screen      | scene_draw (4-pass LIT) + HUD |
| §11 app         | signals, resize, fixed-step main loop |

## Data flow (LIT mode)

```
keys → app_handle_key → scene state
                              │
clock_ns → dt → scene_tick
                              │
                       scene_draw:
                          PASS 1: per cell shade → g_buf, g_vis
                          PASS 2: dust_apply (gated by g_vis)
                          PASS 3: bloom_apply (5×5 Gaussian bright-pass)
                          PASS 4: paint each cell to ncurses
```

## Key patterns to internalise

**Beer-Lambert weighted visibility.** The march sums exp(-σ·d) per
unblocked step; closer steps weigh more. That's why shafts spread OUT
from the sun rather than filling the whole half-screen.

**Continuous RGB pipeline beats preset ramps.** Saturn's lesson, applied
again. ~20 000 effective shades per cell.

**Blackbody temperature = physical chromaticity.** No "theme" tuning;
sun colour is determined by one Kelvin number, derived colours follow.

**Bloom = bright-pass Gaussian + separate buffer.** Two-pass to avoid
feedback compounding.

**Dust = particle system + visibility gate.** Particles only sparkle
inside shafts (multiply contribution by g_vis at the cell). Distance-
spaced trail recording for frame-rate independence.

**Wind gust via slow Perlin.** Dust velocity multiplied by `1 + AMP·
perlin(time)` for organic ebb-and-flow.

**Curated airy ramp.** Open glyphs only — bright cells read as POINTS
OF LIGHT not solid pixels.
