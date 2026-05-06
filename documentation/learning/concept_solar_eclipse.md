# Pass 1 — solar_eclipse.c: Two-sphere raytrace + cinematic eclipse layers

## Core Idea

Per pixel: ray vs SUN sphere + ray vs MOON sphere. Depth-sort. The
moon (much closer) wins where it overlaps. The remarkable visual is
that the moon's angular size JUST exceeds the sun's: the bright
photosphere vanishes, leaving the much fainter CORONA visible.

Five additive shading layers stack on top of the geometry:

| Layer | Formula | What it adds |
|---|---|---|
| PHOTOSPHERE  | limb darken · granulation | textured solar disc |
| CORONA       | exp(-Δα·K) · streamer_fbm · occlusion^γ | structured halo |
| CHROMOSPHERE | thin red ring at d_out ∈ [0, W] when occ ≥ 0.92 | "ring of fire" |
| BEADS        | N=5 Gaussian beads around limb at totality boundary | Bailey's beads |
| BLOOM        | 5×5 bright-pass Gaussian over g_buf | atmospheric glow |

## Blackbody Star Presets (no preset themes)

The photospheric chromaticity comes from one Kelvin via Tanner Helland's
piecewise blackbody curve. 5 presets:

| Name | Kelvin | Description |
|---|---|---|
| SUN-LIKE | 5778 | the actual sun (default) |
| RED-G    | 3500 | red giant |
| ORANGE   | 4500 | warm orange star |
| GIANT    | 8000 | hot blue giant |
| WHITE-D  | 10000 | near-white dwarf |

`palette_from_kelvin(K)` derives sun, corona, chromosphere, bead, moon,
sky colours. Corona and chromosphere are LARGELY K-independent (their
light comes from different physical processes than the photosphere —
1 MK plasma emission vs Hα at 656.3 nm). The photosphere IS the
K-dependent blackbody.

## §9 shaders (sub-sectioned)

```
§9.1 shade_sun           limb darkening + fBm granulation in surface coords
§9.2 shade_corona        radial decay × streamer fBm × occlusion^γ
§9.3 shade_chromosphere  thin red Hα band at d_out ∈ [0, CHROMO_W]
                         only visible when occlusion >= 0.92
§9.4 shade_beads         5 Gaussian beads around lunar limb,
                         each at hash3(k, seed)-jittered angle,
                         each with random per-bead brightness,
                         windowed by Gaussian on |sep − sep_T|
```

## Granulation (§9.1)

fBm sampled in SURFACE coordinates (sphere-stable):

```
surface_u = atan2(N.x, N.z) · GRAN_FREQ      ← longitude
surface_v = N.y                · GRAN_FREQ   ← latitude
granul    = 1 + GRAN_AMP · (fbm(u, v) − 0.5) · 2     ≈ 1 ± 18%
intensity = limb · granul
```

Sampling in surface coords means the texture stays GLUED to the sun
as the camera moves; sampling in screen coords would let it slide.

## Coronal Streamers (§9.2)

Old code: uniform radial Gaussian halo.
New: streamer = `(1 + AMP · (fbm(angle·FA, d·FR + drift) − 0.5)·2)^POW`

The fBm in `(polar_angle, d_out)` space produces visible POLAR PLUMES
and HELMET STREAMERS:
- Angle coord creates radial rays (each fBm "ridge" along the angle
  axis makes one streamer)
- Radial coord adds variation along each ray
- Small `drift = time · STREAM_DRIFT_HZ` makes streamers slowly evolve

The angle coord is NOT time-driven — real streamers don't rotate around
the sun (their structure is tied to the sun's magnetic field, evolving
on months-to-years timescales).

## Bailey's Beads (§9.4)

Real beads are 3-7 spots where sunlight passes through valleys between
mountains on the lunar limb. We fake them with 5 Gaussian beads at
fixed (per-seed) angular positions around the sun's limb:

```
window  = exp(-((sep − sep_T) / DIAMOND_W)²)        Gaussian on |sep − sep_T|
for k in 0..4:
  φ_k     = anti_moon_angle + jitter[k]              hash3(k, seed)·π
  bx, by  = sun_centre + sun_r · (cos φ_k, sin φ_k / ASPECT_Y)
  bead_w  = exp(-distance² / RADIUS²)
  bead_intensity_k = 0.4 + hash01(...) · 0.6         per-bead random brightness
  contribution    += window · bead_w · bead_intensity_k · bead_col
```

Beads only fire when `moon_α > sun_α` (TOTAL pattern). For ANNULAR/
PARTIAL/TRANSIT, the totality boundary is never crossed.

## Chromosphere Ring (§9.3)

Thin red Hα band just outside the photosphere, gated by occlusion:

```
if occ < 0.92 OR d_out > CHROMO_W: return 0          only at totality edge
profile = sin(π · d_out / CHROMO_W)                   smooth 0..1..0
ramp    = clamp((occ − 0.92) / 0.08)
chromos = INTENSITY · profile · ramp · chromos_col
```

## Bloom (§10.2)

Same 5×5 bright-pass Gaussian as saturn/god_rays. Two-pass with
separate g_bloom buffer.

## Patterns

- **TOTAL** — moon angular > sun (1.10×). Full totality with corona,
  chromosphere, beads.
- **PARTIAL** — moon offset above so it never centres. Crescent only.
- **ANNULAR** — moon angular < sun (0.80×). Bright RING of photosphere
  remains visible; no corona (drowned out).
- **TRANSIT** — moon size << sun (0.12×). Tiny dot (Mercury / Venus).

## §-section structure

```
§1 config        (sub-sectioned 1.1-1.13)
§2 clock
§3 math+palette  V3, RGB, blackbody, palette_from_kelvin
§4 noise+RNG
§5 ncurses paint
§6 ray-sphere
§7 occlusion
§8 scene+orbit
§9 shading       9.1 sun · 9.2 corona · 9.3 chromos · 9.4 beads
§10 buffer+post  V3 buf + 5×5 Gaussian bloom
§11 screen+draw  3-pass scene_draw + HUD
§12 app
```

## 3-pass scene_draw

```
PASS 1  per cell shade → g_buf
            ray-sphere sun + moon, depth-sort
            sun cell    : shade_sun
            moon cell   : earthshine tint (palette.moon, not pure black)
            space cell  : shade_corona + shade_chromosphere
            beads added everywhere if at totality threshold
            sparse stars during totality (occ > 0.85)
PASS 2  bloom_apply
PASS 3  paint_cell each → screen
```

## HUD spec

- Yellow status row 0 with star preset + Kelvin label + eclipse phase
- Cyan hint bottom row

## How to verify

- Pause at totality. Look for:
  - Corona has visible STREAKY structure (polar plumes, helmet
    streamers) — not a uniform fuzzy halo
  - Thin RED RING just outside the moon (chromosphere)
  - Moon silhouette has faint cool-blue tint (earthshine), not solid black
- At first/last contact (totality threshold):
  - 5 distinct BAILEY'S BEADS appear around the lunar limb at varied
    angular positions and brightnesses, NOT a single bright dot
- Cycle STAR PRESET (t/T):
  - SUN-LIKE 5778K → warm-white sun, cool-blue corona
  - RED-G 3500K → deep-red sun, more crimson everything
  - WHITE-D 10000K → near-white sun, cooler corona
- Look at the SUN (not eclipsed): faint MOTTLED texture (granulation).
- PARTIAL pattern: crescent forms but no totality → no corona, no
  beads, no chromosphere. ANNULAR: bright RING, no corona, no beads.
  TRANSIT: tiny dot crawls across.

---

# Pass 2 — Pseudocode

## Module map

| § | Purpose |
|---|---|
| §1 config       | frame rate, geometry, granulation, corona, beads |
| §2 clock        | monotonic timer + sleep |
| §3 math+palette | V3, RGB, blackbody, palette_from_kelvin |
| §4 noise+RNG    | Perlin + fBm + hash3 |
| §5 ncurses paint | 6×6×6 cube + airy ramp + paint_cell |
| §6 ray-sphere   | analytic intersection (used twice/pixel) |
| §7 occlusion    | sun-moon angular overlap fraction |
| §8 scene+orbit  | Scene state, pattern params, moon orbit |
| §9 shading      | 9.1 sun · 9.2 corona · 9.3 chromos · 9.4 beads |
| §10 buffer+post | V3 buf + 5×5 Gaussian bloom |
| §11 screen+draw | 3-pass scene_draw + HUD spec compliance |
| §12 app         | signals, resize, fixed-step main loop |

## Data flow

```
keys → app_handle_key → scene state, star_preset, pattern
                              │
clock_ns → dt → scene_tick (orbit phase, flash decay)
                              │
                       scene_draw:
                          build palette from K
                          per cell:
                            ray-sphere sun + moon, depth-sort
                            sun → shade_sun (limb + granulation)
                            moon → palette.moon (earthshine)
                            space → corona + chromosphere
                            beads + stars overlay
                          bloom_apply
                          paint each → screen
```

## Key patterns to internalise

**Five additive layers, each gated physically.** Limb darkening
multiplies; corona is multiplied by occlusion^γ so it's invisible until
totality; chromosphere gated by occlusion threshold; beads gated by
totality boundary window.

**Granulation in surface coords.** Sample fBm at (atan2(N.x,N.z), N.y)
not (cellx, celly), so the texture stays glued to the sphere as it
moves.

**Streamer fBm in (angle, radius) space.** Angle creates radial rays;
radius adds variation. Don't drift the angle coord — real streamers
don't rotate.

**Multiple beads beat one bead.** 5 Gaussian beads with per-bead
random brightnesses are vastly more realistic than a single bright dot
at "the silver edge."

**Same paint pipeline + bloom as saturn/god_rays.** Continuous RGB →
Reinhard → gamma → 6×6×6 cube + airy ramp.
