# Concept — `solar_eclipse.c`: Sun + Moon Transit with Volumetric Corona

## Core Idea

Two spheres on (nearly) the optical axis. Whichever is closer along the view ray covers the other. The remarkable visual of an eclipse is caused by the moon being JUST big enough — angularly — to swallow the sun's bright disc, leaving the much fainter CORONA visible for a few seconds of totality. We reproduce that by keeping corona brightness at zero whenever the bright sun body is visible (the eye adapts to the photosphere and corona becomes invisible) and unlocking it only as the moon occludes more and more of the sun.

---

## The Mental Model

The sun is a desk lamp behind a paper-thin halo of mist (the corona). Normally the bulb is too bright to see the mist. Slide a small piece of card in front of the bulb — the card blocks the bulb but the mist around the silhouette becomes visible. That visibility ratio is the **SUN OCCLUSION FACTOR**.

The card has to be JUST big enough; too small and the bulb still outshines the mist (annular eclipse: bright RING around the card); a tiny card makes no difference (transit). The DIAMOND RING is the moment a sliver of bulb is still visible at the card's edge — a tiny intense flash before the card fully blocks everything.

---

## Algorithm in Steps

1. **ORBIT.** Moon position drifts in world x via `sin(ω·t)`. For PARTIAL pattern, add a small constant y offset so the moon never centres on the sun.

2. **CAMERA.** Camera at origin looking +z. Sun centred at `(0, 0, SUN_Z)`. Moon at `(moon_x, moon_y, MOON_Z)` with `MOON_Z << SUN_Z` (moon is much closer — that's why a small moon can occlude a vastly larger sun).

3. **PER CELL `(sx, sy)`:**
   - Build view ray `d = camera_ray(sx, sy)`.
   - `ray_sphere(sun)` → `t_sun`.
   - `ray_sphere(moon)` → `t_moon`.
   - `moon_in_front = moon_hit AND (¬sun_hit OR t_moon < t_sun)`.
   - If `moon_in_front`: render moon silhouette (dark glyph), continue to corona overlay.
   - Else if `sun_hit`: shade sun with limb darkening.
   - Else: pure space — corona overlay computes brightness.

4. **CORONA OVERLAY** (every space cell):
   ```
   α_view = acos(ray_d · sun_dir)
   d_out  = max(0, α_view − sun_α)
   local  = exp(−d_out · CORONA_K)
   global = pow(occlusion, CORONA_GAMMA)         // γ = 4
   corona = local · global · CORONA_BOOST
   ```

5. **DIAMOND RING** (TOTAL only). When `|sep − |moon_α − sun_α|| < DIAMOND_W` AND `moon_α > sun_α`: a bead lights up at the sun edge OPPOSITE the moon. Fades smoothly on either side of the threshold.

6. **RENDER.** Pick glyph from intensity ramp + colour from theme ramp (sun, corona, moon, space pairs).

---

## Key Formulas

**Moon orbit** (pattern-specific):
```
moon_x = MOON_ORBIT_R · sin(ω · t)              ω = 2π / PERIOD
moon_y = MOON_Y_OFFSET                          (PARTIAL ≠ 0)
```

**Apparent angular radii (small-angle):**
```
sun_α  = atan(SUN_R  / SUN_Z)
moon_α = atan(MOON_R / MOON_Z)
```

**Sun-moon angular separation** (per frame):
```
cos β  = normalize(sun_pos) · normalize(moon_pos)
sep    = acos(cos β)
```

**Sun occlusion factor** (overlap fraction):
```
if sep ≥ sun_α + moon_α                : occlusion = 0
else if sep ≤ |sun_α − moon_α|         : if (moon_α ≥ sun_α): 1
                                          else: (moon_α/sun_α)²       // ANNULAR max
else                                    : (sun_α + moon_α − sep)
                                          / (2·min(sun_α,moon_α))
```

**Limb darkening** (Eddington's approximation, simplified to 1-coefficient law):
```
cos μ  = N · −ray_d                              (N = sphere normal)
L      = LIMB_AMBIENT + LIMB_GAIN · cos μ        // 0.40 + 0.60 · μ
```

**Corona brightness at view ray:**
```
α_view = acos(ray_d · sun_dir)
d_out  = max(0, α_view − sun_α)
local  = exp(−d_out · CORONA_K)                  // K = 16
global = pow(occlusion, CORONA_GAMMA)            // γ = 4 — corona blooms only at totality
corona = local · global · CORONA_BOOST           // 1.40
```

**Diamond-ring bead** (TOTAL only):
```
sep_T  = |moon_α − sun_α|                        // totality boundary
if |sep − sep_T| < DIAMOND_W AND moon_α > sun_α:
  bead_dir_2D = −normalize(moon_screen − sun_screen)
  bead_pos    = sun_screen + bead_dir_2D · sun_r_screen
  strength    = 1 − |sep − sep_T| / DIAMOND_W    // smooth fade
```

---

## Edge Cases and Pitfalls

- **DEPTH MUST BE TESTED.** The moon MUST be closer to the camera than the sun, otherwise the depth-sort never lets the moon win. Set `MOON_Z` to a small fraction of `SUN_Z`. Also make the moon's WORLD radius small enough that its angular size is only ~1.0× the sun's (TOTAL) — too big and the moon dominates everything.

- **OCCLUSION ZERO INSIDE TOTALITY.** When `sep < |sun_α − moon_α|` AND `moon_α > sun_α`, the sun is FULLY covered → `occlusion = 1`. For ANNULAR (moon_α < sun_α) the sun ring ALWAYS shows; max occlusion is `(moon_α/sun_α)² < 1`, so corona never reaches full brightness — exactly right (annular eclipses don't show corona).

- **CORONA CLIPS INSIDE MOON.** Corona is computed for ALL cells but cells INSIDE the moon's silhouette must NOT show corona — corona is the OUTER atmosphere, visible only outside the occluder. Gate corona rendering by "this cell is OUTSIDE both spheres" → only over space pixels.

- **ASPECT RATIO.** Terminal cells are 2× taller than wide. The `fov_v` computation accounts for this so the sun renders ROUND on screen — without it, sun looks like a vertical ellipse.

- **DIAMOND-RING WINDOW.** The window `|sep − sep_T| < DIAMOND_W` is in radians. With `DIAMOND_W = 0.0035` and `sep` changing by ~0.003 rad/sec at speed 1, the window fires for ~0.7 sec each time it's crossed — visible but not lingering.

- **PARTIAL: NEVER FULL TOTALITY.** With `MOON_Y_OFFSET > 0` the minimum `sep > 0`, so the inner condition `sep ≤ |sun_α − moon_α|` never triggers, occlusion peaks below 1, corona stays dim, diamond ring never fires. Good.

- **TRANSIT (tiny moon).** Occlusion is ALWAYS tiny, so corona factor stays 0; no diamond ring. Moon just appears as a small dark dot drifting across a brightly lit sun.

---

## How to Verify

- **PAUSE** (space). Eclipse freezes mid-cycle. Resume: orbit continues from where it stopped.

- **TOTAL** pattern. Watch one full cycle:
  - pre-eclipse — bright sun
  - moon enters from one side
  - crescent shrinks until it's a thin sliver
  - DIAMOND RING flashes — bright bead at sun's far edge
  - TOTALITY — sun body dark, corona blooms outward
  - DIAMOND RING flashes again on the other side
  - moon exits — sun reappears

- **PARTIAL** pattern. Crescent forms but the moon never centres. No corona, no diamond.

- **ANNULAR** pattern. At maximum the moon sits INSIDE the sun silhouette and a bright RING of sun remains visible all the way around. The bright ring is what makes annular eclipses famous; corona is invisible because the bright ring outshines it.

- **TRANSIT** pattern. A tiny dark dot crawls across the sun's face. Sun's brightness barely changes. Mercury / Venus.

- **HUD phase string.** Live-updates: `PRE/POST → APPROACH → TOTALITY/ANNULARITY → DEPART`. Use this to verify the phase machine matches the visual state.

---

## References

- Wikipedia — [Solar eclipse](https://en.wikipedia.org/wiki/Solar_eclipse).
- Wikipedia — [Corona (the sun's outer atmosphere)](https://en.wikipedia.org/wiki/Corona).
- Wikipedia — [Baily's beads / diamond ring effect](https://en.wikipedia.org/wiki/Baily%27s_beads).
- Hestroffer, D. & Magnan, C. (1998) — "Wavelength dependency of the solar limb darkening", *Astronomy & Astrophysics* 333:338.
- Shirley, P. — *Ray Tracing in One Weekend* — canonical ray-sphere intersection.

---

*Source: `raytracing/solar_eclipse.c`*
