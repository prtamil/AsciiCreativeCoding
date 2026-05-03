# Concept — `god_rays_silhouette.c`: Volumetric Light Shafts (Screen-Space)

## Core Idea

Each cell on the screen asks the sun: "Is there a clear line from you to me, through the fog?" The answer comes from MARCHING. Walk N small steps from the cell toward the sun's screen position. At each step, ask: "Is this step inside the silhouette?" Count the unblocked steps, weighted by how far from the eye they are (closer = more contribution because the `exp(-σ·d)` extinction is gentler). The total is how much light gets through.

Bright SHAFTS emerge wherever the line from cell to sun threads a CLEAR channel through the silhouette; dark fog fills the cells where every step lies inside the silhouette.

This is the GPU-engine "screen-space god rays" trick (Crysis-era), reduced to its purest form. The shadow ray collapses to a point-in-shape test because the silhouette is essentially co-planar with the camera; same divergent-cone visual, tiny fraction of the cost of full 3-D shadow rays.

---

## The Mental Model

Picture a dim hallway. There's a door at one end with light coming through a keyhole. Dust hangs in the air. From your perspective at the other end, the dust forms a bright cone radiating out from the keyhole — the divergent shaft you recognise as "god rays".

At a cell directly along the keyhole-to-eye line, every dust mote on that line is illuminated (the keyhole sees those motes), so the shaft is brightest. At a cell off-axis, only the dust motes near the door see the keyhole; the rest sit in the door's shadow. So off-axis cells are dimmer. Result: a luminous cone of dust diverging from the keyhole. Replace "keyhole" with "gap in the silhouette" and you have this demo.

---

## Algorithm in Steps

1. **SUN POSITION.** Per frame, compute `(sun_sx, sun_sy)` — slow horizontal drift across the upper portion of the screen so shaft directions change over time.

2. **PER CELL `(sx, sy)`:**
   - Convert `(sx, sy)` to normalised aspect-corrected `(u, v) ∈ [-1, +1]²`.
   - If `silhouette(u, v)` → render BLACK silhouette glyph and continue.
   - Compute step vector toward sun:
     ```
     step_dx = (sun_sx − sx) / MARCH_STEPS
     step_dy = (sun_sy − sy) / MARCH_STEPS
     step_len = √(step_dx² + (step_dy · ASPECT_Y)²)
     ```
   - March N steps:
     ```
     for i = 1..MARCH_STEPS:
       px, py = sx + step_dx · i, sy + step_dy · i
       d = i · step_len                  // distance from eye in cells
       w = exp(−FOG_SIGMA · d)
       if not silhouette(px, py): accum += w
       total_w += w
     visibility = accum / total_w
     ```
   - Add **sun-disc contribution**: gaussian falloff if cell near sun screen position AND sun isn't behind silhouette.
   - Multiply by **fog jitter** (subtle fBm density variation — light shafts shimmer slightly as wind moves the dust).
   - Map intensity → glyph + theme ramp colour.

3. HUD on bottom row.

---

## Key Formulas

**Aspect-corrected normalised cell coords:**
```
u = (2 · sx + 1 − cols) / cols
v = (2 · sy + 1 − rows) / rows · (rows · ASPECT_Y / cols)
```

**Step vector toward sun** (in cell coordinates):
```
Δsx       = (sun_sx − sx) / MARCH_STEPS
Δsy       = (sun_sy − sy) / MARCH_STEPS
step_len  = √(Δsx² + (Δsy · ASPECT_Y)²)        // visual distance per step
```

**Sample weight at step i** (Beer-Lambert):
```
d_i = i · step_len
w_i = exp(−FOG_SIGMA · d_i)
```

**Visibility accumulation:**
```
accum   = Σ_{i: silhouette FALSE} w_i
total_w = Σ_{i in 1..N}            w_i
vis     = accum / total_w                       // ∈ [0, 1]
```

**Sun-disc contribution** (`sx,sy` near `sun_sx,sun_sy`):
```
dx, dy = sx − sun_sx, (sy − sun_sy) · ASPECT_Y
r²     = dx² + dy²
sun    = exp(−r² / SUN_FALLOFF²) · (1 − sun_in_silhouette)
```

**Fog wind** (subtle visual modulation):
```
wind        = time · FOG_WIND
fog_jitter  = 1 + FOG_JITTER_AMP · 2 · (fbm(u·1.4 + wind, v·1.4) − 0.5)
```

**Total intensity:**
```
intensity = (vis · SHAFT_GAIN + sun · SUN_GAIN) · fog_jitter
glyph     = RAMP_GLYPHS[clamp(⌊intensity · 8⌋, 0, 7)]
```

---

## Silhouette Functions

All return `true` if the point is INSIDE the silhouette. Coordinates `(u, v) ∈ [-1, +1]²` aspect-corrected.

| Pattern   | Definition |
|-----------|------------|
| ARCHWAY   | Two pillar rectangles at u=±0.40, half-annulus arch top for v<0, capstone block, ground line |
| MOUNTAIN  | `v > 0.70 − 1.10·exp(−u²/0.30) − 0.35·exp(−(u−0.55)²/0.05)` (gaussian peak + side bump) |
| COLUMN    | Tall capped rectangle around u=0, cosine-jagged broken top, ground line |
| WINDOWS   | 4×2 grid of arched windows cut from a wall (rectangular body + half-circle top) |
| TREE      | Tapered trunk + 6 capsule branches at varying angles with linear taper |

---

## Edge Cases and Pitfalls

- **CELL ON SUN.** When `(sx, sy) ≈ (sun_sx, sun_sy)`, step vector is near-zero and N samples coincide. The sun-disc gaussian dominates this region, so the visibility result doesn't matter — but if you cared, early-out with `vis = 1` when `dist < 1 cell`.

- **CELL INSIDE SILHOUETTE.** The silhouette test gates everything: inside cells render solid black with NO march. Otherwise march might yield "visible to sun through the silhouette behind", which makes no sense.

- **SHAFT BACKLIT BY OWN OCCLUDER.** The march samples points BETWEEN the cell and the sun. The cell itself is OUTSIDE the silhouette (gated above). If the silhouette is between cell and sun, samples will fall inside it → visibility drops → cell stays dark. That's exactly the SHADOW behind the silhouette. No special-casing needed.

- **SAMPLE POINT GOES OFF-SCREEN.** The march can sample `(px, py)` outside `[0, cols)×[0, rows)`. Silhouette functions are defined in normalised coords, so they work just as well off-screen — points outside the silhouette area render as "clear sky". (Treating off-screen as silhouette would cut off the shaft fans at screen edges.)

- **ASPECT IN STEP-LENGTH.** Terminal cells are ~2× taller than wide. The "visual length" of one step needs `ASPECT_Y` multiplied into the y component, otherwise vertical shafts look longer (more samples = more accumulation) than they should. `step_len` uses `ASPECT_Y · Δsy` in its sqrt.

- **WINDOW ALIGNMENT.** The WINDOWS pattern's holes must be arched/rectangular consistently. The grid of holes is parameterised on `(gu, gv)` within each cell; if the cell arithmetic floors incorrectly at exact boundaries, you get one-pixel-wide false windows. Use a small margin in the boundary tests to avoid this.

- **PERFORMANCE.** `MARCH_STEPS · cols · rows` is the per-frame silhouette-evaluation count. At 20 steps · 240 · 80 ≈ 384 k silhouette evals per frame. Each is a few branches. Modern CPUs handle this in ~5-15 ms. If you push terminal size to 400×120, drop `MARCH_STEPS` to 14 to maintain 30 fps.

---

## How to Verify

- **PAUSE** (space). Sun freezes; shafts stop drifting. Resume: sun continues smoothly.

- **ARCHWAY** pattern. Two pillars visible as black rectangles; curved top visible as a black half-disc. A bright shaft fans out UNDER the arch toward the camera, bright on the line from the arch interior to the sun, dim on the sides.

- **MOUNTAIN** pattern. Black peak fills the lower portion. A bright fan of light spills OVER the ridge wherever the sun is positioned above the ridgeline.

- **WINDOWS** pattern. Several thin parallel shafts streaming through the windows in the cathedral wall — one shaft per window, all converging back toward the sun.

- **TREE** pattern. The trunk and branches form a black silhouette. Many thin shafts thread between the branches (where the sun is visible through the gaps); branches cast sharp dark stripes.

- **Theme cycle** (`t`/`T`). Each theme should produce a recognisably different fog colour while shaft structure stays identical.

- **Speed** (`+`/`−`). Doubling speed should approximately halve the period between sun's leftmost and rightmost positions.

---

## References

- Mitchell, K. (2007) — "Volumetric Light Scattering as a Post-Process", *GPU Gems 3*, ch. 13. The screen-space god-rays technique this demo's algorithm is a direct descendant of.
- Hoffman, N. & Preetham, A. — "Real-time Light Atmosphere Interactions for Outdoor Scenes", *Game Programming Gems 5*.
- Wikipedia — [Crepuscular rays](https://en.wikipedia.org/wiki/Crepuscular_rays). The atmospheric phenomenon ("god rays") this demo simulates.
- Wikipedia — [Beer-Lambert law](https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law). The `exp(-σ·d)` extinction law that weights samples.

---

*Source: `raytracing/god_rays_silhouette.c`*
