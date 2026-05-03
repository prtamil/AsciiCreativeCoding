# Concept — `cloud.c`: Procedural Cloud Layers (fBm Stack)

## Core Idea

To draw a cloud, do not draw a cloud — draw the THRESHOLD of a smooth scalar field. Fractional Brownian motion gives you a scalar field that has interesting features at every scale; ask "is this field over 0.6 here?" and the YES regions look exactly like clouds, because clouds in nature are exactly that: regions where some scalar (water-vapor density) exceeds a threshold. The MORPHOLOGY (puffy vs wispy vs banded) is just the SHAPE you sample the field at — isotropic spheres, horizontally stretched ovals, vertically stacked bands.

---

## The Mental Model

Imagine a sheet of cotton ball stuffing. It has a lumpy density — thicker in some places, thinner in others, with structure at many scales (small fibres clump into tufts which clump into wads). Now cut a 2-D cross-section through it. The lumps you see are the density's THRESHOLD CONTOURS — places where the cotton crosses some fixed threshold. Slide a window over the cross-section: it looks like clouds drifting.

That's literally the algorithm. fBm is the cotton-density function; thresholding turns it into binary "cloud / not-cloud"; soft thresholding (multiple bands) gives shaded glyphs. The wind is just the cross-section sliding over the cotton.

Different cloud TYPES come from how you tilt and squeeze the cotton sheet. Stretch it horizontally → the lumps become long streaks → CIRRUS. Squash it vertically → the lumps become flat pancakes → STRATUS. Leave it alone (and add a domain-warp jiggle for organicness) → CUMULUS.

---

## Algorithm in Steps

1. **PERMUTATION TABLE** — shuffle `perm[256]` from a seed and duplicate to 512 (Perlin's standard trick to skip the modulo in lookup).

2. **EACH FRAME:**
   a. Advance wind: `wind_x += WIND_X · dt · speed_mul`, `wind_y += WIND_Y · dt · speed_mul`, `warp_t += DRIFT · dt · speed_mul`.
   b. For every screen cell `(sx, sy)`:
      i. Compute density per the active pattern:
         - **CUMULUS**: 3 fBm calls — 2 for warp, 1 for main.
         - **CIRRUS**: 1 fBm call, high freq, x-stretched.
         - **STRATUS**: 1 fBm call, low freq, y-stretched bands.
         - **STACKED**: 3 fBm calls, three altitude layers.
      ii. Convert density → level via thresholds.
      iii. If level > 0:
         ```
         glyph = PATTERN_GLYPHS[pattern][level - 1]
         ramp  = PATTERN_RAMP_LEVEL[pattern][level - 1]
         attr  = A_DIM / A_NORMAL / A_BOLD by level
         mvaddch (sy, sx, glyph)
         ```

3. **STACKED-only post-processing**: where the cumulus AND stratus fields BOTH exceed `STORM_THRESH`, occasional `/` lightning glyphs are placed on a sparse hash gate.

---

## Key Formulas

**fBm with N octaves:**
```
f(x, y) = (1/A) · Σ_{k=0..N-1} a_k · perlin(x·2^k, y·2^k)
A       = Σ a_k = 1 + 0.5 + 0.25 + … (so f ∈ roughly [-1, 1])
```
Re-mapped to `[0, 1]` with `f' = 0.5·f + 0.5`.

**Cumulus density** (domain warp pre-pass):
```
qx = fbm(x · WARP_S, y · WARP_S + warp_t)
qy = fbm(x · WARP_S + 5.2, y · WARP_S + 1.3 + warp_t)
d  = fbm((x + qx · WARP_AMT + wind_x) · CUMULUS_S,
         (y + qy · WARP_AMT + wind_y) · CUMULUS_S · ASPECT_Y)
```

**Cirrus density** (anisotropic, high-frequency):
```
d = fbm((x + wind_x · 1.4) · CIRRUS_SX,
        (y + wind_y      ) · CIRRUS_SY)        // scale_y > scale_x
```

**Stratus density** (anisotropic, low-frequency):
```
d = fbm((x + wind_x · 0.7) · STRATUS_SX,
        (y + wind_y      ) · STRATUS_SY)
```

**Density → level** (5 buckets per pattern):
```
d ≤ T_thin  → 0   (clear sky, do not draw)
≤ T_med    → 1   (wispy edge)
≤ T_dense  → 2   (medium body)
≤ T_peak   → 3   (dense)
>          → 4   (core / highlight)
```

**STACKED lightning gate:**
```
storm = (d_cumulus > T_storm) AND (d_stratus > T_storm)
bolt  = storm AND (hash3(x, y, ⌊30·t⌋) % 4000 == 0)
```

---

## Edge Cases and Pitfalls

- **ASPECT RATIO.** Terminal cells are 2× taller than wide. Multiply `sy` by `ASPECT_Y_F` when sampling fBm so cumulus blobs render as visual circles, not horizontal ovals. The CIRRUS pattern wants THE OPPOSITE — a tall scale_y so features are stretched horizontally — so it bypasses the aspect correction.

- **WIND CONTINUITY.** `wind_x` grows monotonically with time. After days it can exceed float precision (~10^7 cells of drift before single-precision Perlin starts to bin into the same integer cells). For a real long-running demo, modulo `wind_x` by perm size periodically; for an interactive showcase that rarely runs more than minutes, ignore.

- **DOMAIN WARP COST.** Each warped cumulus cell does 3 fBm calls (qx, qy, main). At 60 fps × 240×80 cells × 4 octaves that is ~14 M Perlin samples per second. Modern hardware handles it easily, but if you ever need to scale to 8 K terminals, drop the warp passes to 2 octaves.

- **FLOAT-VS-INT IN FLOOR.** Perlin's permutation lookup uses `(int)floorf(x) & 255`. For NEGATIVE `x`, `(int)` cast truncates toward zero — wrong direction; `floorf` rounds toward −∞ — right direction. Don't replace `floorf` with a plain `(int)` cast or the cloud field becomes discontinuous at `x = 0`.

- **ANISOTROPIC SCALES NEED ADJUSTING THRESHOLDS.** CIRRUS uses a much higher `scale_y` than `scale_x`. The fBm output range is the same, but the spatial distribution of high values changes — you typically need a slightly LOWER threshold for cirrus to get a similar sky-fill ratio.

- **LAYER DRAW ORDER IN STACKED.** Render BACK to FRONT (cirrus → cumulus → stratus). If you draw stratus first and let cirrus overdraw, the high cirrus appears IN FRONT of the low stratus — the wrong altitude order — and the depth illusion breaks.

- **LIGHTNING DENSITY.** The storm gate (cumulus AND stratus both dense) is rare; the hash modulo (`% 4000`) is rarer; their product gives ≈ 1 lightning glyph per ~50 frames. Tune the hash modulo if you want more / fewer flashes; the `30·t` in the seed means flashes regenerate at 30 Hz so they don't stick.

---

## How to Verify

- **PAUSE** (space). Clouds freeze in place — no drift, no animation. Resume: clouds slide from exactly where they froze.

- Press **`r`**. The cloud arrangement reshapes (different perm table) but the wind speed and pattern are unchanged.

- **CUMULUS** pattern. Clouds should look ROUND-ish, with organic swirly outlines (the domain-warp signature). If they look like perfect blobs with no swirl, the warp pre-pass is missing.

- **CIRRUS** pattern. Clouds should be obviously HORIZONTAL streaks, much wider than tall.

- **STRATUS** pattern. Clouds form horizontal BANDS that span much of the screen width.

- **STACKED** pattern. Three glyph types should be visible in the SAME view at different cells: `~` (cirrus high), `o`/`O` (cumulus mid), `#` (stratus low). Where stratus and cumulus are BOTH dense, occasional `/` bolts flash in red.

- **Wind direction.** Watch a cloud edge for a few seconds; it should slide leftward-to-rightward at speed proportional to the user's "speed" knob.

---

## References

- Perlin, K. (1985) — *An Image Synthesizer*, SIGGRAPH 1985 (https://mrl.cs.nyu.edu/~perlin/paper445.pdf).
- Mandelbrot, B. (1968) — Fractional Brownian motions, SIAM Review 10:422-437 (where fBm comes from).
- Inigo Quilez — "Domain warping" article (https://iquilezles.org/articles/warp/) — the cumulus pre-pass technique.
- Lague, Sebastian — "Coding Adventure: Procedural Worlds" (YouTube).
- Wikipedia — [Cloud / Cumulus / Cirrus / Stratus](https://en.wikipedia.org/wiki/Cloud).

---

*Source: `procedural/worldgen/cloud.c`*
