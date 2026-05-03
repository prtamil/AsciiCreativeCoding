# Concept — `truchet_tiles.c`: Truchet Tiles + Pattern×Glyph Decoupling

## Core Idea

A complex pattern need not come from a complex rule. Take ONE tiny design — half a square shaded one way, half the other. Cover a grid with copies of it, **rotating each one at random**. The resulting surface is anything but trivial: paths, swirls, mazes, knots all appear out of pure local randomness. The cells don't communicate; they each just flip a coin. The pattern is in the EYE — your visual cortex stitches the per-cell diagonals into globally meaningful curves. That's the Truchet effect (Sébastien Truchet, 1704).

---

## The Mental Model

Imagine a stack of identical playing cards, each one bearing a single diagonal line from corner to corner. Spread them on a checkerboard, randomly choosing which corner each card's line starts from. Step back. Where two adjacent cards have lines that share an edge endpoint, those lines APPEAR continuous to the eye. Long curves emerge — sometimes closing into circles, sometimes meandering across the whole grid. Variations:
- Cards with TWO diagonals (an X) on some flipped orientation → cross-junctions in the pattern.
- Multi-cell cards (one card spans 2 squares) → patterns at multiple scales.

Add LIGHT: shine a slowly-moving spotlight across the pattern. The pattern itself doesn't change; only the BRIGHTNESS shifts. That is the fBm field — a smooth, drifting "luminance" texture we paint OVER the Truchet to bring it to life.

---

## The Decoupled Architecture (Pattern × Glyph)

This file pioneered a design pattern reused across the rest of `procedural/patterns/*.c`:

- **Pattern axis** (`n` / `p`) — the underlying SCALAR FIELD that determines per-cell orientation. Examples:
  - **RANDOM**: `hash3(tile_x, tile_y, seed) / 2³²`
  - **NOISE**:  `fbm2(tile_x · scale, tile_y · scale)` — smooth flowing regions
  - **BANDS**:  `0.5 + 0.5·sin(tx · fx + ty · fy)` — diagonal stripes
  - **VORONOI**: nearest jittered seed in 3×3 neighbourhood — irregular blocky regions

- **Glyph axis** (`g` / `G`) — the CHARACTER MAPPING. Each glyph set carries `(n_orient, tile_w, glyphs[8])` metadata. The pattern's `[0,1]` value is quantised into `n_orient` buckets; the chosen orientation × `tile_w` indexes into `glyphs[]`.

The pattern decides WHERE rotations live; the glyph set decides WHAT each rotation looks like. **Twelve glyph sets × four patterns = 48 distinct combinations** from a single ~600-line file.

Glyph sets in the file:
- 2-orient × 1-cell: `diag` (`/\`), `lens` (`()`), `brkt` (`[]`), `wave` (`~-`)
- 4-orient × 1-cell: `axis` (`/\_|`), `cross` (`/\+X`), `arrow` (`<>^v`), `dots` (`oO#@`)
- 2-orient × 2-cell: `slope` (`,'` `',`), `tri` (`<>` `><`), `wcurv` (`()` `)(`), `wbrkt` (`[]` `][`)

---

## Algorithm in Steps

1. **SEED.** Pick a 32-bit hash seed. Reseed reshuffles all tiles.

2. **EACH FRAME:**
   a. Advance the wind: `wind_x += WIND·dt·speed`.
   b. For every screen cell `(sx, sy)`:
      - Get glyph from TRUCHET — a pure function of `(sx, sy, seed, pattern, set)`:
        ```
        v = pattern_value(pattern, tile_x, tile_y, seed)   // [0, 1]
        orient = floor(v · n_orient)
        glyph = GLYPH_TABLE[set][orient][sub_position]
        ```
        For 1-cell tiles, `sub_position` is always 0. For 2-cell-wide WAVE, `sub_position = sx mod 2`.
      - Get brightness from fBm:
        ```
        b = fbm2(sx · BSC_X + wind_x, sy · BSC_Y · ASPECT_Y)
        ```
      - Map b → level (0..7), pick `PAIR_RAMP_BASE + level`, pick attr (A_DIM / A_NORMAL / A_BOLD).
      - mvaddch(sy, sx, glyph).

3. HUD on top.

---

## Key Formulas

**Tile-cell coordinate:**
```
tile_x = sx / TILE_W      // for 2-cell-wide WAVE, TILE_W=2
tile_y = sy / TILE_H      // 1 for all current patterns
sub    = sx mod TILE_W
```

**Pattern → orientation value** in `[0, 1]`:
```
RANDOM  : hash3(tile_x, tile_y, seed) / 2³²
NOISE   : fbm2(tile_x · NOISE_SCALE_X + ox, tile_y · NOISE_SCALE_Y + oy)
BANDS   : 0.5 + 0.5·sin(tile_x · BANDS_FREQ_X + tile_y · BANDS_FREQ_Y + φ)
VORONOI : nearest_seed(tile_x, tile_y, seed)  →  hash3(seed_cx, seed_cy, seed) / 2³²
```

**Brightness field:**
```
b = fbm2(sx · BSC_X + wind_x, sy · BSC_Y · ASPECT_Y)
```

**Brightness → level → attr:**
```
level = clamp(⌊b · 8⌋, 0, 7)
attr  = level >= 6 ? A_BOLD : level <= 1 ? A_DIM : A_NORMAL
```

**Wind:**
```
wind_x' = wind_x + WIND_X_BASE · dt · (speed / SPEED_DEF)
```

---

## Edge Cases and Pitfalls

- **TILE_W != 1 INDEXING.** WAVE has `TILE_W = 2`; the LEFT cell of a tile uses `sub = 0`, the RIGHT cell uses `sub = 1`. With negative `sx`, `sx % 2` may give -1 in C; bias by adding `TILE_W` and taking modulo again.

- **HASH STABILITY.** The orientation hash uses `(tile_x, tile_y, seed)`. `seed` is fixed until 'r' reseeds. `tile_x, tile_y` are integer screen coordinates — they DO NOT include `wind_x`. The Truchet pattern is therefore STATIC; only the brightness field drifts. Adding `wind_x` to the hash inputs would cause the pattern itself to scroll, which IS a valid look but loses the "static art with moving light" effect.

- **FBM BRIGHTNESS NEEDS NORMALISATION.** `fbm2` returns roughly `[0, 1]`; we use it directly. If the noise occasionally exceeds 1 due to the Perlin gradient bound, clamp before mapping to level — otherwise level can wrap past 7.

- **ASPECT IN BRIGHTNESS.** Multiply `sy` by `ASPECT_Y` when sampling fBm so the "spotlight" appears as a circular blob, not a horizontal oval. The Truchet glyph itself does NOT need aspect correction — it tiles on the discrete cell grid.

- **DON'T USE A_DIM FOR LEVEL 0.** With dark theme entries plus A_DIM, level-0 cells become invisible — the user sees the Truchet pattern with random GAPS where the spotlight is weakest. Keep level 0 at the brightest legible tier (A_DIM over a still-bright ramp slot is OK; A_DIM over a near-black ramp slot is not). The bright-half theme palette enforces this.

- **PER-PATTERN PERM RESHUFFLE.** Every pattern change reshuffles `perm[]` from `seed XOR (pattern * 0xA5A5A5)`. Without it, the brightness field is identical across patterns and the user sees just a different glyph distribution under the same drift.

---

## How to Verify

- **PAUSE** (space). The fBm field freezes; the bright "spotlight" stops moving; the Truchet pattern was already static. Resume: spotlight slides from exactly where it stopped.

- Press **`r`**. Sky-flash, then the tile arrangement REORGANISES — same algorithm, new orientations.

- **RANDOM** pattern. Trace a continuous diagonal line across the screen — it should connect across multiple cells, occasionally bending where neighbouring cells happen to flip orientation. Long ribbons and closed loops are common.

- **NOISE** pattern. Same constraint, different bias: now you should see large continuous regions of "mostly /" and "mostly \" separated by irregular boundaries.

- **BANDS** pattern. Diagonal stripes alternate between orientations.

- **VORONOI** pattern. Irregular blocky regions of same orientation, with boundaries following the Voronoi diagram of jittered seeds.

- **Theme cycle (t / T).** Spotlight intensity should be visible against EVERY theme — even MATRIX (greens) and OCEAN (blues) should show clear contrast between dim and bright spotlight zones.

---

## References

- Truchet, S. (1704) — "Mémoire sur les Combinaisons", Mémoires de l'Académie Royale des Sciences. The original paper.
- Smith, C. (1987) — "The tiling patterns of Sébastien Truchet and the topology of structural hierarchy", Leonardo 20(4). The modern revival; introduces the quarter-circle variant.
- Browne, C. (2008) — "Truchet curves and surfaces", Computers & Graphics 32(2):268-281.
- Wikipedia — [Truchet tiles](https://en.wikipedia.org/wiki/Truchet_tiles).
- Inigo Quilez — Truchet shader article.

---

*Source: `procedural/patterns/truchet_tiles.c`*
