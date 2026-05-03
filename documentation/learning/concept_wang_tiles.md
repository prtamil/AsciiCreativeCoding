# Concept — `wang_tiles.c`: 16-Tile Wang Tilings

## Core Idea

Take a small library of square tiles. Each tile has FOUR edges, each painted in one of a few colours. Rule: when you place tile A next to tile B, the edge they SHARE must match in colour. Now fill a grid by repeatedly picking any tile that satisfies the rule at this position. The rule is local — only the immediate north and west neighbours influence the choice — but the consequence is **GLOBAL**: same-colour edges line up across many tile boundaries to form continuous bands of colour flowing through the picture. **The pattern looks designed; nobody designed it.**

(Hao Wang, 1961.)

---

## The Mental Model

Imagine a child's wooden block set. Each block is a cube with coloured stripes painted on its four side faces. You stack the blocks on a table such that any TWO TOUCHING SIDE FACES wear the same paint colour. Doing this strictly forces stripes that span many blocks — a red stripe carrying through five blocks, separated from a blue stripe by a clean dividing line. Step back: what looks like one elaborate continuous painting is actually 25 independently-chosen blocks held together by a single matching rule.

The PATTERN (`n` / `N`) is what makes the random tile choice prefer red OR blue at each spot — but the matching rule is the same. Pattern bias produces large red regions vs random patches vs banded stripes; the underlying machinery is identical.

---

## Algorithm in Steps

1. **INIT.** Build the 16-tile complete set. `tile_set[i]` has edges `(N, E, S, W) = (i bits 3..0)`. With 2 colours per axis there are `2⁴ = 16` distinct tiles — every combination. This is a "complete" set: for any pair of (W-constraint, N-constraint) at least one tile is valid, so placement never gets stuck.

2. **GENERATE the grid** (whenever seed or pattern changes):
   ```
   for ty in 0..H:
     for tx in 0..W:
       if (tx > 0): expected_w = grid[ty][tx-1].E
       if (ty > 0): expected_n = grid[ty-1][tx].S
       valid = { tile : tile.W matches and tile.N matches }
       scored = score each valid tile by pattern's preference
       chosen = highest-scored, with hash-based tiebreak
       grid[ty][tx] = chosen
   ```
   Pattern preferences:
   - **RANDOM**: 0 for all tiles (uniform pick)
   - **NOISE**: prefer if E,S match fBm-biased colour
   - **STRIPES**: prefer if S matches sin(y)-biased colour
   - **SWIRL**: prefer if E,S match `atan2(y - cy, x - cx)`-biased colour

3. **RENDER each frame:**
   ```
   for sy in screen_rows:
     for sx in screen_cols:
       tx = (sx − gx0) / TILE_W
       ty = (sy − gy0) / TILE_H
       dx = sx mod TILE_W, dy = sy mod TILE_H
       tile = grid[ty][tx]
       (glyph, pair) = glyph_set_render(tile, dx, dy)
       brightness    = fbm2(sx · BSC_X + wind, sy · BSC_Y · ASPECT)
       attr          = level_to_attr(brightness)
       mvaddch(sy, sx, glyph) in pair + attr
   ```

---

## Key Formulas

**Tile index encoding:**
```
tile_set[i].n = (i >> 3) & 1
tile_set[i].e = (i >> 2) & 1
tile_set[i].s = (i >> 1) & 1
tile_set[i].w = (i >> 0) & 1
```

**Wang constraint at cell `(tx, ty)`:**
```
valid = { tile : (tx == 0 OR tile.W = grid[ty][tx-1].E) AND
                 (ty == 0 OR tile.N = grid[ty-1][tx].S) }
```

**Pattern preference for edges to bias:**
```
RANDOM   : no preference
NOISE    : prefer_e = prefer_s = (fbm(tx·s, ty·s) > 0.5)
STRIPES  : prefer_s = (sin(ty·k) > 0)
SWIRL    : θ        = atan2((ty − cy)·2, tx − cx)
           prefer_e = prefer_s = (θ > 0)
```

**Score and pick:**
```
score(tile)     = (tile.E matches prefer_e ? 1 : 0)
                + (tile.S matches prefer_s ? 1 : 0)
chosen = uniform random over { valid : score = max_score }
```

**Brightness modulation:**
```
b = fbm2(sx · BSC_X + wind_x, sy · BSC_Y · ASPECT_Y)
attr = b > 0.65 → A_BOLD ; b < 0.35 → A_DIM ; else A_NORMAL
```

---

## Edge Cases and Pitfalls

- **COMPLETE SET.** With 2 colours per axis and 16 tiles, EVERY (W, N) constraint pair has exactly 4 valid tiles (free choice over E and S). Placement never gets stuck. If you cut the set down to fewer than 16 tiles — to make the visual more constrained — you must verify that for ALL 4 possible (W, N) pairs, at least one tile remains valid.

- **EDGE OF GRID.** The leftmost column has no W neighbour; the top row has no N neighbour. Treat the missing constraint as "any" (tile can have either edge value). The corner tile `(0, 0)` is unconstrained and picks freely from all 16.

- **PREFERENCE ≠ HARD CONSTRAINT.** The pattern's preference biases the choice but never overrides the Wang constraint. NOISE might want a tile with E = 1, but if no valid tile has E = 1 given the W/N constraint, the highest-scoring valid tile (E = 0) is taken instead. This keeps placement always succeeding.

- **GRID SIZE VS SCREEN SIZE.** The grid is rounded down to whole tiles: `grid_w = screen_w / TILE_W`, `grid_h = (screen_h − HUD) / TILE_H`. Cells at the screen edge that fall outside the last tile boundary are not drawn (left blank).

- **TILE_W / TILE_H ASPECT.** With cells 2× taller than wide, a 6×3 tile is 6 cells horizontally and 6 "cell-widths" vertically (3 cells × 2) — visually square. Smaller tiles look horizontally squashed.

- **BRIGHTNESS DRIFT TICK-COUPLED.** `wind_x` advances per second of sim time; on slow ticks the visual drift slows. For consistent visual speed across tick rates, scale by `dt` (already done) and rely on the fixed-step accumulator.

---

## How to Verify

- **EDGES** pattern. Trace any vertical line where two tiles meet. The right border (`|`) of one tile and the left border (`|`) of the next tile are adjacent CELLS on screen — and by Wang constraint they're the same colour. You should never see a sharp colour change at a tile boundary.

- **RANDOM** pattern. Tile diversity is maximum; no large regions of same colour. The whole grid is uniformly speckled.

- **NOISE** pattern. Same constraint, different bias: now you should see large continuous regions of "mostly blue" and "mostly yellow" (or whatever the theme calls them) separated by irregular boundaries. Like fBm clouds.

- **STRIPES** pattern. The S edge bias creates horizontal BANDS: rows alternate between mostly-colour-0 and mostly-colour-1.

- **SWIRL** pattern. Bias angles around the centre into 4 quadrants; you should see two halves of the grid favour different colours, splitting roughly along a horizontal-ish line through the middle.

- Switch GLYPH (`g`) within a fixed pattern. The TILE LAYOUT does not change; only the rendering (glyphs and interior fills) does. Tile boundaries are at the same positions; constraint matching is still visible (just with different glyphs).

- **Pause** (space). Brightness wave freezes; tiles stay where they are. Resume: drift continues from where it stopped.

---

## References

- Wang, H. (1961) — "Proving theorems by pattern recognition II", Bell System Technical Journal 40:1-41. The original Wang tile paper.
- Berger, R. (1966) — "The Undecidability of the Domino Problem". Disproved Wang's "every tileable set is periodically tileable" conjecture by exhibiting an aperiodic 20 426-tile set.
- Cohen, M. F. et al. (2003) — "Wang Tiles for Image and Texture Generation", SIGGRAPH 2003. The modern computer-graphics revival.
- Wikipedia — [Wang tile](https://en.wikipedia.org/wiki/Wang_tile).
- Inigo Quilez — "Wang tiles" article.

---

*Source: `procedural/patterns/wang_tiles.c`*
