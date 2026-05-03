# Concept — `procedural_city.c`: L-System / BSP City

## Core Idea

To make a city, do not build it forward (lay one road, then another, then place buildings); build it BACKWARD. Start with the answer — "this whole rectangle is one city block" — and rewrite that one block into smaller blocks separated by roads. Repeat until the blocks are too small to subdivide further; those terminal blocks are the lots. **The road network and the lot grid emerge for free from the subdivision tree.**

---

## The Mental Model

Think of a sheet of paper and a ruler. You draw ONE line that splits the page in half — that is your first street. Now you have two smaller pages. On each of them you draw another line, splitting them again. Now four. Each new line is a street; the regions they bound are "blocks". Keep recursing on each block until a block is smaller than a couple of buildings — at that point you stop, and call that block a lot.

Equivalently — and this is the L-system view — write a single letter `B` on the page, meaning "the whole city is one Block". Apply a rewriting rule: replace `B` with `R B B` (a Road and two smaller Blocks). Apply the rule again to each of the new B's. Eventually you stop rewriting (the B is too small) and replace it with `L` (a Lot). The string of letters R, R, R, ..., L, L, L, ... — read in order — describes the entire city plan.

The recursion tree IS the city. The depth of the tree IS the road hierarchy: depth-0 splits become arterial roads, depth-3 splits become side streets, depth-7 splits become alleys.

---

## Algorithm in Steps

1. **INIT.** Clear all cells to EMPTY. Pick a random seed.

2. **SUBDIVIDE(rect, depth):**
   a. If `depth ≥ MAX_DEPTH`, OR `rect` cannot fit two min-sized lots in either axis: stamp every cell of `rect` with type BUILDING (or PARK in some patterns) and return.
   b. Choose split axis:
      - if `rect.h × ASPECT_Y > 1.4 × rect.w` → split horizontally
      - else if `rect.w > 1.4 × rect.h × ASPECT_Y` → split vertically
      - else: `hash(rect)` low bit picks one
   c. Choose split position:
      - GRID pattern: exact midpoint
      - others: midpoint + jitter, where jitter ∝ `hash(rect)`, clipped so each half has room for ≥ MIN_LOT
   d. Stamp the split line as ROAD cells (one row or one column).
   e. Recurse on the two halves with depth+1.

3. **STEP TAGGING.** As each cell is stamped, give it a "creation step" that depends on `(depth, kind)`. Lower depth → smaller step. Roads stamp first; lots within the same depth stamp after.

4. **ANIMATE.** Each frame increment `build_step` toward `max_step`. Render only cells with `cell.step ≤ build_step`. The viewer sees the recursion unfold.

5. **HOLD then REBUILD.** After `build_step` reaches `max_step`, hold the finished city for `HOLD_TICKS`, then re-seed and goto 1.

After the city is BUILT, **traffic** activates: 28 directional-arrow cars (`> < ^ v`) move along the road network at ~15 cells/sec, occasionally turning at intersections. Building cells get a per-second hash-driven **window twinkle** (~2 % of windows light up at A_BOLD per second).

---

## Key Formulas

**Aspect-corrected aspect ratio of a block** (cells are 2× taller than wide):
```
aspect = (h · ASPECT_Y) / w
split_horizontally = (aspect > 1.4)         // tall block: cut the long way
```

**Split position** (ORGANIC family):
```
mid    = (lo + hi) / 2
range  = (hi − lo) / 3
jitter = (hash(rect) mod (2·range+1)) − range
split  = clamp(mid + jitter, lo + MIN_LOT, hi − MIN_LOT)
```

**Step tag for a cell stamped during depth d:**
```
step(road) = d · 1000 + ((x − x0) · 500) / span_x
step(lot)  = d · 1000 + 500 + ((x + y) · 500) / (lot_w + lot_h)
```

**Building type by zoning** (DISTRICTS / PARKS):
```
dist    = √(((cx − Cx)·1)² + ((cy − Cy)·ASPECT_Y)²) / D_max
type    = dist < 0.25 → 3 (skyscraper)
          dist < 0.50 → 2 (office)
          dist < 0.75 → 1 (apartment)
          else        → 0 (house)
```

**Window-light twinkle** (after build completes):
```
bold = (hash(x, y, ⌊t⌋) mod 50 == 0)
```
~2 % of building cells light up; pattern shuffles each second.

---

## Edge Cases and Pitfalls

- **MIN-LOT ENFORCEMENT.** If the split position is unconstrained, the recursion can produce 1×1 "lots" with no room for a building. Always clip split to `[lo + MIN_LOT, hi − MIN_LOT]`; if that interval is empty, terminate the recursion at the current rect.

- **ASPECT BIAS.** Without the `ASPECT_Y` multiplier, the algorithm sees a 60×30 cell rect as "wider than tall" and prefers vertical splits, but on a screen 60 cells wide × 30 cells tall is nearly SQUARE (because cells are 2× taller). Always multiply the row extent by `ASPECT_Y` when comparing for the split-axis decision.

- **RECURSION DEPTH BOUND.** Without a depth cap, a 240×80 city with a very small `MIN_LOT` could recurse ~20 levels deep, generating half a million lots and choking the per-frame render. `MAX_DEPTH` keeps the tree shallow enough that the lot count stays under ~2 000 — comfortable for ASCII rendering.

- **ROAD CELL IDENTITY.** After subdivision a cell is either ROAD, BUILDING, PARK, or EMPTY. Roads get a special drawing path that looks at four neighbours; buildings/parks just print their glyph. Don't try to decide in advance whether a road cell is a "+", "-" or "|" — you can't, because it depends on neighbours that haven't been placed yet at the time you're recursing. Decide at render time.

- **STEP MONOTONICITY.** The `cell.step` values must be a 16-bit quantity that grows with depth (so the build animation reveals things in roughly tree-order). With `depth_factor = 1000` and `MAX_DEPTH = 9`, max_step ≈ 10 000 — well under the 65 535 limit of `uint16_t`. If you raise MAX_DEPTH past 50, change `cell.step` to `uint32_t`.

- **RESIZE = REBUILD.** The city's grid size is fixed at scene init; on terminal resize we MUST throw away the existing layout and re-subdivide at the new dimensions. There is no incremental "reflow" of a BSP tree; just rebuild.

- **VISIBILITY-AWARE EDGE DETECTION (for framed buildings).** Building cells render as framed boxes (walls + interior). Edge detection consults BOTH cell type AND visibility (`step ≤ build_step`) — so a building's TOP edge correctly draws as `_` until the row above becomes visible during the build animation, then transitions to interior glyph.

---

## How to Verify

- **PAUSE during the build** (space). The wavefront freezes — cells with `step ≤ build_step` are visible, the rest are blank. Resume: build continues from exactly where it stopped.

- **RESET (r)**. The city collapses to empty and a NEW layout grows in. The road structure should be visibly different — same pattern (e.g. GRID), but the seed-driven choices differ.

- **GRID pattern**. Every road runs through the EXACT middle of its block. The result is a perfectly regular grid: rows of roads at powers-of-2 offsets, columns at powers-of-2 offsets. If splits are off-centre, GRID has accidentally inherited the jitter from another pattern.

- **ORGANIC pattern**. Block sizes visibly vary; some lots are 6×3, others 12×8. Roads do not line up across blocks (compare to GRID where they do). If everything still looks like a grid, the jitter is being clipped to zero.

- **DISTRICTS pattern**. Walk visually outward from the city centre: you should see a CLEAR colour progression from one tint to another over the four building palettes. Skyscraper boxes (with `@` interior, `=` foundation) cluster at the centre. Bumpy little `n` / `m` houses dominate the edges, no frame visible.

- **PARKS pattern**. Visible green `.`, `,` lots scattered through the city — none in the centre cluster, more toward the edges.

- **TWINKLE**. After the build completes (HUD says "BUILT"), watch a particular cell. About once every few seconds it should flicker brighter for one tick — a randomly-lit "window".

- **TRAFFIC**. After BUILT, ~28 directional arrows (`> < ^ v`) appear on the road network and step every few frames. They follow forward direction until the road ends, then pick a new direction; at intersections they occasionally turn. They never leave the road or appear during the build phase.

---

## References

- Lindenmayer, A. (1968) — "Mathematical models for cellular interactions in development", J. Theor. Biol. 18. The original L-system paper.
- Prusinkiewicz & Lindenmayer (1990) — *The Algorithmic Beauty of Plants* (https://algorithmicbotany.org/papers/abop/abop.pdf).
- Parish, Y. & Müller, P. (2001) — "Procedural Modeling of Cities", SIGGRAPH 2001 — the CityEngine paper, L-system roads at real-world scale.
- Wikipedia — [Binary space partitioning](https://en.wikipedia.org/wiki/Binary_space_partitioning).
- Red Blob Games — Dungeon generation with BSP trees.

---

*Source: `procedural/worldgen/procedural_city.c`*
