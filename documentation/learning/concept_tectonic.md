# Concept — `tectonic.c`: Plate-Tectonic Worldmap

## Core Idea

To make a world look geological — not random — you don't draw the mountains. You draw the FORCES that would make the mountains. Tile the plane with plates, give each plate a velocity, look at where the plates push or pull on each other, and let the elevation fall out of those interactions. Add some noise for grit. The mountain chains, ocean ridges, and coastlines emerge for free because you drew the same physics nature uses.

---

## The Mental Model

Picture a jigsaw puzzle floating on water. Each piece is a tectonic plate. The pieces drift in different directions at different speeds. Where two pieces are pushing INTO each other, the edges crumple upward → mountains. Where two pieces are pulling APART, a gap opens between them and water rushes in → rift / ocean ridge. Where pieces SLIDE PAST each other, neither gap nor pile-up → a fault zone. The interior of each piece is just whatever colour the piece was — flat plains for continental plates, deep water for oceanic plates. **All the geography is in the EDGE INTERACTIONS.**

For "jigsaw piece" read "Voronoi cell of a random seed", for "drifting" read "random velocity vector", for "edges crumple" read "+0.5 added to elevation within distance R of a convergent boundary", and you have the algorithm. The whole world is just Voronoi + per-plate constants + a 5-line classifier on the relative velocity at every shared edge.

---

## Algorithm in Steps

1. **PLACE N plate seeds** in a jittered grid. For each: position `(x, y)`, velocity `(vx, vy)` (random direction, speed in `[0.3, 1.0]`), type ∈ {oceanic, continental} 50/50, base_elev = oceanic ? −0.5 : +0.3.

2. **ASSIGN each cell to its nearest plate seed** (Voronoi). Use squared Euclidean with `y · 2` for cell aspect.

3. **DETECT boundaries**: for each cell, if any of its 4 neighbours belongs to a different plate, this is an edge cell.

4. **CLASSIFY each edge** by the velocity of the two plates:
   ```
   n_ab    = (b.pos − a.pos) / |b.pos − a.pos|       boundary normal
   approach = (a.v − b.v) · n_ab
   if approach > +T     → CONVERGENT
   if approach < −T     → DIVERGENT
   else                 → TRANSFORM
   ```

5. **ELEVATION at each cell:**
   For every cell within R of an edge, find its closest edge and apply the type-specific modifier (decayed by `1−d/R`):
   ```
   CONVERGENT  : +0.5 · (1 − d/R)
   DIVERGENT   : −0.4 · (1 − d/R)
   TRANSFORM   : 0
   ```
   `elev = plate.base_elev + modifier + 0.4 · (fbm − 0.5)`. Clamp to `[−1, +1]`.

6. **BIOME = bucket of elev.** 8 buckets `DEEP_OCEAN..PEAKS`.

7. **RENDER per the active pattern.** Repeat from step 1 every ~`REGEN_SECONDS` to bulldoze the world and rebuild from a new seed.

---

## Key Formulas

**Voronoi** (per cell — choose closest plate):
```
d²(plate_i)  = (x − pi.x)² + 4·(y − pi.y)²
cell.plate   = argmin_i d²(plate_i)
```

**Boundary classification:**
```
n  = ((b.x − a.x), 2·(b.y − a.y)) / |...|       // aspect normal
approach = (a.vx − b.vx)·n.x + (a.vy − b.vy)·n.y
perp     = √(|Δv|² − approach²)
kind     = approach >  T   → CONVERGENT
         | approach < −T   → DIVERGENT
         | otherwise        → TRANSFORM
```

**Elevation:**
```
d, t  = (distance, type) of nearest boundary cell within R
mod   = match t with
          CONVERGENT → +0.5 · (1 − d/R)
          DIVERGENT  → −0.4 · (1 − d/R)
          TRANSFORM  →  0
elev  = clamp(plate.base + mod + 0.4·(fbm(x·s, y·s)−0.5), −1, 1)
```

**Biome buckets:**
```
e < −0.55 DEEP_OCEAN | < −0.20 OCEAN  | < 0    COAST
| < +0.15 PLAINS     | < +0.35 HILLS  | < +0.55 MOUNTAINS
| < +0.75 HIGHLANDS  | else PEAKS
```

---

## Edge Cases and Pitfalls

- **DEGENERATE NORMALS.** If two plate seeds end up at identical `(x, y)` the boundary normal is undefined. Jittered-grid placement makes this practically impossible, but `classify_boundary` still guards against zero-length normals and falls back to TRANSFORM.

- **ASPECT EVERYWHERE.** Voronoi distance, boundary-normal direction, AND the elevation-modifier scan radius all need the `y·2` aspect correction — otherwise plates look flat (squashed) and mountain chains form too easily in the y direction. Use it consistently.

- **TRANSFORM MUST DOMINATE WHEN PERPENDICULAR.** With the simple test "approach > T or approach < −T", every boundary becomes either convergent or divergent if T is tiny. We compare `|approach|` to the perpendicular magnitude — a boundary is only transform when the relative motion is mostly TANGENT to the boundary, not NORMAL to it.

- **BOUNDARY-EFFECT RADIUS.** Too small → mountains form only on the exact boundary cells, looking like a line drawing. Too large → mountains everywhere; the plate interior disappears. R ≈ 6 cells (with aspect) leaves a clear "coast plus inland" gradient.

- **ELEVATION CLAMPING.** Without clamp, divergent boundaries on top of an already-oceanic plate (base −0.5) plus the −0.2 noise floor can produce `elev = −1.1`, which falls outside the biome table. Always clamp post-modifier.

- **PLATE COUNT VS MAP SIZE.** Too many plates on a small map and the Voronoi cells become smaller than the boundary radius — every cell is "near a boundary", elevation washes out. Cap plates at 14 and require ≥ 6 cells of plate radius on the smallest map.

- **REGEN PERFORMANCE.** The plate-classification loop's inner Voronoi is `O(W·H · N)`. For 240·80·14 ≈ 270 K ops; fine. If you raise N past ~30 a spatial index becomes worth it.

---

## How to Verify

- **PAUSE during HOLD**: the world freezes. Resume: animation continues exactly where it stopped (water shimmer, peak twinkle).

- Press **`r`**: flash + regenerate. The new world has a different plate count and layout but the SAME biome distribution statistics (e.g. roughly half ocean, half land for the default 50/50 oceanic-continental split).

- **PLATES** pattern: every cell of the same plate has the same tint AND glyph; cells on the boundary are highlighted. Plate seeds are visible as bright `O` markers — you should be able to find one inside every Voronoi cell.

- **STRESS** pattern: the boundary cells form a network of lines crossing the map. Convergent edges (red) should dominate where plates head into each other; divergent (blue) where they pull apart. Transform (yellow) is rarer but appears between plates moving roughly parallel.

- **WORLD** pattern: walk the eye along an EXTENDED CONVERGENT boundary in STRESS — the same line in WORLD should host a chain of MOUNTAINS / PEAKS. Walk a DIVERGENT line — same location should be DEEP_OCEAN / OCEAN. Visual proof the elevation modifier is wired up correctly.

- **ELEVATION** pattern: a smooth gradient with no plate-tint structure — just heights. Compare against WORLD: the same high-elevation cells should map to MOUNTAINS / PEAKS biomes.

---

## References

- Wikipedia — [Plate tectonics](https://en.wikipedia.org/wiki/Plate_tectonics).
- Wikipedia — [Plate boundaries](https://en.wikipedia.org/wiki/Plate_boundaries).
- Wikipedia — [Voronoi diagram](https://en.wikipedia.org/wiki/Voronoi_diagram).
- Lague, Sebastian — "Tectonic Plate Simulation for Procedural Terrain" (YouTube).
- Andy Gainey — "Procedural Worlds from Simple Tiles" (https://experilous.com/1/blog/post/procedural-planet-generation).

---

*Source: `procedural/worldgen/tectonic.c`*
