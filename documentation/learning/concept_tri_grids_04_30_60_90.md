# Concept: Kisrhombille Tiling (30-60-90 Right Triangles)

## Pass 1 — Understanding

### Core Idea
The equilateral tiling from `01_equilateral.c` overlaid with each triangle's three medians. A median connects a vertex to the midpoint of the opposite edge; together they cut every equilateral into 6 congruent 30-60-90 right triangles. Twelve right triangles meet at every original vertex.

### Where the Name Comes From
"Kis" + "rhombille". The rhombille tiling uses 60°-120° rhombi; "kis" is the Conway operator that adds a central point and triangulates. Applied to the rhombille (or equivalently to the triangular tiling), every triangle becomes 6 right triangles.

### Two Layers
1. **Equilateral edges** — picked by the same barycentric test as `01_equilateral`.
2. **Median proximity** — each median is a line segment inside one equilateral; render `/`, `\`, `|` near a median when the perpendicular distance from `(fa, fb)` to the line is below `MEDIAN_T`.

### Median Line Equations (in lattice space, per ▽ triangle)
```
median 1: fa = fb              // from corner P00 to mid of edge opposite
median 2: fa + 2·fb = 1
median 3: 2·fa + fb = 1
```
Signed distance from `(fa, fb)` to a line `aL·x + bL·y + c = 0` is
`|aL·fa + bL·fb + c| / √(aL² + bL²)`.

### Cursor (Same As 01)
Cursor still addresses WHOLE equilaterals (col, row, up); medians are visual decoration only. TRI_DIR table is identical to `01_equilateral`.

### Non-Obvious Decisions
- **Three medians concurrent at centroid**: the cursor's `'@'` lands precisely on this concurrent point. By draw order, the glyph paints over the median ink.
- **MEDIAN_T tuning**: too thin → medians look dashed; too thick → median characters thicken into visual noise. Default 0.06 works at size = 14.
- **30-60-90 angles**: each sub-triangle has these exact angles regardless of equilateral size. Verifiable from the median construction (a median to a 60° angle vertex gives a 30° split of the 60°).

### Key Constants
| Name | Role |
|------|------|
| `TRI_SIZE` | Equilateral side length in pixels |
| `BORDER_W` | Edge-proximity threshold (same as 01) |
| `MEDIAN_T` | Median-proximity threshold (signed distance in lattice units) |

### Open Questions
- What is the dual of the kisrhombille? (Answer: the truncated trihexagonal 3.4.6.4.)
- Why are exactly 12 right triangles meeting at every vertex of the kisrhombille?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — TRI_SIZE, BORDER_W, MEDIAN_T
§4 formula  — pixel ↔ skew lattice + edge + 3 median proximity
§5 cursor   — TRI_DIR (same as 01) + cursor_draw
§6 scene    — grid_draw + scene_draw
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
(row, col) → pixel → pixel_to_tri → (col, row, up, fa, fb)
                                       ↓
                       edge weight  → '/', '\', '_'  (PAIR_BORDER)
                       median dist  → '/', '\', '|'  (PAIR_MEDIAN)  if within MEDIAN_T
arrow key → TRI_DIR[dir][cur->up] → cur += delta (whole triangles)
```
