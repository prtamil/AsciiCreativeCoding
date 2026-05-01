# Concept: Kisrhombille (30-60-90) Direct Placement

## Pass 1 — Understanding

### Core Idea
Same cursor + ObjectPool mechanics as `01_equilateral_direct`, but the underlying grid renders three medians per equilateral, splitting it visually into 6 right 30-60-90 sub-triangles. The cursor still addresses WHOLE equilaterals — placement is by `(col, row, up)`, not by sub-triangle. The medians are decoration that `grid_draw` renders automatically.

### Two-Layer Render
1. Equilateral edges via barycentric weights (same as `tri_grids/01_equilateral`).
2. Median proximity test: for each median line in the cell, check signed-distance from `(fa, fb)`; if below `MEDIAN_T`, render `/`, `\`, or `|` for that median.

### Cursor (Same As 01_equilateral)
TRI_DIR table identical to `01_equilateral`. Centroid lands at the medians' concurrent point.

### Non-Obvious Decisions
- **Centroid lands on medians**: every parent's centroid is the medians' concurrent point. The glyph drawn there sits exactly on top of a median character; by draw order, the glyph wins.
- **MEDIAN_T tuning**: too thin → dashed-looking medians; too thick → median ink thickens into noise.
- **Resize**: cursor and objects keep their addresses; medians and glyphs scale together because both derive from the same cell math.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `BORDER_W` | Edge-proximity threshold |
| `MEDIAN_T` | Median-proximity threshold (signed distance) |
| `N_GLYPHS` | Number of cycle-able glyphs |

### Open Questions
- What if you wanted to address SUB-triangles (the 30-60-90s)? Centroid + barycentric sub-cell index would give 6 addresses per parent.
- How does this relate to the truncated trihexagonal tiling (the dual)?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, BORDER_W, MEDIAN_T, N_GLYPHS
§4 formula  — pixel ↔ skew lattice + edge + 3 median proximity
§5 pool     — TObj{col, row, up, glyph}, pool_find, pool_toggle
§6 cursor   — TRI_DIR (same as 01) + cursor_draw
§7 scene    — grid_draw + scene_draw
§8 screen   — ncurses init / cleanup
§9 app      — signals, main loop
```

### Data Flow
```
arrow → TRI_DIR[dir][cur->up] → cur += delta (whole triangles)
SPACE → pool_toggle(cur)
frame:
    grid_draw → for each cell:
        edge weight  → '/', '\', '_'  (PAIR_BORDER) if min < BORDER_W
        median dist  → '/', '\', '|'  (PAIR_MEDIAN) if dist < MEDIAN_T
    pool_draw + cursor_draw
```
