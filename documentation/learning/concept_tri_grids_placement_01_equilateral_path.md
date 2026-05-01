# Concept: Equilateral Line-of-Sight Path

## Pass 1 — Understanding

### Core Idea
Two markers — START (S) and END (E) — pin two triangles on the equilateral grid. The path between them is the ordered set of triangles a straight line crosses. Computed by walking pixel coordinates from S's centroid to E's centroid in fine steps and asking `pixel_to_tri` "which triangle is this?" — recording each new triangle.

### Pixel Walk vs Bresenham
A true Bresenham line on the triangular lattice would also work, but pixel-walk is simpler:
- The same `pixel_to_tri` already used by grid_draw classifies samples.
- Sampling at `step = size · 0.25` is fine enough that no triangle on the line is skipped (a triangle's smallest dimension is `size · √3/2 / 3 ≈ 0.289 · size`).

### Walk Algorithm
```
sx, sy = centroid pixel of START
ex, ey = centroid pixel of END
dx = ex − sx,   dy = ey − sy
dist = sqrt(dx² + dy²)
n = floor(dist / step) + 1
for i in 0..n:
    t  = i / n
    px = sx + t·dx,  py = sy + t·dy
    pixel_to_tri(px, py) → (col, row, up)
    path_add(col, row, up)        // dedup via path_contains
```

### When Recompute Runs
- on `'s'` (set START at cursor)
- on `'e'` (set END at cursor)
- on `'+/-'` (size change, since centroids depend on size)
- NOT on cursor move alone

### Pool Storage
`PathPool` — flat array of `TPath{col, row, up}`. `path_add` is O(n) linear scan via `path_contains` to dedupe; fine because paths stay short for any visible line.

### Non-Obvious Decisions
- **Centroid-to-centroid line** (not vertex-to-vertex): centroids guarantee the line passes through the interior of each path triangle, simplifying the inclusion test.
- **Zero-length guard**: `dist < 1e-6` returns early after adding the start triangle.
- **Step size tradeoff**: too coarse → skipped triangles; too fine → slow recompute. `size/4` is the empirical sweet spot.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Path pool cap |

### Open Questions
- What does the visible path look like when the line is exactly horizontal or 30° above horizontal?
- How would you implement "shortest path on triangle graph" (BFS) and how does it differ from line-of-sight?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ
§4 formula  — pixel_to_tri, tri_centroid_pixel, tri_edge_char (same as tri/01)
§5 pool     — PathPool: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw + has_start / has_end markers
§7 path     — path_compute pixel-walk → triangle list
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
's' → set START at cursor → path_compute
'e' → set END   at cursor → path_compute
'+/-' → size change → path_compute
frame → grid_draw + path_draw + S/E markers + cursor_draw
```
