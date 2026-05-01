# Concept: Double-Diagonal (Tetrakis) Line-of-Sight Path

## Pass 1 — Understanding

### Core Idea
Pixel-walk path between two wedge centroids on the tetrakis grid. Each sample's `(Q, R, dir)` is recovered by `pixel_to_tri` + the wedge classifier. The path is the ordered list of 4-wedge cells the line crosses.

### Density Note
Wedges are smaller than the underlying square (4 per square), so the sampling step must stay tight (`size/4`). A coarser step risks skipping a wedge that the line crosses corner-on at the apex.

### Wedge Sampling
```
for i in 0..n:
    t  = i / n
    px = sx + t·dx,  py = sy + t·dy
    pixel_to_tri(px, py) → (col, row, dir, fa, fb)    // dir from |dx|>|dy| classifier
    path_add(col, row, dir)
```

### When Recompute Runs
Same triggers as 01_equilateral_path: 's', 'e', '+/-'. NOT on cursor move alone.

### Apex Tie
At the dead centre of any square (fa = fb = 0.5), all 4 wedges meet. The classifier defaults to N/S over E/W via the `≥` comparison; consistent across frames.

### Non-Obvious Decisions
- **Step size**: `size/4` is fine for any reasonable angle. Going coarser saves CPU but breaks correctness.
- **`PathPool` linear scan dedup**: `path_contains` is O(n); paths stay short for any visible line so this is fine.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Path pool cap |

### Open Questions
- For a line passing through the apex of one square, which wedge "wins" the apex pixel?
- Would BFS on the wedge graph give a shorter or different path?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ
§4 formula  — wedge classifier + barycentric per wedge + wedge centroids
§5 pool     — PathPool{col, row, dir}: clear, contains, add, draw
§6 cursor   — TETRA_DIR + step + draw + S/E markers
§7 path     — path_compute pixel-walk
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
's' / 'e' / '+/-' → path_compute → fill PathPool
frame → grid_draw + path_draw + S/E markers + cursor_draw
```
