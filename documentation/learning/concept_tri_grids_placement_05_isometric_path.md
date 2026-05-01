# Concept: Isometric (Solid-Fill) Line-of-Sight Path

## Pass 1 — Understanding

### Core Idea
Same line-walk algorithm as `01_equilateral_path` on the iso solid-fill grid. The only difference is paint: background is solid-filled by palette_index; the path overlays bright '*' glyphs on top of those fills. Path computation is colour-unaware.

### Walk
Identical to 01_equilateral_path: pixel-walk from S centroid to E centroid at `step = size · 0.25`; classify each sample with `pixel_to_tri`; dedup via `path_contains`.

### Visual Twist
Each path '*' lands on top of one of 6 fill colours. The path colour pair is chosen for high luminance contrast over any palette slot. Theme cycle re-tunes background colours but path '*' positions are unchanged.

### Recompute Triggers
'`s'`, `'e'`, `'+/-'`. NOT cursor move alone.

### Non-Obvious Decisions
- **Path is unaware of palette**: the path triangles don't know what colour they're sitting on. Theme cycle just re-tunes background — path persists.
- **Path overlay vs underlying fill**: each path star covers one fill cell entirely (because `mvaddch` overwrites the cell's foreground glyph but keeps the bg).
- **Sampling step**: same `size/4` as 01_equilateral_path.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Path pool cap |
| `N_PALETTE` | 6 — iso fill cycle |

### Open Questions
- Can the path '*' colour itself depend on the cube face it's sitting on (e.g. always pick the complementary colour)?
- How would you make the path animate — chase the line from S to E?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, N_PALETTE
§4 formula  — pixel_to_tri + palette_index + centroid
§5 pool     — PathPool{col, row, up}: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw + S/E markers
§7 path     — path_compute pixel-walk
§8 scene    — solid-fill raster + path + markers + cursor
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
's' / 'e' / '+/-' → path_compute → fill PathPool
frame:
    raster fill (per cell)
    path_draw '*' overlay
    S / E / cursor on top
```
