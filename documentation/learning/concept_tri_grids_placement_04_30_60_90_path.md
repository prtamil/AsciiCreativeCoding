# Concept: Kisrhombille Line-of-Sight Path

## Pass 1 — Understanding

### Core Idea
Same line-walk algorithm as `01_equilateral_path`, on the kisrhombille grid. Path is computed in WHOLE-equilateral lattice space; the medians (the 30-60-90 substructure) are decoration only and do not alter `pixel_to_tri`'s output.

### Walk
Identical to 01_equilateral_path: pixel-walk from S centroid to E centroid at `step = size · 0.25`; classify each sample with `pixel_to_tri`; dedup via `path_contains`.

### Recompute Triggers
'`s'`, `'e'`, `'+/-'`. NOT cursor move alone.

### Visual: Medians Under Path
The path '*' at a centroid lands precisely on the median concurrent point. By draw order, the '*' covers the median ink at that single cell.

### Non-Obvious Decisions
- **Path doesn't see medians**: the medians are visual paint only. Two endpoints on adjacent equilaterals give a 2-entry path even though the line crosses several 30-60-90 wedges.
- **Centroid line stays inside the parent triangle**: same property as 01_equilateral_path.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Path pool cap |

### Open Questions
- What if you wanted the path to follow 30-60-90 wedge edges instead of equilateral edges?
- How would you visualise both the parent path AND the wedge-level path simultaneously?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ
§4 formula  — pixel ↔ skew lattice + centroid + median proximity
§5 pool     — PathPool{col, row, up}: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw + S/E markers
§7 path     — path_compute pixel-walk
§8 scene    — grid_draw (edges + medians) + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
's' / 'e' / '+/-' → path_compute → fill PathPool
frame → grid_draw + path_draw + S/E markers + cursor_draw
```
