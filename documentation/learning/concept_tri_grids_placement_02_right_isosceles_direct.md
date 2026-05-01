# Concept: Right-Isosceles Direct Placement

## Pass 1 — Understanding

### Core Idea
Direct cursor-driven placement on the half-rect grid from `tri_grids/02_right_isosceles.c`. Each unit square holds an UR (up=1) and an LL (up=0) triangle. The cursor address is (col, row, up); SPACE toggles a glyph at that address. Pixel→lattice is axis-aligned — no shear.

### Two Address Spaces (Same Pattern As 01)
1. The GRID — derived per-pixel via `pixel_to_tri = (px/size, py/size)` plus `up = (fa ≥ fb) ? UR : LL`.
2. The OBJECT POOL — flat array of `TObj{col, row, up, glyph}`.

### Pool Toggle (O(1) Remove)
Same swap-with-last as 01_equilateral_direct. `pool_find` is O(n) linear scan.

### Cursor (TRI_DIR for Half-Rect)
LEFT from UR jumps to col-1; LEFT from LL toggles inside the same square. UP from UR jumps to row-1; UP from LL toggles. Two LEFTs always traverse one whole square: `UR → LL(same) → UR(col-1)`.

### Centroid Formulas
```
UR:  a = col + 2/3,  b = row + 1/3   →   px = a · size,  py = b · size
LL:  a = col + 1/3,  b = row + 2/3   →   px = a · size,  py = b · size
```

### Non-Obvious Decisions
- **No skew** vs equilateral_direct: pixel→lattice is just a divide. Forward map and centroids do not need the v2 shear undo.
- **Diagonal tie** at `fa = fb`: `up = (fa ≥ fb) ? UR : LL` defaults to UR. Measure-zero set.
- **Aspect**: CELL_W=2, CELL_H=4 — triangles are right-isosceles in pixel space; on screen they look stretched vertically. Faithful, not buggy.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `BORDER_W` | Edge-proximity threshold |
| `N_GLYPHS` | Number of cycle-able glyphs |

### Open Questions
- What changes if you flip the diagonal from `\` to `/`?
- How does a glyph's screen position differ between UR and LL of the same square?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, BORDER_W, N_GLYPHS
§4 formula  — pixel_to_tri, tri_centroid_pixel, tri_edge_char (axis-aligned)
§5 pool     — TObj struct, pool_find, pool_toggle, pool_draw
§6 cursor   — TRI_DIR + step + draw
§7 scene    — grid_draw + scene_draw
§8 screen   — ncurses init / cleanup
§9 app      — signals, main loop
```

### Data Flow
```
arrow → TRI_DIR[dir][cur->up] → cur += delta
SPACE → pool_toggle(cur)
frame → grid_draw + pool_draw + cursor_draw
```
