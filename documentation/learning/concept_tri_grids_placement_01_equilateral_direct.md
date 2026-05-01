# Concept: Equilateral Direct Placement

## Pass 1 — Understanding

### Core Idea
Direct cursor-driven placement on the equilateral skew lattice from `tri_grids/01_equilateral.c`. The cursor holds (col, row, up); SPACE toggles a glyph at that address. Objects survive resize because they're stored by lattice address, not by pixel position.

### Two Address Spaces
Two structures share the screen each frame:
1. The GRID — derived per-pixel via `pixel_to_tri` from the equilateral skew lattice. No data, rebuilt every frame.
2. The OBJECT POOL — flat array of `TObj{col, row, up, glyph}`. Cursor is one more such address; SPACE toggles its membership.

### Pool Toggle (O(1) Remove)
```
i = pool_find(col, row, up)        // O(n) linear scan
if i >= 0:
    pool->objs[i] = pool->objs[--pool->count]    // swap with last
else:
    pool->objs[pool->count++] = (TObj){col, row, up, glyph}
```

### Cursor (Same as 01_equilateral)
4-direction TRI_DIR table → `(Δcol, Δrow, target_up)`. Two consecutive UP presses advance one full strip from any orientation.

### Centroid Render
For each placed object:
```
tri_centroid_pixel(col, row, up, size) → (cx_pix, cy_pix)
scol = ox + (int)(cx_pix / CELL_W)
srow = oy + (int)(cy_pix / CELL_H)
mvaddch(srow, scol, glyph)
```
The truncation (not round) keeps the glyph inside the triangle interior.

### Non-Obvious Decisions
- **Resize keeps addresses, moves window**: ox/oy are recomputed each frame from `(cols/2, (rows-1)/2)`; the lattice (col, row, up) for an object stays unchanged.
- **Glyph cycle affects NEW placements only**: cycling the glyph index doesn't rewrite already-placed objects.
- **MAX_OBJ silently caps**: full-pool insertions are dropped without warning. 'C' clears the pool.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity (256) |
| `N_GLYPHS` | Number of cycle-able glyphs |
| `BORDER_W` | Edge-proximity threshold for grid render |

### Open Questions
- What changes if you store objects by pixel position instead of lattice address?
- How would multi-select (region drag) integrate with the swap-with-last removal?

---

## Pass 2 — Implementation

### Pseudocode
```
each frame:
    grid_draw   → raster-scan equilateral edges
    pool_draw   → glyph at each placed object's centroid cell
    cursor_draw → '@' at cursor centroid
arrow key → cur ← TRI_DIR[dir][cur->up]
SPACE     → pool_toggle(cur.col, cur.row, cur.up, glyph)
'C'       → pool_clear
'g'       → glyph_idx = (glyph_idx + 1) % N_GLYPHS
```

### Module Map
```
§1 config   — MAX_OBJ, N_GLYPHS, BORDER_W
§4 formula  — pixel_to_tri, tri_centroid_pixel, tri_edge_char (same as tri_grids/01)
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
