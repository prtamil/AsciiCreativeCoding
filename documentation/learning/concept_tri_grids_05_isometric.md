# Concept: Isometric Solid-Fill Triangular Grid

## Pass 1 — Understanding

### Core Idea
The equilateral skew lattice from `01_equilateral`, but instead of rendering edge characters, every triangle is FILLED with a solid colour from a 6-cycle palette. The fill is `mvaddch(' ')` with the colour pair set by `palette_index = (col + 2·row + up) mod 6`. Six triangles meeting at any vertex spell out the full 6-cycle, and the eye reads three adjacent colours as the top, left, and right faces of a cube — producing the classic "stacked cubes" isometric look.

### Palette Hash
```
k = (col + 2·row + up) mod N_PALETTE      // N_PALETTE = 6
if k < 0:  k += N_PALETTE                  // safe negative modulo
pair = PAIR_FILL_BASE + k
```

### Why That Hash?
Walking RIGHT shifts `k` by +1. Walking UP shifts `k` by +2. Toggling `up` shifts `k` by +1. Tracing the 6 triangles around any vertex (alternating up/down across the rhombus diagonal) accumulates `+1 +2 +1 +1 +2 +1 = 8 ≡ 2 (mod 6)` per half-cycle and exactly +12 ≡ 0 over a full cycle — so the colour wheel closes around every vertex.

### Rendering
For every screen cell:
```
pixel_to_tri → (col, row, up)
attron(COLOR_PAIR(PAIR_FILL_BASE + palette_index(col, row, up)))
mvaddch(row, col, ' ')         // space + bg colour = a colour block
```
No edge characters. The cell IS the colour.

### Cursor
'@' renders in `PAIR_CURSOR` (white-on-black) so it always pops on any palette colour. Cursor walks lattice triangles via the same `TRI_DIR` table as `01_equilateral`.

### Non-Obvious Decisions
- **Reverse video for cursor**: necessary because the background is itself coloured. A non-contrasting cursor pair would disappear over its matching fill.
- **Themes**: each theme is a different 6-colour palette. Cycling themes recolours the field but does NOT change the palette HASH — the same triangle index keeps its slot, just with a new colour.
- **Why 2·row not row**: with `(col + row + up)`, walking diagonally (col+1, row+1) would give the same colour, breaking the cube illusion. The 2× weight on `row` makes vertical walks shift colours twice as fast.

### Key Constants
| Name | Role |
|------|------|
| `TRI_SIZE` | Equilateral side length in pixels |
| `N_PALETTE` | 6 — number of palette slots |
| `N_THEMES` | 3 or 4 colour themes |
| `PAIR_FILL_BASE` | First palette pair ID; +k for slot k |

### Open Questions
- Why does the iso illusion only work with exactly 6 colours? (Hint: 6 = number of triangles around a vertex.)
- What hash would tile a different polyhedron — say, octahedra?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — TRI_SIZE, N_PALETTE, N_THEMES, PAIR_FILL_BASE
§3 color    — PAL256 / PAL8 per-theme palettes; init_pair for each slot
§4 formula  — pixel_to_tri (same as 01) + palette_index hash
§5 cursor   — TRI_DIR (same as 01)
§6 scene    — solid-fill raster + cursor
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
(row, col) → pixel → pixel_to_tri → (col, row, up)
                                       ↓
                          palette_index → PAIR_FILL_BASE + k → mvaddch(' ')
arrow key → TRI_DIR[dir][cur->up] → cur += delta
```
