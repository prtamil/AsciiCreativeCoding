# Concept: Equilateral Triangular Grid

## Pass 1 — Understanding

### Core Idea
The base file in the `tri_grids/` series. Every screen pixel is inverse-mapped to a 2-axis SKEW lattice; integer parts give the rhombus, the diagonal `fa + fb = 1` splits the rhombus into ▽ (apex-down) and △ (apex-up) equilaterals. Border characters come from a per-orientation barycentric edge picker. The cursor walks lattice triangles via a 4-direction lookup table.

### Lattice Basis
Two non-orthogonal basis vectors:
```
v1 = (size, 0)             axis-aligned
v2 = (size/2, h)           h = size · √3 / 2
```
A rhombus `(col, row)` = parallelogram with corners P00 + i·v1 + j·v2. Each rhombus holds two triangles separated by its short diagonal.

### Forward Map (lattice → pixel)
```
px = (a + 0.5 · b) · size
py = b · h            with h = size · √3 / 2
```

### Inverse Map (pixel → lattice)
```
b = py / h
a = px / size − 0.5 · b      // undo the v2 shear
col = ⌊a⌋,  row = ⌊b⌋
fa  = a − col,  fb = b − row
up  = (fa + fb ≥ 1) ? △ : ▽
```

### Barycentric Edge Picker
Per orientation, the smallest of three weights names the edge OPPOSITE that vertex; the character map is:
```
▽:  l₁ = 1−fa−fb → '/'   l₂ = fa → '\'   l₃ = fb → '_'
△:  l₁ = 1−fb    → '_'   l₂ = fa+fb−1 → '/'   l₃ = 1−fa → '\'
```
If `min(l₁, l₂, l₃) ≥ BORDER_W`, the cell is interior and skipped.

### Cursor Movement (TRI_DIR)
4 arrow directions × 2 starting orientations → table of `(Δcol, Δrow, target_up)`. Two consecutive UP presses advance one full strip from any starting orientation: `△ → ▽(same rhombus) → △(strip above)`.

### Non-Obvious Decisions
- **Skew over orthogonal**: an axis-aligned lattice would need a more complex per-cell triangle test; the skew makes "which triangle?" a fa+fb comparison.
- **Float floor near boundaries**: 1-ULP jitter at integer borders is acceptable for visual demos. An industrial implementation would round-with-tie-breaking.
- **△ has no horizontal edge above**: UP from △ toggles inside the rhombus rather than crossing an edge — so UP·UP traverses one strip from any orientation.

### Key Constants
| Name | Role |
|------|------|
| `TRI_SIZE` | Side length of one equilateral triangle in pixels |
| `BORDER_W` | Barycentric threshold for "near an edge" |
| `CELL_W, CELL_H` | Sub-pixels per terminal char (2, 4 makes squares look square) |

### Open Questions
- Why does the edge tie-break favour `\` over `_` when two weights are equal?
- What lattice basis would produce hex tilings instead of triangular?

---

## Pass 2 — Implementation

### Pseudocode
```
for each screen (row, col):
    px = (col − ox) · CELL_W
    py = (row − oy) · CELL_H
    pixel_to_tri(px, py) → (col, row, up, fa, fb)
    ch = tri_edge_char(up, fa, fb)  → returns char + min weight m
    if m < BORDER_W:
        mvaddch(row, col, ch)
```

### Module Map
```
§1 config   — TRI_SIZE, BORDER_W, CELL_W, CELL_H
§4 formula  — pixel_to_tri, tri_centroid_pixel, tri_edge_char
§5 cursor   — TRI_DIR[4][2][3], cursor_step, cursor_draw
§6 scene    — grid_draw + scene_draw
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
(row, col) → pixel → pixel_to_tri → (col, row, up) + (fa, fb)
                                       ↓
                              tri_edge_char → '/'  '\'  '_'  → mvaddch
arrow key → TRI_DIR[dir][cur->up] → cur += delta
```
