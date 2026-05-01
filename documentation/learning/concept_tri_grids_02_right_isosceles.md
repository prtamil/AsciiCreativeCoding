# Concept: Right-Isosceles Half-Rect Grid

## Pass 1 — Understanding

### Core Idea
A unit-square lattice bisected by a single `'\'` diagonal. Each square holds an upper-right (UR) and a lower-left (LL) right-isosceles triangle (legs = 1, hypotenuse = √2). Pixel→lattice is axis-aligned — NO shear — making the inverse a plain `(px/size, py/size)`.

### Forward Map (lattice → pixel)
```
px = a · size
py = b · size
```

### Inverse Map (pixel → lattice)
```
a = px / size,   b = py / size
col = ⌊a⌋,        row = ⌊b⌋
fa  = a − col,    fb = b − row
up  = (fa ≥ fb) ? UR : LL          // above the '\' diagonal → UR
```

### Centroids
```
UR centroid:  a = col + 2/3,  b = row + 1/3
LL centroid:  a = col + 1/3,  b = row + 2/3
```

### Barycentric Edge Picker
Three weights per orientation; smallest names the closest edge:
```
UR:  l₁ = 1−fa → '|'   l₂ = fa−fb → '\'   l₃ = fb     → '_'
LL:  l₁ = 1−fb → '_'   l₂ = fa    → '|'   l₃ = fb−fa  → '\'
```

### Cursor Movement (TRI_DIR)
4 arrows × 2 orientations → `(Δcol, Δrow, target_up)`. UP from UR jumps to row−1; UP from LL toggles inside the same square. Two LEFTs from UR traverse one whole square: `UR → LL(same) → UR(col−1)`.

### Non-Obvious Decisions
- **Diagonal tie at fa = fb**: defaults to UR via the `≥` comparison. A measure-zero set; integer round-off may flicker for one frame on resize.
- **One diagonal vs two**: this file is the minimum non-trivial triangulation of a square. Both diagonals (file 03) gives 4 wedges.
- **Aspect**: CELL_W=2, CELL_H=4 means terminal cells are 1:2 wide:tall. Triangles are right-isosceles in pixel space; on screen they look taller than wide — that is faithful, not a bug.

### Key Constants
| Name | Role |
|------|------|
| `TRI_SIZE` | Square side length in pixels |
| `BORDER_W` | Barycentric threshold for edge characters |

### Open Questions
- What changes if the diagonal flips to `/` instead of `\`?
- Why are centroids at thirds (1/3, 2/3) and not halves?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — TRI_SIZE, BORDER_W
§4 formula  — pixel_to_tri, tri_centroid_pixel, tri_edge_char
§5 cursor   — TRI_DIR + step + draw
§6 scene    — grid_draw + scene_draw
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
(row, col) → pixel → pixel_to_tri → (col, row, up=UR/LL) + (fa, fb)
                                       ↓
                        tri_edge_char → '|', '_', '\' → mvaddch
arrow key → TRI_DIR[dir][cur->up] → cur += delta
```
