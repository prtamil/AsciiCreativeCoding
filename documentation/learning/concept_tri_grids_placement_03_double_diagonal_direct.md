# Concept: Double-Diagonal (Tetrakis) Direct Placement

## Pass 1 — Understanding

### Core Idea
Direct cursor-driven placement on the tetrakis grid from `tri_grids/03_double_diagonal.c`. Each square is split into 4 right-isosceles wedges (N/E/S/W). The cursor address is (col, row, dir); SPACE toggles a glyph at that wedge. Each arrow press moves toward the matching compass direction — within the current square if possible, jumping squares when at the matching apex.

### Wedge Address
```
DIR_N = 0,  DIR_E = 1,  DIR_S = 2,  DIR_W = 3
```
Each `(col, row)` square holds all 4 wedges simultaneously.

### Wedge Centroids
Each centroid is 1/3 of the way from the square centre to the apex:
```
N: (col + 1/2, row + 1/6) · size
E: (col + 5/6, row + 1/2) · size
S: (col + 1/2, row + 5/6) · size
W: (col + 1/6, row + 1/2) · size
```

### Cursor Movement (TETRA_DIR)
4 arrows × 4 starting wedges = 16 transitions encoded in `TETRA_DIR[arrow][cur->dir] → (Δcol, Δrow, new_dir)`. An arrow toward the apex direction crosses a square boundary; an arrow opposite stays in the same square.

### Pool Toggle
Same swap-with-last as 01_equilateral_direct, but indexed by `(col, row, dir)` instead of `(col, row, up)`.

### Non-Obvious Decisions
- **4-direction cursor on 4-wedge cells**: a natural fit — every arrow has one specific wedge it leads to.
- **Aspect**: CELL_W=2, CELL_H=4 keeps wedges right-isosceles in pixel space; on screen they look stretched vertically.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity (256) |
| `BORDER_W` | Edge-proximity threshold |

### Open Questions
- Why 4 wedges instead of 8 (vertex configuration would change to `4.4.4.4.4.4.4.4`)?
- How would you implement smooth diagonal movement (NE, NW, SE, SW arrows)?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, BORDER_W, DIR_N/E/S/W
§4 formula  — wedge classifier + barycentric per wedge
§5 pool     — TObj{col, row, dir, glyph}, pool_find, pool_toggle
§6 cursor   — TETRA_DIR[4][4][3] + step + draw
§7 scene    — grid_draw + scene_draw
§8 screen   — ncurses init / cleanup
§9 app      — signals, main loop
```

### Data Flow
```
arrow → TETRA_DIR[arrow][cur->dir] → cur += delta
SPACE → pool_toggle(cur)
frame → grid_draw + pool_draw + cursor_draw
```
