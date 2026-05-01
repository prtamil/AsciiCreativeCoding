# Concept: Tetrakis Square Tiling

## Pass 1 — Understanding

### Core Idea
Each square is split by BOTH diagonals into four right-isosceles wedges meeting at the centre. Wedges are labelled by the direction their apex points: N, E, S, W. Eight triangles meet at every original square corner — vertex configuration `8.8.8.8`.

### Wedge Classifier
Translate the fractional pixel `(fa, fb)` so the square centre is the origin: `(dx, dy) = (fa − 0.5, fb − 0.5)`. The dominant axis and its sign pick the compass direction:
```
|dx| > |dy|, dx > 0  → E
|dx| > |dy|, dx ≤ 0  → W
|dy| ≥ |dx|, dy > 0  → S
|dy| ≥ |dx|, dy ≤ 0  → N
```

### Wedge Centroids
Each centroid is 1/3 of the way from the square centre to the apex vertex:
```
N: a = col + 1/2,  b = row + 1/6
E: a = col + 5/6,  b = row + 1/2
S: a = col + 1/2,  b = row + 5/6
W: a = col + 1/6,  b = row + 1/2
```

### Barycentric Per Wedge
Each wedge has three edges: two diagonals + one square side. The barycentric weights vary per wedge but the smallest weight always names the edge OPPOSITE that vertex; the character is the slope of that edge (`/`, `\`, `|`, `_`).

### Cursor Movement (TETRA_DIR)
4 arrow keys × 4 starting wedges → `(Δcol, Δrow, new_dir)`. An arrow toward the apex direction crosses a square boundary; an arrow opposite stays in the same square and toggles direction.

### Non-Obvious Decisions
- **Centre tie at fa = fb = 0.5**: the `|dx|>|dy|` test using strict `>` and `≥` makes the centre default to N/S over E/W. A measure-zero set; harmless.
- **Vertex configuration 8.8.8.8**: every square corner sees 8 triangles (4 from this square + 4 from the diagonally adjacent squares meeting at that corner). This is one of the standard Archimedean tilings.
- **Coordinate aspect**: `CELL_W=2, CELL_H=4` keeps wedges right-isosceles in pixel space.

### Key Constants
| Name | Role |
|------|------|
| `TRI_SIZE` | Square side length in pixels |
| `BORDER_W` | Barycentric threshold for edge characters |
| `DIR_N/E/S/W` | Wedge direction enum (0..3) |

### Open Questions
- How does the kisrhombille (file 04) generalise this for triangles instead of squares?
- What does the tetrakis tiling look like dual? (Answer: the truncated square tiling 4.8.8.)

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — TRI_SIZE, BORDER_W, DIR_N/E/S/W
§4 formula  — wedge classifier + barycentric per wedge
§5 cursor   — TETRA_DIR[4][4][3] + step + draw
§6 scene    — grid_draw + scene_draw
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
(row, col) → pixel → fractional (fa, fb) → centred (dx, dy)
                                              ↓
                                       wedge classifier → dir ∈ {N,E,S,W}
                                              ↓
                                tri_edge_char → '/', '\', '|', '_' → mvaddch
arrow key → TETRA_DIR[arrow][cur->dir] → cur += delta
```
