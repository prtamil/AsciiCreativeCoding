# Concept: Hex Subdivision (Hex With Three Diagonals)

## Pass 1 — Understanding

### Core Idea
A flat-top hex grid where each hexagon is divided into 6 equilateral triangles by drawing the three "long diagonals" through its centre. Every hex has the property that its circumradius equals its edge length, so each wedge's three sides are the hex circumradius, the hex circumradius, and the hex edge — all equal — making each wedge equilateral.

### Two-Tiered Address
1. **Hex membership** `(Q, R)` via the cube-rounding inverse (same as `hex_grids/01_flat_top.c`).
2. **Sector** `s ∈ 0..5` via `atan2` of the offset from the hex centre, rotated by π/6 so sector 0 is centred on +x.

### Pixel → Hex (cube-round)
```
fq = (2/3 · px) / size
fr = (−1/3 · px + (√3/3) · py) / size
fs = −fq − fr
round (fq, fr, fs) to integers; the residual with the largest |error|
is recomputed from the other two so Q + R + S = 0 holds.
```

### Sector Classifier
```
dx = px − cx,  dy = py − cy        // (cx, cy) = hex centre
ang = atan2(dy, dx)
s = floor((ang + π/6) / (π/3)) mod 6
```
The +π/6 bias places the +x axis solidly in sector 0 (not on its boundary).

### Rendering
For every screen cell:
- `pixel_to_hex` → `(Q, R, dist)` where `dist` is the max cube-coord residual.
- If `dist > 0.5 − BORDER_W`: render the hex border via `angle_char(theta)`.
- Else: test proximity to each of the 3 long diagonals; if within `RADIUS_T_FRAC × size`, render `/`, `\`, or `-` for that radius.

### Cursor (Q, R, sector)
Arrow keys walk hexes via `HEX_DIR[4]` (4 of the 6 hex neighbours). `,` and `.` rotate the sector inside the current hex (CCW / CW).

### Non-Obvious Decisions
- **Six wedges per hex are equilateral**: special property of regular hexagons; would not hold for irregular hexes.
- **angle_char shared with hex_grids**: `theta + π/2` shifts the angle so vertical edges read as `|`.
- **Sector address vs pixel-derived sector**: this file lets the cursor CARRY a sector independent of the cursor pixel position; the placement variants (`06_hex_subdivision_*`) re-derive sector from the cursor pixel for path/scatter computations.

### Key Constants
| Name | Role |
|------|------|
| `HEX_SIZE` | Hex circumradius (= edge length) in pixels |
| `BORDER_W` | Outer-edge proximity threshold |
| `RADIUS_T_FRAC` | Inner-radius proximity as a fraction of hex size |

### Open Questions
- Why does cube-rounding fix the LARGEST residual instead of the smallest?
- What happens visually if the sector bias is removed (no +π/6)?

---

## Pass 2 — Implementation

### Pseudocode
```
for each screen (row, col):
    px = (col − ox) · CELL_W
    py = (row − oy) · CELL_H
    pixel_to_hex(px, py) → (Q, R, dist)
    hex_centre_pixel(Q, R) → (cx, cy)
    if dist > 0.5 − BORDER_W:
        mvaddch(angle_char(atan2(py−cy, px−cx) + π/2))
    else:
        for each of 3 long diagonals:
            d = perpendicular_distance(px−cx, py−cy, diag)
            if d < RADIUS_T_FRAC · size:
                mvaddch(diagonal_char)
```

### Module Map
```
§1 config   — HEX_SIZE, BORDER_W, RADIUS_T_FRAC
§4 formula  — pixel_to_hex (cube-round), sector_of (atan2 bin),
              hex_centre_pixel, wedge_centroid_pixel
§5 cursor   — HEX_DIR[4] + sector rotate
§6 scene    — grid_draw (hex border + 3 radii) + scene_draw
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
(row, col) → pixel → pixel_to_hex → (Q, R) + dist
                                       ↓
                         hex border via angle_char  (outer pixels)
                         3 radii proximity → '/', '\', '-'  (inner pixels)
arrow key → HEX_DIR[idx] → (Q, R) += delta
,/.       → sector ± 1 (mod 6)
```
