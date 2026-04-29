# Concept: Flat-Top Hexagonal Grid

## Pass 1 — Understanding

### Core Idea
The foundational hex grid file. Every screen pixel is inverse-mapped to axial (Q,R) coordinates using the flat-top forward matrix, then `cube_round` identifies the nearest hex. Border glyphs are picked by `angle_char(θ)` — the angle from the hex centre to the pixel determines which of `|`, `/`, `\` to draw.

### Coordinate System
Axial (Q, R) with implicit S = −Q−R. The cube constraint Q+R+S=0 means all valid hexes lie on a 2D plane in 3D cube space. This redundancy makes distance arithmetic trivial: `hex_dist = (|dQ|+|dR|+|dS|)/2`.

### Forward Matrix (flat-top orientation)
```
cx = size × 3/2 × Q
cy = size × (√3/2 × Q  +  √3 × R)
```

### Inverse Matrix
```
fq = (2/3 × px) / size
fr = (−px/3 + √3×py/3) / size
fs = −fq − fr
```
Then `cube_round(fq, fr, fs)` snaps fractional coords to the nearest integer hex.

### Non-Obvious Decisions
- **`cube_round` not just `round()`**: rounding all three independently violates Q+R+S=0. The fix: find which component has the largest rounding error and recompute it from the other two.
- **`angle_char` uses atan2 then quantises**: 6 sectors → 3 border characters (pairs of opposite sectors share the same character).
- **Border rendering O(rows×cols)**: iterating every screen pixel and asking "which hex are you in?" produces correct borders without any polygon-drawing code.

### Key Constants
| Name | Role |
|------|------|
| `HEX_SIZE` | Circumradius of one hex in pixels |
| `CELL_W, CELL_H` | Terminal cell size in pixels |
| `BORDER_W` | Fractional cube-distance threshold for border pixels |

### Open Questions
- What changes in the forward matrix for pointy-top orientation?
- Why does `cube_round` fix the largest error component rather than the smallest?
- Can the same `cube_round` function handle both flat-top and pointy-top?

---

## Pass 2 — Implementation

### Pseudocode
```
for each screen (row, col):
    px = (col - ox) * CELL_W
    py = (row - oy) * CELL_H
    (fq, fr, fs) = inverse_matrix(px, py, size)
    (Q, R) = cube_round(fq, fr, fs)
    dist = max(|fq-Q|, |fr-R|, |fs-S|)
    if dist >= 0.5 - BORDER_W:
        theta = atan2(py - cy(Q,R), px - cx(Q,R))
        mvaddch(row, col, angle_char(theta))
```

### Module Map
```
§1 config  — HEX_SIZE, BORDER_W, CELL_W/H, colour pairs
§4 coords  — cube_round(), hex_to_screen(), angle_char()
§5 grid    — grid_draw(): O(rows×cols) raster scan
§8 app     — main loop, SIGWINCH resize handler
```

### Data Flow
```
(row,col) → inverse_matrix → cube_round → border test → angle_char → mvaddch
```
