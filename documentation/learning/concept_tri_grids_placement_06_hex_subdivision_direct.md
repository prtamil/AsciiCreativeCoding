# Concept: Hex-Subdivision Direct Placement

## Pass 1 — Understanding

### Core Idea
Direct placement on a flat-top hex grid where each hex is split into 6 wedges by 3 long diagonals. The cursor address is `(Q, R, sector)` where `(Q, R)` are axial hex coords and `sector ∈ 0..5` is the wedge angle measured CCW from +x. Arrow keys move the hex; `,` / `.` rotate the cursor sector. SPACE toggles a glyph at the current wedge.

### Two Address Levels
1. The HEX `(Q, R)` — flat-top axial coordinates.
2. The WEDGE `sector` — one of 6 equilateral sub-triangles inside the hex.

### Pixel → Hex (Cube-Round)
```
fq = (2/3 · px) / size
fr = (−1/3 · px + (√3/3) · py) / size
fs = −fq − fr
round (fq, fr, fs) → integers, snap the largest |residual| back so Q+R+S = 0.
```

### Wedge Centroid (1/3 From Centre To Vertex)
```
angle = sector · π/3
r     = size · √3 / 3
cx_w  = cx + r · cos(angle)
cy_w  = cy + r · sin(angle)
```

### Cursor Movement
- arrows: `HEX_DIR[4]` walks 4 of the 6 hex neighbours via `(ΔQ, ΔR)` increments.
- `,` / `.`: `cursor->sector = (sector + Δ + 6) mod 6`.

### Pool Toggle (Same Pattern As Other _direct)
Swap-with-last; O(1) remove. Indexed by `(Q, R, sector)`.

### Non-Obvious Decisions
- **4 arrow keys, 6 hex neighbours**: this file uses ONLY the 4 cardinal arrow keys, mapping to 4 of 6 axial neighbours. Diagonal hex neighbours are reachable by chaining two key-presses. Intentional simplification.
- **Sector vs sector_of**: the cursor CARRIES a sector independently of pixel position. `pixel_to_hex` does NOT classify the sector here (only path / scatter siblings do).
- **dist threshold**: `limit_inner = 0.5 − BORDER_W`. Pixels with `dist > limit_inner` are near a hex edge → render border via `angle_char`.

### Key Constants
| Name | Role |
|------|------|
| `HEX_SIZE` | Hex circumradius (= edge length) in pixels |
| `BORDER_W` | Hex-edge proximity threshold |
| `RADIUS_T_FRAC` | Inner-radius proximity as a fraction of hex size |
| `MAX_OBJ` | Pool capacity |

### Open Questions
- How would you map all 6 hex neighbours to an extended key set (e.g. q/w/e/a/s/d)?
- What changes for pointy-top instead of flat-top? (Different forward matrix; sector orientation.)

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — HEX_SIZE, BORDER_W, RADIUS_T_FRAC, MAX_OBJ
§4 formula  — pixel_to_hex (cube-round), hex_centre_pixel,
              wedge_centroid_pixel, angle_char
§5 pool     — HObj{Q, R, sector, glyph}: clear, find, toggle, draw
§6 cursor   — HEX_DIR[4] + cursor_step_hex + cursor_rotate_sector
§7 scene    — grid_draw (hex border + 3 radii) + scene_draw
§8 screen   — ncurses init / cleanup
§9 app      — signals, main loop
```

### Data Flow
```
arrow → cur->Q/R += HEX_DIR[idx]
,/.   → sector ± 1 (mod 6)
SPACE → pool_toggle(cur->Q, cur->R, cur->sector)
frame → grid_draw + pool_draw + cursor_draw
```
