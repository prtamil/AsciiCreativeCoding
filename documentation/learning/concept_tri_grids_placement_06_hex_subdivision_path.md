# Concept: Hex-Subdivision Line-of-Sight Path

## Pass 1 — Understanding

### Core Idea
Pixel-walk path between two wedge centroids on the hex-subdivision grid. Each sample's `(Q, R, sector)` is recovered by `pixel_to_hex` (cube-rounding) plus a sector classifier from `atan2` of the offset from the hex centre. The path is the ordered list of `(Q, R, sector)` wedges the line crosses.

### Sector Classifier
```
dx = px − cx,  dy = py − cy
ang = atan2(dy, dx)
sector = floor((ang + π/6) / (π/3)) mod 6
```
The +π/6 bias places +x solidly in sector 0 (not on its boundary).

### Walk
```
for i in 0..n:
    t  = i / n
    px = sx + t·dx,  py = sy + t·dy
    pixel_to_hex(px, py) → (Q, R)
    hex_centre_pixel(Q, R) → (cx, cy)
    sector_of(px − cx, py − cy) → s
    path_add(Q, R, s)
```

### Recompute Triggers
'`s'`, `'e'`, `'+/-'`. NOT cursor move alone.

### Density Note
Wedges are smaller than the hex; sampling `step = hex_size · 0.25` is sufficient. A coarser step risks skipping a wedge crossed corner-on at the hex centre.

### Apex At Hex Centre
When the line passes near the hex centre, the +π/6 bias keeps sector 0 deterministic. Without the bias, atan2 ≈ 0 would oscillate between sector 0 and sector 5 due to floating-point rounding.

### Non-Obvious Decisions
- **Two classifiers per sample**: cube-round for hex membership + atan2 for sector. Both are O(1).
- **Recompute on size change**: '+'/'-' must call path_compute; centroids depend on size.
- **PathPool linear scan dedup**: `path_contains` is O(n); paths stay short for any visible line.

### Key Constants
| Name | Role |
|------|------|
| `HEX_SIZE` | Hex circumradius (= edge length) |
| `MAX_OBJ` | Path pool cap |

### Open Questions
- Why does cube-round resolve the LARGEST residual rather than smallest?
- Could the path include hex EDGES (non-wedge segments) when the line aligns with one?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — HEX_SIZE, MAX_OBJ
§4 formula  — pixel_to_hex + sector_of + wedge_centroid_pixel
§5 pool     — PathPool{Q, R, sector}: clear, contains, add, draw
§6 cursor   — HEX_DIR + step_hex + rotate_sector + S/E markers
§7 path     — path_compute pixel-walk
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
's' / 'e' / '+/-' → path_compute → fill PathPool
frame → grid_draw + path_draw + S/E markers + cursor_draw
```
