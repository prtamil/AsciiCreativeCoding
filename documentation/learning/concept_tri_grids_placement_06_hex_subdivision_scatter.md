# Concept: Hex-Subdivision Distance-Coloured Scatter

## Pass 1 — Understanding

### Core Idea
Random scatter of `(Q, R, sector)` wedge addresses around the cursor on the hex-subdivision grid. Each entry coloured by HEX AXIAL distance from the cursor (true graph distance, not Manhattan), bucketed into 6 gradient slots.

### Distance Metric (Exact, Not Manhattan)
```
d_hex = (|ΔQ| + |ΔR| + |ΔQ + ΔR|) / 2
d     = d_hex + (Δsector ? 1 : 0)
bucket = min(d, N_BUCKETS − 1)
```
This is the proper hex graph distance (also known as the Chebyshev distance in cube coords). The +1 sector penalty keeps neighbours inside one hex from collapsing to the same bucket.

### Why Exact Hex Distance
For hex grids, the cube-coordinate formula gives the true minimum-edges path length. Manhattan in axial coords would be wrong by ~half — adjacent hexes that share an edge can have `|ΔQ| + |ΔR| > 1` while their true distance is 1.

### Storage vs Colouring
- **STORAGE**: random scatter once per reseed (SPACE or `+/-` density).
- **COLOURING**: per-frame from cursor position.

### Reseed
LCG `g_seed ^= clock_ns()`; pick offsets in `±SCATTER_RADIUS` and a uniform `sector ∈ 0..5`. Dedup via `pool_add`.

### Non-Obvious Decisions
- **Hex axial distance is exact** — unlike triangular siblings that use Manhattan as a proxy.
- **Sector mismatch +1**: tiny correction that keeps intra-hex gradient visible. Setting it to 0 would mean all 6 wedges of the cursor's hex look identical.
- **Scatter radius is in HEX cells, not pixels**: a radius of 8 covers a roughly 16-hex-wide disc.

### Key Constants
| Name | Role |
|------|------|
| `HEX_SIZE` | Hex circumradius (= edge length) |
| `SCATTER_RADIUS` | Half-extent of spawn box (hex cells) |
| `DENSITY_DEFAULT` | Starting count |
| `N_BUCKETS` | 6 — colour gradient stops |

### Open Questions
- For a Voronoi-style colouring (each cell gets the colour of its nearest scatter dot), what algorithm fits?
- Could the bucket gradient be log-scaled instead of linear to emphasise close-range differences?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — HEX_SIZE, SCATTER_RADIUS, DENSITY_DEFAULT, N_BUCKETS
§4 formula  — pixel_to_hex + wedge_centroid_pixel + hex_distance
§5 pool     — ScatterPool{Q, R, sector}: clear, contains, add, draw
§6 cursor   — HEX_DIR + step_hex + rotate_sector
§7 scatter  — random spawn + per-frame bucket
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
SPACE / +/- → reseed → fill ScatterPool with random (Q, R, sector)
frame:
    grid_draw (hex border + 3 radii)
    for each scatter entry:
        d      = hex_distance(entry, cursor) + Δsector_penalty
        bucket = min(d, N_BUCKETS - 1)
        attron(PAIR_BUCKET_0 + bucket)
        mvaddch(wedge_centroid_screen, '*')
arrow → cur->Q/R += HEX_DIR (no reseed)
,/.   → sector ± 1 (no reseed)
```
