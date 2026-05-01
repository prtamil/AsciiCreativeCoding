# Concept: Right-Isosceles Distance-Coloured Scatter

## Pass 1 — Understanding

### Core Idea
Random scatter of (col, row, up) entries within ±SCATTER_RADIUS of the cursor on the half-rect grid. Each entry coloured by Manhattan-style cell distance from the cursor, bucketed into 6 gradient slots. SPACE reseeds; cursor movement RECOLOURS without reseeding.

### Storage vs Colouring (Same Split As 01)
- **STORAGE**: random scatter once per reseed.
- **COLOURING**: per-frame function of cursor distance.

### Distance Metric
```
d = |obj.col − cur.col| + |obj.row − cur.row| + (obj.up != cur.up ? 1 : 0)
bucket = min(d, N_BUCKETS − 1)
```

### Reseed
LCG `g_seed ^= clock_ns()` then pick random offsets in `±SCATTER_RADIUS` and a random `up`. Dedup via `pool_add`.

### Non-Obvious Decisions
- **Manhattan vs true distance**: a true edge-walk distance on the half-rect lattice has to account for the `\` diagonal. Manhattan is a fast proxy for visual purposes.
- **Reseed on `+/-` density**: increasing/decreasing density without a fresh scatter would make the displayed count drift; this file reseeds on density change.

### Key Constants
| Name | Role |
|------|------|
| `SCATTER_RADIUS` | Half-extent of spawn box (cells) |
| `DENSITY_DEFAULT` | Starting count |
| `N_BUCKETS` | 6 — colour gradient stops |

### Open Questions
- Does true axial distance change the visual significantly?
- How would you animate the gradient (palette cycling vs cursor jitter)?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — SCATTER_RADIUS, DENSITY_DEFAULT, N_BUCKETS
§4 formula  — pixel_to_tri (axis-aligned), tri_centroid_pixel
§5 pool     — ScatterPool: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw
§7 scatter  — random spawn + per-frame bucket
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
SPACE / +/- → reseed → fill ScatterPool with random (col, row, up)
frame:
    for each entry:
        d      = manhattan_distance(entry, cursor)
        bucket = min(d, N_BUCKETS - 1)
        attron(PAIR_BUCKET_0 + bucket)
        mvaddch(centroid_screen, '*')
arrow → cur += TRI_DIR (no reseed)
```
