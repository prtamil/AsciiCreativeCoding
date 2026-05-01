# Concept: Double-Diagonal (Tetrakis) Distance-Coloured Scatter

## Pass 1 — Understanding

### Core Idea
Random scatter of `(col, row, dir)` wedge addresses within ±SCATTER_RADIUS of the cursor on the tetrakis grid. Each entry coloured by Manhattan-style cell distance (with `dir` mismatch as +1 penalty), bucketed into 6 gradient slots.

### Distance Metric
```
d = |obj.col − cur.col| + |obj.row − cur.row| + (obj.dir != cur.dir ? 1 : 0)
bucket = min(d, N_BUCKETS − 1)
```
The `Δdir` term keeps neighbour wedges inside one square from collapsing to the same bucket.

### Reseed
Same LCG approach as 01_equilateral_scatter; `dir` is uniform in 0..3.

### Non-Obvious Decisions
- **Manhattan vs true wedge-graph distance**: an exact wedge-graph distance has to count diagonal-crossings (each diagonal is an edge in the wedge graph). Manhattan is a fast proxy; the `Δdir` penalty is a coarse correction.
- **Reseed-on-density-change**: changing density triggers reseed — visible count tracks requested density.
- **Cursor sector rotation `,`/`.`** changes `cur->dir` only — recolours but does not reseed.

### Key Constants
| Name | Role |
|------|------|
| `SCATTER_RADIUS` | Half-extent of spawn box (cells) |
| `DENSITY_DEFAULT` | Starting count |
| `N_BUCKETS` | 6 — colour gradient stops |

### Open Questions
- How does the gradient differ if the metric uses true wedge-graph distance?
- Could the `dir` mismatch penalty be 2 (forcing a stronger gradient) instead of 1?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — SCATTER_RADIUS, DENSITY_DEFAULT, N_BUCKETS
§4 formula  — wedge classifier + wedge centroids
§5 pool     — ScatterPool{col, row, dir}: clear, contains, add, draw
§6 cursor   — TETRA_DIR + step + draw
§7 scatter  — random spawn + per-frame bucket
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
SPACE / +/- → reseed → fill ScatterPool with random (col, row, dir)
frame:
    for each entry:
        d      = manhattan + dir_penalty
        bucket = min(d, N_BUCKETS - 1)
        attron(PAIR_BUCKET_0 + bucket)
        mvaddch(centroid_screen, '*')
arrow → cur += TETRA_DIR (no reseed)
,/.  → sector ± 1 (no reseed)
```
