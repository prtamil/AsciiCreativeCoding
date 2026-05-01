# Concept: Kisrhombille Distance-Coloured Scatter

## Pass 1 — Understanding

### Core Idea
Random scatter of `(col, row, up)` parent-equilateral addresses on the kisrhombille grid, coloured by Manhattan-style cell distance from the cursor. The medians render automatically beneath each scatter dot; the dot is at the parent centroid.

### Distance Metric (Same As Equilateral_scatter)
```
d = |obj.col − cur.col| + |obj.row − cur.row| + (obj.up != cur.up ? 1 : 0)
bucket = min(d, N_BUCKETS − 1)
```

### Storage vs Colouring
- **STORAGE**: random scatter once per reseed (SPACE or `+/-` density).
- **COLOURING**: per-frame from cursor distance.

### Reseed
LCG `g_seed ^= clock_ns()`; pick offsets in ±SCATTER_RADIUS and a random `up`. Dedup via `pool_add`.

### Non-Obvious Decisions
- **Scatter dots over medians**: each dot's centroid lands on the parent's median concurrent point — the dot covers the median ink at that single cell.
- **Manhattan vs true edge distance**: same caveat as equilateral_scatter — actual edge-walk distance depends on parity. Manhattan is a fast proxy.

### Key Constants
| Name | Role |
|------|------|
| `SCATTER_RADIUS` | Half-extent of spawn box (cells) |
| `DENSITY_DEFAULT` | Starting count |
| `N_BUCKETS` | 6 — colour gradient stops |
| `MEDIAN_T` | Median-proximity threshold |

### Open Questions
- How would the gradient look if the metric used 30-60-90 wedge graph distance instead?
- Could the scatter dots be placed at sub-triangle centroids (6× as many addresses)?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — SCATTER_RADIUS, DENSITY_DEFAULT, N_BUCKETS, MEDIAN_T
§4 formula  — pixel ↔ skew lattice + centroid + median proximity
§5 pool     — ScatterPool{col, row, up}: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw
§7 scatter  — random spawn + per-frame bucket
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
SPACE / +/- → reseed → fill ScatterPool
frame:
    grid_draw (edges + medians)
    for each scatter entry:
        d      = manhattan + Δup penalty
        bucket = min(d, N_BUCKETS - 1)
        attron(PAIR_BUCKET_0 + bucket)
        mvaddch(centroid_screen, '*')
arrow → cur += TRI_DIR (no reseed)
```
