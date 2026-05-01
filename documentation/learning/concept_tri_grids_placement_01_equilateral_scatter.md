# Concept: Equilateral Distance-Coloured Scatter

## Pass 1 — Understanding

### Core Idea
A random scatter of N triangles fills a square region around the cursor. Each placed triangle is coloured on a 6-stop gradient by Manhattan-style cell distance from the cursor — closer = warm, farther = cool. SPACE reseeds; cursor movement RECOLOURS without reseeding.

### Two-Halves Architecture
- **STORAGE**: random scatter generated once per reseed (SPACE or `+/-` density). Stored in `ScatterPool` as `(col, row, up)` entries.
- **COLOURING**: pure function of cursor distance, recomputed every frame. Bucket assignment for each entry happens during render, not during reseed.

This separation means moving the cursor never disturbs the scatter — only the colours flow under it.

### Distance Metric
```
d = |obj.col − cur.col| + |obj.row − cur.row| + (obj.up != cur.up ? 1 : 0)
bucket = min(d, N_BUCKETS − 1)
```
Cheap, monotonic in close range. The `Δup` term is a small penalty for being in a different half of the same rhombus.

### Reseed
```
on SPACE or +/- density change:
    pool->count = 0
    g_seed ^= clock_ns()
    for i in 0..density:
        dC = floor(frand·(2·R+1)) − R         // ±R range
        dR = floor(frand·(2·R+1)) − R
        up = (frand > 0.5) ? △ : ▽
        pool_add(cur.col + dC, cur.row + dR, up)    // dedup
```

### Bucket Palette
Six colour pairs from warm to cool. Theme cycle re-tunes the palette without reseeding.

### Non-Obvious Decisions
- **Cursor movement ≠ reseed**: this is the key UX choice. Lets the user see how distance colouring works with the same scatter from different vantage points.
- **Manhattan vs true edge distance**: the actual edge-walk distance on the equilateral lattice depends on parity. Manhattan is a fast proxy for SHORT distances; for long ones the gradient may "bend" visually. Acceptable for a colouring demo.
- **Density vs dedup**: at high density approaching MAX_OBJ, `pool_add` deduplicates so the visible count may be lower than requested.

### Key Constants
| Name | Role |
|------|------|
| `SCATTER_RADIUS` | Half-extent of the spawn box (cells) |
| `DENSITY_DEFAULT` | Starting count of scatter entries |
| `N_BUCKETS` | 6 — colour gradient stops |

### Open Questions
- What hash would give a true edge-walk distance on the equilateral lattice?
- How would you animate a "ripple" — reseed the bucket assignment with a phase offset over time?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — SCATTER_RADIUS, DENSITY_DEFAULT, N_BUCKETS
§4 formula  — pixel_to_tri, tri_centroid_pixel
§5 pool     — ScatterPool: clear, contains, add, draw
§6 cursor   — TRI_DIR + step + draw
§7 scatter  — random spawn + bucket per frame
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
SPACE / +/- → reseed → fill ScatterPool with random (col, row, up)
frame:
    for each entry:
        d = manhattan_distance(entry, cursor)
        bucket = min(d, N_BUCKETS - 1)
        attron(PAIR_BUCKET_0 + bucket)
        mvaddch(centroid_screen, '*')
arrow → cur += TRI_DIR (no reseed)
```
