# Concept: Procedural Scatter (04_scatter.c)

## Pass 1 — Understanding

### Core Idea
Four placement algorithms that treat object distribution as a **sampling strategy** over grid cells. Each strategy defines a different probability distribution: uniform random, blue-noise rejection, graph-distance order, and distance-weighted probability. Pressing a key triggers one scatter pass on the current grid.

### Mental Model

Think of the grid as a canvas of potential object positions. Each scatter algorithm answers the question "which cells get objects?" differently:

| Key | Name | Distribution |
|-----|------|-------------|
| `R` | Random | Uniform — every valid cell equally likely |
| `M` | Min-distance | Blue-noise — uniform with rejection if too close to existing objects |
| `F` | Flood | BFS-ordered — cells at graph distance 1 before 2 before 3 from cursor |
| `G` | Gradient | Distance-weighted — probability ∝ `1/(dist_from_centre + scale)` |

### Key Equations

**Chebyshev distance** (used by M and G):
```
cheb(r0,c0, r1,c1) = max(|r0-r1|, |c0-c1|)
```
Equal to the number of king-moves (8-connected steps) between cells.

**Gradient threshold** (integer arithmetic to avoid overflow):
```
P(cell) = GRAD_SCALE / (chebyshev(cell, centre) + GRAD_SCALE)
threshold = (long)GRAD_SCALE * RAND_MAX / (dist + GRAD_SCALE)
rand() < threshold  →  place object
```
Using `(long)` cast prevents `GRAD_SCALE * RAND_MAX` overflowing `int`.

**BFS flood**:
```
enqueue(cursor); vis[cursor] = true
while queue not empty and placed < FLOOD_MAX:
    cell = dequeue
    pool_place(cell)
    for each 4-connected neighbour n of cell:
        if not vis[n]: enqueue(n); vis[n]=true
```

### Data Structures

- **ObjectPool**: same as `01_direct.c`.
- **BFS**: heap-allocated `bool vis[area]`, `int qr[area]`, `int qc[area]` (ring buffer). Heap allocation avoids stack overflow on large grids; freed after each flood.
- **`ENQUEUE` macro**: a `#define … do { … } while(0)` macro replacing a nested function (which C11 forbids). `#undef ENQUEUE` follows immediately after the BFS loop.

### Non-Obvious Decisions

- **Chebyshev over Euclidean**: No `sqrt`, no floating point; naturally defines a square "ball" that matches grid aesthetics. The Chebyshev ball of radius k is exactly the cells reachable in k king-moves.
- **`(long)` cast for gradient threshold**: `GRAD_SCALE=6` and `RAND_MAX=2147483647`. Their product is `~12.9 billion` — overflows `int`. Casting to `long` before multiplication is the minimal fix.
- **BFS uses 4-connectivity**: 8-connectivity would produce concentric squares; 4-connectivity produces concentric diamonds (Manhattan ball). Diamonds look more organic on grid types that have diagonal structure.
- **Poisson-disk is O(n²)**: The min-distance rejection test checks every existing object. For `MAX_OBJ=256` this is at most `256 × 400 = 102,400` comparisons per scatter — fast enough to be imperceptible.
- **No `pool_remove`**: Scatter only adds; `C` clears all. The swap-last removal from `01_direct` is unused here and was removed to avoid compiler warnings.

### Key Constants

| Name | Role |
|------|------|
| `RAND_N=40` | objects placed per R press |
| `MAX_TRIES=400` | rejection sampling attempts for M |
| `MIN_DIST=3` | Chebyshev min-distance between objects for M |
| `FLOOD_MAX=120` | max BFS cells for F |
| `GRAD_SCALE=6` | gradient denominator scale; larger = wider coverage |

### Open Questions

- Why does `FLOOD_MAX=120` from the screen centre always reach the same visual radius regardless of grid type?
- What happens to `scatter_gradient` on a diamond grid — does the Chebyshev ball still match the visible cell pattern?
- If `GRAD_SCALE → ∞`, what distribution does `scatter_gradient` converge to?

---

## From the Source

**Algorithm:** Four independent scatter strategies sharing `pool_place` and `GridCtx`. Each strategy is O(MAX_OBJ) or O(grid_area) — all practical for real-time interactive use on a terminal.

**Math:** Rejection sampling (M) is a simplified Bridson Poisson-disk algorithm (Bridson 2007, ACM SIGGRAPH). The Bernoulli gradient (G) is a non-uniform sampling without rejection — each cell is an independent Bernoulli trial with success probability P(cell). BFS (F) is classic breadth-first graph traversal (Cormen et al., Introduction to Algorithms §22.2).

**Rendering:** Background → objects → cursor → HUD. The cursor marks the BFS seed point for F — placing the cursor in different grid positions produces floods from different starting cells.

---

## Pass 2 — Implementation

### Pseudocode

```
scatter_random(pool, g, glyph):
    for i = 0..RAND_N:
        r = rand_range(g->min_r, g->max_r)
        c = rand_range(g->min_c, g->max_c)
        pool_place(pool, r, c, glyph)

scatter_mindist(pool, g, glyph):
    for attempt = 0..MAX_TRIES while pool.count < MAX_OBJ:
        r = rand_range(...)
        c = rand_range(...)
        ok = true
        for i = 0..pool.count: if cheb(r,c, items[i]) < MIN_DIST: ok=false
        if ok: pool_place(pool, r, c, glyph)

scatter_flood(pool, g, cr, cc, glyph):
    alloc vis[], qr[], qc[] of size grid_area
    ENQUEUE(cr, cc)
    while queue not empty and placed < FLOOD_MAX:
        r,c = dequeue; pool_place(r,c); placed++
        for d in 4-dirs: ENQUEUE(r+dr, c+dc)
    free(vis, qr, qc)

scatter_gradient(pool, g, glyph):
    cr,cc = grid centre
    for r,c in all valid cells:
        dist = cheb(r,c, cr,cc)
        threshold = (long)GRAD_SCALE * RAND_MAX / (dist + GRAD_SCALE)
        if rand() < threshold: pool_place(pool, r, c, glyph)
```

### Module Map

```
§1 config   — scatter parameters + grid constants
§4 gridctx  — same GridCtx as 01_direct
§5 pool     — ObjectPool (pool_place, pool_draw, pool_clear)
§6 scatter  — scatter_random, scatter_mindist, scatter_flood, scatter_gradient
§7 cursor   — Cursor (BFS seed), cursor_draw
§8 scene    — scene_draw
§9 screen   — screen_init
§10 app     — main loop
```

### Data Flow

```
getch('R/M/F/G') → scatter_*(pool, g, cursor, 'o')
    → pool_place per cell (bounded by MAX_OBJ)
pool_draw → ctx_to_screen → mvaddch
```
