# Concept: Triangular-Dual Grid

## Pass 1 — Understanding

### Core Idea
The triangular lattice is the dual of the hexagonal lattice: hex centres become triangle vertices, and vice versa. In screen space, up-triangles (▲) and down-triangles (▽) alternate based on cell parity `(row+col)%2`.

### Duality
Every regular tessellation has a dual tessellation:
- Dual of hexagonal → triangular
- Dual of triangular → hexagonal
- Dual of square → square (self-dual)

The triangular tessellation is one of the three regular tessellations (along with square and hexagonal). It has the highest vertex valence (6 triangles meet at each vertex) and the smallest faces.

### Parity Rule
For screen coordinates `(sr, sc)` mapped to triangle cell `(tr, tc)`:
```
parity = (tr + tc) % 2
parity == 0 → up-triangle   (▲, apex at top)
parity == 1 → down-triangle (▽, apex at bottom)
```

### Triangle Centroids
Up-triangle at (tr,tc):   cx = tc×CW + CW/2,  cy = tr×CH + CH/3
Down-triangle at (tr,tc): cx = tc×CW,          cy = tr×CH + 2×CH/3

### Border Characters
- Up-triangle (▲): top edge `_`, left edge `/`, right edge `\`
- Down-triangle (▽): bottom edge `_`, left edge `\`, right edge `/`

### 3-Connectivity
Each triangle has exactly 3 neighbours (shared-edge adjacency), compared to 6 for hex and 4 for rect. This is the minimum connectivity for a complete 2D tessellation.

### Non-Obvious Decisions
- Terminal aspect ratio (cells are ~2× taller than wide in pixels) means equilateral triangles require `CW ≈ 2×CH` to look proportional.
- The dual relationship means you can navigate triangular grids using hex grid coordinates with a simple transform.

### Open Questions
- What is the 3-neighbour adjacency list for an up-triangle at (tr,tc)?
- How does pathfinding on a triangular grid differ from hexagonal?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — CW, CH, triangle aspect ratio constants
§4 coords  — tri_to_screen(), screen_to_tri(), parity()
§5 grid    — grid_draw(): parity determines up/down glyph set
§8 app     — main loop
```

### Data Flow
```
(row,col) → (tr,tc) = (row/CH, col/CW) → parity → border glyph selection → mvaddch
```
