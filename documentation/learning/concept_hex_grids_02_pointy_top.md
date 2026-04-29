# Concept: Pointy-Top Hexagonal Grid

## Pass 1 — Understanding

### Core Idea
Pointy-top is flat-top rotated 30°. The forward matrix swaps the roles of Q and R scaling, and the rows-of-hexes arrangement becomes columns-of-hexes. The `cube_round` kernel and `angle_char` function are identical — only the matrix entries differ.

### Forward Matrix (pointy-top orientation)
```
cx = size × (√3 × Q  +  √3/2 × R)
cy = size × 3/2 × R
```
Compare with flat-top: the `3/2` factor moves from Q to R, and the √3 terms swap roles.

### When to Choose Pointy-Top vs Flat-Top
- **Flat-top**: rows of hexes march horizontally; natural for maps where "east/west" is the primary axis.
- **Pointy-top**: columns of hexes march vertically; natural for grids aligned with screen rows (better aspect-ratio match on many terminals).

### Non-Obvious Decisions
- The four navigable arrow-key directions differ between orientations. In flat-top, left/right are pure Q-axis moves. In pointy-top, up/down are pure R-axis moves.
- Rotating the grid 30° changes which HEX_DIR vectors align with the cursor movement keys.

### Open Questions
- How many distinct hex orientations are there? (Answer: just two — all others are a permutation of axial labels.)
- What is the screen aspect ratio at which flat-top and pointy-top produce equal-looking hexes?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — HEX_SIZE, BORDER_W, orientation constant
§4 coords  — cube_round() (shared), pointy_to_screen(), angle_char()
§5 grid    — grid_draw(): same O(rows×cols) raster, different forward matrix
§8 app     — main loop, same as 01_flat_top
```

### Key Difference from 01_flat_top
```c
/* flat-top */   cx = size * 1.5 * Q;   cy = size * (SQ3_2*Q + SQ3*R);
/* pointy-top */ cx = size * (SQ3*Q + SQ3_2*R);   cy = size * 1.5 * R;
```
One substitution. Everything else is identical.
