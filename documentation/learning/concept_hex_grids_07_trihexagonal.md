# Concept: Trihexagonal (Kagome) Tiling

## Pass 1 — Understanding

### Core Idea
One of the 11 Archimedean tilings, with vertex figure `3.6.3.6` — each vertex is surrounded by alternating triangles and hexagons. Also known as the Kagome lattice in condensed-matter physics (frustrated magnetism, spin ice). Derived by placing a triangle in the centre of alternating hexagonal cells.

### Construction from Hex Grid
1. Start with a flat-top hex grid.
2. For every other hex (checkerboard pattern), draw an inscribed triangle.
3. The result has hexagons (the un-triangulated hexes) and triangles (the inscribed ones).

### Vertex Figure
At each vertex, the cycle of faces is triangle → hexagon → triangle → hexagon (3.6.3.6). All vertices are equivalent (vertex-transitive), which is the defining property of Archimedean tilings.

### Physics Connection
The Kagome lattice appears in frustrated antiferromagnets and photonic crystals. The flat-band topology of the Kagome lattice's tight-binding model is actively studied in condensed matter.

### Rendering
Two cell types require different border characters:
- Hexagonal cells: `|`, `/`, `\` as in the base hex grid.
- Triangular insertions: `_`, `/`, `\` following the triangle's edge orientation.

### Non-Obvious Decisions
- The `3.6.3.6` vertex figure means triangles and hexagons alternate without any `3.3.3.3.3.3` (pure triangular) or `6.6.6` (pure hexagonal) regions.
- The checkerboard selection of "which hexes get triangles" can be done with `(Q+R)%2` in axial space.

### Open Questions
- What is the connectivity of the Kagome lattice (as a graph)?
- How does the trihexagonal tiling relate to the snub hexagonal tiling?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — hex size, triangle insert scale, colour pairs (hex/tri)
§4 coords  — cube_round(), hex_to_screen(), triangle centre computation
§5 grid    — grid_draw(): hex scan + conditional triangle overlay
§8 app     — main loop
```

### Data Flow
```
(row,col) → cube_round → (Q,R) → hex border draw
                        → if (Q+R)%2==0: triangle inscribe → triangle border draw
```
