# Concept: Rhombille Tiling

## Pass 1 — Understanding

### Core Idea
Projects the three visible faces of an isometric cube onto the 2D plane. Each face becomes a rhombus (diamond). Three distinct face orientations tile the plane without gaps or overlaps, producing the classic "stacked cubes" optical illusion.

### Three Face Types
When looking at a cube from the (1,1,1) direction:
- **Top face**: rhombus pointing upward
- **Left face**: rhombus pointing lower-left
- **Right face**: rhombus pointing lower-right

Each rhombus has one long diagonal (vertical for top, ±60° for sides) and one short diagonal perpendicular to it.

### Relationship to Hex Grid
The rhombille tiling is derived from the triangular lattice — each triangle is split into 3 rhombuses meeting at the triangle centroid. The hex grid dual (triangular) dual (rhombille) chain shows all three tilings are related.

### Rendering
Each rhombus is rendered with directional characters (`/`, `\`, `|`, `_`) matching the edge orientation. The three face types are colour-coded to reinforce the 3D illusion.

### Non-Obvious Decisions
- The `√3:1` long:short diagonal ratio of the rhombus matches terminal cell aspect ratio closely, making rhombille look correct without special scaling.
- Colouring the three face types with light/dark/medium shades creates a convincing 3D cube illusion.

### Open Questions
- How does the rhombille tiling relate to the body-centred cubic crystal structure?
- Can you animate the tiling to show "cubes appearing and disappearing"?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — face colours (3 pairs), cell spacing
§4 coords  — rhombus centre computation, face type classification
§5 grid    — grid_draw(): classify each screen position into face type, draw border char
§8 app     — main loop
```
