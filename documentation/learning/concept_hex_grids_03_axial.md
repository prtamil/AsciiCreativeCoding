# Concept: Axial Coordinate Display

## Pass 1 — Understanding

### Core Idea
Visualises the three coordinate axes (Q=0, R=0, S=0) of the hex grid and colour-codes every cell by its ring distance from the origin. The cube constraint Q+R+S=0 means these three axes are not independent — the S axis is always diagonal and passes through the same origin as Q and R.

### The Cube Constraint
Hex grids live in a 2D plane embedded in 3D cube space. The plane equation is Q+R+S=0. This means:
- Moving in the +Q direction also moves in the −S direction.
- The three axis lines are at 120° to each other.
- Ring distance from origin = max(|Q|, |R|, |S|) = (|Q|+|R|+|S|)/2.

### Axis Colouring Rules
- Q-axis: all hexes where R==0 (and therefore S==−Q)
- R-axis: all hexes where Q==0
- S-axis: all hexes where Q+R==0 (i.e. S==0)
- Off-axis: coloured by ring distance (hex_dist bands)

### Non-Obvious Decisions
- Drawing axis hexes with a special character/colour makes the coordinate system legible at a glance. Without it, axial coordinates feel abstract.
- Ring-distance colouring is achieved at render time by evaluating `hex_dist(Q,R,0,0)` for every hex — no pre-computed state.

### Open Questions
- Is there a fourth axis that could be drawn? (No — only three independent directions in a hex grid.)
- How does the ring-distance colouring relate to the six "sectors" of the hex grid?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — axis colours, ring band colours, HEX_SIZE
§4 coords  — cube_round(), hex_to_screen(), hex_dist()
§5 grid    — grid_draw(): per-hex colour assignment, axis detection
§8 app     — main loop
```

### Data Flow
```
(row,col) → cube_round → (Q,R,S) → axis test or hex_dist → colour pair → mvaddch
```
