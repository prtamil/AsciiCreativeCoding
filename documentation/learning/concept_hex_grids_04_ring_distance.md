# Concept: Ring-Distance Colouring

## Pass 1 — Understanding

### Core Idea
Every hex is coloured by its axial distance from the cursor. Moving the cursor recolours the entire grid live, demonstrating that hex distance forms perfect concentric rings — the hex analog of Euclidean circles.

### Distance Formula
```
hex_dist(aQ, aR, bQ, bR) = (|dQ| + |dR| + |dQ+dR|) / 2
```
where `dQ = bQ-aQ`, `dR = bR-aR`. This is the L1 (Manhattan) distance in cube space projected to axial coordinates.

### Why Not Euclidean Distance?
Euclidean distance would produce circles that don't align with hex cell boundaries. The axial distance metric produces rings that are always exactly N cells from the centre — a "hex circle" with exactly 6N cells in its Nth ring.

### Ring Properties
- Ring 0: 1 cell (the centre)
- Ring 1: 6 cells
- Ring N: 6N cells
- Total cells within ring N: 1 + 3N(N+1) = 3N² + 3N + 1

### Non-Obvious Decisions
- Recolour on every frame (not cached) — O(rows×cols) per frame is fast enough for a 30fps terminal.
- The `6N` ring size can be verified interactively by counting cells at a given colour band.

### Open Questions
- How many distinct colour bands can you fit in a terminal-sized hex grid?
- What is the maximum hex_dist from the grid origin to any visible hex?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — N_COLORS, HEX_SIZE, BAND_COLORS[]
§4 coords  — cube_round(), hex_to_screen(), hex_dist(), cursor movement
§5 grid    — grid_draw(): colour = hex_dist % N_COLORS
§8 app     — arrow keys move cursor; grid redraws immediately
```
