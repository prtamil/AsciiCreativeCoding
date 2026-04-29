# Concept: Hex Scatter Placement

## Pass 1 — Understanding

### Core Idea
Four procedural scatter strategies place objects on a hex disc of radius R around the cursor. All strategies operate in axial (Q,R) space, not screen space — the hex metric is used throughout.

### Scatter Strategies

**1 — Uniform**: For each hex in the disc, include it with probability = `density`. Simple Bernoulli trial per cell.

**2 — Min-dist (hex Poisson-disk)**: Accept only if `hex_dist(new, existing) >= mindist` for all existing objects. Rejection sampling in the integer hex metric — no square root needed.

**3 — Flood-fill disc**: BFS from cursor centre, accept all cells within radius R. Deterministic fill of the exact disc — useful for seeing what area the radius covers.

**4 — Gradient density**: `P = FALLOFF/(d+FALLOFF)` where d = `hex_dist(cursor, hex)`. Dense at centre, sparse at edge. Sigmoid-like falloff controlled by the `FALLOFF` constant.

### Disc Enumeration
To iterate over all hexes within ring distance R:
```
for Q in -R..+R:
    for R in max(-R,-Q-R)..min(+R,-Q+R):
        if hex_dist(Q,R,0,0) <= R: process(cQ+Q, cR+R)
```
This is the standard axial-range iteration — no circular approximation needed.

### Non-Obvious Decisions
- Min-dist scatter is O(N²) per scatter event (check against all existing objects). Acceptable because scatter is a user-triggered event, not per-frame.
- The gradient falloff constant `FALLOFF=3` was chosen so the 50% acceptance probability occurs at hex distance 3, giving a visible gradient without being too sparse.

### Open Questions
- How would you implement a stratified (jittered grid) scatter in hex space?
- What is the expected density of min-dist scatter as a fraction of the disc area?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — SCATTER_RADIUS, MINDIST_DEFAULT, DENSITY_DEFAULT, GRAD_FALLOFF
§4 coords  — cube_round(), hex_to_screen(), hex_dist()
§5 scatter — ScatterMode enum, scatter_uniform(), scatter_mindist(), scatter_flood(), scatter_gradient()
§6 scene   — scene_draw(): grid → pool → cursor → HUD (mode/radius/density shown)
§8 app     — 1-4:mode, +/-:radius, d/D:density, m/M:mindist, spc:scatter, C:clear
```

### Data Flow
```
space → scatter_fn(cursor Q,R, radius, density/mindist) → pool_place for accepted hexes
frame → draw grid + pool + cursor
```
