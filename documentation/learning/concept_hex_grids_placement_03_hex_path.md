# Concept: Hex Path Drawing

## Pass 1 — Understanding

### Core Idea
Draws paths between two hex endpoints (A and B) in axial space. Three path types: straight line (hex_lerp_round), ring (6×N steps), and L-path (Q-leg then R-leg). `a` marks endpoint A; `b` marks endpoint B; `space` stamps the path.

### Path Types

**LINE** — `hex_lerp_round(aQ,aR, bQ,bR, t)` with t ∈ [0,1] at N+1 steps:
```
fq = aQ*(1-t) + bQ*t
fr = aR*(1-t) + bR*t
fs = aS*(1-t) + bS*t
(Q,R) = cube_round(fq, fr, fs)
```
Linear interpolation in cube space always produces a straight geodesic.

**RING** — Standard ring walk: start at `centre + N×HEX6[4]`, then 6 legs of N steps each:
```
for i in 0..5: for j in 0..N: walk += HEX6[i]
```
Produces exactly 6N cells.

**L-PATH** — Walk |dQ| steps along Q then |dR| steps along R. Produces a path with exactly one right-angle bend.

### Endpoint UX
- `b` not pressed: cursor acts as endpoint B (live preview as cursor moves).
- `b` pressed: endpoint B is fixed; cursor movement no longer changes the path.
- This design provides "implicit B = cursor" for quick demos and explicit B for precise paths.

### hex_lerp_round
Ordinary `round()` after linear interpolation breaks the Q+R+S=0 constraint. `cube_round` fixes the largest-error component — this is the same invariant repair as in `cube_round()` for the inverse matrix.

### Non-Obvious Decisions
- Green-dot preview (one `'.'` per path hex) is visible because it's a sparse overlay on top of the grid, unlike the pattern overlay which needed full border rendering.
- The ring preview can be computed cheaply by re-running the ring walk in the preview function.

### Open Questions
- What is the hex-space analog of Bresenham's line algorithm?
- Can hex_lerp_round produce duplicate cells? (Yes, at degenerate angles — check and deduplicate.)

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — PATH_MAX_CELLS, HEX6[6][2], colour pairs (ENDPT_A/B, PREVIEW)
§4 coords  — cube_round(), hex_to_screen(), hex_lerp_round(), hex_dist()
§5 path    — PathMode enum, path_compute(), path_preview_draw()
§6 scene   — scene_draw(): grid → preview → pool → markers → cursor → HUD
§8 app     — a:setA, b:setB, l/o/j:mode, spc:stamp, C:clear
```

### Data Flow
```
'a' → has_a=1, aQ=cQ, aR=cR
'b' → has_b=1, bQ=cQ, bR=cR
frame → path_preview_draw(mode, aQ,aR, effective_bQ,bR)
space → pool_place for each path cell
```
