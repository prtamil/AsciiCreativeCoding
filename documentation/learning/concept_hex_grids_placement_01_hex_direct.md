# Concept: Hex Direct Placement

## Pass 1 — Understanding

### Core Idea
Interactive cursor placement on a flat-top hex grid. The cursor lives in axial (Q,R) space; arrow keys step one hex at a time using `HEX_DIR[4]`. `space` toggles a placed object at the cursor hex. This is the hex equivalent of `rect_grids_placement/01_direct.c`.

### Cursor Movement
Four navigable directions for a flat-top grid (matching arrow key intent):
```
HEX_DIR[0] = (+1,  0)  RIGHT
HEX_DIR[1] = (-1,  0)  LEFT
HEX_DIR[2] = ( 0, -1)  upper-right (R decreases → up on screen)
HEX_DIR[3] = ( 0, +1)  lower-left
```
A full 6-direction ring would use `HEX6[6][2]`. The 4-direction subset aligns with arrow keys on a flat-top layout.

### Object Pool
- Fixed-size pool of `MAX_OBJ` placed objects.
- Toggle: if the cursor hex is occupied, remove it; if not, add it.
- Removal uses swap-last: copy the last pool entry over the removed entry, decrement count. O(1) removal without shifting.

### Hit Test
`cube_round(fq, fr, fs)` from the cursor screen position gives the exact hex (Q,R) under the cursor. Comparing with stored objects: `Q==pool[i].Q && R==pool[i].R`.

### Non-Obvious Decisions
- **Cursor drawn with truncation**: `hex_to_screen` uses `round()` for rendering, but `(int)` truncation for the cursor marker to avoid a 1-cell shift on borderline positions.
- **Object rendering uses the same `angle_char` border rasterizer** as the grid, but with a distinct colour pair, so placed objects blend visually with the grid texture.

### Open Questions
- What happens to the cursor position when `+/-` changes HEX_SIZE?
- How would you add multi-select (select a region, then move it)?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — HEX_SIZE, MAX_OBJ, colour pairs (GRID/CURSOR/OBJ/HUD)
§4 coords  — cube_round(), hex_to_screen(), angle_char(), HEX_DIR[4]
§5 pool    — HPool struct, pool_toggle(), pool_draw()
§6 scene   — scene_draw(): grid → objects → cursor → HUD
§8 app     — arrow keys, space, +/-, r, q/ESC
```

### Data Flow
```
arrow key → cursor (Q,R) += HEX_DIR[k]
space     → pool_toggle(cursor Q,R)
frame     → grid_draw() + pool_draw() + cursor_draw()
```
