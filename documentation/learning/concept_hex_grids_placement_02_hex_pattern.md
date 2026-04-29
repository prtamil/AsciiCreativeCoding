# Concept: Hex Pattern Stamp

## Pass 1 — Understanding

### Core Idea
Stamps a pattern of objects onto the hex grid at the cursor position. Four pattern shapes are defined as Boolean predicates on `(dQ, dR)` — the axial offset from the cursor. A per-pixel overlay preview (O(rows×cols)) shows the pattern in bright green before stamping.

### Pattern Predicates
```
DISC: hex_dist(dQ, dR) <= N
RING: hex_dist(dQ, dR) == N
ROW:  dR == 0 && |dQ| <= N
COL:  dQ == 0 && |dR| <= N
```
All predicates are purely mathematical functions of the axial offset — no spatial data structures needed.

### Preview Overlay Design
The preview runs the **full O(rows×cols) rasterizer** (same as the grid draw), but filtered by `pat_test()`. This means:
- Every screen pixel that belongs to a pattern hex shows a full hex border in bright green.
- The preview is visually identical in style to the placed objects.
- Sparse "dot at centre" approaches were rejected because they were invisible on dark terminal backgrounds.

### Stamp Operation
`pool_place(Q, R)` adds a hex to the pool if not already present (deduplication). No duplicates — stamping the same pattern twice is idempotent.

### Glyph Selection
Placed objects use `PAT_GLYPH[mode]` to select a centre character distinct from hex border glyphs:
- DISC: `*`
- RING: `o`
- ROW: `=` (not `-` which is a border char)
- COL: `:` (not `|` which is a border char)

### Non-Obvious Decisions
- Preview rasterizer was the key fix for visibility — border chars at cell centres are invisible against the grid.
- `PAIR_PREVIEW` uses colour 82 (bright green), not 245 (dim grey), for the same reason.

### Open Questions
- Can you preview a pattern on a ring pattern? (What does DISC on top of RING look like?)
- How would you implement a "subtract pattern" (erase) operation?

---

## Pass 2 — Implementation

### Module Map
```
§1 config  — PAT_N_MAX, PAT_GLYPH[4], colour pairs (PREVIEW=82)
§4 coords  — cube_round(), angle_char(), hex_to_screen()
§5 pattern — PatMode enum, pat_test(), pat_overlay(), pool_place()
§6 scene   — scene_draw(): grid → pat_overlay (if preview) → pool → cursor → HUD
§8 app     — m:mode p/P:preview n/N:radius spc:stamp q/ESC
```

### Data Flow
```
'n'/'N' → adjust N
'm'     → cycle PatMode
'p'     → toggle show_preview
frame   → pat_overlay(mode, N, cQ, cR) → green border pixels in predicate hexes
space   → pool_place for each predicate hex within range
```
