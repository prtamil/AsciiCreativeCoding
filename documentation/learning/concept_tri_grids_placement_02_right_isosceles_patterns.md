# Concept: Right-Isosceles Pattern Stamps

## Pass 1 — Understanding

### Core Idea
Static pattern tables of `(Δcol, Δrow, target_up)` triples stamped at the cursor on the half-rect grid. Same approach as `01_equilateral_patterns` but on the axis-aligned UR/LL lattice. Patterns: RING (cursor + neighbours), LINE (8-tri strip), STAR (RING + outer ring), TRI (cursor + 3 corners), SCATTER (random in ±4).

### Pattern Tables
Sentinel-terminated `static const int PAT_xxx[][3]` arrays.

### Why ABSOLUTE target_up
On the half-rect lattice every (col, row) holds BOTH UR and LL — the choice of `up` is a per-entry property, not a parity-dependent flip. Storing `up` absolute keeps stamps invariant under translation.

### SCATTER (5)
LCG-seeded pickups within ±4 cells, with bounded retries to avoid duplicates.

### Pool Toggle (Same As _direct)
`pool_add` deduplicates; `pool_clear` on SPACE.

### Non-Obvious Decisions
- **Patterns ADD, never REPLACE**: re-stamping at the same cursor is a no-op.
- **Glyph cycle**: changing the glyph affects the NEXT stamp; old entries keep their original glyph.
- **MAX_OBJ silent cap**: STAR + repeated SCATTER saturates quickly.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `0xDEAD` | Sentinel for end-of-pattern |
| `N_GLYPHS` | Number of cycle-able glyphs |

### Open Questions
- How would you generate a pattern that respects the diagonal — e.g. only UR triangles?
- Can you build a "fill-bucket" tool that floods all triangles inside a closed boundary?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, N_GLYPHS
§4 formula  — pixel_to_tri (axis-aligned), tri_centroid_pixel
§5 pool     — ObjectPool: clear, find, add, draw
§6 cursor   — TRI_DIR + step + draw
§7 patterns — PAT_RING/LINE/STAR/TRI tables + pattern_stamp + pattern_scatter
§8 scene    — grid_draw + scene_draw
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
'1'..'4' → pattern_stamp(PAT_xxx, cur)
'5'      → pattern_scatter(cur, glyph)
SPACE    → pool_clear
frame    → grid_draw + pool_draw + cursor_draw
```
