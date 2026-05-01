# Concept: Hex-Subdivision Pattern Stamps

## Pass 1 — Understanding

### Core Idea
Static `(ΔQ, ΔR, target_sector)` tables stamped at the cursor on the hex-subdivision grid. Patterns: RING (6 wedges of cursor's hex — full pinwheel), LINE (wedges along a horizontal hex strip), STAR (RING + outer ring of neighbouring hexes), TRI (cursor + 3 alternating sectors), SCATTER (random within ±radius).

### Why ABSOLUTE target_sector
Sectors are global angles measured from +x at every hex centre. The same sector ID in two different hexes points the same compass direction; a delta would have no consistent meaning across hex translations.

### RING Pattern Property
RING covers all 6 sectors of the cursor's hex (a full pinwheel). Restamping at the same hex is a no-op (`pool_add` deduplicates).

### SCATTER (5)
LCG-seeded; picks random `(ΔQ, ΔR)` in a small radius and a random `sector ∈ 0..5`. Bounded retries handle dedup collisions.

### Pool (Same As _direct)
`pool_clear` on SPACE; `pool_add` deduplicates.

### Non-Obvious Decisions
- **Pattern silhouette under cursor sector rotation**: rotating `cur->sector` before stamping has NO effect on the stamp's shape — the stamp's sectors are absolute, not relative to the cursor. Intentional.
- **Sentinel-terminated tables**: same `0xDEAD` as other patterns files.
- **MAX_OBJ silent cap**: STAR (12+) plus repeated SCATTER saturates fast; SPACE clears.

### Key Constants
| Name | Role |
|------|------|
| `HEX_SIZE` | Hex circumradius (= edge length) |
| `MAX_OBJ` | Pool capacity |
| `0xDEAD` | Sentinel for end-of-pattern |
| `N_GLYPHS` | Number of cycle-able glyphs |

### Open Questions
- How would you stamp a HEX-aligned stamp (rotated 60°) — would the sector indices need to rotate too?
- Could you generate a SPIRAL pattern using sector cycling?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — HEX_SIZE, MAX_OBJ, N_GLYPHS
§4 formula  — pixel_to_hex + wedge_centroid_pixel
§5 pool     — ObjectPool{Q, R, sector, glyph}: clear, find, add, draw
§6 cursor   — HEX_DIR + step_hex + rotate_sector
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
