# Concept: Double-Diagonal (Tetrakis) Pattern Stamps

## Pass 1 — Understanding

### Core Idea
Static `(ΔQ, ΔR, target_dir)` tables stamped at the cursor on the tetrakis grid. Patterns: RING (4 wedges of cursor's square — full pinwheel), LINE (horizontal strip), STAR (RING + outer hexes' wedges), CROSS (cursor + 4 cardinal neighbour wedges), SCATTER (random within ±4).

### Why ABSOLUTE target_dir
Each `(col, row)` square holds all 4 wedges. The choice of dir is per-entry — not parity-dependent — so storing `dir` as an absolute integer in 0..3 keeps the stamp's silhouette invariant under translation.

### RING Pattern Property
RING covers the cursor's full square (4 entries). Restamping at the same cursor is a no-op (pool_add deduplicates).

### SCATTER (5)
LCG-seeded; picks `(dC, dR)` in ±4 and a random `dir ∈ 0..3`. Bounded retries handle dedup collisions.

### Non-Obvious Decisions
- **Pattern silhouette under cursor sector rotation**: the stamp's shape is fixed in pattern data — rotating the cursor's `dir` before stamping has NO effect on the resulting stamp. Intentional.
- **Sentinel-terminated tables**: same `0xDEAD` magic value as `01_equilateral_patterns.c`.
- **MAX_OBJ silent cap**: STAR (12+ entries) plus repeated SCATTER saturates quickly.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `0xDEAD` | Sentinel for end-of-pattern |
| `N_GLYPHS` | Number of cycle-able glyphs |

### Open Questions
- How would you build a pattern that's a rotation of RING by 45° (a square instead of a pinwheel)?
- What pattern gives the dual tiling (4.8.8 truncated square) when stamped uniformly?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, N_GLYPHS
§4 formula  — wedge classifier + wedge centroids
§5 pool     — ObjectPool{col, row, dir, glyph}: clear, find, add, draw
§6 cursor   — TETRA_DIR + step + draw
§7 patterns — PAT_RING/LINE/STAR/CROSS tables + pattern_stamp + pattern_scatter
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
