# Concept: Isometric (Solid-Fill) Pattern Stamps

## Pass 1 — Understanding

### Core Idea
Same stamp mechanic as `01_equilateral_patterns` on the iso solid-fill grid. Patterns are unaware of paint — they manipulate addresses (col, row, up). The iso colour wheel decorates the background; stamped glyphs sit on top in `PAIR_CURSOR` (white-on-black).

### Pattern Tables
Identical structure to `01_equilateral_patterns`: sentinel-terminated `(Δcol, Δrow, target_up)` triples for RING / LINE / STAR / TRI / SCATTER.

### Render Pipeline
1. Solid-fill raster (per cell, palette_index hash → bg colour).
2. `pool_draw`: glyph at each placed object's centroid using `PAIR_CURSOR`.
3. `cursor_draw`: '@' on top.

### Why ABSOLUTE target_up
Same reason as `01_equilateral_patterns`: equilateral orientation depends on `(col + row)` parity; absolute `up` keeps the stamp's silhouette invariant under translation.

### Non-Obvious Decisions
- **Theme cycle preserves stamps**: 't' rebuilds palette pairs; stamped objects keep their addresses, so they stay where they were stamped — just with a different colour underneath.
- **Patterns ADD, never REPLACE**: re-stamping at the same cursor is a no-op.
- **MAX_OBJ silent cap**: STAR + repeated SCATTER saturates fast; SPACE clears.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `N_PALETTE` | 6 — iso fill cycle |
| `0xDEAD` | Sentinel for end-of-pattern |
| `N_GLYPHS` | Number of cycle-able glyphs |

### Open Questions
- What if a pattern coloured glyphs by their own palette_index — would it look like a "tagged" object set?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, N_PALETTE, N_GLYPHS
§4 formula  — pixel_to_tri + palette_index + centroid
§5 pool     — ObjectPool: clear, find, add, draw
§6 cursor   — TRI_DIR + step + draw
§7 patterns — PAT_RING/LINE/STAR/TRI tables + pattern_stamp + pattern_scatter
§8 scene    — solid-fill raster + pool + cursor
§9 screen   — ncurses init / cleanup
§10 app     — signals, main loop
```

### Data Flow
```
'1'..'4' → pattern_stamp(PAT_xxx, cur)
'5'      → pattern_scatter(cur, glyph)
SPACE    → pool_clear
't'      → cycle theme → color_init
frame    → raster fill + pool_draw + cursor_draw
```
