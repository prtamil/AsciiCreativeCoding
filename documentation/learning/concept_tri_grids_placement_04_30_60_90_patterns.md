# Concept: Kisrhombille Pattern Stamps

## Pass 1 — Understanding

### Core Idea
Static `(Δcol, Δrow, target_up)` tables stamped at the cursor on the kisrhombille grid. Stamps address WHOLE equilaterals; the median overlay renders independently in `grid_draw` and forms a backdrop for stamped glyphs at parent centroids.

### Same As 01_equilateral_patterns
RING / LINE / STAR / TRI / SCATTER tables; sentinel-terminated; absolute `target_up`. Pattern math is identical to `01_equilateral_patterns`.

### Visual Twist
Each stamped triangle still shows its 3 medians beneath the glyph. Patterns ADD to the pool; they do not REPLACE.

### Non-Obvious Decisions
- **Patterns unaware of medians**: the patterns manipulate addresses; the medians are part of `grid_draw`. Adding a new pattern requires no median-aware logic.
- **Glyph at centroid covers concurrent point**: same as `04_30_60_90_direct`.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `BORDER_W` | Edge-proximity threshold |
| `MEDIAN_T` | Median-proximity threshold |

### Open Questions
- How would you make a pattern that EXPLICITLY uses median geometry — e.g. stamp 6 sub-triangles inside the cursor's parent?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, BORDER_W, MEDIAN_T, N_GLYPHS
§4 formula  — pixel ↔ skew lattice + centroid + median proximity
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
frame    → grid_draw (edges + medians) + pool_draw + cursor_draw
```
