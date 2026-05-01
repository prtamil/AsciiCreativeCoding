# Concept: Equilateral Pattern Stamps

## Pass 1 — Understanding

### Core Idea
Each pattern (RING, LINE, STAR, TRI, SCATTER) is a STATIC array of `(Δcol, Δrow, target_up)` triples relative to the cursor. Pressing `1..5` stamps the pattern by translating the array and inserting each entry into the pool. SCATTER generates a random stamp at each press.

### Pattern Tables (Read-Only)
Stored as `static const int PAT_xxx[][3]` arrays terminated by a sentinel `{0xDEAD, 0, 0}`:
```
PAT_RING  — 6 entries: cursor + 5 neighbours
PAT_LINE  — 8 entries forming a horizontal strip
PAT_STAR  — RING + 6 outer entries
PAT_TRI   — cursor + 3 corners (triforce shape)
```

### Stamp Translation
```
pattern_stamp(pool, PAT_xxx, cursor.col, cursor.row, glyph):
    for each entry (Δc, Δr, up_abs):
        pool_add(pool, cursor.col + Δc, cursor.row + Δr, up_abs, glyph)
```
`pool_add` deduplicates — restamping the same pattern at the same cursor is idempotent.

### Why ABSOLUTE target_up
On the equilateral lattice, triangle orientation depends on `(col + row) parity`. Storing `target_up` as a delta would flip the stamp's silhouette every other position. Storing absolute orientations (0 = ▽, 1 = △) keeps the stamp's shape invariant under translation.

### SCATTER (5)
A run-time random stamp:
```
g_seed ^= clock_ns()
n = 10, tries = 0
while n > 0 and tries < 100:
    dC = floor(frand · 9) − 4       // ±4 range
    dR = floor(frand · 9) − 4
    up = (frand > 0.5) ? △ : ▽
    if pool_add succeeded:  n -= 1
    tries += 1
```
Up to 10 unique entries; bounded retries handle dedup collisions.

### Non-Obvious Decisions
- **Sentinel-terminated tables**: avoids passing entry count separately. Macro `IS_END(p)` checks the magic value.
- **Stamps ADD, never REPLACE**: pressing the same key at the same cursor twice is a no-op; pressing at different cursors leaves both stamps in place. SPACE clears.
- **LCG simple, not crypto**: `g_seed = g_seed · 1103515245 + 12345` — Numerical Recipes ch. 7. Sufficient for visual scatter.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity |
| `0xDEAD` | Sentinel value for end-of-pattern |

### Open Questions
- How would you store rotational variants of each pattern (rotate RING by 60°)?
- What pattern would make the iso-cube illusion pop?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, N_GLYPHS
§4 formula  — pixel_to_tri, tri_centroid_pixel
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
