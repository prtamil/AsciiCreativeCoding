# Concept: Pattern Stamp Placement (02_patterns.c)

## Pass 1 — Understanding

### Core Idea
Replace single-cell toggle with multi-cell stamp patterns. A pattern is a **predicate** — a function `pat_test(pat, dr, dc, N)` that returns true if offset `(dr, dc)` from an anchor cell belongs to the pattern. The predicate drives both the live preview and the actual stamp with the same code.

### Mental Model
When you press a stamp key (`B/F/H/R/V`), the program iterates a bounding box of offsets `(-N..N, -N..N)`, asks the predicate "should this offset be included?", and places an object for every offset that answers yes. The cursor shows the same bounding box evaluated live every frame — the user sees the footprint before committing.

### Key Equations

```
border(dr, dc, N):  (|dr|==N || |dc|==N) && |dr|<=N && |dc|<=N
fill(dr, dc, N):    |dr|<=N && |dc|<=N
hollow(dr, dc, N):  fill(N) && !fill(N-1)        // outermost ring
row(dr, dc, N):     dr==0 && |dc|<=N
col(dr, dc, N):     dc==0 && |dr|<=N
```

### Data Structures

Same `GridCtx` + `ObjectPool` as `01_direct.c`. Added:
- `PatMode` enum: `{PAT_BORDER, PAT_FILL, PAT_HOLLOW, PAT_ROW, PAT_COL}`
- `int N` — pattern half-size, range `[MIN_PAT_N=1, MAX_PAT_N=8]`
- The pattern predicate `pat_test(PatMode, int dr, int dc, int N)` is a pure function — no state.

### Non-Obvious Decisions

- **Predicates vs bitmaps**: A predicate scales with `N` (one parameter changes the entire size); a bitmap would need a different array per size. Predicates also compose: `hollow = fill(N) AND NOT fill(N-1)` without special-casing.
- **Same predicate for preview and stamp**: `preview_draw` and `pattern_stamp` both call `pat_test` over the same offset range. No divergence between what the user sees and what gets placed.
- **Preview uses `A_REVERSE`**: Highlights cells in the cursor colour with reversed foreground/background so the pattern stands out against the grid background without occluding it.
- **Out-of-range cells silently skipped**: Both `pattern_stamp` and `preview_draw` check `r >= g->min_r && r <= g->max_r` etc. — cells outside the grid just don't get placed, no error.
- **`pool_remove` is absent**: Single-cell removal (from 01_direct) isn't needed here — stamping only places, clear (`C`) wipes all. Removing that function eliminated a compiler warning.

### Key Constants

| Name | Role |
|------|------|
| `MIN_PAT_N` | 1 — smallest pattern half-size |
| `MAX_PAT_N` | 8 — largest; gives 17×17 = 289-cell stamp max |
| `PAT_HOLLOW` | = `fill(N) && !fill(N-1)` — only outer ring |

### Open Questions

- What does a `PAT_BORDER` with `N=3` look like on an isometric grid? On a brick grid?
- Can you implement a diagonal strip pattern: `pat_test → |dr - dc| <= 1`?
- What is the maximum number of objects `PAT_FILL` can place with `N=8`? Is 512 (MAX_OBJ) enough?

---

## From the Source

**Algorithm:** Pattern-fill placement. Patterns as predicates rather than bitmaps. The predicate returns a boolean for every `(dr, dc)` offset from the anchor; stamping calls `pool_place` for every true cell.

**Math:** Each pattern predicate is a geometric membership test over integer coordinates. `hollow` is a Boolean difference: the set-minus of two filled squares. `border` is the outer boundary of a filled square.

**Rendering:** Background → objects (`pool_draw`) → preview overlay (`preview_draw` with `A_REVERSE`) → cursor (`cursor_draw`) → HUD. The preview is drawn every frame but only shows the current cursor position — no persistence until stamped.

---

## Pass 2 — Implementation

### Pseudocode

```
pat_test(pat, dr, dc, N):
    ar = |dr|; ac = |dc|
    switch pat:
        BORDER: return (ar==N || ac==N) && ar<=N && ac<=N
        FILL:   return ar<=N && ac<=N
        HOLLOW: return fill(N) && !fill(N-1)
        ROW:    return dr==0 && ac<=N
        COL:    return dc==0 && ar<=N

pattern_stamp(pool, g, cr, cc, pat, N, glyph):
    for dr = -N..N:
        for dc = -N..N:
            if pat_test(pat, dr, dc, N) and in_bounds(g, cr+dr, cc+dc):
                pool_place(pool, cr+dr, cc+dc, glyph)

preview_draw(g, cr, cc, pat, N):
    attron(PAIR_CURSOR | A_REVERSE)
    for dr = -N..N:
        for dc = -N..N:
            if pat_test(pat, dr, dc, N) and in_bounds(g, cr+dr, cc+dc):
                ctx_to_screen(g, cr+dr, cc+dc, &sr, &sc)
                mvaddch(sr, sc, '.')
    attroff(...)
```

### Module Map

```
§1 config   — grid constants, pattern range
§4 gridctx  — same as 01_direct
§5 pool     — ObjectPool (pool_place only; no toggle/remove)
§6 patterns — pat_test, pattern_stamp, preview_draw
§7 cursor   — Cursor, cursor_draw
§8 scene    — scene_draw
§9 screen   — screen_init
§10 app     — main loop + key handler
```

### Data Flow

```
getch() → select PatMode / change N / stamp
pat_test(pat, dr, dc, N) → bool → pool_place / preview_draw
ctx_to_screen → (sr, sc) → mvaddch
```
