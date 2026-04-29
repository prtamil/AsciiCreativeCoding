# Concept: Direct Cursor Placement (01_direct.c)

## Pass 1 — Understanding

### Core Idea
An interactive editor that lets you navigate a cursor and toggle objects on any of 14 grid types. The key design decision is the `GridCtx` abstraction: one struct and one function (`ctx_to_screen`) that converts grid coordinates to screen coordinates for all 14 modes. A single arrow-key handler works for every grid type without special-casing.

### Mental Model
The program maintains two coordinate systems:
- **Grid coordinates** `(r, c)`: integer cell address, meaningful to the simulation. Negative for diamond/iso grids (they are centred on screen).
- **Screen coordinates** `(sr, sc)`: terminal row/column where the character is actually drawn.

`ctx_to_screen` is the only place that knows the mapping between them. Everything else — cursor draw, object draw, HUD — calls `ctx_to_screen` and never computes screen positions directly.

### Key Equations

```
// All non-rotated grids (uniform, square, brick, checkerboard, etc.):
sr = r * ch
sc = c * cw

// Brick horizontal (row-staggered):
sr = r * ch
sc = c * cw + (r % 2) * (cw / 2)

// Brick vertical (col-staggered):
sr = r * ch + (c % 2) * (ch / 2)
sc = c * cw

// Diamond / Isometric (rotated):
sc = ox + (c - r) * IW
sr = oy + (c + r) * IH
```

### Data Structures

**`GridCtx`**: `{mode, rows, cols, cw, ch, ox, oy, range, min_r, max_r, min_c, max_c}`
Configured once by `ctx_init()`. `min_r/max_r/min_c/max_c` give the valid cell range — used for cursor clamping and pool iteration.

**`ObjectPool`**: Flat `Obj items[MAX_OBJ]` with `int count`. `pool_find` is O(n) linear scan. `pool_toggle` places or removes. Swap-last removal keeps the array compact.

**`Cursor`**: Just `{int r, c}`. Moves by ±1 in grid coordinates, clamped to `[min, max]`.

### Non-Obvious Decisions

- **`ctx_to_screen` as the single seam**: All object rendering, cursor rendering, and HUD markers call `ctx_to_screen`. This means fixing a brick-offset bug in one place corrects cursor, objects, and endpoints simultaneously.
- **Brick cursor was off before the fix**: The `default:` case in `ctx_to_screen` was `*sc = c * cw`, ignoring the stagger. The cursor appeared one column to the left of the grid intersection on odd rows. Fix: explicit `GM_BRICK_H` and `GM_BRICK_V` cases.
- **Objects drawn with `(sr+1, sc+1)` offset** (for non-trivial cell sizes): places the object glyph in the interior of the cell rather than the top-left corner junction.
- **Grid cycling `(mode ± 1 + GM_COUNT) % GM_COUNT`**: wraps cleanly, never negative, always valid.

### Key Constants

| Name | Role |
|------|------|
| `GM_COUNT` | 14 — total grid modes, used for cycling |
| `MAX_OBJ` | 512 — object pool capacity |
| `PAIR_ACTIVE` | colour pair for grid name in bottom bar (bright green) |
| `PAIR_LABEL` | colour pair for key hints (light grey 252) |

### Open Questions

- What happens to objects placed on a brick grid when you switch to iso? Where do they appear?
- If you placed 512 objects and switch grids, do they all appear in valid screen positions?
- Why is `pool_toggle` O(n) for membership test but O(1) for removal?

---

## From the Source

**Algorithm:** Direct-manipulation cursor editor. No simulation tick. The main loop is: read input → update cursor/pool → `scene_draw` every frame.

**Math:** `ctx_to_screen` implements the forward transform from grid-cell address to screen pixel. For diamond/iso this is a rotation matrix scaled by the grid pitch: `[sc, sr] = [ox, oy] + [c-r, c+r] * [IW, IH]`.

**Rendering:** Background (`ctx_draw_bg`) → objects (`pool_draw`) → cursor (`cursor_draw`) → HUD. Bottom bar: grid name in bold `PAIR_ACTIVE`, key hints in `PAIR_LABEL` without `A_DIM`.

---

## Pass 2 — Implementation

### Pseudocode

```
main():
    ctx_init(&ctx, GM_UNIFORM, rows, cols)
    cursor_reset(&cur, &ctx)
    pool_clear(&pool)

    each frame:
        ch = getch()
        if arrow: cursor_move(&cur, &ctx, dr, dc)
        if space: pool_toggle(&pool, cur.r, cur.c, 'O')
        if 'a':   ctx_init(&ctx, (mode-1+GM_COUNT)%GM_COUNT, rows, cols)
        if 'e':   ctx_init(&ctx, (mode+1)%GM_COUNT, rows, cols)
        scene_draw(...)
```

### Module Map

```
§1 config   — grid constants (cw/ch per mode), colour pairs
§2 clock    — CLOCK_MONOTONIC sleep
§3 color    — 6 colour pairs, 256-color fallback
§4 gridctx  — GridCtx, GridMode enum, ctx_init, ctx_to_screen, ctx_draw_bg
§5 pool     — ObjectPool, pool_find, pool_toggle, pool_draw
§6 cursor   — Cursor, cursor_reset, cursor_move, cursor_draw
§7 scene    — scene_draw (bg → pool → cursor → HUD)
§8 screen   — screen_init/cleanup, SIGWINCH handler
§9 app      — main loop, signal flags
```

### Data Flow

```
getch() → cursor_move / pool_toggle / ctx_init
ctx_to_screen(g, r, c) → (sr, sc) → mvaddch
                ↑
    used by: cursor_draw, pool_draw, (path endpoints in 03)
```
