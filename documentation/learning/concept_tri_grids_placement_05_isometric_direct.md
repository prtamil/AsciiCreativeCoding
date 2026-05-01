# Concept: Isometric (Solid-Fill) Direct Placement

## Pass 1 — Understanding

### Core Idea
Direct cursor + ObjectPool placement on the iso solid-fill grid from `tri_grids/05_isometric.c`. Same lattice and cursor mechanics as `01_equilateral_direct`; the only difference is paint. Background is solid-filled by `palette_index = (col + 2·row + up) mod 6`; glyphs render in `PAIR_CURSOR` (white-on-black) so they pop against any palette colour.

### Render Pipeline
1. Raster-scan: for every screen cell, `pixel_to_tri → palette_index → mvaddch(' ')` with the fill pair.
2. `pool_draw`: glyph at each placed object's centroid cell using `PAIR_CURSOR`.
3. `cursor_draw`: '@' at cursor centroid.

No edge characters. The cell IS the colour.

### Why White-On-Black For Glyphs
Glyphs sit on top of one of 6 fill colours. A non-contrasting pair would disappear over its matching fill. White-on-black has high luminance contrast against every palette slot.

### Theme Cycle
't' rebuilds the 6 palette pairs (`PAIR_FILL_BASE..+5`) but does NOT change the palette HASH — the same triangle index keeps its slot. Same triangle ends up under a different fill colour after theme change; placed glyphs keep their addresses.

### Non-Obvious Decisions
- **Same TRI_DIR as 01_equilateral**: the lattice is unchanged; only render differs. Cursor walking logic is identical.
- **Palette wraps around vertices**: 6 triangles meeting at any vertex spell out the full 6-cycle, producing the iso "stacked cubes" look — consistent regardless of cursor position.
- **No BORDER_W**: there are no edge characters to threshold; every pixel gets a fill colour.

### Key Constants
| Name | Role |
|------|------|
| `MAX_OBJ` | Pool capacity (256) |
| `N_PALETTE` | 6 — slots in the iso colour cycle |
| `N_THEMES` | Available palettes |
| `PAIR_FILL_BASE` | First palette pair ID |

### Open Questions
- What hash would tile cubes in different orientations (stacked sideways vs upright)?
- How would you add subtle shading per face? (Vary brightness within a fill pair?)

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — MAX_OBJ, N_PALETTE, N_THEMES, PAIR_FILL_BASE
§3 color    — PAL256/PAL8 per-theme palettes; init_pair for each slot
§4 formula  — pixel_to_tri (same as 01) + palette_index hash
§5 pool     — TObj{col, row, up, glyph}, pool_find, pool_toggle
§6 cursor   — TRI_DIR (same as 01) + step + draw
§7 scene    — solid-fill raster + pool + cursor
§8 screen   — ncurses init / cleanup
§9 app      — signals, main loop
```

### Data Flow
```
arrow → TRI_DIR[dir][cur->up] → cur += delta
SPACE → pool_toggle(cur)
't'   → next theme → color_init(theme)
frame:
    raster: pixel_to_tri → palette_index → fill pair → mvaddch(' ')
    pool_draw + cursor_draw (PAIR_CURSOR overlay)
```
