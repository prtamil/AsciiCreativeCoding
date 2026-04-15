# Concept: Hex Life (Cellular Automaton on Hexagonal Grid)

## Pass 1 — Understanding

### Core Idea
Conway's Game of Life generalized to a hexagonal grid. Each cell has 6 neighbors instead of 8. New birth/survival rules are chosen for the 6-neighbor topology. The hexagonal lattice produces qualitatively different patterns — more symmetrical, rounder growth.

### Mental Model
Instead of square tiles, imagine honeycomb. Each hexagon touches exactly 6 others. Apply a birth/survival rule: "a dead cell with exactly 2 live neighbors is born; a live cell survives with 3–4 neighbors." The hexagonal symmetry produces very different structures than square Life.

### Key Math
Offset row coordinate system (even rows shifted right by 0.5):
```
For even row i: neighbors of (i,j) = (i±1,j), (i±1,j+1), (i,j±1)
For odd  row i: neighbors of (i,j) = (i±1,j), (i±1,j-1), (i,j±1)
```

Screen coordinates from hex (i,j):
```
col = j * 2 + (i % 2 == 1 ? 1 : 0)
row = i * (some vertical step)
```

### Data Structures
- `grid[H][W]`: uint8 on logical hex grid
- Double buffer: `next[H][W]`
- Neighbor list: depends on even/odd row

### Non-Obvious Decisions
- **Offset coordinates are confusing**: Even rows shift right by half a cell. This is the "offset row" system — easy to store but fiddly to compute neighbors. Alternative: axial coordinates (cleaner math).
- **6 neighbors change the rule space**: With 8 neighbors, Life rules use counts 0–8. With 6 neighbors, counts are 0–6. Classic Hex Life uses B2/S34.
- **Terminal display**: A hex grid in terminal looks like alternating offset rows of characters. Every other row is shifted one column. Use spaces between cells for visual clarity.
- **Rule discovery**: Hex Life rule B2/S34 produces "gliders" (different shape). B2/S2 is explosive. B3/S5 creates stable blocks.

## From the Source

**Algorithm:** Conway's Game of Life adapted for a hexagonal grid. Each cell has 6 neighbours (vs 8 for square grids). Rule B2/S34: born on exactly 2 live neighbours, survives on 3 or 4 live neighbours. Uses double-buffering: write next generation to the inactive buffer, then swap — O(rows × cols) per step.

**Physics:** Hexagonal CA rules produce qualitatively different behaviour than square-grid Life: the B2/S34 rule creates stable oscillators and gliders unique to the hex topology. 6-neighbour symmetry eliminates diagonal artefacts present in square-grid CAs (more isotropic diffusion of patterns).

**Data-structure:** Flat 2-D array with uint8_t age field. Offset-rows layout: odd rows shifted right by 0.5 cells visually. Neighbour coordinates differ for even vs odd rows (two lookup tables, one per parity). Age-based colour: newborn → yellow, young → cyan, mature → teal, elder → dark.

### Key Constants
| Name | Role |
|------|------|
| BIRTH_MASK | which neighbor counts cause birth (6-bit) |
| SURVIVE_MASK | which neighbor counts allow survival (6-bit) |
| H, W | logical hex grid dimensions |

### Open Questions
- Does Hex Life B2/S34 have gliders? What do they look like?
- Convert to axial coordinates — does the neighbor computation simplify?
- What happens with the "majority rule": born if >3 neighbors, survive if >2?

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_grid[GRID_H_MAX][GRID_W_MAX]` | `int8[60][200]` | ~12 KB | current generation live/dead state |
| `g_next[GRID_H_MAX][GRID_W_MAX]` | `int8[60][200]` | ~12 KB | next generation scratch buffer |
| `g_age[GRID_H_MAX][GRID_W_MAX]` | `uint8[60][200]` | ~12 KB | generations a cell has been continuously alive |
| `g_anext[GRID_H_MAX][GRID_W_MAX]` | `uint8[60][200]` | ~12 KB | next-generation age scratch buffer |

---

## Pass 2 — Implementation

### Pseudocode
```
hex_neighbors(i, j) → list of (ni, nj):
    offsets_even = [(-1,0),(-1,1),(0,-1),(0,1),(1,0),(1,1)]
    offsets_odd  = [(-1,-1),(-1,0),(0,-1),(0,1),(1,-1),(1,0)]
    offsets = offsets_even if i%2==0 else offsets_odd
    return [(i+di, j+dj) for (di,dj) in offsets if in_bounds(i+di,j+dj)]

step():
    for i,j in grid:
        count = sum(grid[ni][nj] for ni,nj in hex_neighbors(i,j))
        alive = grid[i][j]
        if alive:
            next[i][j] = (SURVIVE_MASK >> count) & 1
        else:
            next[i][j] = (BIRTH_MASK >> count) & 1
    swap(grid, next)

draw():
    for i in 0..H:
        for j in 0..W:
            col = j*2 + (i%2)          # offset display
            row = i
            if grid[i][j]: mvaddch(row, col, 'O')
            else:           mvaddch(row, col, '.')
```

### Module Map
```
§1 config    — H, W, BIRTH_MASK, SURVIVE_MASK
§2 init      — random fill, pattern load (glider etc.)
§3 neighbors — hex_neighbors() with even/odd row offset
§4 step      — apply rule with double buffer
§5 draw      — offset row rendering
§6 app       — main loop, keys (step, pause, rule, draw mode)
```

### Data Flow
```
grid[H][W] → hex_neighbors → count → rule → next[H][W]
swap → offset-row display → screen
```
