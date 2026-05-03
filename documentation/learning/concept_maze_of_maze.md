# Concept — `maze_of_maze.c`: Recursive DFS Mazes at Two Scales

## Core Idea

A perfect maze is a SPANNING TREE on the grid graph: every cell is connected, exactly one path between any two cells, no loops. The recursive backtracker builds one by depth-first random walk — at each step you knock down a wall to a fresh neighbour and step in; when no fresh neighbour exists, you back up. The result is a maze with long meandering corridors. To get a **MAZE OF MAZES**, take that algorithm and run it not once but TWICE per scale — once for the outer big maze, then once independently for every cell of that maze. Each cell becomes its own little maze, and you can look at the big picture or zoom into any room and find more structure.

---

## The Mental Model

Imagine a hotel where every room is itself a hotel. You walk down the long corridor of the OUTER hotel, picking doors at random. Open one — inside is not a bedroom but another corridor with more doors. Open one of THOSE — and inside is yet another (smaller) room. The structure repeats. From the lobby you can in principle reach any inner room of any outer room, but you have to navigate the outer maze first and then the inner maze second. Each outer room has its OWN floor plan; no two are the same.

The fact that BOTH levels are MAZES (perfect, every cell reachable) is just the Recursive Backtracker applied twice. The interesting thing is that the two levels are generated independently — outer and inner have no relationship — yet the visual whole reads as a single coherent fractal-ish structure because the eye groups adjacent cells naturally.

---

## Algorithm in Steps

1. **CHOOSE SIZES.** Outer maze is `W_outer × H_outer`; each inner maze is `W_inner × H_inner`. Pattern selects these (CLASSIC = 6×3 outer × 6×4 inner, etc.).

2. **CARVE OUTER.** `dfs_carve(outer, 0, 0)`:
   ```
   a. Mark current cell visited.
   b. Shuffle the 4 directions {N, E, S, W} with a deterministic seeded RNG.
   c. For each direction in shuffled order:
        neighbour = step in that direction
        if neighbour is in-bounds and unvisited:
          remove the wall between current and neighbour
          recurse into neighbour
   ```

3. **CARVE EACH INNER.** For `(ox, oy)` in `[0..W_outer)×[0..H_outer)`:
   ```
   seed_inner = base_seed ^ hash(ox, oy)
   dfs_carve(inner[ox, oy], 0, 0)  with this seed
   ```

4. **EACH FRAME:**
   a. Advance brightness wind.
   b. Compute outer cell screen size = `total_screen / (W_outer, H_outer)`.
   c. Compute inner cell screen size = `(outer cell − 1) / (W_inner, H_inner)`.
   d. For each outer cell, render its inner maze inside the outer cell's interior region.
   e. Render the outer maze on top (its walls overdraw the inner-maze walls at outer-cell boundaries).
   f. For each wall cell drawn, sample the fBm brightness field and choose A_DIM / A_NORMAL / A_BOLD.

5. HUD on top.

---

## Key Formulas

**Wall bit encoding** (per cell):
```
bit 0 (1) = North wall
bit 1 (2) = East wall
bit 2 (4) = South wall
bit 3 (8) = West wall
initial state: all walls present (0xF)
```

**DFS carve, removing wall** between cell `(x, y)` and neighbour at direction `d`:
```
walls[y][x]    &= ~(1 << d)            // remove our wall
walls[ny][nx]  &= ~(1 << OPPOSITE[d])  // remove their wall
```

**Direction tables** (clockwise from N):
```
DX = {  0,  1,  0, -1 }
DY = { -1,  0,  1,  0 }
OPPOSITE = { 2, 3, 0, 1 }
```

**Outer cell screen size:**
```
cw_outer = total_screen_w / W_outer
ch_outer = total_screen_h / H_outer
```

**Inner cell screen size** (within an outer cell, leaving 1 cell for the outer wall on each side, AND 1 extra for the inner maze's own far-edge border):
```
cw_inner = (cw_outer - 2) / W_inner    // (interior_w - 1) / W_inner
ch_inner = (ch_outer - 2) / H_inner
```

**Brightness modulation** (per wall cell):
```
b = fbm2(sx · BSC_X + wind_x, sy · BSC_Y · ASPECT_Y)
attr = (b > 0.65) ? A_BOLD : (b < 0.35 && !no_dim) ? A_DIM : A_NORMAL
```

---

## Edge Cases and Pitfalls

- **DFS RECURSION DEPTH.** Worst case is the longest possible Hamiltonian path through the maze grid — bounded by `W_outer · H_outer` (outer) or `W_inner · H_inner` (inner). With caps at 16 × 16 = 256 cells, recursion depth ≤ 256. Each stack frame is ~50 bytes; total ≤ 13 KB stack, comfortably within default thread stacks.

- **SHARED WALL BETWEEN ADJACENT CELLS.** Each interior wall belongs to TWO cells. The renderer draws BOTH cells' walls, so the interior wall gets drawn twice (idempotent, same colour). Cleaner alternatives (skip duplicates) would track a hash set of edges — not worth the complexity.

- **OUTER CELL VS SCREEN BOUNDARY.** The outermost edge of the outer maze (row 0 north, column 0 west, etc.) has all-cells-have-that-wall-set, so the outer maze always has a complete rectangular border. Good for visual cohesion.

- **INNER MAZE FITTING.** A maze with N cells needs `N · cell_size + 1` screen cells (the trailing `+1` is the maze's far-edge border wall). Use `(interior - 1) / N` to size cells, not `interior / N` — otherwise the maze's `+1` border doesn't fit and the inner-maze drawing silently bails out, leaving only the outer maze on screen. **This was a real bug in the file's first version.**

- **DETERMINISTIC SEED PER INNER MAZE.** We compute each inner maze's seed as `base_seed ^ hash3(ox, oy, salt)`, so the same `base_seed` always produces the same maze of mazes. Reseeding with `r` picks a new `base_seed` — every inner maze regenerates.

- **OUTER WALLS OVER INNER MAZES.** Render inner mazes FIRST, then outer mazes ON TOP. Otherwise outer-cell-boundary walls might be eaten by inner mazes that happen to draw a passage near the boundary.

- **NO_DIM ON INNER WALLS.** Inner walls live at a dimmer ramp slot than outer (`INNER_RAMP=5` vs `OUTER_RAMP=7`). If the brightness fBm field also drives them to `A_DIM`, they vanish entirely in the dim half of the wave. Pass `no_dim=true` for inner walls — clamp their brightness floor to A_NORMAL so they remain readable everywhere.

---

## How to Verify

- **Pause** (space). Brightness freezes. Resume: drift continues.

- Press **`r`**. The flash fires and ALL mazes change (outer + every inner). The pattern stays the same — rooms remain in the same grid arrangement — but the carved corridors all randomise.

- **CLASSIC** pattern. Count outer rooms: 6 × 3 = 18. Trace the outer maze: every room should be reachable from every other through door openings (cells where the outer wall is absent). Inside each room, the inner maze should also be a perfect maze (every inner cell reachable from every other inner cell of THE SAME ROOM).

- Switch to **WIDE**. Now there are only 4 × 2 = 8 outer rooms but each is large and contains a 10 × 6 = 60-cell inner maze. The fine detail per room is visibly higher than CLASSIC.

- Switch to **DENSE**. 32 outer rooms, but each inner maze is only 4 × 3 = 12 cells — rooms read as small "chunks" of layout.

- **Glyph cycle (g)**. The maze layouts don't change; only the wall rendering does. LINES gives clean ASCII art; BLOCKS makes a chunkier look; MIXED visually separates the outer (`#`) from the inner (`+ - |`) layer.

- Inner mazes ARE INDEPENDENT. Two adjacent rooms have unrelated inner layouts — the inner walls do NOT line up across the outer-room boundary.

---

## References

- Wikipedia — [Maze generation algorithm](https://en.wikipedia.org/wiki/Maze_generation_algorithm).
- Buck, Jamis (2015) — *Mazes for Programmers* (Pragmatic Bookshelf). The definitive guide to maze algorithms; recursive backtracker is chapter 1.
- Think Labyrinth — comprehensive maze-algorithm reference (http://www.astrolog.org/labyrnth/algrithm.htm).
- Wikipedia — [Depth-first search](https://en.wikipedia.org/wiki/Depth-first_search).

---

*Source: `procedural/patterns/maze_of_maze.c`*
