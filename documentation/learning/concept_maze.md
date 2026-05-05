# Pass 1 — maze.c: Recursive-Backtracker Generation + BFS Solve

## Core Idea

A perfect maze is a spanning tree of the grid graph: every cell reachable
from every other by exactly one path, no loops, no isolated regions.
DFS carves that tree by walking randomly into unvisited neighbours and
knocking out the wall between them; backtracking when stuck guarantees
every cell ends up connected. Once the tree exists the shortest path
between any two cells is unique, and BFS finds it because tree paths
have no shortcuts.

The current implementation animates both phases: GEN_STEPS=4 DFS pushes
per frame, then SOL_STEPS=16 BFS pops per frame, then the parent chain
traces back from the bottom-right corner to the top-left in glowing
matrix-green.

## Mental Model

Imagine a solid block of concrete divided into rooms by tall walls.
A miner with a sledgehammer starts at one corner. At each room she
picks a random neighbour she has not yet visited and smashes the wall
between them, then walks through. When all neighbours have already
been visited she retraces her steps until she finds one that hasn't.
The pattern of broken walls she leaves is a perfect maze.

After the carve, water poured at the start floods every room at the
same speed. The first drop to reach the goal arrived along the
shortest possible route — that's BFS finding the shortest path.

## Wall Encoding (the critical data trick)

Per-cell 4-bit bitmask:
```
bit 0 = N, bit 1 = E, bit 2 = S, bit 3 = W
0b1111 = all walls closed
0b0000 = all walls open
```

When the miner carves between (r,c) going north into (r-1,c), TWO bits
must clear:
```c
g_walls[r  ][c] &= ~WALL_N;  /* clear north on this side  */
g_walls[r-1][c] &= ~WALL_S;  /* clear south on the neighbour */
```

This symmetric representation means a wall query works from EITHER
side. BFS later asks "can I move from (r,c) north?" and gets a single-
bit answer in O(1) — the carve walked the bookkeeping, not the lookup.

## Display Geometry

Each maze cell occupies a 2×2 patch of terminal characters:
```
+-+-+-+        + corner (0,0), (0,2), (0,4) ...
| | | |        | vertical wall on odd row, even col
+-+-+-+        - horizontal wall on even row, odd col
| | | |        space cell interior on odd row, odd col
+-+-+-+
```

A W×H maze occupies (2W+1) × (2H+1) terminal cells. With HUD_ROWS=2,
the maze pixel rectangle starts at terminal row HUD_ROWS and ends at
HUD_ROWS + 2H. Pixel parities decide which lattice element to draw:

| (pr & 1, pc & 1) | element              | open form | closed form |
|------------------|----------------------|-----------|-------------|
| (0, 0)           | corner               | always `+`            |
| (0, 1)           | horizontal wall      | ' '       | `-`         |
| (1, 0)           | vertical wall        | ' '       | `\|`        |
| (1, 1)           | cell interior        | depends on phase + state |

## Color Encoding (visibility-first)

All pairs use bg = -1 (terminal default). Foregrounds sit in the
bright half of the 256-colour cube so A_BOLD/A_DIM stay legible.

| Pair         | Color      | Used for                    |
|--------------|------------|-----------------------------|
| PAIR_WALL    | 251 grey   | walls and corners (A_BOLD)  |
| PAIR_VISIT   | 244 grey   | carved cell interior        |
| PAIR_FRONT   | 226 yellow | DFS frontier `@` (A_BOLD)   |
| PAIR_BFS     | 117 blue   | BFS-visited cells `.`       |
| PAIR_PATH    | 46 green   | final `*` path (A_BOLD)     |
| PAIR_UNVISIT | 240 grey   | not-yet-carved `#`          |
| PAIR_HUD     | 226 yellow | top status bar (A_BOLD)     |
| PAIR_HINT    | 51 cyan    | bottom key strip (A_BOLD)   |

The earlier version had `init_pair(CP_WALL, 232, 232)` — fg = bg = near-
black, so walls were literally invisible against the default-black
terminal. The current palette guarantees every glyph reads at every
attribute level.

## Worked Example (defaults: 90×23 maze on 80×24 terminal)

`calc_dims(rows=24, cols=80)` rounds down: `mh = (24-2-1)/2 = 10`,
`mw = (80-1)/2 = 39`. Maze cells = 390, total terminal cells used =
21 wall rows × 79 wall cols = 1659.

Generation cost: each DFS step does at most 4 random direction
checks + one carve. 390 cells, two visits each (push + pop) = 780
DFS steps total. At GEN_STEPS=4 per frame and 30 fps, that's ~6.5
seconds of carving — long enough to watch the snake-like advance.

Solve cost: BFS visits up to 390 cells. SOL_STEPS=16 per frame at
30 fps → ~0.8 s. The shortest path on a random recursive-backtracker
maze is typically 1.5–3× the Manhattan distance (here mh+mw–1 = 48),
so expect 60–150 cells of green path.

## Skip-to-solve (space key)

Pressing space during PH_GENERATE runs `while (gen_step()) {}` to
completion BEFORE calling `solve_start()`. This is critical: starting
BFS while half the walls are still closed would let the flood
terminate on a dead end, and the bottom-right cell might be
unreachable through the partial tree.

## Iterative DFS Stack

The recursive backtracker is famously written recursively but a stack
of (r, c) pairs works identically with no stack-overflow risk:

```c
static struct { int r, c; } g_dfs[MAZE_H_MAX * MAZE_W_MAX + 1];
static int g_dfs_top;
```

The +1 is paranoia — a pathological carve that visits every cell before
ever backtracking needs all W·H slots. Each gen_step() reads the
top, picks an unvisited neighbour, carves, pushes (or pops on dead end).

## BFS with Encoded Parents

Parent pointers stored as `r * MAZE_W_MAX + c` — a single int per cell,
no `(r,c)` struct. To walk back: `pr = enc / MAZE_W_MAX; pc = enc %
MAZE_W_MAX`. Sentinel value `-1` means "no parent" (start cell or
unvisited).

The BFS queue is a flat array (`g_bq`) with head/tail indices —
classic ring-buffer-without-the-wraparound because we know it'll
hold at most W·H items.

## Non-Obvious Decisions

- **Animated DFS visual personality.** Recursive-backtracker mazes have
  long winding corridors with few dead ends — visually "river-like".
  Wilson's algorithm or Prim's would produce mazes with shorter
  average path lengths and more branching, but the current carver
  shows a continuous yellow `@` snake which reads better as motion.

- **2×2 cell display, not 1×1.** A 1×1 representation would let cells
  share walls with neighbours, halving width but making the wall/cell
  ambiguity (is this pixel a wall or part of two cells?) confusing.
  2×2 with explicit corner glyphs (`+`) makes the lattice unambiguous.

- **HUD on rows 0 and last.** Status row 0 (yellow A_BOLD) and hint
  row last (cyan A_BOLD) sandwich the maze. The maze pixel rectangle
  starts at HUD_ROWS=2 to leave room for the status; the bottom hint
  uses the very last row regardless of maze height.

- **Resize resets the maze.** SIGWINCH recomputes (g_mh, g_mw) and
  calls maze_reset(). Carrying state through a resize would point off-
  grid (the wall arrays are sized for MAZE_W_MAX/MAZE_H_MAX, but
  g_mw/g_mh shrink on terminal narrowing) and the partial tree would
  no longer be solvable.

## Module Map

```
§1 config  — sizes, wall bits, phase enum, color pair IDs
§2 clock   — clock_ns + clock_sleep_ns
§3 color   — bg=-1 + bright-half foregrounds; PAIR_HUD/PAIR_HINT split
§5 maze    — wall bitmask + iterative DFS carver
§6 solve   — BFS flood + parent-array path reconstruction
§7 scene   — mark_cell helper + draw_lattice_pixel + draw_cell_interior
§8 app     — signals, resize, handle_key, step_simulation, main loop
```

## Pseudocode

```
gen_step():
  if dfs_top == 0: return done
  (r, c) = dfs[dfs_top - 1]
  shuffle dirs[0..3]
  for d in dirs:
    (nr, nc) = (r + DR[d], c + DC[d])
    if in_bounds and not visited[nr][nc]:
      walls[r][c]   &= ~D_WALL[d]
      walls[nr][nc] &= ~D_OPP [d]
      mark visited; push (nr, nc); return
  dfs_top--   # backtrack

solve_step():
  if queue_empty: phase = DONE; return
  (r, c) = queue.pop_front()
  if (r, c) == goal:
    trace_path back via parent[]; phase = DONE; return
  for d in 0..3:
    if walls[r][c] & D_WALL[d]: continue   # wall closed
    (nr, nc) = (r + DR[d], c + DC[d])
    if in_bounds and not bfs_visited[nr][nc]:
      bfs_visited[nr][nc] = 1
      parent[nr][nc] = encode(r, c)
      queue.push_back((nr, nc))
```

## Edge Cases

- **Negative modulo.** Shuffle uses unsigned indices into a fixed 4-
  element array — no negative-mod trap. But indexing `g_walls[r-1][c]`
  from r=0 column would underflow; `(r >= 0) && !(walls[r][c] & WALL_S)`
  short-circuits before the array read.

- **Stack of size W·H + 1.** A pathological carve visits every cell
  before backtracking once; the +1 covers the sentinel push.

- **A_DIM on grayscale walls.** Avoided: PAIR_WALL = 251 with A_BOLD
  (NEVER A_DIM) so walls stay visible against any terminal background.

- **Already-sorted-input quicksort analogue.** Doesn't apply here —
  DFS never has a worst case in this sense; the carve pattern depends
  on RNG order but every random arrangement produces a valid spanning
  tree.

## How to Verify

- After generation, every cell becomes visited. `g_dfs_top == 0` AND
  `g_vis[r][c] == 1` for every (r,c). Visually: no `#` glyphs remain.

- The path is a valid sequence of carved-wall traversals. Walking
  from (mh-1, mw-1) back via parent[] should never cross a wall whose
  bit is set.

- Path length ≥ Manhattan distance (mh + mw – 2). On a 39×10 maze
  the lower bound is 47; observed lengths typically 60–150 cells.

- Doubling MAZE_W_MAX from 90 to 180 should roughly double both
  carve animation duration and BFS solve duration (linear in cell
  count at fixed steps-per-frame).

- Symmetry. If only one side of a wall is cleared (a bug), BFS sees
  the opening from one cell but not the other, and the solve frontier
  gets stuck. The two `&= ~D_WALL[d]` and `&= ~D_OPP[d]` lines must
  both fire on every carve.

## Open Questions

1. Replace the recursive backtracker with Wilson's algorithm (random
   walk until hitting visited territory). Are the carved mazes
   visibly more branched, less river-like?
2. Carve a maze with loops by clearing 10% of remaining walls AFTER
   DFS completes. Does BFS still find a sensible shortest path?
3. Find the maze diameter (longest shortest path between any two
   cells). This is two BFS runs: from any cell to find the farthest
   cell A, then from A to find the farthest cell B. Distance(A,B)
   is the diameter.
4. Visualise the BFS visit order with a colour gradient (early visits
   blue, late visits red). Does the gradient reveal the tree's
   "trunk" and "branches"?

## References

- Reiter, *Maze Generation Algorithms*, https://weblog.jamisbuck.org/2011/2/7/
- Wikipedia, *Maze generation algorithm*,
  https://en.wikipedia.org/wiki/Maze_generation_algorithm
- Red Blob Games, *Introduction to A\* / BFS*,
  https://www.redblobgames.com/pathfinding/a-star/introduction.html
