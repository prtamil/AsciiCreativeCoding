# Concept: Maze Generation and Solving

## Pass 1 — Understanding

### Core Idea
Generate a perfect maze (exactly one path between any two cells) using DFS (recursive backtracker), then solve it using BFS (guaranteed shortest path) or A*. Animate both generation and solving.

### Mental Model
Generation: start in a cell, randomly carve passages to unvisited neighbors. Backtrack when stuck. Like exploring a dungeon — carve new rooms until every room has been visited. The result is a spanning tree of the grid.

Solving: from entrance to exit, flood-fill (BFS) to find the shortest path. Every cell at distance d from entrance is reached before any cell at distance d+1.

### Key Algorithms
DFS generation (recursive backtracker):
- All cells start unvisited
- Pick random unvisited neighbor → remove wall between them → recurse
- Produces long winding corridors

BFS solving:
- From entrance, explore level by level
- First time exit is reached = shortest path
- Reconstruct path backward using parent pointers

### Data Structures
- `cell[H/2][W/2]`: each cell has walls on 4 sides (bitmask: N/S/E/W)
- Or: `grid[H][W]` where odd rows/cols are walls, even are cells
- `visited[H/2][W/2]`: bool for generation
- `parent[H/2][W/2]`: for path reconstruction
- DFS stack or recursion; BFS queue

### Non-Obvious Decisions
- **Cell vs. wall grid**: Two representations — (1) store cells with wall flags, (2) use a fine grid where walls and cells alternate. Option 2 is simpler to draw.
- **Perfect maze = spanning tree**: DFS generates a tree. Every cell reachable from every other cell by exactly one path. No loops, no isolated regions.
- **Wilson's algorithm**: Alternative to DFS — random walk until it hits visited territory, then carve that path. Produces uniform spanning trees (DFS is biased toward long corridors).
- **Animate generation**: Show cells being carved in real time. The DFS stack determines the frontier.

## From the Source

**Algorithm:** Recursive-backtracker DFS maze generation. Maintains a stack of visited cells; at each step, if unvisited neighbours exist, choose one randomly, carve the wall between them, and push. If no unvisited neighbours, pop (backtrack). Produces uniform-random spanning trees.

**Data-structure:** Wall encoding: 4-bit bitmask per cell (N=1, E=2, S=4, W=8). Carving a wall clears the bit in the current cell AND sets the opposite bit in the neighbour (symmetric representation). Display: 2×2 terminal pixels per maze cell, so a W×H maze occupies (2W+1) × (2H+1) terminal cells.

**Math:** BFS solve gives shortest path by cell count (unweighted graph). Neighbours are reachable if the corresponding wall bit is 0. Parent array reconstructs path.

**Performance:** `GEN_STEPS=4` DFS steps per frame (animated generation); `SOL_STEPS=16` BFS steps per frame (animated solve). Generation: O(W×H) total. Solve BFS: O(W×H) total.

---

### Key Constants
| Name | Role |
|------|------|
| ROWS, COLS | maze cell dimensions |
| WALL_CHAR | '#' or '█' |
| PATH_CHAR | ' ' |
| SOLVE_SPEED | BFS steps per frame during solving |

### Open Questions
- How does maze structure (DFS vs. Prim's vs. Wilson's) affect BFS solve time?
- Generate a maze with loops by randomly removing 10% of walls after DFS.
- Can you find the diameter of the maze (longest shortest path)?

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_walls[MAZE_H_MAX][MAZE_W_MAX]` | `unsigned char[]` | ~2 KB | wall bitmask per cell (N=1, E=2, S=4, W=8); 0=open |
| `g_vis[MAZE_H_MAX][MAZE_W_MAX]` | `unsigned char[]` | ~2 KB | visited flag for DFS generation |
| `g_dfs[MAZE_H_MAX * MAZE_W_MAX]` | `struct{int r,c}[]` | ~4 KB | DFS backtrack stack |
| `MAZE_W_MAX`, `MAZE_H_MAX` | constants | N/A | maximum maze size (90 × 23 cells) |
| `GEN_STEPS` | `int` constant | N/A | DFS steps per frame during animated generation (4) |
| `SOL_STEPS` | `int` constant | N/A | BFS steps per frame during animated solve (16) |

## Pass 2 — Implementation

### Pseudocode
```
# Fine-grid representation: maze[2H+1][2W+1]
# Cells at even (row,col), walls between them

generate_dfs():
    stack = [(0,0)]
    visited[0][0] = true
    while stack not empty:
        (r,c) = stack.top()
        unvisited = [neighbor for neighbor in adj(r,c) if not visited]
        if unvisited empty: stack.pop(); continue
        (nr,nc) = random_choice(unvisited)
        # carve wall between (r,c) and (nr,nc)
        wall_r = r + nr; wall_c = c + nc   # midpoint in fine grid
        maze[r*2 + (nr-r)][c*2 + (nc-c)] = OPEN
        visited[nr][nc] = true
        stack.push((nr,nc))

solve_bfs(start, end):
    queue = [start]; visited[start]=true
    while queue:
        pos = queue.dequeue()
        if pos == end: return reconstruct(parent, end)
        for each open neighbor:
            if not visited: visited=true; parent[nbr]=pos; queue.enqueue(nbr)

draw():
    for r in 0..2H+1:
        for c in 0..2W+1:
            ch = WALL_CHAR if maze[r][c]==WALL else PATH_CHAR
            if (r,c) on solution path: ch = '+'
            if (r,c) == start or end: ch = '*'
            mvaddch(r, c, ch)
```

### Module Map
```
§1 config    — ROWS, COLS, GEN_SPEED, SOLVE_SPEED
§2 generate  — dfs_generate() (animated step by step)
§3 solve     — bfs_solve(), a_star_solve()
§4 draw      — maze grid + solution path + generation frontier
§5 app       — main loop (generate → solve → reset), keys
```

### Data Flow
```
empty maze → DFS carve passages → perfect maze
→ BFS solve → solution path → draw maze + overlay path
```
