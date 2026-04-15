# Concept: Graph Search (BFS / DFS / A*)

## Pass 1 — Understanding

### Core Idea
Three fundamental graph traversal algorithms, each with different exploration strategies:
- **BFS**: explores level by level (shortest path in unweighted graphs)
- **DFS**: goes deep before backtracking (good for maze solving, cycle detection)
- **A***: guided by a heuristic (optimal path in weighted graphs)

### Mental Model
BFS: flood fill from the source — all cells at distance 1, then distance 2, etc.
DFS: follow one path as far as possible, backtrack when stuck.
A*: flood fill but prefer cells closer to the goal (heuristic guides direction).

### Key Equations
A* priority: `f(n) = g(n) + h(n)`
- g(n): cost from start to node n
- h(n): estimated cost from n to goal (heuristic)
- Manhattan distance heuristic: `h(n) = |nx - gx| + |ny - gy|`

BFS/DFS have no cost function — all edges have equal weight.

### Data Structures
- Grid: `uint8 grid[H][W]` with WALL=1, OPEN=0
- `visited[H][W]`: bool
- `parent[H][W]`: {row, col} for path reconstruction
- BFS: queue (FIFO)
- DFS: stack (LIFO) or recursion
- A*: min-heap (priority queue) on f-value

### Non-Obvious Decisions
- **BFS finds shortest path, DFS does not**: DFS finds A path, not necessarily shortest. BFS guarantees shortest in unweighted graphs.
- **A* heuristic must be admissible**: Never overestimate the true cost. Manhattan distance is admissible for grid with 4-directional movement.
- **Animate the search frontier**: Show `visited` cells growing in real time. BFS looks like a wavefront; DFS looks like a snake.
- **Path reconstruction**: Store `parent[node] = previous_node` during exploration. Backtrack from goal to start to get the path.

## From the Source

**Algorithm:** Three graph search algorithms animated side-by-side: BFS (queue-based, level-by-level, O(V+E)), DFS (stack-based, explores deeply before backtracking, O(V+E)), A* (priority queue min-heap, f(n)=g(n)+h(n), optimal with admissible heuristic, O((V+E) log V)).

**Data-structure:** Adjacency list graph (N=40 nodes, planar-ish random edges). Spring-repulsion layout (Fruchterman-Reingold): nodes repel like charged particles, edges attract like springs, until equilibrium — purely for visual legibility.

**Math:** Fruchterman-Reingold layout: repulsive force ∝ k²/d, attractive force ∝ d²/k, where k = √(area/N) is the ideal edge length. Converges in O(iterations × (V²+E)). A* heuristic h(n) = **Euclidean distance** (not Manhattan): admissible since it never overestimates the straight-line distance between nodes laid out in 2D space.

---

### Key Constants
| Name | Role |
|------|------|
| DIRS[4] | four movement directions (or 8 for diagonal) |
| WALL char | '#' or 1 in grid |
| STEP_RATE | how many search steps to show per frame |

### Open Questions
- Why is Dijkstra's algorithm a generalization of BFS?
- When does A* degrade to BFS? (h=0 everywhere)
- What happens when the heuristic is inadmissible (overestimates)?

---

## Pass 2 — Implementation

### Pseudocode
```
BFS(start, goal, grid):
    queue = [start]; visited[start]=true; parent[start]=null
    while queue not empty:
        node = queue.pop_front()
        if node == goal: return reconstruct_path(parent, goal)
        for each neighbor of node:
            if not visited[neighbor] and not wall[neighbor]:
                visited[neighbor]=true; parent[neighbor]=node
                queue.push_back(neighbor)
    return []  # no path

A_star(start, goal, grid):
    g[start]=0; f[start]=h(start,goal)
    open_heap = [(f[start], start)]
    parent = {}
    while open_heap not empty:
        (_, node) = heap_pop(open_heap)
        if node == goal: return reconstruct_path(parent, goal)
        for each neighbor of node:
            tentative_g = g[node] + edge_cost(node, neighbor)
            if tentative_g < g.get(neighbor, INF):
                parent[neighbor] = node
                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + h(neighbor, goal)
                heap_push(open_heap, (f[neighbor], neighbor))

reconstruct_path(parent, goal):
    path = []
    node = goal
    while node is not null: path.append(node); node = parent[node]
    return reverse(path)
```

### Module Map
```
§1 config    — H, W, STEP_RATE, algorithm selection
§2 grid      — maze generation (DFS) or random walls
§3 search    — BFS, DFS, A* implementations
§4 draw      — grid + visited frontier + path highlight
§5 app       — main loop, keys (algo select, reset, step, run)
```

### Data Flow
```
maze grid → search algorithm (step by step or batch)
→ visited frontier + found path → colored overlay on grid → screen
```
