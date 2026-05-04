# Concept: Ant Colony Optimization (ACO)

## Pass 1 — Understanding

### Core Idea
Ants find shortest paths through stigmergy: they deposit pheromone on paths they travel. Shorter paths accumulate more pheromone (ants return faster) and attract more ants. Longer paths evaporate. The colony converges on the near-optimal path without any central coordination — Deneubourg's 1990 double-bridge experiment, run live in the terminal.

### Mental Model
Imagine 80 ants leaving a nest and randomly exploring. Each lays a scent trail. Ants that find food and return quickly deposit fresh pheromone before the older pheromone evaporates. Other ants preferentially follow stronger trails. Over time, the shortest path gets the strongest trail and all ants converge to it.

The simulation uses 8-directional grid movement (not a graph). Each ant is a small state machine: SEARCHING ants pick from a forward-arc of three 8-dir candidates weighted by pheromone + food-direction bias; RETURNING ants bee-line back to the nest with ±1-step random deviation, depositing the heavier "return" trail. The pheromone grid `g_ph[H][W]` (float per cell) is the colony's shared memory.

### Key Equations
Pheromone update:
```
τ_ij(t+1) = (1-ρ) · τ_ij(t) + Σ_k Δτ_ij^k
```
Where ρ is evaporation rate and Δτ^k = Q/L_k if ant k used edge (i,j), else 0.

Probability of ant k choosing edge (i,j):
```
P(i→j) = [τ_ij^α · η_ij^β] / Σ_l [τ_il^α · η_il^β]
```
Where η_ij = 1/distance (heuristic), α controls pheromone weight, β controls heuristic weight.

### Data Structures
- Grid or graph: nodes with x,y positions
- `pheromone[N][N]`: float matrix, pheromone on each edge
- `ants[M]`: {current_node, visited[], path_length}
- `best_path[]`: shortest tour found so far

### Non-Obvious Decisions
- **Evaporation is essential**: Without evaporation, pheromone only accumulates and the system never forgets bad paths. ρ≈0.1 is typical.
- **α and β trade-off**: High α means ants blindly follow existing trails (exploitation). High β means they prefer short edges (exploration). Balance: α=1, β=2–5.
- **Elitist update**: Deposit extra pheromone on the best-ever solution each iteration. Helps convergence.
- **Visual**: Show pheromone intensity as ASCII density on the grid edges. Watch trails form and strengthen over time.

### 14 Food-Source Patterns (cycle with `n` / `N`)

Different patterns demonstrate different stigmergic topologies — the colony adapts its trail geometry to whatever the food problem looks like:

| Pattern | Sources | Layout |
|---------|---------|--------|
| DOUBLE | 2 | opposite sides (Deneubourg classic) |
| SINGLE | 1 | upper-right |
| QUAD | 4 | cardinal directions |
| LINE | 3 | collinear, right edge |
| HEXAGON | 6 | hex angles around nest |
| CROSS | 4 | mid-distance cardinals |
| TRIANGLE | 3 | equilateral vertices |
| CIRCLE | 8 | full ring at 45° |
| DIAGONAL | 4 | along main diagonal |
| CLUSTER | 5 | tight clump on right |
| DISTANT | 2 | extreme opposite corners |
| PERIMETER | 6 | spread along screen edges |
| GRID | 9 | 3×3 grid (skipping nest cell) |
| RANDOM | 7 | fixed-seed random positions |

All 14 share the same nest at the grid centre — only food layout changes. Pattern change clears all trails so old pheromone doesn't bias the new geometry.

### 6 Themes + Modern Loop

`t/T` cycles 6 themes: CLASSIC (blue/yellow/magenta), INFRARED, FOREST, NEON, TWILIGHT, SUMI_E (NEGATIVE inverted — white paper bg, dark fg). Every palette entry sits ≥ 24 in the 256-cube (per CLAUDE.md "bright half" rule), so even pheromone slot 0 stays legible under `A_DIM`.

The main loop reads input every iteration at 60 fps; `+`/`-` adjusts a `speed` multiplier (1×–16×) that scales how many sim ticks run per render frame. Earlier versions read input ONCE per frame after running `speed`-many ticks — keys felt laggy at high speed. The modern split keeps key response instant regardless of sim speed.

## From the Source

**Algorithm:** Stigmergic path finding — models the Deneubourg et al. (1990) double-bridge experiment. The colony memory lives in the pheromone field (the environment), not in any individual ant. This is the canonical definition of stigmergy: indirect coordination through environment modification. ACO is a metaheuristic for NP-hard combinatorial optimisation (travelling salesman, vehicle routing).

**Math:** This implementation uses 8-directional movement on a grid (not a graph), making it more visual but less mathematically rigorous than classical graph-based ACO. Pheromone update: `τ ← τ × (1 − ρ) + Δτ`. Path selection: `P(cell) ∝ τ^α × η^β`.

**Performance:** O(N_ants × grid_area) per tick for pheromone sensing over the 8-connected neighborhood.

---

### Key Constants
| Name | Typical | Role |
|------|---------|------|
| M | 10–50 | number of ants |
| ρ | 0.1–0.5 | evaporation rate |
| α | 1.0 | pheromone importance |
| β | 2.0–5.0 | heuristic importance |
| Q | 100 | pheromone deposit constant |
| τ₀ | 0.1 | initial pheromone level |

### Open Questions
- What happens when α=0? (pure greedy, no learning)
- What happens when β=0? (pure pheromone following, no distance heuristic)
- ACO on a dynamic graph where edge costs change mid-simulation?

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_ph[GRID_H_MAX][GRID_W_MAX]` | `float[100][320]` | ~128 KB | pheromone concentration per grid cell |
| `g_ants[N_ANTS]` | `Ant[50]` | ~800 B | ant positions, directions, and states |
| `g_food_row[2]`, `g_food_col[2]` | `int[2]` × 2 | 16 B | coordinates of the two food sources |
| `g_nest_row`, `g_nest_col` | `int` × 2 | 8 B | coordinates of the nest |

---

## Pass 2 — Implementation

### Pseudocode
```
init():
    pheromone[i][j] = TAU0 for all edges
    place ants at random start nodes

ant_tour(ant):
    visited = {start}
    path = [start]
    current = start
    while not all nodes visited:
        probs = []
        for each unvisited neighbor j:
            p = pheromone[current][j]^ALPHA * (1/dist[current][j])^BETA
            probs.append((j, p))
        normalize probs
        next = random_choice(probs)
        path.append(next); visited.add(next); current=next
    return path, total_length(path)

update_pheromones(all_tours):
    pheromone[i][j] *= (1 - RHO)   # evaporate
    for tour, length in all_tours:
        for each edge (i,j) in tour:
            pheromone[i][j] += Q / length

main_loop():
    each iteration:
        tours = [ant_tour(ant) for ant in ants]
        update_pheromones(tours)
        update_best(tours)
        draw_pheromone_grid()
        draw_best_path()
```

### Module Map
```
§1 config    — N_CITIES, M_ANTS, RHO, ALPHA, BETA, Q
§2 init      — random city placement, pheromone init
§3 ants      — ant_tour(), probability calculation
§4 update    — evaporate(), deposit(), elitist update
§5 draw      — pheromone intensity → ASCII, best path highlight
§6 app       — main loop (iterations), keys (speed, reset, params)
```

### Data Flow
```
cities + pheromone → ants probabilistic tours → update pheromone
→ pheromone matrix → density display + best path overlay
```

