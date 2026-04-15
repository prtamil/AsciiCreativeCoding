# Concept: Voronoi Diagram

## Pass 1 — Understanding

### Core Idea
Given N seed points, partition the plane into N regions where each region contains all points closer to its seed than to any other seed. The Voronoi diagram is the set of boundaries between these regions.

### Mental Model
Imagine N cities on a map. Each point on the map belongs to whichever city is closest. The boundary lines between neighboring cities form the Voronoi diagram. It's the "territory" map when distance is the only factor.

### Key Equations
Voronoi region of seed i:
```
V(i) = {x : |x - s_i| ≤ |x - s_j| for all j ≠ i}
```

Fortune's algorithm: O(N log N) sweep line, but for ASCII terminal:
- Brute-force nearest-neighbor: O(W·H·N) — acceptable for N<100, terminal resolution

### Data Structures
- Seeds: `{px, py, color_pair}` array
- For brute force: iterate all pixels, find nearest seed
- For animated: seeds can move (random walk or physics)

### Non-Obvious Decisions
- **Brute-force is fast enough**: At 100×40 terminal resolution with N=20 seeds, brute-force is 80,000 distance comparisons — trivial. Fortune's algorithm is unnecessary.
- **Aspect ratio correction**: Terminal cells are taller than wide. Scale y-distance by CELL_H/CELL_W when computing distances, otherwise the regions look squashed.
- **Animated seeds**: Make seeds drift slowly (random walk, bounce off walls) to create an animated tiling. Recompute every frame.
- **Coloring strategies**:
  - Flat color per region
  - Distance to nearest seed (gradient within each region)
  - Distance to second-nearest seed (highlights the edges)
  - Number of neighbors (the Delaunay dual)

### Key Constants
| Name | Role |
|------|------|
| N_SEEDS | number of Voronoi sites |
| CELL_W, CELL_H | terminal cell aspect correction |
| SPEED | seed movement rate (for animation) |

### Open Questions
- The Delaunay triangulation is the dual of the Voronoi diagram. Can you draw it?
- What is the average number of neighbors in a Voronoi diagram? (Answer: 6 for random points)
- Lloyd's algorithm: repeatedly move each seed to the centroid of its region. What shape does it converge to?

---

## From the Source

**Algorithm:** Brute-force Voronoi — O(cells × seeds) per frame. Efficient for small N (N_SEEDS ≤ 30); for larger N, Fortune's sweep-line algorithm gives O(N log N).

**Math:** The dual graph of the Voronoi diagram is the Delaunay triangulation — every Voronoi edge connects the circumcentres of two Delaunay triangles. Border detection: a cell is a border cell when the distances to the nearest seed and second-nearest seed differ by less than `BORDER_PX`. This approximates the Voronoi edge without exact line computation.

**Physics:** Seeds move under the Langevin equation: `dv/dt = −γ·v + σ·ξ` (ξ = white noise). This is the Ornstein-Uhlenbeck process — Brownian motion with mean-reverting velocity. Terminal speed = NOISE/DAMP.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_app` | `App` | ~400 B | Top-level singleton holding scene, screen, sim_fps, and signal flags |
| `g_app.scene.voronoi.seeds[N_SEEDS]` | `Seed[24]` | ~384 B | Position, velocity, and colour pair for each Voronoi seed |

## Pass 2 — Implementation

### Pseudocode
```
find_nearest(px, py, seeds) → seed_index:
    best_dist = INF; best = 0
    for i in 0..N_SEEDS:
        dx = (px - seeds[i].px) * CELL_W
        dy = (py - seeds[i].py) * CELL_H    # aspect correct
        d = dx²+dy²
        if d < best_dist: best_dist=d; best=i
    return best

draw():
    for row in 0..rows:
        for col in 0..cols:
            i = find_nearest(col, row, seeds)
            attron(COLOR_PAIR(seeds[i].color))
            mvaddch(row, col, region_char(col, row, seeds, i))

region_char(col, row, seeds, nearest) → char:
    # find second nearest
    second_dist = INF
    for j != nearest: ...
    edge_dist = sqrt(second_dist) - sqrt(nearest_dist)
    if edge_dist < 1.5: return '#'    # draw edge
    else: return ' '                   # fill

update_seeds():
    for each seed: random_walk or physics step
```

### Module Map
```
§1 config    — N_SEEDS, CELL_W/H, SPEED
§2 init      — random seed positions and colors
§3 voronoi   — find_nearest(), find_two_nearest()
§4 draw      — sweep pixels, color regions, draw edges
§5 app       — main loop, keys (add/remove seed, freeze, lloyd step)
```

### Data Flow
```
seeds[] → per-pixel nearest-neighbor search → region index
→ color + edge detection → mvaddch → screen
(seeds move each frame for animation)
```
