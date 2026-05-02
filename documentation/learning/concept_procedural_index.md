# Concept: Procedural Generation — Master Index

This index lists every procedural-generation showcase in the
`procedural/` tree, grouped by family, with a one-paragraph
description of the algorithm and a link to its source. For deep
algorithmic discussion see Master.md §X. For the shared
architectural conventions see Framework.md and Architecture.md §179.

The 20 showcases fall into two families based on what they produce:

- **`procedural/generational/`** — discrete, one-shot generators that
  build a structure step-by-step then HOLD on the result.
- **`procedural/fields/`** — continuous noise / flow fields that
  animate forever via time drift and morphing.

---

## Pass 1 — Family overview

### `procedural/generational/` — discrete builders

State machines typically run `BUILDING / SOLVING / HOLD` then
supernova-reset to start over.

| File | Algorithm | Era / Reference |
|---|---|---|
| `wfc_learn.c` | Wave Function Collapse — slow, with entropy heatmap and step-through | Maxim Gumin 2016 |
| `wfc_showcase.c` | WFC — 34-tile alphabet, 3 weight classes, multi-seed, looping spectacle | (same) |
| `maze_backtracker.c` | DFS recursive-backtracker maze + tree-diameter solution beam | Jamis Buck tutorial |
| `wilsons_algorithms_maze_showcase.c` | Wilson's loop-erased random walk uniform spanning tree | Wilson 1996 |
| `bsp_dungeon_showcase.c` | Recursive binary space partition + room placement + L-corridors | Buck "Rooms and Mazes"; classic roguelike |
| `cellular_automata_cave_4-5_rule_showcase.c` | Random fill + iterated 4-5 rule (sweep with double-buffer wavefront) | RogueBasin |
| `drunkards_walk_cave_showcase.c` | Multi-walker random walk cave carving | RogueBasin |
| `diamond_square_heightmap_showcase.c` | Fractal terrain with biome filter keys + height bands | Fournier-Fussell-Carpenter 1982 |
| `voronoi_region_map.c` | 8-seed Voronoi with distance-sorted reveal animation | Standard Voronoi |
| `delaunay_triangulation.c` | Bowyer-Watson incremental Delaunay (Bresenham edge rendering) | Bowyer 1981, Watson 1981 |
| `poission_disk_sampling_showcase.c` | Bridson's fast Poisson-disk sampler with 5×5 cell-grid | Bridson 2007 |

### `procedural/fields/` — continuous fields

State is just `paused/running` with periodic permutation re-shuffle
every ~12 s. Five patterns per file, navigable with `n`/`p`. Ten
themes (`t`/`T`). Five glyph sets `g`/`G` (slim → fat: SLIM, LIGHT,
MEDIUM, HEAVY, FAT).

| File | Algorithm | Patterns |
|---|---|---|
| `perin_noise_flow_showcase.c` | Particles flowing along the Perlin noise gradient | FLOW / HEIGHT / WARP / FBM / CONTOUR |
| `simplex_noise_clouds.c` | Ken Perlin's 2001 simplex noise (isotropic alternative) | CLOUDS / WISPS / TURBULENCE / BILLOW / RIDGED |
| `worley_cellular_noise.c` | Worley's cellular noise — F1, F2, F2-F1 for boundary lines | F1 / F2_F1 / F2 / MANHATTAN / CELL_ID |
| `domain_warped_noise_iq_style.c` | Inigo Quilez recursive domain warping `f(x + h(x + g(x)))` | RAW / WARP1 / WARP2 / WARP3 / RIDGE |
| `curl_noise_vector_field.c` | Divergence-free `∇×ψ` flow over Perlin potential | PARTICLES / VECTOR / POTENTIAL / CURL_MAG / WARPED |
| `flow_field_particles.c` | Algebraic vector fields (no noise) | VORTICES / WAVE / SADDLE / MAGNET / TURBULENT |
| `magnetic_fields.c` | Magnetic-pole field-line viz (red 'N' + blue 'S' markers) | DIPOLE / QUAD / OCTUPOLE / CHAIN / PLASMA |
| `reaction_diffusion_gray_scott.c` | Gray-Scott PDE (explicit Euler + 5-point Laplacian) | SPOTS / STRIPES / MAZES / CORAL / WORMS |
| `midpoint_displacement_coastline.c` | 1-D fractal silhouettes with smooth A→B morphing | COASTLINE / MOUNTAINS / CITY / VALLEY / WAVES |

---

## Pass 2 — Per-algorithm pointers

Each program has full CONCEPTS + MENTAL MODEL teaching blocks at the
top of its source file. Read those first; the per-algorithm summary
below is just the elevator pitch.

### Wave Function Collapse — `wfc_*.c`

Pick the cell with the fewest remaining tile options (lowest
entropy), commit it to one tile, propagate the constraint outward.
Repeat until every cell has exactly one tile. Result: globally-
consistent tiling without backtracking.

### Recursive-Backtracker DFS Maze — `maze_backtracker.c`

DFS random walk carving walls. Backtracks when stuck. Output is a
"perfect maze" (uniform spanning tree). Plus tree-diameter solution
beam via two BFS passes.

### Wilson's UST Maze — `wilsons_algorithms_maze_showcase.c`

Random walk from outside the tree; loop-erase whenever the walk
self-intersects. When the walk hits the tree, absorb. Result is a
**uniform** random spanning tree (every possible maze equally likely)
— a stronger property than DFS.

### Cellular-Automata Cave (4-5 rule)

`cell becomes WALL iff ≥5 of its 8 Moore neighbours are walls`.
Random fill at 45 %, then iterate 4 times. Out-of-grid neighbours
count as walls so caves don't reach the edge.

### Drunkard's Walk Cave

N walkers each pick a random cardinal direction every tick and
flood the cell to FLOOR. Walkers respawn after age limit at a
random already-floor cell. Stops at 45 % carved fraction.

### BSP Dungeon

Recursively split rectangles along their longer axis. Place a
smaller room inside each leaf with random padding. Connect siblings
via L-corridors using post-order traversal of the BSP tree.

### Diamond-Square Heightmap

Recursive midpoint displacement on a 2-D grid. DIAMOND step sets
each square's centre to its 4 corners' average + jitter; SQUARE step
sets cross-midpoints to their cardinal neighbours' average + jitter.
Halve jitter each level → fractal terrain.

### Voronoi Region Map

Brute-force nearest-seed assignment per cell, then sorted-by-distance
reveal. Each seed's region grows visibly from its centre outward.

### Delaunay Triangulation (Bowyer-Watson)

Incremental insertion. Super-triangle covers the map. Each new
point: find triangles whose circumcircle contains it, remove them,
fan new triangles from each cavity-boundary edge to the new point.

### Poisson-Disk Sampling (Bridson)

Active-list approach with a `r/√2` background grid (each cell holds
≤ 1 sample). For each active point, generate candidates in the
annulus `[r, 2r]` around it and check distances against the grid.
After K=30 misses, drop the point from the active list.

### Perlin Noise & FBM Flow

Particles step in the local Perlin-noise-derived angle. fBm = sum of
4 octaves at doubling frequency / halving amplitude. Five
visualisation patterns include the noise itself as heightmap, with
domain warping, and as contour lines.

### Simplex Noise

Perlin's 2001 successor. Triangular simplex lattice eliminates the
square-lattice axis bias of original Perlin noise; faster (3 vs 4
corner contributions in 2-D). Quintic fade curve for C² continuity.

### Worley / Cellular Noise

For each query, find the F1 (nearest) and F2 (second-nearest)
distances to feature points hashed-determined per integer tile.
Different functions give different visualisations: F1 alone for
cellular blobs, F2 − F1 for sharp Voronoi boundary lines.

### Domain Warping (Inigo Quilez)

`f(x + h(x + g(x)))` — recursively offset the noise's input
coordinates by other noise. Each level of warping multiplies visual
richness; the WARP2 (2-level) pattern is IQ's canonical "marble"
look.

### Curl Noise

`v = (∂ψ/∂y, −∂ψ/∂x)` of a scalar potential ψ. Divergence-free by
construction. Particles flowing through this field swirl into
eternal vortices, never converging to sources or diverging from
sinks. Standard tool for fluid/smoke effects.

### Flow Field Particles (Algebraic)

Closed-form vector fields. VORTICES (sum of rotational fields),
WAVE (sin(ωy + t), cos(ωx − t)), SADDLE (hyperbolic stagnation),
MAGNET (gravitational dipole), TURBULENT (sum of 20 random mini-
vortices).

### Magnetic Fields

`B = Σᵢ qᵢ · (r − rᵢ) / |r − rᵢ|³` — sum of inverse-square monopole
contributions. Iron-filings-style visualisation with red 'N' / blue
'S' markers. Five pole configurations from 2-pole DIPOLE up to 14-
pole PLASMA.

### Reaction-Diffusion (Gray-Scott)

`∂U/∂t = D_u·∇²U − UV² + F(1−U)`,
`∂V/∂t = D_v·∇²V + UV² − (F+k)V`. Explicit Euler + 5-point
Laplacian + NEUMANN boundary. Pearson 1993 morphology phase diagram
gives 5 named regions: SPOTS / STRIPES / MAZES / CORAL / WORMS.

### Midpoint-Displacement Coastlines (1-D)

The 1-D ancestor of Diamond-Square. Recursively split a line and
displace each midpoint by random jitter. Used in the Star Trek II
Genesis effect (1982). Five silhouette patterns plus smooth
continuous A→B morphing animation.

---

## Shared Conventions

All files in `procedural/` share:

- §1–§8 (or §1–§9) section layout per CLAUDE.md
- 10 standard themes via `t`/`T`: DEFAULT, MATRIX, NOVA, MONO, OCEAN,
  FIRE, EARTH, FOREST, DESERT, ARCTIC
- 5 glyph sets via `g`/`G`: SLIM, LIGHT, MEDIUM (default), HEAVY, FAT
- 5 algorithmic patterns via `n`/`p`
- Reserved HUD colour pairs: PAIR_HUD = bright yellow (226),
  PAIR_HINT = bright cyan (51), both BOLD
- Self-contained — no shared headers, every file inlines its own
  noise / hash primitives
- ASCII-only rendering (per the CLAUDE.md ASCII-Only rule)
- Density-graded rendering: glow `[0, 1]` → `.` (low) / `*` (mid) /
  `#` (high) by default

---

## Cross-references to existing concept docs

Several procedural showcases extend or replace older concept files:

| Showcase | Older concept doc(s) |
|---|---|
| `bsp_dungeon_showcase.c` | `concept_bsp_tree.md` (1-D tree theory) |
| `cellular_automata_cave_4-5_rule_showcase.c` | `concept_cellular_automata_1d.md` (1-D variant) |
| `magnetic_fields.c` | `concept_magnetic_field.md` |
| `maze_backtracker.c`, `wilsons_*.c` | `concept_maze.md` |
| `perin_noise_flow_showcase.c` | `concept_perlin_landscape.md` |
| `reaction_diffusion_gray_scott.c` | `concept_reaction_diffusion.md`, `concept_reaction_wave.md` |
| `delaunay_triangulation.c` | `concept_tri_grids_11_delaunay.md` |
| `voronoi_region_map.c` | `concept_voronoi.md` |

The procedural showcases bring the same algorithms to a higher
production polish level (themes, glyph sets, patterns, animation
states) while the older concept docs typically focus on the math.
Read the math first; then study the showcase for the engineering.

---

## Build pattern (identical across all)

```
gcc -std=c11 -O2 -Wall -Wextra <path>/<file>.c -o <name> -lncurses -lm
```

No special compile flags. No headers. Each file is one self-contained
showcase.

---

*See Master.md §X for per-algorithm essays and references. See
Architecture.md §179 for framework conventions. See COLOR.md for the
themed-palette architecture. See Visual.md "Procedural Showcases"
section for the shared visual techniques (V-P1 through V-P10).*
