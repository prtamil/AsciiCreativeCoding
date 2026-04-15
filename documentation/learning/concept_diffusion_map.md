# Pass 1 — diffusion_map.c: Diffusion-Limited Aggregation

## Core Idea

**Diffusion-Limited Aggregation (DLA)** grows a fractal cluster one cell at a time. A particle is launched from a random point on a circle surrounding the cluster. It performs a random walk until it touches the cluster, then sticks permanently. Repeating this produces dendrite-like branching arms — the same mechanism responsible for snowflake growth, electrodeposition, and coral formation.

The second mode, **Eden growth**, replaces the random walk with uniform random selection from the frontier (any empty cell adjacent to the cluster). Eden growth is faster and produces rounder, less fractal shapes because diffusion bias toward tips is removed.

## The Mental Model

### Why DLA produces fractals

A random walker is more likely to touch the *tips* of existing branches than the *bays* between them, because diffusion particles reach tips first. Tips grow faster than bays, which grow faster than interior concavities. This tip-enhancement is self-amplifying: taller tips intercept more walkers, become taller still.

The resulting structure has **fractal dimension** ≈ 1.71 in 2D — significantly less than 2 (a filled disk). The aggregate is mostly empty space.

### Launch circle and kill radius

Each walker is launched on a circle of radius `g_radius + LAUNCH_PAD` centred on the cluster origin. `g_radius` is the **Chebyshev radius** (max `max(|dr|, |dc|)`) of the current cluster — the smallest bounding square.

A **kill radius** of `2·g_radius + 3·LAUNCH_PAD` discards walkers that diffuse too far from the cluster. Without this, walkers can wander indefinitely in open space, stalling the simulation. The kill radius grows with the cluster so it never prematurely kills valid walkers.

```c
kill_radius = g_radius * 2 + LAUNCH_PAD * 3
```

### Aspect-ratio correction on launch

Terminal cells are ~2× taller than wide. Without correction, walkers launched at `(cx + r·cos θ, cy + r·sin θ)` would produce an elliptical launch cloud — the cluster would grow wider than tall.

The y-component is halved at launch:
```c
r = g_cy + (int)roundf(dist * sinf(angle) * 0.5f);
c = g_cx + (int)roundf(dist * cosf(angle));
```

This makes the circular launch ring appear circular on screen, so growth is isotropic.

### Eden mode and the frontier

In Eden mode, every frame the code scans the entire grid to collect **frontier cells** (empty cells with at least one cluster neighbour), then picks one uniformly at random. This is O(rows×cols) per step but terminates instantly — no random walk.

Eden clusters grow rounder because every frontier cell has equal probability regardless of its position relative to the cluster geometry. There is no diffusion gradient favouring tips.

### Age-gradient coloring

Each cell records the frame number when it joined (`g_age`). Every frame, `age_delta = g_frame - g_age[r][c]`. Newer cells are brighter and use bolder characters:

```
age_delta ≤  5    '@' bold   (CP_A0 — newest)
         ≤ 20    '#'         (CP_A1)
         ≤ 80    '+'         (CP_A2)
         ≤ 300   ':'         (CP_A3)
         > 300   '.'         (CP_A4 — oldest)
```

This creates a living gradient: the growing tips glow bright while old interior cells fade to dim characters.

## Data Structures

```c
uint8_t  g_grid[ROWS_MAX][COLS_MAX]; /* 0=empty, 1=cluster */
uint16_t g_age [ROWS_MAX][COLS_MAX]; /* frame when cell joined (mod 65536) */
int g_cx, g_cy;       /* grid center */
int g_radius;         /* Chebyshev radius of cluster */
int g_frame;          /* simulation frame counter */
int g_cluster_size;   /* total cells in cluster */
```

## The Main Loop

```
grid_reset():
    memset g_grid, g_age to 0
    place seed at (g_cy, g_cx), g_radius = 0

scene_tick() per frame:
    g_frame++
    if eden_mode:
        for n_walkers: eden_step()
    else:
        for n_walkers: dla_walker_run()

dla_walker_run():
    angle = lcg_f() * 2π
    launch at (cy + dist·sin·0.5, cx + dist·cos)    ← aspect-corrected
    for step in 0..MAX_STEPS-1:
        if out of bounds: return false
        if Chebyshev distance > kill_radius: return false
        if has cluster neighbour and cell is empty:
            grid_add_cell(r, c); return true
        r += DR4[lcg_i(4)];  c += DC4[...]           ← random walk step

eden_step():
    collect all frontier cells into arrays fr[], fc[]
    idx = lcg_i(n)
    grid_add_cell(fr[idx], fc[idx])
```

## Non-Obvious Decisions

### Why `uint16_t` for age and frame mod 65536

Age is recorded as `(uint16_t)(g_frame & 0xFFFF)`. If the simulation runs long enough for `g_frame` to exceed 65535, ages wrap. The `age_delta` computation `g_frame - (int)g_age[r][c]` uses signed arithmetic and remains correct as long as `age_delta < 32768`. At 30 fps this allows ~18 minutes of continuous growth before any artefact — sufficient in practice.

### Why MAX_STEPS = 500

A walker that wanders 500 steps without sticking is either too far from the cluster or in a dead-end pocket. Aborting it and starting a fresh walker is statistically equivalent — the aborted walk contributes nothing to the cluster shape. Too few steps would bias growth (walkers near the cluster always contribute, far walkers never do). Too many steps wastes CPU.

### Why Chebyshev radius rather than Euclidean

Chebyshev distance `max(|dr|, |dc|)` is the natural grid metric: it equals the number of steps to reach a cell in 8-direction movement. It is cheaper to compute than `sqrt(dr²+dc²)` and gives a tight bounding square, which is the correct shape for the terminal grid.

### Eden frontier scan is O(rows×cols) per call

With n_walkers=10 this is 10× full-grid scans per frame. At 80×300 = 24,000 cells this is 240,000 comparisons per frame — fast enough at 30 fps. A more efficient approach would maintain a frontier list, but the simple scan is easier to reason about and fits the grid size.

## From the Source

**Algorithm:** Two aggregation modes. DLA (Diffusion-Limited Aggregation, Witten & Sander 1981): Particles launched from a ring, random-walk until they touch the cluster. Tip-screening effect: tips extend further from the centre and capture walkers preferentially, creating fractal branching with D ≈ 1.7 in 2D. Eden model: directly attach a random frontier cell. No diffusion → no tip screening → compact, rounder shapes (D → 2 as cluster grows; no fractal structure at large scales).

**Math:** DLA fractal dimension D ≈ 1.71 in 2D. Cluster radius `R ~ N^(1/D)` where N = number of particles. Comparison between modes in the same code illustrates how diffusion (randomness in the approach path) is necessary for fractal self-similar morphology.

**Performance:** DLA walker cost: O(R²) expected random-walk steps per particle (hitting probability from radius 2R to R ≈ 1/(log R) in 2D). Eden mode: O(frontier size) per particle — much faster. MAX_STEPS=500 per walker; LAUNCH_PAD=3 launch circle offset. Chebyshev radius used for bounding computation.

**References:** Witten, T.A. & Sander, L.M. (1981) — original DLA paper.

## Open Questions for Pass 3

- Measure the **fractal dimension** by box-counting: count how many boxes of side L are needed to cover the cluster, for L = 1, 2, 4, 8... Should converge to ≈1.71 for true DLA.
- What happens with **8-direction adjacency** instead of 4? The cluster should be denser and rounder (more frontier cells per boundary cell).
- Can you **visualise the phi field** during growth, even approximately? The gradient of hit probability from walker density would show the diffusion field the cluster is growing through.
- Compare the **tip velocity** between DLA and Eden at the same frame count. DLA tips should advance faster because they intercept more walkers.
