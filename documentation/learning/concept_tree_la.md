# Pass 1 — tree_la.c: Dielectric Breakdown Model (DBM)

## Core Idea

The **Dielectric Breakdown Model** (Niemeyer, Pietronero, Wiesmann 1984) grows a fractal tree by solving the **Laplace equation** for electric potential φ in the region outside the growing conductor, then selecting the next cell to grow with probability proportional to `φ^η`.

- Large η concentrates growth at high-φ tips → jagged, sparse lightning
- Small η spreads growth evenly → rounder Eden-like blobs
- The cluster is a grounded conductor (φ = 0); boundary conditions drive φ to 1 at the source

## The Mental Model

### The physics analogy

Imagine the growing tree is a grounded electrical conductor inside a box. The box boundary is held at φ = 1 (or one wall is, depending on the preset). The Laplace equation `∇²φ = 0` describes the equilibrium potential field in the empty space between the tree and the boundary.

The electric field **E** = −∇φ is strongest at the tips of the conductor (where φ drops fastest from 1 to 0 over the smallest distance). In a real dielectric breakdown event, the material ruptures where the field is strongest — the tip grows. DBM models this probabilistically: P(cell grows) ∝ φ^η.

### Gauss-Seidel relaxation

The Laplace equation `φ[r][c] = 0.25*(φ[r-1][c] + φ[r+1][c] + φ[r][c-1] + φ[r][c+1])` is solved iteratively. Each pass updates every non-tree cell to the average of its four neighbours. This converges to the true harmonic solution as passes → ∞.

`N_RELAX = 8` passes per frame is a compromise: enough to keep φ reasonably accurate after each new cell is added, not so many that the frame rate drops. After a cell is added, it becomes a new grounded point and the field around it must re-equilibrate.

**Boundary conditions enforced after each pass:**
```c
phi_enforce_bc():
    preset-specific edges set to 0.0 or 1.0
    tree cells forced to 0.0  (conductor)
```

The initial φ is a linear gradient matching the expected solution for the chosen preset — this dramatically accelerates convergence from the very first frame.

### Growth selection: weighted random scan

Each frame, after relaxation, one frontier cell is added:

```c
total = Σ φ[r][c]^η   for all frontier cells
pick  = lcg_f() * total
scan frontier cells, accumulate cumulative sum
stop when cum ≥ pick → that cell is selected
```

This is O(rows×cols) per grow step (same as Eden mode in diffusion_map.c), which is acceptable for the small number of grow steps per frame (N_GROW = 1).

### The three presets

| Preset | Seed | Source boundary | Shape |
|--------|------|-----------------|-------|
| Tree | Bottom-centre | Top row φ=1, bottom row φ=0 | Upward-growing tree (lightning rod) |
| Lightning | Top-centre | Bottom row φ=1, top row φ=0 | Downward-striking lightning bolt |
| Coral | Centre | All 4 edges φ=1 | Radially symmetric coral/snowflake |

Neumann (zero-gradient) conditions on the open edges (left/right for Tree/Lightning) let the field vary freely there, allowing branches to spread horizontally.

### The η parameter

```
η = 1.0  →  P ∝ φ         (DLA-like, all high-φ cells equally likely)
η = 2.0  →  P ∝ φ²        (tips increasingly favoured)
η = 4.0  →  P ∝ φ⁴        (almost always the single highest-φ cell)
```

At η → ∞ the model selects the maximum-φ frontier cell deterministically, producing a single unbranching needle. At η = 0 every frontier cell has equal probability (Eden growth).

Default values: Tree=1.5, Lightning=2.5, Coral=2.0.

### Age-gradient coloring

Identical to diffusion_map.c: `age_delta = g_step - g_age[r][c]` maps to 5 age levels (CP_A0 newest '@' bold, CP_A4 oldest '.'). The growing tip glows white while older structure fades to dim characters.

## Data Structures

```c
uint8_t  g_tree[ROWS_MAX][COLS_MAX]; /* 1 if in tree/cluster */
float    g_phi [ROWS_MAX][COLS_MAX]; /* electric potential [0,1] */
uint16_t g_age [ROWS_MAX][COLS_MAX]; /* growth step when joined */
int      g_step;       /* growth step counter */
int      g_tree_size;  /* total cells in tree */
Preset   g_preset;
float    g_eta;
```

## The Main Loop

```
scene_tick() per frame:
    phi_relax()         ← N_RELAX Gauss-Seidel passes + BC enforcement
    tree_grow()         ← add N_GROW cells by weighted selection

phi_relax():
    for pass in 0..N_RELAX-1:
        for each non-tree interior cell:
            phi[r][c] = 0.25*(phi[r-1][c]+phi[r+1][c]+phi[r][c-1]+phi[r][c+1])
        phi_enforce_bc()

tree_grow():
    total = Σ_frontier phi[r][c]^eta
    pick  = lcg_f() * total
    scan frontier cells, accumulate → select cell
    g_tree[sel_r][sel_c] = 1
    g_phi [sel_r][sel_c] = 0.0   ← immediately ground the new cell
    g_age [sel_r][sel_c] = g_step
    g_step++
```

## Non-Obvious Decisions

### Why phi is immediately zeroed after selection

After adding a new tree cell, `g_phi[sel_r][sel_c] = 0.0` before the next relaxation. This is correct: the new cell is a conductor (φ = 0 by definition). Without zeroing it, the next pass would average it with its neighbours, producing a φ > 0 conductor cell and a wrong field. The Gauss-Seidel skip condition `if (g_tree[r][c]) continue` then keeps it at 0 for subsequent passes.

### Why the initial phi matters

Cold-starting φ = 0 everywhere (except boundary conditions) would require hundreds of Gauss-Seidel passes to build the gradient. A good initial guess (linear ramp for Tree/Lightning, radial ramp for Coral) converges in just 8 passes/frame from the very first frame — the tree starts growing immediately rather than after a warm-up period.

### Neumann (copy) boundary conditions on open edges

```c
g_phi[r][0]          = g_phi[r][1];
g_phi[r][g_cols - 1] = g_phi[r][g_cols - 2];
```

This sets the derivative dφ/dx = 0 at the left/right walls, meaning the field "exits" perpendicular to those walls with no reflection. Branches near the edges grow naturally rather than being repelled by an artificial φ = 1 or φ = 0 wall.

### N_GROW = 1, not higher

Adding one cell per frame lets the Gauss-Seidel relaxation keep up: 8 passes is sufficient to re-equilibrate the field after a single new grounded point is added near the existing tree. Adding 10 cells per frame would require proportionally more relaxation passes to stay accurate, and the growth pattern would be less fractal (more Eden-like) because the field would lag behind the actual tree shape.

## Open Questions for Pass 3

- At what value of η does the tree transition from fractal to needle-like? Measure branch count vs η.
- How do N_RELAX passes affect the fractal dimension? More passes → more accurate field → stronger tip enhancement. Does this change the Hausdorff dimension?
- The DBM on a hexagonal grid should produce 6-fold symmetric structures (like snowflakes). How would you extend the adjacency and BC code for a hex grid?
- Can you visualise φ directly as a background gradient beneath the tree? This would show the "energy landscape" the tree is growing through.
