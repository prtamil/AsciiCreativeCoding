# Pass 1 — barnsley.c: IFS Chaos Game

## Core Idea

An **Iterated Function System (IFS)** is a finite set of affine contractions. Repeatedly applying one randomly chosen transform — with probability proportional to the transform's weight — generates a point sequence whose density converges to the IFS **attractor**. This is the *chaos game*, introduced by Barnsley (1988).

The attractor is a fractal: a self-similar set that looks the same at any scale, because the set is the union of shrunken copies of itself under each transform.

## The Mental Model

### What an affine transform does

```
x' = a·x + b·y + e
y' = c·x + d·y + f
```

The 2×2 matrix `[[a,b],[c,d]]` handles rotation, scaling, and shearing. The translation vector `(e,f)` shifts the result. For the IFS to have a bounded attractor, every transform must be a **contraction** — its matrix must have singular values < 1, so it shrinks space.

### Why random selection converges to the attractor

The Banach fixed-point theorem guarantees that any contractive system has a unique fixed set. The random orbit visits every neighbourhood of the attractor with probability proportional to the density of visits in the long run. Starting from nearly any point (except the exact fixed point of one of the transforms), the orbit reaches the attractor exponentially fast.

The first ~20 iterations are discarded (warm-up) to move the orbit off the transient before recording hits.

### Why different IFS produce different shapes

| Preset | Key property |
|--------|-------------|
| Barnsley Fern | One near-degenerate transform (prob 0.01) maps everything to the stem; others grow leaflets |
| Sierpinski | Three transforms each scale by 0.5 toward one corner of a triangle |
| Levy C Curve | Two transforms rotate ±45° and scale by √2/2 |
| Dragon Curve | Two transforms; second has negative determinant (reflection) |
| Fractal Tree | Four transforms: one stem, two symmetric branches, one vertical extension |

### Log-density rendering

A `uint16_t` hit accumulator counts visits per terminal cell. Raw counts are highly skewed (stem visited far more than leaf tips), so a direct linear map would leave most of the attractor invisible.

**Log-normalisation:**
```c
t = log1p(hits[r][c]) / log1p(max_hits)
```

`log1p` avoids `log(0)` for zero-hit cells. The result `t ∈ [0,1]` is perceptually uniform — both low-density regions (tips) and high-density regions (stem) are visible simultaneously.

**Density tiers:**
```
t < 0.15            background (skip)
0.15 ≤ t < 0.35     '.'  (CP_L1)
0.35 ≤ t < 0.55     ':'  (CP_L2)
0.55 ≤ t < 0.75     '+'  (CP_L3)
0.75 ≤ t            '@'  (CP_L4, bold)
```

### LCG random number generator

The LCG (`g_lcg * 1664525 + 1013904223`) produces a uniform integer. Dividing by 2²⁴ gives a float in [0,1). It is seeded from `CLOCK_MONOTONIC` nanoseconds so each run differs.

Transform selection uses cumulative probability: scan transforms in order, stop when `rnd ≤ cum[k]`. The last transform has `cum = 1.0` and catches all rounding error.

## Data Structures

```c
typedef struct {
    float a, b, c, d, e, f;  /* affine coefficients */
    float cum;                /* cumulative selection probability */
} Transform;

typedef struct {
    const char *name;
    int         n_transforms;
    Transform   tf[4];
    float       x_min, x_max, y_min, y_max;   /* attractor view extents */
} Preset;

uint16_t g_hits[GRID_ROWS_MAX][GRID_COLS_MAX]; /* hit accumulator */
float    g_cx, g_cy;                           /* current orbit position */
```

## The Main Loop

```
grid_reset():
    memset g_hits to 0
    g_cx = 0.1, g_cy = 0.0   (near-origin, away from any single fixed point)

chaos_step() per frame:
    for i in 0 .. g_iters-1:
        rnd = lcg_f()
        select transform tf with rnd ≤ tf.cum
        (cx, cy) = tf.apply(cx, cy)
        col = (cx - x_min) / x_range * (cols - 1)
        row = 1 + (y_max - cy) / y_range * (draw_rows - 1)  ← y flipped (screen y down)
        if in bounds: g_hits[row][col]++  (capped at HITS_CAP=60000)

render_grid():
    max_hits = max(g_hits)
    for each cell: t = log1p(h) / log1p(max_hits) → pick char and color pair
```

## Non-Obvious Decisions

### Why cap hits at 60,000

`uint16_t` overflows at 65,535. Capping at 60,000 leaves headroom and prevents visual artefacts from wrap-around. After millions of iterations the stem of the fern would otherwise overflow.

### Why y is flipped in the grid mapping

`row = 1 + (y_max - cy) / y_range * draw_rows`

Screen rows increase downward; IFS coordinates have y increasing upward. The `y_max - cy` term inverts the direction so the fern stands upright rather than being flipped.

### Why start at (0.1, 0.0) not (0, 0)

`(0, 0)` is the fixed point of the first Barnsley Fern transform (the stem transform). Starting there would keep the orbit on the stem for many iterations, producing an artificially dark stem on reset. A nearby non-fixed point reaches the full attractor faster.

### Why `g_iters` per frame rather than one iteration per frame

One iteration per frame at 30 fps would take minutes to fill the attractor. Running 8,000 iterations per frame fills the fern visibly in about 1 second and continues building density smoothly.

## From the Source

**Algorithm:** Chaos game (IFS attractor via random iteration). Rather than recursively subdividing regions, the attractor is found by iterating: pick a random transform, apply it to the current point, plot the result. After discarding the first few transient iterates (burn-in), the orbit is on the attractor. Due to Barnsley's theorem: any IFS with a contractivity condition < 1 has a unique compact attractor.

**Math:** Each transform is an affine map `T_i(x,y) = A_i·[x,y]ᵀ + b_i` where A_i is a 2×2 matrix. The probability p_i of choosing transform i should be proportional to `|det(A_i)|` for uniform density across the attractor parts. For the Barnsley fern, stem (T₁, tiny |det|) needs only p₁=1% while main leaflets (T₂, |det|≈0.85) need p₂=85%.

**Performance:** Density grid used rather than drawing individual points — accumulates hit counts per cell and log-normalises for display. LCG random number generator (`g_lcg * 1664525 + 1013904223`) with cap at HITS_CAP=60,000 (uint16_t headroom). ITERS_DEFAULT=8000 per frame fills the fern visibly in about 1 second.

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `Transform` (struct) | `typedef struct` | 28 B | affine coefficients a–f plus cumulative probability `cum` |
| `Preset` (struct) | `typedef struct` | ~120 B | name, n_transforms, 4 transforms, view extents |
| `g_hits[GRID_ROWS_MAX][GRID_COLS_MAX]` | `uint16_t[]` | ~48 KB | hit accumulator; capped at HITS_CAP=60,000 |
| `g_cx`, `g_cy` | `float` | scalar | current orbit position in IFS coordinate space |
| `g_lcg` | `uint32_t` | scalar | LCG RNG state; advances each iteration |
| `GRID_ROWS_MAX`, `GRID_COLS_MAX` | constants | N/A | maximum grid size (80 × 300) |

## Open Questions for Pass 3

- What is the **Hausdorff dimension** of each attractor? The Barnsley fern's theoretical value is ≈1.86. Measure it by box-counting on the hit grid.
- The **Collage Theorem** says you can approximate any shape by finding an IFS whose attractor matches it. Can you add a preset for a custom letter or shape by hand-tuning transforms?
- What happens if `n_transforms` is increased beyond 4? Can the IFS produce a Koch snowflake or Cantor set with the current transform structure?
- The LCG period is 2³² ≈ 4 billion. Does hitting the period boundary cause a visible artefact in the density grid? (It would show as a brief density reset.)
