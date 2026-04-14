# Pass 1 — barnes_hut.c: Barnes–Hut O(N log N) Gravity Tree

## Core Idea

N-body gravity naively requires O(N²) force computations — every body pulls on every other body. The **Barnes–Hut algorithm** reduces this to O(N log N) by grouping distant bodies into a **quadtree** and treating each distant group as a single point mass located at its **centre of mass**. For 400 bodies this is roughly 400 × log₂(400) ≈ 3500 operations vs 400 × 400 = 160,000.

## The Mental Model

### The Opening Angle Criterion

The key question the algorithm asks at every quadtree node is:

> *Is this group of bodies close enough to matter individually, or far enough that I can lump them together?*

The answer is the **opening angle criterion**:

```
s / d  <  θ   →  treat node as a single point mass
s / d  ≥  θ   →  recurse into children
```

Where:
- `s` = side length of the quadtree node's bounding box
- `d` = distance from the body being updated to the node's centre of mass
- `θ` = accuracy threshold (θ=0.5 is standard; lower = more accurate, slower)

A small distant cluster (small s/d) is well approximated by a single mass. A large nearby cluster (large s/d) must be broken down further.

### Quadtree Structure

The space is recursively subdivided into four quadrants: NW, NE, SW, SE.

```
 NW | NE
 ───┼───
 SW | SE
```

Each node stores:
- Bounding box `x0, y0, x1, y1`
- `total_mass` and `cx, cy` — centre of mass of all bodies in this subtree
- `child[4]` — indices into the static node pool (-1 if absent)
- `body_idx` — if ≥0, this is a leaf holding exactly one body

The tree is **rebuilt from scratch every tick** — no incremental updates. With a static node pool (array, no malloc) this is fast: `g_pool_top = 0` resets it in one instruction.

### Tree Build: Incremental Centre of Mass

When inserting body `b` into node `n`:

```c
float new_mass = n->total_mass + b->mass;
n->cx = (n->cx * n->total_mass + b->px * b->mass) / new_mass;
n->cy = (n->cy * n->total_mass + b->py * b->mass) / new_mass;
n->total_mass = new_mass;
```

This **incremental COM update** runs on every node from root to leaf as the body is inserted. By the time the tree is fully built, every internal node's `cx, cy` correctly reflects the weighted average position of all bodies in its subtree.

### Force Traversal

```c
static void qt_force(int ni, int bi, float *fx_out, float *fy_out)
{
    QNode *n = &g_pool[ni];
    
    /* Skip self at leaf */
    if (n->body_idx == bi) return;

    float dx = n->cx - b->px;
    float dy = n->cy - b->py;
    float d  = sqrt(dx*dx + dy*dy + SOFT2);
    float s  = n->x1 - n->x0;          /* node side length */

    if ((s / d) < THETA_DEF || n->child[0] < 0) {
        /* Treat entire node as one point mass */
        float inv = G * n->total_mass / (d² * d);
        *fx_out += inv * dx * b->mass;
        *fy_out += inv * dy * b->mass;
        return;
    }
    /* Recurse into all 4 children */
    for (int c = 0; c < 4; c++)
        qt_force(n->child[c], bi, fx_out, fy_out);
}
```

The **self-force skip** (`body_idx == bi`) is critical: the leaf node's centre of mass IS the body itself, so d→0 would explode the force. By skipping it we avoid both the singularity and the unphysical self-attraction.

**What about internal nodes that include the body's own mass?** When an internal node satisfies `s/d < θ`, `total_mass` includes the body's own mass. The error is `O(m_i / M_total)` — for 400 equal-mass bodies this is 0.25%. Negligible, and standard practice in Barnes–Hut implementations.

### Softening

```c
float d2 = dx*dx + dy*dy + SOFT2;   /* SOFT2 = 12² = 144 px² */
float d  = sqrtf(d2);
float inv = G * M / (d2 * d);        /* = G*M / d³ */
```

Without softening, two bodies passing at d→0 experience force→∞, sending them to infinity in one tick. Softening `ε` sets a minimum effective distance, preventing singularities while barely affecting the long-range field (since for d >> ε, the ε² term is negligible).

## Galaxy Preset Physics

### Flat Rotation Curve

Real galaxy discs have approximately **flat rotation curves**: orbital velocity v is nearly constant at all radii. This requires enclosed mass to grow linearly with radius:

```
M_enc(r) = M_total × (r / R)
v_circ   = sqrt(G × M_enc / r)
          = sqrt(G × M_total / R)   ← constant!
```

The code initialises disk bodies with this circular velocity: `v0 = sqrt(G * M_tot / R)`. Inner bodies orbit faster (shorter period), outer bodies slower — this **differential rotation** naturally shears the initial spiral arms over time, which is physically correct.

### Box-Muller for Bulge

The central bulge uses a Gaussian spatial distribution. The Box-Muller transform generates a standard-normal sample from two uniform random numbers:

```c
static float box_muller(void) {
    float u1 = rng_f() + 1e-7f;
    float u2 = rng_f();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * M_PI * u2);
}
```

`u1 + 1e-7f` prevents `log(0)`. The result is multiplied by `sigma = R × BULGE_SIGMA = R × 0.07` to set the bulge half-width.

### Logarithmic Spiral Arms

```c
float theta = arm_offset + WINDING * logf(r / r_min) + scatter;
```

A logarithmic spiral satisfies `θ = k × ln(r)` — it maintains a constant angle between the tangent and the radius. `WINDING=0.9` controls tightness; higher = tighter wrap. Scatter `± 0.25 rad` prevents a perfectly thin arm and gives a realistic diffuse appearance.

## Brightness Accumulator

Rather than drawing individual body glyphs, each frame:

1. Each active body adds `1.0` to `g_bright[row][col]`
2. All cells are normalised by `b_max`
3. Brightness maps to character ramp: `. : o O @`
4. All cells decay by `DECAY = 0.84` per frame

```
decay: 0.84^30 ≈ 0.004 — after 30 frames a body that left a cell has < 1% of its original brightness
```

This gives a **persistent glow** that fades over ~30 frames, making orbital paths visible and clusters look nebular. The normalisation `norm = b / b_max` ensures the brightest cell always shows `@` regardless of absolute body count.

## Static Node Pool

```c
static QNode g_pool[NODE_POOL_MAX];   /* 16000 nodes */
static int   g_pool_top = 0;

static int qt_alloc(...) {
    if (g_pool_top >= NODE_POOL_MAX) return -1;
    return g_pool_top++;
}
```

Reset: `g_pool_top = 0` — this "frees" all nodes in O(1) by simply resetting the counter. No `malloc`, no `free`, no fragmentation. Maximum nodes used: for N bodies, the tree has at most `4N` nodes (each insertion creates at most 4 new nodes during subdivision). For N=800 this is ≤3200 — well within 16000.

## Data Structures

```c
typedef struct {
    float px, py;   /* position (pixels)     */
    float vx, vy;   /* velocity (pixels/tick) */
    float mass;
    bool  active;
} Body;

typedef struct {
    float x0, y0, x1, y1;   /* bounding box                   */
    float total_mass;        /* sum of all bodies in subtree   */
    float cx, cy;            /* centre of mass                 */
    int   child[4];          /* -1 or pool index               */
    int   body_idx;          /* ≥0 if leaf, else -1            */
    int   depth;
} QNode;

static Body  g_bodies[N_BODIES_MAX];           /* 800 bodies     */
static QNode g_pool[NODE_POOL_MAX];            /* 16000 nodes    */
static float g_bright[GRID_ROWS_MAX][GRID_COLS_MAX];  /* glow grid */
```

## Main Loop

```
per tick:
    g_pool_top = 0                     ← reset pool in O(1)
    qt_build() → insert all bodies     ← O(N log N)
    for each body i:
        fx=0, fy=0
        qt_force(root, i, &fx, &fy)    ← O(log N) per body
        vx += (fx/mass)*dt
        vy += (fy/mass)*dt
        px += vx*dt
        py += vy*dt

per render frame:
    for each body: g_bright[row][col] += 1.0
    normalise by b_max
    render ". : o O @" ramp
    decay all cells × 0.84
    if overlay: draw quadtree midlines (depth ≤ 3)
```

## Non-Obvious Decisions

### Why rebuild the tree every tick?

Incremental updates (insert moved body, rebalance) are complex and error-prone. Rebuilding from scratch with a static pool takes ~N×log(N) pool allocations — with 800 bodies this is ~7200 pool ops per tick, each being a simple array write. At 60 Hz this is ~432,000 array writes per second — negligible on modern hardware.

### Why depth limit at QT_MAX_DEPTH=32?

If two bodies have nearly identical positions, recursive subdivision would continue until one quadrant has side length < 1 pixel, potentially reaching depth 50+. The depth cap prevents stack overflow in qt_insert while allowing any realistic separation.

### Why symplectic Euler instead of velocity Verlet?

Velocity Verlet is 2nd-order (nbody.c uses it). For Barnes–Hut with many bodies, the approximation error from the θ criterion (≈0.5% per step) dominates over the integration error. Symplectic Euler (1st-order but energy-conserving in the long term) is sufficient and simpler — one force evaluation per tick instead of two.

### Overlay at depth ≤ 3

The quadtree at depth 0 is one rectangle (the full screen). Depth 1 = 4 quadrants. Depth 2 = 16 cells. Depth 3 = 64 cells. At depth 4 (256 cells) the lines are so dense they obscure the bodies. The `min 2×2 cells` guard prevents drawing a line inside a single-cell node where it would overwrite body characters.
