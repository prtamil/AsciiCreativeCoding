# Concept: Recursive Barycentric Subdivision

## Pass 1 — Understanding

### Core Idea
Take one big equilateral triangle and recursively split it into 6 smaller triangles by computing the centroid plus the three edge midpoints, then connecting them. At depth N there are 6^N leaf triangles. This is the classical "barycentric subdivision" from algebraic topology.

### Subdivision Step
Given triangle (V₀, V₁, V₂):
```
C    = (V₀ + V₁ + V₂) / 3       // centroid
M₀₁ = (V₀ + V₁) / 2              // edge midpoints
M₁₂ = (V₁ + V₂) / 2
M₂₀ = (V₂ + V₀) / 2
```
Emit six children:
```
(C, V₀, M₀₁), (C, M₀₁, V₁),
(C, V₁, M₁₂), (C, M₁₂, V₂),
(C, V₂, M₂₀), (C, M₂₀, V₀)
```
Each child is a 30-60-90 right triangle similar to the kisrhombille sub-triangle from `04_30_60_90.c`.

### Recursion
```
subdivide(V₀, V₁, V₂, depth):
    if depth == 0:
        emit_leaf(V₀, V₁, V₂)        // draw the triangle's edges
    else:
        compute C, M₀₁, M₁₂, M₂₀
        for each of the 6 children:
            subdivide(child, depth − 1)
```
Stack-only recursion; no dynamic allocation. Drawing is done by Bresenham `line_draw` between leaf vertices.

### Why "Barycentric"?
The centroid (barycentre) is the affine-weighted average of the three vertices with equal weights. The new vertices added at each step are barycentres of subsets of the original — single-vertex (V_i), edge-midpoint pair (M_ij), and full-triangle centroid (C).

### Connection to Other Files
- `08_triforce.c` — 4-way midpoint split (no centroid). 4^N leaves.
- `09_sierpinski.c` — same midpoint split, drop the centre child. 3^N leaves.
- `04_30_60_90.c` — single-level barycentric subdivision applied to the equilateral tiling.

### Non-Obvious Decisions
- **6-way not 7-way**: the centroid triangle (M₀₁, M₁₂, M₂₀) is NOT emitted as a leaf — that would give the triforce. Barycentric subdivision uses C as the seventh point and emits 6 triangles around it.
- **Stack-only**: max recursion depth is small (≤ 5 in the demo) so a few KB of stack is plenty.
- **Bresenham line draw**: each leaf draws its 3 edges as ASCII line segments via slope-character lookup.

### Key Constants
| Name | Role |
|------|------|
| `DEPTH` | Recursion depth (0..5 in the UI) |
| `SIZE_FRAC` | Seed triangle size as a fraction of screen min dimension |

### Open Questions
- What is the Hausdorff dimension of the limit set as depth → ∞? (Answer: 2 — the subdivision fills the parent.)
- How would you flood-fill the resulting tessellation by colour?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — DEPTH, SIZE_FRAC
§3 color     — depth-keyed palette (deeper = different colour)
§4 formula   — slope_char + Bresenham line_draw
§5 subdivide — recursive 6-way split, leaf emitter
§6 scene     — seed_triangle + scene_draw
§7 screen    — ncurses init / cleanup
§8 app       — signals, main loop
```

### Data Flow
```
seed_triangle(V₀, V₁, V₂) → subdivide(depth) → 6^depth leaves
                                        ↓
                              line_draw V₀→V₁, V₁→V₂, V₂→V₀  per leaf
```
