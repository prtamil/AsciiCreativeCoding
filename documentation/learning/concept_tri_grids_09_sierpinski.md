# Concept: Sierpinski Triangle (3-Way Midpoint Subdivision)

## Pass 1 — Understanding

### Core Idea
The classic Sierpinski gasket (Sierpiński 1915). Same midpoint subdivision as `08_triforce.c`, but the inverted CENTRE child is dropped. Only the three corner children survive each step, giving 3^N leaves at depth N. The limit shape has Hausdorff dimension `log₂3 ≈ 1.585`, area zero, and infinite perimeter.

### Subdivision Step
Identical setup as triforce, then KEEP three of four:
```
M₀₁ = (V₀ + V₁) / 2,  M₁₂ = (V₁ + V₂) / 2,  M₂₀ = (V₂ + V₀) / 2

subdivide(V₀,  M₀₁, M₂₀, depth − 1)        // corner at V₀
subdivide(M₀₁, V₁,  M₁₂, depth − 1)        // corner at V₁
subdivide(M₂₀, M₁₂, V₂,  depth − 1)        // corner at V₂
// (M₁₂, M₂₀, M₀₁) — inverted centre — DROPPED
```

### Hausdorff Dimension
Each step replaces 1 triangle with 3 self-similar copies at scale 1/2.
```
N · r^d = 1
3 · (1/2)^d = 1
d = log 3 / log 2 ≈ 1.585
```
This is greater than 1 (the dimension of a curve) and less than 2 (the dimension of an area), characteristic of a fractal.

### Why This Works As Animation
At each finite depth the figure has 3^N triangles totalling area `(3/4)^N` of the parent. Convergence is geometric — depth 9 gives 3^9 = 19683 leaves; the gaps between leaves become visible in ASCII at depth 5 or so on a typical 80×24 terminal.

### Connection to Other Files
- `08_triforce.c` — same split, keeps 4 of 4 children. 4^N leaves vs 3^N.
- `07_barycentric.c` — 6-way split via centroid. 6^N leaves.

### Non-Obvious Decisions
- **One line of difference** vs `08_triforce.c`: skip the recursive call on `(M₁₂, M₂₀, M₀₁)`.
- **Depth cap 9**: 3^9 = 19683 — well within reasonable rendering time. At depth 12 the leaves fall below 1-pixel resolution on small terminals.
- **Pascal's triangle mod 2 connection**: the cells of Pascal's triangle that are ODD form a Sierpinski pattern when viewed as a triangle. This file's geometric subdivision yields the same set in the limit.

### Key Constants
| Name | Role |
|------|------|
| `DEPTH` | Recursion depth (0..9) |
| `SIZE_FRAC` | Seed triangle size relative to screen |

### Open Questions
- What is the Lebesgue measure of the limit set? (Answer: zero — area vanishes.)
- How does the IFS (iterated function system) representation compare to this recursive geometry?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — DEPTH, SIZE_FRAC
§3 color     — depth-keyed palette
§4 formula   — slope_char + Bresenham line_draw (same as 07/08)
§5 subdivide — recursive 3-way split (drops centre child)
§6 scene     — seed_triangle + scene_draw
§7 screen    — ncurses init / cleanup
§8 app       — signals, main loop
```

### Data Flow
```
seed_triangle → subdivide(depth) → 3^depth leaves
                          ↓
              line_draw 3 edges per leaf
```
