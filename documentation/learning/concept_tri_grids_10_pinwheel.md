# Concept: Pinwheel-Style 5-Way Substitution

## Pass 1 — Understanding

### Core Idea
A right triangle with legs 1 and 2 (hypotenuse √5) is recursively split into 5 sub-triangles. The split combines the standard 4-way midpoint subdivision (as in `08_triforce`) with one extra cut: the inverted centre child is bisected by its altitude from the right-angle vertex. Leaf count grows as 5^N. This is the same SHAPE as Conway-Radin's pinwheel tiling but uses a different (simpler) substitution; the comment in §5 documents the deviation.

### Why a 1-2-√5 Triangle?
The Conway-Radin pinwheel uses a right triangle with legs 1 and 2. Its hypotenuse √5 is irrational; this irrationality is what produces an aperiodic tiling — the substitution rule rotates each child by an irrational angle (`atan(1/2)`) of the parent's orientation, so no orientation ever repeats.

### Substitution Step
```
1. midpoints  M₀₁ = (V₀+V₁)/2,  M₁₂ = (V₁+V₂)/2,  M₂₀ = (V₂+V₀)/2
2. emit corners  (V₀, M₀₁, M₂₀), (M₀₁, V₁, M₁₂), (M₂₀, M₁₂, V₂)
3. inverted centre = (M₁₂, M₂₀, M₀₁) — SAME SHAPE as parent
4. find right-angle vertex of inverted centre, drop a perpendicular F
   to the opposite edge → bisect that triangle into 2
5. emit those 2 sub-triangles
```
Total: 3 + 2 = 5 children.

### Foot of Perpendicular
Given a right triangle with right angle at vertex `R` and opposite edge endpoints `(A, B)`:
```
t = ((R−A) · (B−A)) / |B−A|²
F = A + t·(B−A)
```
`F` is the foot of the altitude from `R`. The two new triangles are `(R, A, F)` and `(R, F, B)`.

### Aperiodicity
The substitution mixes integer-ratio midpoints with the perpendicular foot. The resulting child orientations include angles like `atan(1/2) ≈ 26.565°`, irrational with respect to π. After many substitutions, no two leaves share the same orientation, so the tiling cannot be periodic.

### Connection to Other Files
- `08_triforce.c` — the underlying 4-way midpoint split.
- `12_penrose.c` — another aperiodic substitution (golden-ratio splits, 2 prototiles).

### Non-Obvious Decisions
- **5-way not 4-way**: the bisection of the inverted centre is what makes this approach distinct. Without it you would have plain triforce.
- **Documented deviation**: the strict Conway-Radin pinwheel uses a slightly different child-orientation scheme that produces a tiling with exactly 5 prototiles per substitution. This file uses a simpler combinatorics that still produces the visual character of a pinwheel.
- **Right-angle vertex picking**: identified by finding the vertex with the smallest squared edge product of incident sides — robust to ordering.

### Key Constants
| Name | Role |
|------|------|
| `DEPTH` | Recursion depth (0..6) |
| `SIZE_FRAC` | Seed triangle size as a fraction of screen |

### Open Questions
- Why does √5 specifically produce aperiodicity? (Hint: the rotation angle's irrationality.)
- What other right triangles give aperiodic substitutions? (Hint: any with irrational angle ratios.)

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — DEPTH, SIZE_FRAC
§3 color     — depth-keyed palette
§4 formula   — slope_char + Bresenham line_draw + foot_perp
§5 subdivide — recursive 5-way split
§6 scene     — seed_triangle (1-2-√5) + scene_draw
§7 screen    — ncurses init / cleanup
§8 app       — signals, main loop
```

### Data Flow
```
seed_triangle → subdivide(depth) → 5^depth leaves
                          ↓
              line_draw 3 edges per leaf
              foot_perp computes the altitude split
```
