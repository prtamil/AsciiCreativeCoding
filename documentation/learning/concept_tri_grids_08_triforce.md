# Concept: Triforce (4-Way Midpoint Subdivision)

## Pass 1 — Understanding

### Core Idea
Take one equilateral triangle and recursively split it into 4 smaller similar triangles by joining the three edge midpoints. Three corner children match the parent's orientation; the fourth — the inverted centre — is rotated 180°. Leaf count grows as 4^N. This is the well-known "triforce" pattern; the centre child is what `09_sierpinski.c` discards.

### Subdivision Step
Given (V₀, V₁, V₂):
```
M₀₁ = (V₀ + V₁) / 2
M₁₂ = (V₁ + V₂) / 2
M₂₀ = (V₂ + V₀) / 2
```
Emit four children:
```
(V₀,  M₀₁, M₂₀)    corner at V₀, same orientation
(M₀₁, V₁,  M₁₂)    corner at V₁, same orientation
(M₂₀, M₁₂, V₂)     corner at V₂, same orientation
(M₁₂, M₂₀, M₀₁)    inverted centre child, rotated 180°
```

### Why "Loop subdivision base step"
This is the geometric step in Loop subdivision (Charles Loop 1987 thesis), a well-known algorithm for generating smooth surfaces. Loop adds a smoothing step that perturbs the new vertices toward neighbours; this file uses the pure midpoint version with no smoothing.

### Recursion
```
subdivide(V₀, V₁, V₂, depth):
    if depth == 0:
        emit_leaf(V₀, V₁, V₂)
    else:
        compute M₀₁, M₁₂, M₂₀
        subdivide(V₀,  M₀₁, M₂₀, depth − 1)
        subdivide(M₀₁, V₁,  M₁₂, depth − 1)
        subdivide(M₂₀, M₁₂, V₂,  depth − 1)
        subdivide(M₁₂, M₂₀, M₀₁, depth − 1)   // inverted centre
```

### Visual Property
At every depth the figure shows a TRIFORCE: the inverted centre is visible as a rotated triangle at every level of zoom. The pattern is self-similar at scale 1/2.

### Non-Obvious Decisions
- **Four corner-orientation tags would be redundant**: the centre child's orientation is implicit in the order `(M₁₂, M₂₀, M₀₁)`. Recursive calls just keep going; no flag needed.
- **No drop**: this file emits ALL 4 children. `09_sierpinski.c` is one line of difference — it drops the 4th call.
- **Depth cap 7**: 4^7 = 16384 leaves; beyond this each leaf is < 1 pixel and rendering is wasted.

### Key Constants
| Name | Role |
|------|------|
| `DEPTH` | Recursion depth (0..7) |
| `SIZE_FRAC` | Seed triangle size relative to screen |

### Open Questions
- What is the relationship between the triforce and Pascal's triangle mod 2?
- If you keep ONLY the centre child at each step, what shape do you get?

---

## Pass 2 — Implementation

### Module Map
```
§1 config    — DEPTH, SIZE_FRAC
§3 color     — depth-keyed palette
§4 formula   — slope_char + Bresenham line_draw (same as 07)
§5 subdivide — recursive 4-way split
§6 scene     — seed_triangle + scene_draw
§7 screen    — ncurses init / cleanup
§8 app       — signals, main loop
```

### Data Flow
```
seed_triangle → subdivide(depth) → 4^depth leaves
                          ↓
              line_draw 3 edges per leaf
```
