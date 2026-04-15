# Concept: Apollonian Gasket (Circle Packing)

## Pass 1 — Understanding

### Core Idea
Start with three mutually tangent circles. Find the unique circle tangent to all three (Apollonius circle). Then recursively fill each gap with circles tangent to the surrounding three. The result is a fractal circle packing where every gap is filled but the total area remains positive.

### Mental Model
Imagine three soap bubbles pressed together. The gap between them has a unique bubble that fits perfectly. Each new gap created by adding a bubble also has a unique fitting bubble. Keep going forever — you get the Apollonian gasket, a fractal dust of circles.

### Key Equations
Descartes' Circle Theorem: if four mutually tangent circles have curvatures k₁,k₂,k₃,k₄:
```
(k₁+k₂+k₃+k₄)² = 2(k₁²+k₂²+k₃²+k₄²)
```
Solving for k₄:
```
k₄ = k₁+k₂+k₃ ± 2·√(k₁k₂+k₂k₃+k₃k₁)
```
Curvature k = 1/radius (negative for enclosing circles).

Center of the new circle (complex number formulation):
```
k₄·z₄ = k₁z₁ + k₂z₂ + k₃z₃ ± 2·√(k₁k₂z₁z₂ + k₂k₃z₂z₃ + k₃k₁z₃z₁)
```

### Data Structures
- Circle: {curvature k, center (cx, cy)}
- Stack or queue of triples (c1,c2,c3) to process
- Visited set to avoid duplicates

### Non-Obvious Decisions
- **Floating point curvature**: Integer Apollonian gaskets (all curvatures integers) exist for special starting conditions. For general cases use double.
- **Depth limit**: Stop recursion when circle radius < 1 pixel. Without this, infinite recursion.
- **Drawing circles in ASCII**: Draw circle outlines using `(col - cx)² + (row*ASPECT - cy)² ≈ r²` with a tolerance. Or use only character cells whose center is within the ring.
- **Negative curvature**: The outer enclosing circle has negative curvature. Handle the sign correctly in Descartes' theorem.

### Key Constants
| Name | Role |
|------|------|
| MIN_RADIUS | stop recursion below this pixel size |
| MAX_DEPTH | hard recursion depth limit |
| ASPECT | row height / column width correction |

### Open Questions
- What are the first four integers that form an integer Apollonian gasket?
- Does the total area of circles converge or diverge as depth → ∞?
- Can you color circles by depth level?

---

## Pass 2 — Implementation

### Pseudocode
```
struct Circle { double k; double cx, cy; }

descartes_new_k(c1,c2,c3) → (k4a, k4b):
    s = c1.k + c2.k + c3.k
    p = c1.k*c2.k + c2.k*c3.k + c3.k*c1.k
    k4a = s + 2*sqrt(p)
    k4b = s - 2*sqrt(p)

descartes_new_center(c1,c2,c3,k4) → (cx4,cy4):
    # complex Descartes formula
    z1=c1.cx+i*c1.cy, z2=c2.cx+i*c2.cy, z3=c3.cx+i*c3.cy
    znum = c1.k*z1 + c2.k*z2 + c3.k*z3
    product_sum = c1.k*c2.k*z1*z2 + c2.k*c3.k*z2*z3 + c3.k*c1.k*z3*z1
    z4 = (znum ± 2*sqrt(product_sum)) / k4

gasket(c1,c2,c3):
    k4 = descartes_new_k(c1,c2,c3)  # pick correct root
    if 1/k4 < MIN_RADIUS: return
    c4 = Circle{k4, descartes_new_center(c1,c2,c3,k4)}
    draw_circle(c4)
    gasket(c1,c2,c4)
    gasket(c2,c3,c4)
    gasket(c3,c1,c4)

draw_circle(c):
    r = 1/c.k
    for each row, col near (c.cx, c.cy):
        if |dist - r| < 0.7: mvaddch(row, col, 'o')
```

### Module Map
```
§1 config    — MIN_RADIUS, starting triple
§2 math      — descartes_k(), descartes_center() (complex arithmetic)
§3 gasket    — recursive gasket() with depth limit
§4 draw      — draw_circle() with ring test
§5 app       — main (setup starting circles, run gasket, display)
```

### Data Flow
```
three seed circles → recursive Descartes → new circle
→ draw ring → recurse into three new gaps
(stop when radius < MIN_RADIUS)
```

## From the Source

**Algorithm:** Recursive circle-packing via Descartes' Circle Theorem. Given three mutually-tangent circles with curvatures k₁,k₂,k₃ (curvature = 1/radius), the fourth tangent circle satisfies `(k₁+k₂+k₃+k₄)² = 2(k₁²+k₂²+k₃²+k₄²)`. Solving: `k₄ = k₁+k₂+k₃ ± 2√(k₁k₂+k₂k₃+k₃k₁)`. There are two solutions; one is the already-known circle.

**Math:** Complex Descartes theorem gives the centre directly: `z₄ = (k₁z₁+k₂z₂+k₃z₃ ± 2√(k₁k₂z₁z₂+…)) / k₄`. Outer circle has negative curvature (k < 0): it contains all others. Starting pack k = (−1, 2, 2, 3) is the unique integer Apollonian gasket. Recursion terminates when the resulting circle is smaller than one pixel on screen.

**Performance:** Circle count grows as 3^depth; depth=7 (DEPTH_MAX) gives ~6500 circles. Progressively drawn (one circle per frame) so the gasket builds up visually. Terminal aspect correction ASPECT_R scales the radius when converting to cell coordinates.
