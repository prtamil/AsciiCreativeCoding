# Concept: Delaunay Triangulation (Bowyer-Watson)

## Pass 1 — Understanding

### Core Idea
N random points are scattered across the screen and triangulated such that no point lies inside any triangle's circumscribed circle. This is the unique (up to special-case ties) triangulation that maximises the minimum interior angle — Delaunay's classical optimality property. The algorithm is Bowyer-Watson incremental: insert points one at a time, fixing up the triangulation locally.

### Bowyer-Watson Algorithm
```
1. Build a "super-triangle" enclosing every input point.
2. For each input point P:
     a. Find every triangle whose circumcircle contains P → "bad triangles".
     b. Remove the bad triangles, leaving a star-shaped polygonal hole.
     c. Connect P to every vertex on the hole boundary → new triangles fan.
3. Remove triangles that share a vertex with the super-triangle.
```
Each insertion is O(n) worst case; total O(n²). For n in the hundreds, fast enough.

### In-Circumcircle Predicate
Whether `D` lies inside circumcircle of `(A, B, C)` (with CCW orientation):
```
| Ax−Dx  Ay−Dy  (Ax−Dx)²+(Ay−Dy)² |
| Bx−Dx  By−Dy  (Bx−Dx)²+(By−Dy)² |  > 0
| Cx−Dx  Cy−Dy  (Cx−Dx)²+(Cy−Dy)² |
```
Sign convention requires CCW input — we check via `orient2d` and swap if needed.

### orient2d
The signed double-area of triangle (A, B, C):
```
orient2d(A, B, C) = (Bx−Ax)·(Cy−Ay) − (By−Ay)·(Cx−Ax)
```
Positive → CCW, negative → CW, zero → collinear.

### Delaunay Property
A triangulation is Delaunay iff no point is inside any triangle's circumcircle. Equivalently, every internal edge has the "empty circumcircle" property locally. Bowyer-Watson guarantees this by construction.

### Why "Maximises Minimum Angle"
Among all triangulations of a fixed point set, the Delaunay one has the LARGEST minimum interior angle (well, lexicographically — max-min, then second-min, etc.). Practical consequence: triangles are "fat", not slivery — better for finite-element meshes.

### Connection to Other Files
- `01_equilateral.c` — REGULAR triangle tiling (every triangle is equilateral, every angle 60°).
- This file is the IRREGULAR generalisation: any point cloud → an optimal triangulation.
- `geometry/delaunay_triangulation.c` (in this project) — fuller reference with circumcircle visualisation.

### Non-Obvious Decisions
- **Super-triangle large enough**: vertices must lie outside any possible circumcircle of input points. Common choice: 3× the bounding box, so the super-triangle vertices end up far enough that no input point's circumcircle reaches them.
- **CCW enforcement**: the in-circle predicate gets the wrong sign for CW triangles. Code calls `orient2d` and swaps if needed.
- **Cursor cycles triangle list**: `,` / `.` walk the array of current triangles; the highlighted one shows its CCW vertex order.

### Key Constants
| Name | Role |
|------|------|
| `N_POINTS` | Default number of random points |
| `MARGIN` | Border padding inside which points spawn |

### Open Questions
- What does the dual Voronoi diagram look like? (Each Voronoi cell's centre = a Delaunay triangle's circumcentre.)
- How does "constrained Delaunay" differ — when some edges are forced?

---

## Pass 2 — Implementation

### Module Map
```
§1 config   — N_POINTS, MARGIN
§4 formula  — orient2d, in_circumcircle determinant
§5 mesh     — Bowyer-Watson insertion + super-triangle cleanup
§5b draw    — slope_char + Bresenham line_draw
§6 scene    — seed_random + scene_draw
§7 screen   — ncurses init / cleanup
§8 app      — signals, main loop
```

### Data Flow
```
random points → super-triangle → for each point P:
                                    bad = {T : in_circumcircle(T, P)}
                                    remove bad → star hole boundary
                                    fan P → boundary → new triangles
final cleanup: drop triangles sharing super-triangle vertices
```
