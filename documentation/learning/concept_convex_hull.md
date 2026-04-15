# Concept: Convex Hull (Computational Geometry)

## Pass 1 — Understanding

### Core Idea
Given N points in the plane, find the smallest convex polygon that contains all of them (the convex hull). Two classic algorithms: Graham scan (O(N log N)) and Jarvis march / gift wrapping (O(NH) where H = hull size).

### Mental Model
Imagine N nails hammered into a board. Stretch a rubber band around them and let it snap tight. The shape it makes is the convex hull. Only the outermost nails touch the rubber band.

### Key Algorithms
**Graham scan (O(N log N))**:
1. Find the lowest point P₀ (bottom-most, then left-most)
2. Sort other points by polar angle relative to P₀
3. Push P₀ and first two points onto stack
4. For each remaining point: while top-2→top-1→new makes a right turn (or is collinear), pop. Push new point.
5. Stack contains the hull.

**Jarvis march (O(NH))**:
1. Start with left-most point
2. Find the point that makes the smallest counter-clockwise angle from current direction
3. Add to hull, advance. Repeat until back at start.

Cross product for orientation:
```
cross(O, A, B) = (A.x-O.x)*(B.y-O.y) - (A.y-O.y)*(B.x-O.x)
> 0: left turn (CCW), = 0: collinear, < 0: right turn (CW)
```

### Data Structures
- `points[N]`: {x, y} float
- `hull[]`: subset of points in CCW order
- Graham scan stack: array used as stack

### Non-Obvious Decisions
- **Tie-breaking in Graham scan**: When two points have the same polar angle, keep only the farthest (remove closer points before processing).
- **Robustness**: Floating-point cross products can have sign errors near zero. Use robust predicates for exact computation if points are nearly collinear.
- **Animated visualization**: Show points as dots, current search progress highlighted, hull edges drawn as they're finalized.
- **Dynamic hull**: Add/remove points and re-run. For truly dynamic hull maintenance, use a balanced BST — complex to implement.

### Key Constants
| Name | Role |
|------|------|
| N | number of random points |
| POINT_SPEED | how fast points move (for animated version) |
| ALGO | Graham scan or Jarvis |

### Open Questions
- For random points in a disc, what fraction are on the hull (expected)?
- What is the worst case for Jarvis march? (all points on hull)
- Can you find the minimum enclosing circle of the points?

---

## From the Source

**Algorithm:** Two algorithms compared side-by-side — Graham scan and Jarvis march (gift-wrapping).
- Graham scan: pivot is the lowest-y (rightmost on tie) point. Sort by polar angle. Sweep stack — pop when cross-product sign indicates a clockwise (non-left) turn: `(B−A) × (C−A) < 0`.
- Jarvis march: repeatedly find the most counter-clockwise point from the current hull vertex. Optimal when `h ≪ N`.

**Math:** Cross product formula (foundation of all computational geometry primitives):
`(A→B) × (A→C) = (Bx−Ax)(Cy−Ay) − (By−Ay)(Cx−Ax)`

**Performance:** Graham scan O(N log N) — dominated by sort; stack sweep is O(N) since each point is pushed/popped at most once. Jarvis march O(N·h): worst case O(N²) when all points are on the hull, optimal for small h.

---

## Pass 2 — Implementation

### Pseudocode
```
cross(O, A, B) → float:
    return (A.x-O.x)*(B.y-O.y) - (A.y-O.y)*(B.x-O.x)

graham_scan(points) → hull[]:
    p0 = lowest-then-leftmost point
    sort rest by polar angle from p0 (break ties: keep farthest)
    stack = [p0, sorted[0], sorted[1]]
    for pt in sorted[2..]:
        while len(stack)>1 and cross(stack[-2],stack[-1],pt) <= 0:
            stack.pop()
        stack.push(pt)
    return stack

jarvis_march(points) → hull[]:
    start = leftmost point
    hull = [start]
    current = start
    loop:
        next = points[0]
        for pt in points[1..]:
            c = cross(current, next, pt)
            if c < 0: next = pt   # pt is more counter-clockwise
            if c == 0 and dist(current,pt) > dist(current,next): next = pt
        if next == start: break
        hull.append(next); current = next
    return hull

draw(points, hull):
    for pt in points: mvaddch(pt.y, pt.x, '.')
    for i in 0..len(hull):
        draw_line(hull[i], hull[(i+1)%len(hull)])   # hull edges
    for pt in hull: mvaddch(pt.y, pt.x, 'O')        # hull vertices
```

### Module Map
```
§1 config    — N, ALGO, random bounds
§2 geometry  — cross(), dist(), angle_sort()
§3 hull      — graham_scan(), jarvis_march()
§4 draw      — points + hull edges + highlight
§5 app       — main (generate → compute → animate), keys (algo, reset, N)
```

### Data Flow
```
N random points → sort (Graham) or iterate (Jarvis)
→ hull point list → draw edges + highlight hull vertices
(optional: animate step-by-step)
```
