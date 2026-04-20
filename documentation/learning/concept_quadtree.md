# Concept: Quadtree (Spatial Partitioning)

## Pass 1 — Understanding

### Core Idea
A quadtree recursively partitions a 2D rectangular space into four equal quadrants (NW, NE, SW, SE) whenever a leaf node accumulates more than LEAF_CAPACITY points.  Lookup, insertion, and range queries all benefit from pruning — the tree skips entire subtrees that cannot contain the answer.

### Mental Model
Think of a city map.  At first you have one region (the whole city).  When a neighbourhood becomes too crowded you split it into four smaller districts.  Each district can split further.  Finding all coffee shops in a rectangle means checking only the districts that overlap the rectangle — skipping the rest entirely.

### Key Algorithms

**INSERT — O(log N) average:**
```
bsp_insert(node, point):
    if point not in node.boundary: return false
    if node is leaf:
        if node has room: store point; return true
        subdivide(node)          ← splits into 4 children
    try each child; return true when one accepts the point
```

**SUBDIVIDE — O(LEAF_CAPACITY):**
```
subdivide(node):
    mid_x = node.x + node.w / 2
    mid_y = node.y + node.h / 2
    create NW, NE, SW, SE children (half-open rectangles)
    re-insert all existing points into children
    node becomes internal (count = 0, children non-NULL)
```

**QUERY — O(log N + k) where k = results found:**
```
qt_query(node, search_rect, results):
    if node.boundary does NOT overlap search_rect: return   ← PRUNE
    for each point in node: if inside search_rect, collect it
    recurse into all four children
```

### Half-Open Intervals
Boundaries use `[x, x+w) × [y, y+h)`.  The midpoint split produces `[x, x+w/2)` and `[x+w/2, x+w)` — they share no border pixels and together cover the whole region exactly.  Inclusive intervals would cause ambiguity at the midpoint.

### Data Structures
```
QuadNode {
    Rect      boundary;              // half-open [x, x+w) × [y, y+h)
    Point     data[LEAF_CAPACITY];   // stored only in leaf nodes
    int       count;
    QuadNode *nw, *ne, *sw, *se;    // NULL → this is a leaf
}
```

### Non-Obvious Decisions
- **Static pool vs malloc:** `quad_tree_helloworld.c` (ncurses animated version) uses a fixed-size `g_nodes[]` pool — reset = set counter to 0, zero heap fragmentation.  `quadtree.c` (pure C demo) uses `malloc`/`free` for clarity.
- **Children store no points:** An internal node has `count = 0`; all points live in leaves.  This keeps routing simple — you never split mid-insertion.
- **Snapshot before redistribution:** `subdivide()` snapshots the current points into a local array before resetting `count`, because `qt_insert()` modifies `count` during redistribution.
- **Depth limit is optional:** A minimum cell size (`w < 1` or `h < 1`) should guard against infinite subdivision if two points share the same pixel.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| `LEAF_CAPACITY` | 4 | points per leaf before split |
| `SPACE_W` | 56 | demo grid width |
| `SPACE_H` | 22 | demo grid height |
| `QUERY_CAP` | 64 | max results per range query |

### Open Questions
- What happens when two points share the same coordinate?  (Infinite subdivision unless guarded.)
- How does quadtree performance compare to a flat grid for dense uniform vs sparse non-uniform data?
- What is the expected number of nodes for N random points with capacity C?

---

## From the Source

**Files:** `geometry/quad_tree_helloworld.c` (ncurses animated, static pool) and `geometry/quadtree.c` (pure C, malloc, step-by-step demo).

**Algorithm:** Standard PR (Point Region) quadtree.  Each node owns a fixed rectangular boundary; subdivision always splits at the exact midpoint.

**AABB pruning in query:**  Four conditions skip a subtree instantly — search starts right of boundary, ends left of boundary, starts below boundary, ends above boundary.  Any other case means overlap and the subtree must be checked.

**Asymptotic complexity:**
- Insert: O(log N) average, O(N) worst (all points in one quadrant repeatedly)
- Query: O(log N + k), where k = results returned
- Subdivide: O(LEAF_CAPACITY) — constant work to redistribute

**Barnes-Hut gravity** (`physics/barnes_hut.c`) uses the same quadtree structure for O(N log N) gravity approximation — the `s/d < θ` criterion prunes far-away cells just like the range query prunes non-overlapping cells.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `QuadNode.boundary` | `Rect` | 16 B | half-open rectangle this node is responsible for |
| `QuadNode.data[]` | `Point[4]` | ~12 B | points stored in this leaf |
| `QuadNode.count` | `int` | 4 B | number of valid points in data[] |
| `QuadNode.nw/ne/sw/se` | `QuadNode*` | 32 B | child pointers; all NULL → leaf node |
| `g_nodes[]` (helloworld) | `QuadNode[512]` | ~30 KB | static node pool; g_node_count = next free index |

## Pass 2 — Implementation

### Pseudocode
```
qt_insert(node, p):
    if not rect_contains(node.boundary, p): return false
    if node is leaf:
        if node.count < LEAF_CAPACITY:
            node.data[count++] = p; return true
        subdivide(node)          // node is now internal
    for child in [nw, ne, sw, se]:
        if qt_insert(child, p): return true
    return false  // unreachable if boundary covers p

subdivide(node):
    hx = node.w / 2;  hy = node.h / 2
    mx = node.x + hx; my = node.y + hy
    node.nw = new_node(x,  y,  hx, hy)
    node.ne = new_node(mx, y,  hx, hy)
    node.sw = new_node(x,  my, hx, hy)
    node.se = new_node(mx, my, hx, hy)
    snapshot = node.data[0..count]
    node.count = 0
    for p in snapshot: qt_insert(node, p)

qt_query(node, x1, y1, x2, y2, out):
    if not overlaps(node.boundary, x1,y1,x2,y2): return
    for p in node.data:
        if p inside [x1,x2]×[y1,y2]: out.append(p)
    for child in [nw, ne, sw, se]:
        qt_query(child, x1,y1,x2,y2, out)
```

### Module Map
```
§1 constants     — LEAF_CAPACITY, SPACE_W/H, QUERY_CAP
§2 data types    — Point, Rect, QuadNode
§3 memory        — node_new(), tree_free()
§4 rect helpers  — rect_contains_point(), rect_overlaps_range()
§5 core ops      — qt_insert(), subdivide(), qt_query()
§6 inspection    — tree_node_count(), tree_point_count(), tree_depth(), tree_dump()
§7 visualizer    — g_grid[][], grid_put(), grid_draw_border(), grid_render_tree(), grid_print()
§8 demo          — press_enter(), show_tree(), show_query(), main() 8-step walkthrough
```

### Data Flow
```
node_new(full_space)
    → qt_insert() × N  → subdivide() on overflow
    → qt_query(search_rect)
        → rect_overlaps_range() pruning at every node
        → collect k matching points
    → grid_render_tree() → grid_print() (ANSI colored)
    → tree_free()
```
