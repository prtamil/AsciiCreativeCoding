# Concept: BSP Tree — Binary Space Partition

## Pass 1 — Understanding

### Core Idea
A Binary Space Partition tree recursively divides a rectangular region with a single axis-aligned line, creating two children (front / back) instead of four.  The split axis alternates with depth: even depth → VERTICAL (left/right), odd depth → HORIZONTAL (top/bottom).  Range queries prune entire subtrees using the same AABB overlap test as the quadtree.

### Mental Model
Think of a house floor plan.  First you split the whole building left/right with a load-bearing wall (VERTICAL, depth 0).  Then each half is split top/bottom with another wall (HORIZONTAL, depth 1).  Each room can be split again.  Finding everything in a rectangle means checking only rooms whose walls overlap the rectangle — all others are skipped in one test.

### Contrast with Quadtree
| | Quadtree | BSP Tree |
|--|--|--|
| Split shape | cross (+) → 4 children | single line → 2 children |
| Children | NW, NE, SW, SE | front, back |
| Axis strategy | one cross per split | alternates V/H by depth |
| Nodes per level | 4× | 2× |
| Historical use | nearest-neighbor, collision | Doom/Quake rendering, collisions |

### Key Algorithms

**INSERT — O(log N) average:**
```
bsp_insert(node, point):
    if point not in node.boundary: return false
    if node is leaf:
        if node has room: store point; return true
        subdivide(node)          ← splits into front + back
    try front; if rejected try back; return true
```

**SUBDIVIDE — alternates axis by depth:**
```
subdivide(node):
    if node.depth % 2 == 0:            // VERTICAL split
        split_pos = node.x + node.w / 2
        front boundary = [x, y,         w/2,   h]  (left half)
        back  boundary = [x+w/2, y,     w-w/2, h]  (right half)
    else:                              // HORIZONTAL split
        split_pos = node.y + node.h / 2
        front boundary = [x, y,         w,  h/2  ]  (top half)
        back  boundary = [x, y+h/2,     w,  h-h/2]  (bottom half)
    re-insert displaced points into front or back
```

**QUERY — O(log N + k):**
```
bsp_query(node, x1, y1, x2, y2, results):
    if node.boundary does NOT overlap search_rect: return   ← PRUNE
    for each point in node: if inside search_rect, collect it
    recurse into front, then back
```

### Data Structures
```
BSPNode {
    Rect       boundary;
    Point      data[LEAF_CAPACITY];
    int        count;
    int        depth;         // 0 = root; determines split axis
    int        split_pos;     // coordinate of the dividing line
    SplitAxis  split_axis;    // SPLIT_VERTICAL or SPLIT_HORIZONTAL
    BSPNode   *front;         // left (V) or top (H);  NULL → leaf
    BSPNode   *back;          // right (V) or bottom (H); NULL → leaf
}
```

### Non-Obvious Decisions
- **Depth-based axis alternation:** Keeps the tree balanced in both dimensions over time.  An alternative is to always split along the longer dimension — better for non-square regions.
- **Half-open intervals:** `[x, x+w) × [y, y+h)` convention avoids boundary ambiguity: the two halves `[x, x+w/2)` and `[x+w/2, x+w)` partition the space exactly with no overlap.
- **Only 2 children:** The pruning efficiency of BSP is still O(log N + k) like the quadtree, but the constant factor is smaller per level (2 subtrees to prune vs 4).
- **Game development history:** Doom (1993) precomputed the BSP of the level geometry at compile time, enabling the painter's algorithm at runtime without a z-buffer.  Quake used it for collision detection against world geometry.
- **Snapshot before redistribution:** `subdivide()` copies the displaced points to a stack array before resetting `count`, since `bsp_insert()` modifies `count` during redistribution.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| `LEAF_CAPACITY` | 4 | points per leaf before split |
| `SPLIT_VERTICAL` | 0 | even-depth splits left/right |
| `SPLIT_HORIZONTAL` | 1 | odd-depth splits top/bottom |
| `SPACE_W` | 56 | demo grid width |
| `SPACE_H` | 22 | demo grid height |

### Open Questions
- For a non-square region (e.g. 56×22), does depth-based alternation keep both axes balanced, or does the shorter axis subdivide more finely?
- What is the minimum depth at which a BSP tree and quadtree produce equivalent partitions for the same point set?
- How would you extend BSP to 3D? (axis cycles X→Y→Z→X, or longest-axis heuristic)

---

## From the Source

**File:** `geometry/bsp_tree.c` — pure C, malloc-based, step-by-step demo with ANSI colors.

**Splits in the 12-point demo:**
1. Root (depth 0) → VERTICAL at x=28 (root holds 4 pts, E is 5th)
2. Front child (depth 1) → HORIZONTAL at y=11 (front fills with F, G triggers split)
3. Back child (depth 1) → HORIZONTAL at y=11 (back fills with K, L triggers split)

**Final tree structure (7 nodes — 3 internal, 4 leaves):**
```
root — VERTICAL x=28
├── front [0,0 28×22] — HORIZONTAL y=11
│   ├── front.front [0,0 28×11]:  A F H  (3 pts)
│   └── front.back  [0,11 28×11]: C E G I  (4 pts)
└── back  [28,0 28×22] — HORIZONTAL y=11
    ├── back.front  [28,0 28×11]:  B J K  (3 pts)
    └── back.back   [28,11 28×11]: D L  (2 pts)
```

**Query [5,0]..[25,12] prunes the entire back subtree** in one AABB test (search ends at x=25 < back's left edge x=28), then collects A, F, H from front.front and E from front.back.

**Grid characters (ANSI colored):**
- `!` — vertical split line (magenta)
- `=` — horizontal split line (blue)
- `+ - |` — outer boundary box (cyan)
- `A`..`Z` — data points (yellow); `*` — found by query (green)
- `[ ~ ]` — query rectangle (red)

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `BSPNode.boundary` | `Rect` | 16 B | half-open rectangle this node is responsible for |
| `BSPNode.data[]` | `Point[4]` | ~12 B | points stored in this leaf |
| `BSPNode.count` | `int` | 4 B | number of valid points in data[] |
| `BSPNode.depth` | `int` | 4 B | 0 = root; even→V split, odd→H split |
| `BSPNode.split_pos` | `int` | 4 B | coordinate of the split line (0 if leaf) |
| `BSPNode.split_axis` | `SplitAxis` | 4 B | SPLIT_VERTICAL or SPLIT_HORIZONTAL |
| `BSPNode.front/back` | `BSPNode*` | 16 B | children; both NULL → leaf node |

## Pass 2 — Implementation

### Pseudocode
```
bsp_insert(node, p):
    if not rect_contains(node.boundary, p): return false
    if node is leaf:
        if node.count < LEAF_CAPACITY:
            node.data[count++] = p; return true
        subdivide(node)
    if bsp_insert(node.front, p): return true
    if bsp_insert(node.back,  p): return true
    return false

subdivide(node):
    if node.depth % 2 == 0:       // VERTICAL
        half = node.w / 2
        node.split_pos  = node.x + half
        node.split_axis = SPLIT_VERTICAL
        front_rect = { node.x,         node.y, half,          node.h }
        back_rect  = { node.x + half,  node.y, node.w - half, node.h }
    else:                          // HORIZONTAL
        half = node.h / 2
        node.split_pos  = node.y + half
        node.split_axis = SPLIT_HORIZONTAL
        front_rect = { node.x, node.y,        node.w, half          }
        back_rect  = { node.x, node.y + half, node.w, node.h - half }
    node.front = node_new(front_rect, node.depth + 1)
    node.back  = node_new(back_rect,  node.depth + 1)
    snapshot = node.data[0..count]; node.count = 0
    for p in snapshot:
        if not bsp_insert(node.front, p): bsp_insert(node.back, p)

bsp_query(node, x1, y1, x2, y2, out):
    if not overlaps(node.boundary, x1,y1,x2,y2): return
    for p in node.data:
        if p inside [x1,x2]×[y1,y2]: out.append(p)
    bsp_query(node.front, x1,y1,x2,y2, out)
    bsp_query(node.back,  x1,y1,x2,y2, out)
```

### Module Map
```
§1 constants     — LEAF_CAPACITY, SPACE_W/H, QUERY_CAP
§2 data types    — Point, Rect, SplitAxis, BSPNode
§3 memory        — node_new(), tree_free()
§4 rect helpers  — rect_contains_point(), rect_overlaps_range()
§5 core ops      — bsp_insert(), subdivide(), bsp_query()
§6 inspection    — tree_node_count(), tree_point_count(), tree_depth(), tree_dump()
§7 visualizer    — g_grid[][], grid_put(), grid_draw_border(), grid_draw_query_rect()
                   grid_render_bsp(), grid_print()
§8 demo          — press_enter(), show_tree(), show_query(), main() 8-step walkthrough
```

### Data Flow
```
node_new(full_space, depth=0)
    → bsp_insert() × N  → subdivide() on overflow
        → alternates VERTICAL / HORIZONTAL by depth
    → bsp_query(search_rect)
        → rect_overlaps_range() prunes subtrees
        → collect k matching points
    → grid_render_bsp() → grid_print() (ANSI colored ! and = lines)
    → tree_free()
```
