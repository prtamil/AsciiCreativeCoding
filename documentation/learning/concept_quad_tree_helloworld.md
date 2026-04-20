# Concept: Quadtree Hello-World (Animated ncurses Demo)

## Pass 1 — Understanding

### Core Idea
An animated ncurses demonstration that teaches the quadtree data structure in two live phases: INSERT (random points added one by one, subdivisions visible as they happen) and QUERY (a moving rectangle sweeps the space, hit points highlighted in real time).  The algorithm is a PR (Point Region) quadtree — see `concept_quadtree.md` for the full mathematical treatment.

### Mental Model
Watch the tree grow: each new point either drops into an existing leaf or triggers a subdivision that splits the leaf into four colour-coded quadrants.  Then watch the query box move and see which subtrees light up (overlap) versus go dark (pruned) — the AABB pruning is visible because only overlapping node borders change colour.

### What This Demo Adds over `quadtree.c`
| Feature | `quadtree.c` (pure C) | `quad_tree_helloworld.c` (ncurses) |
|---------|----------------------|-------------------------------------|
| Output | Static ASCII grid per step | Real-time animated terminal UI |
| Pacing | Enter-key per step | Fixed-timestep sim loop (30 Hz) |
| Phases | Manual walkthrough | INSERT phase → QUERY phase auto-cycle |
| Info panel | printf narrative | Live scrolling info panel |
| Memory model | malloc per node | Static pool (`g_nodes[512]`) |

### Phases
1. **INSERT phase** — A random point is added every `INSERT_INTERVAL` seconds.  The tree border colour reflects node depth: root (white) → depth-1 (yellow) → depth-2 (red) → deeper (cyan).  The info panel explains what just happened.
2. **QUERY phase** — After all points are inserted, a drifting rectangle sweeps the space.  Borders of overlapping nodes turn green; contained points turn bright.  The panel shows hit count and the pruning explanation.

### Framework Pattern (8 sections)
Follows the same `§1 config → §2 clock → §3 color → §4 coords → §5 entity → §6 scene → §7 screen → §8 app` structure as all other programs in this project.  The static node pool replaces malloc so no heap allocation occurs during the sim loop.

### Data Structures
```
QuadNode {
    Rect      bounds;
    Vec2      points[LEAF_CAPACITY];
    int       point_count;
    int       children[4];         // indices into g_nodes[]; NO_CHILD = -1
    int       depth;
}

TreeCanvas { int top_row, bottom_row, right_col; }  // drawing area

PanelWriter { int panel_col, current_row, last_row; } // info panel cursor
```

### Non-Obvious Decisions
- **Static pool (no malloc):** `g_nodes[512]` with `g_node_count` as the free pointer.  Reset the tree by setting `g_node_count = 1` (root stays at index 0).  Eliminates fragmentation and `free()` during the live loop.
- **Children as int indices:** Children store indices into `g_nodes[]`, not pointers.  Avoids pointer invalidation if the pool array is ever reallocated (it isn't here, but the pattern is safe).
- **Named color pair enum:** `CP_ROOT_BORDER=1, CP_D1_BORDER=2, CP_D2_BORDER=3, CP_DEEP_BORDER=4, CP_POINT_IDLE=5, CP_POINT_HIT=6, CP_QUERY_BOX=7, CP_PANEL=8` — eliminates raw `COLOR_PAIR(n)` literals in draw calls.
- **TreeCanvas / PanelWriter abstractions:** Two small structs that track drawing boundaries and eliminate repetitive `attron/mvprintw/attroff` calls throughout the draw functions.
- **Phase-driven sim:** `DemoPhase` enum (`PHASE_INSERT` / `PHASE_QUERY`) lets `scene_tick()` and `scene_draw()` behave completely differently without `if (phase)` scattered through every function.

### Key Constants
| Name | Value | Role |
|------|-------|------|
| `LEAF_CAPACITY` | 4 | points before subdivision |
| `N_PTS` | 40 | total points to insert |
| `INSERT_INTERVAL` | 0.35 s | time between insertions |
| `POOL_MAX` | 512 | max nodes in static pool |
| `QUERY_RESULT_CAP` | 64 | max query results |
| `SIM_HZ` | 30 | simulation ticks per second |

### Open Questions
- How does the visual change if you increase `LEAF_CAPACITY` to 8?  (Fewer, larger cells before subdivision.)
- What does the query phase look like for a very small rectangle vs. one that covers the whole space?
- How would you visualise the pruning explicitly — e.g., flash pruned subtrees grey for one frame?

---

## From the Source

**File:** `geometry/quad_tree_helloworld.c`

**Insert tick:** `scene_tick()` calls `insert_random_point(s)` every `next_insert_in` seconds.  A `Vec2` with random normalised coordinates `[0, 1)` is inserted into the root node.  After `N_PTS` insertions the phase advances to `PHASE_QUERY`.

**Query tick:** `advance_query_box(q, dt)` moves the query rectangle by `(drift_x, drift_y)`, bouncing off the unit boundary with velocity negation.  `tree_query()` is called every tick.

**Draw pipeline:**
```
scene_draw()
  → erase()
  → draw_tree(root_idx, &sc, cv)       // borders + points, depth-coloured
  → draw_query_box(&sc.query, cv)       // red dashed rectangle (QUERY phase)
  → draw_info_panel(&sc, cols, rows)    // right panel: PanelWriter
  → wnoutrefresh() → doupdate()
```

**draw_tree** is recursive — it calls `draw_node_border()` then `draw_node_points()` per node, then recurses into children.  Depth determines colour pair.

---

# Structure

| Symbol | Type | Size | Role |
|--------|------|------|------|
| `g_nodes[POOL_MAX]` | `QuadNode[512]` | ~30 KB | static node pool; index 0 = root |
| `g_node_count` | `int` | 4 B | next free slot in pool |
| `Scene.phase` | `DemoPhase` | 4 B | INSERT or QUERY |
| `Scene.query` | `QueryBox` | ~32 B | drifting query rectangle + velocity |
| `Scene.query_results[]` | `Vec2[64]` | 512 B | points found in current tick |
| `TreeCanvas` | struct | 12 B | top/bottom/right pixel bounds of tree area |
| `PanelWriter` | struct | 12 B | column + row cursor for info panel |

## Pass 2 — Implementation

### Pseudocode
```
scene_tick(sc, dt):
    if sc.phase == INSERT:
        sc.next_insert_in -= dt
        if sc.next_insert_in <= 0:
            insert_random_point(sc)
            sc.next_insert_in = INSERT_INTERVAL
        if sc.points_inserted == N_PTS: sc.phase = QUERY
    else:
        advance_query_box(&sc.query, dt)
        sc.query_result_count = 0
        tree_query(0, sc.query.bounds, sc.query_results, &sc.query_result_count, QUERY_RESULT_CAP)

draw_tree(node_idx, sc, cv):
    node = g_nodes[node_idx]
    draw_node_border(node, cv)     // box with depth-coloured pair
    draw_node_points(node, sc, cv) // dots; green if hit by query
    for c in node.children:
        if c != NO_CHILD: draw_tree(c, sc, cv)

tree_insert(node_idx, x, y):
    if not in bounds: return false
    if leaf and has room: store; return true
    if leaf and full: subdivide(node_idx)
    for c in children: if tree_insert(c, x, y): return true

tree_query(node_idx, search, results, count, cap):
    if not overlaps(node.bounds, search): return   // PRUNE
    for p in node.points: if inside search: collect
    for c in children: tree_query(c, ...)
```

### Module Map
```
§1 config       — LEAF_CAPACITY, N_PTS, INSERT_INTERVAL, SIM_HZ, POOL_MAX
§2 clock        — monotonic timer, frame sleep
§3 color        — 8 named pairs (CP_ROOT_BORDER .. CP_PANEL)
§4 coords       — TreeCanvas: canvas_make(), canvas_col/row(), canvas_put()
§5 entity       — QuadNode pool, Vec2, Rect, QueryBox
                  tree_insert(), subdivide(), tree_query()
                  tree_total_nodes(), tree_current_depth()
§6 scene        — Scene, DemoPhase, insert_random_point(), advance_query_box()
                  scene_init(), scene_tick()
§7 screen       — PanelWriter, draw_node_border/points, draw_tree()
                  draw_query_box(), draw_info_panel(), scene_draw()
§8 app          — main(): ncurses init → loop: input → tick → draw → sleep
```

### Data Flow
```
scene_init()
  → INSERT phase:
      scene_tick() → insert_random_point()
          → tree_insert(0, x, y) → subdivide() on overflow
      scene_draw() → draw_tree() [depth-coloured borders]
  → QUERY phase:
      scene_tick() → advance_query_box() → tree_query(0, ...)
      scene_draw() → draw_tree() [hit nodes green]
                   → draw_query_box() [red dashed rect]
                   → draw_info_panel() [PanelWriter right panel]
```
