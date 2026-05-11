# algorithms — classic algorithms, visualised one step at a time

A reference for the **algorithm visualisers** in this project. This folder
contains **11 self-contained C programs**, each picking one well-known
algorithm from computational geometry, graph theory, sorting, or contour
extraction, and turning its inner loop into something you can watch.

Every file in this folder is built around the **same primitive**: the
algorithm is rewritten as a **state machine that emits one visible event
per tick**, so the animation IS the algorithm — no hidden steps, no
batched updates, no "and then it sorts". Compare-this, swap-that,
subdivide-here, insert-there: every operation is a frame.

If you read **only one file**, read
[`sort_vis.c`](sort_vis.c) — it is the cleanest exemplar of the
"one operation per tick" pattern, and the discipline is downstream of
it. If you want geometry instead of sorting, start with
[`quad_tree_helloworld.c`](quad_tree_helloworld.c).

---

## Table of contents

1. [How to read this folder](#how-to-read-this-folder)
2. [The unifying primitive](#the-unifying-primitive)
3. [Per-file index](#per-file-index)
4. [Subfamilies](#subfamilies)
   * [Spatial trees — quadtree, k-d tree, BSP](#spatial-trees--quadtree-k-d-tree-bsp)
   * [Computational geometry — hulls, Voronoi, visibility, marching](#computational-geometry--hulls-voronoi-visibility-marching)
   * [Graph search and networks](#graph-search-and-networks)
   * [Sorting](#sorting)
5. [Building and running](#building-and-running)

---

## How to read this folder

The recommended path goes from **simplest state machine** to **richest
geometry**, then up to graph and network algorithms.

```
   1.  sort_vis.c                          one compare or swap per tick
        the cleanest "algorithm = state machine"
                │
                ▼
   2.  quad_tree_helloworld.c              insert / subdivide / query
        animated tree construction
                │
                ▼
   3.  quadtree.c   →   bsp_tree.c   →   kd_tree.c
        three spatial trees, same input, side-by-side comparison
                │
                ▼
   4.  marching_squares.c                  16-case lookup → iso-contour
        scalar-field → polyline pipeline
                │
                ▼
   5.  convex_hull.c                       Graham scan vs Jarvis march
        two hull algorithms, side-by-side panels
                │
                ▼
   6.  voronoi.c       →   visibility_polygon.c
        brute-force nearest-seed      angular-sweep visibility
                │
                ▼
   7.  graph_search.c  →   network_sim.c
        BFS / DFS / A* on a graph     SIR epidemic on small-world graph
```

**Prerequisites graph.** Every file's CONCEPTS block points at the most
relevant siblings. The strongest cross-references inside the folder:

* `quadtree.c`, `bsp_tree.c`, `kd_tree.c` — three answers to the same
  problem ("partition N points in 2-D space"). Read them in that order
  and the trade-offs become explicit.
* `quadtree.c` is the textbook version with a step-through demo;
  `quad_tree_helloworld.c` is the **animated** version with an
  insert / subdivide / query loop. Read the textbook one first if you've
  never seen a quadtree; read the animated one first if you have.
* `convex_hull.c` and `graph_search.c` both use the **side-by-side panel**
  trick — two algorithms running on the same input data, paced by the
  same tick, so the visual difference IS the complexity difference.
* `voronoi.c` and `marching_squares.c` are the two "scalar field" sketches
  here. Both sweep every screen cell every frame; one finds the nearest
  seed, the other finds the iso-contour.

---

## The unifying primitive

Every algorithm in this folder is rewritten as an **iterative coroutine**
that performs **exactly one visible operation per simulation tick**.
This is the single discipline that runs through all 11 files.

```c
/* Sketch — every file's §5 algorithm follows this shape: */
typedef enum { IDLE, RUNNING, DONE } Phase;

static Phase  phase;
static int    cursor_i, cursor_j;   /* where the algorithm currently is */
static Stack  work_stack;           /* if recursive: explicit stack */

static void algo_step(void) {
    if (phase != RUNNING) return;
    /* perform ONE compare, ONE swap, ONE subdivide, ONE neighbour expand */
    /* update cursor_i / cursor_j so §6 scene can highlight them */
    /* if work complete: phase = DONE */
}
```

Three things follow from the "one operation per tick" rule:

1. **No recursion.** Recursive algorithms (quicksort, quadtree subdivide,
   DFS) are rewritten with an **explicit stack**. The stack is part of
   the visible state — you can pause mid-recursion, inspect, resume.
2. **No batched work.** Each tick fires exactly one comparison, one
   swap, one neighbour expansion, one subdivision. The tick rate
   (`+`/`-`) is the algorithm's speed knob.
3. **State survives pause.** Press `p` and the algorithm freezes mid-
   step. Press `p` again and it continues from the same `cursor_i,
   cursor_j`. The animation has no implicit time dependence.

The **rendering** half of every file is the inverse: it reads the
algorithm's cursor variables and highlights the cells the algorithm is
currently touching — yellow for compare, red-bold for swap, cyan for
frontier, dark-grey for visited. The colour key IS the algorithm's
event taxonomy.

```
            ┌───────────────────────────────────────────┐
   tick ──▶ │  algo_step()                              │
            │      ↓                                    │
            │  cursor_i, cursor_j, work_stack updated   │
            └───────────────────────────────────────────┘
                            │
                            ▼
            ┌───────────────────────────────────────────┐
   draw ──▶ │  scene_draw()                             │
            │      reads cursor + state                 │
            │      paints highlights on top of base     │
            └───────────────────────────────────────────┘
                            │
                            ▼
                     ncurses screen
```

This is the same pattern used by `procedural/generational/maze.c` (carve-
one-cell-per-tick) — see [Study alongside](sort_vis.c) lines in the file
header. Algorithms become **legible** when their inner loop runs at
human speed.

---

## Per-file index

| #  | File                          | Algorithm                              | Complexity                | Tick = one |
|----|-------------------------------|----------------------------------------|---------------------------|------------|
| 1  | `sort_vis.c`                  | Bubble / Insertion / Selection / Quick / Heap | O(n²) – O(n log n) | compare or swap |
| 2  | `quad_tree_helloworld.c`      | Quadtree insert + range query (animated)      | O(log N + k) avg   | insert or query frame |
| 3  | `quadtree.c`                  | Quadtree (step-through demo)                  | O(log N + k) avg   | Enter step  |
| 4  | `bsp_tree.c`                  | Binary Space Partition (axis-alternating)     | O(log N) avg       | Enter step  |
| 5  | `kd_tree.c`                   | K-D tree (point = split plane)                | O(log N) avg       | Enter step  |
| 6  | `marching_squares.c`          | Marching Squares iso-contour                  | O(W·H) per frame   | one cell    |
| 7  | `convex_hull.c`               | Graham scan vs Jarvis march (side-by-side)    | O(N log N) / O(N·h)| one step    |
| 8  | `voronoi.c`                   | Brute-force Voronoi + Langevin Brownian drift | O(cells · seeds)   | one frame   |
| 9  | `visibility_polygon.c`        | Angular-sweep exact visibility polygon        | O(W log W), W=walls| one observer frame |
| 10 | `graph_search.c`              | BFS / DFS / A* on a random graph              | O(V+E) / O((V+E) log V) | one node expand |
| 11 | `network_sim.c`               | SIR epidemic on Watts-Strogatz small-world    | O(N + E) per tick  | one infection round |

---

## Subfamilies

### Spatial trees — quadtree, k-d tree, BSP

Four files (`quadtree.c`, `quad_tree_helloworld.c`, `bsp_tree.c`,
`kd_tree.c`) attack the same problem — **partition N points in 2-D space
so range queries beat the brute-force O(N) scan** — three different ways.
The pedagogical payoff is in reading the three side-by-side.

| Tree   | Children/node | What the node stores      | Best when                       |
|--------|---------------|---------------------------|---------------------------------|
| Quad   | 4 (NW/NE/SW/SE) | leaf: ≤ K points        | clustered data, range queries   |
| BSP    | 2             | split axis + position     | back-to-front rendering (Doom)  |
| K-D    | 2             | the point IS the split    | nearest-neighbour search        |

All three subdivide the plane recursively, but they differ in **what is
stored at a node** and **how a node decides where to split**. Layout
sketch (k-d tree, after inserting 7 points):

```
          axis = X
           split = 50
           ┌────┴────┐
          x<50      x≥50
           │          │
       axis = Y    axis = Y
        split=40    split=60
        ┌──┴──┐     ┌──┴──┐
       y<40  y≥40  y<60  y≥60
```

`quadtree.c`, `bsp_tree.c`, `kd_tree.c` use a **single-screen step-through
demo** (press Enter to advance one operation). `quad_tree_helloworld.c`
uses the **animated** version with phases — INSERT until full, then
QUERY with a bouncing rectangle. Both styles are useful; the step-
through helps you trace the algorithm exactly, the animated one builds
the gestalt.

### Computational geometry — hulls, Voronoi, visibility, marching

Four files (`convex_hull.c`, `voronoi.c`, `visibility_polygon.c`,
`marching_squares.c`) cover the **classical 2-D geometry kernel**:

* **Convex hull** — smallest enclosing polygon. Cross-product winding test.
* **Voronoi** — for each pixel, which seed is nearest? Brute force here;
  the file's CONCEPTS block points at Fortune's sweep for production.
* **Visibility polygon** — from an observer, which cells are unoccluded?
  Angular sweep over wall endpoints.
* **Marching squares** — given a scalar field, where is `f(x,y) = T`?
  16-case lookup on a 2×2 corner classification.

The shared primitive across the geometry files is the **2-D cross
product**:

```
(B − A) × (C − A) = (Bx−Ax)(Cy−Ay) − (By−Ay)(Cx−Ax)
```

* Convex hull: cross sign = winding direction → which point stays on the hull.
* Visibility: cross-product determinant = ray-segment intersection.
* Marching squares: corner classification by sign of `f(x,y) − threshold`.

Cross product as winding test is the **foundation primitive of 2-D
computational geometry**; learn it here once and it recurs in every
geometry file in the wider project.

### Graph search and networks

Two files (`graph_search.c`, `network_sim.c`) treat the same data
structure — an **adjacency-list graph** — with two very different
dynamics on top.

* `graph_search.c` runs **BFS / DFS / A\*** on a static random planar
  graph. The graph is built once (Fruchterman-Reingold force-directed
  layout for legibility) and the search is animated one frontier
  expansion per tick. The visual difference between BFS (level-by-level
  ripples), DFS (deep narrow strand), and A* (greedy beeline toward the
  goal) is the file's whole pedagogical point.
* `network_sim.c` runs an **SIR epidemic** on a Watts-Strogatz
  small-world network. Per tick: each infected node infects each
  susceptible neighbour with probability β; each infected node recovers
  with probability γ. The reproduction number `R0 = β·⟨k⟩/γ` controls
  whether the disease explodes or dies out. Left panel shows the network,
  right panel shows the stacked epidemic curve over time.

Both files use a **frontier / event queue** as their core state. The
difference is whether you're searching for a goal (graph_search) or
simulating a process (network_sim). Same data structure, opposite
loops.

### Sorting

`sort_vis.c` stands alone — the only non-geometric file in the folder
but the cleanest exemplar of the "one operation per tick" discipline.
Five comparison sorts (Bubble, Insertion, Selection, Quicksort,
Heapsort) all rewritten as iterative state machines that yield after
one compare-or-swap. Quicksort uses an explicit stack so you can pause
mid-partition. When the array is sorted every bar turns green.

The pedagogical payoff is **watching the same input data sorted five
different ways**. Bubble's slow rightward wave, selection's
distinctive ratcheting min-search, quicksort's recursive partition
ballet, heapsort's two-phase build-then-extract — each has a visual
signature that the implementation reveals more clearly than any
textbook diagram.

---

## Building and running

Every file is self-contained — no shared headers, single `gcc` per binary:

```bash
gcc -std=c11 -O2 -Wall -Wextra <file>.c -o <name> -lncurses -lm
```

The three step-through demos (`quadtree.c`, `bsp_tree.c`, `kd_tree.c`)
don't need `-lncurses` — they print to stdout and pause on Enter. Every
other file needs both `-lncurses` and `-lm`.

**Universal keys** (always present in the animated files):

| Key             | Action                                          |
|-----------------|-------------------------------------------------|
| `q` / `ESC`     | quit                                            |
| `space` / `p`   | pause / resume                                  |
| `r`             | reset / randomise                               |
| `+` / `-`       | speed up / slow down ticks                      |
| `]` / `[`       | sim Hz up / down                                |
| `TAB` / `a`     | cycle algorithm (sort_vis, graph_search)        |
| `t`             | cycle theme (where supported)                   |

**Per-file specials** — see each file's header block:

| File                    | Specific keys                                    |
|-------------------------|--------------------------------------------------|
| `sort_vis.c`            | TAB next algorithm, space scramble                |
| `quad_tree_helloworld.c`| `n` next phase, `r` full reset                    |
| `convex_hull.c`         | space new points                                  |
| `graph_search.c`        | `s` start, `a` cycle algorithm, `r` new graph     |
| `network_sim.c`         | ↑↓ β, ←→ γ, `i` inject infection                  |
| `marching_squares.c`    | `+/-` iso threshold, `m` multi-level toggle       |
| `visibility_polygon.c`  | `+/-` observer speed                              |
| `voronoi.c`             | `]/[` sim Hz                                      |

See the [grids/](../grids/README.md) folder for a similar
"one-discipline-per-folder" treatment of grid drawing and placement;
see [`procedural/generational/maze.c`](../procedural/generational/maze.c)
for the same state-machine pattern applied to recursive backtracker
maze carving.
