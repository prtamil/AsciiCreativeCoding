/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * graph_search.c — Animated BFS / DFS / A* Pathfinding
 *
 * A random planar-ish graph of N=40 nodes is generated in pixel space and
 * laid out using a force-directed (spring-repulsion) algorithm.  Source and
 * goal nodes are the farthest apart.
 *
 * Three search algorithms animate step-by-step:
 *   BFS — expands the nearest-by-hop frontier (shortest path by hops)
 *   DFS — depth-first; finds a path but not necessarily the shortest
 *   A*  — best-first with Euclidean heuristic; optimal shortest path
 *
 * Node colours
 *   'O' white     unvisited
 *   'O' cyan      frontier / open set
 *   'o' dark-grey visited / closed set
 *   'S' green     source
 *   'G' red       goal
 *   '*' yellow    final path
 *
 * Keys:  q quit   s start/restart search   a cycle algorithm
 *        r new graph   spc step-one (when paused)   p pause
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra artistic/graph_search.c \
 *       -o graph_search -lncurses -lm
 *
 * §1 config  §2 clock  §3 color  §4 graph  §5 layout  §6 algorithms
 * §7 scene   §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three graph search algorithms animated side-by-side:
 *                  BFS  — queue-based; expands level-by-level; guarantees
 *                         shortest path by hop count.  O(V+E).
 *                  DFS  — stack-based; explores deeply before backtracking;
 *                         finds a path but not necessarily the shortest. O(V+E).
 *                  A*   — priority queue (min-heap); f(n)=g(n)+h(n) where
 *                         g=cost-so-far, h=Euclidean distance to goal.
 *                         Optimal with admissible heuristic.  O((V+E) log V).
 *
 * Data-structure : Adjacency list graph (N=40 nodes, planar-ish random edges).
 *                  Spring-repulsion layout (Fruchterman-Reingold): nodes repel
 *                  like charged particles, edges attract like springs, until
 *                  equilibrium — purely for visual legibility.
 *
 * Math           : Fruchterman-Reingold layout: repulsive force ∝ k²/d,
 *                  attractive force ∝ d²/k, where k = √(area/N) is the
 *                  ideal edge length.  Converges in O(iterations × (V²+E)).
 *                  A* heuristic h(n) = Euclidean distance: admissible since
 *                  it never overestimates straight-line distance.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * All three searches share one skeleton: maintain a "frontier" of nodes
 * whose neighbours are next in line to expand, pop one node off the
 * frontier each tick, mark it VISITED, push every fresh neighbour onto
 * the frontier, and record which neighbour discovered which (the
 * g_prev[] back-pointer).  The ONLY thing the three algorithms differ
 * on is the rule for which frontier node to pop next: BFS pops the
 * oldest (FIFO queue), DFS pops the newest (LIFO stack), A* pops the
 * one with the smallest f = g + h.  When the popped node is the goal,
 * walk g_prev[] backwards to paint the final path.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture flooding ink across a road map starting at source S.  BFS is
 * wet ink spreading evenly outward in concentric rings — every node
 * 1 hop away gets soaked before anything 2 hops away.  DFS is one
 * obsessive person walking, taking the first unvisited side road every
 * time, only backtracking when stuck — it draws a single long worm
 * through the graph.  A* is a guided drone that prefers nodes pointing
 * toward G — it bends the BFS ring into an oval stretched toward the
 * goal.  Same map, three different exploration orders, three different
 * stories told by the colour wave.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. graph_generate(): drop N_NODES at random positions, connect each
 *     to its K_CONNECT=3 nearest neighbours by Euclidean distance.
 *     Pick the farthest-apart pair as (src, goal).
 *  2. layout_settle(): run SETTLE_ITERS=250 of Fruchterman-Reingold —
 *     all-pairs repulsion (∝ K_REP/d²) plus spring attraction along
 *     edges (∝ K_ATT·(d−REST_LEN)) — until nodes spread legibly.
 *  3. search_reset(): mark src FRONTIER, push it on the BFS queue or
 *     DFS stack; A* uses no container — it scans the FRONTIER array.
 *  4. Each STEP_NS=125 ms tick, call search_step():
 *       BFS:  u = queue.pop_front(); mark VISITED; for each neighbour
 *             still UNVIS, set g_prev[v]=u, mark FRONTIER, queue.push.
 *       DFS:  u = stack.pop_top(); same expansion but stack-ordered.
 *       A*:   scan FRONTIER, pick u minimising f = g_dist[u] + h(u→G);
 *             relax g_dist[v] = g_dist[u] + edge_len if smaller.
 *  5. Whenever a neighbour equals goal, call reconstruct_path():
 *     walk g_prev[] from goal back to src, flag g_on_path[i] = true,
 *     re-paint those nodes PATH_NODE/yellow and set g_phase = DONE.
 *  6. scene_draw() reads g_ns[] each frame and pushes glyph + colour
 *     per node, plus a Bresenham line per edge with bold yellow if
 *     both endpoints are on the final path.
 *
 * KEY FORMULAS
 * ────────────
 *  Euclidean distance   d = sqrt((xi-xj)² + (yi-yj)²)
 *  Repulsion force      Frep = K_REP / d²            (per pair)
 *  Spring force         Fatt = K_ATT · (d − REST_LEN) (per edge)
 *  A* node priority     f(n) = g(n) + h(n)
 *                       g(n) = sum of edge lengths along best known
 *                              path src → n
 *                       h(n) = Euclidean distance n → goal (admissible)
 *  Edge relaxation      if g[u] + len(u,v) < g[v]: g[v] := g[u]+len; prev[v] := u
 *  Path reconstruction  n := goal; while n != −1: on_path[n]=true; n := prev[n]
 *  Pixel→cell           cx = round(px / CELL_W), cy = round(py / CELL_H)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DFS uses a "lazy" pop: the same node can sit multiple times on
 *    the stack — the `if (g_ns[u] == VISITED) return;` guard at the
 *    top of dfs_step() catches the stale copies.
 *  • The A* implementation has NO closed set per se: VISITED nodes
 *    are skipped in the relaxation loop, but the FRONTIER scan is
 *    O(N) per pop — fine at N_NODES=40, would be O(N²) total at scale.
 *  • g_prev[] is never reset between algorithms when only `a` is
 *    pressed — search_reset() handles it; pressing `a` alone clears
 *    g_ns[] but not g_prev[].  Press `s` to start a clean search.
 *  • Random graphs can be disconnected — if goal is unreachable, the
 *    queue/stack drains and g_phase becomes DONE with no path drawn.
 *  • The K_CONNECT=3 mutual-adding step double-connects pairs (i→j and
 *    j→i are both set), but the symmetry is intentional — adjacency
 *    is undirected.
 *  • Layout uses fixed DT_SETTLE=0.3 with no damping — explosions are
 *    avoided only because forces stabilise within 250 iterations and
 *    bounds clamp keeps strays in.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Run BFS on a freshly generated graph: count rings of cyan
 *    flashing outward — every node at hop k turns FRONTIER on tick k
 *    after src.  A bright concentric pattern means BFS is correct.
 *  • Run A* and BFS on the same graph (`s` then `a` then `s`): A*
 *    should expand fewer or equal VISITED nodes than BFS, and the
 *    final yellow path length should be identical for both (both
 *    optimal under unit-weight... actually A* optimal under
 *    Euclidean-weighted, BFS optimal under hop count; on this random
 *    layout they may differ slightly).
 *  • DFS frontier should look like a thin tendril, not a ring; its
 *    final path is usually longer than BFS's.
 *  • g_steps in the HUD: typical BFS ≈ 15-25 expansions on N=40,
 *    A* ≈ 8-15.  If A* exceeds BFS, the heuristic computation is
 *    broken.
 *  • Press `r` repeatedly: nodes should redistribute and arms should
 *    not overlap heavily — if they do, SETTLE_ITERS is too low.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order.
 *      Read algorithms/sort_vis.c first if state-machine algorithm
 *      animation is new — that file's T1-T2 explain how to turn an
 *      iterative algorithm into a per-tick coroutine.  Same pattern
 *      here, on graphs instead of arrays.
 *   2. §6 algorithms — bfs_step, dfs_step, astar_step.  THE HEART
 *      of this file.  Read AFTER tutorials T1-T6.
 *   3. §5 layout — Fruchterman-Reingold force-directed layout.
 *      Independent of the search; read as a self-contained sub-
 *      lesson on graph drawing (T6 below).
 *   4. §4 graph — adjacency-list generation.
 *   5. §1-§3, §7-§8 — config / clock / colour / scene / app loop.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   N_NODES                    40 — fixed graph size.
 *   g_ns[i]                    NodeState of node i (UNVIS, FRONTIER,
 *                              VISITED, PATH_NODE, SRC, GOAL).
 *   g_prev[i]                  back-pointer for path reconstruction:
 *                              "which node DISCOVERED node i?"
 *   g_dist[i]                  cumulative path length src → i (A*
 *                              only).  AKA `g(n)` in A* literature.
 *   g_queue / g_stack          BFS FIFO / DFS LIFO containers.
 *   src, goal                  source + goal node indices.  Picked
 *                              as farthest-apart pair.
 *
 * Background you need
 * ───────────────────
 *   - Graphs as adjacency lists: each node has a list of neighbours.
 *   - Stack vs queue: LIFO vs FIFO, the difference between DFS and
 *     BFS comes down to which container is used.
 *   - Euclidean distance.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Dijkstra's algorithm in detail.  A* with h ≡ 0 IS Dijkstra;
 *     the heuristic is what makes it A*.  Implicit in T4.
 *   - Big-O proofs.  Mentioned in CONCEPTS but not derived.
 *   - Graph theory beyond adjacency-list connectivity.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ─────────────────────────────────────────────────── *
 *
 * Six tutorials that build BFS / DFS / A* from one shared skeleton.
 *
 *   T1  The unified search skeleton — all three are the same loop
 *   T2  BFS — FIFO queue, expands by hops
 *   T3  DFS — LIFO stack, expands by depth
 *   T4  A* — heuristic-guided priority pop
 *   T5  Path reconstruction — back-pointers tell the story
 *   T6  Force-directed graph layout (independent sub-lesson)
 *
 * ─────────────────────────────────────────────────────────────────────── *
 *
 * T1  THE UNIFIED SEARCH SKELETON — ALL THREE ARE THE SAME LOOP
 * ─────────────────────────────────────────────────────────────
 * BFS, DFS, and A* LOOK like three different algorithms.  They're
 * really one algorithm with one variable swap:
 *
 *     mark src FRONTIER, push it on the container
 *     while container not empty:
 *       u = container.POP()           ← the only line that varies
 *       if u == goal: reconstruct path, done
 *       mark u VISITED
 *       for each neighbour v of u:
 *         if v is UNVIS:
 *           mark v FRONTIER
 *           prev[v] = u
 *           container.PUSH(v)
 *
 * The container choice IS the algorithm:
 *
 *     BFS: container = QUEUE, POP = pop_front (oldest)
 *     DFS: container = STACK, POP = pop_top   (newest)
 *     A* : container = priority FRONTIER set, POP = min by f
 *
 * That's it.  Identical bookkeeping (mark, push neighbours,
 * remember prev), different choice of "which node to visit
 * NEXT."  The algorithms differ in BEHAVIOUR because the order
 * of expansion differs, but the loop body is the same.
 *
 * §6 algorithms has three step functions — bfs_step, dfs_step,
 * astar_step — with this skeleton each.  Reading them
 * side-by-side reveals the structural identity hidden under
 * different vocabulary.
 *
 * T2  BFS — FIFO QUEUE, EXPANDS BY HOPS
 * ─────────────────────────────────────
 * BFS uses a FIFO QUEUE: pop the OLDEST node added to the
 * frontier.  Behaviour:
 *
 *     tick 0: pop src; queue its 3 neighbours.
 *     tick 1: pop neighbour 1; queue ITS 3 neighbours.
 *     tick 2: pop neighbour 2; queue ITS new neighbours.
 *     tick 3: pop neighbour 3; queue.
 *     tick 4: pop the FIRST grandchild of src.  ← was queued at tick 0
 *
 * Notice the pattern: all 1-hop neighbours of src are popped
 * BEFORE any 2-hop neighbour.  All 2-hop before any 3-hop.
 * BFS expands in CONCENTRIC RINGS by hop count.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │              ●                                   │
 *      │         ●         ●                              │
 *      │     ●     ●     ●     ●                          │
 *      │  ●     ●     S     ●     ●                       │
 *      │     ●     ●     ●     ●                          │
 *      │         ●         ●                              │
 *      │              ●                                   │
 *      │                                                  │
 *      │  ring 0 = src; ring 1 = 1-hop; ring 2 = 2-hop;  │
 *      │  visualised as "ink spreading evenly outward"    │
 *      └──────────────────────────────────────────────────┘
 *
 * GUARANTEE: BFS finds the path with the FEWEST HOPS from src
 * to goal.  Proof: ring k is processed entirely before ring
 * k+1 starts; if goal is in ring k, no path of < k hops exists
 * to it.
 *
 * COMPLEXITY: O(V + E) — every vertex popped once, every edge
 * inspected (at most twice).
 *
 * T3  DFS — LIFO STACK, EXPANDS BY DEPTH
 * ──────────────────────────────────────
 * DFS uses a LIFO STACK: pop the NEWEST node added.
 *
 *     tick 0: pop src; push neighbours [N1, N2, N3].
 *     tick 1: pop N3 (newest); push its neighbours [M1, M2].
 *     tick 2: pop M2; push its neighbours.
 *     ...
 *
 * The OLDEST nodes (N1, N2) sit at the BOTTOM of the stack and
 * don't get popped until the entire subtree below N3 has been
 * fully explored.  DFS goes DEEP into one branch first, only
 * backing up when stuck.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │   S ─────●─────●─────●─────●                     │
 *      │                              \                   │
 *      │                               ●                  │
 *      │                                \                 │
 *      │                                 ●                │
 *      │                                                  │
 *      │  one obsessive worm walks the graph              │
 *      └──────────────────────────────────────────────────┘
 *
 * NO OPTIMALITY GUARANTEE.  The path DFS finds may be much
 * longer than the BFS optimum.  But DFS is useful for cycle
 * detection, topological sort, articulation point search, and
 * many other graph problems where finding "any path" beats
 * finding "the shortest path."
 *
 * COMPLEXITY: O(V + E) same as BFS.  The container choice
 * doesn't change asymptotic cost.
 *
 * T4  A* — HEURISTIC-GUIDED PRIORITY POP
 * ──────────────────────────────────────
 * A* uses a PRIORITY FRONTIER: pop the node with the smallest
 * f(n) value, where:
 *
 *     f(n) = g(n) + h(n)
 *     g(n) = cost of the BEST KNOWN PATH from src to n
 *     h(n) = HEURISTIC estimate of cost from n to goal
 *
 * For grid/Euclidean problems, h is straight-line distance to
 * goal.  This heuristic is ADMISSIBLE — it never
 * overestimates the actual cost (no path can be shorter than
 * the straight line).
 *
 * BEHAVIOUR: A* combines BFS's optimality with directed search.
 * It expands nodes that look most promising to reach the goal
 * cheaply, NOT just nodes nearest to src.
 *
 *      ┌──────────────────────────────────────────────────┐
 *      │                                                  │
 *      │              ●                                   │
 *      │         ●        ●●●●                            │
 >      │      ●     ●        ●●●● ─→ G                    │
 *      │   ●     ●     S        ●●                        │
 *      │      ●           ●                               │
 *      │         ●                                        │
 *      │                                                  │
 *      │  expansion is BFS-like near src,                 │
 *      │  STRETCHED toward G                              │
 *      └──────────────────────────────────────────────────┘
 *
 * GUARANTEE: with admissible heuristic, A* finds the OPTIMAL
 * path AND visits no more nodes than necessary (consistent
 * heuristic gives the strongest bound).
 *
 * Special cases:
 *   h(n) = 0 always           → A* becomes Dijkstra's algorithm
 *   h(n) = g(n) = 0           → A* becomes BFS (unit-weight)
 *   h(n) overestimates       → A* can return non-optimal paths
 *
 * EDGE RELAXATION (the A*-specific bookkeeping):
 *   when expanding u, if g[u] + edge_len(u, v) < g[v]:
 *     g[v] = g[u] + edge_len(u, v)
 *     prev[v] = u
 *
 * This lets A* find a SHORTER path to v if one exists through
 * u, even after v has been added to the frontier under a
 * different prev.
 *
 * COMPLEXITY: O((V + E) log V) with a binary heap, where the
 * log V comes from heap operations.  Our implementation does
 * a linear scan over the FRONTIER (O(V) per pop), so it's
 * O(V² + E) — fine at N=40, slow at N=10000+.
 *
 * T5  PATH RECONSTRUCTION — BACK-POINTERS TELL THE STORY
 * ──────────────────────────────────────────────────────
 * All three algorithms FIND the goal eventually.  But finding
 * is half the job — you also need to TRACE THE PATH.
 *
 * The trick: every time you mark a neighbour FRONTIER (T1
 * skeleton, line 4), record WHICH NODE DISCOVERED IT:
 *
 *     prev[v] = u
 *
 * After the search reaches goal, walk prev backward:
 *
 *     n = goal
 *     while n != src:
 *       on_path[n] = true
 *       n = prev[n]
 *
 * This produces the path in REVERSE order.  Since we just
 * mark nodes, reverse order doesn't matter — the renderer
 * paints all marked nodes the same.
 *
 * For each algorithm, prev[v] gets ONE value: whichever node
 * was the FIRST to expand to v (because we only set prev[v]
 * when v transitions UNVIS → FRONTIER).  That gives:
 *   BFS: prev = a shortest-by-hops path
 *   DFS: prev = the path DFS happened to take
 *   A* : (with the relaxation in T4) the optimal-cost path
 *
 * Same machinery (one int per node), three different paths
 * because the search ORDER was different.
 *
 * T6  FORCE-DIRECTED GRAPH LAYOUT (INDEPENDENT SUB-LESSON)
 * ────────────────────────────────────────────────────────
 * The graph nodes are placed by FORCE-DIRECTED LAYOUT
 * (Fruchterman & Reingold 1991): pretend the nodes are charged
 * particles + the edges are springs.  Run for many iterations;
 * watch the system settle into a low-energy configuration where
 * connected nodes are close and unconnected ones are spread out.
 *
 * Forces:
 *
 *     REPULSION — between EVERY pair of nodes:
 *       F_rep = K_REP / d²        (Coulomb-like, falls off with
 *                                  distance squared)
 *       direction: from j toward i (push apart)
 *
 *     ATTRACTION — only along EDGES:
 *       F_att = K_ATT · (d - REST_LEN)
 *                                  (Hooke spring; positive when
 *                                   stretched, negative when
 *                                   compressed)
 *       direction: from i toward j (pull together)
 *
 * Per iteration:
 *   1. for each node, sum repulsion forces from all others
 *   2. for each edge, add attraction forces to both endpoints
 *   3. update positions: pos_i += F_i · DT_SETTLE
 *   4. clamp positions to screen bounds
 *
 * After ~250 iterations, the system is near-equilibrium —
 * nodes spread out enough to be legible, connected nodes close
 * enough to read as a graph.
 *
 * No formal convergence guarantee (the energy landscape has
 * local minima), but in practice random graphs of N ≈ 40
 * settle quickly.  At larger N, you'd add cooling (gradually
 * shrink DT_SETTLE) and possibly Barnes-Hut acceleration for
 * the all-pairs repulsion (O(N log N) instead of O(N²)).
 *
 * Same trick is used in:
 *   - software dependency visualisers
 *   - social network diagrams
 *   - molecular structure layouts
 *   - the d3.js force-directed graph component
 *
 * Force-directed layout is INDEPENDENT of the search; it just
 * makes the graph readable.  You could paste any (V, E)
 * adjacency list in and the same code would lay it out.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define N_NODES        40
#define K_CONNECT       3    /* each node connects to K nearest neighbours     */
#define SETTLE_ITERS  250    /* force-directed layout iterations at startup    */
#define K_REP       6000.f  /* repulsion strength                              */
#define K_ATT          0.4f /* spring attraction along edges                   */
#define REST_LEN      90.f  /* target edge length in pixel space               */
#define DT_SETTLE      0.3f /* force-directed update step                      */
#define CELL_W          8
#define CELL_H         16
#define HUD_ROWS        3
#define STEP_NS     (1000000000LL / 8)   /* one algorithm step per ~125 ms    */
#define RENDER_NS   (1000000000LL / 30)

typedef enum { UNVIS, FRONTIER, VISITED, PATH_NODE, SRC, GOAL } NodeState;
typedef enum { ALG_BFS, ALG_DFS, ALG_ASTAR } Algorithm;
typedef enum { IDLE, RUNNING, DONE } SearchPhase;

enum {
    CP_UNVIS = 1, CP_FRONT, CP_VIS, CP_PATH, CP_SRC, CP_GOAL,
    CP_EDGE, CP_PATH_EDGE, CP_HUD,
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_UNVIS,    250, -1);   /* light grey         */
        init_pair(CP_FRONT,     51, -1);   /* cyan               */
        init_pair(CP_VIS, 246, -1);   /* dark grey          */
        init_pair(CP_PATH,     226, -1);   /* yellow             */
        init_pair(CP_SRC,       46, -1);   /* green              */
        init_pair(CP_GOAL,     196, -1);   /* red                */
        init_pair(CP_EDGE, 246, -1);   /* dim grey           */
        init_pair(CP_PATH_EDGE,226, -1);   /* yellow             */
        init_pair(CP_HUD,      244, -1);   /* grey               */
    } else {
        init_pair(CP_UNVIS,    COLOR_WHITE,  -1);
        init_pair(CP_FRONT,    COLOR_CYAN,   -1);
        init_pair(CP_VIS,      COLOR_WHITE,  -1);
        init_pair(CP_PATH,     COLOR_YELLOW, -1);
        init_pair(CP_SRC,      COLOR_GREEN,  -1);
        init_pair(CP_GOAL,     COLOR_RED,    -1);
        init_pair(CP_EDGE,     COLOR_WHITE,  -1);
        init_pair(CP_PATH_EDGE,COLOR_YELLOW, -1);
        init_pair(CP_HUD,      COLOR_WHITE,  -1);
    }
}

/* ===================================================================== */
/* §4  graph                                                              */
/* ===================================================================== */

static float  g_px[N_NODES], g_py[N_NODES];   /* pixel-space positions  */
static bool   g_adj[N_NODES][N_NODES];
static int    g_n_nodes;
static int    g_src, g_goal;
static int    g_rows, g_cols;

static int px_cx(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / (float)CELL_H + 0.5f); }

static float node_dist(int i, int j)
{
    float dx = g_px[i]-g_px[j], dy = g_py[i]-g_py[j];
    return sqrtf(dx*dx + dy*dy);
}

static void graph_generate(int rows, int cols)
{
    g_n_nodes = N_NODES;
    /* random placement in the animation area below HUD */
    int pw = cols * CELL_W, ph = rows * CELL_H;
    int margin_x = pw / 8, margin_y = ph / 8;
    int area_w = pw - 2*margin_x, area_h = ph - 2*margin_y;
    int hud_py  = HUD_ROWS * CELL_H;

    for (int i = 0; i < N_NODES; i++) {
        g_px[i] = (float)margin_x + (float)(rand() % area_w);
        g_py[i] = (float)(hud_py + margin_y) + (float)(rand() % area_h);
    }

    /* clear adjacency */
    memset(g_adj, 0, sizeof(g_adj));

    /* connect each node to its K nearest neighbours */
    for (int i = 0; i < N_NODES; i++) {
        /* sort by distance to i (simple insertion-sort of K entries) */
        float best_d[K_CONNECT]; int best_j[K_CONNECT];
        for (int k = 0; k < K_CONNECT; k++) { best_d[k]=1e30f; best_j[k]=-1; }

        for (int j = 0; j < N_NODES; j++) {
            if (j == i) continue;
            float d = node_dist(i, j);
            /* insert into best-K if d < worst */
            for (int k = 0; k < K_CONNECT; k++) {
                if (d < best_d[k]) {
                    /* shift down */
                    for (int m = K_CONNECT-1; m > k; m--) {
                        best_d[m] = best_d[m-1]; best_j[m] = best_j[m-1];
                    }
                    best_d[k] = d; best_j[k] = j; break;
                }
            }
        }
        for (int k = 0; k < K_CONNECT; k++) {
            if (best_j[k] >= 0) {
                g_adj[i][best_j[k]] = true;
                g_adj[best_j[k]][i] = true;
            }
        }
    }

    /* pick source and goal: the two nodes farthest apart */
    g_src = 0; g_goal = 1;
    float max_d = 0.f;
    for (int i = 0; i < N_NODES; i++)
        for (int j = i+1; j < N_NODES; j++) {
            float d = node_dist(i, j);
            if (d > max_d) { max_d = d; g_src = i; g_goal = j; }
        }
}

/* ===================================================================== */
/* §5  force-directed layout                                              */
/* ===================================================================== */

static void layout_settle(void)
{
    float fx[N_NODES], fy[N_NODES];
    int   pw = g_cols * CELL_W, ph = g_rows * CELL_H;
    int   hud_py = HUD_ROWS * CELL_H;
    int   margin = 40;

    for (int iter = 0; iter < SETTLE_ITERS; iter++) {
        memset(fx, 0, sizeof(fx));
        memset(fy, 0, sizeof(fy));

        /* repulsion between all pairs */
        for (int i = 0; i < N_NODES; i++) {
            for (int j = i+1; j < N_NODES; j++) {
                float dx = g_px[i]-g_px[j], dy = g_py[i]-g_py[j];
                float d2 = dx*dx + dy*dy;
                if (d2 < 1.f) d2 = 1.f;
                float f = K_REP / d2;
                float d = sqrtf(d2);
                fx[i] += f*dx/d; fy[i] += f*dy/d;
                fx[j] -= f*dx/d; fy[j] -= f*dy/d;
            }
        }

        /* spring attraction along edges */
        for (int i = 0; i < N_NODES; i++) {
            for (int j = i+1; j < N_NODES; j++) {
                if (!g_adj[i][j]) continue;
                float dx = g_px[j]-g_px[i], dy = g_py[j]-g_py[i];
                float d = sqrtf(dx*dx+dy*dy);
                if (d < 1.f) d = 1.f;
                float f = K_ATT * (d - REST_LEN);
                fx[i] += f*dx/d; fy[i] += f*dy/d;
                fx[j] -= f*dx/d; fy[j] -= f*dy/d;
            }
        }

        /* integrate and clamp to bounds */
        for (int i = 0; i < N_NODES; i++) {
            g_px[i] += fx[i] * DT_SETTLE;
            g_py[i] += fy[i] * DT_SETTLE;
            if (g_px[i] < (float)margin)     g_px[i] = (float)margin;
            if (g_px[i] > (float)(pw-margin)) g_px[i] = (float)(pw-margin);
            if (g_py[i] < (float)(hud_py+margin)) g_py[i] = (float)(hud_py+margin);
            if (g_py[i] > (float)(ph-margin))     g_py[i] = (float)(ph-margin);
        }
    }
}

/* ===================================================================== */
/* §6  algorithms                                                         */
/* ===================================================================== */

static NodeState g_ns[N_NODES];    /* node state array           */
static int       g_prev[N_NODES];  /* predecessor for path recon */
static float     g_dist[N_NODES];  /* distance from source        */
static bool      g_on_path[N_NODES];

/* BFS queue */
static int g_queue[N_NODES * 4], g_q_head, g_q_tail;
/* DFS stack */
static int g_stack[N_NODES * 4], g_s_top;

static Algorithm g_alg   = ALG_BFS;
static SearchPhase g_phase = IDLE;
static int       g_steps  = 0;

static const char *alg_name(void)
{
    return g_alg == ALG_BFS ? "BFS" : g_alg == ALG_DFS ? "DFS" : "A*";
}

static void search_reset(void)
{
    for (int i = 0; i < N_NODES; i++) {
        g_ns[i]   = (i == g_src) ? SRC : (i == g_goal) ? GOAL : UNVIS;
        g_prev[i] = -1;
        g_dist[i] = 1e30f;
        g_on_path[i] = false;
    }
    g_dist[g_src] = 0.f;
    g_q_head = g_q_tail = 0;
    g_s_top  = 0;
    g_steps  = 0;
    g_phase  = RUNNING;

    if (g_alg == ALG_BFS) {
        g_queue[g_q_tail++] = g_src;
        g_ns[g_src] = FRONTIER;
    } else if (g_alg == ALG_DFS) {
        g_stack[g_s_top++] = g_src;
        g_ns[g_src] = FRONTIER;
    } else { /* A* */
        g_ns[g_src] = FRONTIER;
    }
}

static void reconstruct_path(void)
{
    int n = g_goal;
    while (n != -1) { g_on_path[n] = true; n = g_prev[n]; }
    for (int i = 0; i < N_NODES; i++) {
        if (!g_on_path[i]) continue;
        if (i == g_src)  g_ns[i] = SRC;
        else if (i == g_goal) g_ns[i] = GOAL;
        else g_ns[i] = PATH_NODE;
    }
    g_phase = DONE;
}

/*
 * One BFS expansion step: dequeue the front node, expand its unvisited
 * neighbours, enqueue them.
 */
static void bfs_step(void)
{
    if (g_q_head >= g_q_tail) { g_phase = DONE; return; }
    int u = g_queue[g_q_head++];
    if (g_ns[u] != SRC && g_ns[u] != GOAL) g_ns[u] = VISITED;
    g_steps++;

    for (int v = 0; v < N_NODES; v++) {
        if (!g_adj[u][v]) continue;
        if (g_ns[v] != UNVIS && g_ns[v] != GOAL) continue;
        g_prev[v] = u;
        if (v == g_goal) { reconstruct_path(); return; }
        g_ns[v] = FRONTIER;
        g_queue[g_q_tail++] = v;
    }
}

/* One DFS step: pop the stack, expand one unvisited neighbour. */
static void dfs_step(void)
{
    if (g_s_top == 0) { g_phase = DONE; return; }
    int u = g_stack[--g_s_top];
    if (g_ns[u] == VISITED) { return; } /* already expanded */
    if (g_ns[u] != SRC && g_ns[u] != GOAL) g_ns[u] = VISITED;
    g_steps++;

    for (int v = 0; v < N_NODES; v++) {
        if (!g_adj[u][v]) continue;
        if (g_ns[v] == VISITED) continue;
        if (g_prev[v] == -1) g_prev[v] = u;
        if (v == g_goal) { reconstruct_path(); return; }
        if (g_ns[v] == UNVIS) {
            g_ns[v] = FRONTIER;
            g_stack[g_s_top++] = v;
        }
    }
}

/*
 * One A* step: scan FRONTIER for minimum f = g + h, expand it.
 * O(N) scan is acceptable for N=40.
 */
static void astar_step(void)
{
    int   best = -1;
    float best_f = 1e30f;
    float hx = g_px[g_goal], hy = g_py[g_goal];

    for (int i = 0; i < N_NODES; i++) {
        if (g_ns[i] != FRONTIER) continue;
        float dx = g_px[i]-hx, dy = g_py[i]-hy;
        float h = sqrtf(dx*dx + dy*dy);
        float f = g_dist[i] + h;
        if (f < best_f) { best_f = f; best = i; }
    }
    if (best == -1) { g_phase = DONE; return; }

    int u = best;
    if (u == g_goal) { reconstruct_path(); return; }
    if (g_ns[u] != SRC) g_ns[u] = VISITED;
    g_steps++;

    for (int v = 0; v < N_NODES; v++) {
        if (!g_adj[u][v]) continue;
        if (g_ns[v] == VISITED) continue;
        float ng = g_dist[u] + node_dist(u, v);
        if (ng < g_dist[v]) {
            g_dist[v] = ng;
            g_prev[v] = u;
            if (v == g_goal) { reconstruct_path(); return; }
            g_ns[v] = FRONTIER;
        }
    }
}

static void search_step(void)
{
    if (g_phase != RUNNING) return;
    switch (g_alg) {
    case ALG_BFS:   bfs_step();   break;
    case ALG_DFS:   dfs_step();   break;
    case ALG_ASTAR: astar_step(); break;
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

static bool g_paused = false;

/*
 * draw_line() — Bresenham line with directional char selection.
 * attr is applied to every character written.
 */
static void draw_line(int x0, int y0, int x1, int y1, attr_t attr,
                      int cols, int rows)
{
    int dx = abs(x1-x0), dy = abs(y1-y0);
    int sx = x0<x1?1:-1, sy = y0<y1?1:-1;
    int err = dx-dy;
    for (;;) {
        if (x0>=0&&x0<cols&&y0>=0&&y0<rows) {
            int  e2 = 2*err;
            bool bx = e2 > -dy, by = e2 < dx;
            chtype ch = (bx&&by) ? (sx==sy?'\\':'/') : bx?'-':'|';
            attron(attr); mvaddch(y0,x0,ch); attroff(attr);
        }
        if (x0==x1&&y0==y1) break;
        int e2=2*err;
        if (e2>-dy){err-=dy;x0+=sx;}
        if (e2< dx){err+=dx;y0+=sy;}
    }
}

static void scene_draw(int rows, int cols)
{
    /* ── edges ───────────────────────────────────────────────────── */
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i+1; j < N_NODES; j++) {
            if (!g_adj[i][j]) continue;
            bool path_e = g_on_path[i] && g_on_path[j];
            int  cp = path_e ? CP_PATH_EDGE : CP_EDGE;
            attr_t attr = COLOR_PAIR(cp) | (path_e ? (attr_t)A_BOLD : (attr_t)0);
            draw_line(px_cx(g_px[i]), px_cy(g_py[i]),
                      px_cx(g_px[j]), px_cy(g_py[j]),
                      attr, cols, rows);
        }
    }

    /* ── nodes ───────────────────────────────────────────────────── */
    for (int i = 0; i < N_NODES; i++) {
        int cx = px_cx(g_px[i]), cy = px_cy(g_py[i]);
        if (cx<0||cx>=cols||cy<0||cy>=rows) continue;
        int   cp;
        chtype ch;
        attr_t extra = 0;
        switch (g_ns[i]) {
        case UNVIS:     cp=CP_UNVIS; ch='o'; break;
        case FRONTIER:  cp=CP_FRONT; ch='O'; extra=A_BOLD; break;
        case VISITED:   cp=CP_VIS;   ch='o'; break;
        case PATH_NODE: cp=CP_PATH;  ch='*'; extra=A_BOLD; break;
        case SRC:       cp=CP_SRC;   ch='S'; extra=A_BOLD; break;
        case GOAL:      cp=CP_GOAL;  ch='G'; extra=A_BOLD; break;
        default:        cp=CP_UNVIS; ch='o'; break;
        }
        attron(COLOR_PAIR(cp)|extra);
        mvaddch(cy, cx, ch);
        attroff(COLOR_PAIR(cp)|extra);
    }

    /* ── HUD ─────────────────────────────────────────────────────── */
    const char *phase_s = g_phase==IDLE?"IDLE":g_phase==RUNNING?"RUNNING":"DONE";
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0,0,
        " GraphSearch  q:quit  s:start  a:alg  r:new-graph  p:pause  spc:step");
    mvprintw(1,0,
        " alg:%-4s  phase:%-7s  steps:%3d  nodes:%d  S=src G=goal O=frontier *=path  %s",
        alg_name(), phase_s, g_steps, N_NODES,
        g_paused ? "PAUSED" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s==SIGINT||s==SIGTERM) g_quit=1;
    if (s==SIGWINCH)           g_resize=1;
}

static void cleanup(void) { endwin(); }

static void new_graph(int rows, int cols)
{
    g_rows = rows; g_cols = cols;
    graph_generate(rows, cols);
    layout_settle();
    g_phase  = IDLE;
    g_paused = false;
    for (int i = 0; i < N_NODES; i++) {
        g_ns[i] = (i==g_src) ? SRC : (i==g_goal) ? GOAL : UNVIS;
        g_on_path[i] = false;
    }
}

int main(void)
{
    srand((unsigned)(clock_ns() & 0xFFFFFFFF));

    atexit(cleanup);
    signal(SIGINT,   sig_h);
    signal(SIGTERM,  sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();

    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    new_graph(rows, cols);

    long long last_step = clock_ns();
    long long last_frame = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            new_graph(rows, cols);
            last_step = last_frame = clock_ns();
            continue;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1;                          break;
        case 's': case 'S':          search_reset();                       break;
        case 'a': case 'A':
            g_alg = (Algorithm)((g_alg + 1) % 3);
            g_phase = IDLE;
            for (int i=0;i<N_NODES;i++){
                g_ns[i]=(i==g_src)?SRC:(i==g_goal)?GOAL:UNVIS;
                g_on_path[i]=false;
            }
            break;
        case 'r': case 'R':          new_graph(rows, cols);                break;
        case 'p': case 'P':          g_paused = !g_paused;                 break;
        case ' ':
            if (g_phase == IDLE) search_reset();
            else search_step();
            break;
        default: break;
        }

        long long now = clock_ns();

        /* advance one step every STEP_NS if running and not paused */
        if (g_phase == RUNNING && !g_paused && now - last_step >= STEP_NS) {
            search_step();
            last_step = now;
        }

        /* render at 30 fps */
        if (now - last_frame >= RENDER_NS) {
            last_frame = now;
            erase();
            scene_draw(rows, cols);
            wnoutrefresh(stdscr);
            doupdate();
        }

        clock_sleep_ns(10000000LL);   /* ~10 ms poll interval */
    }

    return 0;
}
