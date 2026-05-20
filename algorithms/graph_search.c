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
 * References
 * ──────────
 *   ── Search algorithms (BFS / DFS / A* — §6) ─────────────────────
 *   [1] Hart, P. E., Nilsson, N. J. & Raphael, B. (1968), "A Formal
 *       Basis for the Heuristic Determination of Minimum Cost Paths",
 *       IEEE Trans. Systems Science and Cybernetics 4(2), pp. 100-107
 *       — the ORIGINAL A* paper.  Formalises admissibility and proves
 *       optimality.  Surprisingly readable for a 1968 paper.
 *   [2] Dijkstra, E. W. (1959), "A Note on Two Problems in Connexion
 *       with Graphs", Numerische Mathematik 1(1), pp. 269-271 — the
 *       3-page paper that defined shortest-path search.  A* with
 *       h ≡ 0 IS Dijkstra; read this to see what A* generalises.
 *   [3] Cormen, T. H., Leiserson, C. E., Rivest, R. L. & Stein, C.
 *       (2009), "Introduction to Algorithms" (3rd ed.), MIT Press —
 *       Ch. 22 (Elementary Graph Algorithms: BFS, DFS) and Ch. 24
 *       (Single-Source Shortest Paths: Dijkstra, Bellman-Ford).  The
 *       standard reference; rigorous proofs of correctness + complexity.
 *   [4] Russell, S. & Norvig, P. (2020), "Artificial Intelligence: A
 *       Modern Approach" (4th ed.), Pearson — Ch. 3 covers BFS, DFS,
 *       Dijkstra and A* in one unified "search" framework.  The
 *       clearest treatment of "the algorithm IS the frontier rule".
 *   [5] Sedgewick, R. & Wayne, K. (2011), "Algorithms" (4th ed.),
 *       Addison-Wesley — Ch. 4 "Graphs" walks through BFS/DFS/Dijkstra
 *       with full Java implementations and complexity analysis.  Most
 *       practical of the three textbooks if you want working code.
 *
 *   ── Force-directed graph layout (§5) ────────────────────────────
 *   [6] Fruchterman, T. M. J. & Reingold, E. M. (1991), "Graph drawing
 *       by force-directed placement", Software: Practice and Experience
 *       21(11), pp. 1129-1164 — the LAYOUT used in §5.  Defines the
 *       k = √(area/N) ideal edge length and the cooling schedule we
 *       omit.  ~12 readable pages.
 *   [7] Eades, P. (1984), "A Heuristic for Graph Drawing", Congressus
 *       Numerantium 42, pp. 149-160 — the spring-model predecessor
 *       to F&R.  Useful for the historical motivation: graphs as
 *       physical systems.
 *   [8] Di Battista, G., Eades, P., Tamassia, R. & Tollis, I. G. (1999),
 *       "Graph Drawing: Algorithms for the Visualization of Graphs",
 *       Prentice Hall — the canonical graph-drawing textbook.  Ch. 10
 *       surveys force-directed methods including barycentric, simulated
 *       annealing, and magnetic-field variants.
 *   [9] Barnes, J. & Hut, P. (1986), "A hierarchical O(N log N) force-
 *       calculation algorithm", Nature 324, pp. 446-449 — how to scale
 *       the all-pairs repulsion in F&R from O(N²) to O(N log N) for
 *       large N.  Not needed at N=40 but the standard speed-up.
 *
 *   ── Rendering (§7) ──────────────────────────────────────────────
 *  [10] Bresenham, J. E. (1965), "Algorithm for computer control of a
 *       digital plotter", IBM Systems Journal 4(1), pp. 25-30 — the
 *       integer-only line rasterisation used by draw_edge_segment.
 *       The directional-glyph trick (`/\|-`) is a small extension.
 *  [11] Foley, J. D., van Dam, A., Feiner, S. K. & Hughes, J. F. (1995),
 *       "Computer Graphics: Principles and Practice" (2nd ed.),
 *       Addison-Wesley — §3.2 covers DDA, Bresenham, and run-length
 *       line rasterisation with clear pseudocode and figures.
 *
 *   ── Online quick reference ─────────────────────────────────────
 *  [12] Patel, A. (Red Blob Games), "Introduction to A*" —
 *       https://www.redblobgames.com/pathfinding/a-star/introduction.html
 *       — the best interactive online explanation of A*.  Read after
 *       [1] to see the algorithm animated in your browser.  Pairs
 *       well with this file's step-by-step terminal animation.
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
 * prev[] back-pointer).  The ONLY thing the three algorithms differ
 * on is the rule for which frontier node to pop next: BFS pops the
 * oldest (FIFO queue), DFS pops the newest (LIFO stack), A* pops the
 * one with the smallest f = g + h.  When the popped node is the goal,
 * walk prev[] backwards to paint the final path.
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
 *             still UNVIS, set prev[v]=u, mark FRONTIER, queue.push.
 *       DFS:  u = stack.pop_top(); same expansion but stack-ordered.
 *       A*:   scan FRONTIER, pick u minimising f = cost[u] + h(u→G);
 *             relax cost[v] = cost[u] + edge_len if smaller.
 *             (cost[] is g(n) in A* literature.)
 *  5. Whenever a neighbour equals goal, call reconstruct_path():
 *     walk prev[] from goal back to src, flag on_path[i] = true,
 *     re-paint those nodes PATH_NODE/yellow and set phase = DONE.
 *  6. scene_draw() reads state[] each frame and pushes glyph + colour
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
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first.
 *   2. §6 algorithms — bfs_step, dfs_step, astar_step.  THE HEART
 *      of this file.  Each step function carries its own pseudocode
 *      docblock; read those before the bodies.
 *   3. §5 layout — Fruchterman-Reingold force-directed layout.
 *      Independent of the search; read as a self-contained sub-
 *      lesson on graph drawing.
 *   4. §4 graph — Scene struct + adjacency-list generation.
 *   5. §1-§3, §7-§8 — config / clock / colour / scene / app loop.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   N_NODES                    40 — fixed graph size.
 *   sc->state[i]               NodeState of node i (UNVIS, FRONTIER,
 *                              VISITED, PATH_NODE, SRC, GOAL).
 *   sc->prev[i]                back-pointer for path reconstruction:
 *                              "which node DISCOVERED node i?"
 *   sc->cost[i]                cumulative path length src → i (A*
 *                              only).  AKA `g(n)` in A* literature.
 *   sc->queue / sc->stack      BFS FIFO / DFS LIFO containers
 *                              (NodeQueue / NodeStack from §4).
 *   sc->src, sc->goal          source + goal node indices.  Picked
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
 *     the heuristic is what makes it A* (see References [1], [2]).
 *   - Big-O proofs.  Mentioned in CONCEPTS but not derived.
 *   - Graph theory beyond adjacency-list connectivity.
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
/* §4  graph — data structures, generation                                */
/* ===================================================================== */

/*
 * Vec2 — 2-D pixel-space position. (See CELL_W / CELL_H in §1 for the
 *   pixel-to-cell conversion.) Y axis points DOWN (terminal convention).
 *   Used for node positions and layout forces alike — the integrator
 *   adds a force vector to a position vector each tick, so storing both
 *   as Vec2 keeps the math symmetric.
 */
typedef struct { float x, y; } Vec2;

/*
 * NodeQueue — FIFO container for BFS.
 *
 *   buf[head .. tail)   queued node indices in arrival order.
 *   push    appends at tail, pop removes from head; both O(1).
 *   empty   when head >= tail.
 *
 *   Sized 4×N_NODES of slack (BFS never reinserts, but the over-allocation
 *   costs nothing and matches the stack's lazy-push budget).
 */
typedef struct {
    int buf[N_NODES * 4];
    int head, tail;
} NodeQueue;

/*
 * NodeStack — LIFO container for DFS.
 *
 *   buf[0 .. top)   stacked node indices in push order.
 *   push    appends at top, pop removes from top; both O(1).
 *   empty   when top == 0.
 *
 *   Sized 4×N_NODES so DFS can LAZILY push neighbours without checking
 *   for duplicates — the pop-time VISITED guard skips stale entries.
 *   Lazy push keeps the inner loop branchless.
 */
typedef struct {
    int buf[N_NODES * 4];
    int top;
} NodeStack;

/*
 * Scene — owns ALL simulation, search, and render state for one run.
 *
 *   ── Graph topology + geometry ──
 *      pos[N_NODES]     pixel-space positions (Vec2)
 *      adj[N][N]        undirected adjacency matrix
 *      src, goal        endpoint indices (farthest-apart pair)
 *      rows, cols       terminal extent in CHARACTER CELLS
 *
 *   ── Per-node search state ──
 *      state[]          NodeState (UNVIS, FRONTIER, VISITED, PATH_NODE, SRC, GOAL)
 *      prev[]           predecessor for path reconstruction (-1 if none)
 *      cost[]           g(n) — best known path cost from src; used by A*
 *      on_path[]        marked by reconstruct_path() once goal is reached
 *
 *   ── Algorithm containers ──
 *      queue            BFS FIFO
 *      stack            DFS LIFO
 *      (A* uses no container — it scans state[] for the min-f FRONTIER)
 *
 *   ── Mode / progress ──
 *      alg              ALG_BFS / ALG_DFS / ALG_ASTAR
 *      phase            IDLE / RUNNING / DONE
 *      steps            expansion counter shown in the HUD
 *      paused           pause flag (toggled by 'p')
 *
 * One instance lives in main(), passed by pointer to every algorithm /
 * layout / draw / init function. The signal-handler flags (g_quit,
 * g_resize) in §8 stay as file-scope globals because POSIX signal
 * handlers can't be passed a pointer.
 */
typedef struct {
    /* Topology + geometry */
    Vec2        pos[N_NODES];
    bool        adj[N_NODES][N_NODES];
    int         src, goal;
    int         rows, cols;

    /* Per-node search state */
    NodeState   state[N_NODES];
    int         prev[N_NODES];
    float       cost[N_NODES];
    bool        on_path[N_NODES];

    /* Containers */
    NodeQueue   queue;
    NodeStack   stack;

    /* Mode */
    Algorithm   alg;
    SearchPhase phase;
    int         steps;
    bool        paused;
} Scene;

/*
 * px_cx / px_cy — pixel-space → cell-space rounding.
 *   The renderer needs (row, col) cell indices; the simulation stores
 *   pixel positions. The +0.5f rounds to nearest cell.
 */
static int px_cx(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / (float)CELL_H + 0.5f); }

/*
 * node_dist — Euclidean distance between two nodes in pixel space.
 *   Used as edge weight in A* (true geometric cost) and as the
 *   reference length in the layout's spring force.
 */
static float node_dist(const Scene *sc, int i, int j)
{
    float dx = sc->pos[i].x - sc->pos[j].x;
    float dy = sc->pos[i].y - sc->pos[j].y;
    return sqrtf(dx*dx + dy*dy);
}

/*
 * scatter_nodes_random — uniform random placement in the play area.
 *
 *   Play area = terminal pixel-rect MINUS:
 *     - top HUD strip (HUD_ROWS rows)
 *     - 1/8 inset on each side (keeps nodes off the screen edge so the
 *       layout integrator has room to spread them out before clamping)
 */
static void scatter_nodes_random(Scene *sc)
{
    int pw = sc->cols * CELL_W, ph = sc->rows * CELL_H;
    int margin_x = pw / 8, margin_y = ph / 8;
    int area_w = pw - 2*margin_x, area_h = ph - 2*margin_y;
    int hud_py = HUD_ROWS * CELL_H;
    if (area_w < 1) area_w = 1;
    if (area_h < 1) area_h = 1;

    for (int i = 0; i < N_NODES; i++) {
        sc->pos[i].x = (float)margin_x + (float)(rand() % area_w);
        sc->pos[i].y = (float)(hud_py + margin_y) + (float)(rand() % area_h);
    }
}

/*
 * insert_into_k_nearest — maintain a sorted "K smallest values" buffer.
 *
 *   Pseudocode:
 *     for k in 0..K−1:
 *       if d < best_d[k]:
 *         shift best_d[k..K−2] right one slot
 *         best_d[k] := d;  best_j[k] := j
 *         return
 *
 *   Used by connect_to_k_nearest. O(K) per insert, O(N · K) per node.
 *   At K_CONNECT=3 this is much cheaper than sorting all N candidates.
 */
static void insert_into_k_nearest(float *best_d, int *best_j, int K,
                                  float d, int j)
{
    for (int k = 0; k < K; k++) {
        if (d < best_d[k]) {
            for (int m = K - 1; m > k; m--) {
                best_d[m] = best_d[m-1];
                best_j[m] = best_j[m-1];
            }
            best_d[k] = d;
            best_j[k] = j;
            return;
        }
    }
}

/*
 * connect_to_k_nearest — add undirected edges from node i to its K closest.
 *
 *   Pseudocode:
 *     init best_d[0..K−1] := +inf;  best_j[0..K−1] := −1
 *     for every other node j:
 *       insert_into_k_nearest(node_dist(i, j), j)
 *     for k in 0..K−1:
 *       adj[i][best_j[k]] := adj[best_j[k]][i] := true   ← undirected
 *
 *   K=3 keeps the graph sparse and planar-ish; higher K makes a denser
 *   web that's harder to read but easier for search to traverse.
 */
static void connect_to_k_nearest(Scene *sc, int i, int K)
{
    float best_d[K_CONNECT];
    int   best_j[K_CONNECT];
    for (int k = 0; k < K; k++) { best_d[k] = 1e30f; best_j[k] = -1; }

    for (int j = 0; j < N_NODES; j++) {
        if (j == i) continue;
        insert_into_k_nearest(best_d, best_j, K, node_dist(sc, i, j), j);
    }
    for (int k = 0; k < K; k++) {
        if (best_j[k] >= 0) {
            sc->adj[i][best_j[k]] = true;
            sc->adj[best_j[k]][i] = true;
        }
    }
}

/*
 * pick_farthest_pair_as_endpoints — choose (src, goal) = graph's diameter.
 *
 *   All-pairs O(N²) scan; the pair with the largest Euclidean distance
 *   becomes (src, goal). Why FARTHEST: forces the search to traverse
 *   most of the graph before reaching goal, making the algorithm's
 *   BEHAVIOUR visible — a 2-hop search is too short to tell BFS from DFS.
 *
 *   Note: this is geometric diameter, not graph diameter (max shortest-path
 *   over all pairs). On random K-nearest graphs they usually agree.
 */
static void pick_farthest_pair_as_endpoints(Scene *sc)
{
    sc->src = 0; sc->goal = 1;
    float max_d = 0.f;
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i + 1; j < N_NODES; j++) {
            float d = node_dist(sc, i, j);
            if (d > max_d) { max_d = d; sc->src = i; sc->goal = j; }
        }
    }
}

/*
 * graph_generate — orchestrator.
 *
 *   Pseudocode:
 *     1. scatter_nodes_random                ← random placement
 *     2. clear adjacency matrix
 *     3. for each i: connect_to_k_nearest(i, K_CONNECT)
 *     4. pick_farthest_pair_as_endpoints     ← sets sc->src, sc->goal
 *
 *   The layout pass (§5) then settles random positions into a legible
 *   drawing; the search (§6) runs against the resulting graph.
 */
static void graph_generate(Scene *sc)
{
    scatter_nodes_random(sc);
    memset(sc->adj, 0, sizeof(sc->adj));
    for (int i = 0; i < N_NODES; i++) connect_to_k_nearest(sc, i, K_CONNECT);
    pick_farthest_pair_as_endpoints(sc);
}

/* ===================================================================== */
/* §5  force-directed layout (Fruchterman & Reingold 1991)                */
/* ===================================================================== */

/*
 * accumulate_repulsion_forces — Coulomb-like all-pairs push.
 *
 *   For every UNORDERED pair (i, j):
 *     d² := ‖pos_i − pos_j‖²            (clamped to ≥ 1 to avoid /0)
 *     F  := K_REP / d²                  (Coulomb-like, ∝ 1/d²)
 *     direction: from j toward i — push apart
 *     fx[i] += F · dx/d;  fy[i] += F · dy/d
 *     fx[j] −= F · dx/d;  fy[j] −= F · dy/d    (Newton's 3rd law)
 *
 *   O(N²) per call. The d² ≥ 1 clamp prevents NaNs when two nodes
 *   coincide; without it the force explodes and ejects one node.
 */
static void accumulate_repulsion_forces(const Scene *sc, float *fx, float *fy)
{
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i + 1; j < N_NODES; j++) {
            float dx = sc->pos[i].x - sc->pos[j].x;
            float dy = sc->pos[i].y - sc->pos[j].y;
            float d2 = dx*dx + dy*dy;
            if (d2 < 1.f) d2 = 1.f;
            float f = K_REP / d2;
            float d = sqrtf(d2);
            fx[i] += f * dx / d;  fy[i] += f * dy / d;
            fx[j] -= f * dx / d;  fy[j] -= f * dy / d;
        }
    }
}

/*
 * accumulate_spring_forces — Hooke-like pull along edges.
 *
 *   For every undirected edge (i, j) in adj:
 *     d := ‖pos_j − pos_i‖              (clamped to ≥ 1)
 *     F := K_ATT · (d − REST_LEN)        (positive when stretched,
 *                                         negative when compressed)
 *     direction: along (i → j) — pull together when stretched
 *
 *   Repulsion (everywhere) + attraction (along edges) has a low-energy
 *   equilibrium where connected nodes sit ≈ REST_LEN apart and
 *   unconnected ones drift apart. That equilibrium IS the legible
 *   drawing the search will animate against.
 */
static void accumulate_spring_forces(const Scene *sc, float *fx, float *fy)
{
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i + 1; j < N_NODES; j++) {
            if (!sc->adj[i][j]) continue;
            float dx = sc->pos[j].x - sc->pos[i].x;
            float dy = sc->pos[j].y - sc->pos[i].y;
            float d  = sqrtf(dx*dx + dy*dy);
            if (d < 1.f) d = 1.f;
            float f = K_ATT * (d - REST_LEN);
            fx[i] += f * dx / d;  fy[i] += f * dy / d;
            fx[j] -= f * dx / d;  fy[j] -= f * dy / d;
        }
    }
}

/*
 * integrate_and_clamp — Euler step + bounds clamp to play area.
 *
 *   pos[i] += F[i] · DT_SETTLE
 *   then clamp each axis into the play rect (HUD-aware top, MARGIN inset).
 *
 *   No damping: F&R 1991 omits it; the force balance reaches equilibrium
 *   on its own within SETTLE_ITERS ≈ 250 rounds. The bounds clamp keeps
 *   transient overshoot from ejecting nodes off-screen.
 */
static void integrate_and_clamp(Scene *sc, const float *fx, const float *fy)
{
    int pw = sc->cols * CELL_W, ph = sc->rows * CELL_H;
    int hud_py = HUD_ROWS * CELL_H;
    const int margin = 40;

    for (int i = 0; i < N_NODES; i++) {
        sc->pos[i].x += fx[i] * DT_SETTLE;
        sc->pos[i].y += fy[i] * DT_SETTLE;
        if (sc->pos[i].x < (float)margin)            sc->pos[i].x = (float)margin;
        if (sc->pos[i].x > (float)(pw - margin))     sc->pos[i].x = (float)(pw - margin);
        if (sc->pos[i].y < (float)(hud_py + margin)) sc->pos[i].y = (float)(hud_py + margin);
        if (sc->pos[i].y > (float)(ph - margin))     sc->pos[i].y = (float)(ph - margin);
    }
}

/*
 * layout_settle — Fruchterman & Reingold force-directed layout driver.
 *
 *   Pseudocode (SETTLE_ITERS iterations):
 *     zero force accumulators
 *     accumulate_repulsion_forces    ← all-pairs Coulomb push
 *     accumulate_spring_forces       ← edge Hooke pull
 *     integrate_and_clamp            ← Euler step, then bounds clamp
 *
 *   No formal convergence proof; in practice N≈40 settles visibly well
 *   under 250 iterations. Larger N would warrant COOLING (gradually
 *   shrink DT_SETTLE — see [6]) and Barnes-Hut O(N log N) repulsion
 *   (see [9]).
 *
 *   References [6] for the original algorithm, [7] for the spring-model
 *   predecessor, [8] for a survey of layout methods.
 */
static void layout_settle(Scene *sc)
{
    float fx[N_NODES], fy[N_NODES];
    for (int iter = 0; iter < SETTLE_ITERS; iter++) {
        memset(fx, 0, sizeof(fx));
        memset(fy, 0, sizeof(fy));
        accumulate_repulsion_forces(sc, fx, fy);
        accumulate_spring_forces   (sc, fx, fy);
        integrate_and_clamp        (sc, fx, fy);
    }
}

/* ===================================================================== */
/* §6  algorithms — BFS, DFS, A* on a shared expansion skeleton           */
/* ===================================================================== */

/* ── Container primitives ──────────────────────────────────────────── */

/*
 * NodeQueue / NodeStack ops — push/pop/empty on the FIFO and LIFO buffers.
 *
 *   Splitting the container API into named primitives makes bfs_step
 *   and dfs_step read as pseudocode against one unified skeleton:
 *   "pop u from container; for each neighbour, push v on container".
 *   The CONTAINER CHOICE is the algorithm (see References [3], [4]).
 */
static inline void queue_push (NodeQueue *q, int v) { q->buf[q->tail++] = v; }
static inline int  queue_pop  (NodeQueue *q)        { return q->buf[q->head++]; }
static inline bool queue_empty(const NodeQueue *q)  { return q->head >= q->tail; }

static inline void stack_push (NodeStack *s, int v) { s->buf[s->top++] = v; }
static inline int  stack_pop  (NodeStack *s)        { return s->buf[--s->top]; }
static inline bool stack_empty(const NodeStack *s)  { return s->top == 0; }

/* ── Display helper ────────────────────────────────────────────────── */

static const char *alg_name(const Scene *sc)
{
    return sc->alg == ALG_BFS ? "BFS" : sc->alg == ALG_DFS ? "DFS" : "A*";
}

/* ── Search bookkeeping shared across BFS / DFS / A* ───────────────── */

/*
 * mark_visited_if_neutral — colour-state transition for an expanded node.
 *
 *   Preserves SRC and GOAL (they have their own colours); any other
 *   node transitions to VISITED on expansion. Used by all three step
 *   functions to avoid clobbering the endpoint glyphs.
 */
static inline void mark_visited_if_neutral(Scene *sc, int u)
{
    if (sc->state[u] != SRC && sc->state[u] != GOAL) sc->state[u] = VISITED;
}

/*
 * search_reset — initialise per-node state and seed the active container.
 *
 *   Pseudocode:
 *     for each node i:
 *       state[i] := SRC if i==src, GOAL if i==goal, else UNVIS
 *       prev[i]  := −1
 *       cost[i]  := +inf       (A* g-score; unused by BFS/DFS)
 *       on_path[i] := false
 *     cost[src] := 0
 *     clear queue and stack
 *     phase := RUNNING
 *     state[src] := FRONTIER
 *     seed the algorithm's container with src (A* uses no container)
 */
static void search_reset(Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        sc->state[i]   = (i == sc->src) ? SRC : (i == sc->goal) ? GOAL : UNVIS;
        sc->prev[i]    = -1;
        sc->cost[i]    = 1e30f;
        sc->on_path[i] = false;
    }
    sc->cost[sc->src] = 0.f;
    sc->queue.head = sc->queue.tail = 0;
    sc->stack.top  = 0;
    sc->steps      = 0;
    sc->phase      = RUNNING;

    sc->state[sc->src] = FRONTIER;
    if      (sc->alg == ALG_BFS) queue_push(&sc->queue, sc->src);
    else if (sc->alg == ALG_DFS) stack_push(&sc->stack, sc->src);
    /* A*: no container — frontier_pick_min_f scans state[] each tick */
}

/*
 * reconstruct_path — back-walk prev[] from goal to source, mark on_path[].
 *
 *   Pseudocode:
 *     n := goal
 *     while n != −1:
 *       on_path[n] := true
 *       n := prev[n]
 *     for each marked node: bump state[] to PATH_NODE (SRC/GOAL preserved)
 *     phase := DONE
 *
 *   prev[i] is set when i transitions UNVIS → FRONTIER (or for A*, when
 *   relax_edge finds a cheaper path through some u). Walking it backward
 *   from goal yields the path in reverse — we just MARK, order doesn't
 *   matter to the renderer.
 */
static void reconstruct_path(Scene *sc)
{
    int n = sc->goal;
    while (n != -1) { sc->on_path[n] = true; n = sc->prev[n]; }
    for (int i = 0; i < N_NODES; i++) {
        if (!sc->on_path[i]) continue;
        if      (i == sc->src)  sc->state[i] = SRC;
        else if (i == sc->goal) sc->state[i] = GOAL;
        else                    sc->state[i] = PATH_NODE;
    }
    sc->phase = DONE;
}

/* ── A* helpers ────────────────────────────────────────────────────── */

/*
 * heuristic_to_goal — h(n) = Euclidean distance from node n to the goal.
 *
 *   ADMISSIBLE: never overestimates the true cost (no path can be
 *   shorter than the straight line). With an admissible heuristic, A*
 *   is OPTIMAL — the first path returned is the cheapest by edge-weight
 *   sum. References [1] proves it; [4] gives a more accessible treatment.
 */
static float heuristic_to_goal(const Scene *sc, int n)
{
    float dx = sc->pos[n].x - sc->pos[sc->goal].x;
    float dy = sc->pos[n].y - sc->pos[sc->goal].y;
    return sqrtf(dx*dx + dy*dy);
}

/*
 * frontier_pick_min_f — scan state[] for the FRONTIER node minimising
 * f = g + h. Returns −1 when the frontier is empty (search exhausted).
 *
 *   O(N) per call — fine at N_NODES=40. A binary heap would give
 *   O(log N) at the cost of an explicit priority queue.
 */
static int frontier_pick_min_f(const Scene *sc)
{
    int   best   = -1;
    float best_f = 1e30f;
    for (int i = 0; i < N_NODES; i++) {
        if (sc->state[i] != FRONTIER) continue;
        float f = sc->cost[i] + heuristic_to_goal(sc, i);
        if (f < best_f) { best_f = f; best = i; }
    }
    return best;
}

/*
 * relax_edge — A* edge relaxation.
 *
 *   if cost[u] + len(u,v) < cost[v]:
 *     cost[v] := cost[u] + len(u,v)
 *     prev[v] := u
 *     state[v] := FRONTIER             (re-add; the next scan picks it up)
 *
 *   Skips v if it's already VISITED (closed-set assumption — admissible
 *   heuristic means the first pop of v is its optimal cost).
 *
 *   Returns true if v == goal so the caller can call reconstruct_path
 *   without descending into the state[v] := FRONTIER assignment.
 */
static bool relax_edge(Scene *sc, int u, int v)
{
    if (sc->state[v] == VISITED) return false;
    float ng = sc->cost[u] + node_dist(sc, u, v);
    if (ng >= sc->cost[v]) return false;
    sc->cost[v] = ng;
    sc->prev[v] = u;
    if (v == sc->goal) return true;
    sc->state[v] = FRONTIER;
    return false;
}

/* ── Step functions — one expansion per call ──────────────────────── */

/*
 * bfs_step — one BFS expansion: queue.pop_front, mark visited, enqueue
 * fresh neighbours.
 *
 *   Pseudocode (matches T1 unified skeleton):
 *     if queue empty: phase := DONE; return
 *     u := queue.pop_front()
 *     mark u VISITED (if neutral)
 *     for each neighbour v of u:
 *       if v ∉ {UNVIS, GOAL}: skip
 *       prev[v] := u
 *       if v == goal: reconstruct_path(); return
 *       state[v] := FRONTIER
 *       queue.push(v)
 *
 *   GUARANTEE: expansion in concentric rings by hop count — the first
 *   path BFS finds is the one with the FEWEST hops from src to goal.
 */
static void bfs_step(Scene *sc)
{
    if (queue_empty(&sc->queue)) { sc->phase = DONE; return; }
    int u = queue_pop(&sc->queue);
    mark_visited_if_neutral(sc, u);
    sc->steps++;

    for (int v = 0; v < N_NODES; v++) {
        if (!sc->adj[u][v]) continue;
        if (sc->state[v] != UNVIS && sc->state[v] != GOAL) continue;
        sc->prev[v] = u;
        if (v == sc->goal) { reconstruct_path(sc); return; }
        sc->state[v] = FRONTIER;
        queue_push(&sc->queue, v);
    }
}

/*
 * dfs_step — one DFS expansion: stack.pop_top, mark visited, push fresh
 * neighbours.
 *
 *   Pseudocode:
 *     if stack empty: phase := DONE; return
 *     u := stack.pop_top()
 *     if u already VISITED: return            ← lazy-pop dedup
 *     mark u VISITED (if neutral)
 *     for each neighbour v of u:
 *       if v already VISITED: skip
 *       if prev[v] == −1: prev[v] := u        ← keep FIRST discoverer
 *       if v == goal: reconstruct_path(); return
 *       if v is UNVIS: state[v] := FRONTIER; stack.push(v)
 *
 *   The lazy-pop dedup (the VISITED check after pop) lets the push loop
 *   stay branchless — duplicates surface at pop time and are dropped.
 *
 *   NO OPTIMALITY GUARANTEE: DFS finds a path, not necessarily the
 *   shortest. Useful as a contrast against BFS on the same graph.
 */
static void dfs_step(Scene *sc)
{
    if (stack_empty(&sc->stack)) { sc->phase = DONE; return; }
    int u = stack_pop(&sc->stack);
    if (sc->state[u] == VISITED) return;
    mark_visited_if_neutral(sc, u);
    sc->steps++;

    for (int v = 0; v < N_NODES; v++) {
        if (!sc->adj[u][v]) continue;
        if (sc->state[v] == VISITED) continue;
        if (sc->prev[v] == -1) sc->prev[v] = u;
        if (v == sc->goal) { reconstruct_path(sc); return; }
        if (sc->state[v] == UNVIS) {
            sc->state[v] = FRONTIER;
            stack_push(&sc->stack, v);
        }
    }
}

/*
 * astar_step — one A* expansion: pick FRONTIER node with min f, relax edges.
 *
 *   Pseudocode:
 *     u := frontier_pick_min_f()
 *     if u == −1:    phase := DONE; return              ← frontier exhausted
 *     if u == goal:  reconstruct_path(); return         ← popped goal: optimal
 *     mark u VISITED (if neutral)
 *     for each neighbour v of u:
 *       if relax_edge(u, v): reconstruct_path(); return ← reached goal
 *
 *   OPTIMALITY: with admissible heuristic (Euclidean distance), the
 *   first time the goal is POPPED its cost equals the true shortest-path
 *   cost. With h ≡ 0, A* degenerates to Dijkstra; with h = g = 0 and
 *   unit weights it degenerates to BFS.
 */
static void astar_step(Scene *sc)
{
    int u = frontier_pick_min_f(sc);
    if (u == -1)       { sc->phase = DONE; return; }
    if (u == sc->goal) { reconstruct_path(sc); return; }
    mark_visited_if_neutral(sc, u);
    sc->steps++;

    for (int v = 0; v < N_NODES; v++) {
        if (!sc->adj[u][v]) continue;
        if (relax_edge(sc, u, v)) { reconstruct_path(sc); return; }
    }
}

/*
 * search_step — dispatch one tick to the currently selected algorithm.
 *   The ONLY function that needs to know which algorithm is active.
 */
static void search_step(Scene *sc)
{
    if (sc->phase != RUNNING) return;
    switch (sc->alg) {
    case ALG_BFS:   bfs_step  (sc); break;
    case ALG_DFS:   dfs_step  (sc); break;
    case ALG_ASTAR: astar_step(sc); break;
    }
}

/* ===================================================================== */
/* §7  scene — rendering pipeline                                         */
/* ===================================================================== */

/*
 * NodeGlyph — visual representation of one node state.
 *   Decouples "what does this state look like" from the drawing loop.
 *   The legend in the HUD (S=src, G=goal, O=frontier, *=path) maps
 *   directly to entries in node_glyph_for below.
 */
typedef struct {
    int    cp;       /* colour pair                       */
    chtype ch;       /* glyph character                   */
    attr_t extra;    /* extra attributes (e.g. A_BOLD)    */
} NodeGlyph;

/*
 * node_glyph_for — translate a NodeState to its visual representation.
 *   Read this as the LEGEND for the demo. Adding a new NodeState only
 *   requires a new case here — the drawing loop stays unchanged.
 */
static NodeGlyph node_glyph_for(NodeState s)
{
    switch (s) {
    case UNVIS:     return (NodeGlyph){CP_UNVIS, 'o', 0};
    case FRONTIER:  return (NodeGlyph){CP_FRONT, 'O', A_BOLD};
    case VISITED:   return (NodeGlyph){CP_VIS,   'o', 0};
    case PATH_NODE: return (NodeGlyph){CP_PATH,  '*', A_BOLD};
    case SRC:       return (NodeGlyph){CP_SRC,   'S', A_BOLD};
    case GOAL:      return (NodeGlyph){CP_GOAL,  'G', A_BOLD};
    default:        return (NodeGlyph){CP_UNVIS, 'o', 0};
    }
}

/*
 * draw_edge_segment — Bresenham line between two CELL points with
 * directional ASCII glyph selection.
 *
 *   At each plot, the major-step axis decides the glyph:
 *     horizontal step only → '-'
 *     vertical step only   → '|'
 *     both axes step       → '\' (same sign) or '/' (opposite)
 *
 *   Renders the edge as a continuous ASCII line that visibly slopes in
 *   the right direction — much more readable than uniform '-' fill.
 *
 *   Reference: Bresenham (1965), "Algorithm for computer control of a
 *   digital plotter", IBM Systems Journal 4(1). The directional-glyph
 *   trick is common in ASCII graph libraries (e.g. graph-easy).
 */
static void draw_edge_segment(const Scene *sc,
                              int x0, int y0, int x1, int y1, attr_t attr)
{
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    for (;;) {
        if (x0 >= 0 && x0 < sc->cols && y0 >= 0 && y0 < sc->rows) {
            int  e2 = 2 * err;
            bool bx = e2 > -dy, by = e2 < dx;
            chtype ch = (bx && by) ? (sx == sy ? '\\' : '/') : bx ? '-' : '|';
            attron(attr); mvaddch(y0, x0, ch); attroff(attr);
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x0 += sx; }
        if (e2 <  dx) { err += dx; y0 += sy; }
    }
}

/*
 * draw_graph_edges — render every adjacency as an ASCII line segment.
 *
 *   Edges where BOTH endpoints are on the final path are drawn in bold
 *   yellow (CP_PATH_EDGE); others are dim grey (CP_EDGE). This makes
 *   the solution stand out against the underlying graph.
 */
static void draw_graph_edges(const Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        for (int j = i + 1; j < N_NODES; j++) {
            if (!sc->adj[i][j]) continue;
            bool path_e = sc->on_path[i] && sc->on_path[j];
            int  cp     = path_e ? CP_PATH_EDGE : CP_EDGE;
            attr_t attr = COLOR_PAIR(cp) | (path_e ? (attr_t)A_BOLD : (attr_t)0);
            draw_edge_segment(sc,
                              px_cx(sc->pos[i].x), px_cy(sc->pos[i].y),
                              px_cx(sc->pos[j].x), px_cy(sc->pos[j].y),
                              attr);
        }
    }
}

/*
 * draw_graph_nodes — render every node's glyph at its cell position.
 *
 *   The visual state (glyph + colour + bold) comes from node_glyph_for,
 *   so this function is pure layout: pixel → cell, bounds check, plot.
 */
static void draw_graph_nodes(const Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        int cx = px_cx(sc->pos[i].x), cy = px_cy(sc->pos[i].y);
        if (cx < 0 || cx >= sc->cols || cy < 0 || cy >= sc->rows) continue;
        NodeGlyph g = node_glyph_for(sc->state[i]);
        attron(COLOR_PAIR(g.cp) | g.extra);
        mvaddch(cy, cx, g.ch);
        attroff(COLOR_PAIR(g.cp) | g.extra);
    }
}

/*
 * draw_hud — top two status lines: controls + algorithm progress.
 *   Row 0: title + key hints.
 *   Row 1: algorithm + phase + step count + legend + pause indicator.
 */
static void draw_hud(const Scene *sc)
{
    const char *phase_s =
        sc->phase == IDLE    ? "IDLE"    :
        sc->phase == RUNNING ? "RUNNING" : "DONE";
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(0, 0,
        " GraphSearch  q:quit  s:start  a:alg  r:new-graph  p:pause  spc:step");
    mvprintw(1, 0,
        " alg:%-4s  phase:%-7s  steps:%3d  nodes:%d  S=src G=goal O=frontier *=path  %s",
        alg_name(sc), phase_s, sc->steps, N_NODES,
        sc->paused ? "PAUSED" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

/*
 * scene_draw — render one frame.
 *
 *   Pseudocode (layered draw, painter's algorithm):
 *     1. graph edges  ─ background lattice; path edges bright
 *     2. graph nodes  ─ glyph + colour from NodeState
 *     3. HUD          ─ top two lines: keys + progress
 *
 *   Edges first so nodes paint over their endpoints; HUD last so its
 *   glyphs sit on top of any node intruding into the top rows.
 */
static void scene_draw(const Scene *sc)
{
    draw_graph_edges(sc);
    draw_graph_nodes(sc);
    draw_hud        (sc);
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

/*
 * scene_reset_visuals — wipe state[] and on_path[] back to the IDLE baseline.
 *
 *   Keeps the topology (pos, adj, src, goal). Called when the user
 *   switches algorithm or generates a new graph: drop any in-progress
 *   or finished search visuals without rebuilding the graph.
 */
static void scene_reset_visuals(Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        sc->state[i]   = (i == sc->src) ? SRC : (i == sc->goal) ? GOAL : UNVIS;
        sc->on_path[i] = false;
    }
}

/*
 * new_graph — orchestrator: generate topology, lay it out, reset visuals.
 *
 *   Pseudocode:
 *     1. graph_generate(sc)         ← scatter + connect + endpoints
 *     2. layout_settle(sc)          ← Fruchterman & Reingold force-directed
 *     3. scene_reset_visuals(sc)    ← state[] / on_path[] back to baseline
 *     4. phase := IDLE; paused := false
 *
 *   Bound to 'r' and also called on initial start + resize.
 */
static void new_graph(Scene *sc)
{
    graph_generate(sc);
    layout_settle(sc);
    scene_reset_visuals(sc);
    sc->phase  = IDLE;
    sc->paused = false;
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

    Scene scene = {0};
    scene.alg = ALG_BFS;
    getmaxyx(stdscr, scene.rows, scene.cols);
    new_graph(&scene);

    long long last_step  = clock_ns();
    long long last_frame = clock_ns();

    while (!g_quit) {

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, scene.rows, scene.cols);
            new_graph(&scene);
            last_step = last_frame = clock_ns();
            continue;
        }

        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27: g_quit = 1;                       break;
        case 's': case 'S':          search_reset(&scene);             break;
        case 'a': case 'A':
            scene.alg   = (Algorithm)((scene.alg + 1) % 3);
            scene.phase = IDLE;
            scene_reset_visuals(&scene);
            break;
        case 'r': case 'R':          new_graph(&scene);                break;
        case 'p': case 'P':          scene.paused = !scene.paused;     break;
        case ' ':
            if (scene.phase == IDLE) search_reset(&scene);
            else                     search_step (&scene);
            break;
        default: break;
        }

        long long now = clock_ns();

        /* advance one step every STEP_NS if running and not paused */
        if (scene.phase == RUNNING && !scene.paused && now - last_step >= STEP_NS) {
            search_step(&scene);
            last_step = now;
        }

        /* render at 30 fps */
        if (now - last_frame >= RENDER_NS) {
            last_frame = now;
            erase();
            scene_draw(&scene);
            wnoutrefresh(stdscr);
            doupdate();
        }

        clock_sleep_ns(10000000LL);   /* ~10 ms poll interval */
    }

    return 0;
}
