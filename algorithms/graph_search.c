/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * graph_search.c — watch BFS, DFS, and A* find a path through a random graph.
 *
 * A graph of 40 nodes is laid out with a spring/repulsion simulation
 * (Fruchterman & Reingold 1991) so it's legible, then one of three searches
 * crawls from source to goal one step at a time. The three differ only in
 * which waiting node they expand next: BFS the oldest, DFS the newest, A* the
 * one that looks closest to the goal.
 *
 * Search refs: Hart/Nilsson/Raphael 1968 (A*); Dijkstra 1959; CLRS ch.22/24.
 * Layout ref: Fruchterman & Reingold 1991. Line drawing: Bresenham 1965.
 * Build: gcc -std=c11 -O2 -Wall -Wextra algorithms/graph_search.c \
 *            -o graph_search -lncurses -lm
 */

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

/* ── §1 config — graph size, layout-force constants, timing ── */

#define N_NODES        40
#define K_CONNECT       3    /* each node connects to K nearest neighbours     */
#define SETTLE_ITERS  250    /* force-directed layout iterations at startup    */
#define K_REP       6000.f  /* repulsion strength                              */
#define K_ATT          0.4f /* spring attraction along edges                   */
#define REST_LEN      90.f  /* target edge length in pixel space               */
#define DT_SETTLE      0.3f /* how far nodes move per layout step              */
#define CELL_W          8   /* sub-pixels per terminal column                 */
#define CELL_H         16   /* sub-pixels per terminal row                    */
#define HUD_ROWS        3   /* top rows reserved for the status bar            */
#define STEP_NS     (1000000000LL / 8)   /* one search step every ~125 ms     */
#define RENDER_NS   (1000000000LL / 30)  /* redraw at ~30 fps                  */

/* What each node is currently doing in the search. Drives its glyph + colour:
 *   UNVIS     not seen yet              FRONTIER  seen, waiting to be expanded
 *   VISITED   expanded, done with it    PATH_NODE on the final answer path
 *   SRC/GOAL  the two endpoints (kept distinct so their glyphs survive). */
typedef enum { UNVIS, FRONTIER, VISITED, PATH_NODE, SRC, GOAL } NodeState;

/* Which of the three searches is running. */
typedef enum { ALG_BFS, ALG_DFS, ALG_ASTAR } Algorithm;

/* Lifecycle of a run: not started / crawling / found-or-exhausted. */
typedef enum { IDLE, RUNNING, DONE } SearchPhase;

/* ncurses colour-pair IDs (start at 1; pair 0 is reserved). */
enum {
    CP_UNVIS = 1, CP_FRONT, CP_VIS, CP_PATH, CP_SRC, CP_GOAL,
    CP_EDGE, CP_PATH_EDGE, CP_HUD,
};

/* ── §2 clock — monotonic nanosecond timer + sleep ── */

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

/* ── §3 color — one colour per node state, with an 8-colour fallback ── */

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

/* ── §4 graph — node positions, adjacency, and the all-in-one Scene struct ── */

/* A point on the screen, measured in sub-pixels (CELL_W/CELL_H per character).
 * y grows downward, the usual terminal convention. Doubles as a force vector
 * during layout, since a force is just a little position nudge. */
typedef struct { float x, y; } Vec2;

/* BFS's waiting line: first node in is the first node out.
 *   buf[head..tail)  node indices, oldest at head
 *   head             index of the next node to pop
 *   tail             index where the next pushed node goes (one past the end)
 * Empty when head reaches tail. Over-sized to 4x N_NODES so it never overflows. */
typedef struct {
    int buf[N_NODES * 4];
    int head, tail;
} NodeQueue;

/* DFS's pile: last node in is the first node out.
 *   buf[0..top)  node indices, newest just below top
 *   top          index where the next pushed node goes; 0 means empty
 * Over-sized to 4x N_NODES so DFS can push the same node more than once
 * without checking for duplicates first — stale copies are simply skipped
 * when popped (see the VISITED guard in dfs_step). */
typedef struct {
    int buf[N_NODES * 4];
    int top;
} NodeStack;

/* Everything about one run lives here — graph shape, search progress, and
 * mode flags — so a single pointer carries the whole world between functions.
 * One Scene sits in main(); the only state kept outside it is the two signal
 * flags in §8, which signal handlers can't reach through a pointer. */
typedef struct {
    /* The graph: where the nodes are and which ones connect. */
    Vec2        pos[N_NODES];              /* node positions, in sub-pixels   */
    bool        adj[N_NODES][N_NODES];     /* adj[i][j] true = edge i—j       */
    int         src, goal;                 /* the two endpoints to connect    */
    int         rows, cols;                /* terminal size, in characters    */

    /* The search's notes on each node. */
    NodeState   state[N_NODES];            /* what each node is doing now      */
    int         prev[N_NODES];             /* who discovered this node (-1=none),
                                              followed backward to trace path  */
    float       cost[N_NODES];             /* shortest distance from src so far;
                                              A* only (the "g" score)          */
    bool        on_path[N_NODES];          /* true once node is on the answer  */

    /* Where waiting nodes live (A* needs neither — it scans state[]). */
    NodeQueue   queue;                     /* BFS frontier                     */
    NodeStack   stack;                     /* DFS frontier                     */

    /* Run mode + HUD readouts. */
    Algorithm   alg;                       /* which search is selected         */
    SearchPhase phase;                     /* idle / running / done            */
    int         steps;                     /* nodes expanded so far            */
    bool        paused;                    /* user froze the animation         */
} Scene;

/* Round a sub-pixel position down to the character cell it falls in. The +0.5f
 * snaps to the nearest cell rather than always rounding toward zero. */
static int px_cx(float px) { return (int)(px / (float)CELL_W + 0.5f); }
static int px_cy(float py) { return (int)(py / (float)CELL_H + 0.5f); }

/* Straight-line distance between two nodes. Serves double duty: A* uses it as
 * the real edge cost, and the layout uses it to measure spring stretch. */
static float node_dist(const Scene *sc, int i, int j)
{
    float dx = sc->pos[i].x - sc->pos[j].x;
    float dy = sc->pos[i].y - sc->pos[j].y;
    return sqrtf(dx*dx + dy*dy);
}

/* Drop every node at a random spot. Keeps clear of the HUD strip up top and
 * leaves a one-eighth border all around, so the layout step has room to push
 * nodes outward before they bump the edges. */
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

/* Slot one candidate into a running "K closest so far" leaderboard, kept sorted
 * nearest-first. If the new distance beats a slot, it shifts the losers down and
 * takes that slot. Cheaper than sorting all candidates when we only want a few. */
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

/* Wire node i to its K nearest neighbours (both directions, so the edge is
 * two-way). A small K keeps the graph an open, readable web; a larger K makes a
 * dense tangle that's harder to follow but easier for the search to cross. */
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

/* Pick the two nodes that sit farthest apart on screen as source and goal.
 * Far-apart endpoints force the search to cross most of the graph, so you can
 * actually see how BFS, DFS, and A* differ — a two-hop hunt would end too fast.
 * (This is the farthest pair by screen distance, not by hop count, but on these
 * random graphs the two usually line up.) */
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

/* Build a fresh random graph: scatter the nodes, wipe old edges, connect each
 * node to its nearest few, and pick the two endpoints. The layout pass (§5)
 * tidies the positions afterward; the search (§6) runs on the result. */
static void graph_generate(Scene *sc)
{
    scatter_nodes_random(sc);
    memset(sc->adj, 0, sizeof(sc->adj));
    for (int i = 0; i < N_NODES; i++) connect_to_k_nearest(sc, i, K_CONNECT);
    pick_farthest_pair_as_endpoints(sc);
}

/* ── §5 layout — settle the random scatter into a readable shape ── *
 *
 * Pretend every node is a magnet that pushes all the others away, and every
 * edge is a spring pulling its two nodes together. Let the whole thing relax
 * for a while and it untangles itself: connected nodes end up a comfortable
 * distance apart, the rest spread out. That settled picture is what we draw.
 * Based on Fruchterman & Reingold (1991). */

/* Every node pushes every other away, harder the closer they are. If two nodes
 * land on the exact same spot the push would blow up to infinity, so we floor
 * the distance at 1. The push on i and j is equal and opposite. */
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

/* Each edge acts like a spring at rest length REST_LEN: pull the two nodes
 * together when stretched past it, push them apart when squeezed below it.
 * Springs (on edges) plus the all-over repulsion above settle into the layout. */
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

/* Nudge each node a little in the direction of its accumulated force, then pin
 * it back inside the visible area (below the HUD, away from the edges) so a
 * sudden big shove can't fling it off-screen. */
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

/* Run the relaxation: each round, total up the pushes and pulls, then move
 * every node one small step. A few hundred rounds is plenty to untangle 40
 * nodes. This all happens at startup, before anything is drawn. */
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

/* ── §6 algorithms — BFS, DFS, A*, all the same loop with a different pick rule ── *
 *
 * All three do the same dance: take one node out of the "waiting" pile, mark it
 * done, and add its undiscovered neighbours to the pile. The only difference is
 * who gets picked next — and that single choice is the whole algorithm:
 *   BFS picks the node that has waited longest (a queue) -> fewest-hops path.
 *   DFS picks the node added most recently  (a stack) -> some path, maybe long.
 *   A*  picks the node that looks closest to the goal -> shortest path by distance. */

/* Queue (BFS) and stack (DFS) operations. Same three moves on each — add, take
 * one out, is-it-empty — so the search loops read like plain English. */
static inline void queue_push (NodeQueue *q, int v) { q->buf[q->tail++] = v; }
static inline int  queue_pop  (NodeQueue *q)        { return q->buf[q->head++]; }
static inline bool queue_empty(const NodeQueue *q)  { return q->head >= q->tail; }

static inline void stack_push (NodeStack *s, int v) { s->buf[s->top++] = v; }
static inline int  stack_pop  (NodeStack *s)        { return s->buf[--s->top]; }
static inline bool stack_empty(const NodeStack *s)  { return s->top == 0; }

/* Short label for the HUD. */
static const char *alg_name(const Scene *sc)
{
    return sc->alg == ALG_BFS ? "BFS" : sc->alg == ALG_DFS ? "DFS" : "A*";
}

/* ── bookkeeping shared by all three searches ── */

/* Mark a just-expanded node as done — but leave the source and goal alone, so
 * their special S/G glyphs aren't overwritten with a plain "visited" dot. */
static inline void mark_visited_if_neutral(Scene *sc, int u)
{
    if (sc->state[u] != SRC && sc->state[u] != GOAL) sc->state[u] = VISITED;
}

/* Wipe the slate for a fresh search: nobody visited, no path known, every cost
 * "infinite" except the source (cost 0). Then mark the source as waiting and
 * drop it into BFS's queue or DFS's stack. A* needs no container — it just
 * scans for waiting nodes each tick. Bound to 's'. */
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

/* The search is over once we reach the goal. Every node remembers who
 * discovered it (prev[]), so we start at the goal and follow that chain of
 * discoverers back to the source, flagging each node as on the answer path.
 * Those nodes then light up, and the run is marked DONE. */
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

/* ── A* helpers ── */

/* A* needs a guess of how much farther a node is from the goal. We use the
 * straight-line distance — which can never overstate the real remaining
 * distance (no route is shorter than a straight line). That "never overstates"
 * property is exactly what makes A* find the truly shortest path. */
static float heuristic_to_goal(const Scene *sc, int n)
{
    float dx = sc->pos[n].x - sc->pos[sc->goal].x;
    float dy = sc->pos[n].y - sc->pos[sc->goal].y;
    return sqrtf(dx*dx + dy*dy);
}

/* A*'s pick rule: of all the waiting nodes, choose the most promising — the one
 * whose distance-so-far plus straight-line-guess-to-goal is smallest. Returns
 * -1 if nothing is waiting, meaning the search ran out of options. Just a linear
 * scan; a heap would be faster but overkill for 40 nodes. */
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

/* A*'s update step: if going through u reaches v more cheaply than any route
 * found so far, record the shorter cost, remember u as v's discoverer, and put
 * v back on the waiting list. Already-finished nodes are left alone. Returns
 * true the moment v is the goal, so the caller can stop and trace the path. */
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

/* ── step functions — each call advances the search one node ── */

/* One BFS step. Take the oldest waiting node, mark it done, and add each of its
 * undiscovered neighbours to the back of the queue. Because the queue serves
 * oldest-first, BFS spreads outward in rings like ripples on a pond, so the
 * first time it touches the goal it has used the fewest possible hops. */
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

/* One DFS step. Take the newest waiting node instead of the oldest, which sends
 * the search plunging deep down one branch before it backs up. It still finds
 * a path, but rarely the shortest — the fun contrast to BFS on the same graph.
 * A node can land on the stack more than once; the "already done?" check right
 * after popping quietly drops the stale copies. */
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

/* One A* step. Pick the most promising waiting node; if it's the goal we're
 * done with the shortest route. Otherwise mark it done and update the cost to
 * reach each neighbour. Because the guess never overstates the remaining
 * distance, the first time the goal comes up its recorded cost is the true
 * shortest — biasing toward the goal lets it skip the wasted wandering BFS does. */
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

/* Advance whichever search the user picked by one step. */
static void search_step(Scene *sc)
{
    if (sc->phase != RUNNING) return;
    switch (sc->alg) {
    case ALG_BFS:   bfs_step  (sc); break;
    case ALG_DFS:   dfs_step  (sc); break;
    case ALG_ASTAR: astar_step(sc); break;
    }
}

/* ── §7 scene — draw the graph and the HUD each frame ── */

/* How one node should look on screen. Keeping the look-up separate from the
 * drawing loop means a new node state only needs a new entry, not new code. */
typedef struct {
    int    cp;       /* which colour pair to use          */
    chtype ch;       /* the character to print            */
    attr_t extra;    /* extra flair, e.g. A_BOLD          */
} NodeGlyph;

/* The legend in one place: turn a node's state into its glyph and colour. */
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

/* Draw a straight line of characters between two cells, one cell at a time
 * (Bresenham's 1965 line algorithm). The glyph at each cell hints at the line's
 * slope there: '-' if it's stepping sideways, '|' if vertically, '/' or '\' on
 * a diagonal — so the edge actually looks like it leans the right way. */
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

/* Draw every edge. Edges with both ends on the final path glow bold yellow so
 * the answer pops out against the dim grey of the rest of the graph. */
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

/* Stamp each node's glyph at its cell, skipping any that fall off-screen. */
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

/* The two status lines up top: key hints on row 0, live progress on row 1. */
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

/* Paint one frame back-to-front: edges first, then nodes on top of them, then
 * the HUD on top of everything (so a stray node can't hide the status bar). */
static void scene_draw(const Scene *sc)
{
    draw_graph_edges(sc);
    draw_graph_nodes(sc);
    draw_hud        (sc);
}

/* ── §8 app — setup, signal handling, input, and the main loop ── */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s==SIGINT||s==SIGTERM) g_quit=1;
    if (s==SIGWINCH)           g_resize=1;
}

static void cleanup(void) { endwin(); }

/* Clear just the search colouring back to the starting look, keeping the same
 * graph. Used when switching algorithm or starting a new graph, so an old or
 * half-finished search doesn't linger on screen. */
static void scene_reset_visuals(Scene *sc)
{
    for (int i = 0; i < N_NODES; i++) {
        sc->state[i]   = (i == sc->src) ? SRC : (i == sc->goal) ? GOAL : UNVIS;
        sc->on_path[i] = false;
    }
}

/* Make a whole new graph from scratch: build it, lay it out, and clear any
 * search state. Runs at startup, on 'r', and after a resize. */
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
