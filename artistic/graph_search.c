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
        init_pair(CP_VIS,      238, -1);   /* dark grey          */
        init_pair(CP_PATH,     226, -1);   /* yellow             */
        init_pair(CP_SRC,       46, -1);   /* green              */
        init_pair(CP_GOAL,     196, -1);   /* red                */
        init_pair(CP_EDGE,     238, -1);   /* dim grey           */
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
