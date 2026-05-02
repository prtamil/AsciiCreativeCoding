/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maze_backtracker.c — Recursive-backtracker DFS maze, animated.
 *
 * DEMO: Begins with a uniform grid of walled cells. A glowing white "dig
 *       head" walks a depth-first random path, carving walls as it goes
 *       and trailing magenta along visited cells. When stuck, it backtracks
 *       (visibly retracing the trail). After every cell is visited, two
 *       BFS passes find the maze's longest path (its tree diameter) and
 *       a gold beam streams along that path from one end to the other.
 *       Hold, supernova-flash reset, repeat forever.
 *
 * Study alongside: ../../misc/maze.c — the same algorithm in plainer ASCII;
 *       this file adds smooth Unicode wall corners, glow effects, the
 *       diameter solver, and an auto-loop state machine.
 *
 * Section map:
 *   §1 config   — grid size, dig pace, glow rates, hold/dissolve
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + wall / trail / head / solution
 *   §5 maze     — Cell, Maze, dig-step, BFS, diameter
 *   §6 scene    — DIG / SOLVE / HOLD state machine, glow decay
 *   §7 screen   — 16-entry wall-corner LUT, render
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate restart)
 *   + / =      more dig-steps per tick (faster)
 *   -          fewer dig-steps per tick (slower)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra maze_backtracker.c -o maze_backtracker \
 *       -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Recursive-backtracker (a.k.a. depth-first-search) maze
 *                  generation. Start at any cell, push it on a stack, mark
 *                  it visited. While the stack is non-empty: peek the top
 *                  cell, look at its unvisited neighbours; if any, pick one
 *                  at random, carve the wall between, push the neighbour;
 *                  otherwise pop (backtrack). When the stack empties, every
 *                  cell has been visited and the resulting graph is a
 *                  uniform spanning tree of the grid — a "perfect maze"
 *                  with exactly one path between any two cells.
 *
 *                  Solution phase: the longest path in a tree (its
 *                  "diameter") is found by two BFS passes — pick any
 *                  vertex A, BFS to find the farthest vertex X, then BFS
 *                  from X to find its farthest vertex Y. The path X→Y is
 *                  the diameter and a maximally interesting maze solution.
 *
 * Data-structure : Per cell: 4-bit wall bitmask (N=1, E=2, S=4, W=8;
 *                  set = wall present), visited flag, two glow floats.
 *                  Carving the wall between A and B means clearing the
 *                  d-bit in A AND the opposite-d bit in B — walls are
 *                  doubly stored so each cell knows its own walls without
 *                  consulting neighbours during render.
 *
 *                  DFS stack is a plain int array sized to total_cells
 *                  (each cell can be on the stack at most once at any
 *                  moment). BFS uses a queue + parent[] of the same size.
 *
 * Rendering      : Maze cells render as 1 char of "interior" plus 1 char
 *                  of wall per side, so a W×H maze occupies (2W+1)×(2H+1)
 *                  terminal cells. Wall corners use a 16-entry Unicode
 *                  box-drawing lookup keyed on which of the 4 surrounding
 *                  wall segments exist (NESW bits). Glows: dig_glow
 *                  (magenta trail), head_glow (white-bold cursor),
 *                  solution_glow (gold beam) — all decay exponentially.
 *
 * Performance    : DIG: O(N) total over the whole run (each cell visited
 *                  once + at most one backtrack pass). SOLVE: 2× O(N) BFS.
 *                  We throttle to dig_steps_per_tick (default 8) so the
 *                  animation reads at human speed. No allocation post-init.
 *
 * References     : • Buck, Jamis — "Maze Generation: Recursive Backtracking"
 *                    (the canonical tutorial):
 *                    https://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking
 *                  • Wikipedia — "Maze generation algorithm":
 *                    https://en.wikipedia.org/wiki/Maze_generation_algorithm
 *                  • Tree-diameter via two BFS:
 *                    https://cp-algorithms.com/graph/tree_painting.html#diameter-of-a-tree
 *                  • Red Blob Games — graph search visualisations:
 *                    https://www.redblobgames.com/pathfinding/a-star/introduction.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A perfect maze on a grid is just a SPANNING TREE: every cell reachable
 * from every other cell, with exactly one route between any pair, no
 * loops. Recursive backtracker builds one by walking randomly until
 * cornered, then unwinding by one step and trying again — exactly the
 * way you'd explore an unfamiliar building. The carved walls aren't
 * "removed walls" so much as "edges of the tree we just built".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture an ant carrying a piece of chalk. The ant marks every floor
 * tile it visits and erases the wall behind it as it crosses. When all
 * four neighbouring tiles are already marked, it walks back along its
 * own marks until it finds an unmarked side-passage, then resumes. The
 * stack in our code is literally that retraced trail.
 *
 * Two visual layers tell the story:
 *   1. The CURRENT HEAD (white) is where the ant is now.
 *   2. The TRAIL (magenta, fading) is where it has been — fresh trail
 *      glows brightly, old trail fades to the maze's resting colour.
 *      When the ant backtracks, the trail momentarily re-brightens
 *      because we touch each retracing cell; that's the visible
 *      signature of "going back to find side-passages".
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Every cell has all 4 walls. Pick a start cell, mark visited,
 *     push on stack.
 *  2. STEP (one operation):
 *     a. Peek the top of the stack — call it A.
 *     b. Find A's unvisited neighbours (those with walls that are still
 *        intact and visited == false on the other side).
 *     c. If any: pick one at random — call it B. Carve the wall between
 *        A and B (clear A's d-bit AND B's opposite-d-bit). Mark B
 *        visited, push B.
 *     d. If none: pop the stack (backtrack one cell).
 *  3. Repeat until the stack is empty. Every cell is now visited; the
 *     wall pattern is a uniform random spanning tree of the grid.
 *  4. SOLVE (maze diameter, optional spectacle):
 *     a. BFS from any cell A → find farthest reachable cell X.
 *     b. BFS from X → find its farthest reachable cell Y, recording
 *        parent[i] for each cell.
 *     c. Walk parent[] from Y back to X — that array is the longest
 *        path in the tree. Stream a gold beam along it.
 *  5. HOLD a moment, then supernova-reset and goto 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Wall bit per direction        : N=1, E=2, S=4, W=8     (1 << d)
 *  opposite(d)                   : (d + 2) mod 4          (N↔S, E↔W)
 *  Wall between A and B (d=A→B)  : clear A.walls bit d
 *                                  clear B.walls bit opposite(d)
 *  Cell idx in flat array        : idx = y * w + x
 *  Maze → screen mapping         : interior at (2y+1, 2x+1)
 *                                  N-wall  at (2y,   2x+1)
 *                                  W-wall  at (2y+1, 2x  )
 *                                  NW-corner at (2y, 2x  )
 *  Maze on screen size           : (2w + 1) × (2h + 1)
 *  Screen → maze (inverse)       : x = (sx - 1) / 2,  y = (sy - 1) / 2
 *  Tree diameter (two-BFS)       : X = farthest(A), Y = farthest(X);
 *                                  diameter = path X..Y, length = dist(X,Y)
 *  Glow decay (per frame)        : glow *= exp(-rate · dt)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DOUBLE WALL STORAGE. Cell A's east wall and cell B's west wall are
 *    the SAME wall — but each cell stores its own bit. Keep both bits in
 *    sync by always carving via maze_carve(A, dir): it clears A's d and
 *    also clears the neighbour's opposite-d. Forgetting to update the
 *    neighbour leaves a wall visible from one side and gone from the
 *    other — looks fine in render (we use cell A for that wall) but
 *    breaks BFS, because B still thinks the wall is up.
 *
 *  • STACK SIZE. The DFS stack can hold up to total_cells entries (when
 *    the dig is at maximum depth, e.g. on a long corridor). Not less.
 *    Sizing it any smaller silently corrupts state when overflow hits.
 *
 *  • OFF-BY-ONE on the (2w+1)×(2h+1) frame. The bottom-right corner of
 *    the frame is at (2h, 2w). If you center with (rows - h)/2, you'll
 *    crop the bottom row of the maze. Use (rows - (2h+1))/2 instead.
 *
 *  • BACKTRACK REVISIT. Popping the stack does NOT "unmark" the cell —
 *    we only pop the position pointer. The cell stays visited so we
 *    don't dig it again. The visible "trail re-brightening on backtrack"
 *    comes from us touching dig_glow during the backtrack, not from
 *    re-visiting in the algorithmic sense.
 *
 *  • CORNER LUT INDEXING. The 16-entry Unicode wall-corner table is
 *    indexed by a 4-bit NESW mask of which OF THE FOUR INCIDENT WALLS
 *    EXIST, not which neighbouring CELLS exist. Off-grid sides count as
 *    "no wall" — so the maze's outer corners (e.g. top-left ┌) come out
 *    correctly without special cases.
 *
 *  • MAZE-DIAMETER vs SHORTEST PATH. We solve for the LONGEST path
 *    (diameter), not the shortest path between fixed endpoints. The
 *    longest path is more visually impressive (fills more of the maze)
 *    and demonstrates a less-obvious BFS trick (two-BFS for tree
 *    diameter). misc/maze.c does the simpler corner-to-corner shortest
 *    path — read it if you want the conventional version.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Initial wall bitmask of every cell = 0b1111 = 15 (all 4 walls). If
 *    any cell starts with a different value, walls[] init is wrong.
 *  • After the dig completes: visited count == total_cells. If lower,
 *    the DFS terminated early (likely a stack-pop bug).
 *  • Wall symmetry: for every interior cell with east wall = 0, the
 *    cell to its east must have west wall = 0. Walk the grid once after
 *    DIG and assert this if you suspect maze_carve is buggy.
 *  • Diameter sanity: the diameter of a uniform spanning tree of an
 *    N-cell grid is empirically Θ(√N) — for a 99×28 = 2772-cell maze
 *    the path is typically 100–300 cells. If solve produces a path
 *    much shorter, BFS is exiting too early; much longer than total
 *    cells and parent[] has a cycle (impossible in a tree → bug).
 *  • Visual: cell (0,0) should have a `+` corner at terminal (gx0, gy0)
 *    when its N and W walls are intact (always true at startup). The
 *    corner LUT collapses every non-zero mask to '+' for ASCII
 *    portability — if a corner shows ' ' where walls are present, the
 *    corner_walls_at logic is reading the wrong neighbour bits.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <locale.h>
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    /* Maze dimensions in CELLS (not screen chars). The renderer needs
     * (2w+1)×(2h+1) terminal cells, so a 99×28 maze fills a 199×57
     * area — fits a 200×60 terminal with HUD margins. */
    MAZE_W_MAX        = 120,
    MAZE_H_MAX        =  40,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    DIG_STEPS_MIN     =   1,
    DIG_STEPS_DEF     =   8,        /* DFS operations per tick */
    DIG_STEPS_MAX     = 256,

    SOLVE_STEPS_DEF   =   1,        /* solution-beam cells per tick → ~60/s */

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* dim grey wall lines     */
    PAIR_VISITED      =   4,        /* maze interior, resting  */
    PAIR_TRAIL        =   5,        /* fresh dig glow (magenta) */
    PAIR_HEAD         =   6,        /* current dig head (white-bold) */
    PAIR_SOLUTION     =   7,        /* diameter beam (gold)    */
    PAIR_SUPERNOVA    =   8,        /* reset flash (yellow)    */
};

/* Glow decay rates — slower than wfc_showcase because the maze is more
 * sparse visually and we want the trail readable for ~1 s. */
#define DIG_GLOW_DECAY      2.5f
#define HEAD_GLOW_DECAY     8.0f    /* head fades fast — only one cell active */
#define SOLUTION_GLOW_DECAY 1.5f
#define SUPERNOVA_DECAY     2.0f
#define GLOW_THRESHOLD      0.05f

/* State-machine timing. */
#define HOLD_SECONDS        2.0f    /* hold finished maze + solution beam */

/* Direction encoding — same N=0, E=1, S=2, W=3 as wfc files. */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };
static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

#define WALL_BIT(d)   (1u << (d))   /* 1=N, 2=E, 4=S, 8=W */
#define ALL_WALLS     0x0Fu

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELLS_MAX  (MAZE_W_MAX * MAZE_H_MAX)

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);   /* reserved bright yellow */
        init_pair(PAIR_HINT,        51, -1);   /* reserved bright cyan   */
        init_pair(PAIR_WALL,       246, -1);   /* mid-grey for walls     */
        init_pair(PAIR_VISITED,     67, -1);   /* steel blue interior    */
        init_pair(PAIR_TRAIL,      201, -1);   /* magenta dig trail      */
        init_pair(PAIR_HEAD,       231, -1);   /* near-white head        */
        init_pair(PAIR_SOLUTION,   220, -1);   /* gold beam              */
        init_pair(PAIR_SUPERNOVA,  226, -1);   /* yellow flash           */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_WALL,      COLOR_WHITE,   -1);
        init_pair(PAIR_VISITED,   COLOR_BLUE,    -1);
        init_pair(PAIR_TRAIL,     COLOR_MAGENTA, -1);
        init_pair(PAIR_HEAD,      COLOR_WHITE,   -1);
        init_pair(PAIR_SOLUTION,  COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
}

/* ===================================================================== */
/* §5  maze                                                               */
/* ===================================================================== */

/*
 * Cell — per-maze-cell state.
 *
 *   walls          : 4-bit bitmask (N=1, E=2, S=4, W=8). Set = wall present.
 *   visited        : true once the dig has touched this cell.
 *   dig_glow       : 1.0 when first dug or re-touched on backtrack;
 *                    decays at DIG_GLOW_DECAY per second.
 *   head_glow      : 1.0 only on the current top of the DFS stack;
 *                    decays at HEAD_GLOW_DECAY per second.
 *   solution_glow  : 1.0 when the solution beam reaches this cell;
 *                    decays at SOLUTION_GLOW_DECAY per second.
 *   supernova_glow : 1.0 immediately after reset; decays at SUPERNOVA_DECAY.
 */
typedef struct {
    uint8_t walls;
    bool    visited;
    float   dig_glow;
    float   head_glow;
    float   solution_glow;
    float   supernova_glow;
} Cell;

/*
 * Maze — the simulation heart.
 *
 * Stage of the run is implicit in two values:
 *   sp > 0          → DFS still digging
 *   sp == 0 && !solved → ready to solve
 *   solved          → ready to hold
 * (The Scene wrapper in §6 owns the explicit state machine that
 * decides what to do in each stage.)
 */
typedef struct {
    int   w, h;
    int   total_cells;
    Cell  cells[CELLS_MAX];

    /* DFS state. */
    int   stack[CELLS_MAX];
    int   sp;                  /* stack pointer; sp==0 means done digging */
    int   visited_count;

    /* BFS scratch — reused for both farthest-finds. */
    int   bfs_queue [CELLS_MAX];
    int   bfs_dist  [CELLS_MAX];
    int   bfs_parent[CELLS_MAX];

    /* Solution path (maze diameter). */
    int   path[CELLS_MAX];
    int   path_len;
    int   solve_progress;      /* index of next cell to light along path */
    bool  solved;
} Maze;

static inline int maze_idx(const Maze *m, int x, int y) { return y * m->w + x; }
static inline bool maze_in_bounds(const Maze *m, int x, int y)
{
    return x >= 0 && x < m->w && y >= 0 && y < m->h;
}

static void maze_reset(Maze *m, int w, int h)
{
    m->w = w;
    m->h = h;
    m->total_cells = w * h;
    m->visited_count = 0;
    m->sp = 0;
    m->path_len = 0;
    m->solve_progress = 0;
    m->solved = false;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        m->cells[i].walls          = ALL_WALLS;
        m->cells[i].visited        = false;
        m->cells[i].dig_glow       = 0.0f;
        m->cells[i].head_glow      = 0.0f;
        m->cells[i].solution_glow  = 0.0f;
        m->cells[i].supernova_glow = 1.0f;   /* reset flash on the whole grid */
    }

    /* Pick a random starting cell, mark it visited, push it. */
    int sx = rand() % w;
    int sy = rand() % h;
    int s  = maze_idx(m, sx, sy);
    m->cells[s].visited = true;
    m->cells[s].dig_glow = 1.0f;
    m->cells[s].head_glow = 1.0f;
    m->visited_count = 1;
    m->stack[m->sp++] = s;
}

/*
 * maze_carve — knock down the wall between cell at (x,y) and its
 * neighbour in direction d. Both sides updated.
 */
static void maze_carve(Maze *m, int x, int y, int d)
{
    int idx  = maze_idx(m, x, y);
    int nx   = x + dir_dx(d);
    int ny   = y + dir_dy(d);
    int nidx = maze_idx(m, nx, ny);

    m->cells[idx ].walls &= (uint8_t)~WALL_BIT(d);
    m->cells[nidx].walls &= (uint8_t)~WALL_BIT(opposite(d));
}

/*
 * maze_dig_step — perform one DFS operation.
 *
 * Returns true if a step happened (something to animate), false if
 * the dig is finished (stack empty).
 *
 * One operation is one of:
 *   • Carve forward into a random unvisited neighbour, push it.
 *   • Pop the stack (backtrack) when no unvisited neighbours.
 *
 * Each call is one operation regardless of which kind. This granularity
 * is what lets the renderer SHOW the head moving and the trail
 * brightening on backtrack: a forward step paints dig_glow on the new
 * cell; a backtrack step paints dig_glow on the cell we popped TO,
 * which makes the trail re-brighten visibly.
 */
static bool maze_dig_step(Maze *m)
{
    if (m->sp <= 0) return false;

    int top = m->stack[m->sp - 1];
    int x   = top % m->w;
    int y   = top / m->w;

    /* Damp the previous head's glow — only the new top of stack should
     * be the bright "ant". (We don't zero it; it just won't be re-set,
     * so the per-frame decay drains it on its own.) */
    m->cells[top].head_glow *= 0.5f;

    /* Find unvisited neighbours. */
    int candidates[N_DIRS];
    int n_cand = 0;
    for (int d = 0; d < N_DIRS; d++) {
        int nx = x + dir_dx(d);
        int ny = y + dir_dy(d);
        if (!maze_in_bounds(m, nx, ny)) continue;
        int nidx = maze_idx(m, nx, ny);
        if (!m->cells[nidx].visited) candidates[n_cand++] = d;
    }

    if (n_cand == 0) {
        /* Backtrack: pop, and brighten the trail at the new top so the
         * user sees the ant retracing. If we popped the very last cell,
         * sp drops to 0 and the dig is over. */
        m->sp--;
        if (m->sp > 0) {
            int new_top = m->stack[m->sp - 1];
            m->cells[new_top].dig_glow  = 1.0f;
            m->cells[new_top].head_glow = 1.0f;
        }
        return true;
    }

    /* Carve forward into a random unvisited neighbour. */
    int d    = candidates[rand() % n_cand];
    int nx   = x + dir_dx(d);
    int ny   = y + dir_dy(d);
    int nidx = maze_idx(m, nx, ny);

    maze_carve(m, x, y, d);
    m->cells[nidx].visited   = true;
    m->cells[nidx].dig_glow  = 1.0f;
    m->cells[nidx].head_glow = 1.0f;
    m->visited_count++;
    m->stack[m->sp++] = nidx;
    return true;
}

/*
 * maze_bfs_farthest — BFS from src; returns the farthest-reachable cell
 * index. Fills bfs_parent[] so the caller can reconstruct the path.
 *
 * Used twice: first from any cell to find one diameter endpoint, then
 * from that endpoint to find the other endpoint AND the path between
 * them. Standard "two-BFS finds tree diameter" trick — works because
 * the dig produces a tree (no cycles).
 */
static int maze_bfs_farthest(Maze *m, int src)
{
    int n = m->total_cells;
    for (int i = 0; i < n; i++) {
        m->bfs_dist  [i] = -1;
        m->bfs_parent[i] = -1;
    }
    int head = 0, tail = 0;
    m->bfs_queue[tail++] = src;
    m->bfs_dist[src] = 0;

    int farthest = src;
    int max_d    = 0;

    while (head < tail) {
        int idx = m->bfs_queue[head++];
        int x   = idx % m->w;
        int y   = idx / m->w;
        for (int d = 0; d < N_DIRS; d++) {
            /* A wall on side d blocks the BFS edge. */
            if (m->cells[idx].walls & WALL_BIT(d)) continue;
            int nx = x + dir_dx(d);
            int ny = y + dir_dy(d);
            if (!maze_in_bounds(m, nx, ny)) continue;
            int nidx = maze_idx(m, nx, ny);
            if (m->bfs_dist[nidx] >= 0) continue;
            m->bfs_dist[nidx]   = m->bfs_dist[idx] + 1;
            m->bfs_parent[nidx] = idx;
            m->bfs_queue[tail++] = nidx;
            if (m->bfs_dist[nidx] > max_d) {
                max_d    = m->bfs_dist[nidx];
                farthest = nidx;
            }
        }
    }
    return farthest;
}

/*
 * maze_compute_diameter — two BFS passes, fills m->path[] with the
 * longest path in the spanning tree (the maze "diameter").
 */
static void maze_compute_diameter(Maze *m)
{
    int a = maze_bfs_farthest(m, 0);    /* one endpoint */
    int b = maze_bfs_farthest(m, a);    /* opposite endpoint, parents from a */

    /* Walk parent[] from b back to a — that's the path. */
    m->path_len = 0;
    int cur = b;
    while (cur != -1 && m->path_len < m->total_cells) {
        m->path[m->path_len++] = cur;
        cur = m->bfs_parent[cur];
    }
    m->solve_progress = 0;
    m->solved = true;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   DIG     — DFS in progress; per tick run dig_steps_per_tick steps.
 *             When sp drops to 0, transition to SOLVE.
 *   SOLVE   — diameter computed; animate the solution beam by advancing
 *             solve_progress one cell at a time.
 *             When all cells lit, transition to HOLD.
 *   HOLD    — display the finished maze + solution beam for HOLD_SECONDS.
 *             Then maze_reset and back to DIG.
 *
 * `paused` freezes everything; `step_request` (single-step) — none here,
 * the showcase is meant to be watched not driven, unlike wfc_learn.c.
 */
typedef enum { SCENE_DIG = 0, SCENE_SOLVE = 1, SCENE_HOLD = 2 } SceneState;

typedef struct {
    Maze        m;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         dig_steps_per_tick;
    int         solve_steps_per_tick;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    maze_reset(&s->m, mw, mh);
    s->state      = SCENE_DIG;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused              = false;
    s->dig_steps_per_tick  = DIG_STEPS_DEF;
    s->solve_steps_per_tick = SOLVE_STEPS_DEF;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay all glows. exp(-rate*dt) is the standard RC-decay form;
     * computed inline because dt is small and the per-cell loop is
     * O(N), not a hot path. */
    float dig_d   = expf(-DIG_GLOW_DECAY      * dt);
    float head_d  = expf(-HEAD_GLOW_DECAY     * dt);
    float sol_d   = expf(-SOLUTION_GLOW_DECAY * dt);
    float nova_d  = expf(-SUPERNOVA_DECAY     * dt);
    int n = s->m.total_cells;
    for (int i = 0; i < n; i++) {
        s->m.cells[i].dig_glow       *= dig_d;
        s->m.cells[i].head_glow      *= head_d;
        s->m.cells[i].solution_glow  *= sol_d;
        s->m.cells[i].supernova_glow *= nova_d;
    }

    switch (s->state) {
    case SCENE_DIG:
        for (int i = 0; i < s->dig_steps_per_tick; i++) {
            if (!maze_dig_step(&s->m)) break;
        }
        if (s->m.sp == 0) {
            maze_compute_diameter(&s->m);
            s->state = SCENE_SOLVE;
        }
        break;

    case SCENE_SOLVE:
        for (int i = 0; i < s->solve_steps_per_tick; i++) {
            if (s->m.solve_progress >= s->m.path_len) break;
            int idx = s->m.path[s->m.solve_progress++];
            s->m.cells[idx].solution_glow = 1.0f;
        }
        if (s->m.solve_progress >= s->m.path_len) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->m.w, s->m.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * wall_corner_glyph[] — ASCII glyph for a corner position keyed by
 * which of the 4 incident wall segments exist.
 *
 * Bits: 1=N, 2=E, 4=S, 8=W. Index 0 = no walls touching → space.
 * Any non-zero mask → '+'.
 *
 * We use ASCII (not Unicode box-drawing) for terminal portability —
 * the classic '+' / '-' / '|' maze look renders correctly on every
 * terminal regardless of locale or font.
 */
static const char wall_corner_glyph[16] = {
    /*  0  ----  */ ' ',
    /*  1  N---  */ '+',
    /*  2  -E--  */ '+',
    /*  3  NE--  */ '+',
    /*  4  --S-  */ '+',
    /*  5  N-S-  */ '+',
    /*  6  -ES-  */ '+',
    /*  7  NES-  */ '+',
    /*  8  ---W  */ '+',
    /*  9  N--W  */ '+',
    /* 10  -E-W  */ '+',
    /* 11  NE-W  */ '+',
    /* 12  --SW  */ '+',
    /* 13  N-SW  */ '+',
    /* 14  -ESW  */ '+',
    /* 15  NESW  */ '+',
};

/*
 * corner_walls_at — what walls touch the corner at maze grid position
 * (cx, cy)? The corner sits at the intersection of up to 4 cells:
 *   NW = (cx-1, cy-1)        NE = (cx,   cy-1)
 *   SW = (cx-1, cy)          SE = (cx,   cy)
 *
 * The 4 incident wall segments are:
 *   N segment  : west edge of NE cell  ↔  east edge of NW cell
 *   S segment  : west edge of SE cell  ↔  east edge of SW cell
 *   E segment  : north edge of SE cell ↔ south edge of NE cell
 *   W segment  : north edge of SW cell ↔ south edge of NW cell
 *
 * Off-grid cells are treated as "no wall on that side" so the maze's
 * outer frame degenerates to the correct corner glyphs.
 */
static int corner_walls_at(const Maze *m, int cx, int cy)
{
    int mask = 0;

    bool ne = maze_in_bounds(m, cx,     cy - 1);
    bool nw = maze_in_bounds(m, cx - 1, cy - 1);
    bool se = maze_in_bounds(m, cx,     cy    );
    bool sw = maze_in_bounds(m, cx - 1, cy    );

    /* North segment exists iff either NE or NW shows a wall on the
     * shared vertical line — that's NE.W or NW.E. */
    if ((ne && (m->cells[maze_idx(m, cx,     cy-1)].walls & WALL_BIT(DIR_W))) ||
        (nw && (m->cells[maze_idx(m, cx - 1, cy-1)].walls & WALL_BIT(DIR_E))))
        mask |= 1;   /* N */

    /* East segment: SE.N or NE.S. */
    if ((se && (m->cells[maze_idx(m, cx, cy    )].walls & WALL_BIT(DIR_N))) ||
        (ne && (m->cells[maze_idx(m, cx, cy - 1)].walls & WALL_BIT(DIR_S))))
        mask |= 2;   /* E */

    /* South segment: SE.W or SW.E. */
    if ((se && (m->cells[maze_idx(m, cx,     cy)].walls & WALL_BIT(DIR_W))) ||
        (sw && (m->cells[maze_idx(m, cx - 1, cy)].walls & WALL_BIT(DIR_E))))
        mask |= 4;   /* S */

    /* West segment: SW.N or NW.S. */
    if ((sw && (m->cells[maze_idx(m, cx - 1, cy    )].walls & WALL_BIT(DIR_N))) ||
        (nw && (m->cells[maze_idx(m, cx - 1, cy - 1)].walls & WALL_BIT(DIR_S))))
        mask |= 8;   /* W */

    return mask;
}

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    setlocale(LC_ALL, "");          /* enable UTF-8 box-drawing */
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * cell_color_for — pick the (pair, attr) for the interior of a maze
 * cell based on which glow is dominant.
 *
 * Priority (highest wins):
 *   supernova_glow  → bright yellow flash
 *   head_glow       → near-white bold (only the current dig head)
 *   solution_glow   → gold beam
 *   dig_glow        → magenta trail
 *   visited         → steel blue resting colour
 *   else            → blank
 */
static bool cell_color_for(const Cell *c, int *pair, int *attr, char *glyph_byte)
{
    *attr = A_NORMAL;
    *glyph_byte = ' ';

    if (c->supernova_glow > GLOW_THRESHOLD) {
        *pair = PAIR_SUPERNOVA;
        *attr = A_BOLD;
        *glyph_byte = '*';
        return true;
    }
    if (c->head_glow > GLOW_THRESHOLD) {
        *pair = PAIR_HEAD;
        *attr = A_BOLD;
        *glyph_byte = '@';
        return true;
    }
    if (c->solution_glow > GLOW_THRESHOLD) {
        *pair = PAIR_SOLUTION;
        *attr = A_BOLD;
        *glyph_byte = '*';
        return true;
    }
    if (c->dig_glow > GLOW_THRESHOLD) {
        *pair = PAIR_TRAIL;
        *attr = A_BOLD;
        *glyph_byte = '.';
        return true;
    }
    if (c->visited) {
        *pair = PAIR_VISITED;
        *attr = A_DIM;
        *glyph_byte = '.';
        return true;
    }
    return false;   /* unvisited — caller skips */
}

/*
 * scene_draw — render the maze into stdscr.
 *
 * Output coordinates per maze cell (mx, my):
 *   interior  at  (gy0 + 2*my + 1, gx0 + 2*mx + 1)
 *   N-wall    at  (gy0 + 2*my,     gx0 + 2*mx + 1)
 *   W-wall    at  (gy0 + 2*my + 1, gx0 + 2*mx    )
 *   NW-corner at  (gy0 + 2*my,     gx0 + 2*mx    )
 *
 * Walls are computed from the cell's own walls bitmask (north and
 * west); corners are computed from the surrounding 4 cells via
 * corner_walls_at to get the right Unicode join glyph.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Maze *m = &s->m;
    int frame_w = 2 * m->w + 1;
    int frame_h = 2 * m->h + 1;

    int gx0 = (cols - frame_w) / 2;
    int gy0 = ((rows - 2) - frame_h) / 2 + 1;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 1) gy0 = 1;

    /* ── walls + corners — top and left edges of every cell, plus the
     * extra bottom row and right column of the outer frame. ──────── */
    for (int my = 0; my <= m->h; my++) {
        for (int mx = 0; mx <= m->w; mx++) {
            int sy_corner = gy0 + 2 * my;
            int sx_corner = gx0 + 2 * mx;

            /* Corner glyph at (mx, my). */
            if (sy_corner >= 0 && sy_corner < rows
                && sx_corner >= 0 && sx_corner < cols) {
                int mask = corner_walls_at(m, mx, my);
                char ch = wall_corner_glyph[mask];
                if (ch != ' ') {
                    attron(COLOR_PAIR(PAIR_WALL));
                    mvaddch(sy_corner, sx_corner,
                            (chtype)(unsigned char)ch);
                    attroff(COLOR_PAIR(PAIR_WALL));
                }
            }

            /* Horizontal wall to the east of the corner — i.e. the
             * north wall of cell (mx, my). Only if mx < m->w. */
            if (mx < m->w) {
                int sx = gx0 + 2 * mx + 1;
                if (sy_corner >= 0 && sy_corner < rows
                    && sx >= 0 && sx < cols) {
                    bool wall;
                    if (my == 0) {
                        wall = (m->cells[maze_idx(m, mx, 0)].walls & WALL_BIT(DIR_N)) != 0;
                    } else if (my == m->h) {
                        wall = (m->cells[maze_idx(m, mx, m->h-1)].walls & WALL_BIT(DIR_S)) != 0;
                    } else {
                        wall = (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_N)) != 0;
                    }
                    if (wall) {
                        attron(COLOR_PAIR(PAIR_WALL));
                        mvaddch(sy_corner, sx, (chtype)(unsigned char)'-');
                        attroff(COLOR_PAIR(PAIR_WALL));
                    }
                }
            }

            /* Vertical wall to the south of the corner — i.e. the
             * west wall of cell (mx, my). Only if my < m->h. */
            if (my < m->h) {
                int sy = gy0 + 2 * my + 1;
                if (sy >= 0 && sy < rows
                    && sx_corner >= 0 && sx_corner < cols) {
                    bool wall;
                    if (mx == 0) {
                        wall = (m->cells[maze_idx(m, 0, my)].walls & WALL_BIT(DIR_W)) != 0;
                    } else if (mx == m->w) {
                        wall = (m->cells[maze_idx(m, m->w-1, my)].walls & WALL_BIT(DIR_E)) != 0;
                    } else {
                        wall = (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_W)) != 0;
                    }
                    if (wall) {
                        attron(COLOR_PAIR(PAIR_WALL));
                        mvaddch(sy, sx_corner, (chtype)(unsigned char)'|');
                        attroff(COLOR_PAIR(PAIR_WALL));
                    }
                }
            }
        }
    }

    /* ── interiors ─────────────────────────────────────────────────── */
    for (int my = 0; my < m->h; my++) {
        int sy = gy0 + 2 * my + 1;
        if (sy < 0 || sy >= rows) continue;
        for (int mx = 0; mx < m->w; mx++) {
            int sx = gx0 + 2 * mx + 1;
            if (sx < 0 || sx >= cols) continue;

            const Cell *c = &m->cells[maze_idx(m, mx, my)];
            int pair, attr;
            char glyph;
            if (!cell_color_for(c, &pair, &attr, &glyph)) continue;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Maze *m = &s->m;
    const char *state_str =
        s->paused                  ? "PAUSED " :
        (s->state == SCENE_DIG)    ? "DIGGING" :
        (s->state == SCENE_SOLVE)  ? "SOLVING" :
                                     "HOLD   ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  steps:%3d  %s  %5d/%-5d ",
             fps, sim_fps, s->dig_steps_per_tick, state_str,
             m->visited_count, m->total_cells);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " RECURSIVE-BACKTRACKER MAZE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   maze_w, maze_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_pick_maze_size — fit a (2w+1)×(2h+1) frame inside the terminal
 * leaving 1 row top (HUD) + 1 row bottom (hint).
 */
static void app_pick_maze_size(App *app)
{
    int avail_w = app->screen.cols - 0;
    int avail_h = app->screen.rows - 2;
    int mw = (avail_w - 1) / 2;
    int mh = (avail_h - 1) / 2;
    if (mw < 4) mw = 4;
    if (mh < 4) mh = 4;
    if (mw > MAZE_W_MAX) mw = MAZE_W_MAX;
    if (mh > MAZE_H_MAX) mh = MAZE_H_MAX;
    app->maze_w = mw;
    app->maze_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_maze_size(app);
    scene_reset(&app->scene, app->maze_w, app->maze_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->maze_w, app->maze_h);
        break;
    case '=': case '+':
        if (s->dig_steps_per_tick < DIG_STEPS_MAX) s->dig_steps_per_tick *= 2;
        if (s->dig_steps_per_tick > DIG_STEPS_MAX) s->dig_steps_per_tick = DIG_STEPS_MAX;
        break;
    case '-':
        s->dig_steps_per_tick /= 2;
        if (s->dig_steps_per_tick < DIG_STEPS_MIN) s->dig_steps_per_tick = DIG_STEPS_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;
    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_maze_size(app);
    scene_init(&app->scene, app->maze_w, app->maze_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
