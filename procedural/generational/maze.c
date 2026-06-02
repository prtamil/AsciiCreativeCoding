/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maze.c — Recursive-Backtracker Maze Generator + BFS Solver
 *
 * DEMO: An empty grid fills with corridors as a depth-first carver
 *       advances cell-by-cell, leaving a yellow '@' frontier.  When
 *       generation finishes a blue BFS flood expands from the top-left;
 *       the moment it touches the bottom-right cell the shortest path
 *       traces back as a glowing green '*' line.  Press r to regenerate.
 *
 * Study alongside: forest_fire.c (same folder) — both are cell-grid
 *   simulations driven by an iterative state machine, and both
 *   colour-code the activity wave on top of a static field.
 *   maze_backtracker.c (same folder) — fancier showcase of the same
 *   carver with glow effects, colour themes, and a longest-path
 *   (diameter) solver instead of this file's BFS shortest-path.
 *
 * Section map:
 *   §1 config+types — sizes, wall bits, phases, colour IDs, Theme, Maze, Scene
 *   §2 performance  — monotonic clock + frame cap
 *   §3 logic        — calc_dims (pure terminal→maze sizing)
 *   §4 simulation   — DFS carve + BFS solve on the Maze, and the per-tick combine
 *   §5 render       — theme palette, mark_cell, draw grid + HUD
 *   §6 app          — signals, resize, key events, main loop
 *
 * Keys:  q/ESC quit   r regen   space skip-to-solve   p pause   1/2/3 sizes
 *        t/T next/prev colour theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/generational/maze.c \
 *       -o maze -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Recursive-backtracker depth-first maze carver
 *                  followed by breadth-first shortest-path solver.
 *                  DFS carves a uniform spanning tree of the grid graph;
 *                  BFS finds the unique tree path from start to goal.
 *
 * Data-structure : Per-cell 4-bit wall bitmask (N=1, E=2, S=4, W=8).
 *                  Carving a wall clears the bit on BOTH sides of the
 *                  edge — symmetric, so wall queries work from either
 *                  cell.  Display: 2 terminal cells per maze cell, so a
 *                  W×H maze needs (2W+1) × (2H+1) characters of screen.
 *
 * Rendering      : Wall lattice on odd/even pixel parities — corners
 *                  '+' at (even,even), horizontal walls '-' at
 *                  (even,odd), vertical walls '|' at (odd,even),
 *                  cell interiors at (odd,odd).  Generation phase shows
 *                  the DFS frontier '@' in yellow; solve phase shows the
 *                  BFS visited set as '.' in cyan and the final path as
 *                  '*' in bright green.
 *
 * Performance    : Generation does GEN_STEPS=4 DFS pushes per frame so
 *                  the carve animation lasts a few seconds at any size.
 *                  Solve does SOL_STEPS=16 BFS pops per frame.  Whole
 *                  algorithm is O(W·H) — even MAZE_H_MAX·MAZE_W_MAX is
 *                  well under 3000 cells.
 *
 * References     : Concept —
 *                  [1] Buck, Jamis — "Mazes for Programmers" (Pragmatic
 *                      Bookshelf, 2015).  The book: recursive backtracker
 *                      and many other algorithms, with working code.
 *                  [2] Buck — "Maze Generation: Recursive Backtracking"
 *                      (the carver this file animates):
 *                      https://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking
 *                  [3] Wikipedia — "Maze generation algorithm":
 *                      https://en.wikipedia.org/wiki/Maze_generation_algorithm
 *                  [4] Cormen, Leiserson, Rivest & Stein — "Introduction to
 *                      Algorithms" (CLRS): BFS/DFS, the basis of both the
 *                      carve and the shortest-path solve.
 *                  Rendering —
 *                  [5] Red Blob Games — "Introduction to A* / BFS", the model
 *                      for the flood-fill + path visualisation:
 *                      https://www.redblobgames.com/pathfinding/a-star/introduction.html
 *                  [6] Padala — "NCURSES Programming HOWTO", TLDP: colour
 *                      pairs, glyph output, non-blocking input, resize.
 *                  [7] xterm 256-colour palette — the index the Theme
 *                      palettes draw from: https://jonasjacek.github.io/colors/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A maze is a spanning tree of the grid: every cell reachable, no loops.
 * DFS carves that tree by walking randomly into unvisited neighbours and
 * knocking out the wall between them; backtracking when stuck guarantees
 * every cell ends up connected.  Once the tree exists the shortest route
 * is unique, and BFS finds it because tree paths have no shortcuts.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a solid block of concrete divided into rooms by tall walls.
 * A miner with a sledgehammer starts at one corner.  At each room she
 * picks a random neighbour she has not yet visited and smashes the wall
 * between them, then walks through.  When all neighbours have already
 * been visited she retraces her steps until she finds one that hasn't.
 * The pattern of broken walls she leaves is a perfect maze.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 * Generation (per DFS step):
 *   1. Look at the cell on top of the stack.
 *   2. Make a random list of its 4 neighbours.
 *   3. Pick the first unvisited one.  If found:
 *        a. Clear the wall bit between the two cells.
 *        b. Mark the neighbour visited and push it.
 *      If none found, pop the stack (backtrack).
 *   4. Repeat until stack empties.
 *
 * Solve (per BFS step):
 *   1. Pop the front of the queue.
 *   2. If it is the goal, walk parent[] back to the start, marking
 *      every cell on the path.  Stop.
 *   3. For each neighbour reachable through an open wall and not yet
 *      visited, record its parent and enqueue it.
 *
 * KEY FORMULAS
 * ────────────
 * Cell ↔ pixel:  pixel_row = 2·cell_row + 1 + HUD_ROWS,
 *                pixel_col = 2·cell_col + 1.
 * Wall between cells (r,c) and (r',c') is open  ⇔  walls[r][c] bit for
 *   the direction (r→r') is 0 and walls[r'][c'] bit for (r'→r) is 0.
 *   The two sides are kept in sync by carving both bits at once.
 * Parent encoding:  parent[r][c] = r·MAZE_W_MAX + c   (–1 = no parent).
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Negative modulo.  Direction shuffles use unsigned indices into a
 *    fixed 4-element array, so this trap doesn't fire — but be careful
 *    not to index walls[] with a negative row from the corner column.
 *  • Stack of size W·H+1.  A pathological carve that visits every cell
 *    before backtracking once needs all W·H slots; the +1 is paranoia.
 *  • Skip-to-solve.  Pressing space during generation must run the DFS
 *    to completion BEFORE starting BFS, otherwise BFS sees half-carved
 *    walls and finds no goal.
 *  • Resize.  SIGWINCH recomputes maze.w/maze.h (calc_dims) and resets the
 *    maze; holding the carve state through a resize would point off-grid.
 *  • A_DIM on grayscale wall colours would make them disappear on a
 *    black terminal.  All wall/frontier/path pairs sit in the bright
 *    half of the 256-colour cube and use A_BOLD to stay visible.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Every cell becomes visited.  By the time generation finishes,
 *    maze.stack_top == 0 and maze.vis is all 1s; visually no '#' remain.
 *  • Path length equals BFS depth at the goal.  Count the '*' cells:
 *    on a 40×10 maze it should be at least 50 (start to far corner is
 *    >= mh+mw–1 cells, often more because the unique tree path
 *    detours).
 *  • Symmetry.  Carving a wall must clear the bit on both sides — if
 *    only one side is cleared, BFS will see the opening from one cell
 *    but not the other and the solve frontier will get stuck.
 *  • Doubling MAZE_W_MAX should roughly double the carve-animation time
 *    (linear in cell count at fixed GEN_STEPS).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * All maze data lives in one Maze; Scene wraps it with the render/run
 * scalars.  Functions take the NARROWEST type — Maze* (mutate) / const Maze*
 * + ints (read) for the deep work; only the orchestrators take Scene*.
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   LOGIC        §3       NOTHING — calc_dims is pure (out-params, no state)
 *   SIMULATION   §4       Maze (walls, vis, stack, phase, BFS scratch, queue)
 *   RENDER       §5       the terminal only — reads the Maze, never writes it
 *   APP          §6       Scene (theme, paused, dims) + g_quit/g_resize; events
 *
 *   No EFFECTS — nothing cosmetic is stored.  The '@' frontier is read from
 *     the DFS stack top, the '.' flood from maze.bfs_vis, the '*' path from
 *     maze.on_path — all SIMULATION state, inspected at render time.
 *   DELAYS — trivial: 'p' sets Scene.paused (freezes the tick); the only wait
 *     is the PERFORMANCE frame cap (§2).  'space' skip-to-solve is not a delay
 *     — it runs the algorithm straight to completion.
 *
 * PER-TICK COMBINE — step_simulation(Scene*) (§4) is the ONE place state
 * advances: GENERATE → GEN_STEPS × gen_step (at stack-empty: solve_start →
 * SOLVE); SOLVE → SOL_STEPS × solve_step (at goal / queue-empty → DONE).
 * RENDER (scene_draw) then PERFORMANCE (frame cap) run every frame, after.
 *
 * USER EVENTS are NOT the tick: keys (handle_key) and resize mutate the Scene
 * directly in §6 — reset, resize re-fit, theme/size cycles, skip-to-solve —
 * but only step_simulation runs the per-frame tick.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config + types                                                     */
/* ===================================================================== */

#define MAZE_W_MAX    90
#define MAZE_H_MAX    23
#define HUD_ROWS       2

#define TARGET_FPS    30
#define NS_PER_SEC    1000000000LL
#define FRAME_NS      (NS_PER_SEC / TARGET_FPS)

#define GEN_STEPS      4   /* DFS pushes per frame during generation */
#define SOL_STEPS     16   /* BFS pops per frame during solve        */

/* Wall bitmask: 4 bits per cell — one per cardinal direction. */
#define WALL_N  1
#define WALL_E  2
#define WALL_S  4
#define WALL_W  8

#define N_DIRS  4   /* four cardinal directions, indexed 0..3 = N, E, S, W */

/* The run's lifecycle.  GENERATE → SOLVE happens automatically when the
 * carve stack empties; SOLVE → DONE when the BFS reaches the goal (or its
 * queue drains).  The renderer keys its glyph set off this phase. */
enum Phase { PH_GENERATE, PH_SOLVE, PH_DONE };

/* Colour-pair IDs — one per semantic layer the renderer draws.  The six maze
 * layers (PAIR_WALL..PAIR_UNVISIT) are recoloured per Theme; PAIR_HUD/HINT are
 * reserved 8/9 across all demos so the status line and hint look identical
 * everywhere, and stay fixed yellow/cyan against any theme. */
enum {
    PAIR_WALL = 1,   /* solid maze walls and corners                 */
    PAIR_VISIT,      /* carved cell interior                         */
    PAIR_FRONT,      /* DFS frontier '@' during generation           */
    PAIR_BFS,        /* BFS-visited cells '.' during solve           */
    PAIR_PATH,       /* final shortest path '*' (matrix-green)       */
    PAIR_UNVISIT,    /* unvisited cell during generation             */
    PAIR_HUD   = 8,  /* top-right status line (yellow, A_BOLD)       */
    PAIR_HINT  = 9   /* bottom-left key hint    (cyan,   A_BOLD)     */
};

/*
 * Theme — recolours the maze's six activity layers.  The HUD pairs stay fixed
 * bright yellow/cyan for legibility against any theme.  Cycled with t/T.
 *
 * WHY every index is HIGH (≥30, and ≥244 for greys): the resting interior and
 * uncarved fill are drawn plain/dim, and the bottom of the colour cube / grey
 * ramp vanishes on a black terminal (project palette rule); the head/flood/
 * path glyphs are A_BOLD and can run hotter.  Indices: xterm-256 palette [7].
 */
typedef struct {
    const char *name;  /* HUD label, e.g. "CLASSIC"                          */
    short wall;        /* the maze wall lattice (+ - |)                      */
    short visit;       /* carved-but-resting interior (drawn as a blank cell)*/
    short front;       /* the DFS carve head '@'                             */
    short bfs;         /* the BFS flood '.'                                  */
    short path;        /* the final shortest path '*'                        */
    short unvisit;     /* not-yet-carved fill '#'                            */
} Theme;

static const Theme THEMES[] = {
    /*  name        wall  visit  head  flood  path  unvis */
    { "CLASSIC",    251,  244,   226,  117,    46,   240 },
    { "OCEAN",      245,   67,    51,   39,   231,    24 },
    { "EMBER",      244,  130,   226,  208,   196,    52 },
    { "FOREST",     244,   71,   154,   40,   226,    28 },
    { "MONO",       250,  245,   231,  248,   255,   240 },
};
#define N_THEMES  ((int)(sizeof THEMES / sizeof THEMES[0]))

/*
 * Maze — the grid plus all working storage to carve it (DFS) and then solve
 * it (BFS).  A "perfect maze" is a uniform spanning tree of the grid graph:
 * every cell reachable, exactly one route between any pair, no loops (Buck
 * [1][2]).  The tree itself lives in walls[]; everything else is the two
 * algorithms' scratch (CLRS [4] for DFS/BFS).
 *
 * WHY walls are a BITMASK stored on BOTH sides of each edge: a cell's 4 bits
 * answer "is side d open?" in one test, and carving clears the bit on the
 * cell AND on its neighbour (the shared edge), so a wall query is correct
 * from either cell — at the cost of always clearing the twin bits together.
 */
typedef struct {
    /* ── structure: the spanning tree itself ── */
    int w, h;                                      /* active size in cells (≤ MAZE_*_MAX)    */
    unsigned char walls[MAZE_H_MAX][MAZE_W_MAX];   /* 4-bit mask N=1 E=2 S=4 W=8; set=closed */
    enum Phase    phase;                           /* which algorithm is running             */

    /* ── DFS carve frontier ── */
    unsigned char vis[MAZE_H_MAX][MAZE_W_MAX];     /* 1 once the carve has reached a cell    */
    /* explicit recursion stack of (r,c); sized W·H+1 because one long snaking
     * corridor can stack every cell before the first backtrack (+1 = paranoia). */
    struct { int r, c; } stack[MAZE_H_MAX * MAZE_W_MAX + 1];
    int  stack_top;                                /* depth; 0 ⇒ carve finished              */

    /* ── BFS solve scratch (start = (0,0), goal = bottom-right) ── */
    int           parent [MAZE_H_MAX][MAZE_W_MAX]; /* predecessor, encoded r*MAZE_W_MAX+c; -1 = none */
    unsigned char bfs_vis[MAZE_H_MAX][MAZE_W_MAX]; /* 1 once the flood has reached a cell    */
    unsigned char on_path[MAZE_H_MAX][MAZE_W_MAX]; /* 1 if on the reconstructed shortest path */
    struct { int r, c; } queue[MAZE_H_MAX * MAZE_W_MAX];  /* BFS FIFO of cells still to expand */
    int  q_head, q_tail;                           /* FIFO: pop at head, push at tail        */
} Maze;

/*
 * Scene — the animated maze run, as a table of contents:
 *   WHAT  : maze — the grid being carved and solved.
 *   HOW   : theme — the active colour palette (render selection).
 *   when  : paused — freeze the tick.   WHERE : rows, cols — terminal size.
 */
typedef struct {
    Maze maze;        /* WHAT: the maze being carved + solved   */
    int  theme;       /* RENDER: index into THEMES              */
    bool paused;      /* run-state: freeze the per-tick combine */
    int  rows, cols;  /* WHERE: terminal size in characters     */
} Scene;

static Scene g_scene;

/* ===================================================================== */
/* §2  performance   (monotonic clock + frame-cap sleep)                  */
/* ===================================================================== */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  logic   (pure decisions: no mutation, no I/O — readable alone)     */
/* ===================================================================== */

/* Largest maze (in cells) whose (2w+1)×(2h+1) frame fits the terminal,
 * leaving HUD_ROWS at the top and 1 hint row at the bottom; clamped to the
 * static maxima and a 2×2 floor.  Pure: writes only the mh, mw out-params. */
static void calc_dims(int rows, int cols, int *mh, int *mw)
{
    *mh = (rows - HUD_ROWS - 1) / 2;
    *mw = (cols - 1) / 2;
    if (*mh > MAZE_H_MAX) *mh = MAZE_H_MAX;
    if (*mw > MAZE_W_MAX) *mw = MAZE_W_MAX;
    if (*mh < 2) *mh = 2;
    if (*mw < 2) *mw = 2;
}

/* True if cell (r,c) is inside the active w×h maze. */
static bool in_grid(const Maze *m, int r, int c)
{
    return r >= 0 && r < m->h && c >= 0 && c < m->w;
}

/* True if (r,c) is the cell the DFS carver is standing on (its stack top) —
 * the '@' frontier the renderer highlights during generation. */
static bool is_carve_head(const Maze *m, int r, int c)
{
    return m->stack_top > 0 &&
           m->stack[m->stack_top - 1].r == r &&
           m->stack[m->stack_top - 1].c == c;
}

/* ===================================================================== */
/* §4  simulation   (advances the Maze: DFS carve, BFS solve, tick combine) */
/* ===================================================================== */

/* Direction tables: index 0..3 = N, E, S, W. */
static const int DR[N_DIRS]     = { -1,  0,  1,  0 };
static const int DC[N_DIRS]     = {  0,  1,  0, -1 };
static const int D_WALL[N_DIRS] = { WALL_N, WALL_E, WALL_S, WALL_W };
static const int D_OPP[N_DIRS]  = { WALL_S, WALL_W, WALL_N, WALL_E };

/* Fisher-Yates shuffle of the four direction indices — randomises which
 * neighbour the carver tries first, which is what makes each maze unique. */
static void shuffle_dirs(int dirs[N_DIRS])
{
    for (int i = N_DIRS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = dirs[i]; dirs[i] = dirs[j]; dirs[j] = t;
    }
}

/* Carve the wall between cell (r,c) and its neighbour in direction d (both
 * sides), mark the neighbour visited, and push it — one forward DFS step. */
static void carve_into(Maze *m, int r, int c, int d)
{
    int nr = r + DR[d], nc = c + DC[d];
    m->vis[nr][nc] = 1;
    m->walls[r ][c ] &= (unsigned char)~D_WALL[d];
    m->walls[nr][nc] &= (unsigned char)~D_OPP [d];
    m->stack[m->stack_top].r = nr;
    m->stack[m->stack_top].c = nc;
    m->stack_top++;
}

static void maze_reset(Maze *m)
{
    memset(m->walls, 0x0F, sizeof m->walls); /* every wall closed */
    memset(m->vis,   0,    sizeof m->vis);
    m->stack[0].r = 0;
    m->stack[0].c = 0;
    m->stack_top  = 1;
    m->vis[0][0]  = 1;
    m->phase      = PH_GENERATE;
}

/*
 * gen_step() — advance the recursive backtracker by one node.
 *
 * We pick a random unvisited neighbour of the stack-top cell, carve
 * the wall between them, push the neighbour, and return.  If the top
 * cell has no unvisited neighbour, we pop (backtrack).  Returns false
 * once the stack empties — the maze is complete.
 */
static bool gen_step(Maze *m)
{
    if (m->stack_top == 0) return false;        /* stack empty → carve complete */

    int r = m->stack[m->stack_top - 1].r;
    int c = m->stack[m->stack_top - 1].c;

    int dirs[N_DIRS] = { 0, 1, 2, 3 };
    shuffle_dirs(dirs);
    for (int k = 0; k < N_DIRS; k++) {
        int d  = dirs[k];
        int nr = r + DR[d], nc = c + DC[d];
        if (in_grid(m, nr, nc) && !m->vis[nr][nc]) {
            carve_into(m, r, c, d);             /* advance the frontier */
            return true;
        }
    }
    m->stack_top--;                             /* boxed in → backtrack */
    return true;
}

static void solve_start(Maze *m)
{
    memset(m->bfs_vis, 0, sizeof m->bfs_vis);
    memset(m->on_path, 0, sizeof m->on_path);
    memset(m->parent, -1, sizeof m->parent);
    m->q_head = m->q_tail = 0;
    m->queue[m->q_tail].r = 0;
    m->queue[m->q_tail].c = 0;
    m->q_tail++;
    m->bfs_vis[0][0] = 1;
    m->phase = PH_SOLVE;
}

/*
 * trace_path() — walk the parent chain back from the goal cell to the
 * start, marking each cell along the way as on the shortest path.
 */
static void trace_path(Maze *m, int gr, int gc)
{
    int pr = gr, pc = gc;
    while (pr >= 0 && pc >= 0) {
        m->on_path[pr][pc] = 1;
        int enc = m->parent[pr][pc];
        if (enc < 0) break;
        pr = enc / MAZE_W_MAX;
        pc = enc % MAZE_W_MAX;
    }
}

/* Enqueue every not-yet-flooded neighbour of (r,c) reachable through an open
 * wall, recording (r,c) as its BFS parent — one expansion of the flood. */
static void bfs_expand(Maze *m, int r, int c)
{
    for (int d = 0; d < N_DIRS; d++) {
        if (m->walls[r][c] & D_WALL[d]) continue;          /* wall closed → no edge */
        int nr = r + DR[d], nc = c + DC[d];
        if (!in_grid(m, nr, nc) || m->bfs_vis[nr][nc]) continue;
        m->bfs_vis[nr][nc] = 1;
        m->parent [nr][nc] = r * MAZE_W_MAX + c;
        m->queue[m->q_tail].r = nr;
        m->queue[m->q_tail].c = nc;
        m->q_tail++;
    }
}

static bool solve_step(Maze *m)
{
    if (m->q_head >= m->q_tail) { m->phase = PH_DONE; return false; }  /* no route */

    int r = m->queue[m->q_head].r;
    int c = m->queue[m->q_head].c;
    m->q_head++;

    if (r == m->h - 1 && c == m->w - 1) {       /* reached the bottom-right goal */
        trace_path(m, r, c);
        m->phase = PH_DONE;
        return false;
    }

    bfs_expand(m, r, c);
    return true;
}

/*
 * step_simulation() — THE per-tick combine.  The one place state advances:
 * GENERATE runs GEN_STEPS carves (auto-starting the solve when the carve
 * finishes); SOLVE runs SOL_STEPS BFS pops.  `paused` freezes it.
 */
static void step_simulation(Scene *s)
{
    if (s->paused) return;
    Maze *m = &s->maze;
    if (m->phase == PH_GENERATE) {
        for (int i = 0; i < GEN_STEPS; i++)
            if (!gen_step(m)) { solve_start(m); break; }
    } else if (m->phase == PH_SOLVE) {
        for (int i = 0; i < SOL_STEPS; i++)
            if (!solve_step(m)) break;
    }
}

/* ===================================================================== */
/* §5  render   (state → screen: reads the Maze, mutates only the terminal) */
/* ===================================================================== */

/*
 * color_apply() — install the active theme's palette.  Every pair uses
 * bg = -1 (terminal default) so the maze blends into any wallpaper.  The
 * six maze layers come from `th`; the HUD/hint pairs stay fixed bright
 * yellow/cyan so the status line is legible against every theme.  Called
 * once at startup and again on each t/T cycle.
 */
static void color_apply(const Theme *th)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_WALL,    th->wall,    -1);
        init_pair(PAIR_VISIT,   th->visit,   -1);
        init_pair(PAIR_FRONT,   th->front,   -1);
        init_pair(PAIR_BFS,     th->bfs,     -1);
        init_pair(PAIR_PATH,    th->path,    -1);
        init_pair(PAIR_UNVISIT, th->unvisit, -1);
        init_pair(PAIR_HUD,     226, -1);   /* fixed yellow status (A_BOLD) */
        init_pair(PAIR_HINT,     51, -1);   /* fixed cyan hint     (A_BOLD) */
    } else {
        init_pair(PAIR_WALL,    COLOR_WHITE,  -1);
        init_pair(PAIR_VISIT,   COLOR_WHITE,  -1);
        init_pair(PAIR_FRONT,   COLOR_YELLOW, -1);
        init_pair(PAIR_BFS,     COLOR_CYAN,   -1);
        init_pair(PAIR_PATH,    COLOR_GREEN,  -1);
        init_pair(PAIR_UNVISIT, COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,     COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,    COLOR_CYAN,   -1);
    }
}

/*
 * mark_cell() — bounds-checked mvaddch with the canonical
 * (chtype)(unsigned char) double-cast that prevents sign-extension
 * corruption on glyphs above 0x7F.  All maze drawing routes through
 * this single helper, so clipping (against the rows/cols viewport) lives
 * in exactly one place.
 */
static void mark_cell(int sr, int sc, char ch, int pair, int attr, int rows, int cols)
{
    if (sr < 0 || sr >= rows) return;
    if (sc < 0 || sc >= cols) return;
    chtype c = (chtype)(unsigned char)ch;
    if (pair) c |= (chtype)COLOR_PAIR(pair);
    if (attr) c |= (chtype)attr;
    mvaddch(sr, sc, c);
}

/*
 * Pixel-grid layout:
 *   row 0 .. HUD_ROWS-1            → HUD lines
 *   row HUD_ROWS + 2*r              → wall row above maze cell row r
 *   row HUD_ROWS + 2*r + 1          → maze cell row r
 * Cell parities (relative to the maze pixel rectangle):
 *   (even,even) → corner '+'
 *   (even,odd)  → horizontal wall '-' or open ' '
 *   (odd,even)  → vertical wall   '|' or open ' '
 *   (odd,odd)   → cell interior
 */
static void draw_lattice_pixel(const Maze *m, int pr, int pc, int rows, int cols)
{
    int sr = pr + HUD_ROWS;
    bool corner = !(pr & 1) && !(pc & 1);
    bool hwall  = !(pr & 1) &&  (pc & 1);
    bool vwall  =  (pr & 1) && !(pc & 1);

    if (corner) {
        mark_cell(sr, pc, '+', PAIR_WALL, A_BOLD, rows, cols);
        return;
    }
    if (hwall) {
        int r = pr / 2 - 1, c = pc / 2;
        bool open = (r >= 0) && !(m->walls[r][c] & WALL_S);
        if (open) mark_cell(sr, pc, ' ', 0, 0, rows, cols);
        else      mark_cell(sr, pc, '-', PAIR_WALL, A_BOLD, rows, cols);
        return;
    }
    if (vwall) {
        int r = pr / 2, c = pc / 2 - 1;
        bool open = (c >= 0) && !(m->walls[r][c] & WALL_E);
        if (open) mark_cell(sr, pc, ' ', 0, 0, rows, cols);
        else      mark_cell(sr, pc, '|', PAIR_WALL, A_BOLD, rows, cols);
    }
}

/* Pick the glyph + colour for the interior of a maze cell. */
static void draw_cell_interior(const Maze *m, int r, int c, int sr, int sc,
                               int rows, int cols)
{
    if (m->phase == PH_GENERATE) {
        if (is_carve_head(m, r, c)) mark_cell(sr, sc, '@', PAIR_FRONT, A_BOLD, rows, cols);
        else if (m->vis[r][c])      mark_cell(sr, sc, ' ', PAIR_VISIT, 0, rows, cols);
        else                        mark_cell(sr, sc, '#', PAIR_UNVISIT, 0, rows, cols);
        return;
    }
    /* PH_SOLVE or PH_DONE */
    if (m->on_path[r][c])      mark_cell(sr, sc, '*', PAIR_PATH, A_BOLD, rows, cols);
    else if (m->bfs_vis[r][c]) mark_cell(sr, sc, '.', PAIR_BFS,  0, rows, cols);
    else if (m->vis[r][c])     mark_cell(sr, sc, ' ', PAIR_VISIT, 0, rows, cols);
    else                       mark_cell(sr, sc, '#', PAIR_UNVISIT, 0, rows, cols);
}

static void draw_grid(const Maze *m, int rows, int cols)
{
    int max_pr = 2 * m->h;
    int max_pc = 2 * m->w;
    for (int pr = 0; pr <= max_pr; pr++) {
        for (int pc = 0; pc <= max_pc; pc++) {
            bool inside = (pr & 1) && (pc & 1);
            if (inside) {
                int r  = pr / 2;
                int c  = pc / 2;
                int sr = pr + HUD_ROWS;
                draw_cell_interior(m, r, c, sr, pc, rows, cols);
            } else {
                draw_lattice_pixel(m, pr, pc, rows, cols);
            }
        }
    }
}

static void draw_hud(const Scene *s)
{
    const Maze *m = &s->maze;
    const char *phase_str =
        (m->phase == PH_GENERATE) ? "carving (DFS)" :
        (m->phase == PH_SOLVE)    ? "solving (BFS)" :
                                    "done";
    char status[80];
    snprintf(status, sizeof status, " maze %dx%d  %s  %s  %s ",
             m->w, m->h, THEMES[s->theme].name, phase_str,
             s->paused ? "PAUSED" : "running");

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    int pad = s->cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
        " q:quit  r:regen  spc:skip-to-solve  p:pause  1/2/3:size  t:theme ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(const Scene *s)
{
    draw_grid(&s->maze, s->rows, s->cols);
    draw_hud(s);
}

/* ===================================================================== */
/* §6  app   (signals, user events, and the main loop)                    */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void install_signals(void)
{
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH, sig_h);
}

/* Put the terminal into raw, non-blocking, no-cursor mode for animation. */
static void terminal_init(void)
{
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
}

static void handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case 'r': case 'R': maze_reset(&s->maze); break;
    case 'p': case 'P': s->paused = !s->paused; break;
    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        color_apply(&THEMES[s->theme]);
        break;
    case 'T':
        s->theme = (s->theme - 1 + N_THEMES) % N_THEMES;
        color_apply(&THEMES[s->theme]);
        break;
    case ' ':
        if (s->maze.phase == PH_GENERATE) {
            while (gen_step(&s->maze)) {}
            solve_start(&s->maze);
        } else if (s->maze.phase == PH_SOLVE) {
            while (solve_step(&s->maze)) {}
            s->maze.phase = PH_DONE;
        }
        break;
    case '1': s->maze.h = 10;          s->maze.w = 40;          maze_reset(&s->maze); break;
    case '2': calc_dims(s->rows, s->cols, &s->maze.h, &s->maze.w); maze_reset(&s->maze); break;
    case '3': s->maze.h = MAZE_H_MAX;  s->maze.w = MAZE_W_MAX;  maze_reset(&s->maze); break;
    default: break;
    }
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    install_signals();
    terminal_init();
    color_apply(&THEMES[g_scene.theme]);

    getmaxyx(stdscr, g_scene.rows, g_scene.cols);
    calc_dims(g_scene.rows, g_scene.cols, &g_scene.maze.h, &g_scene.maze.w);
    maze_reset(&g_scene.maze);

    while (!g_quit) {
        long long frame_start = clock_ns();

        /* USER EVENT: resize — re-fit and reset (not a tick) */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_scene.rows, g_scene.cols);
            calc_dims(g_scene.rows, g_scene.cols, &g_scene.maze.h, &g_scene.maze.w);
            maze_reset(&g_scene.maze);
        }

        /* USER EVENT: keys */
        int ch = getch();
        if (ch != ERR) handle_key(&g_scene, ch);

        step_simulation(&g_scene);   /* PER-TICK COMBINE (see ARCHITECTURE) */

        /* RENDER then PERFORMANCE frame cap */
        erase();
        scene_draw(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();

        /* Frame cap — `elapsed = clock_ns() - frame_start` (no +dt!). */
        long long elapsed = clock_ns() - frame_start;
        clock_sleep_ns(FRAME_NS - elapsed);
    }
    return 0;
}
