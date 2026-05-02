/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wilsons_algorithms_maze_showcase.c
 *   — Wilson's algorithm maze generator, animated.
 *
 * DEMO: Begins with a single seed cell already in the maze; everything
 *       else is empty. A glowing white '@' walker starts at a random
 *       outside cell and wanders by uniform random walk, leaving a
 *       trail of direction arrows ('^>v<') showing each step. When the
 *       walk crosses itself, the looped portion FLASHES RED and erases
 *       — that's loop-erasure, the trick that makes the result a
 *       UNIFORM random spanning tree. When the walk reaches a cell
 *       already in the maze, the entire trail streams into the maze in
 *       a magenta wave, walls carving cell-by-cell. Repeat from a new
 *       outside cell. After every cell is absorbed, two BFS passes
 *       find the maze's longest path (its diameter) and a gold beam
 *       streams along it. Hold, supernova flash, restart forever.
 *
 * Study alongside: ./maze_backtracker.c — the same maze problem solved
 *       by depth-first search. Backtracker is faster and produces
 *       long-corridor-biased trees; Wilson's is slower and produces
 *       UNIFORM spanning trees (every possible maze equally likely).
 *       The visual difference: backtracker grows from one centre,
 *       Wilson's grows from many random fingers.
 *
 * Section map:
 *   §1 config   — grid, walk pace, glow rates, palette
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + walk / head / erase / absorb / solution
 *   §5 maze     — Cell, Maze, Wilson walk + loop-erase + absorb, BFS
 *   §6 scene    — WALKING / ABSORBING / SOLVING / HOLD state machine
 *   §7 screen   — ASCII walls, walk arrows, glows
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate restart)
 *   + / =      more walk steps per tick (faster)
 *   -          fewer walk steps per tick (slower)
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra wilsons_algorithms_maze_showcase.c \
 *       -o wilsons_maze -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Wilson's algorithm (Wilson 1996) generates a UNIFORM
 *                  random spanning tree of a graph by loop-erased random
 *                  walks. Procedure:
 *                    (1) Pick any vertex; mark it as "in the tree".
 *                    (2) Pick any vertex v NOT in the tree; start a
 *                        uniform random walk from v.
 *                    (3) Walk until reaching a tree vertex. Whenever the
 *                        walk revisits a cell already on its own path,
 *                        ERASE the entire loop (loop-erasure).
 *                    (4) Add the loop-erased walk to the tree.
 *                    (5) Repeat from (2) until every vertex is in tree.
 *                  The resulting tree is uniformly distributed over all
 *                  spanning trees of the graph — a property the simpler
 *                  recursive-backtracker DFS does NOT have (DFS produces
 *                  long-corridor-biased trees).
 *
 *                  Solution phase: identical to maze_backtracker.c —
 *                  two BFS passes find the tree diameter, and a gold
 *                  beam streams along the longest path.
 *
 * Data-structure : Per cell: 4-bit wall bitmask (N=1, E=2, S=4, W=8;
 *                  set = wall present), in_maze flag, in_walk flag,
 *                  walk_dir (the direction the walker stepped FROM
 *                  this cell — defines the loop-erased path), and
 *                  six glow floats for the various animation layers.
 *
 *                  Walk state: a flat walk_path[] array recording cells
 *                  in walk order. Loop-erasure = truncate the array to
 *                  the index of the revisited cell + 1, and flag the
 *                  truncated tail with erase_glow for a red flash.
 *
 * Rendering      : ASCII only (per project ASCII-Only rule). Walker = '@'.
 *                  Walk trail cells = direction arrow '^' '>' 'v' '<'
 *                  showing the next step. Maze interiors = dim '.'.
 *                  Walls = '+' for corners and '-' / '|' for segments.
 *                  Glows: walk (cyan), head (white-bold), erase (red
 *                  flash), absorb (magenta wave), solution (gold beam),
 *                  supernova (yellow flash) — all decay exponentially.
 *
 * Performance    : Wilson's expected runtime on a 2-D grid is empirically
 *                  Θ(N · log N) random-walk steps for a tree of N cells —
 *                  slower than DFS's Θ(N) but the property gained is
 *                  uniformity. We throttle to walk_steps_per_tick
 *                  (default 16) so the walker is followable. No
 *                  allocation post-init.
 *
 * References     : • Wilson, D. B. (1996) — "Generating random spanning
 *                    trees more quickly than the cover time" (the
 *                    original paper):
 *                    https://www.cs.cmu.edu/~15859n/RelatedWork/RandomTrees-Wilson.pdf
 *                  • Buck, Jamis — "Maze Generation: Wilson's Algorithm":
 *                    https://weblog.jamisbuck.org/2011/1/20/maze-generation-wilson-s-algorithm
 *                  • Loop-erased random walk overview:
 *                    https://en.wikipedia.org/wiki/Loop-erased_random_walk
 *                  • Tree-diameter via two BFS:
 *                    https://cp-algorithms.com/graph/tree_painting.html#diameter-of-a-tree
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To pick a random spanning tree where every possible tree is equally
 * likely, you can't just walk DFS-style — that biases toward long
 * corridors. Wilson's trick: walk RANDOMLY from outside the tree, and
 * whenever you accidentally cross your own path, ERASE the loop. Once
 * you bump into the existing tree, the loop-erased walk you just made
 * is provably a uniformly-distributed sample of paths, and stitching
 * many such walks together produces a uniform spanning tree.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a snail on graph paper. One square of the paper is "the
 * museum" (in the tree). The snail starts at any other square and
 * crawls randomly. It leaves a sticky trail behind it. If the snail
 * ever crawls onto its own trail — which happens often when wandering
 * randomly — the loop it just made is REMOVED from the trail (the
 * snail "shrinks" the trail back to the crossing point). The snail
 * keeps going. When it finally reaches the museum, the entire trail
 * (now loop-free) is permanently glued to the museum. A new snail is
 * placed at any remaining outside square, and the process repeats.
 *
 * What you SEE on screen:
 *   • Walker '@' = the snail right now.
 *   • Cyan arrows ^ > v < = the trail, each pointing to the next step.
 *   • Sudden RED FLASH = a loop got erased. The cells that flashed
 *     red were on the trail a moment ago and have been wiped out.
 *   • Magenta WAVE = the trail just hit the museum and is being
 *     absorbed cell-by-cell into the permanent maze.
 *   • Steel-blue dots = cells already in the museum (resting maze).
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. All cells have all 4 walls. Pick one random cell, mark it
 *     in_maze. Pick another random cell (not in_maze), start a walk
 *     there: walk_path = [start], walker = start.
 *  2. WALK STEP (one operation):
 *     a. From walker's current cell, pick a random direction d such
 *        that walker + d is in bounds.
 *     b. Set walker_cell.walk_dir = d (record where we stepped).
 *     c. Move walker to next = walker + d.
 *     d. CASE A — next is in_maze: the walk reached the museum.
 *        Switch to ABSORB phase: trace walk_path, carve walls between
 *        consecutive cells, mark each in_maze, clear in_walk.
 *     e. CASE B — next is in_walk: LOOP! Find the index of `next` in
 *        walk_path[]. Mark cells walk_path[index+1 .. walk_len-1] as
 *        in_walk = false, paint erase_glow on them (red flash).
 *        Truncate walk_path to length index+1. Walker is now at next.
 *     f. CASE C — fresh territory: append next to walk_path. Mark
 *        next.in_walk = true, paint walk_glow. Walker at next.
 *  3. When the walk completes (CASE A) and there are still cells with
 *     in_maze == false, pick a new random outside cell and goto 2.
 *  4. When every cell is in_maze, run two BFS passes for the diameter
 *     and animate the gold solution beam. Then HOLD, then reset.
 *
 * KEY FORMULAS
 * ────────────
 *  Wall bit per direction        : N=1, E=2, S=4, W=8     (1 << d)
 *  opposite(d)                   : (d + 2) mod 4
 *  Carve wall (A → B in dir d)   : clear A.walls bit d
 *                                  clear B.walls bit opposite(d)
 *  Cell idx                      : idx = y * w + x
 *  Maze → screen mapping         : interior at (2y+1, 2x+1)
 *                                  N-wall  at (2y,   2x+1)
 *                                  W-wall  at (2y+1, 2x  )
 *                                  NW-corner at (2y, 2x  )
 *  Maze on screen size           : (2w + 1) × (2h + 1)
 *  Loop-erase truncation         : walk_len' = (index of revisit) + 1
 *  Tree diameter (two-BFS)       : same as maze_backtracker.c
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • LOOP-ERASURE OVERWRITE. When walker steps to a cell already on
 *    walk_path[], the SOURCE cell has its walk_dir UPDATED to point at
 *    the destination — this is what re-routes the walk past the loop.
 *    Cells in the loop body (walk_path[index+1 .. walk_len-1]) are
 *    orphaned: they still have walk_dir set in cells[] but they're
 *    no longer reachable by following walk_dirs from walk_start.
 *    Visually we mark them in_walk=false + erase_glow=1.0 so the user
 *    sees them disappear; algorithmically the stale walk_dir bytes do
 *    not hurt because we always overwrite walk_dir when we step.
 *
 *  • RANDOM-WALK CONFINEMENT. The walker may NEVER step onto an
 *    in_maze cell except as the FINAL step. Cells that are already
 *    part of the museum are OFF-LIMITS for normal stepping; only the
 *    "terminate the walk" event reaches them. We enforce this by
 *    picking only directions whose target cell has !in_maze, OR by
 *    treating the in_maze step as "walk done" the moment it happens.
 *    The simpler form is the latter — pick any direction freely; if
 *    the target is in_maze, transition to ABSORB. This is what the
 *    code does.
 *
 *  • UNIFORMITY DEMANDS LOOP-ERASURE. If you skip the loop-erase step
 *    and just walk DFS-like, you get a NON-uniform tree (long-corridor
 *    bias). The whole point of Wilson's is the loop-erase step. If
 *    your output looks identical to recursive-backtracker, you've
 *    bypassed loop-erasure somewhere.
 *
 *  • EARLY-RUN SLOWNESS. Wilson's is slow at the start: there's only
 *    1 cell in the museum, so a random walker takes many steps to
 *    find it. Once the museum is large, walks complete quickly. Total
 *    expected runtime is Θ(N log N) but the constant in the early
 *    phase is much higher than DFS. We compensate by allowing the
 *    user to crank up walk_steps_per_tick.
 *
 *  • OFF-BY-ONE on the (2w+1)×(2h+1) frame — same as
 *    maze_backtracker.c. Use (rows - (2h+1))/2 for vertical centering.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At startup exactly 1 cell is in_maze (the root). If 0 or >1, the
 *    init logic mis-flagged the seed.
 *  • The walker '@' should NEVER appear inside the museum (steel-blue
 *    region). If it does, the in_maze guard in walk_step is broken.
 *  • Loop-erasure visibility: leave the demo running and watch for
 *    sudden red flashes — they should be common (random walks loop
 *    a lot). If you never see red, loop-erasure isn't firing.
 *  • After full generation: in_maze count == total_cells, walk_len == 0,
 *    every cell has at most 4-1=3 walls (every interior cell has at
 *    least one carved wall connecting it to the rest of the tree).
 *  • Diameter sanity: Wilson's UST has a longer expected diameter than
 *    DFS's biased tree on the same grid — typically 1.5-2× longer.
 *    For a 99×28 maze, expect path length 200-500 cells (vs 100-300
 *    for backtracker). If diameter is much shorter, BFS exited early.
 *  • Wall symmetry (same as backtracker): for every interior cell with
 *    east wall = 0, the cell to its east must have west wall = 0.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

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
    /* Wilson's is slow on big mazes AND visually chaotic when the
     * walker covers a lot of ground per second. Cap to ~1200 cells
     * so each individual walk is followable by eye. */
    MAZE_W_MAX        =  60,
    MAZE_H_MAX        =  20,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Walk steps per scene_tick. Default 4 ⇒ ~240 walk steps/sec, slow
     * enough to follow each random move with the eye. Press '+' for
     * faster runs once the algorithm is understood. */
    WALK_STEPS_MIN    =   1,
    WALK_STEPS_DEF    =   4,
    WALK_STEPS_MAX    = 2048,

    /* Absorb cells per tick — animates the magenta wave running along
     * the just-completed walk path. Higher = snappier, lower = wavier. */
    ABSORB_STEPS_DEF  =   3,

    /* Solution beam pace, same as maze_backtracker.c. */
    SOLVE_STEPS_DEF   =   1,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* dim grey wall lines     */
    PAIR_VISITED      =   4,        /* in_maze interior, resting */
    PAIR_WALK         =   5,        /* current walk arrows (cyan) */
    PAIR_HEAD         =   6,        /* walker '@' (white-bold) */
    PAIR_ERASE        =   7,        /* loop-erasure flash (red) */
    PAIR_ABSORB       =   8,        /* absorb wave (magenta)   */
    PAIR_SOLUTION     =   9,        /* diameter beam (gold)    */
    PAIR_SUPERNOVA    =  10,        /* reset flash (yellow)    */
    PAIR_SOURCE       =  11,        /* walk source 'S' (gold-bold) */
};

/* Glow decay rates. Erase is fast (it's a flash); absorb is slow so the
 * wave is readable; walk persists for ~1 s so the walker leaves a
 * visible trail. */
#define WALK_GLOW_DECAY     1.5f    /* arrow trail fade        */
#define HEAD_GLOW_DECAY     8.0f    /* walker halo (1-cell)    */
#define ERASE_GLOW_DECAY    3.0f    /* loop-erase red flash    */
#define ABSORB_GLOW_DECAY   2.0f    /* absorb wave magenta     */
#define SOLUTION_GLOW_DECAY 1.5f
#define SUPERNOVA_DECAY     4.0f    /* fast reset fade so seed visible quickly */
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.0f

/* Direction encoding — same N=0, E=1, S=2, W=3 as other procedural files. */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };
static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

#define WALL_BIT(d)   (1u << (d))
#define ALL_WALLS     0x0Fu

/* Direction arrow glyphs for walk trail — ASCII per CLAUDE.md. */
static inline char dir_arrow(int d)
{
    switch (d) {
    case DIR_N: return '^';
    case DIR_E: return '>';
    case DIR_S: return 'v';
    case DIR_W: return '<';
    }
    return '?';
}

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
        init_pair(PAIR_VISITED,     67, -1);   /* steel blue (in_maze)   */
        init_pair(PAIR_WALK,       117, -1);   /* light cyan (walk trail)*/
        init_pair(PAIR_HEAD,       231, -1);   /* near-white walker      */
        init_pair(PAIR_ERASE,      196, -1);   /* hot red (loop flash)   */
        init_pair(PAIR_ABSORB,     201, -1);   /* magenta absorb wave    */
        init_pair(PAIR_SOLUTION,   220, -1);   /* gold beam              */
        init_pair(PAIR_SUPERNOVA,  226, -1);   /* yellow reset flash     */
        init_pair(PAIR_SOURCE,     214, -1);   /* orange-gold for 'S'    */
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_WALL,      COLOR_WHITE,   -1);
        init_pair(PAIR_VISITED,   COLOR_BLUE,    -1);
        init_pair(PAIR_WALK,      COLOR_CYAN,    -1);
        init_pair(PAIR_HEAD,      COLOR_WHITE,   -1);
        init_pair(PAIR_ERASE,     COLOR_RED,     -1);
        init_pair(PAIR_ABSORB,    COLOR_MAGENTA, -1);
        init_pair(PAIR_SOLUTION,  COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
        init_pair(PAIR_SOURCE,    COLOR_YELLOW,  -1);
    }
}

/* ===================================================================== */
/* §5  maze                                                               */
/* ===================================================================== */

/*
 * Cell — one cell's persistent state.
 *
 *   walls          : 4-bit bitmask (N=1, E=2, S=4, W=8). Set = wall.
 *   in_maze        : true once permanently part of the spanning tree.
 *   in_walk        : true while currently on the active walk_path.
 *   walk_dir       : direction the walker stepped FROM this cell to
 *                    the next cell. Only meaningful when in_walk.
 *   walk_glow      : 1.0 when the walker just visited (or moved
 *                    through) this cell; decays.
 *   head_glow      : 1.0 only on the current walker position; decays
 *                    fast so just a single cell looks "alive".
 *   erase_glow     : 1.0 when the cell is dropped from a loop-erase;
 *                    decays as a red flash.
 *   absorb_glow    : 1.0 when the cell is being absorbed into the
 *                    permanent maze; decays.
 *   solution_glow  : 1.0 when the diameter beam reaches this cell.
 *   supernova_glow : 1.0 immediately after reset; whole-grid yellow.
 */
typedef struct {
    uint8_t walls;
    bool    in_maze;
    bool    in_walk;
    uint8_t walk_dir;
    float   walk_glow;
    float   head_glow;
    float   erase_glow;
    float   absorb_glow;
    float   solution_glow;
    float   supernova_glow;
} Cell;

/*
 * Maze — the whole simulation state.
 *
 * State of the run is implicit in counts:
 *   maze_count < total_cells, walk_len > 0  → walking
 *   absorb_remaining > 0                    → absorbing
 *   maze_count == total_cells               → ready to solve
 *
 * The Scene wrapper in §6 owns the explicit state machine.
 */
typedef struct {
    int   w, h;
    int   total_cells;
    Cell  cells[CELLS_MAX];

    /* Wilson walk state. */
    int   walk_path[CELLS_MAX];   /* cell indices in walk order */
    int   walk_len;               /* 0 means no active walk */
    int   walk_pos;               /* walk_path[walk_len-1] when walk_len>0 */

    int   maze_count;

    /* Absorb-phase state — animation only; algorithm-wise the absorb
     * could be done in one step, but we walk it cell-by-cell for the
     * magenta wave effect. */
    int   absorb_path[CELLS_MAX];
    int   absorb_len;
    int   absorb_progress;        /* index of next cell to absorb */

    /* BFS scratch — same as maze_backtracker.c. */
    int   bfs_queue [CELLS_MAX];
    int   bfs_dist  [CELLS_MAX];
    int   bfs_parent[CELLS_MAX];

    /* Solution path. */
    int   path[CELLS_MAX];
    int   path_len;
    int   solve_progress;
    bool  solved;
} Maze;

static inline int maze_idx(const Maze *m, int x, int y) { return y * m->w + x; }
static inline bool maze_in_bounds(const Maze *m, int x, int y)
{
    return x >= 0 && x < m->w && y >= 0 && y < m->h;
}

/*
 * maze_carve — knock down the wall between cell at (x,y) and its
 * neighbour in direction d. Both sides updated for symmetry — the
 * BFS solver later relies on consistent wall bits on both cells.
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
 * maze_pick_random_outside — pick a random cell where in_maze == false.
 *
 * Reservoir sampling among non-maze cells — single pass, no allocation.
 * Returns -1 if every cell is already in_maze.
 */
static int maze_pick_random_outside(const Maze *m)
{
    int chosen = -1;
    int seen   = 0;
    int n      = m->total_cells;
    for (int i = 0; i < n; i++) {
        if (!m->cells[i].in_maze) {
            seen++;
            if ((rand() % seen) == 0) chosen = i;
        }
    }
    return chosen;
}

/*
 * maze_start_walk — begin a new random walk at cell `start`.
 *
 * Pre: cell `start` is in bounds and !in_maze.
 * Post: walk_path = [start], walk_len = 1, walk_pos = start,
 *       cell.in_walk = true, walk_glow + head_glow flashed.
 */
static void maze_start_walk(Maze *m, int start)
{
    /* Clear previous walk's in_walk flags (defensive — they should
     * already be false because absorb sets them false, and loop-erase
     * sets them false on orphans). */
    for (int i = 0; i < m->walk_len; i++) {
        m->cells[m->walk_path[i]].in_walk = false;
    }
    m->walk_path[0] = start;
    m->walk_len     = 1;
    m->walk_pos     = start;
    m->cells[start].in_walk    = true;
    m->cells[start].walk_glow  = 1.0f;
    m->cells[start].head_glow  = 1.0f;
}

/*
 * maze_reset — full re-init of the maze with one seed cell already
 * "in the museum", and a fresh walk started at another random cell.
 */
static void maze_reset(Maze *m, int w, int h)
{
    m->w = w;
    m->h = h;
    m->total_cells = w * h;
    m->maze_count = 0;
    m->walk_len = 0;
    m->walk_pos = -1;
    m->absorb_len = 0;
    m->absorb_progress = 0;
    m->path_len = 0;
    m->solve_progress = 0;
    m->solved = false;

    int n = w * h;
    for (int i = 0; i < n; i++) {
        m->cells[i].walls          = ALL_WALLS;
        m->cells[i].in_maze        = false;
        m->cells[i].in_walk        = false;
        m->cells[i].walk_dir       = 0;
        m->cells[i].walk_glow      = 0.0f;
        m->cells[i].head_glow      = 0.0f;
        m->cells[i].erase_glow     = 0.0f;
        m->cells[i].absorb_glow    = 0.0f;
        m->cells[i].solution_glow  = 0.0f;
        m->cells[i].supernova_glow = 1.0f;
    }

    /* Seed the museum with one random cell. */
    int seed_x = rand() % w;
    int seed_y = rand() % h;
    int seed   = maze_idx(m, seed_x, seed_y);
    m->cells[seed].in_maze = true;
    m->cells[seed].absorb_glow = 1.0f;
    m->maze_count = 1;

    /* Begin the first walk somewhere else. */
    int start = maze_pick_random_outside(m);
    if (start >= 0) maze_start_walk(m, start);
}

/*
 * maze_walk_step — perform one walk operation.
 *
 * Returns:
 *   1  → stepped (walking continues)
 *   2  → walk hit the maze; absorb path queued, walk_len cleared
 *   0  → no action (walk_len == 0; caller should start another walk
 *        or transition to SOLVE)
 *
 * The three cases (FRESH / LOOP / HIT) are spelled out explicitly so
 * the dataflow is followable while reading.
 */
static int maze_walk_step(Maze *m)
{
    if (m->walk_len == 0) return 0;

    int x = m->walk_pos % m->w;
    int y = m->walk_pos / m->w;

    /* Pick a random direction whose target is in bounds. We retry up
     * to a small constant — every cell has at least 2 in-bounds
     * neighbours (corners have 2), so this terminates quickly. */
    int d, nx, ny;
    do {
        d = rand() % N_DIRS;
        nx = x + dir_dx(d);
        ny = y + dir_dy(d);
    } while (!maze_in_bounds(m, nx, ny));
    int nidx = maze_idx(m, nx, ny);

    /* Record this step at the source. Whether we're extending the walk
     * or rewriting an existing arrow (loop-erasure case), the source
     * cell's walk_dir is now d. */
    m->cells[m->walk_pos].walk_dir = d;
    /* Source cell's head halo damps so only the new walker is bright. */
    m->cells[m->walk_pos].head_glow *= 0.4f;

    /* CASE A — walk reached the museum. Queue the walk for absorb.
     * We DO NOT clear in_walk on the path now — the cells stay marked
     * so they keep showing the cyan arrow trail. absorb_step clears
     * in_walk one cell at a time, so the magenta wave visibly EATS the
     * trail as it advances. */
    if (m->cells[nidx].in_maze) {
        m->absorb_len = 0;
        for (int i = 0; i < m->walk_len; i++)
            m->absorb_path[m->absorb_len++] = m->walk_path[i];
        m->absorb_path[m->absorb_len++] = nidx;
        m->absorb_progress = 0;
        m->walk_len = 0;        /* walking is over */
        return 2;
    }

    /* CASE B — walker stepped onto its own trail. Loop-erase. */
    if (m->cells[nidx].in_walk) {
        /* Find nidx's index in walk_path[]. Linear scan is fine —
         * walks are short (typically 5–50 cells). */
        int hit = -1;
        for (int i = 0; i < m->walk_len; i++) {
            if (m->walk_path[i] == nidx) { hit = i; break; }
        }
        /* Mark cells walk_path[hit+1 .. walk_len-1] as orphaned —
         * red flash + clear in_walk. */
        for (int i = hit + 1; i < m->walk_len; i++) {
            int c = m->walk_path[i];
            m->cells[c].in_walk     = false;
            m->cells[c].erase_glow  = 1.0f;
            m->cells[c].walk_glow  *= 0.3f;   /* dim cyan immediately */
        }
        m->walk_len = hit + 1;
        m->walk_pos = nidx;
        m->cells[nidx].head_glow = 1.0f;
        m->cells[nidx].walk_glow = 1.0f;
        return 1;
    }

    /* CASE C — fresh territory. Extend walk. */
    m->walk_path[m->walk_len++] = nidx;
    m->walk_pos                 = nidx;
    m->cells[nidx].in_walk      = true;
    m->cells[nidx].walk_glow    = 1.0f;
    m->cells[nidx].head_glow    = 1.0f;
    return 1;
}

/*
 * maze_absorb_step — absorb ONE cell from the queued absorb_path
 * into the permanent maze. Carves the wall between consecutive cells
 * along the way.
 *
 * Returns true if a cell was absorbed; false if the absorb is complete.
 *
 * Layout of absorb_path:
 *   absorb_path[0]                 = walk start (was outside)
 *   ...
 *   absorb_path[walk_len-1]        = last walk cell (was outside)
 *   absorb_path[absorb_len-1]      = the maze cell we hit (destination)
 *
 * Each cell C in the FRONT of absorb_path carves the wall between C
 * and the next cell (using C.walk_dir from the walk). The LAST cell
 * (the destination, already in_maze) gets a magenta absorb_glow flash
 * so the user sees the wave terminate inside the museum — but no wall
 * is carved through it (it's already part of the tree).
 */
static bool maze_absorb_step(Maze *m)
{
    if (m->absorb_progress >= m->absorb_len) {
        m->absorb_len = 0;
        m->absorb_progress = 0;
        return false;
    }

    int idx = m->absorb_path[m->absorb_progress];
    bool is_destination = (m->absorb_progress == m->absorb_len - 1);

    if (!is_destination) {
        int x = idx % m->w;
        int y = idx / m->w;
        int d = m->cells[idx].walk_dir;
        maze_carve(m, x, y, d);
    }

    if (!m->cells[idx].in_maze) {
        m->cells[idx].in_maze = true;
        m->maze_count++;
    }
    m->cells[idx].in_walk     = false;
    m->cells[idx].absorb_glow = 1.0f;
    m->cells[idx].walk_glow   = 0.0f;
    m->cells[idx].head_glow   = 0.0f;
    m->absorb_progress++;
    return true;
}

/*
 * BFS for tree diameter — same code shape as maze_backtracker.c §5.
 * See that file for the why; reproduced inline so this file stays
 * self-contained per the project's Self-Contained File Rule.
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

    int farthest = src, max_d = 0;
    while (head < tail) {
        int idx = m->bfs_queue[head++];
        int x   = idx % m->w;
        int y   = idx / m->w;
        for (int d = 0; d < N_DIRS; d++) {
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

static void maze_compute_diameter(Maze *m)
{
    int a = maze_bfs_farthest(m, 0);
    int b = maze_bfs_farthest(m, a);

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
 *   WALKING     — perform walk_steps_per_tick walk steps. Transitions
 *                 to ABSORBING when a step returns 2 (walk hit maze).
 *   ABSORBING   — perform absorb_steps_per_tick absorb steps. When the
 *                 absorb queue empties, either start a new walk
 *                 (maze_count < total_cells) or transition to SOLVING.
 *   SOLVING     — diameter beam animation (1 cell per tick).
 *   HOLD        — wait HOLD_SECONDS, then maze_reset.
 */
typedef enum {
    SCENE_WALKING   = 0,
    SCENE_ABSORBING = 1,
    SCENE_SOLVING   = 2,
    SCENE_HOLD      = 3,
} SceneState;

typedef struct {
    Maze        m;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         walk_steps_per_tick;
    int         absorb_steps_per_tick;
    int         solve_steps_per_tick;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    maze_reset(&s->m, mw, mh);
    s->state      = SCENE_WALKING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused                = false;
    s->walk_steps_per_tick   = WALK_STEPS_DEF;
    s->absorb_steps_per_tick = ABSORB_STEPS_DEF;
    s->solve_steps_per_tick  = SOLVE_STEPS_DEF;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay every glow buffer. expf(-rate*dt) is the textbook RC-decay
     * — computed inline since dt is small and the loop is O(N). */
    float walk_d   = expf(-WALK_GLOW_DECAY     * dt);
    float head_d   = expf(-HEAD_GLOW_DECAY     * dt);
    float erase_d  = expf(-ERASE_GLOW_DECAY    * dt);
    float absorb_d = expf(-ABSORB_GLOW_DECAY   * dt);
    float sol_d    = expf(-SOLUTION_GLOW_DECAY * dt);
    float nova_d   = expf(-SUPERNOVA_DECAY     * dt);
    int n = s->m.total_cells;
    for (int i = 0; i < n; i++) {
        s->m.cells[i].walk_glow      *= walk_d;
        s->m.cells[i].head_glow      *= head_d;
        s->m.cells[i].erase_glow     *= erase_d;
        s->m.cells[i].absorb_glow    *= absorb_d;
        s->m.cells[i].solution_glow  *= sol_d;
        s->m.cells[i].supernova_glow *= nova_d;
    }

    switch (s->state) {

    case SCENE_WALKING: {
        for (int i = 0; i < s->walk_steps_per_tick; i++) {
            int rc = maze_walk_step(&s->m);
            if (rc == 2) {
                s->state = SCENE_ABSORBING;
                break;
            }
            if (rc == 0) break;
        }
    } break;

    case SCENE_ABSORBING: {
        for (int i = 0; i < s->absorb_steps_per_tick; i++) {
            if (!maze_absorb_step(&s->m)) {
                /* Absorb done — start a new walk or move to SOLVE. */
                if (s->m.maze_count >= s->m.total_cells) {
                    maze_compute_diameter(&s->m);
                    s->state = SCENE_SOLVING;
                } else {
                    int start = maze_pick_random_outside(&s->m);
                    if (start >= 0) {
                        maze_start_walk(&s->m, start);
                        s->state = SCENE_WALKING;
                    } else {
                        /* Shouldn't happen — maze_count < total but no
                         * outside cell exists. Be defensive: solve. */
                        maze_compute_diameter(&s->m);
                        s->state = SCENE_SOLVING;
                    }
                }
                break;
            }
        }
    } break;

    case SCENE_SOLVING: {
        for (int i = 0; i < s->solve_steps_per_tick; i++) {
            if (s->m.solve_progress >= s->m.path_len) break;
            int idx = s->m.path[s->m.solve_progress++];
            s->m.cells[idx].solution_glow = 1.0f;
        }
        if (s->m.solve_progress >= s->m.path_len) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
        }
    } break;

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
 * Same canonical pattern as maze_backtracker.c §7 / framework.c §7:
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate
 *
 * ASCII glyphs only:
 *   '+' wall corner, '-' horizontal wall, '|' vertical wall
 *   '@' walker, '^>v<' walk arrows, '*' supernova/absorb/solution,
 *   '.' resting maze cell.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
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
 * cell_visual — pick (pair, attr, glyph) for an interior cell.
 *
 * Priority (highest wins):
 *   supernova_glow   → bright yellow '*'
 *   head_glow        → near-white '@' (the walker, only one cell)
 *   erase_glow       → red '!' (loop-erase flash)
 *   absorb_glow      → magenta '*' (absorb wave)
 *   solution_glow    → gold '*' (diameter beam)
 *   is_source        → orange-gold 'S' (where THIS walk started)
 *   in_walk          → cyan walk arrow ('^>v<')
 *   in_maze          → dim steel-blue '.'
 *   else             → blank (skip)
 *
 * `is_source` is computed by the caller from the maze's walk_path[0]
 * — only one cell is the source at any time. We render it AFTER the
 * dynamic glows (so a walker returning to source briefly shows '@'
 * before reverting to 'S') but BEFORE the regular walk arrow.
 */
static bool cell_visual(const Cell *c, bool is_source,
                        int *pair, int *attr, char *glyph)
{
    *attr  = A_NORMAL;
    *glyph = ' ';

    if (c->supernova_glow > GLOW_THRESHOLD) {
        *pair = PAIR_SUPERNOVA; *attr = A_BOLD; *glyph = '*'; return true;
    }
    if (c->head_glow > GLOW_THRESHOLD) {
        *pair = PAIR_HEAD; *attr = A_BOLD; *glyph = '@'; return true;
    }
    if (c->erase_glow > GLOW_THRESHOLD) {
        *pair = PAIR_ERASE; *attr = A_BOLD; *glyph = '!'; return true;
    }
    if (c->absorb_glow > GLOW_THRESHOLD) {
        *pair = PAIR_ABSORB; *attr = A_BOLD; *glyph = '*'; return true;
    }
    if (c->solution_glow > GLOW_THRESHOLD) {
        *pair = PAIR_SOLUTION; *attr = A_BOLD; *glyph = '*'; return true;
    }
    if (is_source && c->in_walk) {
        *pair = PAIR_SOURCE; *attr = A_BOLD; *glyph = 'S'; return true;
    }
    if (c->in_walk) {
        *pair  = PAIR_WALK;
        *attr  = A_BOLD;
        *glyph = dir_arrow(c->walk_dir);
        return true;
    }
    if (c->in_maze) {
        *pair = PAIR_VISITED; *attr = A_DIM; *glyph = '.'; return true;
    }
    return false;
}

/*
 * scene_draw — render the maze frame (walls + corners) and cell
 * interiors (glows + walk arrows + maze dots) into stdscr.
 *
 * We render walls only where they exist on the cell's bitmask, the
 * corner glyph at every corner intersection (always '+' if any wall
 * touches it, ' ' otherwise — see maze_backtracker.c for the LUT
 * derivation; we use the same '+' collapse here for ASCII portability).
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

    /* ── walls + corners ──────────────────────────────────────────── */
    for (int my = 0; my <= m->h; my++) {
        for (int mx = 0; mx <= m->w; mx++) {
            int sy_corner = gy0 + 2 * my;
            int sx_corner = gx0 + 2 * mx;

            /* Corner: any wall segment incident to this corner? */
            if (sy_corner >= 0 && sy_corner < rows
                && sx_corner >= 0 && sx_corner < cols) {
                bool any = false;
                /* N segment exists iff cell to NE has W wall, OR to NW has E wall */
                if (maze_in_bounds(m, mx,     my-1)
                    && (m->cells[maze_idx(m, mx, my-1)].walls & WALL_BIT(DIR_W))) any = true;
                if (!any && maze_in_bounds(m, mx-1, my-1)
                    && (m->cells[maze_idx(m, mx-1, my-1)].walls & WALL_BIT(DIR_E))) any = true;
                /* E segment: cell SE has N wall, OR NE has S wall */
                if (!any && maze_in_bounds(m, mx,   my)
                    && (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_N))) any = true;
                if (!any && maze_in_bounds(m, mx,   my-1)
                    && (m->cells[maze_idx(m, mx, my-1)].walls & WALL_BIT(DIR_S))) any = true;
                /* S segment: cell SE has W wall, OR SW has E wall */
                if (!any && maze_in_bounds(m, mx,   my)
                    && (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_W))) any = true;
                if (!any && maze_in_bounds(m, mx-1, my)
                    && (m->cells[maze_idx(m, mx-1, my)].walls & WALL_BIT(DIR_E))) any = true;
                /* W segment: cell SW has N wall, OR NW has S wall */
                if (!any && maze_in_bounds(m, mx-1, my)
                    && (m->cells[maze_idx(m, mx-1, my)].walls & WALL_BIT(DIR_N))) any = true;
                if (!any && maze_in_bounds(m, mx-1, my-1)
                    && (m->cells[maze_idx(m, mx-1, my-1)].walls & WALL_BIT(DIR_S))) any = true;
                if (any) {
                    attron(COLOR_PAIR(PAIR_WALL));
                    mvaddch(sy_corner, sx_corner, (chtype)(unsigned char)'+');
                    attroff(COLOR_PAIR(PAIR_WALL));
                }
            }

            /* Horizontal wall — north wall of cell (mx, my). */
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

            /* Vertical wall — west wall of cell (mx, my). */
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

    /* ── interiors ────────────────────────────────────────────────── */
    int source_idx = (m->walk_len > 0) ? m->walk_path[0] : -1;
    for (int my = 0; my < m->h; my++) {
        int sy = gy0 + 2 * my + 1;
        if (sy < 0 || sy >= rows) continue;
        for (int mx = 0; mx < m->w; mx++) {
            int sx = gx0 + 2 * mx + 1;
            if (sx < 0 || sx >= cols) continue;

            int idx = maze_idx(m, mx, my);
            const Cell *c = &m->cells[idx];
            bool is_source = (idx == source_idx);
            int pair, attr;
            char glyph;
            if (!cell_visual(c, is_source, &pair, &attr, &glyph)) continue;

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
        s->paused                       ? "PAUSED  " :
        (s->state == SCENE_WALKING)     ? "WALKING " :
        (s->state == SCENE_ABSORBING)   ? "ABSORB  " :
        (s->state == SCENE_SOLVING)     ? "SOLVING " :
                                          "HOLD    ";

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  walk:%4d  %s  %5d/%-5d ",
             fps, sim_fps, s->walk_steps_per_tick, state_str,
             m->maze_count, m->total_cells);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " WILSON'S ALGORITHM MAZE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " S:source  @:walker  ^>v<:trail  !:erase  *:wave/dest  "
             "q:quit spc:pause r:reset +/-:speed ");
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

static void app_pick_maze_size(App *app)
{
    int avail_w = app->screen.cols;
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
        if (s->walk_steps_per_tick < WALK_STEPS_MAX) s->walk_steps_per_tick *= 2;
        if (s->walk_steps_per_tick > WALK_STEPS_MAX) s->walk_steps_per_tick = WALK_STEPS_MAX;
        break;
    case '-':
        s->walk_steps_per_tick /= 2;
        if (s->walk_steps_per_tick < WALK_STEPS_MIN) s->walk_steps_per_tick = WALK_STEPS_MIN;
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
