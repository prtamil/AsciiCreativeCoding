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
 *       The whole solution path then stays lit until you press r, which
 *       supernova-flashes and starts a fresh maze.
 *
 * Study alongside: maze.c (same folder) — the same algorithm, plainer.
 *       This file adds glow effects, the diameter solver, colour themes,
 *       size presets, and a generate → solve → hold state machine.
 *
 * Section map:
 *   §1 config+types — sizes, themes, presets, glow rates, all struct types
 *   §2 performance  — monotonic clock + sleep + frame cap
 *   §3 logic        — pure helpers: index/geometry, corner mask, glow→colour
 *   §4 simulation   — maze state: reset, carve, dig-step, BFS, diameter
 *   §5 scene        — the per-tick combine (scene_tick) + reset/init
 *   §6 render       — colour, wall-corner LUT, maze + solution + HUD draw
 *   §7 app          — signals, resize, key events, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (immediate restart)
 *   1 … 9 , 0  maze-size preset, simple → complex (0 = largest / fill)
 *   t / T      next / previous colour theme
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
 *                  terminal cells. Wall corners use a 16-entry ASCII
 *                  corner lookup keyed on which of the 4 surrounding wall
 *                  segments exist (NESW bits; any join → '+'). Glows: dig_glow
 *                  (magenta trail), head_glow (white-bold cursor),
 *                  solution_glow (gold beam) — all decay exponentially.
 *
 * Performance    : DIG: O(N) total over the whole run (each cell visited
 *                  once + at most one backtrack pass). SOLVE: 2× O(N) BFS.
 *                  We throttle to dig_steps_per_tick (default 8) so the
 *                  animation reads at human speed. No allocation post-init.
 *
 * References     : Concept —
 *                  [1] Buck, Jamis — "Mazes for Programmers" (Pragmatic
 *                      Bookshelf, 2015).  The book: recursive backtracker
 *                      and a dozen other algorithms, with working code.
 *                  [2] Buck — "Maze Generation: Recursive Backtracking"
 *                      (the canonical tutorial this file follows):
 *                      https://weblog.jamisbuck.org/2010/12/27/maze-generation-recursive-backtracking
 *                  [3] Wikipedia — "Maze generation algorithm":
 *                      https://en.wikipedia.org/wiki/Maze_generation_algorithm
 *                  [4] Cormen, Leiserson, Rivest & Stein — "Introduction to
 *                      Algorithms" (CLRS): BFS/DFS and spanning trees, the
 *                      foundation under the dig and the solver.
 *                  [5] Tree diameter via two BFS passes (proof + code):
 *                      https://cp-algorithms.com/graph/tree_painting.html#diameter-of-a-tree
 *                  Rendering —
 *                  [6] Patel, Amit (Red Blob Games) — interactive graph-search
 *                      and grid visualisations, the model for the glow layers:
 *                      https://www.redblobgames.com/pathfinding/a-star/introduction.html
 *                  [7] Padala — "NCURSES Programming HOWTO", TLDP: colour
 *                      pairs, glyph output, non-blocking input, resize.
 *                  [8] xterm 256-colour palette (the index the themes draw
 *                      from; also the basis for the "bright-half" legibility
 *                      rule on dim/bold glyphs): https://jonasjacek.github.io/colors/
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
 *  5. DONE: keep the whole solution path lit and wait — the user presses
 *     r to supernova-reset and go to 1. No automatic restart.
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
 *  • CORNER LUT INDEXING. The 16-entry ASCII wall-corner table is
 *    indexed by a 4-bit NESW mask of which OF THE FOUR INCIDENT WALLS
 *    EXIST, not which neighbouring CELLS exist. Off-grid sides count as
 *    "no wall" — so the maze's outer corners (e.g. top-left '+') come out
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

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * All state lives in the types declared in §1 (Cell, Maze, Scene, Screen,
 * App).  Every other section is functions grouped by the ONE concern it
 * serves, so what each layer touches is visible from its section alone:
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   LOGIC        §3       NOTHING — pure reads (maze_idx, maze_in_bounds,
 *                         dir_*, corner_walls_at, cell_color_for); no I/O
 *   SIMULATION   §4       Maze: walls, visited, stack/sp, BFS scratch, path
 *   SCENE        §5       the per-tick combine (scene_tick) + reset/init
 *   RENDER       §6       the terminal only — reads Scene, never writes it
 *   APP          §7       App fields (sim_fps, theme, preset, sizes, flags)
 *
 *   EFFECTS — the four glow floats per Cell (dig/head/solution/supernova).
 *             Cosmetic-only: read by RENDER, never consulted by a LOGIC or
 *             SIMULATION decision.  Set where the event happens (maze_dig_step,
 *             maze_reset) and decayed once per tick at the top of scene_tick;
 *             too interleaved with the step to be its own function here.
 *   DELAYS  — trivial: `paused` freezes scene_tick; the only wait is the
 *             PERFORMANCE frame cap (§2).  No holds/timers — the finished
 *             maze persists until r, it is not on a countdown.
 *
 * PER-TICK COMBINE — scene_tick (§5) is the ONE place state advances, in order:
 *     1. EFFECTS    : decay_glows() — fade every glow by exp(-rate·dt)
 *     2. SIMULATION : scene_dig()   → dig_steps × maze_dig_step; at sp==0 run
 *                                     maze_compute_diameter and enter SOLVE
 *                     scene_solve() → light solve_steps path cells; at the
 *                                     path end enter DONE
 *                     scene_hold()  → re-light the whole path (hold it lit)
 *
 * USER EVENTS are NOT the tick: keys (app_handle_key) and resize
 * (app_do_resize) mutate App/Scene directly in §7 — reseeding (scene_reset),
 * re-fitting the maze, re-applying the theme — but none call scene_tick().
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
/* §1  config + types   (the ONLY place data is declared)                 */
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

    RENDER_CAP_FPS    =  60,        /* hard cap on rendered frames/sec (sim ticks run at sim_fps) */
    MAX_FRAME_MS      = 100,        /* clamp one frame's dt — spiral-of-death guard after a stall */
    FPS_UPDATE_MS     = 500,

    /* Colour-pair indices — one per semantic layer the renderer draws (HUD,
     * wall, resting interior, and the four glows).  PAIR_HUD/PAIR_HINT are
     * fixed per CLAUDE.md; PAIR_WALL..PAIR_SUPERNOVA are recoloured per Theme. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* dim grey wall lines     */
    PAIR_VISITED      =   4,        /* maze interior, resting  */
    PAIR_TRAIL        =   5,        /* fresh dig glow (magenta) */
    PAIR_HEAD         =   6,        /* current dig head (white-bold) */
    PAIR_SOLUTION     =   7,        /* diameter beam (gold)    */
    PAIR_SUPERNOVA    =   8,        /* reset flash (yellow)    */
};

/*
 * Theme — recolours the maze's six semantic layers (wall, resting interior,
 * and the four glows).  The HUD stays fixed bright yellow/cyan so it is
 * legible against every theme.  Cycled with t/T.
 *
 * WHY every entry is a HIGH 256-colour index (≥30, and ≥244 for greys): the
 * `visited` interior is drawn A_DIM, and the bottom of the colour cube / grey
 * ramp disappears under A_DIM on a black terminal (project palette rule).  The
 * glows are drawn A_BOLD so they can run hotter.  Indices follow the xterm-256
 * palette [8]; each row of THEMES is one coherent gradient warm/cool/mono.
 */
typedef struct {
    const char *name;     /* HUD label, e.g. "AURORA"                           */
    short wall;           /* maze wall lines (drawn A_NORMAL, mid-grey)          */
    short visited;        /* dug-but-resting interior (A_DIM — keep mid-bright)  */
    short trail;          /* fresh dig-trail glow                               */
    short head;           /* the dig head / cursor                              */
    short solution;       /* the diameter solution beam                         */
    short supernova;      /* whole-grid reset flash                             */
} Theme;

static const Theme THEMES[] = {
    /*  name        wall  visited trail  head  solution  nova */
    { "AURORA",     246,   67,    201,   231,   220,     226 },
    { "EMBER",      240,  130,    202,   231,   220,     196 },
    { "FOREST",     240,   71,    154,   231,   190,      46 },
    { "ICE",        245,   74,     51,   231,   159,     195 },
    { "MONO",       240,  245,    250,   231,   252,     255 },
};
#define N_THEMES  ((int)(sizeof THEMES / sizeof THEMES[0]))

/*
 * Maze-size presets, simple → complex, selected with digit keys (1..9, 0).
 * Each (w,h) is a *desired* cell count; app_pick_maze_size clamps it to what
 * fits the terminal and to MAZE_*_MAX, so a large preset on a small terminal
 * just fills the screen.  The last preset = the maxima → "fill the terminal".
 */
static const struct { const char *name; int w, h; } MAZE_PRESETS[] = {
    { "Tiny",      8,  5 },
    { "Small",    12,  8 },
    { "Cozy",     16, 10 },
    { "Compact",  22, 12 },
    { "Medium",   30, 16 },
    { "Roomy",    40, 20 },
    { "Large",    55, 26 },
    { "Big",      75, 32 },
    { "Huge",    100, 38 },
    { "Max",     MAZE_W_MAX, MAZE_H_MAX },
};
#define N_MAZE_PRESETS  ((int)(sizeof MAZE_PRESETS / sizeof MAZE_PRESETS[0]))

/* Glow decay rates — slower than wfc_showcase because the maze is more
 * sparse visually and we want the trail readable for ~1 s. */
#define DIG_GLOW_DECAY      2.5f
#define HEAD_GLOW_DECAY     8.0f    /* head fades fast — only one cell active */
#define SOLUTION_GLOW_DECAY 1.5f
#define SUPERNOVA_DECAY     2.0f
#define GLOW_THRESHOLD      0.05f

/* Direction encoding — same N=0, E=1, S=2, W=3 as wfc files.  dir_dx/dir_dy/
 * opposite are pure (LOGIC) primitives kept beside the enum they decode. */
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

/*
 * Cell — one square of the maze grid.  Carries both the structural state the
 * algorithm reasons about and the cosmetic "energy" the renderer fades.
 *
 * WHY a wall BITMASK (not four bools): the rule only ever asks "is side d
 * open?", which is one bit test, and carving is one AND-NOT.  Each wall is
 * stored on BOTH neighbouring cells (cell A's east bit == cell B's west bit),
 * so the renderer and the BFS can consult a single cell without looking
 * sideways — at the cost of keeping the twin bits in sync, which is why every
 * carve goes through maze_carve.  Standard recursive-backtracker encoding [2].
 *
 * The four glows are EFFECTS state: decaying intensities in [0,1], cosmetic
 * only — read by the renderer, never consulted by a rule.  Each is set to 1.0
 * at its event and drained by exp(-rate·dt) every tick (scene_tick).
 */
typedef struct {
    /* ── SIMULATION state ── */
    uint8_t walls;          /* 4-bit mask N=1 E=2 S=4 W=8; set = wall present   */
    bool    visited;        /* dig has reached this cell — never dug again      */
    /* ── EFFECTS state (cosmetic glow, 0..1, decays each tick) ── */
    float   dig_glow;       /* magenta trail: 1.0 when dug or retraced          */
    float   head_glow;      /* white head: 1.0 only on the current DFS top      */
    float   solution_glow;  /* gold beam: 1.0 as the diameter path lights up    */
    float   supernova_glow; /* yellow flash: 1.0 across the grid on reset       */
} Cell;

/*
 * Maze — the grid plus all working storage for generating AND solving it.
 *
 * A perfect maze is a uniform spanning tree of the grid: every cell reachable,
 * exactly one route between any pair, no loops.  This struct holds that tree
 * (in cells[].walls) and the scratch each phase needs — the recursive
 * backtracker's explicit DFS stack, and the two-BFS diameter solver's queue
 * and parent links.  Refs: Buck [1][2] (backtracker), CLRS [4] (BFS/DFS),
 * two-BFS tree diameter [5].
 *
 * No separate phase field: the stage is implicit —
 *   sp > 0             → DFS still digging
 *   sp == 0 && !solved → dig done, ready to solve
 *   solved             → diameter found
 * (§5 Scene owns the explicit state machine layered over these.)
 */
typedef struct {
    /* grid geometry + the cells themselves */
    int   w, h;                  /* active size in cells (≤ MAZE_W/H_MAX)        */
    int   total_cells;           /* w*h, cached — bound for BFS and decay loops  */
    Cell  cells[CELLS_MAX];      /* row-major; flat index = y*w + x (maze_idx)   */

    /* DFS dig state — the backtracker frontier as an explicit stack.  Sized
     * CELLS_MAX because one long snaking corridor can stack every cell at once;
     * anything smaller silently corrupts state on overflow. */
    int   stack[CELLS_MAX];      /* cell indices on the current DFS path         */
    int   sp;                    /* stack depth; sp==0 ⇒ dig finished            */
    int   visited_count;         /* cells dug; == total_cells when the dig ends  */

    /* BFS scratch — reused for BOTH farthest-finds of the two-BFS diameter;
     * parent[] lets the 2nd pass rebuild the path by walking endpoint→endpoint. */
    int   bfs_queue [CELLS_MAX]; /* FIFO of cell indices to expand               */
    int   bfs_dist  [CELLS_MAX]; /* hops from the BFS source; -1 = not yet seen  */
    int   bfs_parent[CELLS_MAX]; /* predecessor on the BFS tree; -1 = root/none  */

    /* Solution path = the tree diameter (the longest of all shortest paths). */
    int   path[CELLS_MAX];       /* cell indices, path[0]..path[len-1] end→end   */
    int   path_len;              /* cells in the diameter (~Θ(√N) empirically)   */
    int   solve_progress;        /* next path index for the beam to light        */
    bool  solved;                /* diameter computed, path[] valid              */
} Maze;

static inline int maze_idx(const Maze *m, int x, int y) { return y * m->w + x; }
static inline bool maze_in_bounds(const Maze *m, int x, int y)
{
    return x >= 0 && x < m->w && y >= 0 && y < m->h;
}

/*
 * Scene state machine:
 *
 *   DIG     — DFS in progress; per tick run dig_steps_per_tick steps.
 *             When sp drops to 0, transition to SOLVE.
 *   SOLVE   — diameter computed; animate the solution beam by advancing
 *             solve_progress one cell at a time.
 *             When all cells lit, transition to DONE.
 *   DONE    — the finished maze with the whole diameter path lit; the run
 *             stays here (re-asserting the path glow each tick so it never
 *             fades) until the user presses r, which resets back to DIG.
 *
 * `paused` freezes everything.  There is no auto-reset: a finished maze
 * persists so it can be studied; r starts a fresh one.
 */
typedef enum { SCENE_DIG = 0, SCENE_SOLVE = 1, SCENE_DONE = 2 } SceneState;

/* Scene — the animated maze run: the maze itself plus the knobs and run-state
 * that drive its generate → solve → done lifecycle.  Reads as a contents page:
 *   WHAT      : m — the maze being built and solved.
 *   HOW       : dig/solve steps advanced per tick (the fast-forward throttles).
 *   run-state : which phase we are in, and whether the tick is frozen. */
typedef struct {
    Maze       m;                     /* WHAT: the maze being generated/solved   */
    int        dig_steps_per_tick;    /* HOW: DFS ops per tick (1..DIG_STEPS_MAX) */
    int        solve_steps_per_tick;  /* HOW: beam cells lit per tick            */
    SceneState state;                 /* run-state: DIG / SOLVE / DONE           */
    bool       paused;                /* run-state: freeze the tick              */
} Scene;

/* Screen — the terminal viewport: its current size in character cells.  The
 * receiver for terminal setup/resize and the renderers, so those take a Screen
 * (not the whole App) and stay decoupled from app-level state. */
typedef struct { int cols, rows; } Screen;

/* App — the whole program: the animated Scene plus the terminal and the
 * app-level selections/lifecycle around it.
 *   subsystems : the maze scene + the terminal viewport.
 *   selections : tick rate, colour theme, size preset (all user-driven).
 *   geometry   : maze dimensions derived from preset ∩ terminal fit.
 *   lifecycle  : signal-driven quit / resize flags. */
typedef struct {
    /* subsystems */
    Scene                 scene;
    Screen                screen;
    /* user selections (cycled/typed at runtime) */
    int                   sim_fps;        /* simulation ticks per second   */
    int                   theme;          /* index into THEMES             */
    int                   preset;         /* index into MAZE_PRESETS       */
    /* derived geometry */
    int                   maze_w, maze_h; /* cells, from preset ∩ terminal */
    /* lifecycle flags (set by signal handlers) */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* ===================================================================== */
/* §2  performance   (monotonic clock + sleep; frame cap lives in main)   */
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
/* §3  logic   (pure decisions: read-only, no I/O — cannot be corrupted)  */
/* ===================================================================== */

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

/*
 * cell_color_for — pick the (pair, attr) for the interior of a maze
 * cell based on which glow is dominant.
 *
 * Priority (highest wins):
 *   supernova_glow  → bright yellow flash
 *   head_glow       → near-white bold (only the current dig head)
 *   dig_glow        → magenta trail
 *   visited         → steel blue resting colour
 *   else            → blank
 *
 * NOTE: solution_glow is NOT handled here.  The diameter path is drawn as a
 * connected line by scene_draw_solution (interior nodes + passage connectors)
 * so it reads as one continuous route, not a row of dots.
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

/* True if screen cell (sx,sy) is on-screen — the bounds clamp the renderer
 * applies before every mvaddch. */
static bool in_screen(int sx, int sy, int cols, int rows)
{
    return sx >= 0 && sx < cols && sy >= 0 && sy < rows;
}

/* Directions from (x,y) into in-bounds, not-yet-visited neighbours.  Fills
 * dirs[0..count) and returns the count — the dig's available branch choices. */
static int unvisited_dirs(const Maze *m, int x, int y, int dirs[N_DIRS])
{
    int n = 0;
    for (int d = 0; d < N_DIRS; d++) {
        int nx = x + dir_dx(d);
        int ny = y + dir_dy(d);
        if (!maze_in_bounds(m, nx, ny)) continue;
        if (!m->cells[maze_idx(m, nx, ny)].visited) dirs[n++] = d;
    }
    return n;
}

/* ===================================================================== */
/* §4  simulation   (advances state: mutates the Maze)                    */
/* ===================================================================== */

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
 * maze_backtrack — pop the DFS stack one cell.  Re-brighten the trail at the
 * new top so the user sees the ant retracing; if the last cell is popped, sp
 * hits 0 and the dig is over.
 */
static void maze_backtrack(Maze *m)
{
    m->sp--;
    if (m->sp > 0) {
        int new_top = m->stack[m->sp - 1];
        m->cells[new_top].dig_glow  = 1.0f;
        m->cells[new_top].head_glow = 1.0f;
    }
}

/*
 * maze_advance — carve into neighbour d of (x,y), mark it visited+glowing,
 * and push it: one forward step of the depth-first walk.
 */
static void maze_advance(Maze *m, int x, int y, int d)
{
    int nidx = maze_idx(m, x + dir_dx(d), y + dir_dy(d));
    maze_carve(m, x, y, d);
    m->cells[nidx].visited   = true;
    m->cells[nidx].dig_glow  = 1.0f;
    m->cells[nidx].head_glow = 1.0f;
    m->visited_count++;
    m->stack[m->sp++] = nidx;
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
    if (m->sp <= 0) return false;             /* stack empty → dig finished */

    int top = m->stack[m->sp - 1];
    int x   = top % m->w;
    int y   = top / m->w;

    /* Only the new top should be the bright "ant"; dim the old head (we don't
     * zero it — the per-frame decay drains it on its own). */
    m->cells[top].head_glow *= 0.5f;

    int dirs[N_DIRS];
    int n = unvisited_dirs(m, x, y, dirs);
    if (n == 0) { maze_backtrack(m); return true; }   /* boxed in → retrace */

    maze_advance(m, x, y, dirs[rand() % n]);          /* carve into a random exit */
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
/* §5  scene   (the per-tick combine; reset/init are user-event helpers)  */
/* ===================================================================== */

static void scene_reset(Scene *s, int mw, int mh)
{
    maze_reset(&s->m, mw, mh);
    s->state = SCENE_DIG;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused              = false;
    s->dig_steps_per_tick  = DIG_STEPS_DEF;
    s->solve_steps_per_tick = SOLVE_STEPS_DEF;
    scene_reset(s, mw, mh);
}

/* EFFECTS: drain every cell's glow by exp(-rate·dt) — the per-tick fade.
 * exp(-rate·dt) is the standard RC-decay step; the O(N) loop is not hot. */
static void decay_glows(Maze *m, float dt)
{
    float dig_d  = expf(-DIG_GLOW_DECAY      * dt);
    float head_d = expf(-HEAD_GLOW_DECAY     * dt);
    float sol_d  = expf(-SOLUTION_GLOW_DECAY * dt);
    float nova_d = expf(-SUPERNOVA_DECAY     * dt);
    for (int i = 0; i < m->total_cells; i++) {
        m->cells[i].dig_glow       *= dig_d;
        m->cells[i].head_glow      *= head_d;
        m->cells[i].solution_glow  *= sol_d;
        m->cells[i].supernova_glow *= nova_d;
    }
}

/* DIG: run up to dig_steps_per_tick DFS operations; when the stack empties,
 * compute the diameter and advance to SOLVE. */
static void scene_dig(Scene *s)
{
    for (int i = 0; i < s->dig_steps_per_tick; i++)
        if (!maze_dig_step(&s->m)) break;
    if (s->m.sp == 0) {
        maze_compute_diameter(&s->m);
        s->state = SCENE_SOLVE;
    }
}

/* SOLVE: light up to solve_steps_per_tick more cells along the path (the
 * streaming gold beam); when the whole path is lit, advance to DONE. */
static void scene_solve(Scene *s)
{
    for (int i = 0; i < s->solve_steps_per_tick; i++) {
        if (s->m.solve_progress >= s->m.path_len) break;
        int idx = s->m.path[s->m.solve_progress++];
        s->m.cells[idx].solution_glow = 1.0f;
    }
    if (s->m.solve_progress >= s->m.path_len)
        s->state = SCENE_DONE;
}

/* DONE: re-light the whole diameter each tick (the decay would otherwise fade
 * it) so the finished maze holds its solution until the user presses r. */
static void scene_hold(Scene *s)
{
    for (int i = 0; i < s->m.path_len; i++)
        s->m.cells[s->m.path[i]].solution_glow = 1.0f;
}

/* The per-tick combine: fade the glows, then advance the active phase. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    decay_glows(&s->m, dt);                 /* EFFECTS */
    switch (s->state) {                      /* SIMULATION */
    case SCENE_DIG:   scene_dig(s);   break;
    case SCENE_SOLVE: scene_solve(s); break;
    case SCENE_DONE:  scene_hold(s);  break;
    }
}

/* ===================================================================== */
/* §6  render   (state → screen: reads Scene, mutates only the terminal)  */
/* ===================================================================== */

/* Apply a theme's palette.  HUD/HINT pairs are fixed (bright yellow/cyan)
 * so the HUD stays legible against every theme; the rest come from `th`. */
static void color_apply(const Theme *th)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* reserved */
    init_pair(PAIR_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* reserved */
    if (COLORS >= 256) {
        init_pair(PAIR_WALL,      th->wall,      -1);
        init_pair(PAIR_VISITED,   th->visited,   -1);
        init_pair(PAIR_TRAIL,     th->trail,     -1);
        init_pair(PAIR_HEAD,      th->head,      -1);
        init_pair(PAIR_SOLUTION,  th->solution,  -1);
        init_pair(PAIR_SUPERNOVA, th->supernova, -1);
    } else {
        init_pair(PAIR_WALL,      COLOR_WHITE,   -1);
        init_pair(PAIR_VISITED,   COLOR_BLUE,    -1);
        init_pair(PAIR_TRAIL,     COLOR_MAGENTA, -1);
        init_pair(PAIR_HEAD,      COLOR_WHITE,   -1);
        init_pair(PAIR_SOLUTION,  COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
}

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

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
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
 * scene_draw_solution — draw the diameter path as ONE connected gold line.
 *
 * Per-cell rendering only lights cell interiors, which sit two screen columns
 * apart, so the solution looked like detached dots.  Here we also fill the
 * carved passage BETWEEN consecutive path cells: a '+' node at each cell and
 * a '-' (horizontal move) or '|' (vertical move) connector across the gap.
 * The two diameter endpoints are marked 'S' (path start) and 'E' (path end).
 * The result is a continuous, easy-to-follow route.
 *
 * Only cells the beam has reached (solution_glow up) are drawn, so SOLVE still
 * streams; in DONE every path cell is re-lit each tick, so the whole path
 * shows solid until the user presses r.
 */
static void scene_draw_solution(const Scene *s, int gx0, int gy0,
                                int cols, int rows)
{
    const Maze *m = &s->m;
    attron(COLOR_PAIR(PAIR_SOLUTION) | A_BOLD);
    for (int i = 0; i < m->path_len; i++) {
        int idx = m->path[i];
        if (m->cells[idx].solution_glow <= GLOW_THRESHOLD) continue;
        int x = idx % m->w, y = idx / m->w;

        int sy = gy0 + 2 * y + 1;
        int sx = gx0 + 2 * x + 1;
        /* mark the diameter endpoints S / E; interior nodes get '+' */
        char node = (i == 0)                 ? 'S'
                  : (i == m->path_len - 1)   ? 'E'
                  :                            '+';
        if (sy >= 0 && sy < rows && sx >= 0 && sx < cols)
            mvaddch(sy, sx, (chtype)(unsigned char)node);

        /* connector across the passage to the previous path cell */
        if (i > 0) {
            int pidx = m->path[i - 1];
            if (m->cells[pidx].solution_glow > GLOW_THRESHOLD) {
                int px = pidx % m->w, py = pidx / m->w;
                int csy = gy0 + y + py + 1;   /* midpoint of the two interiors */
                int csx = gx0 + x + px + 1;
                char ch = (py == y) ? '-' : '|';
                if (csy >= 0 && csy < rows && csx >= 0 && csx < cols)
                    mvaddch(csy, csx, (chtype)(unsigned char)ch);
            }
        }
    }
    attroff(COLOR_PAIR(PAIR_SOLUTION) | A_BOLD);
}

/*
 * Maze → screen geometry.  A w×h maze renders as a (2w+1)×(2h+1) frame of
 * '+' / '-' / '|' chars; per cell (mx,my): interior at (2my+1, 2mx+1),
 * N-wall at (2my, 2mx+1), W-wall at (2my+1, 2mx), NW-corner at (2my, 2mx),
 * all offset by the frame origin (gy0, gx0).
 */

/* Top-left terminal cell of the centred maze frame, leaving row 0 for the data
 * HUD and the last row for the hint (hence the rows-2 and the +1). */
static void maze_screen_origin(const Maze *m, int cols, int rows,
                               int *gx0, int *gy0)
{
    int frame_w = 2 * m->w + 1;
    int frame_h = 2 * m->h + 1;
    *gx0 = (cols - frame_w) / 2;
    *gy0 = ((rows - 2) - frame_h) / 2 + 1;
    if (*gx0 < 0) *gx0 = 0;
    if (*gy0 < 1) *gy0 = 1;
}

/* Corner glyph at grid point (mx,my): '+' if any wall segment meets there. */
static void draw_corner_glyph(const Maze *m, int mx, int my,
                              int gx0, int gy0, int cols, int rows)
{
    int sy = gy0 + 2 * my, sx = gx0 + 2 * mx;
    if (!in_screen(sx, sy, cols, rows)) return;
    char ch = wall_corner_glyph[corner_walls_at(m, mx, my)];
    if (ch == ' ') return;
    attron(COLOR_PAIR(PAIR_WALL));
    mvaddch(sy, sx, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(PAIR_WALL));
}

/* North wall of cell (mx,my) — the '-' east of corner (mx,my).  At the top/
 * bottom frame edge it reads the boundary cell's N / S wall. */
static void draw_north_wall(const Maze *m, int mx, int my,
                            int gx0, int gy0, int cols, int rows)
{
    int sy = gy0 + 2 * my, sx = gx0 + 2 * mx + 1;
    if (!in_screen(sx, sy, cols, rows)) return;
    bool wall;
    if (my == 0)         wall = (m->cells[maze_idx(m, mx, 0)].walls & WALL_BIT(DIR_N)) != 0;
    else if (my == m->h) wall = (m->cells[maze_idx(m, mx, m->h-1)].walls & WALL_BIT(DIR_S)) != 0;
    else                 wall = (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_N)) != 0;
    if (!wall) return;
    attron(COLOR_PAIR(PAIR_WALL));
    mvaddch(sy, sx, (chtype)(unsigned char)'-');
    attroff(COLOR_PAIR(PAIR_WALL));
}

/* West wall of cell (mx,my) — the '|' south of corner (mx,my).  At the left/
 * right frame edge it reads the boundary cell's W / E wall. */
static void draw_west_wall(const Maze *m, int mx, int my,
                           int gx0, int gy0, int cols, int rows)
{
    int sy = gy0 + 2 * my + 1, sx = gx0 + 2 * mx;
    if (!in_screen(sx, sy, cols, rows)) return;
    bool wall;
    if (mx == 0)         wall = (m->cells[maze_idx(m, 0, my)].walls & WALL_BIT(DIR_W)) != 0;
    else if (mx == m->w) wall = (m->cells[maze_idx(m, m->w-1, my)].walls & WALL_BIT(DIR_E)) != 0;
    else                 wall = (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_W)) != 0;
    if (!wall) return;
    attron(COLOR_PAIR(PAIR_WALL));
    mvaddch(sy, sx, (chtype)(unsigned char)'|');
    attroff(COLOR_PAIR(PAIR_WALL));
}

/* Draw the wall lattice: at each grid point its corner, the north wall to its
 * east, and the west wall to its south.  The extra row/col (≤ m->h, ≤ m->w)
 * closes the outer frame. */
static void draw_walls(const Maze *m, int gx0, int gy0, int cols, int rows)
{
    for (int my = 0; my <= m->h; my++)
        for (int mx = 0; mx <= m->w; mx++) {
            draw_corner_glyph(m, mx, my, gx0, gy0, cols, rows);
            if (mx < m->w) draw_north_wall(m, mx, my, gx0, gy0, cols, rows);
            if (my < m->h) draw_west_wall (m, mx, my, gx0, gy0, cols, rows);
        }
}

/* Fill each cell interior with its dominant-glow colour (cell_color_for). */
static void draw_interiors(const Maze *m, int gx0, int gy0, int cols, int rows)
{
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

/* scene_draw — the maze in three passes: wall lattice, cell interiors, then
 * the connected gold solution line over the carved passages. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Maze *m = &s->m;
    int gx0, gy0;
    maze_screen_origin(m, cols, rows, &gx0, &gy0);
    draw_walls(m, gx0, gy0, cols, rows);
    draw_interiors(m, gx0, gy0, cols, rows);
    scene_draw_solution(s, gx0, gy0, cols, rows);
}

/* Draw one HUD row left-aligned at `row`, clipped to the terminal width so it
 * can never overflow onto the maze. */
static void draw_hud_row(const Screen *sc, int row, int pair, const char *text)
{
    char buf[256];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > sc->cols) buf[sc->cols] = '\0';
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void screen_draw(const Screen *sc, const Scene *s, double fps, int sim_fps,
                        const char *theme, const char *preset)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Maze *m = &s->m;
    const char *state_str =
        s->paused                  ? "PAUSED " :
        (s->state == SCENE_DIG)    ? "DIGGING" :
        (s->state == SCENE_SOLVE)  ? "SOLVING" :
                                     "SOLVED ";

    /* top row: data — title, theme, size preset, state, progress, rates */
    char data[256];
    snprintf(data, sizeof data,
             " Maze  %s  %s %dx%d  %s  %d/%d cells  steps:%d  %.1f fps  %d Hz ",
             theme, preset, m->w, m->h, state_str,
             m->visited_count, m->total_cells,
             s->dig_steps_per_tick, fps, sim_fps);
    draw_hud_row(sc, 0, PAIR_HUD, data);

    /* bottom row: actions — every interactive key */
    draw_hud_row(sc, sc->rows - 1, PAIR_HINT,
                 " q:quit  spc:pause  r:reset  1-0:size  t:theme  +/-:steps  [/]:Hz ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §7  app   (signals, user events, and the main loop)                    */
/* ===================================================================== */

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * app_pick_maze_size — set the maze size from the active preset, clamped to
 * what fits a (2w+1)×(2h+1) frame in the terminal (1 HUD row top + 1 hint
 * row bottom) and to MAZE_*_MAX.  A preset bigger than the terminal just
 * fills it; the "Max" preset always fills.
 */
static void app_pick_maze_size(App *app)
{
    int fit_w = (app->screen.cols - 1) / 2;   /* widest maze that fits     */
    int fit_h = (app->screen.rows - 3) / 2;   /* rows minus 2 HUD, then /2 */
    if (fit_w > MAZE_W_MAX) fit_w = MAZE_W_MAX;
    if (fit_h > MAZE_H_MAX) fit_h = MAZE_H_MAX;

    int mw = MAZE_PRESETS[app->preset].w;
    int mh = MAZE_PRESETS[app->preset].h;
    if (mw > fit_w) mw = fit_w;
    if (mh > fit_h) mh = fit_h;
    if (mw < 4) mw = 4;
    if (mh < 4) mh = 4;
    app->maze_w = mw;
    app->maze_h = mh;
}

/* USER EVENT: terminal resized — refit the maze and reseed (not a tick). */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_maze_size(app);
    scene_reset(&app->scene, app->maze_w, app->maze_h);
    app->need_resize = 0;
}

/* USER EVENT: dispatch one key press — mutates App/Scene, never a tick. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->maze_w, app->maze_h);
        break;
    case 't':
        app->theme = (app->theme + 1) % N_THEMES;
        color_apply(&THEMES[app->theme]);
        break;
    case 'T':
        app->theme = (app->theme - 1 + N_THEMES) % N_THEMES;
        color_apply(&THEMES[app->theme]);
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
    default:
        /* digit → maze-size preset: 1..9 pick 0..8, 0 picks the 10th */
        if (ch >= '0' && ch <= '9') {
            int idx = (ch == '0') ? 9 : (ch - '1');
            if (idx < N_MAZE_PRESETS) {
                app->preset = idx;
                app_pick_maze_size(app);
                scene_reset(&app->scene, app->maze_w, app->maze_h);
            }
        }
        break;
    }
    return true;
}

static void install_signals(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* One-time setup: seed the selections, bring up the terminal + theme, fit the
 * maze to the screen, and build the first maze. */
static void app_init(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    app->theme   = 0;
    app->preset  = N_MAZE_PRESETS - 1;   /* "Max" → fill the terminal by default */

    screen_init(&app->screen);
    color_apply(&THEMES[app->theme]);
    app_pick_maze_size(app);
    scene_init(&app->scene, app->maze_w, app->maze_h);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signals();

    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* USER EVENT: resize (handled outside the tick) */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* PER-TICK COMBINE: fixed-timestep accumulator drives scene_tick */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* FPS counter: refresh the displayed rate every FPS_UPDATE_MS */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* PERFORMANCE: cap rendered frames to RENDER_CAP_FPS */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        /* RENDER (reads state only) */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps,
                    THEMES[app->theme].name, MAZE_PRESETS[app->preset].name);
        screen_present();

        /* USER EVENT: keys */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
