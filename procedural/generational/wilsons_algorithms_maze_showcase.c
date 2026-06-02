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
 * Section map (layers — see ARCHITECTURE block below):
 *   §1 config       — grid, walk/glow/timing constants, direction utils, palette
 *   §2 performance  — monotonic timer + sleep (frame-cap helpers)
 *   §3 simulation   — Cell/Maze, Wilson walk + loop-erase + absorb, BFS diameter
 *   §4 simulation   — scene & the ONE per-tick combine (glow decay + ops)
 *   §5 render       — HUD, ASCII walls, walk arrows, glow ramps
 *   §6 app          — signals, resize, key events, fixed-timestep main loop
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
 * References     : Concept —
 *                  • Wilson, D. B. (1996) — "Generating random spanning trees
 *                    more quickly than the cover time" (the original paper):
 *                    https://www.cs.cmu.edu/~15859n/RelatedWork/RandomTrees-Wilson.pdf
 *                  • Buck, Jamis — "Mazes for Programmers" (2015): the
 *                    definitive maze-generation book — covers Wilson's, the
 *                    recursive-backtracker contrast, and solving by tree diameter.
 *                  • Buck, Jamis — "Maze Generation: Wilson's Algorithm" (blog
 *                    walkthrough with animations):
 *                    https://weblog.jamisbuck.org/2011/1/20/maze-generation-wilson-s-algorithm
 *                  • Loop-erased random walk — the core primitive:
 *                    https://en.wikipedia.org/wiki/Loop-erased_random_walk
 *                  • Tree diameter via two BFS (the solve phase):
 *                    https://cp-algorithms.com/graph/tree_painting.html#diameter-of-a-tree
 *                  Rendering —
 *                  • Bourke — "Colour Ramping for Data Visualisation": the
 *                    glow-magnitude → colour gradients (cyan comet trail,
 *                    pale→gold→dark solution beam, pink→magenta absorb front).
 *                  • Padala — "NCURSES Programming HOWTO", TLDP: colour pairs,
 *                    glyph output, non-blocking input, resize handling.
 *                  • xterm 256-colour palette — the ramp + accent indices the
 *                    renderer draws from: https://jonasjacek.github.io/colors/
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
 *    DFS's biased tree on the same grid — typically 1.5-2× longer. On an
 *    identical maze, expect a noticeably longer gold path than the
 *    backtracker produces; if it is much shorter, BFS exited early.
 *  • Wall symmetry (same as backtracker): for every interior cell with
 *    east wall = 0, the cell to its east must have west wall = 0.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * State is module-global (typed structs arrive in step 5).  Each layer owns
 * one concern; this table says what each mutates:
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   SIMULATION   §3,§4    the Maze — per-cell walls + in_maze/in_walk/walk_dir,
 *                         the walk_path / absorb_path queues, maze_count, the
 *                         BFS scratch + solution path — and the Scene run state.
 *   RENDER       §5       the terminal only — reads the Maze, never writes it;
 *                         screen_resize re-fits Screen dims (USER EVENT).
 *   APP          §6       run knobs (paused / *_steps_per_tick / sim_fps), maze
 *                         size, signal flags.
 *
 *   LOGIC is real but NOT a separate section — the pure decisions are small
 *     helpers kept beside what they serve: dir_dx / dir_dy / opposite /
 *     dir_arrow (§1), maze_idx / maze_in_bounds (§3), and cell_visual (§5, a
 *     pure state→glyph mapping).  None mutate or do I/O; hoisting these
 *     one-liners out would only hurt locality.  (maze_pick_random_outside is a
 *     read-only scan but draws rand() for its reservoir sample, so it sits in §3.)
 *   EFFECTS is real STATE, not functions: the six per-cell glow buffers
 *     (walk / head / erase / absorb / solution / supernova) plus on_path.
 *     Simulation ops WRITE them (a step paints walk_glow, a loop-erase paints
 *     erase_glow, …); scene_tick DECAYS them; render READS them.  They never
 *     feed back into the algorithm — deleting all glow code would change only
 *     how the maze LOOKS, not which maze is produced.
 *   No DELAYS layer — 'space' (paused) freezes scene_tick; the HOLD state
 *     (hold_timer) pauses on the finished maze before reset; the only real
 *     wait is the §2 frame cap.
 *
 * PER-TICK COMBINE — scene_tick (§4) is the ONE place sim state advances,
 * in fixed order:
 *     1. EFFECTS decay  — multiply every glow buffer by its exp(-rate·dt).
 *     2. SIMULATION     — by phase: WALKING runs walk steps (→ ABSORBING on a
 *                         maze hit); ABSORBING runs absorb steps (→ a new walk,
 *                         or SOLVING when the maze is full); SOLVING streams the
 *                         diameter beam (→ HOLD); HOLD counts down (→ reset).
 *   RENDER (scene_draw + HUD) then PERFORMANCE (frame cap) run once per frame in
 *   main, OUTSIDE the tick.
 *
 * USER EVENTS are NOT the tick: keys (pause / reset / speed / Hz) and resize
 * mutate state directly in §6 — reset and resize call scene_reset / maze_reset
 * — but only scene_tick advances the simulation.
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

    /* HUD reserves rows top + bottom; the maze is centred in between.
     * Top: row 0 = title + stats, row 1 = glyph legend.  Bottom: actions. */
    HUD_TOP_ROWS      =   2,
    HUD_BOT_ROWS      =   1,

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
    PAIR_WALK_MID     =  12,        /* walk trail — mid cyan (comet body) */
    PAIR_WALK_LO      =  13,        /* walk trail — dim cyan (comet tail) */
    PAIR_SOL_HI       =  14,        /* solution beam head — pale gold     */
    PAIR_SOL_LO       =  15,        /* solution path tail / resting path  */
    PAIR_ABSORB_HI    =  16,        /* absorb wave front — bright pink    */
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

/* Render frame cap: the wall-clock loop never spins faster than this,
 * independent of the (possibly higher) simulation tick rate. */
#define RENDER_CAP_FPS  60
/* Spiral-of-death guard: clamp a stalled frame's elapsed time so the
 * fixed-timestep accumulator never tries to catch up forever. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

#define CELLS_MAX  (MAZE_W_MAX * MAZE_H_MAX)

/* ===================================================================== */
/* §2  performance — monotonic clock + sleep (the frame-cap helpers)      */
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
/* §3  simulation — Cell/Maze, Wilson walk + loop-erase + absorb, BFS     */
/* ===================================================================== */

/*
 * Cell — one grid cell's persistent state.  In Wilson's algorithm a cell is
 * either already in the spanning tree, on the active loop-erased walk, or
 * untouched; this struct carries that status, the wall bitmask that IS the
 * maze, and the cosmetic glow layers.
 *
 * THE MAZE (the actual spanning tree the algorithm builds)
 *   walls    : 4-bit bitmask N=1 E=2 S=4 W=8; set = wall present.  Carving a
 *              passage clears the bit on BOTH adjacent cells (see maze_carve),
 *              so the wall graph stays symmetric for the BFS solver.
 *   in_maze  : true once permanently part of the tree ("the museum").
 *
 * THE WALK (Wilson's loop-erased random walk currently in progress)
 *   in_walk  : true while this cell is on the active walk_path.
 *   walk_dir : the direction the walker stepped FROM this cell to the next.
 *              This is HOW the loop-erased path is stored — following walk_dir
 *              from the walk start traces the (re-routed) path, and loop-erasure
 *              just overwrites walk_dir at the crossing cell.  Meaningful only
 *              while in_walk.
 *
 * EFFECTS (cosmetic only — never read by the algorithm; see ARCHITECTURE)
 *   walk_glow      : recency of the trail (cyan comet, fades over ~1 s).
 *   head_glow      : the live walker cell '@' (fast decay → one bright cell).
 *   erase_glow     : a cell just dropped by loop-erasure (red flash).
 *   absorb_glow    : a cell streaming into the museum (magenta wave).
 *   solution_glow  : the diameter beam passing through (gold gradient).
 *   supernova_glow : whole-grid yellow flash right after a reset.
 *   on_path        : stays true after the beam so the finished longest path
 *                    remains lit during HOLD (the resting-diameter payoff).
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
    bool    on_path;        /* true once the diameter beam has claimed this
                             * cell — keeps the longest path lit after the
                             * bright beam glow fades (the HOLD payoff). */
} Cell;

/*
 * Maze — the whole generation-then-solve state for one run.  Two algorithms
 * share it: Wilson's uniform-spanning-tree generation (Wilson 1996) followed by
 * the two-BFS tree-diameter solve.  The active phase is implicit in the counts
 * (the explicit state machine lives on Scene, §4):
 *   maze_count < total_cells, walk_len > 0  → walking
 *   absorb_len  > 0                         → absorbing
 *   maze_count == total_cells               → ready to solve
 *
 * Field groups:
 *   THE GRID       — w, h, total_cells, cells[]: the maze itself.
 *   THE WALK       — walk_path[] is the loop-erased walk as cell indices in
 *                    order; walk_len is its length (0 = no walk); walk_pos is
 *                    the walker, == walk_path[walk_len-1].  Loop-erasure
 *                    truncates walk_len back to (revisit index + 1).
 *   ABSORB (anim)  — absorb_path[]/_len/_progress: the just-finished walk queued
 *                    to stream into the museum one cell per tick (the magenta
 *                    wave).  Algorithmically this could be a single step; it is
 *                    paced cell-by-cell only for the visual.
 *   DIAMETER (BFS) — bfs_queue/dist/parent[]: scratch for the two BFS passes
 *                    that find the tree's longest path (cp-algorithms two-BFS).
 *   SOLUTION       — path[]/path_len: the diameter cells; solve_progress is the
 *                    beam's position streaming along them.
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
        m->cells[i].on_path        = false;
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
 * random_inbounds_dir — pick a uniform random direction whose target cell is
 * in bounds.  Rejection retry; every cell has ≥2 in-bounds neighbours so this
 * terminates quickly.  Pure read (consults only bounds; draws rand()).
 */
static int random_inbounds_dir(const Maze *m, int x, int y)
{
    int d;
    do {
        d = rand() % N_DIRS;
    } while (!maze_in_bounds(m, x + dir_dx(d), y + dir_dy(d)));
    return d;
}

/*
 * walk_queue_absorb — CASE A: the walk reached the museum at `dest`.  Copy the
 * loop-erased walk + dest into absorb_path[] and end the walk; absorb_step then
 * streams it in.  (in_walk stays set so the trail keeps showing until the
 * magenta wave eats it cell-by-cell.)
 */
static void walk_queue_absorb(Maze *m, int dest)
{
    m->absorb_len = 0;
    for (int i = 0; i < m->walk_len; i++)
        m->absorb_path[m->absorb_len++] = m->walk_path[i];
    m->absorb_path[m->absorb_len++] = dest;
    m->absorb_progress = 0;
    m->walk_len = 0;        /* walking is over */
}

/*
 * walk_erase_loop — CASE B: the walker stepped back onto its own trail at
 * `nidx`.  Find nidx in walk_path[], orphan the cells after it (clear in_walk +
 * red erase flash) and truncate the path back to it — loop-erasure, the trick
 * that makes the result a UNIFORM spanning tree.
 */
static void walk_erase_loop(Maze *m, int nidx)
{
    int hit = -1;
    for (int i = 0; i < m->walk_len; i++)
        if (m->walk_path[i] == nidx) { hit = i; break; }

    for (int i = hit + 1; i < m->walk_len; i++) {
        int c = m->walk_path[i];
        m->cells[c].in_walk     = false;
        m->cells[c].erase_glow  = 1.0f;
        m->cells[c].walk_glow  *= 0.3f;   /* dim cyan immediately */
    }
    m->walk_len = hit + 1;
}

/* walk_extend — CASE C: step into fresh territory; append nidx to the walk. */
static void walk_extend(Maze *m, int nidx)
{
    m->walk_path[m->walk_len++] = nidx;
    m->walk_pos                 = nidx;
    m->cells[nidx].in_walk      = true;
    m->cells[nidx].walk_glow    = 1.0f;
    m->cells[nidx].head_glow    = 1.0f;
}

/*
 * maze_walk_step — perform one walk operation.  Returns 1 (stepped), 2 (walk
 * hit the maze → absorb queued, walk ended), or 0 (no active walk).
 */
static int maze_walk_step(Maze *m)
{
    if (m->walk_len == 0) return 0;

    int x = m->walk_pos % m->w;          /* unpack walker cell index */
    int y = m->walk_pos / m->w;

    int d    = random_inbounds_dir(m, x, y);
    int nidx = maze_idx(m, x + dir_dx(d), y + dir_dy(d));

    /* Record the step at the source — this is what re-routes the path past a
     * loop on the erase case — and damp its halo so only the new head is bright. */
    m->cells[m->walk_pos].walk_dir   = d;
    m->cells[m->walk_pos].head_glow *= 0.4f;

    if (m->cells[nidx].in_maze) {        /* CASE A — reached the museum */
        walk_queue_absorb(m, nidx);
        return 2;
    }
    if (m->cells[nidx].in_walk) {        /* CASE B — stepped on own trail */
        walk_erase_loop(m, nidx);
        m->walk_pos = nidx;
        m->cells[nidx].head_glow = 1.0f;
        m->cells[nidx].walk_glow = 1.0f;
        return 1;
    }
    walk_extend(m, nidx);                /* CASE C — fresh territory */
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
}

/* ===================================================================== */
/* §4  simulation — scene & the ONE per-tick combine (glow decay + ops)   */
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

/*
 * Scene — the running showcase, read top-to-bottom as a table of contents.
 * Grouped by concept, not by which key changes them:
 *   WHAT  — m: the Maze being generated then solved (the simulation proper).
 *   PHASE — state + hold_timer: where we are in WALKING→ABSORBING→SOLVING→HOLD
 *           and the HOLD countdown; paused freezes the whole tick.
 *   HOW   — the three per-tick pacing knobs: walk_steps (the '+'/'-' speed),
 *           absorb_steps (magenta-wave pace), solve_steps (beam pace).
 * No render fields live here — colour/theme is fixed, so RENDER owns no Scene
 * state (a theme knob would belong here, grouped apart from the sim knobs).
 */
typedef struct {
    Maze        m;                       /* WHAT:  maze + in-progress walk     */
    SceneState  state;                   /* PHASE: WALKING/ABSORBING/SOLVING/HOLD */
    float       hold_timer;              /* PHASE: seconds left in HOLD        */
    bool        paused;                  /* PHASE: freeze the tick             */
    int         walk_steps_per_tick;     /* HOW:   walk steps per tick (+/-)   */
    int         absorb_steps_per_tick;   /* HOW:   absorb-wave pace            */
    int         solve_steps_per_tick;    /* HOW:   solution-beam pace          */
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

/* EFFECTS: fade every per-cell glow buffer toward 0 this tick.  expf(-rate*dt)
 * is the textbook RC-decay; computed inline since dt is small, the loop O(N). */
static void decay_glows(Maze *m, float dt)
{
    float walk_d   = expf(-WALK_GLOW_DECAY     * dt);
    float head_d   = expf(-HEAD_GLOW_DECAY     * dt);
    float erase_d  = expf(-ERASE_GLOW_DECAY    * dt);
    float absorb_d = expf(-ABSORB_GLOW_DECAY   * dt);
    float sol_d    = expf(-SOLUTION_GLOW_DECAY * dt);
    float nova_d   = expf(-SUPERNOVA_DECAY     * dt);
    int n = m->total_cells;
    for (int i = 0; i < n; i++) {
        m->cells[i].walk_glow      *= walk_d;
        m->cells[i].head_glow      *= head_d;
        m->cells[i].erase_glow     *= erase_d;
        m->cells[i].absorb_glow    *= absorb_d;
        m->cells[i].solution_glow  *= sol_d;
        m->cells[i].supernova_glow *= nova_d;
    }
}

/* WALKING: run up to walk_steps_per_tick walk steps; a maze hit → ABSORBING. */
static void advance_walking(Scene *s)
{
    for (int i = 0; i < s->walk_steps_per_tick; i++) {
        int rc = maze_walk_step(&s->m);
        if (rc == 2) { s->state = SCENE_ABSORBING; break; }
        if (rc == 0) break;
    }
}

/* ABSORBING: stream the walk into the maze; when the queue empties, start a
 * new walk (cells remain) or compute the diameter and switch to SOLVING. */
static void advance_absorbing(Scene *s)
{
    for (int i = 0; i < s->absorb_steps_per_tick; i++) {
        if (maze_absorb_step(&s->m)) continue;   /* still absorbing */

        if (s->m.maze_count >= s->m.total_cells) {
            maze_compute_diameter(&s->m);
            s->state = SCENE_SOLVING;
        } else {
            int start = maze_pick_random_outside(&s->m);
            if (start >= 0) {
                maze_start_walk(&s->m, start);
                s->state = SCENE_WALKING;
            } else {
                /* Shouldn't happen — maze_count < total but no outside cell.
                 * Be defensive: solve. */
                maze_compute_diameter(&s->m);
                s->state = SCENE_SOLVING;
            }
        }
        break;
    }
}

/* SOLVING: stream the gold diameter beam (claims cells for on_path); when the
 * whole path is lit → HOLD. */
static void advance_solving(Scene *s)
{
    for (int i = 0; i < s->solve_steps_per_tick; i++) {
        if (s->m.solve_progress >= s->m.path_len) break;
        int idx = s->m.path[s->m.solve_progress++];
        s->m.cells[idx].solution_glow = 1.0f;
        s->m.cells[idx].on_path       = true;   /* stays lit after the beam */
    }
    if (s->m.solve_progress >= s->m.path_len) {
        s->state      = SCENE_HOLD;
        s->hold_timer = HOLD_SECONDS;
    }
}

/* HOLD: admire the finished maze + highlighted diameter, then reset. */
static void advance_hold(Scene *s, float dt)
{
    s->hold_timer -= dt;
    if (s->hold_timer <= 0.0f)
        scene_reset(s, s->m.w, s->m.h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    decay_glows(&s->m, dt);              /* EFFECTS: fade all glows */

    switch (s->state) {                  /* advance the current phase */
    case SCENE_WALKING:   advance_walking(s);    break;
    case SCENE_ABSORBING: advance_absorbing(s);  break;
    case SCENE_SOLVING:   advance_solving(s);    break;
    case SCENE_HOLD:      advance_hold(s, dt);   break;
    }
}

/* ===================================================================== */
/* §5  render — HUD, ASCII walls, walk arrows, glow ramps                 */
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
        init_pair(PAIR_WALK,        51, -1);   /* bright cyan (trail head)*/
        init_pair(PAIR_HEAD,       231, -1);   /* near-white walker      */
        init_pair(PAIR_ERASE,      196, -1);   /* hot red (loop flash)   */
        init_pair(PAIR_ABSORB,     201, -1);   /* magenta absorb wave    */
        init_pair(PAIR_SOLUTION,   220, -1);   /* gold beam              */
        init_pair(PAIR_SUPERNOVA,  226, -1);   /* yellow reset flash     */
        init_pair(PAIR_SOURCE,     214, -1);   /* orange-gold for 'S'    */
        init_pair(PAIR_WALK_MID,    39, -1);   /* mid cyan  (trail body)  */
        init_pair(PAIR_WALK_LO,     31, -1);   /* dim cyan  (trail tail)  */
        init_pair(PAIR_SOL_HI,     229, -1);   /* pale gold (beam head)   */
        init_pair(PAIR_SOL_LO,     178, -1);   /* dark gold (resting path)*/
        init_pair(PAIR_ABSORB_HI,  213, -1);   /* bright pink (wave front)*/
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
        init_pair(PAIR_WALK_MID,  COLOR_CYAN,    -1);
        init_pair(PAIR_WALK_LO,   COLOR_CYAN,    -1);
        init_pair(PAIR_SOL_HI,    COLOR_WHITE,   -1);
        init_pair(PAIR_SOL_LO,    COLOR_YELLOW,  -1);
        init_pair(PAIR_ABSORB_HI, COLOR_MAGENTA, -1);
    }
}

/*
 * Screen — the terminal viewport: its current size in cells.  A render-target
 * concept kept as its own narrow type so the §5 functions take Screen* alone
 * and never reach for the whole App (keeps render decoupled from app/sim).
 *
 * Per-frame pattern (maze_backtracker.c §7 / framework.c §7):
 *   erase → scene_draw → HUD → wnoutrefresh(stdscr) → doupdate.
 * ASCII glyphs only: '+' wall corner, '-'/'|' wall segments, '@' walker,
 * '^>v<' walk arrows, '!' loop-erase, '*' wave / beam / supernova, '.' resting
 * maze cell.
 *
 *   cols, rows : terminal dimensions, refreshed on resize.
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
 *   absorb_glow      → pink→magenta '*' (absorb wave, gradient by glow)
 *   solution_glow    → gold beam '*' (gradient: pale→gold→dark by glow)
 *   is_source        → orange-gold 'S' (where THIS walk started)
 *   in_walk          → cyan walk arrow '^>v<' (comet gradient by walk_glow)
 *   on_path          → steady dark-gold '*' (the resting longest path)
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
        /* Magenta wave front (bright pink) fading to magenta as it absorbs. */
        *pair  = (c->absorb_glow > 0.6f) ? PAIR_ABSORB_HI : PAIR_ABSORB;
        *attr  = A_BOLD;
        *glyph = '*';
        return true;
    }
    if (c->solution_glow > GLOW_THRESHOLD) {
        /* Gold beam streams along the diameter as a colour gradient: a pale-
         * gold white-hot head, gold body, dark-gold tail behind it. */
        *pair  = (c->solution_glow > 0.6f)  ? PAIR_SOL_HI
               : (c->solution_glow > 0.25f) ? PAIR_SOLUTION : PAIR_SOL_LO;
        *attr  = (c->solution_glow > 0.25f) ? A_BOLD : A_NORMAL;
        *glyph = '*';
        return true;
    }
    if (is_source && c->in_walk) {
        *pair = PAIR_SOURCE; *attr = A_BOLD; *glyph = 'S'; return true;
    }
    if (c->in_walk) {
        /* Comet tail as a cyan gradient: bright-cyan head, mid-cyan body,
         * dim-cyan tail — the walk reads as a moving head with a fading wake. */
        if (c->walk_glow > 0.55f)      { *pair = PAIR_WALK;     *attr = A_BOLD; }
        else if (c->walk_glow > 0.20f) { *pair = PAIR_WALK_MID; *attr = A_BOLD; }
        else                           { *pair = PAIR_WALK_LO;  *attr = A_NORMAL; }
        *glyph = dir_arrow(c->walk_dir);
        return true;
    }
    if (c->on_path) {
        /* The finished longest path stays lit in steady dark-gold beneath the
         * bright streaming beam — so the HOLD frame shows the maze's diameter. */
        *pair = PAIR_SOL_LO; *attr = A_NORMAL; *glyph = '*'; return true;
    }
    if (c->in_maze) {
        *pair = PAIR_VISITED; *attr = A_DIM; *glyph = '.'; return true;
    }
    return false;
}

/* maze_screen_origin — top-left screen cell of the (2w+1)×(2h+1) maze frame,
 * centred in the terminal between the HUD rows. */
static void maze_screen_origin(const Maze *m, int cols, int rows, int *gx0, int *gy0)
{
    int frame_w = 2 * m->w + 1;
    int frame_h = 2 * m->h + 1;
    int x0 = (cols - frame_w) / 2;
    int y0 = ((rows - HUD_TOP_ROWS - HUD_BOT_ROWS) - frame_h) / 2 + HUD_TOP_ROWS;
    if (x0 < 0)            x0 = 0;
    if (y0 < HUD_TOP_ROWS) y0 = HUD_TOP_ROWS;
    *gx0 = x0;
    *gy0 = y0;
}

/* corner_has_wall — is any wall segment incident to corner (mx,my)?  A '+' is
 * drawn there iff so.  (The OR over the eight neighbour edges — see
 * maze_backtracker.c for the LUT derivation; collapsed to '+' for ASCII.) */
static bool corner_has_wall(const Maze *m, int mx, int my)
{
    if (maze_in_bounds(m, mx,   my-1) && (m->cells[maze_idx(m, mx,   my-1)].walls & WALL_BIT(DIR_W))) return true;
    if (maze_in_bounds(m, mx-1, my-1) && (m->cells[maze_idx(m, mx-1, my-1)].walls & WALL_BIT(DIR_E))) return true;
    if (maze_in_bounds(m, mx,   my)   && (m->cells[maze_idx(m, mx,   my)].walls   & WALL_BIT(DIR_N))) return true;
    if (maze_in_bounds(m, mx,   my-1) && (m->cells[maze_idx(m, mx,   my-1)].walls & WALL_BIT(DIR_S))) return true;
    if (maze_in_bounds(m, mx,   my)   && (m->cells[maze_idx(m, mx,   my)].walls   & WALL_BIT(DIR_W))) return true;
    if (maze_in_bounds(m, mx-1, my)   && (m->cells[maze_idx(m, mx-1, my)].walls   & WALL_BIT(DIR_E))) return true;
    if (maze_in_bounds(m, mx-1, my)   && (m->cells[maze_idx(m, mx-1, my)].walls   & WALL_BIT(DIR_N))) return true;
    if (maze_in_bounds(m, mx-1, my-1) && (m->cells[maze_idx(m, mx-1, my-1)].walls & WALL_BIT(DIR_S))) return true;
    return false;
}

/* h_wall_at — horizontal wall on the north edge of cell-column mx at maze-row
 * my?  Top/bottom borders read the outermost cell's N/S wall. */
static bool h_wall_at(const Maze *m, int mx, int my)
{
    if (my == 0)    return (m->cells[maze_idx(m, mx, 0)].walls      & WALL_BIT(DIR_N)) != 0;
    if (my == m->h) return (m->cells[maze_idx(m, mx, m->h-1)].walls & WALL_BIT(DIR_S)) != 0;
    return (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_N)) != 0;
}

/* v_wall_at — vertical wall on the west edge of cell-row my at maze-column mx?
 * Left/right borders read the outermost cell's W/E wall. */
static bool v_wall_at(const Maze *m, int mx, int my)
{
    if (mx == 0)    return (m->cells[maze_idx(m, 0, my)].walls      & WALL_BIT(DIR_W)) != 0;
    if (mx == m->w) return (m->cells[maze_idx(m, m->w-1, my)].walls & WALL_BIT(DIR_E)) != 0;
    return (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_W)) != 0;
}

/* scene_draw_walls — the maze frame: a '+' at each walled corner, '-' / '|'
 * along the existing wall segments, all clipped to the screen. */
static void scene_draw_walls(const Maze *m, int gx0, int gy0, int cols, int rows)
{
    for (int my = 0; my <= m->h; my++) {
        for (int mx = 0; mx <= m->w; mx++) {
            int sy_corner = gy0 + 2 * my;
            int sx_corner = gx0 + 2 * mx;

            if (sy_corner >= 0 && sy_corner < rows && sx_corner >= 0 && sx_corner < cols
                && corner_has_wall(m, mx, my)) {
                attron(COLOR_PAIR(PAIR_WALL));
                mvaddch(sy_corner, sx_corner, (chtype)(unsigned char)'+');
                attroff(COLOR_PAIR(PAIR_WALL));
            }

            if (mx < m->w) {                     /* north edge of cell (mx,my) */
                int sx = gx0 + 2 * mx + 1;
                if (sy_corner >= 0 && sy_corner < rows && sx >= 0 && sx < cols
                    && h_wall_at(m, mx, my)) {
                    attron(COLOR_PAIR(PAIR_WALL));
                    mvaddch(sy_corner, sx, (chtype)(unsigned char)'-');
                    attroff(COLOR_PAIR(PAIR_WALL));
                }
            }

            if (my < m->h) {                     /* west edge of cell (mx,my) */
                int sy = gy0 + 2 * my + 1;
                if (sy >= 0 && sy < rows && sx_corner >= 0 && sx_corner < cols
                    && v_wall_at(m, mx, my)) {
                    attron(COLOR_PAIR(PAIR_WALL));
                    mvaddch(sy, sx_corner, (chtype)(unsigned char)'|');
                    attroff(COLOR_PAIR(PAIR_WALL));
                }
            }
        }
    }
}

/* scene_draw_interiors — the cell glyphs (glows / walk arrows / maze dots) via
 * cell_visual; the single source cell ('S') is flagged from walk_path[0]. */
static void scene_draw_interiors(const Scene *s, int gx0, int gy0, int cols, int rows)
{
    const Maze *m = &s->m;
    int source_idx = (m->walk_len > 0) ? m->walk_path[0] : -1;
    for (int my = 0; my < m->h; my++) {
        int sy = gy0 + 2 * my + 1;
        if (sy < HUD_TOP_ROWS || sy >= rows - HUD_BOT_ROWS) continue;
        for (int mx = 0; mx < m->w; mx++) {
            int sx = gx0 + 2 * mx + 1;
            if (sx < 0 || sx >= cols) continue;

            int idx = maze_idx(m, mx, my);
            int pair, attr; char glyph;
            if (!cell_visual(&m->cells[idx], idx == source_idx, &pair, &attr, &glyph))
                continue;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* scene_draw — centre the maze, draw its walls, then its cell interiors. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    maze_screen_origin(&s->m, cols, rows, &gx0, &gy0);
    scene_draw_walls(&s->m, gx0, gy0, cols, rows);
    scene_draw_interiors(s, gx0, gy0, cols, rows);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Maze *m = &s->m;
    int  cols = sc->cols;
    char buf[256];

    const char *state_str =
        s->paused                     ? "PAUSED " :
        (s->state == SCENE_WALKING)   ? "WALKING" :
        (s->state == SCENE_ABSORBING) ? "ABSORB " :
        (s->state == SCENE_SOLVING)   ? "SOLVING" :
                                        "HOLD   ";

    /* Row 0 — title (left) + stats (right): the data line. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " WILSON'S MAZE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  walk:%d/tick  %s  cells:%d/%d ",
             fps, sim_fps, s->walk_steps_per_tick, state_str,
             m->maze_count, m->total_cells);
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — glyph legend: secondary data (no bold, row 0 stays dominant). */
    snprintf(buf, sizeof buf,
             " S:source  @:walker  ^>v<:trail  !:loop-erase  *:wave/beam ");
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom row — actions: every interactive key. */
    snprintf(buf, sizeof buf,
             " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  app — signals, key/resize events, fixed-timestep main loop         */
/* ===================================================================== */

/*
 * App — the running program: the Scene being animated, the terminal it draws
 * to, and the loop/lifecycle state that is neither simulation nor render.
 * Kept distinct from Scene so the render layer can take Screen* alone, and so
 * the signal handler can reach the run flags via the single g_app instance.
 *   DOMAIN    — scene + screen: the simulation and its draw target.
 *   PACING    — sim_fps: fixed-timestep rate (the [ / ] knob).
 *   GEOMETRY  — maze_w/maze_h: chosen maze size, re-fit to the terminal on
 *               resize (clamped to MAZE_*_MAX so the cell arrays can't overflow).
 *   LIFECYCLE — running/need_resize: signal-written flags (sig_atomic_t).
 */
typedef struct {
    Scene                 scene;        /* DOMAIN:    the simulation       */
    Screen                screen;       /* DOMAIN:    its draw target      */
    int                   sim_fps;      /* PACING:    fixed-timestep rate  */
    int                   maze_w, maze_h; /* GEOMETRY: chosen maze size    */
    volatile sig_atomic_t running;      /* LIFECYCLE: clear to exit        */
    volatile sig_atomic_t need_resize;  /* LIFECYCLE: set by SIGWINCH      */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_maze_size(App *app)
{
    int avail_w = app->screen.cols;
    int avail_h = app->screen.rows - HUD_TOP_ROWS - HUD_BOT_ROWS;
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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* spiral-of-death guard */

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
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
