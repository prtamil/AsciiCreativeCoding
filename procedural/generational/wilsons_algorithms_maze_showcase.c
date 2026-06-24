/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wilsons_algorithms_maze_showcase.c — an animated maze built by Wilson's
 *   algorithm: a random walker wanders the grid, erases its own loops, and
 *   slowly stitches a perfectly fair maze together.
 *
 * Sister file: ./maze_backtracker.c builds a maze the faster depth-first
 *   way. Wilson's is slower but fair — every possible maze is equally
 *   likely. On screen: backtracker grows from one centre, Wilson's grows
 *   from many random fingers.
 *
 * Why this maze is special: a plain random walk would favour long winding
 *   corridors. Wilson's trick is to erase any loop the walker makes, which
 *   (provably) makes every maze equally likely — see Wilson (1996),
 *   "Generating random spanning trees more quickly than the cover time",
 *   and Jamis Buck's "Mazes for Programmers" (2015). The loop-erasing idea:
 *   https://en.wikipedia.org/wiki/Loop-erased_random_walk
 *
 * What you watch happen:
 *   • A white '@' walker starts outside the maze and wanders at random,
 *     dropping arrow tracks '^>v<' that point the way it stepped.
 *   • Cross its own track and the loop FLASHES RED and vanishes — that's
 *     the loop-erasing that keeps the maze fair.
 *   • Reach the maze and the whole track streams in as a magenta wave,
 *     carving walls cell by cell. Then a new walker starts somewhere else.
 *   • Once every cell joins, a gold beam lights the longest path through
 *     the maze (its "diameter"). Hold, flash, restart forever.
 *
 * Section map:
 *   §1 config       — grid size, glow/timing constants, direction helpers
 *   §2 performance  — clock + sleep (the frame-cap helpers)
 *   §3 simulation   — Cell/Maze, the walk + loop-erase + absorb, BFS solve
 *   §4 simulation   — the scene state machine and the one per-tick update
 *   §5 render       — HUD, ASCII walls, walk arrows, glow colours
 *   §6 app          — signals, resize, keys, main loop
 */

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

/* ── §1 config ── */

enum {
    /* Cap the maze size. Wilson's gets slow and visually busy on big
     * grids, so ~1200 cells keeps each walk easy to follow by eye. */
    MAZE_W_MAX        =  60,
    MAZE_H_MAX        =  20,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* How many walk moves to make each tick. The default is slow enough
     * to follow each random move by eye; press '+' to speed it up once
     * you've seen how it works. */
    WALK_STEPS_MIN    =   1,
    WALK_STEPS_DEF    =   4,
    WALK_STEPS_MAX    = 2048,

    /* How many cells the magenta wave eats per tick as the finished walk
     * streams into the maze. Higher = snappier, lower = more wavy. */
    ABSORB_STEPS_DEF  =   3,

    /* How many cells the gold solution beam lights per tick. */
    SOLVE_STEPS_DEF   =   1,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* The HUD eats rows at the top and bottom; the maze sits centred in
     * between. Top: row 0 = title + stats, row 1 = legend. Bottom: keys. */
    HUD_TOP_ROWS      =   2,
    HUD_BOT_ROWS      =   1,

    /* Colour pair slots. PAIR_HUD/PAIR_HINT are reserved by the project. */
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

/* How fast each glow fades. Bigger number = quicker fade. The flashes
 * (erase, supernova) fade fast; the trail and wave linger so you can read
 * them. */
#define WALK_GLOW_DECAY     1.5f    /* arrow trail             */
#define HEAD_GLOW_DECAY     8.0f    /* the walker cell itself  */
#define ERASE_GLOW_DECAY    3.0f    /* red loop-erase flash    */
#define ABSORB_GLOW_DECAY   2.0f    /* magenta absorb wave     */
#define SOLUTION_GLOW_DECAY 1.5f    /* gold solution beam      */
#define SUPERNOVA_DECAY     4.0f    /* reset flash (clears fast so the seed shows) */
#define GLOW_THRESHOLD      0.05f   /* below this a glow counts as gone */

#define HOLD_SECONDS        2.0f

/* The four directions, numbered the same way across the procedural files. */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };
static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

/* One bit per wall. Each cell starts with all four. */
#define WALL_BIT(d)   (1u << (d))
#define ALL_WALLS     0x0Fu

/* The arrow shown for a trail cell, pointing the way the walker stepped. */
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

/* Don't redraw the screen faster than this, even if the simulation ticks
 * faster. Keeps the frame rate steady. */
#define RENDER_CAP_FPS  60
/* If one frame stalls (laptop slept, terminal hung), cap how much time we
 * believe passed — otherwise the sim tries to catch up forever and freezes. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

#define CELLS_MAX  (MAZE_W_MAX * MAZE_H_MAX)

/* ── §2 performance — clock + sleep (the frame-cap helpers) ── */

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

/* ── §3 simulation — Cell/Maze, the walk + loop-erase + absorb, BFS solve ── */

/*
 * Cell — everything one grid square remembers. At any moment a square is
 * already in the finished maze, on the walker's current trail, or untouched.
 * It holds that status, the walls that make up the maze, and a set of glow
 * values that drive the on-screen colour effects.
 *
 * THE MAZE (the finished part the algorithm is building)
 *   walls    : which of the four walls are still standing (one bit each).
 *              Knocking down a wall clears the matching bit on BOTH squares
 *              that share it (see maze_carve), so the two always agree — the
 *              solver later trusts that.
 *   in_maze  : true once this square is permanently part of the maze.
 *
 * THE WALK (the walker's current trail, before loop-erasing)
 *   in_walk  : true while this square is on the live trail.
 *   walk_dir : which way the walker stepped FROM this square to the next one.
 *              This is how the trail is stored: follow walk_dir from the start
 *              and you trace the path. When the walker erases a loop it just
 *              overwrites walk_dir at the crossing square, re-routing the path.
 *              Only meaningful while in_walk.
 *
 * THE GLOWS (looks only — the algorithm never reads these)
 *   walk_glow      : how fresh this trail square is (cyan, fades over ~1 s).
 *   head_glow      : the walker square right now '@' (fades fast → one bright dot).
 *   erase_glow     : a square just dropped by a loop-erase (red flash).
 *   absorb_glow    : a square streaming into the maze (magenta wave).
 *   solution_glow  : the gold beam passing through this square.
 *   supernova_glow : the brief whole-grid yellow flash after a reset.
 *   on_path        : true once the gold beam has claimed this square; stays
 *                    on so the longest path is still lit during the HOLD pause.
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
    bool    on_path;        /* keeps the longest path lit during HOLD, after
                             * the bright beam glow has faded */
} Cell;

/*
 * Maze — everything one full run needs: first to build the maze (Wilson's
 * walk), then to find and light its longest path. Which phase we're in is
 * tracked explicitly on Scene (§4), but you can also read it off the counts:
 *   building, walker out wandering  → maze_count < total_cells, walk_len > 0
 *   streaming a finished walk in     → absorb_len > 0
 *   maze done, ready to solve        → maze_count == total_cells
 *
 * Field groups:
 *   THE GRID    — w, h, total_cells, cells[]: the maze itself.
 *   THE WALK    — walk_path[] lists the trail squares in order; walk_len is
 *                 how many (0 = no walk); walk_pos is where the walker is now
 *                 (the last entry). A loop-erase chops walk_len back to just
 *                 past the square the walker bumped into.
 *   THE WAVE    — absorb_path[]/_len/_progress: a finished trail waiting to
 *                 stream into the maze one square per tick. It could be added
 *                 all at once; we pace it just for the magenta wave look.
 *   SOLVE SCRATCH — bfs_queue/dist/parent[]: working room for the two
 *                 breadth-first passes that find the longest path through the
 *                 maze (the standard two-BFS diameter trick).
 *   THE BEAM    — path[]/path_len: the longest-path squares; solve_progress is
 *                 how far the gold beam has streamed along them.
 */
typedef struct {
    int   w, h;
    int   total_cells;
    Cell  cells[CELLS_MAX];

    /* the walker's current trail */
    int   walk_path[CELLS_MAX];   /* trail squares, in the order walked */
    int   walk_len;               /* 0 = no walk in progress */
    int   walk_pos;               /* where the walker is now (last trail square) */

    int   maze_count;             /* how many squares have joined the maze */

    /* the finished trail streaming in as the magenta wave */
    int   absorb_path[CELLS_MAX];
    int   absorb_len;
    int   absorb_progress;        /* next square to stream in */

    /* working room for the two breadth-first solve passes */
    int   bfs_queue [CELLS_MAX];
    int   bfs_dist  [CELLS_MAX];
    int   bfs_parent[CELLS_MAX];

    /* the longest path, and how far the beam has lit it */
    int   path[CELLS_MAX];
    int   path_len;
    int   solve_progress;
} Maze;

static inline int maze_idx(const Maze *m, int x, int y) { return y * m->w + x; }
static inline bool maze_in_bounds(const Maze *m, int x, int y)
{
    return x >= 0 && x < m->w && y >= 0 && y < m->h;
}

/* Knock down the wall between (x,y) and its neighbour in direction d. Both
 * squares must agree, so we clear the matching bit on each — the solver
 * trusts that later. */
static void maze_carve(Maze *m, int x, int y, int d)
{
    int idx  = maze_idx(m, x, y);
    int nx   = x + dir_dx(d);
    int ny   = y + dir_dy(d);
    int nidx = maze_idx(m, nx, ny);

    m->cells[idx ].walls &= (uint8_t)~WALL_BIT(d);
    m->cells[nidx].walls &= (uint8_t)~WALL_BIT(opposite(d));
}

/* Pick a random square that isn't in the maze yet — the next walk's start.
 * One pass, no extra memory (reservoir sampling). Returns -1 if the maze is
 * already full. */
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

/* Drop a fresh walker at `start` (a square not yet in the maze) and begin a
 * new trail there. */
static void maze_start_walk(Maze *m, int start)
{
    /* Belt-and-braces: clear any leftover trail flags. They should already be
     * clear (the wave and loop-erase clear them), but make sure. */
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

/* Start a brand-new run: all walls up, then plant one random seed square as
 * the maze's first member and send out the first walker. */
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

    /* Plant one random square as the maze's first member. */
    int seed_x = rand() % w;
    int seed_y = rand() % h;
    int seed   = maze_idx(m, seed_x, seed_y);
    m->cells[seed].in_maze = true;
    m->cells[seed].absorb_glow = 1.0f;
    m->maze_count = 1;

    /* Send the first walker out from somewhere else. */
    int start = maze_pick_random_outside(m);
    if (start >= 0) maze_start_walk(m, start);
}

/* Pick a random direction that stays on the grid. Just retries until one
 * lands in bounds — every square has at least two valid neighbours, so this
 * finishes fast. */
static int random_inbounds_dir(const Maze *m, int x, int y)
{
    int d;
    do {
        d = rand() % N_DIRS;
    } while (!maze_in_bounds(m, x + dir_dx(d), y + dir_dy(d)));
    return d;
}

/* The walk reached the maze at `dest`. Copy the finished trail (plus dest)
 * into the wave queue and end the walk; the wave streams it in afterward. We
 * leave the trail showing so the magenta wave can eat it square by square. */
static void walk_queue_absorb(Maze *m, int dest)
{
    m->absorb_len = 0;
    for (int i = 0; i < m->walk_len; i++)
        m->absorb_path[m->absorb_len++] = m->walk_path[i];
    m->absorb_path[m->absorb_len++] = dest;
    m->absorb_progress = 0;
    m->walk_len = 0;        /* the walk is finished */
}

/* The walker stepped back onto its own trail at `nidx` — a loop. Find that
 * square in the trail, drop everything after it (flash red, clear the flags),
 * and cut the trail back to it. Erasing loops like this is the whole trick
 * that keeps every maze equally likely. */
static void walk_erase_loop(Maze *m, int nidx)
{
    int hit = -1;
    for (int i = 0; i < m->walk_len; i++)
        if (m->walk_path[i] == nidx) { hit = i; break; }

    for (int i = hit + 1; i < m->walk_len; i++) {
        int c = m->walk_path[i];
        m->cells[c].in_walk     = false;
        m->cells[c].erase_glow  = 1.0f;
        m->cells[c].walk_glow  *= 0.3f;   /* fade the cyan out right away */
    }
    m->walk_len = hit + 1;
}

/* The walker stepped onto fresh ground; add the new square to the trail. */
static void walk_extend(Maze *m, int nidx)
{
    m->walk_path[m->walk_len++] = nidx;
    m->walk_pos                 = nidx;
    m->cells[nidx].in_walk      = true;
    m->cells[nidx].walk_glow    = 1.0f;
    m->cells[nidx].head_glow    = 1.0f;
}

/* Take one random step. Returns 2 if the walker reached the maze (walk done,
 * wave queued), 1 if it just moved, 0 if there's no walk going on. */
static int maze_walk_step(Maze *m)
{
    if (m->walk_len == 0) return 0;

    int x = m->walk_pos % m->w;
    int y = m->walk_pos / m->w;

    int d    = random_inbounds_dir(m, x, y);
    int nidx = maze_idx(m, x + dir_dx(d), y + dir_dy(d));

    /* Remember which way we left this square (this is what re-routes the trail
     * past a loop) and dim its glow so only the new position stays bright. */
    m->cells[m->walk_pos].walk_dir   = d;
    m->cells[m->walk_pos].head_glow *= 0.4f;

    if (m->cells[nidx].in_maze) {        /* reached the maze */
        walk_queue_absorb(m, nidx);
        return 2;
    }
    if (m->cells[nidx].in_walk) {        /* stepped on our own trail — a loop */
        walk_erase_loop(m, nidx);
        m->walk_pos = nidx;
        m->cells[nidx].head_glow = 1.0f;
        m->cells[nidx].walk_glow = 1.0f;
        return 1;
    }
    walk_extend(m, nidx);                /* fresh ground */
    return 1;
}

/* Stream one square of the finished trail into the maze, knocking down the
 * wall to the next square as we go. Returns true while there's more to add,
 * false once the wave is done.
 *
 * The queue runs trail-start ... last-trail-square, then the maze square the
 * walker hit. Each square except the last carves the wall toward the one
 * after it (using walk_dir). The last square is already in the maze, so it
 * just gets a magenta flash — no wall carved. */
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

/* Breadth-first search from `src`, returning the square farthest from it.
 * Run twice in a row and you get the two ends of the maze's longest path —
 * the classic two-BFS trick. (Same as maze_backtracker.c; copied in to keep
 * this file standalone.) */
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

/* ── §4 simulation — the scene state machine and the one per-tick update ── */

/*
 * The four phases the showcase cycles through:
 *   WALKING   — the walker wanders; switches to ABSORBING when it hits the maze.
 *   ABSORBING — the magenta wave streams the trail in; then either starts a new
 *               walk (maze not full) or moves to SOLVING (maze full).
 *   SOLVING   — the gold beam lights the longest path, a few squares per tick.
 *   HOLD      — pause to admire it, then start over.
 */
typedef enum {
    SCENE_WALKING   = 0,
    SCENE_ABSORBING = 1,
    SCENE_SOLVING   = 2,
    SCENE_HOLD      = 3,
} SceneState;

/*
 * Scene — the whole running showcase in one place:
 *   the maze itself, which phase we're in plus the HOLD countdown and the
 *   pause flag, and three speed knobs for how many walk / wave / beam steps
 *   to do each tick.
 */
typedef struct {
    Maze        m;                       /* the maze, and the walk in progress */
    SceneState  state;                   /* which phase we're in               */
    float       hold_timer;              /* seconds left in the HOLD pause      */
    bool        paused;                  /* freeze everything                   */
    int         walk_steps_per_tick;     /* walk speed (the +/- keys)           */
    int         absorb_steps_per_tick;   /* magenta-wave speed                  */
    int         solve_steps_per_tick;    /* gold-beam speed                     */
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

/* Fade every glow a little this tick. Each step multiplies by a number just
 * under 1, so glows ease smoothly toward zero rather than snapping off. */
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

/* Take several walk steps this tick. If the walker reaches the maze, hand off
 * to the wave phase. */
static void advance_walking(Scene *s)
{
    for (int i = 0; i < s->walk_steps_per_tick; i++) {
        int rc = maze_walk_step(&s->m);
        if (rc == 2) { s->state = SCENE_ABSORBING; break; }
        if (rc == 0) break;
    }
}

/* Stream the trail into the maze this tick. When the wave finishes, either
 * send out a new walker (cells left) or solve the finished maze. */
static void advance_absorbing(Scene *s)
{
    for (int i = 0; i < s->absorb_steps_per_tick; i++) {
        if (maze_absorb_step(&s->m)) continue;   /* wave still running */

        if (s->m.maze_count >= s->m.total_cells) {
            maze_compute_diameter(&s->m);
            s->state = SCENE_SOLVING;
        } else {
            int start = maze_pick_random_outside(&s->m);
            if (start >= 0) {
                maze_start_walk(&s->m, start);
                s->state = SCENE_WALKING;
            } else {
                /* Shouldn't be reachable, but if no outside square is left,
                 * just go solve. */
                maze_compute_diameter(&s->m);
                s->state = SCENE_SOLVING;
            }
        }
        break;
    }
}

/* Light a few more squares of the gold beam along the longest path. Once the
 * whole path is lit, pause on it. */
static void advance_solving(Scene *s)
{
    for (int i = 0; i < s->solve_steps_per_tick; i++) {
        if (s->m.solve_progress >= s->m.path_len) break;
        int idx = s->m.path[s->m.solve_progress++];
        s->m.cells[idx].solution_glow = 1.0f;
        s->m.cells[idx].on_path       = true;   /* stays lit once the beam passes */
    }
    if (s->m.solve_progress >= s->m.path_len) {
        s->state      = SCENE_HOLD;
        s->hold_timer = HOLD_SECONDS;
    }
}

/* Count down the pause on the finished maze, then start a new one. */
static void advance_hold(Scene *s, float dt)
{
    s->hold_timer -= dt;
    if (s->hold_timer <= 0.0f)
        scene_reset(s, s->m.w, s->m.h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    decay_glows(&s->m, dt);              /* fade every glow a bit */

    switch (s->state) {                  /* then advance whatever phase we're in */
    case SCENE_WALKING:   advance_walking(s);    break;
    case SCENE_ABSORBING: advance_absorbing(s);  break;
    case SCENE_SOLVING:   advance_solving(s);    break;
    case SCENE_HOLD:      advance_hold(s, dt);   break;
    }
}

/* ── §5 render — HUD, ASCII walls, walk arrows, glow colours ── */

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
 * Screen — just the terminal's current size in characters. Kept as its own
 * tiny type so the drawing functions only need this, not the whole program.
 *   cols, rows : terminal width and height, re-read whenever it resizes.
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
 * Decide what one interior square looks like: its colour, style, and
 * character. Many things can be true at once, so we check them in priority
 * order and the first match wins:
 *   reset flash     → yellow '*'
 *   the walker      → white '@'
 *   loop-erase      → red '!'
 *   magenta wave    → pink/magenta '*'
 *   gold beam       → pale→gold→dark '*'
 *   this walk's start → orange 'S'
 *   on the trail    → cyan arrow '^>v<'
 *   finished longest path → steady dark-gold '*'
 *   plain maze cell → dim steel-blue '.'
 *   nothing         → blank
 *
 * The caller tells us if this is the start square (only one ever is). We check
 * it after the live glows, so a walker passing back over its start briefly
 * shows '@', but before the plain trail arrow.
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
        /* Bright pink at the wave's front, fading to magenta behind it. */
        *pair  = (c->absorb_glow > 0.6f) ? PAIR_ABSORB_HI : PAIR_ABSORB;
        *attr  = A_BOLD;
        *glyph = '*';
        return true;
    }
    if (c->solution_glow > GLOW_THRESHOLD) {
        /* The gold beam: a pale-gold bright head, gold body, dark-gold tail. */
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
        /* Trail in three shades of cyan — brightest at the head, dimmer behind
         * — so it reads like a moving comet with a fading tail. */
        if (c->walk_glow > 0.55f)      { *pair = PAIR_WALK;     *attr = A_BOLD; }
        else if (c->walk_glow > 0.20f) { *pair = PAIR_WALK_MID; *attr = A_BOLD; }
        else                           { *pair = PAIR_WALK_LO;  *attr = A_NORMAL; }
        *glyph = dir_arrow(c->walk_dir);
        return true;
    }
    if (c->on_path) {
        /* The longest path stays lit in steady dark-gold under the bright beam,
         * so it's still visible during the HOLD pause. */
        *pair = PAIR_SOL_LO; *attr = A_NORMAL; *glyph = '*'; return true;
    }
    if (c->in_maze) {
        *pair = PAIR_VISITED; *attr = A_DIM; *glyph = '.'; return true;
    }
    return false;
}

/* Where the maze's top-left corner goes on screen. The maze is drawn at
 * double size (walls take their own rows and columns), centred between the
 * HUD rows. */
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

/* Does any wall touch this corner? If so we draw a '+' there. We just check
 * all eight wall edges that could meet at the corner. */
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

/* Is there a horizontal wall (a '-') here? For the very top and bottom edges
 * we read the outer cells' top/bottom wall instead. */
static bool h_wall_at(const Maze *m, int mx, int my)
{
    if (my == 0)    return (m->cells[maze_idx(m, mx, 0)].walls      & WALL_BIT(DIR_N)) != 0;
    if (my == m->h) return (m->cells[maze_idx(m, mx, m->h-1)].walls & WALL_BIT(DIR_S)) != 0;
    return (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_N)) != 0;
}

/* Is there a vertical wall (a '|') here? For the left and right edges we read
 * the outer cells' left/right wall instead. */
static bool v_wall_at(const Maze *m, int mx, int my)
{
    if (mx == 0)    return (m->cells[maze_idx(m, 0, my)].walls      & WALL_BIT(DIR_W)) != 0;
    if (mx == m->w) return (m->cells[maze_idx(m, m->w-1, my)].walls & WALL_BIT(DIR_E)) != 0;
    return (m->cells[maze_idx(m, mx, my)].walls & WALL_BIT(DIR_W)) != 0;
}

/* Draw the maze walls: '+' at every corner that has a wall, '-' and '|' along
 * the standing wall segments, all kept inside the screen. */
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

/* Draw what sits inside each cell — glows, trail arrows, maze dots — using
 * cell_visual. The walk's start square (drawn as 'S') is the first trail entry. */
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

    /* Row 0 — title on the left, live stats on the right. */
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

    /* Row 1 — what the symbols mean (not bold, so row 0 stands out). */
    snprintf(buf, sizeof buf,
             " S:source  @:walker  ^>v<:trail  !:loop-erase  *:wave/beam ");
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom row — the keys you can press. */
    snprintf(buf, sizeof buf,
             " q:quit  spc:pause  r:reset  +/-:speed  [/]:Hz ");
    if ((int)strlen(buf) > cols) buf[cols] = '\0';
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6 app — signals, key/resize events, the main loop ── */

/*
 * App — the whole running program: the scene being animated, the terminal it
 * draws to, the chosen maze size, and the few flags the main loop and signal
 * handlers need. Kept apart from Scene so the drawing code only needs Screen,
 * and so the signal handler can reach the run flags through the one g_app.
 *   maze_w/maze_h are re-fit to the terminal on resize, and capped at the MAX
 *   sizes so the cell arrays can never overflow.
 *   running and need_resize are written from signal handlers, hence the
 *   volatile sig_atomic_t type.
 */
typedef struct {
    Scene                 scene;        /* the simulation                  */
    Screen                screen;       /* where it's drawn                */
    int                   sim_fps;      /* tick rate (the [ / ] keys)      */
    int                   maze_w, maze_h; /* chosen maze size              */
    volatile sig_atomic_t running;      /* clear this to quit              */
    volatile sig_atomic_t need_resize;  /* set when the window resizes     */
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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* don't try to catch up forever */

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
