/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maze_backtracker.c — watch a maze dig itself, then solve itself.
 *
 * A glowing head walks a random path, knocking down walls as it goes and
 * backing up when it hits a dead end. Once every cell has been reached, it
 * finds the longest possible route through the maze and lights it up in gold.
 *
 * Sister file: maze.c (same folder) does the same thing with no glow and a
 * plain corner-to-corner solve — read it for the bare-bones version.
 * The "two walks to find a tree's longest path" trick is well known; see
 * https://cp-algorithms.com/graph/tree_painting.html#diameter-of-a-tree
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

/* §1  config + types */

enum {
    /* Biggest maze we'll ever build, counted in cells. Each cell needs two
     * screen columns/rows once you add its walls, so 120x40 cells fills a
     * roughly 241x81 terminal. */
    MAZE_W_MAX        = 120,
    MAZE_H_MAX        =  40,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    DIG_STEPS_MIN     =   1,
    DIG_STEPS_DEF     =   8,        /* dig steps taken per tick */
    DIG_STEPS_MAX     = 256,

    SOLVE_STEPS_DEF   =   1,        /* cells the gold beam lights per tick */

    RENDER_CAP_FPS    =  60,        /* never draw faster than this, even if the sim runs faster */
    MAX_FRAME_MS      = 100,        /* if a frame ran long (we were paused/swapped out), pretend
                                       it was only this long so we don't fast-forward wildly */
    FPS_UPDATE_MS     = 500,

    /* One colour slot per thing we draw. The first two (HUD text) stay fixed
     * bright so they're readable on any theme; the rest get recoloured when
     * you switch themes. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* the wall lines          */
    PAIR_VISITED      =   4,        /* a cell that's been dug but is just sitting there */
    PAIR_TRAIL        =   5,        /* the fading trail behind the head */
    PAIR_HEAD         =   6,        /* the digging head itself */
    PAIR_SOLUTION     =   7,        /* the gold solution beam  */
    PAIR_SUPERNOVA    =   8,        /* the flash when you reset */
};

/*
 * Theme — one colour scheme for the maze. Cycle through them with t / T.
 *
 * Each field is an xterm-256 colour number for one part of the picture.
 * They're all chosen from the bright half of the palette on purpose: the
 * resting interior is drawn dimmed, and dark colours go invisible when dimmed
 * on a black terminal, so even the "dull" parts here are mid-bright.
 */
typedef struct {
    const char *name;     /* shown in the HUD, e.g. "AURORA" */
    short wall;           /* the wall lines */
    short visited;        /* a dug cell sitting quietly */
    short trail;          /* the fading trail behind the head */
    short head;           /* the digging head */
    short solution;       /* the solution beam */
    short supernova;      /* the reset flash */
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
 * Maze sizes you can pick with the number keys (1..9, then 0 for the biggest).
 * Each width/height is just a wish — if it won't fit the terminal it gets
 * shrunk down to what fits (see app_pick_maze_size). The last one ("Max")
 * always fills whatever screen you have.
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

/* How fast each glow fades. Bigger number = fades quicker. The head fades
 * fastest since only one cell is ever the head; the trail lingers about a
 * second so you can follow where the digger has been. */
#define DIG_GLOW_DECAY      2.5f
#define HEAD_GLOW_DECAY     8.0f
#define SOLUTION_GLOW_DECAY 1.5f
#define SUPERNOVA_DECAY     2.0f
#define GLOW_THRESHOLD      0.05f   /* below this a glow counts as "off" */

/* The four compass directions, numbered 0..3. The little helpers below turn a
 * direction into a step (dx/dy) or into the way back (opposite). */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3, N_DIRS = 4 };
static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }
static inline int opposite(int d) { return (d + 2) & 3; }

#define WALL_BIT(d)   (1u << (d))   /* one bit per wall: N=1, E=2, S=4, W=8 */
#define ALL_WALLS     0x0Fu         /* all four walls present */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CELLS_MAX  (MAZE_W_MAX * MAZE_H_MAX)

/*
 * Cell — one square of the maze. Holds the real maze state (its walls and
 * whether it's been dug) plus a few "glow" values that are purely for show.
 *
 * Walls are stored as one bit per side rather than four separate flags,
 * because the only questions we ask are "is this side open?" (one bit test)
 * and "knock this side down" (clear one bit). The catch: a wall between two
 * cells is recorded on BOTH of them, so when you knock one down you have to
 * clear it on both — that's what maze_carve is for.
 *
 * The glows are just brightness levels from 0 (off) to 1 (full). Each one
 * gets set to 1 when its moment happens and quietly fades every tick. They
 * only affect colour; the maze logic never looks at them.
 */
typedef struct {
    uint8_t walls;          /* which walls are still up (N=1 E=2 S=4 W=8) */
    bool    visited;        /* has the digger reached this cell yet? */
    float   dig_glow;       /* the trail glow, flares up when dug or retraced */
    float   head_glow;      /* the head glow, on only at the current dig spot */
    float   solution_glow;  /* the gold beam, on as the solution lights up */
    float   supernova_glow; /* the reset flash, on across the whole grid on reset */
} Cell;

/*
 * Maze — the grid itself, plus all the scratch space for building and solving it.
 *
 * A finished maze has exactly one route between any two cells and no loops
 * (in graph terms, it's a tree). We build it by walking randomly with a stack
 * (dig forward, back up at dead ends), then solve it by finding the two cells
 * that are farthest apart and the route between them.
 *
 * There's no "phase" field — you can tell where we are from the numbers:
 *   stack not empty           → still digging
 *   stack empty, not solved   → done digging, ready to solve
 *   solved                    → finished. (§5's Scene tracks this more explicitly.)
 */
typedef struct {
    /* the grid */
    int   w, h;                  /* size in cells */
    int   total_cells;           /* w*h, kept handy for the loops below */
    Cell  cells[CELLS_MAX];      /* the cells, row by row; cell (x,y) lives at y*w+x */

    /* digging: the stack of cells on the path back to the start. Sized for the
     * whole grid because one long winding corridor can pile up every cell. */
    int   stack[CELLS_MAX];      /* the current path, as cell numbers */
    int   sp;                    /* how many cells are on the stack right now */
    int   visited_count;         /* how many cells have been dug so far */

    /* solving: a queue and a "where did I come from" list, reused for both of
     * the two breadth-first sweeps. The came-from list lets us retrace the route. */
    int   bfs_queue [CELLS_MAX]; /* cells waiting to be explored */
    int   bfs_dist  [CELLS_MAX]; /* steps from the start of the sweep; -1 = not reached */
    int   bfs_parent[CELLS_MAX]; /* the cell we arrived from; -1 = the start */

    /* the answer: the longest route through the maze */
    int   path[CELLS_MAX];       /* the route, one end to the other */
    int   path_len;              /* how many cells long it is */
    int   solve_progress;        /* how far the gold beam has lit it up */
    bool  solved;                /* has the route been worked out yet? */
} Maze;

static inline int maze_idx(const Maze *m, int x, int y) { return y * m->w + x; }
static inline bool maze_in_bounds(const Maze *m, int x, int y)
{
    return x >= 0 && x < m->w && y >= 0 && y < m->h;
}

/*
 * Which stage of the show we're in:
 *   DIG    — still carving the maze.
 *   SOLVE  — maze is built; lighting up the solution beam cell by cell.
 *   DONE   — finished. The solution stays lit until you press r for a new maze.
 */
typedef enum { SCENE_DIG = 0, SCENE_SOLVE = 1, SCENE_DONE = 2 } SceneState;

/* Scene — one run of the animation: the maze, how fast to fast-forward it,
 * which stage we're in, and whether it's paused. */
typedef struct {
    Maze       m;
    int        dig_steps_per_tick;    /* dig this many steps each tick */
    int        solve_steps_per_tick;  /* light this many beam cells each tick */
    SceneState state;
    bool       paused;
} Scene;

/* Screen — how big the terminal is right now. Passed to the drawing code so it
 * doesn't need to know about the rest of the program. */
typedef struct { int cols, rows; } Screen;

/* App — everything the program holds: the running scene, the terminal, the
 * user's current choices, and two flags the OS sets when it's time to quit or
 * the window was resized. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;        /* how many ticks per second to run */
    int                   theme;          /* which theme is selected */
    int                   preset;         /* which size preset is selected */
    int                   maze_w, maze_h; /* the actual maze size, after fitting to the screen */
    volatile sig_atomic_t running;        /* set to 0 to quit (signal handler touches this) */
    volatile sig_atomic_t need_resize;    /* set to 1 when the window changed size */
} App;

static App g_app;

/* §2  timing — read the clock, sleep a bit */

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

/* §3  pure helpers — these only read, never change anything */

/*
 * Which wall lines meet at the grid corner (cx, cy)?
 *
 * A corner is where up to four cells touch, and four wall segments can poke
 * out of it (up, down, left, right). We check each segment by asking the cells
 * on either side of it whether they still have that wall up, and build a little
 * 4-bit answer (the same N/E/S/W bits as everywhere else). Cells off the edge
 * of the grid count as "no wall", which makes the maze's outer border corners
 * come out right with no special handling.
 */
static int corner_walls_at(const Maze *m, int cx, int cy)
{
    int mask = 0;

    bool ne = maze_in_bounds(m, cx,     cy - 1);
    bool nw = maze_in_bounds(m, cx - 1, cy - 1);
    bool se = maze_in_bounds(m, cx,     cy    );
    bool sw = maze_in_bounds(m, cx - 1, cy    );

    /* a wall going up out of the corner? */
    if ((ne && (m->cells[maze_idx(m, cx,     cy-1)].walls & WALL_BIT(DIR_W))) ||
        (nw && (m->cells[maze_idx(m, cx - 1, cy-1)].walls & WALL_BIT(DIR_E))))
        mask |= 1;

    /* a wall going right? */
    if ((se && (m->cells[maze_idx(m, cx, cy    )].walls & WALL_BIT(DIR_N))) ||
        (ne && (m->cells[maze_idx(m, cx, cy - 1)].walls & WALL_BIT(DIR_S))))
        mask |= 2;

    /* a wall going down? */
    if ((se && (m->cells[maze_idx(m, cx,     cy)].walls & WALL_BIT(DIR_W))) ||
        (sw && (m->cells[maze_idx(m, cx - 1, cy)].walls & WALL_BIT(DIR_E))))
        mask |= 4;

    /* a wall going left? */
    if ((sw && (m->cells[maze_idx(m, cx - 1, cy    )].walls & WALL_BIT(DIR_N))) ||
        (nw && (m->cells[maze_idx(m, cx - 1, cy - 1)].walls & WALL_BIT(DIR_S))))
        mask |= 8;

    return mask;
}

/*
 * Decide how a cell's middle should look, based on its brightest glow.
 * Brightest wins: reset flash, then the head, then the trail, then a plain
 * dug cell. An untouched cell returns false so the caller leaves it blank.
 *
 * The gold solution isn't handled here — it's drawn separately as one
 * connected line so it looks like a route, not a scatter of dots.
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
    return false;   /* nothing here — caller skips it */
}

/* Is this screen cell actually on screen? Checked before every draw so we
 * never write off the edge. */
static bool in_screen(int sx, int sy, int cols, int rows)
{
    return sx >= 0 && sx < cols && sy >= 0 && sy < rows;
}

/* List the directions from (x,y) that lead to a neighbour we haven't dug yet —
 * those are the digger's choices for where to go next. Returns how many. */
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

/* §4  the maze logic — building and solving */

/* Start a fresh maze: every wall up, nothing dug, then pick a random cell to
 * begin from. The whole grid flashes (supernova_glow) so a reset is visible. */
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
        m->cells[i].supernova_glow = 1.0f;
    }

    /* drop the digger on a random cell to start */
    int sx = rand() % w;
    int sy = rand() % h;
    int s  = maze_idx(m, sx, sy);
    m->cells[s].visited = true;
    m->cells[s].dig_glow = 1.0f;
    m->cells[s].head_glow = 1.0f;
    m->visited_count = 1;
    m->stack[m->sp++] = s;
}

/* Knock down the wall between (x,y) and the neighbour in direction d. Because
 * each wall is recorded on both cells, we clear it on both. */
static void maze_carve(Maze *m, int x, int y, int d)
{
    int idx  = maze_idx(m, x, y);
    int nx   = x + dir_dx(d);
    int ny   = y + dir_dy(d);
    int nidx = maze_idx(m, nx, ny);

    m->cells[idx ].walls &= (uint8_t)~WALL_BIT(d);
    m->cells[nidx].walls &= (uint8_t)~WALL_BIT(opposite(d));
}

/* Hit a dead end — back up one cell. We re-light the cell we land on so you can
 * see the digger retracing its steps. When the stack empties, the maze is done. */
static void maze_backtrack(Maze *m)
{
    m->sp--;
    if (m->sp > 0) {
        int new_top = m->stack[m->sp - 1];
        m->cells[new_top].dig_glow  = 1.0f;
        m->cells[new_top].head_glow = 1.0f;
    }
}

/* Step forward into the neighbour in direction d: knock the wall down, mark the
 * new cell dug and glowing, and push it so we can find our way back later. */
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
 * Take one dig step: either push into a random unvisited neighbour, or, if
 * boxed in, back up one cell. Returns false once the maze is fully dug.
 *
 * It's one step per call on purpose — that's what lets the animation show the
 * head crawling forward and the trail re-flaring as it backs out of dead ends.
 */
static bool maze_dig_step(Maze *m)
{
    if (m->sp <= 0) return false;             /* nothing left to dig */

    int top = m->stack[m->sp - 1];
    int x   = top % m->w;
    int y   = top / m->w;

    /* dim where the head was; the fade finishes the job on later frames */
    m->cells[top].head_glow *= 0.5f;

    int dirs[N_DIRS];
    int n = unvisited_dirs(m, x, y, dirs);
    if (n == 0) { maze_backtrack(m); return true; }   /* dead end, back up */

    maze_advance(m, x, y, dirs[rand() % n]);          /* pick an exit at random */
    return true;
}

/*
 * Spread out from one cell, one step at a time, until the whole maze is
 * covered, and report the cell that turned out to be farthest away. Along the
 * way it records each cell's came-from neighbour, so we can retrace the route
 * afterwards. Running this twice (start anywhere, then start from the farthest
 * cell it found) lands us on the two ends of the longest route in the maze.
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
            /* a standing wall means there's no passage this way */
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

/* Work out the longest route through the maze and store it in path[]. Find one
 * far end, then the far end from there, then retrace between them. */
static void maze_compute_diameter(Maze *m)
{
    int a = maze_bfs_farthest(m, 0);    /* a far corner from cell 0 */
    int b = maze_bfs_farthest(m, a);    /* the farthest cell from a — the other end */

    /* follow the came-from trail from b back to a to collect the route */
    m->path_len = 0;
    int cur = b;
    while (cur != -1 && m->path_len < m->total_cells) {
        m->path[m->path_len++] = cur;
        cur = m->bfs_parent[cur];
    }
    m->solve_progress = 0;
    m->solved = true;
}

/* §5  the scene — one tick of the whole show */

/* Throw away the current maze and start a brand-new one at the given size. */
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

/* Dim every glow a little, once per tick. Each frame multiplies a glow by a
 * number just under 1, so it fades smoothly toward off rather than blinking. */
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

/* While digging: take a batch of dig steps. When the maze is fully dug, work
 * out the solution and switch to showing it. */
static void scene_dig(Scene *s)
{
    for (int i = 0; i < s->dig_steps_per_tick; i++)
        if (!maze_dig_step(&s->m)) break;
    if (s->m.sp == 0) {
        maze_compute_diameter(&s->m);
        s->state = SCENE_SOLVE;
    }
}

/* While solving: light a few more cells of the gold route each tick so it
 * streams across the maze. Once the whole route is lit, we're done. */
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

/* When finished: re-light the whole route every tick so the fade can't dim it,
 * keeping the solution on screen until the user starts a new maze. */
static void scene_hold(Scene *s)
{
    for (int i = 0; i < s->m.path_len; i++)
        s->m.cells[s->m.path[i]].solution_glow = 1.0f;
}

/* One tick: fade the glows, then do whatever the current stage needs. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    decay_glows(&s->m, dt);
    switch (s->state) {
    case SCENE_DIG:   scene_dig(s);   break;
    case SCENE_SOLVE: scene_solve(s); break;
    case SCENE_DONE:  scene_hold(s);  break;
    }
}

/* §6  drawing — turn the maze into characters on screen */

/* Load a theme's colours. The two HUD slots stay fixed bright so the readout is
 * always legible; everything else takes its colour from the theme. */
static void color_apply(const Theme *th)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);  /* HUD: fixed bright yellow */
    init_pair(PAIR_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);  /* hints: fixed bright cyan */
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
 * What to draw at a corner, looked up by the wall bits from corner_walls_at.
 * No walls meet there → a space; any walls meet → a '+'. We stick to plain
 * ASCII ('+', '-', '|') so the maze looks the same on every terminal.
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
 * Draw the solution as one connected gold line.
 *
 * The cells of the route sit two screen columns apart, so on their own they'd
 * look like loose dots. To join them up we also fill the gap between each pair
 * of neighbours with a '-' or '|'. The two ends are marked 'S' and 'E'. Only
 * cells the beam has reached are drawn, so it still streams in while solving.
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
        /* 'S' and 'E' for the two ends, '+' for everything in between */
        char node = (i == 0)                 ? 'S'
                  : (i == m->path_len - 1)   ? 'E'
                  :                            '+';
        if (sy >= 0 && sy < rows && sx >= 0 && sx < cols)
            mvaddch(sy, sx, (chtype)(unsigned char)node);

        /* fill the gap to the previous cell on the route */
        if (i > 0) {
            int pidx = m->path[i - 1];
            if (m->cells[pidx].solution_glow > GLOW_THRESHOLD) {
                int px = pidx % m->w, py = pidx / m->w;
                int csy = gy0 + y + py + 1;   /* halfway between the two cells */
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
 * Where the maze sits on screen.
 *
 * Each maze cell takes a 2x2 block once you draw its walls, so a w-by-h maze is
 * (2w+1) by (2h+1) characters. Within a cell's block: the middle is the floor,
 * the top and left edges are its walls, the top-left is a corner. The functions
 * below draw each of those, offset by the frame's top-left corner (gx0, gy0).
 */

/* Find the top-left corner so the maze is centred, while leaving the top row
 * for the readout and the bottom row for the key hints. */
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

/* Draw the corner at grid point (mx,my): a '+' if any wall meets there. */
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

/* Draw a cell's top wall (the '-'). On the very bottom row it instead draws
 * the bottom edge of the maze using the last cell's south wall. */
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

/* Draw a cell's left wall (the '|'). On the far right column it instead draws
 * the right edge of the maze using the last cell's east wall. */
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

/* Draw all the walls. We loop one past the edge in each direction so the maze's
 * outer border gets drawn too. */
static void draw_walls(const Maze *m, int gx0, int gy0, int cols, int rows)
{
    for (int my = 0; my <= m->h; my++)
        for (int mx = 0; mx <= m->w; mx++) {
            draw_corner_glyph(m, mx, my, gx0, gy0, cols, rows);
            if (mx < m->w) draw_north_wall(m, mx, my, gx0, gy0, cols, rows);
            if (my < m->h) draw_west_wall (m, mx, my, gx0, gy0, cols, rows);
        }
}

/* Colour in the middle of each cell according to its brightest glow. */
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

/* Draw the whole maze: walls first, then the cell colours, then the gold
 * solution line on top. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Maze *m = &s->m;
    int gx0, gy0;
    maze_screen_origin(m, cols, rows, &gx0, &gy0);
    draw_walls(m, gx0, gy0, cols, rows);
    draw_interiors(m, gx0, gy0, cols, rows);
    scene_draw_solution(s, gx0, gy0, cols, rows);
}

/* Print one line of HUD text, trimmed to the window width so it can't spill
 * over the maze. */
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

    /* top line: the status readout */
    char data[256];
    snprintf(data, sizeof data,
             " Maze  %s  %s %dx%d  %s  %d/%d cells  steps:%d  %.1f fps  %d Hz ",
             theme, preset, m->w, m->h, state_str,
             m->visited_count, m->total_cells,
             s->dig_steps_per_tick, fps, sim_fps);
    draw_hud_row(sc, 0, PAIR_HUD, data);

    /* bottom line: the list of keys you can press */
    draw_hud_row(sc, sc->rows - 1, PAIR_HINT,
                 " q:quit  spc:pause  r:reset  1-0:size  t:theme  +/-:steps  [/]:Hz ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* §7  the program — signals, key handling, and the main loop */

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/*
 * Pick the actual maze size: take the chosen preset, then shrink it to whatever
 * fits the terminal (each cell needs two characters, and we keep one row top and
 * bottom for the HUD). A preset too big for the window just fills the window.
 */
static void app_pick_maze_size(App *app)
{
    int fit_w = (app->screen.cols - 1) / 2;   /* widest maze that fits across */
    int fit_h = (app->screen.rows - 3) / 2;   /* tallest that fits, minus the two HUD rows */
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

/* The window changed size: re-measure it, refit the maze, and start fresh. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_maze_size(app);
    scene_reset(&app->scene, app->maze_w, app->maze_h);
    app->need_resize = 0;
}

/* Handle one key press. Returns false only when it's time to quit. */
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
        /* number keys pick a size: 1..9 are the first nine, 0 is the biggest */
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

/* Get everything ready before the loop starts: defaults, terminal, theme, and
 * the first maze. */
static void app_init(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    app->theme   = 0;
    app->preset  = N_MAZE_PRESETS - 1;   /* default to "Max" so it fills the window */

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

        /* did the window get resized since last time? */
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

        /* run as many fixed-size ticks as the elapsed time has earned, so the
         * maze advances at the same pace no matter the frame rate */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* refresh the fps number shown in the HUD every so often */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* sleep just enough to hold the draw rate at RENDER_CAP_FPS */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        /* draw the frame */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps,
                    THEMES[app->theme].name, MAZE_PRESETS[app->preset].name);
        screen_present();

        /* check for a key press */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
