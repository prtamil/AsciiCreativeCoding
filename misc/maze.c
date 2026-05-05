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
 * Study alongside: misc/forest_fire.c — both are cell-grid simulations
 *   driven by an iterative state machine, and both colour-code the
 *   activity wave on top of a static field.
 *
 * Section map:
 *   §1 config   — sizes, wall bits, phase enum, color pair IDs
 *   §2 clock    — monotonic timer + nanosleep
 *   §3 color    — wall/frontier/path/hud/hint pairs
 *   §5 maze     — wall bitmask + iterative DFS carver
 *   §6 solve    — BFS flood + parent-array path reconstruction
 *   §7 scene    — mark_cell helper + draw_grid + draw_hud
 *   §8 app      — signals, resize, main loop
 *
 * Keys:  q/ESC quit   r regen   space skip-to-solve   p pause   1/2/3 sizes
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra misc/maze.c -o maze -lncurses -lm
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
 * References     : Reiter (2011), "Maze Generation Algorithms",
 *                    https://weblog.jamisbuck.org/2011/2/7/
 *                  Wikipedia, "Maze generation algorithm",
 *                    https://en.wikipedia.org/wiki/Maze_generation_algorithm
 *                  Red Blob Games, "Introduction to A* / BFS",
 *                    https://www.redblobgames.com/pathfinding/a-star/introduction.html
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
 *  • Resize.  SIGWINCH recomputes (g_mh, g_mw) and resets the maze;
 *    holding the carve state through a resize would point off-grid.
 *  • A_DIM on grayscale wall colours would make them disappear on a
 *    black terminal.  All wall/frontier/path pairs sit in the bright
 *    half of the 256-colour cube and use A_BOLD to stay visible.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Every cell becomes visited.  By the time generation finishes,
 *    g_dfs_top == 0 and g_vis is all 1s; visually no '#' tiles remain.
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

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
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

enum Phase { PH_GENERATE, PH_SOLVE, PH_DONE };

/* Color pair IDs.  HUD pairs are reserved 8/9 across all demos so the
 * status line and key hint are stylistically identical everywhere. */
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

/* ===================================================================== */
/* §2  clock                                                              */
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
/* §3  color                                                              */
/* ===================================================================== */

/*
 * color_init() — every pair uses bg = -1 (terminal default) so the
 * maze blends into any wallpaper, and every fg colour sits in the
 * bright half of the 256-colour cube so A_BOLD/A_DIM render legibly.
 * Walls in light grey, frontier in yellow, BFS flood in cyan, final
 * path in matrix green — high contrast from each other and from the
 * yellow/cyan HUD pair so nothing camouflages.
 */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_WALL,    251, -1);   /* light grey walls          */
        init_pair(PAIR_VISIT,   244, -1);   /* dim grey carved interior  */
        init_pair(PAIR_FRONT,   226, -1);   /* bright yellow DFS head    */
        init_pair(PAIR_BFS,     117, -1);   /* light blue BFS flood      */
        init_pair(PAIR_PATH,     46, -1);   /* matrix green path         */
        init_pair(PAIR_UNVISIT, 240, -1);   /* near-black unvisited fill */
        init_pair(PAIR_HUD,     226, -1);   /* yellow status (A_BOLD)    */
        init_pair(PAIR_HINT,     51, -1);   /* cyan hint   (A_BOLD)      */
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

/* ===================================================================== */
/* §5  maze generation (iterative DFS)                                    */
/* ===================================================================== */

static unsigned char g_walls[MAZE_H_MAX][MAZE_W_MAX]; /* per-cell bitmask */
static unsigned char g_vis  [MAZE_H_MAX][MAZE_W_MAX]; /* DFS visited      */
static int g_mh, g_mw;                                /* live maze size   */

/* DFS stack of (r,c) frontier positions */
static struct { int r, c; } g_dfs[MAZE_H_MAX * MAZE_W_MAX + 1];
static int g_dfs_top;

static enum Phase g_phase;
static bool       g_paused;

/* Direction tables: index 0..3 = N, E, S, W. */
static const int DR[4]     = { -1,  0,  1,  0 };
static const int DC[4]     = {  0,  1,  0, -1 };
static const int D_WALL[4] = { WALL_N, WALL_E, WALL_S, WALL_W };
static const int D_OPP[4]  = { WALL_S, WALL_W, WALL_N, WALL_E };

static void maze_reset(void)
{
    memset(g_walls, 0x0F, sizeof g_walls); /* every wall closed */
    memset(g_vis,   0,    sizeof g_vis);
    g_dfs[0].r = 0;
    g_dfs[0].c = 0;
    g_dfs_top  = 1;
    g_vis[0][0] = 1;
    g_phase    = PH_GENERATE;
}

/*
 * gen_step() — advance the recursive backtracker by one node.
 *
 * We pick a random unvisited neighbour of the stack-top cell, carve
 * the wall between them, push the neighbour, and return.  If the top
 * cell has no unvisited neighbour, we pop (backtrack).  Returns false
 * once the stack empties — the maze is complete.
 */
static bool gen_step(void)
{
    if (g_dfs_top == 0) return false;

    int r = g_dfs[g_dfs_top - 1].r;
    int c = g_dfs[g_dfs_top - 1].c;

    int dirs[4] = { 0, 1, 2, 3 };
    for (int i = 3; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = dirs[i]; dirs[i] = dirs[j]; dirs[j] = t;
    }

    for (int k = 0; k < 4; k++) {
        int d  = dirs[k];
        int nr = r + DR[d], nc = c + DC[d];
        if (nr < 0 || nr >= g_mh || nc < 0 || nc >= g_mw) continue;
        if (g_vis[nr][nc]) continue;

        g_vis[nr][nc] = 1;
        g_walls[r ][c ] &= (unsigned char)~D_WALL[d];
        g_walls[nr][nc] &= (unsigned char)~D_OPP [d];
        g_dfs[g_dfs_top].r = nr;
        g_dfs[g_dfs_top].c = nc;
        g_dfs_top++;
        return true;
    }
    g_dfs_top--;   /* backtrack */
    return true;
}

/* ===================================================================== */
/* §6  BFS solve                                                          */
/* ===================================================================== */

static int           g_parent  [MAZE_H_MAX][MAZE_W_MAX]; /* enc r*W+c    */
static unsigned char g_bfs_vis [MAZE_H_MAX][MAZE_W_MAX];
static unsigned char g_on_path [MAZE_H_MAX][MAZE_W_MAX];
static struct { int r, c; } g_bq[MAZE_H_MAX * MAZE_W_MAX];
static int g_bq_head, g_bq_tail;

static void solve_start(void)
{
    memset(g_bfs_vis, 0, sizeof g_bfs_vis);
    memset(g_on_path, 0, sizeof g_on_path);
    memset(g_parent, -1, sizeof g_parent);
    g_bq_head = g_bq_tail = 0;
    g_bq[g_bq_tail].r = 0;
    g_bq[g_bq_tail].c = 0;
    g_bq_tail++;
    g_bfs_vis[0][0] = 1;
    g_phase = PH_SOLVE;
}

/*
 * trace_path() — walk the parent chain back from the goal cell to the
 * start, marking each cell along the way as on the shortest path.
 */
static void trace_path(int gr, int gc)
{
    int pr = gr, pc = gc;
    while (pr >= 0 && pc >= 0) {
        g_on_path[pr][pc] = 1;
        int enc = g_parent[pr][pc];
        if (enc < 0) break;
        pr = enc / MAZE_W_MAX;
        pc = enc % MAZE_W_MAX;
    }
}

static bool solve_step(void)
{
    if (g_bq_head >= g_bq_tail) { g_phase = PH_DONE; return false; }

    int r = g_bq[g_bq_head].r;
    int c = g_bq[g_bq_head].c;
    g_bq_head++;

    if (r == g_mh - 1 && c == g_mw - 1) {
        trace_path(r, c);
        g_phase = PH_DONE;
        return false;
    }

    for (int d = 0; d < 4; d++) {
        if (g_walls[r][c] & D_WALL[d]) continue;
        int nr = r + DR[d], nc = c + DC[d];
        if (nr < 0 || nr >= g_mh || nc < 0 || nc >= g_mw) continue;
        if (g_bfs_vis[nr][nc]) continue;
        g_bfs_vis[nr][nc] = 1;
        g_parent [nr][nc] = r * MAZE_W_MAX + c;
        g_bq[g_bq_tail].r = nr;
        g_bq[g_bq_tail].c = nc;
        g_bq_tail++;
    }
    return true;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

static int g_rows, g_cols;

/*
 * mark_cell() — bounds-checked mvaddch with the canonical
 * (chtype)(unsigned char) double-cast that prevents sign-extension
 * corruption on glyphs above 0x7F.  All maze drawing routes through
 * this single helper so clipping logic lives in exactly one place.
 */
static void mark_cell(int sr, int sc, char ch, int pair, int attr)
{
    if (sr < 0 || sr >= g_rows) return;
    if (sc < 0 || sc >= g_cols) return;
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
static void draw_lattice_pixel(int pr, int pc)
{
    int sr = pr + HUD_ROWS;
    bool corner = !(pr & 1) && !(pc & 1);
    bool hwall  = !(pr & 1) &&  (pc & 1);
    bool vwall  =  (pr & 1) && !(pc & 1);

    if (corner) {
        mark_cell(sr, pc, '+', PAIR_WALL, A_BOLD);
        return;
    }
    if (hwall) {
        int r = pr / 2 - 1, c = pc / 2;
        bool open = (r >= 0) && !(g_walls[r][c] & WALL_S);
        if (open) mark_cell(sr, pc, ' ', 0, 0);
        else      mark_cell(sr, pc, '-', PAIR_WALL, A_BOLD);
        return;
    }
    if (vwall) {
        int r = pr / 2, c = pc / 2 - 1;
        bool open = (c >= 0) && !(g_walls[r][c] & WALL_E);
        if (open) mark_cell(sr, pc, ' ', 0, 0);
        else      mark_cell(sr, pc, '|', PAIR_WALL, A_BOLD);
    }
}

/* Pick the glyph + colour for the interior of a maze cell. */
static void draw_cell_interior(int r, int c, int sr, int sc)
{
    if (g_phase == PH_GENERATE) {
        if (g_dfs_top > 0 &&
            g_dfs[g_dfs_top - 1].r == r &&
            g_dfs[g_dfs_top - 1].c == c) {
            mark_cell(sr, sc, '@', PAIR_FRONT, A_BOLD);
        } else if (g_vis[r][c]) {
            mark_cell(sr, sc, ' ', PAIR_VISIT, 0);
        } else {
            mark_cell(sr, sc, '#', PAIR_UNVISIT, 0);
        }
        return;
    }
    /* PH_SOLVE or PH_DONE */
    if (g_on_path[r][c])      mark_cell(sr, sc, '*', PAIR_PATH, A_BOLD);
    else if (g_bfs_vis[r][c]) mark_cell(sr, sc, '.', PAIR_BFS,  0);
    else if (g_vis[r][c])     mark_cell(sr, sc, ' ', PAIR_VISIT, 0);
    else                      mark_cell(sr, sc, '#', PAIR_UNVISIT, 0);
}

static void draw_grid(void)
{
    int max_pr = 2 * g_mh;
    int max_pc = 2 * g_mw;
    for (int pr = 0; pr <= max_pr; pr++) {
        for (int pc = 0; pc <= max_pc; pc++) {
            bool inside = (pr & 1) && (pc & 1);
            if (inside) {
                int r  = pr / 2;
                int c  = pc / 2;
                int sr = pr + HUD_ROWS;
                draw_cell_interior(r, c, sr, pc);
            } else {
                draw_lattice_pixel(pr, pc);
            }
        }
    }
}

static void draw_hud(void)
{
    const char *phase_str =
        (g_phase == PH_GENERATE) ? "carving (DFS)" :
        (g_phase == PH_SOLVE)    ? "solving (BFS)" :
                                   "done";
    char status[80];
    snprintf(status, sizeof status, " maze %dx%d  %s  %s ",
             g_mw, g_mh, phase_str, g_paused ? "PAUSED" : "running");

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    int pad = g_cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(g_rows - 1, 0,
        " q:quit  r:regen  spc:skip-to-solve  p:pause  1/2/3:size ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void scene_draw(void)
{
    draw_grid();
    draw_hud();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void sig_h(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void cleanup(void) { endwin(); }

static void calc_dims(int rows, int cols, int *mh, int *mw)
{
    *mh = (rows - HUD_ROWS - 1) / 2;
    *mw = (cols - 1) / 2;
    if (*mh > MAZE_H_MAX) *mh = MAZE_H_MAX;
    if (*mw > MAZE_W_MAX) *mw = MAZE_W_MAX;
    if (*mh < 2) *mh = 2;
    if (*mw < 2) *mw = 2;
}

static void handle_key(int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: g_quit = 1; break;
    case 'r': case 'R': maze_reset(); break;
    case 'p': case 'P': g_paused = !g_paused; break;
    case ' ':
        if (g_phase == PH_GENERATE) {
            while (gen_step()) {}
            solve_start();
        } else if (g_phase == PH_SOLVE) {
            while (solve_step()) {}
            g_phase = PH_DONE;
        }
        break;
    case '1': g_mh = 10;          g_mw = 40;          maze_reset(); break;
    case '2': calc_dims(g_rows, g_cols, &g_mh, &g_mw); maze_reset(); break;
    case '3': g_mh = MAZE_H_MAX;  g_mw = MAZE_W_MAX;  maze_reset(); break;
    default: break;
    }
}

static void step_simulation(void)
{
    if (g_paused) return;
    if (g_phase == PH_GENERATE) {
        for (int s = 0; s < GEN_STEPS; s++)
            if (!gen_step()) { solve_start(); break; }
    } else if (g_phase == PH_SOLVE) {
        for (int s = 0; s < SOL_STEPS; s++)
            if (!solve_step()) break;
    }
}

int main(void)
{
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT,  sig_h);
    signal(SIGTERM, sig_h);
    signal(SIGWINCH, sig_h);

    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();

    getmaxyx(stdscr, g_rows, g_cols);
    calc_dims(g_rows, g_cols, &g_mh, &g_mw);
    maze_reset();

    while (!g_quit) {
        long long frame_start = clock_ns();

        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_rows, g_cols);
            calc_dims(g_rows, g_cols, &g_mh, &g_mw);
            maze_reset();
        }

        int ch = getch();
        if (ch != ERR) handle_key(ch);

        step_simulation();

        erase();
        scene_draw();
        wnoutrefresh(stdscr);
        doupdate();

        /* Frame cap — `elapsed = clock_ns() - frame_start` (no +dt!). */
        long long elapsed = clock_ns() - frame_start;
        clock_sleep_ns(FRAME_NS - elapsed);
    }
    return 0;
}
