/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maze.c — builds a random maze, then finds the shortest way through it.
 *
 * A digger carves corridors out of a solid grid (leaving a yellow '@'
 * where it's working), then a flood spreads from the top-left corner
 * until it reaches the bottom-right, and the quickest route lights up
 * green.  Press r to start a fresh maze.
 *
 * Sister files: forest_fire.c (same idea, a wave of activity over a grid)
 *   and maze_backtracker.c (a fancier take on the same digger).
 * The maze idea and the two algorithms come from Jamis Buck's "Mazes for
 * Programmers" (2015) and the classic BFS/DFS chapters of CLRS.
 */

#define _POSIX_C_SOURCE 200809L
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config + types ─────────────────────────────────────────────── */

#define MAZE_W_MAX    90
#define MAZE_H_MAX    23
#define HUD_ROWS       2

#define TARGET_FPS    30
#define NS_PER_SEC    1000000000LL
#define FRAME_NS      (NS_PER_SEC / TARGET_FPS)

/* How much work each frame does.  Small numbers = slower, watchable animation. */
#define GEN_STEPS      4   /* digging steps per frame while building   */
#define SOL_STEPS     16   /* flood steps per frame while solving      */

/* Each cell remembers which of its four sides still has a wall, one bit each. */
#define WALL_N  1
#define WALL_E  2
#define WALL_S  4
#define WALL_W  8

#define N_DIRS  4   /* the four directions, numbered 0..3 = N, E, S, W */

/* Which stage the run is in.  It builds, then solves, then sits done. */
enum Phase { PH_GENERATE, PH_SOLVE, PH_DONE };

/* Names for the colour slots we draw with.  The six maze colours change with
 * the theme; HUD/HINT are pinned at 8/9 (a project-wide habit) so the status
 * line stays the same bright yellow/cyan no matter which theme is on. */
enum {
    PAIR_WALL = 1,   /* the walls and corners            */
    PAIR_VISIT,      /* a finished, empty corridor cell  */
    PAIR_FRONT,      /* '@', where the digger is now     */
    PAIR_BFS,        /* '.', cells the flood has reached */
    PAIR_PATH,       /* '*', the winning shortest route  */
    PAIR_UNVISIT,    /* '#', solid cell not yet dug      */
    PAIR_HUD   = 8,  /* status line, top-right           */
    PAIR_HINT  = 9   /* key hints, bottom-left           */
};

/*
 * Theme — one colour scheme: a colour for each of the six things we draw.
 * You flip between themes with t/T.  Every number here is a deliberately
 * bright xterm-256 colour: the dim end of the palette disappears against a
 * black terminal, so even the "background" cells stay on the visible half.
 */
typedef struct {
    const char *name;  /* shown in the status line, e.g. "CLASSIC" */
    short wall;        /* colour of the walls (+ - |)              */
    short visit;       /* a finished, empty corridor cell          */
    short front;       /* the digger '@'                           */
    short bfs;         /* the flood '.'                            */
    short path;        /* the winning route '*'                    */
    short unvisit;     /* solid, not-yet-dug cell '#'              */
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
 * Maze — the grid itself plus all the scratch space the two algorithms need:
 * the digger that builds it, and the flood that solves it.  A good maze has
 * exactly one route between any two cells and no loops; the digger guarantees
 * that, and because there are no loops the flood's route is the only one.
 *
 * The key trick: each cell stores its own four walls as bits, and the same
 * wall is recorded on BOTH cells that share it.  So "is the door open between
 * us?" gives the same answer whichever cell you ask — at the small cost that
 * opening a door means clearing the bit on both neighbours at once.
 */
typedef struct {
    /* ── the maze itself ── */
    int w, h;                                      /* current size in cells (at most MAZE_*_MAX) */
    unsigned char walls[MAZE_H_MAX][MAZE_W_MAX];   /* per-cell walls; bit set = that side is closed */
    enum Phase    phase;                           /* building, solving, or done             */

    /* ── digger (builds the maze) ── */
    unsigned char vis[MAZE_H_MAX][MAZE_W_MAX];     /* 1 once the digger has reached a cell    */
    /* the digger's trail of cells, so it can back up when it hits a dead end.
     * sized one bigger than the grid: a single snaking corridor can put every
     * cell on the trail at once before it ever backs up. */
    struct { int r, c; } stack[MAZE_H_MAX * MAZE_W_MAX + 1];
    int  stack_top;                                /* how deep the trail is; 0 means done building */

    /* ── flood (solves the maze; starts top-left, aims bottom-right) ── */
    int           parent [MAZE_H_MAX][MAZE_W_MAX]; /* which cell the flood came from, packed as r*MAZE_W_MAX+c; -1 = none */
    unsigned char bfs_vis[MAZE_H_MAX][MAZE_W_MAX]; /* 1 once the flood has reached a cell     */
    unsigned char on_path[MAZE_H_MAX][MAZE_W_MAX]; /* 1 if this cell is on the winning route  */
    struct { int r, c; } queue[MAZE_H_MAX * MAZE_W_MAX];  /* cells waiting for the flood to spread out of them */
    int  q_head, q_tail;                           /* the waiting line: take from head, add at tail */
} Maze;

/* Scene — everything the running program needs: the maze plus the few
 * display settings that wrap around it. */
typedef struct {
    Maze maze;        /* the maze being built and solved        */
    int  theme;       /* which colour scheme is on (index into THEMES) */
    bool paused;      /* true = freeze the animation            */
    int  rows, cols;  /* terminal size, in characters           */
} Scene;

static Scene g_scene;

/* ── §2  performance  (read the clock, sleep to hold a steady frame rate) ── */

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

/* ── §3  logic  (just figures things out — touches nothing else) ──────── */

/* Picks the biggest maze that fits the terminal once we set aside the status
 * line up top and the hint line at the bottom.  Each maze cell takes two
 * screen cells (one for the cell, one for its wall), so the maze can't be
 * more than about half the terminal.  Never smaller than 2x2, never bigger
 * than our fixed limits.  Hands the answer back through mh and mw. */
static void calc_dims(int rows, int cols, int *mh, int *mw)
{
    *mh = (rows - HUD_ROWS - 1) / 2;
    *mw = (cols - 1) / 2;
    if (*mh > MAZE_H_MAX) *mh = MAZE_H_MAX;
    if (*mw > MAZE_W_MAX) *mw = MAZE_W_MAX;
    if (*mh < 2) *mh = 2;
    if (*mw < 2) *mw = 2;
}

/* Is this cell actually inside the maze? */
static bool in_grid(const Maze *m, int r, int c)
{
    return r >= 0 && r < m->h && c >= 0 && c < m->w;
}

/* Is this the cell the digger is standing on right now? (drawn as '@') */
static bool is_carve_head(const Maze *m, int r, int c)
{
    return m->stack_top > 0 &&
           m->stack[m->stack_top - 1].r == r &&
           m->stack[m->stack_top - 1].c == c;
}

/* ── §4  simulation  (builds the maze, then solves it) ────────────────── */

/* Four lookup tables, one row each for N, E, S, W.  Given a direction you get
 * the row/col step to the neighbour (DR/DC), the wall on your side (D_WALL),
 * and the matching wall on the neighbour's side (D_OPP). */
static const int DR[N_DIRS]     = { -1,  0,  1,  0 };
static const int DC[N_DIRS]     = {  0,  1,  0, -1 };
static const int D_WALL[N_DIRS] = { WALL_N, WALL_E, WALL_S, WALL_W };
static const int D_OPP[N_DIRS]  = { WALL_S, WALL_W, WALL_N, WALL_E };

/* Shuffle the four directions into a random order.  The digger tries them in
 * this order, and that randomness is what makes every maze come out different. */
static void shuffle_dirs(int dirs[N_DIRS])
{
    for (int i = N_DIRS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = dirs[i]; dirs[i] = dirs[j]; dirs[j] = t;
    }
}

/* Open the door between this cell and its neighbour in direction d (clearing
 * the wall on both sides), then step the digger onto the neighbour. */
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
    memset(m->walls, 0x0F, sizeof m->walls); /* start solid: every cell walled in */
    memset(m->vis,   0,    sizeof m->vis);
    m->stack[0].r = 0;
    m->stack[0].c = 0;
    m->stack_top  = 1;
    m->vis[0][0]  = 1;
    m->phase      = PH_GENERATE;
}

/* One move of the digger: from where it stands, look around in a random order
 * for a neighbour it hasn't dug yet; step into the first one it finds.  If
 * every neighbour is already dug, back up one cell.  Returns false only once
 * the whole maze is built. */
static bool gen_step(Maze *m)
{
    if (m->stack_top == 0) return false;        /* trail empty → maze is finished */

    int r = m->stack[m->stack_top - 1].r;
    int c = m->stack[m->stack_top - 1].c;

    int dirs[N_DIRS] = { 0, 1, 2, 3 };
    shuffle_dirs(dirs);
    for (int k = 0; k < N_DIRS; k++) {
        int d  = dirs[k];
        int nr = r + DR[d], nc = c + DC[d];
        if (in_grid(m, nr, nc) && !m->vis[nr][nc]) {
            carve_into(m, r, c, d);             /* dig into the neighbour */
            return true;
        }
    }
    m->stack_top--;                             /* dead end → back up */
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

/* Once the flood reaches the goal, follow each cell's "came from" link back to
 * the start, marking every cell on that trail as part of the winning route. */
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

/* Spread the flood out of one cell: step through every open door to a cell the
 * flood hasn't reached, note that it arrived from here, and add it to the line. */
static void bfs_expand(Maze *m, int r, int c)
{
    for (int d = 0; d < N_DIRS; d++) {
        if (m->walls[r][c] & D_WALL[d]) continue;          /* door shut → can't go this way */
        int nr = r + DR[d], nc = c + DC[d];
        if (!in_grid(m, nr, nc) || m->bfs_vis[nr][nc]) continue;
        m->bfs_vis[nr][nc] = 1;
        m->parent [nr][nc] = r * MAZE_W_MAX + c;
        m->queue[m->q_tail].r = nr;
        m->queue[m->q_tail].c = nc;
        m->q_tail++;
    }
}

/* One step of the flood: take the next cell from the line, and either declare
 * victory (if it's the goal) or spread out from it. */
static bool solve_step(Maze *m)
{
    if (m->q_head >= m->q_tail) { m->phase = PH_DONE; return false; }  /* line empty, no route exists */

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

/* The one function that moves things forward each frame: do a few digging
 * steps while building (and kick off solving the moment building ends), or a
 * few flood steps while solving.  Does nothing while paused. */
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

/* ── §5  render  (draws the maze; only the screen changes here) ───────── */

/* Load a theme's colours into ncurses.  Every colour keeps the terminal's own
 * background (so the maze sits on whatever wallpaper you have), and the status
 * line stays fixed yellow/cyan no matter the theme.  Re-run on each t/T press.
 * On a plain 8-colour terminal we fall back to the nearest basic colours. */
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
        init_pair(PAIR_HUD,     226, -1);   /* status line: always bright yellow */
        init_pair(PAIR_HINT,     51, -1);   /* key hints:   always bright cyan   */
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

/* Put one coloured character on screen.  Everything drawn goes through here so
 * the "is it on screen?" check lives in one spot.  The double-cast on ch keeps
 * characters above 127 from getting mangled into garbage. */
static void mark_cell(int sr, int sc, char ch, int pair, int attr, int rows, int cols)
{
    if (sr < 0 || sr >= rows) return;
    if (sc < 0 || sc >= cols) return;
    chtype c = (chtype)(unsigned char)ch;
    if (pair) c |= (chtype)COLOR_PAIR(pair);
    if (attr) c |= (chtype)attr;
    mvaddch(sr, sc, c);
}

/* Each maze cell is drawn as a 2x2 patch of screen characters: the cell itself
 * plus the walls above and to its left.  Walking the screen row/col, even ones
 * land on the wall grid (corners '+', or '-'/'|' walls) and odd ones land on a
 * cell's interior.  This draws everything that ISN'T a cell interior — the
 * corners and the walls, showing a wall only where the cell still has it. */
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

/* Choose what to show inside a cell, based on what's happening to it: the
 * digger '@', the flood '.', the winning route '*', an empty corridor, or a
 * still-solid '#'.  What matters depends on whether we're building or solving. */
static void draw_cell_interior(const Maze *m, int r, int c, int sr, int sc,
                               int rows, int cols)
{
    if (m->phase == PH_GENERATE) {
        if (is_carve_head(m, r, c)) mark_cell(sr, sc, '@', PAIR_FRONT, A_BOLD, rows, cols);
        else if (m->vis[r][c])      mark_cell(sr, sc, ' ', PAIR_VISIT, 0, rows, cols);
        else                        mark_cell(sr, sc, '#', PAIR_UNVISIT, 0, rows, cols);
        return;
    }
    /* solving, or finished */
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

/* ── §6  app  (signals, keypresses, and the main loop) ────────────────── */

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

/* Set up the terminal for animation: read keys instantly without waiting,
 * don't echo them, and hide the blinking cursor. */
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
    case ' ':   /* skip the wait: finish whatever stage we're in right now */
        if (s->maze.phase == PH_GENERATE) {
            /* must finish building before solving — half a maze has no exit */
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

        /* window resized: re-measure, re-fit the maze, and start over */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, g_scene.rows, g_scene.cols);
            calc_dims(g_scene.rows, g_scene.cols, &g_scene.maze.h, &g_scene.maze.w);
            maze_reset(&g_scene.maze);
        }

        int ch = getch();
        if (ch != ERR) handle_key(&g_scene, ch);

        step_simulation(&g_scene);   /* move the build/solve forward */

        erase();
        scene_draw(&g_scene);
        wnoutrefresh(stdscr);
        doupdate();

        /* sleep off whatever time is left so every frame takes the same length */
        long long elapsed = clock_ns() - frame_start;
        clock_sleep_ns(FRAME_NS - elapsed);
    }
    return 0;
}
