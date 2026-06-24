/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cellular_automata_cave_4-5_rule_showcase.c
 *   — Grows organic caves by smoothing random noise, animated as a sweep.
 *
 * Start with a grid of random specks (~45% walls). Repeatedly let each cell
 * vote with its neighbours: if 5+ of its 8 neighbours are walls it becomes a
 * wall, otherwise floor. A few passes of that turns the noise into rounded
 * caverns. The "4-5 rule" smoothing comes from Johnson, Yannakakis &
 * Togelius (2010) and the RogueBasin CA-caves tutorial.
 *
 * Sister file: ./drunkards_walk_cave_showcase.c — the other way to carve
 * organic caves (random walkers eating through rock instead of smoothing).
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
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    /* Smallest cave we allow, so a tiny terminal doesn't give a broken grid. */
    MAP_W_MIN         =  16,
    MAP_H_MIN         =   8,

    /* How many smoothing passes. 4 looks best; fewer is noisy, more melts
     * away the small features. */
    N_ITERATIONS      =   4,

    /* The rule's only knob: a cell turns to wall when at least this many of
     * its 8 neighbours are walls. Lower spreads walls, higher eats them. */
    WALL_THRESHOLD    =   5,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* How many cells we process each tick. 128 makes the whole 4-pass build
     * play out over a few seconds — slow enough to watch the noise settle. */
    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  = 128,
    OPS_PER_TICK_MAX  = 2048,

    FPS_UPDATE_MS     = 500,

    /* One HUD bar at the top, one at the bottom; the cave sits between them. */
    HUD_ROW_TOP       =   1,
    HUD_ROW_BOTTOM    =   1,

    RENDER_FPS_CAP    =  60,
    DT_CAP_MS         = 100,    /* ignore time beyond this in a frame, so a long
                                * stall can't snowball into runaway catch-up */

    /* Colour pairs. PAIR_HUD/PAIR_HINT are reserved by project convention. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,
    PAIR_FLOOR        =   4,
    PAIR_CHANGE       =   5,        /* cell that just flipped — bright accent */
    PAIR_SCAN         =   6,        /* unused; kept to match the sister theme table */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,        /* the bright reset flash */
};

/* How fast each glow fades. Bigger = quicker fade. */
#define CHANGE_GLOW_DECAY   3.0f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f   /* below this a glow is treated as gone */

#define HOLD_SECONDS        2.5f    /* how long we linger on the finished cave */

/* Fraction of cells that start as wall. 0.45 is the classic value. */
#define FILL_RATIO          0.45f

enum { TILE_FLOOR = 0, TILE_WALL = 1 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — one named colour set for the cave, swapped live with t/T.
 *
 * The cave shape never depends on colour, so a theme is just data we hand to
 * the terminal — switching it never touches the algorithm. The four colours
 * go from dim (resting stone) to bright (a freshly-changed cell). Each value
 * is an xterm-256 colour index (0..255); they're kept in the bright half of
 * the palette so even the darkest one stays visible (see CLAUDE.md / COLOR.md).
 */
typedef struct {
    const char *name;    /* shown in the HUD as "theme:NAME (i/N)"          */
    short       wall;    /* resting stone — the dimmest, most common colour */
    short       floor;   /* open cells — a step brighter than wall          */
    short       change;  /* a cell that just changed — bright accent        */
    short       scan;    /* unused here; kept so this table stays a drop-in
                          * match for drunkards_walk_cave_showcase.c        */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      wall floor change scan */
    { "DEFAULT",  240,   67,  226,  220 },   /* grey / blue / yellow / gold  */
    { "MATRIX",    22,   34,   46,  118 },   /* dark green → bright          */
    { "NOVA",      53,  129,  201,  219 },   /* purple → magenta accent      */
    { "MONO",     234,  244,  254,  250 },   /* greyscale gradient           */
    { "OCEAN",     17,   33,   51,   39 },   /* navy → cyan flash            */
    { "FIRE",      52,  124,  226,  196 },   /* dark red → yellow flash      */
    { "EARTH",     58,  137,  230,  173 },   /* brown → cream                */
    { "FOREST",    22,   64,  118,   28 },   /* dark green → bright accent   */
    { "DESERT",    94,  222,  230,  178 },   /* brown → sand                 */
    { "ARCTIC",    18,   39,  231,  159 },   /* navy → white flash           */
};

/* ── §2 clock ── */

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

/* ── §3 color ── */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        init_pair(PAIR_WALL,   t->wall,   -1);
        init_pair(PAIR_FLOOR,  t->floor,  -1);
        init_pair(PAIR_CHANGE, t->change, -1);
        init_pair(PAIR_SCAN,   t->scan,   -1);
    } else {
        init_pair(PAIR_WALL,   COLOR_WHITE,   -1);
        init_pair(PAIR_FLOOR,  COLOR_BLUE,    -1);
        init_pair(PAIR_CHANGE, COLOR_YELLOW,  -1);
        init_pair(PAIR_SCAN,   COLOR_CYAN,    -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_FLASH,      220, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_FLASH,     COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ── §5 model — the cave's data, the rule, the glow, and the sim ── */

/*
 * Grid — the cave's tiles, kept in two copies (a "double buffer").
 *
 * The rule for every cell looks at its neighbours, so all cells must read
 * the SAME old picture while we compute the new one. We read from read[] and
 * write the answers into write[], then copy write[] over read[] once the
 * whole pass is done. That way the order we visit cells in doesn't change the
 * result — every cell updates as if at the same instant. (We copy rather than
 * swap pointers because the renderer mixes both copies mid-sweep, so both
 * must stay valid.) See Johnson, Yannakakis & Togelius (2010); RogueBasin.
 */
typedef struct {
    int     w, h, n;               /* width, height, and n = w*h cells       */
    uint8_t read [CELLS_MAX];      /* the current picture — the rule reads it */
    uint8_t write[CELLS_MAX];      /* where the next picture is built         */
} Grid;

/*
 * Glow — a brightness value per cell, 1.0 when just lit and fading toward 0.
 *
 * Purely decorative: it's how a changed cell flashes and how the reset burst
 * is drawn. The rule never looks at it. The fade multiplies by a factor each
 * tick so it's the same speed regardless of frame rate, and never hits exact
 * zero, so the renderer ignores anything below GLOW_THRESHOLD.
 */
typedef struct {
    float v[CELLS_MAX];            /* per-cell brightness in [0,1] */
} Glow;

/*
 * Sweep — where we are in the build: which pass, and how far across it.
 *
 * The algorithm doesn't care what order we visit cells. We walk them left to
 * right, top to bottom purely so the viewer sees a line crawl across the map
 * and watches the smoothing happen instead of it popping into place.
 */
typedef struct {
    int  iteration;                /* which smoothing pass, 0..N_ITERATIONS  */
    int  idx;                      /* cell reached within this pass, 0..n    */
    bool done;                     /* true once all passes have run          */
} Sweep;

/*
 * Cave — everything being simulated, in one place: the tiles, the two glow
 * layers, the sweep cursor, and a running wall count for the HUD.
 */
typedef struct {
    Grid  grid;          /* the tiles (double buffer)                       */
    Glow  flip;          /* flash on a cell that just changed               */
    Glow  nova;          /* the bright burst lit on every reset             */
    Sweep sweep;         /* how far the build has got                       */
    int   wall_count;    /* walls in the picture as currently shown; kept up
                          * to date as cells change so the HUD's walls% is
                          * right even mid-sweep                            */
} Cave;

/* idx of a cell; commit copies the freshly-built picture over the old one. */
static inline int  grid_idx   (const Grid *g, int x, int y) { return y * g->w + x; }
static inline void grid_commit(Grid *g) { memcpy(g->read, g->write, g->n); }

/* Small helpers shared by both glow layers. */
static inline void  glow_light(Glow *f, int idx)       { f->v[idx] = 1.0f; }
static inline float glow_at   (const Glow *f, int idx) { return f->v[idx]; }
static void glow_fill (Glow *f, int n, float value) { for (int i = 0; i < n; i++) f->v[i] = value; }
static void glow_decay(Glow *f, int n, float rate, float dt)
{
    float k = expf(-rate * dt);
    for (int i = 0; i < n; i++) f->v[i] *= k;
}

/* ── §5 logic — the rule itself, reads only, changes nothing ── */

/* Count how many of (x,y)'s 8 surrounding cells are walls in the current
 * picture. Cells off the edge of the grid count as wall, which keeps a solid
 * border so the cave never opens onto nothing. */
static int moore_wall_count(const Grid *g, int x, int y)
{
    int n = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h)
                n++;                              /* off the edge counts as wall */
            else if (g->read[grid_idx(g, nx, ny)] == TILE_WALL)
                n++;
        }
    }
    return n;
}

/*
 * The whole algorithm, in one line: a cell becomes wall if walls are the
 * local majority (5+ of its 8 neighbours), otherwise floor.
 *
 * The textbook "4-5 rule" actually uses an easier bar for staying a wall (4)
 * than for becoming one (5). This file uses a single 5-or-more test for both,
 * ignoring the cell's own kind — simpler, and what these visuals were tuned to.
 */
static inline uint8_t ca_rule(int wall_neighbours)
{
    return (wall_neighbours >= WALL_THRESHOLD) ? TILE_WALL : TILE_FLOOR;
}

/* ── §5 effects — the flashes; these only ever write the glow layers ── */

/* Flash one cell (called when a cell changes kind). */
static inline void cave_flash_flip(Cave *cave, int idx) { glow_light(&cave->flip, idx); }

/* Light the whole-screen reset burst, and clear any leftover change flashes. */
static void cave_flash_supernova(Cave *cave)
{
    glow_fill(&cave->flip, cave->grid.n, 0.0f);
    glow_fill(&cave->nova, cave->grid.n, 1.0f);
}

/* Fade both glow layers by one tick, each at its own speed. */
static void cave_decay_glow(Cave *cave, float dt)
{
    glow_decay(&cave->flip, cave->grid.n, CHANGE_GLOW_DECAY, dt);
    glow_decay(&cave->nova, cave->grid.n, SUPERNOVA_DECAY,   dt);
}

/* ── §5 simulation — advances the cave (changes grid, sweep, count) ── */

/* True FILL_RATIO of the time. Each call is an independent coin flip — this
 * is the random speckle the smoothing rule later turns into caverns. */
static inline bool roll_wall(void)
{
    return ((float)rand() / (float)RAND_MAX) < FILL_RATIO;
}

/*
 * Start a fresh cave: random speckle fill, sweep and wall count back to zero.
 * Touches only the tiles/sweep/count, not the glow — the caller fires the
 * reset burst separately, so the two stay independent.
 */
static void cave_seed(Cave *cave, int w, int h)
{
    Grid *g = &cave->grid;
    g->w = w;
    g->h = h;
    g->n = w * h;

    cave->sweep.iteration = 0;
    cave->sweep.idx       = 0;
    cave->sweep.done      = false;
    cave->wall_count      = 0;

    for (int i = 0; i < g->n; i++) {
        bool wall = roll_wall();
        g->read[i]  = wall ? TILE_WALL : TILE_FLOOR;
        g->write[i] = g->read[i];
        if (wall) cave->wall_count++;
    }
}

/*
 * Do one small step of work and report whether anything was left to do.
 * A step is either: process one cell (decide its new kind, update the wall
 * count, flash it if it changed, move the cursor on), or, at the end of a
 * pass, lock in the new picture and start the next pass. The main loop calls
 * this many times per tick to control how fast the build plays out.
 */
static bool cave_advance(Cave *cave)
{
    Grid  *g  = &cave->grid;
    Sweep *sw = &cave->sweep;

    if (sw->done) return false;

    /* End of a pass: lock in the new picture and begin the next one. */
    if (sw->idx >= g->n) {
        grid_commit(g);
        sw->iteration++;
        sw->idx = 0;
        if (sw->iteration >= N_ITERATIONS) sw->done = true;
        return !sw->done;
    }

    int x = sw->idx % g->w;
    int y = sw->idx / g->w;

    uint8_t was     = g->read[sw->idx];
    uint8_t becomes = ca_rule(moore_wall_count(g, x, y));
    g->write[sw->idx] = becomes;

    if (becomes != was) {
        /* Keep the count matched to what's actually on screen (new picture
         * for cells already done, old for the rest) so walls% reads right
         * even part-way through a pass. */
        cave->wall_count += (becomes == TILE_WALL) ? +1 : -1;
        cave_flash_flip(cave, sw->idx);
    }
    sw->idx++;
    return true;
}

/* ── §6 scene — where the pieces come together ── */

/* The two phases we alternate between: building a cave, then resting on it
 * before starting over (building is finite, so we can't just run forever). */
typedef enum {
    SCENE_ITERATING = 0,  /* building the cave */
    SCENE_HOLD      = 1,  /* finished — rest on it, then start a new one */
} SceneState;

/*
 * Control — everything the keyboard can change. Kept apart from the Cave to
 * make the point clear: these knobs change the pace and colours of the show,
 * never the cave that comes out. Each one's limits live in §1.
 */
typedef struct {
    bool paused;          /* space — freeze everything                      */
    int  ops_per_tick;    /* +/-   — cells processed per tick (build speed)  */
    int  sim_fps;         /* [ ]   — how many ticks per second              */
    int  theme;           /* t/T   — which colour set                       */
} Control;

/* Scene — the whole show in one bundle: what's simulated, how the user drives
 * it, and where we are in the build/rest cycle. */
typedef struct {
    Cave        cave;        /* what's being simulated     */
    Control     ctrl;        /* the user's knobs           */
    SceneState  state;       /* building or resting        */
    float       hold_timer;  /* seconds left to rest before a re-seed */
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    cave_seed(&s->cave, mw, mh);
    cave_flash_supernova(&s->cave);     /* light the reset burst over everything */
    s->state      = SCENE_ITERATING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused       = false;
    s->ctrl.ops_per_tick = OPS_PER_TICK_DEF;
    s->ctrl.sim_fps      = SIM_FPS_DEFAULT;
    s->ctrl.theme        = 0;
    scene_reset(s, mw, mh);
}

/*
 * One step of the whole show, in order: skip if paused, fade the glows, then
 * either build a bit more of the cave or count down the rest. This is the only
 * place that changes anything; drawing only reads. dt is one tick's worth of time.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    cave_decay_glow(&s->cave, dt);

    switch (s->state) {

    case SCENE_ITERATING:
        /* Only do a fixed chunk of cells per tick, so the build takes a few
         * seconds to watch instead of finishing in one frame. */
        for (int i = 0; i < s->ctrl.ops_per_tick; i++) {
            if (!cave_advance(&s->cave)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->cave.grid.w, s->cave.grid.h);
        }
        break;
    }
}

/* ── §7 render — turn the cave into characters on screen ── */

/* The terminal's size, remembered from the last query. We only re-read it on
 * a resize, and every frame needs it to centre the map and pin the HUD bars. */
typedef struct {
    int cols;   /* width  in characters */
    int rows;   /* height in characters */
} Screen;

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
 * Which tile to show at a cell: the new value if the sweep has already passed
 * it, otherwise the old one. Mixing the two is what makes the moving boundary
 * you see crawl across the map. Once a pass finishes both are equal anyway.
 */
static inline uint8_t cave_view_tile(const Cave *cave, int idx)
{
    return (idx < cave->sweep.idx) ? cave->grid.write[idx] : cave->grid.read[idx];
}

/* Does this cell touch any floor (as currently shown)? A wall only draws a '#'
 * face if it borders open space; walls buried in rock are left blank. */
static bool cave_view_has_floor_neighbour(const Cave *cave, int x, int y)
{
    const Grid *g = &cave->grid;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;
            if (cave_view_tile(cave, grid_idx(g, nx, ny)) == TILE_FLOOR)
                return true;
        }
    }
    return false;
}

/*
 * CellLook — how one cell should look: its colour, style, character, and
 * whether to draw it at all. Working out the look (cell_look) is kept apart
 * from stamping it on screen (scene_draw) so each stays simple.
 */
typedef struct {
    short pair;    /* which colour pair                              */
    int   attr;    /* bold / dim / normal                            */
    char  glyph;   /* the character: '#' '.' '*' or ' '              */
    bool  draw;    /* false means skip it and leave the cell blank   */
} CellLook;

/* Pick a cell's look. Order of priority: reset burst, then change flash, then
 * floor, then a wall face. A wall with no floor next to it is left blank. */
static CellLook cell_look(const Cave *cave, int x, int y)
{
    int     idx = grid_idx(&cave->grid, x, y);
    uint8_t k   = cave_view_tile(cave, idx);

    if (glow_at(&cave->nova, idx) > GLOW_THRESHOLD)
        return (CellLook){ PAIR_SUPERNOVA, A_BOLD, '*', true };

    if (glow_at(&cave->flip, idx) > GLOW_THRESHOLD)
        return (CellLook){ PAIR_CHANGE, A_BOLD, (k == TILE_FLOOR) ? '.' : '#', true };

    if (k == TILE_FLOOR)
        return (CellLook){ PAIR_FLOOR, A_DIM, '.', true };

    if (cave_view_has_floor_neighbour(cave, x, y))
        return (CellLook){ PAIR_WALL, A_NORMAL, '#', true };

    return (CellLook){ 0, A_NORMAL, ' ', false };   /* buried rock — leave blank */
}

/* Centre the grid on screen and stamp each cell. All the look decisions are in
 * cell_look; this just places characters. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Cave *cave = &s->cave;
    const Grid *g    = &cave->grid;

    /* Centre the grid in the band between the two HUD bars. */
    int draw_rows = rows - HUD_ROW_TOP - HUD_ROW_BOTTOM;
    int gx0 = (cols - g->w) / 2;
    int gy0 = HUD_ROW_TOP + (draw_rows - g->h) / 2;
    if (gx0 < 0)           gx0 = 0;
    if (gy0 < HUD_ROW_TOP) gy0 = HUD_ROW_TOP;

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            CellLook look = cell_look(cave, x, y);
            if (!look.draw) continue;

            attron(COLOR_PAIR(look.pair) | look.attr);
            mvaddch(sy, sx, (chtype)(unsigned char)look.glyph);
            attroff(COLOR_PAIR(look.pair) | look.attr);
        }
    }
}

/* Paint one full-width status bar. We write the text clipped to the screen
 * width so an over-long line can't wrap down onto the cave. Used for both bars. */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    if (row < 0 || cols < 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Cave    *cave = &s->cave;
    const Control *ctrl = &s->ctrl;
    int n = cave->grid.n;
    const char *state_str =
        ctrl->paused                  ? "PAUSED"    :
        (s->state == SCENE_ITERATING) ? "ITERATING" :
                                        "HOLD";
    int sweep_pct = (n > 0) ? (100 * cave->sweep.idx  / n) : 0;
    int wall_pct  = (n > 0) ? (100 * cave->wall_count / n) : 0;

    /* Top bar: name, current state, and all the live readouts on one line. */
    char data[200];
    snprintf(data, sizeof data,
             " CA_CAVE B5678/S45678  %-9s  theme:%s (%d/%d)  iter:%d/%d  "
             "sweep:%3d%%  walls:%3d%%  ops:%-4d  %5.1f fps  %3d Hz ",
             state_str, themes[ctrl->theme].name,
             ctrl->theme + 1, N_THEMES,
             cave->sweep.iteration, N_ITERATIONS,
             sweep_pct, wall_pct, ctrl->ops_per_tick, fps, ctrl->sim_fps);

    /* Bottom bar: just the keys you can press. */
    static const char *keys =
        " q:quit  spc:pause  r:reset  t/T:theme  +/-:speed  [/]:Hz ";

    hud_bar(0,            sc->cols, PAIR_HUD,  data);
    hud_bar(sc->rows - 1, sc->cols, PAIR_HINT, keys);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app — the main loop, signals, and input ── */

/*
 * App — the whole program in one bundle: the show, the screen size, the cave
 * size we picked to fit it, and two flags the signal handlers flip. It's the
 * one global because signal handlers get no other way to reach the program;
 * everything else is passed by pointer.
 */
typedef struct {
    Scene                 scene;          /* the show                         */
    Screen                screen;         /* terminal size                    */
    int                   map_w, map_h;   /* cave size chosen to fit the screen */
    volatile sig_atomic_t running;        /* set to 0 to quit (by SIGINT/TERM) */
    volatile sig_atomic_t need_resize;    /* set to 1 by SIGWINCH; re-read size */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    /* Cave fills the width, and the height left after the two HUD rows. */
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_ROW_TOP - HUD_ROW_BOTTOM;
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Control *ctrl = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     ctrl->paused = !ctrl->paused; break;
    case 'r': case 'R':
        scene_reset(&app->scene, app->map_w, app->map_h);
        break;
    case '=': case '+':
        ctrl->ops_per_tick *= 2;
        if (ctrl->ops_per_tick > OPS_PER_TICK_MAX) ctrl->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        ctrl->ops_per_tick /= 2;
        if (ctrl->ops_per_tick < OPS_PER_TICK_MIN) ctrl->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        ctrl->sim_fps += SIM_FPS_STEP;
        if (ctrl->sim_fps > SIM_FPS_MAX) ctrl->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        ctrl->sim_fps -= SIM_FPS_STEP;
        if (ctrl->sim_fps < SIM_FPS_MIN) ctrl->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        ctrl->theme = (ctrl->theme + 1) % N_THEMES;
        theme_apply(ctrl->theme);
        break;
    case 'T':
        ctrl->theme = (ctrl->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(ctrl->theme);
        break;

    default: break;
    }
    return true;
}

/*
 * Run the simulation in fixed-size ticks. We add this frame's elapsed time to
 * a running balance, then spend it one whole tick at a time, carrying the
 * leftover to next frame. This way the cave advances the same per real second
 * whatever the frame rate.
 */
static void app_step_simulation(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->scene.ctrl.sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Sleep off whatever time is left in this frame's budget, so we hold a steady
 * frame rate instead of running flat out and pinning the CPU. */
static void app_pace_frame(int64_t frame_start, int64_t frame_dt)
{
    int64_t budget_ns  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed_ns = clock_ns() - frame_start + frame_dt;
    clock_sleep_ns(budget_ns - elapsed_ns);
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

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* 1. Adopt a new terminal size if SIGWINCH asked for one. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 2. Measure how long since last frame, capped so one long stall
         *    can't pile up into endless catch-up work. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;

        /* 3. Advance the simulation by that much real time. */
        app_step_simulation(app, dt, &sim_accum);

        /* 4. Update the FPS readout once per FPS_UPDATE_MS window. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. Sleep to the frame cap, THEN draw (steady pacing). */
        app_pace_frame(frame_time, dt);
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        /* 6. Drain one input event. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
