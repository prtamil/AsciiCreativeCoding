/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * drunkards_walk_cave_showcase.c — a cave that carves itself.
 *
 * A few "drunkards" stagger randomly through a solid block of stone,
 * eating the cell they land on. Their overlapping wandering leaves
 * behind a winding, natural-looking cave — the trick roguelikes use
 * for caverns. Once enough stone is eaten, it pauses to show off the
 * result, then starts over.
 *
 * Sister file: bsp_dungeon_showcase.c — the opposite approach. BSP makes
 * sharp rectangular rooms; this makes blobby organic caves.
 * Recipe: RogueBasin, "Random Walk Cave Generation".
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

/* §1  config  —  all the tunable numbers, named once */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    /* How many drunkards carve at once. More walkers, more branches,
     * faster. 4 is a good default; not changeable from the keyboard. */
    WALKERS_MAX       =  16,
    WALKERS_DEF       =   4,

    /* Walkers start in a small scatter around the map centre. Starting
     * close together is what makes the cave come out as one connected
     * blob instead of separate islands. */
    SPAWN_JITTER_X    =   3,        /* ± columns */
    SPAWN_JITTER_Y    =   2,        /* ± rows    */

    /* After this many steps a walker teleports back to an already-carved
     * cell. Long enough to see it wander, short enough that it can't drift
     * off to a corner and get stranded there. */
    WALKER_MAX_AGE    = 200,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* How many cells each walker moves per tick. Bigger = faster carve.
     * 2 is slow enough to watch the comet trails. */
    STEPS_PER_TICK_MIN =   1,
    STEPS_PER_TICK_DEF =   2,
    STEPS_PER_TICK_MAX = 256,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,
    RENDER_FPS        =  60,        /* how often the screen repaints */
    MAX_FRAME_MS      = 100,        /* cap a frame's elapsed time so one slow frame
                                       can't make the sim try to catch up forever */

    /* ncurses colour-pair slots. HUD/HINT are reserved project-wide. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* '#' walls    */
    PAIR_FLOOR        =   4,        /* '.' floors   */
    PAIR_WALKER       =   5,        /* '@' walkers  */
    PAIR_TRAIL        =   6,        /* fresh-carve flash */
};

/* How fast the glow effects fade. Bigger number = quicker fade. */
#define CARVE_GLOW_DECAY    2.5f
#define WALKER_GLOW_DECAY   8.0f    /* fast, so only the head + a few cells glow */
#define GLOW_THRESHOLD      0.05f   /* below this a glow counts as gone */

#define HOLD_SECONDS        2.0f    /* how long to admire a finished cave */

/* Stop carving once this fraction of the map is floor. 0.45 looks like an
 * explorable cavern; much less is too sparse, much more is swiss cheese. */
#define FILL_RATIO          0.45f

/* A cell is either solid stone or carved-out floor. */
enum { TILE_WALL = 0, TILE_FLOOR = 1 };

/* The four directions a walker can step (delta helpers are in §4). */
enum { DIR_N = 0, DIR_E = 1, DIR_S = 2, DIR_W = 3 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — a named set of four colours for the cave. The trick: order them
 * dark-to-bright so the scene reads as layered depth — dim stone sits back,
 * the bright walker pops forward. The colours are xterm-256 indices, all kept
 * in the bright half so even the "darkest" one shows up on a black terminal.
 * Cycle them live with t/T. The HUD colours never change so the UI stays
 * readable whatever theme is on.
 */
typedef struct {
    const char *name;     /* shown in the HUD                  */
    short       wall;     /* solid stone — dimmest             */
    short       floor;    /* carved-out cells — a bit brighter */
    short       walker;   /* the '@' — brightest, draws the eye */
    short       trail;    /* the flash when a wall is just carved */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      wall floor walker trail */
    { "DEFAULT",  240,   67,  231,  220 },   /* grey / blue / white / gold   */
    { "MATRIX",    22,   34,  118,   46 },   /* dark green → bright green    */
    { "NOVA",      53,  129,  219,  201 },   /* purple → pink → magenta      */
    { "MONO",     234,  244,  254,  250 },   /* greyscale gradient           */
    { "OCEAN",     17,   33,   51,   39 },   /* navy → cyan                  */
    { "FIRE",      52,  124,  226,  196 },   /* dark red → yellow walker     */
    { "EARTH",     58,  137,  230,  173 },   /* brown → cream                */
    { "FOREST",    22,   64,  144,   28 },   /* dark green → tan walker      */
    { "DESERT",    94,  222,  230,  178 },   /* brown → sand                 */
    { "ARCTIC",    18,   39,  231,  159 },   /* navy → white walker          */
};

/* §2  timing  —  read the clock, sleep for a while */

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

/* §3  state  —  the structs that hold everything */

/*
 * Walker — one drunkard. Each tick it picks a random direction, steps one cell,
 * and carves whatever it lands on. A random walk in 2-D tends to keep returning
 * near where it started (Spitzer, "Principles of Random Walk"), which is exactly
 * why a clump of walkers carves one connected blob instead of wandering off into
 * separate tunnels.
 */
typedef struct {
    int x, y;   /* the cell it's standing on (0..w-1, 0..h-1) */
    int age;    /* steps taken since it last (re)spawned. Once this hits
                 * WALKER_MAX_AGE it teleports to a random floor cell and age
                 * resets — the leash that stops one walker drifting off to a
                 * corner and getting stuck there. */
} Walker;

/*
 * Cave — the map being carved, plus everything needed to carve it. The actual
 * output is `tiles`, the wall/floor grid (RogueBasin, "Random Walk Cave
 * Generation"). Everything else is bookkeeping: the walkers, a progress count,
 * and the cosmetic glow layers.
 *
 * The grid is stored as one flat array, not a 2-D one (index = y*w + x, see
 * cave_idx), so it clears and scans in a single pass. Everything is fixed-size
 * so the running loop never has to allocate memory. The cave is always
 * connected by construction — the walkers all start near the centre, so their
 * carved areas can't help but overlap.
 */
typedef struct {
    /* The grid itself */
    int     w, h;                  /* size used this run (≤ MAP_W/H_MAX)        */
    int     total_cells;           /* w*h, stored so scans skip the multiply    */
    uint8_t tiles[CELLS_MAX];      /* wall or floor, one byte per cell          */

    /* The drunkards doing the carving */
    Walker  walkers[WALKERS_MAX];  /* fixed pool; only the first n_walkers used */
    int     n_walkers;             /* more walkers = denser, branchier cave     */

    /* Progress toward "done" */
    int     floor_count;           /* cells carved so far (only goes up)        */
    int     target_floor;          /* carving stops when floor_count reaches it */

    /* Cosmetic glow layers — one value per cell, 0..1, painted bright then
     * faded each tick. Purely visual; the carving never reads them. */
    float   carve_glow [CELLS_MAX];   /* the flash when a wall is just carved   */
    float   walker_glow[CELLS_MAX];   /* the walker and its short comet tail    */
} Cave;

/*
 * Which phase of the forever-loop we're in. The demo carves a cave (WALKING),
 * pauses to show it off (HOLD), then resets and carves a new one — over and over.
 */
typedef enum {
    SCENE_WALKING = 0,   /* walkers still carving toward the target */
    SCENE_HOLD    = 1,   /* cave done; counting down before a reset */
} SceneState;

/*
 * Scene — the whole running demo in one place: the cave, the user's speed
 * settings, where we are in the loop, and which theme is on.
 */
typedef struct {
    Cave        cave;             /* the cave being carved */

    /* Two separate speed knobs. steps_per_tick is how many cells each walker
     * moves per tick; sim_fps is how many ticks happen per second. Splitting
     * them lets you slow the animation down without changing how chunky each
     * step is. */
    int         steps_per_tick;   /* doubles/halves on +/- */
    int         sim_fps;          /* changes on ]/[        */

    /* Where we are in the loop */
    SceneState  state;            /* WALKING or HOLD              */
    float       hold_timer;       /* seconds left before reset   */
    bool        paused;           /* freezes the sim, not the screen */

    int         theme;            /* which palette (index into themes[]) */
} Scene;

/*
 * Screen — the terminal's size in characters, as ncurses reports it. Re-read
 * whenever the window is resized; the cave is then re-sized to fit.
 */
typedef struct {
    int cols;   /* terminal width  */
    int rows;   /* terminal height */
} Screen;

/* The one running demo, plus its terminal and exit/resize flags. */
static Scene  g_scene;
static Screen g_screen;
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

/* §4  logic  —  small pure helpers: they only read, never change anything */

static inline int dir_dx(int d) { return (d == DIR_E) ? 1 : (d == DIR_W) ? -1 : 0; }
static inline int dir_dy(int d) { return (d == DIR_S) ? 1 : (d == DIR_N) ? -1 : 0; }

static inline int cave_idx(const Cave *c, int x, int y) { return y * c->w + x; }
static inline bool cave_in_bounds(const Cave *c, int x, int y)
{
    return x >= 0 && x < c->w && y >= 0 && y < c->h;
}

/*
 * True if any of the 8 surrounding cells is floor. We only draw a '#' for walls
 * that touch floor, so the solid stone deep inside stays blank and the result
 * looks like a cave rather than a filled-in rectangle.
 */
static bool tile_has_floor_neighbour(const Cave *c, int x, int y)
{
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (!cave_in_bounds(c, nx, ny)) continue;
            if (c->tiles[cave_idx(c, nx, ny)] == TILE_FLOOR) return true;
        }
    }
    return false;
}

/* §5  simulation  —  the only part that actually changes the cave */

/* Light a cell up fully. The comet tail you see is just this brightness fading
 * over the next few ticks. */
static void paint_walker_glow(Cave *c, int x, int y)
{
    c->walker_glow[cave_idx(c, x, y)] = 1.0f;
}

/* Dim every glow a little, once per tick. Tied to elapsed time (not frame
 * count) so glows fade at the same real-world speed no matter the frame rate. */
static void effects_decay(Cave *c, float dt)
{
    float carve_d  = expf(-CARVE_GLOW_DECAY  * dt);
    float walker_d = expf(-WALKER_GLOW_DECAY * dt);
    for (int i = 0; i < c->total_cells; i++) {
        c->carve_glow[i]  *= carve_d;
        c->walker_glow[i] *= walker_d;
    }
}

/* Pick one floor cell at random, giving every floor cell an equal chance in a
 * single pass over the grid. Returns its index, or -1 if nothing is carved yet. */
static int pick_random_floor_cell(const Cave *c)
{
    int chosen = -1, seen = 0;
    for (int i = 0; i < c->total_cells; i++) {
        if (c->tiles[i] == TILE_FLOOR) {
            seen++;
            if ((rand() % seen) == 0) chosen = i;
        }
    }
    return chosen;
}

/*
 * Carve one cell to floor. Only counts (and flashes) if it was stone before —
 * stepping onto an already-carved cell does nothing, which keeps floor_count
 * honest.
 */
static void cave_carve(Cave *c, int x, int y)
{
    int idx = cave_idx(c, x, y);
    if (c->tiles[idx] == TILE_WALL) {
        c->tiles[idx] = TILE_FLOOR;
        c->carve_glow[idx] = 1.0f;
        c->floor_count++;
    }
}

/*
 * Move a walker that's been wandering too long back to a random carved cell.
 * Keeps it inside the cave instead of stranded in a corner. There's always at
 * least one floor cell (spawn_walkers carves the starting cells), so this
 * always finds somewhere to go.
 */
static void cave_respawn_walker(Cave *c, Walker *w)
{
    int cell = pick_random_floor_cell(c);
    if (cell < 0) return;            /* can't really happen */
    w->x = cell % c->w;              /* turn the flat index back into (x, y) */
    w->y = cell / c->w;
    w->age = 0;
}

/* Move one walker one random step: carve the new cell, light it up, and
 * teleport it home if it's been wandering too long. */
static void cave_walker_step(Cave *c, Walker *w)
{
    int dir = rand() & 3;            /* low 2 bits pick one of the four directions */
    int nx  = w->x + dir_dx(dir);
    int ny  = w->y + dir_dy(dir);
    w->age++;

    if (cave_in_bounds(c, nx, ny)) {
        w->x = nx;
        w->y = ny;
        cave_carve(c, nx, ny);          /* eat the new cell if it was stone */
        paint_walker_glow(c, nx, ny);
    } else {
        /* would step off the edge — stay put, but keep glowing so it still
         * looks alive. Costs a tiny bias toward the edges, too small to see. */
        paint_walker_glow(c, w->x, w->y);
    }

    if (w->age >= WALKER_MAX_AGE)
        cave_respawn_walker(c, w);
}

/* Step every walker once. Returns false once the cave has enough floor — that's
 * the signal to stop carving. */
static bool cave_step(Cave *c)
{
    if (c->floor_count >= c->target_floor) {
        return false;
    }
    for (int i = 0; i < c->n_walkers; i++) {
        cave_walker_step(c, &c->walkers[i]);
    }
    return true;
}

/* Wipe the map back to solid stone with no glows — the starting blank slate. */
static void fill_with_walls(Cave *c)
{
    for (int i = 0; i < c->total_cells; i++) {
        c->tiles[i]       = TILE_WALL;
        c->carve_glow[i]  = 0.0f;
        c->walker_glow[i] = 0.0f;
    }
}

/*
 * Drop the walkers in a small scatter around the centre and carve where each
 * one lands. Starting them close together is what makes the finished cave one
 * connected piece.
 */
static void spawn_walkers(Cave *c)
{
    int cx = c->w / 2;
    int cy = c->h / 2;
    for (int i = 0; i < c->n_walkers; i++) {
        int wx = cx + (rand() % (2 * SPAWN_JITTER_X + 1)) - SPAWN_JITTER_X;
        int wy = cy + (rand() % (2 * SPAWN_JITTER_Y + 1)) - SPAWN_JITTER_Y;

        /* keep walkers off the outermost ring of cells */
        if (wx < 1) wx = 1;
        if (wy < 1) wy = 1;
        if (wx >= c->w - 1) wx = c->w - 2;
        if (wy >= c->h - 1) wy = c->h - 2;

        c->walkers[i].x = wx;
        c->walkers[i].y = wy;
        c->walkers[i].age = 0;
        cave_carve(c, wx, wy);
        paint_walker_glow(c, wx, wy);
    }
}

/* Start a brand-new cave: solid stone, walkers placed at the centre, ready to carve. */
static void cave_reset(Cave *c, int w, int h, int n_walkers)
{
    c->w = w;
    c->h = h;
    c->total_cells = w * h;
    c->target_floor = (int)((float)c->total_cells * FILL_RATIO);
    c->n_walkers = (n_walkers < 1) ? 1
                 : (n_walkers > WALKERS_MAX) ? WALKERS_MAX
                 : n_walkers;
    c->floor_count = 0;

    fill_with_walls(c);
    spawn_walkers(c);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    cave_reset(&s->cave, mw, mh, WALKERS_DEF);
    s->state      = SCENE_WALKING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused         = false;
    s->steps_per_tick = STEPS_PER_TICK_DEF;
    s->sim_fps        = SIM_FPS_DEFAULT;
    s->theme          = 0;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;                  /* frozen — change nothing */

    effects_decay(&s->cave, dt);            /* fade the glows a little */

    switch (s->state) {

    case SCENE_WALKING:
        for (int i = 0; i < s->steps_per_tick; i++) {
            if (!cave_step(&s->cave)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->cave.w, s->cave.h);
        }
        break;
    }
}

/* §6  effects  —  the glow layers (the code for them lives up in §5)
 *
 * The two glow buffers are in Cave; their code sits in §5 because the carving
 * code paints them and effects_decay fades them. Painted by cave_carve and
 * paint_walker_glow, faded each tick by effects_decay, read only by the
 * drawing code below to choose a cell's glyph and colour. */

/* §7  render  —  draw the cave and HUD; set up colours */

/*
 * Switch to one of the named palettes. Only touches the four cave colours; the
 * HUD colours stay put. Fine to call any time — the screen picks up the new
 * colours on the next repaint.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        init_pair(PAIR_WALL,   t->wall,   -1);
        init_pair(PAIR_FLOOR,  t->floor,  -1);
        init_pair(PAIR_WALKER, t->walker, -1);
        init_pair(PAIR_TRAIL,  t->trail,  -1);
    } else {
        /* old 8-colour terminal — themes all look the same here */
        init_pair(PAIR_WALL,   COLOR_WHITE,  -1);
        init_pair(PAIR_FLOOR,  COLOR_BLUE,   -1);
        init_pair(PAIR_WALKER, COLOR_WHITE,  -1);
        init_pair(PAIR_TRAIL,  COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
    }
    theme_apply(0);
}

/*
 * Draw a single cell, choosing its look by what matters most: a walker on it
 * wins, then a fresh-carve flash, then plain floor, then wall. Walls only show
 * up where they touch floor, so the untouched stone inside stays blank and the
 * whole thing reads as a cave.
 */
static void draw_cave_cell(const Cave *c, int x, int y, int sy, int sx)
{
    int idx = cave_idx(c, x, y);
    float wg = c->walker_glow[idx];
    float cg = c->carve_glow[idx];
    uint8_t k = c->tiles[idx];

    int  pair = 0, attr = A_NORMAL;
    char glyph = ' ';

    if (wg > GLOW_THRESHOLD) {
        pair = PAIR_WALKER; attr = A_BOLD;   glyph = '@';   /* walker + its tail */
    } else if (cg > GLOW_THRESHOLD) {
        pair = PAIR_TRAIL;  attr = A_BOLD;   glyph = '.';   /* just-carved flash */
    } else if (k == TILE_FLOOR) {
        pair = PAIR_FLOOR;  attr = A_DIM;    glyph = '.';   /* plain floor       */
    } else {
        /* a wall — only draw it if it borders some floor */
        if (tile_has_floor_neighbour(c, x, y)) {
            pair = PAIR_WALL; attr = A_NORMAL; glyph = '#'; /* visible wall face */
        } else {
            return;                                         /* deep stone — leave blank */
        }
    }

    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

static void cave_draw(const Cave *c, int cols, int rows)
{
    /* centre the map, leaving the 2 HUD rows on top and the hint row at the bottom */
    int gx0 = (cols - c->w) / 2;
    int gy0 = ((rows - 3) - c->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < c->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < c->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            draw_cave_cell(c, x, y, sy, sx);
        }
    }
}

/*
 * Draw the "#:wall .:floor @:walker" key on the HUD, each symbol in its actual
 * theme colour so it doubles as a colour swatch. Skipped if it wouldn't fit, so
 * it never spills onto the map.
 */
static void draw_glyph_legend(const Screen *sc)
{
    int lx = getcurx(stdscr);
    const char *legend = "  #:wall  .:floor  @:walker ";
    if (lx + (int)strlen(legend) <= sc->cols) {
        int p = lx + 2;                       /* small gap after the params */

        attron(COLOR_PAIR(PAIR_WALL));
        mvaddch(1, p++, '#');
        attroff(COLOR_PAIR(PAIR_WALL));
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, p, ":wall  ");  p += 7;
        attroff(COLOR_PAIR(PAIR_HUD));

        attron(COLOR_PAIR(PAIR_FLOOR) | A_DIM);
        mvaddch(1, p++, '.');
        attroff(COLOR_PAIR(PAIR_FLOOR) | A_DIM);
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, p, ":floor  ");  p += 8;
        attroff(COLOR_PAIR(PAIR_HUD));

        attron(COLOR_PAIR(PAIR_WALKER) | A_BOLD);
        mvaddch(1, p++, '@');
        attroff(COLOR_PAIR(PAIR_WALKER) | A_BOLD);
        attron(COLOR_PAIR(PAIR_HUD));
        mvprintw(1, p, ":walker");
        attroff(COLOR_PAIR(PAIR_HUD));
    }
}

static void screen_draw(const Screen *sc, const Scene *s, double fps)
{
    erase();
    cave_draw(&s->cave, sc->cols, sc->rows);

    const Cave *c = &s->cave;
    const char *state_str =
        s->paused                     ? "PAUSED " :
        (s->state == SCENE_WALKING)   ? "WALKING" :
                                        "HOLD   ";

    /* Row 0 right — fps, speed, state, progress. */
    char buf[HUD_COLS + 1];
    int pct = (c->target_floor > 0)
            ? (100 * c->floor_count / c->target_floor)
            : 0;
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  steps:%-3d  %s  %3d%%  %5d/%-5d ",
             fps, s->sim_fps, s->steps_per_tick, state_str,
             pct, c->floor_count, c->target_floor);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " DRUNKARD'S WALK CAVE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — theme name (bold) + walker count + map size. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " walkers:%-2d  age-cap:%-3d  fill-target:%d%%  map:%dx%d ",
             c->n_walkers, WALKER_MAX_AGE,
             (int)(FILL_RATIO * 100.0f), c->w, c->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Row 1 right — the glyph/colour key. */
    draw_glyph_legend(sc);

    /* Bottom row — the list of keys you can press. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* §8  platform  —  ncurses startup, window resize, signals */

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

static void on_exit_signal  (int sig) { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Restore the terminal on exit, and turn quit/resize signals into simple flags
 * the main loop checks (the handlers themselves do almost nothing, on purpose). */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* §9  app  —  keypresses, the main loop, main() */

/* Seed the random number generator from the clock so each run is different. */
static void seed_rng(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
}

/* Size the cave to fit the terminal, capped to the fixed buffer. */
static void pick_map_size(const Screen *scr, int *mw, int *mh)
{
    int w = scr->cols;
    int h = scr->rows - 3;     /* leave room for 2 HUD rows + 1 hint row */
    if (w < 16) w = 16;
    if (h < 8)  h = 8;
    if (w > MAP_W_MAX) w = MAP_W_MAX;
    if (h > MAP_H_MAX) h = MAP_H_MAX;
    *mw = w;
    *mh = h;
}

static void app_resize(Scene *s, Screen *scr)
{
    screen_resize(scr);
    int mw, mh;
    pick_map_size(scr, &mw, &mh);
    scene_reset(s, mw, mh);
    g_need_resize = 0;
}

static bool handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, s->cave.w, s->cave.h);
        break;
    case '=': case '+':
        if (s->steps_per_tick < STEPS_PER_TICK_MAX) s->steps_per_tick *= 2;
        if (s->steps_per_tick > STEPS_PER_TICK_MAX) s->steps_per_tick = STEPS_PER_TICK_MAX;
        break;
    case '-':
        s->steps_per_tick /= 2;
        if (s->steps_per_tick < STEPS_PER_TICK_MIN) s->steps_per_tick = STEPS_PER_TICK_MIN;
        break;
    case ']':
        s->sim_fps += SIM_FPS_STEP;
        if (s->sim_fps > SIM_FPS_MAX) s->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        s->sim_fps -= SIM_FPS_STEP;
        if (s->sim_fps < SIM_FPS_MIN) s->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        theme_apply(s->theme);
        break;
    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->theme);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    seed_rng();
    install_signals();

    screen_init(&g_screen);
    int mw, mh;
    pick_map_size(&g_screen, &mw, &mh);
    scene_init(&g_scene, mw, mh);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (g_running) {

        if (g_need_resize) {
            app_resize(&g_scene, &g_screen);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        int64_t tick_ns = TICK_NS(g_scene.sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&g_scene, dt_sec);
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

        /* hold a steady frame rate by sleeping off whatever time is left over */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / RENDER_FPS - elapsed);

        screen_draw(&g_screen, &g_scene, fps_display);
        screen_present();

        int ch = getch();
        if (ch != ERR && !handle_key(&g_scene, ch))
            g_running = 0;
    }

    screen_free(&g_screen);
    return 0;
}
