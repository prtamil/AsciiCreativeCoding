/*
 * snowflake.c  —  DLA (Diffusion-Limited Aggregation) fractal crystal
 *
 * Random walkers are released from the terminal edges.  When a walker
 * lands adjacent to the frozen crystal it sticks with probability
 * STICK_PROB, extending the aggregate by one cell.  The result is a
 * fractal cluster with Hausdorff dimension ≈ 1.71 — the classic
 * snowflake / ice-crystal morphology.
 *
 * STICK_PROB = 1.0  →  sparse, spiky arms (classic DLA)
 * STICK_PROB < 0.7  →  dense, rounded cluster (Eden model limit)
 *
 * The simulation lives entirely in cell space — there are no pixel
 * coordinates.  The grid is rows × cols cells; one cell = one character.
 * No aspect-ratio conversion is needed because the physics is the grid.
 *
 * Color encoding:
 *   Frozen cells are stamped with a band index at freeze time:
 *     band = (frozen_count / BAND_SIZE) % N_ICE_COLORS
 *   Band 0 (newest) → bright white tips.
 *   Band 5 (oldest) → dark navy core.
 *   This produces concentric growth rings visible as the crystal grows.
 *
 * Keys:
 *   q / ESC   quit
 *   r         reset crystal and all walkers
 *   + =       more simultaneous walkers
 *   -         fewer walkers
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra snowflake.c -o snowflake -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid
 *   §5  walker
 *   §6  scene
 *   §7  screen
 *   §8  app
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    WALKER_MIN      =  10,
    WALKER_DEFAULT  = 120,
    WALKER_MAX      = 400,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    N_ICE_COLORS    =   6,
    BAND_SIZE       =  40,   /* frozen cells per color band             */

    HUD_COLS        =  46,
    FPS_UPDATE_MS   = 500,
};

/*
 * STICK_PROB — probability a walker sticks on first contact.
 * Range [0, 1].  Increase toward 1 for sparser, more fractal arms.
 */
#define STICK_PROB   0.90f

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

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

/*
 * Ice crystal palette — six bands, newest (tips) to oldest (core).
 *
 * Band 1 (newest) = bright white
 * Band 6 (oldest) = dark navy
 *
 * 256-color uses xterm colour indices for maximum vividness.
 * 8-color fallback preserves the light → dark gradient.
 */
typedef enum {
    COL_ICE_1  = 1,   /* newest tips — bright white                   */
    COL_ICE_2  = 2,   /* bright cyan                                  */
    COL_ICE_3  = 3,   /* sky blue                                     */
    COL_ICE_4  = 4,   /* cornflower blue                              */
    COL_ICE_5  = 5,   /* royal blue                                   */
    COL_ICE_6  = 6,   /* oldest core — deep navy                      */
    COL_WALKER = 7,   /* dim dots for drifting walkers                */
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_ICE_1,  231, COLOR_BLACK);   /* white           */
        init_pair(COL_ICE_2,   51, COLOR_BLACK);   /* bright cyan     */
        init_pair(COL_ICE_3,  123, COLOR_BLACK);   /* light sky blue  */
        init_pair(COL_ICE_4,   39, COLOR_BLACK);   /* cornflower      */
        init_pair(COL_ICE_5,   27, COLOR_BLACK);   /* royal blue      */
        init_pair(COL_ICE_6,   18, COLOR_BLACK);   /* dark navy       */
        init_pair(COL_WALKER, 238, COLOR_BLACK);   /* dark grey       */
    } else {
        init_pair(COL_ICE_1,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_ICE_2,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_ICE_3,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_ICE_4,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_ICE_5,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_ICE_6,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_WALKER, COLOR_WHITE,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Grid — the frozen crystal aggregate.
 *
 *   cells[y][x] == 0   →  empty
 *   cells[y][x] == n   →  frozen, draw with COLOR_PAIR(n)
 *
 * Colour is assigned at freeze time based on frozen_count / BAND_SIZE,
 * cycling COL_ICE_1..COL_ICE_6 repeatedly.  As growth proceeds the
 * newest cells always carry the lowest band (bright) and old cells
 * carry higher bands (dark), producing visible growth rings.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     frozen_count;
    int     rows;
    int     cols;
} Grid;

static void grid_init(Grid *g, int cols, int rows)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->frozen_count = 0;
    g->cols         = cols;
    g->rows         = rows;

    /* Seed: freeze the center cell as band 0 (COL_ICE_1, bright white) */
    g->cells[rows / 2][cols / 2] = COL_ICE_1;
    g->frozen_count              = 1;
}

static void grid_freeze(Grid *g, int cx, int cy)
{
    int band = (g->frozen_count / BAND_SIZE) % N_ICE_COLORS;
    g->cells[cy][cx] = (uint8_t)(band + 1);
    g->frozen_count++;
}

static bool grid_frozen(const Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return false;
    return g->cells[cy][cx] != 0;
}

/*
 * grid_adjacent_frozen() — true if any cardinal neighbour of (cx,cy)
 * is frozen.  This is the DLA contact test.
 */
static bool grid_adjacent_frozen(const Grid *g, int cx, int cy)
{
    return grid_frozen(g, cx - 1, cy)
        || grid_frozen(g, cx + 1, cy)
        || grid_frozen(g, cx,     cy - 1)
        || grid_frozen(g, cx,     cy + 1);
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t col = g->cells[cy][cx];
            if (col == 0) continue;
            attr_t attr = COLOR_PAIR((int)col);
            if      (col <= 2) attr |= A_BOLD;
            else if (col >= 5) attr |= A_DIM;
            wattron(w, attr);
            mvwaddch(w, cy, cx, (chtype)'*');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  walker                                                             */
/* ===================================================================== */

/*
 * Walker — one randomly diffusing particle.
 *
 * Position (cx, cy) is in cell coordinates.  One step per tick in a
 * random cardinal direction.  When the walker tries to step into or
 * onto a cell adjacent to a frozen cell, it tests STICK_PROB:
 *
 *   — hit frozen cell directly → test sticking from current position
 *   — land adjacent to frozen  → test sticking at new position
 *
 * If it sticks, grid_freeze() is called and the walker is respawned.
 * If it drifts out of bounds, it is immediately respawned on a random edge.
 */
typedef struct {
    int  cx, cy;
    bool active;
} Walker;

static void walker_spawn(Walker *w, int cols, int rows)
{
    int edge = rand() % 4;
    switch (edge) {
    case 0: w->cx = rand() % cols;    w->cy = 0;          break;
    case 1: w->cx = rand() % cols;    w->cy = rows - 1;   break;
    case 2: w->cx = 0;                w->cy = rand() % rows; break;
    default: w->cx = cols - 1;        w->cy = rand() % rows; break;
    }
    w->active = true;
}

/*
 * walker_tick() — advance one walker one step.
 *
 * Returns true when the walker froze a cell (so the caller can respawn it).
 */
static bool walker_tick(Walker *w, Grid *g)
{
    if (!w->active) return false;

    static const int ddx[4] = { 0,  0, -1, 1 };
    static const int ddy[4] = {-1,  1,  0, 0 };
    int dir = rand() % 4;
    int nx  = w->cx + ddx[dir];
    int ny  = w->cy + ddy[dir];

    /* Out of bounds — respawn without testing sticking */
    if (nx < 0 || nx >= g->cols || ny < 0 || ny >= g->rows) {
        walker_spawn(w, g->cols, g->rows);
        return false;
    }

    if (grid_frozen(g, nx, ny)) {
        /*
         * Tried to step into a frozen cell — walker is adjacent.
         * Test sticking from current position (cx, cy).
         */
        if (!grid_frozen(g, w->cx, w->cy)) {
            if ((float)rand() / (float)RAND_MAX < STICK_PROB) {
                grid_freeze(g, w->cx, w->cy);
                return true;
            }
        }
        /* Didn't stick — stay in place this tick */
        return false;
    }

    /* Move to new cell */
    w->cx = nx;
    w->cy = ny;

    /* Test sticking at new position */
    if (grid_adjacent_frozen(g, w->cx, w->cy)) {
        if ((float)rand() / (float)RAND_MAX < STICK_PROB) {
            grid_freeze(g, w->cx, w->cy);
            return true;
        }
    }

    return false;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid   grid;
    Walker walkers[WALKER_MAX];
    int    n_walkers;
    bool   paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    grid_init(&s->grid, cols, rows);
    s->n_walkers = WALKER_DEFAULT;
    s->paused    = false;

    for (int i = 0; i < WALKER_MAX; i++)
        s->walkers[i].active = false;

    for (int i = 0; i < s->n_walkers; i++)
        walker_spawn(&s->walkers[i], cols, rows);
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;

    for (int i = 0; i < s->n_walkers; i++) {
        bool froze = walker_tick(&s->walkers[i], &s->grid);
        if (froze)
            walker_spawn(&s->walkers[i], s->grid.cols, s->grid.rows);
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_draw(&s->grid, w);

    /* Walkers: dim dots, only on empty cells */
    wattron(w, COLOR_PAIR(COL_WALKER) | A_DIM);
    for (int i = 0; i < s->n_walkers; i++) {
        const Walker *wk = &s->walkers[i];
        if (!wk->active) continue;
        if (wk->cy < 0 || wk->cy >= s->grid.rows) continue;
        if (wk->cx < 0 || wk->cx >= s->grid.cols) continue;
        if (s->grid.cells[wk->cy][wk->cx] == 0)
            mvwaddch(w, wk->cy, wk->cx, (chtype)'.');
    }
    wattroff(w, COLOR_PAIR(COL_WALKER) | A_DIM);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols;
    int rows;
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

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  frozen:%-5d  walkers:%-3d  spd:%d",
             fps, sc->grid.frozen_count, sc->n_walkers, sim_fps);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(COL_ICE_2) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_ICE_2) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R':
        scene_init(&app->scene, app->screen.cols, app->screen.rows);
        break;

    case 'p': case 'P': case ' ':
        app->scene.paused = !app->scene.paused;
        break;

    case '=': case '+':
        if (app->scene.n_walkers < WALKER_MAX) {
            int i = app->scene.n_walkers++;
            walker_spawn(&app->scene.walkers[i],
                         app->screen.cols, app->screen.rows);
        }
        break;

    case '-':
        if (app->scene.n_walkers > WALKER_MIN)
            app->scene.n_walkers--;
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
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

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

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene);
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
