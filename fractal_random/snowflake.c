/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * snowflake.c  —  DLA fractal crystal with D6 (6-fold) symmetry
 *
 * Random walkers drift from terminal edges.  When a walker sticks to the
 * frozen aggregate, all 12 positions in the dihedral-6 symmetry group are
 * frozen simultaneously — 6 rotations × 2 reflections.  This forces the
 * classic 6-armed hexagonal snowflake morphology.
 *
 * Color is assigned by Euclidean distance from the centre:
 *   near centre  →  deep navy    (COL_ICE_6)
 *   near tips    →  bright white (COL_ICE_1)
 *
 * Characters are chosen from the frozen-neighbour topology:
 *   *  isolated tip / no neighbours
 *   -  horizontal arm segment
 *   |  vertical arm segment
 *   +  junction / branch point
 *   /  \ diagonal struts
 *
 * Keys:
 *   q / ESC   quit
 *   r         reset
 *   + =       more walkers
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

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : DLA with 6-fold (D6) symmetry constraint.
 *                  Each walker's trajectory is computed only for one of 6
 *                  symmetric sectors.  When it sticks, 5 mirror copies are
 *                  placed simultaneously.  This enforces hexagonal symmetry
 *                  throughout the growth, unlike diffusion_map.c which grows
 *                  freely in any direction.
 *
 * Physics        : Real snow crystal formation: water vapour condenses on a
 *                  dust nucleation site and diffuses outward.  The hexagonal
 *                  symmetry comes from the molecular geometry of ice (H₂O ice Ih).
 *                  DLA with D6 symmetry captures this growth morphology — the
 *                  branching and tip-screening effects produce the characteristic
 *                  dendritic snowflake arms.
 *
 * Math           : D6 symmetry group: 6 rotations × (π/3 each) + 6 reflections.
 *                  The aggregate remains invariant under all 12 symmetry operations.
 *                  Fractal dimension of DLA in 2D ≈ 1.71; with D6 symmetry the
 *                  6 arms grow independently but share the same statistical properties.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
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

    WALKER_MIN      =   5,
    WALKER_DEFAULT  =  80,   /* fewer needed: each stick creates 12 cells */
    WALKER_MAX      = 200,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    N_ICE_COLORS    =   6,

    HUD_COLS        =  46,
    FPS_UPDATE_MS   = 500,
};

/*
 * STICK_PROB — probability a walker sticks on first contact.
 * 0.55 → thicker, denser arms with rounded edges.
 * 0.90 → sparse, spiky fractal arms (classic DLA morphology).
 */
#define STICK_PROB  0.55f

/*
 * ASPECT_R — terminal cell height / width.
 * Typical terminal fonts render cells ~2× taller than wide.
 * Used when mapping between terminal coordinates and Euclidean
 * space for rotations and distance calculations.
 */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

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
 * Ice crystal palette — six distance bands, tips to core.
 *
 * COL_ICE_1 (bright white) — outermost tips, farthest from centre.
 * COL_ICE_6 (deep navy)    — innermost core, closest to centre.
 *
 * 256-color uses xterm indices; 8-color falls back gracefully.
 */
typedef enum {
    COL_ICE_1  = 1,   /* tips     — bright white   */
    COL_ICE_2  = 2,   /* outer    — pale icy cyan   */
    COL_ICE_3  = 3,   /* mid-out  — bright teal     */
    COL_ICE_4  = 4,   /* mid-in   — medium teal     */
    COL_ICE_5  = 5,   /* inner    — ocean teal-blue */
    COL_ICE_6  = 6,   /* core     — light blue      */
    COL_WALKER = 7,   /* walkers  — drifting ice blue */
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        /*
         * Gradient: outer tips (cold/white) → teal (colder) → inner core (frozen/light-blue)
         *
         *  COL_ICE_1  231  #ffffff  white       — tips, "cold"
         *  COL_ICE_2  195  #d7ffff  pale ice    — near-tip transition
         *  COL_ICE_3   51  #00ffff  bright teal — "colder"
         *  COL_ICE_4   44  #00d7d7  medium teal — deeper colder
         *  COL_ICE_5   38  #00afd7  ocean blue  — approaching core
         *  COL_ICE_6  117  #87d7ff  light blue  — core, "frozen"
         */
        init_pair(COL_ICE_1,  231, COLOR_BLACK);   /* white          */
        init_pair(COL_ICE_2,  195, COLOR_BLACK);   /* pale ice cyan  */
        init_pair(COL_ICE_3,   51, COLOR_BLACK);   /* bright teal    */
        init_pair(COL_ICE_4,   44, COLOR_BLACK);   /* medium teal    */
        init_pair(COL_ICE_5,   38, COLOR_BLACK);   /* ocean blue     */
        init_pair(COL_ICE_6,  117, COLOR_BLACK);   /* light blue     */
        init_pair(COL_WALKER,  75, COLOR_BLACK);   /* sky blue drift  */
    } else {
        init_pair(COL_ICE_1,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_ICE_2,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_ICE_3,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_ICE_4,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_ICE_5,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_ICE_6,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_WALKER, COLOR_WHITE,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * D6 symmetry tables.
 *
 *   CA6[k] = cos(k × 60°)
 *   SA6[k] = sin(k × 60°)
 *
 * For each new frozen cell, grid_freeze_symmetric() applies all 12
 * elements of D6 = ⟨r, s | r⁶ = s² = 1, srs = r⁻¹⟩:
 *   6 rotations:   R_k(dx, dy) for k = 0..5
 *   6 reflections: R_k(dx, -dy) for k = 0..5  (reflect across x-axis first)
 *
 * Coordinates are converted to Euclidean space (scaled by ASPECT_R) before
 * rotation and converted back to terminal space after.
 */
static const float CA6[6] = {
     1.0f,  0.5f, -0.5f,
    -1.0f, -0.5f,  0.5f,
};
static const float SA6[6] = {
     0.0f,  0.8660254f,  0.8660254f,
     0.0f, -0.8660254f, -0.8660254f,
};

/*
 * Grid — the frozen crystal aggregate.
 *
 *   cells[y][x] == 0    →  empty
 *   cells[y][x] == n    →  frozen, draw with COLOR_PAIR(n)
 *
 * Color is assigned at freeze time by Euclidean distance from (cx0, cy0).
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     frozen_count;
    int     rows, cols;
    int     cx0, cy0;    /* terminal centre */
    float   max_dist;    /* normalisation radius for distance coloring */
} Grid;

static bool grid_frozen(const Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return false;
    return g->cells[cy][cx] != 0;
}

/* Cardinal-only adjacency for clean spiky DLA arms */
static bool grid_adjacent_frozen(const Grid *g, int cx, int cy)
{
    return grid_frozen(g, cx - 1, cy)
        || grid_frozen(g, cx + 1, cy)
        || grid_frozen(g, cx,     cy - 1)
        || grid_frozen(g, cx,     cy + 1);
}

/*
 * Freeze a single cell with distance-based color.
 * Silent no-op if out-of-bounds or already frozen.
 */
static void grid_freeze_one(Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return;
    if (g->cells[cy][cx] != 0) return;

    float dx_e = (float)(cx - g->cx0);
    float dy_e = (float)(cy - g->cy0) / ASPECT_R;   /* Euclidean y */
    float dist = sqrtf(dx_e * dx_e + dy_e * dy_e);

    /* Map dist → color: far (tips) = bright, near (core) = dark */
    int band = (int)(dist / g->max_dist * (float)N_ICE_COLORS);
    if (band >= N_ICE_COLORS) band = N_ICE_COLORS - 1;
    int col = N_ICE_COLORS - band;   /* 1=tip/white … 6=core/navy */

    g->cells[cy][cx] = (uint8_t)col;
    g->frozen_count++;
}

/*
 * Freeze all 12 D6-symmetric images of cell (cx, cy).
 * Any image that falls outside the grid or is already frozen is silently
 * skipped; all valid, empty images are frozen with distance-based color.
 */
static void grid_freeze_symmetric(Grid *g, int cx, int cy)
{
    float dx = (float)(cx - g->cx0);
    float dy = (float)(cy - g->cy0);

    for (int refl = 0; refl < 2; refl++) {
        float dx_e = dx;
        float dy_e = (refl == 0 ? dy : -dy) / ASPECT_R;  /* into Euclidean */

        for (int k = 0; k < 6; k++) {
            float rx_e = dx_e * CA6[k] - dy_e * SA6[k];
            float ry_e = dx_e * SA6[k] + dy_e * CA6[k];

            int nx = g->cx0 + (int)roundf(rx_e);
            int ny = g->cy0 + (int)roundf(ry_e * ASPECT_R);  /* back to terminal */

            grid_freeze_one(g, nx, ny);
        }
    }
}

/*
 * Choose a display character based on the frozen-neighbour topology.
 *
 * Cardinal + diagonal neighbours are checked so that the crystal arms
 * render as oriented line segments rather than uniform dots.
 */
static chtype grid_cell_char(const Grid *g, int cx, int cy)
{
    bool N  = grid_frozen(g, cx,     cy - 1);
    bool S  = grid_frozen(g, cx,     cy + 1);
    bool E  = grid_frozen(g, cx + 1, cy);
    bool W  = grid_frozen(g, cx - 1, cy);
    bool NE = grid_frozen(g, cx + 1, cy - 1);
    bool NW = grid_frozen(g, cx - 1, cy - 1);
    bool SE = grid_frozen(g, cx + 1, cy + 1);
    bool SW = grid_frozen(g, cx - 1, cy + 1);

    int card = (int)N + (int)S + (int)E + (int)W;

    /* Isolated tip — bright star */
    if (card == 0 && !(NE || NW || SE || SW))
        return (chtype)'*';

    /* Junctions (3+ cardinal connections) — solid block */
    if (card >= 3)
        return (chtype)'#';

    /* Aligned cardinal pairs — thicker arm segments */
    if (N && S && !E && !W) return (chtype)'|';
    if (E && W && !N && !S) return (chtype)'=';

    /* Single cardinal connection → arm end segment */
    if (card == 1) {
        if (N || S) return (chtype)'|';
        return (chtype)'=';
    }

    /* Two misaligned cardinals → corner junction */
    if (card == 2)
        return (chtype)'#';

    /* Diagonal-only connections */
    bool fwd  = NE || SW;
    bool back = NW || SE;
    if (fwd  && !back) return (chtype)'/';
    if (back && !fwd)  return (chtype)'\\';

    return (chtype)'#';
}

static void grid_init(Grid *g, int cols, int rows)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->frozen_count = 0;
    g->cols  = cols;
    g->rows  = rows;
    g->cx0   = cols / 2;
    g->cy0   = rows / 2;
    g->max_dist = hypotf((float)g->cx0, (float)g->cy0 / ASPECT_R) * 0.85f;

    /* Seed: centre cell + tiny arm stubs to bootstrap 6-fold growth */
    grid_freeze_one(g, g->cx0, g->cy0);
    grid_freeze_symmetric(g, g->cx0 + 2, g->cy0);  /* horizontal stub → D6 */
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    /*
     * Pass 1 — glow halo.
     *
     * For every frozen cell, paint its empty 8-neighbours with a dim ':'
     * in the same color band.  This makes each arm appear ~3 cells thick
     * visually even though the frozen structure is 1 cell wide.  Frozen
     * cells drawn in pass 2 overwrite any glow character underneath.
     */
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t col = g->cells[cy][cx];
            if (col == 0) continue;
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    if (dx == 0 && dy == 0) continue;
                    int nx = cx + dx, ny = cy + dy;
                    if (nx < 0 || nx >= g->cols) continue;
                    if (ny < 0 || ny >= g->rows) continue;
                    if (g->cells[ny][nx] != 0) continue;   /* frozen wins */
                    wattron(w, COLOR_PAIR((int)col) | A_DIM);
                    mvwaddch(w, ny, nx, (chtype)':');
                    wattroff(w, COLOR_PAIR((int)col) | A_DIM);
                }
            }
        }
    }

    /*
     * Pass 2 — frozen crystal cells on top of the halo.
     * Tips (col 1, white) and core (col 6, light blue) rendered bold;
     * middle teal bands at normal intensity.
     */
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t col = g->cells[cy][cx];
            if (col == 0) continue;

            attr_t attr = COLOR_PAIR((int)col);
            if (col <= 2 || col == 6) attr |= A_BOLD;

            wattron(w, attr);
            mvwaddch(w, cy, cx, grid_cell_char(g, cx, cy));
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
 * Spawned on a random terminal edge; moves one cardinal step per tick.
 * When the walker reaches a cell adjacent to the frozen aggregate it tests
 * STICK_PROB.  On success, grid_freeze_symmetric() is called — freezing
 * all 12 D6-symmetric images simultaneously — and the walker respawns.
 */
typedef struct {
    int  cx, cy;
    bool active;
} Walker;

static void walker_spawn(Walker *w, int cols, int rows)
{
    int edge = rand() % 4;
    switch (edge) {
    case 0: w->cx = rand() % cols;  w->cy = 0;          break;
    case 1: w->cx = rand() % cols;  w->cy = rows - 1;   break;
    case 2: w->cx = 0;              w->cy = rand() % rows; break;
    default:w->cx = cols - 1;       w->cy = rand() % rows; break;
    }
    w->active = true;
}

static bool walker_tick(Walker *w, Grid *g)
{
    if (!w->active) return false;

    static const int ddx[4] = {  0,  0, -1, 1 };
    static const int ddy[4] = { -1,  1,  0, 0 };
    int dir = rand() % 4;
    int nx  = w->cx + ddx[dir];
    int ny  = w->cy + ddy[dir];

    /* Out of bounds — respawn */
    if (nx < 0 || nx >= g->cols || ny < 0 || ny >= g->rows) {
        walker_spawn(w, g->cols, g->rows);
        return false;
    }

    if (grid_frozen(g, nx, ny)) {
        /* Walker bumped into frozen cell — test sticking from current pos */
        if (!grid_frozen(g, w->cx, w->cy)) {
            if ((float)rand() / (float)RAND_MAX < STICK_PROB) {
                grid_freeze_symmetric(g, w->cx, w->cy);
                return true;
            }
        }
        return false;
    }

    w->cx = nx;
    w->cy = ny;

    /* Test sticking at new position */
    if (grid_adjacent_frozen(g, w->cx, w->cy)) {
        if ((float)rand() / (float)RAND_MAX < STICK_PROB) {
            grid_freeze_symmetric(g, w->cx, w->cy);
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
    wattron(w, COLOR_PAIR(COL_WALKER));
    for (int i = 0; i < s->n_walkers; i++) {
        const Walker *wk = &s->walkers[i];
        if (!wk->active) continue;
        if (wk->cy < 0 || wk->cy >= s->grid.rows) continue;
        if (wk->cx < 0 || wk->cx >= s->grid.cols) continue;
        if (s->grid.cells[wk->cy][wk->cx] == 0)
            mvwaddch(w, wk->cy, wk->cx, (chtype)'.');
    }
    wattroff(w, COLOR_PAIR(COL_WALKER));
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
