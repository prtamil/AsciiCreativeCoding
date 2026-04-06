/*
 * coral.c  —  Anisotropic DLA coral / dendrite fractal growth
 *
 * Diffusion-Limited Aggregation with direction-dependent sticking:
 *   — Seeds are placed evenly along the bottom row (sea floor).
 *   — Walkers are released from random positions in the top row and
 *     drift downward with a gravity bias.
 *   — Sticking probability depends on the direction of the contacted
 *     frozen cell relative to the walker:
 *
 *       frozen below walker  (cy+1 frozen) → p = 0.90  high  (upward growth)
 *       frozen to the side   (cx±1 frozen) → p = 0.40  medium (branching)
 *       frozen above walker  (cy-1 frozen) → p = 0.10  low   (rare droop)
 *
 *   This anisotropy biases the aggregate to grow upward in branching
 *   columns, producing coral / dendritic tree morphology.
 *
 * Walk bias (downward gravity): 50% down, 20% up, 15% left, 15% right.
 * The bias creates realistic "nutrient settling" from above onto the tips.
 *
 * Color encoding:
 *   Each cell is colored at freeze time based on its row:
 *     deep (bottom) → dark brown/red
 *     mid            → orange
 *     tips (top)     → bright yellow / white
 *   This gives a natural depth-gradient independent of growth order.
 *
 * Auto-reset: when the tallest coral branch reaches the top quarter
 *   of the screen the reef is cleared and new seeds are placed.
 *
 * Keys:
 *   q / ESC   quit
 *   r         reset reef
 *   + =       more walkers
 *   -         fewer walkers
 *   ] [       faster / slower
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra coral.c -o coral -lncurses -lm
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
    WALKER_DEFAULT  = 150,
    WALKER_MAX      = 400,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    N_CORAL_COLORS  =   6,
    N_SEEDS         =   8,   /* evenly spaced seeds along the bottom    */

    HUD_COLS        =  46,
    FPS_UPDATE_MS   = 500,
};

/*
 * Sticking probabilities for each contact direction.
 *
 * The asymmetry drives upward growth: walkers drifting down contact
 * the top of frozen branches (below them) and stick readily.
 * Walkers hitting a frozen cell above them (underside contact) rarely
 * stick, so the coral never grows downward from a suspended tip.
 */
#define STICK_P_BELOW   0.90f   /* frozen cell directly below walker   */
#define STICK_P_SIDE    0.40f   /* frozen cell to the left or right    */
#define STICK_P_ABOVE   0.10f   /* frozen cell directly above walker   */

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
 * Coral depth palette — dark at the roots, bright at the tips.
 *
 * COL_CORAL_1  deepest / substrate  — dark brown-red
 * COL_CORAL_6  highest tips         — bright white
 */
typedef enum {
    COL_CORAL_1 = 1,   /* substrate — dark brown                      */
    COL_CORAL_2 = 2,   /* deep coral — dark red                       */
    COL_CORAL_3 = 3,   /* mid coral — coral red / orange              */
    COL_CORAL_4 = 4,   /* upper mid — orange                          */
    COL_CORAL_5 = 5,   /* upper — bright yellow                       */
    COL_CORAL_6 = 6,   /* tips — bright white                         */
    COL_WALKER  = 7,   /* dim dots for drifting walkers               */
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_CORAL_1,  52, COLOR_BLACK);   /* dark brown-red  */
        init_pair(COL_CORAL_2, 124, COLOR_BLACK);   /* medium red      */
        init_pair(COL_CORAL_3, 196, COLOR_BLACK);   /* bright red      */
        init_pair(COL_CORAL_4, 208, COLOR_BLACK);   /* orange          */
        init_pair(COL_CORAL_5, 226, COLOR_BLACK);   /* bright yellow   */
        init_pair(COL_CORAL_6, 231, COLOR_BLACK);   /* white           */
        init_pair(COL_WALKER,  238, COLOR_BLACK);   /* dark grey       */
    } else {
        init_pair(COL_CORAL_1, COLOR_RED,    COLOR_BLACK);
        init_pair(COL_CORAL_2, COLOR_RED,    COLOR_BLACK);
        init_pair(COL_CORAL_3, COLOR_RED,    COLOR_BLACK);
        init_pair(COL_CORAL_4, COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_CORAL_5, COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_CORAL_6, COLOR_WHITE,  COLOR_BLACK);
        init_pair(COL_WALKER,  COLOR_WHITE,  COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Grid — the frozen coral aggregate.
 *
 *   cells[y][x] == 0  →  empty
 *   cells[y][x] == n  →  frozen, draw with COLOR_PAIR(n)
 *
 * Color is assigned at freeze time based on row position:
 *   cy near rows−1 (bottom) → COL_CORAL_1 (dark)
 *   cy near 0      (top)    → COL_CORAL_6 (bright)
 *
 * This height gradient is baked in once, and never changes after freezing.
 *
 * tallest_row tracks the minimum cy of any frozen non-seed cell.
 * When tallest_row < rows/4, the reef has grown high enough to reset.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     frozen_count;
    int     tallest_row;    /* smallest cy seen in a non-seed frozen cell */
    int     rows;
    int     cols;
} Grid;

/*
 * grid_color_for_row() — map cell row to a depth-based color pair.
 *
 * cy == rows-1  (bottom)  →  COL_CORAL_1 (darkest)
 * cy == 0       (top)     →  COL_CORAL_6 (brightest)
 */
static uint8_t grid_color_for_row(int cy, int rows)
{
    /* height ∈ [0,1], 0=bottom, 1=top */
    float h = 1.0f - (float)cy / (float)(rows - 1);
    int   idx = (int)(h * N_CORAL_COLORS);
    if (idx < 0)               idx = 0;
    if (idx >= N_CORAL_COLORS) idx = N_CORAL_COLORS - 1;
    return (uint8_t)(idx + 1);
}

static void grid_place_seeds(Grid *g)
{
    int cy = g->rows - 1;
    for (int i = 0; i < N_SEEDS; i++) {
        int cx = (int)((float)(i + 0.5f) * (float)g->cols / (float)N_SEEDS);
        if (cx >= g->cols) cx = g->cols - 1;
        g->cells[cy][cx] = COL_CORAL_1;
        g->frozen_count++;
    }
}

static void grid_init(Grid *g, int cols, int rows)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->frozen_count = 0;
    g->tallest_row  = rows - 1;
    g->cols         = cols;
    g->rows         = rows;
    grid_place_seeds(g);
}

static void grid_freeze(Grid *g, int cx, int cy)
{
    g->cells[cy][cx] = grid_color_for_row(cy, g->rows);
    g->frozen_count++;
    if (cy < g->tallest_row) g->tallest_row = cy;
}

static bool grid_frozen(const Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return false;
    return g->cells[cy][cx] != 0;
}

/*
 * grid_stick_prob() — direction-dependent sticking probability.
 *
 * Returns the highest applicable probability for the position (cx, cy).
 * Checks each of the four neighbours and picks the contact direction
 * with the greatest sticking weight:
 *
 *   frozen below (cy+1) → STICK_P_BELOW  upward growth favoured
 *   frozen sides        → STICK_P_SIDE   lateral branching
 *   frozen above (cy-1) → STICK_P_ABOVE  downward growth suppressed
 */
static float grid_stick_prob(const Grid *g, int cx, int cy)
{
    float p = 0.0f;
    if (grid_frozen(g, cx, cy + 1) && STICK_P_BELOW > p) p = STICK_P_BELOW;
    if (grid_frozen(g, cx - 1, cy) && STICK_P_SIDE  > p) p = STICK_P_SIDE;
    if (grid_frozen(g, cx + 1, cy) && STICK_P_SIDE  > p) p = STICK_P_SIDE;
    if (grid_frozen(g, cx, cy - 1) && STICK_P_ABOVE > p) p = STICK_P_ABOVE;
    return p;
}

/*
 * Symbols chosen by color band: deeper = denser chars, tips = lighter.
 */
static chtype grid_char_for_col(uint8_t col)
{
    static const char k_chars[] = { '#', '+', '+', '*', '*', '^' };
    if (col < 1 || col > 6) return (chtype)'*';
    return (chtype)(unsigned char)k_chars[col - 1];
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t col = g->cells[cy][cx];
            if (col == 0) continue;
            attr_t attr = COLOR_PAIR((int)col);
            if      (col >= 5) attr |= A_BOLD;
            else if (col <= 1) attr |= A_DIM;
            wattron(w, attr);
            mvwaddch(w, cy, cx, grid_char_for_col(col));
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  walker                                                             */
/* ===================================================================== */

/*
 * Walker — one drifting nutrient particle.
 *
 * Walks with a downward gravity bias:
 *   50% down, 20% up, 15% left, 15% right
 *
 * This models nutrients / sediment drifting down from above onto the
 * growing coral tips.  Upward steps let walkers occasionally pass over
 * a branch and reach a different tip, producing realistic branching.
 *
 * Sticking uses grid_stick_prob() — direction-dependent anisotropy
 * drives upward growth and suppresses downward overhang.
 */
typedef struct {
    int  cx, cy;
    bool active;
} Walker;

static void walker_spawn(Walker *w, int cols, int rows)
{
    /* Release from random position in the top row */
    w->cx    = rand() % cols;
    w->cy    = 0;
    w->active = true;
    (void)rows;
}

/*
 * walker_tick() — advance one step with downward gravity bias.
 *
 * Returns true when the walker froze a cell (caller should respawn it).
 */
static bool walker_tick(Walker *w, Grid *g)
{
    if (!w->active) return false;

    /* Biased direction: 50% down, 20% up, 15% each side */
    int dx = 0, dy = 0;
    int r = rand() % 100;
    if      (r < 50) { dy =  1; }   /* down  */
    else if (r < 70) { dy = -1; }   /* up    */
    else if (r < 85) { dx = -1; }   /* left  */
    else             { dx =  1; }   /* right */

    int nx = w->cx + dx;
    int ny = w->cy + dy;

    /* Out of bounds: wrap horizontally, respawn vertically */
    if (nx < 0)         nx = g->cols - 1;
    if (nx >= g->cols)  nx = 0;
    if (ny < 0 || ny >= g->rows) {
        walker_spawn(w, g->cols, g->rows);
        return false;
    }

    if (grid_frozen(g, nx, ny)) {
        /* Hit a frozen cell — test sticking from current position */
        float p = grid_stick_prob(g, w->cx, w->cy);
        if (p > 0.0f && !grid_frozen(g, w->cx, w->cy)) {
            if ((float)rand() / (float)RAND_MAX < p) {
                grid_freeze(g, w->cx, w->cy);
                return true;
            }
        }
        return false;
    }

    /* Move */
    w->cx = nx;
    w->cy = ny;

    /* Test sticking at new position */
    float p = grid_stick_prob(g, w->cx, w->cy);
    if (p > 0.0f && !grid_frozen(g, w->cx, w->cy)) {
        if ((float)rand() / (float)RAND_MAX < p) {
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

    /*
     * Auto-reset when the tallest branch reaches the top quarter.
     * This prevents the terminal from filling up and keeps the
     * animation cycling continuously.
     */
    if (s->grid.tallest_row < s->grid.rows / 4)
        scene_init(s, s->grid.cols, s->grid.rows);

    for (int i = 0; i < s->n_walkers; i++) {
        bool froze = walker_tick(&s->walkers[i], &s->grid);
        if (froze)
            walker_spawn(&s->walkers[i], s->grid.cols, s->grid.rows);
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_draw(&s->grid, w);

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
    attron(COLOR_PAIR(COL_CORAL_5) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_CORAL_5) | A_BOLD);
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
