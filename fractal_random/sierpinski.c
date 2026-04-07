/*
 * sierpinski.c  —  Sierpinski triangle via the chaos game (IFS)
 *
 * Three vertices define an equilateral triangle.  At each step a random
 * vertex is chosen and the current point moves halfway toward it.  The
 * orbit of this iterated function system traces the Sierpinski triangle
 * attractor — no straight lines, no recursive drawing, just a cloud of
 * random points that converges to the fractal.
 *
 * Color is assigned by which vertex was chosen on the last step:
 *   toward bottom-left  →  cyan
 *   toward bottom-right →  yellow
 *   toward top          →  magenta
 *
 * This gives the characteristic tricolor fractal texture: each of the
 * three main sub-triangles is tinted a different hue, and the same
 * three-color pattern repeats self-similarly at every scale.
 *
 * After TOTAL_ITERS iterations the triangle is held briefly, then the
 * screen clears and it grows again.
 *
 * Keys:
 *   q / ESC   quit
 *   r         reset immediately
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra sierpinski.c -o sierpinski -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid
 *   §5  ifs
 *   §6  scene
 *   §7  screen
 *   §8  app
 */

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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    N_PER_TICK       = 500,     /* IFS iterations per tick               */
    TOTAL_ITERS      = 50000,   /* iterations before reset               */
    DONE_PAUSE_TICKS =  90,     /* ticks to hold completed fractal ~3 s  */

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    HUD_COLS         =  46,
    FPS_UPDATE_MS    = 500,
};

/*
 * Equilateral triangle vertices in math space.
 * V1=bottom-left, V2=bottom-right, V3=top-center.
 *
 * x ∈ [0,1], y ∈ [0, √3/2 ≈ 0.866].
 */
#define V1X  0.0f
#define V1Y  0.0f
#define V2X  1.0f
#define V2Y  0.0f
#define V3X  0.5f
#define V3Y  0.8660254f   /* √3/2 */

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Ensures the triangle looks equilateral rather than squished.
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
 * Vivid triad — one bright color per IFS vertex, evenly spaced on the
 * color wheel.  All three are clearly visible on black.
 *
 *   COL_V1   87   #5fffff  electric cyan   — bottom-left sub-triangle
 *   COL_V2  226   #ffff00  bright yellow   — bottom-right sub-triangle
 *   COL_V3  207   #ff5fff  hot magenta     — top sub-triangle
 */
typedef enum {
    COL_V1  = 1,   /* bottom-left  — electric cyan  */
    COL_V2  = 2,   /* bottom-right — bright yellow  */
    COL_V3  = 3,   /* top          — hot magenta    */
    COL_HUD = 4,
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_V1,   87, COLOR_BLACK);   /* electric cyan  */
        init_pair(COL_V2,  226, COLOR_BLACK);   /* bright yellow  */
        init_pair(COL_V3,  207, COLOR_BLACK);   /* hot magenta    */
        init_pair(COL_HUD,  87, COLOR_BLACK);
    } else {
        init_pair(COL_V1,  COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_V2,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_V3,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_HUD, COLOR_CYAN,    COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Grid — one byte per cell.
 *   0            → empty
 *   COL_V1..V3   → colored fractal point
 *
 * scale_x / scale_y convert math coords (fx, fy) to terminal (col, row).
 * The scales satisfy scale_x = scale_y × ASPECT_R so the triangle appears
 * equilateral on screen despite the non-square character cells.
 *
 * x_off / y_off centre the triangle in the terminal window.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     rows, cols;
    float   scale_x, scale_y;
    int     x_off;   /* terminal col for fx = 0 */
    int     y_off;   /* terminal row for fy = 0 */
} Grid;

static void grid_init(Grid *g, int cols, int rows)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->rows = rows;
    g->cols = cols;

    /*
     * Compute scale_y such that the triangle fits vertically (y ∈ [0, V3Y]).
     * Constrain additionally so the triangle also fits horizontally with
     * the correct aspect ratio (scale_x = scale_y × ASPECT_R, x ∈ [0,1]).
     */
    float sy_rows = (float)(rows - 3) / V3Y;
    float sy_cols = (float)(cols - 4) / ASPECT_R;
    g->scale_y = (sy_rows < sy_cols) ? sy_rows : sy_cols;
    g->scale_x = g->scale_y * ASPECT_R;

    /* Horizontally centre the triangle (width = scale_x × 1.0) */
    g->x_off = (int)((cols - g->scale_x) * 0.5f);
    /* y = 0 sits at the bottom row with a small margin */
    g->y_off = rows - 2;
}

static void grid_plot(Grid *g, float fx, float fy, uint8_t color)
{
    int col = g->x_off + (int)roundf(fx * g->scale_x);
    int row = g->y_off - (int)roundf(fy * g->scale_y);
    if (col < 0 || col >= g->cols || row < 0 || row >= g->rows) return;
    g->cells[row][col] = color;
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int row = 0; row < g->rows; row++) {
        for (int col = 0; col < g->cols; col++) {
            uint8_t c = g->cells[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c) | A_BOLD;
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)'*');
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  ifs                                                                */
/* ===================================================================== */

/*
 * ifs_step — one chaos-game iteration.
 *
 * Pick one of the three vertices uniformly at random.  Move the current
 * point halfway toward that vertex.  Return the vertex index (0, 1, 2)
 * in *which — this becomes the color of the plotted cell.
 */
static void ifs_step(float *x, float *y, int *which)
{
    static const float vx[3] = { V1X, V2X, V3X };
    static const float vy[3] = { V1Y, V2Y, V3Y };
    int v = rand() % 3;
    *x = (*x + vx[v]) * 0.5f;
    *y = (*y + vy[v]) * 0.5f;
    *which = v;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid  grid;
    float fx, fy;
    int   iter_count;
    int   done_ticks;
    bool  paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    grid_init(&s->grid, cols, rows);
    /* Start near the centroid of the triangle */
    s->fx         = (V1X + V2X + V3X) / 3.0f;
    s->fy         = (V1Y + V2Y + V3Y) / 3.0f;
    s->iter_count = 0;
    s->done_ticks = 0;
    s->paused     = false;

    /* Warm-up: run a few iterations so the orbit reaches the attractor */
    for (int i = 0; i < 20; i++) {
        int dummy;
        ifs_step(&s->fx, &s->fy, &dummy);
    }
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;

    if (s->iter_count >= TOTAL_ITERS) {
        s->done_ticks++;
        if (s->done_ticks >= DONE_PAUSE_TICKS)
            scene_init(s, s->grid.cols, s->grid.rows);
        return;
    }

    for (int i = 0; i < N_PER_TICK; i++) {
        int which;
        ifs_step(&s->fx, &s->fy, &which);
        grid_plot(&s->grid, s->fx, s->fy, (uint8_t)(which + 1));
    }
    s->iter_count += N_PER_TICK;
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_draw(&s->grid, w);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

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

static void screen_free(Screen *s)  { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr);

    int pct = sc->iter_count * 100 / TOTAL_ITERS;
    if (pct > 100) pct = 100;
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  pts:%-6d  %3d%%  spd:%d",
             fps, sc->iter_count, pct, sim_fps);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

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

    case ']':
        if (app->sim_fps < SIM_FPS_MAX) app->sim_fps += SIM_FPS_STEP;
        break;
    case '[':
        if (app->sim_fps > SIM_FPS_MIN) app->sim_fps -= SIM_FPS_STEP;
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
