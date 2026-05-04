/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fern.c  —  Barnsley Fern IFS fractal, animated point-by-point
 *
 * Four affine transforms (an Iterated Function System) are applied
 * repeatedly to a single point.  The orbit of the point traces out
 * the Barnsley Fern attractor.  Points are plotted a few hundred per
 * tick so you can watch the fern emerge gradually from what looks like
 * random scattered dots — first the stem appears, then branches, then
 * the fine frond detail.
 *
 * After TOTAL_ITERS iterations the completed fern is held briefly,
 * then the screen clears and it grows again.
 *
 * IFS transforms (classic Barnsley):
 *
 *   f1  (x, y) → (0,       0.16 y )         prob  1%  — stem
 *   f2  (x, y) → (0.85x + 0.04y,            prob 85%  — main leaflets
 *                 -0.04x + 0.85y + 1.6 )
 *   f3  (x, y) → (0.20x - 0.26y,            prob  7%  — left frond
 *                  0.23x + 0.22y + 1.6 )
 *   f4  (x, y) → (-0.15x + 0.28y,           prob  7%  — right frond
 *                   0.26x + 0.24y + 0.44)
 *
 * Color is assigned by height (y value):
 *   roots (y≈0)   →  medium green
 *   tips  (y≈10)  →  bright lemon green
 *
 * Keys:
 *   q / ESC   quit
 *   r         reset fern immediately
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fern.c -o fern -lncurses -lm
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

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : IFS chaos game — single-point iteration.
 *                  Unlike barnsley.c which uses a density grid, fern.c
 *                  plots each individual point as it is generated, so you
 *                  can watch the attractor emerge point-by-point from noise.
 *
 * Math           : The four affine maps have a combined contractivity ≤ 0.85.
 *                  Collage theorem: the IFS attractor is the unique compact set
 *                  K satisfying K = T₁(K) ∪ T₂(K) ∪ T₃(K) ∪ T₄(K).
 *                  Probability assignment: p_i ∝ |det(A_i)| ensures the density
 *                  of plotted points is proportional to the "area" each map covers.
 *                  T₁ (stem): det = 0 × 0.16 − 0 × 0 = 0, so p₁=0.01 (flat map).
 *                  T₂ (leaflets): det ≈ 0.85 × 0.85 − 0.04² ≈ 0.72, so p₂=0.85.
 *
 * Rendering      : Color by height (y value in IFS space) maps naturally to
 *                  botanical structure: dark roots, bright leaf tips.
 *                  The IFS coordinate range is scaled and offset to fill the
 *                  terminal — aspect correction for CELL_W / CELL_H ratio.
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    N_PER_TICK       = 400,     /* IFS iterations computed per tick       */
    TOTAL_ITERS      = 80000,   /* iterations before reset                */
    DONE_PAUSE_TICKS =  90,     /* ticks to hold completed fern (~3 s)    */
    N_FERN_COLORS    =   5,     /* color bands from stem to tip           */

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    HUD_COLS         =  46,
    FPS_UPDATE_MS    = 500,
};

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Used to keep the fern at its natural proportions in the terminal.
 */
#define ASPECT_R    2.0f

/* Fern coordinate bounds (math space) */
#define FERN_X_MIN  (-2.5f)
#define FERN_X_MAX  ( 2.8f)
#define FERN_Y_MIN  ( 0.0f)
#define FERN_Y_MAX  (10.0f)

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
 * Fern depth palette — all vivid greens, stem to tip.
 *
 *   COL_FERN_1  40   #00d700  medium green    — roots / stem
 *   COL_FERN_2  76   #5fd700  bright green    — lower branches
 *   COL_FERN_3  118  #87ff00  lime green      — mid fronds
 *   COL_FERN_4  154  #afff00  yellow-lime     — upper fronds
 *   COL_FERN_5  190  #d7ff00  lemon green     — growing tips
 */
typedef enum {
    COL_FERN_1 = 1,   /* stem    — medium green   */
    COL_FERN_2 = 2,   /* lower   — bright green   */
    COL_FERN_3 = 3,   /* mid     — lime green     */
    COL_FERN_4 = 4,   /* upper   — yellow-lime    */
    COL_FERN_5 = 5,   /* tips    — lemon green    */
    COL_HUD    = 6,
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_FERN_1,  40, COLOR_BLACK);   /* medium green  */
        init_pair(COL_FERN_2,  76, COLOR_BLACK);   /* bright green  */
        init_pair(COL_FERN_3, 118, COLOR_BLACK);   /* lime          */
        init_pair(COL_FERN_4, 154, COLOR_BLACK);   /* yellow-lime   */
        init_pair(COL_FERN_5, 190, COLOR_BLACK);   /* lemon         */
        init_pair(COL_HUD,     51, COLOR_BLACK);   /* cyan hud      */
    } else {
        init_pair(COL_FERN_1, COLOR_GREEN,  COLOR_BLACK);
        init_pair(COL_FERN_2, COLOR_GREEN,  COLOR_BLACK);
        init_pair(COL_FERN_3, COLOR_GREEN,  COLOR_BLACK);
        init_pair(COL_FERN_4, COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_FERN_5, COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HUD,    COLOR_CYAN,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Grid — one byte per cell, 0 = empty, 1..N_FERN_COLORS = fern color.
 *
 * scale_y maps fern y-units to terminal rows.
 * scale_x maps fern x-units to terminal columns (stretched slightly to
 *   fill more horizontal space, since the fern's natural y/x ≈ 2 makes
 *   it very narrow when fitted to screen height alone).
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     rows, cols;
    float   scale_y;   /* fern y-unit → terminal rows  */
    float   scale_x;   /* fern x-unit → terminal cols  */
} Grid;

/*
 * fern_color — map fern y coordinate [FERN_Y_MIN, FERN_Y_MAX] to a
 * color band 1..N_FERN_COLORS.
 */
static uint8_t fern_color(float fy)
{
    float norm = (fy - FERN_Y_MIN) / (FERN_Y_MAX - FERN_Y_MIN);
    int c = 1 + (int)(norm * (float)N_FERN_COLORS);
    if (c < 1) c = 1;
    if (c > N_FERN_COLORS) c = N_FERN_COLORS;
    return (uint8_t)c;
}

/*
 * fern_to_cell — convert fern math coordinates to terminal (col, row).
 *
 * x=0 maps to the horizontal center of the terminal.
 * y=FERN_Y_MIN maps to the bottom row; y=FERN_Y_MAX to near the top.
 */
static void fern_to_cell(const Grid *g, float fx, float fy,
                         int *out_col, int *out_row)
{
    float fern_x_center = (FERN_X_MIN + FERN_X_MAX) * 0.5f;
    *out_col = g->cols / 2 + (int)roundf((fx - fern_x_center) * g->scale_x);
    *out_row = g->rows - 2 - (int)roundf((fy - FERN_Y_MIN)    * g->scale_y);
}

static void grid_init(Grid *g, int cols, int rows)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    memset(g->cells, 0, sizeof g->cells);
    g->rows = rows;
    g->cols = cols;

    /*
     * scale_y: fit fern height to terminal rows.
     * scale_x: stretch x to fill ~45% of terminal width — gives a
     *   visible fern without extreme distortion.
     */
    g->scale_y = (float)(rows - 3) / (FERN_Y_MAX - FERN_Y_MIN);
    g->scale_x = (float)cols * 0.45f / (FERN_X_MAX - FERN_X_MIN);
}

static void grid_plot(Grid *g, float fx, float fy)
{
    int col, row;
    fern_to_cell(g, fx, fy, &col, &row);
    if (col < 0 || col >= g->cols || row < 0 || row >= g->rows) return;
    g->cells[row][col] = fern_color(fy);
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int row = 0; row < g->rows; row++) {
        for (int col = 0; col < g->cols; col++) {
            uint8_t c = g->cells[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c);
            if (c == N_FERN_COLORS) attr |= A_BOLD;
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
 * Barnsley Fern IFS — four affine transforms with probabilities.
 *
 *   f(x, y) = (a·x + b·y + e,   c·x + d·y + f)
 *
 * prob_cum is the cumulative probability × 100 used to pick a transform.
 */
static const struct {
    float a, b, c, d, e, f;
    int   prob_cum;
} k_ifs[4] = {
    {  0.00f,  0.00f,  0.00f,  0.16f,  0.00f, 0.00f,   1 }, /* stem     1% */
    {  0.85f,  0.04f, -0.04f,  0.85f,  0.00f, 1.60f,  86 }, /* main    85% */
    {  0.20f, -0.26f,  0.23f,  0.22f,  0.00f, 1.60f,  93 }, /* left     7% */
    { -0.15f,  0.28f,  0.26f,  0.24f,  0.00f, 0.44f, 100 }, /* right    7% */
};

/*
 * ifs_step — apply one randomly chosen IFS transform to (x, y).
 */
static void ifs_step(float *x, float *y)
{
    int r = rand() % 100;
    int t = (r < k_ifs[0].prob_cum) ? 0
          : (r < k_ifs[1].prob_cum) ? 1
          : (r < k_ifs[2].prob_cum) ? 2 : 3;
    float nx = k_ifs[t].a * (*x) + k_ifs[t].b * (*y) + k_ifs[t].e;
    float ny = k_ifs[t].c * (*x) + k_ifs[t].d * (*y) + k_ifs[t].f;
    *x = nx;
    *y = ny;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid  grid;
    float fx, fy;       /* current IFS orbit point (math coords) */
    int   iter_count;   /* total IFS iterations performed         */
    int   done_ticks;   /* ticks held after completion            */
    bool  paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    grid_init(&s->grid, cols, rows);
    s->fx         = 0.0f;
    s->fy         = 0.0f;
    s->iter_count = 0;
    s->done_ticks = 0;
    s->paused     = false;

    /* Warm-up: run IFS a few iterations before plotting so the orbit
     * reaches the attractor and avoids transient off-fern points.    */
    for (int i = 0; i < 20; i++)
        ifs_step(&s->fx, &s->fy);
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
        ifs_step(&s->fx, &s->fy);
        grid_plot(&s->grid, s->fx, s->fy);
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
