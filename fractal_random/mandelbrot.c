/*
 * mandelbrot.c  —  Mandelbrot set fractal, animated random-pixel fill
 *
 * For every pixel (col, row) the position is mapped to a complex number
 * c = re + im·i.  Starting from z = 0, the iteration z → z² + c is
 * repeated until |z| > 2 (escape) or MAX_ITER is reached (inside the set).
 *
 * Pixels are revealed in a random shuffled order so the fractal
 * materialises from scattered dots, identical in spirit to julia.c.
 *
 * Six preset zoom windows cycle automatically, each showing a different
 * region of the Mandelbrot set:
 *
 *   full set        — the classic overview
 *   seahorse valley — spirals along the main bulge neck
 *   north antenna   — filaments above the main body
 *   deep spiral     — zoomed-in spiral detail
 *   mini mandelbrot — a tiny copy of the full set deep in the boundary
 *   antenna tip     — fine filament at the top of the antenna
 *
 * Color scheme — electric neon (inside → boundary → background):
 *   inside set   →  magenta        *
 *   near inside  →  light purple   #
 *   mid          →  cyan           +
 *   far exterior →  lime green     .
 *   outermost    →  yellow         ,
 *   fast escape  →  (black/empty)
 *
 * Keys:
 *   q / ESC   quit
 *   r / n     skip to next preset immediately
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra mandelbrot.c -o mandelbrot -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid
 *   §5  compute
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    PIXELS_PER_TICK  =  60,    /* pixels computed per simulation tick    */
    MAX_ITER         = 256,    /* Mandelbrot iteration cap               */
    DONE_PAUSE_TICKS =  90,    /* ticks to hold completed view (~3 s)    */
    N_PRESETS        =   6,
    N_COLORS         =   5,

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    HUD_COLS         =  56,
    FPS_UPDATE_MS    = 500,
};

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Corrects the complex-plane mapping so circles stay circular.
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
 * Electric neon palette — all vivid, all visible on black.
 * The Mandelbrot body glows magenta; the boundary erupts in a
 * purple → cyan → green → yellow halo fading to black.
 *
 *   COL_INSIDE  201  #ff00ff  magenta      — fractal interior
 *   COL_C5      141  #af87ff  light purple — very slow escape (near body)
 *   COL_C4       51  #00ffff  bright cyan  — slow escape
 *   COL_C3       82  #5fff00  lime green   — medium escape
 *   COL_C2      226  #ffff00  yellow       — fastest shown escape
 *   (0 = background, not drawn)
 */
typedef enum {
    COL_INSIDE = 1,   /* fractal interior — magenta       */
    COL_C2     = 2,   /* far exterior     — yellow        */
    COL_C3     = 3,   /* exterior         — lime green    */
    COL_C4     = 4,   /* mid-exterior     — cyan          */
    COL_C5     = 5,   /* near-inside      — light purple  */
    COL_HUD    = 6,
} ColorID;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_INSIDE, 201, COLOR_BLACK);   /* magenta      */
        init_pair(COL_C2,    226, COLOR_BLACK);    /* yellow       */
        init_pair(COL_C3,     82, COLOR_BLACK);    /* lime green   */
        init_pair(COL_C4,     51, COLOR_BLACK);    /* bright cyan  */
        init_pair(COL_C5,    141, COLOR_BLACK);    /* light purple */
        init_pair(COL_HUD,    87, COLOR_BLACK);    /* cyan hud     */
    } else {
        init_pair(COL_INSIDE, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_C2,     COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_C3,     COLOR_GREEN,   COLOR_BLACK);
        init_pair(COL_C4,     COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_C5,     COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_HUD,    COLOR_CYAN,    COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Mandelbrot zoom-window presets.
 *
 *   cr, ci    — centre of the window in the complex plane
 *   im_half   — half the imaginary (vertical) extent of the window
 *               re_half is derived at runtime using terminal aspect ratio
 *   name      — label shown in HUD
 */
static const struct {
    float       cr, ci, im_half;
    const char *name;
} k_presets[N_PRESETS] = {
    { -0.5000f,  0.0000f, 1.300f, "full set"        },
    { -0.7450f,  0.1130f, 0.150f, "seahorse valley" },
    {  0.0000f,  0.6500f, 0.300f, "north antenna"   },
    { -0.7220f,  0.2460f, 0.020f, "deep spiral"     },
    { -1.7500f,  0.0000f, 0.080f, "mini mandelbrot" },
    { -0.6000f,  0.6000f, 0.200f, "antenna tip"     },
};

/*
 * Grid — terminal cell buffer and pixel shuffle order.
 *
 *   cells[row][col] == 0    →  not yet computed (or background)
 *   cells[row][col] == 1..5 →  computed color (COL_INSIDE or COL_C2..C5)
 *
 * Pixels are processed in a Fisher-Yates shuffled order (g->order[]) so
 * the fractal appears to materialise from random scattered dots.
 *
 * re_center / im_center — complex plane centre of the current view.
 * re_half / im_half     — half-extents; re_half corrects for ASPECT_R.
 */
typedef struct {
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];
    int     order[GRID_ROWS_MAX * GRID_COLS_MAX];
    int     n_pixels;
    int     progress;
    int     done_ticks;
    int     rows, cols;
    int     preset;
    float   re_center, im_center;
    float   re_half,   im_half;
} Grid;

static void grid_init(Grid *g, int cols, int rows, int preset)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;

    memset(g->cells, 0, sizeof g->cells);
    g->rows       = rows;
    g->cols       = cols;
    g->progress   = 0;
    g->done_ticks = 0;
    g->n_pixels   = rows * cols;
    g->preset     = preset % N_PRESETS;

    g->re_center  = k_presets[g->preset].cr;
    g->im_center  = k_presets[g->preset].ci;
    g->im_half    = k_presets[g->preset].im_half;
    /* re_half: scale real axis so the view is undistorted on the terminal */
    g->re_half    = g->im_half * (float)cols / (float)rows / ASPECT_R;

    /* Fisher-Yates shuffle */
    for (int i = 0; i < g->n_pixels; i++)
        g->order[i] = i;
    for (int i = g->n_pixels - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = g->order[i]; g->order[i] = g->order[j]; g->order[j] = t;
    }
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    static const char k_chars[] = " ,. +#*";
    for (int row = 0; row < g->rows; row++) {
        for (int col = 0; col < g->cols; col++) {
            uint8_t c = g->cells[row][col];
            if (c == 0) continue;
            attr_t attr = COLOR_PAIR((int)c);
            if (c == COL_INSIDE || c == COL_C5) attr |= A_BOLD;
            wattron(w, attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)k_chars[c < 7 ? c : 0]);
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  compute                                                            */
/* ===================================================================== */

/*
 * escape_color — map escape iteration → color band.
 *
 * frac = iter / MAX_ITER ∈ [0, 1).
 *   frac < 0.10              →  0 (background — escaped too quickly)
 *   frac ∈ [0.10, 1.0) asc   →  COL_C2 (yellow) … COL_C5 (purple)
 *
 * High frac = slow escape = near the set boundary = bright/vivid color.
 * Low frac  = fast escape = far from boundary     = dim / background.
 */
static uint8_t escape_color(int iter)
{
    float frac = (float)iter / (float)MAX_ITER;
    if (frac < 0.10f) return 0;
    int band = 2 + (int)((frac - 0.10f) / 0.90f * 4.0f);
    return (uint8_t)(band > 5 ? 5 : band);
}

/*
 * grid_compute — compute the next PIXELS_PER_TICK pixels from the
 * shuffled order and store results.
 *
 * Mandelbrot iteration: start z = 0, iterate z → z² + c where c is the
 * pixel's complex coordinate.  Count steps until |z| > 2 or MAX_ITER.
 */
static void grid_compute(Grid *g)
{
    int end = g->progress + PIXELS_PER_TICK;
    if (end > g->n_pixels) end = g->n_pixels;

    for (int s = g->progress; s < end; s++) {
        int idx = g->order[s];
        int col = idx % g->cols;
        int row = idx / g->cols;

        /* Map (col, row) → complex plane with aspect-ratio correction */
        float cr = g->re_center
                 + ((float)col / (float)(g->cols - 1) - 0.5f) * 2.0f * g->re_half;
        float ci = g->im_center
                 + (0.5f - (float)row / (float)(g->rows - 1)) * 2.0f * g->im_half;

        /* Mandelbrot iteration: z₀ = 0, z → z² + c */
        float zr = 0.0f, zi = 0.0f;
        int   iter;
        for (iter = 0; iter < MAX_ITER; iter++) {
            float zr2 = zr * zr - zi * zi + cr;
            float zi2 = 2.0f * zr * zi     + ci;
            zr = zr2;
            zi = zi2;
            if (zr * zr + zi * zi > 4.0f) break;
        }

        uint8_t c = (iter == MAX_ITER)
                  ? (uint8_t)COL_INSIDE
                  : escape_color(iter);
        if (c > 0)
            g->cells[row][col] = c;
    }
    g->progress = end;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid grid;
    bool paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    grid_init(&s->grid, cols, rows, 0);
    s->paused = false;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;

    Grid *g = &s->grid;

    if (g->progress >= g->n_pixels) {
        g->done_ticks++;
        if (g->done_ticks >= DONE_PAUSE_TICKS)
            grid_init(g, g->cols, g->rows, (g->preset + 1) % N_PRESETS);
        return;
    }
    grid_compute(g);
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

    const Grid *g = &sc->grid;
    int pct = (g->n_pixels > 0) ? g->progress * 100 / g->n_pixels : 100;
    if (pct > 100) pct = 100;

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps  %-16s  %3d%%  spd:%d",
             fps, k_presets[g->preset].name, pct, sim_fps);
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
    int preset = app->scene.grid.preset;
    grid_init(&app->scene.grid, app->screen.cols, app->screen.rows, preset);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case 'r': case 'R': case 'n': case 'N':
        grid_init(&app->scene.grid, app->screen.cols, app->screen.rows,
                  (app->scene.grid.preset + 1) % N_PRESETS);
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
