/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * buddhabrot.c  —  Buddhabrot density-accumulator fractal
 *
 * The Buddhabrot renders the trajectories of the Mandelbrot iteration.
 * For every randomly sampled complex number c that ESCAPES the iteration
 * z → z² + c (|z| exceeds 2 within MAX_ITER steps), every point visited
 * by the orbit is projected onto the terminal grid and a hit counter is
 * incremented.  Regions where many orbits pass through glow brightest —
 * the result looks like a luminous Buddha figure.
 *
 * The Anti-Buddhabrot uses the same technique for orbits that do NOT
 * escape (bounded orbits), revealing the interior structure of the set.
 *
 * Algorithm per tick:
 *   1. Sample SAMPLES_PER_TICK random c values from the bounding box.
 *   2. Skip main cardioid and period-2 bulb (Buddha mode) — they never
 *      escape and waste iterations.
 *   3. Quick-test pass: iterate z from 0, record whether c escapes.
 *   4. If the plot condition is met, trace pass: iterate again and
 *      increment counts[row][col] for each orbit point that falls
 *      within the display region.
 *   5. Track max_count for renormalisation on every draw.
 *
 * The image builds from noise to recognisable form as samples accumulate.
 * After TOTAL_SAMPLES the display holds briefly then cycles to the next
 * preset.
 *
 * Density → colour (5-level nebula gradient, purple → white):
 *   COL_C1  xterm  55  dark blue-purple   '.'
 *   COL_C2  xterm  93  violet             ':'
 *   COL_C3  xterm 141  light purple       '+'
 *   COL_C4  xterm 183  lavender-pink      '#'
 *   COL_C5  xterm 231  white (bold)       '@'
 *
 * Five presets cycle automatically:
 *   buddha 500 · buddha 2000
 *   anti   100 · anti   500 · anti  1000
 *
 * Keys:
 *   q / ESC   quit
 *   n / r     skip to next preset immediately
 *   ] [       faster / slower simulation
 *   p / spc   pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fractal_random/buddhabrot.c -o buddhabrot -lncurses -lm
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

    /*
     * SAMPLES_PER_TICK — random c values tested each simulation tick.
     * Each sample may trace an orbit of up to MAX_ITER points, so this
     * controls both computation cost and animation speed.
     */
    SAMPLES_PER_TICK =  800,

    /*
     * TOTAL_SAMPLES — target accumulation before cycling to next preset.
     * At SIM_FPS_DEFAULT (30) the display completes in ≈ 6 s per preset.
     */
    TOTAL_SAMPLES    = 150000,

    DONE_PAUSE_TICKS =  90,   /* ticks to hold the finished image (~3 s) */
    N_PRESETS        =   5,

    GRID_ROWS_MAX    =  80,
    GRID_COLS_MAX    = 300,

    HUD_COLS         =  52,
    FPS_UPDATE_MS    = 500,
};

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Applied when mapping (col, row) ↔ (re, im) so the fractal is not
 * stretched by the non-square cell shape.
 */
#define ASPECT_R    2.0f

/*
 * Sampling bounding box — large enough to enclose the entire Mandelbrot
 * set and all interesting escaping trajectories.
 */
#define SAMPLE_RE_MIN  -2.5f
#define SAMPLE_RE_MAX   1.0f
#define SAMPLE_IM_MIN  -1.25f
#define SAMPLE_IM_MAX   1.25f

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
/* §3  color & themes                                                     */
/* ===================================================================== */

typedef enum {
    COL_C1  = 1,
    COL_C2  = 2,
    COL_C3  = 3,
    COL_C4  = 4,
    COL_C5  = 5,
    COL_HUD = 6,
} ColorID;

/* Five density levels (sparse → peak) + HUD accent */
typedef struct {
    const char *name;
    int c[5];   /* xterm-256 colours for COL_C1..COL_C5 */
    int hud;    /* HUD accent colour */
} Theme;

#define N_THEMES 8
static const Theme k_themes[N_THEMES] = {
/*          name         C1    C2    C3    C4    C5   hud */
/* 0 */ { "Nebula",   {  55,   93,  141,  183,  231 },  87 },
/* 1 */ { "Matrix",   {  22,   28,   34,   46,   82 },  46 },
/* 2 */ { "Nova",     {  17,   21,   39,  117,  231 },  51 },
/* 3 */ { "Poison",   {  22,  100,  148,  190,  226 }, 154 },
/* 4 */ { "Ocean",    {  17,   18,   24,   38,  159 },  39 },
/* 5 */ { "Fire",     {  52,   88,  196,  214,  226 }, 208 },
/* 6 */ { "Gold",     {  52,   94,  136,  220,  231 }, 226 },
/* 7 */ { "Ice",      {  16,   23,   30,  159,  231 }, 123 },
};

static int g_theme = 0;

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    if (COLORS >= 256) {
        for (int i = 0; i < 5; i++)
            init_pair(COL_C1 + i, th->c[i], COLOR_BLACK);
        init_pair(COL_HUD, th->hud, COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        theme_apply(g_theme);
    } else {
        init_pair(COL_C1,  COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_C2,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_C3,  COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_C4,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_C5,  COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_HUD, COLOR_CYAN,    COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

/*
 * Buddhabrot mode:
 *   MODE_BUDDHA — plot orbits of c values that ESCAPE (standard Buddhabrot)
 *   MODE_ANTI   — plot orbits of c values that DON'T escape (Anti-Buddhabrot)
 */
typedef enum { MODE_BUDDHA, MODE_ANTI } BuddhMode;

/*
 * Presets — vary max_iter and mode to produce different structural views.
 *
 * High max_iter (2000): thin filaments and fine tendrils near the boundary.
 * Anti mode:            traces bounded orbits, illuminates the interior.
 */
static const struct {
    int         max_iter;
    BuddhMode   mode;
    const char *name;
} k_presets[N_PRESETS] = {
    {  500, MODE_BUDDHA, "buddha  500" },
    { 2000, MODE_BUDDHA, "buddha 2000" },
    {  100, MODE_ANTI,   "anti    100" },
    {  500, MODE_ANTI,   "anti    500" },
    { 1000, MODE_ANTI,   "anti   1000" },
};

/*
 * Grid — 2-D hit-count buffer.
 *
 *   counts[row][col]  — number of orbit points that mapped to this cell
 *   max_count         — highest count; used to normalise for display
 *
 * The display region is centred at (re_center, im_center) with half-extents
 * (re_half, im_half).  re_half is derived from the terminal geometry:
 *   re_half = im_half × (cols / rows) / ASPECT_R
 * so the complex plane is displayed without aspect distortion.
 */
typedef struct {
    uint32_t counts[GRID_ROWS_MAX][GRID_COLS_MAX];
    uint32_t max_count;
    int      samples_done;
    int      done_ticks;
    int      rows, cols;
    int      preset;
    float    re_center, im_center;
    float    re_half, im_half;
} Grid;

static void grid_init(Grid *g, int cols, int rows, int preset)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;

    memset(g->counts, 0, sizeof g->counts);
    g->max_count    = 0;
    g->samples_done = 0;
    g->done_ticks   = 0;
    g->rows         = rows;
    g->cols         = cols;
    g->preset       = preset % N_PRESETS;

    /*
     * Centre the display at (-0.5, 0) — the gravitational centre of the
     * Buddhabrot figure.  im_half = 1.3 gives comfortable headroom above
     * and below the figure on most terminal sizes.
     */
    g->re_center = -0.5f;
    g->im_center =  0.0f;
    g->im_half   =  1.3f;
    g->re_half   =  g->im_half * (float)cols / (float)rows / ASPECT_R;
}

/*
 * density_color — map a raw hit count to a colour band.
 *
 * log(1 + count) / log(1 + max_count) compresses the extreme dynamic range
 * of anti-Buddhabrot where attractor cells accumulate max_iter × N_SAMPLES
 * hits.  With sqrt normalisation those attractor cells would set max_count
 * to ~10^6, making every transient cell (count=1–5) return sqrt≈0.001 which
 * still fell in the first visible band → scattered dots on a blank screen.
 *
 * Log normalisation:  log(2) / log(10^6+1) ≈ 0.035  →  below 0.25 → invisible.
 * A cell needs substantial hits before it becomes visible, which is only real
 * attractor structure, not sampling noise.
 *
 * Thresholds (on log-normalised value):
 *   < 0.25  →  invisible  (suppress transient / noise dots)
 *   < 0.45  →  COL_C1
 *   < 0.62  →  COL_C2
 *   < 0.78  →  COL_C3
 *   < 0.90  →  COL_C4
 *   ≥ 0.90  →  COL_C5  (peak, drawn bold)
 */
static uint8_t density_color(uint32_t count, uint32_t max_count, bool anti)
{
    if (count == 0 || max_count == 0) return 0;
    float t = logf(1.0f + (float)count)
            / logf(1.0f + (float)max_count);
    /*
     * Anti-mode attractor cells accumulate max_iter × N_SAMPLES hits, making
     * max_count ~10^6 and every transient cell's t tiny but non-zero.  Raise
     * the invisible floor to 0.25 to suppress those noise dots.
     * Normal Buddhabrot has far lower max_count, so a low floor (0.05) keeps
     * fine orbital structure visible without noise contamination.
     */
    float floor = anti ? 0.25f : 0.05f;
    if (t < floor) return 0;
    if (t < 0.45f) return (uint8_t)COL_C1;
    if (t < 0.62f) return (uint8_t)COL_C2;
    if (t < 0.78f) return (uint8_t)COL_C3;
    if (t < 0.90f) return (uint8_t)COL_C4;
    return               (uint8_t)COL_C5;
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    /* Characters paired to density levels 1–5 */
    static const char k_chars[] = " .:+#@";
    bool anti = (k_presets[g->preset].mode == MODE_ANTI);

    for (int row = 0; row < g->rows; row++) {
        for (int col = 0; col < g->cols; col++) {
            uint8_t c = density_color(g->counts[row][col], g->max_count, anti);
            if (c == 0) continue;

            attr_t attr = COLOR_PAIR((int)c);
            if (c == COL_C5) attr |= A_BOLD;

            wattron(w, attr);
            mvwaddch(w, row, col,
                     (chtype)(unsigned char)k_chars[c < 6 ? c : 5]);
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §5  compute                                                            */
/* ===================================================================== */

/*
 * grid_sample — test one random c value and accumulate orbit density.
 *
 * Buddha mode (anti_mode = false):
 *   Only plot the orbit if c escapes.  Pre-skip the main cardioid and
 *   period-2 bulb to avoid wasting iterations on points that obviously
 *   never escape.
 *
 * Anti mode (anti_mode = true):
 *   Only plot the orbit if c is bounded (does NOT escape).
 *
 * Two-pass approach:
 *   Pass 1 — determine escape status cheaply (terminate on escape).
 *   Pass 2 — if plot condition met, iterate again and accumulate counts.
 *   The double-iteration avoids storing orbit history in a buffer.
 */
static void grid_sample(Grid *g, int max_iter, bool anti_mode)
{
    /* Uniform random sample in the bounding box */
    float cr = SAMPLE_RE_MIN
             + (float)rand() / (float)RAND_MAX
             * (SAMPLE_RE_MAX - SAMPLE_RE_MIN);
    float ci = SAMPLE_IM_MIN
             + (float)rand() / (float)RAND_MAX
             * (SAMPLE_IM_MAX - SAMPLE_IM_MIN);

    /*
     * Main-cardioid check: |z - 0.25| is inside the cardioid iff
     *   q(q + re - 0.25) < 0.25 × im²   where q = |c - 0.25|²
     * Period-2 bulb check: |c + 1|² < 1/16.
     * Both regions are strictly non-escaping — skip them in Buddha mode.
     */
    if (!anti_mode) {
        float q = (cr - 0.25f) * (cr - 0.25f) + ci * ci;
        if (q * (q + cr - 0.25f) < 0.25f * ci * ci)         return;
        if ((cr + 1.0f) * (cr + 1.0f) + ci * ci < 0.0625f)  return;
    }

    /* Pass 1 — escape test */
    float zr = 0.0f, zi = 0.0f;
    int   iter;
    for (iter = 0; iter < max_iter; iter++) {
        float zr2 = zr * zr - zi * zi + cr;
        float zi2 = 2.0f * zr * zi     + ci;
        zr = zr2; zi = zi2;
        if (zr * zr + zi * zi > 4.0f) break;
    }
    bool escaped = (iter < max_iter);

    /* anti_mode XOR escaped: skip if escape status doesn't match mode */
    if (anti_mode == escaped) return;

    /* Pass 2 — trace orbit and accumulate density */
    float re_lo        = g->re_center - g->re_half;
    float im_hi        = g->im_center + g->im_half;
    float re_range_inv = 1.0f / (2.0f * g->re_half);
    float im_range_inv = 1.0f / (2.0f * g->im_half);
    float cols_f       = (float)(g->cols - 1);
    float rows_f       = (float)(g->rows - 1);

    zr = 0.0f; zi = 0.0f;
    for (int i = 0; i < max_iter; i++) {
        /*
         * Map orbit point z = (zr, zi) to terminal grid.
         * im axis is inverted: im increases upward but row increases downward.
         */
        int col = (int)((zr - re_lo)  * re_range_inv * cols_f + 0.5f);
        int row = (int)((im_hi - zi)  * im_range_inv * rows_f + 0.5f);

        /* Unsigned comparison folds the < 0 and >= max cases into one branch */
        if ((unsigned)col < (unsigned)g->cols &&
            (unsigned)row < (unsigned)g->rows) {
            uint32_t v = ++g->counts[row][col];
            if (v > g->max_count) g->max_count = v;
        }

        float zr2 = zr * zr - zi * zi + cr;
        float zi2 = 2.0f * zr * zi     + ci;
        zr = zr2; zi = zi2;
        /* Buddha mode: stop when orbit leaves the escape circle */
        if (!anti_mode && zr * zr + zi * zi > 4.0f) break;
    }
}

/*
 * grid_compute — run SAMPLES_PER_TICK random samples this tick.
 */
static void grid_compute(Grid *g)
{
    int  max_iter = k_presets[g->preset].max_iter;
    bool anti     = (k_presets[g->preset].mode == MODE_ANTI);

    for (int s = 0; s < SAMPLES_PER_TICK; s++)
        grid_sample(g, max_iter, anti);

    g->samples_done += SAMPLES_PER_TICK;
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

    if (g->samples_done >= TOTAL_SAMPLES) {
        /* Hold the completed image briefly, then advance to next preset */
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

    int pct = (TOTAL_SAMPLES > 0)
            ? sc->grid.samples_done * 100 / TOTAL_SAMPLES
            : 100;
    if (pct > 100) pct = 100;

    char buf[HUD_COLS + 32];
    snprintf(buf, sizeof buf,
             "%5.1f fps  %-11s  %3d%%  spd:%d  t:%s",
             fps, k_presets[sc->grid.preset].name, pct, sim_fps,
             k_themes[g_theme].name);
    int hx = s->cols - (int)strlen(buf) - 1;
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

    case 'n': case 'N': case 'r': case 'R':
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

    case 't': case 'T':
        if (COLORS >= 256) {
            g_theme = (g_theme + 1) % N_THEMES;
            theme_apply(g_theme);
        }
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
