/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * terrain.c — Fractal Terrain (Diamond-Square)
 *
 * Builds a random mountain landscape on a 65x65 grid, shows it as coloured
 * ASCII (water through snowcap), and slowly wears it down over time so peaks
 * round off into plains.
 *
 * The two ideas it leans on:
 *   • Diamond-Square — the classic recipe for fake-but-believable terrain.
 *     Start with corner heights, repeatedly fill in the midpoint of each gap
 *     with the average of its neighbours plus a shrinking dash of randomness.
 *     (Fournier/Fussell/Carpenter 1982; Miller 1986 fixed the seams.)
 *   • Thermal weathering — the same way a sandpile slumps: wherever the ground
 *     is too steep, shove a little dirt downhill until it settles.
 *     (Musgrave/Kolb/Mace 1989.)
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/*
 * How the file is laid out. The whole "world" is one square heightmap: built
 * once, then nibbled at each tick. The sections below split that work apart so
 * each piece touches only what it owns:
 *
 *   §1 config      the dial settings (grid size, roughness, erosion strength)
 *   §2 performance the clock and sleep used to pace the frame rate
 *   §3 logic       clampf — keeps a number inside a range, nothing else
 *   §4 simulation  the only code that changes the terrain: build it, erode it
 *   §5 effects     nothing — colours/contours are worked out fresh while drawing
 *   §6 delays      nothing — just the pause flag
 *   §7 render      reads the heightmap and paints it; never changes it
 *   §8 app         the main loop that ties it together and handles keys
 *
 * Each frame (in main): run any due erosion ticks, draw, then read one keypress.
 * Only the erosion tick changes the terrain, and only when not paused. Keys like
 * 'r' (rebuild) and 'e' (toggle erosion) act once per frame, outside the tick.
 */

/* §1  config — the dial settings (grid size, roughness, erosion strength) */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,

    GRID_N          = 6,                  /* grid is 2^GRID_N + 1 cells wide */
    GRID            = (1 << GRID_N) + 1,  /* 65 × 65 heightmap             */
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* How much the random nudge shrinks each round. Lower = smoother, rolling
 * hills; higher = jagged, noisy terrain. 0.60 lands on natural-looking peaks. */
#define ROUGHNESS    0.60f

/* The steepest a slope can get before it starts to slump. A bigger gap between
 * two neighbouring cells than this counts as "too steep, let it slide". */
#define TALUS        0.022f

/* How aggressively the too-steep dirt slides downhill, and how many erosion
 * sweeps run per tick. Small on purpose: the land softens over minutes, not
 * in a flash. */
#define EROSION_RATE 0.0012f
#define ERODE_PASSES 2

/* §2  performance — the clock and sleep used to pace the frame rate */

/* Just the raw timers. How they're used to cap the frame rate lives in main. */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* §3  logic — clampf, keeps a number inside a range, nothing else */

static float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* §4  simulation — the only code that changes the terrain: build it, erode it */

/* ── Terrain ───────────────────────────────────────────────────────────── *
 * The landscape itself: a grid of heights, plus a tally of how much erosion
 * it has been through. Everything else in the program either fills this in or
 * reads it to draw. The user's controls (pause, erosion on/off) deliberately
 * live elsewhere, on Scene — this struct is just the ground.
 *   hmap         the height at each grid point, each value 0.0 (lowest) to
 *                1.0 (highest). The grid is 2^n+1 wide on purpose so the
 *                outer corners always line up when we fill in midpoints.
 *   erode_count  how many erosion sweeps this land has taken (back to 0 on
 *                rebuild) — shown in the HUD so you can watch it climb. */
typedef struct {
    float hmap[GRID][GRID];
    int   erode_count;
} Terrain;

/* ── RNG helpers ────────────────────────────────────────────────────── */

/* A random number from 0 to 1, and one from -1 to +1 — the dice rolls that
 * give the terrain its bumps. */
static float randf01(void) { return (float)rand() / (float)RAND_MAX; }
static float randf11(void) { return randf01() * 2.0f - 1.0f; }

/* ── Diamond-Square generation ──────────────────────────────────────── */

/* Fills the centre of each square: average its four corners, then nudge that
 * up or down by a random amount. `scale` is how big the nudge can be. */
static void diamond_step(float (*h)[GRID], int stride, int half, float scale)
{
    for (int y = 0; y < GRID - 1; y += stride) {
        for (int x = 0; x < GRID - 1; x += stride) {
            float avg = (h[y][x]          + h[y][x + stride]
                       + h[y + stride][x] + h[y + stride][x + stride]) * 0.25f;
            h[y + half][x + half] = clampf(avg + randf11() * scale, 0.0f, 1.0f);
        }
    }
}

/* The other half of the recipe: fills the points sitting on the edges between
 * those centres. Each one averages the neighbours it actually has (4 inside the
 * map, only 2 along the border), then gets the same random nudge. The staggered
 * start per row is what lands us exactly on those edge points. */
static void square_step(float (*h)[GRID], int stride, int half, float scale)
{
    for (int y = 0; y < GRID; y += half) {
        for (int x = ((y / half) & 1) ? 0 : half; x < GRID; x += stride) {
            float sum = 0.0f;
            int   cnt = 0;
            if (y >= half)       { sum += h[y - half][x]; cnt++; }
            if (y + half < GRID) { sum += h[y + half][x]; cnt++; }
            if (x >= half)       { sum += h[y][x - half]; cnt++; }
            if (x + half < GRID) { sum += h[y][x + half]; cnt++; }
            h[y][x] = clampf(sum / (float)cnt + randf11() * scale, 0.0f, 1.0f);
        }
    }
}

/* Stretches the whole map so its lowest point becomes 0 and its highest becomes
 * 1. Without this a flat-ish random map might be all "plains" green; this makes
 * sure every run shows the full range from water to snow. */
static void normalize_heights(float (*h)[GRID])
{
    float lo = 1.0f, hi = 0.0f;
    for (int y = 0; y < GRID; y++)
        for (int x = 0; x < GRID; x++) {
            if (h[y][x] < lo) lo = h[y][x];
            if (h[y][x] > hi) hi = h[y][x];
        }
    float range = hi - lo;
    if (range > 1e-4f)
        for (int y = 0; y < GRID; y++)
            for (int x = 0; x < GRID; x++)
                h[y][x] = (h[y][x] - lo) / range;
}

/* Builds a brand-new landscape from scratch. Each round works at half the spacing
 * of the last and with a smaller random nudge, so we lay down big shapes first
 * and finer wrinkles after — the look of real fractal terrain. */
static void terrain_generate(Terrain *t)
{
    float (*h)[GRID] = t->hmap;

    /* Start by giving the four corners random heights; everything in between
     * gets filled in by the two steps below. */
    h[0][0]               = randf01();
    h[0][GRID - 1]        = randf01();
    h[GRID - 1][0]        = randf01();
    h[GRID - 1][GRID - 1] = randf01();

    float scale = 0.5f;                          /* how big the random nudge can be */
    for (int stride = GRID - 1; stride > 1; stride >>= 1) {
        int half = stride >> 1;
        diamond_step(h, stride, half, scale);
        square_step (h, stride, half, scale);
        scale *= ROUGHNESS;
    }

    normalize_heights(h);
    t->erode_count = 0;
}

/* ── Thermal weathering ─────────────────────────────────────────────── */

/* One round of "let steep ground slump". For every cell, look at its four
 * neighbours; wherever the drop to a neighbour is steeper than TALUS, shift a
 * little height from the high side to the low side. Run enough times and the
 * mountains soften into hills. */
static void terrain_erode(Terrain *t)
{
    float (*h)[GRID] = t->hmap;
    static const int DX[4] = { 1, -1,  0,  0 };
    static const int DY[4] = { 0,  0,  1, -1 };

    for (int pass = 0; pass < ERODE_PASSES; pass++) {
        for (int y = 0; y < GRID; y++) {
            for (int x = 0; x < GRID; x++) {
                for (int d = 0; d < 4; d++) {
                    int nx = x + DX[d];
                    int ny = y + DY[d];
                    if (nx < 0 || nx >= GRID || ny < 0 || ny >= GRID) continue;
                    float diff = h[y][x] - h[ny][nx];
                    if (diff > TALUS) {
                        float move = EROSION_RATE * (diff - TALUS);
                        h[y][x]   -= move;
                        h[ny][nx] += move;
                    }
                }
            }
        }
    }
    t->erode_count++;
}

static void terrain_init(Terrain *t)
{
    memset(t, 0, sizeof *t);
    terrain_generate(t);
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * The running session: the land itself plus the two switches the user flips
 * while watching it.
 *   terrain   the landscape being simulated (built and worn down in §4).
 *   erode     is erosion turned on? toggled with the 'e' key.
 *   paused    is everything frozen? toggled with the spacebar. */
typedef struct {
    Terrain terrain;
    bool    erode;
    bool    paused;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    s->erode = true;
    terrain_init(&s->terrain);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)dt; (void)cols; (void)rows;
    if (s->paused) return;
    if (s->erode)  terrain_erode(&s->terrain);
}

/* §5  effects — nothing stored; colours and glyphs are worked out while drawing */

/* §6  delays — nothing but the pause flag, which Scene already carries */

/* §7  render — reads the heightmap and paints it; never changes it */

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1, 196, COLOR_BLACK);   /* red     */
        init_pair(2, 208, COLOR_BLACK);   /* orange  */
        init_pair(3, 226, COLOR_BLACK);   /* yellow  */
        init_pair(4,  46, COLOR_BLACK);   /* green   */
        init_pair(5,  51, COLOR_BLACK);   /* cyan    */
        init_pair(6, 33, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
        init_pair(8, 226, COLOR_BLACK);   /* yellow  — HUD */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(8, COLOR_YELLOW,  COLOR_BLACK);   /* HUD */
    }
}

/* ── Drawing ────────────────────────────────────────────────────────── */

/* The 65x65 grid rarely matches the terminal's size, so a screen cell usually
 * lands between four grid points. This blends those four heights by how close
 * the cell is to each — a smooth read that avoids blocky stair-steps at any
 * window size. */
static float sample_height(const Terrain *t, int row, int col, int rows, int cols)
{
    float gy_f = (float)(row - 1) / (float)(rows - 2) * (float)(GRID - 1);
    int   gy   = (int)gy_f;
    if (gy > GRID - 2) gy = GRID - 2;
    float ty = gy_f - (float)gy;

    float gx_f = (float)col / (float)(cols - 1) * (float)(GRID - 1);
    int   gx   = (int)gx_f;
    if (gx > GRID - 2) gx = GRID - 2;
    float tx = gx_f - (float)gx;

    return t->hmap[gy    ][gx    ] * (1.0f - tx) * (1.0f - ty)
         + t->hmap[gy    ][gx + 1] * tx          * (1.0f - ty)
         + t->hmap[gy + 1][gx    ] * (1.0f - tx) * ty
         + t->hmap[gy + 1][gx + 1] * tx          * ty;
}

/* Turns a height into what you actually see: which character to draw, and its
 * colour. Low ground is water, high ground is snow, with bands in between — the
 * list of cutoffs below is the whole map legend. */
static char terrain_glyph(float v, int *pair, chtype *attr)
{
    if      (v < 0.20f) { *pair = 6; *attr = A_DIM;  return '~'; }  /* deep water */
    else if (v < 0.30f) { *pair = 6; *attr = 0;      return '~'; }  /* water      */
    else if (v < 0.40f) { *pair = 3; *attr = A_DIM;  return '.'; }  /* coast      */
    else if (v < 0.52f) { *pair = 4; *attr = 0;      return '-'; }  /* plains     */
    else if (v < 0.65f) { *pair = 4; *attr = A_BOLD; return '^'; }  /* hills      */
    else if (v < 0.78f) { *pair = 2; *attr = 0;      return '#'; }  /* mountains  */
    else                { *pair = 5; *attr = A_BOLD; return '*'; }  /* peaks/snow */
}

/* Paints the whole map: for each screen cell, look up the blended height there
 * and stamp down the matching character and colour. */
static void terrain_draw(const Terrain *t, WINDOW *w, int cols, int rows)
{
    if (cols < 2 || rows < 3) return;

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float  v = sample_height(t, row, col, rows, cols);
            int    pair;
            chtype attr;
            char   ch = terrain_glyph(v, &pair, &attr);

            wattron(w, COLOR_PAIR(pair) | attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)ch);
            wattroff(w, COLOR_PAIR(pair) | attr);
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    terrain_draw(&s->terrain, w, cols, rows);
}

/* Screen — just how big the terminal currently is, in character cells. We read
 * this at startup and again whenever the window is resized, so the map can be
 * stretched to fill whatever space is available. */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  erode:%s (%d)  %s ",
             fps, sim_fps,
             sc->erode ? "on " : "off", sc->terrain.erode_count,
             sc->paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(4) | A_BOLD);
    mvprintw(0, 1, " TERRAIN ");
    attroff(COLOR_PAIR(4) | A_BOLD);

    attron(COLOR_PAIR(8) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  r:regenerate  e:erosion  [/]:Hz ");
    attroff(COLOR_PAIR(8) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* §8  app — the main loop that ties it together and handles keys */

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

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': s->paused = !s->paused; break;
    case 'r': case 'R': terrain_generate(&s->terrain); break;
    case 'e': case 'E': s->erode = !s->erode; break;
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
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
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
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        /* If the program was stalled (window dragged, machine busy), don't try
         * to "catch up" all the missed time at once — it would freeze. Cap it. */
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec,
                       app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        float alpha = (float)sim_accum / (float)tick_ns;

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

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps, alpha, dt_sec);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
