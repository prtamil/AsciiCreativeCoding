/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wave.c  —  2-D wave equation interference patterns
 *
 * Simulates the scalar 2-D wave equation on the terminal grid:
 *
 *   ∂²u/∂t² = c² · ∇²u
 *
 * Discretised with the explicit 2nd-order FDTD scheme:
 *
 *   u_new[x,y] = 2·u[x,y] − u_prev[x,y]
 *              + c² · (u[x+1,y]+u[x−1,y]+u[x,y+1]+u[x,y−1] − 4·u[x,y])
 *
 * then multiplied by a damping factor so energy gradually dissipates.
 *
 * CFL stability condition for 2-D: c·√2 ≤ 1  →  c ≤ 0.707
 * We use c = 0.45 for a comfortable stability margin.
 *
 * Five oscillating point sources can be toggled with keys 1–5.
 * Each source has a slightly different frequency so neighbouring pairs
 * create beat patterns and slowly shifting interference fringes.
 *
 * Amplitude is mapped to a signed 9-level ramp:
 *   negative (troughs)  →  dim/cool colours
 *   near zero           →  blank (terminal background shows through)
 *   positive (crests)   →  bright/warm colours
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   1 – 5     toggle source on / off
 *   r         clear grid (all amplitudes → 0)
 *   p         drop a single impulse at centre
 *   t         next colour theme
 *   d / D     more / less damping
 *   + / -     more / fewer sim steps per frame
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/wave.c -o wave -lncurses -lm
 *
 * Sections:  §1 config  §2 clock  §3 colour  §4 grid  §5 sources
 *            §6 scene   §7 screen §8 app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : FDTD (Finite Difference Time Domain) integration of the
 *                  scalar 2-D wave equation using the 2nd-order explicit scheme.
 *                  Unlike the wave_2d.c sponge-boundary variant, this one uses
 *                  multiple independent oscillating sources that drive the field
 *                  continuously, creating persistent standing/travelling waves.
 *
 * Physics        : ∂²u/∂t² = c²·∇²u — the classical wave equation.
 *                  Discretised: u_new = 2u − u_prev + c²·(u_N + u_S + u_E + u_W − 4u)
 *                  Each of the 5 point sources runs at a slightly different
 *                  frequency, producing beat patterns and slowly drifting
 *                  interference fringes between adjacent source pairs.
 *
 * Math           : CFL stability condition for 2-D: c · √2 ≤ 1 → c ≤ 0.707.
 *                  C_SPEED = 0.45 gives CFL number = 0.45·√2 ≈ 0.636.
 *                  Energy dissipation: multiply by DAMPING_DEFAULT = 0.993 per tick.
 *                  Without damping, energy accumulates until numerical overflow.
 *
 * Performance    : O(W×H) per step.  STEPS_DEFAULT=4 sub-steps per render
 *                  frame advance the simulation faster without changing CFL.
 *                  The grid uses three flat arrays (prev/cur/new) for cache
 *                  efficiency; 2D index is y*W + x (row-major).
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define C_SPEED         0.45f    /* wave speed in cells/tick; CFL: c·√2 ≤ 1
                                  * → c ≤ 0.707; 0.45 gives CFL=0.636 (stable) */
#define C_SQ            (C_SPEED * C_SPEED)   /* precomputed for the FDTD update  */
#define DAMPING_DEFAULT 0.993f   /* per-tick amplitude retention; 0.993^(4×30)=0.70
                                  * per second — gentle energy drain without
                                  * over-damping the visible interference pattern */
#define DAMPING_STEP    0.002f
#define DAMPING_MIN     0.960f   /* 0.960^120 ≈ 0.007 — heavy absorption        */
#define DAMPING_MAX     0.999f   /* near-lossless — energy builds slowly         */

#define STEPS_DEFAULT   4        /* sim sub-steps per render frame; higher values
                                  * give faster-travelling waves on screen       */
#define STEPS_MIN       1
#define STEPS_MAX       16
#define STEPS_STEP      1

#define SOURCE_AMP      3.0f     /* oscillating source amplitude in field units  */
#define IMPULSE_AMP     6.0f     /* single-tap amplitude; 2× source for visibility */
#define IMPULSE_RADIUS  3        /* cells around tap that get initialised         */

#define MAX_AMP         4.0f     /* display ceiling for normalisation: u/MAX_AMP
                                  * maps field amplitude to the 8-level colour ramp */
#define ZERO_BAND       0.04f    /* |u/MAX_AMP| below this → blank         */

#define N_LEVELS        4        /* ramp levels per sign (trough / crest)  */
#define N_CP            8        /* colour pairs: N_LEVELS * 2             */
#define CP_BASE         1        /* pairs CP_BASE … CP_BASE+N_CP-1         */
#define CP_HUD          (CP_BASE + N_CP)
#define N_THEMES        4

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define RENDER_FPS              30
#define RENDER_NS       (NS_PER_SEC / RENDER_FPS)

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
    struct timespec r = { (time_t)(ns/NS_PER_SEC), (long)(ns%NS_PER_SEC) };
    nanosleep(&r, NULL);
}

/* ===================================================================== */
/* §3  colour                                                             */
/* ===================================================================== */

/*
 * 8 colour pairs: pairs 1-4 for troughs (level -4 … -1),
 *                 pairs 5-8 for crests  (level +1 … +4).
 * Darkest trough = pair 1, brightest crest = pair 8.
 */
typedef struct { const char *name; int fg[N_CP]; } Theme;

static const Theme k_themes[N_THEMES] = {
    { "water",  {  17,  20,  27,  39,   45,  51, 159, 231 } },
    { "lava",   {  52,  88, 124, 160,  208, 214, 220, 231 } },
    { "plasma", {  53,  56,  93, 129,  165, 171, 207, 231 } },
    { "matrix", {  22,  28,  34,  40,   46,  82, 118, 231 } },
};

/*
 * ASCII chars indexed by level (1=faintest trough … 8=brightest crest).
 * Chosen so troughs feel hollow/receding and crests feel solid/rising.
 */
static const char k_chars[N_CP] = { ',', '.', '-', '~',
                                     '+', '*', '#', '@' };

static void theme_apply(int t)
{
    const Theme *th = &k_themes[t];
    for (int i = 0; i < N_CP; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE + i, th->fg[i], COLOR_BLACK);
        else {
            /* 8-color fallback: troughs dim, crests bold */
            int c = (i < N_LEVELS) ? COLOR_BLUE : COLOR_CYAN;
            init_pair(CP_BASE + i, c, COLOR_BLACK);
        }
    }
    init_pair(CP_HUD, COLORS >= 256 ? 244 : COLOR_WHITE, COLOR_BLACK);
}

static void color_init(int theme) { start_color(); theme_apply(theme); }

/*
 * amplitude_level() — map normalised amplitude n ∈ [-1,1] to pair index.
 *
 *  n < -ZERO_BAND  →  trough levels 0..3  (pairs CP_BASE+0 … CP_BASE+3)
 *  |n| ≤ ZERO_BAND →  -1 (draw nothing)
 *  n >  ZERO_BAND  →  crest  levels 4..7  (pairs CP_BASE+4 … CP_BASE+7)
 */
static int amplitude_level(float u)
{
    float n = u / MAX_AMP;
    if (n >  1.f) n =  1.f;
    if (n < -1.f) n = -1.f;

    float abs_n = n < 0.f ? -n : n;
    if (abs_n <= ZERO_BAND) return -1;

    /* Map abs_n from (ZERO_BAND, 1] to level index 0..N_LEVELS-1 */
    float t = (abs_n - ZERO_BAND) / (1.f - ZERO_BAND);
    int   lv = (int)(t * (float)N_LEVELS);
    if (lv >= N_LEVELS) lv = N_LEVELS - 1;

    return (n < 0.f) ? lv : (N_LEVELS + lv);
}

static attr_t level_attr(int lv)
{
    attr_t a = COLOR_PAIR(CP_BASE + lv);
    /* boldest positive level glows */
    if (lv == N_CP - 1) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

typedef struct {
    float *curr, *prev, *next;  /* three generations                      */
    int    cols, rows;
    float  damping;
    int    theme;
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    size_t n  = (size_t)(cols * rows);
    g->cols   = cols;  g->rows = rows;
    g->curr   = calloc(n, sizeof(float));
    g->prev   = calloc(n, sizeof(float));
    g->next   = calloc(n, sizeof(float));
}

static void grid_free(Grid *g)
{
    free(g->curr); free(g->prev); free(g->next);
    memset(g, 0, sizeof *g);
}

static void grid_clear(Grid *g)
{
    size_t n = (size_t)(g->cols * g->rows) * sizeof(float);
    memset(g->curr, 0, n);
    memset(g->prev, 0, n);
    memset(g->next, 0, n);
}

static void grid_resize(Grid *g, int cols, int rows)
{
    float  dm = g->damping;
    int    th = g->theme;
    grid_free(g);
    grid_alloc(g, cols, rows);
    g->damping = dm;
    g->theme   = th;
}

/*
 * grid_tick() — one FDTD step.
 *
 * Inner cells only (boundary rows/cols remain 0 = hard reflecting walls).
 * The Laplacian uses the standard 4-point stencil; no need for 9-point
 * because the wave equation naturally produces isotropic circles.
 */
static void grid_tick(Grid *g)
{
    int   cols = g->cols, rows = g->rows;
    float csq  = C_SQ;
    float damp = g->damping;
    float *cu  = g->curr, *pv = g->prev, *nx = g->next;

    for (int y = 1; y < rows - 1; y++) {
        int row  = y * cols;
        int rowp = row + cols;
        int rowm = row - cols;
        for (int x = 1; x < cols - 1; x++) {
            int i   = row + x;
            float lap = cu[i+1] + cu[i-1] + cu[rowp+x] + cu[rowm+x]
                        - 4.f * cu[i];
            nx[i] = (2.f*cu[i] - pv[i] + csq*lap) * damp;
        }
    }

    /* rotate: prev ← curr ← next */
    float *tmp = g->prev;
    g->prev    = g->curr;
    g->curr    = g->next;
    g->next    = tmp;
}

/*
 * grid_draw() — render amplitude as signed ASCII brightness.
 * Skips blank cells (level < 0) so the terminal background shows through.
 */
static void grid_draw(const Grid *g, int tcols, int trows)
{
    int cols = g->cols, rows = g->rows;
    const float *cu = g->curr;

    for (int y = 0; y < rows && y < trows; y++) {
        int row = y * cols;
        for (int x = 0; x < cols && x < tcols; x++) {
            int lv = amplitude_level(cu[row + x]);
            if (lv < 0) continue;
            attr_t attr = level_attr(lv);
            attron(attr);
            mvaddch(y, x, (chtype)(unsigned char)k_chars[lv]);
            attroff(attr);
        }
    }
}

/* ===================================================================== */
/* §5  sources                                                            */
/* ===================================================================== */

/*
 * Five oscillating point sources.  Each has a fractional position (so it
 * scales with terminal size) and a distinct frequency to create beat
 * interference between neighbouring active sources.
 */
#define N_SOURCES 5

typedef struct {
    float fx, fy;    /* fractional grid position [0,1] */
    float freq;      /* angular frequency in rad/tick  */
    float phase;     /* current phase                  */
    bool  active;
} Source;

/* Layout: 4 corners + centre.  Frequencies straddle a common value so
   pairs of sources produce slow beats. */
static const float k_src_fx[N_SOURCES] = { 0.22f, 0.78f, 0.50f, 0.22f, 0.78f };
static const float k_src_fy[N_SOURCES] = { 0.25f, 0.25f, 0.50f, 0.75f, 0.75f };
static const float k_src_freq[N_SOURCES] = {
    0.220f, 0.260f, 0.190f, 0.240f, 0.210f   /* rad/tick */
};

static Source g_sources[N_SOURCES];

static void sources_init(void)
{
    for (int i = 0; i < N_SOURCES; i++) {
        g_sources[i].fx     = k_src_fx[i];
        g_sources[i].fy     = k_src_fy[i];
        g_sources[i].freq   = k_src_freq[i];
        g_sources[i].phase  = 0.f;
        g_sources[i].active = false;
    }
    /* start with two sources active so there's immediate visual interest */
    g_sources[0].active = true;
    g_sources[4].active = true;
}

static void sources_inject(Source *srcs, Grid *g)
{
    int cols = g->cols, rows = g->rows;
    float *cu = g->curr;

    for (int i = 0; i < N_SOURCES; i++) {
        Source *s = &srcs[i];
        if (!s->active) continue;

        int sx = (int)(s->fx * (float)(cols - 2)) + 1;
        int sy = (int)(s->fy * (float)(rows - 2)) + 1;
        if (sx < 1 || sx >= cols-1 || sy < 1 || sy >= rows-1) continue;

        cu[sy * cols + sx] += SOURCE_AMP * sinf(s->phase);
        s->phase += s->freq;
    }
}

static void sources_rescale(Source *srcs, int cols, int rows)
{
    /* fractional positions are stable; nothing to recalculate */
    (void)srcs; (void)cols; (void)rows;
}

/* Drop a Gaussian impulse blob at (cx, cy) */
static void grid_impulse(Grid *g, int cx, int cy)
{
    int cols = g->cols, rows = g->rows;
    int R = IMPULSE_RADIUS;
    for (int dy = -R; dy <= R; dy++) {
        for (int dx = -R; dx <= R; dx++) {
            int x = cx + dx, y = cy + dy;
            if (x < 1 || x >= cols-1 || y < 1 || y >= rows-1) continue;
            float d2 = (float)(dx*dx + dy*dy);
            float w  = expf(-d2 / (float)(R * R));
            g->curr[y*cols + x] += IMPULSE_AMP * w;
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid  grid;
    bool  paused;
    bool  needs_clear;
    int   steps;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    grid_alloc(&s->grid, cols, rows);
    s->grid.damping = DAMPING_DEFAULT;
    s->grid.theme   = 0;
    s->steps        = STEPS_DEFAULT;
    sources_init();
    theme_apply(0);
}

static void scene_free(Scene *s)   { grid_free(&s->grid); }

static void scene_resize(Scene *s, int cols, int rows)
{
    sources_rescale(g_sources, cols, rows);
    grid_resize(&s->grid, cols, rows);
    s->needs_clear = true;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    for (int i = 0; i < s->steps; i++) {
        sources_inject(g_sources, &s->grid);
        grid_tick(&s->grid);
    }
}

static void scene_draw(Scene *s, int tcols, int trows)
{
    grid_draw(&s->grid, tcols, trows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(0);
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, Scene *s, double fps)
{
    if (s->needs_clear) {
        erase();
        s->needs_clear = false;
    }
    scene_draw(s, sc->cols, sc->rows);

    /* Source status bar */
    char src_buf[32];
    for (int i = 0; i < N_SOURCES; i++)
        src_buf[i] = g_sources[i].active ? ('1' + i) : '.';
    src_buf[N_SOURCES] = '\0';

    attr_t ha = COLOR_PAIR(CP_HUD) | A_BOLD;
    attron(ha);
    mvprintw(0, 0,
             " Wave  q quit  1-5 src  r clear  p impulse  t theme  d/D damp  +/-  spc pause");
    attroff(ha);
    attron(COLOR_PAIR(CP_HUD));
    mvprintw(1, 0,
             " src:[%s]  theme:%-6s  damp:%.3f  steps:%d  fps:%.0f  %s",
             src_buf,
             k_themes[s->grid.theme].name,
             s->grid.damping,
             s->steps, fps,
             s->paused ? "[PAUSED]" : "");
    attroff(COLOR_PAIR(CP_HUD));
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene  scene;
    Screen screen;
    volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit  (int s) { (void)s; g_app.running     = 0; }
static void on_resize(int s) { (void)s; g_app.need_resize = 1; }
static void cleanup  (void)  { endwin(); }

static bool app_handle_key(App *a, int ch)
{
    Scene *sc = &a->scene;
    Grid  *g  = &sc->grid;

    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': sc->paused = !sc->paused; break;

    case '1': case '2': case '3': case '4': case '5':
        g_sources[ch - '1'].active ^= true;
        break;

    case 'r': case 'R':
        grid_clear(g);
        sc->needs_clear = true;
        break;

    case 'p': case 'P':
        grid_impulse(g, g->cols/2, g->rows/2);
        break;

    case 't': case 'T':
        g->theme = (g->theme + 1) % N_THEMES;
        theme_apply(g->theme);
        sc->needs_clear = true;
        break;

    case 'd':
        g->damping -= DAMPING_STEP;
        if (g->damping < DAMPING_MIN) g->damping = DAMPING_MIN;
        break;
    case 'D':
        g->damping += DAMPING_STEP;
        if (g->damping > DAMPING_MAX) g->damping = DAMPING_MAX;
        break;

    case '+': case '=':
        sc->steps += STEPS_STEP;
        if (sc->steps > STEPS_MAX) sc->steps = STEPS_MAX;
        break;
    case '-':
        sc->steps -= STEPS_STEP;
        if (sc->steps < STEPS_MIN) sc->steps = STEPS_MIN;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t ft=clock_ns(), fa=0; int fc=0; double fpsd=0.;

    while (app->running) {
        if (app->need_resize) {
            screen_resize(&app->screen);
            scene_resize(&app->scene, app->screen.cols, app->screen.rows);
            app->need_resize = 0;
            ft = clock_ns(); fa = 0; fc = 0;
        }

        int64_t now = clock_ns(), dt = now - ft; ft = now;
        if (dt > 100*NS_PER_MS) dt = 100*NS_PER_MS;

        fc++; fa += dt;
        if (fa >= 500*NS_PER_MS) {
            fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
            fc = 0; fa = 0;
        }

        scene_tick(&app->scene);

        int64_t el = clock_ns() - ft + dt;
        clock_sleep_ns(RENDER_NS - el);

        screen_draw(&app->screen, &app->scene, fpsd);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch)) app->running = 0;
    }

    scene_free(&app->scene);
    screen_free(&app->screen);
    return 0;
}
