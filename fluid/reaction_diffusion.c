/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_diffusion.c  —  Gray-Scott reaction-diffusion Turing patterns
 *
 * Two chemicals U and V react and diffuse on the terminal grid:
 *
 *   dU/dt = Du·∇²U  −  U·V²  +  f·(1−U)
 *   dV/dt = Dv·∇²V  +  U·V²  −  (f+k)·V
 *
 *   U — "substrate" (feed chemical, starts at 1)
 *   V — "catalyst"  (pattern chemical, starts at 0)
 *   f — feed rate   (replenishes U)
 *   k — kill rate   (removes V)
 *
 * Laplacian: 9-point isotropic stencil (more round than 4-point):
 *   L(u) = 0.20·(N+S+E+W) + 0.05·(NE+NW+SE+SW) − u
 *
 * The (f,k) parameter pair selects the visual regime — small shifts produce
 * radically different patterns (spots, stripes, coral, mitosis, worms …).
 *
 * After seeding, 600 steps are pre-run before the first frame so the
 * screen shows a developing pattern immediately.
 *
 * Presets: Mitosis  Coral  Stripes  Worms  Maze  Bubbles  Solitons
 *
 * Keys:
 *   q / ESC   quit
 *   space     pause / resume
 *   n / p     next / prev preset
 *   t         next colour theme
 *   r         reseed (keep preset)
 *   s         drop a seed blob at centre
 *   + / -     more / fewer sim steps per frame
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra fluid/reaction_diffusion.c \
 *       -o reaction_diffusion -lncurses -lm
 *
 * Sections:  §1 config  §2 clock  §3 theme  §4 grid  §5 scene
 *            §6 screen  §7 app
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
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define DU              0.20f
#define DV              0.10f
#define DT              1.00f       /* explicit Euler timestep              */
#define STEPS_DEFAULT     16        /* sim steps per render frame           */
#define STEPS_MIN          1
#define STEPS_MAX         64
#define STEPS_STEP         4
#define WARMUP_STEPS     600        /* pre-run before first frame           */
#define SEED_HALF          3        /* half-size of seed square (cells)     */
#define AUTO_CYCLE_FRAMES 800       /* frames before auto-switching theme   */
#define V_DISPLAY_SCALE   2.2f      /* stretch V for display contrast       */

#define RAMP_N   8
#define CP_BASE  1                  /* color pairs: CP_BASE … CP_BASE+N-1  */
#define N_THEMES 4

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define RENDER_FPS          30
#define RENDER_NS   (NS_PER_SEC / RENDER_FPS)

/* ── presets ─────────────────────────────────────────────────────────── */

typedef struct {
    const char *name;
    float       f, k;
    int         n_seeds;    /* number of seed blobs                        */
} Preset;

static const Preset k_presets[] = {
    { "Mitosis",   0.0367f, 0.0649f, 4 },  /* spots that divide          */
    { "Coral",     0.0545f, 0.0630f, 5 },  /* branching coral growth     */
    { "Stripes",   0.0600f, 0.0620f, 3 },  /* labyrinthine stripes       */
    { "Worms",     0.0620f, 0.0610f, 6 },  /* winding worm tendrils      */
    { "Maze",      0.0290f, 0.0570f, 8 },  /* fine maze texture          */
    { "Bubbles",   0.0940f, 0.0590f, 3 },  /* round bubble lattice       */
    { "Solitons",  0.0250f, 0.0500f, 4 },  /* drifting soliton blobs     */
};
#define N_PRESETS (int)(sizeof k_presets / sizeof k_presets[0])

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
/* §3  theme + rendering                                                  */
/* ===================================================================== */

/*
 * ASCII ramp: 8 levels ordered by visual density.
 * Mapped to scaled V in [0, 1].
 */
static const char k_ramp[RAMP_N] = { ' ', '.', ':', '-', '+', '*', '#', '@' };

static const float k_breaks[RAMP_N] = {
    0.00f,  /* ' ' background      */
    0.10f,  /* '.' faint           */
    0.24f,  /* ':' low             */
    0.38f,  /* '-' mid-low         */
    0.52f,  /* '+' mid             */
    0.65f,  /* '*' mid-high        */
    0.78f,  /* '#' hot             */
    0.90f,  /* '@' peak            */
};

static int ramp_idx(float v)
{
    /* Highest threshold v meets or exceeds */
    for (int i = RAMP_N - 1; i >= 0; i--)
        if (v >= k_breaks[i]) return i;
    return 0;
}

typedef struct {
    const char *name;
    int         fg256[RAMP_N];
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "ocean",  { 232,  17,  19,  21,  27,  33,  51, 231 } },
    { "forest", { 232,  22,  28,  34,  40,  46, 118, 231 } },
    { "magma",  { 232,  52,  88, 124, 160, 196, 214, 231 } },
    { "violet", { 232,  54,  56,  93, 129, 165, 201, 231 } },
};

static void theme_apply(int t)
{
    for (int i = 0; i < RAMP_N; i++) {
        if (COLORS >= 256)
            init_pair(CP_BASE + i, k_themes[t].fg256[i], COLOR_BLACK);
        else {
            /* 8-color fallback */
            static const int fb[RAMP_N] = {
                COLOR_BLACK, COLOR_BLUE, COLOR_BLUE,  COLOR_CYAN,
                COLOR_CYAN,  COLOR_WHITE, COLOR_WHITE, COLOR_WHITE };
            init_pair(CP_BASE + i, fb[i], COLOR_BLACK);
        }
    }
}

static void color_init(int theme)
{
    start_color();
    theme_apply(theme);
}

static attr_t cell_attr(int idx)
{
    attr_t a = COLOR_PAIR(CP_BASE + idx);
    if (idx >= RAMP_N - 2) a |= A_BOLD;
    return a;
}

/* ===================================================================== */
/* §4  grid                                                               */
/* ===================================================================== */

typedef struct {
    float *u,  *v;          /* current chemical concentrations             */
    float *u2, *v2;         /* scratch (swap after each step)              */
    int    cols, rows;
    int    preset;
    int    theme;
    int    cycle_frame;     /* frames since last theme change              */
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    size_t n = (size_t)(cols * rows);
    g->cols = cols;  g->rows = rows;
    g->u  = malloc(n * sizeof(float));
    g->v  = malloc(n * sizeof(float));
    g->u2 = malloc(n * sizeof(float));
    g->v2 = malloc(n * sizeof(float));
}

static void grid_free(Grid *g)
{
    free(g->u); free(g->v); free(g->u2); free(g->v2);
    memset(g, 0, sizeof *g);
}

static void grid_resize(Grid *g, int cols, int rows)
{
    int p = g->preset, th = g->theme;
    grid_free(g);
    grid_alloc(g, cols, rows);
    g->preset = p;  g->theme = th;
}

/*
 * grid_seed() — reset U=1, V=0 everywhere, then place n_seeds square blobs
 * of V=1 distributed across the grid.  After seeding, run WARMUP_STEPS
 * sim steps so the first rendered frame already shows a developing pattern.
 */
static void grid_tick(Grid *g);   /* forward declaration for warmup        */

static void grid_seed(Grid *g)
{
    int cols = g->cols, rows = g->rows, n = cols * rows;
    for (int i = 0; i < n; i++) { g->u[i] = 1.f; g->v[i] = 0.f; }

    const Preset *p = &k_presets[g->preset];
    int ns = p->n_seeds;

    for (int s = 0; s < ns; s++) {
        /* spread seeds evenly with jitter */
        int cx = (int)((float)(s * cols) / (float)ns
                       + (float)(cols / ns) * 0.5f
                       + (float)(rand() % (cols / (ns + 1))) - cols / (2*(ns+1)));
        int cy = rows / 2
               + (int)((float)(rand() % (rows / 2)) - (float)(rows / 4));
        cx = ((cx % cols) + cols) % cols;
        cy = ((cy % rows) + rows) % rows;

        for (int dy = -SEED_HALF; dy <= SEED_HALF; dy++) {
            for (int dx = -SEED_HALF; dx <= SEED_HALF; dx++) {
                int x = ((cx+dx)%cols+cols)%cols;
                int y = ((cy+dy)%rows+rows)%rows;
                g->u[y*cols+x] = 0.f;
                g->v[y*cols+x] = 1.f;
            }
        }
    }

    /* fast-forward so the pattern is already developing on first render */
    for (int i = 0; i < WARMUP_STEPS; i++) grid_tick(g);
}

static void grid_drop_seed(Grid *g, int cx, int cy)
{
    int cols = g->cols, rows = g->rows;
    for (int dy = -SEED_HALF; dy <= SEED_HALF; dy++) {
        for (int dx = -SEED_HALF; dx <= SEED_HALF; dx++) {
            int x = ((cx+dx)%cols+cols)%cols;
            int y = ((cy+dy)%rows+rows)%rows;
            g->u[y*cols+x] = 0.f;
            g->v[y*cols+x] = 1.f;
        }
    }
}

/*
 * grid_tick() — one Gray-Scott step with the 9-point isotropic Laplacian.
 *
 * 9-point stencil weights (sum = 0 by construction):
 *   cardinal neighbours (N/S/E/W):    0.20
 *   diagonal neighbours (NE/NW/…):    0.05
 *   centre:                          −1.00
 *
 * This gives more isotropic diffusion than the 4-point stencil, suppressing
 * the grid-aligned artefacts that appear in spots and stripes presets.
 */
static void grid_tick(Grid *g)
{
    int   cols = g->cols, rows = g->rows;
    float f    = k_presets[g->preset].f;
    float k    = k_presets[g->preset].k;

    for (int y = 0; y < rows; y++) {
        int ym = (y == 0)       ? rows-1 : y-1;
        int yp = (y == rows-1)  ? 0      : y+1;
        for (int x = 0; x < cols; x++) {
            int xm = (x == 0)      ? cols-1 : x-1;
            int xp = (x == cols-1) ? 0      : x+1;

            int i = y*cols + x;
            float u = g->u[i], v = g->v[i];

            float Lu =
                0.20f * (g->u[y*cols+xp]  + g->u[y*cols+xm]  +
                         g->u[yp*cols+x]  + g->u[ym*cols+x])
              + 0.05f * (g->u[yp*cols+xp] + g->u[yp*cols+xm] +
                         g->u[ym*cols+xp] + g->u[ym*cols+xm])
              - u;

            float Lv =
                0.20f * (g->v[y*cols+xp]  + g->v[y*cols+xm]  +
                         g->v[yp*cols+x]  + g->v[ym*cols+x])
              + 0.05f * (g->v[yp*cols+xp] + g->v[yp*cols+xm] +
                         g->v[ym*cols+xp] + g->v[ym*cols+xm])
              - v;

            float uvv = u * v * v;
            float nu  = u + DT * (DU*Lu - uvv + f*(1.f - u));
            float nv  = v + DT * (DV*Lv + uvv - (f + k)*v);

            g->u2[i] = nu < 0.f ? 0.f : nu > 1.f ? 1.f : nu;
            g->v2[i] = nv < 0.f ? 0.f : nv > 1.f ? 1.f : nv;
        }
    }

    /* swap current ↔ scratch */
    float *tmp;
    tmp = g->u;  g->u = g->u2;  g->u2 = tmp;
    tmp = g->v;  g->v = g->v2;  g->v2 = tmp;
}

/*
 * grid_draw() — map V concentration to char+color and render.
 *
 * V is scaled by V_DISPLAY_SCALE before the ramp lookup so the pattern
 * occupies the full brightness range even though peak V is typically 0.3–0.5.
 * Auto-cycles the colour theme every AUTO_CYCLE_FRAMES frames.
 */
static bool grid_draw(Grid *g, int tcols, int trows)
{
    bool theme_changed = false;
    g->cycle_frame++;
    if (g->cycle_frame >= AUTO_CYCLE_FRAMES) {
        g->cycle_frame = 0;
        g->theme = (g->theme + 1) % N_THEMES;
        theme_apply(g->theme);
        theme_changed = true;
    }

    int cols = g->cols, rows = g->rows;
    for (int y = 0; y < rows && y < trows; y++) {
        for (int x = 0; x < cols && x < tcols; x++) {
            float v   = g->v[y*cols+x] * V_DISPLAY_SCALE;
            if (v > 1.f) v = 1.f;
            int   idx = ramp_idx(v);
            if (idx == 0) continue;   /* background — leave terminal cell untouched */
            attr_t attr = cell_attr(idx);
            attron(attr);
            mvaddch(y, x, (chtype)(unsigned char)k_ramp[idx]);
            attroff(attr);
        }
    }
    return theme_changed;
}

static void grid_init(Grid *g, int cols, int rows, int preset, int theme)
{
    grid_alloc(g, cols, rows);
    g->preset      = preset;
    g->theme       = theme;
    g->cycle_frame = 0;
    grid_seed(g);
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

typedef struct {
    Grid  grid;
    bool  paused;
    bool  needs_clear;
    int   steps;           /* sim steps per render frame                   */
} Scene;

static void scene_init(Scene *s, int cols, int rows, int preset, int theme)
{
    memset(s, 0, sizeof *s);
    s->steps = STEPS_DEFAULT;
    grid_init(&s->grid, cols, rows, preset, theme);
}

static void scene_free(Scene *s)  { grid_free(&s->grid); }

static void scene_set_preset(Scene *s, int p)
{
    s->grid.preset      = p;
    s->grid.cycle_frame = 0;
    grid_seed(&s->grid);
    s->needs_clear = true;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    grid_resize(&s->grid, cols, rows);
    grid_seed(&s->grid);
    s->needs_clear = true;
}

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    for (int i = 0; i < s->steps; i++) grid_tick(&s->grid);
}

static void scene_draw(Scene *s, int cols, int rows)
{
    bool changed = grid_draw(&s->grid, cols, rows);
    if (changed) s->needs_clear = true;
}

/* ===================================================================== */
/* §6  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s, int theme)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init(theme);
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw(Screen *s, Scene *sc, double fps)
{
    if (sc->needs_clear) {
        erase();
        sc->needs_clear = false;
    }
    scene_draw(sc, s->cols, s->rows);

    /* HUD — drawn over the sim using the brightest ramp color */
    const Grid *g  = &sc->grid;
    attr_t      ha = COLOR_PAIR(CP_BASE + RAMP_N - 1) | A_BOLD;
    attron(ha);
    mvprintw(0, 0, " Reaction-Diffusion  q quit  n/p preset  t theme  "
                   "r reseed  s seed  +/- speed  spc pause");
    attroff(ha);
    attron(COLOR_PAIR(CP_BASE + 3));
    mvprintw(1, 0, " %-8s  theme:%-6s  steps/frame:%2d  fps:%.0f  %s",
             k_presets[g->preset].name,
             k_themes[g->theme].name,
             sc->steps, fps,
             sc->paused ? "[PAUSED]" : "");
    attroff(COLOR_PAIR(CP_BASE + 3));
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §7  app                                                                */
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

    case ' ':  sc->paused = !sc->paused;  break;

    case 'n': case 'N':
        scene_set_preset(sc, (g->preset + 1) % N_PRESETS);           break;
    case 'p': case 'P':
        scene_set_preset(sc, (g->preset + N_PRESETS - 1) % N_PRESETS); break;

    case 't': case 'T':
        g->theme = (g->theme + 1) % N_THEMES;
        g->cycle_frame = 0;
        theme_apply(g->theme);
        sc->needs_clear = true;
        break;

    case 'r': case 'R':
        grid_seed(g);
        sc->needs_clear = true;
        break;

    case 's': case 'S':
        grid_drop_seed(g, g->cols/2, g->rows/2);
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
    srand((unsigned)time(NULL));
    atexit(cleanup);
    signal(SIGINT,   on_exit);
    signal(SIGTERM,  on_exit);
    signal(SIGWINCH, on_resize);

    App *app     = &g_app;
    app->running = 1;

    screen_init(&app->screen, 0);
    scene_init(&app->scene, app->screen.cols, app->screen.rows, 0, 0);

    int64_t ft = clock_ns(), fa = 0; int fc = 0; double fpsd = 0.;

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
