/*
 * plasma.c — Demoscene Plasma / Colour Wave
 *
 * Classic demoscene plasma: sums of sinusoids at each terminal cell
 * mapped through a cycling colour palette.
 *
 *   v(col, row, t) = sin(col·f1 + t·s1)
 *                  + sin(row·f2 + t·s2)
 *                  + sin((col+row)·f3 + t·s3)
 *                  + sin(√(dx²+(2·dy)²)·f4 + t·s4)
 * Normalised to [0,1], then shifted by a phase that rotates at CYCLE_HZ
 * cycles/second — colours flow across the screen without physics.
 *
 * Four frequency presets ('f') give distinct wave structures.
 * Four colour themes ('p') select different palette mappings.
 *
 * Keys:
 *   q/ESC quit   space pause   p next palette   f next frequencies
 *   ] / [   sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra plasma.c -o plasma -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
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

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,   /* plasma recomputes every cell; 30 is plenty */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
    N_FREQ_PRESETS  = 4,
    N_THEMES        = 4,
    N_PAL           = 14,   /* palette entries per theme                  */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CYCLE_HZ    0.20f   /* palette phase cycles per second            */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

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
        init_pair(6,  21, COLOR_BLACK);   /* blue    */
        init_pair(7, 201, COLOR_BLACK);   /* magenta */
    } else {
        init_pair(1, COLOR_RED,     COLOR_BLACK);
        init_pair(2, COLOR_RED,     COLOR_BLACK);
        init_pair(3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(6, COLOR_BLUE,    COLOR_BLACK);
        init_pair(7, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §4  coords — plasma works directly in cell space                       */
/* ===================================================================== */

/* ===================================================================== */
/* §5  entity — Plasma                                                    */
/* ===================================================================== */

typedef struct {
    int    pair;
    chtype attr;
    char   ch;
} PalEntry;

/* Spatial frequencies (rad/cell) and time speeds (rad/s) */
typedef struct {
    float f1, f2, f3, f4;
    float s1, s2, s3, s4;
    const char *name;
} FreqPreset;

static const FreqPreset FREQ_PRESETS[N_FREQ_PRESETS] = {
    { 0.20f, 0.25f, 0.15f, 0.18f,  1.0f, 0.80f, 1.2f, 0.90f, "gentle"    },
    { 0.40f, 0.35f, 0.30f, 0.22f,  1.5f, 1.30f, 1.8f, 1.20f, "energetic" },
    { 0.10f, 0.12f, 0.08f, 0.09f,  0.4f, 0.30f, 0.5f, 0.35f, "grand"     },
    { 0.35f, 0.28f, 0.45f, 0.20f,  2.0f, 1.80f, 2.2f, 1.60f, "turbulent" },
};

static const char *THEME_NAMES[N_THEMES] = {
    "rainbow", "fire", "ocean", "matrix"
};

/*
 * Palette entries per theme.  N_PAL=14 entries arranged so that cycling
 * the plasma phase at CYCLE_HZ makes colours flow smoothly across the
 * screen.  Pair numbers map to §3 color_init() definitions.
 */
static const PalEntry THEMES[N_THEMES][N_PAL] = {
    /* 0: rainbow — blue → cyan → green → yellow → orange → red → magenta */
    {
        {6, A_DIM,  '.'}, {6, 0,      ':'},
        {5, 0,      '+'}, {5, A_BOLD, '+'},
        {4, 0,      '+'}, {4, A_BOLD, '*'},
        {3, 0,      '*'}, {3, A_BOLD, '*'},
        {2, A_BOLD, '#'}, {2, 0,      '#'},
        {1, A_BOLD, '#'}, {1, 0,      ':'},
        {7, 0,      '.'}, {7, A_DIM,  '.'},
    },
    /* 1: fire — dark red → red → orange → yellow */
    {
        {1, A_DIM,  '.'}, {1, A_DIM,  '.'},
        {1, 0,      ':'}, {1, A_BOLD, ':'},
        {2, A_DIM,  '+'}, {2, 0,      '+'},
        {2, A_BOLD, '*'}, {2, A_BOLD, '*'},
        {3, A_DIM,  '#'}, {3, 0,      '#'},
        {3, A_BOLD, '#'}, {3, A_BOLD, '@'},
        {3, A_BOLD, '@'}, {3, A_BOLD, '@'},
    },
    /* 2: ocean — deep blue → cyan → teal */
    {
        {6, A_DIM,  '.'}, {6, A_DIM,  '.'},
        {6, 0,      ':'}, {6, 0,      ':'},
        {6, A_BOLD, ':'}, {5, A_DIM,  '+'},
        {5, 0,      '+'}, {5, A_BOLD, '*'},
        {5, A_BOLD, '#'}, {4, A_DIM,  '#'},
        {4, 0,      '#'}, {4, A_BOLD, '@'},
        {4, A_BOLD, '@'}, {4, A_BOLD, '@'},
    },
    /* 3: matrix — shades of green only */
    {
        {4, A_DIM,  '.'}, {4, A_DIM,  '.'},
        {4, A_DIM,  ':'}, {4, 0,      ':'},
        {4, 0,      '+'}, {4, 0,      '+'},
        {4, 0,      '*'}, {4, A_BOLD, '*'},
        {4, A_BOLD, '#'}, {4, A_BOLD, '#'},
        {4, A_BOLD, '@'}, {4, A_BOLD, '@'},
        {4, A_BOLD, '@'}, {4, A_BOLD, '@'},
    },
};

typedef struct {
    float time;
    int   freq_preset;
    int   theme;
    bool  paused;
} Plasma;

static void plasma_init(Plasma *p)
{
    p->time        = 0.0f;
    p->freq_preset = 0;
    p->theme       = 0;
    p->paused      = false;
}

static void plasma_tick(Plasma *p, float dt)
{
    if (p->paused) return;
    p->time += dt;
}

/*
 * plasma_value — evaluate the plasma formula at one cell.
 *
 * Four sinusoid terms:
 *   • horizontal wave
 *   • vertical wave
 *   • diagonal wave
 *   • radial wave (aspect-corrected: dy × 2 so rings look circular)
 * Sum ∈ [−4, +4]; normalise to [0, 1].
 */
static float plasma_value(int col, int row, int cols, int rows,
                          float t, const FreqPreset *fp)
{
    float cx   = (float)cols * 0.5f;
    float cy   = (float)rows * 0.5f;
    float dx   = (float)col - cx;
    float dy   = ((float)row - cy) * 2.0f;  /* aspect correction */
    float dist = sqrtf(dx * dx + dy * dy);

    float v = sinf((float)col * fp->f1 + t * fp->s1)
            + sinf((float)row * fp->f2 + t * fp->s2)
            + sinf(((float)(col + row)) * fp->f3 + t * fp->s3)
            + sinf(dist * fp->f4 + t * fp->s4);

    return (v + 4.0f) * 0.125f;   /* [0, 1] */
}

static void plasma_draw(const Plasma *p, WINDOW *w, int cols, int rows)
{
    const FreqPreset *fp  = &FREQ_PRESETS[p->freq_preset];
    const PalEntry   *pal = THEMES[p->theme];
    float phase = fmodf(p->time * CYCLE_HZ, 1.0f);

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float v  = plasma_value(col, row, cols, rows, p->time, fp);
            float vs = fmodf(v + phase, 1.0f);
            int   idx = (int)(vs * (float)N_PAL);
            if (idx < 0)      idx = 0;
            if (idx >= N_PAL) idx = N_PAL - 1;

            const PalEntry *e = &pal[idx];
            wattron(w, COLOR_PAIR(e->pair) | e->attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)e->ch);
            wattroff(w, COLOR_PAIR(e->pair) | e->attr);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Plasma plasma; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    plasma_init(&s->plasma);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    plasma_tick(&s->plasma, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    plasma_draw(&s->plasma, w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

    const Plasma *p = &sc->plasma;
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  theme:%s  freq:%s ",
             fps, sim_fps, THEME_NAMES[p->theme],
             FREQ_PRESETS[p->freq_preset].name);
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " PLASMA ");
    attroff(COLOR_PAIR(5) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0,
             " q:quit  spc:pause  p:palette  f:frequencies  [/]:Hz ");
    attroff(COLOR_PAIR(6) | A_DIM);
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

static bool app_handle_key(App *app, int ch)
{
    Plasma *p = &app->scene.plasma;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': p->paused = !p->paused; break;
    case 'p': case 'P':
        p->theme = (p->theme + 1) % N_THEMES;
        break;
    case 'f': case 'F':
        p->freq_preset = (p->freq_preset + 1) % N_FREQ_PRESETS;
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
