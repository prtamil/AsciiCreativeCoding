/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * plasma.c — demoscene plasma effect.
 *
 * Each terminal cell gets a value from adding four sine waves together, and
 * that value picks a colour+glyph from a palette.  Cycling the palette over
 * time makes the colours flow.  Classic 1990s demoscene trick: no grid, no
 * physics — every cell is computed fresh each frame (Vandevenne, lodev.org).
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

/* ── §1 config — tunable constants ── */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,   /* plasma recomputes every cell; 30 is plenty */
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FRAME_CAP_FPS   = 60,   /* hard render frame cap, independent of sim Hz */
    FPS_UPDATE_MS   = 500,
    N_FREQ_PRESETS  = 4,
    N_THEMES        = 4,
    N_PAL           = 14,   /* palette entries per theme                  */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

#define CYCLE_HZ    0.20f   /* palette phase cycles per second            */

/* ── §2 performance — clock helpers (frame timing lives in §7) ── */

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

/* ── §3 types — palette stops, wave presets, and app state ── */

/* PalEntry — one colour stop in a palette: which colour, how bright, which
 * glyph.  Each theme is a small precomputed table so mapping a cell's value to a
 * colour is just an array lookup, cheap enough to do for every cell every frame. */
typedef struct {
    int    pair;   /* ncurses colour-pair id (defined in §6 color_init)      */
    chtype attr;   /* A_DIM / 0 / A_BOLD — dimmer to brighter within a colour */
    char   ch;     /* glyph; denser glyphs ('.' up to '@') read as brighter   */
} PalEntry;

/* FreqPreset — settings for the four sine waves that make up the plasma.  The
 * same formula gives very different looks depending on these numbers, so each
 * preset is a hand-picked combination the 'f' key cycles through. */
typedef struct {
    float f1, f2, f3, f4;   /* how many ripples across the screen (rad/cell) */
    float s1, s2, s3, s4;   /* how fast each wave moves (rad/s)              */
    const char *name;       /* label shown in the HUD                        */
} FreqPreset;

/* Plasma — the whole effect's state, which is tiny because plasma keeps no
 * picture: every cell is recomputed from its position and the current time, so
 * all we store is a clock plus the three things the user can change. */
typedef struct {
    float time;          /* seconds elapsed — drives both the waves and the palette cycle */
    int   freq_preset;   /* which FREQ_PRESETS entry (wave settings)                       */
    int   theme;         /* which THEMES entry (palette of colours)                        */
    bool  paused;        /* true freezes time so the pattern holds still                   */
} Plasma;

/* Scene — wrapper holding the one effect; kept for consistency with other demos. */
typedef struct { Plasma plasma; } Scene;

/* Screen — cached terminal size so the draw loop and HUD don't keep asking
 * ncurses for it; re-read at startup and whenever the window is resized. */
typedef struct { int cols, rows; } Screen;

/* App — ties everything together at the process level.
 *
 * The running/need_resize flags are written by signal handlers and read by the
 * main loop.  Signal handlers can't take extra arguments, so they reach these
 * flags through the file-scope g_app; volatile sig_atomic_t is the only type C
 * promises is safe to touch from inside a handler. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;       /* simulation rate, changed by ] / [   */
    volatile sig_atomic_t running;       /* cleared by SIGINT/SIGTERM to exit   */
    volatile sig_atomic_t need_resize;   /* set by SIGWINCH; loop re-reads size */
} App;

/* ── §4 logic — turn a cell position into a value, no side effects ── */

/* plasma_value — adds four sine waves (left-right, up-down, diagonal, and
 * circular rings from the centre) to get one number for this cell, then squeezes
 * the result into the 0..1 range. */
static float plasma_value(int col, int row, int cols, int rows,
                          float t, const FreqPreset *fp)
{
    float cx   = (float)cols * 0.5f;
    float cy   = (float)rows * 0.5f;
    float dx   = (float)col - cx;
    float dy   = ((float)row - cy) * 2.0f;  /* ×2: cells are ~twice as tall as wide, so this keeps rings round */
    float dist = sqrtf(dx * dx + dy * dy);

    float v = sinf((float)col * fp->f1 + t * fp->s1)
            + sinf((float)row * fp->f2 + t * fp->s2)
            + sinf(((float)(col + row)) * fp->f3 + t * fp->s3)
            + sinf(dist * fp->f4 + t * fp->s4);

    return (v + 4.0f) * 0.125f;   /* four waves sum to -4..4; rescale to 0..1 */
}

/* palette_index — pick which colour stop this value lands on.  Adding `phase`
 * (which creeps up every frame) shifts every cell's colour a little, and that
 * steady shift is what makes the whole pattern appear to flow. */
static int palette_index(float v, float phase)
{
    float vs = fmodf(v + phase, 1.0f);
    int   idx = (int)(vs * (float)N_PAL);
    if (idx < 0)      idx = 0;
    if (idx >= N_PAL) idx = N_PAL - 1;
    return idx;
}

/* ── §5 simulation — the only thing that advances is the clock ── */

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

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    plasma_init(&s->plasma);
}

static void scene_tick(Scene *s, float dt)
{
    plasma_tick(&s->plasma, dt);
}

/* ── §6 render — draw the current state to the terminal ── */

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

static const FreqPreset FREQ_PRESETS[N_FREQ_PRESETS] = {
    { 0.20f, 0.25f, 0.15f, 0.18f,  1.0f, 0.80f, 1.2f, 0.90f, "gentle"    },
    { 0.40f, 0.35f, 0.30f, 0.22f,  1.5f, 1.30f, 1.8f, 1.20f, "energetic" },
    { 0.10f, 0.12f, 0.08f, 0.09f,  0.4f, 0.30f, 0.5f, 0.35f, "grand"     },
    { 0.35f, 0.28f, 0.45f, 0.20f,  2.0f, 1.80f, 2.2f, 1.60f, "turbulent" },
};

static const char *THEME_NAMES[N_THEMES] = {
    "rainbow", "fire", "ocean", "matrix"
};

/* The colour stops for each theme, ordered light to dark so the value-to-colour
 * mapping looks smooth.  Pair numbers refer to the colours set up in color_init. */
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

static void plasma_draw(const Plasma *p, WINDOW *w, int cols, int rows)
{
    const FreqPreset *fp  = &FREQ_PRESETS[p->freq_preset];
    const PalEntry   *pal = THEMES[p->theme];
    float phase = fmodf(p->time * CYCLE_HZ, 1.0f);

    for (int row = 1; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            float v = plasma_value(col, row, cols, rows, p->time, fp);
            const PalEntry *e = &pal[palette_index(v, phase)];
            wattron(w, COLOR_PAIR(e->pair) | e->attr);
            mvwaddch(w, row, col, (chtype)(unsigned char)e->ch);
            wattroff(w, COLOR_PAIR(e->pair) | e->attr);
        }
    }
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows)
{
    plasma_draw(&s->plasma, w, cols, rows);
}

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* draw_hud — status line top-right, the key reminders bottom-left. */
static void draw_hud(const Plasma *p, int cols, int rows, double fps, int sim_fps)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  theme:%s  freq:%s  %s ",
             fps, sim_fps, THEME_NAMES[p->theme],
             FREQ_PRESETS[p->freq_preset].name,
             p->paused ? "PAUSED " : "running");
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  t:theme  f:frequencies  [/]:Hz ");
    attroff(COLOR_PAIR(5) | A_BOLD);
}

static void screen_draw(const Screen *s, const Scene *sc,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows);
    draw_hud(&sc->plasma, s->cols, s->rows, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §7 app — main loop, frame timing, and key/resize handling ── */

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* app_handle_key — apply one keypress; returns false only when the user quits. */
static bool app_handle_key(App *app, int ch)
{
    Plasma *p = &app->scene.plasma;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': case 'p': case 'P': p->paused = !p->paused; break;
    case 't': case 'T':
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
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init(&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        /* window was resized: re-read the new size and restart the timer */
        if (app->need_resize) {
            endwin(); refresh();
            getmaxyx(stdscr, app->screen.rows, app->screen.cols);
            app->need_resize = 0;
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* measure how long since last frame; cap it so a long stall (e.g. laptop
         * sleep) doesn't make the loop try to catch up with a flood of ticks */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* advance the clock in fixed-size steps so the speed is the same
         * regardless of how fast frames actually render */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* update the fps number shown in the HUD a couple of times a second */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* sleep off the leftover time so we hold the frame cap */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);

        screen_draw(&app->screen, &app->scene,
                    fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
