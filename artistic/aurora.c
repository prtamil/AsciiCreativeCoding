/*
 * aurora.c — Aurora Borealis
 *
 * Layered sinusoidal curtains scrolling horizontally, coloured green/cyan
 * at the core with purple/pink fringes at the upper edge.  A second faster
 * noise octave adds brightness shimmer.  Deterministic star-hash fills the
 * background sky with static stars that show through thin aurora regions.
 *
 * Aurora formula per cell (col, row):
 *   x = col/cols × 2π           (0 → 2π across screen width)
 *   y = row / (rows × 0.65)     (0 = top, 1 = aurora bottom)
 *
 *   primary  = sin(x·1.5 + t·0.20) · cos(y·3 + t·0.50 + x·0.25)
 *   shimmer  = cos(x·2.3 − t·0.15) · sin(y·5 + t·0.80 + x·0.40)
 *   v        = primary·0.60 + shimmer·0.40   ∈ [−1, 1]
 *   envelope = sin(π · y)                    0 at y=0,1; peak at y=0.5
 *   intensity = (v·0.5 + 0.5) × envelope
 *
 * Colour mapping (intensity × row position):
 *   top fringe  (y < 0.25):  magenta dim → magenta
 *   upper core  (y < 0.55):  cyan dim    → cyan bold
 *   lower base  (y ≥ 0.55):  green dim   → green bold
 *
 * Keys:
 *   q/ESC quit   space pause   ] / [  sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra aurora.c -o aurora -lncurses -lm
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
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * AURORA_FRAC — fraction of screen height used by aurora.
 * Stars fill the rest.
 */
#define AURORA_FRAC  0.65f

/* Star density: probability that a background cell is a star.
 * A hash of (col,row) provides a deterministic, storage-free star map. */
#define STAR_THRESH  5          /* out of 256: ~2% of background cells    */

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
        init_pair(1, 196, COLOR_BLACK);   /* red      */
        init_pair(2, 208, COLOR_BLACK);   /* orange   */
        init_pair(3, 226, COLOR_BLACK);   /* yellow   */
        init_pair(4,  46, COLOR_BLACK);   /* green    */
        init_pair(5,  51, COLOR_BLACK);   /* cyan     */
        init_pair(6,  21, COLOR_BLACK);   /* blue     */
        init_pair(7, 201, COLOR_BLACK);   /* magenta  */
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
/* §4  coords — aurora works directly in cell space                       */
/* ===================================================================== */

/* ===================================================================== */
/* §5  entity — Aurora                                                    */
/* ===================================================================== */

typedef struct {
    float time;
    bool  paused;
} Aurora;

static void aurora_init(Aurora *a)
{
    a->time   = 0.0f;
    a->paused = false;
}

static void aurora_tick(Aurora *a, float dt)
{
    if (a->paused) return;
    a->time += dt;
}


/*
 * star_at — deterministic star presence check.
 *
 * No storage needed: a fast integer hash of (col, row) decides whether a
 * star occupies this cell.  Same result every frame → no flicker.
 */
static bool star_at(int col, int row, char *out_ch, int *out_pair)
{
    unsigned h = (unsigned)(col * 1234597u ^ row * 987659u ^ (col + row * 31));
    h ^= h >> 13; h *= 0x5bd1e995u; h ^= h >> 15;
    if ((h & 0xFF) >= STAR_THRESH) return false;
    *out_ch   = ((h >> 8) & 3) == 0 ? '*' : ((h >> 8) & 3) == 1 ? '+' : '.';
    *out_pair = ((h >> 10) & 1) ? 3 : 6;   /* yellow or blue */
    return true;
}

static void aurora_draw(const Aurora *a, WINDOW *w, int cols, int rows)
{
    float t        = a->time;
    float aurora_h = (float)rows * AURORA_FRAC;

    for (int row = 1; row < rows - 1; row++) {
        float ay = (float)row / aurora_h;   /* 0 = top, 1 = aurora bottom */

        for (int col = 0; col < cols; col++) {

            /* ── aurora region ───────────────────────────────────── */
            if (ay < 1.0f) {
                float x = (float)col / (float)cols * 2.0f * (float)M_PI;
                float y = ay;

                /* primary curtain: slow horizontal drift + vertical ripple */
                float primary = sinf(x * 1.5f + t * 0.20f)
                              * cosf(y * 3.0f + t * 0.50f + x * 0.25f);

                /* shimmer: faster, different phase offset */
                float shimmer = cosf(x * 2.3f - t * 0.15f + 1.0f)
                              * sinf(y * 5.0f + t * 0.80f + x * 0.40f + 2.5f);

                /* combine → normalise to [0, 1] */
                float v = primary * 0.60f + shimmer * 0.40f;
                v = v * 0.5f + 0.5f;

                /* vertical envelope: 0 at y=0 and y=1, peak at y=0.5 */
                float env = sinf(y * (float)M_PI);

                float intensity = v * env;

                /* ── background / star ── */
                if (intensity < 0.08f) {
                    char sch; int spair;
                    if (star_at(col, row, &sch, &spair)) {
                        wattron(w, COLOR_PAIR(spair) | A_DIM);
                        mvwaddch(w, row, col, (chtype)(unsigned char)sch);
                        wattroff(w, COLOR_PAIR(spair) | A_DIM);
                    }
                    continue;
                }

                /* ── aurora colour + character ── */
                int   pair;
                chtype attr;
                char  ch;

                /* Row position determines hue:
                 *   top fringe  (y < 0.25): magenta / purple
                 *   upper core  (y < 0.55): cyan
                 *   lower base  (y ≥ 0.55): green                         */
                if (y < 0.25f) {
                    pair = 7;   /* magenta fringe */
                    attr = (intensity < 0.20f) ? A_DIM : 0;
                } else if (y < 0.55f) {
                    pair = 5;   /* cyan core */
                    attr = (intensity > 0.55f) ? A_BOLD : 0;
                } else {
                    pair = 4;   /* green base */
                    attr = (intensity > 0.60f) ? A_BOLD
                         : (intensity < 0.18f) ? A_DIM : 0;
                }

                if      (intensity < 0.20f) ch = '.';
                else if (intensity < 0.40f) ch = ':';
                else if (intensity < 0.65f) ch = '|';
                else if (intensity < 0.82f) ch = '!';
                else                        ch = '|';

                wattron(w, COLOR_PAIR(pair) | attr);
                mvwaddch(w, row, col, (chtype)(unsigned char)ch);
                wattroff(w, COLOR_PAIR(pair) | attr);

            } else {
                /* ── below aurora: star field only ── */
                char sch; int spair;
                if (star_at(col, row, &sch, &spair)) {
                    wattron(w, COLOR_PAIR(spair) | A_DIM);
                    mvwaddch(w, row, col, (chtype)(unsigned char)sch);
                    wattroff(w, COLOR_PAIR(spair) | A_DIM);
                }
            }
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct { Aurora aurora; } Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    aurora_init(&s->aurora);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    (void)cols; (void)rows;
    aurora_tick(&s->aurora, dt);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    aurora_draw(&s->aurora, w, cols, rows);
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

    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
             fps, sim_fps, sc->aurora.paused ? "PAUSED" : "");
    int hx = s->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(3) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(3) | A_BOLD);

    attron(COLOR_PAIR(5) | A_BOLD);
    mvprintw(0, 1, " AURORA BOREALIS ");
    attroff(COLOR_PAIR(5) | A_BOLD);

    attron(COLOR_PAIR(6) | A_DIM);
    mvprintw(s->rows - 1, 0, " q:quit  spc:pause  [/]:Hz ");
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
    switch (ch) {
    case 'q': case 'Q': case 27: return false;
    case ' ': app->scene.aurora.paused = !app->scene.aurora.paused; break;
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
