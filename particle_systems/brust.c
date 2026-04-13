/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burst.c  —  ncurses random ASCII burst field
 *
 * Effect: bursts appear at random screen positions with no rocket.
 * Each burst explodes outward with mixed-color ASCII particles that
 * slow down and fade.  Multiple bursts overlap at different ages.
 * Scorch marks accumulate where bursts land.
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   =  -      more / fewer simultaneous bursts
 *   r         clear scorch marks
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra burst.c -o burst -lncurses -lm
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant
 *   §2  clock    — monotonic nanosecond clock + sleep
 *   §3  color    — 7 vivid color pairs, 256-color with 8-color fallback
 *   §4  particle — one flying ASCII fragment
 *   §5  burst    — one explosion: particle pool + state + tick + draw
 *   §6  field    — burst pool + scorch layer + tick + draw
 *   §7  screen   — single stdscr, ncurses internal double buffer
 *   §8  app      — dt loop, input, resize, cleanup
 */

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

enum {
    SIM_FPS_MIN      =  5,
    SIM_FPS_DEFAULT  = 24,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  4,

    BURSTS_MIN       =  1,
    BURSTS_DEFAULT   =  5,
    BURSTS_MAX       = 16,

    PARTICLES        = 48,
    BURST_TICKS      = 22,
    FUSE_MIN         =  8,
    FUSE_RANGE       = 20,

    HUD_COLS         = 28,
    FPS_UPDATE_MS    = 500,
};

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

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

typedef enum {
    C_RED     = 1,
    C_ORANGE  = 2,
    C_YELLOW  = 3,
    C_GREEN   = 4,
    C_CYAN    = 5,
    C_BLUE    = 6,
    C_MAGENTA = 7,
    C_COUNT   = 7,
} Hue;

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(C_RED,     196, COLOR_BLACK);
        init_pair(C_ORANGE,  208, COLOR_BLACK);
        init_pair(C_YELLOW,  226, COLOR_BLACK);
        init_pair(C_GREEN,    46, COLOR_BLACK);
        init_pair(C_CYAN,     51, COLOR_BLACK);
        init_pair(C_BLUE,     21, COLOR_BLACK);
        init_pair(C_MAGENTA, 201, COLOR_BLACK);
    } else {
        init_pair(C_RED,     COLOR_RED,     COLOR_BLACK);
        init_pair(C_ORANGE,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(C_YELLOW,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(C_GREEN,   COLOR_GREEN,   COLOR_BLACK);
        init_pair(C_CYAN,    COLOR_CYAN,    COLOR_BLACK);
        init_pair(C_BLUE,    COLOR_BLUE,    COLOR_BLACK);
        init_pair(C_MAGENTA, COLOR_MAGENTA, COLOR_BLACK);
    }
}

static Hue hue_rand(void) { return (Hue)(1 + rand() % C_COUNT); }

/* ===================================================================== */
/* §4  particle                                                           */
/* ===================================================================== */

#define ASPECT 2.0f

typedef struct {
    float cx, cy;
    float rx, ry;
    float vx, vy;
    float life;
    float decay;
    int   delay;
    char  sym;
    Hue   hue;
    bool  alive;
} Particle;

static const char k_syms[] = "*.+o#@%&$!^~-=|/\\:;,`'\"";
#define NSYMS (int)(sizeof k_syms - 1)

static void particle_spawn(Particle *p, float cx, float cy,
                            float angle, float speed, int delay_ticks)
{
    p->cx    = cx;
    p->cy    = cy;
    p->rx    = 0.0f;
    p->ry    = 0.0f;
    p->vx    = cosf(angle) * speed;
    p->vy    = sinf(angle) * speed;
    p->life  = 0.8f + ((float)rand() / RAND_MAX) * 0.2f;
    p->decay = 0.05f + ((float)rand() / RAND_MAX) * 0.04f;
    p->delay = delay_ticks;
    p->sym   = k_syms[rand() % NSYMS];
    p->hue   = hue_rand();
    p->alive = true;
}

static void particle_tick(Particle *p, int cols, int rows)
{
    if (!p->alive) return;
    if (p->delay > 0) { p->delay--; return; }

    p->vx *= 0.82f;
    p->vy *= 0.82f;
    p->rx += p->vx;
    p->ry += p->vy;
    p->life -= p->decay;

    float sx = p->cx + p->rx * ASPECT;
    float sy = p->cy + p->ry;

    if (p->life <= 0.0f
        || sx < 0 || sx >= (float)cols
        || sy < 0 || sy >= (float)rows)
        p->alive = false;
}

/* particle_draw now takes WINDOW* so it works with any window or stdscr */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    if (!p->alive || p->delay > 0) return;

    int x = (int)(p->cx + p->rx * ASPECT);
    int y = (int)(p->cy + p->ry);
    if (x < 0 || x >= cols || y < 0 || y >= rows) return;

    attr_t attr = COLOR_PAIR(p->hue);
    if (p->life > 0.65f) attr |= A_BOLD;

    wattron(w, attr);
    mvwaddch(w, y, x, (chtype)(unsigned char)p->sym);
    wattroff(w, attr);
}

/* ===================================================================== */
/* §5  burst                                                              */
/* ===================================================================== */

typedef enum {
    BS_IDLE  = 0,
    BS_FLASH = 1,
    BS_LIVE  = 2,
} BurstState;

typedef struct {
    float      cx, cy;
    BurstState state;
    int        ticks;
    int        fuse;
    Particle   parts[PARTICLES];
} Burst;

static void burst_ignite(Burst *b, int cols, int rows)
{
    b->cx    = (float)(2 + rand() % (cols - 4));
    b->cy    = (float)(1 + rand() % (rows - 2));
    b->ticks = 0;
    b->state = BS_FLASH;

    const int MAX_DELAY = 5;
    const int waves     = 4;

    for (int i = 0; i < PARTICLES; i++) {
        float angle = ((float)i / PARTICLES) * 2.0f * (float)M_PI
                      + ((float)rand() / RAND_MAX) * 0.2f;
        float speed = 1.8f + ((float)rand() / RAND_MAX) * 2.8f;
        int   wave  = i % waves;
        int   delay = (wave * MAX_DELAY) / (waves - 1);
        particle_spawn(&b->parts[i], b->cx, b->cy, angle, speed, delay);
    }
}

static void burst_tick(Burst *b, int cols, int rows,
                       void (*scorch_cb)(int x, int y, void *ud), void *ud)
{
    switch (b->state) {
    case BS_IDLE:
        if (--b->fuse <= 0) burst_ignite(b, cols, rows);
        break;
    case BS_FLASH:
        b->state = BS_LIVE;
        b->ticks = 0;
        break;
    case BS_LIVE: {
        bool any = false;
        for (int i = 0; i < PARTICLES; i++) {
            particle_tick(&b->parts[i], cols, rows);
            if (b->parts[i].alive) any = true;
        }
        b->ticks++;
        if (!any || b->ticks >= BURST_TICKS) {
            if (scorch_cb) scorch_cb((int)b->cx, (int)b->cy, ud);
            b->fuse  = FUSE_MIN + rand() % FUSE_RANGE;
            b->state = BS_IDLE;
        }
        break;
    }
    }
}

static void burst_draw(const Burst *b, WINDOW *w, int cols, int rows)
{
    int cx = (int)b->cx;
    int cy = (int)b->cy;

    if (b->state == BS_FLASH) {
        if (cx >= 0 && cx < cols && cy >= 0 && cy < rows) {
            wattron(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
            mvwaddch(w, cy, cx, '*');
            if (cx > 0)       mvwaddch(w, cy,   cx-1, '+');
            if (cx < cols-1)  mvwaddch(w, cy,   cx+1, '+');
            if (cy > 0)       mvwaddch(w, cy-1, cx,   '+');
            if (cy < rows-1)  mvwaddch(w, cy+1, cx,   '+');
            wattroff(w, COLOR_PAIR(C_YELLOW) | A_BOLD);
        }
        return;
    }

    if (b->state == BS_LIVE) {
        for (int i = 0; i < PARTICLES; i++)
            particle_draw(&b->parts[i], w, cols, rows);
    }
}

/* ===================================================================== */
/* §6  field                                                              */
/* ===================================================================== */

typedef struct {
    Burst  bursts[BURSTS_MAX];
    char  *scorch;
    int    cols;
    int    rows;
    int    active_bursts;
} Field;

static void field_scorch_cb(int x, int y, void *ud)
{
    Field *f = (Field *)ud;
    if (x >= 0 && x < f->cols && y >= 0 && y < f->rows)
        f->scorch[y * f->cols + x] = '.';
}

static void field_init(Field *f, int cols, int rows, int burst_count)
{
    f->cols          = cols;
    f->rows          = rows;
    f->active_bursts = burst_count;
    f->scorch        = calloc((size_t)(cols * rows), sizeof(char));

    for (int i = 0; i < BURSTS_MAX; i++) {
        memset(&f->bursts[i], 0, sizeof(Burst));
        f->bursts[i].state = BS_IDLE;
        if (i < burst_count)
            f->bursts[i].fuse = i * (FUSE_MIN + FUSE_RANGE / burst_count);
        else
            f->bursts[i].fuse = INT32_MAX / 2;
    }
}

static void field_free(Field *f)
{
    free(f->scorch);
    *f = (Field){0};
}

static void field_tick(Field *f)
{
    for (int i = 0; i < f->active_bursts; i++)
        burst_tick(&f->bursts[i], f->cols, f->rows, field_scorch_cb, f);
}

static void field_draw(const Field *f, WINDOW *w)
{
    int total = f->cols * f->rows;

    wattron(w, COLOR_PAIR(C_ORANGE) | A_DIM);
    for (int i = 0; i < total; i++) {
        if (!f->scorch[i]) continue;
        int y = i / f->cols;
        int x = i % f->cols;
        if (x < f->cols && y < f->rows)
            mvwaddch(w, y, x, (chtype)(unsigned char)f->scorch[i]);
    }
    wattroff(w, COLOR_PAIR(C_ORANGE) | A_DIM);

    for (int i = 0; i < f->active_bursts; i++)
        burst_draw(&f->bursts[i], w, f->cols, f->rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses' internal double buffer.
 *
 * erase()            — clear newscr (back buffer), no terminal I/O
 * field_draw(stdscr) — write scene into newscr
 * mvprintw / attron  — write HUD into newscr after scene (always on top)
 * wnoutrefresh()     — mark newscr ready, still no terminal I/O
 * doupdate()         — ONE atomic write: diff newscr vs curscr → terminal
 *
 * typeahead(-1) prevents ncurses interrupting output mid-flush to poll
 * stdin, eliminating tearing at high tick rates.
 */
typedef struct {
    int cols;
    int rows;
} Screen;

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

static void screen_free(Screen *s)
{
    (void)s;
    endwin();
}

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_field(Screen *s, const Field *f)
{
    erase();
    field_draw(f, stdscr);
}

static void screen_draw_hud(Screen *s, double fps, int sim_fps, int bursts)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, "%5.1f fps spd:%d burst:%d",
             fps, sim_fps, bursts);
    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(C_YELLOW) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(C_YELLOW) | A_BOLD);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Field                 field;
    Screen                screen;
    int                   sim_fps;
    int                   bursts;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    field_free(&app->field);
    screen_resize(&app->screen);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case '=': case '+':
        if (app->bursts < BURSTS_MAX) {
            int i = app->bursts;
            memset(&app->field.bursts[i], 0, sizeof(Burst));
            app->field.bursts[i].state = BS_IDLE;
            app->field.bursts[i].fuse  = 2 + rand() % FUSE_RANGE;
            app->bursts++;
            app->field.active_bursts = app->bursts;
        }
        break;
    case '-':
        if (app->bursts > BURSTS_MIN) {
            app->bursts--;
            app->field.active_bursts = app->bursts;
        }
        break;

    case 'r': case 'R':
        field_free(&app->field);
        field_init(&app->field, app->screen.cols, app->screen.rows,
                   app->bursts);
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
    app->bursts  = BURSTS_DEFAULT;

    screen_init(&app->screen);
    field_init(&app->field, app->screen.cols, app->screen.rows, app->bursts);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            field_tick(&app->field);
            sim_accum -= tick_ns;
        }
        float alpha = (float)sim_accum / (float)tick_ns;
        (void)alpha;

        /* ── HUD counter ─────────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        /* ── render + HUD ────────────────────────────────────────── */
        screen_draw_field(&app->screen, &app->field);
        screen_draw_hud(&app->screen, fps_display,
                         app->sim_fps, app->bursts);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    field_free(&app->field);
    screen_free(&app->screen);
    return 0;
}
