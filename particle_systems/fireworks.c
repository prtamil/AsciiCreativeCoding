/*
 * fireworks.c  —  ncurses ASCII fireworks
 *
 * Features:
 *   - Single-threaded, no pthreads
 *   - Single stdscr, ncurses internal double buffer — no flicker
 *   - dt (delta-time) loop — physics runs at fixed SIM_FPS, render capped at 60 fps
 *   - SIGWINCH resize: rebuilds show to new terminal dimensions
 *   - Speed control:   ] = faster   [ = slower
 *   - Rocket control:  = = more     - = fewer rockets
 *   - Clean signal / atexit teardown — terminal always restored
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   =  -      more / fewer rockets
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra -lm fireworks.c -o fireworks -lncurses
 *
 * Sections
 * --------
 *   §1  config   — every tunable constant in one block
 *   §2  clock    — monotonic nanosecond clock, portable sleep
 *   §3  color    — color pairs; 256-color with 8-color fallback
 *   §4  particle — one spark emitted by an explosion
 *   §5  rocket   — one ascending rocket + its particle burst
 *   §6  show     — rocket pool + simulation tick
 *   §7  screen   — single stdscr, ncurses internal double buffer
 *   §8  app      — dt loop, input, resize, cleanup
 */

#define _POSIX_C_SOURCE 200809L

/* M_PI is not guaranteed by C99/C11 — define it explicitly. */
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

/*
 * Runtime-mutable settings live in the App struct (§8).
 * Hard limits and fixed constants live here.
 */
enum {
    /* Simulation speed — ticks per second */
    SIM_FPS_MIN      =  10,   /* slowest selectable speed             */
    SIM_FPS_DEFAULT  =  30,   /* startup speed                        */
    SIM_FPS_MAX      =  60,   /* fastest selectable speed             */
    SIM_FPS_STEP     =   5,   /* ] / [ increment                      */

    /* Rockets */
    ROCKETS_MIN      =   1,   /* minimum live rockets                 */
    ROCKETS_DEFAULT  =   6,   /* startup rocket count                 */
    ROCKETS_MAX      =  20,   /* maximum live rockets                 */
    ROCKETS_STEP     =   1,   /* = / - increment                      */

    /* Particles per explosion */
    PARTICLES_PER_BURST = 80,

    /* Physics */
    LAUNCH_SPEED_MIN =   3,   /* rows/sec upward at launch            */
    LAUNCH_SPEED_MAX =   8,   /* rows/sec upward at launch            */

    /* HUD overlay */
    HUD_COLS         =  30,   /* width of the status bar window       */
    FPS_UPDATE_MS    = 500,   /* re-measure FPS every 500 ms          */

    /* Total pool sizes (fixed at compile time) */
    MAX_ROCKETS      =  ROCKETS_MAX,
    MAX_PARTICLES    =  MAX_ROCKETS * PARTICLES_PER_BURST,
};

/* Nanosecond helpers */
#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(fps)  (NS_PER_SEC / (fps))

/*
 * ROCKET_DRAG — deceleration applied to ascending rockets (rows/sec²).
 *   Controls apex height; 9.8 spreads explosions across the upper screen.
 *
 * GRAVITY — downward acceleration on each spark after explosion.
 *   Lower than ROCKET_DRAG so particles spread in all directions before
 *   gravity pulls them down.
 */
#define ROCKET_DRAG   9.8f
#define GRAVITY       4.0f

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — wall-clock nanoseconds via CLOCK_MONOTONIC.
 * Never jumps backward — the only correct choice for a dt loop.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep the requested nanoseconds.
 */
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
 * Fireworks use 7 color pairs, one per spectral hue.
 * Pair IDs are named so every site reads as intent.
 *
 * 256-color foregrounds (vivid):
 *   RED    196   ORANGE 208   YELLOW 226
 *   GREEN   46   CYAN    51   BLUE    21   MAGENTA 201
 *
 * 8-color fallback (same slot order):
 *   COLOR_RED  COLOR_YELLOW  COLOR_YELLOW
 *   COLOR_GREEN COLOR_CYAN   COLOR_BLUE  COLOR_MAGENTA
 */
typedef enum {
    COL_RED     = 1,
    COL_ORANGE  = 2,
    COL_YELLOW  = 3,
    COL_GREEN   = 4,
    COL_CYAN    = 5,
    COL_BLUE    = 6,
    COL_MAGENTA = 7,
    COL_COUNT   = 7,
} ColorID;

static void color_init(void)
{
    start_color();

    if (COLORS >= 256) {
        init_pair(COL_RED,     196, COLOR_BLACK);
        init_pair(COL_ORANGE,  208, COLOR_BLACK);
        init_pair(COL_YELLOW,  226, COLOR_BLACK);
        init_pair(COL_GREEN,    46, COLOR_BLACK);
        init_pair(COL_CYAN,     51, COLOR_BLACK);
        init_pair(COL_BLUE,     21, COLOR_BLACK);
        init_pair(COL_MAGENTA, 201, COLOR_BLACK);
    } else {
        /* 8-color fallback — works in tmux without 256color config */
        init_pair(COL_RED,     COLOR_RED,     COLOR_BLACK);
        init_pair(COL_ORANGE,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_YELLOW,  COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_GREEN,   COLOR_GREEN,   COLOR_BLACK);
        init_pair(COL_CYAN,    COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_BLUE,    COLOR_BLUE,    COLOR_BLACK);
        init_pair(COL_MAGENTA, COLOR_MAGENTA, COLOR_BLACK);
    }
}

/* Return a random ColorID in [COL_RED, COL_COUNT]. */
static ColorID color_rand(void)
{
    return (ColorID)(1 + rand() % COL_COUNT);
}

/* ===================================================================== */
/* §4  particle                                                           */
/* ===================================================================== */

/*
 * Particle — one spark emitted when a rocket explodes.
 *
 * Position (x, y) and velocity (vx, vy) are floats so sub-cell physics
 * accumulates correctly across ticks before snapping to integer cells.
 * life counts down from 1.0 to 0.0 each tick; when it reaches 0 the
 * particle is deactivated.  The symbol and color are assigned at birth
 * and never change — they identify this particle for its lifetime.
 */
typedef struct {
    float   x, y;
    float   vx, vy;
    float   life;       /* 1.0 = fresh, 0.0 = dead                  */
    float   decay;      /* life subtracted per tick                 */
    char    symbol;
    ColorID color;
    bool    active;
} Particle;

/* Pure 7-bit ASCII only — no UTF-8 bytes. */
static const char k_particle_symbols[] = "*+.,`'^-~=o#@%&$!|\\/:;";
#define PARTICLE_SYM_COUNT (int)(sizeof k_particle_symbols - 1)

/*
 * particle_burst() — initialise a full burst of particles at (x, y).
 *
 * Each particle is assigned a random color independently of the rocket's
 * color, so every explosion is a multi-color spray.
 * Particles are spread evenly around 2π with randomised speed so the
 * explosion looks like a sphere rather than a uniform ring.
 * Each particle gets a random lifetime so the burst fades out gradually.
 */
static void particle_burst(Particle *p, int count, float x, float y)
{
    for (int i = 0; i < count; i++) {
        float angle = ((float)i / count) * 2.0f * (float)M_PI
                      + ((float)rand() / RAND_MAX) * 0.3f;
        float speed = 1.5f + ((float)rand() / RAND_MAX) * 3.5f;

        p[i].x      = x;
        p[i].y      = y;
        p[i].vx     = cosf(angle) * speed;
        p[i].vy     = sinf(angle) * speed * 0.5f;  /* squash vertical */
        p[i].life   = 0.6f + ((float)rand() / RAND_MAX) * 0.4f;
        p[i].decay  = 0.03f + ((float)rand() / RAND_MAX) * 0.04f;
        p[i].symbol = k_particle_symbols[rand() % PARTICLE_SYM_COUNT];
        p[i].color  = color_rand();   /* every particle its own color */
        p[i].active = true;
    }
}

/*
 * particle_tick() — advance one simulation step for a single particle.
 * dt_sec is the fixed sim tick duration in seconds.
 */
static void particle_tick(Particle *p, float dt_sec)
{
    if (!p->active) return;

    p->x  += p->vx * dt_sec * 8.0f;   /* scale to terminal columns    */
    p->y  += p->vy * dt_sec * 8.0f;   /* scale to terminal rows       */
    p->vy += GRAVITY * dt_sec;         /* gravity pulls downward       */
    p->life -= p->decay;

    if (p->life <= 0.0f) p->active = false;
}

/*
 * particle_draw() — render one particle into a WINDOW.
 * Brightness is derived from life: bright when fresh, dim when fading.
 */
static void particle_draw(const Particle *p, WINDOW *w, int cols, int rows)
{
    if (!p->active) return;

    int x = (int)p->x;
    int y = (int)p->y;
    if (x < 0 || x >= cols || y < 0 || y >= rows) return;

    attr_t attr = COLOR_PAIR(p->color);
    if (p->life > 0.6f)       attr |= A_BOLD;
    else if (p->life < 0.2f)  attr |= A_DIM;

    wattron(w, attr);
    mvwaddch(w, y, x, (chtype)(unsigned char)p->symbol);
    wattroff(w, attr);
}

/* ===================================================================== */
/* §5  rocket                                                             */
/* ===================================================================== */

/*
 * Rocket — one ascending streak that explodes at its apex.
 *
 * State machine:
 *   IDLE     — waiting to launch; respawns after a random delay
 *   RISING   — travelling upward; explodes when vy >= 0 (apex reached)
 *   EXPLODED — particles still active; rocket slot waits for them to die
 *
 * particles[] is a fixed array of PARTICLES_PER_BURST owned by the rocket.
 * This keeps allocation entirely on the stack / in the Show struct.
 *
 * launch_col  fixed x column for this rocket's lifetime (reassigned on
 *             each new launch).
 * fuse        countdown ticks before an IDLE rocket relaunches.
 */
typedef enum {
    RS_IDLE     = 0,
    RS_RISING   = 1,
    RS_EXPLODED = 2,
} RocketState;

typedef struct {
    float       x, y;
    float       vy;             /* upward velocity (negative = up)     */
    ColorID     color;
    RocketState state;
    int         fuse;           /* ticks until relaunch from IDLE      */
    Particle    particles[PARTICLES_PER_BURST];
} Rocket;

/*
 * rocket_launch() — reset a rocket for a new flight.
 * Starts at the bottom of the screen in a random column.
 */
static void rocket_launch(Rocket *r, int cols, int rows)
{
    r->x     = (float)(rand() % cols);
    r->y     = (float)(rows - 1);
    r->vy    = -(float)(LAUNCH_SPEED_MIN
                + rand() % (LAUNCH_SPEED_MAX - LAUNCH_SPEED_MIN + 1));
    r->color = color_rand();
    r->state = RS_RISING;

    /* Deactivate any lingering particles from the previous burst. */
    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        r->particles[i].active = false;
}

/*
 * rocket_tick() — advance one simulation step.
 * dt_sec is the fixed sim tick in seconds.
 */
static void rocket_tick(Rocket *r, float dt_sec, int cols, int rows)
{
    switch (r->state) {

    case RS_IDLE:
        if (--r->fuse <= 0)
            rocket_launch(r, cols, rows);
        break;

    case RS_RISING:
        r->y  += r->vy * dt_sec * 6.0f;
        r->vy += ROCKET_DRAG * dt_sec * 0.5f;   /* decelerate to apex */

        /* Explode at apex (vy crosses zero) or if it exits top. */
        if (r->vy >= 0.0f || r->y < 2.0f) {
            particle_burst(r->particles, PARTICLES_PER_BURST,
                           r->x, r->y);
            r->state = RS_EXPLODED;
        }
        break;

    case RS_EXPLODED: {
        /* Tick particles; transition to IDLE once all are dead. */
        bool any_alive = false;
        for (int i = 0; i < PARTICLES_PER_BURST; i++) {
            particle_tick(&r->particles[i], dt_sec);
            if (r->particles[i].active) any_alive = true;
        }
        if (!any_alive) {
            /* Random fuse 0.5–2.5 s before next launch. */
            int ticks_per_sec = (int)(1.0f / dt_sec);
            r->fuse  = ticks_per_sec / 2
                       + rand() % (ticks_per_sec * 2);
            r->state = RS_IDLE;
        }
        break;
    }
    } /* switch */
}

/*
 * rocket_draw() — render rocket body and its particles into a WINDOW.
 */
static void rocket_draw(const Rocket *r, WINDOW *w, int cols, int rows)
{
    if (r->state == RS_RISING) {
        int x = (int)r->x;
        int y = (int)r->y;
        if (x >= 0 && x < cols && y >= 0 && y < rows) {
            wattron(w, COLOR_PAIR(r->color) | A_BOLD);
            mvwaddch(w, y, x, '|');
            wattroff(w, COLOR_PAIR(r->color) | A_BOLD);

            /* Draw a short exhaust trail below the rocket. */
            if (y + 1 < rows) {
                wattron(w, COLOR_PAIR(r->color));
                mvwaddch(w, y + 1, x, '\'');
                wattroff(w, COLOR_PAIR(r->color));
            }
        }
    }

    if (r->state == RS_EXPLODED) {
        for (int i = 0; i < PARTICLES_PER_BURST; i++)
            particle_draw(&r->particles[i], w, cols, rows);
    }
}

/* ===================================================================== */
/* §6  show                                                               */
/* ===================================================================== */

/*
 * Show — the simulation state.
 *
 * Owns a pool of Rocket slots.  The active_rockets count is the
 * user-controlled target; slots beyond that index are kept IDLE with a
 * long fuse so they don't fire unexpectedly when the count is raised.
 *
 * active_rockets is runtime-mutable (= / - keys).
 */
typedef struct {
    Rocket rockets[MAX_ROCKETS];
    int    active_rockets;   /* how many slots are actually used      */
} Show;

static void show_init(Show *s, int cols, int rows, int rocket_count)
{
    s->active_rockets = rocket_count;

    for (int i = 0; i < MAX_ROCKETS; i++) {
        if (i < rocket_count) {
            /* Stagger initial launches so they don't all fire at once. */
            rocket_launch(&s->rockets[i], cols, rows);
            s->rockets[i].fuse  = i * 8;        /* spread them out     */
            s->rockets[i].state = RS_IDLE;
        } else {
            s->rockets[i].state = RS_IDLE;
            s->rockets[i].fuse  = INT32_MAX / 2; /* effectively parked  */
            for (int j = 0; j < PARTICLES_PER_BURST; j++)
                s->rockets[i].particles[j].active = false;
        }
    }
}

static void show_free(Show *s)
{
    /* No heap allocations in Show — just zero the struct for safety. */
    memset(s, 0, sizeof *s);
}

/*
 * show_tick() — advance the simulation by one fixed step.
 *
 * dt_sec is the duration of one simulation tick in seconds.
 * Only the first active_rockets slots are ticked; the rest stay parked.
 */
static void show_tick(Show *s, float dt_sec, int cols, int rows)
{
    for (int i = 0; i < s->active_rockets; i++)
        rocket_tick(&s->rockets[i], dt_sec, cols, rows);
}

/*
 * show_draw() — paint all rockets and particles into a WINDOW.
 * werase() is called by the screen layer before this, not here.
 */
static void show_draw(const Show *s, WINDOW *w, int cols, int rows)
{
    for (int i = 0; i < s->active_rockets; i++)
        rocket_draw(&s->rockets[i], w, cols, rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses' internal double buffer.
 *
 * erase()           — clear newscr (the back buffer)
 * mvwaddch/attron   — write scene into newscr, no terminal I/O
 * mvprintw          — write HUD into newscr after scene (always on top)
 * wnoutrefresh()    — mark newscr ready, still no terminal I/O
 * doupdate()        — ONE atomic write: diff newscr vs curscr → terminal
 *
 * typeahead(-1) prevents ncurses interrupting output to poll stdin,
 * which causes tearing on fast terminals.
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

/*
 * screen_draw_show() — erase newscr then paint all rockets into stdscr.
 * Nothing reaches the terminal until screen_present() is called.
 */
static void screen_draw_show(Screen *s, const Show *show)
{
    erase();
    show_draw(show, stdscr, s->cols, s->rows);
}

/*
 * screen_draw_hud() — write HUD into stdscr after the scene.
 * Drawn last so it always sits on top of any particle at row 0.
 */
static void screen_draw_hud(Screen *s,
                              double fps,
                              int    sim_fps,
                              int    rockets)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps spd:%d rkt:%d",
             fps, sim_fps, rockets);

    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(COL_YELLOW) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(COL_YELLOW) | A_BOLD);
}

/*
 * screen_present() — flush newscr to the terminal.
 * wnoutrefresh marks stdscr ready; doupdate sends only changed cells.
 */
static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — top-level owner of all subsystems.
 *
 * Runtime-mutable state:
 *   sim_fps   ticks/second — ] / [
 *   rockets   live count   — = / -
 *
 * Signal flags (sig_atomic_t for safe signal-handler writes):
 *   running      set to 0 by SIGINT/SIGTERM
 *   need_resize  set to 1 by SIGWINCH
 */
typedef struct {
    Show                  show;
    Screen                screen;
    int                   sim_fps;
    int                   rockets;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;   /* global — signal handlers reach it via this */

/* -------------------------------------------------------------------- */
/* signal handlers                                                       */
/* -------------------------------------------------------------------- */

static void on_exit_signal(int sig)
{
    (void)sig;
    g_app.running = 0;
}

static void on_resize_signal(int sig)
{
    (void)sig;
    g_app.need_resize = 1;
}

/* -------------------------------------------------------------------- */
/* atexit cleanup                                                        */
/* -------------------------------------------------------------------- */

static void cleanup(void)
{
    /* endwin() is idempotent — safe even if never fully initialised. */
    endwin();
}

/* -------------------------------------------------------------------- */
/* resize handler                                                        */
/* -------------------------------------------------------------------- */

/*
 * app_do_resize() — rebuild screen + show to new terminal dimensions.
 * show_free + show_init restarts the fireworks at the new size.
 * sim_fps and rockets are preserved across the resize.
 */
static void app_do_resize(App *app)
{
    show_free(&app->show);
    screen_resize(&app->screen);
    show_init(&app->show, app->screen.cols, app->screen.rows, app->rockets);
    app->need_resize = 0;
}

/* -------------------------------------------------------------------- */
/* input handler                                                         */
/* -------------------------------------------------------------------- */

/*
 * app_handle_key() — process one keypress, return false to quit.
 *
 * Keys:
 *   q / ESC   quit
 *   ]         speed up   (sim_fps += STEP, capped at MAX)
 *   [         slow down  (sim_fps -= STEP, capped at MIN)
 *   =  +      more rockets (capped at MAX)
 *   -         fewer rockets (capped at MIN)
 *
 * Rocket count changes update show.active_rockets in-place — no restart.
 * Newly activated slots have already been zeroed / parked in show_init.
 */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {

    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case '=': case '+':
        if (app->rockets < ROCKETS_MAX) {
            /* Activate the next parked slot with a short fuse. */
            int i = app->rockets;
            rocket_launch(&app->show.rockets[i],
                          app->screen.cols, app->screen.rows);
            app->show.rockets[i].fuse  = 5;
            app->show.rockets[i].state = RS_IDLE;
            app->rockets++;
            app->show.active_rockets = app->rockets;
        }
        break;

    case '-':
        if (app->rockets > ROCKETS_MIN) {
            app->rockets--;
            app->show.active_rockets = app->rockets;
        }
        break;

    default:
        break;
    }

    return true;
}

/* -------------------------------------------------------------------- */
/* main — dt simulation loop                                             */
/* -------------------------------------------------------------------- */

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app      = &g_app;
    app->running  = 1;
    app->sim_fps  = SIM_FPS_DEFAULT;
    app->rockets  = ROCKETS_DEFAULT;

    screen_init(&app->screen);

    int cols = app->screen.cols;
    int rows = app->screen.rows;

    show_init(&app->show, cols, rows, app->rockets);

    /*
     * dt loop state
     * -------------
     * frame_time   — absolute ns timestamp at start of last frame
     * sim_accum    — ns banked but not yet consumed by sim ticks
     * fps_accum    — ns elapsed since last FPS measurement
     * frame_count  — frames rendered in the current FPS window
     * fps_display  — last computed FPS value shown in HUD
     */
    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize check ────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset dt after stall */
            sim_accum  = 0;
        }

        /* ── dt measurement ──────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;

        /* Clamp: don't try to catch up after a stall > 100 ms. */
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── simulation accumulator ──────────────────────────────── */
        /*
         * sim_fps is read fresh each iteration so ] / [ take effect
         * immediately without restarting or resetting anything.
         * dt_sec is computed once per tick batch so physics is stable.
         */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            show_tick(&app->show, dt_sec, app->screen.cols, app->screen.rows);
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
        int64_t budget  = NS_PER_SEC / 60;
        clock_sleep_ns(budget - elapsed);

        /* ── render ──────────────────────────────────────────────── */
        screen_draw_show(&app->screen, &app->show);

        /* ── HUD (into stdscr after scene, drawn on top) ─────────── */
        screen_draw_hud(&app->screen, fps_display,
                         app->sim_fps, app->rockets);

        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    /* ── cleanup ─────────────────────────────────────────────────── */
    show_free(&app->show);
    screen_free(&app->screen);   /* calls endwin() */

    return 0;
}
