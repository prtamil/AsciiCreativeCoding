/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fireworks_rain.c  —  ncurses fireworks with matrix-rain arc trails
 *
 * Rockets rise and explode exactly as in fireworks.c.  On explosion every
 * spark grows a matrix-rain trail: a chain of shimmering ASCII characters
 * that follows the exact arc the particle traces through the air.
 *
 * Trail model
 * -----------
 * Each MatrixParticle owns a TRAIL_LEN-slot position history.  Every tick:
 *   1. Current (cx, cy) is pushed into trail[0]; history slides down.
 *   2. Physics advances (cx, cy) forward with gravity.
 *   3. ~75 % of cache chars are re-randomised (matrix shimmer).
 *
 * Draw order: oldest trail slot → newest → live head.
 * Drawing dim entries first lets the brighter head overwrite them when
 * multiple positions map to the same terminal cell (happens near burst
 * centre where particles are tightly packed).
 *
 * Brightness mapping
 *   live head          white | A_BOLD
 *   trail[0] (1 tick ago)  particle color | A_BOLD
 *   trail[1..2]            particle color | A_BOLD
 *   trail[3..TRAIL_LEN/2]  particle color
 *   trail[TRAIL_LEN/2+1..] particle color | A_DIM
 *   life < 0.25 (dying)    all slots → particle color | A_DIM
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   =  -      more / fewer rockets
 *   t         cycle color theme (vivid / matrix / fire / ice / plasma)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain/fireworks_rain.c \
 *       -o fireworks_rain -lncurses -lm
 *
 * Sections
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  matrix particle  (spark + arc trail)
 *   §5  rocket
 *   §6  show
 *   §7  screen
 *   §8  app
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Rocket particle system (fireworks.c) composited with
 *                  matrix-rain arc trails.  Each spark owns a TRAIL_LEN-slot
 *                  position history ring buffer.  Each tick: push current
 *                  position into history, advance physics, re-randomise
 *                  ~75% of cached trail chars (matrix shimmer effect).
 *
 * Physics        : Spark trajectory: explicit Euler with gravity.
 *                  Trail history stores (col, row) snapshots; the most
 *                  recent TRAIL_LEN positions trace the exact parabolic arc.
 *
 * Rendering      : Draw order: oldest trail slot → newest → live head.
 *                  Older slots drawn first so the brighter head overwrites
 *                  them at cells where positions coincide (near burst centre).
 *                  Brightness gradient: head=white bold; tail[0–2]=bold;
 *                  tail[3…N/2]=normal; tail[N/2+1…]=dim.  Life < 0.25 → all dim.
 *
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    /* Simulation speed */
    SIM_FPS_MIN      = 10,
    SIM_FPS_DEFAULT  = 30,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  5,

    /* Rocket pool */
    ROCKETS_MIN      =  1,
    ROCKETS_DEFAULT  =  5,
    ROCKETS_MAX      = 16,

    /* Per-explosion sparks */
    PARTICLES_PER_BURST = 72,

    /*
     * Trail length — number of historic positions kept per spark.
     * Each slot is one simulation tick old.  At SIM_FPS=30 a trail of 16
     * spans ~533 ms of arc history.
     */
    TRAIL_LEN = 16,

    /* Rocket ascent speed (rows/sec at launch) */
    LAUNCH_SPEED_MIN =  3,
    LAUNCH_SPEED_MAX =  8,

    /* HUD */
    HUD_COLS      = 38,
    FPS_UPDATE_MS = 500,

    MAX_ROCKETS = ROCKETS_MAX,
};

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL
#define TICK_NS(fps)  (NS_PER_SEC / (fps))
/*
 * ROCKET_DRAG — deceleration applied to ascending rockets (rows/sec²).
 *   Controls apex height: higher value → rockets explode lower on screen.
 *   9.8 puts explosions throughout the upper two-thirds of the screen.
 *
 * GRAVITY — downward acceleration applied to each spark after explosion.
 *   Lower than ROCKET_DRAG so particles spread in all directions before
 *   gravity pulls them down.  4.0 gives near-symmetric bursts while still
 *   reading as a natural falling arc.
 */
#define ROCKET_DRAG   9.8f
#define GRAVITY       4.0f

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
 * Color pairs
 * -----------
 * Pairs 1–7 (CP_RED … CP_MAGENTA) are the seven spark slots.  Each theme
 * remaps these pairs to a different palette — color_rand() always returns
 * a value in [1, CP_NCOLORS] and the rest of the code is unchanged.
 *
 * Pair 8 (CP_WHITE) is the matrix trail head.  It is always white,
 * independent of the active theme.
 */
typedef enum {
    CP_RED     = 1,
    CP_ORANGE  = 2,
    CP_YELLOW  = 3,
    CP_GREEN   = 4,
    CP_CYAN    = 5,
    CP_BLUE    = 6,
    CP_MAGENTA = 7,
    CP_WHITE   = 8,   /* trail head — always white */
    CP_NCOLORS = 7,   /* number of spark color slots */
} ColorPair;

/*
 * Theme — defines a 7-color palette for the seven spark slots.
 *
 * colors[]   : 256-color terminal fg values (pairs 1–7).
 * fallback[] : 8-color fallback for terminals without 256-color support.
 *
 * Themes:
 *   vivid  — classic multi-hue fireworks (red/orange/yellow/green/cyan/blue/magenta)
 *   matrix — greens only, trails look like Matrix rain arcs
 *   fire   — reds, oranges, yellows; every burst is a flame arc
 *   ice    — blues and cyans; cold, crystalline trails
 *   plasma — purples and magentas; electric neon arcs
 */
typedef struct {
    const char *name;
    int         colors[7];
    int         fallback[7];
} Theme;

static const Theme k_themes[] = {
    { "vivid",
      { 196, 208, 226,  46,  51,  21, 201 },
      { COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
        COLOR_GREEN, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA } },
    { "matrix",
      {  22,  28,  34,  40,  46,  82, 118 },
      { COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
        COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN } },
    { "fire",
      { 196, 160, 202, 208, 214, 220,  88 },
      { COLOR_RED, COLOR_RED, COLOR_YELLOW,
        COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED } },
    { "ice",
      {  21,  27,  33,  39,  45,  51,  87 },
      { COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
        COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE } },
    { "plasma",
      {  53,  57,  93, 129, 165, 201, 207 },
      { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
        COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/*
 * theme_apply() — re-initialise color pairs 1–8 for the chosen theme.
 * Safe to call at any time; takes effect on the next doupdate().
 */
static void theme_apply(int idx)
{
    const Theme *t = &k_themes[idx];
    for (int i = 0; i < 7; i++) {
        int fg = g_has_256 ? t->colors[i] : t->fallback[i];
        init_pair(i + 1, fg, COLOR_BLACK);
    }
    /* CP_WHITE: trail head — always white regardless of theme */
    init_pair(CP_WHITE, g_has_256 ? 255 : COLOR_WHITE, COLOR_BLACK);
}

static ColorPair color_rand(void)
{
    return (ColorPair)(1 + rand() % CP_NCOLORS);
}

/* ===================================================================== */
/* §4  matrix particle                                                    */
/* ===================================================================== */

/*
 * ASCII character pool — same set as matrix_rain.c.
 */
static const char k_chars[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define CHARS_LEN (int)(sizeof k_chars - 1)

static char rand_ch(void)
{
    return k_chars[rand() % CHARS_LEN];
}

/*
 * MatrixParticle — one explosion spark with a matrix-rain arc trail.
 *
 * (cx, cy)          live head position in float screen coordinates.
 * (vx, vy)          velocity (screen units / sec, scaled in tick).
 * trail_x/y[]       position history: [0] = one tick ago, [TRAIL_LEN-1] = oldest.
 * trail_fill        valid entries in trail (ramps 0 → TRAIL_LEN as particle moves).
 * cache[]           shimmering random chars: refreshed ~75 % each tick.
 * life              1.0 (fresh) → 0.0 (dead).
 * decay             life reduction per tick (varied so burst fades out gradually).
 * color             firework hue assigned at burst time.
 */
typedef struct {
    float     cx, cy;
    float     vx, vy;
    float     trail_x[TRAIL_LEN];
    float     trail_y[TRAIL_LEN];
    int       trail_fill;
    float     life;
    float     decay;
    char      cache[TRAIL_LEN];
    ColorPair color;
    bool      active;
} MatrixParticle;

/*
 * trail_attr() — ncurses attribute for one trail slot.
 *
 * i = 0 is the most recent position (one tick ago), highest i is oldest.
 * fading = true when life < 0.25 (particle is dying); dim everything.
 */
static attr_t trail_attr(int i, ColorPair cp, bool fading)
{
    if (fading)
        return COLOR_PAIR(cp) | A_DIM;
    if (i <= 2)
        return COLOR_PAIR(cp) | A_BOLD;
    if (i < TRAIL_LEN / 2)
        return COLOR_PAIR(cp);
    return COLOR_PAIR(cp) | A_DIM;
}

/*
 * mparticle_burst() — spawn a full burst of sparks at (x, y).
 *
 * Angles are evenly spaced around 2π with a small random jitter.
 * Speed varies so the explosion looks like a sphere, not a uniform ring.
 * Each spark gets its own random color independent of the rocket.
 */
static void mparticle_burst(MatrixParticle *p, int count, float x, float y)
{
    for (int i = 0; i < count; i++) {
        float angle = ((float)i / (float)count) * 2.0f * (float)M_PI
                      + ((float)rand() / (float)RAND_MAX) * 0.3f;
        float speed = 1.5f + ((float)rand() / (float)RAND_MAX) * 3.5f;

        p[i].cx         = x;
        p[i].cy         = y;
        p[i].vx         = cosf(angle) * speed;
        p[i].vy         = sinf(angle) * speed;
        p[i].trail_fill = 0;
        p[i].life       = 0.6f + ((float)rand() / (float)RAND_MAX) * 0.4f;
        p[i].decay      = 0.025f + ((float)rand() / (float)RAND_MAX) * 0.035f;
        p[i].color      = color_rand();
        p[i].active     = true;

        for (int k = 0; k < TRAIL_LEN; k++)
            p[i].cache[k] = rand_ch();
    }
}

/*
 * mparticle_tick() — one simulation step.
 *
 * 1. Push current (cx, cy) into trail[0]; slide older entries down.
 * 2. Advance physics: position += velocity * dt; gravity pulls vy down.
 * 3. Shimmer: ~75 % of cache chars are replaced with new random chars.
 * 4. Decay life; deactivate at 0.
 */
static void mparticle_tick(MatrixParticle *p, float dt_sec)
{
    if (!p->active) return;

    /* Push current head into trail history */
    for (int i = TRAIL_LEN - 1; i > 0; i--) {
        p->trail_x[i] = p->trail_x[i - 1];
        p->trail_y[i] = p->trail_y[i - 1];
    }
    p->trail_x[0] = p->cx;
    p->trail_y[0] = p->cy;
    if (p->trail_fill < TRAIL_LEN) p->trail_fill++;

    /* Physics — per-particle gravity variance so arcs don't all trace the same parabola */
    float g = GRAVITY * (0.8f + ((float)rand() / (float)RAND_MAX) * 0.4f);
    p->cx += p->vx * dt_sec * 8.0f;
    p->cy += p->vy * dt_sec * 8.0f;
    p->vy += g * dt_sec;

    /* Shimmer: re-randomise ~75 % of cache each tick */
    for (int k = 0; k < TRAIL_LEN; k++)
        if (rand() % 4 != 0)
            p->cache[k] = rand_ch();

    p->life -= p->decay;
    if (p->life <= 0.0f) p->active = false;
}

/*
 * mparticle_draw() — render the spark and its trail into stdscr.
 *
 * Draw order: oldest trail slot (dimmest) → newest → live head (brightest).
 * Drawing dim entries first means brighter entries overwrite them at cells
 * that map to the same terminal column/row — important near the burst
 * origin where many sparks occupy the same cell.
 */
static void mparticle_draw(const MatrixParticle *p, int cols, int rows)
{
    if (!p->active) return;

    bool fading = (p->life < 0.25f);

    /* Trail — oldest slot first so newest slot paints on top */
    for (int i = p->trail_fill - 1; i >= 0; i--) {
        int x = (int)roundf(p->trail_x[i]);
        int y = (int)roundf(p->trail_y[i]);
        if (x < 0 || x >= cols || y < 0 || y >= rows) continue;

        attr_t attr = trail_attr(i, p->color, fading);
        attron(attr);
        mvaddch(y, x, (chtype)(unsigned char)p->cache[i]);
        attroff(attr);
    }

    /* Live head — drawn last, always wins any overlap */
    int hx = (int)roundf(p->cx);
    int hy = (int)roundf(p->cy);
    if (hx >= 0 && hx < cols && hy >= 0 && hy < rows) {
        attr_t attr = fading ? (COLOR_PAIR(p->color) | A_DIM)
                             : (COLOR_PAIR(CP_WHITE)  | A_BOLD);
        attron(attr);
        mvaddch(hy, hx, (chtype)(unsigned char)p->cache[0]);
        attroff(attr);
    }
}

/* ===================================================================== */
/* §5  rocket                                                             */
/* ===================================================================== */

typedef enum {
    RS_IDLE     = 0,
    RS_RISING   = 1,
    RS_EXPLODED = 2,
} RocketState;

/*
 * Rocket — one ascending streak plus its particle burst.
 *
 * State machine:
 *   IDLE     → waits fuse ticks, then launches.
 *   RISING   → travels upward; explodes at apex (vy ≥ 0) or near top.
 *   EXPLODED → ticks all MatrixParticles; returns to IDLE when all dead.
 */
typedef struct {
    float          x, y;
    float          vy;
    ColorPair      color;
    RocketState    state;
    int            fuse;
    MatrixParticle particles[PARTICLES_PER_BURST];
} Rocket;

static void rocket_launch(Rocket *r, int cols, int rows)
{
    r->x     = (float)(rand() % cols);
    r->y     = (float)(rows - 1);
    r->vy    = -(float)(LAUNCH_SPEED_MIN
                + rand() % (LAUNCH_SPEED_MAX - LAUNCH_SPEED_MIN + 1));
    r->color = color_rand();
    r->state = RS_RISING;

    for (int i = 0; i < PARTICLES_PER_BURST; i++)
        r->particles[i].active = false;
}

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

        /* Explode at apex or if it exits the top of the screen */
        if (r->vy >= 0.0f || r->y < 2.0f) {
            mparticle_burst(r->particles, PARTICLES_PER_BURST, r->x, r->y);
            r->state = RS_EXPLODED;
        }
        break;

    case RS_EXPLODED: {
        bool any_alive = false;
        for (int i = 0; i < PARTICLES_PER_BURST; i++) {
            mparticle_tick(&r->particles[i], dt_sec);
            if (r->particles[i].active) any_alive = true;
        }
        if (!any_alive) {
            /* Random fuse 0.5–2.5 s before next launch */
            int tps    = (int)(1.0f / dt_sec);
            r->fuse    = tps / 2 + rand() % (tps * 2);
            r->state   = RS_IDLE;
        }
        break;
    }

    } /* switch */
}

/*
 * rocket_draw() — rocket body while rising; matrix particle trails when exploded.
 */
static void rocket_draw(const Rocket *r, int cols, int rows)
{
    if (r->state == RS_RISING) {
        int x = (int)r->x;
        int y = (int)r->y;
        if (x >= 0 && x < cols && y >= 0 && y < rows) {
            attron(COLOR_PAIR(r->color) | A_BOLD);
            mvaddch(y, x, '|');
            attroff(COLOR_PAIR(r->color) | A_BOLD);

            if (y + 1 < rows) {
                attron(COLOR_PAIR(r->color));
                mvaddch(y + 1, x, '\'');
                attroff(COLOR_PAIR(r->color));
            }
        }
    }

    if (r->state == RS_EXPLODED) {
        for (int i = 0; i < PARTICLES_PER_BURST; i++)
            mparticle_draw(&r->particles[i], cols, rows);
    }
}

/* ===================================================================== */
/* §6  show                                                               */
/* ===================================================================== */

typedef struct {
    Rocket rockets[MAX_ROCKETS];
    int    active_rockets;
} Show;

static void show_init(Show *s, int cols, int rows, int rocket_count)
{
    s->active_rockets = rocket_count;

    for (int i = 0; i < MAX_ROCKETS; i++) {
        if (i < rocket_count) {
            rocket_launch(&s->rockets[i], cols, rows);
            /* Stagger so they don't all fire at once */
            s->rockets[i].fuse  = i * 8;
            s->rockets[i].state = RS_IDLE;
        } else {
            s->rockets[i].state = RS_IDLE;
            s->rockets[i].fuse  = INT32_MAX / 2;
            for (int j = 0; j < PARTICLES_PER_BURST; j++)
                s->rockets[i].particles[j].active = false;
        }
    }
}

static void show_free(Show *s) { memset(s, 0, sizeof *s); }

static void show_tick(Show *s, float dt_sec, int cols, int rows)
{
    for (int i = 0; i < s->active_rockets; i++)
        rocket_tick(&s->rockets[i], dt_sec, cols, rows);
}

static void show_draw(const Show *s, int cols, int rows)
{
    for (int i = 0; i < s->active_rockets; i++)
        rocket_draw(&s->rockets[i], cols, rows);
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
    start_color();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_draw_hud(Screen *s, double fps, int sim_fps,
                             int rockets, int theme_idx)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, "%5.1f fps spd:%d rkt:%d [%s]",
             fps, sim_fps, rockets, k_themes[theme_idx].name);

    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(CP_YELLOW) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(CP_YELLOW) | A_BOLD);
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
    Show                  show;
    Screen                screen;
    int                   sim_fps;
    int                   rockets;
    int                   theme_idx;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    show_free(&app->show);
    screen_resize(&app->screen);
    show_init(&app->show, app->screen.cols, app->screen.rows, app->rockets);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    switch (ch) {

    case 'q': case 'Q': case 27:
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

    case 't': case 'T':
        app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
        theme_apply(app->theme_idx);
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

    App *app       = &g_app;
    app->running   = 1;
    app->sim_fps   = SIM_FPS_DEFAULT;
    app->rockets   = ROCKETS_DEFAULT;
    app->theme_idx = 0;

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    theme_apply(app->theme_idx);
    show_init(&app->show, app->screen.cols, app->screen.rows, app->rockets);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* ── resize ──────────────────────────────────────────── */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* ── dt ──────────────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── sim accumulator ─────────────────────────────────── */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            show_tick(&app->show, dt_sec,
                      app->screen.cols, app->screen.rows);
            sim_accum -= tick_ns;
        }

        /* ── FPS counter ─────────────────────────────────────── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* ── frame cap ───────────────────────────────────────── */
        int64_t elapsed = clock_ns() - frame_time + dt;
        int64_t budget  = NS_PER_SEC / 60;
        clock_sleep_ns(budget - elapsed);

        /* ── draw + present ──────────────────────────────────── */
        erase();
        show_draw(&app->show, app->screen.cols, app->screen.rows);
        screen_draw_hud(&app->screen, fps_display,
                        app->sim_fps, app->rockets, app->theme_idx);
        screen_present();

        /* ── input ───────────────────────────────────────────── */
        int key = getch();
        if (key != ERR && !app_handle_key(app, key))
            app->running = 0;
    }

    show_free(&app->show);
    screen_free(&app->screen);
    return 0;
}
