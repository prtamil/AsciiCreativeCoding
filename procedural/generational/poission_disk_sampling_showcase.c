/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * poission_disk_sampling_showcase.c — Bridson's Poisson disk sampling, animated.
 *
 * Scatters points so no two land closer than a set distance r — the even-but-
 * not-gridlike spread graphics people call "blue noise." It grows outward from
 * one seed, looping forever. Sister demo: drunkards_walk_cave_showcase.c (also
 * grows from a centre seed, but as a random walk rather than a spacing rule).
 *
 * Method: Bridson 2007, "Fast Poisson Disk Sampling in Arbitrary Dimensions":
 *   https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
 * Point colouring follows Bourke's data-visualisation colour ramps:
 *   paulbourke.net/texture_colour/colourramp
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config + types ── */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    /* Hard caps on how many points and grid cells we keep room for. A run
     * with r=4 on a full screen lands near 450 points; sized far higher so a
     * smaller r (denser cloud) still fits without ever allocating at runtime. */
    MAX_SAMPLES       = 8192,
    MAX_GRID          = 16384,

    /* How many tries each point gets to place a neighbour before we give up on
     * it. 30 is Bridson's recommended number — more packs points slightly
     * tighter but isn't worth the extra work. */
    ATTEMPTS_PER_POINT = 30,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* How many placement tries we run per simulation tick — the animation
     * speed knob. Higher fills the screen faster. */
    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =  64,
    OPS_PER_TICK_MAX  = 4096,

    RENDER_CAP_FPS    =  60,        /* don't redraw faster than this */
    MAX_FRAME_MS      = 100,        /* longest dt we'll trust; protects against a freeze snowballing */
    FPS_UPDATE_MS     = 500,

    /* Colour slots. HUD/HINT are fixed; resting points span a gradient of
     * RAMP_LEN slots; the live frontier and the fresh-point flash get one each. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_ACTIVE       =   3,        /* 'O' points still trying to place neighbours */
    PAIR_FLASH        =   4,        /* '*' a just-accepted point                   */
    PAIR_RAMP_0       =   5,        /* 'o' settled points, gradient base           */
};

#define RAMP_LEN      6            /* gradient steps from seed colour out to the edge */
#define PAIR_RAMP(k)  (PAIR_RAMP_0 + (k))

/* How fast the '*' flash on a new point fades (down to nothing in ~0.7 s). */
#define POINT_GLOW_DECAY    2.5f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

/* The minimum gap r between any two points, in character cells. Bigger = fewer,
 * more spread-out points that are easy to tell apart; smaller = a denser cloud. */
#define POISSON_R           4.0f
#define SQRT2               1.41421356f   /* grid cell = r/√2, so each cell can hold at most one point */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — one colour scheme for the cloud, switchable live with t/T.
 *
 * The `ramp` is the trick that keeps a finished cloud from looking like a flat
 * sheet of identical dots: each settled point is tinted by how far it sits from
 * the seed, so the spread reads as coloured rings showing the order it grew in.
 * `active` and `flash` are bright accents painted on top. All colour numbers are
 * xterm-256 indices, kept in the bright half so nothing vanishes on black.
 */
typedef struct {
    const char *name;             /* shown in the HUD, e.g. "AURORA"          */
    short       ramp[RAMP_LEN];   /* settled-point colours, seed centre → edge */
    short       active;           /* colour of points still placing neighbours */
    short       flash;            /* colour of a just-accepted point           */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*  name        ramp: seed ───────────────────► edge      active flash */
    { "AURORA", {  33,  39,  45,  51, 123, 195 },              226,  231 },
    { "MATRIX", {  28,  34,  40,  46,  82, 120 },              190,  231 },
    { "NOVA",   {  54,  92, 128, 164, 200, 219 },              213,  231 },
    { "MONO",   { 240, 244, 247, 250, 253, 255 },              231,  255 },
    { "OCEAN",  {  24,  31,  38,  45,  51, 123 },              159,  231 },
    { "FIRE",   {  88, 124, 160, 202, 214, 226 },              214,  231 },
    { "EARTH",  {  94, 130, 136, 179, 180, 230 },              222,  231 },
    { "FOREST", {  28,  64,  70, 106, 148, 190 },              226,  231 },
    { "DESERT", { 137, 143, 179, 185, 221, 230 },              223,  231 },
    { "ARCTIC", {  25,  31,  39,  45, 123, 195 },              159,  231 },
};

/*
 * Sample — one point we've placed and kept.
 *
 * The position is a float, not a grid cell, on purpose: the spacing rule is
 * checked in real coordinates and only snapped to a character cell when drawn,
 * so the "at least r apart" promise is exact rather than rounded to the screen.
 */
typedef struct {
    float x, y;          /* position in cell units (fractional)                 */
    int   attempts_left; /* tries this point has left to place a neighbour;
                          * at 0 it drops off the active list (its area is full) */
    float glow;          /* fade timer for the '*' flash: 1.0 when just placed  */
} Sample;

/*
 * Poisson — the whole sampler: the points we've kept, plus the two helper
 * structures that let it run fast.
 *
 * The background grid answers the one expensive question — "is any existing
 * point within r of this candidate?" — without scanning every point. Because
 * each cell is small enough (r/√2) to hold at most one point, we only have to
 * look at the small box of cells nearby. bg_grid[cell] holds the index of the
 * point sitting in that cell, or -1 for empty.
 *
 * The active queue is the to-do list: points that might still place neighbours.
 * New points only ever sprout near recent ones, so we work off this list and
 * drop a point the moment it runs out of tries. When the list empties, we're done.
 */
typedef struct {
    /* map + lookup grid */
    int    w, h;                  /* map size in cells                            */
    float  radius;                /* the minimum gap r between points             */
    float  cell_size;             /* grid cell = r/√2 (so ≤ 1 point per cell)     */
    int    gw, gh;                /* grid size in cells                           */
    int    bg_grid[MAX_GRID];     /* per cell: index of the point there, or -1    */

    /* every kept point, in the order it was placed */
    Sample samples[MAX_SAMPLES];
    int    n_samples;

    /* indices of points still worth trying; n_active == 0 means finished */
    int    active_queue[MAX_SAMPLES];
    int    n_active;
} Poisson;

/*
 * Which phase the animation is in: growing the cloud, then pausing on the
 * finished result before it wipes and starts over.
 */
typedef enum {
    SCENE_GROWING = 0,
    SCENE_HOLD    = 1,
} SceneState;

/* Scene — everything about one animated run: the sampler, where we are in the
 * grow/hold cycle, the speed knob, and which colour theme is showing. */
typedef struct {
    Poisson     p;              /* the sampler being grown                  */
    SceneState  state;          /* growing, or holding on the finished cloud */
    float       hold_timer;     /* seconds left in the hold phase           */
    bool        paused;         /* user pressed space — freeze everything   */
    int         ops_per_tick;   /* placement tries per tick (the speed knob) */
    int         current_theme;  /* index into themes[]                      */
} Scene;

/* Screen — just the terminal's size in characters. Passed to the drawing code
 * so it doesn't have to reach into the whole App. */
typedef struct { int cols, rows; } Screen;

/* App — the whole program: the running scene, the terminal, the tick rate, the
 * map size we fitted to the window, and the quit/resize flags the signal
 * handlers flip. */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;       /* simulation ticks per second    */
    int                   map_w, map_h;  /* map size in cells, fit to window */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* ── §2  performance  (a steady clock + a sleep to cap the frame rate) ── */

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

/* ── §3  logic  (pure look-and-decide helpers; they read, never change state) ── */

/* Which grid cell does this point fall in? Clamps to the edges, so callers
 * don't have to range-check before storing into bg_grid[]. */
static inline int poisson_grid_idx(const Poisson *p, float x, float y)
{
    int gx = (int)(x / p->cell_size);
    int gy = (int)(y / p->cell_size);
    if (gx < 0) gx = 0;
    if (gy < 0) gy = 0;
    if (gx >= p->gw) gx = p->gw - 1;
    if (gy >= p->gh) gy = p->gh - 1;
    return gy * p->gw + gx;
}

/*
 * Is this spot clear? True if the candidate is on-map and at least r away from
 * every point already placed. Thanks to the cell size, only the 5×5 box of
 * cells around the candidate can hold a point close enough to conflict, so we
 * check just those instead of all of them — that's what makes this fast.
 */
static bool poisson_candidate_valid(const Poisson *p, float cx, float cy)
{
    if (cx < 0 || cx >= (float)p->w) return false;
    if (cy < 0 || cy >= (float)p->h) return false;

    int gx = (int)(cx / p->cell_size);
    int gy = (int)(cy / p->cell_size);
    float r2 = p->radius * p->radius;

    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= p->gw || ny < 0 || ny >= p->gh) continue;
            int idx = p->bg_grid[ny * p->gw + nx];
            if (idx < 0) continue;
            float ex = p->samples[idx].x - cx;
            float ey = p->samples[idx].y - cy;
            if (ex * ex + ey * ey < r2) return false;
        }
    }
    return true;
}

/* Picks a colour step (0..RAMP_LEN-1) for a settled point based on how far it
 * is from the seed — that's what turns the cloud into coloured rings. The
 * caller passes 1/half-diagonal so the farthest point maps to the last step. */
static int ramp_index(const Sample *sm, const Sample *seed, float inv_maxd)
{
    float dx = sm->x - seed->x, dy = sm->y - seed->y;
    int k = (int)(sqrtf(dx * dx + dy * dy) * inv_maxd * RAMP_LEN);
    if (k < 0) k = 0;
    if (k >= RAMP_LEN) k = RAMP_LEN - 1;
    return k;
}

/* ── §4  simulation  (everything that changes the cloud, plus the per-tick step) ── */

static inline float rand_unit(void) { return (float)rand() / (float)RAND_MAX; }

/* Pick a random spot in the ring between r and 2r around a point — a random
 * direction, a random distance in that band. Close enough to be a neighbour,
 * far enough not to overlap. */
static void annulus_candidate(const Sample *seed, float r, float *cx, float *cy)
{
    float a  = 2.0f * (float)M_PI * rand_unit();
    float dr = r + r * rand_unit();
    *cx = seed->x + cosf(a) * dr;
    *cy = seed->y + sinf(a) * dr;
}

/* Keep a candidate: record it, drop it into its grid cell, add it to the active
 * list with a full try budget, and start its flash. */
static void poisson_accept(Poisson *p, float cx, float cy)
{
    if (p->n_samples >= MAX_SAMPLES) return;
    int new_idx = p->n_samples++;
    p->samples[new_idx] = (Sample){
        .x = cx, .y = cy,
        .attempts_left = ATTEMPTS_PER_POINT,
        .glow = 1.0f,
    };
    p->bg_grid[poisson_grid_idx(p, cx, cy)] = new_idx;
    p->active_queue[p->n_active++] = new_idx;
}

/*
 * One placement try: grab a random point off the active list, aim at a spot in
 * its ring, keep that spot if it's clear, and retire the point once it's used up
 * its tries. Returns false when the active list is empty — the cloud is finished.
 */
static bool poisson_step(Poisson *p)
{
    if (p->n_active <= 0)
        return false;

    int qi = rand() % p->n_active;
    Sample *seed = &p->samples[p->active_queue[qi]];
    seed->attempts_left--;

    float cx, cy;
    annulus_candidate(seed, p->radius, &cx, &cy);
    if (poisson_candidate_valid(p, cx, cy))
        poisson_accept(p, cx, cy);

    if (seed->attempts_left <= 0)              /* out of tries, its area is full — drop it */
        p->active_queue[qi] = p->active_queue[--p->n_active];  /* fill the hole with the last entry */
    return true;
}

/* Set up the lookup grid for a w×h map and mark every cell empty. */
static void poisson_init_grid(Poisson *p, int w, int h)
{
    p->w = w;
    p->h = h;
    p->radius = POISSON_R;
    p->cell_size = p->radius / SQRT2;
    p->gw = (int)ceilf((float)w / p->cell_size) + 1;
    p->gh = (int)ceilf((float)h / p->cell_size) + 1;
    if (p->gw * p->gh > MAX_GRID) {
        /* Belt-and-braces: our size caps mean this never trips, but never
         * overrun the fixed grid array if they ever change. */
        p->gw = MAX_GRID / p->gh;
    }
    for (int i = 0; i < p->gw * p->gh; i++) p->bg_grid[i] = -1;
}

/* Place the one starting point near the middle (nudged a little so every run
 * doesn't begin in the exact same spot) and prime the lists with it. */
static void poisson_seed_center(Poisson *p)
{
    float sx = (float)p->w / 2.0f + ((float)(rand() % 7) - 3.0f);
    float sy = (float)p->h / 2.0f + ((float)(rand() % 5) - 2.0f);
    if (sx < 1.0f)                 sx = 1.0f;
    if (sx >= (float)p->w - 1.0f)  sx = (float)p->w - 1.5f;
    if (sy < 1.0f)                 sy = 1.0f;
    if (sy >= (float)p->h - 1.0f)  sy = (float)p->h - 1.5f;

    p->samples[0] = (Sample){
        .x = sx, .y = sy,
        .attempts_left = ATTEMPTS_PER_POINT,
        .glow = 1.0f,
    };
    p->bg_grid[poisson_grid_idx(p, sx, sy)] = 0;
    p->n_samples = 1;
    p->active_queue[0] = 0;
    p->n_active = 1;
}

/* Start over: fresh grid, no points, one new seed. */
static void poisson_reset(Poisson *p, int w, int h)
{
    poisson_init_grid(p, w, h);
    p->n_samples = 0;
    p->n_active  = 0;
    poisson_seed_center(p);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    poisson_reset(&s->p, mw, mh);
    s->state      = SCENE_GROWING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->ops_per_tick  = OPS_PER_TICK_DEF;
    s->current_theme = 0;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Fade every point's flash a little this tick. */
    float decay_pt = expf(-POINT_GLOW_DECAY * dt);
    for (int i = 0; i < s->p.n_samples; i++)
        s->p.samples[i].glow *= decay_pt;

    switch (s->state) {

    case SCENE_GROWING:
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!poisson_step(&s->p)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->p.w, s->p.h);
        }
        break;
    }
}

/* ── §5  render  (turns the scene into characters on screen; touches nothing else) ── */

/* Load a theme's colours into the gradient and accent slots. Falls back to a
 * few basic colours on terminals that can't do 256. */
static void theme_apply(const Theme *t)
{
    if (COLORS >= 256) {
        for (int k = 0; k < RAMP_LEN; k++) init_pair(PAIR_RAMP(k), t->ramp[k], -1);
        init_pair(PAIR_ACTIVE, t->active, -1);
        init_pair(PAIR_FLASH,  t->flash,  -1);
    } else {
        for (int k = 0; k < RAMP_LEN; k++) init_pair(PAIR_RAMP(k), COLOR_BLUE, -1);
        init_pair(PAIR_ACTIVE, COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH,  COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);
    theme_apply(&themes[0]);
}

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
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Poisson *p = &s->p;

    int gx0 = (cols - p->w) / 2;
    int gy0 = ((rows - 3) - p->h) / 2 + 2;   /* leave rows 0-1 for the HUD and the last row for hints */
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    /* We need the seed and the map size up front so each point can be coloured
     * by how far it sits from the seed (the coloured-rings effect). */
    const Sample *seed = &p->samples[0];
    float inv_maxd = 1.0f / (0.5f * sqrtf((float)(p->w * p->w + p->h * p->h)) + 1.0f);

    for (int i = 0; i < p->n_samples; i++) {
        const Sample *sm = &p->samples[i];
        int sx = gx0 + (int)(sm->x + 0.5f);
        int sy = gy0 + (int)(sm->y + 0.5f);
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;

        int  pair, attr;
        char glyph;

        if (sm->glow > GLOW_THRESHOLD) {            /* just placed — flashing */
            pair  = PAIR_FLASH;
            attr  = A_BOLD;
            glyph = '*';
        } else if (sm->attempts_left > 0) {         /* still trying to place neighbours */
            pair  = PAIR_ACTIVE;
            attr  = A_BOLD;
            glyph = 'O';
        } else {                                    /* settled — coloured by distance from seed */
            pair  = PAIR_RAMP(ramp_index(sm, seed, inv_maxd));
            attr  = A_NORMAL;
            glyph = 'o';
        }

        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* Print one HUD line, trimmed so it never spills past the screen edge. */
static void draw_hud_row(const Screen *sc, int row, int pair, int attr, const char *text)
{
    char buf[256];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > sc->cols) buf[sc->cols] = '\0';
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | attr);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Poisson *p = &s->p;
    const char *state_str =
        s->paused                     ? "PAUSED " :
        (s->state == SCENE_GROWING)   ? "GROWING" :
                                        "HOLD   ";

    char line[256];

    /* top line: title, theme, phase, live counts, and rates */
    snprintf(line, sizeof line,
             " Poisson Disk  %s  %s  pts:%d  active:%d  %.1f fps  %d Hz ",
             themes[s->current_theme].name, state_str,
             p->n_samples, p->n_active, fps, sim_fps);
    draw_hud_row(sc, 0, PAIR_HUD, A_BOLD, line);

    /* second line: the current settings */
    snprintf(line, sizeof line,
             " r:%.1f  K:%d  ops/tick:%d  map:%dx%d ",
             p->radius, ATTEMPTS_PER_POINT, s->ops_per_tick, p->w, p->h);
    draw_hud_row(sc, 1, PAIR_HUD, A_NORMAL, line);

    /* bottom line: the key list */
    draw_hud_row(sc, sc->rows - 1, PAIR_HINT, A_BOLD,
                 " q:quit  spc:pause  r:reset  t:theme  +/-:speed  [/]:Hz ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §6  app  (signals, key handling, and the main loop) ── */

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(&themes[s->current_theme]);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(&themes[s->current_theme]);
        break;

    default: break;
    }
    return true;
}

static void install_signals(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Start-up: set the flags and tick rate, open the terminal, size the map to the
 * window, and build the first run. */
static void app_init(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signals();

    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* window was resized — refit before doing anything else */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* Run the simulation at a steady rate no matter how fast we draw:
         * bank the elapsed time and spend it one fixed tick at a time. */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
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

        /* sleep off the rest of the frame so we don't redraw too fast */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* handle one keypress, if any */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
