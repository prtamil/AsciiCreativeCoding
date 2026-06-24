/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * coral.c — Grows a branching coral/lichen shape outward from a centre dot.
 *
 * It's Diffusion-Limited Aggregation (DLA): a seed sits in the middle, then
 * tiny "spore" particles wander in from outside and freeze the moment they
 * touch what's already grown. The outer twigs catch wanderers before they
 * reach the inside, so the shape can't fill solid — it keeps branching, the
 * way coral, lichen, or frost does. Cells are coloured by how far they are
 * from the centre, giving rings of colour.
 *
 * Founding paper: Witten & Sander (1981), "Diffusion-Limited Aggregation, a
 * Kinetic Critical Phenomenon", Phys. Rev. Lett. 47, 1400.
 * Practical notes (launch/kill circles, inward-walk speed-up):
 * Paul Bourke, paulbourke.net/fractals/dla/.
 * Palette-brightness rule the presets follow: documentation/COLOR.md.
 *
 * Build: gcc -std=c11 -O2 -Wall -Wextra coral.c -o coral -lncurses -lm
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

/* ── §1  config ── */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 30,
    SIM_FPS_MAX     = 60,
    SIM_FPS_STEP    =  5,

    WALKER_MIN      =  10,
    WALKER_DEFAULT  = 150,
    WALKER_MAX      = 400,

    GRID_ROWS_MAX   =  80,
    GRID_COLS_MAX   = 300,

    N_CORAL_COLORS  =   6,

    /* How far out new particles start, and how far they may stray before
     * we give up and relaunch them. */
    SPAWN_MARGIN    =   5,   /* start walkers this many cells past the tips */
    KILL_MARGIN     =  20,   /* relaunch a walker that strays this far out  */

    FPS_UPDATE_MS   = 500,

    /* How fast we redraw the screen, and a safety cap so one long pause
     * (e.g. the terminal was buried) can't make the sim try to catch up
     * forever and lock up. */
    RENDER_FPS_CAP  =  60,
    DT_CAP_MS       = 100,

    /* One status bar at the top, one key-list bar at the bottom. The coral
     * draws in the rows between them so the bars never sit on top of it. */
    HUD_ROW_TOP     =   1,
    HUD_ROW_BOTTOM  =   1,
};

/* A full turn in radians — used to pick a random launch angle. */
#define TAU          6.2831853f

#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL
#define TICK_NS(f)   (NS_PER_SEC / (f))

/* ── §2  clock ── */

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

/* ── §3  color ── */

/*
 * Names for our colour-pair slots. The six coral slots are the rings from
 * centre to tip; their actual colours aren't set here — each preset (§3.5)
 * fills them in and they get re-set whenever you switch preset. The two HUD
 * slots stay fixed yellow/cyan, matching every other demo in the project.
 */
typedef enum {
    COL_CORAL_1 = 1,   /* innermost ring (the centre)  */
    COL_CORAL_2 = 2,
    COL_CORAL_3 = 3,
    COL_CORAL_4 = 4,
    COL_CORAL_5 = 5,
    COL_CORAL_6 = 6,   /* outermost ring (the tips)    */
    COL_WALKER  = 7,   /* the drifting spore particles */
    COL_HUD     = 8,   /* top status bar  — yellow     */
    COL_HINT    = 9,   /* bottom key bar  — cyan       */
} ColorID;

/* ── §3.5  presets — 15 visual styles ── */

/*
 * Preset — one named look. Switching preset (n/p) changes how the coral
 * looks AND how bushy it grows, but the underlying method is always the
 * same. The shape grows once and then stops; n/p or r starts a fresh one.
 *
 *   name            shown in the top bar
 *   palette[6]      the six ring colours, centre → tip (xterm-256 colour
 *                   numbers, all kept bright enough to show on black)
 *   walker_col      colour of the drifting spore dots
 *   stick_prob      chance a touching particle actually freezes (0..1).
 *                   High = it almost always sticks at the tips, so the tips
 *                   grab everything first and the shape stays wispy and open.
 *                   Low = particles slip past the tips and pack in deeper,
 *                   giving a denser, blobbier shape.
 *   eight_neighbour true also counts diagonal touches, making thicker clumps;
 *                   false only counts up/down/left/right, giving thin twigs.
 *   inward_bias     out of every 100 steps, how many nudge toward the centre
 *                   (higher = particles reach the shape faster).
 *   glyphs[6]       one character per ring, densest at the centre.
 *   bold_tips       brighten the two outer rings for a glow.
 */
typedef struct {
    const char *name;
    short       palette[N_CORAL_COLORS];
    short       walker_col;
    float       stick_prob;
    bool        eight_neighbour;
    int         inward_bias;
    const char *glyphs;
    bool        bold_tips;
} Preset;

#define N_PRESETS 15

static const Preset presets[N_PRESETS] = {
  /*  name        palette (centre→tip)               walk  stick  8nb  bias glyphs    bold */
  { "RAINBOW",  {203,207,226,118, 86,154}, 251, 0.90f, false, 35, "#++**^", true  },
  { "EMBER",    {130,166,202,208,214,220}, 244, 0.92f, false, 30, "##**^^", true  },
  { "OCEAN",    { 26, 32, 39, 45, 51,123}, 245, 0.88f, false, 35, "#++**:", true  },
  { "NEON",     {201,165,129, 93, 57, 51}, 252, 0.96f, false, 40, "**####", true  },
  { "FOREST",   { 28, 34, 40, 70,106,154}, 244, 0.85f, false, 30, "##++^^", false },
  { "ICE",      { 24, 31, 74,117,159,195}, 250, 0.90f, false, 35, "::****", true  },
  { "GOLD",     { 94,136,178,214,220,229}, 244, 0.90f, false, 30, "%%##++", true  },
  { "AMETHYST", { 55, 91,127,163,170,219}, 252, 0.88f, false, 35, "#++**^", true  },
  { "TOXIC",    { 34, 40, 46, 82,118,154}, 250, 0.70f, true,  30, "######", false },
  { "MONO",     {241,245,248,251,253,255}, 246, 0.90f, false, 35, "##**::", false },
  { "SUNSET",   { 54, 90,126,168,204,220}, 245, 0.88f, false, 35, "#++**^", true  },
  { "ROSE",     {161,168,205,211,217,224}, 252, 0.90f, false, 35, "..oo**", true  },
  { "ELECTRIC", { 27, 33, 39, 45, 51, 87}, 252, 0.97f, false, 25, "::****", true  },
  { "LAVA",     { 52, 88,124,160,196,202}, 244, 0.65f, true,  35, "######", true  },
  { "SPRING",   {120,156,192,228,222,159}, 254, 0.88f, false, 30, "**++::", false },
};

/* Copy this preset's colours into the coral colour slots. */
static void preset_apply(const Preset *p)
{
    if (COLORS >= 256) {
        for (int i = 0; i < N_CORAL_COLORS; i++)
            init_pair(COL_CORAL_1 + i, p->palette[i], COLOR_BLACK);
        init_pair(COL_WALKER, p->walker_col, COLOR_BLACK);
    } else {
        init_pair(COL_CORAL_1, COLOR_RED,     COLOR_BLACK);
        init_pair(COL_CORAL_2, COLOR_MAGENTA, COLOR_BLACK);
        init_pair(COL_CORAL_3, COLOR_YELLOW,  COLOR_BLACK);
        init_pair(COL_CORAL_4, COLOR_GREEN,   COLOR_BLACK);
        init_pair(COL_CORAL_5, COLOR_CYAN,    COLOR_BLACK);
        init_pair(COL_CORAL_6, COLOR_WHITE,   COLOR_BLACK);
        init_pair(COL_WALKER,  COLOR_WHITE,   COLOR_BLACK);
    }
}

static void color_init(void)
{
    start_color();
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(COL_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
    preset_apply(&presets[0]);
}

/* ── §4  grid ── */

/*
 * Grid — the frozen coral so far, plus everything needed to keep growing it.
 *
 * Each cell holds 0 (empty) or a small number 1..6 that is both "frozen"
 * and the colour ring to draw it with. Once a cell freezes it never changes,
 * and later particles need to see it right away to build off it — so a single
 * array is correct here, no double-buffering needed. (DLA model: Witten &
 * Sander 1981.)
 */
typedef struct {
    /* the frozen shape itself */
    uint8_t cells[GRID_ROWS_MAX][GRID_COLS_MAX];  /* 0 = empty; 1..6 = frozen,
                                                   * the number is its colour  */
    int     frozen_count;   /* how many cells are frozen (shown in the HUD)   */

    /* where the centre is and how far the shape has spread */
    int     ccx, ccy;       /* the centre cell — where the seed sits          */
    int     max_radius;     /* distance of the farthest frozen cell out       */
    int     reset_radius;   /* growth is finished once max_radius hits this
                             * (about half the shorter screen side); also the
                             * number we divide by to turn distance → colour   */
    int     rows, cols;     /* grid size in cells                             */

    /* The current preset's growth/look settings, copied in so the rest of
     * the code reads them here instead of digging back into presets[]. */
    float       stick_prob;       /* chance a touching particle freezes      */
    bool        eight_neighbour;  /* count diagonal touches too (thicker)    */
    int         inward_bias;      /* steps-per-100 nudged toward the centre  */
    const char *glyphs;           /* 6 ring characters; points into presets[] */
    bool        bold_tips;        /* brighten the two outer rings            */
} Grid;

/* How far the cell (cx,cy) is from the centre, rounded to a whole number. */
static int grid_radius(const Grid *g, int cx, int cy)
{
    int dx = cx - g->ccx, dy = cy - g->ccy;
    return (int)(sqrtf((float)(dx * dx + dy * dy)) + 0.5f);
}

/* Turn a distance-from-centre into a ring colour: 0 = innermost, far = tip. */
static uint8_t grid_color_for_radius(const Grid *g, int radius)
{
    int idx = (int)((float)radius / (float)g->reset_radius * N_CORAL_COLORS);
    if (idx < 0)               idx = 0;
    if (idx >= N_CORAL_COLORS) idx = N_CORAL_COLORS - 1;
    return (uint8_t)(idx + 1);
}

static void grid_place_seed(Grid *g)
{
    g->cells[g->ccy][g->ccx] = COL_CORAL_1;
    g->frozen_count++;
}

static void grid_init(Grid *g, int cols, int rows, const Preset *p)
{
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols < 1) cols = 1;            /* on a tiny terminal the HUD rows  */
    if (rows < 1) rows = 1;            /* could leave nothing — keep 1x1   */
    memset(g->cells, 0, sizeof g->cells);
    g->frozen_count = 0;
    g->cols         = cols;
    g->rows         = rows;
    g->ccx          = cols / 2;
    g->ccy          = rows / 2;
    g->max_radius   = 0;

    /* Take on this preset's growth + look settings. */
    g->stick_prob      = p->stick_prob;
    g->eight_neighbour = p->eight_neighbour;
    g->inward_bias     = p->inward_bias;
    g->glyphs          = p->glyphs;
    g->bold_tips       = p->bold_tips;

    /* Stop growing once the shape reaches roughly the nearest screen edge. */
    int half = (cols < rows ? cols : rows) / 2;
    g->reset_radius = half > 1 ? half - 1 : 1;

    grid_place_seed(g);
}

static void grid_freeze(Grid *g, int cx, int cy)
{
    int r = grid_radius(g, cx, cy);
    g->cells[cy][cx] = grid_color_for_radius(g, r);
    g->frozen_count++;
    if (r > g->max_radius) g->max_radius = r;
}

static bool grid_frozen(const Grid *g, int cx, int cy)
{
    if (cx < 0 || cx >= g->cols || cy < 0 || cy >= g->rows) return false;
    return g->cells[cy][cx] != 0;
}

/*
 * Is this cell next to the frozen shape? If so, a particle sitting here may
 * freeze. The preset decides whether diagonal neighbours count: counting
 * only the four sides gives thin twigs; counting diagonals too gives
 * thicker, fuller growth.
 */
static bool grid_touching(const Grid *g, int cx, int cy)
{
    if (grid_frozen(g, cx + 1, cy) || grid_frozen(g, cx - 1, cy)
     || grid_frozen(g, cx, cy + 1) || grid_frozen(g, cx, cy - 1))
        return true;
    if (g->eight_neighbour)
        return grid_frozen(g, cx + 1, cy + 1) || grid_frozen(g, cx - 1, cy + 1)
            || grid_frozen(g, cx + 1, cy - 1) || grid_frozen(g, cx - 1, cy - 1);
    return false;
}

static void grid_draw(const Grid *g, WINDOW *w)
{
    for (int cy = 0; cy < g->rows; cy++) {
        for (int cx = 0; cx < g->cols; cx++) {
            uint8_t col = g->cells[cy][cx];
            if (col == 0) continue;

            attr_t attr = COLOR_PAIR((int)col);
            if (g->bold_tips && col >= 5) attr |= A_BOLD;   /* brighten the tips */

            /* pick this ring's character from the preset */
            char ch = (col >= 1 && col <= N_CORAL_COLORS) ? g->glyphs[col - 1] : '*';

            wattron(w, attr);
            mvwaddch(w, cy + HUD_ROW_TOP, cx, (chtype)(unsigned char)ch);
            wattroff(w, attr);
        }
    }
}

/* ── §5  walker ── */

/*
 * Walker — one drifting "spore" particle looking for the shape.
 *
 * It starts on a circle just outside the shape and wanders randomly, with a
 * gentle pull toward the centre so it actually finds the shape instead of
 * floating off. When it first touches, it freezes (sometimes — see
 * stick_prob). The pull strength and freeze chance both come from the
 * current preset.
 */
typedef struct {
    int  cx, cy;     /* where the particle is right now (grid cell)            */
    bool active;     /* false = this slot is unused. walkers[] is a fixed pool;
                      * only the first Control.n_walkers slots are in play.    */
} Walker;

/*
 * Drop a particle at a random spot on a circle just outside the shape.
 * Starting close to the tips (not way out at the screen edge) means it
 * reaches the shape in a few steps, so the animation keeps moving.
 */
static void walker_spawn(Walker *w, const Grid *g)
{
    float radius = (float)g->max_radius + (float)SPAWN_MARGIN;
    float theta  = ((float)(rand() % 1000) / 1000.0f) * TAU;

    int cx = g->ccx + (int)(radius * cosf(theta));
    int cy = g->ccy + (int)(radius * sinf(theta));
    if (cx < 0) cx = 0; else if (cx >= g->cols) cx = g->cols - 1;
    if (cy < 0) cy = 0; else if (cy >= g->rows) cy = g->rows - 1;

    w->cx = cx;
    w->cy = cy;
    w->active = true;
}

/* If the particle is touching the shape and wins a stick_prob coin flip,
 * freeze it here. Returns true if it froze. */
static bool try_stick(Walker *w, Grid *g)
{
    if (grid_frozen(g, w->cx, w->cy) || !grid_touching(g, w->cx, w->cy))
        return false;
    if ((float)rand() / (float)RAND_MAX >= g->stick_prob)
        return false;
    grid_freeze(g, w->cx, w->cy);
    return true;
}

/* Which way to step (-1, 0, or +1) to get one cell closer to `to`. */
static inline int step_toward(int from, int to)
{
    return (from < to) - (from > to);
}

/*
 * Has the particle wandered too far to be useful? True if it left the grid
 * or drifted well past the shape. When that happens we just relaunch it
 * instead of letting it roam pointlessly far away.
 */
static bool out_of_play(const Grid *g, int x, int y)
{
    if (x < 0 || x >= g->cols || y < 0 || y >= g->rows) return true;
    return grid_radius(g, x, y) > g->max_radius + KILL_MARGIN;
}

/*
 * Pick this step's direction. Most of the time (inward_bias out of 100) it
 * heads toward the centre so the particle keeps closing in; otherwise it
 * goes a random one of the four directions. Result comes back via dx,dy.
 */
static void walker_pick_step(const Walker *w, const Grid *g, int *dx, int *dy)
{
    *dx = 0;
    *dy = 0;
    if (rand() % 100 < g->inward_bias) {
        int tox = step_toward(w->cx, g->ccx);
        int toy = step_toward(w->cy, g->ccy);
        if ((rand() & 1) && tox) *dx = tox;
        else if (toy)            *dy = toy;
        else                     *dx = tox;
    } else {
        switch (rand() % 4) {
            case 0:  *dx =  1; break;
            case 1:  *dx = -1; break;
            case 2:  *dy =  1; break;
            default: *dy = -1; break;
        }
    }
}

/*
 * Move the particle one step and see if it freezes. Returns true if it did,
 * which tells the caller to launch a fresh particle in its place.
 */
static bool walker_tick(Walker *w, Grid *g)
{
    if (!w->active) return false;

    int dx, dy;
    walker_pick_step(w, g, &dx, &dy);
    int nx = w->cx + dx;
    int ny = w->cy + dy;

    if (out_of_play(g, nx, ny)) {              /* drifted too far → restart it */
        walker_spawn(w, g);
        return false;
    }

    /* Next cell is already part of the shape: try to stick where we are. */
    if (grid_frozen(g, nx, ny))
        return try_stick(w, g);

    /* Otherwise move, then try to stick at the new spot. */
    w->cx = nx;
    w->cy = ny;
    return try_stick(w, g);
}

/* ── §6  scene ── */

/*
 * Control — the settings the user changes with keys, kept in one spot. The
 * allowed ranges live in §1 config; app_handle_key keeps these inside them.
 */
typedef struct {
    int  preset_idx;      /* n/p   — which visual style (index into presets[]) */
    int  n_walkers;       /* +/-   — how many particles are drifting           */
    int  sim_fps;         /* [ ]   — how many growth steps per second          */
    bool paused;          /* space — is growth frozen?                         */
} Control;

/*
 * Scene — the whole running demo in one place:
 *   grid + walkers   the shape and the particles building it
 *   ctrl             the user's settings
 *   done             true once growth has finished and we're just holding
 */
typedef struct {
    Grid    grid;
    Walker  walkers[WALKER_MAX];
    Control ctrl;
    bool    done;
} Scene;

/*
 * Start a fresh growth at this size in the current preset's style: set its
 * colours, reseed the grid, and launch the particles. Shared by startup,
 * regrow, and preset-switch.
 */
static void scene_build(Scene *s, int cols, int rows)
{
    const Preset *p = &presets[s->ctrl.preset_idx];
    preset_apply(p);
    grid_init(&s->grid, cols, rows, p);
    s->done = false;

    for (int i = 0; i < WALKER_MAX; i++)
        s->walkers[i].active = false;
    for (int i = 0; i < s->ctrl.n_walkers; i++)
        walker_spawn(&s->walkers[i], &s->grid);
}

/* Full setup for startup or a resize. Resets particle count and pause, but
 * keeps the chosen preset and speed (those are set once at startup). */
static void scene_init(Scene *s, int cols, int rows)
{
    s->ctrl.n_walkers = WALKER_DEFAULT;
    s->ctrl.paused    = false;
    scene_build(s, cols, rows);
}

/* Restart growth at the same size, keeping the current settings. Only the
 * user triggers this (r, or n/p) — nothing restarts on its own. */
static void scene_regrow(Scene *s)
{
    scene_build(s, s->grid.cols, s->grid.rows);
}

static void scene_tick(Scene *s)
{
    if (s->ctrl.paused || s->done) return;

    /* The shape has reached the edge — mark it finished and just hold it.
     * It won't restart by itself; the user does that with n/p or r. */
    if (s->grid.max_radius >= s->grid.reset_radius) {
        s->done = true;
        return;
    }

    for (int i = 0; i < s->ctrl.n_walkers; i++) {
        bool froze = walker_tick(&s->walkers[i], &s->grid);
        if (froze)
            walker_spawn(&s->walkers[i], &s->grid);
    }
}

static void scene_draw(const Scene *s, WINDOW *w)
{
    grid_draw(&s->grid, w);

    if (s->done) return;          /* finished — don't draw the particles */

    wattron(w, COLOR_PAIR(COL_WALKER) | A_DIM);
    for (int i = 0; i < s->ctrl.n_walkers; i++) {
        const Walker *wk = &s->walkers[i];
        if (!wk->active) continue;
        if (wk->cy < 0 || wk->cy >= s->grid.rows) continue;
        if (wk->cx < 0 || wk->cx >= s->grid.cols) continue;
        if (s->grid.cells[wk->cy][wk->cx] == 0)
            mvwaddch(w, wk->cy + HUD_ROW_TOP, wk->cx, (chtype)'.');
    }
    wattroff(w, COLOR_PAIR(COL_WALKER) | A_DIM);
}

/* ── §7  screen ── */

/*
 * Screen — the terminal's current width and height, remembered here. Every
 * frame needs the size to lay things out, but it only changes on a resize,
 * so we read it once (and again on resize) rather than every frame.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
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
 * Paint one full-width coloured bar of text across `row`. The text is cut
 * off at the screen width on purpose — using mvaddnstr instead of mvprintw
 * stops a too-long line from wrapping down onto the coral.
 */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    if (row < 0 || cols < 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps)
{
    erase();
    scene_draw(sc, stdscr);

    const Control *c = &sc->ctrl;

    /* Top bar: name, current preset, status, and live counters. */
    char data[160];
    snprintf(data, sizeof data,
             " CORAL DLA  %s (%d/%d)  %s  frozen:%-5d  walkers:%-3d  %5.1f fps  %d Hz ",
             presets[c->preset_idx].name, c->preset_idx + 1, N_PRESETS,
             c->paused ? "PAUSED " : sc->done ? "DONE   " : "growing",
             sc->grid.frozen_count, c->n_walkers, fps, c->sim_fps);

    /* Bottom bar: the list of keys you can press. */
    static const char *keys =
        " q:quit  spc:pause  n/p:preset  r:reset  +/-:walkers  [/]:speed ";

    hud_bar(0,           s->cols, COL_HUD,  data);
    hud_bar(s->rows - 1, s->cols, COL_HINT, keys);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §8  app ── */

/*
 * App — everything the program runs on: the demo, the screen size, and two
 * flags that signal handlers flip. It lives in one global because signal
 * handlers can't be handed a pointer, so they reach the program through this
 * one known spot. It's the only global; everything else is passed around.
 *
 * The two flags use volatile sig_atomic_t because that's the only kind of
 * variable a signal handler is allowed to touch safely.
 */
typedef struct {
    Scene                 scene;       /* the demo (shape + settings)        */
    Screen                screen;      /* current terminal size              */
    volatile sig_atomic_t running;     /* set to 0 to quit (Ctrl-C / kill)   */
    volatile sig_atomic_t need_resize; /* set to 1 when the window resized   */
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Build the scene to fit the screen, leaving the top and bottom HUD rows
 * free. The one place that turns screen size into grid size. */
static void app_build_scene(App *app)
{
    scene_init(&app->scene, app->screen.cols,
               app->screen.rows - HUD_ROW_TOP - HUD_ROW_BOTTOM);
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_build_scene(app);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Control *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27: return false;

    case ' ':
        c->paused = !c->paused;
        break;

    case 'n': case 'N':   /* next preset */
        c->preset_idx = (c->preset_idx + 1) % N_PRESETS;
        scene_regrow(&app->scene);
        break;

    case 'p': case 'P':   /* previous preset */
        c->preset_idx = (c->preset_idx + N_PRESETS - 1) % N_PRESETS;
        scene_regrow(&app->scene);
        break;

    case 'r': case 'R':   /* restart growth, same preset */
        scene_regrow(&app->scene);
        break;

    case '=': case '+':
        if (c->n_walkers < WALKER_MAX) {
            int i = c->n_walkers++;
            walker_spawn(&app->scene.walkers[i], &app->scene.grid);
        }
        break;

    case '-':
        if (c->n_walkers > WALKER_MIN)
            c->n_walkers--;
        break;

    case ']':
        c->sim_fps += SIM_FPS_STEP;
        if (c->sim_fps > SIM_FPS_MAX) c->sim_fps = SIM_FPS_MAX;
        break;

    case '[':
        c->sim_fps -= SIM_FPS_STEP;
        if (c->sim_fps < SIM_FPS_MIN) c->sim_fps = SIM_FPS_MIN;
        break;

    default: break;
    }
    return true;
}

/*
 * Run the growth at a steady pace no matter the frame rate. We add up the
 * real time that has passed and spend it in fixed-size ticks; whatever
 * doesn't fill a whole tick is carried over to next frame in *sim_accum.
 */
static void app_step_simulation(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->scene.ctrl.sim_fps);

    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene);
        *sim_accum -= tick_ns;
    }
}

/*
 * Sleep just enough that each frame lasts about the same length, so we hold
 * a steady frame rate instead of spinning the CPU as fast as it can go.
 */
static void app_pace_frame(int64_t frame_start, int64_t frame_dt)
{
    int64_t budget_ns  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed_ns = clock_ns() - frame_start + frame_dt;
    clock_sleep_ns(budget_ns - elapsed_ns);
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
    app->scene.ctrl.sim_fps = SIM_FPS_DEFAULT;   /* set once; kept across resizes */

    screen_init(&app->screen);
    app_build_scene(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* How long since the last frame? Capped so one long pause can't make
         * the sim try to replay a huge backlog and freeze up. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;

        /* Grow by that much time. */
        app_step_simulation(app, dt, &sim_accum);

        /* Refresh the fps number every so often. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Wait out the frame, then draw — keeps the pace even. */
        app_pace_frame(frame_time, dt);
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        /* Handle one keypress if there is one. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
