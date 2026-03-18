/*
 * matrix_rain.c  —  ncurses Matrix rain
 *
 * Features:
 *   - Single-threaded, no pthreads
 *   - Double WINDOW buffer (back/front) swapped every frame — no flicker
 *   - Separate overlay WINDOW for FPS counter (never cleared with rain)
 *   - dt (delta-time) loop drives simulation speed independently of CPU
 *   - ASCII characters only — no UTF-8 / Japanese glyphs
 *   - No background attribute — characters dissolve into black naturally
 *   - SIGWINCH resize: rebuilds windows + rain to new terminal dimensions
 *   - Speed control:   ] = faster   [ = slower
 *   - Density control: = = more     - = fewer columns
 *   - Theme cycling:   t = next theme (green / amber / blue / white)
 *   - Clean signal / atexit teardown — terminal always restored
 *
 * Keys:
 *   q / ESC   quit
 *   ]  [      speed up / slow down
 *   =  -      more / fewer columns
 *   t         cycle color theme
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra matrix_rain.c -o matrix_rain -lncurses
 *
 * Sections
 * --------
 *   §1  config    — every tunable constant in one block
 *   §2  clock     — monotonic nanosecond clock, portable sleep
 *   §3  theme     — named color themes; init_pair wrappers
 *   §4  cell      — virtual pixel (char + shade index)
 *   §5  grid      — 2-D cell array (the off-screen framebuffer)
 *   §6  column    — one falling stream of characters
 *   §7  rain      — collection of columns + simulation tick
 *   §8  screen    — double WINDOW buffer + HUD overlay window
 *   §9  app       — dt loop, input, resize, cleanup
 */

#define _POSIX_C_SOURCE 200809L

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
 * Runtime-mutable settings live in the App struct (§9).
 * Hard limits and fixed constants live here.
 */
enum {
    /* Simulation speed — ticks per second */
    SIM_FPS_MIN      =  4,    /* slowest selectable speed             */
    SIM_FPS_DEFAULT  = 20,    /* startup speed                        */
    SIM_FPS_MAX      = 60,    /* fastest selectable speed             */
    SIM_FPS_STEP     =  2,    /* [ / ] increment                      */

    /* Column trail length (fixed; independent of speed) */
    TRAIL_MIN        =  6,    /* shortest trail (cells)               */
    TRAIL_MAX        = 24,    /* longest  trail (cells)               */

    /* Fall speed: rows per tick */
    SPEED_MIN        =  1,
    SPEED_MAX        =  2,

    /* Density: 1 active column per DIVISOR screen columns */
    DENSITY_MIN      =  1,    /* one column per screen column (dense) */
    DENSITY_DEFAULT  =  2,    /* startup: every other column          */
    DENSITY_MAX      =  6,    /* very sparse                          */

    /* Dissolve: clear this many random cells per row per tick */
    DISSOLVE_FRAC    =  4,    /* cols / DISSOLVE_FRAC cells erased    */

    /* HUD overlay */
    HUD_COLS         = 28,    /* width of the status bar window       */
    FPS_UPDATE_MS    = 500,   /* re-measure FPS every 500 ms          */
};

/* Nanosecond helpers */
#define NS_PER_SEC   1000000000LL
#define NS_PER_MS    1000000LL

/* Compute tick length in ns from a ticks-per-second value. */
#define TICK_NS(fps) (NS_PER_SEC / (fps))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

/*
 * clock_ns() — wall-clock nanoseconds from CLOCK_MONOTONIC.
 * CLOCK_MONOTONIC never jumps backward, making it the correct choice
 * for a dt-based game loop.
 */
static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/*
 * clock_sleep_ns() — sleep the requested nanoseconds.
 * nanosleep() handles EINTR transparently here; we don't need to resume
 * because the caller will simply recalculate dt on the next iteration.
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
/* §3  theme                                                              */
/* ===================================================================== */

/*
 * Shade levels — same for every theme.
 * The theme only changes which xterm-256 color is assigned to each level;
 * the rest of the code is theme-unaware and just passes Shade values.
 *
 *   SHADE_FADE   deepest tail — nearly black
 *   SHADE_DARK   far trail
 *   SHADE_MID    mid trail
 *   SHADE_BRIGHT near head
 *   SHADE_HOT    one cell before head — most saturated
 *   SHADE_HEAD   leading character — always bold white
 */
typedef enum {
    SHADE_FADE   = 1,
    SHADE_DARK   = 2,
    SHADE_MID    = 3,
    SHADE_BRIGHT = 4,
    SHADE_HOT    = 5,
    SHADE_HEAD   = 6,
} Shade;

/*
 * Theme — five distinct xterm-256 foreground color indices per shade
 * level (FADE → HOT), plus a display name.  SHADE_HEAD is always
 * COLOR_WHITE in every theme.
 *
 * Each index is a genuine different color so the gradient is rendered
 * by color alone — no A_DIM tricks needed.  A_BOLD is still applied to
 * BRIGHT/HOT/HEAD to leverage the terminal's brightness boost.
 *
 * xterm-256 color reference used here:
 *
 *   green : 22 28 34 40 82   very-dark-green → neon-green
 *   amber : 94 130 172 214 220  dark-orange  → bright-yellow
 *   blue  : 17 19 21 33 51   dark-navy       → bright-cyan
 *   white : 234 238 244 250 255  near-black  → pure-white
 *
 * tmux requirement: add to ~/.tmux.conf —
 *   set -g default-terminal "tmux-256color"
 *   set -as terminal-overrides ",xterm-256color:Tc"
 * or launch with: TERM=xterm-256color tmux
 */
typedef struct {
    const char *name;
    int         fg[5];   /* xterm-256 index for FADE, DARK, MID, BRIGHT, HOT */
} Theme;

/*
 * Each theme has two palette variants:
 *   fg256  — xterm-256 indices; rich gradient; needs COLORS >= 256
 *   fg8    — standard 8-color fallback; uses A_DIM/A_BOLD for depth
 *
 * theme_apply() picks the right variant at runtime by checking COLORS.
 * This means the same binary renders beautifully in a 256-color xterm
 * and still shows a green/amber/blue/white gradient in tmux when
 * TERM is not set to a 256-color value.
 */
static const Theme k_themes[] = {
    { "green", { 22,  28,  34,  40,  82  } },
    { "amber", { 94,  130, 172, 214, 220 } },
    { "blue",  { 17,  19,  21,  33,  51  } },
    { "white", { 234, 238, 244, 250, 255 } },
};

/*
 * 8-color fallback palettes — parallel order to k_themes.
 * Each entry: { FADE, DARK, MID, BRIGHT, HOT } using standard colors.
 * A_DIM and A_BOLD in shade_attr() provide the brightness gradient
 * when all five levels share the same base color.
 */
static const int k_themes_8color[4][5] = {
    { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN  },
    { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW },
    { COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN   },
    { COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/*
 * theme_apply() — load color pairs for the active theme.
 *
 * Selects 256-color or 8-color palette at runtime:
 *   COLORS >= 256  →  rich xterm-256 gradient (distinct color per shade)
 *   COLORS <  256  →  standard 8-color (A_DIM/A_BOLD carry the gradient)
 *
 * Background is always COLOR_BLACK — works in tmux, screen, and xterm.
 * Safe to call at any time; takes effect on the next rendered frame.
 */
static void theme_apply(int theme_idx)
{
    const int *fg = (COLORS >= 256)
                    ? k_themes[theme_idx].fg
                    : k_themes_8color[theme_idx];

    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
}

/*
 * shade_attr() — ncurses attribute for a shade level.
 *
 * A_DIM  on FADE  makes the tail tip near-invisible (vital for 8-color).
 * A_BOLD on BRIGHT/HOT/HEAD gives the terminal brightness boost.
 * In 256-color mode the distinct color indices already carry the gradient;
 * dim/bold add extra punch on top.
 */
static attr_t shade_attr(Shade s)
{
    switch (s) {
    case SHADE_FADE:   return COLOR_PAIR(SHADE_FADE)   | A_DIM;
    case SHADE_DARK:   return COLOR_PAIR(SHADE_DARK);
    case SHADE_MID:    return COLOR_PAIR(SHADE_MID);
    case SHADE_BRIGHT: return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    case SHADE_HOT:    return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    case SHADE_HEAD:   return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    default:           return A_NORMAL;
    }
}

/* ===================================================================== */
/* §4  cell                                                               */
/* ===================================================================== */

/*
 * Cell — one terminal position in the virtual framebuffer.
 * ch == 0  →  empty; renderer leaves the terminal background untouched.
 */
typedef struct {
    char  ch;
    Shade shade;
} Cell;

/* ===================================================================== */
/* §5  grid                                                               */
/* ===================================================================== */

/*
 * Grid — the simulation's off-screen framebuffer.
 * Row-major: cells[y * cols + x].
 */
typedef struct {
    Cell *cells;
    int   cols;
    int   rows;
} Grid;

static void grid_alloc(Grid *g, int cols, int rows)
{
    g->cols  = cols;
    g->rows  = rows;
    g->cells = calloc((size_t)(cols * rows), sizeof(Cell));
}

static void grid_free(Grid *g)
{
    free(g->cells);
    *g = (Grid){0};
}

static void grid_clear(Grid *g)
{
    memset(g->cells, 0, sizeof(Cell) * (size_t)(g->cols * g->rows));
}

static Cell *grid_at(Grid *g, int x, int y)
{
    return &g->cells[y * g->cols + x];
}

/*
 * grid_scatter_erase() — dissolve texture.
 * Picks one random row and erases (cols / DISSOLVE_FRAC) random cells
 * within it.  Stochastic erasure creates the organic fade-to-black look.
 */
static void grid_scatter_erase(Grid *g)
{
    int y     = rand() % g->rows;
    int count = g->cols / DISSOLVE_FRAC;
    for (int i = 0; i < count; i++) {
        int x = rand() % g->cols;
        *grid_at(g, x, y) = (Cell){0};
    }
}

/* ===================================================================== */
/* §6  column                                                             */
/* ===================================================================== */

/*
 * Column — one falling stream of characters.
 *
 *   col     fixed x position for the stream's lifetime
 *   head_y  current row of the leading character (can be negative = above screen)
 *   length  total lit cells including the head
 *   speed   rows the head advances per sim tick
 *   active  false = slot is free
 *
 * Shade gradient (head → tail, by distance from head):
 *   0          HEAD   white
 *   1          HOT    brightest color
 *   2          BRIGHT
 *   3..len/2   MID
 *   ..len-2    DARK
 *   len-1      FADE   near-invisible tail tip
 */
typedef struct {
    int  col;
    int  head_y;
    int  length;
    int  speed;
    bool active;
} Column;

static const char k_ascii[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define ASCII_LEN (int)(sizeof k_ascii - 1)

static char col_rand_char(void)
{
    return k_ascii[rand() % ASCII_LEN];
}

static Shade col_shade_at(int dist, int length)
{
    if (dist == 0)               return SHADE_HEAD;
    if (dist == 1)               return SHADE_HOT;
    if (dist == 2)               return SHADE_BRIGHT;
    if (dist <= length / 2)      return SHADE_MID;
    if (dist <= length - 2)      return SHADE_DARK;
    return SHADE_FADE;
}

static void col_spawn(Column *c, int x, int rows)
{
    c->col    = x;
    c->head_y = -(rand() % (rows / 2));   /* staggered above screen */
    c->length = TRAIL_MIN + rand() % (TRAIL_MAX - TRAIL_MIN + 1);
    c->speed  = SPEED_MIN + rand() % (SPEED_MAX - SPEED_MIN + 1);
    c->active = true;
}

/* Returns false when the entire trail has scrolled off the bottom. */
static bool col_tick(Column *c, int rows)
{
    c->head_y += c->speed;
    return (c->head_y - c->length) < rows;
}

/* Write this column's cells into the grid, re-rolling chars each tick. */
static void col_paint(const Column *c, Grid *g)
{
    for (int dist = 0; dist < c->length; dist++) {
        int y = c->head_y - dist;
        if (y < 0 || y >= g->rows) continue;

        Cell *cell  = grid_at(g, c->col, y);
        cell->ch    = col_rand_char();
        cell->shade = col_shade_at(dist, c->length);
    }
}

/* ===================================================================== */
/* §7  rain                                                               */
/* ===================================================================== */

/*
 * Rain — simulation state.
 *
 * One Column slot per screen column; indexed by x so spawning and
 * retirement are O(1).  density_divisor controls how many slots are
 * kept active: 1 = every column, 2 = every other, etc.
 */
typedef struct {
    Column *columns;          /* [ncols] */
    int     ncols;
    int     nrows;
    int     density_divisor;  /* runtime-mutable via = / - keys */
    Grid    grid;
} Rain;

static void rain_init(Rain *r, int cols, int rows, int density_divisor)
{
    r->ncols            = cols;
    r->nrows            = rows;
    r->density_divisor  = density_divisor;
    r->columns          = calloc((size_t)cols, sizeof(Column));
    grid_alloc(&r->grid, cols, rows);

    /* Seed initial columns according to starting density. */
    for (int x = 0; x < cols; x++) {
        if (x % density_divisor == 0)
            col_spawn(&r->columns[x], x, rows);
    }
}

static void rain_free(Rain *r)
{
    free(r->columns);
    grid_free(&r->grid);
    *r = (Rain){0};
}

/*
 * rain_tick() — one simulation step.
 *
 *   1. Clear the grid (full repaint each tick — simple and correct).
 *   2. Scatter-erase one random row for the dissolve texture.
 *   3. Tick each active column; retire columns that leave the screen.
 *   4. Paint active columns onto the grid.
 *   5. Randomly activate idle slots to organically maintain density.
 */
static void rain_tick(Rain *r)
{
    grid_clear(&r->grid);
    grid_scatter_erase(&r->grid);

    for (int x = 0; x < r->ncols; x++) {
        Column *c = &r->columns[x];

        if (!c->active) {
            /*
             * Idle slot spawn chance scales with density_divisor:
             * lower divisor (denser) → higher chance to respawn quickly.
             */
            int chance = 15 / r->density_divisor;
            if (rand() % 100 < chance)
                col_spawn(c, x, r->nrows);
            continue;
        }

        if (!col_tick(c, r->nrows)) {
            c->active = false;
            continue;
        }

        col_paint(c, &r->grid);
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — single stdscr, ncurses' internal double buffer.
 *
 * ncurses already maintains curscr (what the terminal shows now) and
 * newscr (what we are building this frame).  Every erase/mvwaddch/attron
 * writes into newscr.  doupdate() diffs newscr vs curscr, sends only
 * changed cells in one write, then sets curscr = newscr.
 *
 * This IS the double buffer.  Adding back/front WINDOW pointers creates
 * a third virtual screen ncurses cannot account for, breaking the diff
 * and producing ghost trails.
 *
 * HUD is written into stdscr after the grid (drawn last = always on top).
 * No separate hud_win needed — same buffer, correct Z-order.
 *
 * typeahead(-1): prevents ncurses from interrupting mid-flush to poll
 * stdin, which causes visible tearing at high tick rates.
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

    start_color();
    use_default_colors();

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
    refresh();   /* re-probes terminal size into LINES / COLS */
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw_grid() — paint the grid into stdscr (newscr).
 *
 * erase() clears newscr so cells that became empty this tick appear as
 * black background — no stale characters linger from previous frames.
 * Nothing reaches the terminal until screen_present() is called.
 */
static void screen_draw_grid(Screen *s, const Grid *cur)
{
    erase();

    int total = cur->cols * cur->rows;
    for (int i = 0; i < total; i++) {
        Cell c = cur->cells[i];
        if (c.ch == 0) continue;

        int y = i / cur->cols;
        int x = i % cur->cols;

        if (x >= s->cols || y >= s->rows) continue;

        attr_t attr = shade_attr(c.shade);
        attron(attr);
        mvaddch(y, x, (chtype)(unsigned char)c.ch);
        attroff(attr);
    }
}

/*
 * screen_draw_hud() — write the HUD into stdscr after the grid.
 * Drawn last so it always sits on top of any rain character at row 0.
 */
static void screen_draw_hud(Screen *s,
                              double fps,
                              int    sim_fps,
                              int    density,
                              int    theme_idx)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             "%5.1f fps spd:%d den:%d [%s]",
             fps, sim_fps, density, k_themes[theme_idx].name);

    int hx = s->cols - HUD_COLS;
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(SHADE_HEAD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(SHADE_HEAD) | A_BOLD);
}

/*
 * screen_present() — flush newscr to the terminal.
 *
 * wnoutrefresh(stdscr) copies stdscr into ncurses' newscr model.
 * doupdate() diffs newscr vs curscr and sends one atomic write.
 */
static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — top-level owner of all subsystems.
 *
 * Runtime-mutable state:
 *   sim_fps        ticks per second — changed by ] [
 *   density        column divisor   — changed by = -
 *   theme_idx      active palette   — changed by t
 *
 * Signal flags:
 *   running        set to 0 by SIGINT/SIGTERM → clean exit
 *   need_resize    set to 1 by SIGWINCH → rebuild windows next frame
 *
 * Both flags are sig_atomic_t so signal handlers can write them safely.
 */
typedef struct {
    Rain                  rain;
    Screen                screen;
    int                   sim_fps;
    int                   density;
    int                   theme_idx;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;   /* global — signal handlers need to reach it */

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
    /*
     * endwin() is idempotent — safe to call even if ncurses was never
     * fully initialised or has already been torn down.
     */
    endwin();
}

/* -------------------------------------------------------------------- */
/* resize handler — called from the main loop when need_resize == 1     */
/* -------------------------------------------------------------------- */

/*
 * app_do_resize() — rebuild screen + rain to the new terminal size.
 *
 * We reinitialise Rain entirely (rain_free + rain_init) because the
 * column array is sized to ncols; a resize can change that dimension.
 * density_divisor and sim_fps are preserved across the resize.
 */
static void app_do_resize(App *app)
{
    rain_free(&app->rain);
    screen_resize(&app->screen);

    int cols = app->screen.cols;
    int rows = app->screen.rows;

    rain_init(&app->rain, cols, rows, app->density);

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
 *   ]         speed up   (sim_fps += SIM_FPS_STEP, capped at MAX)
 *   [         slow down  (sim_fps -= SIM_FPS_STEP, capped at MIN)
 *   =         denser     (density_divisor--, capped at MIN)
 *   -         sparser    (density_divisor++, capped at MAX)
 *   t         next theme (wraps around)
 *
 * Speed and density changes are purely in-place; no rain restart needed
 * because rain_tick() reads density_divisor each tick and sim_fps only
 * changes the tick-gate interval in the accumulator logic.
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
        app->density--;
        if (app->density < DENSITY_MIN) app->density = DENSITY_MIN;
        app->rain.density_divisor = app->density;
        break;

    case '-':
        app->density++;
        if (app->density > DENSITY_MAX) app->density = DENSITY_MAX;
        app->rain.density_divisor = app->density;
        break;

    case 't': case 'T':
        app->theme_idx = (app->theme_idx + 1) % THEME_COUNT;
        theme_apply(app->theme_idx);
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
    signal(SIGINT,  on_exit_signal);
    signal(SIGTERM, on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app       = &g_app;
    app->running   = 1;
    app->sim_fps   = SIM_FPS_DEFAULT;
    app->density   = DENSITY_DEFAULT;
    app->theme_idx = 0;

    screen_init(&app->screen);
    theme_apply(app->theme_idx);

    int cols = app->screen.cols;
    int rows = app->screen.rows;

    rain_init(&app->rain, cols, rows, app->density);

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
        /*
         * Handled at the top of the loop so the first frame after a
         * resize always uses correct dimensions.  We also reset dt
         * accounting to avoid a large spike after the rebuild stall.
         */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();   /* reset dt after resize stall */
            sim_accum  = 0;
        }

        /* ── dt measurement ──────────────────────────────────────── */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;

        /* Clamp: if we stall >100 ms don't try to catch up. */
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        /* ── simulation accumulator ──────────────────────────────── */
        /*
         * sim_fps is read fresh each iteration so speed changes take
         * effect immediately without restarting anything.
         */
        int64_t tick_ns = TICK_NS(app->sim_fps);
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            rain_tick(&app->rain);
            sim_accum -= tick_ns;
        }

        /* ── render ──────────────────────────────────────────────── */
        screen_draw_grid(&app->screen, &app->rain.grid);

        /* ── HUD (written into stdscr after grid, drawn on top) ──── */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }
        screen_draw_hud(&app->screen,
                         fps_display,
                         app->sim_fps,
                         app->density,
                         app->theme_idx);

        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;

        /* ── frame cap ───────────────────────────────────────────── */
        /* Sleep the remainder of a 60-fps budget to avoid 100% CPU. */
        int64_t elapsed = clock_ns() - frame_time + dt;
        int64_t budget  = NS_PER_SEC / 60;
        clock_sleep_ns(budget - elapsed);
    }

    /* ── cleanup ─────────────────────────────────────────────────── */
    rain_free(&app->rain);
    screen_free(&app->screen);   /* calls endwin() */

    return 0;
}
