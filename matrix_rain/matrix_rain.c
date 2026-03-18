/*
 * matrix_rain.c  —  ncurses Matrix rain
 *
 * Features:
 *   - Single-threaded, no pthreads
 *   - ncurses internal double buffer (stdscr / doupdate) — no flicker
 *   - dt (delta-time) loop drives simulation speed independently of CPU
 *   - RENDER INTERPOLATION: column scroll positions are alpha-projected
 *     between ticks so motion is silky smooth at any sim speed
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
 *   §8  screen    — stdscr double buffer + HUD
 *   §9  app       — dt loop, input, resize, cleanup
 *
 * ── RENDER INTERPOLATION ──────────────────────────────────────────────
 *
 * The problem without alpha:
 *   rain_tick() advances head_y by an integer number of rows.
 *   The accumulator fires 0 or 1 ticks per render frame.
 *   When 0 ticks fire, every column is drawn at the same position as
 *   the previous frame → visible stutter/judder at any sim speed.
 *
 * The fix:
 *   After draining the accumulator, sim_accum holds the leftover
 *   nanoseconds — how far we are into the next tick that has not
 *   fired yet.
 *
 *   alpha = sim_accum / tick_ns        ∈ [0.0, 1.0)
 *
 *   At draw time, each column's head is projected forward by
 *   (speed * alpha) fractional rows:
 *
 *     draw_head_y = head_y + speed * alpha
 *
 *   col_paint_interpolated() uses floorf(draw_head_y - dist + 0.5f)
 *   to map the fractional position to a terminal row — "round half up",
 *   the same deterministic rounding used in bounce.c px_to_cell_y().
 *
 * Why forward extrapolation is correct here:
 *   Columns move at constant integer speed (rows/tick) with no
 *   acceleration.  Projecting forward from the current state is
 *   numerically identical to interpolating between prev and current
 *   positions, and requires no extra storage in Column.
 *   If you add variable speed or acceleration, store prev_head_y and
 *   lerp between prev and current instead.
 *
 * Effect:
 *   At sim_fps=20, render at 60 Hz: without alpha, 40 of 60 frames
 *   show no movement.  With alpha, every frame shows a unique
 *   sub-row position — continuous, smooth scroll.
 *
 * Changes from the non-interpolated version:
 *   1. col_paint() gains a float offset parameter → renamed
 *      col_paint_interpolated()
 *   2. rain_draw() is a new function: iterates columns, computes
 *      draw_head_y = head_y + speed*alpha, calls col_paint_interpolated.
 *      rain_tick() is unchanged — physics untouched.
 *   3. screen_draw_rain() calls rain_draw() instead of painting from
 *      the grid.  The Grid struct is kept for the dissolve texture
 *      but is no longer the sole source for drawing.
 *   4. alpha is computed in main() after the accumulator and passed
 *      to screen_draw_rain().
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

enum {
    SIM_FPS_MIN      =  4,
    SIM_FPS_DEFAULT  = 20,
    SIM_FPS_MAX      = 60,
    SIM_FPS_STEP     =  2,

    TRAIL_MIN        =  6,
    TRAIL_MAX        = 24,

    SPEED_MIN        =  1,
    SPEED_MAX        =  2,

    DENSITY_MIN      =  1,
    DENSITY_DEFAULT  =  2,
    DENSITY_MAX      =  6,

    DISSOLVE_FRAC    =  4,

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
/* §3  theme                                                              */
/* ===================================================================== */

typedef enum {
    SHADE_FADE   = 1,
    SHADE_DARK   = 2,
    SHADE_MID    = 3,
    SHADE_BRIGHT = 4,
    SHADE_HOT    = 5,
    SHADE_HEAD   = 6,
} Shade;

typedef struct {
    const char *name;
    int         fg[5];
} Theme;

static const Theme k_themes[] = {
    { "green", { 22,  28,  34,  40,  82  } },
    { "amber", { 94,  130, 172, 214, 220 } },
    { "blue",  { 17,  19,  21,  33,  51  } },
    { "white", { 234, 238, 244, 250, 255 } },
};

static const int k_themes_8color[4][5] = {
    { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN  },
    { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW },
    { COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN   },
    { COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

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

typedef struct {
    char  ch;
    Shade shade;
} Cell;

/* ===================================================================== */
/* §5  grid                                                               */
/* ===================================================================== */

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
 * head_y is an integer physics position (rows).  It is advanced by
 * col_tick() each sim tick.  Drawing uses a projected float position
 * (head_y + speed * alpha) computed in col_paint_interpolated().
 *
 * ch_cache[] stores one random character per trail cell, seeded on spawn
 * and refreshed each tick.  This ensures the interpolated draw always
 * has a character to display even for rows that were not yet written to
 * the grid — important because col_paint_interpolated() bypasses the
 * grid entirely for the live columns.
 */
typedef struct {
    int  col;
    int  head_y;
    int  length;
    int  speed;
    bool active;
    char ch_cache[TRAIL_MAX]; /* one char per trail slot, refreshed each tick */
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
    c->head_y = -(rand() % (rows / 2));
    c->length = TRAIL_MIN + rand() % (TRAIL_MAX - TRAIL_MIN + 1);
    c->speed  = SPEED_MIN + rand() % (SPEED_MAX - SPEED_MIN + 1);
    c->active = true;
    for (int i = 0; i < c->length; i++)
        c->ch_cache[i] = col_rand_char();
}

static bool col_tick(Column *c, int rows)
{
    c->head_y += c->speed;
    /* Refresh characters each tick so glyphs shimmer as they fall */
    for (int i = 0; i < c->length; i++)
        c->ch_cache[i] = col_rand_char();
    return (c->head_y - c->length) < rows;
}

/*
 * col_paint() — write this column into the grid (used by rain_tick for
 * the dissolve/persistence texture; NOT used for the interpolated draw).
 */
static void col_paint(const Column *c, Grid *g)
{
    for (int dist = 0; dist < c->length; dist++) {
        int y = c->head_y - dist;
        if (y < 0 || y >= g->rows) continue;
        Cell *cell  = grid_at(g, c->col, y);
        cell->ch    = c->ch_cache[dist];
        cell->shade = col_shade_at(dist, c->length);
    }
}

/*
 * col_paint_interpolated() — draw one column directly to stdscr using
 * a fractional head position.
 *
 * draw_head_y = head_y + speed * alpha
 *
 *   alpha ∈ [0.0, 1.0) is the fractional tick remainder from the
 *   accumulator.  It projects the head forward by the fraction of a
 *   tick that has elapsed since the last physics step.
 *
 * Row mapping — "round half up":
 *   row = (int)floorf(draw_head_y - dist + 0.5f)
 *
 *   floorf(x + 0.5f) is used instead of roundf(x) to avoid the
 *   "round half to even" tie-breaking in roundf(), which causes a
 *   cell to oscillate between two rows when the fractional part is
 *   exactly 0.5.  floorf(x+0.5) always breaks ties upward —
 *   deterministic, no flicker.
 *
 * Characters come from ch_cache[], seeded at spawn and refreshed
 * each tick in col_tick(), so every rendered frame shows the latest
 * shimmer without requiring the grid to be consulted.
 *
 * col must be < cols, draw_head_y must be computed by the caller.
 */
static void col_paint_interpolated(const Column *c,
                                   float draw_head_y,
                                   int cols, int rows)
{
    if (c->col >= cols) return;

    for (int dist = 0; dist < c->length; dist++) {
        int row = (int)floorf(draw_head_y - (float)dist + 0.5f);
        if (row < 0 || row >= rows) continue;

        char  ch   = c->ch_cache[dist];
        Shade shade = col_shade_at(dist, c->length);

        attr_t attr = shade_attr(shade);
        attron(attr);
        mvaddch(row, c->col, (chtype)(unsigned char)ch);
        attroff(attr);
    }
}

/* ===================================================================== */
/* §7  rain                                                               */
/* ===================================================================== */

typedef struct {
    Column *columns;
    int     ncols;
    int     nrows;
    int     density_divisor;
    Grid    grid;
} Rain;

static void rain_init(Rain *r, int cols, int rows, int density_divisor)
{
    r->ncols            = cols;
    r->nrows            = rows;
    r->density_divisor  = density_divisor;
    r->columns          = calloc((size_t)cols, sizeof(Column));
    grid_alloc(&r->grid, cols, rows);

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
 * rain_tick() — one simulation step.  UNCHANGED from original.
 *
 * The grid is still updated here so the dissolve/persistence texture
 * (grid_scatter_erase) keeps working — it is painted underneath the
 * interpolated live columns each frame.
 */
static void rain_tick(Rain *r)
{
    grid_clear(&r->grid);
    grid_scatter_erase(&r->grid);

    for (int x = 0; x < r->ncols; x++) {
        Column *c = &r->columns[x];

        if (!c->active) {
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

/*
 * rain_draw() — render all columns to stdscr at interpolated positions.
 *
 * Two-pass approach:
 *
 *   Pass 1 — grid (dissolve texture):
 *     Draws the persistent grid cells that rain_tick() painted.
 *     These are the fading echoes of columns that have already passed.
 *     Drawn first so live column heads paint on top.
 *
 *   Pass 2 — live columns (interpolated):
 *     For each active column, computes:
 *       draw_head_y = head_y + speed * alpha
 *     then calls col_paint_interpolated() to draw directly to stdscr
 *     at the fractional-row position.
 *
 * Why two passes instead of only the grid:
 *   The grid is snapped to integer rows (col_paint writes integer y).
 *   Replacing col_paint with col_paint_interpolated in rain_tick() would
 *   write fractional positions into an integer grid — the grid cannot
 *   store sub-row data.  Instead, the grid carries the persistence
 *   texture (fade/dissolve) and the live columns are drawn fresh each
 *   frame at the interpolated float position, bypassing the grid.
 *
 * alpha = 0.0  →  draw_head_y == head_y  (no change from tick position)
 * alpha = 0.9  →  draw_head_y is 90% of a tick ahead of head_y
 */
static void rain_draw(const Rain *r, float alpha, int cols, int rows)
{
    /* Pass 1 — grid (dissolve/persistence texture, integer positions) */
    const Grid *g = &r->grid;
    int total = g->cols * g->rows;
    for (int i = 0; i < total; i++) {
        Cell c = g->cells[i];
        if (c.ch == 0) continue;

        int y = i / g->cols;
        int x = i % g->cols;
        if (x >= cols || y >= rows) continue;

        attr_t attr = shade_attr(c.shade);
        attron(attr);
        mvaddch(y, x, (chtype)(unsigned char)c.ch);
        attroff(attr);
    }

    /* Pass 2 — live column heads at interpolated float positions */
    for (int x = 0; x < r->ncols; x++) {
        const Column *c = &r->columns[x];
        if (!c->active) continue;

        /*
         * Project head forward by (speed * alpha) fractional rows.
         *
         * Example: sim at 20 Hz, render at 60 Hz.
         *   tick fires every 50 ms.
         *   Render fires 33 ms after last tick → alpha ≈ 0.66
         *   Column speed = 1 row/tick:
         *     draw_head_y = head_y + 1 * 0.66 = head_y + 0.66
         *   Rounded to nearest row: shows head_y or head_y+1 depending
         *   on whether we have crossed the 0.5 threshold.
         *   Without alpha: all 3 render frames in this tick show head_y.
         *   With alpha:    frame 1 ≈ 0.33→ head_y, frames 2–3 ≈ 0.66/1.0→ head_y+1.
         *   The column visually scrolls at 20 sim-rows/s regardless of
         *   render rate — perfectly smooth.
         */
        float draw_head_y = (float)c->head_y + (float)c->speed * alpha;
        col_paint_interpolated(c, draw_head_y, cols, rows);
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

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
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/*
 * screen_draw_rain() — build the complete frame in stdscr (newscr).
 *
 * alpha and dt_sec are passed through to rain_draw() for interpolation.
 *
 * Order:
 *   1. erase()         — clear newscr (stale cells become black)
 *   2. rain_draw()     — grid texture (pass 1) + live columns (pass 2)
 *   3. screen_draw_hud — HUD written last, always on top
 *
 * Nothing reaches the terminal until screen_present().
 */
static void screen_draw_rain(Screen *s, const Rain *r, float alpha)
{
    erase();
    rain_draw(r, alpha, s->cols, s->rows);
}

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

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

typedef struct {
    Rain                  rain;
    Screen                screen;
    int                   sim_fps;
    int                   density;
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
    rain_free(&app->rain);
    screen_resize(&app->screen);
    rain_init(&app->rain, app->screen.cols, app->screen.rows, app->density);
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
    app->density   = DENSITY_DEFAULT;
    app->theme_idx = 0;

    screen_init(&app->screen);
    theme_apply(app->theme_idx);
    rain_init(&app->rain, app->screen.cols, app->screen.rows, app->density);

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
            rain_tick(&app->rain);
            sim_accum -= tick_ns;
        }

        /*
         * ── render interpolation alpha ────────────────────────────
         *
         * sim_accum is the leftover nanoseconds after all complete
         * ticks have been drained.  It represents how far we are into
         * the next tick that has not yet fired.
         *
         * alpha = sim_accum / tick_ns       ∈ [0.0, 1.0)
         *
         * Passed to rain_draw() → col_paint_interpolated(), which adds
         *   speed * alpha  fractional rows to each column's drawn head.
         *
         * At sim_fps=20, render at 60 Hz:
         *   tick fires every 50 ms.  Each render frame is 16.7 ms apart.
         *   Frame 1 after tick: alpha ≈ 0.33  → head + 0.33 rows
         *   Frame 2 after tick: alpha ≈ 0.66  → head + 0.66 rows
         *   Frame 3 after tick: alpha ≈ 1.00  → head + 1.00 rows (new tick fires)
         *   Result: column visually advances 1 row over 3 render frames.
         *   Without alpha: column stays frozen at head for all 3 frames,
         *   then jumps 1 row — visible stutter.
         *
         * alpha=0.0 → draw at exact ticked position (no change from before)
         * alpha=0.9 → draw 90% of a tick ahead of ticked position
         */
        float alpha = (float)sim_accum / (float)tick_ns;

        /* ── FPS counter ─────────────────────────────────────────── */
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

        /* ── draw + present ──────────────────────────────────────── */
        screen_draw_rain(&app->screen, &app->rain, alpha);
        screen_draw_hud(&app->screen,
                        fps_display, app->sim_fps,
                        app->density, app->theme_idx);
        screen_present();

        /* ── input ───────────────────────────────────────────────── */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    rain_free(&app->rain);
    screen_free(&app->screen);
    return 0;
}
