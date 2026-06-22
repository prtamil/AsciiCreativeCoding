/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * matrix_rain.c — the falling green code from The Matrix, in your terminal.
 * Each column has its own stream: a bright white head falling downward with
 * a fading trail of random letters behind it, all reshuffling as they fall.
 *
 * Sister demos with the same trick: fireworks_rain.c (arc trails),
 * pulsar_rain.c (radial pulses), sun_rain.c (sun shape).
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config — every tunable number, grouped by what it controls ── */

enum {
    TARGET_FPS       = 60,            /* frames per second we aim for    */

    /* How often the trail letters reshuffle. Lower = chunkier shimmer.  */
    SHIMMER_HZ       = 20,
};

/* Global speed dial the [ and ] keys turn. 1.0 = normal; clamped below. */
#define SPEED_SCALE_DEFAULT  1.0f
#define SPEED_SCALE_MIN      0.25f
#define SPEED_SCALE_MAX      4.0f
#define SPEED_SCALE_STEP     1.25f      /* step by a factor so each press feels the same */

enum {
    TRAIL_MIN        =  6,            /* shortest a stream can be (rows) */
    TRAIL_MAX        = 24,            /* longest — also sizes glyphs[]   */
};

/* Each stream falls at its own speed, picked in this range, so the
 * field doesn't drop in lock-step. Rows per second. */
#define SPEED_MIN_RPS   8.0f
#define SPEED_MAX_RPS  24.0f

/* How crowded the field is: only every density-th column starts lit. */
enum {
    DENSITY_MIN      =  1,    /* every column                           */
    DENSITY_DEFAULT  =  2,    /* every other column                     */
    DENSITY_MAX      =  6,    /* roughly 1 in 6                         */
};

/* Roughly how often a dead column springs back to life, per second.
 * We divide by density so a sparse field also wakes up more slowly.
 * 0.6 means about a 1.7 s wait at density 1, ~10 s at density 6. */
#define RESPAWN_RATE_PER_SEC  0.6f

/* New streams start somewhere above the top edge (up to half a screen
 * up) so they enter staggered instead of all at once. */
#define SPAWN_OFFSCREEN_FRAC  0.5f

/* If a frame takes longer than this, pretend it didn't — stops a slow
 * frame from snowballing into a death spiral. */
#define DT_CAP_SEC  0.10f

/* Colour-pair slots. 1..5 are the trail's brightness bands (the theme
 * sets their colours), 6 is the always-white head, 7..8 are the HUD. */
enum {
    SHADE_FADE      = 1,
    SHADE_DARK,
    SHADE_MID,
    SHADE_BRIGHT,
    SHADE_HOT,

    SHADE_HEAD,

    PAIR_HUD,                         /* bright yellow                   */
    PAIR_HINT,                        /* bright cyan                     */
};

#define NS_PER_SEC    1000000000LL
#define NS_PER_MS     1000000LL

#define HUD_BUF_LEN  64

/* ── §2 clock — monotonic time + sleep ── */

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

/* ── §3 color — HUD pairs + four themed trail palettes ── */

/*
 * Theme — one named colour scheme for the trail, dim to bright.
 *
 * A trail is drawn in five brightness steps, from FADE (the dimmest,
 * oldest letters) up to HOT (the brightest, just behind the head). The
 * head itself is always white and ignores the theme. §4 col_band picks
 * which step each letter gets; this struct just holds the colours.
 *
 * Pressing 't' cycles the presets and theme_apply() rebinds the colour
 * pairs in one shot. The pair numbers stay fixed, so streams already on
 * screen change colour instantly without respawning.
 *
 * We keep two colour lists because terminals differ: fg[] is the nice
 * 256-colour ramp; fg_8[] is the closest match on old 8-colour
 * terminals, where the five steps collapse into one or two colours.
 *
 * The presets: green (classic Matrix), amber (sodium-lamp orange),
 * blue (cyber-noir), white (black-and-white film grain).
 *
 * Members
 *   name    label shown in the HUD ("green", "amber", …); never NULL.
 *   fg[]    five 256-colour values, dimmest first (FADE) to HOT.
 *   fg_8[]  five 8-colour fallbacks, same order.
 *
 * Every colour stays at index 24 or higher: the very dark end of the
 * palette renders as black on a black terminal and would vanish (see
 * CLAUDE.md "Theme Palette Brightness"). Each entry is at least as
 * bright as the one before it.
 */
typedef struct {
    const char *name;
    int         fg[5];                /* dim FADE .. bright HOT (256-colour) */
    int         fg_8[5];              /* 8-colour fallback                  */
} Theme;

static const Theme k_themes[] = {
    { "green",
      {  28,  34,  40,  46,  82 },
      { COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN  } },
    { "amber",
      {  94, 130, 172, 214, 220 },
      { COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW } },
    { "blue",
      {  24,  33,  39,  45,  51 },
      { COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN,   COLOR_CYAN   } },
    { "white",
      { 240, 244, 248, 252, 255 },
      { COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE  } },
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static bool g_has_256 = false;

/* Recolour the trail bands for the chosen theme. Leaves the HUD pairs
 * alone on purpose — they shouldn't change colour when you switch themes. */
static void theme_apply(int theme_idx)
{
    const int *fg = g_has_256
                    ? k_themes[theme_idx].fg
                    : k_themes[theme_idx].fg_8;

    init_pair(SHADE_FADE,   fg[0],       COLOR_BLACK);
    init_pair(SHADE_DARK,   fg[1],       COLOR_BLACK);
    init_pair(SHADE_MID,    fg[2],       COLOR_BLACK);
    init_pair(SHADE_BRIGHT, fg[3],       COLOR_BLACK);
    init_pair(SHADE_HOT,    fg[4],       COLOR_BLACK);
    init_pair(SHADE_HEAD,   COLOR_WHITE, COLOR_BLACK);
}

/* Set up the HUD colours once. Background -1 means "whatever the user's
 * terminal uses", so the HUD blends in instead of sitting in a black box. */
static void hud_pairs_init(void)
{
    init_pair(PAIR_HUD,  g_has_256 ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, g_has_256 ?  51 : COLOR_CYAN,   -1);
}

/* ── §4 column — one falling stream: spawn, advance, shimmer, draw ── */

/* The pool of characters a stream can show. */
static const char k_glyphs[] =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789"
    "!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

static char  rand_glyph(void)  { return k_glyphs[rand() % GLYPHS_LEN]; }
static float urand01(void)     { return (float)rand() / (float)RAND_MAX; }

/*
 * Column — one falling stream in a single terminal column.
 *
 * This is the basic unit of the effect. A stream is a bright head plus a
 * trail of letters reaching up from it; head_y is the head's row and the
 * trail climbs upward from there, fading as it goes. Each frame a stream
 * moves down at a steady speed, occasionally reshuffles its letters, and
 * once it has fully fallen off the bottom it goes inactive and the slot
 * can be reused.
 *
 * Each stream remembers its own letters rather than picking new ones every
 * frame to draw. If we rerolled on every draw the screen would be a blur
 * of static; remembering them lets each letter linger for a moment, so the
 * shuffling reads as texture, not noise. This is the trademark Matrix-rain
 * trick, also used by fireworks_rain.c for its sparks.
 *
 * Members
 *   col          which terminal column this stream lives in; set once at
 *                spawn and equal to the stream's index in the pool.
 *   head_y       the head's row, kept as a float so the stream can glide
 *                smoothly between rows; rounded to a whole row only when
 *                drawn.
 *   speed        rows per second this stream falls; fixed for its life,
 *                randomised at spawn so lanes fall at different rates.
 *   trail_len    how many letters long the stream is, head included;
 *                picked at spawn between TRAIL_MIN and TRAIL_MAX.
 *   glyphs[]     the stream's letters, one per row. [0] is the head, [1..]
 *                the tail. Sized for the longest possible stream so the
 *                whole pool fits in one block with no per-stream malloc.
 *   active       true while any part is still on screen; false once it has
 *                fallen off the bottom, which frees the slot for respawn.
 */
typedef struct {
    int    col;
    float  head_y;
    float  speed;
    int    trail_len;
    char   glyphs[TRAIL_MAX];
    bool   active;
} Column;

/* Start a fresh stream above the top edge with random length, speed,
 * and letters, so each one looks different and enters at its own time. */
static void col_spawn(Column *c, int x, int rows)
{
    c->col       = x;
    c->head_y    = -urand01() * SPAWN_OFFSCREEN_FRAC * (float)rows;
    c->trail_len = TRAIL_MIN + rand() % (TRAIL_MAX - TRAIL_MIN + 1);
    c->speed     = SPEED_MIN_RPS + urand01() * (SPEED_MAX_RPS - SPEED_MIN_RPS);
    c->active    = true;

    for (int i = 0; i < c->trail_len; i++)
        c->glyphs[i] = rand_glyph();
}

/* Move the head down by one frame's worth. Returns false once even the
 * tail has cleared the bottom, telling the caller to retire this stream. */
static bool col_advance(Column *c, float dt, int rows)
{
    c->head_y += c->speed * dt;
    return (c->head_y - (float)c->trail_len) < (float)rows;
}

/* Pick fresh letters for the whole stream. Called only a few times a
 * second, not every frame, so the shuffle reads as a blink, not a blur. */
static void col_shimmer(Column *c)
{
    for (int i = 0; i < c->trail_len; i++)
        c->glyphs[i] = rand_glyph();
}

/* Given how far a letter sits from the head, pick its colour and weight.
 * The head is white, the few cells behind it bright, and it fades to dim
 * toward the tail — the classic Matrix gradient. Ready to hand to attron. */
static attr_t col_band(int dist, int trail_len)
{
    if (dist == 0)               return COLOR_PAIR(SHADE_HEAD)   | A_BOLD;
    if (dist == 1)               return COLOR_PAIR(SHADE_HOT)    | A_BOLD;
    if (dist == 2)               return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
    if (dist <= trail_len / 2)   return COLOR_PAIR(SHADE_MID);
    if (dist <= trail_len - 2)   return COLOR_PAIR(SHADE_DARK);
    return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/*
 * Paint the stream, head first then up the tail, each letter in its band.
 *
 * We round head_y to a row with floor(... + 0.5) rather than roundf because
 * roundf rounds halves to even, which makes a letter flicker between two
 * rows when it lands exactly on a half. This way a half always rounds the
 * same direction, so the fall looks steady.
 */
static void col_draw(const Column *c, int rows)
{
    for (int dist = 0; dist < c->trail_len; dist++) {
        int row = (int)floorf(c->head_y - (float)dist + 0.5f);
        if (row < 0 || row >= rows) continue;

        attr_t attr = col_band(dist, c->trail_len);
        attron(attr);
        mvaddch(row, c->col, (chtype)(unsigned char)c->glyphs[dist]);
        attroff(attr);
    }
}

/* ── §5 scene — the whole field: pool of streams, controls, theme ── */

/*
 * ColumnPool — one stream slot for every terminal column.
 *
 * There's exactly one slot per column because a column never shows more
 * than one stream at a time — that's the look of Matrix rain, streams in
 * their own lanes. Making the array index equal the column number means no
 * lookups. calloc starts every slot inactive; col_spawn fills them in,
 * some at startup and the rest as they wake up over time.
 *
 * This array is the only heap allocation in the program. It's sized from
 * the terminal width at startup, freed by scene_free, and reallocated on
 * each resize. Everything else lives in static storage.
 *
 * Members
 *   columns   the array of stream slots; columns[i] lives in column i.
 *   ncols     terminal width — how many lanes.
 *   nrows     terminal height — used to tell when a stream has fallen off.
 */
typedef struct {
    Column *columns;
    int     ncols;
    int     nrows;
} ColumnPool;

/*
 * SimControls — the live knobs the keyboard turns, plus the shimmer timer.
 *
 * Everything the user can change at runtime lives here, alongside the small
 * timer that decides when the letters reshuffle (it rides along on the same
 * per-frame clock, so it belongs with the rest).
 *
 * Members
 *   paused          when true the field freezes (HUD shows "PAUSED");
 *                   toggled by space.
 *   speed_scale     the global speed dial, kept between SPEED_SCALE_MIN and
 *                   _MAX; ] speeds up, [ slows down.
 *   density         how spread out the streams are, kept between
 *                   DENSITY_MIN and _MAX; +/- adjust it.
 *   shimmer_accum   seconds since the last reshuffle. When it reaches
 *                   1/SHIMMER_HZ every stream reshuffles and it resets, so
 *                   the shimmer runs at a steady pace whatever the frame rate.
 */
typedef struct {
    bool  paused;
    float speed_scale;
    int   density;
    float shimmer_accum;
} SimControls;

/*
 * Scene — all the simulation state for one run, reachable from one pointer.
 *
 * Members
 *   pool        the streams and the field size.
 *   sim         the live knobs and shimmer timer.
 *   theme_idx   which colour theme is showing; the 't' key cycles it.
 */
typedef struct {
    ColumnPool  pool;
    SimControls sim;
    int         theme_idx;
} Scene;

/* Light up some columns right away so the screen isn't blank while the
 * rest wake up on their own over the next few seconds. */
static void seed_initial_streams(Scene *s) {
    for (int x = 0; x < s->pool.ncols; x++) {
        if (x % s->sim.density == 0)
            col_spawn(&s->pool.columns[x], x, s->pool.nrows);
    }
}

static void scene_init(Scene *s, int cols, int rows, int density)
{
    s->pool.columns = calloc((size_t)cols, sizeof(Column));
    s->pool.ncols   = cols;
    s->pool.nrows   = rows;

    s->sim.paused        = false;
    s->sim.speed_scale   = SPEED_SCALE_DEFAULT;
    s->sim.density       = density;
    s->sim.shimmer_accum = 0.0f;

    seed_initial_streams(s);
}

static void scene_free(Scene *s)
{
    free(s->pool.columns);
    s->pool.columns = NULL;
    s->pool.ncols   = 0;
    s->pool.nrows   = 0;
}

/* Returns true on the frames when it's time to reshuffle every stream's
 * letters, then resets its little timer. */
static bool tick_shimmer_pulse(SimControls *sim, float dt) {
    sim->shimmer_accum += dt;
    bool fired = (sim->shimmer_accum >= 1.0f / (float)SHIMMER_HZ);
    if (fired) sim->shimmer_accum = 0.0f;
    return fired;
}

/* The chance a given dead column comes back this frame. We start from a
 * target wakeups-per-second, slow it for sparser fields by dividing by
 * density, then scale by the frame's length to turn a rate into odds. */
static inline float respawn_probability_per_frame(int density, float dt) {
    return (RESPAWN_RATE_PER_SEC * dt) / (float)density;
}

/* Update one column: a live stream falls (and maybe reshuffles, or dies if
 * it fell off); a dead one gets a roll of the dice to spring back to life. */
static inline void tick_one_column(Column *c, int col_x, int nrows,
                                    float scaled_dt, float respawn_p,
                                    bool shimmer_now) {
    if (c->active) {
        if (!col_advance(c, scaled_dt, nrows))
            c->active = false;
        else if (shimmer_now)
            col_shimmer(c);
    } else {
        if (urand01() < respawn_p)
            col_spawn(c, col_x, nrows);
    }
}

/* Advance the whole field one frame: work out this frame's reshuffle flag
 * and respawn odds once, then update every column. */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;

    bool  shimmer_now = tick_shimmer_pulse(&s->sim, dt);
    float scaled_dt   = dt * s->sim.speed_scale;
    float respawn_p   = respawn_probability_per_frame(s->sim.density, dt);

    for (int x = 0; x < s->pool.ncols; x++) {
        tick_one_column(&s->pool.columns[x], x, s->pool.nrows,
                        scaled_dt, respawn_p, shimmer_now);
    }
}

static void scene_draw(const Scene *s)
{
    for (int x = 0; x < s->pool.ncols; x++) {
        const Column *c = &s->pool.columns[x];
        if (c->active) col_draw(c, s->pool.nrows);
    }
}

/* The ]/[ keys turn the speed dial; kept inside the safe range. */
static void scene_scale_speed(Scene *s, float factor) {
    s->sim.speed_scale *= factor;
    if (s->sim.speed_scale < SPEED_SCALE_MIN) s->sim.speed_scale = SPEED_SCALE_MIN;
    if (s->sim.speed_scale > SPEED_SCALE_MAX) s->sim.speed_scale = SPEED_SCALE_MAX;
}

/* The +/- keys crowd or thin the field. A smaller density means more
 * streams, so + passes a negative delta. Kept inside the safe range. */
static void scene_change_density(Scene *s, int delta) {
    int next = s->sim.density + delta;
    if (next < DENSITY_MIN) next = DENSITY_MIN;
    if (next > DENSITY_MAX) next = DENSITY_MAX;
    s->sim.density = next;
}

/* The 't' key moves to the next colour theme and recolours the field. */
static void scene_cycle_theme(Scene *s) {
    s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
    theme_apply(s->theme_idx);
}

/*
 * Throw away the field and build a fresh one, but keep the settings the
 * user chose: speed, density, and theme. Used by both the 'r' reset and a
 * window resize, so they can't drift apart on which settings survive. The
 * shimmer timer is deliberately reset so the reshuffle starts clean.
 */
static void scene_reset_preserving_knobs(Scene *s, int new_cols, int new_rows) {
    float saved_speed = s->sim.speed_scale;
    int   saved_dens  = s->sim.density;
    int   saved_theme = s->theme_idx;

    scene_free(s);
    scene_init(s, new_cols, new_rows, saved_dens);

    s->sim.speed_scale = saved_speed;
    s->theme_idx       = saved_theme;
}

/* ── §6 screen — ncurses setup, HUD, present ── */

/*
 * Screen — the terminal's size in cells, and the home of the ncurses
 * setup/teardown calls.
 *
 * The scene keeps its own copy of these dimensions so the simulation can
 * check whether a stream fell off the bottom without reaching into the
 * render side. They're refreshed from the terminal on startup and on every
 * resize.
 *
 * Members
 *   cols   terminal width in cells; row 0 holds the status line.
 *   rows   terminal height in cells; the bottom row holds the key hint.
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
    typeahead(-1);              /* stop ncurses checking input mid-draw, which tears the frame */
    start_color();
    use_default_colors();       /* needed so -1 means the terminal's own background */
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* Print a line of HUD text in the given colour, bold. Shared by both rows. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, col, "%s", text);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Build the top status line into buf: fps, speed, density, theme, paused. */
static void format_hud_status(const Scene *scene, double fps,
                              char *buf, size_t buflen) {
    snprintf(buf, buflen,
             " %5.1f fps  spd:%.2fx  den:%d  [%s] %s ",
             fps, scene->sim.speed_scale, scene->sim.density,
             k_themes[scene->theme_idx].name,
             scene->sim.paused ? "PAUSED " : "running");
}

/* Draw the status line, right-aligned along the top row. */
static void draw_hud_status(const Screen *s, const Scene *scene, double fps) {
    enum { HUD_TOP_ROW = 0 };
    char buf[HUD_BUF_LEN];
    format_hud_status(scene, fps, buf, sizeof buf);
    int right_col = s->cols - (int)strlen(buf);
    if (right_col < 0) right_col = 0;
    hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* Draw the key reminder along the bottom row. */
static void draw_hud_hint(const Screen *s) {
    static const char *KEY_HINT =
        " q:quit  spc:pause  r:reset  []:speed  +/-:density  t:theme ";
    hud_paint_text(s->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw_hud(const Screen *s, double fps, const Scene *scene)
{
    draw_hud_status(s, scene, fps);
    draw_hud_hint  (s);
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §7 app — signals, resize, main loop ── */

/*
 * FpsCounter — a smoothed frames-per-second readout for the HUD.
 *
 * A raw per-frame number jumps around too much to read, so we tally frames
 * over a short window (half a second) and only update the shown figure when
 * the window fills.
 *
 * Members
 *   frame_count   frames counted so far this window.
 *   window_ns     time elapsed so far this window, in nanoseconds.
 *   display       the smoothed fps figure the HUD prints.
 */
typedef struct {
    int     frame_count;
    int64_t window_ns;
    double  display;
} FpsCounter;

static void fps_counter_init(FpsCounter *f) {
    f->frame_count = 0;
    f->window_ns   = 0;
    f->display     = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
    const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2;       /* half a second */
    f->frame_count++;
    f->window_ns += dt_ns;
    if (f->window_ns < FPS_WINDOW_NS) return;
    f->display     = (double)f->frame_count
                   * (double)NS_PER_SEC / (double)f->window_ns;
    f->frame_count = 0;
    f->window_ns   = 0;
}

/*
 * App — the top-level bundle holding everything that lives across frames.
 *
 * Members
 *   scene         the simulation.
 *   screen        terminal size and ncurses setup.
 *   fps           the smoothed fps readout.
 *   running       cleared on quit or an interrupt signal; the loop watches it.
 *   need_resize   set when the window resizes; handled next time around.
 *
 * The single global g_app exists only so the signal handlers can flip those
 * two flags; everything else is passed around by pointer.
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    FpsCounter            fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal(int sig)   { (void)sig; g_app.running = 0;     }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Rebuild the field at the new window size, keeping the user's speed,
 * density, and theme. The streams that were falling are lost — a one-frame
 * blip, no real harm. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_reset_preserving_knobs(
        &app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

/* Act on one keypress. Returns false only when the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;

    case ' ':           s->sim.paused = !s->sim.paused;                 break;

    case 'r': case 'R':
        scene_reset_preserving_knobs(s, app->screen.cols, app->screen.rows);
        break;

    case ']':           scene_scale_speed(s, SPEED_SCALE_STEP);          break;
    case '[':           scene_scale_speed(s, 1.0f / SPEED_SCALE_STEP);   break;

    case '=': case '+': scene_change_density(s, -1); break;   /* fewer gaps = more streams */
    case '-':           scene_change_density(s, +1); break;   /* bigger gaps = fewer streams */

    case 't': case 'T': scene_cycle_theme(s);        break;

    default: break;
    }
    return true;
}

/*
 * The whole program: set up, then loop measure-time / read-keys / advance /
 * draw / sleep until asked to quit. There's no fixed-step physics here — the
 * rain is just a visual, so one step per frame with the real elapsed time is
 * fine.
 */
int main(void)
{
    srand((unsigned int)clock_ns());
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App   *app   = &g_app;
    Scene *scene = &app->scene;
    app->running = 1;
    fps_counter_init(&app->fps);

    screen_init(&app->screen);
    g_has_256 = (COLORS >= 256);
    scene->theme_idx = 0;
    theme_apply(scene->theme_idx);
    hud_pairs_init();
    scene_init(scene, app->screen.cols, app->screen.rows, DENSITY_DEFAULT);

    const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
    int64_t last_ns = clock_ns();

    while (app->running) {

        /* Handle a pending resize first, so the rest of the frame sees the new size. */
        if (app->need_resize) {
            app_do_resize(app);
            last_ns = clock_ns();
        }

        /* Time since the last frame, capped so a slow frame can't snowball. */
        int64_t now_ns  = clock_ns();
        int64_t dt_ns   = now_ns - last_ns;
        last_ns         = now_ns;
        float   dt      = (float)dt_ns / (float)NS_PER_SEC;
        if (dt > DT_CAP_SEC) dt = DT_CAP_SEC;

        for (int ch; (ch = getch()) != ERR; ) {
            if (!app_handle_key(app, ch)) {
                app->running = 0;
                break;
            }
        }

        scene_tick(scene, dt);

        fps_counter_tick(&app->fps, dt_ns);

        erase();
        scene_draw(scene);
        screen_draw_hud(&app->screen, app->fps.display, scene);
        screen_present();

        /* Sleep off the rest of this frame's budget so we hold a steady rate. */
        int64_t elapsed = clock_ns() - now_ns;
        clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
    }

    scene_free(scene);
    screen_free(&app->screen);
    return 0;
}
