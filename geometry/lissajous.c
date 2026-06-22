/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * lissajous.c — a "harmonograph": two swings, one left-right and one
 * up-down, fading out as they go.  Their combined trail draws figure-8s,
 * stars, petals and spirals that slowly morph as one swing's timing drifts.
 *
 * Curve names and the closed-curve idea: Lawrence, "A Catalog of Special
 * Plane Curves" (1972).  Named after Lissajous (1857), but Bowditch drew
 * the same family ~40 years earlier, so older texts call them Bowditch
 * curves.  The fading instrument itself: Whitaker, Am. J. Phys. 69 (2001).
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config ── */

#define TICK_NS       33333333LL   /* ~30 fps                              */
#define N_CURVE_PTS   2500         /* points sampled along the curve        */
#define N_LOOPS       4            /* how many loops the spiral shows       */
#define DECAY_TOTAL   4.5f         /* fade strength: ~1% brightness at end  */
#define CELL_ASPECT   2.0f         /* a cell is this much taller than wide   */
#define AMP_FRAC      0.92f        /* leave a little breathing room at edge  */

/* One color slot per (theme, brightness), plus two for the HUD rows. */
#define N_LEVELS      4            /* one brightness step per spiral loop   */
#define N_THEMES      4
#define CP_IDX(t,l)   ((t)*N_LEVELS + (l) + 1)
#define CP_HUD        (N_THEMES * N_LEVELS + 1)   /* top info row          */
#define CP_HINT       (N_THEMES * N_LEVELS + 2)   /* bottom keys row       */

/* Reserve one row top and bottom for the HUD; the curve uses the middle. */
#define HUD_TOP_ROWS     1
#define HUD_BOTTOM_ROWS  1

#define TAU              (2.0f * (float)M_PI)   /* one full turn, radians */

/* Once a point has faded below 1% brightness, stop drawing it. */
#define AMP_VISIBLE_MIN  0.01f
/* The two brightest loops get bold; the two dim ones don't. */
#define BOLD_LEVELS      2
/* Keep one blank column on each side so the curve never hugs the edge. */
#define EDGE_MARGIN_X    2

/* How fast the figure morphs, in radians of phase per tick. */
#define DRIFT_DEFAULT 0.004f       /* full sweep takes about 35 seconds    */
#define DRIFT_MIN     0.0005f
#define DRIFT_MAX     0.08f
#define DRIFT_STEP    1.6f

/* The morph slows to a near-stop on each "named" shape so you can see it,
 * then speeds back up.  WIDTH is how wide that slow zone is, SPEED is how
 * slow it gets at the bottom (see dwell_envelope in §4). */
#define DWELL_WIDTH   0.25f
#define DWELL_SPEED   0.25f

#define N_RATIOS      8

/* ── §2  clock ── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3  color ── */

/*
 * Theme -- one named color scheme for the spiral.
 *
 * The spiral has four loops, drawn from outer to inner.  We use two
 * separate cues so both are easy to read at once: the theme picks the
 * hue family, and brightness marks the loop's age.  The outer loop is
 * the newest and brightest; the inner loop is the oldest and dimmest.
 * Every theme keeps that same bright-to-dim shading, so switching themes
 * never changes which loop looks "fresh".
 *
 *   name      label shown in the HUD ("Golden", "Ice", ...)
 *   colors    four 256-color codes, brightest (outer loop) first;
 *             one per loop, so there are exactly N_LEVELS of them
 */
typedef struct {
    const char *name;
    int         colors[N_LEVELS];
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Golden: bright-yellow → gold → dark-orange → brown    */
    { "Golden", { 226, 220, 136,  94 } },
    /* Ice:    bright-cyan  → teal → dark-teal  → navy       */
    { "Ice",    {  51,  38,  23,  17 } },
    /* Ember:  white → orange → red → dark-red               */
    { "Ember",  { 231, 208, 196,  88 } },
    /* Neon:   bright-green → green → dark-green → very-dark */
    { "Neon",   { 118,  82,  28,  22 } },
};

/* The character drawn for each loop, brightest first.  Same four shapes
 * in every theme, so the glyph alone tells you how fresh a loop is. */
static const char k_lev_ch[N_LEVELS] = { '#', '*', '+', '.' };

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        for (int t = 0; t < N_THEMES; t++)
            for (int l = 0; l < N_LEVELS; l++)
                init_pair(CP_IDX(t, l), k_themes[t].colors[l], -1);
        init_pair(CP_HUD,  252, -1);   /* bright gray for data         */
        init_pair(CP_HINT,  51, -1);   /* bright cyan for action keys  */
    } else {
        for (int i = 1; i <= N_THEMES * N_LEVELS; i++)
            init_pair(i, COLOR_YELLOW, -1);
        init_pair(CP_HUD,  COLOR_WHITE, -1);
        init_pair(CP_HINT, COLOR_CYAN,  -1);
    }
}

/* ── §4  sim ── */

/*
 * Ratio -- how fast the two swings go, and what shape that makes.
 *
 * The whole trick is that the speeds are whole numbers.  Whole-number
 * speeds make the trail join back up into a closed shape; any other
 * speed ratio would never close and would just smear over the whole box.
 * The bigger the numbers, the more lobes the figure has.
 *
 *   fx, fy   the two swing speeds, as whole numbers (X swing, Y swing)
 *   name     the shape this ratio makes ("Trefoil", "Pentagram", ...)
 *
 * The pair is stored small-first only by habit; swapping fx and fy just
 * turns the same shape on its side.
 */
typedef struct {
    int         fx, fy;
    const char *name;
} Ratio;

static const Ratio k_ratios[N_RATIOS] = {
    { 1, 2, "1:2 Figure-8"   },
    { 2, 3, "2:3 Trefoil"    },
    { 3, 4, "3:4 Star"       },
    { 1, 3, "1:3 Clover"     },
    { 3, 5, "3:5 Pentagram"  },
    { 2, 5, "2:5 Five-petal" },
    { 4, 5, "4:5 Crown"      },
    { 1, 4, "1:4 Eye"        },
};

/*
 * Scene -- everything we need to remember to draw the next frame.
 *
 * We keep the chosen ratio and theme as small index numbers rather than
 * pointers, so the n/p and c keys just bump a counter and wrap around.
 *
 *   ratio_idx   which shape is showing (0..N_RATIOS-1); n/p keys, and
 *               the tick when a sweep finishes
 *   theme_idx   which color scheme (0..N_THEMES-1); the c key
 *   phase_x     how far into the morph we are, 0..2π radians; the timing
 *               offset between the two swings.  Creeps up each tick; when
 *               it laps, we move on to the next shape
 *   drift       base morph speed per tick, between DRIFT_MIN and MAX;
 *               +/- keys
 *   paused      if true, the figure holds still but you can still recolor
 *               or switch shapes; SPACE toggles it
 */
typedef struct {
    int   ratio_idx;
    int   theme_idx;
    float phase_x;
    float drift;
    bool  paused;
} Scene;

/* Turn the stored index back into the actual ratio / theme it points at,
 * so the rest of the code can just ask for "the current one". */
static inline const Ratio *scene_current_ratio(const Scene *s)
{
    return &k_ratios[s->ratio_idx];
}

static inline const Theme *scene_current_theme(const Scene *s)
{
    return &k_themes[s->theme_idx];
}

static void scene_init(Scene *s)
{
    s->ratio_idx = 0;
    s->theme_idx = 0;
    s->phase_x   = 0.0f;
    s->drift     = DRIFT_DEFAULT;
    s->paused    = false;
}

/* ---- how fast the figure morphs ---- */

/* The clean, symmetric "named" shapes come around at evenly spaced phases.
 * This returns that spacing, so we know where the shapes we want to pause on
 * live. */
static float ratio_key_period(const Ratio *r)
{
    return (float)M_PI / fmaxf((float)r->fx, (float)r->fy);
}

/* A speed multiplier that dips when we're near one of those named shapes,
 * so the figure lingers on it instead of flickering past.  Sitting right on
 * a shape gives DWELL_SPEED (slow); far enough away gives 1.0 (full speed),
 * with a straight ramp between. */
static float dwell_envelope(float phase, float key_period)
{
    float phase_mod    = fmodf(phase, key_period);
    float dist_to_key  = fminf(phase_mod, key_period - phase_mod);
    float dwell_frac   = dist_to_key / (key_period * DWELL_WIDTH);
    float ramp         = fminf(dwell_frac, 1.0f);
    return DWELL_SPEED + (1.0f - DWELL_SPEED) * ramp;
}

/* How far the morph actually advances this tick: the base speed, slowed
 * down whenever we're near a named shape. */
static float eff_drift(const Scene *s)
{
    float key_period = ratio_key_period(scene_current_ratio(s));
    float envelope   = dwell_envelope(s->phase_x, key_period);
    return s->drift * envelope;
}

static inline bool phase_completed_cycle(float phase) { return phase >= TAU; }
static inline int  ratio_idx_next(int cur) { return (cur + 1) % N_RATIOS; }

static void scene_tick(Scene *s)
{
    if (s->paused) return;
    s->phase_x += eff_drift(s);
    if (phase_completed_cycle(s->phase_x)) {
        s->phase_x  -= TAU;
        s->ratio_idx = ratio_idx_next(s->ratio_idx);
    }
}

/* ── §5  scene ── */

/*
 * CurveFrame -- the size and placement of this frame's figure.
 *
 * We work this out fresh every frame rather than storing it, because it
 * depends on the terminal size and the user can resize at any moment;
 * recomputing a handful of numbers is far cheaper than risking stale ones.
 *
 *   t_max    how far we trace along the curve, picked so you always see
 *            N_LOOPS loops no matter which shape is up
 *   decay    how quickly the trail fades; tuned so the trail is down to
 *            ~1% by the end, giving every shape the same spiral depth
 *   cx, cy   where the center of the figure sits on screen
 *   rx, ry   half-width and half-height in cells.  These differ because a
 *            terminal cell is about twice as tall as it is wide, so a shape
 *            that should look round needs a wider rx to compensate
 */
typedef struct {
    float t_max;
    float decay;
    float cx, cy;
    float rx, ry;
} CurveFrame;

/* Trace length = N_LOOPS loops of the slower swing, so the loop count you
 * see stays the same whichever shape is up. */
static float curve_t_max(const Ratio *r)
{
    return (float)N_LOOPS * TAU / fminf((float)r->fx, (float)r->fy);
}

/* Fade rate set so the trail is down to about 1% brightness by the end. */
static float curve_decay_rate(float t_max)
{
    return DECAY_TOTAL / t_max;
}

/* Center the figure: middle of the screen across, middle of the band left
 * over once the top and bottom HUD rows are taken out. */
static void figure_center(int rows, int cols, float *cx, float *cy)
{
    int draw_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    *cx = cols * 0.5f;
    *cy = HUD_TOP_ROWS + draw_rows * 0.5f;
}

/* Biggest figure that still fits, with cells being taller than wide taken
 * into account and a little margin left around the edges. */
static void fit_radii(int rows, int cols, float *rx, float *ry)
{
    int   draw_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    float max_rx   = (cols - EDGE_MARGIN_X) * 0.5f;
    float max_ry   = (draw_rows - 1)        * 0.5f;
    *ry = fminf(max_rx / CELL_ASPECT, max_ry) * AMP_FRAC;
    *rx = *ry * CELL_ASPECT;
}

static CurveFrame curve_frame_make(const Scene *s, int rows, int cols)
{
    const Ratio *r = scene_current_ratio(s);
    CurveFrame cf;
    cf.t_max = curve_t_max(r);
    cf.decay = curve_decay_rate(cf.t_max);
    figure_center(rows, cols, &cf.cx, &cf.cy);
    fit_radii    (rows, cols, &cf.rx, &cf.ry);
    return cf;
}

/* ---- one point at a time ---- */

/* Where sample i sits along the curve in time.  The first sample is the
 * newest (outer end), the last is the oldest (collapsed near the center). */
static inline float sample_time(int i, float t_max)
{
    return (float)i / (float)(N_CURVE_PTS - 1) * t_max;
}

/* How bright a point is at time t -- it fades out the older it gets. */
static inline float envelope_amp(float t, float decay)
{
    return expf(-decay * t);
}

/* One point on the curve, in a tidy -1..1 box.  The two swings combine here;
 * we fold in the brightness too so older points pull in toward the center. */
static inline void lissajous_point(float t, float fx, float fy,
                                   float phase_x, float amp,
                                   float *nx, float *ny)
{
    *nx = amp * sinf(fx * t + phase_x);
    *ny = amp * sinf(fy * t);
}

static inline int cell_round(float v) { return (int)(v + 0.5f); }

/* Place a -1..1 point onto an actual screen cell.  Says false when the
 * point would land on a HUD row or off the screen, so the caller skips it. */
static bool project_to_cell(const CurveFrame *cf, float nx, float ny,
                            int rows, int cols, int *out_col, int *out_row)
{
    int col = cell_round(cf->cx + nx * cf->rx);
    int row = cell_round(cf->cy + ny * cf->ry);
    if (row < HUD_TOP_ROWS || row >= rows - HUD_BOTTOM_ROWS) return false;
    if (col < 0 || col >= cols) return false;
    *out_col = col;
    *out_row = row;
    return true;
}

/* Turn a point's age (0 = newest, 1 = oldest) into which of the four
 * brightness loops it belongs to. */
static inline int age_to_level(float age)
{
    int lev = (int)(age * (float)N_LEVELS);
    if (lev >= N_LEVELS) lev = N_LEVELS - 1;
    return lev;
}

/* Color and weight for a given theme and brightness loop; the bright loops
 * also get bold. */
static attr_t level_attr(int theme_idx, int level)
{
    attr_t attr = COLOR_PAIR(CP_IDX(theme_idx, level));
    if (level < BOLD_LEVELS) attr |= A_BOLD;
    return attr;
}

static void plot_sample(int row, int col, int theme_idx, int level)
{
    char   ch   = k_lev_ch[level];
    attr_t attr = level_attr(theme_idx, level);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(attr);
}

/* Draw the whole spiral.  We walk from the oldest point to the newest so
 * that where loops overlap, the brighter newer point is the one left on top. */
static void scene_draw(const Scene *s, int rows, int cols)
{
    CurveFrame   cf = curve_frame_make(s, rows, cols);
    const Ratio *r  = scene_current_ratio(s);
    float fx = (float)r->fx, fy = (float)r->fy;
    float inv_n_minus_1 = 1.0f / (float)(N_CURVE_PTS - 1);

    for (int i = N_CURVE_PTS - 1; i >= 0; i--) {
        float t   = sample_time  (i, cf.t_max);
        float amp = envelope_amp (t, cf.decay);
        if (amp < AMP_VISIBLE_MIN) continue;

        float nx, ny;
        lissajous_point(t, fx, fy, s->phase_x, amp, &nx, &ny);

        int row, col;
        if (!project_to_cell(&cf, nx, ny, rows, cols, &col, &row)) continue;

        float age   = (float)i * inv_n_minus_1;
        int   level = age_to_level(age);
        plot_sample(row, col, s->theme_idx, level);
    }
}

/* Top row: what's currently showing -- shape, how far into the morph,
 * morph speed, theme, and whether we're paused. */
static void scene_hud_top(const Scene *s, int cols)
{
    (void)cols;
    float phase_pi = s->phase_x / (float)M_PI;   /* show phase in units of π */
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, 0,
             " [Lissajous]  ratio: %-14s  phase: %.2fπ  drift: %.4f  theme: %-6s  %s ",
             scene_current_ratio(s)->name,
             phase_pi,
             s->drift,
             scene_current_theme(s)->name,
             s->paused ? "PAUSED " : "running");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom row: the key reminders. */
static void scene_hud_bottom(int rows, int cols)
{
    (void)cols;
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit   spc:pause   n/p:ratio   +/-:drift   c:theme ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §6  screen ── */

/*
 * Screen -- the terminal's current size, remembered.
 *
 * We hold these as plain numbers so the drawing code can size the figure
 * without having to talk to ncurses itself.  Only startup and a resize
 * ever write to them.
 *
 *   cols   width  in character cells
 *   rows   height in character cells
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *sc)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* ── §7  app ── */

/*
 * App -- everything the program holds onto, in one place.
 *
 * There's a single global one (g_app) so the signal handlers can reach in
 * and set a flag without us threading it through every call.
 *
 * The two flags are volatile sig_atomic_t, not bool, because a signal can
 * land at any instant: that type is the one the standard promises is safe
 * to touch from a handler, and volatile stops the compiler from caching the
 * value in the loop so we'd never notice it changed.
 *
 *   scene        the live figure state
 *   screen       the cached terminal size
 *   running      set to 0 to ask the main loop to quit (Ctrl-C / kill)
 *   need_resize  set to 1 when the terminal was resized
 */
typedef struct {
    Scene                  scene;
    Screen                 screen;
    volatile sig_atomic_t  running;
    volatile sig_atomic_t  need_resize;
} App;

static App g_app;

static void sig_handler(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}
static void cleanup(void) { endwin(); }

/* Act on one keypress.  Returns false only when it was a quit key. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': return false;
    case ' ':           s->paused ^= 1;                                  break;
    case 'c': case 'C': s->theme_idx = (s->theme_idx + 1) % N_THEMES;    break;
    case 'n': case 'N':
        s->ratio_idx = (s->ratio_idx + 1) % N_RATIOS;
        s->phase_x   = 0.0f;
        break;
    case 'p': case 'P':
        s->ratio_idx = (s->ratio_idx + N_RATIOS - 1) % N_RATIOS;
        s->phase_x   = 0.0f;
        break;
    case '+': case '=':
        s->drift *= DRIFT_STEP;
        if (s->drift > DRIFT_MAX) s->drift = DRIFT_MAX;
        break;
    case '-': case '_':
        s->drift /= DRIFT_STEP;
        if (s->drift < DRIFT_MIN) s->drift = DRIFT_MIN;
        break;
    }
    return true;
}

static void install_signal_handlers(void)
{
    signal(SIGINT,   sig_handler);
    signal(SIGTERM,  sig_handler);
    signal(SIGWINCH, sig_handler);
    atexit(cleanup);
}

static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

/* After a resize: pick up the new size and wipe the screen so the next
 * frame is drawn fresh. */
static void apply_resize(App *app)
{
    app->need_resize = 0;
    screen_resize(&app->screen);
    erase();
}

/* Handle every key waiting in the buffer.  Returns false if one was quit. */
static bool drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR) {
        if (!app_handle_key(app, ch)) return false;
    }
    return true;
}

/* Advance the figure one step and paint a full frame: curve plus both HUDs. */
static void frame_render(App *app)
{
    scene_tick(&app->scene);
    erase();
    scene_draw      (&app->scene, app->screen.rows, app->screen.cols);
    scene_hud_top   (&app->scene, app->screen.cols);
    scene_hud_bottom(             app->screen.rows, app->screen.cols);
    wnoutrefresh(stdscr);
    doupdate();
}

/* Wait until it's time for the next frame, then hand back when the one after
 * that is due.  Any lateness is soaked up by the sleep, keeping a steady pace. */
static long long pace_to_deadline(long long deadline)
{
    deadline += TICK_NS;
    clock_sleep_ns(deadline - clock_ns());
    return deadline;
}

/* The main loop: handle a resize, read keys, draw a frame, wait, repeat. */
int main(void)
{
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    long long next_frame_deadline = clock_ns();

    while (app->running) {
        if (app->need_resize)  apply_resize(app);
        if (!drain_input(app)) { app->running = 0; break; }

        frame_render(app);
        next_frame_deadline = pace_to_deadline(next_frame_deadline);
    }

    return 0;
}
