/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * spirograph.c — three Spirograph-style curves drawn in colour, each
 * leaving a glowing trail that slowly fades.  The shapes morph on their
 * own as a hidden parameter drifts back and forth.
 *
 * These are hypotrochoids — the curve a pen draws when a small circle
 * rolls inside a bigger one, exactly like the toothed-gear Spirograph toy.
 * For the math, see Lawrence, "A Catalog of Special Plane Curves" (1972)
 * or Maor, "Trigonometric Delights" (1998).
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

/* ── §1 config ── */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_CURVES        = 3,
    N_THEMES        = 10,
    MAX_ROWS        = 80,
    MAX_COLS        = 240,

    /* Colour pairs 1..N_CURVES are the curve colours (the theme rewrites
     * them); the two above that are the fixed HUD colours. */
    N_PAIRS         = N_CURVES + 2,

    /* One row at the very top and one at the very bottom belong to the
     * HUD; the curves are drawn in the band between them. */
    HUD_TOP_ROWS    = 1,
    HUD_BOTTOM_ROWS = 1,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* How many sub-pixels make up one character cell, so the trace can move
 * smoothly before it snaps to a coarse grid of characters. */
#define CELL_W   8
#define CELL_H   16

/* Each tick the whole trail dims to 98.5% of its brightness, so a path
 * stays visible for about a second and a half before fading out. */
#define FADE        0.985f

/* How far along the curve we advance per tick, and how many points we
 * drop in between so the trace draws as a solid line, not dots. */
#define DELTA_T     0.08f
#define TRACE_STEPS 60

/* How fast the hidden parameter drifts; one full sweep takes ~157 s. */
#define DRIFT_RATE  0.04f

/* The biggest curve reaches about 12.5 units across.  We shrink the
 * whole figure so it fills 85% of the shorter screen side, leaving a
 * margin around the edge. */
#define MAX_CURVE_EXTENT   12.5f
#define FIT_FRACTION        0.85f

/* Brightness used when a pen touches a cell, the cutoff below which a
 * cell is too faint to draw, the two thresholds that pick bold vs dim,
 * and the size of the " .,:+*#@" character ramp. */
#define STAMP_BRIGHTNESS    1.0f
#define TRAIL_VISIBLE_MIN   0.04f
#define BRIGHT_THRESHOLD    0.7f
#define DIM_THRESHOLD       0.25f
#define N_GLYPHS            8

/* Never let the rolling radius shrink past this — keeps the (R-r)/r
 * ratio from blowing up when drift pushes r toward zero. */
#define R_ACTUAL_MIN        0.5f

/* If one frame takes an unusually long time (debugger, laptop sleep),
 * pretend it was at most 100 ms so we don't fire a burst of catch-up
 * ticks.  Screen redraws aim for 60 a second. */
#define DT_CAP_NS           (100 * NS_PER_MS)
#define FRAME_PERIOD_NS     (NS_PER_SEC / 60)

#define KEY_ESC             27

/* Where the HUD's middle text starts, and the fixed cyan/yellow HUD
 * colour pairs (these sit above the curve pairs so the theme never
 * touches them). */
#define HUD_DATA_COL        16
#define HUD_PAIR_TITLE      (N_CURVES + 1)
#define HUD_PAIR_DATA       (N_CURVES + 2)
#define HUD_PAIR_HINT       HUD_PAIR_TITLE
#define FALLBACK_PAIR       1               /* used if a trail cell ever
                                               holds a bad colour index */

/* ── §2 clock ── */

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

/* ── §3 color ── */

/*
 * Theme — a named set of three curve colours.  Pressing 't' loads the
 * theme's colours into ncurses pairs 1..N_CURVES.  Each curve keeps the
 * same pair number, so only the colours behind those numbers change —
 * which means trail cells already on screen instantly switch colour with
 * no need to redraw or clear anything.
 *
 *   name        the label shown in the HUD
 *   colors_256  the colours to use on a 256-colour terminal (preferred)
 *   colors_8    plain-colour fallback for old 8-colour terminals
 * Both colour arrays hold one entry per curve.
 */
typedef struct {
    const char *name;
    int         colors_256[N_CURVES];
    int         colors_8  [N_CURVES];
} Theme;

static const Theme k_themes[N_THEMES] = {
    { "Sunset",       { 196, 208, 226 }, { COLOR_RED,     COLOR_YELLOW,  COLOR_YELLOW  } },
    { "Ocean",        {  21,  51,  87 }, { COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN    } },
    { "Forest",       {  22,  46, 154 }, { COLOR_GREEN,   COLOR_GREEN,   COLOR_GREEN   } },
    { "Cyberpunk",    { 201,  51, 226 }, { COLOR_MAGENTA, COLOR_CYAN,    COLOR_YELLOW  } },
    { "Pastel",       { 218, 158, 183 }, { COLOR_RED,     COLOR_GREEN,   COLOR_MAGENTA } },
    { "Mono",         { 240, 250, 255 }, { COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE   } },
    { "Fire",         {  52, 196, 220 }, { COLOR_RED,     COLOR_RED,     COLOR_YELLOW  } },
    { "Aurora",       {  46,  51,  99 }, { COLOR_GREEN,   COLOR_CYAN,    COLOR_MAGENTA } },
    { "Royal",        {  57, 220,  63 }, { COLOR_MAGENTA, COLOR_YELLOW,  COLOR_BLUE    } },
    { "Mint",         {  23, 121, 195 }, { COLOR_CYAN,    COLOR_CYAN,    COLOR_CYAN    } },
};

/* Step to the next / previous theme, wrapping around the ends. */
static inline int theme_next(int cur) { return (cur + 1) % N_THEMES; }
static inline int theme_prev(int cur) { return (cur + N_THEMES - 1) % N_THEMES; }

/* Recolour the three curve pairs from the chosen theme.  Leaves the HUD
 * pairs alone so the HUD's colours stay the same no matter the theme. */
static void theme_apply(int theme_idx)
{
    const Theme *t = &k_themes[theme_idx];
    if (COLORS >= 256) {
        for (int i = 0; i < N_CURVES; i++)
            init_pair(i + 1, t->colors_256[i], COLOR_BLACK);
    } else {
        for (int i = 0; i < N_CURVES; i++)
            init_pair(i + 1, t->colors_8[i], COLOR_BLACK);
    }
}

/* Set up colour once at startup: the fixed HUD colours, then the first
 * theme's curve colours. */
static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(HUD_PAIR_TITLE,  51, COLOR_BLACK);   /* bright cyan   */
        init_pair(HUD_PAIR_DATA,  226, COLOR_BLACK);   /* bright yellow */
    } else {
        init_pair(HUD_PAIR_TITLE, COLOR_CYAN,   COLOR_BLACK);
        init_pair(HUD_PAIR_DATA,  COLOR_YELLOW, COLOR_BLACK);
    }

    theme_apply(0);
}

/* ── §4 coords: pixel ↔ cell ── */

static inline int px_to_col(float px) { return (int)floorf(px / CELL_W + 0.5f); }
static inline int px_to_row(float py) { return (int)floorf(py / CELL_H + 0.5f); }

/*
 * RenderFrame — where to put the figure on screen this frame.
 *
 * The curve is computed in continuous "unit" space, but the screen is a
 * coarse grid of characters.  We work in fine sub-pixels first (CELL_W ×
 * CELL_H of them per character) and only snap to a character at the very
 * end, so the line stays smooth instead of jumping in stair-steps.
 *
 * We rebuild this every frame from the current width and height rather
 * than storing it, so a terminal resize can never leave it pointing at
 * the old size.  The few sums it takes are nothing.
 *
 *   cx_px, cy_px  the middle of the screen, in sub-pixels
 *   scale         how many sub-pixels one curve unit becomes on screen
 */
typedef struct {
    float cx_px;
    float cy_px;
    float scale;
} RenderFrame;

static RenderFrame render_frame_make(int cols, int rows)
{
    int   shorter_px = (cols * CELL_W < rows * CELL_H)
                       ? cols * CELL_W : rows * CELL_H;
    float scale      = (float)shorter_px * (FIT_FRACTION * 0.5f)
                       / MAX_CURVE_EXTENT;
    return (RenderFrame){
        .cx_px = (float)(cols * CELL_W) * 0.5f,
        .cy_px = (float)(rows * CELL_H) * 0.5f,
        .scale = scale,
    };
}

/* ── §5 entity: Curve, Spirograph ── */

/*
 * Curve — one rolling "pen" and where it is right now.
 *
 * Picture a small circle of radius r rolling around the inside of a big
 * circle of radius R, with a pen stuck to the small circle a distance d
 * from its centre.  As the small circle rolls, the pen draws the curve.
 * If d is small the pen stays inside and draws a flower-like rosette; if
 * d is large the pen swings wide and the curve makes little loops.  How
 * many petals you get comes from the R-to-r ratio; see Lawrence (1972).
 *
 * The trick that makes the shape morph on its own: r isn't fixed.  It
 * gently oscillates around r_base by up to r_amp as `drift` advances,
 * so the petal count slides smoothly between whole numbers with no
 * keypress involved.
 *
 * All values are in curve "units" (the scale-to-screen happens later).
 *   R       radius of the big outer circle              (set once)
 *   r_base  the average radius of the small circle      (set once)
 *   r_amp   how far r swings above and below r_base      (set once)
 *   d       how far the pen sits from the small centre   (set once)
 *   t       how far along the curve we've drawn          (grows each tick)
 *   drift   the slow phase that wobbles r                (grows each tick)
 *   pair    which colour this curve draws in             (set once)
 */
typedef struct {
    float R;
    float r_base;
    float r_amp;
    float d;
    float t;
    float drift;
    int   pair;
} Curve;

/*
 * CurvePreset — the starting recipe for one curve, fixed and never
 * changed.  The three presets seed the demo at startup and again when
 * the user presses 'r'.  Kept as its own const type, separate from the
 * live Curve, so the compiler stops anything from accidentally writing
 * to a preset (a Curve, by contrast, changes every tick).
 *
 *   R, r_base, r_amp, d  the curve's geometry (see Curve above)
 *   t0      where this curve starts drawing — staggered across the three
 *           so they don't all begin at the same point
 *   drift0  starting drift phase — staggered so they morph at different
 *           times and you never see all three stall at once
 *   color_pair  which colour pair this curve uses
 *   name        a label for the HUD ("Pentagon", "Heptagon", ...)
 */
typedef struct {
    float       R;
    float       r_base;
    float       r_amp;
    float       d;
    float       t0;
    float       drift0;
    int         color_pair;
    const char *name;
} CurvePreset;

/* The three starting curves.  Columns, in order:
 *   R   r_base  r_amp   d   t0   drift0  colour-pair  name
 * Curve i gets colour pair i+1; t0 and drift0 are staggered so the
 * three never line up. */
static const CurvePreset k_presets[N_CURVES] = {
    {  5.0f,  3.0f,  0.8f,  5.5f,  0.0f,                     0.0f,    1,  "Pentagon" },
    {  7.0f,  2.0f,  0.6f,  7.0f,  2.0f*(float)M_PI/3.0f,    1.0f,    2,  "Heptagon" },
    {  6.0f,  4.0f,  0.9f,  6.0f,  4.0f*(float)M_PI/3.0f,    2.0f,    3,  "Triangle" },
};

static Curve curve_from_preset(const CurvePreset *p)
{
    return (Curve){
        .R     = p->R,    .r_base = p->r_base,
        .r_amp = p->r_amp, .d     = p->d,
        .t     = p->t0,   .drift  = p->drift0,
        .pair  = p->color_pair,
    };
}

/*
 * Trail — the screen's memory of where the pens have been, so each curve
 * looks like a glowing path that fades behind the pen instead of three
 * bare moving dots.  Every tick we dim the whole thing a little, then
 * stamp the cells the pens just touched back to full brightness.  A cell
 * too faint to matter is skipped when drawing, which is what gives the
 * trail its limited length.
 *
 * Brightness and colour are kept in two separate grids rather than one
 * grid of pairs: the dimming pass only walks the brightness grid, and a
 * tight sweep over one plain array is faster than skipping through a
 * mixed struct.  Both grids are a fixed max size so we never allocate
 * memory while running; the drawing only ever uses the part that fits
 * the real terminal.
 *
 *   brightness  how bright each cell glows, 0 to 1
 *   color_pair  the colour of the last pen to touch each cell — last one
 *               to write wins, which is how overlapping curves blend
 */
typedef struct {
    float brightness[MAX_ROWS][MAX_COLS];
    int   color_pair[MAX_ROWS][MAX_COLS];
} Trail;

/*
 * Spirograph — the whole simulation: the three rolling pens and the one
 * trail they all paint onto.  Everything else in the file is either the
 * HUD or the run-loop plumbing around this.
 *
 * They share a single trail on purpose — the curves drawing over each
 * other is the whole point, and one shared trail makes the overlap blend
 * for free (last pen to touch a cell wins its colour).
 *
 *   trail      the fading memory of past pen positions
 *   curves     the three rolling pens
 *   theme_idx  which colour theme is active; 't' cycles it.  Only changes
 *              colours, never the motion
 *   paused     freezes the motion but keeps drawing, so the figure stays
 *              up and you can still recolour while it's frozen (SPACE)
 */
typedef struct {
    Trail trail;
    Curve curves[N_CURVES];
    int   theme_idx;
    bool  paused;
} Spirograph;

static void spirograph_reset_curves(Spirograph *sg)
{
    for (int i = 0; i < N_CURVES; i++)
        sg->curves[i] = curve_from_preset(&k_presets[i]);
}

static void spirograph_init(Spirograph *sg)
{
    memset(sg, 0, sizeof *sg);
    sg->theme_idx = 0;
    spirograph_reset_curves(sg);
}

static void spirograph_clear_trail(Spirograph *sg)
{
    memset(&sg->trail, 0, sizeof sg->trail);
}

/* ── curve math + state helpers ── */

/* The small circle's radius right now, after the drift wobble — kept at
 * or above a floor so the curve math can't divide by something tiny. */
static float curve_r_actual(const Curve *cv)
{
    float r = cv->r_base + cv->r_amp * sinf(cv->drift);
    return (r < R_ACTUAL_MIN) ? R_ACTUAL_MIN : r;
}

/* The hypotrochoid formula: where the pen is at parameter t.  Rmr (= R-r)
 * and ratio (= (R-r)/r) are passed in already worked out. */
static void hypotrochoid_xy(float Rmr, float ratio, float d, float t,
                            float *x, float *y)
{
    *x = Rmr * cosf(t) + d * cosf(ratio * t);
    *y = Rmr * sinf(t) - d * sinf(ratio * t);
}

/* Move one curve forward by a tick — a bit further along the curve, and
 * a nudge of drift. */
static inline void curve_advance(Curve *cv, float dt)
{
    cv->t     += DELTA_T;
    cv->drift += DRIFT_RATE * dt;
}

/* ── trail helpers ── */

/* Dim every cell a notch so older parts of the trail fade away. */
static void trail_fade(Trail *trail, int rows, int cols)
{
    int row_end = (rows < MAX_ROWS) ? rows : MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;
    for (int r = 0; r < row_end; r++)
        for (int c = 0; c < col_end; c++)
            trail->brightness[r][c] *= FADE;
}

/* Light one cell up fully in a curve's colour. */
static inline void trail_stamp(Trail *trail, int row, int col, int pair)
{
    trail->brightness[row][col] = STAMP_BRIGHTNESS;
    trail->color_pair[row][col] = pair;
}

/* Is this cell in the drawable area — past the HUD rows and inside the
 * grid we have storage for? */
static inline bool trail_in_band(int row, int col, int rows, int cols)
{
    if (col < 0 || col >= cols || col >= MAX_COLS) return false;
    if (row < HUD_TOP_ROWS || row >= rows - HUD_BOTTOM_ROWS
        || row >= MAX_ROWS) return false;
    return true;
}

/* Draw a short stretch of one curve this tick: step along it many small
 * steps, work out the pen position at each, place it on screen, and stamp
 * the cell.  The many sub-steps keep the line solid instead of dotted. */
static void curve_trace(Curve *cv, Trail *trail,
                        const RenderFrame *rf, int rows, int cols)
{
    float r_actual = curve_r_actual(cv);
    float Rmr      = cv->R - r_actual;
    float ratio    = Rmr / r_actual;

    float sub_dt = DELTA_T / (float)TRACE_STEPS;
    for (int step = 0; step < TRACE_STEPS; step++) {
        float t = cv->t + (float)step * sub_dt;

        float x, y;
        hypotrochoid_xy(Rmr, ratio, cv->d, t, &x, &y);

        float px = rf->cx_px + x * rf->scale;
        float py = rf->cy_px + y * rf->scale;

        int col = px_to_col(px);
        int row = px_to_row(py);
        if (trail_in_band(row, col, rows, cols))
            trail_stamp(trail, row, col, cv->pair);
    }
}

/* One step of the simulation: fade the trail, then draw and move each
 * curve.  Does nothing while paused. */
static void spirograph_tick(Spirograph *sg, float dt, int cols, int rows)
{
    if (sg->paused) return;

    RenderFrame rf = render_frame_make(cols, rows);

    trail_fade(&sg->trail, rows, cols);

    for (int ci = 0; ci < N_CURVES; ci++) {
        Curve *cv = &sg->curves[ci];
        curve_trace  (cv, &sg->trail, &rf, rows, cols);
        curve_advance(cv, dt);
    }
}

/* ── drawing the trail ── */

/* Pick the character for a brightness: brighter cells get a denser glyph
 * from the " .,:+*#@" ramp. */
static inline int brightness_to_glyph_idx(float b)
{
    int idx = (int)(b * (float)(N_GLYPHS - 1));
    if (idx < 0)         idx = 0;
    if (idx >= N_GLYPHS) idx = N_GLYPHS - 1;
    return idx;
}

/* Make the brightest cells bold and the faintest dim, for extra contrast
 * between fresh and fading trail. */
static inline chtype brightness_to_attr(float b)
{
    if (b > BRIGHT_THRESHOLD) return A_BOLD;
    if (b < DIM_THRESHOLD)    return A_DIM;
    return 0;
}

/* Guard against a stray colour index — only curve colours (1..N_CURVES)
 * belong in the trail.  Shouldn't happen, but it's cheap insurance. */
static inline int sanitize_pair(int p)
{
    return (p < 1 || p > N_CURVES) ? FALLBACK_PAIR : p;
}

/* Draw one trail cell, unless it's too faint to bother with. */
static void render_trail_cell(WINDOW *w, const Trail *trail, int row, int col)
{
    static const char GLYPHS[N_GLYPHS] = " .,:+*#@";
    float b = trail->brightness[row][col];
    if (b < TRAIL_VISIBLE_MIN) return;

    int    pair = sanitize_pair(trail->color_pair[row][col]);
    char   ch   = GLYPHS[brightness_to_glyph_idx(b)];
    chtype attr = brightness_to_attr(b);

    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/* Draw the whole trail, cell by cell, across the band between the two
 * HUD rows. */
static void spirograph_draw(const Spirograph *sg, WINDOW *w, int cols, int rows)
{
    int row_end = rows - HUD_BOTTOM_ROWS;
    if (row_end > MAX_ROWS) row_end = MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int row = HUD_TOP_ROWS; row < row_end; row++)
        for (int col = 0; col < col_end; col++)
            render_trail_cell(w, &sg->trail, row, col);
}

/* ── §6 scene ── */

/* Scene — a thin wrapper holding whatever is on screen.  It's just the
 * one Spirograph here; the wrapper exists so a different demo could drop
 * into the same app without touching the run loop. */
typedef struct {
    Spirograph sg;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    (void)cols; (void)rows;
    memset(s, 0, sizeof *s);
    spirograph_init(&s->sg);
}

static void scene_tick(Scene *s, float dt, int cols, int rows)
{
    spirograph_tick(&s->sg, dt, cols, rows);
}

static void scene_draw(const Scene *s, WINDOW *w,
                       int cols, int rows, float alpha, float dt_sec)
{
    (void)alpha; (void)dt_sec;
    spirograph_draw(&s->sg, w, cols, rows);
}

/* ── §7 screen ── */

/* Screen — the terminal's current size, remembered so the drawing code
 * can take plain row/col numbers and stay clear of ncurses.  Only filled
 * in at startup and again after a resize.
 *
 *   cols  width in characters
 *   rows  height in characters
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* ── HUD pieces ── */

/* Top-left title. */
static void hud_draw_title(void)
{
    attron(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
    mvprintw(0, 0, " [SPIROGRAPH] ");
    attroff(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
}

/* Top-middle: the active theme name and the three curves' live radii. */
static void hud_draw_curve_radii(const Spirograph *sg)
{
    attron(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
    mvprintw(0, HUD_DATA_COL,
             " theme: %-10s  r=[%.1f, %.1f, %.1f] ",
             k_themes[sg->theme_idx].name,
             (double)curve_r_actual(&sg->curves[0]),
             (double)curve_r_actual(&sg->curves[1]),
             (double)curve_r_actual(&sg->curves[2]));
    attroff(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
}

/* Top-right: frame rate, sim rate, and paused/running. */
static void hud_draw_engine_stats(Screen *s, double fps, int sim_fps, bool paused)
{
    char buf[80];
    snprintf(buf, sizeof buf, " %5.1f fps  sim:%3d Hz  %s ",
             fps, sim_fps, paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    if (len >= s->cols) return;

    attron(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
    mvprintw(0, s->cols - len, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
}

/* Bottom row: the list of keys you can press. */
static void hud_draw_action_bar(Screen *s)
{
    attron(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit   SPACE:pause   r:reset   t:theme   [/]:sim Hz ");
    attroff(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
}

/* Draw the full HUD: title, stats, and key list. */
static void screen_draw_hud(Screen *s, const Scene *sc, double fps, int sim_fps)
{
    const Spirograph *sg = &sc->sg;
    hud_draw_title();
    hud_draw_curve_radii(sg);
    hud_draw_engine_stats(s, fps, sim_fps, sg->paused);
    hud_draw_action_bar(s);
}

static void screen_draw(Screen *s, const Scene *sc,
                        double fps, int sim_fps, float alpha, float dt_sec)
{
    erase();
    scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);
    screen_draw_hud(s, sc, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §8 app ── */

/*
 * App — everything the program needs in one place.  There's a single
 * shared copy (g_app below) so the signal handlers can reach it.
 *
 * The two signal flags live here, not in the simulation: deciding to
 * quit or noticing a resize are run-loop business, and keeping them out
 * of the Scene leaves the simulation as pure, self-contained state.  They
 * are volatile sig_atomic_t because that's the one type safe to touch
 * from a signal handler, and volatile stops the loop from caching a stale
 * value in a register and missing the change.
 *
 *   scene        the simulation
 *   screen       the cached terminal size
 *   sim_fps      how many times a second the curves step; [ and ] change
 *                it.  Separate from the ~60-a-second screen redraw
 *   running      cleared to stop the loop (Ctrl-C / kill)
 *   need_resize  set when the terminal was resized
 */
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
    Spirograph *sg = &app->scene.sg;
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC: return false;
    case ' ': sg->paused = !sg->paused; break;
    case 'r': case 'R':
        spirograph_reset_curves(sg);
        spirograph_clear_trail(sg);
        break;
    case 't':
        sg->theme_idx = theme_next(sg->theme_idx);
        theme_apply(sg->theme_idx);
        break;
    case 'T':
        sg->theme_idx = theme_prev(sg->theme_idx);
        theme_apply(sg->theme_idx);
        break;
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

/* ── startup / shutdown ── */

static void install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    app->sim_fps     = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene, app->screen.cols, app->screen.rows);
}

/* ── main-loop step helpers ── */

/* Handle a terminal resize: get the new size, wipe the trail (its old
 * positions no longer line up), and restart the timing so the next frame
 * doesn't think a huge amount of time passed. */
static void apply_resize(App *app, int64_t *frame_time, int64_t *sim_accum)
{
    endwin(); refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    spirograph_clear_trail(&app->scene.sg);
    app->need_resize = 0;
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

/* Time since the last frame, capped so a long stall doesn't unleash a
 * flood of catch-up ticks. */
static int64_t frame_dt_clamped(int64_t *last_ns)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *last_ns;
    *last_ns    = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned, so the
 * curves advance at a steady rate no matter the frame rate. */
static void frame_drain_sim_ticks(Scene *sc, int sim_fps, float dt_sec,
                                  int cols, int rows, int64_t *accum)
{
    int64_t tick_ns = TICK_NS(sim_fps);
    while (*accum >= tick_ns) {
        scene_tick(sc, dt_sec, cols, rows);
        *accum -= tick_ns;
    }
}

/* Update the displayed frame rate: average it over a short window, then
 * start the count over. */
static void fps_counter_update(int64_t dt, int64_t *accum_ns,
                               int *frame_count, double *fps_out)
{
    *accum_ns += dt;
    (*frame_count)++;
    if (*accum_ns < FPS_UPDATE_MS * NS_PER_MS) return;

    double seconds = (double)(*accum_ns) / (double)NS_PER_SEC;
    *fps_out      = (double)(*frame_count) / seconds;
    *frame_count  = 0;
    *accum_ns     = 0;
}

/* Sleep off the rest of this frame's time budget so we hold a steady
 * frame rate. */
static void frame_sleep_to_target(int64_t frame_start_ns, int64_t work_so_far)
{
    int64_t spent = clock_ns() - frame_start_ns + work_so_far;
    clock_sleep_ns(FRAME_PERIOD_NS - spent);
}

/* Read a pending keypress and act on it.  Returns false only on a quit
 * key. */
static bool drain_input(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Draw one whole frame and push it to the terminal. */
static void frame_render(App *app, double fps, float alpha, float dt_sec)
{
    screen_draw(&app->screen, &app->scene, fps, app->sim_fps, alpha, dt_sec);
    screen_present();
}

/* The main loop: handle a resize, step the sim, draw, sleep, read input,
 * repeat until quit. */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        if (app->need_resize) apply_resize(app, &frame_time, &sim_accum);

        int64_t dt      = frame_dt_clamped(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        frame_drain_sim_ticks(&app->scene, app->sim_fps, dt_sec,
                              app->screen.cols, app->screen.rows, &sim_accum);

        float alpha = (float)sim_accum / (float)tick_ns;

        fps_counter_update(dt, &fps_accum, &frame_count, &fps_display);

        frame_sleep_to_target(frame_time, dt);
        frame_render(app, fps_display, alpha, dt_sec);

        if (!drain_input(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
