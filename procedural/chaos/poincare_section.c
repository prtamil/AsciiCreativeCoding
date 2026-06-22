/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * poincare_section.c — watch the Lorenz attractor as a 2-D dot cloud.
 *
 * The Lorenz system is a 3-D path that loops around chaotically forever.
 * Instead of drawing the whole path, we wait for it to cross one flat
 * plane (heading upward) and drop a dot where it does.  Those dots pile
 * up into the attractor's signature shape: two thin crescents.
 *
 * Sister files: strange_attractor.c (the path drawn in full 3-D),
 * standard_map.c (same trick on a discrete map).
 * Equations and the canonical picture: Lorenz (1963); Sparrow (1982) Fig. 2.4.
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

/* §1  config */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  80,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_DENS_BASE    =   3,    /* first of the 8 density-shade pairs */
    PAIR_LIVE         =  11,    /* colour of the newest-dot marker */
};

#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

#define NS_PER_SEC               1000000000LL
#define NS_PER_MS                   1000000LL
#define TICK_NS(f)               (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET        60
#define RENDER_FRAME_BUDGET_NS   (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS      100

/* How fine we step the equation, and how many steps per frame. */
#define LORENZ_DT                 0.005f
#define INT_STEPS_PER_TICK         50

/* The crossing plane sits at z = rho - 1; set per preset at reset time. */

/* The (x, y) window the dots are plotted in, sized to hold the attractor. */
#define X_MIN                   -25.0f
#define X_MAX                    25.0f
#define Y_MIN                   -30.0f
#define Y_MAX                    30.0f

#define DENS_BAND_COUNT           8
#define DENS_SATURATE_LOG         8.0f

/* Only one preset here: the famous rho = 28 butterfly.  Cranking rho
 * much higher grows the attractor past the fixed window, so we stick
 * to the one setting that fits and is iconic. */
typedef enum {
    PRESET_CLASSIC = 0,
    N_PRESETS,
} Preset;

/*
 * LorenzPreset — one named experiment: the three equation knobs plus a
 * starting point.
 *
 * In the long run the shape depends only on (sigma, rho, beta) — any
 * nearby start gets pulled onto the same attractor.  The starting point
 * still matters for the brief settling-in before that happens.
 *
 *   name        : short label shown in the HUD.
 *   sigma       : equation knob (classic value 10).
 *   rho         : the main dial — picks the dynamical regime (classic 28).
 *   beta        : equation knob (classic 8/3).
 *   x0, y0, z0  : where the path starts; (1, 1, 1) for the classic run.
 *
 * Lives only in the presets[] table below.
 */
typedef struct {
    const char *name;
    float       sigma, rho, beta;
    float       x0, y0, z0;
} LorenzPreset;

static const LorenzPreset presets[N_PRESETS] = {
    { "CLASSIC ", 10.0f, 28.00f, 8.0f/3.0f,  1.0f, 1.0f, 1.0f },
};

/*
 * Theme — one colour scheme for the picture, swapped live with t/T.
 *
 * Keeping colour separate from the drawing code means one key can repaint
 * everything.  Each theme sets two things:
 *
 *   name   : short label shown in the HUD.
 *   band   : eight shades for the dots, from faintest (rarely-hit cells)
 *            to brightest (the busiest cells).  All indices stay at 24 or
 *            above so even the dimmest stays visible on a dark terminal.
 *   live   : colour of the marker on the newest dot; a hot tone so it
 *            pops out against the rest.
 */
typedef struct {
    const char *name;
    short       band[DENS_BAND_COUNT];
    short       live;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  39,  75, 117, 153, 189, 220, 226, 231 }, 196 },
    { "MATRIX",  {  46,  82, 118, 154, 156, 191, 192, 194 }, 226 },
    { "NOVA",    {  99, 135, 171, 207, 213, 219, 225, 231 }, 226 },
    { "MONO",    { 244, 247, 250, 252, 253, 254, 255, 231 }, 226 },
    { "OCEAN",   {  39,  45,  81, 117, 153, 159, 195, 231 }, 226 },
    { "FIRE",    {  88, 124, 160, 196, 202, 208, 220, 227 }, 231 },
    { "EARTH",   { 100, 137, 173, 179, 215, 222, 228, 231 }, 196 },
    { "FOREST",  {  64, 107, 114, 144, 150, 156, 194, 231 }, 226 },
    { "DESERT",  { 130, 173, 179, 215, 222, 228, 230, 231 }, 196 },
    { "ARCTIC",  {  81, 117, 153, 159, 195, 225, 230, 231 }, 196 },
};

/* §2  clock */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec = ns / NS_PER_SEC,
                            .tv_nsec = ns % NS_PER_SEC };
    nanosleep(&req, NULL);
}

/* §3  color */

/* Bright yellow / bright cyan for the HUD text. */
#define PAIR_HUD_FG_256    226
#define PAIR_HINT_FG_256    51

static void theme_apply_pairs_256color(const Theme *t)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, t->band[i], -1);
    init_pair(PAIR_LIVE, t->live, -1);
}

/* On a plain 8-colour terminal we can't show the themes, so every theme
 * falls back to the same readable cyan/red pair. */
static void theme_apply_pairs_8color_fallback(void)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, COLOR_CYAN, -1);
    init_pair(PAIR_LIVE, COLOR_RED, -1);
}

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_apply_pairs_256color(&themes[idx]);
    else               theme_apply_pairs_8color_fallback();
}

/* HUD colours are fixed, not part of any theme, so the status text keeps
 * its contrast no matter which theme is showing. */
static void color_init_hud_pairs(void)
{
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  PAIR_HUD_FG_256,  -1);
        init_pair(PAIR_HINT, PAIR_HINT_FG_256, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    color_init_hud_pairs();
    theme_apply(0);
}

/* §5  physics — Lorenz ODE */

/* A plain triple of floats. */
typedef struct {
    float x, y, z;
} Vec3;

/* Same layout as Vec3; the name just says "this one is a point on the
 * Lorenz path" wherever it's used. */
typedef Vec3 LorenzState;

/*
 * LorenzSystem — the three constants that define the equation:
 *
 *   dx/dt = sigma * (y - x)
 *   dy/dt = x * (rho - z) - y
 *   dz/dt = x * y - beta * z
 *
 *   sigma : equation constant (classic value 10).
 *   rho   : the main dial that picks the behaviour (classic 28).
 *   beta  : equation constant (classic 8/3).
 *
 * Set once per preset, then read on every step.  (Lorenz 1963.)
 */
typedef struct {
    float sigma, rho, beta;
} LorenzSystem;

/*
 * Lorenz — the equation's fixed constants plus its moving position,
 * bundled so callers pass one thing.  Stepping changes only .state;
 * picking a preset changes only .system.
 *
 *   system : the constants (sigma, rho, beta).
 *   state  : the current point (x, y, z).
 */
typedef struct {
    LorenzSystem system;
    LorenzState  state;
} Lorenz;

/* The Lorenz equations themselves: given a point, return how fast each
 * coordinate is changing right now.  One line per equation. */
static inline LorenzState lorenz_deriv(const LorenzState *s,
                                       const LorenzSystem *sys)
{
    LorenzState dy;
    dy.x = sys->sigma * (s->y - s->x);                  /* σ(y − x)         */
    dy.y = s->x * (sys->rho - s->z) - s->y;             /* x(ρ − z) − y     */
    dy.z = s->x * s->y - sys->beta * s->z;              /* xy − βz          */
    return dy;
}

/* Take a point and nudge it along a slope k by step h: a + h*k.  RK4
 * uses this to build its trial points. */
static inline LorenzState state_add(const LorenzState *a,
                                    float h, const LorenzState *k)
{
    LorenzState r;
    r.x = a->x + h * k->x;
    r.y = a->y + h * k->y;
    r.z = a->z + h * k->z;
    return r;
}

/* Same a + h*b as state_add, but takes its arguments by value.  Nothing
 * currently calls it. */
static inline Vec3 vec3_add(Vec3 a, float h, Vec3 b)
{
    Vec3 r = { a.x + h * b.x, a.y + h * b.y, a.z + h * b.z };
    return r;
}

#define RK4_BUTCHER_WEIGHT_SUM   6.0f
#define RK4_MIDPOINT_FRACTION    0.5f

/* Blend the four slope estimates into one, weighting the two middle ones
 * double: (k1 + 2k2 + 2k3 + k4) / 6. */
static inline LorenzState rk4_butcher_weighted_average(
    const LorenzState *k1, const LorenzState *k2,
    const LorenzState *k3, const LorenzState *k4)
{
    LorenzState avg;
    avg.x = (k1->x + 2.0f*k2->x + 2.0f*k3->x + k4->x) / RK4_BUTCHER_WEIGHT_SUM;
    avg.y = (k1->y + 2.0f*k2->y + 2.0f*k3->y + k4->y) / RK4_BUTCHER_WEIGHT_SUM;
    avg.z = (k1->z + 2.0f*k2->z + 2.0f*k3->z + k4->z) / RK4_BUTCHER_WEIGHT_SUM;
    return avg;
}

/* Advance the path one small step with the classic RK4 recipe: sample
 * the slope at the start, twice in the middle, and at the end, then move
 * by their weighted average.  Far more accurate than a single Euler step
 * for the same dt.  (Numerical Recipes Ch. 17.1.) */
static void lorenz_rk4(Lorenz *L, float dt)
{
    const LorenzSystem *sys = &L->system;
    float half_dt = RK4_MIDPOINT_FRACTION * dt;

    LorenzState slope_start    = lorenz_deriv(&L->state, sys);
    LorenzState midpoint_1     = state_add(&L->state, half_dt, &slope_start);
    LorenzState slope_mid_1    = lorenz_deriv(&midpoint_1, sys);
    LorenzState midpoint_2     = state_add(&L->state, half_dt, &slope_mid_1);
    LorenzState slope_mid_2    = lorenz_deriv(&midpoint_2, sys);
    LorenzState endpoint       = state_add(&L->state, dt, &slope_mid_2);
    LorenzState slope_end      = lorenz_deriv(&endpoint, sys);

    LorenzState effective_slope = rk4_butcher_weighted_average(
        &slope_start, &slope_mid_1, &slope_mid_2, &slope_end);
    L->state = state_add(&L->state, dt, &effective_slope);
}

/* §6  Poincaré section grid */

/*
 * SectionGrid — the tally board the dots land on.
 *
 * The grid is one counter per screen cell.  Every time the path crosses
 * the plane, we bump the cell it landed in.  Busy cells get big counts,
 * the picture builds up, and §6 density_band turns each count into a
 * shade.  We also remember the very last crossing so the newest dot can
 * be highlighted.
 *
 * Counting into a fixed grid (rather than keeping a growing list of
 * points) means no memory is allocated while running, and recording a
 * crossing is always the same tiny cost no matter how many have come
 * before.  A cell would need ~1.8 hours of crossings to overflow its
 * 16-bit counter, so we never worry about it.
 *
 *   w, h     : grid size in cells (set at reset, matches the screen area).
 *   count    : w * h, cached so reset only clears the part in use.
 *   hits     : the counters, stored row by row.  Row 0 is the top of the
 *              screen (highest y); the up/down flip is done when recording
 *              so the painters can just walk top to bottom.
 *   last_x,
 *   last_y   : where the most recent crossing landed (real coordinates),
 *              drawn as the highlighted newest dot.
 *   has_live : false until the first crossing, so we don't draw a stale
 *              marker on an empty board.
 *
 * (The crossing-sampling idea is Poincaré 1892.)
 */
typedef struct {
    int      w, h;
    int      count;
    uint16_t hits[CELLS_MAX];
    float    last_x, last_y;
    bool     has_live;
} SectionGrid;

static void section_reset(SectionGrid *g, int w, int h)
{
    g->w = w; g->h = h; g->count = w * h;
    memset(g->hits, 0, sizeof(uint16_t) * (size_t)g->count);
    g->has_live = false;
}

/* Turn a real (x, y) into a grid cell; returns false if it's off the
 * window. */
static bool xy_to_cell(const SectionGrid *g, float x, float y, int *cx, int *cy)
{
    if (x < X_MIN || x > X_MAX || y < Y_MIN || y > Y_MAX) return false;
    *cx = (int)((x - X_MIN) / (X_MAX - X_MIN) * (float)g->w);
    *cy = (int)((y - Y_MIN) / (Y_MAX - Y_MIN) * (float)g->h);
    if (*cx < 0)        *cx = 0;
    if (*cx >= g->w)    *cx = g->w - 1;
    if (*cy < 0)        *cy = 0;
    if (*cy >= g->h)    *cy = g->h - 1;
    return true;
}

static void section_record(SectionGrid *g, float x, float y)
{
    int cx, cy;
    if (!xy_to_cell(g, x, y, &cx, &cy)) return;
    int idx = (g->h - 1 - cy) * g->w + cx;
    if (g->hits[idx] < UINT16_MAX) g->hits[idx]++;
    g->last_x  = x;
    g->last_y  = y;
    g->has_live = true;
}

/* Adding 0.5 before chopping to an int rounds to nearest instead of
 * always rounding down. */
#define DENSITY_BAND_ROUND_OFFSET   0.5f

/* Pick a shade (0..7) for a cell from its hit count, returning -1 for an
 * empty cell so the caller leaves it blank.  We take the log of the count
 * first: the busiest cells get thousands of hits and the faint ones only
 * a handful, so a straight scale would wash the faint stuff out. */
static inline int density_band(uint16_t hits)
{
    if (hits == 0) return -1;

    float saturated_log = logf((float)hits) / DENS_SATURATE_LOG;
    if (saturated_log < 0.0f) saturated_log = 0.0f;
    if (saturated_log > 1.0f) saturated_log = 1.0f;

    int band = (int)(saturated_log * (float)(DENS_BAND_COUNT - 1)
                     + DENSITY_BAND_ROUND_OFFSET);
    if (band > DENS_BAND_COUNT - 1) band = DENS_BAND_COUNT - 1;
    return band;
}

/* §7  state — crossing detection + small typed wrappers */

/*
 * CrossingDetector — spots the moment the path crosses the plane going up.
 *
 * To catch a crossing we compare each new point against the one before:
 * if z was below the plane and now sits on or above it, we just crossed
 * upward.  We then estimate exactly where between the two points the
 * crossing happened.  Remembering the previous point is the whole job.
 *
 *   z_plane    : the plane's height, set to rho - 1.  That's the level of
 *                the attractor's two centres, so the plane slices both
 *                wings evenly — the standard choice (Sparrow Fig. 2.4).
 *   prev_state : the full previous point, used to estimate the crossing.
 *   prev_z     : its z again, kept handy for the quick "did we cross?" test.
 */
typedef struct {
    float       z_plane;
    LorenzState prev_state;
    float       prev_z;
} CrossingDetector;

static void crossing_detector_init(CrossingDetector *d,
                                   float rho, LorenzState initial)
{
    d->z_plane    = rho - 1.0f;
    d->prev_state = initial;
    d->prev_z     = initial.z;
}

/* True if the path just stepped from below the plane to on/above it.
 * Cheap check we run before bothering with the crossing-point math. */
static inline bool crossing_detector_is_upward_z_plane_crossing(
    const CrossingDetector *d, const LorenzState *current)
{
    return d->prev_z < d->z_plane && current->z >= d->z_plane;
}

/* Estimate the (x, y) right at the plane.  The crossing happened somewhere
 * between the previous point and this one; alpha is how far along that gap
 * the plane sits (0 = previous, 1 = current), and we slide x and y the
 * same fraction. */
static inline void crossing_detector_interpolate_xy(
    const CrossingDetector *d, const LorenzState *current,
    float *x_cross, float *y_cross)
{
    float dz    = current->z - d->prev_z;
    float alpha = (dz != 0.0f) ? (d->z_plane - d->prev_z) / dz : 0.0f;
    *x_cross    = d->prev_state.x + alpha * (current->x - d->prev_state.x);
    *y_cross    = d->prev_state.y + alpha * (current->y - d->prev_state.y);
}

/* Stash the current point so the next step has something to compare to. */
static inline void crossing_detector_snapshot(CrossingDetector *d,
                                              const LorenzState *current)
{
    d->prev_state = *current;
    d->prev_z     = current->z;
}

/*
 * PresetState — just an index into presets[], wrapped in a struct.
 *
 * Wrapping the bare int gives the call sites readable names
 * (preset_state_active) instead of raw array indexing, and matches the
 * shape used in sister files.  Cycle helpers are skipped here since
 * there's only one preset.
 *
 *   current : which row of presets[] is loaded (always CLASSIC for now).
 */
typedef struct {
    int current;
} PresetState;

static void preset_state_init(PresetState *p, int initial) { p->current = initial; }
static const LorenzPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }
/* No cycle helpers: there's only one preset.  Add them if the table grows. */

/*
 * PaletteState — the same idea for "which theme is showing".
 *
 * Kept as its own type rather than reusing PresetState even though the
 * shape matches: that way the theme helpers can only ever be handed a
 * PaletteState, so a "next theme" key can't accidentally walk off the
 * end of the wrong table.
 *
 *   current : which row of themes[] is active (starts at 0, the default).
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)        { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)        { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p) { return &themes[p->current]; }
static void palette_state_apply(const PaletteState *p)       { theme_apply(p->current); }

/* §8  scene */

/*
 * Scene — everything that changes while the demo runs, in one place.
 *
 * Bundling it all here means there are no loose globals; the main loop
 * just hands a Scene* around.  The pieces are the moving parts of the
 * simulation:
 *
 *   lorenz   : the equation and its current point.
 *   section  : the tally board the dots accumulate on.
 *   detector : the crossing spotter — the link that turns the smooth path
 *              into discrete dots.
 *   preset   : which experiment is loaded.
 *   palette  : which theme is showing.
 *   paused   : when true the simulation freezes but keeps drawing, so you
 *              can study the picture so far.
 */
typedef struct {
    Lorenz           lorenz;
    SectionGrid      section;
    CrossingDetector detector;
    PresetState      preset;
    PaletteState     palette;
    bool             paused;
} Scene;

/* Load the chosen preset: copy its constants and starting point into the
 * equation, then point the crossing detector at the matching plane. */
static void scene_load_active_preset(Scene *s)
{
    const LorenzPreset *p = preset_state_active(&s->preset);
    s->lorenz.system.sigma = p->sigma;
    s->lorenz.system.rho   = p->rho;
    s->lorenz.system.beta  = p->beta;
    s->lorenz.state.x      = p->x0;
    s->lorenz.state.y      = p->y0;
    s->lorenz.state.z      = p->z0;
    crossing_detector_init(&s->detector, p->rho, s->lorenz.state);
}

static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

static void scene_reset(Scene *s, int w, int h)
{
    section_reset(&s->section, w, h);
    scene_load_active_preset(s);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->paused = false;
    preset_state_init (&s->preset,  PRESET_CLASSIC);
    palette_state_init(&s->palette, 0);
    scene_reset(s, w, h);
}

/* One simulation tick: step the path forward a bunch of small steps, and
 * each time it crosses the plane going up, drop a dot on the board. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;

    for (int i = 0; i < INT_STEPS_PER_TICK; i++) {
        crossing_detector_snapshot(&s->detector, &s->lorenz.state);
        lorenz_rk4(&s->lorenz, LORENZ_DT);
        if (crossing_detector_is_upward_z_plane_crossing(&s->detector,
                                                         &s->lorenz.state)) {
            float x_cross, y_cross;
            crossing_detector_interpolate_xy(&s->detector, &s->lorenz.state,
                                             &x_cross, &y_cross);
            section_record(&s->section, x_cross, y_cross);
        }
    }
}

/* §9  screen */

/*
 * Screen — the terminal's current size, plus the home for all ncurses
 * setup and teardown.  Passing this around lets each painter take one
 * handle instead of asking the terminal for its size every time.
 *
 *   cols : terminal width in character cells.
 *   rows : terminal height; row 0 is the top, row (rows - 1) the bottom.
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
static void screen_resize(Screen *s) { endwin(); refresh();
                                       getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* The character used for each shade.  Picked so they read light-to-heavy
 * even with no colour: a single dot for the faintest cells up to a solid
 * '@' for the busiest. */
static char glyph_for_band(int b)
{
    static const char ramp[DENS_BAND_COUNT] = {
        '.',  /* faintest */
        ':',
        ';',
        '+',
        'o',
        '*',
        '#',
        '@',  /* busiest */
    };
    if (b < 0)                    b = 0;
    if (b > DENS_BAND_COUNT - 1)  b = DENS_BAND_COUNT - 1;
    return ramp[b];
}

/* Box around the plotting area. */
#define SECTION_FRAME_CORNER       '+'
#define SECTION_FRAME_HORIZ        '-'
#define SECTION_FRAME_VERT         '|'

/* Faint x=0 / y=0 guide lines so the eye can find the centre. */
#define SECTION_AXIS_VERT          '|'
#define SECTION_AXIS_HORIZ         '-'
#define SECTION_AXIS_ORIGIN        '+'

/* The two crescents wind around the attractor's two centres; we mark each
 * with a bright '(+)' so you can see what the chaos is circling. */
#define FIXED_POINT_GLYPH_LEFT     '('
#define FIXED_POINT_GLYPH_CENTRE   '+'
#define FIXED_POINT_GLYPH_RIGHT    ')'

static void paint_frame_top_and_bottom_edges(int gx0, int gy0, int w, int h)
{
    int top_row    = gy0 - 1;
    int bottom_row = gy0 + h;
    for (int x = 0; x < w; x++) {
        mvaddch(top_row,    gx0 + x, SECTION_FRAME_HORIZ);
        mvaddch(bottom_row, gx0 + x, SECTION_FRAME_HORIZ);
    }
}

static void paint_frame_left_and_right_edges(int gx0, int gy0, int w, int h)
{
    int left_col  = gx0 - 1;
    int right_col = gx0 + w;
    for (int y = 0; y < h; y++) {
        mvaddch(gy0 + y, left_col,  SECTION_FRAME_VERT);
        mvaddch(gy0 + y, right_col, SECTION_FRAME_VERT);
    }
}

/* Drawn after the edges so the corner '+' overwrites the edge characters
 * where they meet. */
static void paint_frame_corners(int gx0, int gy0, int w, int h)
{
    int top_row    = gy0 - 1,   bottom_row = gy0 + h;
    int left_col   = gx0 - 1,   right_col  = gx0 + w;
    mvaddch(top_row,    left_col,  SECTION_FRAME_CORNER);
    mvaddch(top_row,    right_col, SECTION_FRAME_CORNER);
    mvaddch(bottom_row, left_col,  SECTION_FRAME_CORNER);
    mvaddch(bottom_row, right_col, SECTION_FRAME_CORNER);
}

/* Draw the box one cell outside the data on every side. */
static void paint_section_frame(int gx0, int gy0, int w, int h)
{
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    paint_frame_top_and_bottom_edges(gx0, gy0, w, h);
    paint_frame_left_and_right_edges(gx0, gy0, w, h);
    paint_frame_corners             (gx0, gy0, w, h);
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* Draw the centre guide lines.  Done before the dots so the data paints
 * over them, and left non-bold so they don't compete for attention. */
static void paint_section_axes(const SectionGrid *g, int gx0, int gy0)
{
    int x_zero_col   = (int)((0.0f - X_MIN) / (X_MAX - X_MIN) * (float)g->w);
    int y_zero_row   = (int)((0.0f - Y_MIN) / (Y_MAX - Y_MIN) * (float)g->h);
    int y_zero_screen_row = gy0 + (g->h - 1 - y_zero_row);

    attron(COLOR_PAIR(PAIR_DENS_BASE));
    if (x_zero_col >= 0 && x_zero_col < g->w)
        for (int y = 0; y < g->h; y++)
            mvaddch(gy0 + y, gx0 + x_zero_col, SECTION_AXIS_VERT);
    if (y_zero_row >= 0 && y_zero_row < g->h)
        for (int x = 0; x < g->w; x++)
            mvaddch(y_zero_screen_row, gx0 + x, SECTION_AXIS_HORIZ);
    if (x_zero_col >= 0 && x_zero_col < g->w &&
        y_zero_row >= 0 && y_zero_row < g->h)
        mvaddch(y_zero_screen_row, gx0 + x_zero_col, SECTION_AXIS_ORIGIN);
    attroff(COLOR_PAIR(PAIR_DENS_BASE));
}

/* Mark the two centres the crescents wrap around.  Their position works
 * out to (+/- sqrt(beta*(rho-1)), same), one per wing. */
static void paint_fixed_point_markers(const SectionGrid *g, int gx0, int gy0,
                                      float rho, float beta)
{
    float discriminant = beta * (rho - 1.0f);
    if (discriminant <= 0.0f) return;
    float fp = sqrtf(discriminant);

    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    for (int sign = -1; sign <= 1; sign += 2) {
        float fx = (float)sign * fp;
        float fy = (float)sign * fp;
        int cx, cy;
        if (!xy_to_cell(g, fx, fy, &cx, &cy)) continue;
        int sy = gy0 + (g->h - 1 - cy);
        int sx = gx0 + cx;
        if (sx > gx0 && sx < gx0 + g->w - 1) {
            mvaddch(sy, sx - 1, FIXED_POINT_GLYPH_LEFT);
            mvaddch(sy, sx + 1, FIXED_POINT_GLYPH_RIGHT);
        }
        mvaddch(sy, sx, FIXED_POINT_GLYPH_CENTRE);
    }
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* The main picture: draw each non-empty cell as a shaded character, and
 * skip empty ones so the guide lines underneath still show. */
static void paint_density_cells(const SectionGrid *g, int gx0, int gy0,
                                int cols, int rows)
{
    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= rows - HUD_BOTTOM_ROWS) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int b = density_band(g->hits[y * g->w + x]);
            if (b < 0) continue;
            int pair = PAIR_DENS_BASE + b;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph_for_band(b));
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Highlight the newest dot with an '@', drawn last so it sits on top —
 * you watch it hop around as fresh crossings come in. */
static void paint_live_crossing(const SectionGrid *g, int gx0, int gy0,
                                int cols, int rows)
{
    if (!g->has_live) return;
    int cx, cy;
    if (!xy_to_cell(g, g->last_x, g->last_y, &cx, &cy)) return;
    int sy = gy0 + (g->h - 1 - cy);
    int sx = gx0 + cx;
    if (sx < 0 || sx >= cols ||
        sy < HUD_TOP_ROWS || sy >= rows - HUD_BOTTOM_ROWS) return;
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    mvaddch(sy, sx, '@');
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* Paint the whole picture, back to front, so the layers stack cleanly:
 * guide lines, then the dots, then the centre markers and the newest-dot
 * highlight, then the box on top. */
static void section_paint(const SectionGrid *g, int cols, int rows,
                          float rho, float beta)
{
    /* Centre the plotting area, kept clear of the frame and HUD. */
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 1)                gx0 = 1;
    if (gy0 < HUD_TOP_ROWS + 1) gy0 = HUD_TOP_ROWS + 1;

    paint_section_axes        (g, gx0, gy0);
    paint_density_cells       (g, gx0, gy0, cols, rows);
    paint_fixed_point_markers (g, gx0, gy0, rho, beta);
    paint_live_crossing       (g, gx0, gy0, cols, rows);
    paint_section_frame       (   gx0, gy0, g->w, g->h);
}

static void hud_paint_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " POINCARÉ SECTION (Lorenz) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Builds the status text (fps, Hz, preset, rho) but doesn't draw it, so
 * the caller reads as "format, then paint". */
static void hud_format_top_right_status(char *buf, size_t bufsz,
                                        double fps, int sim_fps,
                                        const Scene *s)
{
    const LorenzPreset *active = preset_state_active(&s->preset);
    snprintf(buf, bufsz,
             " %5.1f fps  %3d Hz  %s [%d/%d]  ρ:%.2f ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : active->name,
             s->preset.current + 1, N_PRESETS,
             (double)s->lorenz.system.rho);
}

/* Paint text flush against the right edge of row 0. */
static void hud_paint_top_right(int cols, const char *text)
{
    int anchor_col = cols - (int)strlen(text);
    if (anchor_col < 0) anchor_col = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, anchor_col, "%s", text);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top_right_status(int cols, double fps, int sim_fps,
                                 const Scene *s)
{
    char buf[HUD_COLS + 1];
    hud_format_top_right_status(buf, sizeof buf, fps, sim_fps, s);
    hud_paint_top_right(cols, buf);
}

/* Widths of the row-1 cells, fixed so the layout doesn't jump as preset
 * and theme names change length. */
#define HUD_PARAM_CELL_WIDTH_PRESET   19   /* " preset:XXXXXXXX " */
#define HUD_PARAM_CELL_WIDTH_THEME    17   /* " theme:XXXXXXXX "  */

static void hud_paint_param_cell_bold(int cursor_x, const char *fmt, const char *value)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, cursor_x, fmt, value);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* The second HUD line: preset name, theme name, then the equation values. */
static void hud_param_row(const Scene *s)
{
    int cursor_x = HUD_LEFT_MARGIN;

    hud_paint_param_cell_bold(cursor_x, " preset:%-8s ",
                              preset_state_active(&s->preset)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_PRESET;

    hud_paint_param_cell_bold(cursor_x, " theme:%-8s ",
                              palette_state_active(&s->palette)->name);
    cursor_x += HUD_PARAM_CELL_WIDTH_THEME;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, cursor_x, " σ:%.1f  ρ:%.2f  β:%.2f  z-plane:%.2f ",
             (double)s->lorenz.system.sigma,
             (double)s->lorenz.system.rho,
             (double)s->lorenz.system.beta,
             (double)s->detector.z_plane);
    attroff(COLOR_PAIR(PAIR_HUD));
}
static void hud_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    section_paint(&s->section, sc->cols, sc->rows,
                  s->lorenz.system.rho, s->lorenz.system.beta);
    hud_paint_top_left_title();
    hud_top_right_status(sc->cols, fps, sim_fps, s);
    hud_param_row(s);
    hud_bottom_hint(sc->rows);
}

/* §10 app */

/*
 * App — the whole program in one struct: simulation, screen, settings,
 * and the flags the signal handlers set.  There's no other global state.
 *
 * It has to be a global (g_app) because signal handlers can only safely
 * reach a global, and they're only allowed to touch the two volatile
 * flags below.  Everything else is reached through normal App* calls.
 *
 *   scene       : the simulation itself.
 *   screen      : terminal size + ncurses handle.
 *   sim_fps     : how fast the physics steps, adjusted with ] and [.  Kept
 *                 separate from the fixed 60 fps drawing rate.
 *   map_w,
 *   map_h       : size of the plotting area in cells.
 *   running     : cleared to stop the main loop (by q/ESC or a signal).
 *   need_resize : set by the window-resize signal; the next frame rebuilds.
 *
 * running and need_resize are volatile sig_atomic_t because signal
 * handlers write them (POSIX requires that type for signal-safety).
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;
static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Cells the box eats: one on each side, so two per axis. */
#define SECTION_FRAME_RESERVED  2

/* Size the plotting area to the terminal, leaving room for the HUD and
 * the box, and clamped to a sane min and the static-array max. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols - SECTION_FRAME_RESERVED;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS - SECTION_FRAME_RESERVED;
    if (mw < 16)        mw = 16;
    if (mh < 8)         mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw; app->map_h = mh;
}
/* Ctrl-C / kill -> quit; terminal resize -> rebuild on the next frame. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* One-time startup: seed, defaults, ncurses, sizing, build the scene. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* If the window was resized, redo the screen and plotting area. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* Time since the last frame, capped so a long stall (e.g. a paused
 * terminal) can't make the physics try to catch up in one giant jump. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Run the physics in fixed-size steps, however much real time has built
 * up.  Using a fixed step (not the variable frame time) keeps the picture
 * the same on a slow terminal as on a fast one. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Refresh the displayed fps a couple of times a second, not every frame. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleep off whatever's left of the frame so drawing holds ~60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed up / slow down the physics, kept within bounds. */
static void app_sim_rate_faster(App *app)
{
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
}
static void app_sim_rate_slower(App *app)
{
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/* Named one-liners so the key table below reads cleanly. */
static void app_toggle_pause      (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_orbit       (App *app) { scene_reset(&app->scene, app->map_w, app->map_h); }
static void app_cycle_theme_next  (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev  (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }

static bool app_handle_key(App *app, int ch);

/* Check for a keypress without blocking; returns false to quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* The key bindings. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_orbit      (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — the timing bookkeeping the main loop carries frame to
 * frame, bundled so main() stays readable.
 *
 *   frame_time  : when the last frame happened.
 *   sim_accum   : real time waiting to be turned into physics steps.
 *   fps_accum   : time since the fps readout was last refreshed.
 *   frame_count : frames drawn since that refresh.
 *   fps_display : the fps figure currently shown in the HUD.
 */
typedef struct {
    int64_t frame_time;
    int64_t sim_accum;
    int64_t fps_accum;
    int     frame_count;
    double  fps_display;
} FrameClock;

static void frame_clock_init(FrameClock *c)
{
    c->frame_time  = clock_ns();
    c->sim_accum   = 0;
    c->fps_accum   = 0;
    c->frame_count = 0;
    c->fps_display = 0.0;
}

static void frame_clock_reset_after_resize(FrameClock *c)
{
    c->frame_time = clock_ns();
    c->sim_accum  = 0;
}

static void frame_clock_advance(FrameClock *c, int64_t dt)
{
    c->sim_accum += dt;
    c->fps_accum += dt;
    c->frame_count++;
}

int main(void)
{
    main_install_signal_handlers();

    App *app = &g_app;
    app_bootstrap(app);

    FrameClock clk;
    frame_clock_init(&clk);

    while (app->running) {
        if (app->need_resize) {
            app_handle_pending_resize(app);
            frame_clock_reset_after_resize(&clk);
        }

        int64_t dt = app_compute_frame_dt(&clk.frame_time);
        frame_clock_advance(&clk, dt);
        app_drain_fixed_timestep(app, &clk.sim_accum);
        app_update_fps_meter(&clk.fps_accum, &clk.frame_count, &clk.fps_display);

        app_throttle_to_render_target(clk.frame_time, dt);
        app_present_frame(app, clk.fps_display);

        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
