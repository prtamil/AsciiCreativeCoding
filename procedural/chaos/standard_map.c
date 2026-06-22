/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * standard_map.c — the Chirikov-Taylor "standard map": a tiny rule you
 * apply over and over to a swarm of points, watching order dissolve
 * into chaos as one knob K turns up.  Where the points pile up gets
 * drawn brighter, so regular orbits show as thin curves and chaotic
 * ones as fuzzy clouds.
 *
 * Key papers, for the values and ideas the code can't explain itself:
 *   Chirikov 1979, Phys. Reports 52(5) — the original map.
 *   Greene 1979, J. Math. Phys. 20 — KC_GREENE = 0.9716354, the K at
 *     which the last regular curve breaks and chaos spans the torus.
 * Sister demos: ./bifurcation.c (1-D map chaos), ./double_pendulum.c
 *   and ./poincare_section.c (continuous-time chaos).
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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define TAU (2.0f * (float)M_PI)

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
    PAIR_DENS_BASE    =   3,   /* first of the 10 brightness-band colour pairs */
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

/* How many points we set loose, on a 16x16 = 256 grid of starts.
 * Fewer points run for longer draw the cleanest picture: each one has
 * time to trace its whole curve before the others paint over it.  Pack
 * in 900+ and the curves blur into one grey fog and every preset looks
 * the same. */
#define TRAJ_GRID_W              16
#define TRAJ_GRID_H              16
#define NUM_TRAJECTORIES         (TRAJ_GRID_W * TRAJ_GRID_H)

/* How many times we advance each point per tick.  Enough to draw a
 * full curve in about half a second, and nowhere near enough work to
 * matter for speed. */
#define ITERATES_PER_TRAJ_PER_TICK   30

/* When to stop.  After this many steps per point the picture has
 * settled; running longer would just over-darken cells, so we freeze
 * it for a clean image (press 'r' to start the same picture over). */
#define MAX_ITERATIONS_PER_TRAJ  6000

/* The brightness ladder: 10 colour bands times 10 glyph shapes give
 * 100 distinct levels.  DENS_SATURATE_LOG sets how many visits count
 * as "fully bright" — tuned so a dense curve reaches the top and the
 * chaotic sea sits comfortably in the middle. */
#define DENS_BAND_COUNT          10
#define DENS_SATURATE_LOG         8.5f   /* roughly 5000 visits = top band */

/* The famous tipping point (Greene 1979): turn K past this and the
 * last regular curve breaks, letting chaos roam the whole torus.
 * Kept here so the HUD can label the current K as below / at / above. */
#define KC_GREENE                 0.9716354f

/* A small no-man's-land around KC_GREENE that reads as "AT Kc", so
 * tiny rounding wobble doesn't make the HUD label flicker. */
#define KC_DEAD_BAND              0.01f

/* Fixed widths for the three labels on the second HUD row so they line
 * up no matter which preset or theme is showing. */
#define HUD_PARAM_PRESET_WIDTH   19
#define HUD_PARAM_THEME_WIDTH    17

/* Smallest grid we'll allow on a tiny terminal — sizes below this get
 * bumped up to stay usable; CELLS_MAX caps the big end. */
#define MAP_W_FLOOR              16
#define MAP_H_FLOOR               8

/*
 * Preset — names for the five pre-chosen settings, used as indexes into
 * the presets[] table just below.  The five K values are picked to look
 * as different as possible, so tapping 'n' walks you from perfect order
 * up through the tipping point and into full chaos.  Boot starts on
 * PRESET_WAVES (the middle), which shows interesting structure right
 * away.  What each one looks like:
 *   PRESET_INTEGRABLE : K=0.00 — no nudge at all; flat horizontal stripes.
 *   PRESET_RIPPLES    : K=0.20 — those stripes gently waved, still tidy.
 *   PRESET_WAVES      : K=0.40 — first loops ("islands") appear.
 *   PRESET_CHAOS_L    : K=2.50 — the classic fuzzy sea dotted with islands.
 *   PRESET_STORM      : K=5.00 — chaos everywhere, only tiny islands left.
 */
typedef enum {
    PRESET_INTEGRABLE = 0,
    PRESET_RIPPLES,
    PRESET_WAVES,
    PRESET_CHAOS_L,
    PRESET_STORM,
    N_PRESETS,
} Preset;

/*
 * MapPreset — one row of the preset table.  K is the standard map's
 * one and only knob, so a preset is really just a name plus a number.
 *   name : label shown in the HUD; padded to 8 chars so the column lines up.
 *   K    : how hard each step nudges the points.  Compare it to KC_GREENE
 *          to know whether chaos stays penned in (K below) or runs free
 *          across the whole torus (K above).
 */
typedef struct {
    const char *name;
    float       K;
} MapPreset;

static const MapPreset presets[N_PRESETS] = {
    { "INTEGRBL", 0.00f },   /* horizontal stripes (no kick)     */
    { "RIPPLES ", 0.20f },   /* all KAM tori, gentle wiggles     */
    { "WAVES   ", 0.40f },   /* clear island chains forming      */
    { "CHAOS_L ", 2.50f },   /* classic chaotic sea + islands    */
    { "STORM   ", 5.00f },   /* sea dominant, scattered islands  */
};

/*
 * Theme — one named colour scheme for the picture.  Each theme is just
 * a ten-step ramp running from dim (cells barely visited) to bright
 * (cells visited a lot); the family of colours is what gives the theme
 * its name.  t/T cycle through them.
 *   name   : label shown in the HUD.
 *   band[] : ten 256-colour codes, dimmest first.  density_paint asks
 *            density_band() for a level 0..9 and looks it up here.
 * All codes sit in the bright half of the palette (CLAUDE.md's
 * brightness rule) so even the dimmest band shows up on black.
 */
typedef struct {
    const char *name;
    short       band[DENS_BAND_COUNT];   /* dim to bright */
} Theme;

#define N_THEMES 10

/* The ten ramps, each climbing dim to bright left to right. */
static const Theme themes[N_THEMES] = {
    /* name        band0  b1   b2   b3   b4   b5   b6   b7   b8   b9 */
    { "DEFAULT", {  31,  39,  45,  81, 117, 153, 189, 220, 226, 231 } },
    { "MATRIX",  {  28,  34,  40,  46,  82, 118, 154, 190, 226, 228 } },
    { "NOVA",    {  93,  99, 135, 171, 207, 213, 219, 225, 227, 231 } },
    { "MONO",    { 240, 244, 247, 249, 251, 252, 253, 254, 255, 231 } },
    { "OCEAN",   {  24,  31,  38,  45,  81, 117, 159, 195, 225, 231 } },
    { "FIRE",    {  88, 124, 160, 196, 202, 208, 214, 220, 226, 228 } },
    { "EARTH",   {  94, 130, 137, 143, 178, 215, 222, 228, 230, 231 } },
    { "FOREST",  {  28,  34,  64, 100, 107, 113, 149, 185, 191, 228 } },
    { "DESERT",  { 130, 166, 173, 179, 215, 222, 228, 230, 231,  255 } },
    { "ARCTIC",  {  24,  31,  45,  51,  81, 117, 159, 195, 225, 231 } },
};

/* §2  clock */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
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

/* Load a theme's ten colours into ncurses' colour-pair slots. */
static inline void theme_install_256(const Theme *t)
{
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, t->band[i], -1);
}

/* Fallback for old terminals that only have 8 colours: spread the
 * basic ANSI colours dim to bright across the ten bands. */
static inline void theme_install_8color_fallback(void)
{
    static const short fb[DENS_BAND_COUNT] = {
        COLOR_BLUE,    COLOR_BLUE,    COLOR_CYAN,    COLOR_CYAN,
        COLOR_GREEN,   COLOR_YELLOW,  COLOR_YELLOW,  COLOR_MAGENTA,
        COLOR_RED,     COLOR_WHITE,
    };
    for (int i = 0; i < DENS_BAND_COUNT; i++)
        init_pair(PAIR_DENS_BASE + i, fb[i], -1);
}

/* Switch to theme idx, picking the rich or the fallback palette
 * depending on what the terminal can do. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) theme_install_256(&themes[idx]);
    else               theme_install_8color_fallback();
}
static void color_init(void)
{
    start_color(); use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* §5  physics — the standard map: one rule, applied over and over */

/*
 * ChirikovSystem — the part that's the same for every point: just K,
 * the strength of the nudge.  Kept apart from a point's position so the
 * step function below is a clean "old position + this K -> new position".
 *   K : how hard each step kicks.  Below KC_GREENE the wandering stays
 *       boxed in; above it, points can roam the whole torus.
 */
typedef struct { float K; } ChirikovSystem;

/*
 * ChirikovState — where one point is right now, on a doughnut-shaped
 * (torus) space where both coordinates wrap around at 2*pi.
 *   x : the angle.
 *   p : the spin / momentum.
 * Both are kept wrapped into [0, 2*pi) by wrap_tau after every step.
 */
typedef struct { float x, p; } ChirikovState;

/* Fold a number back into [0, 2*pi).  fmodf can hand back a negative
 * for negative input, so we add one full turn when that happens. */
static inline float wrap_tau(float v)
{
    v = fmodf(v, TAU);
    if (v < 0.0f) v += TAU;
    return v;
}

/* Move one point forward by a single step — the whole standard map in
 * two lines (Chirikov 1979): first the spin gets a sine-shaped nudge,
 * then the angle advances by the new spin.  Hands back the next
 * position without touching the one passed in. */
static inline ChirikovState chirikov_step(ChirikovState s, ChirikovSystem sys)
{
    s.p = wrap_tau(s.p + sys.K * sinf(s.x));
    s.x = wrap_tau(s.x + s.p);
    return s;
}

/* §6  density — a tally of how often each cell gets visited */

/*
 * DensityGrid — the picture's memory: a tally board, one counter per
 * screen cell, counting how many times the points have landed there.
 * Turning each tally into a brightness is what makes thin curves and
 * fuzzy clouds appear.  density_record bumps the counters; density_paint
 * reads them.  Sized to the terminal at startup and on resize; capped at
 * CELLS_MAX so the buffer can be a plain fixed array, no malloc.
 *   w, h  : grid size in cells.
 *   count : w*h, kept around so loops don't recompute it.
 *   hits  : the tallies, one long row-major run.  Stops climbing at
 *           65535, but that's far past "fully bright" so you never see it.
 */
typedef struct {
    int      w, h;
    int      count;
    uint16_t hits[CELLS_MAX];
} DensityGrid;

static void density_reset(DensityGrid *d, int w, int h)
{
    d->w = w; d->h = h; d->count = w * h;
    memset(d->hits, 0, sizeof(uint16_t) * (size_t)d->count);
}

/* Turn a point's (x, p) into a grid cell.  The y axis is flipped so
 * bigger p draws higher up, the way the textbooks show it. */
static inline void phase_to_cell(const DensityGrid *d,
                                 float x, float p,
                                 int *cx, int *cy)
{
    int x_cell = (int)((x / TAU) * (float)d->w);
    int y_cell = (int)((p / TAU) * (float)d->h);
    if (x_cell < 0)        x_cell = 0;
    if (x_cell >= d->w)    x_cell = d->w - 1;
    if (y_cell < 0)        y_cell = 0;
    if (y_cell >= d->h)    y_cell = d->h - 1;
    *cx = x_cell;
    *cy = d->h - 1 - y_cell;            /* flip y so up means more p */
}

/* Add one to the tally for wherever this point currently sits. */
static inline void density_record(DensityGrid *d, ChirikovState s)
{
    int cx, cy;
    phase_to_cell(d, s.x, s.p, &cx, &cy);
    int idx = cy * d->w + cx;
    if (d->hits[idx] < UINT16_MAX) d->hits[idx]++;
}

/* Turn a tally into a brightness level 0..9.  Uses a log scale so the
 * first few visits brighten fast and busy cells don't all max out.
 * Returns -1 for never-visited cells so the painter can skip them. */
static inline int density_band(uint16_t hits)
{
    if (hits == 0) return -1;
    float v = logf((float)hits) / DENS_SATURATE_LOG;
    if (v < 0.0f) v = 0.0f;
    if (v > 1.0f) v = 1.0f;
    int b = (int)(v * (float)(DENS_BAND_COUNT - 1) + 0.5f);
    if (b > DENS_BAND_COUNT - 1) b = DENS_BAND_COUNT - 1;
    return b;
}

/* §7  state — the swarm of points, plus which preset and theme are on */

/*
 * TrajectoryEnsemble — the whole swarm: every point feels the same K,
 * so K lives here once and only the positions vary.  That's the heart
 * of the demo — same rule, many starting places.  Reset on r/n/p
 * (which re-spreads the points), advanced together by scene_tick.
 *   system  : the shared K, copied in from the current preset.
 *   state[] : each point's (x, p), one struct per point.
 */
typedef struct {
    ChirikovSystem system;
    ChirikovState  state[NUM_TRAJECTORIES];
} TrajectoryEnsemble;

/* Spread the points out evenly across the torus, with a one-cell gap
 * from the edges so the corners aren't unfairly crowded.  Run on every
 * reset / preset change.  K is set separately. */
static void ensemble_init_uniform_grid(TrajectoryEnsemble *e)
{
    int idx = 0;
    for (int j = 0; j < TRAJ_GRID_H; j++) {
        float fy = (float)(j + 1) / (float)(TRAJ_GRID_H + 1);
        for (int i = 0; i < TRAJ_GRID_W; i++) {
            float fx = (float)(i + 1) / (float)(TRAJ_GRID_W + 1);
            e->state[idx].x = fx * TAU;
            e->state[idx].p = fy * TAU;
            idx++;
        }
    }
}

/*
 * PresetState — just remembers which preset is showing.  It's a struct
 * of one int on purpose: giving it its own type means the theme keys
 * can't reach in and bump the preset by mistake.
 *   current : index into presets[], always in [0, N_PRESETS).
 */
typedef struct { int current; } PresetState;

static void preset_state_init      (PresetState *p, int initial) { p->current = initial; }
static void preset_state_cycle_next(PresetState *p)              { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)              { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const MapPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — the twin of PresetState, but for the colour theme.
 * Separate type for the same reason: the preset keys can't accidentally
 * change the theme.  After a t/T cycle, scene_apply_theme reloads the
 * colours so the next frame uses them.
 *   current : index into themes[], always in [0, N_THEMES).
 */
typedef struct { int current; } PaletteState;

static void palette_state_init      (PaletteState *p, int initial) { p->current = initial; }
static void palette_state_cycle_next(PaletteState *p)              { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p)              { p->current = (p->current + N_THEMES - 1) % N_THEMES; }
static const Theme *palette_state_active(const PaletteState *p)    { return &themes[p->current]; }
static void palette_state_apply     (const PaletteState *p)        { theme_apply(p->current); }

/* §8  scene — everything that changes, gathered in one place */

/*
 * Scene — the whole running simulation in one struct, so the main loop
 * can drive it through a handful of named calls.  Each piece keeps its
 * own type, so the physics, the picture, and the menu choices stay
 * cleanly separate.  Freezes once every point has run its full budget.
 *   density          : the tally board / picture (§6).
 *   ensemble         : the swarm of points and their shared K (§7).
 *   preset           : which K-preset is selected.
 *   palette          : which colour theme is selected.
 *   paused           : true means scene_tick does nothing.
 *   iterations_done  : steps taken per point; at MAX_ITERATIONS_PER_TRAJ
 *                      the picture is done and ticking stops.
 */
typedef struct {
    DensityGrid        density;
    TrajectoryEnsemble ensemble;
    PresetState        preset;
    PaletteState       palette;
    bool               paused;
    int                iterations_done;
} Scene;

/* Short-hands so callers don't dig through preset.current every time. */
static inline const MapPreset *scene_active_preset(const Scene *s)
{
    return preset_state_active(&s->preset);
}
static void scene_apply_theme(const Scene *s)
{
    palette_state_apply(&s->palette);
}

/* Copy the chosen preset's K onto the swarm, so the next steps use it.
 * Done on reset and whenever the preset changes. */
static void scene_load_active_preset_into_system(Scene *s)
{
    s->ensemble.system.K = scene_active_preset(s)->K;
}

static void scene_reset(Scene *s, int w, int h)
{
    density_reset(&s->density, w, h);
    ensemble_init_uniform_grid(&s->ensemble);
    scene_load_active_preset_into_system(s);
    s->iterations_done = 0;
}
static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    preset_state_init (&s->preset,  PRESET_WAVES);   /* start mid-cascade */
    palette_state_init(&s->palette, 0);              /* DEFAULT theme */
    scene_reset(s, w, h);
}

/* True once every point has used up its step budget — the picture is
 * done, so there's no reason to keep going. */
static inline bool scene_is_settled(const Scene *s)
{
    return s->iterations_done >= MAX_ITERATIONS_PER_TRAJ;
}

/* Advance every point by one step and tally where each lands. */
static inline void ensemble_advance_one_step(TrajectoryEnsemble *e,
                                             DensityGrid *d)
{
    for (int i = 0; i < NUM_TRAJECTORIES; i++) {
        e->state[i] = chirikov_step(e->state[i], e->system);
        density_record(d, e->state[i]);
    }
}

/* One tick of the simulation: take a batch of steps unless we're paused
 * or already done. */
static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    if (scene_is_settled(s)) return;

    for (int step = 0; step < ITERATES_PER_TRAJ_PER_TICK; step++)
        ensemble_advance_one_step(&s->ensemble, &s->density);

    s->iterations_done += ITERATES_PER_TRAJ_PER_TICK;
}

/* §9  screen */

/*
 * Screen — just the current terminal size, kept in one spot.  ncurses
 * has these as globals; copying them here means the drawing code has a
 * single handle to read and a resize only has to update one place.
 *   cols : width  in characters.
 *   rows : height in characters.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s)   { (void)s; endwin(); }
static void screen_resize(Screen *s) { endwin(); refresh();
                                       getmaxyx(stdscr, s->rows, s->cols); }
static void screen_present(void)     { wnoutrefresh(stdscr); doupdate(); }

/* Pick the character for a brightness level: faint dots for dim cells
 * climbing to a solid '@' for the brightest.  The shape carries part of
 * the brightness and the colour carries the rest. */
static char glyph_for_band(int b)
{
    static const char tiers[DENS_BAND_COUNT] = {
        '.', ',', ':', ';', '+', 'o', '*', 'X', '#', '@'
    };
    if (b < 0)                   return ' ';
    if (b >= DENS_BAND_COUNT)    return '@';
    return tiers[b];
}

/* Which colour-pair slot holds the colour for a brightness level.
 * One place does this, so the painter and the themes can't drift apart. */
static inline short density_band_to_pair(int band)
{
    return (short)(PAIR_DENS_BASE + band);
}

/* Where the top-left of the grid goes: centred in the space left
 * between the top and bottom HUD rows, and nudged so it never overlaps
 * them. */
static inline void density_grid_origin(const DensityGrid *d,
                                       int cols, int rows,
                                       int *gx0, int *gy0)
{
    int x = (cols - d->w) / 2;
    int y = ((rows - HUD_BAND_RESERVED_ROWS) - d->h) / 2 + HUD_TOP_ROWS;
    if (x < 0)              x = 0;
    if (y < HUD_TOP_ROWS)   y = HUD_TOP_ROWS;
    *gx0 = x;
    *gy0 = y;
}

/* True if this screen cell is inside the drawing area, i.e. clear of
 * both the top and bottom HUD rows. */
static inline bool density_cell_in_drawable_band(int sx, int sy,
                                                 int cols, int rows)
{
    return sx >= 0 && sx < cols
        && sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* Draw one cell.  Untouched cells (level below 0) are left blank so the
 * background shows through. */
static inline void paint_density_cell(int sy, int sx, int band)
{
    if (band < 0) return;
    short pair  = density_band_to_pair(band);
    char  glyph = glyph_for_band(band);
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the whole grid: walk every cell, turn its tally into a
 * brightness, and paint it. */
static void density_paint(const DensityGrid *d, int cols, int rows)
{
    int gx0, gy0;
    density_grid_origin(d, cols, rows, &gx0, &gy0);

    for (int y = 0; y < d->h; y++) {
        int sy = gy0 + y;
        for (int x = 0; x < d->w; x++) {
            int sx = gx0 + x;
            if (!density_cell_in_drawable_band(sx, sy, cols, rows)) continue;
            int band = density_band(d->hits[y * d->w + x]);
            paint_density_cell(sy, sx, band);
        }
    }
}

/* The word shown next to the preset counter: what the sim is doing
 * right now — paused, done, or running (then it shows the preset name). */
static inline const char *hud_status_word(const Scene *s)
{
    if (s->paused)            return "PAUSED  ";
    if (scene_is_settled(s))  return "DONE    ";
    return scene_active_preset(s)->name;
}

/* How far along the picture is, 0..100%.  Clamped at 100 because a
 * single batch of steps can nudge the count just past the cap. */
static inline int hud_iteration_percent(const Scene *s)
{
    int pct = (s->iterations_done * 100) / MAX_ITERATIONS_PER_TRAJ;
    if (pct > 100) pct = 100;
    return pct;
}

/* Says whether the current K is below, at, or above the tipping point. */
static inline const char *regime_label_for_K(float K)
{
    if (K < KC_GREENE - KC_DEAD_BAND) return "BELOW Kc";
    if (K > KC_GREENE + KC_DEAD_BAND) return "ABOVE Kc";
    return "AT Kc   ";
}

static inline void hud_write_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " STANDARD MAP (Chirikov) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static inline void hud_write_status_right(int cols, double fps, int sim_fps,
                                          const Scene *s)
{
    const MapPreset *active = scene_active_preset(s);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  K:%.4f  %3d%% ",
             fps, sim_fps, hud_status_word(s),
             s->preset.current + 1, (int)N_PRESETS,
             (double)active->K, hud_iteration_percent(s));
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_top_left_title(void) { hud_write_title(); }

static void hud_top_right_status(int cols, double fps, int sim_fps,
                                 const Scene *s)
{
    hud_write_status_right(cols, fps, sim_fps, s);
}

/* The three labels on the second HUD row: preset, theme, and a
 * below/at/above-Kc readout. */
static inline void hud_write_preset_label(int x, const MapPreset *active)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " preset:%-8s ", active->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_theme_label(int x, const Scene *s)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", palette_state_active(&s->palette)->name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}
static inline void hud_write_threshold_segment(int x, float K)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " trajs:%d  %s  Kc=%.4f ",
             NUM_TRAJECTORIES, regime_label_for_K(K), (double)KC_GREENE);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Lay out the three second-row labels side by side. */
static void hud_param_row(const Scene *s)
{
    const MapPreset *active = scene_active_preset(s);
    int x = HUD_LEFT_MARGIN;

    hud_write_preset_label   (x, active); x += HUD_PARAM_PRESET_WIDTH;
    hud_write_theme_label    (x, s);      x += HUD_PARAM_THEME_WIDTH;
    hud_write_threshold_segment(x, active->K);
}

static void hud_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Draw one whole frame: clear, paint the picture, then the HUD. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    density_paint(&s->density, sc->cols, sc->rows);
    hud_top_left_title();
    hud_top_right_status(sc->cols, fps, sim_fps, s);
    hud_param_row(s);
    hud_bottom_hint(sc->rows);
}

/* §10 app */

/*
 * App — the top-level box holding everything: the simulation, the
 * terminal size, the speed knob, and the two flags the signal handlers
 * set.  It lives in one file-scope variable so those handlers can reach
 * it directly.  main() never pokes the fields itself — it always goes
 * through a named helper.
 *   scene       : the whole simulation (§8).
 *   screen      : the terminal size (§9).
 *   sim_fps     : how many ticks a second to run, kept in [10, 240].
 *   map_w/h     : grid size, kept separate from the screen size so it can
 *                 be capped at CELLS_MAX without distorting the layout.
 *   running     : the main loop runs while this is set; cleared by 'q',
 *                 ESC, or Ctrl-C.
 *   need_resize : set when the terminal is resized, handled at the top of
 *                 the next loop pass.  Both flags are volatile sig_atomic_t
 *                 because a signal handler writes them while the loop reads.
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

/* Hook up the signals: Ctrl-C / kill stop the loop, a resize sets the flag. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Size the grid to fill the terminal minus the HUD rows.  The floors
 * keep it usable on tiny windows; the ceilings stop a huge window from
 * overrunning the fixed buffer. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < MAP_W_FLOOR) mw = MAP_W_FLOOR;
    if (mh < MAP_H_FLOOR) mh = MAP_H_FLOOR;
    if (mw > MAP_W_MAX)   mw = MAP_W_MAX;
    if (mh > MAP_H_MAX)   mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* Start everything up: ncurses, the grid size, and the scene. */
static void app_bootstrap(App *app)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* If the terminal was resized, rebuild the screen and start the picture
 * over at the new size. */
static void app_handle_pending_resize(App *app)
{
    if (!app->need_resize) return;
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* How long since the last frame, capped so that one long pause (say the
 * window was dragged) doesn't make the sim try to catch up all at once. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > SIM_MAX_FRAME_DT_MS * NS_PER_MS)
        dt = SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    return dt;
}

/* Run the sim at a steady rate no matter how fast we draw: spend the
 * banked-up time one fixed tick at a time. */
static void app_drain_fixed_timestep(App *app, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recompute the shown fps every half second so the number doesn't jitter. */
static void app_update_fps_meter(int64_t *fps_accum, int *frame_count,
                                 double *fps_display)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_display = (double)*frame_count
                 / ((double)*fps_accum / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
}

/* Sleep off whatever's left of this frame's time budget so we hold 60 fps. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the frame and show it. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Speed up / slow down the sim, staying within the allowed range. */
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

/* The little actions each key triggers, named so the key table reads clearly. */
static void app_toggle_pause     (App *app) { app->scene.paused = !app->scene.paused; }
static void app_reset_picture    (App *app) { scene_reset(&app->scene, app->map_w, app->map_h); }
static void app_cycle_theme_next (App *app) { palette_state_cycle_next(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_theme_prev (App *app) { palette_state_cycle_prev(&app->scene.palette); scene_apply_theme(&app->scene); }
static void app_cycle_preset_next(App *app)
{
    preset_state_cycle_next(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}
static void app_cycle_preset_prev(App *app)
{
    preset_state_cycle_prev(&app->scene.preset);
    scene_reset(&app->scene, app->map_w, app->map_h);
}

static bool app_handle_key(App *app, int ch);

/* Check for a keypress (without blocking) and act on it; false = quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* The key table: each key runs one of the little actions above. */
static bool app_handle_key(App *app, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            app_toggle_pause     (app); break;
    case 'r': case 'R':  app_reset_picture    (app); break;
    case ']':            app_sim_rate_faster  (app); break;
    case '[':            app_sim_rate_slower  (app); break;
    case 't':            app_cycle_theme_next (app); break;
    case 'T':            app_cycle_theme_prev (app); break;
    case 'n': case 'N':  app_cycle_preset_next(app); break;
    case 'p': case 'P':  app_cycle_preset_prev(app); break;
    default: break;
    }
    return true;
}

/*
 * FrameClock — the loop's stopwatch and bookkeeping, in one place so
 * main() reads as a short list of steps.  It's a loop-only helper, not
 * part of App.
 *   frame_time  : when the previous frame started; used to measure dt.
 *   sim_accum   : sim-time we owe but haven't run yet, paid off one tick
 *                 at a time — this is what keeps the sim rate steady.
 *   fps_accum   : time piled up since we last figured the fps.
 *   frame_count : frames drawn since we last figured the fps.
 *   fps_display : the fps number on screen, refreshed twice a second.
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

/* The whole program, top to bottom: set up, then loop — handle a resize,
 * measure the frame, run the sim, draw, wait, read a key — until quit. */
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
