/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cobweb_diagram.c — draws the "staircase" picture that shows how repeatedly
 * feeding a number through f(x) = r*x*(1-x) settles down, cycles, or goes wild.
 * 30 presets walk r from calm to chaotic. Sister files: bifurcation.c (all r at
 * once) and sensitive_dependence.c (how two nearby starts drift apart).
 * Background: May 1976 (Nature 261); Feigenbaum 1978; Strogatz, Nonlinear Dynamics ch.10.
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* §1 config */

enum {
    SIM_FPS_MIN          =   1,
    SIM_FPS_DEFAULT      =   8,    /* slow on purpose so each step is watchable */
    SIM_FPS_MAX          =  60,
    SIM_FPS_STEP         =   2,

    HUD_COLS             =  80,
    FPS_UPDATE_MS        = 500,

    PAIR_HUD             =   1,
    PAIR_HINT            =   2,
    PAIR_CURVE           =   3,    /* the curve f(x)            */
    PAIR_DIAG            =   4,    /* the diagonal line y = x   */
    PAIR_COBWEB_BASE     =   5,    /* 4 fade shades, oldest..newest */
    PAIR_LIVE            =  10,    /* the live "you are here" marker */
};

#define HUD_TOP_ROWS              2
#define HUD_BOTTOM_ROWS           1
#define HUD_BAND_RESERVED_ROWS    (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN           1

#define NS_PER_SEC                1000000000LL
#define NS_PER_MS                    1000000LL
#define TICK_NS(f)                (NS_PER_SEC / (f))
#define RENDER_FPS_TARGET         60
#define RENDER_FRAME_BUDGET_NS    (NS_PER_SEC / RENDER_FPS_TARGET)
#define SIM_MAX_FRAME_DT_MS       100

#define COBWEB_AGE_BANDS           4
#define COBWEB_MAX              1024   /* how many staircase strokes we keep (2 per step) */
#define ITER_STEPS_PER_TICK        1   /* steps taken each tick */

/*
 * CobwebPreset — one stop on the tour from calm to chaotic.
 *
 * The number r is the only thing that decides how the logistic map behaves, so
 * we hand-pick 30 interesting r values and let the user step through them.
 * Everything starts from the same x0; r is the one knob that moves.
 * The presets[] table just below holds the rows; preset_state_active() reads one.
 *
 *   name : short label shown in the HUD. Padded to 8 chars so the HUD lines up.
 *   r    : the knob, between 1 and 4. Past about 3.5699 things turn chaotic, with
 *          brief calm "windows" popping up at certain r values.
 *   x0   : where iteration starts. Always 0.10 here, since for any start between
 *          0 and 1 you end up at the same long-run behaviour. Only r matters.
 */
typedef struct { const char *name; float r; float x0; } CobwebPreset;

/*
 * Preset — a name for each of the 30 rows in the presets[] table.
 *
 * Naming the rows lets the code (and the user, cycling with n/p) walk them in a
 * meaningful order: settle-to-one-value, then a value flipping between two, four,
 * eight..., then chaos broken up by brief calm spells. They group into three
 * families by r value:
 *   Class A : r between 1 and 3      — settles to a single value
 *   Class B : r between 3 and 3.5699 — the value splits 2, 4, 8, 16...
 *   Class C : r above 3.5699 to 4    — chaos, with occasional calm windows
 * Used only to index presets[]; the [N_PRESETS] size lets the compiler catch a
 * missing row. See Feigenbaum 1978 (why the splitting happens) and Sharkovskii
 * 1964 (why a 3-cycle, P3-WIND, guarantees every other cycle exists).
 */
typedef enum {
    /* Class A — settles to a single value (r between 1 and 3) */
    PRESET_FP_SLOW = 0,    /* r=1.500 — slow approach to x* ≈ 0.333          */
    PRESET_FP_MID,         /* r=2.000 — fast approach to x* = 0.500          */
    PRESET_FP_FAST,        /* r=2.500 — fastest approach (slope ≈ 0)         */
    PRESET_FP_SPIRL,       /* r=2.800 — oscillatory spiral to x* ≈ 0.643     */
    PRESET_FP_EDGE,        /* r=2.950 — near loss of stability               */

    /* Class B — the value splits 2, 4, 8, 16... */
    PRESET_P2_BORN,        /* r=3.050 — 2-cycle just born                    */
    PRESET_P2_MID,         /* r=3.200 — established 2-cycle                  */
    PRESET_P2_WIDE,        /* r=3.400 — wide 2-cycle, near next bifurcation  */
    PRESET_P4_BORN,        /* r=3.460 — 4-cycle                              */
    PRESET_P4_MID,         /* r=3.500 — established 4-cycle                  */
    PRESET_P8_BORN,        /* r=3.550 — 8-cycle                              */
    PRESET_P16_BORN,       /* r=3.566 — 16-cycle (barely distinguishable)    */
    PRESET_FEIGNBM,        /* r=3.5699 — Feigenbaum accumulation point r∞    */

    /* Class C — chaos, broken up by brief calm windows */
    PRESET_CHAOS_A,        /* r=3.575 — narrow chaotic bands                 */
    PRESET_CHAOS_B,        /* r=3.600 — band-merged chaos                    */
    PRESET_P6_WIND,        /* r=3.626 — period-6 window inside chaos         */
    PRESET_CHAOS_C,        /* r=3.650 — typical chaos                        */
    PRESET_P7_WIND,        /* r=3.701 — period-7 window                      */
    PRESET_P5_WIND,        /* r=3.739 — period-5 window (the widest)         */
    PRESET_INTERMTT,       /* r=3.752 — intermittency past the p-5 edge      */
    PRESET_CHAOS_D,        /* r=3.780 — post-p-5 chaos                       */
    PRESET_TANGENT,        /* r=3.828 — tangent bifurcation, just before p-3 */
    PRESET_P3_WIND,        /* r=3.835 — famous period-3 (Li-Yorke 1975)      */
    PRESET_P6_IN_P3,       /* r=3.844 — period-doubled inside the p-3 window */
    PRESET_CRISIS,         /* r=3.857 — interior crisis ending the p-3 window*/
    PRESET_CHAOS_E,        /* r=3.880 — post-p-3 chaos                       */
    PRESET_CHAOS_F,        /* r=3.920 — strong chaos                         */
    PRESET_CHAOS_G,        /* r=3.950 — wide chaos                           */
    PRESET_EDGE_MAX,       /* r=3.990 — near the edge                        */
    PRESET_FULL_MAX,       /* r=4.000 — full ergodic chaos on [0, 1]         */

    N_PRESETS,
} Preset;

/* 30 r-values from calm to chaotic, all starting at the same x0. */
static const CobwebPreset presets[N_PRESETS] = {
    /* Class A — settles to one value */
    { "FP-SLOW ", 1.500f,  0.10f },
    { "FP-MID  ", 2.000f,  0.10f },
    { "FP-FAST ", 2.500f,  0.10f },
    { "FP-SPIRL", 2.800f,  0.10f },
    { "FP-EDGE ", 2.950f,  0.10f },

    /* Class B — the value splits 2, 4, 8, 16... */
    { "P2-BORN ", 3.050f,  0.10f },
    { "P2-MID  ", 3.200f,  0.10f },
    { "P2-WIDE ", 3.400f,  0.10f },
    { "P4-BORN ", 3.460f,  0.10f },
    { "P4-MID  ", 3.500f,  0.10f },
    { "P8-BORN ", 3.550f,  0.10f },
    { "P16-BORN", 3.566f,  0.10f },
    { "FEIGNBM ", 3.5699f, 0.10f },

    /* Class C — chaos, broken up by brief calm windows */
    { "CHAOS-A ", 3.575f,  0.10f },
    { "CHAOS-B ", 3.600f,  0.10f },
    { "P6-WIND ", 3.626f,  0.10f },
    { "CHAOS-C ", 3.650f,  0.10f },
    { "P7-WIND ", 3.701f,  0.10f },
    { "P5-WIND ", 3.739f,  0.10f },
    { "INTERMTT", 3.752f,  0.10f },
    { "CHAOS-D ", 3.780f,  0.10f },
    { "TANGENT ", 3.828f,  0.10f },
    { "P3-WIND ", 3.835f,  0.10f },
    { "P6-IN-P3", 3.844f,  0.10f },
    { "CRISIS  ", 3.857f,  0.10f },
    { "CHAOS-E ", 3.880f,  0.10f },
    { "CHAOS-F ", 3.920f,  0.10f },
    { "CHAOS-G ", 3.950f,  0.10f },
    { "EDGE-MAX", 3.990f,  0.10f },
    { "FULL-MAX", 4.000f,  0.10f },
};

/*
 * Theme — the set of colours for one named look.
 *
 * The drawing code never names a colour directly; it draws through slots like
 * PAIR_CURVE or PAIR_LIVE. theme_apply() points those slots at the colour
 * numbers stored here, so switching themes (t/T) just repoints the slots and
 * nothing else has to change. The active row's index lives in PaletteState.
 * Colour numbers are xterm-256 indices.
 *
 *   curve : the curve f(x).
 *   diag  : the diagonal line y = x.
 *   age[] : 4 shades for the staircase, oldest (dim) to newest (bright). All
 *           kept bright enough to stay visible on a black background.
 *   live  : the live "you are here" marker, a bright accent that stands out.
 */
typedef struct {
    const char *name;
    short       curve;
    short       diag;
    short       age[COBWEB_AGE_BANDS];
    short       live;
} Theme;

#define N_THEMES 10
static const Theme themes[N_THEMES] = {
    { "DEFAULT", 220, 244, {  75, 117, 220, 231 }, 196 },
    { "MATRIX",  118, 244, {  77, 118, 156, 194 }, 226 },
    { "NOVA",    207, 244, { 135, 171, 207, 219 }, 226 },
    { "MONO",    250, 240, { 247, 250, 253, 255 }, 226 },
    { "OCEAN",   159, 244, {  81, 117, 159, 195 }, 226 },
    { "FIRE",    220, 244, { 208, 214, 220, 227 }, 231 },
    { "EARTH",   222, 244, { 143, 179, 215, 222 }, 196 },
    { "FOREST",  150, 244, { 114, 150, 157, 194 }, 226 },
    { "DESERT",  222, 244, { 179, 215, 222, 229 }, 196 },
    { "ARCTIC",  195, 244, { 117, 159, 195, 231 }, 196 },
};

/* §2 clock */

static int64_t clock_ns(void)
{
    struct timespec t; clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { ns / NS_PER_SEC, ns % NS_PER_SEC };
    nanosleep(&req, NULL);
}

/* §3 color */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    if (COLORS >= 256) {
        init_pair(PAIR_CURVE, t->curve, -1);
        init_pair(PAIR_DIAG,  t->diag,  -1);
        init_pair(PAIR_LIVE,  t->live,  -1);
        for (int i = 0; i < COBWEB_AGE_BANDS; i++)
            init_pair(PAIR_COBWEB_BASE + i, t->age[i], -1);
    } else {
        init_pair(PAIR_CURVE, COLOR_YELLOW, -1);
        init_pair(PAIR_DIAG,  COLOR_WHITE,  -1);
        init_pair(PAIR_LIVE,  COLOR_RED,    -1);
        for (int i = 0; i < COBWEB_AGE_BANDS; i++)
            init_pair(PAIR_COBWEB_BASE + i, COLOR_CYAN, -1);
    }
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

/* §5 map — the function f(x) and where we currently are in feeding it */

/*
 * LogisticMap — the function f(x) = r * x * (1 - x), boxed up with its r.
 *
 * Just one number, r. Everything the picture does (settle, cycle, go chaotic)
 * comes from changing that one number. We wrap it in a struct so the function
 * and its r always travel together, and so there's a home if we ever want to
 * cache more per-function data.
 *
 *   r : the knob, between 1 and 4. It settles to one value up to r=3, starts
 *       splitting (2, 4, 8...) after that, and turns chaotic past about 3.5699.
 *       See May 1976 and Feigenbaum 1978.
 */
typedef struct {
    float r;
} LogisticMap;

/* run x through the function once: r * x * (1 - x). */
static inline float logistic_map_eval(const LogisticMap *m, float x)
{
    return m->r * x * (1.0f - x);
}

/*
 * MapIterator — where we are right now in feeding x through the function.
 *
 * The function is one thing; the running value we keep pushing through it is
 * another. Keeping them apart matches how the textbooks talk (the "map" vs the
 * "orbit", the sequence x, f(x), f(f(x)), ...). Lives on Scene, reset on boot /
 * r-press / preset change, advanced once per tick, read to place the '@' marker.
 *
 *   map   : the function being iterated (held by value; it's tiny).
 *   x     : the current value, always between 0 and 1 for r in [1, 4].
 *   steps : how many times we've stepped since the last reset (shown in the HUD).
 */
typedef struct {
    LogisticMap map;
    float       x;
    int         steps;
} MapIterator;

static void map_iterator_init(MapIterator *it, float r, float x0)
{
    it->map.r = r;
    it->x     = x0;
    it->steps = 0;
}

/* take one step: replace x with f(x) and hand back the new value. */
static float map_iterator_step(MapIterator *it)
{
    float x_new = logistic_map_eval(&it->map, it->x);
    it->x = x_new;
    it->steps++;
    return x_new;
}

/* §6 cobweb — the staircase strokes we draw and remember */

/*
 * CobwebSegKind — which of the two strokes a step draws.
 *
 *   SEG_VERT : the up/down stroke, from the diagonal to the curve.
 *   SEG_HORZ : the sideways stroke, from the curve back to the diagonal.
 *
 * We tag each stroke instead of guessing its direction from the points, so the
 * drawing code can just pick '|' or '-' without comparing floats for equality.
 */
typedef enum { SEG_VERT = 0, SEG_HORZ } CobwebSegKind;

/*
 * CobwebSeg — one straight stroke of the staircase.
 *
 * The staircase is just a chain of strokes that alternate up-and-down then
 * sideways. We store each as a start point and an end point, measured 0..1 in
 * both directions (not in screen cells), so the same stored data draws correctly
 * at any window size. The kind tag says whether it's an up/down or sideways stroke.
 *
 *   x0, y0 : where the stroke starts (0..1 in each direction).
 *   x1, y1 : where it ends.
 *   kind   : up/down (SEG_VERT) or sideways (SEG_HORZ).
 */
typedef struct {
    float          x0, y0, x1, y1;
    CobwebSegKind  kind;
} CobwebSeg;

/*
 * CobwebRing — a fixed-size ring that keeps the most recent strokes.
 *
 * Every step adds two strokes (one up/down, one sideways). We hang onto the last
 * COBWEB_MAX of them and fade older ones, so the trail dims as it recedes. Fixed
 * size means no memory is ever allocated while running — it all sits in one
 * preallocated block (about 20 KB). When the ring fills, the newest stroke
 * overwrites the oldest. (Standard circular buffer; see Knuth TAOCP Vol 1.)
 *
 *   seg   : the strokes. Oldest is at (head - count + 1) wrapped around; newest at head.
 *   head  : index of the newest stroke; steps forward and wraps around.
 *   count : how many strokes are valid, up to COBWEB_MAX, since the last reset.
 */
typedef struct {
    CobwebSeg seg[COBWEB_MAX];
    int       head;
    int       count;
} CobwebRing;

static void cobweb_ring_reset(CobwebRing *c) { c->head = 0; c->count = 0; }

static void cobweb_ring_push(CobwebRing *c, float x0, float y0,
                             float x1, float y1, CobwebSegKind k)
{
    c->head = (c->head + 1) % COBWEB_MAX;
    c->seg[c->head] = (CobwebSeg){ x0, y0, x1, y1, k };
    if (c->count < COBWEB_MAX) c->count++;
}

/* pick a fade shade (0 = oldest/dim) for a stroke that is `age` strokes back
 * out of `n` kept. */
static inline int cobweb_ring_age_band(int age, int n)
{
    int band = (COBWEB_AGE_BANDS - 1) - (age * COBWEB_AGE_BANDS) / n;
    if (band < 0)                        band = 0;
    if (band > COBWEB_AGE_BANDS - 1)     band = COBWEB_AGE_BANDS - 1;
    return band;
}

/* §7 viewport — turn 0..1 values into screen cells */

/*
 * UnitViewport — the rule for placing a 0..1 value onto the screen grid.
 *
 * The math lives in a 0..1 by 0..1 square; the terminal draws on a grid of
 * whole cells. This holds the box on screen we draw into and does the stretch,
 * so no drawing code has to redo it. We flip the vertical direction so 0 sits at
 * the bottom, the way these plots are always drawn in textbooks. It's recomputed
 * once per frame from the current window size and only read after that.
 *
 *   gx0, gy0 : top-left cell of the area we draw into.
 *   w, h     : its width and height in cells, with the HUD rows left out so we
 *              never draw over the dashboard.
 */
typedef struct {
    int gx0, gy0;
    int w, h;
} UnitViewport;

static void viewport_compute(UnitViewport *v, int cols, int rows)
{
    v->gx0 = 0;
    v->gy0 = HUD_TOP_ROWS;
    v->w   = cols;
    v->h   = rows - HUD_BAND_RESERVED_ROWS;
}

/* 0..1 across -> which column. */
static inline int viewport_u_to_cx(const UnitViewport *v, float u)
{ return v->gx0 + (int)(u * (float)(v->w - 1)); }

/* 0..1 up -> which row, flipped so 0 is at the bottom. */
static inline int viewport_u_to_cy(const UnitViewport *v, float u)
{ return v->gy0 + (v->h - 1) - (int)(u * (float)(v->h - 1)); }

/* §8 scene — which preset, which theme, and all the live state */

/*
 * PresetState — remembers which of the 30 presets is showing.
 *
 * It's just one number, but wrapping it gives "which preset" a clear home on
 * Scene and lets the n/p keys read as cycle_next + load instead of fiddling
 * with raw index math. Read by scene_load_preset (applies that row's r and x0)
 * and by the HUD.
 *
 *   current : row in presets[], 0..N_PRESETS-1. The cycle helpers wrap around so
 *             it never goes out of range.
 */
typedef struct { int current; } PresetState;

static void preset_state_init(PresetState *p)        { p->current = PRESET_FP_SPIRL; }
static void preset_state_cycle_next(PresetState *p)  { p->current = (p->current + 1) % N_PRESETS; }
static void preset_state_cycle_prev(PresetState *p)  { p->current = (p->current + N_PRESETS - 1) % N_PRESETS; }
static const CobwebPreset *preset_state_active(const PresetState *p) { return &presets[p->current]; }

/*
 * PaletteState — remembers which theme is active.
 *
 * Same idea as PresetState: one wrapped number so the t/T keys stay tidy. Note
 * this struct only holds the index; the actual colours don't change until
 * theme_apply() is called, so callers must call it right after changing this.
 *
 *   current : row in themes[], 0..N_THEMES-1. theme_apply() falls back to 0 if
 *             it ever gets something out of range.
 */
typedef struct { int current; } PaletteState;

static void palette_state_init(PaletteState *p)       { p->current = 0; }
static void palette_state_cycle_next(PaletteState *p) { p->current = (p->current + 1) % N_THEMES; }
static void palette_state_cycle_prev(PaletteState *p) { p->current = (p->current + N_THEMES - 1) % N_THEMES; }

/*
 * Scene — holds everything that changes while running.
 *
 * One tick flows through these in order: the preset says which r to use; the
 * iterator takes one step with that r; the step adds two strokes to the ring;
 * the drawing code later reads the ring (in the chosen theme) to paint it.
 * There's one Scene, owned by App. Only the main loop touches it — the signal
 * handlers just flip flags on App and let the loop react next frame.
 *
 *   iter    : where we are in iterating the function.
 *   cobweb  : the fading staircase strokes behind the live marker.
 *   preset  : which r is selected.
 *   palette : which theme is selected.
 *   paused  : when true, the simulation freezes but drawing continues, so you
 *             see a held still frame. Toggled by space.
 */
typedef struct {
    MapIterator  iter;
    CobwebRing   cobweb;
    PresetState  preset;
    PaletteState palette;
    bool         paused;
} Scene;

static void scene_load_preset(Scene *s)
{
    const CobwebPreset *p = preset_state_active(&s->preset);
    map_iterator_init(&s->iter, p->r, p->x0);
    cobweb_ring_reset(&s->cobweb);
}

static void scene_reset(Scene *s) { scene_load_preset(s); }

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    preset_state_init(&s->preset);
    palette_state_init(&s->palette);
    s->paused = false;
    scene_load_preset(s);
}

static void scene_tick(Scene *s, float dt)
{
    (void)dt;
    if (s->paused) return;
    for (int k = 0; k < ITER_STEPS_PER_TICK; k++) {
        float x_old = s->iter.x;
        float x_new = map_iterator_step(&s->iter);
        /* go up from the diagonal to the curve */
        cobweb_ring_push(&s->cobweb, x_old, x_old, x_old, x_new, SEG_VERT);
        /* go sideways back to the diagonal at the new value */
        cobweb_ring_push(&s->cobweb, x_old, x_new, x_new, x_new, SEG_HORZ);
    }
}

/* §9 screen — the window size plus everything that draws */

/*
 * Screen — remembers the current window size in cells.
 *
 * ncurses owns the terminal; we just cache how wide and tall it is so the
 * drawing code doesn't have to ask ncurses on every single paint. Refreshed at
 * startup and whenever the window is resized.
 *
 *   cols : width in cells.
 *   rows : height in cells, HUD bar included (drawing code subtracts the HUD rows
 *          when it wants just the picture area).
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

/* painters */

/* true if a cell is inside the picture area, so we never draw over the HUD. */
static inline bool in_drawable(int sx, int sy, int cols, int rows)
{
    return sx >= 0 && sx < cols && sy >= HUD_TOP_ROWS && sy < rows - HUD_BOTTOM_ROWS;
}

/* draw the curve f(x) as a row of dots, one per column. It's the fixed backdrop
 * the staircase climbs against. */
static void paint_logistic_curve(const UnitViewport *v, const LogisticMap *m,
                                 int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_CURVE) | A_BOLD);
    for (int x = 0; x < v->w; x++) {
        float u = (float)x / (float)(v->w - 1);
        float y = logistic_map_eval(m, u);
        int sx = v->gx0 + x;
        int sy = viewport_u_to_cy(v, y);
        if (in_drawable(sx, sy, cols, rows)) mvaddch(sy, sx, '.');
    }
    attroff(COLOR_PAIR(PAIR_CURVE) | A_BOLD);
}

/* draw the diagonal line y = x, the other fixed backdrop. */
static void paint_diagonal(const UnitViewport *v, int cols, int rows)
{
    attron(COLOR_PAIR(PAIR_DIAG));
    for (int x = 0; x < v->w; x++) {
        float u  = (float)x / (float)(v->w - 1);
        int   sx = v->gx0 + x;
        int   sy = viewport_u_to_cy(v, u);
        if (in_drawable(sx, sy, cols, rows)) mvaddch(sy, sx, '/');
    }
    attroff(COLOR_PAIR(PAIR_DIAG));
}

/* draw one stroke: '|' if up/down, '-' if sideways, in the given fade shade. */
static void paint_cobweb_segment(const CobwebSeg *s, int band,
                                 const UnitViewport *v, int cols, int rows)
{
    int pair = PAIR_COBWEB_BASE + band;
    attron(COLOR_PAIR(pair) | A_BOLD);
    if (s->kind == SEG_VERT) {
        int sx = viewport_u_to_cx(v, s->x0);
        int ya = viewport_u_to_cy(v, s->y0);
        int yb = viewport_u_to_cy(v, s->y1);
        int lo = ya < yb ? ya : yb;
        int hi = ya < yb ? yb : ya;
        for (int y = lo; y <= hi; y++)
            if (in_drawable(sx, y, cols, rows)) mvaddch(y, sx, '|');
    } else {
        int sy = viewport_u_to_cy(v, s->y0);
        int xa = viewport_u_to_cx(v, s->x0);
        int xb = viewport_u_to_cx(v, s->x1);
        int lo = xa < xb ? xa : xb;
        int hi = xa < xb ? xb : xa;
        for (int x = lo; x <= hi; x++)
            if (in_drawable(x, sy, cols, rows)) mvaddch(sy, x, '-');
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* draw the whole staircase, oldest first, each stroke dimmer the older it is. */
static void paint_cobweb(const CobwebRing *c, const UnitViewport *v,
                         int cols, int rows)
{
    if (c->count == 0) return;
    int n      = c->count;
    int oldest = (c->head - n + 1 + COBWEB_MAX) % COBWEB_MAX;
    for (int i = 0; i < n; i++) {
        int idx  = (oldest + i) % COBWEB_MAX;
        int age  = n - 1 - i;
        int band = cobweb_ring_age_band(age, n);
        paint_cobweb_segment(&c->seg[idx], band, v, cols, rows);
    }
}

/* draw the '@' marker where the staircase is right now, so the eye can follow it. */
static void paint_iterator_point(const MapIterator *it, const UnitViewport *v,
                                 int cols, int rows)
{
    float fx = logistic_map_eval(&it->map, it->x);
    int sx = viewport_u_to_cx(v, it->x);
    int sy = viewport_u_to_cy(v, fx);
    if (!in_drawable(sx, sy, cols, rows)) return;
    attron(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
    mvaddch(sy, sx, '@');
    attroff(COLOR_PAIR(PAIR_LIVE) | A_BOLD);
}

/* draw the whole picture: curve, diagonal, staircase, then the live marker. */
static void paint_scene(const Scene *s, int cols, int rows)
{
    UnitViewport v; viewport_compute(&v, cols, rows);
    paint_logistic_curve(&v, &s->iter.map, cols, rows);
    paint_diagonal(&v, cols, rows);
    paint_cobweb(&s->cobweb, &v, cols, rows);
    paint_iterator_point(&s->iter, &v, cols, rows);
}

/* HUD */

static void hud_draw_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " COBWEB DIAGRAM (logistic) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_draw_top_right_status(int cols, double fps, int sim_fps,
                                      const Scene *s)
{
    const CobwebPreset *p = preset_state_active(&s->preset);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  r:%.4f  step:%d ",
             fps, sim_fps,
             s->paused ? "PAUSED  " : p->name,
             s->preset.current + 1, N_PRESETS,
             (double)p->r, s->iter.steps);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static int hud_field_bold_label(int x, const char *fmt, const char *val, int width)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, fmt, val);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + width;
}

static void hud_draw_param_row(const Scene *s)
{
    const CobwebPreset *p = preset_state_active(&s->preset);
    int x = HUD_LEFT_MARGIN;
    x = hud_field_bold_label(x, " preset:%-8s ", p->name,                    19);
    x = hud_field_bold_label(x, " theme:%-8s ",  themes[s->palette.current].name, 17);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " x:%.4f  f(x):%.4f  segs:%d/%d ",
             (double)s->iter.x,
             (double)logistic_map_eval(&s->iter.map, s->iter.x),
             s->cobweb.count, COBWEB_MAX);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void hud_draw_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:preset  t/T:theme  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* clear, draw the picture, then lay the HUD bars on top. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    paint_scene(s, sc->cols, sc->rows);
    hud_draw_top_left_title();
    hud_draw_top_right_status(sc->cols, fps, sim_fps, s);
    hud_draw_param_row(s);
    hud_draw_bottom_hint(sc->rows);
}

/* §10 app — top-level state and the main loop */

/*
 * App — the whole program's state, in one place.
 *
 * Holds the scene, the window size, the chosen speed, and two flags the signal
 * handlers set. It's a single file-scope variable (g_app) for one reason: signal
 * handlers can't take arguments and can only safely touch global state, so the
 * two flags they write have to be reachable without being passed in. Everything
 * else here is only touched by the main loop.
 *
 *   scene       : everything that changes while running.
 *   screen      : the cached window size.
 *   sim_fps     : how many steps per second, 1..60. Defaults to 8 so each step
 *                 is easy to watch; adjusted by ]/[. Separate from the 60 fps
 *                 drawing rate.
 *   running     : set to 0 to quit. The handlers and the q/ESC key clear it.
 *   need_resize : the resize handler sets this; the loop notices it next frame
 *                 and rebuilds the layout. (Both flags are sig_atomic_t so a
 *                 signal handler can write them safely.)
 */
typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* hook up Ctrl-C / kill to quit cleanly and window-resize to relayout. The
 * handlers only flip flags, which is all a signal handler is allowed to do. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void app_bootstrap(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_reset(&app->scene);
    app->need_resize = 0;
}

static void app_handle_pending_resize(App *app, int64_t *frame_time, int64_t *sim_accum)
{
    if (!app->need_resize) return;
    app_do_resize(app);
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    int64_t dt_cap = (int64_t)SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    if (dt > dt_cap) dt = dt_cap;
    return dt;
}

static void app_drain_fixed_timestep(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

static void app_update_fps_meter(int64_t dt, int *frame_count,
                                 int64_t *fps_accum, double *fps_display)
{
    (*frame_count)++;
    *fps_accum += dt;
    if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        *fps_display = (double)(*frame_count)
                     / ((double)(*fps_accum) / (double)NS_PER_SEC);
        *frame_count = 0; *fps_accum = 0;
    }
}

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

/* speed up / slow down, kept tiny so the key handler stays a clean list. */
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

/* handle one keypress; returns false only when the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */:
        return false;

    case ' ':
        s->paused = !s->paused;
        break;
    case 'r': case 'R':
        scene_reset(s);
        break;

    case ']':            app_sim_rate_faster(app); break;
    case '[':            app_sim_rate_slower(app); break;

    case 't':
        palette_state_cycle_next(&s->palette);
        theme_apply(s->palette.current);
        break;
    case 'T':
        palette_state_cycle_prev(&s->palette);
        theme_apply(s->palette.current);
        break;

    case 'n': case 'N':
        preset_state_cycle_next(&s->preset);
        scene_load_preset(s);
        break;
    case 'p': case 'P':
        preset_state_cycle_prev(&s->preset);
        scene_load_preset(s);
        break;

    default: break;
    }
    return true;
}

static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* set up, then loop: catch up the simulation by a fixed-size step, hold ~60 fps,
 * draw, read a key. Keeping the step size fixed makes the motion smooth no matter
 * how the frame rate wobbles. (Pattern from Glenn Fiedler, "Fix Your Timestep!") */
int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    main_install_signal_handlers();
    App *app = &g_app;
    app_bootstrap(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {
        app_handle_pending_resize    (app, &frame_time, &sim_accum);
        int64_t dt = app_compute_frame_dt(&frame_time);
        app_drain_fixed_timestep     (app, dt, &sim_accum);
        app_update_fps_meter         (dt, &frame_count, &fps_accum, &fps_display);
        app_throttle_to_render_target(frame_time, dt);
        app_present_frame            (app, fps_display);
        if (!app_poll_keyboard(app)) app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
