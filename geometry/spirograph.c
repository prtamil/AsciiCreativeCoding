/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * spirograph.c — Spirograph (Hypotrochoid)
 *
 * Three simultaneous hypotrochoid curves in different colours.  Each curve
 * traces a decaying trail on a per-cell float canvas; the canvas fades each
 * tick so older parts of the curve dim and eventually vanish, letting the
 * current trace shine.  Parameters drift slowly so the pattern evolves.
 *
 * PARAMETRIC FORMULA (hypotrochoid)
 * ───────────────────────────────────
 *   x(t) = (R − r)·cos(t)  +  d·cos((R−r)/r · t)
 *   y(t) = (R − r)·sin(t)  −  d·sin((R−r)/r · t)
 *
 * R = outer radius, r = rolling (inner) radius, d = pen distance from centre.
 * The ratio R/r determines the number of petals; d sets petal size.
 *
 * SLOW DRIFT
 * ──────────
 * r drifts sinusoidally between r_min and r_max at DRIFT_RATE rad/s,
 * gradually shifting between two distinct patterns without any keypress.
 *
 * CANVAS / TRAIL
 * ──────────────
 * float canvas[MAX_ROWS][MAX_COLS]  stores brightness [0,1] per cell.
 * int   cpair[MAX_ROWS][MAX_COLS]   stores the colour pair last written.
 *
 * Each tick:
 *   1. canvas *= FADE  (global brightness decay)
 *   2. For each curve: trace TRACE_STEPS points along the parameter
 *      interval [t, t+DELTA_T) and stamp canvas cells at brightness 1.0.
 *   3. t += DELTA_T.
 *
 * Scale so the largest curve fits within 85% of the shorter screen dimension.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset canvas   ] / [  sim Hz up / down
 *   t / T next / prev theme (10 themes: Sunset, Ocean, Forest, Cyberpunk,
 *                            Pastel, Mono, Fire, Aurora, Royal, Mint)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra spirograph.c -o spirograph -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Parametric curve tracing with a float canvas + fade.
 *                  Rather than drawing discrete points, each curve advances
 *                  T_STEPS_PER_FRAME timesteps per tick, writing brightness
 *                  values to a float canvas.  The canvas decays by FADE_RATE
 *                  per tick, creating the fading trail effect.
 *
 * Math           : Hypotrochoid parametric equations:
 *                    x(t) = (R−r)·cos(t) + d·cos((R−r)/r · t)
 *                    y(t) = (R−r)·sin(t) − d·sin((R−r)/r · t)
 *                  The curve closes when (R−r)/r = p/q (rational ratio).
 *                  With p=3, q=1: deltoid (3-cusp astroid).
 *                  Irrational ratios → the curve never closes (dense in annulus).
 *                  Pen distance d controls petal amplitude: d<(R−r) → inside,
 *                  d>(R−r) → outside → loops at each reversal point.
 *
 * Rendering      : Three simultaneous curves with different r values.
 *                  r drifts sinusoidally (DRIFT_RATE), creating continuous
 *                  morphing between distinct petal patterns without keystrokes.
 * ─────────────────────────────────────────────────────────────────────── */

/* -- REFERENCES ---------------------------------------------------------  *
 *
 * Roulette curves are an ancient subject -- Albrecht Dürer drew them
 * in 1525, and the modern Spirograph(R) toy (Denys Fisher, 1965) is a
 * gear-driven mechanism for tracing one out by hand.  The math below
 * covers the same family the toy traces: hypotrochoids (point inside
 * a rolling inner circle) and their generalisations.
 *
 * Curve theory & catalogues:
 *   [1] Lawrence, J. D. (1972). "A Catalog of Special Plane Curves."
 *       Dover Publications.
 *       -- the canonical catalogue.  Hypotrochoids, epitrochoids, and
 *          their special cases (deltoid, astroid, cardioid, ...) all
 *          live in a single chapter with parametric forms, closure
 *          criteria, and lobe-count formulae.
 *
 *   [2] Lockwood, E. H. (1961). "A Book of Curves."
 *       Cambridge University Press.
 *       -- didactic introduction; the chapter on roulettes derives the
 *          hypotrochoid equations geometrically (rolling-circle
 *          construction) rather than analytically.  Best entry point
 *          if the formulae feel arbitrary.
 *
 *   [3] Cundy, H. M., & Rollett, A. P. (1961). "Mathematical Models."
 *       Oxford University Press.
 *       -- chapter on epi-/hypotrochoid construction, including
 *          physical-model recipes (the Spirograph toy is essentially
 *          one of these).
 *
 *   [4] Maor, E. (1998). "Trigonometric Delights." Princeton Univ. Press.
 *       -- accessible treatment of the parametric form
 *          x(t) = (R−r)cos t + d cos((R−r)/r · t)
 *          and why the rational R/r criterion closes the curve.
 *
 * Dynamics (slow parameter drift):
 *   [5] Strogatz, S. H. (2015). "Nonlinear Dynamics and Chaos" (2nd ed.).
 *       Westview Press.
 *       -- the slow sinusoidal drift in r is a textbook example of
 *          quasi-periodic motion: a fast oscillator (the curve tracing)
 *          modulated by a slow oscillator (the drift in r).  Strogatz
 *          ch. 8 covers two-frequency systems and the rational/irrational
 *          dichotomy that decides whether the trajectory closes.
 *
 * Rendering:
 *   [6] Foley, van Dam, Feiner, Hughes (1995). "Computer Graphics:
 *       Principles and Practice" (2nd ed. in C), chapter 11.
 *       Addison-Wesley.
 *       -- parametric-curve discretisation.  Our TRACE_STEPS uniform
 *          t-sampling per tick is the simplest version of the textbook
 *          uniform-vs-adaptive trade-off.
 *
 *   [7] Akenine-Möller, T., Haines, E., Hoffman, N., et al. (2018).
 *       "Real-Time Rendering" (4th ed.). CRC Press.
 *       -- §12 covers motion blur and accumulation buffers.  Our
 *          per-tick canvas fade (Trail.brightness *= FADE) is the
 *          ASCII analogue of an exponential-decay accumulation buffer:
 *          each pixel is the geometric series of all past stamps,
 *          weighted by FADE^age.
 *
 * ----------------------------------------------------------------------- */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

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

    /*
     * Color-pair allocation:
     *   1 .. N_CURVES         -- curve colors; the active Theme rewrites
     *                             these via init_pair() at each theme switch
     *   N_CURVES+1            -- HUD title / hint  (theme-independent)
     *   N_CURVES+2            -- HUD data           (theme-independent)
     */
    N_PAIRS         = N_CURVES + 2,

    /* HUD layout — one row reserved at top for data, one at bottom
     * for action keys.  The drawing area is rows
     * [HUD_TOP_ROWS, rows - HUD_BOTTOM_ROWS).  Keep both as 1 to
     * preserve the existing draw budget. */
    HUD_TOP_ROWS    = 1,
    HUD_BOTTOM_ROWS = 1,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions */
#define CELL_W   8
#define CELL_H   16

/*
 * FADE — canvas brightness multiplier per tick.
 * At 60 Hz: 0.985^60 ≈ 0.40 → trail visible for ~1.5 seconds.
 */
#define FADE        0.985f

/*
 * DELTA_T — parameter advance per tick (radians).
 * TRACE_STEPS sub-samples per tick fill the curve without gaps.
 */
#define DELTA_T     0.08f
#define TRACE_STEPS 60

/* Slow parameter drift */
#define DRIFT_RATE  0.04f   /* rad/s — one full drift cycle every ~157 s  */

/*
 * Render-frame fitting.
 *
 * The widest preset has R+d ≈ 12.5 (unit scale).  We scale the figure so
 * that bounding radius fits FIT_FRACTION of the SHORTER screen dimension,
 * leaving the rest as margin around the figure.
 */
#define MAX_CURVE_EXTENT   12.5f   /* R + d of the widest preset (unit space) */
#define FIT_FRACTION        0.85f  /* fraction of shorter screen dim used    */

/*
 * Trail brightness mapping.
 *
 *   STAMP_BRIGHTNESS  -- value written when a pen samples a cell
 *   TRAIL_VISIBLE_MIN -- below this, the renderer skips the cell
 *   BRIGHT_THRESHOLD  -- above this, the cell is painted A_BOLD
 *   DIM_THRESHOLD     -- below this (but above visible), painted A_DIM
 *   N_GLYPHS          -- intensity-glyph table size (" .,:+*#@")
 */
#define STAMP_BRIGHTNESS    1.0f
#define TRAIL_VISIBLE_MIN   0.04f
#define BRIGHT_THRESHOLD    0.7f
#define DIM_THRESHOLD       0.25f
#define N_GLYPHS            8

/* Effective rolling-radius clamp: keep (R-r)/r finite when r_amp drives
 * r_actual close to zero.  Anything below R_ACTUAL_MIN gets clipped to it. */
#define R_ACTUAL_MIN        0.5f

/* Frame pacing (§8 main loop). */
#define DT_CAP_NS           (100 * NS_PER_MS)    /* spiral-of-death guard */
#define FRAME_PERIOD_NS     (NS_PER_SEC / 60)    /* 60 Hz render cadence  */

/* Input. */
#define KEY_ESC             27

/* HUD layout & colour-pair semantics.
 *
 * Pairs 1..N_CURVES carry curve colours and are rewritten by theme_apply;
 * pairs N_CURVES+1 and N_CURVES+2 are theme-independent HUD slots. */
#define HUD_DATA_COL        16              /* column where data segment starts */
#define HUD_PAIR_TITLE      (N_CURVES + 1)  /* bright cyan + A_BOLD             */
#define HUD_PAIR_DATA       (N_CURVES + 2)  /* bright yellow + A_BOLD           */
#define HUD_PAIR_HINT       HUD_PAIR_TITLE  /* action bar reuses title pair     */
#define FALLBACK_PAIR       1               /* recover to curve-0 pair if a
                                               trail cell holds a bad index    */

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Theme -- a named palette of N_CURVES colours.
 *
 * On theme switch (the 't' key) the active theme's colours[] are loaded
 * into ncurses pairs 1..N_CURVES via init_pair().  Curve.pair indices
 * stay constant; only the underlying ANSI colour codes change.  This
 * means EXISTING trail cells automatically pick up the new colours --
 * no trail clear needed.  Theme switching becomes a one-pass palette
 * swap, not a redraw.
 *
 * Storage:
 *   colors_256[]  -- 256-colour ANSI codes (preferred when COLORS >= 256)
 *   colors_8[]    -- 8-colour fallback (COLOR_RED, COLOR_CYAN, ...)
 * Both are N_CURVES long; one entry per curve.
 *
 * Reference: Foley/van Dam [6] §13 on indexed-colour LUTs -- theme_apply
 * is the LUT swap, the Trail is the indexed-colour framebuffer.
 */
typedef struct {
    const char *name;
    int         colors_256[N_CURVES];
    int         colors_8  [N_CURVES];
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name            256-colour codes              8-colour fallback                              */
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

/* Wrap a theme index back into [0, N_THEMES). */
static inline int theme_next(int cur) { return (cur + 1) % N_THEMES; }
static inline int theme_prev(int cur) { return (cur + N_THEMES - 1) % N_THEMES; }

/*
 * theme_apply -- rewrite ncurses pairs 1..N_CURVES with the chosen
 * theme's palette.  The HUD pairs are untouched (they live above
 * N_CURVES) so the HUD's cyan/yellow stay constant across themes.
 */
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

/*
 * color_init -- one-time palette setup.
 *
 *   1. HUD pairs (theme-independent) get their fixed yellow/cyan colours.
 *   2. theme_apply(0) installs the default theme's curve palette.
 */
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

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline int px_to_col(float px) { return (int)floorf(px / CELL_W + 0.5f); }
static inline int px_to_row(float py) { return (int)floorf(py / CELL_H + 0.5f); }

/*
 * RenderFrame -- per-frame derived render geometry.
 *
 * Holds the figure center in PIXEL space (sub-cell precision: one
 * pixel = 1/CELL_W of a column or 1/CELL_H of a row) and the unit→pixel
 * scale factor that fits MAX_CURVE_EXTENT within FIT_FRACTION of the
 * shorter screen dimension.
 *
 * WHY pixel space (and not directly cell space):
 *   The curve trace lives in CONTINUOUS unit space; cells are integer
 *   grid positions.  Working in pixel space gives CELL_W × CELL_H
 *   sub-cell positional precision before the snap to integer cells
 *   (px_to_col / px_to_row in §4).  Without that intermediate
 *   precision, two close samples could collide at the same cell, the
 *   trail would lose fine motion, and small pen movements would
 *   appear as staircase steps.  See Foley/van Dam [6] §11 on
 *   parametric-curve discretisation precision.
 *
 * WHY computed fresh each frame and NOT cached on Scene:
 *   Every field is a pure function of (cols, rows).  Caching on Scene
 *   would create a stale-value risk: a SIGWINCH between cache-write
 *   and frame-read would leave RenderFrame pointing at the old
 *   viewport.  Pure recomputation costs ~5 FLOPs/frame -- imperceptible.
 *
 * Fields:
 *   cx_px   = (cols·CELL_W)/2  -- figure center column in pixels
 *   cy_px   = (rows·CELL_H)/2  -- figure center row    in pixels
 *   scale   = shorter_px · FIT_FRACTION/2 / MAX_CURVE_EXTENT
 *             one unit in physics space becomes `scale` pixels on screen
 */
typedef struct {
    float cx_px;    /* figure center X in pixel coords */
    float cy_px;    /* figure center Y in pixel coords */
    float scale;    /* unit→pixel scale factor          */
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

/* ===================================================================== */
/* §5  entity — Curve, Spirograph                                         */
/* ===================================================================== */

/*
 * Curve -- the live state of one rolling hypotrochoid "pen".
 *
 * Physical picture:
 *   A small circle of radius r rolls without slipping INSIDE a large
 *   host circle of radius R.  A pen is rigidly attached to the small
 *   circle at distance d from its centre.  The pen traces a hypotrochoid.
 *     d < R-r   the pen stays inside the host circle (rosette curves)
 *     d > R-r   the pen swings outside the rolling center, looping at
 *               each reversal point (looped curves)
 *   See Lockwood [2] for the geometric rolling-circle derivation.
 *
 * Parametric form (also in CONCEPTS at top of file; Maor [4]):
 *     x(t) = (R-r) cos t + d cos((R-r)/r · t)
 *     y(t) = (R-r) sin t - d sin((R-r)/r · t)
 *   The curve CLOSES iff (R-r)/r is rational.  Lobe count for the
 *   reduced ratio p/q is p (Lawrence [1] §hypotrochoid).
 *
 * Drift modulation of r:
 *     r_actual = r_base + r_amp · sin(drift)
 *   so as `drift` slowly advances (DRIFT_RATE rad/s) the effective r
 *   sweeps a band [r_base - r_amp, r_base + r_amp], and the curve
 *   morphs smoothly between adjacent integer lobe counts.  No keypress,
 *   no discontinuity -- this is the slow modulator in a two-frequency
 *   quasi-periodic system (Strogatz [5] §8).
 *
 * WHY one struct (not three: params + state + style):
 *   Every field describes the same physical pen.  They all travel
 *   together through the simulation -- one Curve per slot in
 *   Spirograph.curves[].  Splitting would require parallel arrays and
 *   index management for no clarity gain.
 *
 * Fields (range / unit / mutator):
 *   R         (0, ∞)  unit       outer host-circle radius   -- const after init
 *   r_base    (0, R)  unit       mean rolling-circle radius -- const after init
 *   r_amp     [0, R)  unit       drift amplitude of r       -- const after init
 *   d         (0, ∞)  unit       pen offset from small-circle centre
 *                                                            -- const after init
 *   t         free    rad        current parameter value    -- advances DELTA_T/tick
 *   drift     free    rad        slow modulator of r_actual -- advances DRIFT_RATE·dt
 *   pair      [1, N_COLORS]      ncurses colour pair index  -- const after init
 */
typedef struct {
    float R;        /* outer radius (unit scale)                          */
    float r_base;   /* inner radius base (unit scale)                     */
    float r_amp;    /* drift amplitude for r                              */
    float d;        /* pen offset (unit scale)                            */
    float t;        /* current parameter value (radians)                  */
    float drift;    /* drift phase (radians), advances by DRIFT_RATE·dt   */
    int   pair;     /* ncurses colour pair                                */
} Curve;

/*
 * CurvePreset -- a hardcoded, named hypotrochoid recipe.
 *
 * Three of these populate the demo at startup and on user 'r' reset
 * (see spirograph_reset_curves).  The R/r_base ratio determines the
 * DOMINANT lobe count when r_actual ≈ r_base (Lawrence [1] lobe
 * formula); the drift modulation means r is rarely exactly r_base,
 * so the visible figure interpolates between adjacent integer cases.
 *
 * WHY a separate "preset" type, distinct from Curve:
 *   1. Const correctness.  k_presets[] is const-static; CurvePreset
 *      can be passed as `const CurvePreset *` and the compiler enforces
 *      that nothing mutates a preset.  Curve, by contrast, IS mutable
 *      (t and drift advance every tick).  Two types make this static-
 *      vs-dynamic split explicit.
 *
 *   2. Index stability.  Presets stay in fixed positions in k_presets[];
 *      adding a new preset doesn't shuffle indices in surprising ways
 *      because each entry is independently named.
 *
 *   3. Documentation surface.  The `name` field carries the human label
 *      ("Pentagon", "Heptagon", "Triangle") for HUD use; embedding it
 *      in the preset keeps name in sync with parameters automatically.
 *
 * Fields:
 *   R           (0, ∞)   outer-circle radius (unit space)
 *   r_base      (0, R)   mean inner-circle radius
 *   r_amp       [0, R)   drift amplitude (∆r above/below r_base)
 *   d           (0, ∞)   pen offset from small-circle centre
 *   t0          free     initial parameter value
 *                        Staggered by 2π/3 across the three presets so
 *                        the curves don't start at the same point.
 *   drift0      free     initial drift phase
 *                        Staggered by 1 rad so the three curves fall
 *                        in/out of "interesting" r_actual values at
 *                        different times -- the user never sees all
 *                        three pause-and-morph simultaneously.
 *   color_pair  [1,N_COLORS]  ncurses pair (matches §3 color_init)
 *   name        non-null      shape family ("Pentagon", "Heptagon", ...)
 */
typedef struct {
    float       R;          /* outer radius (unit scale) */
    float       r_base;     /* inner radius base */
    float       r_amp;      /* drift amplitude for r */
    float       d;          /* pen offset (unit scale) */
    float       t0;         /* initial parameter value */
    float       drift0;     /* initial drift phase */
    int         color_pair; /* ncurses colour pair */
    const char *name;       /* human-readable family ("Pentagon", ...) */
} CurvePreset;

/*
 * The three presets.  t0 is staggered by 2π/3 so curves don't overlap
 * exactly at frame 1; drift0 is staggered by 1 rad so they fall in and
 * out of "interesting" r_actual values at different times.
 *
 *   R   r_base  r_amp     d    t0                      drift0  pair  shape
 */
/*
 * Curve-to-pair assignment: curve i uses ncurses pair (i+1).  Pair
 * indices stay constant; theme_apply rewrites the underlying colours.
 * (Old layout used pairs 5/7/3 from a fixed 7-colour palette; the
 * themed layout collapses curve pairs into 1..N_CURVES so theme_apply
 * has a contiguous range to rewrite.)
 */
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
 * Trail -- the fading float canvas where past curve samples accumulate.
 *
 * WHY this exists (the visual effect):
 *   Without a trail, only the instantaneous pen positions would be
 *   visible -- three moving dots.  The Trail integrates samples over
 *   time, so each curve becomes a glowing path that fades behind the
 *   pen.  Each tick:
 *     1.  brightness[r][c] *= FADE                       (fade everything)
 *     2.  for each new sample at (r,c):
 *           brightness[r][c]  = 1.0                      (stamp full)
 *           color_pair[r][c]  = current pen
 *   The visible cell value is the geometric series over all past stamps
 *   weighted by FADE^age -- the ASCII analogue of an exponential-decay
 *   accumulation buffer in real-time graphics (Akenine-Möller [7] §12
 *   on motion blur).  Cells below TRAIL_VISIBLE_MIN are skipped by
 *   the renderer, so the trail has a finite effective length.
 *
 * WHY parallel arrays (brightness + color_pair) instead of one
 * per-cell struct:
 *   The fade pass touches only brightness[][] for the whole grid;
 *   keeping it in a separate flat array means the fade loop scans
 *   memory sequentially and is friendly to the prefetcher.  A struct
 *   of {float, int} per cell would interleave the two channels, hurt
 *   that locality, and waste alignment padding.
 *
 * Fixed-size storage:
 *   MAX_ROWS × MAX_COLS so allocation is static -- no malloc in the
 *   hot path (CLAUDE.md rule).  The renderer clips to the live
 *   (rows, cols); cells beyond those bounds are simply unused.
 *
 * Fields:
 *   brightness   [0, 1]            visible glow per cell (float for the
 *                                  smooth-decay multiplication)
 *   color_pair   [1, N_COLORS]     which curve's pen last stamped this
 *                                  cell; "last write wins" gives the
 *                                  overlapping-curves blending effect
 */
typedef struct {
    float brightness[MAX_ROWS][MAX_COLS];  /* per-cell brightness, 0..1   */
    int   color_pair[MAX_ROWS][MAX_COLS];  /* per-cell ncurses pair       */
} Trail;

/*
 * Spirograph -- the full simulation state: N_CURVES rolling pens
 * painting onto a shared Trail.
 *
 * Mental model:
 *   N_CURVES "pens" (Curve entries) are rolling around the screen at
 *   different speeds and radii; the canvas (Trail) remembers where
 *   they've been, fading each tick.  This is the entire simulation;
 *   everything else in the file is either chrome (HUD, Screen) or
 *   plumbing (App, signal handlers).
 *
 * WHY one shared Trail (not one Trail per Curve):
 *   The visual effect IS the overlap.  Curves stamping over each
 *   other create the layered multi-colour petals.  With one Trail
 *   per Curve the renderer would have to composite N_CURVES grids
 *   per frame and pick a blending rule; with a single shared Trail
 *   the rule "most recent stamp wins" is free -- it falls out of the
 *   last-write-into-color_pair[r][c] semantic.
 *
 * WHY 'paused' lives here (not on Scene):
 *   paused is a property of THIS simulation, not of "Scene" in the
 *   abstract.  scene_tick delegates to spirograph_tick, which checks
 *   sg->paused at its entry.  If a future Scene variant adds a
 *   different simulation, that simulation gets its own paused flag.
 *
 * Fields:
 *   trail      fading canvas of past pen positions (visual memory)
 *   curves     the N_CURVES rolling-pen state vectors
 *   theme_idx  active row of k_themes[]; cycled by the 't' key.  Only
 *              affects the underlying colours of ncurses pairs
 *              1..N_CURVES via theme_apply -- no simulation impact.
 *   paused     freezes scene_tick; rendering still runs so the figure
 *              stays on screen and the user can still change params
 *              while frozen (SPACE key toggles)
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

/* ---- curve math + state helpers ------------------------------------- */

/*
 * Effective rolling radius after drift modulation, clamped at
 * R_ACTUAL_MIN so (R-r)/r stays finite.
 *
 *     r_actual = max(r_base + r_amp · sin(drift), R_ACTUAL_MIN)
 */
static float curve_r_actual(const Curve *cv)
{
    float r = cv->r_base + cv->r_amp * sinf(cv->drift);
    return (r < R_ACTUAL_MIN) ? R_ACTUAL_MIN : r;
}

/*
 * Hypotrochoid parametric position (unit space):
 *     x = (R-r) cos t + d cos((R-r)/r · t)
 *     y = (R-r) sin t - d sin((R-r)/r · t)
 * Inputs are pre-computed: Rmr = R - r, ratio = (R-r)/r.
 */
static void hypotrochoid_xy(float Rmr, float ratio, float d, float t,
                            float *x, float *y)
{
    *x = Rmr * cosf(t) + d * cosf(ratio * t);
    *y = Rmr * sinf(t) - d * sinf(ratio * t);
}

/* Advance a curve's parametric time and drift by one tick. */
static inline void curve_advance(Curve *cv, float dt)
{
    cv->t     += DELTA_T;
    cv->drift += DRIFT_RATE * dt;
}

/* ---- trail helpers --------------------------------------------------- */

/* Multiply every cell's brightness by FADE.  Clipped to live bounds. */
static void trail_fade(Trail *trail, int rows, int cols)
{
    int row_end = (rows < MAX_ROWS) ? rows : MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;
    for (int r = 0; r < row_end; r++)
        for (int c = 0; c < col_end; c++)
            trail->brightness[r][c] *= FADE;
}

/* Stamp one cell at full brightness with the given colour pair. */
static inline void trail_stamp(Trail *trail, int row, int col, int pair)
{
    trail->brightness[row][col] = STAMP_BRIGHTNESS;
    trail->color_pair[row][col] = pair;
}

/* True iff (row, col) is inside the drawing band: clear of both HUD
 * bands and within the static MAX_ROWS/MAX_COLS storage bounds. */
static inline bool trail_in_band(int row, int col, int rows, int cols)
{
    if (col < 0 || col >= cols || col >= MAX_COLS) return false;
    if (row < HUD_TOP_ROWS || row >= rows - HUD_BOTTOM_ROWS
        || row >= MAX_ROWS) return false;
    return true;
}

/*
 * Trace one curve for DELTA_T worth of parametric time, stamping
 * TRACE_STEPS intermediate samples onto the trail.
 *
 * Reads as pseudocode:
 *   compute r_actual, Rmr, ratio for this tick
 *   for each sub-step:
 *       evaluate hypotrochoid (x, y)
 *       project to pixel space via RenderFrame
 *       snap to cell; clip to drawing band; stamp trail
 */
static void curve_trace(Curve *cv, Trail *trail,
                        const RenderFrame *rf, int rows, int cols)
{
    float r_actual = curve_r_actual(cv);
    float Rmr      = cv->R - r_actual;          /* R - r       */
    float ratio    = Rmr / r_actual;            /* (R-r)/r     */

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

/*
 * spirograph_tick -- advance one simulation step.
 *
 * Reads as pseudocode:
 *   if paused: do nothing
 *   else:
 *       1. fade the trail uniformly
 *       2. for each curve: trace it onto the trail, then advance its state
 */
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

/* ---- trail-cell rendering ------------------------------------------- */

/* Map brightness [0, 1] to glyph index in the " .,:+*#@" ramp. */
static inline int brightness_to_glyph_idx(float b)
{
    int idx = (int)(b * (float)(N_GLYPHS - 1));
    if (idx < 0)         idx = 0;
    if (idx >= N_GLYPHS) idx = N_GLYPHS - 1;
    return idx;
}

/* Choose ncurses attribute by brightness tier.
 *   above BRIGHT_THRESHOLD  -> A_BOLD  (fresh stamp)
 *   below DIM_THRESHOLD     -> A_DIM   (old, fading)
 *   in between              -> plain
 */
static inline chtype brightness_to_attr(float b)
{
    if (b > BRIGHT_THRESHOLD) return A_BOLD;
    if (b < DIM_THRESHOLD)    return A_DIM;
    return 0;
}

/* Recover from an unset / out-of-range colour pair on a Trail cell.
 * Valid pair indices for trail cells are 1..N_CURVES (curves only --
 * HUD pairs never appear in Trail.color_pair).  (Defensive: should
 * never trigger for normal operation, but cheap.) */
static inline int sanitize_pair(int p)
{
    return (p < 1 || p > N_CURVES) ? FALLBACK_PAIR : p;
}

/* Paint one Trail cell if it crosses the visibility threshold. */
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

/*
 * spirograph_draw -- paint the visible portion of the trail.
 *
 * Iterates the drawable band ([HUD_TOP_ROWS, rows - HUD_BOTTOM_ROWS) ×
 * [0, cols)) clipped to MAX_ROWS / MAX_COLS storage.  Each cell decides
 * its own glyph + attribute via render_trail_cell.
 */
static void spirograph_draw(const Spirograph *sg, WINDOW *w, int cols, int rows)
{
    int row_end = rows - HUD_BOTTOM_ROWS;
    if (row_end > MAX_ROWS) row_end = MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int row = HUD_TOP_ROWS; row < row_end; row++)
        for (int col = 0; col < col_end; col++)
            render_trail_cell(w, &sg->trail, row, col);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene -- the abstract "what's on screen this frame" container.
 *
 * Currently a thin wrapper around a single Spirograph simulation.
 *
 * WHY the wrapper at all (when it's one field deep):
 *   The App layer composes Scene + Screen + signal flags into the
 *   top-level App struct.  Holding a Scene type (rather than putting
 *   Spirograph directly on App) means an alternative simulation --
 *   say, a Lissajous demo or a Maurer-rose variant -- can slot into
 *   the same App composition without changes propagating outward.
 *   The cost is one extra `.sg.` in field accesses; the win is a
 *   clean App/Scene/Spirograph layering that mirrors the §6/§5
 *   section split.
 *
 * Fields:
 *   sg   the Spirograph simulation (this file's only "scene")
 */
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

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- cached terminal geometry.
 *
 * WHY cache cols/rows on a struct instead of calling getmaxyx() everywhere:
 *   getmaxyx is cheap but not free, and -- more importantly -- many
 *   functions in the render layer take rows/cols as plain ints.
 *   Passing a Screen* would conflate "I need the size" with "I need
 *   to talk to ncurses".  Holding size as a value lets the math layer
 *   stay terminal-agnostic; only screen_init / the SIGWINCH resize
 *   path in main() ever write into Screen.
 *
 * Update points:
 *   screen_init   -- once at startup
 *   resize path   -- after a SIGWINCH (terminal resized)
 *
 * Fields follow the ncurses convention:
 *   cols   X-extent in character cells
 *   rows   Y-extent in character cells
 * Both are 1-based counts, so valid coordinates are
 *    [0, cols-1] × [0, rows-1].
 * The HUD bands reserve HUD_TOP_ROWS at the top and HUD_BOTTOM_ROWS at
 * the bottom; the trail renderer must stay inside the middle band.
 */
typedef struct {
    int cols;   /* terminal width  in character cells */
    int rows;   /* terminal height in character cells */
} Screen;

static void screen_init(Screen *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) { (void)s; endwin(); }

/* ---- HUD row helpers ------------------------------------------------- */

/* Row 0 left -- the title block. */
static void hud_draw_title(void)
{
    attron(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
    mvprintw(0, 0, " [SPIROGRAPH] ");
    attroff(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
}

/* Row 0 middle -- live rolling-radius values + active theme name. */
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

/* Row 0 right -- live engine stats: render fps, sim Hz, paused/running. */
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

/* Row rows-1 -- action keys, bright cyan + A_BOLD (HUD standard). */
static void hud_draw_action_bar(Screen *s)
{
    attron(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit   SPACE:pause   r:reset   t:theme   [/]:sim Hz ");
    attroff(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
}

/*
 * Top HUD (row 0): title (left) + live curve r-values (middle) +
 *                  fps / sim Hz / paused (right).
 * Bottom HUD (row rows-1): action keys.
 */
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

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App -- the top-level container.
 *
 * Single static instance (g_app, declared below) so the signal handlers
 * can flip flags on it without scattering globals across the file.
 *
 * WHY signal handler flags are on App, not Scene:
 *   Signals interrupt the main loop, not the simulation.  "Should I
 *   quit?" and "did the terminal resize?" are run-loop concerns, not
 *   spirograph-math concerns.  Keeping them off Scene means the
 *   simulation stays PURE state -- snapshottable, replayable from a
 *   seed, testable in isolation.
 *
 * WHY volatile sig_atomic_t (and not bool):
 *   Per POSIX, sig_atomic_t is the only integer type guaranteed atomic
 *   with respect to async signal delivery.  `volatile` forces the
 *   main loop to re-read from memory each iteration rather than
 *   caching the flag in a register; without it the compiler may
 *   legally hoist the read out of the loop and the program would
 *   never see the signal-set value.
 *
 * Fields:
 *   scene        the simulation state           (§6 Scene)
 *   screen       cached terminal dimensions     (§7 Screen)
 *   sim_fps      tick rate of scene_tick; [ and ] keys adjust it
 *                within [SIM_FPS_MIN, SIM_FPS_MAX].  Independent of
 *                render fps -- the main loop renders at a fixed
 *                ~60 Hz wallclock cadence regardless of sim_fps.
 *   running      0 ⇒ main loop should exit      (SIGINT / SIGTERM)
 *   need_resize  1 ⇒ main loop should re-init   (SIGWINCH)
 */
typedef struct {
    Scene                 scene;        /* simulation state                   */
    Screen                screen;       /* terminal size cache                */
    int                   sim_fps;      /* sim-tick rate; [ / ] adjust at runtime */
    volatile sig_atomic_t running;      /* 0 = exit main loop  (SIGINT)       */
    volatile sig_atomic_t need_resize;  /* 1 = re-init screen  (SIGWINCH)     */
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

/* ---- startup / shutdown ---------------------------------------------- */

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

/* ---- main-loop step helpers ----------------------------------------- */

/* Respond to SIGWINCH: re-query terminal size, clear the trail (resize
 * invalidates pixel positions), and reset frame timing so the next dt
 * isn't enormous. */
static void apply_resize(App *app, int64_t *frame_time, int64_t *sim_accum)
{
    endwin(); refresh();
    getmaxyx(stdscr, app->screen.rows, app->screen.cols);
    spirograph_clear_trail(&app->scene.sg);
    app->need_resize = 0;
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

/* Wallclock dt since the previous frame, capped at DT_CAP_NS so a long
 * pause (debugger, suspend) doesn't unleash a flood of sim ticks. */
static int64_t frame_dt_clamped(int64_t *last_ns)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *last_ns;
    *last_ns    = now;
    if (dt > DT_CAP_NS) dt = DT_CAP_NS;
    return dt;
}

/* Fixed-timestep accumulator: drain whole sim ticks at the current
 * sim_fps rate.  Decouples simulation rate from render fps. */
static void frame_drain_sim_ticks(Scene *sc, int sim_fps, float dt_sec,
                                  int cols, int rows, int64_t *accum)
{
    int64_t tick_ns = TICK_NS(sim_fps);
    while (*accum >= tick_ns) {
        scene_tick(sc, dt_sec, cols, rows);
        *accum -= tick_ns;
    }
}

/* Rolling FPS counter -- average frames per second over the most recent
 * FPS_UPDATE_MS window, then reset. */
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

/* Sleep enough to maintain FRAME_PERIOD_NS, given how much wall time
 * this frame already spent. */
static void frame_sleep_to_target(int64_t frame_start_ns, int64_t work_so_far)
{
    int64_t spent = clock_ns() - frame_start_ns + work_so_far;
    clock_sleep_ns(FRAME_PERIOD_NS - spent);
}

/* Pull pending keystroke; dispatch to app_handle_key.  Returns false
 * only when the user pressed a quit key. */
static bool drain_input(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* Paint one complete frame: scene + HUD + present. */
static void frame_render(App *app, double fps, float alpha, float dt_sec)
{
    screen_draw(&app->screen, &app->scene, fps, app->sim_fps, alpha, dt_sec);
    screen_present();
}

/*
 * main() -- the orchestrator.  Each line names one phase of a frame.
 */
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
