/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * burning_ship.c  —  Burning Ship fractal, five showcase zoom presets
 *
 * Escape-time iteration identical to Mandelbrot except both components are
 * folded to the first quadrant before each squaring:
 *
 *     z ← (|Re(z)| + i·|Im(z)|)²  +  c
 *
 * Expanded:
 *     Re' = Re² − Im²       + Re(c)
 *     Im' = 2 · |Re| · |Im| + Im(c)
 *
 * The |Im| fold breaks Mandelbrot's 4-fold symmetry and produces the
 * characteristic ship hull + downward flame filaments.
 *
 * Five zoom presets — a progressive zoom into the ship, each level rich with
 * self-similar detail.  Advance manually with 'n'; the view holds otherwise:
 *   full armada     — overview: main body, ship to the left, flames below
 *   the ship        — iconic hull + fleet sailing right (the namesake)
 *   hull & masts    — the hull edge, masts and rigging
 *   mini-armada     — self-similar ships within the hull's harbour
 *   antenna sweep   — fine filament sweep deep in the rigging
 *
 * Pixels are revealed in a Fisher-Yates shuffled order so the fractal
 * materialises from scattered dots rather than scan-line by scan-line.
 *
 * Color — fire palette by default (inside set → boundary → background):
 *   inside set     →  white / yellow          *
 *   near inside    →  yellow                  #
 *   mid escape     →  amber                   +
 *   far escape     →  orange                  .
 *   outermost      →  red                     ,
 *   fast escape    →  (black / empty)
 *
 * Keys:
 *   q / ESC     quit
 *   n / r       advance to next preset view (presets do NOT auto-cycle)
 *   t / T       cycle color theme  (Fire/Ember/Magma/Ice/Toxic/Nebula/Mono)
 *   ] [         faster / slower reveal
 *   p / spc     pause / resume
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra burning_ship.c -o burning_ship -lncurses -lm
 *
 * Reading order — the file is built bottom-up from the math to the app.
 * Each section is one concept; later sections only call earlier ones.
 *   §1  config   — constants, themes, palette slots
 *   §2  clock    — monotonic time
 *   §3  complex  — Complex number + arithmetic (the plane the fractal lives in)
 *   §4  fractal  — burning-ship escape iteration + escape→colour mapping
 *   §5  view     — Preset table + ViewWindow lens (screen cell ↔ complex point)
 *   §6  canvas   — pixel buffer of colour bands
 *   §7  reveal   — Fisher-Yates random-fill animation
 *   §8  color    — apply a Theme to ncurses colour pairs
 *   §9  scene    — Scene: composes view + canvas + reveal into one picture
 *   §10 screen   — ncurses screen lifecycle + HUD
 *   §11 app      — signals, input, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Escape-time on the burning-ship iteration
 *                  z ← (|Re(z)| + i·|Im(z)|)² + c.  Per pixel, start z = 0,
 *                  iterate; if |z| > 2 at step k, colour by k (escape time).
 *                  Pixels are revealed in shuffled order ("materialise from
 *                  noise").
 *
 * Math           : The absolute-value fold maps the plane to the first
 *                  quadrant before each squaring.  Escape test |z|² > 4
 *                  (squared to avoid a sqrt).  Inside the set = uniform colour.
 *                  Discovered by Michelitsch & Rössler (1992).  The set is NOT
 *                  connected — unlike Mandelbrot — so "islands" appear.
 *
 * Data model     : Five small structs, one concept each —
 *                    Complex     a point/number in the plane
 *                    ViewWindow  the rectangle of the plane we're looking at
 *                    Canvas      the grid of coloured cells we paint
 *                    Reveal      the random order we fill the canvas in
 *                    Scene       all of the above + which preset/theme is live
 *
 * Rendering      : Escape count → 5-band palette via fraction iter / MAX_ITER.
 *                  Inside → most-vivid; near-boundary → bright; far → dim/blank.
 *
 * Performance    : O(MAX_ITER) per pixel worst case, O(1) for fast-escape
 *                  pixels.  Throttled to PIXELS_PER_TICK per tick so the reveal
 *                  plays at a human pace.  Complex helpers are `static inline`,
 *                  so the math notation costs nothing after -O2.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── REFERENCES ───────────────────────────────────────────────────────── *
 *
 * Concepts & maths
 *   [1] Michelitsch, M. & Rössler, O. E. (1992). "The 'Burning Ship' and its
 *       quasi-Julia sets." Computers & Graphics 16(4):435-438.  — the paper
 *       that introduced this fractal and the absolute-value fold.  Start here.
 *   [2] Mandelbrot, B. B. (1982). "The Fractal Geometry of Nature." Freeman.
 *       — foundational: self-similarity, escape sets, the z²+c family.
 *   [3] Peitgen, H.-O. & Richter, P. H. (1986). "The Beauty of Fractals."
 *       Springer.  — the complex dynamics behind escape-time imagery, with
 *       the clearest pictures of how bulbs/antennae/filaments arise.
 *   [4] Peitgen, Jürgens & Saupe (1992). "Chaos and Fractals: New Frontiers
 *       of Science." Springer.  — the Mandelbrot chapter states the
 *       escape-time algorithm in full; the §4 loop is a direct descendant.
 *
 * Rendering & colouring
 *   [5] Härkönen, J. (2007). "On Smooth Fractal Colouring Techniques."
 *       M.Sc. thesis, Åbo Akademi.  — survey of escape-time colouring; how to
 *       go from the integer bands used in §4 to continuous gradients.
 *   [6] Vepstas, L. (2004). "Renormalizing the Mandelbrot Escape."  — derives
 *       the fractional escape count  n + 1 − log₂(log|z|)  that removes the
 *       visible banding (this file deliberately keeps simple bands; this is
 *       the reference to consult before swapping in smooth colouring).
 *   [7] Bourke, P. — fractal collection (paulbourke.net/fractals), incl. a
 *       Burning Ship page, and the character-by-luminance ramp this project's
 *       ASCII renderers borrow.
 *
 * In-repo study companions
 *   [8] mandelbrot.c — the same escape-time engine WITHOUT the fold; diff its
 *       iterate body against §3/§4 here to see exactly what the ship changes.
 *   [9] documentation/COLOR.md — the palette / escape-colouring techniques the
 *       Theme table in §1 is built on.
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  30,
    SIM_FPS_MAX      =  60,
    SIM_FPS_STEP     =   5,

    PIXELS_PER_TICK  =  80,    /* pixels revealed per simulation tick       */
    MAX_ITER         = 256,    /* burning-ship iteration cap                */
    N_PRESETS        =   5,
    N_THEMES         =   7,

    CANVAS_ROWS_MAX  =  80,    /* upper bound on terminal rows we buffer    */
    CANVAS_COLS_MAX  = 300,    /* upper bound on terminal cols we buffer    */

    RENDER_FPS       =  60,    /* display refresh cap (≠ reveal speed)      */
    MAX_FRAME_MS     = 100,    /* clamp on one frame's dt (anti spiral)     */
    HUD_COLS         =  80,
    FPS_UPDATE_MS    = 500,    /* recompute the fps readout this often      */
};

/* |z| > 2 ⇒ the orbit escapes to infinity.  We compare |z|² to 4 to skip the
 * square root in the hot loop. */
#define ESCAPE_RADIUS_SQ  4.0f

/* Pixels that escape within the first 8 % of the iteration budget are far
 * exterior — left blank so the picture isn't a wall of noise. */
#define BACKGROUND_FRAC   0.08f

/* 256-colour index for the bottom action line — a bright cyan that reads on
 * any theme, so the key hints stay legible whatever palette is active. */
#define HINT_CYAN   51

/*
 * ASPECT_R — terminal cell height / width ≈ 2.
 * Corrects the complex-plane mapping so the ship hull stays square-on.
 */
#define ASPECT_R    2.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * ColorID — the set of "colour slots" used throughout the program.
 *
 * WHY a numbered enum: each value does double duty.  It is (a) the id handed to
 * ncurses' init_pair()/COLOR_PAIR(), and (b) the single byte stored in every
 * Canvas cell to remember how that pixel should look.  So one small number
 * answers "what colour is this pixel?" with no extra lookup.
 *
 * WHY this order: an escape-time fractal colours a pixel by HOW FAST its orbit
 * escaped (refs [3][4]).  We bin that speed into bands.  The numbers climb from
 * the fastest-escaping far exterior (C2) up to the slow near-boundary halo
 * (C5), then the never-escaping interior (INSIDE).  Bigger number ⇒ closer to
 * the set ⇒ brighter colour, by convention.
 *
 * Value 0 is reserved: a Canvas cell holding 0 means "not drawn yet / blank".
 */
typedef enum {
    COL_INSIDE = 1,   /* the set itself — orbit never escaped (brightest)      */
    COL_C2     = 2,   /* far exterior   — escaped almost instantly (dimmest)   */
    COL_C3     = 3,   /* exterior       — escaped quickly                      */
    COL_C4     = 4,   /* mid halo       — escaped slowly                       */
    COL_C5     = 5,   /* inner halo     — escaped just outside the set (bright)*/
    COL_HUD    = 6,   /* HUD top data line   (not a fractal colour)            */
    COL_HINT   = 7,   /* HUD bottom action line — fixed cyan                   */
} ColorID;

/*
 * Theme — one named colour scheme (Fire, Ice, …) the user cycles with 't'.
 *
 * WHY it exists: the fractal maths only ever produces a ColorID slot per pixel
 * (COL_INSIDE, COL_C2..C5).  A Theme is the lookup that turns those abstract
 * slots into real on-screen colours, so we can restyle the whole picture by
 * swapping one table row — no recompute of the fractal needed.
 *
 * HOW to read the arrays: index order matches the escape bands, dim → bright,
 *   c[0] → COL_INSIDE   c[1] → COL_C2 (far/dim)  …  c[4] → COL_C5 (near/bright)
 *
 * WHY two arrays: c[] holds 256-colour indices (modern terminals); c8[] holds
 * the 8 classic ANSI colours as a fallback so old terminals still look sane.
 *
 * WHY every value is "bright": a 256-index from the dark bottom of the colour
 * cube is invisible on a black background.  All entries are picked from the
 * bright half, and the visible halo bands (C3..C5) are kept genuinely vivid;
 * only the contrast band C2 may be deep.  (See COLOR.md, ref [9].)
 */
typedef struct {
    const char *name;   /* shown in the HUD, e.g. "Fire"               */
    int c[5];           /* 256-colour indices: INSIDE, C2, C3, C4, C5  */
    int c8[5];          /* 8-colour fallback, same slot order          */
    int hud;            /* 256-colour index for HUD text               */
    int hud8;           /* 8-colour fallback for HUD text              */
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*              name      INSIDE  C2    C3    C4    C5    hud   (8-color fallback)                            hud8 */
    { "Fire",     {231, 160, 202, 214, 226},  {COLOR_WHITE,  COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW}, 226, COLOR_YELLOW  },
    { "Ember",    {230, 130, 172, 208, 220},  {COLOR_YELLOW, COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW}, 214, COLOR_YELLOW  },
    { "Magma",    {231, 124, 196, 209, 223},  {COLOR_WHITE,  COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW}, 220, COLOR_YELLOW  },
    { "Ice",      {231,  24,  31,  39, 159},  {COLOR_WHITE,  COLOR_BLUE,   COLOR_BLUE,   COLOR_CYAN,   COLOR_CYAN},   123, COLOR_CYAN    },
    { "Toxic",    {190,  28,  64, 154, 226},  {COLOR_GREEN,  COLOR_GREEN,  COLOR_GREEN,  COLOR_YELLOW, COLOR_YELLOW}, 154, COLOR_YELLOW  },
    { "Nebula",   {231,  93, 129, 171, 213},  {COLOR_WHITE,  COLOR_MAGENTA,COLOR_MAGENTA,COLOR_CYAN,   COLOR_CYAN},    87, COLOR_CYAN    },
    { "Mono",     {231, 243, 246, 249, 252},  {COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE},  255, COLOR_WHITE   },
};

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

/*
 * clamp_frame_dt — cap a single frame's elapsed time.
 *
 * If the program stalls (debugger, laptop sleep, slow terminal), a huge dt
 * would make the fixed-timestep loop run hundreds of catch-up ticks at once —
 * the "spiral of death".  Capping dt trades one slow frame for stability.
 */
static int64_t clamp_frame_dt(int64_t dt)
{
    int64_t cap = MAX_FRAME_MS * NS_PER_MS;
    return dt > cap ? cap : dt;
}

/*
 * frame_pace — sleep so the whole frame lasts about 1 / target_fps.
 * `frame_start` is when this frame began; `work_done` is time already spent,
 * so we sleep only the leftover budget instead of busy-waiting.
 */
static void frame_pace(int target_fps, int64_t frame_start, int64_t work_done)
{
    int64_t budget = TICK_NS(target_fps);
    int64_t spent  = clock_ns() - frame_start + work_done;
    clock_sleep_ns(budget - spent);
}

/*
 * FpsCounter — a small running readout of frames-per-second for the HUD.
 * It sums frames and elapsed time, then every FPS_UPDATE_MS divides the two
 * for a smooth value (raw per-frame fps jitters too much to read).
 */
typedef struct {
    int64_t accum_ns;   /* real time since the last readout update */
    int     frames;     /* frames counted since then               */
    double  value;      /* last computed fps — what the HUD shows  */
} FpsCounter;

static void fps_count_frame(FpsCounter *f, int64_t dt)
{
    f->frames   += 1;
    f->accum_ns += dt;
    if (f->accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {       /* time to refresh? */
        f->value    = (double)f->frames / ((double)f->accum_ns / (double)NS_PER_SEC);
        f->frames   = 0;
        f->accum_ns = 0;
    }
}

/* ===================================================================== */
/* §3  complex — the number system the fractal iterates in                */
/* ===================================================================== */

/*
 * Complex — one point in the complex plane, written z = re + im·i.
 *
 * WHAT it is, for newcomers: picture a 2-D point (re, im) that comes with a
 * special multiplication rule (see complex_square below).  That rule is the
 * whole reason fractals like this exist — repeatedly squaring-and-adding a
 * point traces out the intricate boundary.  (Background: ref [2].)
 *
 * WHY its own type: every screen pixel stands for one Complex number (its
 * coordinate c), and the running value z is Complex too.  Naming the type and
 * giving it add/square/fold helpers lets the §4 iteration read line-for-line
 * like the maths, instead of a tangle of zr/zi float variables.
 *
 * WHY float, not double: single precision is ~7 digits — plenty for the zoom
 * depths these presets reach, and faster in the millions-of-iterations loop.
 * Zooming far deeper than the presets would eventually require double.
 */
typedef struct {
    float re;   /* real part      — horizontal axis, grows rightward */
    float im;   /* imaginary part — vertical axis,   grows upward    */
} Complex;

/* z = a + b */
static inline Complex complex_add(Complex a, Complex b)
{
    return (Complex){ a.re + b.re, a.im + b.im };
}

/* z² = (re² − im²) + (2·re·im)·i */
static inline Complex complex_square(Complex z)
{
    return (Complex){ z.re * z.re - z.im * z.im, 2.0f * z.re * z.im };
}

/* The burning-ship fold: reflect both parts into the first quadrant,
 * |re| + |im|·i.  This single line is what distinguishes the ship from
 * the Mandelbrot set. */
static inline Complex complex_abs_fold(Complex z)
{
    return (Complex){ fabsf(z.re), fabsf(z.im) };
}

/* |z|² = re² + im²  (squared modulus — compared against ESCAPE_RADIUS_SQ) */
static inline float complex_norm_sq(Complex z)
{
    return z.re * z.re + z.im * z.im;
}

/* ===================================================================== */
/* §4  fractal — the escape-time algorithm                                */
/* ===================================================================== */

/*
 * burning_ship_escape — how many steps the orbit of c survives before it
 * escapes the radius-2 disc.  Returns MAX_ITER if it never escapes (c is
 * inside the set).
 *
 * The loop body is the definition itself:
 *     z ← (fold |·| then square)(z)  +  c
 */
static int burning_ship_escape(Complex c)
{
    Complex z = { 0.0f, 0.0f };
    for (int n = 0; n < MAX_ITER; n++) {
        z = complex_add(complex_square(complex_abs_fold(z)), c);
        if (complex_norm_sq(z) > ESCAPE_RADIUS_SQ)
            return n;
    }
    return MAX_ITER;
}

/*
 * escape_to_band — turn an escape count into a Canvas colour slot.
 *
 *   never escaped (== MAX_ITER) → COL_INSIDE
 *   escaped in first 8 %        → 0 (blank: far exterior)
 *   otherwise                   → COL_C2..COL_C5, slow escape = brighter
 *
 * Slow escape ⇒ near the set boundary ⇒ vivid; fast escape ⇒ far ⇒ dim.
 */
static uint8_t escape_to_band(int escape)
{
    if (escape >= MAX_ITER) return COL_INSIDE;          /* never escaped → the set */

    float frac = (float)escape / (float)MAX_ITER;       /* 0 = instant … →1 = slow */
    if (frac < BACKGROUND_FRAC) return 0;               /* far exterior → blank    */

    /* halo_pos: where in the coloured halo this pixel sits — rescale the
     * surviving range [BACKGROUND_FRAC, 1) onto [0, 1). */
    float halo_pos = (frac - BACKGROUND_FRAC) / (1.0f - BACKGROUND_FRAC);
    int   n_bands  = COL_C5 - COL_C2 + 1;               /* 4 escape bands          */
    int   band     = COL_C2 + (int)(halo_pos * (float)n_bands);
    return (uint8_t)(band > COL_C5 ? COL_C5 : band);
}

/* ===================================================================== */
/* §5  view — the lens between screen cells and the complex plane         */
/* ===================================================================== */

/*
 * Preset — one saved "place to look": where to aim, and how far to zoom in.
 * The k_presets table below is a guided tour of the fractal.
 *
 * WHY centre + half-height (not corners): it matches how zooming works.  To
 * zoom in we keep the centre and shrink im_half — the view stays put and just
 * gets tighter.  Storing corners would mean recomputing them on every zoom.
 *
 * WHY only the vertical half: the horizontal half is computed at runtime from
 * im_half and the live terminal size (view_from_preset), so the picture is
 * never squashed when the window is a different shape or gets resized.
 *
 * WHERE the numbers came from: each was found by an offline render scan, not
 * guessed — every preset lands on rich, detailed set boundary.  The famous
 * "ship" sits near c = -1.755 - 0.035i (ref [1]); the flames and distant fleet
 * are too sparse to fill a terminal, so the tour zooms into the hull instead.
 */
typedef struct {
    const char *name;       /* HUD label, e.g. "the ship"               */
    float       center_re;  /* real part of the point at screen centre  */
    float       center_im;  /* imaginary part of that centre point      */
    /* half the view's HEIGHT in plane units, and the zoom knob:
       smaller im_half = deeper zoom.  e.g. 1.0 = whole fractal, 0.005 = deep. */
    float       im_half;
} Preset;

static const Preset k_presets[N_PRESETS] = {
    { "full armada",   -0.65000f, -0.45000f, 1.00000f },
    { "the ship",      -1.75500f, -0.03500f, 0.04500f },
    { "hull & masts",  -1.77000f, -0.02200f, 0.01800f },
    { "mini-armada",   -1.75600f, -0.02650f, 0.01000f },
    { "antenna sweep", -1.75650f, -0.02650f, 0.00500f },
};

/*
 * ViewWindow — the actual rectangle of the plane being drawn right now.
 *
 * Relationship to Preset: a Preset is the saved recipe (centre + zoom); a
 * ViewWindow is what you get after baking in the LIVE terminal size, so its
 * half_re is already aspect-corrected and ready to use.  view_from_preset
 * turns the former into the latter.
 *
 * WHY it matters: this struct is the single bridge between two worlds — screen
 * cells (col,row, whole numbers) and plane points (Complex, floats).  Keeping
 * that conversion in one place (view_sample) means the fractal maths never
 * needs to know anything about terminals.
 */
typedef struct {
    Complex center;    /* plane point shown at the middle of the screen      */
    float   half_re;   /* half-width  in real units (widened for cell shape) */
    float   half_im;   /* half-height in imaginary units (= preset im_half)  */
} ViewWindow;

/*
 * aspect_correct_half_re — the real half-extent that keeps the picture from
 * looking squashed.  Terminal cells are ~2× taller than wide (ASPECT_R), and
 * the window has its own width:height ratio; scale the vertical half by both
 * so one plane-unit covers the same visual distance across and down.
 */
static float aspect_correct_half_re(float half_im, int cols, int rows)
{
    return half_im * (float)cols / (float)rows / ASPECT_R;
}

/* Build the live view for a preset at the current terminal size. */
static ViewWindow view_from_preset(const Preset *p, int cols, int rows)
{
    ViewWindow v;
    v.center  = (Complex){ p->center_re, p->center_im };
    v.half_im = p->im_half;
    v.half_re = aspect_correct_half_re(p->im_half, cols, rows);
    return v;
}

/*
 * view_sample — which complex number does screen cell (col, row) stand for?
 *
 * Turn each axis into a centre-relative offset in [-1, +1], then scale it by
 * the view's half-extent and add the centre.  The vertical offset is NEGATED
 * so moving DOWN the screen moves DOWN the imaginary axis — that flip is what
 * makes the ship float hull-up instead of hanging upside-down.
 */
static Complex view_sample(const ViewWindow *v, int col, int row, int cols, int rows)
{
    float fx = (float)col / (float)(cols - 1);   /* 0 → 1 left to right */
    float fy = (float)row / (float)(rows - 1);   /* 0 → 1 top to bottom */

    float off_re =  (fx - 0.5f) * 2.0f;          /* −1 left edge  … +1 right edge  */
    float off_im = -(fy - 0.5f) * 2.0f;          /* +1 top  edge  … −1 bottom (flip)*/

    return (Complex){
        .re = v->center.re + off_re * v->half_re,
        .im = v->center.im + off_im * v->half_im,
    };
}

/* ===================================================================== */
/* §6  canvas — the grid of coloured cells we paint to the terminal       */
/* ===================================================================== */

/*
 * Canvas — the picture held in memory: one ColorID per terminal cell.
 *
 * WHY have a buffer at all: it cleanly separates "decide the colour" (the
 * fractal maths) from "put it on screen" (ncurses).  The algorithm only ever
 * writes a number into band[row][col]; canvas_paint is the ONE function that
 * turns those numbers into glyphs.  Either half can change without the other.
 *
 * WHY uint8_t: a cell only needs a small ColorID (0..7), so one byte is plenty
 * and keeps the buffer compact.  0 = blank/not-computed; 1..5 = a fractal band.
 *
 * WHY fixed-size arrays: this project never calls malloc in the running loop
 * (see CLAUDE.md).  The arrays are sized to the biggest terminal we support
 * (CANVAS_*_MAX); the live rows/cols say how much of them is actually in use.
 */
typedef struct {
    int     rows, cols;                              /* portion in use       */
    uint8_t band[CANVAS_ROWS_MAX][CANVAS_COLS_MAX];  /* [row][col] → ColorID */
} Canvas;

static void canvas_reset(Canvas *cv, int cols, int rows)
{
    if (cols > CANVAS_COLS_MAX) cols = CANVAS_COLS_MAX;
    if (rows > CANVAS_ROWS_MAX) rows = CANVAS_ROWS_MAX;
    cv->cols = cols;
    cv->rows = rows;
    memset(cv->band, 0, sizeof cv->band);
}

/*
 * band_glyph — the character drawn for a colour slot.  The ramp gets denser
 * from far exterior (',') toward the set interior ('*'), so glyph weight and
 * colour brightness reinforce each other.
 */
static chtype band_glyph(uint8_t slot)
{
    static const char glyph[8] = {
        [COL_INSIDE] = '*',
        [COL_C2]     = ',',
        [COL_C3]     = '.',
        [COL_C4]     = '+',
        [COL_C5]     = '#',
    };
    return (chtype)(unsigned char)glyph[slot];
}

/*
 * band_attr — the ncurses attributes for a slot: its colour pair, plus A_BOLD
 * on the interior and bright inner halo so they pop against the dimmer bands.
 */
static attr_t band_attr(uint8_t slot)
{
    attr_t attr = COLOR_PAIR((int)slot);
    if (slot == COL_INSIDE || slot == COL_C5) attr |= A_BOLD;
    return attr;
}

/* canvas_paint — blit every non-blank cell to the window. */
static void canvas_paint(const Canvas *cv, WINDOW *w)
{
    for (int row = 0; row < cv->rows; row++) {
        for (int col = 0; col < cv->cols; col++) {
            uint8_t slot = cv->band[row][col];
            if (slot == 0) continue;                 /* not computed / background */
            attr_t attr = band_attr(slot);
            wattron(w, attr);
            mvwaddch(w, row, col, band_glyph(slot));
            wattroff(w, attr);
        }
    }
}

/* ===================================================================== */
/* §7  reveal — the random-fill animation                                 */
/* ===================================================================== */

/*
 * Reveal — decides the ORDER pixels are filled in, purely for visual effect:
 * the fractal condenses out of random dots instead of wiping top-to-bottom.
 *
 * HOW: order[] lists every pixel's position once, then we shuffle it with the
 * Fisher-Yates algorithm — walk from the last slot to the first, swapping each
 * with a randomly chosen earlier slot.  That yields a perfectly uniform
 * shuffle in O(n) (ref: Knuth, "The Art of Computer Programming" Vol. 2,
 * §3.4.2, "Algorithm P").
 *
 * Positions are FLATTENED: a cell (row,col) is stored as one integer
 * row*cols + col, so the whole 2-D grid fits in a single 1-D array we can
 * shuffle.  Unflatten with  col = idx % cols,  row = idx / cols (see
 * scene_color_pixel).
 *
 * `done` is simply how far along order[] we've computed; each tick advances it
 * by PIXELS_PER_TICK until it reaches `total`.
 */
typedef struct {
    int order[CANVAS_ROWS_MAX * CANVAS_COLS_MAX];  /* shuffled flat indices */
    int total;   /* how many pixels this view has (rows * cols)            */
    int done;    /* how many of them have been computed so far             */
} Reveal;

/*
 * fisher_yates_shuffle — uniformly shuffle a[0..n) in place, O(n).
 * Walk from the back; swap each slot with a randomly chosen earlier-or-equal
 * slot, which makes every ordering equally likely.  (Knuth TAOCP Vol. 2,
 * §3.4.2, "Algorithm P".)
 */
static void fisher_yates_shuffle(int *a, int n)
{
    for (int i = n - 1; i > 0; i--) {
        int j = rand() % (i + 1);                   /* uniform pick in [0, i] */
        int tmp = a[i]; a[i] = a[j]; a[j] = tmp;     /* swap into final spot   */
    }
}

static void reveal_shuffle(Reveal *r, int n_pixels)
{
    r->total = n_pixels;
    r->done  = 0;
    for (int i = 0; i < n_pixels; i++)              /* list every pixel once     */
        r->order[i] = i;
    fisher_yates_shuffle(r->order, n_pixels);       /* randomise the visit order */
}

static bool reveal_complete(const Reveal *r) { return r->done >= r->total; }

static int reveal_percent(const Reveal *r)
{
    return r->total ? (int)((long)r->done * 100 / r->total) : 100;
}

/* ===================================================================== */
/* §8  color — bind a Theme to ncurses colour pairs                       */
/* ===================================================================== */

/*
 * bind_palette — point the five fractal slots + the HUD slot at a list of
 * foreground colours, all on black.  Called with either the 256-colour or the
 * 8-colour list, so the repetitive init_pair wiring lives in exactly one place.
 */
static void bind_palette(const int band_fg[5], int hud_fg)
{
    init_pair(COL_INSIDE, band_fg[0], COLOR_BLACK);
    init_pair(COL_C2,     band_fg[1], COLOR_BLACK);
    init_pair(COL_C3,     band_fg[2], COLOR_BLACK);
    init_pair(COL_C4,     band_fg[3], COLOR_BLACK);
    init_pair(COL_C5,     band_fg[4], COLOR_BLACK);
    init_pair(COL_HUD,    hud_fg,     COLOR_BLACK);
}

static void theme_apply(int theme_id)
{
    const Theme *th = &k_themes[theme_id];
    bool truecolor  = COLORS >= 256;                       /* modern vs legacy term */

    bind_palette(truecolor ? th->c   : th->c8,
                 truecolor ? th->hud : th->hud8);
    init_pair(COL_HINT, truecolor ? HINT_CYAN : COLOR_CYAN, COLOR_BLACK);
}

static void color_init(void)
{
    start_color();
    theme_apply(0);   /* Fire; Scene.theme_id tracks the active theme thereafter */
}

/* ===================================================================== */
/* §9  scene — everything needed to render one fractal view               */
/* ===================================================================== */

/*
 * Scene — the whole "what is on screen" state in one place.  Everything the
 * program does reduces to: update a Scene (scene_tick), then draw it
 * (scene_draw).
 *
 * WHY bundle them: view, canvas and reveal are useless apart — you need the
 * lens to know which point a cell is, the canvas to store the colour, and the
 * reveal to know what to compute next.  Grouping them lets one pointer carry
 * the entire picture between functions.
 *
 * WHY ids, not pointers: preset_id / theme_id are plain indices into the
 * k_presets / k_themes tables.  An index is trivial to cycle ( (id+1) % N ),
 * reads straight into the HUD as "view 2/5", and can never dangle.
 */
typedef struct {
    int        preset_id;   /* which k_presets entry is shown (0..N_PRESETS-1) */
    int        theme_id;    /* which k_themes palette is active                */
    ViewWindow view;        /* the plane rectangle this preset maps to         */
    Canvas     canvas;      /* the coloured cells built up so far              */
    Reveal     reveal;      /* how far through the random fill we are          */
    bool       paused;      /* true = stop revealing new pixels                */
} Scene;

/* Point the scene at a preset: rebuild the lens, clear the canvas, reshuffle. */
static void scene_load(Scene *s, int preset_id, int cols, int rows)
{
    s->preset_id = preset_id % N_PRESETS;
    s->view      = view_from_preset(&k_presets[s->preset_id], cols, rows);
    canvas_reset(&s->canvas, cols, rows);
    reveal_shuffle(&s->reveal, s->canvas.rows * s->canvas.cols);
}

static void scene_init(Scene *s, int cols, int rows)
{
    s->theme_id = 0;
    s->paused   = false;
    scene_load(s, 0, cols, rows);
}

/*
 * scene_color_pixel — compute and store the colour of ONE canvas cell.
 * The per-pixel heart of the program, in four named steps:
 *   1. unflatten the shuffled index back into (row, col)
 *   2. ask the lens which complex point that cell is
 *   3. run the escape algorithm, turn the count into a colour band
 *   4. record the band (skip pure-background cells, leaving them blank)
 */
static void scene_color_pixel(Scene *s, int flat_index)
{
    int col = flat_index % s->canvas.cols;          /* unflatten: see Reveal docs */
    int row = flat_index / s->canvas.cols;

    Complex point = view_sample(&s->view, col, row, s->canvas.cols, s->canvas.rows);
    uint8_t band  = escape_to_band(burning_ship_escape(point));

    if (band) s->canvas.band[row][col] = band;
}

/*
 * scene_tick — reveal the next batch of pixels, then stop.
 * Computes PIXELS_PER_TICK cells from the shuffled order; holds once the
 * reveal is complete (presets do not auto-advance).
 */
static void scene_tick(Scene *s)
{
    if (s->paused || reveal_complete(&s->reveal)) return;   /* nothing to reveal */

    int batch_end = s->reveal.done + PIXELS_PER_TICK;       /* how far this tick */
    if (batch_end > s->reveal.total) batch_end = s->reveal.total;

    for (int i = s->reveal.done; i < batch_end; i++)
        scene_color_pixel(s, s->reveal.order[i]);           /* colour one cell   */

    s->reveal.done = batch_end;
}

static void scene_next_preset(Scene *s, int cols, int rows)
{
    scene_load(s, (s->preset_id + 1) % N_PRESETS, cols, rows);
}

static void scene_next_theme(Scene *s)
{
    s->theme_id = (s->theme_id + 1) % N_THEMES;
    theme_apply(s->theme_id);
}

static void scene_draw(const Scene *s, WINDOW *w) { canvas_paint(&s->canvas, w); }

/* ===================================================================== */
/* §10  screen — ncurses lifecycle + HUD                                  */
/* ===================================================================== */

/*
 * Screen — the live terminal size in character cells.  Re-measured at startup
 * and after every resize (SIGWINCH); used to lay out the HUD and to rebuild the
 * ViewWindow so the fractal always fills the current window.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
} Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc)  { (void)sc; endwin(); }

static void screen_resize(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/* hud_data_line — row 0, right-aligned bold: the live "what's happening" data. */
static void hud_data_line(const Screen *sc, const Scene *s, double fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " BurningShip  %5.1f fps  view %d/%d  %s ",
             fps, s->preset_id + 1, N_PRESETS, s->paused ? "PAUSED " : "running");

    int right_x = sc->cols - (int)strlen(buf);          /* push flush to the right */
    if (right_x < 0) right_x = 0;

    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, right_x, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* hud_params_line — row 1, plain: the parameters behind the current view. */
static void hud_params_line(const Scene *s, int sim_fps)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %-14s  %3d%%  spd:%d Hz  theme:%s ",
             k_presets[s->preset_id].name, reveal_percent(&s->reveal),
             sim_fps, k_themes[s->theme_id].name);

    attron(COLOR_PAIR(COL_HUD));
    mvprintw(1, 0, "%s", buf);
    attroff(COLOR_PAIR(COL_HUD));
}

/* hud_action_line — last row, bright cyan: every interactive key. */
static void hud_action_line(const Screen *sc)
{
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  n:next view  spc:pause  t:theme  [ / ]:speed ");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

/*
 * screen_draw — compose one frame: the fractal, then the HUD.
 * HUD layout is fixed — top is data, bottom is actions:
 *   row 0      live data       (hud_data_line)
 *   row 1      parameters      (hud_params_line)
 *   row rows-1 key actions     (hud_action_line)
 */
static void screen_draw(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_draw(s, stdscr);            /* the fractal itself     */
    hud_data_line(sc, s, fps);        /* row 0   — live data    */
    hud_params_line(s, sim_fps);      /* row 1   — parameters   */
    hud_action_line(sc);              /* last row — key actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §11  app — signals, input, main loop                                   */
/* ===================================================================== */

/*
 * App — the top-level container main() owns: the picture (scene), the terminal
 * it lives in (screen), and the one tunable speed (sim_fps).  Kept as a single
 * struct so the loop passes "everything" with one &app.
 */
typedef struct {
    Scene  scene;     /* the fractal picture + all its sub-state               */
    Screen screen;    /* current terminal dimensions                           */
    int    sim_fps;   /* reveal speed, ticks/sec (SIM_FPS_MIN..MAX); ']' / '[' */
} App;

/* The only globals: POSIX signal handlers take no arguments, so they must
 * write a flag the main loop polls. */
static volatile sig_atomic_t g_should_quit   = 0;
static volatile sig_atomic_t g_should_resize = 0;

static void on_stop (int sig) { (void)sig; g_should_quit   = 1; }
static void on_winch(int sig) { (void)sig; g_should_resize = 1; }
static void cleanup (void)    { endwin(); }

static void app_resize(App *a)
{
    screen_resize(&a->screen);
    scene_load(&a->scene, a->scene.preset_id, a->screen.cols, a->screen.rows);
    g_should_resize = 0;
}

/* app_change_speed — nudge the reveal rate by delta, clamped to its range. */
static void app_change_speed(App *a, int delta)
{
    int fps = a->sim_fps + delta;
    if (fps < SIM_FPS_MIN) fps = SIM_FPS_MIN;
    if (fps > SIM_FPS_MAX) fps = SIM_FPS_MAX;
    a->sim_fps = fps;
}

/* app_handle_key — map one keypress to an action.  Returns false to quit. */
static bool app_handle_key(App *a, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:  return false;                    /* quit          */
    case 'r': case 'R':
    case 'n': case 'N':  scene_next_preset(&a->scene,
                                  a->screen.cols, a->screen.rows);  break; /* next view */
    case 't': case 'T':  scene_next_theme(&a->scene);              break; /* palette   */
    case 'p': case 'P':
    case ' ':            a->scene.paused = !a->scene.paused;       break; /* pause     */
    case ']':            app_change_speed(a, +SIM_FPS_STEP);       break; /* faster    */
    case '[':            app_change_speed(a, -SIM_FPS_STEP);       break; /* slower    */
    default: break;
    }
    return true;
}

/*
 * app_run_due_ticks — fixed-timestep catch-up.
 * Add the elapsed time to an accumulator and spend one scene_tick per whole
 * tick's worth, so the reveal advances at sim_fps regardless of frame rate.
 */
static void app_run_due_ticks(App *a, int64_t *sim_accum, int64_t dt)
{
    int64_t tick_ns = TICK_NS(a->sim_fps);
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&a->scene);
        *sim_accum -= tick_ns;
    }
}

/* app_poll_input — handle at most one pending key this frame. */
static void app_poll_input(App *a)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(a, ch))
        g_should_quit = 1;
}

int main(void)
{
    srand((unsigned int)clock_ns());

    atexit(cleanup);
    signal(SIGINT,   on_stop);
    signal(SIGTERM,  on_stop);
    signal(SIGWINCH, on_winch);

    /* static (BSS) — Scene holds large arrays; keep them off the stack */
    static App app = { .sim_fps = SIM_FPS_DEFAULT };

    screen_init(&app.screen);
    scene_init(&app.scene, app.screen.cols, app.screen.rows);

    int64_t    frame_time = clock_ns();
    int64_t    sim_accum  = 0;        /* elapsed time not yet spent on ticks */
    FpsCounter fps        = {0};

    /* Main loop — one pass per displayed frame:
     *   1. apply a pending resize
     *   2. measure elapsed time, clamped against stalls
     *   3. run the simulation ticks that time owes
     *   4. update the fps readout
     *   5. pace the frame, then draw it
     *   6. handle one keypress
     */
    while (!g_should_quit) {

        if (g_should_resize) {
            app_resize(&app);
            frame_time = clock_ns();             /* don't bill resize as elapsed */
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = clamp_frame_dt(now - frame_time);
        frame_time  = now;

        app_run_due_ticks(&app, &sim_accum, dt);            /* advance the reveal     */
        fps_count_frame(&fps, dt);                          /* update the fps readout */

        frame_pace(RENDER_FPS, frame_time, dt);             /* cap the display rate   */
        screen_draw(&app.screen, &app.scene, fps.value, app.sim_fps);
        screen_present();

        app_poll_input(&app);                               /* one key → action/quit  */
    }

    screen_free(&app.screen);
    return 0;
}
