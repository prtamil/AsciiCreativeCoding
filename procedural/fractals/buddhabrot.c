/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * buddhabrot.c — the Buddhabrot fractal, drawn as a heatmap in the terminal.
 *
 * The idea: throw lots of random points c into the complex plane and, for
 * each, run the Mandelbrot iteration z = z*z + c starting from z = 0.  As we
 * iterate, z hops around the plane — that path is its "orbit".  We tally how
 * often each screen cell gets visited across millions of orbits.  The busiest
 * cells glow bright, and the pile-up slowly reveals a ghostly figure.  Two
 * flavours: BUDDHA tracks orbits that fly off to infinity (paints the famous
 * meditating-figure outline); ANTI tracks orbits that stay trapped (paints
 * the set's interior).  Only the ANTI preset ships; BUDDHA code is left in.
 *
 * Sister file: mandelbrot.c (same z=z*z+c iteration, colored by escape time
 * instead of orbit density).
 *
 * Keys: q/ESC quit · n/r next preset · p/spc pause · [/] speed · t/T theme
 * Build: gcc -std=c11 -O2 -Wall -Wextra buddhabrot.c -o buddhabrot -lncurses -lm
 *
 * References (for the things the code can't tell you):
 *   ── Green, M., "The Buddhabrot Technique", 1993 — the original idea of
 *      plotting orbits instead of escape times.  http://superliminal.com/fractals/bbrot/
 *   ── Draves & Reckase, "The Fractal Flame Algorithm", 2003 — the
 *      accumulate-into-a-density-buffer + log brightness trick used here.
 *   ── Bourke, P., "Character representation of greyscale images", 1997 —
 *      the '.' ':' '+' '#' '@' brightness ramp.
 *
 * §1 types & data   §2 random          §3 time
 * §4 view & palette §5 density grid    §6 Mandelbrot & chaos game
 * §7 render & HUD   §8 scene & main
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>          /* memset */
#include <time.h>

/* ===================================================================== */
/* §1  types & data                                                       */
/* ===================================================================== */

/* ---- grid + HUD sizing ------------------------------------------ */
#define GRID_ROWS_MAX        80
#define GRID_COLS_MAX       300
#define HUD_TOP_ROWS          1     /* status line at the top             */
#define HUD_BOTTOM_ROWS       1     /* key hints at the bottom            */

/* ---- iteration / pacing ----------------------------------------- */
#define SIM_FPS_MIN          10
#define SIM_FPS_DEFAULT      30
#define SIM_FPS_MAX          60
#define SIM_FPS_STEP          5

#define SAMPLES_PER_TICK   2000     /* random points tried each tick           */
#define TOTAL_SAMPLES    500000     /* stop tallying once we've tried this many */

/* ---- timing ----------------------------------------------------- */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define FRAME_NS_60FPS (NS_PER_SEC / 60)       /* draw at most 60 times a second */
#define FPS_WINDOW_MS         500              /* average the fps over half a sec */
#define MAX_DT_NS    (100 * NS_PER_MS)         /* ignore stalls longer than this  */

/* ---- Mandelbrot iteration parameters ---------------------------- */
#define ESCAPE_RADIUS_SQ   4.0f     /* once z gets farther than 2 from 0, it's gone */
#define MANDELBROT_Z0_RE   0.0f     /* every orbit starts at z = 0        */
#define MANDELBROT_Z0_IM   0.0f
#define CARDIOID_CUSP_X    0.25f    /* shape constants for the two big "always */
#define PERIOD2_BULB_R2    0.0625f  /* trapped" lobes we can skip-test fast    */
#define PERIOD2_BULB_CX   -1.0f

/* ---- aspect-correct projection ---------------------------------- */
#define ASPECT_CELL_HEIGHT    2.0f    /* terminal cells are about 2x taller than wide */
#define ASPECT_INV            0.5f    /* so squash vertical distances by half         */

/* ---- density tiers ---------------------------------------------- */
#define N_DENSITY_TIERS       5     /* five brightness levels, '.' up to '@' */
#define DENSITY_FLOOR_BUDDHA  0.08f /* dimmer than this stays blank (just noise) */
#define DENSITY_FLOOR_ANTI    0.25f /* anti's hits clump bright, so blank more   */
#define EPS_LOG_MAX           1e-12f /* tiny guard so we never divide by ~zero    */

/* ---- view rectangle --------------------------------------------- */
#define VIEW_RE_HALF       1.75f   /* half the figure's height in math units */
#define VIEW_CENTER_RE    -0.5f    /* the figure's center of mass            */
#define VIEW_CENTER_IM     0.0f    /* it's mirror-symmetric, so center on 0  */

/* ---- HUD ------------------------------------------------------- */
#define HUD_PCT_FULL         100   /* 100% = done tallying                    */

/* ---- LCG constants (Knuth's "minimal standard") ----------------- */
#define LCG_MULTIPLIER     1664525u
#define LCG_INCREMENT   1013904223u
#define LCG_HIGH_BITS_SHIFT     8u      /* keep the top bits; the bottom ones are junk */
#define LCG_UNIT_DENOM     (1u << 24)   /* divide by this to land in [0, 1)            */
#define LCG_SEED_TIME_MIX   123456789LL

/* ---- defaults & input ------------------------------------------- */
#define KEY_ESCAPE          27
#define DEFAULT_PRESET_IDX   0
#define DEFAULT_PALETTE_IDX  0

/* ---- glyphs ------------------------------------------------------ */
#define GLYPH_L1    '.'
#define GLYPH_L2    ':'
#define GLYPH_L3    '+'
#define GLYPH_L4    '#'
#define GLYPH_L5    '@'

/* ---- HUD --------------------------------------------------------- */
#define HUD_BUFFER_BYTES    160

/* ---- Complex — one point on the plane the fractal lives on -------- *
 *
 * Just a pair of coordinates: re is the horizontal spot, im the
 * vertical.  Everything in this program — the random points we test,
 * the orbit as it hops around — is a Complex.  We bundle the two
 * floats together so the math helpers can pass a point around as one
 * value and read like the formula (z = z*z + c) instead of juggling
 * four loose numbers.  Floats (not doubles) are plenty: a terminal
 * cell is coarser than float precision anyway.
 *
 *   re — horizontal coordinate (the "real" part).
 *   im — vertical coordinate (the "imaginary" part). */
typedef struct {
    float re, im;
} Complex;

/* ---- ComplexRect — the box we throw random points into ----------- *
 *
 * The four edges of the region we sample from.  We pick each test
 * point uniformly at random inside this box.  There's one of these
 * (g_sample_box), drawn a little wider than the fractal itself so we
 * also catch the points just outside it whose orbits sweep through.
 *
 *   re_min, re_max — left and right edges.
 *   im_min, im_max — bottom and top edges. */
typedef struct {
    float re_min, re_max;
    float im_min, im_max;
} ComplexRect;

/* ---- BuddhMode — which orbits we keep ----------------------------- *
 *
 * Picks which test points are worth drawing.  The two modes share
 * almost all their code; they differ only in this one choice.
 *
 *   MODE_BUDDHA — keep points whose orbit escapes to infinity.
 *                 These trace the outline of the set and pile up into
 *                 the famous meditating-figure shape.
 *   MODE_ANTI   — keep points whose orbit stays trapped forever.
 *                 These fill in the interior and show its structure. */
typedef enum {
    MODE_BUDDHA,
    MODE_ANTI,
} BuddhMode;

/* ---- MandelbrotPreset — one ready-made variant to look at -------- *
 *
 * A named set of knobs that fully describes one picture.  The drawing
 * code is identical for every preset; only these settings change, so
 * adding a variant is just one more entry in g_presets[].
 *
 *   name      — short label for the status line (keep <= 11 chars).
 *   max_iter  — how many steps to follow each orbit before giving up.
 *               More steps = finer detail but slower.  ~100 is rough,
 *               1000 (used here) is a good fit for a terminal,
 *               10000+ only pays off at print resolution.
 *   mode      — buddha or anti; see BuddhMode above. */
typedef struct {
    const char *name;
    int         max_iter;
    BuddhMode   mode;
} MandelbrotPreset;

/* ---- Palette — one color theme, five brightness levels ----------- *
 *
 * The five colors used from faintest cell to brightest.  Keeping
 * colors in here (instead of sprinkled through the drawing code) means
 * a new theme is just one more entry, and t/T can swap themes by
 * reloading these five colors.  By convention the faintest is a dark
 * tone for the outer haze and the brightest is near-white for the hot
 * core; the levels in between walk across different hues so each
 * brightness step is easy to tell apart.
 *
 *   name    — short label for the status line (keep <= 6 chars).
 *   fg256[] — colors for the five levels on a 256-color terminal.
 *   fg8[]   — fallback colors for an old 8-color terminal. */
typedef struct {
    const char *name;
    int  fg256[N_DENSITY_TIERS];
    int  fg8[N_DENSITY_TIERS];
} Palette;

/* ---- RandLCG — a tiny, fast random-number generator -------------- *
 *
 * Holds one number and shuffles it with a multiply and an add to get
 * the next "random" value.  We pick hundreds of thousands of points
 * per second, so we want randomness that costs almost nothing rather
 * than the high-quality (but slower) library generators.  This kind is
 * known to be sloppy in its lowest bits, so lcg_unit_float only uses
 * the better top bits.  Good enough for scattering dots; don't use it
 * for anything that needs real randomness.
 *
 *   state — the one number it carries.  Must never be zero (a zero
 *           gets stuck at zero forever).  Seeded once from the clock. */
typedef struct {
    uint32_t state;
} RandLCG;

/* ---- DensityGrid — the tally of how busy each cell is ------------ *
 *
 * One counter per screen cell, holding how many orbits have passed
 * through it.  This is the whole trick: a single orbit barely shows
 * up, but after millions of them the busy cells stand out and the
 * figure appears.  We also remember the single busiest count
 * (max_count) so the renderer knows what "fully bright" means.  We
 * keep max_count up to date as we go rather than rescanning the whole
 * grid every frame — safe because counts only ever go up.
 *
 * The grid is a fixed size (the biggest terminal we support) so it can
 * just sit in static memory with no allocating or resizing to worry
 * about.
 *
 *   counts[row][col] — visit count per cell, screen coordinates (row 0
 *                      is the top).  32 bits is far more headroom than
 *                      any real run needs.
 *   max_count        — the highest count seen anywhere in the grid. */
typedef struct {
    uint32_t counts[GRID_ROWS_MAX][GRID_COLS_MAX];
    uint32_t max_count;
} DensityGrid;

/* ---- Viewport — how to place a math point onto the screen -------- *
 *
 * The few numbers needed to turn a point on the plane into a screen
 * cell (a column and a row).  Keeping them in one place means the
 * drawing loop just asks "where does this point go?" and doesn't care
 * about terminal size or the reserved HUD rows; on a resize we redo
 * these numbers once and everything follows.
 *
 * Two quirks worth knowing:
 *
 *  - We turn the figure on its side so it stands tall in the terminal:
 *    its vertical axis runs left-to-right across the screen, and its
 *    horizontal axis runs top-to-bottom (head at top, body downward).
 *
 *  - Terminal cells are about twice as tall as they are wide, so a
 *    step "down" covers more ground than a step "across".  We measure
 *    everything in cell-widths and halve the vertical part to keep the
 *    figure from looking stretched.  `scale` (cells per math unit) is
 *    chosen to fit the figure both ways without distorting it — bigger
 *    terminal, bigger drawing.
 *
 *   rows, cols    — terminal size in cells.
 *   play_rows     — drawable rows left after the two HUD rows; cached
 *                   so we don't redo the subtraction.
 *   center_col    — column the figure's center lands on.
 *   center_row    — row the figure's center lands on.
 *   scale         — cells per math unit; the one zoom knob.
 *   view_mid_re   — the math point that sits at screen center,
 *   view_mid_im     horizontal and vertical (the figure's center of
 *                   mass; vertical is 0 since it's mirror-symmetric). */
typedef struct {
    int    rows, cols;
    int    play_rows;
    int    center_col, center_row;
    float  scale;
    float  view_mid_re, view_mid_im;
} Viewport;

/* ---- FpsCounter — a steady frames-per-second readout ------------- *
 *
 * Counts how many frames happen over about half a second, then divides
 * to get a frame rate for the status line.  We average instead of
 * timing a single frame because one frame's time bounces around wildly
 * (a hiccup here, a fast frame there); the average reads as one calm
 * number that updates a couple of times a second.
 *
 *   accum_ns — time piled up so far in this half-second window.
 *   frames   — frames counted so far in this window.
 *   display  — the last computed rate; what the status line shows. */
typedef struct {
    long long accum_ns;
    int       frames;
    double    display;
} FpsCounter;

/* ---- Scene — everything the running program needs ---------------- *
 *
 * One struct holding the whole live state, so the main loop reads as a
 * short list of steps on one thing (handle keys, advance, draw) and
 * every helper says up front which parts it touches instead of poking
 * at hidden globals.
 *
 *   What we're drawing:
 *     grid         — the visit tally; cleared on reset/resize/preset.
 *     preset       — which variant is active (index into g_presets[]).
 *     samples_done — points tried so far; once it hits TOTAL_SAMPLES
 *                    we stop and the picture holds.
 *     rng          — random source, seeded once from the clock.
 *
 *   How we show it:
 *     view         — point-to-screen placement; redone on resize.
 *     palette      — which color theme is active (index into
 *                    g_palettes[]); changing it doesn't touch the grid.
 *
 *   Controls:
 *     paused       — freeze the work (status line keeps updating).
 *     sim_fps      — how hard to work each second; higher fills the
 *                    picture faster but uses more CPU.
 *
 *   Timing:
 *     fps          — frame-rate readout for the status line. */
typedef struct {
    /* what we're computing */
    DensityGrid    grid;
    int            preset;
    int            samples_done;
    RandLCG        rng;

    /* how we view it */
    Viewport       view;
    int            palette;

    /* control */
    bool           paused;
    int            sim_fps;

    /* timing */
    FpsCounter     fps;
} Scene;

/* ---- colour-pair slots ------------------------------------------ */
enum {
    COL_C1   = 1,   /* sparsest density tier (rim)                   */
    COL_C2   = 2,
    COL_C3   = 3,
    COL_C4   = 4,
    COL_C5   = 5,   /* densest tier (peak, drawn bold)               */
    COL_HUD  = 6,   /* top data row — bright yellow                  */
    COL_HINT = 7,   /* bottom hint row — bright cyan                 */
};

/* ---- sample box (constant config) ------------------------------- */
static const ComplexRect g_sample_box = {
    .re_min = -2.50f, .re_max =  1.00f,
    .im_min = -1.25f, .im_max =  1.25f,
};

/* ---- presets (constant config) ---------------------------------- *
 * Only the anti variant ships.  The buddha-mode code is still here and
 * working; just add a MODE_BUDDHA entry to turn it back on. */
#define N_PRESETS 1
static const MandelbrotPreset g_presets[N_PRESETS] = {
    { "anti     1k", 1000, MODE_ANTI },
};

/* ---- palettes (constant config) --------------------------------- *
 * Each theme runs through several bright hues; the faintest level is a
 * darker tone so the outer haze stays subtle.  Only that lowest level
 * dips into the dim color range; the rest stay bright enough to read. */
#define N_PALETTES 8
static const Palette g_palettes[N_PALETTES] = {
/*                       C1   C2   C3   C4   C5                     C1   C2   C3   C4   C5                                                  walk */
    { "Aurora",     {    53,  51,  46, 226, 231 },   { COLOR_BLUE, COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW, COLOR_WHITE } },  /* dark plum → cyan → lime → yellow → white */
    { "Galaxy",     {    54, 165,  51, 213, 231 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN, COLOR_MAGENTA, COLOR_WHITE } },  /* dim indigo → magenta → cyan → pink → white */
    { "Sunset",     {    24, 201, 196, 214, 226 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_RED, COLOR_RED, COLOR_YELLOW } },  /* deep blue → magenta → red → orange → yellow */
    { "Spectrum",   {    25,  51,  46, 226, 196 },   { COLOR_BLUE, COLOR_CYAN, COLOR_GREEN, COLOR_YELLOW, COLOR_RED } },  /* dim blue → cyan → green → yellow → red */
    { "Vapor",      {    53, 213,  51, 159, 231 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE } },  /* dark plum → pink → cyan → lt cyan → white */
    { "Tropical",   {    28,  46, 226, 208, 196 },   { COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED } },  /* deep green → lime → yellow → orange → red */
    { "Phoenix",    {    53, 201, 196, 214, 226 },   { COLOR_BLUE, COLOR_MAGENTA, COLOR_RED, COLOR_RED, COLOR_YELLOW } },  /* dark violet → magenta → red → orange → yellow */
    { "Neon",       {    89, 201,  39,  51, 231 },   { COLOR_MAGENTA, COLOR_MAGENTA, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE } },  /* dim magenta → magenta → blue → cyan → white */
};

/* ===================================================================== */
/* §2  random — fast LCG for sample selection                             */
/* ===================================================================== */

/* Kick off the generator from the current time, so each run differs.
 * Force it away from zero, which would get stuck. */
static void lcg_seed_from_clock(RandLCG *r)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    r->state = (uint32_t)(ts.tv_nsec ^ (ts.tv_sec * LCG_SEED_TIME_MIX));
    if (r->state == 0) r->state = 1u;
}

/* Advance to the next pseudo-random number. */
static inline uint32_t lcg_next_u32(RandLCG *r)
{
    r->state = r->state * LCG_MULTIPLIER + LCG_INCREMENT;
    return r->state;
}

/* A random number from 0 up to (but not including) 1.  Built from the
 * top bits only, since this generator's bottom bits are low quality. */
static inline float lcg_unit_float(RandLCG *r)
{
    uint32_t high_bits = lcg_next_u32(r) >> LCG_HIGH_BITS_SHIFT;
    return (float)high_bits / (float)LCG_UNIT_DENOM;
}

/* A random number somewhere between lo and hi. */
static inline float lcg_in_range(RandLCG *r, float lo, float hi)
{
    return lo + lcg_unit_float(r) * (hi - lo);
}

/* ===================================================================== */
/* §3  time — monotonic clock + rolling FPS                               */
/* ===================================================================== */

static long long clock_ns_now(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long long)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* Tally this frame; once about half a second has built up, work out
 * the average frame rate and start a fresh window. */
static void fps_tick(FpsCounter *fps, long long frame_ns)
{
    fps->accum_ns += frame_ns;
    fps->frames   += 1;
    if (fps->accum_ns >= FPS_WINDOW_MS * NS_PER_MS) {
        fps->display = (double)fps->frames
                     / ((double)fps->accum_ns / (double)NS_PER_SEC);
        fps->accum_ns = 0;
        fps->frames   = 0;
    }
}

/* ===================================================================== */
/* §4  view & palette                                                     */
/* ===================================================================== */

/* ---- viewport ---------------------------------------------------- */

/* Rows left for the picture after the top and bottom HUD lines. */
static int viewport_play_rows(int rows)
{
    int n = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    return n < 1 ? 1 : n;
}

/* Pick the zoom that fits the figure both across and down without
 * distorting it: try each direction, keep the tighter (smaller) one. */
static float viewport_pick_scale(int cols, int play_rows,
                                 float re_range, float im_range)
{
    float horizontal = (float)cols      / im_range;
    float vertical   = (float)play_rows * ASPECT_CELL_HEIGHT / re_range;
    return horizontal < vertical ? horizontal : vertical;
}

/* Record the terminal size, capped at the grid we have room for. */
static void viewport_set_dimensions(Viewport *v, int rows, int cols)
{
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    v->rows      = rows;
    v->cols      = cols;
    v->play_rows = viewport_play_rows(rows);
}

/* Work out how much of the plane to show.  Height is fixed to cover
 * the whole figure; width is stretched to match the terminal's shape
 * so circles look round, not squashed. */
static void viewport_compute_view_rectangle(const Viewport *v,
                                            float *re_half_out,
                                            float *im_half_out)
{
    *re_half_out = VIEW_RE_HALF;
    *im_half_out = VIEW_RE_HALF * (float)v->cols
                                / ((float)v->play_rows * ASPECT_CELL_HEIGHT);
}

/* Line up the figure's center with the middle of the screen. */
static void viewport_anchor_center(Viewport *v)
{
    v->center_col  = v->cols / 2;
    v->center_row  = HUD_TOP_ROWS + v->play_rows / 2;
    v->view_mid_re = VIEW_CENTER_RE;
    v->view_mid_im = VIEW_CENTER_IM;
}

/* Redo all the placement numbers for the current terminal size. */
static void viewport_fit(Viewport *v, int rows, int cols)
{
    viewport_set_dimensions(v, rows, cols);

    float re_half, im_half;
    viewport_compute_view_rectangle(v, &re_half, &im_half);

    v->scale = viewport_pick_scale(v->cols, v->play_rows,
                                   2.0f * re_half, 2.0f * im_half);

    viewport_anchor_center(v);
}

/* Turn a point on the plane into a screen cell (column, row), turned
 * on its side so the figure stands up.  Returns false if it falls
 * outside the drawable area. */
static inline bool viewport_project(const Viewport *v, Complex z,
                                    int *col_out, int *row_out)
{
    int col = v->center_col + (int)((z.im - v->view_mid_im) * v->scale);
    int row = v->center_row + (int)((z.re - v->view_mid_re) * v->scale * ASPECT_INV);

    if (col < 0 || col >= v->cols)                              return false;
    if (row < HUD_TOP_ROWS || row >= v->rows - HUD_BOTTOM_ROWS) return false;

    *col_out = col;
    *row_out = row;
    return true;
}

/* ---- palette ---------------------------------------------------- */

/* Tier index → ncurses colour-pair slot. */
static const int k_tier_pair_slots[N_DENSITY_TIERS] = {
    COL_C1, COL_C2, COL_C3, COL_C4, COL_C5,
};

/* Install the five density-tier pairs from the active palette.
 * Called at startup and whenever the user cycles `t` / `T`. */
static void palette_apply(int idx)
{
    const Palette *p = &g_palettes[idx];
    if (COLORS >= 256) {
        for (int tier = 0; tier < N_DENSITY_TIERS; tier++)
            init_pair(k_tier_pair_slots[tier], p->fg256[tier], COLOR_BLACK);
    } else {
        for (int tier = 0; tier < N_DENSITY_TIERS; tier++)
            init_pair(k_tier_pair_slots[tier], p->fg8[tier], COLOR_BLACK);
    }
}

/* HUD/HINT pairs are theme-independent — bright yellow data line +
 * bright cyan hint line stay legible against any palette. */
static void palette_init_static(void)
{
    if (COLORS >= 256) {
        init_pair(COL_HUD,  226, COLOR_BLACK);   /* bright yellow */
        init_pair(COL_HINT,  51, COLOR_BLACK);   /* bright cyan   */
    } else {
        init_pair(COL_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(COL_HINT, COLOR_CYAN,   COLOR_BLACK);
    }
}

/* ===================================================================== */
/* §5  density grid                                                       */
/* ===================================================================== */

static void grid_clear(DensityGrid *g)
{
    memset(g->counts, 0, sizeof g->counts);
    g->max_count = 0;
}

/* Bump one cell and update the running maximum.  Caller is
 * responsible for bounds-checking col/row (the viewport projector
 * does that). */
static inline void grid_hit(DensityGrid *g, int row, int col)
{
    uint32_t v = ++g->counts[row][col];
    if (v > g->max_count) g->max_count = v;
}

/* Tier-boundary tables, indexed [0..3] for the four breaks between
 * L1/L2, L2/L3, L3/L4, L4/L5.  t < entry[0] → L1, ..., t ≥ entry[3] → L5.
 *
 * Anti uses a wider spread (0.45 .. 0.90) because its dynamic range
 * is bunched at the high end (only attractor cells approach max).
 * Buddha uses a tighter spread so MORE of the figure registers in
 * mid-tones (without this the figure collapses into a flat blob;
 * with it, the gradient rim → body → core reads cleanly). */
static const float k_anti_density_thresholds[4]   = { 0.45f, 0.62f, 0.78f, 0.90f };
static const float k_buddha_density_thresholds[4] = { 0.30f, 0.45f, 0.60f, 0.78f };

/* Map a normalised log-density value `t` ∈ [0, 1] to a tier index
 * 0..4 (L1..L5), or -1 to skip (below the mode-dependent floor). */
static int density_tier(float t, BuddhMode mode)
{
    float floor = (mode == MODE_ANTI) ? DENSITY_FLOOR_ANTI : DENSITY_FLOOR_BUDDHA;
    if (t < floor) return -1;

    const float *thresholds = (mode == MODE_ANTI)
                                ? k_anti_density_thresholds
                                : k_buddha_density_thresholds;

    for (int tier = 0; tier < N_DENSITY_TIERS - 1; tier++)
        if (t < thresholds[tier]) return tier;
    return N_DENSITY_TIERS - 1;
}

/* ===================================================================== */
/* §6  Mandelbrot iteration & chaos game                                  */
/* ===================================================================== */

/* ---- complex arithmetic ----------------------------------------- */

static inline Complex complex_make(float re, float im)
{
    return (Complex){ re, im };
}

/* One Mandelbrot step: z → z² + c.  The squared modulus |z|² is
 * also returned so the caller can do its escape test without a
 * second multiply. */
static inline Complex mandelbrot_step(Complex z, Complex c, float *abs_sq_out)
{
    float new_re = z.re * z.re - z.im * z.im + c.re;
    float new_im = 2.0f * z.re * z.im        + c.im;
    *abs_sq_out  = new_re * new_re + new_im * new_im;
    return complex_make(new_re, new_im);
}

/* ---- Mandelbrot fast-skip regions ------------------------------- *
 *
 * The interior of the Mandelbrot set has two large always-bounded
 * regions for which we can skip the full escape iteration in
 * BUDDHA mode (those c values never escape).  Standard formulas
 * from Mandelbrot-set rendering literature.
 *
 *   Main cardioid:  q(q + cre − 1/4) < 1/4 · cim²
 *                   where q = (cre − 1/4)² + cim²
 *   Period-2 bulb:  |c + 1|² < 1/16
 */
static inline bool in_main_cardioid(Complex c)
{
    float q = (c.re - CARDIOID_CUSP_X) * (c.re - CARDIOID_CUSP_X)
            + c.im * c.im;
    return q * (q + c.re - CARDIOID_CUSP_X) < 0.25f * c.im * c.im;
}

static inline bool in_period2_bulb(Complex c)
{
    float dx = c.re - PERIOD2_BULB_CX;
    return dx * dx + c.im * c.im < PERIOD2_BULB_R2;
}

/* True if c lies in either always-bounded region — fast skip for
 * BUDDHA mode (no escape to find). */
static inline bool in_known_interior(Complex c)
{
    return in_main_cardioid(c) || in_period2_bulb(c);
}

/* ---- escape test (Pass 1) --------------------------------------- *
 *
 * Iterate z from 0 under z → z² + c.  Return true if |z|² exceeds
 * ESCAPE_RADIUS_SQ within max_iter steps; false otherwise. */
static bool mandelbrot_orbit_escapes(Complex c, int max_iter)
{
    Complex z      = complex_make(MANDELBROT_Z0_RE, MANDELBROT_Z0_IM);
    float   abs_sq = 0.0f;
    for (int i = 0; i < max_iter; i++) {
        z = mandelbrot_step(z, c, &abs_sq);
        if (abs_sq > ESCAPE_RADIUS_SQ) return true;
    }
    return false;
}

/* ---- orbit trace + accumulate (Pass 2) -------------------------- */

/* Project one orbit point onto the density grid; if it lands inside
 * the play area, bump its cell's hit counter. */
static inline void plot_orbit_point(DensityGrid *grid, const Viewport *view, Complex z)
{
    int col, row;
    if (viewport_project(view, z, &col, &row))
        grid_hit(grid, row, col);
}

/* Re-iterate z from 0, plotting each orbit point onto the grid.
 *
 *     for each iteration:
 *         plot the current orbit point to the grid
 *         z ← mandelbrot_step(z, c)
 *         (BUDDHA only) stop if the orbit has escaped
 *
 * In BUDDHA mode we terminate early once the orbit escapes —
 * further points fly off to infinity and don't contribute useful
 * structure.  In ANTI mode the orbit is bounded by construction
 * so we always iterate to max_iter. */
static void mandelbrot_trace_orbit(Complex c, int max_iter, bool anti_mode,
                                   DensityGrid *grid, const Viewport *view)
{
    Complex z      = complex_make(MANDELBROT_Z0_RE, MANDELBROT_Z0_IM);
    float   abs_sq = 0.0f;

    for (int i = 0; i < max_iter; i++) {
        plot_orbit_point(grid, view, z);
        z = mandelbrot_step(z, c, &abs_sq);
        if (!anti_mode && abs_sq > ESCAPE_RADIUS_SQ) break;
    }
}

/* ---- sampler ---------------------------------------------------- */

/* Pick one uniformly-random c value from the sample box. */
static inline Complex sampler_pick_c(RandLCG *rng, const ComplexRect *box)
{
    return complex_make(lcg_in_range(rng, box->re_min, box->re_max),
                        lcg_in_range(rng, box->im_min, box->im_max));
}

/* ---- the per-sample driver -------------------------------------- *
 *
 * Reads as the algorithm itself:
 *
 *     1. pick a random c
 *     2. fast-skip if c is in a known-interior region (buddha only)
 *     3. classify c's orbit (Pass 1: does it escape?)
 *     4. accept c if its escape status matches the mode's filter
 *     5. trace the orbit and accumulate density (Pass 2)
 */
static void buddhabrot_run_one_sample(Scene *s)
{
    const MandelbrotPreset *preset = &g_presets[s->preset];
    bool anti_mode = (preset->mode == MODE_ANTI);

    Complex c = sampler_pick_c(&s->rng, &g_sample_box);

    if (!anti_mode && in_known_interior(c)) return;

    bool escaped = mandelbrot_orbit_escapes(c, preset->max_iter);

    /* anti_mode XOR escaped: skip when escape status fails the filter */
    if (anti_mode == escaped) return;

    mandelbrot_trace_orbit(c, preset->max_iter, anti_mode,
                           &s->grid, &s->view);
}

/* Run SAMPLES_PER_TICK samples this tick.  Stops short if the
 * preset's accumulation target has been reached. */
static void buddhabrot_iterate(Scene *s)
{
    for (int i = 0; i < SAMPLES_PER_TICK; i++) {
        if (s->samples_done >= TOTAL_SAMPLES) return;
        buddhabrot_run_one_sample(s);
        s->samples_done++;
    }
}

/* ===================================================================== */
/* §7  render & HUD                                                       */
/* ===================================================================== */

/* Density tier → glyph lookup.  (Tier → colour-pair slot lives in §4
 * with palette_apply as k_tier_pair_slots[].) */
static const chtype k_tier_glyphs[N_DENSITY_TIERS] = {
    GLYPH_L1, GLYPH_L2, GLYPH_L3, GLYPH_L4, GLYPH_L5,
};

/* Plot one density tier into a screen cell.  Peak tier (L5) gets
 * A_BOLD so the hot-spots really pop. */
static void plot_density_cell(int row, int col, int tier)
{
    chtype glyph = k_tier_glyphs[tier];
    int    pair  = k_tier_pair_slots[tier];
    attr_t bold  = (tier == N_DENSITY_TIERS - 1) ? A_BOLD : 0;

    attron(COLOR_PAIR(pair) | bold);
    mvaddch(row, col, glyph);
    attroff(COLOR_PAIR(pair) | bold);
}

/* Clamp log1p output so the log-tone denominator never goes ~0. */
static inline float log1p_safe(float x)
{
    float v = logf(1.0f + x);
    return v < EPS_LOG_MAX ? EPS_LOG_MAX : v;
}

/* Log-tone normalisation: hits → t ∈ [0, 1]. */
static inline float log_tone_map(uint32_t hits, float log_max)
{
    return logf(1.0f + (float)hits) / log_max;
}

/* Walk-bounds for the render pass.  Clipped to both the play area
 * (between the HUD rows) and the static grid limits. */
static void grid_render_walk_bounds(const Viewport *v,
                                    int *row_top, int *row_lim, int *col_lim)
{
    *row_top = HUD_TOP_ROWS;
    *row_lim = v->rows - HUD_BOTTOM_ROWS;
    *col_lim = v->cols;
    if (*row_lim > GRID_ROWS_MAX) *row_lim = GRID_ROWS_MAX;
    if (*col_lim > GRID_COLS_MAX) *col_lim = GRID_COLS_MAX;
}

/* Convert one cell's hit count to a tier and plot it.  Returns
 * silently if the cell is empty or below the mode-dependent floor. */
static void grid_render_one_cell(uint32_t hits, float log_max, BuddhMode mode,
                                 int row, int col)
{
    if (hits == 0) return;
    int tier = density_tier(log_tone_map(hits, log_max), mode);
    if (tier < 0) return;
    plot_density_cell(row, col, tier);
}

/* Render the density grid as glyph-tier ASCII.
 *
 *     1. Find the play-area bounds (clip to grid limits + HUD rows).
 *     2. Compute log of the running max for normalisation.
 *     3. Walk every cell; map hits → tier via log-tone, plot.
 */
static void grid_render(const DensityGrid *g, const Viewport *v, BuddhMode mode)
{
    int row_top, row_lim, col_lim;
    grid_render_walk_bounds(v, &row_top, &row_lim, &col_lim);

    float log_max = log1p_safe((float)g->max_count);

    for (int row = row_top; row < row_lim; row++)
        for (int col = 0; col < col_lim; col++)
            grid_render_one_cell(g->counts[row][col], log_max, mode, row, col);
}

/* ---- HUD --------------------------------------------------------- */

/* Percentage of the preset's sample target reached so far. */
static int hud_compute_pct_done(const Scene *s)
{
    if (TOTAL_SAMPLES <= 0) return HUD_PCT_FULL;
    int pct = s->samples_done * HUD_PCT_FULL / TOTAL_SAMPLES;
    return pct > HUD_PCT_FULL ? HUD_PCT_FULL : pct;
}

/* Format the top HUD data line into `buf`.  Returns string length. */
static int hud_format_data_line(char *buf, int bufsize, const Scene *s, int pct)
{
    return snprintf(buf, bufsize,
        " [%d/%d] %-11s  thm:%-6s  spd:%d  %3d%%  %5.1f fps  %s ",
        s->preset + 1, N_PRESETS,
        g_presets[s->preset].name,
        g_palettes[s->palette].name,
        s->sim_fps, pct, s->fps.display,
        s->paused ? "PAUSED" : "      ");
}

/* Draw a string right-aligned at row 0 with HUD attrs, clipped to
 * the terminal width so we never write past the right edge. */
static void hud_draw_top_right_aligned(const char *str, int len, int cols)
{
    if (len > cols) len = cols;
    int x = cols - len;
    if (x < 0) x = 0;
    attron(COLOR_PAIR(COL_HUD) | A_BOLD);
    mvprintw(0, x, "%.*s", len, str);
    attroff(COLOR_PAIR(COL_HUD) | A_BOLD);
}

/* Top HUD row: compute progress, format line, draw right-aligned. */
static void hud_draw_data(const Scene *s)
{
    int pct = hud_compute_pct_done(s);

    char buf[HUD_BUFFER_BYTES];
    int  len = hud_format_data_line(buf, sizeof buf, s, pct);

    hud_draw_top_right_aligned(buf, len, s->view.cols);
}

static void hud_draw_hint(const Scene *s)
{
    attron(COLOR_PAIR(COL_HINT) | A_BOLD);
    mvprintw(s->view.rows - 1, 0,
        " q:quit  p:pause  r:reset  [/]:speed  t/T:theme ");
    attroff(COLOR_PAIR(COL_HINT) | A_BOLD);
}

static void hud_draw(const Scene *s)
{
    hud_draw_data(s);
    hud_draw_hint(s);
}

/* ===================================================================== */
/* §8  scene & main                                                       */
/* ===================================================================== */

static volatile sig_atomic_t g_quit   = 0;
static volatile sig_atomic_t g_resize = 0;

static void on_signal(int s)
{
    if (s == SIGINT || s == SIGTERM) g_quit   = 1;
    if (s == SIGWINCH)               g_resize = 1;
}

static void on_exit_cleanup(void) { endwin(); }

/* ---- scene action helpers (named for what the user sees) -------- */

/* Clear the grid and reset the sample counter — start the chaos
 * game fresh.  Doesn't change preset, palette, or RNG state. */
static void scene_rebuild(Scene *s)
{
    grid_clear(&s->grid);
    s->samples_done = 0;
}

static void scene_cycle_preset(Scene *s, int dir)
{
    s->preset = (s->preset + dir + N_PRESETS) % N_PRESETS;
    scene_rebuild(s);
}

static void scene_cycle_palette(Scene *s, int dir)
{
    s->palette = (s->palette + dir + N_PALETTES) % N_PALETTES;
    palette_apply(s->palette);
}

static void scene_change_speed(Scene *s, int delta)
{
    int n = s->sim_fps + delta;
    if (n < SIM_FPS_MIN) n = SIM_FPS_MIN;
    if (n > SIM_FPS_MAX) n = SIM_FPS_MAX;
    s->sim_fps = n;
}

/* ---- scene lifecycle -------------------------------------------- */

static void scene_read_term_size(int *rows_out, int *cols_out)
{
    int rows, cols;
    getmaxyx(stdscr, rows, cols);
    if (rows > GRID_ROWS_MAX) rows = GRID_ROWS_MAX;
    if (cols > GRID_COLS_MAX) cols = GRID_COLS_MAX;
    *rows_out = rows;
    *cols_out = cols;
}

static void scene_init_options(Scene *s)
{
    s->preset       = DEFAULT_PRESET_IDX;
    s->palette      = DEFAULT_PALETTE_IDX;
    s->paused       = false;
    s->sim_fps      = SIM_FPS_DEFAULT;
    s->samples_done = 0;
    s->fps          = (FpsCounter){ .accum_ns = 0, .frames = 0, .display = 0.0 };
}

static void scene_init_rng(Scene *s)
{
    lcg_seed_from_clock(&s->rng);
}

static void scene_init_view_and_grid(Scene *s)
{
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols);
    grid_clear(&s->grid);
}

/* Set the scene to its starting state.
 *
 *     1. options  — preset, palette, pause, FPS, sample counter
 *     2. RNG      — seed from clock
 *     3. view/grid — viewport from terminal; clear grid */
static void scene_init(Scene *s)
{
    scene_init_options(s);
    scene_init_rng(s);
    scene_init_view_and_grid(s);
}

static void scene_resize(Scene *s)
{
    endwin();
    refresh();
    int rows, cols;
    scene_read_term_size(&rows, &cols);
    viewport_fit(&s->view, rows, cols);
    scene_rebuild(s);
}

/* Handle one keystroke.  Each case maps to a named scene operation. */
static void scene_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESCAPE:  g_quit = 1;                          break;
    case 'p': case 'P': case ' ':         s->paused = !s->paused;              break;
    case 'r': case 'R': case 'n': case 'N': scene_cycle_preset(s, +1);         break;
    case ']':                             scene_change_speed(s, +SIM_FPS_STEP); break;
    case '[':                             scene_change_speed(s, -SIM_FPS_STEP); break;
    case 't':                             scene_cycle_palette(s, +1);          break;
    case 'T':                             scene_cycle_palette(s, -1);          break;
    default: break;
    }
}

/* Drain all pending keystrokes (nodelay() returns ERR when none). */
static void scene_process_input(Scene *s)
{
    int ch;
    while ((ch = getch()) != ERR)
        scene_handle_key(s, ch);
}

/* Advance the chaos game by one tick's worth of samples.  Skips the
 * work entirely while paused or once the accumulation target is hit. */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    if (s->samples_done >= TOTAL_SAMPLES) return;
    buddhabrot_iterate(s);
}

/* ---- frame ------------------------------------------------------- */

static void frame_render(const Scene *s)
{
    erase();
    grid_render(&s->grid, &s->view, g_presets[s->preset].mode);
    hud_draw(s);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ---- ncurses + signal setup ------------------------------------ */

static void register_signal_handlers(void)
{
    atexit(on_exit_cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
}

static void ncurses_setup(void)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
}

static void colors_setup(void)
{
    start_color();
    palette_init_static();
}

static void app_init(void)
{
    register_signal_handlers();
    ncurses_setup();
    colors_setup();
}

/* Cap dt so a long stall (process suspended/resumed, kernel hiccup)
 * doesn't trigger a spiral of catch-up ticks. */
static long long clamp_dt(long long dt)
{
    return dt > MAX_DT_NS ? MAX_DT_NS : dt;
}

/* Fixed-step simulation accumulator.  Adds dt to the accumulator
 * and ticks the scene every tick_ns interval, leaving the
 * remainder for the next frame.  Decouples the chaos game's
 * sim_fps Hz from the render rate. */
static void run_simulation_ticks(Scene *s, long long *sim_accum, long long dt)
{
    long long tick_ns = NS_PER_SEC / s->sim_fps;
    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(s);
        *sim_accum -= tick_ns;
    }
}

/* Sleep what's left of the frame budget so the render loop holds
 * its target rate. */
static void frame_pace_to_target(long long frame_start)
{
    long long elapsed = clock_ns_now() - frame_start;
    clock_sleep_ns(FRAME_NS_60FPS - elapsed);
}

/* ---- main loop --------------------------------------------------- *
 *
 * Each iteration reads as the algorithm itself: process input, run
 * one or more simulation ticks, render the frame, update FPS, sleep
 * to target.  The simulation is decoupled from rendering via the
 * fixed-step accumulator inside `run_simulation_ticks`, so the
 * chaos game ticks at sim_fps Hz regardless of render rate. */
int main(void)
{
    app_init();

    static Scene scene;
    scene_init(&scene);
    palette_apply(scene.palette);

    long long t_prev    = clock_ns_now();
    long long sim_accum = 0;

    while (!g_quit) {
        if (g_resize) { g_resize = 0; scene_resize(&scene); }

        long long t_now = clock_ns_now();
        long long dt    = clamp_dt(t_now - t_prev);
        t_prev = t_now;

        scene_process_input(&scene);
        run_simulation_ticks(&scene, &sim_accum, dt);
        frame_render(&scene);
        fps_tick(&scene.fps, dt);

        frame_pace_to_target(t_now);
    }
    return 0;
}
