/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_squares_isocontours_showcase.c
 *
 * Draws contour lines through a field of numbers, the way a map draws
 * the coastline at a chosen sea level. Pick a level T; the "marching
 * squares" trick looks at the four corners of each little square of the
 * grid, notes which corners are above T and which are below, and from
 * that picks the right line glyph to draw. 30 different ways to colour
 * and combine those lines, cycled with n/p.
 *
 * The field of numbers here is some gently drifting noise, but marching
 * squares doesn't care where the numbers come from — a heightmap or a
 * distance field would work just as well.
 *
 * Sister files:
 *   ./perin_noise_flow_showcase.c        — the noise field we contour.
 *   ./signed_distance_field_jfa_showcase.c — its OUTLINE does a soft
 *       version of the same idea without marching squares.
 *
 * Marching squares / cubes references:
 *   Lorensen & Cline (1987), "Marching Cubes", SIGGRAPH'87 — the
 *     original; marching squares is its 2-D simplification.
 *   Nielson & Hamann (1991), "The Asymptotic Decider", IEEE Vis'91 —
 *     the reference on the ambiguous saddle cases (5 and 10).
 *   Bourke, P. — "Polygonising a scalar field":
 *     http://paulbourke.net/geometry/polygonise/  (all the cases drawn).
 *   Wikipedia — "Marching squares".
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
};

/* The HUD lives on three rows: two info rows up top, one key-hint row
 * at the bottom. The rest of the screen is the drawing area. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Column widths for the fields on the second HUD row. Each must match
 * the width its hud_field_* helper actually prints, or neighbouring
 * fields will overlap or leave gaps. */
#define HUD_PATTERN_FIELD_W     21
#define HUD_TIER_FIELD_W        15
#define HUD_THEME_FIELD_W       17
#define HUD_PALETTE_LABEL_W      9
#define HUD_N_PALETTE_BANDS      4

/* Smallest grid we'll draw — anything tinier is unreadable. */
#define MAP_W_MIN               16
#define MAP_H_MIN                8

/* How fast we aim to redraw the screen, and the matching time budget
 * per frame that the throttle sleeps down to. */
#define RENDER_FPS_TARGET       60
#define RENDER_FRAME_BUDGET_NS  (NS_PER_SEC / RENDER_FPS_TARGET)

/* If one frame takes much longer than usual (slow terminal, the window
 * was dragged), don't try to "catch up" with a flood of sim steps.
 * Cap how much time one frame is allowed to count. See Fiedler, "Fix
 * Your Timestep!" (cited on App, §10). */
#define SIM_MAX_FRAME_DT_MS    100

/* How zoomed-in we sample the noise: smaller means bigger, smoother
 * blobs and coarser contours. */
#define FIELD_SCALE         0.08f

/* How fast the field drifts over time — kept slow so the eye can
 * follow how the contour shapes change. */
#define FIELD_DRIFT         0.20f

/* The noise is three layers of detail added together: each layer is
 * twice as fine as the last (that doubling is the "lacunarity") and
 * adds a little less. The three amounts sum to 1.0 so the result
 * naturally lands in [0, 1]. */
#define FBM_LACUNARITY      2.0f
#define FBM_OCT0_FREQ       1.0f
#define FBM_OCT1_FREQ       (FBM_LACUNARITY)
#define FBM_OCT2_FREQ       (FBM_LACUNARITY * FBM_LACUNARITY)
#define FBM_OCT0_AMP        0.5f
#define FBM_OCT1_AMP        0.3f
#define FBM_OCT2_AMP        0.2f

/* Speed-up factor for the drift, doubled/halved by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      16

/* The sea level the user picks with </>: starting value, step size,
 * and the range it's allowed to roam in. */
#define CONTOUR_T_DEFAULT   0.50f
#define CONTOUR_T_STEP      0.05f
#define CONTOUR_T_MIN       0.05f
#define CONTOUR_T_MAX       0.95f

/* THICK pattern: how far above and below the level still counts as
 * "on the line", to make it look fat. */
#define THICK_BAND_HALF     0.04f

/* How many evenly-spaced contour levels each multi-level pattern draws. */
#define TOPO_N_LEVELS       8
#define DENSE_N_LEVELS     16
#define SPARSE_N_LEVELS     4
#define STEPS_N_LEVELS      6
#define RAINBOW_N_LEVELS   12

/* Fixed sea levels for the patterns that draw two or three set lines. */
#define DUAL_T_LOW          0.35f
#define DUAL_T_HIGH         0.65f
#define SHELL_T_LOW         0.46f
#define SHELL_T_HIGH        0.54f
#define INNER_T             0.70f
#define OUTER_T             0.30f
#define SANDWICH_T_LOW      0.30f
#define SANDWICH_T_MID     0.50f
#define SANDWICH_T_HIGH     0.70f

/* Region-fill patterns: stripe thickness and how many zebra bands. */
#define STRIPE_HALFWIDTH    0.07f
#define ZEBRA_N_BANDS       8

/* Timing and sizes for the patterns whose sea level moves on its own. */
#define SWEEP_T_PERIOD_SEC  6.0f
#define PULSE_PERIOD_SEC    3.0f
#define RIPPLE_AMP          0.10f
#define RIPPLE_FREQ_CELLS  10.0f
#define RIPPLE_SPEED        2.0f
#define STAIRS_N_LEVELS     5
#define STAIRS_PERIOD_SEC   8.0f
#define CHAOS_AMP           0.10f

/*
 * Pattern — the 30 ways to draw the contours, cycled with n/p.
 * Grouped into six tiers from simplest to fanciest. This list and the
 * noise_patterns[] table in §7 must stay in the same order; the
 * fixed-size [N_PATTERNS] table makes the compiler catch any mismatch.
 *
 *   Tier 1 SINGLE   : one line  (CONTOUR, THICK, INVERT, DOTS, UNIFORM)
 *   Tier 2 MULTI    : evenly-spaced lines, like a topo map
 *                     (TOPO, DENSE, SPARSE, STEPS, RAINBOW)
 *   Tier 3 PAIRS    : a few set lines (DUAL, SHELL, INNER, OUTER, SANDWICH)
 *   Tier 4 TOPOLOGY : show only certain kinds of cell
 *                     (SADDLE, ORTHO, DIAGONALS, JUNCTIONS, ROUGHNESS)
 *   Tier 5 REGIONS  : fill areas, not just lines
 *                     (ABOVE, BELOW, STRIPE, ZEBRA, PERIMETER)
 *   Tier 6 ANIMATED : the sea level moves on its own
 *                     (SWEEP, PULSE, RIPPLE, STAIRS, CHAOS)
 */
typedef enum {
    PATTERN_CONTOUR = 0,
    PATTERN_THICK,
    PATTERN_INVERT,
    PATTERN_DOTS,
    PATTERN_UNIFORM,
    PATTERN_TOPO,
    PATTERN_DENSE,
    PATTERN_SPARSE,
    PATTERN_STEPS,
    PATTERN_RAINBOW,
    PATTERN_DUAL,
    PATTERN_SHELL,
    PATTERN_INNER,
    PATTERN_OUTER,
    PATTERN_SANDWICH,
    PATTERN_SADDLE,
    PATTERN_ORTHO,
    PATTERN_DIAGONALS,
    PATTERN_JUNCTIONS,
    PATTERN_ROUGHNESS,
    PATTERN_ABOVE,
    PATTERN_BELOW,
    PATTERN_STRIPE,
    PATTERN_ZEBRA,
    PATTERN_PERIMETER,
    PATTERN_SWEEP,
    PATTERN_PULSE,
    PATTERN_RIPPLE,
    PATTERN_STAIRS,
    PATTERN_CHAOS,
    N_PATTERNS,
} Pattern;

/* Defined down in §7 next to the pattern table; named here so the HUD
 * code in §9 can ask for the current pattern's label. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

/*
 * Picks the line character for each of the 16 corner-patterns.
 *
 * Look at one square's four corners, mark each one above the sea level
 * (1) or below (0), and read them off as a 4-bit number 0..15. That
 * number is the index into this table, and the character is the line
 * that best matches how the contour cuts through the square: a slash
 * for one raised corner, a dash for a flat split, a bar for a vertical
 * split, and 'X' for the two awkward "saddle" squares where the contour
 * could go two ways. All-below (0) and all-above (15) draw nothing.
 */
static const char ms_case_glyph[16] = {
    /* 0  */ ' ',
    /* 1  */ '/',
    /* 2  */ '\\',
    /* 3  */ '-',
    /* 4  */ '\\',
    /* 5  */ 'X',  /* saddle */
    /* 6  */ '|',
    /* 7  */ '/',
    /* 8  */ '/',
    /* 9  */ '|',
    /* 10 */ 'X',  /* saddle */
    /* 11 */ '\\',
    /* 12 */ '-',
    /* 13 */ '\\',
    /* 14 */ '/',
    /* 15 */ ' ',
};

/* The two awkward squares where the contour could be drawn two ways. */
static inline bool ms_case_is_saddle(int c) { return c == 5 || c == 10; }

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — one named colour scheme: a name plus four colours going from
 * dim to bright.
 *
 * This keeps "which colours" separate from "how they're used". A
 * pattern only ever says "draw this cell in colour slot 0..3"; the
 * theme decides what those four slots actually look like. Pressing t/T
 * swaps in a different theme's four colours, and no drawing code has to
 * change.
 *
 * The four slots go from dimmest to brightest, used roughly as:
 *   band[0] dim    — faint background strokes
 *   band[1] low    — outer / lower lines
 *   band[2] mid    — the main line for most patterns
 *   band[3] bright — inner lines and highlights
 *
 * name    : short all-caps label shown in the HUD; just a pointer to a
 *           string literal, not copied.
 * band[4] : the four colours, as xterm-256 colour numbers. Every entry
 *           must be at least 24 so even the dimmest stays readable on a
 *           black background (see CLAUDE.md, Theme Palette Brightness).
 *           What gives a theme its feel is the jump from dim to bright,
 *           not the exact hue.
 */
typedef struct {
    const char *name;
    short       band[4];
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    { "DEFAULT", {  17,   33,  220,  231 } },
    { "MATRIX",  {  22,   34,   46,  118 } },
    { "NOVA",    {  53,  129,  201,  219 } },
    { "MONO",    { 234,  244,  250,  254 } },
    { "OCEAN",   {  17,   33,   39,   51 } },
    { "FIRE",    {  52,  124,  208,  226 } },
    { "EARTH",   {  58,  100,  173,  230 } },
    { "FOREST",  {  22,   28,   64,  144 } },
    { "DESERT",  {  94,  130,  173,  222 } },
    { "ARCTIC",  {  18,   39,  159,  231 } },
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

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  ScalarField — the scalar source we contour over                    */
/* ===================================================================== */

/*
 * ScalarField — the grid of numbers we draw contours through.
 *
 * Think of it as a heightmap: one float per cell saying how "high" the
 * field is there. Marching squares doesn't care where these numbers
 * come from — here they're gently drifting noise, but a real heightmap
 * or a distance field would slot in with no other changes.
 *
 * One of these lives on Scene (§8). It's recomputed every sim tick
 * (the time value shifts so the field appears to drift), then read by
 * the marching-squares classifier and the patterns. Patterns get it as
 * const, so they can only read it.
 *
 * w, h    : grid size in cells, set at reset, kept within the screen
 *           limits.
 * count   : w * h, stored so loops don't keep recomputing it.
 * seed    : a fresh random number each reset (r), so every run gets a
 *           different-looking field. We mix it into the noise instead
 *           of touching rand()'s global state.
 * samples : the actual numbers, one per cell, each in [0, 1]. Laid out
 *           row by row, so cell (x, y) is at samples[y * w + x].
 *
 * The noise is "value noise" stacked into "fBm" (a few layers of
 * detail at finer and finer scales). References:
 *   Perlin, K. (1985), "An Image Synthesizer", SIGGRAPH'85.
 *   Quilez, I. — "More noise": https://iquilezles.org/articles/morenoise/
 */
typedef struct {
    int      w, h;
    int      count;
    uint32_t seed;
    float    samples[CELLS_MAX];
} ScalarField;

/* Read one cell's value, but snap any out-of-range coordinate back to
 * the nearest edge cell. Lets patterns near the border ask about cells
 * that don't exist without special-casing the edges. */
static inline float scalar_field_at(const ScalarField *src, int x, int y)
{
    if (x < 0)             x = 0;
    if (x >= src->w)       x = src->w - 1;
    if (y < 0)             y = 0;
    if (y >= src->h)       y = src->h - 1;
    return src->samples[y * src->w + x];
}

/* ---------- value-noise primitives ---------------------------------- */

static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

/* A repeatable "random" value in [0, 1] for a whole-number grid corner.
 * Same corner and seed always give the same number — that repeatability
 * is what makes noise look like terrain instead of static. */
static inline float lattice_scalar(const ScalarField *src, int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + src->seed;
    h = hash32(h);
    return (float)(h >> 8) * (1.0f / 16777215.0f);
}

static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }
static inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }
static inline float clamp01(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

/* One layer of smooth noise: take the four whole-number corners around
 * (x, y) and blend between them, easing the blend near the corners so
 * there are no hard creases. */
static float value_noise(const ScalarField *src, float x, float y)
{
    int   xi = (int)floorf(x);
    int   yi = (int)floorf(y);
    float ux = smoothstep01(x - (float)xi);
    float uy = smoothstep01(y - (float)yi);
    float v00 = lattice_scalar(src, xi,     yi);
    float v10 = lattice_scalar(src, xi + 1, yi);
    float v01 = lattice_scalar(src, xi,     yi + 1);
    float v11 = lattice_scalar(src, xi + 1, yi + 1);
    float top = lerpf(v00, v10, ux);
    float bot = lerpf(v01, v11, ux);
    return lerpf(top, bot, uy);
}

/* The field we actually draw: three layers of noise added together,
 * each finer and fainter than the last. Stacking them this way gives
 * the contours their interesting shapes — lakes, peninsulas, pinch
 * points — at whatever sea level you pick. */
static float fbm_3oct(const ScalarField *src, float x, float y)
{
    float oct0 = value_noise(src, x * FBM_OCT0_FREQ, y * FBM_OCT0_FREQ) * FBM_OCT0_AMP;
    float oct1 = value_noise(src, x * FBM_OCT1_FREQ, y * FBM_OCT1_FREQ) * FBM_OCT1_AMP;
    float oct2 = value_noise(src, x * FBM_OCT2_FREQ, y * FBM_OCT2_FREQ) * FBM_OCT2_AMP;
    return clamp01(oct0 + oct1 + oct2);
}

/* ---------- ScalarField lifecycle ---------------------------------- */

/* Set the grid size and pick a fresh seed. Doesn't fill in the values
 * — scalar_field_rebuild does that next. */
static void scalar_field_reset(ScalarField *src, int w, int h)
{
    src->w     = w;
    src->h     = h;
    src->count = w * h;
    src->seed  = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
}

/* Fill in every cell's value for the current moment in time. Time is
 * folded into the vertical coordinate, so as time advances the whole
 * field looks like it's scrolling downward. */
static void scalar_field_rebuild(ScalarField *src, float t)
{
    for (int y = 0; y < src->h; y++) {
        for (int x = 0; x < src->w; x++) {
            float fx = (float)x * FIELD_SCALE;
            float fy = (float)y * FIELD_SCALE + t;
            src->samples[y * src->w + x] = fbm_3oct(src, fx, fy);
        }
    }
}

/* ===================================================================== */
/* §6  ms — marching-squares classifier                                   */
/* ===================================================================== */

/*
 * The heart of marching squares. For the little square whose top-left
 * is cell (x, y), check each of its four corners against the sea level:
 * a corner above the level sets its bit, below leaves it clear. Pack
 * the four answers into one number 0..15 — that number tells you the
 * shape of the contour through this square (see ms_case_glyph above).
 * Same field and same level always give the same answer. Some patterns
 * call this several times per square with different levels.
 */
static inline int ms_classify(const ScalarField *src,
                              int x, int y, float threshold)
{
    int c = 0;
    if (scalar_field_at(src, x,     y + 1) >= threshold) c |= 1;
    if (scalar_field_at(src, x + 1, y + 1) >= threshold) c |= 2;
    if (scalar_field_at(src, x + 1, y)     >= threshold) c |= 4;
    if (scalar_field_at(src, x,     y)     >= threshold) c |= 8;
    return c;
}

/* ===================================================================== */
/* §7  patterns — 30 contour visualisations + dispatch                    */
/* ===================================================================== */

/*
 * How a pattern works. Every pattern is a function called once for each
 * grid square. It's told where the square is (x, y) and the current
 * controls: user_t (the sea level the user set), field_time (how far
 * the field has drifted, in seconds), and sweep_phase (a 0..1 counter
 * for the patterns that move the level on their own).
 *
 * In return it fills in three answers for that square:
 *   out_glow  — 1.0 to draw it, 0.0 to leave it blank
 *   out_band  — which of the four theme colours to use (0..3)
 *   out_glyph — which character to draw
 *
 * A pattern that wants the "true" contour line asks ms_classify for the
 * corner-pattern and looks up ms_case_glyph; others just pick their own
 * character. The renderer in §9 only reads these three answers and
 * paints — it knows nothing about marching squares.
 */

/* Is this corner-pattern one of the four flat splits — a plain
 * horizontal or vertical cut, no single corner sticking out? */
static inline bool ms_case_is_ortho(int c)
{
    return c == 3 || c == 6 || c == 9 || c == 12;
}

/* Is this one of the squares where the contour slices off a single
 * corner (a diagonal cut)? Covers both one-corner-up and one-corner-
 * down, which are just mirror images of each other. */
static inline bool ms_case_is_diagonal(int c)
{
    return c == 1 || c == 2 || c == 4  || c == 7
        || c == 8 || c == 11|| c == 13 || c == 14;
}

/* Two little helpers so each pattern can say "skip this square" or
 * "draw this character in this colour" in one line instead of three
 * scattered assignments. */
static inline void cell_skip(float *gl, uint8_t *bn, char *gy)
{
    *gl = 0.0f; *bn = 0; *gy = 0;
}
static inline void cell_emit(float *gl, uint8_t *bn, char *gy,
                             int band, char glyph)
{
    *gl = 1.0f; *bn = (uint8_t)(band & 3); *gy = glyph;
}

/* Draw the proper contour line for one square at the given sea level,
 * or skip it if the contour doesn't pass through. Used by every pattern
 * that wants the real marching-squares line. */
static inline void cell_emit_case_glyph(const ScalarField *src,
                                        int x, int y, float t, int band,
                                        float *gl, uint8_t *bn, char *gy)
{
    int c = ms_classify(src, x, y, t);
    char glyph = ms_case_glyph[c];
    if (glyph == ' ') { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, band, glyph);
}

/* ---------- Tier 1 — SINGLE: one line ------------------------------- */

/* CONTOUR — the plain single line at the level the user picked. The
 * basic marching-squares look. */
static void pattern_contour(const ScalarField *src, int x, int y,
                            float user_t, float field_time, float sweep_phase,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    cell_emit_case_glyph(src, x, y, user_t, 2, gl, bn, gy);
}

/* THICK — the same line, drawn fat. A square lights up if the contour
 * passes through at the level or just above or below it; squares right
 * on the line keep their line character, the rest fill with '#'. */
static void pattern_thick(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c_lo = ms_classify(src, x, y, user_t - THICK_BAND_HALF);
    int c    = ms_classify(src, x, y, user_t);
    int c_hi = ms_classify(src, x, y, user_t + THICK_BAND_HALF);

    bool fires = (c_lo > 0 && c_lo < 15)
              || (c    > 0 && c    < 15)
              || (c_hi > 0 && c_hi < 15);
    if (!fires) { cell_skip(gl, bn, gy); return; }

    char glyph = (c > 0 && c < 15) ? ms_case_glyph[c] : '#';
    cell_emit(gl, bn, gy, 2, glyph);
}

/* INVERT — the same line, but coloured by which corner-pattern each
 * square has, so squares with different line shapes show in different
 * tints. */
static void pattern_invert(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int  c     = ms_classify(src, x, y, user_t);
    char glyph = ms_case_glyph[c];
    if (glyph == ' ') { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, c & 3, glyph);
}

/* DOTS — the same line, drawn as a faint trail of dots. */
static void pattern_dots(const ScalarField *src, int x, int y,
                         float user_t, float field_time, float sweep_phase,
                         float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 1, '.');
}

/* UNIFORM — the same line, but every square drawn as a plain '+'. Shows
 * where the contour is without the slashes and bars cluttering it. */
static void pattern_uniform(const ScalarField *src, int x, int y,
                            float user_t, float field_time, float sweep_phase,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, '+');
}

/* ---------- Tier 2 — MULTI: evenly-spaced lines --------------------- */

/* Shared by the topo-map patterns. Tries several evenly-spaced sea
 * levels; if the contour at any of them runs through this square, draw
 * a '+'. The highest such level decides the colour, so the rings end up
 * tinted from outer to inner. */
static inline void cell_emit_multi_levels(const ScalarField *src,
                                          int x, int y, int n_levels,
                                          float *gl, uint8_t *bn, char *gy)
{
    cell_skip(gl, bn, gy);
    for (int level = 0; level < n_levels; level++) {
        float t = (float)(level + 1) / (float)(n_levels + 1);
        int   c = ms_classify(src, x, y, t);
        if (c != 0 && c != 15)
            cell_emit(gl, bn, gy, level & 3, '+');
    }
}

/* TOPO — 8 evenly-spaced lines: a classic contour map. */
static void pattern_topo(const ScalarField *src, int x, int y,
                         float user_t, float field_time, float sweep_phase,
                         float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_multi_levels(src, x, y, TOPO_N_LEVELS, gl, bn, gy);
}

/* DENSE — 16 lines packed close together, so even gentle bumps show. */
static void pattern_dense(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_multi_levels(src, x, y, DENSE_N_LEVELS, gl, bn, gy);
}

/* SPARSE — just 4 lines: only the big shapes. */
static void pattern_sparse(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_multi_levels(src, x, y, SPARSE_N_LEVELS, gl, bn, gy);
}

/* STEPS — like TOPO but keeps the real line characters (the highest
 * level wins each square), so you see both the rings and their slope. */
static void pattern_steps(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_skip(gl, bn, gy);
    for (int level = 0; level < STEPS_N_LEVELS; level++) {
        float t = (float)(level + 1) / (float)(STEPS_N_LEVELS + 1);
        int   c = ms_classify(src, x, y, t);
        if (c != 0 && c != 15)
            cell_emit(gl, bn, gy, level & 3, ms_case_glyph[c]);
    }
}

/* RAINBOW — 12 lines, colour mixed up from both the level and the
 * square's shape so the colour shifts along each line. */
static void pattern_rainbow(const ScalarField *src, int x, int y,
                            float user_t, float field_time, float sweep_phase,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_skip(gl, bn, gy);
    for (int level = 0; level < RAINBOW_N_LEVELS; level++) {
        float t = (float)(level + 1) / (float)(RAINBOW_N_LEVELS + 1);
        int   c = ms_classify(src, x, y, t);
        if (c != 0 && c != 15)
            cell_emit(gl, bn, gy, (level + c) & 3, '+');
    }
}

/* ---------- Tier 3 — PAIRS: a few set lines ------------------------- */

/* DUAL — two fixed lines, a low one and a high one, in different
 * colours. */
static void pattern_dual(const ScalarField *src, int x, int y,
                         float user_t, float field_time, float sweep_phase,
                         float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_skip(gl, bn, gy);
    int c_lo = ms_classify(src, x, y, DUAL_T_LOW);
    if (c_lo > 0 && c_lo < 15) cell_emit(gl, bn, gy, 1, ms_case_glyph[c_lo]);
    int c_hi = ms_classify(src, x, y, DUAL_T_HIGH);
    if (c_hi > 0 && c_hi < 15) cell_emit(gl, bn, gy, 3, ms_case_glyph[c_hi]);
}

/* SHELL — fill every square whose value sits in a narrow band of
 * heights, making the contour look like a solid wall instead of a
 * thin line. */
static void pattern_shell(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    float v = scalar_field_at(src, x, y);
    if (v >= SHELL_T_LOW && v <= SHELL_T_HIGH)
        cell_emit(gl, bn, gy, 2, '#');
    else
        cell_skip(gl, bn, gy);
}

/* INNER — only the high line: tight loops around the field's peaks. */
static void pattern_inner(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_case_glyph(src, x, y, INNER_T, 3, gl, bn, gy);
}

/* OUTER — only the low line: the long, wandering coastline near the
 * field's low points. */
static void pattern_outer(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_case_glyph(src, x, y, OUTER_T, 1, gl, bn, gy);
}

/* SANDWICH — three lines (low, middle, high), each its own colour. The
 * nested-rings look of TOPO but with only three levels. */
static void pattern_sandwich(const ScalarField *src, int x, int y,
                             float user_t, float field_time, float sweep_phase,
                             float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_skip(gl, bn, gy);
    int c_lo  = ms_classify(src, x, y, SANDWICH_T_LOW);
    int c_mid = ms_classify(src, x, y, SANDWICH_T_MID);
    int c_hi  = ms_classify(src, x, y, SANDWICH_T_HIGH);
    if (c_lo  > 0 && c_lo  < 15) cell_emit(gl, bn, gy, 1, ms_case_glyph[c_lo ]);
    if (c_mid > 0 && c_mid < 15) cell_emit(gl, bn, gy, 2, ms_case_glyph[c_mid]);
    if (c_hi  > 0 && c_hi  < 15) cell_emit(gl, bn, gy, 3, ms_case_glyph[c_hi ]);
}

/* ---------- Tier 4 — TOPOLOGY: show only certain squares ------------ */

/* SADDLE — the full line, but the two awkward "could-go-two-ways"
 * squares are flagged bright with an 'X'; everything else stays dim. */
static void pattern_saddle(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    if (ms_case_is_saddle(c))      cell_emit(gl, bn, gy, 3, 'X');
    else                            cell_emit(gl, bn, gy, 1, ms_case_glyph[c]);
}

/* ORTHO — draw only the squares with a flat horizontal or vertical cut,
 * showing the straight parts of the contour. */
static void pattern_ortho(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (!ms_case_is_ortho(c)) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, ms_case_glyph[c]);
}

/* DIAGONALS — the opposite of ORTHO: only the slanted, corner-cutting
 * squares. */
static void pattern_diagonals(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (!ms_case_is_diagonal(c)) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, ms_case_glyph[c]);
}

/* JUNCTIONS — find the awkward "two-ways" squares anywhere in the
 * field's height range and mark them with a bright 'X'; the user's
 * current line is drawn faintly underneath for context.
 *
 * Why check many levels instead of one: at any single level these
 * awkward squares are rare, so checking a stack of levels gathers them
 * all and gives a fuller picture of where the field is genuinely
 * ambiguous. */
static void pattern_junctions(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;

    /* First: does this square go two ways at any of the levels? */
    bool is_saddle_somewhere = false;
    for (int level = 0; level < TOPO_N_LEVELS; level++) {
        float t = (float)(level + 1) / (float)(TOPO_N_LEVELS + 1);
        if (ms_case_is_saddle(ms_classify(src, x, y, t))) {
            is_saddle_somewhere = true;
            break;
        }
    }
    if (is_saddle_somewhere) { cell_emit(gl, bn, gy, 3, 'X'); return; }

    /* Otherwise draw the user's line faintly, so the X's stand out. */
    int c = ms_classify(src, x, y, user_t);
    if (c > 0 && c < 15) cell_emit(gl, bn, gy, 1, ms_case_glyph[c]);
    else                 cell_skip(gl, bn, gy);
}

/* ROUGHNESS — the full line, colour chosen by each square's shape, so
 * the line looks jagged and "rough" where its shape changes quickly. */
static void pattern_roughness(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, c & 3, ms_case_glyph[c]);
}

/* ---------- Tier 5 — REGIONS: fill areas, not lines ---------------- */

/* ABOVE — fill the "land": every square above the sea level. */
static void pattern_above(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    if (scalar_field_at(src, x, y) >= user_t) cell_emit(gl, bn, gy, 3, '#');
    else                          cell_skip(gl, bn, gy);
}

/* BELOW — the opposite of ABOVE: fill the "sea". */
static void pattern_below(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    if (scalar_field_at(src, x, y) <  user_t) cell_emit(gl, bn, gy, 1, '#');
    else                          cell_skip(gl, bn, gy);
}

/* STRIPE — fill a band of heights around the user's level. Like SHELL,
 * but it follows the level the user sets. */
static void pattern_stripe(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    float v = scalar_field_at(src, x, y);
    if (v >= user_t - STRIPE_HALFWIDTH && v <= user_t + STRIPE_HALFWIDTH)
        cell_emit(gl, bn, gy, 2, '#');
    else
        cell_skip(gl, bn, gy);
}

/* ZEBRA — slice the height range into equal bands and fill every other
 * one, giving solid-and-empty stripes across the field. */
static void pattern_zebra(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    float v   = scalar_field_at(src, x, y);
    int   band = (int)(v * (float)ZEBRA_N_BANDS);
    if ((band & 1) == 0) cell_emit(gl, bn, gy, band & 3, '#');
    else                 cell_skip(gl, bn, gy);
}

/* PERIMETER — fill every square the contour touches with a solid '#',
 * so the contour reads as a thick outline instead of thin glyphs. */
static void pattern_perimeter(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, '#');
}

/* ---------- Tier 6 — ANIMATED: the sea level moves itself ----------- */

/* SWEEP — one line whose level slides smoothly up and down on its own,
 * so the contour appears to sweep through all the field's heights. */
static void pattern_sweep(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time;
    float lo  = CONTOUR_T_MIN;
    float hi  = CONTOUR_T_MAX;
    float mid = 0.5f * (lo + hi);
    float amp = 0.5f * (hi - lo);
    float t   = mid + amp * sinf(sweep_phase * 2.0f * (float)M_PI);
    cell_emit_case_glyph(src, x, y, t, 2, gl, bn, gy);
}

/* PULSE — like SWEEP but the level moves with sharper rises and falls,
 * giving the contour a breathing rhythm. */
static void pattern_pulse(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)sweep_phase;
    float lo  = CONTOUR_T_MIN;
    float hi  = CONTOUR_T_MAX;
    float s   = sinf(field_time * (2.0f * (float)M_PI / PULSE_PERIOD_SEC));
    float t   = lo + (hi - lo) * (s * s);     /* squaring sharpens the in/out */
    cell_emit_case_glyph(src, x, y, t, 2, gl, bn, gy);
}

/* RIPPLE — the level nudges up and down in rings spreading out from the
 * centre, so the contour ripples like a stone dropped in a pond. */
static void pattern_ripple(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)sweep_phase;
    float cx   = 0.5f * (float)src->w;
    float cy   = 0.5f * (float)src->h;
    float dx   = (float)x - cx;
    float dy   = (float)y - cy;
    float d    = sqrtf(dx * dx + dy * dy);
    float t    = user_t + RIPPLE_AMP * sinf(d / RIPPLE_FREQ_CELLS
                                            - field_time * RIPPLE_SPEED);
    cell_emit_case_glyph(src, x, y, t, 2, gl, bn, gy);
}

/* STAIRS — the level jumps between a few fixed steps instead of sliding,
 * so the contour snaps from one height to the next. */
static void pattern_stairs(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)sweep_phase;
    float phase = fmodf(field_time, STAIRS_PERIOD_SEC) / STAIRS_PERIOD_SEC;
    int   step  = (int)(phase * (float)STAIRS_N_LEVELS);
    if (step >= STAIRS_N_LEVELS) step = STAIRS_N_LEVELS - 1;
    float t     = (float)(step + 1) / (float)(STAIRS_N_LEVELS + 1);
    cell_emit_case_glyph(src, x, y, t, (step & 3), gl, bn, gy);
}

/* CHAOS — each square jitters its level by a little random amount that
 * changes over time, so the contour shimmers around its real shape. */
static void pattern_chaos(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)sweep_phase;
    uint32_t h32 = hash32((uint32_t)x * 374761393u
                        + (uint32_t)y * 668265263u
                        + (uint32_t)(field_time * 7.0f) * 2147483647u);
    float jitter = ((float)(h32 >> 8) * (1.0f / 16777215.0f) - 0.5f) * 2.0f;
    float t      = user_t + jitter * CHAOS_AMP;
    cell_emit_case_glyph(src, x, y, t, 2, gl, bn, gy);
}

/* ---------- Dispatch table ------------------------------------------- */

typedef void (*ContourPatternFn)(const ScalarField *src, int x, int y,
                                 float user_t, float field_time, float sweep_phase,
                                 float *out_glow, uint8_t *out_band, char *out_glyph);

/*
 * ContourPattern — one row of the table that lists all 30 patterns:
 * its HUD name, its tier label, and the function that draws it.
 *
 * Keeping the patterns in a table means the per-square loop just calls
 * "the current pattern's function" through this row — no big switch to
 * pick the right one. Adding a pattern is: write its function, add an
 * enum, add a row here. The fixed-size table keyed by the enum makes
 * the compiler complain if anything is missing or out of order.
 *
 * name   : the HUD label, padded out to 10 characters so the columns
 *          line up as you flip through patterns.
 * tier   : a short "N-XXXX" tag (tier number plus a 4-letter group
 *          name) shown next to the name.
 * sample : the pattern's drawing function. Called once per square every
 *          sim tick; it writes the square's three answers (draw?,
 *          colour, character).
 */
typedef struct {
    const char       *name;     /* padded to 10 chars for HUD alignment */
    const char       *tier;     /* "N-LABEL" tag, 7 chars               */
    ContourPatternFn  sample;
} ContourPattern;

static const ContourPattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — SINGLE */
    [PATTERN_CONTOUR]    = { "CONTOUR   ", "1-SING ", pattern_contour    },
    [PATTERN_THICK]      = { "THICK     ", "1-SING ", pattern_thick      },
    [PATTERN_INVERT]     = { "INVERT    ", "1-SING ", pattern_invert     },
    [PATTERN_DOTS]       = { "DOTS      ", "1-SING ", pattern_dots       },
    [PATTERN_UNIFORM]    = { "UNIFORM   ", "1-SING ", pattern_uniform    },
    /* Tier 2 — MULTI */
    [PATTERN_TOPO]       = { "TOPO      ", "2-MULT ", pattern_topo       },
    [PATTERN_DENSE]      = { "DENSE     ", "2-MULT ", pattern_dense      },
    [PATTERN_SPARSE]     = { "SPARSE    ", "2-MULT ", pattern_sparse     },
    [PATTERN_STEPS]      = { "STEPS     ", "2-MULT ", pattern_steps      },
    [PATTERN_RAINBOW]    = { "RAINBOW   ", "2-MULT ", pattern_rainbow    },
    /* Tier 3 — PAIRS */
    [PATTERN_DUAL]       = { "DUAL      ", "3-PAIR ", pattern_dual       },
    [PATTERN_SHELL]      = { "SHELL     ", "3-PAIR ", pattern_shell      },
    [PATTERN_INNER]      = { "INNER     ", "3-PAIR ", pattern_inner      },
    [PATTERN_OUTER]      = { "OUTER     ", "3-PAIR ", pattern_outer      },
    [PATTERN_SANDWICH]   = { "SANDWICH  ", "3-PAIR ", pattern_sandwich   },
    /* Tier 4 — TOPOLOGY */
    [PATTERN_SADDLE]     = { "SADDLE    ", "4-TOPO ", pattern_saddle     },
    [PATTERN_ORTHO]      = { "ORTHO     ", "4-TOPO ", pattern_ortho      },
    [PATTERN_DIAGONALS]  = { "DIAGONALS ", "4-TOPO ", pattern_diagonals  },
    [PATTERN_JUNCTIONS]  = { "JUNCTIONS ", "4-TOPO ", pattern_junctions  },
    [PATTERN_ROUGHNESS]  = { "ROUGHNESS ", "4-TOPO ", pattern_roughness  },
    /* Tier 5 — REGIONS */
    [PATTERN_ABOVE]      = { "ABOVE     ", "5-REGN ", pattern_above      },
    [PATTERN_BELOW]      = { "BELOW     ", "5-REGN ", pattern_below      },
    [PATTERN_STRIPE]     = { "STRIPE    ", "5-REGN ", pattern_stripe     },
    [PATTERN_ZEBRA]      = { "ZEBRA     ", "5-REGN ", pattern_zebra      },
    [PATTERN_PERIMETER]  = { "PERIMETER ", "5-REGN ", pattern_perimeter  },
    /* Tier 6 — ANIMATED */
    [PATTERN_SWEEP]      = { "SWEEP     ", "6-ANIM ", pattern_sweep      },
    [PATTERN_PULSE]      = { "PULSE     ", "6-ANIM ", pattern_pulse      },
    [PATTERN_RIPPLE]     = { "RIPPLE    ", "6-ANIM ", pattern_ripple     },
    [PATTERN_STAIRS]     = { "STAIRS    ", "6-ANIM ", pattern_stairs     },
    [PATTERN_CHAOS]      = { "CHAOS     ", "6-ANIM ", pattern_chaos      },
};

static const char *pattern_name(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?         ";
    return noise_patterns[p].name;
}

static const char *pattern_tier(Pattern p)
{
    if ((unsigned)p >= (unsigned)N_PATTERNS) return "?      ";
    return noise_patterns[p].tier;
}

/* ===================================================================== */
/* §8  scene — ContourGrid + PatternState + PaletteState + Scene          */
/* ===================================================================== */

/*
 * ContourGrid — the scratch sheet between deciding and drawing. For
 * each square it holds three things: should we draw it, what colour,
 * and which character.
 *
 * This splits the work in two. Once per sim tick, the active pattern
 * fills this sheet in (it figures out the right character, whether
 * that's a real contour line or its own '+' or '#'). Once per frame,
 * the renderer just reads the sheet and paints — it never needs to
 * know what marching squares is.
 *
 * One of these lives on Scene (§8). Index a square the same row-by-row
 * way as the field: cell (x, y) is at y * w + x.
 *
 * w, h    : grid size, always the same as the field's.
 * count   : w * h, kept handy for the clear-everything loop.
 * glow[]  : draw flag per square — above 0 means draw, 0 or below means
 *           skip. It's a float, not a bool, leaving room for a future
 *           "how strong" use; today the renderer just checks > 0.
 * band[]  : which theme colour (0..3) to draw the square in.
 * glyph[] : the character to draw. 0 also means skip — a second safety
 *           check in case a pattern marked a square to draw but forgot
 *           to set a character.
 */
typedef struct {
    int      w, h;
    int      count;
    float    glow [CELLS_MAX];
    uint8_t  band [CELLS_MAX];
    char     glyph[CELLS_MAX];
} ContourGrid;

static inline int contour_grid_idx(const ContourGrid *cg, int x, int y)
{
    return y * cg->w + x;
}

static void contour_grid_reset(ContourGrid *cg, int w, int h)
{
    cg->w     = w;
    cg->h     = h;
    cg->count = w * h;
    for (int i = 0; i < cg->count; i++) {
        cg->glow [i] = 0.0f;
        cg->band [i] = 0;
        cg->glyph[i] = 0;
    }
}

/*
 * PatternState — everything about "what's happening right now": which
 * pattern is showing and the handful of numbers the patterns read.
 * These five move and are read together, and all get handed to the
 * per-square function, so they're grouped here.
 *
 * Lives on Scene (§8). The keys change them (n/p the pattern, </> the
 * level, +/- the speed); scene_tick advances the two clocks each tick;
 * the patterns and the HUD read them.
 *
 * current     : which pattern is active. Starts at CONTOUR.
 * user_t      : the sea level the user picks with </>, kept in range.
 *               Most patterns use it as-is; the animated ones build on
 *               it with time.
 * field_time  : how long the field has been drifting, in seconds. Folded
 *               into the field so it appears to scroll. Only grows; r
 *               resets it.
 * sweep_phase : a 0..1 counter that loops once every SWEEP_T_PERIOD_SEC
 *               (at normal speed). Drives the SWEEP pattern's moving
 *               level.
 * drift_mult  : speed-up factor (a power of two) applied to both clocks.
 *               Doubled/halved by +/-.
 */
typedef struct {
    Pattern current;
    float   user_t;
    float   field_time;
    float   sweep_phase;
    int     drift_mult;
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current     = PATTERN_CONTOUR;
    ps->user_t      = CONTOUR_T_DEFAULT;
    ps->field_time  = 0.0f;
    ps->sweep_phase = 0.0f;
    ps->drift_mult  = DRIFT_MULT_DEF;
}

/* Move both clocks forward one frame. The drift clock just keeps
 * growing (scrolls the field); the sweep counter wraps back around
 * after it passes 1. */
static void pattern_state_advance_clocks(PatternState *ps, float dt)
{
    float drift = (float)ps->drift_mult;
    ps->field_time  += FIELD_DRIFT * drift * dt;
    ps->sweep_phase += dt * drift / SWEEP_T_PERIOD_SEC;
    if (ps->sweep_phase > 1.0f) ps->sweep_phase -= 1.0f;
}

/* Step to the next/previous pattern, wrapping around the ends. */
static void pattern_state_cycle_to_next(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + 1) % N_PATTERNS);
}
static void pattern_state_cycle_to_prev(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + N_PATTERNS - 1) % N_PATTERNS);
}

/* Double or halve the drift speed, staying within the allowed range. */
static void pattern_state_drift_faster(PatternState *ps)
{
    if (ps->drift_mult < DRIFT_MULT_MAX) ps->drift_mult *= 2;
    if (ps->drift_mult > DRIFT_MULT_MAX) ps->drift_mult  = DRIFT_MULT_MAX;
}
static void pattern_state_drift_slower(PatternState *ps)
{
    ps->drift_mult /= 2;
    if (ps->drift_mult < DRIFT_MULT_MIN) ps->drift_mult = DRIFT_MULT_MIN;
}

/* Nudge the sea level up or down one step, staying within range. */
static void pattern_state_raise_threshold(PatternState *ps)
{
    ps->user_t += CONTOUR_T_STEP;
    if (ps->user_t > CONTOUR_T_MAX) ps->user_t = CONTOUR_T_MAX;
}
static void pattern_state_lower_threshold(PatternState *ps)
{
    ps->user_t -= CONTOUR_T_STEP;
    if (ps->user_t < CONTOUR_T_MIN) ps->user_t = CONTOUR_T_MIN;
}

/*
 * PaletteState — just which theme is showing right now.
 *
 * A whole struct for one number is overkill data-wise, but it gives the
 * "current colour scheme" a tidy home on Scene and matches how the
 * sibling demos are laid out. Lives on Scene (§8); t/T change it.
 *
 * current : index into themes[]. After changing it you MUST call
 *           theme_apply() — this number only says which theme; ncurses
 *           holds the actual colours.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p) { p->current = 0; }

/* Step to the next/previous theme, wrapping around. The caller must
 * then call theme_apply() to actually load the new colours. */
static void palette_state_cycle_to_next(PaletteState *p)
{
    p->current = (p->current + 1) % N_THEMES;
}
static void palette_state_cycle_to_prev(PaletteState *p)
{
    p->current = (p->current + N_THEMES - 1) % N_THEMES;
}

/*
 * Scene — holds all the moving parts of the demo in one place, shared
 * by the main loop, the per-tick update, and the renderer.
 *
 * The data flows through it in order:
 *
 *   source  — the field of numbers we contour      (the data)
 *      ↓ (read by ms_classify and the patterns)
 *   pattern — which pattern, the sea level, clocks  (the controls)
 *      ↓ (decides what to draw)
 *   grid    — the draw/colour/character sheet       (the output)
 *      ↓ (painted by the renderer)
 *   palette — which theme's colours                 (the colours)
 *   paused  — whether the demo is frozen            (the on/off)
 *
 * One instance, owned by App (§10), only ever touched from the main
 * thread. scene_init sets it up, scene_reset re-seeds it (on r or a
 * resize), scene_tick advances it one step.
 *
 * paused : while true, scene_tick does nothing — the clocks and the
 *          field freeze and the renderer keeps showing the last frame.
 *          Toggled by space.
 */
typedef struct {
    ScalarField  source;   /* the field of numbers we contour            */
    ContourGrid  grid;     /* the draw/colour/character sheet            */
    PatternState pattern;  /* active pattern, sea level, clocks          */
    PaletteState palette;  /* active theme                               */
    bool         paused;   /* if true, the demo is frozen                */
} Scene;

/* Refill the field for the current moment in time. */
static void scene_recompute_field(Scene *s)
{
    scalar_field_rebuild(&s->source, s->pattern.field_time);
}

/* Walk every square and let the chosen pattern fill in its three
 * answers, passing along the current controls (sea level, the two
 * clocks). */
static void contour_grid_evaluate_all_cells(ContourGrid *grid,
                                            const ScalarField *src,
                                            ContourPatternFn sample,
                                            float ut, float ft, float sp)
{
    for (int y = 0; y < grid->h; y++) {
        for (int x = 0; x < grid->w; x++) {
            int idx = contour_grid_idx(grid, x, y);
            sample(src, x, y, ut, ft, sp,
                   &grid->glow [idx],
                   &grid->band [idx],
                   &grid->glyph[idx]);
        }
    }
}

/* Run the active pattern over the whole grid. Expects the field to be
 * up to date already (the caller refills it first). */
static void scene_evaluate_contours(Scene *s)
{
    /* look up the current pattern's drawing function... */
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    ContourPatternFn sample = noise_patterns[active].sample;

    /* ...then let it fill in every square. */
    contour_grid_evaluate_all_cells(&s->grid, &s->source, sample,
                                    s->pattern.user_t,
                                    s->pattern.field_time,
                                    s->pattern.sweep_phase);
}

static void scene_reset(Scene *s, int mw, int mh)
{
    scalar_field_reset (&s->source, mw, mh);
    contour_grid_reset (&s->grid,   mw, mh);
    s->pattern.field_time  = 0.0f;
    s->pattern.sweep_phase = 0.0f;
    s->pattern.user_t      = CONTOUR_T_DEFAULT;
    scene_recompute_field   (s);
    scene_evaluate_contours (s);    /* draw a first frame so a paused start isn't blank */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    pattern_state_init (&s->pattern);
    palette_state_init (&s->palette);
    s->paused = false;
    scene_reset(s, mw, mh);
}

/* One step of the demo: move the clocks, refill the field, redraw the
 * grid. Does nothing while paused. */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    pattern_state_advance_clocks(&s->pattern, dt);
    scene_recompute_field   (s);
    scene_evaluate_contours (s);
}

/* ===================================================================== */
/* §9  screen                                                             */
/* ===================================================================== */

/*
 * Screen — remembers how big the terminal is right now (width and
 * height in characters), so the drawing code can centre the grid and
 * right-align the HUD without asking ncurses every frame. Refreshed at
 * startup and whenever the window is resized.
 *
 * rows includes the HUD rows; drawing code subtracts those when it
 * needs just the usable area.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *s)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* ---------- scene_draw helpers (field painter) ----------------------- */

/* Work out where the top-left of the grid goes so it sits centred on
 * screen, leaving the HUD rows clear top and bottom. If the grid is
 * bigger than the space, it just starts at the edge. */
static void viewport_centre_grid_origin(int cols, int rows,
                                        int grid_w, int grid_h,
                                        int *gx0, int *gy0)
{
    int interior_h = rows - HUD_BAND_RESERVED_ROWS;
    *gx0 = (cols   - grid_w) / 2;
    *gy0 = (interior_h - grid_h) / 2 + HUD_TOP_ROWS;
    if (*gx0 < 0)            *gx0 = 0;
    if (*gy0 < HUD_TOP_ROWS) *gy0 = HUD_TOP_ROWS;
}

/* Draw one square at screen position (sx, sy): skip it unless it's
 * marked to draw and has a real character, then paint that character
 * in its theme colour. */
static void contour_cell_paint(const ContourGrid *grid, int gx, int gy,
                               int sx, int sy)
{
    int  idx = contour_grid_idx(grid, gx, gy);
    if (grid->glow[idx] <= 0.0f) return;
    char glyph = grid->glyph[idx];
    if (glyph == 0 || glyph == ' ') return;

    int pair = PAIR_BAND_BASE + (grid->band[idx] & 3);
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Paint the whole grid: centre it, then draw each square that's on
 * screen. The pattern already decided everything; this just paints. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const ContourGrid *grid = &s->grid;
    int gx0, gy0;
    viewport_centre_grid_origin(cols, rows, grid->w, grid->h, &gx0, &gy0);

    for (int y = 0; y < grid->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < grid->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            contour_cell_paint(grid, x, y, sx, sy);
        }
    }
}

/* ---------- screen_draw helpers (HUD layout) ------------------------- */

/* Which sea level the HUD should show. Usually it's the user's level,
 * but SWEEP moves the level itself, so we report where SWEEP has it
 * right now (matching pattern_sweep). */
static float hud_resolve_displayed_threshold(const PatternState *ps)
{
    if (ps->current != PATTERN_SWEEP) return ps->user_t;
    float lo  = CONTOUR_T_MIN;
    float hi  = CONTOUR_T_MAX;
    float mid = 0.5f * (lo + hi);
    float amp = 0.5f * (hi - lo);
    return mid + amp * sinf(ps->sweep_phase * 2.0f * (float)M_PI);
}

/* The "MARCHING SQUARES" title in the top-left corner. */
static void hud_draw_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MARCHING SQUARES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* The status readout in the top-right: frame rate, sim rate, state,
 * which pattern, and the sea level. */
static void hud_draw_top_right_status(int cols, double fps, int sim_fps,
                                      const char *state_str,
                                      int current_idx_zero_based,
                                      float shown_t)
{
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  T:%.2f ",
             fps, sim_fps, state_str,
             current_idx_zero_based + 1, N_PATTERNS, (double)shown_t);
    int hx = cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Draw one labelled field on the second HUD row and return where the
 * next field should start. */
static int hud_field_bold_label(int x, const char *fmt,
                                const char *val, int width)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, fmt, val);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + width;
}

/* The "palette:####" swatch: the four theme colours shown as coloured
 * '#' marks, so you can see the current theme at a glance. */
static int hud_field_palette_swatch(int x)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_PALETTE_LABEL_W;
    for (int i = 0; i < HUD_N_PALETTE_BANDS; i++) {
        int p = PAIR_BAND_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    return x;
}

/* The trailing extras on the second HUD row: zoom, drift speed, and
 * grid size. */
static void hud_field_meta(int x, int drift_mult, int grid_w, int grid_h)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  scale:%.2f  drift:x%d  map:%dx%d ",
             FIELD_SCALE, drift_mult, grid_w, grid_h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* The second HUD row: pattern, tier, theme, the colour swatch, and the
 * extras, laid out left to right. */
static void hud_draw_param_row(const Scene *s)
{
    const PatternState *ps = &s->pattern;
    int x = HUD_LEFT_MARGIN;
    x = hud_field_bold_label(x, " pattern:%-10s ", pattern_name(ps->current),
                             HUD_PATTERN_FIELD_W);
    x = hud_field_bold_label(x, " tier:%-7s ",     pattern_tier(ps->current),
                             HUD_TIER_FIELD_W);
    x = hud_field_bold_label(x, " theme:%-8s ",    themes[s->palette.current].name,
                             HUD_THEME_FIELD_W);
    x = hud_field_palette_swatch(x);
    hud_field_meta(x, ps->drift_mult, s->grid.w, s->grid.h);
}

/* The key list along the bottom of the screen. */
static void hud_draw_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:pattern  t/T:theme  </>:T  +/-:drift  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* Build one full frame: clear, draw the contours, then lay the HUD
 * (title, status, parameter row, key list) on top. */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const PatternState *ps        = &s->pattern;
    const char         *state_str = s->paused ? "PAUSED    "
                                              : pattern_name(ps->current);
    float               shown_t   = hud_resolve_displayed_threshold(ps);

    hud_draw_top_left_title();
    hud_draw_top_right_status(sc->cols, fps, sim_fps, state_str,
                              (int)ps->current, shown_t);
    hud_draw_param_row(s);
    hud_draw_bottom_hint(sc->rows);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §10 app                                                                */
/* ===================================================================== */

/*
 * App — the whole program in one box: the scene, the screen size, the
 * target sim rate, the grid size, and two flags the signal handlers
 * use to talk to the main loop.
 *
 * There's exactly one, g_app, at file scope. It has to be a global
 * because signal handlers can't take arguments and can only safely
 * touch a couple of simple global flags — that's why running and
 * need_resize are here and are the special sig_atomic_t type. The
 * handlers just flip a flag; the main loop notices and does the real
 * work at the start of the next frame.
 *
 * sim_fps     : how many times a second the demo updates (separate from
 *               how often we redraw); ]/[ change it.
 * map_w/h     : the grid size, worked out from the window size and
 *               re-done on resize.
 * running     : set to 0 to quit — by Ctrl-C/kill or by q/ESC.
 * need_resize : set when the window was resized; the loop handles it
 *               next frame.
 *
 * The update loop uses Glenn Fiedler's fixed-timestep approach:
 * https://gafferongames.com/post/fix_your_timestep/
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

/* Pick the grid size from the current window: as big as the window,
 * minus the HUD rows, kept within the smallest/largest we allow. */
static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

/* Handle a window resize: re-read the new size, repick the grid size,
 * and rebuild the scene to fit. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* Speed up or slow down how often the demo updates, kept within range. */
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

/* Act on one keypress. Returns false only when the user asked to quit
 * (q/Q/ESC); true otherwise. */
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
        scene_reset(s, app->map_w, app->map_h);
        break;

    case '=': case '+':  pattern_state_drift_faster   (&s->pattern); break;
    case '-':            pattern_state_drift_slower   (&s->pattern); break;
    case '>': case '.':  pattern_state_raise_threshold(&s->pattern); break;
    case '<': case ',':  pattern_state_lower_threshold(&s->pattern); break;
    case ']':            app_sim_rate_faster          ( app);        break;
    case '[':            app_sim_rate_slower          ( app);        break;
    case 'n': case 'N':  pattern_state_cycle_to_next  (&s->pattern); break;
    case 'p': case 'P':  pattern_state_cycle_to_prev  (&s->pattern); break;

    case 't':
        palette_state_cycle_to_next(&s->palette);
        theme_apply(s->palette.current);
        break;
    case 'T':
        palette_state_cycle_to_prev(&s->palette);
        theme_apply(s->palette.current);
        break;

    default: break;
    }
    return true;
}

/* ---------- main-loop helpers ---------------------------------------- */

/* Wire up the signals: quit cleanly on Ctrl-C/kill, note window
 * resizes. The handlers only flip the two flags, which is all that's
 * safe to do inside a signal handler. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Start everything up: ncurses, window size, and the scene. */
static void app_bootstrap(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* If the window was resized, handle it now and reset the frame clock,
 * so the time the resize took doesn't count as one giant slow frame. */
static void app_handle_pending_resize(App *app,
                                      int64_t *frame_time,
                                      int64_t *sim_accum)
{
    if (!app->need_resize) return;
    app_do_resize(app);
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

/* How much time passed since the last frame, but capped: if one frame
 * was very slow, we don't let it pile up a huge backlog of updates. */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    int64_t dt_cap = (int64_t)SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    if (dt > dt_cap) dt = dt_cap;
    return dt;
}

/* Run as many fixed-size update steps as the elapsed time allows, so
 * the demo always updates at its set rate no matter how the frame rate
 * wobbles. */
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

/* Keep a running average frame rate, refreshed every FPS_UPDATE_MS. */
static void app_update_fps_meter(int64_t dt,
                                 int *frame_count,
                                 int64_t *fps_accum,
                                 double *fps_display)
{
    (*frame_count)++;
    *fps_accum += dt;
    if (*fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
        *fps_display = (double)(*frame_count)
                     / ((double)(*fps_accum) / (double)NS_PER_SEC);
        *frame_count = 0;
        *fps_accum   = 0;
    }
}

/* Sleep off whatever's left of the frame's time budget, so we don't
 * spin the CPU drawing faster than we need to. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* Draw the frame and push it to the terminal. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* Check for a keypress without waiting and act on it. Returns false
 * only if the user asked to quit. */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* The main loop. Set up, then each frame: handle any resize, see how
 * much time passed, run the due updates, refresh the fps reading, sleep
 * to pace the frame, draw, and check the keyboard. Quit when asked. */
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
