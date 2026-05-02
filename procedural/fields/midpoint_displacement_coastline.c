/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * midpoint_displacement_coastline.c
 *   — 1-D midpoint-displacement fractals, 5 silhouette patterns.
 *
 * DEMO: A single endpoint pair becomes a jagged fractal line through
 *       repeated midpoint-displacement subdivision: each segment's
 *       midpoint is set to the average of its endpoints plus a random
 *       jitter, with the jitter halving every level. The resulting
 *       polyline looks naturally rough — like a coastline, mountain
 *       silhouette, or lightning bolt. Five pattern presets configure
 *       the line count, baseline heights, roughness, and rendering
 *       style:
 *         COASTLINE — single horizon, water below, sky above
 *         MOUNTAINS — 4 stacked ranges with depth shading (parallax)
 *         CITY      — high-roughness silhouette, building fill below
 *         VALLEY    — top + bottom lines forming a canyon
 *         WAVES     — 6 horizontal lines stacked as ocean swell
 *       Each line slowly morphs between two random shapes over ~10s,
 *       so the silhouette evolves continuously rather than freezing.
 *       Cycle patterns with n/p, themes with t/T, glyph sets with g/G.
 *
 * Study alongside:
 *   ../generational/diamond_square_heightmap_showcase.c — the 2-D
 *       cousin of midpoint displacement. Same recursive halving idea
 *       but in two dimensions; produces heightmaps instead of silhouettes.
 *   ./perin_noise_flow_showcase.c — Perlin noise also produces smooth
 *       fractal-like fields; midpoint displacement is the older, simpler
 *       algorithm that inspired much of fractal terrain synthesis.
 *
 * Section map:
 *   §1 config   — grid, patterns, palette, themes, glyph sets
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 md       — midpoint displacement core
 *   §6 patterns — 5 line configurations + per-pattern rendering
 *   §7 scene    — Field, scene state, morph animation
 *   §8 screen   — ASCII render: silhouettes + fills
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (regenerate all lines from scratch)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   g / G      next / previous glyph set
 *   + / =      faster morph
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra midpoint_displacement_coastline.c \
 *       -o md_coast -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : 1-D midpoint displacement (Fournier, Fussell &
 *                  Carpenter 1982). For a line from (0, y_left) to
 *                  (n-1, y_right):
 *
 *                    1. line[0] = y_left, line[n-1] = y_right.
 *                    2. step = n − 1, amplitude = A₀.
 *                    3. While step > 1:
 *                       For each midpoint i = step/2 + k·step:
 *                         line[i] = (line[i−step/2] + line[i+step/2]) / 2
 *                                  + random(−amp, +amp)
 *                       step ← step / 2;  amp ← amp · roughness.
 *                    4. line[] now contains a fractal polyline.
 *
 *                  The choice of `roughness` (typically 0.5–0.7)
 *                  controls how rough the line is. 0.5 gives smooth
 *                  rolling hills; 0.7 produces sharp jagged peaks
 *                  (good for cities and lightning).
 *
 *                  The output is a 1-D Brownian-like fractal — same
 *                  family as fBm noise, but built by recursive
 *                  subdivision rather than summed octaves. Cheap and
 *                  classic; what the original Star Trek II Genesis
 *                  effect used to render planet horizons in 1982.
 *
 *                  Animation: we generate two random lines (A and B)
 *                  per layer and continuously lerp between them. When
 *                  the lerp reaches 1, B becomes A and we regenerate
 *                  B. Smooth morphing with no visible "snap".
 *
 * Data-structure : Two float buffers per line (line_a, line_b) plus
 *                  the current interpolated "live" buffer used for
 *                  rendering. Up to 6 lines per pattern (WAVES needs
 *                  the most). 257-point lines for full fractal
 *                  resolution; rendering interpolates to fit any
 *                  terminal width.
 *
 * Rendering      : ASCII only. Each pattern has its own render style:
 *                  COASTLINE / CITY use vertical fill below the line.
 *                  MOUNTAINS layers fills back-to-front with depth
 *                  shading. VALLEY fills outside a canyon. WAVES
 *                  draws thin lines only.
 *
 * Performance    : O(N) per line generation (each level halves point
 *                  spacing but doubles point count → linear total).
 *                  Generation only on reset / morph-roll-over —
 *                  a few hundred cheap operations every ~10 s. Per-
 *                  frame: O(N · L) interpolation + O(W · H) render.
 *
 * References     : • Fournier, Fussell & Carpenter (1982) — "Computer
 *                    rendering of stochastic models", CACM. The paper
 *                    that introduced midpoint displacement to graphics.
 *                  • Mandelbrot, B. (1977) — "Fractals: Form, Chance
 *                    and Dimension". Mathematical context for fractal
 *                    coastlines and the famous Hausdorff-dimension
 *                    "How long is the coast of Britain?" question.
 *                  • Wikipedia — "Diamond-square algorithm" (the 2-D
 *                    extension we don't use here):
 *                    https://en.wikipedia.org/wiki/Diamond-square_algorithm
 *                  • Compare ../generational/diamond_square_heightmap_showcase.c
 *                    in this same project for the 2-D variant.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a straight line from A to B. Find the midpoint. Push it up or
 * down by some random amount. You now have two segments. Repeat on
 * each — find each midpoint, displace by a SMALLER random amount.
 * Repeat again with even smaller amounts. After log₂(N) levels, the
 * line has N points and looks naturally rough — coastline-like at
 * one roughness, mountain-like at another, lightning-like at a third.
 * The whole concept is just "halve, jitter, halve, jitter, halve,
 * jitter".
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine drawing a profile of a coast. You start with two pencil
 * dots at left and right. You aim halfway between them — but instead
 * of drawing exactly there, you nudge your hand a bit (the random
 * jitter). Now you have three dots. You repeat the process between
 * each adjacent pair — find the midpoint, nudge a bit less. Then
 * again with even smaller nudges. After enough rounds you have
 * hundreds of dots forming a wandering line. That's a coastline.
 *
 * Visible layers (per pattern):
 *   COASTLINE : single line, water below, sky above. Press 'r' a
 *               few times to see how varied "naturally random"
 *               coastlines can be while always feeling natural.
 *   MOUNTAINS : 4 silhouettes back-to-front. Front line covers
 *               middle, middle covers back. Depth feels like layers
 *               of mountains receding into mist.
 *   CITY      : one HIGH-roughness line. Sharp peaks become
 *               skyscrapers, valleys become streets. Vertical
 *               building fills below.
 *   VALLEY    : two lines (top + bottom) bracketing a canyon. Both
 *               independently fractal; rocks above and below.
 *   WAVES     : 6 thin lines at different y-baselines. Looks like
 *               ocean cross-section with surface, mid, and deep
 *               waves.
 *
 * Above all: the lines MORPH continuously. Each pattern stays the
 * same configuration but the actual shapes evolve smoothly.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. For each line of the active pattern, generate two random
 *     fractals (line_a and line_b) using midpoint displacement.
 *     morph_t = 0.
 *  2. PER FRAME:
 *     a. Advance morph_t by morph_speed · dt.
 *     b. If morph_t ≥ 1: A ← B, regenerate B, morph_t = 0.
 *     c. Compute live line: line_now[i] = lerp(A[i], B[i], morph_t).
 *     d. Render lines according to the active pattern's render style.
 *  3. Periodic supernova reset re-rolls all line endpoints + B
 *     buffers for variety.
 *
 * KEY FORMULAS
 * ────────────
 *  Midpoint with jitter         :
 *    line[mid] = (line[L] + line[R]) / 2 + uniform(−A, +A)
 *  Amplitude decay              :
 *    A_{level+1} = A_level · roughness   (typically 0.5 to 0.7)
 *  Number of levels             :
 *    log₂(N − 1)
 *  Total line points after K
 *  levels                       : 2^K + 1
 *  Frame interpolation          :
 *    line_now[i] = (1 − t) · A[i] + t · B[i]
 *  Render-x to line-x           :
 *    line_x = (render_x · (n_points − 1)) / (W − 1)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • POINT COUNT MUST BE 2^K + 1. If you use a non-power-of-2-plus-1
 *    point count, the iterative subdivision misses some midpoints.
 *    We use 257 (2⁷ + 1) and interpolate at render time to fit any
 *    terminal width.
 *
 *  • AMPLITUDE STARTING TOO HIGH. If A₀ is large compared to the
 *    vertical extent of the line, points fly off-screen at the
 *    first subdivision. Choose A₀ ≈ 0.4 × line_height for natural
 *    coastlines, less for "smoother" ones.
 *
 *  • ROUGHNESS BELOW 0.5. Below ~0.5 the fractal looks too smooth;
 *    above ~0.7 it looks white-noise-jagged. The sweet spot is
 *    0.55–0.65 for classic mountain/coast shapes. CITY pattern uses
 *    higher roughness (~0.7) on purpose for spikiness.
 *
 *  • CLAMPING. Without bounds, displaced midpoints can escape the
 *    map. We clamp on render rather than during generation —
 *    clamping during generation introduces directional bias.
 *
 *  • MORPH BETWEEN INDEPENDENT FRACTALS. lerp(A, B, t) at midway
 *    looks "averaged" — neither A nor B but some smoothing of both.
 *    That's intentional; the midway state is a valid fractal of its
 *    own with reduced amplitude.
 *
 *  • PATTERN-SPECIFIC RENDER. Each pattern has its own render code.
 *    Don't try to unify them into a single "render line" function —
 *    each pattern does something distinctly different (fill below /
 *    fill outside / draw thin lines / parallax stacking).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • COASTLINE: a single jagged line dividing water (filled below)
 *    from sky (empty above). Resetting (r) gives a different but
 *    equally natural-looking coastline.
 *  • MOUNTAINS: 4 silhouettes visibly stacked, with the front-most
 *    rendering ON TOP of (covering parts of) the rear ones. Depth
 *    shading: rear is lighter (atmospheric perspective).
 *  • CITY: many sharp vertical "buildings" with a jagged top edge —
 *    not smooth like a coastline.
 *  • VALLEY: two visible lines top and bottom; the canyon between
 *    them is empty. Above the top line and below the bottom line
 *    is filled (rock).
 *  • WAVES: 6 thin lines at different vertical positions, all
 *    visible simultaneously. They shouldn't merge into a solid
 *    block — each must be a distinct line.
 *  • Morphing: leave any pattern running for ~30 s. The shapes
 *    should shift continuously, never freezing.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

    /* MD line resolution — must be 2^k + 1. 257 gives 256 segments
     * → 8 levels of subdivision, plenty of fractal detail. */
    LINE_POINTS       = 257,

    /* Maximum lines per pattern (WAVES needs 6, MOUNTAINS 4). */
    MAX_LINES         =   8,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* Morph rate base — fraction of the morph cycle per second. 0.10 →
 * full A→B over 10 seconds. */
#define MORPH_RATE_DEF      0.10f
#define MORPH_RATE_MIN      0.01f
#define MORPH_RATE_MAX      2.00f

#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/*
 * Pattern — five fractal-silhouette presets.
 *
 *   COASTLINE : 1 line, water below
 *   MOUNTAINS : 4 stacked ranges with depth shading
 *   CITY      : 1 high-roughness line, building fill below
 *   VALLEY    : 2 lines forming a canyon
 *   WAVES     : 6 thin lines stacked as ocean swell
 */
typedef enum {
    PATTERN_COASTLINE = 0,
    PATTERN_MOUNTAINS = 1,
    PATTERN_CITY      = 2,
    PATTERN_VALLEY    = 3,
    PATTERN_WAVES     = 4,
    N_PATTERNS        = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_COASTLINE: return "COASTLINE";
    case PATTERN_MOUNTAINS: return "MOUNTAINS";
    case PATTERN_CITY:      return "CITY     ";
    case PATTERN_VALLEY:    return "VALLEY   ";
    case PATTERN_WAVES:     return "WAVES    ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names. Each defines 4 palette colours used for
 * line + fill colouring, plus a flash accent.
 */
typedef struct {
    const char *name;
    short       band[4];
    short       flash;
} Theme;

#define N_THEMES 10

/*
 * Theme palettes for this file are all SHIFTED LIGHT — every band
 * value sits in the bright half of the 256-colour cube (≥ 39 / ≥ 64
 * for greens, etc.). Reason: the coastline patterns paint solid
 * vertical FILLS below their lines, and a dark colour like 17 (navy)
 * or 22 (dark green) blends into a typical dark terminal background,
 * making the silhouette nearly invisible. Pre-shifting every theme
 * upward keeps every band bright enough to read on dark, light, and
 * "transparent" terminals alike. This is a coastline-file-specific
 * tweak — other field files don't fill regions and so can use the
 * full intensity range.
 */
static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  39,  117,  220,  231 }, 226 },   /* sky → cyan → gold → white */
    { "MATRIX",  {  46,   82,  118,  154 }, 226 },   /* bright greens             */
    { "NOVA",    {  99,  165,  213,  219 }, 226 },   /* purple → pink (light)     */
    { "MONO",    { 245,  248,  251,  254 }, 226 },   /* light greyscale           */
    { "OCEAN",   {  39,   51,   81,  159 }, 226 },   /* sky → cyan (all light)    */
    { "FIRE",    { 166,  208,  220,  226 }, 196 },   /* orange → yellow           */
    { "EARTH",   { 137,  173,  215,  230 }, 226 },   /* light tans + cream        */
    { "FOREST",  {  64,  107,  144,  187 }, 226 },   /* mid-green → tan           */
    { "DESERT",  { 179,  215,  222,  230 }, 226 },   /* light sand                */
    { "ARCTIC",  { 117,  153,  195,  231 }, 226 },   /* light blue → white        */
};

/*
 * GlyphSet — 3-character density ramp (slim → fat). Same definitions
 * as flow_field_particles.c so both files feel familiar.
 */
typedef struct {
    const char *name;
    char low, mid, high;
} GlyphSet;

#define N_GLYPH_SETS 5

static const GlyphSet glyph_sets[N_GLYPH_SETS] = {
    { "SLIM",    '.', '\'', ':' },
    { "LIGHT",   '.', '*',  '+' },
    { "MEDIUM",  '.', '*',  '#' },
    { "HEAVY",   'o', 'O',  '@' },
    { "FAT",     '+', '#',  'M' },
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
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  md — midpoint displacement core                                    */
/* ===================================================================== */

static inline float rand_signed(void)
{
    return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}

/*
 * md_generate — fill `line[0..n-1]` with a midpoint-displacement
 * fractal connecting (0, left_y) to (n-1, right_y).
 *
 * n must be a power of 2 plus 1 (e.g. 257 = 2^8 + 1). amp_init is
 * the displacement magnitude at the FIRST level; it shrinks by
 * `roughness` at each subsequent level.
 *
 * Iterative implementation — easier to reason about than the
 * traditional recursive form, and avoids stack pressure for large n.
 */
static void md_generate(float *line, int n,
                        float left_y, float right_y,
                        float amp_init, float roughness)
{
    line[0]     = left_y;
    line[n - 1] = right_y;

    int   step = n - 1;
    float amp  = amp_init;
    while (step > 1) {
        int half = step / 2;
        for (int i = half; i < n - 1; i += step) {
            int   l   = i - half;
            int   r   = i + half;
            float avg = (line[l] + line[r]) * 0.5f;
            line[i]   = avg + rand_signed() * amp;
        }
        step  = half;
        amp  *= roughness;
    }
}

/*
 * line_lerp — populate `out[]` with the per-point linear interpolation
 * of `a[]` and `b[]` at parameter t ∈ [0, 1].
 */
static void line_lerp(const float *a, const float *b, float *out, int n, float t)
{
    float u = 1.0f - t;
    for (int i = 0; i < n; i++) {
        out[i] = u * a[i] + t * b[i];
    }
}

/*
 * line_sample — given the live line array (n_points entries) and a
 * render-x in [0, w − 1], return the interpolated y at that column.
 * Linear interpolation between adjacent points so the line scales to
 * any terminal width.
 */
static float line_sample(const float *line, int n_points, int w, int x)
{
    if (w <= 1) return line[0];
    float fx = (float)x * (float)(n_points - 1) / (float)(w - 1);
    int   xi = (int)fx;
    if (xi < 0) xi = 0;
    if (xi >= n_points - 1) return line[n_points - 1];
    float frac = fx - (float)xi;
    return line[xi] * (1.0f - frac) + line[xi + 1] * frac;
}

/* ===================================================================== */
/* §6  patterns — line setup + render                                     */
/* ===================================================================== */

/*
 * LineSpec — per-pattern configuration of one MD line:
 *   y_centre  — preferred mid-line y (in cell units)
 *   amp_init  — displacement magnitude at level 0
 *   roughness — amp decay factor per level
 *   left_dy   — endpoint y at x=0 relative to y_centre
 *   right_dy  — endpoint y at x=n-1 relative to y_centre
 *   pair_idx  — which palette band (0..3) to render this line/fill in
 */
typedef struct {
    float y_centre;
    float amp_init;
    float roughness;
    float left_dy, right_dy;
    int   pair_idx;
} LineSpec;

/*
 * pattern_specs[] — one entry per pattern, listing its lines and how
 * to lay them out. Filled by pattern_setup() since some specs depend
 * on the runtime map dimensions (h).
 */

/*
 * pattern_setup — populate LineSpec[] for the active pattern using
 * runtime dims. Returns line count.
 *
 * Pattern-specific design notes:
 *   COASTLINE : 1 line at 60 % h. Mild roughness for a natural coast.
 *   MOUNTAINS : 4 lines from the back (high y_centre = nearer top of
 *               screen, dim colour) to the front (low y_centre at the
 *               bottom, brightest colour). Each rear line has its
 *               peak slightly higher than the front to give parallax.
 *   CITY      : 1 line at 70 % h with HIGH roughness. Sharp jaggedy.
 *   VALLEY    : 2 lines — top at 25 % h, bottom at 75 % h. Symmetric.
 *   WAVES     : 6 lines stretching the height, low amplitude each so
 *               they look like layered ripples.
 */
static int pattern_setup(LineSpec out[], int max_n, Pattern p, int h)
{
    int n = 0;
    float fh = (float)h;
    switch (p) {

    case PATTERN_COASTLINE:
        out[n++] = (LineSpec){
            .y_centre = fh * 0.60f,
            .amp_init = fh * 0.20f, .roughness = 0.55f,
            .left_dy = 0.0f, .right_dy = 0.0f,
            .pair_idx = 0,
        };
        break;

    case PATTERN_MOUNTAINS: {
        /* 4 stacked ranges, back to front. y_centre INCREASES as we
         * go back-to-front (further → higher up the screen for a
         * receding-into-distance effect). pair_idx: rear most subtle. */
        float bands[4][2] = {
            /* y_centre_frac, amp_frac (per-line) */
            { 0.45f, 0.18f },   /* back-most */
            { 0.55f, 0.20f },
            { 0.65f, 0.22f },
            { 0.75f, 0.25f },   /* front-most */
        };
        int pair_order[4] = { 3, 2, 1, 0 };  /* rear = lightest, front = darkest */
        for (int i = 0; i < 4 && n < max_n; i++) {
            out[n++] = (LineSpec){
                .y_centre  = fh * bands[i][0],
                .amp_init  = fh * bands[i][1],
                .roughness = 0.58f,
                .left_dy   = 0.0f,
                .right_dy  = 0.0f,
                .pair_idx  = pair_order[i],
            };
        }
        break;
    }

    case PATTERN_CITY:
        out[n++] = (LineSpec){
            .y_centre = fh * 0.70f,
            .amp_init = fh * 0.30f, .roughness = 0.70f,   /* spiky! */
            .left_dy = 0.0f, .right_dy = 0.0f,
            .pair_idx = 1,
        };
        break;

    case PATTERN_VALLEY:
        out[n++] = (LineSpec){      /* top of canyon  */
            .y_centre = fh * 0.25f,
            .amp_init = fh * 0.15f, .roughness = 0.55f,
            .left_dy = 0.0f, .right_dy = 0.0f,
            .pair_idx = 2,
        };
        out[n++] = (LineSpec){      /* bottom of canyon */
            .y_centre = fh * 0.75f,
            .amp_init = fh * 0.15f, .roughness = 0.55f,
            .left_dy = 0.0f, .right_dy = 0.0f,
            .pair_idx = 0,
        };
        break;

    case PATTERN_WAVES:
        /* 6 thin lines stretched across the height. */
        for (int i = 0; i < 6 && n < max_n; i++) {
            float t = (i + 1) / 7.0f;     /* skip y=0 and y=h */
            out[n++] = (LineSpec){
                .y_centre  = fh * t,
                .amp_init  = fh * 0.04f,    /* low amplitude */
                .roughness = 0.55f,
                .left_dy   = 0.0f,
                .right_dy  = 0.0f,
                .pair_idx  = i & 3,         /* cycle palette */
            };
        }
        break;

    default:
        break;
    }
    return n;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * Field — the "live" scene state for this file (no particles).
 *
 *   line_a / line_b : start and target shapes for each MD line
 *   line_now        : per-frame interpolated shape used for rendering
 *   spec[]          : per-line configuration (from pattern_setup)
 *   morph_t         : 0..1 progress from A to B
 *   morph_rate      : how fast morph_t advances per second
 */
typedef struct {
    int      w, h;

    int      n_lines;
    LineSpec spec [MAX_LINES];
    float    line_a   [MAX_LINES][LINE_POINTS];
    float    line_b   [MAX_LINES][LINE_POINTS];
    float    line_now [MAX_LINES][LINE_POINTS];

    float    morph_t;
    float    morph_rate;

    float    supernova_glow_t;
} Field;

/*
 * field_regenerate_b — generate fresh "B" lines for every layer.
 * Called when the morph rolls over (A ← B, regenerate B) and on
 * scene reset.
 */
static void field_regenerate_b(Field *f)
{
    for (int i = 0; i < f->n_lines; i++) {
        const LineSpec *s = &f->spec[i];
        float left  = s->y_centre + s->left_dy
                    + rand_signed() * s->amp_init * 0.5f;
        float right = s->y_centre + s->right_dy
                    + rand_signed() * s->amp_init * 0.5f;
        md_generate(f->line_b[i], LINE_POINTS,
                    left, right,
                    s->amp_init, s->roughness);
    }
}

static void field_reset(Field *f, int w, int h, Pattern pat)
{
    f->w = w;
    f->h = h;
    f->morph_t = 0.0f;
    f->supernova_glow_t = 1.0f;
    f->n_lines = pattern_setup(f->spec, MAX_LINES, pat, h);

    /* Generate A lines, then regenerate B lines as separate fractals. */
    for (int i = 0; i < f->n_lines; i++) {
        const LineSpec *s = &f->spec[i];
        float left_a  = s->y_centre + s->left_dy
                      + rand_signed() * s->amp_init * 0.5f;
        float right_a = s->y_centre + s->right_dy
                      + rand_signed() * s->amp_init * 0.5f;
        md_generate(f->line_a[i], LINE_POINTS,
                    left_a, right_a,
                    s->amp_init, s->roughness);
    }
    field_regenerate_b(f);

    /* Initialise live buffer to A. */
    for (int i = 0; i < f->n_lines; i++) {
        memcpy(f->line_now[i], f->line_a[i], sizeof(f->line_now[i]));
    }
}

typedef struct {
    Field   F;
    bool    paused;
    int     current_theme;
    int     current_glyph_set;
    Pattern current_pattern;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    field_reset(&s->F, mw, mh, s->current_pattern);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused             = false;
    s->current_theme      = 0;
    s->current_glyph_set  = 2;
    s->current_pattern    = PATTERN_COASTLINE;
    s->F.morph_rate       = MORPH_RATE_DEF;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Field *f = &s->F;

    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    /* Advance morph. */
    f->morph_t += f->morph_rate * dt;
    if (f->morph_t >= 1.0f) {
        /* A ← B; regenerate B from scratch. */
        memcpy(f->line_a, f->line_b, sizeof(f->line_a));
        field_regenerate_b(f);
        f->morph_t = 0.0f;
    }

    /* Compute live (interpolated) line for each layer. */
    for (int i = 0; i < f->n_lines; i++) {
        line_lerp(f->line_a[i], f->line_b[i],
                  f->line_now[i], LINE_POINTS, f->morph_t);
    }
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

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

/*
 * draw_fill_below_line — for each on-screen column, fill from the
 * line's y down to the bottom of the map with the given pair / glyph.
 * Used by COASTLINE / CITY / MOUNTAINS.
 */
static void draw_fill_below_line(const float *line, int n_points,
                                 int gx0, int gy0, int map_w, int map_h,
                                 int cols, int rows, int pair, char glyph)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < map_w; x++) {
        float ly = line_sample(line, n_points, map_w, x);
        int y_start = (int)ly;
        if (y_start < 0) y_start = 0;
        if (y_start >= map_h) continue;
        for (int y = y_start; y < map_h; y++) {
            int sx = gx0 + x;
            int sy = gy0 + y;
            if (sx < 0 || sx >= cols) continue;
            if (sy < 0 || sy >= rows) continue;
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        }
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/*
 * draw_line_only — render only the line itself as a thin outline
 * (no fill below or above). Used by WAVES.
 */
static void draw_line_only(const float *line, int n_points,
                           int gx0, int gy0, int map_w, int map_h,
                           int cols, int rows, int pair, char glyph)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < map_w; x++) {
        float ly = line_sample(line, n_points, map_w, x);
        int y = (int)ly;
        if (y < 0 || y >= map_h) continue;
        int sx = gx0 + x;
        int sy = gy0 + y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    }
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Field *f = &s->F;

    int gx0 = (cols - f->w) / 2;
    int gy0 = ((rows - 3) - f->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    int gs_idx = s->current_glyph_set;
    if (gs_idx < 0 || gs_idx >= N_GLYPH_SETS) gs_idx = 0;
    const GlyphSet *gs = &glyph_sets[gs_idx];

    /* Pattern-specific render. */
    switch (s->current_pattern) {

    case PATTERN_COASTLINE:
        /* Water below = palette[0] bg colour, glyph = density-low.
         * Coast line itself drawn on top with mid glyph. */
        draw_fill_below_line(f->line_now[0], LINE_POINTS,
                             gx0, gy0, f->w, f->h, cols, rows,
                             PAIR_BAND_BASE + f->spec[0].pair_idx, gs->low);
        draw_line_only(f->line_now[0], LINE_POINTS,
                       gx0, gy0, f->w, f->h, cols, rows,
                       PAIR_BAND_BASE + 2, gs->high);
        break;

    case PATTERN_MOUNTAINS:
        /* Render back-to-front. Each layer fills below its line, so
         * front layers naturally cover the silhouettes of rear ones.
         * pair_idx is set per LineSpec so colour tracks depth. */
        for (int i = 0; i < f->n_lines; i++) {
            char g = (i == f->n_lines - 1) ? gs->high
                   : (i >= f->n_lines / 2) ? gs->mid
                                           : gs->low;
            draw_fill_below_line(f->line_now[i], LINE_POINTS,
                                 gx0, gy0, f->w, f->h, cols, rows,
                                 PAIR_BAND_BASE + f->spec[i].pair_idx, g);
        }
        break;

    case PATTERN_CITY:
        /* City = high-roughness silhouette; fill below = building mass. */
        draw_fill_below_line(f->line_now[0], LINE_POINTS,
                             gx0, gy0, f->w, f->h, cols, rows,
                             PAIR_BAND_BASE + f->spec[0].pair_idx, gs->high);
        /* Outline of the skyline in a contrasting colour. */
        draw_line_only(f->line_now[0], LINE_POINTS,
                       gx0, gy0, f->w, f->h, cols, rows,
                       PAIR_BAND_BASE + 3, gs->mid);
        break;

    case PATTERN_VALLEY: {
        /* Top line: rock fill ABOVE it (i.e. rows 0..line). */
        const float *top = f->line_now[0];
        const float *bot = f->line_now[1];
        attron(COLOR_PAIR(PAIR_BAND_BASE + f->spec[0].pair_idx) | A_BOLD);
        for (int x = 0; x < f->w; x++) {
            float ly = line_sample(top, LINE_POINTS, f->w, x);
            int y_end = (int)ly;
            if (y_end < 0) continue;
            if (y_end >= f->h) y_end = f->h - 1;
            for (int y = 0; y <= y_end; y++) {
                int sx = gx0 + x;
                int sy = gy0 + y;
                if (sx < 0 || sx >= cols) continue;
                if (sy < 0 || sy >= rows) continue;
                mvaddch(sy, sx, (chtype)(unsigned char)gs->mid);
            }
        }
        attroff(COLOR_PAIR(PAIR_BAND_BASE + f->spec[0].pair_idx) | A_BOLD);

        /* Bottom line: rock fill BELOW it (rows line..h). */
        attron(COLOR_PAIR(PAIR_BAND_BASE + f->spec[1].pair_idx) | A_BOLD);
        for (int x = 0; x < f->w; x++) {
            float ly = line_sample(bot, LINE_POINTS, f->w, x);
            int y_start = (int)ly;
            if (y_start < 0) y_start = 0;
            if (y_start >= f->h) continue;
            for (int y = y_start; y < f->h; y++) {
                int sx = gx0 + x;
                int sy = gy0 + y;
                if (sx < 0 || sx >= cols) continue;
                if (sy < 0 || sy >= rows) continue;
                mvaddch(sy, sx, (chtype)(unsigned char)gs->mid);
            }
        }
        attroff(COLOR_PAIR(PAIR_BAND_BASE + f->spec[1].pair_idx) | A_BOLD);
        break;
    }

    case PATTERN_WAVES:
        /* Each wave is just a thin line. Different palette colours per
         * line so layers are distinguishable. */
        for (int i = 0; i < f->n_lines; i++) {
            draw_line_only(f->line_now[i], LINE_POINTS,
                           gx0, gy0, f->w, f->h, cols, rows,
                           PAIR_BAND_BASE + f->spec[i].pair_idx,
                           gs->mid);
        }
        break;

    default:
        break;
    }

    /* Sparse supernova flash — paint over everything when active. */
    if (f->supernova_glow_t > GLOW_THRESHOLD) {
        attron(COLOR_PAIR(PAIR_SUPERNOVA) | A_BOLD);
        for (int y = 0; y < f->h; y++) {
            int sy = gy0 + y;
            if (sy < 0 || sy >= rows) continue;
            for (int x = 0; x < f->w; x++) {
                int sx = gx0 + x;
                if (sx < 0 || sx >= cols) continue;
                if (((x ^ y) & 7) == 0) mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_SUPERNOVA) | A_BOLD);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Field *f = &s->F;
    const char *state_str = s->paused
                          ? "PAUSED   "
                          : pattern_name(s->current_pattern);

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  morph:%4.2f  rate:%4.2f ",
             fps, sim_fps, state_str,
             (double)f->morph_t, (double)f->morph_rate);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " MIDPOINT-DISPLACEMENT FRACTAL ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 20;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_BAND_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  lines:%d  pts:%d  map:%dx%d ",
             f->n_lines, LINE_POINTS, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Glyph-set on the right of row 1. */
    int gs_idx = s->current_glyph_set;
    if (gs_idx < 0 || gs_idx >= N_GLYPH_SETS) gs_idx = 0;
    const GlyphSet *gs = &glyph_sets[gs_idx];
    char gbuf[32];
    snprintf(gbuf, sizeof gbuf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int grx = sc->cols - (int)strlen(gbuf);
    if (grx < 0) grx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, grx, "%s", gbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " fractal silhouette | n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:morph-rate  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

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

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        s->F.morph_rate *= 2.0f;
        if (s->F.morph_rate > MORPH_RATE_MAX) s->F.morph_rate = MORPH_RATE_MAX;
        break;
    case '-':
        s->F.morph_rate *= 0.5f;
        if (s->F.morph_rate < MORPH_RATE_MIN) s->F.morph_rate = MORPH_RATE_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_reset(s, app->map_w, app->map_h);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_reset(s, app->map_w, app->map_h);
        break;

    case 'g':
        s->current_glyph_set = (s->current_glyph_set + 1) % N_GLYPH_SETS;
        break;
    case 'G':
        s->current_glyph_set = (s->current_glyph_set + N_GLYPH_SETS - 1) % N_GLYPH_SETS;
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
