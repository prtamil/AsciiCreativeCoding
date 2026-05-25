/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * bifurcation.c — Logistic Map Bifurcation Diagram (density-rendered)
 *
 *   x_{n+1} = r · x_n · (1 − x_n)
 *
 * For each screen column a distinct r value is chosen.  WARMUP transient
 * iterations are discarded; the next PLOT iterations are accumulated into
 * a per-cell DENSITY BUFFER.  Each cell's count is mapped through a log
 * curve to one of N_TIERS brightness levels; each tier maps to a colour
 * from the active cosine-gradient theme.
 *
 * Why density rendering vs single-dot plotting:
 *   - period-N attractors land on N rows per column, each hit ~PLOT/N
 *     times -- they become BRIGHT thick lines in the heat map
 *   - chaos spreads iterations thinly across many cells, so each cell
 *     is a faint dot -- the result is a smooth fuzz cloud
 *   - period-3, period-5 etc windows inside chaos pop out as bright
 *     vertical bands
 *   - the iconic textbook appearance falls out for free
 *
 * Sub-pixel r-sampling: each visible column is split into SUBPIXEL
 * fractional r-values, all stamping into the same column.  This
 * anti-aliases the attractor curves and gives smoother continuous
 * regions at modest column counts.
 *
 * The view auto-zooms toward the Feigenbaum accumulation point
 * (r∞ ≈ 3.5699) so the self-similar fractal structure gradually fills
 * the screen.
 *
 * Keys:
 *   q / Q / ESC     quit
 *   space           pause / resume auto-zoom
 *   r / R           reset view to r ∈ [2.50, 4.00]
 *   t / T           next / prev colour theme (10 themes)
 *   ← / →           pan
 *   + / =           zoom in
 *   - / _           zoom out
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra procedural/chaos/bifurcation.c \
 *       -o bifurcation -lncurses -lm
 *
 * Sections:
 *   §1 config   -- sizes, sampling, named constants, colour-pair allocation
 *   §2 clock    -- monotonic ns clock + sleep
 *   §3 color    -- RGB, cosine_palette, ANSI quantizers, Theme + theme_apply
 *   §4 diagram  -- Density buffer, log-tier mapping, build + render
 *   §5 scene    -- ViewWindow + Scene + simulation tick + render pipeline
 *   §6 hud      -- top data bar + bottom action bar helpers
 *   §7 screen   -- Screen + cached terminal geometry
 *   §8 app      -- App + signals + action keys + main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Logistic map iteration with parameter scanning and
 *                 density accumulation.  Per visible column (and SUBPIXEL
 *                 r-values per column), iterate the logistic map for
 *                 WARMUP transient steps, then accumulate PLOT iterations
 *                 into a per-cell density buffer.  Final image: density
 *                 mapped through log curve to N_TIERS brightness levels
 *                 coloured by the active cosine-gradient theme.
 *
 * Math          : The logistic map models population dynamics.
 *                  r < 3      : converges to fixed point  (period 1)
 *                  r ≈ 3.0    : period-2 bifurcation
 *                  r ≈ 3.449  : period-4 bifurcation
 *                  r ≈ 3.544  : period-8
 *                  r ≈ 3.5699 : accumulation point (onset of chaos)
 *                  r > 4      : trajectories diverge
 *                 Successive bifurcations r_n satisfy
 *                  (r_{n+1} − r_n) / (r_{n+2} − r_{n+1}) → δ ≈ 4.6692
 *                 where δ is Feigenbaum's constant -- universal across
 *                 all unimodal maps, not just the logistic.
 *
 * Rendering     : Density rendering is the discrete-cell analogue of a
 *                 graphics accumulation buffer.  Log-scale tier mapping
 *                  tier = round( log(1+d) / log(1+peak) · (N_TIERS-1) )
 *                 keeps faint chaos cells visible without saturating
 *                 the bright periodic-attractor lines.
 *
 * Palette       : Each theme is a cosine gradient (Iñigo Quilez):
 *                  color(t) = a + b · cos(2π · (c·t + d))
 *                 Sampled at t = i/(N_TIERS-1) for the N_TIERS tier stops,
 *                 quantised to nearest ANSI 256-cube colour.
 * ─────────────────────────────────────────────────────────────────────── */

/* -- REFERENCES ---------------------------------------------------------  *
 *
 * Logistic map (the original chaos paper):
 *   [1] May, R. M. (1976). "Simple mathematical models with very
 *       complicated dynamics."  Nature 261, 459-467.
 *       -- the seminal paper that brought the logistic map to wide
 *          attention.  Showed that simple deterministic maps can
 *          exhibit periodic, periodic-doubling, and chaotic behaviour
 *          depending on a single parameter -- a foundational result
 *          for nonlinear dynamics in biology and physics.
 *
 * Feigenbaum's universality:
 *   [2] Feigenbaum, M. J. (1978). "Quantitative universality for a
 *       class of nonlinear transformations."  J. Statistical Physics
 *       19(1), 25-52.
 *       -- discovered the universal scaling constant δ ≈ 4.6692
 *          governing period-doubling cascades.  The constant appears
 *          across ALL unimodal maps (logistic, sine-circle, ...) and
 *          even in real physical systems (Rayleigh-Bénard convection,
 *          driven pendulums) -- the accumulation point in our diagram
 *          (r∞ ≈ 3.5699) is governed by exactly this constant.
 *
 *   [3] Cvitanović, P. (ed.) (1989). "Universality in Chaos" (2nd ed.).
 *       Adam Hilger.
 *       -- anthology collecting Feigenbaum's papers plus the related
 *          renormalisation-group derivations.  Useful background if
 *          [2] feels terse.
 *
 * Textbook treatments:
 *   [4] Strogatz, S. H. (2015). "Nonlinear Dynamics and Chaos" (2nd ed.).
 *       Westview Press.
 *       -- chapter 10 is the canonical pedagogical introduction to the
 *          logistic map: orbit diagrams, the period-3 window, Liapunov
 *          exponents, and the renormalisation argument behind δ.  Best
 *          single starting point.
 *
 *   [5] Devaney, R. L. (2003). "An Introduction to Chaotic Dynamical
 *       Systems" (2nd ed.).  Westview Press.
 *       -- more rigorous treatment; develops the symbolic dynamics that
 *          explain the structure of the periodic windows (where
 *          period-3, period-5 etc bands appear inside the chaos).
 *
 * Rendering:
 *   [6] Akenine-Möller, T., Haines, E., Hoffman, N., et al. (2018).
 *       "Real-Time Rendering" (4th ed.). CRC Press.
 *       -- §23 on accumulation buffer techniques.  Our per-cell
 *          density histogram is the discrete-cell analogue: each
 *          column accumulates PLOT trajectory samples; bright cells =
 *          frequently-visited (attractor lines), dim cells = rarely-
 *          visited (chaos floor).
 *
 *   [7] Quilez, I. (2015). "Palettes."
 *       https://iquilezles.org/articles/palettes/
 *       -- the cosine-gradient palette technique used in §3 theme
 *          generation.  color(t) = a + b·cos(2π·(c·t + d)) per channel
 *          gives smooth procedural palettes from just four RGB triples,
 *          which is why each theme is so compact and yet so flexible.
 *
 * ----------------------------------------------------------------------- */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

/* Iteration counts (per sub-pixel r-sample).  Higher PLOT = brighter
 * envelope lines + smoother chaos; cost is linear in PLOT. */
#define WARMUP             500   /* transient iterations discarded         */
#define PLOT              1500   /* iterations stamped into density buffer */
#define SUBPIXEL             4   /* r-values sampled per visible column    */

/* Frame pacing. */
#define RENDER_NS  (1000000000LL / 30)

/* r-window control. */
#define R_DOMAIN_LO        0.0f   /* logistic-map valid domain lo (exclusive) */
#define R_DOMAIN_HI        4.0f   /* logistic-map valid domain hi (inclusive) */
#define R_DEFAULT_LO       2.50f  /* startup / reset window low end           */
#define R_DEFAULT_HI       4.00f  /* startup / reset window high end          */
#define ZOOM_FACTOR        1.25f
#define PAN_FRAC           0.12f  /* pan moves this fraction of width        */
#define AUTO_ZOOM_SHRINK   0.997f /* window scale per frame (≈ 0.3%)         */
#define ZOOM_MIN_WIDTH     0.002f /* auto-zoom stops below this              */

/* Feigenbaum accumulation point: r∞ where period-doubling cascades to chaos. */
#define FEIGENBAUM_R   3.5699456718695f

/* Static density-buffer capacity.  Real terminal size is clamped to these. */
#define MAX_ROWS   128
#define MAX_COLS   320

/* HUD layout -- top band carries data, bottom band carries action keys.
 * Plot area is rows [HUD_TOP_ROWS, rows - HUD_BOTTOM_ROWS). */
#define HUD_TOP_ROWS                2
#define HUD_BOTTOM_ROWS             1
#define HUD_DATA_COL               16   /* col where the middle data segment starts */

/* r-axis tick labels on HUD row 1. */
#define N_TICKS                     5   /* labels at 5 evenly spaced columns      */
#define TICK_LABEL_RIGHT_MARGIN     6   /* chars reserved on right (label width)  */

/* Brightness tiers.  N_TIERS density levels paired with N_TIERS theme
 * palette stops (cold tier 0 → hot tier N_TIERS-1). */
#define N_TIERS            8
#define BRIGHT_TIER        (N_TIERS - 1)

/*
 * Colour-pair allocation:
 *   1 .. N_TIERS         density tiers (theme-controlled by theme_apply)
 *   N_TIERS + 1          HUD data    (yellow, theme-independent)
 *   N_TIERS + 2          HUD title / hint  (cyan, theme-independent)
 */
#define CP_TIER_0          1
#define CP_HUD             (N_TIERS + 1)
#define CP_HINT            (N_TIERS + 2)

/* HUD colour codes (256-mode). */
#define HUD_DATA_YELLOW_256   226
#define HUD_TITLE_CYAN_256     51

#define KEY_ESC               27

/* Math literals named for their geometric meaning. */
#define TAU               (2.0f * (float)M_PI)

/* ANSI 256-colour cube layout (RGB sub-cube 16..231):
 *   pair_code = ANSI_CUBE_BASE + R·ANSI_CUBE_R_STRIDE
 *                              + G·ANSI_CUBE_G_STRIDE
 *                              + B,    R,G,B ∈ [0, ANSI_CUBE_MAX_STEP] */
#define ANSI_CUBE_BASE       16
#define ANSI_CUBE_R_STRIDE   36
#define ANSI_CUBE_G_STRIDE    6
#define ANSI_CUBE_MAX_STEP    5

/* Per-channel threshold for the 8-colour fallback. */
#define RGB_BIT_THRESHOLD   0.5f

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/* ---- RGB + cosine palette helpers ------------------------------------ */

/*
 * RGB -- normalised colour triple in linear [0, 1]^3.
 *
 * WHY normalised floats (not 0..255 ints):
 *   The cosine-palette formula (Quilez [7]) is naturally float-valued;
 *   coefficients (a, b, c, d) live in [0, 1] for visual coherence.
 *   Keeping the working representation in floats avoids round-trips
 *   through integer space until the very last quantization step
 *   (rgb_to_ansi256 / rgb_to_ansi8).
 *
 * Colour space:
 *   Values are TREATED as linear RGB for blending arithmetic.  Real
 *   ANSI terminals interpret colour codes through sRGB gamma; at the
 *   6-step cube's resolution (~16% per step) the gamma error sits well
 *   below the quantization noise, so we skip the conversion.
 *
 * Read-only after cosine_palette returns -- no RGB mutator anywhere
 * in the program.
 */
typedef struct {
    float r;    /* red   channel, [0, 1] */
    float g;    /* green channel, [0, 1] */
    float b;    /* blue  channel, [0, 1] */
} RGB;

/* Saturate one channel into [0, 1]. */
static inline float clamp01(float v)
{
    if (v < 0.0f) return 0.0f;
    if (v > 1.0f) return 1.0f;
    return v;
}

/* Round-half-up to integer for non-negative v.  Used by the three
 * quantisation paths: ANSI-cube channel, density tier, x→row. */
static inline int round_half_up(float v)
{
    return (int)(v + 0.5f);
}

/*
 * cosine_palette -- procedural gradient sampler (Iñigo Quilez 2015).
 *
 *   color(t) = a + b · cos(TAU · (c · t + d))     evaluated per channel
 *
 *   a   DC offset       (channel midpoint)
 *   b   amplitude       (max excursion above/below a)
 *   c   frequency       (cycles per t ∈ [0, 1]; 0.5 = monotonic sweep)
 *   d   phase           (cosine offset; rotates the gradient)
 *
 * Reference: https://iquilezles.org/articles/palettes/
 */
static RGB cosine_palette(const RGB *a, const RGB *b,
                          const RGB *c, const RGB *d, float t)
{
    RGB out;
    out.r = clamp01(a->r + b->r * cosf(TAU * (c->r * t + d->r)));
    out.g = clamp01(a->g + b->g * cosf(TAU * (c->g * t + d->g)));
    out.b = clamp01(a->b + b->b * cosf(TAU * (c->b * t + d->b)));
    return out;
}

/* Quantise one channel to the 6-step ANSI cube. */
static int rgb_channel_to_cube_step(float v)
{
    int idx = round_half_up(v * (float)ANSI_CUBE_MAX_STEP);
    if (idx < 0)                   idx = 0;
    if (idx > ANSI_CUBE_MAX_STEP)  idx = ANSI_CUBE_MAX_STEP;
    return idx;
}

/* (R, G, B) ∈ [0,1]^3  →  nearest xterm-256 cube colour code. */
static int rgb_to_ansi256(RGB c)
{
    int ri = rgb_channel_to_cube_step(c.r);
    int gi = rgb_channel_to_cube_step(c.g);
    int bi = rgb_channel_to_cube_step(c.b);
    return ANSI_CUBE_BASE
         + ANSI_CUBE_R_STRIDE * ri
         + ANSI_CUBE_G_STRIDE * gi
         + bi;
}

/* 8-colour fallback: threshold each channel at RGB_BIT_THRESHOLD. */
static int rgb_to_ansi8(RGB c)
{
    int r = (c.r > RGB_BIT_THRESHOLD);
    int g = (c.g > RGB_BIT_THRESHOLD);
    int b = (c.b > RGB_BIT_THRESHOLD);
    if (!r && !g && !b) return COLOR_BLACK;
    if ( r && !g && !b) return COLOR_RED;
    if (!r &&  g && !b) return COLOR_GREEN;
    if (!r && !g &&  b) return COLOR_BLUE;
    if ( r &&  g && !b) return COLOR_YELLOW;
    if ( r && !g &&  b) return COLOR_MAGENTA;
    if (!r &&  g &&  b) return COLOR_CYAN;
    return COLOR_WHITE;
}

/* ---- Theme palette table -------------------------------------------- */

/*
 * Theme -- a named cosine-gradient palette (Quilez 2015 [7]).
 *
 * Each theme is FOUR RGB triples (a, b, c, d) that fully specify a
 * procedural gradient:
 *
 *     color(t) = a + b · cos(TAU · (c · t + d))
 *
 * evaluated independently per channel.  Sampling at t ∈ [0, 1] gives
 * the N_TIERS palette stops; theme_apply writes them into ncurses
 * pairs CP_TIER_0..CP_TIER_0+N_TIERS-1.
 *
 * WHY cosine-gradient parameters (and not a fixed N_TIERS-stop table):
 *
 *   1. Compact -- 12 floats per theme instead of 2 × N_TIERS·sizeof(int)
 *      (one int per stop for 256-mode plus one per stop for 8-mode).
 *      The floats DESCRIBE the gradient instead of just listing samples.
 *
 *   2. Resolution-independent -- if we bump N_TIERS from 8 to 16 the
 *      same theme generates a smoother 16-stop palette without a
 *      table edit.
 *
 *   3. Self-documenting -- reading the parameters tells you the gradient
 *      shape: c = 0.5 means "monotonic sweep", c = 1.0 means "full hue
 *      cycle wraps back", d shifts mark phase offsets.
 *
 *   4. Composable -- tweak d.g += 0.1 to rotate the green channel's
 *      hue by a known amount.  Designing a new theme is easier than
 *      picking 8 ANSI codes by trial-and-error.
 *
 * Worked sample -- "Matrix" theme at tier 0 (t = 0):
 *      r = 0.00 + 0.00·cos(TAU·0)         = 0.00
 *      g = 0.55 + 0.45·cos(TAU·0.5)       = 0.55 − 0.45 = 0.10
 *      b = 0.08 + 0.08·cos(TAU·0.5)       = 0.00
 *   → faint dark green.
 * At tier 7 (t = 1):
 *      g = 0.55 + 0.45·cos(TAU·1)         = 1.00
 *      b = 0.08 + 0.08·cos(TAU·1)         = 0.16
 *   → bright green with a slight cyan tint.
 *
 * Theme switching ('t' / 'T') just recomputes the N_TIERS stops and
 * rewrites pairs CP_TIER_0..CP_TIER_0+N_TIERS-1 via init_pair.  Pairs
 * above N_TIERS (HUD title + data) are left untouched so the chrome
 * stays consistent across themes.
 *
 * Reference:  Quilez 2015 [7] for the palette formula.
 */
typedef struct {
    const char *name;  /* HUD-displayable name ("Matrix", "Fire", ...) */
    RGB         a;     /* DC offset    (mid value the channel oscillates around) */
    RGB         b;     /* amplitude    (max excursion above/below a).
                        * Keep a + b ≤ 1 and a − b ≥ 0 to avoid clipping;
                        * cosine_palette clamps after the fact regardless. */
    RGB         c;     /* frequency    (cycles per t ∈ [0, 1]).
                        * c = 0.5 → monotonic sweep; c = 1.0 → full cycle. */
    RGB         d;     /* phase        (cosine offset; rotates the gradient).
                        * d = 0.5 makes the cosine start at trough (cos(π)). */
} Theme;

#define N_THEMES 10

static const Theme k_themes[N_THEMES] = {
    /*  name           a (offset)             b (amplitude)            c (frequency)            d (phase)              */
    { "Matrix",     {0.00f, 0.55f, 0.08f}, {0.00f, 0.45f, 0.08f}, {0.00f, 0.50f, 0.50f}, {0.00f, 0.50f, 0.50f} },
    { "Sunset",     {0.70f, 0.50f, 0.30f}, {0.20f, 0.50f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Ocean",      {0.20f, 0.45f, 0.65f}, {0.20f, 0.40f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.30f, 0.50f} },
    { "Plasma",     {0.55f, 0.40f, 0.55f}, {0.45f, 0.40f, 0.45f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
    { "Fire",       {0.55f, 0.30f, 0.15f}, {0.45f, 0.50f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.30f} },
    { "Mint",       {0.20f, 0.55f, 0.50f}, {0.20f, 0.40f, 0.40f}, {0.50f, 0.50f, 0.50f}, {0.30f, 0.50f, 0.50f} },
    { "Lavender",   {0.55f, 0.45f, 0.65f}, {0.40f, 0.35f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Aurora",     {0.45f, 0.55f, 0.55f}, {0.40f, 0.40f, 0.40f}, {1.00f, 1.00f, 0.50f}, {0.00f, 0.50f, 0.75f} },
    { "Sepia",      {0.45f, 0.35f, 0.25f}, {0.40f, 0.35f, 0.20f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.05f, 0.10f} },
    { "Rainbow",    {0.50f, 0.50f, 0.50f}, {0.50f, 0.50f, 0.50f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
};

static inline int theme_next(int cur) { return (cur + 1) % N_THEMES; }
static inline int theme_prev(int cur) { return (cur + N_THEMES - 1) % N_THEMES; }

/* Position of tier i along the gradient axis t ∈ [0, 1]. */
static inline float tier_to_gradient_t(int i)
{
    return (float)i / (float)(N_TIERS - 1);
}

/*
 * theme_apply -- compute N_TIERS palette stops for theme_idx and load
 * them into ncurses pairs CP_TIER_0..CP_TIER_0+N_TIERS-1.
 */
static void theme_apply(int theme_idx)
{
    const Theme *th = &k_themes[theme_idx];
    bool have_256   = (COLORS >= 256);
    for (int i = 0; i < N_TIERS; i++) {
        float t   = tier_to_gradient_t(i);
        RGB   rgb = cosine_palette(&th->a, &th->b, &th->c, &th->d, t);
        int   code = have_256 ? rgb_to_ansi256(rgb) : rgb_to_ansi8(rgb);
        init_pair(CP_TIER_0 + i, code, -1);
    }
}

/* One-time palette setup: theme-independent HUD chrome first, then theme 0. */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_HUD,  HUD_DATA_YELLOW_256, -1);
        init_pair(CP_HINT, HUD_TITLE_CYAN_256,  -1);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, -1);
        init_pair(CP_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  diagram                                                            */
/* ===================================================================== */

/* ---- Density buffer ---------------------------------------------------- */

/*
 * Density -- 2-D histogram of (column, row) attractor visits.
 *
 * WHY this exists (the rendering technique it enables):
 *   Naive per-point plotting would draw a single '.' for every iterate
 *   and overdraw collisions, discarding the cell-visit-frequency
 *   information that GIVES the bifurcation diagram its iconic
 *   appearance.  Density rendering replaces that with accumulation:
 *
 *     stamp pass:   for each (r, column), iterate the logistic map
 *                   WARMUP+PLOT times; for every iterate, increment
 *                   density[row][col]
 *     render pass:  map each cell's count through a log curve to one of
 *                   N_TIERS brightness levels; brighter cells = more
 *                   frequently visited
 *
 *   Periodic attractors (period-N) land on N rows × cols cells per
 *   column, hit ~PLOT/N times each → high count → bright thick lines.
 *   Chaos spreads PLOT iterates across ~100 rows in a column → ~5-15
 *   per cell → faint fuzz.  Periodicity windows (period-3 etc) emerge
 *   inside the chaos band as bright vertical streaks.
 *
 * Invariants:
 *   I1.  cells[r][c] = number of distinct logistic-map iterates whose
 *        x-value landed in row r of column c during the most recent
 *        diagram_build_density pass.  Cleared to 0 at the start of
 *        every build.
 *   I2.  peak == max(cells[r][c]) over all (r, c) in the stamped
 *        region.  Maintained INCREMENTALLY by density_stamp so the
 *        renderer does not need a second pass to find the max.
 *   I3.  All writes are clipped to the drawing band:
 *           HUD_TOP_ROWS ≤ r < rows - HUD_BOTTOM_ROWS
 *           0 ≤ c < cols
 *
 * Memory:
 *   MAX_ROWS · MAX_COLS · sizeof(int) ≈ 160 KB.  Static -- lives
 *   inside Scene (and Scene inside g_app), so no heap allocation, no
 *   per-frame zeroing of allocated memory.
 *
 * Reference:  Akenine-Möller [6] §23 on accumulation buffers; this is
 *   the discrete-cell analogue.  Same idea used by geometry/maurer_rose
 *   for parametric-curve density; here it integrates chaotic-map
 *   trajectories rather than a polyline.
 *
 * Fields:
 *   cells   per-cell visit counter, bounded above by SUBPIXEL·PLOT
 *           (worst case where all sub-pixel samples for the same column
 *           land on the same row -- happens only for fixed-point r).
 *   peak    running max of cells; the normalisation denominator used
 *           by density_to_tier.
 */
typedef struct {
    int cells[MAX_ROWS][MAX_COLS];  /* per-cell visit counter, 0 = unvisited */
    int peak;                       /* max(cells); maintained by density_stamp */
} Density;

static void density_clear(Density *d)
{
    memset(d->cells, 0, sizeof d->cells);
    d->peak = 0;
}

/* Increment one cell and update running peak. */
static inline void density_stamp(Density *d, int row, int col)
{
    int v = ++d->cells[row][col];
    if (v > d->peak) d->peak = v;
}

/*
 * density_to_tier -- map raw count to brightness tier via log curve.
 *
 *   tier = round( log(1 + d) / log(1 + peak) · (N_TIERS - 1) )
 *
 * Returns -1 for empty cells (caller should skip).  The log curve
 * keeps both bright periodic lines AND faint chaos fuzz visible in
 * the same frame -- linear scaling either saturates or loses the floor.
 */
static inline int density_to_tier(int d, int peak)
{
    if (d <= 0 || peak <= 0) return -1;
    float norm = logf(1.0f + (float)d) / logf(1.0f + (float)peak);
    int   tier = round_half_up(norm * (float)(N_TIERS - 1));
    if (tier < 0)         tier = 0;
    if (tier >= N_TIERS)  tier = N_TIERS - 1;
    return tier;
}

/* Intensity ramp for the N_TIERS brightness levels. */
static inline char tier_glyph(int tier)
{
    static const char ramp[N_TIERS] = {
        '.', ',', ':', ';', '+', '*', '#', '@'
    };
    return ramp[tier];
}

/* A_BOLD on the hottest tier so envelope lines glow. */
static inline chtype tier_attr(int tier)
{
    return (tier == BRIGHT_TIER) ? A_BOLD : 0;
}

/* ===================================================================== */
/* §5  scene                                                              */
/* ===================================================================== */

/* ---- ViewWindow -- the r-axis viewport ------------------------------ */

/*
 * ViewWindow -- the visible slice of r-axis as [r_min, r_max].
 *
 * Coordinate transform from screen column to logistic-map parameter:
 *
 *     r(c) = r_min + (r_max − r_min) · c / (cols − 1)
 *
 * (with sub-pixel refinement via column_to_r in §5).  Shrinking the
 * window zooms in -- the user sees the period-doubling cascade in
 * ever-finer detail as auto-zoom drives the window toward FEIGENBAUM_R.
 * Translating the window pans.
 *
 * WHY a separate substruct (vs flat r_min/r_max on Scene):
 *   The view operations (pan, zoom, recenter, clamp, width-query) form
 *   a coherent abstract data type: "the visible slice of r-axis".
 *   Grouping them lets each operation take `ViewWindow *` and not
 *   accidentally touch unrelated Scene fields.  Also lets
 *   diagram_build_density take `const ViewWindow *` so its inability
 *   to mutate the viewport is enforced by the compiler.
 *
 * Invariants (maintained by view_window_clamp at every mutation site):
 *   I1.  R_DOMAIN_LO ≤ r_min < r_max ≤ R_DOMAIN_HI
 *   I2.  width = r_max - r_min ≥ 0;  may become ≤ ZOOM_MIN_WIDTH
 *        (the auto-zoom drift stops there to avoid float collapse)
 *
 * Reference:  Strogatz [4] ch. 10 for the bifurcation diagram structure
 *   that this viewport is sampling.
 *
 * Fields:
 *   r_min   left edge of the viewport, in r-axis units.
 *           Range: [R_DOMAIN_LO, R_DOMAIN_HI).
 *           Mutators: view_window_init / pan / zoom / recenter_to.
 *   r_max   right edge of the viewport, in r-axis units.
 *           Range: (R_DOMAIN_LO, R_DOMAIN_HI].
 *           Same mutators as r_min.
 */
typedef struct {
    float r_min;
    float r_max;
} ViewWindow;

static void view_window_init(ViewWindow *v)
{
    v->r_min = R_DEFAULT_LO;
    v->r_max = R_DEFAULT_HI;
}

/* Clamp the window into the logistic-map valid r-domain. */
static void view_window_clamp(ViewWindow *v)
{
    if (v->r_min < R_DOMAIN_LO) v->r_min = R_DOMAIN_LO;
    if (v->r_max > R_DOMAIN_HI) v->r_max = R_DOMAIN_HI;
}

static inline float view_window_width(const ViewWindow *v)
{
    return v->r_max - v->r_min;
}

static inline bool view_window_too_narrow(const ViewWindow *v)
{
    return view_window_width(v) <= ZOOM_MIN_WIDTH;
}

/* If the window has slid past R_DOMAIN_LO, push the right edge by
 * the same overshoot so width is preserved.  No-op if in range. */
static void view_window_reflect_off_left(ViewWindow *v)
{
    if (v->r_min < R_DOMAIN_LO) {
        v->r_max += R_DOMAIN_LO - v->r_min;
        v->r_min  = R_DOMAIN_LO;
    }
}

/* Symmetric case: if window has slid past R_DOMAIN_HI, pull the left
 * edge back by the same overshoot.  No-op if in range. */
static void view_window_reflect_off_right(ViewWindow *v)
{
    if (v->r_max > R_DOMAIN_HI) {
        v->r_min -= v->r_max - R_DOMAIN_HI;
        v->r_max  = R_DOMAIN_HI;
    }
}

/*
 * view_window_pan -- translate the window by `fraction` of its width.
 *
 * Reads as pseudocode:
 *   shift both bounds by  width · fraction
 *   reflect off the left edge   (preserve width)
 *   reflect off the right edge  (preserve width)
 *
 * The reflection keeps the window's width invariant under pan, so the
 * user can't accidentally shrink it by sliding past the domain edge.
 */
static void view_window_pan(ViewWindow *v, float fraction)
{
    float w = view_window_width(v);
    v->r_min += w * fraction;
    v->r_max += w * fraction;
    view_window_reflect_off_left(v);
    view_window_reflect_off_right(v);
}

/* Scale the window's width by 1/factor while keeping its centre fixed.
 *   factor > 1  zooms IN  (window shrinks)
 *   factor < 1  zooms OUT (window grows) */
static void view_window_zoom(ViewWindow *v, float factor)
{
    float c  = (v->r_min + v->r_max) * 0.5f;
    float hw = view_window_width(v) / factor * 0.5f;
    v->r_min = c - hw;
    v->r_max = c + hw;
    view_window_clamp(v);
}

/* Shrink the window towards a target r value -- the per-frame auto-zoom
 * step.  New width = old width × shrink, recentred on `target`. */
static void view_window_recenter_to(ViewWindow *v, float target, float shrink)
{
    float hw = view_window_width(v) * 0.5f * shrink;
    v->r_min = target - hw;
    v->r_max = target + hw;
    view_window_clamp(v);
}

/* ---- Scene -- the whole simulation state ---------------------------- */

/*
 * Scene -- the entire simulation state.
 *
 * The state vector of the visualiser: the minimum set of values needed
 * to render the next frame.  Everything else (HUD strings, ncurses
 * cells) is derived.
 *
 * Composition decisions:
 *   - `view` is its own struct (ViewWindow) because the pan/zoom/
 *     recenter operations form a coherent ADT -- see ViewWindow doc.
 *   - `density` is derived data living on Scene rather than in a
 *     helper struct because its lifetime matches Scene (rebuilt every
 *     frame by diagram_build_density; cleared at start).
 *   - `theme_idx` is a flat int (index into k_themes[]).  Cycling
 *     with t/T is a single arithmetic bump; no pointer swizzling.
 *   - `paused` is a flat bool; only scene_tick reads it.  Doesn't
 *     earn its own substruct.
 *
 * Memory:
 *   density is the heavy field (~160 KB).  Scene therefore lives inside
 *   the static g_app (§8); never on the stack, never allocated.
 *
 * Reference:  May [1] for the logistic map this state visualises;
 *   Strogatz [4] §10 for the bifurcation theory it animates.
 *
 * Fields (range / unit / mutator):
 *   view        ViewWindow   --        the live r-axis viewport.
 *                                       Mutator: scene_tick (auto-zoom),
 *                                       action_reset / pan_* / zoom_*.
 *   density     Density      --        per-frame histogram, rebuilt by
 *                                       diagram_build_density.  Read-only
 *                                       during render passes.
 *   theme_idx   [0, N_THEMES)
 *                            --        active theme; t/T cycle.
 *                                       Mutator: action_theme_next/prev
 *                                       (also calls theme_apply).
 *   paused      bool         --        SPACE freezes scene_tick's auto-
 *                                       zoom drift.  Rendering still
 *                                       runs so the user can pan/zoom
 *                                       manually while paused.
 *                                       Mutator: action_pause.
 */
typedef struct {
    ViewWindow view;       /* live r-axis viewport                       */
    Density    density;    /* per-frame attractor histogram (~160 KB)   */
    int        theme_idx;  /* [0, N_THEMES)  active palette index        */
    bool       paused;     /* SPACE freezes auto-zoom drift              */
} Scene;

static void scene_init(Scene *s)
{
    view_window_init(&s->view);
    density_clear(&s->density);
    s->theme_idx = 0;
    s->paused    = false;
}

/*
 * scene_tick -- one simulation step.  Just the auto-zoom drift; the
 * logistic-map iteration is part of the render pass (it depends on the
 * live (rows, cols) so it can't run independently of the frame layout).
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    if (view_window_too_narrow(&s->view)) return;
    view_window_recenter_to(&s->view, FEIGENBAUM_R, AUTO_ZOOM_SHRINK);
}

/* ---- diagram build pass --------------------------------------------- */

/* Logistic-map iteration: x → r·x·(1−x).  One step, hot inner loop. */
static inline float logistic_step(float r, float x)
{
    return r * x * (1.0f - x);
}

/* Map attractor value x ∈ [0, 1] to a screen row inside the plot band. */
static inline int x_to_row(float x, int plot_top, int plot_height)
{
    /* Invert top↔bottom so larger x sits higher on the screen. */
    return plot_top + plot_height - 1
         - round_half_up(x * (float)(plot_height - 1));
}

/* Run WARMUP discards followed by PLOT stamps for one (r, column). */
static void column_accumulate(Density *dens, float r, int col,
                              int plot_top, int plot_bottom_excl)
{
    int plot_h = plot_bottom_excl - plot_top;
    float x = 0.5f;
    for (int i = 0; i < WARMUP; i++) x = logistic_step(r, x);
    for (int i = 0; i < PLOT; i++) {
        x = logistic_step(r, x);
        int row = x_to_row(x, plot_top, plot_h);
        if (row < plot_top || row >= plot_bottom_excl) continue;
        if (row >= MAX_ROWS) continue;
        density_stamp(dens, row, col);
    }
}

/*
 * Sub-pixel r position for sample sx ∈ [0, SUBPIXEL) within column col.
 *
 * Each visible column [col, col+1) is split into SUBPIXEL equal slices;
 * each slice picks its r at the slice midpoint.  This anti-aliases the
 * attractor curves so they look smooth even at modest column counts.
 */
static inline float column_to_r(int col, int sx, int n_cols,
                                float r_min, float r_max)
{
    float t = ((float)col + ((float)sx + 0.5f) / (float)SUBPIXEL)
            / (float)n_cols;
    return r_min + (r_max - r_min) * t;
}

/*
 * diagram_build_density -- scan every visible column, sub-pixel r-sample
 * it, iterate the logistic map, and accumulate counts into the density
 * buffer.  Reads as pseudocode:
 *
 *   clear density
 *   for each visible column:
 *     for each sub-pixel slice:
 *       r = column_to_r(col, sx)
 *       skip if r outside valid map domain
 *       column_accumulate(dens, r, col, ...)
 */
static void diagram_build_density(Density *dens, const ViewWindow *v,
                                  int rows, int cols)
{
    density_clear(dens);

    int plot_top         = HUD_TOP_ROWS;
    int plot_bottom_excl = rows - HUD_BOTTOM_ROWS;
    if (plot_bottom_excl - plot_top < 2) return;

    int n_cols = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int col = 0; col < n_cols; col++) {
        for (int sx = 0; sx < SUBPIXEL; sx++) {
            float r = column_to_r(col, sx, n_cols, v->r_min, v->r_max);
            if (r <= R_DOMAIN_LO || r > R_DOMAIN_HI) continue;
            column_accumulate(dens, r, col, plot_top, plot_bottom_excl);
        }
    }
}

/* ---- render pass ----------------------------------------------------- */

/* Paint one density cell through the active tier ramp + palette. */
static void paint_density_cell(int row, int col, int count, int peak)
{
    int tier = density_to_tier(count, peak);
    if (tier < 0) return;
    chtype attr = COLOR_PAIR(CP_TIER_0 + tier) | tier_attr(tier);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)tier_glyph(tier));
    attroff(attr);
}

/* Walk the plot band; paint every non-empty cell. */
static void render_density(const Density *dens, int rows, int cols)
{
    int row_end = rows - HUD_BOTTOM_ROWS;
    if (row_end > MAX_ROWS) row_end = MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int y = HUD_TOP_ROWS; y < row_end; y++)
        for (int x = 0; x < col_end; x++)
            paint_density_cell(y, x, dens->cells[y][x], dens->peak);
}

/* Compose the diagram: rebuild the density buffer from the current view,
 * then paint it through the tier ramp. */
static void scene_draw(Scene *s, int rows, int cols)
{
    diagram_build_density(&s->density, &s->view, rows, cols);
    render_density(&s->density, rows, cols);
}

/* ===================================================================== */
/* §6  hud                                                                */
/* ===================================================================== */

/*
 * Top HUD (rows 0 .. HUD_TOP_ROWS - 1) -- live data only.
 *   Row 0:  title (cyan) + r-window + theme + peak + status (yellow)
 *   Row 1:  N_TICKS r-axis labels + right-aligned r_inf annotation
 *
 * Bottom HUD (row rows - 1) -- action keys, bright cyan + A_BOLD.
 */

static void hud_draw_title(void)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(0, 0, " [BIFURCATION] ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void hud_draw_state(const Scene *s)
{
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, HUD_DATA_COL,
             " r=[%.4f, %.4f]  dr=%.4f  t:%-9s  peak=%4d  %-7s ",
             (double)s->view.r_min, (double)s->view.r_max,
             (double)view_window_width(&s->view),
             k_themes[s->theme_idx].name, s->density.peak,
             s->paused ? "PAUSED " : "zooming");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Draw the N_TICKS evenly-spaced r-axis labels along HUD row 1. */
static void hud_draw_r_axis_ticks(int cols, const ViewWindow *v)
{
    float w = view_window_width(v);
    for (int i = 0; i < N_TICKS; i++) {
        float t  = (float)i / (float)(N_TICKS - 1);
        int   cx = (int)(t * (float)(cols - 1));
        float r  = v->r_min + w * t;
        if (cx < cols - TICK_LABEL_RIGHT_MARGIN)
            mvprintw(1, cx, "%.3f", r);
    }
}

/* Right-side annotation on HUD row 1: target Feigenbaum point r∞. */
static void hud_draw_feigenbaum_annotation(int cols)
{
    char tag[40];
    snprintf(tag, sizeof tag, " r_inf=%.4f ", (double)FEIGENBAUM_R);
    int len = (int)strlen(tag);
    if (len < cols)
        mvprintw(1, cols - len, "%s", tag);
}

/*
 * hud_draw_tick_labels -- compose HUD row 1.
 *   Reads as pseudocode:
 *     ticks first (left + middle)
 *     then the r_inf annotation (right edge)
 *     all in CP_HUD colour
 */
static void hud_draw_tick_labels(int cols, const ViewWindow *v)
{
    attron(COLOR_PAIR(CP_HUD));
    hud_draw_r_axis_ticks(cols, v);
    hud_draw_feigenbaum_annotation(cols);
    attroff(COLOR_PAIR(CP_HUD));
}

static void hud_draw_action_bar(int rows)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit   SPACE:pause   r:reset   t:theme   "
             "left/right:pan   +/-:zoom ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen -- cached terminal geometry.
 *
 * WHY cache cols/rows on a struct instead of calling getmaxyx() everywhere:
 *   getmaxyx is cheap but not free, and -- more importantly -- the
 *   render layer takes plain ints for sizes.  Passing a Screen* down
 *   the call chain would conflate "I need the size" with "I need to
 *   talk to ncurses".  Holding size as a value lets the math layer
 *   stay terminal-agnostic; only screen_init / screen_resize ever
 *   write into Screen.
 *
 * Capacity clamps (screen_clamp_sizes):
 *   Values are clamped to MAX_ROWS / MAX_COLS at every update so the
 *   Density buffer never sees a row/col out of its static allocation.
 *   Very large terminals see a centred plot with empty margin rather
 *   than a corrupted buffer or a segfault.
 *
 * Update points:
 *   screen_init    -- once at startup
 *   screen_resize  -- after each SIGWINCH (triggered in §8 apply_resize)
 *
 * Fields follow the ncurses convention: cols = X-extent, rows = Y-extent,
 * both 1-based counts so valid coordinates are
 *      [0, cols-1] × [0, rows-1].
 * HUD bands reserve HUD_TOP_ROWS at the top and HUD_BOTTOM_ROWS at the
 * bottom; the diagram renderer stays inside the middle band.
 */
typedef struct {
    int cols;   /* terminal width  in character cells  */
    int rows;   /* terminal height in character cells  */
} Screen;

static void screen_clamp_sizes(Screen *s)
{
    if (s->rows > MAX_ROWS) s->rows = MAX_ROWS;
    if (s->cols > MAX_COLS) s->cols = MAX_COLS;
}

static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_clamp_sizes(s);
}

static void screen_resize(Screen *s)
{
    endwin(); refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_clamp_sizes(s);
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App -- the top-level container.
 *
 * Lives as a single static instance (g_app, declared below) so the
 * signal handlers can poke flags on it without scattering globals
 * across the file.
 *
 * WHY signal-handler flags are on App, not Scene:
 *   Signals interrupt the main loop, not the simulation.  "Should I
 *   quit?" and "did the terminal resize?" are run-loop concerns, not
 *   bifurcation-math concerns.  Keeping them off Scene means the
 *   simulation stays PURE STATE -- snapshottable, replayable from a
 *   single (r_min, r_max, theme_idx) triple, testable in isolation.
 *
 * WHY volatile sig_atomic_t (and not bool):
 *   Per POSIX, sig_atomic_t is the only integer type guaranteed atomic
 *   with respect to async signal delivery.  `volatile` forces the
 *   main loop to re-read from memory each iteration rather than
 *   caching the flag in a register; without it the compiler may
 *   legally hoist the load out of the loop and the program would
 *   never see the signal-set value.
 *
 * Composition:
 *   scene        -- the simulation state (Scene, §5).  Heavy due to
 *                   the Density buffer; stays in BSS via g_app.
 *   screen       -- cached terminal geometry (Screen, §7).
 *   running      -- 0 ⇒ main loop should exit (SIGINT/SIGTERM signal
 *                   handler clears it; main checks each iteration).
 *   need_resize  -- 1 ⇒ main loop should re-init screen
 *                   (SIGWINCH signal handler sets it; main consumes
 *                   it in apply_resize).
 */
typedef struct {
    Scene                 scene;        /* simulation state             */
    Screen                screen;       /* terminal size cache          */
    volatile sig_atomic_t running;      /* 0 = exit main loop  (SIGINT) */
    volatile sig_atomic_t need_resize;  /* 1 = re-init screen  (SIGWINCH) */
} App;

static App g_app;

static void on_signal(int sig)
{
    if (sig == SIGWINCH) g_app.need_resize = 1;
    else                 g_app.running     = 0;
}

static void cleanup(void) { endwin(); }

static void install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_signal);
    signal(SIGTERM,  on_signal);
    signal(SIGWINCH, on_signal);
}

static void app_init(App *app)
{
    app->running     = 1;
    app->need_resize = 0;
    screen_init(&app->screen);
    scene_init(&app->scene);
}

/* SIGWINCH path: re-query terminal size; density and view stay valid. */
static void apply_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

/* ---- key actions (each one a named helper for clarity) -------------- */

static void action_pause       (Scene *s) { s->paused = !s->paused; }
static void action_reset       (Scene *s)
{
    view_window_init(&s->view);
    s->paused = false;
}
static void action_theme_next  (Scene *s)
{
    s->theme_idx = theme_next(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_theme_prev  (Scene *s)
{
    s->theme_idx = theme_prev(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_pan_left    (Scene *s) { view_window_pan(&s->view, -PAN_FRAC); }
static void action_pan_right   (Scene *s) { view_window_pan(&s->view,  PAN_FRAC); }
static void action_zoom_in     (Scene *s) { view_window_zoom(&s->view, ZOOM_FACTOR); }
static void action_zoom_out    (Scene *s) { view_window_zoom(&s->view, 1.0f / ZOOM_FACTOR); }

/* Returns false if the user asked to quit. */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC:  return false;
    case ' ':            action_pause(s);        break;
    case 'r': case 'R':  action_reset(s);        break;
    case 't':            action_theme_next(s);   break;
    case 'T':            action_theme_prev(s);   break;
    case KEY_LEFT:       action_pan_left(s);     break;
    case KEY_RIGHT:      action_pan_right(s);    break;
    case '+': case '=':  action_zoom_in(s);      break;
    case '-': case '_':  action_zoom_out(s);     break;
    default: break;
    }
    return true;
}

static bool drain_input(App *app)
{
    int ch;
    while ((ch = getch()) != ERR) {
        if (!app_handle_key(app, ch)) return false;
    }
    return true;
}

/* Compose one frame: build density, paint diagram, paint HUD. */
static void frame_render(App *app)
{
    erase();
    scene_draw(&app->scene, app->screen.rows, app->screen.cols);
    hud_draw_title();
    hud_draw_state(&app->scene);
    hud_draw_tick_labels(app->screen.cols, &app->scene.view);
    hud_draw_action_bar(app->screen.rows);
    wnoutrefresh(stdscr);
    doupdate();
}

/*
 * main() -- the orchestrator.  Each line is one named phase of a frame.
 */
int main(void)
{
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    while (app->running) {
        if (app->need_resize)  apply_resize(app);
        if (!drain_input(app)) { app->running = 0; break; }

        scene_tick(&app->scene);

        long long t0 = clock_ns();
        frame_render(app);
        clock_sleep_ns(RENDER_NS - (clock_ns() - t0));
    }

    return 0;
}
