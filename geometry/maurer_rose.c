/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * maurer_rose.c — Maurer Rose (discrete-sample rose curve)
 *
 * The classical rose curve in polar form is r = cos(n·θ); it traces a
 * flower with n petals (odd n) or 2n petals (even n).  The Maurer Rose
 * is its DISCRETE cousin: sample θ at fixed d-degree intervals, plot
 * the resulting points (r, θ), and connect consecutive samples with
 * line segments.
 *
 *     For k = 1 .. N_SAMPLES:
 *         θ_k = k · d   (degrees, converted to radians at use site)
 *         r_k = cos(n · θ_k)
 *         P_k = (r_k cos θ_k, r_k sin θ_k)
 *     Draw  P_{k-1} → P_k  for every k.
 *
 * When d is small (≪ 1°) the polyline approximates the smooth rose.
 * When d is larger (10-170°) consecutive samples land FAR APART on the
 * curve, so each line segment crosses the rose's interior.  The 360
 * crossings weave a striking lacework of envelope edges and interior
 * detail.  Both (n, d) drift continuously, slowing near integer pairs
 * so the named classical configurations dwell on screen long enough
 * to read.
 *
 * Reference visual landmarks (n, d ≈):
 *     (2, 39)     2-petal rose, smooth envelope
 *     (3, 47)     3-petal rose with interior weave
 *     (6, 71)     Maurer's classic published example
 *     (7, 11)     dense lacework, 7-fold symmetry
 *     (5, 97)     star-like with 5-fold petals
 *
 * Rendering uses a DENSITY BUFFER: every chord is Bresenham-rasterised
 * into a per-cell counter, then the counter is mapped via sqrt scaling
 * to N_TIERS brightness tiers.  Theme palettes assign cool→hot colours
 * to the tiers so high-density envelope edges glow while sparse interior
 * weaves stay subtle.
 *
 * Keys:
 *   q / Q / ESC  quit
 *   space        pause / resume
 *   r / R        reset  (n → N_DEFAULT, d → D_DEFAULT)
 *   t / T        next / prev theme  (10 themes)
 *   [ / ]        previous / next preset (jump to a named landmark)
 *   + / =        increase drift speed × DRIFT_SPEED_STEP
 *   - / _        decrease drift speed ÷ DRIFT_SPEED_STEP
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra geometry/maurer_rose.c \
 *       -o maurer_rose -lncurses -lm
 *
 * Sections:
 *   §1 config   — sizes, tunings, named constants, colour-pair allocation
 *   §2 clock    — monotonic ns clock + sleep
 *   §3 color    — RGB, cosine_palette, ANSI quantizers, Theme + theme_apply
 *   §4 coords   — RenderFrame (per-frame unit→pixel transform)
 *   §5 entity   — RoseParams, RosePreset, Density, tier ramp, rose math
 *   §6 scene    — Scene + simulation tick + render pipeline
 *   §7 screen   — Screen, HUD helpers, screen_draw
 *   §8 app      — App, signal handlers, action keys, main loop
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm     : Discrete-sample rose curve with chord interpolation.
 *                 Sample N_SAMPLES values of θ in arithmetic progression
 *                 (common difference = d°), evaluate r = cos(n·θ) at each,
 *                 connect consecutive (r, θ) pairs as straight-line
 *                 chords in Cartesian space.  All chords accumulate into
 *                 a per-cell density buffer; the buffer is then painted
 *                 through a tier ramp.
 *
 * Math          : The rose r = cos(n·θ) has period 2π/n and traces
 *                 - n petals  if n is odd
 *                 - 2n petals if n is even
 *                 Discrete sampling at step d closes after lcm(d, 360)/d
 *                 steps -- normally 360 unless gcd(d, 360) > 1, in which
 *                 case the path closes early and yields a sparse star
 *                 (e.g. d = 60° gives a 6-pointed degenerate rose).
 *                 (See Maurer 1987 [1].)
 *
 *                 The interior weave geometry is itself studied as
 *                 chord-family envelope theory (Lockwood [3] §roulettes).
 *                 Slow drift in (n, d) is two-frequency quasi-periodic
 *                 motion in the parameter space (Strogatz [5] §8).
 *
 * Rendering     : Density rendering is the discrete-cell analogue of a
 *                 graphics accumulation buffer (Akenine-Möller [8] §23).
 *                 The sqrt response curve from density to brightness
 *                 tier keeps single-crossing cells visible without
 *                 saturating multi-crossing envelope edges.
 *
 * Palette       : Each theme is a COSINE GRADIENT (Quilez 2015 [9])
 *                 parameterised by four RGB triples (a, b, c, d) per
 *                 channel:
 *                     color(t) = a + b · cos(2π · (c·t + d))
 *                 Sampling at t = i/(N_TIERS-1) gives a smooth cool→hot
 *                 ramp; theme_apply quantises each stop to nearest
 *                 ANSI 256-cube colour (or 8-colour fallback).
 * ─────────────────────────────────────────────────────────────────────── */

/* -- REFERENCES ---------------------------------------------------------  *
 *
 * Discrete-sample rose curves:
 *   [1] Maurer, P. M. (1987). "A Rose is a Rose..."
 *       American Mathematical Monthly 94(7), 631-645.
 *       -- the original paper that defines and names the Maurer rose;
 *          gives the construction P_k from (n, d) and presents the
 *          (6, 71) landmark figure that became the canonical example.
 *
 * Curve theory:
 *   [2] Lawrence, J. D. (1972). "A Catalog of Special Plane Curves."
 *       Dover Publications.
 *       -- chapter on rose curves: r = cos(n·θ) family, petal-count
 *          parity rule (odd n → n petals, even n → 2n petals).
 *
 *   [3] Lockwood, E. H. (1961). "A Book of Curves."
 *       Cambridge University Press.
 *       -- chord-family envelope theory; the geometric construction
 *          behind the interior weave of a Maurer rose.
 *
 *   [4] Maor, E. (1998). "Trigonometric Delights."
 *       Princeton University Press.
 *       -- accessible trig-first treatment of rose curves and
 *          parametric form derivation.
 *
 * Dynamics:
 *   [5] Strogatz, S. H. (2015). "Nonlinear Dynamics and Chaos" (2nd ed.).
 *       Westview Press.
 *       -- §8 quasi-periodic motion; the (n, d) drift is a two-frequency
 *          system in parameter space.
 *
 * Rendering:
 *   [6] Bresenham, J. E. (1965). "Algorithm for computer control of a
 *       digital plotter." IBM Systems Journal 4(1), 25-30.
 *       -- integer DDA line algorithm used to rasterise each chord
 *          into the density buffer.
 *
 *   [7] Foley, van Dam, Feiner, Hughes (1995). "Computer Graphics:
 *       Principles and Practice" (2nd ed. in C), chapter 11.
 *       -- parametric-curve sampling theory; uniform-vs-adaptive
 *          step-size trade-offs for polyline approximation.
 *
 *   [8] Akenine-Möller, T., Haines, E., Hoffman, N., et al. (2018).
 *       "Real-Time Rendering" (4th ed.). CRC Press.
 *       -- §23 on accumulation-buffer techniques; the density buffer
 *          is the discrete-cell analogue.
 *
 *   [9] Quilez, I. (2015). "Palettes."
 *       https://iquilezles.org/articles/palettes/
 *       -- the cosine-gradient palette technique used in §3 theme
 *          generation:  color(t) = a + b·cos(2π·(c·t + d)).  Each
 *          theme is parameterised by four RGB triples (a, b, c, d)
 *          and sampled at N_TIERS positions to produce the per-tier
 *          colour palette at runtime.
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

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

#define NS_PER_SEC        1000000000LL
#define NS_PER_MS         1000000LL

#define TICK_NS           33333333LL              /* ~30 fps             */
#define DT_CAP_NS         (100 * NS_PER_MS)       /* spiral-of-death cap */
#define FRAME_PERIOD_NS   (NS_PER_SEC / 60)       /* render cadence      */
#define FPS_UPDATE_MS     500

#define CELL_W            8
#define CELL_H            16

#define MAX_ROWS          128
#define MAX_COLS          320

/* Maurer-rose parameter drift.
 *
 *   n  -- petal-count parameter of the underlying rose r = cos(n·θ).
 *   d  -- angular step (in degrees) between successive samples.
 *
 * The pair (n, d) defines the figure; both drift continuously.  Drift
 * slows near integer-pair landmarks so named configurations sit on
 * screen long enough to study (the "dwell envelope" -- see §6
 * dwell_envelope).
 */
#define N_DEFAULT         6.0f
#define N_MIN             2.0f
#define N_MAX             9.0f
#define N_SPEED_DEFAULT   0.002f     /* per-tick advance at full speed   */

#define D_DEFAULT         71.0f
#define D_MIN             2.0f
#define D_MAX             178.0f
#define D_SPEED_DEFAULT   0.07f      /* per-tick advance at full speed   */

#define DRIFT_SPEED_STEP  1.5f       /* +/- key multiplier               */
#define DRIFT_SPEED_MIN_F 0.10f      /* speed scaled to >= 10% of default*/
#define DRIFT_SPEED_MAX_F 5.00f      /* speed scaled to <= 500% default  */

/* Dwell envelope (drift slows near integer (n, d)). */
#define DWELL_FLOOR       0.12f      /* speed multiplier at the centre  */
#define DWELL_RAMP        1.88f      /* ramp coefficient                */

/* Sampling. */
#define N_SAMPLES         360        /* chord count per frame           */
#define ROSE_AMP_FRAC     0.48f      /* envelope radius / shorter dim   */

/* Brightness tiers — density → tier mapping uses sqrt of (d/peak).
 * 8 tiers paired with 8 cosine-gradient palette stops; each tier maps
 * to one stop, so the gradient runs cold (tier 0) → hot (tier 7). */
#define N_TIERS           8
#define BRIGHT_TIER       (N_TIERS - 1)

/* HUD bands. */
enum { HUD_TOP_ROWS = 1, HUD_BOTTOM_ROWS = 1 };

/*
 * Colour-pair allocation:
 *   1 .. N_TIERS         density tiers (theme-controlled by theme_apply)
 *   N_TIERS + 1          rim                       (theme-independent)
 *   N_TIERS + 2          centre marker             (theme-independent)
 *   N_TIERS + 3          HUD title / hint  (cyan)  (theme-independent)
 *   N_TIERS + 4          HUD data          (yellow)(theme-independent)
 */
#define CP_TIER_0         1
#define CP_RIM            (N_TIERS + 1)
#define CP_CENTRE         (N_TIERS + 2)
#define HUD_PAIR_TITLE    (N_TIERS + 3)
#define HUD_PAIR_DATA     (N_TIERS + 4)
#define HUD_PAIR_HINT     HUD_PAIR_TITLE

#define KEY_ESC           27

/*
 * Math literals named for their geometric meaning.
 *   TAU         one full turn in radians (= 2π).  Used by cosine_palette
 *               and the rose-curve parametric formulas.
 *   DEG_TO_RAD  conversion from degrees to radians (= π/180).  Used by
 *               maurer_step_index_to_theta to convert the d-step.
 */
#define TAU               (2.0f * (float)M_PI)
#define DEG_TO_RAD        ((float)M_PI / 180.0f)

/*
 * ANSI 256-colour cube layout (RGB sub-cube indices 16..231).
 *   pair_code = ANSI_CUBE_BASE + R·ANSI_CUBE_R_STRIDE
 *                              + G·ANSI_CUBE_G_STRIDE
 *                              + B
 * where R, G, B ∈ [0, ANSI_CUBE_MAX_STEP] indexes into the 6-step
 * per-channel grid {0, 95, 135, 175, 215, 255}.
 */
#define ANSI_CUBE_BASE        16
#define ANSI_CUBE_R_STRIDE    36
#define ANSI_CUBE_G_STRIDE     6
#define ANSI_CUBE_MAX_STEP     5

/* Per-channel threshold for the 8-colour fallback in rgb_to_ansi8.  A
 * channel "is on" if it sits in the upper half of [0, 1].  The eight
 * (R, G, B) on/off combinations correspond exactly to the eight
 * standard ANSI colours. */
#define RGB_BIT_THRESHOLD     0.5f

/* HUD: column where the middle data segment starts (after the title). */
#define HUD_DATA_COL          15

/*
 * Default preset slot used by scene_init.  Index 4 = "classic-6"
 * (n=6, d=71), Maurer's published landmark.  Boot to it so first-
 * impression viewers see the canonical figure.
 */
#define DEFAULT_PRESET_IDX    4

/*
 * HUD chrome colour codes (256-mode).  Kept here so all 256-cube
 * colour choices live in one place; pick by purpose, not by number:
 *   240 -- dim grey, faint enough not to compete with the rose
 *   251 -- light grey, slightly brighter so the centre dot reads
 *    51 -- pure cyan, maximum brightness for HUD title / hints
 *   226 -- pure yellow, maximum brightness for HUD data
 */
#define HUD_RIM_GREY_256       240
#define HUD_CENTRE_GREY_256    251
#define HUD_TITLE_CYAN_256      51
#define HUD_DATA_YELLOW_256    226

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec ts = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                           .tv_nsec = (long)(ns % NS_PER_SEC) };
    nanosleep(&ts, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/* ---- RGB + cosine palette helpers ------------------------------------ */

/*
 * RGB -- a normalised colour triple in linear [0, 1]^3.
 *
 * WHY normalised (not 0..255 ints):
 *   The cosine-palette formula (Quilez [9]) is naturally float-valued;
 *   coefficients (a, b, c, d) live in [0, 1] for visual coherence.
 *   Keeping the working representation in floats avoids round-trips
 *   through integer space until the very last quantization step
 *   (rgb_to_ansi256 / rgb_to_ansi8).
 *
 * Colour space:
 *   These values are TREATED as linear RGB for blending arithmetic.
 *   Real ANSI terminals interpret colour codes through sRGB gamma, so
 *   strictly speaking we should gamma-correct before quantising.  At
 *   the resolution of a 6-step cube (~16% per step) the gamma error is
 *   well below the quantization noise, so we skip the conversion.
 *
 * Fields are read-only after cosine_palette returns -- there is no
 * RGB mutator anywhere in the program.
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

/*
 * cosine_palette -- procedural gradient sampler (Iñigo Quilez 2015 [9]).
 *
 * For each channel, color(t) = a + b · cos(TAU · (c · t + d))
 *
 * Parameter intuition (each is itself an RGB so per-channel control is
 * possible -- different channels can run at different frequencies and
 * phases, which is what produces the rainbow-style hue rotation):
 *
 *   a   DC offset             ≈ mid value the channel oscillates around
 *   b   amplitude (half range) ≈ max excursion above/below a
 *   c   frequency             cosine cycles fit in t ∈ [0, 1]
 *                               c = 0.5 -> half cycle (monotonic sweep)
 *                               c = 1.0 -> one full cycle (wraps back)
 *   d   phase                 cosine offset; controls where each
 *                               channel starts in its cycle
 *
 * Example (monotonic dark→bright green channel):
 *   g = 0.55 + 0.45 · cos(TAU · (0.5 · t + 0.5))
 *     at t=0  -> 0.55 + 0.45·cos(π)    = 0.10  (dim green)
 *     at t=1  -> 0.55 + 0.45·cos(TAU)  = 1.00  (bright green)
 *
 * Reference:  Iñigo Quilez, "Palettes" (2015) [9] --
 *   https://iquilezles.org/articles/palettes/
 */
static RGB cosine_palette(const RGB *a, const RGB *b,
                          const RGB *c, const RGB *d, float t)
{
    RGB out;
    out.r = a->r + b->r * cosf(TAU * (c->r * t + d->r));
    out.g = a->g + b->g * cosf(TAU * (c->g * t + d->g));
    out.b = a->b + b->b * cosf(TAU * (c->b * t + d->b));

    /* Clamp -- the cosine can drive a channel out of [0,1] if (a, b)
     * isn't tuned conservatively.  Clipping rather than rescaling keeps
     * adjacent tiers' relative balance intact. */
    out.r = clamp01(out.r);
    out.g = clamp01(out.g);
    out.b = clamp01(out.b);
    return out;
}

/*
 * rgb_to_ansi256 -- nearest-cube quantizer for the xterm 256-colour
 * palette.  Cube colours sit at indices ANSI_CUBE_BASE .. 231 with
 *   index = ANSI_CUBE_BASE
 *         + ANSI_CUBE_R_STRIDE · R
 *         + ANSI_CUBE_G_STRIDE · G
 *         + B
 * where R, G, B ∈ [0, ANSI_CUBE_MAX_STEP].  We quantize each channel
 * uniformly to ANSI_CUBE_MAX_STEP + 1 steps; the cube's actual step
 * values aren't evenly spaced in CIE but the visual difference is small
 * at this resolution.
 */
static int rgb_channel_to_cube_step(float v)
{
    int idx = (int)(v * (float)ANSI_CUBE_MAX_STEP + 0.5f);
    if (idx < 0)                   idx = 0;
    if (idx > ANSI_CUBE_MAX_STEP)  idx = ANSI_CUBE_MAX_STEP;
    return idx;
}

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

/*
 * rgb_to_ansi8 -- 8-colour fallback for legacy terminals.
 * Threshold each channel at RGB_BIT_THRESHOLD; the 8 possible (R, G, B)
 * on/off corners of the cube map directly to the 8 standard ANSI colours.
 */
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
 * Theme -- a named procedural cool→hot gradient.
 *
 * Each theme is FOUR RGB triples (a, b, c, d) that fully specify a
 * cosine-gradient palette in the sense of Quilez 2015 [9]:
 *
 *     color(t) = a + b · cos(2π · (c · t + d))
 *
 * The same formula is evaluated independently per channel, so a, b, c, d
 * are each RGB triples (not scalars).  Different channels can have
 * different (c, d) which is what produces rainbow-style hue rotation:
 * shift the phase of each channel by 1/3 of a cycle and you get a full
 * hue sweep across t ∈ [0, 1].
 *
 * WHY cosine-gradient parameters (and not a fixed N_TIERS-stop table):
 *
 *   1. Compactness -- 12 floats per theme instead of N_TIERS·sizeof(int)
 *      for 256-mode plus N_TIERS·sizeof(int) for 8-mode.  And the floats
 *      DESCRIBE the gradient instead of just listing its samples.
 *
 *   2. Resolution-independent -- any t ∈ [0, 1] is well-defined.  If we
 *      later bump N_TIERS from 8 to 16 the palette generator scales
 *      without a code change; the gradient stays smooth at any density.
 *
 *   3. Self-documenting -- the parameter values reveal the gradient
 *      shape directly.  c=0.5 means "monotonic sweep over [0,1]",
 *      c=1.0 means "full cycle, wraps back to start", and the phase d
 *      shifts mark obvious offsets.
 *
 *   4. Composable -- nudging one parameter (e.g. d.g += 0.1) rotates
 *      the gradient by a known amount.  Designing a new theme is
 *      easier than picking 8 ANSI codes by trial-and-error.
 *
 *   5. Procedural -- generating themes algorithmically (e.g. random
 *      drift in parameter space for a "screensaver" mode) is trivial
 *      compared to maintaining an N×N_TIERS table of magic ANSI codes.
 *
 * Worked sample -- "Matrix" theme at tier 0 (t = 0):
 *      r = 0.00 + 0.00·cos(2π·0)         = 0.00
 *      g = 0.55 + 0.45·cos(2π·0.5)       = 0.55 + 0.45·(-1) = 0.10
 *      b = 0.08 + 0.08·cos(2π·0.5)       = 0.08 + 0.08·(-1) = 0.00
 *  → faint dark green.
 * At tier 7 (t = 1):
 *      g = 0.55 + 0.45·cos(2π·1)         = 1.00
 *      b = 0.08 + 0.08·cos(2π·1)         = 0.16
 *  → bright green with a slight cyan tint.
 *
 * Theme switching: 't' / 'T' recompute the N_TIERS palette stops via
 * cosine_palette + rgb_to_ansi256 and rewrite pairs CP_TIER_0..
 * CP_TIER_0+N_TIERS-1 via init_pair.  Pairs above N_TIERS (rim,
 * centre, HUD) are left untouched so the chrome stays consistent.
 *
 * Reference:  Quilez 2015 [9] for the palette formula; Foley/van Dam [7]
 *   §13 on indexed-colour LUTs (theme_apply is the LUT swap, the
 *   density buffer is the indexed-colour framebuffer).
 */
typedef struct {
    const char *name;  /* HUD-displayable name ("Matrix", "Sunset", ...) */
    RGB         a;     /* DC offset (mid value the channel oscillates around) */
    RGB         b;     /* amplitude (max excursion above/below a).
                        * Must keep a + b ≤ 1 and a − b ≥ 0 to avoid clipping;
                        * cosine_palette clamps after the fact regardless.    */
    RGB         c;     /* frequency (cycles per t ∈ [0, 1]).
                        * c = 0.5 → monotonic sweep; c = 1.0 → full cycle.    */
    RGB         d;     /* phase (where in the cosine the channel starts).
                        * d = 0.5 makes the cosine start at trough (cos(π)).  */
} Theme;

#define N_THEMES 10

static const Theme k_themes[N_THEMES] = {
    /*
     * Each entry tuned so tier 0 (faintest sparse-density cells) is dim
     * but visible, tier 7 (envelope edges) is the theme's "hot" hue.
     * c = 0.5 gives a monotonic sweep; c = 1.0 wraps a full hue cycle.
     */
    /*  name            a (offset)              b (amplitude)            c (frequency)            d (phase)              */
    { "Matrix",      {0.00f, 0.55f, 0.08f}, {0.00f, 0.45f, 0.08f}, {0.00f, 0.50f, 0.50f}, {0.00f, 0.50f, 0.50f} },
    { "Sunset",      {0.70f, 0.50f, 0.30f}, {0.20f, 0.50f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Ocean",       {0.20f, 0.45f, 0.65f}, {0.20f, 0.40f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.30f, 0.50f} },
    { "Plasma",      {0.55f, 0.40f, 0.55f}, {0.45f, 0.40f, 0.45f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
    { "Fire",        {0.55f, 0.30f, 0.15f}, {0.45f, 0.50f, 0.35f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.30f} },
    { "Mint",        {0.20f, 0.55f, 0.50f}, {0.20f, 0.40f, 0.40f}, {0.50f, 0.50f, 0.50f}, {0.30f, 0.50f, 0.50f} },
    { "Lavender",    {0.55f, 0.45f, 0.65f}, {0.40f, 0.35f, 0.30f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.10f, 0.20f} },
    { "Aurora",      {0.45f, 0.55f, 0.55f}, {0.40f, 0.40f, 0.40f}, {1.00f, 1.00f, 0.50f}, {0.00f, 0.50f, 0.75f} },
    { "Sepia",       {0.45f, 0.35f, 0.25f}, {0.40f, 0.35f, 0.20f}, {0.50f, 0.50f, 0.50f}, {0.00f, 0.05f, 0.10f} },
    { "Rainbow",     {0.50f, 0.50f, 0.50f}, {0.50f, 0.50f, 0.50f}, {1.00f, 1.00f, 1.00f}, {0.00f, 0.33f, 0.67f} },
};

static inline int theme_next(int cur) { return (cur + 1) % N_THEMES; }
static inline int theme_prev(int cur) { return (cur + N_THEMES - 1) % N_THEMES; }

/* Position of tier i along the gradient axis t ∈ [0, 1]. */
static inline float tier_to_gradient_t(int i)
{
    return (float)i / (float)(N_TIERS - 1);
}

/*
 * theme_apply -- compute the N_TIERS palette stops for the chosen theme
 * and load them into ncurses pairs CP_TIER_0..CP_TIER_0+N_TIERS-1.
 *
 * Reads as pseudocode:
 *   theme = k_themes[idx]
 *   for i in [0, N_TIERS):
 *     t   = i / (N_TIERS - 1)         -- gradient position
 *     rgb = cosine_palette(theme, t)  -- procedural color
 *     pair = quantize(rgb)            -- 256-cube or 8-colour
 *     init_pair(CP_TIER_0 + i, pair, -1)
 */
static void theme_apply(int theme_idx)
{
    const Theme *th = &k_themes[theme_idx];
    bool have_256   = (COLORS >= 256);
    for (int i = 0; i < N_TIERS; i++) {
        float t   = tier_to_gradient_t(i);
        RGB   rgb = cosine_palette(&th->a, &th->b, &th->c, &th->d, t);
        int   pair_code = have_256 ? rgb_to_ansi256(rgb) : rgb_to_ansi8(rgb);
        init_pair(CP_TIER_0 + i, pair_code, -1);
    }
}

/* One-time palette setup: theme-independent chrome first, then theme 0. */
static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(CP_RIM,          HUD_RIM_GREY_256,    -1);
        init_pair(CP_CENTRE,       HUD_CENTRE_GREY_256, -1);
        init_pair(HUD_PAIR_TITLE,  HUD_TITLE_CYAN_256,  -1);
        init_pair(HUD_PAIR_DATA,   HUD_DATA_YELLOW_256, -1);
    } else {
        init_pair(CP_RIM,          COLOR_WHITE,  -1);
        init_pair(CP_CENTRE,       COLOR_WHITE,  -1);
        init_pair(HUD_PAIR_TITLE,  COLOR_CYAN,   -1);
        init_pair(HUD_PAIR_DATA,   COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §4  coords                                                             */
/* ===================================================================== */

/*
 * RenderFrame -- per-frame screen-space layout.
 *
 * Bridges three coordinate systems:
 *
 *   unit space:    where the rose math lives -- x, y ∈ [-1, 1]
 *   pixel space:   sub-cell precision -- one pixel = 1/CELL_W of a
 *                  column or 1/CELL_H of a row.  Allows accurate line
 *                  endpoints before snapping to integer cells.
 *   cell space:    integer (col, row) for ncurses output.
 *
 * RenderFrame stores the unit→pixel transform: figure centre in pixels
 * (cx_px, cy_px) and a multiplicative scale (scale_px) that makes the
 * rose's |r| = 1 envelope fill ROSE_AMP_FRAC of the shorter screen
 * dimension.  scene_sample_cell does the rest:
 *      x_px = cx_px + x_unit · scale_px
 *      y_px = cy_px + y_unit · scale_px
 *      col  = round(x_px / CELL_W)
 *      row  = round(y_px / CELL_H)
 *
 * Vertical centring is INSIDE THE DRAWING BAND (rows [HUD_TOP_ROWS,
 * rows - HUD_BOTTOM_ROWS)), not the full screen.  Horizontal centring
 * uses the full column range since the HUD claims rows, not columns.
 *
 * WHY recomputed each frame (and NOT cached on Scene):
 *   Every field is a pure function of (rows, cols).  Caching would
 *   create a stale-value risk on resize (SIGWINCH between cache-write
 *   and frame-read could leave the figure centred wrong).  Recompute
 *   costs ~6 FLOPs/frame -- imperceptible.  Foley/van Dam [7] §11
 *   discusses these clean-each-frame transform pipelines.
 */
typedef struct {
    float cx_px;     /* figure centre, x  (sub-cell pixel coords)        */
    float cy_px;     /* figure centre, y  (sub-cell pixel coords)        */
    float scale_px;  /* unit→pixel scale; 1.0 in unit space becomes
                      * scale_px pixels on screen                        */
} RenderFrame;

static RenderFrame render_frame_make(int rows, int cols)
{
    int   draw_rows = rows - HUD_TOP_ROWS - HUD_BOTTOM_ROWS;
    float draw_h_px = (float)draw_rows * CELL_H;
    float draw_w_px = (float)cols      * CELL_W;
    return (RenderFrame){
        .cx_px    = (float)cols * CELL_W * 0.5f,
        .cy_px    = ((float)HUD_TOP_ROWS + (float)draw_rows * 0.5f) * CELL_H,
        .scale_px = fminf(draw_w_px, draw_h_px) * ROSE_AMP_FRAC,
    };
}

/* Round-half-up to integer cell. */
static inline int cell_round(float v) { return (int)(v + 0.5f); }

/* ===================================================================== */
/* §5  entity                                                             */
/* ===================================================================== */

/*
 * RoseParams -- the live (n, d) drifting parameter pair.
 *
 * The Maurer rose is fully specified by the integer pair (n, d) -- n
 * is the petal parameter of the underlying rose curve r = cos(n·θ), d
 * is the angular step in degrees between consecutive samples.  Both
 * drift as continuous floats so the demo can morph through hundreds
 * of integer landmarks smoothly.
 *
 * WHY (value, speed) together (instead of split structs):
 *   Each parameter has its own natural drift rate.  n drifts SLOWLY
 *   because petal count is a discrete visual feature -- you want it
 *   stable for several seconds at a time.  d drifts FASTER because
 *   the woven interior changes smoothly with d and a slow d makes the
 *   demo feel inert.  Storing speed adjacent to value documents this
 *   per-parameter-rate fact and means scene_tick advances each
 *   parameter with one rule per row rather than juggling four loose
 *   scalars on Scene.
 *
 * Dual-frequency drift framing:
 *   In dynamical-systems language (Strogatz [5] §8) the live point
 *   (n(t), d(t)) traces a trajectory in 2-D parameter space at
 *   constant velocity (n_speed, d_speed).  The dwell envelope makes
 *   velocity STATE-DEPENDENT (slows near integer pairs), so the
 *   trajectory is quasi-periodic rather than purely periodic --
 *   the named landmark configurations are attractors in time.
 *
 * Fields (range / unit / mutator):
 *   n         [N_MIN, N_MAX)    --      petal parameter of r=cos(n·θ).
 *                                       Mutators: scene_tick (drift),
 *                                       action_preset_* keys, reset.
 *   d         [D_MIN, D_MAX)    deg     sample step in degrees.
 *                                       Same mutator set as n.
 *   n_speed   [0, ∞)            /tick   per-tick advance for n.
 *                                       Mutator: action_speed_*
 *                                       multiplies via speed_scale.
 *   d_speed   [0, ∞)            /tick   per-tick advance for d.
 *                                       Same mutator set as n_speed.
 */
typedef struct {
    float n;          /* [N_MIN, N_MAX)   live petal parameter                */
    float d;          /* [D_MIN, D_MAX)   live sample step, degrees            */
    float n_speed;    /* /tick            base drift rate for n               */
    float d_speed;    /* /tick            base drift rate for d               */
} RoseParams;

/*
 * RosePreset -- a hardcoded named (n, d) landmark configuration.
 *
 * WHY this exists as a separate type (not just embedded in RoseParams):
 *
 *   1. Const correctness.  Presets are immutable reference points; the
 *      live parameters drift.  Keeping types distinct means the
 *      compiler stops anyone from accidentally writing into a preset.
 *
 *   2. Identification.  The HUD shows the NEAREST preset name to the
 *      current (n, d) so the user can learn the landmark inventory by
 *      watching the drift cross them.  This needs preset → name lookup;
 *      RoseParams has no name to give.
 *
 *   3. Navigation.  The [ / ] keys snap to previous/next preset,
 *      jumping the drift across the parameter plane.  Preset_idx
 *      ordering becomes the user's "interesting tour" through (n, d)
 *      space.
 *
 *   4. Bibliography.  Each preset name corresponds to a published
 *      figure or a well-known visual family (e.g. "classic-6" is
 *      Maurer's 1987 paper [1] frontispiece).  Naming makes the demo
 *      a callable index into curve-theory literature.
 *
 * Distance metric used to find the "nearest" preset:
 *   The (n, d) space has wildly different scales -- n ranges over ~7,
 *   d over ~176.  scene_nearest_preset_name rescales n by (D_MAX/N_MAX)
 *   before taking Euclidean distance, so a unit in n counts roughly
 *   the same as a unit in d.
 *
 * Fields:
 *   n     petal parameter for this landmark (integer in practice)
 *   d     sample step for this landmark (integer in practice)
 *   name  short human label ("classic-6", "fine-7", ...)
 */
typedef struct {
    float       n;
    float       d;
    const char *name;
} RosePreset;

/*
 * Hand-picked landmarks from Maurer [1] and follow-ups.  Order chosen
 * so [ / ] cycles from simple (low n) to complex (higher n, denser d).
 */
static const RosePreset k_presets[] = {
    { 2.0f,  39.0f, "soft-2"     },   /* gentle 2-petal weave           */
    { 3.0f,  47.0f, "trefoil"    },   /* 3-petal with crisscross        */
    { 4.0f,  89.0f, "quad-89"    },   /* 4-petal star envelope          */
    { 5.0f,  97.0f, "pentagon-5" },   /* 5-fold woven star              */
    { 6.0f,  71.0f, "classic-6"  },   /* Maurer's published landmark    */
    { 7.0f,  11.0f, "fine-7"     },   /* 7-fold dense lacework          */
    { 8.0f, 121.0f, "octa-121"   },   /* 8-fold envelope                */
    { 5.0f,  37.0f, "star-37"    },   /* 5-pointed star (gcd interest)  */
    { 3.0f,  53.0f, "triangle-3" },   /* near-triangle weave            */
    { 2.0f, 179.0f, "deg-179"    },   /* near-period mirror             */
};
#define N_PRESETS ((int)(sizeof(k_presets) / sizeof(k_presets[0])))

/*
 * Density -- the 2-D histogram of chord crossings.
 *
 * WHY this exists (the rendering technique it enables):
 *   The Maurer rose is N_SAMPLES chords overlaid in the same drawable
 *   region.  Naive per-thread overdraw ("last write wins") would
 *   discard the crossing-count information -- you would see only the
 *   last thread to touch each cell.  Density rendering REPLACES that
 *   with accumulation:
 *
 *     stamp pass:   for each chord, increment every cell along it
 *     render pass:  map each cell's count to brightness tier + glyph
 *
 *   The envelope of the chord family emerges naturally because envelope
 *   cells get many crossings (high tier, hot colour); sparse interior
 *   weaves get few (low tier, cool colour).  Same idea as a graphics
 *   accumulation buffer (Akenine-Möller [8] §23), the heat-map family
 *   of visualisations, and a 2-D histogram with float-valued bins.
 *
 * Invariants:
 *   I1.  cells[r][c] is the number of distinct chord segments that
 *        passed through cell (r, c) during the most recent stamp pass.
 *        Cleared to 0 at the start of every scene_build_density.
 *   I2.  peak == max(cells[r][c]) over all (r, c) in the stamped region.
 *        Maintained INCREMENTALLY by density_stamp_line so the renderer
 *        does not need a second pass to find the max.
 *   I3.  All writes are clipped to the drawing band:
 *           HUD_TOP_ROWS ≤ r < rows - HUD_BOTTOM_ROWS
 *           0 ≤ c < cols
 *        The HUD bands are never modified.
 *
 * Memory:
 *   ~MAX_ROWS · MAX_COLS · sizeof(int)  ≈ 160 KB at the default sizes.
 *   Lives inside Scene (and Scene inside the static g_app), so no heap
 *   allocation, no per-frame zeroing of allocated memory.
 *
 * Fields:
 *   cells  [HUD_TOP_ROWS, rows - HUD_BOTTOM_ROWS) × [0, cols)  ::
 *          stamp counter, 0 = unvisited.  Bounded above by N_SAMPLES
 *          (no chord touches a cell more than once per pass; in
 *          practice the peak is ~20-50 at envelope edges).
 *   peak   running max -- the normalisation denominator used by
 *          density_to_tier.
 */
typedef struct {
    int cells[MAX_ROWS][MAX_COLS];  /* per-cell stamp counter */
    int peak;                       /* max(cells); maintained by stamp_line */
} Density;

static void density_clear(Density *d)
{
    memset(d->cells, 0, sizeof d->cells);
    d->peak = 0;
}

/*
 * density_stamp_line -- Bresenham line accumulator (Bresenham 1965 [6]).
 *
 * Each cell touched along the line gets its count incremented; peak
 * is updated incrementally so we know the maximum without a second
 * scan after stamping all chords.  Clipped to the drawing band so the
 * HUD bands stay clean even if a chord endpoint sits on the edge.
 */
static void density_stamp_line(Density *d,
                               int x0, int y0, int x1, int y1,
                               int rows, int cols)
{
    int dx =  abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
    int dy = -abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
    int err = dx + dy;

    for (;;) {
        if ((unsigned)x0 < (unsigned)cols && x0 < MAX_COLS &&
            y0 >= HUD_TOP_ROWS && y0 < rows - HUD_BOTTOM_ROWS && y0 < MAX_ROWS) {
            int v = ++d->cells[y0][x0];
            if (v > d->peak) d->peak = v;
        }
        if (x0 == x1 && y0 == y1) break;
        int e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

/*
 * density_to_tier -- map raw density count to brightness tier.
 *
 *   tier = round( sqrt(d / peak) · (N_TIERS - 1) )
 *
 * Returns -1 for cells with no crossings (caller should skip).  The
 * sqrt curve keeps single-crossing cells visible without saturating
 * envelope-edge clusters.
 */
static inline int density_to_tier(int d, int peak)
{
    if (d <= 0 || peak <= 0) return -1;
    float norm = (float)d / (float)peak;
    int   tier = (int)(sqrtf(norm) * (float)(N_TIERS - 1) + 0.5f);
    if (tier < 0)         tier = 0;
    if (tier >= N_TIERS)  tier = N_TIERS - 1;
    return tier;
}

/* ASCII intensity ramp for the N_TIERS brightness levels.
 * Order chosen so visual weight grows monotonically across tiers:
 *   '.' → ',' → ':' → ';' → '+' → '*' → '#' → '@'.
 * (1 glyph per tier, so each density tier gets its own glyph AND its
 * own palette colour for a doubly-encoded depth signal.) */
static inline char tier_glyph(int tier)
{
    static const char ramp[N_TIERS] = {
        '.', ',', ':', ';', '+', '*', '#', '@'
    };
    return ramp[tier];
}

/* A_BOLD on the brightest tier so envelope edges "glow". */
static inline chtype tier_attr(int tier)
{
    return (tier == BRIGHT_TIER) ? A_BOLD : 0;
}

/* The continuous rose point at angle θ (radians), in unit space. */
static void rose_point(float n, float theta_rad, float *x, float *y)
{
    float r = cosf(n * theta_rad);
    *x = r * cosf(theta_rad);
    *y = r * sinf(theta_rad);
}

/*
 * maurer_step_index_to_theta -- the angle θ_k for the k-th Maurer sample.
 *   θ_k = k · d  (degrees, converted to radians via DEG_TO_RAD)
 * Indexing starts at 1 to match Maurer's original paper [1].
 */
static inline float maurer_step_index_to_theta(int k, float d_deg)
{
    return (float)k * d_deg * DEG_TO_RAD;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene -- the entire simulation state.
 *
 * The state vector of the visualiser (Strogatz [5] §8 framing): the
 * minimum set of values needed to render the next frame.  Everything
 * else (HUD strings, ncurses cells) is derived from these.
 *
 * Composition decisions:
 *
 *   - `params` is its own struct because (value, speed) co-vary
 *     per parameter -- see RoseParams doc.
 *   - `density` and `frame` are derived data living on Scene rather
 *     than in helper structs because their lifetime matches Scene
 *     (rebuilt every frame for density, every resize for frame).
 *   - `preset_idx` and `theme_idx` are flat ints (indices into the
 *     const-static tables k_presets[] / k_themes[]) -- one-arithmetic-
 *     bump for navigation, no pointer swizzling.
 *   - `paused` is a flat bool; only one tick guard, doesn't earn its
 *     own substruct.
 *
 * Memory:
 *   density is the heavy field (~160 KB).  Scene therefore lives inside
 *   the static g_app instance (§8); never on the stack, never allocated.
 *
 * Fields (range / unit / mutator):
 *   params         RoseParams  -- the drifting (n, d) pair.
 *                                  Mutator: scene_tick, action_reset,
 *                                  action_preset_*.
 *   preset_idx     [0, N_PRESETS)
 *                              -- last preset that was snapped to.
 *                                 Mutator: action_preset_next/prev.
 *   speed_scale    [DRIFT_SPEED_MIN_F, DRIFT_SPEED_MAX_F]
 *                              -- global multiplier on n_speed/d_speed.
 *                                 1.0 = default, geometric step on +/-.
 *                                 Mutator: action_speed_faster/slower.
 *   theme_idx      [0, N_THEMES)
 *                              -- active theme.  Mutator: theme_next/prev
 *                                 (also calls theme_apply to load palette).
 *   paused         bool       -- SPACE freezes scene_tick.  Rendering
 *                                 still runs so the rose stays on screen.
 *                                 Mutator: action_pause.
 *   density        Density    -- per-frame chord histogram, rebuilt by
 *                                 scene_build_density.  Read-only after
 *                                 build during render passes.
 *   frame          RenderFrame -- per-layout transform.  Rebuilt by
 *                                  render_frame_make on resize.
 */
typedef struct {
    RoseParams  params;       /* drifting (n, d) + per-param speeds       */
    int         preset_idx;   /* [0, N_PRESETS)  last snapped preset      */
    float       speed_scale;  /* global drift multiplier (+/- keys)       */
    int         theme_idx;    /* [0, N_THEMES)   active theme             */
    bool        paused;       /* SPACE toggles -- freezes scene_tick      */
    Density     density;      /* per-frame chord histogram                */
    RenderFrame frame;        /* unit→pixel transform for this layout    */
} Scene;

/* ---- parameter handling --------------------------------------------- */

/* Apply a RosePreset's (n, d) to the scene, leaving speeds alone. */
static void scene_apply_preset(Scene *s, int idx)
{
    s->preset_idx = idx;
    s->params.n   = k_presets[idx].n;
    s->params.d   = k_presets[idx].d;
}

static void scene_init(Scene *s, int rows, int cols)
{
    memset(s, 0, sizeof *s);
    s->params.n_speed  = N_SPEED_DEFAULT;
    s->params.d_speed  = D_SPEED_DEFAULT;
    s->speed_scale     = 1.0f;
    s->theme_idx       = 0;
    s->paused          = false;
    s->frame           = render_frame_make(rows, cols);
    scene_apply_preset(s, DEFAULT_PRESET_IDX);
}

/* Wrap x back into [lo, hi). */
static float wrap_into(float x, float lo, float hi)
{
    float range = hi - lo;
    while (x >= hi) x -= range;
    while (x <  lo) x += range;
    return x;
}

/*
 * Dwell envelope for a single drifting parameter.
 *
 *   dist  = |x - round(x)|              in [0, 0.5]
 *   mult  = DWELL_FLOOR + DWELL_RAMP · (2·dist)²
 *
 * mult bottoms out near DWELL_FLOOR at integer x and rises smoothly to
 * 1.0 around x ± 0.5.  Same shape that makes lissajous's named figures
 * "sit" on screen long enough to study.
 */
static float dwell_envelope(float x)
{
    float dist = fabsf(x - roundf(x));
    float mult = DWELL_FLOOR + DWELL_RAMP * (dist * dist * 4.0f);
    return (mult > 1.0f) ? 1.0f : mult;
}

/*
 * Combined dwell for the (n, d) pair.  Slowdown happens only when BOTH
 * n and d are near integers -- the named Maurer rose configurations.
 * If only one of (n, d) is integer, the rose is morphing in a
 * non-landmark direction, so drift full speed.
 */
static float pair_dwell(float n, float d)
{
    float mn = dwell_envelope(n);
    float md = dwell_envelope(d);
    return (mn > md) ? mn : md;
}

/*
 * scene_tick -- one simulation step.
 *
 * Reads as pseudocode:
 *   if paused: do nothing
 *   else:
 *     mult = pair_dwell(n, d)             -- both near integer => slow
 *     n   += n_speed * mult * speed_scale -- wrap into [N_MIN, N_MAX)
 *     d   += d_speed * mult * speed_scale -- wrap into [D_MIN, D_MAX)
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;
    float mult = pair_dwell(s->params.n, s->params.d);
    float gain = mult * s->speed_scale;
    s->params.n = wrap_into(s->params.n + s->params.n_speed * gain, N_MIN, N_MAX);
    s->params.d = wrap_into(s->params.d + s->params.d_speed * gain, D_MIN, D_MAX);
}

/* ---- chord generation ---------------------------------------------- */

/* Cell-space integer position of the k-th Maurer sample. */
static void scene_sample_cell(const Scene *s, int k, int *out_col, int *out_row)
{
    float theta = maurer_step_index_to_theta(k, s->params.d);
    float x_u, y_u;
    rose_point(s->params.n, theta, &x_u, &y_u);
    *out_col = cell_round(s->frame.cx_px + x_u * s->frame.scale_px) / CELL_W;
    *out_row = cell_round(s->frame.cy_px + y_u * s->frame.scale_px) / CELL_H;
}

/* Rasterise every chord P_{k-1} → P_k into the density buffer. */
static void scene_build_density(Scene *s, int rows, int cols)
{
    density_clear(&s->density);

    int prev_col, prev_row;
    scene_sample_cell(s, 0, &prev_col, &prev_row);

    for (int k = 1; k <= N_SAMPLES; k++) {
        int col, row;
        scene_sample_cell(s, k, &col, &row);
        density_stamp_line(&s->density, prev_col, prev_row, col, row,
                           rows, cols);
        prev_col = col;
        prev_row = row;
    }
}

/* ---- render layers ------------------------------------------------- */

/* Cosmetic centre marker so the viewer can see the rose's origin. */
static void render_centre(const RenderFrame *rf, int rows, int cols)
{
    int cx = cell_round(rf->cx_px) / CELL_W;
    int cy = cell_round(rf->cy_px) / CELL_H;
    if ((unsigned)cx >= (unsigned)cols || cx >= MAX_COLS) return;
    if (cy < HUD_TOP_ROWS || cy >= rows - HUD_BOTTOM_ROWS) return;
    if (cy >= MAX_ROWS) return;
    attron(COLOR_PAIR(CP_CENTRE) | A_DIM);
    mvaddch(cy, cx, '+');
    attroff(COLOR_PAIR(CP_CENTRE) | A_DIM);
}

/*
 * paint_density_cell -- render one cell of the density heat map.
 *
 * Skips cells with no crossings (tier == -1).  Otherwise looks up the
 * tier's glyph and colour-pair, ORs in the tier-specific attribute
 * (A_BOLD on the hottest tier), and writes the cell.  All three lookup
 * helpers (tier_glyph, tier_attr, CP_TIER_0 + tier) live in §5; this
 * function is just the composition site.
 */
static void paint_density_cell(int row, int col, int density, int peak)
{
    int tier = density_to_tier(density, peak);
    if (tier < 0) return;
    chtype attr = COLOR_PAIR(CP_TIER_0 + tier) | tier_attr(tier);
    attron(attr);
    mvaddch(row, col, (chtype)(unsigned char)tier_glyph(tier));
    attroff(attr);
}

/* Scan the drawing band and paint every non-empty density cell. */
static void render_density(const Density *d, int rows, int cols)
{
    int row_end = rows - HUD_BOTTOM_ROWS;
    if (row_end > MAX_ROWS) row_end = MAX_ROWS;
    int col_end = (cols < MAX_COLS) ? cols : MAX_COLS;

    for (int y = HUD_TOP_ROWS; y < row_end; y++)
        for (int x = 0; x < col_end; x++)
            paint_density_cell(y, x, d->cells[y][x], d->peak);
}

static void scene_draw(Scene *s, int rows, int cols)
{
    scene_build_density(s, rows, cols);
    render_centre  (&s->frame, rows, cols);
    render_density (&s->density, rows, cols);
}

/* Name of the preset closest to the current (n, d) — used in the HUD.
 * Distance metric: Euclidean in (n, d) space, weighted to put n on the
 * same scale as d (n range is ~7, d range is ~176). */
static const char *scene_nearest_preset_name(const Scene *s)
{
    float best_dist  = 1e9f;
    int   best_idx   = 0;
    for (int i = 0; i < N_PRESETS; i++) {
        float dn = (s->params.n - k_presets[i].n) * (D_MAX / N_MAX);
        float dd =  s->params.d - k_presets[i].d;
        float d2 = dn * dn + dd * dd;
        if (d2 < best_dist) { best_dist = d2; best_idx = i; }
    }
    return k_presets[best_idx].name;
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
 * Capacity clamps:
 *   Values are clamped to MAX_ROWS / MAX_COLS at every read (via
 *   screen_clamp) so the Density buffer never sees a row/col out of
 *   its static allocation.  This means very large terminals see a
 *   centred rose with empty margin rather than a corrupted buffer.
 *
 * Update points:
 *   screen_init    -- once at startup
 *   screen_resize  -- on each SIGWINCH (handled in §8 apply_resize)
 *
 * Fields are in ncurses convention: cols = X-extent, rows = Y-extent,
 * both 1-based counts so valid coordinates are
 *      [0, cols-1] × [0, rows-1].
 * The HUD bands reserve HUD_TOP_ROWS at the top and HUD_BOTTOM_ROWS at
 * the bottom; the rose renderer stays inside the middle band.
 */
typedef struct {
    int cols;   /* terminal width  in character cells   */
    int rows;   /* terminal height in character cells   */
} Screen;

static void screen_clamp(Screen *s)
{
    getmaxyx(stdscr, s->rows, s->cols);
    if (s->rows > MAX_ROWS) s->rows = MAX_ROWS;
    if (s->cols > MAX_COLS) s->cols = MAX_COLS;
}

static void screen_init(Screen *s)
{
    initscr();
    cbreak();
    noecho();
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    curs_set(0);
    typeahead(-1);
    color_init();
    screen_clamp(s);
}

static void screen_resize(Screen *s, Scene *sc)
{
    endwin(); refresh();
    screen_clamp(s);
    sc->frame = render_frame_make(s->rows, s->cols);
    erase();
}

/* ---- HUD row helpers ------------------------------------------------- */

/* Row 0 left -- title. */
static void hud_draw_title(void)
{
    attron(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
    mvprintw(0, 0, " [MAURER ROSE] ");
    attroff(COLOR_PAIR(HUD_PAIR_TITLE) | A_BOLD);
}

/* Row 0 middle -- live state: theme, n, d, nearest preset, speed. */
static void hud_draw_state(const Scene *s)
{
    attron(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
    mvprintw(0, HUD_DATA_COL,
             "  t:%-9s  n=%4.2f  d=%5.1f\xC2\xB0  [%-11s]  spd=%.2f\xC3\x97 ",
             k_themes[s->theme_idx].name,
             (double)s->params.n,
             (double)s->params.d,
             scene_nearest_preset_name(s),
             (double)s->speed_scale);
    attroff(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
}
/*  Note: ° (\xC2\xB0) and × (\xC3\x97) are UTF-8 sequences for the
 *  degree sign and multiplication sign.  Terminals overwhelmingly
 *  handle UTF-8 by default; these are display-only and never enter
 *  the density buffer.  Comment dividers and HUD glyphs are exempt
 *  from the ASCII-only rendering rule that applies to the canvas. */

/* Row 0 right -- engine stats. */
static void hud_draw_engine_stats(const Screen *s, double fps, bool paused)
{
    char buf[64];
    snprintf(buf, sizeof buf, " %5.1f fps  %s ",
             fps, paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    if (len >= s->cols) return;
    attron(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
    mvprintw(0, s->cols - len, "%s", buf);
    attroff(COLOR_PAIR(HUD_PAIR_DATA) | A_BOLD);
}

/* Row rows-1 -- action keys, bright cyan + A_BOLD. */
static void hud_draw_action_bar(const Screen *s)
{
    attron(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
    mvprintw(s->rows - 1, 0,
             " q:quit   SPACE:pause   r:reset   t:theme   [/]:preset   +/-:speed ");
    attroff(COLOR_PAIR(HUD_PAIR_HINT) | A_BOLD);
}

static void screen_draw_hud(const Screen *s, const Scene *sc, double fps)
{
    hud_draw_title();
    hud_draw_state(sc);
    hud_draw_engine_stats(s, fps, sc->paused);
    hud_draw_action_bar(s);
}

static void screen_draw(Screen *s, Scene *sc, double fps)
{
    erase();
    scene_draw(sc, s->rows, s->cols);
    screen_draw_hud(s, sc, fps);
    wnoutrefresh(stdscr);
    doupdate();
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
 *   Maurer-rose-math concerns.  Keeping them off Scene means the
 *   simulation stays PURE STATE -- snapshottable, replayable from a
 *   single (n, d, theme_idx) triple, testable in isolation.
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
 *   scene        -- the simulation state (Scene, §6).  Heavy due to
 *                   the Density buffer; stays in BSS via g_app.
 *   screen       -- cached terminal geometry (Screen, §7).
 *   running      -- 0 ⇒ main loop should exit (SIGINT/SIGTERM signal
 *                   handler clears it; main checks each iteration).
 *   need_resize  -- 1 ⇒ main loop should re-init screen + frame
 *                   (SIGWINCH signal handler sets it; main consumes
 *                   it in apply_resize).
 */
typedef struct {
    Scene                 scene;        /* simulation state              */
    Screen                screen;       /* terminal size cache           */
    volatile sig_atomic_t running;      /* 0 = exit main loop  (SIGINT)  */
    volatile sig_atomic_t need_resize;  /* 1 = re-init screen  (SIGWINCH)*/
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
    scene_init(&app->scene, app->screen.rows, app->screen.cols);
}

/* SIGWINCH path: re-query size, recompute RenderFrame, clear canvas. */
static void apply_resize(App *app)
{
    screen_resize(&app->screen, &app->scene);
    app->need_resize = 0;
}

/* ---- key actions (each one a named helper for clarity) -------------- */

static void action_pause          (Scene *s) { s->paused = !s->paused; }
static void action_reset          (Scene *s)
{
    s->params.n = N_DEFAULT;
    s->params.d = D_DEFAULT;
}
static void action_theme_next     (Scene *s)
{
    s->theme_idx = theme_next(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_theme_prev     (Scene *s)
{
    s->theme_idx = theme_prev(s->theme_idx);
    theme_apply(s->theme_idx);
}
static void action_preset_next    (Scene *s)
{
    scene_apply_preset(s, (s->preset_idx + 1) % N_PRESETS);
}
static void action_preset_prev    (Scene *s)
{
    scene_apply_preset(s, (s->preset_idx + N_PRESETS - 1) % N_PRESETS);
}
static void action_speed_faster   (Scene *s)
{
    s->speed_scale *= DRIFT_SPEED_STEP;
    if (s->speed_scale > DRIFT_SPEED_MAX_F) s->speed_scale = DRIFT_SPEED_MAX_F;
}
static void action_speed_slower   (Scene *s)
{
    s->speed_scale /= DRIFT_SPEED_STEP;
    if (s->speed_scale < DRIFT_SPEED_MIN_F) s->speed_scale = DRIFT_SPEED_MIN_F;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC: return false;
    case ' ':                         action_pause(s);        break;
    case 'r': case 'R':               action_reset(s);        break;
    case 't':                         action_theme_next(s);   break;
    case 'T':                         action_theme_prev(s);   break;
    case ']':                         action_preset_next(s);  break;
    case '[':                         action_preset_prev(s);  break;
    case '+': case '=':               action_speed_faster(s); break;
    case '-': case '_':               action_speed_slower(s); break;
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

/* Rolling FPS over FPS_UPDATE_MS windows. */
static void fps_counter_update(int64_t now, int64_t *window_start,
                               int *frame_count, double *fps_out)
{
    (*frame_count)++;
    int64_t elapsed = now - *window_start;
    if (elapsed < FPS_UPDATE_MS * NS_PER_MS) return;
    *fps_out      = (double)(*frame_count) / ((double)elapsed / (double)NS_PER_SEC);
    *frame_count  = 0;
    *window_start = now;
}

/* Sleep until wall-clock 'deadline'; return the next deadline. */
static int64_t pace_to_deadline(int64_t deadline)
{
    deadline += TICK_NS;
    clock_sleep_ns(deadline - clock_ns());
    return deadline;
}

/* Compose one frame: tick the simulation, paint everything. */
static void frame_render(App *app, double fps)
{
    scene_tick(&app->scene);
    screen_draw(&app->screen, &app->scene, fps);
}

/*
 * main() -- the orchestrator.  Each line is one named phase of a frame.
 */
int main(void)
{
    install_signal_handlers();

    App *app = &g_app;
    app_init(app);

    int64_t next_deadline = clock_ns();
    int64_t fps_window    = clock_ns();
    int     frame_count   = 0;
    double  fps           = 0.0;

    while (app->running) {
        if (app->need_resize)  apply_resize(app);
        if (!drain_input(app)) { app->running = 0; break; }

        frame_render(app, fps);
        fps_counter_update(clock_ns(), &fps_window, &frame_count, &fps);
        next_deadline = pace_to_deadline(next_deadline);
    }
    return 0;
}
