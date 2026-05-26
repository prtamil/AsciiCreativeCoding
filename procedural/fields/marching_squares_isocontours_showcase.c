/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * marching_squares_isocontours_showcase.c
 *   — Extract iso-contours from a scalar field using marching squares.
 *     The 2-D ancestor of marching cubes. 30 visualisations explore
 *     single iso-lines, multi-threshold topographic maps, case-aware
 *     topology highlights, region fills, and time-varying thresholds.
 *
 * DEMO: Take any scalar field (here: a small Perlin/value-noise stack
 *       drifting in time). Pick a threshold T. For every 2×2 grid of
 *       samples, classify each corner as ABOVE or BELOW T → 4-bit
 *       case index ∈ {0..15}. A lookup table tells you which edge(s)
 *       of the cell the iso-line crosses; linear interpolation along
 *       each edge gives the actual crossing points. Connect them →
 *       the iso-contour.
 *
 *       30 patterns in 6 tiers (cycle with n / p):
 *         Tier 1 SINGLE   — basic single-iso renderings (CONTOUR,
 *                           THICK, INVERT, DOTS, UNIFORM)
 *         Tier 2 MULTI    — multiple evenly-spaced thresholds
 *                           (TOPO, DENSE, SPARSE, STEPS, RAINBOW)
 *         Tier 3 PAIRS    — specific T combinations (DUAL, SHELL,
 *                           INNER, OUTER, SANDWICH)
 *         Tier 4 TOPOLOGY — case-aware masks: only certain MS cases
 *                           drawn (SADDLE, ORTHO, DIAGONALS,
 *                           JUNCTIONS, ROUGHNESS)
 *         Tier 5 REGIONS  — filled-region visualisations (ABOVE,
 *                           BELOW, STRIPE, ZEBRA, PERIMETER)
 *         Tier 6 ANIMATED — time-varying thresholds (SWEEP, PULSE,
 *                           RIPPLE, STAIRS, CHAOS)
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — the source field we contour over.
 *   ./worley_cellular_noise.c — another field type you could feed
 *       into this iso-contour extractor.
 *   ./value_noise_showcase.c — the simplest noise; gives blocky
 *       contours with visible lattice artefacts.
 *
 * Section map:
 *   §1 config   — grid bounds, T constants, themes, HUD widths,
 *                 sim/render budgets, fBm octave parameters
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD pairs + 10 themes (PAIR_BAND_BASE+0..3)
 *   §5 field    — ScalarField (3-octave fBm value noise)
 *   §6 ms       — marching-squares classifier (4-bit case index)
 *   §7 patterns — 30 contour visualisations + dispatch table
 *   §8 scene    — ScalarField + ContourGrid + Pattern/Palette state;
 *                 per-frame tick + evaluate
 *   §9 screen   — viewport + ContourGrid renderer + HUD layout
 *   §10 app     — signals, resize, key dispatch, main game loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume animation
 *   r          reset (new noise seed)
 *   n / N      next pattern  (p / P previous)
 *   t / T      next / previous theme
 *   > / <      raise / lower user threshold T  (. / , as easier-typed aliases)
 *   + / =      faster drift (× 2)
 *   -          slower drift (/ 2)
 *   ] / [      raise / lower simulation tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *       marching_squares_isocontours_showcase.c -o marching_squares \
 *       -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Marching squares. Given a 2-D scalar field f(x, y)
 *                  sampled on a grid and an iso-threshold T, the
 *                  algorithm:
 *                    1. For each 2×2 block of corner samples, classify
 *                       each corner as 1 (≥ T) or 0 (< T).
 *                    2. Pack the 4 bits into a 4-bit case ∈ {0..15}.
 *                    3. Look up the case in a table that describes
 *                       which cell edges the iso-line crosses
 *                       (0, 1, or 2 edges).
 *                    4. For each crossing edge, linear-interpolate
 *                       the exact crossing position along the edge:
 *                       t = (T − f_lo) / (f_hi − f_lo).
 *                    5. Connect the crossing points → a line segment
 *                       (or two, in ambiguous cases 5 and 10).
 *
 *                  Cases 5 (0101) and 10 (1010) are SADDLES:
 *                  diagonally-opposite corners are above T. The
 *                  algorithm has to choose between two valid pairings
 *                  of the four edge crossings — sample f at the cell
 *                  centre to break the tie consistently.
 *
 *                  Output: a set of line segments approximating the
 *                  contour curve {f(x, y) = T}.
 *
 * Data-structure : Just the source scalar field (a Field of floats)
 *                  plus a 16-entry case → glyph lookup table for the
 *                  ASCII rendering. No persistent contour storage —
 *                  we rebuild it every frame.
 *
 * Rendering      : The "natural" marching-squares output is a set of
 *                  line segments. For ASCII we use a case → glyph
 *                  table that picks the best single character for
 *                  the line direction implied by the case:
 *                    cases 1, 14 (one corner above)     → '/' or '\'
 *                    cases 2, 13                        → '\' or '/'
 *                    cases 3, 12 (horizontal split)     → '-'
 *                    cases 6, 9  (vertical split)       → '|'
 *                    cases 5, 10 (saddle - ambiguous)   → 'X'
 *                    cases 7, 8  / 4, 11 (corners)      → '+' or '*'
 *                    cases 0, 15 (entirely below/above) → empty
 *                  TOPO and DUAL modes overlay multiple contour sets,
 *                  colouring by which threshold the contour belongs to.
 *
 * Performance    : O(W · H) per frame — one case classification + one
 *                  glyph lookup per cell. The source-field sampling
 *                  dominates: 4 corner samples per cell, each a
 *                  ~20-op noise function = ~80 noise ops/cell.
 *                  On an 11K-cell grid at 60 Hz that's ~5 M ops/sec
 *                  — completely trivial.
 *
 * References     : MARCHING SQUARES / CUBES
 *                  • Lorensen, W. E. & Cline, H. E. (1987) —
 *                    "Marching Cubes: A High Resolution 3D Surface
 *                    Construction Algorithm", SIGGRAPH'87. The
 *                    foundational paper; marching squares is the
 *                    direct 2-D simplification.
 *                  • Nielson, G. M. & Hamann, B. (1991) — "The
 *                    Asymptotic Decider: Resolving the Ambiguity in
 *                    Marching Cubes", IEEE Visualization '91. THE
 *                    reference on the saddle-case ambiguity (cases
 *                    5 and 10); directly underpins the SADDLE
 *                    pattern in §7.
 *                  • Newman, T. S. & Yi, H. (2006) — "A survey of
 *                    the marching cubes algorithm", Computers &
 *                    Graphics 30(5). Comprehensive review of
 *                    variants, ambiguity resolution, and
 *                    topology-preserving extensions.
 *                  • Maple, C. (2003) — "Geometric design and space
 *                    planning using the marching squares and
 *                    marching cube algorithms", IV'03. Practical
 *                    coverage of edge cases including saddle
 *                    disambiguation.
 *                  • Bourke, P. — "Polygonising a scalar field":
 *                    http://paulbourke.net/geometry/polygonise/
 *                    The most-cited practical reference — all 256
 *                    MC cases drawn out; the 16-entry MS table here
 *                    is the 2-D projection of the same construction.
 *                  • Wikipedia — "Marching squares":
 *                    https://en.wikipedia.org/wiki/Marching_squares
 *
 *                  BOOKS
 *                  • Schroeder, W., Martin, K. & Lorensen, B. —
 *                    "The Visualization Toolkit" (4th ed., Kitware,
 *                    2006). The iso-contouring chapter is the
 *                    definitive textbook treatment; co-authored by
 *                    the original Marching Cubes author.
 *
 *                  PROCEDURAL SOURCE FIELD (§5 ScalarField)
 *                  • Perlin, K. (1985) — "An Image Synthesizer",
 *                    SIGGRAPH'85.  Foundational noise paper.  The
 *                    3-octave fBm value-noise stack feeding §5 is a
 *                    simpler hash-lattice variant of Perlin's
 *                    gradient noise — the only "source" the
 *                    contour extractor sees.
 *                  • Quilez, I. — "2D distance functions" — useful
 *                    alternative scalar source: many of the example
 *                    shapes ARE iso-contours of distance fields:
 *                    https://iquilezles.org/articles/distfunctions2d/
 *
 *                  ASCII RENDERING
 *                  • Bourke, P. — "Character representation of grey
 *                    scale images":
 *                    http://paulbourke.net/dataformats/asciiart/
 *
 *                  COMPARE IN PROJECT
 *                  • ./perin_noise_flow_showcase.c — the source field
 *                    we contour over.
 *                  • ./signed_distance_field_jfa_showcase.c — the
 *                    OUTLINE pattern there is a "soft" alternative
 *                    iso-line via SDF thresholding, no marching
 *                    squares needed.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * "Where does this scalar field equal T?" If you have function
 * values at every grid point, the iso-line {f = T} must cross
 * certain CELL EDGES — specifically, the edges where one corner is
 * above T and the other is below. Look at all 4 corners of one
 * 2×2 cell as a 4-bit number and look up the geometry of the cross-
 * ings in a 16-entry table.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine flooding the field with water up to height T. Land sticks
 * up above the water, sea floor stays below. Marching squares
 * draws the COASTLINE — the boundary between wet and dry. For each
 * 2×2 patch of cells, the corners tell you how many neighbours are
 * wet vs dry, which sub-edges the coast crosses, and (via linear
 * interp) WHERE along each edge it crosses.
 *
 * 16 cases × 1 cell type = the entire algorithm.
 *
 * AMBIGUOUS SADDLES
 *   Cases 5 (corners 0 and 2 above, 1 and 3 below) and 10 (corners
 *   1 and 3 above, 0 and 2 below) have two valid contour pairings:
 *   you could connect the four crossing points as two arcs going
 *   ROUND each above-corner, OR as two arcs going BETWEEN the two
 *   above-corners. Both are topologically consistent. To pick one,
 *   sample f at the cell centre — whichever pair its sign agrees
 *   with becomes the connection. See the SADDLE pattern.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Sample the source field at the 4 corners of every 2×2 cell.
 *  2. For each cell:
 *       a. Build the case bitmask:
 *            bit 0 ← (corner BL ≥ T)
 *            bit 1 ← (corner BR ≥ T)
 *            bit 2 ← (corner TR ≥ T)
 *            bit 3 ← (corner TL ≥ T)
 *          case_index ∈ {0..15}
 *       b. Look up case_glyph[case_index] → the ASCII character that
 *          best represents the contour passing through this cell.
 *       c. For SADDLE cases (5, 10), optionally disambiguate by
 *          sampling the cell centre.
 *  3. Render the glyph (or blank for cases 0/15 = entirely below/
 *     above threshold).
 *  4. TOPO / DUAL modes loop over multiple thresholds and overlay
 *     the results, colouring by threshold index.
 *
 * KEY FORMULAS
 * ────────────
 *  case_index            : 4-bit packing of (corner ≥ T)
 *  Edge crossing position: t = (T − f_lo) / (f_hi − f_lo)    along the
 *                          edge from f_lo to f_hi
 *  Saddle disambiguation : sample f(centre); centre's relation to T
 *                          picks one of the two pairings
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

/* HUD layout — top carries data, bottom carries actions. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* HUD row-1 column widths — MUST match the printf format-string widths
 * the corresponding hud_field_* helpers emit, otherwise consecutive
 * fields overlap or leave gaps. */
#define HUD_PATTERN_FIELD_W     21
#define HUD_TIER_FIELD_W        15
#define HUD_THEME_FIELD_W       17
#define HUD_PALETTE_LABEL_W      9
#define HUD_N_PALETTE_BANDS      4

/* Viewport-derived grid floor: the algorithm needs at least one
 * MS cell, but anything tinier than this is unreadable. */
#define MAP_W_MIN               16
#define MAP_H_MIN                8

/* Render-loop budget — sleep target for the per-frame throttle. */
#define RENDER_FPS_TARGET       60
#define RENDER_FRAME_BUDGET_NS  (NS_PER_SEC / RENDER_FPS_TARGET)

/* Spiral-of-death guard for the fixed-timestep accumulator: cap any
 * single frame's dt so a slow terminal can't trigger unbounded tick
 * catch-up.  See Fiedler "Fix Your Timestep!" (cited on App, §10). */
#define SIM_MAX_FRAME_DT_MS    100

/* Source-field scale: smaller → bigger smooth blobs, coarser contours. */
#define FIELD_SCALE         0.08f

/* Time drift on the source field — slow so the topology of the
 * contour can be tracked by eye. */
#define FIELD_DRIFT         0.20f

/* fBm value-noise stack — 3 octaves, classic Mandelbrot fBm shape.
 * Each successive octave doubles in frequency (lacunarity = 2) and
 * carries less amplitude.  Amps sum to 1.0 so the result stays in
 * [0, 1] without an explicit normalisation step (the clamp01 in
 * fbm_3oct is defensive against numeric drift, not normalisation). */
#define FBM_LACUNARITY      2.0f
#define FBM_OCT0_FREQ       1.0f
#define FBM_OCT1_FREQ       (FBM_LACUNARITY)
#define FBM_OCT2_FREQ       (FBM_LACUNARITY * FBM_LACUNARITY)
#define FBM_OCT0_AMP        0.5f
#define FBM_OCT1_AMP        0.3f
#define FBM_OCT2_AMP        0.2f

/* Drift multiplier — cranked by +/-, capped low. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      16

/* CONTOUR / SADDLE — default + step for user-adjustable threshold. */
#define CONTOUR_T_DEFAULT   0.50f
#define CONTOUR_T_STEP      0.05f
#define CONTOUR_T_MIN       0.05f
#define CONTOUR_T_MAX       0.95f

/* Tier 1 — THICK pattern: half-width of the fat-contour band around T. */
#define THICK_BAND_HALF     0.04f

/* Tier 2 — multi-threshold variants: how many evenly-spaced levels. */
#define TOPO_N_LEVELS       8
#define DENSE_N_LEVELS     16
#define SPARSE_N_LEVELS     4
#define STEPS_N_LEVELS      6
#define RAINBOW_N_LEVELS   12

/* Tier 3 — fixed-T combinations. */
#define DUAL_T_LOW          0.35f
#define DUAL_T_HIGH         0.65f
#define SHELL_T_LOW         0.46f
#define SHELL_T_HIGH        0.54f
#define INNER_T             0.70f
#define OUTER_T             0.30f
#define SANDWICH_T_LOW      0.30f
#define SANDWICH_T_MID     0.50f
#define SANDWICH_T_HIGH     0.70f

/* Tier 5 — region fills. */
#define STRIPE_HALFWIDTH    0.07f
#define ZEBRA_N_BANDS       8

/* Tier 6 — animated thresholds. */
#define SWEEP_T_PERIOD_SEC  6.0f
#define PULSE_PERIOD_SEC    3.0f
#define RIPPLE_AMP          0.10f
#define RIPPLE_FREQ_CELLS  10.0f
#define RIPPLE_SPEED        2.0f
#define STAIRS_N_LEVELS     5
#define STAIRS_PERIOD_SEC   8.0f
#define CHAOS_AMP           0.10f

/*
 * Pattern — 30 marching-squares visualisations in 6 complexity tiers.
 * Cycle with n/p. Enum order MUST match `noise_patterns[]` in §7
 * (compiler enforces via fixed-size [N_PATTERNS] initialiser).
 *
 *   Tier 1 SINGLE    : CONTOUR, THICK, INVERT, DOTS, UNIFORM
 *   Tier 2 MULTI     : TOPO, DENSE, SPARSE, STEPS, RAINBOW
 *   Tier 3 PAIRS     : DUAL, SHELL, INNER, OUTER, SANDWICH
 *   Tier 4 TOPOLOGY  : SADDLE, ORTHO, DIAGONALS, JUNCTIONS, ROUGHNESS
 *   Tier 5 REGIONS   : ABOVE, BELOW, STRIPE, ZEBRA, PERIMETER
 *   Tier 6 ANIMATED  : SWEEP, PULSE, RIPPLE, STAIRS, CHAOS
 */
typedef enum {
    /* Tier 1 — SINGLE: basic single-iso renderings */
    PATTERN_CONTOUR = 0,
    PATTERN_THICK,
    PATTERN_INVERT,
    PATTERN_DOTS,
    PATTERN_UNIFORM,
    /* Tier 2 — MULTI: evenly-spaced thresholds */
    PATTERN_TOPO,
    PATTERN_DENSE,
    PATTERN_SPARSE,
    PATTERN_STEPS,
    PATTERN_RAINBOW,
    /* Tier 3 — PAIRS: specific T combinations */
    PATTERN_DUAL,
    PATTERN_SHELL,
    PATTERN_INNER,
    PATTERN_OUTER,
    PATTERN_SANDWICH,
    /* Tier 4 — TOPOLOGY: case-aware masks */
    PATTERN_SADDLE,
    PATTERN_ORTHO,
    PATTERN_DIAGONALS,
    PATTERN_JUNCTIONS,
    PATTERN_ROUGHNESS,
    /* Tier 5 — REGIONS: filled visualisations */
    PATTERN_ABOVE,
    PATTERN_BELOW,
    PATTERN_STRIPE,
    PATTERN_ZEBRA,
    PATTERN_PERIMETER,
    /* Tier 6 — ANIMATED: time-varying thresholds */
    PATTERN_SWEEP,
    PATTERN_PULSE,
    PATTERN_RIPPLE,
    PATTERN_STAIRS,
    PATTERN_CHAOS,
    N_PATTERNS,
} Pattern;

/* pattern_name() / pattern_tier() defined in §7 alongside the
 * dispatch table. Forward-declared so the HUD code in §9 can call
 * them without seeing the table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

/*
 * Case → ASCII glyph lookup. Index = 4-bit case packing of
 * (BL, BR, TR, TL) ≥ T. The complement pairs (case k ↔ case 15-k)
 * have the SAME contour topology — only above/below swap — so they
 * share a glyph.
 *
 *   0  / 15 → ' '  (no contour: all below / all above)
 *   1  / 14 → '/'  (single BL corner above)         "
 *   2  / 13 → '\'  (single BR corner above)         "
 *   3  / 12 → '-'  (bottom edge horizontal split)
 *   4  / 11 → '\'  (single TR corner above)
 *   5  /  5 → 'X'  (saddle — ambiguous)
 *   6  /  9 → '|'  (right edge vertical split)
 *   7  /  8 → '/'  (single TL/BR diagonal)
 *   8  /  7 → '/'  (mirror)
 *   9  /  6 → '|'  (mirror)
 *  10  / 10 → 'X'  (saddle — ambiguous)
 *  11  /  4 → '\'  (mirror)
 *  12  /  3 → '-'  (mirror)
 *  13  /  2 → '\'  (mirror)
 *  14  /  1 → '/'  (mirror)
 *  15  /  0 → ' '
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

/* True for the two ambiguous saddle cases. SADDLE pattern uses this. */
static inline bool ms_case_is_saddle(int c) { return c == 5 || c == 10; }

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — a named 4-colour band ramp defining one visual style.
 *
 * INTENT
 *   Decouple "which colours" from "where they're used".  Pattern
 *   code in §7 emits a band index ∈ {0..3} per cell; theme_apply()
 *   in §3 binds that index to one of the four xterm-256 colours
 *   stored here.  Cycling themes (t/T) re-binds the ncurses pairs
 *   in place — no §6/§7/§8 code has to know about colour at all.
 *
 * CONTEXT
 *   themes[N_THEMES] in §1 is the static table of options.  The
 *   active selection lives in PaletteState.current (an index into
 *   themes[]).  theme_apply() pushes this row's colours into
 *   PAIR_BAND_BASE..+3; the renderer then reads those pairs via
 *   COLOR_PAIR(PAIR_BAND_BASE + band).
 *
 * PALETTE LOGIC
 *   The 4 band slots form a perceptual ramp, NOT arbitrary colours:
 *     band[0] — dimmest   : dim context strokes (DOTS, OUTER)
 *     band[1] — low       : OUTER contour, lower thresholds
 *     band[2] — mid       : the "primary" iso-line for most patterns
 *     band[3] — bright    : INNER contour, saddle highlights
 *   Every entry MUST sit ≥ index 24 in the xterm-256 cube so that
 *   ramp tier 0 stays legible against default-black with A_BOLD.
 *   (See CLAUDE.md § Theme Palette Brightness.)
 *
 * MEMBER LOGIC
 *   name    : ≤ 8-char ALL-CAPS label.  Stored as a literal pointer
 *             (no copy); HUD reads themes[palette.current].name
 *             directly.
 *   band[4] : xterm-256 colour indices for bands 0..3.  Picked as a
 *             monotone-brightness ramp; the RELATIVE gradient gives
 *             a theme its character, not absolute hue.
 *
 * REFERENCES
 *   • xterm-256 colour cube — indices 16..231 = 6×6×6 RGB cube,
 *     232..255 = 24-step grayscale ramp.
 *   • ncurses(3X) — start_color, init_pair, COLOR_PAIR.
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
 * ScalarField — the 2-D float grid we extract iso-contours from.
 *
 * INTENT
 *   Decouple "what we're contouring" from "how we contour it".  Any
 *   continuous scalar function f: ℝ² → ℝ could be plugged in here;
 *   marching squares does not care about the source.  For this
 *   showcase the values come from a small 3-octave fBm value-noise
 *   stack with time drift (smooth blobs that read pleasingly under
 *   any iso threshold), but a heightmap, an SDF, a fluid-simulation
 *   pressure field, or any other smooth scalar would all plug in
 *   without changes to §6/§7.
 *
 * CONTEXT
 *   One instance lives on Scene (§8).  Rebuilt every sim tick by
 *   scalar_field_rebuild() (advances the time axis); read by:
 *     • §6 ms_classify         — 4 corner samples per MS cell.
 *     • §7 region-test patterns — Tier 5 ABOVE/BELOW/STRIPE/ZEBRA
 *                                 sample a single corner per cell.
 *   Patterns receive a `const ScalarField *` so they cannot mutate
 *   it during the per-cell sweep.
 *
 * MEMORY
 *   samples[CELLS_MAX] ≈ 44 KB in BSS — no allocation.  At the
 *   default caps of MAP_W_MAX = 200 × MAP_H_MAX = 56 = 11 200
 *   cells × 4 bytes/float.
 *
 * MEMBER LOGIC
 *   w, h    : Grid dimensions in cells.  Set by scalar_field_reset()
 *             at scene reset; bounded by [16, MAP_W_MAX] × [8, MAP_H_MAX].
 *   count   : w · h, cached so the rebuild loop and the per-cell
 *             reads don't recompute the product.
 *   seed    : Per-run hash seed mixed into the lattice-corner hash.
 *             Re-rolled at every reset (r press) so each run
 *             produces a fresh field topology without coupling to
 *             rand() state (we don't want srand() side-effects here).
 *   samples : Per-cell scalar ∈ [0, 1] — fBm of (x · FIELD_SCALE,
 *             y · FIELD_SCALE + time) at lattice corner (x, y).
 *             Row-major: cell (x, y) = samples[y · w + x].  The MS
 *             classifier samples the 4 corners (x, y), (x+1, y),
 *             (x+1, y+1), (x, y+1) per cell — so each cell reads 4
 *             entries that overlap with its neighbours.
 *
 * REFERENCES
 *   • Perlin, K. (1985) — "An Image Synthesizer", SIGGRAPH'85.
 *     Foundational noise paper.  Value noise (used here) is the
 *     simpler cousin of gradient/Perlin noise: hash lattice corners,
 *     then bilerp with smoothstep'd interpolants.
 *   • Mandelbrot, B. & Van Ness, J. (1968) — "Fractional Brownian
 *     motions, fractional noises and applications", SIAM Review
 *     10(4).  The fBm octave-stacking model used by fbm_3oct().
 *   • Quilez, I. — "Value noise" / "More noise":
 *     https://iquilezles.org/articles/morenoise/
 *     Practical write-up of value-noise + fBm in shader form.
 *   • Wang, T. (1997) — "Integer hash function".  hash32() belongs
 *     to the same family of multiply-xor mixers used in value-noise
 *     lattices to avoid storing a permutation table.
 */
typedef struct {
    int      w, h;
    int      count;
    uint32_t seed;
    float    samples[CELLS_MAX];
} ScalarField;

/* Clamped per-cell read — out-of-bounds queries clamp to the nearest
 * edge cell.  Used by patterns (Tier 5 fills, RIPPLE radial term).
 * The cost of the clamp is one comparison per axis on cells that
 * are NEVER out of bounds for the inner pattern loops (which use
 * [0, w) × [0, h)) — the branch predictor eats it. */
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

/* Pseudo-random scalar at integer lattice corner (xi, yi) ∈ [0, 1].
 * The same (xi, yi, seed) triple ALWAYS produces the same value —
 * that's what makes value noise "noise" and not "random". */
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

/* Single-octave smooth value noise — bilinear blend of the 4 lattice
 * corners surrounding (x, y), with smoothstep'd interpolants. */
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

/* 3-octave fBm — what we actually contour.  Multiple frequencies
 * stacked with halving amplitude give the contour interesting
 * topology (lakes, peninsulas, saddles) at any threshold.
 * Each octave: amp · value_noise(x · freq, y · freq). */
static float fbm_3oct(const ScalarField *src, float x, float y)
{
    float oct0 = value_noise(src, x * FBM_OCT0_FREQ, y * FBM_OCT0_FREQ) * FBM_OCT0_AMP;
    float oct1 = value_noise(src, x * FBM_OCT1_FREQ, y * FBM_OCT1_FREQ) * FBM_OCT1_AMP;
    float oct2 = value_noise(src, x * FBM_OCT2_FREQ, y * FBM_OCT2_FREQ) * FBM_OCT2_AMP;
    return clamp01(oct0 + oct1 + oct2);
}

/* ---------- ScalarField lifecycle ---------------------------------- */

/* scalar_field_reset — establish dims + fresh hash seed; samples[]
 * stays uninitialised here (scalar_field_rebuild will overwrite). */
static void scalar_field_reset(ScalarField *src, int w, int h)
{
    src->w     = w;
    src->h     = h;
    src->count = w * h;
    src->seed  = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
}

/* scalar_field_rebuild — overwrite samples[] with f(x, y + t) for the
 * current time t.  Time is added to the y-axis so the field appears
 * to "drift downward" as scene_tick advances. */
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
 * ms_classify — compute the 4-bit marching-squares case at cell
 * (x, y) for threshold T against the given ScalarField.
 *
 *   bit 0 ← bottom-left  corner (x,   y+1) ≥ T
 *   bit 1 ← bottom-right corner (x+1, y+1) ≥ T
 *   bit 2 ← top-right    corner (x+1, y)   ≥ T
 *   bit 3 ← top-left     corner (x,   y)   ≥ T
 *
 * Returns ∈ {0..15}. Cell (x, y) here means "the 2×2 block whose
 * top-left corner is sample (x, y)".  Pure function: same field +
 * same (x, y, T) → same case.  The dispatch in §7 calls this once
 * per cell per pattern; some patterns scan multiple T values per
 * cell (TOPO, JUNCTIONS, SANDWICH).
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
 * Pattern signature
 *
 *   Each pattern is called PER CELL.  It receives (x, y) and the
 *   current "control" inputs:
 *     user_t      — the user-adjustable threshold (+/- on the keyboard)
 *     field_time  — drift accumulator on the source field (seconds)
 *     sweep_phase — cyclic 0..1 phase tied to drift, for animated T
 *
 *   Each pattern writes THREE per-cell outputs:
 *     out_glow    — 1.0 = "this cell visible", 0.0 = skip
 *     out_band    — colour band ∈ {0..3} → indexes PAIR_BAND_BASE+band
 *     out_glyph   — exact ASCII character to draw; 0 = skip even if
 *                   glow > 0 (defensive)
 *
 *   Patterns that need the marching-squares case glyph for a specific
 *   threshold call ms_classify(src, x, y, T) → c, then ms_case_glyph[c].
 *   Patterns that want a uniform glyph (multi-T overlays, region
 *   fills) write their own char.
 *
 *   The §9 renderer is dumb — it reads the three buffers and paints.
 */

/* Helpers — predicates over the 16 marching-squares cases. */

/* Cases where the contour is a horizontal/vertical split (no
 * corners alone) — the "ortho" subset. */
static inline bool ms_case_is_ortho(int c)
{
    return c == 3 || c == 6 || c == 9 || c == 12;
}

/* Cases where the contour cuts off a single corner — the "diagonal"
 * subset.  Includes both 1-corner-above and 1-corner-below (3-above)
 * configurations, which are diagonal mirrors of each other. */
static inline bool ms_case_is_diagonal(int c)
{
    return c == 1 || c == 2 || c == 4  || c == 7
        || c == 8 || c == 11|| c == 13 || c == 14;
}

/* Cell-level emit helpers — set all three outputs in one place so each
 * pattern function reads as a sequence of named "what to draw" decisions
 * instead of three scattered writes. */
static inline void cell_skip(float *gl, uint8_t *bn, char *gy)
{
    *gl = 0.0f; *bn = 0; *gy = 0;
}
static inline void cell_emit(float *gl, uint8_t *bn, char *gy,
                             int band, char glyph)
{
    *gl = 1.0f; *bn = (uint8_t)(band & 3); *gy = glyph;
}

/* Emit the case-aware glyph at threshold T; skip if the case is empty
 * (0 = all below, 15 = all above).  Shared by every single-threshold
 * pattern that wants a "real" marching-squares glyph. */
static inline void cell_emit_case_glyph(const ScalarField *src,
                                        int x, int y, float t, int band,
                                        float *gl, uint8_t *bn, char *gy)
{
    int c = ms_classify(src, x, y, t);
    char glyph = ms_case_glyph[c];
    if (glyph == ' ') { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, band, glyph);
}

/* ---------- Tier 1 — SINGLE: basic single-iso visualisations --------- */

/* CONTOUR — single iso-line at the user-controlled threshold.
 * Classic marching-squares output: case-specific glyph per cell. */
static void pattern_contour(const ScalarField *src, int x, int y,
                            float user_t, float field_time, float sweep_phase,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    cell_emit_case_glyph(src, x, y, user_t, 2, gl, bn, gy);
}

/* THICK — fat contour band. A cell fires if user_t falls inside any
 * of three nearby thresholds (T-half, T, T+half); the glyph is the
 * proper case glyph if the cell IS on the T iso, else '#' fill. */
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

/* INVERT — case-coloured contour: each MS case maps to its own
 * colour band (c & 3) so adjacent contour cells with different
 * geometries read in different tints. */
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

/* DOTS — same contour cells as CONTOUR but rendered with '.' glyphs
 * in the dimmer band 1. Reads as a stippled outline. */
static void pattern_dots(const ScalarField *src, int x, int y,
                         float user_t, float field_time, float sweep_phase,
                         float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 1, '.');
}

/* UNIFORM — same contour cells but rendered with a single '+' glyph,
 * dropping the case information. Useful for "where is the contour"
 * without the orientation noise that varying case glyphs introduce. */
static void pattern_uniform(const ScalarField *src, int x, int y,
                            float user_t, float field_time, float sweep_phase,
                            float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, '+');
}

/* ---------- Tier 2 — MULTI: evenly-spaced thresholds ----------------- */

/* Shared multi-threshold sampler — emit '+' wherever any of n_levels
 * evenly-spaced thresholds fires, with the LAST firing level winning
 * the colour band (produces nested ring colouring). */
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

/* TOPO — 8 evenly-spaced thresholds, classic topographic map. */
static void pattern_topo(const ScalarField *src, int x, int y,
                         float user_t, float field_time, float sweep_phase,
                         float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_multi_levels(src, x, y, TOPO_N_LEVELS, gl, bn, gy);
}

/* DENSE — 16 levels: closely-spaced contours reveal subtle field
 * structure (small bumps, gentle slopes). */
static void pattern_dense(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_multi_levels(src, x, y, DENSE_N_LEVELS, gl, bn, gy);
}

/* SPARSE — 4 levels: just the major elevation bands. */
static void pattern_sparse(const ScalarField *src, int x, int y,
                           float user_t, float field_time, float sweep_phase,
                           float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_multi_levels(src, x, y, SPARSE_N_LEVELS, gl, bn, gy);
}

/* STEPS — 6 levels, identical to TOPO/DENSE but with the case glyph
 * preserved on the highest firing level for a "stepped contour"
 * look (rings + visible orientation). */
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

/* RAINBOW — 12 levels with the band hashed by (level + case) so
 * adjacent cells along the same contour can sit in different bands. */
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

/* ---------- Tier 3 — PAIRS: specific T combinations ------------------ */

/* DUAL — two fixed thresholds in distinct colours. */
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

/* SHELL — fill cells whose field value sits in the narrow band
 * [SHELL_T_LOW, SHELL_T_HIGH].  Reads as a thick contour "wall"
 * rather than a one-cell line. */
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

/* INNER — only the HIGH-T contour (peaks of the field, small closed
 * loops around the local maxima). */
static void pattern_inner(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_case_glyph(src, x, y, INNER_T, 3, gl, bn, gy);
}

/* OUTER — only the LOW-T contour (basins / "coastline" at the
 * field's low watermark; long meandering shapes). */
static void pattern_outer(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)field_time; (void)sweep_phase;
    cell_emit_case_glyph(src, x, y, OUTER_T, 1, gl, bn, gy);
}

/* SANDWICH — three iso-lines (low / mid / high), each in its own
 * band. Shows the field's nested elevation structure with only
 * three rings instead of TOPO's eight. */
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

/* ---------- Tier 4 — TOPOLOGY: case-aware masks ---------------------- */

/* SADDLE — full contour, with the two ambiguous saddle cases (5/10)
 * highlighted in band 3 + 'X' glyph, normal cells in band 1. */
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

/* ORTHO — draw only the horizontal/vertical split cases (3, 6, 9, 12).
 * Reveals the axis-aligned "spine" of the contour. */
static void pattern_ortho(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (!ms_case_is_ortho(c)) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, ms_case_glyph[c]);
}

/* DIAGONALS — only the corner-cut cases.  The complement of ORTHO. */
static void pattern_diagonals(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (!ms_case_is_diagonal(c)) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, ms_case_glyph[c]);
}

/* JUNCTIONS — scan a stack of thresholds and mark every cell that hits
 * a saddle case (5 or 10) on ANY of them with a bright 'X'.  Cells
 * that aren't saddles but lie on the user_t contour are drawn dimly
 * underneath as context.
 *
 * The single-threshold version would be near-empty: saddles are
 * topologically rare on a smooth field at any one T.  Scanning across
 * TOPO_N_LEVELS thresholds collects them all so the X's read as a
 * map of "places where the topology is ambiguous somewhere in the
 * field's elevation range" — which is the pedagogical point. */
static void pattern_junctions(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;

    /* Pass 1 — collect saddles across a stack of evenly-spaced T. */
    bool is_saddle_somewhere = false;
    for (int level = 0; level < TOPO_N_LEVELS; level++) {
        float t = (float)(level + 1) / (float)(TOPO_N_LEVELS + 1);
        if (ms_case_is_saddle(ms_classify(src, x, y, t))) {
            is_saddle_somewhere = true;
            break;
        }
    }
    if (is_saddle_somewhere) { cell_emit(gl, bn, gy, 3, 'X'); return; }

    /* Pass 2 — dim context: the user_t contour, drawn faintly so the
     * X's stand out against it. */
    int c = ms_classify(src, x, y, user_t);
    if (c > 0 && c < 15) cell_emit(gl, bn, gy, 1, ms_case_glyph[c]);
    else                 cell_skip(gl, bn, gy);
}

/* ROUGHNESS — full contour, band = case index mod 4.  Adjacent
 * cells with different MS geometries fall in different colour bands,
 * making the contour look "rough" where the geometry varies fast. */
static void pattern_roughness(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, c & 3, ms_case_glyph[c]);
}

/* ---------- Tier 5 — REGIONS: filled visualisations ------------------ */

/* ABOVE — fill cells whose corner value is above user_t (the
 * "land" side of the iso-line, painted solid). */
static void pattern_above(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    if (scalar_field_at(src, x, y) >= user_t) cell_emit(gl, bn, gy, 3, '#');
    else                          cell_skip(gl, bn, gy);
}

/* BELOW — inverse of ABOVE: fill the "sea" side. */
static void pattern_below(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    if (scalar_field_at(src, x, y) <  user_t) cell_emit(gl, bn, gy, 1, '#');
    else                          cell_skip(gl, bn, gy);
}

/* STRIPE — fill cells in the band [user_t - half, user_t + half].
 * Equivalent to SHELL but tied to the USER threshold. */
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

/* ZEBRA — fill alternate equi-spaced bands ⌊v · ZEBRA_N_BANDS⌋.
 * Half the field is solid, half is empty — striking horizontal
 * stripes when the field is monotone. */
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

/* PERIMETER — fill every cell that lies on any non-empty MS case
 * (i.e. ANY edge crossing).  The result is the contour drawn solidly
 * with '#' rather than the case-specific orientation glyphs. */
static void pattern_perimeter(const ScalarField *src, int x, int y,
                              float user_t, float field_time, float sweep_phase,
                              float *gl, uint8_t *bn, char *gy)
{
    (void)field_time; (void)sweep_phase;
    int c = ms_classify(src, x, y, user_t);
    if (c == 0 || c == 15) { cell_skip(gl, bn, gy); return; }
    cell_emit(gl, bn, gy, 2, '#');
}

/* ---------- Tier 6 — ANIMATED: time-varying thresholds --------------- */

/* SWEEP — single contour with T sine-animating between CONTOUR_T_MIN
 * and CONTOUR_T_MAX.  Reads as the iso-line "panning" through the
 * elevation range. */
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

/* PULSE — squared-sine T animation (sharper rise/fall than SWEEP).
 * Drives the contour outward in a "breath" rhythm. */
static void pattern_pulse(const ScalarField *src, int x, int y,
                          float user_t, float field_time, float sweep_phase,
                          float *gl, uint8_t *bn, char *gy)
{
    (void)user_t; (void)sweep_phase;
    float lo  = CONTOUR_T_MIN;
    float hi  = CONTOUR_T_MAX;
    float s   = sinf(field_time * (2.0f * (float)M_PI / PULSE_PERIOD_SEC));
    float t   = lo + (hi - lo) * (s * s);     /* squared → 0..1 with sharper edges */
    cell_emit_case_glyph(src, x, y, t, 2, gl, bn, gy);
}

/* RIPPLE — per-cell T = user_t + amp · sin(distance/freq − speed·time).
 * Concentric waves emanating from the grid centre; iso-lines wobble
 * in radial sympathy. */
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

/* STAIRS — T cycles through STAIRS_N_LEVELS discrete values.  The
 * contour "snaps" between elevations rather than gliding. */
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

/* CHAOS — per-cell T = user_t + hash(x, y, time) · amp.  The contour
 * shimmers with noise on top of its base shape. */
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
 * ContourPattern — one row of the §7 pattern dispatch table.
 *
 * INTENT
 *   Replace a per-cell switch(Pattern) with a single function-pointer
 *   indirect call.  The hot loop in scene_evaluate_contours becomes
 *   `sample(src, x, y, ut, ft, sp, …)` — no branch on pattern type,
 *   one cache-friendly table.  Adding a new pattern is a three-line
 *   edit: write the fn, append a row here, add its enum.  The
 *   fixed-size [N_PATTERNS] initialiser keyed by enum makes the
 *   compiler complain if the table is incomplete or mis-ordered.
 *
 * CONTEXT
 *   noise_patterns[N_PATTERNS] in §7 is the singular static table.
 *   Read by scene_evaluate_contours() (resolves .sample), and by
 *   pattern_name() / pattern_tier() for the HUD readout.  Never
 *   mutated at runtime — defined `const` so it lives in .rodata.
 *
 * MEMBER LOGIC
 *   name   : HUD label, RIGHT-padded to exactly 10 chars so the
 *            HUD format string produces aligned columns when the
 *            user cycles through patterns via n/p.
 *   tier   : "N-XXXX " 7-char label encoding tier index (1..6) plus
 *            a 4-char mnemonic (SING/MULT/PAIR/TOPO/REGN/ANIM).
 *            Displayed next to the pattern name in the HUD.
 *   sample : Per-cell function pointer matching ContourPatternFn.
 *            Called once per cell per sim tick (~11K cells × 60 Hz
 *            = ~660K calls/sec).  Writes the three output slots
 *            (glow, band, glyph) in the ContourGrid; never reads
 *            them back.
 *
 * REFERENCES
 *   • Designated initialisers (C99 §6.7.9) — `[INDEX] = { … }` lets
 *     the table be defined out of declaration order while staying
 *     enum-keyed.
 *   • Function-pointer dispatch — see Bryant & O'Hallaron,
 *     "Computer Systems: A Programmer's Perspective" §3.10 for the
 *     compiled-code shape of an indirect call vs a switch.
 */
typedef struct {
    const char       *name;     /* 10-char padded for HUD alignment    */
    const char       *tier;     /* 7-char "N-LABEL " padded            */
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
 * ContourGrid — per-cell render-output buffer (the §7 → §9 hand-off).
 *
 * INTENT
 *   Decouple "compute what to draw" (scene_evaluate_contours, runs
 *   every sim tick) from "actually draw it" (scene_draw, runs every
 *   render frame).  Patterns write (glow, band, glyph) triples here;
 *   the renderer just looks up and paints.  Conceptually this is a
 *   tile/cell framebuffer in a software renderer — same idea as the
 *   cbuf[] in this project's raster/ files, but per-character-cell
 *   instead of per-pixel.
 *
 *   Glyph is BAKED IN by the pattern.  Case-aware patterns write
 *   ms_case_glyph[c]; region/multi-T patterns write their own '+',
 *   '#', 'X', '.', etc.  The renderer is intentionally dumb — it
 *   does not know what marching squares is.
 *
 * CONTEXT
 *   One instance lives on Scene (§8).  Written by
 *   scene_evaluate_contours() once per sim tick (one pattern fn call
 *   per cell).  Read by scene_draw() once per render frame.  Index
 *   with contour_grid_idx(cg, x, y) = y · w + x — row-major.
 *
 * MEMORY
 *   glow[] + band[] + glyph[] = 4 + 1 + 1 = 6 bytes/cell × CELLS_MAX
 *   ≈ 66 KB in BSS.  No allocation.  Cache-cold only at the start of
 *   each evaluate pass; the (read, classify, write) cycle stays
 *   inside the inner loop.
 *
 * MEMBER LOGIC
 *   w, h    : Grid dimensions in cells.  Always match the source
 *             ScalarField's dims — set together at scene_reset().
 *   count   : w · h, cached for the zero-init loop and any other
 *             count-based traversal.
 *   glow[]  : Per-cell visibility flag: > 0.0 = paint, ≤ 0.0 = skip.
 *             Stored as float (not bool) so patterns COULD scale it
 *             for intensity if a future feature needs it; the
 *             renderer currently just thresholds against 0.
 *   band[]  : Palette band ∈ {0..3} → PAIR_BAND_BASE + band.
 *             Bottom 2 bits are used (`band & 3` clamps); patterns
 *             are expected to write a clean value, the mask is
 *             defensive.
 *   glyph[] : ASCII character the renderer will emit.  0 = also skip
 *             (defensive double-gate, in case a pattern set glow > 0
 *             but forgot to pick a glyph — which would be a bug).
 *
 * REFERENCES
 *   • Deferred-shading pattern — separate "decide what to draw" from
 *     "draw it" via an intermediate buffer.  Akenine-Möller et al.,
 *     "Real-Time Rendering" (4th ed.) §20.1.
 *   • Struct-of-Arrays (three parallel arrays) instead of
 *     Array-of-Structs (one packed record per cell) — chosen because
 *     the renderer iterates the three fields in lock-step and the
 *     SoA layout avoids padding waste on the 1-byte fields.
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
 * PatternState — active pattern + every scalar the patterns consume.
 *
 * INTENT
 *   Group the five values that together describe "what's animating
 *   right now": which pattern is active, where we are in the field-
 *   drift cycle, where we are in the sweep-phase cycle, at what user
 *   threshold, and at what drift speed.  They change together, they
 *   are read together, and they ALL get forwarded to the per-cell
 *   pattern function — packing them in one struct keeps the call
 *   sites short and the dataflow centralised.
 *
 * CONTEXT
 *   Lives on Scene (§8).  Read by:
 *     • scene_evaluate_contours() — passes user_t / field_time /
 *       sweep_phase down to the active pattern fn.
 *     • scene_tick()              — advances field_time and
 *       sweep_phase by drift_mult · dt each tick.
 *     • screen_draw()             — HUD readouts (pattern name, T).
 *   Mutated by app_handle_key():
 *     • n/p   → current        (modular wraparound through N_PATTERNS)
 *     • </>   → user_t         (clamped to [CONTOUR_T_MIN, _MAX])
 *     • +/-   → drift_mult     (clamped to [DRIFT_MULT_MIN, _MAX])
 *
 * MEMBER LOGIC
 *   current     : Active pattern enum, indexes noise_patterns[].
 *                 Defaults to PATTERN_CONTOUR.
 *   user_t      : User-adjustable iso threshold ∈ [CONTOUR_T_MIN,
 *                 CONTOUR_T_MAX].  Stepped by CONTOUR_T_STEP via
 *                 </> (or . , as easier-to-type aliases).  Read by
 *                 Tier 1/3/4/5 patterns directly; Tier 6 animated
 *                 patterns modulate it with time.
 *   field_time  : Drift accumulator (seconds).  Added to the y-axis
 *                 sample coordinate so the source field appears to
 *                 "scroll downward" as ticks advance.  Monotonically
 *                 increasing within a run; reset only by r.
 *   sweep_phase : Cyclic 0..1 phase that completes one cycle every
 *                 SWEEP_T_PERIOD_SEC of wall-clock at drift_mult=1.
 *                 Drives the Tier-6 SWEEP pattern's sinusoidal T.
 *                 Wraps mod 1.0 each cycle.
 *   drift_mult  : Power-of-2 speed multiplier ∈ [DRIFT_MULT_MIN,
 *                 DRIFT_MULT_MAX] that scales both field_time and
 *                 sweep_phase increments.  Stepped via +/-.  Same
 *                 convention as the sibling showcases in this dir.
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

/* pattern_state_advance_clocks — one frame of clock integration.
 * field_time accumulates linearly (drives field "scroll"); sweep_phase
 * wraps mod 1.0 each cycle (drives Tier-6 SWEEP sinusoidal T). */
static void pattern_state_advance_clocks(PatternState *ps, float dt)
{
    float drift = (float)ps->drift_mult;
    ps->field_time  += FIELD_DRIFT * drift * dt;
    ps->sweep_phase += dt * drift / SWEEP_T_PERIOD_SEC;
    if (ps->sweep_phase > 1.0f) ps->sweep_phase -= 1.0f;
}

/* pattern_state_cycle_to_next/prev — modular pattern cycle through
 * the N_PATTERNS dispatch table.  Bound to n/N and p/P. */
static void pattern_state_cycle_to_next(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + 1) % N_PATTERNS);
}
static void pattern_state_cycle_to_prev(PatternState *ps)
{
    ps->current = (Pattern)(((int)ps->current + N_PATTERNS - 1) % N_PATTERNS);
}

/* pattern_state_drift_faster/slower — power-of-2 step on drift_mult,
 * clamped to [DRIFT_MULT_MIN, DRIFT_MULT_MAX].  Bound to +/-. */
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

/* pattern_state_raise/lower_threshold — step user_t by CONTOUR_T_STEP,
 * clamped to [CONTOUR_T_MIN, CONTOUR_T_MAX].  Bound to </>. */
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
 * PaletteState — index of the currently-active Theme.
 *
 * INTENT
 *   A one-int wrapper is overkill data-wise but valuable
 *   structurally: it gives "which colour scheme is showing" a stable
 *   home on Scene, matches the sub-struct convention used by sibling
 *   showcases, and makes the t/T key handler read as
 *   `s->palette.current = …` instead of `s->theme_idx = …`.
 *
 * CONTEXT
 *   Lives on Scene (§8).  Mutated by app_handle_key() on t/T (with
 *   modular wraparound through N_THEMES) and by palette_state_init()
 *   at reset.  Whenever it changes, theme_apply() (§3) MUST be
 *   called to push the new Theme's colour indices into the ncurses
 *   colour pairs — this struct holds the index, ncurses holds the
 *   actual pair bindings.  Read by screen_draw() for the HUD label.
 *
 * MEMBER LOGIC
 *   current : Index into themes[] ∈ [0, N_THEMES).  Anything outside
 *             that range is invalid; theme_apply() clamps to 0
 *             defensively.
 */
typedef struct {
    int current;
} PaletteState;

static void palette_state_init(PaletteState *p) { p->current = 0; }

/* palette_state_cycle_to_next/prev — modular theme cycle.  Caller MUST
 * call theme_apply() afterwards to push the new colour indices into
 * the ncurses pairs (PaletteState holds the index, ncurses holds the
 * binding).  Bound to t/T. */
static void palette_state_cycle_to_next(PaletteState *p)
{
    p->current = (p->current + 1) % N_THEMES;
}
static void palette_state_cycle_to_prev(PaletteState *p)
{
    p->current = (p->current + N_THEMES - 1) % N_THEMES;
}

/*
 * Scene — composite owner of ALL mutable simulation state.
 *
 * INTENT
 *   Single root that the main loop (§10), the simulation step
 *   (scene_tick), and the renderer (scene_draw) all share.  Composes
 *   the four sub-domains as named members so their roles are
 *   explicit at the type level instead of hidden in a flat field
 *   list.  Each sub-struct owns one concern; Scene owns the
 *   composition + the global paused gate.
 *
 * THE PIPELINE (top to bottom = dataflow order)
 *
 *   source   — the scalar field we contour              (the DATA)
 *               ↓ (sampled by §6 ms_classify + §7 patterns)
 *   pattern  — active pattern + user_t + drift state    (the CONTROL)
 *               ↓ (drives scene_evaluate_contours)
 *   grid     — per-cell (glow, band, glyph) buffer      (the OUTPUT)
 *               ↓ (read by scene_draw in §9)
 *   palette  — active theme index                       (the COLOUR)
 *               + paused flag                           (the GATE)
 *
 *   At a glance:
 *     ScalarField → (pattern + ms_classify) → ContourGrid → renderer
 *
 * CONTEXT
 *   One instance, owned by App in §10.  Touched only from the main
 *   thread — the §10 signal handlers do NOT reach into Scene; they
 *   flip volatile flags on App and let the main loop respond at the
 *   next frame boundary.
 *
 * LIFETIME
 *   scene_init   : zero-init, sub-struct defaults, call scene_reset.
 *   scene_reset  : re-seed ScalarField, zero clocks, rebuild field,
 *                  evaluate contours.  Called on r and on
 *                  SIGWINCH-driven resize.
 *   scene_tick   : advance clocks, rebuild field, re-evaluate.
 *   No teardown — Scene lives in BSS via App; OS reclaims at exit.
 *
 * MEMBER LOGIC
 *   source  : ScalarField  — the scalar source we contour over.
 *   grid    : ContourGrid  — per-cell render-output buffer.
 *   pattern : PatternState — active pattern + clocks + user_t.
 *   palette : PaletteState — active theme index.
 *   paused  : When true, scene_tick early-returns, freezing both
 *             the field-drift clock and the contour re-evaluation.
 *             Renderer keeps drawing the last computed grid → the
 *             user sees a still frame.  Toggled by space.
 */
typedef struct {
    ScalarField  source;   /* §5 — scalar field we contour                */
    ContourGrid  grid;     /* per-cell (glow, band, glyph) output         */
    PatternState pattern;  /* active pattern + user_t + drift state       */
    PaletteState palette;  /* active theme index                          */
    bool         paused;   /* if true, scene_tick early-returns           */
} Scene;

/*
 * scene_recompute_field — rebuild the ScalarField at the current
 * field_time.  Pure side-effect on source.samples[]; the §6/§7
 * functions read from those samples on the next evaluate pass.
 */
static void scene_recompute_field(Scene *s)
{
    scalar_field_rebuild(&s->source, s->pattern.field_time);
}

/* contour_grid_evaluate_all_cells — sweep the grid, calling `sample`
 * once per cell with the current control state (ut, ft, sp).  Pure
 * dispatch: the pattern fn knows what to compute; this just walks
 * the SoA buffers and feeds each cell's three output slots in.
 *
 * Hot loop — one pattern call per cell × ~11K cells × 60 Hz.  Inner
 * body stays a single indirect call thanks to the §7 cell_emit /
 * cell_skip inline helpers each pattern uses. */
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

/*
 * scene_evaluate_contours — dispatch the active pattern at every
 * cell.  Assumes the ScalarField is already current (caller arranges
 * that via scene_recompute_field).
 *
 *   STEP 1 — RESOLVE the active pattern's sampler from the dispatch table
 *   STEP 2 — SAMPLE every cell into the ContourGrid
 */
static void scene_evaluate_contours(Scene *s)
{
    /* STEP 1 — RESOLVE the active pattern */
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    ContourPatternFn sample = noise_patterns[active].sample;

    /* STEP 2 — SAMPLE every cell into the ContourGrid */
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
    scene_evaluate_contours (s);    /* paint initial frame so paused looks ok */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    pattern_state_init (&s->pattern);
    palette_state_init (&s->palette);
    s->paused = false;
    scene_reset(s, mw, mh);
}

/*
 * scene_tick — drive one sim step.
 *
 *   STEP 1 — ADVANCE the per-frame clocks (field-drift, sweep-phase)
 *   STEP 2 — REBUILD the ScalarField for the new field_time
 *   STEP 3 — RE-EVALUATE the active pattern into the ContourGrid
 */
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
 * Screen — ncurses viewport dimensions cache.
 *
 * INTENT
 *   ncurses owns the terminal state.  We just need to know "how
 *   wide / tall is the drawable area right now" so scene_draw can
 *   centre the grid and screen_draw can right-align the HUD without
 *   re-querying ncurses on every paint.  Updated lazily on resize.
 *
 * CONTEXT
 *   One instance lives on App (§10).  Refreshed by screen_init() at
 *   startup and screen_resize() on SIGWINCH.  Read by every render
 *   fn that needs to know where edges or centres are.
 *
 * MEMBER LOGIC
 *   cols : Viewport width in character cells.  Source of truth is
 *          getmaxyx(stdscr, ..., cols) inside screen_init /
 *          screen_resize.
 *   rows : Viewport height in character cells.  Includes the HUD
 *          band — render fns subtract HUD_BAND_RESERVED_ROWS as
 *          needed to get the drawable interior.
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

/* viewport_centre_grid_origin — compute the top-left screen cell where
 * a grid_w × grid_h contour grid should sit inside the cols × rows
 * viewport.  HUD-aware: leaves HUD_TOP_ROWS reserved above and
 * HUD_BOTTOM_ROWS below.  Falls back to the HUD edge if the grid is
 * larger than the drawable interior. */
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

/* contour_cell_paint — emit one cell from the ContourGrid at screen
 * (sx, sy), gated by glow > 0 and a non-empty glyph.  band & 3 is
 * defensive — patterns are expected to write a clean 0..3. */
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

/* scene_draw — paint the ContourGrid's per-cell (glow, band, glyph)
 * buffers into the terminal.  The pattern already decided what to
 * draw; the renderer just looks it up and paints.
 *
 *   STEP 1 — CENTRE the grid inside the viewport (HUD-aware)
 *   STEP 2 — PAINT every in-bounds cell the pattern marked visible
 */
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

/* hud_resolve_displayed_threshold — pick the T value the HUD should
 * show.  For SWEEP, T is animating sinusoidally between CONTOUR_T_MIN
 * and CONTOUR_T_MAX; for every other pattern, the user-set T is what
 * the renderer is actually drawing.  Same formula as pattern_sweep. */
static float hud_resolve_displayed_threshold(const PatternState *ps)
{
    if (ps->current != PATTERN_SWEEP) return ps->user_t;
    float lo  = CONTOUR_T_MIN;
    float hi  = CONTOUR_T_MAX;
    float mid = 0.5f * (lo + hi);
    float amp = 0.5f * (hi - lo);
    return mid + amp * sinf(ps->sweep_phase * 2.0f * (float)M_PI);
}

/* hud_draw_top_left_title — fixed "MARCHING SQUARES" chip on row 0. */
static void hud_draw_top_left_title(void)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MARCHING SQUARES ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* hud_draw_top_right_status — fps · sim Hz · state · pattern N/M · T,
 * right-justified on row 0.  Falls back to column 0 if the formatted
 * string is wider than the viewport. */
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

/* hud_field_bold_label — draw one bold HUD_PAIR field on row 1
 * starting at column `x`, advance by `width`, return the new x. */
static int hud_field_bold_label(int x, const char *fmt,
                                const char *val, int width)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, fmt, val);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + width;
}

/* hud_field_palette_swatch — "palette:####" segment on row 1: a label
 * in plain HUD pair, then HUD_N_PALETTE_BANDS '#' chars each in their
 * own band colour.  Returns the new x. */
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

/* hud_field_meta — trailing "scale · drift · map dims" on row 1. */
static void hud_field_meta(int x, int drift_mult, int grid_w, int grid_h)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  scale:%.2f  drift:x%d  map:%dx%d ",
             FIELD_SCALE, drift_mult, grid_w, grid_h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* hud_draw_param_row — row 1 dashboard: pattern, tier, theme,
 * palette swatch, meta.  Each helper returns the next x so the row
 * reads as a left-to-right pipeline. */
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

/* hud_draw_bottom_hint — single-row key-binding bar at the bottom. */
static void hud_draw_bottom_hint(int rows)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " n/p:pattern  t/T:theme  </>:T  +/-:drift  ]/[:Hz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* screen_draw — full frame composer.
 *
 *   STEP 1 — CLEAR the back buffer
 *   STEP 2 — PAINT the contour field
 *   STEP 3 — OVERLAY the top HUD row (title + right-aligned status)
 *   STEP 4 — OVERLAY the row-1 parameter dashboard
 *   STEP 5 — OVERLAY the bottom key-hint bar
 */
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
 * App — top-level program state; the BSS root.
 *
 * INTENT
 *   Single owner for the simulation, the screen, the sim-rate
 *   target, the active map size, and the two volatile signal flags.
 *   Lives in BSS as g_app (file-scope) for one specific reason:
 *   POSIX signal handlers can ONLY safely touch global async-signal-
 *   safe state, so the two sig_atomic_t flags must be reachable
 *   without arguments.  Everything else on App is touched from the
 *   main thread only.
 *
 * CONTEXT
 *   Exactly one instance: g_app at file scope.  Built in main(),
 *   driven by the main game loop, torn down by atexit(cleanup).
 *   on_exit_signal / on_resize_signal flip the flags; the main loop
 *   polls them at frame boundaries and reacts (clean exit / lazy
 *   resize) — keeping all the heavy work off the signal handler.
 *
 * MEMBER LOGIC
 *   scene       : All mutable simulation state (§8).  See Scene doc
 *                 above for the dataflow pipeline.
 *   screen      : Viewport dimensions cache (§9).
 *   sim_fps     : Target simulation Hz ∈ [SIM_FPS_MIN, SIM_FPS_MAX].
 *                 Drives the fixed-timestep accumulator in main();
 *                 stepped by ]/[ keys.  Independent of render FPS.
 *   map_w, map_h: Grid dims chosen by app_pick_map_size() from the
 *                 current Screen.  Re-derived on resize; clamped to
 *                 [16, MAP_W_MAX] × [8, MAP_H_MAX] (BSS cap).
 *   running     : 0 = exit the main loop next iteration.  Cleared by
 *                 the SIGINT/SIGTERM handlers and by q/ESC in
 *                 app_handle_key().  sig_atomic_t for handler safety.
 *   need_resize : Set by the SIGWINCH handler; main loop polls it
 *                 before the next frame and calls app_do_resize().
 *                 sig_atomic_t for handler safety.
 *
 * REFERENCES
 *   • Fix Your Timestep! — Glenn Fiedler:
 *     https://gafferongames.com/post/fix_your_timestep/
 *     The fixed-timestep accumulator pattern main() uses.
 *   • POSIX.1-2017 § 2.4.3 — async-signal-safe functions and signal
 *     handler discipline.  Why running/need_resize are sig_atomic_t.
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

/* app_pick_map_size — derive the contour grid dims from the current
 * viewport, leaving HUD_BAND_RESERVED_ROWS for the dashboard, then
 * clamp to [MAP_*_MIN, MAP_*_MAX] (the BSS cap on samples[]). */
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

/* app_do_resize — full re-init after SIGWINCH: re-read viewport
 * dims, re-derive map size, rebuild the scene at the new size. */
static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* app_sim_rate_faster/slower — step sim_fps by SIM_FPS_STEP, clamped
 * to [SIM_FPS_MIN, SIM_FPS_MAX].  Bound to ]/[. */
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

/* app_handle_key — keyboard dispatch.  Each case forwards to a tiny
 * named mutator so the switch reads as a pseudocode binding table.
 * Returns false on exit-request (q/Q/ESC); true otherwise. */
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

/* main_install_signal_handlers — clean exit on SIGINT/SIGTERM, lazy
 * resize on SIGWINCH.  POSIX async-signal-safe path: handlers touch
 * only the sig_atomic_t flags on g_app. */
static void main_install_signal_handlers(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* app_bootstrap — ncurses + viewport + scene initial state. */
static void app_bootstrap(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

/* app_handle_pending_resize — if SIGWINCH flagged a resize, do it
 * and reset the frame clocks so dt doesn't include the resize cost
 * (which would falsely trigger the spiral-of-death cap below). */
static void app_handle_pending_resize(App *app,
                                      int64_t *frame_time,
                                      int64_t *sim_accum)
{
    if (!app->need_resize) return;
    app_do_resize(app);
    *frame_time = clock_ns();
    *sim_accum  = 0;
}

/* app_compute_frame_dt — clock delta since last frame, clamped at
 * SIM_MAX_FRAME_DT_MS so a slow terminal can't trigger unbounded
 * sim-tick catch-up below (spiral-of-death guard). */
static int64_t app_compute_frame_dt(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    int64_t dt_cap = (int64_t)SIM_MAX_FRAME_DT_MS * NS_PER_MS;
    if (dt > dt_cap) dt = dt_cap;
    return dt;
}

/* app_drain_fixed_timestep — Fiedler accumulator: integrate as many
 * fixed-rate sim ticks as fit into the elapsed dt, so the sim runs
 * at exactly sim_fps Hz regardless of render FPS jitter. */
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

/* app_update_fps_meter — sliding-window average over the last
 * FPS_UPDATE_MS of wall-clock time. */
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

/* app_throttle_to_render_target — sleep for the remainder of the
 * RENDER_FRAME_BUDGET so the loop doesn't burn CPU between frames. */
static void app_throttle_to_render_target(int64_t frame_time, int64_t dt)
{
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(RENDER_FRAME_BUDGET_NS - elapsed);
}

/* app_present_frame — render the world + flush ncurses back-buffer. */
static void app_present_frame(App *app, double fps_display)
{
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
}

/* app_poll_keyboard — non-blocking getch + dispatch.  Returns false
 * iff the user requested exit (q / Q / ESC). */
static bool app_poll_keyboard(App *app)
{
    int ch = getch();
    if (ch == ERR) return true;
    return app_handle_key(app, ch);
}

/* main — game-loop driver.
 *
 *   STEP 1 — SEED rng + INSTALL signal handlers + BOOTSTRAP app
 *   STEP 2 — LOOP:
 *              a. honour any pending resize
 *              b. compute dt (clamped)
 *              c. drain fixed-timestep sim ticks
 *              d. update fps meter
 *              e. throttle to render budget
 *              f. present the frame
 *              g. poll input → maybe exit
 *   STEP 3 — TEARDOWN screen
 */
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
