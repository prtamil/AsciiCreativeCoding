/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * signed_distance_field_jfa_showcase.c
 *   — Compute the Euclidean distance field of a set of seed points
 *     using the Jump Flood Algorithm (Rong & Tan, 2006). Visualise it
 *     30 different ways across 6 complexity tiers.
 *
 * DEMO: Scatter a handful of "seed" points on a grid. For every other
 *       cell, find the seed it's closest to (and how far). The result
 *       — the Euclidean Distance Field (EDF) — is the building block
 *       for many graphics techniques (Valve-style SDF font rendering,
 *       Voronoi diagrams, raymarching, soft shadows, outline strokes).
 *       Computed via the Jump Flood Algorithm: log₂(N) passes where
 *       each cell "looks" at neighbours k = N/2, N/4, … away and
 *       inherits whichever seed is closest. Seeds wobble continuously
 *       (sin/cos around fixed anchors) so the JFA result animates.
 *
 *       30 patterns in 6 tiers (cycle with n / p):
 *         Tier 1 DISTANCE  — raw distance mappings (SDF, INVERSE …)
 *         Tier 2 BANDS     — periodic structuring (RINGS, SAW, ZEBRA …)
 *         Tier 3 VORONOI   — cell-id-driven effects (CELLS, SPECKLE …)
 *         Tier 4 CONTOURS  — iso-line effects (OUTLINE, HALO, AURA …)
 *         Tier 5 STRUCTURE — neighbour-aware Voronoi topology
 *                            (EDGES, VERTICES, SKELETON …)
 *         Tier 6 ANIMATED  — patterns driven by the wobble clock
 *                            (PULSE, SCAN, ECHO, PLASMA, CHAOS)
 *
 * Study alongside:
 *   ./worley_cellular_noise.c — also produces Voronoi-style cells, but
 *       streaming + analytic (3×3 hash lookup per query). This file
 *       precomputes everything via JFA — different cost trade-off.
 *   ../generational/voronoi_region_map.c — alternative precomputed
 *       Voronoi using a different algorithm.
 *
 * Section map:
 *   §1 config   — grid, patterns, seed count, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 algorithm + data + animation + optimization
 *     §5a JFA   — JFAGrid, jfa_pass, jfa_grid_run
 *     §5b data  — Seed, seeds_scatter
 *     §5c anim  — seeds_animate (wobble around anchors)
 *     §5d opt   — sub-pixel sdf_read for smooth gradients
 *   §6 patterns — 30 visualisations of the same JFA output + dispatch
 *   §7 scene    — GlowField + GlyphRamp + PatternState + PaletteState
 *                 + Scene composition + scene_evaluate_field
 *   §8 screen   — Screen, GridPlacement, scene_draw, HUD helpers
 *   §9 app      — App harness, FrameClock, signals, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume animation
 *   r          reset (new seed scatter)
 *   n / N      next pattern  (p / P previous)
 *   t / T      next / previous theme
 *   ] / [      raise / lower simulation tick Hz
 *   + / =      faster wobble drift (× 2)
 *   -          slower wobble drift (/ 2)
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra signed_distance_field_jfa_showcase.c \
 *       -o sdf_jfa -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Jump Flood Algorithm (JFA). For an N×N grid with
 *                  K seed points (K ≪ N²), compute the Euclidean
 *                  distance field in O(N² log N) — way faster than
 *                  the naive O(N²·K) "check every seed per cell".
 *
 *                  Each grid cell holds a 2-int "nearest seed coords"
 *                  pair. Initialisation: seed cells point to
 *                  themselves; everyone else points to (-1, -1).
 *
 *                  For step k = N/2, N/4, N/8, ..., 1:
 *                    For each cell p:
 *                      For each of 9 offsets in {-k, 0, +k}²:
 *                        Look at the neighbour q = p + offset.
 *                        If q.seed exists AND its distance to p is
 *                        shorter than p's current best, p adopts
 *                        q's seed.
 *
 *                  After ⌈log₂(N)⌉ passes the whole grid has been
 *                  "flooded" — every cell holds its true nearest
 *                  seed. Reading distance is then a single sqrt.
 *
 * Data-structure : JFAGrid — two int16_t grids (nearest_x[],
 *                  nearest_y[]) holding each cell's currently-believed
 *                  nearest seed coords, plus a scratch_x/scratch_y
 *                  pair for ping-pong (one buffer read, one written,
 *                  swap each pass — keeps passes independent and
 *                  correct).
 *
 *                  Seed[N_SEEDS] — fixed anchor + per-axis wobble
 *                  phase + current float position (sub-pixel
 *                  sdf_read) + int snap (JFA ownership).
 *
 *                  GlowField — the per-cell (glow ∈ [0,1], band ∈
 *                  {0..3}) output buffer the renderer reads from.
 *
 * Rendering      : ASCII only. Density-graded glyphs ('.', '*', '#')
 *                  via GlyphRamp in 4 theme palette colours. Each
 *                  pattern picks (glow, band) from (distance,
 *                  seed-id, neighbour-topology) in a pattern-specific
 *                  way; the renderer is pattern-agnostic.
 *
 * Performance    : log₂(N) passes × N² cells × 9 neighbours
 *                  = 9·N²·log₂(N) cell-comparisons total.
 *                  For an 11K-cell grid, that's ~1.3M comparisons —
 *                  way under one frame. The algorithm REGENERATES
 *                  the full field every reset / animation tick (it
 *                  could be incremental, but for showcase clarity
 *                  we recompute from scratch).
 *
 * References     : JFA + DISTANCE TRANSFORMS (the algorithm)
 *                  • Rong, G. & Tan, T-S. (2006) — "Jump Flooding in
 *                    GPU with Applications to Voronoi Diagram and
 *                    Distance Transform", I3D'06. The original JFA
 *                    paper — source of the log₂(N) step schedule and
 *                    the 9-neighbour propagation rule used in §5a:
 *                    https://dl.acm.org/doi/10.1145/1111411.1111431
 *                  • Felzenszwalb, P. & Huttenlocher, D. (2012) —
 *                    "Distance Transforms of Sampled Functions",
 *                    Theory of Computing 8(19). The exact O(n)
 *                    two-pass alternative when you don't need GPU
 *                    parallelism; useful contrast to JFA's
 *                    approximate-but-trivially-parallel approach.
 *                  • Aurenhammer, F. (1991) — "Voronoi diagrams: a
 *                    survey of a fundamental geometric data
 *                    structure", ACM Comput. Surv. 23(3). Background
 *                    for the Tier-3/5 VORONOI / EDGES / VERTICES
 *                    patterns — formal definition of the cell dual,
 *                    triple-points, and the medial axis:
 *                    https://dl.acm.org/doi/10.1145/116873.116880
 *
 *                  SDF-DERIVED PATTERNS (the 30 patterns)
 *                  • Green, C. (2007) — "Improved Alpha-Tested
 *                    Magnification for Vector Textures and Special
 *                    Effects", SIGGRAPH'07 (Valve). The famous SDF-
 *                    font paper; foundational reasoning for the
 *                    Tier-4 OUTLINE / SHELL / HALO iso-line work:
 *                    https://valvesoftware.github.io/SourceEngine2007/Alpha-tested_Magnification.pdf
 *                  • Quilez, I. — "2D distance functions". The
 *                    catalogue of analytic SDFs and combinators
 *                    (union, smin, abs, mod) — direct source for
 *                    the Tier-2 RINGS/SAW/WAVE band patterns and
 *                    the Tier-4 contour effects:
 *                    https://iquilezles.org/articles/distfunctions2d/
 *                  • Quilez, I. — "Smooth minimum". The smoothstep-
 *                    blended union used as the conceptual model
 *                    behind Tier-4 AURA's multi-σ Gaussian sum:
 *                    https://iquilezles.org/articles/smin/
 *
 *                  ASCII RENDERING (SDF → terminal)
 *                  • Bourke, P. — "Character representation of grey
 *                    scale images". Source of the canonical luminance
 *                    ramps; the GlyphRamp '.', '*', '#' here is a
 *                    coarse 3-tier subset of Bourke's 10-char ramp:
 *                    http://paulbourke.net/dataformats/asciiart/
 *                  • Wikipedia — "ANSI escape code § 8-bit". The
 *                    xterm/ANSI 256-colour palette (16 system + 6³
 *                    cube + 24 greys) that themes[] in §1 indexes:
 *                    https://en.wikipedia.org/wiki/ANSI_escape_code#8-bit
 *                  • Padala, P. — "NCURSES Programming HOWTO".
 *                    Pragmatic reference for the §3/§8 rendering
 *                    APIs (init_pair, mvaddch, doupdate, A_BOLD):
 *                    https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *                  COMPARE IN PROJECT
 *                  • ./worley_cellular_noise.c — streaming Voronoi
 *                    via 3×3 hash lookup (no precompute); same cell
 *                    structure, very different mechanism.
 *                  • ../generational/voronoi_region_map.c —
 *                    alternative precomputed Voronoi (static
 *                    lattice, no animation) vs this file's full
 *                    JFA recompute per tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * "How far is every grid cell from the nearest of K marked cells?"
 * Naively: check every seed per cell → O(N²K). JFA cuts this by
 * letting cells COPY their neighbour's answer if the neighbour
 * already knows a closer seed. After log₂(N) rounds of progressively
 * smaller jumps, the right answer reaches every cell.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the seeds as candles in a pitch-dark room laid out on
 * graph paper. You want to know, for every grid square, which candle
 * is closest. You can't see the candles directly — but you can ask
 * your immediate neighbour cells "which candle do YOU know about?"
 * and if their answer would be closer to you than yours, you adopt
 * it.
 *
 * Doing this with neighbours at distance 1 only would take O(N)
 * rounds (light propagates one square at a time). JFA's trick: also
 * ask neighbours at distance N/2, N/4, N/8, ..., 1 — large jumps
 * carry seed information across the grid quickly, smaller jumps
 * refine it locally. log₂(N) passes total.
 *
 * Once each cell knows ITS nearest seed, distance is one sqrt.
 *
 * ALGORITHM IN STEPS  (per "regenerate the field" call — jfa_grid_run)
 * ──────────────────
 *  1. Initialise: nearest_x[c] = c.x, nearest_y[c] = c.y for every
 *     SEED cell. For every other cell, nearest_x[c] = nearest_y[c]
 *     = JFA_SENTINEL.                                  (jfa_grid_init)
 *  2. For step k = max(W,H) / 2, halving each pass down to 1:
 *       a. For every cell c at (cx, cy):
 *          For each of 9 offsets in {-k, 0, +k}²:
 *            Look at neighbour q = c + offset.
 *            If q is in bounds AND q has an owner AND its owner is
 *            closer to c than c's current best, copy q's owner.
 *       b. Swap the ping-pong buffers.                       (jfa_pass)
 *  3. After log₂(N) passes, every cell knows its true nearest seed.
 *  4. Read distance: dist(c) = √((c.x − nearest_x[c])² +
 *                                (c.y − nearest_y[c])²)       (sdf_read)
 *
 * KEY FORMULAS
 * ────────────
 *  Distance                    : d(p, q) = √((px − qx)² + (py − qy)²)
 *  JFA step schedule           : k_i = ⌈N/2^(i+1)⌉, for i = 0 … log₂N
 *  Cell count examined per pass: 9 × N²  (8 neighbours + self)
 *  Total work                  : 9 · N² · log₂(N)
 *  N here                      = max(grid_w, grid_h)
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

    /* JFA needs sentinel "no seed yet"; use a value that can never be
     * a real coordinate. int16_t max = 32767 — far above MAP_W_MAX. */
    JFA_SENTINEL      = INT16_MAX,

    /* Number of seed points scattered on each reset. Sparse so cells
     * are large and the JFA wavefronts are easy to see. */
    N_SEEDS           = 20,

    /* Color pair indices. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,
};

/*
 * Continuous seed animation (Worley-style wobble).
 *
 *   WOBBLE_RADIUS_CELLS — each seed orbits its anchor with this
 *                          radius in cells. ~5 cells of drift on a
 *                          200×56 grid with 20 seeds (≈ 10-cell
 *                          average spacing) → cell boundaries shift
 *                          visibly without seeds colliding.
 *   WOBBLE_RATE         — radians per second per unit of drift_mult.
 *                          0.5 ≈ one full sine cycle per 12 seconds —
 *                          slow enough that the integer-pixel snap of
 *                          the JFA reads as smooth motion to the eye
 *                          (each pixel transition takes several
 *                          frames; faster rates make snapping visible).
 */
#define WOBBLE_RADIUS_CELLS  5.0f
#define WOBBLE_RATE          0.5f

/*
 * Sub-pixel rounding — converting a float seed position to the
 * integer cell that JFA treats as the seed's owner.  Adding 0.5f
 * before the int cast gives round-to-nearest instead of truncation
 * (round-toward-zero would jitter the integer snap by ±1 right at
 * the half-cell boundary).
 */
#define ROUND_TO_NEAREST_BIAS    0.5f

/*
 * is_local_max tolerance (distance units).  A neighbour's distance
 * has to exceed ours by MORE than this to disqualify us from the
 * medial axis.  Without an epsilon, two cells with bit-equal
 * distance would alternately disqualify each other depending on
 * scan order — the result would be a flickering, broken ridge.
 * 0.01 cells is well below one screen pixel so the visual is unaffected.
 */
#define MEDIAL_AXIS_TOLERANCE    0.01f

/*
 * JFA "no candidate yet" cost.  jfa_pass tracks the best
 * squared-distance seen so far; an unseeded cell starts at this
 * value so the first valid neighbour ALWAYS wins.  INT32_MAX is
 * fine — every real squared-distance on this grid is far smaller
 * (max ≈ 200² + 56² ≈ 43 000 << 2^31).
 */
#define JFA_INFINITE_SQ_DISTANCE INT32_MAX

/*
 * Palette quantisation — continuous glow ∈ [0, 1] → discrete band
 * index ∈ {0..3}. Same scheme as the sibling field showcases.
 */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3
#define GLOW_TO_BAND_GAIN   3.999f

#define GLOW_THRESHOLD      0.05f

/* HUD layout — top carries data, bottom carries actions. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Drift multiplier — cranked by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      8
#define DRIFT_MULT_MAX      16

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/* OUTLINE pattern — which iso-distance to draw (cells). */
#define OUTLINE_DISTANCE    8.0f
#define OUTLINE_THICKNESS   1.5f    /* half-width either side of OUTLINE_DISTANCE */

/*
 * Pattern — thirty JFA-output visualisations grouped into six
 * complexity tiers, ordered simple → complex. Cycle with n/p. The
 * enum order MUST match `noise_patterns[]` in §6 (compiler enforces
 * via fixed-size [N_PATTERNS] initialiser).
 *
 *   Tier 1 DISTANCE  : SDF, INVERSE, SQRT, SQUARED, CAPPED
 *   Tier 2 BANDS     : RINGS, SAW, ZEBRA, WAVE, SHELLS
 *   Tier 3 VORONOI   : CELLS, CRACKLE, SPECKLE, RAINBOW, TWINKLE
 *   Tier 4 CONTOURS  : OUTLINE, DUAL, SHELL, HALO, AURA
 *   Tier 5 STRUCTURE : EDGES, VERTICES, INTERIOR, WEAVE, SKELETON
 *   Tier 6 ANIMATED  : PULSE, SCAN, ECHO, PLASMA, CHAOS
 */
typedef enum {
    /* Tier 1 — DISTANCE: raw distance mappings */
    PATTERN_SDF = 0,
    PATTERN_INVERSE,
    PATTERN_SQRT,
    PATTERN_SQUARED,
    PATTERN_CAPPED,
    /* Tier 2 — BANDS: periodic structuring of distance */
    PATTERN_RINGS,
    PATTERN_SAW,
    PATTERN_ZEBRA,
    PATTERN_WAVE,
    PATTERN_SHELLS,
    /* Tier 3 — VORONOI: cell-id-driven effects */
    PATTERN_CELLS,
    PATTERN_CRACKLE,
    PATTERN_SPECKLE,
    PATTERN_RAINBOW,
    PATTERN_TWINKLE,
    /* Tier 4 — CONTOURS: iso-line uses */
    PATTERN_OUTLINE,
    PATTERN_DUAL,
    PATTERN_SHELL,
    PATTERN_HALO,
    PATTERN_AURA,
    /* Tier 5 — STRUCTURE: neighbour-aware (Voronoi topology) */
    PATTERN_EDGES,
    PATTERN_VERTICES,
    PATTERN_INTERIOR,
    PATTERN_WEAVE,
    PATTERN_SKELETON,
    /* Tier 6 — ANIMATED: use the wobble time directly */
    PATTERN_PULSE,
    PATTERN_SCAN,
    PATTERN_ECHO,
    PATTERN_PLASMA,
    PATTERN_CHAOS,
    N_PATTERNS,
} Pattern;

/* pattern_name() / pattern_tier() defined in §6 alongside the
 * dispatch table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Frame-loop timing limits — see Glenn Fiedler "Fix Your Timestep!". */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — one entry in themes[]: a named 256-colour palette.
 *
 * INTENT
 *   Decouple "the colours" from "the renderer". scene_draw() never
 *   touches xterm indices directly; it just writes glow into the
 *   GlowField and lets the §3 colour pairs (rebound on every t/T
 *   key by theme_apply()) decide what those band tiers look like.
 *   Same algorithm, ten visually distinct moods.
 *
 * CONTEXT
 *   N_THEMES const instances in themes[] (§3). Read by theme_apply()
 *   on init and on every t/T keypress; the result is pushed into
 *   ncurses colour pairs PAIR_BAND_BASE … PAIR_BAND_BASE+3 so all
 *   subsequent COLOR_PAIR() calls in §8 pick up the new palette.
 *   PaletteState.current (§7) is the index into this array.
 *
 * THE 4-TIER GRADIENT
 *   band[0] → band[3] correspond to GlowField.band ∈ {0..3}, which
 *   come from glow_to_palette_band (quartile of glow). All four
 *   entries MUST sit in the BRIGHT half of the 256-colour cube
 *   (≥ 24 for cube, ≥ 244 for greys) — bottom-of-palette colours
 *   are invisible against default-black with A_DIM. See
 *   CLAUDE.md "Theme Palette Brightness" for the rule.
 *
 *   Convention: band[0] = darkest tier (faintest glow, "void");
 *               band[3] = brightest tier (dense core).
 *   Sibling demos use the same ordering, so a theme that looks
 *   right here will look right in worley / simplex / value_noise.
 *
 * MEMBER LOGIC
 *   name    : 7-char-padded display label shown by hud_draw_theme_field.
 *             Stored as a const char* so themes[] sits in .rodata.
 *   band[4] : xterm-256 colour indices, one per GlowField.band tier.
 *             short (not uint8_t) only because init_pair takes short.
 *             Must satisfy the brightness rule above.
 */
typedef struct {
    const char *name;          /* HUD label, 7-char padded            */
    short       band[4];       /* xterm-256 idx per GlowField.band    */
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

/* terminal_supports_256_colours — wrapper for the "do we have the
 * xterm 256-colour cube?" check.  Named so the branches below read
 * as a capability question instead of an arithmetic threshold. */
static inline bool terminal_supports_256_colours(void) { return COLORS >= 256; }

/* theme_clamp_index — fall back to theme 0 if the requested index is
 * out of range.  Defensive — only caller (app_handle_key) already
 * mods by N_THEMES, but indices arriving from elsewhere (e.g. a
 * future config file) shouldn't crash the renderer. */
static inline int theme_clamp_index(int idx)
{
    return (idx < 0 || idx >= N_THEMES) ? 0 : idx;
}

/* theme_bind_palette_256 — push the 4 band colours from a Theme
 * literal straight into ncurses pairs.  256-colour terminals see
 * the exact xterm indices the theme designer chose. */
static void theme_bind_palette_256(const Theme *t)
{
    for (int band = 0; band < N_PALETTE_BANDS; band++)
        init_pair(PAIR_BAND_BASE + band, t->band[band], -1);
}

/* theme_bind_palette_fallback_8color — ANSI-only terminals lose
 * theme variety but still get a 4-tier gradient via the 8-colour
 * BLUE→CYAN→MAGENTA→YELLOW ramp.  Sibling demos use the same
 * fallback so missing themes degrade consistently. */
static void theme_bind_palette_fallback_8color(void)
{
    static const short ansi_band_ramp[N_PALETTE_BANDS] = {
        COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
    };
    for (int band = 0; band < N_PALETTE_BANDS; band++)
        init_pair(PAIR_BAND_BASE + band, ansi_band_ramp[band], -1);
}

/*
 * theme_apply — install theme #idx's colours into ncurses.
 *
 *   STEP 1 — CLAMP the requested index to [0, N_THEMES)
 *   STEP 2 — BIND palette (256-colour path or 8-colour fallback)
 *
 * Side effect: all subsequent COLOR_PAIR(PAIR_BAND_BASE+i) calls in
 * §8 pick up the new palette.  No GlowField re-write needed.
 */
static void theme_apply(int idx)
{
    /* STEP 1 — clamp into valid range */
    int safe_idx = theme_clamp_index(idx);

    /* STEP 2 — bind via the capability-appropriate path */
    if (terminal_supports_256_colours())
        theme_bind_palette_256(&themes[safe_idx]);
    else
        theme_bind_palette_fallback_8color();
}

/* hud_bind_pairs — assign the always-on HUD/HINT pairs (bright
 * yellow + bright cyan, see CLAUDE.md "HUD Standard").  Separate
 * from theme_apply because these NEVER change — they're the
 * keyboard-legend colours, not the algorithm's palette. */
static void hud_bind_pairs(void)
{
    if (terminal_supports_256_colours()) {
        init_pair(PAIR_HUD,   226, -1);     /* bright yellow */
        init_pair(PAIR_HINT,   51, -1);     /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
}

/*
 * color_init — once-only bootstrap of the ncurses colour system.
 *
 *   STEP 1 — wake up colour mode + opt into "background = -1"
 *   STEP 2 — bind the fixed HUD/HINT pairs
 *   STEP 3 — install the default theme (#0) into the band pairs
 */
static void color_init(void)
{
    /* STEP 1 — enable colour + use the terminal's default background */
    start_color();
    use_default_colors();

    /* STEP 2 — bind HUD/HINT (theme-independent legibility) */
    hud_bind_pairs();

    /* STEP 3 — install the default theme into the band pairs */
    theme_apply(0);
}

/* ===================================================================== */
/* §5  algorithm + data + animation + optimization                        */
/* ===================================================================== */

/*
 * §5 is LAYERED so a learner can read the JFA algorithm cleanly
 * without rendering tricks getting in the way. The layers are:
 *
 *   §5a  ALGORITHM    — JFAGrid + jfa_grid_init/pass/run.
 *                        The pure integer-grid Jump Flood Algorithm
 *                        (Rong & Tan 2006). Operates on int seed
 *                        positions only; no animation, no rendering.
 *
 *   §5b  DATA         — Seed struct + seeds_scatter.
 *                        The feature points the JFA computes
 *                        distances to. Each Seed carries an anchor
 *                        (its static "home") plus animation state.
 *
 *   §5c  ANIMATION    — seeds_animate.
 *                        The wobble effect (sin/cos drift around the
 *                        anchor). Lives OUTSIDE the JFA; the JFA
 *                        consumes whatever positions seeds_animate
 *                        writes.
 *
 *   §5d  OPTIMIZATION — sdf_seed_id + sdf_read.
 *                        Sub-pixel distance read used by patterns.
 *                        Uses each seed's FLOAT position instead of
 *                        its int snap so distance gradients are
 *                        continuous within cells. Pure rendering
 *                        trick; doesn't affect what JFA computes.
 *
 * Types are declared up front (both JFAGrid and Seed) so the four
 * function layers below can reference each other freely.
 */

/* ---------- §5: types ------------------------------------------------- */

/*
 * JFAGrid — the working data structure of the Jump Flood Algorithm.
 *
 * INTENT
 *   Hold the per-cell "nearest seed" information that the JFA
 *   computes and refines across log₂(N) passes. After jfa_grid_run
 *   completes, nearest_x[c] / nearest_y[c] are the integer coords
 *   of the seed closest to cell c. Reading distance from this grid
 *   is then a single sqrt of dist_sq.
 *
 * CONTEXT
 *   One instance lives on Scene (§7). Written by jfa_grid_run() at
 *   scene_init / scene_reset and once per scene_tick. Read by every
 *   §6 pattern via sdf_read() / sdf_seed_id() / direct
 *   nearest_x/y[idx] access. Index cells with
 *   jfa_idx(grid, x, y) = y · w + x — row-major to match every
 *   loop nest in §5a / §6.
 *
 * PING-PONG
 *   JFA passes can't read and write the same buffer mid-pass — the
 *   result of pass P depends on values from pass P-1 across the
 *   whole grid. Two buffer pairs are kept; jfa_pass reads from one,
 *   writes to the other, jfa_grid_run swaps for the next pass.
 *   After log₂(N) passes the result lives in nearest_x/y (the final
 *   copy-back step in jfa_grid_run handles parity).
 *
 * MEMORY
 *   4 × CELLS_MAX × sizeof(int16_t) ≈ 89 KB. All in BSS — no
 *   runtime allocations. int16_t is plenty: grid up to 32767/axis,
 *   far beyond MAP_W_MAX (200) / MAP_H_MAX (56).
 *
 * MEMBER LOGIC
 *   w, h         : Grid dimensions in cells. Set by jfa_grid_run's
 *                  caller (scene_reset writes them) before the
 *                  first pass. Plain int so loop-counter arithmetic
 *                  stays sign-safe across all of §5a / §6.
 *   nearest_x[]  : Post-run state — for cell c, the integer X coord
 *   nearest_y[]    of the seed closest to c. JFA_SENTINEL means "no
 *                  seed reached this cell yet" (only seen briefly
 *                  inside jfa_grid_init; after a full run, every
 *                  in-bounds cell has a valid owner). Authoritative
 *                  source for sdf_seed_id() / sdf_read().
 *   scratch_x[]  : Ping-pong scratch — receives the OUTPUT of the
 *   scratch_y[]    next jfa_pass when nearest_* is the input. Never
 *                  read outside §5a. Sized identically to nearest_*
 *                  because jfa_pass swaps buffer pointers, not
 *                  array slots.
 *
 * Ref: Rong & Tan (2006), "Jump Flooding in GPU with Applications
 *      to Voronoi Diagram and Distance Transform", I3D'06.
 */
typedef struct {
    int     w, h;                    /* grid dims (cells), set at reset   */
    int16_t nearest_x[CELLS_MAX];    /* post-run: cell → nearest seed x   */
    int16_t nearest_y[CELLS_MAX];    /* post-run: cell → nearest seed y   */
    int16_t scratch_x[CELLS_MAX];    /* ping-pong scratch (output side)   */
    int16_t scratch_y[CELLS_MAX];    /* ping-pong scratch (output side)   */
} JFAGrid;

/*
 * Seed — one feature point with all its state.
 *
 * INTENT
 *   One struct, one array — instead of six parallel arrays. Members
 *   partition cleanly by purpose:
 *
 *     - ANCHOR + PHASES (fixed at reset, by seeds_scatter)
 *     - CURRENT FLOAT position fx, fy   — for sub-pixel sdf_read
 *     - CURRENT INT snap     ix, iy     — for JFA ownership
 *
 *   The float/int split is the §5d optimisation: float drives
 *   smooth gradient reads, int drives discrete JFA ownership.
 *
 * CONTEXT
 *   N_SEEDS instances live as Scene.seeds[N_SEEDS] (§7). Lifecycle:
 *     • seeds_scatter (§5b)    — at reset: pick anchors + phases.
 *     • seeds_animate (§5c)    — every tick: compute fx/fy/ix/iy
 *                                from anchor + wobble(wobble_time).
 *     • jfa_grid_init  (§5a)   — reads ix/iy as the integer seed
 *                                coords stamped into JFAGrid.
 *     • sdf_read       (§5d)   — reads fx/fy for sub-pixel distance.
 *   The struct is therefore the BRIDGE between the wobble effect
 *   (animation), the JFA (algorithm), and the per-cell reads
 *   (optimisation) — owning them in one place makes the three roles
 *   obvious.
 *
 * MEMORY
 *   sizeof(Seed) = 2·int16 + 2·float + 2·float + 2·int16 = 24 bytes.
 *   N_SEEDS=20 → 480 bytes total on Scene. Negligible vs the JFAGrid
 *   (~89 KB) and GlowField (~88 KB).
 *
 * MEMBER LOGIC
 *   anchor_x, anchor_y : Integer home position ∈ [0, w) × [0, h).
 *                        Fixed across animation; set ONLY by
 *                        seeds_scatter (called at reset). The wobble
 *                        orbits around this point.
 *   phase_x, phase_y   : Per-axis wobble phase ∈ [0, 2π) in radians.
 *                        Independent x/y phases — set
 *                        independently at scatter time so each seed
 *                        traces ELLIPSES of its own orientation/
 *                        eccentricity instead of identical circles.
 *                        Without independent phases all seeds would
 *                        drift in lockstep along a single direction.
 *   fx, fy             : Current float position
 *                          fx = anchor_x + WOBBLE_RADIUS · sin(t + phase_x)
 *                          fy = anchor_y + WOBBLE_RADIUS · cos(t + phase_y)
 *                        clamped to [0, w-1] × [0, h-1]. Updated
 *                        every tick by seeds_animate. Used by
 *                        sdf_read for the sub-pixel distance trick
 *                        (continuous gradient inside each cell).
 *   ix, iy             : round(fx, fy). The integer coords JFA reads
 *                        for ownership. Updates by ±1 only when fx/fy
 *                        crosses a half-cell boundary — that's
 *                        why distance gradients read smooth (fx/fy)
 *                        but Voronoi cell BORDERS still snap to
 *                        integer cells.
 */
typedef struct {
    int16_t anchor_x, anchor_y;      /* fixed home position (cells)        */
    float   phase_x,  phase_y;       /* per-axis wobble phase (radians)    */
    float   fx, fy;                  /* sub-pixel position (for sdf_read)  */
    int16_t ix, iy;                  /* int snap (for JFA ownership)       */
} Seed;

/* Small helpers — used by every layer below. */
static inline int jfa_idx(const JFAGrid *g, int x, int y)
{
    return y * g->w + x;
}

/* Squared Euclidean distance — sqrt deferred to the read side
 * (kept as int to avoid sqrt in the JFA's tight comparison loop). */
static inline int dist_sq(int ax, int ay, int bx, int by)
{
    int dx = ax - bx;
    int dy = ay - by;
    return dx * dx + dy * dy;
}

/* ---------- §5a  ALGORITHM — pure JFA on integer positions ----------- */

/*
 * jfa_grid_init — STAGE 1. Reset every cell to SENTINEL ("no seed
 * yet"); stamp each seed's int position with its own coords. After
 * this, only seed cells have ownership; jfa_pass propagates from there.
 */
static void jfa_grid_init(JFAGrid *g, const Seed *seeds, int n_seeds)
{
    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        g->nearest_x[i] = JFA_SENTINEL;
        g->nearest_y[i] = JFA_SENTINEL;
    }
    for (int s = 0; s < n_seeds; s++) {
        int x = seeds[s].ix;
        int y = seeds[s].iy;
        if (x < 0 || x >= g->w || y < 0 || y >= g->h) continue;
        int idx = jfa_idx(g, x, y);
        g->nearest_x[idx] = (int16_t)x;
        g->nearest_y[idx] = (int16_t)y;
    }
}

/*
 * jfa_pass — one Jump Flood pass at step size k.
 *
 * For every cell (cx, cy), look at the 9 neighbours at offsets
 * {-k, 0, +k}² in the SOURCE grid. Inherit whichever of them has a
 * seed closer to (cx, cy) than our own current best.
 *
 * src and dst MUST be different buffers (see PING-PONG note above).
 */
/*
 * jfa_consider_candidate_owner — inner test of the 9-neighbour probe.
 *
 * Inspect the seed-owner stored at (nx, ny) in the source grid;
 * if its seed is closer to OUR cell (cx, cy) than the best owner
 * we've found so far, adopt it.  Out-of-bounds and "no seed yet"
 * (sentinel) neighbours are no-ops.  This is the per-candidate
 * step of the propagation rule from Rong & Tan (2006) §3.
 */
static inline void jfa_consider_candidate_owner(
    const JFAGrid *g, int cx, int cy, int nx, int ny,
    const int16_t *src_x, const int16_t *src_y,
    int *best_d, int *best_sx, int *best_sy)
{
    if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) return;

    int neigh_idx = jfa_idx(g, nx, ny);
    int candidate_sx = src_x[neigh_idx];
    int candidate_sy = src_y[neigh_idx];
    if (candidate_sx == JFA_SENTINEL) return;        /* neighbour has no owner yet */

    int candidate_d = dist_sq(cx, cy, candidate_sx, candidate_sy);
    if (candidate_d < *best_d) {                     /* closer than our current best? */
        *best_d  = candidate_d;
        *best_sx = candidate_sx;
        *best_sy = candidate_sy;
    }
}

/*
 * jfa_pass — one Jump-Flood propagation pass at step size k.
 *
 *   FOR every cell c = (cx, cy):
 *     1. LOAD current best owner from src
 *     2. PROBE the 8 neighbours at offsets {-k, 0, +k}² (skip self)
 *        — keep whichever has the closest owner-seed to c
 *     3. COMMIT the winner into dst
 *
 * src and dst MUST be different buffers (see PING-PONG note on
 * JFAGrid).
 */
static void jfa_pass(const JFAGrid *g, int k,
                     const int16_t *src_x, const int16_t *src_y,
                     int16_t *dst_x, int16_t *dst_y)
{
    for (int cy = 0; cy < g->h; cy++) {
        for (int cx = 0; cx < g->w; cx++) {
            int self_idx = jfa_idx(g, cx, cy);

            /* STEP 1 — LOAD: start with whatever owner this cell already had */
            int best_sx = src_x[self_idx];
            int best_sy = src_y[self_idx];
            int best_d  = (best_sx == JFA_SENTINEL)
                        ? JFA_INFINITE_SQ_DISTANCE
                        : dist_sq(cx, cy, best_sx, best_sy);

            /* STEP 2 — PROBE the 8 neighbours at jump step k */
            for (int dy = -k; dy <= k; dy += k) {
                for (int dx = -k; dx <= k; dx += k) {
                    if (dx == 0 && dy == 0) continue;        /* skip self */
                    jfa_consider_candidate_owner(
                        g, cx, cy, cx + dx, cy + dy,
                        src_x, src_y,
                        &best_d, &best_sx, &best_sy);
                }
            }

            /* STEP 3 — COMMIT the winning owner into the dst buffer */
            dst_x[self_idx] = (int16_t)best_sx;
            dst_y[self_idx] = (int16_t)best_sy;
        }
    }
}

/* jfa_initial_jump_step — the FIRST jump size in the halving schedule.
 * Rong & Tan use ⌊max(W,H)/2⌋ so the very first pass can already
 * carry seed information from opposite sides of the grid. */
static inline int jfa_initial_jump_step(const JFAGrid *g)
{
    int n_max = (g->w > g->h) ? g->w : g->h;
    return n_max / 2;
}

/* jfa_next_jump_step — schedule step: halve until 1, then stop.
 * The full sequence is N/2, N/4, …, 2, 1 — log₂(N) total passes. */
static inline int jfa_next_jump_step(int k) { return k >> 1; }

/* jfa_swap_ping_pong — flip src↔dst so the next pass reads what the
 * previous pass just wrote.  Keeps the two passes operating on
 * disjoint buffers (a JFA correctness requirement). */
static inline void jfa_swap_ping_pong(int16_t **src_x, int16_t **src_y,
                                      int16_t **dst_x, int16_t **dst_y)
{
    int16_t *tx = *src_x; *src_x = *dst_x; *dst_x = tx;
    int16_t *ty = *src_y; *src_y = *dst_y; *dst_y = ty;
}

/* jfa_finalise_into_canonical — copy-back if parity left the final
 * winners in the scratch buffer.  jfa_pass writes into dst; the next
 * iteration swaps src↔dst.  After an EVEN number of passes the
 * final result lives in nearest_*; after an ODD number, in
 * scratch_*.  Callers (and §6 patterns) ALWAYS read from
 * nearest_*, so we copy back when needed. */
static inline void jfa_finalise_into_canonical(JFAGrid *g,
                                               const int16_t *final_src_x,
                                               const int16_t *final_src_y)
{
    if (final_src_x == g->nearest_x) return;        /* already canonical */

    int n = g->w * g->h;
    for (int i = 0; i < n; i++) {
        g->nearest_x[i] = final_src_x[i];
        g->nearest_y[i] = final_src_y[i];
    }
}

/*
 * jfa_grid_run — STAGE 2.  Drive the full JFA pipeline.
 *
 *   STEP 1 — STAMP seed coords into the grid (sentinel everywhere else)
 *   STEP 2 — RUN the halving-step schedule: k = N/2, N/4, …, 1
 *            (each pass ping-pongs through the two buffer pairs)
 *   STEP 3 — FINALISE: ensure the result lives in g->nearest_*
 *
 * After this returns, every cell's nearest_x/y points to the seed
 * (in integer coords) that owns it; sdf_read / sdf_seed_id can be
 * called on any cell.
 */
static void jfa_grid_run(JFAGrid *g, const Seed *seeds, int n_seeds)
{
    /* STEP 1 — STAMP: seed cells get their own coords, others SENTINEL */
    jfa_grid_init(g, seeds, n_seeds);

    /* STEP 2 — RUN: log₂(N) propagation passes, halving k each time */
    int16_t *src_x = g->nearest_x, *src_y = g->nearest_y;
    int16_t *dst_x = g->scratch_x, *dst_y = g->scratch_y;

    for (int k = jfa_initial_jump_step(g); k >= 1; k = jfa_next_jump_step(k)) {
        jfa_pass            (g, k, src_x, src_y, dst_x, dst_y);
        jfa_swap_ping_pong  (&src_x, &src_y, &dst_x, &dst_y);
    }

    /* STEP 3 — FINALISE: copy back if parity left winners in scratch */
    jfa_finalise_into_canonical(g, src_x, src_y);
}

/* ---------- §5b  DATA — seed scatter --------------------------------- */

/*
 * seeds_scatter — randomise anchor positions and wobble phases for
 * all seeds. Called on every reset (r). Does NOT set the current
 * fx/fy/ix/iy — caller follows with seeds_animate(..., 0.0f) to
 * populate them.
 */
static void seeds_scatter(Seed *seeds, int n_seeds, int w, int h)
{
    for (int s = 0; s < n_seeds; s++) {
        seeds[s].anchor_x = (int16_t)(rand() % w);
        seeds[s].anchor_y = (int16_t)(rand() % h);
        seeds[s].phase_x  = (float)rand() / (float)RAND_MAX
                          * 2.0f * (float)M_PI;
        seeds[s].phase_y  = (float)rand() / (float)RAND_MAX
                          * 2.0f * (float)M_PI;
    }
}

/* ---------- §5c  ANIMATION — wobble (independent of the JFA) --------- */

/* seed_compute_wobble_offset — the wobble formula in one place.
 *
 *   offset = ( R · sin(t + φx),  R · cos(t + φy) )
 *
 * Independent x/y phases turn what would be a circle into an
 * ELLIPSE oriented per-seed; that's what makes the field look
 * organic instead of every seed orbiting in lockstep. */
static inline void seed_compute_wobble_offset(const Seed *s, float t,
                                              float *wx, float *wy)
{
    *wx = WOBBLE_RADIUS_CELLS * sinf(t + s->phase_x);
    *wy = WOBBLE_RADIUS_CELLS * cosf(t + s->phase_y);
}

/* seed_clamp_to_grid — keep the float position inside the grid
 * rectangle [0, w-1] × [0, h-1].  A seed that escaped the grid
 * would hash to JFA_SENTINEL and disappear from the diagram. */
static inline void seed_clamp_to_grid(float *fx, float *fy, int w, int h)
{
    if (*fx < 0.0f)              *fx = 0.0f;
    if (*fx > (float)(w - 1))    *fx = (float)(w - 1);
    if (*fy < 0.0f)              *fy = 0.0f;
    if (*fy > (float)(h - 1))    *fy = (float)(h - 1);
}

/* seed_snap_to_integer_cell — round float → nearest integer cell.
 * The +0.5 bias gives round-to-nearest instead of round-toward-zero;
 * without it, the int snap would jitter by ±1 at half-cell crossings. */
static inline int16_t seed_snap_to_integer_cell(float f)
{
    return (int16_t)(f + ROUND_TO_NEAREST_BIAS);
}

/* seed_commit_position — write the float position (used by sdf_read
 * for sub-pixel smoothness) AND the integer snap (used by JFA for
 * discrete ownership) together; the two MUST stay in sync. */
static inline void seed_commit_position(Seed *s, float fx, float fy)
{
    s->fx = fx;
    s->fy = fy;
    s->ix = seed_snap_to_integer_cell(fx);
    s->iy = seed_snap_to_integer_cell(fy);
}

/*
 * seeds_animate — every tick, recompute each seed's current position
 * from its anchor + wobble at time t.
 *
 *   FOR every seed s:
 *     STEP 1 — COMPUTE the wobble offset (sin/cos around the anchor)
 *     STEP 2 — APPLY: position = anchor + offset, clamped to grid
 *     STEP 3 — COMMIT: write float (sdf_read) + int snap (JFA)
 */
static void seeds_animate(Seed *seeds, int n_seeds, int w, int h, float t)
{
    for (int s = 0; s < n_seeds; s++) {
        /* STEP 1 — wobble offset = (R·sin, R·cos) with independent x/y phases */
        float wx, wy;
        seed_compute_wobble_offset(&seeds[s], t, &wx, &wy);

        /* STEP 2 — apply offset to the anchor, clamp inside the grid */
        float fx = (float)seeds[s].anchor_x + wx;
        float fy = (float)seeds[s].anchor_y + wy;
        seed_clamp_to_grid(&fx, &fy, w, h);

        /* STEP 3 — write float position + integer snap (both consumers) */
        seed_commit_position(&seeds[s], fx, fy);
    }
}

/* ---------- §5d  OPTIMIZATION — sub-pixel distance reads ------------- */

/*
 * sdf_seed_id — for cell (x, y), return the index ∈ [0, n_seeds)
 * of the seed that owns it, or -1 if no owner. Linear search through
 * the seeds array matching the JFA grid's int coords; for n_seeds=20
 * this is well under any cache pressure.
 */
static inline int sdf_seed_id(const JFAGrid *grid, const Seed *seeds,
                              int n_seeds, int x, int y)
{
    int idx = jfa_idx(grid, x, y);
    int sx  = grid->nearest_x[idx];
    int sy  = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) return -1;
    for (int s = 0; s < n_seeds; s++) {
        if (seeds[s].ix == sx && seeds[s].iy == sy) return s;
    }
    return -1;
}

/*
 * sdf_read — distance from cell (x, y) to its nearest seed.
 *
 * Sub-pixel: looks up the owner's INDEX via the int grid, then
 * computes distance using the seed's FLOAT position. The SDF varies
 * smoothly within each cell as the float position drifts, even when
 * the int snap hasn't moved yet — that's what makes Tier 1/2/4
 * gradients feel fluid instead of stair-stepped.
 */
static inline float sdf_read(const JFAGrid *grid, const Seed *seeds,
                             int n_seeds, int x, int y)
{
    int s = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (s < 0) return -1.0f;
    float dx = seeds[s].fx - (float)x;
    float dy = seeds[s].fy - (float)y;
    return sqrtf(dx * dx + dy * dy);
}

/* ===================================================================== */
/* §6  patterns — 30 visualisations of the same JFA output + dispatch     */
/* ===================================================================== */

/*
 * Signature: every pattern function takes
 *   (grid, seeds, n_seeds, x, y, t, &glow, &band)
 *
 *   grid     = JFA result grid (already populated by jfa_grid_run).
 *              Holds w/h plus per-cell nearest-seed coordinates.
 *   seeds    = the seed array — needed for sub-pixel sdf_read.
 *   n_seeds  = number of valid seeds.
 *   x, y     = current cell coords.
 *   t        = wobble_time (Tier 6 patterns read it for animation;
 *              others may use it for subtle modulation).
 *   out_glow = scalar ∈ [0, 1].
 *   out_band = palette index ∈ {0..3}.
 *
 * All patterns read JFA state via three accessors only:
 *   sdf_read(grid, seeds, n_seeds, x, y)      — distance to nearest seed
 *   sdf_seed_id(grid, seeds, n_seeds, x, y)   — index of nearest seed
 *   grid->nearest_x/y[jfa_idx(grid, x, y)]    — raw owner coords for hashing
 *
 * Typical max distance used to normalise the raw SDF for visualisation.
 * Empirical — half the grid diagonal would be ~100; we use 25 because
 * with 20 seeds the typical max distance in any cell is closer to 20.
 */
#define SDF_TYPICAL_MAX     25.0f

/* Spacing of bands in Tier-2 patterns, in distance units. */
#define BAND_SPACING        4.0f

/* Halo decay scale for Tier-4 HALO / AURA (Gaussian σ in distance units). */
#define HALO_SIGMA          5.0f

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline int band_from_glow(float g)
{
    return (int)(g * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK;
}

/* Per-cell pseudo-random scalar ∈ [0, 1] derived from a uint32 — used
 * by Tier-3 RAINBOW / SPECKLE / TWINKLE for per-cell "personality". */
static inline float jfa_hash_to_unit(uint32_t v)
{
    /* Reuse Worley/Value-style splitmix mixer. */
    v = (v ^ (v >> 16)) * 0x7feb352du;
    v = (v ^ (v >> 15)) * 0x846ca68bu;
    v =  v ^ (v >> 16);
    return (float)(v & 0xFFFFu) * (1.0f / 65535.0f);
}

/* Pack the F1-seed coords into one 32-bit value, then hash. Gives a
 * stable per-cell key (every cell in the same Voronoi region produces
 * the same hash). */
static inline uint32_t jfa_cell_key(int sx, int sy)
{
    return ((uint32_t)(sx & 0xFFFF) << 16) | (uint32_t)(sy & 0xFFFF);
}

/* ---------- STRUCTURE helpers (used by Tier 5) ------------------------ *
 *
 * These look at the 4-connectivity neighbours of a cell to detect
 * Voronoi topology: cell-boundary edges (cells whose nearest seed
 * differs from a neighbour's), triple-point vertices, and medial-axis
 * peaks.
 */

/* True iff at least one of the 4-neighbours of (x, y) has a different
 * F1-owner than (x, y) — i.e. (x, y) sits on a Voronoi cell edge. */
static bool is_voronoi_edge(const JFAGrid *g, int x, int y)
{
    int idx_me = jfa_idx(g, x, y);
    int sx = g->nearest_x[idx_me];
    int sy = g->nearest_y[idx_me];
    if (sx == JFA_SENTINEL) return false;
    static const int dx4[4] = { -1, 1,  0, 0 };
    static const int dy4[4] = {  0, 0, -1, 1 };
    for (int k = 0; k < 4; k++) {
        int nx = x + dx4[k];
        int ny = y + dy4[k];
        if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;
        int idx_n = jfa_idx(g, nx, ny);
        if (g->nearest_x[idx_n] != sx || g->nearest_y[idx_n] != sy) return true;
    }
    return false;
}

/* owner_already_listed — tiny linear scan: have we seen this seed
 * coord pair in the per-cell unique-owner tally yet?  N is at most 5
 * (4-neighbours + self) so a hash set would be slower than the scan. */
static inline bool owner_already_listed(const int *sx_seen, const int *sy_seen,
                                        int n, int sx, int sy)
{
    for (int i = 0; i < n; i++)
        if (sx_seen[i] == sx && sy_seen[i] == sy) return true;
    return false;
}

/* Count distinct F1-owners among (x, y) and its 4-neighbours.
 *
 *   FOR each of the 5 cells (self + N/E/S/W):
 *     IF in-bounds AND has-owner AND owner-not-yet-listed:
 *       append owner to the running tally
 *   RETURN tally size
 *
 * Returns 1 (interior cell) to 5 (rare).  Used by Tier-5 VERTICES:
 * 3+ means a Voronoi vertex / triple-point. */
static int count_distinct_neighbours(const JFAGrid *g, int x, int y)
{
    /* 5-step neighbourhood scan: self + 4-connectivity */
    static const int neighbour_dx5[5] = { 0, -1, 1,  0, 0 };
    static const int neighbour_dy5[5] = { 0,  0, 0, -1, 1 };

    int sx_seen[5];
    int sy_seen[5];
    int unique_count = 0;

    for (int k = 0; k < 5; k++) {
        int nx = x + neighbour_dx5[k];
        int ny = y + neighbour_dy5[k];
        if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;

        int idx = jfa_idx(g, nx, ny);
        int neighbour_sx = g->nearest_x[idx];
        int neighbour_sy = g->nearest_y[idx];
        if (neighbour_sx == JFA_SENTINEL) continue;        /* no owner here */

        if (owner_already_listed(sx_seen, sy_seen, unique_count,
                                 neighbour_sx, neighbour_sy)) continue;

        sx_seen[unique_count] = neighbour_sx;
        sy_seen[unique_count] = neighbour_sy;
        unique_count++;
    }
    return unique_count;
}

/* True iff d(x, y) ≥ d at all 4-neighbours. Approximates the MEDIAL
 * AXIS — the ridge of points equidistant from 2+ seeds. Discrete-grid
 * artefacts make this a bit noisy but visually it traces the cell
 * boundaries from the SDF side (highest distance interior strips). */
static bool is_local_max(const JFAGrid *g, const Seed *seeds, int n_seeds,
                         int x, int y)
{
    float d_me = sdf_read(g, seeds, n_seeds, x, y);
    if (d_me < 0.0f) return false;
    static const int dx4[4] = { -1, 1,  0, 0 };
    static const int dy4[4] = {  0, 0, -1, 1 };
    for (int k = 0; k < 4; k++) {
        int nx = x + dx4[k];
        int ny = y + dy4[k];
        if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;
        float d_n = sdf_read(g, seeds, n_seeds, nx, ny);
        if (d_n > d_me + MEDIAL_AXIS_TOLERANCE) return false;
    }
    return true;
}

/* ---------- Tier 1 — DISTANCE: raw distance mappings ------------------ */

/* SDF — raw distance; bright = far from any seed. The canonical
 * Euclidean Distance Field look. */
static void pattern_sdf(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                        float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* INVERSE — bright at seeds, dark at cell edges. Highlights centres. */
static void pattern_inverse(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = 1.0f - clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* SQRT — square-root compression. Lifts the mid-range; softer contrast. */
static void pattern_sqrt(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = sqrtf(clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f));
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* SQUARED — quadratic emphasis. Centres stay dark; only the far cells
 * really light up. High-contrast inverse of SQRT. */
static void pattern_squared(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float lin = clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = lin * lin;
    *out_band = band_from_glow(*out_glow);
}

/* CAPPED — clamp the distance at half-max so far cells form a uniform
 * plateau. Gives a "filled cell interior" look without the gradient
 * roll-off near edges. */
static void pattern_capped(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                           float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float g = clampf(d / (SDF_TYPICAL_MAX * 0.5f), 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 2 — BANDS: periodic structuring of distance ---------- */

/* RINGS — concentric iso-distance bands via a smooth triangular wave. */
static void pattern_rings(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float u   = fmodf(d, BAND_SPACING) / BAND_SPACING;       /* [0, 1) */
    float tri = (u < 0.5f) ? (u * 2.0f) : (2.0f - u * 2.0f);
    *out_glow = tri;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* SAW — sawtooth bands. Sharp drop-off → ring → drop-off. Reads as
 * directional motion outward from each seed. */
static void pattern_saw(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                        float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = fmodf(d, BAND_SPACING) / BAND_SPACING;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* ZEBRA — hard binary bands. Either bright or dark, nothing in between. */
static void pattern_zebra(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    int ring = (int)(d / BAND_SPACING);
    *out_glow = (ring & 1) ? 0.85f : 0.15f;
    *out_band = ring & PALETTE_BAND_MASK;
}

/* WAVE — sine-modulated rings. Smooth peaks and troughs spreading
 * outward; reads like ripples on a pond. */
static void pattern_wave(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = sinf(d * (2.0f * (float)M_PI / BAND_SPACING)) * 0.5f + 0.5f;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* SHELLS — exponential-decay layers. Each "shell" is bright at its
 * inner edge and decays outward; multiple shells nest from each seed. */
static void pattern_shells(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                           float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float u = fmodf(d, BAND_SPACING);
    *out_glow = expf(-u * 1.5f);
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* ---------- Tier 3 — VORONOI: cell-id-driven effects ------------------ */

/* CELLS — solid colour per Voronoi region with a subtle inward gradient
 * (brighter at the seed, dimmer at edges) so each cell has a focal
 * point. Band index = seed id. */
static void pattern_cells(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    int sid = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (sid < 0) { *out_glow = 0.0f; *out_band = 0; return; }
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = 1.0f - clampf(d * 0.06f, 0.0f, 0.4f);
    *out_band = sid & PALETTE_BAND_MASK;
}

/* CRACKLE — Voronoi cells with dark edges. Like CELLS but actively
 * darkens away from the seed centre — reads as dried mud. */
static void pattern_crackle(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    int sid = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (sid < 0) { *out_glow = 0.0f; *out_band = 0; return; }
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = clampf(1.0f - d * 0.10f, 0.0f, 1.0f);
    *out_band = sid & PALETTE_BAND_MASK;
}

/* SPECKLE — each cell has its OWN brightness (drawn from hashing the
 * seed coords). Cell colours don't follow a gradient; they tile flat
 * at hash-derived intensities. */
static void pattern_speckle(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)seeds; (void)n_seeds; (void)t;
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    float v = jfa_hash_to_unit(jfa_cell_key(sx, sy));
    *out_glow = 0.3f + v * 0.7f;
    *out_band = (int)(v * 3.999f) & PALETTE_BAND_MASK;
}

/* RAINBOW — cell brightness from a different hash bit. Same idea as
 * SPECKLE but the band index varies per-cell as a fast hash, so
 * adjacent cells contrast visibly even when their brightnesses agree. */
static void pattern_rainbow(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    uint32_t k = jfa_cell_key(sx, sy);
    float v = jfa_hash_to_unit(k);
    /* Distance fade so cells aren't pure flat tiles. */
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = clampf((1.0f - d * 0.04f) * (0.3f + v * 0.7f), 0.0f, 1.0f);
    *out_band = (int)(jfa_hash_to_unit(k ^ 0xdeadbeefu) * 3.999f) & PALETTE_BAND_MASK;
}

/* TWINKLE — cells pulse in/out, with the pulse rate hashed per-cell so
 * they don't all blink in sync. Each cell flickers at its own slow
 * rhythm. */
static void pattern_twinkle(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    uint32_t k    = jfa_cell_key(sx, sy);
    float    rate = 0.5f + jfa_hash_to_unit(k) * 2.5f;
    float    pulse = 0.5f + 0.5f * sinf(t * rate);
    float    d    = sdf_read(grid, seeds, n_seeds, x, y);
    *out_glow = clampf((1.0f - d * 0.06f) * pulse, 0.0f, 1.0f);
    *out_band = (int)k & PALETTE_BAND_MASK;
}

/* ---------- Tier 4 — CONTOURS: iso-line uses -------------------------- */

/* OUTLINE — single soft contour at OUTLINE_DISTANCE. Triangle falloff
 * either side gives the iso-line a glow. */
static void pattern_outline(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                            float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float diff = fabsf(d - OUTLINE_DISTANCE);
    if (diff > OUTLINE_THICKNESS) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = 1.0f - diff / OUTLINE_THICKNESS;
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* DUAL — two iso-lines (inner + outer), in different palette bands.
 * Shows the same seed set "nested". */
static void pattern_dual(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float d_inner = 4.0f;
    float d_outer = 10.0f;
    float diff_i = fabsf(d - d_inner);
    float diff_o = fabsf(d - d_outer);
    if (diff_i < OUTLINE_THICKNESS) {
        *out_glow = 1.0f - diff_i / OUTLINE_THICKNESS;
        *out_band = 1;
    } else if (diff_o < OUTLINE_THICKNESS) {
        *out_glow = 1.0f - diff_o / OUTLINE_THICKNESS;
        *out_band = 3;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* SHELL — soft annulus filled in the band between d=2 and d=7. Like
 * a thick OUTLINE; useful for "stroke" effects. */
static void pattern_shell(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float d_lo = 2.0f, d_hi = 7.0f;
    if (d < d_lo || d > d_hi) { *out_glow = 0.0f; *out_band = 0; return; }
    float u = (d - d_lo) / (d_hi - d_lo);
    /* Smoothstep so the edges of the shell fade. */
    *out_glow = u * u * (3.0f - 2.0f * u) * (1.0f - u) * 4.0f;
    if (*out_glow > 1.0f) *out_glow = 1.0f;
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* HALO — Gaussian decay around each seed. Bright at the centre,
 * fading smoothly outward. The basic "glow around a point" shape. */
static void pattern_halo(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = expf(-(d * d) / (HALO_SIGMA * HALO_SIGMA));
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* AURA — multi-layered Gaussians. Three decay scales summed; gives
 * each cell a bright core, a halo, and a faint outer glow. */
static void pattern_aura(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    (void)t;
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float s1 = HALO_SIGMA * 0.5f;
    float s2 = HALO_SIGMA;
    float s3 = HALO_SIGMA * 2.0f;
    float g = expf(-(d * d) / (s1 * s1)) * 0.6f
            + expf(-(d * d) / (s2 * s2)) * 0.3f
            + expf(-(d * d) / (s3 * s3)) * 0.15f;
    *out_glow = clampf(g, 0.0f, 1.0f);
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* ---------- Tier 5 — STRUCTURE: neighbour-aware (Voronoi topology) ---- */

/* EDGES — bright on Voronoi cell boundaries (cells whose nearest seed
 * differs from a 4-neighbour). Traces the dual of the Delaunay graph. */
static void pattern_edges(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    if (is_voronoi_edge(grid, x, y)) {
        *out_glow = 1.0f;
        *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* VERTICES — bright at Voronoi vertices (triple-points where 3+ cells
 * meet). Rare, isolated bright pixels. */
static void pattern_vertices(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band)
{
    (void)seeds; (void)n_seeds; (void)t;
    int n = count_distinct_neighbours(grid, x, y);
    if (n >= 3) {
        *out_glow = 1.0f;
        *out_band = 3;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* INTERIOR — the inverse of EDGES: bright everywhere EXCEPT on cell
 * boundaries. Reads as filled tiles with thin dark cracks. */
static void pattern_interior(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band)
{
    (void)t;
    int sid = sdf_seed_id(grid, seeds, n_seeds, x, y);
    if (sid < 0) { *out_glow = 0.0f; *out_band = 0; return; }
    if (is_voronoi_edge(grid, x, y)) {
        *out_glow = 0.0f;
        *out_band = 0;
    } else {
        float d = sdf_read(grid, seeds, n_seeds, x, y);
        *out_glow = clampf(1.0f - d * 0.05f, 0.0f, 1.0f);
        *out_band = sid & PALETTE_BAND_MASK;
    }
}

/* WEAVE — RINGS but masked to cell interiors. Each cell gets its own
 * concentric ring set, edges are dark. Reads as tiled water-ripples. */
static void pattern_weave(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    (void)t;
    if (is_voronoi_edge(grid, x, y)) {
        *out_glow = 0.0f;
        *out_band = 0;
        return;
    }
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float u   = fmodf(d, BAND_SPACING) / BAND_SPACING;
    float tri = (u < 0.5f) ? (u * 2.0f) : (2.0f - u * 2.0f);
    *out_glow = tri;
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* SKELETON — medial-axis approximation. Bright where the SDF is
 * locally maximal — i.e. the points equidistant from multiple seeds.
 * Discrete-grid noise makes the ridges patchy; reads as a sparse
 * Voronoi-graph trace. */
static void pattern_skeleton(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band)
{
    (void)t;
    if (is_local_max(grid, seeds, n_seeds, x, y)) {
        *out_glow = 1.0f;
        *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
    } else {
        *out_glow = 0.0f;
        *out_band = 0;
    }
}

/* ---------- Tier 6 — ANIMATED: drive on wobble_time ------------------- */

/* PULSE — global brightness modulation in time. Same SDF but breathes. */
static void pattern_pulse(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float pulse = 0.5f + 0.5f * sinf(t * 1.5f);
    float g     = clampf(d / SDF_TYPICAL_MAX, 0.0f, 1.0f);
    *out_glow = g * pulse;
    *out_band = band_from_glow(g);
}

/* SCAN — single iso-line whose threshold sweeps outward through time.
 * Reads as a radar pulse expanding from every seed simultaneously. */
static void pattern_scan(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    /* Threshold sweeps 0..SDF_TYPICAL_MAX over one cycle. */
    float threshold = fmodf(t * 4.0f, SDF_TYPICAL_MAX);
    float diff = fabsf(d - threshold);
    if (diff > OUTLINE_THICKNESS * 1.5f) { *out_glow = 0.0f; *out_band = 0; return; }
    *out_glow = 1.0f - diff / (OUTLINE_THICKNESS * 1.5f);
    *out_band = sdf_seed_id(grid, seeds, n_seeds, x, y) & PALETTE_BAND_MASK;
}

/* ECHO — multiple iso-lines, all moving outward simultaneously. Like
 * SCAN but layered — concentric pulses propagating from each seed. */
static void pattern_echo(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                         float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float phase = d - t * 3.0f;
    float u   = fmodf(phase, BAND_SPACING);
    if (u < 0.0f) u += BAND_SPACING;
    u /= BAND_SPACING;
    *out_glow = sinf(u * 2.0f * (float)M_PI) * 0.5f + 0.5f;
    *out_band = ((int)(d / BAND_SPACING)) & PALETTE_BAND_MASK;
}

/* PLASMA — sin combinations of distance and time. Classic demoscene
 * plasma effect re-keyed off the SDF. */
static void pattern_plasma(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                           float *out_glow, int *out_band)
{
    float d = sdf_read(grid, seeds, n_seeds, x, y);
    if (d < 0.0f) { *out_glow = 0.0f; *out_band = 0; return; }
    float v = sinf(d * 0.4f + t)
            + cosf((float)x * 0.15f + t * 0.7f)
            + sinf((float)y * 0.12f + t * 0.5f);
    float g = (v + 3.0f) / 6.0f;
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* CHAOS — animated Voronoi: each cell pulses + brightness driven by
 * its distance to its seed + a per-cell hashed phase. Reads as a
 * field of breathing cells, each on its own clock. */
static void pattern_chaos(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                          float *out_glow, int *out_band)
{
    int idx = jfa_idx(grid, x, y);
    int sx = grid->nearest_x[idx], sy = grid->nearest_y[idx];
    if (sx == JFA_SENTINEL) { *out_glow = 0.0f; *out_band = 0; return; }
    uint32_t k     = jfa_cell_key(sx, sy);
    float    phase = jfa_hash_to_unit(k) * 2.0f * (float)M_PI;
    float    pulse = 0.5f + 0.5f * sinf(t * 2.0f + phase);
    float    d     = sdf_read(grid, seeds, n_seeds, x, y);
    float    fade  = clampf(1.0f - d * 0.05f, 0.0f, 1.0f);
    *out_glow = pulse * fade;
    *out_band = (int)k & PALETTE_BAND_MASK;
}

/* ---------- Dispatch table -------------------------------------------- */

typedef void (*JFAPatternFn)(const JFAGrid *grid, const Seed *seeds, int n_seeds,
                        int x, int y, float t,
                             float *out_glow, int *out_band);

/*
 * NoisePattern — one entry in the §6 dispatch table.
 *
 * INTENT
 *   Replace a giant switch(pattern) in scene_evaluate_field() with
 *   a NAMED, INDEXED table of (label, tier, sampler) records. Adding
 *   a new pattern is a 3-step recipe: write pattern_foo(), add
 *   PATTERN_FOO to the Pattern enum (§1), append one initialiser
 *   here. No call-site touches.
 *
 * CONTEXT
 *   N_PATTERNS const instances in noise_patterns[] (§6), keyed by
 *   the Pattern enum. Looked up exactly once per sim tick by
 *   scene_evaluate_field() to get the active sampler; looked up by
 *   pattern_name() / pattern_tier() every frame for the HUD readout.
 *   PatternState.current (§7) is the index.
 *
 *   The 30 patterns are organised in 6 tiers (DIST / BAND / VOR /
 *   CONT / STRC / ANIM) that share a sampler signature but stress
 *   different aspects of the underlying SDF/Voronoi structure. The
 *   tier string is a HUD readout; the actual grouping is purely
 *   pedagogical (lower tiers exercise the raw distance, higher
 *   tiers exercise the cell topology + animation).
 *
 * MEMBER LOGIC
 *   name   : Pattern label shown by hud_draw_pattern_field. Always
 *            10 chars padded with trailing spaces so the HUD column
 *            stays aligned regardless of name length. The padding
 *            is BAKED INTO the literal — printf("%-10s") would work
 *            too but baking it keeps the table self-documenting.
 *   tier   : "N-LABEL " — 7-char padded tag like "1-DIST ", "5-STRC ".
 *            Read by hud_draw_tier_field. The leading digit lets the
 *            user see the curriculum order at a glance.
 *   sample : Function pointer to the pattern function. Signature is
 *            JFAPatternFn (just above): takes (grid, seeds, n_seeds,
 *            x, y, t, &glow, &band) and writes the two outputs.
 *            const because the table is static; the FUNCTION reads
 *            grid/seeds, but the TABLE entry itself never changes.
 */
typedef struct {
    const char   *name;         /* 10-char padded for HUD alignment  */
    const char   *tier;         /* 7-char "N-LABEL " padded          */
    JFAPatternFn  sample;       /* dispatched by scene_evaluate_field */
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — DISTANCE */
    [PATTERN_SDF]       = { "SDF       ", "1-DIST ", pattern_sdf       },
    [PATTERN_INVERSE]   = { "INVERSE   ", "1-DIST ", pattern_inverse   },
    [PATTERN_SQRT]      = { "SQRT      ", "1-DIST ", pattern_sqrt      },
    [PATTERN_SQUARED]   = { "SQUARED   ", "1-DIST ", pattern_squared   },
    [PATTERN_CAPPED]    = { "CAPPED    ", "1-DIST ", pattern_capped    },
    /* Tier 2 — BANDS */
    [PATTERN_RINGS]     = { "RINGS     ", "2-BAND ", pattern_rings     },
    [PATTERN_SAW]       = { "SAW       ", "2-BAND ", pattern_saw       },
    [PATTERN_ZEBRA]     = { "ZEBRA     ", "2-BAND ", pattern_zebra     },
    [PATTERN_WAVE]      = { "WAVE      ", "2-BAND ", pattern_wave      },
    [PATTERN_SHELLS]    = { "SHELLS    ", "2-BAND ", pattern_shells    },
    /* Tier 3 — VORONOI */
    [PATTERN_CELLS]     = { "CELLS     ", "3-VOR  ", pattern_cells     },
    [PATTERN_CRACKLE]   = { "CRACKLE   ", "3-VOR  ", pattern_crackle   },
    [PATTERN_SPECKLE]   = { "SPECKLE   ", "3-VOR  ", pattern_speckle   },
    [PATTERN_RAINBOW]   = { "RAINBOW   ", "3-VOR  ", pattern_rainbow   },
    [PATTERN_TWINKLE]   = { "TWINKLE   ", "3-VOR  ", pattern_twinkle   },
    /* Tier 4 — CONTOURS */
    [PATTERN_OUTLINE]   = { "OUTLINE   ", "4-CONT ", pattern_outline   },
    [PATTERN_DUAL]      = { "DUAL      ", "4-CONT ", pattern_dual      },
    [PATTERN_SHELL]     = { "SHELL     ", "4-CONT ", pattern_shell     },
    [PATTERN_HALO]      = { "HALO      ", "4-CONT ", pattern_halo      },
    [PATTERN_AURA]      = { "AURA      ", "4-CONT ", pattern_aura      },
    /* Tier 5 — STRUCTURE */
    [PATTERN_EDGES]     = { "EDGES     ", "5-STRC ", pattern_edges     },
    [PATTERN_VERTICES]  = { "VERTICES  ", "5-STRC ", pattern_vertices  },
    [PATTERN_INTERIOR]  = { "INTERIOR  ", "5-STRC ", pattern_interior  },
    [PATTERN_WEAVE]     = { "WEAVE     ", "5-STRC ", pattern_weave     },
    [PATTERN_SKELETON]  = { "SKELETON  ", "5-STRC ", pattern_skeleton  },
    /* Tier 6 — ANIMATED */
    [PATTERN_PULSE]     = { "PULSE     ", "6-ANIM ", pattern_pulse     },
    [PATTERN_SCAN]      = { "SCAN      ", "6-ANIM ", pattern_scan      },
    [PATTERN_ECHO]      = { "ECHO      ", "6-ANIM ", pattern_echo      },
    [PATTERN_PLASMA]    = { "PLASMA    ", "6-ANIM ", pattern_plasma    },
    [PATTERN_CHAOS]     = { "CHAOS     ", "6-ANIM ", pattern_chaos     },
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
/* §7  scene — JFAGrid + Seeds + GlowField + GlyphRamp                    */
/*             + PatternState + PaletteState + Scene                       */
/* ===================================================================== */

/*
 * GlowField — the per-cell output grid the pipeline writes into and
 * the renderer reads from.
 *
 * INTENT
 *   Decouple "compute the scalar field" (scene_evaluate_field, runs
 *   on every sim tick) from "paint the scalar field" (scene_draw,
 *   runs on every render frame). The GlowField is the snapshot the
 *   renderer reads in between ticks.
 *
 * CONTEXT
 *   Lives on Scene. Written by scene_evaluate_field() once per sim
 *   tick after jfa_grid_run has finished; read by scene_draw() once
 *   per render frame. Index with glow_field_idx(gf, x, y) = y·w + x.
 *
 * MEMBER LOGIC
 *   w, h    : Grid dimensions in cells. Match the JFAGrid's dims.
 *   count   : w · h, cached so the buffer-clearing loop and the
 *             evaluator don't recompute it each tick.
 *   glow[]  : Per-cell scalar ∈ [0, 1]. Producer: scene_evaluate_field
 *             dispatches the active pattern's sampler at every cell
 *             and writes here. Consumer: scene_draw → GlyphRamp.
 *   band[]  : Per-cell palette band ∈ {0..3} (quartile of glow).
 *             Computed once per tick to save the renderer the work.
 */
typedef struct {
    int      w, h;                   /* grid dims (cells), set at reset */
    int      count;                  /* w * h, cached for tight loops   */
    float    glow[CELLS_MAX];        /* per-cell scalar ∈ [0, 1]        */
    uint8_t  band[CELLS_MAX];        /* per-cell palette tier ∈ {0..3}  */
} GlowField;

static inline int glow_field_idx(const GlowField *gf, int x, int y)
{
    return y * gf->w + x;
}

static void glow_field_reset(GlowField *gf, int w, int h)
{
    gf->w     = w;
    gf->h     = h;
    gf->count = w * h;
    for (int i = 0; i < gf->count; i++) {
        gf->glow[i] = 0.0f;
        gf->band[i] = 0;
    }
}

/*
 * GlyphRamp — the density → (glyph, attr) mapping. Captures "how
 * density becomes a character" as a NAMED RULE rather than an inline
 * if/else chain in the renderer.
 *
 *   glow > thresh_high  →  glyph_high  bold     (dense core)
 *   glow > thresh_mid   →  glyph_mid   bold     (mid density)
 *   glow > thresh_low   →  glyph_low   normal   (faint trace)
 *   else                →  not drawn            (transparent)
 *
 * INVARIANT  thresh_high > thresh_mid > thresh_low > 0.
 * GLYPHS     '#', '*', '.' — coarse 3-tier subset of Paul Bourke's
 *            canonical luminance ramp "@%#*+=-:. " (densest → sparsest).
 */
typedef struct {
    float thresh_high;    /* > this → glyph_high  (default 0.65)  */
    float thresh_mid;     /* > this → glyph_mid   (default 0.30)  */
    float thresh_low;     /* > this → glyph_low   (default 0.05)  */
    char  glyph_high;     /* dense core    glyph                  */
    char  glyph_mid;      /* mid-density   glyph                  */
    char  glyph_low;      /* faint-trace   glyph                  */
} GlyphRamp;

/*
 * GlyphChoice — return value from glyph_ramp_pick(). Reads as a pure
 * function: GlyphChoice gc = glyph_ramp_pick(ramp, glow). Caller
 * composes (glyph, attr) with the palette band index they pulled
 * from elsewhere — GlyphRamp deliberately doesn't know about colour.
 */
typedef struct {
    char glyph;         /* the char to draw — only valid if visible   */
    int  attr;          /* ncurses attribute: A_BOLD or A_NORMAL      */
    bool visible;       /* false → leave the cell empty (transparent) */
} GlyphChoice;

static void glyph_ramp_init(GlyphRamp *gr)
{
    gr->thresh_high = GLYPH_HIGH_THRESH;
    gr->thresh_mid  = GLYPH_MID_THRESH;
    gr->thresh_low  = GLOW_THRESHOLD;
    gr->glyph_high  = '#';
    gr->glyph_mid   = '*';
    gr->glyph_low   = '.';
}

static GlyphChoice glyph_ramp_pick(const GlyphRamp *gr, float glow)
{
    GlyphChoice c = { .glyph = ' ', .attr = A_NORMAL, .visible = false };
    if      (glow > gr->thresh_high) { c.glyph = gr->glyph_high; c.attr = A_BOLD;   c.visible = true; }
    else if (glow > gr->thresh_mid)  { c.glyph = gr->glyph_mid;  c.attr = A_BOLD;   c.visible = true; }
    else if (glow > gr->thresh_low)  { c.glyph = gr->glyph_low;  c.attr = A_NORMAL; c.visible = true; }
    return c;
}

/*
 * PatternState — active pattern + how the seed wobble animates.
 *
 *   current    : Active pattern enum, indexes noise_patterns[].
 *                Cycled by n/N (forward) and p/P (back).
 *   wobble_time: Wobble-phase accumulator (radians). Advanced each
 *                tick by:  wobble_time += WOBBLE_RATE · drift_mult · dt
 *                Fed into seeds_animate to drive seed motion. Reset
 *                only by r (scene_reset).
 *   drift_mult : User-controlled wobble-rate multiplier in powers of
 *                2 ∈ [DRIFT_MULT_MIN, DRIFT_MULT_MAX]. Stepped *=2
 *                / /=2 by +/- so the user perceives discrete speed
 *                jumps instead of a continuous slider.
 */
typedef struct {
    Pattern current;       /* active pattern, indexes noise_patterns[]      */
    float   wobble_time;   /* wobble-phase accumulator (radians)            */
    int     drift_mult;    /* powers of 2 ∈ [DRIFT_MULT_MIN, DRIFT_MULT_MAX] */
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current     = PATTERN_SDF;
    ps->wobble_time = 0.0f;
    ps->drift_mult  = DRIFT_MULT_DEF;
}

/*
 * PaletteState — the active theme index. One-int wrapper kept as a
 * named struct so "which colour scheme is showing" has a stable
 * home on Scene. On every change, the new theme's xterm indices are
 * pushed into ncurses colour pairs via theme_apply() (side effect
 * outside the struct, since ncurses owns the actual palette state).
 *
 *   current : Index into themes[] ∈ [0, N_THEMES). Cycled by t/T.
 */
typedef struct {
    int current;          /* index into themes[] ∈ [0, N_THEMES) */
} PaletteState;

static void palette_state_init(PaletteState *p)
{
    p->current = 0;
}

/*
 * Scene — composite owner of ALL mutable simulation state.
 *
 * THE PIPELINE (top to bottom = dataflow order)
 *
 *   grid     — JFA result grid                       (the ALGORITHM)
 *               ↑ (rebuilt every tick by jfa_grid_run)
 *   seeds    — seed positions + wobble phases        (the DATA)
 *               ↑ (re-animated every tick by seeds_animate)
 *   pattern  — active pattern + wobble accumulator   (the ANIMATION)
 *               ↓ (drives scene_evaluate_field)
 *   field    — per-cell glow + band                  (the OUTPUT BUFFER)
 *               ↓ (read by scene_draw)
 *   ramp     — density → glyph mapping               (the RENDER RULE)
 *               + palette colour pairs (via ncurses, set by theme_apply)
 *               ↓
 *   palette  — active theme index                    (the COLOUR CHOICE)
 *               + paused flag                        (the CONTROL)
 *
 *   The whole pipeline at a glance: seeds → JFA grid → glow field
 *   (via pattern) → ramp + palette (via renderer).
 *
 * LIFETIME
 *   scene_init   — zero-init, set defaults, scatter seeds, run full
 *                  JFA + initial field evaluation.
 *   scene_reset  — clear field, zero wobble, re-scatter seeds, run
 *                  full JFA + initial evaluation. Called on r.
 *   scene_tick   — advance wobble, re-animate seeds, re-run JFA,
 *                  re-evaluate field.
 *   No teardown — Scene lives in BSS via App; OS reclaims it.
 */
typedef struct {
    JFAGrid       grid;            /* §5a — JFA result grid (~90 KB)         */
    Seed          seeds[N_SEEDS];  /* §5b — seed positions + wobble state    */
    GlowField     field;           /* per-cell (glow, band) — ~88 KB         */
    GlyphRamp     ramp;            /* density → glyph rule (read-mostly)     */
    PatternState  pattern;         /* active pattern + wobble + speed        */
    PaletteState  palette;         /* active theme index                     */
    bool          paused;          /* if true, scene_tick early-returns      */
} Scene;

/* glow ∈ [0, 1] → palette band ∈ {0, 1, 2, 3}. Quantises the
 * continuous scalar into a discrete colour tier. The 3.999f "almost
 * 4" gain keeps glow == 1.0 from wrapping past band 3. */
static inline uint8_t glow_to_palette_band(float glow)
{
    return (uint8_t)((int)(glow * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK);
}

/*
 * scene_recompute_jfa — re-place the seeds at their current wobble
 * positions and rebuild the JFA grid from them. Cost ≈ 800 K
 * cell-comparisons at 200×56 with log₂(200)=8 passes — well under
 * the 16 ms render budget at 60 Hz.
 */
static void scene_recompute_jfa(Scene *s)
{
    seeds_animate(s->seeds, N_SEEDS, s->grid.w, s->grid.h,
                  s->pattern.wobble_time);
    jfa_grid_run(&s->grid, s->seeds, N_SEEDS);
}

/* glow_field_sample_cell — one inner step of scene_evaluate_field.
 *
 *   call active pattern at (x, y, t)  →  (raw_glow, raw_band)
 *   clamp glow to [0, 1]              → safe to feed GlyphRamp
 *   mask band by PALETTE_BAND_MASK    → safe to index colour pairs
 *   write into GlowField[x, y]
 */
static inline void glow_field_sample_cell(
    GlowField *field, int x, int y,
    JFAPatternFn sample_pattern,
    const JFAGrid *grid, const Seed *seeds, float t)
{
    float raw_glow = 0.0f;
    int   raw_band = 0;
    sample_pattern(grid, seeds, N_SEEDS, x, y, t, &raw_glow, &raw_band);

    int idx          = glow_field_idx(field, x, y);
    field->glow[idx] = clampf(raw_glow, 0.0f, 1.0f);
    field->band[idx] = (uint8_t)(raw_band & PALETTE_BAND_MASK);
}

/*
 * scene_evaluate_field — drive the pattern → glow pipeline for one
 * sim tick. Assumes the JFA grid + seeds are already current (caller
 * arranges that via scene_recompute_jfa).
 *
 *   STEP 1 — RESOLVE the active pattern's sampler from the dispatch table
 *   STEP 2 — SAMPLE every cell: pattern → (glow, band) → GlowField
 *
 * Hot loop — one pattern call per cell × ~11K cells × 60 Hz.  Inner
 * body kept tight via `static inline` helpers.
 */
static void scene_evaluate_field(Scene *s)
{
    /* STEP 1 — RESOLVE the active pattern's sampler */
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    JFAPatternFn   sample_pattern = noise_patterns[active].sample;
    const JFAGrid *grid           = &s->grid;
    const Seed    *seeds          = s->seeds;
    GlowField     *field          = &s->field;
    float          t              = s->pattern.wobble_time;

    /* STEP 2 — SAMPLE every cell into the GlowField */
    for (int y = 0; y < field->h; y++) {
        for (int x = 0; x < field->w; x++) {
            glow_field_sample_cell(field, x, y,
                                   sample_pattern, grid, seeds, t);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->grid.w = mw;
    s->grid.h = mh;
    s->pattern.wobble_time = 0.0f;
    seeds_scatter(s->seeds, N_SEEDS, mw, mh);
    scene_recompute_jfa  (s);
    scene_evaluate_field (s);     /* paint initial frame so paused looks ok */
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    glyph_ramp_init   (&s->ramp);
    pattern_state_init(&s->pattern);
    palette_state_init(&s->palette);
    s->paused = false;
    scene_reset(s, mw, mh);
}

/*
 * scene_tick — drive one sim step.
 *
 * The full SDF/JFA pipeline runs every tick (continuous wobble
 * animation means no shortcut):
 *   1. advance wobble accumulator
 *   2. re-animate seed positions
 *   3. re-run full JFA
 *   4. re-evaluate the active pattern → GlowField
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->pattern.wobble_time += WOBBLE_RATE * (float)s->pattern.drift_mult * dt;
    scene_recompute_jfa (s);
    scene_evaluate_field(s);
}

/* ===================================================================== */
/* §8  screen — terminal viewport + scene_draw + HUD                       */
/* ===================================================================== */

/*
 * Screen — the terminal viewport's dimensions, cached from ncurses.
 *
 * INTENT
 *   Decouple "what the terminal is" from "what the simulation is".
 *   ncurses owns the actual screen buffer, the input loop, and the
 *   colour-pair table; Screen just remembers cols × rows so we
 *   don't call getmaxyx() once per cell. Separating from Scene also
 *   matches the pattern in sibling demos where mock-rendering paths
 *   need terminal dims without simulation state.
 *
 * CONTEXT
 *   One instance lives on App (§9). Written by screen_init() (on
 *   startup) and screen_resize() (on every SIGWINCH); read by
 *   compute_grid_placement() and screen_draw() each frame to
 *   right-align the state bar and centre the GlowField.
 *
 * LIFETIME
 *   screen_init   — calls initscr(), configures ncurses, captures
 *                   cols/rows.
 *   screen_resize — re-captures cols/rows after a SIGWINCH. Owner
 *                   (App) follows up with scene_reset to re-size
 *                   the GlowField + JFAGrid.
 *   screen_free   — calls endwin() to restore the terminal.
 *
 * MEMBER LOGIC
 *   cols : Terminal width in cells. ≥ 16 enforced by
 *          app_pick_map_size — smaller and the HUD title chip plus
 *          state bar won't fit on the same row.
 *   rows : Terminal height in cells. ≥ 8 enforced by
 *          app_pick_map_size; HUD reserves HUD_TOP_ROWS + 1 rows
 *          (title + status above, action hint below).
 *   Ordering matches the ncurses getmaxyx(stdscr, rows, cols)
 *   convention — rows first because curses originally targeted
 *   line printers (lines are the primary axis).
 */
typedef struct {
    int cols;     /* terminal width  in cells (≥ 16)  */
    int rows;     /* terminal height in cells (≥ 8)   */
} Screen;

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

/* ---------- scene_draw: grid → glyphs --------------------------------- */

/*
 * GridPlacement — the top-left screen cell where the field's (0, 0)
 * cell will be drawn. The result of centering the GlowField inside
 * the terminal viewport, holding the HUD rows clear.
 *
 * INTENT
 *   Lift the centering math out of the per-cell inner loop. Without
 *   GridPlacement the loop would have to redo `(cols - field_w) / 2`
 *   plus the HUD-row offset on every cell — w·h times per frame
 *   (~11K calls). With it, the loop body becomes `screen_y =
 *   origin_y + y` which the compiler folds into the loop induction
 *   variable.
 *
 *   Also names the "where on screen" decision — a reader sees
 *   `compute_grid_placement(...)` and immediately knows what's
 *   happening instead of staring at four lines of `/ 2` arithmetic
 *   inline.
 *
 * CONTEXT
 *   Stack-local — computed once at the top of scene_draw() (§8) by
 *   compute_grid_placement(field_w, field_h, cols, rows) and then
 *   passed implicitly via the local `place` to every cell.
 *
 *   Coordinate convention matches ncurses: (origin_y, origin_x) is
 *   the (row, col) of the top-left screen cell to start drawing at.
 *   Y grows DOWN (towards the bottom of the terminal), X grows RIGHT.
 *
 * INVARIANTS
 *   origin_x ≥ 0
 *   origin_y ≥ HUD_TOP_ROWS         (kept clear for the title bar)
 *   The bottom HUD_BOTTOM_ROWS row is kept clear by sizing the
 *   field, not by clamping here — so origin_y + field_h may run up
 *   to (rows - HUD_BOTTOM_ROWS).
 *
 *   Negative results from the centering arithmetic (over-large
 *   field on a small viewport) are clamped to the viewport edge —
 *   the field is cropped, not wrapped. The "cell off-screen?" check
 *   in the drawer's inner loop handles the rest.
 *
 * MEMBER LOGIC
 *   origin_x : Screen column (cells) of the field's leftmost cell.
 *              0 if the field is wider than the viewport.
 *   origin_y : Screen row (cells) of the field's topmost cell.
 *              HUD_TOP_ROWS at minimum (never overlaps the title bar).
 */
typedef struct {
    int origin_x;   /* screen column where field's left edge starts   */
    int origin_y;   /* screen row    where field's top edge starts    */
} GridPlacement;

static GridPlacement compute_grid_placement(int field_w, int field_h,
                                            int cols, int rows)
{
    GridPlacement p;
    p.origin_x = (cols - field_w) / 2;
    p.origin_y = ((rows - HUD_BAND_RESERVED_ROWS) - field_h) / 2
                 + HUD_TOP_ROWS;
    if (p.origin_x < 0)            p.origin_x = 0;
    if (p.origin_y < HUD_TOP_ROWS) p.origin_y = HUD_TOP_ROWS;
    return p;
}

/* The one-cell paint primitive: bind colour, emit glyph, unbind. */
static inline void draw_glyph_at(int screen_y, int screen_x,
                                 char glyph, int color_pair, int attr)
{
    attron(COLOR_PAIR(color_pair) | attr);
    mvaddch(screen_y, screen_x, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(color_pair) | attr);
}

/* band_to_color_pair — quantised band index → ncurses colour-pair
 * id.  PAIR_BAND_BASE is the first band pair (§1), bands 0..3 take
 * the next four slots.  The mask is belt-and-braces; the field's
 * band[] is already masked at write time. */
static inline int band_to_color_pair(uint8_t band)
{
    return PAIR_BAND_BASE + (band & PALETTE_BAND_MASK);
}

/* glow_field_paint_cell — one inner step of scene_draw.  Read the
 * GlowField at (x, y), pick a glyph via the ramp, and (if visible)
 * emit a coloured character at screen position (screen_x, screen_y). */
static inline void glow_field_paint_cell(
    const GlowField *field, const GlyphRamp *ramp,
    int x, int y, int screen_x, int screen_y)
{
    int   cell_idx = glow_field_idx(field, x, y);
    float glow     = field->glow[cell_idx];

    GlyphChoice pick = glyph_ramp_pick(ramp, glow);
    if (!pick.visible) return;

    int color_pair = band_to_color_pair(field->band[cell_idx]);
    draw_glyph_at(screen_y, screen_x, pick.glyph, color_pair, pick.attr);
}

/*
 * scene_draw — paint the GlowField into the terminal using the
 * Scene's GlyphRamp and the currently-bound colour palette.
 *
 *   STEP 1 — PLACE: centre the field inside the viewport (HUD-aware)
 *   STEP 2 — PAINT: for every cell in bounds, draw via the ramp
 *
 * Pure read of Scene state — no simulation here.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const GlowField *field = &s->field;
    const GlyphRamp *ramp  = &s->ramp;

    /* STEP 1 — compute the field's top-left on screen */
    GridPlacement place = compute_grid_placement(field->w, field->h,
                                                 cols, rows);

    /* STEP 2 — paint every in-bounds cell */
    for (int y = 0; y < field->h; y++) {
        int screen_y = place.origin_y + y;
        if (screen_y < 0 || screen_y >= rows) continue;
        for (int x = 0; x < field->w; x++) {
            int screen_x = place.origin_x + x;
            if (screen_x < 0 || screen_x >= cols) continue;

            glow_field_paint_cell(field, ramp, x, y, screen_x, screen_y);
        }
    }
}

/* ---------- HUD layout widths ----------------------------------------- */

#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " SDF / JUMP FLOOD "
#define HUD_BOTTOM_HINT_TEXT \
    " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit "

/* ---------- HUD: per-segment drawers ---------------------------------- *
 *
 * Each segment takes (row, x, ...) and returns the next x cursor
 * position so the caller can chain them. Names describe what's
 * INSIDE the segment, not how it looks.
 */

/* Row 0 LEFT — the program title chip. */
static int hud_draw_title_chip(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, "%s", HUD_TITLE_TEXT);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + (int)strlen(HUD_TITLE_TEXT);
}

/* Row 0 RIGHT — fps + Hz + state + pattern index + drift multiplier. */
static void hud_draw_state_bar(int row, int cols,
                               double fps, int sim_fps,
                               const PatternState *ps, bool paused)
{
    const char *state_text = paused ? "PAUSED    " : pattern_name(ps->current);
    char        buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  drift:x%-2d ",
             fps, sim_fps, state_text,
             (int)ps->current + 1, N_PATTERNS,
             ps->drift_mult);
    int right_aligned_x = cols - (int)strlen(buf);
    if (right_aligned_x < 0) right_aligned_x = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, right_aligned_x, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 segment — "pattern:<NAME>" */
static int hud_draw_pattern_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-10s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_PATTERN_FIELD;
}

/* Row 1 segment — "tier:<N-LABEL>" */
static int hud_draw_tier_field(int row, int x, Pattern p)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " tier:%-7s ", pattern_tier(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_TIER_FIELD;
}

/* Row 1 segment — "theme:<NAME>" */
static int hud_draw_theme_field(int row, int x, int theme_idx)
{
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_W_THEME_FIELD;
}

/* Row 1 segment — "palette:" label + N_PALETTE_BANDS coloured swatches.
 * Each swatch is a '#' rendered in the corresponding band's colour pair
 * so the user can see at a glance what each density-tier looks like in
 * the active theme. */
static int hud_draw_palette_swatches(int row, int x)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_W_PALETTE_LABEL;
    for (int band = 0; band < N_PALETTE_BANDS; band++) {
        int color_pair = PAIR_BAND_BASE + band;
        attron(COLOR_PAIR(color_pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(color_pair) | A_BOLD);
        x += 1;
    }
    return x;
}

/* Row 1 tail — algorithm parameter readout: seed count + grid dims. */
static void hud_draw_stats_field(int row, int x, const GlowField *gf)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  seeds:%d  map:%dx%d ", N_SEEDS, gf->w, gf->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — keymap reminder. Fixed text; cyan + bold for contrast. */
static void hud_draw_action_hint(int row)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(row, 0, "%s", HUD_BOTTOM_HINT_TEXT);
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ---------- screen_draw: scene + full HUD ----------------------------- */

/*
 * screen_draw — per-frame composite renderer. Pure read of state.
 *
 *   STEP 1  erase  — ncurses double-buffer
 *   STEP 2  scene  — paint the GlowField
 *   STEP 3  HUD row 0 — title (left) + state bar (right)
 *   STEP 4  HUD row 1 — pattern | tier | theme | swatches | stats
 *   STEP 5  HUD last row — action hint
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    /* Row 0 — title + state bar */
    hud_draw_title_chip(HUD_TITLE_ROW, HUD_LEFT_MARGIN);
    hud_draw_state_bar (HUD_TITLE_ROW, sc->cols, fps, sim_fps,
                        &s->pattern, s->paused);

    /* Row 1 — chained left-to-right segments */
    int x = HUD_LEFT_MARGIN;
    x = hud_draw_pattern_field   (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_tier_field      (HUD_STATUS_ROW, x, s->pattern.current);
    x = hud_draw_theme_field     (HUD_STATUS_ROW, x, s->palette.current);
    x = hud_draw_palette_swatches(HUD_STATUS_ROW, x);
    hud_draw_stats_field         (HUD_STATUS_ROW, x, &s->field);

    /* Bottom row — keymap hint */
    hud_draw_action_hint(sc->rows - 1);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app — App harness + FrameClock + main loop                          */
/* ===================================================================== */

/*
 * App — the top-level harness. Owns every other piece of state.
 *
 * SINGLETON
 *   One file-scope instance (g_app). Signal handlers need to reach
 *   the `running` / `need_resize` flags from outside any caller's
 *   scope; the handlers ONLY touch the two volatile sig_atomic_t
 *   flags, the main loop owns everything else.
 *
 * MEMBER LOGIC
 *   scene       : The simulation. See Scene in §7.
 *   screen      : Terminal cols/rows. Refreshed on every SIGWINCH.
 *   sim_fps     : Tick rate in Hz. ] / [ adjusts in SIM_FPS_STEP-Hz
 *                 increments, clamped to [SIM_FPS_MIN, SIM_FPS_MAX].
 *   map_w,      : Grid dims selected by app_pick_map_size() from the
 *   map_h         terminal cols/rows minus HUD reservations.
 *   running     : Cleared by SIGINT/SIGTERM or q/ESC.
 *   need_resize : Set by SIGWINCH; consumed by main loop.
 *
 *   The volatile sig_atomic_t types are required: POSIX only
 *   guarantees sig_atomic_t writes are async-safe inside a signal
 *   handler, and `volatile` prevents the compiler from caching the
 *   read in the main loop.
 */
typedef struct {
    Scene                 scene;        /* the simulation                       */
    Screen                screen;       /* terminal dims, refreshed on resize   */
    int                   sim_fps;      /* tick rate Hz, user-adjustable by ]/[ */
    int                   map_w;        /* chosen grid width  (cells)           */
    int                   map_h;        /* chosen grid height (cells)           */
    volatile sig_atomic_t running;      /* main-loop flag, 0 → exit             */
    volatile sig_atomic_t need_resize;  /* SIGWINCH→1, main loop consumes       */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
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

/* ---------- Scene action helpers (input → state mutation) ------------- *
 *
 * Each helper names ONE keystroke-triggered state change, so
 * app_handle_key reads as a keymap rather than a switch full of
 * arithmetic. Direction parameters use ±1 throughout for "next/prev"
 * symmetry.
 */

/* Cycle the active pattern by ±1 with wraparound through N_PATTERNS. */
static void scene_cycle_pattern(Scene *s, int direction)
{
    int next = ((int)s->pattern.current + direction + N_PATTERNS) % N_PATTERNS;
    s->pattern.current = (Pattern)next;
}

/* Cycle the active theme by ±1 with wraparound, then rebind ncurses
 * colour pairs to the new palette via theme_apply (side effect). */
static void scene_cycle_theme(Scene *s, int direction)
{
    s->palette.current = (s->palette.current + direction + N_THEMES)
                         % N_THEMES;
    theme_apply(s->palette.current);
}

/* Drift speed control — clean ×2 / ÷2 steps so the user perceives
 * discrete speed changes, not a slider. */
static void scene_drift_double(PatternState *ps)
{
    if (ps->drift_mult < DRIFT_MULT_MAX) ps->drift_mult *= 2;
    if (ps->drift_mult > DRIFT_MULT_MAX) ps->drift_mult = DRIFT_MULT_MAX;
}
static void scene_drift_halve(PatternState *ps)
{
    ps->drift_mult /= 2;
    if (ps->drift_mult < DRIFT_MULT_MIN) ps->drift_mult = DRIFT_MULT_MIN;
}

/* Adjust sim tick rate by delta Hz, clamped to [SIM_FPS_MIN, MAX]. */
static void app_adjust_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

/*
 * app_handle_key — the keymap. Returns false on quit, true otherwise.
 * Every arm is a single named action so the switch reads as
 * "key → behaviour" without arithmetic inline.
 */
static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':            s->paused = !s->paused;                 break;
    case 'r': case 'R':  scene_reset(s, app->map_w, app->map_h); break;

    case '=': case '+':  scene_drift_double(&s->pattern);        break;
    case '-':            scene_drift_halve (&s->pattern);        break;

    case ']':            app_adjust_sim_fps(app, +SIM_FPS_STEP); break;
    case '[':            app_adjust_sim_fps(app, -SIM_FPS_STEP); break;

    case 't':            scene_cycle_theme  (s, +1);             break;
    case 'T':            scene_cycle_theme  (s, -1);             break;

    case 'n': case 'N':  scene_cycle_pattern(s, +1);             break;
    case 'p': case 'P':  scene_cycle_pattern(s, -1);             break;

    default:                                                     break;
    }
    return true;
}

/* ---------- FrameClock: timing accumulators for the main loop --------- */

/*
 * FrameClock — every running scalar the main loop's timing needs,
 * grouped under one name. Groups TWO independent integrator patterns
 * that share `frame_dt_ns` as their input:
 *
 *   1) FIXED-TIMESTEP SIM DRIVER  (sim_accum_ns)
 *      Each frame's wall-clock dt is poured into the accumulator;
 *      the accumulator is drained one TICK_NS chunk at a time into
 *      scene_tick() calls. The sim sees a STABLE per-tick dt
 *      regardless of frame jitter. (Glenn Fiedler, "Fix Your
 *      Timestep!")
 *
 *   2) FPS METER                  (fps_accum_ns + fps_frame_count)
 *      Sum frame dts and count frames; every FPS_UPDATE_MS of wall
 *      clock, divide frames/seconds to refresh `fps_display`. The
 *      HUD reads `fps_display` only — never the running accums.
 *
 * MEMBER LOGIC
 *   frame_time_ns   : Wall clock at the START of the current frame.
 *                     Next frame's dt = clock_ns() − frame_time_ns.
 *   sim_accum_ns    : Wall-clock ns received from frames but not yet
 *                     drained into sim ticks.
 *   fps_accum_ns    : Wall-clock ns since the last FPS readout
 *                     refresh.
 *   fps_frame_count : Frames since the last FPS readout refresh.
 *   fps_display     : Last computed instantaneous frame rate (Hz) —
 *                     the ONE member the HUD actually reads.
 */
typedef struct {
    int64_t frame_time_ns;     /* wall clock at start of current frame    */
    int64_t sim_accum_ns;      /* unconsumed time → sim ticks (Fiedler)   */
    int64_t fps_accum_ns;      /* ns since last FPS recomputation         */
    int     fps_frame_count;   /* frames since last FPS recomputation     */
    double  fps_display;       /* last computed FPS — the value HUD reads */
} FrameClock;

static void frame_clock_init(FrameClock *fc)
{
    fc->frame_time_ns   = clock_ns();
    fc->sim_accum_ns    = 0;
    fc->fps_accum_ns    = 0;
    fc->fps_frame_count = 0;
    fc->fps_display     = 0.0;
}

/* Re-anchor the clock after a stall (e.g. SIGWINCH resize) so we don't
 * burst-tick to "catch up" on time the user wasn't seeing the sim. */
static void frame_clock_resync(FrameClock *fc)
{
    fc->frame_time_ns = clock_ns();
    fc->sim_accum_ns  = 0;
}

/* Advance the frame clock; return dt since previous frame, capped at
 * MAX_FRAME_DT_NS to prevent the spiral-of-death failure mode. */
static int64_t frame_clock_advance(FrameClock *fc)
{
    int64_t now = clock_ns();
    int64_t dt  = now - fc->frame_time_ns;
    fc->frame_time_ns = now;
    if (dt > MAX_FRAME_DT_NS) dt = MAX_FRAME_DT_NS;
    return dt;
}

/* FPS meter — accumulate frame dt; when FPS_UPDATE_MS worth has piled
 * up, divide frames by elapsed seconds and refresh the display value. */
static void fps_meter_observe(FrameClock *fc, int64_t frame_dt_ns)
{
    fc->fps_frame_count++;
    fc->fps_accum_ns += frame_dt_ns;
    if (fc->fps_accum_ns >= FPS_UPDATE_MS * NS_PER_MS) {
        double elapsed_sec = (double)fc->fps_accum_ns / (double)NS_PER_SEC;
        fc->fps_display     = (double)fc->fps_frame_count / elapsed_sec;
        fc->fps_frame_count = 0;
        fc->fps_accum_ns    = 0;
    }
}

/* ---------- Main-loop step helpers ----------------------------------- */

/* Fiedler's fixed-timestep tick drain. Pour this frame's dt into the
 * accumulator, then consume it one TICK_NS chunk at a time, calling
 * scene_tick() with a STABLE per-tick dt — so the simulation behaves
 * identically regardless of render-frame jitter. */
static void app_run_fixed_step_ticks(App *app, FrameClock *fc,
                                     int64_t frame_dt_ns)
{
    int64_t tick_ns     = TICK_NS(app->sim_fps);
    float   tick_dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    fc->sim_accum_ns += frame_dt_ns;
    while (fc->sim_accum_ns >= tick_ns) {
        scene_tick(&app->scene, tick_dt_sec);
        fc->sim_accum_ns -= tick_ns;
    }
}

/* Sleep so the render loop spins at RENDER_FPS_TARGET. */
static void app_throttle_to_render_rate(int64_t frame_start_ns,
                                        int64_t frame_dt_ns)
{
    int64_t target_frame_period_ns = NS_PER_SEC / RENDER_FPS_TARGET;
    int64_t time_consumed_ns       = clock_ns() - frame_start_ns
                                   + frame_dt_ns;
    clock_sleep_ns(target_frame_period_ns - time_consumed_ns);
}

/* Drain one key event (non-blocking) and route it to app_handle_key. */
static void app_pump_input(App *app)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
        app->running = 0;
}

/* Wire SIGINT / SIGTERM → exit flag, SIGWINCH → resize flag. */
static void app_install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/*
 * main — top-level driver. Reads as pseudocode:
 *
 *   PHASE 1 bootstrap     (PRNG seed, signal handlers, atexit cleanup)
 *   PHASE 2 init          (terminal → map size → simulation)
 *   PHASE 3 main loop     (resize → frame_dt → fixed-step ticks →
 *                          fps meter → throttle → draw → input)
 *   PHASE 4 shutdown      (restore terminal)
 */
int main(void)
{
    /* PHASE 1 — bootstrap */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    app_install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    /* PHASE 2 — terminal + simulation init */
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    /* PHASE 3 — main loop */
    FrameClock clock;
    frame_clock_init(&clock);

    while (app->running) {
        if (app->need_resize) {
            app_do_resize(app);
            frame_clock_resync(&clock);
        }

        int64_t frame_dt_ns = frame_clock_advance(&clock);
        app_run_fixed_step_ticks   (app, &clock, frame_dt_ns);
        fps_meter_observe          (&clock, frame_dt_ns);
        app_throttle_to_render_rate(clock.frame_time_ns, frame_dt_ns);

        screen_draw(&app->screen, &app->scene,
                    clock.fps_display, app->sim_fps);
        screen_present();
        app_pump_input(app);
    }

    /* PHASE 4 — shutdown */
    screen_free(&app->screen);
    return 0;
}
