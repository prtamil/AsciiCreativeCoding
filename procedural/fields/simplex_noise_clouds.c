/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * simplex_noise_clouds.c
 *   — Ken Perlin's simplex noise, thirty patterns across six tiers.
 *
 * DEMO: A flowing scalar field generated from 2-D simplex noise — the
 *       improved-noise variant Perlin published in 2001 to fix the
 *       directional bias and grid artefacts of his original 1985
 *       Perlin noise. Thirty patterns, grouped into six tiers ordered
 *       simple → complex, visualise the same noise primitive through
 *       progressively more elaborate mappings:
 *         Tier 1 RAW       — direct noise (CLOUDS, BILLOW, RIDGED,
 *                            WISPS, CRESTS)
 *         Tier 2 MAPPED    — sin/cos/pow transforms of fBm (CONTOURS,
 *                            MARBLE, ZEBRA, RIPPLES, THRESHOLD)
 *         Tier 3 TURBULENCE— Σ|octave| stacks (TURBULENCE, STORM,
 *                            INFERNO, VEINS, EMBERS)
 *         Tier 4 WARPED    — domain warping (WARP, WHIRL, DUNES,
 *                            CURRENTS, FRACTAL)
 *         Tier 5 COMPOSITE — multi-field combine (NEBULA, AURORA,
 *                            PLASMA, LIGHTNING, GALAXY)
 *         Tier 6 MASKED    — noise × spatial mask (SOLAR, MOSAIC,
 *                            VIGNETTE, COSMOS, SUPERNOVA)
 *       Field drifts so all patterns evolve. Cycle patterns with n/p,
 *       themes with t/T.
 *
 * Study alongside:
 *   ./perin_noise_flow_showcase.c — Perlin (1985) gradient noise;
 *       the predecessor to simplex. Shape similarities; simplex is
 *       faster and less directionally biased.
 *   ./domain_warped_noise_iq_style.c — domain warping, also Perlin-
 *       based. Different "what to do with the noise" technique.
 *   ./worley_cellular_noise.c — cellular noise instead of gradient.
 *
 * Section map:
 *   §1 config   — grid, patterns, palette, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 noise    — Simplex 2-D + fBm
 *   §6 patterns — 30 noise mappings in 6 tiers + dispatch table
 *   §7 scene    — Field, scene state, per-frame grid update
 *   §8 screen   — ASCII render: density-graded glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new permutation)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra simplex_noise_clouds.c \
 *       -o simplex_clouds -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Simplex noise (Perlin 2001). The mathematical fix
 *                  for several issues with the original 1985 Perlin
 *                  noise:
 *                    • PERLIN samples a square lattice, requiring
 *                      bilinear interpolation across 4 corners. The
 *                      square has a directional bias — features tend
 *                      to align with axes.
 *                    • SIMPLEX samples a triangular (simplex) lattice
 *                      — in 2-D, a single triangle suffices, with
 *                      barycentric-style contribution from each of its
 *                      3 vertices. Triangles tile space isotropically,
 *                      so no axis bias.
 *                    • Faster too: 3 vertex contributions in 2-D vs 4
 *                      for Perlin, scaling better in higher dimensions
 *                      (the gap widens as N grows).
 *
 *                  The 2-D algorithm:
 *                    1. SKEW input (x, y) so unit squares become unit
 *                       triangles: skew factor F2 = (√3 − 1)/2.
 *                    2. Find which simplex (one of two triangles inside
 *                       each skewed unit square) contains the point —
 *                       trivially "lower" if x0 > y0, "upper" otherwise.
 *                    3. UNSKEW each of the 3 simplex vertices back to
 *                       Cartesian and compute (Δx, Δy) from the query.
 *                    4. For each vertex: t = 0.5 − Δ² (the radial
 *                       falloff; goes negative outside the support
 *                       circle, in which case the contribution is 0);
 *                       if t > 0, contribution = t⁴ × (gradient · Δ).
 *                    5. Sum the 3 contributions, scale by 70 so the
 *                       output range is roughly [−1, 1].
 *
 *                  Thirty patterns, organised into six complexity
 *                  tiers, build on this primitive — see §6.
 *
 * Data-structure : Same 256-entry permutation table as Perlin, plus a
 *                  12-direction gradient table (vs Perlin's 8). Per-cell
 *                  glow + colour buffers as in the other field files.
 *                  No allocation post-init.
 *
 * Rendering      : ASCII only, density-graded ('.', '*', '#') in 4
 *                  theme palette colours. Each pattern produces glow ∈
 *                  [0, 1] which selects both the glyph (by threshold)
 *                  and the colour band (by quartile).
 *
 * Performance    : 1 simplex_sample() call per fBm octave per cell.
 *                  Tier-1 CLOUDS uses 1 simplex_fbm() (= 4 octaves =
 *                  4 simplex_sample calls/cell). Tier-4 FRACTAL is
 *                  the worst case (2-level domain warp = 4 standalone
 *                  samples + 1 fbm = 8 calls/cell). With ~11 K cells
 *                  at 60 Hz that's ~2.6 M to ~5 M simplex calls/sec
 *                  — roughly half the cost of equivalent Perlin
 *                  thanks to the 3-vertex vs 4-vertex difference.
 *                  Fits in well under 5 % CPU on modern hardware.
 *
 * References     : NOISE THEORY
 *                  • Perlin, K. (1985) — "An Image Synthesizer",
 *                    SIGGRAPH'85, pp.287-296. The original gradient
 *                    noise paper; also defines "turbulence" Σ|fBm|
 *                    (Tier 3) and the marble derivation (Tier 2 MARBLE).
 *                  • Perlin, K. (2001) — "Noise hardware" (SIGGRAPH
 *                    course notes). The simplex variant — what
 *                    simplex_sample() implements:
 *                    https://www.csee.umbc.edu/~olano/s2002c36/ch02.pdf
 *                  • Gustavson, S. (2005) — "Simplex Noise Demystified".
 *                    Implementation walkthrough; this file's simplex
 *                    code follows it verbatim:
 *                    https://weber.itn.liu.se/~stegu/simplexnoise/simplexnoise.pdf
 *
 *                  fBm AND DERIVATIVES (the 30 patterns)
 *                  • Ebert, Musgrave, Peachey, Perlin, Worley —
 *                    "Texturing & Modeling: A Procedural Approach"
 *                    (Morgan Kaufmann, 3rd ed. 2002, ISBN 1-55860-848-6).
 *                    The foundational text. Musgrave's chapter is the
 *                    source of Tier 1 RIDGED ("ridged multifractal");
 *                    Perlin's chapter covers fBm / turbulence / marble.
 *                  • Quilez, I. — "fbm". Augmented fBm techniques
 *                    (amplitude warping, derivative-domain variants):
 *                    https://iquilezles.org/articles/fbm/
 *                  • Quilez, I. — "Domain warping". Direct source for
 *                    Tier 4 — IQ's nested fbm(p + fbm(p + fbm(p)))
 *                    construction (Tier 4 FRACTAL is this verbatim):
 *                    https://iquilezles.org/articles/warp/
 *
 *                  ASCII RENDERING (noise → terminal)
 *                  • Bourke, P. — "Character representation of grey
 *                    scale images". Source of the canonical luminance
 *                    ramps; the GlyphRamp '.', '*', '#' here is a
 *                    coarse subset of Bourke's 10-char ramp:
 *                    http://paulbourke.net/dataformats/asciiart/
 *                  • Wikipedia — "ANSI escape code § 8-bit". The
 *                    xterm/ANSI 256-colour palette (16 system + 6³
 *                    cube + 24 greys) that themes[] in §1 and the
 *                    HUD pairs in §3 index into:
 *                    https://en.wikipedia.org/wiki/ANSI_escape_code#8-bit
 *                  • Padala, P. — "NCURSES Programming HOWTO". The
 *                    pragmatic reference for the §3/§8 rendering
 *                    APIs (init_pair, mvaddch, doupdate, A_BOLD):
 *                    https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *                  COMPARE IN PROJECT
 *                  • ./perin_noise_flow_showcase.c — the Perlin 1985
 *                    predecessor; same fBm shape, axis-biased lattice.
 *                  • ./domain_warped_noise_iq_style.c — focused
 *                    treatment of the Tier 4 (WARPED) technique.
 *                  • ../generational/voronoi_region_map.c — cellular
 *                    "discrete regions" instead of "smooth field".
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Simplex noise produces a smooth scalar field over arbitrary
 * coordinates — like Perlin noise, but built on a triangular lattice
 * instead of a square one. The result has no preferred axis; the
 * "wind" of the noise moves diagonally just as readily as
 * horizontally. For cloud-like patterns, that isotropy reads as
 * organic — clouds shouldn't have a grain.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a tiled honeycomb of equilateral triangles. Each triangle
 * vertex has a random gradient direction. To sample at a point P:
 *   1. Find which triangle P is in.
 *   2. From each of the 3 vertices, draw a vector to P, dot it with
 *      that vertex's gradient, and weight by a radial falloff.
 *   3. Sum those 3 contributions.
 * Because triangles are isotropic (no preferred axis) and only 3
 * vertices contribute (vs 4 for square Perlin), the function is
 * cheaper AND more visually neutral.
 *
 * Layered with fractional Brownian motion (sum noise at progressively
 * doubled frequency / halved amplitude), the result is a multi-scale
 * cloud field — large rolling shapes with small bumps on top.
 *
 * The 30 patterns differ in HOW they map noise to glow. They form six
 * complexity tiers — each tier is a different *technique class*, so
 * the visual jump between tiers is obvious; within a tier the 5
 * variants explore one knob:
 *   • Tier 1 RAW        : direct noise — fBm, |fbm|, 1−|fbm|, etc.
 *   • Tier 2 MAPPED     : sin/cos/pow of fBm — bands, marble, stripes.
 *   • Tier 3 TURBULENCE : Σ|octaveᵢ| stacks — storms, embers, veins.
 *   • Tier 4 WARPED     : domain warping (sample noise at coords
 *                          themselves perturbed by noise) — swirls,
 *                          dunes, flowing currents.
 *   • Tier 5 COMPOSITE  : two or more noise/spatial fields combined
 *                          — nebula, aurora, plasma, lightning, galaxy.
 *   • Tier 6 MASKED     : noise × spatial mask (radial / quantise /
 *                          vignette) — solar corona, mosaic, supernova.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Convert cell coord to noise space: (fx, fy) = (x, y) · NOISE_SCALE.
 *  2. Run the active pattern's noise function with current field_time
 *     added to fy (slow drift).
 *  3. Map noise → glow ∈ [0, 1] and colour band ∈ {0, 1, 2, 3}.
 *  4. Render: pick density glyph from glow; theme colour from band.
 *  5. The user can press 'r' to re-shuffle the permutation table for
 *     a fresh noise field (or 'n'/'p' to switch pattern). There is no
 *     automatic reset — the field drifts indefinitely.
 *
 * KEY FORMULAS
 * ────────────
 *  Simplex skew (2-D)            : F2 = (√3 − 1) / 2 ≈ 0.366
 *  Simplex unskew (2-D)          : G2 = (3 − √3) / 6 ≈ 0.211
 *  Falloff per vertex            : t = 0.5 − (Δx² + Δy²)
 *                                  contribution = t⁴ · (grad · (Δx, Δy))
 *                                  (zero if t ≤ 0)
 *  Output scale                  : multiply by 70 so result is roughly
 *                                  in [−1, 1]
 *  fBm                            : Σᵢ aᵢ · simplex(2ⁱ x), aᵢ = 1/2ⁱ
 *  Turbulence                    : Σᵢ aᵢ · |simplex(2ⁱ x)|
 *  Billow                        : |fbm(x)|
 *  Ridged                        : 1 − |fbm(x)|
 *  Domain warp (IQ, tier 4)      : fbm(x + fbm(x, y), y + fbm(x', y'))
 *  Quantise (tier 6 MOSAIC)      : floor(fbm · k) / (k − 1)
 *  Radial mask (tier 6)          : max(0, 1 − r² · falloff),
 *                                  where (nx, ny) ∈ [−1, 1]² and
 *                                  r² = nx² + ny²
 *  Per-pattern formulas inline next to each pattern_* function in §6;
 *  the dispatch table is `noise_patterns[]` (same section).
 *
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

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* PAIR_BAND_BASE..+3 = 4 palette colours */
    PAIR_FLASH        =   7,    /* vestigial — cross-file palette parity  */
};

#define GLOW_THRESHOLD      0.05f

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state + [N/M], drift)
 *   row 1           : pattern/theme/palette + scale/oct/map
 *   row HUD_TOP..N-2: noise-field map
 *   row N-1         : keyboard action hint
 */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Noise scale: ~0.04 noise-units per cell → ~8 noise-units across
 * the grid → 3-4 large feature periods visible. */
#define NOISE_SCALE         0.04f

/* Field drift in noise-coord units per second. */
#define FIELD_DRIFT         0.10f

/* Drift multiplier — cranked by +/-. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* fBm stack: 4 octaves with halving amplitude / doubling frequency. */
#define FBM_OCTAVES         4

/* WISPS pattern: x-scale multiplier (smaller = longer horizontal
 * features). Y-scale stays at 1. */
#define WISPS_X_SCALE       0.40f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Palette quantisation — turn a continuous glow ∈ [0, 1] into a
 * discrete palette-band index ∈ {0..3}.
 *
 *   N_PALETTE_BANDS      — how many tiers the theme defines (must
 *                           match Theme.band[] array length).
 *   PALETTE_BAND_MASK    — cheap `& MASK` modulo (works because
 *                           N_PALETTE_BANDS is a power of two).
 *   GLOW_TO_BAND_GAIN    — glow * GAIN → 0..3 via integer truncation.
 *                           Set to N_PALETTE_BANDS − ε so that
 *                           glow == 1.0 still floors to band 3 instead
 *                           of overflowing to a non-existent band 4.
 */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3       /* N_PALETTE_BANDS - 1, MASK form */
#define GLOW_TO_BAND_GAIN   3.999f  /* almost-4 to keep glow=1.0 → band 3 */

/*
 * Simplex skew (F2) and unskew (G2) constants. Used in §5 to map
 * between Cartesian and triangular-lattice coordinates.
 *
 *   F2 = (√3 − 1) / 2  ≈ 0.366  — multiplied by (x + y) to shear the
 *                                  square grid into a simplex grid.
 *                                  Used inside simplex_locate.
 *   G2 = (3 − √3) / 6  ≈ 0.211  — the inverse shear. Used once in
 *                                  simplex_locate (to unskew the
 *                                  cell origin) and once per corner
 *                                  inside simplex_evaluate_corner
 *                                  (to unskew each corner's offset
 *                                  back to Cartesian space).
 *
 * Full precision matters: error in these constants propagates through
 * the floor() inside simplex_locate and can put a query point in the
 * wrong simplex cell, producing visible seam artefacts. Values from
 * Stefan Gustavson's reference implementation.
 *
 * SIMPLEX_OUTPUT_GAIN is the empirical scalar (Gustavson) that maps
 * the corner-sum into approximately [-1, 1].
 */
#define SIMPLEX_F2            0.36602540378443864f /* skew  (sqrt(3) - 1) / 2 */
#define SIMPLEX_G2            0.21132486540518713f /* unskew (3 - sqrt(3)) / 6 */
#define SIMPLEX_OUTPUT_GAIN   70.0f                /* corner-sum scale → ~[-1,1] */

/*
 * Simplex algorithm sizing constants.
 *
 *   N_SIMPLEX_CORNERS   — every 2-D simplex (triangle) has 3 vertices;
 *                          we sum one contribution per vertex.
 *   N_GRADIENT_DIRS     — grad2[] holds 12 directions (Perlin's
 *                          balanced 12-set: 4 diagonals + 4 axials
 *                          duplicated to round-out the lookup).
 *                          Used as the modulus when hashing into grad2.
 *   PERM_TABLE_SIZE     — Fisher-Yates table holds 256 unique values;
 *                          duplicated to 512 entries for wrap-free index.
 *   PERM_TABLE_INDEX_MASK — `i & MASK` is the cheap form of `i % SIZE`
 *                          (valid because SIZE is a power of two).
 */
#define N_SIMPLEX_CORNERS     3
#define N_GRADIENT_DIRS       12
#define PERM_TABLE_SIZE       256
#define PERM_TABLE_INDEX_MASK 255   /* SIZE - 1; valid because SIZE = 2^8 */

/*
 * Pattern — thirty mappings of simplex noise, grouped into six
 * complexity tiers. Cycle with n/p. The enum order MUST match the
 * `noise_patterns[]` dispatch table in §6 (compiler enforces this
 * via the fixed-size [N_PATTERNS] array initialiser — a missing or
 * mis-ordered row is a compile error).
 *
 *   Tier 1 RAW        : CLOUDS, BILLOW, RIDGED, WISPS, CRESTS
 *   Tier 2 MAPPED     : CONTOURS, MARBLE, ZEBRA, RIPPLES, THRESHOLD
 *   Tier 3 TURBULENCE : TURBULENCE, STORM, INFERNO, VEINS, EMBERS
 *   Tier 4 WARPED     : WARP, WHIRL, DUNES, CURRENTS, FRACTAL
 *   Tier 5 COMPOSITE  : NEBULA, AURORA, PLASMA, LIGHTNING, GALAXY
 *   Tier 6 MASKED     : SOLAR, MOSAIC, VIGNETTE, COSMOS, SUPERNOVA
 */
typedef enum {
    /* Tier 1 — RAW: direct noise */
    PATTERN_CLOUDS = 0,
    PATTERN_BILLOW,
    PATTERN_RIDGED,
    PATTERN_WISPS,
    PATTERN_CRESTS,
    /* Tier 2 — MAPPED: sin/cos/pow transforms */
    PATTERN_CONTOURS,
    PATTERN_MARBLE,
    PATTERN_ZEBRA,
    PATTERN_RIPPLES,
    PATTERN_THRESHOLD,
    /* Tier 3 — TURBULENCE: Σ|octave| stacks */
    PATTERN_TURBULENCE,
    PATTERN_STORM,
    PATTERN_INFERNO,
    PATTERN_VEINS,
    PATTERN_EMBERS,
    /* Tier 4 — WARPED: domain warping */
    PATTERN_WARP,
    PATTERN_WHIRL,
    PATTERN_DUNES,
    PATTERN_CURRENTS,
    PATTERN_FRACTAL,
    /* Tier 5 — COMPOSITE: multi-field combine */
    PATTERN_NEBULA,
    PATTERN_AURORA,
    PATTERN_PLASMA,
    PATTERN_LIGHTNING,
    PATTERN_GALAXY,
    /* Tier 6 — MASKED: noise × spatial mask */
    PATTERN_SOLAR,
    PATTERN_MOSAIC,
    PATTERN_VIGNETTE,
    PATTERN_COSMOS,
    PATTERN_SUPERNOVA,
    N_PATTERNS,
} Pattern;

/* pattern_name() / pattern_tier() defined in §6 alongside dispatch table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Frame-loop timing limits.
 *
 *   MAX_FRAME_DT_NS — cap the per-frame dt at 100 ms before feeding
 *                     it to the fixed-timestep accumulator. Without
 *                     this cap, a long stall (debugger, swap-storm,
 *                     SIGSTOP) would queue thousands of sim ticks
 *                     all firing at once when the process resumes
 *                     — the classic "spiral of death" failure mode
 *                     described in Glenn Fiedler's "Fix Your
 *                     Timestep!" article.
 *   RENDER_FPS_TARGET — render-throttle target. The sleep before
 *                     I/O paces the render loop at this rate even
 *                     when sim_fps is set higher.
 */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — a named 4-tier colour palette + accent for the simplex
 * cloud renderer.
 *
 * INTENT
 *   Decouple the "what colours" choice from the "how to render"
 *   pipeline. The same GlowField + GlyphRamp drives 10 different
 *   visual moods just by swapping which 4 colour pairs ncurses is
 *   bound to (see theme_apply() in §3).
 *
 * CONTEXT
 *   Indexed by PaletteState.current (§7); cycled via t / T. The
 *   four band entries map 1:1 onto GlowField.band quartile
 *   {0,1,2,3} — band 0 is the dimmest tier, band 3 the brightest.
 *   The renderer reads `themes[palette.current].band[idx]` once per
 *   theme change and rebuilds the ncurses colour pairs.
 *
 * MEMBER LOGIC
 *   name   : short uppercase label shown in HUD row 1. ≤7 chars so
 *            the "theme:%-8s" format string never overflows.
 *   band[] : xterm-256 colour-cube indices (see ANSI escape code
 *            § 8-bit). Per CLAUDE.md "Theme Palette Brightness",
 *            every entry MUST sit in the bright half of the cube
 *            (≥ 24 for the 6³ cube, ≥ 244 for the greyscale ramp).
 *            Bottom-of-palette colours go invisible against the
 *            default-black bg when rendered with A_DIM.
 *   flash  : 256-colour accent reserved for rare-event highlights.
 *            Unused in this file; kept for cross-file palette parity
 *            so a future event-flash feature (e.g. a "starburst on
 *            high-glow cell") drops in without theme rework.
 *
 * Ref: ANSI escape code § 8-bit (Wikipedia link in References block).
 */
typedef struct {
    const char *name;        /* HUD label, ≤7 chars                         */
    short       band[4];     /* xterm-256 idx, one per GlowField.band tier  */
    short       flash;       /* xterm-256 accent (reserved, unused here)    */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 234,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  52,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
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
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  noise — SimplexNoise primitive + fBm stacks                        */
/* ===================================================================== */

/*
 * SimplexNoise — the noise primitive's runtime state.
 *
 * INTENT
 *   Encapsulate the ONE piece of mutable state simplex noise needs
 *   — its randomised permutation table — so the algorithm can be
 *   passed as `const SimplexNoise *` and its purity is type-checked.
 *   Everything else simplex needs (the 12 gradient directions,
 *   the skew constants F2/G2) is a compile-time mathematical fact
 *   and lives as file-scope const.
 *
 * CONTEXT
 *   Lives on Scene (§7). Pattern functions in §6 receive a
 *   const-pointer so the dataflow at every call site reads
 *   "sample noise, don't mutate it". Reshuffling only happens via
 *   simplex_reshuffle() on user reset (r key).
 *
 * MEMBER LOGIC — perm[512]
 *   • Holds a Fisher-Yates permutation of 0..255 in [0..255], with
 *     the same values DUPLICATED into [256..511]. The double-length
 *     form lets us index `perm[hash_i + perm[hash_j]]` directly
 *     (the outer index can reach up to 255 + 255 = 510) without
 *     wrap-around bookkeeping. The mask in simplex_evaluate_corner
 *     keeps hash_i / hash_j in [0, 255] on the inner reads, so the
 *     doubling is only strictly needed for the OUTER read — but it
 *     simplifies the call site by making both reads bounds-safe.
 *     simplex_sample makes 6 perm reads per call (3 corner hashes
 *     × 2 reads each), so any saved bookkeeping there matters.
 *   • uint8_t: each entry holds 0..255 exactly; no headroom wasted.
 *     512 bytes total, fits in 8 cache lines.
 *   • Reshuffled at every reset so different runs (and different
 *     r-presses) produce visibly different noise fields.
 *
 * Refs: Perlin 2001 (the simplex paper), Gustavson §"Permutation
 *       array" (the doubled-perm trick comes from his canonical
 *       reference implementation).
 */
typedef struct {
    uint8_t perm[512];  /* Fisher-Yates'd 0..255 doubled into [256..511]    */
} SimplexNoise;

/*
 * grad2[][2] — 12 gradient directions for 2-D simplex noise. The
 * standard set: edges of a regular icosahedron projected to 2-D,
 * with two of the axial directions duplicated to bring the count to
 * 12 (a power-of-12-friendly lookup via `% 12`). The 12-set is
 * balanced (sum-to-zero) and matches Perlin/Gustavson references.
 *
 * Stays file-scope const — it's mathematical, not state. Shared by
 * every SimplexNoise instance (we only ever have one, but conceptually
 * the const-ness is what matters).
 */
static const int8_t grad2[N_GRADIENT_DIRS][2] = {
    {  1,  1 }, { -1,  1 }, {  1, -1 }, { -1, -1 },
    {  1,  0 }, { -1,  0 }, {  1,  0 }, { -1,  0 },
    {  0,  1 }, {  0, -1 }, {  0,  1 }, {  0, -1 },
};

/*
 * simplex_reshuffle — Fisher-Yates the 0..N-1 sequence using rand,
 * then duplicate it into perm[N..2N-1] so callers can index with
 * `perm[i + offset]` without wrap-around bookkeeping. Called at each
 * reset (r key) so different runs produce different noise fields.
 */
static void simplex_reshuffle(SimplexNoise *sn)
{
    /* STEP 1 — identity sequence 0..N-1 */
    uint8_t base[PERM_TABLE_SIZE];
    for (int i = 0; i < PERM_TABLE_SIZE; i++) base[i] = (uint8_t)i;

    /* STEP 2 — Fisher-Yates: swap each element with a random earlier slot */
    for (int i = PERM_TABLE_SIZE - 1; i > 0; i--) {
        int     j         = rand() % (i + 1);
        uint8_t swap_tmp  = base[i];
        base[i]           = base[j];
        base[j]           = swap_tmp;
    }

    /* STEP 3 — duplicate into the second half so perm[i + k] with
     * k ∈ {0, 1, i1, j1} never crosses the table end without wrap */
    for (int i = 0; i < PERM_TABLE_SIZE; i++) {
        sn->perm[i]                    = base[i];
        sn->perm[i + PERM_TABLE_SIZE]  = base[i];
    }
}

/* ───── simplex_sample — types and helpers ───────────────────────────── *
 *
 * The 2-D simplex algorithm decomposes naturally into 4 named stages.
 * Each stage gets a dedicated helper below; simplex_sample itself is
 * then 4 lines of real work — a pseudocode driver.
 *
 *   STAGE 1  LOCATE   — skew + unskew to find which simplex cell
 *                        contains the query and the query's offset
 *                        within it.                  → simplex_locate
 *
 *   STAGE 2  PICK     — choose which of the cell's 2 triangles holds
 *                        the query and enumerate its 3 corners as
 *                        data.                → simplex_pick_corners
 *
 *   STAGE 3  EVALUATE — per corner: displacement → gradient hash →
 *                        radial-falloff × dot product.
 *                                          → simplex_evaluate_corner
 *
 *   STAGE 4  SUM + SCALE — accumulate the 3 contributions and rescale
 *                           to roughly [-1, 1].
 *                                            → simplex_sample (body)
 *
 * Two small inline helpers (simplex_hash_gradient_at and
 * simplex_corner_contribution) break out the SMALLEST reusable
 * pieces: a perm-table lookup, and the kernel that turns
 * (gradient, displacement) into a scalar contribution.
 */

/*
 * SimplexCornerOffset — one of the 3 vertices of a 2-D simplex unit
 * cell, expressed as an integer (di, dj) offset in the SKEWED lattice
 * from the cell's first corner.
 *
 * INTENT
 *   Turn "the 3 corners of a triangle" into data so simplex_sample's
 *   inner work can be a single for-loop over an array. Without this
 *   type the 3-fold symmetry has to be expressed as 3 unrolled blocks
 *   of identical math — which obscures the algorithm.
 *
 * VALUES
 *   Corner 0 ≡ (0, 0)                   — cell origin (always).
 *   Corner 1 ≡ (1, 0)   LOWER triangle  (query right of y=x diagonal),
 *             (0, 1)   UPPER triangle  (query above the diagonal).
 *   Corner 2 ≡ (1, 1)                   — opposite vertex (always).
 *
 * MEMORY / COST
 *   2 ints × 3 corners = 24 bytes per call, lives in registers; the
 *   3-iteration loop is inlined/unrolled at -O2 so this abstraction
 *   costs literally zero at runtime versus the hand-unrolled form.
 *
 * MEMBER LOGIC
 *   di : column offset in skewed lattice space, ∈ {0, 1}. Added to
 *        cell_i for both perm-table lookups and Cartesian-displacement
 *        recovery (see simplex_evaluate_corner).
 *   dj : row offset, same role on the other axis, ∈ {0, 1}.
 *
 * Ref: Gustavson §"Setting up the simplex grid" — the diagonal split
 *      and corner numbering used here come from that walkthrough.
 */
typedef struct {
    int di;   /* skewed-x offset within the cell, ∈ {0, 1} */
    int dj;   /* skewed-y offset within the cell, ∈ {0, 1} */
} SimplexCornerOffset;

/*
 * SimplexQuery — what simplex_locate computes: enough information for
 * every later stage to derive a specific corner's displacement and
 * to seed the perm-table hash.
 *
 * INTENT
 *   Bundle STAGE-1's result (skewed cell index + Cartesian offset
 *   from cell origin) so simplex_pick_corners and
 *   simplex_evaluate_corner each take ONE `const SimplexQuery *`
 *   parameter instead of 4 loose floats / ints.
 *
 * CONTEXT
 *   Produced by simplex_locate(x, y). Consumed by:
 *     • simplex_pick_corners — reads corner0_dx/dy to decide which
 *       of the cell's two triangles contains the query.
 *     • simplex_evaluate_corner — reads everything: cell_i/j seed
 *       the perm hash, corner0_dx/dy is the base of every corner's
 *       Cartesian displacement.
 *
 * MEMBER LOGIC
 *   cell_i,         — Integer cell index in the SKEWED lattice
 *   cell_j            (after STAGE 1's skew + floor). Identifies
 *                     which unit triangle PAIR contains the query.
 *                     Used (unmasked) as input to the perm-table
 *                     hash; the mask is applied per-corner inside
 *                     simplex_evaluate_corner.
 *
 *   corner0_dx,     — Query's Cartesian displacement from the cell's
 *   corner0_dy        FIRST corner (after STAGE 1's unskew). Two
 *                     roles:
 *                       (a) base for every corner's displacement:
 *                            corner_k_disp = corner0_d − (di, dj)
 *                                            + (di + dj)·G2·(1, 1).
 *                       (b) triangle-pick test: query is in the
 *                            lower triangle iff
 *                            corner0_dx > corner0_dy (right of the
 *                            cell's y = x diagonal).
 */
typedef struct {
    int   cell_i, cell_j;          /* integer cell index, skewed lattice  */
    float corner0_dx, corner0_dy;  /* query's Cartesian offset from origin*/
} SimplexQuery;

/*
 * simplex_hash_gradient_at — turn a pair of integer lattice coords
 * (already masked into the perm table's range) into a gradient-
 * direction index ∈ [0, N_GRADIENT_DIRS).
 *
 * The two-level perm[corner_hash_i + perm[corner_hash_j]] is what
 * gives Perlin-family noise its pseudo-random-but-repeatable
 * property: the same (i, j) yields the same gradient every time,
 * but the mapping looks random to a sampler walking nearby cells.
 *
 * The perm table is doubled-length (2 × PERM_TABLE_SIZE entries) so
 * the sum `corner_hash_i + perm[corner_hash_j]` (up to 510) lands
 * on a valid slot without wrap-around bookkeeping.
 */
static inline int simplex_hash_gradient_at(const SimplexNoise *sn,
                                           int corner_hash_i,
                                           int corner_hash_j)
{
    int row_offset = sn->perm[corner_hash_j];
    return sn->perm[corner_hash_i + row_offset] % N_GRADIENT_DIRS;
}

/*
 * simplex_corner_contribution — the per-corner kernel. Computes:
 *
 *     (0.5 − r²)⁴ · (gradient · displacement)
 *
 *   where r² = dx² + dy² and the gradient is one of N_GRADIENT_DIRS
 *   fixed directions in grad2[]. Returns 0 outside the corner's
 *   support disk (where r² ≥ 0.5).
 *
 * The (0.5 − r²)⁴ "radial falloff" smoothly tapers each corner's
 * influence to zero at the disk boundary; the power-of-four exponent
 * makes the noise function AND its first derivative continuous
 * across simplex boundaries (C¹ continuity).
 *
 * Ref: Gustavson §"The contribution from each corner".
 */
static inline float simplex_corner_contribution(int gradient_idx,
                                                float dx, float dy)
{
    float radial_falloff = 0.5f - dx * dx - dy * dy;
    if (radial_falloff <= 0.0f) return 0.0f;
    radial_falloff *= radial_falloff;       /*  (0.5 − r²)²  */
    radial_falloff *= radial_falloff;       /*  (0.5 − r²)⁴  */
    float gradient_dot_displacement =
        (float)grad2[gradient_idx][0] * dx +
        (float)grad2[gradient_idx][1] * dy;
    return radial_falloff * gradient_dot_displacement;
}

/*
 * simplex_locate — STAGE 1: find the simplex cell containing the
 * query (xin, yin) and the query's Cartesian offset within it.
 *
 *   SKEW   — shear input space by F2 along (1, 1) so each unit
 *            Cartesian square becomes a unit-edge triangle pair.
 *            FLOOR identifies the integer simplex cell.
 *   UNSKEW — shear back by G2 to recover the cell's first corner
 *            in Cartesian space. The query's displacement from that
 *            corner is what every later stage needs.
 *
 *     F2 = (√3 − 1) / 2  ≈ 0.366
 *     G2 = (3 − √3) / 6  ≈ 0.211
 *
 * Full precision in SIMPLEX_F2 / SIMPLEX_G2 matters (see §1) to
 * avoid edge-cell artefacts where a query straddles the floor()
 * boundary.
 *
 * Ref: Gustavson §"Skewing".
 */
static inline SimplexQuery simplex_locate(float xin, float yin)
{
    /* SKEW + floor → integer simplex cell. */
    float skew_offset = (xin + yin) * SIMPLEX_F2;
    int   cell_i      = (int)floorf(xin + skew_offset);
    int   cell_j      = (int)floorf(yin + skew_offset);

    /* UNSKEW the cell origin → query's offset in Cartesian. */
    float unskew_offset = (float)(cell_i + cell_j) * SIMPLEX_G2;
    SimplexQuery q;
    q.cell_i     = cell_i;
    q.cell_j     = cell_j;
    q.corner0_dx = xin - ((float)cell_i - unskew_offset);
    q.corner0_dy = yin - ((float)cell_j - unskew_offset);
    return q;
}

/*
 * simplex_pick_corners — STAGE 2: choose which of the cell's two
 * triangles holds the query, then enumerate its 3 corners as data.
 *
 * The y = x diagonal of the skewed cell splits each unit Cartesian
 * square into TWO simplices:
 *
 *   • LOWER (corner 1 = (1, 0)) — query is RIGHT of the diagonal,
 *                                  i.e. corner0_dx > corner0_dy.
 *   • UPPER (corner 1 = (0, 1)) — query is ABOVE the diagonal,
 *                                  i.e. corner0_dx ≤ corner0_dy.
 *
 * Corners 0 and 2 are always (0, 0) and (1, 1); only the middle
 * one (corner 1) depends on the triangle choice.
 *
 * Writes the 3 corners into out[N_SIMPLEX_CORNERS]; the caller
 * iterates over them in simplex_sample's loop.
 */
static inline void simplex_pick_corners(float corner0_dx, float corner0_dy,
                                        SimplexCornerOffset out[N_SIMPLEX_CORNERS])
{
    bool is_lower_triangle = (corner0_dx > corner0_dy);
    out[0] = (SimplexCornerOffset){ 0, 0 };           /* cell origin     */
    out[1] = is_lower_triangle
           ? (SimplexCornerOffset){ 1, 0 }            /* LOWER  △        */
           : (SimplexCornerOffset){ 0, 1 };           /* UPPER  △        */
    out[2] = (SimplexCornerOffset){ 1, 1 };           /* opposite vertex */
}

/*
 * simplex_evaluate_corner — STAGE 3: one corner's contribution to
 * the noise sum. Combines three sub-operations under a single name:
 *
 *   (a) CARTESIAN DISPLACEMENT from the query to this corner.
 *       The corner sits at skewed offset (di, dj) from the cell
 *       origin; in Cartesian space that maps to
 *
 *           (dx, dy) = (corner0_dx, corner0_dy)
 *                      − (di, dj)
 *                      + (di + dj) · G2 · (1, 1)
 *
 *       The (di + dj)·G2 term UNSKEWS the corner's offset back into
 *       the Cartesian frame where r² = dx² + dy² makes geometric
 *       sense for the contribution kernel.
 *
 *   (b) GRADIENT HASH via the perm table, seeded by (cell + (di, dj)).
 *       Both axes masked into the table's range with
 *       PERM_TABLE_INDEX_MASK (cheap `& 255`).
 *
 *   (c) CONTRIBUTION KERNEL — (0.5 − r²)⁴ × (grad · disp) — via
 *       simplex_corner_contribution above.
 *
 * Inlined at the call site at -O2 so per-corner cost is identical
 * to the original hand-unrolled body, but the call NAMES what the
 * arithmetic adds up to.
 */
static inline float simplex_evaluate_corner(const SimplexNoise *sn,
                                            const SimplexQuery *q,
                                            SimplexCornerOffset corner)
{
    int   di            = corner.di;
    int   dj            = corner.dj;
    float corner_unskew = (float)(di + dj) * SIMPLEX_G2;
    float dx            = q->corner0_dx - (float)di + corner_unskew;
    float dy            = q->corner0_dy - (float)dj + corner_unskew;
    int   hash_i        = (q->cell_i + di) & PERM_TABLE_INDEX_MASK;
    int   hash_j        = (q->cell_j + dj) & PERM_TABLE_INDEX_MASK;
    int   gradient_idx  = simplex_hash_gradient_at(sn, hash_i, hash_j);
    return simplex_corner_contribution(gradient_idx, dx, dy);
}

/*
 * simplex_sample — 2-D simplex noise. Returns roughly [-1, 1].
 *
 * Pseudocode:
 *
 *   q       = LOCATE(x, y)                       // skew + unskew
 *   corners = PICK_CORNERS(q.corner0_offset)     // 3-vertex triangle
 *   sum     = Σ  EVALUATE_CORNER(noise, q, c)    // 3 corners
 *   return    SCALE · sum
 *
 * Follows Stefan Gustavson, "Simplex Noise Demystified" — the
 * canonical reference implementation. Pure function of (sn, x, y).
 */
static float simplex_sample(const SimplexNoise *sn, float xin, float yin)
{
    /* STAGE 1 — LOCATE cell + query's offset within it. */
    SimplexQuery q = simplex_locate(xin, yin);

    /* STAGE 2 — PICK simplex (lower/upper triangle) + ENUMERATE
     *           its 3 corners as data. */
    SimplexCornerOffset corners[N_SIMPLEX_CORNERS];
    simplex_pick_corners(q.corner0_dx, q.corner0_dy, corners);

    /* STAGE 3 — SUM the 3 corners' contributions. Each call does:
     *           displacement → gradient hash → falloff · dot product. */
    float corner_sum = 0.0f;
    for (int k = 0; k < N_SIMPLEX_CORNERS; k++)
        corner_sum += simplex_evaluate_corner(sn, &q, corners[k]);

    /* STAGE 4 — SCALE the sum into the conventional [-1, 1] range.
     * SIMPLEX_OUTPUT_GAIN = 70.0f is Gustavson's empirical fit: with
     * random gradients and the (0.5 − r²)⁴ kernel, the raw sum has
     * standard deviation ~0.014, so ~70× maps typical values into
     * the unit band. */
    return SIMPLEX_OUTPUT_GAIN * corner_sum;
}

/*
 * simplex_fbm — fractional Brownian motion stack of simplex_sample.
 * Sums FBM_OCTAVES bands of doubling frequency / halving amplitude.
 * Output is normalised by the running amplitude sum so it stays in
 * roughly [−1, 1].
 *
 *   fBm(x) = Σᵢ aᵢ · simplex(2ⁱx)        where aᵢ = 1/2ⁱ
 *
 * This is the "multi-scale terrain" or "natural turbulence" stack
 * Perlin first published in his 1985 paper.
 */
static float simplex_fbm(const SimplexNoise *sn, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * simplex_sample(sn, x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/*
 * simplex_fbm_abs — fBm of |simplex_sample|. Always ≥ 0 because of
 * the absolute value. This is Perlin's "turbulence" function from
 * his marble-rendering paper — the |·| introduces folds at every
 * zero crossing of the underlying noise, producing sharper-edged
 * textures than plain fBm.
 *
 *   turbulence(x) = Σᵢ aᵢ · |simplex(2ⁱx)|
 */
static float simplex_fbm_abs(const SimplexNoise *sn, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * fabsf(simplex_sample(sn, x * freq, y * freq));
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return total / max_amp;
}

/* ===================================================================== */
/* §6  patterns — 30 noise mappings in 6 tiers + dispatch table           */
/* ===================================================================== */

/*
 * Signature: all pattern functions take (sn, fx, fy, t, nx, ny) where
 *   sn      = const SimplexNoise* — the noise primitive's perm table.
 *             Threaded through every pattern so the dataflow is
 *             explicit: a pattern *samples* noise, never mutates it.
 *   fx, fy  = noise-space coords (cell × NOISE_SCALE)
 *   t       = pattern.field_time (drift accumulator)
 *   nx, ny  = screen-space coords normalised to [-1, 1] with (0, 0)
 *             at grid center — only used by radial / vertical-mask
 *             patterns (galaxy, solar, aurora, vignette, supernova,
 *             inferno). Other patterns mark (nx, ny) unused with
 *             (void) casts.
 *
 * Return value is glow ∈ [0, 1]; clamped by the caller anyway, but
 * keep it well-behaved here for HUD legibility.
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

static inline float smoothstepf(float e0, float e1, float x)
{
    float t = clampf((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* ---------- Tier 1 — RAW: direct noise -------------------------------- */

/*
 * CLOUDS — plain fBm mapped to [0, 1]. Soft cumulus look; the baseline
 * against which every other pattern varies.
 */
static float pattern_clouds(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    return simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
}

/*
 * BILLOW — |fbm|. Negatives flip up so peaks form at every zero
 * crossing — bumpy puffs with dark valleys between them.
 */
static float pattern_billow(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    return clampf(fabsf(simplex_fbm(sn, x, y + t)), 0.0f, 1.0f);
}

/*
 * RIDGED — 1 − |fbm|. Inverse of BILLOW: peaks at noise zeros, valleys
 * at extremes. Looks like sharp cirrus ridges — Musgrave's classic
 * "ridged multifractal", here over simplex instead of Perlin.
 */
static float pattern_ridged(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    return clampf(1.0f - fabsf(simplex_fbm(sn, x, y + t)), 0.0f, 1.0f);
}

/*
 * WISPS — anisotropic stretch. Sample at a smaller x-scale so
 * horizontal features last longer in screen space — cloud streaks
 * instead of round puffs. Drift accelerated ×1.5 so they appear to
 * fly past.
 */
static float pattern_wisps(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    return simplex_fbm(sn, x * WISPS_X_SCALE, y + t * 1.5f) * 0.5f + 0.5f;
}

/*
 * CRESTS — (1 − |fbm|)⁴. RIDGED raised to a high power makes the
 * peaks razor-sharp and the valleys almost-black, like windblown
 * snow crests under low sun.
 */
static float pattern_crests(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float r = 1.0f - fabsf(simplex_fbm(sn, x, y + t));
    r *= r; r *= r;                          /* pow 4 */
    return clampf(r, 0.0f, 1.0f);
}

/* ---------- Tier 2 — MAPPED: sin/cos/pow transforms ------------------- */

/*
 * CONTOURS — fractional part of fbm · 8. Treats the noise as a height
 * map and "rolls over" every 1/8th of an isoline, producing nested
 * contour bands that follow the underlying smooth field.
 */
static float pattern_contours(const SimplexNoise *sn, float x, float y, float t,
                              float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t) * 8.0f;
    return v - floorf(v);
}

/*
 * MARBLE — sin(x · k + fbm · K). Pure sin gives perfect stripes; the
 * fbm term wobbles the stripe phase so they bend like marble veining.
 */
static float pattern_marble(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t);
    return sinf(x * 3.0f + v * 6.0f) * 0.5f + 0.5f;
}

/*
 * ZEBRA — sign(sin(fbm · 12)) mapped to high/low. Hard binary stripes
 * that twist along the noise field's flow direction. Either bright
 * or dark — nothing in between.
 */
static float pattern_zebra(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float v = sinf(simplex_fbm(sn, x, y + t) * 12.0f);
    return v > 0.0f ? 0.90f : 0.10f;
}

/*
 * RIPPLES — sin(fbm · 16) · ½ + ½. Many tightly-packed concentric
 * sinusoidal bands; reads like raindrops on a still pond viewed from
 * above. CONTOURS but smooth instead of sawtooth.
 */
static float pattern_ripples(const SimplexNoise *sn, float x, float y, float t,
                             float nx, float ny)
{
    (void)nx; (void)ny;
    return sinf(simplex_fbm(sn, x, y + t) * 16.0f) * 0.5f + 0.5f;
}

/*
 * THRESHOLD — smoothstep at ~0.5 of (fbm + 1)/2. Binary cloud cover
 * with anti-aliased edges. The thin "smoothstep range" produces sharp
 * island shapes that morph as the field drifts.
 */
static float pattern_threshold(const SimplexNoise *sn, float x, float y, float t,
                               float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    return smoothstepf(0.45f, 0.55f, v);
}

/* ---------- Tier 3 — TURBULENCE: Σ|octave| stacks --------------------- */

/*
 * TURBULENCE — Perlin's classic Σ aᵢ·|simplex(2ⁱx)|. Always ≥ 0; the
 * |·| introduces folds at every zero crossing producing the sharp-
 * edged "storm cloud" appearance.
 */
static float pattern_turbulence(const SimplexNoise *sn, float x, float y, float t,
                                float nx, float ny)
{
    (void)nx; (void)ny;
    return clampf(simplex_fbm_abs(sn, x, y + t), 0.0f, 1.0f);
}

/*
 * STORM — turbulence², gain 1.5. The squaring pushes mid-values down
 * and highlights peaks — produces darker, more violently-contrasted
 * storm cells.
 */
static float pattern_storm(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm_abs(sn, x, y + t);
    return clampf(v * v * 1.5f, 0.0f, 1.0f);
}

/*
 * INFERNO — turbulence × vertical mask (brighter near bottom). Uses
 * ny ∈ [-1, 1] from the caller. Drift accelerated ×2 so flames flick
 * upward visibly. Reads as rising heat / fire.
 */
static float pattern_inferno(const SimplexNoise *sn, float x, float y, float t,
                             float nx, float ny)
{
    (void)nx;
    float turb = simplex_fbm_abs(sn, x, y + t * 2.0f);
    float rise = (1.0f - ny) * 0.5f + 0.25f; /* bright at bottom (ny→1) */
    return clampf(turb * rise * 1.3f, 0.0f, 1.0f);
}

/*
 * VEINS — 1 − √turbulence. The square root crushes high turbulence
 * values toward 1; subtracting from 1 turns them into thin dark
 * lines on a bright background — like leaf veins or river deltas.
 */
static float pattern_veins(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm_abs(sn, x, y + t);
    return clampf(1.0f - sqrtf(v + 0.05f), 0.0f, 1.0f);
}

/*
 * EMBERS — turbulence with a soft threshold at 0.55. Below the
 * threshold the value is dimmed (×0.4); above, it's amplified (×3).
 * Sparse bright "embers" glow over a dark field.
 */
static float pattern_embers(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm_abs(sn, x, y + t * 1.5f);
    if (v > 0.55f) return clampf((v - 0.55f) * 3.0f + 0.5f, 0.0f, 1.0f);
    return v * 0.4f;
}

/* ---------- Tier 4 — WARPED: domain warping --------------------------- */

/*
 * WARP — IQ-style domain warp: sample fbm at coords themselves
 * perturbed by single-octave noise. The "currents-pushing-clouds-
 * around" look; the field gains apparent advection without any time
 * integration.
 */
static float pattern_warp(const SimplexNoise *sn, float x, float y, float t,
                          float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x, y + t);
    float qy = simplex_sample(sn, x + 5.2f, y + 1.3f + t);
    return simplex_fbm(sn, x + qx, y + qy) * 0.5f + 0.5f;
}

/*
 * WHIRL — rotate (x, y) by a noise-controlled angle, then sample.
 * Where the noise rotation field has zeros/extrema, swirls and
 * vortices appear. Uses single-octave noise for the angle so the
 * vortices are large and readable.
 */
static float pattern_whirl(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float a  = simplex_sample(sn, x * 0.5f, y * 0.5f + t) * 2.0f * (float)M_PI;
    float ca = cosf(a), sa = sinf(a);
    float wx = x * ca - y * sa;
    float wy = x * sa + y * ca;
    return simplex_fbm(sn, wx, wy + t) * 0.5f + 0.5f;
}

/*
 * DUNES — strong x-axis warp only. Sample fbm at x perturbed by 2·noise,
 * y at half-frequency. Produces long crescent ridges that wrap around
 * each other like sand dunes seen from a low angle.
 */
static float pattern_dunes(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x, y) * 2.0f;
    return simplex_fbm(sn, x + qx + t, y * 0.5f) * 0.5f + 0.5f;
}

/*
 * CURRENTS — subtle two-axis warp (gain 0.4) so the noise field
 * gently flows rather than violently rearranges. Reads as a river
 * delta or slow ocean currents.
 */
static float pattern_currents(const SimplexNoise *sn, float x, float y, float t,
                              float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x * 2.0f, y * 2.0f + t);
    float qy = simplex_sample(sn, x * 2.0f + 5.0f, y * 2.0f + 5.0f + t);
    return simplex_fbm(sn, x + 0.4f * qx, y + 0.4f * qy + t) * 0.5f + 0.5f;
}

/*
 * FRACTAL — IQ's 2-level domain warp: warp the warp. Produces deeply
 * nested swirly detail. Costs more simplex_sample calls than other
 * patterns (4 single-octave for warps + 1 fbm = 8 simplex calls/cell),
 * still comfortably under 1 ms for an 11K-cell grid.
 */
static float pattern_fractal(const SimplexNoise *sn, float x, float y, float t,
                             float nx, float ny)
{
    (void)nx; (void)ny;
    float qx = simplex_sample(sn, x, y + t);
    float qy = simplex_sample(sn, x + 5.2f, y + 1.3f + t);
    float rx = simplex_sample(sn, x + 2.0f * qx + 1.7f, y + 2.0f * qy + 9.2f + t);
    float ry = simplex_sample(sn, x + 2.0f * qx + 8.3f, y + 2.0f * qy + 2.8f + t);
    return simplex_fbm(sn, x + 2.0f * rx, y + 2.0f * ry + t) * 0.5f + 0.5f;
}

/* ---------- Tier 5 — COMPOSITE: multi-field combine ------------------- */

/*
 * NEBULA — two independent fbm fields multiplied. Where both are high
 * you get bright cores; where either is low, dim — yielding sparse
 * bright clouds against a darker void.
 */
static float pattern_nebula(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float a = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    float b = simplex_fbm(sn, x * 2.0f + 7.0f, y * 2.0f + 3.0f + t) * 0.5f + 0.5f;
    return clampf(a * b * 2.0f, 0.0f, 1.0f);
}

/*
 * AURORA — horizontal sin bands modulated by fbm phase, masked by a
 * Gaussian along ny (brightest at the horizon ny ≈ 0). Reads as
 * curtains of light moving across a night sky.
 */
static float pattern_aurora(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx;
    float band = sinf(x * 1.5f + simplex_fbm(sn, x, y + t) * 4.0f) * 0.5f + 0.5f;
    float vert = expf(-ny * ny * 1.5f);
    return clampf(band * vert * 1.2f, 0.0f, 1.0f);
}

/*
 * PLASMA — classic demoscene plasma: sum of three sinusoids whose
 * arguments are perturbed by shared fbm. The three sins interfere
 * to produce shifting blob fields with no preferred axis.
 */
static float pattern_plasma(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t);
    float s = sinf(x * 2.0f + v * 3.0f)
            + cosf(y * 2.5f + v * 3.0f)
            + sinf((x + y) * 1.5f + v * 2.0f);
    return (s + 3.0f) / 6.0f;
}

/*
 * LIGHTNING — (1 − |fbm|)¹². Extreme power on the ridged primitive
 * collapses almost everything to zero except the sharpest ridges,
 * which then read as branching bolts of lightning.
 */
static float pattern_lightning(const SimplexNoise *sn, float x, float y, float t,
                               float nx, float ny)
{
    (void)nx; (void)ny;
    float r = 1.0f - fabsf(simplex_fbm(sn, x, y + t));
    float p = r * r;     /*  ^2  */
    p *= p;              /*  ^4  */
    p *= p;              /*  ^8  */
    p *= r * r * r * r;  /* ^12  */
    return clampf(p * 1.8f, 0.0f, 1.0f);
}

/*
 * GALAXY — polar (r, θ) coords with spiral arms: sin(2θ + 6r + fbm·3).
 * The radial exp(−r/2) falloff lets the arms fade toward the rim.
 * Uses normalized screen-space (nx, ny) from the caller.
 */
static float pattern_galaxy(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    float r      = sqrtf(nx * nx + ny * ny);
    float a      = atan2f(ny, nx);
    float spiral = sinf(a * 2.0f + r * 6.0f + simplex_fbm(sn, x, y + t) * 3.0f);
    return clampf((spiral * 0.5f + 0.5f) * expf(-r * 0.5f) * 1.4f,
                  0.0f, 1.0f);
}

/* ---------- Tier 6 — MASKED: noise × spatial mask --------------------- */

/*
 * SOLAR — turbulence × radial exponential. Bright corona at center
 * tapers smoothly to dark at the rim. Drift accelerated ×2 so the
 * corona seethes like a sun's surface.
 */
static float pattern_solar(const SimplexNoise *sn, float x, float y, float t,
                           float nx, float ny)
{
    float r      = sqrtf(nx * nx + ny * ny);
    float corona = simplex_fbm_abs(sn, x, y + t * 2.0f);
    float mask   = expf(-r * 1.2f);
    return clampf(corona * mask * 1.8f, 0.0f, 1.0f);
}

/*
 * MOSAIC — quantise (fbm + 1)/2 into 6 discrete levels. Reads as
 * stained glass: large flat patches separated by sharp boundaries.
 * Demonstrates the same noise field through a posterise filter.
 */
static float pattern_mosaic(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float v = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    return floorf(v * 6.0f) / 5.0f;
}

/*
 * VIGNETTE — plain fbm clouds masked by (1 − 0.8·r²). The corners go
 * dark; the field reads as if viewed through a camera with a vignette
 * lens. Subtle, but gives every other pattern a frame for comparison.
 */
static float pattern_vignette(const SimplexNoise *sn, float x, float y, float t,
                              float nx, float ny)
{
    float r2   = nx * nx + ny * ny;
    float v    = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    float mask = clampf(1.0f - r2 * 0.8f, 0.0f, 1.0f);
    return v * mask;
}

/*
 * COSMOS — dim fbm background with sparse bright "stars" picked out
 * by a high-frequency single-octave simplex thresholded at 0.6. Only
 * cells where the high-freq noise spikes are lit, producing a
 * starfield over a faint nebula glow.
 */
static float pattern_cosmos(const SimplexNoise *sn, float x, float y, float t,
                            float nx, float ny)
{
    (void)nx; (void)ny;
    float bg    = simplex_fbm(sn, x, y + t * 0.3f) * 0.5f + 0.5f;
    float stars = simplex_sample(sn, x * 6.0f + 13.0f, y * 6.0f + 7.0f + t * 0.5f);
    bg *= 0.3f;
    if (stars > 0.6f) bg += (stars - 0.6f) * 2.5f;
    return clampf(bg, 0.0f, 1.0f);
}

/*
 * SUPERNOVA — radial rays from center (sin(8θ)) × global pulse
 * (sin(2t)) + a noise wash. The whole field breathes with the pulse;
 * eight bright spokes rotate as ny drifts.
 */
static float pattern_supernova(const SimplexNoise *sn, float x, float y, float t,
                               float nx, float ny)
{
    float r     = sqrtf(nx * nx + ny * ny);
    float a     = atan2f(ny, nx);
    float rays  = sinf(a * 8.0f) * 0.5f + 0.5f;
    float pulse = sinf(t * 2.0f) * 0.3f + 0.7f;
    float n     = simplex_fbm(sn, x, y + t) * 0.5f + 0.5f;
    return clampf((rays * pulse + n * 0.4f) * expf(-r * 0.8f) * 2.0f,
                  0.0f, 1.0f);
}

/* ---------- Dispatch table -------------------------------------------- */
/*
 * NoisePatternFn — function-pointer signature shared by all 30 pattern
 * samplers. Spelt out here so the dispatch table is type-safe and so
 * a reader can grep one place to see what every pattern function MUST
 * accept.
 *
 *   sn      — read-only noise context (perm table). Threaded through
 *             so patterns can't silently mutate the primitive.
 *   fx, fy  — noise-space coords (cell × NOISE_SCALE).
 *   t       — PatternState.field_time, the drift accumulator.
 *   nx, ny  — screen-space normalized coords ∈ [-1, 1] with (0, 0)
 *             at grid centre. Only radial / vertical-mask patterns
 *             use these; the rest mark them (void) and ignore.
 *
 *   return  — glow ∈ [0, 1]. Clamped by the caller too, but each
 *             pattern keeps it well-behaved.
 */
typedef float (*NoisePatternFn)(const SimplexNoise *sn,
                                float fx, float fy, float t,
                                float nx, float ny);

/*
 * NoisePattern — one row of the dispatch table: (display-name,
 * tier-label, sampler function pointer).
 *
 * INTENT
 *   Replace what would otherwise be a 30-case switch with a table
 *   lookup keyed by the Pattern enum. Adding a new pattern is then
 *   THREE coordinated edits (enum value, table row, function body)
 *   and the compiler enforces alignment via the fixed-size [N_PATTERNS]
 *   initialiser.
 *
 * CONTEXT
 *   The whole `noise_patterns[]` array (below this typedef) is the
 *   source of truth for pattern → (name, tier, fn). The Pattern enum
 *   in §1 provides readable indices; pattern_name() and pattern_tier()
 *   in this section are the only outside accessors.
 *
 * MEMBER LOGIC
 *   name   : Display name, FIXED 10-char left-padded. The constant
 *            width is deliberate — HUD column alignment must stay
 *            stable as the user cycles patterns with n/p (otherwise
 *            the HUD jitters as each name changes length).
 *   tier   : 7-char "N-LABEL " form (e.g. "3-TURB ", "5-COMP ").
 *            Encodes both the complexity tier (1-6) and the technique
 *            class (RAW/MAP/TURB/WARP/COMP/MASK). Same fixed-width
 *            rationale as `name`.
 *   sample : Function pointer to one of the 30 pattern_* functions.
 *            Indirection cost is one indirect call per cell, dwarfed
 *            by the cost of the noise samples inside the function
 *            itself: one simplex_fbm call = FBM_OCTAVES (4) calls to
 *            simplex_sample. Tier-4 FRACTAL is the worst case at 8
 *            simplex calls per cell (4 standalone + 1 fbm).
 */
typedef struct {
    const char     *name;      /* HUD-padded display name, 10 chars   */
    const char     *tier;      /* HUD-padded tier label, "N-LABEL "   */
    NoisePatternFn  sample;    /* the pattern's per-cell sampler      */
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — RAW */
    [PATTERN_CLOUDS]     = { "CLOUDS    ", "1-RAW  ", pattern_clouds     },
    [PATTERN_BILLOW]     = { "BILLOW    ", "1-RAW  ", pattern_billow     },
    [PATTERN_RIDGED]     = { "RIDGED    ", "1-RAW  ", pattern_ridged     },
    [PATTERN_WISPS]      = { "WISPS     ", "1-RAW  ", pattern_wisps      },
    [PATTERN_CRESTS]     = { "CRESTS    ", "1-RAW  ", pattern_crests     },
    /* Tier 2 — MAPPED */
    [PATTERN_CONTOURS]   = { "CONTOURS  ", "2-MAP  ", pattern_contours   },
    [PATTERN_MARBLE]     = { "MARBLE    ", "2-MAP  ", pattern_marble     },
    [PATTERN_ZEBRA]      = { "ZEBRA     ", "2-MAP  ", pattern_zebra      },
    [PATTERN_RIPPLES]    = { "RIPPLES   ", "2-MAP  ", pattern_ripples    },
    [PATTERN_THRESHOLD]  = { "THRESHOLD ", "2-MAP  ", pattern_threshold  },
    /* Tier 3 — TURBULENCE */
    [PATTERN_TURBULENCE] = { "TURBULENCE", "3-TURB ", pattern_turbulence },
    [PATTERN_STORM]      = { "STORM     ", "3-TURB ", pattern_storm      },
    [PATTERN_INFERNO]    = { "INFERNO   ", "3-TURB ", pattern_inferno    },
    [PATTERN_VEINS]      = { "VEINS     ", "3-TURB ", pattern_veins      },
    [PATTERN_EMBERS]     = { "EMBERS    ", "3-TURB ", pattern_embers     },
    /* Tier 4 — WARPED */
    [PATTERN_WARP]       = { "WARP      ", "4-WARP ", pattern_warp       },
    [PATTERN_WHIRL]      = { "WHIRL     ", "4-WARP ", pattern_whirl      },
    [PATTERN_DUNES]      = { "DUNES     ", "4-WARP ", pattern_dunes      },
    [PATTERN_CURRENTS]   = { "CURRENTS  ", "4-WARP ", pattern_currents   },
    [PATTERN_FRACTAL]    = { "FRACTAL   ", "4-WARP ", pattern_fractal    },
    /* Tier 5 — COMPOSITE */
    [PATTERN_NEBULA]     = { "NEBULA    ", "5-COMP ", pattern_nebula     },
    [PATTERN_AURORA]     = { "AURORA    ", "5-COMP ", pattern_aurora     },
    [PATTERN_PLASMA]     = { "PLASMA    ", "5-COMP ", pattern_plasma     },
    [PATTERN_LIGHTNING]  = { "LIGHTNING ", "5-COMP ", pattern_lightning  },
    [PATTERN_GALAXY]     = { "GALAXY    ", "5-COMP ", pattern_galaxy     },
    /* Tier 6 — MASKED */
    [PATTERN_SOLAR]      = { "SOLAR     ", "6-MASK ", pattern_solar      },
    [PATTERN_MOSAIC]     = { "MOSAIC    ", "6-MASK ", pattern_mosaic     },
    [PATTERN_VIGNETTE]   = { "VIGNETTE  ", "6-MASK ", pattern_vignette   },
    [PATTERN_COSMOS]     = { "COSMOS    ", "6-MASK ", pattern_cosmos     },
    [PATTERN_SUPERNOVA]  = { "SUPERNOVA ", "6-MASK ", pattern_supernova  },
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
/* §7  scene — GlowField + GlyphRamp + PatternState + PaletteState + Scene */
/* ===================================================================== */

/*
 * GlowField — the per-cell output grid the pipeline writes into and
 * the renderer reads from.
 *
 * INTENT
 *   Decouple "compute the scalar field" (scene_evaluate_field, runs
 *   on every sim tick) from "paint the scalar field" (scene_draw,
 *   runs on every render frame). Sim tick rate and frame rate can
 *   differ; the GlowField is the snapshot the renderer reads in
 *   between ticks. (For this file they're locked together but the
 *   abstraction holds regardless.)
 *
 * CONTEXT
 *   Lives on Scene (§7). Written by scene_evaluate_field() once per
 *   sim tick; read by scene_draw() once per render frame. Index with
 *   glow_field_idx(gf, x, y) = y·w + x — row-major, matches the
 *   write loops in both producer and consumer.
 *
 * MEMORY
 *   No allocation post-init — both buffers are sized at CELLS_MAX
 *   in BSS storage so the hot path never touches malloc/free (per
 *   CLAUDE.md "Memory Allocation"). Worst case ~88 KB at MAP_W_MAX
 *   × MAP_H_MAX = 200×56 = 11200 cells × (4 + 1) bytes.
 *
 * MEMBER LOGIC
 *   w, h    : Grid dimensions in cells. Set by app_pick_map_size()
 *             at init and every resize. Bounded by [16, MAP_W_MAX] ×
 *             [8, MAP_H_MAX]. Plain int so loop counter arithmetic
 *             stays sign-safe.
 *   count   : w · h, cached at reset so the buffer-clearing loop
 *             and the dispatcher don't recompute it each tick.
 *             Always ≤ CELLS_MAX (enforced by app_pick_map_size).
 *   glow[]  : Per-cell scalar value ∈ [0, 1]. Producer:
 *             scene_evaluate_field() calls the active pattern's
 *             sampler at every cell and writes here. Consumer:
 *             scene_draw() reads here via GlyphRamp to pick a
 *             glyph. Float — fully sufficient for ASCII output.
 *   band[]  : Per-cell palette band ∈ {0, 1, 2, 3} — quartile of
 *             glow, computed once per tick to save the renderer the
 *             work. Formula:
 *               band = (uint8_t)((int)(glow * 3.999f) & 3)
 *             The 3.999f is "almost 4 but never exactly"; this
 *             ensures glow == 1.0 still rounds down to band 3.
 *             Multiplying by exact 4 then masking with &3 would
 *             wrap 4 → 0 at the bright tip — the .999f keeps the
 *             upper edge safely below 4.0 so the mask is a no-op.
 *             uint8_t — only 2 bits needed, 8 are cheapest.
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
 * GlyphRamp — the density → (glyph, attr) mapping. The single rule
 * that turns the GlowField's float values into terminal characters.
 *
 * INTENT
 *   Capture "how density becomes a character" as a NAMED RULE
 *   rather than an inline if/else chain in the renderer. Three
 *   benefits: (1) the threshold values are visible as data, not
 *   magic numbers buried in code; (2) a future feature can swap
 *   ramps per theme; (3) the renderer reads `glyph_ramp_pick(...)`
 *   which scans cleaner than `if (glow > 0.65) ...`.
 *
 * THE RAMP
 *   Three descending bands; below the lowest threshold the cell is
 *   NOT drawn (the terminal background shows through — that's why
 *   patterns like NEBULA and EMBERS have visible "void" areas).
 *
 *      glow > thresh_high  →  glyph_high  bold     (dense core)
 *      glow > thresh_mid   →  glyph_mid   bold     (mid density)
 *      glow > thresh_low   →  glyph_low   normal   (faint trace)
 *      else                →  not drawn            (transparent)
 *
 * INVARIANT
 *   thresh_high > thresh_mid > thresh_low > 0
 *   Defaults: 0.65 / 0.30 / 0.05 (from GLYPH_HIGH_THRESH /
 *   GLYPH_MID_THRESH / GLOW_THRESHOLD in §1).
 *
 * GLYPH CHOICE
 *   '#', '*', '.' is a coarse 3-tier subset of Paul Bourke's
 *   canonical 10-char luminance ramp "@%#*+=-:. " (densest →
 *   sparsest). Three tiers chosen for perceptual contrast at
 *   terminal resolution — finer ramps add bytes but no visible
 *   detail at 80×24.
 *
 * Ref: Bourke, "Character representation of grey scale images"
 *      (link in References block above).
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
 * GlyphChoice — return value from glyph_ramp_pick(). A small POD
 * struct rather than out-parameters so the call site reads as a
 * pure function:
 *
 *   GlyphChoice gc = glyph_ramp_pick(ramp, glow);
 *   if (gc.visible) draw_cell(gc.glyph, palette[band], gc.attr);
 *
 * Note: caller composes (glyph, attr) with the palette band index
 * they pulled from elsewhere — GlyphRamp deliberately doesn't know
 * about colour. Separation of concerns: ramp = density, palette =
 * colour, renderer = composition.
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
 * PatternState — the active pattern + how it animates over time.
 *
 * INTENT
 *   Group the three values that together describe "the current
 *   animation": which pattern, where in its drift cycle, and at
 *   what speed. The grouping is conceptual, not technical — they
 *   change together (user presses n/p resets nothing; +/- adjusts
 *   speed; r resets time + reshuffles noise). Pulling them onto
 *   their own struct gives the animation a name.
 *
 * CONTEXT
 *   Lives on Scene (§7). Read by scene_evaluate_field() (which
 *   dispatches `noise_patterns[current].sample` with `field_time`)
 *   and by screen_draw() (which shows pattern/tier/drift in HUD).
 *   Mutated by app_handle_key() in §9 and by scene_tick().
 *
 * MEMBER LOGIC
 *   current    : Active pattern enum, indexes noise_patterns[].
 *                Defaults to PATTERN_CLOUDS at scene_init. Cycled
 *                by n/N (forward) and p/P (back) with wraparound
 *                through N_PATTERNS.
 *
 *   field_time : Drift accumulator in noise-space units. Advanced
 *                each sim tick by:
 *                  field_time += FIELD_DRIFT × drift_mult × dt
 *                Added to the y-coordinate when sampling noise so
 *                the field appears to "scroll" indefinitely.
 *                Monotonically increasing; reset only by r
 *                (scene_reset). At drift_mult=1 it advances 0.10
 *                noise units per second; that's ~2.5 cells/s of
 *                apparent motion at NOISE_SCALE=0.04.
 *
 *                FLOAT IS FINE here even after hours of runtime —
 *                simplex_sample is periodic in 256 cell-coords, so
 *                very large t just wraps inside the perm-table
 *                hash. No accumulating precision concern.
 *
 *   drift_mult : User-controlled drift-rate multiplier, scaled by
 *                +/- keys. Always a power of 2 in [DRIFT_MULT_MIN,
 *                DRIFT_MULT_MAX] = [1, 32]. Implemented as *=2 /
 *                /=2 in the key handler so the user perceives clear
 *                ×2 / ÷2 steps rather than fine-grained interpolation.
 */
typedef struct {
    Pattern current;      /* active pattern enum, indexes noise_patterns[] */
    float   field_time;   /* drift accumulator (noise-space units)         */
    int     drift_mult;   /* powers of 2 ∈ [DRIFT_MULT_MIN, DRIFT_MULT_MAX] */
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current    = PATTERN_CLOUDS;
    ps->field_time = 0.0f;
    ps->drift_mult = DRIFT_MULT_DEF;
}

/*
 * PaletteState — the active theme index.
 *
 * INTENT
 *   Currently a one-int wrapper, but kept as a named struct on
 *   Scene so the responsibility "which colour scheme is showing"
 *   has a stable home. Two concrete extensions this anticipates:
 *     (a) themed glyph ramps — adding a `GlyphRamp custom_ramp`
 *         field here so each theme can override '.', '*', '#';
 *     (b) interpolation between themes — adding `int from, to;
 *         float t;` for smooth crossfades.
 *   Both fit without disturbing Scene's layout.
 *
 * CONTEXT
 *   Read by screen_draw() (HUD shows `themes[current].name`) and
 *   by app_handle_key() in §9 (cycled by t / T). On every change
 *   the new theme's xterm colour indices are pushed into the
 *   ncurses colour pairs via theme_apply() in §3 — note this is a
 *   side effect outside the struct, since ncurses owns the actual
 *   palette state.
 *
 * MEMBER LOGIC
 *   current : Index into themes[] in §1. Always in [0, N_THEMES);
 *             wraparound is handled in the key handler via mod.
 *             Defaults to 0 (the "DEFAULT" theme) at scene_init.
 */
typedef struct {
    int current;          /* index into themes[] ∈ [0, N_THEMES) */
} PaletteState;

static void palette_state_init(PaletteState *p)
{
    p->current = 0;
}

/*
 * Scene — the composite owner of ALL mutable simulation state.
 *
 * INTENT
 *   ONE struct holds the whole world the simulation cares about,
 *   built from the four domain structs above plus the SimplexNoise
 *   primitive (§5). Every function that needs more than one piece
 *   of state takes `Scene *` instead of a long arg list; the call
 *   sites stay short and the dataflow is centralised.
 *
 * THE PIPELINE (top to bottom = dataflow order)
 *
 *   noise    — the simplex permutation table        (the ALGORITHM)
 *               ↓ (sampled by pattern functions)
 *   pattern  — active pattern + drift state         (the ANIMATION)
 *               ↓ (drives scene_evaluate_field)
 *   field    — per-cell glow + band buffers         (the DATA)
 *               ↓ (read by scene_draw)
 *   ramp     — density → glyph mapping              (the RENDER RULE)
 *               + palette colour pairs (via ncurses, set by theme_apply)
 *               ↓
 *   palette  — active theme index                   (the COLOUR CHOICE)
 *               + paused flag                       (the CONTROL)
 *
 *   The whole pipeline at a glance: noise → field (via pattern) →
 *   ramp + palette (via renderer).
 *
 * MEMORY LAYOUT
 *   GlowField is by far the largest member (~88 KB for glow[] +
 *   band[] at MAP_W_MAX × MAP_H_MAX). Placed early in the struct
 *   so the noise + pattern preamble that the inner loop touches
 *   first sits in the warm cache.
 *
 * LIFETIME
 *   scene_init    — allocates nothing, just zero-inits + seeds.
 *   scene_reset   — clears the field, zeroes the drift accumulator,
 *                   reshuffles the simplex perm table. Called on r.
 *   scene_tick    — advance drift, evaluate field.
 *   No teardown — Scene lives in BSS via App in §9; OS reclaims it.
 */
typedef struct {
    SimplexNoise  noise;     /* §5 — perm table (mutable)                   */
    GlowField     field;     /* per-cell output grid (largest member)       */
    GlyphRamp     ramp;      /* density → glyph rule (read-mostly)          */
    PatternState  pattern;   /* active pattern + drift accumulator + speed  */
    PaletteState  palette;   /* active theme index                          */
    bool          paused;    /* if true, scene_tick early-returns           */
} Scene;

/*
 * The three per-cell coordinate mappings used by scene_evaluate_field.
 * Each names one (X → Y) transform the inner loop performs.
 */

/* cell index → noise-space coordinate (cells × NOISE_SCALE). Used as
 * the (x, y) argument to every pattern sampler. */
static inline float cell_to_noise_coord(int cell)
{
    return (float)cell * NOISE_SCALE;
}

/* cell index → normalized screen coordinate ∈ [-1, +1], (0, 0) at
 * grid centre. Used by radial / vertical-mask patterns to know
 * "where on screen am I" without caring about grid resolution. */
static inline float cell_to_normalized_coord(int cell, int n_cells)
{
    if (n_cells <= 1) return 0.0f;
    return (float)cell * (2.0f / (float)(n_cells - 1)) - 1.0f;
}

/* glow ∈ [0, 1] → palette band ∈ {0, 1, 2, 3}. Quantises the
 * continuous scalar into a discrete colour tier. See the comment on
 * GLOW_TO_BAND_GAIN in §1 for the "almost-4" trick. */
static inline uint8_t glow_to_palette_band(float glow)
{
    return (uint8_t)((int)(glow * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK);
}

/*
 * scene_evaluate_field — drive the whole pipeline for one sim tick.
 *
 *   FOR every cell (x, y):
 *     fx, fy = noise-space coords        // cell_to_noise_coord
 *     nx, ny = normalized [-1, 1] coords // cell_to_normalized_coord
 *     glow   = active_pattern(noise, fx, fy, drift, nx, ny)
 *     glow   = clamp(glow, 0, 1)
 *     band   = quantize(glow → 4 tiers)  // glow_to_palette_band
 *     write (glow, band) → GlowField[x, y]
 *
 * Hot loop — one pattern call per cell × ~11K cells × 60 Hz means
 * the body must stay tight. Helpers are `static inline` for zero
 * dispatch cost.
 */
static void scene_evaluate_field(Scene *s)
{
    /* STEP 1 — RESOLVE the active pattern's sampler */
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    NoisePatternFn      sample_pattern = noise_patterns[active].sample;
    const SimplexNoise *noise          = &s->noise;
    GlowField          *field          = &s->field;
    float               drift          = s->pattern.field_time;

    /* STEP 2 — SAMPLE PER CELL: pattern → glow → band, write into GlowField */
    for (int y = 0; y < field->h; y++) {
        float ny = cell_to_normalized_coord(y, field->h);
        float fy = cell_to_noise_coord(y);
        for (int x = 0; x < field->w; x++) {
            float nx   = cell_to_normalized_coord(x, field->w);
            float fx   = cell_to_noise_coord(x);
            float glow = clampf(sample_pattern(noise, fx, fy, drift, nx, ny),
                                0.0f, 1.0f);

            int idx          = glow_field_idx(field, x, y);
            field->glow[idx] = glow;
            field->band[idx] = glow_to_palette_band(glow);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->pattern.field_time = 0.0f;
    simplex_reshuffle(&s->noise);
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

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->pattern.field_time += FIELD_DRIFT * (float)s->pattern.drift_mult * dt;
    scene_evaluate_field(s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — the terminal viewport's dimensions, cached from ncurses.
 *
 * INTENT
 *   Decouple "what the terminal is" from "what the simulation is".
 *   ncurses owns the actual screen buffer, the input loop, and the
 *   colour-pair table; Screen just remembers cols × rows so we
 *   don't call getmaxyx() once per cell. Separating from Scene
 *   also matches the pattern in sibling demos where mock-rendering
 *   paths need terminal dims without simulation state.
 *
 * LIFETIME
 *   screen_init   — calls initscr(), configures ncurses, captures
 *                   cols/rows.
 *   screen_resize — re-captures cols/rows after a SIGWINCH. Owner
 *                   (App) follows up with scene_reset to re-size
 *                   the GlowField.
 *   screen_free   — calls endwin() to restore the terminal.
 *
 * MEMBER LOGIC
 *   cols : terminal width  in cells.  ≥ 16 enforced by
 *          app_pick_map_size (smaller and the HUD won't fit).
 *   rows : terminal height in cells.  ≥ 8 enforced by
 *          app_pick_map_size; HUD reserves 3 rows at top/bottom.
 *   Ordering matches the ncurses getmaxyx(stdscr, rows, cols)
 *   convention — rows first because curses originally targeted
 *   line-printers (lines are the primary axis).
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

/* ---------- scene_draw: grid → glyphs ---------------------------------- */

/*
 * GridPlacement — the top-left screen cell where the field's (0, 0)
 * cell will be drawn. The result of centering the GlowField inside
 * the terminal viewport, holding the HUD rows clear.
 *
 * INTENT
 *   Lift the centering math out of the per-cell inner loop. Without
 *   GridPlacement the loop would have to redo `(cols - field_w) / 2`
 *   plus the HUD-row offset on every cell — w·h times per frame.
 *   With it, the loop body becomes `screen_y = origin_y + y` which
 *   the compiler can fold into the loop induction variable.
 *
 *   Also names the "where on screen" decision so a reader sees
 *   `compute_grid_placement(...)` and immediately knows what's
 *   happening, instead of staring at four lines of `/ 2` arithmetic
 *   inline.
 *
 * CONTEXT
 *   Computed once at the top of scene_draw() (§8) by
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
 *   The bottom HUD_BOTTOM_ROWS rows are kept clear by sizing the
 *   field, not by clamping here — so origin_y + field_h may run up
 *   to (rows - HUD_BOTTOM_ROWS).
 *
 *   Negative results from the centering arithmetic (over-large field
 *   on a small viewport) are clamped to the viewport edge — the
 *   field is cropped, not wrapped. The "cell off-screen?" check in
 *   the drawer's inner loop handles the rest.
 *
 * MEMBER LOGIC
 *   origin_x : screen column (cells) of the field's leftmost cell.
 *              0 if the field is wider than the viewport.
 *   origin_y : screen row (cells) of the field's topmost cell.
 *              HUD_TOP_ROWS at minimum (never overlaps the title bar).
 */
typedef struct {
    int origin_x;   /* screen column where field's left edge starts   */
    int origin_y;   /* screen row    where field's top edge starts    */
} GridPlacement;

/* Centre the GlowField inside the terminal viewport, respecting the
 * HUD's reserved top + bottom rows. Negative offsets are clamped to
 * the viewport edge so an over-large field is cropped, not wrapped. */
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

/*
 * scene_draw — paint the GlowField into the terminal using the
 * Scene's GlyphRamp and the currently-bound colour palette.
 *
 *   place = centre field inside viewport
 *   FOR every cell:
 *     glyph_pick = ramp(glow)
 *     IF glyph_pick.visible:
 *       draw glyph_pick.glyph at place + (x, y) with band's colour
 *
 * Pure read of Scene state — no simulation here.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const GlowField *field = &s->field;
    const GlyphRamp *ramp  = &s->ramp;
    GridPlacement    place = compute_grid_placement(field->w, field->h,
                                                    cols, rows);

    for (int y = 0; y < field->h; y++) {
        int screen_y = place.origin_y + y;
        if (screen_y < 0 || screen_y >= rows) continue;
        for (int x = 0; x < field->w; x++) {
            int screen_x = place.origin_x + x;
            if (screen_x < 0 || screen_x >= cols) continue;

            int   cell_idx = glow_field_idx(field, x, y);
            float glow     = field->glow[cell_idx];

            GlyphChoice pick = glyph_ramp_pick(ramp, glow);
            if (!pick.visible) continue;

            int color_pair = PAIR_BAND_BASE
                           + (field->band[cell_idx] & PALETTE_BAND_MASK);
            draw_glyph_at(screen_y, screen_x,
                          pick.glyph, color_pair, pick.attr);
        }
    }
}

/* ---------- HUD layout widths ----------------------------------------- *
 *
 * Per-segment widths for the status row 1 chain. Each `hud_draw_*`
 * helper consumes (and returns) the cursor x; these widths are how
 * far each segment moves it. Kept named so a future format tweak
 * touches one line.
 */
#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " SIMPLEX NOISE CLOUDS "
#define HUD_BOTTOM_HINT_TEXT \
    " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit "

/* ---------- HUD: per-segment drawers ---------------------------------- *
 *
 * Each segment takes (row, x, ...) and returns the next x cursor
 * position so the caller can chain them. Names describe what's
 * INSIDE the segment, not how it looks ("pattern_field" not
 * "left_chunk").
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

/* Row 1 tail — algorithm parameter readout: noise scale, fBm octaves,
 * grid dimensions. Read-only display of the values that govern the
 * pattern functions in §6. */
static void hud_draw_stats_field(int row, int x, const GlowField *gf)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  scale:%.2f  oct:%d  map:%dx%d ",
             NOISE_SCALE, FBM_OCTAVES, gf->w, gf->h);
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
 * screen_draw — the per-frame composite renderer. Pure read of state.
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
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the top-level harness. Owns every other piece of state.
 *
 * INTENT
 *   ONE struct at the top of the dependency tree, holding the
 *   simulation (Scene), the terminal binding (Screen), the tick
 *   rate, the chosen map dimensions, and the two signal flags
 *   driving the main loop. main() touches App and only App; every
 *   other function takes App* (for write paths) or const App*
 *   (for read paths) — or a sub-struct pointer if it only needs
 *   part of the state.
 *
 * SINGLETON
 *   One file-scope instance (g_app, see below). Signal handlers
 *   need to reach the `running` / `need_resize` flags from outside
 *   any caller's scope, and POSIX signal handlers can't take a
 *   user pointer — a file-scope `App` is the simplest correct
 *   path. The handlers ONLY touch the volatile sig_atomic_t flags;
 *   the main loop owns everything else.
 *
 * LIFETIME
 *   main() does: zero-init g_app → screen_init → app_pick_map_size →
 *   scene_init → loop → screen_free. No teardown beyond endwin()
 *   via atexit() — App is in BSS so the OS reclaims it at exit.
 *
 * MEMBER LOGIC
 *   scene       : The simulation. See Scene above.
 *
 *   screen      : Terminal cols/rows. Refreshed on every SIGWINCH
 *                 by app_do_resize().
 *
 *   sim_fps     : Tick rate in Hz. Determines TICK_NS used by the
 *                 fixed-timestep accumulator in main(). Adjustable
 *                 by ] (faster) / [ (slower) keys; clamped to
 *                 [SIM_FPS_MIN, SIM_FPS_MAX] = [10, 240].
 *
 *   map_w,      : Grid dimensions selected by app_pick_map_size()
 *   map_h         from the terminal cols/rows minus HUD reservations.
 *                 Bounded by [16..MAP_W_MAX] × [8..MAP_H_MAX].
 *                 Passed to scene_reset on init and resize.
 *
 *   running     : Cleared to 0 by SIGINT or SIGTERM handlers (and
 *                 by app_handle_key on q/ESC). Main loop exits when
 *                 this goes false. `volatile sig_atomic_t` because:
 *                 (1) POSIX guarantees only sig_atomic_t writes are
 *                 async-safe inside a signal handler; (2) `volatile`
 *                 prevents the compiler from caching the read in
 *                 the main loop (the handler can flip it between
 *                 any two iterations).
 *
 *   need_resize : Set to 1 by SIGWINCH handler; consumed by main
 *                 loop, which then calls app_do_resize() and clears
 *                 it. Same volatile-sig_atomic_t rationale as
 *                 `running`. The signal → flag → main-loop-consumes
 *                 pattern keeps the handler tiny and signal-safe
 *                 (no libc calls beyond what's listed in
 *                 signal-safety(7)).
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

/* Drift speed control — multiplier doubles or halves in clean ×2 / ÷2
 * steps so the user perceives discrete speed changes, not a slider. */
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
 * grouped under one name.
 *
 * INTENT
 *   The original main loop carried FIVE loose locals (frame_time,
 *   sim_accum, fps_accum, frame_count, fps_display) that all had
 *   to be initialised together, reset together on resize, and
 *   advanced together each frame. Grouping them makes the loop
 *   body a sequence of NAMED operations on one object
 *   (`frame_clock_advance`, `fps_meter_observe`, …) instead of a
 *   collection of bare arithmetic against scattered locals.
 *
 * CONTEXT
 *   One instance, declared as a `FrameClock clock;` local in main()
 *   (§9). The lifetime is exactly main's stack frame. Owns no
 *   resources; never freed; reset via frame_clock_resync() after a
 *   stall (e.g. SIGWINCH).
 *
 * THE TWO SUB-ACCUMULATORS
 *   FrameClock combines two independent integrator patterns that
 *   happen to share `frame_dt_ns` as their input:
 *
 *     1) FIXED-TIMESTEP SIM DRIVER  (sim_accum_ns)
 *        Each frame's wall-clock dt is poured into the accumulator;
 *        the accumulator is drained one TICK_NS chunk at a time into
 *        scene_tick() calls. The simulation sees a STABLE per-tick
 *        dt regardless of frame jitter. This is the well-known
 *        "Glenn Fiedler / 'Fix Your Timestep!'" pattern.
 *
 *     2) FPS METER                  (fps_accum_ns + fps_frame_count)
 *        Sum frame dts and count frames; every FPS_UPDATE_MS of wall
 *        clock, divide frames/seconds to refresh `fps_display`. The
 *        HUD reads `fps_display` only — never the running accums.
 *
 * MEMORY / COST
 *   3 × int64 + 1 × int + 1 × double = 36 bytes. Stack-resident in
 *   main(). Touched a handful of times per frame; cache cost trivial.
 *
 * MEMBER LOGIC
 *   frame_time_ns   : Wall clock (CLOCK_MONOTONIC, in nanoseconds)
 *                     at the START of the current frame. The next
 *                     frame's dt = clock_ns() − frame_time_ns.
 *                     int64 because nanoseconds since boot doesn't
 *                     fit in 32 bits.
 *
 *   sim_accum_ns    : Wall-clock ns received from frames but not yet
 *                     drained into sim ticks. Always ≥ 0; never
 *                     resets except on stall (frame_clock_resync).
 *                     In steady state it oscillates between 0 and
 *                     TICK_NS as the loop pumps it.
 *
 *   fps_accum_ns    : Wall-clock ns since the last FPS readout
 *                     refresh. Reset to 0 every FPS_UPDATE_MS.
 *
 *   fps_frame_count : Number of frames observed since the last FPS
 *                     readout refresh. Reset alongside fps_accum_ns.
 *                     Plain int — even 10,000 fps for 500 ms is
 *                     5000, nowhere near INT_MAX.
 *
 *   fps_display     : Last computed instantaneous frame rate
 *                     (frames per second, as a double). The ONE
 *                     member the renderer / HUD actually reads;
 *                     the accumulators above only feed this.
 *
 * Refs:
 *   • Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com)
 *     — canonical write-up of the accumulator+fixed-step pattern.
 *   • POSIX clock_gettime(CLOCK_MONOTONIC) — the wall-clock source
 *     (in §2 clock); guaranteed strictly increasing, immune to NTP
 *     jumps, suitable for frame timing.
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

/* Sleep so the render loop spins at RENDER_FPS_TARGET. Compute how
 * much wall-clock has been consumed this iteration (tick work + the
 * frame-to-frame gap) and sleep the remainder of the frame budget. */
static void app_throttle_to_render_rate(int64_t frame_start_ns,
                                        int64_t frame_dt_ns)
{
    int64_t target_frame_period_ns = NS_PER_SEC / RENDER_FPS_TARGET;
    int64_t time_consumed_ns       = clock_ns() - frame_start_ns
                                   + frame_dt_ns;
    clock_sleep_ns(target_frame_period_ns - time_consumed_ns);
}

/* Drain one key event (non-blocking) and route it to app_handle_key.
 * Quit returns false from the handler → we clear app->running. */
static void app_pump_input(App *app)
{
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
        app->running = 0;
}

/* Wire SIGINT / SIGTERM → exit flag, SIGWINCH → resize flag.
 * Handler bodies are tiny (one volatile flip each) so they stay
 * inside the POSIX signal-safety envelope. */
static void app_install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/*
 * main — top-level driver. Reads as pseudocode:
 *
 *   bootstrap (PRNG seed, signal handlers, atexit cleanup)
 *   init terminal → pick grid size → init simulation
 *   init frame clock
 *   LOOP until app.running goes false:
 *     handle pending resize
 *     advance frame clock → frame_dt
 *     run fixed-step sim ticks (Fiedler)
 *     fps meter
 *     throttle to render rate
 *     draw scene + HUD; present
 *     pump input (may clear running)
 *   shutdown terminal
 */
int main(void)
{
    /* STEP 1 — bootstrap */
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    app_install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    /* STEP 2 — terminal + simulation init */
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    /* STEP 3 — main loop */
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

    /* STEP 4 — shutdown */
    screen_free(&app->screen);
    return 0;
}
