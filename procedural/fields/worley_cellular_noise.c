/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * worley_cellular_noise.c
 *   — Steven Worley's cellular / Voronoi-style noise, 30 patterns
 *     across 6 tiers.
 *
 * DEMO: A scattered field of "feature points" — one per integer tile
 *       of noise space — drives 30 visually distinct patterns, all
 *       genuinely derived from the Worley/Voronoi primitive. For any
 *       cell on screen the algorithm finds the three nearest feature
 *       points among the 3×3 surrounding tiles, then the active
 *       pattern decides what to render with those distances. Patterns
 *       are grouped in 6 complexity tiers (cycle with n/p):
 *         Tier 1 F-FUNCTIONS — raw F_n outputs (F1, F2, F3, F2−F1,
 *                              F1/F2): the classic Worley building
 *                              blocks
 *         Tier 2 F-COMBOS    — other arithmetic on F values (F1+F2,
 *                              F1·F2, F3−F2, 1−(F2−F1), 1−F1)
 *         Tier 3 METRICS     — F1 with non-Euclidean distance norms
 *                              (Manhattan, Chebyshev, super-ellipse
 *                              Lp=3, star Lp=½, stretched anisotropic)
 *         Tier 4 IDENTITY    — cell_id hash drives a visual property
 *                              (solid colour, multiplicatively-weighted
 *                              Voronoi, twinkle, random glow, crackle)
 *         Tier 5 MULTI-SCALE — Worley fBm stacks (fBm, turbulence,
 *                              ridged, nested, checker)
 *         Tier 6 HYBRIDS     — Worley-on-Worley compositions
 *                              (domain warp, metric blend, halo,
 *                              energy, chaos)
 *       Feature points wobble slightly with time, so all patterns
 *       evolve organically rather than freezing.
 *
 * Study alongside:
 *   ./domain_warped_noise_iq_style.c — Perlin-based field showcases.
 *       Worley vs Perlin: same "evaluate a noise field at every cell"
 *       skeleton, but Worley is DISCRETE (distance to feature points)
 *       where Perlin is CONTINUOUS (smooth gradient noise). Perlin
 *       gives rolling hills; Worley gives cells with hard or soft
 *       edges depending on the chosen distance function.
 *   ../generational/voronoi_region_map.c — Worley's CELL_ID pattern
 *       is essentially a streaming Voronoi diagram. The voronoi file
 *       precomputes everything; this one evaluates on-demand and
 *       lets every cell's owner change as feature points wobble.
 *
 * Section map:
 *   §1 config   — grid, patterns, scale, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (4 palette colours each)
 *   §5 noise    — hash, feature-point sampling, F1/F2/F3 query
 *                  under N distance metrics
 *   §6 patterns — 30 noise mappings in 6 tiers + dispatch table
 *   §7 scene    — Field, scene state, per-frame grid update
 *   §8 screen   — ASCII render: density-graded glyphs
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new feature-point hash seed)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster wobble drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra worley_cellular_noise.c \
 *       -o worley -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Worley noise (Worley 1996, "A cellular texture basis
 *                  function"). Divide the plane into integer-aligned
 *                  unit tiles. Each tile contains exactly one "feature
 *                  point" whose sub-tile position is determined by a
 *                  deterministic hash of the tile coordinates plus a
 *                  per-run seed. To evaluate the noise at a query
 *                  point (x, y):
 *                    1. Find the integer tile (xi, yi) containing it.
 *                    2. For each of the 3×3 neighbouring tiles
 *                       (xi+dx, yi+dy), dx, dy ∈ {-1, 0, 1}:
 *                       a. Hash the tile coords to get the feature
 *                          point's position in [0, 1)² inside the tile.
 *                       b. Compute the Euclidean (or Manhattan, or
 *                          Chebyshev) distance from (x, y) to that
 *                          point.
 *                    3. Track F1 = nearest, F2 = second-nearest.
 *                  The 3×3 lookup is sufficient: any feature point in
 *                  a tile further than 1 cell away is necessarily
 *                  beyond distance √2, larger than any in-bounds F1
 *                  candidate. The function is fast (constant work per
 *                  query) and produces patterns whose CELL STRUCTURE
 *                  matches a Voronoi diagram of the feature points.
 *
 *                  Thirty patterns, organised into six tiers, build on
 *                  this primitive: raw F_n values (F1/F2/F3/F2−F1/…),
 *                  arithmetic combos, alternative distance metrics
 *                  (Manhattan/Chebyshev/Minkowski-p), cell-id-driven
 *                  effects (multiplicatively-weighted Voronoi,
 *                  twinkle), Worley fBm stacks at multiple scales,
 *                  and Worley-on-Worley compositions (domain warp,
 *                  metric blend, halo). They share the same query
 *                  loop; only the metric and the post-query mapping
 *                  differ.
 *
 * Data-structure : One uint32_t hash seed (re-rolled at reset) and a
 *                  hash function — no pre-allocated point list. Every
 *                  feature point is implicit: (cx, cy) → hash → (ox, oy).
 *                  That makes Worley noise infinite-extent and
 *                  zero-memory; we don't store any of it.
 *
 *                  Per-cell glow + colour buffers as in the other
 *                  field showcases. Field overwritten each frame.
 *
 * Rendering      : ASCII only. Density-graded glyphs ('.', '*', '#')
 *                  in 4 theme palette colours. Pattern picks both glow
 *                  and colour band from F1/F2/F3 and the cell identity.
 *                  Animation: each feature point wobbles with a phase
 *                  derived from its hash + a time accumulator, so cell
 *                  boundaries breathe and shift slowly.
 *
 * Performance    : 1+ worley_query per cell per frame. Each query
 *                  examines N_WORLEY_NEIGHBOURS (9) tiles → 9 hashes
 *                  + 9 distance computations. Tier 1-4 patterns call
 *                  worley_query once; Tier 5 (multi-scale fBm) and
 *                  Tier 6 (hybrids) call it 2-3 times. On a 200×56
 *                  grid at 60 Hz that's ~6 M hashes/sec for Tier 1-4
 *                  and up to ~18 M/sec for the heaviest patterns —
 *                  still well under CPU budget. The hash itself is a
 *                  4-mul integer mixer; no float math beyond the
 *                  distance calculation.
 *
 * References     : CELLULAR / VORONOI THEORY
 *                  • Worley, S. (1996) — "A cellular texture basis
 *                    function", SIGGRAPH'96. The original paper.
 *                    F1/F2/F3 conventions and the 3×3-tile lookup
 *                    invariant come from here:
 *                    https://dl.acm.org/doi/10.1145/237170.237267
 *                  • Ebert, Musgrave, Peachey, Perlin, Worley —
 *                    "Texturing & Modeling: A Procedural Approach"
 *                    (Morgan Kaufmann, 3rd ed. 2002, ISBN
 *                    1-55860-848-6). Worley's own chapter expands
 *                    the F_n combos used in Tier 1-2 (F1+F2, F1·F2,
 *                    F3−F2 …) and discusses the metric variants of
 *                    Tier 3.
 *                  • Aurenhammer, F. (1991) — "Voronoi diagrams: a
 *                    survey of a fundamental geometric data
 *                    structure", ACM Comput. Surv. 23(3). Reference
 *                    for the multiplicatively-weighted Voronoi
 *                    diagram (Tier 4 WEIGHTED) and the metric-
 *                    dependent cell shapes (Tier 3):
 *                    https://dl.acm.org/doi/10.1145/116873.116880
 *
 *                  WORLEY-DERIVED PATTERNS (the 30 patterns)
 *                  • Quilez, I. — "Voronoi - basic / Voronoi lines".
 *                    The F2 − F1 edge trick (this file's default
 *                    boot pattern), with the "thinness multiplier"
 *                    explained:
 *                    https://iquilezles.org/articles/voronoilines/
 *                  • Quilez, I. — "Voronoise". Parameterised
 *                    interpolation between cellular noise and a
 *                    Perlin-like smooth field; generalises Tier 4
 *                    RANDOM_GLOW / TWINKLE:
 *                    https://iquilezles.org/articles/voronoise/
 *                  • Quilez, I. — "Domain warping". Direct source
 *                    for Tier 6 DOMAIN_WARP and CHAOS — IQ's
 *                    fbm(p + fbm(p)) construction applies to
 *                    Worley fields just as well:
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
 *                    cube + 24 greys) that themes[] in §1 indexes:
 *                    https://en.wikipedia.org/wiki/ANSI_escape_code#8-bit
 *                  • Padala, P. — "NCURSES Programming HOWTO".
 *                    Pragmatic reference for the §3/§8 rendering
 *                    APIs (init_pair, mvaddch, doupdate, A_BOLD):
 *                    https://tldp.org/HOWTO/NCURSES-Programming-HOWTO/
 *
 *                  COMPARE IN PROJECT
 *                  • ./simplex_noise_clouds.c — simplex's smooth
 *                    gradient field vs Worley's discrete cells;
 *                    same fBm idea, very different visuals.
 *                  • ./perin_noise_flow_showcase.c — Perlin's 1985
 *                    predecessor (gradient noise, not cellular).
 *                  • ../generational/voronoi_region_map.c — a
 *                    precomputed Voronoi diagram (static lattice)
 *                    vs this file's on-demand feature-point hashing
 *                    (streaming, wobbling).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Scatter random "stars" across a grid, one per square tile. For any
 * point in space, ask "how far am I from the closest star?". That
 * distance is your noise value. Repeat for every screen cell and you
 * get a cellular pattern — dark near each star, brighter as you move
 * away from any star, brightest along the lines exactly between two
 * stars. Different distance metrics (Euclidean / Manhattan /
 * Chebyshev) produce different cell shapes (circles / diamonds /
 * squares). Different functions of the distances (F1 / F2 / F2-F1)
 * highlight different features (centres / edges / blobs).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Each tile of the grid harbours one hidden firefly. The firefly's
 * exact position in the tile is determined by a HASH — the same tile
 * coordinates always give the same firefly position, but two adjacent
 * tiles see un-correlated positions. To shade any pixel, ask: "of
 * the 9 nearby fireflies (this tile + 8 neighbours), which is closest?"
 * Use that distance as the brightness. The result is a pattern where
 * every firefly is at a dark centre, bordered by bright "moats"
 * exactly halfway to its neighbours.
 *
 * Visible layers:
 *   1. The CELL STRUCTURE — visible in every pattern as the locus of
 *      brightest cells, which traces the Voronoi diagram of the
 *      hidden feature points.
 *   2. The PATTERN-SPECIFIC mapping — what we DO with F1/F2/F3 and
 *      cell-id determines whether centres or edges are bright,
 *      whether cells are circles or diamonds, whether each cell has
 *      its own colour or fades smoothly into the next.
 *   3. The TIME WOBBLE — feature points oscillate slightly within
 *      their tiles, so cell boundaries shift continuously. The Voronoi
 *      structure is visible but never frozen.
 *
 * ALGORITHM IN STEPS  (per cell, per frame)
 * ──────────────────
 *  1. Convert cell coordinate to noise space: (fx, fy) = (x, y) ·
 *     NOISE_SCALE.
 *  2. Find integer tile: (xi, yi) = (⌊fx⌋, ⌊fy⌋).
 *  3. Initialise F1 = F2 = F3 = +∞, cell_id = 0.
 *  4. For each of the 9 tiles around (xi, yi):
 *     a. Hash tile coords + run-seed → 32-bit value h.
 *     b. Extract (ox, oy) ∈ [0, 1)² from h's bits.
 *     c. Wobble: ox += A · sin(t + phase_x); oy += A · cos(t + phase_y).
 *        Phases come from other bits of h so neighbouring fireflies
 *        wobble at different rates.
 *     d. Feature point position: (cx + ox, cy + oy).
 *     e. Distance d to (fx, fy) under the active metric.
 *     f. If d < F1: F3=F2; F2=F1; F1=d; cell_id=h.
 *        Else if d < F2: F3=F2; F2=d.
 *        Else if d < F3: F3=d.
 *  5. Pattern function: turn (F1, F2, F3, cell_id) into (glow, colour
 *     band). Tier-5 and Tier-6 patterns call worley_query multiple
 *     times (different scales / different metrics / warped coords)
 *     and combine the results.
 *  6. Write to GlowField.glow[cell] and GlowField.band[cell].
 *
 * KEY FORMULAS
 * ────────────
 *  Hash mixer (32-bit)         : h = mix(xi · 374761393 + yi · 668265263 + seed)
 *  Feature pos in tile         : (ox, oy) = (high_16(h), low_16(h)) / 2¹⁶
 *  Wobble                      : ox' = ox + W · sin(t + phase),
 *                                oy' = oy + W · cos(t + phase)
 *  Euclidean distance (L²)     : √(Δx² + Δy²)
 *  Manhattan distance (L¹)     : |Δx| + |Δy|
 *  Chebyshev distance (L∞)     : max(|Δx|, |Δy|)
 *  Minkowski distance (Lp)     : (|Δx|^p + |Δy|^p)^(1/p)
 *  Star "metric" (L½, non-norm): (√|Δx| + √|Δy|)²
 *  F2 − F1 (edge highlight)    : near-zero on cell boundaries
 *  Worley fBm                  : Σᵢ aᵢ · F1(2ⁱ·x), aᵢ = 1/2ⁱ
 *  Weighted Voronoi            : argmin over points  d_i / w_i,
 *                                w_i = hash-derived "radius"; cells
 *                                with bigger w_i claim more territory
 *  Per-pattern formulas inline next to each pattern_* function in §6;
 *  dispatch table is `noise_patterns[]` (same section).
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
 *   row 0           : title + state bar (fps, Hz, pattern, drift)
 *   row 1           : pattern + theme + palette swatches + parameters
 *   row HUD_TOP..N-2: noise-field map
 *   row N-1         : keyboard action hint
 */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Noise scale: ~0.1 noise-units per cell → 1 tile every 10 cells →
 * about 20 × 6 = 120 cells across a 200 × 56 grid. */
#define NOISE_SCALE         0.10f

/* Field drift in noise-coord units per second (drives the wobble's
 * time component). */
#define FIELD_DRIFT         0.40f

/* Wobble amplitude. Keep < 0.4 to ensure feature points stay roughly
 * inside their original tiles (preserves the 3×3 lookup invariant). */
#define WOBBLE_AMOUNT       0.20f

/* Drift multiplier — scaled by +/-, capped low so the cells don't
 * boil too aggressively. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      16

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Palette quantisation — continuous glow ∈ [0, 1] → discrete band
 * index ∈ {0..3}. Same scheme as the simplex file.
 *
 *   N_PALETTE_BANDS    — # of tiers (matches Theme.band[] length).
 *   PALETTE_BAND_MASK  — cheap `& MASK` modulo (works because
 *                         N_PALETTE_BANDS is a power of two).
 *   GLOW_TO_BAND_GAIN  — glow * GAIN → int 0..3 via truncation.
 *                         Set to N_PALETTE_BANDS − ε so glow == 1.0
 *                         still floors to band 3 (not band 4).
 */
#define N_PALETTE_BANDS     4
#define PALETTE_BAND_MASK   3       /* N_PALETTE_BANDS - 1 */
#define GLOW_TO_BAND_GAIN   3.999f  /* almost-4 to keep glow=1.0 → band 3 */

/*
 * Worley algorithm sentinels & sizing.
 *
 *   WORLEY_DISTANCE_INF      — "no candidate seen yet" sentinel for
 *                               F1/F2/F3 in worley_result_empty.
 *                               Any plausible distance under our
 *                               metrics is < 5, so 1e10 is "definitely
 *                               larger than any real candidate".
 *   WORLEY_NEIGHBOUR_RADIUS  — radius of the 3×3 tile scan (=1).
 *                               Provably sufficient under L¹/L²/L∞
 *                               when WOBBLE_AMOUNT < 0.5; see CONCEPTS.
 *   N_WORLEY_NEIGHBOURS      — (2·R + 1)² = 9 tiles examined per query.
 */
#define WORLEY_DISTANCE_INF      1.0e10f
#define WORLEY_NEIGHBOUR_RADIUS  1
#define N_WORLEY_NEIGHBOURS      9

/*
 * Hash → unit-float / phase / byte conversions.
 *
 *   HASH_UNIT_FROM_16BIT  — convert a 16-bit hash slice (0..65535)
 *                            to a unit float in [0, 1]. Used by
 *                            hash_to_offset to place a feature point
 *                            inside its tile.
 *   HASH_UNIT_FROM_BYTE   — convert one hash byte (0..255) to [0, 1].
 *                            Used by Tier-4 patterns for per-cell
 *                            random properties.
 *   HASH_PHASE_FROM_BYTE  — convert one hash byte to a phase in
 *                            radians. /40 ≈ 2π/255 spans roughly one
 *                            full sine cycle, so different bytes
 *                            give visually independent wobble phases.
 *   HASH_PERSONALITY_SALT — XOR'd into cell_id before re-hashing in
 *                            cell_hash_to_unit. The 0xa5 alternating-
 *                            bits byte is a canonical debug pattern;
 *                            its role here is to decorrelate
 *                            consecutive cell_ids in the rare event
 *                            two adjacent cells hash close together.
 */
#define HASH_UNIT_FROM_16BIT    (1.0f / 65535.0f)
#define HASH_UNIT_FROM_BYTE     (1.0f / 255.0f)
#define HASH_PHASE_FROM_BYTE    (1.0f / 40.0f)
#define HASH_PERSONALITY_SALT   0xa5a5a5a5u

/*
 * Weighted Voronoi (Tier 4 WEIGHTED).
 *
 * Each feature point in the weighted variant has a "territory weight"
 * drawn from one hash byte:  w = WEIGHTED_W_MIN + b/255 · RANGE.
 * The cell boundary between two weighted points sits where d/w
 * equalises — points with larger w claim more area. [0.5, 1.5] gives
 * roughly 3:1 area asymmetry between the extreme cases.
 */
#define WEIGHTED_W_MIN          0.5f
#define WEIGHTED_W_RANGE        1.0f   /* MAX = MIN + RANGE = 1.5 */

/*
 * Stretched metric (Tier 3 STRETCHED). Anisotropic Manhattan:
 *   d = STRETCHED_X_WEIGHT · |Δx|  +  |Δy|
 * with x-weight > 1 the cells stretch along the y-axis (a horizontal
 * unit step costs more than a vertical one).
 */
#define STRETCHED_X_WEIGHT      2.0f

/*
 * Pattern — thirty Worley/Voronoi mappings, grouped into six
 * complexity tiers. Cycle with n/p. The enum order MUST match the
 * `noise_patterns[]` dispatch table in §6 (compiler enforces this
 * via the fixed-size [N_PATTERNS] array initialiser — a missing or
 * mis-ordered row is a compile error).
 *
 *   Tier 1 F-FUNCTIONS : F1, F2, F3, F2_F1, F1_OVER_F2
 *   Tier 2 F-COMBOS    : F1_PLUS_F2, F1_TIMES_F2, F3_F2,
 *                         F2_F1_INV, F1_INV
 *   Tier 3 METRICS     : MANHATTAN, CHEBYSHEV, SUPERELLIPSE,
 *                         STAR, STRETCHED
 *   Tier 4 IDENTITY    : CELL_ID, WEIGHTED, TWINKLE,
 *                         RANDOM_GLOW, CRACKLE
 *   Tier 5 MULTI-SCALE : WORLEY_FBM, WORLEY_TURBULENCE,
 *                         WORLEY_RIDGED, NESTED, CHECKER
 *   Tier 6 HYBRIDS     : DOMAIN_WARP, METRIC_BLEND, HALO,
 *                         ENERGY, CHAOS
 */
typedef enum {
    /* Tier 1 — F-FUNCTIONS: raw F_n outputs */
    PATTERN_F1 = 0,
    PATTERN_F2,
    PATTERN_F3,
    PATTERN_F2_F1,
    PATTERN_F1_OVER_F2,
    /* Tier 2 — F-COMBOS: arithmetic on F values */
    PATTERN_F1_PLUS_F2,
    PATTERN_F1_TIMES_F2,
    PATTERN_F3_F2,
    PATTERN_F2_F1_INV,
    PATTERN_F1_INV,
    /* Tier 3 — METRICS: F1 with different distance norms */
    PATTERN_MANHATTAN,
    PATTERN_CHEBYSHEV,
    PATTERN_SUPERELLIPSE,
    PATTERN_STAR,
    PATTERN_STRETCHED,
    /* Tier 4 — IDENTITY: cell_id hash drives a visual property */
    PATTERN_CELL_ID,
    PATTERN_WEIGHTED,
    PATTERN_TWINKLE,
    PATTERN_RANDOM_GLOW,
    PATTERN_CRACKLE,
    /* Tier 5 — MULTI-SCALE: Worley fBm stacks */
    PATTERN_WORLEY_FBM,
    PATTERN_WORLEY_TURBULENCE,
    PATTERN_WORLEY_RIDGED,
    PATTERN_NESTED,
    PATTERN_CHECKER,
    /* Tier 6 — HYBRIDS: Worley-on-Worley compositions */
    PATTERN_DOMAIN_WARP,
    PATTERN_METRIC_BLEND,
    PATTERN_HALO,
    PATTERN_ENERGY,
    PATTERN_CHAOS,
    N_PATTERNS,
} Pattern;

/* pattern_name() / pattern_tier() defined in §6 alongside dispatch table. */
static const char *pattern_name(Pattern p);
static const char *pattern_tier(Pattern p);

/*
 * Distance metrics — Lp norms plus one anisotropic variant.
 * Used by worley_query as a per-call parameter; each pattern picks
 * its own (Tier 3 patterns vary the metric, the rest default to
 * Euclidean).
 *
 *   EUCLIDEAN    p = 2     — round cells (default; Worley's
 *                             original)
 *   MANHATTAN    p = 1     — diamond cells (L¹ taxicab)
 *   CHEBYSHEV    p = ∞     — square cells (L∞)
 *   SUPERELLIPSE p = 3     — rounded squares (Lp, p > 2)
 *   STAR         p = 0.5   — concave 4-pointed stars (not a true
 *                             metric; triangle inequality fails)
 *   STRETCHED    weighted  — Manhattan-like with 2× weight on x:
 *                             2|Δx| + |Δy| → cells stretched along y
 */
typedef enum {
    METRIC_EUCLIDEAN    = 0,
    METRIC_MANHATTAN    = 1,
    METRIC_CHEBYSHEV    = 2,
    METRIC_SUPERELLIPSE = 3,
    METRIC_STAR         = 4,
    METRIC_STRETCHED    = 5,
} Metric;

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Frame-loop timing limits.
 *
 *   MAX_FRAME_DT_NS    — cap per-frame dt at 100 ms before feeding
 *                         the fixed-timestep accumulator. Without
 *                         this cap, a long stall (debugger, SIGSTOP,
 *                         swap-storm) would queue thousands of sim
 *                         ticks to fire when the process resumes —
 *                         the "spiral of death" failure mode (see
 *                         Glenn Fiedler, "Fix Your Timestep!").
 *   RENDER_FPS_TARGET  — render-throttle target. The sleep before
 *                         I/O paces the render loop at this rate
 *                         even when sim_fps is set higher.
 */
#define MAX_FRAME_DT_NS    (100 * NS_PER_MS)
#define RENDER_FPS_TARGET  60

/*
 * Theme — a named 4-tier colour palette + accent for the cellular
 * noise renderer.
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
 *            Vestigial in this file (no event flash) — kept so the
 *            theme table stays compatible with sibling demos that
 *            DO use the accent.
 *
 * Ref: ANSI escape code § 8-bit (link in References block).
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
/* §5  noise — hash, feature points, F1 / F2 / F3 query under N metrics   */
/* ===================================================================== */

/*
 * WorleyNoise — the noise primitive's runtime state. Just a 32-bit
 * seed; cellular noise (unlike Perlin/simplex) doesn't need any
 * permutation table — the per-tile feature point is HASHED on demand
 * from (xi, yi, seed). That makes the noise infinite-extent and
 * zero-memory.
 *
 * The struct lives on Scene (§7). Pattern functions in §6 receive
 * `const WorleyNoise *` so the dataflow is explicit at the call site:
 * a pattern *samples* noise but never *mutates* it. Reseeding happens
 * only via worley_reseed() on user reset (r key).
 *
 * Ref: Worley 1996 §4 — "A perfect hash function of three integers
 *      (i, j, k) → number of points in tile (i, j, k)" is how the
 *      paper avoids storing the feature points.
 */
typedef struct {
    uint32_t seed;
} WorleyNoise;

/*
 * hash32 — small integer mixer. 4 multiplies + 4 shifts. Output is a
 * decent-quality 32-bit hash for visual noise (passes basic chi-square
 * but not crypto-grade). Good enough for picking feature-point
 * positions deterministically.
 *
 * Constants from the "splitmix64" family adapted to 32 bits — see
 * Murmur / xxHash literature for similar mixers.
 */
static inline uint32_t hash32(uint32_t x)
{
    x = (x ^ (x >> 16)) * 0x7feb352du;
    x = (x ^ (x >> 15)) * 0x846ca68bu;
    x = (x ^ (x >> 16));
    return x;
}

/*
 * tile_hash — 32-bit hash of (xi, yi, wn->seed). Different (xi, yi)
 * pairs almost always give different hash values; the seed re-rolls
 * the entire space for variety across runs.
 */
static inline uint32_t tile_hash(const WorleyNoise *wn, int xi, int yi)
{
    uint32_t h = (uint32_t)xi * 374761393u
               + (uint32_t)yi * 668265263u
               + wn->seed;
    return hash32(h);
}

/*
 * hash_byte / hash_halfword — extract a named bit-slice from a 32-bit
 * hash so the noise code reads "byte 1 of the hash" instead of
 * "(h >> 8) & 0xFFu". Each call costs one shift + one mask; inlined
 * everywhere.
 *
 *   byte_idx ∈ {0..3}  → bits [8·idx, 8·idx + 8)
 *   half_idx ∈ {0, 1}  → bits [16·idx, 16·idx + 16)
 */
static inline uint32_t hash_byte(uint32_t h, int byte_idx)
{
    return (h >> (byte_idx * 8)) & 0xFFu;
}

static inline uint32_t hash_halfword(uint32_t h, int half_idx)
{
    return (h >> (half_idx * 16)) & 0xFFFFu;
}

/*
 * hash_to_offset — pick a feature point's position in [0, 1)² inside
 * a tile, from a 32-bit hash. High 16 bits drive ox, low 16 bits
 * drive oy — statistically uncorrelated enough for visual noise.
 */
static inline void hash_to_offset(uint32_t h, float *ox, float *oy)
{
    *ox = (float)hash_halfword(h, 1) * HASH_UNIT_FROM_16BIT;
    *oy = (float)hash_halfword(h, 0) * HASH_UNIT_FROM_16BIT;
}

/*
 * worley_reseed — reroll wn->seed. Called at every reset (r key) so
 * different runs produce visibly different feature-point fields.
 */
static void worley_reseed(WorleyNoise *wn)
{
    wn->seed = (uint32_t)rand() ^ ((uint32_t)rand() << 16);
}

/*
 * metric_distance — distance from (ex, ey) under the requested norm.
 * Used inside the 3×3 query loop; isolated as a helper so each Tier-3
 * pattern selects a metric and the query loop stays metric-agnostic.
 *
 * SUPERELLIPSE and STAR are cubics / sqrt-based; both still cheap
 * (no transcendentals), so the per-query cost stays within ~2× of
 * pure Euclidean even for the most exotic metric.
 */
static inline float metric_distance(Metric metric, float ex, float ey)
{
    float ax = fabsf(ex);
    float ay = fabsf(ey);
    switch (metric) {
    case METRIC_MANHATTAN:    return ax + ay;
    case METRIC_CHEBYSHEV:    return ax > ay ? ax : ay;
    case METRIC_SUPERELLIPSE: return cbrtf(ax*ax*ax + ay*ay*ay);  /* L³ */
    case METRIC_STAR: {                                           /* L½ */
        float sx = sqrtf(ax);
        float sy = sqrtf(ay);
        float s  = sx + sy;
        return s * s;
    }
    case METRIC_STRETCHED:    return STRETCHED_X_WEIGHT * ax + ay;
    case METRIC_EUCLIDEAN:
    default:                  return sqrtf(ex*ex + ey*ey);
    }
}

/*
 * WorleyResult — what one worley_query call returns for a single
 * noise-space point.
 *
 * INTENT
 *   Bundle the three Worley F-distances plus the F1-owning tile's
 *   hash so pattern functions can compose ANY combination they
 *   need (F1, F2, F3, F2−F1, F1/F2, F1+F2, F1·F2, F3−F2, …) from
 *   ONE query call. Without this, each pattern would have to
 *   re-run the 3×3 lookup with a different "what to track" rule.
 *
 * CONTEXT
 *   Returned by worley_query() (§5) and the weighted variant.
 *   Consumed by all 30 pattern_* functions in §6. Tier 1-2
 *   patterns read only the F-fields; Tier 4 patterns also read
 *   cell_id to derive a per-cell visual property.
 *
 * MEMBER LOGIC
 *   f1 : Distance to the NEAREST feature point under the active
 *        metric. Always finite; in Euclidean, bounded by ~√2 inside
 *        the central tile.
 *   f2 : Distance to the SECOND-nearest. Always ≥ f1. The Voronoi
 *        edge of the F1-owning cell lies where f1 → f2 (so
 *        f2 − f1 → 0).
 *   f3 : Distance to the THIRD-nearest. Always ≥ f2. Required for
 *        Tier 1 PATTERN_F3 and Tier 2 PATTERN_F3_F2. If only
 *        f1/f2 are needed the loop still tracks f3 — the extra
 *        comparison is essentially free.
 *   cell_id : 32-bit hash of the F1-owning tile's integer coords.
 *        Deterministic per tile (same coords → same hash) and
 *        well-mixed (adjacent tiles have un-correlated hashes).
 *        Tier 4 patterns derive band, brightness, twinkle phase,
 *        etc. from this.
 *
 * Ref: Worley 1996, §3 — defines the F_n family of functions over
 *      the cellular distance set.
 */
typedef struct {
    float    f1;        /* distance to nearest feature point        */
    float    f2;        /* distance to second-nearest               */
    float    f3;        /* distance to third-nearest                */
    uint32_t cell_id;   /* hash of F1-owning tile (per-cell identity) */
} WorleyResult;

/* ───── worley_query — types and helpers ─────────────────────────────── *
 *
 * The Worley query decomposes naturally into 3 named stages. Each
 * stage gets a dedicated helper below; worley_query itself becomes a
 * pseudocode driver of "locate central tile → init cascade → scan
 * neighbourhood".
 *
 *   STAGE 1  LOCATE TILE — floor(fx, fy) → integer cell (xi, yi)
 *   STAGE 2  INIT CASCADE — F1=F2=F3=INF, cell_id=0
 *                                                  → worley_result_empty
 *   STAGE 3  SCAN 3×3 — for each neighbouring tile:
 *              · feature_point_at(tile, t)   — hashed + wobbled point
 *              · metric_distance(...)        — under the active norm
 *              · consider_distance(&r, d, h) — update F-cascade
 */

/*
 * WorleyFeaturePoint — the wobbling feature point that lives in one
 * lattice tile.
 *
 * INTENT
 *   Bundle the three values worley_feature_point_at returns: the
 *   Cartesian position (after wobble) and the hash that identifies
 *   the owning tile. Without this struct the inner loop would
 *   carry six separate locals (cx, cy, h, ox, oy, then px, py).
 *
 * CONTEXT
 *   Produced by worley_feature_point_at(). Consumed by the
 *   worley_query inner loop, which subtracts the query point from
 *   (px, py) to get displacement and passes hash to
 *   worley_consider_distance for cell_id tracking.
 *
 * MEMBER LOGIC
 *   px, py : Cartesian position of the feature point AFTER wobble.
 *            Lives in noise space (same coordinate frame as the
 *            query point); subtract query coords to get displacement.
 *   hash   : 32-bit hash of (tile_x, tile_y, seed). Tier 4 patterns
 *            key off this for per-cell visual properties.
 */
typedef struct {
    float    px, py;    /* Cartesian position (after wobble offset)     */
    uint32_t hash;      /* hash of the source tile (carries cell_id)    */
} WorleyFeaturePoint;

/* Sentinel-initialised empty result. F1 ≤ F2 ≤ F3 = INF means any
 * real distance will replace the cascade slot it lands in. */
static inline WorleyResult worley_result_empty(void)
{
    WorleyResult r = {
        .f1      = WORLEY_DISTANCE_INF,
        .f2      = WORLEY_DISTANCE_INF,
        .f3      = WORLEY_DISTANCE_INF,
        .cell_id = 0,
    };
    return r;
}

/*
 * worley_feature_point_at — locate the wobbling feature point in
 * tile (cx, cy) at simulation time t. Fuses three sub-operations:
 *
 *   (a) HASH the tile coords → 32-bit identity.
 *   (b) PLACE the base point in [0, 1)² from the hash's bit slices
 *       (hash_to_offset).
 *   (c) WOBBLE the position with a hash-derived phase so neighbouring
 *       tiles oscillate out of sync (otherwise everything would
 *       breathe in lockstep).
 *
 * The returned Cartesian position lets the caller subtract the query
 * point to get displacement; the hash is returned so cell_id-aware
 * patterns get it for free.
 */
static inline WorleyFeaturePoint worley_feature_point_at(
    const WorleyNoise *wn, int cx, int cy, float t)
{
    WorleyFeaturePoint fp;
    fp.hash = tile_hash(wn, cx, cy);

    /* Base position in [0, 1)² inside the tile. */
    float ox, oy;
    hash_to_offset(fp.hash, &ox, &oy);

    /* Per-point wobble phase ∈ ~[0, 2π) from independent hash bytes
     * so adjacent points oscillate at unrelated phases. */
    float phase_x = (float)hash_byte(fp.hash, 1) * HASH_PHASE_FROM_BYTE;
    float phase_y = (float)hash_byte(fp.hash, 3) * HASH_PHASE_FROM_BYTE;
    ox += WOBBLE_AMOUNT * sinf(t + phase_x);
    oy += WOBBLE_AMOUNT * cosf(t + phase_y);

    fp.px = (float)cx + ox;
    fp.py = (float)cy + oy;
    return fp;
}

/*
 * worley_consider_distance — update the F-cascade (F1 ≤ F2 ≤ F3)
 * with one new candidate distance. When d becomes the new minimum
 * we also remember the source hash as cell_id (only F1 has an
 * identity; F2 / F3 owners aren't tracked because no pattern needs
 * them).
 *
 * The 3-way comparison cascade is what makes worley_query's loop
 * THE SAME WORK regardless of how many F_n's the caller needs —
 * tracking F3 alongside F1/F2 costs one extra branch per tile.
 */
static inline void worley_consider_distance(WorleyResult *r,
                                            float d, uint32_t source_hash)
{
    if (d < r->f1) {
        r->f3      = r->f2;
        r->f2      = r->f1;
        r->f1      = d;
        r->cell_id = source_hash;
    } else if (d < r->f2) {
        r->f3 = r->f2;
        r->f2 = d;
    } else if (d < r->f3) {
        r->f3 = d;
    }
}

/*
 * worley_query — for the noise-space point (fx, fy), find F1, F2, F3
 * feature-point distances under the requested metric, plus the hash
 * of the F1-owning tile.
 *
 * Pseudocode shape:
 *
 *   tile          = floor(fx, fy)                  // STAGE 1
 *   r             = empty WorleyResult             // STAGE 2
 *   for each of the 9 neighbouring tiles:          // STAGE 3
 *     fp = feature_point_at(neighbour, t)
 *     d  = metric_distance(fp − query)
 *     consider_distance(r, d, fp.hash)
 *   return r
 *
 * With WOBBLE_AMOUNT < 0.4 every feature point stays roughly inside
 * its tile, so the 3×3 lookup is provably complete for L¹/L²/L∞.
 * Non-metric distances (STAR) lose this guarantee but the visual
 * effect remains well-defined.
 */
static WorleyResult worley_query(const WorleyNoise *wn,
                                 float fx, float fy,
                                 float t, Metric metric)
{
    /* STAGE 1 — LOCATE central tile containing the query point. */
    int xi = (int)floorf(fx);
    int yi = (int)floorf(fy);

    /* STAGE 2 — INIT the F-cascade to "no candidate yet". */
    WorleyResult r = worley_result_empty();

    /* STAGE 3 — SCAN the 3×3 neighbourhood; each tile's wobbling
     * feature point is a candidate for the cascade. */
    for (int dy = -WORLEY_NEIGHBOUR_RADIUS; dy <= WORLEY_NEIGHBOUR_RADIUS; dy++) {
        for (int dx = -WORLEY_NEIGHBOUR_RADIUS; dx <= WORLEY_NEIGHBOUR_RADIUS; dx++) {
            WorleyFeaturePoint fp = worley_feature_point_at(wn, xi + dx, yi + dy, t);
            float d = metric_distance(metric, fp.px - fx, fp.py - fy);
            worley_consider_distance(&r, d, fp.hash);
        }
    }
    return r;
}

/*
 * worley_query_weighted — multiplicatively-weighted Voronoi variant.
 *
 * Same 3-stage shape as worley_query, but each feature point has a
 * hash-derived "territory weight" w ∈ [WEIGHTED_W_MIN, MIN+RANGE];
 * the comparison distance is the raw Euclidean d divided by w, so
 * points with bigger w claim more area.
 *
 *   Weighted Voronoi conventionally uses L² distance — see
 *   Aurenhammer 1991 §3 for the geometric construction.
 *
 * Used only by Tier 4 PATTERN_WEIGHTED.
 */
static WorleyResult worley_query_weighted(const WorleyNoise *wn,
                                          float fx, float fy, float t)
{
    /* STAGE 1 — LOCATE central tile. */
    int xi = (int)floorf(fx);
    int yi = (int)floorf(fy);

    /* STAGE 2 — INIT the F-cascade. */
    WorleyResult r = worley_result_empty();

    /* STAGE 3 — SCAN the 3×3 neighbourhood with weight-divided d. */
    for (int dy = -WORLEY_NEIGHBOUR_RADIUS; dy <= WORLEY_NEIGHBOUR_RADIUS; dy++) {
        for (int dx = -WORLEY_NEIGHBOUR_RADIUS; dx <= WORLEY_NEIGHBOUR_RADIUS; dx++) {
            WorleyFeaturePoint fp = worley_feature_point_at(wn, xi + dx, yi + dy, t);

            /* Raw Euclidean distance (weighted Voronoi uses L² only). */
            float ex    = fp.px - fx;
            float ey    = fp.py - fy;
            float d_raw = sqrtf(ex * ex + ey * ey);

            /* Per-point territory weight ∈ [WEIGHTED_W_MIN, MIN + RANGE]. */
            float weight = WEIGHTED_W_MIN
                         + (float)hash_byte(fp.hash, 2) * HASH_UNIT_FROM_BYTE
                                                        * WEIGHTED_W_RANGE;
            worley_consider_distance(&r, d_raw / weight, fp.hash);
        }
    }
    return r;
}

/* ===================================================================== */
/* §6  patterns — 30 noise mappings in 6 tiers + dispatch table           */
/* ===================================================================== */

/*
 * Signature: all pattern functions take (wn, fx, fy, t, &glow, &band):
 *   wn       = const WorleyNoise *  — the seed-holding primitive,
 *              threaded through every pattern so the dataflow is
 *              explicit and the function can't silently mutate it.
 *   fx, fy   = noise-space coords (cell × NOISE_SCALE).
 *   t        = pattern.field_time — the wobble drift accumulator.
 *   out_glow = scalar ∈ [0, 1] — the cell's brightness.
 *   out_band = palette index ∈ {0..3}; Tier 4 patterns derive this
 *              from cell_id, every other tier derives it from the
 *              glow quartile (band_from_glow).
 */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* glow ∈ [0, 1] → palette band ∈ {0, 1, 2, 3}.
 * The almost-4 gain keeps glow=1.0 inside band 3 instead of
 * overflowing past it. */
static inline int band_from_glow(float g)
{
    return (int)(g * GLOW_TO_BAND_GAIN) & PALETTE_BAND_MASK;
}

/* cell_id hash → deterministic [0, 1] scalar.
 * Used by Tier-4 RANDOM_GLOW / TWINKLE / HALO patterns so every
 * Voronoi cell has its own "personality" without storing any
 * per-cell state. The XOR + re-hash with HASH_PERSONALITY_SALT
 * decorrelates consecutive cell_ids; the low half-word of the
 * remix gives ~16 bits of independent entropy per cell. */
static inline float cell_hash_to_unit(uint32_t cell_id)
{
    uint32_t remixed = hash32(cell_id ^ HASH_PERSONALITY_SALT);
    return (float)hash_halfword(remixed, 0) * HASH_UNIT_FROM_16BIT;
}

/* ---------- Tier 1 — F-FUNCTIONS: raw F_n outputs --------------------- */

/* F1 — distance to nearest feature point. Dark cell centres, bright
 * cell edges. The baseline cellular noise. */
static void pattern_f1(const WorleyNoise *wn, float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* F2 — distance to second-nearest. Smoother, larger blobs since the
 * second-nearest point's "territory" is intrinsically wider. */
static void pattern_f2(const WorleyNoise *wn, float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f2 - 0.3f) / 1.6f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* F3 — distance to third-nearest. Even smoother than F2; reads as a
 * faint, low-frequency cell structure. */
static void pattern_f3(const WorleyNoise *wn, float fx, float fy, float t,
                       float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f3 - 0.5f) / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* F2−F1 — sharp Voronoi edges. Near zero at cell boundaries (where
 * F1 ≈ F2); inverted so boundaries are BRIGHT. The classic. */
static void pattern_f2_f1(const WorleyNoise *wn, float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(clampf(r.f1 / 1.414f, 0.0f, 1.0f));
}

/* F1 / F2 — the ratio. → 0 at cell centres (F1 tiny), → 1 at cell
 * edges (F1 ≈ F2). Like F1 but with a non-linear gradient. */
static void pattern_f1_over_f2(const WorleyNoise *wn, float fx, float fy, float t,
                               float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / (r.f2 + 0.001f), 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 2 — F-COMBOS: other arithmetic on F values ----------- */

/* F1 + F2 — additive. Bright everywhere except cell centres; reads
 * as a "soft cushion" cellular field. */
static void pattern_f1_plus_f2(const WorleyNoise *wn, float fx, float fy, float t,
                               float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f1 + r.f2 - 0.3f) / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* F1 · F2 — multiplicative. Strongly suppressed near cell centres
 * (F1 → 0); reads as cells with bright outlines and very dark cores. */
static void pattern_f1_times_f2(const WorleyNoise *wn, float fx, float fy, float t,
                                float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 * r.f2 * 1.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* F3 − F2 — third-vs-second edges. Thinner and more "branching"
 * than F2−F1; highlights the dual edges of the Voronoi graph. */
static void pattern_f3_f2(const WorleyNoise *wn, float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf((r.f3 - r.f2) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* 1 − (F2 − F1) → not inverted. Edges DARK, cells BRIGHT — the
 * complement of the standard F2_F1 pattern. */
static void pattern_f2_f1_inv(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* 1 − F1 — bright centres. The complement of F1. Each cell has a
 * bright core that fades toward the edges. */
static void pattern_f1_inv(const WorleyNoise *wn, float fx, float fy, float t,
                           float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 3 — METRICS: F1 with different distance norms -------- */

/* MANHATTAN — F1 with L¹: cells become diamonds (45°-aligned). */
static void pattern_manhattan(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_MANHATTAN);
    float g = clampf(r.f1 / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* CHEBYSHEV — F1 with L∞ (= max(|Δx|, |Δy|)): cells become axis-
 * aligned squares. Max distance inside any cell is 1.0. */
static void pattern_chebyshev(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_CHEBYSHEV);
    float g = clampf(r.f1, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* SUPERELLIPSE — F1 with L³: cells become rounded squares
 * (super-ellipses). The smooth interpolation between L² (circle)
 * and L∞ (square). */
static void pattern_superellipse(const WorleyNoise *wn, float fx, float fy, float t,
                                 float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_SUPERELLIPSE);
    float g = clampf(r.f1 / 1.4f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* STAR — F1 with L½ ("(√|Δx|+√|Δy|)²"): cells become concave
 * 4-pointed stars. Not strictly a metric (triangle inequality
 * fails) but produces a striking pattern. */
static void pattern_star(const WorleyNoise *wn, float fx, float fy, float t,
                         float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_STAR);
    float g = clampf(r.f1 / 1.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* STRETCHED — F1 with anisotropic Manhattan (2|Δx| + |Δy|): cells
 * stretched along the y-axis. Demonstrates how weighted norms
 * deform the cell shape without changing point positions. */
static void pattern_stretched(const WorleyNoise *wn, float fx, float fy, float t,
                              float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_STRETCHED);
    float g = clampf(r.f1 / 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 4 — IDENTITY: cell_id hash drives a visual property -- */

/* CELL_ID — solid colour per Voronoi cell. The classic Voronoi
 * region map; band index = cell_id mod 4. Glow has a slight
 * inward gradient so each cell has a focal point. */
static void pattern_cell_id(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    *out_glow = 1.0f - clampf(r.f1 * 0.4f, 0.0f, 0.4f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* WEIGHTED — multiplicatively-weighted Voronoi. Each feature point
 * has a hash-derived "radius" wᵢ ∈ [0.5, 1.5]; the cell boundary
 * is where d₁/w₁ = d₂/w₂. Points with bigger wᵢ claim more
 * territory — irregular cell sizes instead of uniform. */
static void pattern_weighted(const WorleyNoise *wn, float fx, float fy, float t,
                             float *out_glow, int *out_band)
{
    WorleyResult r = worley_query_weighted(wn, fx, fy, t);
    *out_glow = 1.0f - clampf(r.f1 * 0.5f, 0.0f, 0.5f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* TWINKLE — per-cell brightness modulated by sin(t · rate(cell_id)).
 * Each cell flickers at its own rate, derived from its hash. */
static void pattern_twinkle(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float rate    = 0.5f + cell_hash_to_unit(r.cell_id) * 3.0f;
    float pulse   = 0.5f + 0.5f * sinf(t * rate);
    float g_base  = 1.0f - clampf(r.f1 * 0.4f, 0.0f, 0.4f);
    *out_glow = clampf(g_base * pulse, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* RANDOM_GLOW — per-cell static brightness from cell_id. Each cell
 * gets a fixed pseudo-random glow ∈ [0, 1]; the visual is a
 * Voronoi mosaic where every cell has its own tone. */
static void pattern_random_glow(const WorleyNoise *wn, float fx, float fy, float t,
                                float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    *out_glow = cell_hash_to_unit(r.cell_id);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* CRACKLE — F2−F1 edges combined with cell-id-derived darkening.
 * Sharp Voronoi edges, but each cell's interior brightness varies
 * by hash. Reads like dried mud / lava cracks. */
static void pattern_crackle(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    WorleyResult r       = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float        edges   = 1.0f - clampf((r.f2 - r.f1) * 3.0f, 0.0f, 1.0f);
    float        cell_lo = 0.3f + 0.7f * cell_hash_to_unit(r.cell_id);
    *out_glow = clampf(edges * cell_lo, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* ---------- Tier 5 — MULTI-SCALE: Worley fBm stacks ------------------- *
 *
 * Three octaves of F1 sampled at frequencies 1·x, 2·x, 4·x with
 * amplitudes 1, ½, ¼. The amplitude-normaliser is the sum of those
 * amplitudes (1.75) so the output stays in roughly [0, 1].
 */
#define WORLEY_FBM_OCTAVES   3
#define WORLEY_FBM_AMP_SUM   1.75f   /* 1 + 0.5 + 0.25 */

/* WORLEY_FBM — Σ aᵢ · F1(2ⁱ·x). Classic cellular fBm; resembles
 * stacked cellular noise — looks like clusters of cells of mixed
 * sizes. */
static void pattern_worley_fbm(const WorleyNoise *wn, float fx, float fy, float t,
                               float *out_glow, int *out_band)
{
    float total = 0.0f;
    float amp   = 1.0f;
    float freq  = 1.0f;
    for (int o = 0; o < WORLEY_FBM_OCTAVES; o++) {
        WorleyResult r = worley_query(wn, fx * freq, fy * freq, t, METRIC_EUCLIDEAN);
        total += amp * r.f1 / 1.414f;
        amp   *= 0.5f;
        freq  *= 2.0f;
    }
    float g = clampf(total / WORLEY_FBM_AMP_SUM, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* WORLEY_TURBULENCE — Σ aᵢ · (1 − (F2−F1))(2ⁱ·x). Multi-scale
 * Voronoi edges — fine edges overlaid on coarse edges. */
static void pattern_worley_turbulence(const WorleyNoise *wn, float fx, float fy, float t,
                                      float *out_glow, int *out_band)
{
    float total = 0.0f;
    float amp   = 1.0f;
    float freq  = 1.0f;
    for (int o = 0; o < WORLEY_FBM_OCTAVES; o++) {
        WorleyResult r = worley_query(wn, fx * freq, fy * freq, t, METRIC_EUCLIDEAN);
        float edges    = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
        total += amp * edges;
        amp   *= 0.5f;
        freq  *= 2.0f;
    }
    float g = clampf(total / WORLEY_FBM_AMP_SUM, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* WORLEY_RIDGED — Σ aᵢ · (1 − F1)(2ⁱ·x). Inverted Worley fBm;
 * sharp bright cell centres at every scale. Resembles cloud
 * surface from above. */
static void pattern_worley_ridged(const WorleyNoise *wn, float fx, float fy, float t,
                                  float *out_glow, int *out_band)
{
    float total = 0.0f;
    float amp   = 1.0f;
    float freq  = 1.0f;
    for (int o = 0; o < WORLEY_FBM_OCTAVES; o++) {
        WorleyResult r = worley_query(wn, fx * freq, fy * freq, t, METRIC_EUCLIDEAN);
        float ridged   = 1.0f - clampf(r.f1 / 1.414f, 0.0f, 1.0f);
        total += amp * ridged;
        amp   *= 0.5f;
        freq  *= 2.0f;
    }
    float g = clampf(total / WORLEY_FBM_AMP_SUM, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* NESTED — F1(x) + ½ · F1(2x), averaged. Large cells with smaller
 * cells overlaid; gives a 2-tier hierarchy without the full fBm
 * stack cost. */
static void pattern_nested(const WorleyNoise *wn, float fx, float fy, float t,
                           float *out_glow, int *out_band)
{
    WorleyResult rc = worley_query(wn, fx,        fy,        t, METRIC_EUCLIDEAN);
    WorleyResult rf = worley_query(wn, fx * 2.0f, fy * 2.0f, t, METRIC_EUCLIDEAN);
    float g = clampf((rc.f1 + rf.f1 * 0.5f) / 1.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* CHECKER — alternate fine / coarse scale per integer cell parity.
 * Adjacent cells of the noise lattice display Worley at different
 * frequencies; the result is a 2-tier patchwork. */
static void pattern_checker(const WorleyNoise *wn, float fx, float fy, float t,
                            float *out_glow, int *out_band)
{
    int parity = ((int)floorf(fx) + (int)floorf(fy)) & 1;
    float scale = parity ? 2.0f : 0.5f;
    WorleyResult r = worley_query(wn, fx * scale, fy * scale, t, METRIC_EUCLIDEAN);
    float g = clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* ---------- Tier 6 — HYBRIDS: Worley-on-Worley compositions ----------- */

/* DOMAIN_WARP — F1 sampled at coords perturbed by another Worley
 * field. Cells appear "smudged" or "flowing" — the same kind of
 * trick IQ uses on Perlin, applied here to cellular noise. */
static void pattern_domain_warp(const WorleyNoise *wn, float fx, float fy, float t,
                                float *out_glow, int *out_band)
{
    WorleyResult warp_x = worley_query(wn, fx * 0.5f,         fy * 0.5f,         t, METRIC_EUCLIDEAN);
    WorleyResult warp_y = worley_query(wn, fx * 0.5f + 5.2f,  fy * 0.5f + 1.3f,  t, METRIC_EUCLIDEAN);
    float wx = (warp_x.f1 - 0.5f) * 1.5f;
    float wy = (warp_y.f1 - 0.5f) * 1.5f;
    WorleyResult r = worley_query(wn, fx + wx, fy + wy, t, METRIC_EUCLIDEAN);
    float g = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* METRIC_BLEND — average of F1 under Euclidean and Manhattan. The
 * cell shapes interpolate between rounded and diamond — visually a
 * hybrid you can't get from any single metric. */
static void pattern_metric_blend(const WorleyNoise *wn, float fx, float fy, float t,
                                 float *out_glow, int *out_band)
{
    WorleyResult re = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    WorleyResult rm = worley_query(wn, fx, fy, t, METRIC_MANHATTAN);
    float g = clampf(0.5f * re.f1 / 1.414f + 0.5f * rm.f1 / 2.0f, 0.0f, 1.0f);
    *out_glow = g;
    *out_band = band_from_glow(g);
}

/* HALO — each cell has a bright "core" with a halo decaying outward.
 * The halo's peak brightness varies per cell via cell_id; reads
 * like glowing fireflies hovering in their cells. */
static void pattern_halo(const WorleyNoise *wn, float fx, float fy, float t,
                         float *out_glow, int *out_band)
{
    WorleyResult r        = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float        peak     = 0.5f + 0.5f * cell_hash_to_unit(r.cell_id);
    float        falloff  = expf(-r.f1 * 4.0f);          /* tight halo around centre */
    *out_glow = clampf(peak * falloff, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* ENERGY — pow(1 − F1, 8). Extreme power on (1 − F1) produces
 * razor-sharp bright pinpoints exactly at the cell centres,
 * everything else dark. Reads as plasma sparks. */
static void pattern_energy(const WorleyNoise *wn, float fx, float fy, float t,
                           float *out_glow, int *out_band)
{
    WorleyResult r = worley_query(wn, fx, fy, t, METRIC_EUCLIDEAN);
    float v = 1.0f - clampf(r.f1 / 1.414f, 0.0f, 1.0f);
    float g = v * v;      /* ²  */
    g     *= g;            /* ⁴  */
    g     *= g;            /* ⁸  */
    *out_glow = clampf(g * 1.5f, 0.0f, 1.0f);
    *out_band = band_from_glow(*out_glow);
}

/* CHAOS — domain-warped turbulence. Combines two of the heavier
 * techniques: warp the query coords by a coarse F1 field, then
 * read fine F2−F1 edges. Looks like turbulent fluid cells. */
static void pattern_chaos(const WorleyNoise *wn, float fx, float fy, float t,
                          float *out_glow, int *out_band)
{
    WorleyResult warp = worley_query(wn, fx * 0.5f, fy * 0.5f, t, METRIC_EUCLIDEAN);
    float        wx   = (warp.f1 - 0.5f) * 2.0f;
    float        wy   = (warp.cell_id & 1u) ? wx : -wx;    /* asymmetric warp */
    WorleyResult r    = worley_query(wn, fx + wx, fy + wy, t, METRIC_EUCLIDEAN);
    float        edges = 1.0f - clampf((r.f2 - r.f1) * 2.5f, 0.0f, 1.0f);
    *out_glow = clampf(edges, 0.0f, 1.0f);
    *out_band = (int)(r.cell_id & PALETTE_BAND_MASK);
}

/* ---------- Dispatch table -------------------------------------------- */

/*
 * NoisePatternFn — one signature shared by all 30 pattern samplers.
 * Spelt out here so the dispatch table is type-safe and a reader can
 * grep ONE place to see what every pattern function MUST accept.
 */
typedef void (*NoisePatternFn)(const WorleyNoise *wn,
                               float fx, float fy, float t,
                               float *out_glow, int *out_band);

/*
 * NoisePattern — one row of the dispatch table: (display-name,
 * tier-label, sampler function pointer).
 *
 * INTENT
 *   Replace what would otherwise be a 30-case switch with a table
 *   lookup keyed by the Pattern enum. Adding a new pattern is then
 *   THREE coordinated edits (enum value, table row, function body)
 *   and the compiler enforces alignment via the fixed-size [N_PATTERNS]
 *   initialiser — a missing or mis-ordered row is a compile error.
 *
 * CONTEXT
 *   The whole `noise_patterns[]` array (below this typedef) is the
 *   source of truth for pattern → (name, tier, fn). The Pattern enum
 *   in §1 provides readable indices; pattern_name() and pattern_tier()
 *   in this section are the only outside accessors. scene_evaluate_field
 *   in §7 fetches `noise_patterns[active].sample` once per tick.
 *
 * MEMBER LOGIC
 *   name   : Display name, FIXED 10-char left-padded. The constant
 *            width is deliberate — HUD column alignment must stay
 *            stable as the user cycles patterns with n/p (otherwise
 *            the HUD jitters as each name changes length).
 *   tier   : 7-char "N-LABEL " form (e.g. "1-FN   ", "5-MULTI",
 *            "6-HYB  "). Encodes both the complexity tier (1-6) and
 *            the technique class (F-FUNCTIONS / F-COMBOS / METRICS /
 *            IDENTITY / MULTI-SCALE / HYBRIDS). Same fixed-width
 *            rationale as `name`.
 *   sample : Function pointer to one of the 30 pattern_* functions.
 *            Indirection cost is one indirect call per cell, dwarfed
 *            by the cost of worley_query inside the function (9
 *            hashes + 9 distance evaluations per query, and Tier
 *            5-6 patterns call worley_query 2-3× per cell).
 */
typedef struct {
    const char     *name;      /* HUD-padded display name, 10 chars */
    const char     *tier;      /* HUD-padded tier label, 7 chars    */
    NoisePatternFn  sample;    /* the pattern's per-cell sampler    */
} NoisePattern;

static const NoisePattern noise_patterns[N_PATTERNS] = {
    /* Tier 1 — F-FUNCTIONS */
    [PATTERN_F1]                = { "F1        ", "1-FN   ", pattern_f1                },
    [PATTERN_F2]                = { "F2        ", "1-FN   ", pattern_f2                },
    [PATTERN_F3]                = { "F3        ", "1-FN   ", pattern_f3                },
    [PATTERN_F2_F1]             = { "F2-F1     ", "1-FN   ", pattern_f2_f1             },
    [PATTERN_F1_OVER_F2]        = { "F1/F2     ", "1-FN   ", pattern_f1_over_f2        },
    /* Tier 2 — F-COMBOS */
    [PATTERN_F1_PLUS_F2]        = { "F1+F2     ", "2-COMBO", pattern_f1_plus_f2        },
    [PATTERN_F1_TIMES_F2]       = { "F1*F2     ", "2-COMBO", pattern_f1_times_f2       },
    [PATTERN_F3_F2]             = { "F3-F2     ", "2-COMBO", pattern_f3_f2             },
    [PATTERN_F2_F1_INV]         = { "F2-F1 INV ", "2-COMBO", pattern_f2_f1_inv         },
    [PATTERN_F1_INV]            = { "F1 INVERT ", "2-COMBO", pattern_f1_inv            },
    /* Tier 3 — METRICS */
    [PATTERN_MANHATTAN]         = { "MANHATTAN ", "3-METR ", pattern_manhattan         },
    [PATTERN_CHEBYSHEV]         = { "CHEBYSHEV ", "3-METR ", pattern_chebyshev         },
    [PATTERN_SUPERELLIPSE]      = { "SUPRELLIPS", "3-METR ", pattern_superellipse      },
    [PATTERN_STAR]              = { "STAR      ", "3-METR ", pattern_star              },
    [PATTERN_STRETCHED]         = { "STRETCHED ", "3-METR ", pattern_stretched         },
    /* Tier 4 — IDENTITY */
    [PATTERN_CELL_ID]           = { "CELL_ID   ", "4-IDENT", pattern_cell_id           },
    [PATTERN_WEIGHTED]          = { "WEIGHTED  ", "4-IDENT", pattern_weighted          },
    [PATTERN_TWINKLE]           = { "TWINKLE   ", "4-IDENT", pattern_twinkle           },
    [PATTERN_RANDOM_GLOW]       = { "RND_GLOW  ", "4-IDENT", pattern_random_glow       },
    [PATTERN_CRACKLE]           = { "CRACKLE   ", "4-IDENT", pattern_crackle           },
    /* Tier 5 — MULTI-SCALE */
    [PATTERN_WORLEY_FBM]        = { "WORLEY_FBM", "5-MULTI", pattern_worley_fbm        },
    [PATTERN_WORLEY_TURBULENCE] = { "TURBULENCE", "5-MULTI", pattern_worley_turbulence },
    [PATTERN_WORLEY_RIDGED]     = { "RIDGED    ", "5-MULTI", pattern_worley_ridged     },
    [PATTERN_NESTED]            = { "NESTED    ", "5-MULTI", pattern_nested            },
    [PATTERN_CHECKER]           = { "CHECKER   ", "5-MULTI", pattern_checker           },
    /* Tier 6 — HYBRIDS */
    [PATTERN_DOMAIN_WARP]       = { "DOM_WARP  ", "6-HYB  ", pattern_domain_warp       },
    [PATTERN_METRIC_BLEND]      = { "MTRC_BLEND", "6-HYB  ", pattern_metric_blend      },
    [PATTERN_HALO]              = { "HALO      ", "6-HYB  ", pattern_halo              },
    [PATTERN_ENERGY]            = { "ENERGY    ", "6-HYB  ", pattern_energy            },
    [PATTERN_CHAOS]             = { "CHAOS     ", "6-HYB  ", pattern_chaos             },
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
 * GlowField — the per-cell output grid that the pipeline writes into
 * and the renderer reads from.
 *
 * INTENT
 *   Decouple "compute the cellular noise field" (scene_evaluate_field,
 *   runs once per sim tick) from "paint the cellular noise field"
 *   (scene_draw, runs once per render frame). The GlowField is the
 *   snapshot the renderer reads in between ticks.
 *
 * CONTEXT
 *   Lives on Scene (§7). Written by scene_evaluate_field() — one
 *   pattern-sampler call per cell, results written here. Read by
 *   scene_draw() — uses glow + band via the GlyphRamp to pick a
 *   glyph + colour pair. Index with glow_field_idx(gf, x, y) =
 *   y · w + x (row-major).
 *
 * MEMORY
 *   No allocation post-init. Both buffers are sized at CELLS_MAX in
 *   BSS storage so the hot path never touches malloc/free.
 *
 * MEMBER LOGIC
 *   w, h    : Grid dimensions in cells. Set by app_pick_map_size()
 *             at init and on every resize. Bounded by [16..MAP_W_MAX]
 *             × [8..MAP_H_MAX].
 *   count   : w · h, cached at reset so the clearing loop and the
 *             dispatcher don't recompute it each tick.
 *   glow[]  : Per-cell scalar ∈ [0, 1]. The pattern sampler in §6
 *             writes here.
 *   band[]  : Per-cell palette band ∈ {0, 1, 2, 3}. Tier-4 patterns
 *             derive it from cell_id; every other tier from glow
 *             quartile.
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
 * GlyphRamp — the density → (glyph, attr) mapping.
 *
 * INTENT
 *   Capture "how density becomes a character" as a NAMED RULE
 *   rather than an inline if/else in the renderer. Three benefits:
 *   (1) the threshold values are visible as data; (2) a future
 *   feature can swap ramps per theme; (3) the renderer reads
 *   `glyph_ramp_pick(...)` which scans cleaner than `if (glow > X)`.
 *
 * THE RAMP
 *      glow > thresh_high  →  glyph_high  bold     (dense core)
 *      glow > thresh_mid   →  glyph_mid   bold     (mid density)
 *      glow > thresh_low   →  glyph_low   normal   (faint trace)
 *      else                →  not drawn            (transparent)
 *
 * INVARIANT
 *   thresh_high > thresh_mid > thresh_low > 0
 *   Defaults from GLYPH_HIGH_THRESH / GLYPH_MID_THRESH / GLOW_THRESHOLD.
 *
 * GLYPH CHOICE
 *   '#', '*', '.' is a coarse 3-tier subset of Paul Bourke's
 *   canonical 10-char luminance ramp "@%#*+=-:. ". Three tiers
 *   chosen for perceptual contrast at terminal resolution.
 *
 * Ref: Bourke, "Character representation of grey scale images".
 */
typedef struct {
    float thresh_high;   /* > this → glyph_high  (default 0.65) */
    float thresh_mid;    /* > this → glyph_mid   (default 0.30) */
    float thresh_low;    /* > this → glyph_low   (default 0.05) */
    char  glyph_high;    /* dense core   glyph                  */
    char  glyph_mid;     /* mid-density  glyph                  */
    char  glyph_low;     /* faint-trace  glyph                  */
} GlyphRamp;

/*
 * GlyphChoice — return value from glyph_ramp_pick(). Small POD struct
 * so the call site reads as a pure function:
 *
 *   GlyphChoice gc = glyph_ramp_pick(ramp, glow);
 *   if (gc.visible) draw_cell(gc.glyph, palette[band], gc.attr);
 *
 * Caller composes (glyph, attr) with the palette band index from
 * elsewhere — GlyphRamp deliberately doesn't know about colour.
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
 *   what speed. They change together, so giving them a struct
 *   gives the animation a name.
 *
 * MEMBER LOGIC
 *   current    : Active pattern enum, indexes noise_patterns[].
 *                Defaults to PATTERN_F2_F1 (the sharpest Voronoi-
 *                edge pattern — best boot frame). Cycled by n/p.
 *   field_time : Wobble-drift accumulator (noise-space units).
 *                Advanced each tick by FIELD_DRIFT × drift_mult × dt.
 *                Feeds the per-tile wobble phase inside worley_query.
 *                Monotonically increasing; never reset except on r.
 *   drift_mult : User-controlled drift-rate multiplier. Power of 2
 *                in [DRIFT_MULT_MIN, DRIFT_MULT_MAX] = [1, 16].
 */
typedef struct {
    Pattern current;          /* active pattern enum                  */
    float   field_time;       /* drift accumulator (noise-space units)*/
    int     drift_mult;       /* powers of 2 ∈ [MIN, MAX]             */
} PatternState;

static void pattern_state_init(PatternState *ps)
{
    ps->current    = PATTERN_F2_F1;   /* boundary-line pattern by default */
    ps->field_time = 0.0f;
    ps->drift_mult = DRIFT_MULT_DEF;
}

/*
 * PaletteState — the active theme index.
 *
 * INTENT
 *   Currently a one-int wrapper, but kept as a named struct on
 *   Scene so the responsibility "which colour scheme is showing"
 *   has a stable home. Future themed-glyph support or theme
 *   crossfades would extend this struct without disturbing Scene.
 *
 * MEMBER LOGIC
 *   current : Index into themes[] in §1. Always in [0, N_THEMES);
 *             wraparound is handled in the key handler via mod.
 */
typedef struct {
    int current;              /* index into themes[] ∈ [0, N_THEMES) */
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
 *   noise    — the WorleyNoise seed                  (the ALGORITHM)
 *               ↓ (sampled by pattern functions)
 *   pattern  — active pattern + drift state         (the ANIMATION)
 *               ↓ (drives scene_evaluate_field)
 *   field    — per-cell glow + band buffers         (the DATA)
 *               ↓ (read by scene_draw)
 *   ramp     — density → glyph mapping              (the RENDER RULE)
 *               + palette colour pairs (via ncurses)
 *               ↓
 *   palette  — active theme index                   (the COLOUR CHOICE)
 *               + paused flag                       (the CONTROL)
 *
 *   Reads in dataflow order: noise → field (via pattern) →
 *   ramp + palette (via renderer).
 */
typedef struct {
    WorleyNoise   noise;     /* §5 — seed-holding primitive (mutable)       */
    GlowField     field;     /* per-cell output grid (largest member)       */
    GlyphRamp     ramp;      /* density → glyph rule (read-mostly)          */
    PatternState  pattern;   /* active pattern + drift accumulator + speed  */
    PaletteState  palette;   /* active theme index                          */
    bool          paused;    /* if true, scene_tick early-returns           */
} Scene;

/* ---------- Per-cell mapping helper used by scene_evaluate_field ----- */

/* cell index → noise-space coordinate (cells × NOISE_SCALE). Used as
 * the (x, y) argument to every pattern sampler. */
static inline float cell_to_noise_coord(int cell)
{
    return (float)cell * NOISE_SCALE;
}

/*
 * scene_evaluate_field — drive the whole pipeline for one sim tick.
 *
 *   FOR every cell (x, y):
 *     fx, fy = noise-space coords          // cell_to_noise_coord
 *     (glow, band) = active_pattern(noise, fx, fy, drift)
 *     write (glow, band) → GlowField[x, y]
 *
 * Hot loop — one pattern call per cell × ~11K cells × 60 Hz. The
 * pattern functions are tiny but worley_query itself examines 9
 * tiles per call; the heaviest patterns (Tier 5 fBm, Tier 6 hybrids)
 * call worley_query 2-3× per cell.
 */
static void scene_evaluate_field(Scene *s)
{
    /* STEP 1 — RESOLVE the active pattern's sampler. */
    Pattern active = s->pattern.current;
    if ((unsigned)active >= (unsigned)N_PATTERNS) return;
    NoisePatternFn      sample_pattern = noise_patterns[active].sample;
    const WorleyNoise  *noise          = &s->noise;
    GlowField          *field          = &s->field;
    float               drift          = s->pattern.field_time;

    /* STEP 2 — SAMPLE PER CELL: pattern writes (glow, band) directly. */
    for (int y = 0; y < field->h; y++) {
        float fy = cell_to_noise_coord(y);
        for (int x = 0; x < field->w; x++) {
            float fx   = cell_to_noise_coord(x);
            float glow = 0.0f;
            int   band = 0;
            sample_pattern(noise, fx, fy, drift, &glow, &band);

            int idx          = glow_field_idx(field, x, y);
            field->glow[idx] = glow;
            field->band[idx] = (uint8_t)(band & PALETTE_BAND_MASK);
        }
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    glow_field_reset(&s->field, mw, mh);
    s->pattern.field_time = 0.0f;
    worley_reseed(&s->noise);
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
    /* Drive the per-tile wobble (drift accumulator advances drift_mult× rate). */
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
 *   also matches the pattern in sibling demos.
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
 *   cols : terminal width  in cells. ≥ 16 enforced by
 *          app_pick_map_size (smaller and the HUD won't fit).
 *   rows : terminal height in cells. ≥ 8 enforced by
 *          app_pick_map_size; HUD reserves 3 rows top/bottom.
 *   Ordering matches the ncurses getmaxyx(stdscr, rows, cols)
 *   convention — rows first because curses originally targeted
 *   line-printers.
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
 *   Y grows DOWN (toward bottom of terminal), X grows RIGHT.
 *
 * INVARIANTS
 *   origin_x ≥ 0
 *   origin_y ≥ HUD_TOP_ROWS         (kept clear for the title bar)
 *   The bottom HUD_BOTTOM_ROWS rows are kept clear by sizing the
 *   field, not by clamping here — so origin_y + field_h may run up
 *   to (rows − HUD_BOTTOM_ROWS).
 *
 *   Negative results from the centering arithmetic (over-large
 *   field on a small viewport) are clamped to the viewport edge —
 *   the field is cropped, not wrapped. The "cell off-screen?"
 *   check in the drawer's inner loop handles the rest.
 *
 * MEMBER LOGIC
 *   origin_x : screen column (cells) of the field's leftmost cell.
 *              0 if the field is wider than the viewport.
 *   origin_y : screen row (cells) of the field's topmost cell.
 *              HUD_TOP_ROWS at minimum (never overlaps title bar).
 */
typedef struct {
    int origin_x;   /* screen column where field's left edge starts */
    int origin_y;   /* screen row    where field's top edge starts  */
} GridPlacement;

/* Centre the GlowField inside the terminal viewport, respecting the
 * HUD's reserved top + bottom rows. Negative offsets are clamped so
 * an over-large field is cropped, not wrapped. */
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
 * scene_draw — paint the GlowField using the Scene's GlyphRamp and
 * the currently-bound colour palette.
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

            int color_pair = PAIR_BAND_BASE + (field->band[cell_idx] & PALETTE_BAND_MASK);
            draw_glyph_at(screen_y, screen_x,
                          pick.glyph, color_pair, pick.attr);
        }
    }
}

/* ---------- HUD layout widths ----------------------------------------- *
 *
 * Per-segment widths for the status row 1 chain. Each hud_draw_*
 * helper consumes (and returns) the cursor x; these widths are how
 * far each segment moves it.
 */
#define HUD_W_PATTERN_FIELD   21   /* " pattern:%-10s "          */
#define HUD_W_TIER_FIELD      15   /* " tier:%-7s "              */
#define HUD_W_THEME_FIELD     17   /* " theme:%-8s "             */
#define HUD_W_PALETTE_LABEL    9   /* " palette:"                */

#define HUD_TITLE_ROW          0
#define HUD_STATUS_ROW         1
#define HUD_TITLE_TEXT         " WORLEY CELLULAR NOISE "
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

/* Row 1 segment — "palette:" label + N_PALETTE_BANDS coloured
 * swatches so the user can see each density-tier's colour at a glance
 * in the active theme. */
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

/* Row 1 tail — algorithm parameter readout: noise scale, wobble
 * amplitude, grid dimensions. */
static void hud_draw_stats_field(int row, int x, const GlowField *gf)
{
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  scale:%.2f  wobble:%.2f  map:%dx%d ",
             NOISE_SCALE, WOBBLE_AMOUNT, gf->w, gf->h);
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
 *   main() does: zero-init g_app → screen_init → app_pick_map_size
 *   → scene_init → loop → screen_free. No teardown beyond endwin()
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
 *   running     : Cleared to 0 by SIGINT / SIGTERM handlers (and
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
 * arithmetic. Direction parameters use ±1 throughout for next/prev
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
 *   (frame_clock_advance, fps_meter_observe, …) instead of a
 *   collection of bare arithmetic against scattered locals.
 *
 * THE TWO SUB-ACCUMULATORS
 *   FrameClock combines two independent integrator patterns that
 *   happen to share `frame_dt_ns` as their input:
 *
 *     1) FIXED-TIMESTEP SIM DRIVER  (sim_accum_ns)
 *        Each frame's wall-clock dt is poured into the accumulator;
 *        drained one TICK_NS chunk at a time into scene_tick() calls.
 *        The "Glenn Fiedler / 'Fix Your Timestep!'" pattern.
 *
 *     2) FPS METER                  (fps_accum_ns + fps_frame_count)
 *        Sum frame dts and count frames; every FPS_UPDATE_MS of wall
 *        clock, divide frames/seconds to refresh `fps_display`. The
 *        HUD reads `fps_display` only.
 *
 * Refs:
 *   • Glenn Fiedler, "Fix Your Timestep!" (gafferongames.com)
 *   • POSIX clock_gettime(CLOCK_MONOTONIC) — the wall-clock source
 *     in §2 clock_ns(); strictly increasing, immune to NTP jumps.
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

/* Sleep so the render loop spins at ~60 Hz. Compute how much wall-
 * clock has been consumed this iteration (tick work + frame-to-frame
 * gap) and sleep the remainder of the frame budget. */
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
