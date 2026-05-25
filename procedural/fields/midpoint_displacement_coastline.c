/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * midpoint_displacement_coastline.c
 *   — 1-D midpoint-displacement fractals, 30 silhouette patterns.
 *
 * DEMO: A single endpoint pair becomes a jagged fractal line through
 *       repeated midpoint-displacement subdivision: each segment's
 *       midpoint is set to the average of its endpoints plus a random
 *       jitter, with the jitter halving every level. The resulting
 *       polyline looks naturally rough — like a coastline, mountain
 *       silhouette, or lightning bolt. THIRTY pattern presets span
 *       simple → complex, organised by line count and visual style:
 *
 *         Tier 1 — single line   : COASTLINE SKYLINE HORIZON DUNES CITY
 *         Tier 2 — paired lines  : VALLEY CAVE CLIFF PLATEAU REEF
 *         Tier 3 — small stack   : MOUNTAINS HILLS ALPS RIDGES TERRACES
 *         Tier 4 — multi-stack   : WAVES STRATA FOREST RAPIDS SEDIMENT
 *         Tier 5 — exotic        : ATOLLS CRYSTAL SAWTOOTH NEBULA STORM
 *                                  AURORA CEILING ISLANDS PETRA CHAOS
 *
 *       Each line slowly morphs between two random shapes over ~10s,
 *       so the silhouette evolves continuously rather than freezing.
 *       The state bar shows [N/30] so you can see where you are in
 *       the cycle. Cycle patterns with n/p, themes with t/T, glyph
 *       sets with g/G.
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
 *   §1 config   — grid, patterns, palette, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes
 *   §5 md       — midpoint displacement core
 *   §6 patterns — 30 line configurations + render-style dispatcher
 *   §7 scene    — Scene composing Grid, FractalField, SimState,
 *                 Controls; per-frame tick / morph-keyframe pipeline
 *   §8 screen   — ASCII render: silhouettes + fills + HUD drawers
 *   §9 app      — signals, resize, named main-loop helpers
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
 * Data-structure : Two float keyframe buffers per line (keyframe_a,
 *                  keyframe_b) plus the per-frame interpolated live[]
 *                  buffer used for rendering. Up to 8 lines per pattern
 *                  (CHAOS / STRATA need the most). 257-point lines for
 *                  full fractal resolution; rendering interpolates to
 *                  fit any terminal width.
 *
 *                  Each line carries its own LineSpec — fractal params
 *                  (y_centre, amp, roughness, endpoint asymmetry) plus
 *                  rendering hints (fill direction, palette band,
 *                  outline band, glyph density). Patterns are nothing
 *                  more than the LineSpec list returned by a small
 *                  per-pattern placer function; the renderer is
 *                  pattern-agnostic and just walks the list.
 *
 * Rendering      : ASCII only. Three render styles selected per line:
 *                    FILL_BELOW  — paint from line down to map bottom
 *                                 (coastline water, mountain mass)
 *                    FILL_ABOVE  — paint from map top down to line
 *                                 (ceiling, sky-fill, cave roof)
 *                    THIN_LINE   — paint only the line (waves, ridges)
 *                  Any of the fill styles can carry an optional thin
 *                  outline in a contrasting palette band — that's how
 *                  COASTLINE gets its bright shore-line on top of the
 *                  water fill. Multi-line patterns render back-to-front
 *                  so front layers cover rear ones; depth shading
 *                  (low/mid/high glyph density) gives the parallax
 *                  effect on MOUNTAINS / HILLS / ALPS.
 *
 * Performance    : O(N) per line generation (each level halves point
 *                  spacing but doubles point count → linear total).
 *                  Generation only on reset / morph-roll-over —
 *                  a few hundred cheap operations every ~10 s. Per-
 *                  frame: O(N · L) interpolation + O(W · H) render.
 *
 * References     : Algorithm / math
 *                  ────────────────
 *                  • Fournier, A., Fussell, D. & Carpenter, L. (1982) —
 *                    "Computer rendering of stochastic models",
 *                    Communications of the ACM 25(6):371-384. The
 *                    paper that introduced midpoint displacement to
 *                    graphics; the iterative subdivision loop in §5
 *                    is direct from this paper. Also the original
 *                    1-D coastline demo on which this file is modelled.
 *                  • Mandelbrot, B. B. (1983) — "The Fractal Geometry
 *                    of Nature", Freeman (revised "Fractals: Form,
 *                    Chance and Dimension", 1977). Mathematical context
 *                    for fractal coastlines: Hausdorff dimension, self-
 *                    similarity, and the famous "How long is the coast
 *                    of Britain?" question (Science 156(3775), 1967).
 *                    Explains why MD looks natural — it produces
 *                    statistically self-similar curves with the same
 *                    fractal dimension as a real coastline.
 *                  • Voss, R. F. (1985) — "Random fractal forgeries",
 *                    in R. A. Earnshaw, ed., "Fundamental Algorithms
 *                    for Computer Graphics", Springer NATO ASI Series.
 *                    The terrain-synthesis chapter that taught a
 *                    generation of graphics programmers MD / fBm /
 *                    successive random additions; the source of the
 *                    "roughness exponent" intuition (roughness = 2^-H
 *                    for fractal dimension D = 2 - H).
 *                  • Peitgen, H.-O. & Saupe, D., eds. (1988) — "The
 *                    Science of Fractal Images", Springer. Ch. 2
 *                    (Saupe) is the definitive MD reference: covers
 *                    the iterative implementation, the roughness ↔
 *                    fBm relationship, and the visible-crease artifact.
 *                    The most accessible book-length treatment.
 *                  • Miller, G. S. P. (1986) — "The definition and
 *                    rendering of terrain maps", SIGGRAPH '86. Documents
 *                    the well-known MD-creasing artifact (visible
 *                    discontinuities at every halving level) and
 *                    proposes the "square-square" subdivision as a fix.
 *                    Useful when learners notice the artifact and ask
 *                    "why is that happening?".
 *
 *                  Rendering / visualisation
 *                  ─────────────────────────
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    The canonical density → glyph ramp reference.
 *                    Underlies the GlyphDensity (LOW/MID/HIGH) mapping
 *                    in §6 and the five GlyphSet ramps in §1.
 *                  • Paul Bourke — "Fractal terrain":
 *                    https://paulbourke.net/fractals/terrain/
 *                    Hands-on examples of MD output at different
 *                    roughness values; useful side-by-side comparison
 *                    when calibrating the per-pattern roughness in §6.
 *
 *                  See also
 *                  ────────
 *                  • ../generational/diamond_square_heightmap_showcase.c
 *                    — the 2-D cousin of midpoint displacement. Same
 *                    recursive halving idea but in two dimensions,
 *                    producing heightmaps instead of silhouettes.
 *                  • ./magnetic_fields.c, ./flow_field_particles.c,
 *                    ./curl_noise_vector_field.c — the same Scene /
 *                    Grid / *Field / SimState / Controls composition
 *                    pattern applied to different algorithms; reading
 *                    them together shows how the architecture stays
 *                    constant while the algorithm changes.
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
 * Visible style families:
 *   Single line   (Tier 1) : one fractal line. COASTLINE / SKYLINE /
 *                            HORIZON / DUNES / CITY differ only in
 *                            y_centre, roughness, and fill direction.
 *                            Easiest tier to read — the algorithm in
 *                            isolation.
 *   Paired lines  (Tier 2) : two lines bracketing or stacking — VALLEY
 *                            (canyon), CAVE (narrow gap), CLIFF
 *                            (asymmetric drop), PLATEAU (flat mesa),
 *                            REEF (coral + ocean floor).
 *   Small stacks  (Tier 3) : 3-4 lines with depth shading. MOUNTAINS,
 *                            HILLS, ALPS differ only in roughness;
 *                            RIDGES and TERRACES vary count and style.
 *                            The PARALLAX illusion appears here.
 *   Multi-stacks  (Tier 4) : 5-7 thin or shallow lines, low amplitude.
 *                            WAVES, STRATA, FOREST, RAPIDS, SEDIMENT
 *                            — texture-like rather than silhouette-like.
 *   Exotic        (Tier 5) : extreme roughness (CRYSTAL 0.85),
 *                            asymmetric drops (SAWTOOTH), upside-down
 *                            fills (CEILING stalactites), and CHAOS —
 *                            8 lines with everything randomised at
 *                            init time.
 *
 * Above all: the lines MORPH continuously. Each pattern stays the
 * same configuration but the actual shapes evolve smoothly.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. For each line of the active pattern, generate two random
 *     fractals (keyframe_a and keyframe_b) using midpoint displacement.
 *     morph_t = 0; live ← keyframe_a.
 *  2. PER FRAME:
 *     a. advance_morph:   morph_t += morph_rate · dt.
 *     b. morph_completed? rotate_keyframes (A ← B, regen B); reset morph_t.
 *     c. interpolate_live_lines: live[i] = lerp(keyframe_a[i],
 *                                                keyframe_b[i], morph_t).
 *     d. render_line per spec: dispatch on RenderStyle.
 *  3. The user can press 'r' to re-roll all A/B fractals (or 'n'/'p'
 *     to switch pattern, which also re-rolls). There is no automatic
 *     reset — the morph runs indefinitely.
 *
 * KEY FORMULAS
 * ────────────
 *  Midpoint with jitter         :
 *    line[mid] = midpoint_y(line[L], line[R]) + jitter(amp)
 *              = (line[L] + line[R]) · 0.5 + uniform(−amp, +amp)
 *  Amplitude decay              :
 *    A_{level+1} = A_level · roughness   (typically 0.30 to 0.85)
 *  Number of levels             :
 *    log₂(LINE_POINTS − 1)              = log₂(256) = 8
 *  Total line points after K
 *  levels                       : 2^K + 1
 *  Frame interpolation          :
 *    live[i] = (1 − morph_t) · keyframe_a[i] + morph_t · keyframe_b[i]
 *  Render-x to line-x           :
 *    line_x = (render_x · (LINE_POINTS − 1)) / (map_w − 1)
 *
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

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
    PAIR_FLASH        =   7,    /* vestigial — kept for cross-file parity  */
};

/* Morph rate base — fraction of the morph cycle per second. 0.10 →
 * full A→B over 10 seconds. */
#define MORPH_RATE_DEF      0.10f
#define MORPH_RATE_MIN      0.01f
#define MORPH_RATE_MAX      2.00f

/* Palette width — number of distinct colour bands per theme. Each
 * LineSpec carries pair_idx ∈ [0, N_BANDS). */
#define N_BANDS             4

/* CHAOS pattern — randomisation ranges. Each parameter is drawn
 * uniformly from [LO, LO + RANGE) at INIT time. Concentrating the
 * magic numbers here makes the chaos pattern tunable without touching
 * the placer body. */
#define CHAOS_LINES_TARGET   8       /* upper line count (clamped to max_n) */
#define CHAOS_Y_LO           0.10f   /* y_centre fraction lo bound          */
#define CHAOS_Y_RANGE        0.80f   /* y_centre fraction range             */
#define CHAOS_AMP_LO         0.05f   /* amp fraction lo bound               */
#define CHAOS_AMP_RANGE      0.20f   /* amp fraction range                  */
#define CHAOS_ROUGH_LO       0.30f   /* roughness lo bound                  */
#define CHAOS_ROUGH_RANGE    0.55f   /* roughness range                     */
#define CHAOS_OUTLINE_DENOM  3       /* 1-in-N chance the line gets outline */
#define CHAOS_STYLE_COUNT    3       /* distinct RenderStyle values         */

/* MD-algorithm constants. */
#define MIDPOINT_AVG_WEIGHT  0.5f   /* in midpoint_y(L,R) = (L+R)·weight   */
#define JITTER_HALFRANGE     0.5f   /* endpoint random jitter is ±amp·this */

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state + [N/M], morph)
 *   row 1           : pattern/theme/palette + lines/pts/map + glyph
 *   row HUD_TOP..N-2: silhouette field
 *   row N-1         : keyboard action hint
 *
 * Row-1 segments are laid out left-to-right at fixed column widths so
 * each segment knows where the next one starts. Centralising the
 * widths means changing one number reflows the whole row coherently. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W     20    /* " pattern:XXXXXXXXX " column slot */
#define HUD_THEME_FIELD_W       17    /* " theme:XXXXXXXX "   column slot */
#define HUD_PALETTE_LABEL_W      9    /* " palette:"          column slot */

/*
 * Pattern — 30 fractal-silhouette presets spanning simple → complex.
 *
 * INTENT. The simulation kernel (MD-fractal generate + per-line A→B
 * morph) is identical for all patterns; only the LineSpec list
 * returned by pattern_setup changes. Adding a new pattern is one
 * placer function + one enum value + one dispatcher line — that's it.
 *
 * TIER ORGANISATION. Patterns are grouped in five tiers of increasing
 * visual / structural complexity:
 *   1 (1 line)   — single-line silhouettes, varying roughness + style
 *   2 (2 lines)  — paired lines bracketing or stacking
 *   3 (3-4 lines)— small layered stacks with depth shading
 *   4 (5-7 lines)— multi-layer stacks, texture-like
 *   5 (1-8 lines)— exotic: extreme params, mixed styles, CHAOS
 * Enum values are contiguous within each tier so n/p cycling naturally
 * walks through "easy first" to "complex last". N_PATTERNS = 30.
 *
 * Cycle with n/p; the HUD shows [N/30] so users see where they are.
 */
typedef enum {
    /* Tier 1 — single-line silhouettes */
    PATTERN_COASTLINE =  0,
    PATTERN_SKYLINE   =  1,
    PATTERN_HORIZON   =  2,
    PATTERN_DUNES     =  3,
    PATTERN_CITY      =  4,

    /* Tier 2 — paired lines */
    PATTERN_VALLEY    =  5,
    PATTERN_CAVE      =  6,
    PATTERN_CLIFF     =  7,
    PATTERN_PLATEAU   =  8,
    PATTERN_REEF      =  9,

    /* Tier 3 — small layered stacks */
    PATTERN_MOUNTAINS = 10,
    PATTERN_HILLS     = 11,
    PATTERN_ALPS      = 12,
    PATTERN_RIDGES    = 13,
    PATTERN_TERRACES  = 14,

    /* Tier 4 — multi-layer stacks */
    PATTERN_WAVES     = 15,
    PATTERN_STRATA    = 16,
    PATTERN_FOREST    = 17,
    PATTERN_RAPIDS    = 18,
    PATTERN_SEDIMENT  = 19,

    /* Tier 5 — exotic / max-complex */
    PATTERN_ATOLLS    = 20,
    PATTERN_CRYSTAL   = 21,
    PATTERN_SAWTOOTH  = 22,
    PATTERN_NEBULA    = 23,
    PATTERN_STORM     = 24,
    PATTERN_AURORA    = 25,
    PATTERN_CEILING   = 26,
    PATTERN_ISLANDS   = 27,
    PATTERN_PETRA     = 28,
    PATTERN_CHAOS     = 29,

    N_PATTERNS        = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_COASTLINE: return "COASTLINE";
    case PATTERN_SKYLINE:   return "SKYLINE  ";
    case PATTERN_HORIZON:   return "HORIZON  ";
    case PATTERN_DUNES:     return "DUNES    ";
    case PATTERN_CITY:      return "CITY     ";
    case PATTERN_VALLEY:    return "VALLEY   ";
    case PATTERN_CAVE:      return "CAVE     ";
    case PATTERN_CLIFF:     return "CLIFF    ";
    case PATTERN_PLATEAU:   return "PLATEAU  ";
    case PATTERN_REEF:      return "REEF     ";
    case PATTERN_MOUNTAINS: return "MOUNTAINS";
    case PATTERN_HILLS:     return "HILLS    ";
    case PATTERN_ALPS:      return "ALPS     ";
    case PATTERN_RIDGES:    return "RIDGES   ";
    case PATTERN_TERRACES:  return "TERRACES ";
    case PATTERN_WAVES:     return "WAVES    ";
    case PATTERN_STRATA:    return "STRATA   ";
    case PATTERN_FOREST:    return "FOREST   ";
    case PATTERN_RAPIDS:    return "RAPIDS   ";
    case PATTERN_SEDIMENT:  return "SEDIMENT ";
    case PATTERN_ATOLLS:    return "ATOLLS   ";
    case PATTERN_CRYSTAL:   return "CRYSTAL  ";
    case PATTERN_SAWTOOTH:  return "SAWTOOTH ";
    case PATTERN_NEBULA:    return "NEBULA   ";
    case PATTERN_STORM:     return "STORM    ";
    case PATTERN_AURORA:    return "AURORA   ";
    case PATTERN_CEILING:   return "CEILING  ";
    case PATTERN_ISLANDS:   return "ISLANDS  ";
    case PATTERN_PETRA:     return "PETRA    ";
    case PATTERN_CHAOS:     return "CHAOS    ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* §9 main loop — spiral-of-death dt clamp + terminal redraw rate cap
 * (Glenn Fiedler's "Fix Your Timestep"). */
#define DT_MAX_NS       (100 * NS_PER_MS)   /* hard cap on per-frame dt */
#define FRAME_CAP_FPS   60                  /* terminal redraw budget   */

/*
 * Theme — one complete colour palette for the silhouette layer. Ten of
 * these live in themes[], cycled by t/T.
 *
 * INTENT. The smallest data needed to recolour every line + fill in
 * the scene: 4 ramp colours arranged dim → bright, plus a "flash"
 * accent (vestigial here — kept for cross-file palette parity with
 * curl_noise_vector_field.c, magnetic_fields.c, flow_field_particles.c,
 * domain_warped_noise_iq_style.c).
 *
 * BANDING MODEL. Each LineSpec carries a pair_idx ∈ [0, 4) selecting
 * its colour band. The four-band split forces theme authors to design
 * the ramp as a coherent low → high progression — same discipline as
 * a Houdini ramp parameter or Substance Designer gradient. Multi-layer
 * patterns (MOUNTAINS, FOREST, SEDIMENT) assign band 3 → 0 across
 * back → front layers so colour itself encodes depth, on top of the
 * GlyphDensity depth shading.
 *
 * COLOUR FORMAT. xterm-256 indices (NOT RGB). When the terminal exposes
 * fewer than 256 colours, theme_apply() substitutes a fixed 8-colour
 * cycle so the demo still runs on legacy TTYs.
 *
 * REFERENCE. The 256-colour cube layout (16 base + 6³ cube + 24-step
 * grayscale ramp) is documented in XTerm's ctlseqs.ms; the band values
 * in themes[] are picked from that cube. See also CLAUDE.md "Theme
 * Palette Brightness" for this project's bright-half constraint.
 */
typedef struct {
    const char *name;                /* short uppercase label shown in HUD      */
    short       band[N_BANDS];       /* ramp colours: 0 = dim/low, 3 = bright   */
    short       flash;               /* vestigial — kept for cross-file parity  */
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
 * GlyphSet — 3-character density ramp (slim → fat). Five sets exist
 * (SLIM → FAT thickness ramp); cycle with g/G.
 *
 * INTENT. Different themes want different glyph weights to read
 * cleanly. MONO (greyscale) looks better with the fatter HEAVY ramp;
 * MATRIX looks more "wireframe" with SLIM. Decoupling glyph choice
 * from theme choice lets the user pair them freely without coding up
 * theme × ramp combinations.
 *
 * USAGE. Each LineSpec carries a GlyphDensity (LOW / MID / HIGH); the
 * renderer picks the matching glyph from this struct at draw time.
 * That double-decoupling (theme picks palette, GlyphSet picks ramp,
 * LineSpec picks which slot of the ramp) is how the same five sets
 * service all 30 patterns without per-pattern customisation.
 *
 * REFERENCE. The principle of mapping density → glyph along a coarse
 * ramp is Paul Bourke's ASCII grey-scale ramp idea:
 *   https://paulbourke.net/dataformats/asciiart/
 * Bourke proposes 10-step and 70-step ramps for grayscale images; we
 * use a 3-step ramp because silhouette rendering is intentionally
 * monotonic — finer steps would just add noise without information.
 */
typedef struct {
    const char *name;             /* short label shown in HUD glyph indicator */
    char        low;              /* sparsest glyph — back layers, dim trails */
    char        mid;              /* middle weight                            */
    char        high;             /* densest glyph — front layers, outlines   */
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
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
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
/* §5  md — midpoint displacement core                                    */
/* ===================================================================== */

/* rand_signed — uniform sample in [-1, +1]. The atomic source of all
 * randomness in this file. */
static inline float rand_signed(void)
{
    return ((float)rand() / (float)RAND_MAX) * 2.0f - 1.0f;
}

/* midpoint_y — the average-of-endpoints step of one MD subdivision.
 * Named so the algorithm reads as "midpoint plus jitter" rather than
 * "(L + R) * 0.5f plus random". */
static inline float midpoint_y(float left, float right)
{
    return (left + right) * MIDPOINT_AVG_WEIGHT;
}

/* jitter — the random-displacement step of one MD subdivision. */
static inline float jitter(float amp)
{
    return rand_signed() * amp;
}

/*
 * md_generate — fill `line[0..n-1]` with a midpoint-displacement
 * fractal connecting (0, left_y) to (n-1, right_y).
 *
 * n must be a power of 2 plus 1 (e.g. 257 = 2^8 + 1). amp_init is
 * the displacement magnitude at the FIRST level; it shrinks by
 * `roughness` at each subsequent level.
 *
 * The inner-loop body now reads as the algorithm states it:
 *   line[i] = midpoint_y(line[L], line[R]) + jitter(amp)
 * which is exactly the FFC 1982 paper's recursion equation.
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
            line[i] = midpoint_y(line[i - half], line[i + half])
                    + jitter(amp);
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
/* §6  patterns — 30 line configurations + render-style dispatcher        */
/* ===================================================================== */

/*
 * RenderStyle — what the renderer does with one line. Each LineSpec
 * carries its own style so the renderer is pattern-agnostic.
 *   FILL_BELOW — paint from the line down to the bottom of the map.
 *                Foundation of COASTLINE / CITY / MOUNTAINS / etc.
 *   FILL_ABOVE — paint from the top of the map down to the line. The
 *                visual inverse of FILL_BELOW; used for ceilings, cave
 *                roofs, sky-fill, stalactites.
 *   THIN_LINE  — paint only the line itself (one glyph per column).
 *                Used by WAVES / RIDGES / STRATA / AURORA where the
 *                line is the entire visual.
 */
typedef enum {
    STYLE_FILL_BELOW = 0,
    STYLE_FILL_ABOVE = 1,
    STYLE_THIN_LINE  = 2,
} RenderStyle;

/*
 * GlyphDensity — which glyph of the active GlyphSet to use for this
 * line. Multi-layer patterns vary density across layers to encode
 * depth: GLYPH_LOW (sparsest, recedes) for back layers, GLYPH_HIGH
 * (densest, prominent) for front layers. The parallax illusion on
 * MOUNTAINS / HILLS / ALPS / FOREST comes from this.
 */
typedef enum {
    GLYPH_LOW  = 0,
    GLYPH_MID  = 1,
    GLYPH_HIGH = 2,
} GlyphDensity;

/*
 * LineSpec — everything needed to generate ONE MD line and render it.
 * One LineSpec drives one fractal polyline through its entire life:
 * md_generate consumes the fractal params at keyframe time, render_line
 * consumes the render params at draw time.
 *
 * FRACTAL PARAMS. Fed to md_generate (§5). The four-knob design
 * (y_centre + amp + roughness + endpoint dys) is the smallest set
 * that spans every pattern in §6 — from gentle coastlines to chaotic
 * crystals. Larger sets exist (Voss successive random additions adds
 * per-level seeds; Saupe diamond-square adds 2-D constraints) but for
 * 1-D MD this is sufficient.
 *
 * ROUGHNESS NOTE. The `roughness` field is the per-level amplitude
 * multiplier. It maps to fBm's Hurst exponent H via roughness = 2^-H;
 * H ∈ [0, 1] corresponds to roughness ∈ [0.5, 1.0]. We allow the full
 * [0.30, 0.85] range — values < 0.5 give "smoother than Brownian"
 * (DUNES, PLATEAU, TERRACES); values > 0.7 give the spiky "more
 * chaotic than Brownian" look (CITY, CRYSTAL). Voss (1985) gives the
 * theoretical foundation; Saupe in Peitgen & Saupe (1988) ch. 2 has
 * the practical roughness-tuning guide.
 *
 * RENDER PARAMS. Consumed by render_line (§8). The (pair_idx, style,
 * glyph_density) triple is the minimal description of what to paint;
 * outline_idx is optional (set to -1 to skip). The render layer is
 * pattern-agnostic — every visual decision lives in the LineSpec, not
 * in scene_draw.
 *
 * REFERENCES. Fournier-Fussell-Carpenter 1982 (the MD algorithm
 * itself); Voss 1985 (roughness ↔ H mapping); Peitgen & Saupe 1988
 * ch. 2 (practical roughness tuning).
 */
typedef struct {
    /* Fractal params — fed to md_generate(). */

    /* Preferred mid-line y (cell units). Acts as the "anchor" of the
     * line — keyframes will hover around this value plus jitter. */
    float        y_centre;

    /* Displacement magnitude at the FIRST level of subdivision (cell
     * units). Shrinks by `roughness` at each subsequent level. Larger
     * amp = taller peaks / deeper valleys. */
    float        amp_init;

    /* Per-level amplitude multiplier. < 0.5 = smoother than Brownian
     * (dunes, hills), 0.5 = Brownian, > 0.5 = rougher (cities, crystals).
     * Maps to fBm Hurst exponent H via roughness = 2^-H. */
    float        roughness;

    /* Endpoint y-offsets relative to y_centre. Non-zero values give
     * the line an overall slope baked into its endpoints (CLIFF,
     * SAWTOOTH, PETRA-tilted-strata). Zero = symmetric endpoints. */
    float        left_dy, right_dy;

    /* Render params — consumed by render_line(). */

    /* Palette band 0..3 for the fill / thin-line colour. */
    int          pair_idx;

    /* Palette band 0..3 for an optional outline drawn on top of a
     * fill (-1 = no outline). Lets COASTLINE paint a bright shore-line
     * over its water fill, CITY a contrasting skyline over its building
     * mass, CRYSTAL bright tips over the crystal body. */
    int          outline_idx;

    /* FILL_BELOW / FILL_ABOVE / THIN_LINE — see RenderStyle. */
    RenderStyle  style;

    /* GLYPH_LOW / MID / HIGH — selects which of the GlyphSet's three
     * glyphs the renderer uses. Drives depth shading on multi-layer
     * patterns (back layers GLYPH_LOW, front GLYPH_HIGH). */
    GlyphDensity glyph_density;
} LineSpec;

/* ── tier 1 — single-line silhouettes (1-5) ─────────────────────── *
 * One MD line per pattern. Varying y_centre, roughness, and fill
 * direction gives five distinct silhouette styles from the same
 * algorithm. Easiest tier to read — the algorithm in isolation. */

/* COASTLINE — water below a gentle coast line. The canonical MD demo:
 * coined the name "fractal coastline" (Mandelbrot 1967). */
static int place_coastline(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.60f, .amp_init = fh * 0.20f, .roughness = 0.55f,
        .pair_idx = 0, .outline_idx = 2,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* SKYLINE — visual inverse of COASTLINE: fill ABOVE the line, so the
 * silhouette reads as a coloured ceiling against a clear floor. */
static int place_skyline(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.30f, .amp_init = fh * 0.18f, .roughness = 0.55f,
        .pair_idx = 3, .outline_idx = 1,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* HORIZON — bare thin line, no fill. Shows the raw MD output with
 * nothing else on screen — the "what does this algorithm actually
 * produce" pattern. */
static int place_horizon(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.50f, .amp_init = fh * 0.10f, .roughness = 0.50f,
        .pair_idx = 2, .outline_idx = -1,
        .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* DUNES — smooth rolling silhouette. roughness=0.40 (well below
 * Brownian's 0.50) makes amp decay fast, giving a quiet line with
 * gentle large-scale undulation only — sand-dune feel. */
static int place_dunes(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.65f, .amp_init = fh * 0.15f, .roughness = 0.40f,
        .pair_idx = 1, .outline_idx = -1,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_MID,
    };
    return 1;
}

/* CITY — opposite of DUNES. High roughness (0.75) keeps the amp
 * alive at every level — sharp peaks become skyscrapers, valleys
 * become streets. Outline draws the skyline against the sky. */
static int place_city(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.70f, .amp_init = fh * 0.30f, .roughness = 0.75f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* ── tier 2 — paired lines (6-10) ──────────────────────────────── *
 * Two MD lines bracketing or stacking. Demonstrates how FILL_ABOVE
 * and FILL_BELOW combine to build enclosed spaces. */

/* VALLEY — canyon: top line fills upward (the ceiling rock), bottom
 * line fills downward (the floor rock); the gap between is the open
 * canyon. */
static int place_valley(LineSpec out[], int max_n, int h)
{
    if (max_n < 2) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.25f, .amp_init = fh * 0.15f, .roughness = 0.55f,
        .pair_idx = 2, .outline_idx = -1,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_MID,
    };
    out[1] = (LineSpec){
        .y_centre = fh * 0.75f, .amp_init = fh * 0.15f, .roughness = 0.55f,
        .pair_idx = 0, .outline_idx = -1,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_MID,
    };
    return 2;
}

/* CAVE — narrow VALLEY: same construction but tighter gap (y=0.40 +
 * y=0.60) and outlined roof/floor in band[3] for the dramatic glow. */
static int place_cave(LineSpec out[], int max_n, int h)
{
    if (max_n < 2) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.40f, .amp_init = fh * 0.08f, .roughness = 0.55f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_HIGH,
    };
    out[1] = (LineSpec){
        .y_centre = fh * 0.60f, .amp_init = fh * 0.08f, .roughness = 0.55f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 2;
}

/* CLIFF — asymmetric endpoints (left low, right high) plus mid
 * roughness gives the classic terrain-cliff drop. The slope is
 * baked into the endpoint dys; the MD jitter on top gives natural
 * roughness. */
static int place_cliff(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre  = fh * 0.50f,
        .amp_init  = fh * 0.10f, .roughness = 0.55f,
        .left_dy   = -fh * 0.25f, .right_dy = +fh * 0.25f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* PLATEAU — very low roughness (0.30) collapses MD into a near-flat
 * line with one big midpoint displacement and nothing else — gives
 * the broad flat-top + steep-sides feel of a mesa. */
static int place_plateau(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.50f, .amp_init = fh * 0.25f, .roughness = 0.30f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* REEF — two stacked FILL_BELOWs: deep sea floor (low band, low
 * density) in the back, coral reef (bright band, outlined) in front.
 * Front fills cover the back so depth reads correctly. */
static int place_reef(LineSpec out[], int max_n, int h)
{
    if (max_n < 2) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.85f, .amp_init = fh * 0.06f, .roughness = 0.55f,
        .pair_idx = 0, .outline_idx = -1,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_LOW,
    };
    out[1] = (LineSpec){
        .y_centre = fh * 0.60f, .amp_init = fh * 0.18f, .roughness = 0.65f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 2;
}

/* ── tier 3 — small layered stacks (11-15) ─────────────────────── *
 * 3-4 lines with depth shading. The parallax illusion comes from
 * GlyphDensity rising back-to-front (low → mid → high). */

/* MOUNTAINS — 4 ranges back-to-front. y_centre decreases as we go
 * back (further → higher on screen), pair_idx tracks depth.
 * Fournier-Fussell-Carpenter 1982 fig.3. */
static int place_mountains(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.45f, 0.55f, 0.65f, 0.75f };
    const float amps[4]  = { 0.18f, 0.20f, 0.22f, 0.25f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    const GlyphDensity gd[4] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.58f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* HILLS — MOUNTAINS with roughness dropped to 0.40 → smoother
 * silhouettes, gentler rolling. Same parallax architecture. */
static int place_hills(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[3]    = { 0.55f, 0.65f, 0.75f };
    const float amps[3]  = { 0.18f, 0.20f, 0.22f };
    const int   pairs[3] = { 3, 2, 0 };
    const GlyphDensity gd[3] = { GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.40f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* ALPS — MOUNTAINS with roughness pushed to 0.70+ → sharp jagged
 * peaks. Roughness rises with depth so the front range is the most
 * spiky of all. */
static int place_alps(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[3]     = { 0.40f, 0.55f, 0.70f };
    const float amps[3]   = { 0.25f, 0.28f, 0.30f };
    const float roughs[3] = { 0.70f, 0.72f, 0.75f };
    const int   pairs[3]  = { 3, 2, 0 };
    const GlyphDensity gd[3] = { GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = roughs[i],
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* RIDGES — 4 thin lines stacked, no fill. Each line is a clear ridge
 * silhouette without the visual weight of a fill, so all 4 layers
 * stay independently visible. */
static int place_ridges(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.30f, 0.45f, 0.60f, 0.75f };
    const float amps[4]  = { 0.10f, 0.12f, 0.15f, 0.18f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.65f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* TERRACES — 4 nearly-flat FILL_BELOW lines stacked. roughness=0.30
 * makes each line flat-ish; stacked at successive y values they read
 * as a stepped agricultural-terrace silhouette. */
static int place_terraces(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.30f, 0.45f, 0.60f, 0.75f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    const GlyphDensity gd[4] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.08f, .roughness = 0.30f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* ── tier 4 — multi-layer stacks (16-20) ───────────────────────── *
 * 5-7 lines with low amplitude. Texture-like rather than silhouette-
 * like; the cumulative pattern is what the user sees, not any single
 * line. */

/* WAVES — 6 thin ocean-surface lines stacked vertically. Low amp +
 * mid roughness gives the layered ocean-cross-section look. */
static int place_waves(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    for (int i = 0; i < 6 && n < max_n; i++) {
        float t = (float)(i + 1) / 7.0f;
        out[n++] = (LineSpec){
            .y_centre = fh * t, .amp_init = fh * 0.04f, .roughness = 0.55f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* STRATA — 7 nearly-flat thin lines. Very low amp (0.02) + very low
 * roughness (0.35) → near-horizontal lines, like sedimentary rock
 * strata viewed edge-on. */
static int place_strata(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    for (int i = 0; i < 7 && n < max_n; i++) {
        float t = (float)(i + 1) / 8.0f;
        out[n++] = (LineSpec){
            .y_centre = fh * t, .amp_init = fh * 0.02f, .roughness = 0.35f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* FOREST — 5 FILL_BELOW canopy layers with elevated roughness (0.65)
 * → the bumpy tops read as treetops, the stacked layers as receding
 * tree-line. Lower amp than MOUNTAINS so the canopies stay tight. */
static int place_forest(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[5]    = { 0.40f, 0.50f, 0.60f, 0.70f, 0.80f };
    const int   pairs[5] = { 3, 2, 1, 2, 0 };
    const GlyphDensity gd[5] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 5 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.10f, .roughness = 0.65f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* RAPIDS — 5 thin lines closely spaced around mid-height, very low
 * amp + high roughness → fast-rippling water surface, like a river
 * cross-section. */
static int place_rapids(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[5] = { 0.45f, 0.50f, 0.55f, 0.60f, 0.65f };
    for (int i = 0; i < 5 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.02f, .roughness = 0.70f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* SEDIMENT — 5 FILL_BELOW layers with very low roughness, palette
 * cycling top-to-bottom. The stacked fills cover earlier layers, so
 * what remains visible is the SHELF of each successive layer — exactly
 * the visual signature of a layered geological cross-section. */
static int place_sediment(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[5]    = { 0.45f, 0.55f, 0.65f, 0.75f, 0.85f };
    const int   pairs[5] = { 3, 2, 1, 0, 0 };
    const GlyphDensity gd[5] = { GLYPH_LOW, GLYPH_LOW, GLYPH_MID, GLYPH_HIGH, GLYPH_HIGH };
    for (int i = 0; i < 5 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.10f, .roughness = 0.40f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = gd[i],
        };
    }
    return n;
}

/* ── tier 5 — exotic / max-complex (21-30) ─────────────────────── *
 * Extreme parameters: roughness near 1, fill-above, asymmetric
 * endpoints, full randomisation. The frontier of what MD can do. */

/* ATOLLS — high-roughness shore line at low y → many small peaks
 * poking out of the "water". Each peak reads as an island in a chain. */
static int place_atolls(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.78f, .amp_init = fh * 0.10f, .roughness = 0.80f,
        .pair_idx = 0, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* CRYSTAL — extreme roughness (0.85): amp barely decays across levels,
 * so high-frequency jitter is preserved all the way down. Result is a
 * sharply spiked silhouette, like a crystal cluster. */
static int place_crystal(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.55f, .amp_init = fh * 0.30f, .roughness = 0.85f,
        .pair_idx = 2, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* SAWTOOTH — CLIFF amplified: large endpoint asymmetry + high amp +
 * high roughness. The big slope plus all-scale roughness give a
 * dramatic saw-toothed terrain. */
static int place_sawtooth(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre  = fh * 0.55f,
        .amp_init  = fh * 0.25f, .roughness = 0.70f,
        .left_dy   = -fh * 0.20f, .right_dy = +fh * 0.20f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* NEBULA — 6 thin lines at varied y_centre with HIGH amp + LOW
 * roughness. The smooth large-scale variations + irregular spacing
 * make the lines look like overlapping gas wisps. */
static int place_nebula(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[6] = { 0.20f, 0.30f, 0.45f, 0.55f, 0.70f, 0.85f };
    for (int i = 0; i < 6 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * 0.12f, .roughness = 0.40f,
            .pair_idx = i & 3, .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* STORM — combination pattern: 3 FILL_ABOVE turbulent cloud layers at
 * the top + 1 FILL_BELOW ground line at the bottom. Demonstrates how
 * different styles compose into a single coherent scene. */
static int place_storm(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float cloud_ys[3]   = { 0.15f, 0.25f, 0.35f };
    const float cloud_amps[3] = { 0.15f, 0.18f, 0.20f };
    const int   cloud_p[3]    = { 2, 1, 0 };
    const GlyphDensity cloud_gd[3] = { GLYPH_LOW, GLYPH_MID, GLYPH_HIGH };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * cloud_ys[i], .amp_init = fh * cloud_amps[i],
            .roughness = 0.70f,
            .pair_idx = cloud_p[i], .outline_idx = -1,
            .style = STYLE_FILL_ABOVE, .glyph_density = cloud_gd[i],
        };
    }
    if (n < max_n) {
        out[n++] = (LineSpec){
            .y_centre = fh * 0.85f, .amp_init = fh * 0.10f, .roughness = 0.55f,
            .pair_idx = 0, .outline_idx = -1,
            .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* AURORA — 4 thin lines tightly clustered in the upper third of the
 * screen, low roughness for smooth flowing wisps. Reads as ribbons of
 * aurora-borealis light moving across the sky. */
static int place_aurora(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const float ys[4]    = { 0.20f, 0.25f, 0.30f, 0.35f };
    const float amps[4]  = { 0.15f, 0.12f, 0.10f, 0.08f };
    const int   pairs[4] = { 3, 2, 1, 0 };
    for (int i = 0; i < 4 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre = fh * ys[i], .amp_init = fh * amps[i], .roughness = 0.35f,
            .pair_idx = pairs[i], .outline_idx = -1,
            .style = STYLE_THIN_LINE, .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/* CEILING — high-roughness FILL_ABOVE at low y_centre. Spikes point
 * down from the filled ceiling, reading as stalactites in a cave roof. */
static int place_ceiling(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.30f, .amp_init = fh * 0.15f, .roughness = 0.80f,
        .pair_idx = 1, .outline_idx = 3,
        .style = STYLE_FILL_ABOVE, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* ISLANDS — like ATOLLS but lower roughness → fewer, larger islands
 * with smoother shorelines. The contrast pair makes ATOLLS feel "rocky"
 * and ISLANDS feel "tropical". */
static int place_islands(LineSpec out[], int max_n, int h)
{
    if (max_n < 1) return 0;
    float fh = (float)h;
    out[0] = (LineSpec){
        .y_centre = fh * 0.75f, .amp_init = fh * 0.10f, .roughness = 0.65f,
        .pair_idx = 0, .outline_idx = 2,
        .style = STYLE_FILL_BELOW, .glyph_density = GLYPH_HIGH,
    };
    return 1;
}

/* PETRA — 3 tilted sandstone layers, each with different endpoint
 * asymmetry so they don't stack flat. The visual evokes the layered
 * tilted strata of the Petra rose-city carvings. Each row of the
 * layers[] table is one stratum; the loop body is pure data → struct. */
static int place_petra(LineSpec out[], int max_n, int h)
{
    int n = 0;
    float fh = (float)h;
    const struct {
        float        y_frac, left_tilt, right_tilt;
        int          pair;
        GlyphDensity density;
    } layers[3] = {
        { 0.45f, -0.10f, +0.05f, 3, GLYPH_LOW  },
        { 0.60f, +0.05f, -0.10f, 2, GLYPH_MID  },
        { 0.75f, -0.05f, +0.10f, 1, GLYPH_HIGH },
    };
    for (int i = 0; i < 3 && n < max_n; i++) {
        out[n++] = (LineSpec){
            .y_centre      = fh * layers[i].y_frac,
            .amp_init      = fh * 0.10f, .roughness = 0.50f,
            .left_dy       = fh * layers[i].left_tilt,
            .right_dy      = fh * layers[i].right_tilt,
            .pair_idx      = layers[i].pair, .outline_idx = -1,
            .style         = STYLE_FILL_BELOW,
            .glyph_density = layers[i].density,
        };
    }
    return n;
}

/* random_unit_uniform — uniform sample in [0, 1). Coarser than
 * rand_signed (~1000 steps) but the only place we need it is the
 * CHAOS placer where exact resolution doesn't matter. */
static inline float random_unit_uniform(void)
{
    return (float)(rand() % 1000) / 1000.0f;
}

/* random_in_range — uniform sample in [lo, lo + range). */
static inline float random_in_range(float lo, float range)
{
    return lo + random_unit_uniform() * range;
}

/* random_render_style — pick FILL_BELOW / FILL_ABOVE / THIN_LINE
 * with equal probability. */
static RenderStyle random_render_style(void)
{
    switch (rand() % CHAOS_STYLE_COUNT) {
    case 0:  return STYLE_FILL_BELOW;
    case 1:  return STYLE_FILL_ABOVE;
    default: return STYLE_THIN_LINE;
    }
}

/* random_outline_band — return a palette band index or -1 (no outline).
 * The 1-in-CHAOS_OUTLINE_DENOM probability means outlines are accents,
 * not the dominant visual. */
static int random_outline_band(void)
{
    return (rand() % CHAOS_OUTLINE_DENOM == 0) ? (rand() % N_BANDS) : -1;
}

/* CHAOS — 8 lines, every parameter randomised at INIT (not per frame
 * — the morph still works). Maximum visual complexity, no two presses
 * of 'r' look alike. */
static int place_chaos(LineSpec out[], int max_n, int h)
{
    int target = CHAOS_LINES_TARGET;
    if (target > max_n) target = max_n;
    int n = 0;
    float fh = (float)h;
    for (int i = 0; i < target; i++) {
        out[n++] = (LineSpec){
            .y_centre      = fh * random_in_range(CHAOS_Y_LO,     CHAOS_Y_RANGE),
            .amp_init      = fh * random_in_range(CHAOS_AMP_LO,   CHAOS_AMP_RANGE),
            .roughness     =      random_in_range(CHAOS_ROUGH_LO, CHAOS_ROUGH_RANGE),
            .pair_idx      = rand() % N_BANDS,
            .outline_idx   = random_outline_band(),
            .style         = random_render_style(),
            .glyph_density = GLYPH_HIGH,
        };
    }
    return n;
}

/*
 * pattern_setup — DISPATCHER. For each pattern, call its placer and
 * return the line count. One line per case — pure pseudocode.
 */
static int pattern_setup(LineSpec out[], int max_n, Pattern p, int h)
{
    switch (p) {

    /* Tier 1 — single-line silhouettes */
    case PATTERN_COASTLINE: return place_coastline(out, max_n, h);
    case PATTERN_SKYLINE:   return place_skyline  (out, max_n, h);
    case PATTERN_HORIZON:   return place_horizon  (out, max_n, h);
    case PATTERN_DUNES:     return place_dunes    (out, max_n, h);
    case PATTERN_CITY:      return place_city     (out, max_n, h);

    /* Tier 2 — paired lines */
    case PATTERN_VALLEY:    return place_valley   (out, max_n, h);
    case PATTERN_CAVE:      return place_cave     (out, max_n, h);
    case PATTERN_CLIFF:     return place_cliff    (out, max_n, h);
    case PATTERN_PLATEAU:   return place_plateau  (out, max_n, h);
    case PATTERN_REEF:      return place_reef     (out, max_n, h);

    /* Tier 3 — small layered stacks */
    case PATTERN_MOUNTAINS: return place_mountains(out, max_n, h);
    case PATTERN_HILLS:     return place_hills    (out, max_n, h);
    case PATTERN_ALPS:      return place_alps     (out, max_n, h);
    case PATTERN_RIDGES:    return place_ridges   (out, max_n, h);
    case PATTERN_TERRACES:  return place_terraces (out, max_n, h);

    /* Tier 4 — multi-layer stacks */
    case PATTERN_WAVES:     return place_waves    (out, max_n, h);
    case PATTERN_STRATA:    return place_strata   (out, max_n, h);
    case PATTERN_FOREST:    return place_forest   (out, max_n, h);
    case PATTERN_RAPIDS:    return place_rapids   (out, max_n, h);
    case PATTERN_SEDIMENT:  return place_sediment (out, max_n, h);

    /* Tier 5 — exotic / max-complex */
    case PATTERN_ATOLLS:    return place_atolls   (out, max_n, h);
    case PATTERN_CRYSTAL:   return place_crystal  (out, max_n, h);
    case PATTERN_SAWTOOTH:  return place_sawtooth (out, max_n, h);
    case PATTERN_NEBULA:    return place_nebula   (out, max_n, h);
    case PATTERN_STORM:     return place_storm    (out, max_n, h);
    case PATTERN_AURORA:    return place_aurora   (out, max_n, h);
    case PATTERN_CEILING:   return place_ceiling  (out, max_n, h);
    case PATTERN_ISLANDS:   return place_islands  (out, max_n, h);
    case PATTERN_PETRA:     return place_petra    (out, max_n, h);
    case PATTERN_CHAOS:     return place_chaos    (out, max_n, h);

    default:                return 0;
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built out of four small sub-structs. Each owns ONE
 * concern, and Scene composes them. Splitting concerns into clearly-
 * typed sub-structs makes function signatures self-describing: any
 * function that takes `const Grid *` clearly cannot mutate the fractal;
 * any function that takes `FractalField *` clearly does not advance
 * morph time; and so on. Same architectural shape as the other field
 * files in this directory.
 */

/*
 * Grid — canvas geometry. Pure data: no buffers, no state. Lives at
 * the top of Scene because every layer (line generation, screen
 * centring, the render pipeline) needs the dimensions.
 *
 * INVARIANT. w ≤ MAP_W_MAX, h ≤ MAP_H_MAX always hold. app_pick_map_size()
 * enforces the clamp once per resize; downstream code may assume it.
 *
 * NO total_cells. Unlike the cell-space files (magnetic_fields,
 * flow_field_particles) we don't index a per-cell buffer here — the
 * render reads directly from the live[] lines, so this struct stays
 * minimal.
 */
typedef struct {
    int w, h;     /* current map width / height in cells */
} Grid;

static inline bool grid_in_bounds(const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * FractalField — the algorithm's data source. Mirrors PoleField in
 * magnetic_fields.c and AttractorPool in flow_field_particles.c:
 * a fixed-size pool of "live data" the renderer reads from.
 *
 *   spec[]        — per-line LineSpec (fractal params + render hints).
 *   keyframe_a[]  — A keyframe per line: start of the current A→B
 *                   morph. Each keyframe is a fully-generated MD
 *                   fractal polyline.
 *   keyframe_b[]  — B keyframe per line: target of the current morph.
 *                   When the morph completes, A ← B and B is regen'd.
 *   live[]        — the per-frame interpolated line. The renderer
 *                   reads ONLY this; it is the entire render contract
 *                   between scene_tick (writer) and scene_draw (reader).
 *
 * INTENT. The A/B/live split is the classic KEYFRAME + LERP animation
 * pattern. Keyframes are expensive to compute (md_generate runs
 * log₂(LINE_POINTS-1) levels, ~256 displacements per line). By
 * generating only at scene reset + morph rollover (every ~1/morph_rate
 * seconds — default ~10s) we keep the per-frame work to a cheap
 * linear interpolation in line_lerp.
 *
 * REFERENCES. Keyframe + in-between animation, Disney 1981 (Mealing).
 * Fractional Brownian motion / MD fractals, Fournier-Fussell-Carpenter
 * 1982 — see §5 for the algorithm reference.
 */
typedef struct {
    int      n_lines;
    LineSpec spec       [MAX_LINES];
    float    keyframe_a [MAX_LINES][LINE_POINTS];
    float    keyframe_b [MAX_LINES][LINE_POINTS];
    float    live       [MAX_LINES][LINE_POINTS];
} FractalField;

/*
 * SimState — the single mutable per-tick scalar. Separated from
 * Controls (keyboard-driven knobs) so it is obvious what scene_tick
 * mutates vs. what the user does.
 *
 * morph_t advances each tick by ctrl.morph_rate · dt. When it crosses
 * 1.0, the morph "rolls over": keyframe A ← keyframe B, B is
 * regenerated to a fresh fractal, and morph_t resets to 0. The next
 * morph cycle then plays out from the new starting point.
 *
 * One scalar = one named struct. Tiny but worth its own type for
 * symmetry with the other field files (SimState in magnetic_fields,
 * curl_noise, flow_field, domain_warped — all just one or two floats).
 */
typedef struct {
    float morph_t;     /* 0..1 progress along the current A→B morph */
} SimState;

/*
 * Controls — user-facing knobs. Mutated only by app_handle_key(),
 * read by scene_tick (paused, morph_rate) and screen_draw (theme,
 * glyph_set, pattern).
 *
 * INTENT. Same model-vs-user-intent split as the other field files:
 * the dependency is strictly one-way. app_handle_key writes Controls,
 * never SimState; scene_tick reads Controls but only writes SimState
 * + FractalField buffers. Keeps the code linear.
 *
 * NO prev_pattern. Switching patterns triggers a full scene_reset
 * (see cycle_pattern in §9) because each pattern needs its own
 * LineSpec list — there's no need to detect the switch with a
 * one-tick lag.
 */
typedef struct {
    /* Gate for scene_tick: when true the morph freezes but the render
     * loop continues so the HUD stays responsive. */
    bool    paused;

    int     current_theme;       /* index into themes[N_THEMES]          */
    int     current_glyph_set;   /* index into glyph_sets[N_GLYPH_SETS]  */
    Pattern current_pattern;     /* the active fractal-silhouette preset */

    /* Fraction of an A→B morph completed per second. Doubled by '+',
     * halved by '-', clamped to [MORPH_RATE_MIN, MORPH_RATE_MAX].
     * 0.10 → full A→B over 10 seconds (the default). */
    float   morph_rate;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom is
 * meant to be the fastest way to understand the program:
 *   grid     → where things live      (geometry)
 *   fractal  → the MD lines           (data source / keyframes / live)
 *   sim      → animation state        (morph_t)
 *   ctrl     → user knobs             (pattern, theme, morph_rate, …)
 *
 * ORDERING. Each sub-struct depends only on those declared above it:
 * grid is leaf-level; fractal needs grid bounds for placement; sim
 * mutates fractal via interpolate_live_lines; ctrl decides which sim
 * path runs. A reader scanning top-down meets every concept before it
 * is used.
 */
typedef struct {
    Grid          grid;       /* canvas dimensions                          */
    FractalField  fractal;    /* MD lines + per-line A/B keyframes + live   */
    SimState      sim;        /* morph_t — mutated only by scene_tick       */
    Controls      ctrl;       /* mutated only by app_handle_key             */
} Scene;

/* ── fractal generation pipeline ─────────────────────────────────── *
 * MD-fractal generation factored into ONE primitive operation
 * (generate_keyframe) and named bulk operations that work on the
 * whole FractalField. */

/*
 * generate_keyframe — produce ONE fresh MD-fractal line for the given
 * spec. The endpoints are anchored at y_centre ± left_dy / ± right_dy
 * but jittered by ±amp_init/2 so successive keyframes have distinct
 * starting shapes (no two morphs are identical).
 */
static void generate_keyframe(const LineSpec *s, float *out)
{
    float endpoint_jitter = s->amp_init * JITTER_HALFRANGE;
    float left            = s->y_centre + s->left_dy  + jitter(endpoint_jitter);
    float right           = s->y_centre + s->right_dy + jitter(endpoint_jitter);
    md_generate(out, LINE_POINTS, left, right, s->amp_init, s->roughness);
}

static void generate_all_keyframes_a(FractalField *ff)
{
    for (int i = 0; i < ff->n_lines; i++)
        generate_keyframe(&ff->spec[i], ff->keyframe_a[i]);
}

static void generate_all_keyframes_b(FractalField *ff)
{
    for (int i = 0; i < ff->n_lines; i++)
        generate_keyframe(&ff->spec[i], ff->keyframe_b[i]);
}

/*
 * rotate_keyframes — keyframe rollover at the end of each morph:
 * A ← B (the line that WAS the target becomes the new start), then B
 * is regenerated to a fresh fractal. The morph then runs again from
 * A=old_B to B=new fractal. Net effect: continuous evolution with no
 * visible snap.
 */
static void rotate_keyframes(FractalField *ff)
{
    memcpy(ff->keyframe_a, ff->keyframe_b, sizeof(ff->keyframe_a));
    generate_all_keyframes_b(ff);
}

static void copy_keyframes_a_to_live(FractalField *ff)
{
    for (int i = 0; i < ff->n_lines; i++)
        memcpy(ff->live[i], ff->keyframe_a[i], sizeof(ff->live[i]));
}

static void interpolate_live_lines(FractalField *ff, float morph_t)
{
    for (int i = 0; i < ff->n_lines; i++)
        line_lerp(ff->keyframe_a[i], ff->keyframe_b[i],
                  ff->live[i], LINE_POINTS, morph_t);
}

/* ── morph clock ─────────────────────────────────────────────────── *
 * Three trivial operations on SimState, named so scene_tick reads as
 * pseudocode rather than algebra. */

static void advance_morph(SimState *sim, float morph_rate, float dt)
{
    sim->morph_t += morph_rate * dt;
}

static bool morph_completed(const SimState *sim)
{
    return sim->morph_t >= 1.0f;
}

static void reset_morph(SimState *sim)
{
    sim->morph_t = 0.0f;
}

/* ── reset / init pipeline ───────────────────────────────────────── */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w = w;
    g->h = h;
}

static void install_pattern(FractalField *ff, Pattern p, const Grid *g)
{
    ff->n_lines = pattern_setup(ff->spec, MAX_LINES, p, g->h);
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions     (&s->grid, w, h);
    reset_morph               (&s->sim);
    install_pattern           (&s->fractal, s->ctrl.current_pattern, &s->grid);
    generate_all_keyframes_a  (&s->fractal);
    generate_all_keyframes_b  (&s->fractal);
    copy_keyframes_a_to_live  (&s->fractal);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused             = false;
    s->ctrl.current_theme      = 0;
    s->ctrl.current_glyph_set  = 2;        /* MEDIUM */
    s->ctrl.current_pattern    = PATTERN_COASTLINE;
    s->ctrl.morph_rate         = MORPH_RATE_DEF;
    scene_reset(s, w, h);
}

/* ── tick pipeline ───────────────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     advance morph_t by morph_rate · dt.
 *     if morph_t completed: rotate keyframes (A ← B, regen B), reset.
 *     interpolate every line: live ← lerp(keyframe_a, keyframe_b, t).
 *
 * Each step is one named call. */

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    advance_morph             (&s->sim, s->ctrl.morph_rate, dt);
    if (morph_completed(&s->sim)) {
        rotate_keyframes      (&s->fractal);
        reset_morph           (&s->sim);
    }
    interpolate_live_lines    (&s->fractal, s->sim.morph_t);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal dimensions cached after the last successful
 * getmaxyx(). Refreshed on SIGWINCH via screen_resize().
 *
 * INTENT. Kept tiny because the rest of ncurses' state lives
 * implicitly in stdscr; we only need the dimensions to centre the
 * map and position HUD elements. Anything else (current attribute,
 * cursor position, colour-pair table) is owned by ncurses internals,
 * not by us — exposing it here would just duplicate state.
 *
 * NOT IN SCENE. Screen lives at App-level, not Scene, because Scene
 * is "the simulation" and the simulation is display-agnostic: any
 * scene state should be reusable on a different display backend. The
 * terminal-cell dimensions are a property of the display, not the sim.
 */
typedef struct {
    int cols;   /* terminal width  in character cells */
    int rows;   /* terminal height in character cells */
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
 * draw_fill_above_line — visual inverse of draw_fill_below_line: for
 * each on-screen column, fill from row 0 down to the line's y. Used
 * by SKYLINE, VALLEY-top, CAVE-top, CEILING, STORM clouds.
 */
static void draw_fill_above_line(const float *line, int n_points,
                                  int gx0, int gy0, int map_w, int map_h,
                                  int cols, int rows, int pair, char glyph)
{
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < map_w; x++) {
        float ly = line_sample(line, n_points, map_w, x);
        int y_end = (int)ly;
        if (y_end < 0) continue;
        if (y_end >= map_h) y_end = map_h - 1;
        for (int y = 0; y <= y_end; y++) {
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
 * (no fill below or above). Used by THIN_LINE patterns + as the
 * outline pass on top of any fill style.
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

/* pick_glyph — translate LineSpec density into the matching GlyphSet
 * character. Three-tier mapping reads as pseudocode in render_line. */
static char pick_glyph(GlyphDensity density, const GlyphSet *gs)
{
    switch (density) {
    case GLYPH_LOW:  return gs->low;
    case GLYPH_MID:  return gs->mid;
    case GLYPH_HIGH: return gs->high;
    }
    return gs->high;
}

/* maybe_paint_outline — if the spec carries an outline band, draw the
 * line a second time as a thin overlay in that colour at GLYPH_HIGH.
 * Hoisted so the FILL_BELOW and FILL_ABOVE branches of render_line
 * share one outline implementation. */
static void maybe_paint_outline(const LineSpec *spec, const float *line, int n_points,
                                 int gx0, int gy0, int map_w, int map_h,
                                 int cols, int rows, const GlyphSet *gs)
{
    if (spec->outline_idx < 0) return;
    draw_line_only(line, n_points, gx0, gy0, map_w, map_h, cols, rows,
                    PAIR_BAND_BASE + spec->outline_idx, gs->high);
}

/*
 * render_line — the per-line render dispatcher. Reads as pseudocode:
 *   pick glyph for density.
 *   pick palette pair.
 *   switch on style → paint the matching fill / thin line.
 *   if any fill, maybe overlay an outline.
 *
 * SINGLE DISPATCH POINT. scene_draw just calls this for each line in
 * spec[]; the renderer is pattern-agnostic, every pattern's visual
 * decisions live in its LineSpec.
 */
static void render_line(const LineSpec *spec, const float *line, int n_points,
                         int gx0, int gy0, int map_w, int map_h,
                         int cols, int rows, const GlyphSet *gs)
{
    char glyph = pick_glyph(spec->glyph_density, gs);
    int  pair  = PAIR_BAND_BASE + spec->pair_idx;

    switch (spec->style) {
    case STYLE_FILL_BELOW:
        draw_fill_below_line(line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, pair, glyph);
        maybe_paint_outline (spec, line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, gs);
        break;
    case STYLE_FILL_ABOVE:
        draw_fill_above_line(line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, pair, glyph);
        maybe_paint_outline (spec, line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, gs);
        break;
    case STYLE_THIN_LINE:
        draw_line_only      (line, n_points, gx0, gy0, map_w, map_h,
                              cols, rows, pair, glyph);
        break;
    }
}

/* compute_centred_origin — top-left corner where the map is drawn.
 * Centres horizontally; reserves HUD_BAND_RESERVED_ROWS rows total
 * (HUD_TOP_ROWS top + HUD_BOTTOM_ROWS bottom). */
static void compute_centred_origin(const Grid *g, int cols, int rows,
                                    int *out_gx0, int *out_gy0)
{
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0)            gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;
    *out_gx0 = gx0;
    *out_gy0 = gy0;
}

static const GlyphSet *active_glyph_set(const Controls *c)
{
    int idx = c->current_glyph_set;
    if (idx < 0 || idx >= N_GLYPH_SETS) idx = 0;
    return &glyph_sets[idx];
}

/*
 * scene_draw — walk the LineSpec list in declaration order. For fill
 * styles, that's back-to-front (later lines cover earlier), which is
 * exactly what the multi-layer patterns rely on for depth shading.
 */
static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);
    const GlyphSet *gs = active_glyph_set(&s->ctrl);

    for (int i = 0; i < s->fractal.n_lines; i++) {
        render_line(&s->fractal.spec[i], s->fractal.live[i], LINE_POINTS,
                     gx0, gy0, s->grid.w, s->grid.h, cols, rows, gs);
    }
}

/* ── HUD draw pipeline ───────────────────────────────────────────── *
 * Five named drawers, called from screen_draw in z-order. The TOP HUD
 * (rows 0..1) carries DATA — current state with [N/30] index, parameter
 * readouts, palette swatch, glyph indicator. The BOTTOM HUD (row N-1)
 * carries ACTIONS — key bindings only.
 *
 * draw_hud_status_line internally composes several smaller segment
 * drawers, one per fixed-width row-1 field. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  morph:%4.2f  rate:%4.2f ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             (double)s->sim.morph_t, (double)c->morph_rate);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " MIDPOINT-DISPLACEMENT FRACTAL ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── row 1 segment drawers ───────────────────────────────────────── *
 * draw_hud_status_line lays out row 1 left-to-right as a sequence of
 * fixed-width segments. Each segment paints its content and returns
 * the new x-cursor; the last (sim_counts) does not need to return. */

static int draw_status_pattern_field(int row, int x, Pattern p)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-9s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_PATTERN_FIELD_W;
}

static int draw_status_theme_field(int row, int x, int theme_idx)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_THEME_FIELD_W;
}

static int draw_status_palette_label(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + HUD_PALETTE_LABEL_W;
}

static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < N_BANDS; i++) {
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* draw_status_sim_counts — last segment: live line + point counts +
 * map dimensions. No return — end of the row. */
static void draw_status_sim_counts(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  lines:%d  pts:%d  map:%dx%d ",
             s->fractal.n_lines, LINE_POINTS, s->grid.w, s->grid.h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    int x = HUD_LEFT_MARGIN;
    x = draw_status_pattern_field (1, x, c->current_pattern);
    x = draw_status_theme_field   (1, x, c->current_theme);
    x = draw_status_palette_label (1, x);
    x = draw_palette_swatch       (1, x);
        draw_status_sim_counts    (1, x, s);
}

/* draw_hud_glyph_indicator — right-aligned on row 1. Live sample of
 * the three glyphs in the current set so the user can see what each
 * set looks like before switching with g/G. */
static void draw_hud_glyph_indicator(const Scene *s, const Screen *sc)
{
    const GlyphSet *gs = active_glyph_set(&s->ctrl);
    char buf[32];
    snprintf(buf, sizeof buf, " glyph:%-7s [%c%c%c] ",
             gs->name, gs->low, gs->mid, gs->high);
    int gx = sc->cols - (int)strlen(buf);
    if (gx < 0) gx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, gx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_bottom_hint — row N-1: ACTIONS only (key bindings). */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  r:reset  spc:pause  +/-:morph-rate  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw              (s, sc->cols, sc->rows);
    draw_hud_state_bar      (sc, s, fps, sim_fps);   /* row 0  : data    */
    draw_hud_title          ();                       /* row 0  : data    */
    draw_hud_status_line    (s);                      /* row 1  : data    */
    draw_hud_glyph_indicator(s, sc);                  /* row 1  : data    */
    draw_bottom_hint        (sc);                     /* row N-1: actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — top-level program state. One instance, g_app, lives in BSS
 * so the signal handlers can reach it without a global Scene pointer.
 * The App owns the Scene and the Screen and adds:
 *   • simulation parameters that are not really "scene state"
 *     (sim_fps, map_w, map_h);
 *   • signal-driven flags that must be sig_atomic_t for safety.
 *
 * SIGNAL-HANDLER DISCIPLINE. The handlers do nothing but set a flag.
 * The main loop polls those flags and performs the actual work
 * (cleanup, resize) in normal execution context. Standard async-
 * signal-safe pattern — anything that touches ncurses or malloc MUST
 * happen outside the handler.
 *
 * QUALIFIER NOTE. The running / need_resize flags carry BOTH
 * `volatile` and `sig_atomic_t`. sig_atomic_t guarantees writes from
 * a handler are observed atomically by the main loop; volatile
 * prevents the compiler from caching the read in a register across
 * loop iterations. Both qualifiers are required — sig_atomic_t alone
 * permits caching, volatile alone permits torn writes from a handler.
 *
 * REFERENCE. W. Richard Stevens & Stephen Rago — "Advanced Programming
 * in the UNIX Environment" (3rd ed), ch. 10 on signals, for the full
 * discussion of async-signal-safety and sig_atomic_t.
 */
typedef struct {
    Scene                 scene;     /* the simulation                       */
    Screen                screen;    /* current terminal dimensions          */

    int                   sim_fps;   /* tick rate; mutated by '[' and ']'    */
    int                   map_w;     /* chosen map width,  ≤ MAP_W_MAX       */
    int                   map_h;     /* chosen map height, ≤ MAP_H_MAX       */

    volatile sig_atomic_t running;       /* 0 = exit main loop               */
    volatile sig_atomic_t need_resize;   /* 1 = pending SIGWINCH             */
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

/* ── keyboard handlers ──────────────────────────────────────────────── *
 * Each key is one named action; app_handle_key is just a dispatcher. */

/* bump_morph_rate — '+' / '-' geometric step on morph rate. */
static void bump_morph_rate(Controls *c, int dir)
{
    if (dir > 0) c->morph_rate *= 2.0f;
    else         c->morph_rate *= 0.5f;
    if (c->morph_rate > MORPH_RATE_MAX) c->morph_rate = MORPH_RATE_MAX;
    if (c->morph_rate < MORPH_RATE_MIN) c->morph_rate = MORPH_RATE_MIN;
}

/* bump_sim_fps — '[' / ']' linear step on tick rate, clamped. */
static void bump_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

static void cycle_theme(Controls *c, int dir)
{
    c->current_theme = (c->current_theme + dir + N_THEMES) % N_THEMES;
    theme_apply(c->current_theme);
}

static void cycle_glyph_set(Controls *c, int dir)
{
    c->current_glyph_set =
        (c->current_glyph_set + dir + N_GLYPH_SETS) % N_GLYPH_SETS;
}

/* cycle_pattern — n/p step + full scene reset. We reset because each
 * pattern needs its own LineSpec list; reusing the previous pattern's
 * specs would give a glitched first frame. */
static void cycle_pattern(App *app, int dir)
{
    Controls *c = &app->scene.ctrl;
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
    scene_reset(&app->scene, app->map_w, app->map_h);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                              break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);    break;
    case '=': case '+': bump_morph_rate (c,   +1);                           break;
    case '-':           bump_morph_rate (c,   -1);                           break;
    case ']':           bump_sim_fps    (app, +SIM_FPS_STEP);                break;
    case '[':           bump_sim_fps    (app, -SIM_FPS_STEP);                break;
    case 't':           cycle_theme     (c,   +1);                           break;
    case 'T':           cycle_theme     (c,   -1);                           break;
    case 'g':           cycle_glyph_set (c,   +1);                           break;
    case 'G':           cycle_glyph_set (c,   -1);                           break;
    case 'n': case 'N': cycle_pattern   (app, +1);                           break;
    case 'p': case 'P': cycle_pattern   (app, -1);                           break;
    default: break;
    }
    return true;
}

/* ── main-loop helpers ──────────────────────────────────────────────── */

static void install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* advance_frame_clock — read the monotonic clock, compute dt since
 * the last call, clamp at the spiral-of-death guard. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* simulate_pending_ticks — drain the fixed-timestep accumulator.
 * Source: Glenn Fiedler, "Fix Your Timestep". */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* maybe_update_fps_counter — every FPS_UPDATE_MS, fold the running
 * frame count into a smoothed fps reading. */
static double maybe_update_fps_counter(int64_t *fps_accum,
                                        int *frame_count,
                                        double previous)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return previous;
    double fps = (double)(*frame_count) /
                  ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
    return fps;
}

/* cap_frame_rate — sleep so frames are at most 1/target_fps apart. */
static void cap_frame_rate(int64_t work_done_ns, int target_fps)
{
    int64_t budget = NS_PER_SEC / target_fps;
    clock_sleep_ns(budget - work_done_ns);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init      (&app->screen);
    app_pick_map_size(app);
    scene_init       (&app->scene, app->map_w, app->map_h);

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

        int64_t dt      = advance_frame_clock(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        simulate_pending_ticks(app, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum  += dt;
        fps_display = maybe_update_fps_counter(&fps_accum, &frame_count, fps_display);

        cap_frame_rate((clock_ns() - frame_time) + dt, FRAME_CAP_FPS);

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
