/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * domain_warped_noise_iq_style.c
 *   — Inigo Quilez-style domain warping over a Perlin/FBM field.
 *
 * DEMO: Five static-field patterns, each more "warped" than the last.
 *       Cycle them with n/p to see the technique build up:
 *         RAW   — plain fractional Brownian motion (no warp)
 *         WARP1 — one warp pass: f(x + g(x))
 *         WARP2 — TWO warp passes (IQ's classic): f(x + h(x + g(x)))
 *         WARP3 — three warp passes — the deepest nesting
 *         RIDGE — WARP2 with the ridged transform (1 − 2|n − ½|)²,
 *                 sharp marble veins on a smooth background
 *       The field drifts slowly so all patterns evolve over time.
 *       Rendering uses a 10-step ASCII density ramp (` .:-=+*#%@`)
 *       with colour DECOUPLED from intensity — each cell's colour
 *       comes from a second independent noise channel (the warp
 *       displacement), so the screen reads like two superimposed
 *       fluids rather than a single banded heightmap.
 *
 * Study alongside: ./perin_noise_flow_showcase.c — the FLOW-based
 *       Perlin showcase; this file focuses on STATIC-field renderings
 *       with progressively deeper warping. The same Perlin & FBM
 *       primitives are used, but the patterns here all answer the
 *       question "what happens if we recursively offset the noise
 *       input by other noise?". The two files together cover most of
 *       the practical Perlin-noise idioms.
 *
 * Section map:
 *   §1 config   — grid, patterns, palette, themes, named constants
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (4 palette colours each)
 *   §5 noise    — Noise context (perm table) + Perlin 2-D + fBm
 *   §6 patterns — Sample-returning RAW / WARP1 / WARP2 / WARP3 / RIDGE
 *   §7 scene    — Scene struct composing Grid, RenderBuffers,
 *                 SimState, Controls (+ Noise from §5); per-frame
 *                 grid-update pipeline
 *   §8 screen   — ASCII render: 10-step density ramp + HUD drawers
 *   §9 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new noise permutation, fresh field)
 *   n / N      next pattern
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster field drift (animation speeds up)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra domain_warped_noise_iq_style.c \
 *       -o iq_warp -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Domain warping. Given a noise function f(x) and another
 *                  noise function g(x), the warped value is f(x + g(x)).
 *                  Repeat: warp the warped — f(x + h(x + g(x))) — and you
 *                  get arbitrarily complex patterns from a few simple
 *                  Perlin lookups. Inigo Quilez popularised the technique
 *                  in graphics; the canonical "IQ warp" is two-level:
 *
 *                      q = (fbm(x),                fbm(x + 5.2, y + 1.3))
 *                      r = (fbm(x + 4·q + 1.7),    fbm(x + 4·q + 8.3))
 *                      f = fbm(x + 4·r)
 *
 *                  The constants 5.2, 1.3 etc. de-correlate the q and r
 *                  components so they don't track each other. The "× 4"
 *                  multipliers control how far the warp pushes the input
 *                  — larger values give wilder, more chaotic patterns.
 *
 *                  This file implements 5 patterns showing progressively
 *                  deeper warping (RAW → WARP1 → WARP2 → WARP3) plus a
 *                  RIDGE variant that applies (1 − 2|n − ½|)² to a
 *                  WARP2 value, producing sharp marble veins on a
 *                  smooth background instead of rolling blobs.
 *
 * Data-structure : Scene is the umbrella context, composed of five
 *                  sub-structs so each layer has ONE clear role and
 *                  no globals leak across them:
 *                    Grid          — w, h, total_cells + grid_idx().
 *                    Noise         — Perlin permutation table; passed
 *                                    by const pointer to every sampler
 *                                    so noise reads are obviously
 *                                    stateless.
 *                    RenderBuffers — per-cell glow + colour band; the
 *                                    complete render contract between
 *                                    the pattern layer (§6) and the
 *                                    screen layer (§8).
 *                    SimState      — field_time + supernova flash
 *                                    envelope. What the simulation
 *                                    mutates per tick.
 *                    Controls      — pattern, theme, drift, pause. What
 *                                    the keyboard mutates.
 *                  Each frame fully overwrites RenderBuffers by sampling
 *                  the active pattern at every cell. No particles, no
 *                  heap; everything BSS.
 *
 * Rendering      : ASCII only. Two INDEPENDENT noise signals drive
 *                  each cell:
 *                    INTENSITY → glyph from the 10-step Bourke ramp
 *                                " .:-=+*#%@" (blank to fully lit),
 *                                with A_BOLD on the top 5 levels for
 *                                a soft inner-glow effect.
 *                    COLOUR    → palette band 0..3 from a different
 *                                noise sample — for warp patterns the
 *                                warp-displacement x-component; for
 *                                RAW a far-offset fbm sample.
 *                  Decoupling intensity and colour is the trick that
 *                  separates "looks like a 1-D ramp" from "looks like
 *                  two flowing fluids superimposed". A perceptual
 *                  gamma curve (powf(·, 0.65)) is applied to intensity
 *                  to brighten the mid-tones so the field reads as
 *                  illuminated rather than muddy.
 *
 * Performance    : Worst pattern (WARP3) does ~13 FBM calls per cell ×
 *                  4 octaves per FBM = 52 perlin lookups per cell. On a
 *                  200×56 grid at 60 Hz that's ~35 M perlin lookups/sec
 *                  ≈ 1.5 % of one core on modern hardware. Comfortably
 *                  within budget for terminal rendering.
 *
 * References     : Algorithm (domain warping + Perlin + fBm)
 *                  ────────────────────────────────────────
 *                  • Inigo Quilez — "Domain warping". The canonical
 *                    article that introduced this technique to
 *                    graphics; contains both WARP1 and the two-level
 *                    WARP2 used here, plus the exact de-correlation
 *                    constants (5.2, 1.3, 4.0, 1.7, …):
 *                    https://iquilezles.org/articles/warp/
 *                  • Inigo Quilez — "fbm" (companion piece on
 *                    fractal noise stacks — the foundation that
 *                    domain warping builds on):
 *                    https://iquilezles.org/articles/fbm/
 *                  • Ken Perlin (2002) — "Improving Noise", SIGGRAPH.
 *                    Source of the quintic fade t·t·t·(t·(t·6−15)+10)
 *                    and the 8-direction gradient hash table used by
 *                    grad() / noise_perlin2d() in §5. The 2002 update
 *                    fixes the C¹-discontinuity artefacts of the 1985
 *                    original at integer lattice points.
 *                  • Ken Perlin (1985) — "An Image Synthesizer",
 *                    SIGGRAPH. The original noise paper — historical
 *                    context for the 2002 update. Introduces the
 *                    perm-table hashing scheme that survives intact
 *                    in our Noise struct.
 *                  • Ebert, Musgrave, Peachey, Perlin & Worley
 *                    — "Texturing & Modeling: A Procedural Approach"
 *                    (3rd ed, 2003, Morgan Kaufmann). The canonical
 *                    book on procedural noise; the fBm chapter is
 *                    the definitive treatment of lacunarity (= 2.0)
 *                    and persistence (= 0.5) — the two magic numbers
 *                    in noise_fbm()'s octave loop.
 *                  • Patricio Gonzalez Vivo & Jen Lowe — "The Book
 *                    of Shaders", ch. 13 (Generative Designs: fBm +
 *                    domain warping). Step-by-step interactive
 *                    tutorial with live shader demos — the gentlest
 *                    way to build intuition for what f(x + g(x))
 *                    actually does to a pattern:
 *                    https://thebookofshaders.com/13/
 *
 *                  Rendering (intensity → glyph + colour)
 *                  ──────────────────────────────────────
 *                  • Paul Bourke — "Character representation of grey
 *                    scale images":
 *                    https://paulbourke.net/dataformats/asciiart/
 *                    Source of the 10-step density ramp " .:-=+*#%@"
 *                    used in §8. Discusses ramp selection for
 *                    different fonts and target intensity ranges.
 *                  • Charles Poynton — "Gamma FAQ" (and the longer
 *                    "Frequently Asked Questions about Gamma").
 *                    Background for the perceptual gamma curve
 *                    applied in §6 (INTENSITY_GAMMA = 0.65). Explains
 *                    why uncorrected mid-range noise looks "muddy"
 *                    and how to invert monitor gamma.
 *
 *                  See also
 *                  ────────
 *                  • Compare ./perin_noise_flow_showcase.c — particle-
 *                    flow style Perlin demo. The same noise sampled
 *                    as motion vs. as a colour heightmap, side by
 *                    side, makes the relationship between the two
 *                    visualisation styles crisp.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Plain Perlin noise produces smooth rolling hills, but everything is
 * roughly axis-aligned and grid-shaped at the lattice level. Domain
 * warping breaks that bias by letting NOISE distort its own input
 * coordinates. Each level of warping multiplies the visual richness;
 * by the second level you have something that looks like swirled
 * marble, smoke, or fluid — patterns that no simple closed-form
 * formula could produce, but emerge naturally from f(x + g(x)).
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a plain noise pattern on a sheet of paper. Now imagine
 * grabbing the paper at random points and PULLING — every point
 * shifts by a noise-determined amount. The result is the same
 * underlying field, but distorted in a way that itself looks noisy.
 * Repeat the pull on the already-pulled paper and the distortion
 * compounds: smooth blobs become curves, curves become whorls,
 * whorls become chaotic eddies. WARP1, WARP2, WARP3 are exactly
 * this, applied 1, 2, and 3 times.
 *
 * RIDGE adds a different idea: instead of |noise| being smooth,
 * take 1 − |noise| so the SHARP transitions through zero become
 * the visible features. The result looks like marble veins or
 * cracked glass — sharp lines on a smooth background. Combined
 * with two-level warping, it's the closest a noise function gets
 * to looking hand-drawn.
 *
 * Visible layers:
 *   1. Density glyphs from a 10-step ramp " .:-=+*#%@". Drives the
 *      perceived BRIGHTNESS of each cell.
 *   2. Four-colour palette bands driven by an INDEPENDENT noise
 *      signal (the warp x-displacement, or a far-offset fbm sample
 *      for RAW). So the colour map "flows" separately from the
 *      brightness map — a key trick for visual depth.
 *   3. Slow drift — every pattern evolves over a few seconds as the
 *      noise field's "time axis" advances.
 *   4. Perceptual gamma curve on intensity so 0.5 noise reads as a
 *      bright '*' rather than a dim '.'.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ──────────────────
 *  1. Advance sim.field_time by FIELD_DRIFT × dt. This time slips
 *     into the y-coordinate of every Perlin lookup so the field
 *     appears to flow.
 *  2. For every cell (x, y):
 *     a. Sample the active pattern at (x, y, t). Result is a 2-tuple
 *        Sample{intensity, color}, both in [0, 1] and produced by
 *        DIFFERENT noise computations.
 *     b. gamma-boost intensity; quantise color into 4 bands.
 *  3. Render: glyph from glyph_ramp[intensity × 10]; pair from
 *     PAIR_BAND_BASE + band; A_BOLD if the glyph is in the upper
 *     half of the ramp.
 *  4. The user can press 'r' to reshuffle perm[] (a fresh noise
 *     field) and trigger a brief supernova flash for visual
 *     confirmation. There is no automatic reset — the field runs
 *     indefinitely until you ask for a new one.
 *
 * KEY FORMULAS
 * ────────────
 *  Perlin 2-D                   : as in ./perin_noise_flow_showcase.c
 *  FBM                           : Σᵢ aᵢ · perlin(2ⁱ x), with aᵢ = 1/2ⁱ
 *  WARP1 (1-level)              : f(x + a · g(x))
 *  WARP2 (2-level, IQ classic)  : f(x + b · h(x + a · g(x)))
 *  WARP3 (3-level)              : f(x + c · h₂(x + b · h(x + a · g(x))))
 *  RIDGE                         : (1 − 2 · |WARP2(x) − ½|)²   (sharpens)
 *  Drift offset                  : sample fbm(x, y + t), so increasing t
 *                                  effectively slides the field upward
 *                                  in noise-space.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DE-CORRELATION OFFSETS. The two halves of q (qx, qy) come from
 *    DIFFERENT noise samples — one at (x, y), the other at (x + 5.2,
 *    y + 1.3). If you use the SAME sample for both, qx == qy and
 *    the warp degenerates into a 1-D push along the diagonal.
 *    Always offset by an irrational-ish constant.
 *
 *  • WARP MAGNITUDE. The "× 4" in IQ's article assumes input
 *    coordinates in roughly [0, 1]. If your coords are pixel-scale
 *    you need to scale UP the warp by an equivalent factor or scale
 *    DOWN your noise input. We pre-multiply by NOISE_SCALE = 0.04 so
 *    the inputs land in roughly [0, 8] across the grid, and use
 *    WARP_AMOUNT = 4.0 directly.
 *
 *  • COMPUTE COST GROWS GEOMETRICALLY. WARP3 needs 13 FBM calls per
 *    cell vs RAW's 1. On a slow terminal or a much bigger grid this
 *    becomes the bottleneck. Reduce FBM_OCTAVES or downsample the
 *    grid if you go past 200×60.
 *
 *  • RIDGE INTENSITY MAPPING. (1 − 2|n − ½|)² peaks at n = ½ and is 0
 *    at n = 0 or 1; the squaring then thins the bright tier so only
 *    the very middle survives. Bright cells trace the half-noise
 *    contour — that's the marble vein. Colour along each vein comes
 *    from the warp's qx channel (see §6), so the SAME vein can shift
 *    colour along its length — also correct, not a bug.
 *
 *  • DRIFT SPEED VS COMPUTE COST. With WARP3 active and drift_mult
 *    cranked up via '+', the field updates faster than is comfortable
 *    to watch. Halve drift_mult with '-', or pause briefly with space.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Cycle to RAW first; the pattern should look like rolling
 *    cellular blobs — no swirls, no cracks. If you see swirls in
 *    RAW, the FBM is silently warping somewhere.
 *  • Switch to WARP1 — pattern visibly distorts, blobs start curving.
 *  • WARP2 — distinct swirly/marble look. This is the IQ canonical
 *    output; should be visibly more chaotic than WARP1.
 *  • WARP3 — even more eddies. The difference WARP2 → WARP3 is
 *    subtler than WARP1 → WARP2 (diminishing returns).
 *  • RIDGE — sharp light-coloured ridges on a darker background;
 *    veined like marble.
 *  • All patterns should evolve smoothly when you wait — drift is
 *    working. If frozen, FIELD_DRIFT is 0 or scene_tick is paused.
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
    PAIR_BAND_BASE    =   3,    /* base of N_BANDS contiguous palette pairs */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* Decay for the supernova flash on reset. */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Noise scale: ~0.04 noise-units per cell → about 8 noise-units
 * across the whole 200-wide grid → ~3-4 visible "feature periods". */
#define NOISE_SCALE         0.04f

/* Field drift in noise-coord units per second. Slow drift so the
 * pattern visibly evolves but doesn't churn. */
#define FIELD_DRIFT         0.10f

/* Drift multiplier — cranked by +/-, capped to keep CPU happy. */
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* FBM stack parameters. PERSISTENCE is the amplitude ratio between
 * successive octaves; LACUNARITY is the frequency ratio. The
 * canonical "1/f noise" values are 0.5 and 2.0 — each octave is
 * half as loud and twice as dense as the previous, producing the
 * fractal scaling. See Ebert et al., fBm chapter. */
#define FBM_OCTAVES         4
#define FBM_PERSISTENCE     0.5f
#define FBM_LACUNARITY      2.0f

/* Domain-warp magnitude. IQ's canonical value is 4.0 for inputs in
 * [0, 1]; we apply it after multiplying inputs by NOISE_SCALE so the
 * effective push lands in the right perceptual range. */
#define WARP_AMOUNT         4.0f

/* De-correlation offsets — the constants that prevent two halves of
 * the warp vector from being perfectly correlated. Pulled from IQ's
 * article. Don't change these unless you want a different look. */
#define DECOR_QY_X          5.2f
#define DECOR_QY_Y          1.3f
#define DECOR_RX_X          1.7f
#define DECOR_RX_Y          9.2f
#define DECOR_RY_X          8.3f
#define DECOR_RY_Y          2.8f
#define DECOR_SX_X          7.5f
#define DECOR_SX_Y          3.1f
#define DECOR_SY_X          6.8f
#define DECOR_SY_Y          4.4f

/* ── Named tunables ─────────────────────────────────────────────────── *
 * Single-value tunables grouped here so §1 lists every dial the
 * program exposes. Names convey each value's algorithmic or display
 * role so the call sites stay self-documenting. */

/* §7 supernova — initial flash glow value on reset; decays via
 * SUPERNOVA_DECAY each tick until below GLOW_THRESHOLD. */
#define SUPERNOVA_FLASH_INIT     1.0f

/* §7 banding — quantise colour ∈ [0, 1) → palette band 0..N_BANDS-1.
 * Scale sits just under N_BANDS so a colour value of 1.0 doesn't
 * overflow into band N_BANDS (which has no colour pair). */
#define N_BANDS                  4
#define BAND_QUANTIZE_SCALE      3.999f

/* §6 RAW colour decorrelation — how far to push the colour-channel
 * fbm sample away from the intensity-channel sample so the two read
 * as uncorrelated patterns. 4× the q-warp offsets puts them well
 * beyond one feature period in noise-space. */
#define RAW_COLOR_DECOR_FACTOR   4.0f

/* §8 supernova twinkle — sparse-mask period for the post-reset flash.
 * (x ^ y) & MASK == 0 lights one cell in (MASK + 1), giving a sparse
 * star-field look rather than a uniform white-out. */
#define SUPERNOVA_SPARSE_MASK    3

/* §8 glyph banding — quantise intensity ∈ [0, 1) → glyph_ramp index
 * 0..N_GLYPHS-1. Same trick as BAND_QUANTIZE_SCALE but for the
 * density ramp: sits just under N_GLYPHS so intensity == 1.0
 * doesn't overflow into N_GLYPHS (which would index past the
 * ramp). */
#define GLYPH_QUANTIZE_SCALE     (N_GLYPHS - 0.001f)

/* ── Density ramp ────────────────────────────────────────────────── *
 * Paul Bourke's 10-step ASCII grey-scale ramp. Maps a normalised
 * intensity in [0, 1] linearly onto one of these 10 glyphs:
 *   0 ' '  blank          5 '+'  ← A_BOLD from here up
 *   1 '.'  faint dot      6 '*'
 *   2 ':'  pair           7 '#'
 *   3 '-'  dash           8 '%'
 *   4 '='  double dash    9 '@'  brightest
 *
 * Ten steps is the minimum for the field to read as smoothly shaded
 * at terminal cell-size; coarser ramps look posterised at any
 * nontrivial noise variance.
 */
static const char glyph_ramp[] = " .:-=+*#%@";
#define N_GLYPHS         10
#define GLYPH_BOLD_FROM   5    /* indices ≥ this draw A_BOLD          */

/* Perceptual gamma applied to raw noise intensity before glyph pick.
 * Values < 1 push midtones brighter — a 0.5 sample becomes a clearly-
 * lit '*' (index 6) rather than a dim '.' (index 4). Standard CRT-era
 * monitor gamma is 2.2; the inverse (~0.45) is too bright; 0.65 is a
 * tuned middle that keeps the contrast without washing out the highs. */
#define INTENSITY_GAMMA  0.65f

/* ── HUD layout ──────────────────────────────────────────────────── *
 * Top HUD carries DATA, bottom HUD carries ACTIONS:
 *   row 0           : title + state bar (fps, Hz, state, drift)
 *   row 1           : pattern, theme, palette swatch, noise params
 *   row 2           : ramp legend " .:-=+*#%@  dim → bright"
 *   row HUD_TOP..N-2: playable field
 *   row N-1         : keyboard action hint
 */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      18    /* " pattern:XXXXXXX " column     */
#define HUD_THEME_FIELD_W        17    /* " theme:XXXXXXXX " column      */
#define HUD_PALETTE_LABEL_W       9    /* " palette:" prefix             */
#define HUD_PALETTE_SWATCH_N      4    /* one '#' per band               */

/*
 * Pattern — five domain-warp variants, cycle with n/p.
 *
 *   RAW    : plain FBM, no warp — the baseline you compare against
 *   WARP1  : single warp pass, f(x + a·g(x))
 *   WARP2  : two warp passes (IQ classic), f(x + b·h(x + a·g(x)))
 *   WARP3  : three warp passes — the deepest nesting
 *   RIDGE  : WARP2 with the ridged transform 1 − 2|n − ½|, sharp
 *            veins instead of smooth blobs
 */
typedef enum {
    PATTERN_RAW   = 0,
    PATTERN_WARP1 = 1,
    PATTERN_WARP2 = 2,
    PATTERN_WARP3 = 3,
    PATTERN_RIDGE = 4,
    N_PATTERNS    = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_RAW:   return "RAW    ";
    case PATTERN_WARP1: return "WARP1  ";
    case PATTERN_WARP2: return "WARP2  ";
    case PATTERN_WARP3: return "WARP3  ";
    case PATTERN_RIDGE: return "RIDGE  ";
    default:            return "?      ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* §9 main loop — spiral-of-death dt clamp + frame rate cap. The DT
 * cap is the standard guard from Glenn Fiedler's "Fix Your Timestep"
 * — if the frame stalled for > 100 ms, pretend it didn't, so the
 * sim doesn't try to catch up with hundreds of ticks. */
#define DT_MAX_NS                (100 * NS_PER_MS)
#define FRAME_CAP_FPS            60

/*
 * Theme — a complete colour palette for one visual style. Ten of
 * these live in themes[], cycled by t/T. The structure carries the
 * smallest data needed to recolour the entire program: N_BANDS
 * ramp colours and one flash highlight.
 *
 * INTENT. Each cell in RenderBuffers holds a colour band 0..3,
 * picked by quantising Sample.color — the colour channel that flows
 * INDEPENDENTLY of intensity (see §6). The band indexes band[i] to
 * choose the foreground colour. Keeping exactly N_BANDS = 4 colours
 * forces theme authors to design the ramp as a coherent low → high
 * progression rather than a random scatter — the same discipline
 * as a Houdini ramp parameter or a Substance Designer gradient.
 *
 * UI CHROME. PAIR_HUD / PAIR_HINT / PAIR_SUPERNOVA stay theme-
 * independent so the HUD remains legible against any palette;
 * theme_apply() never touches them.
 *
 * COLOUR FORMAT. xterm-256 indices (NOT RGB). When the terminal
 * exposes fewer than 256 colours, theme_apply() falls back to a
 * fixed 8-colour cycle so the demo still runs on legacy TTYs.
 */
typedef struct {
    const char *name;            /* short uppercase label shown in HUD       */
    short       band[N_BANDS];   /* ramp colours: 0 = dim/low, 3 = bright    */
    short       flash;           /* supernova / reset flash highlight colour */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },   /* navy → sky → gold → white  */
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },   /* greens                     */
    { "NOVA",    {  53,  129,  201,  219 }, 226 },   /* purple → pink → magenta    */
    { "MONO",    { 234,  244,  250,  254 }, 226 },   /* greyscale                  */
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },   /* navy → bright cyan         */
    { "FIRE",    {  52,  124,  208,  226 }, 196 },   /* dark red → yellow          */
    { "EARTH",   {  58,  100,  173,  230 }, 226 },   /* brown → cream              */
    { "FOREST",  {  22,   28,   64,  144 }, 226 },   /* greens                     */
    { "DESERT",  {  94,  130,  173,  222 }, 226 },   /* sandy                      */
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },   /* navy → ice → white         */
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
/* §5  noise — Noise context + Perlin 2-D + FBM                           */
/* ===================================================================== */

/*
 * Noise — the entire random-source state of the program. Wrapping
 * the permutation table in a struct means there is exactly one piece
 * of mutable noise data, it lives inside Scene, and every sampler
 * takes a `const Noise *` so reads are obviously stateless.
 *
 * ALGORITHM. Classic Perlin gradient noise. The permutation table is
 * a random shuffle of [0..255] used to hash 2-D lattice coordinates
 * into gradient indices. Re-shuffling perm[] yields a completely new
 * noise field — triggered on demand by the 'r' key (manual reset).
 *
 * REFERENCE. Ken Perlin (2002) — "Improving Noise" SIGGRAPH paper,
 * which describes both the perm-table hashing scheme and the
 * 8-direction gradient table used by grad() below. The 1985 original
 * used a cubic fade with visible C¹-discontinuity artefacts at
 * integer lattice points; we use the 2002 quintic fade.
 */
typedef struct {
    /* Doubled permutation: perm[i + 256] == perm[i] for i in [0..255].
     * The doubling lets noise_perlin2d() index perm[X+1] without a
     * modulo. X is already masked to 0..255, so X+1 is at most 256 —
     * safely inside the doubled buffer. Classic micro-opt from
     * Perlin's reference Java code; saves one AND per lookup,
     * roughly 10 % of the inner-loop cost in profiling. WARP3 makes
     * ~52 perlin lookups per cell, so this matters at full grid. */
    uint8_t perm[512];
} Noise;

/* fisher_yates_shuffle_256 — unbiased uniform random shuffle of
 * [0..255]. Textbook Fisher-Yates: walk from the end backwards,
 * swap each element with a random earlier element. O(n). */
static void fisher_yates_shuffle_256(uint8_t base[256])
{
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
}

/* mirror_perm_for_double_lookup — copy a 256-byte permutation into
 * a 512-byte buffer twice (upper half mirrors the lower). This
 * removes a modulo from noise_perlin2d when indexing perm[X+1]. */
static void mirror_perm_for_double_lookup(uint8_t perm[512],
                                          const uint8_t base[256])
{
    for (int i = 0; i < 256; i++) {
        perm[i]       = base[i];
        perm[i + 256] = base[i];
    }
}

static void noise_shuffle(Noise *n)
{
    uint8_t base[256];
    fisher_yates_shuffle_256      (base);
    mirror_perm_for_double_lookup (n->perm, base);
}

/* Perlin 2002 quintic fade: 6t⁵ − 15t⁴ + 10t³. C² at lattice edges. */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

/* grad — one of 8 gradient·(x, y) dot products selected by the low
 * 3 bits of `hash`. Perlin 2002's table of 8 directions: ±1·u ± 2·v
 * where (u, v) ∈ {(x, y), (y, x)}. */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * noise_perlin2d — sample classic Perlin gradient noise at (x, y).
 * Output range ≈ [-1, 1]. Five algorithmic steps:
 *   (1) Find the integer lattice cell + fractional offset.
 *   (2) Apply the quintic fade — C² across cell boundaries.
 *   (3) Hash the 4 corner indices via the perm table.
 *   (4) gradient · (offset from corner to sample point) at each.
 *   (5) Bilinear interpolation using the faded weights.
 */
static float noise_perlin2d(const Noise *n, float x, float y)
{
    /* (1) Lattice cell + fractional offset. The & 255 wrap gives the
     *     noise a period of 256 lattice cells. */
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    /* (2) Quintic fade for smooth derivatives at lattice borders. */
    float u = fade(x);
    float v = fade(y);

    /* (3) Hash the 4 corner indices via the perm table. A and B are
     *     row-hashes for the left and right edges of the cell. */
    int A = n->perm[X    ] + Y;
    int B = n->perm[X + 1] + Y;

    /* (4) Gradient · offset at each of the 4 corners. */
    float n00 = grad(n->perm[A    ], x,        y       );   /* corner (0,0) */
    float n10 = grad(n->perm[B    ], x - 1.0f, y       );   /* corner (1,0) */
    float n01 = grad(n->perm[A + 1], x,        y - 1.0f);   /* corner (0,1) */
    float n11 = grad(n->perm[B + 1], x - 1.0f, y - 1.0f);   /* corner (1,1) */

    /* (5) Bilinear interpolation with the faded weights. */
    return lerp_f(lerp_f(n00, n10, u),
                  lerp_f(n01, n11, u), v);
}

/*
 * noise_fbm — fractional Brownian motion stack. Sums FBM_OCTAVES
 * copies of Perlin noise, each at twice the frequency and half the
 * amplitude of the previous. Result ≈ [-1, 1] after normalisation
 * by Σamp. See Ebert et al., fBm chapter, for the canonical
 * derivation of lacunarity and persistence parameters.
 */
static float noise_fbm(const Noise *n, float x, float y)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * noise_perlin2d(n, x * freq, y * freq);
        max_amp += amp;
        amp     *= FBM_PERSISTENCE;
        freq    *= FBM_LACUNARITY;
    }
    return total / max_amp;
}

/* ===================================================================== */
/* §6  patterns — 5 domain-warp variants                                  */
/* ===================================================================== */

/*
 * Sample — what every pattern returns. TWO scalars per cell, both in
 * [0, 1] but coming from DIFFERENT noise computations:
 *
 *   intensity → drives the density glyph from glyph_ramp
 *   color     → drives the palette band 0..N_BANDS-1
 *
 * Decoupling these is the visual-richness trick. With a single value
 * driving both, every cell's glyph and colour move together (low
 * glow = dark colour, high glow = bright colour) — the screen reads
 * as a 1-D heightmap with posterised steps. With two channels, the
 * brightness pattern and the colour pattern flow INDEPENDENTLY, so
 * you see one fluid layered over another. That's the difference
 * between "noise printout" and "stunning".
 *
 * GAMMA POLICY. The `intensity` slot is the gamma-corrected value
 * that scene_update_grid writes directly into RenderBuffers.glow;
 * each pattern is responsible for applying gamma_boost (or, for
 * RIDGE, the squaring transform that replaces it). `color` is
 * always raw noise in [0, 1] — colours don't get perceptual
 * correction because bands are quantised, so a smoother input
 * curve has no observable effect downstream.
 */
typedef struct {
    /* Post-curve intensity ∈ [0, 1] → glyph_ramp index. The pattern
     * that produced this Sample has already applied the appropriate
     * non-linear curve: gamma_boost(·) for RAW / WARP1 / WARP2 /
     * WARP3, and the (1 − 2|n − ½|)² ridge transform for RIDGE. The
     * render layer treats this as the final perceived brightness
     * and just looks up the glyph_ramp index. */
    float intensity;

    /* Raw colour noise ∈ [0, 1] → palette band via
     * quantize_color_to_band(). Sourced from an independent noise
     * channel: warp x-displacement (qx) for the warp patterns, a
     * far-offset fbm for RAW. NOT gamma-corrected. */
    float color;
} Sample;

/*
 * gamma_boost — perceptual brightness curve. INTENSITY_GAMMA < 1
 * pulls mid-range noise values up toward "lit" without crushing the
 * highlights. Equivalent to the inverse of monitor gamma — the
 * standard trick for translating raw sim values into something the
 * eye reads as illumination.
 */
static inline float gamma_boost(float v)
{
    return powf(v, INTENSITY_GAMMA);
}

/* remap_signed_to_unit — convert a noise-range value in [-1, 1] to
 * the unit interval [0, 1]. The single most common output transform
 * for any noise-based renderer; appears once per pattern. */
static inline float remap_signed_to_unit(float v)
{
    return v * 0.5f + 0.5f;
}

/*
 * Vec2 — a 2-D vector. Used inside the warp pipeline so layered
 * formulas read like vector ops instead of two loose floats. The
 * pipeline is conceptually:
 *
 *   base position  →  layer 1 warp vector (q)
 *                  →  base + W·q  →  layer 2 warp vector (r)
 *                                 →  base + W·r  →  …
 *
 * Each arrow is one Vec2 function call below.
 */
typedef struct { float x, y; } Vec2;

/*
 * WarpOffsets — the four de-correlation offsets that parameterise
 * one warp layer. Each layer samples TWO fbm values (one per output
 * channel); these offsets shift the input coordinates of each
 * channel so the channels read as independent rather than as two
 * copies of the same scalar (Quilez 2008, "Domain warping").
 *
 *   dx_x, dx_y → offsets for the x-channel sample
 *   dy_x, dy_y → offsets for the y-channel sample
 *
 * The exact values come from IQ's article; any non-zero, non-equal
 * pair works.
 */
typedef struct {
    float dx_x, dx_y;
    float dy_x, dy_y;
} WarpOffsets;

/* Three warp layers, each with its own de-correlation offsets. The
 * x-channel of the first layer (Q) uses zero offset so the noise
 * coordinate IS the input position; subsequent layers offset both
 * channels to keep them independent of the previous layer too. */
static const WarpOffsets Q_OFFSETS = { 0.0f,       0.0f,       DECOR_QY_X, DECOR_QY_Y };
static const WarpOffsets R_OFFSETS = { DECOR_RX_X, DECOR_RX_Y, DECOR_RY_X, DECOR_RY_Y };
static const WarpOffsets S_OFFSETS = { DECOR_SX_X, DECOR_SX_Y, DECOR_SY_X, DECOR_SY_Y };

/*
 * sample_warp_layer — read one 2-channel warp displacement at
 * `base`. The two channels come from DIFFERENT noise positions
 * (shifted by `o`) so the result is a true (x, y) vector field
 * rather than two copies of one scalar. Time `t` drifts the
 * y-axis sample coord, animating the warp.
 */
static Vec2 sample_warp_layer(const Noise *n, Vec2 base, WarpOffsets o, float t)
{
    return (Vec2){
        .x = noise_fbm(n, base.x + o.dx_x, base.y + o.dx_y + t),
        .y = noise_fbm(n, base.x + o.dy_x, base.y + o.dy_y + t),
    };
}

/*
 * warp_position — apply WARP_AMOUNT × v to a base position. THIS
 * is the actual "domain warp" step: shift the input coordinates of
 * the NEXT layer by the previous layer's output vector. The
 * algorithm's defining operation.
 */
static inline Vec2 warp_position(Vec2 base, Vec2 v)
{
    return (Vec2){
        base.x + WARP_AMOUNT * v.x,
        base.y + WARP_AMOUNT * v.y,
    };
}

/*
 * sample_final_intensity — sample fBm at the warped position with a
 * time-drifted y-coord. "Final" because this is the last fbm call
 * in the warp chain — its output IS the cell's intensity (before
 * any per-pattern remap / gamma / ridge transform).
 */
static inline float sample_final_intensity(const Noise *n, Vec2 p, float t)
{
    return noise_fbm(n, p.x, p.y + t);
}

/*
 * RAW — plain fBm for intensity, a FAR-OFFSET fbm sample for colour.
 * Both sampled at the SAME position (no warp), but the colour sample
 * is pushed RAW_COLOR_DECOR_FACTOR × the q-warp offsets away in
 * noise-space, so it reads as an independent cloud pattern. The
 * baseline you compare WARP1/2/3 against to see how much the warps
 * actually distort.
 */
static Sample pattern_raw(const Noise *n, float x, float y, float t)
{
    Vec2 intensity_pos = { x,                                       y };
    Vec2 color_pos     = { x + RAW_COLOR_DECOR_FACTOR * DECOR_QY_X,
                           y + RAW_COLOR_DECOR_FACTOR * DECOR_QY_Y };

    float intensity = sample_final_intensity(n, intensity_pos, t);
    float color     = sample_final_intensity(n, color_pos,     t);

    return (Sample){
        .intensity = gamma_boost(remap_signed_to_unit(intensity)),
        .color     = remap_signed_to_unit(color),
    };
}

/*
 * WARP1 — single domain warp. Sample the q-layer warp vector, use it
 * to displace the input, sample fBm at the warped position. Colour
 * comes from the q-layer's x-channel so it varies independently of
 * intensity — the screen reads as two flowing patterns superimposed.
 */
static Sample pattern_warp1(const Noise *n, float x, float y, float t)
{
    Vec2 base = { x, y };

    Vec2 q          = sample_warp_layer    (n, base, Q_OFFSETS, t);
    float intensity = sample_final_intensity(n, warp_position(base, q), t);

    return (Sample){
        .intensity = gamma_boost(remap_signed_to_unit(intensity)),
        .color     = remap_signed_to_unit(q.x),
    };
}

/*
 * warp2_raw — shared helper for WARP2 and RIDGE. Two-layer warp
 * pipeline written as named pseudocode:
 *
 *     q ← sample warp at base
 *     r ← sample warp at (base displaced by q)
 *     v ← sample final intensity at (base displaced by r)
 *
 * Returns intensity in [0, 1] BEFORE gamma so RIDGE can substitute
 * its own (sharper) curve. Colour comes from q.x — the first warp
 * layer carries the largest-scale visual variation, so it's the
 * most legible channel to drive colour with.
 */
static Sample warp2_raw(const Noise *n, float x, float y, float t)
{
    Vec2 base = { x, y };

    Vec2 q          = sample_warp_layer    (n, base,                       Q_OFFSETS, t);
    Vec2 r          = sample_warp_layer    (n, warp_position(base, q),     R_OFFSETS, t);
    float intensity = sample_final_intensity(n, warp_position(base, r), t);

    return (Sample){
        .intensity = remap_signed_to_unit(intensity),
        .color     = remap_signed_to_unit(q.x),
    };
}

/* WARP2 — IQ's canonical two-level marble look. warp2_raw does the
 * work; we just apply the standard gamma curve to intensity. */
static Sample pattern_warp2(const Noise *n, float x, float y, float t)
{
    Sample s = warp2_raw(n, x, y, t);
    s.intensity = gamma_boost(s.intensity);
    return s;
}

/*
 * WARP3 — three-level warp. Same pipeline as warp2 with one more
 * displacement layer (s) wrapped around the outside. Colour comes
 * from s.x rather than q.x so it visually tracks the deepest
 * distortion layer instead of the smoothest — gives WARP3 its
 * busier colour map.
 */
static Sample pattern_warp3(const Noise *n, float x, float y, float t)
{
    Vec2 base = { x, y };

    Vec2 q          = sample_warp_layer    (n, base,                       Q_OFFSETS, t);
    Vec2 r          = sample_warp_layer    (n, warp_position(base, q),     R_OFFSETS, t);
    Vec2 s          = sample_warp_layer    (n, warp_position(base, r),     S_OFFSETS, t);
    float intensity = sample_final_intensity(n, warp_position(base, s), t);

    return (Sample){
        .intensity = gamma_boost(remap_signed_to_unit(intensity)),
        .color     = remap_signed_to_unit(s.x),
    };
}

/*
 * RIDGE — sharpened veins. Take 1 − 2|n − ½| over the underlying
 * warp2 value so the intensity peaks on the half-noise contour, then
 * SQUARE the result to thin the ridges and brighten the cores. Looks
 * like marble veins or cracked stone. Colour reuses warp2's qx so
 * the veins themselves carry colour variation along their length.
 *
 * Note: no gamma here. The squaring already remaps the distribution,
 * and a gamma boost on top would muddy the dark background.
 */
static Sample pattern_ridge(const Noise *n, float x, float y, float t)
{
    Sample s = warp2_raw(n, x, y, t);    /* raw, pre-gamma */
    float v = s.intensity - 0.5f;
    float ridge = 1.0f - 2.0f * fabsf(v);
    ridge = ridge * ridge;                /* sharpen */
    return (Sample){
        .intensity = ridge,
        .color     = s.color,
    };
}

/* sample_pattern — dispatcher. The single seam where Pattern values
 * become a Sample; all of §7's grid loop is agnostic to which
 * pattern is active. */
static Sample sample_pattern(const Noise *n, Pattern p,
                              float fx, float fy, float t)
{
    switch (p) {
    case PATTERN_RAW:   return pattern_raw  (n, fx, fy, t);
    case PATTERN_WARP1: return pattern_warp1(n, fx, fy, t);
    case PATTERN_WARP2: return pattern_warp2(n, fx, fy, t);
    case PATTERN_WARP3: return pattern_warp3(n, fx, fy, t);
    case PATTERN_RIDGE: return pattern_ridge(n, fx, fy, t);
    default:            return (Sample){ 0.0f, 0.0f };
    }
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The scene is built out of five small structs. Each owns ONE
 * concern; Scene composes them. Splitting concerns into clearly-
 * typed sub-structs makes function signatures self-describing: any
 * function that takes `const Grid *` clearly cannot mutate buffers;
 * any function that takes `RenderBuffers *` clearly does not sample
 * noise; and so on.
 */

/*
 * Grid — map geometry. Pure data: no buffers, no state, no
 * allocation. Lives at the top of Scene because every layer (noise
 * sampling, screen centring, the pattern loop) needs the dimensions.
 *
 * INDEXING. Row-major: cell (x, y) → y·w + x. This matches the
 * memory layout of RenderBuffers (single flat array of CELLS_MAX
 * cells), so the y-outer / x-inner loop order in scene_update_grid
 * and scene_draw is cache-friendly — each row is contiguous.
 *
 * INVARIANT. w · h ≤ CELLS_MAX always holds; app_pick_map_size()
 * clamps to MAP_W_MAX × MAP_H_MAX. The clamp is enforced once per
 * resize; downstream code may assume it.
 */
typedef struct {
    int w, h;         /* current map width / height in cells              */
    int total_cells;  /* = w · h, cached so hot loops skip the multiply   */
} Grid;

static inline int grid_idx(const Grid *g, int x, int y) { return y * g->w + x; }

/*
 * RenderBuffers — the per-cell raster output. Two parallel arrays
 * indexed by grid_idx(g, x, y):
 *   glow  : intensity 0..1 → glyph index in glyph_ramp
 *   color : palette band 0..N_BANDS-1
 *
 * The screen layer reads ONLY these two arrays. Patterns write into
 * them via write_cell. That is the entire render contract — no
 * other channel of communication between simulation and display.
 * Splitting render output from simulation state is the classic
 * graphics-pipeline decoupling (Foley & van Dam, ch. 2).
 *
 * SoA RATIONALE. Two separate arrays (struct-of-arrays) rather than
 * one array of cell-structs because (a) the screen layer reads glow
 * far more often than color, so keeping it dense improves cache use,
 * and (b) clearing one channel without touching the other is trivial.
 */
typedef struct {
    /* Per-cell intensity 0..1, drives the density glyph from
     * glyph_ramp. Below GLOW_THRESHOLD (0.05) the cell renders as
     * blank. scene_update_grid overwrites this each frame — there
     * is no decay, since every pattern fully repaints the field. */
    float   glow [CELLS_MAX];

    /* Per-cell palette band 0..N_BANDS-1. Indexes into the current
     * Theme's band[] via PAIR_BAND_BASE + color. Masked with
     * (N_BANDS - 1) in the draw loop so any out-of-range value
     * cannot pick a wrong pair — defence in depth. */
    uint8_t color[CELLS_MAX];
} RenderBuffers;

static void buffers_clear(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow [i] = 0.0f;
        b->color[i] = 0;
    }
}

/*
 * SimState — values that evolve per tick under the simulation's own
 * control. Separated from Controls (keyboard-driven knobs) so it is
 * obvious at a glance what scene_tick mutates vs. what the user
 * does. Two small fields, each playing a distinct role:
 *
 *   field_time       → animates the noise field by drifting the
 *                      y-axis sample coordinate. Without it the
 *                      potential would be static and every frame
 *                      would show the same field. With it the
 *                      vortex/marble structure slowly mutates
 *                      without changing the underlying perm[].
 *   supernova_glow_t → cosmetic flourish triggered by scene_reset()
 *                      (manual 'r' or initial start). Decays
 *                      exponentially via SUPERNOVA_DECAY so it
 *                      vanishes within ~1 second.
 */
typedef struct {
    /* Time offset added to the noise y-coordinate before sampling.
     * Increment per tick is FIELD_DRIFT × drift_mult × dt. Units are
     * "noise-units" — the same scale as fx, fy in scene_update_grid.
     * Cleared to 0 by scene_reset(); otherwise grows monotonically. */
    float field_time;

    /* Envelope for the post-reset sparkle. Set to SUPERNOVA_FLASH_INIT
     * on reset, decays per tick by expf(-SUPERNOVA_DECAY * dt). While
     * the envelope is above GLOW_THRESHOLD, scene_draw paints a sparse
     * '*' field masked by (x ^ y) & SUPERNOVA_SPARSE_MASK so the
     * flash looks like a random twinkle rather than a uniform blanket. */
    float supernova_glow_t;
} SimState;

/*
 * Controls — user-facing knobs. Mutated by app_handle_key(), read by
 * scene_tick() and screen_draw(). Grouping them makes the keyboard
 * handler trivial: `Controls *c = &scene.ctrl;` once at the top,
 * then every key case is a one-line mutation on `c`.
 *
 * The split between SimState and Controls is the "model vs. user
 * intent" line common to interactive programs: SimState answers
 * "where is the simulation right now"; Controls answers "what has
 * the user asked for". scene_tick reads Controls but never writes
 * to it. The keyboard handler writes to Controls but never to
 * SimState. This one-way dependency keeps the code linear.
 */
typedef struct {
    /* Gate for scene_tick: when true the field is frozen but the
     * render loop continues so the HUD stays responsive. */
    bool    paused;

    /* Multiplier on FIELD_DRIFT — speeds / slows the noise
     * animation. Doubled by '+', halved by '-', clamped to
     * [DRIFT_MULT_MIN, DRIFT_MULT_MAX] = [1, 32]. The doubling step
     * gives a logarithmic feel: each press is the "same size change"
     * regardless of current speed. */
    int     drift_mult;

    /* Index into themes[N_THEMES]. Mutated by 't'/'T' via
     * cycle_theme(); applied immediately to the colour pairs by
     * theme_apply(). The HUD reads themes[current_theme].name. */
    int     current_theme;

    /* The active visualisation; mutated by 'n'/'p' via cycle_pattern.
     * Unlike particle-based demos there is no `prev_pattern` edge
     * detector — every pattern fully overwrites RenderBuffers in
     * scene_update_grid, so switching never leaves ghost cells from
     * the previous pattern. */
    Pattern current_pattern;
} Controls;

/*
 * Scene — the umbrella context. Reading this struct top-to-bottom
 * is meant to be the fastest way to understand the program:
 *   grid       → where things live              (geometry)
 *   noise      → what the field is sampled from (perm table)
 *   buf        → what gets drawn                (render output)
 *   sim        → animation state                (drift, supernova)
 *   ctrl       → user knobs                     (pattern, drift, …)
 *
 * ORDERING. The field order is deliberate — each sub-struct only
 * depends on those declared above it. grid is leaf-level data;
 * noise is independent of grid; buf is sized by CELLS_MAX (an upper
 * bound on grid); sim mutates noise + buf via scene_tick; ctrl
 * decides which sim path runs. A reader scanning top-down meets
 * every concept before it is used.
 *
 * COMPOSITION. There is no Scene-wide invariant that crosses sub-
 * struct boundaries — each sub-struct can be reasoned about (and
 * tested) in isolation. The only function that sees all of Scene
 * at once is scene_tick() and scene_draw(). This is composition
 * over inheritance: no virtual dispatch — just five clearly named
 * structs glued together by direct field access.
 *
 * REFERENCE. Mike Acton — "Data-Oriented Design and C++" (CppCon
 * 2014) for the broader argument that good struct layout IS good
 * code.
 */
typedef struct {
    Grid          grid;       /* immutable per frame (only resize mutates)  */
    Noise         noise;      /* mutated only by scene_reset (shuffle)      */
    RenderBuffers buf;        /* written by patterns, read by scene_draw    */
    SimState      sim;        /* mutated only by scene_tick                 */
    Controls      ctrl;       /* mutated only by app_handle_key             */
} Scene;

/* ── grid-update pipeline ─────────────────────────────────────────── *
 * scene_update_grid is the per-frame pseudocode:
 *
 *     for each cell (x, y):
 *         fx, fy = cell × NOISE_SCALE
 *         sample = sample_pattern(noise, active_pattern, fx, fy, t)
 *         clamp sample to [0, 1]
 *         write glyph + colour band into RenderBuffers
 *
 * Each step below is one named helper. */

static inline float clamp_unit(float v)
{
    return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

/* quantize_color_to_band — map a continuous colour value in [0, 1)
 * to a palette band 0..N_BANDS-1. BAND_QUANTIZE_SCALE sits just
 * under N_BANDS so a value of 1.0 doesn't overflow; the trailing
 * mask is defence in depth. */
static inline int quantize_color_to_band(float color)
{
    return (int)(color * BAND_QUANTIZE_SCALE) & (N_BANDS - 1);
}

/* write_cell — commit one Sample into the render buffer at idx. */
static inline void write_cell(RenderBuffers *b, int idx, Sample s)
{
    b->glow [idx] = clamp_unit(s.intensity);
    b->color[idx] = (uint8_t)quantize_color_to_band(clamp_unit(s.color));
}

static void scene_update_grid(Scene *s)
{
    const Grid    *g  = &s->grid;
    const Noise   *n  = &s->noise;
    RenderBuffers *b  = &s->buf;
    Pattern        p  = s->ctrl.current_pattern;
    float          t  = s->sim.field_time;

    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            Sample sample = sample_pattern(n, p, fx, fy, t);
            write_cell(b, grid_idx(g, x, y), sample);
        }
    }
}

/* ── reset / init pipeline ────────────────────────────────────────── */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->field_time       = 0.0f;
    sim->supernova_glow_t = SUPERNOVA_FLASH_INIT;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions(&s->grid, w, h);
    reset_sim_state      (&s->sim);
    buffers_clear        (&s->buf, s->grid.total_cells);
    noise_shuffle        (&s->noise);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused          = false;
    s->ctrl.drift_mult      = DRIFT_MULT_DEF;
    s->ctrl.current_theme   = 0;
    s->ctrl.current_pattern = PATTERN_WARP2;   /* IQ classic by default */
    scene_reset(s, w, h);
}

/* ── tick pipeline ────────────────────────────────────────────────── *
 * scene_tick is the per-frame pseudocode:
 *
 *     if paused: stop.
 *     decay supernova flash envelope.
 *     advance field_time by FIELD_DRIFT × drift_mult × dt.
 *     repaint the grid from the active pattern.
 *
 * There is NO automatic perm reshuffle — the user drives that via
 * 'r' / scene_reset(). */

static void decay_supernova_flash(Scene *s, float dt)
{
    s->sim.supernova_glow_t *= expf(-SUPERNOVA_DECAY * dt);
}

static void advance_field_time(Scene *s, float dt)
{
    s->sim.field_time += FIELD_DRIFT * (float)s->ctrl.drift_mult * dt;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    decay_supernova_flash (s, dt);
    advance_field_time    (s, dt);
    scene_update_grid     (s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal dimensions cached after the last successful
 * getmaxyx(). Refreshed on SIGWINCH via screen_resize(). Kept tiny
 * because the rest of ncurses' state lives implicitly in stdscr; we
 * only need the dimensions to position HUD elements and centre the
 * map.
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

/* ── scene_draw pipeline ──────────────────────────────────────────── *
 * Per cell, scene_draw decides one of three things:
 *
 *     1. Supernova flash active? → twinkle '*' (sparse mask)
 *     2. Density-band rule? → glyph from glyph_ramp
 *     3. Otherwise → skip (blank)
 *
 * Each branch returns a CellDraw; paint_cell is the SINGLE ncurses
 * I/O point for the field grid. */

/*
 * CellDraw — the output of the per-cell glyph-pick chain. A tiny
 * value-type that bundles "what to paint at this position"; the
 * single I/O function paint_cell() converts a CellDraw into the
 * actual ncurses attron/mvaddch/attroff triple.
 *
 * INTENT. Each pick_cell helper (cell_supernova_sparkle,
 * cell_density_band) is a pure function — input cell state, output
 * CellDraw. Combining them via the priority chain in pick_cell()
 * stays branch-only; no side effects, no shared state. paint_cell
 * is then the SINGLE place the program calls into ncurses for the
 * field grid, which makes the rendering pipeline trivial to retarget
 * (test harnesses can collect CellDraws into a buffer for snapshot
 * comparison without touching the terminal).
 *
 * The `skip` flag is the explicit "blank — don't draw anything"
 * signal. It is distinct from drawing a space character, because
 * skipping lets the previous frame's pixel show through (we erase()
 * once per frame, not per cell). For static-field patterns this
 * doesn't matter; for the supernova twinkle it's what makes the
 * sparse star-field look star-like rather than blanket-like.
 */
typedef struct {
    int  pair;   /* colour pair index: PAIR_BAND_BASE+band or PAIR_SUPERNOVA */
    int  attr;   /* ncurses attributes: A_BOLD or A_NORMAL                   */
    char glyph;  /* the ASCII byte to paint                                  */
    bool skip;   /* true → render nothing for this cell                      */
} CellDraw;

/* compute_centred_origin — top-left corner where the map is drawn.
 * Centres horizontally; reserves HUD_BAND_RESERVED_ROWS rows total
 * (HUD_TOP_ROWS state/params/legend + HUD_BOTTOM_ROWS actions).
 * Clamps so the map never overlaps the HUD even on short terminals. */
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

/* cell_supernova_sparkle — twinkle during the post-reset flash.
 * The (x ^ y) & MASK pattern lights one cell in (MASK + 1), giving
 * a sparse star-field rather than a uniform white-out. Cells with
 * active glow always light so the underlying field stays visible. */
static CellDraw cell_supernova_sparkle(int x, int y, float trail_glow)
{
    bool sparkle_lit = ((x ^ y) & SUPERNOVA_SPARSE_MASK) == 0;
    if (!sparkle_lit && trail_glow <= GLOW_THRESHOLD)
        return (CellDraw){ .skip = true };
    return (CellDraw){ .pair = PAIR_SUPERNOVA, .attr = A_BOLD, .glyph = '*' };
}

/* quantize_glow_to_glyph_index — map intensity ∈ [0, 1] to a glyph
 * index 0..N_GLYPHS-1, with clamping. Sibling of
 * quantize_color_to_band but for the density ramp. */
static inline int quantize_glow_to_glyph_index(float glow)
{
    int gi = (int)(glow * GLYPH_QUANTIZE_SCALE);
    if (gi < 0)         gi = 0;
    if (gi >= N_GLYPHS) gi = N_GLYPHS - 1;
    return gi;
}

/* cell_density_band — the standard ASCII density-ramp picker. Pick
 * a glyph index from intensity; bold the upper half of the ramp.
 * Index 0 is the blank tier — render it as `skip` so the cell stays
 * empty rather than wasting a no-op mvaddch(' '). */
static CellDraw cell_density_band(uint8_t band, float glow)
{
    int gi = quantize_glow_to_glyph_index(glow);
    if (gi == 0) return (CellDraw){ .skip = true };
    return (CellDraw){
        .pair  = PAIR_BAND_BASE + (band & (N_BANDS - 1)),
        .attr  = (gi >= GLYPH_BOLD_FROM) ? A_BOLD : A_NORMAL,
        .glyph = glyph_ramp[gi],
    };
}

/* pick_cell — execute the priority chain for one cell. */
static CellDraw pick_cell(const Scene *s, int x, int y)
{
    int   idx        = grid_idx(&s->grid, x, y);
    float trail_glow = s->buf.glow[idx];

    if (s->sim.supernova_glow_t > GLOW_THRESHOLD)
        return cell_supernova_sparkle(x, y, trail_glow);

    return cell_density_band(s->buf.color[idx], trail_glow);
}

/* paint_cell — the ONE ncurses I/O point for the field grid. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    int gx0, gy0;
    compute_centred_origin(&s->grid, cols, rows, &gx0, &gy0);

    for (int y = 0; y < s->grid.h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < s->grid.w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            paint_cell(sy, sx, pick_cell(s, x, y));
        }
    }
}

/* ── HUD draw pipeline ────────────────────────────────────────────── *
 * Six named drawers, one per HUD element. The TOP HUD (rows 0..2)
 * carries DATA — current state, parameter readouts, glyph legend.
 * The BOTTOM HUD (row N-1) carries ACTIONS — key bindings only.
 * screen_draw assembles them in z-order: scene first, then HUD
 * painted over the top. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED "
                                      : pattern_name(c->current_pattern);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  drift:x%-2d ",
             fps, sim_fps, state_str, c->drift_mult);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " DOMAIN WARP (IQ STYLE) ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* draw_palette_swatch — paint one '#' per band colour. Returns the
 * column just past the swatch so the caller can continue laying out. */
static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < HUD_PALETTE_SWATCH_N; i++) {
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* ── row 1 segment drawers ─────────────────────────────────────────── *
 * draw_hud_status_line lays out the row left-to-right as a sequence of
 * fixed-width segments. Each segment is a named drawer that takes the
 * current x-cursor, paints its content, and returns the new x-cursor.
 * The driver function is then pure pseudocode. */

static int draw_status_pattern_field(int row, int x, Pattern p)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-7s ", pattern_name(p));
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

/* draw_status_noise_params — last segment on the row; prints the
 * runtime-relevant noise parameters (scale, warp magnitude, octave
 * count, current map size). No return — nothing else follows. */
static void draw_status_noise_params(int row, int x, const Grid *g)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x,
             "  scale:%.2f  warp:%.1f  oct:%d  map:%dx%d ",
             NOISE_SCALE, WARP_AMOUNT, FBM_OCTAVES, g->w, g->h);
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
        draw_status_noise_params  (1, x, &s->grid);
}

/* draw_hud_ramp_legend — row 2: explains the glyph alphabet so the
 * viewer can READ what they're seeing. This is DATA, not ACTION —
 * the bottom hint is reserved for key bindings only. Drawn in
 * non-bold PAIR_HUD so it sits below the bold state bar in visual
 * hierarchy. */
static void draw_hud_ramp_legend(void)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(2, HUD_LEFT_MARGIN,
             " ramp:  .  :  -  =  +  *  #  %%  @   dim -> bright ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* draw_bottom_hint — row N-1: ACTIONS only (key bindings). */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:drift  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                         double fps, int sim_fps)
{
    erase();
    scene_draw            (s, sc->cols, sc->rows);
    draw_hud_state_bar    (sc, s, fps, sim_fps);   /* row 0  : data    */
    draw_hud_title        ();                       /* row 0  : data    */
    draw_hud_status_line  (s);                      /* row 1  : data    */
    draw_hud_ramp_legend  ();                       /* row 2  : data    */
    draw_bottom_hint      (sc);                     /* row N-1: actions */
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the top-level program state. ONE instance, g_app, lives in
 * BSS so the signal handlers can reach it without a global Scene
 * pointer. The App owns the Scene and the Screen and adds:
 *   • simulation parameters that are not really "scene state"
 *     (sim_fps, map_w, map_h);
 *   • signal-driven flags that must be sig_atomic_t for safety.
 *
 * SIGNAL-HANDLER DISCIPLINE. The handlers do nothing but set a
 * flag. The main loop polls those flags and performs the actual
 * work (cleanup, resize) in normal execution context. Standard
 * async-signal-safe pattern — see signal(7).
 *
 * REFERENCE. W. Richard Stevens & Stephen Rago — "Advanced
 * Programming in the UNIX Environment" (3rd ed), ch. 10 on signals.
 */
typedef struct {
    Scene                 scene;   /* the simulation                    */
    Screen                screen;  /* current terminal dimensions       */

    int                   sim_fps; /* tick rate; mutated by '[' and ']' */
    int                   map_w;   /* chosen map width,  ≤ MAP_W_MAX    */
    int                   map_h;   /* chosen map height, ≤ MAP_H_MAX    */

    /* sig_atomic_t guarantees writes from a handler are observed
     * atomically by the main loop; `volatile` prevents the compiler
     * from caching the read in a register across loop iterations.
     * Both qualifiers are required. */
    volatile sig_atomic_t running;       /* 0 = exit main loop          */
    volatile sig_atomic_t need_resize;   /* 1 = pending SIGWINCH        */
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
    screen_resize    (&app->screen);
    app_pick_map_size(app);
    scene_reset      (&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* ── keyboard handlers ──────────────────────────────────────────────── *
 * Each key is one named action; app_handle_key is just a dispatcher. */

/* bump_drift_geometric — '+' / '-' geometric step on drift_mult.
 * Doubling / halving (rather than ±1) gives a logarithmic feel:
 * each press is the same perceptual change across the full clamped
 * range. dir = +1 doubles; -1 halves. */
static void bump_drift_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->drift_mult < DRIFT_MULT_MAX) c->drift_mult *= 2;
        if (c->drift_mult > DRIFT_MULT_MAX) c->drift_mult = DRIFT_MULT_MAX;
    } else {
        c->drift_mult /= 2;
        if (c->drift_mult < DRIFT_MULT_MIN) c->drift_mult = DRIFT_MULT_MIN;
    }
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

static void cycle_pattern(Controls *c, int dir)
{
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                            break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);  break;
    case '=': case '+': bump_drift_geometric(c, +1);                       break;
    case '-':           bump_drift_geometric(c, -1);                       break;
    case ']':           bump_sim_fps(app, +SIM_FPS_STEP);                  break;
    case '[':           bump_sim_fps(app, -SIM_FPS_STEP);                  break;
    case 't':           cycle_theme  (c, +1);                              break;
    case 'T':           cycle_theme  (c, -1);                              break;
    case 'n': case 'N': cycle_pattern(c, +1);                              break;
    case 'p': case 'P': cycle_pattern(c, -1);                              break;
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
 * the last call, clamp dt at DT_MAX_NS (spiral-of-death guard).
 * Updates *frame_time to "now" in place; returns dt (clamped). */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* simulate_pending_ticks — drain the fixed-timestep accumulator.
 * Runs scene_tick() once per tick_ns worth of accumulated real time;
 * the simulation logic sees a CONSTANT dt_sec independent of frame
 * rate. Source: Glenn Fiedler, "Fix Your Timestep". */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* maybe_update_fps_counter — every FPS_UPDATE_MS, fold the running
 * frame count into a smoothed fps reading. Returns the new fps if
 * the window just closed, otherwise the previous value. */
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

/* cap_frame_rate — sleep so frames are at most 1/target_fps apart.
 * `work_done_ns` is the wall-clock time already consumed this frame;
 * we sleep the remainder of the per-frame budget. */
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
