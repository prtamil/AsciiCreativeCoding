/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * quasicrystal.c
 *   — Quasicrystal interference patterns: sum N plane waves at
 *     angles θ_k = k·π/N over the screen, animate by drifting each
 *     wave's phase. Choosing N coprime with 2, 3, 4, 6 (the Bravais
 *     symmetries) yields APERIODIC long-range order — rotational
 *     symmetry without translational repetition. The same recipe
 *     that produces real-world quasicrystals like the Penrose tiling.
 *
 * DEMO: Concentric stars and rosettes flicker into existence and
 *       slowly morph as the wave phases drift. With 5-fold symmetry
 *       you see Penrose-like 10-pointed stars; 7-fold gives 14-pointed
 *       stars; 11-fold is so dense the pattern looks almost cloud-like
 *       at small scales but breaks into intricate symmetric stars at
 *       large.
 *
 *       PATTERN (n / N) — number of waves N:
 *
 *         TRI       N=3 — periodic hexagonal crystal (for contrast)
 *         PENTA     N=5 — 10-fold quasicrystal, Penrose-flavoured
 *         HEPTA     N=7 — 14-fold quasicrystal, denser stars
 *         UNDECA    N=11 — extremely intricate, near-cloud detail
 *
 *       GLYPH (g / G) — how the intensity field is rendered:
 *
 *         RAMP      8-step density ramp across full [-1, +1] range
 *         PEAKS     only positive intensity drawn (highlights)
 *         CONTOUR   only zero-crossings shown (wave fronts)
 *         WAVES     bipolar — peaks bright, troughs dim
 *
 *       'r' randomises the global phase, snapping the pattern to a
 *       fresh state. The phase offset persists; subsequent drift
 *       continues smoothly from there.
 *
 * Study alongside:
 *   ../patterns/truchet_tiles.c
 *      — also a "static-pattern + slow drift" showcase, but
 *        Truchet's structure is per-cell hash; here it's a continuous
 *        scalar field.
 *   ../fields/perin_noise_flow_showcase.c
 *      — Perlin-driven flow vs cosine-sum-driven interference.
 *        Both produce smooth scalar fields; cosine sums uniquely give
 *        rotational long-range order.
 *
 * Section map:
 *   §1 config        — wavelength, wave counts, themes
 *   §2 clock         — monotonic timer + sleep
 *   §3 color         — HUD reserved + 10 themes (8-step ramp + accents)
 *   §5 quasicrystal  — hash, wave-direction precompute, intensity
 *                      sampler, level mapper
 *   §6 scene         — state, scene_tick (advance time + phase)
 *   §7 screen        — per-cell render through chosen glyph set + HUD
 *   §8 app           — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume phase drift
 *   r          randomise phase offset (snap to new state)
 *   n / N      next pattern  (TRI → PENTA → HEPTA → UNDECA)
 *   p / P      previous pattern
 *   g / G      next / previous glyph set
 *   t / T      next / previous theme
 *   + / =      faster phase drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/patterns/quasicrystal.c \
 *     -o quasicrystal -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Plane-wave interference. Define N wave vectors
 *
 *                    k̂_m = (cos θ_m, sin θ_m)         θ_m = m · π / N
 *
 *                  for m = 0, 1, … , N−1. Pick a common spatial
 *                  frequency ω = 2π / λ. Pick per-wave phases
 *
 *                    φ_m(t) = t · (r_base + m · r_delta)
 *
 *                  drifting at slightly different rates so the pattern
 *                  morphs over time rather than rigidly translating.
 *                  The intensity field is the sum of N cosine waves:
 *
 *                    I(x, y, t)
 *                      = (1/N) · Σ_m cos( ω·(x·cos θ_m + y·sin θ_m)
 *                                          + φ_m(t) )
 *
 *                  ≈ in [-1, +1].  Renderer maps each cell's intensity
 *                  through one of four glyph sets:
 *
 *                    RAMP    : intensity ∈ [-1, +1] → level 0..7
 *                              → glyph from ' .,:-o#@' ramp.
 *                    PEAKS   : negative intensity → blank;
 *                              positive intensity → bright glyph.
 *                    CONTOUR : |intensity| < ε → glyph;  else blank.
 *                              Renders only the zero-crossings —
 *                              the wave-front network.
 *                    WAVES   : positive → bright (peak crests);
 *                              negative → dim (trough valleys).
 *
 *                  Why N=5, 7, 11 give APERIODIC patterns: the
 *                  Crystallographic Restriction Theorem says only
 *                  rotational symmetries of order 1, 2, 3, 4, 6 can
 *                  arise in 2-D periodic lattices. A cosine-sum of
 *                  N waves at θ_m = m·π/N has 2N-fold rotational
 *                  symmetry; for N coprime with {1, 2, 3, 4, 6} the
 *                  resulting symmetry order (10, 14, 22 etc.) cannot
 *                  be a Bravais lattice — the pattern necessarily
 *                  has long-range order without translational
 *                  periodicity, the defining property of a
 *                  quasicrystal.
 *
 * Data-structure : Two precomputed arrays — wave_cos[N], wave_sin[N]
 *                  — refreshed when the pattern (and thus N) changes.
 *                  Plus the global Scene state. No grid is stored;
 *                  every cell is recomputed each frame.
 *
 * Rendering      : ASCII only. Per cell: N cos calls + a few
 *                  multiplies for the dot product with each wave
 *                  vector. With N=11, that's 11 cos/cell, ~12.7M cos
 *                  per second on a 240×80 × 60 fps screen. Around
 *                  6 % of one CPU core. No allocation per frame.
 *
 * Performance    : Dominated by the N cosine evaluations per cell.
 *                  Could be optimised by precomputing a 1-D table of
 *                  cos(ω·distance) values at integer step lengths
 *                  per wave, but the straightforward version is
 *                  already fast enough at terminal resolutions.
 *
 * References     :
 *   • Shechtman, D. et al. (1984) — "Metallic Phase with Long-Range
 *     Orientational Order and No Translational Symmetry",
 *     Phys. Rev. Lett. 53(20):1951.  Original quasicrystal
 *     observation; Shechtman's 2011 Nobel Prize.
 *   • Wikipedia — Quasicrystal
 *     https://en.wikipedia.org/wiki/Quasicrystal
 *   • Wikipedia — Crystallographic Restriction Theorem
 *     https://en.wikipedia.org/wiki/Crystallographic_restriction_theorem
 *   • Penrose, R. (1974) — "The Role of Aesthetics in Pure and
 *     Applied Mathematical Research", Bulletin of the IMA 10:266.
 *     The Penrose tiling.
 *   • Mike Bostock — "Quasicrystals" (interactive)
 *     https://bl.ocks.org/mbostock/3019563
 *   • Shadertoy — many quasicrystal shaders
 *     https://www.shadertoy.com/results?query=quasicrystal
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To make a pattern with rotational symmetry but no translational
 * periodicity, sum cosine waves in N evenly-spaced directions
 * around the circle. If N is coprime with the lattice symmetries
 * (1, 2, 3, 4, 6), the resulting interference cannot tile
 * periodically — yet it still has perfect rotational symmetry.
 * That is a QUASICRYSTAL: long-range orientational order without
 * a unit cell.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Drop a stone in a still pond and you get circular ripples.
 * Drop two stones at opposite ends and where the ripple-fronts
 * meet, you get an interference pattern — bright bands where the
 * waves add, dark bands where they cancel. Now imagine N stones
 * dropped in a perfect circle around the pond, all at the same
 * moment. Each stone emits a wave; the waves all reach the pond
 * centre simultaneously and cross at angles 2π/N apart.
 *
 * Look down at the surface from above. The interference pattern
 * has N-fold (or 2N-fold) rotational symmetry — rotate it by
 * 2π/N and it looks the same. But for "weird" N like 5, 7, 11,
 * it CANNOT be the result of stamping a single tile periodically;
 * the symmetry is incompatible with any wallpaper group. The
 * pattern is LITERALLY EVERYWHERE, infinite, intricate, ordered,
 * yet has no repeating unit. That is a quasicrystal.
 *
 * Now let each stone's ripple drift at its own slow rate. The
 * pattern is still N-fold symmetric at every moment, but it
 * MORPHS — stars shift, troughs deepen, new constellations of
 * peaks form and dissolve. That is the animation.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. CHOOSE N (the wave count). Precompute wave_cos[m],
 *     wave_sin[m] for m = 0..N-1 with angle m·π/N.
 *  2. EACH FRAME:
 *     a. Advance global time t.
 *     b. For every screen cell (sx, sy):
 *        i.   x = sx;  y = sy · ASPECT_Y      // aspect correction
 *        ii.  intensity = 0
 *        iii. For each wave m:
 *               wx     = x · wave_cos[m] + y · wave_sin[m]
 *               φ_m    = (t + offset) · (r_base + m · r_delta)
 *               intensity += cos( ω·wx + φ_m )
 *        iv.  intensity /= N                  // → ≈ [-1, +1]
 *        v.   Map intensity → glyph + colour via active GlyphSet.
 *        vi.  mvaddch(sy, sx, glyph) in selected pair + attr.
 *  3. HUD on top.
 *
 * KEY FORMULAS
 * ────────────
 *  Wave vector for wave m:
 *    θ_m   = m · π / N                  ω = 2π / λ
 *    k̂_m  = (cos θ_m, sin θ_m)
 *
 *  Intensity at (x, y, t):
 *    I = (1/N) · Σ_{m=0..N-1}  cos( ω · k̂_m · (x, y) + φ_m(t) )
 *
 *  Per-wave phase rate (gives morphing animation):
 *    φ_m(t) = t · (r_base + m · r_delta)
 *
 *  Intensity → ramp level (RAMP glyph):
 *    level = clamp(⌊(I + 1)·4⌋, 0, 7)
 *
 *  Zero-crossing (CONTOUR glyph):
 *    drawn when |I| < ε ; brighter when closer to zero.
 *
 *  Bipolar (WAVES glyph):
 *    I > +T_high → core peak '#'
 *    I > 0       → mid peak  '*'
 *    I > -T_high → mid trough '.'
 *    else         → core trough ','
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • N COPRIME RULE. Periodic vs aperiodic depends on N. N = 3 and
 *    N = 6 give visually-perfect HEXAGONAL periodic patterns —
 *    useful for contrast in the demo, but they are NOT
 *    quasicrystals. The rule: N must be coprime with the divisors
 *    of 12 (for 2-D Bravais). 5, 7, 11 work; 3 doesn't.
 *
 *  • COSINE COUNT AT N=11. 11 cosines per cell × 19 200 cells × 60
 *    fps ≈ 12.7 M cos calls/s. Modern CPUs handle it (~6 % of one
 *    core). If you push N to 31 just to see what happens, expect
 *    framerate to drop noticeably.
 *
 *  • ASPECT CORRECTION. Multiply screen y by ASPECT_Y_F (=2) when
 *    sampling so the pattern's circles look round on terminals
 *    where cells are 2× taller than wide. Without it, the
 *    quasicrystal stars become horizontal ovals.
 *
 *  • PHASE-RATE SPREAD. If all waves drift at the SAME rate, the
 *    pattern just translates rigidly across the screen — no
 *    morphing. A small per-wave delta (k · 0.07) makes the relative
 *    phases shift, so the pattern visibly REORGANISES, which is
 *    much more interesting.
 *
 *  • CONTOUR THRESHOLD AT N=11. With 11 waves the intensity field
 *    has very fine-scale structure; |I|<0.05 leaves visible gaps in
 *    the contour. For higher N, widen the threshold (or use a
 *    derivative-based contour) so the lines stay continuous.
 *
 *  • RANDOMISE PHASE. 'r' adds a random offset to t. The drift
 *    continues from there, so the offset persists across frames.
 *    Don't subtract the offset on the next frame — that would
 *    snap it back.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • PAUSE (space). Pattern freezes. Resume: drift continues from
 *    exactly where it stopped. Verifies the time accumulator.
 *
 *  • PENTA pattern. Look for 10-pointed stars (5-fold symmetry, but
 *    cosine doubles it). Counting points around the brightest peak
 *    near the centre should give 10. The Penrose-tiling kinship is
 *    visible.
 *
 *  • HEPTA pattern. Stars now have 14 points. The pattern is
 *    visibly denser and finer than PENTA at the same wavelength.
 *
 *  • UNDECA pattern. Visual density makes individual stars hard to
 *    count, but rotational symmetry is still perfect about any
 *    centre. Pause and rotate your head 360°/22 ≈ 16.4° — the
 *    pattern looks the same.
 *
 *  • TRI (N=3) pattern. You'll see HEXAGONAL periodic structure —
 *    cells repeat in a tilable lattice. This is NOT a quasicrystal;
 *    it's included for comparison so the difference is visible
 *    when you switch to PENTA.
 *
 *  • CONTOUR glyph. The drawn lines should form a network of
 *    closed curves — these are the level sets I = 0 of the cosine
 *    sum. They DO NOT cross periodically.
 *
 *  • PEAKS glyph. Should show only the bright cells (positive
 *    intensity). Half the screen (where I < 0) is blank. The
 *    visible "stars" are the wave crests.
 *
 *  • Theme cycle (t / T). Each theme should produce visible
 *    contrast. RAMP and WAVES use the full ramp[0..7]; PEAKS uses
 *    ramp[3..7]; CONTOUR uses bright slots.
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
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Maximum wave count we'll ever use — sized for the UNDECA pattern. */
    N_WAVES_MAX         =  11,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 intensity tints       */
    PAIR_HOT            =  11,    /* peak accent                      */
    PAIR_COLD           =  12,    /* trough accent                    */
    PAIR_FLASH          =  13,    /* phase-randomise flash            */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y_F           2.0f

/*
 * Spatial wavelength (in cell units) at which each wave oscillates.
 * Smaller = denser, more wave fronts visible; larger = bigger features.
 * 14 strikes a good balance for terminal resolutions.
 */
#define WAVELENGTH           14.0f
#define BASE_FREQ            (2.0f * (float)M_PI / WAVELENGTH)

/*
 * Per-wave phase drift.
 *   ANIMATION_RATE_BASE  — radians per second the WHOLE pattern drifts
 *                          (uniform across waves; rigid translation).
 *   ANIMATION_RATE_DELTA — per-wave INCREMENT on top of the base rate.
 *                          Without this all waves shift in lockstep
 *                          and the pattern only translates; with it
 *                          the relative phases evolve so the pattern
 *                          MORPHS (much more interesting).
 */
#define ANIMATION_RATE_BASE  0.50f
#define ANIMATION_RATE_DELTA 0.07f

/*
 * Pattern enum — one entry per wave count we showcase. The "TRI"
 * (N=3) pattern is included for CONTRAST: it's hexagonal periodic,
 * not a quasicrystal, and gives the eye a clear reference for what
 * "periodic" looks like before flipping to PENTA / HEPTA / UNDECA.
 */
typedef enum {
    PATTERN_TRI    = 0,    /* N = 3   (hexagonal, periodic)            */
    PATTERN_PENTA  = 1,    /* N = 5   (10-fold quasicrystal)           */
    PATTERN_HEPTA  = 2,    /* N = 7   (14-fold quasicrystal)           */
    PATTERN_UNDECA = 3,    /* N = 11  (22-fold quasicrystal)           */
    N_PATTERNS     = 4,
} Pattern;

static int pattern_n_waves(Pattern p)
{
    switch (p) {
    case PATTERN_TRI:    return 3;
    case PATTERN_PENTA:  return 5;
    case PATTERN_HEPTA:  return 7;
    case PATTERN_UNDECA: return 11;
    default:             return 5;
    }
}

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_TRI:    return "TRI   ";
    case PATTERN_PENTA:  return "PENTA ";
    case PATTERN_HEPTA:  return "HEPTA ";
    case PATTERN_UNDECA: return "UNDECA";
    default:             return "?     ";
    }
}

/* GlyphSet — how the [-1, +1] intensity is rendered. */
typedef enum {
    GLYPH_RAMP    = 0,
    GLYPH_PEAKS   = 1,
    GLYPH_CONTOUR = 2,
    GLYPH_WAVES   = 3,
    N_GLYPH_SETS  = 4,
} GlyphSet;

static const char *glyph_set_name(GlyphSet g)
{
    switch (g) {
    case GLYPH_RAMP:    return "ramp ";
    case GLYPH_PEAKS:   return "peaks";
    case GLYPH_CONTOUR: return "cntr ";
    case GLYPH_WAVES:   return "waves";
    default:            return "?    ";
    }
}

/* Density-ramp glyphs (low → high intensity). Used by RAMP / PEAKS /
 * WAVES; CONTOUR uses a single thresholded glyph.  ASCII-only. */
static const char RAMP_GLYPHS[8] = { '`', '.', ',', ':', '-', 'o', '#', '@' };

/* Contour band thickness — cells with |intensity| < CONTOUR_BAND_HALF
 * are drawn; brighter the closer to zero. */
#define CONTOUR_BAND_HALF    0.20f

/*
 * Themes — every entry sits in the bright half of the 256-colour
 * cube so even A_DIM cells stay legible against a default-black
 * terminal.  See "Theme Palette Brightness" in /CLAUDE.md.
 */
typedef struct {
    const char *name;
    short       ramp[8];
    short       hot;
    short       cold;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name       0    1    2    3    4    5    6    7   hot cold */
    { "DEFAULT",{ 24,  31,  39,  70,  76, 137, 215, 230 }, 196,  39 },
    { "MATRIX", { 28,  34,  40,  46,  76, 118, 154, 192 }, 226,  39 },
    { "NOVA",   { 60,  91, 134, 165, 207, 213, 219, 231 }, 196,  39 },
    { "MONO",   {240, 243, 245, 247, 249, 251, 253, 255 }, 226,  39 },
    { "OCEAN",  { 24,  25,  31,  38,  45,  51, 117, 195 }, 196,  21 },
    { "FIRE",   { 88, 124, 130, 166, 202, 208, 214, 226 }, 226,  21 },
    { "EARTH",  { 94, 130, 137, 173, 179, 215, 222, 230 }, 196,  39 },
    { "FOREST", { 28,  34,  40,  70,  76, 112, 156, 192 }, 196,  39 },
    { "DESERT", {130, 137, 143, 173, 179, 215, 222, 229 }, 196,  21 },
    { "ARCTIC", { 24,  31,  67, 110, 117, 153, 195, 231 }, 196,  39 },
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
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_HOT,  t->hot,  -1);
        init_pair(PAIR_COLD, t->cold, -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_HOT,  COLOR_RED,  -1);
        init_pair(PAIR_COLD, COLOR_CYAN, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
        init_pair(PAIR_FLASH, 226, -1);
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  quasicrystal — wave precompute, intensity sampler                  */
/* ===================================================================== */

/* hash3 — only used for the reseed-flash overlay. */
static inline uint32_t hash3(int wx, int wy, int wz)
{
    uint32_t h = (uint32_t)wx * 73856093u
               ^ (uint32_t)wy * 19349663u
               ^ (uint32_t)wz * 83492791u;
    h ^= h >> 16;
    h *= 0x85ebca6bu;
    h ^= h >> 13;
    h *= 0xc2b2ae35u;
    h ^= h >> 16;
    return h;
}

/*
 * Per-wave direction tables. Refilled on pattern change so the
 * intensity sampler can iterate without recomputing trig per cell.
 */
static float wave_cos[N_WAVES_MAX];
static float wave_sin[N_WAVES_MAX];

static void waves_init(int n)
{
    if (n < 1) n = 1;
    if (n > N_WAVES_MAX) n = N_WAVES_MAX;
    for (int k = 0; k < n; k++) {
        float angle = (float)k * (float)M_PI / (float)n;
        wave_cos[k] = cosf(angle);
        wave_sin[k] = sinf(angle);
    }
}

/*
 * compute_intensity — sum N plane waves at the screen cell.
 *
 *   x  = sx                        (screen x in cell units)
 *   y  = sy * ASPECT_Y_F          (aspect-corrected screen y)
 *   I  = (1/N) Σ cos(ω(x·cos θ_k + y·sin θ_k) + φ_k(t))
 *
 * Returns roughly [-1, +1] (exact bounds depend on phase alignment;
 * the maximum is 1.0 when all N waves constructively interfere).
 */
static float compute_intensity(int sx, int sy, float t, int n_waves)
{
    float fx = (float)sx;
    float fy = (float)sy * ASPECT_Y_F;
    float sum = 0.0f;
    for (int k = 0; k < n_waves; k++) {
        float wx    = fx * wave_cos[k] + fy * wave_sin[k];
        float phase = t * (ANIMATION_RATE_BASE
                         + (float)k * ANIMATION_RATE_DELTA);
        sum += cosf(BASE_FREQ * wx + phase);
    }
    return sum / (float)n_waves;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool     paused;
    int      speed;
    int      current_theme;
    Pattern  current_pattern;
    GlyphSet current_glyph;
    float    time_secs;
    float    phase_offset;       /* added to time; randomised by 'r' */
    float    flash_t;
} Scene;

static void scene_pattern_changed(Scene *s)
{
    waves_init(pattern_n_waves(s->current_pattern));
}

static void scene_reseed(Scene *s)
{
    /* Randomise the phase offset — pattern snaps to a new state. */
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->phase_offset * 100.0f), 0xDECAF);
    s->phase_offset = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->flash_t = 1.0f;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_PENTA;
    s->current_glyph   = GLYPH_RAMP;
    s->phase_offset    = 0.0f;
    scene_pattern_changed(s);
}

/*
 * scene_tick — advance time and decay the flash overlay. Speed
 * scales the time advance, so fast/slow drift is just a multiplier
 * on the phase rate.
 */
static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-4.0f * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct {
    int cols, rows;
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
 * intensity_to_glyph — choose (glyph, ramp_idx, attr) from the
 * intensity value and the active GlyphSet. Returns false when the
 * cell should be left blank (e.g., negative intensity in PEAKS mode,
 * far-from-zero intensity in CONTOUR mode).
 */
static bool intensity_to_glyph(float intensity, GlyphSet g,
                               char *glyph, int *ramp_idx, int *attr)
{
    *attr = A_NORMAL;

    switch (g) {
    case GLYPH_RAMP: {
        /* [-1, +1] → [0, 7] */
        int level = (int)((intensity + 1.0f) * 4.0f);
        if (level < 0) level = 0;
        if (level > 7) level = 7;
        *glyph = RAMP_GLYPHS[level];
        *ramp_idx = level;
        if (level >= 6) *attr = A_BOLD;
        else if (level <= 1) *attr = A_DIM;
        return true;
    }

    case GLYPH_PEAKS: {
        /* Only positive intensity; brighter the closer to +1. */
        if (intensity < 0.0f) return false;
        int level = (int)(intensity * 8.0f);
        if (level < 0) level = 0;
        if (level > 7) level = 7;
        *glyph = RAMP_GLYPHS[level];
        *ramp_idx = level;
        if (level >= 6) *attr = A_BOLD;
        else if (level <= 1) *attr = A_DIM;
        return true;
    }

    case GLYPH_CONTOUR: {
        /* Zero-crossing band: brighter the closer to I=0. */
        float dist = fabsf(intensity);
        if (dist > CONTOUR_BAND_HALF) return false;
        float strength = 1.0f - dist / CONTOUR_BAND_HALF;     /* [0, 1] */
        int level = (int)(strength * 8.0f);
        if (level < 0) level = 0;
        if (level > 7) level = 7;
        *glyph = (strength > 0.65f) ? '*' : (strength > 0.30f) ? '.' : '`';
        *ramp_idx = level;
        if (level >= 6) *attr = A_BOLD;
        return true;
    }

    case GLYPH_WAVES: {
        /* Bipolar — peaks bright, troughs dim. */
        if (intensity > 0.55f) {
            *glyph = '#'; *ramp_idx = 7; *attr = A_BOLD;
        } else if (intensity > 0.15f) {
            *glyph = 'o'; *ramp_idx = 5;
        } else if (intensity > -0.15f) {
            *glyph = '.'; *ramp_idx = 3; *attr = A_DIM;
        } else if (intensity > -0.55f) {
            *glyph = ','; *ramp_idx = 2; *attr = A_DIM;
        } else {
            *glyph = '`'; *ramp_idx = 1; *attr = A_DIM;
        }
        return true;
    }

    case N_GLYPH_SETS:
        return false;
    }
    return false;
}

static void scene_draw(const Screen *sc, const Scene *s)
{
    int top = 2, bottom = sc->rows - 1;
    int n = pattern_n_waves(s->current_pattern);
    float t = s->time_secs + s->phase_offset;

    /* Centre the field on the screen so the rotational symmetry
     * point sits at the middle of the renderable area. */
    int cx = sc->cols / 2;
    int cy = (top + bottom) / 2;

    for (int sy = top; sy < bottom; sy++) {
        int rel_y = sy - cy;
        for (int sx = 0; sx < sc->cols; sx++) {
            int rel_x = sx - cx;
            float intensity = compute_intensity(rel_x, rel_y, t, n);

            char glyph;
            int  ramp_idx, attr;
            if (!intensity_to_glyph(intensity, s->current_glyph,
                                    &glyph, &ramp_idx, &attr)) continue;

            int pair = PAIR_RAMP_BASE + ramp_idx;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* Reseed flash overlay. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = top; sy < bottom; sy += 2) {
            for (int sx = 0; sx < sc->cols; sx += 2) {
                if (((sx ^ sy ^ seed) & 7) == 0)
                    mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const char *state_str = s->paused ? "PAUSED" : "DRIFT ";

    /* Row 0 right — primary status. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " QUASICRYSTAL ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 — pattern + glyph + theme + ramp swatch + N. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-6s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " glyph:%-5s ", glyph_set_name(s->current_glyph));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 14;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    for (int i = 0; i < 8; i++) {
        int p = PAIR_RAMP_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, (chtype)(unsigned char)RAMP_GLYPHS[i]);
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  N=%d  phase:%5.2f ",
             pattern_n_waves(s->current_pattern),
             (double)(s->time_secs + s->phase_offset));
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  g/G:glyph  t/T:theme  +/-:drift  spc:pause  r:reseed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                               break;

    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed  = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed  = SPEED_MIN;
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
        scene_pattern_changed(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_pattern_changed(s);
        break;

    case 'g':
        s->current_glyph = (GlyphSet)(((int)s->current_glyph + 1) % N_GLYPH_SETS);
        break;
    case 'G':
        s->current_glyph = (GlyphSet)(((int)s->current_glyph + N_GLYPH_SETS - 1) % N_GLYPH_SETS);
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
    scene_init(&app->scene);

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
