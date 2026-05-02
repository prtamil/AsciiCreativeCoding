/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_galaxy.c
 *   — A rotating procedural galaxy: logarithmic spiral arms perturbed
 *     by Perlin/fBm noise, sampled per cell with no stored stars.
 *
 * DEMO: A galaxy fills the screen and slowly rotates. Bright stars
 *       cluster along curved spiral arms; the dense yellow bulge sits
 *       at the centre. Outside the arms, the disk thins into a sparse
 *       blue halo. Cycle through four cosmic morphologies:
 *
 *         SPIRAL      4-arm logarithmic spiral (the default)
 *         BARRED      2 arms emerging from a long central bar
 *                     (this is what the Milky Way actually looks like)
 *         ELLIPTICAL  no arms — a smooth Gaussian "ball" of stars,
 *                     for visual contrast with the spiral patterns
 *         NEBULA      spiral arms with visible glowing dust clouds
 *                     painted in the gaps between stars (fBm noise)
 *
 *       Nothing is stored. For every screen cell we (a) rotate it into
 *       galaxy-frame coordinates, (b) evaluate a closed-form density
 *       function at that point, (c) hash to decide whether a star
 *       lives there, and (d) draw or skip. The galaxy is therefore
 *       infinite-resolution, deterministic, and free of allocations.
 *
 * Study alongside:
 *   ../worldgen/procedural_star_field_parallax_noise_showcase.c
 *      — that file uses the SAME hash-based-procedural-content trick,
 *        but for parallel sheets of parallax stars (depth from speed).
 *        This file uses the trick for a single curved field (depth
 *        from spiral structure). Together they teach two complementary
 *        ways to make worlds out of pure functions.
 *   ../fields/perin_noise_flow_showcase.c
 *      — the canonical Perlin / fBm reference; the noise scaffolding
 *        in §5 is copied inline from there per the self-contained-file
 *        rule.
 *
 * Section map:
 *   §1 config    — galaxy geometry, patterns, themes, scroll knobs
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — HUD reserved + 10 themes (4 star + 4 nebula tints)
 *   §5 galaxy    — hash3, perlin/fbm, density(r, θ, pattern)
 *   §6 scene     — Scene state, scene_tick (rotation + noise drift)
 *   §7 screen    — per-cell density gating, glyph + colour selection
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset angle, reseed noise
 *   n / N      next pattern  (SPIRAL → BARRED → ELLIPTICAL → NEBULA)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster rotation
 *   -          slower rotation
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/procedural_galaxy.c \
 *     -o galaxy -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Three pieces composed together.
 *
 *                  (1) Logarithmic spiral — Bernoulli's curve
 *                        r = a · exp(b · θ)
 *                      or equivalently θ = ln(r/a) / b.
 *                      Self-similar (zoom in, see the same shape) and
 *                      scale-invariant — the canonical arm shape of
 *                      every disk galaxy. The constant b sets the
 *                      "pitch": pitch_angle = atan(b). Real galaxies
 *                      cluster around b ≈ 0.20 (pitch ≈ 11°). A
 *                      multi-arm galaxy is just N copies of the same
 *                      spiral, rotated by 2π·k/N for k = 0..N-1.
 *
 *                  (2) Density model — closed form per pattern.
 *                      density(r, θ) = bulge(r) + disk(r) · arm(r, θ)
 *                      where:
 *                        bulge(r) = exp(-r²/σ_b²)         central core
 *                        disk(r)  = exp(-r²/σ_d²)         radial taper
 *                        arm(r,θ) = exp(-arc²/W²)         arm proximity
 *                        arc      = (θ - ln(r)/b) wrapped
 *                                   to [-π/N, +π/N], times r,
 *                                   so it's the arc-length distance
 *                                   to the nearest arm at radius r.
 *                      The BARRED variant adds an elliptical bar term:
 *                        bar(x,y) = exp(-(x/B_x)² - (y/B_y)²)
 *                      The ELLIPTICAL variant drops arm() entirely.
 *
 *                  (3) Hash gate — content from a pure function.
 *                      For each visible cell we compute (gx, gy) in
 *                      the galaxy's rotated reference frame, sample
 *                      density, and roll a hash:
 *                        h        = hash3(floor(gx), floor(gy/A), p)
 *                        h_unit   = (h & 0xFFFFFF) / 2²⁴   ∈ [0, 1)
 *                        is_star  = h_unit < density · K
 *                      where K is a global star-density scale. The
 *                      same galaxy point always yields the same hash,
 *                      so as the rotation matrix changes, stars do
 *                      NOT flicker — they translate cleanly.
 *
 *                  Plus an optional fBm noise term that perturbs the
 *                  arm width by ±20%, giving the arms an organic,
 *                  irregular appearance instead of mathematically clean
 *                  edges. In the NEBULA pattern the same fBm field is
 *                  also rendered as a glowing cloud overlay in cells
 *                  where no star was placed.
 *
 * Data-structure : NONE. The galaxy is a function, not an array.
 *                  State is purely:
 *                    - 256-entry permutation table for the noise
 *                    - rotation angle, noise-time offset
 *                    - pattern / theme / speed selectors
 *                  No grid, no entity pool, no spatial index.
 *
 * Rendering      : ASCII only. Per cell, evaluate density. If a star
 *                  is gated, choose glyph & colour by density and by
 *                  radius:
 *                    radius bucket → 4 star tints (warm bulge → cool
 *                                    halo; mimics the actual stellar-
 *                                    population gradient of galaxies).
 *                    density bucket → A_BOLD '#'/'O'/'*'  (high)
 *                                     A_NORMAL '*'/'+'/'o' (mid)
 *                                     A_DIM '.'/'`'/','/'\''
 *                  In NEBULA cells the fBm value, gated on density,
 *                  paints '#'/'*'/'.' nebula glyphs in a separate
 *                  4-tint palette.
 *
 * Performance    : O(W·H) per frame. For each cell:
 *                    1 cos/sin + 1 sqrt + 1 atan2 + 1 log + 1 exp +
 *                    fbm (4 perlin = 4·fade + 4·lerps) + hash
 *                  ≈ 350 ns per cell on a current CPU. At 240×80×60fps
 *                  ≈ 4 ms/frame ≈ 24 % of one core. No allocation in
 *                  the loop, no I/O, no branches that depend on state.
 *
 * References     :
 *   • Wikipedia — Logarithmic spiral
 *     https://en.wikipedia.org/wiki/Logarithmic_spiral
 *   • Wikipedia — Spiral galaxy / pitch angle
 *     https://en.wikipedia.org/wiki/Spiral_galaxy
 *   • Wikipedia — Density wave theory (why arms exist at all)
 *     https://en.wikipedia.org/wiki/Density_wave_theory
 *   • Lin & Shu (1964) — On the spiral structure of disk galaxies
 *     ApJ 140, 646 — the classic density-wave paper.
 *   • Perlin, K. (2002) — Improving Noise (quintic fade)
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   • Inigo Quilez — "Painting a galaxy" (procedural galaxy shader)
 *     https://iquilezles.org/articles/warp/
 *   • Red Blob Games — Map generation with noise
 *     https://www.redblobgames.com/articles/noise/introduction.html
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A galaxy is not stars; a galaxy is a DENSITY FIELD over the plane.
 * Where the field is high you sprinkle bright stars; where it's medium
 * you sprinkle dim stars; where it's zero you sprinkle nothing. The
 * shape of the field — bulge plus disk plus arms — is determined by
 * a handful of closed-form Gaussian terms, with the spiral arms coming
 * from one elegant equation: the logarithmic spiral. Add slight noise
 * for organicness, rotate the whole thing slowly for life, and you
 * have a galaxy that fits in 1 KB of code.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture the screen as a piece of black paper. You hold a stencil
 * in front of it — the stencil is opaque except for the shape of the
 * galaxy: a bright disc in the middle, two or four curved slits going
 * outward, and a sparse halo at the edges. You spray-paint white
 * through the stencil. Where the stencil is transparent, lots of
 * paint specks land; where it's nearly opaque, very few do. The
 * resulting picture LOOKS like a galaxy because the stencil shape
 * encodes "a galaxy".
 *
 * Now replace the stencil with a function. density(r, θ) returns a
 * number in [0, 1] for any point on the paper — this is the local
 * "transparency". Replace the spray-paint with a hash: for each pixel,
 * roll a fair die and place a star iff the die-roll falls under the
 * density value. The galaxy emerges automatically. Rotate the
 * function (substitute θ → θ + ωt) and the galaxy turns.
 *
 * The LOGARITHMIC SPIRAL part is the one piece of geometry you have
 * to internalise. Every point (r, θ) on the spiral satisfies
 *   θ = ln(r) / b + 2πk/N
 * for some integer k. The MAGIC of doing math in (r, θ) instead of
 * (x, y) is that "distance to the nearest arm" becomes a one-line
 * formula: subtract ln(r)/b from θ, wrap into one period, multiply
 * by r. That's the arc-length to the arm. Plug into a Gaussian and
 * you have arm density. No iteration, no nearest-point search, no
 * data structure — pure trigonometry.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. Each frame, advance the rotation angle: angle += ω · dt.
 *  2. For every visible screen cell (sx, sy):
 *     a. Translate so (cx, cy) is the galaxy centre:
 *          fx = sx − cx
 *          fy = (sy − cy) · 2          // aspect: cells are 2× tall
 *     b. Rotate INTO the galaxy frame (inverse of the screen rotation)
 *        by the current angle:
 *          gx =  fx·cos(angle) + fy·sin(angle)
 *          gy = −fx·sin(angle) + fy·cos(angle)
 *     c. Normalise to galaxy-radius units:
 *          gnx = gx / R0     (R0 = galaxy half-width in cells)
 *          gny = gy / R0
 *     d. Polar:
 *          r  = √(gnx² + gny²)
 *          θ  = atan2(gny, gnx)
 *        If r > DISK_MAX, this cell is outside the galaxy — skip.
 *     e. Sample fBm(gnx, gny, noise_time) — used to perturb the arm
 *        width and (in NEBULA) drive the cloud overlay.
 *     f. Compute density per the active pattern (see §5).
 *     g. Quantise the galaxy-frame coords to integer "world cells",
 *        hash, normalise to a uniform fraction h_unit ∈ [0, 1).
 *     h. If h_unit < density · STAR_PROB_SCALE → STAR:
 *          – pick colour from r-bucket (warm centre → cool edge)
 *          – pick glyph + brightness from density bucket
 *          – mvaddch
 *        Else if pattern == NEBULA and density > 0.05 and fBm > T:
 *          – pick nebula tint, paint cloud glyph
 *        Else: leave cell black.
 *  3. Draw HUD, present, sleep.
 *
 * KEY FORMULAS
 * ────────────
 *  Logarithmic spiral (one arm, k = 0):
 *    r = a · exp(b · θ)            ⇔    θ = ln(r/a) / b
 *
 *  N-arm spiral — angular distance from (r, θ) to the nearest arm:
 *    α    = θ − ln(r) / b                                      // raw phase
 *    seg  = 2π / N
 *    α    = α − seg · round(α / seg)        // wrap to [-seg/2, +seg/2]
 *    arc  = α · r                            // arc-length to arm
 *
 *  Arm density (Gaussian falloff with optional noise modulation):
 *    W    = ARM_WIDTH · (1 + 0.2 · (fbm − 0.5) · 2)
 *    arm  = exp(-arc² / W²)
 *
 *  Bulge / disk (Gaussian envelopes):
 *    bulge(r) = exp(-r² / σ_b²)
 *    disk(r)  = exp(-r² / σ_d²)
 *
 *  Bar (BARRED pattern only) — elliptical core, oriented along x:
 *    bar(x, y) = exp(-x² / B_x² − y² / B_y²)
 *
 *  Composite density:
 *    SPIRAL/NEBULA :  bulge + disk · arm
 *    BARRED        :  bulge·0.6 + bar·BAR_AMP + disk·arm·(1−bar)
 *    ELLIPTICAL    :  bulge·0.9 + halo·0.45                  // no θ
 *
 *  Star gate:
 *    h_unit = (hash3(qx, qy, pat) & 0xFFFFFF) / 2²⁴
 *    is_star = h_unit < density · STAR_PROB_SCALE
 *
 *  Rotation (rigid — every point rotates at the same angular speed):
 *    angle' = angle + ω · dt
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • LOG SINGULARITY. ln(r) → −∞ as r → 0. Clamp r to a small floor
 *    (≈ 0.04) before computing the spiral phase, OR let the bulge term
 *    dominate at small r so the arm term's behaviour at the singularity
 *    doesn't matter. Both — clamp AND let bulge dominate.
 *
 *  • ASPECT RATIO. Terminal cells are ~2× taller than wide. Without
 *    multiplying (sy − cy) by 2 in fy, the galaxy renders as a flat
 *    horizontal ellipse rather than a circle. Always correct in the
 *    rendering math. Use floor(gy/2) when quantising for the hash so
 *    the integer galaxy cells stay roughly square.
 *
 *  • RIGID VS DIFFERENTIAL ROTATION. Real galaxy arms are density
 *    waves — the stars passing through them rotate at different angu-
 *    lar speeds at different radii (Ω(r) ∝ 1/r approximately). If you
 *    rotate the density field rigidly (ω constant in r), the visual
 *    arms STAY the same shape; if you let ω depend on r, the arms
 *    wind up infinitely tight in finite time (the "winding problem"
 *    that motivated density-wave theory). For this demo we use rigid
 *    rotation — the field IS the arms, so they don't wind.
 *
 *  • HASH JITTER UNDER ROTATION. floor(gx), floor(gy) jump by 1 as
 *    the rotated coord crosses an integer line. So as the galaxy
 *    turns, individual stars STEP between adjacent cells rather than
 *    sliding smoothly. This is unavoidable in cell-based rendering.
 *    The eye reads it as scintillation — actually adds to the
 *    "starlight" illusion.
 *
 *  • PATTERN-DEPENDENT HASH. We pass the pattern index as the third
 *    arg to hash3 so each pattern has its own star arrangement.
 *    Otherwise switching from SPIRAL to ELLIPTICAL with the same
 *    rotation angle would leave many of the same stars visible —
 *    looks like a half-collapsed transition rather than a genuinely
 *    different galaxy.
 *
 *  • OFF-SCREEN R. After rotation+aspect correction a screen-corner
 *    cell can have r > 1.5. Skip these immediately (with r² > some
 *    cutoff²) to save the log/exp work on cells that will never have
 *    a star.
 *
 *  • ATAN2 BRANCH CUT. atan2 returns θ ∈ (−π, π]. The arm wrap
 *    (`α − seg · round(α/seg)`) takes care of branch-cut wrap, so
 *    you don't need to remap θ first. (Simple θ + 2π only when α
 *    range exceeds [-2π, 2π], which it doesn't here.)
 *
 *  • PROB SCALE. With density ≈ 1.0 at the bulge centre, the gate
 *    h_unit < density places stars in EVERY cell at the bulge —
 *    there are not enough free cells. Multiply density by a small
 *    constant (STAR_PROB_SCALE ≈ 0.15) so the *peak* per-cell prob
 *    is ~15 %; gives a satisfyingly dense bulge without saturation.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space): the galaxy freezes in place. Press space again:
 *    rotation resumes from the same angle. Verifies fixed-step time.
 *  • Press r: rotation snaps to angle 0 and the noise re-seeds. The
 *    overall galaxy SHAPE is unchanged (same density function), only
 *    the specific star positions rearrange. Verifies hash/density
 *    separation.
 *  • SPIRAL: count arms. Should be 4. The arms should be CURVED, not
 *    straight, with a clear sense of "winding" outward. If they look
 *    like spokes on a wheel, SPIRAL_PITCH is too high — the formula
 *    has degenerated into θ = constant lines.
 *  • BARRED: a bright bar should run horizontally through the centre
 *    (when angle = 0). Two arms should emerge from its END points,
 *    not from the centre. If they emerge from the centre, the bar
 *    factor isn't suppressing the inner arm region.
 *  • ELLIPTICAL: should look like a fuzzy round ball with no visible
 *    structure. If you see arms, the pattern dispatch is broken.
 *  • NEBULA: in the gaps between visible stars, you should see soft
 *    coloured cloud puffs that drift slowly (noise time advancing).
 *    The clouds should ONLY appear inside the visible disk, not in
 *    the outer halo or background.
 *  • Speed +/-: pressing + should make rotation noticeably faster.
 *    Pressing - should slow it down. The user-facing speed value in
 *    the HUD updates on each press. Halving the speed should halve
 *    the visible angular velocity exactly.
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

    /* Rotation-speed knob in arbitrary user units. The actual rad/sec
     * value is ROTATION_RATE × (speed / SPEED_DEF). */
    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Number of arms in the SPIRAL / NEBULA patterns. BARRED uses 2. */
    N_ARMS_SPIRAL       =   4,
    N_ARMS_BARRED       =   2,

    /* Color pair indices. PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_STAR_BASE      =   3,    /* +0..+3 = 4 star tints           */
    PAIR_NEBULA_BASE    =   7,    /* +0..+3 = 4 nebula tints         */
    PAIR_FLASH          =  11,    /* reset flash                     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/*
 * Cell-aspect correction. Terminal cells are ~2× taller than wide;
 * multiply the row offset by ASPECT_Y in the rendering math so the
 * galaxy looks circular instead of horizontally squashed.
 */
#define ASPECT_Y           2.0f

/*
 * Galaxy geometry in NORMALISED units where r = 1 is the disk edge.
 * Tweaking these values dramatically reshapes the galaxy:
 *   BULGE_SIGMA   smaller → tighter, hotter bulge
 *   DISK_SIGMA    larger  → arms reach further out
 *   DISK_R_MAX    hard cutoff radius (1.0 = full disk)
 *   SPIRAL_PITCH  b in r=a·exp(bθ); smaller = tighter winding
 *   ARM_WIDTH     half-width of an arm in arc-length units
 *   BULGE_AMP / DISK_AMP — relative weights of the two terms
 */
#define BULGE_SIGMA        0.13f
#define DISK_SIGMA         0.45f
#define DISK_R_MAX         1.00f
#define SPIRAL_PITCH       0.30f       /* b in r = a·exp(bθ)        */
#define ARM_WIDTH          0.20f
#define BULGE_AMP          0.95f
#define DISK_AMP           0.95f

/* BARRED-pattern bar dimensions (in normalised x/y units). The bar is
 * always oriented along the x-axis in the galaxy frame. */
#define BAR_LEN            0.36f
#define BAR_WIDTH          0.09f
#define BAR_AMP            0.85f

/* ELLIPTICAL-pattern halo radius. */
#define ELLIPTICAL_SIGMA   0.50f
#define ELLIPTICAL_AMP     0.55f

/* Noise — fBm parameters. NOISE_FREQ is the "feature frequency" in
 * normalised galaxy units; 1.5 gives lobes ~0.7 galaxy-radius wide. */
#define NOISE_FREQ         1.5f
#define NOISE_DRIFT        0.10f       /* noise-coord units / sec   */
#define ARM_NOISE_AMP      0.20f       /* ±20 % arm-width perturb   */
#define FBM_OCTAVES        4
#define NEBULA_THRESH      0.55f       /* min fbm to glow as cloud  */

/* Star-density gate. Density at peak ≈ 1.0; this scalar caps per-
 * cell star probability so the bulge doesn't saturate every cell. */
#define STAR_PROB_SCALE    0.18f

/* Rotation rate at speed = SPEED_DEF, in radians per second. The
 * galaxy completes one revolution every 2π / ROTATION_RATE seconds.
 * 0.06 → ~105 s per turn at default speed. */
#define ROTATION_RATE      0.06f

/*
 * Glyphs by density tier. Bright = peak (bulge core, arm centres);
 * mid = body of arms; dim = halo / arm edges.
 */
static const char STAR_BRIGHT[4] = { '*', 'O', '+', '#' };
static const char STAR_MID   [4] = { '*', '+', 'o', '.' };
static const char STAR_DIM   [4] = { '.', '`', ',', '\'' };

/*
 * Pattern — four ways to assemble the density field. Cycle with n / p.
 */
typedef enum {
    PATTERN_SPIRAL     = 0,
    PATTERN_BARRED     = 1,
    PATTERN_ELLIPTICAL = 2,
    PATTERN_NEBULA     = 3,
    N_PATTERNS         = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SPIRAL:     return "SPIRAL    ";
    case PATTERN_BARRED:     return "BARRED    ";
    case PATTERN_ELLIPTICAL: return "ELLIPTICAL";
    case PATTERN_NEBULA:     return "NEBULA    ";
    default:                 return "?         ";
    }
}

/*
 * Themes — same 10-name menu as the rest of the procedural showcases.
 * Star tints are ordered warm (bulge) → cool (halo). Nebula tints are
 * the cloud palette used in PATTERN_NEBULA.
 */
typedef struct {
    const char *name;
    short       star  [4];
    short       nebula[4];
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name      star{0,1,2,3}              nebula{0,1,2,3}             */
    { "DEFAULT", { 226, 230, 159,  39 }, { 162, 129, 105,  74 } },
    { "MATRIX",  { 230, 226, 118,  46 }, {  22,  28,  34,  40 } },
    { "NOVA",    { 231, 219, 201, 129 }, {  53,  91, 165, 207 } },
    { "MONO",    { 254, 250, 246, 240 }, { 234, 236, 238, 242 } },
    { "OCEAN",   { 231, 159,  51,  39 }, {  17,  18,  31,  44 } },
    { "FIRE",    { 231, 226, 208, 196 }, {  52,  88, 124, 160 } },
    { "EARTH",   { 230, 222, 173, 100 }, {  58,  64, 100, 137 } },
    { "FOREST",  { 231, 156, 118,  64 }, {  22,  28,  34,  65 } },
    { "DESERT",  { 230, 222, 173, 130 }, {  94, 130, 137, 173 } },
    { "ARCTIC",  { 231, 195, 159,  39 }, {  17,  18,  19,  24 } },
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
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_STAR_BASE   + i), t->star  [i], -1);
            init_pair((short)(PAIR_NEBULA_BASE + i), t->nebula[i], -1);
        }
    } else {
        static const short fb_star[4]   = { COLOR_WHITE,   COLOR_YELLOW,
                                            COLOR_CYAN,    COLOR_BLUE };
        static const short fb_nebula[4] = { COLOR_MAGENTA, COLOR_RED,
                                            COLOR_BLUE,    COLOR_CYAN };
        for (int i = 0; i < 4; i++) {
            init_pair((short)(PAIR_STAR_BASE   + i), fb_star  [i], -1);
            init_pair((short)(PAIR_NEBULA_BASE + i), fb_nebula[i], -1);
        }
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
/* §5  galaxy — hash, perlin/fbm, density(r, θ, pattern)                  */
/* ===================================================================== */

/*
 * hash3 — stateless 3-int → 32-bit avalanche hash.
 *
 * Standard spatial-hash multipliers (Teschner et al. 2003) followed by
 * a splitmix-style finaliser. Used to convert "quantised galaxy
 * coordinate + pattern index" into a uniform 32-bit value, from which
 * we extract a [0, 1) fraction for the star gate and high bits for
 * glyph selection.
 */
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
 * Perlin / fBm — copied inline per the self-contained-file rule.
 * See ../fields/perin_noise_flow_showcase.c for the full derivation.
 */
static uint8_t perm[512];

static void perm_shuffle(void)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

static inline float fade_q(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }
static inline float grad2(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

static float perlin2d(float x, float y)
{
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

static float fbm2(float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;          /* → [0, 1]      */
}

/* ----------------------------------------------------------------------- *
 * Density helpers.
 * ----------------------------------------------------------------------- */

/*
 * spiral_arc — arc-length distance from (r, θ) to the nearest of N
 * logarithmic-spiral arms.
 *
 * Derivation:
 *   The k-th arm of a log spiral satisfies θ = ln(r)/b + 2πk/N.
 *   Define α = θ − ln(r)/b. Modulo 2π/N, α tells you the angular
 *   distance to the nearest arm (k chosen automatically by wrapping).
 *   Multiply by r to get the arc-length distance — the "physical"
 *   distance along a circle of radius r.
 *
 * At very small r, ln(r) → −∞ so the arm phase becomes meaningless.
 * Clamp r to a small floor before evaluating; in practice the bulge
 * term swamps the arm contribution at the centre, so the result of
 * this function near r=0 has negligible visual effect.
 */
static float spiral_arc(float r, float theta, int n_arms)
{
    float r_safe = (r < 0.04f) ? 0.04f : r;
    float seg = 2.0f * (float)M_PI / (float)n_arms;
    float psi = logf(r_safe) / SPIRAL_PITCH;
    float alpha = theta - psi;
    alpha = alpha - seg * floorf(alpha / seg + 0.5f);   /* wrap to [-seg/2, +seg/2] */
    return alpha * r;
}

/*
 * arm_factor — Gaussian falloff in arc-length space, with the arm
 * width perturbed by ±ARM_NOISE_AMP based on a centred fBm sample.
 * The noise modulation breaks the perfect mathematical regularity
 * of the analytic spiral, giving the arms an organic, irregular look.
 */
static float arm_factor(float arc_dist, float fbm_centered)
{
    float w = ARM_WIDTH * (1.0f + ARM_NOISE_AMP * fbm_centered);
    if (w < 0.05f) w = 0.05f;
    return expf(-arc_dist * arc_dist / (w * w));
}

static inline float bulge_factor(float r)
{
    return expf(-r * r / (BULGE_SIGMA * BULGE_SIGMA));
}
static inline float disk_envelope(float r)
{
    return expf(-r * r / (DISK_SIGMA * DISK_SIGMA));
}

/*
 * bar_factor — elliptical Gaussian for the central bar of a barred
 * spiral. In the galaxy frame the bar lies along the x-axis (horizon-
 * tally when angle = 0); rotation of the whole frame turns it bodily.
 */
static float bar_factor(float r, float theta)
{
    float gx = r * cosf(theta);
    float gy = r * sinf(theta);
    return expf(-(gx * gx) / (BAR_LEN  * BAR_LEN )
                -(gy * gy) / (BAR_WIDTH * BAR_WIDTH));
}

/*
 * density_at — assemble the per-pattern density field. Input r is
 * normalised so the disk edge sits at r ≈ 1.0; θ is in radians;
 * fbm_val is a [0, 1] noise sample at the same point, used to
 * perturb arm widths (and, in NEBULA, paint the cloud overlay).
 *
 * Returns a value roughly in [0, 1]; multiplied by STAR_PROB_SCALE
 * to give the actual per-cell star probability.
 */
static float density_at(float r, float theta, float fbm_val, Pattern p)
{
    if (r > DISK_R_MAX) return 0.0f;
    float fbm_c = (fbm_val - 0.5f) * 2.0f;             /* [-1, 1]    */
    float bulge = bulge_factor(r);

    if (p == PATTERN_ELLIPTICAL) {
        float halo = expf(-r * r / (ELLIPTICAL_SIGMA * ELLIPTICAL_SIGMA));
        return bulge * 0.9f + halo * ELLIPTICAL_AMP;
    }

    if (p == PATTERN_BARRED) {
        float bar  = bar_factor(r, theta);
        float disk = disk_envelope(r);
        float arc  = spiral_arc(r, theta, N_ARMS_BARRED);
        float arm  = arm_factor(arc, fbm_c);
        /* Bar suppresses arms in the central region — multiply by
         * (1 - bar) so arms only emerge once you leave the bar. */
        return bulge * 0.55f
             + bar   * BAR_AMP
             + disk  * arm * (1.0f - bar) * DISK_AMP;
    }

    /* SPIRAL or NEBULA — both share the 4-arm structure. */
    {
        float disk = disk_envelope(r);
        float arc  = spiral_arc(r, theta, N_ARMS_SPIRAL);
        float arm  = arm_factor(arc, fbm_c);
        return bulge * BULGE_AMP + disk * arm * DISK_AMP;
    }
}

/*
 * star_color_idx — pick one of 4 star tints by RADIUS. Mimics the
 * stellar-population gradient of real galaxies: warm (yellow-orange)
 * older stars in the bulge, cool (blue-white) younger stars further
 * out. The boundaries are tuned to the Gaussian widths above.
 */
static int star_color_idx(float r)
{
    if      (r < 0.10f) return 0;     /* bulge core — warm           */
    else if (r < 0.30f) return 1;     /* inner disk — cream          */
    else if (r < 0.60f) return 2;     /* outer disk — white/blue     */
    else                return 3;     /* halo — faint                */
}

/* ===================================================================== */
/* §6  scene — state + tick                                               */
/* ===================================================================== */

/*
 * Scene — the entire animated state. There is no per-cell array,
 * no entity pool. Every frame the renderer derives the picture from
 * just these few floats and the active pattern.
 *
 *   angle           : galaxy rotation (radians, accumulating)
 *   noise_time      : drifts fBm so the cloud / arm-perturbation
 *                     pattern slowly evolves
 *   time_secs       : wall clock for the reset-flash overlay
 *   paused          : freeze-on-space
 *   speed           : 1..SPEED_MAX user knob; ω scales linearly
 *   current_theme   : 0..N_THEMES-1
 *   current_pattern : SPIRAL / BARRED / ELLIPTICAL / NEBULA
 *   flash_t         : 0..1 reset-flash intensity, decays each tick
 */
typedef struct {
    float   angle;
    float   noise_time;
    float   time_secs;
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   flash_t;
} Scene;

static void scene_reset(Scene *s)
{
    s->angle      = 0.0f;
    s->noise_time = 0.0f;
    s->time_secs  = 0.0f;
    s->flash_t    = 1.0f;
    perm_shuffle();
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_SPIRAL;
    scene_reset(s);
}

/*
 * scene_tick — advance rotation and noise drift. The entire continuous-
 * state portion of the simulation lives here; rendering is purely a
 * function of (angle, noise_time, pattern, theme).
 *
 * Rigid rotation is used, NOT differential — see the MENTAL MODEL
 * "winding problem" note for why density-wave galaxies do not actually
 * have rigidly-rotating arms in nature.
 */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    s->flash_t   *= expf(-4.0f * dt);
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->angle      += ROTATION_RATE * speed_mul * dt;
    s->noise_time += NOISE_DRIFT  * speed_mul * dt;
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — terminal dimensions plus the derived galaxy-frame layout.
 *   cx, cy : galaxy centre, in screen-cell coordinates
 *   r0     : galaxy half-width, in HORIZONTAL cells. The vertical
 *            extent on screen is r0/ASPECT_Y cells, so the rendered
 *            galaxy is 2·r0 wide and r0 tall.
 */
typedef struct {
    int cols, rows;
    int cx, cy, r0;
} Screen;

static void screen_layout(Screen *s)
{
    s->cx = s->cols / 2;
    /* Centre vertically inside the strip below the 2-row HUD and
     * above the 1-row hint. */
    int top = 2, bottom = s->rows - 1;
    s->cy = (top + bottom) / 2;

    int max_h = (s->cols - 2) / 2;
    int max_v = (int)(((float)(bottom - top - 1)) * ASPECT_Y / 2.0f);
    s->r0 = (max_h < max_v) ? max_h : max_v;
    if (s->r0 < 8) s->r0 = 8;
}

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
    screen_layout(s);
}
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
    screen_layout(s);
}

/*
 * scene_draw — per-cell density gating, glyph + colour selection, draw.
 *
 * Per cell:
 *   1. Translate to galaxy-centre, apply aspect correction (y × 2).
 *   2. Rotate INTO the galaxy frame by inverse(angle).
 *   3. Normalise by R0 → unit-radius galaxy coordinates.
 *   4. Cheap rejection: if r² > DISK_R_MAX² + ε, skip.
 *   5. Sample fBm for arm-width perturbation (and NEBULA cloud).
 *   6. Compute density per pattern.
 *   7. Hash quantised galaxy coords for star gate; on hit, pick
 *      colour by radius and glyph/attr by density tier.
 *   8. NEBULA pattern only: empty cells with density > 0.05 and
 *      fbm > NEBULA_THRESH render a cloud glyph in the nebula tint.
 *
 * The whole function is a pure projection of (angle, noise_time,
 * pattern) onto the screen — there is no state mutation.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int top    = 2;
    int bottom = sc->rows - 1;

    float cosA = cosf(s->angle);
    float sinA = sinf(s->angle);

    bool pat_nebula = (s->current_pattern == PATTERN_NEBULA);

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {

            /* (a) screen-cell offset → physical (aspect-corrected) */
            float fx = (float)(sx - sc->cx);
            float fy = (float)(sy - sc->cy) * ASPECT_Y;

            /* (b) rotate into the galaxy reference frame */
            float gx_phys =  fx * cosA + fy * sinA;
            float gy_phys = -fx * sinA + fy * cosA;

            /* (c) normalise to galaxy-radius units */
            float gnx = gx_phys / (float)sc->r0;
            float gny = gy_phys / (float)sc->r0;

            float r2 = gnx * gnx + gny * gny;
            if (r2 > DISK_R_MAX * DISK_R_MAX + 0.05f) continue;

            float r     = sqrtf(r2);
            float theta = atan2f(gny, gnx);

            /* (d) fBm — feeds both arm perturbation and nebula */
            float fbm_val = fbm2(gnx * NOISE_FREQ,
                                 gny * NOISE_FREQ + s->noise_time);

            /* (e) density */
            float dens = density_at(r, theta, fbm_val, s->current_pattern);
            if (dens < 0.01f) continue;

            /* (f) hash gate. Quantise gx_phys / gy_phys so that one
             *     "galaxy world cell" ≈ one screen cell (with aspect
             *     correction so the integer cells stay roughly square).
             *     Pattern index salts the hash so each pattern has a
             *     distinct star arrangement. */
            int qx = (int)floorf(gx_phys);
            int qy = (int)floorf(gy_phys / ASPECT_Y);
            uint32_t h = hash3(qx, qy, (int)s->current_pattern);

            float h_unit = (float)(h & 0xFFFFFFu) / 16777216.0f;
            float prob   = dens * STAR_PROB_SCALE;

            if (h_unit < prob) {
                /* STAR */
                int  color_idx = star_color_idx(r);
                int  glyph_idx = (int)((h >> 8) & 3u);
                int  attr;
                char glyph;
                if      (dens > 0.55f) { attr = A_BOLD;   glyph = STAR_BRIGHT[glyph_idx]; }
                else if (dens > 0.20f) { attr = A_NORMAL; glyph = STAR_MID   [glyph_idx]; }
                else                    { attr = A_DIM;    glyph = STAR_DIM   [glyph_idx]; }

                int pair = PAIR_STAR_BASE + color_idx;
                attron(COLOR_PAIR(pair) | attr);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | attr);
                continue;
            }

            /* (g) NEBULA cloud overlay. Empty cells inside the disk
             *     with high fbm pick up a coloured cloud glyph. */
            if (pat_nebula && dens > 0.05f && fbm_val > NEBULA_THRESH) {
                float intensity = (fbm_val - NEBULA_THRESH)
                                / (1.0f - NEBULA_THRESH);
                int   color_idx = (int)((h >> 16) & 3u);
                int   attr;
                char  glyph;
                if      (intensity > 0.65f) { attr = A_BOLD;   glyph = '#'; }
                else if (intensity > 0.30f) { attr = A_NORMAL; glyph = '*'; }
                else                         { attr = A_DIM;    glyph = '.'; }

                int pair = PAIR_NEBULA_BASE + color_idx;
                attron(COLOR_PAIR(pair) | attr);
                mvaddch(sy, sx, (chtype)(unsigned char)glyph);
                attroff(COLOR_PAIR(pair) | attr);
            }
        }
    }

    /* Reset flash — sparse '*' overlay that fades quickly. */
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

/*
 * screen_draw — clear, draw scene, draw HUD.
 *   row 0 : title (left) + fps/Hz/state/speed (right)
 *   row 1 : pattern + theme + 4-colour palettes + r0
 *   bottom: key-hint strip (PAIR_HINT, A_BOLD)
 */
static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s);

    const char *state_str = s->paused
                          ? "PAUSED    "
                          : pattern_name(s->current_pattern);

    /* row 0 right: status */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* row 0 left: title */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " PROCEDURAL GALAXY ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* row 1 left: pattern + theme + palettes + radius */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-10s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 21;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " stars:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 7;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_STAR_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '*');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " neb:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 5;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_NEBULA_BASE + i;
        attron(COLOR_PAIR(p));
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p));
        x++;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  r0:%-3d  arms:%d  pitch:%.2f ",
             sc->r0,
             s->current_pattern == PATTERN_BARRED ? N_ARMS_BARRED : N_ARMS_SPIRAL,
             SPIRAL_PITCH);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* bottom: key-hint strip */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:reset  q:quit ");
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
    case 'r': case 'R': scene_reset(s);                                break;

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
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
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
