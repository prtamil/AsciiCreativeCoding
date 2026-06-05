/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_galaxy.c
 *   — A rotating procedural galaxy: logarithmic spiral arms perturbed
 *     by Perlin/fBm noise, sampled per cell with no stored stars.
 *
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
 * Section map (re-cut by CONCERN — see ARCHITECTURE block below):
 *   §1 CONFIG       — constants, data tables (glyphs, themes), Pattern enum
 *   §2 PERFORMANCE  — timing primitives (throttle policy lives in main)
 *   §3 LOGIC        — pure: hash, perlin/fbm, the whole density field, colour
 *   §4 SIMULATION   — scene_tick (rotation + noise drift) + reset/reseed
 *   §5 EFFECTS      — cosmetic-only state (one-line note: none stored)
 *   §6 DELAYS       — pauses, holds, timers (one-line note: pause only)
 *   §7 RENDER       — per-cell density projection + HUD; reads only
 *   §8 APP          — user events + per-tick combine + main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset angle, reseed noise
 *   n / N      next / prev pattern  (cycles all 15 morphologies)
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
 *                  In the dusty patterns (NEBULA / FLOCCULENT / STARBURST)
 *                  the fBm value, gated on density, paints '#'/'*'/'.'
 *                  cloud glyphs in a separate 4-tint palette.
 *
 * Performance    : O(W·H) per frame. For each cell:
 *                    1 cos/sin + 1 sqrt + 1 atan2 + 1 log + 1 exp +
 *                    fbm (4 perlin = 4·fade + 4·lerps) + hash
 *                  ≈ 350 ns per cell on a current CPU. At 240×80×60fps
 *                  ≈ 4 ms/frame ≈ 24 % of one core. No allocation in
 *                  the loop, no I/O, no branches that depend on state.
 *
 * References     :
 *
 *   Galaxy structure & morphology (the concepts) —
 *   • Wikipedia — "Galaxy morphological classification". The Hubble sequence
 *     (elliptical → lenticular → spiral / barred → irregular) plus ring and
 *     special types: the map the 15 presets are drawn from.
 *     https://en.wikipedia.org/wiki/Galaxy_morphological_classification
 *   • Wikipedia — "Logarithmic spiral" (the arm curve r = a·e^{bθ})
 *     https://en.wikipedia.org/wiki/Logarithmic_spiral
 *   • Wikipedia — "Spiral galaxy" / pitch angle
 *     https://en.wikipedia.org/wiki/Spiral_galaxy
 *   • Lin & Shu (1964) — "On the spiral structure of disk galaxies", ApJ 140,
 *     646: density-wave theory, why arms persist. See also
 *     https://en.wikipedia.org/wiki/Density_wave_theory
 *   • Sérsic (1963) bulge + Freeman (1970) exponential-disk surface-brightness
 *     laws — what the radial bulge_factor / disk_envelope approximate (with
 *     Gaussians here, for cheapness).  https://en.wikipedia.org/wiki/Sersic_profile
 *
 *   Procedural noise & rendering —
 *   • Perlin, K. (2002) — "Improving Noise" (the quintic-fade gradient noise)
 *     https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *   • Inigo Quilez — "Painting a galaxy" (closed-form procedural galaxy shader)
 *     https://iquilezles.org/articles/warp/
 *   • Red Blob Games — "Making maps with noise functions"
 *     https://www.redblobgames.com/articles/noise/introduction.html
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

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * Re-cut from first principles into separated concern-layers (a SEPARATION
 * pass: RELOCATE + LABEL only — every function body is byte-identical, nothing
 * renamed). This galaxy is unusual in that almost all the "algorithm" is PURE:
 * the picture is a stateless function of three numbers, so §3 LOGIC is large
 * and §4 SIMULATION is tiny. Layer → section → what it mutates:
 *
 *   LAYER        §   MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   CONFIG       §1  nothing — compile-time constants + const data tables
 *                    (star/dust glyph rows, themes) + the Pattern enum.
 *   PERFORMANCE  §2  nothing — clock_ns / clock_sleep_ns are pure timers; the
 *                    frame cap + fixed-timestep accumulator are POLICY in main.
 *   LOGIC        §3  nothing — the spatial hash (+ its hash_unit [0,1) roll),
 *                    the Perlin/fBm samplers, the entire density field
 *                    (spiral/arm/bulge/disk/ring/bar → density_at), and the pure
 *                    cell deciders (star_color_idx radius→tint, pattern_has_dust).
 *                    All pure; perm[] is a stable table reseeded only from §4, so
 *                    no RENDER/EFFECTS reorder can corrupt a LOGIC result.
 *   SIMULATION   §4  Scene.{angle,noise_time,paused,speed,current_theme,
 *                    current_pattern} + the global perm[] noise table. scene_tick
 *                    advances rotation+drift; scene_reset/init/perm_shuffle seed.
 *                    The ONLY writers of sim state.
 *   EFFECTS      §5  (none) — no stored cosmetic buffer; the dust glow is
 *                    derived at render time. One-line section, not a real layer.
 *   DELAYS       §6  (none) — only Scene.paused (early-returns scene_tick); no
 *                    holds/dwells, the galaxy rotates continuously.
 *   RENDER       §7  ncurses back buffer + colour-pair table only (theme_apply,
 *                    color_init, scene_draw + draw_star/dust_cell, screen_draw +
 *                    the HUD draw_* helpers). Reads §4 state and re-evaluates §3
 *                    per cell; never writes simulation state.
 *   APP          §8  App.{running,need_resize,sim_fps}; drives Scene via the
 *                    combine + user events.
 *
 * PER-TICK COMBINE (the one place state advances — main(), §8):
 *
 *     while (sim_accum >= tick_ns)        // PERFORMANCE: fixed timestep
 *         scene_tick()                    //   SIMULATION (rotation + drift)
 *     scene_draw() ; screen_draw()        // RENDER (reads only; density re-derived)
 *     screen_present()
 *     getch() → app_handle_key()          // USER EVENTS — see below
 *
 * Nothing other than scene_tick() advances simulation state. User events
 * (app_handle_key / app_do_resize) mutate Scene/Screen on a keypress or
 * SIGWINCH — and 'r' reseeds the noise via perm_shuffle — but they run once per
 * frame OUTSIDE the accumulator loop, not as part of the tick.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ===================================================================== */
/* §1  CONFIG  -- constants, data tables (glyphs, themes), Pattern enum  */
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

/*
 * Extra-pattern parameters — the wider 15-galaxy catalogue. Each new
 * morphology reuses the primitives (bulge / disk / spiral / bar / ring /
 * noise) with its own arm count, winding pitch and width.
 *   PITCH smaller  → more tightly wound arms (TIGHT < PINWHEEL < default < GRAND)
 *   RING_*         → bright Gaussian shell radius/width (RING, CARTWHEEL)
 *   SOMBRERO_THIN  → vertical half-thickness of an edge-on disk
 *   NUCLEUS_SIGMA  → tiny brilliant active nucleus (SEYFERT)
 */
#define N_ARMS_GRAND       2           /* two bold "grand design" arms (M51) */
#define N_ARMS_PINWHEEL    6           /* many fine arms (M101)              */
#define PITCH_GRAND        0.34f       /* loosely wound, sweeping            */
#define PITCH_PINWHEEL     0.22f       /* tighter                            */
#define PITCH_TIGHT        0.15f       /* very tightly coiled               */
#define ARM_WIDTH_GRAND    0.30f       /* wide, high-contrast arms           */
#define RING_R             0.58f       /* ring-galaxy shell radius           */
#define RING_W             0.12f       /* ring shell half-width              */
#define SOMBRERO_THIN      0.10f       /* edge-on disk vertical half-thick   */
#define NUCLEUS_SIGMA      0.045f      /* Seyfert point-nucleus radius       */

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

/* Per-cell render thresholds (density d ∈ [0,1], dust intensity ∈ [0,1]). */
#define CELL_CULL_MARGIN   0.05f       /* slack on the r² disk-cull test     */
#define DENS_EMPTY         0.01f       /* below this a cell is empty space    */
#define STAR_DENS_BRIGHT   0.55f       /* density → glyph brightness tiers    */
#define STAR_DENS_MID      0.20f
#define DUST_DENS_MIN      0.05f       /* min density for a dust-cloud cell   */
#define DUST_INTENS_BRIGHT 0.65f       /* dust fbm-intensity → glyph tiers     */
#define DUST_INTENS_MID    0.30f

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

/* ── Pattern ───────────────────────────────────────────────────────────── *
 * Fifteen ways to assemble the SAME density field. The intent: a galaxy's
 * "type" is not different data — it is a different closed-form recipe over one
 * shared set of primitives (bulge + disk + log-spiral arms + bar + ring +
 * noise). So a morphology is just a branch in density_at(), and switching
 * patterns costs nothing (no rebuild, no allocation). Each value is a real
 * galaxy class from the Hubble sequence + special types, differing only in arm
 * count, winding pitch, ring/bar/edge-on geometry and how much noise it mixes
 * in. The enum value also salts the star-placement hash, so each pattern gets a
 * distinct star arrangement. The first four keep their original indices (and
 * exact look); the rest extend the catalogue.
 * REFS: Hubble sequence — Wikipedia "Galaxy morphological classification";
 *       see the file-header References block.
 */
typedef enum {
    PATTERN_SPIRAL     = 0,   /* 4-arm logarithmic spiral (the classic)   */
    PATTERN_BARRED,           /* central bar + 2 trailing arms            */
    PATTERN_ELLIPTICAL,       /* smooth featureless Gaussian blob (E0-ish)*/
    PATTERN_NEBULA,           /* 4-arm spiral + glowing dust clouds       */
    PATTERN_GRAND,            /* 2 bold sweeping arms (grand design, M51) */
    PATTERN_PINWHEEL,         /* 6 fine arms (M101)                       */
    PATTERN_TIGHT,            /* 2 very tightly coiled arms               */
    PATTERN_FLOCCULENT,       /* patchy, noise-fragmented arms            */
    PATTERN_LENTICULAR,       /* bulge + smooth armless disk (S0)         */
    PATTERN_RING,             /* bright ring, faint centre (Hoag's Object)*/
    PATTERN_CARTWHEEL,        /* ring + central bar spokes (collision)    */
    PATTERN_SOMBRERO,         /* edge-on disk + bulge + dust lane         */
    PATTERN_STARBURST,        /* blazing core + patchy star-forming bursts*/
    PATTERN_SEYFERT,          /* brilliant point nucleus + faint disk (AGN)*/
    PATTERN_IRREGULAR,        /* clumpy, asymmetric (dwarf irregular)     */
    N_PATTERNS,
} Pattern;

/* ── Theme ─────────────────────────────────────────────────────────────── *
 * WHAT  One named colour palette (10 ship; t/T cycles). theme_apply() loads its
 *       codes into the ncurses pairs. Same 10-name menu as the sibling showcases.
 *
 * VALUE LOGIC  Two 4-entry ramps, indexed the way the renderer buckets a cell:
 *       star[4]   — by RADIUS, ordered warm → cool to mimic the real stellar-
 *                   population gradient (old yellow-red bulge stars in the
 *                   centre, young blue-white disk stars outward; star[0]=core …
 *                   star[3]=halo). Indexed by star_color_idx(r).
 *       nebula[4] — the dust-cloud palette, indexed by hash bits, used only by
 *                   the dusty patterns (NEBULA / FLOCCULENT / STARBURST).
 *       Every code sits in the bright half of the 256-colour cube so even A_DIM
 *       cells stay legible on default-black. REFS: documentation/COLOR.md. */
typedef struct {
    const char *name;       /* HUD label (t/T cycles)                       */
    short       star  [4];  /* star tints, core→halo (by star_color_idx)    */
    short       nebula[4];  /* dust-cloud tints (dusty patterns only)       */
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
/* §2  PERFORMANCE  -- timing primitives (throttle policy in main, §8)   */
/* ===================================================================== */

/* Timing primitives only. The 60 fps frame cap and the fixed-timestep
 * accumulator that decide how many scene_tick()s run per frame are POLICY,
 * applied in main() (§8). */

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
/* §3  LOGIC  -- pure decisions: hash, noise, density field, colour      */
/* ===================================================================== */

/* Pure functions — the bulk of the algorithm. Given their arguments (and the
 * read-only noise table / theme data) each returns a value with NO mutation
 * and NO I/O: the spatial hash (+ hash_unit, its [0,1) roll), the Perlin/fBm
 * samplers, the whole density field (spiral / arm / bulge / disk / ring / bar
 * factors → density_at), and the pure cell deciders (star_color_idx,
 * pattern_has_dust). The glyph-picking + drawing itself lives in §7.
 *
 * The one piece of data here, perm[], is a stable lookup table reseeded ONLY
 * by perm_shuffle (§4, on reset) — never by RENDER/EFFECTS — so a frame's
 * render order cannot change any LOGIC result. density_at is evaluated fresh
 * per cell by the renderer; it is decision, not stored state. */

/* 10-char padded names so the HUD field width stays fixed. */
static const char *pattern_name(Pattern p)
{
    static const char *names[N_PATTERNS] = {
        "SPIRAL    ", "BARRED    ", "ELLIPTICAL", "NEBULA    ", "GRANDDSGN ",
        "PINWHEEL  ", "TIGHTCOIL ", "FLOCCULENT", "LENTICULAR", "RING      ",
        "CARTWHEEL ", "SOMBRERO  ", "STARBURST ", "SEYFERT   ", "IRREGULAR ",
    };
    return ((int)p >= 0 && (int)p < N_PATTERNS) ? names[p] : "?         ";
}

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

/* Uniform fraction in [0, 1) from the low 24 bits of a hash (24 bits → exactly
 * representable as a float). This is the roll the per-cell star gate compares
 * against the star probability. */
static inline float hash_unit(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) / 16777216.0f;     /* 2^24 */
}

/*
 * Perlin / fBm — copied inline per the self-contained-file rule.
 * See ../fields/perin_noise_flow_showcase.c for the full derivation.
 *
 * perm[] — the noise permutation table: a shuffle of 0..255 DUPLICATED into a
 * 512-entry array (perm[i] == perm[i+256]). The doubling is the classic Perlin
 * trick: gradient lookups add two indices in 0..255 and read perm[A+1] etc.,
 * which can reach 510 — duplicating lets that happen without a wrap/mask in the
 * hot path. Reseeded by perm_shuffle (§4 SIMULATION) on reset; read-only to the
 * §3 samplers, which is why the noise field is deterministic between reseeds.
 */
static uint8_t perm[512];

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
/* spiral_arc_p — like spiral_arc but with an explicit winding pitch, so the
 * tighter (TIGHT, PINWHEEL) and looser (GRAND) spirals can share the math. */
static float spiral_arc_p(float r, float theta, int n_arms, float pitch)
{
    float r_safe = (r < 0.04f) ? 0.04f : r;
    float seg = 2.0f * (float)M_PI / (float)n_arms;
    float psi = logf(r_safe) / pitch;
    float alpha = theta - psi;
    alpha = alpha - seg * floorf(alpha / seg + 0.5f);   /* wrap to [-seg/2, +seg/2] */
    return alpha * r;
}
static float spiral_arc(float r, float theta, int n_arms)
{
    return spiral_arc_p(r, theta, n_arms, SPIRAL_PITCH);
}

/*
 * arm_factor_w — Gaussian falloff in arc-length space at an explicit base
 * width, with the width perturbed by ±ARM_NOISE_AMP from a centred fBm sample.
 * The noise modulation breaks the perfect mathematical regularity of the
 * analytic spiral, giving the arms an organic, irregular look.
 */
static float arm_factor_w(float arc_dist, float fbm_centered, float base_width)
{
    float w = base_width * (1.0f + ARM_NOISE_AMP * fbm_centered);
    if (w < 0.05f) w = 0.05f;
    return expf(-arc_dist * arc_dist / (w * w));
}
static float arm_factor(float arc_dist, float fbm_centered)
{
    return arm_factor_w(arc_dist, fbm_centered, ARM_WIDTH);
}

static inline float bulge_factor(float r)
{
    return expf(-r * r / (BULGE_SIGMA * BULGE_SIGMA));
}
static inline float disk_envelope(float r)
{
    return expf(-r * r / (DISK_SIGMA * DISK_SIGMA));
}

/* ring_factor — a bright Gaussian shell peaking at radius ring_r (for ring
 * galaxies like Hoag's Object and the collisional Cartwheel). */
static inline float ring_factor(float r, float ring_r, float ring_w)
{
    float d = r - ring_r;
    return expf(-d * d / (ring_w * ring_w));
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
    float patch = 0.5f + 0.5f * fbm_c;                 /* [ 0, 1] noise gate */
    float bulge = bulge_factor(r);
    float disk  = disk_envelope(r);

    switch (p) {

    case PATTERN_ELLIPTICAL: {
        float halo = expf(-r * r / (ELLIPTICAL_SIGMA * ELLIPTICAL_SIGMA));
        return bulge * 0.9f + halo * ELLIPTICAL_AMP;
    }

    case PATTERN_BARRED: {
        float bar  = bar_factor(r, theta);
        float arc  = spiral_arc(r, theta, N_ARMS_BARRED);
        float arm  = arm_factor(arc, fbm_c);
        /* Bar suppresses arms in the central region — multiply by
         * (1 - bar) so arms only emerge once you leave the bar. */
        return bulge * 0.55f
             + bar   * BAR_AMP
             + disk  * arm * (1.0f - bar) * DISK_AMP;
    }

    case PATTERN_GRAND: {           /* 2 bold, wide, loosely-wound arms */
        float arm = arm_factor_w(spiral_arc_p(r, theta, N_ARMS_GRAND, PITCH_GRAND),
                                 fbm_c, ARM_WIDTH_GRAND);
        return bulge * BULGE_AMP + disk * arm * DISK_AMP * 1.10f;
    }

    case PATTERN_PINWHEEL: {        /* 6 fine, tighter arms */
        float arm = arm_factor(spiral_arc_p(r, theta, N_ARMS_PINWHEEL, PITCH_PINWHEEL),
                               fbm_c);
        return bulge * 0.70f + disk * arm * DISK_AMP;
    }

    case PATTERN_TIGHT: {           /* 2 very tightly coiled arms */
        float arm = arm_factor(spiral_arc_p(r, theta, 2, PITCH_TIGHT), fbm_c);
        return bulge * BULGE_AMP + disk * arm * DISK_AMP;
    }

    case PATTERN_FLOCCULENT: {      /* 4 arms shredded by noise into patches */
        float arm = arm_factor(spiral_arc(r, theta, N_ARMS_SPIRAL), fbm_c);
        return bulge * 0.60f + disk * arm * patch * DISK_AMP * 1.30f;
    }

    case PATTERN_LENTICULAR:        /* smooth bulge + armless disk (S0) */
        return bulge * BULGE_AMP + disk * DISK_AMP * 0.55f;

    case PATTERN_RING: {            /* bright shell, faint centre */
        float ring = ring_factor(r, RING_R, RING_W);
        return bulge * 0.22f + ring * DISK_AMP;
    }

    case PATTERN_CARTWHEEL: {       /* ring + central bar spokes */
        float ring = ring_factor(r, RING_R, RING_W);
        float bar  = bar_factor(r, theta);
        return bulge * 0.40f + ring * DISK_AMP + bar * BAR_AMP * 0.70f;
    }

    case PATTERN_SOMBRERO: {        /* edge-on: thin disk + bulge + dust lane */
        float gx   = r * cosf(theta);
        float gy   = r * sinf(theta);
        float thin = expf(-(gy * gy) / (SOMBRERO_THIN * SOMBRERO_THIN));
        float span = expf(-(gx * gx) / (DISK_SIGMA * DISK_SIGMA));
        return bulge * BULGE_AMP + thin * span * DISK_AMP;
    }

    case PATTERN_STARBURST: {       /* blazing core + patchy bursts */
        float arm = arm_factor(spiral_arc(r, theta, N_ARMS_SPIRAL), fbm_c);
        return bulge * 1.10f + disk * arm * patch * DISK_AMP * 1.10f;
    }

    case PATTERN_SEYFERT: {         /* brilliant point nucleus + faint disk */
        float nucleus = expf(-r * r / (NUCLEUS_SIGMA * NUCLEUS_SIGMA));
        float arm = arm_factor(spiral_arc(r, theta, N_ARMS_SPIRAL), fbm_c);
        return nucleus * 1.20f + bulge * 0.30f + disk * arm * DISK_AMP * 0.50f;
    }

    case PATTERN_IRREGULAR:         /* clumpy, asymmetric — noise-driven */
        return disk * patch * patch * DISK_AMP * 1.40f + bulge * 0.20f;

    case PATTERN_SPIRAL:
    case PATTERN_NEBULA:
    default: {                      /* both share the classic 4-arm structure */
        float arc  = spiral_arc(r, theta, N_ARMS_SPIRAL);
        float arm  = arm_factor(arc, fbm_c);
        return bulge * BULGE_AMP + disk * arm * DISK_AMP;
    }
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

/* Which morphologies paint the glowing dust-cloud overlay (the NEBULA effect):
 * the nebula itself plus the two patchy, gas-rich types. */
static bool pattern_has_dust(Pattern p)
{
    return p == PATTERN_NEBULA || p == PATTERN_FLOCCULENT || p == PATTERN_STARBURST;
}

/* ===================================================================== */
/* §4  SIMULATION  -- advances state (only writers of sim state)         */
/* ===================================================================== */

/* The ONLY writers of simulation state. scene_tick advances the two continuous
 * floats — the rotation angle and the noise-time drift — once per tick;
 * scene_reset / scene_init (with perm_shuffle, the noise reseed) set them on
 * reset. Mutates: Scene.{angle,noise_time,paused,speed,current_theme,
 * current_pattern} and the global perm[] table. scene_tick() is the single
 * per-tick entry point (called only from main, §8); user events (key/resize)
 * also mutate Scene but are NOT part of the tick -- see §8. */

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

/*
 * Scene — the entire animated state, as a table of contents. There is no
 * per-cell array and no entity pool: the whole galaxy is a pure function of
 * these few numbers (the renderer re-derives every pixel each frame). Fields
 * group by concept, not by which key changes them. scene_tick() (§4) is the
 * only per-tick writer; user events set the knobs/selections.
 */
typedef struct {
    /* WHAT is simulated — the galaxy's two evolving phases */
    float   angle;            /* rotation, radians, accumulating          */
    float   noise_time;       /* fBm drift phase — clouds/arms slowly move */
    /* HOW the user drives it — the tunable simulation knob */
    int     speed;            /* 1..SPEED_MAX; rotation + drift scale by it */
    /* WHAT we are looking at — RENDER selections, not simulation */
    Pattern current_pattern;  /* active morphology (n/p); salts the star hash */
    int     current_theme;    /* index into themes[] (t/T)                */
    /* WHEN we are — run-state */
    bool    paused;           /* freeze rotation/drift (render continues)  */
} Scene;

static void scene_reset(Scene *s)
{
    s->angle      = 0.0f;
    s->noise_time = 0.0f;
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
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->angle      += ROTATION_RATE * speed_mul * dt;
    s->noise_time += NOISE_DRIFT  * speed_mul * dt;
}

/* ===================================================================== */
/* §5  EFFECTS  -- cosmetic-only state                                   */
/* ===================================================================== */

/* No EFFECTS layer. Nothing cosmetic is stored: the NEBULA / FLOCCULENT /
 * STARBURST dust glow is derived at render time from the fBm sample inside
 * scene_draw (§7), gated on density — there is no buffer to advance. (The
 * yellow reset flash was removed earlier, taking its flash_t state with it.) */

/* ===================================================================== */
/* §6  DELAYS  -- pauses, holds, timers                                  */
/* ===================================================================== */

/* No separate layer. The only timing control is the pause toggle
 * (Scene.paused), which early-returns scene_tick (§4). There are no holds or
 * per-phase dwells — the galaxy rotates continuously. */

/* ===================================================================== */
/* §7  RENDER  -- state -> screen (reads only, never mutates sim)        */
/* ===================================================================== */

/* state -> screen. Each frame scene_draw projects the simulation (angle,
 * noise_time, pattern) onto the grid by evaluating the §3 density/noise
 * functions per cell; screen_draw lays the HUD over it; theme_apply /
 * color_init load the colour table. Reads Scene / Screen + §3 LOGIC; writes
 * ONLY the ncurses back buffer and colour pairs, never simulation state. */

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
    } else {
        init_pair(PAIR_HUD,   COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,  COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── Screen ────────────────────────────────────────────────────────────── *
 * The terminal viewport plus the derived galaxy-frame mapping — pure
 * presentation geometry, holding NO simulation state. Recomputed by
 * screen_layout() at startup and on every SIGWINCH. It is the bridge the
 * renderer inverts per cell: screen cell (sx,sy) → physical offset from the
 * centre → (÷ r0) → unit-radius galaxy coords (r ≤ 1 at the disk edge) that
 * feed density_at(). r0 is measured in HORIZONTAL cells; because terminal cells
 * are ~2× tall (ASPECT_Y), the vertical extent is r0/ASPECT_Y, so the disk reads
 * as a circle, 2·r0 wide and r0 tall. */
typedef struct {
    int cols, rows;     /* full terminal size (getmaxyx)                    */
    int cx, cy;         /* galaxy centre, in screen-cell coords             */
    int r0;             /* galaxy half-width in horizontal cells (the r=1 scale) */
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

/* Draw one STAR cell: tint by radius (stellar-population gradient), glyph and
 * brightness by density tier. Hash bits choose the glyph variant. */
static void draw_star_cell(int sy, int sx, float r, float dens, uint32_t h)
{
    int  glyph_idx = (int)((h >> 8) & 3u);
    int  attr;
    char glyph;
    if      (dens > STAR_DENS_BRIGHT) { attr = A_BOLD;   glyph = STAR_BRIGHT[glyph_idx]; }
    else if (dens > STAR_DENS_MID)    { attr = A_NORMAL; glyph = STAR_MID   [glyph_idx]; }
    else                              { attr = A_DIM;    glyph = STAR_DIM   [glyph_idx]; }

    int pair = PAIR_STAR_BASE + star_color_idx(r);
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/* Draw one DUST cell of the cloud overlay: glyph by fBm intensity above the
 * cloud threshold, tint chosen from the nebula palette by hash bits. */
static void draw_dust_cell(int sy, int sx, float fbm_val, uint32_t h)
{
    float intensity = (fbm_val - NEBULA_THRESH) / (1.0f - NEBULA_THRESH);  /* → [0,1] */
    int   attr;
    char  glyph;
    if      (intensity > DUST_INTENS_BRIGHT) { attr = A_BOLD;   glyph = '#'; }
    else if (intensity > DUST_INTENS_MID)    { attr = A_NORMAL; glyph = '*'; }
    else                                     { attr = A_DIM;    glyph = '.'; }

    int pair = PAIR_NEBULA_BASE + (int)((h >> 16) & 3u);
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * scene_draw — project the galaxy onto the screen. For each cell: map it into
 * the galaxy frame (centre + aspect-correct → rotate by inverse(angle) →
 * normalise by r0), cull cells outside the disk, sample the fBm-perturbed
 * density field, then either place a star (hash gate vs density) or, for dusty
 * morphologies, a cloud glyph. A pure projection of (angle, noise_time,
 * pattern) onto the screen — no state mutation.
 *
 * Render leaf, NOT a tick orchestrator: takes only the three numbers it draws
 * from, never the whole Scene.
 */
static void scene_draw(const Screen *sc, float angle, float noise_time, Pattern pattern)
{
    int top    = 2;
    int bottom = sc->rows - 1;

    float cosA = cosf(angle);
    float sinA = sinf(angle);

    bool has_dust = pattern_has_dust(pattern);

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {

            /* Project screen cell → galaxy frame: centre + aspect-correct,
             * rotate by inverse(angle), normalise by r0 to unit-radius coords. */
            float fx = (float)(sx - sc->cx);
            float fy = (float)(sy - sc->cy) * ASPECT_Y;
            float gx_phys =  fx * cosA + fy * sinA;
            float gy_phys = -fx * sinA + fy * cosA;
            float gnx = gx_phys / (float)sc->r0;
            float gny = gy_phys / (float)sc->r0;

            /* Cull cells outside the disk. */
            float r2 = gnx * gnx + gny * gny;
            if (r2 > DISK_R_MAX * DISK_R_MAX + CELL_CULL_MARGIN) continue;

            float r     = sqrtf(r2);
            float theta = atan2f(gny, gnx);

            /* Sample the density field (fBm perturbs the arms). */
            float fbm_val = fbm2(gnx * NOISE_FREQ, gny * NOISE_FREQ + noise_time);
            float dens    = density_at(r, theta, fbm_val, pattern);
            if (dens < DENS_EMPTY) continue;

            /* Star gate: hash the quantised galaxy cell (pattern salts it so
             * each morphology has its own arrangement) and roll against the
             * density-scaled probability. */
            uint32_t h = hash3((int)floorf(gx_phys),
                               (int)floorf(gy_phys / ASPECT_Y), (int)pattern);

            if (hash_unit(h) < dens * STAR_PROB_SCALE)
                draw_star_cell(sy, sx, r, dens, h);
            else if (has_dust && dens > DUST_DENS_MIN && fbm_val > NEBULA_THRESH)
                draw_dust_cell(sy, sx, fbm_val, h);
        }
    }

}

/* Draw a 4-tint palette swatch at row 1, col x; returns the next free column. */
static int draw_swatch(int x, int base_pair, char glyph, int attr)
{
    for (int i = 0; i < 4; i++) {
        attron(COLOR_PAIR(base_pair + i) | attr);
        mvaddch(1, x, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(base_pair + i) | attr);
        x++;
    }
    return x;
}

/* Row 0 right — primary status: fps, sim Hz, phase/pause, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s, double fps, int sim_fps)
{
    const char *state_str = s->paused ? "PAUSED    " : pattern_name(s->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (n/N counter), theme, the star + nebula tint swatches, and
 * the galaxy stats (radius, arm count, pitch). Fixed left-aligned layout. */
static void draw_param_line(const Screen *sc, const Scene *s)
{
    int x = 1;
    char pbuf[40];
    snprintf(pbuf, sizeof pbuf, " pattern:%s %2d/%-2d ",
             pattern_name(s->current_pattern),
             (int)s->current_pattern + 1, (int)N_PATTERNS);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, "%s", pbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(pbuf);

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " stars:");  attroff(COLOR_PAIR(PAIR_HUD));
    x = draw_swatch(x + 7, PAIR_STAR_BASE, '*', A_BOLD);
    attron(COLOR_PAIR(PAIR_HUD));  mvprintw(1, x, " neb:");    attroff(COLOR_PAIR(PAIR_HUD));
    x = draw_swatch(x + 5, PAIR_NEBULA_BASE, '#', A_NORMAL);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  r0:%-3d  arms:%d  pitch:%.2f ",
             sc->r0,
             s->current_pattern == PATTERN_BARRED ? N_ARMS_BARRED : N_ARMS_SPIRAL,
             SPIRAL_PITCH);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:reset  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/*
 * screen_draw — clear, draw scene, then lay the HUD over it (status, title,
 * params, hint).
 *
 * The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, run-state. A const
 * read can't re-couple the layers; scene_draw and the leaf decisions stay narrow.
 */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, s->angle, s->noise_time, s->current_pattern);

    draw_status_line(sc, s, fps, sim_fps);

    /* row 0 left: title */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " PROCEDURAL GALAXY ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(sc, s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  APP  -- events + per-tick combine + main loop                     */
/* ===================================================================== */

/* Owns the App aggregate, signal flags, user-event handlers and the main
 * loop. main() is the ONE place that combines the layers per tick, in fixed
 * order:  scene_tick (SIM) -> scene_draw + screen_draw (RENDER) ->
 * screen_present -> input. app_handle_key() / app_do_resize() mutate state on
 * USER EVENTS (a keypress or SIGWINCH) and are deliberately OUTSIDE the tick. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * Top-level harness binding the simulation (scene) to the terminal (screen),
 * plus the loop's PERFORMANCE knob and the async signal flags. A single static
 * instance (g_app) exists ONLY so the signal handlers — which may fire between
 * any two instructions — can reach the flags; everything else passes App
 * explicitly. Only init + the main loop touch it whole. The fixed-timestep rate
 * (sim_fps) decouples simulation cadence from frame rate (Fiedler, "Fix Your
 * Timestep!"; §8 main()). */
typedef struct {
    /* the two worlds it binds */
    Scene                 scene;        /* WHAT is simulated + shown        */
    Screen                screen;       /* WHERE it is drawn                */
    /* loop control */
    int                   sim_fps;      /* fixed-timestep rate, SIM_FPS_* Hz*/
    /* volatile sig_atomic_t: written from signal handlers, so the compiler
     * must re-read them each loop and the write is atomic w.r.t. the
     * interrupted code. */
    volatile sig_atomic_t running;      /* cleared by SIGINT/SIGTERM → exit */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, served next loop*/
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
