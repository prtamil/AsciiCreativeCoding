/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perin_noise_flow_showcase.c — 30 ways to look at a Perlin-noise field.
 *
 * One noise generator drives thirty visuals. Four of them push particles
 * around like dust on the wind; the other 26 just colour every cell from
 * a math formula each frame. The whole field drifts slowly so nothing
 * sits still. Keys: n/p switch visuals, r makes a brand-new field, t/T
 * change colours.
 *
 * Sister file: ./curl_noise_vector_field.c. The plain FLOW visual here
 * lets streams pile up and thin out; the CURL visual (from that file)
 * keeps flow swirling without ever bunching together. Perlin noise is
 * the classic reference: Ken Perlin, "An image synthesizer", SIGGRAPH '85
 * (https://mrl.cs.nyu.edu/~perlin/paper445.pdf).
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra perin_noise_flow_showcase.c \
 *       -o perlin_flow -lncurses -lm
 */

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

    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* How long a particle lives, in ticks. Each one picks a random
     * lifetime in this range so they don't all die at once — that keeps
     * the streams fresh instead of letting old paths dominate. */
    AGE_MIN_TICKS     =  60,        /* about 1 second at 60 Hz */
    AGE_MAX_TICKS     = 360,        /* about 6 seconds          */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* How fast particles move, in cells per second. The default is slow
     * enough to follow a single trail by eye, fast enough that the flow
     * keeps changing. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* ncurses colour-pair slots. PAIR_HUD/PAIR_HINT are the standard
     * heads-up display colours used across the whole project. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,        /* first of 4 trail colours */
    PAIR_FLASH        =   7,        /* unused here; kept to match sister files */

    /* How many colours a theme has. Every cell and particle carries a
     * band number from 0 to N_BANDS-1. */
    N_BANDS           =   4,
};

/* How fast trails fade, per second. Tuned so a trail stays readable for
 * a few seconds, then disappears cleanly. */
#define GLOW_DECAY          0.6f
#define GLOW_THRESHOLD      0.05f   /* below this a cell is drawn as blank */

/* The screen reserves two rows up top and one at the bottom for the
 * heads-up display; the noise map fills everything in between. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* How zoomed-in the noise is. Smaller = big rolling shapes, larger =
 * fine speckly detail. */
#define NOISE_SCALE         0.04f

/* How fast the whole field drifts, per second. Slow enough that the
 * flow keeps changing without churning. */
#define FIELD_DRIFT         0.10f

/* Brightness cutoffs for picking the trail glyph: above HIGH it's '#',
 * above MID it's '*', otherwise a faint '.'. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — which of the 30 visuals is showing. Cycle with n/p.
 *
 * All 30 share the same noise generator and the same slow drift; they
 * only differ in how they turn a noise value into something on screen.
 * Four of them (FLOW, VORTEX, CURL, JITTER) move particles around; the
 * rest just recolour every cell from a formula each frame. They're
 * grouped into five tiers, roughly simplest to most elaborate.
 */
typedef enum {
    /* Tier 1 — show the noise more or less directly */
    PATTERN_FLOW       =  0,    /* particles drift along the noise        */
    PATTERN_HEIGHT     =  1,    /* bright where noise is high             */
    PATTERN_VALLEYS    =  2,    /* the inverse: bright where noise is low */
    PATTERN_BANDED     =  3,    /* snap heights into a few flat levels    */
    PATTERN_BINARY     =  4,    /* just on or off, split down the middle  */

    /* Tier 2 — reshape a single noise sample */
    PATTERN_RIDGE      =  5,    /* sharp ridges where noise crosses zero  */
    PATTERN_BILLOW     =  6,    /* puffy cloud bumps (inverse of ridge)   */
    PATTERN_ZEBRA      =  7,    /* parallel stripes                       */
    PATTERN_RINGS      =  8,    /* rings from the centre, warped by noise */
    PATTERN_PLASMA     =  9,    /* old-school demoscene plasma look       */

    /* Tier 3 — bend the coordinates before sampling */
    PATTERN_CONTOUR    = 10,    /* contour lines like a topographic map   */
    PATTERN_WARP       = 11,    /* noise nudges its own coordinates once  */
    PATTERN_WARP_DEEP  = 12,    /* warp twice for a much swirlier look    */
    PATTERN_WARPRIDGE  = 13,    /* ridges on top of a warped field        */
    PATTERN_SLOPE      = 14,    /* bright on steep slopes, dark on flats  */

    /* Tier 4 — stack many copies of noise at different sizes */
    PATTERN_FBM        = 15,    /* layered noise: detail at every scale   */
    PATTERN_FBM_HIGH   = 16,    /* same idea, twice the layers / detail   */
    PATTERN_RIDGED     = 17,    /* layered ridges → sharp mountain ranges */
    PATTERN_TURBLENC   = 18,    /* layered |noise| → the base for marble  */
    PATTERN_FBM_INV    = 19,    /* layered noise, inverted                */

    /* Tier 5 — textures, the moving particle modes, and chaos */
    PATTERN_MARBLE     = 20,    /* marble veins                           */
    PATTERN_WOOD       = 21,    /* wood-grain rings                       */
    PATTERN_FIRE       = 22,    /* flames, hot at the bottom              */
    PATTERN_CLOUDS     = 23,    /* soft cloud blobs                       */
    PATTERN_CAVES      = 24,    /* carved-out cave shapes                 */
    PATTERN_STARS      = 25,    /* sparse bright specks                   */
    PATTERN_VORTEX     = 26,    /* particles swirl sideways to the flow   */
    PATTERN_CURL       = 27,    /* particles in endless swirls, no piling */
    PATTERN_JITTER     = 28,    /* FLOW with a random wobble added        */
    PATTERN_CHAOS      = 29,    /* several noise samples mashed together  */

    N_PATTERNS         = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_FLOW:       return "FLOW     ";
    case PATTERN_HEIGHT:     return "HEIGHT   ";
    case PATTERN_VALLEYS:    return "VALLEYS  ";
    case PATTERN_BANDED:     return "BANDED   ";
    case PATTERN_BINARY:     return "BINARY   ";
    case PATTERN_RIDGE:      return "RIDGE    ";
    case PATTERN_BILLOW:     return "BILLOW   ";
    case PATTERN_ZEBRA:      return "ZEBRA    ";
    case PATTERN_RINGS:      return "RINGS    ";
    case PATTERN_PLASMA:     return "PLASMA   ";
    case PATTERN_CONTOUR:    return "CONTOUR  ";
    case PATTERN_WARP:       return "WARP     ";
    case PATTERN_WARP_DEEP:  return "WARP_DEEP";
    case PATTERN_WARPRIDGE:  return "WARPRIDGE";
    case PATTERN_SLOPE:      return "SLOPE    ";
    case PATTERN_FBM:        return "FBM      ";
    case PATTERN_FBM_HIGH:   return "FBM_HIGH ";
    case PATTERN_RIDGED:     return "RIDGED   ";
    case PATTERN_TURBLENC:   return "TURBLENC ";
    case PATTERN_FBM_INV:    return "FBM_INV  ";
    case PATTERN_MARBLE:     return "MARBLE   ";
    case PATTERN_WOOD:       return "WOOD     ";
    case PATTERN_FIRE:       return "FIRE     ";
    case PATTERN_CLOUDS:     return "CLOUDS   ";
    case PATTERN_CAVES:      return "CAVES    ";
    case PATTERN_STARS:      return "STARS    ";
    case PATTERN_VORTEX:     return "VORTEX   ";
    case PATTERN_CURL:       return "CURL     ";
    case PATTERN_JITTER:     return "JITTER   ";
    case PATTERN_CHAOS:      return "CHAOS    ";
    default:                 return "?        ";
    }
}

/* How hard WARP bends its coordinates. Higher = more swirly. */
#define WARP_AMOUNT         5.0f

/* How many layers of noise FBM stacks. 4 looks detailed without going
 * grainy. */
#define FBM_OCTAVES         4

/* The noise heights CONTOUR draws lines at, and how thick each line is. */
#define CONTOUR_BAND_WIDTH  0.04f
static const float CONTOUR_LEVELS[4] = { 0.20f, 0.40f, 0.60f, 0.80f };

/* ── Tier-2 tuning ────────────────────────────────────────────────── */
#define BANDED_LEVELS          6      /* how many flat steps BANDED snaps to   */
#define BINARY_THRESHOLD       0.5f   /* where BINARY cuts on vs off           */
#define ZEBRA_FREQ             3.0f   /* how close together ZEBRA's stripes are*/
#define RING_FREQ              0.30f  /* how close together the rings are      */
#define RING_AMP               1.0f   /* how much noise warps the rings        */
#define PLASMA_FREQ            0.05f  /* size of PLASMA's diagonal waves       */

/* ── Tier-3 tuning ────────────────────────────────────────────────── */
#define WARP_DEEP_AMOUNT       8.0f   /* strength of the second warp pass      */
#define SLOPE_EPS              1.0f   /* how far apart SLOPE samples to measure steepness */
#define SLOPE_GAIN             4.0f   /* brightness boost so slopes show up    */

/* ── Tier-4 tuning ────────────────────────────────────────────────── */
#define FBM_HIGH_OCTAVES       8      /* layer count for the high-detail FBM   */
/* When stacking noise layers: each layer is this much smaller (lacunarity)
 * and this much fainter (gain) than the last. */
#define MULTIFRAC_LACUNARITY   2.0f
#define MULTIFRAC_GAIN         0.5f

/* ── Tier-5 tuning ────────────────────────────────────────────────── */
#define MARBLE_FREQ            0.06f  /* spacing of marble veins               */
#define MARBLE_AMP             5.0f   /* how much the veins wander             */
#define WOOD_FREQ              0.08f  /* spacing of wood-grain rings           */
#define WOOD_AMP               3.0f   /* how much the grain wanders            */
#define FIRE_DRIFT_MULT        4.0f   /* fire flickers faster than the rest    */
/* CLOUDS and CAVES fade in between a low and a high noise level for a
 * soft edge instead of a hard cut. */
#define CLOUDS_THRESH_LO       0.45f
#define CLOUDS_THRESH_HI       0.65f
#define CAVES_THRESH_LO        0.40f
#define CAVES_THRESH_HI        0.60f
#define STARS_FREQ_MULT        4.0f   /* STARS samples finer noise             */
#define STARS_THRESHOLD        0.85f  /* only the brightest ~15% become stars  */
#define JITTER_ANGLE_RANGE     1.5f   /* how wild JITTER's random wobble is    */
#define CHAOS_WARP             10.0f  /* warp strength in the CHAOS mix         */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* If a frame takes a really long time (laptop slept, terminal stalled),
 * pretend it was at most this long so the sim doesn't try to catch up
 * with a huge burst of ticks. Idea from Glenn Fiedler's "Fix Your
 * Timestep". FRAME_CAP_FPS is how often we actually redraw. */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/* A quarter turn, used by VORTEX. And the smallest length we'll bother
 * to normalise — anything shorter we leave alone to avoid divide-by-zero. */
#define VORTEX_TURN_RADIANS  ((float)M_PI * 0.5f)
#define VECTOR_EPSILON       1e-6f

/* Turns a brightness from 0..1 into a colour band 0..N_BANDS-1. The odd
 * 3.99 (instead of 4) keeps brightness 1.0 from spilling into band 4. */
#define GLOW_COL_SCALE       3.99f

/* Widths of the fields on the status row, so each one knows where the
 * next starts. */
#define HUD_PATTERN_FIELD_W   20    /* " pattern:XXXXXXXXX " */
#define HUD_THEME_FIELD_W     17    /* " theme:XXXXXXXX "    */
#define HUD_PALETTE_LABEL_W    9    /* " palette:"           */

/*
 * Theme — one colour scheme for the trails. Ten of them live in
 * themes[]; t/T cycles between them.
 *
 * Each theme is just four colours, arranged dark to bright. A cell's
 * band number (0..3) picks which of the four to use: for the moving
 * particle modes the band is the particle's own colour, picked at
 * birth; for the formula modes it's set by how bright the cell is.
 *
 * The numbers are xterm-256 colour indices, not red/green/blue. On
 * old terminals that only have 8 colours, theme_apply() swaps in a
 * fixed blue/cyan/magenta/yellow set so the demo still runs. Keep every
 * colour in the bright half of the palette (see CLAUDE.md "Theme Palette
 * Brightness") or the darkest ones vanish against a black background.
 */
typedef struct {
    const char *name;              /* short label shown in the HUD            */
    short       trail[N_BANDS];    /* the four colours, dark (0) to bright (3)*/
    short       flash;             /* unused here; kept to match sister files */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      trail0 trail1 trail2 trail3 flash */
    { "DEFAULT", {  33,  117,  220,  220 }, 226 },   /* sky / cyan / gold      */
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },   /* greens                 */
    { "NOVA",    {  53,  129,  201,  219 }, 226 },   /* purple → magenta       */
    { "MONO",    { 240,  244,  250,  254 }, 226 },   /* greyscale              */
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },   /* navy → cyan            */
    { "FIRE",    {  88,  124,  208,  226 }, 196 },   /* dark red → yellow      */
    { "EARTH",   {  58,  100,  173,  230 }, 226 },   /* brown → cream          */
    { "FOREST",  {  22,   28,   64,  144 }, 226 },   /* greens                 */
    { "DESERT",  {  94,  130,  173,  222 }, 226 },   /* sandy                  */
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },   /* navy → ice → white     */
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
            init_pair(PAIR_TRAIL_BASE + i, t->trail[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[N_BANDS] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < N_BANDS; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
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
/* §5  noise — Perlin 2-D                                                 */
/* ===================================================================== */

/*
 * NoiseField — the random ingredient every visual is built from. It's a
 * shuffled list of the numbers 0..255 that the noise function uses as a
 * lookup table. We store it shuffled once and then copied a second time
 * (512 entries total) so the noise code can read two neighbouring
 * entries without worrying about running off the end. A different
 * shuffle gives a completely different field — that's what 'r' does.
 * From Ken Perlin's original 1985 noise implementation.
 */
typedef struct {
    uint8_t perm[512];     /* shuffled 0..255, then the same list again */
} NoiseField;

/*
 * The noise field that perlin2d() is currently reading from. perlin2d is
 * called thousands of times a frame, so rather than pass the field into
 * every single call we just point this at the active one once (in
 * noise_activate, during reset). Same idea as ncurses' stdscr: one
 * "current" thing everyone shares.
 */
static const NoiseField *g_active_noise;

static inline void noise_activate(const NoiseField *nf)
{
    g_active_noise = nf;
}

/* Build a fresh random field: shuffle 0..255, copy it twice into perm[],
 * and make it the active one. Each call gives a brand-new look. */
static void shuffle_noise_field(NoiseField *nf)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        nf->perm[i]       = base[i];
        nf->perm[i + 256] = base[i];
    }
    noise_activate(nf);
}

/* An S-shaped easing curve. When the noise blends between two grid
 * points, feeding the blend amount through this first makes the result
 * ease in and out instead of changing linearly, which is what keeps the
 * field looking smooth rather than blocky. (Perlin's "improved noise".) */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

/* ── small math helpers used by the Tier-2..5 visuals ──────────────── */
static inline float fract(float x)             { return x - floorf(x); }  /* just the part after the decimal point */
static inline float clampf(float x, float lo, float hi)  /* keep x within [lo, hi] */
{
    return x < lo ? lo : (x > hi ? hi : x);
}
/* Like clampf, but eases smoothly from 0 to 1 between a and b instead of
 * snapping — gives soft edges. */
static inline float smoothstep(float a, float b, float x)
{
    float t = clampf((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3.0f - 2.0f * t);
}

/* Fold the noise so the middle becomes a sharp peak and both extremes
 * become valleys — this is what makes ridges. */
static inline float ridge_shape(float n)  { return 1.0f - fabsf(n); }

/* The opposite fold: the extremes become bumps and the middle a valley —
 * gives rounded, puffy shapes. */
static inline float billow_shape(float n) { return fabsf(n); }

/* Picks one of 8 fixed directions from the hashed table value and reports
 * how much the offset point lines up with it. This little dot-product is
 * the heart of why neighbouring points in the field point similar ways. */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * The noise function itself. Given any point, it returns one smoothly-
 * varying value, roughly between -1 and 1. It works by looking at the
 * four grid corners around the point, asking each which way it "leans",
 * and blending those answers with the eased weights from fade(). Every
 * visual in this file is built on top of this one call.
 */
static float perlin2d(float x, float y)
{
    const uint8_t *perm = g_active_noise->perm;

    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    float u = fade(x);
    float v = fade(y);

    int A  = perm[X    ] + Y;
    int B  = perm[X + 1] + Y;

    float n00 = grad(perm[A    ], x,        y       );
    float n10 = grad(perm[B    ], x - 1.0f, y       );
    float n01 = grad(perm[A + 1], x,        y - 1.0f);
    float n11 = grad(perm[B + 1], x - 1.0f, y - 1.0f);

    return lerp(
        lerp(n00, n10, u),
        lerp(n01, n11, u),
        v
    );
}

/* The plainest visual: the raw noise, shifted into a 0..1 brightness.
 * Many other visuals start from this. */
static float pattern_height_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/* "Domain warp": sample the noise once, use that result to shove the
 * coordinates sideways, then sample again at the shoved spot. The field
 * ends up curvier and more organic than plain noise. The two shoves use
 * different offsets so they don't move in lockstep. (Technique from
 * Inigo Quilez.) */
static float pattern_warp_at(float x, float y, float t)
{
    float qx = perlin2d(x * NOISE_SCALE,
                        y * NOISE_SCALE + t);
    float qy = perlin2d((x + 5.2f) * NOISE_SCALE,
                        (y + 1.3f) * NOISE_SCALE + t);
    float n  = perlin2d((x + qx * WARP_AMOUNT) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT) * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/* Stacks several copies of the noise on top of each other — each copy
 * smaller and fainter than the last. You get the big shapes of plain
 * noise plus matching detail at every smaller size. This is the standard
 * "rocks on hills on mountains" trick for natural-looking terrain.
 * (Often called fBm, fractional Brownian motion.) */
static float pattern_fbm_at(float x, float y, float t)
{
    float total   = 0.0f;
    float amp     = 1.0f;
    float freq    = 1.0f;
    float max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * NOISE_SCALE * freq,
                                  y * NOISE_SCALE * freq + t * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* ── three ways to stack noise layers ──────────────────────────────── *
 * These three all loop over "octaves" — layers of noise, each smaller
 * and fainter than the last — but combine the layers differently. Each
 * returns a 0..1 brightness. */

/* The general layer-stacker, with a tunable number of layers. Same idea
 * as pattern_fbm_at: smooth, every-scale detail. */
static float fbm_octaves(float x, float y, float t,
                          int octaves, float lacunarity, float gain)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * perlin2d(x * NOISE_SCALE * freq,
                                   y * NOISE_SCALE * freq + t * freq);
        max_amp += amp;
        amp     *= gain;
        freq    *= lacunarity;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/* Same stacking, but each layer is folded to be always-positive first.
 * That folding leaves sharp creases, giving a wispy, turbulent look —
 * the starting point for the marble, wood, and fire textures. */
static float turbulence_octaves(float x, float y, float t,
                                 int octaves, float lacunarity, float gain)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        total   += amp * fabsf(perlin2d(x * NOISE_SCALE * freq,
                                         y * NOISE_SCALE * freq + t * freq));
        max_amp += amp;
        amp     *= gain;
        freq    *= lacunarity;
    }
    return total / max_amp;
}

/* Same stacking, but every layer is run through ridge_shape and squared
 * to sharpen the peaks. Produces craggy mountain-ridge terrain.
 * (Musgrave's ridged multifractal, "Texturing & Modeling", ch. 16.) */
static float ridged_octaves(float x, float y, float t,
                             int octaves, float lacunarity, float gain)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < octaves; o++) {
        float n = perlin2d(x * NOISE_SCALE * freq,
                           y * NOISE_SCALE * freq + t * freq);
        float r = ridge_shape(n);
        total   += amp * r * r;
        max_amp += amp;
        amp     *= gain;
        freq    *= lacunarity;
    }
    return total / max_amp;
}

/* Turns the noise into a swirling flow direction at a point. The trick:
 * measure how the noise changes left-to-right and up-to-down, then turn
 * those slopes a quarter turn. Flow built this way never piles up or
 * empties out — it can only swirl, like a stirred liquid. This is what
 * the CURL visual rides on. (Bridson et al., "Curl-noise for procedural
 * fluid flow", SIGGRAPH 2007.) */
static void curl_noise_2d(float x, float y, float t,
                           float *out_vx, float *out_vy)
{
    const float eps = SLOPE_EPS;
    float dn_dy = (perlin2d(x * NOISE_SCALE,
                             (y + eps) * NOISE_SCALE + t)
                 - perlin2d(x * NOISE_SCALE,
                             (y - eps) * NOISE_SCALE + t)) * 0.5f;
    float dn_dx = (perlin2d((x + eps) * NOISE_SCALE,
                             y * NOISE_SCALE + t)
                 - perlin2d((x - eps) * NOISE_SCALE,
                             y * NOISE_SCALE + t)) * 0.5f;
    *out_vx =  dn_dy;
    *out_vy = -dn_dx;
}

/* Lights up only the cells whose noise value sits near one of a few
 * chosen heights, drawing thin lines — exactly like the contour lines on
 * a topographic map. It also reports which line a cell is closest to (via
 * out_band) so the caller can colour each line differently. */
static float pattern_contour_at(float x, float y, float t, int *out_band)
{
    float n = pattern_height_at(x, y, t);
    float min_d = 1.0f;
    int closest = 0;
    int n_levels = (int)(sizeof CONTOUR_LEVELS / sizeof CONTOUR_LEVELS[0]);
    for (int i = 0; i < n_levels; i++) {
        float d = fabsf(n - CONTOUR_LEVELS[i]);
        if (d < min_d) { min_d = d; closest = i; }
    }
    *out_band = closest;
    if (min_d < CONTOUR_BAND_WIDTH) {
        return 1.0f - min_d / CONTOUR_BAND_WIDTH;
    }
    return 0.0f;
}

/* ── Tier-1 ──────────────────────────────────────────────────────── */

/* HEIGHT flipped: low spots glow, high spots go dark. */
static float pattern_valleys_at(float x, float y, float t)
{
    return 1.0f - pattern_height_at(x, y, t);
}

/* Rounds every height down to one of a few flat levels, so the field
 * looks terraced like a stepped hillside. */
static float pattern_banded_at(float x, float y, float t)
{
    float h = pattern_height_at(x, y, t);
    return floorf(h * BANDED_LEVELS) / (float)BANDED_LEVELS;
}

/* Pure on/off: anything above the halfway mark is full bright, the rest
 * is dark. */
static float pattern_binary_at(float x, float y, float t)
{
    return pattern_height_at(x, y, t) > BINARY_THRESHOLD ? 1.0f : 0.0f;
}

/* ── Tier-2 ──────────────────────────────────────────────────────── */

/* Fold the noise into sharp ridge lines — mountain silhouettes from a
 * single noise sample, no layering needed. */
static float pattern_ridge_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return ridge_shape(n);
}

/* The opposite fold: rounded, puffy bumps instead of sharp ridges. */
static float pattern_billow_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return billow_shape(n);
}

/* Run the noise through a wave to get parallel stripes that bend along
 * the field. */
static float pattern_zebra_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return 0.5f + 0.5f * sinf(n * (float)M_PI * ZEBRA_FREQ);
}

/* Rings spreading out from the centre, with the noise wobbling them so
 * they're not perfect circles. */
static float pattern_rings_at(float x, float y, float t,
                                float cx, float cy)
{
    float dx = x - cx, dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy);
    float n  = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return 0.5f + 0.5f * sinf(r * RING_FREQ + n * RING_AMP);
}

/* The classic 90s "plasma" wash: a diagonal wave whose phase is pushed
 * around by the noise. */
static float pattern_plasma_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return 0.5f + 0.5f * sinf(n * (float)M_PI
                              + sinf((x + y) * PLASMA_FREQ));
}

/* ── Tier-3 ──────────────────────────────────────────────────────── */

/* Like WARP, but warp a second time using the first warp's result.
 * Far swirlier and more organic. (Quilez's "deep warp".) */
static float pattern_warp_deep_at(float x, float y, float t)
{
    float qx = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    float qy = perlin2d((x + 5.2f) * NOISE_SCALE,
                        (y + 1.3f) * NOISE_SCALE + t);
    float rx = perlin2d((x + qx * WARP_AMOUNT) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT) * NOISE_SCALE + t);
    float ry = perlin2d((x + qx * WARP_AMOUNT + 5.2f) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT + 1.3f) * NOISE_SCALE + t);
    float n  = perlin2d((x + rx * WARP_DEEP_AMOUNT) * NOISE_SCALE,
                        (y + ry * WARP_DEEP_AMOUNT) * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/* Warp the field first, then fold it into ridges — the warp bends the
 * ridges into wandering rivers and canyons. */
static float pattern_warpridge_at(float x, float y, float t)
{
    float qx = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    float qy = perlin2d((x + 5.2f) * NOISE_SCALE,
                        (y + 1.3f) * NOISE_SCALE + t);
    float n  = perlin2d((x + qx * WARP_AMOUNT) * NOISE_SCALE,
                        (y + qy * WARP_AMOUNT) * NOISE_SCALE + t);
    return ridge_shape(n);
}

/* Measures how steeply the field changes around each point by comparing
 * nearby samples: flat areas stay dark, steep cliffs light up. */
static float pattern_slope_at(float x, float y, float t)
{
    const float eps = SLOPE_EPS;
    float n_xp = perlin2d((x + eps) * NOISE_SCALE, y * NOISE_SCALE + t);
    float n_xm = perlin2d((x - eps) * NOISE_SCALE, y * NOISE_SCALE + t);
    float n_yp = perlin2d(x * NOISE_SCALE, (y + eps) * NOISE_SCALE + t);
    float n_ym = perlin2d(x * NOISE_SCALE, (y - eps) * NOISE_SCALE + t);
    float dn_dx = (n_xp - n_xm) * 0.5f;
    float dn_dy = (n_yp - n_ym) * 0.5f;
    float mag = sqrtf(dn_dx * dn_dx + dn_dy * dn_dy);
    return clampf(mag * SLOPE_GAIN, 0.0f, 1.0f);
}

/* ── Tier-4 ──────────────────────────────────────────────────────── */

/* FBM with twice as many layers — more fine detail. */
static float pattern_fbm_high_at(float x, float y, float t)
{
    return fbm_octaves(x, y, t, FBM_HIGH_OCTAVES,
                        MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
}

/* Layered ridges: sharp mountain ranges with detail at every size. */
static float pattern_ridged_at(float x, float y, float t)
{
    return ridged_octaves(x, y, t, FBM_OCTAVES,
                           MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
}

/* The turbulent, wispy stack — also the base ingredient for marble,
 * wood, and fire below. */
static float pattern_turblenc_at(float x, float y, float t)
{
    return turbulence_octaves(x, y, t, FBM_OCTAVES,
                               MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
}

/* FBM flipped: dark where FBM is bright and vice versa. */
static float pattern_fbm_inv_at(float x, float y, float t)
{
    return 1.0f - pattern_fbm_at(x, y, t);
}

/* ── Tier-5 ──────────────────────────────────────────────────────── */

/* Vertical stripes warped by turbulence give marble veins. This is the
 * famous one — Perlin showed marble was just a wave bent by turbulence
 * back in 1985. */
static float pattern_marble_at(float x, float y, float t)
{
    float turb = turbulence_octaves(x, y, t, FBM_OCTAVES,
                                     MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return 0.5f + 0.5f * sinf(x * MARBLE_FREQ + turb * MARBLE_AMP);
}

/* Rings around the centre, warped by turbulence — wood grain. */
static float pattern_wood_at(float x, float y, float t,
                              float cx, float cy)
{
    float dx = x - cx, dy = y - cy;
    float r  = sqrtf(dx * dx + dy * dy);
    float turb = turbulence_octaves(x, y, t, FBM_OCTAVES,
                                     MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return fract(r * WOOD_FREQ + turb * WOOD_AMP);
}

/* Turbulence fading out toward the top of the screen, so flames pool at
 * the bottom. It also flickers faster than the rest of the demo. */
static float pattern_fire_at(float x, float y, float t, int h)
{
    float turb = turbulence_octaves(x, y, t * FIRE_DRIFT_MULT, FBM_OCTAVES,
                                     MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    float mask = 1.0f - (float)y / (float)h;   /* hot at bottom, cool up top */
    return clampf(turb * 2.0f * mask, 0.0f, 1.0f);
}

/* Layered noise with a soft cutoff: dense areas become cloud blobs,
 * the rest clear sky. */
static float pattern_clouds_at(float x, float y, float t)
{
    float n = fbm_octaves(x, y, t, FBM_OCTAVES,
                           MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return smoothstep(CLOUDS_THRESH_LO, CLOUDS_THRESH_HI, n);
}

/* CLOUDS flipped: dense areas become hollow caves, sparse areas become
 * solid rock. */
static float pattern_caves_at(float x, float y, float t)
{
    float n = fbm_octaves(x, y, t, FBM_OCTAVES,
                           MULTIFRAC_LACUNARITY, MULTIFRAC_GAIN);
    return 1.0f - smoothstep(CAVES_THRESH_LO, CAVES_THRESH_HI, n);
}

/* Sample fine noise, sharpen it to peaks, and keep only the very
 * brightest — a sparse scattering of specks like a starfield. */
static float pattern_stars_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE * STARS_FREQ_MULT,
                       y * NOISE_SCALE * STARS_FREQ_MULT + t);
    float r = ridge_shape(n);
    return r > STARS_THRESHOLD ? r : 0.0f;
}

/* Three noise samples at different sizes, warped and wave-mixed into one
 * busy composite. The most complex-looking visual; nowhere repeats. */
static float pattern_chaos_at(float x, float y, float t)
{
    float n1 = perlin2d(x * NOISE_SCALE,
                        y * NOISE_SCALE + t);
    float n2 = perlin2d(x * NOISE_SCALE * 2.0f,
                        y * NOISE_SCALE * 2.0f - t);
    float n3 = perlin2d((x + n1 * CHAOS_WARP) * NOISE_SCALE,
                        (y + n2 * CHAOS_WARP) * NOISE_SCALE + t * 1.5f);
    float result = sinf(n1 * 4.0f) * cosf(n2 * 4.0f) + n3;
    return clampf(result * 0.4f + 0.5f, 0.0f, 1.0f);
}

/* ===================================================================== */
/* §6  scene — Grid, Particles, RenderBuffers, SimState, Controls         */
/* ===================================================================== */

/*
 * The whole program's state is one Scene struct, made of six smaller
 * pieces (NoiseField from §5 plus the five below). Each piece holds one
 * kind of thing, which also makes function signatures honest: a function
 * handed a `const Grid *` plainly can't scribble on the buffers, and so
 * on. The sister field files are built the same way.
 */

/*
 * Particle — one grain of dust drifting on the flow. It has no weight or
 * momentum: each tick it just reads the flow direction where it sits and
 * steps that way. It doesn't remember its own trail — the fading marks it
 * leaves behind live in RenderBuffers.glow instead.
 *
 * Each particle is given a random lifespan so they don't all expire
 * together. Without that, a few long-lived grains stuck in slow spots
 * would hog the screen and the overall shape of the flow would stop
 * showing.
 */
typedef struct {
    /* Where the grain is, as fractions of a cell (so it can move less
     * than a whole cell per tick). The renderer rounds to a whole cell
     * only when it paints. */
    float x, y;

    /* Which of the four theme colours this grain wears, chosen at birth.
     * Different colours let you see streams cross without merging. */
    int   color_idx;

    /* How many ticks old it is, and the random age at which it dies and
     * respawns somewhere new. */
    int   age;
    int   max_age;
} Particle;

/*
 * Grid — just the size of the drawing area. Every layer needs the width
 * and height, so it sits at the top of Scene.
 *
 * Cells are stored row by row, so cell (x, y) lives at index y*w + x —
 * the grid_idx helper below does that. The size is always kept within
 * the fixed buffer limit (200x56) by app_pick_map_size.
 */
typedef struct {
    int w, h;            /* map size in cells */
    int total_cells;     /* w*h, kept handy so hot loops skip the multiply */
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — the picture, one entry per cell, in two arrays:
 *   glow  : brightness 0..1, which picks the character ('.', '*', '#')
 *   color : which of the four theme colours to use
 *
 * This is the only thing the drawing code looks at. The simulation
 * writes here (a particle marks the cell it lands on; the formula modes
 * fill in every cell each frame), the screen reads here, and the two
 * sides don't otherwise touch. They're kept as two separate arrays
 * because the fade step only needs glow, so this avoids walking over the
 * colour bytes too.
 */
typedef struct {
    /* Brightness 0..1. A particle sets its cell to 1.0; every tick all
     * cells dim a little so trails fade. The formula modes overwrite the
     * whole thing each frame instead. */
    float   glow [CELLS_MAX];

    /* Colour band 0..3. For the particle modes it's the colour of the
     * last grain that landed; for the formula modes it comes from how
     * bright the cell is. */
    uint8_t color[CELLS_MAX];
} RenderBuffers;

/*
 * Particles — the bag of dust grains. It's one big array plus a count of
 * how many are actually in use, so spawning and respawning never need to
 * allocate memory. Only the first `n` are alive. The array is sized for
 * the worst case; the demo normally uses far fewer.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];
    int      n;                   /* how many are alive, 0..MAX_PARTICLES */
} Particles;

/*
 * SimState — the one number that changes on its own every tick: how far
 * the field has drifted. It's added to the noise so the whole picture
 * slowly slides through "noise space" and keeps evolving. Kept apart from
 * Controls (the user's knobs) so it's clear what the sim changes versus
 * what the keyboard changes.
 */
typedef struct {
    float field_time;     /* how far the field has drifted so far */
} SimState;

/*
 * Controls — the knobs the keyboard turns. Only app_handle_key writes
 * these; the sim and the drawing code only read them. Keeping that one-
 * way means the keyboard never reaches into the simulation directly.
 */
typedef struct {
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
} Controls;

/*
 * Scene — everything, in one place. Reading top to bottom is the quickest
 * way to see how the program fits together: where things live, what gets
 * drawn, the moving grains, the noise they ride on, the drift, and the
 * user's knobs. Each piece only relies on the ones above it.
 */
typedef struct {
    Grid          grid;       /* size of the drawing area              */
    RenderBuffers buf;        /* the picture: brightness + colour       */
    Particles     particles;  /* the drifting grains                    */
    NoiseField    noise;      /* the random source everything rides on  */
    SimState      sim;        /* the drift, changed only by scene_tick  */
    Controls      ctrl;       /* user knobs, changed only by keypresses */
} Scene;

/* Turn a brightness into one of the four colour bands. Used by the
 * formula modes that don't have their own colour rule. */
static inline int glow_to_col(float g)
{
    return (int)(g * GLOW_COL_SCALE) & (N_BANDS - 1);
}

/* ── particle pipeline ───────────────────────────────────────────── *
 * One small function per step a grain goes through — born, find its
 * direction, move, leave a mark, age, check if it's done. The bigger
 * functions further down just string these together. */

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/* True for the four visuals that move particles around. */
static bool is_particle_pattern(Pattern p)
{
    return p == PATTERN_FLOW   || p == PATTERN_VORTEX
        || p == PATTERN_CURL   || p == PATTERN_JITTER;
}

/* ── building blocks for the particle modes ──────────────────────── */

/* Read the noise right where this grain is sitting. */
static inline float sample_noise_at_particle(const Particle *p, float field_time)
{
    return perlin2d(p->x * NOISE_SCALE,
                    p->y * NOISE_SCALE + field_time);
}

/* Turn a heading (an angle) into an actual velocity at the current speed. */
static inline void velocity_from_angle(float angle, int speed,
                                        float *out_vx, float *out_vy)
{
    *out_vx = cosf(angle) * (float)speed;
    *out_vy = sinf(angle) * (float)speed;
}

/* A small random nudge to the heading, used by JITTER to roughen up the
 * paths. */
static inline float random_jitter_angle(float range)
{
    return ((float)rand() / (float)RAND_MAX - 0.5f) * range;
}

/* Rescale a direction to length 1 (skipping it if it's basically zero,
 * to avoid dividing by nothing). CURL needs this because its raw flow
 * strength swings wildly but we want every grain to move at one speed. */
static inline void normalise_2d(float *vx, float *vy)
{
    float m = sqrtf((*vx) * (*vx) + (*vy) * (*vy));
    if (m > VECTOR_EPSILON) { *vx /= m; *vy /= m; }
}

/* Works out which way a grain should move, depending on the current
 * visual:
 *   FLOW   : head in the direction the noise points
 *   VORTEX : same, but a quarter turn off, so grains circle instead
 *   CURL   : follow the swirling curl flow (never piles up)
 *   JITTER : FLOW with a random wobble added
 */
static void particle_velocity_at(Pattern pat, const Particle *p,
                                  float field_time, int speed,
                                  float *out_vx, float *out_vy)
{
    switch (pat) {
    case PATTERN_FLOW: {
        float angle = sample_noise_at_particle(p, field_time) * (float)M_PI;
        velocity_from_angle(angle, speed, out_vx, out_vy);
        return;
    }
    case PATTERN_VORTEX: {
        float angle = sample_noise_at_particle(p, field_time) * (float)M_PI
                    + VORTEX_TURN_RADIANS;
        velocity_from_angle(angle, speed, out_vx, out_vy);
        return;
    }
    case PATTERN_CURL: {
        float vx, vy;
        curl_noise_2d(p->x, p->y, field_time, &vx, &vy);
        normalise_2d(&vx, &vy);
        *out_vx = vx * (float)speed;
        *out_vy = vy * (float)speed;
        return;
    }
    case PATTERN_JITTER: {
        float angle = sample_noise_at_particle(p, field_time) * (float)M_PI
                    + random_jitter_angle(JITTER_ANGLE_RANGE);
        velocity_from_angle(angle, speed, out_vx, out_vy);
        return;
    }
    default:
        *out_vx = 0.0f;
        *out_vy = 0.0f;
        return;
    }
}

/* Move the grain by its velocity for one tick, and count it a tick older. */
static void advect_particle_euler(Particle *p, float vx, float vy, float dt)
{
    p->x += vx * dt;
    p->y += vy * dt;
    p->age++;
}

/* Light up the cell the grain is on, full brightness. The trail behind it
 * fades on its own via the per-tick dimming. */
static void deposit_trail_hit(RenderBuffers *buf, const Grid *g,
                                int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(g, cx, cy)) return;
    int idx = grid_idx(g, cx, cy);
    buf->glow [idx] = 1.0f;
    buf->color[idx] = (uint8_t)color_idx;
}

/* Is the grain done? Either it's too old or it wandered off-screen — both
 * mean respawn it. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

/* One grain's whole turn: find its direction, move, mark where it lands,
 * and respawn it if it's done. */
static void particle_step(RenderBuffers *buf, const Grid *g,
                          Particle *p, Pattern pat,
                          float field_time, float dt, int speed)
{
    float vx, vy;
    particle_velocity_at(pat, p, field_time, speed, &vx, &vy);
    advect_particle_euler(p, vx, vy, dt);
    deposit_trail_hit    (buf, g, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, g))
        particle_spawn   (p, g);
}

/* For the formula visuals (everything that isn't a particle mode): fill
 * in brightness and colour for every cell from the current visual's
 * formula. Overwrites the whole picture each frame, so there's no fade. */
static void rasterise_pattern_field(RenderBuffers *buf, const Grid *g,
                                     Pattern p, float t)
{
    float cx = (float)g->w * 0.5f;
    float cy = (float)g->h * 0.5f;
    int   h  = g->h;

    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            int   idx  = grid_idx(g, x, y);
            float fx   = (float)x, fy = (float)y;
            float glow = 0.0f;
            int   col  = 0;

            switch (p) {

            /* Tier 1 */
            case PATTERN_HEIGHT:    glow = pattern_height_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_VALLEYS:   glow = pattern_valleys_at (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_BANDED:    glow = pattern_banded_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_BINARY:    glow = pattern_binary_at  (fx, fy, t); col = glow > 0.5f ? 3 : 0; break;

            /* Tier 2 */
            case PATTERN_RIDGE:     glow = pattern_ridge_at   (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_BILLOW:    glow = pattern_billow_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_ZEBRA:     glow = pattern_zebra_at   (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_RINGS:     glow = pattern_rings_at   (fx, fy, t, cx, cy); col = glow_to_col(glow); break;
            case PATTERN_PLASMA:    glow = pattern_plasma_at  (fx, fy, t); col = glow_to_col(glow); break;

            /* Tier 3 */
            case PATTERN_CONTOUR: {
                int band;
                glow = pattern_contour_at(fx, fy, t, &band);
                col  = band & 3;
                break;
            }
            case PATTERN_WARP:      glow = pattern_warp_at      (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_WARP_DEEP: glow = pattern_warp_deep_at (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_WARPRIDGE: glow = pattern_warpridge_at (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_SLOPE:     glow = pattern_slope_at     (fx, fy, t); col = glow_to_col(glow); break;

            /* Tier 4 */
            case PATTERN_FBM:       glow = pattern_fbm_at       (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_FBM_HIGH:  glow = pattern_fbm_high_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_RIDGED:    glow = pattern_ridged_at    (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_TURBLENC:  glow = pattern_turblenc_at  (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_FBM_INV:   glow = pattern_fbm_inv_at   (fx, fy, t); col = glow_to_col(glow); break;

            /* Tier 5 (the formula ones; particle modes are handled elsewhere) */
            case PATTERN_MARBLE:    glow = pattern_marble_at    (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_WOOD:      glow = pattern_wood_at      (fx, fy, t, cx, cy); col = glow_to_col(glow); break;
            case PATTERN_FIRE:      glow = pattern_fire_at      (fx, fy, t, h); col = glow_to_col(glow); break;
            case PATTERN_CLOUDS:    glow = pattern_clouds_at    (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_CAVES:     glow = pattern_caves_at     (fx, fy, t); col = glow_to_col(glow); break;
            case PATTERN_STARS:     glow = pattern_stars_at     (fx, fy, t); col = 3; break;
            case PATTERN_CHAOS:     glow = pattern_chaos_at     (fx, fy, t); col = glow_to_col(glow); break;

            default:
                continue;       /* particle modes are drawn by scene_tick, not here */
            }

            buf->glow [idx] = glow;
            buf->color[idx] = (uint8_t)(col & 3);
        }
    }
}

/* ── per-frame update and reset ──────────────────────────────────── *
 * scene_tick runs once per frame; scene_reset starts a fresh field. Both
 * are just short lists of the named steps below. */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->field_time = 0.0f;
}

static void buffers_clear(RenderBuffers *buf, int n)
{
    for (int i = 0; i < n; i++) {
        buf->glow [i] = 0.0f;
        buf->color[i] = 0;
    }
}

static void decay_trail_glow(RenderBuffers *buf, int n, float dt)
{
    float decay = expf(-GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) buf->glow[i] *= decay;
}

static void advance_noise_time(SimState *sim, float dt)
{
    sim->field_time += FIELD_DRIFT * dt;
}

static void install_fresh_noise(NoiseField *nf)
{
    shuffle_noise_field(nf);
}

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void step_all_particles(Scene *s, float dt)
{
    int spd = s->ctrl.speed;
    for (int i = 0; i < s->particles.n; i++)
        particle_step(&s->buf, &s->grid,
                       &s->particles.pool[i],
                       s->ctrl.current_pattern,
                       s->sim.field_time, dt, spd);
}

/* Wipe the picture clean. Called when switching visuals so the old
 * trails don't haunt the new one. */
static void scene_clear_field(Scene *s)
{
    buffers_clear(&s->buf, s->grid.total_cells);
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions  (&s->grid, w, h);
    reset_sim_state        (&s->sim);
    buffers_clear          (&s->buf, s->grid.total_cells);
    install_fresh_noise    (&s->noise);
    spawn_all_particles    (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused           = false;
    s->ctrl.speed            = SPEED_DEF;
    s->ctrl.current_theme    = 0;
    s->ctrl.current_pattern  = PATTERN_FLOW;
    scene_reset(s, w, h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    advance_noise_time(&s->sim, dt);

    if (is_particle_pattern(s->ctrl.current_pattern)) {
        decay_trail_glow  (&s->buf, s->grid.total_cells, dt);
        step_all_particles(s, dt);
    } else {
        rasterise_pattern_field(&s->buf, &s->grid,
                                 s->ctrl.current_pattern,
                                 s->sim.field_time);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/*
 * Screen — just the terminal's current width and height, remembered so we
 * can centre the map and place the HUD. Refreshed when the window is
 * resized. It's kept separate from the simulation because the sim doesn't
 * care what it's drawn on; everything else about the terminal is ncurses'
 * business, not ours.
 */
typedef struct {
    int cols;   /* terminal width  in characters */
    int rows;   /* terminal height in characters */
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
 * CellDraw — the decision of what to draw at one cell: which colour, bold
 * or not, which character, or skip it entirely. classify_trail_cell fills
 * one in (no terminal calls), then paint_cell does the actual drawing.
 * Splitting it this way keeps all the ncurses calls in one spot.
 */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

/* Decide how to draw one cell from its brightness and colour: brighter
 * cells get a denser character, dim ones a dot, and anything too faint is
 * skipped. (The '.'/'*'/'#' ramp is the usual ASCII-art shading trick.) */
static CellDraw classify_trail_cell(float glow, uint8_t color)
{
    int pair = PAIR_TRAIL_BASE + (color & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '#' };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '*' };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = '.' };
    return (CellDraw){ .skip = true };
}

/* The single place that actually writes a trail character to the screen. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

/* Work out the top-left corner to start drawing the map at, so it sits
 * centred with the HUD rows left clear above and below. */
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

/* Draw the whole map: for every cell, decide what it should look like and
 * paint it. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid          *g   = &s->grid;
    const RenderBuffers *buf = &s->buf;
    int gx0, gy0;
    compute_centred_origin(g, cols, rows, &gx0, &gy0);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int idx = grid_idx(g, x, y);
            paint_cell(sy, sx,
                        classify_trail_cell(buf->glow[idx], buf->color[idx]));
        }
    }
}

/* ── the heads-up display ─────────────────────────────────────────── *
 * The top two rows show what's going on — fps, which visual, theme, a
 * little colour swatch, the settings. The bottom row lists the keys you
 * can press. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  speed:%-3d ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             c->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " PERLIN-NOISE FLOW ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── the second HUD row, piece by piece ──────────────────────────── *
 * Each of these draws one labelled field left-to-right and returns where
 * the next one should start. */

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
        int pair = PAIR_TRAIL_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* The last field on the row: noise zoom, drift speed, and map size. */
static void draw_status_sim_counts(int row, int x, const Grid *g)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  scale:%.2f  drift:%.2f  map:%dx%d ",
             NOISE_SCALE, FIELD_DRIFT, g->w, g->h);
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
        draw_status_sim_counts    (1, x, &s->grid);
}

/* The bottom row: the list of keys. */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:speed  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw           (s, sc->cols, sc->rows);
    draw_hud_state_bar   (sc, s, fps, sim_fps);
    draw_hud_title       ();
    draw_hud_status_line (s);
    draw_bottom_hint     (sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * App — the program as a whole: the simulation, the terminal size, a few
 * settings, and a couple of flags. There's one of these, g_app, kept
 * global so the signal handlers can reach it.
 *
 * The two flags are written by signal handlers (when you press Ctrl-C or
 * resize the window). A handler can fire at any moment, so it only flips a
 * flag and gets out; the main loop notices the flag and does the real work
 * safely. The volatile + sig_atomic_t types are exactly what's needed for
 * a flag shared between a handler and the main loop — see Stevens & Rago,
 * "Advanced Programming in the UNIX Environment", ch. 10.
 */
typedef struct {
    Scene                 scene;     /* the simulation                       */
    Screen                screen;    /* current terminal size                */

    int                   sim_fps;   /* tick rate; changed by '[' and ']'    */
    int                   map_w;     /* chosen map width                     */
    int                   map_h;     /* chosen map height                    */

    volatile sig_atomic_t running;       /* set to 0 to quit                 */
    volatile sig_atomic_t need_resize;   /* set to 1 when the window resized */
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

/* ── keyboard ────────────────────────────────────────────────────── *
 * One small function per action; app_handle_key just routes each key to
 * the right one. */

/* '+' doubles the speed, '-' halves it (kept within limits). */
static void bump_speed_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->speed < SPEED_MAX) c->speed *= 2;
        if (c->speed > SPEED_MAX) c->speed = SPEED_MAX;
    } else {
        c->speed /= 2;
        if (c->speed < SPEED_MIN) c->speed = SPEED_MIN;
    }
}

/* '[' and ']' nudge the tick rate up or down (kept within limits). */
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

/* Switch to the next/previous visual and wipe the screen, so old trails
 * don't flash through before the new visual takes over. */
static void cycle_pattern(App *app, int dir)
{
    Controls *c = &app->scene.ctrl;
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
    scene_clear_field(&app->scene);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                              break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);    break;
    case '=': case '+': bump_speed_geometric(c,   +1);                       break;
    case '-':           bump_speed_geometric(c,   -1);                       break;
    case ']':           bump_sim_fps        (app, +SIM_FPS_STEP);            break;
    case '[':           bump_sim_fps        (app, -SIM_FPS_STEP);            break;
    case 't':           cycle_theme         (c,   +1);                       break;
    case 'T':           cycle_theme         (c,   -1);                       break;
    case 'n': case 'N': cycle_pattern       (app, +1);                       break;
    case 'p': case 'P': cycle_pattern       (app, -1);                       break;
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

/* How long since the last frame, capped so a long stall doesn't make the
 * sim try to fast-forward through a flood of catch-up ticks. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned, so the
 * sim runs at a steady rate no matter the frame rate. (Glenn Fiedler,
 * "Fix Your Timestep".) */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Recompute the fps shown in the HUD, but only every half second so the
 * number doesn't flicker. */
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

/* Sleep off the rest of the frame's time budget so we don't redraw faster
 * than needed and burn CPU. */
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
