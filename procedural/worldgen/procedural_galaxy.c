/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * procedural_galaxy.c — a rotating galaxy drawn straight from math, with no
 * stored stars: for every cell on screen we work out how bright the galaxy
 * is there and roll a dice to decide whether a star sits in it. Spiral arms
 * come from a curve called a logarithmic spiral; wobbly noise makes the arms
 * look organic instead of perfectly clean.
 *
 * References the code can't give you:
 *   Galaxy shapes (the 15 presets):
 *     en.wikipedia.org/wiki/Galaxy_morphological_classification  (Hubble sequence)
 *     en.wikipedia.org/wiki/Logarithmic_spiral   — the arm curve
 *     Lin & Shu (1964), "On the spiral structure of disk galaxies", ApJ 140, 646
 *   Noise & rendering:
 *     Perlin, K. (2002), "Improving Noise"  — mrl.cs.nyu.edu/~perlin/paper445.pdf
 *     Inigo Quilez, "Painting a galaxy"     — iquilezles.org/articles/warp/
 *   Sister files (same trick, different worlds):
 *     ../worldgen/procedural_star_field_parallax_noise_showcase.c
 *     ../fields/perin_noise_flow_showcase.c   — the Perlin/fBm noise is copied
 *       from here, since each file must stand alone.
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

/* ── §1  CONFIG — constants, data tables (glyphs, themes), Pattern enum ── */

enum {
    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    /* How fast the galaxy spins, as a user-facing dial (not the real
     * radians/sec — that's scaled in scene_tick). */
    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Arm counts: the plain spiral and nebula have 4, the barred one has 2. */
    N_ARMS_SPIRAL       =   4,
    N_ARMS_BARRED       =   2,

    /* ncurses colour-pair slots. The HUD and hint slots are fixed by the
     * project's style guide; the rest are the star and dust palettes. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_STAR_BASE      =   3,    /* 4 star tints live at +0..+3     */
    PAIR_NEBULA_BASE    =   7,    /* 4 dust tints live at +0..+3     */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/*
 * A terminal character is about twice as tall as it is wide. We stretch the
 * vertical math by this factor so the round galaxy looks round, not squashed.
 */
#define ASPECT_Y           2.0f

/*
 * Galaxy shape, measured so that radius 1 is the outer edge of the disk.
 * Changing any of these reshapes the galaxy:
 *   BULGE_SIGMA   smaller = a tighter, brighter centre blob
 *   DISK_SIGMA    bigger  = arms reach further out
 *   DISK_R_MAX    where we stop drawing (1.0 = the whole disk)
 *   SPIRAL_PITCH  how tightly the arms wind; smaller = more coiled
 *   ARM_WIDTH     how thick an arm is
 *   BULGE_AMP / DISK_AMP — how much the centre vs. the disk count
 */
#define BULGE_SIGMA        0.13f
#define DISK_SIGMA         0.45f
#define DISK_R_MAX         1.00f
#define SPIRAL_PITCH       0.30f
#define ARM_WIDTH          0.20f
#define BULGE_AMP          0.95f
#define DISK_AMP           0.95f

/* The central bar of a barred galaxy: how long and how wide it is. It always
 * lies along the x-axis before the whole galaxy is rotated. */
#define BAR_LEN            0.36f
#define BAR_WIDTH          0.09f
#define BAR_AMP            0.85f

/* The smooth round elliptical galaxy: how big its glow is. */
#define ELLIPTICAL_SIGMA   0.50f
#define ELLIPTICAL_AMP     0.55f

/*
 * Knobs for the extra galaxy types. Each one just reuses the same building
 * blocks (centre blob, disk, spiral arms, bar, ring, noise) with a different
 * number of arms, a different winding tightness, and so on. Smaller pitch
 * means more tightly coiled arms.
 */
#define N_ARMS_GRAND       2           /* two bold sweeping arms (like M51)  */
#define N_ARMS_PINWHEEL    6           /* many fine arms (like M101)         */
#define PITCH_GRAND        0.34f       /* loosely wound                      */
#define PITCH_PINWHEEL     0.22f       /* tighter                            */
#define PITCH_TIGHT        0.15f       /* very tightly coiled                */
#define ARM_WIDTH_GRAND    0.30f       /* wide, bold arms                    */
#define RING_R             0.58f       /* how far out the bright ring sits   */
#define RING_W             0.12f       /* how thick the ring is              */
#define SOMBRERO_THIN      0.10f       /* thinness of an edge-on disk        */
#define NUCLEUS_SIGMA      0.045f      /* size of a tiny brilliant core      */

/* Wobble noise. NOISE_FREQ sets how big the blobs are, DRIFT how fast they
 * slowly move, ARM_NOISE_AMP how much they roughen the arm edges, and
 * NEBULA_THRESH how strong the noise must be before it glows as a cloud. */
#define NOISE_FREQ         1.5f
#define NOISE_DRIFT        0.10f
#define ARM_NOISE_AMP      0.20f       /* nudges arm width up/down by 20%     */
#define FBM_OCTAVES        4
#define NEBULA_THRESH      0.55f

/* Dims down the star-placing odds so the bright centre doesn't fill every
 * single cell with a star. */
#define STAR_PROB_SCALE    0.18f

/* Cut-offs the renderer uses per cell. The brightness ones decide which
 * glyph tier a star or dust cloud gets. */
#define CELL_CULL_MARGIN   0.05f       /* a little slack so the disk edge isn't clipped */
#define DENS_EMPTY         0.01f       /* below this the cell is just empty space        */
#define STAR_DENS_BRIGHT   0.55f
#define STAR_DENS_MID      0.20f
#define DUST_DENS_MIN      0.05f
#define DUST_INTENS_BRIGHT 0.65f
#define DUST_INTENS_MID    0.30f

/* How fast the galaxy turns at the default speed. At 0.06 a full turn takes
 * about 105 seconds. */
#define ROTATION_RATE      0.06f

/*
 * Star glyphs by brightness: bright for the densest spots (the core and the
 * middle of arms), mid for the body of an arm, dim for the faint outskirts.
 */
static const char STAR_BRIGHT[4] = { '*', 'O', '+', '#' };
static const char STAR_MID   [4] = { '*', '+', 'o', '.' };
static const char STAR_DIM   [4] = { '.', '`', ',', '\'' };

/* ── Pattern ───────────────────────────────────────────────────────────── *
 * The fifteen galaxy types you can flip through. The key idea: a galaxy's
 * "type" is not different data, it's a different recipe stirred from the same
 * handful of ingredients (centre blob, disk, spiral arms, bar, ring, noise).
 * So a type is just one branch inside density_at(), and switching is instant —
 * nothing is rebuilt or allocated. Each is a real class of galaxy, differing
 * only in how many arms it has, how tightly they wind, and so on. The chosen
 * value also gets mixed into the star-placing dice, so every type shows a
 * different scatter of stars. The first four keep their original look; the
 * rest fill out the catalogue.
 * Names come from the Hubble sequence (see the file header).
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
 * One named colour scheme; ten ship and t/T cycles them. theme_apply() copies
 * its colour codes into the ncurses pairs.
 *
 * Each theme holds two little 4-colour ladders:
 *   star[4]   — star colours, picked by how far a star sits from the centre.
 *               Ordered warm-to-cool to echo real galaxies: old yellow-red
 *               stars in the core, young blue-white stars further out.
 *               star[0] is the core, star[3] is the faint outer halo.
 *   nebula[4] — dust-cloud colours, picked at random from the hash. Only the
 *               dusty types (NEBULA / FLOCCULENT / STARBURST) use these.
 * Every code is in the bright half of the palette so even dimmed cells stay
 * readable on a black background. See documentation/COLOR.md. */
typedef struct {
    const char *name;       /* shown in the HUD                             */
    short       star  [4];  /* star colours, core to halo                   */
    short       nebula[4];  /* dust-cloud colours (dusty types only)        */
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

/* ── §2  PERFORMANCE — reading the clock and sleeping ── */

/* Just two clock helpers. The actual frame-rate capping lives in main (§8). */

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

/* ── §3  LOGIC — the math that decides what's where: hash, noise, density ── */

/* Everything here is pure: hand it some numbers, get a number back, no side
 * effects. This is where the whole galaxy actually lives — the dice (hash),
 * the wobble noise (Perlin/fBm), and density_at(), which says how bright the
 * galaxy is at any point. The renderer in §7 calls these per cell. */

/* All names padded to 10 chars so the HUD column never jumps around. */
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
 * Scrambles three whole numbers into one well-mixed 32-bit number. Same inputs
 * always give the same output, so a fixed galaxy point keeps its star instead
 * of flickering. We feed it a cell's location plus the pattern, then use the
 * result both as a dice roll (low bits) and to pick a glyph (high bits).
 * Multipliers from Teschner et al. (2003), finished with a splitmix mixer.
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

/* Turns a hash into a dice roll: a number from 0 up to (but not including) 1.
 * The star test compares this against the odds of placing a star. */
static inline float hash_unit(uint32_t h)
{
    return (float)(h & 0xFFFFFFu) / 16777216.0f;     /* 16777216 = 2^24 */
}

/*
 * Perlin noise (and fBm, several layers of it stacked) — the source of the
 * organic wobble. Copied from ../fields/perin_noise_flow_showcase.c so this
 * file stands alone.
 *
 * perm[] is the noise's shuffled lookup table: the numbers 0..255 in random
 * order, then written twice back to back (perm[i] equals perm[i+256]). The
 * doubling is a classic Perlin shortcut — lookups sometimes add 1 to an index
 * near 255, and the second copy lets that overflow read a valid slot without
 * any wrap-around check in the inner loop. Reshuffled only on reset (§4); the
 * noise functions only read it, which is why the same seed always draws the
 * same galaxy.
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

/* ── building blocks of the brightness field ── */

/*
 * How far a point sits from the nearest spiral arm, measured along its circle.
 * Zero means dead on an arm; bigger means out in the gap between arms. The
 * brightness functions below use this to fade arms in and out.
 *
 * Each arm is the curve θ = ln(r)/pitch (plus an even slice of the circle for
 * each extra arm). We subtract that off and wrap into one slice, so we always
 * land on the closest arm, then scale by r to turn the leftover angle into a
 * real distance. Right at the centre ln(r) blows up, so we clamp r to a small
 * floor first; the bright core swamps the arms there anyway, so it doesn't show.
 *
 * spiral_arc_p takes the winding tightness as an argument so the tight and
 * loose galaxy types can share this code; spiral_arc just uses the default.
 */
static float spiral_arc_p(float r, float theta, int n_arms, float pitch)
{
    float r_safe = (r < 0.04f) ? 0.04f : r;
    float seg = 2.0f * (float)M_PI / (float)n_arms;
    float psi = logf(r_safe) / pitch;
    float alpha = theta - psi;
    alpha = alpha - seg * floorf(alpha / seg + 0.5f);   /* fold onto the nearest arm */
    return alpha * r;
}
static float spiral_arc(float r, float theta, int n_arms)
{
    return spiral_arc_p(r, theta, n_arms, SPIRAL_PITCH);
}

/*
 * How bright an arm is at a given distance from it: full on the arm, fading
 * smoothly to nothing as you move away. The arm's width is wobbled a little by
 * the noise so the edges look ragged and natural instead of razor-clean.
 * arm_factor_w lets the caller set the base width; arm_factor uses the default.
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

/* The bright blob at the centre — brightest in the middle, fading outward. */
static inline float bulge_factor(float r)
{
    return expf(-r * r / (BULGE_SIGMA * BULGE_SIGMA));
}
/* The overall disk — how the brightness tapers off toward the edge. */
static inline float disk_envelope(float r)
{
    return expf(-r * r / (DISK_SIGMA * DISK_SIGMA));
}

/* A bright ring at a chosen radius — for ring galaxies like Hoag's Object and
 * the Cartwheel. Brightest right on the ring, fading on either side. */
static inline float ring_factor(float r, float ring_r, float ring_w)
{
    float d = r - ring_r;
    return expf(-d * d / (ring_w * ring_w));
}

/*
 * The straight bar across the centre of a barred galaxy: a bright oval lying
 * along the x-axis. The whole galaxy gets rotated elsewhere, which turns the
 * bar with it.
 */
static float bar_factor(float r, float theta)
{
    float gx = r * cosf(theta);
    float gy = r * sinf(theta);
    return expf(-(gx * gx) / (BAR_LEN  * BAR_LEN )
                -(gy * gy) / (BAR_WIDTH * BAR_WIDTH));
}

/*
 * The heart of the file: how bright the galaxy is at one point, for the chosen
 * type. r is the distance from the centre (1 is the disk edge), theta is the
 * angle around it, and fbm_val is the noise at that spot (it roughens the arms
 * and, for the dusty types, paints the clouds). Each type just mixes the
 * building blocks above its own way. The answer comes out around 0..1 and is
 * later scaled down into the odds of placing a star.
 */
static float density_at(float r, float theta, float fbm_val, Pattern p)
{
    if (r > DISK_R_MAX) return 0.0f;
    float fbm_c = (fbm_val - 0.5f) * 2.0f;             /* re-centre noise to -1..1 */
    float patch = 0.5f + 0.5f * fbm_c;                 /* 0..1 patchiness mask     */
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
        /* Fade the arms out wherever the bar is strong, so they only start
         * once you're past the ends of the bar. */
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
    default: {                      /* both are the plain 4-arm spiral */
        float arc  = spiral_arc(r, theta, N_ARMS_SPIRAL);
        float arm  = arm_factor(arc, fbm_c);
        return bulge * BULGE_AMP + disk * arm * DISK_AMP;
    }
    }
}

/*
 * Picks a star's colour from how far out it is. This copies real galaxies:
 * warm yellow-orange stars near the centre, cooler blue-white stars further
 * out. Returns 0 (core) through 3 (faint outer halo).
 */
static int star_color_idx(float r)
{
    if      (r < 0.10f) return 0;     /* core   — warm        */
    else if (r < 0.30f) return 1;     /* inner  — cream       */
    else if (r < 0.60f) return 2;     /* outer  — white/blue  */
    else                return 3;     /* halo   — faint       */
}

/* True for the types that also paint glowing dust clouds: the nebula plus the
 * two patchy, gas-rich ones. */
static bool pattern_has_dust(Pattern p)
{
    return p == PATTERN_NEBULA || p == PATTERN_FLOCCULENT || p == PATTERN_STARBURST;
}

/* ── §4  SIMULATION — the only code that changes the galaxy's state ── */

/* Almost nothing changes over time here: just the spin angle and a slow drift
 * of the noise. scene_tick nudges those forward each tick; reset/init set them
 * back to the start and reshuffle the noise. */

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
 * Scene holds the whole animated galaxy — and that's only a handful of numbers,
 * because the picture is rebuilt from scratch every frame rather than stored.
 * There is no array of stars anywhere. scene_tick is the only thing that
 * changes these each tick; key presses set the knobs and selections.
 */
typedef struct {
    /* what's moving */
    float   angle;            /* current spin, in radians, growing over time   */
    float   noise_time;       /* slow drift that makes clouds and arms shift    */
    /* the one knob the user turns */
    int     speed;            /* 1..SPEED_MAX; scales spin and drift            */
    /* what we're looking at (a choice, not motion) */
    Pattern current_pattern;  /* which galaxy type (n/p); also seeds the stars  */
    int     current_theme;    /* which colour scheme (t/T)                      */
    /* run state */
    bool    paused;           /* freeze the motion (drawing keeps going)        */
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
 * One step forward in time: turn the galaxy a little and drift the noise a
 * little. That's all the motion there is. We spin the whole galaxy as one rigid
 * disk; real spiral arms don't actually turn this way, but it looks right and
 * is simple.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->angle      += ROTATION_RATE * speed_mul * dt;
    s->noise_time += NOISE_DRIFT  * speed_mul * dt;
}

/* ── §5  EFFECTS — none ──
 * Nothing decorative is stored. The dust glow for the cloudy types is worked
 * out fresh at draw time, so there's no buffer to keep here. */

/* ── §6  DELAYS — none ──
 * The only timing control is the pause key, handled in scene_tick. The galaxy
 * just turns steadily, with no waits or holds. */

/* ── §7  RENDER — turn the numbers into characters on screen ── */

/* This draws everything. scene_draw works out, cell by cell, what the galaxy
 * looks like right now; screen_draw lays the HUD on top; theme_apply and
 * color_init set up the colours. None of it changes the galaxy's state — it
 * only reads and paints. */

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
 * Where the galaxy sits on the terminal: its size, its centre, and how big to
 * draw it. No galaxy state lives here, just layout. Recomputed at startup and
 * whenever the window is resized. The renderer uses it backwards — from a
 * character cell back to a point in the galaxy — to decide what to draw there.
 * r0 is the radius in columns; since cells are about twice as tall as wide, the
 * disk ends up looking round: 2·r0 wide and r0 tall. */
typedef struct {
    int cols, rows;     /* terminal size                                    */
    int cx, cy;         /* where the galaxy's centre lands on screen        */
    int r0;             /* galaxy radius, in columns (this is the r=1 mark)  */
} Screen;

static void screen_layout(Screen *s)
{
    s->cx = s->cols / 2;
    /* Centre it in the space left between the two HUD rows up top and the
     * one hint row at the bottom. */
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

/* Draw one star: colour by how far out it is, glyph and brightness by how dense
 * the galaxy is there. A few hash bits pick which glyph shape, for variety. */
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

/* Draw one patch of glowing dust cloud: brighter where the noise is stronger,
 * coloured from the dust palette by a few hash bits. */
static void draw_dust_cell(int sy, int sx, float fbm_val, uint32_t h)
{
    float intensity = (fbm_val - NEBULA_THRESH) / (1.0f - NEBULA_THRESH);  /* rescale to 0..1 */
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
