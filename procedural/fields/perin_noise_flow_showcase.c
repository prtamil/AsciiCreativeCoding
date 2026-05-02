/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * perin_noise_flow_showcase.c
 *   — Particles flowing along a 2-D Perlin-noise gradient field.
 *
 * DEMO: 256 particles wander a coherent vector field derived from
 *       Perlin noise. At every step, each particle samples the noise
 *       at its current position; the noise value (in [-1, 1]) is
 *       multiplied by 2π to produce an angle, and the particle steps
 *       one unit in that direction. The result: smooth, curving
 *       streamlines that trace the "wind" pattern of the noise field.
 *       Trails fade behind each particle, density-shaded into ASCII
 *       glyphs ('.', '*', '#'). The field slowly drifts via a time
 *       offset, so the flow evolves rather than looping. Periodic
 *       supernova reset re-shuffles the Perlin permutation table for
 *       a wholly new flow pattern.
 *
 * Study alongside: ../generational/voronoi_region_map.c — the other
 *       "field over the whole grid" showcase. Voronoi answers a
 *       discrete classifier question (nearest seed); Perlin noise is
 *       a continuous scalar field that you USE to drive other things
 *       (terrain, particles, textures). This file uses it as a flow
 *       field; the same noise + a height ramp would give terrain.
 *
 * Section map:
 *   §1 config   — grid, particle count, noise scale, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (4 trail colours each)
 *   §5 noise    — Perlin permutation + 2-D gradient noise
 *   §6 flow     — Particle, Flow, particle_step, scene state machine
 *   §7 screen   — ASCII render: density-graded trail glyphs
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (new noise permutation)
 *   n / N      next pattern (FLOW → HEIGHT → WARP → FBM → CONTOUR → ...)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster particles / faster field drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra perin_noise_flow_showcase.c \
 *       -o perlin_flow -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two pieces composed together.
 *                  (1) Perlin noise — Ken Perlin's 1985 gradient noise
 *                      function. Pseudo-random values at integer lattice
 *                      points are smoothly interpolated to produce a
 *                      continuous, naturally-varying scalar field. We
 *                      use the standard 2-D variant with a 256-entry
 *                      permutation table (duplicated to 512 to avoid
 *                      modulo at lookup time), Perlin's quintic fade
 *                      curve t³(t·(t·6-15)+10), and 8 hashed gradient
 *                      directions per lattice corner.
 *                  (2) Flow visualisation — for each particle, read
 *                      n = perlin(x·scale, y·scale + t) and let
 *                      angle = n · 2π. Step the particle by
 *                      (cos angle, sin angle) · speed each tick.
 *                      Curving paths emerge because the noise field
 *                      is locally smooth, so neighbouring particles
 *                      follow similar directions and the system
 *                      organises into streamlines.
 *
 *                  The TIME OFFSET (the "+ t" in y) makes the field
 *                  drift slowly, so the flow evolves rather than
 *                  looping in a fixed pattern. The drift rate is
 *                  tunable; default ~0.1 noise-units per second.
 *
 * Data-structure : Static permutation array (perm[512]) for noise.
 *                  Static particle pool (Particle[MAX_PARTICLES]).
 *                  Per-cell trail_glow + trail_color arrays for the
 *                  fading streamline visualisation. No heap allocation.
 *
 * Rendering      : ASCII only. Per-cell trail_glow drives a density
 *                  ramp:
 *                    glow > 0.65 → '#'  (high density, BOLD)
 *                    glow > 0.30 → '*'  (medium)
 *                    glow > 0.05 → '.'  (low / fading)
 *                    else        → blank
 *                  Each cell also tracks the COLOUR of the most-recent
 *                  particle to visit it (one of 4 theme colours), so
 *                  multiple particle "tribes" produce visibly distinct
 *                  streams. The density ramp on top of the colour
 *                  variation gives the flow a textured, painted look.
 *
 * Performance    : O(N_particles) per tick — each particle does 1
 *                  Perlin lookup + 1 cell paint. With 256 particles
 *                  at 60 Hz that's ~15K perlin evaluations per second,
 *                  about 120 K arithmetic ops — completely free.
 *                  Trail decay is O(W·H) per tick (one multiply per
 *                  cell), ~700 K ops per second on a 200×56 grid.
 *                  Headroom for any speedup.
 *
 * References     : • Perlin, K. (1985) — "An image synthesizer", SIGGRAPH.
 *                    The original paper:
 *                    https://web.archive.org/web/20221002071856/https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *                  • Perlin, K. (2002) — "Improving noise" (the quintic
 *                    fade curve we use):
 *                    https://mrl.cs.nyu.edu/~perlin/paper445.pdf
 *                  • Inigo Quilez — "Domain warping" + flow-field
 *                    techniques:
 *                    https://iquilezles.org/articles/warp/
 *                  • The Book of Shaders — chapter on noise:
 *                    https://thebookofshaders.com/11/
 *                  • Wikipedia — "Perlin noise":
 *                    https://en.wikipedia.org/wiki/Perlin_noise
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Perlin noise is a way to build a SMOOTH, naturally-varying scalar
 * field over an arbitrary domain. Think of it as a height map of
 * rolling hills with no axis bias and no obvious repetition. By
 * itself it's just a cloud-like pattern; combine it with anything
 * directional (here: an angle for particle motion) and you get
 * structures that look organic — wind, fluid, smoke, cellular flow.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the screen has invisible arrows on it, one per cell. Each
 * arrow points in a direction determined by the local noise value.
 * Drop hundreds of dust particles on the screen. Each particle reads
 * its arrow and steps in that direction. The dust accumulates along
 * the streamlines — visually the same as iron filings around a
 * magnet, except the field is a 2-D Perlin "wind".
 *
 * Why does the result look organic? Because Perlin noise is
 * SPATIALLY COHERENT — neighbouring points have nearby values, so
 * neighbouring particles see similar arrows, so streams form. Pure
 * white noise (uncorrelated random per cell) would scatter the
 * particles; Perlin's smoothness is what produces flow.
 *
 * Visible layers:
 *   1. Trails of '.', '*', '#' in 4 theme colours — recently-visited
 *      cells in their colour, density-shaded by glow.
 *   2. A faint structural pattern — high-density convergence zones
 *      (where streams meet) and sparse divergence zones (where they
 *      separate) emerge naturally from the noise topology.
 *   3. The flow EVOLVES — over a few seconds you see streams curve,
 *      bifurcate, dissolve, reform. That's the field-time drift.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Shuffle perm[256], duplicate to perm[512]. Spawn N
 *     particles at random in-bounds positions, each with a random
 *     colour (1 of 4 theme tribes) and a random max_age.
 *  2. STEP (one operation per particle, per tick):
 *     a. n = perlin2d(x · NOISE_SCALE, y · NOISE_SCALE + field_time)
 *     b. angle = n · 2π · ANGLE_SCALE
 *     c. (vx, vy) = (cos angle, sin angle) · SPEED
 *     d. (x, y) ← (x, y) + (vx, vy) · dt
 *     e. age++. If age ≥ max_age OR (x, y) out of bounds:
 *        respawn at random in-bounds position, age = 0,
 *        new random colour and max_age.
 *     f. paint trail_glow[cell(x,y)] = 1.0,
 *               trail_color[cell(x,y)] = particle.colour.
 *  3. DECAY. Every cell's trail_glow *= exp(-DECAY · dt) per tick.
 *  4. DRIFT. field_time += FIELD_DRIFT · dt — slow horizontal slide
 *     of the noise field so the flow evolves.
 *  5. Periodically (every RESET_SECONDS): supernova flash, re-shuffle
 *     perm[], spawn fresh particles. Goto 2.
 *
 * KEY FORMULAS
 * ────────────
 *  Perlin 2-D                   :
 *    1. (X, Y) = floor(x), floor(y); (xf, yf) = x − X, y − Y
 *    2. (u, v) = fade(xf), fade(yf)   where fade(t) = 6t⁵ − 15t⁴ + 10t³
 *    3. Hash 4 lattice corners via perm[] table
 *    4. Compute 4 gradient·offset dot products (grad function)
 *    5. Bilinear lerp using (u, v) → noise value in roughly [-1, 1]
 *  Particle angle               : θ = n · 2π
 *  Step                         : (Δx, Δy) = (cos θ, sin θ) · SPEED · dt
 *  Field-drift                  : field_time' = field_time + DRIFT · dt
 *  Glow decay                   : glow' = glow · exp(-DECAY · dt)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PERMUTATION TABLE SIZE. The perm[] is duplicated (256 → 512)
 *    so the lookups perm[X] and perm[X+1] never need a modulo. Skip
 *    the duplication and you'll get wrong noise values at X = 255.
 *
 *  • FADE CURVE CHOICE. Perlin's original 1985 paper used 3t² − 2t³
 *    (cubic). The 2002 "improved noise" paper recommends the quintic
 *    6t⁵ − 15t⁴ + 10t³ — it has zero second derivative at the lattice
 *    points, so the field is C² continuous. The cubic version has
 *    visible gridding artefacts at the lattice. Always use quintic.
 *
 *  • GRADIENT FUNCTION. Our grad() picks one of 8 directions based
 *    on the low 3 bits of the hash. The result is in roughly [-1, 1]
 *    after the bilinear lerp, but the EXACT bound is √(N/2) for an
 *    N-D grid (so √2/2 ≈ 0.707 for 2-D after our scaling). We
 *    multiply by 1.414 to roughly normalise.
 *
 *  • OUT-OF-BOUNDS PARTICLES. With (cos, sin) motion a particle CAN
 *    walk straight off the map. Respawn-on-OOB handles this; without
 *    it, particles disappear and the flow gradually empties the map.
 *
 *  • TIME DRIFT VS PARTICLE SPEED. If the field drifts faster than
 *    particles flow, the streams break up faster than they form.
 *    Default DRIFT = 0.1, SPEED = 8 cells/sec — drift slower than
 *    motion, streams persist long enough to be visible.
 *
 *  • TRAIL OVERWRITE. When two particles visit the same cell in the
 *    same tick, the LATER one wins (its colour persists). For random
 *    particle spawn order this is unbiased; if you sort particles
 *    by colour, the colour distribution becomes visibly skewed.
 *
 *  • PERLIN AT INTEGER COORDS. perlin(0, 0) = 0 always, by
 *    construction. Don't accidentally sample at exact integers if
 *    you want random values; offset coordinates by the particle's
 *    sub-cell position (we do — particles store float x, y).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At t=0, particles spawn uniformly. Wait ~3 s; you should see
 *    distinct streamlines forming, not a uniform speckle. If the
 *    particles never organise, the noise field is too low-amplitude
 *    or the particles are stepping in random directions (check
 *    cosf/sinf signs).
 *  • Streamlines curve smoothly — no sharp 90-degree turns. Sharp
 *    turns indicate either fade-curve cubic instead of quintic, or
 *    the perm[] table not duplicated.
 *  • Field drift visible: the streamlines move slowly across the
 *    screen, curving differently as the underlying field shifts.
 *  • Different themes change colours but flow STRUCTURE is identical
 *    for the same RNG seed. Algorithm is theme-independent.
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

    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* Particle lifetime range (ticks). Random max_age in this window
     * so different particles "tire out" at different times → the
     * streamlines are constantly being refreshed at random points. */
    AGE_MIN_TICKS     =  60,        /* 1.0 s @ 60 Hz */
    AGE_MAX_TICKS     = 360,        /* 6.0 s         */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Speed multiplier (cells per second per particle). Default 8 ≈
     * 0.13 cells/tick at 60 Hz — slow enough to read individual
     * trails, fast enough that the flow evolves visibly. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* How often to restart with a fresh permutation. ~12 s gives time
     * to appreciate one flow before the next. */
    RESET_TICKS_DEF   = 12 * 60,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_TRAIL_BASE   =   3,        /* PAIR_TRAIL_BASE..+3 = 4 trail colours */
    PAIR_FLASH        =   7,        /* spawn flash                            */
    PAIR_SUPERNOVA    =   8,        /* yellow reset flash                     */
};

/* Glow decay rate per second. 0.6 → glow ≈ 5% after 5 seconds, so
 * trails persist long enough to read but fade out cleanly. */
#define GLOW_DECAY          0.6f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

/* Noise scale: lower = larger features, higher = finer detail. 0.04
 * gives noise periods of ~25 cells — balanced for 200×56 grids. */
#define NOISE_SCALE         0.04f

/* Field drift per second (in noise-coordinate units). Slow drift so
 * the flow visibly evolves but doesn't churn. */
#define FIELD_DRIFT         0.10f

/* Density thresholds for the ASCII glyph ramp. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * Pattern — five distinct ways to visualise a Perlin-noise field.
 * Cycle through with n/p keys.
 *
 *   FLOW    : particles follow the noise gradient (the original demo)
 *   HEIGHT  : direct noise → density+colour, like a heightmap
 *   WARP    : domain-warped noise (Inigo Quilez technique) — more
 *             swirly and organic than raw noise
 *   FBM     : fractional Brownian motion (stacked octaves) — adds
 *             fine detail on top of the large-scale shape
 *   CONTOUR : only cells near specific noise levels render — looks
 *             like a topographic map
 */
typedef enum {
    PATTERN_FLOW    = 0,
    PATTERN_HEIGHT  = 1,
    PATTERN_WARP    = 2,
    PATTERN_FBM     = 3,
    PATTERN_CONTOUR = 4,
    N_PATTERNS      = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_FLOW:    return "FLOW   ";
    case PATTERN_HEIGHT:  return "HEIGHT ";
    case PATTERN_WARP:    return "WARP   ";
    case PATTERN_FBM:     return "FBM    ";
    case PATTERN_CONTOUR: return "CONTOUR";
    default:              return "?      ";
    }
}

/* WARP pattern: amount the warp distorts the input coordinates.
 * Higher = more swirly, lower = closer to plain noise. */
#define WARP_AMOUNT         5.0f

/* FBM pattern: how many octaves to stack. 4 is a good sweet spot —
 * adds visible fine detail without becoming noisy. */
#define FBM_OCTAVES         4

/* CONTOUR pattern: noise values that draw bands. The band width
 * controls how thick each contour line appears. */
#define CONTOUR_BAND_WIDTH  0.04f
static const float CONTOUR_LEVELS[4] = { 0.20f, 0.40f, 0.60f, 0.80f };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names as the other procedural showcases. Each
 * theme defines 4 trail colours (cycling per particle) plus a flash
 * accent. PAIR_HUD/HINT/SUPERNOVA stay theme-independent.
 */
typedef struct {
    const char *name;
    short       trail[4];
    short       flash;
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
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_TRAIL_BASE + i, t->trail[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_TRAIL_BASE + i, fallback[i], -1);
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
/* §5  noise — Perlin 2-D                                                 */
/* ===================================================================== */

/*
 * Permutation table — Ken Perlin's classic 256-entry array, shuffled
 * at each reset and duplicated to perm[512] so lookups perm[X] and
 * perm[X+1] never need an explicit modulo.
 */
static uint8_t perm[512];

/*
 * perm_shuffle — Fisher-Yates the 0..255 sequence using rand(), then
 * duplicate into perm[256..511]. Different rand() state → different
 * permutation → wholly different noise field. Called at each reset.
 */
static void perm_shuffle(void)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i]       = base[i];
        perm[i + 256] = base[i];
    }
}

/*
 * fade — Perlin's improved-noise quintic fade curve.
 *   fade(t) = 6t⁵ − 15t⁴ + 10t³
 * Has zero first AND second derivative at t=0 and t=1, so the
 * resulting noise is C² continuous across lattice boundaries.
 */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp(float a, float b, float t) { return a + t * (b - a); }

/*
 * grad — pick one of 8 gradient directions based on the low 3 bits
 * of the hash, and return its dot product with the offset (x, y).
 * The 8 directions are the diagonals and axes of a 2-D unit square.
 */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * perlin2d — Perlin's 2-D noise function. Returns a value roughly in
 * [-1, 1]. The magnitude bound is √(N/2) for N-D noise, ≈ 1 in 2-D
 * after the small constant scaling we apply.
 */
static float perlin2d(float x, float y)
{
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

/*
 * pattern_height_at — basic noise visualization. Returns noise value
 * remapped to [0, 1] so it can be used directly as a glow intensity.
 */
static float pattern_height_at(float x, float y, float t)
{
    float n = perlin2d(x * NOISE_SCALE, y * NOISE_SCALE + t);
    return n * 0.5f + 0.5f;
}

/*
 * pattern_warp_at — domain-warped noise (Inigo Quilez style). Sample
 * the noise field, use the result to OFFSET the coordinates, then
 * sample again. Produces curving, organic-looking patterns that feel
 * less "lumpy" than raw Perlin noise. The two-step warp uses
 * different y-offsets so the qx and qy components are statistically
 * independent.
 */
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

/*
 * pattern_fbm_at — fractional Brownian motion. Sums multiple octaves
 * of Perlin noise: each octave doubles the frequency and halves the
 * amplitude. The result has the same large-scale shape as plain
 * Perlin but with self-similar fine detail at smaller scales.
 *
 * For terrain, fBm is the standard way to add "rocks on hills on
 * mountains" — every scale gets a contribution.
 */
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

/*
 * pattern_contour_at — only "lights up" cells near specific noise
 * levels (CONTOUR_LEVELS). The result is intensity (1.0 at the level,
 * fading to 0 at ±CONTOUR_BAND_WIDTH) plus the index of the closest
 * level (returned via *out_band, so the caller can colour each
 * contour line distinctly).
 *
 * Visually like a topographic map — the noise value is constant along
 * each line, so neighbouring cells track each other in a smooth path.
 */
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

/* ===================================================================== */
/* §6  flow — Particle, scene state, simulation tick                      */
/* ===================================================================== */

/*
 * Particle — one flowing dust grain.
 *
 *   x, y     : continuous position in cell units
 *   color_idx: 0..3 — picks one of 4 theme trail colours
 *   age      : ticks alive
 *   max_age  : random lifetime; respawn once age == max_age
 */
typedef struct {
    float x, y;
    int   color_idx;
    int   age;
    int   max_age;
} Particle;

/*
 * Flow — the whole simulation state.
 *
 *   tiles+glow per cell:
 *     trail_glow[]  — per-cell intensity, decays each tick
 *     trail_color[] — the colour_idx of the most recent visitor
 *     supernova_glow_t — single global flag for the reset flash
 *
 *   particles[]    — pool, n_particles in use
 *
 *   field_time     — drift offset added to noise y-coord; advances
 *                    by FIELD_DRIFT · dt each tick
 *
 *   reset_countdown — ticks until the next supernova reset
 */
typedef struct {
    int      w, h;
    int      total_cells;

    float    trail_glow [CELLS_MAX];
    uint8_t  trail_color[CELLS_MAX];
    float    supernova_glow_t;

    Particle particles[MAX_PARTICLES];
    int      n_particles;

    float    field_time;
    int      reset_countdown;
} Flow;

static inline int flow_idx(const Flow *f, int x, int y) { return y * f->w + x; }
static inline bool flow_in_bounds(const Flow *f, int x, int y)
{
    return x >= 0 && x < f->w && y >= 0 && y < f->h;
}

/*
 * particle_spawn — initialise a particle at a random in-bounds cell
 * with a random colour and a random max_age.
 */
static void particle_spawn(Flow *f, Particle *p)
{
    p->x         = (float)(rand() % f->w);
    p->y         = (float)(rand() % f->h);
    p->color_idx = rand() & 3;
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/*
 * particle_step — sample noise at the particle's current position,
 * derive an angle, step the particle. Respawn on OOB or max_age.
 */
static void particle_step(Flow *f, Particle *p, float dt, int speed)
{
    float n     = perlin2d(p->x * NOISE_SCALE,
                           p->y * NOISE_SCALE + f->field_time);
    float angle = n * (float)M_PI;          /* wider sweep than 2π·n */
    float vx    = cosf(angle) * (float)speed;
    float vy    = sinf(angle) * (float)speed;

    p->x += vx * dt;
    p->y += vy * dt;
    p->age++;

    int cx = (int)p->x;
    int cy = (int)p->y;
    if (flow_in_bounds(f, cx, cy)) {
        int idx = flow_idx(f, cx, cy);
        f->trail_glow[idx]  = 1.0f;
        f->trail_color[idx] = (uint8_t)p->color_idx;
    }

    if (p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)f->w
        || p->y < 0.0f || p->y >= (float)f->h) {
        particle_spawn(f, p);
    }
}

/*
 * flow_reset — clear the trail field, re-shuffle the Perlin
 * permutation, respawn every particle. Painted with a global
 * supernova_glow_t = 1.0 that fades over ~0.4 s.
 */
static void flow_reset(Flow *f, int w, int h)
{
    f->w = w;
    f->h = h;
    f->total_cells = w * h;
    f->n_particles = N_PARTICLES_DEF;
    f->field_time = 0.0f;
    f->reset_countdown = RESET_TICKS_DEF;
    f->supernova_glow_t = 1.0f;

    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i]  = 0.0f;
        f->trail_color[i] = 0;
    }

    perm_shuffle();
    for (int i = 0; i < f->n_particles; i++) {
        particle_spawn(f, &f->particles[i]);
    }
}

/*
 * Scene — wraps the Flow and the user-facing toggles. There's no
 * discrete state machine like the generational showcases — flow runs
 * continuously with a periodic reset.
 */
typedef struct {
    Flow    Flo;
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    int     reset_total_ticks;     /* updated each scene_init */
} Scene;

/*
 * field_update_grid — for non-FLOW patterns, recompute trail_glow
 * and trail_color at every cell using the active pattern's noise
 * function. Called once per scene_tick. No decay needed because we
 * fully overwrite each frame.
 */
static void field_update_grid(Flow *f, Pattern p, float t)
{
    for (int y = 0; y < f->h; y++) {
        for (int x = 0; x < f->w; x++) {
            int idx = flow_idx(f, x, y);
            float glow = 0.0f;
            int   col  = 0;

            switch (p) {
            case PATTERN_HEIGHT: {
                float n = pattern_height_at((float)x, (float)y, t);
                glow = n;
                col  = (int)(n * 3.99f) & 3;
                break;
            }
            case PATTERN_WARP: {
                float n = pattern_warp_at((float)x, (float)y, t);
                glow = n;
                col  = (int)(n * 3.99f) & 3;
                break;
            }
            case PATTERN_FBM: {
                float n = pattern_fbm_at((float)x, (float)y, t);
                glow = n;
                col  = (int)(n * 3.99f) & 3;
                break;
            }
            case PATTERN_CONTOUR: {
                int band;
                glow = pattern_contour_at((float)x, (float)y, t, &band);
                col  = band & 3;
                break;
            }
            default:
                continue;       /* PATTERN_FLOW handled by particles */
            }
            f->trail_glow[idx]  = glow;
            f->trail_color[idx] = (uint8_t)(col & 3);
        }
    }
}

/*
 * scene_clear_field — wipe the trail buffer. Called when the user
 * switches patterns so we don't leave streaks from the previous
 * pattern's particles.
 */
static void scene_clear_field(Scene *s)
{
    Flow *f = &s->Flo;
    for (int i = 0; i < f->total_cells; i++) {
        f->trail_glow[i]  = 0.0f;
        f->trail_color[i] = 0;
    }
}

static void scene_reset(Scene *s, int mw, int mh)
{
    flow_reset(&s->Flo, mw, mh);
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused           = false;
    s->speed            = SPEED_DEF;
    s->current_theme    = 0;
    s->current_pattern  = PATTERN_FLOW;
    s->reset_total_ticks = RESET_TICKS_DEF;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    Flow *f = &s->Flo;

    /* Supernova fades regardless of pattern. */
    float decay_n = expf(-SUPERNOVA_DECAY * dt);
    f->supernova_glow_t *= decay_n;

    /* Drift the noise field — applies to every pattern. */
    f->field_time += FIELD_DRIFT * dt;

    /* Pattern-specific simulation. */
    if (s->current_pattern == PATTERN_FLOW) {
        /* Decay only matters for FLOW (where particles repaint).
         * Other patterns fully overwrite each frame. */
        float decay_t = expf(-GLOW_DECAY * dt);
        for (int i = 0; i < f->total_cells; i++) {
            f->trail_glow[i] *= decay_t;
        }
        for (int i = 0; i < f->n_particles; i++) {
            particle_step(f, &f->particles[i], dt, s->speed);
        }
    } else {
        /* Static-field patterns (HEIGHT/WARP/FBM/CONTOUR): rewrite
         * every cell each frame from the current noise state. */
        field_update_grid(f, s->current_pattern, f->field_time);
    }

    /* Periodic reset for variety. */
    f->reset_countdown--;
    if (f->reset_countdown <= 0) {
        flow_reset(f, f->w, f->h);
    }
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Flow *f = &s->Flo;

    int gx0 = (cols - f->w) / 2;
    int gy0 = ((rows - 3) - f->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < f->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < f->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = flow_idx(f, x, y);
            float ng = f->supernova_glow_t;     /* same value for whole grid */
            float tg = f->trail_glow[idx];

            int  pair, attr;
            char glyph;

            if (ng > GLOW_THRESHOLD) {
                /* Reset flash — sparse-ish so it reads as a flash. */
                if (((x ^ y) & 3) != 0) {
                    /* Skip most cells so the flash isn't a solid fill. */
                    if (tg <= GLOW_THRESHOLD) continue;
                }
                pair = PAIR_SUPERNOVA;
                attr = A_BOLD;
                glyph = '*';
            } else if (tg > GLYPH_HIGH_THRESH) {
                pair  = PAIR_TRAIL_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = '#';
            } else if (tg > GLYPH_MID_THRESH) {
                pair  = PAIR_TRAIL_BASE + (f->trail_color[idx] & 3);
                attr  = A_BOLD;
                glyph = '*';
            } else if (tg > GLOW_THRESHOLD) {
                pair  = PAIR_TRAIL_BASE + (f->trail_color[idx] & 3);
                attr  = A_NORMAL;
                glyph = '.';
            } else {
                continue;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Flow *f = &s->Flo;
    const char *state_str = s->paused
                          ? "PAUSED "
                          : pattern_name(s->current_pattern);

    /* Ticks until next reset, in seconds (rough — assumes 60 Hz). */
    float reset_secs = (float)f->reset_countdown / 60.0f;
    if (reset_secs < 0.0f) reset_secs = 0.0f;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d  reset:%4.1fs ",
             fps, sim_fps, state_str, s->speed, (double)reset_secs);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " PERLIN-NOISE FLOW ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — pattern + theme + colour swatches. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-7s ", pattern_name(s->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 18;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    for (int i = 0; i < 4; i++) {
        int p = PAIR_TRAIL_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             "  scale:%.2f  drift:%.2f  map:%dx%d ",
             NOISE_SCALE, FIELD_DRIFT, f->w, f->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " .:low  *:mid  #:high | n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;
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

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->speed < SPEED_MAX) s->speed *= 2;
        if (s->speed > SPEED_MAX) s->speed = SPEED_MAX;
        break;
    case '-':
        s->speed /= 2;
        if (s->speed < SPEED_MIN) s->speed = SPEED_MIN;
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

    /* Pattern cycling — n forward, p backward. Clear the trail
     * buffer on switch so leftover particle trails don't ghost into
     * the new pattern's view. */
    case 'n': case 'N':
        s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
        scene_clear_field(s);
        break;
    case 'p': case 'P':
        s->current_pattern = (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
        scene_clear_field(s);
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
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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
