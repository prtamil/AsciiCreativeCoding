/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * volcano.c — an erupting volcano built from five layered passes (sky,
 * mountain, lava flows, smoke plume, particles) over one shared pool of
 * 1024 particles.  Cycle eruption styles with n/N and colour themes with
 * t/T.  The eruption styles are named after the real volcanological
 * taxonomy: Strombolian, Vulcanian, Plinian, Hawaiian (Pyle, 2015).
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

/* ── §1 CONFIG — constants, enums, theme + pattern tables, glyph ramps ── */

enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_SKY_BASE     =  3,    /* +0..+3 — top→horizon sky gradient    */
    PAIR_MOUNTAIN     =  7,    /* mountain silhouette                  */
    PAIR_LAVA_BASE    =  8,    /* +0..+5 — heat ramp dim→hot           */
    PAIR_PLUME_BASE   = 14,    /* +0..+3 — plume gradient core→edge    */
    PAIR_PAPER        = 18,    /* NEGATIVE white-paper bg              */

    /* Particle pool. */
    PARTICLES_MAX     = 1024,

    /* Mountain heightmap maximum width. */
    MTN_MAX_W         = 280,

    /* Plume buffer dimensions (mirror screen extents up to limits). */
    SCREEN_MAX_W      = 280,
    SCREEN_MAX_H      = 90,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Mountain. */
#define MTN_PEAK_FRAC    0.42f     /* peak y as fraction of rows         */
#define MTN_SLOPE        0.55f     /* rise per col away from peak        */
#define MTN_NOISE_AMP    1.4f      /* 1-D surface noise amplitude (rows) */
#define MTN_NOISE_SCALE  0.18f     /* 1-D surface noise scale            */
#define CRATER_RADIUS    4.0f      /* cells; the bowl at the summit      */
#define CRATER_DEPTH     2.0f      /* dip at crater centre (rows)        */
#define CRATER_X_JITTER  0.30f     /* vent column wander, fraction of cols */
#define MTN_FLOW_MAX_LEN 24        /* longest lava-flow streamline (cells) */

/* Plume. */
#define PLUME_BASE_W     5.0f      /* base width at crater rim (cells)   */
#define PLUME_SPREAD     0.40f     /* width grows with altitude          */
#define PLUME_NOISE_S    0.18f     /* fBm domain scale                   */
#define PLUME_WIND       2.5f      /* +x drift speed (cells/sec)         */
#define PLUME_RISE       1.8f      /* column upward drift in noise space */
#define PLUME_THRESHOLD  0.45f     /* fbm above this → visible plume     */
#define PLUME_NOISE_SEED 7777u     /* fixed seed for the plume fBm field */

/* Particle physics. */
#define GRAVITY          24.0f     /* +y, cells/sec²                     */
#define BOMB_DRAG        0.04f     /* per second                         */
#define EMBER_BUOYANCY   12.0f     /* upward acceleration                */
#define ASH_BUOYANCY      4.0f
#define COOL_BOMB         0.18f    /* temp decay per sec                 */
#define COOL_EMBER        0.30f
#define COOL_ASH          0.05f    /* ash barely cools — long grey drift */
#define COOL_SPARK        2.50f    /* sparks fade fast                   */

/* Per-type force coupling (multipliers on shared forces). */
#define EMBER_TURBULENCE      2.0f /* ember vx random kick (±, cells/s²) */
#define EMBER_WIND_FACTOR     0.4f /* fraction of PLUME_WIND felt by embers */
#define ASH_TURBULENCE        1.0f /* ash vx random kick (±, cells/s²)   */
#define ASH_WIND_FACTOR       0.6f /* fraction of PLUME_WIND felt by ash */
#define SPARK_GRAVITY_FACTOR  0.5f /* sparks feel reduced gravity        */

/* Off-screen cull margins — how far past the edge before a particle dies. */
#define PARTICLE_CULL_MARGIN_Y 2.0f /* rows above top / below bottom     */
#define PARTICLE_CULL_MARGIN_X 3.0f /* cols past left / right            */

/* Ranges per particle type for spawn-time randomisation. */
#define BOMB_LIFE_MIN     2.0f
#define BOMB_LIFE_MAX     4.0f
#define BOMB_SPEED_MIN   18.0f
#define BOMB_SPEED_MAX   30.0f
#define BOMB_ANGLE       1.05f     /* ±radians from straight up          */

#define EMBER_LIFE_MIN    3.0f
#define EMBER_LIFE_MAX    6.0f
#define EMBER_SPEED_MIN   3.0f
#define EMBER_SPEED_MAX   8.0f

#define ASH_LIFE_MIN      6.0f
#define ASH_LIFE_MAX     12.0f
#define ASH_SPEED_MIN     2.0f
#define ASH_SPEED_MAX     5.0f

#define SPARK_LIFE_MIN    0.4f
#define SPARK_LIFE_MAX    0.9f
#define SPARK_SPEED_MIN  18.0f
#define SPARK_SPEED_MAX  35.0f

/* Bomb trail length. */
#define BOMB_TRAIL_LEN   4

/* Spawn scatter (± cells around the vent) and initial heat (0..1). */
#define BOMB_SPAWN_DX     1.5f
#define EMBER_SPAWN_DX    2.0f
#define EMBER_SPAWN_VX    1.5f      /* ember initial horizontal velocity (±) */
#define EMBER_SPAWN_TEMP  0.9f
#define ASH_SPAWN_DX      3.0f
#define ASH_SPAWN_LIFT    4.0f      /* ash starts up to this many rows above vent */
#define ASH_SPAWN_VX      2.0f      /* ash initial horizontal velocity (±)   */
#define ASH_SPAWN_TEMP    0.4f
#define SPARK_SPAWN_DX    2.0f
#define SPARK_SPAWN_ANGLE 1.5f      /* ± radians from vertical for spark jets */
#define SPARK_VY_FACTOR   0.8f      /* spark upward-velocity scale           */

/* Burst intensity boosts (multiply the base intensity for that volley). */
#define VULCAN_BURST_BOOST   1.3f
#define AMBIENT_BURST_BOOST  1.1f

/*
 * Pattern — which of the four eruption styles is active (n/N cycles
 * through them).  Each value indexes the patterns[] table below; the names
 * are real volcano types (Pyle, 2015), each with its own feel:
 *   PAT_STROMBOLIAN — steady rhythmic bursts; lots of glowing embers.
 *   PAT_VULCANIAN   — quiet most of the time, then a sudden violent volley.
 *   PAT_PLINIAN     — one huge towering ash column, very heavy plume.
 *   PAT_HAWAIIAN    — a bright lava fountain with rivers down the slopes.
 * N_PATTERNS is the count, kept last so the "wrap to next style" math and
 * the patterns[] table size stay in sync.
 */
typedef enum {
    PAT_STROMBOLIAN = 0,
    PAT_VULCANIAN   = 1,
    PAT_PLINIAN     = 2,
    PAT_HAWAIIAN    = 3,
    N_PATTERNS      = 4,
} Pattern;

/*
 * EruptionPattern — one row of the patterns[] table, one row per style.
 * This is what makes a style a style: the physics never branches on which
 * style is active, it just reads these numbers, so adding a new style means
 * adding a new row, not new code.  Each row is basically "how often does
 * each kind of particle appear, and how does the plume and lava look".
 */
typedef struct {
    const char *name;          /* label shown in the HUD, padded to fixed width  */
    float bomb_per_sec;        /* average lava bombs launched per second         */
    float ember_per_sec;       /* average rising glowing embers per second       */
    float ash_per_sec;         /* average slow grey ash specks per second        */
    float spark_per_sec;       /* average fast crater-rim sparks per second      */
    float plume_intensity;     /* smoke-column density: ~0.6 wispy to ~1.8 towering */
    bool  vulcanian_bursts;    /* if true, also fire a big bomb volley every few sec */
    float lava_flow_amount;    /* 0..1 how bright the rivers down the slopes glow */
} EruptionPattern;

static const EruptionPattern patterns[N_PATTERNS] = {
    /* name           bomb  ember  ash    spark  plume  burst  flow  */
    /* STROMBOLIAN — moderate rhythmic                              */
    { "STROMBOLIAN", 12.0f,  60.0f,  35.0f,  30.0f, 1.0f,  false, 0.30f },
    /* VULCANIAN   — periodic violent bursts                        */
    { "VULCANIAN  ",  4.0f,  35.0f,  25.0f,  18.0f, 0.8f,  true,  0.20f },
    /* PLINIAN     — continuous massive plume                       */
    { "PLINIAN    ",  6.0f,  90.0f,  90.0f,  20.0f, 1.8f,  false, 0.25f },
    /* HAWAIIAN    — strong fountain + lava flows, lighter ash     */
    { "HAWAIIAN   ", 22.0f, 100.0f,  20.0f,  60.0f, 0.6f,  false, 0.95f },
};

#define VULCAN_BURST_INTERVAL_MIN  4.0f
#define VULCAN_BURST_INTERVAL_MAX  7.0f
#define VULCAN_BURST_BOMBS         55

/*
 * Ambient burst — an occasional dramatic surge that fires no matter which
 * style is active.  The gap between surges is random and wide (8 to 22 sec)
 * so the timing stays unpredictable.  Its bomb count is deliberately kept
 * below the Vulcanian volley so a true Vulcanian burst still feels bigger.
 */
#define AMBIENT_BURST_INTERVAL_MIN  8.0f
#define AMBIENT_BURST_INTERVAL_MAX 22.0f
#define AMBIENT_BURST_BOMBS         28
#define AMBIENT_BURST_SPARKS        18
#define AMBIENT_BURST_EMBERS        40

/* Render tuning. */
#define GLOW_PULSE_RATE   3.0f     /* crater-glow pulse rate (radians/sec) */
#define FLOW_VISIBLE_MIN  0.05f    /* skip drawing flows dimmer than this  */

/* User intensity knob (+/-): geometric step, clamped to [MIN, MAX]. */
#define INTENSITY_STEP_UP    1.15f
#define INTENSITY_STEP_DOWN  0.85f
#define INTENSITY_MAX        4.0f
#define INTENSITY_MIN        0.20f

/* Frame loop: cap simulated dt so a long stall can't trigger a spiral
 * of death (one giant catch-up tick).  Milliseconds. */
#define DT_CAP_MS         100

/*
 * Theme — one complete colour scheme, swapped live with t/T (it only
 * re-assigns the terminal colour pairs; no geometry is rebuilt).  Each
 * array is an ordered colour ramp, dark end first.  The renderer picks a
 * slot by a meaningful 0..1 amount — sky by how high the row is, lava by
 * how hot the particle is, plume by how dense the smoke is — so position in
 * the array means something, it's not arbitrary.  All colours sit in the
 * bright half of the xterm-256 cube so even the darkest tier stays visible
 * against the terminal background (project palette-brightness rule).
 */
typedef struct {
    const char *name;          /* label shown in the HUD, padded to fixed width */
    short       sky[4];        /* sky gradient, top of screen to horizon        */
    short       mountain[3];   /* mountain shading: shadow, mid, lit            */
    short       lava[6];       /* heat ramp: cooled/dim up to white-hot         */
    short       plume[4];      /* smoke ramp: dense core out to wispy edge      */
    bool        inverted;      /* true = dark ink on a white "paper" background */
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* DAY: cobalt sky → cyan, grey-rock mountain, classic orange lava,
     * grey ash plume.                                                  */
    { "DAY      ",
      {  24,  31,  39,  75 },
      { 240, 244, 248 },
      {  88, 124, 196, 208, 220, 226 },
      { 240, 244, 248, 252 }, false },

    /* DUSK: sunset gradient (deep magenta → orange → pale), dark
     * mountain silhouette, vivid lava (pops on dark), warm ash.        */
    { "DUSK     ",
      {  53,  88, 167, 215 },
      {  60,  66,  72 },
      { 124, 160, 196, 208, 220, 229 },
      { 235, 240, 244, 248 }, false },

    /* NIGHT: deep navy sky, dark mountain, hottest lava (white-hot
     * core), dim grey plume.                                           */
    { "NIGHT    ",
      {  17,  18,  24,  60 },
      {  60,  66,  72 },
      {  88, 124, 196, 220, 226, 231 },
      { 234, 238, 242, 246 }, false },

    /* MARS: pink-rust sky, rust mountain, white-hot lava, brown ash.   */
    { "MARS     ",
      { 124,  88, 130, 173 },
      { 130, 137, 173 },
      { 130, 166, 208, 220, 226, 231 },
      { 137, 144, 173, 180 }, false },

    /* ASHFALL: muted grey throughout, plume dominates.                 */
    { "ASHFALL  ",
      { 235, 240, 244, 247 },
      { 234, 238, 242 },
      {  88, 130, 166, 202, 208, 214 },
      { 248, 250, 252, 254 }, false },

    /* MONO: monochrome — white / light-grey throughout; depth comes from the
     * A_DIM / A_BOLD brightness attrs, not hue (silhouette study).          */
    { "MONO     ",
      { 244, 248, 250, 252 },
      { 248, 251, 254 },
      { 245, 248, 250, 252, 254, 231 },
      { 247, 250, 252, 254 }, false },
};

/* Glyph ramps. */
static const char LAVA_GLYPHS[6]   = { '.', ':', '+', '*', '#', '@' };
static const char EMBER_GLYPHS[6]  = { '.', ',', ':', ';', '+', '*' };
static const char ASH_GLYPHS[4]    = { '.', ',', ':', ';' };
static const char PLUME_GLYPHS[4]  = { ',', ':', '+', '*' };
static const char SPARK_GLYPHS[3]  = { '.', '*', '#' };
static const char MTN_GLYPH        = '#';
static const char MTN_RIDGE_GLYPH  = '@';   /* topmost mountain pixel  */

/* ── §2 LOGIC — pure math, RNG, fBm noise ── */

/* Clamp v into [lo, hi]: below lo → lo, above hi → hi, else unchanged. */
static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* Turn a 0..1 amount into a ramp slot 0..n_slots-1.  The tiny -0.001 keeps
 * an input of exactly 1.0 from landing one slot past the end.  Used to map
 * heat / density / brightness onto a glyph or colour ramp. */
static inline int unit_to_slot(float t01, int n_slots)
{
    int s = (int)(t01 * ((float)n_slots - 0.001f));
    if (s < 0)         s = 0;
    if (s >= n_slots)  s = n_slots - 1;
    return s;
}

/*
 * lcg_next — one step of a simple random-number generator: multiply the
 * current state by a constant, add another, and let it wrap around at 32
 * bits.  These two constants are the well-known Numerical Recipes pair,
 * picked so the sequence runs through all 2^32 values before repeating.
 */
static inline uint32_t lcg_next(uint32_t *st)
{
    *st = *st * 1664525u + 1013904223u;
    return *st;
}

/*
 * lcg_unit — next random float in [0, 1).  This kind of generator has weak
 * low bits (the bottom bit just flips back and forth), so we throw the
 * bottom 8 away, keep the top 24, and scale that down into [0, 1).
 */
static inline float lcg_unit(uint32_t *st)
{
    return (float)(lcg_next(st) >> 8) / (float)(1u << 24);
}

/* lcg_range — random float somewhere in [lo, hi). */
static inline float lcg_range(uint32_t *st, float lo, float hi)
{
    return lo + (hi - lo) * lcg_unit(st);
}

static uint32_t g_rng = 0xCAFEBABEu;

/*
 * hash2d — turn a grid point (x,z) and a seed into a scrambled 32-bit
 * number that is the same every time for the same inputs.  Each input is
 * multiplied by its own big odd constant so they land in different parts of
 * the number, then the last two lines stir the bits hard so that two
 * neighbouring cells come out looking totally unrelated.  This is what lets
 * the noise below look random while still being repeatable.
 */
static inline uint32_t hash2d(int x, int z, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)z * 668265263u
               + seed        * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

/* hash_unit — hash2d squeezed into [0,1), same bit trick as lcg_unit. */
static inline float hash_unit(int x, int z, uint32_t seed)
{
    return (float)(hash2d(x, z, seed) >> 8) / (float)(1u << 24);
}

/*
 * smoothstep01 — an ease curve that maps 0..1 to 0..1 but flattens out at
 * both ends instead of running in a straight line.  Used to soften how the
 * noise below blends between cells, which hides the grid-like creases a
 * plain straight blend would leave behind.
 */
static inline float smoothstep01(float t) { return t * t * (3.0f - 2.0f * t); }

/*
 * vnoise2d — smooth random-looking value in [0,1] for any point.  The idea:
 * lay an invisible grid over the plane, give each grid corner a random
 * value, then for a point inside a cell, blend its four corner values
 * together.  The blend uses the eased smoothstep weights so the result
 * flows smoothly from one cell to the next instead of showing seams.
 */
static float vnoise2d(float x, float z, uint32_t seed)
{
    int   xi = (int)floorf(x), zi = (int)floorf(z);
    float fx = x - (float)xi,   fz = z - (float)zi;
    float v00 = hash_unit(xi,     zi,     seed);
    float v10 = hash_unit(xi + 1, zi,     seed);
    float v01 = hash_unit(xi,     zi + 1, seed);
    float v11 = hash_unit(xi + 1, zi + 1, seed);
    float sx  = smoothstep01(fx);
    float sz  = smoothstep01(fz);
    float a   = v00 * (1.0f - sx) + v10 * sx;
    float b   = v01 * (1.0f - sx) + v11 * sx;
    return a * (1.0f - sz) + b * sz;
}

/*
 * fbm2d — layered noise that adds detail at several sizes at once, the way
 * real clouds and rock have both big shapes and fine texture.  It sums a
 * few passes of the noise above: each pass is twice as fine and counts half
 * as much as the one before.  Three passes is plenty here.  Each pass uses
 * a different seed so the layers don't line up and reinforce, and the final
 * divide keeps the result in 0..1.  (Mandelbrot 1982; Perlin 1985.)
 */
static float fbm2d(float x, float z, uint32_t seed)
{
    float h = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
    for (int i = 0; i < 3; i++) {
        h    += amp * vnoise2d(x * freq, z * freq, seed + (uint32_t)i * 17u);
        norm += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return h / norm;
}

/* fbm1d — 1-D version of the noise above, for the mountain's surface. */
static float fbm1d(float x, uint32_t seed)
{
    return fbm2d(x, 0.0f, seed);
}

/* ── §3 PERFORMANCE — monotonic clock + sleep ── */

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

/* ── §4 RENDER-SETUP — colour-pair / theme palette configuration ── */

/* Load one theme's colours into the terminal's colour-pair table.  Called
 * at startup and whenever t/T switches themes.  Falls back to basic 8
 * colours when the terminal can't do 256. */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &themes[idx];
    short bg = t->inverted ? 231 : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_SKY_BASE + i), t->sky[i], bg);
        init_pair(PAIR_MOUNTAIN, t->mountain[1], bg);   /* one dominant tone */
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_LAVA_BASE + i), t->lava[i], bg);
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_PLUME_BASE + i), t->plume[i], bg);
        init_pair(PAIR_PAPER, 16, t->inverted ? 231 : -1);
    } else {
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_SKY_BASE + i), COLOR_BLUE,    -1);
        init_pair(PAIR_MOUNTAIN, COLOR_WHITE, -1);
        static const short fb_lava[6] = {
            COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
            COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < 6; i++)
            init_pair((short)(PAIR_LAVA_BASE + i),
                      t->inverted ? COLOR_BLACK : fb_lava[i],
                      t->inverted ? COLOR_WHITE : (short)-1);
        for (int i = 0; i < 4; i++)
            init_pair((short)(PAIR_PLUME_BASE + i),
                      t->inverted ? COLOR_BLACK : COLOR_WHITE,
                      t->inverted ? COLOR_WHITE : (short)-1);
        init_pair(PAIR_PAPER, COLOR_BLACK, COLOR_WHITE);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── §5 SIMULATION — state + advance ── */

/* --- mountain: heightmap silhouette + crater + lava-flow streams ------ */

/*
 * Mountain — the volcano's fixed shape, built once per seed and only read
 * after that.  Since we view it from the side, we only need to know, for
 * each screen column, the top row where rock starts — that one number per
 * column (silhouette_y) IS the whole outline.  The shape is a cone with
 * fractal noise roughening its surface so it looks like eroded rock, and a
 * small bowl carved at the top for the crater.
 *
 * The two lava flows are traced once down the slopes and stored as their
 * cells, not recomputed each frame, because the terrain never changes — the
 * renderer just recolours those cells every frame.
 */
typedef struct {
    int   silhouette_y[MTN_MAX_W]; /* top rock row in each column (smaller = taller) */
    int   crater_x;                /* column of the crater centre                    */
    int   crater_y;                /* row of the crater floor; particles spawn here  */
    /* The two lava flows as lists of cells walked out from the crater.
     * flow_*_n is how many cells each list actually holds. */
    int   flow_l_x[MTN_MAX_W];     /* left flow: column of each cell                 */
    int   flow_l_y[MTN_MAX_W];     /* left flow: row of each cell                    */
    int   flow_l_n;                /* number of cells in the left flow              */
    int   flow_r_x[MTN_MAX_W];     /* right flow: column of each cell                */
    int   flow_r_y[MTN_MAX_W];     /* right flow: row of each cell                   */
    int   flow_r_n;                /* number of cells in the right flow             */
} Mountain;

/*
 * pick_crater_column — choose which column the crater sits in: near the
 * middle, nudged left or right at random, but kept in the middle half so
 * the whole cone still fits on screen.
 */
static int pick_crater_column(int cols, uint32_t *seed)
{
    int crater_x = cols / 2
                 + (int)((lcg_unit(seed) - 0.5f) * (float)cols * CRATER_X_JITTER);
    if (crater_x < cols / 4)     crater_x = cols / 4;
    if (crater_x > 3 * cols / 4) crater_x = 3 * cols / 4;
    return crater_x;
}

/*
 * build_cone_silhouette — fill in the basic mountain outline: a triangle
 * that rises toward the crater, with fractal noise added so the slopes look
 * rough instead of perfectly straight.
 */
static void build_cone_silhouette(Mountain *m, int cols, int rows,
                                  int crater_x, int peak_y, uint32_t seed)
{
    for (int x = 0; x < cols; x++) {
        float dx   = (float)abs(x - crater_x);
        float cone = (float)peak_y + dx * MTN_SLOPE;
        float n    = fbm1d((float)x * MTN_NOISE_SCALE, seed) - 0.5f;
        cone += n * 2.0f * MTN_NOISE_AMP;
        if (cone < 0)        cone = 0;
        if (cone > rows - 2) cone = rows - 2;
        m->silhouette_y[x] = (int)(cone + 0.5f);
    }
}

/*
 * carve_crater — scoop a small rounded bowl out of the summit so the top
 * reads as a crater dip instead of a sharp point.
 */
static void carve_crater(Mountain *m, int cols, int rows, int crater_x)
{
    for (int x = crater_x - (int)CRATER_RADIUS;
             x <= crater_x + (int)CRATER_RADIUS; x++) {
        if (x < 0 || x >= cols) continue;
        float dxn = ((float)(x - crater_x)) / CRATER_RADIUS;
        if (dxn < -1.0f || dxn > 1.0f) continue;
        float depth = CRATER_DEPTH * 0.5f * (1.0f + cosf((float)M_PI * dxn));
        m->silhouette_y[x] += (int)depth;
        if (m->silhouette_y[x] > rows - 2) m->silhouette_y[x] = rows - 2;
    }
}

/*
 * trace_lava_flow — walk one lava river out from the crater rim, going left
 * (dir -1) or right (dir +1), dropping one cell per column just under the
 * surface.  Fills xs/ys and returns how many cells it laid.
 */
static int trace_lava_flow(const Mountain *m, int cols, int crater_x, int dir,
                           int max_len, int *xs, int *ys)
{
    int n = 0;
    for (int d = (int)CRATER_RADIUS + 1; d < max_len; d++) {
        int x = crater_x + dir * d;
        if (x < 0 || x >= cols) break;
        xs[n] = x;
        ys[n] = m->silhouette_y[x] + 1;
        n++;
    }
    return n;
}

/*
 * mountain_build — build the whole mountain in four steps: pick the crater
 * column, draw the rough cone outline, carve the crater bowl, then trace a
 * lava river down each slope.
 */
static void mountain_build(Mountain *m, int cols, int rows, uint32_t seed)
{
    if (cols > MTN_MAX_W) cols = MTN_MAX_W;

    int peak_y   = (int)((float)rows * MTN_PEAK_FRAC);
    int crater_x = pick_crater_column(cols, &seed);

    build_cone_silhouette(m, cols, rows, crater_x, peak_y, seed);
    carve_crater(m, cols, rows, crater_x);

    m->crater_x = crater_x;
    m->crater_y = m->silhouette_y[crater_x];

    /* Flows run a quarter of the way to the edge, capped at MTN_FLOW_MAX_LEN. */
    int max_len = (cols / 4 < MTN_FLOW_MAX_LEN) ? cols / 4 : MTN_FLOW_MAX_LEN;
    m->flow_l_n = trace_lava_flow(m, cols, crater_x, -1, max_len,
                                  m->flow_l_x, m->flow_l_y);
    m->flow_r_n = trace_lava_flow(m, cols, crater_x, +1, max_len,
                                  m->flow_r_x, m->flow_r_y);
}

/* --- particles: single pool, four types ------------------------------- */

/*
 * ParticleType — which of the four kinds of flying debris a particle is.
 * All four share one struct and one pool; the physics step looks at this
 * tag to decide how each kind moves and cools, so adding a new kind is just
 * a new case, not a new array.  What each one is:
 *   PT_BOMB  — a lava bomb: heavy, arcs up and falls, leaves a smoke trail.
 *   PT_EMBER — a glowing speck that floats up through the smoke and cools.
 *   PT_ASH   — a fine grey fleck that drifts a long time before fading.
 *   PT_SPARK — a tiny, very hot, very short-lived spark off the crater rim.
 */
typedef enum {
    PT_BOMB  = 0,
    PT_EMBER = 1,
    PT_ASH   = 2,
    PT_SPARK = 3,
} ParticleType;

/*
 * Particle — one slot in the fixed pool of debris (Reeves, 1983).  There's
 * no free list: `active` says whether a slot is in use, a new particle
 * grabs the first inactive slot, and a particle marks itself inactive when
 * it dies.  So PARTICLES_MAX is simply the most debris that can be on
 * screen at once.  Positions are in character cells, not pixels.
 */
typedef struct {
    bool         active;      /* slot in use? false = free to reuse             */
    ParticleType type;        /* which kind, picks the physics branch          */
    /* where it is and how fast it's moving (cells, cells/sec) */
    float        x, y;        /* position; y increases downward                 */
    float        vx, vy;      /* velocity; negative vy means moving up          */
    /* how long it lives (seconds) */
    float        age, life;   /* dies once age reaches life                     */
    /* appearance */
    float        temp;        /* heat, 0 cold to 1 white-hot; picks the colour  */
    /* bomb-only smoke trail: recent positions, newest first */
    float        tx[BOMB_TRAIL_LEN]; /* past x positions (just for the trail)   */
    float        ty[BOMB_TRAIL_LEN]; /* past y positions                        */
    int          trail_n;     /* how many trail samples are valid yet           */
} Particle;

static int particle_alloc(Particle *pool)
{
    for (int i = 0; i < PARTICLES_MAX; i++) {
        if (!pool[i].active) return i;
    }
    return -1;       /* pool full */
}

static void particles_clear(Particle *pool)
{
    memset(pool, 0, PARTICLES_MAX * sizeof *pool);
}

/*
 * particle_spawn_bomb — launch a lava bomb from the crater at a random
 * upward angle and speed.
 */
static void particle_spawn_bomb(Particle *pool, int crater_x, int crater_y,
                                float intensity)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_BOMB;
    p->x      = (float)crater_x + lcg_range(&g_rng, -BOMB_SPAWN_DX, BOMB_SPAWN_DX);
    p->y      = (float)crater_y + 0.5f;
    float angle_off = lcg_range(&g_rng, -BOMB_ANGLE, BOMB_ANGLE);
    float speed     = lcg_range(&g_rng, BOMB_SPEED_MIN, BOMB_SPEED_MAX) * intensity;
    p->vx     = sinf(angle_off) * speed;
    p->vy     = -cosf(angle_off) * speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, BOMB_LIFE_MIN, BOMB_LIFE_MAX);
    p->temp   = 1.0f;
    /* Seed the trail with the start position so it draws nothing stale. */
    for (int k = 0; k < BOMB_TRAIL_LEN; k++) {
        p->tx[k] = p->x;
        p->ty[k] = p->y;
    }
    p->trail_n = 0;
}

static void particle_spawn_ember(Particle *pool, int crater_x, int crater_y)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_EMBER;
    p->x      = (float)crater_x + lcg_range(&g_rng, -EMBER_SPAWN_DX, EMBER_SPAWN_DX);
    p->y      = (float)crater_y;
    float speed = lcg_range(&g_rng, EMBER_SPEED_MIN, EMBER_SPEED_MAX);
    p->vx     = lcg_range(&g_rng, -EMBER_SPAWN_VX, EMBER_SPAWN_VX);
    p->vy     = -speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, EMBER_LIFE_MIN, EMBER_LIFE_MAX);
    p->temp   = EMBER_SPAWN_TEMP;
    p->trail_n = 0;
}

static void particle_spawn_ash(Particle *pool, int crater_x, int crater_y)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_ASH;
    p->x      = (float)crater_x + lcg_range(&g_rng, -ASH_SPAWN_DX, ASH_SPAWN_DX);
    p->y      = (float)crater_y - lcg_range(&g_rng, 0.0f, ASH_SPAWN_LIFT);
    float speed = lcg_range(&g_rng, ASH_SPEED_MIN, ASH_SPEED_MAX);
    p->vx     = lcg_range(&g_rng, -ASH_SPAWN_VX, ASH_SPAWN_VX);
    p->vy     = -speed;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, ASH_LIFE_MIN, ASH_LIFE_MAX);
    p->temp   = ASH_SPAWN_TEMP;
    p->trail_n = 0;
}

static void particle_spawn_spark(Particle *pool, int crater_x, int crater_y)
{
    int idx = particle_alloc(pool);
    if (idx < 0) return;
    Particle *p = &pool[idx];
    p->active = true;
    p->type   = PT_SPARK;
    p->x      = (float)crater_x + lcg_range(&g_rng, -SPARK_SPAWN_DX, SPARK_SPAWN_DX);
    p->y      = (float)crater_y;
    float angle = lcg_range(&g_rng, -SPARK_SPAWN_ANGLE, SPARK_SPAWN_ANGLE);
    float speed = lcg_range(&g_rng, SPARK_SPEED_MIN, SPARK_SPEED_MAX);
    p->vx     = sinf(angle) * speed;
    p->vy     = -cosf(angle) * speed * SPARK_VY_FACTOR;
    p->age    = 0;
    p->life   = lcg_range(&g_rng, SPARK_LIFE_MIN, SPARK_LIFE_MAX);
    p->temp   = 1.0f;
    p->trail_n = 0;
}

/* Record the bomb's current spot at the front of its trail history,
 * pushing the older spots back one. */
static void bomb_trail_push(Particle *p)
{
    for (int k = BOMB_TRAIL_LEN - 1; k > 0; k--) {
        p->tx[k] = p->tx[k - 1];
        p->ty[k] = p->ty[k - 1];
    }
    p->tx[0] = p->x;
    p->ty[0] = p->y;
    p->trail_n = BOMB_TRAIL_LEN;
}

/* Move a bomb one step: pulled down by gravity, slowed a little by air,
 * cooling slowly, and adding its spot to the smoke trail. */
static void bomb_step(Particle *p, float dt, float drag)
{
    bomb_trail_push(p);
    p->vy += GRAVITY * dt;
    p->vx *= drag;
    p->vy *= drag;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_BOMB * dt;
}

/* Move an ember: floats upward, nudged sideways by wind and random
 * turbulence, and cools fairly fast. */
static void ember_step(Particle *p, float dt)
{
    p->vy -= EMBER_BUOYANCY * dt;
    p->vx += lcg_range(&g_rng, -EMBER_TURBULENCE, EMBER_TURBULENCE) * dt;
    p->vx += PLUME_WIND * EMBER_WIND_FACTOR * dt;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_EMBER * dt;
}

/* Move ash: drifts up only weakly, blows sideways more in the wind, and
 * barely cools, so it lingers a long time. */
static void ash_step(Particle *p, float dt)
{
    p->vy -= ASH_BUOYANCY * dt;
    p->vx += lcg_range(&g_rng, -ASH_TURBULENCE, ASH_TURBULENCE) * dt;
    p->vx += PLUME_WIND * ASH_WIND_FACTOR * dt;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_ASH * dt;
}

/* Move a spark: fast, only lightly pulled down, and cools almost at once. */
static void spark_step(Particle *p, float dt)
{
    p->vy += GRAVITY * SPARK_GRAVITY_FACTOR * dt;
    p->x  += p->vx * dt;
    p->y  += p->vy * dt;
    p->temp -= COOL_SPARK * dt;
}

/* A particle is done when it runs out of life, cools to black, or wanders
 * off screen (with a little margin so it doesn't vanish right at the edge). */
static bool particle_is_dead(const Particle *p, int rows, int cols)
{
    if (p->age >= p->life)                            return true;
    if (p->temp <= 0)                                 return true;
    if (p->y < -PARTICLE_CULL_MARGIN_Y)               return true;
    if (p->y > (float)rows + PARTICLE_CULL_MARGIN_Y)  return true;
    if (p->x < -PARTICLE_CULL_MARGIN_X ||
        p->x > (float)cols + PARTICLE_CULL_MARGIN_X)  return true;
    return false;
}

/*
 * particles_tick — step every live particle once: move it by its own
 * rules, age it, and free its slot if it just died.
 */
static void particles_tick(Particle *pool, float dt, int rows, int cols)
{
    float bomb_drag = 1.0f - BOMB_DRAG * dt;
    if (bomb_drag < 0) bomb_drag = 0;

    for (int i = 0; i < PARTICLES_MAX; i++) {
        Particle *p = &pool[i];
        if (!p->active) continue;

        switch (p->type) {
        case PT_BOMB:  bomb_step (p, dt, bomb_drag); break;
        case PT_EMBER: ember_step(p, dt);            break;
        case PT_ASH:   ash_step  (p, dt);            break;
        case PT_SPARK: spark_step(p, dt);            break;
        }

        p->age += dt;
        if (particle_is_dead(p, rows, cols)) p->active = false;
    }
}

/* --- scene state + the per-tick combiner ------------------------------ */

/*
 * Scene — everything about the running volcano in one place: the mountain,
 * the live particles, the user's settings, the clock, and the timers that
 * decide when to spawn things.  The per-frame step works on this; the draw
 * code only reads it.
 */
typedef struct {
    /* the things being simulated */
    Mountain mountain;                  /* the fixed mountain shape + lava paths    */
    Particle ejecta[PARTICLES_MAX];     /* the pool of flying debris                */

    /* user-controlled settings */
    int    current_pattern;             /* which eruption style (index into patterns[]) */
    float  intensity;                   /* overall how-much knob on all spawn rates */
    bool   paused;                      /* true freezes spawning and physics        */

    /* clock and spawn timing (seconds) */
    float  time;                        /* seconds of simulation elapsed            */
    float  bomb_accum, ember_accum, ash_accum, spark_accum; /* fractional spawns waiting to fire */
    float  next_burst_at;               /* time the next Vulcanian volley fires     */
    float  next_ambient_at;             /* time the next ambient surge fires        */
    float  last_ambient_age;            /* seconds since the last ambient surge     */

    /* world generation */
    uint32_t seed;                      /* seed for the mountain shape and spawns   */

    /* terminal size + colours */
    int    cols, rows;                  /* current terminal size in cells           */
    int    current_theme;               /* which colour theme (index into themes[]) */
} Scene;

/*
 * spawn_continuous — the steady stream of debris.  Each type has a counter
 * that fills up at its spawn rate; every time a counter passes 1, one
 * particle of that type is born.  This runs every frame.
 */
static void spawn_continuous(Scene *s, const EruptionPattern *pp, float dt)
{
    Particle *pool     = s->ejecta;
    int       crater_x = s->mountain.crater_x;
    int       crater_y = s->mountain.crater_y;
    float     intensity = s->intensity;

    s->bomb_accum  += dt * pp->bomb_per_sec  * intensity;
    s->ember_accum += dt * pp->ember_per_sec * intensity;
    s->ash_accum   += dt * pp->ash_per_sec   * intensity;
    s->spark_accum += dt * pp->spark_per_sec * intensity;

    while (s->bomb_accum >= 1.0f) {
        particle_spawn_bomb(pool, crater_x, crater_y, intensity);
        s->bomb_accum -= 1.0f;
    }
    while (s->ember_accum >= 1.0f) {
        particle_spawn_ember(pool, crater_x, crater_y);
        s->ember_accum -= 1.0f;
    }
    while (s->ash_accum >= 1.0f) {
        particle_spawn_ash(pool, crater_x, crater_y);
        s->ash_accum -= 1.0f;
    }
    while (s->spark_accum >= 1.0f) {
        particle_spawn_spark(pool, crater_x, crater_y);
        s->spark_accum -= 1.0f;
    }
}

/*
 * maybe_vulcanian_burst — only for the Vulcanian style: when its timer is
 * up, fire a big volley of bombs and set the timer for the next one.
 */
static void maybe_vulcanian_burst(Scene *s, const EruptionPattern *pp)
{
    if (pp->vulcanian_bursts && s->time >= s->next_burst_at) {
        for (int i = 0; i < VULCAN_BURST_BOMBS; i++)
            particle_spawn_bomb(s->ejecta, s->mountain.crater_x,
                                s->mountain.crater_y,
                                s->intensity * VULCAN_BURST_BOOST);
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
    }
}

/*
 * maybe_ambient_burst — fires for every style at a random 8-22 sec spacing:
 * a mixed surge of bombs, sparks and embers so even calm styles get the
 * occasional dramatic moment.  Otherwise it just bumps the "time since last
 * surge" counter the HUD shows.
 */
static void maybe_ambient_burst(Scene *s, float dt)
{
    if (s->time >= s->next_ambient_at) {
        Particle *pool = s->ejecta;
        int cx = s->mountain.crater_x, cy = s->mountain.crater_y;
        for (int i = 0; i < AMBIENT_BURST_BOMBS; i++)
            particle_spawn_bomb(pool, cx, cy, s->intensity * AMBIENT_BURST_BOOST);
        for (int i = 0; i < AMBIENT_BURST_SPARKS; i++)
            particle_spawn_spark(pool, cx, cy);
        for (int i = 0; i < AMBIENT_BURST_EMBERS; i++)
            particle_spawn_ember(pool, cx, cy);
        s->next_ambient_at = s->time
                           + lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                                AMBIENT_BURST_INTERVAL_MAX);
        s->last_ambient_age = 0.0f;
    } else {
        s->last_ambient_age += dt;
    }
}

/*
 * scene_tick — advance the whole simulation by dt seconds, and the only
 * place that does: skip if paused, move the clock, spawn the steady stream,
 * check the two burst timers, then move every particle.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt;

    const EruptionPattern *pp = &patterns[s->current_pattern];

    spawn_continuous(s, pp, dt);
    maybe_vulcanian_burst(s, pp);
    maybe_ambient_burst(s, dt);
    particles_tick(s->ejecta, dt, s->rows, s->cols);
}

/* ── §6 RENDER — state to screen (read-only) ── */

/*
 * draw_bomb_trail — paint one bomb's fading smoke tail.  Skips the newest
 * spot (the bomb itself is drawn later); each older spot is dimmer and uses
 * a cooler colour so the tail trails off.
 */
static void draw_bomb_trail(const Particle *p, int rows_eff, int cols, bool inverted)
{
    for (int k = 1; k < p->trail_n; k++) {
        int tx = (int)(p->tx[k] + 0.5f);
        int ty = (int)(p->ty[k] + 0.5f);
        if (tx < 0 || tx >= cols)     continue;
        if (ty < 0 || ty >= rows_eff) continue;
        int slot = unit_to_slot(p->temp, 6) - k;   /* cooler the further back it is */
        if (slot < 0) slot = 0;
        if (slot > 5) slot = 5;
        char glyph = (k == 1) ? '*' : (k == 2 ? '+' : '.');
        attr_t a = inverted ? A_NORMAL : A_DIM;
        attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | a);
        mvaddch(ty, tx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | a);
    }
}

/*
 * draw_particle_head — draw one particle at (ix,iy), picking its character
 * and colour from its kind and how hot it is.  Each kind has its own ramp;
 * hotter looks brighter.
 */
static void draw_particle_head(const Particle *p, int ix, int iy, bool inverted)
{
    float temp = p->temp;
    if (temp < 0) temp = 0;
    if (temp > 1) temp = 1;

    char   glyph;
    int    pair;
    attr_t attr;

    switch (p->type) {
    case PT_BOMB: {
        int slot = unit_to_slot(temp, 6);
        glyph = LAVA_GLYPHS[slot];
        pair  = PAIR_LAVA_BASE + slot;
        attr  = (slot >= 4 && !inverted) ? A_BOLD : A_NORMAL;
        break;
    }
    case PT_EMBER: {
        int slot = unit_to_slot(temp, 6);
        glyph = EMBER_GLYPHS[slot];
        pair  = PAIR_LAVA_BASE + slot;
        attr  = (slot <= 1 && !inverted) ? A_DIM
              : (slot >= 5 && !inverted) ? A_BOLD
              :                             A_NORMAL;
        break;
    }
    case PT_ASH: {
        int slot = unit_to_slot(temp, 4);
        glyph = ASH_GLYPHS[slot];
        pair  = PAIR_PLUME_BASE + slot;
        attr  = inverted ? A_NORMAL : A_DIM;
        break;
    }
    case PT_SPARK:
    default: {
        int slot = unit_to_slot(temp, 3);
        glyph = SPARK_GLYPHS[slot];
        pair  = PAIR_LAVA_BASE + 4 + (slot >= 1 ? 1 : 0);
        attr  = inverted ? A_NORMAL : A_BOLD;
        break;
    }
    }
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(iy, ix, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
}

/*
 * particles_draw — draw the bomb trails first, then every particle on top,
 * so the bombs sit over their own tails.
 */
static void particles_draw(const Particle *pool, int rows, int cols, bool inverted)
{
    int rows_eff = rows - 1;       /* keep the bottom row free for the HUD */

    /* trails first, under everything else */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        const Particle *p = &pool[i];
        if (p->active && p->type == PT_BOMB)
            draw_bomb_trail(p, rows_eff, cols, inverted);
    }

    /* then every live particle, skipping any off screen */
    for (int i = 0; i < PARTICLES_MAX; i++) {
        const Particle *p = &pool[i];
        if (!p->active) continue;
        int ix = (int)(p->x + 0.5f);
        int iy = (int)(p->y + 0.5f);
        if (ix < 0 || ix >= cols)     continue;
        if (iy < 0 || iy >= rows_eff) continue;
        draw_particle_head(p, ix, iy, inverted);
    }
}

/* How wide the smoke column is at height h above the crater (it widens as
 * it rises). */
static inline float plume_half_width(float h)
{
    return PLUME_BASE_W + h * PLUME_SPREAD;
}

/* Fade across the column: full strength in the middle, fading to zero at
 * the edge, and below zero past it (callers treat <= 0 as outside). */
static inline float plume_cone_falloff(float dx, float half_w)
{
    return 1.0f - (dx * dx) / (half_w * half_w);
}

/*
 * plume_density — how thick the smoke is at one cell.  It samples the noise
 * field, but shifts where it samples over time: sideways with the wind and
 * upward as the column rises, so the smoke looks like it's flowing rather
 * than sitting still.  Scaled by the style's plume strength and the edge
 * fade.
 */
static float plume_density(float dx, float h, float time,
                           float intensity, float falloff)
{
    float nx = dx * PLUME_NOISE_S - time * PLUME_WIND * PLUME_NOISE_S;
    float ny = h  * PLUME_NOISE_S - time * PLUME_RISE * PLUME_NOISE_S;
    return fbm2d(nx, ny, PLUME_NOISE_SEED) * intensity * falloff;
}

/*
 * plume_draw — for each cell inside the smoke column above the crater,
 * check how thick the smoke is there and, if it's thick enough, paint it.
 */
static void plume_draw(const Mountain *mtn, int rows, int cols,
                          float time, float intensity, bool inverted)
{
    int rows_eff = rows - 1;
    int crater_x = mtn->crater_x;
    int crater_y = mtn->crater_y;

    for (int row = 0; row < rows_eff; row++) {
        if (row > crater_y) break;     /* below the crater is inside the mountain */

        float h = (float)(crater_y - row);     /* how high above the crater */
        if (h < 0.5f) continue;
        float half_w = plume_half_width(h);

        for (int col = 0; col < cols; col++) {
            float dx = (float)(col - crater_x);
            if (fabsf(dx) > half_w) continue;             /* outside the column */
            if (row > mtn->silhouette_y[col]) continue;   /* hidden by the mountain */

            float falloff = plume_cone_falloff(dx, half_w);
            if (falloff <= 0) continue;

            float density = plume_density(dx, h, time, intensity, falloff);
            if (density < PLUME_THRESHOLD) continue;

            /* turn the thickness into a colour/character slot */
            float t_n = (density - PLUME_THRESHOLD) / (1.0f - PLUME_THRESHOLD);
            if (t_n > 1.0f) t_n = 1.0f;
            int slot = unit_to_slot(t_n, 4);

            char glyph = PLUME_GLYPHS[slot];
            int  pair  = PAIR_PLUME_BASE + slot;
            attr_t attr = inverted ? A_NORMAL
                        : (slot <= 1) ? A_DIM : A_NORMAL;
            attron(COLOR_PAIR(pair) | attr);
            mvaddch(row, col, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* The lowest rock row anywhere — used as the horizon for the sky gradient. */
static int silhouette_max(const Mountain *mtn, int cols)
{
    int m = 0;
    for (int x = 0; x < cols; x++)
        if (mtn->silhouette_y[x] > m) m = mtn->silhouette_y[x];
    return m < 1 ? 1 : m;
}

/*
 * scene_draw_sky — fill the sky above the mountain with a top-to-bottom
 * colour gradient: deepest at the top, palest near the horizon.
 */
static void scene_draw_sky(const Mountain *mtn, int cols, int rows, bool inverted)
{
    int rows_eff = rows - 1;
    int horizon_y = silhouette_max(mtn, cols);   /* the gradient spans down to here */

    for (int row = 0; row < rows_eff; row++) {
        if (row >= horizon_y) break;
        int slot = (row * 4) / horizon_y;
        if (slot < 0) slot = 0;
        if (slot > 3) slot = 3;

        char glyph = ' ';   /* sky is just the background colour */
        attr_t attr = (inverted) ? A_NORMAL
                    : (slot == 0) ? A_DIM : A_NORMAL;
        int pair = PAIR_SKY_BASE + slot;
        attron(COLOR_PAIR(pair) | attr);
        for (int col = 0; col < cols; col++) {
            if (row > mtn->silhouette_y[col]) continue;     /* that cell is rock */
            mvaddch(row, col, (chtype)(unsigned char)glyph);
        }
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/*
 * scene_draw_mountain — fill each column with rock from its top edge down,
 * using a different character on the very top row to mark the ridge.
 */
static void scene_draw_mountain(const Mountain *mtn, int cols, int rows, bool inverted)
{
    int rows_eff = rows - 1;
    attr_t attr = inverted ? A_NORMAL : A_BOLD;
    attron(COLOR_PAIR(PAIR_MOUNTAIN) | attr);
    for (int col = 0; col < cols; col++) {
        int top = mtn->silhouette_y[col];
        if (top < 0)            top = 0;
        if (top >= rows_eff)    continue;
        for (int row = top; row < rows_eff; row++) {
            char ch = (row == top) ? MTN_RIDGE_GLYPH : MTN_GLYPH;
            mvaddch(row, col, (chtype)(unsigned char)ch);
        }
    }
    attroff(COLOR_PAIR(PAIR_MOUNTAIN) | attr);
}

/*
 * scene_draw_lava_flows — light up the two lava rivers, hottest near the
 * crater and dimming as they reach down the slope.
 */
static void scene_draw_lava_flows(const Mountain *mtn, int cols, int rows,
                                  float lava_flow_amount, bool inverted)
{
    int rows_eff = rows - 1;
    if (lava_flow_amount < FLOW_VISIBLE_MIN) return;

    for (int side = 0; side < 2; side++) {
        const int *xs = (side == 0) ? mtn->flow_l_x : mtn->flow_r_x;
        const int *ys = (side == 0) ? mtn->flow_l_y : mtn->flow_r_y;
        int n          = (side == 0) ? mtn->flow_l_n : mtn->flow_r_n;

        for (int i = 0; i < n; i++) {
            float frac   = (float)i / (float)(n > 1 ? n - 1 : 1);
            float bright = (1.0f - frac) * lava_flow_amount;
            int   slot   = 5 - (int)(bright * 6.0f);
            if (slot < 0) slot = 0;
            if (slot > 5) slot = 5;

            int  yy = ys[i];
            int  xx = xs[i];
            if (yy < 0 || yy >= rows_eff) continue;
            if (xx < 0 || xx >= cols)     continue;

            char glyph = LAVA_GLYPHS[slot];
            attr_t attr = (slot >= 4 && !inverted) ? A_BOLD : A_NORMAL;
            attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
            mvaddch(yy, xx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
        }
    }
}

/*
 * scene_draw_crater_glow — a few cells in the crater that pulse with hot
 * colours, so the eye has a glowing centre to anchor on.
 */
static void scene_draw_crater_glow(const Mountain *mtn, int cols, int rows,
                                   float time, bool inverted)
{
    int rows_eff = rows - 1;
    int cx = mtn->crater_x;
    int cy = mtn->crater_y;

    for (int dx = -(int)CRATER_RADIUS + 1;
             dx <= (int)CRATER_RADIUS - 1; dx++) {
        int x = cx + dx;
        if (x < 0 || x >= cols) continue;
        int y = cy - 1;            /* one row above the crater floor */
        if (y < 0 || y >= rows_eff) continue;

        float pulse = 0.5f + 0.5f * sinf(time * GLOW_PULSE_RATE + (float)dx * 0.5f);
        int slot = 4 + (int)(pulse * 1.5f);     /* flicker between the two hottest colours */
        if (slot < 4) slot = 4;
        if (slot > 5) slot = 5;
        char glyph = LAVA_GLYPHS[slot];
        attr_t attr = inverted ? A_NORMAL : A_BOLD;
        attron(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
        mvaddch(y, x, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(PAIR_LAVA_BASE + slot) | attr);
    }
}

/* Count how many particles are live, for the HUD readout. */
static int particles_active_count(const Particle *pool)
{
    int n = 0;
    for (int i = 0; i < PARTICLES_MAX; i++)
        if (pool[i].active) n++;
    return n;
}

static void scene_render(const Scene *s)
{
    bool                   inverted = themes[s->current_theme].inverted;
    const EruptionPattern *style    = &patterns[s->current_pattern];
    int                    rows_eff = s->rows - 1;

    /* Paper themes: fill the screen white first, then draw dark ink on top. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_PAPER));
        for (int row = 0; row < rows_eff; row++)
            for (int col = 0; col < s->cols; col++)
                mvaddch(row, col, ' ');
        attroff(COLOR_PAIR(PAIR_PAPER));
    }

    scene_draw_sky        (&s->mountain, s->cols, s->rows, inverted);
    scene_draw_mountain   (&s->mountain, s->cols, s->rows, inverted);
    scene_draw_lava_flows (&s->mountain, s->cols, s->rows,
                           style->lava_flow_amount, inverted);
    scene_draw_crater_glow(&s->mountain, s->cols, s->rows, s->time, inverted);

    plume_draw    (&s->mountain, s->rows, s->cols, s->time,
                   style->plume_intensity, inverted);
    particles_draw(s->ejecta, s->rows, s->cols, inverted);
}

/* ── §7 EVENTS — init / resize / reset (rebuild the world) ── */

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_pattern = PAT_STROMBOLIAN;
    s->current_theme   = 0;
    s->cols            = cols;
    s->rows            = rows;
    s->seed            = (uint32_t)clock_ns();
    s->time            = 0.0f;
    s->intensity       = 1.0f;
    s->next_burst_at   = lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                            VULCAN_BURST_INTERVAL_MAX);
    s->next_ambient_at = lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                            AMBIENT_BURST_INTERVAL_MAX);
    s->last_ambient_age = 0.0f;
    g_rng = s->seed ^ 0xBEEFu;

    mountain_build(&s->mountain, cols, rows, s->seed);
    particles_clear(s->ejecta);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
    mountain_build(&s->mountain, cols, rows, s->seed);
    particles_clear(s->ejecta);
}

static void scene_reset(Scene *s)
{
    s->seed = (uint32_t)clock_ns() ^ 0xA5A5A5A5u;
    g_rng   = s->seed ^ 0xBEEFu;
    s->time = 0.0f;
    s->next_burst_at = lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                          VULCAN_BURST_INTERVAL_MAX);
    s->next_ambient_at = lcg_range(&g_rng, AMBIENT_BURST_INTERVAL_MIN,
                                            AMBIENT_BURST_INTERVAL_MAX);
    s->last_ambient_age = 0.0f;
    mountain_build(&s->mountain, s->cols, s->rows, s->seed);
    particles_clear(s->ejecta);
}

/* ── §8 SCREEN + APP — ncurses I/O, HUD, input, main loop + frame cap ── */

/*
 * Screen — the terminal's current size in cells.  This is the real, current
 * size (re-read at startup and after every resize); the Scene keeps its own
 * copy, but this one is what triggers rebuilding the mountain on a resize.
 */
typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) { (void)sc; endwin(); }
static void screen_resize_curses(Screen *sc)
{
    endwin();
    refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_render(s);

    /* top row: status line in yellow, clipped so it never wraps */
    char buf[200];
    snprintf(buf, sizeof buf,
             " VOLCANO   %s   pattern:%s   theme:%s   particles:%4d   "
             "intensity:%4.2f   %5.1f fps  %3d Hz ",
             s->paused ? "PAUSED " : "ERUPT  ",
             patterns[s->current_pattern].name,
             themes[s->current_theme].name,
             particles_active_count(s->ejecta),
             (double)s->intensity,
             fps, sim_fps);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(0, x, ' ');
    mvprintw(0, 0, "%.*s", sc->cols, buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* bottom row: the key hints in cyan */
    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%.*s", sc->cols,
             " spc:pause  r:reseed  n/N:pat  t/T:theme  +/-:int  [/]:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/*
 * App — the whole program in one struct.  There's a single global one so
 * the signal handlers can reach the two flags below.  Those two flags are
 * volatile sig_atomic_t because a signal handler can change them at any
 * moment: volatile stops the main loop from caching a stale value, and
 * sig_atomic_t guarantees reading or writing them happens in one piece.
 */
typedef struct {
    Scene                 scene;       /* the whole simulation                    */
    Screen                screen;      /* the terminal it draws to                */
    int                   sim_fps;     /* target frames per second ([ and ] keys) */
    volatile sig_atomic_t running;     /* cleared by a quit signal to end the loop */
    volatile sig_atomic_t need_resize; /* set by a resize signal, handled next frame */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize_curses(&app->screen);
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reset(s);                               break;

    case 'n':
        s->current_pattern = (s->current_pattern + 1) % N_PATTERNS;
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
        break;
    case 'N':
        s->current_pattern = (s->current_pattern + N_PATTERNS - 1) % N_PATTERNS;
        s->next_burst_at = s->time
                         + lcg_range(&g_rng, VULCAN_BURST_INTERVAL_MIN,
                                              VULCAN_BURST_INTERVAL_MAX);
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case '=': case '+':
        s->intensity *= INTENSITY_STEP_UP;
        if (s->intensity > INTENSITY_MAX) s->intensity = INTENSITY_MAX;
        break;
    case '-':
        s->intensity *= INTENSITY_STEP_DOWN;
        if (s->intensity < INTENSITY_MIN) s->intensity = INTENSITY_MIN;
        break;

    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
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
    scene_init(&app->scene, app->screen.cols, app->screen.rows);

    int64_t frame_time  = clock_ns();
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* 1. apply a pending terminal resize before measuring time. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
        }

        /* 2. time since the last frame, capped so a long stall can't cause one huge catch-up step. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;

        /* 3. advance the simulation by that timestep. */
        float dt_sec = (float)dt / (float)NS_PER_SEC;
        scene_tick(&app->scene, dt_sec);

        /* 4. rolling fps measurement (refresh every FPS_UPDATE_MS). */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. frame cap — sleep out the remainder of this tick's budget. */
        int64_t target_ns = TICK_NS(app->sim_fps);
        int64_t elapsed   = clock_ns() - frame_time + dt;
        clock_sleep_ns(target_ns - elapsed);

        /* 6. render the frame + HUD. */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* 7. drain one input event. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
