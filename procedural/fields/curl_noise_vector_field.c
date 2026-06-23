/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * curl_noise_vector_field.c — five ways to look at a swirling flow field.
 *
 * We build a smooth flow that swirls into whirlpools but never lets
 * particles pile up or empty out, then show the same field five ways:
 * flowing particles, arrows, a heightmap, a brightness map, and a
 * warped variant. The trick (rotate the slope of a noise field by 90
 * degrees so flow circles peaks instead of climbing them) is from
 * Bridson, Houriham & Nordenstam, "Curl-Noise for Procedural Fluid
 * Flow", SIGGRAPH 2007.
 *
 * Sister files: perin_noise_flow_showcase.c (the same noise used as a
 * slope, so particles DO pile up — the contrast case), and
 * domain_warped_noise_iq_style.c (the warping trick the WARPED view uses).
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

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* The dot pool, used by the PARTICLES and WARPED views. */
    MAX_PARTICLES     = 1024,
    N_PARTICLES_DEF   =  256,

    /* How long a dot lives before respawning, in ticks (picked at random
     * in this range so they don't all vanish at once). */
    AGE_MIN_TICKS     =  60,
    AGE_MAX_TICKS     = 360,

    /* How fast the dots move, in cells per second. */
    SPEED_MIN         =   1,
    SPEED_DEF         =   8,
    SPEED_MAX         =  64,

    /* The VECTOR view draws one arrow every few cells, not on every
     * cell. 4 wide by 2 tall because terminal characters are about
     * twice as tall as they are wide, so this looks roughly square. */
    VECTOR_LATTICE_X  =   4,
    VECTOR_LATTICE_Y  =   2,

    /* Color slots. The first two are the standard HUD colours. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* this plus the next 3 are the 4 ramp colours */
    PAIR_FLASH        =   7,
    PAIR_SUPERNOVA    =   8,
};

/* How fast a particle's trail fades. Only the particle views fade;
 * the others repaint every cell from scratch each frame. */
#define TRAIL_GLOW_DECAY    0.6f
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f   /* below this a cell is treated as blank */

/* How zoomed-in the noise is, and how fast the field drifts over time. */
#define NOISE_SCALE         0.05f
#define FIELD_DRIFT         0.10f
#define DRIFT_MULT_MIN      1
#define DRIFT_MULT_DEF      1
#define DRIFT_MULT_MAX      32

/* How far apart we sample the field to measure which way it slopes.
 * Too small and float rounding swamps the answer; too big and we miss
 * local detail. 0.5 noise-units is the sweet spot. */
#define CURL_EPS            0.5f

/* How many layers of noise we stack for detail (each finer than the last). */
#define FBM_OCTAVES         3

/* How hard the WARPED view bends the field. */
#define WARP_AMOUNT         3.0f

/* Brightness cutoffs that pick which character a cell gets. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

typedef enum {
    PATTERN_PARTICLES = 0,
    PATTERN_VECTOR    = 1,
    PATTERN_POTENTIAL = 2,
    PATTERN_CURL_MAG  = 3,
    PATTERN_WARPED    = 4,
    N_PATTERNS        = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_PARTICLES: return "PARTICLES";
    case PATTERN_VECTOR:    return "VECTOR   ";
    case PATTERN_POTENTIAL: return "POTENTIAL";
    case PATTERN_CURL_MAG:  return "CURL_MAG ";
    case PATTERN_WARPED:    return "WARPED   ";
    default:                return "?        ";
    }
}

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* The rest of §1: every remaining tunable, named and gathered here so
 * one look lists all the knobs. */

/* The WARPED view samples the noise twice for its bending; these two
 * unequal offsets keep the two samples from moving in lockstep. Any
 * non-zero, unequal pair works. */
#define WARP_OFFSET_X            5.2f
#define WARP_OFFSET_Y            1.3f

/* If a particle's speed is below this, treat it as standing still and
 * don't try to scale it to unit length — that would be dividing by zero. */
#define VELOCITY_EPSILON         1e-6f

/* Brightness a particle stamps onto the cell it lands on (full, then fades). */
#define TRAIL_HIT_INTENSITY      1.0f

/* Brightness of the flash that fires on reset (then fades out). */
#define SUPERNOVA_FLASH_INIT     1.0f

/* Brightness tweaks before a cell is binned into a colour band.
 * The first stretches the field's strength to fill the bright range;
 * the other two shift the heightmap from its -1..1 range into 0..1. */
#define CURL_MAG_VISUAL_GAIN     1.5f
#define POTENTIAL_REMAP_MID      0.5f
#define POTENTIAL_REMAP_RANGE    0.5f

/* We sort each cell's brightness into one of 4 colour bands. The scale
 * is just under 4 so a brightness of exactly 1.0 still lands in band 3,
 * not a non-existent band 4. */
#define N_BANDS                  4
#define BAND_QUANTIZE_SCALE      3.999f

/* Picking an arrow from a direction: below STATIONARY it's a dot; the
 * DOMINANCE ratio decides whether one axis wins (straight arrow) or
 * the two are close (diagonal). */
#define ARROW_STATIONARY_THRESH  0.05f
#define ARROW_AXIS_DOMINANCE     2.0f

/* Lights up one cell out of (MASK+1) during the reset flash, so it
 * looks like scattered sparkles instead of a solid sheet. */
#define SUPERNOVA_SPARSE_MASK    3

/* HUD layout. The top 3 rows show info; the bottom row lists the keys;
 * the field fills the space between. */
#define HUD_TOP_ROWS             3
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1
#define HUD_PATTERN_FIELD_W      20
#define HUD_THEME_FIELD_W        17
#define HUD_PALETTE_LABEL_W      9
#define HUD_PALETTE_SWATCH_N     4

/* If one frame stalls for more than 100 ms (laptop slept, etc.),
 * cap the elapsed time so the sim doesn't frantically run hundreds of
 * catch-up ticks. Standard guard from Glenn Fiedler's "Fix Your Timestep". */
#define DT_MAX_NS                (100 * NS_PER_MS)
#define FRAME_CAP_FPS            60

/*
 * Theme — one colour scheme. Ten of these live in themes[], cycled
 * with t/T. A cell's brightness picks one of four ramp colours (dark
 * to bright); the flash colour is used for the reset sparkle. Holding
 * exactly four colours keeps each theme a clean dark-to-light ramp.
 *
 * The numbers are xterm-256 colour codes, not RGB. On a terminal with
 * fewer than 256 colours, theme_apply() falls back to a fixed 8-colour set.
 */
typedef struct {
    const char *name;     /* short label shown in the HUD                   */
    short       band[4];  /* the ramp: band[0] darkest, band[3] brightest   */
    short       flash;    /* colour of the reset sparkle                    */
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
/* §5  noise — Noise context + Perlin + fBm + potential + curl            */
/* ===================================================================== */

/*
 * Noise — the program's one source of randomness. It's a shuffled
 * list of the numbers 0..255 that Perlin noise uses to look up which
 * way the field tilts at each grid corner. Reshuffling it (the 'r'
 * key) gives a brand-new field; otherwise the same field stays put and
 * we animate it by slowly drifting through it (see SimState.field_time).
 *
 * This is classic Perlin gradient noise (Ken Perlin, "Improving
 * Noise", SIGGRAPH 2002).
 */
typedef struct {
    /* The shuffled list, but stored twice back-to-back (512 entries,
     * the second half copies the first). That lets the sampler look up
     * perm[X+1] without worrying about running off the end. A small
     * speed trick from Perlin's own reference code. */
    uint8_t perm[512];
} Noise;

/* Shuffle 0..255 into a random order, every order equally likely
 * (textbook Fisher-Yates). */
static void fisher_yates_shuffle_256(uint8_t base[256])
{
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    for (int i = 255; i > 0; i--) {
        int j = rand() % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
}

/* Copy the shuffled list into the doubled buffer so the second half
 * repeats the first. See the Noise struct for why. */
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
    fisher_yates_shuffle_256       (base);
    mirror_perm_for_double_lookup  (n->perm, base);
}

/* An S-shaped easing curve. Bends a 0..1 value so the noise blends
 * smoothly across cell edges instead of looking faceted. (Perlin 2002.) */
static inline float fade(float t)
{
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

static inline float lerp_f(float a, float b, float t) { return a + t * (b - a); }

/* Picks one of 8 fixed slope directions from the low bits of the hash
 * and measures how much the point (x, y) leans along it. */
static inline float grad(int hash, float x, float y)
{
    int h = hash & 7;
    float u = (h < 4) ? x : y;
    float v = (h < 4) ? y : x;
    return ((h & 1) ? -u : u) + ((h & 2) ? -2.0f * v : 2.0f * v);
}

/*
 * Reads the noise value at point (x, y); result lands roughly in -1..1.
 * It finds the grid square the point sits in, reads a slope at each of
 * the four corners, and smoothly blends them based on where the point
 * falls inside the square.
 */
static float noise_perlin2d(const Noise *n, float x, float y)
{
    /* Which grid square, and where inside it (0..1). The & 255 makes
     * the noise repeat every 256 units. */
    int X = (int)floorf(x) & 255;
    int Y = (int)floorf(y) & 255;
    x -= floorf(x);
    y -= floorf(y);

    /* Soften the position so the blend across edges looks smooth. */
    float u = fade(x);
    float v = fade(y);

    /* Look up which slope lives at each corner of the square. */
    int A = n->perm[X    ] + Y;
    int B = n->perm[X + 1] + Y;

    /* How much each corner's slope leans toward our point. */
    float n00 = grad(n->perm[A    ], x,        y       );  /* corner (0,0) */
    float n10 = grad(n->perm[B    ], x - 1.0f, y       );  /* corner (1,0) */
    float n01 = grad(n->perm[A + 1], x,        y - 1.0f);  /* corner (0,1) */
    float n11 = grad(n->perm[B + 1], x - 1.0f, y - 1.0f);  /* corner (1,1) */

    /* Blend the four corners into one value. */
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/*
 * Stacks several noise layers, each one finer and fainter than the
 * last, then averages them. This adds small-scale detail on top of the
 * broad shape so the field looks natural rather than blobby. Result
 * still lands roughly in -1..1. (Ebert et al., "Texturing & Modeling".)
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
        amp     *= 0.5f;          /* next layer is half as strong */
        freq    *= 2.0f;          /* and twice as fine            */
    }
    return total / max_amp;       /* keep the result in -1..1 no matter how many layers */
}

/* The smooth height field whose tilt we'll turn into the flow. Drifting
 * time into the y coordinate is what makes the field slowly evolve. */
static inline float noise_potential(const Noise *n, float x, float y, float t)
{
    return noise_fbm(n, x, y + t);
}

/* A wobblier version of the height field: we use one noise sample to
 * nudge where we read the next one, which bends the field into more
 * eddies. (Inigo Quilez, "Domain Warping".) */
static float noise_warped_potential(const Noise *n, float x, float y, float t)
{
    float qx = noise_fbm(n, x,                  y                 + t);
    float qy = noise_fbm(n, x + WARP_OFFSET_X,  y + WARP_OFFSET_Y + t);
    return noise_fbm(n, x + WARP_AMOUNT * qx,
                        y + WARP_AMOUNT * qy + t);
}

/* Reads the height field, using the plain or warped version depending
 * on the flag, so the flow code below doesn't have to care which. */
static inline float psi_at(const Noise *n, float x, float y, float t, bool warp)
{
    return warp ? noise_warped_potential(n, x, y, t)
                : noise_potential       (n, x, y, t);
}

/*
 * The heart of curl noise: turn the height field into a flow direction.
 * We measure which way the field slopes (by comparing nearby samples),
 * then rotate that slope a quarter turn. Flow that circles around peaks
 * instead of climbing them never lets particles pile up. (Bridson 2007.)
 */
static void noise_curl_at(const Noise *n, float x, float y, float t, bool warp,
                          float *out_vx, float *out_vy)
{
    /* Read the field just to each side of the point. */
    float psi_yp = psi_at(n, x,            y + CURL_EPS, t, warp);
    float psi_ym = psi_at(n, x,            y - CURL_EPS, t, warp);
    float psi_xp = psi_at(n, x + CURL_EPS, y,            t, warp);
    float psi_xm = psi_at(n, x - CURL_EPS, y,            t, warp);

    /* How steeply the field rises going up vs. going right. */
    float dpsi_dy = (psi_yp - psi_ym) / (2.0f * CURL_EPS);
    float dpsi_dx = (psi_xp - psi_xm) / (2.0f * CURL_EPS);

    /* Rotate that slope a quarter turn to get the flow direction. */
    *out_vx =  dpsi_dy;
    *out_vy = -dpsi_dx;
}

/* ===================================================================== */
/* §6  patterns — shared helpers                                          */
/* ===================================================================== */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : v > hi ? hi : v;
}

/* Turn a brightness (0..1) into one of the 4 colour bands. The mask
 * is a safety net so a stray value can't pick a band that doesn't exist. */
static inline int quantize_glow_to_band(float glow)
{
    return (int)(glow * BAND_QUANTIZE_SCALE) & (N_BANDS - 1);
}

/* Pick the arrow character that best matches a flow direction: a dot
 * if it's barely moving, a straight arrow if it's mostly horizontal or
 * vertical, otherwise a diagonal slash. */
static char arrow_for(float vx, float vy)
{
    float ax = fabsf(vx), ay = fabsf(vy);

    /* Barely moving. */
    if (ax < ARROW_STATIONARY_THRESH && ay < ARROW_STATIONARY_THRESH)
        return '.';

    /* One direction clearly dominates. */
    if (ax > ARROW_AXIS_DOMINANCE * ay) return vx > 0.0f ? '>' : '<';
    if (ay > ARROW_AXIS_DOMINANCE * ax) return vy > 0.0f ? 'v' : '^';

    /* Diagonal: which way depends on whether the two parts agree in sign. */
    return ((vx * vy) > 0.0f) ? '\\' : '/';
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/* The scene is six small structs, each in charge of one thing, bundled
 * together by the Scene struct at the bottom. Splitting them this way
 * keeps each function's job obvious from what it's handed. */

/*
 * Particle — one dot carried along by the flow. Only the PARTICLES and
 * WARPED views use these; the other three leave them sitting idle.
 *
 * A particle has no momentum: each step it just moves in the flow's
 * current direction. That's the whole point — with no inertia and a
 * flow that can't pile things up, the dots can never bunch together.
 * (Bridson 2007, particle update.)
 *
 * Each one also lives for a limited time and then respawns elsewhere,
 * so the dots keep spreading around the field instead of all draining
 * into the same few loops. The random lifetime staggers the respawns.
 */
typedef struct {
    float x, y;       /* position, in grid cells (fractional)               */
    int   color_idx;  /* which of the 4 ramp colours this dot uses          */
    int   age;        /* how many ticks it's been alive                     */
    int   max_age;    /* respawn once age reaches this (or if it leaves the grid) */
} Particle;

/*
 * Grid — just the map's size. No pixels, no state. It sits first
 * because everything else needs the dimensions.
 *
 * Cells are stored row by row, so cell (x, y) lives at y*w + x. Looping
 * y on the outside and x on the inside therefore walks straight through
 * memory, which is fast. Width times height is always within CELLS_MAX
 * (app_pick_map_size clamps it), so the fixed-size buffers below always fit.
 */
typedef struct {
    int w, h;         /* map width / height in cells                      */
    int total_cells;  /* w * h, kept around so hot loops skip the multiply */
} Grid;

static inline int grid_idx(const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds(const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * RenderBuffers — what each cell should show. This is the only thing
 * that passes from the simulation to the drawing code: the patterns
 * fill these arrays, and the screen reads them and nothing else. That
 * clean handoff means the drawing could change without touching any
 * pattern. Three separate arrays (rather than one array of structs) so
 * the screen can scan the brightness array tightly, and so one array
 * can be wiped without disturbing the others.
 *
 * The brightness-to-character mapping follows Paul Bourke's ASCII
 * grey-scale ramp.
 */
typedef struct {
    /* Brightness, 0..1. Particle trails stamp 1.0 and fade; the other
     * views just overwrite this every frame. Below GLOW_THRESHOLD the
     * cell shows nothing. */
    float   glow [CELLS_MAX];

    /* Which of the 4 ramp colours this cell uses. */
    uint8_t color[CELLS_MAX];

    /* A character to force, or 0 to let brightness pick one. The VECTOR
     * view puts its arrow characters here; everything else leaves it 0. */
    char    glyph[CELLS_MAX];
} RenderBuffers;

static void buffers_clear(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow[i]  = 0.0f;
        b->color[i] = 0;
        b->glyph[i] = 0;
    }
}

/*
 * Particles — a fixed block of dots, of which the first n are alive.
 * Pre-allocating the whole block means spawning and respawning never
 * has to ask the system for memory. The default 256 is enough to fill
 * the visible swirls without turning into a solid haze.
 */
typedef struct {
    Particle pool[MAX_PARTICLES];  /* only the first n are in use */
    int      n;                    /* how many are alive          */
} Particles;

/*
 * SimState — the parts of the world that the simulation moves on its
 * own each tick, kept apart from the user's knobs (Controls) so it's
 * clear what the sim changes versus what the keyboard changes.
 */
typedef struct {
    /* How far we've drifted through the noise. Grows a little each tick,
     * which slowly reshapes the swirls without picking a new field. Reset
     * back to 0 by scene_reset(). */
    float field_time;

    /* Brightness of the reset sparkle. Set high on reset, fades to
     * nothing within about a second. While it's lit, the screen shows
     * scattered '*' sparkles. */
    float supernova_glow_t;
} SimState;

/*
 * Controls — the user's knobs. The keyboard handler writes these; the
 * simulation only reads them. Keeping them apart from SimState draws a
 * clean line: SimState is where things are, Controls is what the user
 * asked for.
 */
typedef struct {
    /* Frozen? The sim stops but the screen keeps redrawing, so the HUD
     * stays live and you can still switch themes or views while paused. */
    bool    paused;

    /* How fast the dots move, in cells per second. '+' and '-' double
     * and halve it (doubling feels like an even step at any speed). */
    int     speed;

    /* How fast the field itself drifts, on top of the particle speed.
     * '+' / '-' nudge this alongside speed. */
    int     drift_mult;

    int     current_theme;     /* index into themes[]      */
    Pattern current_pattern;   /* which view is showing     */

    /* Last frame's view. Comparing it to current_pattern tells us the
     * moment the user switched, so we can wipe the screen clean —
     * otherwise old trails would bleed into the new view. */
    Pattern prev_pattern;
} Controls;

/*
 * Scene — everything, in one place. The six pieces are listed in the
 * order they depend on each other, so reading top to bottom you meet
 * each idea before it's used: the map, the noise it samples, the cells
 * it draws, the dots that move, the animation state, and the user's knobs.
 */
typedef struct {
    Grid          grid;       /* the map size; only a resize changes it       */
    Noise         noise;      /* the field; only a reset reshuffles it        */
    RenderBuffers buf;        /* what to draw; patterns fill it, screen reads it */
    Particles     particles;  /* the dots; idle unless a particle view is on  */
    SimState      sim;        /* animation state; only scene_tick changes it  */
    Controls      ctrl;       /* user knobs; only the keyboard changes them   */
} Scene;

static void particle_spawn(Particle *p, const Grid *g)
{
    p->x         = (float)(rand() % g->w);
    p->y         = (float)(rand() % g->h);
    p->color_idx = rand() & (N_BANDS - 1);
    p->age       = 0;
    p->max_age   = AGE_MIN_TICKS + rand() % (AGE_MAX_TICKS - AGE_MIN_TICKS);
}

/* Moving one dot: read which way the flow points here, step along it,
 * leave a mark, and respawn if it's too old or has left the grid. The
 * four lines of particle_step_curl below are exactly those four steps. */

/* Read the flow direction at the dot's spot and shrink it to a unit
 * length. We strip out the strength here so every dot moves at the same
 * pace (the speed knob sets that) no matter how strong the flow is
 * locally. Skip the shrink if it's basically still, to avoid dividing
 * by zero. */
static void sample_unit_velocity(const Scene *s, float px, float py, bool warp,
                                  float *out_vx, float *out_vy)
{
    noise_curl_at(&s->noise,
                  px * NOISE_SCALE, py * NOISE_SCALE,
                  s->sim.field_time, warp, out_vx, out_vy);
    float mag = sqrtf((*out_vx) * (*out_vx) + (*out_vy) * (*out_vy));
    if (mag > VELOCITY_EPSILON) {
        *out_vx /= mag;
        *out_vy /= mag;
    }
}

/* Nudge the dot along the flow by one frame's worth of motion, and age
 * it. No momentum — it always goes wherever the flow currently points. */
static void advect_particle_euler(Particle *p, float vx, float vy,
                                   float dt, int speed)
{
    p->x += vx * (float)speed * dt;
    p->y += vy * (float)speed * dt;
    p->age++;
}

/* Light up the cell the dot is sitting on at full brightness. The
 * trail you see is the older cells fading out behind it. */
static void deposit_trail_hit(Scene *s, int cx, int cy, int color_idx)
{
    if (!grid_in_bounds(&s->grid, cx, cy)) return;
    int idx = grid_idx(&s->grid, cx, cy);
    s->buf.glow [idx] = TRAIL_HIT_INTENSITY;
    s->buf.color[idx] = (uint8_t)color_idx;
    s->buf.glyph[idx] = 0;        /* let brightness pick the character */
}

/* Has this dot run out its life or wandered off the grid? Either way
 * it's due for a respawn. */
static bool particle_is_expired(const Particle *p, const Grid *g)
{
    return p->age >= p->max_age
        || p->x < 0.0f || p->x >= (float)g->w
        || p->y < 0.0f || p->y >= (float)g->h;
}

static void particle_step_curl(Particle *p, Scene *s, float dt, bool warp)
{
    float vx, vy;
    sample_unit_velocity (s, p->x, p->y, warp, &vx, &vy);
    advect_particle_euler(p, vx, vy, dt, s->ctrl.speed);
    deposit_trail_hit    (s, (int)p->x, (int)p->y, p->color_idx);
    if (particle_is_expired(p, &s->grid))
        particle_spawn   (p, &s->grid);
}

/* The three non-particle views (VECTOR, POTENTIAL, CURL_MAG) figure
 * out what each cell should look like one cell at a time;
 * scene_update_static walks the grid and calls the right one. */

/* What one cell should look like: how bright, which colour, and which
 * character (or 0 to let brightness decide). */
typedef struct {
    float glow;   /* brightness, 0..1            */
    int   band;   /* which ramp colour           */
    char  glyph;  /* forced character, or 0      */
} CellPaint;

/* POTENTIAL view: show the field's height directly as brightness. The
 * remap shifts the height from its -1..1 range up into 0..1. */
static CellPaint compute_potential_cell(const Noise *n, float fx, float fy, float t)
{
    float psi  = noise_potential(n, fx, fy, t);
    float glow = psi * POTENTIAL_REMAP_RANGE + POTENTIAL_REMAP_MID;
    return (CellPaint){ .glow  = glow,
                        .band  = quantize_glow_to_band(glow),
                        .glyph = 0 };
}

/* CURL_MAG view: brightness = how strongly the flow is moving here, so
 * the whirlpool centres glow and the calm areas stay dim. The gain
 * brightens it up to use the full range. */
static CellPaint compute_curl_magnitude_cell(const Noise *n, float fx, float fy, float t)
{
    float vx, vy;
    noise_curl_at(n, fx, fy, t, false, &vx, &vy);
    float mag  = sqrtf(vx * vx + vy * vy);
    float glow = clampf(mag * CURL_MAG_VISUAL_GAIN, 0.0f, 1.0f);
    return (CellPaint){ .glow  = glow,
                        .band  = quantize_glow_to_band(glow),
                        .glyph = 0 };
}

/* VECTOR view: an arrow pointing the way the flow goes here, with the
 * flow's strength as its brightness so weak spots stay faint. */
static CellPaint compute_vector_arrow_cell(const Noise *n, float fx, float fy, float t)
{
    float vx, vy;
    noise_curl_at(n, fx, fy, t, false, &vx, &vy);
    float mag  = sqrtf(vx * vx + vy * vy);
    float glow = clampf(mag * CURL_MAG_VISUAL_GAIN, 0.0f, 1.0f);
    return (CellPaint){ .glow  = glow,
                        .band  = quantize_glow_to_band(glow),
                        .glyph = arrow_for(vx, vy) };
}

/* Is this one of the spaced-out cells that gets an arrow? The rest are
 * left blank so the arrows don't crowd each other. */
static inline bool is_vector_lattice_point(int x, int y)
{
    return (x % VECTOR_LATTICE_X) == 0
        && (y % VECTOR_LATTICE_Y) == 0;
}

/* Clear the field to blank before drawing arrows, since only a few
 * cells get one and the rest must read as empty. */
static void wipe_vector_layer(RenderBuffers *b, int n)
{
    for (int i = 0; i < n; i++) {
        b->glow [i] = 0.0f;
        b->glyph[i] = 0;
    }
}

static inline void write_cell(RenderBuffers *b, int idx, CellPaint p)
{
    b->glow [idx] = p.glow;
    b->color[idx] = (uint8_t)(p.band & (N_BANDS - 1));
    b->glyph[idx] = (uint8_t) p.glyph;
}

/* Redraw the whole field for one of the non-particle views: walk every
 * cell, work out its look, and store it. */
static void scene_update_static(Scene *s, Pattern pat)
{
    const Grid    *g = &s->grid;
    const Noise   *n = &s->noise;
    RenderBuffers *b = &s->buf;
    float          t = s->sim.field_time;

    if (pat == PATTERN_VECTOR) wipe_vector_layer(b, g->total_cells);

    for (int y = 0; y < g->h; y++) {
        for (int x = 0; x < g->w; x++) {
            float fx = (float)x * NOISE_SCALE;
            float fy = (float)y * NOISE_SCALE;
            CellPaint cp;

            switch (pat) {
            case PATTERN_VECTOR:
                if (!is_vector_lattice_point(x, y)) continue;
                cp = compute_vector_arrow_cell  (n, fx, fy, t); break;
            case PATTERN_POTENTIAL:
                cp = compute_potential_cell     (n, fx, fy, t); break;
            case PATTERN_CURL_MAG:
                cp = compute_curl_magnitude_cell(n, fx, fy, t); break;
            default:
                continue;
            }

            write_cell(b, grid_idx(g, x, y), cp);
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

static void spawn_all_particles(Particles *ps, const Grid *g)
{
    ps->n = N_PARTICLES_DEF;
    for (int i = 0; i < ps->n; i++)
        particle_spawn(&ps->pool[i], g);
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions(&s->grid, w, h);
    reset_sim_state      (&s->sim);
    buffers_clear        (&s->buf, s->grid.total_cells);
    noise_shuffle        (&s->noise);
    spawn_all_particles  (&s->particles, &s->grid);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused          = false;
    s->ctrl.speed           = SPEED_DEF;
    s->ctrl.drift_mult      = DRIFT_MULT_DEF;
    s->ctrl.current_theme   = 0;
    s->ctrl.current_pattern = PATTERN_PARTICLES;
    s->ctrl.prev_pattern    = PATTERN_PARTICLES;
    scene_reset(s, w, h);
}

/* One simulation step. If paused, do nothing. Otherwise: clear the
 * screen if the user just switched views, fade the reset sparkle, drift
 * the field, and run whichever view is active. The field never reshuffles
 * on its own — only the 'r' key does that. */

static void detect_pattern_switch_and_wipe(Scene *s)
{
    if (s->ctrl.current_pattern == s->ctrl.prev_pattern) return;
    buffers_clear(&s->buf, s->grid.total_cells);
    s->ctrl.prev_pattern = s->ctrl.current_pattern;
}

static void decay_supernova_flash(Scene *s, float dt)
{
    s->sim.supernova_glow_t *= expf(-SUPERNOVA_DECAY * dt);
}

static void advance_field_time(Scene *s, float dt)
{
    s->sim.field_time += FIELD_DRIFT * (float)s->ctrl.drift_mult * dt;
}

static bool pattern_uses_particles(Pattern p)
{
    return p == PATTERN_PARTICLES || p == PATTERN_WARPED;
}

static void decay_trail_glow(RenderBuffers *b, int n, float dt)
{
    float decay = expf(-TRAIL_GLOW_DECAY * dt);
    for (int i = 0; i < n; i++) b->glow[i] *= decay;
}

static void step_all_particles(Scene *s, float dt, bool warp)
{
    for (int i = 0; i < s->particles.n; i++)
        particle_step_curl(&s->particles.pool[i], s, dt, warp);
}

static void simulate_particle_patterns(Scene *s, float dt, bool warp)
{
    decay_trail_glow (&s->buf, s->grid.total_cells, dt);
    step_all_particles(s, dt, warp);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;

    detect_pattern_switch_and_wipe(s);
    decay_supernova_flash         (s, dt);
    advance_field_time            (s, dt);

    Pattern pat = s->ctrl.current_pattern;
    if (pattern_uses_particles(pat))
        simulate_particle_patterns(s, dt, pat == PATTERN_WARPED);
    else
        scene_update_static       (s, pat);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/* Screen — just the terminal's current width and height, re-read
 * whenever the window resizes. We use it to centre the map and place
 * the HUD; ncurses tracks everything else. */
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

/* For each cell, the drawing code decides one of four things to show,
 * in priority order: a reset sparkle, an arrow the view forced, a
 * brightness character, or nothing. Each choice returns a CellDraw, and
 * paint_cell is the only place that actually writes to the screen. */

typedef struct {
    int  pair;    /* colour to use   */
    int  attr;    /* bold or normal  */
    char glyph;   /* character to put */
    bool skip;    /* true = draw nothing here */
} CellDraw;

/* Where the map's top-left corner goes: centred sideways, with rows
 * reserved top and bottom for the HUD. Clamped so the map never spills
 * onto the HUD on a short terminal. */
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

/* The scattered '*' sparkles during the reset flash. Only some cells
 * sparkle (so it looks like stars, not a solid sheet), but any cell with
 * a live trail still lights up so trails show through the flash. */
static CellDraw cell_supernova_sparkle(int x, int y, float trail_glow)
{
    bool sparkle_lit = ((x ^ y) & SUPERNOVA_SPARSE_MASK) == 0;
    if (!sparkle_lit && trail_glow <= GLOW_THRESHOLD)
        return (CellDraw){ .skip = true };
    return (CellDraw){ .pair = PAIR_SUPERNOVA, .attr = A_BOLD, .glyph = '*' };
}

/* Draw the character a view forced into this cell (the VECTOR arrows),
 * coloured by its band and bold. */
static CellDraw cell_with_override_glyph(const RenderBuffers *b, int idx)
{
    return (CellDraw){
        .pair  = PAIR_BAND_BASE + (b->color[idx] & (N_BANDS - 1)),
        .attr  = A_BOLD,
        .glyph = b->glyph[idx],
    };
}

/* The usual brightness characters: '#' for bright, '*' for medium,
 * '.' for dim, nothing below that. (Bourke's grey-scale ramp.) */
static CellDraw cell_density_band(uint8_t band, float glow)
{
    int pair = PAIR_BAND_BASE + (band & (N_BANDS - 1));
    if (glow > GLYPH_HIGH_THRESH) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '#' };
    if (glow > GLYPH_MID_THRESH ) return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '*' };
    if (glow > GLOW_THRESHOLD   ) return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = '.' };
    return (CellDraw){ .skip = true };
}

/* Decide what a single cell shows, trying each option in priority order. */
static CellDraw pick_cell(const Scene *s, int x, int y)
{
    int   idx        = grid_idx(&s->grid, x, y);
    float trail_glow = s->buf.glow[idx];

    if (s->sim.supernova_glow_t > GLOW_THRESHOLD)
        return cell_supernova_sparkle(x, y, trail_glow);

    if (s->buf.glyph[idx] != 0 && trail_glow > GLOW_THRESHOLD)
        return cell_with_override_glyph(&s->buf, idx);

    return cell_density_band(s->buf.color[idx], trail_glow);
}

/* The single spot that actually writes a field cell to the terminal. */
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

/* The HUD, one small drawer per piece. The top three rows show info
 * (state, settings, a key to the characters); the bottom row lists the
 * keys you can press. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);

    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, state_str, c->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " CURL NOISE VECTOR FIELD ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Draw a little '#' in each ramp colour as a preview. Hands back the
 * next free column so the caller can keep laying things out. */
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

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    const Grid     *g = &s->grid;
    int x = HUD_LEFT_MARGIN;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " pattern:%-9s ", pattern_name(c->current_pattern));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += HUD_PATTERN_FIELD_W;

    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[c->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += HUD_THEME_FIELD_W;

    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += HUD_PALETTE_LABEL_W;

    x = draw_palette_swatch(1, x);

    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  drift:x%-2d  eps:%.2f  map:%dx%d ",
             c->drift_mult, CURL_EPS, g->w, g->h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* A key to what the characters mean, so you can read the picture. */
static void draw_hud_glyph_legend(void)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(2, HUD_LEFT_MARGIN,
             " legend:  .:low  *:mid  #:high   arrows: > < ^ v / \\ ");
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* The list of keys along the bottom row. */
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
    scene_draw            (s, sc->cols, sc->rows);  /* the field, underneath */
    draw_hud_state_bar    (sc, s, fps, sim_fps);
    draw_hud_title        ();
    draw_hud_status_line  (s);
    draw_hud_glyph_legend ();
    draw_bottom_hint      (sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the whole program in one struct. There's a single copy, g_app,
 * kept global so the signal handlers can reach it. It owns the scene and
 * the screen, plus the tick rate and map size and two flags the signal
 * handlers flip. The handlers only flip a flag; the main loop notices
 * and does the real work (resize, cleanup), because doing it inside a
 * handler isn't safe.
 */
typedef struct {
    Scene                 scene;   /* the simulation               */
    Screen                screen;  /* terminal size                */

    int                   sim_fps; /* tick rate; '[' and ']' change it */
    int                   map_w;   /* map width  */
    int                   map_h;   /* map height */

    /* Set by the signal handlers, read by the main loop. The two
     * qualifiers together make these safe to share that way. */
    volatile sig_atomic_t running;       /* goes 0 to quit        */
    volatile sig_atomic_t need_resize;   /* set 1 when resized    */
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

/* '+' and '-': double or halve both the particle speed and the field
 * drift, kept within their limits. Doubling feels like the same step
 * whether you're slow or fast. dir +1 speeds up, -1 slows down. */
static void bump_speed_geometric(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->speed      < SPEED_MAX)      c->speed      *= 2;
        if (c->speed      > SPEED_MAX)      c->speed      = SPEED_MAX;
        if (c->drift_mult < DRIFT_MULT_MAX) c->drift_mult *= 2;
        if (c->drift_mult > DRIFT_MULT_MAX) c->drift_mult = DRIFT_MULT_MAX;
    } else {
        c->speed      /= 2;
        if (c->speed      < SPEED_MIN)      c->speed      = SPEED_MIN;
        c->drift_mult /= 2;
        if (c->drift_mult < DRIFT_MULT_MIN) c->drift_mult = DRIFT_MULT_MIN;
    }
}

/* '[' and ']': raise or lower the tick rate, kept within its limits. */
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
    case ' ':           c->paused = !c->paused;                           break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h); break;
    case '=': case '+': bump_speed_geometric(c, +1);                      break;
    case '-':           bump_speed_geometric(c, -1);                      break;
    case ']':           bump_sim_fps(app, +SIM_FPS_STEP);                 break;
    case '[':           bump_sim_fps(app, -SIM_FPS_STEP);                 break;
    case 't':           cycle_theme  (c, +1);                             break;
    case 'T':           cycle_theme  (c, -1);                             break;
    case 'n': case 'N': cycle_pattern(c, +1);                             break;
    case 'p': case 'P': cycle_pattern(c, -1);                             break;
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

/* How long since the last frame, capped so a long stall can't make the
 * sim try to catch up all at once. Also moves the clock forward. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* Run as many fixed-size sim steps as the elapsed time has earned, so
 * the sim always advances by the same amount per step no matter the
 * frame rate. (Glenn Fiedler, "Fix Your Timestep".) */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Roughly twice a second, work out the real frame rate from the frames
 * counted so far. The rest of the time it just returns the old number. */
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

/* Sleep off the rest of this frame's time budget so we don't run faster
 * than the target rate (and don't burn the CPU spinning). */
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
