/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hydraulic.c — carve a landscape by letting water do the work.
 *
 * Start from smooth fractal hills, drop thousands of tiny water droplets
 * on them, and let each one run downhill: it digs dirt out of steep slopes
 * and drops it back on flat ground. Rivers, valleys and deltas appear on
 * their own — nobody draws them. Fifteen views (n/p) show the same terrain
 * different ways: plain biome map, glowing water trails, where it was cut
 * vs filled, slope arrows, hillshade, contours, and so on.
 *
 * The droplet model is from Beyer's thesis (TU München, 2015) and Sebastian
 * Lague's "Coding Adventure: Hydraulic Erosion" (2019). Base terrain is
 * Perlin fBm noise. The cartographic views (hillshade, hypsometric tint,
 * contours) follow Imhof, "Cartographic Relief Presentation" (1982).
 * Sister file: ../worldgen/tectonic.c builds the same kind of terrain from
 * plate tectonics instead of by eroding noise.
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra \
 *     procedural/worldgen/hydraulic.c \
 *     -o hydraulic -lncurses -lm
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

/* The file is split into concern-layers, one per §-section. The rule that
 * keeps it sane: only §4 (SIMULATION) ever writes the terrain. Everything in
 * §3 just reads and returns a value; §7 only draws; user keypresses (§8) run
 * once a frame, outside the simulation step. */

/* ===================================================================== *
 * §1  CONFIG  -- constants, data tables, core state types               *
 * ===================================================================== */

enum {
    MAP_W_MAX           = 240,
    MAP_H_MAX           =  80,
    CELLS_MAX           = MAP_W_MAX * MAP_H_MAX,

    /* Smallest map we'll draw, so a tiny terminal still shows something. */
    MAP_W_MIN           =  16,
    MAP_H_MIN           =   8,
    HUD_TOP_ROWS        =   2,    /* rows 0–1 belong to the HUD        */

    DROPLET_MAX_STEPS   =  32,    /* how far one droplet walks before it dies */
    EROSION_BRUSH_R     =   2,    /* a droplet's bite spreads over a 2-cell disc */

    /* Droplets to release each step at the default speed; scales with speed. */
    DROPLETS_PER_TICK_DEF =  12,

    /* Once this many droplets have run on one terrain, pause and then start
     * a fresh terrain. */
    DROPLETS_PER_GEN    = 8000,
    HOLD_TICKS          = 6 * 60,    /* the pause length, ~6 s at 60 fps  */

    SIM_FPS_MIN         =  10,
    SIM_FPS_DEFAULT     =  60,
    SIM_FPS_MAX         = 240,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Colour-pair slot numbers. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = the 8 elevation colours  */
    PAIR_HOT            =  11,    /* warm accent: cuts / peaks         */
    PAIR_COLD           =  12,    /* cool accent: fills / water        */
    PAIR_WATER          =  13,    /* the live water trail              */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* How often we redraw the screen. The simulation runs at its own rate
 * (SIM_FPS_*), separate from drawing. */
#define FRAME_CAP_FPS   60
/* If a frame ever takes longer than this (process paused, terminal blocked),
 * pretend only this much time passed — otherwise we'd try to catch up by
 * running a huge burst of steps at once and stall the program. */
#define MAX_FRAME_NS    (100 * NS_PER_MS)

/* Terminal cells are about twice as tall as they are wide. We stretch the
 * vertical direction by this when sampling so hills look round, not squashed. */
#define ASPECT_Y_F           2.0f

/* Settings for the starting fractal terrain. Smaller scale = bigger hills. */
#define FBM_SCALE_X          0.045f
#define FBM_SCALE_Y          0.090f
#define FBM_OCTAVES          5
/* Pushes high spots higher and low spots lower so peaks and valleys stand out. */
#define FBM_GAMMA            1.30f

/* Droplet behaviour, using Sebastian Lague's default numbers. */
#define INERTIA              0.05f
#define SEDIMENT_CAPACITY_K  4.0f
#define MIN_CAPACITY         0.01f
#define ERODE_RATE           0.30f
#define DEPOSIT_RATE         0.30f
#define EVAPORATE_RATE       0.01f
#define GRAVITY              4.0f
#define INITIAL_WATER        1.0f
#define INITIAL_SPEED        1.0f
/* If a droplet's heading gets this short it has stalled on flat ground, so we
 * stop it (also dodges a divide-by-almost-zero when we rescale the heading). */
#define DROPLET_STALL_LEN    1e-4f

/* How the glowing droplet trails fade. 0.92 per step keeps a single visit
 * visible for about half a second. The two thresholds split the glow into
 * bright / medium / faint. */
#define WATER_TRAIL_DECAY     0.92f
#define WATER_TRAIL_HIGH      0.55f
#define WATER_TRAIL_MID       0.20f

/* EROSION view: how far a cell must differ from its starting height before we
 * paint it as cut or filled. */
#define EROSION_THRESH_LOW    0.012f
#define EROSION_THRESH_HIGH   0.05f

/* SLOPE view: multiplies steepness to pick a brightness level. */
#define SLOPE_SCALE           18.0f

/* Hillshade (RELIEF / CANYONS / BATHY / NIGHT): a fixed light in the top-left,
 * with the slope exaggerated so even gentle ground looks three-dimensional. */
#define RELIEF_LIGHT_X       -0.55f
#define RELIEF_LIGHT_Y       -0.55f
#define RELIEF_LIGHT_Z        0.63f
#define RELIEF_EXAGGERATE     7.0f

/* CONTOUR / FLOW: round elevation into this many bands; a contour line is drawn
 * wherever a cell's band differs from a neighbour's. */
#define CONTOUR_LEVELS        16

/* RIDGES: curvature past this much marks a ridge crest or a valley floor. */
#define RIDGE_THRESH          0.006f

/* ── Biome ─────────────────────────────────────────────────────────────── *
 * Eight elevation bands, from deep ocean (0) up to snowy peaks (7). A cell's
 * height (0..1) decides its band in height_to_biome(): roughly, below 0.30 is
 * sea, above 0.85 is snow. The clever bit: this same 0..7 number is also the
 * row to use in the colour table, the glyph table and the colour-pair list,
 * so colours, characters and bands can never drift apart. That's why all
 * three tables are exactly N_BIOMES (8) long — change 8 and you must retune
 * all of them. */
typedef enum {
    BIOME_DEEP_OCEAN = 0,
    BIOME_OCEAN      = 1,
    BIOME_COAST      = 2,
    BIOME_PLAINS     = 3,
    BIOME_HILLS      = 4,
    BIOME_MOUNTAINS  = 5,
    BIOME_HIGHLANDS  = 6,
    BIOME_PEAKS      = 7,
    N_BIOMES         = 8,
} Biome;

/* Two characters per band; a per-cell hash picks one of the pair so the
 * ground looks textured instead of a flat field of one symbol. */
static const char BIOME_GLYPHS[N_BIOMES][2] = {
    { '~', ',' },     /* DEEP_OCEAN */
    { '~', '_' },     /* OCEAN      */
    { '.', ',' },     /* COAST      */
    { '.', '_' },     /* PLAINS     */
    { '_', '^' },     /* HILLS      */
    { '^', 'A' },     /* MOUNTAINS  */
    { 'A', '#' },     /* HIGHLANDS  */
    { '#', '@' },     /* PEAKS      */
};

/* Characters from faint to dense, used wherever a view wants "darker = lower /
 * flatter, brighter = higher / steeper". */
static const char ELEV_RAMP[N_BIOMES] = {
    '`', '.', ',', ':', '-', '^', '#', '@'
};

/* Pattern — the fifteen ways the same terrain can be drawn. The n/p keys
 * cycle through them; each entry's comment is its one-line job. */
typedef enum {
    PATTERN_TERRAIN  = 0,   /* biome glyph map                         */
    PATTERN_DROPLETS,       /* live water trails over dim terrain      */
    PATTERN_EROSION,        /* cut/fill diff vs initial                */
    PATTERN_SLOPE,          /* gradient magnitude + flow arrows        */
    PATTERN_RELIEF,         /* colour hillshade — looks 3-D            */
    PATTERN_CONTOUR,        /* topographic isolines                    */
    PATTERN_HYPSO,          /* filled hypsometric tint                 */
    PATTERN_ASPECT,         /* slope-facing compass (8 hues)           */
    PATTERN_RIDGES,         /* ridge / valley skeleton                 */
    PATTERN_CANYONS,        /* carved channels glow over shaded land   */
    PATTERN_BATHY,          /* animated ocean caustics, calm land      */
    PATTERN_PLASMA,         /* animated colour field over the shape    */
    PATTERN_NIGHT,          /* dark sea, lit ridges, glowing peaks     */
    PATTERN_FLOW,           /* contour bands sweeping downhill         */
    PATTERN_HEAT,           /* two-tone thermal elevation map          */
    N_PATTERNS,
} Pattern;

/* Names padded to 8 characters so the HUD label stays the same width. */
static const char *PATTERN_NAMES[N_PATTERNS] = {
    "TERRAIN ", "DROPLETS", "EROSION ", "SLOPE   ", "RELIEF  ",
    "CONTOUR ", "HYPSO   ", "ASPECT  ", "RIDGES  ", "CANYONS ",
    "BATHY   ", "PLASMA  ", "NIGHT   ", "FLOW    ", "HEAT    ",
};

/* ── Theme ─────────────────────────────────────────────────────────────── *
 * One named colour scheme; the t/T keys cycle the ten of them. A theme just
 * says which terminal colours stand for low/high ground and for cut/fill, and
 * theme_apply() loads them into ncurses.
 *
 * ramp[] is a low-to-high colour run, one colour per elevation band (index 0
 * = deepest, 7 = highest) — like the colours on a relief map, blue seas up
 * through green plains to white peaks. hot and cold are the two accent
 * colours for the "what changed" views: hot marks ground that was cut away
 * (and peaks), cold marks ground that was filled in (and water). Every colour
 * is from the bright half of the palette so dimmed cells still show up on a
 * black terminal (see CLAUDE.md "Theme Palette Brightness").
 *
 * Map-colour idea from Imhof, "Cartographic Relief Presentation" (1982);
 * palette notes in documentation/COLOR.md. */
typedef struct {
    const char *name;            /* shown in the HUD                          */
    short       ramp[N_BIOMES];  /* low→high terrain colours                  */
    short       hot;             /* accent for cuts / peaks                   */
    short       cold;            /* accent for fills / water                  */
} Theme;

#define N_THEMES 10

/* Every colour here is kept in the bright half of the palette so even dimmed
 * cells stay readable on a black terminal (see /CLAUDE.md). */
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

/* ── Heightmap ─────────────────────────────────────────────────────────── *
 * The terrain itself: a w-by-h grid of heights, kept in flat arrays (cell
 * (x,y) lives at y*w+x — always reached through hidx() so the row math is in
 * one place). This is the one thing the whole program is about; all 15 views
 * are just different ways of reading it, and only §4 ever writes to it.
 *
 * We keep one array per kind of value (rather than a struct per cell) because
 * the busy loops sweep one value across the whole grid at a time, and packing
 * that value together makes those sweeps fast. The arrays are fixed-size, so
 * nothing is allocated while the program runs. */
typedef struct {
    /* Grid size in cells; capped at MAP_W_MAX × MAP_H_MAX. */
    int    w, h;

    /* Two copies of the heights, both 0..1. height[] is the live terrain that
     * droplets keep changing; initial[] is a frozen snapshot of how it looked
     * before any erosion. Keeping both lets the "what changed" views show
     * height minus initial — where ground was cut away vs piled up — which is
     * really the point of an erosion demo. */
    float  height     [CELLS_MAX];   /* current terrain                  */
    float  initial    [CELLS_MAX];   /* original, for the cut/fill view  */

    /* The fading droplet trails — purely for looks, not physics. Set to 1.0
     * wherever a droplet steps, then multiplied down a little each step so a
     * single visit glows for about half a second. Only the DROPLETS view
     * reads it; deleting it would change nothing about the terrain. */
    float  water_trail[CELLS_MAX];   /* recent-visit glow, fades over time */

    /* Bookkeeping for the current terrain. seed picks which random terrain we
     * grew (it's re-rolled from the clock each time, so every world differs).
     * droplets_done counts how many droplets have run; once it reaches
     * DROPLETS_PER_GEN the run is finished and the pause begins. */
    int    seed;                     /* which random terrain             */
    int    droplets_done;            /* droplets run so far this terrain */
} Heightmap;

/* ===================================================================== *
 * §2  PERFORMANCE  -- timing primitives                                 *
 * ===================================================================== */

/* Just the two clock helpers. How fast we run (the frame cap and the
 * fixed-step loop) is decided in main(), §8. */

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

/* ===================================================================== *
 * §3  LOGIC  -- pure decisions: no mutation, no I/O                     *
 * ===================================================================== */

/* Every function here just takes its inputs and hands back an answer — it
 * never changes the terrain or touches the screen. That makes it safe to
 * call from anywhere. Shared by the simulation (§4) and the renderers (§7). */

static const char *pattern_name(Pattern p)
{
    return ((int)p >= 0 && (int)p < N_PATTERNS) ? PATTERN_NAMES[p]
                                                : "?       ";
}

/* Turns three integers into one well-scrambled number — handy for "pick
 * something that looks random but stays the same for this cell". */
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

/* Perlin noise, the source of the starting hills. Copied in whole so this
 * file needs no shared headers. The next few helpers are its plumbing. */
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
    x -= floorf(x); y -= floorf(y);
    float u = fade_q(x), v = fade_q(y);
    int A = perm[X    ] + Y;
    int B = perm[X + 1] + Y;
    float n00 = grad2(perm[A    ], x,        y       );
    float n10 = grad2(perm[B    ], x - 1.0f, y       );
    float n01 = grad2(perm[A + 1], x,        y - 1.0f);
    float n11 = grad2(perm[B + 1], x - 1.0f, y - 1.0f);
    return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/* Stacks several layers of Perlin noise — each one finer and fainter than the
 * last — to get natural-looking lumpy terrain. Result is rescaled to 0..1. */
static float fbm2(float x, float y)
{
    float total = 0.0f, amp = 1.0f, freq = 1.0f, max_amp = 0.0f;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total   += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp     *= 0.5f;
        freq    *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

static inline int hidx(const Heightmap *hm, int x, int y) { return y * hm->w + x; }

/* Height at a point between cells, blended smoothly from the four cells
 * around it. No edge checks here — the caller (the droplet step) already made
 * sure (x,y) isn't right at the far edge. */
static inline float h_bilinear(const Heightmap *hm, float x, float y)
{
    int xi = (int)x, yi = (int)y;
    float fx = x - (float)xi, fy = y - (float)yi;
    int idx = hidx(hm, xi, yi);
    float h00 = hm->height[idx];
    float h10 = hm->height[idx + 1];
    float h01 = hm->height[idx + hm->w];
    float h11 = hm->height[idx + hm->w + 1];
    return (h00 * (1.0f - fx) + h10 * fx) * (1.0f - fy)
         + (h01 * (1.0f - fx) + h11 * fx) * fy;
}

/* Height of a whole cell, with out-of-range coordinates pulled back to the
 * nearest edge — so renderers that peek at neighbours can do so safely even at
 * the very border of the map. */
static inline float h_at(const Heightmap *hm, int x, int y)
{
    if (x < 0) x = 0; else if (x >= hm->w) x = hm->w - 1;
    if (y < 0) y = 0; else if (y >= hm->h) y = hm->h - 1;
    return hm->height[hidx(hm, x, y)];
}

/* Sorts a height (0..1) into one of the eight elevation bands. */
static inline int height_to_biome(float e)
{
    if (e < 0.18f) return BIOME_DEEP_OCEAN;
    if (e < 0.30f) return BIOME_OCEAN;
    if (e < 0.40f) return BIOME_COAST;
    if (e < 0.50f) return BIOME_PLAINS;
    if (e < 0.62f) return BIOME_HILLS;
    if (e < 0.75f) return BIOME_MOUNTAINS;
    if (e < 0.85f) return BIOME_HIGHLANDS;
    return BIOME_PEAKS;
}

/* Which way the ground slopes at a cell, found by comparing the cells on
 * either side. The up/down result is divided by ASPECT_Y_F because cells are
 * taller than wide, so a vertical step covers more ground. */
static inline void cell_gradient(const Heightmap *hm, int x, int y,
                                  float *gx, float *gy)
{
    int xm = (x > 0)        ? x - 1 : x;
    int xp = (x < hm->w - 1) ? x + 1 : x;
    int ym = (y > 0)        ? y - 1 : y;
    int yp = (y < hm->h - 1) ? y + 1 : y;
    *gx = (hm->height[hidx(hm, xp, y)] - hm->height[hidx(hm, xm, y)]) * 0.5f;
    *gy = (hm->height[hidx(hm, x, yp)] - hm->height[hidx(hm, x, ym)]) * 0.5f / ASPECT_Y_F;
}

/* How brightly a cell would catch the light (0 = shadow, 1 = full glare),
 * based on which way the ground faces relative to a fixed top-left light. The
 * slope is exaggerated first so even gentle ground shows some shading. Used by
 * all the hillshade views (RELIEF/CANYONS/BATHY/NIGHT). */
static inline float cell_shade(const Heightmap *hm, int x, int y)
{
    float gx, gy;
    cell_gradient(hm, x, y, &gx, &gy);
    gx *= RELIEF_EXAGGERATE;
    gy *= RELIEF_EXAGGERATE;
    float inv = 1.0f / sqrtf(gx * gx + gy * gy + 1.0f);
    float s = (-gx * RELIEF_LIGHT_X - gy * RELIEF_LIGHT_Y
               + RELIEF_LIGHT_Z) * inv;
    if (s < 0.0f) s = 0.0f;
    if (s > 1.0f) s = 1.0f;
    return s;
}

/* Turns a brightness (0..1) into a 0..7 step for picking a ramp character. */
static inline int shade_to_level(float sh)
{
    int l = (int)(sh * 7.0f + 0.5f);
    return l < 0 ? 0 : l > 7 ? 7 : l;
}

/* How much of a droplet's bite a nearby cell gets: full strength at the
 * droplet's own cell, tapering to nothing at the edge of the brush disc, zero
 * beyond it. Spreading the bite this way carves rounded valleys instead of
 * jagged single-cell scratches. */
static inline float brush_weight(int dx, int dy)
{
    float d = sqrtf((float)(dx * dx + dy * dy));
    return (d > (float)EROSION_BRUSH_R) ? 0.0f
                                        : 1.0f - d / (float)EROSION_BRUSH_R;
}

/* How much dirt a droplet is allowed to carry right now. The faster it's
 * moving, the more water it still has, and the steeper its drop, the more it
 * can hold — but never less than a tiny floor, so even a near-flat step can
 * still erode a little. */
static inline float sediment_capacity(float dh, float speed, float water)
{
    float cap = (-dh) * speed * water * SEDIMENT_CAPACITY_K;
    return (cap < MIN_CAPACITY) ? MIN_CAPACITY : cap;
}

/* Reads the ground right under a droplet: returns the blended height there and
 * fills in the local slope through (*gx,*gy). Doing both in one read means the
 * droplet's steering and its height change agree on exactly the same four
 * cells. The caller has already checked the point isn't at the far edge. */
static inline float bed_sample(const Heightmap *hm, float x, float y,
                               float *gx, float *gy)
{
    int xi = (int)x, yi = (int)y;
    float fx = x - (float)xi, fy = y - (float)yi;
    int idx = hidx(hm, xi, yi);
    float h00 = hm->height[idx];
    float h10 = hm->height[idx + 1];
    float h01 = hm->height[idx + hm->w];
    float h11 = hm->height[idx + hm->w + 1];
    *gx = (h10 - h00) * (1.0f - fy) + (h11 - h01) * fy;
    *gy = (h01 - h00) * (1.0f - fx) + (h11 - h10) * fx;
    return h00 * (1 - fx) * (1 - fy) + h10 * fx * (1 - fy)
         + h01 * (1 - fx) *      fy  + h11 * fx *      fy;
}

/* True once this terrain has used up its whole batch of droplets. */
static inline bool erosion_complete(const Heightmap *hm)
{
    return hm->droplets_done >= DROPLETS_PER_GEN;
}

/* Downhill flow arrow for a cell: the gradient points UPHILL, so water flows
 * the opposite way — pick the cardinal arrow of −gradient. Shared by the
 * SLOPE and CANYONS views. */
static inline char flow_arrow(float gx, float gy)
{
    if (fabsf(gx) > fabsf(gy)) return (gx > 0) ? '<' : '>';
    else                       return (gy > 0) ? '^' : 'v';
}

/* Contour-line glyph for a cell: an isoline runs PERPENDICULAR to the
 * gradient, so a mostly-horizontal gradient draws a vertical bar and vice
 * versa, with diagonals in between. Shared by the CONTOUR and FLOW views. */
static inline char contour_glyph(float gx, float gy)
{
    if      (fabsf(gx) > 2.0f * fabsf(gy)) return '|';
    else if (fabsf(gy) > 2.0f * fabsf(gx)) return '-';
    else    return (gx * gy > 0.0f) ? '\\' : '/';
}

/* Which of 8 compass sectors the downhill direction (−gradient) falls in,
 * 0..7 starting at east and turning CCW. Used by the ASPECT view to colour
 * and arrow each cell by the way its slope faces. */
static inline int downhill_sector(float gx, float gy)
{
    float ang = atan2f(-gy, -gx);
    return (int)floorf((ang + (float)M_PI) / (2.0f * (float)M_PI) * 8.0f) & 7;
}

/* Discrete Laplacian (∇²) at a cell — the 4-neighbour curvature of the
 * height field. Strongly negative = a convex ridge crest, strongly positive
 * = a concave valley floor. Used by the RIDGES view. */
static inline float cell_laplacian(const Heightmap *hm, int x, int y)
{
    return h_at(hm, x - 1, y) + h_at(hm, x + 1, y)
         + h_at(hm, x, y - 1) + h_at(hm, x, y + 1) - 4.0f * h_at(hm, x, y);
}

/* ===================================================================== *
 * §4  SIMULATION  -- advances state (only writers of sim state)         *
 * ===================================================================== */

/* The only place the terrain actually changes. scene_tick() is the one
 * entry point, called once per simulation step from main (§8). Keypresses
 * also change settings, but they're not part of a tick — see §8. */

static void perm_shuffle(int seed)
{
    uint8_t base[256];
    for (int i = 0; i < 256; i++) base[i] = (uint8_t)i;
    uint32_t st = (uint32_t)seed * 2654435761u;
    for (int i = 255; i > 0; i--) {
        st = st * 1664525u + 1013904223u;
        int j = (int)(st >> 16) % (i + 1);
        uint8_t t = base[i]; base[i] = base[j]; base[j] = t;
    }
    for (int i = 0; i < 256; i++) {
        perm[i      ] = base[i];
        perm[i + 256] = base[i];
    }
}

/*
 * heightmap_generate — sample fBm noise into height + initial, zero
 * out water_trail. Aspect-corrected y so blob features look round
 * on screen.
 */
static void heightmap_generate(Heightmap *hm, int seed)
{
    hm->seed = seed;
    hm->droplets_done = 0;
    perm_shuffle(seed);

    for (int y = 0; y < hm->h; y++) {
        for (int x = 0; x < hm->w; x++) {
            float nx = (float)x * FBM_SCALE_X;
            float ny = (float)y * FBM_SCALE_Y;
            float n  = fbm2(nx, ny);
            n = powf(n, FBM_GAMMA);                /* sharpen highs */
            int idx = hidx(hm, x, y);
            hm->height[idx]      = n;
            hm->initial[idx]     = n;
            hm->water_trail[idx] = 0.0f;
        }
    }
}

/* ----------------------------------------------------------------------- *
 * Erosion brush — disc-weighted height subtract.                          *
 * ----------------------------------------------------------------------- */

static void erode_brush(Heightmap *hm, float fx, float fy, float amount)
{
    int cx = (int)fx, cy = (int)fy;

    /* First pass — total disc weight, so the bite is normalised to `amount`
     * regardless of how much of the disc is clipped at the map edge. */
    float total_w = 0.0f;
    for (int dy = -EROSION_BRUSH_R; dy <= EROSION_BRUSH_R; dy++)
        for (int dx = -EROSION_BRUSH_R; dx <= EROSION_BRUSH_R; dx++)
            total_w += brush_weight(dx, dy);
    if (total_w < 1e-6f) return;

    /* Second pass — subtract each in-bounds cell's share of `amount`. */
    for (int dy = -EROSION_BRUSH_R; dy <= EROSION_BRUSH_R; dy++) {
        int ny = cy + dy;
        if (ny < 0 || ny >= hm->h) continue;
        for (int dx = -EROSION_BRUSH_R; dx <= EROSION_BRUSH_R; dx++) {
            int nx = cx + dx;
            if (nx < 0 || nx >= hm->w) continue;
            float w = brush_weight(dx, dy);
            hm->height[hidx(hm, nx, ny)] -= amount * w / total_w;
        }
    }
}

/* ── Droplet ───────────────────────────────────────────────────────────── *
 * WHAT  One water particle in particle-based ("droplet") hydraulic erosion.
 *       The sim never carves rivers directly: it drops thousands of these and
 *       lets each trace a downhill path, eroding the bed where it accelerates
 *       and depositing where it slows or oversaturates. Stream networks EMERGE
 *       as the sum of many paths — no river is ever drawn deliberately.
 *
 * LIFETIME  Stack-local, never stored: spawned (droplet_spawn), walked for
 *       ≤ DROPLET_MAX_STEPS (32) steps (droplet_simulate), discarded. Its only
 *       lasting trace is the height[] it edited and the water_trail[] it lit.
 *
 * REFS  Heading + carrying-capacity model — Beyer (2015, "The droplet");
 *       Lague (2019). Tuning constants in §1 (INERTIA, SEDIMENT_CAPACITY_K,
 *       MIN_CAPACITY, ERODE_RATE, DEPOSIT_RATE, EVAPORATE_RATE, GRAVITY). */
typedef struct {
    /* ── where it is / where it's heading ─────────────────────────────────
     * (x,y) is a FRACTIONAL position in cell space (bilinearly sampled, never
     * snapped to a cell). (dx,dy) is the heading, renormalised to unit length
     * each step: dx = dx·INERTIA − gx·(1−INERTIA). With INERTIA = 0.05 it is
     * ~95% downhill −gradient + 5% memory — enough momentum to cross flats
     * without jittering, not enough to climb. */
    float x, y;          /* position in cell space          */
    float dx, dy;        /* unit heading (steered downhill) */

    /* ── the load it transports ───────────────────────────────────────────
     * Carrying capacity each step:  cap = max(−dh · speed · water ·
     * SEDIMENT_CAPACITY_K, MIN_CAPACITY).  sediment < cap on a descent →
     * ERODE (take min(cap−sediment)·ERODE_RATE, −dh); sediment > cap or going
     * uphill → DEPOSIT. speed grows on descents (v² += −dh·GRAVITY), boosting
     * capacity; water starts at INITIAL_WATER and decays ×(1−EVAPORATE_RATE)
     * each step, steadily shrinking capacity so the droplet must finally drop
     * its load. */
    float speed;         /* kinetic energy → carrying capacity */
    float water;         /* evaporates along the path          */
    float sediment;      /* mass currently suspended           */
} Droplet;

/* Drop `amount` of sediment back onto the bed under a droplet at fractional
 * (x,y), bilinearly split across the four corner cells — the inverse of the
 * brush bite, so a slowing droplet lays its load down smoothly. */
static void deposit_sediment(Heightmap *hm, float x, float y, float amount)
{
    int xi = (int)x, yi = (int)y;
    float fx = x - (float)xi, fy = y - (float)yi;
    int idx = hidx(hm, xi, yi);
    hm->height[idx]              += amount * (1.0f - fx) * (1.0f - fy);
    hm->height[idx + 1]          += amount *         fx  * (1.0f - fy);
    hm->height[idx + hm->w]      += amount * (1.0f - fx) *         fy;
    hm->height[idx + hm->w + 1]  += amount *         fx  *         fy;
}

/* Walk one droplet across the bed for up to DROPLET_MAX_STEPS, eroding and
 * depositing as it goes. Reads top-to-bottom as the per-step algorithm; the
 * physics of each step lives in the named helpers (bed_sample,
 * sediment_capacity, erode_brush, deposit_sediment). */
static void droplet_simulate(Heightmap *hm, Droplet *d)
{
    for (int step = 0; step < DROPLET_MAX_STEPS; step++) {
        /* Stop if the droplet (or its next cell) would leave the grid. */
        int xi = (int)d->x, yi = (int)d->y;
        if (xi < 0 || xi >= hm->w - 1 || yi < 0 || yi >= hm->h - 1) return;

        /* Mark the visited cell for the cosmetic water trail (§5). */
        hm->water_trail[hidx(hm, xi, yi)] = 1.0f;

        /* Sample the bed here: height under the droplet + downhill slope. */
        float gx, gy;
        float old_h = bed_sample(hm, d->x, d->y, &gx, &gy);

        /* Steer: blend old heading with −gradient (INERTIA), renormalise. */
        d->dx = d->dx * INERTIA - gx * (1.0f - INERTIA);
        d->dy = d->dy * INERTIA - gy * (1.0f - INERTIA);
        float len = sqrtf(d->dx * d->dx + d->dy * d->dy);
        if (len < DROPLET_STALL_LEN) return;        /* stalled on a flat */
        d->dx /= len; d->dy /= len;

        /* Take a unit step; stop if it lands off-grid. */
        float new_x = d->x + d->dx;
        float new_y = d->y + d->dy;
        int new_xi = (int)new_x, new_yi = (int)new_y;
        if (new_xi < 0 || new_xi >= hm->w - 1 ||
            new_yi < 0 || new_yi >= hm->h - 1) return;

        /* Height change over the step decides erode vs deposit. */
        float dh  = h_bilinear(hm, new_x, new_y) - old_h;
        float cap = sediment_capacity(dh, d->speed, d->water);

        if (d->sediment > cap || dh > 0.0f) {
            /* DEPOSIT: fill an uphill rise (≤ dh), or shed the excess above
             * capacity at DEPOSIT_RATE. */
            float amount = (dh > 0.0f)
                         ? (dh < d->sediment ? dh : d->sediment)
                         : (d->sediment - cap) * DEPOSIT_RATE;
            d->sediment -= amount;
            deposit_sediment(hm, d->x, d->y, amount);
        } else {
            /* ERODE: take the capacity deficit at ERODE_RATE, capped at the
             * drop so the brush never digs below the step it is descending. */
            float wanted = (cap - d->sediment) * ERODE_RATE;
            float amount = (wanted < -dh) ? wanted : -dh;
            erode_brush(hm, d->x, d->y, amount);
            d->sediment += amount;
        }

        /* Energetics: speed gains kinetic energy on descents, water evaporates. */
        float v_sq = d->speed * d->speed - dh * GRAVITY;
        d->speed = (v_sq > 0.0f) ? sqrtf(v_sq) : 0.0f;
        d->water *= (1.0f - EVAPORATE_RATE);

        d->x = new_x;
        d->y = new_y;
    }
}

/*
 * droplet_spawn — pick a random in-bounds cell and initialise. Uses
 * rand() so different droplets land in different places (rand() is
 * srand()-seeded once in main from the wall clock).
 */
static void droplet_spawn(Droplet *d, int w, int h)
{
    d->x = 1.0f + (float)(rand() % (w - 2));
    d->y = 1.0f + (float)(rand() % (h - 2));
    d->dx = 0.0f;  d->dy = 0.0f;
    d->speed    = INITIAL_SPEED;
    d->water    = INITIAL_WATER;
    d->sediment = 0.0f;
}

/* ── Scene ─────────────────────────────────────────────────────────────── *
 * WHAT  The whole simulated-and-shown world: one eroding Heightmap plus the
 *       few knobs and clocks that drive and time it. Reads top-to-bottom like
 *       a table of contents (WHAT / HOW / view / WHEN). The loose scalars are
 *       each too thin to deserve their own type, so they live here grouped by
 *       the CONCEPT they belong to — not by which key toggles them (a colour
 *       theme is a RENDER concept even though a key flips it, NOT a sim knob).
 *
 * PHASE MACHINE  hm.droplets_done + hold_countdown encode a 3-state cycle, run
 *       once per tick by scene_tick(): ERODING (spawn droplets) → SETTLED
 *       (budget reached → arm HOLD) → HOLDING (count down) → regenerate. This
 *       is why the two counters live one level apart: progress is a property
 *       of the terrain (Heightmap), the hold timer is a property of the run. */
typedef struct {
    /* WHAT is simulated — the domain object. */
    Heightmap hm;

    /* HOW the user drives it — the one tunable simulation knob.
     * Spawn rate = DROPLETS_PER_TICK_DEF · speed / SPEED_DEF (clamped ≥ 1);
     * speed ∈ [SPEED_MIN..SPEED_MAX] = 1..64, doubled/halved by +/−. Trades
     * watchability (speed 1: follow one trail) for throughput (speed 64). */
    int       speed;            /* droplets spawned per tick (scaled)        */

    /* WHAT we are looking at — render selections (RENDER, not simulation).
     * Changing either never touches physics: same heightmap, different read. */
    Pattern   current_pattern;  /* which of the 15 views is drawn (n/p)      */
    int       current_theme;    /* index into themes[] (t/T), 0..N_THEMES-1  */

    /* WHEN we are — run clock + state (the DELAYS concept, §6, woven into §4).
     * time_secs accumulates dt every tick: it both phases all animated views
     * and is hashed into the next seed at regen, so each world differs.
     * hold_countdown is armed to HOLD_TICKS (~6 s) when erosion completes. */
    float     time_secs;        /* simulated seconds; animation + reseed     */
    int       hold_countdown;   /* ticks left in the post-erosion HOLD       */
    bool      paused;           /* simulation frozen; rendering continues    */
} Scene;

static void scene_rebuild(Scene *s)
{
    int seed = (int)hash3((int)(s->time_secs * 1000.0f),
                          s->hm.w, s->hm.h ^ 0x9E3779B9);
    heightmap_generate(&s->hm, seed);
    s->hold_countdown = 0;
}

static void scene_init(Scene *s, int map_w, int map_h)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_TERRAIN;
    s->hm.w = map_w;
    s->hm.h = map_h;
    scene_rebuild(s);
}

static void scene_resize_to(Scene *s, int map_w, int map_h)
{
    s->hm.w = map_w;
    s->hm.h = map_h;
    scene_rebuild(s);
}

/* Fade every cell's water trail one tick toward zero (EFFECTS, §5). Runs
 * whether or not we are still eroding, so trails keep decaying after the
 * last droplet lands. */
static void decay_water_trails(Heightmap *hm)
{
    int total = hm->w * hm->h;
    for (int i = 0; i < total; i++)
        hm->water_trail[i] *= WATER_TRAIL_DECAY;
}

/* Spawn and run one tick's worth of droplets. Count scales linearly with the
 * speed knob (SPEED_DEF=8 → 12/tick → ~720/s → ~11 s to spend the budget),
 * and the loop also stops the instant the per-generation budget is hit. */
static void run_erosion_batch(Heightmap *hm, int speed)
{
    int per_tick = DROPLETS_PER_TICK_DEF * speed / SPEED_DEF;
    if (per_tick < 1) per_tick = 1;
    for (int i = 0; i < per_tick && !erosion_complete(hm); i++) {
        Droplet d;
        droplet_spawn(&d, hm->w, hm->h);
        droplet_simulate(hm, &d);
        hm->droplets_done++;
    }
}

/* One simulation step = the scene phase machine (§6 DELAYS woven in):
 *   ERODING  → spawn a batch; when the budget is spent, arm the HOLD.
 *   HOLDING  → count the HOLD down.
 *   ELAPSED  → regenerate a fresh terrain and start over.
 * Paused freezes the machine but not the wall clock. */
static void scene_tick(Scene *s, float dt)
{
    s->time_secs += dt;
    if (s->paused) return;

    decay_water_trails(&s->hm);

    if (!erosion_complete(&s->hm)) {
        run_erosion_batch(&s->hm, s->speed);
        if (erosion_complete(&s->hm))
            s->hold_countdown = HOLD_TICKS;     /* just settled → hold */
    } else if (s->hold_countdown > 0) {
        s->hold_countdown--;
    } else {
        scene_rebuild(s);
    }
}

/* ===================================================================== *
 * §5  EFFECTS  -- cosmetic-only state                                   *
 * ===================================================================== */

/* Nothing of its own to put here. The only purely-cosmetic state is the
 * fading droplet trails (water_trail[]): they're written and faded back in
 * §4 and only read by render_droplets(). Every other glow — twinkling peaks,
 * plasma, caustics — is computed fresh while drawing, never stored. */

/* ===================================================================== *
 * §6  DELAYS  -- pauses, holds, timers                                  *
 * ===================================================================== */

/* Nothing of its own here either. The pause between finishing one terrain
 * and growing the next is a single counter (Scene.hold_countdown), ticked
 * down inside scene_tick() in §4. Pause works by early-returning from the
 * same function. */

/* ===================================================================== *
 * §7  RENDER  -- state -> screen (reads only, never mutates sim)        *
 * ===================================================================== */

/* Turns the current state into characters on screen. Only writes to the
 * ncurses buffer (and the colour table at startup); never changes the
 * terrain, so dropping or redrawing a frame is always safe. */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < N_BIOMES; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], -1);
        init_pair(PAIR_HOT,   t->hot,    -1);
        init_pair(PAIR_COLD,  t->cold,   -1);
        init_pair(PAIR_WATER, 51, -1);          /* bright cyan      */
    } else {
        static const short fb[N_BIOMES] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_GREEN,
            COLOR_GREEN, COLOR_YELLOW,COLOR_YELLOW, COLOR_WHITE,
        };
        for (int i = 0; i < N_BIOMES; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_HOT,   COLOR_RED,  -1);
        init_pair(PAIR_COLD,  COLOR_BLUE, -1);
        init_pair(PAIR_WATER, COLOR_CYAN, -1);
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
 * WHAT  The terminal viewport: its full character size, the heightmap
 *       rectangle chosen to fit inside it, and the top-left offset where that
 *       rectangle is centred. Pure presentation geometry — holds NO simulation
 *       state — recomputed by screen_layout() at startup and on every SIGWINCH.
 *
 * WHY  This is the project's ONE coordinate bridge: map cell (x,y) draws at
 *       terminal (gy0 + y, gx0 + x), computed in exactly one place so no
 *       renderer hand-rolls the offset. map_w/map_h are clamped to [16..MAP_W_MAX] ×
 *       [8..MAP_H_MAX]: the upper bound stops a huge terminal from indexing
 *       past the static CELLS_MAX store; the lower keeps a tiny one usable.
 *       Rows 0–1 are the HUD and the last row the hint line, so gy0 starts at
 *       row 2 and the field is inset to avoid overwriting them. */
typedef struct {
    int cols, rows;      /* full terminal size (getmaxyx)        */
    int map_w, map_h;    /* heightmap rectangle that fits inside */
    int gx0, gy0;        /* its top-left origin on screen        */
} Screen;

static void screen_layout(Screen *s)
{
    int top = HUD_TOP_ROWS, bottom = s->rows - 1;   /* HUD top, hint bottom */
    int avail_h = bottom - top;
    int avail_w = s->cols;

    int mw = avail_w;
    int mh = avail_h;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;

    s->map_w = mw;
    s->map_h = mh;
    s->gx0 = (avail_w - mw) / 2;
    s->gy0 = top + (avail_h - mh) / 2;
    if (s->gx0 < 0)   s->gx0 = 0;
    if (s->gy0 < top) s->gy0 = top;
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

/* ----------------------------------------------------------------------- *
 * Pattern renderers.                                                       *
 * ----------------------------------------------------------------------- */

/* Terrain — eroded heightmap as a biome map. Peaks twinkle. */
static void render_terrain(const Screen *sc, const Heightmap *hm, float t)
{
    int twinkle_t = (int)t;

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            int   bi  = height_to_biome(e);
            uint32_t hh = hash3(x, y, hm->seed);
            char  glyph;
            int   pair = PAIR_RAMP_BASE + bi;
            int   attr = A_NORMAL;

            if (bi <= BIOME_OCEAN) {
                /* Slow water shimmer on ocean cells. */
                static const char waves[4] = { '~', '_', '~', ',' };
                int phase = (x + y * 2 + (int)(t * 3.0f)) & 3;
                glyph = waves[phase];
                if (bi == BIOME_DEEP_OCEAN) attr = A_DIM;
            } else {
                glyph = BIOME_GLYPHS[bi][hh & 1];
            }

            if (bi == BIOME_PEAKS) {
                attr = A_BOLD;
                if ((hash3(x, y, twinkle_t) % 50u) == 0u) attr |= A_BOLD;
            } else if (bi >= BIOME_HIGHLANDS) {
                attr = A_BOLD;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Droplets — terrain dimmed + bright water trails on top. */
static void render_droplets(const Screen *sc, const Heightmap *hm)
{

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float trail = hm->water_trail[hidx(hm, x, y)];
            float e     = hm->height[hidx(hm, x, y)];
            int   bi    = height_to_biome(e);
            uint32_t hh = hash3(x, y, hm->seed);
            char  glyph;
            int   pair;
            int   attr;

            if (trail > WATER_TRAIL_HIGH) {
                glyph = '~';
                pair  = PAIR_WATER;
                attr  = A_BOLD;
            } else if (trail > WATER_TRAIL_MID) {
                glyph = '~';
                pair  = PAIR_WATER;
                attr  = A_NORMAL;
            } else if (trail > 0.05f) {
                glyph = '.';
                pair  = PAIR_WATER;
                attr  = A_DIM;
            } else {
                /* Backdrop biome glyph, dimmed so trails pop. */
                glyph = BIOME_GLYPHS[bi][hh & 1];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Erosion — cut/fill diff vs initial heightmap. */
static void render_erosion(const Screen *sc, const Heightmap *hm)
{

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            int idx = hidx(hm, x, y);
            float diff = hm->height[idx] - hm->initial[idx];
            char  glyph;
            int   pair;
            int   attr;

            if (diff < -EROSION_THRESH_HIGH) {
                glyph = '-';  pair = PAIR_HOT;  attr = A_BOLD;
            } else if (diff < -EROSION_THRESH_LOW) {
                glyph = '-';  pair = PAIR_HOT;  attr = A_NORMAL;
            } else if (diff > +EROSION_THRESH_HIGH) {
                glyph = '+';  pair = PAIR_COLD; attr = A_BOLD;
            } else if (diff > +EROSION_THRESH_LOW) {
                glyph = '+';  pair = PAIR_COLD; attr = A_NORMAL;
            } else {
                /* Unchanged — dim biome backdrop. */
                int bi = height_to_biome(hm->height[idx]);
                uint32_t hh = hash3(x, y, hm->seed);
                glyph = BIOME_GLYPHS[bi][hh & 1];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Slope — gradient magnitude as a heatmap; arrow glyph for direction
 * of -gradient (downhill = the way water flows). */
static void render_slope(const Screen *sc, const Heightmap *hm)
{

    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float gx, gy;
            cell_gradient(hm, x, y, &gx, &gy);
            float mag = sqrtf(gx * gx + gy * gy);
            int level = (int)(mag * SLOPE_SCALE);
            if (level < 0)         level = 0;
            if (level >= N_BIOMES) level = N_BIOMES - 1;

            char glyph = (mag < 1e-4f) ? '.' : flow_arrow(gx, gy);

            int pair = PAIR_RAMP_BASE + level;
            int attr = (level >= 6) ? A_BOLD
                     : (level <= 1) ? A_DIM
                     : A_NORMAL;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Relief — colour by elevation (biome ramp), brightness by hillshade.
 * Reads as a lit 3-D surface. */
static void render_relief(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float sh  = cell_shade(hm, x, y);
            int   bi  = height_to_biome(hm->height[hidx(hm, x, y)]);
            char  glyph = ELEV_RAMP[shade_to_level(sh)];
            int   pair  = PAIR_RAMP_BASE + bi;
            int   attr  = sh > 0.70f ? A_BOLD : sh < 0.22f ? A_DIM : A_NORMAL;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Contour — bright isolines where the quantised height band changes;
 * line glyph orients perpendicular to the gradient. Ocean faint, land
 * between lines left blank for a clean topographic-sheet look. */
static void render_contour(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = h_at(hm, x, y);
            int   lv  = (int)(e * CONTOUR_LEVELS);
            int   lvr = (int)(h_at(hm, x + 1, y) * CONTOUR_LEVELS);
            int   lvd = (int)(h_at(hm, x, y + 1) * CONTOUR_LEVELS);
            char  glyph;
            int   pair, attr;

            if (lv != lvr || lv != lvd) {
                float gx, gy;
                cell_gradient(hm, x, y, &gx, &gy);
                glyph = contour_glyph(gx, gy);
                int bi = lv * N_BIOMES / (CONTOUR_LEVELS + 1);
                if (bi > 7) bi = 7;
                pair = PAIR_RAMP_BASE + bi;
                attr = A_BOLD;
            } else if (height_to_biome(e) <= BIOME_OCEAN) {
                glyph = '~';
                pair  = PAIR_RAMP_BASE + height_to_biome(e);
                attr  = A_DIM;
            } else {
                continue;       /* land between lines: leave blank */
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Hypso — solid filled hypsometric tint: every cell a bold block in its
 * elevation colour. The clean "painted map" complement to TERRAIN. */
static void render_hypso(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            int  bi    = height_to_biome(hm->height[hidx(hm, x, y)]);
            char glyph = (bi <= BIOME_OCEAN) ? '~' : '#';
            int  pair  = PAIR_RAMP_BASE + bi;

            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Aspect — which compass direction each slope faces. Downhill bearing is
 * binned into 8 sectors, each a distinct ramp hue + arrow glyph. */
static void render_aspect(const Screen *sc, const Heightmap *hm)
{
    static const char dirg[8] = { '>', '/', '^', '\\', '<', '/', 'v', '\\' };
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float gx, gy;
            cell_gradient(hm, x, y, &gx, &gy);
            float mag = sqrtf(gx * gx + gy * gy);
            char  glyph;
            int   pair, attr;

            if (mag < 5e-4f) {
                glyph = '.';  pair = PAIR_RAMP_BASE;  attr = A_DIM;
            } else {
                int sect = downhill_sector(gx, gy);
                glyph = dirg[sect];
                pair  = PAIR_RAMP_BASE + sect;
                attr  = mag > 0.02f ? A_BOLD : A_NORMAL;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Ridges — discrete Laplacian skeleton: strongly convex cells are ridge
 * crests (warm '^'), strongly concave are valleys (cool 'v'), the rest a
 * dim biome backdrop. Exposes the drainage structure. */
static void render_ridges(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            float lap = cell_laplacian(hm, x, y);
            char  glyph;
            int   pair, attr;

            if (lap < -RIDGE_THRESH) {
                glyph = '^';  pair = PAIR_HOT;   attr = A_BOLD;
            } else if (lap > RIDGE_THRESH) {
                glyph = 'v';  pair = PAIR_COLD;  attr = A_NORMAL;
            } else {
                int bi = height_to_biome(e);
                glyph = BIOME_GLYPHS[bi][hash3(x, y, hm->seed) & 1];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Canyons — cells the droplets carved below their initial height glow as
 * cyan river arrows (deeper cut = brighter); untouched land is a dim 3-D
 * hillshade so the channel network stands out in relief. */
static void render_canyons(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            int   idx  = hidx(hm, x, y);
            float diff = hm->height[idx] - hm->initial[idx];
            char  glyph;
            int   pair, attr;

            if (diff < -EROSION_THRESH_LOW) {
                float gx, gy;
                cell_gradient(hm, x, y, &gx, &gy);
                glyph = flow_arrow(gx, gy);
                pair = PAIR_WATER;
                attr = (diff < -EROSION_THRESH_HIGH) ? A_BOLD : A_NORMAL;
            } else {
                glyph = ELEV_RAMP[shade_to_level(cell_shade(hm, x, y))];
                pair  = PAIR_RAMP_BASE + height_to_biome(hm->height[idx]);
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Bathy — underwater focus: shallows-to-deep ocean ramp with animated
 * caustic shimmer; land is a calm dim hillshade so the eye stays on the
 * living sea. */
static void render_bathy(const Screen *sc, const Heightmap *hm, float t)
{
    static const char cg[4] = { '.', '~', '-', '=' };
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e  = hm->height[hidx(hm, x, y)];
            int   bi = height_to_biome(e);
            char  glyph;
            int   pair, attr;

            if (bi <= BIOME_COAST) {
                float caustic = sinf(x * 0.35f + y * 0.22f + t * 2.3f)
                              + sinf(x * 0.13f - y * 0.31f + t * 1.7f);
                int ci = (int)((caustic + 2.0f) / 4.0f * 3.99f);
                if (ci < 0) ci = 0;
                if (ci > 3) ci = 3;
                int depth = (int)((0.40f - e) * 16.0f);
                if (depth < 0) depth = 0;
                if (depth > 5) depth = 5;
                glyph = cg[ci];
                pair  = PAIR_RAMP_BASE + (5 - depth);   /* shallow=bright */
                attr  = ci >= 3 ? A_BOLD : ci == 0 ? A_DIM : A_NORMAL;
            } else {
                glyph = ELEV_RAMP[shade_to_level(cell_shade(hm, x, y))];
                pair  = PAIR_RAMP_BASE + bi;
                attr  = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Plasma — pure eye-candy: a colour phase advances with elevation, two
 * sine waves and time, cycling the 8 ramp hues so colour bands flow
 * across the terrain shape. */
static void render_plasma(const Screen *sc, const Heightmap *hm, float t)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e = hm->height[hidx(hm, x, y)];
            float v = e * 7.0f + t * 0.8f
                    + sinf(x * 0.10f + t) * 0.8f
                    + cosf(y * 0.13f - t) * 0.8f;
            int idx = (int)v % N_BIOMES;
            if (idx < 0) idx += N_BIOMES;
            char glyph = ELEV_RAMP[idx];
            int  pair  = PAIR_RAMP_BASE + idx;

            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* Night — atmospheric mood: black sea with rare moon glints, dim land,
 * hillshade-lit ridgelines, and snow peaks that glow and twinkle. */
static void render_night(const Screen *sc, const Heightmap *hm, float t)
{
    int glint_t   = (int)(t * 2.0f);
    int twinkle_t = (int)(t * 3.0f);
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            int   bi  = height_to_biome(e);
            char  glyph;
            int   pair, attr;

            if (bi <= BIOME_OCEAN) {
                if ((hash3(x, y, glint_t) % 80u) != 0u) continue;  /* black */
                glyph = '.';  pair = PAIR_RAMP_BASE + bi;  attr = A_DIM;
            } else if (bi >= BIOME_HIGHLANDS) {
                glyph = BIOME_GLYPHS[bi][hash3(x, y, hm->seed) & 1];
                if (bi == BIOME_PEAKS && (hash3(x, y, twinkle_t) % 40u) == 0u)
                    glyph = '*';
                pair = PAIR_RAMP_BASE + 7;  attr = A_BOLD;
            } else if (cell_shade(hm, x, y) > 0.62f) {
                glyph = '^';  pair = PAIR_RAMP_BASE + 5;  attr = A_NORMAL;
            } else {
                glyph = BIOME_GLYPHS[bi][hash3(x, y, hm->seed) & 1];
                pair  = PAIR_RAMP_BASE + bi;  attr = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Flow — animated contours: the same isoline test as CONTOUR but the
 * bands scroll downhill over time, so bright lines appear to cascade
 * from peaks to sea like sheet flow. */
static void render_flow(const Screen *sc, const Heightmap *hm, float t)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e     = h_at(hm, x, y);
            float phase = e * CONTOUR_LEVELS - t * 2.0f;
            float frac  = phase - floorf(phase);
            char  glyph;
            int   pair, attr;

            if (frac < 0.18f) {
                float gx, gy;
                cell_gradient(hm, x, y, &gx, &gy);
                glyph = contour_glyph(gx, gy);
                int bi = (int)(e * 7.99f);
                if (bi > 7) bi = 7;
                pair = PAIR_RAMP_BASE + bi;  attr = A_BOLD;
            } else {
                int bi = height_to_biome(e);
                glyph = (bi <= BIOME_OCEAN) ? '~' : '.';
                pair  = PAIR_RAMP_BASE + bi;  attr = A_DIM;
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Heat — two-tone thermal map: warm accent above mid-elevation, cool
 * below, density glyph by height. A stark cartographer's heat readout. */
static void render_heat(const Screen *sc, const Heightmap *hm)
{
    for (int y = 0; y < hm->h; y++) {
        int sy = sc->gy0 + y;
        if (sy < HUD_TOP_ROWS || sy >= sc->rows - 1) continue;
        for (int x = 0; x < hm->w; x++) {
            int sx = sc->gx0 + x;
            if (sx < 0 || sx >= sc->cols) continue;

            float e   = hm->height[hidx(hm, x, y)];
            int   lvl = (int)(e * 7.99f);
            if (lvl < 0) lvl = 0;
            if (lvl > 7) lvl = 7;
            char glyph = ELEV_RAMP[lvl];
            int  pair  = (e >= 0.5f) ? PAIR_HOT : PAIR_COLD;
            int  attr  = (lvl >= 6 || lvl <= 1) ? A_BOLD : A_NORMAL;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void scene_draw(const Screen *sc, const Heightmap *hm,
                       Pattern pattern, float t)
{
    switch (pattern) {
    case PATTERN_TERRAIN:  render_terrain (sc, hm, t);  break;
    case PATTERN_DROPLETS: render_droplets(sc, hm);     break;
    case PATTERN_EROSION:  render_erosion (sc, hm);     break;
    case PATTERN_SLOPE:    render_slope   (sc, hm);     break;
    case PATTERN_RELIEF:   render_relief  (sc, hm);     break;
    case PATTERN_CONTOUR:  render_contour (sc, hm);     break;
    case PATTERN_HYPSO:    render_hypso   (sc, hm);     break;
    case PATTERN_ASPECT:   render_aspect  (sc, hm);     break;
    case PATTERN_RIDGES:   render_ridges  (sc, hm);     break;
    case PATTERN_CANYONS:  render_canyons (sc, hm);     break;
    case PATTERN_BATHY:    render_bathy   (sc, hm, t);  break;
    case PATTERN_PLASMA:   render_plasma  (sc, hm, t);  break;
    case PATTERN_NIGHT:    render_night   (sc, hm, t);  break;
    case PATTERN_FLOW:     render_flow    (sc, hm, t);  break;
    case PATTERN_HEAT:     render_heat    (sc, hm);     break;
    case N_PATTERNS:       break;       /* unreachable sentinel */
    }
}

/* Draw the 8-swatch colour-ramp legend at (row,x); returns the x past it. */
static int draw_ramp_legend(int row, int x)
{
    for (int i = 0; i < N_BIOMES; i++) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        mvaddch(row, x, (chtype)(unsigned char)ELEV_RAMP[i]);
        attroff(COLOR_PAIR(PAIR_RAMP_BASE + i) | A_BOLD);
        x++;
    }
    return x;
}

/* Row 0 right — primary status: fps, sim Hz, phase, speed. Right-aligned. */
static void draw_status_line(const Screen *sc, const Scene *s,
                             double fps, int sim_fps)
{
    const char *phase = s->paused                 ? "PAUSED  "
                      : erosion_complete(&s->hm)  ? "SETTLED "
                                                  : "ERODING ";
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf, " %5.1f fps  %3d Hz  %s  speed:%-3d ",
             fps, sim_fps, phase, s->speed);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* Row 1 — pattern (with n/N counter), theme, ramp legend, droplet progress.
 * Fixed left-aligned layout, so it needs the Scene but not the Screen. */
static void draw_param_line(const Scene *s)
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

    char tbuf[32];
    snprintf(tbuf, sizeof tbuf, " theme:%-8s ", themes[s->current_theme].name);
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, "%s", tbuf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += (int)strlen(tbuf);

    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " ramp:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 6;
    x = draw_ramp_legend(1, x);

    int pct = (DROPLETS_PER_GEN > 0)
            ? (s->hm.droplets_done * 100) / DROPLETS_PER_GEN : 100;
    if (pct > 100) pct = 100;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  droplets:%5d/%-5d  %3d%% ",
             s->hm.droplets_done, (int)DROPLETS_PER_GEN, pct);
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* Bottom row — the key legend. Lists every interactive key (HUD standard). */
static void draw_hint(const Screen *sc)
{
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  +/-:speed  ]/[:tickHz  spc:pause  r:regen  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* The one render function that takes the whole Scene (read-only): the HUD's
 * concept IS whole-scene status — pattern, theme, speed, progress, run-state.
 * A const read can't re-couple the layers; the leaf renderers stay narrow.
 * Reads as: draw the chosen view, then lay the HUD over it. */
static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(sc, &s->hm, s->current_pattern, s->time_secs);

    draw_status_line(sc, s, fps, sim_fps);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " HYDRAULIC EROSION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    draw_param_line(s);
    draw_hint(sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== *
 * §8  APP  -- events + per-tick combine + main loop                     *
 * ===================================================================== */

/* The glue: signal flags, key handlers, and the main loop that ties it all
 * together. Each frame runs the simulation, draws it, then handles input. */

/* ── App ───────────────────────────────────────────────────────────────── *
 * WHAT  Top-level harness binding the simulation (scene) to the terminal
 *       (screen), plus the loop's PERFORMANCE knob and the async signal flags.
 *       A single static instance (g_app) exists ONLY so the signal handlers —
 *       which may fire between any two instructions — can reach the flags;
 *       everything else passes App explicitly. Just init + the main loop touch
 *       App whole; every other function takes the narrowest sub-type (§5).
 *
 * REFS  Fixed-timestep accumulator — sim_fps decouples the simulation rate
 *       from the render frame rate, so physics is deterministic regardless of
 *       terminal speed (Fiedler, "Fix Your Timestep!"; §8 main()). */
typedef struct {
    /* the two worlds it binds */
    Scene                 scene;        /* WHAT is simulated + shown         */
    Screen                screen;       /* WHERE it is drawn                 */

    /* loop control */
    int                   sim_fps;      /* fixed-timestep rate, SIM_FPS_* Hz */
    /* volatile sig_atomic_t: written from signal handlers, so the compiler
     * must re-read them each loop and the write must be atomic w.r.t. the
     * interrupted code. */
    volatile sig_atomic_t running;      /* cleared by SIGINT/SIGTERM → exit  */
    volatile sig_atomic_t need_resize;  /* set by SIGWINCH, served next loop */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    scene_resize_to(&app->scene, app->screen.map_w, app->screen.map_h);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_rebuild(s);                              break;

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

/* Fixed-timestep catch-up: run as many simulation ticks as the accumulated
 * real time (*sim_accum) allows, each advancing the scene by one fixed dt,
 * draining the accumulator as it goes. Decoupling the sim rate from the frame
 * rate keeps physics deterministic regardless of how fast frames are drawn
 * (Fiedler, "Fix Your Timestep!"). */
static void run_pending_ticks(Scene *s, int64_t *sim_accum,
                              int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(s, dt_sec);
        *sim_accum -= tick_ns;
    }
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
    scene_init(&app->scene, app->screen.map_w, app->screen.map_h);

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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* spiral-of-death guard */

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        run_pending_ticks(&app->scene, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
