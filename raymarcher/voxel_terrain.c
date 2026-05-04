/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * voxel_terrain.c — top-down heightmap viewer with hill-shading + sun shadows.
 *
 * DEMO: Render a 256×256 fBm heightmap from straight overhead — a
 *       living topographic map.  Each terminal cell samples ONE world
 *       (x, z) position; the cell's COLOUR comes from the height tier
 *       at that point (low → high), and its BRIGHTNESS comes from a
 *       Lambertian slope-vs-sun shading combined with a precomputed
 *       sun shadowmap (cells in cast shadow darken).  The map drifts
 *       slowly; arrow keys pan around; +/- zoom in/out.  The
 *       heightmap is toroidally tiled so panning never finds an edge.
 *
 *       Distinct from a first-person voxel-space (Comanche) flythrough:
 *       there is no perspective, no march, no horizon — just one
 *       heightmap lookup per terminal cell, plus a four-sample
 *       gradient for hill shading.  The result reads instantly as a
 *       topographic map at any size from 60×20 upward.
 *
 *       Themes (cycle with t / T):
 *         CLASSIC   forest greens climbing through bare to snow
 *         DESERT    sandy browns through tan into bone
 *         SNOW      cold blue lowlands climbing into bright white peaks
 *         VOLCANIC  charred crimson through ember orange to ash
 *         ALIEN     violet bedrock through magenta into cyan haze
 *         SUNSET    warm umber + amber into pale gold
 *
 * Study alongside:
 *   procedural/perlin_landscape.c — same fBm field; rendered as a
 *                                STATIC map.  voxel_terrain.c is
 *                                that same data with sun-shaded relief
 *                                and panning camera.
 *   particle_systems/snow.c       — completely different demo, but
 *                                shares the "render the same data
 *                                from different camera" idea.
 *
 * Section map:
 *   §1 config    — constants, themes
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair height ramp
 *   §4 noise     — hash-based value noise + multi-octave fBm
 *   §5 terrain   — heightmap regen, sample, sun shadowmap precompute
 *   §6 scene     — top-down pan/zoom camera, tick (drift), render
 *   §7 screen    — ncurses init / draw / resize
 *   §8 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume drift
 *   r            reseed terrain
 *   n / N        next / previous terrain TYPE
 *                  (HILLS / MOUNTAINS / PLAINS / CANYONS / ARCHIPELAGO / MESA)
 *   t / T        next / previous theme
 *   ← / →        pan west / east
 *   ↑ / ↓        pan north / south
 *   + / =        zoom in   (smaller world per cell, more detail)
 *   -            zoom out  (more terrain visible)
 *   s            cycle sun direction (rebuilds shadow map)
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/voxel_terrain.c \
 *       -o voxel_terrain -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Direct top-down heightmap rasterisation with
 *                  Lambertian hill shading + precomputed sun shadow.
 *
 *                  Per terminal cell (col, row):
 *
 *                     wx = pan_x + (col − cols/2) · zoom
 *                     wz = pan_z + (row − rows/2) · zoom · CELL_ASPECT
 *                     h  = terrain_sample(wx, wz)         (bilinear, toroidal)
 *
 *                     // Slope (gradient of heightmap at world point)
 *                     dh_x = (sample(wx+ε, wz) − sample(wx−ε, wz)) · 0.5
 *                     dh_z = (sample(wx, wz+ε) − sample(wx, wz−ε)) · 0.5
 *
 *                     // Surface normal (heightmap is z = h(x, z))
 *                     N = normalise(−dh_x, 1, −dh_z)
 *
 *                     // Lambert (light direction = unit vector toward sun)
 *                     n_dot_sun = N · sun_dir
 *                     light = ambient + (1 − ambient) · max(0, n_dot_sun)
 *                     if shadowed_by_terrain(wx, wz):
 *                         light *= shadow_factor
 *
 *                  Pixel:
 *                     height_slot = ⌊h / MAX_HEIGHT · 7.999⌋   ∈ {0..7}
 *                     glyph       = HEIGHT_GLYPHS[height_slot]
 *                     pair        = PAIR_TERRAIN_BASE + height_slot
 *                     attr        = A_BOLD if light > 0.75
 *                                   A_DIM  if light < 0.45 or shadowed
 *                                   A_NORMAL otherwise
 *
 *                  Why this gives a readable map:
 *                    HEIGHT carries the terrain "type" (water vs grass vs
 *                    rock vs snow) — colour reads as biome.  LIGHT carries
 *                    the 3-D shape — sunlit slopes brighten, shaded slopes
 *                    dim, cast shadows go DIM.  Reading the two together
 *                    gives the same instant geographic intuition as a
 *                    USGS shaded-relief topographic map.
 *
 * Data-structure : Same as the original first-person version —
 *                  heightmap[MAP_N][MAP_N] float, shadowmap[MAP_N][MAP_N]
 *                  byte, both toroidally indexed.  Heightmap samples
 *                  use bilinear interpolation, so panning at fractional
 *                  cell rates produces smooth motion (no jaggy snap).
 *
 * Rendering      : ASCII only.  Eight glyph tiers from low (water-like
 *                  `~`) to high (peak `@`).  The slope shading + colour
 *                  gradient does most of the visual work; the glyph
 *                  reinforces the height tier.
 *
 * Performance    : Per pixel: 1 centre-sample + 4 neighbour samples (for
 *                  the gradient) + 1 shadowmap byte fetch + a handful of
 *                  ops.  Each terrain_sample is 4 array fetches + 2
 *                  lerps.  At 80×24 = 1 920 pixels: ~40 000 array
 *                  fetches per frame.  Trivial — fills 200×60 at 60 fps.
 *
 * References     :
 *   • Yoëli, P. (1965) — "Analytical hill shading", *Surveying and
 *     Mapping* 25(4):573–584.  The Lambert-cosine hill shading we use.
 *   • Imhof, E. — *Cartographic Relief Presentation* (1965).  The
 *     foundational text on shaded-relief topographic maps; the
 *     "north-west sun, oblique" convention we follow here is from §IV.
 *   • Mandelbrot, B. — *The Fractal Geometry of Nature* (1982), §28.
 *     Multi-octave fBm for landscape generation.
 *   • Quílez, I. — [Terrain Raymarching](https://iquilezles.org/articles/terrainmarching/).
 *     The first-person counterpart of this file; useful for the
 *     contrast between projection-based and direct rendering.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Look at the landscape from straight above as if it's a photograph
 * taken from a satellite.  Every terminal cell is one square patch of
 * ground; its colour says "how high" (water/grass/snow), its
 * brightness says "is this slope facing the sun, or in shade".  Pan
 * the camera to look at different parts; zoom to widen or narrow the
 * coverage.  No 3-D camera, no perspective, no march loop — just a
 * direct lookup per cell, with a 5-sample stencil for hill shading.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a giant heightmap printed on graph paper, with one number
 * per square telling you the elevation there.  Now imagine pointing a
 * lamp at it from a fixed direction.  For each square, you ask "how
 * tilted is this patch and which way?".  If the patch tilts toward
 * the lamp it gets bright; if it tilts away it gets dim.  If a taller
 * patch over THERE is between this patch and the lamp, this patch is
 * in cast shadow and goes extra-dim.  Finally, colour each square
 * according to its elevation tier (blue for low, white for peak).
 * That's the entire rendering pipeline.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. STARTUP — generate heightmap from 4-octave value-noise fBm,
 *     biased with `pow(h, 1.6)` for sharper peaks.  Generate shadow
 *     map by marching each cell toward the sun and flagging "blocked".
 *
 *  2. PER FRAME, PER CELL (col, row):
 *
 *        wx = pan_x + (col − cols/2) · zoom
 *        wz = pan_z + (row − rows/2) · zoom · CELL_ASPECT
 *        h  = bilinear sample heightmap[wx, wz]
 *
 *        // 4-tap gradient (slope of the heightmap)
 *        dh_x = (sample(wx+1, wz) − sample(wx−1, wz)) · 0.5
 *        dh_z = (sample(wx, wz+1) − sample(wx, wz−1)) · 0.5
 *
 *        // Surface normal (graph z = h(x, z))
 *        N = (−dh_x, 1, −dh_z) / |N|
 *
 *        // Lambert
 *        n_dot_sun = N · sun_dir_world
 *        n_dot_sun = max(0, n_dot_sun)
 *        light     = AMBIENT + (1 − AMBIENT) · n_dot_sun
 *        if shadow_at(wx, wz):
 *            light *= SHADOW_FACTOR             (~0.4)
 *
 *  3. PAINT:
 *        height_slot  = ⌊h / MAX_HEIGHT · 7.999⌋
 *        glyph        = HEIGHT_GLYPHS[height_slot]
 *        pair         = PAIR_TERRAIN_BASE + height_slot
 *        if shadow_at OR light < 0.45 → A_DIM
 *        else if light > 0.75         → A_BOLD
 *        else                         → A_NORMAL
 *
 *  4. TICK — `pan_x += drift_x · dt; pan_z += drift_z · dt;` so the
 *     map slowly scrolls under the viewer.  Pause freezes both drift
 *     and any wind/animation; arrows still pan in pause.
 *
 *  5. RESIZE / RESEED / SUN-CYCLE — rebuild heightmap and/or shadowmap
 *     as appropriate; ~10 ms each, hidden by the next frame.
 *
 * KEY FORMULAS
 * ────────────
 *  Camera mapping (top-down):
 *    wx = pan_x + (col − cols/2) · zoom
 *    wz = pan_z + (row − rows/2) · zoom · CELL_ASPECT
 *
 *  Height-map gradient (central differences, 1-cell stencil):
 *    dh/dx ≈ (h(x+1, z) − h(x−1, z)) / 2
 *    dh/dz ≈ (h(x, z+1) − h(x, z−1)) / 2
 *
 *  Surface normal (heightmap parameterised as y = h(x, z)):
 *    N = normalise(−dh/dx,  1,  −dh/dz)
 *
 *  Sun direction in world coordinates:
 *    sun = (cos(yaw)·cos(pitch),  sin(pitch),  sin(yaw)·cos(pitch))
 *
 *  Lambertian shading with cast shadow:
 *    n_dot_sun = max(0, N · sun)
 *    light     = AMBIENT + (1 − AMBIENT) · n_dot_sun
 *    if shadowed:  light *= SHADOW_FACTOR
 *
 *  Bilinear toroidal heightmap sample:
 *    xi = ⌊wx⌋ & MAP_MASK,  zi = ⌊wz⌋ & MAP_MASK
 *    fx = wx − ⌊wx⌋,        fz = wz − ⌊wz⌋
 *    return bilerp(h[xi,zi], h[xi+1,zi], h[xi,zi+1], h[xi+1,zi+1])
 *
 *  Aspect correction (terminal cells 2× taller than wide):
 *    wz_per_row = zoom · 2.0
 *    so a square world patch projects to a square screen region.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • PURE-FLAT GROUND.  When dh_x = dh_z = 0, normal = (0, 1, 0); the
 *    Lambert is just sun.y = sin(sun_pitch).  All flat terrain
 *    renders the same brightness regardless of orientation.  This is
 *    correct (flat ground IS uniformly lit by an overhead-ish sun)
 *    but uniform regions can look "dead".  The shadowmap brings
 *    them back to life when neighbour peaks cast shadows over them.
 *
 *  • EPS = 1.0 GRADIENT.  Sampling neighbours one heightmap-cell away
 *    works at any zoom because the heightmap is bilinearly sampled —
 *    even when zoom < 1 (sub-cell rendering), the gradient is still
 *    well-defined.
 *
 *  • TOROIDAL WRAP.  pan_x and pan_z grow without bound; the heightmap
 *    LOOKUP wraps via `& MAP_MASK`.  Long pans never find an edge.
 *
 *  • CELL_ASPECT.  Terminal cells are ~2× taller than wide.  Without
 *    multiplying wz_per_row by CELL_ASPECT, the map renders squashed
 *    vertically (1° latitude looks like 0.5° to the eye).
 *
 *  • SUN DIRECTION CHANGE.  Rebuilding shadowmap takes ~10 ms.  We
 *    only rebuild on `s` (cycle sun) or `r` (reseed terrain) — not
 *    every frame.  If you want time-of-day animation, recompute every
 *    N seconds, not every frame.
 *
 *  • ZOOM NEAR ZERO.  zoom < ZOOM_MIN clamps to ZOOM_MIN to avoid
 *    division-by-zero-ish degeneracy in the gradient stencil; below
 *    ~0.05 the visible region is smaller than a single heightmap cell
 *    and looks like a single colour.
 *
 *  • PAUSE.  Freezes auto-drift but NOT the arrow-key pan — useful
 *    for "stop the auto motion, look around manually".
 *
 *  • BIG PAN STEP AT HIGH ZOOM.  Pan step is `PAN_STEP_COLS · zoom`
 *    so each keypress moves a constant fraction of the visible
 *    screen, regardless of zoom level.  Keeps pan feel consistent.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At default zoom you should see a topographic-map-style picture:
 *    bands of colour following elevation, hill-shaded with sun from
 *    one side.
 *
 *  • Press space.  Drift stops.  Map holds still.  Arrow keys still
 *    pan.  Resume — drift picks up where it stopped.
 *
 *  • Press `s`.  Sun direction cycles; the LIGHT side and SHADOW side
 *    of every hill flip.  Brief ~10 ms freeze during the rebuild.
 *
 *  • Press `r`.  New noise seed → entirely different landscape; same
 *    map starting position, different elevation pattern.
 *
 *  • Press `+` to zoom in.  Each cell now covers a smaller world
 *    region; you see finer height detail.  Press `-` to zoom out;
 *    eventually the toroidal wrap shows the same map repeating.
 *
 *  • Cycle themes (`t`/`T`).  Geometry identical, only the elevation
 *    palette changes.  CLASSIC → DESERT is the most striking flip.
 *
 *  • If the hill shading looks WRONG (slopes facing INTO the sun
 *    looking DARK), you have a sign error in the normal.  N's y
 *    component must be positive (pointing UP, not DOWN).
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
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,

    /* Power-of-two so we can use bitwise AND for the toroidal wrap. */
    MAP_N            = 256,
    MAP_MASK         = MAP_N - 1,

    SHADOW_STEPS     =  32,

    FPS_UPDATE_MS    = 500,

    /* Color pair indices. */
    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_TERRAIN_BASE =  3,    /* +0..+7 — height ramp                 */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Terrain — global height-tier scale.  Per-terrain params live in
 * the `terrains[]` table below; they all rescale into [0, MAX_HEIGHT]
 * so the renderer's tier mapping is uniform across types. */
#define MAX_HEIGHT       28.0f      /* peak terrain altitude              */

/* Top-down camera. */
#define ZOOM_DEFAULT     1.5f       /* world units per terminal column    */
#define ZOOM_MIN         0.20f
#define ZOOM_MAX         8.0f
#define ZOOM_STEP        1.25f      /* multiplicative zoom factor         */
#define PAN_STEP_COLS    6.0f       /* world units per ←/→ keypress       */
#define PAN_STEP_ROWS    3.0f       /* world units per ↑/↓ keypress       */
#define DRIFT_X          1.5f       /* auto-pan east  (units / sec)       */
#define DRIFT_Z          0.7f       /* auto-pan south (units / sec)       */

/* Hill shading. */
#define AMBIENT          0.30f      /* min lighting on shadow side        */
#define SHADOW_FACTOR    0.50f      /* darken cells in cast shadow        */
#define BRIGHT_THRESHOLD 0.78f      /* above → A_BOLD                     */
#define DIM_THRESHOLD    0.45f      /* below → A_DIM                      */

/* Sun direction presets — yaw, pitch in radians.  Cycle with `s`.
 *
 * Top-down maps look best with sun yaw at oblique angles (NE / SE / SW
 * / NW) and pitch around 30–45° — peak shadows reveal terrain shape
 * dramatically without the whole map going dark. */
#define N_SUN_DIRS  4
static const float g_sun_dirs[N_SUN_DIRS][2] = {
    /* yaw     pitch */
    {  0.79f, 0.55f },   /* NE,   medium-high      */
    {  2.36f, 0.50f },   /* SW,   medium           */
    { -0.79f, 0.40f },   /* NW,   low (long shadow)*/
    { -2.36f, 0.65f },   /* SE,   high             */
};

/* Theme enum is just the palette. */
typedef enum {
    THEME_CLASSIC  = 0,
    THEME_DESERT   = 1,
    THEME_SNOW     = 2,
    THEME_VOLCANIC = 3,
    THEME_ALIEN    = 4,
    THEME_SUNSET   = 5,
    N_THEMES       = 6,
} Theme;

static const char *theme_name(Theme t)
{
    switch (t) {
    case THEME_CLASSIC:  return "CLASSIC ";
    case THEME_DESERT:   return "DESERT  ";
    case THEME_SNOW:     return "SNOW    ";
    case THEME_VOLCANIC: return "VOLCANIC";
    case THEME_ALIEN:    return "ALIEN   ";
    case THEME_SUNSET:   return "SUNSET  ";
    default:             return "?       ";
    }
}

/*
 * Per-theme palette — 8 height tiers, low → high.  All entries sit in
 * the bright half of the 256-colour cube per the CLAUDE.md "Theme
 * Palette Brightness" rule, so even the lowest tier stays visible.
 */
typedef struct {
    short height[8];   /* low → high                                  */
} ThemePalette;

static const ThemePalette themes[N_THEMES] = {
    /* CLASSIC: forest greens climbing through bare rock to snow-cap. */
    { { 28,  34,  70,  76, 107, 137, 144, 195 } },

    /* DESERT: dune browns → sandy tan → bleached pale.               */
    { {130, 137, 173, 179, 215, 222, 230, 195 } },

    /* SNOW: cold blue lowlands climbing into bright white peaks.     */
    { { 24,  67, 110, 152, 195, 251, 254, 255 } },

    /* VOLCANIC: charred crimson + ember orange + cool ash highlights.*/
    { { 52,  88, 124, 166, 208, 215, 245, 250 } },

    /* ALIEN: violet bedrock → magenta meadow → cyan haze.            */
    { { 53,  91, 134, 165, 207, 159, 152, 195 } },

    /* SUNSET: ember-red lowlands climbing through amber to pale gold.*/
    { { 88, 130, 172, 208, 215, 222, 229, 230 } },
};

/* Glyph ramp by height tier: low (water-ish) → high (peaks). */
static const char HEIGHT_GLYPHS[8] = {
    '~', '.', ',', ':', '+', '*', '#', '@'
};

/*
 * Terrain type — different fBm post-processing chains produce visually
 * distinct landscapes from the same noise field.  All chain outputs are
 * normalised into [0, 1] before the final · MAX_HEIGHT step so the
 * renderer's tier bucketing works uniformly across types.
 *
 * The chain knobs:
 *   noise_scale   : domain stretch — small = smoother, large = busier
 *   bias_exp      : pow(h, exp) sharpens peaks (>1) or flattens (<1)
 *   ridge         : if true, fold via 1 − 2·|h − 0.5| → ridge-noise
 *                   (high cells become THIN RIDGES rather than broad
 *                   peaks; produces canyon walls / spire architecture)
 *   water_level   : if > 0, clamp h below this fraction to 0 (water)
 *                   so lowlands flatten into a uniform pool — the
 *                   "archipelago" effect
 *   terrace_steps : if > 0, quantise h into N flat plateaus
 *                   (mesa / step-pyramid look)
 */
typedef enum {
    TERRAIN_HILLS       = 0,
    TERRAIN_MOUNTAINS   = 1,
    TERRAIN_PLAINS      = 2,
    TERRAIN_CANYONS     = 3,
    TERRAIN_ARCHIPELAGO = 4,
    TERRAIN_MESA        = 5,
    N_TERRAINS          = 6,
} TerrainType;

typedef struct {
    const char *name;
    float       noise_scale;
    float       bias_exp;
    bool        ridge;
    float       water_level;     /* fraction of [0,1] clamped to 0     */
    int         terrace_steps;   /* 0 = no terrace                     */
} TerrainParams;

static const TerrainParams terrains[N_TERRAINS] = {
    /*  name              n_scale  bias  ridge water  steps */
    /* HILLS       */   { "HILLS      ", 0.045f, 1.60f, false, 0.00f,  0 },
    /* MOUNTAINS   */   { "MOUNTAINS  ", 0.035f, 2.00f, false, 0.00f,  0 },
    /* PLAINS      */   { "PLAINS     ", 0.030f, 1.00f, false, 0.00f,  0 },
    /* CANYONS     */   { "CANYONS    ", 0.045f, 1.40f, true,  0.00f,  0 },
    /* ARCHIPELAGO */   { "ARCHIPELAGO", 0.040f, 1.50f, false, 0.30f,  0 },
    /* MESA        */   { "MESA       ", 0.045f, 1.30f, false, 0.00f,  5 },
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
        const ThemePalette *t = &themes[idx];
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_TERRAIN_BASE + i), t->height[i], -1);
    } else {
        /* 8-colour fallback. */
        static const short fb[8] = {
            COLOR_BLUE,    COLOR_GREEN,   COLOR_GREEN,   COLOR_YELLOW,
            COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE,   COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_TERRAIN_BASE + i), fb[i], -1);
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

/* ===================================================================== */
/* §4  noise                                                              */
/* ===================================================================== */

static inline uint32_t hash2d(int x, int z, uint32_t seed)
{
    uint32_t h = (uint32_t)x * 374761393u
               + (uint32_t)z * 668265263u
               + seed        * 2147483647u;
    h = (h ^ (h >> 13)) * 1274126177u;
    return h ^ (h >> 16);
}

static inline float hash_unit(int x, int z, uint32_t seed)
{
    return (float)(hash2d(x, z, seed) >> 8) / (float)(1u << 24);
}

static inline float smoothstep01(float t)
{
    return t * t * (3.0f - 2.0f * t);
}

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

static float fbm2d(float x, float z, uint32_t seed)
{
    float h = 0.0f, amp = 1.0f, freq = 1.0f, norm = 0.0f;
    for (int i = 0; i < 4; i++) {
        h    += amp * vnoise2d(x * freq, z * freq, seed + (uint32_t)i * 17u);
        norm += amp;
        amp  *= 0.5f;
        freq *= 2.0f;
    }
    return h / norm;
}

/* ===================================================================== */
/* §5  terrain — heightmap + shadowmap                                    */
/* ===================================================================== */

static float   g_heightmap[MAP_N][MAP_N];
static uint8_t g_shadowmap[MAP_N][MAP_N];

/*
 * terrain_regen — fill g_heightmap using the chosen TerrainType's
 * post-processing chain.
 *
 *   1. fbm2d        → base noise in [0, 1]
 *   2. ridge fold   → 1 − 2·|h − 0.5|  (turns peaks into thin ridges)
 *   3. bias exp     → pow(h, exp)       (sharpens peaks)
 *   4. terracing    → quantise to N flat plateaus
 *   5. water clip   → fraction below water_level → 0
 *   6. · MAX_HEIGHT → final world-space altitude
 *
 * Each step is independent and gated by its TerrainParams flag, so
 * adding new terrain types is a one-row addition to `terrains[]`
 * without touching this loop.
 */
static void terrain_regen(uint32_t seed, TerrainType type)
{
    if (type < 0 || type >= N_TERRAINS) type = TERRAIN_HILLS;
    const TerrainParams *p = &terrains[type];

    for (int z = 0; z < MAP_N; z++) {
        for (int x = 0; x < MAP_N; x++) {
            float h = fbm2d((float)x * p->noise_scale,
                            (float)z * p->noise_scale, seed);

            if (p->ridge) {
                /* Ridge noise: 1 − 2·|h − 0.5| ∈ [0, 1].
                 * Peaks of the original fbm become thin ridges; the
                 * high tier of the height ramp lights up only on
                 * narrow lines rather than broad summits.            */
                h = 1.0f - 2.0f * fabsf(h - 0.5f);
                if (h < 0.0f) h = 0.0f;
            }

            if (p->bias_exp != 1.0f) {
                h = powf(h, p->bias_exp);
            }

            if (p->terrace_steps > 0) {
                /* Quantise into N flat plateaus → mesa look.         */
                float steps = (float)p->terrace_steps;
                h = floorf(h * steps + 0.5f) / steps;
            }

            if (p->water_level > 0.0f && h < p->water_level) {
                /* Below water level — flatten into a uniform pool.   */
                h = 0.0f;
            }

            g_heightmap[z][x] = h * MAX_HEIGHT;
        }
    }
}

/*
 * terrain_sample — bilinear lookup at world position (wx, wz),
 * heightmap toroidally tiled every MAP_N cells.
 */
static float terrain_sample(float wx, float wz)
{
    int   xi = (int)floorf(wx);
    int   zi = (int)floorf(wz);
    float fx = wx - (float)xi;
    float fz = wz - (float)zi;
    int   x0 = xi & MAP_MASK,       z0 = zi & MAP_MASK;
    int   x1 = (xi + 1) & MAP_MASK, z1 = (zi + 1) & MAP_MASK;

    float h00 = g_heightmap[z0][x0];
    float h10 = g_heightmap[z0][x1];
    float h01 = g_heightmap[z1][x0];
    float h11 = g_heightmap[z1][x1];
    float h0  = h00 * (1.0f - fx) + h10 * fx;
    float h1  = h01 * (1.0f - fx) + h11 * fx;
    return h0 * (1.0f - fz) + h1 * fz;
}

/*
 * shadow_regen — for each heightmap cell, march toward the sun and
 * flag "shadowed" if any later terrain rises above the line connecting
 * the cell to the sun.
 *
 * Same as the original first-person version — independent of the
 * camera, so a top-down view uses the same precomputed shadow data.
 */
static void shadow_regen(float sun_yaw, float sun_pitch)
{
    const float SHADOW_BIAS = 0.05f;

    float horiz_len = cosf(sun_pitch);
    float sun_dx    = cosf(sun_yaw) * horiz_len;
    float sun_dz    = sinf(sun_yaw) * horiz_len;
    float dy_per_xz = tanf(sun_pitch);

    for (int z = 0; z < MAP_N; z++) {
        for (int x = 0; x < MAP_N; x++) {
            float h0 = g_heightmap[z][x];
            uint8_t shadowed = 0;
            for (int s = 1; s <= SHADOW_STEPS; s++) {
                float wx = (float)x + (float)s * sun_dx;
                float wz = (float)z + (float)s * sun_dz;
                float h_at  = terrain_sample(wx, wz);
                float h_ray = h0 + (float)s * dy_per_xz;
                if (h_at > h_ray + SHADOW_BIAS) {
                    shadowed = 1;
                    break;
                }
            }
            g_shadowmap[z][x] = shadowed;
        }
    }
}

static inline uint8_t shadow_at(float wx, float wz)
{
    int xi = (int)floorf(wx) & MAP_MASK;
    int zi = (int)floorf(wz) & MAP_MASK;
    return g_shadowmap[zi][xi];
}

/* ===================================================================== */
/* §6  scene — top-down pan + zoom + render                              */
/* ===================================================================== */

typedef struct {
    bool         paused;
    int          current_theme;
    TerrainType  current_terrain;
    uint32_t     seed;
    int          cols, rows;

    /* Top-down camera. */
    float        pan_x, pan_z;     /* world coords at SCREEN CENTRE     */
    float        zoom;             /* world units per terminal column   */

    /* Sun. */
    int          sun_dir_idx;
    float        sun_yaw, sun_pitch;
} Scene;

static void scene_apply_sun(Scene *s)
{
    s->sun_yaw   = g_sun_dirs[s->sun_dir_idx][0];
    s->sun_pitch = g_sun_dirs[s->sun_dir_idx][1];
    shadow_regen(s->sun_yaw, s->sun_pitch);
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_theme   = THEME_CLASSIC;
    s->current_terrain = TERRAIN_HILLS;
    s->seed            = (uint32_t)clock_ns();
    s->cols            = cols;
    s->rows            = rows;
    s->pan_x           = (float)MAP_N * 0.5f;
    s->pan_z           = (float)MAP_N * 0.5f;
    s->zoom            = ZOOM_DEFAULT;
    s->sun_dir_idx     = 0;

    terrain_regen(s->seed, s->current_terrain);
    scene_apply_sun(s);
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reseed(Scene *s)
{
    s->seed = (uint32_t)clock_ns() ^ 0xA5A5A5A5u;
    terrain_regen(s->seed, s->current_terrain);
    shadow_regen(s->sun_yaw, s->sun_pitch);
}

static void scene_cycle_sun(Scene *s)
{
    s->sun_dir_idx = (s->sun_dir_idx + 1) % N_SUN_DIRS;
    scene_apply_sun(s);
}

/*
 * scene_cycle_terrain — switch to the next/previous terrain type.
 *
 * Always rebuilds BOTH the heightmap (new fbm post-processing) AND
 * the shadowmap (because shadows depend on the new heights).  ~10 ms
 * total — the brief freeze is the rebuild, not a hang.
 */
static void scene_cycle_terrain(Scene *s, int dir)
{
    int idx = (int)s->current_terrain + dir;
    while (idx < 0) idx += N_TERRAINS;
    s->current_terrain = (TerrainType)(idx % N_TERRAINS);
    terrain_regen(s->seed, s->current_terrain);
    shadow_regen(s->sun_yaw, s->sun_pitch);
}

/*
 * scene_tick — only the slow auto-drift.  Pan from arrow keys is
 * applied immediately in the input handler (scene_pan_*).
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->pan_x += DRIFT_X * dt;
    s->pan_z += DRIFT_Z * dt;
}

static void scene_pan(Scene *s, float dx, float dz)
{
    s->pan_x += dx;
    s->pan_z += dz;
}

static void scene_zoom(Scene *s, float factor)
{
    s->zoom *= factor;
    if (s->zoom < ZOOM_MIN) s->zoom = ZOOM_MIN;
    if (s->zoom > ZOOM_MAX) s->zoom = ZOOM_MAX;
}

/*
 * scene_render — top-down hill-shaded heightmap.
 *
 *   1. Compute world-space sun direction once per frame.
 *   2. For each terminal cell, sample heightmap at the cell's world
 *      position; sample 4 neighbours for the gradient; combine with
 *      shadowmap into a Lambert + cast-shadow brightness.
 *   3. Map height tier → glyph + colour pair; brightness → attribute.
 *
 * We batch attron/attroff: the per-pixel (pair, attr) often repeats
 * across runs of similar elevation/lighting, so caching the last and
 * only switching on change cuts attribute thrash 3-5×.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    float zoom_x   = s->zoom;
    float zoom_z   = s->zoom * CELL_ASPECT;
    float ox       = s->pan_x - (float)s->cols   * 0.5f * zoom_x;
    float oz       = s->pan_z - (float)rows_eff  * 0.5f * zoom_z;

    /* Sun direction in world coords. */
    float horiz = cosf(s->sun_pitch);
    float sx    = cosf(s->sun_yaw) * horiz;
    float sy    = sinf(s->sun_pitch);
    float sz    = sinf(s->sun_yaw) * horiz;

    int     last_pair = -1;
    attr_t  last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {
        float wz = oz + ((float)row + 0.5f) * zoom_z;
        for (int col = 0; col < s->cols; col++) {
            float wx = ox + ((float)col + 0.5f) * zoom_x;

            float h = terrain_sample(wx, wz);

            /* Gradient via 4-tap central difference (1-cell stencil). */
            float h_l = terrain_sample(wx - 1.0f, wz);
            float h_r = terrain_sample(wx + 1.0f, wz);
            float h_n = terrain_sample(wx, wz - 1.0f);
            float h_u = terrain_sample(wx, wz + 1.0f);
            float dh_x = (h_r - h_l) * 0.5f;
            float dh_z = (h_u - h_n) * 0.5f;

            /* Surface normal, normalised. */
            float nx = -dh_x;
            float ny = 1.0f;
            float nz = -dh_z;
            float nlen = sqrtf(nx*nx + ny*ny + nz*nz);
            if (nlen > 1e-6f) {
                nx /= nlen; ny /= nlen; nz /= nlen;
            }

            float n_dot_sun = nx*sx + ny*sy + nz*sz;
            if (n_dot_sun < 0.0f) n_dot_sun = 0.0f;

            float light = AMBIENT + (1.0f - AMBIENT) * n_dot_sun;
            uint8_t shadowed = shadow_at(wx, wz);
            if (shadowed) light *= SHADOW_FACTOR;

            /* Height → tier slot. */
            int slot = (int)(h / MAX_HEIGHT * 7.999f);
            if (slot < 0) slot = 0;
            if (slot > 7) slot = 7;

            char   glyph = HEIGHT_GLYPHS[slot];
            int    pair  = PAIR_TERRAIN_BASE + slot;
            attr_t attr;
            if (shadowed || light < DIM_THRESHOLD)      attr = A_DIM;
            else if (light > BRIGHT_THRESHOLD)          attr = A_BOLD;
            else                                        attr = A_NORMAL;

            if (pair != last_pair || attr != last_attr) {
                if (last_pair >= 0)
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                attron(COLOR_PAIR(pair) | attr);
                last_pair = pair;
                last_attr = attr;
            }
            mvaddch(row, col, (chtype)(unsigned char)glyph);
        }
    }
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}
/* ── end §6 — to understand the ncurses I/O wrapper, read §7 screen ── */

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

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

    char buf[260];
    snprintf(buf, sizeof buf,
             " TERRAIN MAP   %s   type:%s   theme:%s   "
             "pan:(%6.1f,%6.1f)   zoom:%4.2f   sun:%d   "
             "%5.1f fps  %3d Hz   "
             "n/N:type  t/T:theme  arrows:pan  +/-:zoom  s:sun  r:reseed  spc:pause  q:quit ",
             s->paused ? "PAUSED " : "DRIFT  ",
             terrains[s->current_terrain].name,
             theme_name(s->current_theme),
             (double)s->pan_x, (double)s->pan_z,
             (double)s->zoom, s->sun_dir_idx,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
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
    case 'r': case 'R': scene_reseed(s);                              break;
    case 's': case 'S': scene_cycle_sun(s);                           break;

    case 'n':           scene_cycle_terrain(s, +1);                   break;
    case 'N':           scene_cycle_terrain(s, -1);                   break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case KEY_LEFT:
        scene_pan(s, -PAN_STEP_COLS * s->zoom, 0.0f);
        break;
    case KEY_RIGHT:
        scene_pan(s, +PAN_STEP_COLS * s->zoom, 0.0f);
        break;
    case KEY_UP:
        scene_pan(s, 0.0f, -PAN_STEP_ROWS * s->zoom * CELL_ASPECT);
        break;
    case KEY_DOWN:
        scene_pan(s, 0.0f, +PAN_STEP_ROWS * s->zoom * CELL_ASPECT);
        break;

    case '=': case '+':
        scene_zoom(s, 1.0f / ZOOM_STEP);   /* zoom IN  → smaller world/cell */
        break;
    case '-':
        scene_zoom(s, ZOOM_STEP);          /* zoom OUT → larger world/cell  */
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
