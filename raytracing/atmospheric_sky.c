/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * atmospheric_sky.c
 *   — Procedural sky over a procedural mountain silhouette. Per-pixel
 *     ray direction picks a colour from the active theme's
 *     horizon→zenith gradient; brightness modulated by Rayleigh-style
 *     scattering and by proximity to the sun. Mountains anchor the
 *     horizon; clouds drift across the sky; stars come out at night;
 *     the sun rises and sets over a 60-second day/night cycle.
 *
 * DEMO: A sky and a horizon. Mountains silhouetted black at the
 *       bottom; sky filling everything above. The sky's colour
 *       gradient anchors at the horizon (warm peach/orange when the
 *       sun is low, blue when high) and fades to the zenith colour at
 *       the top of the screen. A bright sun disc transits across the
 *       sky during the TRANSIT pattern; cloud wisps drift slowly past
 *       on a slow horizontal wind; at night the screen turns black
 *       and stars come out, twinkling with per-star phases.
 *
 *       PATTERN (n / N) — sun-position preset:
 *
 *         DAWN     sun rising on the east horizon — warm horizon belt,
 *                  blue zenith, sun disc visible
 *         DAY      sun high overhead — clean blue sky, bright sun
 *         DUSK     sun setting in the west — fierce orange/red horizon
 *                  band, twilight zenith, sun disc grazing horizon
 *         NIGHT    sun deep below — black sky with twinkling stars,
 *                  thin red glow on horizon (afterglow)
 *         TRANSIT  full ~60 s day/night cycle, all phases visible
 *
 *       'r' randomises phase / star arrangement / mountain silhouette.
 *
 * Study alongside:
 *   sphere_raytrace.c — same V3 vector math.
 *   ../procedural/worldgen/cloud.c — same glyph-density rendering
 *                                    technique (theme ramp + density
 *                                    glyph), here applied to
 *                                    atmospheric scattering instead of
 *                                    cloud-density fBm.
 *
 * Section map:
 *   §1 config    — constants, themes (sky-specific gradients)
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme ramp + accent pairs
 *   §5 sky       — V3 math, sun position, scattering, ground
 *                  silhouette, cloud sampler, star sampler
 *   §6 scene     — Scene state, scene_tick (advance time)
 *   §7 screen    — per-cell composited render + HUD
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume sun motion
 *   r          randomise (phase, stars, mountains)
 *   n / N      next pattern  (DAWN → DAY → DUSK → NIGHT → TRANSIT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster transit
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/atmospheric_sky.c \
 *       -o atmospheric_sky -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Composited sky-renderer with three layers:
 *
 *                  (1) GROUND silhouette — procedural mountain profile
 *                      via 2-octave sin sum, sampled per screen column.
 *                      Cells below the silhouette curve render as dark
 *                      "earth" colour; cells above render the sky.
 *
 *                  (2) SKY gradient — for each above-horizon cell:
 *                      compute view-altitude factor (0 at horizon, 1 at
 *                      zenith) and pick a colour from the active
 *                      theme's 8-step ramp such that:
 *                        - at noon: zenith is darker blue (ramp[3]),
 *                          horizon is lighter blue (ramp[5])
 *                        - at sunset: zenith is twilight (ramp[2]),
 *                          horizon is fierce orange (ramp[7])
 *                        - at night: everything is dim (ramp[0..1])
 *                      Brightness modulated by sun proximity:
 *                      Rayleigh phase function (3/4)(1+cos²θ) brightens
 *                      cells looking near the sun direction.
 *
 *                  (3) OVERLAYS — drawn on top of the sky:
 *                      - Sun disc + soft halo where the view direction
 *                        points within SUN_DISC_THRESH of the sun.
 *                      - Cloud bands: fBm noise sampled with horizontal
 *                        wind drift; bright bg + warm-tinted at sunset.
 *                      - Stars: hash-gated sparse '*' glyphs with
 *                        per-star sin-phase twinkle, visible only at
 *                        night (foreground glyph stays white over the
 *                        dark night sky).
 *
 *                  The terminal-rendering technique throughout is the
 *                  PROJECT-STANDARD glyph-density + theme-ramp pattern
 *                  (same as cloud.c, truchet_tiles.c): no background-
 *                  colour fill, no per-cube-color pairs. Each cell is
 *                  one glyph from `' .,:-^#@'` in one of 8 theme ramp
 *                  colours, optionally A_BOLD/A_DIM. Universally
 *                  compatible across terminals.
 *
 * Data-structure : NONE for the sky; pure function of (sx, sy, t,
 *                  pattern, seed, theme). Per-frame: one Scene struct.
 *
 * Rendering      : ASCII only. Per cell, pick one glyph + one theme
 *                  pair + one attr. No background-colour fill. Works
 *                  identically on every terminal that supports basic
 *                  256-color or 8-color attributes.
 *
 * Performance    : One cosf, one dotp, one sqrtf per cell, plus
 *                  occasional fBm samples for clouds. ~60 ns/cell.
 *                  At 240×80 × 30 fps ≈ 35 ms/sec, around 4 % of one
 *                  CPU core.
 *
 * References     :
 *   • Wikipedia — Rayleigh scattering
 *     https://en.wikipedia.org/wiki/Rayleigh_scattering
 *   • Sean O'Neil — "Accurate Atmospheric Scattering" (GPU Gems 2,
 *     ch. 16). The pragmatic real-time shader formulation that this
 *     file's per-pixel brightness modulation is loosely based on.
 *   • Inigo Quilez — Atmosphere demo
 *     https://www.shadertoy.com/view/lslXDr
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Every cell on the screen is a view ray. If the ray hits the
 * mountain silhouette, the cell is dark earth. Otherwise the cell is
 * sky — and the sky's colour at that point depends on TWO things:
 * (a) WHERE in the sky the ray points (low to the horizon or high
 * to the zenith), and (b) WHERE THE SUN IS (just risen, overhead,
 * setting, gone). At sunset, looking at the horizon, the colour is
 * warm orange because blue light has been scattered away over the
 * long air-path the sun's rays travel; looking at the zenith from
 * the same place, the colour is purple-blue because we see scattered
 * light from the side. We approximate that physics with two simple
 * gradient lookups + a sun-proximity brightness boost.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Every theme has an 8-step colour ramp. Read it as a horizon-to-zenith
 * gradient of THE SKY AT SUNSET: ramp[7] is fierce horizon orange,
 * ramp[3] is a high blue, ramp[0] is darkest twilight zenith. To
 * render at noon we slide the gradient: zenith uses a HIGHER index
 * (lighter blue) and horizon uses a slightly lower index (mid blue).
 * To render at night, both ends collapse to ramp[0..1] and we layer
 * stars over the top. The sun is just a bright glyph plopped at the
 * sun's screen position; the mountain silhouette is just a dark glyph
 * below a sin-of-x curve. Three layers, no shaders.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SUN POSITION. Pattern selects a fixed altitude (DAWN, DAY,
 *     DUSK, NIGHT) or animates it (TRANSIT: alt = sin(t · ω)).
 *
 *  2. PER FRAME, PER CELL (sx, sy):
 *     a. Compute mountain silhouette height at this column.
 *     b. If sy is below the silhouette, render dark earth and return.
 *     c. Compute view-altitude factor in [0, 1]: 0 at the horizon
 *        line, 1 at the very top of the screen.
 *     d. Compute sun-proximity factor: cos² of angle between view
 *        direction (from screen-y) and the sun's altitude.
 *     e. Pick a horizon ramp index and a zenith ramp index based on
 *        sun altitude (cool day vs warm sunset vs dim night).
 *     f. Lerp between them by view-altitude → final ramp index.
 *     g. Boost the ramp index by sun-proximity (cells looking near
 *        the sun look brighter / warmer).
 *     h. If view direction is INSIDE the sun disc, override with the
 *        bright sun glyph.
 *     i. Sample cloud fBm; if above threshold, blend in cloud glyph.
 *     j. At night, hash-gated stars overlay dark sky cells.
 *     k. Render glyph + colour + attribute.
 *
 *  3. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Sun direction:
 *    sun_alt = pattern_alt(pattern, t)            // ∈ [-1, 1]
 *    sun_y   = sin(sun_alt)                       // height factor
 *
 *  Mountain silhouette (per column):
 *    silhouette_y(sx) = base_y + A1·sin(sx·k1 + φ1) + A2·sin(sx·k2 + φ2)
 *    cell_is_ground   = (sy ≥ silhouette_y(sx))
 *
 *  View-altitude factor (above-horizon cells):
 *    view_y_factor    = (silhouette_y(sx) - sy) / silhouette_y(sx)   // 0..1
 *
 *  Sun-proximity (vertical-only approximation, since sun and view
 *  are both in the screen-vertical plane):
 *    sun_prox         = cos²((sun_screen_y - sy) · pixel_to_radian)
 *
 *  Day factor (smooth night→day transition):
 *    day_factor       = smoothstep(-0.10, 0.05, sun_y)
 *
 *  Ramp index lerp:
 *    horizon_idx      = lerp(NIGHT_HORIZON_IDX, DAY_HORIZON_IDX,  day_factor)
 *    zenith_idx       = lerp(NIGHT_ZENITH_IDX,  DAY_ZENITH_IDX,   day_factor)
 *    ramp_idx         = lerp(horizon_idx,       zenith_idx,       view_y_factor)
 *    ramp_idx        += SUN_BOOST · sun_prox · day_factor          // warmer near sun
 *
 *  Glyph density from local "intensity":
 *    intensity        = (1 - view_y_factor) · day_factor · 1.0
 *                     + sun_prox · day_factor · 1.5
 *    glyph            = ramp_glyphs[clamp(⌊intensity·8⌋, 0, 7)]
 *
 *  Cloud sampler:
 *    cloud            = fbm(sx · 0.04 + wind, sy · 0.10)
 *    if cloud > CLOUD_THRESH and sy is in cloud-altitude band:
 *      override glyph + ramp index with cloud glyph
 *
 *  Star sampler (night only):
 *    if (1 - day_factor) > 0.5 AND hash3(sx, sy, seed) % STAR_DENS == 0:
 *      twinkle = 0.5 + 0.5·sin(2π·t·1.0 + (hash >> 16)/65536·2π)
 *      if twinkle > 0.4: render '*' in PAIR_STAR (bright white)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SILHOUETTE LANDS ON HUD ROW. The mountain silhouette is computed
 *    in screen coordinates and could overlap the bottom HUD row. Cap
 *    `silhouette_y` to `rows − 2` so the HUD is never overwritten.
 *
 *  • SUN_BELOW_HORIZON SUN BOOST. We bump ramp index by sun proximity
 *    to brighten cells near the sun. If the sun is BELOW horizon (in
 *    the ground), proximity for above-horizon cells is the LARGEST
 *    near the horizon — exactly what we want for sunset glow. So the
 *    formula naturally gives "afterglow" without special-casing.
 *
 *  • NIGHT FALLBACK. When `day_factor < 0.05`, the sun's contribution
 *    to ramp_idx vanishes (multiplied by day_factor). The whole sky
 *    collapses to ramp[0..1] (darkest theme tints). Stars overlay on
 *    top.
 *
 *  • SUN DISC AT HIGH ALTITUDE. The sun's screen position is computed
 *    by mapping sun_alt directly to a row. At fully overhead (sun_alt
 *    = π/2), the sun should be at row 0 (top of screen); at horizon
 *    (sun_alt = 0), at the silhouette line. We map linearly between
 *    these.
 *
 *  • RAMP IDX OVERFLOW. After all the lerps and boosts, ramp_idx can
 *    exceed 7 or go below 0. Always clamp to [0, 7] before indexing
 *    into theme.ramp[].
 *
 *  • CLOUD ALTITUDE BAND. Clouds should appear in the LOWER HALF of
 *    the sky (close to horizon), not at the zenith. Restrict the
 *    cloud sampler to view_y_factor < 0.6.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Sun freezes mid-arc; clouds and twinkles freeze
 *    too. Resume: animation continues from exactly where it stopped.
 *
 *  • DAWN pattern. Sun visible just above horizon; warm peach belt;
 *    blue zenith; mountains in clear silhouette below.
 *
 *  • DAY pattern. Sun visible high; uniform blue sky; minimal horizon
 *    glow.
 *
 *  • DUSK pattern. Sun grazing horizon (or just below); fierce orange
 *    band along the horizon; sky deepens to twilight purple at zenith.
 *
 *  • NIGHT pattern. Black sky; visible stars twinkling; thin red
 *    afterglow on horizon for the first few seconds, then darkness.
 *
 *  • TRANSIT pattern. Watch the sun arc from low east → high overhead
 *    → low west → below horizon over ~60 s. Horizon colour smoothly
 *    morphs through the cycle.
 *
 *  • Theme cycle (t/T). Each theme produces a recognisably different
 *    atmosphere at the same sun position.
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
    SIM_FPS_DEFAULT     =  30,
    SIM_FPS_MAX         = 120,
    SIM_FPS_STEP        =  10,

    SPEED_MIN           =   1,
    SPEED_DEF           =   8,
    SPEED_MAX           =  64,

    HUD_COLS            =  80,
    FPS_UPDATE_MS       = 500,

    /* Color pair indices.  PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD            =   1,
    PAIR_HINT           =   2,
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 sky tints              */
    PAIR_GROUND         =  11,    /* dark earth                        */
    PAIR_STAR           =  12,    /* bright white                      */
    PAIR_SUN            =  13,    /* bright sun                        */
    PAIR_CLOUD          =  14,    /* cloud highlight                   */
    PAIR_FLASH          =  15,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Day-night smooth fade. */
#define DAY_FADE_BELOW       (-0.10f)
#define DAY_FADE_ABOVE        0.05f

/* TRANSIT pattern animation period (full day-night cycle). */
#define TRANSIT_PERIOD_S     60.0f

/* Mountain silhouette parameters. */
#define MOUNT_BASE_Y_FRAC    0.78f          /* base height as frac of avail rows */
#define MOUNT_AMP_FRAC       0.10f
#define MOUNT_FREQ_1         0.05f
#define MOUNT_FREQ_2         0.13f

/* Sun on screen — angular radius (in screen rows). */
#define SUN_RADIUS_ROWS      3
#define SUN_HALO_ROWS        7

/* Star density: 1 in N pixels can host a star. */
#define STAR_DENSITY         180
#define STAR_THRESH          0.40f
#define STAR_TWINKLE_HZ      0.5f

/* Cloud parameters. */
#define CLOUD_SCALE_X        0.040f
#define CLOUD_SCALE_Y        0.090f
#define CLOUD_WIND           4.0f      /* horizontal wind, cells/sec   */
#define CLOUD_THRESH         0.55f
#define CLOUD_BAND_FRAC      0.55f     /* clouds only in lower half    */
#define FBM_OCTAVES          3

/* Ramp-index endpoints used in horizon→zenith lerp.
 * Higher number → BRIGHTER ramp slot. */
#define DAY_HORIZON_IDX      6.5f
#define DAY_ZENITH_IDX       3.5f
#define DUSK_HORIZON_IDX     7.5f      /* clamp at 7 — fierce sunset   */
#define DUSK_ZENITH_IDX      2.0f
#define NIGHT_HORIZON_IDX    1.0f
#define NIGHT_ZENITH_IDX     0.0f

/* Sun proximity boost — shifts ramp index toward 7 (brightest). */
#define SUN_BOOST            1.6f

/* Pattern enum. */
typedef enum {
    PATTERN_DAWN    = 0,
    PATTERN_DAY     = 1,
    PATTERN_DUSK    = 2,
    PATTERN_NIGHT   = 3,
    PATTERN_TRANSIT = 4,
    N_PATTERNS      = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_DAWN:    return "DAWN   ";
    case PATTERN_DAY:     return "DAY    ";
    case PATTERN_DUSK:    return "DUSK   ";
    case PATTERN_NIGHT:   return "NIGHT  ";
    case PATTERN_TRANSIT: return "TRANSIT";
    default:              return "?      ";
    }
}

/*
 * Themes — each ramp[8] is a HORIZON→ZENITH gradient suitable for
 * sunset, with ramp[7] = fiercest horizon and ramp[0] = darkest
 * twilight. Daytime renders use ramp[3..6]; sunset uses ramp[2..7];
 * night uses ramp[0..1] + stars.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule so even A_DIM cells stay
 * legible.
 */
typedef struct {
    const char *name;
    short       ramp[8];
    short       sun;
    short       star;
    short       ground;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        zenith           horizon  sun  star ground */
    /*           [0]  [1]  [2]  [3]  [4]  [5]  [6]  [7]                */
    { "DEFAULT", { 24,  31,  61,  68, 110, 217, 209, 226 }, 226, 231,  236 },
    { "MATRIX",  { 28,  34,  40,  46,  82, 118, 154, 226 }, 226, 231,  235 },
    { "NOVA",    { 60,  91, 134, 165, 207, 213, 219, 231 }, 226, 231,  235 },
    { "MONO",    {240, 243, 245, 247, 249, 251, 253, 255 }, 255, 255,  238 },
    { "OCEAN",   { 24,  25,  31,  38,  45,  87, 117, 195 }, 226, 231,  236 },
    { "FIRE",    { 88, 124, 130, 166, 196, 208, 214, 226 }, 226, 231,  236 },
    { "EARTH",   { 94, 130, 137, 173, 179, 215, 222, 230 }, 226, 231,  235 },
    { "FOREST",  { 28,  34,  40,  64,  70, 112, 156, 192 }, 226, 231,  235 },
    { "DESERT",  {130, 137, 143, 173, 179, 215, 222, 229 }, 226, 231,  236 },
    { "ARCTIC",  { 24,  31,  67, 110, 117, 153, 195, 231 }, 231, 231,  238 },
};

/* Density-glyph ramp from sparse to dense. Used for sky-cell
 * brightness — sparser glyphs read as "dim sky", denser as "bright".
 *
 * Tuned for AIRY sky: brightest cells use '+' / '*' rather than the
 * blocky '#' / '@' so the sky reads as light atmosphere instead of
 * solid pixel blocks. */
static const char RAMP_GLYPHS[8] = { '`', '.', ',', ':', ';', '-', '+', '*' };

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
        init_pair(PAIR_GROUND, t->ground, -1);
        init_pair(PAIR_STAR,   t->star,   -1);
        init_pair(PAIR_SUN,    t->sun,    -1);
        init_pair(PAIR_CLOUD,  255,       -1);   /* near-white          */
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,   COLOR_CYAN,
            COLOR_WHITE, COLOR_YELLOW,COLOR_YELLOW, COLOR_RED,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i], -1);
        init_pair(PAIR_GROUND, COLOR_BLACK,  -1);
        init_pair(PAIR_STAR,   COLOR_WHITE,  -1);
        init_pair(PAIR_SUN,    COLOR_YELLOW, -1);
        init_pair(PAIR_CLOUD,  COLOR_WHITE,  -1);
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
/* §5  sky — math, sun, mountains, clouds, stars                          */
/* ===================================================================== */

/* hash3 — drives star placement. */
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

/* Perlin scaffold — copied inline per the self-contained-file rule.
 * Only used for cloud sampling. */
static uint8_t perm[512];

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

static inline float fade_q(float t) { return t*t*t*(t*(t*6.f-15.f)+10.f); }
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

static float fbm2(float x, float y)
{
    float total = 0, amp = 1, freq = 1, max_amp = 0;
    for (int o = 0; o < FBM_OCTAVES; o++) {
        total += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp *= 0.5f; freq *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

static inline float smoothstep(float a, float b, float x)
{
    if (x < a) return 0;
    if (x > b) return 1;
    float t = (x - a) / (b - a);
    return t * t * (3.0f - 2.0f * t);
}

/*
 * sun_screen_pos — where the sun sits in screen-cell coordinates.
 * sun_alt ∈ [-π/2, π/2]; we map +π/2 (overhead) to row 0 (top), 0
 * (horizon) to the silhouette base row, and negative values to BELOW
 * the silhouette (sun has set / not risen).
 *
 * sun_az controls horizontal position; for our patterns it cycles
 * smoothly over the day so the sun ARCs across the screen rather
 * than rising vertically in place.
 */
static void sun_screen_pos(float sun_alt, float sun_az,
                           int cols, int rows,
                           int silhouette_base_y,
                           int *out_sx, int *out_sy)
{
    /* horizontal: az ∈ [-π/2, π/2] maps across the full cols */
    float fx = (sun_az + (float)M_PI / 2.0f) / (float)M_PI;   /* [0, 1] */
    fx = fx * fx * (3.0f - 2.0f * fx);                          /* smoothstep ease */
    *out_sx = (int)(fx * (float)cols);

    /* vertical: alt ∈ [-π/2, π/2] maps from silhouette_base_y down to
     * row 0 up.  Above horizon (alt > 0): top half of sky.
     * Below horizon (alt < 0): below silhouette (off-screen for sun). */
    float top_y    = 1.0f;                                      /* row 1 (just below HUD) */
    float horiz_y  = (float)silhouette_base_y;
    float below_y  = (float)(rows + 5);                         /* off-screen below */

    if (sun_alt > 0) {
        float u = sun_alt / ((float)M_PI / 2.0f);   /* 0..1 */
        u = sqrtf(u);                                /* nonlinear so sun lingers near horizon */
        *out_sy = (int)(horiz_y + (top_y - horiz_y) * u);
    } else {
        float u = -sun_alt / ((float)M_PI / 2.0f);
        *out_sy = (int)(horiz_y + (below_y - horiz_y) * u);
    }
}

/*
 * mountain_silhouette_y — return the row index of the mountain
 * silhouette at column sx. Cells with sy >= this value are ground;
 * cells above are sky.
 */
static int mountain_silhouette_y(int sx, int rows, float seed_phase)
{
    int avail = rows - 1;       /* leave bottom for HUD */
    float base = (float)avail * MOUNT_BASE_Y_FRAC;
    float amp  = (float)avail * MOUNT_AMP_FRAC;
    float h = base
            + amp        * sinf((float)sx * MOUNT_FREQ_1 + seed_phase)
            + amp * 0.4f * sinf((float)sx * MOUNT_FREQ_2 + seed_phase * 1.7f)
            + amp * 0.2f * sinf((float)sx * MOUNT_FREQ_2 * 2.3f + seed_phase * 0.5f);
    int y = (int)h;
    if (y < 2)         y = 2;
    if (y > avail - 1) y = avail - 1;
    return y;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   phase_offset;
    float   mount_phase;        /* offset for mountain silhouette  */
    int     star_seed;
    float   flash_t;
} Scene;

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->phase_offset * 100.0f), 0xC0FFEE);
    s->phase_offset = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->mount_phase  = ((float)((h >> 16) & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->star_seed    = (int)(h ^ 0x5A5A5A5Au);
    s->flash_t      = 1.0f;
    perm_shuffle(s->star_seed);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_TRANSIT;
    s->phase_offset    = 0.0f;
    s->mount_phase     = 0.0f;
    s->star_seed       = 0xDECAF;
    s->flash_t         = 1.0f;
    perm_shuffle(s->star_seed);
}

static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-4.0f * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* sun_alt_az for the current pattern. */
static void scene_sun(const Scene *s, float *out_alt, float *out_az)
{
    switch (s->current_pattern) {
    case PATTERN_DAWN:    *out_alt = +0.20f; *out_az = -0.95f; break;
    case PATTERN_DAY:     *out_alt = +1.00f; *out_az =  0.00f; break;
    case PATTERN_DUSK:    *out_alt = +0.05f; *out_az = +1.10f; break;
    case PATTERN_NIGHT:   *out_alt = -0.80f; *out_az = +1.50f; break;
    case PATTERN_TRANSIT:
    default: {
        float omega = 2.0f * (float)M_PI / TRANSIT_PERIOD_S;
        float phase = s->time_secs * omega + s->phase_offset;
        *out_alt = ((float)M_PI / 2.0f) * 0.95f * sinf(phase);
        *out_az  = -((float)M_PI / 2.0f) * 0.9f  * cosf(phase);
        break;
    }
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

/*
 * scene_draw — composite the three layers per cell:
 *   1. Ground (silhouette + below)
 *   2. Sky gradient (horizon→zenith colour, sun-proximity brightness)
 *   3. Overlays (sun disc + halo, clouds, stars)
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    const Theme *th = &themes[s->current_theme];
    (void)th;       /* used implicitly via PAIR_RAMP_BASE pairs */

    float sun_alt, sun_az;
    scene_sun(s, &sun_alt, &sun_az);
    float sun_y = sinf(sun_alt);
    float day_factor = smoothstep(DAY_FADE_BELOW, DAY_FADE_ABOVE, sun_y);

    /* Sun screen position uses a representative silhouette base row. */
    int silhouette_ref = mountain_silhouette_y(sc->cols / 2,
                                               sc->rows, s->mount_phase);
    int sun_sx, sun_sy;
    sun_screen_pos(sun_alt, sun_az, sc->cols, sc->rows,
                   silhouette_ref, &sun_sx, &sun_sy);

    /* Per-pattern horizon/zenith ramp index endpoints, lerped via
     * day_factor between night and day, then nudged toward dusk
     * when sun is low (warm horizon, cool zenith). */
    float dusk_factor = smoothstep(0.30f, 0.05f, sun_y);   /* 1 at low sun */
    float h_idx_day  = lerp_f(DAY_HORIZON_IDX, DUSK_HORIZON_IDX, dusk_factor);
    float z_idx_day  = lerp_f(DAY_ZENITH_IDX,  DUSK_ZENITH_IDX,  dusk_factor);

    float horizon_idx = lerp_f(NIGHT_HORIZON_IDX, h_idx_day, day_factor);
    float zenith_idx  = lerp_f(NIGHT_ZENITH_IDX,  z_idx_day, day_factor);

    int top    = 0;
    int bottom = sc->rows - 1;

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {

            int silh_y = mountain_silhouette_y(sx, sc->rows, s->mount_phase);

            /* ─ Ground ─────────────────────────────────────────────── */
            if (sy >= silh_y) {
                /* Ground glyph: '/' or '\' with 25% mix gives a
                 * subtle texture; pick by hash for variety. */
                char gg = '#';
                int hh = (int)hash3(sx, sy, 0xEA12);
                gg = (hh & 3) == 0 ? '#' : (hh & 3) == 1 ? '@' : '#';
                attron(COLOR_PAIR(PAIR_GROUND));
                mvaddch(sy, sx, (chtype)(unsigned char)gg);
                attroff(COLOR_PAIR(PAIR_GROUND));
                continue;
            }

            /* ─ Sky ─────────────────────────────────────────────────── */

            /* view_y_factor: 0 at horizon (silhouette line), 1 at top
             * row. Higher = closer to zenith. */
            float vy = (float)(silh_y - sy) / (float)silh_y;
            if (vy < 0) vy = 0;
            if (vy > 1) vy = 1;

            /* Ramp index lerp horizon→zenith. */
            float ramp_f = lerp_f(horizon_idx, zenith_idx, vy);

            /* Sun proximity boost — brighter near sun screen position. */
            float dx = (float)(sx - sun_sx);
            float dy = (float)(sy - sun_sy) * 2.0f;        /* aspect */
            float dist2 = dx * dx + dy * dy;
            float prox = expf(-dist2 / (40.0f * 40.0f));   /* gaussian falloff */
            ramp_f += SUN_BOOST * prox * day_factor;

            /* Brightness for glyph density. */
            float intensity = day_factor * (0.4f + 0.6f * (1.0f - vy))
                            + day_factor * 1.2f * prox;

            /* ─ Sun disc / halo ─────────────────────────────────────── */
            bool drew_sun = false;
            if (sun_y > -0.05f && dist2 < (float)(SUN_HALO_ROWS * SUN_HALO_ROWS) * 4) {
                /* Halo region. Use small bright glyphs so the sun
                 * reads as a sparkle rather than a solid blob. */
                float r = sqrtf(dist2);
                if (r < (float)(SUN_RADIUS_ROWS * 2)) {
                    attron(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                    mvaddch(sy, sx, '*');
                    attroff(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                    drew_sun = true;
                } else if (r < (float)(SUN_HALO_ROWS * 2)) {
                    char hg = (r < (float)(SUN_HALO_ROWS * 1.4f)) ? '+' : ':';
                    attron(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                    mvaddch(sy, sx, (chtype)(unsigned char)hg);
                    attroff(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                    drew_sun = true;
                }
            }
            if (drew_sun) continue;

            /* ─ Cloud overlay ───────────────────────────────────────── */
            if (vy < CLOUD_BAND_FRAC && day_factor > 0.3f) {
                float wind = s->time_secs * CLOUD_WIND;
                float c = fbm2((float)sx * CLOUD_SCALE_X + wind,
                               (float)sy * CLOUD_SCALE_Y);
                if (c > CLOUD_THRESH) {
                    /* Cloud cell — wispy glyphs so clouds read as
                     * soft vapour, not solid blocks. */
                    char cg = (c > 0.75f) ? '*' : (c > 0.65f) ? '+' : ':';
                    int cp  = (dusk_factor > 0.4f) ? PAIR_RAMP_BASE + 7 : PAIR_CLOUD;
                    attron(COLOR_PAIR(cp) | A_BOLD);
                    mvaddch(sy, sx, (chtype)(unsigned char)cg);
                    attroff(COLOR_PAIR(cp) | A_BOLD);
                    continue;
                }
            }

            /* ─ Star overlay (night only) ───────────────────────────── */
            if (day_factor < 0.5f) {
                uint32_t h = hash3(sx, sy, s->star_seed);
                if ((h % STAR_DENSITY) == 0u) {
                    float phase = (float)((h >> 16) & 0xFFFFu) / 65536.0f
                                * 2.0f * (float)M_PI;
                    float tw = 0.5f + 0.5f * sinf(2.0f * (float)M_PI
                                                  * STAR_TWINKLE_HZ
                                                  * s->time_secs + phase);
                    float bright = (1.0f - day_factor) * tw;
                    if (bright > STAR_THRESH) {
                        char sg = (bright > 0.8f) ? '*' : '.';
                        int  sa = (bright > 0.8f) ? A_BOLD : A_NORMAL;
                        attron(COLOR_PAIR(PAIR_STAR) | sa);
                        mvaddch(sy, sx, (chtype)(unsigned char)sg);
                        attroff(COLOR_PAIR(PAIR_STAR) | sa);
                        continue;
                    }
                }
            }

            /* ─ Plain sky cell ─────────────────────────────────────── */
            int ramp_idx = (int)(ramp_f + 0.5f);
            if (ramp_idx < 0) ramp_idx = 0;
            if (ramp_idx > 7) ramp_idx = 7;

            int gly_idx = (int)(intensity * 8.0f);
            if (gly_idx < 0) gly_idx = 0;
            if (gly_idx > 7) gly_idx = 7;

            int attr = (gly_idx >= 6) ? A_BOLD
                     : (gly_idx <= 1) ? A_DIM
                     : A_NORMAL;
            int pair = PAIR_RAMP_BASE + ramp_idx;
            char glyph = RAMP_GLYPHS[gly_idx];

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

    float sun_alt, sun_az;
    scene_sun(s, &sun_alt, &sun_az);

    const char *state_str = s->paused ? "PAUSED" : pattern_name(s->current_pattern);

    char buf[160];
    snprintf(buf, sizeof buf,
             " ATMOSPHERIC SKY   %s   theme:%-8s   sun:%+.2f rad   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             (double)sun_alt, fps, sim_fps, s->speed);
    int row = sc->rows - 1;
    int len = (int)strlen(buf);
    if (len > sc->cols) buf[sc->cols] = 0;

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
        clock_sleep_ns(NS_PER_SEC / 30 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
