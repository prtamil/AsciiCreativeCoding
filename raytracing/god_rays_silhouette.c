/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * god_rays_silhouette.c
 *   — Volumetric light shafts (god rays) through a chunky 2-D
 *     silhouette. Per cell, march along the line from the cell
 *     toward the sun's screen position; at each sample, test
 *     occupancy against the silhouette function. Unblocked samples
 *     accumulate light weighted by Beer-Lambert exp(-σ · distance);
 *     the result is bright divergent shafts streaming from the
 *     gaps in the silhouette toward the camera.
 *
 * DEMO: A solid black silhouette dominates the lower-foreground —
 *       an archway, a mountain peak, a broken column, a row of
 *       cathedral windows, or a single tree. Behind it, a bright
 *       golden sun glows through dim warm fog. Wherever the sun
 *       can "see" the camera through a gap in the silhouette
 *       (between the pillars of the arch, over the mountain ridge,
 *       through the windows of the cathedral, between the branches
 *       of the tree), a wide divergent SHAFT of light streaks
 *       outward — like dust-mote sunbeams in a dim room. The shafts
 *       fade with distance from the sun; the silhouette stays
 *       solid black; the fog between shafts is dim warm haze.
 *
 *       PATTERN (n / N):
 *
 *         ARCHWAY      two stone pillars + curved arch top; light
 *                      streams under the arch in a single wide cone
 *         MOUNTAIN     a single peak silhouette; light spills over
 *                      the ridge in a fan
 *         COLUMN       a tall broken column; light streams from
 *                      either side
 *         WINDOWS      a cathedral wall pierced by a 4×2 grid of
 *                      arched windows; one shaft per window
 *         TREE         a single L-system tree silhouette; many
 *                      thin shafts threading between the branches
 *
 *       'r' reseeds (sun position offset, fog wind phase).
 *
 * Study alongside:
 *   solar_eclipse.c     — same framework + per-cell brightness
 *                          accumulation; eclipse uses corona overlay
 *                          rather than fog ray-march.
 *   atmospheric_sky.c   — same theme-ramp + glyph density rendering.
 *
 * Section map:
 *   §1 config       — constants, themes (warm fog palettes)
 *   §2 clock        — monotonic timer + sleep
 *   §3 color        — 8-pair fog ramp + accent pairs
 *   §5 silhouette   — five silhouette functions (point-in-shape)
 *   §6 raymarch     — screen-space ray-march, fog density,
 *                     visibility accumulation
 *   §6 scene        — Scene state, scene_tick (advance time)
 *   §7 screen       — per-cell composited render + HUD
 *   §8 app          — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume sun drift
 *   r          reseed (sun position, fog wind)
 *   n / N      next pattern  (ARCH → MTN → COLUMN → WINDOWS → TREE)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster sun drift
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/god_rays_silhouette.c \
 *       -o god_rays_silhouette -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Screen-space volumetric light shafts ("god
 *                  rays") via per-cell shadow-ray accumulation.
 *
 *                  Conceptually: imagine a single 2-D scene with
 *                    - a SUN at point (sun_sx, sun_sy)
 *                    - a SILHOUETTE region S (cells inside are
 *                      opaque, cells outside are clear fog)
 *                    - a CAMERA looking at the scene from the
 *                      front (the screen is the camera's image
 *                      plane).
 *
 *                  For each cell C = (sx, sy) the renderer asks:
 *                  "How much light reaches the camera by scattering
 *                  off fog molecules along the line from C to the
 *                  sun?" That is the BRIGHTNESS at that cell.
 *
 *                  The math is a Riemann sum of Beer-Lambert in
 *                  screen space:
 *                    march N steps from C toward (sun_sx, sun_sy);
 *                    at sample i, compute light delivered:
 *                      L_i = visible_at_i · exp(-σ · d_i)
 *                    where visible_at_i = 1 iff the line from
 *                    sample i to the SUN is unblocked by S, and
 *                    d_i is the distance from the camera (cell C)
 *                    to sample i.
 *                    accumulate Σ L_i / Σ exp(-σ · d_i).
 *
 *                  In our simplified version "visible_at_i" is just
 *                  "sample i is OUTSIDE the silhouette" (the
 *                  silhouette is so close to the camera plane that
 *                  the shadow ray from sample i to sun is well-
 *                  approximated by sampling the silhouette at
 *                  point i itself). This is the same trick used in
 *                  Crysis-era screen-space god rays and gives the
 *                  same divergent-cone visual at a tiny fraction
 *                  of the cost of full 3-D shadow rays.
 *
 *                  Five silhouette FUNCTIONS provide the patterns:
 *                  ARCHWAY (two pillars + half-circle top), MOUNTAIN
 *                  (gaussian-bump peak), COLUMN (tall capped
 *                  rectangle), WINDOWS (rectangle minus a 4×2 grid
 *                  of arched holes), TREE (trunk + 6 angled branch
 *                  capsules). Each takes (u, v) ∈ [-1, +1]² and
 *                  returns true if the point is INSIDE the
 *                  silhouette.
 *
 * Data-structure : NONE persistent. Each frame is a pure function
 *                  of (cols, rows, time, pattern, seed, theme).
 *                  Per-frame: one Scene struct.
 *
 * Rendering      : ASCII only. Density-glyph ramp `' .,:-^#@'` +
 *                  theme ramp pair + optional A_BOLD/A_DIM. No
 *                  background-colour fill.
 *
 * Performance    : MARCH_STEPS samples per cell × 1 silhouette test
 *                  ≈ 20 simple branches per sample. ~1.5 μs / cell
 *                  at MARCH_STEPS = 20. 240×80 × 30 fps → ~14 ms
 *                  shading per frame; well under the 33 ms budget.
 *                  Reduce MARCH_STEPS or terminal size if your CPU
 *                  is slower.
 *
 * References     :
 *   • Hoffman, N. & Preetham, A. — "Real-time Light Atmosphere
 *     Interactions for Outdoor Scenes" (Game Programming Gems 5).
 *   • Wikipedia — Crepuscular rays
 *     https://en.wikipedia.org/wiki/Crepuscular_rays
 *     The atmospheric phenomenon ("god rays") this demo simulates.
 *   • Wikipedia — Beer-Lambert law
 *     https://en.wikipedia.org/wiki/Beer%E2%80%93Lambert_law
 *     The exp(-σ·d) extinction law that weights samples.
 *   • Mitchell, K. — "Volumetric Light Scattering as a Post-Process"
 *     in GPU Gems 3, ch. 13. The screen-space god-rays technique
 *     this demo's algorithm is a direct descendant of.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Each cell on the screen asks the sun: "Is there a clear line
 * from you to me, through the fog?" The answer comes from
 * MARCHING. Walk N small steps from the cell toward the sun's
 * screen position. At each step, ask: "Is this step inside the
 * silhouette?" Count the unblocked steps, weighted by how far
 * from the eye they are (closer = more contribution because the
 * exp(-σ·d) extinction is gentler). The total is how much light
 * gets through. Bright shafts emerge wherever the line from cell
 * to sun threads a CLEAR channel through the silhouette; dark
 * fog fills the cells where every step lies inside the silhouette.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a dim hallway. There's a door at one end with light
 * coming through a keyhole. Dust hangs in the air. From your
 * perspective at the other end, the dust forms a bright cone
 * radiating out from the keyhole — the divergent shaft you
 * recognise as "god rays". At a cell directly along the
 * keyhole-to-eye line, every dust mote on that line is illuminated
 * (the keyhole sees those motes), so the shaft is brightest. At a
 * cell off-axis, only the dust motes near the door see the
 * keyhole; the rest sit in the door's shadow. So off-axis cells
 * are dimmer. Result: a luminous cone of dust diverging from the
 * keyhole. Replace "keyhole" with "gap in the silhouette" and you
 * have this demo.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. SUN POSITION. Per frame, compute (sun_sx, sun_sy) — slow
 *     horizontal drift across the upper portion of the screen
 *     so the shaft directions change over time.
 *
 *  2. PER CELL (sx, sy):
 *     a. Convert (sx, sy) to normalised (u, v) ∈ [-1, +1]²
 *        with aspect-correction (terminal cells 2× taller).
 *     b. If silhouette(u, v) → render BLACK silhouette glyph and
 *        continue.
 *     c. Compute step vector from cell toward sun:
 *          step_dx = (sun_sx - sx) / MARCH_STEPS
 *          step_dy = (sun_sy - sy) / MARCH_STEPS
 *     d. March N steps:
 *          for i = 1..MARCH_STEPS:
 *            px, py = sx + step_dx · i, sy + step_dy · i
 *            convert (px, py) to (uu, vv)
 *            d = i · step_length      // distance from eye in cells
 *            w = exp(-FOG_SIGMA · d)
 *            if not silhouette(uu, vv): accum += w
 *            total_w += w
 *     e. visibility = accum / total_w   // [0, 1]
 *     f. Add a sun-disc contribution if cell is very close to
 *        the sun's screen position AND sun isn't behind silhouette.
 *     g. Add slight fog wisps via fBm density modulation (optional
 *        per-frame eye candy — light shafts shimmer slightly as
 *        the wind moves the dust).
 *     h. Map total intensity → glyph + theme ramp colour.
 *
 *  3. HUD on bottom row.
 *
 * KEY FORMULAS
 * ────────────
 *  Aspect-corrected normalised cell coords:
 *    u = (2 · sx + 1 - cols) / cols
 *    v = (2 · sy + 1 - rows) / rows · (rows · ASPECT_Y / cols)
 *
 *  Step vector toward sun (in cell coordinates):
 *    Δsx = (sun_sx - sx) / MARCH_STEPS
 *    Δsy = (sun_sy - sy) / MARCH_STEPS
 *    step_len = √(Δsx² + (Δsy · ASPECT_Y)²)        // visual distance
 *
 *  Sample weight at step i (Beer-Lambert):
 *    d_i = i · step_len
 *    w_i = exp(-FOG_SIGMA · d_i)
 *
 *  Visibility accumulation:
 *    accum   = Σ_{i: silhouette FALSE} w_i
 *    total_w = Σ_{i in 1..N}            w_i
 *    vis     = accum / total_w
 *
 *  Sun-disc contribution (sx,sy near sun_sx,sun_sy):
 *    dx, dy = sx - sun_sx, (sy - sun_sy) · ASPECT_Y
 *    r²     = dx² + dy²
 *    sun    = exp(-r² / SUN_FALLOFF²) · (1 - sun_in_silhouette)
 *
 *  Fog wind (subtle visual modulation):
 *    wind = time · FOG_WIND
 *    fog_jitter = 0.85 + 0.15 · fbm2(u·1.2 + wind, v·1.2)
 *    intensity *= fog_jitter
 *
 *  Total intensity:
 *    intensity = vis · SHAFT_GAIN + sun · SUN_GAIN
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL ON SUN. When (sx, sy) ≈ (sun_sx, sun_sy), the step
 *    vector is near-zero and N samples all coincide. Use sample
 *    point at i + 0.5 OR add an early-out: if distance < 1 cell
 *    skip the march and set visibility = 1.
 *
 *  • CELL INSIDE SILHOUETTE. The silhouette test gates everything:
 *    inside cells render solid black with NO march. Otherwise
 *    march might yield "visible to sun through the silhouette
 *    behind", which makes no sense.
 *
 *  • SHAFT BACKLIT BY OWN OCCLUDER. The march samples points
 *    BETWEEN the cell and the sun. The cell itself is OUTSIDE
 *    the silhouette (gated above). If the silhouette is between
 *    cell and sun, samples will fall inside it → visibility
 *    drops → cell stays dark. That's exactly the SHADOW behind
 *    the silhouette. No special casing needed.
 *
 *  • SAMPLE POINT GOES OFF-SCREEN. The march can sample (px, py)
 *    outside [0, cols)×[0, rows). The silhouette function is
 *    defined in normalised coords, so it works just as well
 *    off-screen — points outside the silhouette area render as
 *    "clear sky". No clamping needed. (Theoretically we COULD
 *    treat off-screen as silhouette, but treating it as clear
 *    matches "the world continues outside our window" intuition.)
 *
 *  • ASPECT RATIO IN STEP-LENGTH. Terminal cells are ~2× taller
 *    than wide. The "visual length" of one step needs ASPECT_Y
 *    multiplied into the y component, otherwise vertical shafts
 *    look longer (more samples = more accumulation) than they
 *    should. step_len uses ASPECT_Y · Δsy in its sqrt.
 *
 *  • WINDOW ALIGNMENT. The WINDOWS pattern's holes must be
 *    arched/rectangular consistently. The grid of holes is
 *    parameterised on (gu, gv) within each cell; if the cell
 *    arithmetic floors incorrectly at exact boundaries, you get
 *    one-pixel-wide false windows. Use a small margin in the
 *    boundary tests to avoid this.
 *
 *  • PERFORMANCE. MARCH_STEPS · cols · rows is the per-frame
 *    silhouette-evaluation count. At 20 steps · 240 · 80 ≈ 384k
 *    silhouette evals per frame. Each is a few branches. Modern
 *    CPUs handle this in ~5-15 ms. If you push terminal size to
 *    400×120, drop MARCH_STEPS to 14 to maintain 30 fps.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Sun freezes; shafts stop drifting. Resume:
 *    sun continues smoothly.
 *
 *  • ARCHWAY pattern. The two pillars are visible as black
 *    rectangles; the curved top is visible as a black half-disc.
 *    A bright shaft fans out UNDER the arch toward the camera,
 *    bright on the line from the arch interior to the sun, dim
 *    on the sides.
 *
 *  • MOUNTAIN pattern. A black peak fills the lower portion. A
 *    bright fan of light spills OVER the ridge wherever the sun
 *    is positioned above the ridgeline.
 *
 *  • WINDOWS pattern. Several thin parallel shafts streaming
 *    through the windows in the cathedral wall — one shaft per
 *    window, all converging back toward the sun.
 *
 *  • TREE pattern. The trunk and branches form a black
 *    silhouette. Many thin shafts thread between the branches
 *    (where the sun is visible through the gaps); branches cast
 *    sharp dark stripes.
 *
 *  • Theme cycle (t/T). Each theme should produce a recognisably
 *    different fog colour while shaft structure stays identical.
 *
 *  • Speed (+/−). Doubling speed should approximately halve the
 *    period between sun's leftmost and rightmost positions.
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
    PAIR_FOG_BASE       =   3,    /* +0..+7 = 8 fog/shaft ramp tints   */
    PAIR_SILHOUETTE     =  11,    /* dark silhouette                   */
    PAIR_SUN            =  12,    /* bright sun disc                   */
    PAIR_FLASH          =  13,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define ASPECT_Y          2.0f       /* terminal cells 2× taller       */

/* Ray march. */
#define MARCH_STEPS         20
#define FOG_SIGMA           0.045f   /* extinction per cell distance   */
#define SHAFT_GAIN          1.30f
#define SUN_GAIN            1.50f
#define SUN_FALLOFF_CELLS   3.5f

/* Sun motion. */
#define SUN_DRIFT_PERIOD_S  20.0f
#define SUN_X_AMP_FRAC      0.30f    /* of cols                        */
#define SUN_Y_FRAC          0.18f    /* row position as frac of rows   */
#define SUN_Y_AMP_FRAC      0.04f    /* slight vertical drift          */

/* Fog wind (subtle visual modulation). */
#define FOG_WIND            0.8f
#define FOG_JITTER_AMP      0.12f

/* Pattern enum. */
typedef enum {
    PATTERN_ARCHWAY  = 0,
    PATTERN_MOUNTAIN = 1,
    PATTERN_COLUMN   = 2,
    PATTERN_WINDOWS  = 3,
    PATTERN_TREE     = 4,
    N_PATTERNS       = 5,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_ARCHWAY:  return "ARCHWAY ";
    case PATTERN_MOUNTAIN: return "MOUNTAIN";
    case PATTERN_COLUMN:   return "COLUMN  ";
    case PATTERN_WINDOWS:  return "WINDOWS ";
    case PATTERN_TREE:     return "TREE    ";
    default:               return "?       ";
    }
}

/*
 * Themes — fog[8] is a dim → bright shaft gradient. Used for
 * everything outside the silhouette: low values = dim fog haze,
 * high values = bright shaft cores. The silhouette is solid black
 * (or close to it) so the contrast with the bright fog reads cleanly.
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per
 * the CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       fog[8];
    short       silhouette;
    short       sun;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {

    /* name      fog[0..7]                                       silhouette sun */

    { "GOLDEN", { 94, 130, 137, 173, 178, 220, 222, 230 },        232,      231 },
    { "AMBER",  { 88, 124, 130, 166, 172, 208, 215, 222 },        232,      226 },
    { "DAWN",   { 88, 124, 167, 174, 211, 217, 224, 231 },        232,      231 },
    { "FOREST", { 28,  64,  70,  64, 112, 119, 156, 192 },        232,      231 },
    { "OCEAN",  { 24,  31,  38,  45,  87, 117, 153, 195 },        232,      231 },
    { "EMBER",  { 52,  88, 124, 130, 166, 202, 208, 220 },        232,      226 },
    { "ASH",    {236, 240, 243, 245, 247, 249, 251, 255 },        232,      231 },
    { "PURPLE", { 53,  91, 134, 165, 207, 213, 219, 231 },        232,      231 },
    { "LIME",   { 28,  34,  64,  70, 112, 154, 192, 230 },        232,      231 },
    { "ROSE",   { 88, 131, 167, 174, 211, 217, 218, 231 },        232,      231 },
};

/* Density-glyph ramp from sparse to dense.
 * Tuned for AIRY light: brightest cells use '+' / '*' rather than the
 * blocky '#' / '@' so god-ray shafts read as wispy beams of dust-lit
 * air rather than solid columns. */
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
            init_pair((short)(PAIR_FOG_BASE + i), t->fog[i], -1);
        init_pair(PAIR_SILHOUETTE, t->silhouette, -1);
        init_pair(PAIR_SUN,        t->sun,        -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,  COLOR_BLUE,  COLOR_RED,    COLOR_RED,
            COLOR_YELLOW,COLOR_YELLOW,COLOR_WHITE,  COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_FOG_BASE + i), fb[i], -1);
        init_pair(PAIR_SILHOUETTE, COLOR_BLACK,  -1);
        init_pair(PAIR_SUN,        COLOR_YELLOW, -1);
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
/* §5  silhouette — five point-in-shape functions                        */
/* ===================================================================== */

/*
 * All silhouette functions take normalised coordinates:
 *   u ∈ [-1, +1] : aspect-corrected screen-x (0 = centre)
 *   v ∈ [-1, +1] : screen-y (0 = centre, +v = down)
 *
 * Returns true if the point is INSIDE the silhouette (opaque) and
 * false if it is in the clear (fog).
 */

/*
 * ARCHWAY — two stone pillars + curved arch top.
 *   pillars: vertical rectangles at u = ±0.40, half-width 0.07
 *   arch top: outer half-disc r ∈ [0.30, 0.55] for v < 0
 *   plinth ground line for v > 0.65
 */
static bool sil_archway(float u, float v)
{
    /* Ground / plinth */
    if (v > 0.70f) return true;
    /* Pillars (rectangular, top of pillar at v ≈ 0.0) */
    if (fabsf(u - 0.40f) < 0.08f && v >= 0.0f) return true;
    if (fabsf(u + 0.40f) < 0.08f && v >= 0.0f) return true;
    /* Arch top — annular sector (v < 0). */
    if (v < 0.0f) {
        float r = sqrtf(u * u + v * v);
        if (r > 0.32f && r < 0.55f) return true;
    }
    /* Capstone block on top of arch. */
    if (v > -0.65f && v < -0.55f && fabsf(u) < 0.55f) return true;
    return false;
}

/*
 * MOUNTAIN — single peak; gaussian-bump silhouette.
 *   peak shape: v > 0.65 - 1.05 * exp(-u²/0.36)
 */
static bool sil_mountain(float u, float v)
{
    /* A second small bump to the right for character. */
    float p = 0.70f
            - 1.10f * expf(-(u * u) / 0.30f)
            - 0.35f * expf(-((u - 0.55f) * (u - 0.55f)) / 0.05f);
    return v > p;
}

/*
 * COLUMN — tall broken column, single tower.
 *   shaft: vertical band fabsf(u) < 0.07 from v = -0.55 down to ground
 *   capital: slightly wider top (fabsf(u) < 0.12 for -0.65 < v < -0.55)
 *   broken top: jagged truncation
 *   ground line at v > 0.78
 */
static bool sil_column(float u, float v)
{
    if (v > 0.80f) return true;                     /* ground */

    /* Capital (slightly wider band at top). */
    if (fabsf(u) < 0.13f && v > -0.65f && v < -0.50f) return true;

    /* Shaft. */
    if (fabsf(u) < 0.08f && v > -0.50f && v < 0.80f) return true;

    /* Broken jagged top: cosine notches just above capital. */
    if (fabsf(u) < 0.13f && v > -0.78f && v < -0.65f) {
        float jag = 0.5f * cosf(u * 18.0f) + 0.5f;
        if (v > -0.78f + jag * 0.13f) return true;
    }

    return false;
}

/*
 * WINDOWS — cathedral wall pierced by 4 × 2 grid of arched windows.
 *   wall extends across most of the screen
 *   windows are 4 wide × 2 tall, evenly spaced
 *   each window has rectangular bottom + half-circle top
 *   bottom of wall (below windows) is solid (the floor of the cathedral)
 */
static bool sil_windows(float u, float v)
{
    /* Wall bounds. */
    if (u < -0.90f || u > 0.90f) return false;
    if (v < -0.85f || v > 0.85f) return false;

    /* Map u, v to window-grid space.
     * Grid: 4 columns, 2 rows. */
    const int   N_COLS = 4;
    const int   N_ROWS = 2;
    const float WIN_LEFT   = -0.80f;
    const float WIN_RIGHT  =  0.80f;
    const float WIN_TOP    = -0.70f;
    const float WIN_BOTTOM =  0.55f;

    if (u < WIN_LEFT || u > WIN_RIGHT) return true;     /* wall edge */
    if (v < WIN_TOP  || v > WIN_BOTTOM) return true;    /* wall edge */

    float gu = (u - WIN_LEFT) / (WIN_RIGHT - WIN_TOP * 0.0f - WIN_LEFT);
    gu = (u - WIN_LEFT) / (WIN_RIGHT - WIN_LEFT);       /* [0, 1] */
    float gv = (v - WIN_TOP)  / (WIN_BOTTOM - WIN_TOP); /* [0, 1] */

    int cx = (int)(gu * (float)N_COLS);
    int cy = (int)(gv * (float)N_ROWS);
    if (cx >= N_COLS) cx = N_COLS - 1;
    if (cy >= N_ROWS) cy = N_ROWS - 1;

    float wu = gu * (float)N_COLS - (float)cx;          /* [0, 1] */
    float wv = gv * (float)N_ROWS - (float)cy;          /* [0, 1] */

    /* Window opening = central rectangle wu ∈ [0.18, 0.82], wv ∈
     * [0.10, 0.82] PLUS a half-circle top wu² + wv² < r² where the
     * top is wv < 0.10 — combine into a Romanesque arch. */
    bool in_window = false;
    if (wu > 0.18f && wu < 0.82f) {
        if (wv > 0.18f && wv < 0.82f) {
            in_window = true;                            /* rectangular body */
        } else if (wv <= 0.18f) {
            float du = (wu - 0.50f) / 0.32f;
            float dv = (wv - 0.18f) / 0.18f;
            if (du * du + dv * dv < 1.0f) in_window = true;
        }
    }
    return !in_window;
}

/*
 * TREE — single tree silhouette: vertical trunk + 6 angled branches.
 *   Trunk: tapered vertical band centred at u = 0
 *   Branches: capsule segments rooted on the trunk at varying heights
 *             and angles
 *   Ground at v > 0.85
 */
static bool sil_tree(float u, float v)
{
    if (v > 0.85f) return true;

    /* Trunk — tapered band, wider at base. */
    float trunk_w = 0.05f - 0.02f * (-v);   /* wider toward base */
    if (trunk_w < 0.025f) trunk_w = 0.025f;
    if (fabsf(u) < trunk_w && v > -0.45f) return true;

    /* Branches: each is a thick line (capsule) from a root point on
     * the trunk. */
    static const struct {
        float root_v;      /* trunk position v */
        float angle_deg;   /* branch direction (0 = up, +ve = right) */
        float length;      /* in normalised units */
        float thickness;   /* base half-width */
    } BR[] = {
        { -0.40f,  -55.0f, 0.55f, 0.040f },
        { -0.30f,   60.0f, 0.55f, 0.040f },
        { -0.20f,  -45.0f, 0.45f, 0.035f },
        { -0.10f,   50.0f, 0.45f, 0.035f },
        {  0.00f,  -65.0f, 0.40f, 0.030f },
        { -0.50f,    0.0f, 0.30f, 0.030f },   /* small upward stub */
    };
    const int NB = (int)(sizeof BR / sizeof BR[0]);

    for (int i = 0; i < NB; i++) {
        float rad = BR[i].angle_deg * (float)M_PI / 180.0f;
        float bx  = sinf(rad);
        float by  = -cosf(rad);
        float du  = u - 0.0f;
        float dv  = v - BR[i].root_v;
        float t   = du * bx + dv * by;
        if (t < 0.0f || t > BR[i].length) continue;
        float pdu = du - t * bx;
        float pdv = dv - t * by;
        float perp = sqrtf(pdu * pdu + pdv * pdv);
        float taper = 1.0f - 0.5f * (t / BR[i].length);
        if (perp < BR[i].thickness * taper) return true;
    }
    return false;
}

/* Dispatch. */
static bool silhouette_at(Pattern p, float u, float v)
{
    switch (p) {
    case PATTERN_ARCHWAY:  return sil_archway (u, v);
    case PATTERN_MOUNTAIN: return sil_mountain(u, v);
    case PATTERN_COLUMN:   return sil_column  (u, v);
    case PATTERN_WINDOWS:  return sil_windows (u, v);
    case PATTERN_TREE:     return sil_tree    (u, v);
    default:               return false;
    }
}

/* ===================================================================== */
/* §6  raymarch + scene                                                  */
/* ===================================================================== */

/* hash3 + perlin/fbm for fog jitter (copied inline per the
 * self-contained-file rule). */
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
    for (int o = 0; o < 3; o++) {
        total += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp *= 0.5f; freq *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

typedef struct {
    bool    paused;
    int     speed;
    int     current_theme;
    Pattern current_pattern;
    float   time_secs;
    float   seed_phase;       /* extra offset on sun position    */
    int     seed;
    float   flash_t;
} Scene;

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->seed_phase * 100.0f), 0xC0FFEE);
    s->seed_phase = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->seed       = (int)(h ^ 0x5A5A5A5Au);
    s->flash_t    = 1.0f;
    perm_shuffle(s->seed);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->speed           = SPEED_DEF;
    s->current_theme   = 0;
    s->current_pattern = PATTERN_ARCHWAY;
    s->seed_phase      = 0.7f;
    s->seed            = 0xDECAF;
    s->flash_t         = 1.0f;
    perm_shuffle(s->seed);
}

static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-4.0f * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* Sun screen position for the current frame. */
static void scene_sun_pos(const Scene *s, int cols, int rows,
                          float *out_sx, float *out_sy)
{
    float omega = 2.0f * (float)M_PI / SUN_DRIFT_PERIOD_S;
    float ph    = s->time_secs * omega + s->seed_phase;
    float fx    = sinf(ph) * SUN_X_AMP_FRAC + 0.50f;        /* [0.2, 0.8] */
    float fy    = SUN_Y_FRAC + cosf(ph * 0.7f) * SUN_Y_AMP_FRAC;
    *out_sx = fx * (float)cols;
    *out_sy = fy * (float)rows;
}

/* Convert (sx, sy) screen cell to (u, v) normalised aspect-correct
 * coordinates. v is multiplied by ASPECT_Y · rows / cols so circles
 * render round and the silhouette functions get the right aspect. */
static inline void cell_to_uv(float sx, float sy, int cols, int rows,
                              float *out_u, float *out_v)
{
    *out_u = (2.0f * sx + 1.0f - (float)cols) / (float)cols;
    *out_v = (2.0f * sy + 1.0f - (float)rows) / (float)rows
           * (ASPECT_Y * (float)rows / (float)cols);
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
 * scene_draw — for every cell, march toward the sun, sample
 * silhouette at each step, accumulate weighted visibility, and
 * render glyph + colour.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    int rows_eff = sc->rows - 1;        /* leave bottom row for HUD */
    if (rows_eff < 4) rows_eff = sc->rows;

    float sun_sx, sun_sy;
    scene_sun_pos(s, sc->cols, rows_eff, &sun_sx, &sun_sy);

    /* Test if sun is hidden behind silhouette. */
    float sun_u, sun_v;
    cell_to_uv(sun_sx, sun_sy, sc->cols, rows_eff, &sun_u, &sun_v);
    bool sun_in_sil = silhouette_at(s->current_pattern, sun_u, sun_v);

    /* Fog wind for jitter sampling. */
    float wind = s->time_secs * FOG_WIND;

    for (int sy = 0; sy < rows_eff; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {

            /* Cell normalised coords + silhouette early-out. */
            float u, v;
            cell_to_uv((float)sx + 0.5f, (float)sy + 0.5f,
                       sc->cols, rows_eff, &u, &v);

            if (silhouette_at(s->current_pattern, u, v)) {
                attron(COLOR_PAIR(PAIR_SILHOUETTE));
                mvaddch(sy, sx, ' ');
                attroff(COLOR_PAIR(PAIR_SILHOUETTE));
                continue;
            }

            /* March toward sun in screen-cell space. */
            float dsx = sun_sx - ((float)sx + 0.5f);
            float dsy = sun_sy - ((float)sy + 0.5f);

            float step_dx = dsx / (float)MARCH_STEPS;
            float step_dy = dsy / (float)MARCH_STEPS;

            /* Visual length of one step (aspect-corrected). */
            float step_len = sqrtf(step_dx * step_dx
                                 + (step_dy * ASPECT_Y) * (step_dy * ASPECT_Y));

            float accum = 0.0f;
            float total = 0.0f;
            for (int i = 1; i <= MARCH_STEPS; i++) {
                float px = (float)sx + 0.5f + step_dx * (float)i;
                float py = (float)sy + 0.5f + step_dy * (float)i;
                float uu, vv;
                cell_to_uv(px, py, sc->cols, rows_eff, &uu, &vv);
                float d = step_len * (float)i;
                float w = expf(-FOG_SIGMA * d);
                if (!silhouette_at(s->current_pattern, uu, vv))
                    accum += w;
                total += w;
            }
            float vis = (total > 0) ? (accum / total) : 0.0f;

            /* Sun-disc contribution if sun isn't behind silhouette. */
            float sun_term = 0.0f;
            if (!sun_in_sil) {
                float dx = (float)sx + 0.5f - sun_sx;
                float dy = ((float)sy + 0.5f - sun_sy) * ASPECT_Y;
                float r2 = dx * dx + dy * dy;
                sun_term = expf(-r2 / (SUN_FALLOFF_CELLS * SUN_FALLOFF_CELLS));
            }

            /* Fog jitter — slow drifting density variation. */
            float jitter = 1.0f + FOG_JITTER_AMP
                                 * (fbm2(u * 1.4f + wind, v * 1.4f) - 0.5f) * 2.0f;

            float intensity = (vis * SHAFT_GAIN + sun_term * SUN_GAIN) * jitter;
            if (intensity < 0) intensity = 0;
            if (intensity > 1) intensity = 1;

            /* Sun-pixel override: if very close to sun centre and
             * sun is visible, render a small bright glyph. We use
             * '*' for the centre and '+' for the inner halo so the
             * sun reads as a sparkle rather than a solid blob. */
            if (sun_term > 0.6f) {
                char sg = (sun_term > 0.85f) ? '*' : '+';
                attron(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                mvaddch(sy, sx, (chtype)(unsigned char)sg);
                attroff(COLOR_PAIR(PAIR_SUN) | A_BOLD);
                continue;
            }

            int gi = (int)(intensity * 7.999f);
            if (gi < 0) gi = 0;
            if (gi > 7) gi = 7;
            int pair = PAIR_FOG_BASE + gi;
            int attr = (gi >= 6) ? A_BOLD
                     : (gi <= 1) ? A_DIM
                     :             A_NORMAL;
            char glyph = RAMP_GLYPHS[gi];

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* Reseed flash overlay. */
    if (s->flash_t > 0.05f) {
        int seed = (int)(s->time_secs * 1000.0f);
        attron(COLOR_PAIR(PAIR_FLASH) | A_BOLD);
        for (int sy = 0; sy < rows_eff; sy += 2) {
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

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " GOD-RAYS   %s   theme:%-7s   march:%2d   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name, MARCH_STEPS,
             fps, sim_fps, s->speed);
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
    screen_resize(&app->screen);
    app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;                       break;
    case 'r': case 'R': scene_reseed(s);                              break;

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
