/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * solar_eclipse.c
 *   — Two ray-sphere tests + corona overlay + diamond-ring flash.
 *     A bright limb-darkened sun sits centred; a dark moon sphere
 *     (closer to the camera) drifts across it on a sin-orbit. When
 *     the moon fully covers the sun, the corona blooms outward as a
 *     soft radial halo; at the precise moment first/last contact
 *     transitions to/from totality, a brief bright bead — the
 *     diamond ring — flashes at the sun edge opposite the moon.
 *
 * DEMO: A glowing golden sun fills ~40 cells across the screen,
 *       darker at its limb (limb darkening), brightest at its
 *       centre. A dark moon drifts in from one side. As the moon
 *       slides over the sun:
 *
 *         - the bright photosphere shrinks to a crescent
 *         - just as the moon swallows the last bit of crescent, a
 *           tiny bright BEAD flashes at the sun's edge — DIAMOND RING
 *         - the moon is now fully over the sun: TOTALITY. The bright
 *           sun body is gone. Around the moon's silhouette, a soft
 *           radial CORONA halos out to ~2× the sun radius.
 *         - the moon continues past, and at the other edge another
 *           diamond ring flashes at the moment totality ends
 *         - the sun fades back in as a crescent, then a partial
 *           disc, then full
 *
 *       The cycle loops continuously over ~30 s.
 *
 *       PATTERN (n / N):
 *
 *         TOTAL    moon angular size slightly > sun (1.10×). Full
 *                  totality with corona + diamond ring.
 *         PARTIAL  moon high enough that it never fully crosses the
 *                  sun's centre. Crescent only; no totality, no
 *                  corona, no diamond.
 *         ANNULAR  moon angular size < sun (0.80×). At maximum the
 *                  moon sits inside the sun and a bright RING of
 *                  photosphere remains visible — no corona, the
 *                  bright ring outshines it.
 *         TRANSIT  moon angular size << sun (0.12×). A tiny dot
 *                  crawls across the sun's face — Mercury or Venus
 *                  transit, no occlusion of any consequence.
 *
 *       'r' reseeds (random orbit phase + small y-offset).
 *
 * Study alongside:
 *   saturn_with_rings.c — same ray-sphere intersection routine and
 *                          shadow-test idiom; this file uses two
 *                          spheres but no shadow ray (corona is a
 *                          per-pixel screen-space halo, not a
 *                          traced volume).
 *
 * Section map:
 *   §1 config   — constants, themes (sun + corona palettes)
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair theme ramps + accent pairs
 *   §5 eclipse  — V3 math, ray-sphere, sun shading, corona, phase
 *                 machine, diamond-ring detection
 *   §6 scene    — Scene state, scene_tick (advance time)
 *   §7 screen   — per-cell composited render + HUD
 *   §8 app      — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume orbit
 *   r          reseed (orbit phase + small y-offset)
 *   n / N      next pattern  (TOTAL → PARTIAL → ANNULAR → TRANSIT)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster orbit
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/solar_eclipse.c \
 *       -o solar_eclipse -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Per-pixel raytracing of TWO spheres (sun + moon)
 *                  with a screen-space CORONA overlay and a precise
 *                  DIAMOND-RING bead at the totality boundary.
 *
 *                  For every screen cell:
 *                    1. Build a view ray from the camera.
 *                    2. Intersect with the SUN sphere → t_sun.
 *                    3. Intersect with the MOON sphere → t_moon.
 *                    4. Depth-sort: smaller t wins; the loser is
 *                       OCCLUDED. Moon is positioned closer to the
 *                       camera (smaller z) than the sun, so when
 *                       the moon's silhouette covers the sun in
 *                       angular terms, t_moon < t_sun and the moon
 *                       wins.
 *                    5. Sun visible  → shade with limb darkening.
 *                       Moon visible → render dark silhouette.
 *                       Neither     → render space + corona overlay.
 *                    6. CORONA: even when neither sphere is hit by
 *                       the view ray, a per-pixel computation gives
 *                       the corona brightness — exp(-d/σ) where d is
 *                       the angular distance from the view ray to
 *                       the sun's centre direction. Multiplied by
 *                       SUN-OCCLUSION FACTOR so the corona only
 *                       appears at meaningful brightness during
 *                       totality.
 *                    7. DIAMOND RING: when (in TOTAL pattern) the
 *                       moon-sun angular separation crosses the
 *                       totality boundary |moon_α − sun_α|, a brief
 *                       window fires a bright BEAD at the sun edge
 *                       opposite the moon's direction. The bead
 *                       fades over a few frames.
 *
 *                  Note that we use rays in 3-D for the geometry
 *                  (so we get true depth-sort and proper occlusion)
 *                  but the corona uses a SCREEN-SPACE radial halo
 *                  — that's not "ray-marched volumetric" but it
 *                  matches what an observer SEES (the corona is
 *                  optically thin and projects to a 2-D halo around
 *                  the moon's silhouette).
 *
 * Data-structure : NONE persistent. Each frame is a pure function
 *                  of (cols, rows, time, pattern, seed, theme).
 *
 * Rendering      : ASCII only. Density-glyph ramp `' .,:-^#@'` +
 *                  theme ramp pair + optional A_BOLD/A_DIM. No
 *                  background-colour fill.
 *
 * Performance    : Two ray-sphere tests + a few transcendentals
 *                  per cell. ~50 ns per cell on a modern CPU.
 *                  240×80 × 30 fps ≈ 1.5 ms shading per frame, well
 *                  under the 33 ms frame budget.
 *
 * References     :
 *   • Wikipedia — Solar eclipse
 *     https://en.wikipedia.org/wiki/Solar_eclipse
 *   • Wikipedia — Corona (the sun's outer atmosphere)
 *     https://en.wikipedia.org/wiki/Corona
 *   • Wikipedia — Baily's beads / diamond ring effect
 *     https://en.wikipedia.org/wiki/Baily%27s_beads
 *   • Shirley, P. — "Ray Tracing in One Weekend" — canonical
 *     ray-sphere intersection.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Two spheres on the optical axis (well — one is, one drifts).
 * Whichever is closer along the view ray covers the other. The
 * remarkable visual of an eclipse is caused entirely by the moon
 * being JUST big enough — angularly — to swallow the sun's bright
 * disc, leaving the much fainter corona visible for a few seconds
 * of totality. We reproduce that by keeping the corona at zero
 * brightness whenever the bright sun body is visible (the eye
 * adapts to the photosphere and corona becomes invisible) and
 * unlocking it only as the moon occludes more and more of the sun.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * The sun is a desk lamp behind a paper-thin halo of mist (the
 * corona). Normally the bulb is too bright to see the mist.
 * Slide a small piece of card in front of the bulb — the card
 * blocks the bulb but the mist around the silhouette becomes
 * visible. That visibility ratio is the SUN OCCLUSION FACTOR. The
 * card has to be JUST big enough; too small and the bulb still
 * outshines the mist (annular eclipse: bright ring around the
 * card); a tiny card makes no difference (transit). The diamond
 * ring is the moment a sliver of bulb is still visible at the
 * card's edge — a tiny intense flash before the card fully blocks
 * everything.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. ORBIT. moon position drifts in world x via sin(ω·t). For
 *     PARTIAL pattern, add a small constant y offset so the moon
 *     never centres on the sun.
 *
 *  2. CAMERA. Camera at origin, looking +z. Sun centred at
 *     (0, 0, SUN_Z). Moon at (moon_x, moon_y, MOON_Z) with
 *     MOON_Z << SUN_Z (moon is much closer — that's why a small
 *     moon can occlude a vastly larger sun).
 *
 *  3. PER CELL (sx, sy):
 *     a. View ray d = camera_ray(sx, sy).
 *     b. ray-sphere(sun)  → t_sun
 *     c. ray-sphere(moon) → t_moon
 *     d. moon_in_front = moon_hit AND (¬sun_hit OR t_moon < t_sun).
 *     e. If moon_in_front: render moon silhouette (dark glyph),
 *        and continue to corona overlay (corona renders OUTSIDE
 *        moon disc, see step f).
 *     f. Else if sun_hit: shade sun with limb darkening.
 *     g. Else: pure space — corona overlay computes brightness.
 *
 *  4. CORONA OVERLAY (per cell, regardless of which sphere wins).
 *     Compute angular distance from view ray to direction-to-sun:
 *       cos α = ray_d · normalize(sun_pos - cam_pos)
 *       α     = acos(cos α)
 *     Then:
 *       d_outward     = α - sun_apparent_r            (≥ 0 outside)
 *       corona_local  = exp(-d_outward · CORONA_K)
 *       corona_global = pow(occlusion, CORONA_GAMMA)  (0..1)
 *       corona        = corona_local · corona_global
 *
 *  5. DIAMOND RING (TOTAL only). Compute screen-projected sun and
 *     moon centres; compute their separation. When
 *     |sep − |moon_r − sun_r|| < DIAMOND_W and moon is BIGGER than
 *     sun: a bead lights up at the sun edge OPPOSITE the moon.
 *     Fades smoothly on either side of the threshold.
 *
 *  6. RENDER. Pick glyph from intensity ramp + colour from theme
 *     ramp (sun, corona, moon, space pairs).
 *
 * KEY FORMULAS
 * ────────────
 *  Moon orbit (pattern-specific):
 *    moon_x = MOON_ORBIT_R · sin(ω · t)              ω = 2π / PERIOD
 *    moon_y = MOON_Y_OFFSET                          (PARTIAL ≠ 0)
 *
 *  Apparent angular radii (small-angle):
 *    sun_α  = atan(SUN_R  / SUN_Z)
 *    moon_α = atan(MOON_R / MOON_Z)
 *
 *  Sun-moon angular separation (per frame):
 *    cos β  = normalize(sun_pos) · normalize(moon_pos)
 *    sep    = acos(cos β)
 *
 *  Sun occlusion factor (overlap fraction):
 *    if sep ≥ sun_α + moon_α                : occlusion = 0
 *    else if sep ≤ |sun_α − moon_α|         : if (moon_α ≥ sun_α): 1
 *                                              else: (moon_α/sun_α)²
 *    else                                    : (sun_α + moon_α − sep)
 *                                              / (2·min(sun_α,moon_α))
 *
 *  Limb darkening (Eddington's approximation, simplified):
 *    cos μ  = N · -ray_d                              (N = sphere normal)
 *    L      = 0.40 + 0.60 · cos μ
 *
 *  Corona brightness at view ray:
 *    α_view = acos(ray_d · sun_dir)
 *    d_out  = max(0, α_view − sun_α)
 *    local  = exp(-d_out · CORONA_K)
 *    global = pow(occlusion, CORONA_GAMMA)
 *    corona = local · global · CORONA_BOOST
 *
 *  Diamond-ring bead (TOTAL only):
 *    sep_T  = |moon_α − sun_α|        (totality boundary)
 *    if |sep − sep_T| < DIAMOND_W AND moon_α > sun_α:
 *      bead_dir_2D = -normalize(moon_screen - sun_screen)
 *      bead_pos    = sun_screen + bead_dir_2D · sun_r_screen
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DEPTH MUST BE TESTED. The moon MUST be closer to the camera
 *    than the sun, otherwise the depth-sort never lets the moon
 *    win. Set MOON_Z to a small fraction of SUN_Z. Also make the
 *    moon's WORLD radius small enough that its angular size is
 *    only 1.0× the sun's (TOTAL) — too big and the moon dominates
 *    everything.
 *
 *  • OCCLUSION ZERO INSIDE TOTALITY. When sep < |sun_α − moon_α|
 *    AND moon_α > sun_α, the sun is FULLY covered → occlusion = 1.
 *    For ANNULAR (moon_α < sun_α) the sun ring ALWAYS shows; the
 *    max occlusion is (moon_α/sun_α)² < 1, so corona never reaches
 *    full brightness — exactly right (annular eclipses don't show
 *    corona).
 *
 *  • CORONA CLIPS INSIDE MOON. The corona is computed for ALL
 *    cells (including those where the moon wins the depth-sort),
 *    but cells INSIDE the moon's silhouette must NOT show corona —
 *    the corona is the OUTER atmosphere, visible only outside the
 *    occluder. We gate corona rendering by "this cell is OUTSIDE
 *    both spheres" → only over space pixels.
 *
 *  • ASPECT RATIO. Terminal cells are 2× taller than wide. The
 *    fov_v computation accounts for this so the sun renders ROUND
 *    on screen — without it, sun looks like a vertical ellipse.
 *
 *  • DIAMOND-RING WINDOW. The window |sep − sep_T| < DIAMOND_W is
 *    in radians. With DIAMOND_W = 0.001 and sep changing by ~0.003
 *    rad/sec at speed 1, the window fires for ~0.7 sec each time
 *    it's crossed — visible but not lingering.
 *
 *  • PARTIAL: NEVER FULL TOTALITY. With MOON_Y_OFFSET > 0 the
 *    minimum sep > 0, so the inner condition sep ≤ |sun_α − moon_α|
 *    never triggers, occlusion peaks below 1, corona stays dim,
 *    diamond ring never fires. Good.
 *
 *  • TRANSIT (tiny moon): occlusion is ALWAYS tiny, so corona
 *    factor stays 0; no diamond ring. The moon just appears as a
 *    small dark dot drifting across a brightly lit sun.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Eclipse freezes mid-cycle. Resume: orbit
 *    continues from where it stopped.
 *
 *  • TOTAL pattern. Watch one full cycle:
 *      pre-eclipse — bright sun
 *      moon enters from one side
 *      crescent shrinks until it's a thin sliver
 *      DIAMOND RING flashes — bright bead at sun's far edge
 *      TOTALITY — sun body dark, corona blooms outward
 *      DIAMOND RING flashes again on the other side
 *      moon exits — sun reappears
 *
 *  • PARTIAL pattern. Crescent forms but the moon never centres.
 *    No corona, no diamond.
 *
 *  • ANNULAR pattern. At maximum the moon sits INSIDE the sun
 *    silhouette and a bright RING of sun remains visible all the
 *    way around. The bright ring is what makes annular eclipses
 *    famous; corona is invisible because the bright ring outshines
 *    it.
 *
 *  • TRANSIT pattern. A tiny dark dot crawls across the sun's
 *    face. The sun's brightness barely changes. Mercury / Venus.
 *
 *  • Theme cycle (t/T). Sun, corona, and space tints all change.
 *
 *  • Speed (+/−). Doubling speed halves the cycle period.
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
    PAIR_SUN_BASE       =   3,    /* +0..+7 = 8 sun limb tints          */
    PAIR_CORONA_BASE    =  11,    /* +0..+7 = 8 corona tints            */
    PAIR_MOON           =  19,    /* moon silhouette                    */
    PAIR_SPACE          =  20,    /* deep space                         */
    PAIR_DIAMOND        =  21,    /* diamond-ring flash                 */
    PAIR_FLASH          =  22,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Geometry — world units. */
#define SUN_Z              50.0f
#define MOON_Z              5.0f
#define SUN_R               5.0f       /* world radius of sun           */
#define MOON_BASE_R_TOTAL   (0.55f)    /* moon angular = 1.10 × sun     */
#define MOON_BASE_R_ANNULAR (0.40f)    /* moon angular = 0.80 × sun     */
#define MOON_BASE_R_TRANSIT (0.06f)    /* moon angular = 0.12 × sun     */

/* Camera — at origin looking +z. */
#define FOV_H             0.20f        /* tan of half horizontal FOV    */
#define ASPECT_Y          2.0f

/* Orbit. */
#define ECLIPSE_PERIOD_S  30.0f
#define MOON_ORBIT_X_FRAC 1.5f         /* orbit radius as fraction of   */
                                       /* (sun_α + moon_α) projected.   */
#define PARTIAL_Y_OFFSET  0.025f       /* radians y-offset for PARTIAL  */

/* Limb darkening — Eddington-style. */
#define LIMB_AMBIENT      0.40f
#define LIMB_GAIN         0.60f

/* Corona. */
#define CORONA_K          16.0f        /* exp(-d · K) angular falloff   */
#define CORONA_GAMMA      4.0f         /* power for occlusion ramp      */
#define CORONA_BOOST      1.40f        /* overall brightness multiplier */

/* Diamond ring. */
#define DIAMOND_W         0.0035f      /* radians around sep_T          */
#define DIAMOND_RADIUS    1.6f         /* bead radius in cells          */

/* Stars (very sparse — not the focus). */
#define STAR_DENSITY      300

/* Pattern enum. */
typedef enum {
    PATTERN_TOTAL    = 0,
    PATTERN_PARTIAL  = 1,
    PATTERN_ANNULAR  = 2,
    PATTERN_TRANSIT  = 3,
    N_PATTERNS       = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_TOTAL:   return "TOTAL  ";
    case PATTERN_PARTIAL: return "PARTIAL";
    case PATTERN_ANNULAR: return "ANNULAR";
    case PATTERN_TRANSIT: return "TRANSIT";
    default:              return "?      ";
    }
}

/*
 * Themes — two 8-step ramps:
 *   sun[0..7]    : limb-darkening ramp (0 = darkest near limb, 7 =
 *                  brightest centre)
 *   corona[0..7] : corona ramp (0 = dim outer haze, 7 = bright inner
 *                  halo near silhouette)
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule.
 */
typedef struct {
    const char *name;
    short       sun   [8];
    short       corona[8];
    short       moon;
    short       space;
    short       diamond;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {

    /* name      sun[0..7]                                       corona[0..7]                                  moon space diamond */

    { "GOLDEN",  {130, 166, 172, 208, 214, 220, 226, 231 },     {238, 240, 243, 247, 250, 252, 254, 231 },    232, 233, 231 },
    { "AMBER",   {130, 137, 173, 179, 215, 220, 222, 229 },     {238, 240, 244, 248, 251, 253, 230, 231 },    232, 233, 231 },
    { "WHITE",   {238, 244, 248, 250, 252, 253, 254, 255 },     {238, 240, 243, 246, 249, 252, 254, 255 },    232, 232, 255 },
    { "ROSE",    { 88, 124, 167, 174, 211, 217, 224, 231 },     {238, 240, 244, 247, 250, 252, 224, 231 },    232, 233, 231 },
    { "ICE",     { 24,  31,  67, 110, 117, 153, 195, 231 },     {238, 240, 244, 247, 250, 153, 195, 231 },    232, 233, 231 },
    { "FIRE",    { 88, 124, 130, 166, 196, 208, 214, 226 },     {238, 240, 244, 247, 250, 252, 254, 226 },    232, 233, 231 },
    { "EMBER",   { 52,  88, 124, 130, 166, 202, 208, 220 },     {238, 240, 243, 246, 249, 252, 220, 226 },    232, 233, 231 },
    { "PLASMA",  { 53,  91, 127, 165, 201, 207, 213, 231 },     {238, 240, 244, 248, 251, 213, 219, 231 },    232, 233, 231 },
    { "OCEAN",   { 24,  31,  38,  45,  87, 117, 195, 231 },     {238, 240, 244, 247, 250, 153, 195, 231 },    232, 233, 231 },
    { "MONO",    {238, 240, 243, 246, 249, 252, 254, 255 },     {238, 240, 243, 246, 249, 252, 254, 255 },    232, 232, 255 },
};

/* Density-glyph ramp from sparse to dense.
 *
 * Tuned for AIRY light: brightest cells use '+' / '*' rather than the
 * blocky '#' / '@' so the sun body, corona streamers, and diamond-ring
 * bead all read as light/plasma rather than solid pixel blocks. */
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
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_SUN_BASE    + i), t->sun   [i], -1);
            init_pair((short)(PAIR_CORONA_BASE + i), t->corona[i], -1);
        }
        init_pair(PAIR_MOON,    t->moon,    -1);
        init_pair(PAIR_SPACE,   t->space,   -1);
        init_pair(PAIR_DIAMOND, t->diamond, -1);
    } else {
        static const short fb[8] = {
            COLOR_RED,    COLOR_RED,    COLOR_YELLOW, COLOR_YELLOW,
            COLOR_YELLOW, COLOR_WHITE,  COLOR_WHITE,  COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_SUN_BASE    + i), fb[i],         -1);
            init_pair((short)(PAIR_CORONA_BASE + i), COLOR_WHITE,   -1);
        }
        init_pair(PAIR_MOON,    COLOR_BLACK, -1);
        init_pair(PAIR_SPACE,   COLOR_BLACK, -1);
        init_pair(PAIR_DIAMOND, COLOR_WHITE, -1);
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
/* §5  eclipse — V3 math, ray-sphere, shading, corona, phase            */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3 (float x, float y, float z) { return (V3){x, y, z}; }
static inline V3    v3_add(V3 a, V3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3_sub(V3 a, V3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3_scl(V3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3_dot(V3 a, V3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3 v3_norm(V3 a) {
    float l = sqrtf(v3_dot(a, a));
    if (l < 1e-12f) return v3(0, 0, 0);
    return v3_scl(a, 1.0f / l);
}

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

/*
 * Ray-sphere intersection. Camera at origin, ray direction `rd`
 * (normalised). Returns true on hit; out_t = distance to nearest
 * intersection in front of camera.
 */
static bool ray_sphere(V3 ro, V3 rd, V3 center, float r, float *out_t)
{
    V3    oc   = v3_sub(ro, center);
    float b    = v3_dot(oc, rd);
    float c    = v3_dot(oc, oc) - r * r;
    float disc = b * b - c;
    if (disc < 0) return false;
    float sq = sqrtf(disc);
    float t  = -b - sq;
    if (t < 1e-3f) t = -b + sq;
    if (t < 1e-3f) return false;
    *out_t = t;
    return true;
}

/*
 * occlusion_factor — fraction of sun's apparent area covered by the
 * moon at the current angular separation `sep`. 0 = no overlap, 1 =
 * sun entirely hidden. For ANNULAR (moon_α < sun_α) maxes out at
 * (moon_α / sun_α)².
 */
static float occlusion_factor(float sep, float sun_a, float moon_a)
{
    float sum = sun_a + moon_a;
    float dif = fabsf(sun_a - moon_a);
    if (sep >= sum) return 0.0f;
    if (sep <= dif) {
        if (moon_a >= sun_a) return 1.0f;
        return (moon_a / sun_a) * (moon_a / sun_a);
    }
    /* partial overlap — linear approximation */
    float t = (sum - sep) / (2.0f * fminf(sun_a, moon_a));
    if (moon_a >= sun_a) return t;
    return t * (moon_a / sun_a) * (moon_a / sun_a);
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    /* Pattern-controlled. */
    float moon_world_r;     /* world radius of moon                    */
    float moon_y_world;     /* world y offset for PARTIAL              */
    bool  totality_possible;/* TOTAL only — gate diamond-ring code     */
    bool  annular_mode;     /* ANNULAR — render bright ring at max     */
    bool  show_corona;      /* TOTAL + PARTIAL                         */
} PatternParams;

typedef struct {
    bool          paused;
    int           speed;
    int           current_theme;
    Pattern       current_pattern;
    PatternParams pp;
    float         time_secs;
    float         seed_phase;       /* extra phase per reseed          */
    float         flash_t;
} Scene;

static void pattern_set(Scene *s, Pattern p)
{
    s->current_pattern = p;
    PatternParams *pp = &s->pp;
    pp->moon_world_r      = MOON_BASE_R_TOTAL;
    pp->moon_y_world      = 0.0f;
    pp->totality_possible = false;
    pp->annular_mode      = false;
    pp->show_corona       = false;

    switch (p) {
    case PATTERN_TOTAL:
        pp->moon_world_r      = MOON_BASE_R_TOTAL;
        pp->totality_possible = true;
        pp->show_corona       = true;
        break;
    case PATTERN_PARTIAL:
        pp->moon_world_r      = MOON_BASE_R_TOTAL;
        pp->moon_y_world      = PARTIAL_Y_OFFSET * MOON_Z;
        pp->show_corona       = true;
        /* moon_y_world is converted from radians to world offset by
         * multiplying by MOON_Z (small-angle: y_world ≈ α · z). */
        break;
    case PATTERN_ANNULAR:
        pp->moon_world_r      = MOON_BASE_R_ANNULAR;
        pp->annular_mode      = true;
        break;
    case PATTERN_TRANSIT:
        pp->moon_world_r      = MOON_BASE_R_TRANSIT;
        break;
    case N_PATTERNS: break;
    }
}

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->seed_phase * 100.0f), 0xC0FFEE);
    s->seed_phase = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->flash_t    = 1.0f;
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused           = false;
    s->speed            = SPEED_DEF;
    s->current_theme    = 0;
    s->seed_phase       = 0.0f;
    s->flash_t          = 1.0f;
    pattern_set(s, PATTERN_TOTAL);
}

static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-4.0f * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* Compute moon world position for current frame. */
static V3 scene_moon_pos(const Scene *s)
{
    /* sun_α and moon_α (small-angle) used to compute orbit amplitude. */
    float sun_a  = atanf(SUN_R              / SUN_Z );
    float moon_a = atanf(s->pp.moon_world_r / MOON_Z);
    float orbit_x_world = MOON_Z * (sun_a + moon_a) * MOON_ORBIT_X_FRAC;

    float omega = 2.0f * (float)M_PI / ECLIPSE_PERIOD_S;
    float phase = omega * s->time_secs + s->seed_phase;
    float mx = orbit_x_world * sinf(phase);
    return v3(mx, s->pp.moon_y_world, MOON_Z);
}

/* Phase string for HUD. */
static const char *phase_str_for(const Scene *s)
{
    V3 sun_pos  = v3(0, 0, SUN_Z);
    V3 moon_pos = scene_moon_pos(s);
    V3 sun_dir  = v3_norm(sun_pos);
    V3 moon_dir = v3_norm(moon_pos);
    float cos_b = v3_dot(sun_dir, moon_dir);
    if (cos_b > 1.f)  cos_b = 1.f;
    if (cos_b < -1.f) cos_b = -1.f;
    float sep   = acosf(cos_b);

    float sun_a  = atanf(SUN_R               / SUN_Z );
    float moon_a = atanf(s->pp.moon_world_r  / MOON_Z);
    float sum    = sun_a + moon_a;
    float dif    = fabsf(sun_a - moon_a);

    if (sep >= sum) return "PRE/POST ";
    if (sep <= dif) {
        if (s->pp.annular_mode) return "ANNULARITY";
        if (moon_a >= sun_a)     return "TOTALITY ";
        return "MAX-OCC  ";
    }
    /* partial — approach vs depart based on direction of motion */
    float omega = 2.0f * (float)M_PI / ECLIPSE_PERIOD_S;
    float phase = omega * s->time_secs + s->seed_phase;
    float vx    = cosf(phase);     /* d(moon_x)/dt sign */
    float mx    = sinf(phase);
    bool  departing = (vx * mx) > 0;
    return departing ? "DEPART   " : "APPROACH ";
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

/* Camera state computed once per frame. */
typedef struct {
    V3   pos;
    float fov_h, fov_v;
    int  cols, rows;
} Camera;

static void camera_make(Camera *c, int cols, int rows)
{
    c->cols  = cols;
    c->rows  = rows;
    c->pos   = v3(0, 0, 0);
    c->fov_h = FOV_H;
    c->fov_v = FOV_H * (float)rows * ASPECT_Y / (float)cols;
}

static V3 camera_ray(const Camera *c, int sx, int sy)
{
    float u = ( (2.0f * (float)sx + 1.0f) - (float)c->cols)
            / (float)c->cols * c->fov_h;
    float v = -((2.0f * (float)sy + 1.0f) - (float)c->rows)
            / (float)c->rows * c->fov_v;
    return v3_norm(v3(u, v, 1.0f));
}

/* World-point to screen cell — needed for diamond-ring positioning. */
static void world_to_screen(const Camera *c, V3 p, float *sx_f, float *sy_f)
{
    /* Project onto the screen plane at z=1. */
    float u = p.x / p.z;
    float v = p.y / p.z;
    *sx_f = ((u / c->fov_h) + 1.0f) * 0.5f * (float)c->cols - 0.5f;
    *sy_f = ((-v / c->fov_v) + 1.0f) * 0.5f * (float)c->rows - 0.5f;
}

/*
 * scene_draw — composite per cell:
 *   1. Two ray-sphere tests (sun + moon).
 *   2. Depth-sort.
 *   3. Sun cell  → limb-darkened glyph (or annular ring boost).
 *      Moon cell → dark silhouette.
 *      Space     → corona overlay (if in totality), else dark space
 *                  with optional star.
 *   4. Diamond-ring overlay (TOTAL only): bright bead at sun edge
 *      opposite the moon, when sep ≈ |sun_α − moon_α|.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    Camera cam;
    camera_make(&cam, sc->cols, sc->rows - 1);

    V3 sun_pos  = v3(0, 0, SUN_Z);
    V3 moon_pos = scene_moon_pos(s);

    V3    sun_dir  = v3_norm(sun_pos);
    V3    moon_dir = v3_norm(moon_pos);
    float cos_b    = v3_dot(sun_dir, moon_dir);
    if (cos_b > 1.f)  cos_b = 1.f;
    if (cos_b < -1.f) cos_b = -1.f;
    float sep      = acosf(cos_b);

    float sun_a  = atanf(SUN_R              / SUN_Z );
    float moon_a = atanf(s->pp.moon_world_r / MOON_Z);
    float occlusion = occlusion_factor(sep, sun_a, moon_a);

    /* Diamond ring proximity (TOTAL only — moon_a > sun_a). */
    float diamond_strength = 0.0f;
    float bead_sx = 0, bead_sy = 0;
    if (s->pp.totality_possible && moon_a > sun_a) {
        float sep_T = moon_a - sun_a;
        float w     = fabsf(sep - sep_T);
        if (w < DIAMOND_W) {
            diamond_strength = 1.0f - (w / DIAMOND_W);
            /* Bead direction: opposite of moon offset from sun (at
             * sun's far edge). */
            float sun_sx, sun_sy, moon_sx, moon_sy;
            world_to_screen(&cam, sun_pos,  &sun_sx,  &sun_sy);
            world_to_screen(&cam, moon_pos, &moon_sx, &moon_sy);
            float dx = sun_sx - moon_sx;
            float dy = (sun_sy - moon_sy) * ASPECT_Y;     /* aspect */
            float dl = sqrtf(dx * dx + dy * dy);
            if (dl < 1e-6f) dl = 1.0f;
            /* Sun apparent radius in cells. */
            float sun_r_screen = sun_a / cam.fov_h * (float)cam.cols * 0.5f;
            bead_sx = sun_sx + (dx / dl) * sun_r_screen;
            bead_sy = sun_sy + (dy / dl) * sun_r_screen / ASPECT_Y;
        }
    }

    int top    = 0;
    int bottom = sc->rows - 1;     /* bottom row = HUD */

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {

            V3 ray_d = camera_ray(&cam, sx, sy);

            float t_sun = 0, t_moon = 0;
            bool  sun_h  = ray_sphere(cam.pos, ray_d, sun_pos,  SUN_R,
                                      &t_sun);
            bool  moon_h = ray_sphere(cam.pos, ray_d, moon_pos,
                                      s->pp.moon_world_r, &t_moon);

            int  pair  = PAIR_SPACE;
            char glyph = ' ';
            int  attr  = A_NORMAL;
            bool drew  = false;

            if (moon_h && (!sun_h || t_moon < t_sun)) {
                /* Moon body (closer) — silhouette. */
                pair  = PAIR_MOON;
                glyph = ' ';     /* black space, no character */
                attr  = A_NORMAL;
                drew  = true;
            } else if (sun_h) {
                /* Sun visible — limb darkening. */
                V3 hit = v3_add(cam.pos, v3_scl(ray_d, t_sun));
                V3 N   = v3_norm(v3_sub(hit, sun_pos));
                V3 to_cam = v3_scl(ray_d, -1.0f);
                float mu = v3_dot(N, to_cam);
                if (mu < 0) mu = 0;
                float L = LIMB_AMBIENT + LIMB_GAIN * mu;

                /* ANNULAR: at maximum, bright ring of sun visible.
                 * Boost intensity near limb-edge slightly so the
                 * visible ring reads as "ring of sun". */
                if (s->pp.annular_mode && occlusion > 0.5f && mu < 0.4f)
                    L = fminf(1.0f, L + 0.15f);

                if (L < 0) L = 0;
                if (L > 1) L = 1;

                int gi = (int)(L * 7.999f);
                if (gi < 0) gi = 0;
                if (gi > 7) gi = 7;
                pair  = PAIR_SUN_BASE + gi;
                glyph = RAMP_GLYPHS[gi];
                attr  = (gi >= 6) ? A_BOLD
                      : (gi <= 1) ? A_DIM
                      :             A_NORMAL;
                drew  = true;
            }

            /* ── Corona overlay (only over space cells) ──────────── */
            if (!drew && s->pp.show_corona) {
                /* angular distance from view ray to sun direction */
                float c = v3_dot(ray_d, sun_dir);
                if (c > 1.f)  c = 1.f;
                if (c < -1.f) c = -1.f;
                float a_view = acosf(c);
                float d_out  = a_view - sun_a;
                if (d_out < 0) d_out = 0;
                float local  = expf(-d_out * CORONA_K);
                float global = powf(occlusion, CORONA_GAMMA);
                float corona = local * global * CORONA_BOOST;
                if (corona > 0.05f) {
                    if (corona > 1) corona = 1;
                    int gi = (int)(corona * 7.999f);
                    if (gi < 0) gi = 0;
                    if (gi > 7) gi = 7;
                    pair  = PAIR_CORONA_BASE + gi;
                    glyph = RAMP_GLYPHS[gi];
                    attr  = (gi >= 6) ? A_BOLD
                          : (gi <= 1) ? A_DIM
                          :             A_NORMAL;
                    drew  = true;
                }
            }

            if (!drew) {
                /* Pure space cell — optional sparse star. */
                uint32_t h = hash3(sx, sy, (int)(s->seed_phase * 1000.0f));
                if ((h % STAR_DENSITY) == 0u) {
                    pair  = PAIR_DIAMOND;   /* re-use bright pair */
                    glyph = '.';
                    attr  = A_DIM;
                } else {
                    pair  = PAIR_SPACE;
                    glyph = ' ';
                    attr  = A_NORMAL;
                }
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* ── Diamond-ring bead overlay (drawn on top) ────────────────── */
    if (diamond_strength > 0.01f) {
        attron(COLOR_PAIR(PAIR_DIAMOND) | A_BOLD);
        int rcells = (int)(DIAMOND_RADIUS + 0.5f);
        for (int dy = -rcells; dy <= rcells; dy++) {
            for (int dx = -rcells; dx <= rcells; dx++) {
                float fx = (float)dx;
                float fy = (float)dy * ASPECT_Y;
                float r2 = fx * fx + fy * fy;
                if (r2 > (DIAMOND_RADIUS * DIAMOND_RADIUS) * ASPECT_Y * ASPECT_Y)
                    continue;
                int px = (int)(bead_sx + 0.5f) + dx;
                int py = (int)(bead_sy + 0.5f) + dy;
                if (px < 0 || px >= sc->cols || py < 0 || py >= bottom)
                    continue;
                char g = (r2 < 1.5f) ? '*' : (r2 < 4.0f) ? '+' : '.';
                mvaddch(py, px, (chtype)(unsigned char)g);
            }
        }
        attroff(COLOR_PAIR(PAIR_DIAMOND) | A_BOLD);
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

    const char *state_str = s->paused ? "PAUSED " : pattern_name(s->current_pattern);
    const char *phase_str = phase_str_for(s);

    char buf[220];
    snprintf(buf, sizeof buf,
             " SOLAR-ECLIPSE   %s   phase:%s   theme:%-6s   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, phase_str, themes[s->current_theme].name,
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
        pattern_set(s,
            (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS));
        break;
    case 'p': case 'P':
        pattern_set(s,
            (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS));
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
