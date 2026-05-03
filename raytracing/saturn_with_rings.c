/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * saturn_with_rings.c
 *   — Analytic raytraced ringed planet. One big sphere centred in
 *     view, one flat annulus on the equatorial plane. Each screen
 *     cell is a view ray; depth-sort sphere-vs-ring hits to get
 *     occlusion (ring passes BEHIND the planet), shadow-ray to the
 *     sun to get the dark band the planet casts on the back of the
 *     rings. Slow rotation animates the sun azimuth, sweeping the
 *     day/night terminator across the planet and the shadow stripe
 *     across the rings.
 *
 * DEMO: A huge planet — 40-ish cells across — sits in the middle of
 *       a starfield. A thin flat ring system extends ~80 cells
 *       horizontally on either side, an ellipse in screen space
 *       because we view the rings at a small tilt. The planet has
 *       a clean DAY/NIGHT terminator: half lit, half dark. The
 *       ring system disappears BEHIND the planet (the planet's
 *       silhouette occludes the back of the ring) and the planet
 *       casts a dark SHADOW BAND across the back portion of the
 *       ring nearest the sun's anti-direction. As time advances,
 *       the sun orbits slowly: the terminator sweeps around the
 *       planet, the shadow band sweeps across the ring, and the
 *       lit phase cycles smoothly — never the same frame twice.
 *
 *       PATTERN (n / N):
 *
 *         SATURN     banded cream planet, broad rings with a thin
 *                    Cassini Division gap; the canonical look
 *         URANUS     pale smooth planet, thin rings tilted nearly
 *                    edge-on (rings as a near-line on screen)
 *         RINGED-EARTH  blue-green planet with procedural
 *                    continent / ocean texture, speculative thin
 *                    rings — what Earth would look like if it had
 *                    a Saturn-style ring system
 *         EXOPLANET  per-seed random tint and band frequency, rings
 *                    of random width and density
 *
 *       'r' reseeds (new random orientation, new continent layout
 *       on RINGED-EARTH, new tints on EXOPLANET).
 *
 * Study alongside:
 *   atmospheric_sky.c — same V3 vector math, same theme-ramp +
 *                       glyph-density rendering convention.
 *   sphere_raytrace.c — the foundational ray-sphere intersection
 *                       routine that this file extends with a ring
 *                       plane and shadow rays.
 *
 * Section map:
 *   §1 config    — constants, themes (planet/ring tint pairs)
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme ramp + accent pairs
 *   §5 raytrace  — V3 math, ray-sphere, ray-ring, shadow ray, sun,
 *                  planet shading (latitude bands, continents),
 *                  ring shading (Cassini gap, density modulation)
 *   §6 scene     — Scene state, scene_tick (advance sun)
 *   §7 screen    — per-cell composited render + HUD
 *   §8 app       — signals, resize, fixed-step main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume sun rotation
 *   r          reseed (orientation, continents, exoplanet tints)
 *   n / N      next pattern  (SATURN → URANUS → RINGED-EARTH → EXOPLANET)
 *   p / P      previous pattern
 *   t / T      next / previous theme
 *   + / =      faster rotation
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/saturn_with_rings.c \
 *       -o saturn_with_rings -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Per-pixel analytic raytracing of two primitives —
 *                  a SPHERE (the planet) and a flat ANNULUS (the ring
 *                  system, an axis-aligned plane y=0 clipped to a
 *                  radial band [R_IN, R_OUT]). For each screen cell:
 *
 *                    1. Build a view ray in world space from the
 *                       camera through that pixel.
 *                    2. Intersect the ray with the sphere → t_sphere.
 *                    3. Intersect the ray with the ring plane and
 *                       reject hits outside the annulus → t_ring.
 *                    4. Depth-sort: whichever t is smaller wins. The
 *                       loser is OCCLUDED.
 *                    5. Shade the winner:
 *                       - sphere: Lambert against the sun direction,
 *                         plus latitude-band brightness modulation
 *                         (SATURN), or longitude/latitude continent
 *                         lookup via fBm noise (RINGED-EARTH).
 *                       - ring: per-radius density modulation +
 *                         Cassini Division dimming + SHADOW RAY to
 *                         the sun (does the line from this ring point
 *                         in the sun's direction hit the sphere? if
 *                         yes, this ring point is in the planet's
 *                         shadow → darken).
 *                    6. If neither hit, render the background sky:
 *                       deep "space" colour, optionally a hash-gated
 *                       star.
 *                    7. Map shade intensity → glyph in the project
 *                       density ramp `' .,:-^#@'` and theme ramp
 *                       index → ncurses colour pair.
 *
 *                  The sun is a directional light, animated by an
 *                  azimuthal angle ω·t (with ω = 2π/ROTATION_PERIOD).
 *                  As ω·t advances, the lit hemisphere of the planet
 *                  sweeps around, AND the shadow stripe on the ring
 *                  sweeps with it — both come from the SAME sun
 *                  vector, so they stay perfectly synchronised. This
 *                  consistency is what sells the "3-D" look.
 *
 * Data-structure : NONE persistent. Each frame is a pure function
 *                  of (cols, rows, time, pattern, seed, theme).
 *                  Per-frame: one Scene struct + a tiny camera
 *                  transform. No vertex buffers, no scenegraph, no
 *                  acceleration structure — at two primitives, brute
 *                  force IS the optimum.
 *
 * Rendering      : ASCII only. Project-standard glyph-density ramp
 *                  `' .,:-^#@'` + theme ramp colour pair + optional
 *                  A_BOLD/A_DIM. No background-colour fill, no per-
 *                  pixel pair allocation. Universally compatible.
 *
 * Performance    : One sphere intersection (≈10 mul) + one ring-plane
 *                  intersection (≈5 mul) + at most one shadow ray
 *                  (another ≈10 mul) per cell. ~30-50 ns per cell on
 *                  modern hardware. At 240×80 × 30 fps ≈ 18 M cells/s
 *                  ≈ 1 ms shading per frame, well under the 33 ms
 *                  frame budget.
 *
 * References     :
 *   • Shirley, P. — "Ray Tracing in One Weekend"
 *     https://raytracing.github.io/books/RayTracingInOneWeekend.html
 *     The canonical 100-line ray-sphere intersection in C++.
 *   • Wikipedia — Ring system (astronomy)
 *     https://en.wikipedia.org/wiki/Ring_system_(astronomy)
 *     Saturn's Cassini Division at ~117,500 km, between A and B
 *     rings — the dark gap visible in any decent telescope shot.
 *   • Wikipedia — Saturn
 *     https://en.wikipedia.org/wiki/Saturn — the iconic ringed
 *     planet, source of the "instant recognition" trick this demo
 *     relies on.
 *   • Inigo Quilez — Sphere intersection
 *     https://iquilezles.org/articles/intersectors/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * For every cell on the screen, fire one ray from the camera
 * through that cell into the world. Two things might be in the way:
 * the sphere or the ring. Whichever is closer wins; the other is
 * hidden behind it. To know if a point on the ring is in the
 * planet's shadow, fire a SECOND ray from that ring point toward
 * the sun and check whether the sphere is in the way. That is all
 * the math; everything else is just colour and glyph mapping.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a chocolate-coated marble (the planet) sitting on a
 * paper plate (the ring), photographed from one corner of the room
 * with a flashlight (the sun) on the other side. The marble's lit
 * half faces the flashlight; the other half is dark — that's the
 * Lambertian terminator. The marble blocks light from reaching the
 * far side of the paper plate, casting a dark stripe — that's the
 * shadow ray hitting the sphere. The plate dips behind the marble
 * where the marble is between the camera and the back of the plate
 * — that's the depth-sort. Ring in front: ring wins. Sphere in
 * front: sphere wins. We just do this test for every pixel.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. CAMERA. Place camera at (0, CAM_H, -CAM_D) looking at origin.
 *     Compute basis (forward, right, up). Per cell (sx, sy):
 *
 *       u  = (2·sx + 1 − cols) / cols · fov_h
 *       v  = −(2·sy + 1 − rows) / rows · fov_v
 *       d  = normalize(forward + u·right + v·up)
 *       ray = (origin = cam_pos, dir = d)
 *
 *  2. RAY-SPHERE. With sphere at origin, radius R:
 *
 *       oc   = ray.origin − origin
 *       b    = oc · d
 *       c    = oc·oc − R²
 *       disc = b² − c
 *       if disc < 0: miss
 *       t    = −b − √disc        (front face)
 *       if t < 0: miss            (sphere is behind camera)
 *
 *  3. RAY-RING. Plane y = 0:
 *
 *       if |d.y| < ε: miss        (ray parallel to plane)
 *       t    = −ray.origin.y / d.y
 *       if t < 0: miss            (plane is behind camera)
 *       hit  = ray.origin + t·d
 *       r²   = hit.x² + hit.z²
 *       if r² < R_IN² or r² > R_OUT²: miss   (outside annulus)
 *
 *  4. DEPTH SORT. Whichever t is smaller wins. The other is hidden.
 *
 *  5. SHADE THE WINNER.
 *     Sphere: N = (hit − origin) / R; lambert = max(0, N · L);
 *             apply latitude band (SATURN) or fBm continent map
 *             (RINGED-EARTH).
 *     Ring:   density = base + sin(r · k) variation;
 *             if Cassini gap radius nearby: density *= dim;
 *             shadow = test_shadow_ray(hit, sun);
 *             intensity = density · ring_lambert · (shadow ? dark : 1).
 *
 *  6. NO HIT. Render space cell — dark background, optional
 *     hash-gated star.
 *
 *  7. INTENSITY → GLYPH. Map intensity ∈ [0, 1] to
 *     RAMP_GLYPHS[(int)(I·8)]; pick theme ramp index by the
 *     primitive (planet ramp slots vs ring ramp slots) and shade.
 *
 * KEY FORMULAS
 * ────────────
 *  Sun direction (azimuth ω·t, fixed elevation):
 *    sun_az  = ω · t                      ω = 2π / ROTATION_PERIOD
 *    sun_dir = (cos sun_az,  SUN_ELEV_Y,  sin sun_az), normalised
 *
 *  Lambert (diffuse) on sphere:
 *    N       = normalize(P − O)
 *    lambert = max(0, N · sun_dir)
 *    shade   = AMBIENT + (1 − AMBIENT) · lambert
 *
 *  Latitude bands (SATURN / EXOPLANET):
 *    lat     = N.y                        // [-1, 1]
 *    band    = 1 + BAND_AMP · sin(lat · BAND_FREQ + BAND_PHASE)
 *    shade  *= band
 *
 *  Continent map (RINGED-EARTH):
 *    u       = atan2(N.x, N.z) / π        // longitude [-1, 1]
 *    v       = N.y                         // latitude  [-1, 1]
 *    land    = fbm(u·LAND_FREQ + φ, v·LAND_FREQ) > LAND_THRESH
 *    ramp    = land ? LAND_RAMP_IDX : SEA_RAMP_IDX
 *
 *  Ring radial coordinate (annulus point P_ring):
 *    r       = √(P_ring.x² + P_ring.z²)
 *    θ       = atan2(P_ring.z, P_ring.x)
 *
 *  Ring density (per-radius modulation + Cassini gap):
 *    density = 0.6 + 0.4 · sin(r · RING_BAND_FREQ + θ · 0.3)
 *    if |r − CASSINI_R| < CASSINI_W: density *= CASSINI_DIM
 *
 *  Ring shadow test (ray from ring point toward sun):
 *    shadow_orig = P_ring + ε · sun_dir   // bias to avoid self-hit
 *    in_shadow   = sphere_intersect(shadow_orig, sun_dir).hit
 *
 *  Ring lambert (rings are flat, normal = (0,1,0); use |dot|):
 *    ring_lambert = AMBIENT + (1 − AMBIENT) · |sun_dir.y|
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DEPTH SORT MUST INCLUDE BOTH HITS. If only the sphere is
 *    tested and the ring isn't, the ring is invisible everywhere
 *    the sphere doesn't cover. If only the ring is tested where it
 *    annulus-clips, the ring would show THROUGH the sphere. Always
 *    compute both, then compare.
 *
 *  • RAY-PLANE GRAZING. When |dir.y| is tiny (camera near edge-on
 *    to ring), t blows up and any rounding produces flickering.
 *    Reject |dir.y| < 1e-5 to skip the test entirely; the rings
 *    just don't render that frame in those cells (which is correct —
 *    edge-on rings are infinitely thin).
 *
 *  • SHADOW RAY SELF-HIT. The shadow ray starts ON the ring plane
 *    at hit_xz. Without a small bias along the sun direction, the
 *    sphere intersection may "hit" the ring point itself due to
 *    floating-point round-off. Bias by ε = 1e-3 along sun_dir.
 *
 *  • SUN BEHIND RING. The ring's diffuse lambert uses |sun_dir.y|
 *    not max(0, sun_dir.y) — both the top and bottom faces of the
 *    ring should look lit when the sun is on either side (rings
 *    are double-sided). Without abs, the bottom of the ring goes
 *    black when the sun rises above the equator.
 *
 *  • CAMERA TOO HIGH OR TOO LOW. CAM_HEIGHT controls the ring tilt
 *    on screen. At CAM_HEIGHT = 0, rings collapse to a horizontal
 *    line (no ellipse, hard to read). At CAM_HEIGHT > 3, rings
 *    look nearly circular but the planet looks very low on the
 *    page. CAM_HEIGHT ≈ 1.0–1.5 is the sweet spot.
 *
 *  • ASPECT RATIO. Terminal cells are ~2× taller than they are
 *    wide. Without ASPECT_Y compensation in the v coordinate, the
 *    sphere renders as a vertical ellipse. fov_v = fov_h · rows ·
 *    ASPECT_Y / cols pre-multiplies the y direction so the sphere
 *    is round on screen.
 *
 *  • RAMP IDX OUT OF RANGE. After lambert × band × shadow lookups,
 *    intensity can be < 0 (no — we max(0)) or > 1 (yes, when band
 *    overshoots). Always clamp before indexing into theme.ramp[].
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Pause (space). Sun freezes; the terminator on the planet and
 *    the shadow band on the ring stay aligned. Resume: both
 *    advance together.
 *
 *  • SATURN pattern. You should be able to see the Cassini Division
 *    gap as a thin DARKER ring within the broader bright ring. The
 *    planet should show 4–6 horizontal latitude BANDS of slightly
 *    different brightness.
 *
 *  • Watch the SHADOW BAND. As the sun rotates, the dark stripe
 *    the planet casts on the ring sweeps from the back of the ring
 *    to the front and around. It is always anti-sun-direction from
 *    the planet's centre.
 *
 *  • RINGED-EARTH pattern. Press 'r' a few times. Continent layouts
 *    change but stay structurally Earth-like — irregular landmasses
 *    over an ocean. The terminator falls across both land and sea.
 *
 *  • URANUS pattern. The rings tilt nearly edge-on. They should
 *    appear as a thin near-horizontal LINE crossing in front of and
 *    behind the planet.
 *
 *  • Theme cycle (t/T). The planet ramp, ring ramp, and space
 *    background should all change tint while the structure stays
 *    identical.
 *
 *  • Speed (+/−). Doubling speed should approximately halve the
 *    time the sun takes to complete one full azimuthal sweep.
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
    PAIR_RAMP_BASE      =   3,    /* +0..+7 = 8 generic tints           */
    PAIR_RING_BASE      =  11,    /* +0..+7 = 8 ring tints              */
    PAIR_STAR           =  19,    /* bright white                       */
    PAIR_SUN            =  20,    /* bright sun                         */
    PAIR_SPACE          =  21,    /* deep space (almost-black)          */
    PAIR_FLASH          =  22,
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Planet / ring geometry in WORLD UNITS. Planet radius defines scale. */
#define PLANET_RADIUS         1.00f
#define RING_R_IN_DEFAULT     1.45f
#define RING_R_OUT_DEFAULT    2.55f
#define CASSINI_R_DEFAULT     2.10f      /* radius of Cassini Division */
#define CASSINI_W_DEFAULT     0.05f      /* half-width of the gap      */
#define CASSINI_DIM           0.18f      /* brightness inside Cassini  */
#define RING_BAND_FREQ        18.0f      /* per-radius density bands   */

/* Camera. CAM_DIST = how far back; CAM_HEIGHT = tilt up over rings. */
#define CAM_DIST_DEFAULT      4.7f
#define CAM_HEIGHT_DEFAULT    1.10f
#define FOV_H                 0.55f      /* tan of half horizontal FOV */
#define ASPECT_Y              2.0f       /* terminal cells 2× taller   */

/* Sun rotation (pattern-controlled). */
#define ROTATION_PERIOD_S     30.0f
#define SUN_ELEV_Y            0.18f      /* sun elevation above equator */

/* Lighting. */
#define AMBIENT               0.18f      /* dark side ambient floor    */

/* Stars (background hash density). */
#define STAR_DENSITY          280
#define STAR_TWINKLE_HZ       0.4f

/* TRANSIT pattern animation period (full day-night cycle). */
typedef enum {
    PATTERN_SATURN  = 0,
    PATTERN_URANUS  = 1,
    PATTERN_EARTH   = 2,
    PATTERN_EXO     = 3,
    N_PATTERNS      = 4,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SATURN: return "SATURN  ";
    case PATTERN_URANUS: return "URANUS  ";
    case PATTERN_EARTH:  return "EARTH-R ";
    case PATTERN_EXO:    return "EXO     ";
    default:             return "?       ";
    }
}

/*
 * Themes — each ramp[8] is a low→high SHADE gradient. ramp[0] is the
 * darkest tint we use for "deep ambient" and ramp[7] the brightest
 * "fully lit highlight". The PLANET uses ramp[0..7]; the RINGS use a
 * second 8-step ramp tuned to ring colours (cream / silver / dust).
 *
 * All entries sit in the BRIGHT HALF of the 256-colour cube per the
 * CLAUDE.md "Theme Palette Brightness" rule so even A_DIM cells stay
 * legible.
 */
typedef struct {
    const char *name;
    short       planet[8];
    short       ring  [8];
    short       sun;
    short       star;
    short       space;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* name        planet[0..7]                                 ring[0..7]                                  sun  star space */

    { "SATURN",   { 94, 130, 137, 173, 179, 215, 222, 230 },   { 95, 137, 144, 180, 187, 222, 229, 230 }, 226, 231, 233 },
    { "MARS",     { 88, 124, 130, 166, 172, 208, 215, 222 },   {138, 145, 180, 187, 223, 229, 230, 231 }, 226, 231, 233 },
    { "OCEAN",    { 24,  31,  38,  45,  87, 117, 153, 195 },   {103, 110, 146, 153, 188, 195, 230, 231 }, 226, 231, 233 },
    { "FOREST",   { 28,  34,  40,  64,  70, 112, 156, 192 },   {101, 108, 144, 151, 187, 194, 230, 231 }, 226, 231, 233 },
    { "FIRE",     { 88, 124, 130, 166, 196, 208, 214, 226 },   {130, 137, 173, 180, 215, 222, 229, 231 }, 231, 231, 233 },
    { "ARCTIC",   {  24, 31,  67, 110, 117, 153, 195, 231 },   {103, 110, 146, 153, 188, 195, 224, 231 }, 231, 231, 233 },
    { "VIOLET",   { 53,  54,  91, 134, 135, 176, 213, 219 },   {103, 139, 146, 182, 189, 219, 224, 231 }, 226, 231, 233 },
    { "MONO",     {235, 240, 243, 245, 247, 249, 251, 255 },   {238, 242, 244, 246, 248, 250, 252, 255 }, 226, 231, 232 },
    { "GOLD",     { 94, 130, 137, 178, 179, 220, 221, 229 },   {130, 137, 173, 179, 215, 220, 229, 231 }, 231, 231, 233 },
    { "NEON",     { 53,  91, 134, 165, 207, 213, 219, 231 },   { 60,  98, 135, 171, 213, 219, 224, 231 }, 226, 231, 233 },
};

/* Density-glyph ramp from sparse to dense. Used for shade-to-glyph.
 *
 * Tuned for AIRY look: brightest cells use '+' / '*' rather than the
 * blocky '#' / '@' so the planet's sunlit hemisphere and the bright
 * ring bands read as light/dust rather than solid pixel blocks. */
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
            init_pair((short)(PAIR_RAMP_BASE + i), t->planet[i], -1);
            init_pair((short)(PAIR_RING_BASE + i), t->ring  [i], -1);
        }
        init_pair(PAIR_SUN,   t->sun,   -1);
        init_pair(PAIR_STAR,  t->star,  -1);
        init_pair(PAIR_SPACE, t->space, -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,   COLOR_BLUE,   COLOR_CYAN,   COLOR_CYAN,
            COLOR_WHITE,  COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,
        };
        for (int i = 0; i < 8; i++) {
            init_pair((short)(PAIR_RAMP_BASE + i), fb[i],         -1);
            init_pair((short)(PAIR_RING_BASE + i), COLOR_WHITE,   -1);
        }
        init_pair(PAIR_SUN,   COLOR_YELLOW, -1);
        init_pair(PAIR_STAR,  COLOR_WHITE,  -1);
        init_pair(PAIR_SPACE, COLOR_BLACK,  -1);
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
/* §5  raytrace — V3 math, intersections, shading                        */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3    v3_add (V3 a, V3 b) { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3_sub (V3 a, V3 b) { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3_scl (V3 a, float s) { return v3(a.x*s, a.y*s, a.z*s); }
static inline float v3_dot (V3 a, V3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3_cross(V3 a, V3 b) {
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline V3 v3_norm(V3 a) {
    float l = sqrtf(v3_dot(a, a));
    if (l < 1e-12f) return v3(0, 0, 0);
    return v3_scl(a, 1.0f / l);
}

/* hash3 — drives star placement + per-frame randomness. */
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
 * Used for the RINGED-EARTH continent map and for EXOPLANET tinting. */
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
    for (int o = 0; o < 4; o++) {
        total += amp * perlin2d(x * freq, y * freq);
        max_amp += amp;
        amp *= 0.5f; freq *= 2.0f;
    }
    return (total / max_amp) * 0.5f + 0.5f;
}

/*
 * Ray-sphere intersection. Sphere is at `center` with radius `r`.
 * Returns true if the ray hits in front of the camera (t > epsilon).
 * `out_t` is the distance along ray.dir.
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
    if (t < 1e-3f) t = -b + sq;     /* try far face */
    if (t < 1e-3f) return false;
    *out_t = t;
    return true;
}

/*
 * Ray-plane intersection (plane y = 0) clipped to an annulus
 * R_IN ≤ √(x²+z²) ≤ R_OUT. Returns true on hit; out_t is distance.
 */
static bool ray_ring(V3 ro, V3 rd,
                     float r_in, float r_out,
                     float *out_t, V3 *out_hit)
{
    if (fabsf(rd.y) < 1e-5f) return false;     /* parallel */
    float t = -ro.y / rd.y;
    if (t < 1e-3f) return false;               /* behind camera */
    V3 hit = v3_add(ro, v3_scl(rd, t));
    float r2 = hit.x * hit.x + hit.z * hit.z;
    if (r2 < r_in * r_in)  return false;
    if (r2 > r_out * r_out) return false;
    *out_t   = t;
    *out_hit = hit;
    return true;
}

/*
 * Shadow ray test — does the ray (origin + ε·dir, dir) hit the
 * planet sphere? The bias prevents the origin (which lives ON the
 * ring plane) from self-intersecting the sphere by floating-point
 * round-off.
 */
static bool shadow_hit_sphere(V3 origin, V3 dir, V3 sphere_c, float sphere_r)
{
    V3    o    = v3_add(origin, v3_scl(dir, 1e-3f));
    V3    oc   = v3_sub(o, sphere_c);
    float b    = v3_dot(oc, dir);
    float c    = v3_dot(oc, oc) - sphere_r * sphere_r;
    float disc = b * b - c;
    if (disc < 0) return false;
    float t = -b - sqrtf(disc);
    return t > 1e-3f;
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

typedef struct {
    /* Pattern-controlled geometry. */
    float ring_r_in;
    float ring_r_out;
    float cam_height;       /* effective tilt over ring plane         */
    float band_freq;        /* latitude bands on planet (per radian)  */
    float band_amp;         /* band amplitude                         */
    bool  has_continents;   /* RINGED-EARTH vs banded planets         */
    bool  has_cassini;      /* SATURN-style gap                       */
    float cassini_r;
    float cassini_w;
    float ring_density_min; /* base brightness floor inside annulus   */
    int   planet_ramp_lo;   /* low ramp slot for planet shade         */
    int   planet_ramp_hi;
    int   planet_ramp_lo_alt;  /* alternate (sea / "ocean" of EARTH)  */
    int   planet_ramp_hi_alt;
    int   ring_ramp_lo;
    int   ring_ramp_hi;
} PatternParams;

typedef struct {
    bool          paused;
    int           speed;
    int           current_theme;
    Pattern       current_pattern;
    PatternParams pp;
    float         time_secs;
    float         seed_phase;       /* extra phase per reseed         */
    float         continent_phase;  /* shifts EARTH continent map     */
    int           star_seed;
    float         flash_t;
} Scene;

static void pattern_set(Scene *s, Pattern p)
{
    s->current_pattern = p;
    PatternParams *pp = &s->pp;
    /* defaults */
    pp->ring_r_in        = RING_R_IN_DEFAULT;
    pp->ring_r_out       = RING_R_OUT_DEFAULT;
    pp->cam_height       = CAM_HEIGHT_DEFAULT;
    pp->band_freq        = 8.0f;
    pp->band_amp         = 0.18f;
    pp->has_continents   = false;
    pp->has_cassini      = false;
    pp->cassini_r        = CASSINI_R_DEFAULT;
    pp->cassini_w        = CASSINI_W_DEFAULT;
    pp->ring_density_min = 0.55f;
    pp->planet_ramp_lo   = 0;
    pp->planet_ramp_hi   = 7;
    pp->planet_ramp_lo_alt = 0;
    pp->planet_ramp_hi_alt = 7;
    pp->ring_ramp_lo     = 1;
    pp->ring_ramp_hi     = 7;

    switch (p) {
    case PATTERN_SATURN:
        pp->band_freq        = 12.0f;
        pp->band_amp         = 0.20f;
        pp->has_cassini      = true;
        pp->ring_r_in        = 1.45f;
        pp->ring_r_out       = 2.65f;
        pp->cassini_r        = 2.10f;
        pp->cassini_w        = 0.06f;
        pp->cam_height       = 1.10f;
        break;
    case PATTERN_URANUS:
        pp->band_freq        = 4.0f;
        pp->band_amp         = 0.06f;        /* very smooth          */
        pp->has_cassini      = false;
        pp->ring_r_in        = 1.30f;
        pp->ring_r_out       = 1.90f;        /* thin rings           */
        pp->ring_density_min = 0.35f;
        pp->cam_height       = 0.45f;        /* more edge-on         */
        break;
    case PATTERN_EARTH:
        pp->band_freq        = 0.0f;         /* no bands             */
        pp->band_amp         = 0.0f;
        pp->has_continents   = true;
        pp->has_cassini      = false;
        pp->ring_r_in        = 1.40f;
        pp->ring_r_out       = 2.00f;        /* speculative thin     */
        pp->ring_density_min = 0.45f;
        pp->cam_height       = 1.20f;
        /* EARTH uses ring_ramp for SEA, planet ramp for LAND */
        break;
    case PATTERN_EXO:
        /* randomised by seed_phase below */
        pp->band_freq        = 4.0f + ((float)((int)(s->seed_phase * 10) % 100) * 0.18f);
        pp->band_amp         = 0.10f + ((float)(((int)(s->seed_phase * 31)) & 7) * 0.04f);
        pp->ring_r_in        = 1.35f + ((float)(((int)(s->seed_phase * 17)) & 7) * 0.04f);
        pp->ring_r_out       = pp->ring_r_in + 0.6f
                             + ((float)(((int)(s->seed_phase * 13)) & 7) * 0.10f);
        pp->has_cassini      = ((int)(s->seed_phase * 100) & 1) != 0;
        pp->cassini_r        = (pp->ring_r_in + pp->ring_r_out) * 0.5f;
        pp->ring_density_min = 0.40f;
        pp->cam_height       = 0.7f
                             + ((float)(((int)(s->seed_phase * 7)) & 7) * 0.10f);
        break;
    case N_PATTERNS: break;
    }
}

static void scene_reseed(Scene *s)
{
    uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                       (int)(s->seed_phase * 100.0f), 0xC0FFEE);
    s->seed_phase      = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
    s->continent_phase = ((float)((h >> 16) & 0xFFFFu) / 65536.0f) * 8.0f;
    s->star_seed       = (int)(h ^ 0x5A5A5A5Au);
    s->flash_t         = 1.0f;
    perm_shuffle(s->star_seed);
    /* Re-apply current pattern so EXO picks up new randomness. */
    pattern_set(s, s->current_pattern);
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->paused           = false;
    s->speed            = SPEED_DEF;
    s->current_theme    = 0;
    s->seed_phase       = 1.0f;
    s->continent_phase  = 3.0f;
    s->star_seed        = 0xDECAF;
    s->flash_t          = 1.0f;
    perm_shuffle(s->star_seed);
    pattern_set(s, PATTERN_SATURN);
}

static void scene_tick(Scene *s, float dt)
{
    s->flash_t *= expf(-4.0f * dt);
    if (s->paused) return;
    float speed_mul = (float)s->speed / (float)SPEED_DEF;
    s->time_secs += dt * speed_mul;
}

/* Sun direction — animated azimuth, fixed slight elevation. */
static V3 scene_sun_dir(const Scene *s)
{
    float omega = 2.0f * (float)M_PI / ROTATION_PERIOD_S;
    float az    = s->time_secs * omega + s->seed_phase;
    return v3_norm(v3(cosf(az), SUN_ELEV_Y, sinf(az)));
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

/* Camera basis vectors — recomputed once per frame. */
typedef struct {
    V3 pos, fwd, right, up;
    float fov_h, fov_v;
    int cols, rows;
} Camera;

static void camera_make(Camera *c, int cols, int rows, float cam_height)
{
    c->cols = cols;
    c->rows = rows;
    c->pos  = v3(0.0f, cam_height, -CAM_DIST_DEFAULT);
    V3 target = v3(0.0f, 0.0f, 0.0f);
    V3 worldup = v3(0.0f, 1.0f, 0.0f);
    c->fwd   = v3_norm(v3_sub(target, c->pos));
    c->right = v3_norm(v3_cross(c->fwd, worldup));
    c->up    = v3_cross(c->right, c->fwd);
    c->fov_h = FOV_H;
    c->fov_v = FOV_H * (float)rows * ASPECT_Y / (float)cols;
}

static V3 camera_ray(const Camera *c, int sx, int sy)
{
    float u = ( (2.0f * (float)sx + 1.0f) - (float)c->cols)
            / (float)c->cols * c->fov_h;
    float v = -((2.0f * (float)sy + 1.0f) - (float)c->rows)
            / (float)c->rows * c->fov_v;
    V3 dir = v3_add(c->fwd,
                    v3_add(v3_scl(c->right, u),
                           v3_scl(c->up,    v)));
    return v3_norm(dir);
}

/*
 * shade_planet — given the view-ray hit point on the sphere, compute
 * the final ramp index, glyph, and attribute. Handles latitude bands,
 * continent fBm, and lambert.
 */
static void shade_planet(const Scene *s, V3 hit, V3 sun_dir,
                         int *out_pair, char *out_glyph, int *out_attr)
{
    V3 N = v3_norm(hit);     /* sphere is at origin so N = hit / R */

    float lambert = v3_dot(N, sun_dir);
    if (lambert < 0) lambert = 0;
    float shade = AMBIENT + (1.0f - AMBIENT) * lambert;

    /* Latitude bands (SATURN, EXO). */
    if (s->pp.band_amp > 0.001f) {
        float band = 1.0f + s->pp.band_amp
                          * sinf(N.y * s->pp.band_freq + s->seed_phase * 1.7f);
        shade *= band;
    }

    bool is_land = false;
    if (s->pp.has_continents) {
        float u = atan2f(N.x, N.z) / (float)M_PI;     /* longitude */
        float v = N.y;                                 /* latitude  */
        float land = fbm2(u * 3.5f + s->continent_phase,
                          v * 2.5f + s->continent_phase * 0.7f);
        is_land = (land > 0.55f);
    }

    if (shade < 0) shade = 0;
    if (shade > 1) shade = 1;

    int glyph_i = (int)(shade * 7.999f);
    if (glyph_i < 0) glyph_i = 0;
    if (glyph_i > 7) glyph_i = 7;

    int ramp_lo = is_land ? s->pp.planet_ramp_lo_alt : s->pp.planet_ramp_lo;
    int ramp_hi = is_land ? s->pp.planet_ramp_hi_alt : s->pp.planet_ramp_hi;

    int ramp_idx = ramp_lo + (int)(shade * (float)(ramp_hi - ramp_lo) + 0.5f);
    if (ramp_idx < 0) ramp_idx = 0;
    if (ramp_idx > 7) ramp_idx = 7;

    int pair_base = (s->current_pattern == PATTERN_EARTH && !is_land)
                  ? PAIR_RING_BASE     /* sea uses ring ramp (cool tints) */
                  : PAIR_RAMP_BASE;

    *out_pair  = pair_base + ramp_idx;
    *out_glyph = RAMP_GLYPHS[glyph_i];
    *out_attr  = (glyph_i >= 6) ? A_BOLD
              :  (glyph_i <= 1) ? A_DIM
              :                   A_NORMAL;
}

/*
 * shade_ring — given hit point on the annulus, compute pair, glyph,
 * attr. Includes radial density modulation, Cassini Division, and
 * shadow-ray test against the planet.
 */
static void shade_ring(const Scene *s, V3 hit, V3 sun_dir,
                       int *out_pair, char *out_glyph, int *out_attr)
{
    float r     = sqrtf(hit.x * hit.x + hit.z * hit.z);
    float theta = atan2f(hit.z, hit.x);

    /* Radial density modulation — fine bright/dark bands. */
    float density = s->pp.ring_density_min
                  + (1.0f - s->pp.ring_density_min)
                    * (0.5f + 0.5f * sinf(r * RING_BAND_FREQ
                                          + theta * 0.4f
                                          + s->seed_phase));

    /* Cassini Division — thin dim band at a specific radius. */
    if (s->pp.has_cassini && fabsf(r - s->pp.cassini_r) < s->pp.cassini_w)
        density *= CASSINI_DIM;

    /* Lambert: rings are flat (normal = (0,1,0)). Use abs because
     * rings are double-sided — both top and bottom faces lit. */
    float ring_lambert = fabsf(sun_dir.y);
    float lit = AMBIENT + (1.0f - AMBIENT) * ring_lambert;

    /* Shadow ray — does light from sun reach this ring point? */
    bool shadowed = shadow_hit_sphere(hit, sun_dir,
                                      v3(0, 0, 0), PLANET_RADIUS);
    float shadow_factor = shadowed ? 0.18f : 1.0f;

    float shade = density * lit * shadow_factor;
    if (shade < 0) shade = 0;
    if (shade > 1) shade = 1;

    int glyph_i = (int)(shade * 7.999f);
    if (glyph_i < 0) glyph_i = 0;
    if (glyph_i > 7) glyph_i = 7;

    int ramp_idx = s->pp.ring_ramp_lo
                 + (int)(shade * (float)(s->pp.ring_ramp_hi - s->pp.ring_ramp_lo)
                         + 0.5f);
    if (ramp_idx < 0) ramp_idx = 0;
    if (ramp_idx > 7) ramp_idx = 7;

    *out_pair  = PAIR_RING_BASE + ramp_idx;
    *out_glyph = RAMP_GLYPHS[glyph_i];
    *out_attr  = (glyph_i >= 6) ? A_BOLD
              :  (glyph_i <= 1) ? A_DIM
              :                   A_NORMAL;
}

/*
 * shade_space — background cell. Optional hash-gated star with
 * twinkle. Otherwise an almost-black space glyph.
 */
static void shade_space(const Scene *s, int sx, int sy,
                        int *out_pair, char *out_glyph, int *out_attr)
{
    uint32_t h = hash3(sx, sy, s->star_seed);
    if ((h % STAR_DENSITY) == 0u) {
        float phase = (float)((h >> 16) & 0xFFFFu) / 65536.0f
                    * 2.0f * (float)M_PI;
        float tw = 0.5f + 0.5f * sinf(2.0f * (float)M_PI
                                       * STAR_TWINKLE_HZ
                                       * s->time_secs + phase);
        if (tw > 0.65f) {
            *out_pair  = PAIR_STAR;
            *out_glyph = (tw > 0.85f) ? '*' : '.';
            *out_attr  = (tw > 0.85f) ? A_BOLD : A_NORMAL;
            return;
        }
        if (tw > 0.40f) {
            *out_pair  = PAIR_STAR;
            *out_glyph = '.';
            *out_attr  = A_DIM;
            return;
        }
    }
    *out_pair  = PAIR_SPACE;
    *out_glyph = ' ';
    *out_attr  = A_NORMAL;
}

/*
 * scene_draw — for every cell, build a ray, intersect with planet
 * and ring, depth-sort, shade the winner (or shade as background),
 * and render one glyph. The bottom row is reserved for the HUD.
 */
static void scene_draw(const Screen *sc, const Scene *s)
{
    /* Build camera + sun direction once per frame. */
    Camera cam;
    camera_make(&cam, sc->cols, sc->rows - 1, s->pp.cam_height);
    V3 sun_dir = scene_sun_dir(s);

    int top    = 0;
    int bottom = sc->rows - 1;     /* bottom row = HUD */

    for (int sy = top; sy < bottom; sy++) {
        for (int sx = 0; sx < sc->cols; sx++) {

            V3 ray_d = camera_ray(&cam, sx, sy);

            float t_sphere = 0, t_ring = 0;
            V3    hit_ring = {0, 0, 0};
            bool  hit_sphere_b = ray_sphere(cam.pos, ray_d,
                                            v3(0, 0, 0), PLANET_RADIUS,
                                            &t_sphere);
            bool  hit_ring_b   = ray_ring(cam.pos, ray_d,
                                          s->pp.ring_r_in, s->pp.ring_r_out,
                                          &t_ring, &hit_ring);

            int  pair = PAIR_SPACE;
            char glyph = ' ';
            int  attr = A_NORMAL;

            if (hit_sphere_b && (!hit_ring_b || t_sphere < t_ring)) {
                /* Planet wins. */
                V3 hit = v3_add(cam.pos, v3_scl(ray_d, t_sphere));
                shade_planet(s, hit, sun_dir, &pair, &glyph, &attr);

            } else if (hit_ring_b) {
                /* Ring wins. */
                shade_ring(s, hit_ring, sun_dir, &pair, &glyph, &attr);

            } else {
                /* Background. */
                shade_space(s, sx, sy, &pair, &glyph, &attr);
            }

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

    V3 sd = scene_sun_dir(s);
    float az_deg = atan2f(sd.z, sd.x) * 180.0f / (float)M_PI;
    const char *state_str = s->paused ? "PAUSED" : pattern_name(s->current_pattern);

    char buf[200];
    snprintf(buf, sizeof buf,
             " SATURN-WITH-RINGS   %s   theme:%-8s   sun_az:%+6.1f°   "
             "%5.1f fps  %3d Hz  speed:%-3d   "
             "n/p:pat  t/T:theme  +/-:speed  spc:pause  r:reseed  q:quit ",
             state_str, themes[s->current_theme].name,
             (double)az_deg, fps, sim_fps, s->speed);
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
