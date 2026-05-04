/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb.c — clean 3-D Mandelbulb explorer for the terminal.
 *
 * DEMO: One iconic 3-D Mandelbulb (power = 8), sphere-traced with
 *       Phong shading + soft shadow + ambient occlusion, coloured by
 *       smooth iteration count.  A slow auto-orbit shows the fractal
 *       from every angle; arrow keys override the orbit for manual
 *       inspection.  No starfield, no animated palette, no power
 *       morph, no near-miss glow corona — every pixel of every cell
 *       is dedicated to making the FRACTAL silhouette read clearly.
 *
 *       Designed from first principles for terminal low-res output:
 *       single shading pipeline, single colour-by-iter strategy, no
 *       starfield or palette rotation distractions — every pixel is
 *       used to make the FRACTAL silhouette read clearly.
 *
 *       Themes (cycle with t / T):
 *         CLASSIC   warm orange-red-yellow (Daniel White's iconic look)
 *         ICE       cool blues climbing into white
 *         PLASMA    high-sat magenta → cyan → yellow (fractal-art neon)
 *         MONO      grayscale, no hue distraction — best for shape study
 *         NEGATIVE  white bg + dark fg, photographic-negative inversion
 *
 * Study alongside:
 *   raymarcher/kifs_fractal.c        — the angular cousin (folding
 *                                      fractal); same sphere-trace
 *                                      pipeline, very different
 *                                      visual character (sharp temple
 *                                      architecture vs. organic blobs).
 *   raymarcher/raster/mandelbulb_raster.c — same fractal, polygon-
 *                                      rasterised pipeline (study the
 *                                      contrast in approach).
 *
 * Section map:
 *   §1 config    — themes, raymarch tunables, control rates
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 5-pair theme apply (with inverted-bg support)
 *   §4 vec3      — value-type 3-D math
 *   §5 mandelbulb— DE (with smooth iter), normal estimator
 *   §6 march     — sphere trace + soft shadow + AO + Phong shade
 *   §7 scene     — camera orbit, animation, full-frame render
 *   §8 screen    — ncurses init / draw / resize
 *   §9 app       — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume auto-orbit
 *   r            reset camera
 *   t / T        next / previous theme
 *   ← / →        manual orbit (yaw)
 *   ↑ / ↓        manual orbit (pitch)
 *   z / Z        zoom in / out
 *   i / I        iterations −1 / +1   (depth of fractal; affects detail)
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/mandelbulb.c \
 *       -o mandelbulb -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sphere tracing of the Mandelbulb distance estimator
 *                  (Daniel White & Paul Nylander, 2009).
 *
 *                  ITERATION (per evaluation point p, with z₀ = p):
 *                     for i = 1..N:
 *                         r = |z|
 *                         if r > BAILOUT: break
 *                         (θ, φ) = (acos(z.y/r), atan2(z.z, z.x))
 *                         dr     = r^(power−1) · power · dr + 1
 *                         z      = r^power · (sin(p·θ)·cos(p·φ),
 *                                             cos(p·θ),
 *                                             sin(p·θ)·sin(p·φ)) + p
 *                  DISTANCE ESTIMATE:
 *                     DE(p) = ½ · log(r) · r / dr     (Hubbard-Douady form)
 *                  SMOOTH ITER (for colouring):
 *                     smooth = i + 1 − log₂(log(r)/log(BAILOUT)) / log₂(power)
 *                  This is a CONTINUOUS index (not just the integer i)
 *                  that varies smoothly across the surface — adjacent
 *                  cells get continuous colour, no banding.
 *
 *                  SPHERE TRACE: t = 0; for step = 1..MAX_STEPS:
 *                     d = DE(origin + t · dir)
 *                     if d < HIT_EPS: hit
 *                     t += d · 0.85    // step under-relaxation
 *                  The 0.85 multiplier prevents overshoot at grazing
 *                  angles; without it the trace can step PAST a thin
 *                  surface lobe and miss the hit.
 *
 *                  SHADING:
 *                     N    = central-difference normal of DE(p)
 *                     ndl  = max(0, N · light_dir)
 *                     soft = soft-shadow ray toward light (16 steps,
 *                            tracking min(d/t) for soft penumbra)
 *                     ao   = 1 − (march_steps / MAX_STEPS) · 0.7
 *                            (cells that took many steps are inside
 *                             concavities — natural ambient occlusion)
 *                     L    = AMBIENT + (1 − AMBIENT) · ndl · soft · ao
 *
 *                  PIXEL ENCODING:
 *                     glyph slot = ⌊L · 7.999⌋     (luminance ramp)
 *                     colour slot = ⌊smooth/N · 7.999⌋  (depth palette)
 *
 * Data-structure : Stateless math.  No tables, no LUTs.  Each pixel
 *                  re-evaluates the iteration ~30 times during sphere
 *                  trace + 6 times for normal estimation + 16 times for
 *                  soft shadow.
 *
 * Rendering      : ASCII only.  Glyph from Lambert luminance, colour
 *                  pair from smooth iteration count.  The two are
 *                  decoupled — glyph reads as SHAPE (Phong lighting),
 *                  colour reads as DEPTH (escape iteration).
 *
 * Performance    : ~50 march steps × ~8 iters × ~10 ops = ~4 000 ops
 *                  per pixel for trace; another ~1 500 for shading.
 *                  At 80×24 = 1 920 pixels: ~10 M ops/frame.  Holds
 *                  60 fps comfortably; for 200×60 lower iters with `i`.
 *
 * References     :
 *   • White, D. & Nylander, P. (2009) — original Mandelbulb formulation
 *     and triplex algebra.  https://www.skytopia.com/project/fractal/mandelbulb.html
 *   • Hart, J. C. (1996) — "Sphere Tracing", *The Visual Computer* 12(10).
 *     The sphere-trace iteration we use.
 *   • Hubbard, J. H. & Douady, A. — distance-estimator derivation
 *     (the `½·log(r)·r/dr` form).
 *   • Quílez, I. — [Mandelbulb article](https://iquilezles.org/articles/mandelbulb/).
 *     The clearest derivation of the smooth iteration count and
 *     central-difference normal estimator.
 *   • Christensen, M. H. — "Distance Estimated 3D Fractals — a Tutorial",
 *     parts I–V (2011), syntopia.github.io.  Soft shadow and AO
 *     recipes for distance-estimated fractals.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * The Mandelbulb is a 3-D analogue of the Mandelbrot set.  Each point
 * in 3-D space gets ITERATED through a non-linear function (raise the
 * point to the 8th power in spherical coordinates, add the original
 * point, repeat).  Points whose orbits stay bounded form the fractal's
 * "interior"; points whose orbits diverge are "outside".  We render by
 * sphere-tracing rays through 3-D space; at each step the DISTANCE
 * ESTIMATOR tells us how far the ray can safely advance before
 * possibly hitting the fractal.  Once we hit, lighting + colour give
 * the final pixel.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Think of the fractal as an alien fruit suspended in space.  You
 * shine a flashlight on it from above-and-to-the-right (the LIGHT
 * direction).  Your eye looks at it from a slowly orbiting camera.
 * Each pixel of your view is a ray from the camera toward the fruit;
 * that ray walks in steps until it touches the surface, then asks
 * three questions:
 *   – Which way does the surface face? (normal → shape)
 *   – Is the spot in shadow from another lobe? (soft shadow → drama)
 *   – Is the spot inside a tight crevice? (AO → depth)
 * The answers combine into a brightness number that picks the glyph.
 * A second number — how MANY iterations the spot took to "decide" it
 * was inside the fractal — picks the colour.  Glyph carries shape;
 * colour carries depth.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. CAMERA — orbit-yaw + orbit-pitch advance with time (auto), plus
 *     manual user yaw/pitch from arrow keys.  Camera position is
 *     spherical: distance D from origin at (yaw, pitch).
 *
 *  2. PER PIXEL — compute ray dir from camera basis + screen UV
 *     (with terminal cell aspect 2:1 corrected).
 *
 *  3. SPHERE TRACE — iterate:
 *
 *        t = 0
 *        for step = 1..MAX_STEPS:
 *            p = camera + t · ray_dir
 *            d = mandelbulb_de(p)
 *            if d < HIT_EPS · (1 + t · 0.01):  // adaptive epsilon
 *                hit at p, save smooth iter
 *                break
 *            if t > MAX_T:
 *                miss → background
 *                break
 *            t += d · STEP_RELAX             // 0.85 under-relaxation
 *
 *  4. ON HIT — compute normal via central differences:
 *
 *        N_x = de(p + (ε,0,0)) − de(p − (ε,0,0))
 *        N_y = de(p + (0,ε,0)) − de(p − (0,ε,0))
 *        N_z = de(p + (0,0,ε)) − de(p − (0,0,ε))
 *        N   = normalise(N_x, N_y, N_z)
 *
 *     Cast a soft-shadow ray from p toward LIGHT_DIR; track the
 *     minimum (d / t_along_shadow_ray) — that's the "how close did
 *     this ray come to a surface" → soft penumbra factor.
 *
 *     Compute AO from the original march step count: cells that took
 *     many steps to converge sit inside concavities and darken.
 *
 *  5. COMBINE: L = AMBIENT + (1 − AMBIENT) · ndl · soft · ao.
 *     Glyph slot = ⌊L · 7.999⌋.
 *     Colour slot = ⌊smooth_iter / max_iter · 7.999⌋.
 *
 *  6. ANIMATE — auto-orbit yaw advances with time.  Pause freezes it.
 *     Manual yaw/pitch ADD to the orbital values.
 *
 * KEY FORMULAS
 * ────────────
 *  Mandelbulb iteration in spherical → Cartesian:
 *    z_{n+1} = r^p · (sin(pθ)·cos(pφ),  cos(pθ),  sin(pθ)·sin(pφ))  +  p₀
 *  where (r, θ, φ) = (|z_n|, acos(z_n.y/r), atan2(z_n.z, z_n.x)).
 *
 *  Distance estimator:
 *    DE(p) = ½ · log(r) · r / dr
 *  with dr update each iter:
 *    dr_{n+1} = r^(p−1) · p · dr_n + 1
 *
 *  Smooth iteration count (Quílez):
 *    smooth = i + 1 − log₂(log(r)/log(BAILOUT)) / log₂(power)
 *  ranges from 0 (escapes immediately) to N_ITER (interior).
 *
 *  Sphere-trace step under-relaxation:
 *    t_{k+1} = t_k + α · DE(p_k),   α ∈ (0.5, 1.0]
 *  smaller α = more steps but never overshoot.
 *
 *  Adaptive hit epsilon (cone tracing):
 *    ε_hit(t) = HIT_EPS · (1 + t · ADAPTIVE_FACTOR)
 *
 *  Soft-shadow factor (Christensen):
 *    soft = clamp(min over march of K · d / t,  AMBIENT,  1)
 *
 *  Ambient occlusion from march complexity:
 *    ao = clamp(1 − (steps/MAX_STEPS) · 0.7,  AO_FLOOR,  1)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DE BREAKDOWN AT POWER ≈ 1.  At power < 2 the iteration is nearly
 *    linear and the DE returns nonsense.  We pin power = 8 (default
 *    Mandelbulb).  Power adjustments would need a different DE form.
 *
 *  • r = 0 IN ITERATION.  acos(z.y/r) divides by r.  The very first
 *    iteration with z = origin (r = 0) is fine because we test
 *    `r > BAILOUT` before the spherical conversion — but a recurrent
 *    return to origin would crash.  In practice z grows fast under
 *    iteration so this never happens for points outside a tiny
 *    pathological set.
 *
 *  • OVERSHOOT WITHOUT UNDER-RELAXATION.  At grazing angles the DE
 *    can return a distance larger than the true safe step (the DE
 *    is a LOWER bound on distance, but optimistic near singularities).
 *    α = 0.85 prevents this from breaking the trace.
 *
 *  • ADAPTIVE EPSILON.  Far cells have lower precision needs; their
 *    HIT_EPS scales up with t to avoid wasting steps on sub-cell
 *    detail.  Without this the trace at large t can never converge
 *    because the DE bottoms out at a noisy ε floor.
 *
 *  • ITERATIONS COST.  Each iteration involves 1 acos, 1 atan2, 4
 *    sin/cos, 2 pow.  Doubling N_ITER doubles per-pixel cost.  Default
 *    8 is the sweet spot for terminal resolution; lower with `i` on
 *    large terminals.
 *
 *  • NORMAL EPSILON.  Too small → noisy normals from DE quantisation;
 *    too large → smoothed normals lose surface detail.  4·HIT_EPS is
 *    a good compromise.
 *
 *  • CAMERA INSIDE THE BULB.  Zooming past the hull (cam_dist < ~1.2)
 *    places the camera inside; the trace then walks outward and the
 *    fractal renders inverted.  We clamp `cam_dist ≥ CAM_DIST_MIN`
 *    to keep the camera outside.
 *
 *  • PAUSE freezes orbit, NOT input — you can still manually orbit
 *    and zoom while paused.
 *
 *  • INVERTED THEME (NEGATIVE) needs special handling: the empty
 *    background must be pre-filled with the white bg pair, and the
 *    A_BOLD/A_DIM modulation is disabled (it would invert the
 *    brightness intent on light fg over white bg).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press space.  Auto-orbit freezes.  The fractal holds at its
 *    current angle.  Resume — orbit picks up smoothly.
 *
 *  • Press `r`.  Camera resets to default (auto-orbit yaw=0, manual
 *    offsets cleared, distance default).
 *
 *  • Press `t`.  Theme cycles.  Geometry identical, only colours
 *    change.  CLASSIC → ICE is the most striking flip.
 *
 *  • Press `i` to lower iterations.  At iters=4 the fractal becomes
 *    a smooth blob; at iters=8 (default) it has well-defined lobes
 *    and crevices; at iters=12 finest surface detail visible (also
 *    slowest).
 *
 *  • Press `→` repeatedly.  Manual yaw advances; combined with auto
 *    orbit you get faster rotation.  Hold `←` to slow / reverse.
 *
 *  • Press `↑` / `↓` to look from above / below.  At extreme pitches
 *    you see the bulb's polar lobes — the iconic "head" and "feet".
 *
 *  • Press `z`.  Zoom in until surface detail fills the screen; the
 *    raymarcher's adaptive epsilon should keep edges crisp.  `Z` to
 *    back out.
 *
 *  • At 60×20, the fractal silhouette should still read as 3-D.  If
 *    the bulb looks flat, AO or soft shadow has been disabled — both
 *    are critical for depth perception at low res.
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

    ITERS_MIN        =   3,
    ITERS_MAX        =  14,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD          =  1,
    PAIR_HINT         =  2,
    PAIR_RAMP_BASE    =  3,    /* +0..+7 — depth ramp                  */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h/w                  */

/* Mandelbulb iteration. */
#define MANDELBULB_POWER 8.0f
#define BAILOUT          4.0f      /* |z| > this → escaped              */
#define ITERS_DEFAULT    8

/* Sphere trace. */
#define MAX_STEPS        90
#define HIT_EPS          1.0e-3f
#define ADAPTIVE_FACTOR  0.012f    /* ε grows with t                    */
#define MAX_T            6.0f
#define STEP_RELAX       0.85f     /* under-relaxation factor           */
#define BOUND_RADIUS     1.5f      /* skip rays outside this sphere     */

/* Normal estimator. */
#define NORMAL_EPS       3.5e-3f

/* Soft shadow. */
#define SHADOW_STEPS     16
#define SHADOW_NEAR      0.012f
#define SHADOW_FAR       2.5f
#define SHADOW_K         32.0f     /* hardness                          */
#define SHADOW_FLOOR     0.30f     /* cells in shadow stay this lit     */

/* Lighting. */
#define AMBIENT          0.18f
#define AO_FLOOR         0.35f
#define AO_STRENGTH      0.70f

/* Light direction (will be normalised at setup). */
#define LIGHT_X          0.55f
#define LIGHT_Y          0.75f
#define LIGHT_Z         -0.25f

/* Camera. */
#define CAM_DIST_DEFAULT  3.2f
#define CAM_DIST_MIN      1.5f
#define CAM_DIST_MAX      8.0f
#define CAM_DIST_STEP     0.20f
#define FOV_DEG          45.0f
#define ORBIT_YAW_RATE    0.30f    /* rad / sec auto-orbit              */
#define ORBIT_PITCH_DEF   0.25f    /* default static tilt above equator */
#define MANUAL_YAW_STEP   0.12f    /* per arrow keypress                */
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX  1.30f    /* clamp short of poles              */

/*
 * Themes — each is an 8-tier depth ramp, slot 0 = outermost shell
 * (escape early, low smooth iter) → slot 7 = innermost (escape late
 * / interior).  All entries sit in the bright half of the 256-cube
 * per the CLAUDE.md theme rule, EXCEPT the NEGATIVE theme which uses
 * dark fg over a white bg (handled by the inverted flag).
 */
typedef struct {
    const char *name;
    short       ramp[8];
    bool        inverted;       /* white bg, dark fg, A_BOLD/A_DIM off */
} Theme;

#define N_THEMES 5

static const Theme themes[N_THEMES] = {
    /* CLASSIC: warm crimson → red → orange → amber → yellow → bone.
     * Daniel White's original Mandelbulb images had this palette;
     * still the iconic "alien fruit lit by sunset" look.            */
    { "CLASSIC ",
      { 124, 160, 196, 202, 208, 214, 220, 229 }, false },

    /* ICE: deep teal → bright cyan → ice blue → near-white.         */
    { "ICE     ",
      {  30,  37,  44,  51,  87, 123, 159, 195 }, false },

    /* PLASMA: high-saturation neon arc — magenta → cyan → yellow.   */
    { "PLASMA  ",
      { 125, 165, 207, 213,  87, 123, 220, 229 }, false },

    /* MONO: clean grayscale.  Best for studying fractal shape with
     * zero hue distraction.  No black at slot 0 — visible grey.     */
    { "MONO    ",
      { 240, 244, 247, 250, 252, 253, 254, 231 }, false },

    /* NEGATIVE: photographic-negative inversion.  White bg, dark fg.
     * Cool/outer fractal regions blend into white; deep interior
     * stamps as solid black.  A_BOLD/A_DIM disabled (see CONCEPTS). */
    { "NEGATIVE",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true  },
};

/* Glyph ramp: empty (slot 0) → dense (slot 7).  Slot 0 is `.` not
 * space, so even the dimmest hit pixel paints something.  Background
 * (rays that miss entirely) is handled separately. */
static const char LUMA_GLYPHS[8] = { '.', ',', ':', ';', '+', '*', '#', '@' };

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
    const Theme *t = &themes[idx];
    short bg256 = t->inverted ? 231 : -1;
    short bg8   = t->inverted ? COLOR_WHITE : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], bg256);
    } else {
        static const short fb[8] = {
            COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
            COLOR_RED,  COLOR_RED,  COLOR_YELLOW,  COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_RAMP_BASE + i),
                      t->inverted ? COLOR_BLACK : fb[i], bg8);
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
/* §4  vec3                                                               */
/* ===================================================================== */

typedef struct { float x, y, z; } V3;

static inline V3    v3(float x, float y, float z)   { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)               { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub(V3 a, V3 b)               { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale(float s, V3 a)          { return (V3){s*a.x, s*a.y, s*a.z}; }
static inline float v3dot(V3 a, V3 b)               { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3cross(V3 a, V3 b)             {
    return (V3){ a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x };
}
static inline float v3len(V3 a)                     { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a) {
    float L = v3len(a);
    return L > 1e-12f ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

/* ===================================================================== */
/* §5  mandelbulb — distance estimator + smooth iter + normal             */
/* ===================================================================== */

/*
 * mandelbulb_de — distance estimator with optional smooth iter output.
 *
 * The Hubbard-Douady-style DE formula `0.5·log(r)·r/dr` works for ANY
 * complex iteration where you can track `dr` (the modulus of the
 * derivative).  For the Mandelbulb in spherical-power form, dr updates
 * as `dr_new = r^(p−1) · p · dr + 1`.
 *
 * The smooth iter count comes from the renormalisation
 *   smooth = i + 1 − log₂(log(r)/log(BAILOUT)) / log₂(power)
 * which gives a continuous index across the surface — adjacent
 * pixels get adjacent smooth values, so colour gradients are smooth
 * (no integer-iter banding).
 *
 * Pass `smooth_out = NULL` if you don't need the smooth count
 * (sphere trace doesn't; only the FINAL hit point's smooth matters).
 */
static float mandelbulb_de(V3 p, int max_iter, float *smooth_out)
{
    V3    z   = p;
    float dr  = 1.0f;
    float r   = 0.0f;
    float log2_power = log2f(MANDELBULB_POWER);
    float log_bail   = logf(BAILOUT);

    int i;
    for (i = 0; i < max_iter; i++) {
        r = sqrtf(z.x*z.x + z.y*z.y + z.z*z.z);
        if (r > BAILOUT) break;

        /* spherical (theta = polar from +y, phi = azimuth in xz). */
        float theta = acosf(z.y / r);
        float phi   = atan2f(z.z, z.x);

        /* dr update — keeps track of derivative magnitude. */
        dr = powf(r, MANDELBULB_POWER - 1.0f) * MANDELBULB_POWER * dr + 1.0f;

        /* Apply the power. */
        float zr = powf(r, MANDELBULB_POWER);
        theta *= MANDELBULB_POWER;
        phi   *= MANDELBULB_POWER;

        /* Back to Cartesian + add original point (Mandelbulb formula). */
        float sin_th = sinf(theta);
        z.x = zr * sin_th * cosf(phi) + p.x;
        z.y = zr * cosf(theta)        + p.y;
        z.z = zr * sin_th * sinf(phi) + p.z;
    }

    if (smooth_out) {
        if (i >= max_iter) {
            *smooth_out = (float)max_iter;       /* didn't escape       */
        } else {
            float ln_r = logf(r);
            if (ln_r > 0.0f && log_bail > 0.0f)
                *smooth_out = (float)i + 1.0f
                            - log2f(ln_r / log_bail) / log2_power;
            else
                *smooth_out = (float)i;
        }
    }
    return 0.5f * logf(r) * r / dr;
}

/*
 * mandelbulb_normal — surface normal at p via central differences.
 *
 * Why central (not forward) differences: forward differences bias
 * the normal toward one octant and produce a visible asymmetry under
 * Phong shading.  Central costs 6 DE evaluations vs 3, but the
 * geometry is correct.
 */
static V3 mandelbulb_normal(V3 p, int max_iter)
{
    float e = NORMAL_EPS;
    float dx = mandelbulb_de(v3(p.x + e, p.y, p.z), max_iter, NULL)
             - mandelbulb_de(v3(p.x - e, p.y, p.z), max_iter, NULL);
    float dy = mandelbulb_de(v3(p.x, p.y + e, p.z), max_iter, NULL)
             - mandelbulb_de(v3(p.x, p.y - e, p.z), max_iter, NULL);
    float dz = mandelbulb_de(v3(p.x, p.y, p.z + e), max_iter, NULL)
             - mandelbulb_de(v3(p.x, p.y, p.z - e), max_iter, NULL);
    return v3norm(v3(dx, dy, dz));
}

/* ===================================================================== */
/* §6  march — sphere trace + soft shadow + AO + Phong                    */
/* ===================================================================== */

/*
 * sphere_trace — Hart 1996, with under-relaxation (α = STEP_RELAX) and
 * adaptive hit epsilon.
 *
 *   t = 0
 *   for step:
 *       d = DE(origin + t · dir)
 *       if d < HIT_EPS · (1 + t · ADAPTIVE_FACTOR): hit
 *       t += d · STEP_RELAX
 *
 * Returns t (negative on miss).  The `*out_steps` count is the
 * number of march steps required — passed back so the caller can
 * derive an AO factor (cells in concavities take many steps to
 * converge).
 */
static float sphere_trace(V3 origin, V3 dir, int max_iter, int *out_steps)
{
    float t = 0.0f;
    int   step;
    for (step = 0; step < MAX_STEPS; step++) {
        V3 p = v3add(origin, v3scale(t, dir));
        float d = mandelbulb_de(p, max_iter, NULL);
        float eps = HIT_EPS * (1.0f + t * ADAPTIVE_FACTOR);
        if (d < eps) {
            *out_steps = step;
            return t;
        }
        if (t > MAX_T) break;
        t += d * STEP_RELAX;
    }
    *out_steps = step;
    return -1.0f;     /* miss */
}

/*
 * soft_shadow — Christensen-style soft shadow ray.
 *
 *   Starts at p + ε·light_dir to avoid self-shadowing; marches
 *   toward the light, tracking min(K · d / t) over the path.  The
 *   running minimum converts a hard shadow ray into a soft penumbra.
 *   K controls hardness (higher = sharper shadow edge).
 */
static float soft_shadow(V3 origin, V3 light_dir, int max_iter)
{
    float result = 1.0f;
    float t = SHADOW_NEAR;
    for (int i = 0; i < SHADOW_STEPS; i++) {
        V3 p = v3add(origin, v3scale(t, light_dir));
        float d = mandelbulb_de(p, max_iter, NULL);
        if (d < HIT_EPS) return SHADOW_FLOOR;
        float k = SHADOW_K * d / t;
        if (k < result) result = k;
        t += d;
        if (t > SHADOW_FAR) break;
    }
    return (result < SHADOW_FLOOR) ? SHADOW_FLOOR
                                   : (result > 1.0f ? 1.0f : result);
}

/*
 * shade — combine Lambert + soft shadow + AO into final luminance.
 *
 *   light = AMBIENT + (1 − AMBIENT) · ndl · soft · ao
 *
 * AO is a cheap step-count proxy: cells inside concavities take
 * many DE evaluations to converge (the trace bumps along the wall
 * geometry), so step count correlates with concavity → naturally
 * darkens crevices.
 */
static float shade(V3 hit_p, V3 normal, V3 light_dir,
                   int max_iter, int march_steps)
{
    float ndl = v3dot(normal, light_dir);
    if (ndl < 0.0f) ndl = 0.0f;

    float soft = soft_shadow(hit_p, light_dir, max_iter);

    float ao = 1.0f - ((float)march_steps / (float)MAX_STEPS) * AO_STRENGTH;
    if (ao < AO_FLOOR) ao = AO_FLOOR;

    float L = AMBIENT + (1.0f - AMBIENT) * ndl * soft * ao;
    if (L < 0.0f) L = 0.0f;
    if (L > 1.0f) L = 1.0f;
    return L;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool   paused;
    int    current_theme;
    int    iters;
    int    cols, rows;

    float  cam_dist;
    float  orbit_yaw;            /* auto-advancing                    */
    float  orbit_pitch;          /* fixed default tilt                */
    float  user_yaw, user_pitch; /* manual offsets                    */
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->current_theme = 0;
    s->iters         = ITERS_DEFAULT;
    s->cols          = cols;
    s->rows          = rows;
    s->cam_dist      = CAM_DIST_DEFAULT;
    s->orbit_yaw     = 0.5f;
    s->orbit_pitch   = ORBIT_PITCH_DEF;
    s->user_yaw      = 0.0f;
    s->user_pitch    = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reset_cam(Scene *s)
{
    s->cam_dist     = CAM_DIST_DEFAULT;
    s->orbit_yaw    = 0.5f;
    s->orbit_pitch  = ORBIT_PITCH_DEF;
    s->user_yaw     = 0.0f;
    s->user_pitch   = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->orbit_yaw += ORBIT_YAW_RATE * dt;
    if (s->orbit_yaw >  (float)(2.0 * M_PI)) s->orbit_yaw -= (float)(2.0 * M_PI);
}

/*
 * scene_render — full-frame raymarch.
 *
 *   1. Camera basis from orbit + user offsets.
 *   2. For each terminal cell, sphere-trace; on hit shade + map to
 *      glyph + colour pair.
 *   3. Pre-fill white bg if NEGATIVE theme; skip stars (no stars).
 *   4. Batch attron/attroff: track last (pair, attr), only switch
 *      on change — cuts attribute thrash 3-5× on uniform regions.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    bool inverted = themes[s->current_theme].inverted;

    /* For inverted themes, paint white over the canvas region first. */
    if (inverted) {
        attron(COLOR_PAIR(PAIR_RAMP_BASE));
        for (int row = 0; row < rows_eff; row++)
            for (int col = 0; col < s->cols; col++)
                mvaddch(row, col, ' ');
        attroff(COLOR_PAIR(PAIR_RAMP_BASE));
    }

    /* Camera basis (orbit yaw + manual offset, fixed default pitch). */
    float yaw   = s->orbit_yaw   + s->user_yaw;
    float pitch = s->orbit_pitch + s->user_pitch;
    if (pitch >  MANUAL_PITCH_MAX) pitch =  MANUAL_PITCH_MAX;
    if (pitch < -MANUAL_PITCH_MAX) pitch = -MANUAL_PITCH_MAX;

    V3 cam_origin = v3(
        s->cam_dist * cosf(pitch) * cosf(yaw),
        s->cam_dist * sinf(pitch),
        s->cam_dist * cosf(pitch) * sinf(yaw)
    );
    V3 fwd   = v3norm(v3sub(v3(0,0,0), cam_origin));
    V3 wup   = v3(0, 1, 0);
    V3 right = v3norm(v3cross(fwd, wup));
    V3 up    = v3cross(right, fwd);

    /* Light direction (world-fixed, normalised). */
    V3 light = v3norm(v3(LIGHT_X, LIGHT_Y, LIGHT_Z));

    float fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    float phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;

    int     last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t  last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            float u =  ((float)col + 0.5f) / (float)s->cols * 2.0f - 1.0f;
            float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
            V3 dir = v3norm(v3add(fwd,
                v3add(v3scale(u * fov_t, right),
                      v3scale(v * fov_t * phys_aspect, up))));

            int   march_steps = 0;
            float t = sphere_trace(cam_origin, dir, s->iters, &march_steps);

            if (t < 0.0f) {
                /* Miss — leave default-bg (or white pre-fill). */
                if (!inverted && last_pair >= 0) {
                    attroff(COLOR_PAIR(last_pair) | last_attr);
                    last_pair = -1;
                }
                continue;
            }

            V3    hit_p   = v3add(cam_origin, v3scale(t, dir));
            V3    normal  = mandelbulb_normal(hit_p, s->iters);
            float lum     = shade(hit_p, normal, light, s->iters, march_steps);

            /* Re-evaluate DE to capture smooth iter at the hit point. */
            float smooth;
            (void)mandelbulb_de(hit_p, s->iters, &smooth);

            int slot_lum = (int)(lum * 7.999f);
            if (slot_lum < 0) slot_lum = 0;
            if (slot_lum > 7) slot_lum = 7;

            float t_color = smooth / (float)s->iters;
            if (t_color < 0.0f) t_color = 0.0f;
            if (t_color > 1.0f) t_color = 1.0f;
            int slot_clr = (int)(t_color * 7.999f);
            if (slot_clr < 0) slot_clr = 0;
            if (slot_clr > 7) slot_clr = 7;

            char glyph = LUMA_GLYPHS[slot_lum];
            int  pair  = PAIR_RAMP_BASE + slot_clr;

            attr_t attr;
            if (inverted) {
                attr = A_NORMAL;     /* see CONCEPTS for why            */
            } else {
                attr = (slot_lum >= 6) ? A_BOLD
                     : (slot_lum <= 1) ? A_DIM
                     :                   A_NORMAL;
            }

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
/* ── end §7 — to understand the ncurses wrapper, read §8 screen ── */

/* ===================================================================== */
/* §8  screen                                                             */
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

    char buf[200];
    snprintf(buf, sizeof buf,
             " MANDELBULB   %s   theme:%s   iters:%2d   dist:%4.2f   "
             "%5.1f fps  %3d Hz   "
             "t/T:theme  i/I:iters  arrows:orbit  z/Z:zoom  spc:pause  r:reset  q:quit ",
             s->paused ? "PAUSED " : "ORBITING",
             themes[s->current_theme].name,
             s->iters, (double)s->cam_dist,
             fps, sim_fps);

    int row = sc->rows - 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int x = 0; x < sc->cols; x++) mvaddch(row, x, ' ');
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
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
    case 'r': case 'R': scene_reset_cam(s); s->iters = ITERS_DEFAULT; break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'i':
        if (s->iters > ITERS_MIN) s->iters--;
        break;
    case 'I':
        if (s->iters < ITERS_MAX) s->iters++;
        break;

    case 'z':
        s->cam_dist -= CAM_DIST_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
        break;
    case 'Z':
        s->cam_dist += CAM_DIST_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        break;

    case KEY_LEFT:  s->user_yaw   -= MANUAL_YAW_STEP;   break;
    case KEY_RIGHT: s->user_yaw   += MANUAL_YAW_STEP;   break;
    case KEY_UP:    s->user_pitch += MANUAL_PITCH_STEP; break;
    case KEY_DOWN:  s->user_pitch -= MANUAL_PITCH_STEP; break;

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
