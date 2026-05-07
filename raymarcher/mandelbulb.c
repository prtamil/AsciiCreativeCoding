/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * mandelbulb.c — clean 3-D Mandelbulb explorer for the terminal
 *
 * DEMO: One iconic 3-D Mandelbulb (power = 8), sphere-traced with
 *       Phong shading + soft shadow + ambient occlusion, coloured by
 *       smooth iteration count.  A slow auto-orbit shows the fractal
 *       from every angle; arrow keys override the orbit for manual
 *       inspection.  No starfield, no animated palette, no power
 *       morph, no near-miss glow corona — every pixel of every cell
 *       is dedicated to making the FRACTAL silhouette read clearly.
 *
 *       Themes (cycle with t / T):
 *         CLASSIC   warm orange-red-yellow (Daniel White's iconic look)
 *         ICE       cool blues climbing into white
 *         PLASMA    high-sat magenta → cyan → yellow (fractal-art neon)
 *         MONO      grayscale, no hue distraction — best for shape study
 *         NEGATIVE  white bg + dark fg, photographic-negative inversion
 *
 * Study alongside:
 *   raymarcher/kifs_fractal.c       — angular cousin (folding fractal);
 *                                      same sphere-trace pipeline, very
 *                                      different visual character
 *                                      (sharp temple architecture vs.
 *                                      organic blobs).
 *   raster/mandelbulb_raster.c      — same fractal, polygon-rasterised
 *                                      pipeline.  Studying both side-
 *                                      by-side reveals the contrast
 *                                      between "evaluate per-pixel"
 *                                      (this file) and "tessellate
 *                                      once, render forever".
 *   raymarcher/donut.c              — simpler raymarcher friend; read
 *                                      donut.c first for the camera +
 *                                      ray-gen + z-buffer skeleton.
 *
 * Section map:
 *   §1 config    — themes, raymarch tunables, control rates, pair IDs
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — 8-pair theme apply (with inverted-bg support)
 *   §4 vec3      — value-type 3-D math
 *   §5 mandelbulb— iteration helpers (Spherical / dr / apply_power)
 *                  + DE (with smooth iter) + central-diff normal
 *   §6 march     — sphere trace + soft shadow + AO + Phong shade
 *   §7 scene     — camera basis, ray gen, decorate + emit, render
 *   §8 screen    — ncurses init / resize / HUD draw / present
 *   §9 app       — main loop, signals, key handling, cleanup
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

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sphere tracing of the Mandelbulb distance estimator
 *                  (Daniel White & Paul Nylander, 2009).
 *
 *                  ITERATION (per evaluation point p, with z₀ = p):
 *                     for i = 1..N:
 *                         (r, θ, φ)  = to_spherical(z)         // |z|, polar, azimuth
 *                         if r > BAILOUT: break
 *                         dr         = update_dr(dr, r)        // r^(p−1)·p·dr + 1
 *                         z          = apply_power_and_add(s, p)
 *
 *                  DISTANCE ESTIMATE (Hubbard-Douady form):
 *                     DE(p) = ½ · log(r) · r / dr
 *
 *                  SMOOTH ITER (continuous index for colouring):
 *                     smooth = i + 1 − log₂(log r / log BAILOUT) / log₂(power)
 *                  Adjacent pixels get continuous colour, no banding.
 *
 *                  SPHERE TRACE: t = 0; for step = 1..MAX_STEPS:
 *                     d = DE(origin + t · dir)
 *                     if d < HIT_EPS · (1 + t · ADAPTIVE_FACTOR): hit
 *                     t += d · STEP_RELAX                        // ≈ 0.85
 *
 *                  The 0.85 multiplier prevents overshoot at grazing
 *                  angles; without it the trace can step PAST a thin
 *                  surface lobe and miss the hit.
 *
 *                  SHADING:
 *                     N    = central-difference normal of DE(p)
 *                     ndl  = max(0, N · light_dir)
 *                     soft = soft-shadow ray toward light (16 steps,
 *                            tracking min(K · d / t) for soft penumbra)
 *                     ao   = 1 − (march_steps / MAX_STEPS) · AO_STRENGTH
 *                            (cells that took many march steps sit in
 *                             concavities — natural ambient occlusion)
 *                     L    = AMBIENT + (1 − AMBIENT) · ndl · soft · ao
 *
 *                  PIXEL ENCODING:
 *                     glyph slot = ⌊L · LUMA_SLOT_FLT⌋             (shape)
 *                     colour slot = ⌊smooth/iters · LUMA_SLOT_FLT⌋ (depth)
 *
 * Data-structure : Stateless math.  No tables, no LUTs.  Each pixel
 *                  re-evaluates the iteration ~70 times during sphere
 *                  trace + 6 times for normal estimation + ~16 times
 *                  for the soft-shadow ray.
 *
 * Rendering      : ASCII only.  Glyph from Lambertian luminance,
 *                  colour pair from smooth iteration count.  The two
 *                  are decoupled — glyph reads as SHAPE (Phong
 *                  lighting), colour reads as DEPTH (escape iter).
 *
 * Performance    : ~50 march × ~8 iters × ~10 ops ≈ 4 000 ops per
 *                  pixel for the trace; another ~1 500 for shading.
 *                  At 80×24 = 1 920 pixels: ~10 M ops/frame.  Holds
 *                  60 fps comfortably; for 200×60 lower iters with `i`.
 *
 * References     :
 *   • White, D. & Nylander, P. (2009) — original Mandelbulb formulation
 *     and triplex algebra.
 *     https://www.skytopia.com/project/fractal/mandelbulb.html
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for the
 *     Antialiased Ray Tracing of Implicit Surfaces," *The Visual
 *     Computer* 12(10):527–545.  The sphere-trace iteration we use.
 *   • Hubbard, J. H. & Douady, A. (1985) — "Étude dynamique des
 *     polynômes complexes," Pub. Math. Orsay.  Origin of the
 *     `½ · log(r) · r / dr` distance estimator.
 *   • Quílez, I. — "Mandelbulb"
 *     https://iquilezles.org/articles/mandelbulb/
 *     The clearest derivation of the smooth iteration count and
 *     central-difference normal estimator.
 *   • Christensen, M. H. — "Distance Estimated 3D Fractals — a
 *     Tutorial," parts I–V (2011), syntopia.github.io.  Soft shadow
 *     and AO recipes for distance-estimated fractals.
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
 * "interior"; points whose orbits diverge are "outside".  We render
 * by sphere-tracing rays through 3-D space; at each step the DISTANCE
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
 *   – Which way does the surface face?         (normal      → shape)
 *   – Is the spot in shadow from another lobe? (soft shadow → drama)
 *   – Is the spot inside a tight crevice?      (AO          → depth)
 * The answers combine into a brightness number that picks the glyph.
 * A second number — how MANY iterations the spot took to "decide" it
 * was inside the fractal — picks the colour.  Glyph carries shape;
 * colour carries depth.
 *
 *      ┌─────────────────────────────────────────────────────────────┐
 *      │                                                             │
 *      │   light                                                     │
 *      │      ☀  ↘                                                   │
 *      │            ↘                                                │
 *      │              ╭──────╮         camera                        │
 *      │             ╱        ╲      ←──────  👁                     │
 *      │            ╱  bulb    ╲      one ray per cell               │
 *      │           ╱            ╲                                    │
 *      │            ╲          ╱                                     │
 *      │             ╲________╱                                      │
 *      │                                                             │
 *      │   per ray:  trace → hit? → normal + smooth + shade → cell   │
 *      └─────────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  Per frame:
 *
 *    1. Build camera basis (origin, fwd, right, up) from orbit yaw,
 *       orbit pitch, manual yaw, manual pitch, cam_dist.
 *    2. Normalise the light direction.
 *
 *  Per pixel:
 *
 *    3. RAY DIRECTION — through (col, row), aspect-corrected.
 *
 *    4. SPHERE TRACE:
 *           t = 0
 *           for step = 1..MAX_STEPS:
 *               p   = origin + t · dir
 *               d   = DE(p)
 *               eps = HIT_EPS · (1 + t · ADAPTIVE_FACTOR)
 *               if d < eps      → hit
 *               if t > MAX_T    → miss
 *               t  += d · STEP_RELAX
 *
 *    5. ON HIT:
 *           N      = central-difference normal of DE
 *           smooth = continuous escape count (for colour)
 *           soft   = soft-shadow ray toward light
 *           ao     = 1 − (march_steps / MAX_STEPS) · AO_STRENGTH
 *           L      = AMBIENT + (1−AMBIENT) · max(0, N·L_dir) · soft · ao
 *
 *    6. DECORATE:
 *           glyph slot = ⌊L · LUMA_SLOT_FLT⌋
 *           colour slot = ⌊smooth/iters · LUMA_SLOT_FLT⌋
 *           Cell  = (LUMA_GLYPHS[lum_slot], PAIR_RAMP_BASE+clr_slot, attr)
 *
 *    7. EMIT — paint with attribute batching (only attron/off when
 *       the pair OR attr changes).
 *
 *  Per tick:
 *    8. ANIMATE — orbit_yaw advances; pause freezes it.
 *
 * KEY FORMULAS
 * ────────────
 *   Spherical coords of z:
 *     r     = |z|
 *     θ     = acos(z.y / r)         (polar from +y)
 *     φ     = atan2(z.z, z.x)       (azimuth in xz)
 *
 *   Mandelbulb iteration step:
 *     z' = r^p · (sin pθ · cos pφ, cos pθ, sin pθ · sin pφ) + p₀
 *     dr' = r^(p−1) · p · dr + 1
 *
 *   Distance estimate:
 *     DE(p) = ½ · log(r) · r / dr
 *
 *   Smooth iteration (continuous):
 *     smooth = i + 1 − log₂(log r / log BAILOUT) / log₂(power)
 *
 *   Sphere-trace step under-relaxation:
 *     t_{k+1} = t_k + α · DE(p_k),    α = STEP_RELAX ∈ (0.5, 1.0]
 *
 *   Adaptive hit epsilon (cone tracing):
 *     ε_hit(t) = HIT_EPS · (1 + t · ADAPTIVE_FACTOR)
 *
 *   Soft-shadow factor (Christensen):
 *     soft = clamp(min over march of K · d / t,  SHADOW_FLOOR,  1)
 *
 *   AO from march-step count:
 *     ao = clamp(1 − (steps/MAX_STEPS) · AO_STRENGTH,  AO_FLOOR,  1)
 *
 *   Aspect correction (cells 2× taller than wide):
 *     phys_aspect = (rows · 2) / cols
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • DE BREAKDOWN AT POWER ≈ 1.  At power < 2 the iteration is nearly
 *     linear and the DE returns nonsense.  We pin power = 8 (default
 *     Mandelbulb).  Power adjustments would need a different DE form.
 *
 *   • r = 0 IN ITERATION.  acos(z.y / r) divides by r.  The very
 *     first iteration with z = origin (r = 0) is fine because we test
 *     `r > BAILOUT` BEFORE the spherical conversion — but a recurrent
 *     return to origin would crash.  In practice z grows fast under
 *     iteration so this never happens for points outside a tiny
 *     pathological set.  `to_spherical` defends with a length check
 *     anyway.
 *
 *   • OVERSHOOT WITHOUT UNDER-RELAXATION.  The DE is a LOWER bound on
 *     true distance, but optimistic near singularities.  STEP_RELAX
 *     = 0.85 prevents the trace from punching through thin lobes.
 *
 *   • ADAPTIVE EPSILON.  Far cells have lower precision needs; their
 *     hit ε scales up with t.  Without this the trace at large t
 *     never converges because the DE bottoms out at a noisy floor.
 *
 *   • ITERATION COST.  Each iteration involves 1 acos, 1 atan2, 4
 *     sin/cos, 2 pow.  Doubling N_ITER doubles per-pixel cost.
 *     Default 8 is the sweet spot for terminal resolution; lower
 *     with `i` on large terminals.
 *
 *   • NORMAL EPSILON.  Too small → noisy normals from DE
 *     quantisation; too large → smoothed normals lose surface
 *     detail.  4 · HIT_EPS is the empirical compromise.
 *
 *   • CAMERA INSIDE THE BULB.  Zooming past the hull (cam_dist <
 *     ~1.2) places the camera inside; the trace then walks outward
 *     and the fractal renders inverted.  We clamp cam_dist ≥
 *     CAM_DIST_MIN to keep the camera outside.
 *
 *   • PAUSE freezes orbit, NOT input — you can still manually orbit
 *     and zoom while paused.
 *
 *   • INVERTED THEME (NEGATIVE) needs special handling: the empty
 *     background must be pre-filled with the white bg pair, and the
 *     A_BOLD/A_DIM modulation is disabled (it would invert the
 *     brightness intent on light fg over white bg).
 *
 *   • HUD VS THEME.  HUD pairs are theme-independent (yellow + cyan
 *     on default bg).  In NEGATIVE theme this leaves the HUD strip
 *     on the terminal default background while the canvas is white;
 *     the visual band is acceptable for a niche theme.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press space.  Auto-orbit freezes.  The fractal holds at its
 *     current angle.  Resume — orbit picks up smoothly.
 *
 *   • Press `r`.  Camera resets to default (orbit yaw = 0, manual
 *     offsets cleared, distance default, iters default).
 *
 *   • Press `t`.  Theme cycles.  Geometry identical, only colours
 *     change.  CLASSIC → ICE is the most striking flip.
 *
 *   • Press `i` to lower iterations.  At iters = 4 the fractal
 *     becomes a smooth blob; at iters = 8 (default) it has well-
 *     defined lobes and crevices; at iters = 12 the finest surface
 *     detail is visible (also slowest).
 *
 *   • Press `→` repeatedly.  Manual yaw advances; combined with
 *     auto orbit you get faster rotation.  Hold `←` to slow / reverse.
 *
 *   • Press `↑` / `↓` to look from above / below.  At extreme
 *     pitches you see the bulb's polar lobes — the iconic "head"
 *     and "feet".
 *
 *   • Press `z`.  Zoom in until surface detail fills the screen;
 *     the raymarcher's adaptive epsilon should keep edges crisp.
 *     `Z` to back out.
 *
 *   • At 60 × 20, the fractal silhouette should still read as 3-D.
 *     If the bulb looks flat, AO or soft shadow has been disabled —
 *     both are critical for depth perception at low res.
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

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
    SIM_FPS_MIN      =  10,
    SIM_FPS_DEFAULT  =  60,
    SIM_FPS_MAX      = 120,
    SIM_FPS_STEP     =  10,
    FPS_UPDATE_MS    = 500,
    HUD_ROWS         =   2,    /* row 0 status + last row hint */
    ITERS_MIN        =   3,
    ITERS_MAX        =  14,
    ITERS_DEFAULT    =   8,    /* the canonical Mandelbulb at p = 8 */
};

/* §1.2 colour-pair IDs. */
enum {
    PAIR_HUD         =  1,     /* yellow + bold — top status row     */
    PAIR_HINT        =  2,     /* cyan   + bold — last row key hint  */
    PAIR_RAMP_BASE   =  3,     /* +0..+7 — depth ramp                */
};

/* §1.3 time helpers. */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

#define CELL_ASPECT      2.0f      /* terminal cell h / w */

/* §1.4 Mandelbulb iteration. */
#define MANDELBULB_POWER 8.0f
#define BAILOUT          4.0f      /* |z| > this → escaped */

/* §1.5 sphere trace.
 *
 *   MAX_STEPS         hard cap on march steps per ray.
 *   HIT_EPS           "we're touching the surface" threshold at t=0.
 *   ADAPTIVE_FACTOR   ε grows with t to avoid wasting steps on sub-
 *                     cell precision at large distances.
 *   MAX_T             ray-length budget; past this we declare miss.
 *   STEP_RELAX        under-relaxation factor on each step (DE is a
 *                     LOWER bound; α < 1 prevents grazing-angle
 *                     overshoot).
 */
#define MAX_STEPS        90
#define HIT_EPS          1.0e-3f
#define ADAPTIVE_FACTOR  0.012f
#define MAX_T            6.0f
#define STEP_RELAX       0.85f

/* §1.6 normal estimator. */
#define NORMAL_EPS       3.5e-3f

/* §1.7 soft shadow.
 *
 *   SHADOW_K       hardness — larger = sharper edge.
 *   SHADOW_FLOOR   minimum factor (cells in shadow stay this lit).
 */
#define SHADOW_STEPS     16
#define SHADOW_NEAR      0.012f
#define SHADOW_FAR       2.5f
#define SHADOW_K         32.0f
#define SHADOW_FLOOR     0.30f

/* §1.8 lighting + AO + ambient. */
#define AMBIENT          0.18f
#define AO_FLOOR         0.35f
#define AO_STRENGTH      0.70f

/* Light direction (normalised at use). */
#define LIGHT_X          0.55f
#define LIGHT_Y          0.75f
#define LIGHT_Z         -0.25f

/* §1.9 camera. */
#define CAM_DIST_DEFAULT  3.2f
#define CAM_DIST_MIN      1.5f      /* outside the bulb hull */
#define CAM_DIST_MAX      8.0f
#define CAM_DIST_STEP     0.20f
#define FOV_DEG          45.0f
#define ORBIT_YAW_RATE    0.30f     /* rad / sec auto-orbit              */
#define ORBIT_PITCH_DEF   0.25f     /* default static tilt above equator */
#define MANUAL_YAW_STEP   0.12f     /* per arrow keypress                */
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX  1.30f     /* clamp short of poles              */

/* §1.10 quantisation — number of glyph / colour slots. */
#define LUMA_SLOTS       8
#define LUMA_SLOT_FLT    7.999f     /* (LUMA_SLOTS - 0.001) */

/* §1.11 themes — each is an 8-tier depth ramp.  Slot 0 = outermost
 * shell (escape early, low smooth iter); slot 7 = innermost.  All
 * entries sit in the bright half of the 256-cube per the CLAUDE.md
 * theme rule, EXCEPT the NEGATIVE theme which uses dark fg over a
 * white bg (handled by the `inverted` flag).
 */
typedef struct {
    const char *name;
    short       ramp[LUMA_SLOTS];
    bool        inverted;       /* white bg, dark fg, A_BOLD/A_DIM off */
} Theme;

#define N_THEMES 5

static const Theme THEMES[N_THEMES] = {
    /* CLASSIC: warm crimson → red → orange → amber → yellow → bone.
     * Daniel White's original Mandelbulb images had this palette;
     * still the iconic "alien fruit lit by sunset" look.            */
    { "CLASSIC ",
      { 124, 160, 196, 202, 208, 214, 220, 229 }, false },

    /* ICE: deep teal → bright cyan → ice blue → near-white. */
    { "ICE     ",
      {  30,  37,  44,  51,  87, 123, 159, 195 }, false },

    /* PLASMA: high-saturation neon arc — magenta → cyan → yellow. */
    { "PLASMA  ",
      { 125, 165, 207, 213,  87, 123, 220, 229 }, false },

    /* MONO: clean grayscale.  Best for studying fractal shape with
     * zero hue distraction.  No black at slot 0 — visible grey.   */
    { "MONO    ",
      { 240, 244, 247, 250, 252, 253, 254, 231 }, false },

    /* NEGATIVE: photographic-negative inversion.  White bg, dark fg.
     * Cool/outer fractal regions blend into white; deep interior
     * stamps as solid black.  A_BOLD/A_DIM disabled (see CONCEPTS). */
    { "NEGATIVE",
      { 253, 250, 245, 240, 237, 234, 232,  16 }, true  },
};

/* §1.12 luminance ramp — slot 0 = `.` (faint shape) → slot 7 = `@`
 * (full opacity).  Slot 0 is `.` not space, so even the dimmest hit
 * pixel paints something visible against the background. */
static const char LUMA_GLYPHS[LUMA_SLOTS] = {
    '.', ',', ':', ';', '+', '*', '#', '@'
};

/* ── §2 clock — monotonic timer + sleep ──────────────────────────────── */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = { .tv_sec  = (time_t)(ns / NS_PER_SEC),
                            .tv_nsec = (long)  (ns % NS_PER_SEC) };
    nanosleep(&req, NULL);
}

/* ── §3 color — depth ramp + HUD/hint pairs ──────────────────────────── */

/*
 * theme_apply — re-init the eight depth-ramp pairs for the chosen
 * theme.  Called at startup and on every t / T keypress.  Geometry
 * is unaffected; only the smooth-iter → colour table changes.
 *
 * For inverted themes the bg is white (256-colour 231) so that
 * subsequent `mvaddch(' ')` calls in scene_render's pre-fill paint
 * the whole canvas white before the fractal is drawn over it.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    const Theme *t = &THEMES[idx];
    short bg256 = t->inverted ? 231        : -1;
    short bg8   = t->inverted ? COLOR_WHITE : -1;

    if (COLORS >= 256) {
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_RAMP_BASE + i), t->ramp[i], bg256);
    } else {
        static const short FB[LUMA_SLOTS] = {
            COLOR_BLUE, COLOR_BLUE, COLOR_MAGENTA, COLOR_MAGENTA,
            COLOR_RED,  COLOR_RED,  COLOR_YELLOW,  COLOR_WHITE,
        };
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_RAMP_BASE + i),
                      t->inverted ? COLOR_BLACK : FB[i], bg8);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3   (float x, float y, float z) { return (V3){x, y, z}; }
static inline V3    v3add(V3 a, V3 b)                { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub(V3 a, V3 b)                { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3scale(float s, V3 a)           { return v3(s*a.x, s*a.y, s*a.z); }
static inline float v3dot(V3 a, V3 b)                { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline V3    v3cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y,
              a.z*b.x - a.x*b.z,
              a.x*b.y - a.y*b.x);
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3    v3norm(V3 a)
{
    float L = v3len(a);
    return (L > 1e-12f) ? v3scale(1.0f / L, a) : v3(0, 1, 0);
}

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ── §5 mandelbulb — iteration + DE + smooth iter + normal ───────────── *
 *
 * The iteration body, expanded line-by-line with one helper per math
 * step, mirrors the pseudocode in the MENTAL MODEL:
 *
 *     for i = 1..N:
 *         (r, θ, φ)  = to_spherical(z)         |z|, polar, azimuth
 *         if r > BAILOUT: break
 *         dr         = update_dr(dr, r)        r^(p−1)·p·dr + 1
 *         z          = apply_power_and_add(s, p)
 *
 * Each helper is `static inline` — at -O2 the compiler inlines them
 * back into one tight loop, so no performance is given up for the
 * extra readability.
 */

/*
 * Spherical — z in spherical coordinates relative to +y as the polar
 * axis.  Computed once per iteration step and consumed by both
 * update_dr and apply_power_and_add.
 *
 *   r     = |z|
 *   theta = acos(z.y / r)        polar angle from +y
 *   phi   = atan2(z.z, z.x)      azimuth in the xz plane
 */
typedef struct { float r, theta, phi; } Spherical;

static inline Spherical to_spherical(V3 z)
{
    float r = sqrtf(z.x*z.x + z.y*z.y + z.z*z.z);
    Spherical s = { r, 0.0f, 0.0f };
    if (r > 1e-20f) {
        s.theta = acosf(z.y / r);
        s.phi   = atan2f(z.z, z.x);
    }
    return s;
}

/*
 * update_dr — one step of the running derivative magnitude.
 *
 *     dr_new = r^(p − 1) · p · dr + 1
 *
 * Tracking dr lets us compute the Hubbard-Douady DE
 * `½ · log(r) · r / dr` at the end of the iteration.
 */
static inline float update_dr(float dr, float r)
{
    return powf(r, MANDELBULB_POWER - 1.0f) * MANDELBULB_POWER * dr + 1.0f;
}

/*
 * apply_power_and_add — the Mandelbulb's signature step.  Take the
 * spherical-coordinate triplet, raise the radius to the 8th power,
 * multiply both angles by 8, convert back to Cartesian, then add the
 * original point c (the parameter the iteration is parameterised by).
 *
 *     z' = r^p · (sin pθ · cos pφ, cos pθ, sin pθ · sin pφ) + c
 *
 * This is the spherical analogue of the 2-D Mandelbrot's z² + c.
 */
static inline V3 apply_power_and_add(Spherical s, V3 c)
{
    float zr     = powf(s.r, MANDELBULB_POWER);
    float p_th   = MANDELBULB_POWER * s.theta;
    float p_ph   = MANDELBULB_POWER * s.phi;
    float sin_th = sinf(p_th);
    return v3(zr * sin_th * cosf(p_ph) + c.x,
              zr * cosf(p_th)          + c.y,
              zr * sin_th * sinf(p_ph) + c.z);
}

/*
 * mandelbulb_de — distance estimator with optional smooth iter output.
 *
 * Pseudocode (see KEY FORMULAS in MENTAL MODEL):
 *
 *     z = p_orig
 *     dr = 1
 *     for i in 0..max_iter:
 *         s = to_spherical(z)
 *         if s.r > BAILOUT: break
 *         dr = update_dr(dr, s.r)
 *         z  = apply_power_and_add(s, p_orig)
 *     return ½ · log(s.r) · s.r / dr
 *
 * If `smooth_out` is non-NULL, also writes the continuous escape
 * count for colouring (smooth across the surface — no integer
 * banding).  Pass NULL when computing normals (where the smooth
 * count isn't needed).
 */
static float mandelbulb_de(V3 p, int max_iter, float *smooth_out)
{
    V3        z         = p;
    float     dr        = 1.0f;
    Spherical s         = { 0.0f, 0.0f, 0.0f };
    int       i;

    const float log2_power = log2f(MANDELBULB_POWER);
    const float log_bail   = logf(BAILOUT);

    for (i = 0; i < max_iter; i++) {
        s  = to_spherical(z);
        if (s.r > BAILOUT) break;
        dr = update_dr(dr, s.r);
        z  = apply_power_and_add(s, p);
    }

    if (smooth_out) {
        if (i >= max_iter) {
            *smooth_out = (float)max_iter;       /* didn't escape */
        } else {
            float ln_r = logf(s.r);
            if (ln_r > 0.0f && log_bail > 0.0f)
                *smooth_out = (float)i + 1.0f
                            - log2f(ln_r / log_bail) / log2_power;
            else
                *smooth_out = (float)i;
        }
    }
    return 0.5f * logf(s.r) * s.r / dr;
}

/*
 * mandelbulb_normal — surface normal at p via central differences.
 *
 *     N_x = de(p + ε x̂) − de(p − ε x̂)
 *     N_y = de(p + ε ŷ) − de(p − ε ŷ)
 *     N_z = de(p + ε ẑ) − de(p − ε ẑ)
 *     N   = normalise(N_x, N_y, N_z)
 *
 * Central (not forward) differences cost twice as many DE evaluations
 * (6 vs 3) but the geometry comes out symmetric — forward differences
 * bias the normal toward one octant and produce visibly skewed Phong
 * shading.  6 DE evals per hit pixel is by far the dominant frame
 * cost.  Normal-only DE skips the smooth_iter pass for speed.
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

/* ── §6 march — sphere trace + soft shadow + AO + Phong shade ────────── */

/*
 * TraceResult — what the sphere trace returns.  `march_steps` is the
 * number of steps taken; the shader uses it as a cheap AO signal.
 */
typedef struct {
    bool  hit;
    V3    p;
    int   march_steps;
} TraceResult;

/*
 * sphere_trace — Hart 1996 with under-relaxation (α = STEP_RELAX) and
 * adaptive hit epsilon.  Pseudocode:
 *
 *     t = 0
 *     for step in 0..MAX_STEPS:
 *         p   = origin + t · dir
 *         d   = DE(p)
 *         eps = HIT_EPS · (1 + t · ADAPTIVE_FACTOR)
 *         if d < eps: return hit at p, with march_steps
 *         if t > MAX_T: return miss
 *         t += d · STEP_RELAX
 */
static TraceResult sphere_trace(V3 origin, V3 dir, int max_iter)
{
    TraceResult tr = { false, {0, 0, 0}, 0 };
    float t = 0.0f;
    int   step;

    for (step = 0; step < MAX_STEPS; step++) {
        V3    p   = v3add(origin, v3scale(t, dir));
        float d   = mandelbulb_de(p, max_iter, NULL);
        float eps = HIT_EPS * (1.0f + t * ADAPTIVE_FACTOR);

        if (d < eps) {
            tr.hit         = true;
            tr.p           = p;
            tr.march_steps = step;
            return tr;
        }
        if (t > MAX_T) break;
        t += d * STEP_RELAX;
    }
    tr.march_steps = step;
    return tr;
}

/*
 * soft_shadow — Christensen-style soft shadow ray.
 *
 *     start at origin + SHADOW_NEAR · light_dir   (avoid self-shadow)
 *     march toward the light, tracking min(K · d / t)
 *     return that minimum, clamped to [SHADOW_FLOOR, 1]
 *
 * The running minimum converts a hard shadow ray into a soft penumbra:
 * if any sample came CLOSE to a surface (small d at a given t), the
 * ratio K·d/t drops, dimming the light contribution proportionally.
 * SHADOW_K controls hardness — bigger K = sharper shadow edge.
 */
static float soft_shadow(V3 origin, V3 light_dir, int max_iter)
{
    float result = 1.0f;
    float t      = SHADOW_NEAR;

    for (int i = 0; i < SHADOW_STEPS; i++) {
        V3    p = v3add(origin, v3scale(t, light_dir));
        float d = mandelbulb_de(p, max_iter, NULL);

        if (d < HIT_EPS) return SHADOW_FLOOR;     /* fully blocked */

        float k = SHADOW_K * d / t;
        if (k < result) result = k;

        t += d;
        if (t > SHADOW_FAR) break;
    }
    return clampf(result, SHADOW_FLOOR, 1.0f);
}

/*
 * shade — combine Lambert + soft shadow + AO into the final luminance.
 *
 *     L = AMBIENT + (1 − AMBIENT) · max(0, N·L_dir) · soft · ao
 *
 * AO is a cheap step-count proxy: cells inside concavities take many
 * march steps to converge (the trace bumps along the wall geometry),
 * so step count correlates with concavity → naturally darkens
 * crevices.  Cleanest free AO you can ask for in a sphere tracer.
 */
static float shade(V3 hit_p, V3 normal, V3 light_dir,
                   int max_iter, int march_steps)
{
    float ndl = v3dot(normal, light_dir);
    if (ndl < 0.0f) ndl = 0.0f;

    float soft = soft_shadow(hit_p, light_dir, max_iter);

    float ao = 1.0f - ((float)march_steps / (float)MAX_STEPS) * AO_STRENGTH;
    if (ao < AO_FLOOR) ao = AO_FLOOR;

    return clampf(AMBIENT + (1.0f - AMBIENT) * ndl * soft * ao, 0.0f, 1.0f);
}

/* ── §7 scene — camera basis, ray gen, decorate + emit, render ───────── */

typedef struct {
    bool   paused;
    int    current_theme;
    int    iters;
    int    cols, rows;

    /* Camera. */
    float  cam_dist;
    float  orbit_yaw;            /* auto-advancing                    */
    float  orbit_pitch;          /* fixed default tilt                */
    float  user_yaw, user_pitch; /* manual offsets via arrow keys     */
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

/* §7.1 camera basis + per-pixel ray generation. */

typedef struct {
    V3    origin;
    V3    fwd, right, up;
    float fov_t;
    float phys_aspect;
} Camera;

/*
 * camera_basis — orthonormal (fwd, right, up) at the orbiting camera
 * position, plus the FOV tangent and aspect correction.
 *
 *   yaw   = orbit_yaw + user_yaw
 *   pitch = clamp(orbit_pitch + user_pitch, ±MAX_PITCH)
 *   eye   = cam_dist · (cos pitch · cos yaw, sin pitch, cos pitch · sin yaw)
 *   fwd   = normalise(0 − eye) = normalise(−eye)
 *   right = normalise(fwd × world_up)
 *   up    = right × fwd
 */
static Camera camera_basis(const Scene *s, int rows_eff)
{
    float yaw   = s->orbit_yaw   + s->user_yaw;
    float pitch = clampf(s->orbit_pitch + s->user_pitch,
                         -MANUAL_PITCH_MAX, MANUAL_PITCH_MAX);

    Camera c;
    c.origin = v3(s->cam_dist * cosf(pitch) * cosf(yaw),
                  s->cam_dist * sinf(pitch),
                  s->cam_dist * cosf(pitch) * sinf(yaw));
    c.fwd         = v3norm(v3sub(v3(0, 0, 0), c.origin));
    V3 wup        = v3(0, 1, 0);
    c.right       = v3norm(v3cross(c.fwd, wup));
    c.up          = v3cross(c.right, c.fwd);
    c.fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    c.phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;
    return c;
}

/*
 * pixel_ray — direction from camera through pixel (col, row).
 *
 *   u   ∈ [−1, +1]  along screen x
 *   v   ∈ [−1, +1]  along screen y (flipped: top of screen → +v)
 *   dir = forward + u·tan(FOV/2)·right + v·tan(FOV/2)·aspect·up
 */
static V3 pixel_ray(int col, int row, int cols, int rows_eff, const Camera *c)
{
    float u =  ((float)col + 0.5f) / (float)cols     * 2.0f - 1.0f;
    float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
    V3 sx = v3scale(u * c->fov_t,                  c->right);
    V3 sy = v3scale(v * c->fov_t * c->phys_aspect, c->up);
    return v3norm(v3add(c->fwd, v3add(sx, sy)));
}

/* §7.2 hit assembly + cell decoration. */

/*
 * Hit — the complete per-pixel result of "did the ray hit, and if so,
 * everything the shader needs about that hit".
 */
typedef struct {
    bool  hit;
    V3    p;
    V3    normal;
    float smooth;
    float luminance;
    int   march_steps;
} Hit;

/*
 * assemble_hit — given a TraceResult, fill in normal + smooth + final
 * luminance.  Pseudocode:
 *
 *     if !tr.hit: return Hit{ hit = false }
 *     normal     = mandelbulb_normal(tr.p)
 *     smooth     = mandelbulb_de(tr.p, &out)            (smooth-only)
 *     luminance  = shade(tr.p, normal, light, march_steps)
 *
 * One-stop: every per-hit DE evaluation lives in one place so the
 * cost is auditable and the orchestrator stays small.
 */
static Hit assemble_hit(TraceResult tr, int max_iter, V3 light)
{
    Hit h = { tr.hit, tr.p, v3(0, 1, 0), 0.0f, 0.0f, tr.march_steps };
    if (!tr.hit) return h;

    h.normal = mandelbulb_normal(tr.p, max_iter);
    (void)mandelbulb_de(tr.p, max_iter, &h.smooth);
    h.luminance = shade(tr.p, h.normal, light, max_iter, tr.march_steps);
    return h;
}

/*
 * Cell — a (glyph, colour pair, attribute) decoration of one terminal
 * cell.  shade_hit returns this; emit_cell paints it.  pair < 0 is the
 * miss sentinel: don't paint anything (the canvas pre-fill or the
 * default-bg already shows through).
 */
typedef struct { char glyph; int pair; attr_t attr; } Cell;

/*
 * to_slot — quantise a [0, 1] value to an integer slot 0..LUMA_SLOTS−1.
 * Clamps gracefully on out-of-range inputs.
 */
static int to_slot(float x_01)
{
    int s = (int)(x_01 * LUMA_SLOT_FLT);
    if (s < 0)             s = 0;
    if (s >= LUMA_SLOTS)   s = LUMA_SLOTS - 1;
    return s;
}

/*
 * luma_attr — A_BOLD for the brightest slots, A_DIM for the darkest,
 * A_NORMAL otherwise.  Disabled (returns A_NORMAL) for inverted
 * themes: dark-fg-on-white-bg flips the brightness intent of A_BOLD
 * (it makes the foreground LIGHTER on most terminals, which would
 * REDUCE contrast against the white bg).
 */
static attr_t luma_attr(int slot, bool inverted)
{
    if (inverted)        return A_NORMAL;
    if (slot >= 6)       return A_BOLD;
    if (slot <= 1)       return A_DIM;
    return                      A_NORMAL;
}

/*
 * shade_hit — pixel decoration.  Hit → Cell.
 *
 *     if !h->hit:                    miss sentinel (pair = −1)
 *     glyph slot   = to_slot(luminance)
 *     colour slot  = to_slot(smooth / max_iter)
 *     glyph        = LUMA_GLYPHS[lum]
 *     pair         = PAIR_RAMP_BASE + clr
 *     attr         = luma_attr(lum, inverted)
 */
static Cell shade_hit(const Hit *h, int max_iter, bool inverted)
{
    if (!h->hit) {
        return (Cell){ ' ', -1, 0 };       /* miss → don't paint */
    }
    int s_lum = to_slot(h->luminance);
    int s_clr = to_slot(h->smooth / (float)max_iter);
    return (Cell){
        .glyph = LUMA_GLYPHS[s_lum],
        .pair  = PAIR_RAMP_BASE + s_clr,
        .attr  = luma_attr(s_lum, inverted),
    };
}

/*
 * emit_cell — paint one cell, batching attron/attroff so we only call
 * them when (pair, attr) actually changes.  Halves attribute thrash
 * on uniform regions.  Skips silently when the cell is a miss
 * sentinel (pair < 0).
 */
static void emit_cell(int row, int col, Cell c,
                      int *last_pair, attr_t *last_attr)
{
    if (c.pair < 0) return;       /* miss — leave the cell as-is */

    if (c.pair != *last_pair || c.attr != *last_attr) {
        if (*last_pair >= 0) attroff(COLOR_PAIR(*last_pair) | *last_attr);
        attron(COLOR_PAIR(c.pair) | c.attr);
        *last_pair = c.pair;
        *last_attr = c.attr;
    }
    mvaddch(row, col, (chtype)(unsigned char)c.glyph);
}

/*
 * prefill_canvas — for inverted themes, paint the fractal canvas
 * region white before any hits draw over it.  Misses then naturally
 * show through as white.
 */
static void prefill_canvas(int y0, int rows_eff, int cols, bool inverted)
{
    if (!inverted) return;
    attron(COLOR_PAIR(PAIR_RAMP_BASE));
    for (int row = 0; row < rows_eff; row++)
        for (int col = 0; col < cols; col++)
            mvaddch(y0 + row, col, ' ');
    attroff(COLOR_PAIR(PAIR_RAMP_BASE));
}

/* §7.3 the orchestrator — one tiny double loop. */

/*
 * scene_render — full-frame raymarch.  The body reads as the four-
 * line algorithm: trace → assemble → decorate → emit.
 *
 * One ray per terminal cell.  No virtual canvas, no upscale — the
 * cells ARE the pixels.  The fractal is rendered into rows
 * [y0, y0 + rows_eff − 1]; row 0 (status) and rows−1 (hint) are
 * reserved for the HUD and painted by hud_draw().
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - HUD_ROWS;
    if (rows_eff < 1) return;

    bool inverted = THEMES[s->current_theme].inverted;
    int  y0       = 1;            /* shift down 1 for the top HUD row */

    prefill_canvas(y0, rows_eff, s->cols, inverted);

    Camera cam   = camera_basis(s, rows_eff);
    V3     light = v3norm(v3(LIGHT_X, LIGHT_Y, LIGHT_Z));

    int    last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            V3          ray = pixel_ray(col, row, s->cols, rows_eff, &cam);
            TraceResult tr  = sphere_trace(cam.origin, ray, s->iters);
            Hit         h   = assemble_hit(tr, s->iters, light);
            Cell        c   = shade_hit(&h, s->iters, inverted);
            emit_cell(y0 + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ── §8 screen — ncurses init / resize / HUD / present ───────────────── */

typedef struct { int cols, rows; } Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free        (Screen *sc) { (void)sc; endwin(); }

static void screen_resize_curses(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
}

/*
 * hud_draw — CLAUDE.md HUD spec:
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 *
 * Both rows always use A_BOLD so the HUD stays legible against any
 * fractal colour underneath (including inverted-theme white).
 */
static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    char status[140];
    snprintf(status, sizeof status,
             " %5.1f fps  %3d Hz  theme:%s  iters:%2d  dist:%4.2f  %s ",
             fps, sim_fps,
             THEMES[s->current_theme].name,
             s->iters, (double)s->cam_dist,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > sc->cols) slen = sc->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - slen, "%s", status);
    mvprintw(0, 0, " MANDELBULB ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  t/T:theme  i/I:iters  "
             "z/Z:zoom  arrows:orbit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps)
{
    erase();
    scene_render(s);
    hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ── §9 app — main loop, signals, key handling, cleanup ──────────────── */

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
static void cleanup         (void)    { endwin(); }

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

    case 'i': if (s->iters > ITERS_MIN) s->iters--; break;
    case 'I': if (s->iters < ITERS_MAX) s->iters++; break;

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
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    scene_init (&app->scene, app->screen.cols, app->screen.rows);

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

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
