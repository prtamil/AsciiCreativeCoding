/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kifs_fractal.c — Kaleidoscopic IFS (Knighty's folding fractal).
 *
 * DEMO: Sphere-traces a 3-D fractal whose distance estimator is built
 *       by repeatedly FOLDING space and SCALING toward a fixed point.
 *       Each fold is a simple reflection; chaining N of them produces
 *       sharp, brutalist, temple-like architecture — alien angular
 *       cathedrals that look nothing like Mandelbulb's organic blobs.
 *       Colour comes from an "orbit trap" (closest approach to the
 *       origin during the fold sequence) so the surface is naturally
 *       chromatic — different folds visit different trap radii. A
 *       slow camera orbit + slowly animated fold rotation keeps the
 *       fractal walking on its own.
 *
 *       Three presets (cycle with n / N):
 *         TETRA     Sierpinski tetrahedron — three plane folds along
 *                   (x+y), (x+z), (y+z); scale 2 about the (1,1,1)
 *                   corner.  The canonical KIFS — 4-fold symmetric.
 *         MENGER    Menger-sponge variant — abs fold + descending
 *                   sort + scale 3 about (1,1,1) + a final z-fold.
 *                   Boxy, 8-fold symmetric, classic recursion grid.
 *         KIFS_ROT  Modified KIFS with a per-iteration y-rotation +
 *                   abs fold + sort + scale.  The rotation animates
 *                   over time so the fractal slowly reshapes.
 *
 *       Six themes (cycle with t / T) recolour the orbit-trap palette:
 *         GOLD     warm temple gold → bone white
 *         ICE      pale cyan → ice white
 *         COBALT   deep blue → electric cyan
 *         COPPER   bronze → orange highlights
 *         ALIEN    violet → magenta → cyan
 *         MONO     pure greyscale (high-contrast etching)
 *
 * Study alongside:
 *   raymarcher/mandelbulb_explorer.c  — same sphere-trace skeleton, but
 *           Mandelbulb's z = z^p + c iteration produces ORGANIC blobs.
 *           KIFS produces ANGULAR architecture — read both to feel the
 *           difference between "spherical iteration" and "folding".
 *   raymarcher/sdf_gallery.c          — SDF composition primer.
 *
 * Section map:
 *   §1 config   — presets, themes, raymarch tunables
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — 8-pair orbit-trap ramp + bg pair
 *   §4 vec3     — value-type 3-D math
 *   §5 kifs     — fold loop (de + de_with_trap + normal estimator)
 *   §6 march    — sphere-trace + lighting
 *   §7 scene    — camera orbit, animation, full-frame render
 *   §8 screen   — ncurses init / draw / resize
 *   §9 app      — signals, fixed-step main loop
 *
 * Keys:
 *   q / ESC      quit
 *   space        pause / resume animation (camera orbit + fold rot)
 *   r            reset camera
 *   n / N        next / previous preset    (TETRA / MENGER / KIFS_ROT)
 *   t / T        next / previous theme
 *   ← / →        manual orbit (yaw)
 *   ↑ / ↓        manual orbit (pitch)
 *   z / Z        zoom out / in
 *   i / I        iterations −1 / +1   (depth of fractal recursion)
 *   ] / [        sim Hz up / down
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/kifs_fractal.c \
 *       -o kifs_fractal -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sphere tracing of a Kaleidoscopic Iterated Function
 *                  System (KIFS) distance estimator.
 *
 *                  The DE is built by REPEATEDLY FOLDING and SCALING
 *                  the input point, then evaluating the distance to a
 *                  simple primitive (sphere or box) in the folded
 *                  space.  Each fold is a piecewise affine map (a
 *                  reflection across a plane); chaining N of them
 *                  produces a self-similar fractal because the inverse
 *                  of a fold replicates the primitive across the fold
 *                  axis.  Critically, fold + scale together compose to
 *                  a *contractive* map, so every point in space orbits
 *                  toward the fractal attractor.
 *
 *                  One sphere-trace step:
 *                     d = kifs_de(p)        // returns distance to fractal
 *                     p += d · ray_dir      // safe step, never overshoot
 *                  Repeat until d < ε (hit) or t > t_max (miss).
 *
 *                  TETRA preset:
 *                     for i = 1..N:
 *                         if (x+y < 0) (x,y) = (−y, −x)        // 3 plane folds
 *                         if (x+z < 0) (x,z) = (−z, −x)
 *                         if (y+z < 0) (y,z) = (−z, −y)
 *                         p = p · scale − offset · (scale − 1)  // contract
 *
 *                  MENGER preset:
 *                     for i = 1..N:
 *                         p = abs(p)                            // octant fold
 *                         sort_descending(p)                    // 3 swaps
 *                         p = p · scale − offset · (scale − 1)
 *                         if (p.z < −0.5·offset.z·(scale−1))    // z-fold-back
 *                             p.z += offset.z · (scale − 1)
 *
 *                  KIFS_ROT preset:
 *                     for i = 1..N:
 *                         p = rot_y(p, fold_rot)                // animated angle
 *                         p = abs(p)
 *                         if (p.x < p.y) swap                   // 1 swap only
 *                         p = p · scale − offset · (scale − 1)
 *
 *                  Final distance:
 *                     return primitive_distance(p) · scale^(−N)
 *                  The pow() compensates for the cumulative contraction
 *                  so the DE stays in WORLD-space units.
 *
 * Data-structure : Stateless DE function.  No tables, no LUTs.  Each
 *                  pixel re-evaluates the fold sequence ~30 times during
 *                  its sphere trace + ~7 times for normal estimation.
 *
 * Rendering      : ASCII only.  Glyph from Lambert luminance (` .,-+*#%@`),
 *                  colour pair from orbit trap (closest fold-orbit
 *                  approach to origin) so adjacent surfaces in the same
 *                  fold-layer share a hue.  Background = default bg.
 *
 * Performance    : ~30 march steps × ~10 fold iters × ~20 ops = ~6 000
 *                  ops per pixel.  At 80×24 = 1 920 pixels: ~12 M ops/
 *                  frame, easy at 60 fps.  At 200×60: ~70 M/frame, still
 *                  tractable.  Lower iters (i / I) for big terminals.
 *
 * References     :
 *   • Knighty (2010) — original KIFS thread on Fractal Forums:
 *     https://www.fractalforums.com/3d-fractal-generation/kaleidoscopic-(escape-time-ifs)/
 *     The genealogy of the technique.  Folds + scale + iterate.
 *   • Christensen, M. H. — "Distance Estimated 3D Fractals — a Tutorial",
 *     parts I–V (2011), syntopia.github.io. The most accessible
 *     write-up of KIFS, Mandelbox, and the DE compensation `· scale^−N`.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for the
 *     Antialiased Ray Tracing of Implicit Surfaces", *The Visual
 *     Computer* 12(10):527–545.  The sphere-trace iteration we use.
 *   • Quílez, I. — [Distance Functions](https://iquilezles.org/articles/distfunctions/).
 *     The sphere/box primitive distances and the central-difference
 *     normal estimator at the bottom of §5 are from this article.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Self-similarity from origami: take a piece of space, fold it three
 * times along a few mirror planes, then SHRINK it toward a fixed point.
 * Repeat 10–14 times.  Anywhere a fold lands the point onto its mirror
 * image, the inverse shows the SAME structure — that's where the
 * fractal's recursion comes from.  After all the folding, ask "how
 * far am I from a unit sphere right now?"  That number, divided by the
 * total contraction (`scale^N`), is the distance from the original
 * point to the FRACTAL.  Now sphere-trace it.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a grid of mirrors arranged like a kaleidoscope.  Every time
 * you look into a mirror you see a shrunken copy of yourself behind
 * the mirror plane; the reflection reflects again, ad infinitum.  KIFS
 * is the same idea in code: folding `p = abs(p)` is the mathematical
 * statement "for the rest of this iteration, treat p as if it were on
 * the positive side of three mirror planes at x=0, y=0, z=0".  After
 * N folds, the point sits in one tiny chamber of an infinite mirror
 * cathedral.  The distance to a single test sphere in that chamber,
 * scaled back to the outside world, is the distance to the cathedral
 * itself.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. RAY DIRECTION.  For each terminal cell, compute the world-space
 *     ray from the camera position (orbiting at distance R) through
 *     the cell's screen position.  Aspect-corrected for terminal
 *     cells being 2× taller than wide.
 *
 *  2. SPHERE TRACE.  t = 0; for step = 1..MAX_STEPS:
 *
 *        p = origin + t · dir
 *        d = kifs_de(p)
 *        if d < HIT_EPS                    → hit at p
 *        if t > MAX_T                       → miss (escape to background)
 *        t += d
 *
 *  3. KIFS DE.  Inside kifs_de, run N fold iterations:
 *
 *        TETRA   3 plane folds + scale-translate
 *        MENGER  abs + descending sort + scale-translate + z-correct
 *        KIFS_ROT  y-rotate + abs + 1 swap + scale-translate
 *
 *     Alongside, track `orbit_trap = min |p|² over all folds` for
 *     colouring.  Final return:
 *
 *        d_primitive = (sphere or box) distance in folded space
 *        return d_primitive · scale^(−N)
 *
 *  4. NORMAL.  At the hit point, central-difference the DE on each axis
 *     to get the surface normal.  6 extra DE calls per hit pixel.
 *
 *  5. SHADE.  Lambert: lum = 0.15 + 0.85 · max(0, N · L), where L is
 *     a fixed light direction.  Map lum → glyph slot 0..7.  Map orbit
 *     trap → colour pair slot 0..7.  Combine glyph + colour + attr.
 *
 *  6. MISS.  Background — black + space char (default bg shows through).
 *
 *  7. ANIMATE.  Tick advances camera orbit yaw and (for KIFS_ROT) the
 *     fold rotation angle.  Pause freezes both.
 *
 * KEY FORMULAS
 * ────────────
 *  Sphere trace step (Hart 1996):
 *    p_{k+1} = p_k + d(p_k) · ray_dir
 *
 *  Plane fold (reflect about plane with normal n̂, through origin):
 *    if (p · n̂) < 0:  p −= 2 · (p · n̂) · n̂
 *
 *  Octant fold (mirror across all three axis planes):
 *    p = (|p_x|, |p_y|, |p_z|)
 *
 *  Scale-toward-offset (contraction with fixed point at offset):
 *    p = p · scale − offset · (scale − 1)
 *    fixed point: offset itself (verify: offset = offset·s − offset·(s−1) ✓)
 *
 *  DE compensation (un-do cumulative scaling):
 *    d_world = d_folded · scale^(−N)
 *
 *  Central-difference normal (Quílez):
 *    N_x = de(p + (ε,0,0)) − de(p − (ε,0,0))        // similarly y, z
 *    N   = normalise(N_x, N_y, N_z)
 *
 *  Orbit trap (closest approach to origin during fold sequence):
 *    trap = min over i ∈ 1..N of |p_i|
 *    used for colour (smaller trap → "deeper" cathedral interior)
 *
 *  Aspect correction (terminal cells 2× taller than wide):
 *    phys_aspect = (rows · 2) / cols
 *    ray_dir = fwd + u·tan(FOV/2)·right
 *                  + v·tan(FOV/2)·phys_aspect·up
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SCALE NEAR 1.  A scale very close to 1 is non-contractive — the
 *    DE never converges and the fractal becomes a thin shell.  We
 *    clamp `scale ≥ 1.5` per preset to keep the fold loop stable.
 *
 *  • TOO MANY ITERATIONS.  Past iters ≈ 20, scale^(−N) underflows
 *    single-precision floats and the DE returns 0 everywhere.  We
 *    clamp ITERS_MAX = 18.  Pressing `I` past that no-ops.
 *
 *  • CAMERA INSIDE THE FRACTAL.  Zooming in past the bounding sphere
 *    sometimes places the camera inside a fold chamber.  Sphere-trace
 *    then walks AWAY from the surface.  We clamp `cam_dist ≥ 1.6` so
 *    the camera always sits outside the fractal hull.
 *
 *  • BACK-FACE HITS.  KIFS folds can produce concave geometry; the
 *    sphere trace will hit the closer face of any concavity.  Lambert
 *    lighting from a fixed direction makes the back-faces look dark
 *    correctly.  Two-sided shading would wash this out — keep the
 *    one-sided Lambert.
 *
 *  • NORMAL EPS TOO SMALL.  At very high iters the DE becomes nearly
 *    discontinuous between fold cells.  ε too small → noisy normals.
 *    NORMAL_EPS = 4·HIT_EPS empirically gives clean shading.
 *
 *  • ORBIT-TRAP RANGE.  The trap value is unbounded in theory.  In
 *    practice for our presets it sits in [0, 2].  We clamp + scale to
 *    [0, 1] before mapping to the 8-tier colour ramp; values past 2
 *    saturate to the warmest pair.
 *
 *  • PRESET CHANGE.  Switching presets MAY land the camera awkwardly
 *    relative to the new fractal's bounding sphere.  We don't reset
 *    the camera on preset change — `r` does that explicitly.
 *
 *  • PER-FRAME FOLD ROTATION (KIFS_ROT).  The rotation angle is part
 *    of scene state and affects EVERY DE call this frame.  Animating
 *    it across frames produces the morphing crystal effect; if it ever
 *    crosses a symmetry axis the fractal "snaps" briefly — that's the
 *    exact moment the fold loops over a discrete reflection.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press space.  Camera orbit freezes.  Fold rotation freezes.
 *    Resume — motion picks up smoothly.
 *
 *  • Press n a few times.  TETRA → MENGER → KIFS_ROT.  Each preset
 *    has a distinct silhouette: TETRA has a 4-fold tetrahedral
 *    symmetry; MENGER has an obvious 8-cube grid; KIFS_ROT slowly
 *    morphs.
 *
 *  • Press ↓ (decrease iters via i).  Fractal becomes coarser — fewer
 *    levels of recursion, more visible primitive shape (sphere/box).
 *    Press ↑ (I) — fractal becomes more intricate, more cathedral
 *    detail at the cost of CPU.
 *
 *  • Press z / Z.  Camera distance changes; the fractal stays put.
 *    At small cam_dist you should see fold-chamber detail filling
 *    the screen.
 *
 *  • Press t.  Only colours change — geometry is identical across
 *    themes.  GOLD → ICE is the most striking transition.
 *
 *  • At 60 × 20, the fractal silhouette should still read as
 *    architecture, not noise.  If you see speckle, lower iters.
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
    ITERS_MAX        =  18,

    FPS_UPDATE_MS    = 500,

    PAIR_HUD         =   1,
    PAIR_HINT        =   2,
    PAIR_TRAP_BASE   =   3,    /* +0..+7 — orbit-trap colour ramp     */
    PAIR_BG          =  11,    /* fractal "miss" background           */
};

#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* Camera defaults. */
#define CAM_DIST_DEFAULT  3.6f
#define CAM_DIST_MIN      1.6f
#define CAM_DIST_MAX     12.0f
#define CAM_DIST_STEP     0.20f
#define FOV_DEG          55.0f
#define CELL_ASPECT       2.0f      /* terminal cell h/w                 */
#define ORBIT_YAW_RATE    0.35f     /* rad / sec  (auto orbit)           */
#define FOLD_ROT_RATE     0.18f     /* rad / sec  (KIFS_ROT animation)   */
#define MANUAL_YAW_STEP   0.12f
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX  1.20f     /* clamp so we don't flip past the pole */

/* Sphere-trace tunables. */
#define MAX_STEPS         70
#define HIT_EPS           1.5e-3f
#define MAX_T            14.0f
#define NORMAL_EPS        6.0e-3f

typedef enum {
    PRESET_TETRA    = 0,
    PRESET_MENGER   = 1,
    PRESET_KIFS_ROT = 2,
    N_PRESETS       = 3,
} Preset;

/*
 * PresetParams — all per-preset tunables in one row.  scale, offset,
 * default iter count, and a "primitive" tag (sphere = 0, box = 1).
 */
typedef struct {
    const char *name;
    int   default_iters;
    float scale;
    float offset_x, offset_y, offset_z;
    int   primitive;              /* 0 = sphere, 1 = box                */
    float bound_radius;           /* sanity hint for camera distance     */
} PresetParams;

static const PresetParams presets[N_PRESETS] = {
    /*  name        iters  scale   offx offy offz  prim  bound  */
    /* TETRA   */ { "TETRA   ", 12, 2.00f,  1.00f, 1.00f, 1.00f,  0,  1.8f },
    /* MENGER  */ { "MENGER  ",  7, 3.00f,  1.00f, 1.00f, 1.00f,  1,  1.8f },
    /* KIFS_ROT*/ { "KIFS_ROT", 10, 2.05f,  0.85f, 1.10f, 0.85f,  0,  1.8f },
};

/*
 * Themes — 8-step orbit-trap ramp from cool/dim (slot 0) to hot/bright
 * (slot 7).  Per CLAUDE.md, every entry sits in the bright half of the
 * 256-colour cube.
 */
typedef struct {
    const char *name;
    short       trap[8];
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* GOLD   — warm temple gold deepening into bone white */
    { "GOLD   ", { 130, 137, 173, 215, 222, 229, 230, 231 } },
    /* ICE    — pale arctic blue/cyan into pure white      */
    { "ICE    ", {  24,  31,  38,  45,  87, 153, 195, 231 } },
    /* COBALT — deep sea blue → vivid cyan highlights      */
    { "COBALT ", {  25,  26,  27,  33,  39,  45,  51, 159 } },
    /* COPPER — bronze → orange → bone                     */
    { "COPPER ", { 130, 166, 173, 209, 215, 222, 229, 230 } },
    /* ALIEN  — violet → magenta → cyan                    */
    { "ALIEN  ", {  53,  91, 134, 165, 207, 213, 219, 159 } },
    /* MONO   — pure greyscale                             */
    { "MONO   ", { 244, 246, 248, 250, 252, 253, 254, 255 } },
};

/* Luminance ramp: dark → bright, 8 slots. */
static const char LUMA_GLYPHS[8] = { '`', '.', ',', ':', '-', '+', '*', '#' };

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
            init_pair((short)(PAIR_TRAP_BASE + i), t->trap[i], -1);
    } else {
        static const short fb[8] = {
            COLOR_BLUE,    COLOR_MAGENTA, COLOR_CYAN,    COLOR_GREEN,
            COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE,
        };
        for (int i = 0; i < 8; i++)
            init_pair((short)(PAIR_TRAP_BASE + i), fb[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);
        init_pair(PAIR_HINT,  51, -1);
        init_pair(PAIR_BG,   234, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
        init_pair(PAIR_BG,   COLOR_BLACK,  -1);
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
/* §5  kifs — fold loop + DE                                              */
/* ===================================================================== */

/*
 * kifs_fold_iter — one iteration of the fold sequence for the chosen
 * preset.  Modifies p in place.  Tracks orbit-trap if `trap` non-null.
 *
 * Why split fold-iter from full DE: the full DE wraps this in a loop
 * `iters` times, then evaluates a primitive distance and back-scales.
 * Splitting lets the normal estimator (which calls de() six times)
 * share one well-tested fold body.
 */
static inline void kifs_fold_iter_tetra(V3 *p)
{
    if (p->x + p->y < 0) { float t = -p->y; p->y = -p->x; p->x = t; }
    if (p->x + p->z < 0) { float t = -p->z; p->z = -p->x; p->x = t; }
    if (p->y + p->z < 0) { float t = -p->z; p->z = -p->y; p->y = t; }
}

static inline void kifs_fold_iter_menger(V3 *p)
{
    p->x = fabsf(p->x); p->y = fabsf(p->y); p->z = fabsf(p->z);
    if (p->x < p->y) { float t = p->x; p->x = p->y; p->y = t; }
    if (p->x < p->z) { float t = p->x; p->x = p->z; p->z = t; }
    if (p->y < p->z) { float t = p->y; p->y = p->z; p->z = t; }
}

static inline void kifs_fold_iter_rot(V3 *p, float c, float s)
{
    /* Rotate around y axis. */
    float xr = p->x * c - p->z * s;
    float zr = p->x * s + p->z * c;
    p->x = xr; p->z = zr;
    p->x = fabsf(p->x); p->y = fabsf(p->y); p->z = fabsf(p->z);
    if (p->x < p->y) { float t = p->x; p->x = p->y; p->y = t; }
}

/*
 * box_de — distance from origin-centred unit box (signed).  Quílez form.
 * sphere_de — distance from origin-centred unit sphere.
 */
static inline float sphere_de(V3 p) { return v3len(p) - 1.0f; }

static inline float box_de(V3 p)
{
    float qx = fabsf(p.x) - 1.0f;
    float qy = fabsf(p.y) - 1.0f;
    float qz = fabsf(p.z) - 1.0f;
    float dx = fmaxf(qx, 0), dy = fmaxf(qy, 0), dz = fmaxf(qz, 0);
    float outside = sqrtf(dx*dx + dy*dy + dz*dz);
    float inside  = fminf(fmaxf(qx, fmaxf(qy, qz)), 0.0f);
    return outside + inside;
}

/*
 * kifs_de_with_trap — full DE for the active preset.  Also writes the
 * minimum |p|² seen during the fold iterations to *trap_out (the
 * "orbit trap"; smaller → deeper inside the fractal cathedral).
 *
 * Pass NULL for trap_out if you don't need it (sphere-trace marching
 * doesn't need the trap until the final hit).
 */
typedef struct {
    int   preset;
    int   iters;
    float scale, sm1;            /* scale, scale − 1                  */
    float offx, offy, offz;
    float fold_rot_c, fold_rot_s;
    float inv_scale_pow;         /* scale^(−iters) cached              */
} KifsParams;

static float kifs_de_with_trap(V3 p, const KifsParams *kp, float *trap_out)
{
    float trap = 1e10f;

    switch (kp->preset) {

    case PRESET_TETRA:
        for (int i = 0; i < kp->iters; i++) {
            kifs_fold_iter_tetra(&p);
            p.x = p.x * kp->scale - kp->offx * kp->sm1;
            p.y = p.y * kp->scale - kp->offy * kp->sm1;
            p.z = p.z * kp->scale - kp->offz * kp->sm1;
            float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
            if (r2 < trap) trap = r2;
        }
        if (trap_out) *trap_out = sqrtf(trap);
        return sphere_de(p) * kp->inv_scale_pow;

    case PRESET_MENGER:
        for (int i = 0; i < kp->iters; i++) {
            kifs_fold_iter_menger(&p);
            p.x = p.x * kp->scale - kp->offx * kp->sm1;
            p.y = p.y * kp->scale - kp->offy * kp->sm1;
            p.z = p.z * kp->scale - kp->offz * kp->sm1;
            /* z-fold-back keeps p.z in the right range for the box DE. */
            if (p.z < -0.5f * kp->offz * kp->sm1)
                p.z += kp->offz * kp->sm1;
            float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
            if (r2 < trap) trap = r2;
        }
        if (trap_out) *trap_out = sqrtf(trap);
        return box_de(p) * kp->inv_scale_pow;

    case PRESET_KIFS_ROT:
    default:
        for (int i = 0; i < kp->iters; i++) {
            kifs_fold_iter_rot(&p, kp->fold_rot_c, kp->fold_rot_s);
            p.x = p.x * kp->scale - kp->offx * kp->sm1;
            p.y = p.y * kp->scale - kp->offy * kp->sm1;
            p.z = p.z * kp->scale - kp->offz * kp->sm1;
            float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
            if (r2 < trap) trap = r2;
        }
        if (trap_out) *trap_out = sqrtf(trap);
        return sphere_de(p) * kp->inv_scale_pow;
    }
}

static inline float kifs_de(V3 p, const KifsParams *kp)
{
    return kifs_de_with_trap(p, kp, NULL);
}

/*
 * kifs_normal — surface normal at p via central differences.
 *
 * Why central (not forward) differences: forward differences bias the
 * normal toward one side and produce a visible asymmetry in shading
 * when the surface is highly folded.  Central differences cost twice
 * as many DE calls (6 vs 3) but the geometry is correct.
 */
static V3 kifs_normal(V3 p, const KifsParams *kp)
{
    float e = NORMAL_EPS;
    float dx = kifs_de(v3(p.x + e, p.y, p.z), kp)
             - kifs_de(v3(p.x - e, p.y, p.z), kp);
    float dy = kifs_de(v3(p.x, p.y + e, p.z), kp)
             - kifs_de(v3(p.x, p.y - e, p.z), kp);
    float dz = kifs_de(v3(p.x, p.y, p.z + e), kp)
             - kifs_de(v3(p.x, p.y, p.z - e), kp);
    return v3norm(v3(dx, dy, dz));
}

/* ===================================================================== */
/* §6  march — sphere trace + lighting                                    */
/* ===================================================================== */

typedef struct {
    bool  hit;
    V3    p;
    V3    normal;
    float trap;        /* normalised orbit trap                       */
    int   steps;       /* march step count (for AO-style brightness)  */
} Hit;

/*
 * sphere_trace — Hart 1996.  Walk along the ray taking steps equal to
 * the current DE.  Distance estimates are conservative lower bounds on
 * true distance, so we never overshoot the surface.
 *
 * On hit, evaluate trap separately (we DON'T track it during the march
 * — only the FINAL fold position's trap matters for surface colour).
 */
static Hit sphere_trace(V3 origin, V3 dir, const KifsParams *kp)
{
    Hit out = { false, {0,0,0}, {0,1,0}, 0.0f, 0 };
    float t = 0.0f;
    for (int i = 0; i < MAX_STEPS; i++) {
        V3 p = v3add(origin, v3scale(t, dir));
        float d = kifs_de(p, kp);
        if (d < HIT_EPS) {
            out.hit    = true;
            out.p      = p;
            out.steps  = i;
            float trap = 0.0f;
            (void)kifs_de_with_trap(p, kp, &trap);
            /* Empirical: trap typically ∈ [0, 1.4]; scale to [0, 1]. */
            float t_n = trap * (1.0f / 1.4f);
            if (t_n > 1.0f) t_n = 1.0f;
            if (t_n < 0.0f) t_n = 0.0f;
            out.trap   = t_n;
            out.normal = kifs_normal(p, kp);
            return out;
        }
        if (t > MAX_T) break;
        t += d;
    }
    return out;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

typedef struct {
    bool   paused;
    int    current_preset;
    int    current_theme;
    int    iters_override;     /* 0 = use preset default, else override */
    int    cols, rows;

    /* Camera. */
    float  cam_dist;
    float  orbit_yaw;          /* auto-advancing                       */
    float  orbit_pitch;
    float  user_yaw, user_pitch; /* user manual offsets                */

    /* For KIFS_ROT preset. */
    float  fold_rot;           /* current animated rotation angle      */
} Scene;

static int scene_iters(const Scene *s)
{
    int it = (s->iters_override > 0)
           ? s->iters_override
           : presets[s->current_preset].default_iters;
    if (it < ITERS_MIN) it = ITERS_MIN;
    if (it > ITERS_MAX) it = ITERS_MAX;
    return it;
}

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->paused          = false;
    s->current_preset  = PRESET_TETRA;
    s->current_theme   = 0;
    s->iters_override  = 0;
    s->cols            = cols;
    s->rows            = rows;
    s->cam_dist        = CAM_DIST_DEFAULT;
    s->orbit_yaw       = 0.5f;
    s->orbit_pitch     = 0.25f;
    s->user_yaw        = 0.0f;
    s->user_pitch      = 0.0f;
    s->fold_rot        = 0.4f;
}

static void scene_resize(Scene *s, int cols, int rows)
{
    s->cols = cols;
    s->rows = rows;
}

static void scene_reset_cam(Scene *s)
{
    s->cam_dist    = CAM_DIST_DEFAULT;
    s->orbit_yaw   = 0.5f;
    s->orbit_pitch = 0.25f;
    s->user_yaw    = 0.0f;
    s->user_pitch  = 0.0f;
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->orbit_yaw += ORBIT_YAW_RATE * dt;
    if (s->orbit_yaw >  (float)(2.0 * M_PI)) s->orbit_yaw -= (float)(2.0 * M_PI);
    if (s->orbit_yaw < -(float)(2.0 * M_PI)) s->orbit_yaw += (float)(2.0 * M_PI);

    s->fold_rot += FOLD_ROT_RATE * dt;
    if (s->fold_rot >  (float)(2.0 * M_PI)) s->fold_rot -= (float)(2.0 * M_PI);
}

/*
 * scene_build_kifs — pack the per-frame scene state into a flat
 * KifsParams the DE can read without chasing pointers in its hot loop.
 */
static void scene_build_kifs(const Scene *s, KifsParams *kp)
{
    const PresetParams *pp = &presets[s->current_preset];
    int iters = scene_iters(s);
    kp->preset = s->current_preset;
    kp->iters  = iters;
    kp->scale  = pp->scale;
    kp->sm1    = pp->scale - 1.0f;
    kp->offx   = pp->offset_x;
    kp->offy   = pp->offset_y;
    kp->offz   = pp->offset_z;
    kp->fold_rot_c = cosf(s->fold_rot);
    kp->fold_rot_s = sinf(s->fold_rot);
    kp->inv_scale_pow = expf(-(float)iters * logf(pp->scale));
}

/*
 * scene_render — full-frame raymarch.
 *
 * One ray per terminal cell, no virtual canvas / no upscale: terminal
 * cells ARE the pixels.  This keeps the code simple and the visual
 * "honest" at low res (no smoothing artefacts that hide poor sampling).
 *
 * We batch attron/attroff: if the next cell wants the same (pair, attr)
 * as the previous, we skip the switch.  Cuts ~5× attribute thrash on
 * uniform regions.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - 1;
    if (rows_eff < 1) return;

    /* Camera basis (orbit + manual offsets). */
    float yaw   = s->orbit_yaw   + s->user_yaw;
    float pitch = s->orbit_pitch + s->user_pitch;
    if (pitch >  MANUAL_PITCH_MAX) pitch =  MANUAL_PITCH_MAX;
    if (pitch < -MANUAL_PITCH_MAX) pitch = -MANUAL_PITCH_MAX;

    V3 cam_origin = {
        s->cam_dist * cosf(pitch) * cosf(yaw),
        s->cam_dist * sinf(pitch),
        s->cam_dist * cosf(pitch) * sinf(yaw),
    };
    V3 fwd   = v3norm(v3sub(v3(0,0,0), cam_origin));
    V3 wup   = v3(0, 1, 0);
    V3 right = v3norm(v3cross(fwd, wup));
    V3 up    = v3cross(right, fwd);

    /* Light direction — fixed in world space, slightly back-right. */
    V3 light = v3norm(v3(0.55f, 0.75f, 0.35f));

    float fov_t       = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
    float phys_aspect = ((float)rows_eff * CELL_ASPECT) / (float)s->cols;

    KifsParams kp;
    scene_build_kifs(s, &kp);

    int     last_pair = -1;
    attr_t  last_attr = 0;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            float u =  ((float)col + 0.5f) / (float)s->cols * 2.0f - 1.0f;
            float v = -(((float)row + 0.5f) / (float)rows_eff * 2.0f - 1.0f);
            V3 dir = v3norm(v3add(fwd,
                v3add(v3scale(u * fov_t, right),
                      v3scale(v * fov_t * phys_aspect, up))));

            Hit h = sphere_trace(cam_origin, dir, &kp);

            int     pair;
            attr_t  attr;
            char    glyph;

            if (h.hit) {
                /* Lambert + ambient. */
                float ndl = v3dot(h.normal, light);
                if (ndl < 0.0f) ndl = 0.0f;
                float lum = 0.18f + 0.82f * ndl;

                /* Step-count AO: rays that took many steps to converge
                 * sit inside concavities → darken slightly. */
                float ao = 1.0f - (float)h.steps / (float)MAX_STEPS;
                if (ao < 0.6f) ao = 0.6f;
                lum *= ao;

                int slot_lum = (int)(lum * 7.999f);
                if (slot_lum < 0) slot_lum = 0;
                if (slot_lum > 7) slot_lum = 7;

                /* Colour by orbit trap (independent of lighting). */
                int slot_clr = (int)(h.trap * 7.999f);
                if (slot_clr < 0) slot_clr = 0;
                if (slot_clr > 7) slot_clr = 7;

                glyph = LUMA_GLYPHS[slot_lum];
                pair  = PAIR_TRAP_BASE + slot_clr;
                attr  = (slot_lum >= 6) ? A_BOLD
                      : (slot_lum <= 1) ? A_DIM
                      :                   A_NORMAL;
            } else {
                glyph = ' ';
                pair  = PAIR_BG;
                attr  = A_NORMAL;
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
             " KIFS   %s   preset:%s   theme:%s   iters:%2d   "
             "dist:%4.2f   %5.1f fps  %3d Hz   "
             "n/N:preset  t/T:theme  i/I:iters  z/Z:zoom  "
             "arrows:orbit  spc:pause  r:reset  q:quit ",
             s->paused ? "PAUSED" : "RUNNING",
             presets[s->current_preset].name,
             themes[s->current_theme].name,
             scene_iters(s),
             (double)s->cam_dist, fps, sim_fps);

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
    case 'r': case 'R': scene_reset_cam(s); s->iters_override = 0;    break;

    case 'n':
        s->current_preset = (s->current_preset + 1) % N_PRESETS;
        s->iters_override = 0;
        break;
    case 'N':
        s->current_preset = (s->current_preset + N_PRESETS - 1) % N_PRESETS;
        s->iters_override = 0;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    case 'i': {
        int it = scene_iters(s);
        if (it > ITERS_MIN) s->iters_override = it - 1;
        break;
    }
    case 'I': {
        int it = scene_iters(s);
        if (it < ITERS_MAX) s->iters_override = it + 1;
        break;
    }

    case 'z':
        s->cam_dist += CAM_DIST_STEP;
        if (s->cam_dist > CAM_DIST_MAX) s->cam_dist = CAM_DIST_MAX;
        break;
    case 'Z':
        s->cam_dist -= CAM_DIST_STEP;
        if (s->cam_dist < CAM_DIST_MIN) s->cam_dist = CAM_DIST_MIN;
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
