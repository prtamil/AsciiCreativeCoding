/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kifs_fractal.c — Kaleidoscopic IFS (Knighty's folding fractal)
 *
 * DEMO: Sphere-traces a 3-D fractal whose distance estimator (DE) is
 *       built by repeatedly FOLDING space and SCALING toward a fixed
 *       point.  Each fold is a simple reflection; chaining N of them
 *       produces sharp, brutalist, temple-like architecture — alien
 *       angular cathedrals that look nothing like Mandelbulb's organic
 *       blobs.  Surface colour comes from an "orbit trap" (the closest
 *       approach to the origin during the fold sequence), so adjacent
 *       fold-chambers share a hue.  A slow camera orbit + slowly
 *       animated fold rotation keep the fractal walking on its own.
 *
 *       Three presets (cycle with n / N):
 *         TETRA     Sierpinski tetrahedron — three plane folds along
 *                   (x+y), (x+z), (y+z); scale 2 about (1, 1, 1).
 *                   The canonical KIFS — 4-fold symmetric.
 *         MENGER    Menger-sponge variant — abs fold + descending
 *                   sort + scale 3 about (1, 1, 1) + a final z-fold.
 *                   Boxy, 8-fold symmetric, classic recursion grid.
 *         KIFS_ROT  Modified KIFS — per-iteration y-rotation + abs
 *                   fold + 1 swap + scale.  The rotation animates
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
 *   raymarcher/donut.c        — same volumetric/raymarcher folder, but
 *           a far simpler algorithm (parametric point sampling + z-buffer).
 *           Read donut.c first; KIFS is what happens when you replace
 *           "sample a torus surface" with "iterate a fold sequence".
 *   raymarcher/sdf_gallery.c  — primer on simple SDF primitives;
 *           kifs_de() is essentially "fold space first, then ask a
 *           sphere or box SDF in the folded space".
 *   raster/mandelbulb_raster.c — same fractal-DE family but spherical
 *           iteration produces ORGANIC blobs; KIFS produces ANGULAR
 *           architecture.  Read both to feel the difference.
 *
 * Section map:
 *   §1 config   — frame, camera, sphere-trace, lighting, presets, themes
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — orbit-trap palette + HUD/hint pairs (CLAUDE.md spec)
 *   §4 vec3     — value-type 3-D math
 *   §5 kifs     — per-iteration folds + DE + normal estimator
 *   §6 march    — sphere-trace + Hit struct
 *   §7 scene    — camera basis, ray gen, shade, render orchestrator
 *   §8 screen   — ncurses init / resize / HUD draw / present
 *   §9 app      — main loop, signals, keys
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

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sphere tracing of a Kaleidoscopic Iterated Function
 *                  System (KIFS) distance estimator.
 *
 *                  The DE is built by REPEATEDLY FOLDING and SCALING
 *                  the input point, then evaluating the distance to a
 *                  simple primitive (sphere or box) in the folded
 *                  space.  Each fold is a piecewise affine map (a
 *                  reflection across a plane); chaining N of them
 *                  produces a self-similar fractal because the
 *                  inverse of each fold replicates the primitive
 *                  across the fold axis.  Critically, fold + scale
 *                  together compose to a *contractive* map, so every
 *                  point in space orbits toward the fractal attractor.
 *
 *                  One sphere-trace step:
 *                     d = kifs_de(p)        // distance to fractal
 *                     p += d · ray_dir      // safe step, never overshoots
 *                  Repeat until d < ε (hit) or t > MAX_T (miss).
 *
 *                  The iteration body is the SAME shape for every preset:
 *                     for i = 1..N:
 *                         fold_iter        (preset-specific reflection)
 *                         contract         (p · scale − offset · (s−1))
 *                         menger_z_foldback (only the MENGER preset)
 *                         track_orbit_trap (running closest-to-origin)
 *                     return primitive_de(p) · scale^(−N)
 *
 *                  The final scale^(−N) compensates for the cumulative
 *                  contraction so the DE stays in WORLD-space units.
 *                  (If you forget it, the fractal is the same SHAPE
 *                  but the sphere-trace overshoots and never hits.)
 *
 * Data-structure : Stateless DE function.  No tables, no LUTs, no
 *                  caching.  Each pixel re-evaluates the fold
 *                  sequence ~30 times during its sphere trace + 6
 *                  times for normal estimation.  All per-frame
 *                  parameters live in `KifsParams` (preset, iters,
 *                  scale, offset, fold rotation, cached scale^−N) —
 *                  built once in `scene_build_kifs` and passed by
 *                  const-pointer to every DE call.
 *
 * Rendering      : ASCII only.  Glyph from Lambertian luminance plus
 *                  step-count AO; colour pair from orbit trap (closest
 *                  fold-orbit approach to origin) so adjacent surfaces
 *                  in the same fold-chamber share a hue.  Background
 *                  is the default terminal bg.
 *
 * Performance    : ~30 march steps × ~10 fold iters × ~20 ops ≈ 6 000
 *                  ops per pixel.  At 80×24 = 1 920 pixels: ~12 M ops/
 *                  frame, easy at 60 fps.  At 200×60: ~70 M/frame, still
 *                  tractable.  Lower iters (i / I) for big terminals.
 *
 * References     :
 *   • Knighty (2010) — original KIFS thread on Fractal Forums.
 *     The genealogy of the technique: folds + scale + iterate.
 *     https://www.fractalforums.com/3d-fractal-generation/kaleidoscopic-(escape-time-ifs)/
 *   • Christensen, M. H. — "Distance Estimated 3D Fractals — a
 *     Tutorial," parts I–V (2011), syntopia.github.io.  The most
 *     accessible write-up of KIFS, Mandelbox, and the DE
 *     compensation `· scale^−N`.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces," *The Visual
 *     Computer* 12(10):527–545.  The sphere-trace iteration we use.
 *   • Quílez, I. — "Distance Functions"
 *     https://iquilezles.org/articles/distfunctions/
 *     Source of the sphere/box primitive distances and the central-
 *     difference normal estimator at the bottom of §5.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Self-similarity from origami.  Take a piece of space, fold it three
 * times along a few mirror planes, then SHRINK it toward a fixed
 * point.  Repeat 10–14 times.  Anywhere a fold lands a point onto its
 * mirror image, the inverse shows the SAME structure — that's where
 * the fractal's recursion comes from.  After all the folding, ask
 * "how far am I from a unit sphere right now?"  That number, divided
 * by the total contraction (scale^N), is the distance from the
 * original point to the FRACTAL.  Now sphere-trace it.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a grid of mirrors arranged like a kaleidoscope.  Every
 * time you look into a mirror you see a shrunken copy of yourself
 * behind the mirror plane; the reflection reflects again, ad
 * infinitum.  KIFS is the same idea in code: folding `p = abs(p)` is
 * the mathematical statement "for the rest of this iteration, treat
 * p as if it were on the positive side of three mirror planes at
 * x=0, y=0, z=0".  After N folds, the point sits in one tiny chamber
 * of an infinite mirror cathedral.  The distance to a single test
 * sphere in that chamber, scaled back to the outside world, is the
 * distance to the cathedral itself.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │  outside world         (fold sequence)         attractor │
 *      │     ●     ─────fold─────►  ●  ─fold─►  ●  ─►  ··· →  ◆   │
 *      │     ↑                                                    │
 *      │  ray hit p                                  test sphere  │
 *      │                                                          │
 *      │  d_world(p) = d_test(folded_p) · scale^(−N)              │
 *      └─────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  Per pixel:
 *
 *    1. RAY DIRECTION.  Build the world-space ray from the camera
 *       through the cell, aspect-corrected for terminal cells being
 *       2× taller than wide.
 *
 *    2. SPHERE TRACE.  t = 0; for step = 1..MAX_STEPS:
 *           p = origin + t · dir
 *           d = kifs_de(p)
 *           if d < HIT_EPS  → hit
 *           if t > MAX_T    → miss (background)
 *           t += d
 *
 *    3. KIFS DE.  Inside `kifs_de_with_trap`, run N fold iterations
 *       (each iteration ~4 lines):
 *           fold_iter           preset-specific reflection
 *           contract            p ← p · scale − offset · (scale−1)
 *           menger_z_foldback   only when preset == MENGER
 *           track_orbit_trap    update min |p|² seen
 *       Then return primitive_de(p) · scale^(−N).
 *
 *    4. NORMAL.  At the hit point, central-difference the DE on each
 *       axis (6 extra DE calls) → surface normal.
 *
 *    5. SHADE.  Lambert: lum = AMBIENT_LUM + DIFFUSE_LUM · max(0, N·L).
 *       Multiply by a step-count AO factor (deeper concavities took
 *       more march steps → darker).  Map lum → glyph slot 0..7,
 *       trap → colour-pair slot 0..7.
 *
 *    6. MISS.  Background — space char, default bg.
 *
 *  Per frame:
 *    7. ANIMATE.  scene_tick advances orbit yaw and (for KIFS_ROT)
 *       the fold rotation angle.
 *
 * KEY FORMULAS
 * ────────────
 *   Sphere trace step (Hart 1996):
 *     p_{k+1} = p_k + d(p_k) · ray_dir
 *
 *   Plane fold (reflect about plane with normal n̂, through origin):
 *     if (p · n̂) < 0:  p −= 2 · (p · n̂) · n̂
 *
 *   Octant fold (mirror across all three axis planes):
 *     p = (|p_x|, |p_y|, |p_z|)
 *
 *   Scale-toward-offset (contraction with fixed point at offset):
 *     p = p · scale − offset · (scale − 1)
 *     fixed-point check: p* = offset:
 *         offset · scale − offset · (scale − 1) = offset       ✓
 *
 *   DE compensation (un-do cumulative scaling):
 *     d_world = d_folded · scale^(−N)
 *
 *   Central-difference normal (Quílez):
 *     N_x = de(p + ε x̂) − de(p − ε x̂)         (similarly y, z)
 *     N   = normalise(N_x, N_y, N_z)
 *
 *   Orbit trap (closest approach to origin during fold sequence):
 *     trap = min over i ∈ 1..N of |p_i|
 *     used for colour: smaller trap → "deeper" cathedral interior.
 *
 *   Aspect correction (cells 2× taller than wide):
 *     phys_aspect = (rows · 2) / cols
 *     ray = fwd
 *         + u · tan(FOV/2)              · right
 *         + v · tan(FOV/2) · phys_aspect · up
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • SCALE NEAR 1.  A scale very close to 1 is non-contractive — the
 *     DE never converges and the fractal becomes a thin shell.  Per
 *     preset, we keep `scale ≥ 2.0`.
 *
 *   • TOO MANY ITERATIONS.  Past iters ≈ 20, scale^(−N) underflows
 *     single-precision and the DE returns 0 everywhere.  We clamp
 *     ITERS_MAX = 18.  Pressing `I` past that no-ops.
 *
 *   • CAMERA INSIDE THE FRACTAL.  Zooming in past the bounding sphere
 *     can place the camera inside a fold chamber; the trace then walks
 *     AWAY from the surface.  We clamp `cam_dist ≥ CAM_DIST_MIN` so
 *     the camera always sits outside the fractal hull.
 *
 *   • BACK-FACE HITS.  KIFS folds produce concavities; the trace
 *     hits the closer face first.  One-sided Lambert (ambient + N·L)
 *     keeps back-faces correctly dark.  Two-sided shading would wash
 *     this out.
 *
 *   • NORMAL EPS.  At very high iters the DE becomes nearly
 *     discontinuous between fold cells.  ε too small → noisy
 *     normals.  NORMAL_EPS = 4 · HIT_EPS empirically gives clean
 *     shading.
 *
 *   • ORBIT-TRAP RANGE.  In theory unbounded; in practice the trap
 *     for our presets sits in [0, ~1.4].  We clamp + scale to [0, 1]
 *     before mapping to the 8-tier ramp; values past 1.4 saturate to
 *     the warmest pair.
 *
 *   • PRESET CHANGE doesn't reset the camera (you might want to keep
 *     your view).  Press `r` to reset both camera and iters.
 *
 *   • PER-FRAME FOLD ROTATION (KIFS_ROT).  The rotation angle is part
 *     of scene state and affects EVERY DE call this frame.  As it
 *     advances each frame the fractal slowly morphs; if it crosses a
 *     symmetry axis the fractal "snaps" briefly — that's the moment
 *     the fold loops over a discrete reflection.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press space.  Camera orbit freezes.  Fold rotation freezes.
 *     Resume — motion picks up smoothly.
 *
 *   • Press n a few times.  TETRA → MENGER → KIFS_ROT.  Each preset
 *     has a distinct silhouette: TETRA has 4-fold tetrahedral
 *     symmetry; MENGER has an obvious 8-cube grid; KIFS_ROT slowly
 *     morphs.
 *
 *   • Press i (decrease iters).  Fractal becomes coarser — fewer
 *     levels of recursion, more visible primitive shape.  Press I
 *     (increase) → more cathedral detail at the cost of CPU.
 *
 *   • Press z / Z.  Camera distance changes; the fractal stays put.
 *     At small cam_dist you should see fold-chamber detail filling
 *     the screen.
 *
 *   • Press t.  Only colours change — geometry is identical across
 *     themes.  GOLD → ICE is the most striking transition.
 *
 *   • At 60 × 20, the fractal silhouette should still read as
 *     architecture, not noise.  If you see speckle, lower iters.
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
    HUD_ROWS         =   2,         /* row 0 status + last row hint */
    ITERS_MIN        =   3,
    ITERS_MAX        =  18,
};

/* §1.2 colour-pair IDs (CLAUDE.md HUD pair numbering at the top). */
enum {
    PAIR_HUD         =  1,          /* yellow status row 0          */
    PAIR_HINT        =  2,          /* cyan key-hint last row       */
    PAIR_TRAP_BASE   =  3,          /* +0..+7 — orbit-trap ramp     */
    PAIR_BG          = 11,          /* fractal "miss" background    */
};

/* §1.3 time helpers. */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL
#define TICK_NS(f)      (NS_PER_SEC / (f))

/* §1.4 camera. */
#define CAM_DIST_DEFAULT  3.6f
#define CAM_DIST_MIN      1.6f      /* don't enter the fractal hull */
#define CAM_DIST_MAX     12.0f
#define CAM_DIST_STEP     0.20f
#define FOV_DEG          55.0f
#define CELL_ASPECT       2.0f      /* terminal cell h / w           */
#define ORBIT_YAW_RATE    0.35f     /* rad / sec — auto orbit         */
#define FOLD_ROT_RATE     0.18f     /* rad / sec — KIFS_ROT animation */
#define MANUAL_YAW_STEP   0.12f
#define MANUAL_PITCH_STEP 0.08f
#define MANUAL_PITCH_MAX  1.20f     /* clamp so we don't flip past pole */

/* §1.5 sphere-trace tunables.
 *
 * MAX_STEPS    march hard limit (Hart's iteration may not converge in
 *              concave regions; this stops it from running forever).
 * HIT_EPS      surface threshold; "we're touching the fractal".
 * MAX_T        total ray length before we declare a miss.
 * NORMAL_EPS   gradient sample step for the central-difference
 *              normal; should be ≈ 4 × HIT_EPS for clean shading. */
#define MAX_STEPS         70
#define HIT_EPS           1.5e-3f
#define MAX_T            14.0f
#define NORMAL_EPS        6.0e-3f

/* §1.6 lighting + shading.
 *
 * Lambertian model: lum = AMBIENT_LUM + DIFFUSE_LUM · max(0, N·L)
 * Step-count "AO": more steps to converge → deeper concavity → dimmer.
 * The AO factor never falls below AO_FLOOR so very deep concavities
 * still show their structure (don't fade to pure black). */
#define AMBIENT_LUM       0.18f
#define DIFFUSE_LUM       0.82f
#define AO_FLOOR          0.60f

/* §1.7 quantisation — number of glyph slots / colour slots.
 *
 *   LUMA_SLOTS               eight glyph tiers in LUMA_GLYPHS[]
 *   LUMA_SLOT_FLT            float scaler used for [0, 1) → [0, 7]
 *   TRAP_NORM_RANGE          empirical max trap value (in practice
 *                            our presets sit in [0, ~1.4]); we divide
 *                            and clamp before mapping to colour slot. */
#define LUMA_SLOTS         8
#define LUMA_SLOT_FLT      7.999f       /* (LUMA_SLOTS - 0.001) */
#define TRAP_NORM_RANGE    1.4f
#define TRAP_NORM_INV      (1.0f / TRAP_NORM_RANGE)

/* §1.8 fractal preset table.
 *
 * Each row defines one preset's geometry: how many fold iterations
 * by default, the contraction scale, the contraction fixed point
 * (offset), and which primitive to evaluate after the folds.
 *
 * primitive: 0 = sphere SDF, 1 = box SDF.
 */
typedef enum {
    PRESET_TETRA    = 0,
    PRESET_MENGER   = 1,
    PRESET_KIFS_ROT = 2,
    N_PRESETS       = 3,
} Preset;

typedef struct {
    const char *name;
    int   default_iters;
    float scale;
    float offset_x, offset_y, offset_z;
    int   primitive;            /* 0 = sphere, 1 = box */
    float bound_radius;         /* hint for camera distance */
} PresetParams;

static const PresetParams PRESETS[N_PRESETS] = {
    /*   name        iters scale  offx offy offz  prim  bound */
    /* TETRA   */  { "TETRA   ", 12, 2.00f, 1.00f, 1.00f, 1.00f,  0,  1.8f },
    /* MENGER  */  { "MENGER  ",  7, 3.00f, 1.00f, 1.00f, 1.00f,  1,  1.8f },
    /* KIFS_ROT*/  { "KIFS_ROT", 10, 2.05f, 0.85f, 1.10f, 0.85f,  0,  1.8f },
};

/* §1.9 themes — 8-step orbit-trap ramp.  Per CLAUDE.md, every entry
 * sits in the bright half of the 256-colour cube. */
typedef struct {
    const char *name;
    short       trap[LUMA_SLOTS];
} Theme;

#define N_THEMES 6

static const Theme THEMES[N_THEMES] = {
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

/* §1.10 luminance ramp — dim → bright. */
static const char LUMA_GLYPHS[LUMA_SLOTS] = {
    '`', '.', ',', ':', '-', '+', '*', '#'
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

/* ── §3 color — orbit-trap palette + HUD/hint pairs ──────────────────── */

/*
 * theme_apply — re-init the eight orbit-trap pairs for the chosen
 * theme.  Called at startup and on every t / T keypress.  Geometry is
 * unaffected; only the trap → colour table changes.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &THEMES[idx];
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_TRAP_BASE + i), t->trap[i], -1);
    } else {
        static const short FB[LUMA_SLOTS] = {
            COLOR_BLUE,    COLOR_MAGENTA, COLOR_CYAN,    COLOR_GREEN,
            COLOR_YELLOW,  COLOR_YELLOW,  COLOR_WHITE,   COLOR_WHITE,
        };
        for (int i = 0; i < LUMA_SLOTS; i++)
            init_pair((short)(PAIR_TRAP_BASE + i), FB[i], -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);   /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);   /* bright cyan   */
        init_pair(PAIR_BG,   242, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
        init_pair(PAIR_BG,   COLOR_BLACK,  -1);
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

/* ── §5 kifs — fold loop + DE + normal estimator ─────────────────────── *
 *
 * The DE has the same SHAPE for every preset:
 *
 *     for i = 1..iters:
 *         fold_iter             preset-specific reflection
 *         contract              p ← p · scale − offset · (scale−1)
 *         menger_z_foldback     only when preset == MENGER
 *         track_orbit_trap      update min |p|² seen so far
 *     return primitive_de(p) · inv_scale_pow
 *
 * Each helper below is one mathematical step in that pseudocode.  The
 * orchestrator `kifs_de_with_trap` reads top-to-bottom as the four
 * lines above, with no buried logic.
 *
 * KifsParams holds every per-frame parameter the DE needs in registers
 * — it's built once per frame in scene_build_kifs and passed by const
 * pointer to the DE so the hot loop never chases pointers into Scene.
 */

typedef struct {
    int   preset;
    int   iters;
    float scale;
    float sm1;             /* scale − 1                          */
    float offx, offy, offz;
    float fold_rot_c;      /* cos(fold_rot)  — KIFS_ROT only     */
    float fold_rot_s;      /* sin(fold_rot)  — KIFS_ROT only     */
    float inv_scale_pow;   /* scale^(−iters), cached             */
} KifsParams;

/* §5.1 fold helpers — one per preset.  Mutate p in place. */

/*
 * fold_iter_tetra — three plane folds making the Sierpinski
 * tetrahedron.  Each `if` reflects p across one of the three planes
 * x+y=0, x+z=0, y+z=0 (only when p is on the wrong side).  The four
 * fixed points of the resulting iteration are the tetrahedron's
 * vertices.
 */
static inline void fold_iter_tetra(V3 *p)
{
    if (p->x + p->y < 0) { float t = -p->y; p->y = -p->x; p->x = t; }
    if (p->x + p->z < 0) { float t = -p->z; p->z = -p->x; p->x = t; }
    if (p->y + p->z < 0) { float t = -p->z; p->z = -p->y; p->y = t; }
}

/*
 * fold_iter_menger — abs-fold (mirror across all three axis planes)
 * followed by a descending sort.  After the sort, p.x ≥ p.y ≥ p.z;
 * combined with the contract step that follows this gives the Menger-
 * sponge cell removal pattern.
 */
static inline void fold_iter_menger(V3 *p)
{
    p->x = fabsf(p->x); p->y = fabsf(p->y); p->z = fabsf(p->z);
    if (p->x < p->y) { float t = p->x; p->x = p->y; p->y = t; }
    if (p->x < p->z) { float t = p->x; p->x = p->z; p->z = t; }
    if (p->y < p->z) { float t = p->y; p->y = p->z; p->z = t; }
}

/*
 * fold_iter_rot — KIFS_ROT preset: rotate around y by the (animated)
 * fold rotation, then abs-fold + a single swap.  The rotation is what
 * gives KIFS_ROT its characteristic morphing-crystal look as the
 * angle advances each frame.
 */
static inline void fold_iter_rot(V3 *p, float c, float s)
{
    float xr = p->x * c - p->z * s;
    float zr = p->x * s + p->z * c;
    p->x = xr; p->z = zr;
    p->x = fabsf(p->x); p->y = fabsf(p->y); p->z = fabsf(p->z);
    if (p->x < p->y) { float t = p->x; p->x = p->y; p->y = t; }
}

/*
 * fold_iter — dispatch to the right fold by preset.  The compiler
 * inlines this and the switch becomes a single branch on a value
 * that's constant within a frame, so the branch predictor wins
 * trivially.
 */
static inline void fold_iter(V3 *p, const KifsParams *kp)
{
    switch (kp->preset) {
    case PRESET_TETRA:    fold_iter_tetra (p);                                 break;
    case PRESET_MENGER:   fold_iter_menger(p);                                 break;
    case PRESET_KIFS_ROT: fold_iter_rot   (p, kp->fold_rot_c, kp->fold_rot_s); break;
    }
}

/* §5.2 contract + menger fold-back — shared post-steps. */

/*
 * contract_toward_offset — the contractive map after each fold.
 *
 *    p ← p · scale  −  offset · (scale − 1)
 *
 * Fixed point: p* = offset.  Verify: offset·s − offset·(s−1) = offset.
 * After many iterations every point in space is dragged to one of the
 * map's attractor leaves; THAT'S the fractal we render.
 */
static inline void contract_toward_offset(V3 *p, const KifsParams *kp)
{
    p->x = p->x * kp->scale - kp->offx * kp->sm1;
    p->y = p->y * kp->scale - kp->offy * kp->sm1;
    p->z = p->z * kp->scale - kp->offz * kp->sm1;
}

/*
 * menger_z_foldback — MENGER-only post-step.  Pulls p.z back into a
 * sensible range for the box DE that follows so we don't end up
 * measuring distance from a point on the WRONG side of the box.
 *
 * Pure heuristic — every public KIFS implementation includes this
 * line for the Menger preset; without it the central column reads
 * the wrong distance and the recursion ladder visibly breaks.
 */
static inline void menger_z_foldback(V3 *p, const KifsParams *kp)
{
    if (p->z < -0.5f * kp->offz * kp->sm1)
        p->z += kp->offz * kp->sm1;
}

/*
 * track_orbit_trap — running minimum of |p|² across all fold
 * iterations.  Smaller trap = the point passed CLOSER to the origin
 * during folding, which empirically corresponds to "deeper inside
 * the cathedral".  The final scaled value drives the colour pair.
 */
static inline void track_orbit_trap(V3 p, float *trap_sq)
{
    float r2 = p.x*p.x + p.y*p.y + p.z*p.z;
    if (r2 < *trap_sq) *trap_sq = r2;
}

/* §5.3 primitive DEs (Quílez form). */

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

static inline float primitive_de(int preset, V3 p)
{
    return (preset == PRESET_MENGER) ? box_de(p) : sphere_de(p);
}

/* §5.4 the orchestrator — one tiny loop. */

/*
 * kifs_de_with_trap — full DE for the active preset.  Pseudocode:
 *
 *     trap = ∞
 *     for i = 1..iters:
 *         fold_iter
 *         contract_toward_offset
 *         menger_z_foldback   (only MENGER)
 *         track_orbit_trap
 *     trap_out = √trap
 *     return primitive_de(p) · scale^(−iters)
 *
 * Pass `trap_out = NULL` to skip the trap copy-out (the sphere-trace
 * only needs it on hit, not during marching).
 */
static float kifs_de_with_trap(V3 p, const KifsParams *kp, float *trap_out)
{
    float trap_sq = 1e10f;

    for (int i = 0; i < kp->iters; i++) {
        fold_iter              (&p, kp);
        contract_toward_offset (&p, kp);
        if (kp->preset == PRESET_MENGER)
            menger_z_foldback  (&p, kp);
        track_orbit_trap       (p, &trap_sq);
    }

    if (trap_out) *trap_out = sqrtf(trap_sq);
    return primitive_de(kp->preset, p) * kp->inv_scale_pow;
}

static inline float kifs_de(V3 p, const KifsParams *kp)
{
    return kifs_de_with_trap(p, kp, NULL);
}

/*
 * kifs_normal — surface normal at p via central differences.
 *
 *     N_x = de(p + ε x̂) − de(p − ε x̂)
 *     N_y = de(p + ε ŷ) − de(p − ε ŷ)
 *     N_z = de(p + ε ẑ) − de(p − ε ẑ)
 *     N   = normalise(N_x, N_y, N_z)
 *
 * Central (not forward) differences cost twice as many DE calls
 * (6 vs 3) but the geometry comes out symmetric — forward differences
 * bias the normal toward one axis and produce visibly skewed shading
 * on highly-folded surfaces.
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

/* ── §6 march — sphere trace ─────────────────────────────────────────── */

/*
 * Hit — what the sphere trace returns to the shader.
 *
 *   hit       did the ray reach the surface?
 *   p         hit position (only valid if hit)
 *   normal    surface normal at p (only valid if hit)
 *   trap      orbit-trap value normalised to [0, 1]
 *   steps     march iteration count — used as a cheap AO signal
 *             (deeper concavities take more steps to converge)
 */
typedef struct {
    bool  hit;
    V3    p;
    V3    normal;
    float trap;
    int   steps;
} Hit;

/*
 * sphere_trace — Hart 1996.  Walk along the ray taking steps equal to
 * the current DE.  Distance estimates are conservative LOWER bounds on
 * true distance, so we never overshoot the surface.  Iterate until
 * either |d| drops below HIT_EPS (hit) or t exceeds MAX_T (miss).
 *
 * On hit we re-evaluate the DE once more to extract the orbit trap
 * (we don't track it during the march because only the LAST fold
 * position's trap matters for colour).  This costs one extra DE
 * evaluation per hit pixel — cheap relative to the normal estimator.
 */
static Hit sphere_trace(V3 origin, V3 dir, const KifsParams *kp)
{
    Hit out = { false, {0,0,0}, {0,1,0}, 0.0f, 0 };
    float t = 0.0f;

    for (int i = 0; i < MAX_STEPS; i++) {
        V3    p = v3add(origin, v3scale(t, dir));
        float d = kifs_de(p, kp);

        if (d < HIT_EPS) {
            float trap = 0.0f;
            (void)kifs_de_with_trap(p, kp, &trap);

            out.hit    = true;
            out.p      = p;
            out.steps  = i;
            out.trap   = clampf(trap * TRAP_NORM_INV, 0.0f, 1.0f);
            out.normal = kifs_normal(p, kp);
            return out;
        }

        if (t > MAX_T) break;
        t += d;
    }
    return out;
}

/* ── §7 scene — camera basis, ray gen, shade, render orchestrator ────── */

typedef struct {
    bool   paused;
    int    current_preset;
    int    current_theme;
    int    iters_override;       /* 0 = use preset default */
    int    cols, rows;

    /* Camera state. */
    float  cam_dist;
    float  orbit_yaw;             /* auto-advancing */
    float  orbit_pitch;
    float  user_yaw, user_pitch;  /* manual offsets via arrow keys */

    /* KIFS_ROT animated angle (also used by scene_build_kifs). */
    float  fold_rot;
} Scene;

/*
 * scene_iters — how many fold iterations the active preset runs.
 * Defaults to the preset's value; user can override via i / I.
 */
static int scene_iters(const Scene *s)
{
    int it = (s->iters_override > 0)
           ? s->iters_override
           : PRESETS[s->current_preset].default_iters;
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

    s->fold_rot  += FOLD_ROT_RATE * dt;
    if (s->fold_rot >  (float)(2.0 * M_PI)) s->fold_rot -= (float)(2.0 * M_PI);
}

/*
 * scene_build_kifs — pack per-frame state into a flat KifsParams the
 * DE inner loop can read without chasing pointers.  Called ONCE per
 * frame.  Caches `inv_scale_pow = scale^(−iters)` so the per-pixel
 * DE doesn't recompute it.
 */
static void scene_build_kifs(const Scene *s, KifsParams *kp)
{
    const PresetParams *pp = &PRESETS[s->current_preset];
    int iters = scene_iters(s);

    kp->preset        = s->current_preset;
    kp->iters         = iters;
    kp->scale         = pp->scale;
    kp->sm1           = pp->scale - 1.0f;
    kp->offx          = pp->offset_x;
    kp->offy          = pp->offset_y;
    kp->offz          = pp->offset_z;
    kp->fold_rot_c    = cosf(s->fold_rot);
    kp->fold_rot_s    = sinf(s->fold_rot);
    kp->inv_scale_pow = expf(-(float)iters * logf(pp->scale));
}

/* §7.1 camera basis + per-pixel ray. */

typedef struct {
    V3    origin;
    V3    fwd, right, up;
    float fov_t;
    float phys_aspect;
} Camera;

/*
 * camera_basis — orthonormal (fwd, right, up) at the orbiting camera
 * position, plus the FOV tangent and aspect correction for ray gen.
 *
 *   yaw   = orbit_yaw + user_yaw
 *   pitch = clamp(orbit_pitch + user_pitch, ±MAX_PITCH)
 *   eye   = cam_dist · (cos pitch · cos yaw, sin pitch, cos pitch · sin yaw)
 *   fwd   = normalise(origin − eye) = normalise(−eye)
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
    c.fwd        = v3norm(v3sub(v3(0, 0, 0), c.origin));
    V3 wup       = v3(0, 1, 0);
    c.right      = v3norm(v3cross(c.fwd, wup));
    c.up         = v3cross(c.right, c.fwd);
    c.fov_t      = tanf(FOV_DEG * (float)M_PI / 180.0f * 0.5f);
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

/* §7.2 lighting + shading. */

/*
 * lambert_with_ao — Lambertian shading + step-count AO.
 *
 *   diffuse = max(0, N · L)
 *   lum     = AMBIENT_LUM + DIFFUSE_LUM · diffuse
 *   ao      = max(AO_FLOOR, 1 − steps / MAX_STEPS)
 *   final   = lum · ao
 *
 * AO floor stops very-deep concavities from washing out completely,
 * so the cathedral interior stays readable.
 */
static float lambert_with_ao(V3 normal, int steps, V3 light)
{
    float ndl = v3dot(normal, light);
    if (ndl < 0.0f) ndl = 0.0f;
    float lum = AMBIENT_LUM + DIFFUSE_LUM * ndl;

    float ao = 1.0f - (float)steps / (float)MAX_STEPS;
    if (ao < AO_FLOOR) ao = AO_FLOOR;
    return lum * ao;
}

/*
 * to_slot — quantise a [0, 1] value to an integer slot index 0..7.
 * Clamps gracefully on out-of-range inputs.
 */
static int to_slot(float x_01)
{
    int s = (int)(x_01 * LUMA_SLOT_FLT);
    if (s < 0)               s = 0;
    if (s >= LUMA_SLOTS)     s = LUMA_SLOTS - 1;
    return s;
}

/*
 * Cell — the (glyph, colour pair, attribute) decoration of one
 * terminal cell.  shade_hit returns this; emit_cell paints it.
 */
typedef struct { char glyph; int pair; attr_t attr; } Cell;

/*
 * shade_hit — given the sphere-trace result and the world light
 * direction, decide what to draw in this cell.
 *
 *   miss → background
 *   hit  → glyph from luma slot,  colour from orbit-trap slot,
 *          attr (BOLD / NORMAL / DIM) from luma slot
 */
static Cell shade_hit(const Hit *h, V3 light)
{
    if (!h->hit) {
        return (Cell){ ' ', PAIR_BG, A_NORMAL };
    }

    float lum    = lambert_with_ao(h->normal, h->steps, light);
    int   s_lum  = to_slot(lum);
    int   s_clr  = to_slot(h->trap);

    return (Cell){
        .glyph = LUMA_GLYPHS[s_lum],
        .pair  = PAIR_TRAP_BASE + s_clr,
        .attr  = (s_lum >= 6) ? A_BOLD
               : (s_lum <= 1) ? A_DIM
               :                A_NORMAL,
    };
}

/*
 * emit_cell — paint one cell, batching attron/attroff so we only
 * call them when (pair, attr) actually changes.  Halves attribute
 * thrash on uniform regions.
 */
static void emit_cell(int row, int col, Cell c,
                      int *last_pair, attr_t *last_attr)
{
    if (c.pair != *last_pair || c.attr != *last_attr) {
        if (*last_pair >= 0) attroff(COLOR_PAIR(*last_pair) | *last_attr);
        attron(COLOR_PAIR(c.pair) | c.attr);
        *last_pair = c.pair;
        *last_attr = c.attr;
    }
    mvaddch(row, col, (chtype)(unsigned char)c.glyph);
}

/* §7.3 the orchestrator — one tiny double loop. */

/*
 * scene_render — full-frame raymarch.  Reads top-to-bottom as the
 * algorithm pseudocode itself: each line is one of the named helpers.
 *
 * One ray per terminal cell, no virtual canvas / no upscale: terminal
 * cells ARE the pixels.  Visual is "honest" at low res — no smoothing
 * artefacts that hide poor sampling.
 */
static void scene_render(const Scene *s)
{
    int rows_eff = s->rows - HUD_ROWS;
    if (rows_eff < 1) return;

    Camera     cam   = camera_basis(s, rows_eff);
    V3         light = v3norm(v3(0.55f, 0.75f, 0.35f));
    KifsParams kp;
    scene_build_kifs(s, &kp);

    int    last_pair = -1;
    attr_t last_attr = 0;

    /* HUD takes row 0 (status) and row rows-1 (hint).  We render the
     * scene into rows [1, rows-2] inclusive so neither overlaps. */
    int y0 = 1;

    for (int row = 0; row < rows_eff; row++) {
        for (int col = 0; col < s->cols; col++) {
            V3   ray = pixel_ray (col, row, s->cols, rows_eff, &cam);
            Hit  h   = sphere_trace(cam.origin, ray, &kp);
            Cell c   = shade_hit (&h, light);
            emit_cell(y0 + row, col, c, &last_pair, &last_attr);
        }
    }

    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ── §8 screen — ncurses init / resize / HUD draw / present ──────────── */

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
 * fractal colour underneath.
 */
static void hud_draw(const Screen *sc, const Scene *s,
                     double fps, int sim_fps)
{
    /* Top row: yellow title + status. */
    char status[140];
    snprintf(status, sizeof status,
             " %5.1f fps  %3d Hz  preset:%s  theme:%s  iters:%2d  dist:%4.2f  %s ",
             fps, sim_fps,
             PRESETS[s->current_preset].name,
             THEMES[s->current_theme].name,
             scene_iters(s),
             (double)s->cam_dist,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > sc->cols) slen = sc->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - slen, "%s", status);
    mvprintw(0, 0, " KIFS · FRACTAL ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Bottom row: cyan key hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  r:reset  n/N:preset  t/T:theme  "
             "i/I:iters  z/Z:zoom  arrows:orbit ");
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
