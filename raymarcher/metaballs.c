/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * metaballs.c — SDF metaballs with smooth-min blending
 *
 * DEMO: Six spheres orbit on independent Lissajous paths.  Each sphere
 *       is described by a SIGNED DISTANCE FUNCTION (SDF) — a tiny
 *       function that returns "how far am I from the surface, with
 *       sign showing inside/outside?".  The whole scene is the
 *       SMOOTH MIN of the six per-sphere SDFs: instead of taking the
 *       hard min (which would give six disjoint balls), we use a
 *       polynomial that bumps the result slightly down where two
 *       SDFs are close to each other.  The resulting iso-surface
 *       (where the field equals 0) bulges outward into the gap
 *       between two approaching balls, producing the iconic "wax
 *       neck" of a metaball blob.
 *
 *       Sphere-traced.  Phong shaded.  Optional Quílez soft shadows.
 *       Surface coloured by mean curvature: high-curvature sphere
 *       tips get the warmest hue in the active theme; flat merged
 *       saddles get the coolest.
 *
 *       Vary k live: small k (press j) → balls barely touch;
 *                    large k (press k) → fully melted single blob.
 *
 *       Themes (cycle with c):
 *         CLASSIC    deep-blue → orange (the canonical look)
 *         OCEAN      navy → bright cyan
 *         EMBER      dark-red → bright yellow
 *         NEON       magenta → green
 *
 * Study alongside:
 *   raymarcher/mandelbulb.c    — same sphere-trace + Phong skeleton,
 *                                 iterated-fractal SDF instead of a
 *                                 finite union of spheres
 *   raymarcher/kifs_fractal.c  — folding-fractal SDF; both files use
 *                                 estimate_normal / estimate_curvature
 *                                 the same way, even though their
 *                                 underlying field is wildly different
 *   raymarcher/donut.c         — a non-SDF raymarcher (parametric
 *                                 sampling + z-buffer) — read for
 *                                 contrast: SDF vs parametric
 *                                 rendering
 *
 * Section map:
 *   §1 config    — frame, canvas, raymarch, lighting, orbit table
 *   §2 clock     — monotonic timer + sleep
 *   §3 color     — theme palette + HUD/hint pairs (CLAUDE.md spec)
 *   §4 vec3      — value-type 3-D math + clampf
 *   §5 sdf       — sphere_sdf, smin, scene_sdf, normal, curvature
 *   §6 march     — sphere_trace, soft_shadow, phong, cast_ray
 *   §7 scene     — orbits, camera, canvas, render, decorate, draw
 *   §8 screen    — ncurses init / resize / HUD draw / present
 *   §9 app       — main loop, signals, key handling, cleanup
 *
 * Keys:
 *   q / Q / ESC   quit
 *   space         pause / resume orbit
 *   j / k         blend k smaller / larger    (more separate / more merged)
 *   c / C         cycle theme  (CLASSIC → OCEAN → EMBER → NEON)
 *   s / S         toggle soft shadows
 *   + / =         faster animation
 *   - / _         slower animation
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raymarcher/metaballs.c \
 *       -o metaballs -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Sphere-traced rendering of an SDF field built by
 *                  SMOOTH-MIN'ing six sphere SDFs.
 *
 *                  Per pixel:
 *                    1. ray   = camera through pixel, aspect-corrected
 *                    2. trace = Hart-1996 sphere trace of scene_sdf
 *                    3. on hit:
 *                         normal    = tetrahedron finite differences of SDF
 *                         curvature = Laplacian of SDF / 2  (≈ mean curvature)
 *                         shadow    = Quílez soft penumbra ray to light
 *                         shade     = Phong (ambient + N·L + spec) · shadow
 *                    4. (shade, curvature) → glyph + theme-band colour
 *
 *                  scene_sdf is the foldr of smooth-min over balls:
 *                    d = sphere_sdf(p, ball[0])
 *                    for i in 1..N−1:  d = smin(d, sphere_sdf(p, ball[i]), k)
 *
 *                  smin (Quílez polynomial form):
 *                    h    = max(k − |a − b|, 0) / k
 *                    smin = min(a, b) − h² · k / 4
 *                  At |a − b| > k it equals plain min (no blend); at
 *                  a = b the result is a − k/4 (max bump).
 *
 *                  Tetrahedron normal trick (Inigo Quílez):
 *                    Sample SDF at 4 vertices of a regular tetrahedron
 *                    around p, combine with sign vectors → gradient.
 *                    4 SDF evals vs 6 for central differences, no
 *                    accuracy loss for typical SDFs.
 *
 *                  Mean curvature from SDF:
 *                    For any SDF f, ∇²f at the surface = 2H, where H
 *                    is the mean curvature.  Sphere of radius r:
 *                    Laplacian = 2/r.  Saddle: ≈ 0.  We use the six-
 *                    sample Laplacian stencil and clamp to [0, 1].
 *
 * Data-structure : Stateless math.  No tables, no LUTs.  Per-frame
 *                  state lives in `Scene` (centres + radii + k_blend
 *                  + theme + speed); the inner SDF takes a `SceneSDF`
 *                  view (centres + radii + N + k_blend) so the four
 *                  hot-loop helpers (sdf, normal, curvature, shadow)
 *                  carry one pointer instead of four arguments.
 *
 *                  A half-resolution `Canvas` (2×2 terminal cells per
 *                  canvas pixel) stores `(shade, curvature)` per pixel
 *                  so the render and draw passes are decoupled.
 *
 * Rendering      : ASCII only.  Glyph from shade (Phong luma →
 *                  " .,:;+*oxOX#@"), colour pair from curvature
 *                  (theme × 8-band ramp).  Background = default bg.
 *                  Bright shades (> BOLD_SHADE_THRESHOLD) add A_BOLD
 *                  for an extra punch on highlights.
 *
 * Performance    : ~64 march steps × ~6 sphere evals + 4 normal +
 *                  6 curvature + ~16 shadow ≈ 600 SDF evaluations per
 *                  HIT pixel.  At 80×24 with the 2×2 canvas → 480
 *                  canvas pixels.  Holds 24 fps comfortably; toggle
 *                  shadows off (`s`) to roughly double the frame rate
 *                  on slow terminals.
 *
 * References     :
 *   • Quílez, I. (2013) — "Smooth Minimum"
 *     https://iquilezles.org/articles/smin/
 *     The original article on the polynomial smooth-min used here.
 *   • Quílez, I. — "Distance Functions"
 *     https://iquilezles.org/articles/distfunctions/
 *     Source of the sphere SDF, soft-shadow ray, and tetrahedron
 *     normal.
 *   • Quílez, I. — "Normals for an SDF"
 *     https://iquilezles.org/articles/normalsSDF/
 *     The 4-tap tetrahedron gradient trick.
 *   • Hart, J. C. (1996) — "Sphere Tracing: A Geometric Method for
 *     the Antialiased Ray Tracing of Implicit Surfaces," *The Visual
 *     Computer* 12(10):527–545.  The march-by-DE iteration we use.
 *   • Phong, B. T. (1975) — "Illumination for Computer Generated
 *     Pictures," *CACM* 18(6):311–317.  The lighting model.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * One function rules everything.  An SDF f(p) returns "how far is p
 * from the nearest surface, with sign showing inside vs outside?".
 * Hard-min'ing six sphere SDFs gives six disjoint balls.  REPLACING
 * that hard min with a polynomial smooth-min converts the boolean
 * union into a soft union: the field gently bulges outward wherever
 * two distance fields are within k of each other, producing the
 * characteristic "wax dripping between blobs" look.  Everything
 * else — sphere tracing, Phong shading, curvature colour — is
 * bookkeeping on top of that one swap.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture each ball as a small magnet wrapped in a thin sheet of
 * warm wax.  When two magnets get within distance k, the wax
 * stretches into a smooth neck instead of forcing the spheres to
 * clip through each other.  The k slider is "wax temperature":
 * k = 0.05 is cold, brittle, the spheres just touch; k = 4 is
 * molten, you barely see individuals.  The render is a still
 * photograph of that wax with one spotlight, tinted by how curved
 * each surface patch is.
 *
 *      ┌─────────────────────────────────────────────────────────┐
 *      │                                                         │
 *      │     light                                               │
 *      │       ☀  ↘                                              │
 *      │            ↘                                            │
 *      │            ⬤    (hard min: disjoint)                    │
 *      │              ⬤                                          │
 *      │                                                         │
 *      │            ⬤▬⬤  (smooth min: necked!)                   │
 *      │                                                         │
 *      │   per pixel:  trace → hit? → normal + curv +            │
 *      │                              shadow + shade → cell      │
 *      └─────────────────────────────────────────────────────────┘
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  Per frame:
 *    1. ANIMATE — advance scene time; recompute each ball's centre on
 *       its Lissajous orbit (BallOrbit has freq_x/y/z, phase_x/y, r).
 *    2. CAMERA  — origin (0, 0, CAM_Z), FOV tangent, aspect for the
 *       half-resolution canvas (CELL_W × CELL_H = 2 × 2 terminal
 *       cells per canvas pixel).
 *    3. LIGHT   — slowly orbiting position; one fixed direction in
 *       world space, normalised at use.
 *
 *  Per canvas pixel (px, py):
 *    4. RAY DIRECTION via pixel_ray.
 *    5. SPHERE TRACE: t = 0; loop:
 *           p = origin + t · dir
 *           d = scene_sdf(p)         (smooth-min of all spheres)
 *           if d < HIT_EPS: hit at t
 *           if t > MAX_DIST: miss
 *           t += d                    (largest safe step)
 *    6. ON HIT: cast_ray collects everything in a Hit:
 *           normal     = estimate_normal     (4-tap tetrahedron)
 *           shadow     = soft_shadow ray to light (16 steps, penumbra)
 *           shade      = Phong(N·L + (R·V)^shin) · shadow
 *           curvature  = estimate_curvature (6-tap Laplacian / 2)
 *
 *  After all pixels:
 *    7. DRAW — per canvas pixel, decorate(shade, curvature, theme):
 *           glyph   = LUMA_RAMP[shade]
 *           pair    = PAIR_THEME_BASE + theme · N + curv-band
 *           attr    = (shade > BOLD_THRESH) ? A_BOLD : 0
 *       Then emit a CELL_W × CELL_H block of that glyph + pair +
 *       attr at the canvas pixel's screen position.
 *
 * KEY FORMULAS
 * ────────────
 *   Sphere SDF:
 *     f(p, c, r) = |p − c| − r
 *
 *   Smooth min:
 *     h    = max(k − |a − b|, 0) / k
 *     smin = min(a, b) − h² · k / 4
 *
 *   Scene SDF:
 *     d = f(p, ball₀)
 *     d = smin(d, f(p, ballᵢ), k)   for i = 1..N−1
 *
 *   Sphere trace:
 *     p_{k+1} = p_k + d(p_k) · dir
 *
 *   Tetrahedron normal:
 *     k₀ = (+1, −1, −1)   k₁ = (−1, +1, −1)
 *     k₂ = (−1, −1, +1)   k₃ = (+1, +1, +1)
 *     ∇f ≈ Σ k_i · f(p + ε k_i),    N = normalise(∇f)
 *
 *   Mean curvature:
 *     ∇²f ≈ (Σ f(p ± ε ê_axis) − 6 · f(p)) / ε²
 *     mean curv = ∇²f / 2
 *
 *   Phong shading:
 *     L = AMBIENT + shadow · (KD · max(0, N · L) + KS · max(0, R · V)^SHIN)
 *     R = 2 (N · L) N − L
 *
 *   Soft shadow (Quílez):
 *     res = 1
 *     march from surface toward light; res = min(res, K · d / t)
 *     return clamp(res, 0, 1)
 *
 *   Aspect correction:
 *     phys_aspect = (rows · CELL_H · CELL_ASPECT) / (cols · CELL_W)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *   • k → 0 in smin causes division by zero.  K_MIN floor (0.05)
 *     keeps h finite; setting K_MIN to 0 will crash on smin.
 *
 *   • Concave necks between two balls have NEGATIVE Laplacian
 *     (saddle curvature).  estimate_curvature returns the raw
 *     value; the caller clamps to [0, 1] so concave regions register
 *     as the lowest band rather than wrapping.
 *
 *   • RM_HIT_EPS too small (< 1e-4) plus large k_blend makes the
 *     smooth region's distance estimate over-shoot — sphere tracing
 *     assumes a Lipschitz=1 SDF, and smin slightly violates that
 *     near the seam.  Keeping HIT_EPS at 5e-3 hides the violation
 *     in practice.
 *
 *   • dt teleport on slow first frame.  We cap dt at 200 ms so the
 *     orbit never advances by more than ~7 simulation steps in one
 *     frame, regardless of how slow the previous frame was.
 *
 *   • Canvas allocation depends on cols / CELL_W and rows / CELL_H —
 *     guard against zero (very narrow terminal) by clamping to 1.
 *
 *   • Soft shadows multiply per-hit work by SHADOW_STEPS ≈ 16.
 *     Toggling them off (`s`) on slow terminals roughly doubles the
 *     frame rate.  No visible silhouette change — only the shadow
 *     boundary softness.
 *
 *   • HUD takes 2 rows (top status + bottom hint).  Canvas is
 *     vertically centred in the remaining `rows − 2` rows.  On a
 *     terminal with rows < 6 the canvas height clamps to 1.
 *
 * HOW TO VERIFY
 * ─────────────
 *   • Press j five times to drop k below the merge threshold.  The
 *     six balls separate into distinctly-bordered spheres with no
 *     visible neck between them.
 *
 *   • Press k five times to push k high.  The six should look like
 *     one amoeba; the curvature colouring shows the merged saddle
 *     regions in the COOLEST band.
 *
 *   • Pause (space) and confirm the orbit freezes but key handling
 *     still works — proves render and tick are decoupled.
 *
 *   • Toggle soft shadows (`s`).  Shadow boundary softens visibly
 *     without changing the silhouette or curvature colours.
 *
 *   • Cycle themes (`c`).  Geometry identical, only the curvature
 *     palette changes.
 *
 *   • Resize the terminal — canvas should reallocate and re-centre.
 *
 *   • Set N_BALLS = 1 in §1 and recompile.  The result is a single
 *     sphere; k_blend has no effect (smin is only invoked between
 *     two SDFs).  Verifies the algorithm degenerates correctly.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate + UI layout. */
enum {
    TARGET_FPS    = 24,
    FPS_UPDATE_MS = 500,
    HUD_ROWS      = 2,            /* row 0 status + last row hint */
    DT_CAP_MS     = 200,          /* cap dt to avoid orbit teleport */

    N_BALLS       = 6,
    N_THEMES      = 4,
    N_CURV_BANDS  = 8,
};

/* §1.2 colour-pair IDs.
 *
 *   PAIR_HUD                       yellow + bold (status row)
 *   PAIR_HINT                      cyan   + bold (key-hint row)
 *   PAIR_THEME_BASE + θ·N + b      curvature band b in theme θ
 */
#define PAIR_HUD          1
#define PAIR_HINT         2
#define PAIR_THEME_BASE   3        /* +0..+(N_THEMES * N_CURV_BANDS − 1) */
#define PAIR_FOR(theme, band)   (PAIR_THEME_BASE + (theme) * N_CURV_BANDS + (band))

/* §1.3 time helpers. */
#define NS_PER_SEC      1000000000LL
#define NS_PER_MS          1000000LL

/* §1.4 canvas — half-resolution block rendering.
 *
 * Each canvas pixel is drawn as a CELL_W × CELL_H block of terminal
 * cells.  2 × 2 quarters the per-frame ray count, keeping the frame
 * rate comfortable despite the heavier SDF math.  CELL_ASPECT is
 * the physical height-to-width ratio of one terminal character cell;
 * multiplied into the vertical ray direction so circles render round.
 */
#define CELL_W        2
#define CELL_H        2
#define CELL_ASPECT   2.0f

/* §1.5 sphere-trace tunables.
 *
 * RM_MAX_STEPS  hard cap on march steps per ray.
 * RM_HIT_EPS    "we're touching the surface" threshold.
 * RM_MAX_DIST   ray-length budget; past this we declare miss.
 */
#define RM_MAX_STEPS    64
#define RM_HIT_EPS      0.005f
#define RM_MAX_DIST     12.0f

/* §1.6 camera. */
#define CAM_Z           5.0f       /* camera at (0, 0, CAM_Z), looking −z */
#define FOV_HALF_TAN    0.55f      /* tan(FOV / 2) — vertical & horizontal */

/* §1.7 smooth-min blend k.
 *
 *   K_MIN  → balls stay separate (almost-hard min)
 *   K_MAX  → balls merge into one blob
 *   K_MIN  must be > 0 to avoid division by zero in smin
 */
#define K_DEFAULT       0.8f
#define K_MIN           0.05f
#define K_MAX           4.0f
#define K_STEP          1.35f      /* multiplier per j / k keypress */

/* §1.8 animation speed (multiplier on dt that drives the orbit). */
#define SPD_DEFAULT     0.35f
#define SPD_MIN         0.02f
#define SPD_MAX         3.0f
#define SPD_STEP        1.35f

/* §1.9 Phong shading coefficients.
 *
 *   KA      ambient            — base lit even where N·L = 0
 *   KD      diffuse            — Lambert N·L term
 *   KS      specular           — (R·V)^SHIN term
 *   SHIN    specular sharpness
 *   BOLD_SHADE_THRESHOLD       — A_BOLD added when shade exceeds this
 */
#define KA                       0.08f
#define KD                       0.75f
#define KS                       0.45f
#define SHIN                     32.0f
#define BOLD_SHADE_THRESHOLD     0.72f

/* §1.10 soft shadow.
 *
 *   SHADOW_STEPS        max march steps for the shadow ray
 *   SHADOW_K            penumbra hardness — bigger = sharper edge
 *   SHADOW_BIAS         offset along normal at march start (avoids
 *                       self-shadow due to numerical noise)
 *   SHADOW_NEAR         t-min for the shadow march (skip the bias zone)
 */
#define SHADOW_STEPS    16
#define SHADOW_K        8.0f
#define SHADOW_BIAS     0.01f
#define SHADOW_NEAR     0.02f

/* §1.11 normal + curvature estimation.
 *
 * ε for the gradient stencil should be small enough to capture local
 * geometry but large enough that DE quantisation noise doesn't
 * dominate.  CURV_SCALE maps the raw Laplacian (which for our
 * default sphere radii peaks near 4) into [0, 1] for the colour ramp.
 */
#define NORMAL_EPS      0.004f
#define CURV_EPS        0.06f
#define CURV_SCALE      0.25f

/* §1.12 ball orbits — Lissajous frequencies + phase offsets + radii.
 *
 * Each ball traces:
 *     x = ORBIT_RX · sin(freq_x · t + phase_x)
 *     y = ORBIT_RY · sin(freq_y · t + phase_y)
 *     z = ORBIT_RZ · cos(freq_z · t)
 *
 * Different (freq_x, freq_y, freq_z) ratios produce non-repeating
 * paths.  Phase offsets spread the balls evenly at t = 0.  Radii vary
 * slightly so the balls look distinct when separated.
 */
typedef struct {
    float freq_x, freq_y, freq_z;
    float phase_x, phase_y;
    float radius;
} BallOrbit;

#define ORBIT_RX  1.35f
#define ORBIT_RY  0.75f
#define ORBIT_RZ  0.40f

static const BallOrbit ORBITS[N_BALLS] = {
    /*  fx     fy     fz     phx     phy     r     */
    {  1.0f,  2.0f,  1.5f,  0.000f, 0.785f,  0.60f },
    {  2.0f,  1.0f,  3.0f,  1.047f, 0.000f,  0.55f },
    {  1.5f,  3.0f,  1.0f,  2.094f, 1.571f,  0.50f },
    {  3.0f,  1.0f,  2.0f,  3.141f, 0.524f,  0.58f },
    {  2.5f,  1.5f,  1.0f,  4.189f, 3.927f,  0.45f },
    {  1.0f,  2.5f,  2.0f,  5.236f, 0.262f,  0.52f },
};

/* §1.13 themes — N_THEMES × N_CURV_BANDS.
 *
 * Each theme is an 8-band ramp from low curvature (slot 0, cool/dim)
 * to high curvature (slot 7, warm/bright).  Per CLAUDE.md every
 * entry sits in the bright half of the 256-cube.
 */
typedef struct {
    const char *name;
    short       bands[N_CURV_BANDS];
} Theme;

static const Theme THEMES[N_THEMES] = {
    /* CLASSIC: deep blue (low) → orange (high) — the canonical look. */
    { "CLASSIC ", {  27,  33,  38,  44, 130, 166, 202, 214 } },
    /* OCEAN: navy → bright cyan. */
    { "OCEAN   ", {  25,  26,  27,  31,  38,  45,  51, 159 } },
    /* EMBER: dark red → bright yellow. */
    { "EMBER   ", {  88, 124, 160, 196, 202, 208, 214, 228 } },
    /* NEON: magenta → green. */
    { "NEON    ", { 201, 165, 129,  93,  57,  82, 118, 155 } },
};

/* §1.14 luminance ramp — dark → bright.
 *
 * Slot 0 is space, so a hit pixel with shade ≈ 0 (back-facing or
 * deep shadow) leaves the cell empty.  The ambient term (KA) keeps
 * any visible surface above slot 0 in practice.
 */
static const char LUMA_RAMP[] = " .,:;+*oxOX#@";
#define LUMA_RAMP_LEN   ((int)(sizeof LUMA_RAMP - 1))

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

/* ── §3 color — theme palette + HUD/hint pairs ───────────────────────── */

static void color_init(void)
{
    start_color();
    use_default_colors();

    if (COLORS >= 256) {
        init_pair(PAIR_HUD,  226, -1);     /* bright yellow */
        init_pair(PAIR_HINT,  51, -1);     /* bright cyan   */
        for (int t = 0; t < N_THEMES; t++)
            for (int b = 0; b < N_CURV_BANDS; b++)
                init_pair((short)PAIR_FOR(t, b), THEMES[t].bands[b], -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
        /* 8-colour fallback: low bands cyan, high bands yellow/white. */
        for (int t = 0; t < N_THEMES; t++)
            for (int b = 0; b < N_CURV_BANDS; b++)
                init_pair((short)PAIR_FOR(t, b),
                          (b < 3) ? COLOR_BLUE
                        : (b < 5) ? COLOR_CYAN
                        : (b < 7) ? COLOR_YELLOW
                        :           COLOR_WHITE,
                          -1);
    }
}

/* ── §4 vec3 — value-type 3-D math ───────────────────────────────────── */

typedef struct { float x, y, z; } Vec3;

static inline Vec3  v3   (float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3  v3add(Vec3 a, Vec3 b)            { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline Vec3  v3sub(Vec3 a, Vec3 b)            { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline Vec3  v3mul(Vec3 a, float s)           { return v3(a.x*s,   a.y*s,   a.z*s); }
static inline float v3dot(Vec3 a, Vec3 b)            { return a.x*b.x + a.y*b.y + a.z*b.z; }
static inline float v3len(Vec3 a)                    { return sqrtf(v3dot(a, a)); }
static inline Vec3  v3norm(Vec3 a)
{
    float L = v3len(a);
    return (L > 1e-12f) ? v3mul(a, 1.0f / L) : v3(0, 0, 1);
}

static inline float clampf(float x, float lo, float hi)
{
    return x < lo ? lo : (x > hi ? hi : x);
}

/* ── §5 sdf — sphere, smooth-min, scene, normal, curvature ───────────── *
 *
 * Every helper in this section is a function from "a point in space"
 * to "some scalar quantity of that point".  Each one teaches one
 * piece of the SDF mental model.
 */

/*
 * sphere_sdf — the canonical Signed Distance Function.
 *
 *     f(p, c, r) = |p − c| − r
 *
 * Returns:
 *   • POSITIVE  if p is OUTSIDE the sphere   (= true distance to surface)
 *   • ZERO      if p is exactly on the surface
 *   • NEGATIVE  if p is INSIDE  the sphere   (= negated radial distance)
 *
 * Verify:
 *   p = c           → f = −r          (deepest interior)
 *   p = c + r·n̂     → f =  0          (surface)
 *   p far from c    → f ≈ |p − c|     (true Euclidean distance)
 *
 * Why this matters: this single function is enough to:
 *   1. RAYMARCH.  Step exactly f(p) along a ray; never overshoot.
 *   2. INSIDE/OUTSIDE TEST.  Just check the sign.
 *   3. UNION two spheres:        min(f₁, f₂)         (boolean OR)
 *   4. INTERSECT:                max(f₁, f₂)         (boolean AND)
 *   5. DIFFERENCE A − B:         max(f_A, −f_B)      (boolean SUB)
 *   6. SOFT BLEND:               smin(f₁, f₂, k)     (this demo)
 */
static inline float sphere_sdf(Vec3 p, Vec3 c, float r)
{
    return v3len(v3sub(p, c)) - r;
}

/*
 * smin — polynomial smooth minimum.  Quílez (2013).
 *
 * Goal: a function that behaves like min(a, b) when a and b are FAR
 * apart, but smoothly transitions to a slightly-smaller value when
 * they are CLOSE (within k of each other).
 *
 * Why?  In the SDF world, "smaller distance" = "closer to surface".
 * When two surfaces approach each other, smin returns a value SMALLER
 * than min — so we report being CLOSER than either surface alone.
 * The iso-surface (where the field equals zero) then bulges OUTWARD
 * into the gap between the two objects.  That's the metaball "neck".
 *
 * Polynomial form (mathematically equivalent to Quílez's mix-style):
 *
 *     h    = max(k − |a − b|, 0) / k         blend amount, ∈ [0, 1]
 *     smin = min(a, b) − h² · k / 4          subtract a "bump"
 *
 * Behaviour:
 *     |a − b| > k     → h = 0  → returns plain min(a, b)
 *     a = b           → h = 1  → returns a − k/4   (max bump)
 *     k = 0           → DIVISION BY ZERO          (caller clamps k ≥ K_MIN)
 *
 * The bump magnitude is bounded by k/4: even at peak blending, the
 * result drops by at most a quarter of the blend radius.  The SDF
 * remains well-behaved (Lipschitz ≈ 1) for the sphere trace.  This
 * is why we cap RM_HIT_EPS away from zero (see EDGE CASES).
 *
 * The function is C¹-continuous: both smin and its derivative are
 * smooth across the transition at |a − b| = k.
 */
static float smin(float a, float b, float k)
{
    float h = fmaxf(k - fabsf(a - b), 0.0f) / k;
    return fminf(a, b) - h * h * k * 0.25f;
}

/*
 * SceneSDF — read-only "view" of the scene, passed to every helper
 * in §5 / §6 by const pointer.  Avoids passing four parameters
 * (centres, radii, n, k_blend) to each call, and lets the inner
 * loop see all four hot-path values from one cache line.
 */
typedef struct {
    const Vec3  *centres;
    const float *radii;
    int          n;
    float        k_blend;
} SceneSDF;

/*
 * scene_sdf — smooth-min reduction over all metaballs.
 *
 *     d = sphere_sdf(p, ball₀)
 *     for i in 1..N−1:
 *         d = smin(d, sphere_sdf(p, ballᵢ), k)
 *     return d
 *
 * The order of folding doesn't affect the result for well-separated
 * balls; with finite k the order produces tiny visual differences at
 * three-way contact points, invisible at terminal resolution.
 */
static float scene_sdf(Vec3 p, const SceneSDF *s)
{
    float d = sphere_sdf(p, s->centres[0], s->radii[0]);
    for (int i = 1; i < s->n; i++)
        d = smin(d, sphere_sdf(p, s->centres[i], s->radii[i]), s->k_blend);
    return d;
}

/*
 * estimate_normal — surface normal at p via the TETRAHEDRON trick
 * (Inigo Quílez, normalsSDF.html).
 *
 * Standard central differences need 6 SDF evaluations (±ε along each
 * axis).  We can do it with 4 by sampling at the vertices of a
 * regular tetrahedron and combining with the vertex sign vectors:
 *
 *     k₀ = (+1, −1, −1)        k₁ = (−1, +1, −1)
 *     k₂ = (−1, −1, +1)        k₃ = (+1, +1, +1)
 *
 *     ∇f ≈ Σ k_i · f(p + ε · k_i)
 *
 * Which expands componentwise to:
 *     ∂f/∂x ≈  +f₀ − f₁ − f₂ + f₃
 *     ∂f/∂y ≈  −f₀ + f₁ − f₂ + f₃
 *     ∂f/∂z ≈  −f₀ − f₁ + f₂ + f₃
 *
 * 4 evals vs 6 with no accuracy loss for typical SDFs.  Saves
 * ~33 % of the per-hit normal cost — a real speedup since this
 * runs once per HIT pixel.
 */
static Vec3 estimate_normal(Vec3 p, const SceneSDF *s)
{
    float e  = NORMAL_EPS;
    float f0 = scene_sdf(v3(p.x + e, p.y - e, p.z - e), s);
    float f1 = scene_sdf(v3(p.x - e, p.y + e, p.z - e), s);
    float f2 = scene_sdf(v3(p.x - e, p.y - e, p.z + e), s);
    float f3 = scene_sdf(v3(p.x + e, p.y + e, p.z + e), s);
    return v3norm(v3( f0 - f1 - f2 + f3,
                     -f0 + f1 - f2 + f3,
                     -f0 - f1 + f2 + f3));
}

/*
 * estimate_curvature — mean curvature ≈ ½ · Laplacian(SDF).
 *
 * For ANY SDF f, the Laplacian ∇²f at the surface equals 2H, where
 * H is the mean curvature.  Why?  An SDF locally looks like a signed
 * radial-distance field, and the Laplacian recovers the trace of the
 * curvature tensor.
 *
 * Sphere of radius r:
 *     f = √(x² + y² + z²) − r
 *     ∂²f/∂x² = (y² + z²) / r³,  similarly for y, z
 *     ∇²f = (2x² + 2y² + 2z²) / r³ = 2/r       (at the surface)
 *
 * Saddle / flat merged region:    Laplacian ≈ 0
 * Concave neck:                   Laplacian < 0  (caller clamps to 0)
 *
 * Six-sample Laplacian stencil:
 *     ∇²f(p) ≈ (Σ over 6 axis-aligned neighbours of f − 6·f(p)) / ε²
 *
 * We use the value to colour the surface by curvature: peaks of
 * individual spheres (high curvature → warm hue) versus merged
 * saddle regions (low curvature → cool hue).
 */
static float estimate_curvature(Vec3 p, const SceneSDF *s)
{
    float e  = CURV_EPS;
    float c0 = scene_sdf(p, s);
    float lap =
        scene_sdf(v3(p.x + e, p.y,     p.z    ), s) +
        scene_sdf(v3(p.x - e, p.y,     p.z    ), s) +
        scene_sdf(v3(p.x,     p.y + e, p.z    ), s) +
        scene_sdf(v3(p.x,     p.y - e, p.z    ), s) +
        scene_sdf(v3(p.x,     p.y,     p.z + e), s) +
        scene_sdf(v3(p.x,     p.y,     p.z - e), s)
        - 6.0f * c0;
    return lap / (e * e);
}

/* ── §6 march — sphere trace + soft shadow + Phong + cast_ray ────────── */

/*
 * sphere_trace — Hart 1996.  March along a ray, taking a step exactly
 * equal to the current SDF.  Distance estimates are LOWER bounds on
 * true distance, so we never overshoot the surface.
 *
 *     t = 0
 *     for step = 1..MAX_STEPS:
 *         p = origin + t · dir
 *         d = scene_sdf(p)
 *         if d < HIT_EPS: hit, return t
 *         if t > MAX_DIST: miss, return -1
 *         t += d
 *
 * Returns the ray-parameter t at the hit, or −1.0 on miss.
 */
static float sphere_trace(Vec3 origin, Vec3 dir, const SceneSDF *s)
{
    float t = 0.0f;
    for (int i = 0; i < RM_MAX_STEPS; i++) {
        Vec3  p = v3add(origin, v3mul(dir, t));
        float d = scene_sdf(p, s);
        if (d < RM_HIT_EPS) return t;
        if (t > RM_MAX_DIST) break;
        t += d;
    }
    return -1.0f;
}

/*
 * soft_shadow — Quílez penumbra.  March from a hit point toward the
 * light, tracking the closest SDF approach normalised by travel
 * distance:
 *
 *     res = 1
 *     while not at light:
 *         h = scene_sdf(here)
 *         if h < tiny:  return 0  (fully blocked)
 *         res = min(res, K · h / t)
 *         step h along ray
 *     return clamp(res, 0, 1)
 *
 * The min(K·h/t) tracking gives a SOFT penumbra: the closer the
 * shadow ray comes to a surface (small h at distance t), the more
 * the light is dimmed proportionally.  K controls hardness — bigger
 * K = sharper shadow edge.
 *
 * Returns 1 if the path to the light is clear, 0 if fully blocked,
 * and a soft factor in [0, 1] for partial occlusion.
 */
static float soft_shadow(Vec3 origin, Vec3 to_light_dir,
                          float t_min, float t_max, const SceneSDF *s)
{
    float res = 1.0f;
    float t   = t_min;
    for (int i = 0; i < SHADOW_STEPS && t < t_max; i++) {
        float h = scene_sdf(v3add(origin, v3mul(to_light_dir, t)), s);
        if (h < RM_HIT_EPS * 0.5f) return 0.0f;
        res = fminf(res, SHADOW_K * h / t);
        t  += fmaxf(h, RM_HIT_EPS);
    }
    return clampf(res, 0.0f, 1.0f);
}

/*
 * phong — Phong shading at a hit point.
 *
 *     L_dir = normalise(light_pos − hit)        light direction
 *     V_dir = normalise(cam_pos   − hit)        view direction
 *     ndl   = max(0, N · L_dir)                 Lambert
 *     R_dir = 2 · (N · L_dir) · N − L_dir       reflection
 *     spec  = max(0, R · V)^SHIN
 *     I     = KA + shadow · (KD · ndl + KS · spec)
 *
 * The shadow factor [0, 1] multiplies the direct (diffuse + specular)
 * component but not the ambient — so cells in deep shadow stay at
 * KA, never pure black.
 */
static float phong(Vec3 hit, Vec3 N, Vec3 cam, Vec3 light, float shadow)
{
    Vec3  L   = v3norm(v3sub(light, hit));
    Vec3  V   = v3norm(v3sub(cam,   hit));
    float ndl = fmaxf(0.0f, v3dot(N, L));
    Vec3  R   = v3sub(v3mul(N, 2.0f * ndl), L);
    float sp  = powf(fmaxf(0.0f, v3dot(R, V)), SHIN);
    return clampf(KA + shadow * (KD * ndl + KS * sp), 0.0f, 1.0f);
}

/*
 * Hit — full per-pixel result of one ray cast.  `hit = false` when
 * the ray missed; the other fields are then irrelevant.  The caller
 * passes (shade, curvature) to the canvas storage and discards the
 * rest.
 */
typedef struct {
    bool  hit;
    Vec3  p;
    Vec3  normal;
    float shade;
    float curvature;
} Hit;

/*
 * cast_ray — one full per-pixel pipeline.  Pseudocode:
 *
 *     t = sphere_trace(origin, dir, scene)
 *     if t < 0:        return Hit{ hit = false }
 *
 *     hit_p     = origin + t · dir
 *     normal    = estimate_normal(hit_p)
 *     shadow    = soft_shadow_ray(hit_p, light)   if soft_shadows else 1
 *     shade     = phong(hit_p, normal, origin, light, shadow)
 *     raw_curv  = estimate_curvature(hit_p)
 *     curvature = clamp(raw_curv * CURV_SCALE, 0, 1)
 *     return Hit{ ... }
 *
 * Single function call per pixel makes the per-frame DE-evaluation
 * budget auditable in one place.
 */
static Hit cast_ray(Vec3 origin, Vec3 dir, const SceneSDF *s,
                     Vec3 light, bool soft_shadows)
{
    Hit h = { false, {0, 0, 0}, {0, 0, 1}, 0.0f, 0.0f };

    float t = sphere_trace(origin, dir, s);
    if (t < 0.0f) return h;

    h.hit    = true;
    h.p      = v3add(origin, v3mul(dir, t));
    h.normal = estimate_normal(h.p, s);

    /* Shadow factor — soft penumbra ray, or 1.0 if shadows disabled. */
    float shadow = 1.0f;
    if (soft_shadows) {
        Vec3  L_dir   = v3norm(v3sub(light, h.p));
        Vec3  shad_o  = v3add(h.p, v3mul(h.normal, SHADOW_BIAS));
        float to_lite = v3len(v3sub(light, h.p));
        shadow = soft_shadow(shad_o, L_dir, SHADOW_NEAR, to_lite, s);
    }

    h.shade     = phong(h.p, h.normal, origin, light, shadow);
    float raw_c = estimate_curvature(h.p, s);
    h.curvature = clampf(raw_c * CURV_SCALE, 0.0f, 1.0f);
    return h;
}

/* ── §7 scene — orbits, camera, canvas, render, decorate, draw ───────── */

/* §7.1 ball orbits + scene state. */

typedef struct {
    Vec3   centres[N_BALLS];
    float  radii  [N_BALLS];
    float  time;                 /* scene time (seconds × speed) */
    float  k_blend;
    float  speed;
    int    theme;
    bool   paused;
    bool   soft_shadows;
} Scene;

/*
 * ball_position_at — Lissajous orbit point for ball i at scene time t.
 *
 *     x = ORBIT_RX · sin(freq_x · t + phase_x)
 *     y = ORBIT_RY · sin(freq_y · t + phase_y)
 *     z = ORBIT_RZ · cos(freq_z · t)
 */
static Vec3 ball_position_at(int i, float t)
{
    const BallOrbit *o = &ORBITS[i];
    return v3(ORBIT_RX * sinf(o->freq_x * t + o->phase_x),
              ORBIT_RY * sinf(o->freq_y * t + o->phase_y),
              ORBIT_RZ * cosf(o->freq_z * t));
}

/* light_position_at — slowly orbiting light fixture so the highlights
 * sweep across the surface as time passes. */
static Vec3 light_position_at(float t)
{
    return v3(cosf(t * 0.6f) * 4.0f,
              sinf(t * 0.6f * 0.45f) * 2.0f + 2.5f,
              3.5f);
}

static void scene_update_balls(Scene *s)
{
    for (int i = 0; i < N_BALLS; i++) {
        s->centres[i] = ball_position_at(i, s->time);
        s->radii  [i] = ORBITS[i].radius;
    }
}

static void scene_init(Scene *s)
{
    memset(s, 0, sizeof *s);
    s->k_blend      = K_DEFAULT;
    s->speed        = SPD_DEFAULT;
    s->theme        = 0;
    s->paused       = false;
    s->soft_shadows = true;
    scene_update_balls(s);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;
    s->time += dt * s->speed;
    scene_update_balls(s);
}

static SceneSDF scene_sdf_view(const Scene *s)
{
    return (SceneSDF){
        .centres = s->centres,
        .radii   = s->radii,
        .n       = N_BALLS,
        .k_blend = s->k_blend,
    };
}

/* §7.2 camera + ray gen for the half-resolution canvas. */

typedef struct {
    Vec3  origin;
    float fov_t;
    float phys_aspect;
} Camera;

/*
 * camera_for_canvas — fixed camera at (0, 0, CAM_Z) looking along −z.
 *
 * `phys_aspect` corrects for the fact that terminal cells are CELL_H
 * pixels tall and CELL_W pixels wide, plus an extra CELL_ASPECT
 * factor for cells being physically taller than wide.  Result:
 * circular metaballs render as circles, not vertically-stretched
 * ellipses.
 */
static Camera camera_for_canvas(int cw, int ch)
{
    return (Camera){
        .origin      = v3(0.0f, 0.0f, CAM_Z),
        .fov_t       = FOV_HALF_TAN,
        .phys_aspect = ((float)ch * (float)CELL_H * CELL_ASPECT)
                     / ((float)cw * (float)CELL_W),
    };
}

/*
 * pixel_ray — direction from camera through canvas pixel (px, py).
 *
 *     u   ∈ [−1, +1] along screen x
 *     v   ∈ [−1, +1] along screen y (top of screen → +v)
 *     dir = normalise(u · fov_t,  v · fov_t · aspect,  −1)
 */
static Vec3 pixel_ray(int px, int py, int cw, int ch, const Camera *c)
{
    float u =  ((float)px + 0.5f) / (float)cw * 2.0f - 1.0f;
    float v = -((float)py + 0.5f) / (float)ch * 2.0f + 1.0f;
    return v3norm(v3(u * c->fov_t,
                     v * c->fov_t * c->phys_aspect,
                     -1.0f));
}

/* §7.3 canvas — half-resolution storage + render orchestrator. */

typedef struct {
    int    w, h;
    float *shades;        /* w*h; < 0 = miss */
    float *curvs;         /* w*h */
} Canvas;

static void canvas_alloc(Canvas *c, int term_cols, int term_rows)
{
    int draw_rows = term_rows - HUD_ROWS;
    if (draw_rows < 1) draw_rows = 1;

    c->w = term_cols / CELL_W;
    c->h = draw_rows / CELL_H;
    if (c->w < 1) c->w = 1;
    if (c->h < 1) c->h = 1;

    size_t n  = (size_t)c->w * (size_t)c->h;
    c->shades = malloc(n * sizeof(float));
    c->curvs  = malloc(n * sizeof(float));
}

static void canvas_free(Canvas *c)
{
    free(c->shades); c->shades = NULL;
    free(c->curvs);  c->curvs  = NULL;
    c->w = c->h = 0;
}

/*
 * canvas_render — fill `shades` and `curvs` for every canvas pixel.
 *
 *     cam   = camera_for_canvas(canvas.w, canvas.h)
 *     for py, px:
 *         ray = pixel_ray(px, py, ..., cam)
 *         hit = cast_ray(cam.origin, ray, scene, light, shadows)
 *         shades[i] = hit ? hit.shade     : -1
 *         curvs [i] = hit ? hit.curvature :  0
 *
 * The camera basis is computed ONCE per frame, not per pixel.
 */
static void canvas_render(Canvas *c, const Scene *s)
{
    Camera   cam   = camera_for_canvas(c->w, c->h);
    Vec3     light = light_position_at(s->time);
    SceneSDF view  = scene_sdf_view(s);

    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            Vec3 ray = pixel_ray(px, py, c->w, c->h, &cam);
            Hit  h   = cast_ray(cam.origin, ray, &view,
                                light, s->soft_shadows);
            int  idx = py * c->w + px;
            c->shades[idx] = h.hit ? h.shade     : -1.0f;
            c->curvs [idx] = h.hit ? h.curvature :  0.0f;
        }
    }
}

/* §7.4 decorate + emit (canvas → terminal cells). */

/*
 * Cell — the (glyph, colour pair, attribute) decoration of one
 * canvas pixel.  A miss is encoded as `pair = −1` (the painter
 * skips silently, leaving the cell empty).
 */
typedef struct { char glyph; int pair; attr_t attr; } Cell;

/*
 * shade_to_glyph — map shade ∈ [0, 1] to a glyph from LUMA_RAMP.
 * Out-of-range inputs clamp gracefully.
 */
static char shade_to_glyph(float shade)
{
    int idx = (int)(shade * (float)(LUMA_RAMP_LEN - 1) + 0.5f);
    if (idx < 0)               idx = 0;
    if (idx >= LUMA_RAMP_LEN)  idx = LUMA_RAMP_LEN - 1;
    return LUMA_RAMP[idx];
}

/*
 * curvature_to_band — map curvature ∈ [0, 1] to a colour band 0..7.
 */
static int curvature_to_band(float curv)
{
    int b = (int)(curv * (float)(N_CURV_BANDS - 1) + 0.5f);
    if (b < 0)               b = 0;
    if (b >= N_CURV_BANDS)   b = N_CURV_BANDS - 1;
    return b;
}

/*
 * decorate — turn a (shade, curvature, theme) triple into a Cell.
 *
 *   miss              → Cell{ ' ', -1, 0 }   (painter skips)
 *   shade ∈ [0, 1]    → glyph + theme-band pair
 *   shade > BOLD_THR  → A_BOLD added for highlight punch
 */
static Cell decorate(float shade, float curv, int theme)
{
    if (shade < 0.0f) return (Cell){ ' ', -1, 0 };
    int band = curvature_to_band(curv);
    return (Cell){
        .glyph = shade_to_glyph(shade),
        .pair  = PAIR_FOR(theme, band),
        .attr  = (shade > BOLD_SHADE_THRESHOLD) ? A_BOLD : 0,
    };
}

/*
 * emit_block — paint a CELL_W × CELL_H block at (tx0, ty0) with one
 * cell's glyph / pair / attr.  Skips silently for miss cells (pair < 0)
 * and for blocks that fall partly off-screen.
 */
static void emit_block(int tx0, int ty0, Cell c, int term_cols, int term_rows)
{
    if (c.pair < 0) return;

    attr_t a = COLOR_PAIR(c.pair) | c.attr;
    attron(a);
    for (int by = 0; by < CELL_H; by++) {
        int ty = ty0 + by;
        if (ty < 0 || ty >= term_rows) continue;
        for (int bx = 0; bx < CELL_W; bx++) {
            int tx = tx0 + bx;
            if (tx < 0 || tx >= term_cols) continue;
            mvaddch(ty, tx, (chtype)(unsigned char)c.glyph);
        }
    }
    attroff(a);
}

/*
 * canvas_draw — walk every canvas pixel, decorate it, emit the block
 * at the right screen offset.
 *
 *   off_x = (term_cols − canvas.w · CELL_W) / 2          horizontal centre
 *   off_y = 1 + (term_rows − HUD_ROWS − canvas.h · CELL_H) / 2
 *
 * The +1 in off_y leaves row 0 free for the top HUD; the
 * −HUD_ROWS reserves row rows−1 for the hint strip.  Canvas is
 * vertically centred in the remaining `rows − HUD_ROWS` rows.
 */
static void canvas_draw(const Canvas *c, int term_cols, int term_rows, int theme)
{
    int off_x = (term_cols - c->w * CELL_W) / 2;
    int off_y = 1 + (term_rows - HUD_ROWS - c->h * CELL_H) / 2;
    if (off_x < 0) off_x = 0;
    if (off_y < 1) off_y = 1;

    for (int py = 0; py < c->h; py++) {
        for (int px = 0; px < c->w; px++) {
            int   idx = py * c->w + px;
            Cell  cell = decorate(c->shades[idx], c->curvs[idx], theme);
            int   tx0  = off_x + px * CELL_W;
            int   ty0  = off_y + py * CELL_H;
            emit_block(tx0, ty0, cell, term_cols, term_rows);
        }
    }
}

/* ── §8 screen — ncurses init / resize / HUD / present ───────────────── */

typedef struct {
    int    cols, rows;
    Canvas canvas;
} Screen;

static void screen_init(Screen *sc)
{
    initscr();
    noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, sc->rows, sc->cols);
    canvas_alloc(&sc->canvas, sc->cols, sc->rows);
}

static void screen_free(Screen *sc)
{
    canvas_free(&sc->canvas);
    endwin();
}

static void screen_resize(Screen *sc)
{
    endwin(); refresh();
    getmaxyx(stdscr, sc->rows, sc->cols);
    canvas_free (&sc->canvas);
    canvas_alloc(&sc->canvas, sc->cols, sc->rows);
}

/*
 * hud_draw — CLAUDE.md HUD spec.
 *   row 0          PAIR_HUD  (yellow + bold) — title left, status right
 *   row rows-1     PAIR_HINT (cyan   + bold) — key hint
 */
static void hud_draw(const Screen *sc, const Scene *s, double fps)
{
    char status[140];
    snprintf(status, sizeof status,
             " %5.1f fps  k=%4.2f  spd=%4.2f  shadow=%-3s  theme=%s  %s ",
             fps, (double)s->k_blend, (double)s->speed,
             s->soft_shadows ? "on" : "off",
             THEMES[s->theme].name,
             s->paused ? "PAUSED" : "running");
    int slen = (int)strlen(status); if (slen > sc->cols) slen = sc->cols;

    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, sc->cols - slen, "%s", status);
    mvprintw(0, 0, " METABALLS ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " q:quit  spc:pause  j/k:blend  s:shadow  c:theme  +/-:speed ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps)
{
    erase();
    canvas_render(&sc->canvas, s);
    canvas_draw  (&sc->canvas, sc->cols, sc->rows, s->theme);
    hud_draw     (sc, s, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ── §9 app — main loop, signals, key handling, cleanup ──────────────── */

typedef struct {
    Scene                 scene;
    Screen                screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup         (void)    { endwin(); }

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           s->paused = !s->paused;       break;

    case 'k':
        if (s->k_blend < K_MAX) s->k_blend *= K_STEP;
        if (s->k_blend > K_MAX) s->k_blend  = K_MAX;
        break;
    case 'j':
        if (s->k_blend > K_MIN) s->k_blend /= K_STEP;
        if (s->k_blend < K_MIN) s->k_blend  = K_MIN;
        break;

    case 's': case 'S':
        s->soft_shadows = !s->soft_shadows;
        break;

    case 'c': case 'C':
        s->theme = (s->theme + 1) % N_THEMES;
        break;

    case '+': case '=':
        if (s->speed < SPD_MAX) s->speed *= SPD_STEP;
        if (s->speed > SPD_MAX) s->speed  = SPD_MAX;
        break;
    case '-': case '_':
        if (s->speed > SPD_MIN) s->speed /= SPD_STEP;
        if (s->speed < SPD_MIN) s->speed  = SPD_MIN;
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

    screen_init(&app->screen);
    scene_init (&app->scene);

    int64_t frame_time  = clock_ns();
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app->need_resize = 0;
            screen_resize(&app->screen);
            frame_time = clock_ns();
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;
        float dt_sec = (float)dt / (float)NS_PER_SEC;

        int ch;
        while ((ch = getch()) != ERR)
            if (!app_handle_key(app, ch)) { app->running = 0; break; }

        scene_tick(&app->scene, dt_sec);

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        screen_draw(&app->screen, &app->scene, fps_display);

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);
    }

    screen_free(&app->screen);
    return 0;
}
