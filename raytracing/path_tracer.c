/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * path_tracer.c — progressive Monte Carlo path tracer · Cornell Box
 *
 * DEMO: A solid Cornell Box (red left wall, green right wall, white
 *       floor/ceiling/back, warm light overhead, gold + indigo spheres)
 *       slowly resolves from random noise into a clean image as samples
 *       accumulate. Color bleeding makes the gold sphere blush red on
 *       its left flank and the indigo sphere green on its right —
 *       global illumination you can see materialise in real time.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same scene primitives, NO bounces
 *                                     (direct phong only). Read this
 *                                     first to see what a single ray
 *                                     hit looks like before adding
 *                                     stochastic sampling.
 *   raytracing/cube_raytrace.c     — same skeleton, slab method.
 *   raytracing/capsule_raytrace.c  — same skeleton, decomposed analytic.
 *
 * Section map:
 *   §1 config       — frame rate, PT depth/RR/SPP caps, scene coords, ramp
 *   §2 clock        — monotonic timer + sleep
 *   §3 vec3         — V3 math
 *   §4 rng          — xorshift32 + per-pixel-per-frame seed
 *   §5 scene        — Cornell-box materials, quads, spheres
 *                     §5.1 Material  §5.2 Quad  §5.3 Sphere  §5.4 lookups
 *   §6 intersection — ray ∩ scene
 *                     §6.1 ray_quad   §6.2 ray_sphere   §6.3 scene_hit
 *   §7 path trace   — THE CORE
 *                     §7.1 onb (orthonormal basis around N)
 *                     §7.2 cos_sample_hemi (Malley's method)
 *                     §7.3 path_trace (iterative; RR; throughput chain)
 *   §8 framebuffer  — progressive accumulator
 *                     §8.1 accum buffer + reset
 *                     §8.2 accum_add_frame (one frame's samples)
 *                     §8.3 tone-map + accum_draw
 *   §9 screen       — color init, rgb→pair, HUD spec compliant
 *                     §9.1 color_init    §9.2 rgb_to_pair    §9.3 hud_draw
 *   §10 app         — signals, resize, main loop
 *
 * Keys:
 *   r          reset accumulator (restart convergence from sample 0)
 *   p / SPC    pause / resume sampling
 *   + / =      more samples per frame  (faster convergence, lower fps)
 *   -          fewer samples per frame (higher fps, slower convergence)
 *   q / ESC    quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/path_tracer.c \
 *       -o path_tracer -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Unidirectional Monte Carlo path tracing.
 *                  Per pixel, cast a ray. At each surface hit:
 *                    (1) if the surface is emissive, accumulate its
 *                        radiance times the path's surviving throughput
 *                        and terminate;
 *                    (2) otherwise, multiply throughput by surface
 *                        albedo, sample a random new direction from the
 *                        cosine-weighted hemisphere around the surface
 *                        normal, and recurse with the new ray.
 *                  Russian-roulette termination at depth ≥ RR_DEPTH:
 *                  survive with probability p = max(throughput); on
 *                  survival multiply throughput by 1/p (preserves
 *                  unbiasedness) — finite expected depth, no truncation.
 *
 * Data-structure : Static per-pixel accumulator
 *                    g_accum[MAX_H][MAX_W][3]   — sum of radiance over
 *                                                  ALL samples ever cast
 *                                                  at this pixel.
 *                    g_samples                  — total samples count.
 *                    Display = g_accum / g_samples, tone-mapped.
 *                  Resetting accum_reset() zeroes both. The ring is
 *                  THE convergence record: each frame just adds more
 *                  samples to the running sum.
 *
 * Rendering      : Reinhard tone map L' = L/(1+L) compresses the
 *                  open-ended HDR radiance into [0,1), then a 1/2.2
 *                  gamma encode produces sRGB-perceptual values.
 *                  Final RGB is quantised to xterm's 6×6×6 colour cube
 *                  (216 shades) and Bourke's 92-character density ramp
 *                  picks the glyph from luminance.
 *
 * Performance    : Per pixel per frame: SPP paths × MAX_DEPTH bounces
 *                  × scene_hit(O(quads) + O(spheres)) intersections.
 *                  At 8 quads, 2 spheres, MAX_DEPTH=7, SPP=2 that's
 *                  ~140 intersections/pixel/frame on average (RR ends
 *                  most paths early). Modern CPU: ~5 ns each → ~700 ns
 *                  /pixel/frame. A 200×60 terminal is 12 000 cells, so
 *                  ~8 ms shading per frame at SPP=2 — comfortable at
 *                  30 Hz. SPP=8 stretches that to ~32 ms — still ok.
 *
 * References     : Kajiya, "The Rendering Equation," SIGGRAPH '86.
 *                  Pharr, Jakob & Humphreys, "Physically Based
 *                    Rendering: From Theory to Implementation" 4e
 *                    (free online: pbr-book.org). Definitive reference;
 *                    chapters 13-14 cover Monte Carlo and path tracing.
 *                  Malley, "A Shading Method for Computer Generated
 *                    Images," MS thesis, U. Utah, 1988. (Cosine-
 *                    weighted hemisphere sampling.)
 *                  Veach, "Robust Monte Carlo Methods for Light
 *                    Transport Simulation," PhD thesis, Stanford 1997.
 *                    (Russian roulette, bidirectional methods.)
 *                  Cornell Box: Goral et al., "Modeling the
 *                    Interaction of Light Between Diffuse Surfaces,"
 *                    SIGGRAPH '84. (The original radiosity test scene
 *                    that became this folder's standard benchmark.)
 *                  Reinhard et al., "Photographic Tone Reproduction
 *                    for Digital Images," SIGGRAPH '02.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Pretend you're a photon flying BACKWARDS in time. Start at the
 * camera, fly out, hit a wall, bounce in a random direction, hit
 * another wall, bounce again, … until you eventually land on a light
 * (or give up). Multiply the colours of every wall you bounced off,
 * times the light's emission. That's ONE estimate of what colour this
 * pixel should be. Do thousands of these random walks per pixel and
 * average. The average converges to the true image.
 *
 * That's the entire algorithm. Everything else (importance sampling,
 * Russian roulette, tone mapping) is an optimisation on top.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a SOAP-OPERA RUMOUR network. Each character (surface) has
 * a "story-attenuation" factor — the fraction of any rumour they pass
 * along. The light is the only character who STARTS rumours. To find
 * out what rumour reaches your eye, you walk randomly back through
 * the gossip chain until you reach the light, multiplying the
 * attenuation factor of every character you visit:
 *
 *      eye  ←──  red wall (×0.65 in red)  ←──  white floor (×0.73)
 *           ←──  ceiling (×0.73)   ←──  LIGHT (radiance 15)
 *
 * Final colour ≈ 15 · 0.73 · 0.73 · 0.65  in red (lots gets through),
 *                15 · 0.73 · 0.73 · 0.05  in green (most absorbed).
 *
 * That's COLOUR BLEEDING — the red of the wall stains the rumour as
 * it travels back to your eye. Run a few thousand random walks per
 * pixel and average; the noise smooths into a clean image.
 *
 *      ┌──────────── ceiling (white) ──────────┐
 *      │                  light                │
 *      │                  ▼ ▼ ▼                │
 *      │               ╲╲╲│╱╱╱                 │
 *      │ red          rays scatter        green│
 *      │ wall         ──────────          wall │
 *      │                                       │
 *      │             ●           ●             │
 *      │          gold        indigo           │
 *      └──────────── floor (white) ────────────┘
 *
 *               ↑     ↑
 *         camera at (0, 0.05, -1.5), looking +Z
 *
 * The camera sits at the open front face. Each terminal cell fires
 * SPP rays per frame. Those rays bounce around inside the box,
 * eventually reaching the light at the top — and the colours of
 * everything they touched on the way mix into the pixel.
 *
 * ALGORITHM IN STEPS  (per pixel per sample)
 * ─────────────────────────────────────────
 *  1. Build a primary ray from the camera through a JITTERED sub-
 *     pixel offset. Jitter gives free anti-aliasing.
 *  2. throughput = (1, 1, 1)        ← what's left of the photon
 *     col        = (0, 0, 0)        ← accumulated radiance
 *  3. Loop up to MAX_DEPTH bounces:
 *      a. Find nearest surface hit.  If miss → break (black background).
 *      b. If surface is EMISSIVE:
 *            col += throughput · emission ;  break.
 *      c. RUSSIAN ROULETTE (depth ≥ RR_DEPTH):
 *            p = max(throughput.r, throughput.g, throughput.b)
 *            if rng > p → break (path killed)
 *            else throughput /= p   (compensate killed paths)
 *      d. Multiply throughput by surface albedo:
 *            throughput = throughput · albedo
 *      e. Pick a new direction from the cosine-weighted hemisphere
 *         around the surface normal.
 *      f. Push the ray origin by ε·N to avoid self-intersection.
 *  4. Return col.
 *  5. accum[pixel] += col;  samples += 1.
 *  6. Display = tone_map(accum / samples).
 *
 * KEY FORMULAS
 * ────────────
 *   Rendering equation (Kajiya 1986):
 *     L_o(p,ω_o) = L_e(p,ω_o) + ∫_Ω f_r(p,ω_i,ω_o) · L_i(p,ω_i) · (ω_i·n) dω_i
 *
 *   Lambertian BRDF:
 *     f_r = ρ / π       where ρ = albedo (in [0,1]).
 *
 *   Cosine-weighted hemisphere PDF:
 *     p(ω) = (n·ω) / π
 *
 *   Monte Carlo estimate per bounce (one sample):
 *     L̂ = f_r · L_i · cosθ / p(ω)
 *        = (ρ/π) · L_i · cosθ / (cosθ/π)
 *        = ρ · L_i           ← cosθ and π cancel: weight is just ρ.
 *
 *   That cancellation is the WHOLE REASON we use cosine-weighted
 *   hemisphere sampling. Naïve uniform sampling would leave a `cosθ`
 *   factor in the estimator that makes near-grazing samples noisy.
 *
 *   Malley's method for cosine-weighted samples (no rejection):
 *     r1, r2 ~ U[0,1)
 *     φ = 2π · r1
 *     local ω = (cosφ · √r2,  sinφ · √r2,  √(1−r2))
 *     world ω = onb(n) · local ω
 *
 *   Russian roulette (Veach):
 *     p = max(throughput.r, .g, .b)
 *     kill with prob (1−p); on survival throughput *= 1/p
 *     ⇒ E[contribution] is unchanged → unbiased.
 *
 *   Reinhard tone map + gamma:
 *     L'  = L / (1 + L)
 *     out = L'^(1/2.2)
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   Path:  eye → floor → red wall → light
 *   Materials:
 *     floor   albedo (0.73, 0.73, 0.73)
 *     red     albedo (0.65, 0.05, 0.05)
 *     light   emission (15, 14, 11)
 *
 *   throughput evolution:
 *     start:        (1.00, 1.00, 1.00)
 *     after floor:  (0.73, 0.73, 0.73)
 *     after wall:   (0.73·0.65, 0.73·0.05, 0.73·0.05)
 *                 = (0.4745, 0.0365, 0.0365)
 *     light hit, contribution = throughput · emission:
 *                 = (0.4745·15, 0.0365·14, 0.0365·11)
 *                 = (7.12, 0.51, 0.40)
 *
 *   That is BRIGHT RED — exactly the colour bleeding visible on the
 *   white floor near the red wall. Other paths to the same pixel may
 *   bounce different ways and contribute different colours; the
 *   AVERAGE over thousands of samples is the converged image.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SELF-INTERSECTION. After a bounce we push the ray origin by
 *    1e-4 · N. Without the push, the next intersection finds the
 *    surface we just left at t≈0 and the path stops one bounce
 *    short. The push must be along N (not into the surface).
 *  • DOUBLE-SIDED NORMAL. For a quad we pick the normal that faces
 *    the incoming ray. For a sphere we flip the outward normal if it
 *    points away from the ray. Forgetting either gives black sides
 *    or NaN bounces.
 *  • RR PROBABILITY. p = max channel, NOT mean — we want to keep
 *    paths that still carry SIGNIFICANT energy in any single channel.
 *    Mean would over-kill paths whose energy is concentrated in one
 *    band (deep red, etc).
 *  • FIRST RR DEPTH. Killing too early (depth=0,1) raises variance
 *    sharply because every path has roughly equal expected
 *    contribution. RR_DEPTH = 3 is the standard "hold off until
 *    bounces are getting dim" choice.
 *  • CONVERGENCE = √-LAW. Variance of an N-sample average scales
 *    as 1/N, so noise ∝ 1/√N. To halve the visible noise you must
 *    QUADRUPLE the samples. ACCUM_CAP = 8192 is plenty for visual
 *    cleanliness; rendering past that just burns CPU.
 *  • DOMINANT LIGHT VIA BOUNCES ONLY. This implementation has NO
 *    direct-light sampling (no NEE). Light is found purely by random
 *    bounces lucking into the light's footprint. That's slow but
 *    pedagogically clean — every line of code is path tracing,
 *    nothing is "the next-event sampler". Adding NEE would cut
 *    convergence time ~5× but add another 80 lines.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Press 'r' then watch:
 *      sample 1     → almost solid noise, hint of bright spot at light
 *      sample 32    → silhouettes visible, lots of grain
 *      sample 256   → clean walls, slight grain on the spheres
 *      sample 2048  → near-converged, only subtle noise remains
 *  • COLOUR BLEED: gold sphere should have a slightly REDDISH tint
 *    on its LEFT flank (bouncing off red wall) and slightly GREENISH
 *    tint on its RIGHT (off green wall). Same for indigo sphere.
 *  • SOFT SHADOWS: the floor under each sphere should be slightly
 *    darker than the surrounding floor (less direct illumination from
 *    the area light reaches there).
 *  • CEILING NEAR LIGHT: the parts of the ceiling adjacent to the
 *    light quad should be slightly brighter than the corners (bounce
 *    light from the floor warming the ceiling indirectly).
 *  • Increasing SPP with '+' should make the image converge visibly
 *    faster but the per-frame fps should drop proportionally.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <signal.h>
#include <time.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate */
#define TARGET_FPS    30                   /* PT is compute-heavy        */
#define DT_CAP_NS     200000000LL          /* 0.2 sec — spiral-of-death  */

/* §1.2 view geometry */
#define ASPECT        0.47f                /* terminal cell W/H ratio    */
#define FOV_DEG       66.0f                /* horizontal FOV             */

/* §1.3 path tracing */
#define MAX_DEPTH     7                    /* hard depth cap per path    */
#define RR_DEPTH      3                    /* RR starts at this depth    */
#define SPP_DEFAULT   2                    /* samples per pixel per frame*/
#define SPP_MIN       1
#define SPP_MAX       8
#define ACCUM_CAP     8192                 /* auto-pause once converged  */
#define RAY_EPS       1e-4f                /* origin offset along N      */

#define MAX_W         320                  /* static accumulator width   */
#define MAX_H         100                  /* static accumulator height  */

/*
 * §1.4 Cornell box coordinate system:
 *   x ∈ [-1, 1]   left (red) → right (green)
 *   y ∈ [-1, 1]   floor → ceiling
 *   z ∈ [ 0, 2]   open front → back wall
 *   Camera at (0, 0.05, -1.5) looking toward +Z. The front (z=0) is
 *   open so the camera sees in.
 */
#define CAM_X         0.00f
#define CAM_Y         0.05f
#define CAM_Z        -1.50f

/* §1.5 character ramp — Paul Bourke 92-char density ladder. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.6 ncurses pair IDs */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube      */
#define PAIR_HUD       217                 /* yellow row 0 status        */
#define PAIR_HINT      218                 /* cyan bottom hint strip     */
#define PAIR_BAR_FILL  219                 /* progress-bar filled cells  */
#define PAIR_BAR_EMPTY 220                 /* progress-bar empty cells   */

/* ── §2 clock ────────────────────────────────────────────────────────── */

static long long clock_ns(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns)
{
    if (ns <= 0) return;
    struct timespec ts = { ns / 1000000000LL, ns % 1000000000LL };
    nanosleep(&ts, NULL);
}

/* ── §3 vec3 ─────────────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3     (float x, float y, float z) { return (V3){x,y,z}; }
static inline V3    v3add  (V3 a, V3 b)     { return v3(a.x+b.x, a.y+b.y, a.z+b.z); }
static inline V3    v3sub  (V3 a, V3 b)     { return v3(a.x-b.x, a.y-b.y, a.z-b.z); }
static inline V3    v3mul  (V3 a, V3 b)     { return v3(a.x*b.x, a.y*b.y, a.z*b.z); }
static inline V3    v3s    (float s, V3 a)  { return v3(s*a.x,   s*a.y,   s*a.z);   }
static inline float v3dot  (V3 a, V3 b)     { return a.x*b.x + a.y*b.y + a.z*b.z;   }
static inline float v3len  (V3 a)           { return sqrtf(v3dot(a, a));            }
static inline V3    v3norm (V3 a)           { float l=v3len(a); return l>1e-9f ? v3s(1.f/l,a) : v3(0,1,0); }
static inline V3    v3cross(V3 a, V3 b)
{
    return v3(a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x);
}
static inline float v3maxc (V3 a)           { return a.x>a.y ? (a.x>a.z?a.x:a.z) : (a.y>a.z?a.y:a.z); }

/* ── §4 RNG (xorshift32, decorrelated per pixel per frame) ───────────── */

typedef uint32_t Rng;

/* rng_f — uniform float in [0, 1) via xorshift32. */
static float rng_f(Rng *r)
{
    *r ^= *r << 13;
    *r ^= *r >> 17;
    *r ^= *r << 5;
    return (float)(*r >> 1) * (1.f / (float)0x7FFFFFFF);
}

/*
 * rng_seed — produce an independent seed for (px, py, frame).
 *
 * The three large primes + xorshift warm-up scramble the components
 * so adjacent pixels and adjacent frames don't share correlated
 * sequences. Without decorrelation, neighbouring pixels would receive
 * the same sample sequence and the noise would form visible streaks.
 */
static Rng rng_seed(int px, int py, int frame)
{
    uint32_t s = (uint32_t)(px * 1973 + py * 9277 + frame * 26699 + 1);
    s ^= s << 13; s ^= s >> 7; s ^= s << 17;
    return s ? s : 1u;
}

/* ── §5 scene (Cornell box) ──────────────────────────────────────────── */

/* §5.1 ── Material ──────────────────────────────────────────────────── */

/*
 * Lambertian diffuse only.
 *   albedo ∈ [0,1]³ — fraction of light reflected per channel
 *   emit            — radiance emitted (zero for non-emissive)
 *
 * With Lambertian BRDF f_r = albedo/π and cosine-weighted hemisphere
 * sampling p(ω) = cosθ/π, the Monte Carlo estimator simplifies to
 *   weight = albedo
 * (the cosθ and π factors cancel — see MENTAL MODEL → KEY FORMULAS).
 * This is why "throughput *= albedo" is the entire bounce update.
 */
typedef struct { V3 albedo; V3 emit; } Mat;

static const Mat k_mats[] = {
    /* 0  white  */ { {0.73f, 0.73f, 0.73f}, {  0,    0,    0 } },
    /* 1  red    */ { {0.65f, 0.05f, 0.05f}, {  0,    0,    0 } },
    /* 2  green  */ { {0.12f, 0.45f, 0.15f}, {  0,    0,    0 } },
    /* 3  light  */ { {  0,     0,     0  }, { 15.f, 14.f, 11.f} }, /* warm */
    /* 4  gold   */ { {0.80f, 0.58f, 0.18f}, {  0,    0,    0 } },
    /* 5  indigo */ { {0.22f, 0.28f, 0.82f}, {  0,    0,    0 } },
};

/* §5.2 ── Quad (axis-aligned rectangular plane) ───────────────────── */

/*
 * A Cornell-box wall is an axis-aligned rectangle.
 *   axis = 0 → X-plane: lo/hi are bounds in (Y, Z)
 *   axis = 1 → Y-plane: lo/hi are bounds in (X, Z)
 *   axis = 2 → Z-plane: lo/hi are bounds in (X, Y)
 *
 * Encoding the wall as "fixed coordinate + 2-axis bounds" lets the
 * intersection test be a single divide + two range checks (§6.1) —
 * far cheaper than a triangle.
 */
typedef struct {
    int   axis;        /* 0=X plane, 1=Y plane, 2=Z plane              */
    float pos;         /* coordinate of the plane on its fixed axis    */
    float lo[2], hi[2];/* bounds in the two free axes                  */
    int   mat;         /* material index into k_mats                   */
} Quad;

/*
 * Cornell-box quads. The light at y=0.98 sits just below the ceiling
 * (y=1.0); rays going UP through the light's XZ footprint hit it
 * before the ceiling (smaller t) and pick up the emission.
 */
static const Quad k_quads[] = {
    /* floor    y=-1   x∈[-1,1] z∈[0,2] */ { 1,-1.0f, {-1.f, 0.f}, {1.f, 2.f}, 0 },
    /* ceiling  y=+1   x∈[-1,1] z∈[0,2] */ { 1, 1.0f, {-1.f, 0.f}, {1.f, 2.f}, 0 },
    /* back     z=+2   x∈[-1,1] y∈[-1,1]*/ { 2, 2.0f, {-1.f,-1.f}, {1.f, 1.f}, 0 },
    /* left     x=-1   y∈[-1,1] z∈[0,2] */ { 0,-1.0f, {-1.f, 0.f}, {1.f, 2.f}, 1 },
    /* right    x=+1   y∈[-1,1] z∈[0,2] */ { 0, 1.0f, {-1.f, 0.f}, {1.f, 2.f}, 2 },
    /* light    y=0.98 centred overhead */ { 1, 0.98f,{-0.36f,0.62f},{0.36f,1.38f},3 },
};
#define N_QUADS  ((int)(sizeof k_quads / sizeof k_quads[0]))

/* §5.3 ── Sphere ───────────────────────────────────────────────────── */

/*
 * Two diffuse spheres just above the floor. Bottom at y ≈ -0.98 with
 * floor at y = -1.0, leaving a small gap to avoid numerical
 * self-intersection between sphere and floor.
 */
typedef struct { V3 c; float r; int mat; } Sphere;

static const Sphere k_spheres[] = {
    { {-0.46f, -0.60f, 0.82f}, 0.38f, 4 },   /* gold  , left  */
    { { 0.44f, -0.60f, 1.16f}, 0.38f, 5 },   /* indigo, right */
};
#define N_SPHERES ((int)(sizeof k_spheres / sizeof k_spheres[0]))

/* §5.4 ── shorthand: emissive predicate ────────────────────────────── */

static inline bool mat_is_light(const Mat *m)
{
    return m->emit.x > 0.f || m->emit.y > 0.f || m->emit.z > 0.f;
}

/* ── §6 intersection ─────────────────────────────────────────────────── */

/*
 * Hit record: position, surface normal (oriented toward the ray), and
 * the material index. The normal-toward-ray convention means the
 * hemisphere sampling in §7.2 always produces directions on the
 * outgoing side without needing a flip later.
 */
typedef struct { float t; V3 P, N; int mat; } Hit;

/* §6.1 ── ray vs axis-aligned quad ─────────────────────────────────── */

/*
 * On the fixed axis: t = (pos − ro_axis) / rd_axis. Then the hit
 * point's two free-axis coordinates must lie within [lo, hi]. The
 * outward normal is the basis vector of the fixed axis with sign
 * chosen so it faces the incoming ray.
 */
static int ray_quad(V3 ro, V3 rd, const Quad *q,
                    float t_min, float *t_out, V3 *n_out)
{
    float dc, oc;
    switch (q->axis) {
    case 0:  dc = rd.x; oc = ro.x; break;
    case 1:  dc = rd.y; oc = ro.y; break;
    default: dc = rd.z; oc = ro.z; break;
    }
    if (fabsf(dc) < 1e-9f) return 0;                /* ray parallel to plane */

    float t = (q->pos - oc) / dc;
    if (t < t_min) return 0;

    float px = ro.x + t * rd.x;
    float py = ro.y + t * rd.y;
    float pz = ro.z + t * rd.z;

    float u, vv;
    switch (q->axis) {
    case 0:  u = py; vv = pz; break;
    case 1:  u = px; vv = pz; break;
    default: u = px; vv = py; break;
    }
    if (u < q->lo[0] || u > q->hi[0] || vv < q->lo[1] || vv > q->hi[1])
        return 0;

    /* Normal: basis vector of fixed axis, sign opposing rd. */
    V3 n = {0,0,0};
    switch (q->axis) {
    case 0:  n.x = (dc > 0.f) ? -1.f : 1.f; break;
    case 1:  n.y = (dc > 0.f) ? -1.f : 1.f; break;
    default: n.z = (dc > 0.f) ? -1.f : 1.f; break;
    }
    *t_out = t;
    *n_out = n;
    return 1;
}

/* §6.2 ── ray vs sphere ─────────────────────────────────────────────── */

/*
 * Standard quadratic. With unit-length rd, the equation
 *   |ro + t·rd − c|² = r²
 * expands to t² + 2(b)t + (oc·oc − r²) = 0 where b = rd·oc.
 * Discriminant disc = b² − (oc·oc − r²). Roots t = −b ± √disc.
 *
 * We pick the nearest root that's beyond t_min (front face for an
 * outside ray; far root only if the ray is inside the sphere — which
 * can't happen here since materials are diffuse and rays don't enter
 * spheres).
 */
static int ray_sphere(V3 ro, V3 rd, const Sphere *s,
                      float t_min, float *t_out)
{
    V3    oc   = v3sub(ro, s->c);
    float b    = v3dot(rd, oc);
    float disc = b * b - v3dot(oc, oc) + s->r * s->r;
    if (disc < 0.f) return 0;

    float sq = sqrtf(disc);
    float t  = -b - sq;
    if (t < t_min) t = -b + sq;
    if (t < t_min) return 0;
    *t_out = t;
    return 1;
}

/* §6.3 ── scene_hit (find nearest surface) ─────────────────────────── */

/*
 * Loop all primitives and keep the smallest valid t. For 8 quads + 2
 * spheres this brute-force test is fine; a real renderer would put a
 * BVH here, but BVH belongs in a separate teaching file (raymarcher.c
 * uses spatial structures).
 */
static int scene_hit(V3 ro, V3 rd, float t_min, Hit *h)
{
    float t_best = 1e30f;
    int   any    = 0;

    for (int i = 0; i < N_QUADS; i++) {
        float t; V3 n;
        if (ray_quad(ro, rd, &k_quads[i], t_min, &t, &n) && t < t_best) {
            t_best = t;
            h->t   = t;
            h->P   = v3add(ro, v3s(t, rd));
            h->N   = n;
            h->mat = k_quads[i].mat;
            any    = 1;
        }
    }
    for (int i = 0; i < N_SPHERES; i++) {
        float t;
        if (ray_sphere(ro, rd, &k_spheres[i], t_min, &t) && t < t_best) {
            t_best = t;
            h->t = t;
            h->P = v3add(ro, v3s(t, rd));
            V3 outN = v3norm(v3sub(h->P, k_spheres[i].c));
            /* Flip outward normal to face the incoming ray. */
            h->N   = (v3dot(outN, rd) < 0.f) ? outN : v3s(-1.f, outN);
            h->mat = k_spheres[i].mat;
            any    = 1;
        }
    }
    return any;
}

/* ── §7 path trace (THE CORE) ────────────────────────────────────────── */

/* §7.1 ── onb: orthonormal basis around a normal ────────────────────── */

/*
 * Given a unit normal n, construct two perpendicular unit vectors u
 * and v so {u, v, n} is a right-handed orthonormal basis. Used by the
 * cosine-hemisphere sampler to translate "local +Z = normal" samples
 * into world coordinates.
 *
 * The "pick non-parallel up" trick: if n is mostly along X (|n.x| ≥ 0.9)
 * use Y as the seed; otherwise use X. Either way `up × n` is non-zero.
 */
static void onb(V3 n, V3 *u, V3 *v)
{
    V3 up = (fabsf(n.x) < 0.9f) ? v3(1, 0, 0) : v3(0, 1, 0);
    *u = v3norm(v3cross(up, n));
    *v = v3cross(n, *u);
}

/* §7.2 ── cos_sample_hemi: cosine-weighted hemisphere sample ────────── */

/*
 * Malley's method (1988): the 2D uniform-disk sampling distribution,
 * lifted to a hemisphere via z = √(1 − r²), produces a 3D distribution
 * whose density is exactly cosθ/π — which is what we want.
 *
 *   r1 ∈ [0,1)  →  φ = 2π · r1
 *   r2 ∈ [0,1)  →  in-plane radius = √r2
 *                  z   (cosθ)      = √(1 − r2)
 *
 * Local sample is (cosφ·√r2, sinφ·√r2, √(1−r2)). Transform to world
 * using the (u, v, n) basis from §7.1.
 *
 * Why this method beats rejection sampling:
 *   • Always two RNG calls — no loop with worst-case unbounded retries.
 *   • Numerically stable at grazing angles where rejection thrashes.
 *   • PDF analytically known and matches the Lambertian BRDF — see the
 *     KEY FORMULAS block above for the cancellation argument.
 */
static V3 cos_sample_hemi(V3 n, Rng *rng)
{
    float r1  = rng_f(rng);
    float r2  = rng_f(rng);
    float phi = 2.f * (float)M_PI * r1;
    float sr2 = sqrtf(r2);                          /* sinθ */
    float lx  = cosf(phi) * sr2;
    float ly  = sinf(phi) * sr2;
    float lz  = sqrtf(1.f - r2);                    /* cosθ */
    V3 u, vv;
    onb(n, &u, &vv);
    return v3norm(v3add(v3s(lx, u),
                  v3add(v3s(ly, vv),
                        v3s(lz, n))));
}

/* §7.3 ── path_trace: iterative random walk ─────────────────────────── */

/*
 * Trace ONE path from (ro, rd) and return the radiance reaching the
 * camera along it. Iterative (not recursive) so deep paths don't risk
 * stack overflow.
 *
 * The throughput vector multiplies the emission of any light we hit
 * later. Starts at (1,1,1) (no surfaces traversed yet) and shrinks
 * with each Lambertian bounce by a factor of `albedo`.
 *
 * Russian roulette starts at depth ≥ RR_DEPTH so that early bounces —
 * which carry most of the energy — don't die prematurely. After that
 * point, paths whose throughput has dimmed get killed with high
 * probability, which is correct: contributions through a 0.001-
 * throughput path are dwarfed by contributions through a 0.5-
 * throughput path, so spending samples on the dim ones is wasteful.
 */
static V3 path_trace(V3 ro, V3 rd, Rng *rng)
{
    V3 col        = v3(0, 0, 0);
    V3 throughput = v3(1, 1, 1);

    for (int depth = 0; depth < MAX_DEPTH; depth++) {
        Hit h;
        if (!scene_hit(ro, rd, RAY_EPS, &h)) break;       /* miss → black */

        const Mat *m = &k_mats[h.mat];

        /* (a) Emissive surface — accumulate and stop. */
        if (mat_is_light(m)) {
            col = v3add(col, v3mul(throughput, m->emit));
            break;
        }

        /* (b) Russian roulette (only after RR_DEPTH bounces). */
        if (depth >= RR_DEPTH) {
            float p = v3maxc(throughput);
            if (p < 1e-4f || rng_f(rng) > p) break;       /* killed */
            throughput = v3s(1.f / p, throughput);        /* compensate */
        }

        /* (c) Lambertian bounce: throughput *= albedo. */
        throughput = v3mul(throughput, m->albedo);

        /* (d) Sample new direction; offset origin off the surface. */
        rd = cos_sample_hemi(h.N, rng);
        ro = v3add(h.P, v3s(RAY_EPS, h.N));
    }
    return col;
}

/* ── §8 framebuffer (progressive accumulator) ────────────────────────── */

/* §8.1 ── accumulator + reset ──────────────────────────────────────── */

/*
 * Per-pixel running sum of radiance over ALL samples ever cast since
 * the last reset, plus the total sample count. Display = sum / count.
 *
 * Static arrays (no malloc) sized to the largest terminal we'll ever
 * encounter. On resize we reset rather than reallocate — the previous
 * accumulator is meaningless at the new resolution anyway.
 */
static float g_accum[MAX_H][MAX_W][3];
static int   g_samples = 0;

static void accum_reset(void)
{
    memset(g_accum, 0, sizeof g_accum);
    g_samples = 0;
}

/* §8.2 ── accum_add_frame: cast `spp` samples per pixel ────────────── */

/*
 * One frame of additional samples. For each pixel:
 *   1. Compute SPP independent Rng seeds (frame*SPP+s decorrelates
 *      across frames).
 *   2. For each sample, jitter the sub-pixel position uniformly in
 *      [-0.5, +0.5)² to anti-alias.
 *   3. Build the primary ray direction from the jittered position.
 *   4. path_trace; add the result to the accumulator.
 *
 * No tone-mapping here — that happens in accum_draw so we keep the
 * accumulator in linear HDR space. Tone-mapping the running sum
 * each frame would compound: the right place is "after divide".
 */
static void accum_add_frame(int cols, int rows, int spp, int frame_idx)
{
    float fov_tan = tanf(FOV_DEG * (float)M_PI / 360.f);
    float cx = cols * 0.5f, cy = rows * 0.5f;

    V3 cam_pos = v3(CAM_X, CAM_Y, CAM_Z);
    V3 cam_fwd = v3(0, 0, 1);
    V3 cam_rgt = v3(1, 0, 0);
    V3 cam_up  = v3(0, 1, 0);

    for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
        for (int col = 0; col < cols && col < MAX_W; col++) {
            float sr = 0.f, sg = 0.f, sb = 0.f;

            for (int s = 0; s < spp; s++) {
                Rng rng = rng_seed(col, row, frame_idx * spp + s);

                /* Sub-pixel jitter for free anti-aliasing. */
                float jx = rng_f(&rng) - 0.5f;
                float jy = rng_f(&rng) - 0.5f;
                float pu =  ((col + jx) - cx) / cx * fov_tan;
                float pv = -((row + jy) - cy) / cx * fov_tan / ASPECT;

                V3 rd = v3norm(v3add(cam_fwd,
                              v3add(v3s(pu, cam_rgt),
                                    v3s(pv, cam_up))));

                V3 c = path_trace(cam_pos, rd, &rng);
                sr += c.x; sg += c.y; sb += c.z;
            }

            g_accum[row][col][0] += sr;
            g_accum[row][col][1] += sg;
            g_accum[row][col][2] += sb;
        }
    }
    g_samples += spp;
}

/* §8.3 ── tone-mapping + draw ──────────────────────────────────────── */

static inline float reinhard(float x) { return x / (1.f + x); }
static inline float gamma_enc(float x)
{
    return powf(x < 0.f ? 0.f : (x > 1.f ? 1.f : x), 1.f / 2.2f);
}
static inline float rec601_luma(float r, float g, float b)
{
    return 0.2126f * r + 0.7152f * g + 0.0722f * b;
}

/*
 * Per cell:
 *   1. linear average:    L = accum / samples
 *   2. Reinhard tone-map: L' = L / (1 + L)        (HDR → [0,1))
 *   3. gamma encode:      out = L'^(1/2.2)        (linear → sRGB)
 *   4. luminance → ASCII density char
 *   5. RGB → nearest 6×6×6 xterm cube colour pair
 *
 * Step 1 must happen BEFORE tone-mapping — tone-mapping a sum of N
 * samples is not the same as N times the tone-map of a single sample.
 */
static void accum_draw(int cols, int rows)
{
    if (g_samples == 0) return;
    float inv = 1.f / (float)g_samples;

    for (int row = 0; row < rows - 1 && row < MAX_H; row++) {
        for (int col = 0; col < cols && col < MAX_W; col++) {
            float r = g_accum[row][col][0] * inv;
            float g = g_accum[row][col][1] * inv;
            float b = g_accum[row][col][2] * inv;

            r = gamma_enc(reinhard(r));
            g = gamma_enc(reinhard(g));
            b = gamma_enc(reinhard(b));

            float luma = rec601_luma(r, g, b);
            int   ri   = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
            if (ri < 0)            ri = 0;
            if (ri >= RAMP_LEN)    ri = RAMP_LEN - 1;
            char ch = k_ramp[ri];

            int ri5 = (int)(r * 5.f + 0.5f); if (ri5 > 5) ri5 = 5; if (ri5 < 0) ri5 = 0;
            int gi5 = (int)(g * 5.f + 0.5f); if (gi5 > 5) gi5 = 5; if (gi5 < 0) gi5 = 0;
            int bi5 = (int)(b * 5.f + 0.5f); if (bi5 > 5) bi5 = 5; if (bi5 < 0) bi5 = 0;
            int pair = PAIR_CUBE_BASE + ri5 * 36 + gi5 * 6 + bi5;
            attron(COLOR_PAIR(pair) | A_BOLD);
            mvaddch(row, col, (chtype)(unsigned char)ch);
            attroff(COLOR_PAIR(pair) | A_BOLD);
        }
    }
}

/* ── §9 screen ───────────────────────────────────────────────────────── */

static int g_256 = 0;

/* §9.1 ── color_init: 6×6×6 cube + reserved HUD/HINT/bar pairs ─────── */

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);

    if (g_256) {
        /* 216-colour xterm cube: pairs PAIR_CUBE_BASE..+215. */
        for (int i = 0; i < 216; i++)
            init_pair((short)(PAIR_CUBE_BASE + i), 16 + i, -1);
        init_pair(PAIR_HUD,        226, -1);  /* bright yellow         */
        init_pair(PAIR_HINT,        51, -1);  /* bright cyan           */
        init_pair(PAIR_BAR_FILL,    46, -1);  /* bright green (filled) */
        init_pair(PAIR_BAR_EMPTY,  240, -1);  /* dim grey (empty)      */
    } else {
        init_pair(PAIR_CUBE_BASE,  COLOR_WHITE,  -1);
        init_pair(PAIR_HUD,        COLOR_YELLOW, -1);
        init_pair(PAIR_HINT,       COLOR_CYAN,   -1);
        init_pair(PAIR_BAR_FILL,   COLOR_GREEN,  -1);
        init_pair(PAIR_BAR_EMPTY,  COLOR_WHITE,  -1);
    }
}

/* §9.2 ── progress bar (samples / ACCUM_CAP) ───────────────────────── */

static void draw_progress_bar(int row, int cols, int samples)
{
    int bar_w  = cols / 3;
    if (bar_w < 8)  bar_w = 8;
    if (bar_w > 60) bar_w = 60;
    int filled = (int)((float)samples / (float)ACCUM_CAP * bar_w);
    if (filled > bar_w) filled = bar_w;

    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD));
    mvaddch(row, x++, '[');
    for (int i = 0; i < bar_w; i++) {
        bool on = (i < filled);
        int  pair = on ? PAIR_BAR_FILL : PAIR_BAR_EMPTY;
        int  attr = on ? A_BOLD        : A_DIM;
        attron(COLOR_PAIR(pair) | attr);
        mvaddch(row, x++, (chtype)(unsigned char)(on ? '=' : '-'));
        attroff(COLOR_PAIR(pair) | attr);
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvaddch(row, x++, ']');
    attroff(COLOR_PAIR(PAIR_HUD));
}

/* §9.3 ── hud_draw (HUD spec compliant) ────────────────────────────── */

/*
 * Layout:
 *   row 0 (top, yellow + BOLD)         status: fps / spp / samples / state
 *   row 1 (top, yellow + progress bar) convergence visualisation
 *   row rows-1 (bottom, cyan + BOLD)   key hint strip
 */
static void hud_draw(int cols, int rows, float fps,
                     int spp, int samples, bool paused)
{
    /* §9.3.1 status — top-right. */
    char buf[120];
    snprintf(buf, sizeof buf,
             " %5.1f fps  spp:%d  samples:%-5d  %s ",
             (double)fps, spp, samples,
             paused           ? "PAUSED   " :
             samples >= ACCUM_CAP ? "CONVERGED" : "tracing  ");
    int len = (int)strlen(buf);
    if (len > cols) len = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* §9.3.2 title — top-left, same row. */
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, " PATH TRACER · CORNELL BOX ");
    attroff(COLOR_PAIR(PAIR_HUD));

    /* §9.3.3 progress bar — row 1. */
    if (rows > 4) draw_progress_bar(1, cols, samples);

    /* §9.3.4 hint — bottom row, cyan + BOLD. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  r:reset  +/-:spp ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §10 app ─────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint  (int s) { (void)s; g_run    = 0; }
static void on_sigwinch(int s) { (void)s; g_resize = 1; }
static void cleanup(void)      { endwin(); }

int main(void)
{
    signal(SIGINT,   on_sigint);
    signal(SIGTERM,  on_sigint);
    signal(SIGWINCH, on_sigwinch);
    atexit(cleanup);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);

    int cols, rows;
    getmaxyx(stdscr, rows, cols);
    color_init();
    accum_reset();

    int       spp       = SPP_DEFAULT;
    bool      paused    = false;
    float     fps       = 0.f;
    long long fps_acc   = 0;
    int       fps_cnt   = 0;
    long long frame_ns  = 1000000000LL / TARGET_FPS;
    long long last      = clock_ns();
    int       frame_idx = 0;

    while (g_run) {

        /* §10.1 resize — invalidate accumulator, restart frame indexing. */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
            accum_reset();
            frame_idx = 0;
        }

        /* §10.2 timing — wall clock dt with cap. */
        long long now = clock_ns();
        long long dt  = now - last;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        last = now;

        /* §10.3 fps rolling average over half-second windows. */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* §10.4 add samples — auto-pause once converged. */
        if (!paused && g_samples < ACCUM_CAP)
            accum_add_frame(cols, rows, spp, frame_idx++);

        /* §10.5 paint frame. */
        long long t0 = clock_ns();
        erase();
        accum_draw(cols, rows);
        hud_draw(cols, rows, fps, spp, g_samples, paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* §10.6 input. Resetting samples on +/- because the average
         * mixes paths with different SPP weighting if you don't. */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27 /* ESC */:
            g_run = 0; break;
        case ' ': case 'p': case 'P':
            paused = !paused; break;
        case 'r': case 'R':
            accum_reset(); frame_idx = 0; break;
        case '+': case '=':
            if (spp < SPP_MAX) spp++;
            accum_reset(); frame_idx = 0; break;
        case '-': case '_':
            if (spp > SPP_MIN) spp--;
            accum_reset(); frame_idx = 0; break;
        default: break;
        }

        /* §10.7 frame cap. */
        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    return 0;
}
