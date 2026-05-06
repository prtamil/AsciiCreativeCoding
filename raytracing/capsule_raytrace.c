/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * capsule_raytrace.c — analytic ray-traced capsule (cylinder + 2 hemispheres)
 *
 * DEMO: A glossy pill-shaped capsule rotating in space, lit by three
 *       coloured lights. Cycle through phong → normals → fresnel → depth
 *       to see how the same geometry feeds different shading models.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same skeleton, ONE quadratic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *   raytracing/cube_raytrace.c     — same skeleton, slab method
 *
 * Section map:
 *   §1 config     — frame rate, FOV, capsule geometry, camera, ramp, HUD
 *   §2 clock      — monotonic timer + sleep
 *   §3 math       — V3, Mat3 (with a clear inverse-rotation explanation)
 *   §4 color      — themes + 256-colour cube + ASCII-ramp painter
 *   §5 capsule    — THE CORE: analytic ray-capsule intersection
 *                   §5.1 cylinder body (2D ray-circle in projected plane)
 *                   §5.2 hemisphere caps (sphere quadratic at endpoint)
 *                   §5.3 ray_capsule dispatcher
 *   §6 shading    — phong / normals / fresnel / depth
 *   §7 render     — one frame: ray-per-cell, object-space transform
 *   §8 screen     — ncurses init + HUD
 *   §9 app        — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   t         cycle theme (bronze → frost → ember → pine → dusk → pearl)
 *   p / SPC   pause / resume rotation
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/capsule_raytrace.c \
 *       -o capsule_rt -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-capsule intersection. A capsule is the set of
 *                  points within distance r of a finite line segment AB.
 *                  Decompose the test into two analytic sub-problems:
 *                  (1) ray vs INFINITE cylinder along AB (a quadratic in t,
 *                      after projecting out the axial component);
 *                  (2) ray vs HEMISPHERE cap at one endpoint (a sphere
 *                      quadratic). Whichever sub-problem produces the smallest
 *                      positive t is the surface hit. No marching, no mesh.
 *
 * Data-structure : Capsule ≡ (V3 A, V3 B, float r). Two endpoints + one
 *                  radius. Object-space axis is fixed at Y; rotation is
 *                  handled by transforming the RAY into object space rather
 *                  than rotating the capsule.
 *
 * Rendering      : One ray per terminal cell (no AA). Phong shading with
 *                  three coloured world-space lights (warm key, cool fill,
 *                  bright rim). Luminance maps to a 92-character ASCII ramp;
 *                  hue maps to the 6×6×6 256-colour cube. Four shade modes
 *                  show how the same geometry feeds different visual outputs.
 *
 * Performance    : Pure analytic intersection — closed-form quadratic solve
 *                  per pixel, no iteration, no spatial structures. Trivially
 *                  fast for a capsule. The rotation cost is one matrix-vector
 *                  multiply per pixel (transform ray to object space).
 *
 * References     : Inigo Quílez, "Capsule — intersection",
 *                    https://iquilezles.org/articles/intersectors/
 *                  Real-Time Rendering 4e §22.7 (ray-cylinder, ray-sphere).
 *                  Shirley & Marschner, "Fundamentals of Computer Graphics"
 *                    4e ch. 4 (ray-tracing primitives).
 *                  Schlick, "An Inexpensive BRDF Model for Physically-based
 *                    Rendering", Comp. Graph. Forum 13(3) (1994). [fresnel]
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A capsule is a sphere "smeared" along a line segment. A point P lies on
 * the capsule's surface iff the closest point on segment AB to P is exactly
 * radius r away. Splitting the ray test by where that closest point lives
 * (interior of AB → cylinder body; endpoint A → cap A; endpoint B → cap B)
 * turns one nasty geometry test into two textbook quadratics.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine a soap bubble with a paperclip stuck through it. Now drag the
 * paperclip while the bubble keeps the same fixed distance from it. The
 * surface the bubble traces is exactly a capsule:
 *
 *           ╭────────────────╮          ← cap B (hemisphere)
 *          ╱                  ╲
 *         (         B          )
 *         │                    │
 *         │  ←   tube body  →  │       ← cylindrical band of radius r
 *         │                    │
 *         (         A          )
 *          ╲                  ╱
 *           ╰────────────────╯          ← cap A (hemisphere)
 *
 * For ray-tracing:
 *
 *   (a) Look at the side  →  the band is a 2D ray-vs-circle problem after
 *       you ignore (project out) motion ALONG the axis.
 *   (b) Look at an end    →  each cap is a half-sphere; full sphere math
 *       works, then we just remember to bound-check the axial side.
 *
 * The clever trick (Quílez) is: solve the cylinder first, then if the hit
 * lies axially BEYOND the segment, retest the appropriate end-cap. So at
 * most ONE sphere test is ever needed (the nearer endpoint, picked by the
 * sign of the axial coordinate of the failed cylinder hit).
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS  (per pixel)
 * ──────────────────────────────────────────────────
 *  1. Build a primary ray from the camera into world space:
 *        rd = normalize( fwd  +  pu·right  +  pv·up )      where pu, pv are
 *        normalised screen coords scaled by tan(FOV/2) and aspect.
 *  2. Transform ray into capsule's object space (axis = Y) by applying M^T
 *     (the rotation matrix's transpose = its inverse).
 *  3. Solve ray vs INFINITE CYLINDER (§5.1):
 *        a = |ba|² − (ba·rd)²          [ray length² minus axial projection²]
 *        b = |ba|²(rd·oa) − (ba·oa)(ba·rd)
 *        c = |ba|²(|oa|²−r²) − (ba·oa)²
 *        h = b² − a·c     (discriminant of the projected 2D problem)
 *        t_body = (−b − √h)/a
 *        Compute axial signed coordinate y = ba·oa + t_body·(ba·rd).
 *        If 0 < y < |ba|²  →  BODY HIT. Normal: radial outward (P − A) with
 *        the axial component subtracted off, then normalised.
 *  4. Otherwise, pick the nearer cap by the sign of y:
 *        y ≤ 0       →  cap centred at A
 *        y ≥ |ba|²   →  cap centred at B
 *  5. Solve ray vs SPHERE at that cap centre (§5.2). Standard quadratic.
 *        If miss: ray doesn't hit the capsule at all → return 0.
 *  6. Transform normal back to world space (multiply by M).
 *  7. Shade with chosen mode (phong / normals / fresnel / depth).
 *
 * KEY FORMULAS
 * ────────────
 *   ba       = B − A                    (segment vector, |ba|² = baba)
 *   oa       = ro − A                   (ray origin relative to A)
 *
 *   CYLINDER QUADRATIC (Quílez form, no division):
 *     a = baba − bard²                  bard = ba·rd
 *     b = baba·rdoa − baoa·bard         rdoa = rd·oa, baoa = ba·oa
 *     c = baba·(oaoa − r²) − baoa²      oaoa = oa·oa
 *     h = b² − a·c
 *     t = (−b − √h) / a
 *     y = baoa + t·bard                 (axial coordinate of hit; 0..baba)
 *
 *   BODY NORMAL  (radial, axial component removed):
 *     N_body = normalize( (oa + t·rd) − (y/baba)·ba )
 *
 *   CAP SPHERE QUADRATIC at endpoint (cap = A or B):
 *     oc = ro − cap                     (ro = capsule-space ray origin)
 *     b' = rd·oc
 *     c' = oc·oc − r²
 *     h' = b'² − c'
 *     t  = −b' − √h'                    (front face)
 *
 *   CAP NORMAL  (away from cap centre):
 *     N_cap = normalize( oc + t·rd )
 *
 * WORKED EXAMPLE  (verify the math by hand)
 * ─────────────────────────────────────────
 *   Capsule:  A=(0,−0.65,0)  B=(0,+0.65,0)  r=0.35
 *   Camera:   ro=(0,0,−3.4)  rd=(0,0,+1)     (head-on along +Z)
 *
 *   ba   = (0, 1.30, 0)         baba = 1.69
 *   oa   = (0, 0.65,−3.4)       oaoa = 0.4225 + 11.56 = 11.98
 *   bard = ba·rd = 0
 *   baoa = ba·oa = 0.845
 *   rdoa = rd·oa = −3.4
 *
 *   a = 1.69 − 0          = 1.69
 *   b = 1.69·(−3.4) − 0   = −5.746
 *   c = 1.69·(11.98 − 0.1225) − 0.714 = 20.04 − 0.714 = 19.33
 *   h = 33.02 − 32.67     = 0.35       ← > 0, the ray hits the band
 *
 *   t = (5.746 − 0.59)/1.69 ≈ 3.05
 *   y = 0.845 + 3.05·0    = 0.845      ← 0 < y < 1.69, BODY HIT
 *   P_obj = ro + t·rd = (0, 0, −0.35)  ← lies on the front of the tube
 *   N_body = normalize((0, 0.65, −0.35) − (0.845/1.69)·(0,1.3,0))
 *          = normalize((0, 0, −0.35)) = (0, 0, −1)  ← faces camera ✓
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Ray exactly along axis: bard² → baba, a → 0; the cylinder formula
 *    divides by zero. Mathematically the ray sees only caps then, and
 *    the cap test catches it. In practice axis-parallel rays are zero
 *    measure for a rotating capsule, but a guarded a < 1e-9 path would
 *    skip the cylinder test.
 *  • t > 1e-4f epsilon avoids self-intersection at t≈0 when the camera
 *    is grazing the surface or starts inside.
 *  • At the body/cap boundary (y = 0 or y = baba), the body normal and
 *    cap normal AGREE — both are radially outward from the same circle.
 *    No discontinuity is visible; the shading is C¹ across the seam.
 *  • Inverse-rotation trick: rotating a capsule is expensive (move both
 *    endpoints + maintain orientation). Keeping the capsule fixed at the
 *    Y axis and rotating the RAY is equivalent and one matrix-vector
 *    multiply per ray instead. The world↔object boundary is §7.
 *  • Ramp index uses (int)(lum·(N−1)) so lum=1 maps to RAMP_LEN−1 (the
 *    densest char), not to N (out of bounds).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • A static capsule viewed head-on along its axis should look like a
 *    DISC (the front cap silhouette), not a pill — the cylinder is
 *    end-on. Toggle MODE_NORMAL: the disc should be coloured (0,0,−1)
 *    → blue·red·zero in the centre, transitioning radially.
 *  • A capsule viewed perpendicular to its axis shows the classic pill
 *    silhouette: rectangle of width 2r flanked by two arcs of radius r.
 *  • In MODE_DEPTH, the silhouette has the brightest band at the
 *    centre (closest to camera) fading at the rim (oblique grazing
 *    angle, larger t).
 *  • Worked-example check: a head-on ray to a unit-tall capsule at
 *    distance 3.4 should hit at t ≈ 3.05; inspect the source if your
 *    capsule looks wrong.
 *  • Doubling the radius should double the apparent thickness and
 *    leave caps of the same proportional size.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 199309L
#include <ncurses.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <signal.h>

#ifndef M_PI
#  define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

/* §1.1 frame rate */
#define TARGET_FPS    60
#define DT_CAP_NS     100000000LL          /* 0.1 sec — spiral-of-death cap */

/* §1.2 view geometry */
#define ASPECT        0.47f                /* terminal cell W/H ratio       */
#define FOV_DEG       55.0f                /* full vertical-equivalent FOV  */

/* §1.3 capsule (object space, axis along Y, centred on origin) */
#define CAP_HALF_H    0.65f                /* segment endpoints at ±this   */
#define CAP_R         0.35f                /* tube and cap radius          */

/* §1.4 rotation rates (rad/sec) */
#define ROT_Y         0.45f                /* primary spin around Y         */
#define ROT_X         0.22f                /* slow tilt around X            */

/* §1.5 camera (orbits along −Z; bigger distance = smaller capsule) */
#define CAM_DIST_DEF  3.4f
#define CAM_DIST_MIN  1.8f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f

/* §1.6 shading */
#define AMBIENT       0.05f
#define SHININESS     48.0f                /* phong exponent                */

/* §1.7 character ramp — Paul Bourke 92-char density ladder.
 * Index 0 (space) is invisible; index N−1 ('@') is densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.8 ncurses pair IDs (256-colour cube + reserved HUD/HINT) */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD       217
#define PAIR_HINT      218

/* §1.9 epsilon for ray distances */
#define T_EPS         1e-4f                /* reject t < this (self-hit)  */

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

/* ── §3 math (V3, Mat3) ──────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3add   (V3 a, V3 b)    { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub   (V3 a, V3 b)    { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale (float s, V3 a) { return (V3){s*a.x,  s*a.y,  s*a.z};   }
static inline V3    v3mul   (V3 a, V3 b)    { return (V3){a.x*b.x, a.y*b.y, a.z*b.z}; }
static inline float v3dot   (V3 a, V3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z;    }
static inline float v3len   (V3 a)          { return sqrtf(v3dot(a, a));             }
static inline V3    v3norm  (V3 a)          { float l=v3len(a); return l>1e-9f ? v3scale(1.f/l,a) : (V3){0,1,0}; }
static inline V3    v3reflect(V3 v, V3 n)   { return v3sub(v, v3scale(2.f*v3dot(v,n), n)); }
static inline V3    v3clamp1(V3 v)
{
    return (V3){ v.x<0?0:v.x>1?1:v.x, v.y<0?0:v.y>1?1:v.y, v.z<0?0:v.z>1?1:v.z };
}

/* 3×3 rotation matrix — rows stored as V3 for `v3dot` ergonomics. */
typedef struct { V3 r[3]; } Mat3;

/*
 * mat3_rot — composed rotation Rx(rx) · Ry(ry).
 *
 * This is the OBJECT→WORLD transform (mat3_mul). For ray tracing we
 * need the OPPOSITE direction (rotate WORLD ray into capsule's object
 * space, where the axis stays fixed at Y). For a pure rotation the
 * inverse equals the transpose, so mat3_mulT below is the inverse —
 * cheap, no determinant, no division.
 */
static Mat3 mat3_rot(float rx, float ry)
{
    float cx = cosf(rx), sx = sinf(rx);
    float cy = cosf(ry), sy = sinf(ry);
    Mat3 m;
    m.r[0] = (V3){  cy,      0.f,    sy    };
    m.r[1] = (V3){  sx*sy,   cx,    -sx*cy };
    m.r[2] = (V3){ -cx*sy,   sx,     cx*cy };
    return m;
}

/* M · v   — object → world (forward) */
static V3 mat3_mul(Mat3 m, V3 v)
{
    return (V3){ v3dot(m.r[0], v), v3dot(m.r[1], v), v3dot(m.r[2], v) };
}

/* M^T · v — world → object (inverse, since M is orthogonal) */
static V3 mat3_mulT(Mat3 m, V3 v)
{
    return (V3){
        m.r[0].x*v.x + m.r[1].x*v.y + m.r[2].x*v.z,
        m.r[0].y*v.x + m.r[1].y*v.y + m.r[2].y*v.z,
        m.r[0].z*v.x + m.r[1].z*v.y + m.r[2].z*v.z
    };
}

/* ── §4 color / themes ───────────────────────────────────────────────── */

/*
 * Theme — five colour vectors per palette:
 *   obj         base albedo of the capsule
 *   spec        specular highlight tint (key + rim)
 *   key/fill/rim three-light tint (warm·cool·accent)
 */
typedef struct {
    V3 obj, spec, key_col, fill_col, rim_col;
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* bronze — warm antique gold-brown */
    {{0.80f,0.50f,0.18f},{1.00f,0.88f,0.55f},
     {1.00f,0.90f,0.60f},{0.30f,0.20f,0.60f},{0.90f,0.45f,0.08f},"bronze"},
    /* frost — cold pale blue-white */
    {{0.72f,0.88f,1.00f},{0.92f,0.96f,1.00f},
     {0.80f,0.90f,1.00f},{0.12f,0.20f,0.70f},{0.55f,0.82f,1.00f},"frost"},
    /* ember — deep orange-red with bright core */
    {{0.92f,0.38f,0.08f},{1.00f,0.82f,0.45f},
     {1.00f,0.78f,0.38f},{0.25f,0.05f,0.45f},{1.00f,0.28f,0.05f},"ember"},
    /* pine — deep forest green */
    {{0.12f,0.58f,0.22f},{0.50f,0.92f,0.55f},
     {0.65f,1.00f,0.50f},{0.05f,0.28f,0.50f},{0.10f,0.80f,0.30f},"pine"},
    /* dusk — soft purple-mauve */
    {{0.55f,0.25f,0.75f},{0.88f,0.72f,1.00f},
     {0.90f,0.80f,1.00f},{0.10f,0.05f,0.55f},{0.70f,0.25f,0.90f},"dusk"},
    /* pearl — warm cream with soft highlights */
    {{0.92f,0.88f,0.80f},{1.00f,0.98f,0.92f},
     {1.00f,0.95f,0.80f},{0.30f,0.35f,0.70f},{0.95f,0.80f,0.55f},"pearl"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

static int g_256;     /* 1 if 256-colour cube available, 0 = mono fallback */

static void color_init(void)
{
    start_color();
    use_default_colors();
    g_256 = (COLORS >= 256);
    if (g_256) {
        /* Pairs 1..216 ↔ 6×6×6 RGB cube (xterm 16..231).
         * Index = r·36 + g·6 + b + 1, all in {0..5}. */
        for (int i = 0; i < 216; i++)
            init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
    }
    init_pair(PAIR_HUD,  226, -1);          /* bright yellow                 */
    init_pair(PAIR_HINT,  51, -1);          /* bright cyan                   */
}

/*
 * draw_color — paint one cell with a colour and a luminance.
 *
 * Hue → 6×6×6 cube pair (≈ RGB resolution 1/5).
 * Luminance → ASCII-density character. Two channels of the same pixel:
 *   - colour gives "what it is" (gold rim vs blue fill)
 *   - density gives "how bright it is" (dim '. ' vs solid '@')
 * Both together convey shading on a monochrome glyph grid better than
 * either alone.
 */
static void draw_color(int row, int col, V3 c, float lum)
{
    if (lum < 0.f) lum = 0.f;
    if (lum > 1.f) lum = 1.f;
    char ch = k_ramp[(int)(lum * (RAMP_LEN - 1))];

    if (g_256) {
        int r5 = (int)(c.x * 5.f + .5f); if (r5 > 5) r5 = 5;
        int g5 = (int)(c.y * 5.f + .5f); if (g5 > 5) g5 = 5;
        int b5 = (int)(c.z * 5.f + .5f); if (b5 > 5) b5 = 5;
        int pair = PAIR_CUBE_BASE + r5*36 + g5*6 + b5;
        attron(COLOR_PAIR(pair));
        mvaddch(row, col, (chtype)(unsigned char)ch);
        attroff(COLOR_PAIR(pair));
    } else {
        mvaddch(row, col, (chtype)(unsigned char)ch);
    }
}

/* ── §5 ray-capsule intersection (THE CORE) ──────────────────────────── */

/*
 * Capsule = sphere of radius r swept along segment AB.
 *
 * Object-space convention: A = (0,−CAP_HALF_H,0), B = (0,+CAP_HALF_H,0).
 * Axis is fixed at Y. Rotation is applied to the RAY, not the capsule
 * (see §7 — the inverse-rotation trick).
 *
 * Decomposition (Quílez):
 *   §5.1 — solve the infinite cylinder. If the axial coordinate of the
 *          hit lies in [0, |ba|²] the hit is on the BODY.
 *   §5.2 — otherwise, test the nearer hemisphere endpoint as a SPHERE.
 *
 * The dispatcher §5.3 calls these in order and returns the first valid
 * positive t.
 */

/* §5.1 ── ray vs INFINITE CYLINDER body, with axial bounds check ─────── */

/*
 * cylinder_test — hit-test the infinite cylinder of radius r along ba.
 *
 * Inputs:  ro, rd (object-space ray); ba = B−A, oa = ro−A; r (radius);
 *          baba = |ba|² (passed in to avoid recomputing).
 * Outputs: on hit, *t_out = ray parameter, *N_out = world-radial normal.
 *          Also writes *y_out = signed axial coordinate (used by the
 *          dispatcher to choose which cap to test on miss).
 *
 * Returns 1 on body hit, 0 if either:
 *   - the ray misses the infinite cylinder (h < 0), OR
 *   - the hit lies axially outside [0, |ba|²] (ray hits cylinder but
 *     beyond the segment endpoints — caller must try a cap).
 *
 * Key insight: the 3D cylinder problem reduces to a 2D ray-circle
 * problem when you project the ray onto the plane perpendicular to ba.
 * The Quílez form below performs that projection algebraically without
 * explicitly normalizing ba — it just keeps everything multiplied by
 * baba so divisions don't appear until the very end.
 *
 * Equivalent to ray_sphere() in sphere_raytrace.c for the "quadratic in
 * t" pattern, but with one extra axial-bound check on top.
 */
static int cylinder_test(V3 ro, V3 rd,
                         V3 ba, V3 oa, float r, float baba,
                         float *t_out, V3 *N_out, float *y_out)
{
    (void)ro;                                 /* unused — ro−A is precomputed in oa */
    float bard = v3dot(ba, rd);               /* axial component of ray direction */
    float baoa = v3dot(ba, oa);               /* axial offset of ray origin       */
    float rdoa = v3dot(rd, oa);
    float oaoa = v3dot(oa, oa);

    /* Quadratic coefficients (Quílez form, no division by |ba|). */
    float a = baba - bard * bard;
    float b = baba * rdoa - baoa * bard;
    float c = baba * (oaoa - r * r) - baoa * baoa;
    float h = b * b - a * c;
    if (h < 0.f) return 0;                    /* misses infinite cylinder       */

    float t = (-b - sqrtf(h)) / a;            /* nearest front-face hit         */
    float y = baoa + t * bard;                /* axial coord, range [0, baba]   */
    *y_out = y;

    /* Axial bound check: hit only counts if it lies between A and B. */
    if (t > T_EPS && y > 0.f && y < baba) {
        /* Radial outward normal: P − A minus its axial projection.
         * (oa + t·rd) is P − A in capsule space. The axial component is
         * (y/baba)·ba, so subtracting it leaves the perpendicular part. */
        V3 p_minus_A = v3add(oa, v3scale(t, rd));
        *N_out = v3norm(v3sub(p_minus_A, v3scale(y / baba, ba)));
        *t_out = t;
        return 1;
    }
    return 0;                                 /* axial out of range — try cap   */
}

/* §5.2 ── ray vs HEMISPHERE CAP (single sphere quadratic) ────────────── */

/*
 * cap_test — hit-test a sphere of radius r centred at the cap.
 *
 * Caller passes oc = ro − cap_centre. We don't need to know which cap
 * (A or B) — only the offset vector. The hemisphere bound is implicitly
 * satisfied: if the dispatcher reached us, the cylinder produced an
 * axial coordinate y outside [0, baba], so the relevant half-space of
 * the sphere is the one this cap is responsible for.
 *
 * Returns 1 on hit (writes *t_out, *N_out). Otherwise 0.
 */
static int cap_test(V3 rd, V3 oc, float r,
                    float *t_out, V3 *N_out)
{
    float b = v3dot(rd, oc);
    float c = v3dot(oc, oc) - r * r;
    float h = b * b - c;
    if (h < 0.f) return 0;                    /* misses sphere entirely         */

    float t = -b - sqrtf(h);                  /* front face                     */
    if (t < T_EPS) {
        t = -b + sqrtf(h);                    /* maybe inside-out — try back    */
        if (t < T_EPS) return 0;
    }
    /* Cap normal points from centre to surface. */
    *N_out = v3norm(v3add(oc, v3scale(t, rd)));
    *t_out = t;
    return 1;
}

/* §5.3 ── dispatcher: cylinder body, then nearer cap on fall-through ─── */

/*
 * ray_capsule — top-level ray vs capsule.
 *
 * Tries the cylinder first (cheap, covers most pixels of a side-on
 * capsule). If the cylinder produced no valid in-bounds hit, picks the
 * nearer cap by the sign of the axial coordinate y and tests it as a
 * sphere.
 *
 * On hit returns 1 with *t_hit and *N_os (object-space normal).
 */
static int ray_capsule(V3 ro, V3 rd,
                       V3 A,  V3 B,  float r,
                       float *t_hit, V3 *N_os)
{
    V3    ba   = v3sub(B,  A);
    V3    oa   = v3sub(ro, A);
    float baba = v3dot(ba, ba);

    /* y = 0 default is safe: if the cylinder fully misses (h < 0),
     * cylinder_test returns 0 without writing y; in that case the ray
     * cannot hit either cap either (same radius, same axis), and
     * cap_test will return 0 below regardless of which cap we pick. */
    float y = 0.f;
    if (cylinder_test(ro, rd, ba, oa, r, baba, t_hit, N_os, &y))
        return 1;

    /* Cylinder either missed entirely (caller doesn't see y) or the hit
     * was axially outside the segment. In the latter case, y tells us
     * which cap to test:
     *   y ≤ 0     ⇒ closer to A
     *   y ≥ baba  ⇒ closer to B
     * If the cylinder fully missed (h<0), y is uninitialised — but a
     * miss against the infinite cylinder means the ray's perpendicular
     * distance to the axis is > r, so it cannot hit either cap either.
     * We pick A by default; cap_test will return 0 in that case and the
     * caller will see "miss" overall. */
    V3 oc = (y >= baba) ? v3sub(oa, ba) : oa;
    return cap_test(rd, oc, r, t_hit, N_os);
}

/* ── §6 shading ──────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_FRESNEL, MODE_DEPTH, MODE_N } ShadeMode;
static const char *const k_mode_names[] = { "phong","normals","fresnel","depth" };

/* Three fixed world-space lights — positions, not directions. */
static const V3 L_KEY  = { 3.0f, 4.0f, -2.0f };
static const V3 L_FILL = {-4.0f, 1.0f, -1.0f };
static const V3 L_RIM  = { 0.5f,-1.0f,  5.0f };

/*
 * Phong: ambient + Σ_lights (kd·N·L  +  ks·(R·V)^n).
 *
 * The reflection direction for incident light L hitting surface N is
 *   R = 2·(N·L)·N − L
 * (the same operator as v3reflect, with the incident vector pre-flipped
 * because v3reflect operates on the OUTGOING ray). For a viewer looking
 * along V_dir, we compare R against V_dir; (R·V)^shininess sharpens the
 * highlight as shininess grows.
 */
static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3 col = v3scale(AMBIENT, th->obj);

    /* §6.1 KEY light — primary diffuse + sharp specular. */
    {
        V3    L = v3norm(v3sub(L_KEY, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), SHININESS);
        col = v3add(col, v3scale(d * 0.65f, v3mul(th->obj, th->key_col)));
        col = v3add(col, v3scale(s * 0.55f, th->spec));
    }
    /* §6.2 FILL light — soft diffuse, no specular. Lifts the shadow side. */
    {
        V3    L = v3norm(v3sub(L_FILL, P));
        float d = fmaxf(0.f, v3dot(N, L));
        col = v3add(col, v3scale(d * 0.22f, v3mul(th->obj, th->fill_col)));
    }
    /* §6.3 RIM light — wide specular kissing the back silhouette. */
    {
        V3    L = v3norm(v3sub(L_RIM, P));
        float d = fmaxf(0.f, v3dot(N, L));
        V3    R = v3reflect(v3scale(-1.f, L), N);
        float s = powf(fmaxf(0.f, v3dot(R, V_dir)), 10.f);
        col = v3add(col, v3scale(d * 0.18f, v3mul(th->obj, th->rim_col)));
        col = v3add(col, v3scale(s * 0.65f, th->rim_col));
    }
    return v3clamp1(col);
}

/* §6.4 Normal mode — RGB-encoded surface normal (diagnostic).
 * Each component goes from −1..+1 → 0..1 so all three become valid colours. */
static V3 shade_normal(V3 N)
{
    return (V3){ N.x*.5f + .5f, N.y*.5f + .5f, N.z*.5f + .5f };
}

/*
 * §6.5 Fresnel mode — Schlick approximation: F = F₀ + (1−F₀)(1−cosθ)^5.
 *
 * For F₀ = 0 (full dielectric, air), F simplifies to (1−cosθ)^5. The
 * capsule then looks dark facing the camera and glows along the rim —
 * the classic "glass pill" silhouette. The cylinder band shows the
 * effect strongly because its normal sweeps through 0° to 90° relative
 * to the view ray as you move from the axis to the silhouette.
 */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th)
{
    float cosA    = fabsf(v3dot(N, V_dir));
    float inv     = 1.f - cosA;
    float fresnel = inv * inv * inv * inv * inv;       /* (1−cosθ)^5 */
    V3 core = v3scale(0.06f, th->obj);
    V3 edge = v3clamp1(v3add(v3scale(0.7f, th->spec), v3scale(0.5f, th->rim_col)));
    return v3clamp1(v3add(v3scale(1.f - fresnel, core), v3scale(fresnel, edge)));
}

/* §6.6 Depth mode — encode hit distance as brightness (closer = bright).
 * d² (rather than d) gives a steeper falloff so the silhouette pops. */
static V3 shade_depth(float t, float t_max, const Theme *th)
{
    float d = 1.f - fminf(t / t_max, 1.f);
    d = d * d;
    return v3clamp1(v3scale(d, th->obj));
}

/* Rec. 601 luminance for ramp-index choice. */
static inline float rec601_luma(V3 c)
{
    return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* ── §7 render frame ─────────────────────────────────────────────────── */

/*
 * The inverse-rotation trick:
 *
 *   Forward way    :  rotate capsule, fire ray in world space.
 *                     Cost: rotate every primitive every frame.
 *
 *   Backward way   :  keep capsule fixed at object space (axis = Y),
 *                     fire ray, then transform ray INTO object space
 *                     by multiplying by M^T = M⁻¹ (orthogonal matrix).
 *                     Cost: ONE matrix×ray per pixel.
 *
 * For a rotating object this is much cheaper than rotating geometry
 * every frame, and the intersection math stays canonical.
 */
static void render(int cols, int rows,
                   float angle_x, float angle_y, float cam_dist,
                   int theme_idx, ShadeMode mode)
{
    const Theme *th  = &g_themes[theme_idx % THEME_N];
    float fov_tan    = tanf(FOV_DEG * (float)M_PI / 360.f);

    Mat3 M           = mat3_rot(angle_x, angle_y);

    /* §7.1 — fixed camera on −Z, looking toward origin. */
    V3 cam = { 0.f, 0.f, -cam_dist };
    V3 fwd = { 0.f, 0.f,  1.f };
    V3 rgt = { 1.f, 0.f,  0.f };
    V3 up  = { 0.f, 1.f,  0.f };

    /* §7.2 — capsule endpoints in OBJECT space (never rotated). */
    V3 A = { 0.f, -CAP_HALF_H, 0.f };
    V3 B = { 0.f, +CAP_HALF_H, 0.f };

    float cx = cols * 0.5f, cy = rows * 0.5f;

    /* §7.3 — primary loop: one ray per cell. Skip the bottom row for HUD. */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            /* Normalised screen coords with terminal-cell aspect baked in. */
            float pu =  (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;

            V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu, rgt),
                                               v3scale(pv, up))));

            /* Transform ray to object space (capsule fixed at axis Y). */
            V3 ro_os = mat3_mulT(M, cam);
            V3 rd_os = mat3_mulT(M, rd_ws);

            float t_hit;
            V3    N_os;
            if (!ray_capsule(ro_os, rd_os, A, B, CAP_R, &t_hit, &N_os))
                continue;

            /* Normal back to world space; hit point in world space. */
            V3 P_ws  = v3add(cam, v3scale(t_hit, rd_ws));
            V3 N_ws  = mat3_mul(M, N_os);
            V3 V_dir = v3norm(v3sub(cam, P_ws));

            V3 color;
            switch (mode) {
            default:
            case MODE_PHONG:    color = shade_phong  (P_ws, N_ws, V_dir, th); break;
            case MODE_NORMAL:   color = shade_normal (N_ws);                   break;
            case MODE_FRESNEL:  color = shade_fresnel(N_ws, V_dir, th);        break;
            case MODE_DEPTH:    color = shade_depth  (t_hit, cam_dist*2.2f,th);break;
            }

            /* Normal-mode brightness uses a green-weighted version so
             * the ramp tracks the "green is brightest" intuition the
             * eye applies to RGB normal visualisations. */
            float lum = (mode == MODE_NORMAL)
                ? (N_ws.x*.5f+.5f)*.3f + (N_ws.y*.5f+.5f)*.6f + (N_ws.z*.5f+.5f)*.1f
                : rec601_luma(color);

            draw_color(row, col, color, lum);
        }
    }
}

/* ── §8 screen / HUD ─────────────────────────────────────────────────── */

static void hud_draw(int cols, int rows, float fps,
                     int theme_idx, ShadeMode mode, float cam_dist, int paused)
{
    /* §8.1 top-right status row 0 — yellow, BOLD (HUD spec). */
    char buf[96];
    snprintf(buf, sizeof buf, " %5.1f fps  dist:%.1f  %-7s  %s ",
             (double)fps, (double)cam_dist,
             g_themes[theme_idx % THEME_N].name,
             paused ? "PAUSED " : "running");
    int len = (int)strlen(buf);
    if (len > cols) len = cols;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, cols - len, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* §8.2 top-left mode label row 0 — same yellow, no bold (secondary). */
    char buf2[48];
    snprintf(buf2, sizeof buf2, " mode: %s ", k_mode_names[mode]);
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(0, 0, "%s", buf2);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* §8.3 bottom hint strip — cyan, BOLD. ASCII only. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(rows - 1, 0,
             " q:quit  spc/p:pause  s:mode  t:theme  +/-:zoom ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9 app ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run    = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint  (int s) { (void)s; g_run    = 0; }
static void on_sigwinch(int s) { (void)s; g_resize = 1; }

static void cleanup(void) { endwin(); }

int main(void)
{
    signal(SIGINT,   on_sigint);
    signal(SIGTERM,  on_sigint);
    signal(SIGWINCH, on_sigwinch);

    initscr();
    cbreak(); noecho(); curs_set(0);
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);
    typeahead(-1);
    atexit(cleanup);
    color_init();

    int cols, rows;
    getmaxyx(stdscr, rows, cols);

    int       theme_idx = 0;
    ShadeMode mode      = MODE_PHONG;
    float     cam_dist  = CAM_DIST_DEF;
    float     angle_x   = 0.f;
    float     angle_y   = 0.f;
    int       paused    = 0;

    float     fps       = 0.f;
    long long fps_acc   = 0;
    int       fps_cnt   = 0;
    long long frame_ns  = 1000000000LL / TARGET_FPS;
    long long last      = clock_ns();

    while (g_run) {
        /* §9.1 resize. */
        if (g_resize) {
            g_resize = 0;
            endwin(); refresh();
            getmaxyx(stdscr, rows, cols);
        }

        /* §9.2 timing. dt is wall-clock; cap to avoid huge jumps after
         * a stall (debugger pause, suspend/resume). */
        long long now = clock_ns();
        long long dt  = now - last;
        if (dt > DT_CAP_NS) dt = DT_CAP_NS;
        last = now;

        /* §9.3 advance rotation if not paused. */
        if (!paused) {
            angle_y += ROT_Y * (float)dt * 1e-9f;
            angle_x += ROT_X * (float)dt * 1e-9f;
        }

        /* §9.4 fps rolling average over half-second windows. */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* §9.5 paint frame. */
        long long t0 = clock_ns();
        erase();
        render(cols, rows, angle_x, angle_y, cam_dist, theme_idx, mode);
        hud_draw(cols, rows, fps, theme_idx, mode, cam_dist, paused);
        wnoutrefresh(stdscr);
        doupdate();

        /* §9.6 input. */
        int ch = getch();
        switch (ch) {
        case 'q': case 'Q': case 27 /* ESC */:
            g_run = 0; break;
        case ' ': case 'p': case 'P':
            paused = !paused; break;
        case 's': case 'S':
            mode = (ShadeMode)((mode + 1) % MODE_N); break;
        case 't': case 'T':
            theme_idx = (theme_idx + 1) % THEME_N; break;
        case '+': case '=':
            cam_dist -= CAM_DIST_STEP;
            if (cam_dist < CAM_DIST_MIN) cam_dist = CAM_DIST_MIN;
            break;
        case '-': case '_':
            cam_dist += CAM_DIST_STEP;
            if (cam_dist > CAM_DIST_MAX) cam_dist = CAM_DIST_MAX;
            break;
        default: break;
        }

        /* §9.7 frame cap. */
        clock_sleep_ns(frame_ns - (clock_ns() - t0));
    }
    return 0;
}
