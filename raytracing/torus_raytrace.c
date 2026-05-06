/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * torus_raytrace.c — analytic ray-traced torus (quartic intersection)
 *
 * DEMO: A glossy donut tumbling in space, lit by three coloured lights.
 *       Cycle phong → normals → fresnel → depth to see the same
 *       quartic-solved geometry through different shading models.
 *       The torus is the ONLY primitive in this folder that needs a
 *       degree-4 polynomial — sphere is degree 2, cylinder is degree
 *       2, AABB is linear. The torus's tube curving around a ring
 *       creates the extra algebraic complexity.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — quadratic intersection (degree 2)
 *   raytracing/cube_raytrace.c     — slab method (linear per axis)
 *   raytracing/capsule_raytrace.c  — decomposed analytic (cyl + 2 spheres)
 *   raytracing/path_tracer.c       — same RGB-cube paint pipeline,
 *                                    multi-bounce Monte Carlo on top
 *
 * Section map:
 *   §1 config     — frame rate, FOV, torus geometry, camera, ramp, HUD
 *   §2 clock      — monotonic timer + sleep
 *   §3 math       — V3, Mat3 (with the inverse-rotation explanation)
 *   §4 color      — themes + 256-colour cube + ASCII-ramp painter
 *   §5 torus      — THE CORE: ray-torus quartic + numerical solver
 *                   §5.1 q_eval (Horner-form polynomial evaluation)
 *                   §5.2 ray_torus (derive coefficients, scan + bisect)
 *                   §5.3 torus_normal (closest-point geometric formula)
 *   §6 shading    — phong / normals / fresnel / depth
 *                   §6.1 KEY light  §6.2 FILL light  §6.3 RIM light
 *                   §6.4 shade_phong glue §6.5 normal §6.6 fresnel/depth
 *   §7 render     — one frame: ray-per-cell, object-space transform
 *   §8 screen     — ncurses init + HUD
 *   §9 app        — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   t         cycle theme (titanium → solar → cobalt → forest → rose → chrome)
 *   p / SPC   pause / resume rotation
 *   r         reset rotation angles to zero
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/torus_raytrace.c \
 *       -o torus_rt -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-torus intersection via QUARTIC ROOT-
 *                  FINDING. The torus implicit equation
 *                    (√(x² + z²) − R)² + y² = r²
 *                  contains a square root that, when expanded against
 *                  the ray P(t) = ro + t·rd, must be squared to remove
 *                  the radical — squaring once gives a degree-2
 *                  expression on each side; combining gives a degree-4
 *                  polynomial in t. Coefficient algebra:
 *
 *                  Let oc = ro, dr = rd, C0 = |ro|² + R² − r².
 *                    A = 4 (rd · ro)
 *                    B = 4 (rd · ro)² + 2·C0 − 4·R² (rdx² + rdz²)
 *                    C = 4 (rd · ro)·C0 − 8·R² (rdx·rox + rdz·roz)
 *                    D = C0² − 4·R² (rox² + roz²)
 *                  Solve t⁴ + A·t³ + B·t² + C·t + D = 0 for the
 *                  smallest positive real root. We use SAMPLE-AND-BISECT:
 *                  evaluate the polynomial at uniformly-spaced t-values,
 *                  scan for sign changes, then bisect to refine to ~10⁻⁶
 *                  precision. This avoids the numerical instability of
 *                  the closed-form Ferrari solution.
 *
 *                  Surface normal at hit point P:
 *                    project P onto the XZ plane (P_xz),
 *                    rescale to the ring centreline at radius R,
 *                    N = normalise(P − ring_pt)
 *                  This is the geometric "closest point on the tube
 *                  centreline" formula and is faster + more stable
 *                  than computing the implicit gradient.
 *
 * Data-structure : Torus ≡ (float R, float r). Two scalars. The torus
 *                  is fixed in the XZ plane at the origin in OBJECT
 *                  space; rotation is applied to the RAY (inverse-
 *                  rotation trick), so the geometry stays simple.
 *                  Themes are RGB triplets per role; 216 ncurses pairs
 *                  pre-allocated as a 6×6×6 RGB cube.
 *
 * Rendering      : One ray per terminal cell. RGB shading → quantised
 *                  to xterm 6×6×6 cube + Bourke 92-char density ramp.
 *                  Same paint pipeline as the rest of the folder; the
 *                  quartic solver is the only torus-specific code.
 *
 * Performance    : ~Q_SAMPLES (256) polynomial evaluations + Q_BISECT
 *                  (40) bisections per pixel that hits → ~300 floating-
 *                  point ops in the worst case. Sphere: ~10. Cube: ~6.
 *                  The torus is intrinsically the most expensive
 *                  analytic primitive in this folder. At 240×80 cells
 *                  × 60 fps ≈ 6 ms shading per frame on modern CPU.
 *
 * References     : Hanrahan, P. "Ray Tracing Algebraic Surfaces,"
 *                    SIGGRAPH '83. (The general approach for higher-
 *                    degree implicit surfaces.)
 *                  Inigo Quílez, "Torus — intersection,"
 *                    https://iquilezles.org/articles/intersectors/
 *                  Press et al., "Numerical Recipes in C" 2e §5.6
 *                    (root finding by bracketing + bisection).
 *                  Hearn & Baker, "Computer Graphics with OpenGL" 4e
 *                    ch. 10 (implicit surface ray tracing).
 *                  Wikipedia: Torus — implicit equation derivation.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A torus is the set of points whose distance to a horizontal CIRCLE
 * (the "ring centreline") equals r — the tube radius. The circle has
 * radius R (the major radius). Equivalently: a sphere of radius r
 * SWEPT along a circle of radius R produces a torus.
 *
 * To test if a ray hits a torus, write that condition algebraically
 * (a quartic in t), then numerically find the smallest positive root.
 * The square-root in the distance formula forces the polynomial to
 * be degree 4 — sphere is degree 2 because the centreline is a single
 * POINT not a curve.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 *
 *           ╭────────────────╮
 *          ╱                  ╲
 *         │   ●─────────────   │      ←  R (major radius)
 *         │   tube centreline  │           horizontal circle
 *          ╲                  ╱
 *           ╰────────────────╯
 *                    ↑
 *               r (minor radius)
 *               tube cross-section
 *
 * Picture a donut sitting flat on a table. The dough forms a circular
 * "centreline" (radius R) running through the middle of the tube; the
 * tube itself is a circle of radius r in cross-section. Every point
 * on the donut's surface is at distance r from the nearest point on
 * the centreline.
 *
 * For ray tracing, plug the parametric ray P(t) = ro + t·rd into the
 * torus implicit equation:
 *
 *         (√(P.x² + P.z²) − R)² + P.y² = r²
 *
 * The √ is unavoidable — it's the distance from P to the Y axis. We
 * isolate the √ on one side and SQUARE both sides to remove it. The
 * algebra (a few pages on paper) collapses to a degree-4 polynomial
 * in t with four nice closed-form coefficients. Solving the quartic
 * is the entire intersection problem — no special cases, no spatial
 * structure, no marching.
 *
 * For numerical solving we DON'T use Ferrari's closed-form formula
 * (it's notoriously unstable near tangent rays — small input
 * perturbations cause large output errors). Instead:
 *
 *   1. Sample the polynomial at Q_SAMPLES uniformly-spaced t-values
 *      in [ε, T_MAX].
 *   2. Wherever consecutive samples have OPPOSITE SIGNS, a real root
 *      lies in that interval (intermediate value theorem).
 *   3. Bisect inside the interval Q_BISECT times to nail the root
 *      down to floating-point precision.
 *   4. Return the FIRST positive root we find — that's the front-face
 *      hit in ray order.
 *
 * The normal is computed geometrically, not as the implicit gradient:
 * project P down to the XZ plane, rescale that 2D vector to length R
 * (giving the closest centreline point), and the normal is just the
 * unit vector from that point to P.
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS  (per pixel)
 * ──────────────────────────────────────────────────
 *  1. Build a primary ray from camera into world space:
 *        rd_ws = normalize( fwd + pu·right + pv·up )
 *  2. Transform ray to OBJECT space (torus stays in the XZ plane there):
 *        ro_os = M^T · cam            (M is the torus's rotation)
 *        rd_os = M^T · rd_ws
 *  3. Compute the four quartic coefficients A, B, C, D from the
 *     pre-derived formulas (§5.2):
 *        rod    = rd · ro
 *        C0     = |ro|² + R² − r²
 *        A      = 4·rod
 *        B      = 4·rod² + 2·C0 − 4·R²·(rdx² + rdz²)
 *        C      = 4·rod·C0 − 8·R²·(rdx·rox + rdz·roz)
 *        D      = C0² − 4·R²·(rox² + roz²)
 *  4. Scan polynomial values q(t) = t⁴ + A·t³ + B·t² + C·t + D over
 *     uniformly-spaced t ∈ [ε, T_MAX]. On the FIRST sign change,
 *     bisect inside that bracket → return the refined root.
 *  5. Compute hit point P_os = ro_os + t·rd_os in object space.
 *  6. Compute torus normal N_os via the closest-point formula.
 *  7. Transform normal back to world: N_ws = M · N_os.
 *  8. Shade with chosen mode (phong / normals / fresnel / depth).
 *
 * KEY FORMULAS
 * ────────────
 *   Implicit torus (in XZ plane):
 *     (√(x² + z²) − R)² + y² = r²
 *
 *   Quartic from substituting P(t) = ro + t·rd  (rd unit-length):
 *     t⁴ + A·t³ + B·t² + C·t + D = 0
 *     A = 4 · (rd · ro)
 *     B = 4 (rd·ro)² + 2 (|ro|² + R² − r²) − 4 R² (rdx² + rdz²)
 *     C = 4 (rd·ro) (|ro|² + R² − r²) − 8 R² (rdx·rox + rdz·roz)
 *     D = (|ro|² + R² − r²)² − 4 R² (rox² + roz²)
 *
 *   Horner evaluation (3 mults + 3 adds vs 6 mults naive):
 *     q(t) = t·(t·(t·(t + A) + B) + C) + D
 *
 *   Closest-point normal:
 *     P_xz    = (P.x, 0, P.z)
 *     ρ       = |P_xz|
 *     ring_pt = (R/ρ) · P_xz             (normalise then scale to R)
 *     N       = normalize(P − ring_pt)
 *
 *   Phong shading (per light):
 *     L    = normalize(light_pos − P)
 *     R    = 2(N·L)·N − L
 *     I    = albedo · max(0, N·L) · light_col      ← diffuse
 *          + spec_col · max(0, R·V_dir)^shininess  ← specular
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   Torus R=0.68, r=0.28 in the XZ plane. Head-on equatorial ray:
 *     ro = (0, 0, −3.4)        ← camera centred, equatorial plane
 *     rd = (0, 0, +1)          ← looking toward +Z
 *
 *   Pre-computed quantities:
 *     |ro|² = 0 + 0 + 11.56 = 11.56
 *     rod   = rd · ro = −3.4
 *     rxz²  = ro.x² + ro.z² = 11.56
 *     rdxz_d = rd.x·ro.x + rd.z·ro.z = −3.4
 *     rdxz² = rd.x² + rd.z² = 1.0
 *     C0    = 11.56 + 0.4624 − 0.0784 = 11.944
 *
 *   Coefficients:
 *     A = 4·(−3.4)                          = −13.6
 *     B = 4·11.56 + 2·11.944 − 4·0.4624·1.0 = 68.28
 *     C = 4·(−3.4)·11.944 − 8·0.4624·(−3.4) = −149.9
 *     D = 11.944² − 4·0.4624·11.56          = 121.27
 *
 *   Geometrically: the ray runs along z, so it should hit the torus
 *   at four points where (|z| − R)² = r² (since x = 0, y = 0):
 *     |z| − R = ±r  →  |z| ∈ {R − r, R + r} = {0.40, 0.96}
 *   Four roots in t (mapping z = −3.4 + t):
 *     t1 ≈ 2.44   (front-near, z = −0.96)
 *     t2 ≈ 3.00   (front-far,  z = −0.40)
 *     t3 ≈ 3.80   (back-near,  z = +0.40)
 *     t4 ≈ 4.36   (back-far,   z = +0.96)
 *
 *   Verify by plugging t = 2.44 into q(t):
 *     q(2.44) ≈ 35.5 − 197.4 + 406.5 − 365.8 + 121.3 ≈ 0.1   ✓ root!
 *   And t = 2.4:
 *     q(2.4)  ≈ 33.18 − 188.0 + 393.3 − 359.8 + 121.3 ≈ 0.0  ✓
 *
 *   The scan-and-bisect solver finds t1 first (smallest positive),
 *   returns t = 2.44 → smallest positive front-face hit.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • RAY MISSING TORUS BUT NEAR HOLE. A ray that passes through the
 *    centre of the donut hole (along the axis) doesn't hit at all —
 *    the polynomial has no real roots. We detect this as no sign
 *    change in the scan window. Camera at (0, 1.8, −3.4) looking at
 *    origin produces such a ray for the centre cell.
 *  • MULTIPLE ROOTS FROM ONE BRACKET. A "double root" (tangent ray)
 *    appears as the polynomial dipping to zero and returning without
 *    crossing — no sign change, missed. With our discrete sampling,
 *    tangent rays are typically lost (acceptable for visual quality;
 *    a more robust solver would also find local extrema).
 *  • COEFFICIENT MAGNITUDE. With camera at distance 3-4 and torus at
 *    origin, polynomial coefficients can be up to ~200. Single-
 *    precision float retains ~7 decimal digits — fine for 10⁻⁶
 *    precision after 40 bisections (each halves the bracket).
 *  • SCAN STEP SIZE. Q_SAMPLES = 256 gives Δt = T_MAX/256 ≈ 0.07.
 *    Two roots within 0.07 of each other (a near-tangent pass) can
 *    be missed. Pickup is monotonic when two roots are well-separated.
 *  • NORMAL AT HOLE CENTRE. If P = origin (ρ = 0), the ring_pt is
 *    undefined (divide-by-zero). We default to (R, 0, 0) — but this
 *    case can't actually occur on the torus surface (origin is in
 *    the hole, not on the tube), so the safety code is unreachable.
 *  • OBJECT-SPACE TRANSFORM. The torus stays at (0,0,0) in the XZ
 *    plane in OBJECT space. Rotation is applied to the RAY by M^T,
 *    not to the geometry. This keeps the quartic coefficients clean.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Static torus, no rotation, camera elevated: silhouette is a
 *    classic donut shape — outer circle of radius R+r and inner
 *    "hole" of radius R-r. With R=0.68, r=0.28 that's 0.96 outer
 *    and 0.40 inner.
 *  • MODE_NORMAL: each direction gets a unique RGB. The TOP of the
 *    tube (positive Y) → green-dominant. The OUTER side (away from
 *    Y axis) → red/blue. The INNER side (toward axis) → opposite.
 *    This rainbow-on-tube look is the easiest way to see normals
 *    are oriented correctly.
 *  • MODE_FRESNEL: dark when looking head-on (cosθ ≈ 1 → F ≈ 0),
 *    bright at silhouette (cosθ ≈ 0 → F ≈ 1). Both inner and outer
 *    silhouette glow brightly.
 *  • MODE_DEPTH: cells closer to camera brighter. As the torus
 *    rotates, the pattern flows — closer parts swell forward.
 *  • Worked-example check: a head-on equatorial ray (ro=(0,0,−3.4),
 *    rd=(0,0,1)) should produce 4 roots: t ≈ 2.44, 3.00, 3.80, 4.36
 *    corresponding to entry-front, entry-back, exit-front, exit-back
 *    of the tube. The solver returns the FIRST one, t ≈ 2.44.
 *  • Cycle THEME (t): obj/spec/light tints all change while geometry
 *    stays fixed.
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
#define FOV_DEG       52.0f                /* full vertical-equivalent FOV  */

/* §1.3 torus (object space, in XZ plane, centred at origin)
 *
 *   R    major radius — distance from origin to ring centreline
 *   r    minor radius — tube cross-section radius
 *
 * The ratio r/R determines tube fatness. r/R = 0.41 is a fairly
 * chunky donut; 0.20 would be a slender ring; 0.50 a fat tire. */
#define TORUS_R       0.68f
#define TORUS_r       0.28f

/* §1.4 rotation rates (rad/sec) */
#define ROT_Y         0.40f                /* primary spin around Y         */
#define ROT_X         0.18f                /* slow tilt around X            */

/* §1.5 camera (orbits along −Z, elevated to see hole + tube) */
#define CAM_DIST_DEF  3.4f
#define CAM_DIST_MIN  1.6f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f
#define CAM_HEIGHT    1.8f                 /* elevation above equator       */

/* §1.6 shading */
#define AMBIENT       0.04f
#define SHININESS     60.0f                /* phong exponent for KEY        */

/* §1.7 quartic solver — sample then bisect.
 *
 * Q_SAMPLES   number of uniformly-spaced t-values to evaluate q(t) at
 *             when scanning for sign changes
 * Q_BISECT    bisection iterations after a sign change is found
 *             (each halves the bracket — 40 → ~10⁻¹² precision)
 * Q_T_MAX     furthest t-value to scan (rays beyond never find roots
 *             in front of the camera at typical distances)
 *
 * Trade-offs:
 *   - More Q_SAMPLES → more reliable detection of close-paired roots,
 *     but linearly more polynomial evaluations per pixel.
 *   - More Q_BISECT → more precise root, but only logarithmic gain
 *     after ~30 iterations (machine precision is reached).
 */
#define Q_SAMPLES     256
#define Q_BISECT       40
#define Q_T_MAX       18.0f

/* §1.8 character ramp — Paul Bourke 92-char density ladder. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.9 ncurses pair IDs (256-colour cube + reserved HUD/HINT) */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD       217
#define PAIR_HINT      218

/* §1.10 epsilon */
#define T_EPS         1e-3f                /* lower bound for t scan      */

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
static inline V3    v3cross (V3 a, V3 b)    { return (V3){a.y*b.z-a.z*b.y, a.z*b.x-a.x*b.z, a.x*b.y-a.y*b.x}; }
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
 * need the OPPOSITE direction (rotate WORLD ray into the torus's
 * object space, where the ring stays in the XZ plane). For a pure
 * rotation the inverse equals the transpose, so mat3_mulT below is
 * the inverse — cheap, no determinant, no division.
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
 *   obj         base albedo of the torus
 *   spec        specular highlight tint (KEY + RIM)
 *   key/fill/rim three-light tint (warm·cool·accent)
 */
typedef struct {
    V3 obj, spec, key_col, fill_col, rim_col;
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* titanium — cool silver */
    {{0.72f,0.72f,0.78f},{1.00f,1.00f,1.00f},
     {1.00f,0.96f,0.88f},{0.22f,0.32f,0.80f},{0.82f,0.88f,1.00f},"titanium"},
    /* solar — molten gold-orange */
    {{0.95f,0.62f,0.08f},{1.00f,0.90f,0.55f},
     {1.00f,0.90f,0.65f},{0.35f,0.12f,0.08f},{1.00f,0.55f,0.10f},"solar"},
    /* cobalt — electric blue */
    {{0.08f,0.32f,0.92f},{0.55f,0.75f,1.00f},
     {0.78f,0.88f,1.00f},{0.04f,0.08f,0.55f},{0.28f,0.58f,1.00f},"cobalt"},
    /* forest — deep organic green */
    {{0.12f,0.58f,0.22f},{0.50f,1.00f,0.58f},
     {0.68f,1.00f,0.50f},{0.04f,0.28f,0.42f},{0.15f,0.80f,0.35f},"forest"},
    /* rose — warm pink-red */
    {{0.90f,0.28f,0.48f},{1.00f,0.78f,0.82f},
     {1.00f,0.85f,0.72f},{0.30f,0.05f,0.25f},{1.00f,0.35f,0.55f},"rose"},
    /* chrome — high-contrast silver */
    {{0.82f,0.82f,0.88f},{1.00f,1.00f,1.00f},
     {1.00f,1.00f,1.00f},{0.15f,0.20f,0.50f},{0.92f,0.92f,1.00f},"chrome"},
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
 * Hue → 6×6×6 cube pair; luminance → ASCII-density character.
 * Both channels of the same pixel: colour says "what it is," density
 * says "how bright it is."
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

/* ── §5 ray-torus intersection (THE CORE) ────────────────────────────── */

/*
 * Torus geometry:
 *   Torus lies in the XZ plane centred at the origin. Major radius R,
 *   minor radius r (tube cross-section). Points on the surface
 *   satisfy:
 *       (√(x² + z²) − R)² + y² = r²
 *
 * Substituting P(t) = ro + t·rd and squaring twice (to remove the
 * radical) collapses the surface equation to a quartic in t — see the
 * MENTAL MODEL block above for the algebra and §5.2 for the resulting
 * coefficient formulas.
 *
 * The solver is split into three named functions:
 *   §5.1 q_eval        evaluate the quartic at a given t (Horner form)
 *   §5.2 ray_torus     derive A,B,C,D + scan-bisect for the smallest root
 *   §5.3 torus_normal  closest-point normal at a hit point
 */

/* §5.1 q_eval — Horner-form polynomial evaluation ─────────────────── */

/*
 * q(t) = t⁴ + A·t³ + B·t² + C·t + D
 *
 * Horner factoring rearranges to:
 *   q(t) = ((((t)·1 + A)·t + B)·t + C)·t + D
 *
 * which is 3 multiplications + 4 additions instead of 6 multiplications
 * naive. With Q_SAMPLES = 256 evaluations per pixel × cols × rows,
 * the saving adds up.
 */
static inline float q_eval(float t, float A, float B, float C, float D)
{
    return t*(t*(t*(t + A) + B) + C) + D;
}

/* §5.2 ray_torus — derive coefficients + scan-bisect for smallest root */

/*
 * ray_torus — analytic ray vs torus intersection.
 *
 * Inputs:  ro, rd — ray in OBJECT space (torus stays in XZ plane)
 *          R, r_minor — torus major + minor radii
 * Output:  on hit, *t_hit = smallest positive t.
 * Returns: 1 on hit, 0 on miss.
 *
 * Algorithm:
 *   1. Compute the four quartic coefficients from the closed-form
 *      formulas (derived by substituting P = ro + t·rd into the torus
 *      equation and squaring to remove the radical).
 *   2. Scan q(t) at uniformly-spaced t-values starting from T_EPS.
 *      A sign change between consecutive samples brackets a root
 *      (intermediate value theorem applied to the continuous quartic).
 *   3. On the FIRST sign change, bisect inside that bracket Q_BISECT
 *      times to refine the root to ~10⁻¹² precision.
 *   4. Return that t — it's the smallest positive real root, hence
 *      the front-face hit.
 *
 * Why scan-bisect and not Ferrari's closed-form quartic formula?
 *   Ferrari is mathematically beautiful (substitute u = t + A/4 to
 *   eliminate the cubic term, then solve a resolvent cubic, then take
 *   square roots) but NUMERICALLY UNSTABLE. Tangent rays and near-
 *   tangent rays produce coefficients where Ferrari's discriminant
 *   passes near zero — small input errors blow up into large output
 *   errors. The scan-bisect approach is slower (~300 ops vs ~50 for
 *   Ferrari) but completely stable: it never divides by anything
 *   that could be zero.
 */
static int ray_torus(V3 ro, V3 rd, float R, float r_minor, float *t_hit)
{
    /* Pre-compute frequently-used dot products. */
    float po2    = v3dot(ro, ro);                       /* |ro|²        */
    float rod    = v3dot(rd, ro);                       /* rd · ro      */
    float rxz2   = ro.x*ro.x + ro.z*ro.z;               /* in-plane |ro|² */
    float rdxz_d = rd.x*ro.x + rd.z*ro.z;               /* in-plane rd·ro*/
    float rdxz2  = rd.x*rd.x + rd.z*rd.z;               /* in-plane |rd|²*/
    float C0     = po2 + R*R - r_minor*r_minor;         /* common offset */

    /* Quartic coefficients (see MENTAL MODEL → KEY FORMULAS). */
    float A = 4.f * rod;
    float B = 4.f*rod*rod + 2.f*C0 - 4.f*R*R*rdxz2;
    float C = 4.f*rod*C0  - 8.f*R*R*rdxz_d;
    float D = C0*C0       - 4.f*R*R*rxz2;

    /* Scan: evaluate q at uniformly-spaced t, look for sign changes. */
    float dt  = Q_T_MAX / (float)Q_SAMPLES;
    float t0  = T_EPS;
    float f0  = q_eval(t0, A, B, C, D);

    for (int i = 1; i <= Q_SAMPLES; i++) {
        float t1 = (float)i * dt;
        float f1 = q_eval(t1, A, B, C, D);

        if (f0 * f1 < 0.f) {
            /* Sign change → root in (t0, t1). Bisect to refine. */
            float lo = t0, hi = t1, flo = f0;
            for (int j = 0; j < Q_BISECT; j++) {
                float mid  = (lo + hi) * 0.5f;
                float fmid = q_eval(mid, A, B, C, D);
                if (flo * fmid < 0.f) {
                    hi = mid;                /* root in (lo, mid)       */
                } else {
                    lo = mid; flo = fmid;    /* root in (mid, hi)       */
                }
            }
            *t_hit = (lo + hi) * 0.5f;       /* midpoint of final bracket */
            return 1;
        }
        t0 = t1; f0 = f1;
    }
    return 0;                                /* no sign change → miss   */
}

/* §5.3 torus_normal — closest-point geometric formula ──────────────── */

/*
 * Outward surface normal at a hit point P on a torus in the XZ plane.
 *
 * Geometric construction:
 *   1. Project P onto the XZ plane: P_xz = (P.x, 0, P.z)
 *   2. The closest point on the ring centreline lies along the same
 *      direction at distance R from origin: ring_pt = (R / |P_xz|) · P_xz
 *   3. Outward normal is the unit vector from ring_pt to P:
 *      N = normalize(P − ring_pt)
 *
 * Why this and not the implicit gradient?
 *   The gradient of f = (√(x²+z²)−R)² + y² − r² involves a 1/√(x²+z²)
 *   term that's expensive (sqrt + divide) and unstable when |P_xz| is
 *   small (near the Y axis — although that can't happen on the torus
 *   surface, only for the hole). The closest-point formula is one
 *   normalize per pixel; cleaner.
 *
 * Edge case |P_xz| ≈ 0: P is on the Y axis, which is INSIDE the
 * torus's hole — not on the surface. Defensive default (R, 0, 0).
 */
static V3 torus_normal(V3 P, float R)
{
    V3    P_xz   = { P.x, 0.f, P.z };
    float rho    = v3len(P_xz);
    V3    ring_pt = (rho > 1e-9f)
                    ? v3scale(R / rho, P_xz)
                    : (V3){R, 0.f, 0.f};     /* defensive default */
    return v3norm(v3sub(P, ring_pt));
}

/* ── §6 shading ──────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_FRESNEL, MODE_DEPTH, MODE_N } ShadeMode;
static const char *const k_mode_names[] = { "phong","normals","fresnel","depth" };

/* Three fixed world-space lights — POSITIONS, not directions.
 * Per pixel we compute L = normalize(light_pos − P) so each light
 * direction depends on the hit point (point lights, not directional). */
static const V3 L_KEY  = { 3.0f, 4.0f, -2.0f };   /* upper-right, warm */
static const V3 L_FILL = {-4.0f, 1.5f, -1.0f };   /* upper-left,  cool */
static const V3 L_RIM  = { 0.5f,-1.0f,  5.0f };   /* behind,    accent */

/* §6.1 ── KEY light: dominant warm diffuse + sharp specular ─────────── */

static V3 light_key(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3    L = v3norm(v3sub(L_KEY, P));
    float d = fmaxf(0.f, v3dot(N, L));
    V3    R = v3reflect(v3scale(-1.f, L), N);
    float s = powf(fmaxf(0.f, v3dot(R, V_dir)), SHININESS);

    V3 diff = v3scale(d * 0.65f, v3mul(th->obj, th->key_col));
    V3 spec = v3scale(s * 0.55f, th->spec);
    return v3add(diff, spec);
}

/* §6.2 ── FILL light: soft cool diffuse, no specular ────────────────── */

static V3 light_fill(V3 P, V3 N, const Theme *th)
{
    V3    L = v3norm(v3sub(L_FILL, P));
    float d = fmaxf(0.f, v3dot(N, L));
    return v3scale(d * 0.22f, v3mul(th->obj, th->fill_col));
}

/* §6.3 ── RIM light: accent specular on the silhouette ──────────────── */

static V3 light_rim(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3    L = v3norm(v3sub(L_RIM, P));
    float d = fmaxf(0.f, v3dot(N, L));
    V3    R = v3reflect(v3scale(-1.f, L), N);
    float s = powf(fmaxf(0.f, v3dot(R, V_dir)), 10.f);

    V3 diff = v3scale(d * 0.18f, v3mul(th->obj, th->rim_col));
    V3 spec = v3scale(s * 0.65f, th->rim_col);
    return v3add(diff, spec);
}

/* §6.4 ── shade_phong: ambient + KEY + FILL + RIM ───────────────────── */

static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th)
{
    V3 col = v3scale(AMBIENT, th->obj);
    col = v3add(col, light_key (P, N, V_dir, th));
    col = v3add(col, light_fill(P, N,        th));
    col = v3add(col, light_rim (P, N, V_dir, th));
    return v3clamp1(col);
}

/* §6.5 ── shade_normal: RGB-encoded surface normal (diagnostic) ─────── */

/*
 * Each component remapped from [-1,+1] → [0,1]. For a torus, the
 * tube wraps around the ring so the normal points OUTWARD radially
 * at the outer edge (largest |x|+|z|), INWARD radially at the inner
 * edge (smallest |x|+|z|), and ±Y on top/bottom. NORMAL mode shows
 * a beautiful rainbow ring with two distinct colour belts (one on
 * each tube hemisphere).
 */
static V3 shade_normal(V3 N)
{
    return (V3){ N.x*.5f + .5f, N.y*.5f + .5f, N.z*.5f + .5f };
}

/* §6.6 ── shade_fresnel + shade_depth (alternative diagnostics) ─────── */

/*
 * Schlick approximation: F = F₀ + (1−F₀)(1−cosθ)^5. With F₀ = 0 (full
 * dielectric) this simplifies to (1−cosθ)^5 — dark head-on, bright
 * at grazing angles. The torus's curved tube produces a complex
 * pattern of bright fresnel bands at every silhouette edge (outer
 * and inner ring + top and bottom of the tube).
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

/* shade_depth — encode hit distance as brightness. */
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
 *   Forward way    :  rotate torus (the implicit-equation coefficients
 *                     would have to be recomputed in the rotated frame —
 *                     painful), fire ray in world space.
 *
 *   Backward way   :  keep torus FIXED in object space (ring in XZ
 *                     plane, centred at origin). Rotate the RAY into
 *                     object space by M^T = M⁻¹ (orthogonal matrix).
 *                     Cost: ONE matrix×ray per pixel.
 *
 * The quartic coefficients are clean ONLY in this fixed object-space
 * configuration. Rotating the torus would force every coefficient to
 * be recomputed with cross terms — orders of magnitude more code.
 */
static void render(int cols, int rows,
                   float angle_x, float angle_y, float cam_dist,
                   int theme_idx, ShadeMode mode)
{
    const Theme *th  = &g_themes[theme_idx % THEME_N];
    float fov_tan    = tanf(FOV_DEG * (float)M_PI / 360.f);

    Mat3 M           = mat3_rot(angle_x, angle_y);

    /* §7.1 — elevated camera looking at origin. */
    V3 cam = { 0.f, CAM_HEIGHT, -cam_dist };
    V3 fwd = v3norm(v3sub((V3){0,0,0}, cam));
    V3 wup = { 0.f, 1.f, 0.f };
    V3 rgt = v3norm(v3cross(fwd, wup));
    V3 up  = v3cross(rgt, fwd);

    float cx = cols * 0.5f, cy = rows * 0.5f;

    /* §7.2 — primary loop: one ray per cell. Skip bottom row for HUD. */
    for (int row = 0; row < rows - 1; row++) {
        for (int col = 0; col < cols; col++) {
            /* Normalised screen coords with terminal-cell aspect baked in. */
            float pu =  (col - cx) / cx * fov_tan;
            float pv = -(row - cy) / cx * fov_tan / ASPECT;

            V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu, rgt),
                                               v3scale(pv, up))));

            /* Transform ray to object space (torus fixed in XZ plane). */
            V3 ro_os = mat3_mulT(M, cam);
            V3 rd_os = mat3_mulT(M, rd_ws);

            float t_hit;
            if (!ray_torus(ro_os, rd_os, TORUS_R, TORUS_r, &t_hit))
                continue;

            /* Hit point in OBJECT and WORLD space; world-space normal. */
            V3 P_os  = v3add(ro_os, v3scale(t_hit, rd_os));
            V3 P_ws  = v3add(cam,   v3scale(t_hit, rd_ws));
            V3 N_os  = torus_normal(P_os, TORUS_R);
            V3 N_ws  = mat3_mul(M, N_os);
            V3 V_dir = v3norm(v3sub(cam, P_ws));

            V3    color;
            float lum;

            switch (mode) {
            default:
            case MODE_PHONG:
                color = shade_phong(P_ws, N_ws, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_NORMAL:
                color = shade_normal(N_ws);
                lum   = (N_ws.x*.5f+.5f)*.3f
                      + (N_ws.y*.5f+.5f)*.6f
                      + (N_ws.z*.5f+.5f)*.1f;
                break;
            case MODE_FRESNEL:
                color = shade_fresnel(N_ws, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_DEPTH:
                color = shade_depth(t_hit, cam_dist * 2.5f, th);
                lum   = rec601_luma(color);
                break;
            }

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
    snprintf(buf, sizeof buf, " %5.1f fps  dist:%.1f  %-9s  %s ",
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
             " q:quit  spc/p:pause  s:mode  t:theme  r:reset  +/-:zoom ");
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
            float sec = (float)dt * 1e-9f;
            angle_y += ROT_Y * sec;
            angle_x += ROT_X * sec;
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
        case 'r': case 'R':
            angle_x = 0.f; angle_y = 0.f; break;
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
