/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sphere_raytrace.c — analytic ray-traced sphere (the foundational demo)
 *
 * DEMO: A glossy sphere (gold by default) orbiting in space, lit by
 *       three coloured lights (warm key, cool fill, bright rim).
 *       Cycle through phong → normals → fresnel → depth to see how
 *       the same ray-sphere intersection feeds different shading
 *       models. The single primitive in the simplest possible setup
 *       — this is the "Hello World" of analytic ray tracing.
 *
 * Study alongside:
 *   raytracing/cube_raytrace.c     — same skeleton, slab method
 *   raytracing/capsule_raytrace.c  — same skeleton, decomposed analytic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *   raytracing/path_tracer.c       — same RGB-cube paint pipeline,
 *                                    multi-bounce Monte Carlo on top
 *
 * Section map:
 *   §1 config     — frame rate, FOV, sphere geometry, camera, ramp, HUD
 *   §2 clock      — monotonic timer + sleep
 *   §3 math       — V3 helpers
 *   §4 color      — themes + 256-colour cube + ASCII-ramp painter
 *   §5 sphere     — THE CORE: analytic ray-sphere quadratic
 *   §6 shading    — phong / normals / fresnel / depth
 *                   §6.1 KEY light  §6.2 FILL light  §6.3 RIM light
 *                   §6.4 phong glue §6.5 normal mode §6.6 fresnel/depth
 *   §7 render     — one frame: ray-per-cell, three-point lighting
 *   §8 screen     — ncurses init + HUD
 *   §9 app        — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   t         cycle theme (gold → ice → crimson → emerald → amethyst → neon)
 *   p / SPC   pause / resume orbit
 *   + / =     zoom in (orbit closer)
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/sphere_raytrace.c \
 *       -o sphere_rt -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-sphere intersection — exact, no marching
 *                  steps, no mesh, no acceleration structure. For a ray
 *                  ro + t·rd and a sphere at centre c with radius R, the
 *                  hit condition |ro + t·rd − c|² = R² expands to a
 *                  quadratic in t:
 *                    a·t² + b·t + c = 0
 *                  where (with rd unit-length and oc = ro − c):
 *                    a = 1
 *                    b = 2 (rd · oc)
 *                    c = |oc|² − R²
 *                  Discriminant = b² − 4ac. If < 0: ray misses entirely.
 *                  Else t = (−b ± √disc) / 2. Pick the nearest positive
 *                  root for the front-face hit.
 *
 *                  Surface normal at hit point P: N = (P − c) / R.
 *                  Three-point lighting (key/fill/rim) drives the
 *                  Phong shading model: kd · (N·L) + ks · (R·V)^n.
 *
 * Data-structure : NONE persistent. Each frame is a pure function of
 *                  (cols, rows, orbit_angle, cam_dist, theme, mode).
 *                  Per-frame: one camera basis. Themes are RGB
 *                  triplets per role (object / specular / per-light
 *                  tint). 216 ncurses pairs are pre-allocated as a
 *                  6×6×6 RGB cube; computed RGB ∈ [0,1]³ quantises
 *                  to one of those at draw time.
 *
 * Rendering      : One ray per terminal cell (no AA). RGB shaded
 *                  output → quantised to xterm 6×6×6 cube + Bourke
 *                  92-char density ramp. Both colour and glyph
 *                  density carry shading information.
 *
 * Performance    : Per pixel: one ray-sphere test (~10 multiplications),
 *                  three Phong light contributions (~50 multiplications
 *                  total), one cube quantise + ramp lookup. Trivially
 *                  fast — sphere is the cheapest analytic primitive.
 *                  240×80 cells × 60 fps ≈ 1 ms shading per frame.
 *
 * References     : Whitted, T. "An Improved Illumination Model for
 *                    Shaded Display," CACM 23(6), 1980. (Foundational
 *                    paper on analytic ray-tracing of spheres + Phong
 *                    lighting.)
 *                  Phong, B.T. "Illumination for Computer Generated
 *                    Pictures," CACM 18(6), 1975. (The shading model.)
 *                  Schlick, C. "An Inexpensive BRDF Model for
 *                    Physically-based Rendering," Comp. Graph. Forum
 *                    13(3), 1994. (The Fresnel approximation we use.)
 *                  Shirley, P. "Ray Tracing in One Weekend"
 *                    https://raytracing.github.io/books/RayTracingInOneWeekend.html
 *                  Inigo Quílez, "Sphere — intersection"
 *                    https://iquilezles.org/articles/intersectors/
 *                  Real-Time Rendering 4e §22.7 (ray-sphere variants).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To test if a ray hits a sphere, ask: "for what value of t along the
 * ray does the ray's distance to the sphere centre equal the sphere
 * radius?" Square both sides (so we don't deal with the square root)
 * and you get a QUADRATIC in t. Solve the quadratic. If real solutions
 * exist, the ray hits — pick the smallest positive one. That's it.
 * No marching, no iteration, no special cases (other than discriminant
 * < 0 = miss). The geometry of "where does a line meet a sphere" maps
 * exactly onto "solve a quadratic equation."
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the ray as a moving point. At time t = 0 it's at the camera;
 * at time t = ∞ it's gone forever. As t advances, the point traces a
 * straight line. We're asking: "between t = 0 and t = ∞, when does
 * this point sit exactly on the sphere's surface?"
 *
 *     ●─────────────────────●─── ray (at parameter t)
 *     ↑                     ↑
 *     ro                    P(t) = ro + t·rd
 *
 *           ╭────────╮
 *          ╱  sphere  ╲
 *         │            │
 *         │     ●c    │      ← centre c, radius R
 *          ╲          ╱
 *           ╰────────╯
 *
 * Distance from P(t) to c is |ro + t·rd − c|. Setting that equal to R
 * (and squaring to remove the sqrt) gives a quadratic. Geometric
 * interpretation:
 *
 *   t² + 2(rd · oc)·t + (|oc|² − R²) = 0     where oc = ro − c
 *      ╲___╱     ╲_________╱     ╲____________╱
 *       a            b/2                c
 *
 *   discriminant = b² − 4ac
 *      < 0  →  ray passes WIDE of the sphere — never hits.
 *      = 0  →  ray is exactly tangent to the sphere.
 *      > 0  →  two real roots: the entry and exit times.
 *
 * Once we have the hit point, the SURFACE NORMAL is just the unit
 * vector from the centre to the hit:
 *
 *   N = (P − c) / R
 *
 * Three coloured lights illuminate the sphere via the standard Phong
 * model: each light contributes a soft Lambertian (cosine of N·L) and
 * a sharp specular ((R·V)^n). The KEY light is the dominant warm
 * source; the FILL light lifts the shadow side with a cool tone;
 * the RIM light kisses the silhouette from behind for separation.
 *
 * DRAWING METHOD  /  ALGORITHM IN STEPS  (per pixel)
 * ──────────────────────────────────────────────────
 *  1. Build a primary ray from the orbiting camera into world space:
 *        rd = normalize( fwd + pu·right + pv·up )
 *     where (pu, pv) are screen coordinates scaled by tan(FOV/2)
 *     and aspect.
 *  2. Solve the ray-sphere quadratic (§5):
 *        b = rd · (ro − c)
 *        c = |ro − c|² − R²
 *        disc = b² − c
 *        if disc < 0: MISS, continue.
 *        t = -b - √disc   (front face)
 *  3. Compute the hit point P = ro + t·rd and surface normal
 *        N = (P − c) / R
 *  4. SHADE in continuous RGB:
 *        col = ambient + KEY + FILL + RIM
 *     Each light contributes diffuse (and KEY/RIM also specular):
 *        L = normalize(light_pos − P)
 *        d = max(0, N · L)                         ← Lambertian
 *        R = reflect(−L, N) = 2(N·L)N − L          ← reflection of L
 *        s = max(0, R · V_dir)^shininess           ← Phong specular
 *  5. Quantise → 6×6×6 cube pair + 92-char density ramp + A_BOLD/A_DIM.
 *
 * KEY FORMULAS
 * ────────────
 *   Ray-sphere quadratic (rd unit, oc = ro − c):
 *     b    = rd · oc
 *     c    = oc · oc − R²
 *     disc = b² − c
 *     t    = -b − √disc      (front face; back face is -b + √disc)
 *
 *   Surface normal (sphere centred at c):
 *     N = (P − c) / R         and |N| = 1 by construction
 *
 *   Phong shading per light:
 *     L    = normalize(light_pos − P)
 *     R    = 2(N·L)·N − L
 *     I    = albedo · max(0, N·L) · light_col      ← diffuse
 *          + spec_col · max(0, R·V_dir)^shininess  ← specular
 *
 *   Schlick Fresnel (used by MODE_FRESNEL):
 *     cosθ = |N · V_dir|
 *     F    = (1 − cosθ)^5     ← assumes F₀ = 0 (full dielectric)
 *
 * WORKED EXAMPLE  (verify by hand)
 * ────────────────────────────────
 *   Sphere at origin, R = 1. Camera at (0, 0.55, -3.6) (orbit_ang = 0,
 *   cam_dist = 3.6). Centre cell of screen, so pu = pv = 0, rd ≈ fwd.
 *
 *   fwd = normalize((0,0,0) − (0, 0.55, -3.6))
 *       = normalize((0, -0.55, 3.6))
 *       ≈ (0, -0.151, 0.989)
 *
 *   Ray-sphere:
 *     oc   = ro − 0 = (0, 0.55, -3.6)
 *     b    = rd · oc = 0·0 + (-0.151)·0.55 + 0.989·(-3.6)
 *          = -0.0830 − 3.560 ≈ -3.643
 *     c    = oc·oc − R² = (0 + 0.3025 + 12.960) − 1 = 12.262
 *     disc = b² − c = 13.272 − 12.262 = 1.010
 *     t    = -b − √disc = 3.643 − 1.005 = 2.638
 *
 *   Hit point:
 *     P = ro + t·rd = (0, 0.55 − 0.398, -3.6 + 2.609)
 *       = (0, 0.152, -0.991)
 *     |P| ≈ 1.003 (≈1.0; small rounding ok)
 *     N = P / 1 ≈ (0, 0.151, -0.987)        ← faces toward camera
 *
 *   KEY light at (3, 4, -2):
 *     L_dir = (3, 4-0.152, -2-(-0.991)) = (3, 3.848, -1.009)
 *     |L_dir| = √(9 + 14.81 + 1.02) ≈ 4.983
 *     L = (0.602, 0.772, -0.202)
 *     N·L = 0 + 0.117 + 0.199 = 0.316
 *     diffuse = 0.316     ← hit is partially lit by KEY
 *
 *   For the GOLD theme (obj = 0.90, 0.72, 0.18):
 *     key_diff_RGB = 0.316 · 0.65 · (obj · key_col)
 *                  = 0.205 · (0.90, 0.662, 0.126)
 *                  ≈ (0.185, 0.136, 0.026)
 *
 *   With ambient (0.04 · obj) + KEY diffuse + FILL/RIM contributions
 *   and specular, the final RGB at this cell is roughly (0.40, 0.32,
 *   0.08) — a warm gold with slight specular pop. Reinhard tone-map
 *   keeps it inside [0,1], gamma encodes, quantises to a cube cell
 *   in the warm-yellow region, and density ≈ 0.32 → mid ramp glyph.
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DISCRIMINANT NEAR ZERO. The ray is nearly tangent. t = -b ± √disc
 *    are very close to each other; rounding can flip the sign. The
 *    1e-4 epsilon in `t1 < 1e-4f` check rejects effectively-zero
 *    distances and avoids self-intersection.
 *  • CAMERA INSIDE SPHERE. Then c = |oc|² − R² < 0, and disc = b² − c
 *    > 0 always, so we get a hit. -b ± √disc gives one negative and
 *    one positive root; we want the positive one (the exit face).
 *    The code handles this by trying t0 first, falling back to t1.
 *  • RAY DIRECTION NOT UNIT-LENGTH. The quadratic relies on |rd| = 1.
 *    If you pass a non-unit vector the formula needs `a = rd·rd` term.
 *    We always normalise rd in render(), so this is safe.
 *  • PURE DIFFUSE LIMIT. As shininess → 0, (R·V)^n → 1 everywhere
 *    and the sphere looks flat-shaded. As shininess → ∞ the highlight
 *    becomes a single bright pixel. SHININESS = 52 gives a tight
 *    plastic-like specular.
 *  • SPECULAR ON DARK SIDE. We multiply specular by max(0, N·L) — no,
 *    we don't always; KEY's specular is s alone (not gated by d). On
 *    the back of the sphere (N·L < 0), the reflection vector still
 *    has a positive R·V at certain angles, producing a "ghost" highlight.
 *    Look at the unlit hemisphere with very shiny shaders to see this
 *    artifact. For more physically correct shading, gate specular by
 *    (N·L > 0). We don't here — it's the simplest Phong model on
 *    purpose, to keep the file small.
 *  • TONE-MAP AT PAINT. Quantising linear HDR puts everything in one
 *    cube cell because the cube is uniform [0,1]³. Tone-mapping
 *    inside paint_cell opens the dynamic range first.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Static sphere (paused, orbit_ang = 0): gold sphere, warm KEY
 *    light from the upper-right, soft cool FILL on the left, bright
 *    RIM from behind catching the silhouette. The brightest specular
 *    spot should be on the upper-right surface.
 *  • MODE_NORMAL: each surface direction gets a unique RGB colour.
 *    The +Z hemisphere (toward camera at orbit_ang=0) is mostly blue
 *    (0.5, 0.5, 1.0) at centre fading to red/green at silhouette.
 *  • MODE_FRESNEL: dark in the middle (head-on, cosθ ≈ 1 → F ≈ 0),
 *    bright at the silhouette (grazing, cosθ ≈ 0 → F ≈ 1). The
 *    classic glass-marble look.
 *  • MODE_DEPTH: closer cells brighter, farther cells darker. The
 *    silhouette should be the dimmest part (largest t).
 *  • Worked-example check: at orbit_ang=0, cam_dist=3.6, the centre-
 *    cell ray hits the sphere at t ≈ 2.638, normal ≈ (0, 0.15, -0.99).
 *    Inspect the source if your sphere appears in the wrong place.
 *  • Cycle THEME (t): the obj/specular/light tints all change while
 *    the geometry stays fixed.
 *  • Zoom (+/-): smaller cam_dist → bigger sphere on screen, brighter
 *    edges (RIM gets closer). Larger → smaller sphere, dimmer overall.
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
#define FOV_DEG       58.0f                /* full vertical-equivalent FOV  */

/* §1.3 sphere (object space, centred at origin) */
#define SPHERE_R      1.0f                 /* unit-radius sphere            */

/* §1.4 orbit (camera) — orbits the sphere at fixed distance + height */
#define ORBIT_SPEED   0.32f                /* radians / second              */
#define CAM_HEIGHT    0.55f                /* camera elevation above equator*/
#define CAM_DIST_DEF  3.6f
#define CAM_DIST_MIN  1.9f
#define CAM_DIST_MAX  7.0f
#define CAM_DIST_STEP 0.25f

/* §1.5 shading */
#define AMBIENT       0.04f                /* fraction of obj as ambient    */
#define SHININESS     52.0f                /* phong exponent for KEY        */

/* §1.6 character ramp — Paul Bourke 92-char density ladder.
 * Index 0 (space) is invisible; index N−1 ('@') is densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN  ((int)(sizeof k_ramp - 1))

/* §1.7 ncurses pair IDs (256-colour cube + reserved HUD/HINT) */
#define PAIR_CUBE_BASE   1                 /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD       217
#define PAIR_HINT      218

/* §1.8 epsilon for ray distances */
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

/* ── §3 math (V3) ────────────────────────────────────────────────────── */

typedef struct { float x, y, z; } V3;

static inline V3    v3add   (V3 a, V3 b)    { return (V3){a.x+b.x, a.y+b.y, a.z+b.z}; }
static inline V3    v3sub   (V3 a, V3 b)    { return (V3){a.x-b.x, a.y-b.y, a.z-b.z}; }
static inline V3    v3scale (float s, V3 a) { return (V3){s*a.x, s*a.y, s*a.z};      }
static inline V3    v3mul   (V3 a, V3 b)    { return (V3){a.x*b.x, a.y*b.y, a.z*b.z}; }
static inline float v3dot   (V3 a, V3 b)    { return a.x*b.x + a.y*b.y + a.z*b.z;     }
static inline float v3len   (V3 a)          { return sqrtf(v3dot(a, a));              }
static inline V3    v3norm  (V3 a)          { float l=v3len(a); return l>1e-9f ? v3scale(1.f/l, a) : (V3){0,1,0}; }
static inline V3    v3cross (V3 a, V3 b)    { return (V3){a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x}; }
static inline V3    v3reflect(V3 v, V3 n)   { return v3sub(v, v3scale(2.f*v3dot(v,n), n)); }
static inline V3    v3clamp1(V3 v)
{
    return (V3){ v.x<0?0:v.x>1?1:v.x, v.y<0?0:v.y>1?1:v.y, v.z<0?0:v.z>1?1:v.z };
}

/* ── §4 color / themes ───────────────────────────────────────────────── */

/*
 * Theme — five colour vectors per palette:
 *   obj         base albedo of the sphere
 *   spec        specular highlight tint (KEY + RIM)
 *   key/fill/rim three-light tint (warm·cool·accent)
 *
 * The Phong shader in §6 multiplies obj by the light tint for diffuse
 * contributions, and uses spec/rim_col directly for specular highlights.
 */
typedef struct {
    V3 obj, spec, key_col, fill_col, rim_col;
    const char *name;
} Theme;

static const Theme g_themes[] = {
    /* gold — warm metallic yellow */
    {{0.90f,0.72f,0.18f},{1.00f,0.95f,0.65f},
     {1.00f,0.92f,0.70f},{0.25f,0.35f,0.80f},{0.95f,0.55f,0.10f},"gold"},
    /* ice — cold pale blue */
    {{0.50f,0.78f,1.00f},{0.85f,0.93f,1.00f},
     {0.75f,0.88f,1.00f},{0.15f,0.25f,0.75f},{0.45f,0.80f,1.00f},"ice"},
    /* crimson — saturated red */
    {{0.88f,0.12f,0.12f},{1.00f,0.75f,0.60f},
     {1.00f,0.82f,0.65f},{0.15f,0.08f,0.50f},{1.00f,0.35f,0.08f},"crimson"},
    /* emerald — rich green */
    {{0.08f,0.78f,0.28f},{0.60f,1.00f,0.70f},
     {0.75f,1.00f,0.60f},{0.08f,0.35f,0.60f},{0.15f,0.90f,0.35f},"emerald"},
    /* amethyst — neon purple */
    {{0.62f,0.18f,0.88f},{0.85f,0.65f,1.00f},
     {0.88f,0.78f,1.00f},{0.08f,0.08f,0.60f},{0.78f,0.18f,0.88f},"amethyst"},
    /* neon — cyan/yellow electric */
    {{0.08f,0.92f,0.88f},{0.70f,1.00f,0.95f},
     {0.88f,0.92f,0.50f},{0.08f,0.38f,0.80f},{0.18f,1.00f,0.75f},"neon"},
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
 *   - colour gives "what it is" (warm gold vs cool ice)
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

/* ── §5 ray-sphere intersection (THE CORE) ───────────────────────────── */

/*
 * ray_sphere — analytic ray vs sphere intersection.
 *
 * Sphere is centred at the ORIGIN with radius r (so we don't pass a
 * centre vector — saves an argument). Ray is (ro, rd) with rd unit-
 * length.
 *
 * Algebra:
 *   |ro + t·rd|² = r²
 *   (ro + t·rd) · (ro + t·rd) = r²
 *   |rd|²·t² + 2(rd·ro)·t + (|ro|² − r²) = 0
 * With |rd| = 1:
 *   t² + 2(rd·ro)·t + (|ro|² − r²) = 0
 *
 * Standard quadratic with a=1, half-b = rd·ro, c = |ro|²−r². The
 * "half-b" form (using b = rd·ro, not 2·rd·ro) lets us write
 *   disc = b² − c        (instead of b²/4 − c)
 *   t    = -b ± √disc    (instead of (-b ± √disc) / 2)
 * — a small saving of two divisions per pixel.
 *
 * Returns 1 on hit; *t_hit gets the nearest t > T_EPS. The T_EPS guard
 * rejects t-values close to zero, preventing self-intersection at the
 * surface and numerical instability when the ray grazes the sphere
 * (disc ≈ 0 case where t0 and t1 are nearly equal).
 *
 * Equivalent to ray_quad / ray_aabb in the other primitive files —
 * each is "the analytic solver for ONE primitive shape." Sphere is
 * the simplest such solver in the entire raytracing folder.
 */
static int ray_sphere(V3 ro, V3 rd, float r, float *t_hit)
{
    float b    = v3dot(rd, ro);
    float c    = v3dot(ro, ro) - r * r;
    float disc = b * b - c;
    if (disc < 0.f) return 0;                /* ray misses sphere       */

    float sq = sqrtf(disc);
    float t0 = -b - sq;                      /* near root (front face) */
    float t1 = -b + sq;                      /* far root  (back face)  */

    /* Back face beyond camera = whole sphere is behind us → miss. */
    if (t1 < T_EPS) return 0;

    /* Prefer front face; fall back to back face if camera is INSIDE
     * the sphere (then t0 < 0 < t1 — exit through the far wall). */
    *t_hit = (t0 > T_EPS) ? t0 : t1;
    return 1;
}

/* ── §6 shading ──────────────────────────────────────────────────────── */

typedef enum { MODE_PHONG=0, MODE_NORMAL, MODE_FRESNEL, MODE_DEPTH, MODE_N } ShadeMode;
static const char *const k_mode_names[] = { "phong","normals","fresnel","depth" };

/* Three fixed world-space lights — POSITIONS, not directions.
 * Per pixel we compute L = normalize(light_pos − P) so each light
 * direction depends on the hit point (point lights, not directional). */
static const V3 L_KEY  = { 3.0f, 4.0f, -2.0f };   /* upper-right, warm     */
static const V3 L_FILL = {-4.0f, 1.0f, -1.0f };   /* upper-left,  cool     */
static const V3 L_RIM  = { 0.5f,-1.0f,  5.0f };   /* behind,      accent   */

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

/*
 * The FILL light is intentionally diffuse-only. Its job is to LIFT
 * the unlit hemisphere from pure shadow into a soft cool tone. Adding
 * specular here would produce a second highlight that competes with
 * the KEY's, breaking the three-point-lighting visual hierarchy.
 */
static V3 light_fill(V3 P, V3 N, const Theme *th)
{
    V3    L = v3norm(v3sub(L_FILL, P));
    float d = fmaxf(0.f, v3dot(N, L));
    return v3scale(d * 0.22f, v3mul(th->obj, th->fill_col));
}

/* §6.3 ── RIM light: accent specular on the silhouette ──────────────── */

/*
 * The RIM light sits BEHIND the sphere relative to the camera. Its
 * specular component (with a low shininess of 10) creates a wide
 * brilliant glow on the silhouette edge — the "rim" effect that makes
 * the sphere read as separated from the background.
 */
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
 * Each component remapped from [-1,+1] → [0,1] so all three channels
 * are valid colours. This is the canonical "normal visualisation"
 * across the entire raytracing folder — useful to verify that the
 * sphere's normals point correctly outward at every pixel.
 *
 * For the sphere centred at origin, N at the +Z silhouette is (0,0,1)
 * → RGB (0.5, 0.5, 1.0) (blue). The +X side → red-ish. The +Y top →
 * green-ish. A rotating sphere in this mode shows a familiar
 * "rainbow ball" identifying every direction by its colour.
 */
static V3 shade_normal(V3 N)
{
    return (V3){ N.x*.5f + .5f, N.y*.5f + .5f, N.z*.5f + .5f };
}

/* §6.6 ── shade_fresnel + shade_depth (alternative diagnostics) ─────── */

/*
 * Schlick approximation: F = F₀ + (1−F₀)(1−cosθ)^5. With F₀ = 0 (full
 * dielectric, like air↔glass), this simplifies to (1−cosθ)^5. Result
 * is dark head-on (cosθ ≈ 1 → F ≈ 0) and bright at grazing angles
 * (cosθ → 0, F → 1) — the classic "glass marble" look.
 *
 * The blend parameter `fresnel` mixes a dim core colour with a bright
 * edge colour. Increasing F₀ would brighten the head-on view (more
 * metallic).
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

/*
 * shade_depth — encode hit distance as brightness (closer = bright,
 * farther = dim). d² (rather than d) gives a steeper falloff so the
 * silhouette pops as obviously "at the back."
 */
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
 * The orbiting-camera trick:
 *
 *   Instead of rotating the sphere, we orbit the CAMERA around the
 *   sphere at a fixed distance. This is mathematically equivalent and
 *   keeps the sphere's intersection math trivial: the sphere stays at
 *   the origin so ray_sphere() never needs a centre argument.
 *
 *   cam = (cam_dist · sin(θ),  CAM_HEIGHT,  -cam_dist · cos(θ))
 *
 *   At θ=0 the camera sits at (0, h, -d) — looking toward +Z.
 *   At θ=π/2 the camera is at (d, h, 0) — looking toward -X.
 *   The camera basis (fwd, right, up) is rebuilt each frame from the
 *   look direction toward the origin.
 */
static void render(int cols, int rows,
                   float orbit_ang, float cam_dist,
                   int theme_idx, ShadeMode mode)
{
    const Theme *th  = &g_themes[theme_idx % THEME_N];
    float fov_tan    = tanf(FOV_DEG * (float)M_PI / 360.f);

    /* §7.1 — orbiting camera + look-at basis. */
    V3 cam = { cam_dist * sinf(orbit_ang), CAM_HEIGHT, -cam_dist * cosf(orbit_ang) };
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

            V3 rd = v3norm(v3add(fwd, v3add(v3scale(pu, rgt),
                                            v3scale(pv, up))));

            float t_hit;
            if (!ray_sphere(cam, rd, SPHERE_R, &t_hit)) continue;

            V3 P     = v3add(cam, v3scale(t_hit, rd));
            V3 N     = v3norm(P);                 /* sphere centred at origin */
            V3 V_dir = v3norm(v3sub(cam, P));

            V3    color;
            float lum;

            switch (mode) {
            default:
            case MODE_PHONG:
                color = shade_phong(P, N, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_NORMAL:
                color = shade_normal(N);
                /* Green-weighted luma in NORMAL mode tracks the
                 * "green is brightest" intuition the eye applies to
                 * RGB normal visualisations. */
                lum   = (N.x*.5f+.5f)*.3f + (N.y*.5f+.5f)*.6f + (N.z*.5f+.5f)*.1f;
                break;
            case MODE_FRESNEL:
                color = shade_fresnel(N, V_dir, th);
                lum   = rec601_luma(color);
                break;
            case MODE_DEPTH:
                color = shade_depth(t_hit, cam_dist * 2.2f, th);
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
    float     orbit_ang = 0.f;
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

        /* §9.3 advance orbit angle if not paused. */
        if (!paused) orbit_ang += ORBIT_SPEED * (float)dt * 1e-9f;

        /* §9.4 fps rolling average over half-second windows. */
        fps_acc += dt; fps_cnt++;
        if (fps_acc >= 500000000LL) {
            fps     = (float)fps_cnt * 1e9f / (float)fps_acc;
            fps_acc = 0; fps_cnt = 0;
        }

        /* §9.5 paint frame. */
        long long t0 = clock_ns();
        erase();
        render(cols, rows, orbit_ang, cam_dist, theme_idx, mode);
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
