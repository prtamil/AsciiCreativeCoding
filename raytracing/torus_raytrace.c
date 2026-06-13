/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

#define _POSIX_C_SOURCE 199309L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 CONFIG — constants (data only)
 * ───────────────────────────────────────────────────────── */

/* §1.1 frame rate */
#define TARGET_FPS 60
#define DT_CAP_NS 100000000LL /* 0.1 sec — spiral-of-death cap */

/* §1.2 view geometry */
#define ASPECT 0.47f  /* terminal cell W/H ratio       */
#define FOV_DEG 52.0f /* full vertical-equivalent FOV  */

/* §1.3 torus (object space, in XZ plane, centred at origin)
 *
 *   R    major radius — distance from origin to ring centreline
 *   r    minor radius — tube cross-section radius
 *
 * The ratio r/R determines tube fatness. r/R = 0.41 is a fairly
 * chunky donut; 0.20 would be a slender ring; 0.50 a fat tire. */
#define TORUS_R 0.68f
#define TORUS_r 0.28f

/* §1.4 rotation rates (rad/sec) */
#define ROT_Y 0.40f /* primary spin around Y         */
#define ROT_X 0.18f /* slow tilt around X            */

/* §1.5 camera (orbits along −Z, elevated to see hole + tube) */
#define CAM_DIST_DEF 3.4f
#define CAM_DIST_MIN 1.6f
#define CAM_DIST_MAX 7.0f
#define CAM_DIST_STEP 0.25f
#define CAM_HEIGHT 1.8f /* elevation above equator       */

/* §1.6 shading */
#define AMBIENT 0.20f   /* dim copy of albedo as ambient */
#define SHININESS 75.0f /* phong exponent — high = metal */
/* three-point lighting recipe (gains per light; see §6.1 shade_phong) */
#define KEY_DIFFUSE 1.00f       /* KEY diffuse gain — primary illumination  */
#define KEY_SPEC_GAIN 1.30f     /* KEY specular gain — hard highlight       */
#define FILL_DIFFUSE 0.55f      /* FILL diffuse gain — lifts the shadow side*/
#define RIM_DIFFUSE 0.40f       /* RIM diffuse gain — back-edge glow        */
#define RIM_SPEC_GAIN 1.20f     /* RIM specular gain — silhouette kiss      */
#define RIM_SHININESS 10.0f     /* RIM Phong exponent — wide cone (vs 75)   */
#define DEPTH_FAR_SCALE 2.5f    /* depth mode: t_max = cam_dist * this      */
#define FRESNEL_CORE 0.06f      /* fresnel head-on dim tint of albedo       */
#define FRESNEL_EDGE_GAIN 1.10f /* fresnel grazing-edge specular gain       */

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
#define Q_SAMPLES 256
#define Q_BISECT 40
#define Q_T_MAX 18.0f

/* §1.8 character ramp — Paul Bourke 92-char density ladder. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.9 ncurses pair IDs (256-colour cube + reserved HUD/HINT) */
#define PAIR_CUBE_BASE 1 /* + 0..215 = 6×6×6 cube       */
#define PAIR_HUD 217
#define PAIR_HINT 218

/* §1.10 epsilon */
#define T_EPS 1e-3f /* lower bound for t scan      */

/* ── §2 PERFORMANCE — clock
 * ────────────────────────────────────────────────────────── */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3 LOGIC — math (V3, Mat3), pure
 * ──────────────────────────────────────────────── */

/*
 * V3 — a 3-component single-precision vector, the one arithmetic type the
 * whole ray tracer is built on. It wears three hats, all sharing x/y/z:
 *   • a POSITION  (camera eye, hit point P, light position) in world space
 *   • a DIRECTION (ray rd, surface normal N, view vector V_dir) — kept unit
 *     length by v3norm wherever the math assumes |v| = 1
 *   • a COLOUR    (albedo / specular / emissive / per-pixel RGB), each
 *     channel a linear-light intensity, conventionally in [0, 1]
 * Reusing one type is deliberate: the same dot / scale / add that move points
 * also blend colours, so the helpers below serve geometry and shading alike.
 * Ref: Shirley, "Ray Tracing in One Weekend" (vec3); Foley & van Dam (vectors).
 */
typedef struct {
  float x, y, z; /* position / direction / linear RGB — see above */
} V3;

static inline V3 v3add(V3 a, V3 b) {
  return (V3){a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline V3 v3sub(V3 a, V3 b) {
  return (V3){a.x - b.x, a.y - b.y, a.z - b.z};
}
static inline V3 v3scale(float s, V3 a) {
  return (V3){s * a.x, s * a.y, s * a.z};
}
static inline float v3dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3len(V3 a) { return sqrtf(v3dot(a, a)); }
static inline V3 v3norm(V3 a) {
  float l = v3len(a);
  return l > 1e-9f ? v3scale(1.f / l, a) : (V3){0, 1, 0};
}
static inline V3 v3cross(V3 a, V3 b) {
  return (V3){a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
              a.x * b.y - a.y * b.x};
}
static inline V3 v3reflect(V3 v, V3 n) {
  return v3sub(v, v3scale(2.f * v3dot(v, n), n));
}
static inline V3 v3clamp1(V3 v) {
  return (V3){v.x < 0   ? 0
              : v.x > 1 ? 1
                        : v.x,
              v.y < 0   ? 0
              : v.y > 1 ? 1
                        : v.y,
              v.z < 0   ? 0
              : v.z > 1 ? 1
                        : v.z};
}

/*
 * Mat3 — a 3×3 rotation matrix, the torus's orientation made concrete. Rows are
 * stored as V3 so a matrix×vector is three v3dot calls (ergonomic,
 * cache-tight). Built once per frame by mat3_rot from the tumble angles, then
 * applied two ways: mat3_mul rotates object→world (the surface normal);
 * mat3_mulT rotates world→object (the rays). Because a rotation matrix is
 * ORTHOGONAL its inverse equals its transpose (M⁻¹ = Mᵀ) — that identity is the
 * whole inverse-rotation trick in §7: leave the torus fixed in object space and
 * rotate the rays into it, instead of recomputing the quartic in a rotated
 * frame. Ref: orthogonal-matrix inverse = transpose; rotation matrices (Foley &
 * van Dam).
 */
typedef struct {
  V3 r[3]; /* the three rows of the rotation matrix (stored for v3dot) */
} Mat3;

/*
 * mat3_rot — composed rotation Rx(rx) · Ry(ry).
 *
 * This is the OBJECT→WORLD transform (mat3_mul). For ray tracing we
 * need the OPPOSITE direction (rotate WORLD ray into the torus's
 * object space, where the ring stays in the XZ plane). For a pure
 * rotation the inverse equals the transpose, so mat3_mulT below is
 * the inverse — cheap, no determinant, no division.
 */
static Mat3 mat3_rot(float rx, float ry) {
  float cx = cosf(rx), sx = sinf(rx);
  float cy = cosf(ry), sy = sinf(ry);
  Mat3 m;
  m.r[0] = (V3){cy, 0.f, sy};
  m.r[1] = (V3){sx * sy, cx, -sx * cy};
  m.r[2] = (V3){-cx * sy, sx, cx * cy};
  return m;
}

/* M · v   — object → world (forward) */
static V3 mat3_mul(Mat3 m, V3 v) {
  return (V3){v3dot(m.r[0], v), v3dot(m.r[1], v), v3dot(m.r[2], v)};
}

/* M^T · v — world → object (inverse, since M is orthogonal) */
static V3 mat3_mulT(Mat3 m, V3 v) {
  return (V3){m.r[0].x * v.x + m.r[1].x * v.y + m.r[2].x * v.z,
              m.r[0].y * v.x + m.r[1].y * v.y + m.r[2].y * v.z,
              m.r[0].z * v.x + m.r[1].z * v.y + m.r[2].z * v.z};
}

/* ── §4 RENDER + DATA — color / themes
 * ───────────────────────────────────────────────── */

/*
 * Theme — PBR-flavoured material descriptor (matches cube_raytrace.c
 * and sphere_raytrace.c so all three primitives share one material
 * vocabulary).
 *
 * First-principles redesign: the LIGHTS are pure WHITE, and each
 * material's distinctive look comes entirely from its own properties.
 * Under white light, gold looks like gold and blue plastic looks like
 * blue plastic — the colour is intrinsic to the material, not painted
 * on by a tinted light source.
 *
 * Fields:
 *   albedo          body diffuse colour (what the material LOOKS LIKE
 *                   under uniform white illumination)
 *   specular        F0 — Fresnel reflectance at normal incidence:
 *                     METALS:      tinted to match albedo (gold spec
 *                                  is warm yellow because gold reflects
 *                                  yellow at the highlight)
 *                     DIELECTRICS: near-white (~4% achromatic
 *                                  reflectance for non-conductors)
 *   emissive        self-luminance, added AFTER lighting — visible
 *                   even in shadow. Mostly (0,0,0); used for neon.
 *   diffuse_weight  scales the diffuse contribution from albedo:
 *                     metals ≈ 0.15, gems ≈ 0.70, plastic/ceramic ≈ 0.85,
 *                     glass ≈ 0.10, neon ≈ 0.20.
 *
 * The 20 themes below are organised into 4 families. Switching themes
 * with `t / T` cycles through all 20 in order.
 *
 * Ref: physically-based rendering metal/dielectric workflow — F0 (Fresnel
 * reflectance at normal incidence) and the albedo/specular split (Cook &
 * Torrance 1982; Karis 2013, "Real Shading in Unreal Engine 4"). The Phong
 * specular in §6 is a cheap stand-in for a full microfacet BRDF.
 */
typedef struct {
  V3 albedo;            /* body diffuse colour                      */
  V3 specular;          /* F0 — metal: matches albedo; die: white   */
  V3 emissive;          /* self-glow (added after lighting)         */
  float diffuse_weight; /* 0.10..0.90 — metal/dielectric scale      */
  const char *name;     /* HUD label, e.g. "gold" / "sapphire"      */
} Theme;

static const Theme g_themes[] = {
    /* === METALS (12) — spec hue MATCHES albedo, low diffuse_weight ===
     * Real metals reflect nearly all incident light specularly. Their
     * F0 (Fresnel at normal incidence) is what tints the highlight. */

    /* gold     — warm yellow precious metal                            */
    {{1.00f, 0.77f, 0.34f},
     {1.00f, 0.77f, 0.34f},
     {0.f, 0.f, 0.f},
     0.15f,
     "gold"},
    /* silver   — bright cool precious metal, near-pure white           */
    {{0.97f, 0.96f, 0.92f},
     {0.97f, 0.96f, 0.92f},
     {0.f, 0.f, 0.f},
     0.15f,
     "silver"},
    /* copper   — warm orange-red metal                                 */
    {{0.96f, 0.64f, 0.54f},
     {0.96f, 0.64f, 0.54f},
     {0.f, 0.f, 0.f},
     0.15f,
     "copper"},
    /* bronze   — warm brown alloy (Cu+Sn)                              */
    {{0.78f, 0.55f, 0.30f},
     {0.78f, 0.55f, 0.30f},
     {0.f, 0.f, 0.f},
     0.15f,
     "bronze"},
    /* brass    — yellow-green alloy (Cu+Zn)                            */
    {{0.85f, 0.70f, 0.25f},
     {0.85f, 0.70f, 0.25f},
     {0.f, 0.f, 0.f},
     0.15f,
     "brass"},
    /* platinum — cool greyish-white precious metal                     */
    {{0.83f, 0.81f, 0.78f},
     {0.83f, 0.81f, 0.78f},
     {0.f, 0.f, 0.f},
     0.15f,
     "platinum"},
    /* titanium — dark silvery metal                                    */
    {{0.62f, 0.60f, 0.55f},
     {0.62f, 0.60f, 0.55f},
     {0.f, 0.f, 0.f},
     0.15f,
     "titanium"},
    /* iron     — neutral grey base metal                               */
    {{0.56f, 0.57f, 0.58f},
     {0.56f, 0.57f, 0.58f},
     {0.f, 0.f, 0.f},
     0.15f,
     "iron"},
    /* steel    — cool blue-grey alloy                                  */
    {{0.65f, 0.70f, 0.78f},
     {0.65f, 0.70f, 0.78f},
     {0.f, 0.f, 0.f},
     0.15f,
     "steel"},
    /* chrome   — mirror-bright cool metal                              */
    {{0.92f, 0.94f, 0.96f},
     {0.92f, 0.94f, 0.96f},
     {0.f, 0.f, 0.f},
     0.15f,
     "chrome"},
    /* mercury  — liquid silver                                         */
    {{0.85f, 0.85f, 0.88f},
     {1.00f, 1.00f, 1.00f},
     {0.f, 0.f, 0.f},
     0.15f,
     "mercury"},
    /* aluminum — pale neutral metal                                    */
    {{0.91f, 0.92f, 0.92f},
     {0.91f, 0.92f, 0.92f},
     {0.f, 0.f, 0.f},
     0.15f,
     "aluminum"},

    /* === GEMS (4) — saturated body + WHITE spec, mid diffuse_weight =
     * Gems are dielectrics; their Fresnel reflectance is achromatic.
     * Body colour comes from absorption inside the crystal. */

    /* ruby     — red corundum (Cr-doped)                               */
    {{0.85f, 0.10f, 0.18f},
     {1.00f, 0.95f, 0.95f},
     {0.f, 0.f, 0.f},
     0.70f,
     "ruby"},
    /* emerald  — green beryl (Cr-doped)                                */
    {{0.10f, 0.70f, 0.30f},
     {0.95f, 1.00f, 0.95f},
     {0.f, 0.f, 0.f},
     0.70f,
     "emerald"},
    /* sapphire — blue corundum (Fe/Ti-doped)                           */
    {{0.10f, 0.30f, 0.88f},
     {0.95f, 0.95f, 1.00f},
     {0.f, 0.f, 0.f},
     0.70f,
     "sapphire"},
    /* amethyst — purple quartz                                         */
    {{0.55f, 0.30f, 0.85f},
     {1.00f, 0.95f, 1.00f},
     {0.f, 0.f, 0.f},
     0.70f,
     "amethyst"},

    /* === DIELECTRICS (3) — body colour + WHITE spec ===================
     * Plastics, ceramics, and glass. F0 is achromatic (~4%); body
     * colour comes from sub-surface absorption. */

    /* plastic  — saturated blue plastic, full body colour              */
    {{0.20f, 0.40f, 0.92f},
     {1.00f, 1.00f, 1.00f},
     {0.f, 0.f, 0.f},
     0.85f,
     "plastic"},
    /* glass    — dark base + bright spec fakes transparency            */
    {{0.10f, 0.12f, 0.16f},
     {1.00f, 1.00f, 1.00f},
     {0.f, 0.f, 0.f},
     0.10f,
     "glass"},
    /* ceramic  — soft warm-cream porcelain                             */
    {{0.92f, 0.90f, 0.85f},
     {1.00f, 0.98f, 0.95f},
     {0.f, 0.f, 0.f},
     0.85f,
     "ceramic"},

    /* === EMISSIVE (1) — neon glow ====================================
     * Neon plasma is self-emissive. The albedo is the dim "off" tube
     * colour; the emissive value glows hot pink even in shadow because
     * emissive is added AFTER lighting. */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f, 0.02f, 0.10f},
     {0.80f, 0.80f, 1.00f},
     {1.00f, 0.20f, 0.85f},
     0.20f,
     "neon"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

static int g_256; /* 1 if 256-colour cube available, 0 = mono fallback */

static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    /* Pairs 1..216 ↔ 6×6×6 RGB cube (xterm 16..231).
     * Index = r·36 + g·6 + b + 1, all in {0..5}. */
    for (int i = 0; i < 216; i++)
      init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
  }
  init_pair(PAIR_HUD, 226, -1); /* bright yellow                 */
  init_pair(PAIR_HINT, 51, -1); /* bright cyan                   */
}

/*
 * draw_color — paint one cell with a colour and a luminance.
 *
 * Hue → 6×6×6 cube pair; luminance → ASCII-density character.
 * Both channels of the same pixel: colour says "what it is," density
 * says "how bright it is."
 */
/* luma_to_ramp_char — map a luminance in [0,1] to a Bourke density glyph:
 * 0 → ' ' (sparsest), 1 → '@' (densest). The "how bright" channel. */
static char luma_to_ramp_char(float lum) {
  if (lum < 0.f)
    lum = 0.f;
  if (lum > 1.f)
    lum = 1.f;
  return k_ramp[(int)(lum * (RAMP_LEN - 1))];
}

/* rgb_to_cube_pair — map a linear RGB colour to its nearest xterm 6×6×6
 * colour-cube ncurses pair. Each channel quantizes to {0..5}; the pair index
 * is the cube offset r·36 + g·6 + b + base. The "what colour" channel. */
static int rgb_to_cube_pair(V3 c) {
  int r5 = (int)(c.x * 5.f + .5f);
  if (r5 > 5)
    r5 = 5;
  int g5 = (int)(c.y * 5.f + .5f);
  if (g5 > 5)
    g5 = 5;
  int b5 = (int)(c.z * 5.f + .5f);
  if (b5 > 5)
    b5 = 5;
  return PAIR_CUBE_BASE + r5 * 36 + g5 * 6 + b5;
}

static void draw_color(int row, int col, V3 c, float lum) {
  char ch = luma_to_ramp_char(lum);
  if (g_256) {
    int pair = rgb_to_cube_pair(c);
    attron(COLOR_PAIR(pair));
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair));
  } else {
    mvaddch(row, col, (chtype)(unsigned char)ch);
  }
}

/* ── §5 LOGIC — ray-torus intersection (THE CORE), pure
 * ────────────────────────────── */

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
static inline float q_eval(float t, float A, float B, float C, float D) {
  return t * (t * (t * (t + A) + B) + C) + D;
}

/* torus_quartic_coeffs — derive q(t)=t⁴+A·t³+B·t²+C·t+D for the ray ro+t·rd vs
 * a torus (R major, r_minor tube), by substituting the ray into the implicit
 * surface equation and squaring twice. See MENTAL MODEL → KEY FORMULAS. */
static void torus_quartic_coeffs(V3 ro, V3 rd, float R, float r_minor, float *A,
                                 float *B, float *C, float *D) {
  float po2 = v3dot(ro, ro);                  /* |ro|²        */
  float rod = v3dot(rd, ro);                  /* rd · ro      */
  float rxz2 = ro.x * ro.x + ro.z * ro.z;     /* in-plane |ro|² */
  float rdxz_d = rd.x * ro.x + rd.z * ro.z;   /* in-plane rd·ro*/
  float rdxz2 = rd.x * rd.x + rd.z * rd.z;    /* in-plane |rd|²*/
  float C0 = po2 + R * R - r_minor * r_minor; /* common offset */
  *A = 4.f * rod;
  *B = 4.f * rod * rod + 2.f * C0 - 4.f * R * R * rdxz2;
  *C = 4.f * rod * C0 - 8.f * R * R * rdxz_d;
  *D = C0 * C0 - 4.f * R * R * rxz2;
}

/* bisect_root — refine a root of q bracketed in (lo, hi), where q(lo)=flo has
 * opposite sign to q(hi), by Q_BISECT halvings (intermediate value theorem).
 * Returns the final bracket midpoint. The numerically-stable alternative to
 * Ferrari's closed-form quartic, which divides by near-zero on tangent rays. */
static float bisect_root(float lo, float hi, float flo, float A, float B,
                         float C, float D) {
  for (int j = 0; j < Q_BISECT; j++) {
    float mid = (lo + hi) * 0.5f;
    float fmid = q_eval(mid, A, B, C, D);
    if (flo * fmid < 0.f) {
      hi = mid; /* root in (lo, mid)       */
    } else {
      lo = mid;
      flo = fmid; /* root in (mid, hi)       */
    }
  }
  return (lo + hi) * 0.5f; /* midpoint of final bracket */
}

/* §5.2 ray_torus — derive coefficients + scan-bisect for smallest root */

/*
 * ray_torus — analytic ray vs torus intersection (T3, T4).
 *
 * Inputs:
 *   ro, rd      ray in OBJECT space, rd unit-length
 *   R           major radius (centreline circle)
 *   r_minor     minor radius (tube cross-section)
 * Output:
 *   *t_hit      smallest positive t where the ray meets the torus
 * Returns: 1 on hit, 0 on miss.
 *
 * Pseudocode (ray_torus + helpers torus_quartic_coeffs / bisect_root):
 *   po2     = |ro|²                          common dot products
 *   rod     = rd · ro
 *   rxz²    = ro.x² + ro.z²                  in-plane projections
 *   rdxz_d  = rd.x·ro.x + rd.z·ro.z
 *   rdxz²   = rd.x² + rd.z²
 *   C0      = po2 + R² − r²                  recurring constant
 *   --- quartic coefficients (T3 derivation): ---
 *   A = 4·rod
 *   B = 4·rod² + 2·C0 − 4·R²·rdxz²
 *   C = 4·rod·C0 − 8·R²·rdxz_d
 *   D = C0² − 4·R²·rxz²
 *   --- scan and bisect (T4): ---
 *   dt = Q_T_MAX / Q_SAMPLES
 *   t0 = ε;  f0 = q(t0)
 *   for i = 1 .. Q_SAMPLES:
 *     t1 = i·dt;  f1 = q(t1)
 *     if f0 · f1 < 0:                        sign change → root in (t0, t1)
 *       lo, hi, flo = t0, t1, f0
 *       for j = 0 .. Q_BISECT-1:
 *         mid = (lo + hi) / 2
 *         fmid = q(mid)
 *         if flo · fmid < 0: hi = mid
 *         else:              lo = mid;  flo = fmid
 *       *t_hit = (lo + hi) / 2
 *       return 1
 *     t0 = t1;  f0 = f1
 *   return 0                                  no sign change → miss
 *
 * Mental model: q(t) is a smooth degree-4 polynomial. Real roots
 * are points where q crosses zero. The sign-change test (f0·f1 < 0)
 * is the intermediate value theorem in code: between a positive
 * value and a negative value, a continuous function must cross
 * zero. Bisection halves the bracket each iteration; after 40
 * halvings we know the root to ~2⁻⁴⁰ ≈ 10⁻¹² of the original
 * bracket — far below single-precision float resolution.
 *
 * Why scan-bisect and not Ferrari's closed-form quartic formula?
 *   Ferrari is mathematically beautiful (substitute u = t + A/4 to
 *   eliminate the cubic term, solve a resolvent cubic, take square
 *   roots) but NUMERICALLY UNSTABLE. Tangent rays produce
 *   coefficients where the discriminants pass near zero — small
 *   input errors blow up into large output errors. Scan-bisect is
 *   slower (~300 ops vs ~50 for Ferrari) but COMPLETELY STABLE:
 *   it never divides by anything that could be zero.
 *
 * Edge cases:
 *   - Tangent ray. q dips to zero without crossing. No sign change
 *     in adjacent samples → missed (acceptable).
 *   - Two roots within one Δt bin. Only the FIRST one is found.
 *     With Q_SAMPLES = 256 and T_MAX = 18 m, Δt ≈ 0.07 m — much
 *     smaller than the torus's diameter, so this only matters
 *     near-tangent silhouettes.
 *   - Camera inside the torus tube. Doesn't happen in our scene
 *     (camera at z = -3.4, torus at origin); if it did, the first
 *     positive root would still be the exit hit.
 */
static int ray_torus(V3 ro, V3 rd, float R, float r_minor, float *t_hit) {
  /* Derive the quartic q(t) = t⁴ + A·t³ + B·t² + C·t + D for this ray. */
  float A, B, C, D;
  torus_quartic_coeffs(ro, rd, R, r_minor, &A, &B, &C, &D);

  /* Scan: evaluate q at uniformly-spaced t, look for sign changes. */
  float dt = Q_T_MAX / (float)Q_SAMPLES;
  float t0 = T_EPS;
  float f0 = q_eval(t0, A, B, C, D);

  for (int i = 1; i <= Q_SAMPLES; i++) {
    float t1 = (float)i * dt;
    float f1 = q_eval(t1, A, B, C, D);

    if (f0 * f1 < 0.f) {
      /* Sign change → a root lies in (t0, t1); refine it and return. */
      *t_hit = bisect_root(t0, t1, f0, A, B, C, D);
      return 1;
    }
    t0 = t1;
    f0 = f1;
  }
  return 0; /* no sign change → miss   */
}

/* §5.3 torus_normal — closest-point geometric formula ──────────────── */

/*
 * torus_normal — outward surface normal at a hit point P (T5).
 *
 * Inputs:  P (object-space hit point), R (major radius)
 * Returns: unit vector pointing outward from the torus surface
 *
 * Pseudocode (mirrors body 1:1):
 *   P_xz    = (P.x, 0, P.z)                     drop Y component
 *   ρ       = |P_xz|
 *   ring_pt = (ρ > ε) ? (R/ρ) · P_xz : (R, 0, 0)  closest centreline point
 *   return  norm(P − ring_pt)
 *
 * Mental model: the torus is a circle SWEPT by a tube of radius r.
 * The outward normal at any point P on the tube points FROM the
 * closest centreline point TO P. To find the closest centreline
 * point, project P onto the XZ plane (drop Y) and rescale to
 * length R — the resulting (R/|P_xz|)·P_xz is the centreline point
 * at the same azimuth as P.
 *
 * Why this and not the implicit gradient?
 *   The gradient of f = (√(x²+z²)−R)² + y² − r² involves a 1/√(x²+z²)
 *   term — chain rule on a square root. Expensive (sqrt + divide)
 *   and unstable when |P_xz| is small. The closest-point formula
 *   needs ONE normalize per pixel and works at all hit points.
 *
 * Worked example: P = (0.96, 0, 0) (a point on the torus's outer
 *   equator with R = 0.68, r = 0.28):
 *     P_xz    = (0.96, 0, 0)
 *     ρ       = 0.96
 *     ring_pt = (0.68/0.96) · (0.96, 0, 0) = (0.68, 0, 0)
 *     N       = norm((0.96 − 0.68, 0, 0)) = norm((0.28, 0, 0))
 *             = (1, 0, 0)
 *   ✓ The outer equator point's outward normal is +X — points away
 *   from the Y axis, which is correct.
 *
 * Edge case |P_xz| ≈ 0: P is on the Y axis, which is INSIDE the
 * torus's hole — not on the surface. Defensive default (R, 0, 0)
 * is unreachable in practice but keeps the code from dividing by
 * zero in pathological inputs.
 */
static V3 torus_normal(V3 P, float R) {
  V3 P_xz = {P.x, 0.f, P.z};
  float rho = v3len(P_xz);
  V3 ring_pt = (rho > 1e-9f) ? v3scale(R / rho, P_xz)
                             : (V3){R, 0.f, 0.f}; /* defensive default */
  return v3norm(v3sub(P, ring_pt));
}

/* ── §6 LOGIC — shading, pure
 * ──────────────────────────────────────────────────────── */

/*
 * ShadeMode — which shading model §7 runs for every ray that hits the torus.
 * The geometry (ray_torus → P, N, V_dir) is IDENTICAL across modes; only the
 * surface-colour function changes — that is the whole pedagogical point: one
 * quartic-solved intersection feeds four different "what does this look like?"
 * answers. Cycled live with `s` (wraps via % MODE_N). Ref: Phong 1975
 * (lighting); Schlick 1994 (Fresnel term); normal/depth are standard G-buffer
 * visualisations.
 */
typedef enum {
  MODE_PHONG = 0, /* three-point Phong lighting — the "real" material look */
  MODE_NORMAL,    /* surface normal mapped to RGB — diagnostic            */
  MODE_FRESNEL,   /* view-grazing Fresnel rim — diagnostic                */
  MODE_DEPTH,     /* hit distance along the ramp — diagnostic             */
  MODE_N          /* count — enables `% MODE_N` cycling                   */
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "fresnel",
                                           "depth"};

/* Three fixed world-space lights — POSITIONS, not directions, all
 * PURE WHITE. Per pixel we compute L = normalize(light_pos − P) so
 * each light direction depends on the hit point (point lights, not
 * directional). The lights have NO tint of their own — every visible
 * colour comes from the material (Theme.albedo + Theme.specular). */
static const V3 LIGHT_KEY = {3.0f, 4.0f, -2.0f};   /* upper-right    */
static const V3 LIGHT_FILL = {-4.0f, 1.5f, -1.0f}; /* upper-left     */
static const V3 LIGHT_RIM = {0.5f, -1.0f, 5.0f};   /* behind         */

/* add_phong_light — accumulate one WHITE point light's Phong contribution
 * onto `col`: Lambert diffuse (max(0, N·L) · diffuse_weight · diffuse_gain ·
 * albedo) plus a specular lobe (max(0, R·V_dir)^shininess · spec_gain · F0).
 * light_pos is a WORLD position; direction L = norm(light_pos − P) is
 * recomputed per hit (point light). spec_gain = 0 gives a pure diffuse fill.
 * Ref: Phong 1975. */
static V3 add_phong_light(V3 col, V3 light_pos, V3 P, V3 N, V3 V_dir,
                          float diffuse_gain, float spec_gain, float shininess,
                          const Theme *th) {
  V3 L = v3norm(v3sub(light_pos, P));
  float d = fmaxf(0.f, v3dot(N, L));
  col = v3add(col, v3scale(d * th->diffuse_weight * diffuse_gain, th->albedo));
  V3 R = v3reflect(v3scale(-1.f, L), N);
  float s = powf(fmaxf(0.f, v3dot(R, V_dir)), shininess);
  col = v3add(col, v3scale(s * spec_gain, th->specular));
  return col;
}

/* §6.1 ── shade_phong: PBR-flavoured Phong (ambient + KEY + FILL + RIM)
 *
 * Per light:
 *   diffuse  = max(0, N · L) * albedo * diffuse_weight * weight
 *   specular = max(0, R · V)^SHININESS  *  Theme.specular * weight
 *
 * Metals have specular tinted to albedo (gold spec is yellow);
 * dielectrics have specular ≈ white. The diffuse_weight scales the
 * body contribution low for metals (≈0.15) and high for plastics
 * (≈0.85), giving each material family its characteristic look under
 * the same white lighting setup.
 *
 * Emissive is added LAST, after lighting, so neon glows even in
 * shadow regions. ────────────────────────────────────────────────── */

static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th) {
  V3 col = v3scale(AMBIENT, th->albedo); /* ambient floor: dim body albedo  */

  /* §6.1.1 KEY  — primary diffuse + sharp specular (upper-right). */
  col = add_phong_light(col, LIGHT_KEY, P, N, V_dir, KEY_DIFFUSE, KEY_SPEC_GAIN,
                        SHININESS, th);
  /* §6.1.2 FILL — soft diffuse only (spec_gain 0); lifts the shadow side. */
  col = add_phong_light(col, LIGHT_FILL, P, N, V_dir, FILL_DIFFUSE, 0.f,
                        SHININESS, th);
  /* §6.1.3 RIM  — wide-cone specular kissing the back silhouette (behind). */
  col = add_phong_light(col, LIGHT_RIM, P, N, V_dir, RIM_DIFFUSE, RIM_SPEC_GAIN,
                        RIM_SHININESS, th);

  /* §6.1.4 Emissive — added before clamp so neon glows even in shadow. */
  col = v3add(col, th->emissive);
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
static V3 shade_normal(V3 N) {
  return (V3){N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f};
}

/* §6.6 ── shade_fresnel + shade_depth (alternative diagnostics) ─────── */

/*
 * Schlick approximation: F = F₀ + (1−F₀)(1−cosθ)^5. With F₀ = 0 (full
 * dielectric) this simplifies to (1−cosθ)^5 — dark head-on, bright
 * at grazing angles. The torus's curved tube produces a complex
 * pattern of bright fresnel bands at every silhouette edge (outer
 * and inner ring + top and bottom of the tube).
 */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th) {
  float cosA = fabsf(v3dot(N, V_dir));
  float inv = 1.f - cosA;
  float fresnel = inv * inv * inv * inv * inv; /* (1−cosθ)^5 */
  V3 core = v3scale(FRESNEL_CORE, th->albedo);
  V3 edge = v3clamp1(v3scale(FRESNEL_EDGE_GAIN, th->specular));
  return v3clamp1(v3add(v3scale(1.f - fresnel, core), v3scale(fresnel, edge)));
}

/* shade_depth — encode hit distance as brightness. */
static V3 shade_depth(float t, float t_max, const Theme *th) {
  float d = 1.f - fminf(t / t_max, 1.f);
  d = d * d;
  return v3clamp1(v3scale(d, th->albedo));
}

/* Rec. 601 luminance for ramp-index choice. */
static inline float rec601_luma(V3 c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* shade_pixel — dispatch one torus hit to the active shading model, returning
 * its display colour and (via *lum) the luminance that picks the ramp glyph.
 * The geometry (P, N, V_dir, t_hit) is mode-independent; only the colour / luma
 * rule differs — the demo's core lesson (one quartic hit, four looks). */
static V3 shade_pixel(ShadeMode mode, V3 P, V3 N, V3 V_dir, float t_hit,
                      float cam_dist, const Theme *th, float *lum) {
  V3 color;
  switch (mode) {
  default:
  case MODE_PHONG:
    color = shade_phong(P, N, V_dir, th);
    *lum = rec601_luma(color);
    break;
  case MODE_NORMAL:
    color = shade_normal(N);
    *lum = (N.x * .5f + .5f) * .3f + (N.y * .5f + .5f) * .6f +
           (N.z * .5f + .5f) * .1f;
    break;
  case MODE_FRESNEL:
    color = shade_fresnel(N, V_dir, th);
    *lum = rec601_luma(color);
    break;
  case MODE_DEPTH:
    color = shade_depth(t_hit, cam_dist * DEPTH_FAR_SCALE, th);
    *lum = rec601_luma(color);
    break;
  }
  return color;
}

/* ── SIMULATION STATE — Scene (read by §7 render, driven by §9) ─────────── *
 *
 * Scene — the whole demo's persistent mutable state in one place: the table
 * of contents the loop in §9 drives. There is no Camera sub-type — the camera
 * is fixed at (0, CAM_HEIGHT, −cam_dist) and only its distance varies — and the
 * torus SHAPE (TORUS_R, TORUS_r) is compile-time config (§1); the only torus
 * STATE is its orientation, so the two tumble angles live directly here.
 *   WHAT : angle_x, angle_y — the torus tumble (advance each tick; reset by
 * `r`) HOW  : cam_dist         — camera distance from origin (the zoom knob)
 *          paused           — sim knob: freeze the tumble advance (DELAYS gate)
 *          theme_idx, mode  — render knobs: which material + which shading
 * model (No WHERE struct: run/resize flags are signal-written globals in §9,
 * and the frame timer is loop-local in main — neither is a reusable concept
 * here.)
 * ─────────────────────────────────────────────────────────────────────── */

typedef struct {
  float angle_x;  /* WHAT: torus tumble pitch (rad) — advances each tick   */
  float angle_y;  /* WHAT: torus tumble yaw   (rad) — advances each tick   */
  float cam_dist; /* HOW:  camera distance from origin (zoom knob)         */
  int paused;     /* HOW:  freeze the tumble advance (the DELAYS gate)     */
  int theme_idx;  /* HOW (render): active material, index into g_themes    */
  ShadeMode mode; /* HOW (render): active shading model                    */
} Scene;

/* scene_init — boot state: gold material, phong shading, default zoom, torus
 * untilted (angles 0), running (not paused). The ONLY full initialiser. */
static void scene_init(Scene *s) {
  s->angle_x = 0.f;
  s->angle_y = 0.f;
  s->cam_dist = CAM_DIST_DEF;
  s->paused = 0;
  s->theme_idx = 0;
  s->mode = MODE_PHONG;
}

/* scene_reset — the `r` key: return the torus to its untilted pose (both
 * tumble angles 0) without touching zoom / material / pause. */
static void scene_reset(Scene *s) {
  s->angle_x = 0.f;
  s->angle_y = 0.f;
}

/* scene_tick — advance the simulation by one wall-clock step of dt_ns
 * nanoseconds: spin the torus (yaw ROT_Y, pitch ROT_X), unless paused. The
 * ONLY function that advances simulation state (§9.3 calls it per frame). */
static void scene_tick(Scene *s, long long dt_ns) {
  if (s->paused)
    return;
  float sec = (float)dt_ns * 1e-9f;
  s->angle_y += ROT_Y * sec;
  s->angle_x += ROT_X * sec;
}

/* ── §7 RENDER — render frame
 * ─────────────────────────────────────────────────── */

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
static void render(const Scene *s, int cols, int rows) {
  const Theme *th = &g_themes[s->theme_idx % THEME_N];
  float fov_tan = tanf(FOV_DEG * (float)M_PI / 360.f);

  Mat3 M = mat3_rot(s->angle_x, s->angle_y);

  /* §7.1 — elevated camera looking at origin. */
  V3 cam = {0.f, CAM_HEIGHT, -s->cam_dist};
  V3 fwd = v3norm(v3sub((V3){0, 0, 0}, cam));
  V3 wup = {0.f, 1.f, 0.f};
  V3 rgt = v3norm(v3cross(fwd, wup));
  V3 up = v3cross(rgt, fwd);

  float cx = cols * 0.5f, cy = rows * 0.5f;

  /* §7.2 — primary loop: one ray per cell. Skip bottom row for HUD. */
  for (int row = 0; row < rows - 1; row++) {
    for (int col = 0; col < cols; col++) {
      /* Normalised screen coords with terminal-cell aspect baked in. */
      float pu = (col - cx) / cx * fov_tan;
      float pv = -(row - cy) / cx * fov_tan / ASPECT;

      V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu, rgt), v3scale(pv, up))));

      /* Transform ray to object space (torus fixed in XZ plane). */
      V3 ro_os = mat3_mulT(M, cam);
      V3 rd_os = mat3_mulT(M, rd_ws);

      float t_hit;
      if (!ray_torus(ro_os, rd_os, TORUS_R, TORUS_r, &t_hit))
        continue;

      /* Hit point in OBJECT and WORLD space; world-space normal. */
      V3 P_os = v3add(ro_os, v3scale(t_hit, rd_os));
      V3 P_ws = v3add(cam, v3scale(t_hit, rd_ws));
      V3 N_os = torus_normal(P_os, TORUS_R);
      V3 N_ws = mat3_mul(M, N_os);
      V3 V_dir = v3norm(v3sub(cam, P_ws));

      float lum;
      V3 color =
          shade_pixel(s->mode, P_ws, N_ws, V_dir, t_hit, s->cam_dist, th, &lum);
      draw_color(row, col, color, lum);
    }
  }
}

/* ── §8 RENDER — screen / HUD
 * ─────────────────────────────────────────────────── */

static void hud_draw(const Scene *s, float fps, int cols, int rows) {
  /* §8.1 top-right status row 0 — yellow, BOLD (HUD spec). */
  char buf[96];
  snprintf(buf, sizeof buf, " %5.1f fps  dist:%.1f  %-9s  %s ", (double)fps,
           (double)s->cam_dist, g_themes[s->theme_idx % THEME_N].name,
           s->paused ? "PAUSED " : "running");
  int len = (int)strlen(buf);
  if (len > cols)
    len = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* §8.2 top-left mode label row 0 — same yellow, no bold (secondary). */
  char buf2[48];
  snprintf(buf2, sizeof buf2, " mode:%-9s ", k_mode_names[s->mode]);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, "%s", buf2);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* §8.3 bottom hint strip — cyan, BOLD. ASCII only. */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc/p:pause  s:mode  t:theme  r:reset  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9 SIMULATION + APP — tick, events, loop
 * ──────────────────────────────────────────────────────────── */

static volatile sig_atomic_t g_run = 1;
static volatile sig_atomic_t g_resize = 0;
static void on_sigint(int s) {
  (void)s;
  g_run = 0;
}
static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}

static void cleanup(void) { endwin(); }

int main(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  atexit(cleanup);
  color_init();

  int cols, rows;
  getmaxyx(stdscr, rows, cols);

  Scene scene;
  scene_init(&scene);

  float fps = 0.f;
  long long fps_acc = 0;
  int fps_cnt = 0;
  long long frame_ns = 1000000000LL / TARGET_FPS;
  long long last = clock_ns();

  while (g_run) {
    /* One iteration = one frame. Layers combine in the §9.1..§9.7 order
     * documented in ARCHITECTURE; only §9.3 advances simulation state. */

    /* §9.1 USER EVENT — apply pending resize (NOT part of the tick). */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
    }

    /* §9.2 PERFORMANCE — timing. dt is wall-clock; cap to avoid huge jumps
     * after a stall (debugger pause, suspend/resume). */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* §9.3 SIMULATION (the tick) — scene_tick advances the tumble; paused =
     * DELAYS. */
    scene_tick(&scene, dt);

    /* §9.4 PERFORMANCE — fps rolling average over half-second windows. */
    fps_acc += dt;
    fps_cnt++;
    if (fps_acc >= 500000000LL) {
      fps = (float)fps_cnt * 1e9f / (float)fps_acc;
      fps_acc = 0;
      fps_cnt = 0;
    }

    /* §9.5 RENDER combine — the one place state → screen (render + HUD). */
    long long t0 = clock_ns();
    erase();
    render(&scene, cols, rows);
    hud_draw(&scene, fps, cols, rows);
    wnoutrefresh(stdscr);
    doupdate();

    /* §9.6 USER EVENTS — keys (knobs / reset / quit); NOT part of the tick. */
    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27 /* ESC */:
      g_run = 0;
      break;
    case ' ':
    case 'p':
    case 'P':
      scene.paused = !scene.paused;
      break;
    case 'r':
    case 'R':
      scene_reset(&scene);
      break;
    case 's':
    case 'S':
      scene.mode = (ShadeMode)((scene.mode + 1) % MODE_N);
      break;
    case 't':
    case 'T':
      scene.theme_idx = (scene.theme_idx + 1) % THEME_N;
      break;
    case '+':
    case '=':
      scene.cam_dist -= CAM_DIST_STEP;
      if (scene.cam_dist < CAM_DIST_MIN)
        scene.cam_dist = CAM_DIST_MIN;
      break;
    case '-':
    case '_':
      scene.cam_dist += CAM_DIST_STEP;
      if (scene.cam_dist > CAM_DIST_MAX)
        scene.cam_dist = CAM_DIST_MAX;
      break;
    default:
      break;
    }

    /* §9.7 PERFORMANCE — frame cap. */
    clock_sleep_ns(frame_ns - (clock_ns() - t0));
  }
  return 0;
}

/*
 * torus_raytrace.c — analytic ray-traced torus (quartic intersection)
 *
 * DEMO: A glossy donut tumbling in space, lit by three FIXED WHITE
 *       LIGHTS. Each material's distinctive look comes from its own
 *       PBR properties — gold has a tinted yellow highlight, glass
 *       shows a near-white specular peak, sapphire reads as deep
 *       blue. Cycle phong → normals → fresnel → depth to see the
 *       same quartic-solved geometry through different shading
 *       models. The torus is the ONLY primitive in this folder that
 *       needs a degree-4 polynomial — sphere is degree 2, cylinder
 *       is degree 2, AABB is linear. The torus's tube curving around
 *       a ring creates the extra algebraic complexity.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — quadratic intersection (degree 2)
 *   raytracing/cube_raytrace.c     — slab method (linear per axis)
 *   raytracing/capsule_raytrace.c  — decomposed analytic (cyl + 2 spheres)
 *   raytracing/path_tracer.c       — same RGB-cube paint pipeline,
 *                                    multi-bounce Monte Carlo on top
 *
 * Section map (see ARCHITECTURE for the layer model):
 *   §1 config     — [DATA]   frame rate, FOV, torus geometry, camera, ramp, HUD
 *   §2 clock      — [PERF]   monotonic timer + sleep
 *   §3 math       — [LOGIC]  V3, Mat3 (with the inverse-rotation explanation)
 *   §4 color      — [RENDER+DATA] themes + 256-colour cube + ramp painter
 *   §5 torus      — [LOGIC]  THE CORE: ray-torus quartic + numerical solver
 *                   §5.1 q_eval (Horner-form polynomial evaluation)
 *                   torus_quartic_coeffs (derive A/B/C/D) · bisect_root
 * (refine) §5.2 ray_torus (derive → scan → bisect driver) §5.3 torus_normal
 * (closest-point geometric formula) §6 shading    — [LOGIC]  phong / normals /
 * fresnel / depth (pure) add_phong_light → shade_phong (§6.1: KEY/FILL/RIM
 * lights) shade_normal (§6.5)  shade_fresnel + shade_depth (§6.6) shade_pixel —
 * picks the shading model per ray §7 render     — [RENDER] one frame:
 * ray-per-cell, object-space transform §8 screen     — [RENDER] ncurses init +
 * HUD §9 app        — [SIM+APP] tick (tumble), events, performance, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   t / T     next / previous theme (20 PBR materials: 12 metals,
 *             4 gems, 3 dielectrics, 1 emissive — see §4)
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

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL — read in that order
 *      as prose. Read sphere_raytrace.c first if you don't already
 *      know ray-sphere; the quartic is the analytic step BEYOND
 *      ray-sphere, and most of the code is the same.
 *   2. §1 config — every constant has a unit-bearing comment.
 *   3. §5 ray-torus — THE CORE. Read AFTER tutorials T2-T4. q_eval
 *      (§5.1) is one line of Horner-form polynomial evaluation;
 *      ray_torus (§5.2) is the scan-and-bisect solver; torus_normal
 *      (§5.3) is the closest-point geometric formula.
 *   4. §6 shading — five small shaders, identical to sphere_raytrace.c.
 *      Read AFTER tutorial T6.
 *   5. §3 math has Mat3 helpers (rotation matrix + transpose-multiply)
 *      with the inverse-rotation trick in §3 explanation. Read AFTER
 *      tutorial T1.
 *   6. §4 themes — pure data, skim. Same Theme structure as the rest
 *      of the folder.
 *   7. §7 render + §8 hud + §9 app — infrastructure, skip on first
 *      read.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   ro / rd            primary ray as (origin, unit direction)
 *                      In §7 we have ro_ws / rd_ws (world-space) and
 *                      ro_os / rd_os (object-space, after inverse-
 *                      rotation). The quartic solver works in
 *                      object space.
 *   R / r              torus radii — MAJOR (centreline) and MINOR
 *                      (tube cross-section). NOT to be confused with
 *                      the reflection vector in shading code.
 *   A, B, C, D         the four quartic coefficients (named after
 *                      their canonical positions in t⁴ + A·t³ +
 *                      B·t² + C·t + D = 0).
 *   q(t)               the quartic polynomial value at parameter t.
 *
 *   Single-letter names (i, c, t) appear inside tight loops.
 *
 * Background you need
 * ───────────────────
 *   - Same as sphere_raytrace.c: 3-D vectors, Lambertian shading,
 *     6×6×6 cube quantisation.
 *   - PLUS: the quadratic formula is degree 2, so a tangent ray
 *     to a sphere is a "double root". The torus is degree 4 — up
 *     to four real intersections in t. Make peace with that.
 *   - Numerical root-finding by BISECTION: if f(a) and f(b) have
 *     opposite signs, a root lies in [a, b] — halve the bracket
 *     repeatedly to nail it down.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Closed-form quartic solution (Ferrari's method). Too unstable
 *     near tangent rays; we use scan-and-bisect instead.
 *   - Galois theory. Yes, quartics are the LAST polynomial degree
 *     with a closed-form solution; you don't need to know why.
 *   - Acceleration structures.
 *
 * ─────────────────────────────────────────────────────────────────── */

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
 *                  20 themes are PBR-flavored material descriptors
 *                  (albedo, specular F0, emissive, diffuse_weight)
 *                  shared with sphere_raytrace.c and cube_raytrace.c.
 *                  216 ncurses pairs pre-allocated as a 6×6×6 RGB
 *                  cube.
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

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Eight short tutorials that build the renderer from first
 * principles. Read in order; each builds on the previous.
 *
 *   T1  The inverse-rotation trick — rotate the ray, not the torus
 *   T2  The torus implicit equation — distance to a circle
 *   T3  Squaring twice — deriving the quartic
 *   T4  Sample-then-bisect — numerical root-finding without Ferrari
 *   T5  Closest-point normal vs implicit gradient
 *   T6  PBR-flavored Phong (cross-reference sphere_raytrace.c T3-T4)
 *   T7  Three debug overlays — NORMAL / FRESNEL / DEPTH
 *   T8  Continuous-RGB pipeline → 6×6×6 cube + 92-char ramp
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  THE INVERSE-ROTATION TRICK — ROTATE THE RAY, NOT THE TORUS
 * ───────────────────────────────────────────────────────────────
 * The torus is an animated object; we want it to tumble in space.
 * Two ways to do this:
 *
 *   FORWARD WAY: rotate the geometry. Recompute the implicit
 *   equation's coefficients in the rotated frame each frame.
 *   This is messy — the quartic coefficients (T3) assume the torus
 *   sits flat in the XZ plane at the origin. Rotating the geometry
 *   means re-deriving the algebra.
 *
 *   INVERSE WAY: leave the torus fixed in OBJECT SPACE (XZ plane,
 *   origin), and rotate the RAY into that frame. If M is the
 *   torus's rotation matrix, then for each pixel:
 *
 *       ro_os = M^T · cam_world          (camera in object space)
 *       rd_os = M^T · rd_world           (direction in object space)
 *
 *   M^T is M's TRANSPOSE — the inverse of a rotation matrix is its
 *   transpose. Now we test the ray against the canonical torus
 *   equation, find a hit point P_os in object space, compute the
 *   normal N_os in object space, then transform the normal back to
 *   world space for shading:
 *
 *       N_world = M · N_os
 *
 *   The quartic coefficients stay simple because the torus is
 *   always at the canonical position. See §3 for Mat3, mat3_mul,
 *   mat3_mulT (transpose-multiply).
 *
 * T2  THE TORUS IMPLICIT EQUATION — DISTANCE TO A CIRCLE
 * ───────────────────────────────────────────────────────
 * A torus is the locus of points at distance r (minor radius) from
 * a fixed CIRCLE of radius R (major radius). For our canonical
 * torus the centreline circle lies in the XZ plane at the origin:
 *
 *   centreline = { (R·cosφ, 0, R·sinφ) : φ ∈ [0, 2π) }
 *
 * For an arbitrary point (x, y, z), the closest point on the
 * centreline has the same azimuth as (x, z). Project (x, z) onto a
 * unit-length direction and rescale:
 *
 *     ρ        = √(x² + z²)             distance from Y axis
 *     ring_pt  = (R·x/ρ, 0, R·z/ρ)      closest centreline point
 *
 * Distance from (x, y, z) to ring_pt:
 *
 *     d² = (x − R·x/ρ)² + y² + (z − R·z/ρ)²
 *        = x²(1 − R/ρ)² + y² + z²(1 − R/ρ)²
 *        = (x² + z²)(1 − R/ρ)² + y²
 *        = ρ²·(1 − R/ρ)² + y²
 *        = (ρ − R)² + y²
 *
 * The torus surface is d = r, so the implicit equation is:
 *
 *     (√(x² + z²) − R)² + y² = r²
 *
 * The square root is unavoidable in this form — it's the distance
 * from the point to the Y axis. Squaring twice (T3) eliminates it.
 *
 * T3  SQUARING TWICE — DERIVING THE QUARTIC
 * ──────────────────────────────────────────
 * Substitute the parametric ray P(t) = ro + t·rd into the implicit
 * equation:
 *
 *     (√((ro.x + t·rd.x)² + (ro.z + t·rd.z)²) − R)² + (ro.y + t·rd.y)² = r²
 *
 * Let u = (ro.x + t·rd.x)² + (ro.z + t·rd.z)². Then √u is the
 * radial component. Move the y² term to the LHS:
 *
 *     (√u − R)²  =  r² − (ro.y + t·rd.y)²
 *
 * Expand the LHS:
 *     u − 2R·√u + R²  =  r² − (ro.y + t·rd.y)²
 *
 * Isolate the √u term:
 *     2R·√u  =  u + R² − r² + (ro.y + t·rd.y)²
 *
 * SQUARE BOTH SIDES (this is where the quartic emerges):
 *     4R²·u  =  (u + R² − r² + (ro.y + t·rd.y)²)²
 *
 * Expand both sides into polynomials in t. The LHS u is degree 2;
 * the RHS is the square of (u + ...), which is degree 4 in t.
 * Collect by powers:
 *
 *     t⁴ · 1
 *   + t³ · 4(rd · ro)
 *   + t² · [4(rd·ro)² + 2(|ro|² + R² − r²) − 4R²(rd.x² + rd.z²)]
 *   + t  · [4(rd·ro)(|ro|² + R² − r²) − 8R²(rd.x·ro.x + rd.z·ro.z)]
 *   +     [(|ro|² + R² − r²)² − 4R²(ro.x² + ro.z²)]
 *   = 0
 *
 * Name the coefficients A, B, C, D for the t³ .. t⁰ terms (the
 * leading t⁴ has coefficient 1 because rd is unit-length, so
 * rd·rd = 1):
 *
 *     t⁴ + A·t³ + B·t² + C·t + D = 0
 *
 * That's the polynomial we need to solve. See §5.2 ray_torus for
 * the code that computes A, B, C, D from a given ray + (R, r).
 *
 * Worked example (head-on equatorial ray, R = 0.68, r = 0.28):
 *   ro = (0, 0, -3.4)
 *   rd = (0, 0, +1)
 *
 *   Pre-computed quantities:
 *     po2     = |ro|² = 0 + 0 + 11.56 = 11.56
 *     rod     = rd · ro = -3.4
 *     rxz²    = ro.x² + ro.z² = 11.56     (in-plane part of |ro|²)
 *     rdxz_d  = rd.x·ro.x + rd.z·ro.z = -3.4
 *     rdxz²   = rd.x² + rd.z² = 1.0       (in-plane part of |rd|²)
 *     C0      = po2 + R² − r² = 11.56 + 0.4624 − 0.0784 = 11.944
 *
 *   Coefficients:
 *     A = 4·(-3.4)                                = -13.6
 *     B = 4·(-3.4)² + 2·11.944 − 4·0.4624·1.0
 *       = 46.24 + 23.888 − 1.850                  = 68.28
 *     C = 4·(-3.4)·11.944 − 8·0.4624·(-3.4)
 *       = -162.44 + 12.578                        = -149.9
 *     D = 11.944² − 4·0.4624·11.56
 *       = 142.66 − 21.39                          = 121.27
 *
 *   Geometric expectation: the ray runs along z, so it should hit
 *   the torus where (|z| − R)² = r² (since x = 0, y = 0):
 *     |z| − R = ±r   →   |z| ∈ {R − r, R + r} = {0.40, 0.96}
 *
 *   Mapping z = -3.4 + t (the ray goes from z = -3.4 toward z = +∞):
 *     t1 ≈ 2.44   (entry-near, z = -0.96)
 *     t2 ≈ 3.00   (entry-far,  z = -0.40)
 *     t3 ≈ 3.80   (exit-near,  z = +0.40)
 *     t4 ≈ 4.36   (exit-far,   z = +0.96)
 *
 *   Verify q(2.44) using the coefficients above (Horner):
 *     ((2.44 + (-13.6))·2.44 + 68.28)·2.44 + (-149.9))·2.44 + 121.27
 *     ≈ ((-27.20)·2.44 + 68.28)·2.44 + (-149.9))·2.44 + 121.27
 *     ≈ (1.91·2.44 + (-149.9))·2.44 + 121.27
 *     ≈ (-145.24)·2.44 + 121.27
 *     ≈ -354.39 + 121.27 ≈ -233.1     hmm, not zero
 *
 *   ...because Horner evaluation needs a leading 1 for the t⁴
 *   coefficient. Re-doing with q(t) = t·(t·(t·(t + A) + B) + C) + D:
 *     step 1: 2.44 + A = 2.44 − 13.6 = -11.16
 *     step 2: (-11.16)·2.44 + B = -27.23 + 68.28 = 41.05
 *     step 3: (41.05)·2.44 + C = 100.16 − 149.9  = -49.74
 *     step 4: (-49.74)·2.44 + D = -121.36 + 121.27 = -0.09  ✓
 *
 *   q(2.44) ≈ -0.09 — close to zero (root!). Plug t = 2.45 to see
 *   it's already turning negative further, and t = 2.43 to see
 *   it was positive — that's the sign change that the scan catches.
 *
 * T4  SAMPLE-THEN-BISECT — NUMERICAL ROOT-FINDING WITHOUT FERRARI
 * ───────────────────────────────────────────────────────────────
 * Quartics have a closed-form solution (Ferrari's method, 1540) but
 * it's NUMERICALLY UNSTABLE near tangent rays. Small input
 * perturbations cause large root errors — the rendered torus
 * surface flickers. Real production raytracers use either a
 * specialised stable quartic solver or numerical methods.
 *
 * We use SAMPLE-AND-BISECT — the simplest robust method:
 *
 *   1. SCAN: evaluate q(t) at Q_SAMPLES = 256 uniformly-spaced
 *      values of t in [ε, T_MAX]. Keep track of the previous q value.
 *
 *   2. SIGN CHANGE: when consecutive samples have OPPOSITE signs,
 *      the intermediate value theorem says a real root lies in
 *      that interval (q is continuous, it must cross zero between
 *      a positive value and a negative value).
 *
 *   3. BISECT: halve the interval Q_BISECT = 40 times. Each
 *      iteration:
 *          mid = (a + b) / 2
 *          if q(a) and q(mid) have opposite signs: b = mid
 *          else:                                   a = mid
 *      After 40 halvings the interval is 2⁻⁴⁰ ≈ 10⁻¹² of the
 *      original — far below single-precision float resolution.
 *
 *   4. RETURN the FIRST positive root we find. That's the front-
 *      face hit of the ray.
 *
 * Trade-off: tangent rays where q grazes zero without crossing
 * (a "double root") aren't found — there's no sign change. The
 * torus's silhouette can show tiny single-pixel gaps at exactly-
 * tangent angles. Acceptable visual cost.
 *
 * Cost: 256 polynomial evaluations + 40 bisections per pixel that
 * has any sign change ≈ 300 floating-point ops in the hit case,
 * 256 + 0 in the miss case. Sphere is ~10 ops; torus is intrinsic-
 * ally more expensive. See §5 for the implementation.
 *
 * Worked example (continuing from T3, ray with sign change in
 *   bracket [2.43, 2.44]; q(2.43) > 0, q(2.44) < 0; bisect):
 *
 *     iter   lo      hi      mid     q(mid)        next bracket
 *     ────   ────    ────    ────    ────          ────────────
 *     0      2.430   2.440   2.435   ≈ +0.40       lo = 2.435
 *     1      2.435   2.440   2.4375  ≈ +0.16       lo = 2.4375
 *     2      2.4375  2.440   2.43875 ≈ +0.04       lo = 2.43875
 *     3      2.43875 2.440   2.43938 ≈ -0.03       hi = 2.43938
 *     4      2.43875 2.43938 2.43906 ≈ +0.005      lo = 2.43906
 *     ...
 *     40     bracket width 2⁻⁴⁰·0.01 ≈ 10⁻¹⁴
 *
 *   The bracket halves every iteration; after 40 iterations we
 *   know t to about 10⁻¹⁴ — far beyond float precision. Output
 *   is the midpoint of the final bracket.
 *
 * Read §5.1 q_eval for Horner-form polynomial evaluation (3 mults
 * + 3 adds per t-value, vs. 6 mults naive).
 *
 * T5  CLOSEST-POINT NORMAL VS IMPLICIT GRADIENT
 * ──────────────────────────────────────────────
 * Once we have the hit point P, we need the surface normal at P.
 * Two ways:
 *
 *   IMPLICIT GRADIENT. The normal to an implicit surface F(p) = 0
 *   is ∇F evaluated at P. Compute the partial derivatives of
 *   (√(x² + z²) − R)² + y² − r² with respect to x, y, z, normalise.
 *   Works but requires the radical's derivative (chain rule on √).
 *
 *   CLOSEST-POINT GEOMETRY. The normal at any point P on the torus
 *   surface points FROM the closest centreline point TO P:
 *
 *       P_xz    = (P.x, 0, P.z)
 *       ρ       = |P_xz|
 *       ring_pt = (R/ρ) · P_xz       project P_xz to centreline
 *       N       = normalize(P − ring_pt)
 *
 *   This is the same construction as T2 — the closest centreline
 *   point — but now we use it for the normal direction. No radical
 *   derivatives, just vector projection + subtraction.
 *
 * The closest-point method is FASTER (one normalise vs three
 * partial derivatives) and STABLER (no chain rule on a radical).
 * See §5.3 torus_normal.
 *
 * T6  PBR-FLAVORED PHONG  (cross-reference)
 * ──────────────────────────────────────────
 * The shading model is identical to sphere_raytrace.c — three
 * white lights at fixed positions, PBR-flavored Phong with
 * material-tinted F0 and a low diffuse_weight for metals. Read
 * sphere_raytrace.c tutorials T3-T4 for the full discussion.
 *
 * Per pixel that hits the torus:
 *
 *     ambient   = AMBIENT_K · albedo
 *     for each of (KEY, FILL, RIM):
 *       diffuse  += weight · diffuse_weight · max(0, N·L) · albedo
 *       specular += weight · max(0, R·V_dir)^SHININESS · F0
 *     col = ambient + diffuse + specular + emissive
 *
 * No torus-specific shading; the whole point of the inverse-
 * rotation trick (T1) is that once you have N at P (in world
 * space), the shader doesn't care what shape produced it.
 *
 * T7  THREE DEBUG OVERLAYS — NORMAL / FRESNEL / DEPTH
 * ────────────────────────────────────────────────────
 * Cycling `s' switches between four shading modes. Each is a
 * different pedagogical lens on the same scene:
 *
 *   PHONG     The full pipeline (PBR Phong with three white lights).
 *
 *   NORMAL    RGB-encoded surface normal: N → (N+1)/2. Each
 *             component remapped from [-1,+1] → [0,1]. The TOP of
 *             the tube (positive Y) reads green-dominant; the OUTER
 *             edge (away from Y axis) reads red/blue; the INNER
 *             edge (toward Y axis) reads opposite. The rainbow-on-
 *             tube look is the easiest way to verify normals are
 *             oriented correctly all the way around.
 *
 *   FRESNEL   Schlick (1 − cosθ)^5. Dark head-on, bright at
 *             silhouette. Both INNER and OUTER tube silhouettes
 *             glow brightly — diagnostic for "are the geometric
 *             silhouettes where I expect them?"
 *
 *   DEPTH     Brightness ∝ 1 − t/t_max. As the torus rotates, the
 *             pattern flows: closer parts swell brighter.
 *
 * The three debug modes converge in ONE FRAME (no randomness).
 * Switching INTO them is instant.
 *
 * T8  CONTINUOUS-RGB PIPELINE → 6×6×6 CUBE + 92-CHAR RAMP
 * ────────────────────────────────────────────────────────
 * Identical pipeline to sphere_raytrace.c (T7) and path_tracer.c.
 *
 *   1. Per-pixel shading produces an HDR linear RGB triplet.
 *   2. Reinhard tone-map per channel:  L' = L / (1 + L)
 *   3. Gamma encode 1/2.2.
 *   4. Quantise each channel to {0..5}: 216 cube cells.
 *   5. Pair index = 16 + 36·R5 + 6·G5 + B5.
 *   6. Density character = Rec.601 luma → ramp[index].
 *   7. A_BOLD on luma > 0.85, A_DIM on luma < 0.15.
 *
 * Tone-mapping happens BEFORE quantisation. Quantising linear HDR
 * puts every pixel above ~0.85 into the same cube cell — no
 * headroom for the specular peaks. Reinhard spreads the dynamic
 * range over the full cube uniformly.
 *
 * Read §6 paint_cell.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE — concern layers & the per-tick combine ────────────── *
 *
 * This demo grew from one combined advance-and-render flow; the concerns
 * below were always present but interleaved. They are now isolated into
 * labelled sections. The only cross-layer state is the Scene aggregate
 * (tumble + knobs) plus a few timing locals in main() (§9), see MUTATES.
 *
 *   LAYER        SECTION(S)        MUTATES
 *   ----------   ---------------   -----------------------------------------
 *   SIMULATION   §9.3 scene_tick   scene.angle_x, scene.angle_y (tumble)
 *   LOGIC        §3 math,          nothing — pure functions; same inputs
 *                §5 ray-torus,     give the same V3/Mat3/float/int, so render
 *                §6 shading        or effects ordering cannot corrupt them
 *   EFFECTS      — none —          the glow/fresnel "look" is derived at
 *                                  render time from material + geometry,
 *                                  never stored as cosmetic state
 *   RENDER       §4 color/draw,    the terminal only (ncurses screen); reads
 *                §7 render,        simulation + knob state, never writes it.
 *                §8 HUD            color_init also sets g_256 ONCE at startup
 *   DELAYS       §9.3 guard        none — `scene.paused` (knob) gates advance
 *   PERFORMANCE  §2 clock,         last, dt, fps_acc, fps_cnt, fps; sleep —
 *                §9.2/.4/.7        timing only, never simulation state
 *
 * PER-TICK COMBINE — one loop iteration = one frame; layers combine in this
 * fixed order (§9), and ONLY step 3 advances simulation state:
 *   1. (event)  apply pending resize            §9.1   — NOT part of the tick
 *   2. perf     measure dt, cap to DT_CAP_NS     §9.2
 *   3. SIM      scene_tick: angle_x/y += ROT·dt        §9.3  ← the tick
 *   4. perf     roll fps average                 §9.4
 *   5. RENDER   erase → render → HUD → present   §9.5   ← the one state→screen
 *   6. (event)  drain key input                  §9.6   — NOT part of the tick
 *   7. perf     sleep to frame-cap               §9.7
 *
 * User EVENTS (resize §9.1, keys §9.6) are NOT the tick — they set inputs the
 * next tick and render read. Two touch simulation state directly, OUTSIDE the
 * tick: zoom sets scene.cam_dist; RESET (`r`) → scene_reset zeroes the angles.
 * Theme/mode/pause are render or gate knobs.
 * ─────────────────────────────────────────────────────────────────── */
