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

/* ── §1 CONFIG — constants, enums, ramp & layout (data only)
 * ──────────────────────────────────────────────────────── */

/* §1.1 frame rate. */
#define TARGET_FPS 60
#define DT_CAP_NS 100000000LL /* 0.1 s — spiral-of-death cap   */

/* §1.2 view geometry — terminal cell aspect (W/H) and full vertical FOV. */
#define ASPECT 0.47f
#define FOV_DEG 55.0f

/* §1.3 cube (object space, axis-aligned, centred on origin). */
#define CUBE_S 0.80f /* half-extent: spans [-S, +S]   */

/* §1.4 wireframe mode. */
#define WIRE_THRESH 0.055f  /* edge band as fraction of S    */
#define WIRE_LUMA_BASE 0.7f /* edge brightness at the band rim       */
#define WIRE_LUMA_GAIN 0.3f /* extra brightness ramped to the edge   */

/* §1.5 rotation rates (rad/sec). */
#define ROT_Y 0.52f /* primary spin around Y         */
#define ROT_X 0.35f /* tilt around X                 */

/* §1.6 camera distance (orbits along −Z; bigger = cube looks smaller). */
#define CAM_DIST_DEF 3.2f
#define CAM_DIST_MIN 1.5f
#define CAM_DIST_MAX 7.0f
#define CAM_DIST_STEP 0.25f

/* §1.7 shading constants. */
#define AMBIENT 0.20f
#define SHININESS 75.0f     /* phong exponent — high = metal */
#define DEPTH_FAR_SCALE 2.f /* MODE_DEPTH far plane = cam_dist × this */

/* §1.8 character ramp — Paul Bourke 92-char density ladder. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.9 ncurses pair IDs (216 cube + reserved HUD/HINT). */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* §1.10 numerical epsilons. */
#define T_EPS 1e-4f        /* reject t < this (self-hit)    */
#define PARALLEL_EPS 1e-9f /* |ray_dir_i| below this = parallel */

/* §1.11 dummy "infinity" used as initial t_enter / t_exit. */
#define BIG_T 1e30f

/* ── §2 PERFORMANCE — monotonic clock + sleep
 * ───────────────────────────────────────────────────────── */

/*
 * clock_ns — wall-clock time in nanoseconds, monotonic.
 *
 * Why CLOCK_MONOTONIC: we care about ELAPSED real time (for animation
 * dt), not wall date. CLOCK_MONOTONIC never goes backward across NTP
 * adjustments, DST shifts, or system clock changes.
 */
static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/*
 * clock_sleep_ns — best-effort sleep for the requested nanoseconds.
 *
 * Used by the main loop to cap the frame rate without burning the
 * CPU at 100%; we subtract (clock_ns() - frame_start) from the target
 * frame time and sleep the remainder.
 */
static void clock_sleep_ns(long long nanoseconds) {
  if (nanoseconds <= 0)
    return;
  struct timespec request = {nanoseconds / 1000000000LL,
                             nanoseconds % 1000000000LL};
  nanosleep(&request, NULL);
}

/* ── §3 LOGIC: math (pure) — V3 + Mat3
 * ────────────────────────────────────────────── *
 *
 * V3 — three floats by value. All vector helpers are inline to avoid
 * call overhead in the per-pixel loop.
 *
 * Mat3 — a 3×3 rotation matrix stored as three V3 ROWS. Storing rows
 * (rather than columns) makes "M · v" a clean trio of v3dot() calls.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * V3 — a point or direction in 3-D space (also reused as a linear-RGB colour
 * triple, since both are three floats under the same algebra).
 * WHY by-value, not a pointer: a V3 is 12 bytes and rides in registers, which
 * is cheaper than chasing a pointer in the per-pixel hot loop and keeps every
 * vector helper pure (§3 LOGIC). No 4th/homogeneous component: this tracer
 * intersects rays analytically and never does a projective divide, so w would
 * be dead weight.
 * Ref: Shirley, "Ray Tracing in One Weekend"; Foley & van Dam, vectors ch.
 */
typedef struct {
  float x, y,
      z; /* Cartesian components — or linear R,G,B when used as colour */
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
  float length = v3len(a);
  return (length > 1e-9f) ? v3scale(1.f / length, a) : (V3){0.f, 1.f, 0.f};
}

/*
 * v3reflect — reflect outgoing vector v across surface normal n.
 *
 * Identity:  v_reflected = v − 2·(v·n)·n
 *
 * Geometric reading: subtract twice the COMPONENT of v along n. In
 * §6 phong shading we feed v3reflect the NEGATED light vector (−L)
 * so the function's "outgoing" semantics match: light-as-outgoing
 * reflects to viewing-as-outgoing.
 */
static inline V3 v3reflect(V3 v, V3 n) {
  return v3sub(v, v3scale(2.f * v3dot(v, n), n));
}

static inline V3 v3clamp01(V3 v) {
  return (V3){v.x < 0.f   ? 0.f
              : v.x > 1.f ? 1.f
                          : v.x,
              v.y < 0.f   ? 0.f
              : v.y > 1.f ? 1.f
                          : v.y,
              v.z < 0.f   ? 0.f
              : v.z > 1.f ? 1.f
                          : v.z};
}

/*
 * Mat3 — a 3x3 rotation matrix, stored as three V3 ROWS.
 * WHY rows: with row storage "M*v" is exactly three v3dot() calls (row[i]*v).
 * Only rotations live here, so the matrix is always orthonormal and its
 * transpose IS its inverse (Mt == M^-1). That identity is the whole trick in
 * §8: a world-space ray is pushed into object space with Mt*v — no inverse is
 * ever computed. No translation/scale rows, so 3x3 (not 4x4) suffices.
 * Ref: Foley & van Dam, "Computer Graphics: Principles and Practice",
 * transforms.
 */
typedef struct {
  V3 row[3]; /* orthonormal basis rows; row[i]*v = component i of M*v */
} Mat3;

/*
 * mat3_rotation — composed rotation Rx(angle_x) · Ry(angle_y).
 *
 * Purpose: build the OBJECT→WORLD rotation matrix.
 * Inputs : angle_x, angle_y in radians.
 * Output : Mat3 with three V3 rows.
 *
 * Composition order: Y first, then X. For an axis-aligned cube both
 * rotations matter — Y rotates the silhouette around the vertical
 * axis (cube → diamond → square), X tilts to expose top/bottom faces.
 *
 * Why this matrix flavour: T7 explains we rotate the RAY by the
 * INVERSE of this matrix. Since rotations are orthogonal, the inverse
 * is the transpose — see mat3_mulT below.
 */
static Mat3 mat3_rotation(float angle_x, float angle_y) {
  float cos_x = cosf(angle_x), sin_x = sinf(angle_x);
  float cos_y = cosf(angle_y), sin_y = sinf(angle_y);
  Mat3 m;
  m.row[0] = (V3){cos_y, 0.f, sin_y};
  m.row[1] = (V3){sin_x * sin_y, cos_x, -sin_x * cos_y};
  m.row[2] = (V3){-cos_x * sin_y, sin_x, cos_x * cos_y};
  return m;
}

/*
 * mat3_mul — forward transform v_world = M · v_obj.
 *
 * Each output component is a dot of one matrix row with the input.
 * Used in §8 to bring the OBJECT-space surface normal back to WORLD
 * space for shading.
 */
static V3 mat3_mul(Mat3 m, V3 v) {
  return (V3){v3dot(m.row[0], v), v3dot(m.row[1], v), v3dot(m.row[2], v)};
}

/*
 * mat3_mulT — inverse transform v_obj = Mᵀ · v_world.
 *
 * For an orthogonal matrix Mᵀ = M⁻¹. Same operation count as
 * mat3_mul; just a transposed memory pattern. No determinant, no
 * division.
 *
 * Used in §8 to push the WORLD-space ray into OBJECT space where
 * the cube is axis-aligned.
 */
static V3 mat3_mulT(Mat3 m, V3 v) {
  return (V3){m.row[0].x * v.x + m.row[1].x * v.y + m.row[2].x * v.z,
              m.row[0].y * v.x + m.row[1].y * v.y + m.row[2].y * v.z,
              m.row[0].z * v.x + m.row[1].z * v.y + m.row[2].z * v.z};
}

/* ── §4 CONFIG/DATA: materials — Theme type + theme table
 * ─────────────────────────────────────────────── *
 *
 * Two channels make every cell expressive on a monochrome glyph grid:
 *
 *   COLOUR    256-colour 6×6×6 cube → "what" the surface is.
 *   DENSITY   ASCII-ramp character → "how bright" the surface is.
 *
 * Themes vary the obj/spec/light tints; the geometry pipeline doesn't
 * care which theme is active.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── Theme — PBR-flavoured material descriptor ────────────────────── *
 *
 * First-principles redesign: the LIGHTS are pure WHITE, and each
 * material's distinctive look comes entirely from its own properties.
 * Under white light, gold looks like gold (warm yellow metal) and
 * blue plastic looks like blue plastic — the colour is intrinsic to
 * the material, not painted on by a tinted light source.
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
 *                     metals      ≈ 0.15 (real metals diffuse very
 *                                         little — almost everything
 *                                         reflects via specular)
 *                     gems        ≈ 0.70 (saturated body + bright spec)
 *                     plastic /
 *                       ceramic   ≈ 0.85 (full body colour)
 *                     glass       ≈ 0.10 (very dark body, bright spec
 *                                         fakes the transparent look)
 *                     neon        ≈ 0.20 (low diffuse + bright emissive)
 *
 * The 20 themes below are organised into 4 families. Switching themes
 * with `t / T` cycles through all 20 in order.
 *
 * Metal albedos are taken from PBR reference tables (Naty Hoffman,
 * "Physics and Math of Shading", SIGGRAPH 2013) and lightly tweaked
 * for terminal-renderer contrast.
 * ─────────────────────────────────────────────────────────────────── */
typedef struct {
  V3 albedo;            /* body diffuse colour                      */
  V3 specular;          /* F0 — metal: matches albedo; die: white   */
  V3 emissive;          /* self-glow (added after lighting)         */
  float diffuse_weight; /* 0.10..0.90 — metal/dielectric scale      */
  const char *name;
} Theme;

static const Theme g_themes[] = {
    /* === METALS (12) — spec hue MATCHES albedo, low diffuse_weight ===
     * Real metals reflect nearly all incident light specularly. Their
     * F0 (Fresnel at normal incidence) is what tints the highlight,
     * giving each metal its signature colour. */

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
     * Neon plasma is a self-emissive material. The albedo is the dim
     * "off" tube colour; the emissive value glows hot pink even in
     * shadow because emissive is added AFTER lighting. */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f, 0.02f, 0.10f},
     {0.80f, 0.80f, 1.00f},
     {1.00f, 0.20f, 0.85f},
     0.20f,
     "neon"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

/* color_init() and draw_color() are RENDER, not config — they live in the
 * RENDER layer at §7.5 (below), keeping §4 pure data. See ARCHITECTURE. */

/* ── §5 LOGIC: ray-AABB intersection (pure) — THE CORE
 * ────────────────────────────── *
 *
 * The most important section in this file. Three short functions
 * implementing the slab method:
 *
 *   §5.1 slab_test       — solve the 1D ray-vs-interval problem
 *   §5.2 ray_aabb        — combine three slab results into a hit
 *   §5.3 face_edge_dist  — fraction-to-edge of a hit point on its
 *                          face (used by wireframe mode)
 *
 * All three operate in OBJECT space. The cube is axis-aligned, centred
 * at the origin, with half-extent CUBE_S.
 *
 * On hit, ray_aabb populates a BoxHit struct so the renderer can:
 *   - shade the hit (needs `t_hit` and `normal_obj`),
 *   - run debug overlays (needs `axis_at_enter`, `t_enter`, `t_exit`),
 *   - run wireframe mode (needs the world-/object-space hit point).
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * BoxHit — the "hit record" for the slab method: everything §5's ray-AABB
 * intersection learns about one ray, packed so the renderer (§8) and the debug
 * overlays (§7) need no recomputation. The slab test intersects the ray with
 * three axis-aligned slabs and keeps the overlap interval [t_enter, t_exit];
 * the cube is hit iff that interval is non-empty and in front of the camera.
 * Filled in OBJECT space (cube axis-aligned at the origin); §8 rotates the
 * normal back to world space for lighting.
 *   hit           1 if the ray meets the box, else 0 (test this first).
 *   axis_at_enter which axis {0,1,2}=X,Y,Z tightened t_enter — i.e. which face
 *                 was entered; it selects the face normal. Also diagnostic
 *                 (DEBUG_AXIS colours by it).
 *   t_hit         ray parameter of the visible surface: t_enter normally, or
 *                 t_exit when the camera is INSIDE the box (we see the back).
 *   t_enter       largest per-axis slab ENTRY (inside all three slabs).
 *                 Exposed for the DEBUG_INTERVAL overlay.
 *   t_exit        smallest per-axis slab EXIT (leaving any slab). Exposed for
 *                 DEBUG_INTERVAL; t_exit−t_enter is the chord length.
 *   normal_obj    outward face normal at the hit (one of ±X/±Y/±Z), object
 *                 space; the input to every term in §6.
 *   inside        1 if the camera origin began inside the box (t_enter < 0),
 *                 so t_hit fell back to the exit face.
 * Ref: Kay & Kajiya (1986) slab method; Ericson, "Real-Time Collision
 *      Detection" §5.3.3; Quilez box intersector.
 */
typedef struct {
  int hit;           /* 1 = ray meets the box, 0 = miss (test first)   */
  int axis_at_enter; /* {0,1,2}=X,Y,Z — which slab tightened t_enter    */
  float t_hit;       /* visible-surface t: t_enter, or t_exit if inside */
  float t_enter;     /* max per-axis slab entry  — DEBUG_INTERVAL       */
  float t_exit;      /* min per-axis slab exit   — DEBUG_INTERVAL       */
  V3 normal_obj;     /* outward face normal ±X/±Y/±Z, OBJECT space      */
  int inside;        /* 1 if camera origin was inside the box           */
} BoxHit;

/*
 * Box — the domain object this file renders: an axis-aligned cube centred on
 * the origin, the same half_extent on every axis, plus the orientation it is
 * currently spinning to. Geometry (half_extent) is fixed at scene_init;
 * spin_x/spin_y are advanced every tick by scene_advance(). The §5 slab math
 * keeps the cube axis-aligned and rotates the RAY into object space instead,
 * so the intersection stays in a fixed, simple frame.
 *   half_extent  the cube spans [-half_extent, +half_extent] on each axis.
 *   spin_x       live pitch about X (radians) — advanced by the tick.
 *   spin_y       live yaw   about Y (radians) — advanced by the tick.
 * Ref: AABB / slab method — Kay & Kajiya (1986); Ericson, "Real-Time
 *      Collision Detection" §5.3.
 */
typedef struct {
  float half_extent; /* cube spans [-half_extent, +half_extent] per axis */
  float spin_x;      /* live pitch (rad) — advanced by the tick          */
  float spin_y;      /* live yaw   (rad) — advanced by the tick          */
} Box;

/*
 * §5.1 slab_test — solve the 1D ray-vs-slab problem on one axis.
 *
 * Purpose: turn one axis's slab boundaries into a time interval
 *          [t_enter_slab, t_exit_slab]. Handles parallel rays.
 *
 * Inputs:
 *   ray_origin_i   the i-th component of ray origin (1D)
 *   ray_dir_i      the i-th component of ray direction (1D)
 *   half_extent    slab spans [-half_extent, +half_extent] on this axis
 *
 * Outputs (only valid on success):
 *   *out_t_enter   ray parameter at slab entry
 *   *out_t_exit    ray parameter at slab exit
 *   *out_enter_sign sign of the OUTWARD normal on the entry face
 *                   for this axis (it's −sign(ray_dir_i)).
 *
 * Returns: 1 if the ray ever sits inside this slab (possibly forever
 *            when parallel and inside),
 *          0 if the ray is parallel AND outside (forever-miss case).
 *
 * Pseudocode:
 *   if |ray_dir_i| < ε:                       (parallel to slab)
 *     if ray_origin_i ∉ [-h, +h]: return MISS forever
 *     else:                       no t-bound; t_enter = -∞, t_exit = +∞
 *   else:
 *     inv = 1 / ray_dir_i
 *     t0  = (-h - ray_origin_i) * inv
 *     t1  = ( h - ray_origin_i) * inv
 *     t_enter = min(t0, t1)
 *     t_exit  = max(t0, t1)
 *     enter_sign = -sign(ray_dir_i)
 *
 * Why min/max instead of branching on sign(ray_dir_i): when ray_dir_i
 * is negative, t0 and t1 swap roles (dividing by a negative flips
 * which is smaller). The min/max recovers the correct ordering with
 * one comparison, no sign branch. T2 has the geometric picture.
 */
static int slab_test(float ray_origin_i, float ray_dir_i, float half_extent,
                     float *out_t_enter, float *out_t_exit,
                     float *out_enter_sign) {
  if (fabsf(ray_dir_i) < PARALLEL_EPS) {
    /* Parallel to the two walls of this slab. */
    if (ray_origin_i < -half_extent || ray_origin_i > half_extent)
      return 0;            /* outside → permanent miss */
    *out_t_enter = -BIG_T; /* slab imposes no bound    */
    *out_t_exit = BIG_T;
    *out_enter_sign = 0.f; /* no entry face contribution */
    return 1;
  }
  float inv_dir = 1.f / ray_dir_i;
  float t0 = (-half_extent - ray_origin_i) * inv_dir;
  float t1 = (half_extent - ray_origin_i) * inv_dir;
  if (t0 < t1) {
    *out_t_enter = t0;
    *out_t_exit = t1;
  } else {
    *out_t_enter = t1;
    *out_t_exit = t0;
  }
  /* Entry-face outward normal: opposite sign to ray direction. */
  *out_enter_sign = (ray_dir_i > 0.f) ? -1.f : 1.f;
  return 1;
}

/*
 * §5.2 ray_aabb — slab-method ray vs axis-aligned box.
 *
 * Purpose: combine three slab tests (one per axis) into a single
 *          hit/miss decision and recover the entry face's normal.
 *
 * Inputs:  ray_origin, ray_dir   OBJECT-space ray
 *          half_extent           cube half-side (box = [-h, +h]^3)
 *
 * Output:  BoxHit struct populated for the hit, .hit=0 on miss.
 *
 * Pseudocode:
 *   t_enter = -∞;  t_exit = +∞;  axis_at_enter = -1
 *   for axis i in 0,1,2:
 *     slab_test → t_enter_slab, t_exit_slab, enter_sign
 *     if MISS: return MISS
 *     if t_enter_slab > t_enter:
 *       t_enter       = t_enter_slab
 *       axis_at_enter = i
 *       enter_sign_g  = enter_sign
 *     if t_exit_slab  < t_exit:  t_exit = t_exit_slab
 *     if t_enter > t_exit: return MISS
 *
 *   if t_exit < ε: return MISS              (whole interval behind)
 *   if t_enter < ε:                         (camera inside box)
 *     t_hit = t_exit
 *     find axis owning t_exit; enter_sign_g = +sign(ray_dir on that axis)
 *   else:
 *     t_hit = t_enter
 *
 *   normal_obj = (axis basis vector)[axis_at_enter] * enter_sign_g
 *   return HIT(t_hit, axis_at_enter, normal_obj)
 *
 * Tightening logic: t_enter only grows (max-of-slabs); t_exit only
 * shrinks (min-of-slabs). The moment they cross, no t can satisfy
 * all three slabs and we early-exit.
 */
static BoxHit ray_aabb(V3 ray_origin, V3 ray_dir, float half_extent) {
  /* Pack components into arrays for a clean axis loop. */
  float ray_origin_arr[3] = {ray_origin.x, ray_origin.y, ray_origin.z};
  float ray_dir_arr[3] = {ray_dir.x, ray_dir.y, ray_dir.z};

  BoxHit out = {.hit = 0};

  float t_enter = -BIG_T, t_exit = BIG_T;
  int axis_at_enter = -1;
  float enter_sign = 0.f;

  for (int axis = 0; axis < 3; axis++) {
    float t_enter_slab, t_exit_slab, slab_enter_sign;
    if (!slab_test(ray_origin_arr[axis], ray_dir_arr[axis], half_extent,
                   &t_enter_slab, &t_exit_slab, &slab_enter_sign))
      return out; /* parallel & outside → MISS */

    /* Tighten entry: take the LATEST start. */
    if (t_enter_slab > t_enter) {
      t_enter = t_enter_slab;
      axis_at_enter = axis;
      enter_sign = slab_enter_sign;
    }
    /* Tighten exit: take the EARLIEST end. */
    if (t_exit_slab < t_exit)
      t_exit = t_exit_slab;

    /* Empty intersection? No t can satisfy all three slabs. */
    if (t_enter > t_exit)
      return out;
  }

  if (t_exit < T_EPS)
    return out; /* whole interval behind us */

  out.t_enter = t_enter;
  out.t_exit = t_exit;

  /* Two cases:
   *   t_enter > T_EPS  → ordinary entry hit, t_hit = t_enter.
   *   t_enter ≤ T_EPS  → ray ORIGIN is INSIDE the box → return the
   *                       EXIT (t_hit = t_exit) and the exit-face
   *                       normal. We rebuild that normal from
   *                       sign(ray_dir) on the axis whose t_exit
   *                       matched the global t_exit. */
  float t_hit;
  if (t_enter > T_EPS) {
    t_hit = t_enter;
    out.inside = 0;
  } else {
    t_hit = t_exit;
    out.inside = 1;
    /* Find which axis owns t_exit, and use sign(ray_dir) — the ray
     * is LEAVING through that wall, so the normal points the same
     * way as ray_dir on that axis. */
    for (int axis = 0; axis < 3; axis++) {
      float t_enter_slab, t_exit_slab, slab_enter_sign;
      if (!slab_test(ray_origin_arr[axis], ray_dir_arr[axis], half_extent,
                     &t_enter_slab, &t_exit_slab, &slab_enter_sign))
        return out;
      if (fabsf(t_exit_slab - t_exit) < 1e-6f) {
        axis_at_enter = axis;
        enter_sign = (ray_dir_arr[axis] > 0.f) ? +1.f : -1.f;
        break;
      }
    }
    if (axis_at_enter < 0)
      return out;
  }

  /* Face normal: the axis basis vector with the recorded sign. */
  out.normal_obj = (V3){0.f, 0.f, 0.f};
  if (axis_at_enter == 0)
    out.normal_obj.x = enter_sign;
  else if (axis_at_enter == 1)
    out.normal_obj.y = enter_sign;
  else
    out.normal_obj.z = enter_sign;

  out.t_hit = t_hit;
  out.axis_at_enter = axis_at_enter;
  out.hit = 1;
  return out;
}

/*
 * §5.3 face_edge_dist — fraction-to-edge of a hit point on its face.
 *
 * Purpose: tell wireframe mode how close a hit is to the boundary of
 *          its face. Returns 0 exactly on an edge, 1 at the face
 *          centre.
 *
 * Inputs:
 *   point_obj      OBJECT-space hit point
 *   normal_obj     hit-face normal (one of ±X, ±Y, ±Z)
 *   half_extent    cube half-side
 *
 * Pseudocode:
 *   pick the two coordinates that LIE WITHIN the face (the two NOT
 *     aligned with the normal axis):
 *       if normal is ±X: u = point.y, v = point.z
 *       if normal is ±Y: u = point.x, v = point.z
 *       if normal is ±Z: u = point.x, v = point.y
 *   distance to the nearest u-edge = h - |u|
 *   distance to the nearest v-edge = h - |v|
 *   return min(du, dv) / h   ← normalised to [0, 1]
 *
 * Returned value is 0 exactly on an edge of the face and 1 at the
 * face centre. Wireframe mode draws cells with value < WIRE_THRESH.
 */
static float face_edge_dist(V3 point_obj, V3 normal_obj, float half_extent) {
  float u, v;
  if (fabsf(normal_obj.x) > 0.5f) {
    u = point_obj.y;
    v = point_obj.z;
  } else if (fabsf(normal_obj.y) > 0.5f) {
    u = point_obj.x;
    v = point_obj.z;
  } else {
    u = point_obj.x;
    v = point_obj.y;
  }

  float du = half_extent - fabsf(u);
  float dv = half_extent - fabsf(v);
  return fminf(du, dv) / half_extent;
}

/*
 * face_uv — return the in-face UV coordinates of a hit point.
 *
 * Used by the FACE-UV debug overlay. Outputs are in [-1, +1] (after
 * normalising by half_extent), so a centre-of-face hit reports (0,0)
 * and a corner reports (±1, ±1).
 */
static void face_uv(V3 point_obj, V3 normal_obj, float half_extent,
                    float *out_u, float *out_v) {
  float u, v;
  if (fabsf(normal_obj.x) > 0.5f) {
    u = point_obj.y;
    v = point_obj.z;
  } else if (fabsf(normal_obj.y) > 0.5f) {
    u = point_obj.x;
    v = point_obj.z;
  } else {
    u = point_obj.x;
    v = point_obj.y;
  }
  *out_u = u / half_extent;
  *out_v = v / half_extent;
}

/* ── §6 LOGIC: shading (pure) — surface → colour
 * ─────────────────────────────────────────────────────── *
 *
 * Once §5 has identified a hit point and its surface normal, §6 turns
 * those into an RGB colour. Four orthogonal modes (T9) are provided.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * ShadeMode — which surface→colour view §6 produces for a hit: four ORTHOGONAL
 * views of the same geometry (one lit look + three diagnostics) so the user can
 * SEE what the renderer computes. Cycled with `s`. Trailing MODE_N is the
 * member count, enabling `(mode + 1) % MODE_N` wraparound; order matches
 * k_mode_names[] and the switch in shade_surface() (§6). MODE_PHONG Blinn-Phong
 * lit material — the physically-motivated look. MODE_NORMAL  normal vector
 * mapped xyz→rgb — inspect the surface normals. MODE_WIRE    draw only cells
 * near a face edge (face_edge_dist < WIRE_THRESH) — the cube's wireframe
 * silhouette. MODE_DEPTH   distance-to-camera ramp — inspect depth ordering.
 * Ref: Phong (1975); Blinn (1977).
 */
typedef enum {
  MODE_PHONG = 0, /* Blinn-Phong lit material (the "real" look)    */
  MODE_NORMAL,    /* normal xyz → rgb (debug normals)              */
  MODE_WIRE,      /* edge-only wireframe (face_edge_dist band)     */
  MODE_DEPTH,     /* distance ramp (debug depth)                   */
  MODE_N          /* count — enables % MODE_N wraparound           */
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "wireframe",
                                           "depth"};

/* Three fixed world-space lights — POSITIONS, not directions.
 * Cinematic three-point setup:
 *   KEY  = warm above-front-right (the bright "sun")
 *   FILL = cool low-front-left    (lifts the shadow side)
 *   RIM  = bright back-low        (separates silhouette from bg)
 */
static const V3 LIGHT_KEY = {3.0f, 4.0f, -2.0f};
static const V3 LIGHT_FILL = {-4.0f, 1.0f, -1.0f};
static const V3 LIGHT_RIM = {0.5f, -1.0f, 5.0f};

/* Per-light gains for the three-point rig (visual weights, not physical).
 * KEY is the dominant light; FILL is diffuse-only; RIM is a wide backlight. */
#define KEY_DIFFUSE 1.00f  /* KEY  diffuse weight                           */
#define KEY_SPECULAR 1.30f /* KEY  specular weight (sharp highlight)        */
#define FILL_DIFFUSE 0.55f /* FILL diffuse weight (lifts the shadow side)   */
#define RIM_DIFFUSE 0.40f  /* RIM  diffuse weight                           */
#define RIM_SPECULAR 1.20f /* RIM  specular weight (silhouette kiss)        */
#define RIM_SHININESS 10.f /* RIM  specular exponent (wider than SHININESS) */

/*
 * add_phong_light — accumulate ONE white point-light's Blinn-Phong
 * contribution (diffuse + specular) onto a running colour. Extracted from
 * shade_phong so the three-point rig reads as three named lights; the
 * per-light recipe is identical, only the gains differ.
 *   light_dir = normalize(light_pos − point)        (lights are POSITIONS)
 *   diffuse   = max(0, N·light_dir) · diffuse_weight · diffuse_gain → albedo
 *   specular  = max(0, reflect(−light_dir,N)·V)^shininess · specular_gain → F0
 * v3reflect() reflects an OUTGOING vector, so we pass −light_dir: light arrives
 * along −light_dir and the reflection is what we compare against the view dir.
 * A diffuse-only light (e.g. FILL) passes specular_gain = 0. Pure (const
 * Theme*).
 */
static V3 add_phong_light(V3 colour, V3 light_pos, V3 point_world,
                          V3 normal_world, V3 view_dir, const Theme *th,
                          float diffuse_gain, float specular_gain,
                          float shininess) {
  V3 light_dir = v3norm(v3sub(light_pos, point_world));
  float diffuse = fmaxf(0.f, v3dot(normal_world, light_dir));
  colour = v3add(
      colour, v3scale(diffuse * th->diffuse_weight * diffuse_gain, th->albedo));
  V3 reflect_dir = v3reflect(v3scale(-1.f, light_dir), normal_world);
  float specular = powf(fmaxf(0.f, v3dot(reflect_dir, view_dir)), shininess);
  colour = v3add(colour, v3scale(specular * specular_gain, th->specular));
  return colour;
}

/*
 * shade_phong — three-point Phong with PURE WHITE lights.
 *
 * For each white light at position L_pos:
 *   light_dir   = normalize(L_pos − point_world)
 *   diffuse     = max(0, normal · light_dir)
 *   reflect_dir = reflect(−light_dir, normal)
 *   specular    = max(0, reflect_dir · view_dir)^SHININESS
 *
 * The lights are pure white — every material's distinctive look comes
 * from its OWN albedo and specular tint, not from a coloured key
 * filter. Under white light:
 *   gold ALBEDO + gold SPEC      → looks like gold
 *   blue ALBEDO + WHITE SPEC     → looks like blue plastic
 *   dark ALBEDO + WHITE SPEC     → looks like glass
 * Material-as-material, not material-tinted-by-light.
 *
 * Per-light weighting: KEY > FILL > RIM in diffuse; KEY and RIM
 * contribute specular; FILL is diffuse-only.
 *
 * Diffuse contribution is multiplied by th->diffuse_weight:
 *   metals       ≈ 0.15 — real metals reflect almost everything
 *                         specularly; near-zero diffuse.
 *   gems         ≈ 0.70 — saturated body + bright white spec.
 *   dielectrics  ≈ 0.85 — full body colour.
 *   glass        ≈ 0.10 — very dark body, bright spec fakes
 *                         transparency.
 *
 * Emissive is added AFTER the lighting sum and BEFORE the clamp, so
 * neon glows hot pink even in shadow.
 *
 * On a cube, each face is FLAT-LIT — the normal is constant across
 * the whole face, so diffuse and specular don't vary across pixels
 * of the same face. The visual result is six flat-shaded quads.
 */
static V3 shade_phong(V3 point_world, V3 normal_world, V3 view_dir,
                      const Theme *th) {
  /* ambient: a dim, flat fraction of the body albedo (fills the shadow). */
  V3 colour = v3scale(AMBIENT, th->albedo);

  /* three-point rig, all WHITE lights — the material tints itself: */
  colour = add_phong_light(colour, LIGHT_KEY, point_world, normal_world,
                           view_dir, th, KEY_DIFFUSE, KEY_SPECULAR, SHININESS);
  colour = add_phong_light(colour, LIGHT_FILL, point_world, normal_world,
                           view_dir, th, FILL_DIFFUSE, 0.f, SHININESS);
  colour =
      add_phong_light(colour, LIGHT_RIM, point_world, normal_world, view_dir,
                      th, RIM_DIFFUSE, RIM_SPECULAR, RIM_SHININESS);

  /* emissive: added AFTER lighting, BEFORE clamp → neon glows even in shadow.
   */
  colour = v3add(colour, th->emissive);
  return v3clamp01(colour);
}

/*
 * shade_normal — RGB-encode the surface normal as a colour.
 *
 *   N ∈ [−1,1]³   →   (N+1)/2 ∈ [0,1]³
 *
 * For an axis-aligned cube each face has a constant normal so each
 * face becomes a single flat colour:
 *   +X (1.0, 0.5, 0.5)   −X (0.0, 0.5, 0.5)
 *   +Y (0.5, 1.0, 0.5)   −Y (0.5, 0.0, 0.5)
 *   +Z (0.5, 0.5, 1.0)   −Z (0.5, 0.5, 0.0)
 * Cleanest possible diagnostic for normal correctness.
 */
static V3 shade_normal(V3 normal_world) {
  return (V3){normal_world.x * 0.5f + 0.5f, normal_world.y * 0.5f + 0.5f,
              normal_world.z * 0.5f + 0.5f};
}

/*
 * shade_wire — wireframe edge tint.
 *
 * Called only for cells where face_edge_dist returned a value below
 * WIRE_THRESH. Saturation rises toward the edge centre (edge_dist=0)
 * and falls farther away. The face's normal-encoded colour serves as
 * the base hue so each edge inherits its face's colour.
 */
static V3 shade_wire(V3 normal_world, float edge_dist) {
  V3 colour = shade_normal(normal_world);
  /* Saturation peaks at the edge centre (k=1) and fades to k=0 at
   * the WIRE_THRESH boundary. */
  float k = 1.f - edge_dist / WIRE_THRESH;
  return v3clamp01(v3scale(0.6f + 0.4f * k, colour));
}

/*
 * shade_depth — encode hit distance as brightness (closer = brighter).
 *
 *   depth_norm = 1 − min(t / t_max, 1)
 *   bright     = depth_norm²              (steeper falloff)
 *
 * Sanity check: in DEPTH mode, the brightest pixel sits at the
 * nearest point on the visible silhouette.
 */
static V3 shade_depth(float t_hit, float distance_max, const Theme *th) {
  float depth_norm = 1.f - fminf(t_hit / distance_max, 1.f);
  depth_norm = depth_norm * depth_norm;
  return v3clamp01(v3scale(depth_norm, th->albedo));
}

/*
 * rec601_luma — perceptual brightness from RGB (Rec. 601).
 *
 *   Y = 0.299·R + 0.587·G + 0.114·B
 *
 * Used to choose the ASCII ramp character for a coloured pixel.
 */
static inline float rec601_luma(V3 c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/*
 * normal_vis_luma — brightness for the MODE_NORMAL view. There the colour IS
 * the normal remapped (N+1)/2 → RGB, so plain Rec.601 luma reads oddly; this
 * weights the remapped channels green-heavy (0.30/0.60/0.10) to match the
 * eye's "green is brightest" intuition for normal-map visualisations.
 */
static inline float normal_vis_luma(V3 normal_world) {
  return (normal_world.x * 0.5f + 0.5f) * 0.30f +
         (normal_world.y * 0.5f + 0.5f) * 0.60f +
         (normal_world.z * 0.5f + 0.5f) * 0.10f;
}

/*
 * shade_surface — produce the on-screen colour + brightness for one hit in the
 * selected ShadeMode (`s` cycles it). Returns 0 when the pixel should be left
 * blank — only MODE_WIRE does this, skipping face interiors so just the cube's
 * edges draw. The single place a hit becomes a drawable cell, so render()'s
 * loop reads as steps. MODE_DEPTH measures against a far plane at
 * cam_dist·DEPTH_FAR_SCALE. Pure: const Theme*, writes only its out-params.
 */
static int shade_surface(ShadeMode mode, const BoxHit *hit, V3 hit_point_obj,
                         V3 hit_point_world, V3 normal_world, V3 view_dir,
                         float cam_dist, float half_extent, const Theme *th,
                         V3 *out_colour, float *out_luminance) {
  switch (mode) {
  default:
  case MODE_PHONG:
    *out_colour = shade_phong(hit_point_world, normal_world, view_dir, th);
    *out_luminance = rec601_luma(*out_colour);
    break;
  case MODE_NORMAL:
    *out_colour = shade_normal(normal_world);
    *out_luminance = normal_vis_luma(normal_world);
    break;
  case MODE_WIRE: {
    float edge_dist =
        face_edge_dist(hit_point_obj, hit->normal_obj, half_extent);
    if (edge_dist > WIRE_THRESH)
      return 0; /* face interior — nothing to draw */
    *out_colour = shade_wire(normal_world, edge_dist);
    *out_luminance =
        WIRE_LUMA_BASE + WIRE_LUMA_GAIN * (1.f - edge_dist / WIRE_THRESH);
    break;
  }
  case MODE_DEPTH:
    *out_colour = shade_depth(hit->t_hit, cam_dist * DEPTH_FAR_SCALE, th);
    *out_luminance = rec601_luma(*out_colour);
    break;
  }
  return 1;
}

/* ── §7 LOGIC: debug-overlay decisions (pure)
 * ──────────────────────────────────────────────── *
 *
 * Three overlays that REPLACE the normal shaded colour with a
 * visualisation of the §5 intersection's intermediate state. Cycled
 * with `d`. Each teaches one specific aspect of the slab method —
 * see T10.
 *
 * The overlays read BoxHit fields populated by §5 plus a few values
 * the renderer (§8) computed locally (object-space hit point for the
 * FACE-UV overlay).
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * DebugMode — an overlay that REPLACES the shaded colour with a picture of the
 * slab method's internal state, so §5 is not a black box. Each reads BoxHit
 * fields already computed (DEBUG_FACE_UV also uses the object-space hit point
 * the renderer has on hand). Cycled with `d`; trailing DEBUG_N is the count for
 * `% DEBUG_N` wraparound; order matches k_debug_names[] and apply_debug() (§7).
 *   DEBUG_OFF       no overlay — keep the §6 shaded colour.
 *   DEBUG_AXIS      tint by axis_at_enter (R/G/B = entered face X/Y/Z).
 *   DEBUG_INTERVAL  brightness from the slab overlap width (t_exit − t_enter).
 *   DEBUG_FACE_UV   colour by the hit's in-face (u,v) coordinates.
 */
typedef enum {
  DEBUG_OFF = 0,  /* no overlay — keep the shaded colour            */
  DEBUG_AXIS,     /* tint by axis_at_enter (R/G/B = X/Y/Z face)     */
  DEBUG_INTERVAL, /* brightness from slab width (t_exit − t_enter)  */
  DEBUG_FACE_UV,  /* colour by in-face (u,v) coordinates            */
  DEBUG_N         /* count — enables % DEBUG_N wraparound           */
} DebugMode;
static const char *const k_debug_names[] = {"off", "axis", "interval",
                                            "face-uv"};

/* Helper: simple two-stop colour gradient driven by t ∈ [0,1]. */
static V3 gradient_cold_hot(float t) {
  if (t < 0.f)
    t = 0.f;
  if (t > 1.f)
    t = 1.f;
  V3 cold = (V3){0.10f, 0.20f, 0.95f};
  V3 hot = (V3){1.00f, 0.85f, 0.20f};
  return (V3){cold.x + (hot.x - cold.x) * t, cold.y + (hot.y - cold.y) * t,
              cold.z + (hot.z - cold.z) * t};
}

/*
 * apply_debug — given a hit and an overlay mode, return the overlay
 *               colour and luminance.
 *
 * AXIS       primary-colour-coded faces:
 *              red   = X-axis face won (axis_at_enter == 0)
 *              green = Y-axis face (axis_at_enter == 1)
 *              blue  = Z-axis face (axis_at_enter == 2)
 *            Teaches WHICH axis is "tightest" for each pixel — i.e.
 *            which slab tightened the t_enter race.
 *
 * INTERVAL   brightness from (t_exit − t_enter) — "thickness of the
 *            box-overlap interval" = "how much of the ray is inside".
 *            Bright at face centres, dim at silhouette edges.
 *            Teaches the geometric meaning of the interval overlap.
 *
 * FACE-UV    colour by the face's local (u,v) ∈ [-1,+1]² mapped to
 *            an RGB gradient. Reveals each face's parameterisation —
 *            the same coordinates wireframe uses to detect edges.
 *
 * Outputs are written to *out_colour and *out_luminance. The caller
 * passes them straight to draw_color().
 */
static void apply_debug(DebugMode mode, const BoxHit *hit, V3 hit_point_obj,
                        float half_extent, V3 *out_colour,
                        float *out_luminance) {
  switch (mode) {
  case DEBUG_AXIS: {
    V3 c = (V3){0.f, 0.f, 0.f};
    switch (hit->axis_at_enter) {
    case 0:
      c = (V3){0.95f, 0.20f, 0.20f};
      break; /* red    */
    case 1:
      c = (V3){0.20f, 0.95f, 0.30f};
      break; /* green  */
    case 2:
      c = (V3){0.30f, 0.40f, 1.00f};
      break; /* blue   */
    default:
      break;
    }
    *out_colour = c;
    *out_luminance = 0.85f;
    break;
  }
  case DEBUG_INTERVAL: {
    /* Interval thickness in object-space units, normalised by
     * the cube's diagonal (≈ 2·sqrt(3)·half_extent ≈ 3.46·h).
     * For a head-on view through the centre the thickness is
     * 2·half_extent, normalised to ≈ 0.58 — comfortably bright. */
    float thickness = hit->t_exit - hit->t_enter;
    float norm = thickness / (2.f * sqrtf(3.f) * half_extent);
    if (norm < 0.f)
      norm = 0.f;
    if (norm > 1.f)
      norm = 1.f;
    *out_colour = gradient_cold_hot(norm);
    *out_luminance = 0.3f + 0.65f * norm;
    break;
  }
  case DEBUG_FACE_UV: {
    float u, v;
    face_uv(hit_point_obj, hit->normal_obj, half_extent, &u, &v);
    /* (u,v) ∈ [-1,+1] → (R,G) ∈ [0,1]. Blue stays at 0.5 so the
     * overall brightness is constant — only hue changes. */
    *out_colour = (V3){u * 0.5f + 0.5f, v * 0.5f + 0.5f, 0.5f};
    *out_luminance = 0.85f;
    break;
  }
  default:
    /* DEBUG_OFF — caller keeps its own colour. */
    break;
  }
}

/* ── §7.5 RENDER: colour output — ncurses pairs + cell painter ─────── */

static int g_have_256; /* 1 if 256-colour cube is available, 0 = mono */

/*
 * color_init — bind ncurses colour pairs.
 *
 * Pair scheme:
 *   1..216    6×6×6 RGB cube (xterm 16..231) — used by draw_color().
 *   217       PAIR_HUD  — bright yellow on default bg.
 *   218       PAIR_HINT — bright cyan on default bg.
 */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_have_256 = (COLORS >= 256);
  if (g_have_256) {
    for (int i = 0; i < 216; i++)
      init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
  }
  init_pair(PAIR_HUD, 226, -1);
  init_pair(PAIR_HINT, 51, -1);
}

/*
 * draw_color — paint one cell with a colour and a luminance.
 *
 * Inputs : row, col       terminal cell
 *          colour         0..1 RGB triplet
 *          luminance      0..1 brightness picking the ramp character
 */
static void draw_color(int row, int col, V3 colour, float luminance) {
  if (luminance < 0.f)
    luminance = 0.f;
  if (luminance > 1.f)
    luminance = 1.f;
  char ch = k_ramp[(int)(luminance * (RAMP_LEN - 1))];

  if (g_have_256) {
    int red_5 = (int)(colour.x * 5.f + 0.5f);
    if (red_5 > 5)
      red_5 = 5;
    int green_5 = (int)(colour.y * 5.f + 0.5f);
    if (green_5 > 5)
      green_5 = 5;
    int blue_5 = (int)(colour.z * 5.f + 0.5f);
    if (blue_5 > 5)
      blue_5 = 5;
    int pair_idx = PAIR_CUBE_BASE + red_5 * 36 + green_5 * 6 + blue_5;
    attron(COLOR_PAIR(pair_idx));
    mvaddch(row, col, (chtype)(unsigned char)ch);
    attroff(COLOR_PAIR(pair_idx));
  } else {
    mvaddch(row, col, (chtype)(unsigned char)ch);
  }
}

/* ── §8 RENDER: per-pixel frame → screen
 * ────────────────────────────────────────────────── *
 *
 * One ray per terminal cell. The pipeline:
 *
 *   1. Compute camera basis (forward / right / up — fixed; the CUBE
 *      rotates, not the camera).
 *   2. Pre-build the WORLD↔OBJECT rotation matrix from the current
 *      angles. Used as Mᵀ on the ray (T7) and as M on the normal.
 *   3. For each cell:
 *        a. screen u, v in [-1,1] with FOV expansion + ASPECT correction
 *        b. world ray (origin, direction)
 *        c. object-space ray via mat3_mulT
 *        d. ray_aabb()
 *        e. on hit: world-space hit + normal, shade, optionally apply
 *           debug overlay, draw
 *
 * The bottom row is left blank so the cyan hint strip in §9 has
 * somewhere to live.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * primary_ray_dir — the camera ray through one terminal cell, in WORLD space.
 * cell → normalized screen coords in [-1,1] (centre origin) → scale by the
 * half-FOV tangent → divide the VERTICAL axis by ASPECT so the terminal's tall
 * cells don't squash the square cube into a rectangle → combine with the camera
 * basis and normalize. Both axes use the WIDTH centre, so x and y share one
 * pixel scale before the aspect correction. Pure: builds a direction.
 */
static V3 primary_ray_dir(int col, int row, float screen_centre_x,
                          float screen_centre_y, float fov_half_tan,
                          V3 view_right, V3 view_up, V3 view_forward) {
  float screen_u = (col - screen_centre_x) / screen_centre_x * fov_half_tan;
  float screen_v =
      -(row - screen_centre_y) / screen_centre_x * fov_half_tan / ASPECT;
  return v3norm(v3add(view_forward, v3add(v3scale(screen_u, view_right),
                                          v3scale(screen_v, view_up))));
}

static void render(const Box *box, const Theme *th, ShadeMode shade_mode,
                   DebugMode debug_mode, float cam_dist, int cols, int rows) {
  float fov_half_tan = tanf(FOV_DEG * (float)M_PI / 360.f);

  /* §8.1 — WORLD↔OBJECT rotation. (Mᵀ goes WORLD→OBJECT for the ray;
   *         M goes OBJECT→WORLD for the normal.) */
  Mat3 rotation = mat3_rotation(box->spin_x, box->spin_y);

  /* §8.2 — camera basis (fixed). */
  V3 camera_origin = {0.f, 0.f, -cam_dist};
  V3 view_forward = {0.f, 0.f, 1.f};
  V3 view_right = {1.f, 0.f, 0.f};
  V3 view_up = {0.f, 1.f, 0.f};

  float screen_centre_x = cols * 0.5f;
  float screen_centre_y = rows * 0.5f;

  /* §8.3 — Mᵀ applied to the camera origin once (it doesn't depend
   *         on the pixel). */
  V3 ray_origin_obj = mat3_mulT(rotation, camera_origin);

  /* §8.4 — primary loop: one camera ray per cell, read top-to-bottom. */
  for (int row = 0; row < rows - 1; row++) {
    for (int col = 0; col < cols; col++) {
      V3 ray_dir_world =
          primary_ray_dir(col, row, screen_centre_x, screen_centre_y,
                          fov_half_tan, view_right, view_up, view_forward);
      V3 ray_dir_obj =
          mat3_mulT(rotation, ray_dir_world); /* world → object (T7) */

      BoxHit hit = ray_aabb(ray_origin_obj, ray_dir_obj, box->half_extent);
      if (!hit.hit)
        continue;

      /* Reconstruct the hit in OBJECT and WORLD space (t is identical in both —
       * a pure rotation preserves distance). */
      V3 hit_point_obj = v3add(ray_origin_obj, v3scale(hit.t_hit, ray_dir_obj));
      V3 hit_point_world =
          v3add(camera_origin, v3scale(hit.t_hit, ray_dir_world));
      V3 normal_world = mat3_mul(rotation, hit.normal_obj);
      V3 view_dir = v3norm(v3sub(camera_origin, hit_point_world));

      /* Shade for the active mode (MODE_WIRE skips face interiors). */
      V3 colour;
      float luminance;
      if (!shade_surface(shade_mode, &hit, hit_point_obj, hit_point_world,
                         normal_world, view_dir, cam_dist, box->half_extent, th,
                         &colour, &luminance))
        continue;

      if (debug_mode != DEBUG_OFF)
        apply_debug(debug_mode, &hit, hit_point_obj, box->half_extent, &colour,
                    &luminance);

      draw_color(row, col, colour, luminance);
    }
  }
}

/* ── §9 RENDER: HUD → screen ──────────────────────────────────────────────────
 * *
 *
 * Two-row HUD per project spec:
 *   row 0    yellow status (right-aligned) + yellow mode label (left)
 *   rows-1   cyan key-hint strip (BOLD; never DIM, must read against
 *            any animation behind it)
 *
 * ─────────────────────────────────────────────────────────────────── */

static void hud_draw(int cols, int rows, float fps, const Theme *th,
                     ShadeMode shade_mode, DebugMode debug_mode, float cam_dist,
                     int paused) {
  /* §9.1 right-aligned status. fps lives here so it never gets
   * clipped by a long mode label on narrow terminals. */
  char status[96];
  snprintf(status, sizeof status, " %5.1f fps  dist:%.1f  %-9s  %s ",
           (double)fps, (double)cam_dist, th->name,
           paused ? "PAUSED " : "running");
  int status_len = (int)strlen(status);
  if (status_len > cols)
    status_len = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - status_len, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* §9.2 left-aligned mode + debug labels (yellow without bold). */
  char left_label[80];
  snprintf(left_label, sizeof left_label, " mode:%-9s  debug:%-8s ",
           k_mode_names[shade_mode], k_debug_names[debug_mode]);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, "%s", left_label);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* §9.3 bottom-left cyan key-hint strip (BOLD, ASCII only). */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(
      rows - 1, 0,
      " q:quit  spc/p:pause  s:mode  d:debug  t:theme  r:reset  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9.5 SCENE — runtime state aggregate + lifecycle ───────────────── *
 *
 * One struct holding everything that changes at runtime, so the app loop
 * below reads like a table of contents. Functions elsewhere take the
 * NARROWEST type they need (const Box*, const Theme*, scalars); only the
 * lifecycle orchestrators here take Scene*, so "everything hangs off
 * Scene" never re-couples the layers.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  /* WHAT is simulated — the one domain object (its orientation lives inside
   * it, so there are no loose angle fields here). */
  Box box; /* the spinning cube (geometry + orientation)     */

  /* HOW the user views it — RENDER/view knobs, grouped by THAT concept (not
   * by "things the keyboard changes"): none feed the simulation, they only
   * change the picture. All toggled in §10.5. */
  int theme_idx;        /* index into g_themes[] (read mod THEME_N)       */
  ShadeMode shade_mode; /* which §6 surface→colour view to run            */
  DebugMode debug_mode; /* which §7 overlay (DEBUG_OFF = none)            */
  float cam_dist;       /* camera pull-back along -Z; +/- clamp it to     */
                        /* [CAM_DIST_MIN, CAM_DIST_MAX]                   */
  int paused;           /* DELAYS gate: when set, scene_advance skips spin */

  /* WHERE we draw — terminal size, re-read on SIGWINCH (§10.1) so render and
   * the HUD always match the real window. */
  int cols, rows; /* terminal columns × rows                        */

  /* frame pacing — PERFORMANCE readout only; never affects the image. Once
   * per ~0.5 s window: fps = fps_frames * 1e9 / fps_accum_ns, then both
   * accumulators reset (a rolling average that smooths per-frame jitter). */
  float fps;              /* last computed rolling frames/sec (for the HUD) */
  long long fps_accum_ns; /* nanoseconds summed in the current window       */
  int fps_frames;         /* frames summed in the current window            */
} Scene;

/*
 * scene_init — opening state: cube geometry + zero spin, default view knobs,
 * current terminal size. Orchestrator: takes the whole Scene.
 */
static void scene_init(Scene *s) {
  s->box.half_extent = CUBE_S;
  s->box.spin_x = 0.f;
  s->box.spin_y = 0.f;
  s->theme_idx = 0;
  s->shade_mode = MODE_PHONG;
  s->debug_mode = DEBUG_OFF;
  s->cam_dist = CAM_DIST_DEF;
  s->paused = 0;
  getmaxyx(stdscr, s->rows, s->cols);
  s->fps = 0.f;
  s->fps_accum_ns = 0;
  s->fps_frames = 0;
}

/*
 * scene_reset — the `r` key: return the spin to zero. That orientation is the
 * only evolving simulation state; view knobs (theme, cam, mode) are untouched.
 * Orchestrator: takes Scene*. A USER EVENT, not part of the tick.
 */
static void scene_reset(Scene *s) {
  s->box.spin_x = 0.f;
  s->box.spin_y = 0.f;
}

/*
 * scene_advance — one tick of state evolution: spin the cube (unless paused)
 * and fold dt into the rolling fps. The ONLY writer of simulation state inside
 * the tick; render/hud never mutate the Scene. Orchestrator: takes Scene*.
 */
static void scene_advance(Scene *s, long long dt_ns) {
  if (!s->paused) {
    float seconds = (float)dt_ns * 1e-9f;
    s->box.spin_y += ROT_Y * seconds;
    s->box.spin_x += ROT_X * seconds;
  }
  s->fps_accum_ns += dt_ns;
  s->fps_frames++;
  if (s->fps_accum_ns >= 500000000LL) {
    s->fps = (float)s->fps_frames * 1e9f / (float)s->fps_accum_ns;
    s->fps_accum_ns = 0;
    s->fps_frames = 0;
  }
}

/* ── §10 APP / TICK — combines all layers (see ARCHITECTURE)
 * ────────────────────────────────────────────────────────── */

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
  typeahead(-1); /* prevent input tearing */
  atexit(cleanup);
  color_init();

  Scene scene;
  scene_init(&scene);

  long long frame_ns = 1000000000LL / TARGET_FPS;
  long long last = clock_ns();

  while (g_run) {
    /* §10.1 USER EVENT — apply pending SIGWINCH (control state, not the tick).
     */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, scene.rows, scene.cols);
    }

    /* §10.2 PERFORMANCE — wall-clock dt with spiral-of-death cap. */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* §10.3 TICK — advance simulation + fps (the only per-tick Scene mutation;
     *         spin is skipped while paused — the DELAYS layer; the `r` key also
     *         zeroes the spin, but as a USER EVENT in §10.5). */
    scene_advance(&scene, dt);

    /* §10.4 RENDER combine — the ONE place the layers meet, in order:
     *         erase -> render -> hud_draw -> doupdate. Reads state, never
     * writes it. */
    const Theme *th = &g_themes[scene.theme_idx % THEME_N];
    long long frame_start = clock_ns();
    erase();
    render(&scene.box, th, scene.shade_mode, scene.debug_mode, scene.cam_dist,
           scene.cols, scene.rows);
    hud_draw(scene.cols, scene.rows, scene.fps, th, scene.shade_mode,
             scene.debug_mode, scene.cam_dist, scene.paused);
    wnoutrefresh(stdscr);
    doupdate();

    /* §10.5 USER EVENTS — mutate view/control state (theme, mode, cam, pause,
     *         reset, quit); NOT part of the tick, never advance simulation. */
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
      scene.shade_mode = (ShadeMode)((scene.shade_mode + 1) % MODE_N);
      break;
    case 'd':
    case 'D':
      scene.debug_mode = (DebugMode)((scene.debug_mode + 1) % DEBUG_N);
      break;
    case 't':
      scene.theme_idx = (scene.theme_idx + 1) % THEME_N;
      break;
    case 'T':
      scene.theme_idx = (scene.theme_idx + THEME_N - 1) % THEME_N;
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

    /* §10.6 PERFORMANCE — frame cap. */
    clock_sleep_ns(frame_ns - (clock_ns() - frame_start));
  }
  return 0;
}

/*
 * cube_raytrace.c — analytic ray-traced cube via the slab method.
 *
 * DEMO: A solid cube tumbling in space, lit by a three-point rig of white
 *       lights (each material tints itself).
 *       Cycle four shading modes (phong → normals → wireframe →
 *       depth) and three debug overlays (off → axis → interval →
 *       face-uv) to see how the slab method works on the inside.
 *       Watch the silhouette change from square (face-on) to hexagon
 *       (corner-on) as the cube rotates.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same skeleton, ONE quadratic
 *   raytracing/capsule_raytrace.c  — same skeleton, decomposed analytic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *
 * Section map (each section is tagged with its concern — CONFIG /
 * PERFORMANCE / LOGIC / RENDER / SCENE / APP; see the ARCHITECTURE block
 * at the bottom of the file for the full layer table):
 *   §1   CONFIG       — frame rate, FOV, cube geometry, gains, ramp, IDs
 *   §2   PERFORMANCE  — monotonic clock + sleep
 *   §3   LOGIC: math  — V3, Mat3 (object↔world transforms)
 *   §4   CONFIG/DATA  — Theme material table + 256-colour cube
 *   §5   LOGIC: core  — Box type + ray-AABB slab method
 *                       §5.1 slab_test  §5.2 ray_aabb  §5.3 face_edge_dist
 *   §6   LOGIC: shade — add_phong_light, phong/normals/wireframe/depth,
 *                       shade_surface dispatcher
 *   §7   LOGIC: debug — overlays that visualise the §5 intermediate state
 *   §7.5 RENDER       — colour output: ncurses pairs + cell painter
 *   §8   RENDER       — per-pixel frame (primary_ray_dir → shade → draw)
 *   §9   RENDER       — ncurses HUD (yellow status row, cyan hint strip)
 *   §9.5 SCENE        — runtime state aggregate + scene_init/reset/advance
 *   §10  APP / TICK   — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode   (phong → normals → wireframe → depth)
 *   d         cycle debug overlay (off → axis → interval → face-uv)
 *   t / T     cycle theme — 20 materials in 4 families:
 *               metals (12)  gold, silver, copper, bronze, brass, platinum,
 *                            titanium, iron, steel, chrome, mercury, aluminum
 *               gems    (4)  ruby, emerald, sapphire, amethyst
 *               dielectrics  plastic, glass, ceramic
 *               emissive     neon
 *   p / SPC   pause / resume rotation
 *   r         reset rotation angles to zero
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/cube_raytrace.c \
 *       -o cube_rt -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL (in this order, as prose).
 *   2. §1 config — the constants you can twiddle.
 *   3. §5 cube (the core math) — read AFTER tutorials T2-T5.
 *   4. §8 render — see how the math is fed by per-pixel rays.
 *   5. Other sections only if curious.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Long descriptive names everywhere — `t_enter` not `tn`,
 *   `axis_at_enter` not `near_axis`. Every line becomes a
 *   self-explanatory sentence.
 *
 *   Suffixes name COORDINATE SPACE explicitly:
 *     `_world`  scene-fixed coordinates (camera & lights live here)
 *     `_obj`    cube-local coordinates (axis-aligned at the origin)
 *
 *   When a single name lacks a space suffix it's space-independent
 *   (e.g. `half_extent`, `t_hit`).
 *
 * Background you need
 * ───────────────────
 *   - 1-D number-line intuition (interval [a,b], when do two intervals
 *     overlap?).
 *   - Linear interpolation along a parametric ray P(t) = O + t·D.
 *   - That a 3×3 rotation matrix has inverse = transpose.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Any prior raymarching/raytracing knowledge — the file teaches it.
 *   - Quadratic formulas — the slab method has none.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-vs-axis-aligned-box (AABB) by the
 *                  SLAB METHOD. An AABB is the INTERSECTION of three
 *                  pairs of parallel planes — three "slabs", one per
 *                  axis. Per-axis the ray spends t ∈ [t_enter_i,
 *                  t_exit_i] inside the slab. The ray is inside the
 *                  BOX only when all three intervals overlap, i.e.
 *                    t_enter = max(t_enter_x, t_enter_y, t_enter_z)
 *                    t_exit  = min(t_exit_x,  t_exit_y,  t_exit_z)
 *                  Hit iff t_enter ≤ t_exit AND t_exit > 0. The face
 *                  the ray entered through is exactly the slab whose
 *                  t_enter became the global maximum — and its
 *                  outward normal is one of ±X, ±Y, ±Z determined by
 *                  the sign of the ray direction along that axis.
 *
 * Data-structure : AABB ≡ (V3 min, V3 max). Stored implicitly as a
 *                  single half-extent CUBE_S because the box is
 *                  axis-aligned and centred at the origin in OBJECT
 *                  space; world-space rotation is applied to the RAY
 *                  (inverse-rotation trick), not to the box geometry.
 *
 * Rendering      : One ray per terminal cell. Phong with three
 *                  coloured world-space lights. Normals mode encodes N
 *                  as RGB (excellent diagnostic — each face is a single
 *                  flat colour). Wireframe mode draws only cells within
 *                  WIRE_THRESH of the nearest face edge. Depth mode
 *                  encodes hit distance as a brightness gradient.
 *
 * Performance    : O(3) per pixel — three single-slab solves, three
 *                  max/min tightenings. No square roots, no trig, no
 *                  iterations. The cube is the CHEAPEST primitive in
 *                  the raytracing folder.
 *
 * References     : Andrew Woo, "Fast Ray-Box Intersection",
 *                    in Andrew S. Glassner (ed.) "Graphics Gems" (1990).
 *                  Inigo Quílez, "Box — intersection",
 *                    https://iquilezles.org/articles/intersectors/
 *                  Kay & Kajiya, "Ray Tracing Complex Scenes",
 *                    SIGGRAPH '86 (original slab method paper).
 *                  Real-Time Rendering 4e §22.7 (ray-box variants).
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * To check if a ray hits a 3D box, ask THREE 1D questions:
 *   "When does the ray cross the X faces?"
 *   "When does it cross the Y faces?"
 *   "When does it cross the Z faces?"
 * Each answer is a time interval [t_enter, t_exit]. The ray is inside
 * the box iff all THREE intervals share a common moment. Three
 * intervals share a moment iff the LATEST entry is still earlier than
 * the EARLIEST exit. The slab method reduces "is the ray inside this
 * box?" to "do three 1D intervals overlap?" — and interval overlap is
 * two arithmetic comparisons.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a cube as a sandwich of three pairs of parallel walls:
 *
 *           ┌─────────────────────┐
 *          ╱                     ╱│
 *         ╱       Y-slab        ╱ │
 *        ┌─────────────────────┐  │ ← X-slab (left/right walls)
 *        │                     │  │
 *        │    box interior     │  │
 *        │  =  X ∩ Y ∩ Z slabs │ ╱
 *        │                     │╱  ← Z-slab (front/back walls)
 *        └─────────────────────┘
 *
 * As a ray flies through space, it spends a time interval inside each
 * slab:
 *
 *   t →  ───[==== X-slab ====]──────────────────
 *   t →  ─────────[========= Y-slab ===========]
 *   t →  ───[============ Z-slab ===]──────────
 *               ↑                ↑
 *               t_enter        t_exit  ← ray IS inside the box
 *                                         in this overlap region
 *
 * The ray is inside the box ONLY in the overlap of all three intervals.
 * `t_enter` is the most-restrictive lower bound (the LATEST start);
 * `t_exit` is the most-restrictive upper bound (the EARLIEST end). If
 * the upper bound has not run out yet by the time the latest slab
 * starts, the ray made it inside.
 *
 * The face that produced `t_enter` is the face the ray entered
 * through, because that's the slab that was the "last" to start
 * admitting the ray.
 *
 * Coordinate-system pipeline (T8 unpacks each step):
 *
 *      SCREEN ─FOV─→ CAMERA ─cam basis─→ WORLD ─Mᵀ─→ OBJECT
 *                                                       ↓
 *                                                  intersection
 *                                                       ↓
 *      OBJECT ────M────→ WORLD ──────→ shading ──→ pixel
 *
 * Everything left of "intersection" is per-frame ceremony; the
 * intersection happens in OBJECT space where the cube is axis-aligned.
 *
 * ALGORITHM IN STEPS  (per pixel)
 * ───────────────────────────────
 *  1. Build a primary ray (origin + direction) in WORLD space from the
 *     screen coordinates and the camera's basis.
 *  2. Apply Mᵀ to the ray to put it in OBJECT space (the cube stays
 *     axis-aligned there).
 *  3. For each axis i ∈ {0,1,2}:
 *        if ray_dir_i ≈ 0:
 *           if ray_origin_i is outside [-s, s]:  ray runs forever
 *               outside this slab → MISS.
 *           else:                              this slab imposes no
 *               t-bound (ray sits inside it permanently).
 *        else:
 *           t0 = (-s - ray_origin_i) / ray_dir_i
 *           t1 = (+s - ray_origin_i) / ray_dir_i
 *           t_enter_slab = min(t0, t1)
 *           t_exit_slab  = max(t0, t1)
 *           if t_enter_slab > t_enter:
 *              t_enter       = t_enter_slab
 *              axis_at_enter = i
 *              enter_sign    = -sign(ray_dir_i)
 *           if t_exit_slab  < t_exit:  t_exit = t_exit_slab
 *           if t_enter > t_exit:  early exit → MISS
 *  4. After three axes:
 *        if t_exit < ε:                    MISS (interval all behind us)
 *        if t_enter < ε:  t_hit = t_exit   ray ORIGIN is inside box;
 *                                          use the EXIT face instead
 *        else:            t_hit = t_enter  ordinary entry hit
 *        N_obj = ±axis_i for axis_at_enter, sign = enter_sign.
 *  5. Transform N_obj back to world (M·N_obj) and shade.
 *
 * KEY FORMULAS
 * ────────────
 *   For axis i, plane at position p, with ray_dir_i ≠ 0:
 *     t = (p − ray_origin_i) / ray_dir_i      [P(t) = O + t·D]
 *
 *   Per-slab interval for box [-s, s] on axis i:
 *     t0 = (−s − ray_origin_i) / ray_dir_i
 *     t1 = (+s − ray_origin_i) / ray_dir_i
 *     t_enter_slab = min(t0, t1)            [enter time]
 *     t_exit_slab  = max(t0, t1)            [exit  time]
 *
 *   Combine across three axes:
 *     t_enter = max(t_enter_x, t_enter_y, t_enter_z)
 *     t_exit  = min(t_exit_x,  t_exit_y,  t_exit_z)
 *
 *   Hit test:
 *     hit ⇔ t_enter ≤ t_exit AND t_exit > 0
 *
 *   Outward normal at entry:
 *     N_obj = e_i · (−sign(ray_dir_i))
 *     where i is the axis that set t_enter.
 *     (At t = t_enter the ray is just stepping ONTO that face, so the
 *      outward normal points OPPOSITE to the ray direction component
 *      on that axis.)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • ray_dir_i ≈ 0 (ray parallel to slab): use a 1e-9 tolerance. If
 *    ray_origin_i ∈ [-s, s] the slab imposes no bound; otherwise the
 *    ray never enters that slab → return MISS immediately.
 *  • Inside the box (t_enter < 0 < t_exit): the "entry" is in the
 *    past; we return t_exit and the EXIT face's normal. This is
 *    correct for self-intersecting hits — a ray starting inside a box
 *    leaves through one face.
 *  • Corner / edge hits: two slabs may produce numerically equal
 *    t_enter values — the dispatcher picks one (whichever was checked
 *    first whose tn was strictly greater). On a terminal grid this
 *    discontinuity is invisible because the pixel at the geometric
 *    corner is already a single cell.
 *  • t < T_EPS reject prevents self-intersection at t ≈ 0 when the
 *    camera grazes a face.
 *  • The inverse-rotation trick: rotating a cube means rotating 8
 *    vertices and 6 face equations. Keeping the cube fixed at the
 *    origin and rotating the RAY by Mᵀ is equivalent and costs ONE
 *    matrix×ray multiply per pixel — and lets us keep the box AABB,
 *    so the slab method's per-axis test stays a scalar division (a
 *    rotated box would need general-plane equations: 3 dot products
 *    per slab instead of 1 division).
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Static cube (press `r` to reset, then SPC to pause), viewed
 *    head-on: silhouette is a SQUARE of solid colour (one visible
 *    face). MODE_NORMAL paints it a single flat colour — the (0,0,−1)
 *    face is RGB(0.5, 0.5, 0).
 *  • Cube viewed from a corner (let it rotate ~45°/45°): silhouette
 *    is a HEXAGON, and you see THREE faces meeting at the central
 *    vertex forming a tri-radial "Y". MODE_NORMAL paints each face a
 *    different flat colour.
 *  • `d` → AXIS overlay: the three visible faces of the corner-on
 *    view should each show a different flat colour (red = X-axis face,
 *    green = Y-axis face, blue = Z-axis face).
 *  • `d` → INTERVAL overlay: bright at the centre of the cube
 *    silhouette (the ray plunges deepest there), dimmest at the
 *    silhouette edges (grazing rays).
 *  • `d` → FACE-UV overlay: each visible face shows a 2D gradient
 *    (red ↔ green) revealing its local UV parameterisation.
 *  • Worked-example check: head-on ray to a 1.6-cube at distance 3.2
 *    should hit at t = 2.4. (Cube spans [-0.8, +0.8]; closest face is
 *    z = -0.8, ray starts at z = -3.2, so t = (-0.8 - (-3.2)) / 1 = 2.4.)
 *  • Doubling CUBE_S doubles the silhouette in every dimension and
 *    halves the apparent distance — bringing the cube closer.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Ten short tutorials that build the algorithm from first principles.
 * Read in order; each builds on the previous.
 *
 *   T1  What problem are we solving?
 *   T2  The simplest case: ray vs 1D INTERVAL
 *   T3  Two axes: ray vs 2D RECTANGLE
 *   T4  Three axes: ray vs 3D BOX
 *   T5  Recovering the entry FACE (not just the time)
 *   T6  Edge cases — parallel rays + camera-inside-box
 *   T7  The inverse-rotation trick (rotate the RAY)
 *   T8  Coordinate systems at a glance
 *   T9  Four shading modes — same geometry, four pictures
 *   T10 White light is the lab; the material is the specimen
 *   T11 Metals vs dielectrics — F0, Fresnel, and the spec-tint trick
 *   T12 Three debug overlays — making the slab method visible
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT PROBLEM ARE WE SOLVING?
 * ────────────────────────────────
 * Given:
 *   • a ray  (origin O, unit direction D)
 *   • an axis-aligned box  [-s, +s]^3  (centred at origin, half-side s)
 * Find:
 *   • the smallest t > 0 such that O + t·D lies on the box's surface,
 *   • the outward unit normal at that point — one of the six axis
 *     basis vectors ±X, ±Y, ±Z, since every face of an axis-aligned
 *     box has a normal perpendicular to one of the coordinate axes.
 *
 * Why a box and not a more general shape? Because boxes are the
 * cheapest primitives in graphics — bounding-volume hierarchies use
 * them by the millions. AND every face has a constant normal, which
 * makes shading trivially flat (no smooth shading wanted on a cube).
 *
 * The trick we'll exploit: the box is the INTERSECTION of three
 * 1D slabs. Each slab is "the region between two parallel walls."
 * Intersection of slabs ↔ intersection of time intervals, which is
 * just max-of-mins / min-of-maxes.
 *
 * T2  THE SIMPLEST CASE: RAY VS 1D INTERVAL
 * ──────────────────────────────────────────
 * Strip the world down to one dimension. The "box" is the interval
 * [-s, +s] on the number line. The ray is a 1D parametric line
 *   P(t) = ray_origin + t · ray_dir.
 *
 * When does the ray sit inside the interval? When P(t) is between
 * -s and +s:
 *
 *     -s ≤ ray_origin + t · ray_dir ≤ +s
 *
 * Solve each side for t:
 *
 *     t0 = (-s - ray_origin) / ray_dir         when ray_dir ≠ 0
 *     t1 = (+s - ray_origin) / ray_dir
 *
 *     t_enter = min(t0, t1)        (whichever wall we cross first)
 *     t_exit  = max(t0, t1)        (whichever wall we cross last)
 *
 * The ray is inside the interval for t ∈ [t_enter, t_exit]. Done —
 * one division per wall, two comparisons, no quadratics.
 *
 * Why might (t0, t1) need a swap? Because if ray_dir < 0 (ray going
 * leftward), dividing by a negative flips the inequality, so what
 * "should" be t_enter ends up larger than t_exit. The min/max
 * swap fixes that without a separate sign branch.
 *
 * T3  TWO AXES: RAY VS 2D RECTANGLE
 * ──────────────────────────────────
 * Now do T2 on each axis independently. The ray spends time
 * [t_enter_x, t_exit_x] inside the X-strip (region between the left
 * and right walls), and time [t_enter_y, t_exit_y] inside the Y-strip
 * (region between the bottom and top walls).
 *
 *      t_enter_x          t_exit_x
 *           │                │
 *      ────[==================]────────────────────  X-strip interval
 *
 *               t_enter_y                     t_exit_y
 *                    │                            │
 *      ──────────────[============================]  Y-strip interval
 *
 * The ray is inside the rectangle iff it's inside BOTH strips at the
 * same moment — the OVERLAP of the two intervals:
 *
 *     t_enter = max(t_enter_x, t_enter_y)
 *     t_exit  = min(t_exit_x,  t_exit_y)
 *
 * Hit iff t_enter ≤ t_exit. The picture above gives a hit; if the
 * X-strip ended before the Y-strip began we'd see no overlap and the
 * ray would miss.
 *
 * T4  THREE AXES: RAY VS 3D BOX
 * ──────────────────────────────
 * Same trick, third axis. Now we have THREE slab intervals:
 *
 *     X-slab: [t_enter_x, t_exit_x]
 *     Y-slab: [t_enter_y, t_exit_y]
 *     Z-slab: [t_enter_z, t_exit_z]
 *
 * The ray is inside the box iff all three intervals overlap. Three
 * intervals overlap iff the LATEST start is ≤ the EARLIEST end:
 *
 *     t_enter = max(t_enter_x, t_enter_y, t_enter_z)
 *     t_exit  = min(t_exit_x,  t_exit_y,  t_exit_z)
 *
 *     hit ⇔ t_enter ≤ t_exit  AND  t_exit > 0
 *
 * That's the slab method in one line. The implementation in §5.2
 * walks the three axes once, tightens t_enter and t_exit on each
 * iteration, and bails the moment they cross.
 *
 * T5  RECOVERING THE ENTRY FACE (NOT JUST THE TIME)
 * ──────────────────────────────────────────────────
 * Knowing t_enter is enough to compute the hit POINT (it's just
 * O + t·D). But for shading we also need the surface NORMAL — and
 * the slab method gives us that too, almost for free.
 *
 * Insight: the axis whose t_enter became the GLOBAL maximum is the
 * "last" wall the ray crossed before getting inside. That axis's
 * face IS the entry face.
 *
 *   axis_at_enter = arg max over i of t_enter_i
 *
 * The entry face is on the +axis side or the -axis side depending on
 * which way the ray is going:
 *
 *   ray moving in +X direction → entered through the −X face
 *   ray moving in −X direction → entered through the +X face
 *
 * Hence enter_sign = -sign(ray_dir_i) for the chosen axis i. The
 * outward normal is just enter_sign × the axis basis vector:
 *
 *   axis_at_enter = 0 → N = (enter_sign, 0, 0)
 *   axis_at_enter = 1 → N = (0, enter_sign, 0)
 *   axis_at_enter = 2 → N = (0, 0, enter_sign)
 *
 * The code in §5.1 records enter_sign inside slab_test; §5.2's loop
 * remembers axis_at_enter as it tightens t_enter. By the time the
 * loop ends, normal computation is one assignment.
 *
 * T6  EDGE CASES — PARALLEL RAYS + CAMERA-INSIDE-BOX
 * ────────────────────────────────────────────────────
 * Two cases the naïve formula stumbles on:
 *
 * (a) Ray parallel to a slab (ray_dir_i ≈ 0).
 *     Dividing by ray_dir_i would be a division by zero. Fortunately
 *     the geometry tells us the answer:
 *       - If ray_origin_i ∈ [-s, +s] the ray sits inside this slab
 *         FOREVER. We treat the slab as imposing no t-bound (write
 *         t_enter = -∞, t_exit = +∞ for this axis).
 *       - If ray_origin_i ∉ [-s, +s] the ray is OUTSIDE this slab
 *         forever — no t can ever satisfy all three slabs. Return
 *         MISS immediately, no need to test the others.
 *
 * (b) Ray origin inside the box (t_enter < 0 < t_exit).
 *     The "entry" already happened in the past; we want the EXIT
 *     point instead. The slab method handles this with one branch:
 *       t_hit = (t_enter > 0) ? t_enter : t_exit
 *     The exit-face normal is found the same way as the entry
 *     normal but on the axis that owns t_exit, with the sign FLIPPED
 *     (the ray is leaving, so the normal points the same direction
 *     as ray_dir_i, not opposite).
 *
 * The implementation in §5.2 handles both cases.
 *
 * T7  THE INVERSE-ROTATION TRICK (ROTATE THE RAY)
 * ────────────────────────────────────────────────
 * Naive way: every frame, rotate the cube's 8 vertices and 6 face
 * equations by the current orientation matrix M. Cost grows with
 * scene size.
 *
 * Smarter way: keep the box AXIS-ALIGNED at the origin, and rotate
 * the RAY into the cube's local space:
 *
 *     ray_obj.origin    = Mᵀ · ray_world.origin
 *     ray_obj.direction = Mᵀ · ray_world.direction
 *
 * Why Mᵀ? Because rotation matrices are ORTHOGONAL — their inverse
 * equals their transpose. Computing Mᵀ is free (just access pattern);
 * computing M⁻¹ generally requires a determinant and division.
 *
 * After intersecting in OBJECT space we get a normal N_obj. Bring it
 * back to world space:
 *
 *     N_world = M · N_obj
 *
 * Cost: ONE matrix-vector multiply per ray. As a bonus, keeping the
 * box AABB in object space is what lets the slab method use scalar
 * division per axis instead of full plane equations — a rotated box
 * would force 3 dot products per slab.
 *
 * T8  COORDINATE SYSTEMS AT A GLANCE
 * ───────────────────────────────────
 * Four spaces appear in this file:
 *
 *      SCREEN          (col, row)         pixel grid, integer
 *        │
 *        │  pu = (col − cx)/cx · tan(FOV/2)
 *        │  pv = −(row − cy)/cx · tan(FOV/2) / ASPECT
 *        ▼
 *      CAMERA          (pu, pv)           normalised, axis-aligned
 *        │
 *        │  ray_world = camera_origin
 *        │  view_dir  = normalize(forward + pu·right + pv·up)
 *        ▼
 *      WORLD           (x, y, z)          scene-fixed coords; lights here
 *        │
 *        │  ray_obj.origin = Mᵀ · ray_world.origin
 *        │  ray_obj.dir    = Mᵀ · ray_world.dir
 *        ▼
 *      OBJECT          (x, y, z)          cube axis-aligned at origin
 *        │
 *        │  intersection (§5)
 *        ▼
 *      hit_point_obj, normal_obj
 *        │
 *        │  hit_point_world = camera_origin + t · view_dir   (in world!)
 *        │  normal_world    = M · normal_obj
 *        ▼
 *      WORLD shading (§6) → pixel colour → SCREEN.
 *
 * The hit point is computed in world space directly from the world
 * ray + the t we obtained — t is the same in both spaces (rotation
 * preserves distances). World-space hit points let shading use
 * world-space lights with no extra transforms.
 *
 * For wireframe mode we also need the hit point in OBJECT space to
 * measure distance-to-edge in face-local coordinates (§5.3).
 *
 * T9  FOUR SHADING MODES — SAME GEOMETRY, FOUR PICTURES
 * ──────────────────────────────────────────────────────
 * After we know the hit point and its normal we still have to turn
 * those into a colour. Four orthogonal modes (cycle with `s`):
 *
 *   PHONG       Three pure-WHITE lights (key + fill + rim). Diffuse =
 *               max(N·L, 0); specular = (R·V)^shininess. The material's
 *               distinctive colour comes from its OWN albedo and spec
 *               tint — the lights are white so each material reads as
 *               itself, not as a coloured filter. Diffuse weight per
 *               material: metals ≈ 0.15 (spec-dominated), gems ≈ 0.70,
 *               dielectrics ≈ 0.85, glass ≈ 0.10. Emissive (added after
 *               lighting, before clamp) lets neon glow in shadow.
 *   NORMALS     N → (N+1)/2, channel-by-channel. Each face becomes
 *               a SINGLE flat colour because every cube face has a
 *               constant normal. The six possible colours:
 *                 +X (1.0, 0.5, 0.5)   −X (0.0, 0.5, 0.5)
 *                 +Y (0.5, 1.0, 0.5)   −Y (0.5, 0.0, 0.5)
 *                 +Z (0.5, 0.5, 1.0)   −Z (0.5, 0.5, 0.0)
 *               The cleanest possible diagnostic for normal correctness.
 *   WIREFRAME   Draw only cells within WIRE_THRESH of a face edge in
 *               OBJECT-space face-UV coordinates (§5.3 face_edge_dist).
 *               Body interior is black; you see exactly the edges of
 *               the front-facing parts of the cube. From a corner-on
 *               view: 9 edges visible (the 6-edge silhouette plus the
 *               3 internal edges meeting at the central vertex).
 *   DEPTH       Brightness from t: closer = brighter. Useful as a
 *               sanity check (the centre of the visible silhouette
 *               should be the brightest spot).
 *
 * T10 WHITE LIGHT IS THE LAB; THE MATERIAL IS THE SPECIMEN
 * ─────────────────────────────────────────────────────────
 * A common amateur-lighting trick is to TINT the lights to flatter
 * each material — warm key for gold, cool key for silver, blue rim
 * for plastic. It looks slick at first glance but it CONFLATES two
 * orthogonal things: "what the LIGHT is doing" and "what the
 * MATERIAL is".
 *
 * The first-principles fix: keep all lights pure WHITE. Then any
 * colour the eye sees comes from the MATERIAL itself. This is how
 * PBR (physically-based rendering) sees the world, and the same
 * logic an art gallery uses — neutral white-balanced spotlights so
 * each painting shows its TRUE colours. A warm gallery light would
 * make every Mondrian lean orange.
 *
 * In this file, every per-light colour is unit white:
 *
 *     LIGHT_KEY  = (1, 1, 1)         pure white sun
 *     LIGHT_FILL = (1, 1, 1)         pure white fill
 *     LIGHT_RIM  = (1, 1, 1)         pure white rim
 *
 * The KEY/FILL/RIM weights still differ in INTENSITY (KEY brightest,
 * RIM dimmest, geometric falloff per light position) but they're all
 * achromatic. So:
 *
 *     gold  albedo + gold  spec   →  looks like real gold
 *     blue  albedo + WHITE spec   →  looks like real blue plastic
 *     dark  albedo + WHITE spec   →  looks like real glass
 *     dim   albedo + EMISSIVE     →  looks like real neon
 *
 * Material-as-material, not material-tinted-by-light.
 *
 * Try it:
 *  - Pause (`p`), cycle `t` through all 20 materials. Each material
 *    should be recognisable as ITSELF — gold reads gold, plastic
 *    reads plastic, glass reads glass, neon glows pink.
 *  - If the lights were tinted, every material would lean toward
 *    the dominant light's hue and the recognition would break.
 *
 * T11 METALS VS DIELECTRICS — F0, FRESNEL, AND THE SPEC-TINT TRICK
 * ─────────────────────────────────────────────────────────────────
 * Three numbers describe how light interacts with a surface:
 *
 *   ALBEDO   the body colour from sub-surface absorption — what you
 *            see when the surface is EVENLY illuminated and viewed
 *            head-on (no highlight).
 *   F0       the fraction of light reflected at NORMAL INCIDENCE
 *            (cell facing camera, 0° angle to view).
 *   FRESNEL  the curve raising reflectance to ≈ 1 at GRAZING angles
 *            (cell at 90° to view, like the rim of the silhouette).
 *
 * Real materials split into two regimes along the F0 axis:
 *
 *   METALS       F0 has the SAME TINT as the visible body colour.
 *                  gold's   measured F0 = (1.000, 0.766, 0.336)  ← yellow!
 *                  silver's measured F0 = (0.972, 0.960, 0.915)  ← white-ish
 *                  copper's measured F0 = (0.955, 0.638, 0.538)  ← pink-orange
 *                That tinted F0 is what makes a metal look "metallic"
 *                — its highlights reflect the metal's own colour
 *                because metals are conductors and light can't enter
 *                the bulk to be re-coloured by absorption. Whatever
 *                gets reflected gets reflected AT THE METAL'S TINT.
 *
 *   DIELECTRICS  F0 is achromatic: ~0.04 (4%) across the spectrum.
 *                Plastic, glass, gem, ceramic, wood, water — all of
 *                them. The body colour comes ENTIRELY from absorption
 *                inside the material; the highlight is a small WHITE
 *                fraction reflected off the surface.
 *
 * Why it matters: the DIFFERENCE between a gold cube and a yellow
 * plastic cube is not the body colour (both yellow) but the
 * highlight — gold's highlight is YELLOW, plastic's is WHITE. That
 * single tint difference is what makes "metal" look like metal and
 * "plastic" look like plastic.
 *
 * Our Theme struct encodes this directly:
 *
 *   typedef struct {
 *       V3 albedo;          body colour
 *       V3 specular;        F0:  metal → albedo;  dielectric → white
 *       V3 emissive;        self-glow (mostly 0; neon glows hot pink)
 *       float diffuse_weight;
 *       const char *name;
 *   } Theme;
 *
 * The `diffuse_weight` field captures the OTHER physical truth:
 * metals reflect almost everything specularly, with very little
 * diffuse contribution. So:
 *
 *   metal:        diffuse_weight = 0.15  (low; mostly specular reflects)
 *   gem:          diffuse_weight = 0.70  (saturated body + bright spec)
 *   plastic:      diffuse_weight = 0.85  (full body colour)
 *   glass:        diffuse_weight = 0.10  (dark body, bright white spec)
 *   neon:         diffuse_weight = 0.20  (low diffuse + huge emissive)
 *
 * `add_phong_light` makes this concrete (the KEY light, gains
 * KEY_DIFFUSE / KEY_SPECULAR):
 *
 *     // Diffuse contribution: WHITE light × albedo × diffuse_weight
 *     colour += diffuse · diffuse_weight · KEY_DIFFUSE · albedo
 *
 *     // Specular contribution: WHITE light × F0
 *     //   metal       F0 = tinted albedo  → tinted highlight
 *     //   dielectric  F0 = (1, 1, 1)      → white highlight
 *     colour += specular · KEY_SPECULAR · spec_F0
 *
 * On a CUBE this contrast is especially dramatic because each face
 * has a CONSTANT normal, so a single face shows the metal's
 * highlight as a uniform tinted glow. A flat-shaded gold face looks
 * like burnished gold; a flat-shaded plastic face looks like an
 * injection-moulded part with a single pure-white highlight bloom.
 *
 * Try it:
 *  - Cycle to gold (`t` until name says "gold") — watch the bright
 *    face. The whole face is WARM YELLOW.
 *  - Cycle to plastic — same geometry, same lighting; the bright
 *    face is BLUE in the body but the highlight band fades to
 *    WHITE. That's the dielectric signature.
 *  - Cycle to ruby (red gem dielectric) — body deep red, highlight
 *    near-white. Same dielectric signature.
 *  - Cycle to neon — even on the SHADOW faces, the surface still
 *    glows hot pink because the emissive value is added AFTER the
 *    lighting sum and BEFORE the clamp.
 *
 * T12 THREE DEBUG OVERLAYS — MAKING THE SLAB METHOD VISIBLE
 * ──────────────────────────────────────────────────────────
 * Cycling `d` switches the active overlay. Each REPLACES the shaded
 * colour with a visualisation of one piece of intermediate state from
 * §5, teaching one specific aspect of the slab method:
 *
 *   OFF        normal shade-mode output (T9).
 *   AXIS       colour the surface by `axis_at_enter`:
 *                red   = X-axis face won the t_enter race
 *                green = Y-axis face won
 *                blue  = Z-axis face won
 *              You SEE the dispatcher's decision boundary — at a
 *              corner-on view, three differently-coloured triangles
 *              meet at the central vertex.
 *   INTERVAL   brightness from (t_exit − t_enter) — "thickness of
 *              the overlap interval" = "how much of the ray sits
 *              inside the box". Bright at face centres (long dive
 *              through the box), dim at silhouette edges (grazing
 *              rays barely scratch the corners).
 *   FACE-UV    colour by the hit point's two in-face coordinates,
 *              normalised to [-1,+1] and remapped to RGB. Reveals
 *              the local 2D parameterisation each face has — the
 *              same parameterisation used by wireframe to detect
 *              edge-cells.
 *
 * Each overlay reads BoxHit fields populated by §5; toggling them
 * costs only a final tint.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * ════════════════════════════════════════════════════════════════════
 *  ARCHITECTURE — concern layers + state model (steps 4-5)
 * ════════════════════════════════════════════════════════════════════
 *
 *  Separated, labelled layers. Each function lives in exactly one layer.
 *  LOGIC is pure (no mutation, no I/O), so reordering or deleting
 *  RENDER/EFFECTS can never change a LOGIC result.
 *
 *  All runtime state lives on ONE aggregate, Scene (§9.5), so the app
 *  loop reads as a table of contents. Functions take the NARROWEST type
 *  they need — const Box* / const Theme* / scalars for reads — and ONLY
 *  the scene_init / scene_reset / scene_advance lifecycle takes Scene*,
 *  so "everything hangs off Scene" never re-couples the layers.
 *
 *  LAYER        SECTION(S)                 MUTATES
 *  ----------   ------------------------   ---------------------------------
 *  CONFIG       §1, §4                     nothing — constants, enums and
 *                                          material tables (compile-time data)
 *  PERFORMANCE  §2; §10.2/.6; scene_advance  OS sleep, dt cap + rolling fps;
 *                                          no scene-geometry state
 *  LOGIC        §3, §5, §6, §7             nothing — pure functions returning
 *                                          values / out-params (math, ray-AABB
 *                                          slab intersection, shading,
 *                                          debug-overlay decisions)
 *  RENDER       §7.5, §8, §9               the SCREEN (ncurses cells) + the
 *                                          g_have_256 capability flag; reads
 *                                          Scene, never writes it
 *  SIMULATION   scene_advance() (§9.5)     Scene.box.spin_x/_y — the rotation,
 *                                          the ONLY evolving scene state;
 *                                          also zeroed by scene_reset (the `r`
 *                                          key, a user event)
 *  DELAYS       Scene.paused gate (§9.5)   nothing — pause merely skips the
 *                                          SIMULATION step; no holds/timers
 *  EFFECTS      — none —                   no stored cosmetic state: wireframe
 *                                          edges, depth fade and debug tints
 * are all derived at render time from the per-pixel hit, never stored
 *
 *  DOMAIN TYPES (nouns a ray-tracing text names): Box (AABB half-extent +
 *  live spin, §5), Theme (PBR-style material, §4), the hit record BoxHit
 *  (§5), V3 / Mat3 (§3). Scene (§9.5) is the only aggregate; there is no
 *  role-named State / Config / Context struct.
 *
 *  Timestep note: PERFORMANCE uses a VARIABLE wall-clock dt with a spiral-
 *  of-death cap (§10.2) plus a frame cap (§10.6) — NOT a fixed-timestep
 *  accumulator, because the scene is a pure function of elapsed rotation
 *  and needs no sub-stepping.
 *
 *  PER-TICK COMBINE ORDER (main loop — the one place the layers meet):
 *    §10.1 resize   USER EVENT  apply pending SIGWINCH (control state)
 *    §10.2 dt       PERFORMANCE wall-clock dt, capped
 *    §10.3 advance  scene_advance(&scene, dt): SIMULATION spin (if !paused)
 *                                              + PERFORMANCE rolling fps
 *    §10.4 paint    RENDER      erase -> render -> hud_draw -> doupdate
 *    §10.5 input    USER EVENTS mutate Scene view/control knobs (NOT the tick)
 *    §10.6 sleep    PERFORMANCE frame cap
 *
 *  User events (§10.1 resize, §10.5 keys incl. the `r` reset via
 *  scene_reset, the signal handlers) mutate control/view state and
 *  g_run/g_resize. They are NOT the tick; the only simulation write they
 *  make is scene_reset.
 * ════════════════════════════════════════════════════════════════════
 */
