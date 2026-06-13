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

/* §1.3 capsule (object space, axis along Y, centred on origin). */
#define CAP_HALF_H 0.65f /* segment endpoints at ±this    */
#define CAP_R 0.35f      /* tube and cap radius           */

/* §1.4 rotation rates (rad/sec). */
#define ROT_Y 0.45f /* primary spin around Y         */
#define ROT_X 0.22f /* slow tilt around X            */

/* §1.5 camera distance (orbits along −Z; bigger = capsule looks smaller). */
#define CAM_DIST_DEF 3.4f
#define CAM_DIST_MIN 1.8f
#define CAM_DIST_MAX 7.0f
#define CAM_DIST_STEP 0.25f

/* §1.6 shading constants. */
#define AMBIENT 0.20f
#define SHININESS 75.0f      /* phong exponent — high = metal */
#define DEPTH_FAR_SCALE 2.2f /* MODE_DEPTH far plane = cam_dist × this */

/* §1.7 character ramp — Paul Bourke 92-char density ladder.
 * Index 0 (space) is invisible; index N-1 ('@') is densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.8 ncurses pair IDs (216 cube + reserved HUD/HINT). */
#define PAIR_CUBE_BASE 1 /* + 0..215 = 6×6×6 cube         */
#define PAIR_HUD 217
#define PAIR_HINT 218

/* §1.9 numerical epsilon for ray distances. */
#define T_EPS 1e-4f /* reject t < this (self-hit)    */

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
 * Used by the main loop to cap the frame rate without burning the CPU
 * at 100%; we subtract (clock_ns() - frame_start) from the target frame
 * time and sleep the remainder.
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
 * call overhead in the per-pixel loop. The ray-tracer operates entirely
 * on V3s; no V4/homogeneous coords appear because we have no
 * perspective-correct interpolation step (no triangles).
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
 * The reflection identity for an outgoing vector v and a unit normal n:
 *     v_reflected = v − 2·(v·n)·n
 *
 * Geometric reading: subtract twice the COMPONENT of v along n. This
 * is symmetric — incoming = outgoing of the same operator.
 *
 * In §6 phong shading we feed v3reflect the NEGATED light vector (−L)
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
 * mat3_rotation — build R = Rx(angle_x) · Ry(angle_y).
 *
 * Purpose: build the OBJECT→WORLD rotation matrix.
 * Inputs : angle_x, angle_y in radians.
 * Output : Mat3 with three V3 rows.
 *
 * Composition order: Y first, then X. Rotating Y first (the capsule's
 * own axis of rotational symmetry) doesn't change the silhouette but
 * does change WHICH part of the surface faces the lights — so it
 * still matters. The X tilt then changes the silhouette over time.
 *
 * Why this matrix flavour: T7 explains we rotate the RAY by the
 * INVERSE of this matrix. Since rotations are orthogonal, the inverse
 * is the transpose — see mat3_mulT below. We never compute the
 * inverse explicitly.
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
 * Stored row-by-row, so this is three v3dot() calls — efficient and
 * trivial to read.
 *
 * Used in §8 to bring the OBJECT-space surface normal back to WORLD
 * space for shading.
 */
static V3 mat3_mul(Mat3 m, V3 v) {
  return (V3){v3dot(m.row[0], v), v3dot(m.row[1], v), v3dot(m.row[2], v)};
}

/*
 * mat3_mulT — inverse transform v_obj = Mᵀ · v_world.
 *
 * For an ORTHOGONAL matrix (any pure rotation) Mᵀ = M⁻¹. We get the
 * inverse for free by reading the matrix in the transposed access
 * pattern below — no determinant, no division. Same operation count
 * as mat3_mul; just a different memory pattern.
 *
 * Used in §8 to push the WORLD-space ray into OBJECT space where the
 * capsule's axis is fixed at Y.
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
 *   COLOUR    256-colour 6×6×6 cube → "what" the surface is (gold rim
 *             vs cool fill).
 *   DENSITY   ASCII-ramp character → "how bright" the surface is (dim
 *             '.' vs solid '@').
 *
 * Together they recover ~9 bits of per-cell information from a 1-byte
 * glyph plus a colour pair. Themes vary the obj/spec/light tints; the
 * geometry pipeline doesn't care which theme is active.
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

/* ── §5 LOGIC: ray-capsule intersection (pure) — THE CORE
 * ─────────────────────────── *
 *
 * The most important section in this file. Three short functions
 * implementing the recipe from MENTAL MODEL → ALGORITHM IN STEPS:
 *
 *   §5.1 cylinder_test  — ray vs INFINITE cylinder + axial bound check
 *   §5.2 cap_test       — ray vs SPHERE at one endpoint
 *   §5.3 ray_capsule    — dispatcher (cylinder first, then nearer cap)
 *
 * All three operate in OBJECT space. The capsule's axis is along Y; A
 * is the lower endpoint, B is the upper.
 *
 * On hit, every test populates a CapsuleHit struct so the renderer can:
 *   - shade the hit (needs `hit_distance` and `normal_obj`),
 *   - run debug overlays (needs `axial_norm`, `discrim`, `part`).
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * HitPart — which primitive of the capsule a ray struck. A capsule decomposes
 * into one cylinder body plus two end-cap spheres, so a hit is classified by
 * WHICH sub-test won (ray_capsule keeps the nearest). Shading ignores this,
 * but the DEBUG_HIT_TYPE overlay (§7) colours by it to make the three-part
 * decomposition visible, and HIT_NONE is ray_capsule's miss sentinel. Values
 * are 0..3 so the tag doubles as a small index.
 * Ref: Quilez, capsule intersector; Ericson, "Real-Time Collision Detection".
 */
typedef enum {
  HIT_NONE = 0,  /* miss — ray cleared the whole capsule (sentinel)  */
  HIT_BODY = 1,  /* cylinder body, between the two endpoints         */
  HIT_CAP_A = 2, /* sphere cap at endpoint A (lower, -Y)             */
  HIT_CAP_B = 3  /* sphere cap at endpoint B (upper, +Y)             */
} HitPart;

/*
 * CapsuleHit — the "hit record": everything §5's intersection learns about one
 * ray, packed so the renderer (§8) and overlays (§7) need no recomputation.
 * Filled in OBJECT space (capsule's local frame, axis along Y); §8 rotates the
 * normal back to world space for lighting. The split between shading fields and
 * diagnostic fields is deliberate — see each member.
 *   part         which primitive won; HIT_NONE means miss (test this first).
 *   hit_distance ray parameter t (distance along the unit ray). The smallest
 *                positive t across the sub-tests is the nearest visible
 * surface. normal_obj   outward UNIT surface normal at the hit, object space;
 * the input to every diffuse/specular/fresnel term in §6. axial_norm   y/|AB|
 * in [0,1]: how far up the segment a BODY hit landed (0 = A end, 1 = B end).
 * Diagnostic only — feeds DEBUG_AXIAL. discrim      sqrt(max(h,0)) of the
 * cylinder quadratic = how deep inside the tube the ray passed (0 at the
 * silhouette, large through the centre). Written even on a miss so
 * DEBUG_DISCRIM can show near misses. Diagnostic only. Ref: ray-sphere /
 * ray-cylinder quadratics — Shirley, RTiOW; Quilez intersectors.
 */
typedef struct {
  HitPart part;       /* which sub-primitive won (HIT_NONE = miss)            */
  float hit_distance; /* ray parameter t; nearest positive root = surface     */
  V3 normal_obj;    /* outward unit normal, OBJECT space (-> world in §8)    */
  float axial_norm; /* y/|AB| in [0,1] along segment — DEBUG_AXIAL only      */
  float discrim; /* sqrt(max(h,0)) of cyl. quadratic — DEBUG_DISCRIM only */
} CapsuleHit;

/*
 * Capsule — THE domain object this file renders: a line segment (endpoints
 * A..B) thickened by a radius, i.e. the Minkowski sum of a segment and a
 * sphere (the canonical "capsule" / swept-sphere primitive). The segment is
 * the locus of sphere CENTRES, so the same radius rounds the body and both
 * caps — that single shared r is what makes the surface C1-smooth at the
 * body->cap seam (no crease).
 * WHY geometry + orientation together: the angles are not loose scene state —
 * they describe THIS capsule's pose, so they belong with it (step-5 cohesion).
 * Geometry is fixed at scene_init; only spin_* evolve, advanced each tick by
 * scene_advance(). The object stays axis-aligned (A=(0,-hh,0), B=(0,+hh,0));
 * §8 applies the rotation to the RAY instead, keeping §5's math in a fixed,
 * simple frame.
 *   radius       tube + cap radius, object units (shared by body and caps).
 *   half_height  half the A..B segment of sphere-centres along +Y; total
 *                visual height is about 2*(half_height + radius).
 *   spin_x       live pitch about X (rad); += ROT_X * dt each unpaused tick.
 *   spin_y       live yaw   about Y (rad); += ROT_Y * dt each unpaused tick.
 * Ref: Quilez, capsule SDF/intersector; Ericson, "Real-Time Collision
 *      Detection" (capsule = swept sphere).
 */
typedef struct {
  float radius;      /* tube + cap radius (object units); shared body & caps */
  float half_height; /* half the A..B segment of sphere-centres, along +Y    */
  float spin_x; /* live pitch (rad) — += ROT_X*dt each unpaused tick    */
  float spin_y; /* live yaw   (rad) — += ROT_Y*dt each unpaused tick    */
} Capsule;

/*
 * §5.1 cylinder_test — ray vs the INFINITE cylinder along AB,
 *                       with axial bound check inside (0, baba).
 *
 * Purpose:
 *   1. Solve the ray-vs-circle quadratic projected onto the perp-plane
 *      (T3, T4).
 *   2. If a real hit exists, recover its axial coordinate y (T5).
 *   3. If 0 < y < baba, accept it as a BODY hit and compute the radial
 *      outward normal.
 *
 * Inputs:
 *   ray_dir         OBJECT-space ray direction (unit)
 *   axis_seg        ba = B − A
 *   origin_to_A     oa = ray_origin − A
 *   radius          r
 *   axis_len_sq     baba = |ba|² (passed in to skip recomputation)
 *
 * Outputs (only valid on body hit):
 *   *out_distance   ray parameter t > 0
 *   *out_normal_obj radial outward normal in OBJECT space
 *   *out_axial_norm y/baba ∈ (0,1)
 *   *out_discrim    sqrt(max(h, 0)) — set even on miss for the debug
 *                   overlay (lets us SEE where the cylinder is "almost"
 *                   hit even when no body hit was registered).
 *
 * Returns: HIT_BODY on success, HIT_NONE on either:
 *   - cylinder miss (h < 0), or
 *   - cylinder hit outside (0, baba) (caller must try a cap).
 *
 * Pseudocode:
 *   axis_dot_dir       = axis_seg · ray_dir
 *   axis_dot_origin    = axis_seg · origin_to_A
 *   dir_dot_origin     = ray_dir  · origin_to_A
 *   origin_dot_origin  = origin_to_A · origin_to_A
 *
 *   quad_a = axis_len_sq − axis_dot_dir²
 *   quad_b = axis_len_sq · dir_dot_origin
 *          − axis_dot_origin · axis_dot_dir
 *   quad_c = axis_len_sq · (origin_dot_origin − r²)
 *          − axis_dot_origin²
 *   discriminant = quad_b² − quad_a · quad_c
 *
 *   if discriminant < 0:           return MISS  (sets *out_discrim = 0)
 *
 *   t            = (−quad_b − √discriminant) / quad_a
 *   axial_coord  = axis_dot_origin + t · axis_dot_dir
 *
 *   if NOT (T_EPS < t  AND  0 < axial_coord < axis_len_sq):
 *     return MISS but still output axial_coord for the dispatcher
 *
 *   point_minus_A = origin_to_A + t · ray_dir
 *   axial_part    = (axial_coord / axis_len_sq) · axis_seg
 *   normal_obj    = normalize( point_minus_A − axial_part )
 *
 *   return HIT_BODY
 */
static HitPart cylinder_test(V3 ray_dir, V3 axis_seg, V3 origin_to_A,
                             float radius, float axis_len_sq,
                             float *out_distance, V3 *out_normal_obj,
                             float *out_axial_norm, float *out_discrim) {
  float axis_dot_dir = v3dot(axis_seg, ray_dir);
  float axis_dot_origin = v3dot(axis_seg, origin_to_A);
  float dir_dot_origin = v3dot(ray_dir, origin_to_A);
  float origin_dot_origin = v3dot(origin_to_A, origin_to_A);

  /* Quílez form (see T4): scaled by axis_len_sq so divisions cancel. */
  float quad_a = axis_len_sq - axis_dot_dir * axis_dot_dir;
  float quad_b = axis_len_sq * dir_dot_origin - axis_dot_origin * axis_dot_dir;
  float quad_c = axis_len_sq * (origin_dot_origin - radius * radius) -
                 axis_dot_origin * axis_dot_origin;

  float discriminant = quad_b * quad_b - quad_a * quad_c;

  if (discriminant < 0.f) {
    *out_discrim = 0.f;
    return HIT_NONE;
  }
  *out_discrim = sqrtf(discriminant);

  float hit_distance = (-quad_b - *out_discrim) / quad_a;
  float axial_coord = axis_dot_origin + hit_distance * axis_dot_dir;

  /* Tell the dispatcher where on the axis we hit even when the body
   * test fails — it uses this to pick which cap to try next (T6). */
  *out_axial_norm = axial_coord / axis_len_sq;

  if (hit_distance > T_EPS && axial_coord > 0.f && axial_coord < axis_len_sq) {
    /* Radial outward normal: take (P − A) and SUBTRACT its axial
     * component. (axial_coord / axis_len_sq) · axis_seg is the
     * axial component, and the remainder is perpendicular to the
     * axis — that's the radial direction we want. */
    V3 point_minus_A = v3add(origin_to_A, v3scale(hit_distance, ray_dir));
    V3 axial_part = v3scale(axial_coord / axis_len_sq, axis_seg);
    V3 radial = v3sub(point_minus_A, axial_part);

    *out_distance = hit_distance;
    *out_normal_obj = v3norm(radial);
    return HIT_BODY;
  }
  return HIT_NONE;
}

/*
 * §5.2 cap_test — ray vs a SPHERE centred at the supplied cap centre.
 *
 * Purpose: handle the rounded-end case after cylinder_test fell
 *          through (T2). The dispatcher passes us the offset vector
 *          origin_to_cap = ray_origin − cap_centre, so we don't need
 *          to know which cap (A or B) it is.
 *
 * Pseudocode:
 *   quad_b = ray_dir · origin_to_cap
 *   quad_c = origin_to_cap · origin_to_cap − r²
 *   discriminant = quad_b² − quad_c
 *   if discriminant < 0: return MISS
 *
 *   root = sqrt(discriminant)
 *   t = −quad_b − root            (front face)
 *   if t < T_EPS:                 (camera inside? try back face)
 *     t = −quad_b + root
 *     if t < T_EPS: return MISS
 *
 *   normal_obj = normalize( origin_to_cap + t · ray_dir )
 *   return hit
 *
 * We don't enforce a hemisphere bound (e.g. y ≥ 0 for cap A) — if the
 * dispatcher routed us to cap A it's because the cylinder hit was on
 * cap A's side of the segment, so the relevant half of the sphere is
 * automatically the one this ray touches.
 *
 * Returns the HitPart label the caller wants stamped on the result —
 * cap_test is shape-agnostic, the dispatcher tells us what to label.
 */
static HitPart cap_test(V3 ray_dir, V3 origin_to_cap, float radius,
                        HitPart label_on_hit, float *out_distance,
                        V3 *out_normal_obj) {
  float quad_b = v3dot(ray_dir, origin_to_cap);
  float quad_c = v3dot(origin_to_cap, origin_to_cap) - radius * radius;
  float discriminant = quad_b * quad_b - quad_c;

  if (discriminant < 0.f)
    return HIT_NONE;

  float root = sqrtf(discriminant);
  float hit_distance = -quad_b - root;
  if (hit_distance < T_EPS) {
    hit_distance = -quad_b + root;
    if (hit_distance < T_EPS)
      return HIT_NONE;
  }
  V3 point_from_centre = v3add(origin_to_cap, v3scale(hit_distance, ray_dir));
  *out_distance = hit_distance;
  *out_normal_obj = v3norm(point_from_centre);
  return label_on_hit;
}

/*
 * §5.3 ray_capsule — top-level dispatcher.
 *
 * Purpose: try the cylinder first (cheap, covers most pixels of a
 *          side-on capsule). If the cylinder produced no in-bounds
 *          hit, pick the nearer cap by the sign of the axial coord
 *          (T6) and test it as a sphere.
 *
 * Inputs:  ray_origin, ray_dir   OBJECT-space ray
 *          A, B                  capsule endpoints (object space)
 *          radius                tube/cap radius
 *
 * Output:  CapsuleHit struct (defined at the top of §5).
 *          On miss, .part = HIT_NONE; other fields meaningless.
 *
 * Pseudocode:
 *   axis_seg     = B − A
 *   origin_to_A  = ray_origin − A
 *   axis_len_sq  = axis_seg · axis_seg
 *
 *   try cylinder_test:
 *     if HIT_BODY:        return body hit
 *     else:               axial coord written as side-effect (used below)
 *
 *   if axial_norm >= 1:   test sphere at B  (origin_to_cap = oa − ba)
 *                         label = HIT_CAP_B
 *   else:                 test sphere at A  (origin_to_cap = oa)
 *                         label = HIT_CAP_A
 *
 *   on cap hit, set axial_norm to 0 (cap A) or 1 (cap B).
 */
static CapsuleHit ray_capsule(V3 ray_origin, V3 ray_dir, V3 A, V3 B,
                              float radius) {
  V3 axis_seg = v3sub(B, A);
  V3 origin_to_A = v3sub(ray_origin, A);
  float axis_len_sq = v3dot(axis_seg, axis_seg);

  CapsuleHit hit = {.part = HIT_NONE};
  /* Default 0 routes a cylinder MISS to cap A — cap_test will return
   * HIT_NONE anyway in that case (see EDGE CASES in MENTAL MODEL),
   * so the algorithm stays correct. */
  hit.axial_norm = 0.f;
  hit.discrim = 0.f;

  HitPart body_result = cylinder_test(
      ray_dir, axis_seg, origin_to_A, radius, axis_len_sq, &hit.hit_distance,
      &hit.normal_obj, &hit.axial_norm, &hit.discrim);
  if (body_result == HIT_BODY) {
    hit.part = HIT_BODY;
    return hit;
  }

  /* Cylinder either missed entirely (h < 0) or its hit was axially
   * out of bounds. Pick the nearer cap by the sign of axial_norm. */
  int pick_B = (hit.axial_norm >= 1.f);
  V3 cap_centre = pick_B ? B : A;
  HitPart cap_label = pick_B ? HIT_CAP_B : HIT_CAP_A;
  V3 origin_to_cap = v3sub(ray_origin, cap_centre);

  HitPart cap_result = cap_test(ray_dir, origin_to_cap, radius, cap_label,
                                &hit.hit_distance, &hit.normal_obj);
  if (cap_result != HIT_NONE) {
    hit.part = cap_result;
    hit.axial_norm = pick_B ? 1.f : 0.f;
  }
  return hit;
}

/* ── §6 LOGIC: shading (pure) — surface → colour
 * ─────────────────────────────────────────────────────── *
 *
 * Once §5 has identified a hit point and its surface normal, §6 turns
 * those into an RGB colour. Four orthogonal modes (T9) are provided.
 * The renderer (§8) feeds whichever the user has selected with `s`.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * ShadeMode — which surface->colour function §6 runs for a hit: four ORTHOGONAL
 * views of the same geometry (one lit look + three diagnostics) so the user can
 * SEE what the renderer computes. Cycled with `s`. The trailing MODE_N is the
 * member count, which makes `(mode + 1) % MODE_N` wrap cleanly; the order here
 * must match k_mode_names[] and the switch in render() (§8).
 *   MODE_PHONG   Blinn-Phong lit material — the physically-motivated look.
 *   MODE_NORMAL  normal vector mapped xyz->rgb — inspect the surface normals.
 *   MODE_FRESNEL grazing/Fresnel term only — inspect rim/edge falloff.
 *   MODE_DEPTH   distance-to-camera ramp — inspect depth ordering.
 * Ref: Phong (1975); Blinn (1977); Schlick (1994) Fresnel approximation.
 */
typedef enum {
  MODE_PHONG = 0, /* Blinn-Phong lit material (the "real" look)     */
  MODE_NORMAL,    /* normal xyz -> rgb (debug normals)              */
  MODE_FRESNEL,   /* Fresnel/grazing term only (debug rim)          */
  MODE_DEPTH,     /* distance ramp (debug depth)                    */
  MODE_N          /* count — enables % MODE_N wraparound            */
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "fresnel",
                                           "depth"};

/* Three fixed world-space lights — POSITIONS, not directions.
 * Shading computes (light_position − hit_point), normalises, and uses
 * that as the per-pixel light direction (so closer surfaces get a
 * slight wraparound effect). Numbers chosen for visual contrast — a
 * cinematic three-point setup, not physical accuracy:
 *   KEY  = warm above-front-right     (the bright "sun")
 *   FILL = cool low-front-left        (lifts the shadow side)
 *   RIM  = bright back-low            (separates silhouette from bg)
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
 * Orchestrates ambient + three named lights (each via add_phong_light) +
 * emissive, then clamps. The per-light diffuse/specular formula lives in
 * add_phong_light; shade_phong only picks the rig and the gains.
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
 * Diagnostic mode. A correct intersection should show:
 *   - body: smooth radial gradient as N rotates around the axis;
 *   - caps: hemispherical gradient (red dominates near +X, etc).
 * Aliasing or wrong normals show up as harsh colour discontinuities.
 */
static V3 shade_normal(V3 normal_world) {
  return (V3){normal_world.x * 0.5f + 0.5f, normal_world.y * 0.5f + 0.5f,
              normal_world.z * 0.5f + 0.5f};
}

/*
 * shade_fresnel — Schlick approximation, F₀ = 0 (clear dielectric).
 *
 *   cos_angle      = |normal · view_dir|
 *   fresnel_factor = (1 − cos_angle)⁵
 *
 * F₀ = 0 simplifies the full Schlick expression to just (1−cosθ)⁵, so
 * the surface appears DARK facing the camera (cos_angle ≈ 1 → factor ≈
 * 0) and bright at the silhouette (cos_angle ≈ 0 → factor ≈ 1). The
 * classic "glass pill" look: dark middle, glowing rim.
 *
 * Useful for inspecting the silhouette transition between body and
 * caps — they SHOULD merge smoothly without a visible crease.
 */
static V3 shade_fresnel(V3 normal_world, V3 view_dir, const Theme *th) {
  float cos_angle = fabsf(v3dot(normal_world, view_dir));
  float one_minus_cos = 1.f - cos_angle;
  float fresnel_factor = one_minus_cos * one_minus_cos * one_minus_cos *
                         one_minus_cos * one_minus_cos;
  V3 core = v3scale(0.06f, th->albedo);
  V3 edge = v3clamp01(v3scale(1.20f, th->specular));
  return v3clamp01(v3add(v3scale(1.f - fresnel_factor, core),
                         v3scale(fresnel_factor, edge)));
}

/*
 * shade_depth — encode hit distance as brightness (closer = brighter).
 *
 *   depth_norm = 1 − min(t / t_max, 1)         linear depth in [0,1]
 *   bright     = depth_norm²                   steeper falloff
 *
 * Sanity check: in DEPTH mode, the brightest pixel should sit at the
 * nearest point on the silhouette (centre of body for a side-on view).
 */
static V3 shade_depth(float hit_distance, float distance_max, const Theme *th) {
  float depth_norm = 1.f - fminf(hit_distance / distance_max, 1.f);
  depth_norm = depth_norm * depth_norm;
  return v3clamp01(v3scale(depth_norm, th->albedo));
}

/*
 * rec601_luma — perceptual brightness from RGB (Rec. 601 weighting).
 *
 *   Y = 0.299·R + 0.587·G + 0.114·B
 *
 * Used to choose the ASCII ramp character for a coloured pixel. Green
 * dominates because the human eye is most sensitive to green.
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
 * shade_surface — run the shading function the user selected (`s` cycles
 * ShadeMode); the single place a hit becomes colour, so render()'s loop reads
 * as steps. MODE_DEPTH measures against a far plane at
 * cam_dist·DEPTH_FAR_SCALE. Pure: const Theme*, no mutation.
 */
static V3 shade_surface(ShadeMode mode, V3 hit_point_world, V3 normal_world,
                        V3 view_dir, float hit_distance, float cam_dist,
                        const Theme *th) {
  switch (mode) {
  default:
  case MODE_PHONG:
    return shade_phong(hit_point_world, normal_world, view_dir, th);
  case MODE_NORMAL:
    return shade_normal(normal_world);
  case MODE_FRESNEL:
    return shade_fresnel(normal_world, view_dir, th);
  case MODE_DEPTH:
    return shade_depth(hit_distance, cam_dist * DEPTH_FAR_SCALE, th);
  }
}

/* ── §7 LOGIC: debug-overlay decisions (pure)
 * ──────────────────────────────────────────────── *
 *
 * Three overlays that REPLACE the normal shaded colour with a
 * visualisation of the §5 intersection's intermediate state. Cycled
 * with `d`. Each teaches one specific mental model — see T10.
 *
 * The overlays read the CapsuleHit fields populated by §5; no extra
 * computation is needed beyond a final tint.
 *
 * ─────────────────────────────────────────────────────────────────── */

/*
 * DebugMode — an overlay that REPLACES the shaded colour with a picture of
 * §5's internal state, so the three-part capsule math is not a black box. Each
 * overlay only reads CapsuleHit fields already computed (no extra work). Cycled
 * with `d`; trailing DEBUG_N is the count for `% DEBUG_N` wraparound; order
 * matches k_debug_names[] and the switch in apply_debug() (§7).
 *   DEBUG_OFF       no overlay — keep the §6 shaded colour.
 *   DEBUG_HIT_TYPE  tint by HitPart (body vs cap A vs cap B).
 *   DEBUG_AXIAL     tint by axial_norm (position along the segment).
 *   DEBUG_DISCRIM   tint by the cylinder discriminant (depth into the tube).
 */
typedef enum {
  DEBUG_OFF = 0,  /* no overlay — keep the shaded colour            */
  DEBUG_HIT_TYPE, /* tint by HitPart (which sub-test fired)         */
  DEBUG_AXIAL,    /* tint by axial_norm y/|AB| ∈ [0,1]              */
  DEBUG_DISCRIM,  /* tint by cylinder discriminant (tube depth)     */
  DEBUG_N         /* count — enables % DEBUG_N wraparound           */
} DebugMode;
static const char *const k_debug_names[] = {"off", "hit-type", "axial",
                                            "discrim"};

/*
 * gradient_cold_hot — simple two-stop colour gradient driven by t∈[0,1].
 *   t = 0  → deep blue
 *   t = 1  → bright gold
 * Used by the AXIAL and DISCRIM overlays to map a scalar to a colour.
 */
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
 *               colour and luminance. If mode == DEBUG_OFF the caller
 *               keeps its original shading.
 *
 * HIT_TYPE      cyan = body, magenta = cap A, yellow = cap B.
 *               Teaches the dispatcher in §5.3.
 * AXIAL         gradient by axial_norm. Body shows a smooth cold→hot
 *               sweep; caps render flat at their endpoint. Teaches
 *               the bound-check logic in §5.1.
 * DISCRIM       brightness from sqrt(h) of the cylinder quadratic.
 *               Teaches what a "small h" geometrically means
 *               (grazing rays at the silhouette).
 *
 * Outputs are written to *out_colour and *out_luminance. The caller
 * passes them to draw_color() unchanged.
 */
static void apply_debug(DebugMode mode, const CapsuleHit *hit, V3 *out_colour,
                        float *out_luminance) {
  switch (mode) {
  case DEBUG_HIT_TYPE: {
    V3 c;
    switch (hit->part) {
    case HIT_BODY:
      c = (V3){0.20f, 0.95f, 0.95f};
      break; /* cyan    */
    case HIT_CAP_A:
      c = (V3){0.90f, 0.20f, 0.95f};
      break; /* magenta */
    case HIT_CAP_B:
      c = (V3){1.00f, 0.95f, 0.20f};
      break; /* yellow  */
    default:
      c = (V3){0.f, 0.f, 0.f};
      break;
    }
    *out_colour = c;
    *out_luminance = 0.85f;
    break;
  }
  case DEBUG_AXIAL: {
    float t_norm = hit->axial_norm;
    if (t_norm < 0.f)
      t_norm = 0.f;
    if (t_norm > 1.f)
      t_norm = 1.f;
    *out_colour = gradient_cold_hot(t_norm);
    *out_luminance = 0.4f + 0.55f * t_norm;
    break;
  }
  case DEBUG_DISCRIM: {
    /* The discriminant grows roughly linearly with how far INSIDE
     * the cylinder body the ray plunges (zero at silhouette, big
     * in middle). Normalise by a typical max so we get [0,1]. */
    float norm = hit->discrim / (CAP_R * 1.4f);
    if (norm < 0.f)
      norm = 0.f;
    if (norm > 1.f)
      norm = 1.f;
    *out_colour = gradient_cold_hot(norm);
    *out_luminance = norm;
    break;
  }
  default:
    /* DEBUG_OFF — caller keeps its own colour/luminance. */
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
 *             Index = red·36 + green·6 + blue + 1, all in {0..5}.
 *   217       PAIR_HUD  — bright yellow on default bg (HUD spec).
 *   218       PAIR_HINT — bright cyan on default bg (HUD spec).
 *
 * All-or-nothing: if the terminal lacks 256 colours we fall back to
 * monochrome (just the density ramp). Themes still cycle for display
 * but produce identical grayscale output.
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
 *
 * Pseudocode:
 *   ch       = ramp[ luminance · (RAMP_LEN−1) ]
 *   pair_idx = index_into_6x6x6_cube( colour )
 *   mvaddch(row, col, ch | COLOR_PAIR(pair_idx))
 *
 * Why two channels: see §4 header.
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
 *   1. Compute the camera basis (forward / right / up — fixed; the
 *      CAPSULE rotates, not the camera).
 *   2. Pre-build the WORLD↔OBJECT rotation matrix from the current
 *      angles. Used as Mᵀ on the ray (T7) and as M on the normal.
 *   3. For each cell:
 *        a. screen u, v in [-1,1] with FOV expansion + ASPECT correction
 *        b. world ray (origin, direction)
 *        c. object-space ray via mat3_mulT
 *        d. ray_capsule()
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
 * cells don't squash circles into ellipses → combine with the camera basis and
 * normalize. Both axes use the WIDTH centre, so x and y share one pixel scale
 * before the aspect correction. Pure: builds a direction, mutates nothing.
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

static void render(const Capsule *capsule, const Theme *th,
                   ShadeMode shade_mode, DebugMode debug_mode, float cam_dist,
                   int cols, int rows) {
  float fov_half_tan = tanf(FOV_DEG * (float)M_PI / 360.f);

  /* §8.1 — WORLD↔OBJECT rotation. (Mᵀ goes WORLD→OBJECT for the ray;
   *         M goes OBJECT→WORLD for the normal.) */
  Mat3 rotation = mat3_rotation(capsule->spin_x, capsule->spin_y);

  /* §8.2 — camera basis (fixed). */
  V3 camera_origin = {0.f, 0.f, -cam_dist};
  V3 view_forward = {0.f, 0.f, 1.f};
  V3 view_right = {1.f, 0.f, 0.f};
  V3 view_up = {0.f, 1.f, 0.f};

  /* §8.3 — capsule endpoints in OBJECT space (NEVER rotated). */
  V3 endpoint_A = {0.f, -capsule->half_height, 0.f};
  V3 endpoint_B = {0.f, +capsule->half_height, 0.f};

  float screen_centre_x = cols * 0.5f;
  float screen_centre_y = rows * 0.5f;

  /* §8.4 — Mᵀ applied to the camera origin once (it doesn't depend
   *         on the pixel). The view direction varies per pixel and
   *         gets transformed inside the loop. */
  V3 ray_origin_obj = mat3_mulT(rotation, camera_origin);

  /* §8.5 — primary loop: one camera ray per cell, read top-to-bottom. */
  for (int row = 0; row < rows - 1; row++) {
    for (int col = 0; col < cols; col++) {
      V3 ray_dir_world =
          primary_ray_dir(col, row, screen_centre_x, screen_centre_y,
                          fov_half_tan, view_right, view_up, view_forward);
      V3 ray_dir_obj =
          mat3_mulT(rotation, ray_dir_world); /* world → object (T7) */

      CapsuleHit hit = ray_capsule(ray_origin_obj, ray_dir_obj, endpoint_A,
                                   endpoint_B, capsule->radius);
      if (hit.part == HIT_NONE)
        continue;

      /* Lift the hit back into WORLD space (t is identical in both spaces —
       * a pure rotation preserves distance). */
      V3 hit_point_world =
          v3add(camera_origin, v3scale(hit.hit_distance, ray_dir_world));
      V3 normal_world = mat3_mul(rotation, hit.normal_obj);
      V3 view_dir = v3norm(v3sub(camera_origin, hit_point_world));

      V3 colour = shade_surface(shade_mode, hit_point_world, normal_world,
                                view_dir, hit.hit_distance, cam_dist, th);
      float luminance = (shade_mode == MODE_NORMAL)
                            ? normal_vis_luma(normal_world)
                            : rec601_luma(colour);

      if (debug_mode != DEBUG_OFF)
        apply_debug(debug_mode, &hit, &colour, &luminance);

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
  snprintf(status, sizeof status, " %5.1f fps  dist:%.1f  %-8s  %s ",
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
  snprintf(left_label, sizeof left_label, " mode:%-7s  debug:%-8s ",
           k_mode_names[shade_mode], k_debug_names[debug_mode]);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, "%s", left_label);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* §9.3 bottom-left cyan key-hint strip (BOLD, ASCII only). */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc/p:pause  s:mode  d:debug  t:theme  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9.5 SCENE — runtime state aggregate + lifecycle ───────────────── *
 *
 * One struct holding everything that changes at runtime, so the app loop
 * below reads like a table of contents. Functions elsewhere take the
 * NARROWEST type they need (const Capsule*, const Theme*, scalars); only
 * the lifecycle orchestrators here take Scene*, so "everything hangs off
 * Scene" never re-couples the layers.
 *
 * ─────────────────────────────────────────────────────────────────── */

typedef struct {
  /* WHAT is simulated — the one domain object (its own orientation lives
   * inside it, so there are no loose angle fields here). */
  Capsule capsule; /* the spinning capsule (geometry + orientation)  */

  /* HOW the user views it — RENDER/view knobs, grouped by THAT concept (not
   * by "things the keyboard changes"): none of these feed the simulation,
   * they only change the picture. All toggled in §10.5. */
  int theme_idx;        /* index into g_themes[] (read mod THEME_N)       */
  ShadeMode shade_mode; /* which §6 surface->colour function to run       */
  DebugMode debug_mode; /* which §7 overlay (DEBUG_OFF = none)            */
  float cam_dist;       /* camera pull-back along -Z; +/- clamp it to     */
                        /* [CAM_DIST_MIN, CAM_DIST_MAX]                   */
  int paused;           /* DELAYS gate: when set, scene_advance skips spin */

  /* WHERE we draw — terminal size, re-read on SIGWINCH (§10.1) so render
   * and the HUD always match the real window. */
  int cols, rows; /* terminal columns x rows                        */

  /* frame pacing — PERFORMANCE readout only; never affects the image. Once
   * per ~0.5 s window: fps = fps_frames * 1e9 / fps_accum_ns, then both
   * accumulators reset (a rolling average that smooths per-frame jitter). */
  float fps;              /* last computed rolling frames/sec (for the HUD) */
  long long fps_accum_ns; /* nanoseconds summed in the current window       */
  int fps_frames;         /* frames summed in the current window            */
} Scene;

/*
 * scene_init — opening state: capsule geometry + zero spin, default view
 * knobs, current terminal size. Orchestrator: takes the whole Scene.
 */
static void scene_init(Scene *s) {
  s->capsule.radius = CAP_R;
  s->capsule.half_height = CAP_HALF_H;
  s->capsule.spin_x = 0.f;
  s->capsule.spin_y = 0.f;
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
 * scene_advance — one tick of state evolution: spin the capsule (unless
 * paused) and fold dt into the rolling fps. The ONLY writer of simulation
 * state; render/hud never mutate the Scene. Orchestrator: takes Scene*.
 */
static void scene_advance(Scene *s, long long dt_ns) {
  if (!s->paused) {
    s->capsule.spin_y += ROT_Y * (float)dt_ns * 1e-9f;
    s->capsule.spin_x += ROT_X * (float)dt_ns * 1e-9f;
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
  typeahead(-1); /* prevent input tearing of output */
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

    /* §10.3 TICK — advance simulation + fps (the only per-tick Scene
     *         mutation; spin is skipped while paused — the DELAYS layer). */
    scene_advance(&scene, dt);

    /* §10.4 RENDER combine — the ONE place the layers meet, in order:
     *         erase -> render -> hud_draw -> doupdate. Reads state, never
     * writes it. */
    const Theme *th = &g_themes[scene.theme_idx % THEME_N];
    long long frame_start = clock_ns();
    erase();
    render(&scene.capsule, th, scene.shade_mode, scene.debug_mode,
           scene.cam_dist, scene.cols, scene.rows);
    hud_draw(scene.cols, scene.rows, scene.fps, th, scene.shade_mode,
             scene.debug_mode, scene.cam_dist, scene.paused);
    wnoutrefresh(stdscr);
    doupdate();

    /* §10.5 USER EVENTS — mutate view/control state (theme, mode, cam, pause,
     *         quit); NOT part of the tick, never advance simulation. */
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
 * capsule_raytrace.c — analytic ray-trace of a capsule (segment + ball).
 *
 * DEMO: A glossy pill-shaped capsule rotating in space, lit by a
 *       three-point rig of WHITE lights (each material tints itself).
 *       Cycle four shading modes (phong → normals →
 *       fresnel → depth) to see how the same geometry feeds different
 *       visual outputs, and three debug overlays (off → hit-type →
 *       axial → discriminant) to SEE the intersection algorithm work.
 *
 * Study alongside:
 *   raytracing/sphere_raytrace.c   — same skeleton, ONE quadratic
 *   raytracing/torus_raytrace.c    — same skeleton, QUARTIC (much harder)
 *   raytracing/cube_raytrace.c     — same skeleton, slab method
 *
 * Section map (each section is tagged with its concern — CONFIG /
 * PERFORMANCE / LOGIC / RENDER / SCENE / APP; see the ARCHITECTURE block
 * at the bottom of the file for the full layer table):
 *   §1   CONFIG       — frame rate, FOV, capsule geometry, gains, ramp, IDs
 *   §2   PERFORMANCE  — monotonic clock + sleep
 *   §3   LOGIC: math  — V3, Mat3 (object↔world transforms)
 *   §4   CONFIG/DATA  — Theme material table + 256-colour cube
 *   §5   LOGIC: core  — Capsule type + ray-vs-capsule decomposition
 *                       §5.1 cylinder test  §5.2 cap test  §5.3 dispatcher
 *   §6   LOGIC: shade — add_phong_light, phong/normals/fresnel/depth,
 *                       shade_surface dispatcher
 *   §7   LOGIC: debug — overlays that visualise the §5 intermediate state
 *   §7.5 RENDER       — colour output: ncurses pairs + cell painter
 *   §8   RENDER       — per-pixel frame (primary_ray_dir → shade → draw)
 *   §9   RENDER       — ncurses HUD (yellow status row, cyan hint strip)
 *   §9.5 SCENE        — runtime state aggregate + scene_init/scene_advance
 *   §10  APP / TICK   — signals, input, main loop
 *
 * Keys:
 *   s         cycle shade mode (phong → normals → fresnel → depth)
 *   d         cycle debug overlay (off → hit-type → axial → discrim)
 *   t / T     cycle theme — 20 materials in 4 families:
 *               metals (12)  gold, silver, copper, bronze, brass, platinum,
 *                            titanium, iron, steel, chrome, mercury, aluminum
 *               gems    (4)  ruby, emerald, sapphire, amethyst
 *               dielectrics  plastic, glass, ceramic
 *               emissive     neon
 *   p / SPC   pause / resume rotation
 *   + / =     zoom in
 *   -         zoom out
 *   q / ESC   quit
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra raytracing/capsule_raytrace.c \
 *       -o capsule_rt -lncurses -lm
 */

/* ── HOW TO READ THIS FILE ──────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS, MENTAL MODEL, GUIDED TUTORIAL (in this order, as prose).
 *   2. §1 config — the constants you can twiddle.
 *   3. §5 capsule (the core math) — read AFTER tutorials T2-T6.
 *   4. §8 render — see how the math is fed by per-pixel rays.
 *   5. Other sections only if curious.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Long descriptive names everywhere — `axis_dot_dir` not `bard`,
 *   `discriminant` not `h`. Every line becomes a self-explanatory
 *   sentence; no glossary required.
 *
 *   Suffixes name COORDINATE SPACE explicitly:
 *     `_world`  scene-fixed coordinates (camera & lights live here)
 *     `_obj`    capsule-local coordinates (axis is Y)
 *
 *   When a single name lacks a suffix it's space-independent
 *   (e.g. `radius`, `discriminant`).
 *
 * Background you need
 * ───────────────────
 *   - 3-D vector math (dot, scale, normalise).
 *   - The quadratic formula and what its discriminant means
 *     geometrically (positive → two real roots → ray hits the surface).
 *   - That a 3×3 rotation matrix has inverse = transpose.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Any prior raymarching/raytracing knowledge — the file teaches it.
 *   - GPU shading languages — everything is plain CPU-side C.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── CONCEPTS ───────────────────────────────────────────────────────── *
 *
 * Algorithm      : Analytic ray-vs-capsule. A capsule is the Minkowski
 *                  sum of a finite line segment AB and a ball of radius
 *                  r — every point within distance r of the segment.
 *                  The ray test decomposes into:
 *                    (a) ray vs INFINITE CYLINDER along AB,
 *                    (b) ray vs HEMISPHERE CAP at endpoint A or B.
 *                  Whichever yields the smallest valid t is the hit.
 *                  Closed-form throughout — no marching, no mesh.
 *
 * Data-structure : A capsule is just three numbers' worth of state:
 *                    A : V3      first endpoint
 *                    B : V3      second endpoint
 *                    r : float   tube + cap radius
 *                  Object-space convention: A and B sit on the Y axis;
 *                  world-space orientation is recovered by rotating the
 *                  RAY into object space (much cheaper than rotating
 *                  the capsule every frame — see T7).
 *
 * Rendering      : One ray per terminal cell, no anti-aliasing. The
 *                  surface is shaded by one of four modes (phong /
 *                  normal / fresnel / depth) and optionally re-tinted
 *                  by one of three debug overlays. Hue → 256-colour
 *                  6×6×6 cube, brightness → 92-char Bourke ramp.
 *
 * Performance    : Pure analytic — one quadratic per pixel for the
 *                  body test, optionally a second for the nearer cap.
 *                  No iteration, no spatial structures. Heaviest cost
 *                  is the shading (powf for the specular highlight); a
 *                  capsule is one of the cheapest non-trivial
 *                  primitives to ray-trace.
 *
 * References     : Inigo Quílez, "Capsule — intersection",
 *                    https://iquilezles.org/articles/intersectors/
 *                  Real-Time Rendering 4e §22.7 (ray-cylinder, ray-sphere).
 *                  Shirley & Marschner, "Fundamentals of Computer Graphics"
 *                    4e ch. 4 (raytracing primitives).
 *                  Schlick, "An Inexpensive BRDF Model for Physically-based
 *                    Rendering", Comp. Graph. Forum 13(3), 1994 [fresnel].
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ───────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * A capsule is a sphere SMEARED along a line segment. So solving "where
 * does my ray hit the capsule?" splits naturally into "where does my
 * ray hit the cylindrical part?" and "where does it hit the rounded
 * ends?". Each sub-problem reduces to a quadratic in t, and we keep
 * whichever answer is the closest valid hit. Three quadratics, no
 * iteration — done.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a soap bubble whose centre is dragged along a finite line
 * segment from A to B. As the centre moves, the bubble traces out a
 * tube of radius r between A and B and leaves a hemispherical "cap" at
 * each endpoint.
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
 * For ray-tracing, we treat the band as an infinite cylinder and check
 * AFTERWARD whether the hit lies between A and B along the axis. If it
 * does, body hit. If it doesn't, retest the nearer endpoint as a
 * sphere. Total quadratics solved per pixel: at most two.
 *
 * Coordinate-system pipeline (T8 unpacks each step):
 *
 *       SCREEN ─FOV─→ CAMERA ─cam basis─→ WORLD ─Mᵀ─→ OBJECT
 *                                                        ↓
 *                                                   intersection
 *                                                        ↓
 *       OBJECT ────M────→ WORLD ──────→ shading ──→ pixel
 *
 * Everything left of "intersection" is per-frame ceremony; the
 * intersection happens in OBJECT space where the capsule's axis is
 * fixed at Y. Everything right of it is shading.
 *
 * ALGORITHM IN STEPS  (per pixel)
 * ───────────────────────────────
 *  1. Build a primary ray (origin + direction) in WORLD space from the
 *     screen coordinates and the camera's basis.
 *  2. Apply Mᵀ (the rotation matrix's transpose) to the ray, putting
 *     it in OBJECT space where the capsule lives along the Y axis.
 *  3. Solve ray vs INFINITE CYLINDER:
 *        a = |ba|² − (ba·rd)²
 *        b = |ba|²(rd·oa) − (ba·oa)(ba·rd)
 *        c = |ba|²(|oa|² − r²) − (ba·oa)²
 *        h = b² − a·c                          (discriminant)
 *        t = (−b − √h)/a                       (front face)
 *        y = (ba·oa) + t·(ba·rd)               (axial coord of hit)
 *     If 0 < y < |ba|², it's a BODY hit. Compute the radial normal:
 *        N_obj = normalize( (oa + t·rd) − (y/|ba|²)·ba )
 *  4. Otherwise the ray either missed the cylinder or hit it outside
 *     the segment. Pick the cap to test:
 *        y ≤ 0     → cap at A
 *        y ≥ |ba|² → cap at B
 *  5. Ray-vs-sphere quadratic at the chosen cap centre:
 *        oc = (object-space ray origin) − (cap centre)
 *        b' = rd · oc      c' = oc·oc − r²      h' = b'² − c'
 *        t  = −b' − √h'
 *     If h' < 0 the ray missed — return "no hit".
 *     Otherwise N_obj = normalize(oc + t·rd).
 *  6. Bring the normal back to WORLD space: N_world = M · N_obj.
 *  7. Shade in WORLD space (phong / normals / fresnel / depth).
 *  8. Optionally re-tint by the active debug overlay.
 *
 * KEY FORMULAS
 * ────────────
 *   ba   = B − A                    (segment vector)
 *   oa   = ro − A                   (ray origin relative to A)
 *   baba = |ba|²                    (axis length squared, used to skip
 *                                    explicit normalisation of ba)
 *
 *   CYLINDER QUADRATIC (Quílez, division-free until the very end):
 *     a = baba − (ba·rd)²
 *     b = baba·(rd·oa) − (ba·oa)·(ba·rd)
 *     c = baba·(|oa|² − r²) − (ba·oa)²
 *     h = b² − a·c
 *     t = (−b − √h) / a
 *     y = (ba·oa) + t·(ba·rd)
 *
 *   BODY NORMAL  (radial outward, axial component removed):
 *     N_obj = normalize( (oa + t·rd) − (y/baba)·ba )
 *
 *   CAP SPHERE QUADRATIC:
 *     b' = rd · oc      c' = oc·oc − r²      h' = b'² − c'
 *     t  = −b' − √h'
 *
 *   CAP NORMAL  (away from cap centre):
 *     N_obj = normalize( oc + t·rd )
 *
 *   WORLD/OBJECT TRANSFORMS:
 *     ray_obj    = (Mᵀ·cam_world, Mᵀ·dir_world)   (M orthogonal ⇒ M⁻¹=Mᵀ)
 *     N_world    = M · N_obj
 *
 *   SCHLICK FRESNEL (used in shade mode):
 *     F = F₀ + (1 − F₀)·(1 − cosθ)⁵        with cosθ = |N · V_dir|
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • Ray parallel to the axis: (ba·rd)² → baba so a → 0, dividing by
 *    zero. Mathematically the cylinder term degenerates; only the caps
 *    see the ray. In practice axis-parallel rays are zero-measure for
 *    a rotating capsule but a guarded `if (a < 1e-9) skip cylinder`
 *    would harden it.
 *  • t < T_EPS reject. Without this the camera grazing or starting
 *    inside the surface would trigger spurious self-hits at t ≈ 0.
 *  • Body-cap seam (y = 0 or y = baba): the body normal and cap normal
 *    AGREE (both point radially outward from the same circle) so no
 *    crease appears — shading is C¹ across the join.
 *  • Ramp index uses `(int)(luminance * (RAMP_LEN−1))` so luminance=1
 *    maps to the densest character, not to RAMP_LEN (out of bounds).
 *  • The dispatcher reuses `axial_norm` to pick a cap when the
 *    cylinder misses. A pure miss leaves axial_norm uninitialised, but
 *    the same miss implies "perp distance to axis > r", so cap A's
 *    sphere test will return MISS too — the default 0 is safe.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • End-on view (camera looks down the capsule's axis): you should
 *    see a circular DISC, not a pill — only the front cap is visible.
 *    Switch to NORMALS: the disc is overwhelmingly blue (N ≈ −Z).
 *  • Side-on view: classic pill silhouette — central rectangle of
 *    width 2r flanked by two arcs of radius r.
 *  • `d` → HIT-TYPE overlay: at side-on view a cyan band (body) capped
 *    by magenta and yellow blobs (cap A, cap B).
 *  • `d` → AXIAL overlay: at side-on view a smooth bottom-to-top
 *    gradient across the body, constant on each cap.
 *  • `d` → DISCRIM overlay: bright disc, brightest on-axis, fading
 *    to black at the silhouette where the ray grazes (h → 0).
 *  • Worked example (T2 + T3 in tutorials): a head-on ray to a
 *    unit-tall capsule at distance 3.4 should hit at t ≈ 3.05.
 *  • Doubling CAP_R should double the apparent thickness of the body
 *    and leave proportional caps.
 *
 * ─────────────────────────────────────────────────────────────────── */

/* ── GUIDED TUTORIAL ────────────────────────────────────────────────── *
 *
 * Ten short tutorials that build the algorithm from first principles.
 * Read in order; each builds on the previous.
 *
 *   T1  What problem are we solving?
 *   T2  The simplest case: ray vs SPHERE
 *   T3  The next case:    ray vs INFINITE CYLINDER
 *   T4  Why the Quílez form has no division
 *   T5  Bound-checking the cylinder hit (axial coordinate)
 *   T6  Choosing the right cap on a fall-through
 *   T7  The inverse-rotation trick (rotate the RAY)
 *   T8  Coordinate systems at a glance
 *   T9  Four shading modes — same geometry, four pictures
 *   T10 White light is the lab; the material is the specimen
 *   T11 Metals vs dielectrics — F0, Fresnel, and the spec-tint trick
 *   T12 Three debug overlays — making the algorithm visible
 *
 * ─────────────────────────────────────────────────────────────────── *
 *
 * T1  WHAT PROBLEM ARE WE SOLVING?
 * ────────────────────────────────
 * Given:
 *   • a ray  (origin O, unit direction D)
 *   • a capsule  (segment AB, radius r)
 * Find:
 *   • the smallest t > 0 such that O + t·D lies on the capsule's
 *     surface,
 *   • the outward unit normal at that point.
 *
 * Why a capsule and not a more general shape? Because capsules pop up
 * everywhere — limbs of stick figures, wires, swords, animal bones —
 * AND have closed-form intersection AND have rounded ends that hide
 * tessellation artifacts on a low-res terminal.
 *
 * The trick we'll exploit: a capsule is the SET-WISE UNION of a finite
 * cylinder (the "tube body") with two hemispheres (the "caps"), all
 * sharing the same surface. So instead of solving one nasty implicit
 * equation, we solve up to three textbook quadratics and combine.
 *
 * T2  THE SIMPLEST CASE: RAY VS SPHERE
 * ────────────────────────────────────
 * Sphere of centre C, radius r:
 *
 *     |O + t·D − C|² = r²
 *
 * Expand and collect terms in t. With oc = O − C:
 *
 *     (D·D)·t² + 2·(D·oc)·t + (oc·oc − r²) = 0
 *
 * Since D is a unit vector, D·D = 1, so:
 *
 *     t² + 2·b·t + c = 0     where b = D·oc, c = oc·oc − r²
 *     t = −b ± √(b² − c)
 *
 *     b² − c is the DISCRIMINANT. If it's negative, the ray misses.
 *     The smaller root (−b − √…) is the FRONT face hit.
 *
 * Pseudocode:
 *
 *     b = dot(D, O − C)
 *     c = |O − C|² − r²
 *     h = b·b − c
 *     if (h < 0) return MISS
 *     t = −b − sqrt(h)
 *     N = normalize((O + t·D) − C)
 *
 * Read the file's `cap_test` (§5.2) — that IS this exact formula, with
 * `oc` precomputed by the dispatcher.
 *
 * T3  THE NEXT CASE: RAY VS INFINITE CYLINDER
 * ────────────────────────────────────────────
 * A cylinder of axis line (point A, unit direction n̂, radius r) is the
 * locus of points whose perpendicular distance to the axis equals r:
 *
 *     |(P − A) − ((P − A)·n̂)·n̂|  =  r
 *
 * The expression in absolute-value brackets is "(P−A) with its axial
 * component removed". So the equation says: project P onto the plane
 * perpendicular to the axis; that 2-D projection is at distance r from
 * the axis line. In other words, RAY-VS-CYLINDER is morally a 2-D
 * RAY-VS-CIRCLE viewed in the perp-plane.
 *
 * Substitute P = O + t·D and square both sides to drop the absolute
 * value. Define D_perp = D − (D·n̂)·n̂ and oa_perp similarly. The
 * equation collapses to:
 *
 *     |D_perp|² · t²
 *   + 2·(D_perp · oa_perp) · t
 *   + (|oa_perp|² − r²) = 0          where oa = O − A
 *
 * — a textbook 2-D ray-vs-circle quadratic. We've literally reduced
 * the 3-D cylinder problem to a 2-D circle problem in the perp-plane.
 *
 * T4  WHY THE QUÍLEZ FORM HAS NO DIVISION
 * ────────────────────────────────────────
 * Computing |D_perp|² etc. requires us to first NORMALISE n̂, i.e.
 * divide ba by |ba|. Quílez's trick is to rescale the WHOLE quadratic
 * by baba = |ba|² so the divisions cancel:
 *
 *     a = baba − (ba·D)²              (= baba · |D_perp|²)
 *     b = baba·(D·oa) − (ba·oa)·(ba·D) (= baba · (D_perp · oa_perp))
 *     c = baba·(|oa|² − r²) − (ba·oa)² (= baba · (|oa_perp|² − r²))
 *     h = b² − a·c                    (sign matches the original)
 *     t = (−b − √h) / a               (the baba factor cancels)
 *
 * Now there's only ONE division (the final one) and zero divisions
 * involving square roots. Numerically robust and a hair faster than
 * the normalised form. The discriminant h has the right SIGN so the
 * miss/hit decision is unaffected.
 *
 * Read §5.1 `cylinder_test` to see the seven-line implementation.
 *
 * T5  BOUND-CHECKING THE CYLINDER HIT (AXIAL COORDINATE)
 * ───────────────────────────────────────────────────────
 * The cylinder above is INFINITE — the formula doesn't know where A
 * and B are along the axis. So once we have a hit at parameter t we
 * ask: "WHERE along the axis did we hit?"
 *
 *     y = (ba·oa) + t·(ba·D)
 *
 * y is the axial coordinate scaled by baba. So the hit lies between
 * A and B iff:
 *
 *     0 ≤ y ≤ baba           (after scaling, [0, baba] = [A, B])
 *
 * Geometrically:
 *
 *                   y < 0           0 ≤ y ≤ baba       y > baba
 *                      │                  │                │
 *     A ●═══════════════════════ tube body ══════════════════● B
 *     (cap A's domain)        (cylinder body)        (cap B's domain)
 *
 * In-bounds → return body hit. Out-of-bounds → fall through to the
 * cap test (T6). The same y tells us WHICH cap.
 *
 * T6  CHOOSING THE RIGHT CAP ON A FALL-THROUGH
 * ─────────────────────────────────────────────
 * If y ≤ 0 the would-be cylinder hit is on cap A's side of the
 * segment; if y ≥ baba it's on cap B's side. So we test only ONE cap,
 * not both — picking by the sign of y:
 *
 *     y ≤ 0     → test sphere centred at A
 *     y ≥ baba  → test sphere centred at B
 *
 * Why does this work? Because if the ray were going to hit the OTHER
 * cap, the cylinder hit would have been on that cap's side instead.
 * (Geometric reason: a straight ray crosses any half-plane in a
 * single direction.)
 *
 * Edge case: what if the cylinder MISSED entirely (h < 0)? Then we
 * never wrote y. Quirk: a ray that misses the infinite cylinder also
 * cannot hit either cap (its perpendicular distance to the axis is
 * already > r). So testing cap A (or whatever) returns MISS too —
 * the algorithm is still correct, just spends one extra quadratic.
 *
 * T7  THE INVERSE-ROTATION TRICK (ROTATE THE RAY)
 * ────────────────────────────────────────────────
 * Naive way: every frame, rotate the capsule's endpoints A and B by
 * the current orientation matrix M. Cost: 6 floats per frame. Fine
 * for one capsule; generalises poorly to scenes with many primitives.
 *
 * Smarter way: keep A and B FIXED at the canonical Y axis, and rotate
 * the RAY into the capsule's local space:
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
 * because rotation matrices preserve length and orthogonality (no
 * "inverse-transpose for normals" gymnastics needed for non-uniform
 * scaling — there is no scaling here).
 *
 * Cost: ONE matrix-vector multiply per ray instead of N matrix-vector
 * multiplies per frame for N capsules. Wins big as scenes grow.
 *
 * T8  COORDINATE SYSTEMS AT A GLANCE
 * ───────────────────────────────────
 * Four spaces appear in this file. Keeping them straight avoids hours
 * of confusion:
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
 *      WORLD           (x, y, z)          scene-fixed coords; lights live here
 *        │
 *        │  ray_obj.origin = Mᵀ · ray_world.origin
 *        │  ray_obj.dir    = Mᵀ · ray_world.dir
 *        ▼
 *      OBJECT          (x, y, z)          capsule axis fixed at Y
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
 * Note one asymmetry: we transform the RAY into object space but
 * compute the hit point in WORLD space (using the world-space ray +
 * the t we obtained). Either is mathematically correct because t is
 * the same in both spaces (rotation preserves distances). Using
 * world-space hit points lets shading use world-space lights with no
 * extra transforms.
 *
 * T9  FOUR SHADING MODES — SAME GEOMETRY, FOUR PICTURES
 * ──────────────────────────────────────────────────────
 * After we know the hit point and its normal we still have to turn
 * those into a colour. Four orthogonal modes (cycle with `s`):
 *
 *   PHONG    Three pure-WHITE lights (key + fill + rim). Diffuse =
 *            max(N·L, 0); specular = (R·V)^shininess. The material's
 *            distinctive colour comes from its OWN albedo and spec
 *            tint — the lights are white so each material reads as
 *            itself, not as a coloured filter. Diffuse weight per
 *            material: metals ≈ 0.15 (spec-dominated), gems ≈ 0.70,
 *            dielectrics ≈ 0.85, glass ≈ 0.10. Emissive (added after
 *            lighting, before clamp) lets neon glow in shadow.
 *   NORMALS  N → (N+1)/2, channel-by-channel. RGB-encodes the
 *            surface normal as a colour. A correct intersection shows
 *            a smooth radial gradient on the body and hemispherical
 *            colour wheels on the caps. Useful for verifying
 *            intersection correctness.
 *   FRESNEL  Schlick: F = F₀ + (1−F₀)(1−|N·V|)⁵. Looks like glass —
 *            dark facing, glowing edge. Best mode for showing the
 *            silhouette transition between body and caps (which
 *            should be invisible — see EDGE CASES).
 *   DEPTH    Brightness from t: closer = brighter. Useful as a
 *            sanity check (the centre of the silhouette should be the
 *            brightest spot in DEPTH mode — closest to the camera).
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
 * Why it matters: the DIFFERENCE between gold and yellow plastic is
 * not the body colour (both yellow) but the highlight — gold's
 * highlight is YELLOW, plastic's is WHITE. That single tint
 * difference is what makes "metal" look like metal and "plastic"
 * look like plastic.
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
 * Try it:
 *  - Cycle to gold (`t` until name says "gold") — watch the highlight
 *    on the bright side of the capsule. WARM YELLOW.
 *  - Cycle to plastic — same geometry, same lighting; highlight is
 *    pure WHITE. The body is blue but the highlight is white. That's
 *    the dielectric signature.
 *  - Cycle to ruby (red gem dielectric) — body deep red, highlight
 *    near-white. Same dielectric signature.
 *  - Cycle to neon — even on the SHADOW side, the surface still glows
 *    hot pink because the emissive value is added AFTER the lighting
 *    sum and BEFORE the clamp.
 *
 * T12 THREE DEBUG OVERLAYS — MAKING THE ALGORITHM VISIBLE
 * ────────────────────────────────────────────────────────
 * Cycling `d` switches the active overlay. Each REPLACES the shaded
 * colour with a visualisation of one piece of intermediate state from
 * §5, teaching one specific mental model:
 *
 *   OFF        normal shade-mode output (T9).
 *   HIT-TYPE   colour the surface by which sub-test fired:
 *                cyan    = body  (cylinder test, T3)
 *                magenta = cap A (sphere at lower endpoint, T6)
 *                yellow  = cap B (sphere at upper endpoint, T6)
 *              You SEE the boundary between cylinder hits and cap
 *              hits. Teaches the dispatcher in §5.3.
 *   AXIAL      colour by y/baba ∈ [0,1] using a cold→hot gradient.
 *              Body shows a smooth bottom-to-top sweep; caps render
 *              flat at their endpoint (0 for A, 1 for B). Teaches
 *              the bound check in §5.1.
 *   DISCRIM    map sqrt(h) (cylinder discriminant) to brightness.
 *              Bright on-axis where the ray plunges deep; black at
 *              the silhouette where the ray grazes (h → 0). Teaches
 *              what a "small h" geometrically means.
 *
 * Each overlay reuses the per-pixel CapsuleHit fields populated by
 * §5, so toggling them costs only a final tint.
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
 *  they need — const Capsule* / const Theme* / scalars for reads — and
 *  ONLY the scene_init / scene_advance lifecycle takes Scene*, so
 *  "everything hangs off Scene" never re-couples the layers.
 *
 *  LAYER        SECTION(S)                 MUTATES
 *  ----------   ------------------------   ---------------------------------
 *  CONFIG       §1, §4                     nothing — constants, enums and
 *                                          material tables (compile-time data)
 *  PERFORMANCE  §2; §10.2/.6; scene_advance  OS sleep, dt cap + rolling fps;
 *                                          no scene-geometry state
 *  LOGIC        §3, §5, §6, §7             nothing — pure functions returning
 *                                          values / out-params (math,
 *                                          ray-capsule intersection, shading,
 *                                          debug-overlay decisions)
 *  RENDER       §7.5, §8, §9               the SCREEN (ncurses cells) + the
 *                                          g_have_256 capability flag; reads
 *                                          Scene, never writes it
 *  SIMULATION   scene_advance() (§9.5)     Scene.capsule.spin_x/_y — the
 *                                          rotation, the ONLY evolving scene
 *                                          state and its sole writer
 *  DELAYS       Scene.paused gate (§9.5)   nothing — pause merely skips the
 *                                          SIMULATION step; no holds/timers
 *  EFFECTS      — none —                   no stored cosmetic state: fresnel,
 *                                          depth-fade and debug tints are all
 *                                          derived at render time from the
 *                                          per-pixel hit, never stored
 *
 *  DOMAIN TYPES (nouns a ray-tracing text names): Capsule (segment + radius
 *  + live spin, §5), Theme (PBR-style material, §4), CapsuleHit / HitPart
 *  (the hit record, §5), V3 / Mat3 (§3). Scene (§9.5) is the sole aggregate;
 *  there is no role-named State / Config / Context struct.
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
 *  User events (§10.1 resize, §10.5 keys, the signal handlers) mutate
 *  control/view state and g_run/g_resize. They are NOT the tick and never
 *  advance SIMULATION state.
 * ════════════════════════════════════════════════════════════════════
 */
