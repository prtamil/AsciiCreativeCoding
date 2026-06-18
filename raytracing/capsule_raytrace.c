/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * capsule_raytrace.c — a glossy spinning capsule (a tube with rounded ends),
 * ray-traced one ray per terminal cell. 's' cycles the look (lit / normals /
 * fresnel / depth), 'd' reveals how the hit is found, 't' cycles materials.
 *
 * Sister files, same skeleton with different shape maths: sphere_raytrace.c,
 * torus_raytrace.c, cube_raytrace.c (all in raytracing/).
 */

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

/* ── §1 settings — every number you can tweak ── */

/* §1.1 frame rate. */
#define TARGET_FPS 60
#define DT_CAP_NS 100000000LL /* longest step we honour, so a hiccup can't fast-forward the spin */

/* §1.2 the shape of one terminal cell (width ÷ height) and how wide the view is. */
#define ASPECT 0.47f
#define FOV_DEG 55.0f

/* §1.3 the capsule, before it's spun: axis runs up the Y axis, centred on origin. */
#define CAP_HALF_H 0.65f /* half the tube length (the two ends sit at ±this) */
#define CAP_R 0.35f      /* tube and end-cap radius                          */

/* §1.4 how fast it turns (radians per second). */
#define ROT_Y 0.45f /* main spin                     */
#define ROT_X 0.22f /* slow tilt                     */

/* §1.5 how far back the camera sits (bigger = capsule looks smaller). */
#define CAM_DIST_DEF 3.4f
#define CAM_DIST_MIN 1.8f
#define CAM_DIST_MAX 7.0f
#define CAM_DIST_STEP 0.25f

/* §1.6 lighting. */
#define AMBIENT 0.20f        /* dim base light so the shadow side isn't pure black */
#define SHININESS 75.0f      /* highlight tightness — higher = sharper, glossier   */
#define DEPTH_FAR_SCALE 2.2f /* depth view: far edge = this × camera distance      */

/* §1.6b fresnel view — the "glass pill" rim look. */
#define FRESNEL_CORE 0.06f /* how dark the body is where it faces you   */
#define FRESNEL_EDGE 1.20f /* how bright the rim glows at the silhouette */

/* §1.6c debug-overlay brightness tuning (§7). */
#define DBG_HIT_LUMA 0.85f        /* hit-type overlay: flat brightness        */
#define DBG_AXIAL_LUMA_MIN 0.40f  /* axial overlay: dimmest, at the bottom end */
#define DBG_AXIAL_LUMA_SPAN 0.55f /* axial overlay: brightening toward the top */
#define DBG_DISCRIM_SCALE 1.4f    /* discrim overlay: typical max depth = this × radius */

/* §1.7 the characters we draw with, faint to dense (Paul Bourke's ramp).
 * Index 0 is a space (invisible); the last one ('@') is the densest. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* §1.8 colour-slot numbers: a 216-colour cube, plus two reserved for the HUD. */
#define PAIR_CUBE_BASE 1 /* slots 1..216 hold the 6×6×6 colour cube */
#define PAIR_HUD 217
#define PAIR_HINT 218

/* §1.9 ignore hits closer than this — keeps a surface from shadowing itself. */
#define T_EPS 1e-4f

/* ── §2 clock — a steady timer and a sleep, to pace the frames ── */

/* A counter that only ever moves forward — we use the gaps between readings to
 * time the animation. (The plain wall clock can jump around from NTP or DST.) */
static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* Sleep the leftover of a frame so we hold the target rate instead of spinning
 * the CPU at 100%. */
static void clock_sleep_ns(long long nanoseconds) {
  if (nanoseconds <= 0)
    return;
  struct timespec request = {nanoseconds / 1000000000LL,
                             nanoseconds % 1000000000LL};
  nanosleep(&request, NULL);
}

/* ── §3 math — points, directions, and a rotation ── */

/* Three floats: a point or direction in space, or an RGB colour (same algebra
 * either way). Passed by value because it's tiny and rides in CPU registers,
 * which is faster than chasing a pointer in the per-pixel loop. */
typedef struct {
  float x, y, z;
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

/* A rotation, stored as its three rows. Because it's a pure rotation, undoing it
 * is just flipping it across the diagonal (its transpose) — no real matrix
 * inverse needed. That's the whole trick in §8: instead of rotating the capsule,
 * we rotate each ray the opposite way and test against a capsule that never
 * moves. (Only rotation, no move/scale, so 3×3 is enough.) */
typedef struct {
  V3 row[3];
} Mat3;

/* Build the capsule's current orientation: spin around Y, then tilt around X.
 * (Spinning around its own long axis doesn't change the outline, but it does
 * change which part faces the lights, so it still matters.) */
static Mat3 mat3_rotation(float angle_x, float angle_y) {
  float cos_x = cosf(angle_x), sin_x = sinf(angle_x);
  float cos_y = cosf(angle_y), sin_y = sinf(angle_y);
  Mat3 m;
  m.row[0] = (V3){cos_y, 0.f, sin_y};
  m.row[1] = (V3){sin_x * sin_y, cos_x, -sin_x * cos_y};
  m.row[2] = (V3){-cos_x * sin_y, sin_x, cos_x * cos_y};
  return m;
}

/* Apply the rotation. §8 uses it to bring the surface's facing direction from
 * the capsule's frame back into world space for lighting. */
static V3 mat3_mul(Mat3 m, V3 v) {
  return (V3){v3dot(m.row[0], v), v3dot(m.row[1], v), v3dot(m.row[2], v)};
}

/* Apply the rotation backwards (read the matrix flipped across its diagonal).
 * §8 uses this to turn a world-space ray into the capsule's own frame, where the
 * capsule stands still along Y and the maths is simple. */
static V3 mat3_mulT(Mat3 m, V3 v) {
  return (V3){m.row[0].x * v.x + m.row[1].x * v.y + m.row[2].x * v.z,
              m.row[0].y * v.x + m.row[1].y * v.y + m.row[2].y * v.z,
              m.row[0].z * v.x + m.row[1].z * v.y + m.row[2].z * v.z};
}

/* ── §4 materials — the look-up table of surfaces ── */

/* One material the capsule can be made of (the t/T keys cycle through the table
 * below). The lights are plain white; each material's character comes entirely
 * from these numbers, so gold looks like gold and blue plastic like blue plastic
 * without ever tinting the lights. Metal colours come from standard reference
 * tables (Naty Hoffman, "Physics and Math of Shading", SIGGRAPH 2013), nudged
 * for terminal contrast.
 *
 *   albedo         the body colour you'd see under plain white light
 *   specular       the colour of the shiny highlight — metals tint it to match
 *                  their body (gold's glint is yellow); non-metals keep it white
 *   emissive       self-glow added after the lighting, so it shows even in
 *                  shadow; zero for everything except neon
 *   diffuse_weight how much body colour shows vs. how much is pure shine:
 *                  ~0.15 metals (almost all shine), ~0.7 gems, ~0.85 plastic,
 *                  ~0.10 glass (dark body, bright shine fakes see-through) */
typedef struct {
  V3 albedo;            /* body colour                              */
  V3 specular;          /* highlight colour                         */
  V3 emissive;          /* self-glow (added after lighting)         */
  float diffuse_weight; /* body-vs-shine balance, ~0.10..0.90       */
  const char *name;
} Theme;

static const Theme g_themes[] = {
    /* METALS (12) — highlight tinted to match the body, very little flat colour.
     * A metal is almost all shine, and the shine carries its signature hue. */

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

    /* GEMS (4) — deep saturated body colour with a white highlight. */

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

    /* NON-METALS (3) — plastic, glass, ceramic: full body colour, white shine. */

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

    /* GLOWING (1) — neon: a dark "off" body plus a self-glow that shows even in
     * shadow (the glow is added after the lighting). */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f, 0.02f, 0.10f},
     {0.80f, 0.80f, 1.00f},
     {1.00f, 0.20f, 0.85f},
     0.20f,
     "neon"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

/* ── §5 the core — where does a ray hit the capsule? ──
 *
 * A capsule is a tube with a ball on each end, so we test three simple shapes:
 * the endless tube (§5.1), a sphere at each end (§5.2), and a dispatcher that
 * tries the tube first, then the nearer ball (§5.3). All of it works in the
 * capsule's own frame, where its axis stands straight up the Y axis. */

/* Which part of the capsule a ray hit. The dispatcher keeps the nearest. Shading
 * doesn't care which, but the 'd' debug overlay colours by it so you can see the
 * three-piece breakdown; HIT_NONE means the ray missed entirely. */
typedef enum {
  HIT_NONE = 0,  /* missed the whole capsule          */
  HIT_BODY = 1,  /* the tube, between the two ends     */
  HIT_CAP_A = 2, /* the ball at the bottom end (−Y)    */
  HIT_CAP_B = 3  /* the ball at the top end (+Y)       */
} HitPart;

/* Everything we learn about one ray's hit, packed so the renderer and the debug
 * overlays don't have to recompute anything. Found in the capsule's own frame;
 * §8 rotates the facing direction back to world space for lighting.
 *   part         which piece was hit (HIT_NONE = miss — check this first)
 *   hit_distance how far along the ray the surface is (nearest wins)
 *   normal_obj   which way the surface faces there (drives all the lighting)
 *   axial_norm   how far up the tube the hit landed, 0 (bottom)..1 (top) — only
 *                for the debug overlay
 *   discrim      how deep through the tube the ray passed (0 grazing the edge,
 *                large through the middle) — only for the debug overlay, and
 *                filled even on a miss so it can show near-misses */
typedef struct {
  HitPart part;
  float hit_distance;
  V3 normal_obj;
  float axial_norm;
  float discrim;
} CapsuleHit;

/* The thing we're drawing: a line segment fattened by a radius — slide a ball
 * along a stick and trace the surface it sweeps out. One shared radius rounds
 * both the tube and the two end balls, which is why the body blends into the
 * caps with no visible seam. The spin angles live here (not as loose globals)
 * because they describe THIS capsule's pose. The capsule itself never moves —
 * §8 spins the rays the opposite way instead, so the maths stays in one fixed,
 * simple frame.
 *   radius       thickness of the tube and the end balls
 *   half_height  half the stick length, measured along Y
 *   spin_x/y     current tilt and turn, nudged a little each tick */
typedef struct {
  float radius;
  float half_height;
  float spin_x;
  float spin_y;
} Capsule;

/*
 * §5.1 — does the ray hit the tube? Pretend the tube runs forever, solve where
 * the ray pierces that endless cylinder, and accept the hit only if it lands
 * between the two ends. On a body hit it also reports the outward (sideways)
 * normal. It always writes how far along the axis the hit was (so the dispatcher
 * knows which end-ball to try next) and how deep the ray passed (for the debug
 * overlay) — even when the hit is out of bounds or missed.
 * (Quílez's capsule intersector; the quadratic is scaled so divisions cancel.)
 */
static HitPart cylinder_test(V3 ray_dir, V3 axis_seg, V3 origin_to_A,
                             float radius, float axis_len_sq,
                             float *out_distance, V3 *out_normal_obj,
                             float *out_axial_norm, float *out_discrim) {
  float axis_dot_dir = v3dot(axis_seg, ray_dir);
  float axis_dot_origin = v3dot(axis_seg, origin_to_A);
  float dir_dot_origin = v3dot(ray_dir, origin_to_A);
  float origin_dot_origin = v3dot(origin_to_A, origin_to_A);

  /* the three quadratic coefficients (scaled by axis length so divisions cancel) */
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

  /* how far up the axis the hit sits — the dispatcher reads this to pick which
   * end-ball to try if the body hit is rejected */
  *out_axial_norm = axial_coord / axis_len_sq;

  if (hit_distance > T_EPS && axial_coord > 0.f && axial_coord < axis_len_sq) {
    /* the surface faces straight out from the axis: take the hit point relative
     * to A and strip off its along-the-axis part, leaving the sideways part */
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
 * §5.2 — does the ray hit one rounded end? It's just a ray-vs-sphere test at the
 * end's centre. The caller passes the ray's offset from that centre, so this
 * doesn't care which end it is, and hands back the label to stamp on the hit. We
 * try the near face first, then the far face (in case the camera is inside).
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
 * §5.3 — put it together: try the tube first (it covers most of a side-on
 * capsule), and if that gives no in-bounds hit, test the nearer end-ball, chosen
 * by which end the ray passed. Returns the nearest hit, or HIT_NONE on a miss.
 */
static CapsuleHit ray_capsule(V3 ray_origin, V3 ray_dir, V3 A, V3 B,
                              float radius) {
  V3 axis_seg = v3sub(B, A);
  V3 origin_to_A = v3sub(ray_origin, A);
  float axis_len_sq = v3dot(axis_seg, axis_seg);

  CapsuleHit hit = {.part = HIT_NONE};
  /* default to the bottom end; a full tube miss can only mean both ends miss
   * too, so cap_test will return HIT_NONE there anyway */
  hit.axial_norm = 0.f;
  hit.discrim = 0.f;

  HitPart body_result = cylinder_test(
      ray_dir, axis_seg, origin_to_A, radius, axis_len_sq, &hit.hit_distance,
      &hit.normal_obj, &hit.axial_norm, &hit.discrim);
  if (body_result == HIT_BODY) {
    hit.part = HIT_BODY;
    return hit;
  }

  /* tube missed, or its hit was past one end — try the end the ray ran past */
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

/* ── §6 shading — turn a hit point into a colour ── */

/* The four ways to colour a hit, cycled with 's' — one real look plus three
 * diagnostic views of the same geometry. MODE_N is just the count, so
 * `(mode + 1) % MODE_N` cycles; keep this order in step with k_mode_names[].
 *   MODE_PHONG   the lit material — the "real" look
 *   MODE_NORMAL  paint by which way the surface faces — to inspect the normals
 *   MODE_FRESNEL only the glancing-angle rim glow — to inspect the edges
 *   MODE_DEPTH   nearer = brighter — to inspect depth ordering */
typedef enum {
  MODE_PHONG = 0,
  MODE_NORMAL,
  MODE_FRESNEL,
  MODE_DEPTH,
  MODE_N
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "fresnel",
                                           "depth"};

/* The three lamps, given as positions in the world (the shading aims each one
 * from the hit point toward its position). It's the classic film three-point
 * setup, picked for looks, not realism:
 *   KEY  = bright main light, up/front/right
 *   FILL = soft light, low/front/left — lifts the shadow side
 *   RIM  = back light — outlines the capsule against the dark */
static const V3 LIGHT_KEY = {3.0f, 4.0f, -2.0f};
static const V3 LIGHT_FILL = {-4.0f, 1.0f, -1.0f};
static const V3 LIGHT_RIM = {0.5f, -1.0f, 5.0f};

/* How strongly each lamp contributes (looks, not physics). KEY leads; FILL is
 * soft and glare-free; RIM is a wide backlight that kisses the silhouette. */
#define KEY_DIFFUSE 1.00f
#define KEY_SPECULAR 1.30f
#define FILL_DIFFUSE 0.55f /* fill is diffuse-only (no highlight) */
#define RIM_DIFFUSE 0.40f
#define RIM_SPECULAR 1.20f
#define RIM_SHININESS 10.f /* rim highlight is wider than the key's */

/* Add one white lamp's light to a running colour: a soft body glow that's
 * strongest where the surface faces the lamp, plus a bright highlight where the
 * lamp would glint toward the eye. The highlight uses the "halfway between lamp
 * and eye" test (Blinn-Phong) — a broad, steady spot that reads cleanly on the
 * coarse grid instead of flickering as a lone cell. `wrap` softens the
 * light/shadow edge so the shadow side keeps a gentle gradient; we turn it on
 * for the main KEY lamp only, so FILL and RIM still shape the form. A glow-only
 * lamp (FILL) passes a highlight strength of 0. */
static V3 add_phong_light(V3 colour, V3 light_pos, V3 point_world,
                          V3 normal_world, V3 view_dir, const Theme *th,
                          float diffuse_gain, float specular_gain,
                          float shininess, int wrap) {
  V3 light_dir = v3norm(v3sub(light_pos, point_world));
  float n_dot_l = v3dot(normal_world, light_dir);
  float diffuse;
  if (wrap) {
    float lit = 0.5f * n_dot_l + 0.5f; /* half-Lambert: light wraps around */
    diffuse = lit * lit;
  } else {
    diffuse = fmaxf(0.f, n_dot_l);
  }
  colour = v3add(
      colour, v3scale(diffuse * th->diffuse_weight * diffuse_gain, th->albedo));
  V3 half_vec = v3norm(v3add(light_dir, view_dir));
  float specular = powf(fmaxf(0.f, v3dot(normal_world, half_vec)), shininess);
  colour = v3add(colour, v3scale(specular * specular_gain, th->specular));
  return colour;
}

/* The lit look: a dim base light plus the three lamps, then the material's own
 * glow added on top. The lamps are plain white, so each material shows its own
 * colour (gold reads as gold, blue plastic as blue plastic) rather than being
 * tinted by the light. The KEY lamp gets the soft wrap; the glow is added last,
 * before clamping, so neon stays bright even in shadow. */
static V3 shade_phong(V3 point_world, V3 normal_world, V3 view_dir,
                      const Theme *th) {
  /* a dim base light so nothing is pure black */
  V3 colour = v3scale(AMBIENT, th->albedo);

  /* the three lamps (only KEY gets the soft wrap — the last argument) */
  colour =
      add_phong_light(colour, LIGHT_KEY, point_world, normal_world, view_dir, th,
                      KEY_DIFFUSE, KEY_SPECULAR, SHININESS, 1);
  colour =
      add_phong_light(colour, LIGHT_FILL, point_world, normal_world, view_dir,
                      th, FILL_DIFFUSE, 0.f, SHININESS, 0);
  colour =
      add_phong_light(colour, LIGHT_RIM, point_world, normal_world, view_dir, th,
                      RIM_DIFFUSE, RIM_SPECULAR, RIM_SHININESS, 0);

  colour = v3add(colour, th->emissive); /* self-glow, on top of the lighting */
  return v3clamp01(colour);
}

/* Diagnostic: paint each point by which way its surface faces (the three facing
 * components become red/green/blue). Correct normals show smooth gradients;
 * wrong ones show harsh colour jumps. */
static V3 shade_normal(V3 normal_world) {
  return (V3){normal_world.x * 0.5f + 0.5f, normal_world.y * 0.5f + 0.5f,
              normal_world.z * 0.5f + 0.5f};
}

/* Diagnostic: the "glass pill" rim glow — dark where the surface faces you, bright
 * at the glancing edges (a Fresnel/grazing falloff). Good for checking the body
 * and caps merge into one smooth silhouette with no crease. */
static V3 shade_fresnel(V3 normal_world, V3 view_dir, const Theme *th) {
  float cos_angle = fabsf(v3dot(normal_world, view_dir));
  float one_minus_cos = 1.f - cos_angle;
  float fresnel_factor = one_minus_cos * one_minus_cos * one_minus_cos *
                         one_minus_cos * one_minus_cos;
  V3 core = v3scale(FRESNEL_CORE, th->albedo);
  V3 edge = v3clamp01(v3scale(FRESNEL_EDGE, th->specular));
  return v3clamp01(v3add(v3scale(1.f - fresnel_factor, core),
                         v3scale(fresnel_factor, edge)));
}

/* Diagnostic: nearer surfaces drawn brighter (and squared for a steeper
 * falloff). The brightest spot should be the nearest point of the silhouette. */
static V3 shade_depth(float hit_distance, float distance_max, const Theme *th) {
  float depth_norm = 1.f - fminf(hit_distance / distance_max, 1.f);
  depth_norm = depth_norm * depth_norm;
  return v3clamp01(v3scale(depth_norm, th->albedo));
}

/* Perceived brightness of a colour — picks which ramp character to draw. Green
 * counts most because the eye is most sensitive to it. */
static inline float rec601_luma(V3 c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* Brightness for the normals view: its colour IS the facing direction, so plain
 * luma reads oddly — this weights the channels green-heavy to match how bright
 * each looks to the eye. */
static inline float normal_vis_luma(V3 normal_world) {
  return (normal_world.x * 0.5f + 0.5f) * 0.30f +
         (normal_world.y * 0.5f + 0.5f) * 0.60f +
         (normal_world.z * 0.5f + 0.5f) * 0.10f;
}

/* Run whichever look the 's' key has selected — the one spot where a hit becomes
 * a colour, so render()'s loop stays a short list of steps. */
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

/* ── §7 debug overlays — make the hit maths visible ── */

/* Overlays (cycled with 'd') that recolour each pixel by what §5 found, turning
 * the capsule maths into a picture instead of a black box. They only read the
 * hit record §5 already filled in — no extra work. DEBUG_N is the count for the
 * cycle; keep this order in step with k_debug_names[].
 *   DEBUG_OFF       no overlay — keep the lit colour
 *   DEBUG_HIT_TYPE  colour by which piece was hit (tube vs each end)
 *   DEBUG_AXIAL     colour by how far along the tube the hit landed
 *   DEBUG_DISCRIM   colour by how deep through the tube the ray passed */
typedef enum {
  DEBUG_OFF = 0,
  DEBUG_HIT_TYPE,
  DEBUG_AXIAL,
  DEBUG_DISCRIM,
  DEBUG_N
} DebugMode;
static const char *const k_debug_names[] = {"off", "hit-type", "axial",
                                            "discrim"};

/* Map a 0..1 value to a colour, deep blue → bright gold, for the overlays. */
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

/* Replace a pixel's colour with the chosen overlay's picture of the hit (or
 * leave it alone when the overlay is off). Writes through out_colour/out_luminance
 * for the caller to draw as-is. */
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
    *out_luminance = DBG_HIT_LUMA;
    break;
  }
  case DEBUG_AXIAL: {
    float t_norm = hit->axial_norm;
    if (t_norm < 0.f)
      t_norm = 0.f;
    if (t_norm > 1.f)
      t_norm = 1.f;
    *out_colour = gradient_cold_hot(t_norm);
    *out_luminance = DBG_AXIAL_LUMA_MIN + DBG_AXIAL_LUMA_SPAN * t_norm;
    break;
  }
  case DEBUG_DISCRIM: {
    /* how deep the ray went through the tube — 0 at the grazing edge, biggest
     * through the middle; scale by a typical max to land in 0..1 */
    float norm = hit->discrim / (CAP_R * DBG_DISCRIM_SCALE);
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

/* ── §7.5 colour output — load the colour slots, paint a cell ── */

static int g_have_256; /* does the terminal have 256 colours? else mono */

/* Fill in the colour slots once: the 216-colour cube for the picture, plus the
 * two HUD colours. On a terminal without 256 colours we skip the cube and fall
 * back to the brightness ramp alone (still readable, just grey). */
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

/* Paint one cell. Two channels carry the surface: the character comes from how
 * bright it is, the colour from snapping its RGB to the nearest cube slot. */
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

/* ── §8 render — one ray per cell, hit, shade, paint ── */

/* The camera ray through one terminal cell. Turn the cell into screen
 * coordinates centred on the middle, widen by the field of view, and squeeze the
 * up/down part by the cell aspect so the capsule doesn't come out as a tall oval;
 * then aim it using the camera's right/up/forward and make it unit length. */
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

  /* the capsule's current orientation (we'll spin rays by its opposite) */
  Mat3 rotation = mat3_rotation(capsule->spin_x, capsule->spin_y);

  /* the camera — fixed in place; the capsule is what turns */
  V3 camera_origin = {0.f, 0.f, -cam_dist};
  V3 view_forward = {0.f, 0.f, 1.f};
  V3 view_right = {1.f, 0.f, 0.f};
  V3 view_up = {0.f, 1.f, 0.f};

  /* the two tube ends, in the capsule's own (never-rotated) frame */
  V3 endpoint_A = {0.f, -capsule->half_height, 0.f};
  V3 endpoint_B = {0.f, +capsule->half_height, 0.f};

  float screen_centre_x = cols * 0.5f;
  float screen_centre_y = rows * 0.5f;

  /* the camera position in the capsule's frame — same for every pixel, so do it
   * once; each ray's direction gets turned inside the loop */
  V3 ray_origin_obj = mat3_mulT(rotation, camera_origin);

  /* one ray per cell (last row left blank for the hint strip) */
  for (int row = 0; row < rows - 1; row++) {
    for (int col = 0; col < cols; col++) {
      V3 ray_dir_world =
          primary_ray_dir(col, row, screen_centre_x, screen_centre_y,
                          fov_half_tan, view_right, view_up, view_forward);
      V3 ray_dir_obj = mat3_mulT(rotation, ray_dir_world); /* into capsule frame */

      CapsuleHit hit = ray_capsule(ray_origin_obj, ray_dir_obj, endpoint_A,
                                   endpoint_B, capsule->radius);
      if (hit.part == HIT_NONE)
        continue;

      /* find the hit and its facing direction back in world space (distance is
       * the same either way — turning doesn't change lengths) */
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

/* ── §9 HUD — the status line and the key hints ── */

static void hud_draw(int cols, int rows, float fps, const Theme *th,
                     ShadeMode shade_mode, DebugMode debug_mode, float cam_dist,
                     int paused) {
  /* status, pinned to the right; fps sits in its own slot so a long material
   * name can't push it off a narrow screen */
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

  /* current shade + debug mode, top-left */
  char left_label[80];
  snprintf(left_label, sizeof left_label, " mode:%-7s  debug:%-8s ",
           k_mode_names[shade_mode], k_debug_names[debug_mode]);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, "%s", left_label);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* key hints along the bottom */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc/p:pause  s:mode  d:debug  t:theme  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §9.5 scene — everything that changes while it runs ── */

/* One bundle of all the runtime state, so the main loop reads like a table of
 * contents. Most functions take just the piece they need; only scene_init /
 * scene_advance take the whole Scene.
 *   capsule    the spinning shape (its pose lives inside it, not as loose globals)
 *   theme_idx  } the view knobs — they only change the picture, never the
 *   shade_mode } physics: which material, which look, which debug overlay, how
 *   debug_mode } far the camera sits, and whether the spin is frozen
 *   cam_dist
 *   paused
 *   cols/rows  terminal size, re-read on resize so the picture fits
 *   fps*       the fps readout (a rolling average), shown in the HUD only */
typedef struct {
  Capsule capsule;

  int theme_idx;
  ShadeMode shade_mode;
  DebugMode debug_mode;
  float cam_dist;
  int paused;

  int cols, rows;

  float fps;
  long long fps_accum_ns;
  int fps_frames;
} Scene;

/* Set the opening state: the capsule, default view knobs, the current size. */
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

/* One tick: turn the capsule a little (unless paused) and update the fps
 * average. The only place the running state changes — drawing never touches it. */
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

/* ── §10 app — set up, run the loop, handle keys ── */

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
    /* window resized? re-read the size first (this happens between frames) */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, scene.rows, scene.cols);
    }

    /* time since last frame, capped so a hiccup can't fast-forward the spin */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* advance: the one place the running state changes (spin frozen while paused) */
    scene_advance(&scene, dt);

    /* draw: build the frame and the HUD, then flip it on screen */
    const Theme *th = &g_themes[scene.theme_idx % THEME_N];
    long long frame_start = clock_ns();
    erase();
    render(&scene.capsule, th, scene.shade_mode, scene.debug_mode,
           scene.cam_dist, scene.cols, scene.rows);
    hud_draw(scene.cols, scene.rows, scene.fps, th, scene.shade_mode,
             scene.debug_mode, scene.cam_dist, scene.paused);
    wnoutrefresh(stdscr);
    doupdate();

    /* keys: each one is a one-off change to the view/controls, never the spin */
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

    /* sleep off the rest of the frame to hold the target rate */
    clock_sleep_ns(frame_ns - (clock_ns() - frame_start));
  }
  return 0;
}
