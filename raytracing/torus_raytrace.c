/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* torus_raytrace.c — a lit, tumbling donut drawn into the terminal in colour.
 * For each character cell we shoot one ray; finding where it hits a donut takes
 * solving a 4th-degree equation (the hard part — sphere and cube are much
 * easier). Press 's' to switch between the real lit look and three debug views.
 * Sister files: sphere_raytrace.c / cube_raytrace.c / capsule_raytrace.c (same
 * skeleton, easier shapes); path_tracer.c (same colour pipeline, many bounces).
 * Keys are shown along the bottom of the screen while it runs. */

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

/* §1 — config: every tunable number in one place, so none of them turn up as
 * a mystery value in the code below. */

/* How fast we run. */
#define TARGET_FPS 60
#define DT_CAP_NS 100000000LL         /* if a frame stalls past this (0.1s), pretend it didn't */
#define FPS_AVG_WINDOW_NS 500000000LL /* refresh the on-screen fps number every 0.5s */

/* The camera lens. Terminal cells are about twice as tall as wide, so we
 * squash by ASPECT to keep the donut round instead of egg-shaped. */
#define ASPECT 0.47f  /* cell width / height */
#define FOV_DEG 52.0f /* how wide the view is, top to bottom */

/* The donut, sitting flat at the origin: a ring of radius R, made of a tube of
 * radius r. The tube/ring ratio sets how chunky it looks. */
#define TORUS_R 0.68f /* ring radius — centre of the tube to the centre of the hole */
#define TORUS_r 0.28f /* tube radius — how fat the tube is */

/* How fast the donut tumbles (radians per second). */
#define ROT_Y 0.40f /* main spin */
#define ROT_X 0.18f /* slow tilt */

/* The camera sits up high looking down at the donut; +/- change how far back. */
#define CAM_DIST_DEF 3.4f /* starting distance */
#define CAM_DIST_MIN 1.6f /* closest zoom */
#define CAM_DIST_MAX 7.0f /* farthest zoom */
#define CAM_DIST_STEP 0.25f
#define CAM_HEIGHT 1.8f   /* how high above the donut it rides */

/* Lighting. A faint ambient floor so nothing is pure black, a shiny-highlight
 * tightness, and the relative strengths of the three lights (see shade_phong).
 * The colour comes from the material, not the lights — they're white. */
#define AMBIENT 0.20f   /* dim base light so the shadow side isn't pure black */
#define SHININESS 75.0f /* highlight tightness — higher = sharper, glossier */
#define KEY_DIFFUSE 1.00f   /* main light — primary brightness */
#define KEY_SPEC_GAIN 1.30f /* main light — its hard highlight */
#define FILL_DIFFUSE 0.55f  /* fill light — softens the shadow side */
#define RIM_DIFFUSE 0.40f   /* back light — faint glow on the far edge */
#define RIM_SPEC_GAIN 1.20f /* back light — a highlight that grazes the silhouette */
#define RIM_SHININESS 10.0f /* back light's highlight is wide and soft (vs 75) */
#define DEPTH_FAR_SCALE 2.5f    /* depth view: how far counts as "fully far" */
#define FRESNEL_CORE 0.06f      /* glass view: how dark the head-on centre is */
#define FRESNEL_EDGE_GAIN 1.10f /* glass view: how bright the grazing edge is */

/* How we find where a ray hits the donut. There's no neat formula, so we walk
 * along the ray sampling a function that's zero exactly at the surface, spot
 * where it flips sign (we just passed through the surface), then zero in on the
 * crossing by repeatedly halving the gap. More samples catch thin near-edge
 * hits; more halvings pin the hit down more precisely. */
#define Q_SAMPLES 256  /* sample points along each ray */
#define Q_BISECT 40    /* halvings to pin down the crossing */
#define Q_T_MAX 18.0f  /* how far along the ray to bother looking */

/* The characters we draw with, faintest (space) to densest ('@'), so a
 * brighter cell picks a denser character. From Paul Bourke's ASCII-art ramp. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* Colour slots we hand to ncurses: a 6×6×6 grid of 216 colours starting at
 * PAIR_CUBE_BASE, plus two for the HUD. */
#define PAIR_CUBE_BASE 1
#define PAIR_HUD 217
#define PAIR_HINT 218

/* ignore hits closer than this — they're floating-point noise at the surface */
#define T_EPS 1e-3f /* lower bound for t scan      */

/* §2 — keeping time: a steady nanosecond clock and a plain sleep, used to
 * pace the loop. The clock only moves forward, so timing never glitches if the
 * system clock gets adjusted. */

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

/* §3 — small 3-D maths: vectors, and a rotation matrix for the tumble. */

/* Three floats that wear three hats, all using the same add/scale/dot helpers:
 *   • a point in space (the camera, a hit point, a light)
 *   • a direction (a ray, which way a surface faces) — kept length-1 by v3norm
 *   • a colour (red, green, blue), normally 0..1
 * One type for all three is on purpose: the same maths that moves points also
 * blends colours. */
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

/* The donut's current tumble, as a little 3×3 grid of numbers (a rotation).
 * Built once a frame from the two tumble angles, then used two ways: turn a
 * direction from the donut's own frame out into the world (mat3_mul), and the
 * reverse, turn a world direction into the donut's frame (mat3_mulT). The handy
 * trick: for a pure rotation, going the reverse way is just reading the grid by
 * columns instead of rows — no costly inverse needed. That's what lets us keep
 * the donut sitting still in its own frame and spin the rays into it instead,
 * where the hit maths is far simpler (see §7). */
typedef struct {
  V3 r[3]; /* the three rows of the rotation */
} Mat3;

/* Build the tumble rotation from the pitch and yaw angles. */
static Mat3 mat3_rot(float rx, float ry) {
  float cx = cosf(rx), sx = sinf(rx);
  float cy = cosf(ry), sy = sinf(ry);
  Mat3 m;
  m.r[0] = (V3){cy, 0.f, sy};
  m.r[1] = (V3){sx * sy, cx, -sx * cy};
  m.r[2] = (V3){-cx * sy, sx, cx * cy};
  return m;
}

/* turn a direction from the donut's frame out into the world */
static V3 mat3_mul(Mat3 m, V3 v) {
  return (V3){v3dot(m.r[0], v), v3dot(m.r[1], v), v3dot(m.r[2], v)};
}

/* turn a direction from the world into the donut's frame (the reverse) */
static V3 mat3_mulT(Mat3 m, V3 v) {
  return (V3){m.r[0].x * v.x + m.r[1].x * v.y + m.r[2].x * v.z,
              m.r[0].y * v.x + m.r[1].y * v.y + m.r[2].y * v.z,
              m.r[0].z * v.x + m.r[1].z * v.y + m.r[2].z * v.z};
}

/* §4 — materials and putting colour on screen. */

/* One material — what a surface is made of. The lights are plain white, so a
 * material's whole look comes from these numbers; that's why gold looks gold
 * under any light. Shared with the other shape demos so they all describe
 * materials the same way. Cycled with t / T.
 *   albedo   — the body colour you'd see under flat white light.
 *   specular — the colour of the shiny highlight. For metals it matches the
 *              body colour (gold glints yellow); for everything else it's
 *              near-white.
 *   emissive — colour it glows on its own, added after lighting so it shows
 *              even in shadow. Zero except for neon.
 *   diffuse_weight — how much body colour shows vs. highlight: low for metals
 *              (~0.15, mostly shiny), high for plastics (~0.85, mostly body). */
typedef struct {
  V3 albedo;            /* body colour */
  V3 specular;          /* highlight colour (metal: = albedo; else: ~white) */
  V3 emissive;          /* self-glow, added after lighting (neon only) */
  float diffuse_weight; /* body-vs-highlight balance, 0.10..0.90 */
  const char *name;     /* HUD label, e.g. "gold" / "sapphire" */
} Theme;

static const Theme g_themes[] = {
    /* === METALS (12) — highlight matches the body colour, little body shows.
     * Metals are almost all shine, and the shine carries their colour. */

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

    /* === GEMS (4) — deep saturated body, white highlight, medium body show.
     * The colour comes from light being absorbed inside the crystal. */

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

    /* === DIELECTRICS (3) — plastics, ceramic, glass: body colour + a plain
     * white highlight. */

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

    /* === EMISSIVE (1) — neon: glows on its own. The body is the dim "off"
     * tube colour; the glow shows hot pink even on the shadow side. */

    /* neon     — hot pink/magenta self-glow                            */
    {{0.05f, 0.02f, 0.10f},
     {0.80f, 0.80f, 1.00f},
     {1.00f, 0.20f, 0.85f},
     0.20f,
     "neon"},
};
#define THEME_N ((int)(sizeof g_themes / sizeof g_themes[0]))

static int g_256; /* true if the terminal has the full 256-colour set */

/* claim the colour slots once at startup, so painting each cell is cheap */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    /* fill the 216 slots with the 6×6×6 colour grid (xterm colours 16..231) */
    for (int i = 0; i < 216; i++)
      init_pair(PAIR_CUBE_BASE + i, 16 + i, -1);
  }
  init_pair(PAIR_HUD, 226, -1); /* bright yellow */
  init_pair(PAIR_HINT, 51, -1); /* bright cyan */
}

/* brightness (0..1) → a character: faint cells get a sparse one, bright cells
 * a dense one. This is the "how bright" half of a cell. */
static char luma_to_ramp_char(float lum) {
  if (lum < 0.f)
    lum = 0.f;
  if (lum > 1.f)
    lum = 1.f;
  return k_ramp[(int)(lum * (RAMP_LEN - 1))];
}

/* a colour → its nearest slot in the 6×6×6 grid: snap each channel to one of 6
 * levels. This is the "what colour" half of a cell. */
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

/* §5 — does a ray hit the donut, and where? This is the hard part. A donut's
 * surface is described by an equation with a square root in it; plug the ray
 * into it and clear the root and you get a 4th-degree polynomial in the
 * distance along the ray (a "quartic"). Wherever that polynomial crosses zero
 * is where the ray touches the donut. Three steps: build the polynomial, find
 * its first crossing (ray_torus), then get the surface direction there
 * (torus_normal). */

/* The four numbers that define q(t) = t⁴ + A·t³ + B·t² + C·t + D — the
 * polynomial whose zero-crossings are the distances along a ray where it meets
 * the donut. (The leading t⁴ part is always 1, so it isn't stored.) Bundled so
 * the builder, the evaluator, and the crossing-finder pass one value, not four
 * loose floats. */
typedef struct {
  float A, B, C, D; /* coefficients of t³, t², t, and the constant term */
} Quartic;

/* Evaluate the quartic at distance t. Written "inside-out" (nested multiplies)
 * — same answer as spelling out t⁴+…, just fewer multiplications, which matters
 * because we call this hundreds of times per ray. */
static inline float q_eval(float t, Quartic q) {
  return t * (t * (t * (t + q.A) + q.B) + q.C) + q.D;
}

/* Build the quartic for one ray against the donut: plug the ray into the
 * donut's surface equation and clear the square root. The algebra is fiddly;
 * the result is just these four coefficients. */
static Quartic torus_quartic_coeffs(V3 ro, V3 rd, float R, float r_minor) {
  float po2 = v3dot(ro, ro);                  /* |ro|²        */
  float rod = v3dot(rd, ro);                  /* rd · ro      */
  float rxz2 = ro.x * ro.x + ro.z * ro.z;     /* in-plane |ro|² */
  float rdxz_d = rd.x * ro.x + rd.z * ro.z;   /* in-plane rd·ro*/
  float rdxz2 = rd.x * rd.x + rd.z * rd.z;    /* in-plane |rd|²*/
  float C0 = po2 + R * R - r_minor * r_minor; /* common offset */
  Quartic q;
  q.A = 4.f * rod;
  q.B = 4.f * rod * rod + 2.f * C0 - 4.f * R * R * rdxz2;
  q.C = 4.f * rod * C0 - 8.f * R * R * rdxz_d;
  q.D = C0 * C0 - 4.f * R * R * rxz2;
  return q;
}

/* Pin down a crossing already trapped between lo and hi (the quartic has
 * opposite signs at the two ends): repeatedly halve the gap, keep the half that
 * still straddles the crossing, return the middle. We use this rather than the
 * textbook closed-form quartic formula because that formula blows up on grazing
 * rays; halving never divides by anything risky. */
static float bisect_root(float lo, float hi, float flo, Quartic q) {
  for (int j = 0; j < Q_BISECT; j++) {
    float mid = (lo + hi) * 0.5f;
    float fmid = q_eval(mid, q);
    if (flo * fmid < 0.f) {
      hi = mid; /* root in (lo, mid)       */
    } else {
      lo = mid;
      flo = fmid; /* root in (mid, hi)       */
    }
  }
  return (lo + hi) * 0.5f; /* midpoint of final bracket */
}

/* Find where the ray first hits the donut. Walk outward in small steps,
 * checking the quartic's sign at each: the moment it flips (was positive, now
 * negative or vice-versa) we just stepped through the surface, so the hit sits
 * between the last two steps — hand that gap to bisect_root to pin down. We
 * take the first crossing, which is the nearest visible surface. Returns the
 * distance in *t_hit, or 0 if the ray misses entirely. The ray is given in the
 * donut's own frame (so the donut lies flat).
 *
 * A couple of corners: a ray that just grazes the surface dips to zero without
 * flipping sign, so it reads as a miss (fine). And if two hits fall in the same
 * step we only catch the first — but the steps are far finer than the donut, so
 * that only ever happens right at the silhouette edge. */
static int ray_torus(V3 ro, V3 rd, float R, float r_minor, float *t_hit) {
  Quartic q = torus_quartic_coeffs(ro, rd, R, r_minor);

  /* step outward, watching for the sign to flip */
  float dt = Q_T_MAX / (float)Q_SAMPLES;
  float t0 = T_EPS;
  float f0 = q_eval(t0, q);

  for (int i = 1; i <= Q_SAMPLES; i++) {
    float t1 = (float)i * dt;
    float f1 = q_eval(t1, q);

    if (f0 * f1 < 0.f) { /* sign flipped → the surface is between t0 and t1 */
      *t_hit = bisect_root(t0, t1, f0, q);
      return 1;
    }
    t0 = t1;
    f0 = f1;
  }
  return 0; /* never flipped → the ray missed */
}

/* Which way the donut's surface faces at a hit point. Picture the tube as
 * wrapped around a centre ring: the outward direction is simply the line from
 * the nearest point on that ring to the hit. To find the nearest ring point,
 * drop the height (flatten onto the donut's plane) and stretch back out to the
 * ring's radius. (The `rho` near zero case can't really happen — it would be on
 * the donut's central axis, inside the hole — but the guard avoids a divide by
 * zero.) */
static V3 torus_normal(V3 P, float R) {
  V3 P_xz = {P.x, 0.f, P.z};
  float rho = v3len(P_xz);
  V3 ring_pt = (rho > 1e-9f) ? v3scale(R / rho, P_xz)
                             : (V3){R, 0.f, 0.f}; /* defensive default */
  return v3norm(v3sub(P, ring_pt));
}

/* §6 — working out a colour for a donut hit. Four ways to look at the same
 * hit, switched with the 's' key. */

/* The four views. The geometry (where we hit, which way it faces) is identical
 * in all of them — only the colouring changes:
 *   PHONG   — the real, lit material look. The default.
 *   NORMAL  — paint each spot by which way it faces; a quick "are the surface
 *             directions right?" rainbow check.
 *   FRESNEL — a glass-marble look: dark facing you, bright at the edges.
 *   DEPTH   — brightness by distance: near is bright, far is dark. */
typedef enum {
  MODE_PHONG = 0,
  MODE_NORMAL,
  MODE_FRESNEL,
  MODE_DEPTH,
  MODE_N /* how many views there are, so 's' can wrap around */
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "fresnel",
                                           "depth"};

/* The three lights, as positions in the world. They're plain white and stay
 * put while the donut tumbles, so the bright spot sweeps across it. All the
 * colour comes from the material, never the lights. */
static const V3 LIGHT_KEY = {3.0f, 4.0f, -2.0f};   /* main, from the upper-right */
static const V3 LIGHT_FILL = {-4.0f, 1.0f, -1.0f}; /* fill, from the upper-left */
static const V3 LIGHT_RIM = {0.5f, -1.0f, 5.0f};   /* back light, from behind */

/* Add one white lamp's light to a running colour: a soft body glow that's
 * strongest where the surface faces the lamp, plus a bright highlight where the
 * lamp would glint toward the eye. The highlight uses the "halfway between lamp
 * and eye" test (Blinn-Phong) — a broad, steady spot that reads cleanly on the
 * coarse grid instead of flickering as a lone cell. `wrap` softens the
 * light/shadow edge so the shadow side keeps a gentle gradient; we turn it on
 * for the main KEY lamp only, so FILL and RIM still shape the form. A glow-only
 * lamp (FILL) passes a highlight strength of 0. */
static V3 add_phong_light(V3 col, V3 light_pos, V3 P, V3 N, V3 V_dir,
                          float diffuse_gain, float spec_gain, float shininess,
                          const Theme *th, int wrap) {
  V3 L = v3norm(v3sub(light_pos, P));
  float n_dot_l = v3dot(N, L);
  float diffuse;
  if (wrap) {
    float lit = 0.5f * n_dot_l + 0.5f; /* half-Lambert: light wraps around */
    diffuse = lit * lit;
  } else {
    diffuse = fmaxf(0.f, n_dot_l);
  }
  col = v3add(col,
              v3scale(diffuse * th->diffuse_weight * diffuse_gain, th->albedo));
  V3 H = v3norm(v3add(L, V_dir)); /* halfway between lamp and eye */
  float s = powf(fmaxf(0.f, v3dot(N, H)), shininess);
  col = v3add(col, v3scale(s * spec_gain, th->specular));
  return col;
}

/* The lit look: a dim base light plus the three lamps, then the material's own
 * glow added on top. The lamps are plain white, so each material shows its own
 * colour (gold reads as gold, blue plastic as blue plastic). The KEY lamp gets
 * the soft wrap; the glow is added last, before clamping, so neon stays bright
 * even in shadow. */
static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th) {
  V3 col = v3scale(AMBIENT, th->albedo); /* dim base so nothing is pure black */

  /* the three lamps — only KEY gets the soft wrap (the last argument) */
  col = add_phong_light(col, LIGHT_KEY, P, N, V_dir, KEY_DIFFUSE, KEY_SPEC_GAIN,
                        SHININESS, th, 1);
  col = add_phong_light(col, LIGHT_FILL, P, N, V_dir, FILL_DIFFUSE, 0.f,
                        SHININESS, th, 0);
  col = add_phong_light(col, LIGHT_RIM, P, N, V_dir, RIM_DIFFUSE, RIM_SPEC_GAIN,
                        RIM_SHININESS, th, 0);

  /* neon's own glow, added last so it shows even on the shadow side */
  col = v3add(col, th->emissive);
  return v3clamp1(col);
}

/* Debug view: paint a spot by which way it faces (each axis → a colour
 * channel), so the donut becomes a rainbow. On a torus the tube faces outward
 * round the rim, inward toward the hole, and up/down on top and bottom — so you
 * see two clear colour belts. A quick "are the surface directions right?" check. */
static V3 shade_normal(V3 N) {
  return (V3){N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f};
}

/* Glass-marble view: dark looking straight at the surface, bright at the
 * grazing edges — and a torus has lots of edges (outer rim, inner hole, top and
 * bottom of the tube), so it lights up with bright outlines everywhere. */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th) {
  float cosA = fabsf(v3dot(N, V_dir)); /* 1 head-on, 0 at the grazing edge */
  float inv = 1.f - cosA;
  float fresnel = inv * inv * inv * inv * inv; /* near 0 inside, near 1 at the rim */
  V3 core = v3scale(FRESNEL_CORE, th->albedo);
  V3 edge = v3clamp1(v3scale(FRESNEL_EDGE_GAIN, th->specular));
  return v3clamp1(v3add(v3scale(1.f - fresnel, core), v3scale(fresnel, edge)));
}

/* Depth view: brightness by distance — near surfaces bright, far ones dark
 * (drop-off steepened so it reads easily). */
static V3 shade_depth(float t, float t_max, const Theme *th) {
  float d = 1.f - fminf(t / t_max, 1.f);
  d = d * d;
  return v3clamp1(v3scale(d, th->albedo));
}

/* how bright a colour looks to the eye (green counts most) */
static inline float rec601_luma(V3 c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* Pick the colour for one donut hit using whichever view is active, and hand
 * back its brightness (via *lum) for choosing the character. Same hit, four
 * looks — the whole point of the demo. */
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

/* The scene's state. The donut never moves from the origin — it just tumbles
 * in place — and its size is fixed config, so the only thing that "happens" is
 * its two tumble angles turning. Everything else is a knob the user changes
 * with keys. There's no camera struct: the camera is fixed except for how far
 * back it sits (the zoom). Only init/reset/tick rewrite this; drawing reads it. */
typedef struct {
  float angle_x;  /* tumble pitch — advances each tick */
  float angle_y;  /* tumble yaw   — advances each tick */
  float cam_dist; /* how far back the camera sits (zoom) */
  int paused;     /* freeze the tumble */
  int theme_idx;  /* which material */
  ShadeMode mode; /* which of the four views */
} Scene;

/* start fresh at launch: gold, lit view, default zoom, untilted, running */
static void scene_init(Scene *s) {
  s->angle_x = 0.f;
  s->angle_y = 0.f;
  s->cam_dist = CAM_DIST_DEF;
  s->paused = 0;
  s->theme_idx = 0;
  s->mode = MODE_PHONG;
}

/* the 'r' key: set the donut back upright, leaving zoom/material/pause alone */
static void scene_reset(Scene *s) {
  s->angle_x = 0.f;
  s->angle_y = 0.f;
}

/* The one and only place the tumble advances (skipped while paused). dt_ns is
 * how much real time passed, so it spins at the same rate at any frame rate. */
static void scene_tick(Scene *s, long long dt_ns) {
  if (s->paused)
    return;
  float sec = (float)dt_ns * 1e-9f;
  s->angle_y += ROT_Y * sec;
  s->angle_x += ROT_X * sec;
}

/* §7 — drawing a frame.
 *
 * The trick that keeps the hit maths simple: instead of spinning the donut
 * (which would make its surface equation horrendous), we leave the donut parked
 * flat and spin each RAY the opposite way before testing it. Because a rotation
 * "undoes" by just reading its grid sideways, that backward spin is one cheap
 * step per cell, and the donut stays in the easy flat-at-the-origin position. */
static void render(const Scene *s, int cols, int rows) {
  const Theme *th = &g_themes[s->theme_idx % THEME_N];
  float fov_tan = tanf(FOV_DEG * (float)M_PI / 360.f); /* half the view angle */

  Mat3 M = mat3_rot(s->angle_x, s->angle_y);

  /* the camera, up high looking down at the origin; rgt/up are the screen's
   * right and up directions from there */
  V3 cam = {0.f, CAM_HEIGHT, -s->cam_dist};
  V3 fwd = v3norm(v3sub((V3){0, 0, 0}, cam));
  V3 wup = {0.f, 1.f, 0.f};
  V3 rgt = v3norm(v3cross(fwd, wup));
  V3 up = v3cross(rgt, fwd);

  float cx = cols * 0.5f, cy = rows * 0.5f;

  /* one ray per cell; leave the bottom row for the HUD */
  for (int row = 0; row < rows - 1; row++) {
    for (int col = 0; col < cols; col++) {
      /* this cell as an offset from screen centre; the up/down term is flipped
       * (screen rows go down, world up goes up) and divided by ASPECT so the
       * donut stays round in tall cells */
      float pu = (col - cx) / cx * fov_tan;
      float pv = -(row - cy) / cx * fov_tan / ASPECT;

      V3 rd_ws = v3norm(v3add(fwd, v3add(v3scale(pu, rgt), v3scale(pv, up))));

      /* spin the ray into the donut's frame, where the donut lies flat */
      V3 ro_os = mat3_mulT(M, cam);
      V3 rd_os = mat3_mulT(M, rd_ws);

      float t_hit;
      if (!ray_torus(ro_os, rd_os, TORUS_R, TORUS_r, &t_hit))
        continue; /* missed the donut — leave the cell as background */

      /* the hit point (in both frames) and the surface direction back in the
       * world, plus the direction back to the camera */
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

/* §8 — the text overlay: a status line along the top and the key hints along
 * the bottom. */

static void hud_draw(const Scene *s, float fps, int cols, int rows) {
  /* status, top-right */
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

  /* current view, top-left */
  char buf2[48];
  snprintf(buf2, sizeof buf2, " mode:%-9s ", k_mode_names[s->mode]);
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, "%s", buf2);
  attroff(COLOR_PAIR(PAIR_HUD));

  /* key hints, bottom row */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc/p:pause  s:mode  t:theme  r:reset  +/-:zoom ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §9 — the program itself: signals, the keyboard, and the main loop. */

/* set by signal handlers, checked by the loop */
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

/* install_signals — quit on INT/TERM, flag a resize on WINCH, restore the
 * terminal on exit. */
static void install_signals(void) {
  signal(SIGINT, on_sigint);
  signal(SIGTERM, on_sigint);
  signal(SIGWINCH, on_sigwinch);
  atexit(cleanup);
}

/* screen_init — put the terminal into raw, non-blocking, no-echo render mode
 * (hidden cursor, keypad, no input tearing) and allocate the colour pairs. */
static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  color_init();
}

/* handle_key — apply one keypress to the scene knobs. Returns 0 to quit, 1 to
 * keep running. A user event, NOT part of the per-frame tick. */
static int handle_key(Scene *s, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return 0;
  case ' ':
  case 'p':
  case 'P':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reset(s);
    break;
  case 's':
  case 'S':
    s->mode = (ShadeMode)((s->mode + 1) % MODE_N);
    break;
  case 't':
  case 'T':
    s->theme_idx = (s->theme_idx + 1) % THEME_N;
    break;
  case '+':
  case '=':
    s->cam_dist -= CAM_DIST_STEP;
    if (s->cam_dist < CAM_DIST_MIN)
      s->cam_dist = CAM_DIST_MIN;
    break;
  case '-':
  case '_':
    s->cam_dist += CAM_DIST_STEP;
    if (s->cam_dist > CAM_DIST_MAX)
      s->cam_dist = CAM_DIST_MAX;
    break;
  default:
    break;
  }
  return 1;
}

int main(void) {
  install_signals();
  screen_init();

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
    /* one loop = one frame */

    /* apply a pending window resize */
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
    }

    /* how much real time passed, capped so a long stall doesn't lurch ahead */
    long long now = clock_ns();
    long long dt = now - last;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    last = now;

    /* advance the tumble (does nothing while paused) */
    scene_tick(&scene, dt);

    /* refresh the on-screen fps number every so often */
    fps_acc += dt;
    fps_cnt++;
    if (fps_acc >= FPS_AVG_WINDOW_NS) {
      fps = (float)fps_cnt * 1e9f / (float)fps_acc;
      fps_acc = 0;
      fps_cnt = 0;
    }

    /* draw the frame and flip it to the screen */
    long long t0 = clock_ns();
    erase();
    render(&scene, cols, rows);
    hud_draw(&scene, fps, cols, rows);
    wnoutrefresh(stdscr);
    doupdate();

    /* read one keypress; quitting is the only thing that stops the loop */
    int ch = getch();
    if (ch != ERR && !handle_key(&scene, ch))
      g_run = 0;

    /* sleep off the rest of the frame's budget so we don't run flat out */
    clock_sleep_ns(frame_ns - (clock_ns() - t0));
  }
  return 0;
}

