/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* sphere_raytrace.c — a lit sphere drawn into the terminal in colour. For each
 * character cell we shoot one ray; if it hits the sphere we work out the
 * colour there and draw it. The camera slowly circles the sphere. Press 's' to
 * switch between the real lit look and three diagnostic views.
 * Sister files: cube_raytrace.c / capsule_raytrace.c / torus_raytrace.c (same
 * skeleton, other shapes); path_tracer.c (same colour pipeline, many bounces).
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
#define DT_CAP_NS 100000000LL         /* if a frame stalls longer than this (0.1s), pretend it didn't */
#define FPS_AVG_WINDOW_NS 500000000LL /* refresh the on-screen fps number every 0.5s */

/* The camera lens. Terminal cells are about twice as tall as wide, so we
 * squash by ASPECT to keep the sphere round instead of egg-shaped. */
#define ASPECT 0.47f  /* cell width / height */
#define FOV_DEG 58.0f /* how wide the view is, top to bottom */

#define SPHERE_R 1.0f /* the sphere's radius; it sits at the origin */

/* The camera circles the sphere at a fixed height; +/- change how far back. */
#define ORBIT_SPEED 0.32f /* how fast it circles (radians per second) */
#define CAM_HEIGHT 0.55f  /* how high above the sphere's equator it rides */
#define CAM_DIST_DEF 3.6f /* starting distance */
#define CAM_DIST_MIN 1.9f /* closest zoom */
#define CAM_DIST_MAX 7.0f /* farthest zoom */
#define CAM_DIST_STEP 0.25f

/* Lighting. A faint ambient floor so nothing is pure black, a shiny-highlight
 * tightness, and the relative strengths of the three lights (see shade_phong).
 * The sphere's colour comes from the material, not the lights — they're white. */
#define AMBIENT 0.20f      /* faint glow everywhere, so no cell is fully black */
#define AMBIENT_HEMI 0.50f /* and a touch brighter on top than the bottom, like
                              soft sky light, so the sphere reads as round */
#define SHININESS 75.0f /* highlight tightness — high = a small sharp metal glint */
#define KEY_DIFFUSE 1.00f   /* main light — primary brightness */
#define KEY_SPEC_GAIN 1.30f /* main light — its hard highlight */
#define FILL_DIFFUSE 0.55f  /* fill light — softens the shadow side */
#define RIM_DIFFUSE 0.40f   /* back light — faint glow on the far edge */
#define RIM_SPEC_GAIN 1.20f /* back light — a highlight that grazes the silhouette */
#define RIM_SHININESS 10.0f /* back light's highlight is wide and soft (vs 75) */
/* "Powerful light" mode (the 'l' key): turn the fill and back lights off and
 * crank the main light up, so one strong source clearly shapes the sphere —
 * bright lit side, dark shadow side, a hard highlight. */
#define POWER_DIFFUSE 2.00f   /* boosted main-light brightness */
#define POWER_SPEC_GAIN 2.50f /* boosted main-light highlight */
#define DEPTH_FAR_SCALE 2.2f    /* depth view: how far counts as "fully far" */
#define FRESNEL_CORE 0.06f      /* glass view: how dark the head-on centre is */
#define FRESNEL_EDGE_GAIN 1.10f /* glass view: how bright the grazing edge is */

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

/* ignore a hit closer than this — it's the surface hitting itself */
#define T_EPS 1e-4f

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

/* §3 — small 3-D vector maths. */

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
    /* sapphire — deep blue. The green is nudged to 0.22 on purpose: with the
     * terminal's coarse 6-level colour grid, 0.30 made the smooth shading band
     * into ugly patches; 0.22 keeps it smooth. Blue pushed to 0.95 for punch. */
    {{0.10f, 0.22f, 0.95f},
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

/* §5 — does a ray hit the sphere? This is the heart of the whole program: the
 * "where does a line cross a ball" test. It solves the quadratic you get from
 * "for what distance along the ray is the point exactly one radius from the
 * centre?" and hands back the nearest hit in front of us, or 0 for a miss. The
 * ray direction must be length 1. (The far root is kept so it still works if
 * the camera is ever inside the sphere.) */
static int ray_sphere(V3 ro, V3 rd, float r, float *t_hit) {
  float b = v3dot(rd, ro);
  float c = v3dot(ro, ro) - r * r;
  float disc = b * b - c;
  if (disc < 0.f)
    return 0; /* ray passes wide of the sphere */

  float sq = sqrtf(disc);
  float t0 = -b - sq; /* near side (where the ray enters) */
  float t1 = -b + sq; /* far side  (where it exits) */

  if (t1 < T_EPS) /* even the far side is behind us → miss */
    return 0;

  /* take the near side; fall back to the far side if we started inside */
  *t_hit = (t0 > T_EPS) ? t0 : t1;
  return 1;
}

/* §6 — working out a colour for a sphere hit. Four ways to look at the same
 * hit, switched with the 's' key. */

/* The four views. The geometry (where we hit, which way it faces) is identical
 * in all of them — only the colouring changes:
 *   PHONG   — the real, lit material look. The default.
 *   NORMAL  — paint each spot by which way it faces; a quick "are the surface
 *             directions right?" rainbow check.
 *   FRESNEL — a glass-marble look: dark in the middle, bright at the rim.
 *   DEPTH   — brightness by distance: near is bright, far is dark. */
typedef enum {
  MODE_PHONG = 0,
  MODE_NORMAL,
  MODE_FRESNEL,
  MODE_DEPTH,
  MODE_N /* how many modes there are, so 's' can wrap around */
} ShadeMode;
static const char *const k_mode_names[] = {"phong", "normals", "fresnel",
                                           "depth"};

/* The three lights, as positions in the world. They're plain white and stay
 * put while the camera circles, so the bright spot drifts across the sphere —
 * which is how you can tell a featureless ball is turning. All the colour
 * comes from the material, never the lights. */
static const V3 LIGHT_KEY = {3.0f, 4.0f, -2.0f};   /* main, from the upper-right */
static const V3 LIGHT_FILL = {-4.0f, 1.0f, -1.0f}; /* fill, from the upper-left */
static const V3 LIGHT_RIM = {0.5f, -1.0f, 5.0f};   /* back light, from behind */

/* Add one white light's contribution to a spot: a soft glow, brightest where
 * the surface faces the light, plus a shiny highlight where the light bounces
 * straight toward the camera. spec_gain = 0 means a soft fill with no
 * highlight. */
static V3 add_phong_light(V3 col, V3 light_pos, V3 P, V3 N, V3 V_dir,
                          float diffuse_gain, float spec_gain, float shininess,
                          const Theme *th) {
  V3 L = v3norm(v3sub(light_pos, P));
  float d = fmaxf(0.f, v3dot(N, L));
  col = v3add(col, v3scale(d * th->diffuse_weight * diffuse_gain, th->albedo));
  /* Only add a highlight where the light actually reaches the surface
   * (d > 0). Without this gate a light behind the sphere can leak a
   * specular spot onto the front face it can't possibly light. */
  if (d > 0.f) {
    V3 R = v3reflect(v3scale(-1.f, L), N);
    float s = powf(fmaxf(0.f, v3dot(R, V_dir)), shininess);
    col = v3add(col, v3scale(s * spec_gain, th->specular));
  }
  return col;
}

/* The real lit look: start from a faint ambient floor, then add the lights —
 * the three-point rig normally, or one boosted light in 'l' mode. The material
 * decides the colours; the lights are white. P is the hit point, N which way
 * the surface faces there, V_dir the way back to the camera. */
static V3 shade_phong(V3 P, V3 N, V3 V_dir, const Theme *th, int power) {
  /* Hemisphere ambient floor: a little brighter on top, dimmer underneath,
   * like soft sky light from above — not a flat fill. The gentle top-to-
   * bottom gradient keeps the sphere reading as round even at orbit angles
   * where the three lights barely graze the visible face (which matters on a
   * coarse cell grid). Stays above zero, so no pure-black "hole" cells. */
  float amb = AMBIENT * (1.f + AMBIENT_HEMI * N.y);
  V3 col = v3scale(amb, th->albedo);

  if (power) {
    /* one strong light only (the 'l' key): fill + back off, main light cranked
     * up, so its shaping is obvious — bright lit side, near-black shadow side */
    col = add_phong_light(col, LIGHT_KEY, P, N, V_dir, POWER_DIFFUSE,
                          POWER_SPEC_GAIN, SHININESS, th);
  } else {
    /* main light: the primary brightness + a sharp highlight */
    col = add_phong_light(col, LIGHT_KEY, P, N, V_dir, KEY_DIFFUSE,
                          KEY_SPEC_GAIN, SHININESS, th);
    /* fill light: softens the shadow side, no highlight of its own */
    col = add_phong_light(col, LIGHT_FILL, P, N, V_dir, FILL_DIFFUSE, 0.f,
                          SHININESS, th);
    /* back light: a soft wide glow that catches the far edge */
    col = add_phong_light(col, LIGHT_RIM, P, N, V_dir, RIM_DIFFUSE,
                          RIM_SPEC_GAIN, RIM_SHININESS, th);
  }

  /* neon's own glow, added last so it shows even on the shadow side */
  col = v3add(col, th->emissive);
  return v3clamp1(col);
}

/* Debug view: paint a spot by which way it faces (each axis → a colour
 * channel), so the sphere becomes a smooth "rainbow ball." A quick way to
 * confirm the surface directions come out right everywhere. */
static V3 shade_normal(V3 N) {
  return (V3){N.x * .5f + .5f, N.y * .5f + .5f, N.z * .5f + .5f};
}

/* Glass-marble view: dark looking straight at the surface, bright at the
 * grazing edge — like light glancing off a glass ball. The brightening rushes
 * in only very near the rim, so the inside stays dark with a bright outline. */
static V3 shade_fresnel(V3 N, V3 V_dir, const Theme *th) {
  float cosA = fabsf(v3dot(N, V_dir)); /* 1 head-on, 0 at the grazing edge */
  float inv = 1.f - cosA;
  float fresnel = inv * inv * inv * inv * inv; /* near 0 inside, near 1 at the rim */
  V3 core = v3scale(FRESNEL_CORE, th->albedo);
  V3 edge = v3clamp1(v3scale(FRESNEL_EDGE_GAIN, th->specular));
  return v3clamp1(v3add(v3scale(1.f - fresnel, core), v3scale(fresnel, edge)));
}

/* Depth view: brightness by distance — near surfaces bright, far ones dark
 * (and the drop-off is steepened so it's easy to read). */
static V3 shade_depth(float t, float t_max, const Theme *th) {
  float d = 1.f - fminf(t / t_max, 1.f);
  d = d * d;
  return v3clamp1(v3scale(d, th->albedo));
}

/* how bright a colour looks to the eye (green counts most) */
static inline float rec601_luma(V3 c) {
  return 0.299f * c.x + 0.587f * c.y + 0.114f * c.z;
}

/* Pick the colour for one sphere hit using whichever view is active, and hand
 * back its brightness (via *lum) for choosing the character. Same hit, four
 * looks — the whole point of the demo. */
static V3 shade_pixel(ShadeMode mode, V3 P, V3 N, V3 V_dir, float t_hit,
                      float cam_dist, const Theme *th, int power, float *lum) {
  V3 color;
  switch (mode) {
  default:
  case MODE_PHONG:
    color = shade_phong(P, N, V_dir, th, power);
    *lum = rec601_luma(color);
    break;
  case MODE_NORMAL:
    color = shade_normal(N);
    /* weight green most — the eye reads green as the brightest channel */
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

/* The scene's state. The sphere never moves — the camera circles it, so the
 * camera is just an angle (how far round) and a distance (the zoom). The rest
 * are the knobs the user changes with keys. Only init / tick rewrite this; the
 * drawing code only reads it. */

/* where the camera is on its circle around the sphere */
typedef struct {
  float orbit_ang; /* how far round (radians); advances each tick */
  float dist;      /* how far back (the zoom) */
} Camera;

typedef struct {
  Camera camera;  /* the orbiting viewpoint */
  int paused;     /* freeze the orbit */
  int theme_idx;  /* which material */
  ShadeMode mode; /* which of the four views */
  int powerlight; /* 1 = one strong light, 0 = balanced three-point */
} Scene;

/* start fresh at launch: gold, lit view, default zoom, running */
static void scene_init(Scene *s) {
  s->camera.orbit_ang = 0.f;
  s->camera.dist = CAM_DIST_DEF;
  s->paused = 0;
  s->theme_idx = 0;
  s->mode = MODE_PHONG;
  s->powerlight = 0; /* start three-point; pressing 'l' shows the contrast */
}

/* The one and only place the camera advances round its circle (skipped while
 * paused). dt_ns is how much real time passed, so the speed is frame-rate
 * independent. */
static void scene_tick(Scene *s, long long dt_ns) {
  if (s->paused)
    return;
  s->camera.orbit_ang += ORBIT_SPEED * (float)dt_ns * 1e-9f;
}

/* §7 — drawing a frame. */

/* Draw one frame: place the camera on its circle, then shoot a ray through
 * every cell and colour whatever it hits. We orbit the camera rather than spin
 * the sphere — same picture, but it keeps the sphere parked at the origin so
 * the hit test stays simple. */
static void render(const Scene *s, int cols, int rows) {
  const Theme *th = &g_themes[s->theme_idx % THEME_N];
  float fov_tan = tanf(FOV_DEG * (float)M_PI / 360.f); /* half the view angle */

  /* place the camera on its circle and point it at the sphere; rgt/up are the
   * screen's right and up directions from there */
  V3 cam = {s->camera.dist * sinf(s->camera.orbit_ang), CAM_HEIGHT,
            -s->camera.dist * cosf(s->camera.orbit_ang)};
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
       * sphere stays round in tall cells */
      float pu = (col - cx) / cx * fov_tan;
      float pv = -(row - cy) / cx * fov_tan / ASPECT;

      V3 rd = v3norm(v3add(fwd, v3add(v3scale(pu, rgt), v3scale(pv, up))));

      float t_hit;
      if (!ray_sphere(cam, rd, SPHERE_R, &t_hit))
        continue; /* missed the sphere — leave the cell as background */

      V3 P = v3add(cam, v3scale(t_hit, rd)); /* the hit point */
      V3 N = v3norm(P);                      /* which way the surface faces (sphere at origin) */
      V3 V_dir = v3norm(v3sub(cam, P));      /* back toward the camera */

      float lum;
      V3 color = shade_pixel(s->mode, P, N, V_dir, t_hit, s->camera.dist, th,
                             s->powerlight, &lum);
      draw_color(row, col, color, lum);
    }
  }
}

/* §8 — the text overlay: a status line along the top and the key hints along
 * the bottom. */

static void hud_draw(const Scene *s, float fps, int cols, int rows) {
  /* status, top-right */
  char buf[96];
  snprintf(buf, sizeof buf, " %5.1f fps  dist:%.1f  %-9s  light:%s  %s ",
           (double)fps, (double)s->camera.dist,
           g_themes[s->theme_idx % THEME_N].name,
           s->powerlight ? "1-POWER" : "3-point",
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
           " q:quit  spc/p:pause  s:mode  t:theme  l:light  +/-:zoom ");
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
  case 's':
  case 'S':
    s->mode = (ShadeMode)((s->mode + 1) % MODE_N);
    break;
  case 't':
  case 'T':
    s->theme_idx = (s->theme_idx + 1) % THEME_N;
    break;
  case 'l':
  case 'L':
    s->powerlight = !s->powerlight;
    break;
  case '+':
  case '=':
    s->camera.dist -= CAM_DIST_STEP;
    if (s->camera.dist < CAM_DIST_MIN)
      s->camera.dist = CAM_DIST_MIN;
    break;
  case '-':
  case '_':
    s->camera.dist += CAM_DIST_STEP;
    if (s->camera.dist > CAM_DIST_MAX)
      s->camera.dist = CAM_DIST_MAX;
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

    /* advance the orbit (does nothing while paused) */
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

