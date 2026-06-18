/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* solar_eclipse.c — a solar eclipse drawn into the terminal in colour.
 * For every character cell we trace one ray and step it through the thin
 * glowing gas around the sun (the corona), so the corona, the red rim, the
 * prominences and the beads of light all fall out of one physics loop rather
 * than being painted on. The moon drifts across the sun and back.
 * Sister files: sphere_raytrace.c (the ray-vs-sphere math up close),
 * path_tracer.c (the colour-to-terminal pipeline). Keys show along the
 * bottom of the screen while it runs. */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* §0 — the two core number-bags, declared first so everything below can use
 * them. Both are just three floats with no methods. */

/* A point or a direction in 3-D space (which one is clear from the name). */
typedef struct {
  float x, y, z;
} V3;
/* A colour. Brightness can go above 1.0 (it's "HDR"); §5 squashes it down to
 * something the terminal can show. */
typedef struct {
  float r, g, b;
} RGB;

/* §1 — config: every tunable number in one place, so none of them turn up as
 * a mystery value buried in the code below. */

/* How fast time moves. The world updates on a steady clock (ticks/sec); speed
 * multiplies how fast the moon drifts (turn it down for slow-mo at the edge
 * of totality). */
enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))
#define DT_CAP_NS (100 * NS_PER_MS) /* after a stall, never advance more than this */

/* The camera lens. zoom shrinks the field of view (bigger objects); ASPECT_Y
 * squashes vertically so the sun stays round, not egg-shaped, in tall cells. */
#define ASPECT_Y 2.0f
#define FOV_H_BASE 0.40f /* base field-of-view width; effective = this / zoom */
#define ZOOM_MIN 0.25f
#define ZOOM_MAX 8.0f
#define ZOOM_STEP 1.25f

/* Where everything sits. The sun is far and big; the moon is near and small,
 * but because it's so much closer it looks LARGER in the sky — which is what
 * lets it cover the sun. Made deliberately larger (~1.7x) so totality reads
 * clearly: a dark moon with the corona haloed around it. */
#define SUN_Z 50.0f             /* sun distance */
#define SUN_R 3.5f              /* sun radius */
#define MOON_Z 5.0f             /* moon distance (much nearer) */
#define MOON_BASE_R_TOTAL 0.62f /* moon radius: fully covers the sun */
#define MOON_BASE_R_ANNULAR 0.32f /* smaller: leaves a ring of sun showing */
#define MOON_BASE_R_TRANSIT 0.04f /* tiny: just a dot crossing the disc */

#define MOON_SCALE_MIN 0.5f  /* live moon-size multiplier (m / M keys) */
#define MOON_SCALE_MAX 2.0f
#define MOON_SCALE_STEP 1.10f

#define ECLIPSE_PERIOD_S 30.0f /* seconds for the moon to cross and return */
#define MOON_ORBIT_X_FRAC 1.5f /* how far it swings sideways (1.5 = clean in/out) */
#define PARTIAL_Y_OFFSET 0.025f /* lift for the PARTIAL pattern, so it never centres */

/* The corona — the faint glowing gas around the sun. Its thickness falls off
 * fast with distance from the sun's edge, keeping the glow tight. CORONA_REACH
 * is where we treat it as empty so the ray can stride through quickly. (The
 * real corona is fantastically faint; these numbers are scaled up so it shows
 * on screen — the shape of the falloff is what's real, not the magnitude.) */
#define CORONA_REACH 8.0f   /* corona fades to nothing past this x sun radius */
#define CORONA_SIGMA0 0.18f /* how thick the corona is right at the sun's edge */
#define CORONA_DECAY 3.0f   /* how fast it thins out with distance */

/* The chromosphere — the thin red rim hugging the sun's surface. Real one is
 * paper-thin; we make it 2.5% of the radius so it's visible at this size. */
#define CHROMOS_THICK 0.025f /* shell thickness, as a fraction of sun radius */
#define CHROMOS_SIGMA0 6.0f  /* how dense (and bright-red) it is */

/* Spicules — the fine hairy texture on the red rim (countless little jets on
 * the real sun). We fake it with noise; hot patches lean toward yellow. */
#define SPICULE_FREQ_PHI 12.0f /* noise detail around the rim */
#define SPICULE_FREQ_TH 6.0f   /* noise detail up/down the rim */
#define SPICULE_BASE 0.5f      /* baseline rim brightness */
#define SPICULE_AMP 1.2f       /* how much the texture varies it */

/* Prominences — the bright arcs of gas looping off the sun's edge. A handful,
 * placed at random angles each reseed, each with its own height and width. */
#define PROM_COUNT 5       /* how many loops */
#define PROM_HEIGHT 0.18f  /* tallest reach, as a fraction of sun radius */
#define PROM_LATERAL 0.18f /* angular width of each */
#define PROM_SIGMA0 8.0f   /* how dense / bright they are */

/* The ray-marcher's dials. We walk each ray forward in small steps, sampling
 * the glowing gas; these set how many steps, how bright each glowing thing is
 * on screen, and where the marcher can speed up. The brightness gains are big
 * because real space spans a huge range we have to squeeze into a terminal. */
#define MARCH_STEPS 96               /* fine steps along a ray */
#define MARCH_T_MAX (SUN_Z + SUN_R * 6.0f) /* how far a sky ray marches */
#define LUT_SIZE 256                 /* entries in the see-through table (§10) */
#define LUT_R_MAX (SUN_R * (CORONA_REACH + 2.0f)) /* outer radius that table covers */
#define IN_SCATTER_GAIN 14.0f        /* corona brightness on screen */
#define SUN_EMIT_HDR 8.0f            /* sun-surface brightness (saturates to white) */
#define HA_EMIT_GAIN 3.0f            /* red-rim + prominence brightness */
#define MARCH_COARSE_MULT 4.0f       /* step 4x bigger out where space is empty */
#define CORONA_FINE_MARGIN 0.5f      /* keep fine stepping a touch past the corona */
/* Step ultra-fine within this x sun-radius of the sun: the red rim and the
 * prominences are thinner than a normal step, so without this the ray would
 * stride right over them and they'd come out dotty. The band is wide enough
 * that any step able to reach the rim already starts inside it. */
#define MARCH_DETAIL_REACH 1.30f
#define MARCH_DETAIL_DT (SUN_R * CHROMOS_THICK * 0.5f) /* ~2 samples across the rim */
#define TRANSMITTANCE_CUTOFF 1e-3f /* stop a ray once almost no light is left */
#define DENSITY_EPS 1e-6f          /* skip empty space */
#define VIS_EPS 1e-4f              /* skip in-scatter where the sun is fully hidden */

/* Eye adaptation. Just like your eyes, the corona only shows once the bright
 * sun is hidden. This gate (0..1) multiplies the corona's brightness: 0 while
 * the sun is in view, rising to 1 at totality. Bump FLOOR up to keep a faint
 * glow always. */
#define CORONA_GATE_FLOOR 0.00f
#define CORONA_GATE_RANGE 1.00f

/* How the surfaces catch light. The sun's disc is brightest at its centre and
 * dims toward the edge (we see through more of its hazy air there). The moon's
 * near face is dark — turned away from the sun — so it only catches a faint
 * cool-blue "earthshine" (light bounced off the Earth). EARTHSHINE_GAIN is kept
 * small on purpose: the tone-mapping lifts dim values hard, so a small amount
 * still shows; too much and the moon greys out and stops reading as a dark
 * silhouette against the corona. */
#define LIMB_AMBIENT 0.40f      /* sun-edge brightness (fraction of centre) */
#define LIMB_GAIN 0.60f         /* extra brightness toward the centre */
#define MOON_ALBEDO 0.12f       /* how reflective the moon is (it's dark) */
#define EARTH_ALBEDO_BLUE 0.45f /* the blue tint of earthshine */
#define EARTHSHINE_GAIN 0.04f   /* how bright the moon's earthshine glow is */

/* The moon's bumpy edge. We dent its outline with valleys so that, at the
 * moment of contact, sunlight leaks through the gaps as bright "beads". A few
 * wide lopsided ones plus many narrow ones; one wide valley per reseed is
 * extra-deep and makes the single bright "diamond ring" flash. The lopsided
 * shape makes beads pop in sharply on one side and fade on the other, like
 * real mountain shadows. All baked into one angle->dent table per reseed. */
#define LUNAR_VALLEY_COUNT 14    /* wide valleys */
#define LUNAR_VALLEY_DEPTH 0.020f
#define LUNAR_VALLEY_WIDTH 0.06f
#define LUNAR_VALLEY_ASYM 0.45f   /* how lopsided each wide valley is */
#define LUNAR_DIAMOND_BOOST 2.5f  /* the one extra-deep valley (diamond ring) */

#define LUNAR_MICRO_COUNT 30      /* narrow valleys (the smaller beads) */
#define LUNAR_MICRO_DEPTH 0.010f
#define LUNAR_MICRO_WIDTH 0.025f

#define LUNAR_LUT_SIZE 512        /* table resolution (big enough for the narrow ones) */

/* Background stars — only visible during totality, when the sun isn't washing
 * them out. Each gets a random temperature, so the field has the real warm-to-
 * cool colour spread instead of being all white. */
#define STAR_DENSITY 250       /* roughly one star per this many sky cells */
#define STAR_OCC_VISIBLE 0.85f /* only show stars once the sun is this hidden */
#define STAR_K_MIN 3000.0f     /* coolest (red) star temperature */
#define STAR_K_MAX 30000.0f    /* hottest (blue) */
#define STAR_LUMA_MAX 0.04f    /* only place stars on cells dimmer than this */
#define STAR_BRIGHT_MIN 0.6f   /* dimmest star */
#define STAR_BRIGHT_RANGE 0.4f /* brightness spread above the minimum */

/* The off-screen image we render into before copying to the terminal. Fixed
 * size so we never allocate while running; a terminal bigger than this just
 * clips (rare — 400x200 is generous). */
#define BUF_MAX_W 400
#define BUF_MAX_H 200

/* Colour slots we hand to ncurses. We pack a 6x6x6 grid of 216 colours into
 * consecutive slots from PAIR_CUBE_BASE; the low slots are the HUD. The draw
 * characters are all "open" shapes (no solid blocks), so bright cells look
 * like points of light rather than filled pixels. */
enum {
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_SUN_FALLBACK = 3,
  PAIR_EVENT_HOT = 4,
  PAIR_CUBE_BASE = 16,
};

static const char k_ramp[] = " .'`,-_:;~=+*oO0";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* Turning a colour into a cell: snap it onto the 6-levels-per-channel grid,
 * and draw the very brightest cells bold and the very darkest dim for a bit
 * of extra contrast. */
#define CUBE_SIDE 6
#define LUMA_BOLD_THRESH 0.85f
#define LUMA_DIM_THRESH 0.15f

/* Which kind of eclipse we're showing, switched with the n / p keys. Each one
 * just sets a different moon size or offset (see pattern_set). */
typedef enum {
  PATTERN_TOTAL = 0,   /* moon fully covers the sun → corona + beads */
  PATTERN_PARTIAL = 1, /* moon rides high → just a bite out of the disc */
  PATTERN_ANNULAR = 2, /* moon too small → a "ring of fire" left showing */
  PATTERN_TRANSIT = 3, /* tiny moon → a dot crossing the disc */
  N_PATTERNS = 4,      /* how many, so the keys can wrap around */
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_TOTAL:
    return "TOTAL  ";
  case PATTERN_PARTIAL:
    return "PARTIAL";
  case PATTERN_ANNULAR:
    return "ANNULAR";
  case PATTERN_TRANSIT:
    return "TRANSIT";
  default:
    return "?      ";
  }
}

/* A type of star, named by its real astronomy class, with one number: how hot
 * it is (in Kelvin). That temperature alone decides the sun's colour — and
 * since the corona is just scattered sunlight, its colour too. So a cool star
 * gives a deep-red sun and a hot one a blue sun, all from real physics, no
 * hand-picked palettes. Cycled with t / T. */
typedef struct {
  const char *name;
  float kelvin; /* temperature; drives every colour in the scene */
} Star;

static const Star STARS[] = {
    {"M-DWARF", 3500.0f}, {"K-STAR ", 4500.0f},  {"G-STAR ", 5778.0f},
    {"A-STAR ", 9500.0f}, {"B-STAR ", 18000.0f},
};
#define N_STARS ((int)(sizeof STARS / sizeof STARS[0]))

/* §2 — keeping time: a steady nanosecond clock and a plain sleep, used to pace
 * the loop. The clock only moves forward, so timing never glitches if the
 * system clock gets adjusted. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

/* sleep this long (does nothing for zero/negative); the loop uses it to avoid
 * pinning a CPU core at full speed */
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* §3 — small maths: 3-D vectors, colour arithmetic, and the helpers that turn
 * a finished HDR colour into something a screen can show. All tiny and called
 * a lot, so they're inline. */

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3_add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3_sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3_scl(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline float v3_dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline float v3_len(V3 a) { return sqrtf(v3_dot(a, a)); }
static inline V3 v3_norm(V3 a) {
  float l = v3_len(a);
  if (l < 1e-12f) /* guard against a zero-length vector */
    return v3(0, 0, 0);
  return v3_scl(a, 1.0f / l);
}

static inline RGB rgb_make(float r, float g, float b) { return (RGB){r, g, b}; }
static inline RGB rgb_add(RGB a, RGB b) {
  return rgb_make(a.r + b.r, a.g + b.g, a.b + b.b);
}
static inline RGB rgb_mul(RGB a, RGB b) {
  return rgb_make(a.r * b.r, a.g * b.g, a.b * b.b);
}
static inline RGB rgb_scl(RGB a, float s) {
  return rgb_make(a.r * s, a.g * s, a.b * s);
}

static inline float clamp01(float x) {
  return x < 0.f ? 0.f : (x > 1.f ? 1.f : x);
}
/* fold an angle back into the -180°..180° range — i.e. the shortest way round
 * a circle. Used wherever we measure angular distance (the moon's bumpy edge,
 * the prominences). */
static inline float wrap_pi(float a) {
  while (a > (float)M_PI)
    a -= 2.0f * (float)M_PI;
  while (a < -(float)M_PI)
    a += 2.0f * (float)M_PI;
  return a;
}
/* gently squash very bright values into 0..1 so nothing blows out to white */
static inline float reinhard(float x) { return x / (1.f + x); }
/* nudge brightness to match how a screen actually shows it */
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }
/* how bright a colour looks to the eye (green counts most) */
static inline float luma_of(RGB c) {
  return 0.2126f * c.r + 0.7152f * c.g + 0.0722f * c.b;
}

/* The colour something glows at a given temperature: cool is red, hot is blue
 * (think of heated metal). This is where every colour in the scene starts —
 * the sun, the stars, and the corona that scatters the sun's light. The magic
 * numbers are a well-known curve fit (Tanner Helland, 2012); it gives the hue,
 * not the brightness. */
static RGB blackbody_rgb(float kelvin) {
  float K = kelvin / 100.0f;
  float r, g, b;

  if (K <= 66.0f)
    r = 1.0f;
  else
    r = 329.7f * powf(K - 60.0f, -0.1332f) / 255.0f;

  if (K <= 66.0f)
    g = (99.47f * logf(K) - 161.12f) / 255.0f;
  else
    g = 288.12f * powf(K - 60.0f, -0.0755f) / 255.0f;

  if (K >= 66.0f)
    b = 1.0f;
  else if (K <= 19.0f)
    b = 0.0f;
  else
    b = (138.52f * logf(K - 10.0f) - 305.04f) / 255.0f;

  return rgb_make(clamp01(r), clamp01(g), clamp01(b));
}

/* §4 — randomness: a hash that turns a few integers into a repeatable random
 * number (for star spots, valley shapes, prominence placement), plus Perlin
 * noise — smooth natural-looking randomness — for the hairy texture on the red
 * rim. The noise table is reshuffled on reseed ('r') so each look is new. */

/* scramble three integers into one repeatable random number */
static inline uint32_t hash3(int wx, int wy, int wz) {
  uint32_t h = (uint32_t)wx * 73856093u ^ (uint32_t)wy * 19349663u ^
               (uint32_t)wz * 83492791u;
  h ^= h >> 16;
  h *= 0x85ebca6bu;
  h ^= h >> 13;
  h *= 0xc2b2ae35u;
  h ^= h >> 16;
  return h;
}

/* turn a scrambled number into a random float from 0 up to (not quite) 1 */
static inline float hash01(uint32_t h) {
  return (float)(h & 0xFFFFFFu) * (1.f / (float)0x1000000u);
}

/* Perlin noise machinery. The maths is standard Perlin — look it up if
 * curious; here it's a black box that returns a smooth random value. The
 * shuffled table below is what makes each reseed look different. */

static uint8_t perm[512]; /* the noise's shuffled lookup table */

/* reshuffle the noise table so reseeding ('r') gives a fresh texture */
static void perm_shuffle(int seed) {
  uint8_t base[256];
  for (int i = 0; i < 256; i++)
    base[i] = (uint8_t)i;
  uint32_t st = (uint32_t)seed * 2654435761u;
  for (int i = 255; i > 0; i--) {
    st = st * 1664525u + 1013904223u;
    int j = (int)(st >> 16) % (i + 1);
    uint8_t t = base[i];
    base[i] = base[j];
    base[j] = t;
  }
  for (int i = 0; i < 256; i++) {
    perm[i] = base[i];
    perm[i + 256] = base[i];
  }
}

static inline float fade_q(float t) {
  return t * t * t * (t * (t * 6.f - 15.f) + 10.f);
}
static inline float lerp_f(float a, float b, float t) {
  return a + t * (b - a);
}
static inline float grad2(int hash, float x, float y) {
  int h = hash & 7;
  float u = (h < 4) ? x : y;
  float v = (h < 4) ? y : x;
  return ((h & 1) ? -u : u) + ((h & 2) ? -2.f * v : 2.f * v);
}

/* smooth pseudo-random value at point (x,y), wobbling around 0 */
static float perlin2d(float x, float y) {
  int X = (int)floorf(x) & 255;
  int Y = (int)floorf(y) & 255;
  x -= floorf(x);
  y -= floorf(y);
  float u = fade_q(x), v = fade_q(y);
  int A = perm[X] + Y;
  int B = perm[X + 1] + Y;
  float n00 = grad2(perm[A], x, y);
  float n10 = grad2(perm[B], x - 1.f, y);
  float n01 = grad2(perm[A + 1], x, y - 1.f);
  float n11 = grad2(perm[B + 1], x - 1.f, y - 1.f);
  return lerp_f(lerp_f(n00, n10, u), lerp_f(n01, n11, u), v);
}

/* stack a few sizes of Perlin noise for richer, cloud-like detail; 0..1 */
static float fbm2(float x, float y) {
  float total = 0.f, amp = 1.f, freq = 1.f, max_amp = 0.f;
  for (int o = 0; o < 3; o++) {
    total += amp * perlin2d(x * freq, y * freq);
    max_amp += amp;
    amp *= 0.5f;
    freq *= 2.0f;
  }
  return (total / max_amp) * 0.5f + 0.5f;
}

/* §5 — putting colour on screen. A terminal can't show arbitrary colours, so
 * we set up a fixed 6×6×6 grid of 216 colours once, then snap each computed
 * colour to the nearest one. Old 8-colour terminals fall back to plain yellow
 * — the eclipse still reads, just monochrome. */

static int g_256; /* true if the terminal has the full 256-colour set */

/* claim the colour slots once at startup, so painting each cell is cheap */
static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
    init_pair(PAIR_EVENT_HOT, 196, -1);
  } else {
    init_pair(PAIR_SUN_FALLBACK, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_EVENT_HOT, COLOR_RED, -1);
  }
}

/* paint_cell's pipeline, as named steps. */

/* snap one display-encoded channel [0,1] to a cube axis 0..CUBE_SIDE-1 */
static inline int cube_axis(float c) {
  int q = (int)(c * (float)(CUBE_SIDE - 1) + 0.5f);
  return q < 0 ? 0 : (q > CUBE_SIDE - 1 ? CUBE_SIDE - 1 : q);
}

/* display-encoded RGB → its slot in the 6×6×6 colour cube */
static inline int rgb_to_cube_pair(float r, float g, float b) {
  return PAIR_CUBE_BASE + cube_axis(r) * CUBE_SIDE * CUBE_SIDE +
         cube_axis(g) * CUBE_SIDE + cube_axis(b);
}

/* brightness → density glyph (faint space..bright '0') */
static inline char ramp_glyph(float luma) {
  int ri = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
  return k_ramp[ri < 0 ? 0 : (ri >= RAMP_LEN ? RAMP_LEN - 1 : ri)];
}

/* brightness → bold for the brightest cells, dim for the darkest */
static inline int luma_to_attr(float luma) {
  return (luma > LUMA_BOLD_THRESH)  ? A_BOLD
         : (luma < LUMA_DIM_THRESH) ? A_DIM
                                    : A_NORMAL;
}

/*
 * paint_cell — the only place a computed colour meets the terminal. First tame
 * its brightness into a showable range, then pick the nearest grid colour for
 * the hue and a character for the brightness. (Any post-process would go here.)
 */
static void paint_cell(int sx, int sy, RGB col) {
  RGB enc = rgb_make(gamma_enc(reinhard(col.r)), gamma_enc(reinhard(col.g)),
                     gamma_enc(reinhard(col.b)));
  float luma = luma_of(enc);
  char glyph = ramp_glyph(luma);

  if (g_256) {
    int pair = rgb_to_cube_pair(enc.r, enc.g, enc.b);
    int attr = luma_to_attr(luma);
    attron(COLOR_PAIR(pair) | attr);
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(pair) | attr);
  } else {
    attron(COLOR_PAIR(PAIR_SUN_FALLBACK));
    mvaddch(sy, sx, (chtype)(unsigned char)glyph);
    attroff(COLOR_PAIR(PAIR_SUN_FALLBACK));
  }
}

/* §6 — does a ray hit a sphere? This is the "where does a line cross a ball"
 * test, used for the sun. It hands back the distance to the nearest hit in
 * front of us (in *out_t), or false if the ray sails past. The moon gets a
 * bumpy-edge variant in §7. */
static bool ray_sphere(V3 ro, V3 rd, V3 c, float r, float *out_t) {
  V3 oc = v3_sub(ro, c);
  float b = v3_dot(oc, rd);
  float cc = v3_dot(oc, oc) - r * r;
  float disc = b * b - cc;
  if (disc < 0)
    return false;
  float sq = sqrtf(disc);
  float t = -b - sq;
  if (t < 1e-3f)
    t = -b + sq;
  if (t < 1e-3f)
    return false;
  *out_t = t;
  return true;
}

/* §7 — the moon's bumpy edge. A real moon has mountains and valleys around
 * its rim, and that's what makes the beads of light: at the moment the sun is
 * almost covered, sunlight leaks through the low spots. We fake it by letting
 * the moon's radius dip a little depending on direction. All the dips are
 * worked out once per reseed and stored in a table indexed by angle around the
 * rim; the runtime lookup is then instant. */

static float lunar_lut[LUNAR_LUT_SIZE]; /* angle-around-the-rim → how much the radius dips */

/* Work out the moon's dented outline for this reseed and bake it into the
 * table. Scatter the wide and narrow valleys at random angles, make one wide
 * one extra-deep (the diamond ring), then fill each table slot with the total
 * dip at that angle. Same seed → same outline. */
static void build_lunar_lut(int seed) {
  /* The wide, lopsided valleys — the main dents in the outline. */
  float phi_v[LUNAR_VALLEY_COUNT];
  float w2_left_v[LUNAR_VALLEY_COUNT];
  float w2_right_v[LUNAR_VALLEY_COUNT];
  float d_v[LUNAR_VALLEY_COUNT];

  /* Pick which macro valley is the diamond-ring (extra depth). */
  uint32_t dr_h = hash3(0, seed, 0xD1A47000u);
  int diamond_idx = (int)(hash01(dr_h) * (float)LUNAR_VALLEY_COUNT);
  if (diamond_idx >= LUNAR_VALLEY_COUNT)
    diamond_idx = LUNAR_VALLEY_COUNT - 1;

  for (int k = 0; k < LUNAR_VALLEY_COUNT; k++) {
    uint32_t h = hash3(k, seed, 0x1A11E1);
    phi_v[k] = (hash01(h) - 0.5f) * 2.0f * (float)M_PI;
    float w_base = LUNAR_VALLEY_WIDTH * (0.5f + 1.0f * hash01(h ^ 0xE1u));

    /* one side steep, one side gradual — beads pop in sharply on the steep
     * side and fade out on the gradual side, like real mountain shadows */
    float asym_sign = (hash01(h ^ 0xAA00u) > 0.5f) ? 1.0f : -1.0f;
    float w_left = w_base * (1.0f + asym_sign * LUNAR_VALLEY_ASYM);
    float w_right = w_base * (1.0f - asym_sign * LUNAR_VALLEY_ASYM);
    w2_left_v[k] = w_left * w_left;
    w2_right_v[k] = w_right * w_right;

    float d_k = LUNAR_VALLEY_DEPTH * (0.4f + 0.6f * hash01(h ^ 0xD0u));
    if (k == diamond_idx)
      d_k *= LUNAR_DIAMOND_BOOST;
    d_v[k] = d_k;
  }

  /* The narrow valleys — the smaller, crisper beads. */
  float phi_s[LUNAR_MICRO_COUNT];
  float w2_s[LUNAR_MICRO_COUNT];
  float d_s[LUNAR_MICRO_COUNT];
  for (int k = 0; k < LUNAR_MICRO_COUNT; k++) {
    uint32_t h = hash3(k, seed, 0xCAFEBABEu);
    phi_s[k] = (hash01(h) - 0.5f) * 2.0f * (float)M_PI;
    float w_k = LUNAR_MICRO_WIDTH * (0.6f + 0.8f * hash01(h ^ 0xE2u));
    w2_s[k] = w_k * w_k;
    d_s[k] = LUNAR_MICRO_DEPTH * (0.5f + 0.7f * hash01(h ^ 0xD1u));
  }

  for (int i = 0; i < LUNAR_LUT_SIZE; i++) {
    float phi = ((float)i / (float)LUNAR_LUT_SIZE) * 2.0f * (float)M_PI -
                (float)M_PI; /* phi ∈ [-π, π) */

    float total = 0.0f;
    /* add up the wide valleys (each steeper on one side) */
    for (int k = 0; k < LUNAR_VALLEY_COUNT; k++) {
      float dphi = wrap_pi(phi - phi_v[k]);
      float w2 = (dphi < 0.0f) ? w2_left_v[k] : w2_right_v[k];
      total += d_v[k] * expf(-(dphi * dphi) / w2);
    }
    /* add up the narrow valleys */
    for (int k = 0; k < LUNAR_MICRO_COUNT; k++) {
      float dphi = wrap_pi(phi - phi_s[k]);
      total += d_s[k] * expf(-(dphi * dphi) / w2_s[k]);
    }
    lunar_lut[i] = total;
  }
}

/* Look up the moon's dented radius toward a given point on its rim, blending
 * smoothly between the two nearest table entries. */
static float lunar_R_at(V3 from_centre, float moon_R_base, int seed) {
  (void)seed; /* baked into the LUT at build time */
  float phi = atan2f(from_centre.y, from_centre.x); /* (-π, π] */
  float u = (phi + (float)M_PI) / (2.0f * (float)M_PI) * (float)LUNAR_LUT_SIZE;
  if (u < 0.0f)
    u += (float)LUNAR_LUT_SIZE;
  if (u >= LUNAR_LUT_SIZE)
    u -= (float)LUNAR_LUT_SIZE;
  int i0 = (int)u;
  int i1 = (i0 + 1) % LUNAR_LUT_SIZE;
  float fr = u - (float)i0;
  float depth = lunar_lut[i0] * (1.0f - fr) + lunar_lut[i1] * fr;
  return moon_R_base * (1.0f - depth);
}

/* Does this ray hit the (dented) moon? First a quick test against a plain
 * round moon — if that misses, the dented one misses too (the dents only ever
 * cut inward), so we skip the costly lookup. Otherwise we re-test against the
 * dented radius right where the ray grazes the edge. This is the eye-ray test;
 * the soft shadow edge uses §8 instead. */
static bool ray_moon(V3 ro, V3 rd, V3 c, float moon_R_base, int seed,
                     float *out_t) {
  V3 oc = v3_sub(ro, c);
  float b = v3_dot(oc, rd);

  /* quick miss-test against a plain round moon first */
  float oc2 = v3_dot(oc, oc);
  float r2 = moon_R_base * moon_R_base;
  float disc = b * b - (oc2 - r2);
  if (disc < 0)
    return false;
  float sq = sqrtf(disc);
  float t_far = -b + sq;
  if (t_far < 1e-3f)
    return false;

  /* it's near the moon — re-test against the dented edge right here */
  V3 nearest = v3_add(ro, v3_scl(rd, -b));
  V3 radial = v3_sub(nearest, c);
  float r_eff = lunar_R_at(v3_norm(radial), moon_R_base, seed);

  float cc_p = oc2 - r_eff * r_eff;
  float disc_p = b * b - cc_p;
  if (disc_p < 0)
    return false;
  float sq_p = sqrtf(disc_p);
  float t = -b - sq_p;
  if (t < 1e-3f)
    t = -b + sq_p;
  if (t < 1e-3f)
    return false;
  *out_t = t;
  return true;
}

/* §8 — how much of the sun a point can see. From a spot out in the corona,
 * the sun and moon both look like little discs in the sky; this works out what
 * fraction of the sun's disc the moon is covering. Doing it as a smooth
 * fraction (rather than a yes/no "blocked?") is what gives the shadow a soft,
 * gradual edge instead of a hard line. */

/* Area of the lens-shaped sliver where two overlapping circles meet (radii
 * r1, r2, centre-to-centre distance d). The caller only calls this when the
 * circles genuinely partly overlap. */
static float circle_overlap_area(float d, float r1, float r2) {
  float r1s = r1 * r1, r2s = r2 * r2;
  float a1_arg = (d * d + r1s - r2s) / (2.0f * d * r1);
  float a2_arg = (d * d + r2s - r1s) / (2.0f * d * r2);
  if (a1_arg > 1.0f)
    a1_arg = 1.0f;
  if (a1_arg < -1.0f)
    a1_arg = -1.0f;
  if (a2_arg > 1.0f)
    a2_arg = 1.0f;
  if (a2_arg < -1.0f)
    a2_arg = -1.0f;
  float A1 = r1s * acosf(a1_arg);
  float A2 = r2s * acosf(a2_arg);

  float t1 = -d + r1 + r2;
  float t2 = d + r1 - r2;
  float t3 = d - r1 + r2;
  float t4 = d + r1 + r2;
  float prod = t1 * t2 * t3 * t4;
  if (prod < 0.0f)
    prod = 0.0f;
  float A3 = 0.5f * sqrtf(prod);

  return A1 + A2 - A3;
}

/* What fraction of the sun (0..1) is visible from point p: 1 if the moon is
 * out of the way, 0 if it fully covers it, something in between at the edge. */
static float visible_fraction_sun(V3 p, V3 sun_pos, V3 moon_pos, float moon_R) {
  V3 to_sun_v = v3_sub(sun_pos, p);
  V3 to_moon_v = v3_sub(moon_pos, p);
  float dist_sun = v3_len(to_sun_v);
  float dist_moon = v3_len(to_moon_v);
  if (dist_sun < SUN_R + 1e-3f)
    return 0.0f;
  if (dist_moon < moon_R + 1e-3f)
    return 0.0f;

  float a_sun = asinf(SUN_R / dist_sun);
  float a_moon = asinf(moon_R / dist_moon);

  V3 d_sun = v3_scl(to_sun_v, 1.0f / dist_sun);
  V3 d_moon = v3_scl(to_moon_v, 1.0f / dist_moon);
  float cos_g = v3_dot(d_sun, d_moon);
  if (cos_g > 1.0f)
    cos_g = 1.0f;
  if (cos_g < -1.0f)
    cos_g = -1.0f;
  float gamma = acosf(cos_g);

  if (gamma >= a_sun + a_moon)
    return 1.0f;
  if (gamma <= fabsf(a_sun - a_moon)) {
    if (a_moon >= a_sun)
      return 0.0f;
    float r = a_moon / a_sun;
    return 1.0f - r * r;
  }

  /* the edge case: how much of the sun's disc the moon's disc covers */
  float overlap = circle_overlap_area(gamma, a_sun, a_moon);
  float sun_a = (float)M_PI * a_sun * a_sun;
  if (sun_a < 1e-9f)
    return 1.0f;
  return clamp01(1.0f - overlap / sun_a);
}

/* §9 — the glowing gas around the sun, and how thick and bright it is at any
 * point. It's two parts: an even haze (the corona plus the thin red rim) that
 * only depends on distance from the sun, and a few lopsided arcs (the
 * prominences) on top. Keeping the even part separate is what lets §10
 * precompute its see-through table cheaply. */

/* The gas's state for this frame: the sun's temperature (which sets all the
 * colours), plus where the prominence arcs sit and how big they are. The
 * prominence values are rolled fresh on reseed. */
typedef struct {
  float kelvin;                  /* sun temperature; drives every colour */
  float prom_phi[PROM_COUNT];    /* where each arc sits around the rim */
  float prom_height[PROM_COUNT]; /* how far each reaches out */
  float prom_amp[PROM_COUNT];    /* how dense / bright each is */
} Medium;

/* How thick the even haze is at distance r from the sun's centre: the corona
 * (thinning with distance) plus the thin red rim. Zero inside the sun and far
 * out past the corona. */
static float sigma_sph(float r) {
  if (r < SUN_R)
    return 0.0f;
  if (r > SUN_R * CORONA_REACH)
    return 0.0f;

  float u = SUN_R / r;
  float cor = CORONA_SIGMA0 * powf(u, CORONA_DECAY);

  float h = (r - SUN_R) / (SUN_R * CHROMOS_THICK);
  float chrom = 0.0f;
  if (h >= 0.0f && h <= 1.0f) {
    float fade = 1.0f - h;
    chrom = CHROMOS_SIGMA0 * fade * fade;
  }
  return cor + chrom;
}

/* Extra thickness from the prominence arcs at a point. Each is a soft blob at
 * its own angle around the rim, fading out toward its own top. Added on top of
 * the even haze, so the arcs can stick out asymmetrically. */
static float sigma_prom(const Medium *m, V3 p_local) {
  float r = v3_len(p_local);
  if (r < SUN_R)
    return 0.0f;
  if (r > SUN_R * (1.0f + PROM_HEIGHT))
    return 0.0f;

  float phi = atan2f(p_local.x, p_local.z);
  float h_above = (r - SUN_R) / SUN_R;
  if (h_above > PROM_HEIGHT)
    return 0.0f;

  float total = 0.0f;
  for (int k = 0; k < PROM_COUNT; k++) {
    float reach_k = m->prom_height[k];
    if (h_above > reach_k)
      continue;
    float dphi = wrap_pi(phi - m->prom_phi[k]);
    float lat = expf(-(dphi * dphi) / (PROM_LATERAL * PROM_LATERAL));
    float t_rad = h_above / reach_k;
    float rad = (1.0f - t_rad) * (1.0f - t_rad);
    total += m->prom_amp[k] * lat * rad;
  }
  return PROM_SIGMA0 * total;
}

/* Total gas thickness at a point — the even haze plus any prominence arcs.
 * The marcher asks for this once per step. */
static float density_total(const Medium *m, V3 p_local) {
  float r = v3_len(p_local);
  return sigma_sph(r) + sigma_prom(m, p_local);
}

/* The light the gas gives off on its own at a point (the red rim and the
 * prominences — the corona doesn't glow itself, it only scatters sunlight).
 * The rim is textured with noise (the hairy spicules) and leans a touch yellow
 * where it's hot; the prominences stay deep red. */
static RGB emission_at(const Medium *m, V3 p_local) {
  float r = v3_len(p_local);
  if (r < SUN_R)
    return rgb_make(0, 0, 0);
  if (r > SUN_R * (1.0f + PROM_HEIGHT))
    return rgb_make(0, 0, 0);

  RGB ha_cool = rgb_make(1.00f, 0.18f, 0.12f);
  RGB ha_hot = rgb_make(1.00f, 0.45f, 0.18f);

  float chr_em = 0.0f;
  float spicule_n = 0.0f;
  float h = (r - SUN_R) / (SUN_R * CHROMOS_THICK);
  if (h >= 0.0f && h <= 1.0f) {
    float fade = 1.0f - h;
    fade = fade * fade * fade * fade;

    float r_safe = fmaxf(r, 1e-6f);
    float yy = p_local.y / r_safe;
    if (yy > 1.0f)
      yy = 1.0f;
    if (yy < -1.0f)
      yy = -1.0f;
    float theta = acosf(yy);
    float phi = atan2f(p_local.x, p_local.z);
    spicule_n = fbm2(phi * SPICULE_FREQ_PHI, theta * SPICULE_FREQ_TH);
    float spicule_factor = SPICULE_BASE + SPICULE_AMP * spicule_n;

    chr_em = CHROMOS_SIGMA0 * fade * spicule_factor;
  }

  float pr_em = sigma_prom(m, p_local);

  RGB chr_col = rgb_make(ha_cool.r + (ha_hot.r - ha_cool.r) * spicule_n,
                         ha_cool.g + (ha_hot.g - ha_cool.g) * spicule_n,
                         ha_cool.b + (ha_hot.b - ha_cool.b) * spicule_n);

  RGB out = rgb_scl(chr_col, chr_em);
  out = rgb_add(out, rgb_scl(ha_cool, pr_em));
  return out;
}

/* §10 — a speed trick. As a ray crosses the corona we keep needing "how much
 * of the sun's light makes it out to this point through the gas in the way?"
 * Working that out fresh every time would be far too slow. But because the
 * even haze only depends on distance from the sun, the answer only depends on
 * how far out the point is — so we compute it once at startup for a range of
 * distances and just look it up afterwards. */

static float trans_lut[LUT_SIZE]; /* how much the haze dims light, by distance from the sun */

/* Fill that table once at startup by adding up the haze from each distance
 * outward. (The haze never changes, so this never needs redoing.) */
static void build_trans_lut(void) {
  const int N_INT = 32;
  trans_lut[0] = 0.0f;
  float r_prev = SUN_R;
  for (int i = 1; i < LUT_SIZE; i++) {
    float r_curr =
        SUN_R + (LUT_R_MAX - SUN_R) * ((float)i / (float)(LUT_SIZE - 1));
    float ds = (r_curr - r_prev) / (float)N_INT;
    float tau = trans_lut[i - 1];
    for (int j = 0; j < N_INT; j++) {
      float s = r_prev + (j + 0.5f) * ds;
      tau += sigma_sph(s) * ds;
    }
    trans_lut[i] = tau;
    r_prev = r_curr;
  }
}

/* Look up that fraction (0..1) for a point: 0 inside the sun, 1 once you're
 * past all the gas, blended from the table in between. */
static float lookup_transmittance(V3 p_local) {
  float r = v3_len(p_local);
  if (r < SUN_R)
    return 0.0f;
  if (r > LUT_R_MAX)
    return 1.0f;

  float fr = (r - SUN_R) / (LUT_R_MAX - SUN_R) * (float)(LUT_SIZE - 1);
  int ir = (int)fr;
  if (ir < 0)
    ir = 0;
  if (ir > LUT_SIZE - 2)
    ir = LUT_SIZE - 2;
  float t = fr - (float)ir;
  float tau = trans_lut[ir] * (1.0f - t) + trans_lut[ir + 1] * t;
  return expf(-tau);
}

/* §11 — the heart of the program: follow one ray and work out the colour it
 * sees. Walk it forward in small steps; at each step add the sunlight the gas
 * scatters toward us plus the light the gas glows with, dimming as the gas
 * blocks more of it; then, if the ray finally lands on the sun or the moon,
 * add that surface's colour. A few small helpers below feed into it. */

/* How strongly a gas particle scatters light toward us versus other angles.
 * It doesn't favour any colour, which is exactly why the corona ends up the
 * same colour as the sun. */
static inline float thomson_phase(float cos_theta) {
  return (3.0f / (16.0f * (float)M_PI)) * (1.0f + cos_theta * cos_theta);
}

/* How big the sun looks in the sky from a point that far away — i.e. how much
 * of the sky it fills. Bigger when you're closer to it. */
static inline float sun_solid_angle(float dist_sun) {
  float sin_a = SUN_R / fmaxf(dist_sun, SUN_R + 1e-3f);
  if (sin_a > 1.0f)
    sin_a = 1.0f;
  float cos_a = sqrtf(fmaxf(1.0f - sin_a * sin_a, 0.0f));
  return 2.0f * (float)M_PI * (1.0f - cos_a);
}

/* The sun's surface colour where a ray hits it: bright, and a little dimmer
 * toward the edge (we look through more of its hazy air there). */
static RGB shade_sun_surface(V3 ro, V3 rd, V3 sun_pos, float t_sun,
                             RGB sun_em) {
  V3 p = v3_add(ro, v3_scl(rd, t_sun));
  V3 N = v3_norm(v3_sub(p, sun_pos));
  float mu = -v3_dot(N, rd);
  if (mu < 0.f)
    mu = 0.f;
  float lim = LIMB_AMBIENT + LIMB_GAIN * mu;
  return rgb_scl(sun_em, lim);
}

/* The moon's surface colour where a ray hits it. We're looking at its night
 * side (it's between us and the sun), so the sun barely touches it; what fills
 * the dark disc is the faint cool-blue glow bounced off the Earth. */
static RGB shade_moon_surface(V3 ro, V3 rd, V3 sun_pos, V3 moon_pos,
                              float t_moon, RGB sun_em, RGB earth_tint) {
  V3 p = v3_add(ro, v3_scl(rd, t_moon));
  V3 N = v3_norm(v3_sub(p, moon_pos));
  V3 to_sun_v = v3_sub(sun_pos, p);
  float dist_sun = v3_len(to_sun_v);
  V3 to_sun = v3_scl(to_sun_v, 1.0f / fmaxf(dist_sun, 1e-6f));
  float cos_sn = v3_dot(N, to_sun);

  RGB direct = rgb_make(0.f, 0.f, 0.f);
  if (cos_sn > 0.0f) {
    float Lf = MOON_ALBEDO / (float)M_PI * cos_sn;
    direct = rgb_scl(sun_em, Lf);
  }
  RGB earth = rgb_scl(earth_tint, EARTHSHINE_GAIN);
  return rgb_add(direct, earth);
}

/* Walk a ray through the gas from the eye out to t_max, adding up the sunlight
 * the gas scatters our way plus the light it glows with, and dimming as it
 * goes. `transmittance` comes in as how much light still survives and is left
 * at the fraction reaching t_max, so the caller can dim the surface behind it. */
static RGB march_medium(V3 ro, V3 rd, const Medium *m, V3 sun_pos, V3 moon_pos,
                        float moon_R, float t_max, RGB sun_em,
                        float corona_gate, float *transmittance) {
  /* Step big through empty space, smaller through the corona. Out past the
   * corona the gas is nothing, so big strides there cost nothing. */
  float dt_fine = t_max / (float)MARCH_STEPS;
  float dt_coarse = dt_fine * MARCH_COARSE_MULT;
  float corona_outer = SUN_R * (CORONA_REACH + CORONA_FINE_MARGIN);
  float corona_outer_R2 = corona_outer * corona_outer;
  /* The chromosphere shell and prominences are far thinner than one fine
   * step, so close to the photosphere we step ultra-fine — otherwise the eye
   * ray strides right over them and the red ring + prominences come out dotty
   * and flickery. The band is wide enough that any step able to reach the
   * shell already starts inside it (so it's ultra-fine and can't skip it). */
  float detail_outer = SUN_R * MARCH_DETAIL_REACH;
  float detail_outer_R2 = detail_outer * detail_outer;

  RGB radiance_accum = rgb_make(0.f, 0.f, 0.f);
  float t_along_ray = 0.0f;

  while (t_along_ray < t_max && *transmittance > TRANSMITTANCE_CUTOFF) {
    /* Step size from distance to sun at the start of the step: ultra-fine in
     * the near-surface detail band, fine through the corona, coarse outside. */
    V3 p_now = v3_add(ro, v3_scl(rd, t_along_ray));
    V3 p_now_loc = v3_sub(p_now, sun_pos);
    float r2_now = v3_dot(p_now_loc, p_now_loc);
    float dt = (r2_now < detail_outer_R2)   ? MARCH_DETAIL_DT
               : (r2_now < corona_outer_R2) ? dt_fine
                                            : dt_coarse;
    if (t_along_ray + dt > t_max)
      dt = t_max - t_along_ray;

    /* how much gas is here, sampled at the middle of the step */
    float t_mid = t_along_ray + 0.5f * dt;
    V3 p = v3_add(ro, v3_scl(rd, t_mid));
    V3 p_local = v3_sub(p, sun_pos);
    float density = density_total(m, p_local);

    if (density >= DENSITY_EPS) {
      V3 to_sun_v = v3_sub(sun_pos, p);
      float dist_sun = v3_len(to_sun_v);
      V3 to_sun = v3_scl(to_sun_v, 1.0f / fmaxf(dist_sun, 1e-6f));

      /* sunlight this bit of gas bounces toward us — but only as much sun as
       * it can still see past the moon (that's what softens the shadow edge) */
      float vis = visible_fraction_sun(p, sun_pos, moon_pos, moon_R);
      if (vis > VIS_EPS) {
        float omega = sun_solid_angle(dist_sun);
        float tr_to_sun = lookup_transmittance(p_local);
        float cos_th = v3_dot(rd, to_sun);
        float phase = thomson_phase(cos_th);
        float in_sc = density * phase * tr_to_sun * omega * vis *
                      IN_SCATTER_GAIN * dt * corona_gate;
        radiance_accum = rgb_add(
            radiance_accum, rgb_scl(rgb_scl(sun_em, in_sc), *transmittance));
      }

      /* light this bit of gas glows with on its own (red rim + prominences) */
      RGB em = emission_at(m, p_local);
      if (em.r + em.g + em.b > 0.f) {
        radiance_accum = rgb_add(
            radiance_accum,
            rgb_scl(em, *transmittance * dt * HA_EMIT_GAIN * corona_gate));
      }

      /* dim everything behind this step by how much gas it just passed through */
      *transmittance *= expf(-density * dt);
    }
    t_along_ray += dt;
  }
  return radiance_accum;
}

/* The colour one ray sees — the whole job in one place. Find whether it hits
 * the sun, the moon, or nothing; walk the gas up to that point adding glow and
 * scattered light; then add the surface's colour, dimmed by whatever gas was
 * in front of it. Everything else in the file just feeds this. */
static RGB trace_ray(V3 ro, V3 rd, const Medium *m, V3 sun_pos, V3 moon_pos,
                     float moon_R, int seed, RGB sun_em, RGB earth_tint,
                     float corona_gate) {
  /* what does the ray hit? (the moon uses its bumpy-edge test, for beads) */
  float t_sun = 0.f, t_moon = 0.f;
  bool hit_sun = ray_sphere(ro, rd, sun_pos, SUN_R, &t_sun);
  bool hit_moon = ray_moon(ro, rd, moon_pos, moon_R, seed, &t_moon);

  /* keep the nearer hit; if it hits nothing, march out to the sky distance */
  float t_max;
  int surf_type = 0; /* 0 = sky, 1 = sun, 2 = moon */
  if (hit_moon && (!hit_sun || t_moon < t_sun)) {
    t_max = t_moon;
    surf_type = 2;
  } else if (hit_sun) {
    t_max = t_sun;
    surf_type = 1;
  } else {
    t_max = MARCH_T_MAX;
    surf_type = 0;
  }

  /* walk the gas; transmittance comes back as how much light still gets
   * through to the surface behind it */
  float transmittance = 1.0f;
  RGB radiance_accum = march_medium(ro, rd, m, sun_pos, moon_pos, moon_R, t_max,
                                    sun_em, corona_gate, &transmittance);

  /* add the hit surface's colour, dimmed by the gas in front of it */
  if (surf_type == 1) {
    RGB surf = shade_sun_surface(ro, rd, sun_pos, t_sun, sun_em);
    radiance_accum = rgb_add(radiance_accum, rgb_scl(surf, transmittance));
  } else if (surf_type == 2) {
    RGB surf = shade_moon_surface(ro, rd, sun_pos, moon_pos, t_moon, sun_em,
                                  earth_tint);
    radiance_accum = rgb_add(radiance_accum, rgb_scl(surf, transmittance));
  }
  /* sky: nothing to add */

  return radiance_accum;
}

/* §12 — the scene: everything we're showing and how the user is driving it,
 * plus the clock that moves the moon. The pieces here play different roles,
 * marked per group: the structs hold the state; pattern_set / reseed / init
 * change it on a keypress; scene_tick is the one thing the clock advances; and
 * the scene_* readers just work out facts from it without changing anything. */

/* The geometry of the current kind of eclipse: how big the moon is and whether
 * it rides off-centre, plus a flag or two for the HUD. */
typedef struct {
  float moon_world_r;     /* moon radius (sets how much of the sun it covers) */
  float moon_y_world;     /* vertical offset (PARTIAL rides high) */
  bool totality_possible; /* can this one go fully total? (for the HUD label) */
  bool show_corona;       /* unused; kept for the HUD */
} PatternParams;

/* Everything the program needs to draw and run the eclipse, in one place: which
 * eclipse and the random seeds that flavour it, the clock the moon rides on,
 * and the settings the user changes with the keys. Only init / reseed / tick
 * rewrite this whole thing — the drawing code just reads it. */
typedef struct {
  /* which eclipse, and its random identity */
  Pattern current_pattern; /* TOTAL / PARTIAL / ANNULAR / TRANSIT */
  PatternParams pp;        /* the moon geometry for it */
  Medium medium;           /* the corona + prominence gas */
  int seed;                /* random seed for the gas and the moon's edge */
  float seed_phase;        /* where in its crossing the moon starts out */

  /* the clock */
  float time_secs; /* how long it's been running; the moon rides on this */

  /* what the user changes with the keys */
  bool paused;      /* freeze the motion */
  int speed;        /* how fast the moon moves */
  int star_idx;     /* which star → the sun's colour */
  float zoom;       /* how zoomed in */
  float moon_scale; /* live moon-size tweak */
} Scene;

/* Set the moon geometry for a kind of eclipse (on the n / p keys). We don't
 * special-case the patterns anywhere else — we just resize/move the moon here
 * and the same renderer produces all four looks. */
static void pattern_set(Scene *s, Pattern p) {
  s->current_pattern = p;
  PatternParams *pp = &s->pp;
  pp->moon_world_r = MOON_BASE_R_TOTAL;
  pp->moon_y_world = 0.0f;
  pp->totality_possible = false;
  pp->show_corona = true;
  switch (p) {
  case PATTERN_TOTAL:
    pp->moon_world_r = MOON_BASE_R_TOTAL;
    pp->totality_possible = true;
    break;
  case PATTERN_PARTIAL:
    pp->moon_world_r = MOON_BASE_R_TOTAL;
    pp->moon_y_world = PARTIAL_Y_OFFSET * MOON_Z;
    break;
  case PATTERN_ANNULAR:
    pp->moon_world_r = MOON_BASE_R_ANNULAR;
    break;
  case PATTERN_TRANSIT:
    pp->moon_world_r = MOON_BASE_R_TRANSIT;
    break;
  case N_PATTERNS:
    break;
  }
}

/* Roll a fresh look for the gas: new prominence arcs, a new moon edge, and a
 * new rim texture. Run on 'r' and at startup. */
static void medium_reseed(Medium *m, int seed) {
  /* scatter the prominence arcs (angle, reach, brightness) */
  for (int k = 0; k < PROM_COUNT; k++) {
    uint32_t h = hash3(k, seed, 0xF1A3E);
    m->prom_phi[k] = hash01(h) * 2.0f * (float)M_PI;
    m->prom_height[k] = PROM_HEIGHT * (0.40f + 0.60f * hash01(h ^ 0xA1u));
    m->prom_amp[k] = 0.50f + 0.50f * hash01(h ^ 0xB2u);
  }
  build_lunar_lut(seed); /* re-dent the moon's edge */
  perm_shuffle(seed);    /* re-shuffle the rim texture noise */
}

/* pick a new random seed and moon start-position, then reseed the gas */
static void scene_reseed(Scene *s) {
  uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                     (int)(s->seed_phase * 100.0f), 0xC0FFEE);
  s->seed_phase = ((float)(h & 0xFFFFu) / 65536.0f) * 2.0f * (float)M_PI;
  s->seed = (int)(h ^ 0x5A5A5A5Au);
  medium_reseed(&s->medium, s->seed);
}

/* start fresh at launch: a total eclipse of the Sun, sensible defaults */
static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->star_idx = 2; /* G-STAR (the Sun) by default */
  s->zoom = 1.0f;
  s->moon_scale = 1.0f;
  s->seed_phase = 0.0f;
  s->seed = 0xDECAF;
  medium_reseed(&s->medium, s->seed);
  pattern_set(s, PATTERN_TOTAL);
}

/* The one and only place the clock moves forward. Does nothing while paused. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_DEF;
  s->time_secs += dt * speed_mul;
}

/* §12 readers — work out the moon's current size, where it is, and how much
 * of the sun it covers from the camera. Pure: they read the scene, change
 * nothing. (The HUD uses these; the renderer uses §8 instead, which gives a
 * smooth edge.) */

/* the moon's actual size right now: its pattern size times the user's tweak */
static inline float scene_moon_r(const Scene *s) {
  return s->pp.moon_world_r * s->moon_scale;
}

/* where the moon is right now — it slides side to side on a gentle sine curve */
static V3 scene_moon_pos(const Scene *s) {
  float sun_a = atanf(SUN_R / SUN_Z);
  float moon_a = atanf(scene_moon_r(s) / MOON_Z);
  float orbit_x_world = MOON_Z * (sun_a + moon_a) * MOON_ORBIT_X_FRAC;
  float omega = 2.0f * (float)M_PI / ECLIPSE_PERIOD_S;
  float phase = omega * s->time_secs + s->seed_phase;
  float mx = orbit_x_world * sinf(phase);
  return v3(mx, s->pp.moon_y_world, MOON_Z);
}

/* how much of the sun the moon is covering right now (0..1), from the camera —
 * drives the HUD progress bar and decides when the background stars come out */
static float scene_occlusion(const Scene *s) {
  V3 sun_pos = v3(0, 0, SUN_Z);
  V3 moon_pos = scene_moon_pos(s);
  V3 sun_dir = v3_norm(sun_pos);
  V3 moon_dir = v3_norm(moon_pos);
  float cos_b = v3_dot(sun_dir, moon_dir);
  if (cos_b > 1.f)
    cos_b = 1.f;
  if (cos_b < -1.f)
    cos_b = -1.f;
  float sep = acosf(cos_b);
  float sun_a = atanf(SUN_R / SUN_Z);
  float moon_a = atanf(scene_moon_r(s) / MOON_Z);
  float sum = sun_a + moon_a;
  float dif = fabsf(sun_a - moon_a);
  if (sep >= sum)
    return 0.f;
  if (sep <= dif) {
    if (moon_a >= sun_a)
      return 1.f;
    return (moon_a / sun_a) * (moon_a / sun_a);
  }
  float t = (sum - sep) / (2.0f * fminf(sun_a, moon_a));
  if (moon_a >= sun_a)
    return t;
  return t * (moon_a / sun_a) * (moon_a / sun_a);
}

/* §13 — the camera: turns a screen cell into the ray that shoots out through
 * it. A simple pinhole — where the eye sits, how wide the view is, and the
 * picture size. */

typedef struct {
  V3 pos;             /* eye position */
  float fov_h, fov_v; /* how wide the view is, across and up/down */
  int cols, rows;     /* picture size in cells */
} Camera;

static void camera_make(Camera *c, int cols, int rows, float fov_h) {
  c->cols = cols;
  c->rows = rows;
  c->pos = v3(0, 0, 0);
  c->fov_h = fov_h;
  c->fov_v = fov_h * (float)rows * ASPECT_Y / (float)cols;
}

/* the ray through screen cell (sx, sy). The up/down part is flipped because
 * screen rows count downward while the world's "up" points up. */
static V3 camera_ray(const Camera *c, int sx, int sy) {
  float u =
      ((2.0f * (float)sx + 1.0f) - (float)c->cols) / (float)c->cols * c->fov_h;
  float v =
      -((2.0f * (float)sy + 1.0f) - (float)c->rows) / (float)c->rows * c->fov_v;
  return v3_norm(v3(u, v, 1.0f));
}

/* §14 — drawing a frame. We trace a ray for every cell into an off-screen
 * image, then copy that image to the terminal. Also sets up the terminal
 * window and re-measures it on resize. */

/* The terminal we're drawing to — just its current size, re-read whenever the
 * window changes so the picture and HUD always fit. */
typedef struct {
  int cols, rows; /* terminal size in cells */
} Screen;

/* put the terminal into the mode we need: no echo, keys read immediately and
 * without blocking, cursor hidden, colours ready */
static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* on a window resize, tear down and re-measure so we match the new size */
static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* the off-screen image we draw into, then copy to the terminal */
static RGB g_buf[BUF_MAX_H][BUF_MAX_W];

/* a star's colour at cell (c, r), or black if this cell happens not to get
 * one. The same cells get stars each frame (it's hashed from position), and
 * each star gets a random temperature for a warm-to-cool colour spread. */
static RGB background_star(int c, int r, int seed) {
  uint32_t h = hash3(c, r, seed);
  if ((h % STAR_DENSITY) != 0u)
    return rgb_make(0.f, 0.f, 0.f);
  float br = STAR_BRIGHT_MIN + hash01(h >> 8) * STAR_BRIGHT_RANGE;
  float u_K = hash01(h ^ 0xC0DEC0DEu);
  float kK = STAR_K_MIN * powf(STAR_K_MAX / STAR_K_MIN, u_K);
  return rgb_scl(blackbody_rgb(kK), br);
}

/* Draw one frame: set up the camera, the sun's colour, and the per-frame
 * lighting, then trace a ray for every cell and copy the result to the screen.
 * Keeping all this here lets trace_ray stay a pure "one ray → one colour". */
static void scene_draw(const Screen *sc, const Scene *s) {
  /* leave the top and bottom rows for the HUD (unless the window is tiny) */
  int rows_eff = sc->rows - 2;
  int row_off = 1;
  if (rows_eff < 4) {
    rows_eff = sc->rows;
    row_off = 0;
  }
  if (rows_eff > BUF_MAX_H)
    rows_eff = BUF_MAX_H;
  int cols_eff = sc->cols;
  if (cols_eff > BUF_MAX_W)
    cols_eff = BUF_MAX_W;

  Camera cam;
  camera_make(&cam, cols_eff, rows_eff, FOV_H_BASE / s->zoom);

  V3 sun_pos = v3(0, 0, SUN_Z);
  V3 moon_pos = scene_moon_pos(s);
  float moon_R = scene_moon_r(s);

  Medium m_local = s->medium;
  m_local.kelvin = STARS[s->star_idx].kelvin;

  RGB sun_chrom = blackbody_rgb(m_local.kelvin);
  RGB sun_em = rgb_scl(sun_chrom, SUN_EMIT_HDR);

  /* the cool-blue earthshine colour — the sun's light tinted as if bounced
   * off the blue Earth */
  RGB earth_alb = rgb_make(0.30f, 0.45f, EARTH_ALBEDO_BLUE);
  RGB earth_tint = rgb_mul(sun_em, earth_alb);

  /* eye adaptation: how strong the corona shows this frame (0 with the sun in
   * view, rising to full as it's covered) */
  float cam_vis_sun = visible_fraction_sun(cam.pos, sun_pos, moon_pos, moon_R);
  float corona_gate =
      CORONA_GATE_FLOOR + CORONA_GATE_RANGE * (1.0f - cam_vis_sun);

  /* are we near totality? a per-frame fact, so work it out once rather than
   * for every dim cell below */
  bool stars_visible = scene_occlusion(s) > STAR_OCC_VISIBLE;

  /* trace a ray for every cell into the off-screen image */
  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      V3 ray_d = camera_ray(&cam, c, r);
      RGB col = trace_ray(cam.pos, ray_d, &m_local, sun_pos, moon_pos, moon_R,
                          s->seed, sun_em, earth_tint, corona_gate);

      /* sprinkle stars on the dark sky cells once we're near totality */
      if (stars_visible && luma_of(col) < STAR_LUMA_MAX)
        col = rgb_add(col, background_star(c, r, s->seed));

      g_buf[r][c] = col;
    }
  }

  /* copy the image to the terminal */
  for (int r = 0; r < rows_eff; r++) {
    for (int c = 0; c < cols_eff; c++) {
      paint_cell(c, r + row_off, g_buf[r][c]);
    }
  }
}

/* §15 — the text overlay: a status line and a "TOTALITY" flash along the top,
 * and the key hints along the bottom. */

static void hud_draw(const Screen *sc, const Scene *s, double fps,
                     int sim_fps) {
  float occ = scene_occlusion(s);

  int bar_w = 10;
  int bar_fill = (int)(occ * (float)bar_w + 0.5f);
  if (bar_fill > bar_w)
    bar_fill = bar_w;
  if (bar_fill < 0)
    bar_fill = 0;
  char bar[16] = {0};
  for (int i = 0; i < bar_w; i++)
    bar[i] = (i < bar_fill) ? '#' : '.';

  const Star *st = &STARS[s->star_idx];
  float sun_a = atanf(SUN_R / SUN_Z) * 180.f / (float)M_PI;
  float moon_a = atanf(scene_moon_r(s) / MOON_Z) * 180.f / (float)M_PI;
  char buf[300];
  snprintf(
      buf, sizeof buf,
      " %5.1f fps %3d Hz  %s  %s %5.0fK  s:%4.2f m:%4.2f deg  [%s] %3.0f%%  "
      "z:%3.1fx mz:%3.1fx  spd:%d ",
      fps, sim_fps, s->paused ? "PAUSED " : pattern_name(s->current_pattern),
      st->name, (double)st->kelvin, (double)sun_a, (double)moon_a, bar,
      (double)(occ * 100.0f), (double)s->zoom, (double)s->moon_scale, s->speed);
  int len = (int)strlen(buf);
  if (len > sc->cols)
    len = sc->cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sc->cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " SOLAR-ECLIPSE-PT · PATH-TRACED ");
  attroff(COLOR_PAIR(PAIR_HUD));

  /* Totality event flash. */
  if (s->pp.totality_possible && occ > 0.99f) {
    const char *lab = " [ TOTALITY ] ";
    int elen = (int)strlen(lab);
    attron(COLOR_PAIR(PAIR_EVENT_HOT) | A_BOLD);
    mvprintw(0, sc->cols - len - elen - 1, "%s", lab);
    attroff(COLOR_PAIR(PAIR_EVENT_HOT) | A_BOLD);
  }

  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(sc->rows - 1, 0,
           " q:quit  spc:pause  n/p:pat  t/T:star  z/Z:zoom  m/M:moon  +/-:spd "
           " []:Hz  r:reseed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  scene_draw(sc, s);
  hud_draw(sc, s, fps, sim_fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* §16 — the program itself: keyboard, signals, and the main loop that ties it
 * all together. */

/* The whole running program in one place: the scene, the terminal we draw to,
 * how fast the world updates, and two flags the signal handlers flip. */
typedef struct {
  Scene scene;   /* the eclipse being shown */
  Screen screen; /* the terminal we draw to */
  int sim_fps;   /* how many times a second the world updates */
  /* These two are touched by signal handlers, so they need the special
   * "safe to poke from a signal" type. */
  volatile sig_atomic_t running;     /* set to 0 to quit */
  volatile sig_atomic_t need_resize; /* set when the window was resized */
} App;

static App g_app;

/* signal handlers just flip a flag and return; the loop notices next time round */

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/* Act on one keypress. Returns false only for quit, which ends the program. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reseed(s);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
  case '_':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  case 't':
    s->star_idx = (s->star_idx + 1) % N_STARS;
    break;
  case 'T':
    s->star_idx = (s->star_idx + N_STARS - 1) % N_STARS;
    break;

  case 'z':
    s->zoom *= ZOOM_STEP;
    if (s->zoom > ZOOM_MAX)
      s->zoom = ZOOM_MAX;
    break;
  case 'Z':
    s->zoom /= ZOOM_STEP;
    if (s->zoom < ZOOM_MIN)
      s->zoom = ZOOM_MIN;
    break;

  case 'm':
    s->moon_scale /= MOON_SCALE_STEP;
    if (s->moon_scale < MOON_SCALE_MIN)
      s->moon_scale = MOON_SCALE_MIN;
    break;
  case 'M':
    s->moon_scale *= MOON_SCALE_STEP;
    if (s->moon_scale > MOON_SCALE_MAX)
      s->moon_scale = MOON_SCALE_MAX;
    break;

  case 'n':
  case 'N':
    pattern_set(s, (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS));
    break;
  case 'p':
  case 'P':
    pattern_set(
        s, (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS));
    break;
  }
  return true;
}

/* wire up the OS signals: Ctrl-C / kill ask us to quit, a window resize sets a
 * flag, and we always put the terminal back on the way out */
static void install_signals(void) {
  struct sigaction sa = {0};
  sa.sa_handler = on_exit_signal;
  sigaction(SIGINT, &sa, NULL);
  sigaction(SIGTERM, &sa, NULL);
  sa.sa_handler = on_resize_signal;
  sigaction(SIGWINCH, &sa, NULL);
  atexit(cleanup);
}

static void app_init(App *app) {
  memset(app, 0, sizeof *app);
  scene_init(&app->scene);
  screen_init(&app->screen);
  app->sim_fps = SIM_FPS_DEFAULT;
  app->running = 1;
  build_trans_lut();
}

static void app_run(App *app) {
  int64_t prev = clock_ns();
  int64_t sim_accum = 0;
  int64_t frame_count = 0;
  int64_t fps_window_start = prev;
  double fps_meas = 0.0;

  install_signals();

  while (app->running) {
    /* handle any keys waiting (these change settings or quit — not the clock) */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }
    if (!app->running)
      break;
    /* apply a pending window resize */
    if (app->need_resize) {
      screen_resize(&app->screen);
      app->need_resize = 0;
    }

    /* how much real time passed, capped so a long stall doesn't lurch ahead */
    int64_t now = clock_ns();
    int64_t dt = now - prev;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;
    prev = now;

    /* advance the clock in fixed-size steps, so the motion runs at the same
     * speed whatever the frame rate */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, (float)tick_ns / (float)NS_PER_SEC);
      sim_accum -= tick_ns;
    }

    /* draw the frame and flip it to the screen */
    screen_draw(&app->screen, &app->scene, fps_meas, app->sim_fps);
    screen_present();

    /* refresh the fps number once a second */
    frame_count++;
    if (now - fps_window_start >= NS_PER_SEC) {
      fps_meas = (double)frame_count * (double)NS_PER_SEC /
                 (double)(now - fps_window_start);
      frame_count = 0;
      fps_window_start = now;
    }

    /* sleep off the rest of the frame's time budget so we don't run flat out */
    int64_t target = clock_ns();
    int64_t left = TICK_NS(SIM_FPS_DEFAULT * 2) - (target - now);
    if (left > 0)
      clock_sleep_ns(left);
  }
}

int main(void) {
  app_init(&g_app);
  app_run(&g_app);
  cleanup();
  return 0;
}

