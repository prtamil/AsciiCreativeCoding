/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */

/* saturn_with_rings.c — a ringed planet drawn into the terminal in colour.
 * For every character cell we shoot one ray; it either hits the planet (a
 * sphere), the rings (a flat disc), or empty space, and we colour the cell
 * to match. The rest is lighting and a few different worlds to look at.
 * Sister files: sphere_raytrace.c (the ray-vs-sphere math up close),
 * path_tracer.c (the colour-to-terminal pipeline). Keys are shown along
 * the bottom of the screen while it runs. */

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

/* §1 — config: every tunable number in one place, so none of them show up
 * as a mystery value buried in the code below. */

/* How fast time moves. The world updates on a steady clock (ticks/sec);
 * the speed setting just multiplies how fast the sun travels. */
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
#define DT_CAP_NS (100 * NS_PER_MS) /* if a frame stalls longer than this, pretend it didn't */
#define RENDER_FPS_CAP 60 /* don't redraw faster than 60 times a second */
#define FPS_SAMPLE_MS 500 /* how often the fps number on screen refreshes */

/* The camera lens. Terminal cells are about twice as tall as they are wide,
 * so we squash the view vertically to keep the planet round, not egg-shaped. */
#define ASPECT_Y 2.0f
#define FOV_H 0.55f /* how wide the view is; bigger = more zoomed out */

/* Sizes of things, measured so the planet's radius is exactly 1. */
#define PLANET_RADIUS 1.00f
#define RING_R_IN_DEFAULT 1.45f  /* rings begin this far from the centre... */
#define RING_R_OUT_DEFAULT 2.55f /* ...and end here */
#define CASSINI_R_DEFAULT 2.10f  /* the famous dark gap in Saturn's rings sits here */
#define CASSINI_W_DEFAULT 0.06f  /* and is about this wide */
#define CASSINI_DIM 0.18f        /* how dark the gap goes (0 would be black) */
#define RING_BAND_FREQ 18.0f     /* how many light/dark bands run across the rings */

#define CAM_DIST_DEFAULT 4.7f    /* how far back the camera sits */
#define CAM_HEIGHT_DEFAULT 1.10f /* how high it floats above the rings */

/* The sun slowly circles the scene, and everything is lit by it. */
#define ROTATION_PERIOD_S 30.0f /* seconds for the sun to go all the way round */
#define SUN_ELEV_Y 0.45f        /* how high the sun rides; higher lights the rings more */
#define SUN_PHASE0 -0.70f       /* where the sun starts out: front and a bit to the side,
                                   so the planet's lit face is the one we see at startup
                                   (pressing reseed scrambles this) */

/* How strong each kind of light is. The planet gets a warm main light (the
 * sun), a cool soft fill so its dark side isn't pure black, a shiny highlight,
 * and a glowing edge where the atmosphere catches the light. */
#define AMBIENT_K 0.18f /* faint glow on everything, so nothing is fully black */
#define FILL_K 0.40f    /* strength of the cool fill light */
/* The fill light shines from a fixed direction — up and toward the camera,
 * like soft daylight from the sky. We pin it here on purpose. If it came from
 * straight opposite the sun it would light up the far night side and make the
 * planet look flat and weirdly two-faced. */
#define FILL_DIR_X 0.00f
#define FILL_DIR_Y 0.70f
#define FILL_DIR_Z -0.70f
#define SPEC_SHININESS 24.0f /* bigger = a tighter, sharper shiny spot */
#define SPEC_K 0.40f         /* how strong that shiny spot is */
#define LIMB_K 0.50f         /* how much the planet darkens toward its edge */
#define RIM_WIDTH 0.28f      /* how thick the glowing edge is */
#define RIM_STRENGTH 0.85f   /* how bright the glowing edge is */
#define CONTINENT_LAND_THRESH 0.55f /* on the Earth world: above this is land, below is sea */
#define GLOW_SHARP 3.0f             /* how focused the lit-from-behind ring glow is */
#define GLOW_GAIN 1.20f             /* how bright that glow gets */

/* Soft shadow of the planet falling on the rings. We treat the sun as a
 * small disc rather than a single point, so the shadow's edge comes out
 * soft and fuzzy like a real one instead of razor-sharp. */
#define SOFT_SHADOW_SAMPLES 8    /* shadow rays per cell; more = smoother but slower */
#define SUN_ANGULAR_RADIUS 0.05f /* how big the sun looks in the sky (about 3 degrees) */

/* Anti-aliasing: how many slightly-nudged rays we average per cell to smooth
 * out jagged edges. 1 is fastest and roughest, 4 is slowest and smoothest. */
enum { SPP_MIN_VAL = 1, SPP_MAX_VAL = 4, SPP_DEF_VAL = 2 };

/* The three ways to view the scene, switched with the 'm' key. Two of them
 * are debugging views for when something looks wrong:
 *   LIT    — the real, fully-lit picture. The default.
 *   FLAT   — just the raw surface colours, with no lighting at all. Lets you
 *            check the bands and continents on their own.
 *   NORMAL — colours each spot by which way the surface faces. A good sphere
 *            shows a smooth colour gradient; the rings show a rainbow ring.
 *            A fast "is the shape right?" check. */
typedef enum {
  SHADE_LIT = 0,
  SHADE_FLAT = 1,
  SHADE_NORMAL = 2,
  SHADE_N = 3, /* how many modes exist, so 'm' can wrap back to the start */
} ShadeMode;

static const char *shade_mode_name(ShadeMode m) {
  switch (m) {
  case SHADE_LIT:
    return "LIT   ";
  case SHADE_FLAT:
    return "FLAT  ";
  case SHADE_NORMAL:
    return "NORMAL";
  default:
    return "?     ";
  }
}

/* Background stars. */
#define STAR_DENSITY 180     /* roughly one cell in this many gets a star */
#define STAR_TWINKLE_HZ 0.4f /* how fast they twinkle */

/* Colour slots we hand to ncurses. We pack a 6×6×6 grid of 216 colours into
 * consecutive slots starting at PAIR_CUBE_BASE; the low slots are the HUD. */
enum {
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_FLASH = 3,
  PAIR_CUBE_BASE = 8,
};

/* The characters we draw with, from faintest (space) to densest (@). A
 * brighter cell picks a denser character, so brightness still reads even
 * with no colour. This particular ordering is Paul Bourke's ASCII ramp. */
static const char k_ramp[] =
    " `.-':_,^=;><+!rc*/"
    "z?sLTv)J7(|Fi{C}fI31tlu[neoZ5Yxjya]2ESwqkP6h9d4VpOGbUAKXHm8RD#$Bg0MNWQ%&@";
#define RAMP_LEN ((int)(sizeof k_ramp - 1))

/* Turning a colour into a terminal cell. We snap each colour onto the 6×6×6
 * grid, and judge brightness with the standard TV weights (the eye sees green
 * as much brighter than blue). */
#define CUBE_SIDE 6      /* the colour grid is 6 levels per channel */
#define LUMA_W_R 0.2126f /* how much each channel counts toward brightness */
#define LUMA_W_G 0.7152f /* (these add up to 1) */
#define LUMA_W_B 0.0722f
#define LUMA_BOLD_THRESH 0.85f /* brighter than this draws bold */
#define LUMA_DIM_THRESH 0.15f  /* darker than this draws dim */

/* §2 — keeping time: a steady nanosecond clock and a plain sleep, used to
 * pace the loop. The clock only ever moves forward, so timing never glitches
 * if the system clock gets adjusted. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

/* §3 — small 3-D vector maths. The same three-number type doubles as a
 * colour, because colours add and scale exactly the way vectors do. */

/* A point or a direction in 3-D space — or a colour, when we read x/y/z as
 * red/green/blue. It's tiny, so we pass it around by value and keep the
 * helpers below simple and fast inside the per-pixel loop. */
typedef struct {
  float x, y, z;
} V3;

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }
static inline V3 v3_add(V3 a, V3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline V3 v3_sub(V3 a, V3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline V3 v3_scl(V3 a, float s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline V3 v3_mul(V3 a, V3 b) {
  return v3(a.x * b.x, a.y * b.y, a.z * b.z);
}
static inline float v3_dot(V3 a, V3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline V3 v3_cross(V3 a, V3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline V3 v3_norm(V3 a) {
  float l = sqrtf(v3_dot(a, a));
  if (l < 1e-12f)
    return v3(0, 0, 0);
  return v3_scl(a, 1.f / l);
}
static inline V3 v3_lerp(V3 a, V3 b, float t) {
  return v3_add(v3_scl(a, 1.f - t), v3_scl(b, t));
}
static inline V3 v3_max0(V3 a) {
  return v3(a.x < 0 ? 0 : a.x, a.y < 0 ? 0 : a.y, a.z < 0 ? 0 : a.z);
}

static inline float clamp01(float x) {
  if (x < 0.f)
    return 0.f;
  if (x > 1.f)
    return 1.f;
  return x;
}

/* gently squash bright values into 0..1 so nothing blows out to pure white */
static inline float reinhard(float x) { return x / (1.f + x); }
/* nudge brightness to match how a screen actually shows it */
static inline float gamma_enc(float x) { return powf(clamp01(x), 1.f / 2.2f); }

/* a soft 0-to-1 ramp between e0 and e1 — eases in and out instead of jumping */
static inline float smoothstep(float e0, float e1, float x) {
  float t = clamp01((x - e0) / (e1 - e0));
  return t * t * (3.f - 2.f * t);
}

/* §4 — noise: natural-looking randomness (blotches, coastlines) that's smooth
 * rather than static-y. Perlin noise is the classic recipe; fBm just layers a
 * few sizes of it for more detail. Drives the Earth world's continents and the
 * random "exo" planets. The maths is standard Perlin — look it up if curious;
 * here it's just a black box that hands back smooth random values. */

static uint8_t perm[512]; /* shuffled lookup table the noise reads from */

/* reshuffle the noise table so reseeding ('r') produces a different world */
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

/* layer 4 sizes of Perlin noise for richer detail; returns roughly 0..1 */
static float fbm2(float x, float y) {
  float total = 0, amp = 1, freq = 1, max_amp = 0;
  for (int o = 0; o < 4; o++) {
    total += amp * perlin2d(x * freq, y * freq);
    max_amp += amp;
    amp *= 0.5f;
    freq *= 2.0f;
  }
  return (total / max_amp) * 0.5f + 0.5f;
}

/* scramble three integers into one number — used to place stars and to give
 * each cell its own repeatable bit of randomness */
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

/* §5 — themes: the set of colours for each world, switched with t / T. */

/* One world's colour scheme. The shader does all its work in full colour and
 * only snaps to the limited terminal palette at the very end, so these can be
 * any colours we like. The four light colours follow the classic photo-studio
 * setup: a warm main light, a cool fill to soften the shadows, plus a couple
 * of accent tints. land_col / sea_col are used only by the Earth world. */
typedef struct {
  const char *name;
  V3 planet_base;      /* the planet's main colour */
  V3 planet_band_tint; /* colour blended into its light/dark bands */
  V3 ring_base;        /* the rings' main colour */
  V3 sun_col;          /* warm main light */
  V3 fill_col;         /* cool fill light */
  V3 rim_col;          /* colour of the glowing edge */
  V3 spec_col;         /* colour of the shiny highlight */
  V3 sky_col;          /* dim colour of the empty space behind everything */
  V3 land_col;         /* Earth world only: land */
  V3 sea_col;          /* Earth world only: sea */
} Theme;

#define N_THEMES 8

static const Theme themes[N_THEMES] = {
    /* SATURN — cream + golden bands, ivory rings */
    {"SATURN",
     {0.92f, 0.84f, 0.66f},
     {0.55f, 0.45f, 0.28f},
     {0.85f, 0.79f, 0.66f},
     {1.00f, 0.92f, 0.78f},
     {0.32f, 0.42f, 0.58f},
     {1.00f, 0.62f, 0.30f},
     {1.00f, 0.95f, 0.85f},
     {0.04f, 0.05f, 0.08f},
     {0.50f, 0.45f, 0.30f},
     {0.10f, 0.20f, 0.45f}},

    /* MARS — rust planet, dust rings */
    {"MARS",
     {0.78f, 0.45f, 0.28f},
     {0.50f, 0.22f, 0.12f},
     {0.72f, 0.55f, 0.42f},
     {1.00f, 0.86f, 0.65f},
     {0.32f, 0.22f, 0.40f},
     {1.00f, 0.40f, 0.18f},
     {1.00f, 0.92f, 0.78f},
     {0.05f, 0.04f, 0.06f},
     {0.55f, 0.30f, 0.18f},
     {0.20f, 0.15f, 0.10f}},

    /* OCEAN — blue planet, silver rings */
    {"OCEAN",
     {0.32f, 0.55f, 0.85f},
     {0.10f, 0.28f, 0.55f},
     {0.78f, 0.85f, 0.92f},
     {1.00f, 0.95f, 0.85f},
     {0.55f, 0.65f, 0.85f},
     {0.45f, 0.85f, 1.00f},
     {1.00f, 1.00f, 1.00f},
     {0.04f, 0.05f, 0.10f},
     {0.30f, 0.65f, 0.30f},
     {0.10f, 0.30f, 0.65f}},

    /* FOREST — green planet, pale rings */
    {"FOREST",
     {0.28f, 0.62f, 0.30f},
     {0.10f, 0.32f, 0.12f},
     {0.78f, 0.85f, 0.62f},
     {1.00f, 0.92f, 0.72f},
     {0.42f, 0.62f, 0.55f},
     {0.55f, 1.00f, 0.45f},
     {1.00f, 0.95f, 0.80f},
     {0.04f, 0.06f, 0.05f},
     {0.40f, 0.62f, 0.20f},
     {0.10f, 0.28f, 0.55f}},

    /* FIRE — molten planet, ember rings */
    {"FIRE",
     {0.95f, 0.42f, 0.18f},
     {0.65f, 0.10f, 0.05f},
     {0.92f, 0.55f, 0.28f},
     {1.00f, 0.78f, 0.40f},
     {0.45f, 0.18f, 0.40f},
     {1.00f, 0.30f, 0.10f},
     {1.00f, 0.85f, 0.65f},
     {0.06f, 0.04f, 0.04f},
     {0.85f, 0.42f, 0.15f},
     {0.40f, 0.10f, 0.05f}},

    /* ARCTIC — pale icy blue planet, white rings */
    {"ARCTIC",
     {0.78f, 0.88f, 0.95f},
     {0.45f, 0.62f, 0.78f},
     {0.92f, 0.96f, 1.00f},
     {1.00f, 0.95f, 0.88f},
     {0.55f, 0.70f, 0.92f},
     {0.85f, 0.95f, 1.00f},
     {1.00f, 1.00f, 1.00f},
     {0.04f, 0.06f, 0.10f},
     {0.78f, 0.88f, 0.92f},
     {0.30f, 0.55f, 0.78f}},

    /* VIOLET — magenta gas giant, pink rings */
    {"VIOLET",
     {0.62f, 0.32f, 0.78f},
     {0.30f, 0.10f, 0.45f},
     {0.85f, 0.62f, 0.92f},
     {1.00f, 0.85f, 1.00f},
     {0.42f, 0.55f, 0.78f},
     {1.00f, 0.45f, 0.85f},
     {1.00f, 0.92f, 1.00f},
     {0.05f, 0.03f, 0.08f},
     {0.55f, 0.30f, 0.62f},
     {0.18f, 0.10f, 0.30f}},

    /* GOLD — molten gold planet, brass rings */
    {"GOLD",
     {0.95f, 0.78f, 0.32f},
     {0.60f, 0.42f, 0.10f},
     {0.92f, 0.78f, 0.42f},
     {1.00f, 0.92f, 0.65f},
     {0.42f, 0.30f, 0.55f},
     {1.00f, 0.55f, 0.18f},
     {1.00f, 0.95f, 0.78f},
     {0.05f, 0.04f, 0.05f},
     {0.78f, 0.62f, 0.22f},
     {0.30f, 0.20f, 0.08f}},
};

/* §6 — colour on screen. A terminal can't show arbitrary colours, so we set
 * up a fixed 6×6×6 grid of 216 colours once, and later snap each computed
 * colour to the nearest one. Old terminals with only 8 colours fall back to
 * plain white. */

static int g_256; /* true if the terminal has the full 256-colour set */

static void color_init(void) {
  start_color();
  use_default_colors();
  g_256 = (COLORS >= 256);
  if (g_256) {
    /* claim 216 slots for the colour grid, plus a few for the HUD */
    for (int i = 0; i < 216; i++)
      init_pair((short)(PAIR_CUBE_BASE + i), (short)(16 + i), -1);
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
    init_pair(PAIR_FLASH, 226, -1);
  } else {
    /* bare 8-colour terminal: just white, plus HUD colours */
    init_pair(PAIR_CUBE_BASE, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
  }
}

/* The little steps that turn one finished colour into one character on screen. */

/* round one colour channel (0..1) to the nearest of the 6 grid levels */
static inline int quantize_axis(float c) {
  int q = (int)(c * (float)(CUBE_SIDE - 1) + 0.5f);
  if (q > CUBE_SIDE - 1)
    q = CUBE_SIDE - 1;
  if (q < 0)
    q = 0;
  return q;
}

/* rein in over-bright values, then correct for how a screen shows colour */
static inline V3 tonemap_gamma(V3 hdr) {
  return v3(gamma_enc(reinhard(hdr.x)), gamma_enc(reinhard(hdr.y)),
            gamma_enc(reinhard(hdr.z)));
}

/* find this colour's slot in the 6×6×6 grid */
static inline int rgb_to_cube_pair(V3 enc) {
  int r = quantize_axis(enc.x), g = quantize_axis(enc.y),
      b = quantize_axis(enc.z);
  return PAIR_CUBE_BASE + r * CUBE_SIDE * CUBE_SIDE + g * CUBE_SIDE + b;
}

/* how bright this colour looks to the eye (green counts the most) */
static inline float rec601_luma(V3 enc) {
  return LUMA_W_R * enc.x + LUMA_W_G * enc.y + LUMA_W_B * enc.z;
}

/* brighter colours pick a denser character from the ramp */
static inline char luma_to_ramp_glyph(float luma) {
  int ri = (int)(luma * (float)(RAMP_LEN - 1) + 0.5f);
  if (ri < 0)
    ri = 0;
  if (ri >= RAMP_LEN)
    ri = RAMP_LEN - 1;
  return k_ramp[ri];
}

/* draw the brightest cells bold and the darkest dim, for extra contrast */
static inline int luma_to_attr(float luma) {
  return (luma > LUMA_BOLD_THRESH)  ? A_BOLD
         : (luma < LUMA_DIM_THRESH) ? A_DIM
                                    : A_NORMAL;
}

/* actually stamp the character on the screen — the only place we touch ncurses */
static inline void paint_glyph(int y, int x, int pair, int attr, char ch) {
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(y, x, (chtype)(unsigned char)ch);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Put one finished colour onto one screen cell. Brightness has to be tamed
 * BEFORE snapping to the palette — otherwise almost everything rounds to the
 * same few grid colours and the whole picture goes flat. */
static void paint_cell(int sx, int sy, V3 col) {
  V3 enc = tonemap_gamma(col);

  if (g_256) {
    int pair = rgb_to_cube_pair(enc);
    float luma = rec601_luma(enc);
    paint_glyph(sy, sx, pair, luma_to_attr(luma), luma_to_ramp_glyph(luma));
  } else {
    /* 8-colour terminal: pick a character by brightness, no colour grid */
    float luma = rec601_luma(enc);
    paint_glyph(sy, sx, PAIR_CUBE_BASE, A_NORMAL, luma_to_ramp_glyph(luma));
  }
}

/* §7 — the worlds. A Pattern just names one; the actual numbers that say how
 * it looks live in PatternParams, filled in by pattern_set further down. */

/* Which world we're looking at, switched with the n / P keys. */
typedef enum {
  PATTERN_SATURN = 0, /* banded gas giant with the gapped rings */
  PATTERN_URANUS = 1, /* pale planet, faint thin rings, seen nearly edge-on */
  PATTERN_EARTH = 2,  /* blue-green continents, no bands, modest rings */
  PATTERN_EXO = 3,    /* a made-up planet that changes when you reseed */
  N_PATTERNS = 4,     /* how many worlds, so the keys can wrap around */
} Pattern;

static const char *pattern_name(Pattern p) {
  switch (p) {
  case PATTERN_SATURN:
    return "SATURN ";
  case PATTERN_URANUS:
    return "URANUS ";
  case PATTERN_EARTH:
    return "EARTH-R";
  case PATTERN_EXO:
    return "EXO    ";
  default:
    return "?      ";
  }
}

/* All the numbers that make one world look the way it does — ring size, how
 * many bands, where the dark gap sits, whether it has continents, and how the
 * camera is framed. The user never sets these directly: pattern_set works them
 * out from the chosen world (and the random seed, for the "exo" one), and the
 * shading code only reads them. */
typedef struct {
  float ring_r_in, ring_r_out;      /* where the rings start and end */
  float cam_height;                 /* how high the camera floats above the rings */
  float band_freq, band_amp;        /* how many bands across the planet, and how strong */
  bool has_continents, has_cassini; /* draw land and sea? carve a dark ring gap? */
  float cassini_r, cassini_w;       /* where that gap sits, and how wide it is */
  float ring_density_floor;         /* how solid the rings stay at their faintest (0..1) */
} PatternParams;

/* §8 — the ray tests: given a ray (a start point and a direction), find where
 * it hits the planet (a sphere) or the rings (a flat ring shape). This is just
 * geometry — all the colour and lighting comes later, in §9. */

/* Does this ray hit the sphere, and if so, how far away? It solves the classic
 * "where does a line cross a ball" equation and hands back the distance to the
 * nearest hit in front of us; returns false if the ray sails past. (It also
 * copes with starting inside the ball, just in case.) The distance comes back
 * in *out_t. */
static bool ray_sphere(V3 ro, V3 rd, V3 center, float r, float *out_t) {
  V3 oc = v3_sub(ro, center);
  float b = v3_dot(oc, rd);
  float c = v3_dot(oc, oc) - r * r;
  float disc = b * b - c;
  if (disc < 0.f)
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

/* Does this ray hit the rings? The rings are just a flat disc lying level,
 * with a hole in the middle. We find where the ray crosses that flat plane,
 * then check the spot landed between the inner and outer edges. Hands back the
 * distance and the exact spot. Everything that makes the rings look
 * interesting (bands, the gap, the glow, the shadow) is added later in §9. */
static bool ray_ring(V3 ro, V3 rd, float r_in, float r_out, float *out_t,
                     V3 *out_hit) {
  if (fabsf(rd.y) < 1e-5f)
    return false;
  float t = -ro.y / rd.y;
  if (t < 1e-3f)
    return false;
  V3 hit = v3_add(ro, v3_scl(rd, t));
  float r2 = hit.x * hit.x + hit.z * hit.z;
  if (r2 < r_in * r_in)
    return false;
  if (r2 > r_out * r_out)
    return false;
  *out_t = t;
  *out_hit = hit;
  return true;
}

/* A quick yes/no: from this spot, looking toward `dir`, is the sphere in the
 * way? We use it to ask "can this point see the sun, or does the planet block
 * it?" — a cheap, sharp-edged shadow test. */
static bool hard_shadow_sphere(V3 origin, V3 dir, V3 sphere_c, float sphere_r) {
  V3 o = v3_add(origin, v3_scl(dir, 1e-3f)); /* nudge off the surface so it can't shadow itself */
  V3 oc = v3_sub(o, sphere_c);
  float b = v3_dot(oc, dir);
  float c = v3_dot(oc, oc) - sphere_r * sphere_r;
  float disc = b * b - c;
  if (disc < 0.f)
    return false;
  float t = -b - sqrtf(disc);
  return t > 1e-3f;
}

/* How much sun actually reaches this spot, from 0 (full shadow) to 1 (full
 * sun). We treat the sun as a small disc and fire several rays spread across
 * it; the answer is the fraction that get past the planet. A partial fraction
 * is exactly what gives a shadow its soft, fuzzy edge. The spread is fixed per
 * cell per frame, so the fuzz stays put instead of crawling around. */
static float soft_shadow(V3 origin, V3 sun_dir, V3 sphere_c, float sphere_r,
                         int sx, int sy, int frame) {
  /* two directions across the sun's face, so we can spread samples over it */
  V3 up_seed = (fabsf(sun_dir.y) < 0.9f) ? v3(0, 1, 0) : v3(1, 0, 0); /* avoid a bad axis when the sun is nearly overhead */
  V3 tx = v3_norm(v3_cross(up_seed, sun_dir));
  V3 ty = v3_cross(sun_dir, tx);

  int n_lit = 0;
  for (int i = 0; i < SOFT_SHADOW_SAMPLES; i++) {
    uint32_t h = hash3(sx, sy, frame * SOFT_SHADOW_SAMPLES + i);
    float r1 = hash01(h);
    float r2 = hash01(h ^ 0xA5A5A5A5u);
    float ang = 2.f * (float)M_PI * r1;
    float rad = sqrtf(r2) * SUN_ANGULAR_RADIUS;
    V3 offset =
        v3_add(v3_scl(tx, cosf(ang) * rad), v3_scl(ty, sinf(ang) * rad));
    V3 dir = v3_norm(v3_add(sun_dir, offset));
    if (!hard_shadow_sphere(origin, dir, sphere_c, sphere_r))
      n_lit++;
  }
  return (float)n_lit / (float)SOFT_SHADOW_SAMPLES;
}

/* §9 — shading: work out what colour a spot is, given which way it faces, the
 * sun, and the camera. One function per thing we can hit (planet, rings,
 * space), plus a few small lighting helpers below. */

/* The edge of a planet looks darker than its middle, because near the rim
 * we're looking through much more of its hazy air. This fades a spot toward
 * the edge. (NdotV is near 1 facing us, near 0 at the edge.) */
static float limb_darken(float NdotV) { return powf(clamp01(NdotV), LIMB_K); }

/* A warm halo glows right at the planet's lit edge, where sunlight skims
 * through the atmosphere. Brightest at the very rim, fading inward, and only
 * on the sunlit side. */
static V3 atmospheric_rim(float NdotV, float NdotL, V3 rim_col) {
  if (NdotL <= 0.f || NdotV >= RIM_WIDTH)
    return v3(0, 0, 0);
  float t = 1.f - NdotV / RIM_WIDTH; /* 1 right at the edge, 0 further in */
  float rim = t * t * NdotL * RIM_STRENGTH;
  return v3_scl(rim_col, rim);
}

/* The bright pinpoint highlight of a shiny surface: strongest when the sun
 * bounces straight off the surface into the camera. `shininess` controls how
 * tight that spot is. Returns 0..1; shade_planet tints and dims it. */
static float phong_specular(V3 N, V3 L, V3 V, float shininess) {
  V3 R = v3_sub(v3_scl(N, 2.f * v3_dot(N, L)), L);
  return powf(fmaxf(0.f, v3_dot(R, V)), shininess);
}

/* Declared up here so the planet shader below can ask the rings how much
 * shadow they throw on the planet (the full versions are just below). */
static float ring_density_at(float r, float theta, const PatternParams *pp,
                             float seed_phase);
static float ring_transmittance(V3 P, V3 sun_dir, const PatternParams *pp,
                                float seed_phase);

/* For the NORMAL debug view: paint a spot by which way it faces, as a colour. */
static V3 normal_to_rgb(V3 n) {
  return v3(n.x * 0.5f + 0.5f, n.y * 0.5f + 0.5f, n.z * 0.5f + 0.5f);
}

/* The planet's own surface colour at a spot, before any light hits it: either
 * the stripey bands of a gas giant, or land-and-sea for the Earth world. */
static V3 planet_albedo(const Theme *th, const PatternParams *pp, V3 N,
                        float continent_phase, float seed_phase, Pattern pat) {
  V3 albedo = th->planet_base;
  if (pp->band_amp > 0.001f) {
    float band = sinf(N.y * pp->band_freq + seed_phase * 1.7f);
    float t = 0.5f + 0.5f * band; /* turn the wave into a 0..1 blend amount */
    t *= pp->band_amp * 2.0f;     /* how strongly the bands show */
    if (t > 1.f)
      t = 1.f;
    albedo = v3_lerp(albedo, th->planet_band_tint, t);
  }
  if (pp->has_continents) {
    float u = atan2f(N.x, N.z) / (float)M_PI; /* longitude on the globe */
    float v = N.y;                            /* latitude on the globe */
    float land =
        fbm2(u * 3.5f + continent_phase, v * 2.5f + continent_phase * 0.7f);
    bool is_land = (land > CONTINENT_LAND_THRESH);
    albedo = (is_land || pat != PATTERN_EARTH) ? th->land_col : th->sea_col;
  }
  return albedo;
}

/* Work out the lit colour of a spot on the planet. Start from the surface's
 * own colour, add up the light landing on it (a faint glow everywhere, the
 * main sunlight on the lit side, a soft fill, and a shiny highlight), then
 * darken the edge, dim whatever the rings are shadowing, and add the warm rim
 * glow. The two debug views bail out early with just the plain colour. */
static V3 shade_planet(const Theme *th, const PatternParams *pp,
                       float continent_phase, float seed_phase, V3 hit,
                       V3 view_dir, V3 sun_dir, Pattern pat, ShadeMode mode) {
  V3 N = v3_norm(hit); /* which way the surface faces here */
  /* The fill light comes from a fixed spot (up and toward us), not from
   * opposite the sun. Opposite-the-sun fill lights up the far night side and
   * makes the planet look flat and oddly two-faced. */
  V3 fill_dir = v3_norm(v3(FILL_DIR_X, FILL_DIR_Y, FILL_DIR_Z));

  V3 albedo = planet_albedo(th, pp, N, continent_phase, seed_phase, pat);

  /* Debug views skip lighting entirely. */
  if (mode == SHADE_NORMAL)
    return normal_to_rgb(N);
  if (mode == SHADE_FLAT)
    return albedo;

  /* How squarely the sun, the fill light, and our eye each line up with the
   * surface. 1 means head-on, 0 means edge-on. */
  float NdotL = v3_dot(N, sun_dir);
  if (NdotL < 0.f)
    NdotL = 0.f;
  float NdotF = v3_dot(N, fill_dir);
  if (NdotF < 0.f)
    NdotF = 0.f;
  float NdotV = fabsf(v3_dot(N, view_dir));

  /* The shiny highlight, only where the sun actually reaches. */
  float spec = phong_specular(N, sun_dir, view_dir, SPEC_SHININESS) *
               (NdotL > 0.f ? 1.f : 0.f);

  /* The four kinds of light, each tinted and scaled by how directly it lands. */
  V3 ambient = v3_scl(albedo, AMBIENT_K);
  V3 diffuse = v3_scl(v3_mul(albedo, th->sun_col), NdotL);
  V3 fill = v3_scl(v3_mul(albedo, th->fill_col), NdotF * FILL_K);
  V3 specular = v3_scl(th->spec_col, spec * SPEC_K);

  /* The rings can sit between this spot and the sun and shadow it. Only the
   * direct sunlight (sun, highlight, and the rim below) gets dimmed; the
   * ambient and fill come from all around, so they're left alone. */
  float ring_through = ring_transmittance(hit, sun_dir, pp, seed_phase);
  diffuse = v3_scl(diffuse, ring_through);
  specular = v3_scl(specular, ring_through);

  /* Darken toward the edge — but only the light that bounced off the surface
   * toward us (sun + highlight). The ambient and fill arrive from everywhere,
   * so dimming them too would turn the edge pitch black. */
  float limb = limb_darken(NdotV);
  V3 atmo_attenuated = v3_scl(v3_add(diffuse, specular), limb);
  V3 col = v3_add(v3_add(ambient, fill), atmo_attenuated);

  /* The warm glowing edge, also dimmed where the rings shadow it. */
  V3 rim = atmospheric_rim(NdotV, NdotL, th->rim_col);
  col = v3_add(col, v3_scl(rim, ring_through));

  return col;
}

/* How solid the rings are at a given spot, from 0 (empty) to 1 (opaque). The
 * spot is given as distance-from-centre and angle-around. Three things shape
 * it: the rings fade in softly at their inner and outer edges, light/dark
 * bands ripple across them, and the dark gap dips the density down where it
 * sits. The SAME function decides both the visible rings and the shadow they
 * cast on the planet, so the two always match. (r = distance, theta = angle.) */
static float ring_density_at(float r, float theta, const PatternParams *pp,
                             float seed_phase) {
  float edge_in = smoothstep(pp->ring_r_in, pp->ring_r_in + 0.04f, r);
  float edge_out = 1.f - smoothstep(pp->ring_r_out - 0.04f, pp->ring_r_out, r);
  float density =
      pp->ring_density_floor +
      (1.0f - pp->ring_density_floor) *
          (0.5f + 0.5f * sinf(r * RING_BAND_FREQ + theta * 0.4f + seed_phase));
  density *= edge_in * edge_out;

  if (pp->has_cassini) {
    float gap = smoothstep(0.f, pp->cassini_w, fabsf(r - pp->cassini_r));
    density *= CASSINI_DIM + (1.f - CASSINI_DIM) * gap;
  }
  return density;
}

/* How much sunlight gets through the rings to reach a spot on the planet,
 * from 1 (nothing in the way) down to 0 (a solid ring band blocks it). We
 * follow the line from the spot toward the sun, see where it crosses the ring
 * plane, and if that crossing lands on the rings we look up how solid they are
 * there. This is what paints the rings' shadow stripes across the planet. */
static float ring_transmittance(V3 P, V3 sun_dir, const PatternParams *pp,
                                float seed_phase) {
  /* spot and sun on the same side of the rings → the line never crosses them */
  if (sun_dir.y * P.y >= 0.f)
    return 1.f;
  if (fabsf(sun_dir.y) < 1e-5f) /* line runs flat along the rings → skip */
    return 1.f;

  float t = -P.y / sun_dir.y;
  if (t < 1e-3f)
    return 1.f;

  V3 crossing = v3_add(P, v3_scl(sun_dir, t));
  float r2 = crossing.x * crossing.x + crossing.z * crossing.z;
  float r_in2 = pp->ring_r_in * pp->ring_r_in;
  float r_out2 = pp->ring_r_out * pp->ring_r_out;
  if (r2 < r_in2 || r2 > r_out2)
    return 1.f;

  float r = sqrtf(r2);
  float theta = atan2f(crossing.z, crossing.x);
  return 1.f - ring_density_at(r, theta, pp, seed_phase);
}

/* Colour of a spot on the rings. Look up how solid the rings are here, work
 * out how much sun reaches it (soft shadow of the planet), and add the dusty
 * glow you get when the sun is roughly behind the rings and shines through
 * them. The two debug views bail out early. */
static V3 shade_ring(const Theme *th, const PatternParams *pp, float seed_phase,
                     V3 hit, V3 view_dir, V3 sun_dir, int sx, int sy, int frame,
                     ShadeMode mode) {
  float r = sqrtf(hit.x * hit.x + hit.z * hit.z);
  float theta = atan2f(hit.z, hit.x);
  float density = ring_density_at(r, theta, pp, seed_phase);

  /* Debug views. The rings all face the same way, so a plain facing-direction
   * view would be one flat colour; instead we paint the angle as a rainbow and
   * the solidity as brightness, which actually shows the ring structure. */
  if (mode == SHADE_NORMAL) {
    return v3(cosf(theta) * 0.5f + 0.5f, density, sinf(theta) * 0.5f + 0.5f);
  }
  if (mode == SHADE_FLAT) {
    return v3_scl(th->ring_base, density);
  }

  /* How much sun reaches here, softened by the planet's fuzzy shadow. */
  V3 planet_c = v3(0, 0, 0);
  float shadow =
      soft_shadow(hit, sun_dir, planet_c, PLANET_RADIUS, sx, sy, frame);

  /* Brightness from direct sun. We keep a floor so the rings never go fully
   * black even when the sun is edge-on: real ring particles still catch some
   * light bouncing off their neighbours. */
  float diffuse = (0.25f + 0.75f * fabsf(sun_dir.y)) * shadow;

  /* The lit-from-behind glow: thin parts of the rings light up when the sun is
   * roughly behind them from where we're looking. */
  float backlight = -v3_dot(sun_dir, view_dir); /* positive when the sun is behind */
  if (backlight < 0.f)
    backlight = 0.f;
  float glow = powf(backlight, GLOW_SHARP) * (1.f - density) * GLOW_GAIN;

  /* Add it all up: a base glow, the sunlit colour, the dust glow, and a faint
   * haze around the rings even where they're nearly empty. */
  V3 base = v3_scl(th->ring_base, density);
  V3 ambient = v3_scl(base, AMBIENT_K * 1.5f);
  V3 lit = v3_scl(v3_mul(base, th->sun_col), diffuse);
  V3 fwd = v3_scl(th->sun_col, glow * density);
  V3 haze = v3_scl(th->sun_col, glow * 0.15f);

  return v3_add(ambient, v3_add(lit, v3_add(fwd, haze)));
}

/* Colour of empty space: the dim background, with an occasional twinkling
 * star. A cell is a star if its scrambled coordinates happen to land on one. */
static V3 shade_space(const Theme *th, int sx, int sy, int star_seed,
                      float time_secs) {
  V3 base = th->sky_col;
  uint32_t h = hash3(sx, sy, star_seed);
  if ((h % STAR_DENSITY) == 0u) {
    float phase = hash01(h >> 16) * 2.f * (float)M_PI;
    float tw =
        0.5f +
        0.5f * sinf(2.f * (float)M_PI * STAR_TWINKLE_HZ * time_secs + phase);
    if (tw > 0.40f) {
      /* give each star a warm, cool, or white tint, and let it brighten and
       * dim as it twinkles */
      float warm = hash01(h ^ 0xC0FFEEu);
      V3 star_col;
      if (warm > 0.66f)
        star_col = v3(1.0f, 0.85f, 0.65f); /* warm */
      else if (warm > 0.33f)
        star_col = v3(0.85f, 0.92f, 1.0f); /* cool */
      else
        star_col = v3(1.0f, 1.0f, 1.0f); /* pure */
      float br = 0.4f + 0.7f * tw;
      return v3_add(base, v3_scl(star_col, br));
    }
  }
  return base;
}

/* §10 — the scene: everything about what we're showing and how the user is
 * driving it, plus the clock that makes the sun go round. */

/* Everything the program needs to know to draw and run the scene, in one
 * place. The first group is which world we're showing and its random "seeds"
 * (small numbers that make each world look unique); then the clock; then the
 * settings the user changes with the keys. Only init/reseed/tick rewrite this
 * whole thing — the drawing code just reads it. */
typedef struct {
  /* which world, and the random seeds that flavour it */
  Pattern current_pattern; /* Saturn / Uranus / Earth / Exo */
  PatternParams pp;        /* the worked-out look of that world */
  float seed_phase;        /* where the sun starts + the exo planet's flavour */
  float continent_phase;   /* shuffles the Earth world's continents */
  int star_seed;           /* fixes the star pattern */

  /* the clock */
  float time_secs; /* how long it's been running; the sun rides on this */

  /* what the user can change with the keys */
  bool paused;          /* freeze the sun's motion */
  int speed;            /* how fast the sun travels */
  int current_theme;    /* which colour scheme */
  int spp;              /* anti-aliasing quality */
  ShadeMode shade_mode; /* normal view or a debug view */

  int frame; /* counts redraws; used to make the stars twinkle */
} Scene;

/* Which way the sun is shining right now — it circles slowly as time passes. */
static V3 scene_sun_dir(const Scene *s) {
  float omega = 2.f * (float)M_PI / ROTATION_PERIOD_S;
  float az = s->time_secs * omega + s->seed_phase;
  return v3_norm(v3(cosf(az), SUN_ELEV_Y, sinf(az)));
}

/* Fill in all the look-numbers for one world: start from sensible defaults,
 * then change what's special about each. The Exo world makes up its values
 * from its seed. Called when you switch worlds, reseed, or start up. */
static void pattern_set(Scene *s, Pattern p) {
  s->current_pattern = p;
  PatternParams *pp = &s->pp;
  pp->ring_r_in = RING_R_IN_DEFAULT;
  pp->ring_r_out = RING_R_OUT_DEFAULT;
  pp->cam_height = CAM_HEIGHT_DEFAULT;
  pp->band_freq = 8.0f;
  pp->band_amp = 0.18f;
  pp->has_continents = false;
  pp->has_cassini = false;
  pp->cassini_r = CASSINI_R_DEFAULT;
  pp->cassini_w = CASSINI_W_DEFAULT;
  pp->ring_density_floor = 0.55f;

  switch (p) {
  case PATTERN_SATURN:
    pp->band_freq = 12.0f;
    pp->band_amp = 0.20f;
    pp->has_cassini = true;
    pp->ring_r_in = 1.45f;
    pp->ring_r_out = 2.65f;
    pp->cassini_r = 2.10f;
    pp->cassini_w = 0.06f;
    pp->cam_height = 1.10f;
    break;
  case PATTERN_URANUS:
    pp->band_freq = 4.0f;
    pp->band_amp = 0.06f;
    pp->ring_r_in = 1.30f;
    pp->ring_r_out = 1.90f;
    pp->ring_density_floor = 0.35f;
    pp->cam_height = 0.45f;
    break;
  case PATTERN_EARTH:
    pp->band_freq = 0.0f;
    pp->band_amp = 0.0f;
    pp->has_continents = true;
    pp->ring_r_in = 1.40f;
    pp->ring_r_out = 2.00f;
    pp->ring_density_floor = 0.45f;
    pp->cam_height = 1.20f;
    break;
  case PATTERN_EXO: {
    int hsp = (int)(s->seed_phase * 100.0f);
    pp->band_freq = 4.0f + ((float)(hsp % 100) * 0.18f);
    pp->band_amp = 0.10f + ((float)((hsp >> 2) & 7) * 0.04f);
    pp->ring_r_in = 1.35f + ((float)((hsp >> 4) & 7) * 0.04f);
    pp->ring_r_out = pp->ring_r_in + 0.6f + ((float)((hsp >> 6) & 7) * 0.10f);
    pp->has_cassini = (hsp & 1) != 0;
    pp->cassini_r = (pp->ring_r_in + pp->ring_r_out) * 0.5f;
    pp->ring_density_floor = 0.40f;
    pp->cam_height = 0.7f + ((float)((hsp >> 8) & 7) * 0.10f);
    break;
  }
  case N_PATTERNS:
    break;
  }
}

/* Roll fresh randomness: a new sun angle, new continents, new stars, and a
 * new exo planet. Bound to the 'r' key. */
static void scene_reseed(Scene *s) {
  uint32_t h = hash3((int)(s->time_secs * 1000.0f),
                     (int)(s->seed_phase * 100.0f), 0xC0FFEE);
  s->seed_phase = hash01(h) * 2.f * (float)M_PI;
  s->continent_phase = hash01(h >> 16) * 8.0f;
  s->star_seed = (int)(h ^ 0x5A5A5A5Au);
  perm_shuffle(s->star_seed);
  pattern_set(s, s->current_pattern);
}

/* Start fresh at launch: Saturn, fully-lit view, sensible defaults. */
static void scene_init(Scene *s) {
  memset(s, 0, sizeof *s);
  s->speed = SPEED_DEF;
  s->seed_phase = SUN_PHASE0;
  s->continent_phase = 3.0f;
  s->star_seed = 0xDECAF;
  s->spp = SPP_DEF_VAL;
  s->shade_mode = SHADE_LIT;
  perm_shuffle(s->star_seed);
  pattern_set(s, PATTERN_SATURN);
}

/* The one and only place the clock moves forward. Does nothing while paused. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  float speed_mul = (float)s->speed / (float)SPEED_DEF;
  s->time_secs += dt * speed_mul;
}

/* §11 — the camera and the main draw loop. The camera turns each screen cell
 * into a ray pointing out into the world; the draw loop fires those rays and
 * paints whatever they hit. */

/* A simple pinhole camera: where the eye sits, plus three directions (forward,
 * right, up) that frame the view. We rebuild it every frame from the chosen
 * camera height. The up/down field of view is squashed to make up for the tall
 * terminal cells, so the planet stays round instead of egg-shaped. */
typedef struct {
  V3 pos, fwd, right, up; /* eye position and the three view directions */
  float fov_h, fov_v;     /* how wide the view is, across and up/down */
  int cols, rows;         /* size of the picture, in cells */
} Camera;

/* Aim the camera at the planet from the given height, and work out its
 * forward/right/up directions. The up/down view is narrowed to cancel out the
 * terminal's tall cells — without that, the planet would look like an egg. */
static void camera_make(Camera *c, int cols, int rows, float cam_height) {
  c->cols = cols;
  c->rows = rows;
  c->pos = v3(0.f, cam_height, -CAM_DIST_DEFAULT);
  V3 target = v3(0.f, 0.f, 0.f);
  V3 worldup = v3(0.f, 1.f, 0.f);
  c->fwd = v3_norm(v3_sub(target, c->pos));
  c->right = v3_norm(v3_cross(c->fwd, worldup));
  c->up = v3_cross(c->right, c->fwd);
  c->fov_h = FOV_H;
  c->fov_v = FOV_H * (float)rows * ASPECT_Y / (float)cols;
}

/* Turn a spot on the screen into the ray that shoots out through it. The
 * up/down part is flipped, because screen rows count downward while the
 * world's "up" points up. */
static V3 camera_ray(const Camera *c, float fx, float fy) {
  float u = ((2.f * fx + 1.f) - (float)c->cols) / (float)c->cols * c->fov_h;
  float v = -((2.f * fy + 1.f) - (float)c->rows) / (float)c->rows * c->fov_v;
  return v3_norm(v3_add(c->fwd, v3_add(v3_scl(c->right, u), v3_scl(c->up, v))));
}

/* Fire one ray and return the colour it sees: test it against the planet and
 * the rings, keep whichever is closer, and colour that; if it hits neither,
 * it's looking at empty space. (sx, sy, and the frame number ride along so the
 * rings' soft shadow stays steady instead of shimmering each frame.) */
static V3 trace_one(const Scene *s, const Theme *th, const Camera *cam,
                    V3 sun_dir, float fx, float fy, int sx, int sy) {
  V3 rd = camera_ray(cam, fx, fy);

  float t_sphere = 0.f, t_ring = 0.f;
  V3 hit_ring = v3(0, 0, 0);
  bool hit_s = ray_sphere(cam->pos, rd, v3(0, 0, 0), PLANET_RADIUS, &t_sphere);
  bool hit_r = ray_ring(cam->pos, rd, s->pp.ring_r_in, s->pp.ring_r_out,
                        &t_ring, &hit_ring);

  if (hit_s && (!hit_r || t_sphere < t_ring)) {
    V3 hit = v3_add(cam->pos, v3_scl(rd, t_sphere));
    V3 view = v3_norm(v3_scl(rd, -1.f));
    return shade_planet(th, &s->pp, s->continent_phase, s->seed_phase, hit,
                        view, sun_dir, s->current_pattern, s->shade_mode);
  }
  if (hit_r) {
    V3 view = v3_norm(v3_scl(rd, -1.f));
    return shade_ring(th, &s->pp, s->seed_phase, hit_ring, view, sun_dir, sx,
                      sy, s->frame, s->shade_mode);
  }
  return shade_space(th, sx, sy, s->star_seed, s->time_secs);
}

/* Colour one cell by firing a few rays nudged to slightly different spots
 * inside it and averaging the results — that's what smooths jagged edges. The
 * nudges are fixed per cell per frame, so edges don't crawl around. */
static V3 supersample_cell(const Scene *s, const Theme *th, const Camera *cam,
                           V3 sun_dir, int sx, int sy, int top, int spp) {
  V3 acc = v3(0, 0, 0);
  for (int p = 0; p < spp; p++) {
    uint32_t h = hash3(sx, sy, s->frame * spp + p);
    float jx = hash01(h) - 0.5f; /* a small nudge within the cell */
    float jy = hash01(h ^ 0xDEADBEEFu) - 0.5f;
    float fx = (float)sx + 0.5f + jx; /* the nudged spot we actually aim at */
    float fy = (float)(sy - top) + 0.5f + jy;
    acc = v3_add(acc, trace_one(s, th, cam, sun_dir, fx, fy, sx, sy));
  }
  return v3_scl(acc, 1.f / (float)spp);
}

/* Draw the whole picture: set up the camera and the sun once, then colour
 * every cell. The top and bottom rows are left for the HUD. */
static void scene_draw(int cols, int rows, const Scene *s) {
  Camera cam;
  camera_make(&cam, cols, rows - 2, s->pp.cam_height);
  V3 sun_dir = scene_sun_dir(s);
  const Theme *th = &themes[s->current_theme];

  int top = 1;        /* leave row 0 for the status line */
  int bot = rows - 1; /* and the last row for the key hints */
  if (rows < 5) {     /* too short for a HUD: just use every row */
    top = 0;
    bot = rows;
  }

  int spp = s->spp;
  if (spp < SPP_MIN_VAL)
    spp = SPP_MIN_VAL;
  if (spp > SPP_MAX_VAL)
    spp = SPP_MAX_VAL;

  for (int sy = top; sy < bot; sy++)
    for (int sx = 0; sx < cols; sx++)
      paint_cell(sx, sy,
                 supersample_cell(s, th, &cam, sun_dir, sx, sy, top, spp));
}

/* §12 — the text overlay: a status line along the top and the key hints along
 * the bottom. */

static void hud_draw(int cols, int rows, const Scene *s, double fps,
                     int sim_fps) {
  /* status line, top-right */
  V3 sd = scene_sun_dir(s);
  float az = atan2f(sd.z, sd.x) * 180.f / (float)M_PI; /* sun angle, in degrees */

  char buf[160];
  snprintf(buf, sizeof buf,
           " %5.1f fps  %3d Hz  %s  %-8s  %s  spp:%d  sun:%+5.0f°  speed:%-2d ",
           fps, sim_fps,
           s->paused ? "PAUSED " : pattern_name(s->current_pattern),
           themes[s->current_theme].name, shade_mode_name(s->shade_mode),
           s->spp, (double)az, s->speed);
  int len = (int)strlen(buf);
  if (len > cols)
    len = cols;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, cols - len, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* title, top-left */
  attron(COLOR_PAIR(PAIR_HUD));
  mvprintw(0, 0, " SATURN-WITH-RINGS · RGB ");
  attroff(COLOR_PAIR(PAIR_HUD));

  /* key hints, bottom row */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(rows - 1, 0,
           " q:quit  spc:pause  n/P:pattern  t/T:theme  m:mode  s:spp  "
           "+/-:speed  []:Hz  r:reseed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §13 — the program itself: signals, the keyboard, and the main loop that
 * ties everything together. */

/* The whole running program in one place: the scene, the current terminal
 * size, how fast the world updates, and two flags the signal handlers flip. */
typedef struct {
  Scene scene;    /* the world being shown */
  int cols, rows; /* terminal size (re-read when the window resizes) */
  int sim_fps;    /* how many times a second the world updates */
  /* These two are touched by signal handlers, so they need the special
   * "safe to poke from a signal" type. */
  volatile sig_atomic_t running;     /* set to 0 to quit */
  volatile sig_atomic_t need_resize; /* set when the window was resized */
} App;

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/* Wire up the OS signals: Ctrl-C / kill ask us to quit, a window resize sets a
 * flag, and on the way out we always put the terminal back to normal. */
static void install_signals(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

/* Put the terminal into the mode we need: don't echo keys, read them
 * immediately without waiting, hide the cursor, and set up the colours. */
static void screen_init(void) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1);
  color_init();
}

/* Act on one keypress. Returns false only for quit, which ends the program. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
  case 'p':
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
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    break;

  case 'n':
  case 'N':
    pattern_set(s, (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS));
    break;
  case 'P': /* prev pattern (capital P, since lowercase p = pause) */
    pattern_set(
        s, (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS));
    break;

  case 's':
  case 'S': {
    int next = s->spp == 1 ? 2 : (s->spp == 2 ? 4 : 1);
    s->spp = next;
    break;
  }
  case 'm':
  case 'M':
    s->shade_mode = (ShadeMode)(((int)s->shade_mode + 1) % SHADE_N);
    break;
  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  install_signals();

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init();
  getmaxyx(stdscr, app->rows, app->cols);

  scene_init(&app->scene);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* The window was resized: have ncurses re-measure it, and reset the timer
     * so the pause for resizing doesn't count as elapsed time. */
    if (app->need_resize) {
      app->need_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, app->rows, app->cols);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* How much real time passed since last frame. Capped, so that after a long
     * stall the sun doesn't suddenly jump way ahead. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > DT_CAP_NS)
      dt = DT_CAP_NS;

    /* Advance the world in fixed-size steps: bank the elapsed time and take one
     * step for each whole step's worth. This keeps the motion the same speed
     * whether drawing is fast or slow. */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    /* Refresh the fps number on screen, averaged over a short window. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_SAMPLE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Draw everything: clear, paint the scene, lay the HUD on top, and flip it
     * all to the screen in one update. */
    long long t0 = clock_ns();
    erase();
    scene_draw(app->cols, app->rows, &app->scene);
    hud_draw(app->cols, app->rows, &app->scene, fps_display, app->sim_fps);
    wnoutrefresh(stdscr);
    doupdate();

    /* Read one keypress if there is one; quitting is the only thing that stops
     * the loop. */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;

    /* Bump the frame counter (it drives the star twinkle), then sleep just long
     * enough to hold a steady redraw rate. */
    app->scene.frame++;
    clock_sleep_ns(NS_PER_SEC / RENDER_FPS_CAP - (clock_ns() - t0));
  }

  return 0;
}

