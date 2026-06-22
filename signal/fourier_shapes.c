/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fourier_shapes.c — picks one of 21 preset 2-D shapes, breaks it into
 * spinning arms (a DFT), and animates the arm chain redrawing the shape.
 * The headline is the bar up top: how much of the shape's "energy" the
 * arms you've turned on have captured so far. Add arms and watch it fill.
 *
 * Sister files: signal/fourier_draw.c (you trace the shape by hand),
 * signal/epicycles.c (the same idea taught in depth), signal/fft_vis.c
 * (the 1-D audio version), signal/dft_helloworld.c (start here if you've
 * never met a DFT). The bar is Parseval's theorem made visible
 * (Parseval 1799); intro video: 3Blue1Brown "But what is a Fourier
 * series?". Math background: Smith, "The Scientist and Engineer's Guide
 * to DSP", ch. 8.
 */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config — the knobs ── */

#define N_SAMPLES 256 /* points sampled per shape; also the max arm count   */
#define TRAIL_LEN 512 /* how many past tip positions the trail remembers    */
#define RENDER_FPS 30
#define RENDER_NS (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES 360  /* frames it takes the chain to trace once around */
#define AUTO_ADD_FRAMES 8 /* frames between auto-added arms                  */
#define N_CIRCLES 5       /* most orbit guide rings drawn at once            */
#define SHAPE_SCALE 0.38f /* how big the shape is, as a slice of the screen  */
#define CELL_W 8          /* a terminal cell is this many pixels wide        */
#define CELL_H 16         /* and this many tall (so cells are 2x taller)     */

/* Colour-pair slots. The first ten change with the theme; the last five
 * (HUD/hint/energy bar) stay fixed so they're always readable. */
enum {
  CP_ARM_HI = 1,  /* big arm     */
  CP_ARM_MID = 2, /* medium arm  */
  CP_ARM_LO = 3,  /* tiny arm    */
  CP_CIRCLE = 4,  /* orbit rings */
  CP_TRAIL_1 = 5, /* trail, newest */
  CP_TRAIL_2 = 6, /* trail, middle */
  CP_TRAIL_3 = 7, /* trail, oldest */
  CP_BOB = 8,     /* tip marker   */
  CP_PIVOT = 9,   /* centre marker */
  CP_GHOST = 10,  /* faint target-shape dots */
  CP_HUD = 11,        /* top status line */
  CP_HINT = 12,       /* bottom key hints */
  CP_ENERGY = 13,     /* bar when nearly done   (>= 80%) */
  CP_ENERGY_MID = 14, /* bar when recognisable  (50..80%) */
  CP_ENERGY_LO = 15,  /* bar when rough         (< 50%) */
};

/* ── §2  clock — a steady nanosecond timer and a sleep ── */

static long long clock_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  /* The pause that keeps us at 30 fps instead of pinning a CPU core. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3  themes — the colour palettes you cycle with t / T ── */

/*
 * One colour scheme for the animation. Every colour comes in two flavours:
 * a rich 256-colour version and a plain 8-colour fallback for old
 * terminals. We deliberately avoid the darkest codes (16-23 and 232-239)
 * because they look like black and vanish on screen.
 */
typedef struct {
  const char *name;
  /* 256-colour version: one foreground colour per thing we draw */
  short arm_hi, arm_mid, arm_lo; /* the three arm-size buckets   */
  short circle;                  /* orbit guide rings            */
  short trail_1, trail_2, trail_3; /* trail, newest to oldest    */
  short bob, pivot;                /* tip marker, centre marker  */
  short ghost;                     /* faint target-shape dots    */
  /* 8-colour fallback: same things, basic colours */
  short arm_hi8, arm_mid8, arm_lo8;
  short circle8;
  short trail_1_8, trail_2_8, trail_3_8;
  short bob8, pivot8;
  short ghost8;
} Theme;

#define N_THEMES 10

static const Theme k_themes[N_THEMES] = {
    /*  0  Classic   — bright white arms, gold/orange/red trail */
    {"Classic   ", 231,          87,         246,         243,
     226,          208,          196,        231,         220,
     240,          COLOR_WHITE,  COLOR_CYAN, COLOR_WHITE, COLOR_WHITE,
     COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,  COLOR_WHITE, COLOR_YELLOW,
     COLOR_WHITE},
    /*  1  Fire      — red/orange spectrum, peach guides */
    {"Fire      ", 208,          202,       130,         215,
     226,          214,          196,       231,         208,
     240,          COLOR_RED,    COLOR_RED, COLOR_RED,   COLOR_WHITE,
     COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_WHITE, COLOR_RED,
     COLOR_WHITE},
    /*  2  Neon      — magenta/violet/pink, lavender guides */
    {"Neon      ",  207,           165,           105,           141,
     219,           213,           201,           231,           207,
     240,           COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,
     COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,   COLOR_MAGENTA,
     COLOR_WHITE},
    /*  3  Ocean     — teal/blue spectrum */
    {"Ocean     ", 123,        45,          31,         110,        159,
     87,           51,         231,         45,         240,        COLOR_CYAN,
     COLOR_BLUE,   COLOR_BLUE, COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
     COLOR_WHITE,  COLOR_CYAN, COLOR_WHITE},
    /*  4  Matrix    — green spectrum, classic terminal look */
    {"Matrix    ", 118,         46,          28,          64,
     154,          118,         46,          231,         118,
     240,          COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
     COLOR_GREEN,  COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_GREEN,
     COLOR_WHITE},
    /*  5  Sunset    — peach/pink/magenta gradient */
    {"Sunset    ", 215,           209,       174,         217,
     226,          213,           198,       231,         220,
     240,          COLOR_RED,     COLOR_RED, COLOR_RED,   COLOR_WHITE,
     COLOR_YELLOW, COLOR_MAGENTA, COLOR_RED, COLOR_WHITE, COLOR_YELLOW,
     COLOR_WHITE},
    /*  6  Nova      — cosmic stellar: bright white core, orange explosion */
    {"Nova      ", 231,         220,           175,           145,
     226,          208,         197,           231,           175,
     240,          COLOR_WHITE, COLOR_YELLOW,  COLOR_MAGENTA, COLOR_WHITE,
     COLOR_YELLOW, COLOR_RED,   COLOR_MAGENTA, COLOR_WHITE,   COLOR_MAGENTA,
     COLOR_WHITE},
    /*  7  Mono      — high-contrast greyscale */
    {"Mono      ", 231,         252,         245,         240,
     255,          250,         244,         231,         246,
     240,          COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE,  COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
     COLOR_WHITE},
    /*  8  Forest    — green/olive/brown earthy palette */
    {"Forest    ", 154,          70,          28,          100,
     226,          178,          130,         231,         178,
     240,          COLOR_GREEN,  COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
     COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,   COLOR_WHITE, COLOR_YELLOW,
     COLOR_WHITE},
    /*  9  Cyberpunk — electric cyan + hot pink high-contrast */
    {"Cyberpunk ",  51,
     207,           165,
     141,           226,
     207,           197,
     231,           207,
     240,           COLOR_CYAN,
     COLOR_MAGENTA, COLOR_MAGENTA,
     COLOR_WHITE,   COLOR_YELLOW,
     COLOR_MAGENTA, COLOR_RED,
     COLOR_WHITE,   COLOR_MAGENTA,
     COLOR_MAGENTA},
};

static int g_active_theme_index = 0;

/* Repaint the animation colours from theme `idx`; leaves the HUD/energy
 * colours alone so they stay readable no matter the theme. */
static void apply_theme(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  const Theme *t = &k_themes[idx];
  if (COLORS >= 256) {
    init_pair(CP_ARM_HI, t->arm_hi, -1);
    init_pair(CP_ARM_MID, t->arm_mid, -1);
    init_pair(CP_ARM_LO, t->arm_lo, -1);
    init_pair(CP_CIRCLE, t->circle, -1);
    init_pair(CP_TRAIL_1, t->trail_1, -1);
    init_pair(CP_TRAIL_2, t->trail_2, -1);
    init_pair(CP_TRAIL_3, t->trail_3, -1);
    init_pair(CP_BOB, t->bob, -1);
    init_pair(CP_PIVOT, t->pivot, -1);
    init_pair(CP_GHOST, t->ghost, -1);
  } else {
    init_pair(CP_ARM_HI, t->arm_hi8, -1);
    init_pair(CP_ARM_MID, t->arm_mid8, -1);
    init_pair(CP_ARM_LO, t->arm_lo8, -1);
    init_pair(CP_CIRCLE, t->circle8, -1);
    init_pair(CP_TRAIL_1, t->trail_1_8, -1);
    init_pair(CP_TRAIL_2, t->trail_2_8, -1);
    init_pair(CP_TRAIL_3, t->trail_3_8, -1);
    init_pair(CP_BOB, t->bob8, -1);
    init_pair(CP_PIVOT, t->pivot8, -1);
    init_pair(CP_GHOST, t->ghost8, -1);
  }
}

/* Set up the fixed HUD/energy colours once, then load the first theme. */
static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1);        /* bright yellow */
    init_pair(CP_HINT, 51, -1);        /* bright cyan   */
    init_pair(CP_ENERGY, 46, -1);      /* green  (>= 80%) */
    init_pair(CP_ENERGY_MID, 226, -1); /* yellow (50..80%) */
    init_pair(CP_ENERGY_LO, 196, -1);  /* red    (< 50%) */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
    init_pair(CP_ENERGY, COLOR_GREEN, -1);
    init_pair(CP_ENERGY_MID, COLOR_YELLOW, -1);
    init_pair(CP_ENERGY_LO, COLOR_RED, -1);
  }
  apply_theme(0);
}

/* ── §4  shape helpers — building blocks the shape catalog reuses ── */

/* A 2-D point, stored as a complex number: re is x, im is y. Packing
 * the point this way lets the DFT treat the whole shape as one signal. */
typedef struct {
  float re, im;
} Cplx;

/* Greatest common divisor. Spirograph shapes use it to find how many
 * times the small gear must go round before the pen returns home. */
static int gcd_int(int a, int b) {
  while (b) {
    int t = b;
    b = a % b;
    a = t;
  }
  return a;
}

/* Walk evenly around a polygon's outline and drop N points along it.
 * Its sharp corners are what make polygon shapes slow to reconstruct. */
static void sample_poly(const float *vx, const float *vy, int nv, Cplx *out,
                        int N) {
  for (int i = 0; i < N; i++) {
    float t = (float)i / (float)N * (float)nv;
    int seg = (int)t % nv;
    float fraction = t - floorf(t);
    int nxt = (seg + 1) % nv;
    out[i].re = vx[seg] + (vx[nxt] - vx[seg]) * fraction;
    out[i].im = vy[seg] + (vy[nxt] - vy[seg]) * fraction;
  }
}

/* A spirograph rosette: a pen on a small gear (radius r) rolling INSIDE
 * a big ring (radius R), pen offset d from the small gear's centre.
 * However tangled it looks, it's really just two circular motions
 * added, so a handful of arms reproduce it almost perfectly. */
static void sample_hypotrochoid(int R, int r, float d, Cplx *out, int N) {
  int reps = r / gcd_int(R, r);
  float period = 2.f * (float)M_PI * (float)reps;
  float diff = (float)(R - r);
  float scale = ((float)(R - r) + d);
  if (scale <= 0.001f)
    scale = 1.f;
  for (int k = 0; k < N; k++) {
    float t = period * (float)k / (float)N;
    out[k].re = (diff * cosf(t) + d * cosf(diff / (float)r * t)) / scale;
    out[k].im = (diff * sinf(t) - d * sinf(diff / (float)r * t)) / scale;
  }
}

/* Same idea as the hypotrochoid, but the small gear rolls on the
 * OUTSIDE of the ring. Still two simple rotations added, just with
 * looping outer petals instead of an inner rosette. */
static void sample_epitrochoid(int R, int r, float d, Cplx *out, int N) {
  int reps = r / gcd_int(R, r);
  float period = 2.f * (float)M_PI * (float)reps;
  float sum = (float)(R + r);
  float scale = sum + d;
  if (scale <= 0.001f)
    scale = 1.f;
  for (int k = 0; k < N; k++) {
    float t = period * (float)k / (float)N;
    out[k].re = (sum * cosf(t) - d * cosf(sum / (float)r * t)) / scale;
    out[k].im = (sum * sinf(t) - d * sinf(sum / (float)r * t)) / scale;
  }
}

/* ── §5 / §6  the shape catalog — sample_path fills in one shape ── */

/*
 * Each case below writes N points of one shape into out[]. They're
 * grouped by how hard they are to rebuild from arms:
 *   polygons (sharp corners) are the slowest;
 *   smooth curves (circles, Lissajous, spirographs) are the fastest;
 *   cusped and compound curves sit in between.
 * Keeping the shapes here, apart from the maths, means adding a new one
 * is just adding a case.
 */

static const char *k_shape_names[] = {
    "Square          ",
    "Arrow           ",
    "Star-7          ",
    "Cardioid        ",
    "Lissajous 3:2   ",
    "Rose r=cos(2t)  ",
    "Lissajous 5:4   ",
    "Lissajous 7:3   ",
    "Hypotrochoid 5:3",
    "Hypotrochoid 7:4",
    "Epitrochoid 3:1 ",
    "Deltoid (3-cusp)",
    "Hypocycloid 5   ",
    "Gear 8-tooth    ",
    "Crescent        ",
    "Lightning       ",
    "Lissajous 11:7  ",
    "Beaded circle   ",
    "Compound spiro  ",
    "Closed spiral   ",
    "Torus knot 2:5  ",
};
#define N_SHAPES 21

static void sample_path(int si, Cplx *out, int N) {
  switch (si) {

  case 0: {
    /* Square. Four sharp corners make it slow to rebuild. */
    static const float sx[] = {-1.f, 1.f, 1.f, -1.f};
    static const float sy[] = {-1.f, -1.f, 1.f, 1.f};
    sample_poly(sx, sy, 4, out, N);
    break;
  }

  case 1: {
    /* Arrow. A 7-corner outline; even slower to rebuild than the square. */
    static const float ax[] = {0.f, .65f, .30f, .30f, -.30f, -.30f, -.65f};
    static const float ay[] = {1.f, .10f, .10f, -.85f, -.85f, .10f, .10f};
    sample_poly(ax, ay, 7, out, N);
    break;
  }

  case 2: {
    /* 7-pointed star: 14 corners alternating far out and close in. */
    float vx[14], vy[14];
    for (int i = 0; i < 14; i++) {
      float r = (i % 2 == 0) ? 1.f : 0.40f;
      float a = -(float)M_PI / 2.f + (float)i * (float)M_PI / 7.f;
      vx[i] = r * cosf(a);
      vy[i] = r * sinf(a);
    }
    sample_poly(vx, vy, 14, out, N);
    break;
  }

  case 3: {
    /* Cardioid (heart curve). Smooth, so it rebuilds in a few arms. Its
     * centre of mass sits off-origin, so we shift it back to centre. */
    float cx = 0.f;
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      float r = 0.5f * (1.f - cosf(t));
      out[k].re = r * cosf(t);
      out[k].im = r * sinf(t);
      cx += out[k].re;
    }
    cx /= (float)N;
    for (int k = 0; k < N; k++)
      out[k].re -= cx;
    break;
  }

  case 4: {
    /* Lissajous 3:2 — a sine in x against a sine in y. Just two pure
     * tones, so four arms rebuild it exactly. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = sinf(3.f * t + (float)M_PI / 4.f);
      out[k].im = sinf(2.f * t);
    }
    break;
  }

  case 5: {
    /* 4-petal rose. The radius dips negative, which folds the curve
     * back through the centre and traces all four petals. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      float r = cosf(2.f * t);
      out[k].re = r * cosf(t);
      out[k].im = r * sinf(t);
    }
    break;
  }

  case 6: {
    /* Lissajous 5:4 — a denser weave than 3:2, still four arms. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = sinf(5.f * t + (float)M_PI / 3.f);
      out[k].im = sinf(4.f * t);
    }
    break;
  }

  case 7: {
    /* Lissajous 7:3 — wider ratio, flower-like crossings, still four arms. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = sinf(7.f * t);
      out[k].im = sinf(3.f * t);
    }
    break;
  }

  case 8: {
    /* Spirograph rosette (gears 5 and 3). */
    sample_hypotrochoid(5, 3, 5.f, out, N);
    break;
  }

  case 9: {
    /* Another spirograph rosette (gears 7 and 4), more intricate. */
    sample_hypotrochoid(7, 4, 6.f, out, N);
    break;
  }

  case 10: {
    /* Outer-rolling spirograph with looping petals. */
    sample_epitrochoid(3, 1, 2.f, out, N);
    break;
  }

  case 11: {
    /* Deltoid: a triangle-ish curve with a sharp point (cusp) at each
     * of its three corners. Mostly smooth, so it rebuilds at medium speed.
     *   x(t) = (2 cos t + cos 2t) / 3
     *   y(t) = (2 sin t - sin 2t) / 3 */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = (2.f * cosf(t) + cosf(2.f * t)) / 3.f;
      out[k].im = (2.f * sinf(t) - sinf(2.f * t)) / 3.f;
    }
    break;
  }

  case 12: {
    /* Five-cusp hypocycloid: a star-ish curve with five sharp points.
     *   x(t) = (4 cos t + cos 4t) / 5
     *   y(t) = (4 sin t - sin 4t) / 5 */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = (4.f * cosf(t) + cosf(4.f * t)) / 5.f;
      out[k].im = (4.f * sinf(t) - sinf(4.f * t)) / 5.f;
    }
    break;
  }

  case 13: {
    /* 8-tooth gear: the radius jumps in and out eight times around,
     * making square teeth. The sudden jumps make it slow to rebuild. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      float r = 0.6f + 0.3f * (sinf(8.f * t) > 0.f ? 1.f : -1.f);
      out[k].re = r * cosf(t);
      out[k].im = r * sinf(t);
    }
    break;
  }

  case 14: {
    /* Crescent moon: a big right-hand arc joined to a smaller inner arc.
     * Smooth apart from the two join points, so it rebuilds at medium speed. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      if (t < (float)M_PI) {
        /* outer arc, top to bottom on the right */
        float theta = (float)M_PI / 2.f - t;
        out[k].re = cosf(theta);
        out[k].im = sinf(theta);
      } else {
        /* inner arc, bottom to top, shifted right */
        float theta = -(float)M_PI / 2.f + (t - (float)M_PI);
        out[k].re = 0.30f + 0.70f * cosf(theta);
        out[k].im = 0.70f * sinf(theta);
      }
    }
    break;
  }

  case 15: {
    /* Lightning bolt: a 7-point zigzag. The sharpest, slowest-to-rebuild
     * shape here — the bar is still climbing past 60 arms. */
    static const float lx[] = {0.00f,  0.55f, -0.20f, 0.30f,
                               -0.40f, 0.20f, 0.00f};
    static const float ly[] = {1.00f,  0.40f,  0.10f, -0.30f,
                               -0.60f, -1.00f, 0.00f};
    sample_poly(lx, ly, 7, out, N);
    break;
  }

  case 16: {
    /* Lissajous 11:7 — looks like a screen full of tangled curves, yet
     * it's still just two tones: four arms rebuild it, same as the 3:2.
     * A busy picture doesn't mean a complicated shape. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = sinf(11.f * t + (float)M_PI / 4.f);
      out[k].im = sinf(7.f * t);
    }
    break;
  }

  case 17: {
    /* Beaded circle: a plain circle with a small fast wobble added,
     * like 8 beads on a necklace. Unlike the gear, the wobble adds in
     * x and y directly, so it stays simple — four arms rebuild it. */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      out[k].re = cosf(t) + 0.25f * cosf(8.f * t);
      out[k].im = sinf(t) + 0.25f * sinf(8.f * t);
    }
    break;
  }

  case 18: {
    /* Two spirographs added together. Each needs two arms, so the sum
     * needs four — adding shapes just adds their arms. */
    Cplx tmp[N_SAMPLES];
    sample_hypotrochoid(4, 2, 2.f, out, N);
    sample_hypotrochoid(6, 3, 2.f, tmp, N);
    for (int k = 0; k < N; k++) {
      out[k].re = 0.5f * (out[k].re + tmp[k].re);
      out[k].im = 0.5f * (out[k].im + tmp[k].im);
    }
    break;
  }

  case 19: {
    /* A spiral that winds out, then jumps straight back to the centre
     * to close the loop. That snap-back acts like a sharp corner, so
     * even though the spiral itself looks smooth, it rebuilds slowly. */
    int spiral_end = N * 9 / 10; /* first 90% of the points draw the spiral */
    for (int k = 0; k < spiral_end; k++) {
      float t = 4.f * (float)M_PI * (float)k / (float)spiral_end;
      float r = 0.95f * t / (4.f * (float)M_PI);
      out[k].re = r * cosf(t);
      out[k].im = r * sinf(t);
    }
    float end_re = out[spiral_end - 1].re;
    float end_im = out[spiral_end - 1].im;
    for (int k = spiral_end; k < N; k++) {
      float frac = (float)(k - spiral_end) / (float)(N - spiral_end);
      out[k].re = end_re * (1.f - frac);
      out[k].im = end_im * (1.f - frac);
    }
    break;
  }

  case 20: {
    /* Torus knot (2, 5): a knotted loop flattened to 2-D — a 5-petal
     * ripple riding on a 2-turn carrier. A few arms rebuild it.
     *   x(t) = (2 + cos(5t)) * cos(2t) / 3
     *   y(t) = (2 + cos(5t)) * sin(2t) / 3 */
    for (int k = 0; k < N; k++) {
      float t = 2.f * (float)M_PI * (float)k / (float)N;
      float r = 2.f + cosf(5.f * t);
      out[k].re = r * cosf(2.f * t) / 3.f;
      out[k].im = r * sinf(2.f * t) / 3.f;
    }
    break;
  }
  }
}

/* ── §7  shape tiers — how fast each shape rebuilds ── */

/*
 * Every shape is tagged Fast, Med, or Slow by how quickly the energy bar
 * fills as you add arms, and the HUD shows it (e.g. "[Fast 5/9]") so you
 * can pick a shape knowing what to expect:
 *   Fast — basically done by ~6 arms (smooth curves)
 *   Med  — most of the way by ~10 arms, slow finish (cusped/compound)
 *   Slow — needs 50+ arms to look right (sharp corners)
 */
typedef enum { TIER_FAST = 0, TIER_MED, TIER_SLOW, TIER_COUNT } Tier;

static const Tier k_shape_tier[N_SHAPES] = {
    /*  0 Square           */ TIER_SLOW,
    /*  1 Arrow            */ TIER_SLOW,
    /*  2 Star-7           */ TIER_MED,
    /*  3 Cardioid         */ TIER_FAST,
    /*  4 Lissajous 3:2    */ TIER_FAST,
    /*  5 Rose r=cos(2t)   */ TIER_MED,
    /*  6 Lissajous 5:4    */ TIER_FAST,
    /*  7 Lissajous 7:3    */ TIER_FAST,
    /*  8 Hypotrochoid 5:3 */ TIER_FAST,
    /*  9 Hypotrochoid 7:4 */ TIER_FAST,
    /* 10 Epitrochoid 3:1  */ TIER_FAST,
    /* 11 Deltoid          */ TIER_MED,
    /* 12 Hypocycloid 5    */ TIER_MED,
    /* 13 Gear 8-tooth     */ TIER_SLOW,
    /* 14 Crescent         */ TIER_MED,
    /* 15 Lightning        */ TIER_SLOW,
    /* 16 Lissajous 11:7   */ TIER_FAST,
    /* 17 Beaded circle    */ TIER_MED,
    /* 18 Compound spiro   */ TIER_FAST,
    /* 19 Closed spiral    */ TIER_SLOW,
    /* 20 Torus knot 2:5   */ TIER_MED,
};

static const char *k_tier_name[TIER_COUNT] = {"Fast", "Med ", "Slow"};

/* Works out what the HUD's "[Fast 5/9]" chip should say: this shape's
 * tier, its place within that tier, and how many shapes share it. */
static void shape_tier_position(int shape_idx, const char **out_tier_name,
                                int *out_pos, int *out_total) {
  Tier t = k_shape_tier[shape_idx];
  *out_tier_name = k_tier_name[t];
  *out_pos = 0;
  *out_total = 0;
  for (int i = 0; i < N_SHAPES; i++) {
    if (k_shape_tier[i] != t)
      continue;
    (*out_total)++;
    if (i <= shape_idx)
      *out_pos = *out_total;
  }
}

/* ── §8  the DFT — find the spinning arms hidden in the shape ── */

/*
 * The discrete Fourier transform: turns the N shape points into N
 * frequency bins, each one telling us how strong a particular spin rate
 * is. We run it just once per shape change, not every frame.
 *
 * The one trick: instead of calling cos/sin inside the inner loop, we
 * compute the per-step rotation once and keep multiplying by it. Same
 * answer, far fewer trig calls. (See dft_helloworld.c for the plain
 * version.)
 */
static void compute_dft(const Cplx *in, Cplx *out, int N) {
  for (int n = 0; n < N; n++) {
    float twiddle_step_re = cosf(-2.f * (float)M_PI * (float)n / (float)N);
    float twiddle_step_im = sinf(-2.f * (float)M_PI * (float)n / (float)N);
    float twiddle_re = 1.f, twiddle_im = 0.f;
    float acc_re = 0.f, acc_im = 0.f;

    for (int k = 0; k < N; k++) {
      /* add this sample, rotated by the running angle */
      acc_re += in[k].re * twiddle_re - in[k].im * twiddle_im;
      acc_im += in[k].re * twiddle_im + in[k].im * twiddle_re;

      /* nudge the running angle on by one step */
      float next_re =
          twiddle_re * twiddle_step_re - twiddle_im * twiddle_step_im;
      twiddle_im = twiddle_re * twiddle_step_im + twiddle_im * twiddle_step_re;
      twiddle_re = next_re;
    }
    out[n].re = acc_re;
    out[n].im = acc_im;
  }
}

/* ── §9  the arm list — one spinning arm per frequency ── */

/*
 * One rotating arm of the chain, read straight out of one DFT bin.
 *   amp   — arm length (how big a circle this arm sweeps)
 *   phase — where on its circle the arm starts
 *   freq  — how many turns it makes per full shape trace; the sign says
 *           which way it spins.
 */
typedef struct {
  float amp;
  float phase;
  int freq;
} Epicycle;

/* Sort helper: longest arm first. Putting the big arms up front is what
 * lets "the first M arms" always be the M that capture the most shape. */
static int epic_cmp(const void *a, const void *b) {
  float da = ((const Epicycle *)a)->amp;
  float db = ((const Epicycle *)b)->amp;
  return (da < db) - (da > db);
}

static Epicycle g_epicycle_table[N_SAMPLES]; /* all arms, longest first */
static int g_total_epicycle_count = 0;       /* how many arms exist     */
static int g_active_epicycle_count = 0;      /* how many are switched on */

/* g_cumulative_energy_fraction[k]: with the first k+1 arms turned on,
 * what fraction of the whole shape have we captured (0 to 1)? This is
 * exactly what the energy bar shows. We fill it in once per shape so the
 * bar is a cheap lookup every frame. */
static float g_cumulative_energy_fraction[N_SAMPLES];
static float g_total_signal_energy; /* the "100%" we measure against */

/* The shape's original points, kept around to draw the faint target. */
static Cplx g_resampled_source[N_SAMPLES];

/* Turn the chosen shape into the sorted arm list plus the energy lookup.
 * Called whenever the shape changes (startup, n/p, reset) — this is
 * where the one-time DFT cost lives, kept out of the per-frame path. */
static void build_epicycles(int shape_idx) {
  Cplx dft[N_SAMPLES];
  sample_path(shape_idx, g_resampled_source, N_SAMPLES);
  compute_dft(g_resampled_source, dft, N_SAMPLES);

  float inv_N = 1.f / (float)N_SAMPLES;
  for (int n = 0; n < N_SAMPLES; n++) {
    float re = dft[n].re, im = dft[n].im;
    int f = (n <= N_SAMPLES / 2) ? n : n - N_SAMPLES; /* fold the top half to negative spins */
    g_epicycle_table[n] =
        (Epicycle){sqrtf(re * re + im * im) * inv_N, atan2f(im, re), f};
  }
  qsort(g_epicycle_table, N_SAMPLES, sizeof(Epicycle), epic_cmp);
  g_total_epicycle_count = N_SAMPLES;

  /* total "size" of the shape: add up every arm's squared length */
  g_total_signal_energy = 0.f;
  for (int n = 0; n < N_SAMPLES; n++)
    g_total_signal_energy += g_epicycle_table[n].amp * g_epicycle_table[n].amp;

  /* running total as a fraction of that, arm by arm — the bar's data */
  float acc = 0.f;
  for (int n = 0; n < N_SAMPLES; n++) {
    acc += g_epicycle_table[n].amp * g_epicycle_table[n].amp;
    g_cumulative_energy_fraction[n] =
        (g_total_signal_energy > 0.f) ? acc / g_total_signal_energy : 0.f;
  }
}

/* ── §10  trail — the fading path the arm tip leaves behind ── */

/*
 * A fixed-size loop of the last TRAIL_LEN tip positions. write_head is
 * where the next point goes; once full it wraps and overwrites the
 * oldest. filled_count tracks how many of the slots are real yet.
 */
typedef struct {
  float pixel_x[TRAIL_LEN];
  float pixel_y[TRAIL_LEN];
  int write_head;
  int filled_count;
} Trail;

static void trail_push(Trail *t, float pixel_x, float pixel_y) {
  t->pixel_x[t->write_head] = pixel_x;
  t->pixel_y[t->write_head] = pixel_y;
  t->write_head = (t->write_head + 1) % TRAIL_LEN;
  if (t->filled_count < TRAIL_LEN)
    t->filled_count++;
}

static void trail_clear(Trail *t) {
  /* just forget the old points; no need to wipe the arrays */
  t->write_head = 0;
  t->filled_count = 0;
}

/* ── §11  scene state — everything the animation tracks ── */

static int g_screen_rows, g_screen_cols;
static float g_pivot_pixel_x, g_pivot_pixel_y; /* where the chain starts   */
static float g_pixel_scale;             /* turns shape units into pixels   */
static float g_animation_phase_radians; /* the angle that drives the spin  */
static int g_auto_add_counter;          /* frames since last auto-added arm */
static bool g_simulation_paused;
static bool g_auto_add_enabled;
static bool g_show_orbit_circles;
static bool g_show_ghost_overlay;
static int g_active_shape_index;
static Trail g_tip_trail;

/* The arm-chain joints in pixel space: joint[0] is the centre, and each
 * joint[i+1] is where arm i ends (and arm i+1 begins). The last one is
 * the pen tip that draws the shape. */
static float g_joint_pixel_x[N_SAMPLES + 1];
static float g_joint_pixel_y[N_SAMPLES + 1];

/* The one place we turn pixel coordinates into terminal cells. */
static int px_cx(float pixel_x) {
  return (int)(pixel_x / (float)CELL_W + 0.5f);
}
static int px_cy(float pixel_y) {
  return (int)(pixel_y / (float)CELL_H + 0.5f);
}

/* ── §12  the arm chain — where each joint sits right now ── */

/* Start at the centre and follow the arms one by one: each arm points off
 * at its current angle and lengthens the chain. We save every joint so the
 * drawing code can just connect the dots, and so we compute it only once
 * per frame. */
static void scene_compute_chain(void) {
  float x = g_pivot_pixel_x, y = g_pivot_pixel_y;
  g_joint_pixel_x[0] = x;
  g_joint_pixel_y[0] = y;
  for (int i = 0; i < g_active_epicycle_count; i++) {
    float ang = (float)g_epicycle_table[i].freq * g_animation_phase_radians +
                g_epicycle_table[i].phase;
    float r = g_epicycle_table[i].amp * g_pixel_scale;
    x += r * cosf(ang);
    y += r * sinf(ang);
    g_joint_pixel_x[i + 1] = x;
    g_joint_pixel_y[i + 1] = y;
  }
}

/* ── §13  setting up and stepping the scene ── */

/* Start fresh on a new shape: one arm on, animation at the top, empty
 * trail, then rebuild the arm list for that shape. */
static void scene_reset(int shape_idx) {
  g_active_shape_index = shape_idx;
  g_animation_phase_radians = 0.f;
  g_auto_add_counter = 0;
  g_active_epicycle_count = 1;
  trail_clear(&g_tip_trail);
  build_epicycles(g_active_shape_index);
  scene_compute_chain();
}

/* Size the scene to the terminal and switch the default toggles on. */
static void scene_init(int rows, int cols) {
  g_screen_rows = rows;
  g_screen_cols = cols;
  g_pivot_pixel_x = (float)(cols * CELL_W) * 0.5f;
  g_pivot_pixel_y = (float)(rows * CELL_H) * 0.5f;
  float min_pixels = fminf((float)(cols * CELL_W), (float)(rows * CELL_H));
  g_pixel_scale = min_pixels * SHAPE_SCALE;
  g_simulation_paused = false;
  g_auto_add_enabled = true;
  g_show_orbit_circles = true;
  g_show_ghost_overlay = true;
  scene_reset(0);
}

/* One frame of motion: maybe add an arm, spin a little, redraw the chain. */
static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* every so often, switch on one more arm */
  if (g_auto_add_enabled && g_active_epicycle_count < g_total_epicycle_count) {
    g_auto_add_counter++;
    if (g_auto_add_counter >= AUTO_ADD_FRAMES) {
      g_auto_add_counter = 0;
      g_active_epicycle_count++;
    }
  }

  /* spin forward; when we come full circle, start the trail over */
  g_animation_phase_radians += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
  if (g_animation_phase_radians >= 2.f * (float)M_PI) {
    g_animation_phase_radians -= 2.f * (float)M_PI;
    trail_clear(&g_tip_trail);
  }

  /* move the arms and drop the new tip position into the trail */
  scene_compute_chain();
  trail_push(&g_tip_trail, g_joint_pixel_x[g_active_epicycle_count],
             g_joint_pixel_y[g_active_epicycle_count]);
}

/* ── §14  drawing helpers — straight lines and orbit rings ── */

/* Draw a straight line between two cells, picking a glyph that matches its
 * slope: dash for flat, bar for steep, slash for diagonal. */
static void draw_line_seg(int x0, int y0, int x1, int y1, attr_t attr) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  for (;;) {
    if (x0 >= 0 && x0 < g_screen_cols && y0 >= 0 && y0 < g_screen_rows) {
      int e2 = 2 * err;
      bool bx = e2 > -dy, by = e2 < dx;
      chtype ch = (bx && by) ? (sx == sy ? '\\' : '/') : bx ? '-' : '|';
      attron(attr);
      mvaddch(y0, x0, ch);
      attroff(attr);
    }
    if (x0 == x1 && y0 == y1)
      break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }
}

/* Draw the dotted circle one arm sweeps. It comes out as an ellipse on
 * screen because terminal cells are twice as tall as they are wide. */
static void draw_orbit(float piv_px, float piv_py, float r_px) {
  if (r_px < (float)CELL_W * 0.5f)
    return; /* too small to see; skip it */
  float rx = r_px / (float)CELL_W;
  float ry = r_px / (float)CELL_H;
  int pcx = px_cx(piv_px), pcy = px_cy(piv_py);
  int steps = (int)(2.f * (float)M_PI * fmaxf(rx, ry)) + 4;
  for (int i = 0; i < steps; i++) {
    float a = 2.f * (float)M_PI * (float)i / (float)steps;
    int x = pcx + (int)(rx * cosf(a) + 0.5f);
    int y = pcy + (int)(ry * sinf(a) + 0.5f);
    if (x >= 0 && x < g_screen_cols && y >= 0 && y < g_screen_rows) {
      attron(COLOR_PAIR(CP_CIRCLE));
      mvaddch(y, x, '.');
      attroff(COLOR_PAIR(CP_CIRCLE));
    }
  }
}

/* ── §15  drawing the scene — painted back to front ── */

/* Painted in order so the most useful stuff ends up on top: the faint
 * target first, then orbit guides, the trail, the arms, and the markers. */
static void scene_draw(void) {
  /* faint ':' dots showing the real shape, so you can judge the fit
   * (every other point, to keep it from looking solid) */
  if (g_show_ghost_overlay) {
    attron(COLOR_PAIR(CP_GHOST));
    for (int k = 0; k < N_SAMPLES; k += 2) {
      float px = g_pivot_pixel_x + g_resampled_source[k].re * g_pixel_scale;
      float py = g_pivot_pixel_y + g_resampled_source[k].im * g_pixel_scale;
      int cx = px_cx(px), cy = px_cy(py);
      if (cx >= 0 && cx < g_screen_cols && cy >= 0 && cy < g_screen_rows)
        mvaddch(cy, cx, ':');
    }
    attroff(COLOR_PAIR(CP_GHOST));
  }

  /* faint guide rings for the few biggest arms */
  if (g_show_orbit_circles) {
    int nc = g_active_epicycle_count < N_CIRCLES ? g_active_epicycle_count
                                                 : N_CIRCLES;
    for (int i = 0; i < nc; i++)
      draw_orbit(g_joint_pixel_x[i], g_joint_pixel_y[i],
                 g_epicycle_table[i].amp * g_pixel_scale);
  }

  /* the tip's path, brightest where it's newest */
  {
    int draw = g_tip_trail.filled_count;
    int start = (g_tip_trail.write_head - draw + TRAIL_LEN) % TRAIL_LEN;
    for (int i = 0; i < draw; i++) {
      int idx = (start + i) % TRAIL_LEN;
      int cx = px_cx(g_tip_trail.pixel_x[idx]);
      int cy = px_cy(g_tip_trail.pixel_y[idx]);
      if (cx < 0 || cx >= g_screen_cols || cy < 0 || cy >= g_screen_rows)
        continue;
      float age = (float)i / (float)draw; /* 0 = oldest, 1 = newest */
      int cp = age > 0.70f ? CP_TRAIL_1 : age > 0.35f ? CP_TRAIL_2 : CP_TRAIL_3;
      attron(COLOR_PAIR(cp) | A_BOLD);
      mvaddch(cy, cx, '*');
      attroff(COLOR_PAIR(cp) | A_BOLD);
    }
  }

  /* the arms themselves, coloured by length */
  for (int i = 0; i < g_active_epicycle_count; i++) {
    float r_px = g_epicycle_table[i].amp * g_pixel_scale;
    int cp = r_px > g_pixel_scale * 0.10f   ? CP_ARM_HI
             : r_px > g_pixel_scale * 0.02f ? CP_ARM_MID
                                            : CP_ARM_LO;
    draw_line_seg(px_cx(g_joint_pixel_x[i]), px_cy(g_joint_pixel_y[i]),
                  px_cx(g_joint_pixel_x[i + 1]), px_cy(g_joint_pixel_y[i + 1]),
                  COLOR_PAIR(cp) | A_BOLD);
  }

  /* '@' on the pen tip that's drawing the shape */
  {
    int bx = px_cx(g_joint_pixel_x[g_active_epicycle_count]);
    int by = px_cy(g_joint_pixel_y[g_active_epicycle_count]);
    if (bx >= 0 && bx < g_screen_cols && by >= 0 && by < g_screen_rows) {
      attron(COLOR_PAIR(CP_BOB) | A_BOLD);
      mvaddch(by, bx, '@');
      attroff(COLOR_PAIR(CP_BOB) | A_BOLD);
    }
  }

  /* '+' on the fixed centre the chain hangs from */
  {
    int pcx = px_cx(g_pivot_pixel_x), pcy = px_cy(g_pivot_pixel_y);
    if (pcx >= 0 && pcx < g_screen_cols && pcy >= 0 && pcy < g_screen_rows) {
      attron(COLOR_PAIR(CP_PIVOT) | A_BOLD);
      mvaddch(pcy, pcx, '+');
      attroff(COLOR_PAIR(CP_PIVOT) | A_BOLD);
    }
  }
}

/* ── §16  the overlay — status line, energy bar, key hints ── */

/* The text and bar laid over the animation: a status line top-right, the
 * energy bar on the second row, and the key reminders along the bottom. */
static void scene_draw_hud(void) {
  float efrac = (g_active_epicycle_count > 0 && g_total_signal_energy > 0.f)
                    ? g_cumulative_energy_fraction[g_active_epicycle_count - 1]
                    : 0.f;

  /* status line: shape, tier, arm count, captured %, theme, toggles */
  {
    const char *tier_name;
    int tier_pos = 0, tier_total = 0;
    shape_tier_position(g_active_shape_index, &tier_name, &tier_pos,
                        &tier_total);

    char buf[220];
    snprintf(buf, sizeof buf,
             " FourierShapes  %s [%s %d/%d]  arms:%3d/%d  energy:%5.1f%%  "
             "thm:%s  %s%s%s ",
             k_shape_names[g_active_shape_index], tier_name, tier_pos,
             tier_total, g_active_epicycle_count, g_total_epicycle_count,
             efrac * 100.f, k_themes[g_active_theme_index].name,
             g_auto_add_enabled ? "auto " : "       ",
             g_show_orbit_circles ? "circles " : "        ",
             g_show_ghost_overlay ? "ghost" : "     ");
    int x = g_screen_cols - (int)strlen(buf);
    if (x < 0)
      x = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, x, "%s", buf);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
  }

  /* PAUSED badge, top-left, only while paused */
  if (g_simulation_paused) {
    attron(COLOR_PAIR(CP_HUD) | A_BOLD | A_REVERSE);
    mvprintw(0, 0, " PAUSED ");
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD | A_REVERSE);
  }

  /* the headline: how much of the shape the active arms capture */
  {
    int bar_col = 1, bar_row = 1, bar_w = 50;
    if (bar_col + bar_w + 16 >= g_screen_cols)
      bar_w = g_screen_cols - bar_col - 17;
    if (bar_w < 12)
      bar_w = 12;

    /* red while rough, yellow once recognisable, green when nearly done */
    int bar_pair = (efrac >= 0.80f)   ? CP_ENERGY
                   : (efrac >= 0.50f) ? CP_ENERGY_MID
                                      : CP_ENERGY_LO;

    /* brackets at the ends, '=' for the filled part, '-' for the rest */
    attron(COLOR_PAIR(CP_HUD));
    mvaddch(bar_row, bar_col, '[');
    mvaddch(bar_row, bar_col + bar_w - 1, ']');
    attroff(COLOR_PAIR(CP_HUD));

    int filled = (int)(efrac * (float)(bar_w - 2));
    attron(COLOR_PAIR(bar_pair) | A_BOLD);
    for (int i = 0; i < filled; i++)
      mvaddch(bar_row, bar_col + 1 + i, '=');
    attroff(COLOR_PAIR(bar_pair) | A_BOLD);

    attron(COLOR_PAIR(CP_HUD));
    for (int i = filled; i < bar_w - 2; i++)
      mvaddch(bar_row, bar_col + 1 + i, '-');
    mvprintw(bar_row, bar_col + bar_w + 1, "Parseval power");
    attroff(COLOR_PAIR(CP_HUD));
  }

  /* key reminders along the bottom */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(g_screen_rows - 1, 0,
           " q:quit  spc:pause  n/p:shape  +/-:arms  r:reset  a:auto  "
           "c:circles  g:ghost  t/T:theme ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §17  terminal setup and screen flush ── */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1); /* stop ncurses peeking at input mid-draw, which tears */
  color_init();
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §18  the program — signals, the main loop, and the keys ── */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int s) {
  if (s == SIGINT || s == SIGTERM)
    g_should_quit = 1;
  if (s == SIGWINCH)
    g_resize_pending = 1;
}

static void cleanup(void) { endwin(); }

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  screen_init();

  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  scene_init(rows, cols);

  long long last_ns = clock_ns();

  while (!g_should_quit) {

    /* terminal was resized: rebuild everything for the new size */
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, rows, cols);
      scene_init(rows, cols);
      last_ns = clock_ns();
      continue;
    }

    /* handle a keypress, if any */
    int ch = getch();
    switch (ch) {
    case 'q':
    case 'Q':
    case 27:
      g_should_quit = 1;
      break;
    case ' ':
      g_simulation_paused = !g_simulation_paused;
      break;
    case 'n':
    case 'N':
      scene_reset((g_active_shape_index + 1) % N_SHAPES);
      break;
    case 'p':
    case 'P':
      scene_reset((g_active_shape_index + N_SHAPES - 1) % N_SHAPES);
      break;
    case '+':
    case '=':
      if (g_active_epicycle_count < g_total_epicycle_count)
        g_active_epicycle_count++;
      break;
    case '-':
      if (g_active_epicycle_count > 1) {
        g_active_epicycle_count--;
        trail_clear(&g_tip_trail);
      }
      break;
    case 'r':
    case 'R':
      scene_reset(g_active_shape_index);
      break;
    case 'a':
    case 'A':
      g_auto_add_enabled = !g_auto_add_enabled;
      break;
    case 'c':
    case 'C':
      g_show_orbit_circles = !g_show_orbit_circles;
      break;
    case 'g':
    case 'G':
      g_show_ghost_overlay = !g_show_ghost_overlay;
      break;
    case 't':
      g_active_theme_index = (g_active_theme_index + 1) % N_THEMES;
      apply_theme(g_active_theme_index);
      break;
    case 'T':
      g_active_theme_index = (g_active_theme_index + N_THEMES - 1) % N_THEMES;
      apply_theme(g_active_theme_index);
      break;
    default:
      break;
    }

    /* advance one frame */
    long long now_ns = clock_ns();
    long long dt = now_ns - last_ns;
    last_ns = now_ns;
    if (dt > 100000000LL)
      dt = 100000000LL; /* don't let a long stall jump the animation */
    (void)dt;

    scene_tick();

    /* repaint */
    erase();
    scene_draw();
    scene_draw_hud();
    screen_present();

    /* wait out the rest of the frame so we hold 30 fps */
    clock_sleep_ns(RENDER_NS - (clock_ns() - now_ns));
  }

  return 0;
}
