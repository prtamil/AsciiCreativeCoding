/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * epicycles.c — draw any closed 2-D curve as a chain of rotating arms.
 * Each arm is a spinning wheel; pin the next, smaller wheel to the
 * previous one's tip and the last tip traces a shape (heart, star,
 * butterfly...).  The Fourier transform works out each arm's size,
 * speed, and starting angle from the chosen curve.
 *
 * Sister file: signal/fft_vis.c — the same maths in 1-D.
 * Background: 3Blue1Brown, "But what is a Fourier series?" (YouTube).
 */

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

/* §1  config — every constant and enum lives here */

/* §1.1  how many samples and how fast it animates */

#define DFT_SAMPLE_COUNT 256 /* samples taken, and the most arms we can draw */
#define ANIMATION_FRAMES_PER_CYCLE                                             \
  360                                /* one full trace of the curve = 12 s */
#define AUTO_GROW_INTERVAL_FRAMES 12 /* add one arm every 12 frames */

/* §1.2  how much of the tip's recent path we remember */

#define TIP_TRAIL_LENGTH 600 /* about 1.5 loops of recent tip path */

/* §1.3  frame-rate cap */

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_FPS_CAP 30
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS_CAP)

/* §1.4  how big one terminal cell is, in the chain's own fine units.
 * Most monospace fonts draw a cell about twice as tall as wide, so we
 * give it 8 units across and 16 down.  The chain math runs in these
 * units; we only round to (col, row) at draw time. */

#define CELL_PIXEL_WIDTH 8
#define CELL_PIXEL_HEIGHT 16

/* §1.5  sizes and cutoffs for what gets drawn */

#define SHAPE_PIXEL_SCALE_FRAC 0.38f /* curve fills this fraction of the screen */
#define ORBIT_DRAW_COUNT_MAX 6       /* draw at most this many orbit guides */
#define ORBIT_DRAW_MIN_RADIUS_PX ((float)CELL_PIXEL_WIDTH * 0.5f)
#define ARM_RADIUS_THRESHOLD_LARGE 0.10f /* fraction of the overall size */
#define ARM_RADIUS_THRESHOLD_MID 0.02f

/* §1.6  colour-pair names.  ncurses keeps pair 0 for itself, so ours
 * start at 1.  §5 binds these to real colours.  HUD and HINT stay the
 * same bright yellow/cyan in every theme so the status bar is readable. */

enum {
  PAIR_ARM_LARGE = 1, /* biggest arms (top of sorted table)       */
  PAIR_ARM_MID,       /* medium arms                              */
  PAIR_ARM_SMALL,     /* small / tail arms                         */
  PAIR_ORBIT,         /* orbit guide ellipses                     */
  PAIR_TRAIL_NEW,     /* most recent tip-path points              */
  PAIR_TRAIL_MID,     /* middle-aged tip-path points              */
  PAIR_TRAIL_OLD,     /* oldest still-visible tip-path points     */
  PAIR_TIP_BOB,       /* the tip marker '@'                        */
  PAIR_PIVOT,         /* the pivot marker '+'                      */
  PAIR_HUD,           /* top status line — theme-invariant        */
  PAIR_HINT,          /* bottom key hint — theme-invariant         */
};

/* §1.7  how many shapes in the catalogue (table in §6) */

#define SHAPE_COUNT 20

/* §1.8  how many colour themes (table in §4) */

#define THEME_COUNT 6

/* §2  clock — a forward-only timer plus a sleep, for frame pacing.
 * We use the monotonic clock (not wall-clock time) so a system clock
 * adjustment can't make the frame pacer hiccup. */

static int64_t clock_now_ns(void) {
  /* The main loop subtracts two of these readings to measure how long
   * a frame took and how long to sleep before the next one. */
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  /* Sit still until the next frame is due — without this the loop
   * would peg a CPU core at 100%. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / NS_PER_SEC, ns % NS_PER_SEC};
  nanosleep(&ts, NULL);
}

/* §3  complex — a tiny complex-number library.
 * A complex number is just a 2-D point that knows how to rotate: adding
 * two of them adds the points; multiplying rotates one by the other's
 * angle and scales by its length.  That makes a rotating arm a single
 * multiply.  We roll our own 2-float struct instead of <complex.h> so
 * it prints cleanly in a debugger.  Five operations are all we need:
 * add, multiply, point-on-the-unit-circle, length, angle. */

typedef struct {
  float re; /* real part      */
  float im; /* imaginary part */
} ComplexNumber;

static inline ComplexNumber complex_make(float re, float im) {
  return (ComplexNumber){re, im};
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re + b.re, a.im + b.im);
}

static inline ComplexNumber complex_mul(ComplexNumber a, ComplexNumber b) {
  /* Multiplying rotates a by b's angle and scales it by b's length. */
  return complex_make(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
}

static inline ComplexNumber complex_exp_unit(float theta_radians) {
  /* The point on the unit circle at angle theta — i.e. (cos, sin)
   * packed as a complex number so we can do arithmetic with it. */
  return complex_make(cosf(theta_radians), sinf(theta_radians));
}

static inline float complex_magnitude(ComplexNumber z) {
  return sqrtf(z.re * z.re + z.im * z.im);
}

static inline float complex_argument(ComplexNumber z) {
  /* atan2 (not plain atan) so the angle comes out right in all four
   * quadrants. */
  return atan2f(z.im, z.re);
}

/* §4  themes — six colour palettes the user cycles with t / T.
 * Each theme is one row of 256-colour codes, one code per animation
 * pair.  Every code sits in the bright half of the palette: codes in
 * the dark end (16-23 in the colour cube, 232-239 in the grey ramp)
 * look black on a default background and would vanish.  HUD and HINT
 * are deliberately left out so the status bar stays readable in every
 * theme.  To add a theme: bump THEME_COUNT and add a row. */

typedef struct {
  const char *display_name; /* shown in the status bar */
  short arm_large_color;    /* the biggest, most important arms */
  short arm_mid_color;      /* medium arms */
  short arm_small_color;    /* the tiny corrective arms at the tail */
  short orbit_color;        /* faint orbit guides */
  short trail_new_color;    /* freshest dots of the tip's path */
  short trail_mid_color;    /* middle-aged trail dots */
  short trail_old_color;    /* oldest still-visible trail dots */
  short tip_color;          /* the pen marker at the very tip */
  short pivot_color;        /* the marker where the chain is anchored */
} ThemePalette;

static const ThemePalette theme_table[THEME_COUNT] = {
    /* Classic — bright white arms, gold/orange/red trail, yellow pivot. */
    {"Classic ", 231, 51, 246, 244, 226, 208, 196, 231, 220},
    /* Ocean — pale blues + cyan, gold trail for warm contrast. */
    {"Ocean   ", 195, 117, 39, 31, 226, 220, 208, 195, 51},
    /* Sunset — peach-to-red palette, magenta trail. */
    {"Sunset  ", 223, 215, 209, 174, 201, 213, 198, 231, 220},
    /* Matrix — green spectrum on default-black. */
    {"Matrix  ", 120, 46, 34, 28, 154, 76, 34, 231, 226},
    /* Mono — high-contrast greyscale. */
    {"Mono    ", 231, 250, 244, 240, 252, 247, 242, 231, 246},
    /* Neon — purple/cyan/pink, club-flyer aesthetic. */
    {"Neon    ", 213, 51, 207, 240, 226, 207, 197, 231, 226},
};

/* §5  colors — set up the colour pairs and switch themes */

static void apply_theme(int theme_index) {
  /* Point the nine animation pairs at a theme's colours, leaving HUD
   * and HINT alone.  One place does this so startup and the t/T keys
   * share the same logic. */
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const ThemePalette *t = &theme_table[theme_index];

  if (COLORS >= 256) {
    init_pair(PAIR_ARM_LARGE, t->arm_large_color, -1);
    init_pair(PAIR_ARM_MID, t->arm_mid_color, -1);
    init_pair(PAIR_ARM_SMALL, t->arm_small_color, -1);
    init_pair(PAIR_ORBIT, t->orbit_color, -1);
    init_pair(PAIR_TRAIL_NEW, t->trail_new_color, -1);
    init_pair(PAIR_TRAIL_MID, t->trail_mid_color, -1);
    init_pair(PAIR_TRAIL_OLD, t->trail_old_color, -1);
    init_pair(PAIR_TIP_BOB, t->tip_color, -1);
    init_pair(PAIR_PIVOT, t->pivot_color, -1);
  } else {
    /* On an 8-colour terminal there aren't enough colours for themes
     * to matter; fall back to one legible set. */
    init_pair(PAIR_ARM_LARGE, COLOR_WHITE, -1);
    init_pair(PAIR_ARM_MID, COLOR_CYAN, -1);
    init_pair(PAIR_ARM_SMALL, COLOR_WHITE, -1);
    init_pair(PAIR_ORBIT, COLOR_WHITE, -1);
    init_pair(PAIR_TRAIL_NEW, COLOR_YELLOW, -1);
    init_pair(PAIR_TRAIL_MID, COLOR_YELLOW, -1);
    init_pair(PAIR_TRAIL_OLD, COLOR_RED, -1);
    init_pair(PAIR_TIP_BOB, COLOR_WHITE, -1);
    init_pair(PAIR_PIVOT, COLOR_YELLOW, -1);
  }
}

static void colors_init(void) {
  /* Set the always-the-same HUD and HINT colours once, then load the
   * default theme.  Later theme switches go through apply_theme and
   * leave these two alone. */
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  apply_theme(0);
}

/* §6  shapes — 20 curves to trace, each a function of t in [0, 2pi).
 * Every curve is closed (its start and end meet) and sized to fit
 * inside the unit circle; §12 scales it up to the screen later.  The
 * first ten are smooth and need only a handful of arms; the last ten
 * have corners or cusps and need many more (and show the Gibbs
 * ringing described below).  The table at the end maps an index to a
 * name and a sampler; n / p step through it. */

/* §6.1  walk around a regular polygon's edge */

static void polygon_sample(float t, int n_sides, float *out_x, float *out_y) {
  /* March around the polygon at a steady pace: figure out which edge t
   * lands on, then slide along that edge between its two corners.  One
   * helper covers Triangle, Square, and Pentagon. */
  float normalised = t / (2.0f * (float)M_PI) * (float)n_sides;
  int segment = (int)floorf(normalised) % n_sides;
  float fraction = normalised - floorf(normalised);
  float angle_base = -(float)M_PI / 2.0f; /* start at the top */
  float step_angle = 2.0f * (float)M_PI / (float)n_sides;
  float angle_a = angle_base + (float)segment * step_angle;
  float angle_b = angle_base + (float)(segment + 1) * step_angle;
  float x_a = cosf(angle_a), y_a = sinf(angle_a);
  float x_b = cosf(angle_b), y_b = sinf(angle_b);
  *out_x = x_a + (x_b - x_a) * fraction;
  *out_y = y_a + (y_b - y_a) * fraction;
}

/* §6.2  star — same idea, but corners alternate near and far */

static void star_sample(float t, int n_points, float inner_ratio, float *out_x,
                        float *out_y) {
  /* Like polygon_sample, but every other corner sits closer to the
   * centre (at inner_ratio) to make the points of a star. */
  int total_segs = 2 * n_points;
  float normalised = t / (2.0f * (float)M_PI) * (float)total_segs;
  int segment = (int)floorf(normalised) % total_segs;
  float fraction = normalised - floorf(normalised);
  float angle_base = -(float)M_PI / 2.0f;
  float step_angle = (float)M_PI / (float)n_points;
  float angle_a = angle_base + (float)segment * step_angle;
  float angle_b = angle_base + (float)(segment + 1) * step_angle;
  float radius_a = (segment % 2 == 0) ? 1.00f : inner_ratio;
  float radius_b = (segment % 2 == 0) ? inner_ratio : 1.00f;
  float x_a = radius_a * cosf(angle_a), y_a = radius_a * sinf(angle_a);
  float x_b = radius_b * cosf(angle_b), y_b = radius_b * sinf(angle_b);
  *out_x = x_a + (x_b - x_a) * fraction;
  *out_y = y_a + (y_b - y_a) * fraction;
}

/* §6.3  smooth shapes (indices 0..9) */

static void shape_sample_circle(float t, float *out_x, float *out_y) {
  /* The simplest target — one arm draws it exactly.  Good for checking
   * the rest of the pipeline works. */
  *out_x = cosf(t);
  *out_y = sinf(t);
}

static void shape_sample_ellipse(float t, float *out_x, float *out_y) {
  /* Two arms draw it exactly: one spinning each way. */
  *out_x = cosf(t);
  *out_y = 0.6f * sinf(t);
}

static void shape_sample_cardioid(float t, float *out_x, float *out_y) {
  /* A heart shape, very smooth — the same curve that bounds the main
   * blob of the Mandelbrot set. */
  float r = (1.0f - cosf(t)) * 0.5f; /* halved so it fits the unit circle */
  *out_x = r * cosf(t);
  *out_y = r * sinf(t);
}

static void shape_sample_limacon(float t, float *out_x, float *out_y) {
  /* Pascal's "snail" — a cousin of the cardioid with a dimple. */
  float r = 0.5f + 0.5f * cosf(t);
  *out_x = r * cosf(t);
  *out_y = r * sinf(t);
}

static void shape_sample_heart(float t, float *out_x, float *out_y) {
  /* The well-known "Wolfram heart".  Its formula is already a sum of a
   * few cosine waves, so the transform recovers it cleanly. */
  *out_x = 16.0f * sinf(t) * sinf(t) * sinf(t) / 17.0f;
  *out_y = -(13.0f * cosf(t) - 5.0f * cosf(2.0f * t) - 2.0f * cosf(3.0f * t) -
             1.0f * cosf(4.0f * t)) /
           17.0f;
}

static void shape_sample_trefoil(float t, float *out_x, float *out_y) {
  /* A three-lobed knot, flattened onto the plane.  Smooth, but needs
   * around 20 arms to look right. */
  *out_x = (sinf(t) + 2.0f * sinf(2.0f * t)) / 3.2f;
  *out_y = (cosf(t) - 2.0f * cosf(2.0f * t)) / 3.2f;
}

static void shape_sample_figure_eight(float t, float *out_x, float *out_y) {
  /* A figure-eight (Gerono's lemniscate).  Both coordinates are pure
   * waves, so very few arms suffice. */
  *out_x = sinf(t);
  *out_y = sinf(t) * cosf(t);
}

static void shape_sample_lemniscate(float t, float *out_x, float *out_y) {
  /* Bernoulli's infinity-symbol — smoother through the centre crossing
   * than the figure-eight above. */
  float denom = 1.0f + sinf(t) * sinf(t);
  *out_x = cosf(t) / denom;
  *out_y = sinf(t) * cosf(t) / denom;
}

static void shape_sample_egg(float t, float *out_x, float *out_y) {
  /* An oval that's narrower at one end. */
  *out_x = cosf(t) * (1.0f + 0.18f * cosf(t)) * 0.85f;
  *out_y = sinf(t);
}

static void shape_sample_bean(float t, float *out_x, float *out_y) {
  /* A kidney-bean shape — smooth but lopsided, pinched on one side. */
  *out_x = cosf(t);
  *out_y = sinf(t) * (1.0f + cosf(t)) * 0.5f;
}

/* §6.4  sharp shapes (indices 10..19) — corners and cusps */

static void shape_sample_triangle(float t, float *x, float *y) {
  polygon_sample(t, 3, x, y);
}
static void shape_sample_square(float t, float *x, float *y) {
  polygon_sample(t, 4, x, y);
}
static void shape_sample_pentagon(float t, float *x, float *y) {
  polygon_sample(t, 5, x, y);
}

static void shape_sample_star(float t, float *x, float *y) {
  star_sample(t, 5, 0.42f, x, y);
}
static void shape_sample_star_7(float t, float *x, float *y) {
  star_sample(t, 7, 0.50f, x, y);
}
static void shape_sample_hexagram(float t, float *x, float *y) {
  /* Star of David — the inner radius is chosen so the two overlapping
   * triangles stay equilateral. */
  star_sample(t, 6, 0.577f, x, y);
}

static void shape_sample_astroid(float t, float *out_x, float *out_y) {
  /* A four-pointed star with sharp cusps on the axes — a good test of
   * how the transform handles pointed corners. */
  float c = cosf(t), s = sinf(t);
  *out_x = c * c * c;
  *out_y = s * s * s;
}

static void shape_sample_rose5(float t, float *out_x, float *out_y) {
  /* A five-petal flower; each petal comes to a sharp point at the
   * centre, which makes it a hard shape to draw. */
  float r = cosf(5.0f * t);
  *out_x = r * cosf(t);
  *out_y = r * sinf(t);
}

static void shape_sample_maltese(float t, float *out_x, float *out_y) {
  /* A disc bitten inward at the four points of the compass, bulging
   * back out at the diagonals. */
  float r = 1.0f - 0.7f * sinf(2.0f * t) * sinf(2.0f * t);
  *out_x = r * cosf(t);
  *out_y = r * sinf(t);
}

static void shape_sample_butterfly(float t, float *out_x, float *out_y) {
  /* Fay's butterfly curve (1989).  Its detail spreads across many
   * waves, so it deliberately needs a lot of arms to look right. */
  float envelope =
      expf(cosf(t)) - 2.0f * cosf(4.0f * t) - powf(sinf(t / 12.0f), 5.0f);
  *out_x = sinf(t) * envelope / 5.0f;
  *out_y = -cosf(t) * envelope / 5.0f;
}

/* §6.5  the catalogue: name + the function that draws each shape */

typedef struct {
  const char *display_name;                          /* shown in the HUD */
  void (*sampler)(float t, float *out_x, float *out_y); /* point at parameter t */
} ShapeEntry;

static const ShapeEntry shape_table[SHAPE_COUNT] = {
    /* 0..9   smooth  — a handful of arms is enough */
    {"Circle      ", shape_sample_circle},
    {"Ellipse     ", shape_sample_ellipse},
    {"Cardioid    ", shape_sample_cardioid},
    {"Limacon     ", shape_sample_limacon},
    {"Heart       ", shape_sample_heart},
    {"Trefoil     ", shape_sample_trefoil},
    {"Figure-8    ", shape_sample_figure_eight},
    {"Lemniscate  ", shape_sample_lemniscate},
    {"Egg         ", shape_sample_egg},
    {"Bean        ", shape_sample_bean},
    /* 10..19 sharp   — corners and cusps need 50+ arms */
    {"Triangle    ", shape_sample_triangle},
    {"Square      ", shape_sample_square},
    {"Pentagon    ", shape_sample_pentagon},
    {"Star (5pt)  ", shape_sample_star},
    {"Star (7pt)  ", shape_sample_star_7},
    {"Hexagram    ", shape_sample_hexagram},
    {"Astroid     ", shape_sample_astroid},
    {"Rose (5pet) ", shape_sample_rose5},
    {"Maltese     ", shape_sample_maltese},
    {"Butterfly   ", shape_sample_butterfly},
};

/* §7  sample — turn a curve into a list of points the transform can chew on.
 * We dot the curve at N evenly-spaced marks around its parameter and
 * pack each (x, y) into a complex number. */

static void sample_shape_into(int shape_index, ComplexNumber *out_samples,
                              int sample_count) {
  if (shape_index < 0 || shape_index >= SHAPE_COUNT)
    shape_index = 0;
  const ShapeEntry *entry = &shape_table[shape_index];
  for (int k = 0; k < sample_count; k++) {
    float t = (float)k / (float)sample_count * 2.0f * (float)M_PI;
    float x, y;
    entry->sampler(t, &x, &y);
    out_samples[k] = complex_make(x, y);
  }
}

/* §8  dft — the recipe machine: it reads the sampled curve and reports
 * how much of each whole-number rotation speed it contains.  Each
 * output bin becomes one arm's size, speed, and starting angle.
 *
 * It works by spinning a "probe" at each speed and seeing how strongly
 * the samples line up with it.  The naive way calls sin/cos for every
 * sample-and-speed pair; instead we step the probe forward by one
 * complex multiply each sample (the "twiddle" trick), which cuts the
 * sin/cos calls from thousands to a couple hundred.  Runs once per
 * shape change, never per frame, so the cost stays out of the hot
 * loop. */

static void dft_compute_with_twiddle(const ComplexNumber *input_samples,
                                     ComplexNumber *output_bins,
                                     int sample_count) {
  for (int n = 0; n < sample_count; n++) {
    float twiddle_angle = -2.0f * (float)M_PI * (float)n / (float)sample_count;
    ComplexNumber twiddle_step = complex_exp_unit(twiddle_angle);
    ComplexNumber twiddle_power = complex_make(1.0f, 0.0f);
    ComplexNumber accumulator = complex_make(0.0f, 0.0f);

    for (int k = 0; k < sample_count; k++) {
      accumulator = complex_add(accumulator,
                                complex_mul(input_samples[k], twiddle_power));
      twiddle_power = complex_mul(twiddle_power, twiddle_step);
    }
    output_bins[n] = accumulator;
  }
}

/* §9  epicycle — one arm boiled down to the three numbers the chain
 * walker needs: how long it is, where it starts, and how fast (and
 * which way) it spins.  Amplitude is already divided by the sample
 * count here, so §12 never has to think about that factor. */

typedef struct {
  float amplitude_normalised; /* arm length, roughly 0..1 in shape units */
  float phase_offset_radians; /* starting angle */
  int frequency_signed;       /* turns per loop; negative means clockwise */
} Epicycle;

static Epicycle epicycle_table[DFT_SAMPLE_COUNT];
static int total_epicycle_count = 0;
static int active_epicycle_count = 1;

static int epicycle_compare_descending(const void *a, const void *b) {
  /* Sorts longest arm first, so dialing the arm count up from 1 always
   * adds the next most important arm. */
  float amp_a = ((const Epicycle *)a)->amplitude_normalised;
  float amp_b = ((const Epicycle *)b)->amplitude_normalised;
  if (amp_a < amp_b)
    return 1;
  if (amp_a > amp_b)
    return -1;
  return 0;
}

/* §10  build — the whole offline pipeline in one call: sample the
 * curve, run the transform, turn each output bin into an arm, then sort
 * the arms longest-first.  Called at startup, on shape change (n/p),
 * and on reset (r) — never per frame. */

static void build_sorted_epicycle_table(int shape_index) {
  static ComplexNumber input_samples[DFT_SAMPLE_COUNT];
  static ComplexNumber dft_bins[DFT_SAMPLE_COUNT];

  sample_shape_into(shape_index, input_samples, DFT_SAMPLE_COUNT);
  dft_compute_with_twiddle(input_samples, dft_bins, DFT_SAMPLE_COUNT);

  float inverse_n = 1.0f / (float)DFT_SAMPLE_COUNT;
  for (int n = 0; n < DFT_SAMPLE_COUNT; n++) {
    Epicycle e;
    e.amplitude_normalised = complex_magnitude(dft_bins[n]) * inverse_n;
    e.phase_offset_radians = complex_argument(dft_bins[n]);
    /* The upper-half bins really stand for arms spinning the other way
     * (clockwise), so we relabel them as negative speeds.  Skip this and
     * every arm spins the same way and the shape comes out mirrored. */
    e.frequency_signed = (n <= DFT_SAMPLE_COUNT / 2) ? n : n - DFT_SAMPLE_COUNT;
    epicycle_table[n] = e;
  }
  qsort(epicycle_table, DFT_SAMPLE_COUNT, sizeof(Epicycle),
        epicycle_compare_descending);
  total_epicycle_count = DFT_SAMPLE_COUNT;
}

/* §11  trail — the glowing path the pen tip leaves behind.
 * A fixed-size circular buffer of recent tip positions: when it fills
 * up, each new point overwrites the oldest one, so it always holds the
 * last TIP_TRAIL_LENGTH points without ever growing. */

typedef struct {
  float pixel_x[TIP_TRAIL_LENGTH]; /* x of each stored tip position */
  float pixel_y[TIP_TRAIL_LENGTH]; /* y of each stored tip position */
  int write_head;   /* where the next point goes; wraps around */
  int filled_count; /* how many slots are in use, capped at the size */
} TipTrail;

static TipTrail tip_trail;

static void trail_push(TipTrail *trail, float pixel_x, float pixel_y) {
  trail->pixel_x[trail->write_head] = pixel_x;
  trail->pixel_y[trail->write_head] = pixel_y;
  trail->write_head = (trail->write_head + 1) % TIP_TRAIL_LENGTH;
  if (trail->filled_count < TIP_TRAIL_LENGTH)
    trail->filled_count++;
}

static void trail_clear(TipTrail *trail) {
  /* Just resets the counters — no need to wipe the stored points. */
  trail->write_head = 0;
  trail->filled_count = 0;
}

/* §12  chain — follow the arms tip to tip and record where the joints
 * land, so the drawing code can just connect dots. */

/* The corners of the chain.  joint[0] is the anchor; joint[i+1] is the
 * tip of arm i.  One extra slot keeps indexing by the arm count safe. */
static float joint_pixel_x[DFT_SAMPLE_COUNT + 1];
static float joint_pixel_y[DFT_SAMPLE_COUNT + 1];

/* Where the chain is anchored and how big it is on screen — set by
 * scene_init from the terminal size. */
static float pivot_pixel_x = 0.0f;
static float pivot_pixel_y = 0.0f;
static float shape_pixel_scale = 1.0f;

/* The animation angle, nudged forward one notch per frame. */
static float animation_phase_radians = 0.0f;

static void compute_chain_joint_positions(void) {
  /* Start at the anchor and, arm by arm, step off in the direction that
   * arm currently points, building up the joint positions the renderer
   * will draw. */
  float x = pivot_pixel_x;
  float y = pivot_pixel_y;
  joint_pixel_x[0] = x;
  joint_pixel_y[0] = y;
  for (int i = 0; i < active_epicycle_count; i++) {
    const Epicycle *e = &epicycle_table[i];
    float angle = (float)e->frequency_signed * animation_phase_radians +
                  e->phase_offset_radians;
    float radius_pixels = e->amplitude_normalised * shape_pixel_scale;
    x += radius_pixels * cosf(angle);
    y += radius_pixels * sinf(angle);
    joint_pixel_x[i + 1] = x;
    joint_pixel_y[i + 1] = y;
  }
}

/* §13  scene — the live animation state and the once-per-frame update.
 * These globals are the user-facing knobs (pause, theme, arm count...).
 * They're plain file-scope variables rather than a struct so you can
 * grep a name and find everywhere it's touched. */

static int screen_rows = 0;
static int screen_cols = 0;
static int active_shape_index = 0;
static int active_theme_index = 0;
static bool simulation_paused = false;
static bool auto_grow_enabled = true;
static bool show_orbits_enabled = true;
static bool show_debug_spectrum = false;  /* 'd' */
static bool show_debug_arm_table = false; /* 'D' */
static int auto_grow_counter = 0;

static void scene_reset_for_shape(int shape_index) {
  /* Switch to a shape (or restart the current one): rebuild its arms,
   * rewind to one arm, and clear the trail. */
  active_shape_index = shape_index;
  animation_phase_radians = 0.0f;
  auto_grow_counter = 0;
  active_epicycle_count = 1;
  trail_clear(&tip_trail);
  build_sorted_epicycle_table(active_shape_index);
  compute_chain_joint_positions();
}

static void scene_init(int rows, int cols) {
  /* Fit the chain to the current terminal size.  Runs at startup and
   * again on every resize. */
  screen_rows = rows;
  screen_cols = cols;

  /* Anchor the chain at the centre of the screen. */
  pivot_pixel_x = (float)(cols * CELL_PIXEL_WIDTH) * 0.5f;
  pivot_pixel_y = (float)(rows * CELL_PIXEL_HEIGHT) * 0.5f;

  /* Size the curve off the smaller screen dimension so it still fits
   * on a tall, narrow terminal. */
  float min_pixels = fminf((float)(cols * CELL_PIXEL_WIDTH),
                           (float)(rows * CELL_PIXEL_HEIGHT));
  shape_pixel_scale = min_pixels * SHAPE_PIXEL_SCALE_FRAC;

  simulation_paused = false;
  auto_grow_enabled = true;
  show_orbits_enabled = true;
  show_debug_spectrum = false;
  show_debug_arm_table = false;
  scene_reset_for_shape(active_shape_index);
}

static void animation_tick(void) {
  /* Move the animation forward one frame: maybe add an arm, advance the
   * angle, refresh the joints, and drop a point in the trail.  All the
   * state changes live here so the drawing code stays read-only. */
  if (simulation_paused)
    return;

  /* When auto-grow is on, slip in one more arm every so often so the
   * shape sharpens up on its own, hands-free. */
  if (auto_grow_enabled && active_epicycle_count < total_epicycle_count) {
    auto_grow_counter++;
    if (auto_grow_counter >= AUTO_GROW_INTERVAL_FRAMES) {
      auto_grow_counter = 0;
      active_epicycle_count++;
    }
  }

  /* Nudge the angle forward — one degree per frame on the default
   * setting, so a full loop takes 360 frames. */
  animation_phase_radians +=
      2.0f * (float)M_PI / (float)ANIMATION_FRAMES_PER_CYCLE;

  /* Each time the angle completes a full turn, wipe the trail so the
   * next lap draws on a clean slate instead of piling dots on dots. */
  if (animation_phase_radians >= 2.0f * (float)M_PI) {
    animation_phase_radians -= 2.0f * (float)M_PI;
    trail_clear(&tip_trail);
  }

  compute_chain_joint_positions();
  trail_push(&tip_trail, joint_pixel_x[active_epicycle_count],
             joint_pixel_y[active_epicycle_count]);
}

/* §14  bresenham — draw a straight line between two cells using only
 * whole-number steps (no floating point in the loop). */

static inline int pixel_to_cell_col(float pixel_x) {
  /* The +0.5 rounds to the nearest cell.  Fine here because tip and arm
   * positions are never negative on a sane terminal. */
  return (int)(pixel_x / (float)CELL_PIXEL_WIDTH + 0.5f);
}

static inline int pixel_to_cell_row(float pixel_y) {
  return (int)(pixel_y / (float)CELL_PIXEL_HEIGHT + 0.5f);
}

static char bresenham_glyph_for_step(int err, int dx, int dy, int sx, int sy) {
  /* Pick a character that matches the line's local slant: a slash for a
   * diagonal step, a dash for sideways, a bar for up/down. */
  int e2 = 2 * err;
  bool advance_x = (e2 > -dy);
  bool advance_y = (e2 < dx);
  if (advance_x && advance_y)
    return (sx == sy) ? '\\' : '/';
  if (advance_x)
    return '-';
  return '|';
}

static void bresenham_line(int x0, int y0, int x1, int y1, attr_t attr) {
  /* Bresenham's classic line walk: a running error term decides, at each
   * cell, whether to step across, down, or both. */
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1;
  int sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;

  for (;;) {
    if (x0 >= 0 && x0 < screen_cols && y0 >= 0 && y0 < screen_rows) {
      char g = bresenham_glyph_for_step(err, dx, dy, sx, sy);
      attron(attr);
      mvaddch(y0, x0, (chtype)(unsigned char)g);
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

/* §15  ellipse — draw the circular orbit guide for one arm.
 * The orbit is a true circle in the chain's units, but terminal cells
 * are twice as tall as wide, so on screen it has to be squashed into an
 * ellipse to look round — wider in cells than it is tall.  We just dot
 * '.' around it, using more dots for bigger orbits so they don't look
 * gappy. */

static void render_orbit_ellipse(float pivot_pixel_x_local,
                                 float pivot_pixel_y_local,
                                 float radius_pixels) {
  if (radius_pixels < ORBIT_DRAW_MIN_RADIUS_PX)
    return;

  float semi_axis_x = radius_pixels / (float)CELL_PIXEL_WIDTH;
  float semi_axis_y = radius_pixels / (float)CELL_PIXEL_HEIGHT;
  int centre_col = pixel_to_cell_col(pivot_pixel_x_local);
  int centre_row = pixel_to_cell_row(pivot_pixel_y_local);

  /* More dots for a bigger orbit; the +4 keeps a tiny one from
   * shrinking to a single dot. */
  int sample_count =
      (int)(2.0f * (float)M_PI * fmaxf(semi_axis_x, semi_axis_y)) + 4;

  attr_t attr = COLOR_PAIR(PAIR_ORBIT);
  attron(attr);
  for (int i = 0; i < sample_count; i++) {
    float theta = 2.0f * (float)M_PI * (float)i / (float)sample_count;
    int x = centre_col + (int)(semi_axis_x * cosf(theta) + 0.5f);
    int y = centre_row + (int)(semi_axis_y * sinf(theta) + 0.5f);
    if (x >= 0 && x < screen_cols && y >= 0 && y < screen_rows)
      mvaddch(y, x, '.');
  }
  attroff(attr);
}

/* §16  render_orbits — the faint guide circles, drawn first (behind
 * everything else). */

static void render_orbit_layer(void) {
  /* One orbit per joint, but capped: past a handful they just clutter
   * the view, and the tiny ones shrink to a single dot anyway. */
  if (!show_orbits_enabled)
    return;
  int draw_count = (active_epicycle_count < ORBIT_DRAW_COUNT_MAX)
                       ? active_epicycle_count
                       : ORBIT_DRAW_COUNT_MAX;
  for (int i = 0; i < draw_count; i++) {
    float radius_pixels =
        epicycle_table[i].amplitude_normalised * shape_pixel_scale;
    render_orbit_ellipse(joint_pixel_x[i], joint_pixel_y[i], radius_pixels);
  }
}

/* §17  render_trail — the fading tip path.
 * We draw it oldest dot first so the freshest dots land on top, and we
 * colour each dot by age in three bands (oldest, middle, newest) — three
 * tiers read more cleanly in a terminal than a smooth fade would. */

static void render_trail_layer(void) {
  int filled = tip_trail.filled_count;
  if (filled == 0)
    return;

  int oldest_index =
      (tip_trail.write_head - filled + TIP_TRAIL_LENGTH) % TIP_TRAIL_LENGTH;
  for (int i = 0; i < filled; i++) {
    int index = (oldest_index + i) % TIP_TRAIL_LENGTH;
    int col = pixel_to_cell_col(tip_trail.pixel_x[index]);
    int row = pixel_to_cell_row(tip_trail.pixel_y[index]);
    if (col < 0 || col >= screen_cols)
      continue;
    if (row < 0 || row >= screen_rows)
      continue;

    float age_fraction = (float)i / (float)filled;
    int pair_id = (age_fraction > 0.70f)   ? PAIR_TRAIL_NEW
                  : (age_fraction > 0.35f) ? PAIR_TRAIL_MID
                                           : PAIR_TRAIL_OLD;
    attr_t attr = COLOR_PAIR(pair_id) | A_BOLD;
    attron(attr);
    mvaddch(row, col, '*');
    attroff(attr);
  }
}

/* §18  render_chain — the arms themselves, drawn as connected lines. */

static int arm_pair_for_radius(float radius_pixels) {
  /* Colour an arm by its length so the few big arms stand out from the
   * crowd of tiny fine-tuning arms when there are many. */
  if (radius_pixels > shape_pixel_scale * ARM_RADIUS_THRESHOLD_LARGE)
    return PAIR_ARM_LARGE;
  if (radius_pixels > shape_pixel_scale * ARM_RADIUS_THRESHOLD_MID)
    return PAIR_ARM_MID;
  return PAIR_ARM_SMALL;
}

static void render_chain_layer(void) {
  /* One line per arm, joint to joint. */
  for (int i = 0; i < active_epicycle_count; i++) {
    float radius_pixels =
        epicycle_table[i].amplitude_normalised * shape_pixel_scale;
    int pair = arm_pair_for_radius(radius_pixels);
    attr_t attr = COLOR_PAIR(pair) | A_BOLD;
    bresenham_line(pixel_to_cell_col(joint_pixel_x[i]),
                   pixel_to_cell_row(joint_pixel_y[i]),
                   pixel_to_cell_col(joint_pixel_x[i + 1]),
                   pixel_to_cell_row(joint_pixel_y[i + 1]), attr);
  }
}

/* §19  render_markers — the two standout dots: the pen at the tip and
 * the anchor at the pivot. */

static void render_markers_layer(void) {
  /* '@' marks the pen at the very tip, '+' marks the anchor; both bright
   * and bold so they pop against the trail and arms. */

  int tip_col = pixel_to_cell_col(joint_pixel_x[active_epicycle_count]);
  int tip_row = pixel_to_cell_row(joint_pixel_y[active_epicycle_count]);
  if (tip_col >= 0 && tip_col < screen_cols && tip_row >= 0 &&
      tip_row < screen_rows) {
    attr_t attr = COLOR_PAIR(PAIR_TIP_BOB) | A_BOLD;
    attron(attr);
    mvaddch(tip_row, tip_col, '@');
    attroff(attr);
  }

  int pivot_col = pixel_to_cell_col(pivot_pixel_x);
  int pivot_row = pixel_to_cell_row(pivot_pixel_y);
  if (pivot_col >= 0 && pivot_col < screen_cols && pivot_row >= 0 &&
      pivot_row < screen_rows) {
    attr_t attr = COLOR_PAIR(PAIR_PIVOT) | A_BOLD;
    attron(attr);
    mvaddch(pivot_row, pivot_col, '+');
    attroff(attr);
  }
}

/* §20  render_debug — two optional info overlays, off by default.
 *   'd' — a bar chart of the first ~30 arm lengths.  Its profile shows
 *         how fast the curve's detail dies off: one tall bar for a
 *         circle, a long slow tail for a spiky shape.
 *   'D' — a small table of the first few arms' numbers, handy for
 *         sanity-checking the transform by eye.
 * Both reuse the arm colour pairs, so they follow the active theme. */

#define DEBUG_SPECTRUM_BAR_COUNT 30
#define DEBUG_SPECTRUM_BAR_HEIGHT 6
#define DEBUG_ARM_TABLE_ROWS 8

static void render_debug_spectrum(void) {
  if (!show_debug_spectrum)
    return;

  int num_bars = (active_epicycle_count < DEBUG_SPECTRUM_BAR_COUNT)
                     ? active_epicycle_count
                     : DEBUG_SPECTRUM_BAR_COUNT;
  if (num_bars <= 0)
    return;

  float max_amp = epicycle_table[0].amplitude_normalised;
  if (max_amp <= 0.0f)
    return;

  int chart_left = 1;
  int chart_bottom = screen_rows - 3; /* leave room for hint line */

  /* Label row sits one below the bottom of the chart. */
  if (chart_bottom + 1 < screen_rows) {
    attron(COLOR_PAIR(PAIR_HINT));
    mvprintw(chart_bottom + 1, chart_left, "amplitude (sorted by rank)");
    attroff(COLOR_PAIR(PAIR_HINT));
  }

  for (int i = 0; i < num_bars; i++) {
    float frac = epicycle_table[i].amplitude_normalised / max_amp;
    int h = (int)(frac * (float)DEBUG_SPECTRUM_BAR_HEIGHT + 0.5f);
    if (h < 1 && frac > 0.001f)
      h = 1;
    for (int row = 0; row < h; row++) {
      int x = chart_left + i;
      int y = chart_bottom - row;
      if (x < 0 || x >= screen_cols)
        continue;
      if (y < 1 || y >= screen_rows)
        continue;
      int pair = (i < 1)   ? PAIR_ARM_LARGE
                 : (i < 6) ? PAIR_ARM_MID
                           : PAIR_ARM_SMALL;
      attron(COLOR_PAIR(pair) | A_BOLD);
      mvaddch(y, x, '#');
      attroff(COLOR_PAIR(pair) | A_BOLD);
    }
  }
}

static void render_debug_arm_table(void) {
  if (!show_debug_arm_table)
    return;

  int rows_to_show = (active_epicycle_count < DEBUG_ARM_TABLE_ROWS)
                         ? active_epicycle_count
                         : DEBUG_ARM_TABLE_ROWS;
  int x = 2;
  int y = 2;
  if (y + rows_to_show + 1 >= screen_rows)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "  i  freq    amp     phase ");
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int i = 0; i < rows_to_show; i++) {
    const Epicycle *e = &epicycle_table[i];
    int pair = (i < 1)   ? PAIR_ARM_LARGE
               : (i < 6) ? PAIR_ARM_MID
                         : PAIR_ARM_SMALL;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(y + 1 + i, x, " %2d  %4d  %5.3f  %+5.2f", i, e->frequency_signed,
             (double)e->amplitude_normalised, (double)e->phase_offset_radians);
    attroff(COLOR_PAIR(pair) | A_BOLD);
  }
}

/* §21  hud — the status line up top and the key hint along the bottom.
 * Both stay bright yellow / bright cyan and bold in every theme so they
 * never get lost behind the animation's colours. */

static void hud_paint_status(int term_cols, double measured_fps) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " Epicycles  %s  thm:%s  arms:%3d/%-3d  %s  %s  %5.1f fps  %s ",
           shape_table[active_shape_index].display_name,
           theme_table[active_theme_index].display_name, active_epicycle_count,
           total_epicycle_count, auto_grow_enabled ? "auto" : "manual",
           show_orbits_enabled ? "orbits" : "no-orbits", measured_fps,
           simulation_paused ? "PAUSED " : "running");
  int slen = (int)strlen(buf);
  int sx = term_cols - slen;
  if (sx < 0)
    sx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0,
           " q:quit  spc:pause  n/p:shape  t/T:theme  +/-:arms  "
           "r:reset  a:auto  c:circles  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §22  screen — all the ncurses bookkeeping: set up, tear down, react to
 * resizes, and push a finished frame to the terminal. */

typedef struct {
  int rows, cols; /* current terminal size in cells */
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  noecho();
  cbreak();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1); /* stop ncurses pausing to check for input mid-draw, which tears */
  colors_init();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *screen) {
  /* After a resize, ncurses only notices the new size if we bounce it
   * with endwin + refresh before asking how big the terminal now is. */
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, double measured_fps) {
  /* Order matters: later layers paint over earlier ones where they
   * overlap, and the HUD goes on last so it's always on top. */
  erase();
  render_orbit_layer();
  render_trail_layer();
  render_chain_layer();
  render_markers_layer();
  render_debug_spectrum();
  render_debug_arm_table();
  hud_paint_status(screen->cols, measured_fps);
  hud_paint_hint(screen->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* §23  app — the main loop, the signal handlers, and the keyboard. */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* A signal handler can do very little safely, so it only flips a flag
   * and lets the main loop act on it. */
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int ch) {
  /* Returns false when the user asks to quit; ignores unknown keys. */
  switch (ch) {
  case 'q':
  case 'Q':
  case 27: /* 27 = ESC */
    return false;

  case ' ':
    simulation_paused = !simulation_paused;
    break;

  case 'n':
  case 'N':
    scene_reset_for_shape((active_shape_index + 1) % SHAPE_COUNT);
    break;
  case 'p':
  case 'P':
    scene_reset_for_shape((active_shape_index + SHAPE_COUNT - 1) % SHAPE_COUNT);
    break;

  case 't':
    active_theme_index = (active_theme_index + 1) % THEME_COUNT;
    apply_theme(active_theme_index);
    break;
  case 'T':
    active_theme_index = (active_theme_index + THEME_COUNT - 1) % THEME_COUNT;
    apply_theme(active_theme_index);
    break;

  case '+':
  case '=':
    if (active_epicycle_count < total_epicycle_count)
      active_epicycle_count++;
    break;
  case '-':
    if (active_epicycle_count > 1) {
      active_epicycle_count--;
      trail_clear(&tip_trail);
    }
    break;

  case 'r':
  case 'R':
    scene_reset_for_shape(active_shape_index);
    break;

  case 'a':
  case 'A':
    auto_grow_enabled = !auto_grow_enabled;
    break;

  case 'c':
  case 'C':
    show_orbits_enabled = !show_orbits_enabled;
    break;

  case 'd':
    show_debug_spectrum = !show_debug_spectrum;
    break;
  case 'D':
    show_debug_arm_table = !show_debug_arm_table;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  Screen screen;
  screen_init(&screen);
  scene_init(screen.rows, screen.cols);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;

  while (!g_should_quit) {
    int64_t frame_start_ns = clock_now_ns();

    /* ── input ────────────────────────────────────────────────── */
    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(ch)) {
        g_should_quit = 1;
        break;
      }
    }

    /* ── resize ───────────────────────────────────────────────── */
    if (g_resize_pending) {
      g_resize_pending = 0;
      screen_resize(&screen);
      scene_init(screen.rows, screen.cols);
      prev_frame_ns = clock_now_ns();
    }

    /* ── dt + rolling-fps measurement ─────────────────────────── */
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= 500 * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    /* ── animation + render ──────────────────────────────────── */
    animation_tick();
    screen_present_frame(&screen, measured_fps);

    /* ── frame cap (sleep BEFORE next iteration's I/O) ───────── */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  return 0;
}
