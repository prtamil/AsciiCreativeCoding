/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fir_filter.c — build a low-pass, high-pass, or band-pass filter on the
 * fly from a windowed-sinc recipe, run it over a test signal, and watch
 * the input, the filter, its frequency response, and the output side by
 * side.
 *
 * Sister files: signal/convolution_helloworld.c (the slide-multiply-sum
 * step used to apply the filter), signal/idft_helloworld.c (the same
 * filtering done in the frequency domain instead).
 * Windowed-sinc design: Smith, "The Scientist and Engineer's Guide to
 * Digital Signal Processing", ch. 14-16 (free online).
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

/* §1  config */

#define N 64 /* test signal length, in samples */
#define K                                                                      \
  21 /* filter length; kept odd so the kernel has one exact center tap */
#define K_CENTER (K / 2)
#define N_HALF (N / 2)
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 120 /* one full cutoff sweep, about 4 seconds */

enum {
  PAIR_INPUT = 1,   /* input wave bars             */
  PAIR_OUTPUT = 2,  /* filtered output wave bars   */
  PAIR_FILTER = 3,  /* filter impulse response bars */
  PAIR_MAGRESP = 4, /* magnitude response bars     */
  PAIR_PHRESP = 5,  /* phase response bars (debug) */
  PAIR_LABEL = 6,   /* panel labels                */
  PAIR_HUD = 7,     /* top status line             */
  PAIR_HINT = 8,    /* bottom key hint             */
};

/* §2  clock */

static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* §3  complex
 *
 * A complex number, used only by the DFT in §9 that draws the frequency-
 * response panels. The filtering itself stays in plain real numbers.
 */

typedef struct {
  float re; /* real part      */
  float im; /* imaginary part */
} ComplexNumber;

static const ComplexNumber complex_zero = {0.0f, 0.0f};

static inline ComplexNumber complex_make(float re, float im) {
  return (ComplexNumber){re, im};
}

static inline ComplexNumber complex_from_angle(float angle_radians) {
  return complex_make(cosf(angle_radians), sinf(angle_radians));
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re + b.re, a.im + b.im);
}

static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z) {
  return complex_make(scale * z.re, scale * z.im);
}

static inline float complex_magnitude(ComplexNumber z) {
  return sqrtf(z.re * z.re + z.im * z.im);
}

/* §4  colors */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_INPUT, 51, -1);    /* bright cyan      */
    init_pair(PAIR_OUTPUT, 154, -1);  /* yellow-green     */
    init_pair(PAIR_FILTER, 213, -1);  /* magenta-pink     */
    init_pair(PAIR_MAGRESP, 220, -1); /* gold             */
    init_pair(PAIR_PHRESP, 117, -1);  /* sky blue         */
    init_pair(PAIR_LABEL, 244, -1);   /* mid grey         */
    init_pair(PAIR_HUD, 226, -1);     /* bright yellow    */
    init_pair(PAIR_HINT, 51, -1);     /* bright cyan      */
  } else {
    init_pair(PAIR_INPUT, COLOR_CYAN, -1);
    init_pair(PAIR_OUTPUT, COLOR_GREEN, -1);
    init_pair(PAIR_FILTER, COLOR_MAGENTA, -1);
    init_pair(PAIR_MAGRESP, COLOR_YELLOW, -1);
    init_pair(PAIR_PHRESP, COLOR_BLUE, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* §5  sinc_helper
 *
 * The sinc curve sin(pi*x)/(pi*x): a tall bump at the middle that ripples
 * away to either side. It is the shape a perfect low-pass filter would
 * have if it could go on forever, so it is where every filter here starts.
 */
static float sinc(float x) {
  /* At x = 0 the formula is 0/0; its true value there is 1, so special-case it. */
  if (fabsf(x) < 1e-7f)
    return 1.0f;
  float pi_x = (float)M_PI * x;
  return sinf(pi_x) / pi_x;
}

/* §6  windows
 *
 * The window picks how gently we fade the filter's ends to zero. Fading
 * harder cleans up junk frequencies in the stopband but blurs the cutoff;
 * fading less does the opposite. These four trade that off in steps.
 */

typedef enum {
  WIN_RECT = 0, /* no fade at all; sharpest cutoff but the most junk */
  WIN_HANN,     /* gentle fade; a good everyday default              */
  WIN_HAMMING,  /* a touch cleaner stopband than Hann               */
  WIN_BLACKMAN, /* hardest fade; cleanest stopband, blurriest cutoff */
  WIN_COUNT
} WindowKind;

static const char *window_name[WIN_COUNT] = {"Rectangular", "Hann       ",
                                             "Hamming    ", "Blackman   "};

/* The fade factor for tap i: ~1 in the middle, tapering toward 0 at both
 * ends. design_filter() multiplies each tap by this. */
static float window_coefficient(WindowKind kind, int i) {
  float two_pi_over_K_minus_1 = 2.0f * (float)M_PI / (float)(K - 1);
  switch (kind) {
  case WIN_RECT:
    return 1.0f;
  case WIN_HANN:
    return 0.5f - 0.5f * cosf(two_pi_over_K_minus_1 * (float)i);
  case WIN_HAMMING:
    return 0.54f - 0.46f * cosf(two_pi_over_K_minus_1 * (float)i);
  case WIN_BLACKMAN:
    return 0.42f - 0.5f * cosf(two_pi_over_K_minus_1 * (float)i) +
           0.08f * cosf(2.0f * two_pi_over_K_minus_1 * (float)i);
  default:
    return 1.0f;
  }
}

/* §7  filter_design — build the windowed-sinc kernel */

typedef enum {
  FILT_LOW_PASS = 0, /* keep low frequencies, drop high ones    */
  FILT_HIGH_PASS,    /* keep high frequencies, drop low ones    */
  FILT_BAND_PASS,    /* keep a band in the middle, drop the rest */
  FILT_COUNT
} FilterKind;

static const char *filter_name[FILT_COUNT] = {"low-pass  ", "high-pass ",
                                              "band-pass "};

/* Fill h with a plain low-pass kernel for cutoff fc (in cycles per sample):
 * a sinc bump centered in the middle of the K taps. High-pass and band-pass
 * are both built by combining one or two of these. */
static void lowpass_sinc(float fc, float *h) {
  float center = (float)(K - 1) * 0.5f; /* the middle tap, as a float */
  for (int i = 0; i < K; i++) {
    float x = 2.0f * fc * ((float)i - center);
    h[i] = 2.0f * fc * sinc(x);
  }
}

/* Build the actual filter kernel h for the chosen type, cutoff, and window.
 * This is the heart of the demo: first shape the kernel, then fade its ends. */
static void design_filter(FilterKind kind, int cutoff_bin, WindowKind window,
                          float *h) {
  /* The user picks the cutoff as a bin number out of N; the math wants it
   * as a fraction of the sample rate, which is just bin / N. */
  float cutoff_normalised = (float)cutoff_bin / (float)N;

  /* Step 1: shape the kernel for the chosen filter type. */
  switch (kind) {
  case FILT_LOW_PASS: {
    lowpass_sinc(cutoff_normalised, h);
    break;
  }
  case FILT_HIGH_PASS: {
    /* High-pass is "everything except the low-pass": flip the low-pass
     * kernel upside down, then poke a 1 into the center tap. */
    lowpass_sinc(cutoff_normalised, h);
    for (int i = 0; i < K; i++)
      h[i] = -h[i];
    h[K_CENTER] += 1.0f;
    break;
  }
  case FILT_BAND_PASS: {
    /* Band-pass is the gap between two low-passes: keep what the wider one
     * passes, subtract what the narrower one passes. The cutoff knob is the
     * center of the band; we fix the half-width at 3 bins. */
    float band_half_bins = 3.0f;
    float fc_lo = ((float)cutoff_bin - band_half_bins) / (float)N;
    float fc_hi = ((float)cutoff_bin + band_half_bins) / (float)N;
    if (fc_lo < 0.01f)
      fc_lo = 0.01f;
    if (fc_hi > 0.49f)
      fc_hi = 0.49f;
    float h_lo[K], h_hi[K];
    lowpass_sinc(fc_lo, h_lo);
    lowpass_sinc(fc_hi, h_hi);
    for (int i = 0; i < K; i++)
      h[i] = h_hi[i] - h_lo[i];
    break;
  }
  default:
    break;
  }

  /* Step 2: fade the ends to zero so the cut-off edges don't ring. */
  for (int i = 0; i < K; i++)
    h[i] *= window_coefficient(window, i);
}

/* §8  convolve — run the filter over the signal
 *
 * Applying the filter: slide the kernel along the signal, and at each spot
 * line up the K taps with K input samples, multiply each pair, and add them
 * up for one output sample. Same operation as convolution_helloworld.c. */
static void convolve(const float *input_signal, const float *kernel_taps,
                     float *output_signal) {
  /* Past this spot the kernel would hang off the end of the input. */
  int last_valid_output_position = N - K;

  for (int output_position = 0; output_position <= last_valid_output_position;
       output_position++) {

    float weighted_sum = 0.0f;

    for (int kernel_tap_index = 0; kernel_tap_index < K; kernel_tap_index++) {
      float kernel_weight = kernel_taps[kernel_tap_index];
      float input_sample = input_signal[output_position + kernel_tap_index];
      weighted_sum += kernel_weight * input_sample;
    }

    output_signal[output_position] = weighted_sum;
  }

  /* The last K-1 spots have no full window of input, so leave them blank. */
  for (int output_position = last_valid_output_position + 1;
       output_position < N; output_position++) {
    output_signal[output_position] = 0.0f;
  }
}

/* §9  dft_response — measure what the filter does to each frequency
 *
 * Ask the kernel, for every frequency, "how much do you pass through, and
 * how much do you delay it?" The answers (strength and phase) feed the
 * magnitude and phase panels. This is only for the picture; it has nothing
 * to do with actually filtering the signal.
 *
 * We pad the K-tap kernel out to N with zeros (more taps = a finer answer)
 * and run a plain textbook DFT, which is plenty fast at this small size. */
static void compute_filter_response(const float *h, float *out_magnitude,
                                    float *out_phase) {
  float padded[N];
  for (int i = 0; i < N; i++)
    padded[i] = (i < K) ? h[i] : 0.0f;

  for (int k = 0; k <= N_HALF; k++) {
    ComplexNumber acc = complex_zero;
    for (int n = 0; n < N; n++) {
      float angle = -2.0f * (float)M_PI * (float)k * (float)n / (float)N;
      ComplexNumber twist = complex_from_angle(angle);
      ComplexNumber twisted = complex_scale_by_real(padded[n], twist);
      acc = complex_add(acc, twisted);
    }
    out_magnitude[k] = complex_magnitude(acc);
    out_phase[k] = atan2f(acc.im, acc.re);
  }
}

/* §10  signals — the test inputs you can feed the filter */

typedef enum {
  SIG_SINES = 0, /* three pure tones mixed (bins 3, 11, 23)   */
  SIG_CHIRP,     /* a tone that slides from low to high        */
  SIG_SQUARE,    /* a square wave (lots of high harmonics)     */
  SIG_IMPULSE,   /* a single spike, the rest zeros             */
  SIG_NOISE,     /* fixed pseudo-random fuzz                   */
  SIG_COUNT
} SignalKind;

static const char *signal_name[SIG_COUNT] = {
    "Sum sines", "Chirp    ", "Square   ", "Impulse  ", "Noise    "};

/* Fill the buffer with the chosen test signal. All stay within about ±1 so
 * the panels scale the same way. */
static void generate_signal(SignalKind kind, float *output_signal) {
  switch (kind) {
  case SIG_SINES: {
    /* Three well-separated tones; watch which survive each filter. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      output_signal[n] = 0.5f * cosf(2.0f * (float)M_PI * 3.0f * t) +
                         0.4f * cosf(2.0f * (float)M_PI * 11.0f * t) +
                         0.3f * cosf(2.0f * (float)M_PI * 23.0f * t);
    }
    break;
  }
  case SIG_CHIRP: {
    /* A tone that slides steadily from low to high across the buffer. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      float lo = 1.0f, hi = (float)N * 0.5f - 1.0f;
      float phase_cycles = lo * t + (hi - lo) * t * t * 0.5f;
      output_signal[n] = sinf(2.0f * (float)M_PI * phase_cycles);
    }
    break;
  }
  case SIG_SQUARE: {
    /* A square wave; its sharp corners pack in high harmonics, so a
     * low-pass visibly rounds it off. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      float arg = 2.0f * (float)M_PI * 3.0f * t;
      output_signal[n] = sinf(arg) >= 0.0f ? 0.7f : -0.7f;
    }
    break;
  }
  case SIG_IMPULSE: {
    /* A single spike. Filter it and the output IS the filter's shape,
     * which is exactly what "impulse response" means. */
    for (int n = 0; n < N; n++)
      output_signal[n] = 0.0f;
    output_signal[N / 4] = 1.0f;
    break;
  }
  case SIG_NOISE: {
    /* Fixed fuzz with energy at every frequency, so you can see the
     * filter carve out just the band it keeps. */
    unsigned int seed = 1;
    for (int n = 0; n < N; n++) {
      seed = seed * 1664525u + 1013904223u;
      float r = (float)(seed >> 16) / 65535.0f;
      output_signal[n] = (r - 0.5f) * 1.6f;
    }
    break;
  }
  default:
    break;
  }
}

/* §11  scene_state — everything one frame works on, plus reset */

static float g_input_signal[N];                /* the test signal going in    */
static float g_kernel_taps[K];                 /* the current filter          */
static float g_magnitude_response[N_HALF + 1]; /* how much each freq passes   */
static float g_phase_response[N_HALF + 1];     /* each freq's delay (debug)   */
static float g_magnitude_peak = 1.0f;          /* tallest bar, for scaling    */
static float g_output_signal[N];               /* the filtered result         */
static float g_input_peak = 1.0f;              /* tallest input, for scaling  */
static float g_output_peak = 1.0f;             /* tallest output, for scaling */

static SignalKind g_signal_kind = SIG_SINES;
static FilterKind g_filter_kind = FILT_LOW_PASS;
static WindowKind g_window_kind = WIN_HANN;
static int g_cutoff_bin = 5;
static bool g_simulation_paused = false;
static bool g_auto_sweep_cutoff = true;
static float g_sweep_phase_radians = 0.0f;

static bool g_show_phase_panel = false;  /* 'd' overlay */
static bool g_show_kernel_table = false; /* 'D' overlay */

static void scene_reset(void) {
  g_signal_kind = SIG_SINES;
  g_filter_kind = FILT_LOW_PASS;
  g_window_kind = WIN_HANN;
  g_cutoff_bin = 5;
  g_simulation_paused = false;
  g_auto_sweep_cutoff = true;
  g_sweep_phase_radians = 0.0f;
}

/* §12  scene_tick — everything that happens for one frame */
static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* When sweeping, walk the cutoff smoothly back and forth (sin) between
   * bin 3 and N/2-3, leaving room on both sides for band-pass. */
  if (g_auto_sweep_cutoff) {
    g_sweep_phase_radians += 2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
    if (g_sweep_phase_radians > 2.0f * (float)M_PI)
      g_sweep_phase_radians -= 2.0f * (float)M_PI;
    float s = (sinf(g_sweep_phase_radians) + 1.0f) * 0.5f;
    g_cutoff_bin = 3 + (int)(s * ((float)N * 0.5f - 6.0f) + 0.5f);
  }

  generate_signal(g_signal_kind, g_input_signal);
  design_filter(g_filter_kind, g_cutoff_bin, g_window_kind, g_kernel_taps);
  convolve(g_input_signal, g_kernel_taps, g_output_signal);
  compute_filter_response(g_kernel_taps, g_magnitude_response,
                          g_phase_response);

  /* Note each panel's tallest value so the bars fill the space nicely. */
  g_input_peak = 1e-6f;
  for (int n = 0; n < N; n++) {
    float a = fabsf(g_input_signal[n]);
    if (a > g_input_peak)
      g_input_peak = a;
  }
  g_output_peak = 1e-6f;
  for (int n = 0; n < N; n++) {
    float a = fabsf(g_output_signal[n]);
    if (a > g_output_peak)
      g_output_peak = a;
  }
  g_magnitude_peak = 1e-6f;
  for (int k = 0; k <= N_HALF; k++) {
    if (g_magnitude_response[k] > g_magnitude_peak)
      g_magnitude_peak = g_magnitude_response[k];
  }
}

/* §13  scene_input — react to keys */

static void scene_cycle_filter(int direction) {
  g_filter_kind =
      (FilterKind)((g_filter_kind + direction + FILT_COUNT) % FILT_COUNT);
}

static void scene_cycle_window(int direction) {
  g_window_kind =
      (WindowKind)((g_window_kind + direction + WIN_COUNT) % WIN_COUNT);
}

static void scene_cycle_signal(int direction) {
  g_signal_kind =
      (SignalKind)((g_signal_kind + direction + SIG_COUNT) % SIG_COUNT);
}

static void scene_adjust_cutoff(int delta) {
  if (g_auto_sweep_cutoff)
    return;
  g_cutoff_bin += delta;
  if (g_cutoff_bin < 3)
    g_cutoff_bin = 3;
  if (g_cutoff_bin > N_HALF - 3)
    g_cutoff_bin = N_HALF - 3;
}

/* §14  draw_bar — one vertical bar, up or down from a baseline */

static void draw_bar(int column, int baseline_row, int bar_height_cells,
                     bool growing_upward, int colour_pair) {
  if (bar_height_cells <= 0)
    return;
  attron(COLOR_PAIR(colour_pair) | A_BOLD);
  for (int dy = 0; dy < bar_height_cells; dy++) {
    int row = growing_upward ? (baseline_row - dy) : (baseline_row + dy + 1);
    if (row < 0 || row >= LINES)
      continue;
    if (column < 0 || column >= COLS)
      continue;
    mvaddch(row, column, growing_upward ? '|' : '.');
  }
  attroff(COLOR_PAIR(colour_pair) | A_BOLD);
}

/* §15  draw_signal — the input and output wave panels */

static void draw_signal_panel(const float *signal, int top_row, int height_rows,
                              float peak, int colour_pair) {
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int spacing = (COLS / N >= 2) ? 2 : 1;
  int columns = (COLS / spacing < N) ? COLS / spacing : N;

  for (int n = 0; n < columns; n++) {
    float v = signal[n] / (peak + 1e-6f);
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    for (int s = 0; s < spacing; s++)
      draw_bar(n * spacing + s, midline_row, h, pos, colour_pair);
  }
}

/* §16  draw_filter — the filter, its strength, and its phase panels */

static void draw_impulse_response_panel(int top_row, int height_rows) {
  /* The filter's K taps as bars centered on the panel: up for positive,
   * down for negative. */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  /* Tallest tap, so the bars fill the panel. */
  float peak = 0.0f;
  for (int i = 0; i < K; i++) {
    float a = fabsf(g_kernel_taps[i]);
    if (a > peak)
      peak = a;
  }
  if (peak < 1e-6f)
    peak = 1.0f;

  /* Center the kernel across the screen, 3 columns per tap. */
  int tap_w = 3;
  int total_w = K * tap_w;
  int left = (COLS - total_w) / 2;
  if (left < 0)
    left = 0;

  for (int i = 0; i < K; i++) {
    float v = g_kernel_taps[i] / peak;
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    for (int s = 0; s < tap_w - 1; s++) {
      int col = left + i * tap_w + s;
      draw_bar(col, midline_row, h, pos, PAIR_FILTER);
    }
  }

  /* Zero line. */
  attron(COLOR_PAIR(PAIR_LABEL));
  int mid_left = left;
  int mid_right = left + total_w;
  if (mid_left < 0)
    mid_left = 0;
  if (mid_right > COLS)
    mid_right = COLS;
  for (int x = mid_left; x < mid_right; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

static void draw_magnitude_response_panel(int top_row, int height_rows) {
  /* How much each frequency gets through, as bars rising from the bottom.
   * Tall = passed, short = blocked; the overall shape is the passband. */
  int baseline_row = top_row + height_rows - 1;
  int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
  int max_bins = COLS / bin_w;
  if (max_bins > N_HALF + 1)
    max_bins = N_HALF + 1;

  for (int k = 0; k < max_bins; k++) {
    float v = g_magnitude_response[k] / (g_magnitude_peak + 1e-6f);
    int h = (int)(v * (float)height_rows + 0.5f);
    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(k * bin_w + bx, baseline_row, h, true, PAIR_MAGRESP);
  }

  /* A ':' column marking where the cutoff sits. */
  if (g_cutoff_bin <= max_bins) {
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    for (int dy = 0; dy < height_rows; dy++) {
      int row = top_row + dy;
      int col = g_cutoff_bin * bin_w + bin_w / 2;
      if (col < COLS && row < LINES)
        mvaddch(row, col, ':');
    }
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  }
}

static void draw_phase_response_panel(int top_row, int height_rows) {
  /* How much each frequency is delayed, as bars above/below a midline.
   * Our filters are symmetric, so this comes out as a clean ramp that
   * snaps back when it wraps past pi: every frequency shifts by the same
   * amount, which is why the filter delays the signal without warping it. */
  if (height_rows < 3)
    return;

  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
  int max_bins = COLS / bin_w;
  if (max_bins > N_HALF + 1)
    max_bins = N_HALF + 1;

  for (int k = 0; k < max_bins; k++) {
    /* Where almost nothing gets through, the phase is just noise; skip it. */
    if (g_magnitude_response[k] < 1e-3f * g_magnitude_peak)
      continue;

    float fraction = g_phase_response[k] / (float)M_PI;
    bool pos = (g_phase_response[k] >= 0.0f);
    int h = (int)(fabsf(fraction) * (float)half_height + 0.5f);
    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(k * bin_w + bx, midline_row, h, pos, PAIR_PHRESP);
  }

  /* Zero line. */
  attron(COLOR_PAIR(PAIR_LABEL));
  for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

/* §17  draw_debug — the 'D' overlay listing the filter's exact numbers */

static void draw_kernel_table_overlay(void) {
  if (!g_show_kernel_table)
    return;

  int x = 2, y = 2;
  if (y + K + 2 >= LINES - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Kernel taps  %s  (K = %d, win = %s)",
           filter_name[g_filter_kind], K, window_name[g_window_kind]);
  attroff(COLOR_PAIR(PAIR_HINT));

  /* Lay the taps out in two columns to save room. */
  int rows_per_col = (K + 1) / 2;
  for (int i = 0; i < K; i++) {
    int row_offset = i % rows_per_col;
    int col_offset = (i / rows_per_col) * 22;
    attron(COLOR_PAIR(PAIR_FILTER) | A_BOLD);
    mvprintw(y + 1 + row_offset, x + col_offset, "  k[%2d] = %+8.5f", i,
             (double)g_kernel_taps[i]);
    attroff(COLOR_PAIR(PAIR_FILTER) | A_BOLD);
  }
}

/* §18  hud — status line, key hint, and the frame composer */

static void draw_hud(void) {
  char status[200];
  snprintf(status, sizeof status,
           " FIR filter  N=%d K=%d  signal:%s  filter:%s  win:%s  "
           "cutoff:%2d  %s  %s ",
           N, K, signal_name[g_signal_kind], filter_name[g_filter_kind],
           window_name[g_window_kind], g_cutoff_bin,
           g_auto_sweep_cutoff ? "AUTO  " : "MANUAL",
           g_simulation_paused ? "PAUSED" : "      ");
  int x = COLS - (int)strlen(status);
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hint(void) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(LINES - 1, 0,
           " q:quit  spc:pause  f:filter  w:window  s:signal "
           " a:auto/manual  +/-:cutoff  d:phase  D:taps  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Reserve rows for the HUD, labels, and hint; the phase panel borrows
   * some height from the rest when it's switched on. */
  int rows_for_panels = LINES - 6;
  if (rows_for_panels < 12)
    rows_for_panels = 12;

  int phase_h = g_show_phase_panel ? rows_for_panels / 5 : 0;
  int main_h = rows_for_panels - phase_h;
  int input_h = main_h / 4;
  int filter_h = main_h / 4;
  int magresp_h = main_h / 4;
  int output_h = main_h - input_h - filter_h - magresp_h;

  int input_label_row = 1;
  int input_top_row = 2;
  int filter_label_row = 2 + input_h;
  int filter_top_row = 3 + input_h;
  int magresp_label_row = 3 + input_h + filter_h;
  int magresp_top_row = 4 + input_h + filter_h;
  int output_label_row = 4 + input_h + filter_h + magresp_h;
  int output_top_row = 5 + input_h + filter_h + magresp_h;
  int phase_top_row = output_top_row + output_h + (g_show_phase_panel ? 1 : 0);

  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(input_label_row, 0, "Input x[n]");
  mvprintw(filter_label_row, 0, "Filter h[i]  (K = %d taps, centered)", K);
  mvprintw(magresp_label_row, 0,
           "Magnitude response |H[k]|  (cutoff marked with ':')");
  mvprintw(output_label_row, 0, "Output y[n] = h * x");
  if (g_show_phase_panel)
    mvprintw(phase_top_row - 1, 0,
             "Phase response arg(H[k])  (linear ramp = linear phase)");
  attroff(COLOR_PAIR(PAIR_LABEL));

  draw_signal_panel(g_input_signal, input_top_row, input_h, g_input_peak,
                    PAIR_INPUT);
  draw_impulse_response_panel(filter_top_row, filter_h);
  draw_magnitude_response_panel(magresp_top_row, magresp_h);
  draw_signal_panel(g_output_signal, output_top_row, output_h, g_output_peak,
                    PAIR_OUTPUT);
  if (g_show_phase_panel)
    draw_phase_response_panel(phase_top_row, phase_h);

  draw_kernel_table_overlay();

  /* HUD goes on top of everything else, so draw it last. */
  draw_hud();
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* §19  app — signal handlers, key dispatch, and the main loop */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static void cleanup_screen(void) { endwin(); }

static bool app_handle_key(int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return true;
  case ' ':
    g_simulation_paused = !g_simulation_paused;
    break;
  case 'f':
    scene_cycle_filter(+1);
    break;
  case 'F':
    scene_cycle_filter(-1);
    break;
  case 'w':
    scene_cycle_window(+1);
    break;
  case 'W':
    scene_cycle_window(-1);
    break;
  case 's':
    scene_cycle_signal(+1);
    break;
  case 'S':
    scene_cycle_signal(-1);
    break;
  case 'a':
  case 'A':
    g_auto_sweep_cutoff = !g_auto_sweep_cutoff;
    break;
  case '+':
  case '=':
  case '.':
    scene_adjust_cutoff(+1);
    break;
  case '-':
  case ',':
    scene_adjust_cutoff(-1);
    break;
  case 'd':
    g_show_phase_panel = !g_show_phase_panel;
    break;
  case 'D':
    g_show_kernel_table = !g_show_kernel_table;
    break;
  case 'r':
  case 'R':
    scene_reset();
    break;
  default:
    break;
  }
  return false;
}

int main(void) {
  atexit(cleanup_screen);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init();

  scene_reset();

  long long next_frame_ns = clock_now_ns();

  while (!g_should_quit) {
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
    }

    int ch;
    while ((ch = getch()) != ERR) {
      if (app_handle_key(ch)) {
        g_should_quit = 1;
        break;
      }
    }

    long long now = clock_now_ns();
    if (now >= next_frame_ns) {
      scene_tick();
      render_frame();
      next_frame_ns += RENDER_TICK_NS;
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
