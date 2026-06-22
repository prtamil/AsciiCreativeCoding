/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fft_helloworld.c — a live Cooley-Tukey FFT on a moving cosine. Top panel
 * shows the wave, bottom panel shows its spectrum, and every frame we also
 * run the slow textbook DFT to prove the fast one gives the same answer.
 *
 * Original FFT paper: Cooley & Tukey (1965), Math. Comp. 19, 297-301.
 * Sister files: signal/dft_helloworld.c (the DFT-only warm-up),
 *               signal/fft_vis.c (the full-featured version with windows).
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

/* §1  config — constants and colour IDs */

#define N 32     /* sample count. Must be a power of 2 — the algorithm   */
                 /* relies on it. Change this and LOG2_N together.       */
#define LOG2_N 5 /* log2 of N. Kept by hand in step with N above.        */
#define N_HALF (N / 2)
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 90 /* frames for one full auto-sweep, ~3 sec */
#define FREQ_LO 1.0f
#define FREQ_HI ((float)N * 0.5f - 1.0f)
#define ARM_TABLE_ROWS 8 /* how many of the loudest bins the 'D' table shows */

enum {
  PAIR_SIG = 1,       /* the input wave                */
  PAIR_SPEC = 2,      /* spectrum bars                 */
  PAIR_SPIKE = 3,     /* the loudest spectrum bar      */
  PAIR_LABEL = 4,     /* panel labels                  */
  PAIR_HUD = 5,       /* status line                   */
  PAIR_HINT = 6,      /* key hint                      */
  PAIR_PHASE_POS = 7, /* phase bars pointing up        */
  PAIR_PHASE_NEG = 8, /* phase bars pointing down      */
};

/* §2  clock — timer and sleep for pacing the frame rate */

static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  /* Sleep instead of busy-waiting, or we'd burn a whole CPU core
   * just to redraw 30 times a second. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* §3  complex — a tiny complex-number type and its operations.
 *
 * A complex number is just a point on a 2-D plane: a real part and an
 * imaginary part. We roll our own 2-float struct (instead of C's built-in
 * <complex.h>) so each FFT step reads as one named call like complex_mul,
 * and so it prints cleanly in a debugger. These few helpers are everything
 * the FFT needs. */

typedef struct {
  float re; /* real part      — the horizontal coordinate */
  float im; /* imaginary part — the vertical coordinate   */
} ComplexNumber;

static const ComplexNumber complex_zero = {0.0f, 0.0f};
static const ComplexNumber complex_one = {1.0f, 0.0f};

static inline ComplexNumber complex_make(float real_part, float imag_part) {
  return (ComplexNumber){real_part, imag_part};
}

/* Turn an angle into a point on the unit circle (Euler's formula). This is
 * how the FFT gets its "twiddle" — a rotation by a chosen angle. */
static inline ComplexNumber complex_from_angle(float angle_radians) {
  return complex_make(cosf(angle_radians), sinf(angle_radians));
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re + b.re, a.im + b.im);
}

static inline ComplexNumber complex_sub(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re - b.re, a.im - b.im);
}

/* Multiplying two complex numbers rotates and stretches: the angles add and
 * the lengths multiply. Multiplying by a unit-circle value is pure rotation,
 * which is exactly what the butterfly's twiddle does. */
static inline ComplexNumber complex_mul(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
}

/* Stretch a complex number by a plain number, no rotation. The naive DFT
 * uses it because its input samples are ordinary real values. */
static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z) {
  return complex_make(scale * z.re, scale * z.im);
}

/* How long the arrow is — the "loudness" of a spectrum bin. Throws phase
 * away. Also used to measure how far apart the FFT and DFT answers are. */
static inline float complex_magnitude(ComplexNumber z) {
  return sqrtf(z.re * z.re + z.im * z.im);
}

/* §4  colors — set up the colour pairs */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_SIG, 51, -1);        /* bright cyan    */
    init_pair(PAIR_SPEC, 154, -1);      /* yellow-green   */
    init_pair(PAIR_SPIKE, 46, -1);      /* bright green   */
    init_pair(PAIR_LABEL, 244, -1);     /* mid grey       */
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow  */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan    */
    init_pair(PAIR_PHASE_POS, 213, -1); /* magenta-pink   */
    init_pair(PAIR_PHASE_NEG, 117, -1); /* sky blue       */
  } else {
    init_pair(PAIR_SIG, COLOR_CYAN, -1);
    init_pair(PAIR_SPEC, COLOR_GREEN, -1);
    init_pair(PAIR_SPIKE, COLOR_GREEN, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_PHASE_POS, COLOR_MAGENTA, -1);
    init_pair(PAIR_PHASE_NEG, COLOR_BLUE, -1);
  }
}

/* §5  dft_naive — the slow textbook spectrum, our reference answer.
 *
 * This is the plodding direct way to compute the spectrum: for every output
 * bin, sweep across all the input samples and add up their contributions.
 * That double loop is N*N work — fine at N=32, hopeless at audio sizes,
 * which is the whole reason the FFT exists. We keep it only as the trusted
 * answer to check the FFT against each frame. */
static void compute_dft_naive(const float *input_signal,
                              ComplexNumber *output_spectrum) {
  for (int output_bin_index = 0; output_bin_index < N; output_bin_index++) {

    ComplexNumber running_sum = complex_zero;

    for (int input_sample_index = 0; input_sample_index < N;
         input_sample_index++) {

      float twist_angle_radians = -2.0f * (float)M_PI *
                                  (float)output_bin_index *
                                  (float)input_sample_index / (float)N;

      ComplexNumber twist = complex_from_angle(twist_angle_radians);
      ComplexNumber twisted =
          complex_scale_by_real(input_signal[input_sample_index], twist);

      running_sum = complex_add(running_sum, twisted);
    }

    output_spectrum[output_bin_index] = running_sum;
  }
}

/* §6  bit_helpers — flip the low bits of an integer.
 *
 * Mirror the lowest bit_count bits of value end-for-end: bit 0 swaps with
 * the top bit, and so on. The FFT needs this to know where each input sample
 * belongs before it starts (see §7). */

static int reverse_low_bits(int value, int bit_count) {
  /* e.g. 13 = 01101 reversed over 5 bits becomes 10110 = 22. */
  int reversed_value = 0;
  for (int bit_position = 0; bit_position < bit_count; bit_position++)
    reversed_value |= ((value >> bit_position) & 1)
                      << (bit_count - 1 - bit_position);
  return reversed_value;
}

/* §7  bit_reverse — shuffle the buffer into the order the FFT wants.
 *
 * The FFT's first step is to reorder the samples so that the pairs it later
 * combines sit side by side. That target order turns out to be: send each
 * sample at index i to the slot whose number is i's bits reversed. We walk
 * the buffer and swap each such pair once. The "only swap when the partner
 * index is larger" test is what makes it once and not twice (a second visit
 * would just undo the first). */

static void bit_reverse_permute(ComplexNumber *buffer) {
  for (int original_index = 0; original_index < N; original_index++) {
    int reversed_index = reverse_low_bits(original_index, LOG2_N);
    if (reversed_index > original_index) {
      ComplexNumber temp = buffer[original_index];
      buffer[original_index] = buffer[reversed_index];
      buffer[reversed_index] = temp;
    }
  }
}

/* §8  butterfly — the engine that does one pass of the FFT.
 *
 * This is the heart of the whole thing. One "stage" pairs up slots a fixed
 * distance apart, and each pair is combined into two results with a single
 * rotate-and-combine step called a butterfly (it looks like wings if you
 * draw it). The rotation amount is the "twiddle". Rather than recompute the
 * twiddle with a fresh sin/cos every step — which is slow — we work out one
 * step's rotation once, then keep multiplying to walk the angle forward.
 * Each stage doubles the pairing distance: 2, then 4, then 8, up to N. */

static void run_butterfly_stage(ComplexNumber *buffer, int stage_width) {
  int half = stage_width / 2;

  /* One rotation step for this stage; we'll multiply by it repeatedly. */
  ComplexNumber twiddle_step =
      complex_from_angle(-2.0f * (float)M_PI / (float)stage_width);

  for (int group_start = 0; group_start < N; group_start += stage_width) {

    /* Each group starts with no rotation, then turns a bit per step. */
    ComplexNumber twiddle = complex_one;

    for (int j = 0; j < half; j++) {
      int even_slot_index = group_start + j;
      int odd_slot_index = even_slot_index + half;

      /* Rotate the partner, then split into a sum and a difference. */
      ComplexNumber t = complex_mul(twiddle, buffer[odd_slot_index]);
      buffer[odd_slot_index] = complex_sub(buffer[even_slot_index], t);
      buffer[even_slot_index] = complex_add(buffer[even_slot_index], t);

      /* Turn the rotation forward by one step for the next pair. */
      twiddle = complex_mul(twiddle, twiddle_step);
    }
  }
}

/* §9  fft — the whole transform, in three lines.
 *
 * All the work lives in §7 and §8. The FFT itself is just: reorder the
 * input, then run the butterfly passes with the pairing distance doubling
 * each time. When it returns, buffer holds the spectrum. */

static void fft(ComplexNumber *buffer) {
  bit_reverse_permute(buffer);
  for (int stage_width = 2; stage_width <= N; stage_width <<= 1)
    run_butterfly_stage(buffer, stage_width);
}

/* §10  signal — build the test wave */

static void generate_cosine(float frequency_bin, float *output_signal) {
  /* A cosine at the given frequency. The frequency can be fractional; when
   * it is, the spectrum spike smears across nearby bins ("leakage"). That's
   * how the DFT behaves, not a bug. */
  for (int sample_index = 0; sample_index < N; sample_index++) {
    float phase_radians =
        2.0f * (float)M_PI * frequency_bin * (float)sample_index / (float)N;
    output_signal[sample_index] = cosf(phase_radians);
  }
}

/* §11  scene_state — the data the demo carries between frames */

static float g_signal[N];                       /* the input wave            */
static ComplexNumber g_fft_workspace_buffer[N]; /* spectrum from the FFT     */
static ComplexNumber g_dft_reference_buffer[N]; /* spectrum from the slow DFT*/
static float g_magnitude[N_HALF + 1];           /* bar height per bin        */
static float g_magnitude_peak = 1.0f;           /* tallest bar, for scaling  */
static float g_max_verification_error = 0.0f;   /* biggest FFT-vs-DFT gap     */

static float g_freq_bin = FREQ_LO;             /* current wave frequency     */
static bool g_simulation_paused = false;
static bool g_auto_sweep_enabled = true;       /* frequency moves on its own */
static float g_animation_phase_radians = 0.0f; /* where the sweep is in its cycle */

static bool g_show_phase_panel = false;        /* 'd' overlay                */
static bool g_show_arm_table = false;          /* 'D' overlay                */

static void scene_reset(void) {
  g_freq_bin = FREQ_LO;
  g_auto_sweep_enabled = true;
  g_animation_phase_radians = 0.0f;
  g_simulation_paused = false;
}

/* §12  scene_tick — do one frame's worth of work.
 *
 * Pick the frequency, build the wave, run the FFT, also run the slow DFT,
 * and measure how far the two answers differ. No drawing happens here; the
 * draw functions just read what this leaves behind. */
static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* Move the frequency along on its own (auto-sweep mode). */
  if (g_auto_sweep_enabled) {
    g_animation_phase_radians +=
        2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
    if (g_animation_phase_radians > 2.0f * (float)M_PI)
      g_animation_phase_radians -= 2.0f * (float)M_PI;

    /* Use a sine so the frequency eases back and forth smoothly between its
     * low and high limits, instead of jumping. */
    float sin_normalised_to_unit =
        (sinf(g_animation_phase_radians) + 1.0f) * 0.5f;
    g_freq_bin = FREQ_LO + sin_normalised_to_unit * (FREQ_HI - FREQ_LO);
  }

  generate_cosine(g_freq_bin, g_signal);

  /* The FFT works on complex numbers, so drop the real wave into the real
   * parts and leave the imaginary parts at zero. */
  for (int sample_index = 0; sample_index < N; sample_index++)
    g_fft_workspace_buffer[sample_index] =
        complex_make(g_signal[sample_index], 0.0f);

  fft(g_fft_workspace_buffer);

  /* Turn each bin into a bar height, and remember the tallest for scaling.
   * Only the first half is unique, so that's all we keep. */
  g_magnitude_peak = 1e-6f;
  for (int bin_index = 0; bin_index <= N_HALF; bin_index++) {
    g_magnitude[bin_index] =
        complex_magnitude(g_fft_workspace_buffer[bin_index]);
    if (g_magnitude[bin_index] > g_magnitude_peak)
      g_magnitude_peak = g_magnitude[bin_index];
  }

  /* Run the slow reference spectrum so we can check the FFT against it. */
  compute_dft_naive(g_signal, g_dft_reference_buffer);

  /* Find the worst disagreement between the two; the HUD reports it. */
  g_max_verification_error = 0.0f;
  for (int bin_index = 0; bin_index < N; bin_index++) {
    ComplexNumber difference = complex_sub(g_fft_workspace_buffer[bin_index],
                                           g_dft_reference_buffer[bin_index]);
    float bin_error = complex_magnitude(difference);
    if (bin_error > g_max_verification_error)
      g_max_verification_error = bin_error;
  }
}

/* §13  scene_input — change the frequency by hand */

static void scene_adjust_freq(float delta_bins) {
  /* Ignored while auto-sweep drives the frequency; otherwise nudge it and
   * keep it inside the allowed range. */
  if (g_auto_sweep_enabled)
    return;
  g_freq_bin += delta_bins;
  if (g_freq_bin < FREQ_LO)
    g_freq_bin = FREQ_LO;
  if (g_freq_bin > FREQ_HI)
    g_freq_bin = FREQ_HI;
}

/* §14  draw_bar — the one bar-drawing helper every panel shares */

static void draw_bar(int column, int baseline_row, int bar_height_cells,
                     bool growing_upward, int colour_pair) {
  /* Draw a vertical bar from a starting row, going up or down. */
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

/* §15  draw_signal — the top panel: the input wave */

static void draw_signal_panel(int top_row, int height_rows) {
  /* One bar per sample, measured from the centre line: positive samples
   * reach up, negative ones reach down. */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int columns_to_draw = (COLS < N) ? COLS : N;
  int spacing_per_sample = (COLS / N >= 2) ? 2 : 1;

  for (int sample_index = 0; sample_index < columns_to_draw; sample_index++) {

    float v = g_signal[sample_index];
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);

    for (int s = 0; s < spacing_per_sample; s++)
      draw_bar(sample_index * spacing_per_sample + s, midline_row, h, pos,
               PAIR_SIG);
  }
}

/* §16  draw_spectrum — the bottom panel: the spectrum */

static void draw_spectrum_panel(int top_row, int height_rows,
                                int *out_dominant_bin) {
  /* One bar per bin, rising from the panel floor. The loudest bin is
   * highlighted, and its index is handed back for the HUD. */
  int baseline_row = top_row + height_rows - 1;
  int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
  int max_bins = COLS / bin_w;
  if (max_bins > N_HALF + 1)
    max_bins = N_HALF + 1;

  int dominant_bin = 0;
  for (int bin_index = 1; bin_index <= N_HALF; bin_index++)
    if (g_magnitude[bin_index] > g_magnitude[dominant_bin])
      dominant_bin = bin_index;
  *out_dominant_bin = dominant_bin;

  for (int bin_index = 0; bin_index < max_bins; bin_index++) {
    float v = g_magnitude[bin_index] / (g_magnitude_peak + 1e-6f);
    int h = (int)(v * (float)height_rows + 0.5f);
    int pair = (bin_index == dominant_bin) ? PAIR_SPIKE : PAIR_SPEC;

    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(bin_index * bin_w + bx, baseline_row, h, true, pair);
  }
}

/* §17  draw_phase — the optional 'd' panel: each bin's phase.
 *
 * Phase is "where in its cycle" a bin sits — the part the magnitude bars
 * throw away. This overlay shows it: each bin gets a bar up (magenta) or
 * down (sky blue) from a centre line. */
static void draw_phase_panel(int top_row, int height_rows) {
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

  for (int bin_index = 0; bin_index < max_bins; bin_index++) {
    ComplexNumber z = g_fft_workspace_buffer[bin_index];
    /* A near-silent bin has meaningless phase, so don't draw one. */
    if (complex_magnitude(z) < 1e-3f)
      continue;

    float phase_radians = atan2f(z.im, z.re);
    float fraction = phase_radians / (float)M_PI;
    bool pos = (phase_radians >= 0.0f);
    int h = (int)(fabsf(fraction) * (float)half_height + 0.5f);

    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(bin_index * bin_w + bx, midline_row, h, pos,
               pos ? PAIR_PHASE_POS : PAIR_PHASE_NEG);
  }

  /* A line across the middle marks zero phase. */
  attron(COLOR_PAIR(PAIR_LABEL));
  for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

/* §18  draw_arm_table — the optional 'D' panel: the loudest bins as numbers.
 *
 * A small table in the corner listing the strongest bins with their exact
 * loudness and phase, so you can read off the values rather than eyeball the
 * bars. */
static void draw_arm_table(void) {
  /* We only need the top few, so a partial selection sort that pulls the
   * biggest to the front is enough — no need to sort the whole list. */
  int sorted_bin_indices[N_HALF + 1];
  for (int i = 0; i <= N_HALF; i++)
    sorted_bin_indices[i] = i;
  for (int i = 0; i < ARM_TABLE_ROWS && i <= N_HALF; i++) {
    int max_at = i;
    for (int j = i + 1; j <= N_HALF; j++)
      if (g_magnitude[sorted_bin_indices[j]] >
          g_magnitude[sorted_bin_indices[max_at]])
        max_at = j;
    int tmp = sorted_bin_indices[i];
    sorted_bin_indices[i] = sorted_bin_indices[max_at];
    sorted_bin_indices[max_at] = tmp;
  }

  int x = 2, y = 2;
  if (y + ARM_TABLE_ROWS + 1 >= LINES - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, " bin   |X[k]|     arg(rad)");
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int row = 0; row < ARM_TABLE_ROWS && row <= N_HALF; row++) {
    int bin_index = sorted_bin_indices[row];
    ComplexNumber z = g_fft_workspace_buffer[bin_index];
    float phase = atan2f(z.im, z.re);
    attron(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
    mvprintw(y + 1 + row, x, "  %2d   %7.3f   %+6.3f", bin_index,
             (double)g_magnitude[bin_index], (double)phase);
    attroff(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
  }
}

/* §19  hud — status line, key hint, and the frame composer */

static void draw_hud(int dominant_bin) {
  int dft_ops = N * N;
  int fft_ops = (N / 2) * LOG2_N;

  char status[200];
  snprintf(status, sizeof status,
           " FFT helloworld  N=%d  freq=%5.2f (peak bin=%d)  "
           "FFT %d vs DFT %d ops  err=%.1e %s  %s  %s ",
           N, (double)g_freq_bin, dominant_bin, fft_ops, dft_ops,
           (double)g_max_verification_error,
           (g_max_verification_error < 1e-4f) ? "[FFT == DFT]" : "[FFT != DFT]",
           g_auto_sweep_enabled ? "AUTO  " : "MANUAL",
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
           " q:quit  spc:pause  a:auto/manual  +/-:freq  ,/.:fine "
           " d:phase  D:bins  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Leave room for the status line, two panel labels, and the hint. */
  int rows_for_panels = LINES - 4;
  if (rows_for_panels < 6)
    rows_for_panels = 6;

  /* When the phase panel is on, it borrows some rows from the rest. */
  int phase_h = g_show_phase_panel ? rows_for_panels / 4 : 0;
  int main_h = rows_for_panels - phase_h;
  int sig_h = main_h / 2;
  int spec_h = main_h - sig_h;

  int sig_label_row = 1;
  int sig_top_row = 2;
  int spec_label_row = 2 + sig_h;
  int spec_top_row = 3 + sig_h;
  int phase_top_row = 3 + sig_h + spec_h; /* only used when phase panel is on */

  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(sig_label_row, 0, "Input x[n] = cos(2*pi * freq * n / N)");
  mvprintw(spec_label_row, 0, "FFT magnitude |X[k]|  (bins 0..%d)", N_HALF);
  if (g_show_phase_panel)
    mvprintw(phase_top_row - 1, 0,
             "FFT phase arg(X[k])  (magenta = +, sky = -)");
  attroff(COLOR_PAIR(PAIR_LABEL));

  draw_signal_panel(sig_top_row, sig_h);
  int dominant_bin = 0;
  draw_spectrum_panel(spec_top_row, spec_h, &dominant_bin);
  if (g_show_phase_panel)
    draw_phase_panel(phase_top_row, phase_h);

  /* Overlay goes over the panels but under the HUD. */
  if (g_show_arm_table)
    draw_arm_table();

  /* HUD last so it stays on top. */
  draw_hud(dominant_bin);
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* §20  app — signal handling, the main loop, and key handling */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* Only set a flag here — the real work happens back in the main loop,
   * which is the only safe thing to do from inside a signal handler. */
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
  case 'a':
  case 'A':
    g_auto_sweep_enabled = !g_auto_sweep_enabled;
    break;
  case '+':
  case '=':
    scene_adjust_freq(+1.0f);
    break;
  case '-':
    scene_adjust_freq(-1.0f);
    break;
  case '.':
    scene_adjust_freq(+0.1f);
    break;
  case ',':
    scene_adjust_freq(-0.1f);
    break;
  case 'd':
    g_show_phase_panel = !g_show_phase_panel;
    break;
  case 'D':
    g_show_arm_table = !g_show_arm_table;
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
      /* If we fell way behind (terminal was hidden, say), jump back to now
       * instead of trying to catch up frame by frame. */
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
