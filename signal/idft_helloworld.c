/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * idft_helloworld.c — run the inverse DFT and watch a spectrum turn
 * back into a signal.  Five modes show off what you can do once you
 * can edit the spectrum and reconstruct: verify the round trip, low-
 * pass, high-pass, keep only magnitude, keep only phase.
 *
 * The phase-only mode (4) is the famous Oppenheim & Lim 1981 result,
 * "The importance of phase in signals" (Proc. IEEE 69(5)).
 * Sister files: signal/dft_helloworld.c (the forward DFT, read first),
 * signal/fft_helloworld.c (same thing, faster), and
 * signal/convolution_helloworld.c (modes 1-2 done in the time domain).
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

/* ── §1  config ── */

#define N 32
#define N_HALF (N / 2)
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 90 /* one full cutoff sweep takes about 3 seconds */
#define ARM_TABLE_ROWS 8       /* 'D' table lists this many of the loudest bins */

enum {
  PAIR_INPUT = 1,     /* the input wave                          */
  PAIR_SPEC = 2,      /* spectrum bars                           */
  PAIR_SPEC_KILL = 3, /* spectrum bars the active mode zeroed    */
  PAIR_RECON = 4,     /* the reconstructed wave                  */
  PAIR_LABEL = 5,     /* panel labels                            */
  PAIR_HUD = 6,       /* HUD top status                          */
  PAIR_HINT = 7,      /* bottom hint                             */
  PAIR_PHASE_POS = 8, /* phase panel, positive bars              */
  PAIR_PHASE_NEG = 9, /* phase panel, negative bars              */
};

/* ── §2  clock ── */

static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  /* Without this the loop would burn a whole CPU core just to
   * repaint 30 times a second. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3  complex ── */

/*
 * A complex number, stored as its real and imaginary parts.  Picture
 * it as a little arrow on a 2D plane: re is how far right, im is how
 * far up.  The IDFT is just a pile of these arrows added together, so
 * having a tidy type for them lets every line below read like the math
 * instead of two lines of float juggling per step.
 */
typedef struct {
  float re; /* real part      — horizontal reach of the arrow */
  float im; /* imaginary part — vertical reach of the arrow   */
} ComplexNumber;

static const ComplexNumber complex_zero = {0.0f, 0.0f};

static inline ComplexNumber complex_make(float real_part, float imag_part) {
  return (ComplexNumber){real_part, imag_part};
}

/* Turn an angle into an arrow of length 1 pointing that way
 * (Euler's formula). */
static inline ComplexNumber complex_from_angle(float angle_radians) {
  return complex_make(cosf(angle_radians), sinf(angle_radians));
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re + b.re, a.im + b.im);
}

static inline ComplexNumber complex_sub(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re - b.re, a.im - b.im);
}

/* Multiplying two arrows adds their angles and multiplies their
 * lengths.  When one arrow has length 1, this just spins the other
 * one — which is exactly the twist the IDFT applies to each bin. */
static inline ComplexNumber complex_mul(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re);
}

static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z) {
  return complex_make(scale * z.re, scale * z.im);
}

/* Length of the arrow. */
static inline float complex_magnitude(ComplexNumber z) {
  return sqrtf(z.re * z.re + z.im * z.im);
}

/* ── §4  colors ── */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_INPUT, 51, -1);      /* bright cyan      */
    init_pair(PAIR_SPEC, 154, -1);      /* yellow-green     */
    init_pair(PAIR_SPEC_KILL, 240, -1); /* dim grey         */
    init_pair(PAIR_RECON, 213, -1);     /* magenta-pink     */
    init_pair(PAIR_LABEL, 244, -1);     /* mid grey         */
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow    */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan      */
    init_pair(PAIR_PHASE_POS, 213, -1); /* magenta-pink     */
    init_pair(PAIR_PHASE_NEG, 117, -1); /* sky blue         */
  } else {
    init_pair(PAIR_INPUT, COLOR_CYAN, -1);
    init_pair(PAIR_SPEC, COLOR_GREEN, -1);
    init_pair(PAIR_SPEC_KILL, COLOR_WHITE, -1);
    init_pair(PAIR_RECON, COLOR_MAGENTA, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_PHASE_POS, COLOR_MAGENTA, -1);
    init_pair(PAIR_PHASE_NEG, COLOR_BLUE, -1);
  }
}

/* ── §5  dft — forward DFT ── */

/* The forward transform: turn the wave into a spectrum.  At N = 32 the
 * plain double loop is fast enough, and it reads straight off the math.
 * The IDFT below is the same loop with the angle's sign flipped and a
 * 1/N at the end.  (See signal/dft_helloworld.c for the full story.) */
static void compute_dft(const float *input_signal,
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

/* ── §6  idft — inverse DFT, the star of this file ── */

/*
 * Rebuild the wave from its spectrum.  Each output sample is a sum
 * over every bin of "that bin's arrow, spun by an amount that depends
 * on where we are in time."  It's the forward DFT run backwards: spin
 * the arrows the other way and divide by N at the end, and you land
 * exactly back on the wave you started from.
 *
 * In:  input_spectrum[0..N-1]  — N bins (complex).
 * Out: output_signal[0..N-1]   — N samples the caller hands us room for.
 *      For the real waves we feed it the imaginary parts come out as
 *      basically zero, so the drawing code only ever looks at .re.
 * We don't touch the input and we don't allocate anything.
 */
static void compute_idft(const ComplexNumber *input_spectrum,
                         ComplexNumber *output_signal) {
  /* The divide-by-N at the end.  Without it the rebuilt wave comes
   * out N times too big. */
  float normalisation_one_over_N = 1.0f / (float)N;

  for (int output_sample_index = 0; output_sample_index < N;
       output_sample_index++) {

    ComplexNumber running_sum = complex_zero;

    for (int input_bin_index = 0; input_bin_index < N; input_bin_index++) {

      /* The +2*pi here (the forward DFT used -2*pi) is the one flip
       * that makes this the inverse: the arrows spin the other way. */
      float twist_angle_radians = +2.0f * (float)M_PI * (float)input_bin_index *
                                  (float)output_sample_index / (float)N;

      ComplexNumber twist = complex_from_angle(twist_angle_radians);

      ComplexNumber twisted =
          complex_mul(input_spectrum[input_bin_index], twist);

      running_sum = complex_add(running_sum, twisted);
    }

    output_signal[output_sample_index] =
        complex_scale_by_real(normalisation_one_over_N, running_sum);
  }
}

/* ── §7  signal_kinds — the test waves you can cycle through ── */

/* The five input waves, picked so each mode shows off something:
 * a square + mode 4 is the phase-only surprise, an impulse + mode 0
 * is the simplest possible round-trip check, and so on. */
typedef enum {
  SIG_SINES = 0, /* two cosines added together */
  SIG_SQUARE,    /* square wave                */
  SIG_SAWTOOTH,  /* sawtooth ramp              */
  SIG_IMPULSE,   /* a single spike at the middle */
  SIG_NOISE,     /* the same random-looking wave every time */
  SIG_COUNT
} SignalKind;

static const char *signal_name[SIG_COUNT] = {
    "Sum sines", "Square   ", "Sawtooth ", "Impulse  ", "Noise    "};

/* Fill output_signal[0..N-1] with the chosen wave.  Everything stays
 * within ±1 so the panels all scale the same way. */
static void generate_signal(SignalKind kind, float *output_signal) {
  switch (kind) {
  case SIG_SINES: {
    /* 3 cycles plus 7 cycles across the buffer — a clean, simple
     * spectrum, which makes it a good default. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      output_signal[n] = 0.6f * cosf(2.0f * (float)M_PI * 3.0f * t) +
                         0.4f * cosf(2.0f * (float)M_PI * 7.0f * t);
    }
    break;
  }
  case SIG_SQUARE: {
    /* Square wave, 4 cycles per buffer.  The sharp edges pack in
     * lots of high frequencies, so the filter modes (1-2) have a
     * lot to chew on. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      float arg = 2.0f * (float)M_PI * 4.0f * t;
      output_signal[n] = sinf(arg) >= 0.0f ? 0.7f : -0.7f;
    }
    break;
  }
  case SIG_SAWTOOTH: {
    /* A ramp that resets 4 times across the buffer.  Like the
     * square but richer — it has every frequency, not just the
     * odd ones. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      float u = 4.0f * t - floorf(4.0f * t);
      output_signal[n] = 1.4f * u - 0.7f;
    }
    break;
  }
  case SIG_IMPULSE: {
    /* Flat zero except for one spike in the middle.  A single
     * spike has a perfectly flat spectrum, which makes it the
     * cleanest round-trip test there is. */
    for (int n = 0; n < N; n++)
      output_signal[n] = 0.0f;
    output_signal[N / 2] = 1.0f;
    break;
  }
  case SIG_NOISE: {
    /* Random-looking, but seeded the same way every frame so the
     * picture holds still while you study a mode. */
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

/* ── §8  mode_kinds — what we do to the spectrum before rebuilding ── */

/* The five things the demo can do to the spectrum.  Each one is a
 * different little lesson about what magnitude and phase carry. */
typedef enum {
  MODE_ROUND_TRIP = 0, /* leave it alone — should rebuild the wave exactly */
  MODE_LOW_PASS,       /* drop the high bins — smooths the wave        */
  MODE_HIGH_PASS,      /* drop the low bins — keeps only sharp changes */
  MODE_MAG_ONLY,       /* throw away phase — wave collapses to a centred blob */
  MODE_PHASE_ONLY,     /* throw away magnitude — wave keeps its SHAPE  */
  MODE_COUNT
} Mode;

static const char *mode_name[MODE_COUNT] = {
    "round-trip ", "low-pass   ", "high-pass  ", "mag-only   ", "phase-only "};

/* ── §9  spectrum_mod — the one place the modes differ ── */

/*
 * Edit the spectrum in place according to the active mode.  This tiny
 * switch is the whole difference between the modes; everything else in
 * the program is shared.
 *
 * The two filters (low/high-pass) always zero bin k and its mirror
 * bin N-k together.  A real wave's spectrum is mirror-symmetric, and
 * keeping that symmetry is what guarantees the rebuilt wave stays
 * real instead of sprouting an imaginary part.  The mag-only and
 * phase-only modes deliberately break that symmetry; for the
 * symmetric test waves the rebuilt wave still comes out essentially
 * real anyway.
 */

/* For each bin, true if a filter mode just zeroed it.  The spectrum
 * panel paints these in dim grey so you can see the filter's shape. */
static bool g_bin_killed[N];

static void apply_spectrum_modification(ComplexNumber *spectrum, Mode mode,
                                        int cutoff_bin) {
  for (int k = 0; k < N; k++)
    g_bin_killed[k] = false;

  switch (mode) {
  case MODE_ROUND_TRIP: {
    /* Touch nothing.  Rebuilding from this should match the input
     * wave to within rounding error. */
    break;
  }

  case MODE_LOW_PASS: {
    /* Keep the low bins and their mirror twins; drop the middle. */
    for (int k = 0; k < N; k++) {
      if (k > cutoff_bin && k < N - cutoff_bin) {
        spectrum[k] = complex_zero;
        g_bin_killed[k] = true;
      }
    }
    break;
  }

  case MODE_HIGH_PASS: {
    /* The opposite: drop the low bins and their twins, keep the
     * middle. */
    for (int k = 0; k < N; k++) {
      if (k < cutoff_bin || k > N - cutoff_bin) {
        spectrum[k] = complex_zero;
        g_bin_killed[k] = true;
      }
    }
    break;
  }

  case MODE_MAG_ONLY: {
    /* Keep each bin's loudness, zero its phase.  Throwing away phase
     * throws away all timing info, so the rebuilt wave piles up into
     * a symmetric blob in the middle. */
    for (int k = 0; k < N; k++) {
      float magnitude = complex_magnitude(spectrum[k]);
      spectrum[k] = complex_make(magnitude, 0.0f);
    }
    break;
  }

  case MODE_PHASE_ONLY: {
    /* Keep each bin's phase, force its loudness to 1.  The surprise
     * (Oppenheim & Lim 1981): phase alone keeps the wave's shape. */
    for (int k = 0; k < N; k++) {
      float magnitude = complex_magnitude(spectrum[k]);
      if (magnitude > 1e-6f) {
        spectrum[k] = complex_scale_by_real(1.0f / magnitude, spectrum[k]);
      } else {
        /* Empty bin — nothing to normalise, so leave it at zero. */
        spectrum[k] = complex_zero;
      }
    }
    break;
  }

  default:
    break;
  }
}

/* ── §10  scene_state — the data each frame works on ── */

static float g_input_signal[N];              /* the wave we start from */
static ComplexNumber g_spectrum[N];          /* its spectrum           */
static ComplexNumber g_spectrum_modified[N]; /* spectrum after the mode edits it */
static ComplexNumber g_reconstruction[N];    /* wave rebuilt from that  */
static float g_magnitude[N_HALF + 1];        /* bar heights for the spectrum panel */
static float g_magnitude_peak = 1.0f;        /* tallest of those, for scaling */
static float g_round_trip_error = 0.0f;      /* how far the rebuild missed (mode 0) */
static float g_signal_peak = 1.0f;           /* scaling for the input panel */
static float g_recon_peak = 1.0f;            /* scaling for the rebuilt panel */

static SignalKind g_signal_kind = SIG_SINES;
static Mode g_mode = MODE_ROUND_TRIP;
static int g_cutoff_bin = 4; /* where the filters cut */
static bool g_simulation_paused = false;
static bool g_auto_sweep_cutoff = true;     /* slide the cutoff back and forth on its own */
static float g_sweep_phase_radians = 0.0f;  /* how far along that slide we are */

static bool g_show_phase_panel = false; /* 'd' overlay */
static bool g_show_arm_table = false;   /* 'D' overlay */

static void scene_reset(void) {
  g_signal_kind = SIG_SINES;
  g_mode = MODE_ROUND_TRIP;
  g_cutoff_bin = 4;
  g_simulation_paused = false;
  g_auto_sweep_cutoff = true;
  g_sweep_phase_radians = 0.0f;
}

/* ── §11  scene_tick — rebuild everything for one frame ── */

static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* In the filter modes, slide the cutoff smoothly back and forth so
   * you can watch the wave change without touching the keyboard. */
  bool mode_uses_cutoff =
      (g_mode == MODE_LOW_PASS) || (g_mode == MODE_HIGH_PASS);
  if (g_auto_sweep_cutoff && mode_uses_cutoff) {
    g_sweep_phase_radians += 2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
    if (g_sweep_phase_radians > 2.0f * (float)M_PI)
      g_sweep_phase_radians -= 2.0f * (float)M_PI;
    float s = (sinf(g_sweep_phase_radians) + 1.0f) * 0.5f;
    g_cutoff_bin = 1 + (int)(s * ((float)N * 0.5f - 2.0f) + 0.5f);
  }

  /* The pipeline: make a wave, transform it, edit the spectrum,
   * transform back. */
  generate_signal(g_signal_kind, g_input_signal);
  compute_dft(g_input_signal, g_spectrum);

  for (int k = 0; k < N; k++)
    g_spectrum_modified[k] = g_spectrum[k];
  apply_spectrum_modification(g_spectrum_modified, g_mode, g_cutoff_bin);

  compute_idft(g_spectrum_modified, g_reconstruction);

  /* Find the tallest value in each panel so the bars fill the space
   * no matter how loud or quiet the wave is. */
  g_magnitude_peak = 1e-6f;
  for (int k = 0; k <= N_HALF; k++) {
    g_magnitude[k] = complex_magnitude(g_spectrum_modified[k]);
    if (g_magnitude[k] > g_magnitude_peak)
      g_magnitude_peak = g_magnitude[k];
  }
  g_signal_peak = 1e-6f;
  for (int n = 0; n < N; n++) {
    float a = fabsf(g_input_signal[n]);
    if (a > g_signal_peak)
      g_signal_peak = a;
  }
  g_recon_peak = 1e-6f;
  for (int n = 0; n < N; n++) {
    float a = fabsf(g_reconstruction[n].re);
    if (a > g_recon_peak)
      g_recon_peak = a;
  }

  /* Only the round-trip mode has a "right answer" to compare against,
   * so only it reports an error. */
  g_round_trip_error = 0.0f;
  if (g_mode == MODE_ROUND_TRIP) {
    for (int n = 0; n < N; n++) {
      float diff = g_reconstruction[n].re - g_input_signal[n];
      float err = fabsf(diff);
      if (err > g_round_trip_error)
        g_round_trip_error = err;
    }
  }
}

/* ── §12  scene_input — keys that change what we show ── */

static void scene_cycle_mode(int direction) {
  g_mode = (Mode)((g_mode + direction + MODE_COUNT) % MODE_COUNT);
}

static void scene_cycle_signal(int direction) {
  g_signal_kind =
      (SignalKind)((g_signal_kind + direction + SIG_COUNT) % SIG_COUNT);
}

static void scene_adjust_cutoff(int delta) {
  /* Manual nudges only count when the auto-slide is off. */
  if (g_auto_sweep_cutoff)
    return;
  g_cutoff_bin += delta;
  if (g_cutoff_bin < 1)
    g_cutoff_bin = 1;
  if (g_cutoff_bin > N_HALF - 1)
    g_cutoff_bin = N_HALF - 1;
}

/* ── §13  draw_bar — one vertical bar ── */

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

/* ── §14  draw_signals — the input and rebuilt wave panels ── */

static void draw_signal_panel(const float *signal, int top_row, int height_rows,
                              float peak, int colour_pair) {
  /* Draw the wave as bars from a centre line: above for positive
   * samples, below for negative. */
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

static void draw_complex_real_panel(const ComplexNumber *signal, int top_row,
                                    int height_rows, float peak,
                                    int colour_pair) {
  /* Same as draw_signal_panel, but for the rebuilt wave, which is
   * stored as complex numbers.  We only draw the real part — the
   * imaginary part is just rounding noise. */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int spacing = (COLS / N >= 2) ? 2 : 1;
  int columns = (COLS / spacing < N) ? COLS / spacing : N;

  for (int n = 0; n < columns; n++) {
    float v = signal[n].re / (peak + 1e-6f);
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    for (int s = 0; s < spacing; s++)
      draw_bar(n * spacing + s, midline_row, h, pos, colour_pair);
  }
}

/* ── §15  draw_spectrum — the spectrum, phase, and bin-table panels ── */

static void draw_spectrum_panel(int top_row, int height_rows) {
  /* One upward bar per bin.  Bins the active filter killed go dim
   * grey, so you can see the filter's shape. */
  int baseline_row = top_row + height_rows - 1;
  int bin_w = (COLS / (N_HALF + 1) >= 3) ? 3 : 2;
  int max_bins = COLS / bin_w;
  if (max_bins > N_HALF + 1)
    max_bins = N_HALF + 1;

  for (int k = 0; k < max_bins; k++) {
    float v = g_magnitude[k] / (g_magnitude_peak + 1e-6f);
    int h = (int)(v * (float)height_rows + 0.5f);
    int pair = g_bin_killed[k] ? PAIR_SPEC_KILL : PAIR_SPEC;

    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(k * bin_w + bx, baseline_row, h, true, pair);
  }

  /* A ':' column marking where the filter cuts (filter modes only). */
  bool mode_uses_cutoff =
      (g_mode == MODE_LOW_PASS) || (g_mode == MODE_HIGH_PASS);
  if (mode_uses_cutoff && g_cutoff_bin <= max_bins) {
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

static void draw_phase_panel(int top_row, int height_rows) {
  /* The 'd' overlay: each bin's phase (its angle) as a bar from a
   * centre line — pink for positive, blue for negative.  This is the
   * half of the spectrum the magnitude panel above doesn't show.
   * Near-silent bins are skipped, since their angle is just noise. */
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
    ComplexNumber z = g_spectrum_modified[bin_index];
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

  attron(COLOR_PAIR(PAIR_LABEL));
  for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

static void draw_arm_table_overlay(void) {
  /* The 'D' overlay: a little table of the loudest bins with their
   * exact numbers, for checking the picture by hand. */
  if (!g_show_arm_table)
    return;

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
    ComplexNumber z = g_spectrum_modified[bin_index];
    float phase = atan2f(z.im, z.re);
    attron(COLOR_PAIR(PAIR_SPEC) | A_BOLD);
    mvprintw(y + 1 + row, x, "  %2d   %7.3f   %+6.3f", bin_index,
             (double)g_magnitude[bin_index], (double)phase);
    attroff(COLOR_PAIR(PAIR_SPEC) | A_BOLD);
  }
}

/* ── §16  hud — status line, key hints, and the frame composer ── */

static void draw_hud(void) {
  char status[200];
  bool mode_uses_cutoff =
      (g_mode == MODE_LOW_PASS) || (g_mode == MODE_HIGH_PASS);

  if (g_mode == MODE_ROUND_TRIP) {
    snprintf(status, sizeof status,
             " IDFT helloworld  N=%d  signal:%s  mode:%s  "
             "round-trip err=%.1e %s  %s ",
             N, signal_name[g_signal_kind], mode_name[g_mode],
             (double)g_round_trip_error,
             (g_round_trip_error < 1e-4f) ? "[PASS]" : "[FAIL]",
             g_simulation_paused ? "PAUSED" : "      ");
  } else if (mode_uses_cutoff) {
    snprintf(status, sizeof status,
             " IDFT helloworld  N=%d  signal:%s  mode:%s  "
             "cutoff=%2d  %s  %s ",
             N, signal_name[g_signal_kind], mode_name[g_mode], g_cutoff_bin,
             g_auto_sweep_cutoff ? "AUTO  " : "MANUAL",
             g_simulation_paused ? "PAUSED" : "      ");
  } else {
    snprintf(status, sizeof status,
             " IDFT helloworld  N=%d  signal:%s  mode:%s  %s ", N,
             signal_name[g_signal_kind], mode_name[g_mode],
             g_simulation_paused ? "PAUSED" : "      ");
  }

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
           " q:quit  spc:pause  m/M:mode  s/S:signal  a:auto/manual "
           " +/-:cutoff  d:phase  D:bins  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Hand out the rows: 5 go to the HUD, labels, and hint; the rest
   * split between the panels.  The phase panel borrows some when it's on. */
  int rows_for_panels = LINES - 5;
  if (rows_for_panels < 9)
    rows_for_panels = 9;
  int phase_h = g_show_phase_panel ? rows_for_panels / 5 : 0;
  int main_h = rows_for_panels - phase_h;
  int input_h = main_h / 3;
  int spec_h = main_h / 3;
  int recon_h = main_h - input_h - spec_h;

  int input_label_row = 1;
  int input_top_row = 2;
  int spec_label_row = 2 + input_h;
  int spec_top_row = 3 + input_h;
  int recon_label_row = 3 + input_h + spec_h;
  int recon_top_row = 4 + input_h + spec_h;
  int phase_top_row = 4 + input_h + spec_h + recon_h;

  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(input_label_row, 0, "Input x[n]");
  mvprintw(spec_label_row, 0,
           "Spectrum |X[k]|  (dim grey = killed by mode, ':' = cutoff)");
  mvprintw(recon_label_row, 0, "Reconstruction y[n] = IDFT(modified X)");
  if (g_show_phase_panel)
    mvprintw(phase_top_row - 1, 0, "Phase arg(X[k])  (magenta = +, sky = -)");
  attroff(COLOR_PAIR(PAIR_LABEL));

  draw_signal_panel(g_input_signal, input_top_row, input_h, g_signal_peak,
                    PAIR_INPUT);
  draw_spectrum_panel(spec_top_row, spec_h);
  draw_complex_real_panel(g_reconstruction, recon_top_row, recon_h,
                          g_recon_peak, PAIR_RECON);
  if (g_show_phase_panel)
    draw_phase_panel(phase_top_row, phase_h);

  /* Overlays go on top of the panels, the HUD on top of everything. */
  draw_arm_table_overlay();
  draw_hud();
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §17  app — signals, the main loop, and key handling ── */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* A signal handler can't safely do much, so just flip a flag and
   * let the main loop deal with it. */
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
  case 'm':
    scene_cycle_mode(+1);
    break;
  case 'M':
    scene_cycle_mode(-1);
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
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
