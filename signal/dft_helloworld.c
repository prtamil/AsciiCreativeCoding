/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * dft_helloworld.c — a live, smallest-possible Discrete Fourier Transform.
 * It feeds a moving cosine through the textbook DFT every frame and draws
 * the input wave on top and its frequency spectrum below, as bar charts.
 *
 * Sister files: signal/fft_helloworld.c (same demo, faster FFT instead),
 * signal/fft_vis.c (scaled up), signal/epicycles.c (DFT on 2-D curves).
 * Reference: Smith, "The Scientist and Engineer's Guide to DSP", ch. 8.
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

/* §1  config — every knob lives here */

#define N 32           /* how many samples we transform     */
#define N_HALF (N / 2) /* spectrum panel shows bins 0..N/2  */
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 90 /* one back-and-forth sweep is about 3 seconds */
#define FREQ_LO 1.0f           /* lowest frequency we sweep to / clamp at  */
#define FREQ_HI ((float)N * 0.5f - 1.0f) /* highest; N/2 is the fastest wave we can show */
#define ARM_TABLE_ROWS 8                 /* the 'D' table lists this many loudest bins */

/* Colour slots.  HUD / HINT / PHASE colours are fixed so the readouts stay
 * legible no matter what the animation is doing (per CLAUDE.md HUD Standard). */
enum {
  PAIR_SIG = 1,       /* time-domain wave bars                      */
  PAIR_SPEC = 2,      /* spectrum bars (medium magnitude)           */
  PAIR_SPIKE = 3,     /* spectrum bar at the dominant bin           */
  PAIR_LABEL = 4,     /* panel labels                               */
  PAIR_HUD = 5,       /* HUD top status (bright yellow + bold)      */
  PAIR_HINT = 6,      /* bottom hint  (bright cyan + bold)          */
  PAIR_PHASE_POS = 7, /* phase panel positive bars (magenta-pink)   */
  PAIR_PHASE_NEG = 8, /* phase panel negative bars (sky blue)       */
};

/* §2  clock — a steady timer and a sleep, both in nanoseconds */

/* A clock that only ever moves forward (never jumps when the system time is
 * adjusted) so frame timing stays smooth. */
static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  /* Without this the loop would peg a CPU core just to redraw 30 times/sec. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* §3  complex — a tiny complex-number type so the DFT reads like the math */

/* A complex number, which you can also picture as an arrow / point in the
 * plane: re is how far right, im is how far up.  We use our own 2-float
 * struct instead of <complex.h> so it prints cleanly in a debugger and the
 * handful of operations below stay obvious.  These six names are all the
 * complex math this file needs. */
typedef struct {
  float re; /* real part: the arrow's horizontal reach      */
  float im; /* imaginary part: the arrow's vertical reach   */
} ComplexNumber;

/* Plain zero (0 + 0i) — the starting value the DFT sum builds up from. */
static const ComplexNumber complex_zero = {0.0f, 0.0f};

static inline ComplexNumber complex_make(float real_part, float imag_part) {
  return (ComplexNumber){real_part, imag_part};
}

/* Turn an angle into a unit-length arrow pointing that way (Euler's formula).
 * This is the bridge that lets us "rotate by an angle" via a multiplication. */
static inline ComplexNumber complex_from_angle(float angle_radians) {
  return complex_make(cosf(angle_radians), sinf(angle_radians));
}

static inline ComplexNumber complex_add(ComplexNumber a, ComplexNumber b) {
  return complex_make(a.re + b.re, a.im + b.im);
}

/* Stretch an arrow by a plain number without turning it. */
static inline ComplexNumber complex_scale_by_real(float scale,
                                                  ComplexNumber z) {
  return complex_make(scale * z.re, scale * z.im);
}

/* How long the arrow is — for a spectrum bin, how loud that frequency is. */
static inline float complex_magnitude(ComplexNumber z) {
  return sqrtf(z.re * z.re + z.im * z.im);
}

/* §4  colors — set up the colour slots, with an 8-colour fallback */

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

/* §5  dft_naive — the heart of the file: the DFT, straight from its formula */

/*
 * For each frequency bin k, this asks "how much of frequency k is in the
 * signal?".  The trick: spin every sample backward by an amount that grows
 * with k, then add them all up.  If the signal really has frequency k, the
 * spins line all the samples up and the sum is big; otherwise they point
 * every which way and cancel out to nearly zero.  So a pure tone makes one
 * bin (and its mirror twin) light up while the rest stay dark.
 *
 * output_spectrum is the caller's array; we fill all N bins.  This is the
 * slow, obvious O(N^2) way on purpose — the faster FFT is in fft_helloworld.c.
 */
static void compute_dft_naive(const float *input_signal,
                              ComplexNumber *output_spectrum) {
  for (int output_bin_index = 0; output_bin_index < N; output_bin_index++) {

    ComplexNumber running_sum = complex_zero;

    for (int input_sample_index = 0; input_sample_index < N;
         input_sample_index++) {

      /* How far to spin this sample.  Bigger k means more turns; the minus
       * sign spins backward, which is just the standard forward-DFT choice. */
      float twist_angle_radians = -2.0f * (float)M_PI *
                                  (float)output_bin_index *
                                  (float)input_sample_index / (float)N;

      /* Spin the sample by that angle and add it in.  (The cos/sin that make
       * this the slow part live inside complex_from_angle.) */
      ComplexNumber twist = complex_from_angle(twist_angle_radians);
      ComplexNumber twisted =
          complex_scale_by_real(input_signal[input_sample_index], twist);

      running_sum = complex_add(running_sum, twisted);
    }

    output_spectrum[output_bin_index] = running_sum;
  }
}

/* §6  signal — make the test wave we feed the DFT */

/* Fill output_signal with one pure cosine.  frequency_bin says how many full
 * cycles fit in the N-sample window (1 = one slow cycle, N/2 = the fastest
 * wave we can represent).  It may be fractional — a non-whole frequency
 * smears the spectrum spike across nearby bins (spectral leakage). */
static void generate_cosine(float frequency_bin, float *output_signal) {
  for (int sample_index = 0; sample_index < N; sample_index++) {
    float phase_radians =
        2.0f * (float)M_PI * frequency_bin * (float)sample_index / (float)N;
    output_signal[sample_index] = cosf(phase_radians);
  }
}

/* §7  scene_state — everything the demo remembers between frames */

static float g_input_signal[N];         /* the wave we drew this frame */
static ComplexNumber g_dft_spectrum[N]; /* the DFT result, all N bins  */
static float g_magnitude[N_HALF + 1];   /* loudness of bins 0..N/2     */
static float g_magnitude_peak = 1.0f;   /* tallest bar, so panels self-scale */

static float g_freq_bin = FREQ_LO;             /* frequency of the current wave */
static bool g_simulation_paused = false;
static bool g_auto_sweep_enabled = true;       /* sweeping vs. driven by + / -  */
/* Where we are in the back-and-forth sweep.  Named "animation" to keep it
 * distinct from the DFT's own phase, which is a different thing entirely. */
static float g_animation_phase_radians = 0.0f;

static bool g_show_phase_panel = false; /* 'd' overlay */
static bool g_show_arm_table = false;   /* 'D' overlay */

static void scene_reset(void) {
  g_freq_bin = FREQ_LO;
  g_auto_sweep_enabled = true;
  g_animation_phase_radians = 0.0f;
  g_simulation_paused = false;
}

/* §8  scene_tick — move the demo forward one frame (no drawing here) */

/* Pick this frame's frequency, build the wave, transform it, then measure
 * each bin's loudness.  Paused freezes everything on the last result. */
static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* Sweep the frequency smoothly back and forth.  We drive it with sin (not
   * a straight ramp) so it eases at the turnarounds instead of snapping. */
  if (g_auto_sweep_enabled) {
    g_animation_phase_radians +=
        2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
    if (g_animation_phase_radians > 2.0f * (float)M_PI)
      g_animation_phase_radians -= 2.0f * (float)M_PI;

    float sin_normalised_to_unit =
        (sinf(g_animation_phase_radians) + 1.0f) * 0.5f;
    g_freq_bin = FREQ_LO + sin_normalised_to_unit * (FREQ_HI - FREQ_LO);
  }

  generate_cosine(g_freq_bin, g_input_signal);
  compute_dft_naive(g_input_signal, g_dft_spectrum);

  /* Loudness of each shown bin, plus the loudest of them so the bars can be
   * scaled to fill the panel.  Start the peak just above zero so dividing by
   * it later is always safe. */
  g_magnitude_peak = 1e-6f;
  for (int bin_index = 0; bin_index <= N_HALF; bin_index++) {
    g_magnitude[bin_index] = complex_magnitude(g_dft_spectrum[bin_index]);
    if (g_magnitude[bin_index] > g_magnitude_peak)
      g_magnitude_peak = g_magnitude[bin_index];
  }
}

/* §9  scene_input — nudge the frequency by hand (manual mode only) */

static void scene_adjust_freq(float delta_bins) {
  /* Does nothing while sweeping; otherwise keeps freq inside the legal range. */
  if (g_auto_sweep_enabled)
    return;
  g_freq_bin += delta_bins;
  if (g_freq_bin < FREQ_LO)
    g_freq_bin = FREQ_LO;
  if (g_freq_bin > FREQ_HI)
    g_freq_bin = FREQ_HI;
}

/* §10  draw_bar — the one bar-drawing helper every panel shares */

/* Draw a vertical bar in one column, starting at baseline_row and growing up
 * ('|') or down ('.').  Off-screen cells are skipped so resizing can't crash. */
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

/* §11  draw_signal — top panel: the input wave */

/* One bar per sample, rising or dropping from the panel's centre line.  The
 * wave never exceeds ±1, so we just scale it to half the panel height. */
static void draw_signal_panel(int top_row, int height_rows) {
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int columns_to_draw = (COLS < N) ? COLS : N;
  int spacing_per_sample = (COLS / N >= 2) ? 2 : 1;

  for (int sample_index = 0; sample_index < columns_to_draw; sample_index++) {

    float v = g_input_signal[sample_index];
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);

    for (int s = 0; s < spacing_per_sample; s++)
      draw_bar(sample_index * spacing_per_sample + s, midline_row, h, pos,
               PAIR_SIG);
  }
}

/* §12  draw_spectrum — bottom panel: how loud each frequency is */

/* One upward bar per bin (0..N/2), tallest = loudest.  The loudest bin gets a
 * brighter colour and is handed back so the HUD can name it. */
static void draw_spectrum_panel(int top_row, int height_rows,
                                int *out_dominant_bin) {
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

/* §13  draw_phase — optional 'd' panel: each bin's phase */

/* Phase is the other half of a bin — where in its cycle the wave starts.  The
 * loudness panel throws it away; this shows it.  Bars rise (magenta) for a
 * positive phase and drop (sky blue) for a negative one. */
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
    ComplexNumber z = g_dft_spectrum[bin_index];
    /* A near-silent bin has no meaningful phase, just noise — skip it. */
    if (complex_magnitude(z) < 1e-3f)
      continue;

    float phase_radians = atan2f(z.im, z.re);
    float fraction = phase_radians / (float)M_PI; /* now between -1 and 1, for the bar height */
    bool pos = (phase_radians >= 0.0f);
    int h = (int)(fabsf(fraction) * (float)half_height + 0.5f);

    for (int bx = 0; bx < bin_w - 1; bx++)
      draw_bar(bin_index * bin_w + bx, midline_row, h, pos,
               pos ? PAIR_PHASE_POS : PAIR_PHASE_NEG);
  }

  /* The centre line marks zero phase, so up vs. down reads at a glance. */
  attron(COLOR_PAIR(PAIR_LABEL));
  for (int x = 0; x < COLS && x < max_bins * bin_w; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

/* §14  draw_arm_table — optional 'D' panel: the loudest bins, as numbers */

/* A small table in the top-left listing the loudest few bins with their exact
 * loudness and phase — handy for checking results by eye. */
static void draw_arm_table(void) {
  /* We only need the top few bins, so we sort just those into place (a partial
   * selection sort) instead of ordering the whole list. */
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
    ComplexNumber z = g_dft_spectrum[bin_index];
    float phase = atan2f(z.im, z.re);
    attron(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
    mvprintw(y + 1 + row, x, "  %2d   %7.3f   %+6.3f", bin_index,
             (double)g_magnitude[bin_index], (double)phase);
    attroff(COLOR_PAIR(PAIR_SPIKE) | A_BOLD);
  }
}

/* §15  hud — top status line, bottom key hints, and the frame composer */

static void draw_hud(int dominant_bin) {
  char status[160];
  snprintf(status, sizeof status,
           " DFT helloworld  N=%d  freq=%5.2f (peak bin=%d)  %s  %s ", N,
           (double)g_freq_bin, dominant_bin,
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

  /* Leave 4 rows for the chrome: HUD, two panel labels, and the hint line. */
  int rows_for_panels = LINES - 4;
  if (rows_for_panels < 6)
    rows_for_panels = 6;

  /* When the phase panel is on it takes a quarter of the height from the rest. */
  int phase_h = g_show_phase_panel ? rows_for_panels / 4 : 0;
  int main_h = rows_for_panels - phase_h;
  int sig_h = main_h / 2;
  int spec_h = main_h - sig_h;

  int sig_label_row = 1;
  int sig_top_row = 2;
  int spec_label_row = 2 + sig_h;
  int spec_top_row = 3 + sig_h;
  int phase_top_row = 3 + sig_h + spec_h; /* only used when the phase panel is on */

  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(sig_label_row, 0, "Input x[n] = cos(2*pi * freq * n / N)");
  mvprintw(spec_label_row, 0, "DFT magnitude |X[k]|  (bins 0..%d)", N_HALF);
  if (g_show_phase_panel)
    mvprintw(phase_top_row - 1, 0,
             "DFT phase arg(X[k])  (magenta = +, sky = -)");
  attroff(COLOR_PAIR(PAIR_LABEL));

  draw_signal_panel(sig_top_row, sig_h);
  int dominant_bin = 0;
  draw_spectrum_panel(spec_top_row, spec_h, &dominant_bin);
  if (g_show_phase_panel)
    draw_phase_panel(phase_top_row, phase_h);

  if (g_show_arm_table)
    draw_arm_table();

  /* HUD last so it sits on top of everything else. */
  draw_hud(dominant_bin);
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* §16  app — signal handlers, the main loop, and key handling */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* A signal handler must do almost nothing, so we just raise a flag here and
   * let the main loop react to it when it's safe. */
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static void cleanup_screen(void) { endwin(); }

/* Acts on one keypress; hands back true only when the user asked to quit. */
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
      /* If we fell way behind (e.g. the terminal was hidden), skip ahead
       * instead of trying to render a backlog of frames all at once. */
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
