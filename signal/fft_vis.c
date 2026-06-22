/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fft_vis.c — a live FFT explorer: feed it a waveform and watch three
 * stacked panels — the signal itself, the frequencies hiding inside it,
 * and a scrolling history of those frequencies over time.  Seven signal
 * shapes, four smoothing windows to play with leakage.
 *
 * Sister file: signal/epicycles.c runs the same transform on a 2-D curve.
 * Algorithm: Cooley & Tukey (1965), Math. Comp. 19, 297-301; windows from
 * Harris (1978), Proc. IEEE 66(1).
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

/* ── §1  config — all the tunable numbers live here ── */

/* How many samples we transform per frame.  Must be a power of two
 * (the FFT below relies on it), and LOG2_N_FFT must stay its log base 2
 * — they're separate so the FFT never has to compute a log at runtime. */
#define N_FFT 128
#define LOG2_N_FFT 7

#define N_SINE_COMPONENTS 3

#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)

/* The 'n' key dumps in a burst of random noise that fades over ~1 second. */
#define NOISE_AMP_PEAK 0.30f
#define NOISE_DECAY_RATE 0.03f

/* How fast the time waveform drifts sideways each frame (purely cosmetic). */
#define ANIMATION_PHASE_STEP_RAD 0.06f

/* How many past spectra the scrolling waterfall remembers. */
#define SPEC_HISTORY_LEN 120

#define N_THEMES 5
#define N_WINDOWS 4
#define N_SIGNAL_MODES 7

/* Colour-pair slots, bound in §11.  ncurses keeps pair 0 for the default
 * background, so ours start at 1.  HUD and HINT stay the same across every
 * theme so the on-screen text is always readable. */
enum {
  PAIR_BG = 1,
  PAIR_TIME_POS,  /* time wave, above the midline */
  PAIR_TIME_NEG,  /* time wave, below the midline */
  PAIR_FREQ_LOW,  /* quiet frequency bars         */
  PAIR_FREQ_HIGH, /* loud frequency bars          */
  PAIR_HUD,       /* yellow + bold, fixed         */
  PAIR_HINT,      /* cyan + bold, fixed           */
};

/* ── §2  clock — a forward-only timer and a sleep ── */

/* CLOCK_MONOTONIC only ever counts up — clock changes (NTP, daylight
 * saving) can't make it jump — so the main loop can safely subtract two
 * readings to measure how long is left before the next frame. */
static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* Idle until the next frame is due, instead of spinning the CPU flat out. */
static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3  bit_reverse — reshuffle the input before the FFT runs ──
 *
 * The iterative FFT below works on neighbouring pairs and expects each
 * pair to be the right "even sample / odd sample" couple.  They only
 * line up if we first reorder the input: the sample at index i moves to
 * the index whose bits are i's read backwards (so for N=8, index 1 = 001
 * goes to 100 = 4, index 3 = 011 goes to 110 = 6, and so on).  One cheap
 * pass, run once per FFT. */

/* Read the low `bit_count` bits of `value` backwards — write the binary
 * digits down, then read them right to left. */
static int reverse_low_bits(int value, int bit_count) {
  int reversed = 0;
  for (int b = 0; b < bit_count; b++)
    reversed |= ((value >> b) & 1) << (bit_count - 1 - b);
  return reversed;
}

/* Swap each sample into its bit-reversed slot.  The `j > i` guard means
 * every pair is swapped exactly once instead of swapped back again. */
static void bit_reverse_permute(float *real_buffer, float *imag_buffer,
                                int sample_count) {
  for (int i = 0; i < sample_count; i++) {
    int j = reverse_low_bits(i, LOG2_N_FFT);
    if (j > i) {
      float tmp;
      tmp = real_buffer[i];
      real_buffer[i] = real_buffer[j];
      real_buffer[j] = tmp;
      tmp = imag_buffer[i];
      imag_buffer[i] = imag_buffer[j];
      imag_buffer[j] = tmp;
    }
  }
}

/* ── §4  butterfly_stage — one round of the FFT ──
 *
 * Each round chops the array into equal groups of `stage_width` slots.
 * Inside a group, the first half are "even" results and the second half
 * "odd"; we pair them up and combine each pair with a rotation factor
 * (the "twiddle").  Rounds get wider until one group covers everything.
 *
 * The twiddle is the same rotation raised to a growing power, W, W^2,
 * W^3...  Rather than call sin/cos each step (slow), we set up W once and
 * keep multiplying by it — one trig call per round instead of one per
 * pair. */
static void run_butterfly_stage(float *real_buffer, float *imag_buffer,
                                int sample_count, int stage_width) {
  int half = stage_width >> 1;
  float twiddle_step_angle = -2.0f * (float)M_PI / (float)stage_width;
  float twiddle_step_re = cosf(twiddle_step_angle);
  float twiddle_step_im = sinf(twiddle_step_angle);

  for (int i = 0; i < sample_count; i += stage_width) {
    float twiddle_re = 1.0f; /* start the rotation at no rotation */
    float twiddle_im = 0.0f;

    for (int j = 0; j < half; j++) {
      /* rotate the odd partner: complex multiply spelled out as
       * (a+bi)(c+di) = (ac - bd) + (ad + bc)i */
      float t_re = twiddle_re * real_buffer[i + j + half] -
                   twiddle_im * imag_buffer[i + j + half];
      float t_im = twiddle_re * imag_buffer[i + j + half] +
                   twiddle_im * real_buffer[i + j + half];

      /* the two outputs are the even partner plus/minus the rotated odd */
      real_buffer[i + j + half] = real_buffer[i + j] - t_re;
      imag_buffer[i + j + half] = imag_buffer[i + j] - t_im;

      real_buffer[i + j] += t_re;
      imag_buffer[i + j] += t_im;

      /* bump the rotation up one power; stash the new real part first so
       * we don't overwrite it before computing the new imaginary part */
      float next_twiddle_re =
          twiddle_re * twiddle_step_re - twiddle_im * twiddle_step_im;
      twiddle_im = twiddle_re * twiddle_step_im + twiddle_im * twiddle_step_re;
      twiddle_re = next_twiddle_re;
    }
  }
}

/* ── §5  fft — the whole transform, in place ──
 *
 * Reorder the input, then run wider and wider rounds (groups of 2, 4, 8,
 * ... up to N).  When it's done the same arrays hold the frequency
 * result instead of the original samples. */
static void fft(float *real_buffer, float *imag_buffer, int sample_count) {
  bit_reverse_permute(real_buffer, imag_buffer, sample_count);
  for (int stage_width = 2; stage_width <= sample_count; stage_width <<= 1)
    run_butterfly_stage(real_buffer, imag_buffer, sample_count, stage_width);
}

/* ── §6  windows — smooth the signal's ends to fight leakage ──
 *
 * The FFT secretly assumes the buffer repeats forever, so if the wave
 * doesn't end where it began there's a jump at the seam, and that jump
 * smears one clean frequency spike across many bins ("leakage").  A
 * window is a bell-shaped fade that eases the signal down to near-zero at
 * both ends, removing the jump.  The cost is that every spike gets a
 * little wider.  Rectangular = no fade; the others fade progressively
 * more.  Try the 'w' key on an off-bin frequency to see the difference. */

typedef enum {
  WINDOW_RECT = 0,
  WINDOW_HANN,
  WINDOW_HAMMING,
  WINDOW_BLACKMAN,
} WindowKind;

static const char *window_name[N_WINDOWS] = {
    "Rectangular",
    "Hann       ",
    "Hamming    ",
    "Blackman   ",
};

/* The fade multiplier for sample `n`.  Shared so the 'd' overlay can draw
 * the bell shape using exactly what the FFT applies. */
static float window_coefficient(WindowKind kind, int n, int N) {
  if (kind == WINDOW_RECT)
    return 1.0f;
  float two_pi_over_M = 2.0f * (float)M_PI / (float)(N - 1);
  switch (kind) {
  case WINDOW_HANN:
    return 0.5f - 0.5f * cosf(two_pi_over_M * (float)n);
  case WINDOW_HAMMING:
    return 0.54f - 0.46f * cosf(two_pi_over_M * (float)n);
  case WINDOW_BLACKMAN:
    return 0.42f - 0.5f * cosf(two_pi_over_M * (float)n) +
           0.08f * cosf(2.0f * two_pi_over_M * (float)n);
  default:
    return 1.0f;
  }
}

/* Fade the signal in place.  Run on the FFT's copy only, so the time
 * panel can still show the original un-faded waveform. */
static void apply_window(float *signal, int N, WindowKind kind) {
  if (kind == WINDOW_RECT)
    return;
  for (int n = 0; n < N; n++)
    signal[n] *= window_coefficient(kind, n, N);
}

/* ── §7  signal_helpers ── */

/* Where are we inside the current cycle, as a fraction from 0 up to 1?
 * Shared by the square, sawtooth, and triangle generators below. */
static inline float periodic_phase(int sample_index, float frequency_bin,
                                   float animation_phase_radians) {
  float t = frequency_bin * (float)sample_index / (float)N_FFT +
            animation_phase_radians * frequency_bin / (2.0f * (float)M_PI);
  return t - floorf(t);
}

/* ── §8  signal_modes — one generator per waveform shape ──
 *
 * Each returns a single sample, roughly in -1..1, for the given index.
 * The whole point of trying different shapes is that each leaves its own
 * recognisable pattern of spikes in the frequency panel. */

/* Flips between +1 and -1 once per cycle.  Shows only odd harmonics. */
static float square_sample(int n, float freq, float anim_phase) {
  return periodic_phase(n, freq, anim_phase) < 0.5f ? 1.0f : -1.0f;
}

/* Ramps up then snaps back.  Shows every harmonic, fading as 1/n. */
static float sawtooth_sample(int n, float freq, float anim_phase) {
  return 2.0f * periodic_phase(n, freq, anim_phase) - 1.0f;
}

/* Up then down in straight lines.  Odd harmonics that die off fast. */
static float triangle_sample(int n, float freq, float anim_phase) {
  float p = periodic_phase(n, freq, anim_phase);
  return 4.0f * fabsf(p - 0.5f) - 1.0f;
}

/* A tone that slides from frequency lo up to hi across the buffer.  Paints
 * a clean diagonal streak in the scrolling waterfall. */
static float chirp_sample(int n, float lo, float hi) {
  float t = (float)n / (float)N_FFT;
  float phase_cycles = lo * t + (hi - lo) * t * t * 0.5f;
  return sinf(2.0f * (float)M_PI * phase_cycles);
}

/* A carrier tone whose loudness wobbles at a slower rate (like AM radio).
 * Shows three spikes: the carrier plus one on each side. */
static float am_sample(int n, float carrier_freq, float mod_freq,
                       float anim_phase) {
  float t = (float)n / (float)N_FFT;
  float modulator =
      cosf(2.0f * (float)M_PI * mod_freq * t + anim_phase * mod_freq);
  float carrier_s =
      cosf(2.0f * (float)M_PI * carrier_freq * t + anim_phase * carrier_freq);
  return (1.0f + 0.5f * modulator) * carrier_s * 0.55f;
}

/* A single blip every `period` samples — a row of evenly spaced clicks.
 * Evenly spaced clicks in time give evenly spaced spikes in frequency. */
static float impulse_sample(int n, int period) {
  if (period <= 0)
    period = 1;
  return (n % period == 0) ? 0.85f : 0.0f;
}

/* ── §9  signal_dispatch — pick a generator and fill the buffer ── */

/* One sine wave in the "sine sum" mode: its pitch, its loudness, and
 * whether it's currently switched on (keys 1/2/3). */
typedef struct {
  float frequency_bin; /* which bin it lands on, 1 .. N/2-1 */
  float amplitude;     /* loudness, 0 .. 1                   */
  bool enabled;        /* is this component switched on?     */
} SineComponent;

typedef enum {
  MODE_SINE_SUM = 0,
  MODE_SQUARE,
  MODE_SAWTOOTH,
  MODE_TRIANGLE,
  MODE_CHIRP,
  MODE_AM,
  MODE_IMPULSE,
} SignalMode;

static const char *mode_name[N_SIGNAL_MODES] = {
    "Sine sum", "Square  ", "Sawtooth", "Triangle",
    "Chirp   ", "AM      ", "Impulse ",
};

/* Everything the user can tweak about the signal.  Only the fields that
 * matter for the current `mode` are used; the rest just hold their last
 * values so switching modes back and forth keeps your settings. */
typedef struct {
  SignalMode mode;                            /* which waveform is active */
  SineComponent sine_components[N_SINE_COMPONENTS]; /* the sine-sum mode    */
  float frequency_single_bin;                 /* pitch for square/saw/tri */
  float chirp_lo_bin;                         /* chirp: start frequency   */
  float chirp_hi_bin;                         /* chirp: end frequency     */
  float am_carrier_bin;                       /* AM: the main tone        */
  float am_modulator_bin;                     /* AM: the wobble rate      */
  int impulse_period_samples;                 /* impulse: samples per blip */
} SignalParams;

/* The 'n' key fills this with random values and sets the fade to 1; every
 * frame build_signal adds it in and the fade shrinks toward 0. */
static float g_noise_buffer[N_FFT];
static float g_noise_decay_envelope = 0.0f;

/* Fill out_signal with one buffer of the chosen waveform, plus whatever's
 * left of the noise burst on top. */
static void build_signal(const SignalParams *p, float *out_signal,
                         float anim_phase) {
  for (int n = 0; n < N_FFT; n++) {
    float v = 0.0f;
    switch (p->mode) {
    case MODE_SINE_SUM: {
      for (int c = 0; c < N_SINE_COMPONENTS; c++) {
        if (!p->sine_components[c].enabled)
          continue;
        float freq = p->sine_components[c].frequency_bin;
        float t = (float)n / (float)N_FFT;
        v += p->sine_components[c].amplitude *
             sinf(2.0f * (float)M_PI * freq * t + anim_phase * freq);
      }
      break;
    }
    case MODE_SQUARE:
      v = 0.7f * square_sample(n, p->frequency_single_bin, anim_phase);
      break;
    case MODE_SAWTOOTH:
      v = 0.7f * sawtooth_sample(n, p->frequency_single_bin, anim_phase);
      break;
    case MODE_TRIANGLE:
      v = 0.8f * triangle_sample(n, p->frequency_single_bin, anim_phase);
      break;
    case MODE_CHIRP:
      v = 0.8f * chirp_sample(n, p->chirp_lo_bin, p->chirp_hi_bin);
      break;
    case MODE_AM:
      v = am_sample(n, p->am_carrier_bin, p->am_modulator_bin, anim_phase);
      break;
    case MODE_IMPULSE:
      v = impulse_sample(n, p->impulse_period_samples);
      break;
    }
    v += g_noise_buffer[n] * g_noise_decay_envelope;
    out_signal[n] = v;
  }
}

/* ── §10  themes — colour palettes you cycle with 't' ──
 *
 * Five sets of colours.  Every code sits in the bright half of the
 * 256-colour range so even the dimmest bars stay visible on a black
 * terminal. */

/* One palette: a name plus the colour for each kind of bar. */
typedef struct {
  const char *display_name;
  short time_pos_color;   /* time wave above the line */
  short time_neg_color;   /* time wave below the line */
  short freq_low_color;   /* quiet frequency bars     */
  short freq_high_color;  /* loud frequency bars      */
  short reserved_color;   /* spare slot, not used yet */
} ThemePalette;

static const ThemePalette theme_table[N_THEMES] = {
    {"Cyan-Green", 51, 39, 46, 82, 82},
    {"Fire      ", 196, 202, 208, 226, 226},
    {"Purple    ", 201, 171, 141, 231, 231},
    {"Mono      ", 252, 245, 244, 255, 255},
    {"Ocean     ", 195, 117, 75, 39, 39},
};

/* ── §11  colors — ncurses colour-pair setup ── */

/* Repoint the four animation colours at a different palette.  The fixed
 * HUD and HINT colours are deliberately left alone. */
static void apply_theme(int theme_index) {
  if (theme_index < 0 || theme_index >= N_THEMES)
    theme_index = 0;
  const ThemePalette *t = &theme_table[theme_index];
  init_pair(PAIR_BG, -1, -1);
  init_pair(PAIR_TIME_POS, t->time_pos_color, -1);
  init_pair(PAIR_TIME_NEG, t->time_neg_color, -1);
  init_pair(PAIR_FREQ_LOW, t->freq_low_color, -1);
  init_pair(PAIR_FREQ_HIGH, t->freq_high_color, -1);
}

/* Set up the fixed HUD/HINT colours once, then load the starting theme. */
static void colors_init(int initial_theme_index) {
  start_color();
  use_default_colors();
  init_pair(PAIR_HUD, 226, -1); /* bright yellow */
  init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  apply_theme(initial_theme_index);
}

/* ── §12  draw_bar — stack glyphs into one vertical bar ──
 *
 * Draws a column of characters from the baseline up ('|') or down ('.').
 * The two different glyphs let you tell positive from negative at a
 * glance, even before the colour registers. */
static void draw_bar(int column, int baseline_row, int bar_height_cells,
                     bool growing_upward, int colour_pair) {
  if (bar_height_cells <= 0)
    return;
  attron(COLOR_PAIR(colour_pair));
  for (int dy = 0; dy < bar_height_cells; dy++) {
    int row = growing_upward ? (baseline_row - dy) : (baseline_row + dy + 1);
    if (row < 0 || row >= LINES)
      continue;
    if (column < 0 || column >= COLS)
      continue;
    mvaddch(row, column, growing_upward ? '|' : '.');
  }
  attroff(COLOR_PAIR(colour_pair));
}

/* ── §13  spectrogram — the scrolling history of past spectra ──
 *
 * Each frame's spectrum becomes one row, stored as small intensity
 * levels (0..4) rather than raw numbers, so drawing is just a quick
 * lookup.  It's a ring buffer: new rows overwrite the oldest, and `head`
 * marks where the next row goes while `count` says how many are filled. */

#define DEBUG_NOTE_STUB_so_section_below_is_clean 1

static unsigned char spec_history_buffer[SPEC_HISTORY_LEN][N_FFT / 2];
static int spec_history_head_index = 0;
static int spec_history_filled_count = 0;

/* Bucket a magnitude (0..1 against the loudest bin) into one of five
 * brightness levels for the waterfall glyphs. */
static unsigned char quantise_intensity(float fraction_of_max) {
  if (fraction_of_max < 0.10f)
    return 0;
  if (fraction_of_max < 0.30f)
    return 1;
  if (fraction_of_max < 0.55f)
    return 2;
  if (fraction_of_max < 0.80f)
    return 3;
  return 4;
}

/* Record this frame's spectrum as the newest row in the history. */
static void spec_history_push(const float *magnitude, float magnitude_peak) {
  for (int k = 0; k < N_FFT / 2; k++) {
    float frac = magnitude[k] / (magnitude_peak + 1e-6f);
    spec_history_buffer[spec_history_head_index][k] = quantise_intensity(frac);
  }
  spec_history_head_index = (spec_history_head_index + 1) % SPEC_HISTORY_LEN;
  if (spec_history_filled_count < SPEC_HISTORY_LEN)
    spec_history_filled_count++;
}

/* Wipe the history — used on reset, resize, and mode/waterfall toggles so
 * stale rows from the old setup don't linger. */
static void spec_history_clear(void) {
  spec_history_head_index = 0;
  spec_history_filled_count = 0;
  memset(spec_history_buffer, 0, sizeof spec_history_buffer);
}

/* Paint the history: newest spectrum along the bottom, older ones
 * climbing upward, frequency running left to right. */
static void draw_waterfall(int top_row, int left_col, int height_rows,
                           int width_cols) {
  if (height_rows <= 0)
    return;
  if (width_cols > N_FFT / 2)
    width_cols = N_FFT / 2;

  int rows_to_draw = (height_rows < spec_history_filled_count)
                         ? height_rows
                         : spec_history_filled_count;

  for (int offset = 0; offset < rows_to_draw; offset++) {
    int idx = (spec_history_head_index - 1 - offset + SPEC_HISTORY_LEN) %
              SPEC_HISTORY_LEN;
    int y = top_row + height_rows - 1 - offset;
    if (y < 0 || y >= LINES)
      continue;

    for (int k = 0; k < width_cols; k++) {
      int x = left_col + k;
      if (x < 0 || x >= COLS)
        continue;
      unsigned char tier = spec_history_buffer[idx][k];
      if (tier == 0)
        continue;
      char glyph;
      int pair;
      switch (tier) {
      default:
        continue;
      case 1:
        glyph = '.';
        pair = PAIR_FREQ_LOW;
        break;
      case 2:
        glyph = '+';
        pair = PAIR_FREQ_LOW;
        break;
      case 3:
        glyph = 'o';
        pair = PAIR_FREQ_HIGH;
        break;
      case 4:
        glyph = '#';
        pair = PAIR_FREQ_HIGH;
        break;
      }
      attron(COLOR_PAIR(pair) | A_BOLD);
      mvaddch(y, x, glyph);
      attroff(COLOR_PAIR(pair) | A_BOLD);
    }
  }
}

/* ── §14  layout — where each panel sits this frame ──
 *
 * Top to bottom: HUD row, then the TIME panel, the FREQ panel, the
 * optional SPECTROGRAM, then a status row and the key-hint row.  We
 * recompute from the current screen size every frame, which is what
 * makes resizing just work.  With the waterfall off, TIME and FREQ split
 * the space in half; with it on, they each take a quarter and the
 * waterfall gets the rest. */

/* The computed row positions for one frame's layout. */
typedef struct {
  int time_label_row, time_top_row, time_height_rows, time_baseline_row;
  int freq_label_row, freq_top_row, freq_height_rows, freq_baseline_row;
  int water_label_row, water_top_row, water_height_rows;
  int status_row, hint_row;
} PanelLayout;

static PanelLayout compute_layout(bool show_waterfall) {
  /* fixed rows: HUD, status, hint, and one label per visible panel */
  int reserved = 5 + (show_waterfall ? 1 : 0);
  int avail = LINES - reserved;
  if (avail < 6)
    avail = 6;

  int time_h, freq_h, water_h;
  if (show_waterfall) {
    time_h = avail / 4;
    if (time_h < 3)
      time_h = 3;
    freq_h = avail / 4;
    if (freq_h < 3)
      freq_h = 3;
    water_h = avail - time_h - freq_h;
    if (water_h < 3)
      water_h = 3;
  } else {
    time_h = avail / 2;
    if (time_h < 3)
      time_h = 3;
    freq_h = avail - time_h;
    water_h = 0;
  }

  PanelLayout L;
  L.time_label_row = 1;
  L.time_top_row = L.time_label_row + 1;
  L.time_height_rows = time_h;
  L.time_baseline_row = L.time_top_row + time_h - 1;

  L.freq_label_row = L.time_baseline_row + 1;
  L.freq_top_row = L.freq_label_row + 1;
  L.freq_height_rows = freq_h;
  L.freq_baseline_row = L.freq_top_row + freq_h - 1;

  if (show_waterfall) {
    L.water_label_row = L.freq_baseline_row + 1;
    L.water_top_row = L.water_label_row + 1;
    L.water_height_rows = water_h;
  } else {
    L.water_label_row = 0;
    L.water_top_row = 0;
    L.water_height_rows = 0;
  }
  L.status_row = LINES - 2;
  L.hint_row = LINES - 1;
  return L;
}

/* ── §15  draw_time — the raw waveform ──
 *
 * Draws each sample as a bar around a centre line: positive samples go up,
 * negative go down.  Scaled to the loudest sample so it always fills the
 * panel nicely. */
static void draw_time_panel(const float *signal_pre_window, float signal_peak,
                            const PanelLayout *L) {
  int half_height = L->time_height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = L->time_top_row + half_height;

  int columns_to_draw = COLS < N_FFT ? COLS : N_FFT;
  for (int n = 0; n < columns_to_draw; n++) {
    float v = signal_pre_window[n] / (signal_peak + 1e-6f);
    bool pos = v >= 0.0f;
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    draw_bar(n, midline_row, h, pos, pos ? PAIR_TIME_POS : PAIR_TIME_NEG);
  }
}

/* ── §16  draw_freq — the spectrum bars ──
 *
 * One upward bar per frequency bin, taller = stronger.  Each bin is one
 * or two columns wide depending on screen room.  Returns how many bins
 * fit so the waterfall below can line up with the same width. */
static int draw_freq_panel(const float *magnitude, float magnitude_peak,
                           const PanelLayout *L) {
  int bin_width_cells = (COLS / (N_FFT / 2)) >= 2 ? 2 : 1;
  int max_bins = COLS / bin_width_cells;
  if (max_bins > N_FFT / 2)
    max_bins = N_FFT / 2;

  for (int k = 0; k < max_bins; k++) {
    float v = magnitude[k] / (magnitude_peak + 1e-6f);
    int h = (int)(v * (float)L->freq_height_rows + 0.5f);
    int pair_id =
        (h > L->freq_height_rows / 2) ? PAIR_FREQ_HIGH : PAIR_FREQ_LOW;
    for (int bx = 0; bx < bin_width_cells; bx++)
      draw_bar(k * bin_width_cells + bx, L->freq_baseline_row, h, true,
               pair_id);
  }
  return max_bins;
}

/* ── §17  draw_debug — two optional teaching overlays ──
 *
 * Both just read what the algorithm produced and draw on top; neither
 * changes anything.  'd' traces the active window's bell shape over the
 * time panel.  'D' lists the loudest bins with their phase, so you can
 * pause and watch how phase shifts even while the spectrum stays put. */

#define DEBUG_PHASE_TABLE_ROWS 8

/* Trace the window's fade curve across the time panel as a row of '*'. */
static void draw_debug_window_overlay(WindowKind kind, const PanelLayout *L) {
  if (kind == WINDOW_RECT)
    return; /* a flat window has nothing to show */
  int half_height = L->time_height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = L->time_top_row + half_height;

  int columns = COLS < N_FFT ? COLS : N_FFT;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int n = 0; n < columns; n++) {
    float w = window_coefficient(kind, n, N_FFT);
    int h = (int)(w * (float)half_height + 0.5f);
    int y = midline_row - h;
    if (y >= 0 && y < LINES)
      mvaddch(y, n, '*');
  }
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* List the loudest few bins in the corner with magnitude and phase. */
static void draw_debug_phase_table(const float *fft_real, const float *fft_imag,
                                   const float *magnitude) {
  int sorted_bin_indices[N_FFT / 2];
  for (int i = 0; i < N_FFT / 2; i++)
    sorted_bin_indices[i] = i;
  for (int i = 0; i < DEBUG_PHASE_TABLE_ROWS && i < N_FFT / 2; i++) {
    int max_at = i;
    for (int j = i + 1; j < N_FFT / 2; j++)
      if (magnitude[sorted_bin_indices[j]] >
          magnitude[sorted_bin_indices[max_at]])
        max_at = j;
    int tmp = sorted_bin_indices[i];
    sorted_bin_indices[i] = sorted_bin_indices[max_at];
    sorted_bin_indices[max_at] = tmp;
  }

  int x = 2, y = 2;
  if (y + DEBUG_PHASE_TABLE_ROWS + 1 >= LINES)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "  bin   |X[k]|     arg(rad)");
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int i = 0; i < DEBUG_PHASE_TABLE_ROWS && i < N_FFT / 2; i++) {
    int k = sorted_bin_indices[i];
    float phase_radians = atan2f(fft_imag[k], fft_real[k]);
    attron(COLOR_PAIR(PAIR_FREQ_HIGH) | A_BOLD);
    mvprintw(y + 1 + i, x, " %4d  %7.4f   %+6.3f", k, (double)magnitude[k],
             (double)phase_radians);
    attroff(COLOR_PAIR(PAIR_FREQ_HIGH) | A_BOLD);
  }
}

/* ── §18  draw_hud — the status line and the key hint ── */

static void draw_hud_top(int theme_index, bool paused) {
  char buf[160];
  snprintf(buf, sizeof buf, " FFT  N=%d  FFT %d ops vs DFT %d ops  thm:%s ",
           N_FFT, N_FFT * LOG2_N_FFT, N_FFT * N_FFT,
           theme_table[theme_index].display_name);
  int x = COLS - (int)strlen(buf);
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  if (paused) {
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
    mvprintw(0, 0, " PAUSED ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
  }
}

static void draw_hud_hint(int hint_row) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(hint_row, 0,
           " q:quit spc:pause m/M:mode w/W:win 1-3 j/k +/- ,/. n:noise"
           " g:waterfall t/T:theme d:wenv D:phase r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §19  app_state — everything the program is currently doing ── */

/* The full state of the running demo: what the user has selected, plus
 * the scratch buffers the per-frame pipeline reuses (no allocation in the
 * loop — these are filled fresh every frame). */
typedef struct {
  SignalParams sig_params;
  int selected_component_index;     /* which sine the +/- keys affect */
  WindowKind active_window;
  int active_theme_index;
  bool paused;
  bool show_waterfall;
  bool show_debug_window_overlay;   /* 'd' overlay on/off */
  bool show_debug_phase_table;      /* 'D' overlay on/off */

  float animation_phase_radians;    /* the cosmetic sideways drift */

  /* scratch buffers, refilled each frame */
  float signal_pre_window[N_FFT];   /* the raw waveform, before fading */
  float fft_real_buffer[N_FFT];     /* faded copy in, spectrum out     */
  float fft_imag_buffer[N_FFT];
  float magnitude[N_FFT / 2];       /* strength of each frequency bin  */
  float magnitude_peak;             /* loudest bin, for auto-scaling   */
  float signal_peak;                /* loudest sample, for auto-scaling */
} App;

static void app_set_defaults(App *a) {
  a->sig_params.mode = MODE_SINE_SUM;
  a->sig_params.sine_components[0] = (SineComponent){4.0f, 1.0f, true};
  a->sig_params.sine_components[1] = (SineComponent){11.0f, 0.6f, true};
  a->sig_params.sine_components[2] = (SineComponent){23.0f, 0.4f, true};
  a->sig_params.frequency_single_bin = 5.0f;
  a->sig_params.chirp_lo_bin = 2.0f;
  a->sig_params.chirp_hi_bin = 30.0f;
  a->sig_params.am_carrier_bin = 16.0f;
  a->sig_params.am_modulator_bin = 3.0f;
  a->sig_params.impulse_period_samples = 16;

  a->selected_component_index = 0;
  a->active_window = WINDOW_RECT;
  a->active_theme_index = 0;
  a->paused = false;
  a->show_waterfall = true;
  a->show_debug_window_overlay = false;
  a->show_debug_phase_table = false;
  a->animation_phase_radians = 0.0f;
  spec_history_clear();
}

/* Nudge whichever frequency the current mode cares about.  All four of
 * the +/-/,/. keys land here; the mode decides what actually moves. */
static void app_adjust_freq(App *a, float delta_bins) {
  float lo = 1.0f, hi = (float)N_FFT * 0.5f - 1.0f;
  switch (a->sig_params.mode) {
  case MODE_SINE_SUM: {
    SineComponent *c =
        &a->sig_params.sine_components[a->selected_component_index];
    c->frequency_bin += delta_bins;
    if (c->frequency_bin < lo)
      c->frequency_bin = lo;
    if (c->frequency_bin > hi)
      c->frequency_bin = hi;
    break;
  }
  case MODE_CHIRP:
    a->sig_params.chirp_hi_bin += delta_bins;
    if (a->sig_params.chirp_hi_bin < a->sig_params.chirp_lo_bin + 1.0f)
      a->sig_params.chirp_hi_bin = a->sig_params.chirp_lo_bin + 1.0f;
    if (a->sig_params.chirp_hi_bin > hi)
      a->sig_params.chirp_hi_bin = hi;
    break;
  case MODE_AM:
    a->sig_params.am_carrier_bin += delta_bins;
    if (a->sig_params.am_carrier_bin < a->sig_params.am_modulator_bin + 1.0f)
      a->sig_params.am_carrier_bin = a->sig_params.am_modulator_bin + 1.0f;
    if (a->sig_params.am_carrier_bin > hi)
      a->sig_params.am_carrier_bin = hi;
    break;
  case MODE_IMPULSE: {
    int step = (delta_bins >= 0.5f) ? 1 : (delta_bins <= -0.5f) ? -1 : 0;
    a->sig_params.impulse_period_samples += step;
    if (a->sig_params.impulse_period_samples < 2)
      a->sig_params.impulse_period_samples = 2;
    if (a->sig_params.impulse_period_samples > N_FFT / 2)
      a->sig_params.impulse_period_samples = N_FFT / 2;
    break;
  }
  default:
    a->sig_params.frequency_single_bin += delta_bins;
    if (a->sig_params.frequency_single_bin < lo)
      a->sig_params.frequency_single_bin = lo;
    if (a->sig_params.frequency_single_bin > hi)
      a->sig_params.frequency_single_bin = hi;
    break;
  }
}

/* Build the little parameter readout shown beside the TIME panel label. */
static void mode_param_summary(const App *a, char *buf, size_t buflen) {
  const SignalParams *p = &a->sig_params;
  switch (p->mode) {
  case MODE_SINE_SUM:
    snprintf(
        buf, buflen, "[c%d f=%.1f]", a->selected_component_index + 1,
        (double)p->sine_components[a->selected_component_index].frequency_bin);
    break;
  case MODE_CHIRP:
    snprintf(buf, buflen, "lo=%.1f hi=%.1f", (double)p->chirp_lo_bin,
             (double)p->chirp_hi_bin);
    break;
  case MODE_AM:
    snprintf(buf, buflen, "carrier=%.1f mod=%.1f", (double)p->am_carrier_bin,
             (double)p->am_modulator_bin);
    break;
  case MODE_IMPULSE:
    snprintf(buf, buflen, "period=%d", p->impulse_period_samples);
    break;
  default:
    snprintf(buf, buflen, "f=%.1f", (double)p->frequency_single_bin);
    break;
  }
}

/* ── §20  app_compute — the once-per-frame number crunching ──
 *
 * Make the waveform, run it through the window and the FFT, turn the
 * result into per-bin strengths, file it into the history, and note the
 * loudest sample and bin so the panels can auto-scale. */
static void app_compute(App *a) {
  build_signal(&a->sig_params, a->signal_pre_window,
               a->animation_phase_radians);

  for (int n = 0; n < N_FFT; n++) {
    a->fft_real_buffer[n] = a->signal_pre_window[n];
    a->fft_imag_buffer[n] = 0.0f;
  }
  apply_window(a->fft_real_buffer, N_FFT, a->active_window);

  fft(a->fft_real_buffer, a->fft_imag_buffer, N_FFT);

  a->magnitude_peak = 1e-6f;
  for (int k = 0; k < N_FFT / 2; k++) {
    a->magnitude[k] = sqrtf(a->fft_real_buffer[k] * a->fft_real_buffer[k] +
                            a->fft_imag_buffer[k] * a->fft_imag_buffer[k]) /
                      ((float)N_FFT * 0.5f);
    if (a->magnitude[k] > a->magnitude_peak)
      a->magnitude_peak = a->magnitude[k];
  }

  a->signal_peak = 1e-6f;
  for (int n = 0; n < N_FFT; n++) {
    float av = fabsf(a->signal_pre_window[n]);
    if (av > a->signal_peak)
      a->signal_peak = av;
  }

  if (!a->paused && a->show_waterfall)
    spec_history_push(a->magnitude, a->magnitude_peak);
}

/* Draw the whole frame.  Order matters: later layers cover earlier ones
 * where they overlap, so the HUD and hint go on last. */
static void app_draw(const App *a) {
  erase();
  PanelLayout L = compute_layout(a->show_waterfall);

  /* panel labels */
  char param_buf[64];
  mode_param_summary(a, param_buf, sizeof param_buf);
  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(L.time_label_row, 0, "TIME x[n]   mode:%s   win:%s   %s",
           mode_name[a->sig_params.mode], window_name[a->active_window],
           param_buf);
  mvprintw(L.freq_label_row, 0, "FREQ |X[k]|   bins 0..%d   peak=%.3f",
           N_FFT / 2 - 1, (double)a->magnitude_peak);
  if (a->show_waterfall) {
    mvprintw(L.water_label_row, 0, "SPECTROGRAM  newest=bottom  history=%d",
             spec_history_filled_count);
  }
  attroff(COLOR_PAIR(PAIR_HINT));

  /* the three panels */
  draw_time_panel(a->signal_pre_window, a->signal_peak, &L);
  int max_bins_drawn = draw_freq_panel(a->magnitude, a->magnitude_peak, &L);
  if (a->show_waterfall && L.water_height_rows > 0)
    draw_waterfall(L.water_top_row, 0, L.water_height_rows, max_bins_drawn);

  /* optional teaching overlays */
  if (a->show_debug_window_overlay)
    draw_debug_window_overlay(a->active_window, &L);
  if (a->show_debug_phase_table)
    draw_debug_phase_table(a->fft_real_buffer, a->fft_imag_buffer,
                           a->magnitude);

  /* status row */
  attron(COLOR_PAIR(PAIR_HUD));
  if (a->sig_params.mode == MODE_SINE_SUM) {
    mvprintw(L.status_row, 0, "Components: ");
    for (int c = 0; c < N_SINE_COMPONENTS; c++) {
      if (c == a->selected_component_index)
        attron(A_REVERSE);
      if (a->sig_params.sine_components[c].enabled)
        attron(A_BOLD);
      printw("[%d] f=%.1f a=%.1f%s  ", c + 1,
             (double)a->sig_params.sine_components[c].frequency_bin,
             (double)a->sig_params.sine_components[c].amplitude,
             a->sig_params.sine_components[c].enabled ? "" : " OFF");
      attroff(A_BOLD);
      attroff(A_REVERSE);
    }
  } else {
    mvprintw(L.status_row, 0, "Mode: %s   %s", mode_name[a->sig_params.mode],
             param_buf);
  }
  attroff(COLOR_PAIR(PAIR_HUD));

  /* HUD and key hint, painted last so nothing covers them */
  draw_hud_top(a->active_theme_index, a->paused);
  draw_hud_hint(L.hint_row);

  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §21  app_input — turn keypresses into state changes ── */

/* Handle one keypress.  Returns true only when it's time to quit. */
static bool app_handle_key(App *a, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return true;

  case ' ':
    a->paused = !a->paused;
    break;

  /* sine-sum controls — these keys do nothing in the other modes */
  case '1':
    if (a->sig_params.mode == MODE_SINE_SUM)
      a->sig_params.sine_components[0].enabled =
          !a->sig_params.sine_components[0].enabled;
    break;
  case '2':
    if (a->sig_params.mode == MODE_SINE_SUM)
      a->sig_params.sine_components[1].enabled =
          !a->sig_params.sine_components[1].enabled;
    break;
  case '3':
    if (a->sig_params.mode == MODE_SINE_SUM)
      a->sig_params.sine_components[2].enabled =
          !a->sig_params.sine_components[2].enabled;
    break;
  case 'j':
    if (a->sig_params.mode == MODE_SINE_SUM)
      a->selected_component_index =
          (a->selected_component_index + 1) % N_SINE_COMPONENTS;
    break;
  case 'k':
    if (a->sig_params.mode == MODE_SINE_SUM)
      a->selected_component_index =
          (a->selected_component_index + N_SINE_COMPONENTS - 1) %
          N_SINE_COMPONENTS;
    break;

  /* frequency tweaks: +/- coarse, ,/. fine */
  case '+':
  case '=':
    app_adjust_freq(a, +1.0f);
    break;
  case '-':
    app_adjust_freq(a, -1.0f);
    break;
  case '.':
    app_adjust_freq(a, +0.1f);
    break;
  case ',':
    app_adjust_freq(a, -0.1f);
    break;

  /* cycle the signal shape and the window */
  case 'm':
    a->sig_params.mode =
        (SignalMode)((a->sig_params.mode + 1) % N_SIGNAL_MODES);
    spec_history_clear();
    break;
  case 'M':
    a->sig_params.mode =
        (SignalMode)((a->sig_params.mode + N_SIGNAL_MODES - 1) %
                     N_SIGNAL_MODES);
    spec_history_clear();
    break;
  case 'w':
    a->active_window = (WindowKind)((a->active_window + 1) % N_WINDOWS);
    break;
  case 'W':
    a->active_window =
        (WindowKind)((a->active_window + N_WINDOWS - 1) % N_WINDOWS);
    break;

  /* overlays, noise, theme, reset */
  case 'g':
  case 'G':
    a->show_waterfall = !a->show_waterfall;
    spec_history_clear();
    break;
  case 'd':
    a->show_debug_window_overlay = !a->show_debug_window_overlay;
    break;
  case 'D':
    a->show_debug_phase_table = !a->show_debug_phase_table;
    break;
  case 'n':
  case 'N':
    for (int i = 0; i < N_FFT; i++)
      g_noise_buffer[i] =
          ((float)rand() / (float)RAND_MAX * 2.0f - 1.0f) * NOISE_AMP_PEAK;
    g_noise_decay_envelope = 1.0f;
    break;
  case 't':
    a->active_theme_index = (a->active_theme_index + 1) % N_THEMES;
    apply_theme(a->active_theme_index);
    break;
  case 'T':
    a->active_theme_index = (a->active_theme_index + N_THEMES - 1) % N_THEMES;
    apply_theme(a->active_theme_index);
    break;
  case 'r':
  case 'R':
    app_set_defaults(a);
    apply_theme(a->active_theme_index);
    break;

  default:
    break;
  }
  return false;
}

/* ── §22  app_main — signal handlers and the main loop ── */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

/* Signal handlers can't safely do much, so just raise a flag and let the
 * main loop act on it. */
static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static void cleanup_screen(void) { endwin(); }

int main(void) {
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);
  atexit(cleanup_screen);

  initscr();
  cbreak();
  noecho();
  curs_set(0);
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  typeahead(-1); /* stop ncurses peeking at input mid-draw and tearing it */

  App a;
  app_set_defaults(&a);
  colors_init(a.active_theme_index);

  long long next_frame_ns = clock_now_ns();

  while (!g_should_quit) {
    /* the terminal changed size: rebuild it and drop the now-mismatched
     * history (column count may have changed) */
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
      spec_history_clear();
    }

    /* drain every pending keypress */
    int ch;
    while ((ch = getch()) != ERR) {
      if (app_handle_key(&a, ch)) {
        g_should_quit = 1;
        break;
      }
    }

    /* once it's time, advance and redraw; otherwise sleep till then */
    long long now = clock_now_ns();
    if (now >= next_frame_ns) {
      if (!a.paused) {
        a.animation_phase_radians += ANIMATION_PHASE_STEP_RAD;
        if (a.animation_phase_radians > 2.0f * (float)M_PI)
          a.animation_phase_radians -= 2.0f * (float)M_PI;
        if (g_noise_decay_envelope > 0.0f)
          g_noise_decay_envelope -= NOISE_DECAY_RATE;
        if (g_noise_decay_envelope < 0.0f)
          g_noise_decay_envelope = 0.0f;
      }
      app_compute(&a);
      app_draw(&a);
      next_frame_ns += RENDER_TICK_NS;
      /* if we fell way behind (terminal was hidden or suspended), jump the
       * clock forward instead of racing through a backlog of frames */
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
