/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * convolution_helloworld.c — watch 1-D convolution happen one step at
 * a time: a little kernel slides across a signal, and at each stop you
 * see the multiply-and-add that produces one output sample.  Three
 * stacked panels: the input, the kernel, and the output it builds up.
 *
 * Sister demos: signal/fir_filter.c (same operation, real filter
 * design), signal/idft_helloworld.c and signal/dft_helloworld.c
 * (the frequency-domain side of the same coin).
 * Gentlest reference: Smith, "The Scientist and Engineer's Guide to
 * Digital Signal Processing", ch. 6-7 (free online).
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

#define N 64          /* input signal length            */
#define KERNEL_MAX 15 /* longest kernel we allow        */
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define AUTO_SLIDE_FRAMES 3 /* frames the kernel waits before sliding */

/* Colours for each thing on screen.  HUD/HINT colours are fixed (not
 * theme-able) so the status text stays readable over any animation. */
enum {
  PAIR_INPUT = 1,     /* input wave bars                              */
  PAIR_OUTPUT = 2,    /* output wave bars                             */
  PAIR_KERNEL = 3,    /* kernel weight bars                           */
  PAIR_KERNEL_HI = 4, /* highlight on input samples under the kernel  */
  PAIR_LABEL = 5,     /* panel labels                                 */
  PAIR_HUD = 6,       /* HUD top status                               */
  PAIR_HINT = 7,      /* bottom hint                                  */
};

/* §2  clock */

static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static void clock_sleep_ns(long long ns) {
  /* Sleep instead of busy-looping, so we don't peg a CPU core. */
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* §3  colors */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_INPUT, 51, -1);      /* bright cyan      */
    init_pair(PAIR_OUTPUT, 154, -1);    /* yellow-green     */
    init_pair(PAIR_KERNEL, 213, -1);    /* magenta-pink     */
    init_pair(PAIR_KERNEL_HI, 226, -1); /* bright yellow    */
    init_pair(PAIR_LABEL, 244, -1);     /* mid grey         */
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow    */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan      */
  } else {
    init_pair(PAIR_INPUT, COLOR_CYAN, -1);
    init_pair(PAIR_OUTPUT, COLOR_GREEN, -1);
    init_pair(PAIR_KERNEL, COLOR_MAGENTA, -1);
    init_pair(PAIR_KERNEL_HI, COLOR_YELLOW, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* §4  signal_kinds */

/* The five test signals you can feed in (cycle with 's').  Each one
 * shows off a different filter behaviour; SIG_COUNT just marks how
 * many there are. */
typedef enum {
  SIG_IMPULSE = 0, /* one tall spike at the centre, flat elsewhere */
  SIG_STEP,        /* low for the first half, high for the second  */
  SIG_SINES,       /* two sine waves added together                */
  SIG_SQUARE,      /* on/off square wave                           */
  SIG_NOISE,       /* same random-looking jitter every frame       */
  SIG_COUNT
} SignalKind;

static const char *signal_name[SIG_COUNT] = {
    "Impulse  ", "Step     ", "Sum sines", "Square   ", "Noise    "};

/* §5  signal_generators */

/* Fill the caller's array with one of the test signals.  Every signal
 * stays within ±1 so all of them fit the same panel height. */
static void generate_signal(SignalKind kind, float *output_signal) {
  switch (kind) {
  case SIG_IMPULSE: {
    /* A single spike.  Worth knowing: filtering a spike spits the
     * kernel itself back out (the "impulse response"). */
    for (int n = 0; n < N; n++)
      output_signal[n] = 0.0f;
    output_signal[N / 2] = 1.0f;
    break;
  }
  case SIG_STEP: {
    for (int n = 0; n < N; n++)
      output_signal[n] = (n < N / 2) ? -0.5f : 0.5f;
    break;
  }
  case SIG_SINES: {
    /* A slow wave plus a fast wave.  Smoothing kernels keep the slow
     * one and erase the fast one — a nice thing to watch. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      output_signal[n] = 0.6f * sinf(2.0f * (float)M_PI * 3.0f * t) +
                         0.4f * sinf(2.0f * (float)M_PI * 11.0f * t);
    }
    break;
  }
  case SIG_SQUARE: {
    /* A square wave.  Its sharp jumps make smoothing kernels ring and
     * make the Edge kernel light up. */
    for (int n = 0; n < N; n++) {
      float t = (float)n / (float)N;
      float arg = 2.0f * (float)M_PI * 5.0f * t;
      output_signal[n] = sinf(arg) >= 0.0f ? 0.7f : -0.7f;
    }
    break;
  }
  case SIG_NOISE: {
    /* Random-looking, but we reseed to 1 every call so it's the SAME
     * noise each frame — otherwise the output would jitter. */
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

/* §6  kernel_kinds */

/* The six kernels you can slide across the signal (cycle with 'k').
 * The numbers are the actual weights; the words say what each does. */
typedef enum {
  KERN_IDENTITY = 0, /* {0, 0, 1, 0, 0}     leaves the signal alone  */
  KERN_BOX,          /* {1, 1, 1, 1, 1}/5   plain average, smooths   */
  KERN_GAUSSIAN,     /* {1, 4, 6, 4, 1}/16  gentler smooth, less ring */
  KERN_EDGE,         /* {-1, 0, 1}          reacts to changes        */
  KERN_SHARPEN,      /* {-1,-1, 5,-1,-1}    exaggerates edges         */
  KERN_INVERSE,      /* {0, 0,-1, 0, 0}     flips the signal upside-down */
  KERN_COUNT
} KernelKind;

static const char *kernel_name[KERN_COUNT] = {"Identity ", "Box      ",
                                              "Gaussian ", "Edge     ",
                                              "Sharpen  ", "Inverse  "};

/* §7  kernel_generators */

/* Write the chosen kernel's weights into the caller's array, and hand
 * back its length through out_length. */
static void generate_kernel(KernelKind kind, float *kernel_weights,
                            int *out_length) {
  switch (kind) {
  case KERN_IDENTITY: {
    /* One 1 in the middle: the output comes out the same as the input. */
    *out_length = 5;
    kernel_weights[0] = 0.0f;
    kernel_weights[1] = 0.0f;
    kernel_weights[2] = 1.0f;
    kernel_weights[3] = 0.0f;
    kernel_weights[4] = 0.0f;
    break;
  }
  case KERN_BOX: {
    /* All equal, adding up to 1: a plain average of the neighbours. */
    *out_length = 5;
    for (int i = 0; i < 5; i++)
      kernel_weights[i] = 0.2f;
    break;
  }
  case KERN_GAUSSIAN: {
    /* Bell-shaped weights: heaviest in the middle.  Smooths like Box
     * but more gently, with less ringing at sharp jumps. */
    *out_length = 5;
    const float w[] = {1.0f, 4.0f, 6.0f, 4.0f, 1.0f};
    for (int i = 0; i < 5; i++)
      kernel_weights[i] = w[i] / 16.0f;
    break;
  }
  case KERN_EDGE: {
    /* Right minus left: big where the signal is changing fast, near
     * zero where it's flat. */
    *out_length = 3;
    kernel_weights[0] = -1.0f;
    kernel_weights[1] = 0.0f;
    kernel_weights[2] = +1.0f;
    break;
  }
  case KERN_SHARPEN: {
    /* Boost the centre, subtract the neighbours: leaves smooth parts
     * alone but makes edges stand out more. */
    *out_length = 5;
    kernel_weights[0] = -1.0f;
    kernel_weights[1] = -1.0f;
    kernel_weights[2] = +5.0f;
    kernel_weights[3] = -1.0f;
    kernel_weights[4] = -1.0f;
    break;
  }
  case KERN_INVERSE: {
    /* Identity, flipped negative: the output is the input upside-down. */
    *out_length = 5;
    kernel_weights[0] = 0.0f;
    kernel_weights[1] = 0.0f;
    kernel_weights[2] = -1.0f;
    kernel_weights[3] = 0.0f;
    kernel_weights[4] = 0.0f;
    break;
  }
  default:
    *out_length = 1;
    kernel_weights[0] = 1.0f;
    break;
  }
}

/* §8  convolve — the whole point of the file */

/* The one operation everything else exists to show off.  For each spot
 * the kernel can sit, line up the kernel weights with the input samples
 * underneath, multiply each pair, and add them up — that sum is one
 * output sample.  Slide one spot right and repeat.
 *
 * The caller owns every array; we only write into output_signal and
 * never touch the input. */
static void convolve(const float *input_signal, const float *kernel_weights,
                     int kernel_length, float *output_signal) {
  /* Past here the kernel would hang off the right end of the input. */
  int last_valid_output_position = N - kernel_length;

  for (int output_position = 0; output_position <= last_valid_output_position;
       output_position++) {

    float weighted_sum = 0.0f;

    for (int kernel_tap_index = 0; kernel_tap_index < kernel_length;
         kernel_tap_index++) {

      float kernel_weight = kernel_weights[kernel_tap_index];
      float input_sample = input_signal[output_position + kernel_tap_index];
      weighted_sum += kernel_weight * input_sample;
    }

    output_signal[output_position] = weighted_sum;
  }

  /* The kernel can't reach these last spots, so just leave them at zero. */
  for (int output_position = last_valid_output_position + 1;
       output_position < N; output_position++) {
    output_signal[output_position] = 0.0f;
  }
}

/* §9  scene_state */

static float g_input_signal[N];
static float g_output_signal[N];
static float g_kernel_weights[KERNEL_MAX];
static int g_kernel_length;
static int g_kernel_position = 0; /* current slide position    */

static SignalKind g_signal_kind = SIG_SINES;
static KernelKind g_kernel_kind = KERN_GAUSSIAN;

static bool g_simulation_paused = false;
static bool g_auto_slide_enabled = true;
static int g_auto_slide_counter = 0;

/* Debug overlay toggles (preserved across kernel/signal cycling). */
static bool g_show_live_arithmetic = false; /* 'd' overlay */
static bool g_show_kernel_table = false;    /* 'D' overlay */

static void scene_reset(void) {
  g_kernel_position = 0;
  g_auto_slide_counter = 0;
  g_simulation_paused = false;
  g_auto_slide_enabled = true;
  generate_signal(g_signal_kind, g_input_signal);
  generate_kernel(g_kernel_kind, g_kernel_weights, &g_kernel_length);
}

/* §10  scene_tick — advance one frame */

static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* In auto mode, nudge the kernel one spot right every few frames so
   * the slide is slow enough to follow; wrap back to the start at the end. */
  if (g_auto_slide_enabled) {
    g_auto_slide_counter++;
    if (g_auto_slide_counter >= AUTO_SLIDE_FRAMES) {
      g_auto_slide_counter = 0;
      g_kernel_position++;
      if (g_kernel_position > N - g_kernel_length)
        g_kernel_position = 0;
    }
  }

  /* Recompute the whole output (cheap at this size). */
  convolve(g_input_signal, g_kernel_weights, g_kernel_length, g_output_signal);
}

/* §11  scene_input — react to keys */

static void scene_step_kernel(int delta) {
  /* Hand-stepping only makes sense when auto-slide is off. */
  if (g_auto_slide_enabled)
    return;
  g_kernel_position += delta;
  if (g_kernel_position < 0)
    g_kernel_position = 0;
  if (g_kernel_position > N - g_kernel_length)
    g_kernel_position = N - g_kernel_length;
}

static void scene_cycle_kernel(int direction) {
  g_kernel_kind =
      (KernelKind)((g_kernel_kind + direction + KERN_COUNT) % KERN_COUNT);
  generate_kernel(g_kernel_kind, g_kernel_weights, &g_kernel_length);
  if (g_kernel_position > N - g_kernel_length)
    g_kernel_position = N - g_kernel_length;
}

static void scene_cycle_signal(int direction) {
  g_signal_kind =
      (SignalKind)((g_signal_kind + direction + SIG_COUNT) % SIG_COUNT);
  generate_signal(g_signal_kind, g_input_signal);
}

/* §12  draw_bar */

/* Draw one vertical bar growing up or down from a baseline row.  Every
 * panel is built out of these. */
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

/* §13  draw_signal_panel — draws the input or output wave */

/* One bar per sample, rising or falling from the panel's middle line.
 * Samples in [highlight_left, highlight_right) get a different colour —
 * the input panel uses this to flag the samples the kernel is on right now. */
static void draw_signal_panel(const float *signal, int top_row, int height_rows,
                              int colour_pair, int highlight_left,
                              int highlight_right) {
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int spacing = (COLS / N >= 2) ? 2 : 1;
  int columns_to_draw = (COLS / spacing < N) ? COLS / spacing : N;

  for (int n = 0; n < columns_to_draw; n++) {
    float v = signal[n];
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    int pair = (n >= highlight_left && n < highlight_right) ? PAIR_KERNEL_HI
                                                            : colour_pair;
    for (int s = 0; s < spacing; s++)
      draw_bar(n * spacing + s, midline_row, h, pos, pair);
  }
}

/* §14  draw_kernel_panel — kernel sitting over the samples it touches */

/* Draw the kernel's bars right above the input samples it's multiplying
 * at this moment.  Bar height is the weight (up for positive, down for
 * negative), scaled by the biggest weight so any kernel fits the panel. */
static void draw_kernel_panel(int top_row, int height_rows) {
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  int spacing = (COLS / N >= 2) ? 2 : 1;

  /* Biggest weight, used to scale the bars so the tallest one fills the panel. */
  float peak = 0.0f;
  for (int i = 0; i < g_kernel_length; i++) {
    float a = fabsf(g_kernel_weights[i]);
    if (a > peak)
      peak = a;
  }
  if (peak < 1e-6f)
    peak = 1.0f;

  for (int i = 0; i < g_kernel_length; i++) {
    int n = g_kernel_position + i;
    if (n >= N)
      continue;
    float v = g_kernel_weights[i] / peak;
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);
    for (int s = 0; s < spacing; s++)
      draw_bar(n * spacing + s, midline_row, h, pos, PAIR_KERNEL);
  }

  /* A dashed line marking where weight zero is. */
  attron(COLOR_PAIR(PAIR_LABEL));
  int mid_w = N * spacing;
  for (int x = 0; x < mid_w && x < COLS; x++)
    mvaddch(midline_row, x, '-');
  attroff(COLOR_PAIR(PAIR_LABEL));
}

/* §15  draw_debug — optional 'd' and 'D' overlays */

/* Spell out the multiply-and-add for the sample the kernel is on right
 * now, one line per term plus the total.  Pause to read it at leisure. */
static void draw_live_arithmetic_overlay(void) {
  if (!g_show_live_arithmetic)
    return;
  int x = 2, y = 2;
  if (y + g_kernel_length + 2 >= LINES - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Live arithmetic for y[%d]:", g_kernel_position);
  attroff(COLOR_PAIR(PAIR_HINT));

  float running_sum = 0.0f;
  for (int i = 0; i < g_kernel_length; i++) {
    float w = g_kernel_weights[i];
    float s = g_input_signal[g_kernel_position + i];
    float p = w * s;
    running_sum += p;
    attron(COLOR_PAIR(PAIR_KERNEL_HI) | A_BOLD);
    mvprintw(y + 1 + i, x, "  k[%d]*x[%2d] = %+.3f * %+.3f = %+.4f", i,
             g_kernel_position + i, (double)w, (double)s, (double)p);
    attroff(COLOR_PAIR(PAIR_KERNEL_HI) | A_BOLD);
  }

  attron(COLOR_PAIR(PAIR_OUTPUT) | A_BOLD);
  mvprintw(y + 1 + g_kernel_length, x, "  ── sum = %+.4f  =  y[%d]",
           (double)running_sum, g_kernel_position);
  attroff(COLOR_PAIR(PAIR_OUTPUT) | A_BOLD);
}

/* List the kernel's name and each weight as plain numbers. */
static void draw_kernel_table_overlay(void) {
  if (!g_show_kernel_table)
    return;
  int x = 2, y = 2;
  /* Drop below the live-arithmetic box if it's showing, so they don't overlap. */
  if (g_show_live_arithmetic)
    y += g_kernel_length + 3;
  if (y + g_kernel_length + 2 >= LINES - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Kernel table:  %s  (K = %d)", kernel_name[g_kernel_kind],
           g_kernel_length);
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int i = 0; i < g_kernel_length; i++) {
    attron(COLOR_PAIR(PAIR_KERNEL) | A_BOLD);
    mvprintw(y + 1 + i, x, "  k[%2d] = %+.4f", i, (double)g_kernel_weights[i]);
    attroff(COLOR_PAIR(PAIR_KERNEL) | A_BOLD);
  }
}

/* §16  hud — status line, key hint, and the frame layout */

static void draw_hud(void) {
  char status[200];
  snprintf(
      status, sizeof status,
      " Convolution  N=%d K=%d  signal:%s  kernel:%s  pos:%2d/%-2d  %s  %s ", N,
      g_kernel_length, signal_name[g_signal_kind], kernel_name[g_kernel_kind],
      g_kernel_position, N - g_kernel_length,
      g_auto_slide_enabled ? "AUTO  " : "MANUAL",
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
           " q:quit  spc:pause  a:auto/manual  </>:slide  k:kernel "
           " s:signal  d:arith  D:table  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Reserve 5 rows for the HUD, labels and hint; split the rest into
   * three equal panels (input, kernel, output). */
  int rows_for_panels = LINES - 5;
  if (rows_for_panels < 9)
    rows_for_panels = 9;
  int input_h = rows_for_panels / 3;
  int kernel_h = rows_for_panels / 3;
  int output_h = rows_for_panels - input_h - kernel_h;

  int input_label_row = 1;
  int input_top_row = 2;
  int kernel_label_row = 2 + input_h;
  int kernel_top_row = 3 + input_h;
  int output_label_row = 3 + input_h + kernel_h;
  int output_top_row = 4 + input_h + kernel_h;

  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(input_label_row, 0,
           "Input x[n]   (yellow = under the kernel right now)");
  mvprintw(kernel_label_row, 0,
           "Kernel k[i]  (positive grows up, negative grows down)");
  mvprintw(output_label_row, 0, "Output y[n]  =  sum_i  k[i] * x[n + i]");
  attroff(COLOR_PAIR(PAIR_LABEL));

  draw_signal_panel(g_input_signal, input_top_row, input_h, PAIR_INPUT,
                    g_kernel_position, g_kernel_position + g_kernel_length);
  draw_kernel_panel(kernel_top_row, kernel_h);
  draw_signal_panel(g_output_signal, output_top_row, output_h, PAIR_OUTPUT, -1,
                    -1); /* output panel: nothing highlighted */

  /* Drawn after the panels so the overlays sit on top of them. */
  draw_live_arithmetic_overlay();
  draw_kernel_table_overlay();

  /* HUD and hint go last so nothing paints over them. */
  draw_hud();
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* §17  app — signals, main loop, keys */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  /* Only allowed to flip a flag in a signal handler; the loop does the rest. */
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static void cleanup_screen(void) { endwin(); }

/* Returns true if the key means "quit". */
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
    g_auto_slide_enabled = !g_auto_slide_enabled;
    break;
  case '<':
  case ',':
    scene_step_kernel(-1);
    break;
  case '>':
  case '.':
    scene_step_kernel(+1);
    break;
  case 'k':
    scene_cycle_kernel(+1);
    break;
  case 'K':
    scene_cycle_kernel(-1);
    break;
  case 's':
    scene_cycle_signal(+1);
    break;
  case 'S':
    scene_cycle_signal(-1);
    break;
  case 'd':
    g_show_live_arithmetic = !g_show_live_arithmetic;
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
      /* Snap forward if we fell badly behind (e.g. terminal hidden). */
      if (clock_now_ns() > next_frame_ns + 5 * RENDER_TICK_NS)
        next_frame_ns = clock_now_ns() + RENDER_TICK_NS;
    } else {
      clock_sleep_ns(next_frame_ns - now);
    }
  }

  return 0;
}
