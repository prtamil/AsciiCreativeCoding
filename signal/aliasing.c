/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aliasing.c — shows why sampling can't tell a fast wave from a slow one.
 * A smooth "true" wave is drawn on top; below it, the same wave seen only
 * at N sample points. Sweep the frequency up and the sampled version folds
 * back to a lower one once it passes half the sample rate — Nyquist's limit.
 *
 * Nyquist (1928) and Shannon (1949) sampling theorem.
 * Sister files: signal/dft_helloworld.c, signal/fft_vis.c, signal/fir_filter.c
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

#define N 32              /* how many samples we take across the buffer */
#define HIGH_RES_FACTOR 8 /* extra points per sample, to draw a smooth curve */
#define HIGH_RES_N (N * HIGH_RES_FACTOR)
#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define SWEEP_PERIOD_FRAMES 240 /* one full frequency sweep, about 8 seconds */
#define FREQ_LO 0.5f
#define FREQ_HI ((float)N * 2.0f) /* sweep all the way to twice the sample rate */

enum {
  PAIR_CONTINUOUS = 1, /* the smooth true wave on top */
  PAIR_SAMPLE = 2,     /* the dots at each sample point */
  PAIR_RECON = 3,      /* lines joining the dots */
  PAIR_GHOST = 4,      /* faint copy of the true wave behind the samples */
  PAIR_LABEL = 5,      /* panel captions */
  PAIR_HUD = 6,        /* top status line */
  PAIR_HINT = 7,       /* bottom key hints */
  PAIR_NYQUIST = 8,    /* red "ALIASED" warning */
};

/* ── §2  clock ── */

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

/* ── §3  colors ── */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_CONTINUOUS, 51, -1); /* bright cyan */
    init_pair(PAIR_SAMPLE, 226, -1);    /* bright yellow */
    init_pair(PAIR_RECON, 154, -1);     /* yellow-green */
    init_pair(PAIR_GHOST, 244, -1);     /* mid grey */
    init_pair(PAIR_LABEL, 244, -1);     /* mid grey */
    init_pair(PAIR_HUD, 226, -1);       /* bright yellow */
    init_pair(PAIR_HINT, 51, -1);       /* bright cyan */
    init_pair(PAIR_NYQUIST, 196, -1);   /* bright red */
  } else {
    init_pair(PAIR_CONTINUOUS, COLOR_CYAN, -1);
    init_pair(PAIR_SAMPLE, COLOR_YELLOW, -1);
    init_pair(PAIR_RECON, COLOR_GREEN, -1);
    init_pair(PAIR_GHOST, COLOR_WHITE, -1);
    init_pair(PAIR_LABEL, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_NYQUIST, COLOR_RED, -1);
  }
}

/* ── §4  aliased_freq ── */

/* The whole point of the file in two lines: given a real frequency and a
 * sample rate, what frequency does the sampled wave SEEM to have? Anything
 * above half the sample rate folds back down into [0, fs/2] and pretends to
 * be slower. (Nyquist 1928, Shannon 1949.) The HUD shows whatever this hands
 * back. fabsf first so a negative frequency folds the same as its positive. */
static float aliased_frequency(float true_freq, float sample_rate) {
  /* drop whole multiples of the sample rate — those look identical */
  float f_mod = fmodf(fabsf(true_freq), sample_rate);

  /* anything in the top half mirrors back down to a lower frequency */
  if (f_mod > sample_rate * 0.5f)
    f_mod = sample_rate - f_mod;

  return f_mod;
}

/* ── §5  signal_continuous (the smooth true wave) ── */

static float g_continuous[HIGH_RES_N];

static void generate_continuous_signal(float frequency_cycles_per_buffer) {
  /* The "real" wave, sampled densely so it draws smoothly — our ground truth. */
  for (int i = 0; i < HIGH_RES_N; i++) {
    float t = (float)i / (float)HIGH_RES_N;
    g_continuous[i] =
        cosf(2.0f * (float)M_PI * frequency_cycles_per_buffer * t);
  }
}

/* ── §6  signal_sampled (what the sampler actually sees) ── */

static float g_sampled[N];

static void generate_sampled_signal(float frequency_cycles_per_buffer) {
  /* Same wave, but read at only N points — all the digital side ever gets.
   * Many different real waves could produce these same N values, and that
   * ambiguity is exactly what makes aliasing possible. */
  for (int n = 0; n < N; n++) {
    float t = (float)n / (float)N;
    g_sampled[n] = cosf(2.0f * (float)M_PI * frequency_cycles_per_buffer * t);
  }
}

/* ── §7  scene_state ── */

static float g_true_frequency_cycles_per_buffer = FREQ_LO;
static float g_aliased_frequency_cycles_per_buffer = 0.0f;
static bool g_simulation_paused = false;
static bool g_auto_sweep_enabled = true;
static float g_animation_phase_radians = 0.0f;

static bool g_show_foldback_overlay = false; /* 'd' overlay on/off */
static bool g_show_frequency_table = false;  /* 'D' overlay on/off */

static void scene_reset(void) {
  g_true_frequency_cycles_per_buffer = FREQ_LO;
  g_simulation_paused = false;
  g_auto_sweep_enabled = true;
  g_animation_phase_radians = 0.0f;
}

/* ── §8  scene_tick ── */

static void scene_tick(void) {
  if (g_simulation_paused)
    return;

  /* Drift the frequency smoothly up and back down. The sweep range goes well
   * past Nyquist and past the sample rate, so you see the wave fold more than
   * once. (sin gives the gentle there-and-back motion.) */
  if (g_auto_sweep_enabled) {
    g_animation_phase_radians +=
        2.0f * (float)M_PI / (float)SWEEP_PERIOD_FRAMES;
    if (g_animation_phase_radians > 2.0f * (float)M_PI)
      g_animation_phase_radians -= 2.0f * (float)M_PI;
    float s = (sinf(g_animation_phase_radians) + 1.0f) * 0.5f;
    g_true_frequency_cycles_per_buffer = FREQ_LO + s * (FREQ_HI - FREQ_LO);
  }

  generate_continuous_signal(g_true_frequency_cycles_per_buffer);
  generate_sampled_signal(g_true_frequency_cycles_per_buffer);

  g_aliased_frequency_cycles_per_buffer =
      aliased_frequency(g_true_frequency_cycles_per_buffer, (float)N);
}

/* ── §9  scene_input ── */

static void scene_adjust_freq(float delta) {
  /* Manual nudge only — ignored while the auto sweep is running. */
  if (g_auto_sweep_enabled)
    return;
  g_true_frequency_cycles_per_buffer += delta;
  if (g_true_frequency_cycles_per_buffer < FREQ_LO)
    g_true_frequency_cycles_per_buffer = FREQ_LO;
  if (g_true_frequency_cycles_per_buffer > FREQ_HI)
    g_true_frequency_cycles_per_buffer = FREQ_HI;
}

/* ── §10  draw_continuous ── */

static void draw_continuous_panel(int top_row, int height_rows) {
  /* One vertical bar per column: taller means a bigger wave value. Positive
   * grows up from the centre line, negative grows down. */
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  for (int c = 0; c < COLS; c++) {
    /* pick the wave point that lands closest to this column */
    int idx = (int)((float)c / (float)COLS * (float)HIGH_RES_N);
    if (idx < 0)
      idx = 0;
    if (idx >= HIGH_RES_N)
      idx = HIGH_RES_N - 1;
    float v = g_continuous[idx];
    bool pos = (v >= 0.0f);
    int h = (int)(fabsf(v) * (float)half_height + 0.5f);

    attron(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
    for (int dy = 0; dy < h; dy++) {
      int row = pos ? (midline_row - dy) : (midline_row + dy + 1);
      if (row < 0 || row >= LINES)
        continue;
      mvaddch(row, c, pos ? '|' : '.');
    }
    attroff(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
  }
}

/* ── §11  draw_ghost ── */

static void draw_ghost_at_column(int col, int midline_row, int half_height) {
  /* Faint dot of the true wave, so you can compare it against the samples. */
  int idx = (int)((float)col / (float)COLS * (float)HIGH_RES_N);
  if (idx < 0 || idx >= HIGH_RES_N)
    return;
  float v = g_continuous[idx];
  int row = midline_row - (int)(v * (float)half_height + 0.5f);
  if (row >= 0 && row < LINES) {
    attron(COLOR_PAIR(PAIR_GHOST));
    mvaddch(row, col, '.');
    attroff(COLOR_PAIR(PAIR_GHOST));
  }
}

/* ── §12  draw_recon ── */

static void draw_recon_segment(int col0, int row0, int col1, int row1) {
  /* Straight line between two sample dots (Bresenham). Joining the dots is the
   * naive "what you'd rebuild from samples" guess — and where the alias shows. */
  int dx = abs(col1 - col0), dy = abs(row1 - row0);
  int sx = col0 < col1 ? 1 : -1, sy = row0 < row1 ? 1 : -1;
  int err = dx - dy;
  attron(COLOR_PAIR(PAIR_RECON) | A_BOLD);
  for (;;) {
    if (col0 >= 0 && col0 < COLS && row0 >= 0 && row0 < LINES) {
      int e2 = 2 * err;
      bool bx = e2 > -dy, by = e2 < dx;
      chtype ch = (bx && by) ? (sx == sy ? '\\' : '/') : bx ? '-' : '|';
      mvaddch(row0, col0, ch);
    }
    if (col0 == col1 && row0 == row1)
      break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      col0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      row0 += sy;
    }
  }
  attroff(COLOR_PAIR(PAIR_RECON) | A_BOLD);
}

/* ── §13  draw_sampled ── */

/* Three layers stacked in the bottom panel: the faint true wave behind, the
 * dot-to-dot lines, then bright dots at the actual samples on top. Seeing all
 * three at once is what makes the alias obvious. */
static void draw_sampled_panel(int top_row, int height_rows) {
  int half_height = height_rows / 2;
  if (half_height < 1)
    half_height = 1;
  int midline_row = top_row + half_height;

  /* behind: the faint true wave, thinned to every other column */
  for (int c = 0; c < COLS; c += 2)
    draw_ghost_at_column(c, midline_row, half_height);

  /* work out where each sample dot sits, then join neighbours with lines */
  int sample_columns[N];
  int sample_rows[N];
  for (int n = 0; n < N; n++) {
    sample_columns[n] = (int)(((float)n + 0.5f) / (float)N * (float)COLS);
    if (sample_columns[n] < 0)
      sample_columns[n] = 0;
    if (sample_columns[n] >= COLS)
      sample_columns[n] = COLS - 1;
    sample_rows[n] =
        midline_row - (int)(g_sampled[n] * (float)half_height + 0.5f);
  }
  for (int n = 0; n + 1 < N; n++) {
    draw_recon_segment(sample_columns[n], sample_rows[n], sample_columns[n + 1],
                       sample_rows[n + 1]);
  }

  /* on top: the bright dots that are the only real data we have */
  for (int n = 0; n < N; n++) {
    int col = sample_columns[n];
    int row = sample_rows[n];
    if (row >= 0 && row < LINES && col >= 0 && col < COLS) {
      attron(COLOR_PAIR(PAIR_SAMPLE) | A_BOLD);
      mvaddch(row, col, '*');
      attroff(COLOR_PAIR(PAIR_SAMPLE) | A_BOLD);
    }
  }
}

/* ── §14  draw_debug ── */

/* Two optional overlays that just print numbers; neither changes anything.
 * 'd' walks through the fold-back math live; 'D' lists the raw frequencies. */

static void draw_foldback_overlay(void) {
  if (!g_show_foldback_overlay)
    return;
  int x = 2, y = 2;
  if (y + 8 >= LINES - 1)
    return;

  float true_f = g_true_frequency_cycles_per_buffer;
  float fs = (float)N;
  float f_mod = fmodf(fabsf(true_f), fs);
  bool folded = (f_mod > fs * 0.5f);
  float alias = folded ? (fs - f_mod) : f_mod;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Fold-back arithmetic for true_f = %.2f:", (double)true_f);
  attroff(COLOR_PAIR(PAIR_HINT));

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(y + 1, x, "  Step 1:  f_mod = %.2f mod %.0f = %.2f",
           (double)fabsf(true_f), (double)fs, (double)f_mod);
  if (folded) {
    mvprintw(y + 2, x,
             "  Step 2:  f_mod (%.2f) > fs/2 (%.0f), so:", (double)f_mod,
             (double)(fs * 0.5f));
    mvprintw(y + 3, x, "           alias = %.0f - %.2f = %.2f", (double)fs,
             (double)f_mod, (double)alias);
  } else {
    mvprintw(y + 2, x,
             "  Step 2:  f_mod (%.2f) <= fs/2 (%.0f), so:", (double)f_mod,
             (double)(fs * 0.5f));
    mvprintw(y + 3, x, "           alias = f_mod = %.2f", (double)alias);
  }
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  attron(COLOR_PAIR(folded ? PAIR_NYQUIST : PAIR_RECON) | A_BOLD);
  mvprintw(y + 5, x, "  → Apparent frequency: %.2f cycles/buffer",
           (double)alias);
  if (folded)
    mvprintw(y + 6, x, "  → ALIASED (true content lost)");
  else
    mvprintw(y + 6, x, "  → Faithful (below Nyquist)");
  attroff(COLOR_PAIR(folded ? PAIR_NYQUIST : PAIR_RECON) | A_BOLD);
}

static void draw_frequency_table(void) {
  if (!g_show_frequency_table)
    return;
  /* slide down if the 'd' overlay is already on, so the two don't overlap */
  int x = 2, y = 2;
  if (g_show_foldback_overlay)
    y += 8;
  if (y + 6 >= LINES - 1)
    return;

  float fs = (float)N;
  float nyquist = fs * 0.5f;
  float true_f = g_true_frequency_cycles_per_buffer;
  float alias_f = g_aliased_frequency_cycles_per_buffer;
  float true_norm = true_f / fs; /* re-expressed as cycles per single sample */
  float alias_norm = alias_f / fs;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "Frequency table:");
  attroff(COLOR_PAIR(PAIR_HINT));

  attron(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
  mvprintw(y + 1, x, "  sample rate fs       %6.2f cyc/buf  (= N)", (double)fs);
  mvprintw(y + 2, x, "  Nyquist limit fs/2   %6.2f cyc/buf", (double)nyquist);
  mvprintw(y + 3, x,
           "  true frequency       %6.2f cyc/buf  (= %.4f cyc/sample)",
           (double)true_f, (double)true_norm);
  mvprintw(y + 4, x,
           "  alias frequency      %6.2f cyc/buf  (= %.4f cyc/sample)",
           (double)alias_f, (double)alias_norm);
  attroff(COLOR_PAIR(PAIR_CONTINUOUS) | A_BOLD);
}

/* ── §15  hud ── */

static void draw_hud(void) {
  char status[200];
  bool above_nyquist =
      (g_true_frequency_cycles_per_buffer > (float)N * 0.5f + 1e-3f);
  bool at_nyquist =
      (fabsf(g_true_frequency_cycles_per_buffer - (float)N * 0.5f) < 0.01f);

  const char *status_label = at_nyquist      ? "AT NYQUIST"
                             : above_nyquist ? "ALIASED   "
                                             : "BELOW NYQ ";

  snprintf(
      status, sizeof status,
      " Aliasing  N=%d  fs/2=%.1f  true_f=%5.2f  alias_f=%5.2f  %s  %s  %s ", N,
      (double)((float)N * 0.5f), (double)g_true_frequency_cycles_per_buffer,
      (double)g_aliased_frequency_cycles_per_buffer, status_label,
      g_auto_sweep_enabled ? "AUTO  " : "MANUAL",
      g_simulation_paused ? "PAUSED" : "      ");
  int x = COLS - (int)strlen(status);
  if (x < 0)
    x = 0;
  int pair = above_nyquist ? PAIR_NYQUIST : PAIR_HUD;
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvprintw(0, x, "%s", status);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void draw_hint(void) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(LINES - 1, 0,
           " q:quit  spc:pause  a:auto/manual  +/-:freq  ,/.:fine "
           " d:foldback  D:table  r:reset ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void render_frame(void) {
  erase();

  /* Reserve 4 rows (status, two captions, hints); split the rest into two
   * equal panels — true wave on top, samples below. */
  int rows_for_panels = LINES - 4;
  if (rows_for_panels < 8)
    rows_for_panels = 8;
  int top_h = rows_for_panels / 2;
  int bottom_h = rows_for_panels - top_h;

  int top_label_row = 1;
  int top_panel_top = 2;
  int bottom_label_row = 2 + top_h;
  int bottom_panel_top = 3 + top_h;

  attron(COLOR_PAIR(PAIR_LABEL));
  mvprintw(top_label_row, 0, "Continuous signal x(t)  (the truth)");
  mvprintw(bottom_label_row, 0,
           "Sampled at fs=%d   '*'=sample point  green=reconstruction  "
           "grey=true ghost",
           N);
  attroff(COLOR_PAIR(PAIR_LABEL));

  draw_continuous_panel(top_panel_top, top_h);
  draw_sampled_panel(bottom_panel_top, bottom_h);

  /* overlays sit over the panels but under the status line */
  draw_foldback_overlay();
  draw_frequency_table();

  draw_hud();
  draw_hint();

  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §16  app ── */

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
  case 'a':
  case 'A':
    g_auto_sweep_enabled = !g_auto_sweep_enabled;
    break;
  case '+':
  case '=':
    scene_adjust_freq(+0.5f);
    break;
  case '-':
    scene_adjust_freq(-0.5f);
    break;
  case '.':
    scene_adjust_freq(+0.1f);
    break;
  case ',':
    scene_adjust_freq(-0.1f);
    break;
  case 'd':
    g_show_foldback_overlay = !g_show_foldback_overlay;
    break;
  case 'D':
    g_show_frequency_table = !g_show_frequency_table;
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
