/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * fourier_draw.c — draw any closed path with the arrow keys, press ENTER,
 * and watch a chain of spinning circles redraw it.  The recipe machine is
 * the Fourier transform: it turns your scribble into a list of rotating
 * arms whose tip retraces the loop.
 *
 * Sister files (the algorithm comes apart the same way):
 *   signal/epicycles.c     same chain, but from preset parametric shapes.
 *                          That file is the deep dive on the DFT itself.
 *   signal/fourier_shapes.c preset shapes plus an energy bar.
 *   signal/fft_vis.c       the 1-D cousin on an audio-style signal.
 *
 * Reference: 3Blue1Brown, "But what is a Fourier series? ... drawing with
 * circles" (YouTube) is the friendliest intro to epicycles.
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config — every constant + enum lives here ── */

#define N_SAMPLES 256 /* how finely we sample the path = most arms possible */
#define RAW_MAX 8192  /* most cursor points we'll record */

#define RENDER_FPS 30
#define RENDER_TICK_NS (1000000000LL / RENDER_FPS)
#define CYCLE_FRAMES 300  /* frames to trace the whole shape once */
#define AUTO_ADD_FRAMES 8 /* frames between auto-adding one arm */

#define TRAIL_LEN 600     /* how many tip points the fading trail remembers */
#define N_CIRCLES_DRAWN 6 /* orbit guide circles drawn (the biggest arms) */
#define ARM_TABLE_ROWS 8  /* rows in the 'D' arm-info panel */

/* Each terminal cell is about twice as tall as it is wide.  We say a cell
 * is 8 sub-pixels wide and 16 tall so the spinning circles, which run in
 * this finer pixel space, come out round instead of squashed sideways. */
#define CELL_W 8
#define CELL_H 16

#define ROWS_MAX 128 /* most rows the drawn-cell grid can hold */
#define COLS_MAX 512 /* most cols the drawn-cell grid can hold */

#define N_THEMES 5

/* Colour slots.  The first eleven get re-coloured every time you cycle
 * themes; HUD/HINT/GHOST keep fixed bright colours so the status text and
 * the comparison ghost stay readable against whatever the animation does. */
enum {
  PAIR_ARM_HI = 1,
  PAIR_ARM_MID,
  PAIR_ARM_LO,
  PAIR_CIRCLE,
  PAIR_TRAIL_NEW,
  PAIR_TRAIL_MID,
  PAIR_TRAIL_OLD,
  PAIR_TIP,
  PAIR_PIVOT,
  PAIR_CURSOR,
  PAIR_PATH,
  PAIR_HUD,
  PAIR_HINT,
  PAIR_GHOST,
};

/* ── §2  clock — a forward-only timer and a sleep ── */

/* A clock that only ticks forward (immune to clock changes), so the main
 * loop can measure how long to wait before the next frame. */
static long long clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

/* Without this the loop would peg a CPU core doing nothing between frames. */
static void clock_sleep_ns(long long ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / 1000000000LL, ns % 1000000000LL};
  nanosleep(&ts, NULL);
}

/* ── §3  dft — turn the path into a list of spinning circles ── */

/* A complex number: a 2-D point written as (real, imaginary).  We pack a
 * path point (x, y) as x + i·y so one Fourier transform handles both axes
 * at once. */
typedef struct {
  float re, im;
} Cplx;

/* The Fourier transform: it asks the path "how much of you spins at each
 * whole-number speed?" and answers with one circle per speed.  We avoid
 * calling sin/cos inside the inner loop by spinning a fixed "step" rotation
 * forward one notch at a time — far fewer trig calls, same answer.  Runs
 * once when you press ENTER, never in the per-frame path.  Derivation lives
 * in signal/epicycles.c. */
static void compute_dft_with_twiddle(const Cplx *input, Cplx *output, int N) {
  for (int n = 0; n < N; n++) {
    float twiddle_step_re = cosf(-2.f * (float)M_PI * (float)n / (float)N);
    float twiddle_step_im = sinf(-2.f * (float)M_PI * (float)n / (float)N);
    float twiddle_re = 1.f, twiddle_im = 0.f;
    float acc_re = 0.f, acc_im = 0.f;

    for (int k = 0; k < N; k++) {
      acc_re += input[k].re * twiddle_re - input[k].im * twiddle_im;
      acc_im += input[k].re * twiddle_im + input[k].im * twiddle_re;
      float next_twiddle_re =
          twiddle_re * twiddle_step_re - twiddle_im * twiddle_step_im;
      twiddle_im = twiddle_re * twiddle_step_im + twiddle_im * twiddle_step_re;
      twiddle_re = next_twiddle_re;
    }
    output[n].re = acc_re;
    output[n].im = acc_im;
  }
}

/* ── §4  resample — space the path out evenly (the one trick unique here) ── */

/*
 * The Fourier transform expects samples spread evenly along the path.  But
 * the user might draw fast in some spots and slow in others, so the raw
 * points bunch up unevenly.  This walks the path with a tape measure and
 * drops N new points at equal distances along it, regardless of the original
 * spacing — so the math doesn't get fooled into thinking a slowly-drawn
 * stretch matters more.  It also slides the shape so its centre sits at the
 * origin, and reports the farthest point out (used later to size it to fit
 * the screen).  Returns that radius; fills output_samples and the centroid.
 *
 * The arc[] running-length buffer goes on the heap, not the stack, because
 * the path can be RAW_MAX = 8192 points and that's a lot of stack.  The
 * search marches `j` forward only, so the whole thing is linear time.
 */
static float resample_path_to_uniform_arc_length(
    const float *raw_pixel_x, const float *raw_pixel_y, int raw_point_count,
    Cplx *output_samples, int output_count, float *out_centroid_x,
    float *out_centroid_y) {
  float *arc = malloc((size_t)raw_point_count * sizeof(float));
  if (!arc)
    return 1.f;

  /* Running distance from the start to each point along the path. */
  arc[0] = 0.f;
  for (int i = 1; i < raw_point_count; i++) {
    float dx = raw_pixel_x[i] - raw_pixel_x[i - 1];
    float dy = raw_pixel_y[i] - raw_pixel_y[i - 1];
    arc[i] = arc[i - 1] + sqrtf(dx * dx + dy * dy);
  }
  float total = arc[raw_point_count - 1];
  if (total < 0.001f)
    total = 1.f; /* a single-cell scribble has no length; avoid divide-by-0 */

  /* Drop a new point every (total / N) of distance, sliding along whichever
   * raw segment that distance falls in. */
  int j = 0;
  for (int k = 0; k < output_count; k++) {
    float s_k = (float)k / (float)output_count * total;
    while (j < raw_point_count - 2 && arc[j + 1] < s_k)
      j++;
    float span = arc[j + 1] - arc[j];
    float t = (span > 0.001f) ? (s_k - arc[j]) / span : 0.f;
    output_samples[k].re =
        raw_pixel_x[j] + t * (raw_pixel_x[j + 1] - raw_pixel_x[j]);
    output_samples[k].im =
        raw_pixel_y[j] + t * (raw_pixel_y[j + 1] - raw_pixel_y[j]);
  }
  free(arc);

  /* Find the shape's centre point and slide it to the origin. */
  float mean_re = 0.f, mean_im = 0.f;
  for (int k = 0; k < output_count; k++) {
    mean_re += output_samples[k].re;
    mean_im += output_samples[k].im;
  }
  mean_re /= (float)output_count;
  mean_im /= (float)output_count;

  if (out_centroid_x)
    *out_centroid_x = mean_re;
  if (out_centroid_y)
    *out_centroid_y = mean_im;

  float max_radius = 0.f;
  for (int k = 0; k < output_count; k++) {
    output_samples[k].re -= mean_re;
    output_samples[k].im -= mean_im;
    float r = sqrtf(output_samples[k].re * output_samples[k].re +
                    output_samples[k].im * output_samples[k].im);
    if (r > max_radius)
      max_radius = r;
  }
  return (max_radius > 0.001f) ? max_radius : 1.f;
}

/* ── §5  epicycle_table — one spinning arm per circle, biggest first ── */

/* One rotating arm in the chain.  Each comes straight out of the Fourier
 * transform: how long the arm is, where it starts pointing, and how fast it
 * spins. */
typedef struct {
  float amplitude_normalised; /* arm length, 0..1 (gets scaled to pixels later) */
  float phase_offset_radians; /* the angle it points at when the clock is 0 */
  int frequency_signed;       /* spins per cycle; negative means clockwise */
} Epicycle;

/* Sort comparator: longest arm first. */
static int epicycle_compare_descending(const void *a, const void *b) {
  float aa = ((const Epicycle *)a)->amplitude_normalised;
  float bb = ((const Epicycle *)b)->amplitude_normalised;
  return (aa < bb) - (aa > bb);
}

static Epicycle g_epicycle_table[N_SAMPLES];
static int g_total_epicycle_count = 0;
static int g_active_epicycle_count = 0;

/* Run the transform, read each circle's length/angle/speed out of it, and
 * sort so the biggest arms come first.  That way "draw with M arms" always
 * means the M that matter most. */
static void build_sorted_epicycle_table(const Cplx *samples, int N) {
  Cplx dft[N_SAMPLES];
  compute_dft_with_twiddle(samples, dft, N);
  float inv_N = 1.f / (float)N;
  for (int n = 0; n < N; n++) {
    Epicycle e;
    e.amplitude_normalised =
        sqrtf(dft[n].re * dft[n].re + dft[n].im * dft[n].im) * inv_N;
    e.phase_offset_radians = atan2f(dft[n].im, dft[n].re);
    /* The upper half of the transform is really the clockwise (negative)
     * speeds folded around; map them back so those arms spin the right way. */
    e.frequency_signed = (n <= N / 2) ? n : n - N;
    g_epicycle_table[n] = e;
  }
  qsort(g_epicycle_table, N, sizeof(Epicycle), epicycle_compare_descending);
  g_total_epicycle_count = N;
  g_active_epicycle_count = 1;
}

/* ── §6  themes — colour sets you cycle with t/T ── */

/* One colour scheme.  Each names a colour for every moving piece — the
 * arms, the orbit guides, the trail (three fade tiers), the tip and pivot
 * markers, the draw-mode cursor and path.  Colours stay in the bright half
 * of the palette so nothing vanishes against a black terminal.  The *8
 * fields are plainer fallbacks for old 8-colour terminals. */
typedef struct {
  short arm_hi, arm_mid, arm_lo;
  short circle, trail_new, trail_mid, trail_old;
  short tip, pivot, cursor, path;
  short arm_hi8, arm_mid8, arm_lo8;
  short circle8, trail_new8, trail_mid8, trail_old8;
  short tip8, pivot8, cursor8, path8;
  const char *display_name;
} ThemePalette;

static const ThemePalette theme_table[N_THEMES] = {
    /* Classic — bright white arms, gold/orange/red trail */
    {231,          51,           245,        244,         226,
     208,          196,          231,        220,         226,
     250,          COLOR_WHITE,  COLOR_CYAN, COLOR_WHITE, COLOR_WHITE,
     COLOR_YELLOW, COLOR_YELLOW, COLOR_RED,  COLOR_WHITE, COLOR_YELLOW,
     COLOR_YELLOW, COLOR_WHITE,  "Classic"},
    /* Fire — red/orange arms, peach orbit guides */
    {208,          196,          88,        215,         226,
     214,          196,          231,       208,         226,
     202,          COLOR_RED,    COLOR_RED, COLOR_RED,   COLOR_WHITE,
     COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_WHITE, COLOR_RED,
     COLOR_YELLOW, COLOR_RED,    "Fire"},
    /* Neon — magenta/violet arms, lavender guides */
    {207,           165,           93,          141,
     219,           213,           201,         231,
     207,           219,           201,         COLOR_MAGENTA,
     COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE, COLOR_MAGENTA,
     COLOR_MAGENTA, COLOR_MAGENTA, "Neon"},
    /* Ocean — teal/blue arms, steel-blue guides */
    {123,         45,         31,          110,        159,        87,
     51,          231,        45,          123,        39,         COLOR_CYAN,
     COLOR_BLUE,  COLOR_BLUE, COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE,
     COLOR_WHITE, COLOR_CYAN, COLOR_CYAN,  COLOR_BLUE, "Ocean"},
    /* Matrix — green spectrum, olive-green guides */
    {118,         46,          28,          64,          154,
     118,         46,          231,         118,         154,
     82,          COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
     COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_GREEN,
     COLOR_GREEN, COLOR_GREEN, "Matrix"},
};

static int g_active_theme_index = 0;

/* ── §7  colors — load a theme, set up the fixed HUD colours ── */

/* Re-colour only the eleven animation slots; HUD/HINT/GHOST stay put. */
static void apply_theme(int theme_index) {
  if (theme_index < 0 || theme_index >= N_THEMES)
    theme_index = 0;
  const ThemePalette *t = &theme_table[theme_index];
  if (COLORS >= 256) {
    init_pair(PAIR_ARM_HI, t->arm_hi, -1);
    init_pair(PAIR_ARM_MID, t->arm_mid, -1);
    init_pair(PAIR_ARM_LO, t->arm_lo, -1);
    init_pair(PAIR_CIRCLE, t->circle, -1);
    init_pair(PAIR_TRAIL_NEW, t->trail_new, -1);
    init_pair(PAIR_TRAIL_MID, t->trail_mid, -1);
    init_pair(PAIR_TRAIL_OLD, t->trail_old, -1);
    init_pair(PAIR_TIP, t->tip, -1);
    init_pair(PAIR_PIVOT, t->pivot, -1);
    init_pair(PAIR_CURSOR, t->cursor, -1);
    init_pair(PAIR_PATH, t->path, -1);
  } else {
    init_pair(PAIR_ARM_HI, t->arm_hi8, -1);
    init_pair(PAIR_ARM_MID, t->arm_mid8, -1);
    init_pair(PAIR_ARM_LO, t->arm_lo8, -1);
    init_pair(PAIR_CIRCLE, t->circle8, -1);
    init_pair(PAIR_TRAIL_NEW, t->trail_new8, -1);
    init_pair(PAIR_TRAIL_MID, t->trail_mid8, -1);
    init_pair(PAIR_TRAIL_OLD, t->trail_old8, -1);
    init_pair(PAIR_TIP, t->tip8, -1);
    init_pair(PAIR_PIVOT, t->pivot8, -1);
    init_pair(PAIR_CURSOR, t->cursor8, -1);
    init_pair(PAIR_PATH, t->path8, -1);
  }
}

/* Set the fixed HUD/HINT/GHOST colours once, then load the first theme. */
static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);   /* bright yellow */
    init_pair(PAIR_HINT, 51, -1);   /* bright cyan   */
    init_pair(PAIR_GHOST, 244, -1); /* medium grey   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_GHOST, COLOR_WHITE, -1);
  }
  apply_theme(0);
}

/* ── §8  trail — the fading streak the pen tip leaves ── */

/* A fixed-size loop of recent tip positions.  When it fills up, new points
 * overwrite the oldest — so we always show roughly the last TRAIL_LEN points
 * of the pen's path without ever growing. */
typedef struct {
  float pixel_x[TRAIL_LEN];
  float pixel_y[TRAIL_LEN];
  int write_head;   /* where the next point goes */
  int filled_count; /* how many points are live, capped at TRAIL_LEN */
} TipTrail;

static TipTrail g_tip_trail;

static void trail_push(TipTrail *t, float pixel_x, float pixel_y) {
  t->pixel_x[t->write_head] = pixel_x;
  t->pixel_y[t->write_head] = pixel_y;
  t->write_head = (t->write_head + 1) % TRAIL_LEN;
  if (t->filled_count < TRAIL_LEN)
    t->filled_count++;
}

/* Forget the trail (just resets the counters; old data is harmless). */
static void trail_clear(TipTrail *t) {
  t->write_head = 0;
  t->filled_count = 0;
}

/* ── §9  draw_primitives — pixel↔cell, lines, orbit circles ── */

static int g_screen_rows = 0; /* current terminal size; refreshed on resize */
static int g_screen_cols = 0;

/* Round a fine pixel coordinate down to the terminal cell it lands in. */
static int px_to_cell_x(float pixel_x) {
  return (int)(pixel_x / CELL_W + 0.5f);
}
static int px_to_cell_y(float pixel_y) {
  return (int)(pixel_y / CELL_H + 0.5f);
}

/* Draw a straight line of characters between two cells, picking '-', '|',
 * '/' or '\\' to match the slope.  Skips the bottom row, which the hint
 * line owns. */
static void render_bresenham_line(int x0, int y0, int x1, int y1, attr_t attr) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  for (;;) {
    if (x0 >= 0 && x0 < g_screen_cols && y0 >= 0 && y0 < g_screen_rows - 1) {
      int e2 = 2 * err;
      bool bx = e2 > -dy, by = e2 < dx;
      chtype glyph = (bx && by) ? (sx == sy ? '\\' : '/') : bx ? '-' : '|';
      attron(attr);
      mvaddch(y0, x0, glyph);
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

/* The grid of cells the user has drawn on (logically part of §10, declared
 * here because the next function needs it). */
static uint8_t g_drawn_cells[ROWS_MAX][COLS_MAX];

/* Same line-walk, but flags cells in the drawn grid instead of painting the
 * screen.  Keeps the drawn line unbroken even when fast key-repeat jumps the
 * cursor several cells in one go. */
static void mark_bresenham_path_in_grid(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
  int err = dx - dy;
  for (;;) {
    if (y0 >= 0 && y0 < ROWS_MAX && x0 >= 0 && x0 < COLS_MAX)
      g_drawn_cells[y0][x0] = 1;
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

/* Draw the guide circle for one arm as a ring of dots.  It comes out as an
 * ellipse on screen only because cells are taller than wide; to the eye it
 * reads as round.  We dot more points around bigger rings so they stay
 * smooth. */
static void render_orbit_ellipse(float pivot_pixel_x, float pivot_pixel_y,
                                 float radius_pixels) {
  if (radius_pixels < (float)CELL_W * 0.5f)
    return;

  float semi_axis_x = radius_pixels / (float)CELL_W;
  float semi_axis_y = radius_pixels / (float)CELL_H;
  int center_col = px_to_cell_x(pivot_pixel_x);
  int center_row = px_to_cell_y(pivot_pixel_y);
  int sample_count =
      (int)(2.f * (float)M_PI * fmaxf(semi_axis_x, semi_axis_y)) + 4;

  attr_t attr = COLOR_PAIR(PAIR_CIRCLE);
  attron(attr);
  for (int i = 0; i < sample_count; i++) {
    float theta = 2.f * (float)M_PI * (float)i / (float)sample_count;
    int x = center_col + (int)(semi_axis_x * cosf(theta) + 0.5f);
    int y = center_row + (int)(semi_axis_y * sinf(theta) + 0.5f);
    if (x >= 0 && x < g_screen_cols && y >= 0 && y < g_screen_rows - 1)
      mvaddch(y, x, '.');
  }
  attroff(attr);
}

/* ── §10  draw_state — the cursor, the path you're recording ── */

/* Which half of the program we're in: drawing, or playing it back. */
typedef enum { STATE_DRAW, STATE_PLAY } AppState;

static AppState g_app_state = STATE_DRAW;

/* The drawing cursor, in whole cells (one cell per arrow press). */
static int g_cursor_col = 0;
static int g_cursor_row = 0;

/* The recorded path, in fine pixel coordinates — one point per keypress. */
static float g_raw_pixel_x[RAW_MAX];
static float g_raw_pixel_y[RAW_MAX];
static int g_raw_point_count = 0;

/* (g_drawn_cells, the on/off grid of drawn cells, lives up in §9.) */

static bool g_draw_error_too_few = false; /* set when you try to play <4 points */

/* Put the cursor in the middle and wipe any earlier drawing. */
static void reset_draw_state(void) {
  g_cursor_col = g_screen_cols / 2;
  g_cursor_row = (g_screen_rows - 1) / 2;
  g_raw_point_count = 0;
  g_draw_error_too_few = false;
  memset(g_drawn_cells, 0, sizeof g_drawn_cells);
}

/* Record where the cursor is now.  Skips a point that repeats the last one,
 * because a zero-length step would later trip up the even-spacing math. */
static void record_current_cell_as_raw_point(void) {
  if (g_raw_point_count >= RAW_MAX)
    return;
  float pixel_x = (float)g_cursor_col * CELL_W;
  float pixel_y = (float)g_cursor_row * CELL_H;
  if (g_raw_point_count > 0 &&
      g_raw_pixel_x[g_raw_point_count - 1] == pixel_x &&
      g_raw_pixel_y[g_raw_point_count - 1] == pixel_y)
    return;
  g_raw_pixel_x[g_raw_point_count] = pixel_x;
  g_raw_pixel_y[g_raw_point_count] = pixel_y;
  g_raw_point_count++;
  if (g_cursor_row >= 0 && g_cursor_row < ROWS_MAX && g_cursor_col >= 0 &&
      g_cursor_col < COLS_MAX)
    g_drawn_cells[g_cursor_row][g_cursor_col] = 1;
}

/* Move the cursor one step, drawing in every cell it crosses and recording
 * the new spot.  The fill keeps the on-screen line solid; the single
 * recorded point is the real data the transform will use. */
static void move_cursor_with_bresenham_fill(int delta_col, int delta_row) {
  int new_col = g_cursor_col + delta_col;
  int new_row = g_cursor_row + delta_row;
  if (new_col < 0)
    new_col = 0;
  if (new_col >= g_screen_cols)
    new_col = g_screen_cols - 1;
  if (new_row < 0)
    new_row = 0;
  if (new_row >= g_screen_rows - 1)
    new_row = g_screen_rows - 2;
  if (new_col == g_cursor_col && new_row == g_cursor_row)
    return;

  mark_bresenham_path_in_grid(g_cursor_col, g_cursor_row, new_col, new_row);
  g_cursor_col = new_col;
  g_cursor_row = new_row;
  record_current_cell_as_raw_point();
}

/* Size of the drawing in cells (width x height), for the draw-mode HUD. */
static bool compute_bbox_in_cells(int *out_w_cells, int *out_h_cells) {
  if (g_raw_point_count == 0) {
    *out_w_cells = 0;
    *out_h_cells = 0;
    return false;
  }
  float min_x = g_raw_pixel_x[0], max_x = g_raw_pixel_x[0];
  float min_y = g_raw_pixel_y[0], max_y = g_raw_pixel_y[0];
  for (int i = 1; i < g_raw_point_count; i++) {
    if (g_raw_pixel_x[i] < min_x)
      min_x = g_raw_pixel_x[i];
    if (g_raw_pixel_x[i] > max_x)
      max_x = g_raw_pixel_x[i];
    if (g_raw_pixel_y[i] < min_y)
      min_y = g_raw_pixel_y[i];
    if (g_raw_pixel_y[i] > max_y)
      max_y = g_raw_pixel_y[i];
  }
  *out_w_cells = (int)((max_x - min_x) / CELL_W + 0.5f) + 1;
  *out_h_cells = (int)((max_y - min_y) / CELL_H + 0.5f) + 1;
  return true;
}

/* ── §11  draw_compute — turn the drawing into a playable chain ── */

/* Where and how big to draw the chain.  Worked out once when you press
 * ENTER and reused every frame.  Pivot is the chain's anchor (screen
 * centre); scale stretches the unit-sized shape to fit the window; the
 * centroid and max radius are kept so a resize can re-fit it. */
static float g_screen_pivot_pixel_x = 0.f;
static float g_screen_pivot_pixel_y = 0.f;
static float g_pixel_scale_per_unit = 1.f;
static float g_max_radius_pixels = 1.f;
static float g_path_centroid_pixel_x = 0.f;
static float g_path_centroid_pixel_y = 0.f;

/* The chain's joints this frame: joint[0] is the anchor, joint[i+1] the tip
 * of arm i, and the last one is the pen. */
static float g_joint_pixel_x[N_SAMPLES + 1];
static float g_joint_pixel_y[N_SAMPLES + 1];

static bool g_close_path_enabled = false; /* the 'o' toggle */

/* The ENTER transition: even out the path, transform it, size it to the
 * screen, and switch to playback.  Bails out (staying in draw mode) if the
 * drawing is too short to make sense of. */
static bool switch_from_draw_to_play(void) {
  if (g_raw_point_count < 4) {
    g_draw_error_too_few = true;
    return false;
  }
  g_draw_error_too_few = false;

  /* Work on a copy so the 'o' toggle stays reversible.  If closing is on,
   * tack a copy of the first point onto the end to seal the loop. */
  static float local_pixel_x[RAW_MAX + 1];
  static float local_pixel_y[RAW_MAX + 1];
  int n = g_raw_point_count;
  memcpy(local_pixel_x, g_raw_pixel_x, (size_t)n * sizeof(float));
  memcpy(local_pixel_y, g_raw_pixel_y, (size_t)n * sizeof(float));
  if (g_close_path_enabled) {
    local_pixel_x[n] = local_pixel_x[0];
    local_pixel_y[n] = local_pixel_y[0];
    n++;
  }

  /* Even out the spacing, then read off the circles, biggest first. */
  Cplx samples[N_SAMPLES];
  g_max_radius_pixels = resample_path_to_uniform_arc_length(
      local_pixel_x, local_pixel_y, n, samples, N_SAMPLES,
      &g_path_centroid_pixel_x, &g_path_centroid_pixel_y);

  build_sorted_epicycle_table(samples, N_SAMPLES);

  /* Anchor at the centre; scale the shape to about 40% of the smaller side. */
  g_screen_pivot_pixel_x = (float)g_screen_cols * CELL_W * 0.5f;
  g_screen_pivot_pixel_y = (float)(g_screen_rows - 1) * CELL_H * 0.5f;
  float screen_radius_pixels = fminf((float)g_screen_cols * CELL_W,
                                     (float)(g_screen_rows - 1) * CELL_H) *
                               0.40f;
  g_pixel_scale_per_unit = screen_radius_pixels / g_max_radius_pixels;

  g_app_state = STATE_PLAY;
  return true;
}

/* ── §12  play_state — spin the chain forward one frame at a time ── */

static float g_animation_phase_radians = 0.f; /* the master clock, 0..2pi */
static int g_auto_add_counter = 0;
static bool g_simulation_paused = false;
static bool g_auto_add_enabled = true;
static bool g_show_orbit_circles = true;

/* Overlay toggles, kept when you bounce between draw and play. */
static bool g_show_ghost_overlay = true;
static bool g_show_arm_table = false;

/* Place every joint for the current clock angle: start at the anchor, then
 * step out along each arm in turn.  The last joint is the pen tip. */
static void compute_chain_joint_positions(void) {
  float x = g_screen_pivot_pixel_x;
  float y = g_screen_pivot_pixel_y;
  g_joint_pixel_x[0] = x;
  g_joint_pixel_y[0] = y;
  for (int i = 0; i < g_active_epicycle_count; i++) {
    const Epicycle *e = &g_epicycle_table[i];
    float angle = (float)e->frequency_signed * g_animation_phase_radians +
                  e->phase_offset_radians;
    float radius = e->amplitude_normalised * g_pixel_scale_per_unit;
    x += radius * cosf(angle);
    y += radius * sinf(angle);
    g_joint_pixel_x[i + 1] = x;
    g_joint_pixel_y[i + 1] = y;
  }
}

/* Advance the animation by one frame.  Just updates state; the drawing code
 * in §14 reads it afterwards. */
static void play_simulation_tick(void) {
  if (g_simulation_paused)
    return;

  /* If auto-add is on, slip in one more arm every few frames. */
  if (g_auto_add_enabled && g_active_epicycle_count < g_total_epicycle_count) {
    g_auto_add_counter++;
    if (g_auto_add_counter >= AUTO_ADD_FRAMES) {
      g_auto_add_counter = 0;
      g_active_epicycle_count++;
    }
  }

  /* Tick the clock.  When it completes a lap, wipe the trail so the next
   * lap draws on a fresh canvas instead of piling up. */
  g_animation_phase_radians += 2.f * (float)M_PI / (float)CYCLE_FRAMES;
  if (g_animation_phase_radians >= 2.f * (float)M_PI) {
    g_animation_phase_radians -= 2.f * (float)M_PI;
    trail_clear(&g_tip_trail);
  }

  compute_chain_joint_positions();
  trail_push(&g_tip_trail, g_joint_pixel_x[g_active_epicycle_count],
             g_joint_pixel_y[g_active_epicycle_count]);
}

/* ── §13  project — place an original point where the chain would draw it ── */

/* Run an original drawn point through the exact same centre-scale-anchor
 * steps the chain uses, so the faded "ghost" of your drawing lands right on
 * top of what the chain produces and the two can be compared. */
static void project_raw_pixel_to_cell(float raw_pixel_x, float raw_pixel_y,
                                      int *out_col, int *out_row) {
  float screen_pixel_x =
      (raw_pixel_x - g_path_centroid_pixel_x) * g_pixel_scale_per_unit +
      g_screen_pivot_pixel_x;
  float screen_pixel_y =
      (raw_pixel_y - g_path_centroid_pixel_y) * g_pixel_scale_per_unit +
      g_screen_pivot_pixel_y;
  *out_col = px_to_cell_x(screen_pixel_x);
  *out_row = px_to_cell_y(screen_pixel_y);
}

/* ── §14  render_layers — paint back to front: ghost, orbits, trail, arms, marks ── */

/* The faded dotted copy of your original drawing, sitting under the chain so
 * you can watch the reconstruction close in on it. */
static void render_ghost_overlay(void) {
  if (!g_show_ghost_overlay || g_raw_point_count < 2)
    return;
  attron(COLOR_PAIR(PAIR_GHOST));
  for (int i = 0; i < g_raw_point_count; i++) {
    int col, row;
    project_raw_pixel_to_cell(g_raw_pixel_x[i], g_raw_pixel_y[i], &col, &row);
    if (col >= 0 && col < g_screen_cols && row >= 0 && row < g_screen_rows - 1)
      mvaddch(row, col, '.');
  }
  attroff(COLOR_PAIR(PAIR_GHOST));
}

/* Guide circles for the biggest few arms.  Capped at N_CIRCLES_DRAWN because
 * more just becomes clutter and the tiny ones shrink to a single dot anyway. */
static void render_orbit_layer(void) {
  if (!g_show_orbit_circles)
    return;
  int draw_count = (g_active_epicycle_count < N_CIRCLES_DRAWN)
                       ? g_active_epicycle_count
                       : N_CIRCLES_DRAWN;
  for (int i = 0; i < draw_count; i++) {
    float radius_pixels =
        g_epicycle_table[i].amplitude_normalised * g_pixel_scale_per_unit;
    render_orbit_ellipse(g_joint_pixel_x[i], g_joint_pixel_y[i], radius_pixels);
  }
}

/* The pen's recent path, drawn oldest-first so newer dots sit on top.  Three
 * colour bands fake a fade from old to fresh. */
static void render_trail_layer(void) {
  int filled = g_tip_trail.filled_count;
  if (filled == 0)
    return;
  int oldest_index = (g_tip_trail.write_head - filled + TRAIL_LEN) % TRAIL_LEN;
  for (int i = 0; i < filled; i++) {
    int idx = (oldest_index + i) % TRAIL_LEN;
    int col = px_to_cell_x(g_tip_trail.pixel_x[idx]);
    int row = px_to_cell_y(g_tip_trail.pixel_y[idx]);
    if (col < 0 || col >= g_screen_cols)
      continue;
    if (row < 0 || row >= g_screen_rows - 1)
      continue;
    float age_fraction = (float)i / (float)(filled > 1 ? filled : 1);
    int pair_id = (age_fraction > 0.70f)   ? PAIR_TRAIL_NEW
                  : (age_fraction > 0.35f) ? PAIR_TRAIL_MID
                                           : PAIR_TRAIL_OLD;
    attron(COLOR_PAIR(pair_id) | A_BOLD);
    mvaddch(row, col, '*');
    attroff(COLOR_PAIR(pair_id) | A_BOLD);
  }
}

/* Pick a colour band by arm size so big arms stand out from tiny ones. */
static int arm_pair_for_radius(float radius_pixels) {
  if (radius_pixels > g_pixel_scale_per_unit * 0.08f)
    return PAIR_ARM_HI;
  if (radius_pixels > g_pixel_scale_per_unit * 0.02f)
    return PAIR_ARM_MID;
  return PAIR_ARM_LO;
}

/* The arms themselves: a line from each joint to the next, coloured by size. */
static void render_chain_layer(void) {
  for (int i = 0; i < g_active_epicycle_count; i++) {
    float radius_pixels =
        g_epicycle_table[i].amplitude_normalised * g_pixel_scale_per_unit;
    attr_t attr = COLOR_PAIR(arm_pair_for_radius(radius_pixels)) | A_BOLD;
    render_bresenham_line(px_to_cell_x(g_joint_pixel_x[i]),
                          px_to_cell_y(g_joint_pixel_y[i]),
                          px_to_cell_x(g_joint_pixel_x[i + 1]),
                          px_to_cell_y(g_joint_pixel_y[i + 1]), attr);
  }
}

/* Bright markers: '@' for the pen at the tip, '+' for the anchor it all
 * hangs from. */
static void render_markers_layer(void) {
  int tip_col = px_to_cell_x(g_joint_pixel_x[g_active_epicycle_count]);
  int tip_row = px_to_cell_y(g_joint_pixel_y[g_active_epicycle_count]);
  if (tip_col >= 0 && tip_col < g_screen_cols && tip_row >= 0 &&
      tip_row < g_screen_rows - 1) {
    attron(COLOR_PAIR(PAIR_TIP) | A_BOLD);
    mvaddch(tip_row, tip_col, '@');
    attroff(COLOR_PAIR(PAIR_TIP) | A_BOLD);
  }
  int pivot_col = px_to_cell_x(g_screen_pivot_pixel_x);
  int pivot_row = px_to_cell_y(g_screen_pivot_pixel_y);
  if (pivot_col >= 0 && pivot_col < g_screen_cols && pivot_row >= 0 &&
      pivot_row < g_screen_rows - 1) {
    attron(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
    mvaddch(pivot_row, pivot_col, '+');
    attroff(COLOR_PAIR(PAIR_PIVOT) | A_BOLD);
  }
}

/* ── §15  render_debug — the 'D' panel listing the loudest arms ── */

/* A little text table of the biggest arms (size, speed, start angle) for
 * anyone who wants to check the numbers or see which arms do the heavy
 * lifting. */
static void render_arm_table_overlay(void) {
  if (!g_show_arm_table)
    return;
  int rows_to_show = (g_active_epicycle_count < ARM_TABLE_ROWS)
                         ? g_active_epicycle_count
                         : ARM_TABLE_ROWS;
  int x = 2, y = 2;
  if (y + rows_to_show + 1 >= g_screen_rows - 1)
    return;

  attron(COLOR_PAIR(PAIR_HINT));
  mvprintw(y, x, "  i  freq    amp     phase");
  attroff(COLOR_PAIR(PAIR_HINT));

  for (int i = 0; i < rows_to_show; i++) {
    const Epicycle *e = &g_epicycle_table[i];
    int pair = (i < 1) ? PAIR_ARM_HI : (i < 6) ? PAIR_ARM_MID : PAIR_ARM_LO;
    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(y + 1 + i, x, " %2d  %4d  %5.3f  %+5.2f", i, e->frequency_signed,
             (double)e->amplitude_normalised, (double)e->phase_offset_radians);
    attroff(COLOR_PAIR(pair) | A_BOLD);
  }
}

/* ── §16  hud — status top-right, key hints bottom, paused chip ── */

static void hud_paint_top_right(const char *text) {
  int x = g_screen_cols - (int)strlen(text);
  if (x < 0)
    x = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, x, "%s", text);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_paused_chip(void) {
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
  mvprintw(0, 0, " PAUSED ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD | A_REVERSE);
}

static void hud_paint_hint(const char *text) {
  move(g_screen_rows - 1, 0);
  clrtoeol();
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(g_screen_rows - 1, 0, "%s", text);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §17  draw_mode_render — the screen while you're drawing ── */

static void render_draw_mode(void) {
  /* The drawn cells as '*'. */
  for (int r = 0; r < g_screen_rows - 1 && r < ROWS_MAX; r++) {
    for (int c = 0; c < g_screen_cols && c < COLS_MAX; c++) {
      if (!g_drawn_cells[r][c])
        continue;
      attron(COLOR_PAIR(PAIR_PATH));
      mvaddch(r, c, '*');
      attroff(COLOR_PAIR(PAIR_PATH));
    }
  }

  /* The cursor, on top. */
  if (g_cursor_row < g_screen_rows - 1) {
    attron(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
    mvaddch(g_cursor_row, g_cursor_col, '@');
    attroff(COLOR_PAIR(PAIR_CURSOR) | A_BOLD);
  }

  /* Status line: point count, cursor, drawing size, toggles, theme. */
  int bw = 0, bh = 0;
  compute_bbox_in_cells(&bw, &bh);
  char buf[180];
  if (g_draw_error_too_few) {
    snprintf(buf, sizeof buf,
             " DRAW  need >= 4 pts  pts:%d/%d  cur:(%d,%d)  bbox:%dx%d  "
             "closed:%s  thm:%s ",
             g_raw_point_count, RAW_MAX, g_cursor_col, g_cursor_row, bw, bh,
             g_close_path_enabled ? "Y" : "N",
             theme_table[g_active_theme_index].display_name);
  } else {
    snprintf(buf, sizeof buf,
             " DRAW  pts:%d/%d  cur:(%d,%d)  bbox:%dx%d  closed:%s  thm:%s ",
             g_raw_point_count, RAW_MAX, g_cursor_col, g_cursor_row, bw, bh,
             g_close_path_enabled ? "Y" : "N",
             theme_table[g_active_theme_index].display_name);
  }
  hud_paint_top_right(buf);
  hud_paint_hint(
      " arrows/WASD:draw  ENTER/g:play  c:clear  o:close  t/T:theme  q:quit ");
}

/* ── §18  play_mode_render — the screen during playback ── */

static void render_play_mode(void) {
  /* Back-to-front so the important stuff ends up on top. */
  render_ghost_overlay();
  render_orbit_layer();
  render_trail_layer();
  render_chain_layer();
  render_markers_layer();

  render_arm_table_overlay();

  char buf[180];
  snprintf(buf, sizeof buf, " PLAY  arms:%d/%d  thm:%s  closed:%s  ghost:%s ",
           g_active_epicycle_count, g_total_epicycle_count,
           theme_table[g_active_theme_index].display_name,
           g_close_path_enabled ? "Y" : "N",
           g_show_ghost_overlay ? "ON" : "off");
  hud_paint_top_right(buf);
  if (g_simulation_paused)
    hud_paint_paused_chip();
  hud_paint_hint(" r:redraw  p:pause  +/-:arms  c:circles  o:close  d:ghost  "
                 "D:arms  t/T:theme  a:auto  q:quit ");
}

/* ── §19  screen — start and stop ncurses ── */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init();
}

static void screen_cleanup(void) { endwin(); }

/* ── §20  app — signals, the main loop, key handling ── */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGINT || sig == SIGTERM)
    g_should_quit = 1;
  if (sig == SIGWINCH)
    g_resize_pending = 1;
}

/* 'o' during playback: flip path-closing and rebuild the chain on the spot
 * so you can watch the seam come and go without going back to draw mode. */
static void handle_play_mode_close_toggle(void) {
  g_close_path_enabled = !g_close_path_enabled;
  int saved_active = g_active_epicycle_count;
  if (switch_from_draw_to_play()) {
    /* The rebuild resets the arm count to 1; put your count back so the
     * before/after comparison stays steady. */
    g_active_epicycle_count = saved_active;
    if (g_active_epicycle_count > g_total_epicycle_count)
      g_active_epicycle_count = g_total_epicycle_count;
    g_app_state = STATE_PLAY;
  }
}

int main(void) {
  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  screen_init();
  getmaxyx(stdscr, g_screen_rows, g_screen_cols);
  reset_draw_state();

  long long last_frame_ns = clock_now_ns();

  while (!g_should_quit) {
    /* resize */
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, g_screen_rows, g_screen_cols);
      if (g_app_state == STATE_DRAW) {
        reset_draw_state();
      } else {
        /* Re-fit the chain to the new window using the radius we saved, so
         * it keeps filling about 40% of the screen no matter how often you
         * resize. */
        g_screen_pivot_pixel_x = (float)g_screen_cols * CELL_W * 0.5f;
        g_screen_pivot_pixel_y = (float)(g_screen_rows - 1) * CELL_H * 0.5f;
        float screen_radius_pixels =
            fminf((float)g_screen_cols * CELL_W,
                  (float)(g_screen_rows - 1) * CELL_H) *
            0.40f;
        g_pixel_scale_per_unit = screen_radius_pixels / g_max_radius_pixels;
        trail_clear(&g_tip_trail);
      }
      last_frame_ns = clock_now_ns();
      continue;
    }

    /* input */
    int ch;
    while ((ch = getch()) != ERR) {
      if (g_app_state == STATE_DRAW) {
        switch (ch) {
        case 'q':
        case 'Q':
        case 27:
          g_should_quit = 1;
          break;
        case KEY_UP:
        case 'w':
        case 'W':
          move_cursor_with_bresenham_fill(0, -1);
          break;
        case KEY_DOWN:
        case 's':
        case 'S':
          move_cursor_with_bresenham_fill(0, 1);
          break;
        case KEY_LEFT:
        case 'a':
        case 'A':
          move_cursor_with_bresenham_fill(-1, 0);
          break;
        case KEY_RIGHT:
        case 'd':
        case 'D':
          move_cursor_with_bresenham_fill(1, 0);
          break;
        case '\n':
        case '\r':
        case 'g':
        case 'G':
          switch_from_draw_to_play();
          break;
        case 'c':
        case 'C':
          reset_draw_state();
          break;
        case 'o':
        case 'O':
          g_close_path_enabled = !g_close_path_enabled;
          break;
        case 't':
          g_active_theme_index = (g_active_theme_index + 1) % N_THEMES;
          apply_theme(g_active_theme_index);
          break;
        case 'T':
          g_active_theme_index =
              (g_active_theme_index + N_THEMES - 1) % N_THEMES;
          apply_theme(g_active_theme_index);
          break;
        }
      } else { /* STATE_PLAY */
        switch (ch) {
        case 'q':
        case 'Q':
        case 27:
          g_should_quit = 1;
          break;
        case 'r':
        case 'R':
          g_app_state = STATE_DRAW;
          g_draw_error_too_few = false;
          break;
        case ' ':
        case 'p':
        case 'P':
          g_simulation_paused = !g_simulation_paused;
          break;
        case '+':
        case '=':
          if (g_active_epicycle_count < g_total_epicycle_count) {
            g_active_epicycle_count++;
            trail_clear(&g_tip_trail);
          }
          break;
        case '-':
          if (g_active_epicycle_count > 1) {
            g_active_epicycle_count--;
            trail_clear(&g_tip_trail);
          }
          break;
        case 'c':
        case 'C':
          g_show_orbit_circles = !g_show_orbit_circles;
          break;
        case 'o':
        case 'O':
          handle_play_mode_close_toggle();
          break;
        case 'd':
          g_show_ghost_overlay = !g_show_ghost_overlay;
          break;
        case 'D':
          g_show_arm_table = !g_show_arm_table;
          break;
        case 'a':
        case 'A':
          g_auto_add_enabled = !g_auto_add_enabled;
          break;
        case 't':
          g_active_theme_index = (g_active_theme_index + 1) % N_THEMES;
          apply_theme(g_active_theme_index);
          break;
        case 'T':
          g_active_theme_index =
              (g_active_theme_index + N_THEMES - 1) % N_THEMES;
          apply_theme(g_active_theme_index);
          break;
        }
      }
    }

    /* tick */
    if (g_app_state == STATE_PLAY)
      play_simulation_tick();

    /* draw */
    erase();
    if (g_app_state == STATE_DRAW)
      render_draw_mode();
    else
      render_play_mode();
    wnoutrefresh(stdscr);
    doupdate();

    /* hold the frame rate */
    long long now_ns = clock_now_ns();
    clock_sleep_ns(RENDER_TICK_NS - (now_ns - last_frame_ns));
    last_frame_ns = clock_now_ns();
  }
  return 0;
}
