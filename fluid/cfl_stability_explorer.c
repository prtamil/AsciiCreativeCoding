/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cfl_stability_explorer.c — watch a 2-D wave simulation stay stable or blow up.
 *
 * A ripple-on-a-pond wave is stepped forward with the simplest scheme there is
 * (leapfrog finite differences).  One dial, the CFL number, decides everything:
 * at or below 1.0 the ripples behave; above 1.0 they grow without bound and the
 * screen fills with noise.  Press 1-9/0 to tour ten named regimes and watch the
 * meter cross the cliff in real time.  This stability limit is the rule behind
 * the original Courant-Friedrichs-Lewy 1928 paper.
 *
 * Sister files: physics/waves.c (same wave PDE, two solver engines),
 *               physics/rk_method_comparision.c (same stability story for ODEs).
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

/* §1  config — every tunable constant lives here, so the code below has no
 * unexplained magic numbers. */

enum {
  TARGET_FPS = 60,
  SIM_STEPS_PER_FRAME_MAX = 4, /* most sim steps we'll run per rendered frame */

  /* Biggest grid we'll ever need; the live size shrinks to fit the terminal. */
  GRID_ROWS_MAX = 80,
  GRID_COLS_MAX = 200,

  /* Rows reserved for the dashboard: a 4-row status block up top, a 1-row key
   * hint at the very bottom.  The wave field fills everything in between. */
  HUD_ROWS_TOP = 4,
  HUD_ROWS_BOT = 1,

  FPS_UPDATE_MS = 500,

  RAMP_LEVELS = 9,        /* brightness steps in the colour ramp, per sign */
  GROWTH_HISTORY_LEN = 4, /* how many recent peaks the growth detector keeps */
};

/* Wave speed c, in cells per second — the user nudges it with c/C. */
#define WAVE_SPEED_DEFAULT 20.0f
#define WAVE_SPEED_MIN 2.0f
#define WAVE_SPEED_MAX 200.0f

/* Time step dt, in seconds per tick — the user nudges it with +/-. */
#define STEP_SECONDS_DEFAULT 0.030f
#define STEP_SECONDS_MIN 0.0005f
#define STEP_SECONDS_MAX 0.5000f

#define EXPLOSION_THRESHOLD 1.0e6f /* peak this big means it's blown up; reset */
#define DROP_AMPLITUDE 1.0f        /* height of the water-drop bump */
#define DROP_RADIUS_CELLS 4.0f     /* how wide the drop spreads, in cells */
#define EIGENMODE_AMPLITUDE                                                    \
  0.05f /* small seed — it either grows or holds */

/*
 * cfl_preset — one named landmark on the stability scale, paired with the CFL
 *   value it parks at.  Pressing 1-9/0 walks these in order, from rock-solid
 *   stable through the cliff at CFL = 1.0 into runaway blow-up.
 *
 *   We store the CFL target rather than a dt because CFL is what actually
 *   decides stability: each preset then re-derives its own dt from the current
 *   wave speed, so pressing 6 (EDGE) always lands exactly at the cliff no
 *   matter what speed 'c' the user has dialed in.  The label rides along so the
 *   HUD can name the regime back at the user.  Scale follows CFL (Courant-
 *   Friedrichs-Lewy 1928).
 */
typedef struct {
    const char *name; /* short tag shown in the HUD, 8 chars or fewer */
    float       cfl;  /* the CFL value this preset aims for */
} cfl_preset;

#define N_PRESETS 10

static const cfl_preset cfl_presets[N_PRESETS] = {
    /*  1  */ { "GLASSY",  0.20f }, /* very low CFL, heavy num. dissipation   */
    /*  2  */ { "QUIET",   0.40f }, /* clean ripples that gently decay        */
    /*  3  */ { "WAVES",   0.60f }, /* comfortable working point              */
    /*  4  */ { "RIPPLES", 0.80f }, /* close to optimum, minimal dissipation  */
    /*  5  */ { "MARGIN",  0.95f }, /* approaching the cliff, still stable    */
    /*  6  */ { "EDGE",    1.00f }, /* exact CFL boundary — neutral stability */
    /*  7  */ { "DRIFT",   1.05f }, /* mildly unstable, slow exponential grow */
    /*  8  */ { "TILT",    1.15f }, /* faster growth, visible explosion ~30s  */
    /*  9  */ { "EXPLODE", 1.30f }, /* rapid blow-up, ~10s to NaN             */
    /*  0  */ { "RUNAWAY", 1.50f }, /* extreme, screen saturates in seconds   */
};

/* The sqrt(2) that turns wave-speed times time-step into the CFL number.
 * It's sqrt(2) because we're in 2-D; it would be sqrt(3) in 3-D. */
#define CFL_DIM_FACTOR 1.41421356237309504880f

/* Colour-pair slots: a block for the wave ramp, then a few for the HUD. */
enum {
  PAIR_BACKGROUND = 1,
  PAIR_POS_BASE = 2,                           /* +0..+RAMP_LEVELS-1 */
  PAIR_NEG_BASE = PAIR_POS_BASE + RAMP_LEVELS, /* +0..+RAMP_LEVELS-1 */
  PAIR_HUD = PAIR_NEG_BASE + RAMP_LEVELS,
  PAIR_HINT,
  PAIR_STABLE, /* CFL meter zone colours */
  PAIR_MARGINAL,
  PAIR_UNSTABLE,
  PAIR_WARN, /* white-on-red flashing alert */
};

/* Colour ramps, one row per theme.  Each ramp goes from faint to bright across
 * the 9 levels; positive amplitudes use one ramp, negatives the other, so peaks
 * and troughs read as opposite colours.  Every colour stays bright enough to
 * show up against the default background — the very dark cube/gray codes look
 * black and were avoided on purpose. */
enum { N_THEMES = 6 };

static const int theme_pos256[N_THEMES][RAMP_LEVELS] = {
    /* "WAVE"    — bright red ramp,  red → yellow → white  */
    { 124, 160, 196, 202, 208, 214, 220, 226, 231 },
    /* "FIRE"    — orange-yellow heat,  orange → yellow → white */
    { 130, 166, 202, 208, 214, 220, 226, 230, 231 },
    /* "NEON"    — electric pink,  magenta → pink → white  */
    { 165, 201, 207, 213, 219, 225, 226, 230, 231 },
    /* "OCEAN"   — bright crests,  light blue → white      */
    { 117, 153, 189, 195, 225, 231, 231, 231, 231 },
    /* "PLASMA"  — high-energy red→yellow                  */
    { 196, 202, 208, 214, 220, 226, 230, 231, 231 },
    /* "AURORA"  — green crests,  green → yellow-green     */
    {  46,  82, 118, 154, 190, 226, 227, 228, 229 },
};

static const int theme_neg256[N_THEMES][RAMP_LEVELS] = {
    /* "WAVE"    — bright cool ramp,  blue → cyan → light  */
    {  27,  33,  39,  45,  51,  87, 123, 159, 195 },
    /* "FIRE"    — purple troughs,  deep purple → pink     */
    {  90,  91,  92, 128, 129, 165, 201, 207, 219 },
    /* "NEON"    — cyan troughs to balance the pink crests */
    {  33,  39,  45,  51,  87, 123, 159, 195, 231 },
    /* "OCEAN"   — deep navy troughs,  navy → cyan         */
    {  25,  27,  33,  39,  45,  51,  87, 123, 159 },
    /* "PLASMA"  — purple troughs                          */
    {  53,  54,  91, 128, 129, 165, 201, 207, 213 },
    /* "AURORA"  — magenta troughs balancing green crests  */
    {  90, 127, 164, 200, 207, 213, 219, 225, 231 },
};

static const char *theme_name[N_THEMES] = {
    "WAVE", "FIRE", "NEON", "OCEAN", "PLASMA", "AURORA"
};

/* §2  clock — a steady clock for frame timing.  CLOCK_MONOTONIC never jumps
 * backward when the system clock is adjusted, which a wall clock can. */

#define NS_PER_SEC 1000000000LL

static int64_t clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / NS_PER_SEC, ns % NS_PER_SEC};
  nanosleep(&ts, NULL);
}

/* §3  colors — peaks and troughs need to look different, so positive amplitudes
 * get one colour ramp and negatives get another, meeting at the neutral
 * background at zero.  The HUD uses bright yellow for status, bright cyan for
 * the key hints. */

static int wave_pair_for(float amplitude, int theme) {
  /* Pick a brightness level for this amplitude; sign picks the ramp. */
  float clamped = amplitude;
  if (clamped > 1.0f)
    clamped = 1.0f;
  if (clamped < -1.0f)
    clamped = -1.0f;

  int level = (int)(fabsf(clamped) * (RAMP_LEVELS - 0.001f));
  if (level >= RAMP_LEVELS)
    level = RAMP_LEVELS - 1;
  (void)theme; /* the theme was already baked into the colour pairs at init */
  return (clamped >= 0.0f) ? PAIR_POS_BASE + level : PAIR_NEG_BASE + level;
}

static void color_init(int theme) {
  start_color();
  use_default_colors();

  /* Load the chosen theme's positive and negative ramps into colour pairs. */
  for (int level = 0; level < RAMP_LEVELS; level++) {
    if (COLORS >= 256) {
      init_pair(PAIR_POS_BASE + level, theme_pos256[theme][level], -1);
      init_pair(PAIR_NEG_BASE + level, theme_neg256[theme][level], -1);
    } else {
      init_pair(PAIR_POS_BASE + level, COLOR_RED, -1);
      init_pair(PAIR_NEG_BASE + level, COLOR_BLUE, -1);
    }
  }

  /* HUD colours.  The -1 background means "leave whatever the user's terminal
   * uses".  Stable/marginal/unstable double as the meter's traffic-light zones. */
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);         /* bright yellow */
    init_pair(PAIR_HINT, 51, -1);         /* bright cyan   */
    init_pair(PAIR_STABLE, 82, -1);       /* lime green    */
    init_pair(PAIR_MARGINAL, 220, -1);    /* amber         */
    init_pair(PAIR_UNSTABLE, 196, -1);    /* bright red    */
    init_pair(PAIR_WARN, 231, COLOR_RED); /* white on red */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
    init_pair(PAIR_STABLE, COLOR_GREEN, -1);
    init_pair(PAIR_MARGINAL, COLOR_YELLOW, -1);
    init_pair(PAIR_UNSTABLE, COLOR_RED, -1);
    init_pair(PAIR_WARN, COLOR_WHITE, COLOR_RED);
  }
}

/* §4  grid — the wave field, stored as three snapshots in time. */

/*
 * grid_state — holds the wave at three moments: last step, this step, and the
 *   next step we're computing.  We need all three because predicting where a
 *   point goes next depends on where it was AND where it just came from (a wave
 *   carries momentum).  This is the classic leapfrog scheme (Yee 1966).
 *
 *   The three snapshots are plain arrays; wave_past/now/next are just pointers
 *   that say which array is playing which role right now.  After each step we
 *   rotate the pointers instead of copying data around — the array that was
 *   "past" becomes free scratch for the next "next".  Cheap, and the only
 *   thing tracking time order is which pointer points where.
 *
 *   The arrays are sized for the largest grid we'll ever show and allocated
 *   once, up front (never in the hot loop).  When the terminal resizes we just
 *   use a smaller rows x cols corner and ignore the rest.  Cells are laid out
 *   row by row; reach them through grid_at / grid_set so the indexing math
 *   lives in one place.
 */
typedef struct {
    /* Active size — at most GRID_*_MAX, shrunk to fit the terminal. */
    int rows;
    int cols;

    /* Which array is past / now / next right now.  These rotate each step;
     * there is no separate "current buffer" flag — the pointers are it. */
    float *wave_past;
    float *wave_now;
    float *wave_next;

    /* The three storage arrays the pointers above always point into. */
    float buf_a[GRID_ROWS_MAX * GRID_COLS_MAX];
    float buf_b[GRID_ROWS_MAX * GRID_COLS_MAX];
    float buf_c[GRID_ROWS_MAX * GRID_COLS_MAX];
} grid_state;

static inline float grid_at(const float *buf, int cols, int r, int c) {
  return buf[r * cols + c];
}

static inline void grid_set(float *buf, int cols, int r, int c, float v) {
  buf[r * cols + c] = v;
}

static void grid_rotate_buffers(grid_state *g) {
  /* Slide every role back one step: next is now "now", now is "past", and the
   * old "past" is freed up to become next step's scratch. */
  float *was_past = g->wave_past;
  g->wave_past = g->wave_now;
  g->wave_now = g->wave_next;
  g->wave_next = was_past;
}

/* §5  init — three ways to seed the field: flat (for resets), a water drop
 * (a bell-shaped bump that ripples outward), or an eigenmode (a clean standing-
 * wave pattern that blows up neatly when unstable, ideal for measuring growth).
 * Each seed is written into BOTH past and now so the wave starts from rest. */

static void init_zero(grid_state *g) {
  memset(g->wave_past, 0, sizeof g->buf_a);
  memset(g->wave_now, 0, sizeof g->buf_a);
  memset(g->wave_next, 0, sizeof g->buf_a);
}

static void init_gaussian_drop(grid_state *g, float cy, float cx,
                               float amplitude, float radius_cells) {
  init_zero(g);
  float two_sigma_sq = 2.0f * radius_cells * radius_cells;
  for (int r = 0; r < g->rows; r++) {
    for (int c = 0; c < g->cols; c++) {
      float dy = (float)r - cy;
      float dx = (float)c - cx;
      float v = amplitude * expf(-(dy * dy + dx * dx) / two_sigma_sq);
      grid_set(g->wave_past, g->cols, r, c, v);
      grid_set(g->wave_now, g->cols, r, c, v);
    }
  }
}

static void init_eigenmode(grid_state *g, int mode_y, int mode_x,
                           float amplitude) {
  init_zero(g);
  /* A product of sines is a "pure note" of this grid — a single standing-wave
   * pattern the simulation keeps intact rather than scrambling.  That's why it
   * grows so cleanly (just gets louder, no speckle) once the sim goes unstable. */
  for (int r = 0; r < g->rows; r++) {
    for (int c = 0; c < g->cols; c++) {
      float ry = (float)(r + 1) / (float)(g->rows + 1);
      float rx = (float)(c + 1) / (float)(g->cols + 1);
      float v = amplitude * sinf((float)M_PI * mode_y * ry) *
                sinf((float)M_PI * mode_x * rx);
      grid_set(g->wave_past, g->cols, r, c, v);
      grid_set(g->wave_now, g->cols, r, c, v);
    }
  }
}

/* §6  laplacian — how curved the field is at one cell: how far this cell sits
 * above or below the average of its four neighbours.  That curvature is what
 * pulls the wave back and makes it oscillate.  The caller stays one cell in
 * from every edge, so we never read off the grid. */

static inline float laplacian_5pt(const float *u, int cols, int r, int c) {
  return u[(r - 1) * cols + c]     /* north */
         + u[(r + 1) * cols + c]   /* south */
         + u[r * cols + (c - 1)]   /* west  */
         + u[r * cols + (c + 1)]   /* east  */
         - 4.0f * u[r * cols + c]; /* centre × 4 (subtracted) */
}

/* §7  step — advance the wave one tick.  Each cell's next value is its current
 * value carried forward by its motion since last step, plus a nudge from the
 * local curvature (§6).  beta = wave-speed times time-step is how far the wave
 * travels per tick; squaring it is just how the math shakes out.  The edges are
 * pinned to zero, so waves bounce off the walls (and flip sign) like a string
 * tied down at both ends. */

static void wave_step(grid_state *g, float wave_speed_cps, float step_seconds) {
  float beta = wave_speed_cps * step_seconds;
  float beta_squared = beta * beta;
  int cols = g->cols;

  /* Fill in every interior cell first; we must finish reading the old field
   * before any of it is overwritten, so the whole grid is computed into
   * wave_next, then the roles rotate at the end. */
  for (int r = 1; r < g->rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      float lap = laplacian_5pt(g->wave_now, cols, r, c);
      float u_now = g->wave_now[r * cols + c];
      float u_past = g->wave_past[r * cols + c];
      float u_next = 2.0f * u_now - u_past + beta_squared * lap;
      g->wave_next[r * cols + c] = u_next;
    }
  }

  /* Pin the four edges to zero — the walls the wave reflects off. */
  for (int c = 0; c < cols; c++) {
    g->wave_next[0 * cols + c] = 0.0f;
    g->wave_next[(g->rows - 1) * cols + c] = 0.0f;
  }
  for (int r = 0; r < g->rows; r++) {
    g->wave_next[r * cols + 0] = 0.0f;
    g->wave_next[r * cols + (cols - 1)] = 0.0f;
  }

  grid_rotate_buffers(g);
}

/* §8  cfl — the numbers the dashboard reports.  cfl_compute gives the single
 * stability dial (1.0 is the cliff).  dt_critical is the biggest time step that
 * still keeps you under the cliff at the current wave speed.  The theoretical
 * growth tells you, when unstable, how much louder the wave gets each tick — a
 * prediction we can check against what's actually measured (§9). */

static float cfl_compute(float wave_speed_cps, float step_seconds) {
  return wave_speed_cps * step_seconds * CFL_DIM_FACTOR;
}

static float dt_critical(float wave_speed_cps) {
  return 1.0f / (wave_speed_cps * CFL_DIM_FACTOR);
}

static float growth_per_step_theoretical(float cfl_number) {
  /* Below the cliff nothing grows; above it, this is how fast it does. */
  if (cfl_number <= 1.0f)
    return 1.0f;
  return cfl_number + sqrtf(cfl_number * cfl_number - 1.0f);
}

static float time_to_double(float growth_per_step, float step_seconds) {
  /* Roughly how many seconds for the wave's peak to double in height. */
  if (growth_per_step <= 1.0001f)
    return INFINITY;
  return logf(2.0f) / logf(growth_per_step) * step_seconds;
}

/* §9  measure — watch for blow-up and report the live growth rate. */

/*
 * measure_state — tracks how the wave's loudest point is trending so the HUD
 *   can show a steady "growth per step" and we can catch a runaway.
 *
 *   A single before/after comparison is too jumpy to trust: a healthy wave
 *   sloshes up and down, so two back-to-back peaks can differ a lot for no bad
 *   reason.  So we keep the last few peak readings in a small ring of slots and
 *   measure the trend across the whole window — smooth enough to read, quick
 *   enough to catch trouble early.
 *
 *   The HUD shows this MEASURED growth right next to the THEORETICAL one (§8);
 *   when they match, especially with the eigenmode seed, you're watching the
 *   stability theory confirm itself live.
 *
 *   explosion_count survives a blow-up reset on purpose, so the dashboard can
 *   say "blown up N times" while you hunt for the cliff across presets.
 */
typedef struct {
    /* The recent peaks, kept in a small ring buffer. */
    float history[GROWTH_HISTORY_LEN]; /* last few max|u| readings */
    int   write_index;                 /* next slot to overwrite */
    int   filled;                      /* how many slots are real yet */

    /* Latest readings the HUD shows directly. */
    float current_max_abs;             /* loudest point right now */
    float empirical_growth_per_step;   /* measured trend across the window */

    /* Counters. */
    long tick_count;                   /* ticks since start, never reset */
    int  explosion_count;              /* blow-ups so far; survives resets */
} measure_state;

static void measure_init(measure_state *m) { memset(m, 0, sizeof *m); }

static float scan_max_abs(const float *u, int rows, int cols) {
  float best = 0.0f;
  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      float v = fabsf(u[r * cols + c]);
      if (v > best)
        best = v;
    }
  }
  return best;
}

static bool measure_update(measure_state *m, const grid_state *g) {
  /* Find the loudest point and remember it. */
  float now_max = scan_max_abs(g->wave_now, g->rows, g->cols);
  m->current_max_abs = now_max;

  /* Drop it into the ring of recent peaks. */
  m->history[m->write_index] = now_max;
  m->write_index = (m->write_index + 1) % GROWTH_HISTORY_LEN;
  if (m->filled < GROWTH_HISTORY_LEN)
    m->filled++;

  /* Per-step growth: how much the peak changed across the window, spread
   * evenly over the steps in it. */
  if (m->filled >= 2) {
    int oldest_index = m->write_index; /* oldest entry is the next to overwrite */
    float oldest = m->history[oldest_index];
    if (oldest > 1e-12f) {
      float ratio = now_max / oldest;
      float steps = (float)(m->filled - 1);
      m->empirical_growth_per_step = powf(ratio, 1.0f / steps);
    }
  }

  m->tick_count++;

  /* Blown up? Too loud, or gone non-finite. */
  if (now_max > EXPLOSION_THRESHOLD || isnan(now_max) || isinf(now_max)) {
    m->explosion_count++;
    return true; /* tell the caller to reset before NaNs spread */
  }
  return false;
}

/* §10  paint_field — draw the wave, one grid cell to one terminal cell.  Colour
 * carries the sign and strength of the amplitude; the glyph adds a second cue
 * (denser characters for bigger waves) so it still reads on poor-contrast
 * terminals. */

static char glyph_for_amp(float magnitude) {
  if (magnitude < 0.05f)
    return ' ';
  if (magnitude < 0.15f)
    return '.';
  if (magnitude < 0.30f)
    return ':';
  if (magnitude < 0.50f)
    return '+';
  if (magnitude < 0.75f)
    return '*';
  return '#';
}

static void paint_field(WINDOW *win, const grid_state *g, int theme) {
  /* Shift every row down by the top status block so the field starts below it. */
  for (int r = 0; r < g->rows; r++) {
    for (int c = 0; c < g->cols; c++) {
      float v = grid_at(g->wave_now, g->cols, r, c);
      float mag = fabsf(v);
      int pair = wave_pair_for(v, theme);
      char glyph = glyph_for_amp(mag);
      attr_t a = COLOR_PAIR(pair);
      if (mag > 0.50f)
        a |= A_BOLD;
      wattron(win, a);
      mvwaddch(win, HUD_ROWS_TOP + r, c, (chtype)(unsigned char)glyph);
      wattroff(win, a);
    }
  }
}

/* §11  paint_hud — the dashboard: a four-row status block up top (title, live
 * numbers, the CFL meter, running stats) and a key-hint row at the very bottom.
 * Bright colours, bold, never dim, so it stays readable over the animation. */

/*
 * hud_data — a flat snapshot of just the numbers the dashboard needs for one
 *   frame.  Building this once and handing it to each painter keeps the drawing
 *   code from having to reach into the simulator's internals; the painters never
 *   touch grid_state or measure_state directly.  It's built fresh each frame and
 *   thrown away — it's a copy, not live state.
 */
typedef struct {
    /* The headline numbers (params row + meter). */
    float cfl_number;             /* the stability dial; 1.0 is the cliff */
    float step_seconds;           /* dt, the time step */
    float dt_critical_now;        /* biggest dt that still stays under the cliff */
    float wave_speed_cps;         /* c, the wave speed */

    /* Growth side by side: what we measured vs what theory predicts.  When they
     * agree you're watching the stability theory check out. */
    float empirical_growth;
    float theoretical_growth;
    float doubling_time;          /* seconds for the peak to double */

    /* Running stats. */
    float max_amplitude;
    long  tick_count;
    int   explosion_count;

    /* What the user has selected (shown in title + stats rows). */
    int   theme;
    int   preset_idx;
    const char *preset_name;      /* the active regime's name */
    bool  paused;
} hud_data;

static int cfl_zone_pair(float cfl_number) {
  if (cfl_number < 0.70f)
    return PAIR_STABLE;
  if (cfl_number < 1.00f)
    return PAIR_MARGINAL;
  return PAIR_UNSTABLE;
}

static const char *cfl_zone_label(float cfl_number) {
  if (cfl_number < 0.70f)
    return "stable  ";
  if (cfl_number < 1.00f)
    return "marginal";
  return "UNSTABLE";
}

static void paint_hud_title(WINDOW *win, int row, int cols, bool paused) {
  wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int c = 0; c < cols; c++)
    mvwaddch(win, row, c, '-');
  const char *title = "[ CFL STABILITY EXPLORER ]";
  mvwprintw(win, row, (cols - (int)strlen(title)) / 2, "%s", title);
  if (paused) {
    wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
    wattron(win, COLOR_PAIR(PAIR_MARGINAL) | A_BOLD);
    mvwprintw(win, row, cols - 9, " PAUSED ");
    wattroff(win, COLOR_PAIR(PAIR_MARGINAL) | A_BOLD);
    wattron(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
  }
  wattroff(win, COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void paint_hud_params(WINDOW *win, int row, const hud_data *h) {
  int cp = cfl_zone_pair(h->cfl_number);
  wattron(win, COLOR_PAIR(PAIR_HUD));
  mvwprintw(win, row, 1, " CFL = ");
  wattroff(win, COLOR_PAIR(PAIR_HUD));

  wattron(win, COLOR_PAIR(cp) | A_BOLD);
  wprintw(win, "%6.4f  %-9s", h->cfl_number, cfl_zone_label(h->cfl_number));
  wattroff(win, COLOR_PAIR(cp) | A_BOLD);

  wattron(win, COLOR_PAIR(PAIR_HUD));
  wprintw(win, "  dt=%7.4fs  c=%5.1fc/s  dt_crit=%7.4fs  ", h->step_seconds,
          h->wave_speed_cps, h->dt_critical_now);
  wattroff(win, COLOR_PAIR(PAIR_HUD));

  /* Measured growth next to the predicted one; with the eigenmode seed they
   * should line up almost exactly. */
  bool growing = h->empirical_growth > 1.0001f && isfinite(h->doubling_time);
  int pair = growing ? PAIR_UNSTABLE : PAIR_STABLE;
  wattron(win, COLOR_PAIR(pair) | A_BOLD);
  wprintw(win, "|mu|=%7.5f (theory %7.5f)",
          h->empirical_growth > 0.0f ? h->empirical_growth : 1.0f,
          h->theoretical_growth);
  wattroff(win, COLOR_PAIR(pair) | A_BOLD);
  if (growing) {
    wattron(win, COLOR_PAIR(PAIR_UNSTABLE) | A_BOLD);
    wprintw(win, "  t*2=%5.2fs", h->doubling_time);
    wattroff(win, COLOR_PAIR(PAIR_UNSTABLE) | A_BOLD);
  }
}

static void paint_hud_stats(WINDOW *win, int row, const hud_data *h) {
  wattron(win, COLOR_PAIR(PAIR_HUD));
  mvwprintw(win, row, 1,
            " preset=%-7s  max|u|=%9.4g  ticks=%8ld  exploded=%3d  theme=%-6s",
            h->preset_name, (double)h->max_amplitude, h->tick_count,
            h->explosion_count, theme_name[h->theme]);
  wattroff(win, COLOR_PAIR(PAIR_HUD));
}

static void paint_hud_hints(WINDOW *win, int row) {
  wattron(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvwprintw(win, row, 0,
            " q:quit  spc:pause  s:step  1-9,0:CFL preset  +/-:dt  "
            "c/C:speed  m:mode  d:drop  r:reset  t:theme ");
  wattroff(win, COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §12  paint_meter — a traffic-light bar from CFL 0 to 1.5.  Green up to 0.7,
 * amber up to 1.0, red beyond.  A '|' marks the cliff at 1.0 and a '#' marks
 * where you are now; when the '#' crosses the '|' the wave starts growing
 * instead of just sloshing. */

#define CFL_METER_MAX 1.5f

static void paint_cfl_meter(WINDOW *win, int row, int width, float cfl_number) {
  int bar_inner = width - 2; /* leave room for a bracket each side */
  if (bar_inner < 10)
    return;

  /* Where the zone edges and the current value land along the bar. */
  int idx_07 = (int)(0.70f / CFL_METER_MAX * bar_inner);
  int idx_10 = (int)(1.00f / CFL_METER_MAX * bar_inner);
  int idx_now = (int)(cfl_number / CFL_METER_MAX * bar_inner);
  if (idx_now < 0)
    idx_now = 0;
  if (idx_now >= bar_inner)
    idx_now = bar_inner - 1;

  wattron(win, COLOR_PAIR(PAIR_HUD));
  mvwaddch(win, row, 0, '[');

  for (int i = 0; i < bar_inner; i++) {
    int pair;
    if (i < idx_07)
      pair = PAIR_STABLE;
    else if (i < idx_10)
      pair = PAIR_MARGINAL;
    else
      pair = PAIR_UNSTABLE;
    char glyph = '=';
    if (i == idx_10)
      glyph = '|'; /* the cliff at CFL 1.0 */
    if (i == idx_now)
      glyph = '#'; /* where we are now */
    attr_t a = COLOR_PAIR(pair);
    if (i == idx_now)
      a |= A_BOLD;
    wattron(win, a);
    mvwaddch(win, row, 1 + i, (chtype)(unsigned char)glyph);
    wattroff(win, a);
  }

  wattron(win, COLOR_PAIR(PAIR_HUD));
  mvwaddch(win, row, 1 + bar_inner, ']');
  wattroff(win, COLOR_PAIR(PAIR_HUD));
}

/* §13  scene — the glue that owns the simulation and measurement state and ties
 * the wave (which knows nothing about terminals) to the dashboard (which knows
 * nothing about wave physics).  scene_tick advances and measures; scene_draw
 * paints; the smaller helpers handle resets and key actions. */

/*
 * init_kind — which pattern to drop in when the field resets.  DROP is a water-
 *   drop bump: pretty, lively ripples, but a messy mix of wavelengths so the
 *   growth number is hard to read.  EIGENMODE is a single clean standing-wave
 *   pattern: less flashy, but when unstable it grows at one steady rate that
 *   matches the predicted growth almost exactly — the good one for measuring.
 */
typedef enum { INIT_DROP, INIT_EIGENMODE } init_kind;

/*
 * scene_state — the one place that holds everything this demo runs on.  Fields
 *   are grouped by who reads them: the simulation, the renderer, or both.  That
 *   grouping isn't cosmetic — keeping a display-only choice like the colour
 *   theme out of the physics is what guarantees the same CFL always produces the
 *   same wave, no matter what colours are on screen.
 *
 *   Things that deliberately live elsewhere: frame timing (local to main), the
 *   signal flags (file-scope), and the theme/preset/glyph tables (file-scope
 *   constants).
 */
typedef struct {
    /* The simulation: the field itself plus its running stats.  Both reset
     * together on 'r'. */
    grid_state    grid;
    measure_state measure;

    /* Shared by sim and dashboard, adjusted live by the user. */
    float wave_speed_cps;          /* c — wave speed (c/C keys) */
    float step_seconds;            /* dt — time step (+/- keys) */

    /* What the next reset produces, and whether the clock is running. */
    int       preset_idx;          /* which named regime is active */
    init_kind last_init;           /* drop or eigenmode, for the next reset */
    bool      paused;
    bool      step_once;           /* take exactly one tick, then stay paused ('s') */

    /* Display only — changing this never touches the physics. */
    int theme;
} scene_state;

static void scene_apply_init(scene_state *s) {
  if (s->last_init == INIT_EIGENMODE) {
    init_eigenmode(&s->grid, 1, 1, EIGENMODE_AMPLITUDE);
  } else {
    init_gaussian_drop(&s->grid, (float)(s->grid.rows - 1) * 0.5f,
                       (float)(s->grid.cols - 1) * 0.5f, DROP_AMPLITUDE,
                       DROP_RADIUS_CELLS);
  }
  measure_init(&s->measure);
}

static void scene_resize_to_terminal(scene_state *s) {
  int term_rows, term_cols;
  getmaxyx(stdscr, term_rows, term_cols);

  int field_rows = term_rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  int field_cols = term_cols;
  if (field_rows < 4)
    field_rows = 4;
  if (field_cols < 8)
    field_cols = 8;
  if (field_rows > GRID_ROWS_MAX)
    field_rows = GRID_ROWS_MAX;
  if (field_cols > GRID_COLS_MAX)
    field_cols = GRID_COLS_MAX;

  s->grid.rows = field_rows;
  s->grid.cols = field_cols;
  s->grid.wave_past = s->grid.buf_a;
  s->grid.wave_now = s->grid.buf_b;
  s->grid.wave_next = s->grid.buf_c;
}

static void scene_init(scene_state *s) {
  memset(s, 0, sizeof *s);
  s->wave_speed_cps = WAVE_SPEED_DEFAULT;
  s->step_seconds = STEP_SECONDS_DEFAULT;
  s->theme = 0;
  s->last_init = INIT_DROP;
  s->paused = false;

  scene_resize_to_terminal(s);
  scene_apply_init(s);
}

static void scene_set_cfl_preset(scene_state *s, float target_cfl) {
  /* Work backwards from the wanted CFL to the time step that produces it. */
  s->step_seconds = target_cfl / (s->wave_speed_cps * CFL_DIM_FACTOR);
  if (s->step_seconds < STEP_SECONDS_MIN)
    s->step_seconds = STEP_SECONDS_MIN;
  if (s->step_seconds > STEP_SECONDS_MAX)
    s->step_seconds = STEP_SECONDS_MAX;
}

static void scene_tick(scene_state *s) {
  if (s->paused && !s->step_once)
    return;
  s->step_once = false;

  wave_step(&s->grid, s->wave_speed_cps, s->step_seconds);

  bool exploded = measure_update(&s->measure, &s->grid);
  if (exploded) {
    scene_apply_init(s);
    /* Re-seeding wipes the stats, so keep the blow-up tally across it. */
    int saved_count = s->measure.explosion_count;
    measure_init(&s->measure);
    s->measure.explosion_count = saved_count;
  }
}

static void scene_draw(scene_state *s, WINDOW *win) {
  werase(win);
  paint_field(win, &s->grid, s->theme);

  int term_rows, term_cols;
  getmaxyx(win, term_rows, term_cols);

  float cfl_n = cfl_compute(s->wave_speed_cps, s->step_seconds);
  hud_data h = {
      .cfl_number = cfl_n,
      .step_seconds = s->step_seconds,
      .dt_critical_now = dt_critical(s->wave_speed_cps),
      .wave_speed_cps = s->wave_speed_cps,
      .empirical_growth = s->measure.empirical_growth_per_step,
      .theoretical_growth = growth_per_step_theoretical(cfl_n),
      .doubling_time =
          time_to_double(s->measure.empirical_growth_per_step, s->step_seconds),
      .max_amplitude = s->measure.current_max_abs,
      .tick_count = s->measure.tick_count,
      .explosion_count = s->measure.explosion_count,
      .theme = s->theme,
      .preset_idx = s->preset_idx,
      .preset_name = cfl_presets[s->preset_idx].name,
      .paused = s->paused,
  };

  /* Status block on top, key hints on the last row. */
  paint_hud_title (win, 0, term_cols, h.paused);
  paint_hud_params(win, 1, &h);
  paint_cfl_meter (win, 2, term_cols, h.cfl_number);
  paint_hud_stats (win, 3, &h);
  paint_hud_hints (win, term_rows - 1);

  wnoutrefresh(win);
  doupdate();
}

/* §14  input — turn a keypress into a scene action.  Returns false to quit. */

static bool scene_handle_key(scene_state *s, int key) {
  switch (key) {
  case 'q':
  case 'Q':
  case 27:        /* ESC */
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 's':
    if (s->paused)
      s->step_once = true;
    break;

  /* '1'..'9' are presets 1..9; '0' is the tenth. */
  case '1': case '2': case '3': case '4': case '5':
  case '6': case '7': case '8': case '9':
  case '0': {
    int idx = (key == '0') ? 9 : (key - '1');
    s->preset_idx = idx;
    scene_set_cfl_preset(s, cfl_presets[idx].cfl);
    break;
  }

  case '+':
  case '=':
    s->step_seconds *= 1.05f;
    if (s->step_seconds > STEP_SECONDS_MAX)
      s->step_seconds = STEP_SECONDS_MAX;
    break;
  case '-':
  case '_':
    s->step_seconds /= 1.05f;
    if (s->step_seconds < STEP_SECONDS_MIN)
      s->step_seconds = STEP_SECONDS_MIN;
    break;

  case 'c':
    s->wave_speed_cps *= 0.90f;
    if (s->wave_speed_cps < WAVE_SPEED_MIN)
      s->wave_speed_cps = WAVE_SPEED_MIN;
    break;
  case 'C':
    s->wave_speed_cps *= 1.10f;
    if (s->wave_speed_cps > WAVE_SPEED_MAX)
      s->wave_speed_cps = WAVE_SPEED_MAX;
    break;

  case 'm':
    s->last_init = INIT_EIGENMODE;
    scene_apply_init(s);
    break;
  case 'd':
    s->last_init = INIT_DROP;
    scene_apply_init(s);
    break;

  case 'r':
    measure_init(&s->measure);
    scene_apply_init(s);
    break;

  case 't':
    s->theme = (s->theme + 1) % N_THEMES;
    color_init(s->theme);
    break;

  default:
    break;
  }
  return true;
}

/* §15  screen — start up and tear down ncurses. */

static void screen_init(int theme) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, true);
  nodelay(stdscr, true);
  curs_set(0);
  typeahead(-1);
  color_init(theme);
}

static void screen_cleanup(void) { endwin(); }

/* §16  app — the main loop and signal handling. */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

int main(void) {
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  static scene_state scene;
  screen_init(0);
  atexit(screen_cleanup);
  scene_init(&scene);

  const int64_t frame_ns = NS_PER_SEC / TARGET_FPS;

  while (!g_should_quit) {
    int64_t frame_start = clock_now_ns();

    /* Window changed size — rebuild ncurses and re-fit the grid. */
    if (g_resize_pending) {
      g_resize_pending = 0;
      endwin();
      refresh();
      scene_resize_to_terminal(&scene);
      scene_apply_init(&scene);
    }

    /* Handle every key waiting in the queue. */
    int key;
    while ((key = getch()) != ERR) {
      if (!scene_handle_key(&scene, key)) {
        g_should_quit = 1;
        break;
      }
    }

    /* Step the wave a few times per frame, but cap it so a slow frame can't
     * snowball into an ever-growing backlog of catch-up steps. */
    for (int n = 0; n < SIM_STEPS_PER_FRAME_MAX; n++) {
      scene_tick(&scene);
      if (scene.paused)
        break;
    }

    scene_draw(&scene, stdscr);

    int64_t spent = clock_now_ns() - frame_start;
    if (spent < frame_ns)
      clock_sleep_ns(frame_ns - spent);
  }

  return 0;
}
