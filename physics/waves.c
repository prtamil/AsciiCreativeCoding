/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * waves.c — ripples and interference patterns on the terminal grid.
 *
 * The same physics, two ways of computing it. Press `m` to switch.
 *   FDTD     — step a grid of "water height" cells forward in time; ripples
 *              spread, bounce off the walls, and fade. This is the honest
 *              numerical simulation.
 *   ANALYTIC — skip the simulation and just add up the sine waves coming
 *              from each source. No grid, instant, always stable.
 * Both read the same list of wave sources, so the picture matches. Presets
 * 1-5 arrange the sources into famous setups (double-slit, ripple tank, etc.).
 *
 * References the code can't give you:
 *   FDTD grid method        — Yee 1966; Taflove, "Computational Electrodynamics"
 *   wave physics / optics    — Crawford "Waves"; Hecht "Optics"; Born & Wolf
 *   ASCII intensity ramp     — Bourke, paulbourke.net/dataformats/asciiart
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

/* ── §1 config ── */
/* Every knob lives here so there are no mystery numbers further down. */

/* A terminal cell is taller than it is wide. These are its pixel size, used
 * so ripples come out round instead of squashed in the analytic engine. */
#define CELL_W 8
#define CELL_H 16

/* Biggest field we'll ever allocate (the real size is clamped to the window). */
#define GRID_W_MAX 300
#define GRID_H_MAX 100

/* How many rows the HUD steals: two at the top, one at the bottom. */
#define HUD_ROWS_TOP 2
#define HUD_ROWS_BOT 1

/* ── FDTD engine ── */
#define C_SPEED 0.45f /* wave speed in cells/tick; staying under 0.707 keeps it stable */
#define C_SQ (C_SPEED * C_SPEED)
#define DAMPING_DEF 0.993f
#define DAMPING_STEP 0.002f
#define DAMPING_MIN 0.960f
#define DAMPING_MAX 0.999f
#define FDTD_STEPS_DEF 4 /* tiny physics steps run per drawn frame */
#define FDTD_STEPS_MIN 1
#define FDTD_STEPS_MAX 16
#define SOURCE_AMP 3.0f
#define IMPULSE_AMP 6.0f
#define IMPULSE_RADIUS 3

/* ── Analytic engine ── */
#define OMEGA_DEF 0.15f /* how fast a source cycles, in radians per frame */
#define OMEGA_STEP 0.01f
#define LAMBDA_DEF 20.0f /* wavelength, measured in columns */
#define LAMBDA_STEP 2.0f
#define LAMBDA_MIN 6.0f
#define LAMBDA_MAX 60.0f

/* ── Shared rendering ── */
#define MAX_AMP 4.0f    /* what counts as "full height" when picking a glyph */
#define ZERO_BAND 0.06f /* heights this close to flat are left blank, so still water (nodal lines) shows */
#define N_LEVELS 4      /* glyph steps per side (crest or trough) */
#define N_FIELD_CP 8    /* 4 trough colours + 4 crest colours */
#define N_THEMES 10
#define N_SRC_MAX 8

/* ── Colour pair slots ── */
enum {
  CP_FIELD0 = 1, /* the 8 field colours: 0-3 troughs, 4-7 crests */
  CP_FIELD7 = CP_FIELD0 + N_FIELD_CP - 1,
  CP_HUD,     /* HUD text (status + params) */
  CP_HINT,    /* bottom key hint */
  CP_SRC_SEL, /* the source you're currently editing */
  CP_SRC_OFF, /* every other source */
};

/* ── Timing ── */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_FPS 30
#define RENDER_NS (NS_PER_SEC / RENDER_FPS)

/* ── §2 clock ── */
/* Read the time and sleep. Only the main loop uses these, for frame timing. */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 color ── */
/* The 10 named colour schemes, the glyph-and-colour ramp for a wave height,
 * and theme switching. Themes only recolour the water; the HUD stays a fixed
 * bright yellow/cyan so it's always readable. */

/*
 * Theme — one of the 10 named colour schemes (MATRIX, FIRE, ...).
 *
 * Each theme colours the 8 height levels: the four trough colours (the wave
 * dipping below flat) and the four crest colours (rising above it). It does
 * NOT touch the HUD colours — those are kept a loud yellow/cyan on purpose so
 * the readout stays legible over any animation.
 *
 * Two colour tracks because not every terminal is the same:
 *   - fg256 is the nice 256-colour version (one shade per level).
 *   - fg8_neg / fg8_pos are the fallback for old 8-colour terminals, which
 *     can't show four shades — so all troughs share one colour, all crests
 *     another, and the glyph shape carries the intensity instead.
 *
 * Every fg256 value is kept fairly bright (>= 24); the darkest cube/gray
 * colours would just read as black on a default terminal background.
 *
 * The glyph ramp ", . - ~ + * # @" that pairs with these colours follows
 * Bourke's classic ASCII-grayscale design.
 */
typedef struct {
  const char *name;        /* short label shown in the HUD */
  short fg256[N_FIELD_CP]; /* 256-colour shades: [0..3] troughs, [4..7] crests */
  short fg8_neg;           /* 8-colour fallback: the one trough colour */
  short fg8_pos;           /* 8-colour fallback: the one crest colour */
} Theme;

static const Theme k_themes[N_THEMES] = {
    {"MATRIX", {28, 34, 40, 46, 82, 118, 154, 231}, COLOR_GREEN, COLOR_GREEN},
    {"FIRE", {52, 88, 124, 160, 208, 214, 220, 231}, COLOR_BLUE, COLOR_YELLOW},
    {"OCEANIC", {24, 25, 31, 39, 45, 51, 159, 231}, COLOR_BLUE, COLOR_CYAN},
    {"NEON",
     {53, 56, 93, 129, 165, 200, 207, 231},
     COLOR_MAGENTA,
     COLOR_MAGENTA},
    {"MONO",
     {240, 241, 242, 243, 250, 252, 254, 231},
     COLOR_WHITE,
     COLOR_WHITE},
    {"ICE", {25, 31, 32, 39, 44, 51, 195, 231}, COLOR_BLUE, COLOR_CYAN},
    {"NOVA",
     {54, 56, 92, 128, 164, 200, 213, 231},
     COLOR_MAGENTA,
     COLOR_MAGENTA},
    {"FOREST", {28, 34, 40, 46, 100, 142, 184, 231}, COLOR_GREEN, COLOR_YELLOW},
    {"DESERT", {52, 94, 130, 166, 208, 214, 222, 231}, COLOR_RED, COLOR_YELLOW},
    {"ECLIPSE",
     {53, 89, 125, 161, 197, 213, 219, 231},
     COLOR_MAGENTA,
     COLOR_WHITE},
};

/* The eight glyphs, faint to bold. The first four are for dips below flat,
 * the last four for peaks above it. */
static const char k_field_chars[N_FIELD_CP] = {',', '.', '-', '~',
                                               '+', '*', '#', '@'};

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  bool full = (COLORS >= 256);
  for (int i = 0; i < N_FIELD_CP; i++) {
    short fg = full ? t->fg256[i] : (i < N_LEVELS ? t->fg8_neg : t->fg8_pos);
    init_pair((short)(CP_FIELD0 + i), fg, -1);
  }
  init_pair(CP_HUD, full ? 226 : COLOR_YELLOW, -1); /* bright yellow */
  init_pair(CP_HINT, full ? 51 : COLOR_CYAN, -1);   /* bright cyan   */
  init_pair(CP_SRC_SEL, full ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_SRC_OFF, full ? 244 : COLOR_WHITE, -1);
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);
}

/* Turn a height (between -1 and 1) into which glyph to draw. Almost-flat water
 * returns -1, meaning "leave it blank" — that's how the calm still lines
 * between ripples show up. Below flat picks a trough glyph, above flat picks a
 * crest glyph, brighter the further from flat. */
static int amplitude_level(float u_norm) {
  if (u_norm > 1.f)
    u_norm = 1.f;
  if (u_norm < -1.f)
    u_norm = -1.f;
  float au = u_norm < 0.f ? -u_norm : u_norm;
  if (au <= ZERO_BAND)
    return -1;
  float t = (au - ZERO_BAND) / (1.f - ZERO_BAND);
  int lv = (int)(t * (float)N_LEVELS);
  if (lv >= N_LEVELS)
    lv = N_LEVELS - 1;
  return (u_norm < 0.f) ? lv : (N_LEVELS + lv);
}

static attr_t level_attr(int lv) {
  attr_t a = COLOR_PAIR(CP_FIELD0 + lv);
  if (lv == N_FIELD_CP - 1)
    a |= A_BOLD; /* brightest crest glows */
  return a;
}

/* ── §4 fdtd-grid ── */
/* The honest simulation: a grid of water-height cells we step forward in
 * time. This section only knows about the grid itself — not about wave
 * sources, colours, or drawing. grid_tick does one step by calling four
 * small helpers in order. */

/*
 * Grid — the water surface for the simulation, kept as three snapshots in time.
 *
 * To work out the next instant of the water, you need to know not just where
 * the surface is now, but where it was a moment ago — that's how you tell
 * which way each point is moving. So we keep three full copies: the surface a
 * moment ago, the surface now, and the new surface we're writing.
 *
 * When a step finishes we don't copy the data around — we just relabel the
 * three buffers (a moment ago ← now ← new), which is instant no matter how big
 * the grid is. The classic grid method for waves (Yee 1966; Taflove §3.6).
 *
 * Fields:
 *   prev, cur, next — the three snapshots (a moment ago / now / being written).
 *                     Each is a flat cols×rows array, stored row by row.
 *   cols, rows      — grid size in cells.
 *   damping         — how fast ripples fade, a touch below 1.0. Each step
 *                     multiplies the surface by this, so energy slowly bleeds
 *                     away instead of bouncing around the box forever. The
 *                     user nudges it with d/D in FDTD mode; it means nothing
 *                     in analytic mode, which is why it lives on the grid and
 *                     not on the scene.
 *
 * The three buffers are allocated once at startup (and again only on a window
 * resize). Nothing here allocates while running.
 */
typedef struct {
  float *prev; /* the surface a moment ago */
  float *cur;  /* the surface right now */
  float *next; /* the new surface being written this step */

  int cols;
  int rows;

  float damping; /* per-step fade factor, just under 1.0 */
} Grid;

static void grid_alloc(Grid *g, int cols, int rows) {
  size_t n = (size_t)cols * (size_t)rows;
  g->cols = cols;
  g->rows = rows;
  g->prev = calloc(n, sizeof(float));
  g->cur = calloc(n, sizeof(float));
  g->next = calloc(n, sizeof(float));
}

static void grid_free(Grid *g) {
  free(g->prev);
  free(g->cur);
  free(g->next);
  memset(g, 0, sizeof *g);
}

static void grid_clear(Grid *g) {
  size_t n = (size_t)g->cols * (size_t)g->rows * sizeof(float);
  memset(g->prev, 0, n);
  memset(g->cur, 0, n);
  memset(g->next, 0, n);
}

static void grid_resize(Grid *g, int cols, int rows) {
  float d = g->damping;
  grid_free(g);
  grid_alloc(g, cols, rows);
  g->damping = d;
}

/* How curved the surface is at one cell: compare it to its four neighbours.
 * If they're all higher, the cell is in a dip and will get pushed up; if all
 * lower, it'll get pushed down. This bulge-or-dip number is what makes ripples
 * spread. (Yee 1966; Taflove §3.6.) */
static inline float laplacian_4point_cross(const float *u, int i, int north_i,
                                           int south_i) {
  return u[i + 1] + u[i - 1] + u[north_i] + u[south_i] - 4.f * u[i];
}

/* Work out where this cell will be a moment from now. Take where it is now,
 * add how it's been moving (now minus a moment ago), and bend it by the
 * bulge-or-dip number so ripples spread. This is the standard wave-equation
 * step (Yee 1966; LeVeque §1.4). */
static inline float wave_central_difference_step(float u_cur, float u_prev,
                                                 float laplacian, float csq) {
  return 2.f * u_cur - u_prev + csq * laplacian;
}

/* Step every inside cell forward one tick and fade it a touch. The edge cells
 * are left at zero, so they act like solid walls the waves bounce off; the
 * fading stops those bounces from echoing around the box forever. */
static void sweep_interior_with_central_diff(Grid *g) {
  const int cols = g->cols, rows = g->rows;
  const float csq = C_SQ;
  const float damp = g->damping;
  const float *cu = g->cur, *pv = g->prev;
  float *nx = g->next;

  for (int y = 1; y < rows - 1; y++) {
    int row = y * cols;
    int rowp = row + cols; /* row index of y+1 (south) */
    int rowm = row - cols; /* row index of y-1 (north) */
    for (int x = 1; x < cols - 1; x++) {
      int i = row + x;
      float lap = laplacian_4point_cross(cu, i, rowm + x, rowp + x);
      nx[i] = wave_central_difference_step(cu[i], pv[i], lap, csq) * damp;
    }
  }
}

/* Hand the three snapshots their new jobs: now becomes a-moment-ago, the
 * freshly-written surface becomes now, and the old a-moment-ago is recycled as
 * scratch for next time. Just swaps labels, so it's instant however big the
 * grid is — no copying. See the Grid note. */
static inline void rotate_time_level_buffers(Grid *g) {
  float *tmp = g->prev;
  g->prev = g->cur;
  g->cur = g->next;
  g->next = tmp;
}

/* One step of the simulation: work out the next surface for every inside cell,
 * then shuffle the three snapshots along so "now" points at the new surface. */
static void grid_tick(Grid *g) {
  sweep_interior_with_central_diff(g);
  rotate_time_level_buffers(g);
}

/* Plop a soft round bump into the water at (cx, cy) — like dropping a pebble.
 * We poke the same bump into both "now" and "a moment ago" so the water starts
 * still and then falls, instead of being flung upward on the very first step. */
static void grid_impulse(Grid *g, int cx, int cy) {
  int R = IMPULSE_RADIUS;
  for (int dy = -R; dy <= R; dy++) {
    for (int dx = -R; dx <= R; dx++) {
      int x = cx + dx, y = cy + dy;
      if (x < 1 || x >= g->cols - 1 || y < 1 || y >= g->rows - 1)
        continue;
      float d2 = (float)(dx * dx + dy * dy);
      float w = expf(-d2 / (float)(R * R));
      int i = y * g->cols + x;
      g->cur[i] += IMPULSE_AMP * w;
      g->prev[i] += IMPULSE_AMP * w;
    }
  }
}

/* ── §5 sources ── */
/* A "source" is a point that makes waves, like a finger tapping the water.
 * Both engines read the same list of sources, so the picture matches. This
 * section has the Source struct, the list of sources, the helpers to
 * add/delete/move/pick one, and the 5 presets that arrange them into the
 * famous setups (double-slit, ripple tank, beat, radial, five-oscillator). */

/*
 * Source — one point that makes waves, like a finger dipping into the water
 * over and over at a steady beat. Both engines read the same source list, so
 * a preset looks the same whichever engine is drawing it. (The analytic engine
 * is just what the honest simulation settles into; Born & Wolf §8.6.)
 *
 * Position (x, y) is in grid cells inside the water area, not pixels and not
 * counted from the top-left of the terminal. The two engines use the position
 * differently: the simulation rounds it to a whole cell and taps that cell; the
 * analytic engine measures the real distance from the source to each cell
 * (stretched to match the cell's tall shape, so the rings come out round).
 *
 * omega is "how fast it cycles," measured per drawn frame — NOT per tiny
 * physics step. The simulation divides it down internally so a source completes
 * the same amount of cycle per frame no matter how many physics steps run. That
 * way the waves look the same speed when you switch engines or change steps.
 *
 * Two phase fields because they play different roles:
 *   phase_init  the source's fixed head-start, set by the preset (the radial
 *               preset spaces these out to make the rotating star). Both
 *               engines read it.
 *   phase_run   a running clock the simulation advances every step. It starts
 *               at phase_init. The analytic engine ignores it — it works the
 *               phase out straight from the source position and time.
 *
 * lambda (wavelength, in columns) is only used by the analytic engine. The
 * simulation's wavelength is decided for it by omega and the wave speed, so
 * changing lambda in simulation mode does nothing visible — that's why +/- mean
 * different things in the two engines (see app_handle_key).
 */
typedef struct {
  /* Where the source sits, in grid cells inside the water area. */
  float x; /* column, 0 .. gw */
  float y; /* row,    0 .. gh */

  /* How fast and how wide the waves are. */
  float omega;  /* cycle speed, per drawn frame (both engines)        */
  float lambda; /* wavelength in columns       (analytic engine only) */

  /* Phase = where in its cycle the source is. */
  float phase_init; /* fixed head-start from the preset             */
  float phase_run;  /* running clock the simulation advances (sim)  */

  bool active; /* is this source currently switched on? */
} Source;

static Source g_src[N_SRC_MAX];
static int g_nactive = 0;
static int g_selected = 0;

static void source_clear_all(void) {
  memset(g_src, 0, sizeof g_src);
  g_nactive = 0;
  g_selected = 0;
}

static int source_add(float x, float y, float omega, float lambda,
                      float phase0) {
  for (int s = 0; s < N_SRC_MAX; s++) {
    if (!g_src[s].active) {
      g_src[s] = (Source){x, y, omega, lambda, phase0, phase0, true};
      g_nactive++;
      g_selected = s;
      return s;
    }
  }
  return -1;
}

static void source_delete(int s) {
  if (s < 0 || s >= N_SRC_MAX || !g_src[s].active)
    return;
  g_src[s].active = false;
  g_nactive--;
  for (int i = 0; i < N_SRC_MAX; i++)
    if (g_src[i].active) {
      g_selected = i;
      return;
    }
  g_selected = 0;
}

static void source_select_step(int dir) {
  if (g_nactive == 0)
    return;
  int s = g_selected;
  for (int i = 0; i < N_SRC_MAX; i++) {
    s = (s + dir + N_SRC_MAX) % N_SRC_MAX;
    if (g_src[s].active) {
      g_selected = s;
      return;
    }
  }
}

static void source_move(int dc, int dr, int gw, int gh) {
  if (!g_src[g_selected].active)
    return;
  Source *s = &g_src[g_selected];
  s->x += dc;
  s->y += dr;
  if (s->x < 0)
    s->x = 0;
  if (s->x >= (float)gw)
    s->x = (float)(gw - 1);
  if (s->y < 0)
    s->y = 0;
  if (s->y >= (float)gh)
    s->y = (float)(gh - 1);
}

/* Preset 1 — DOUBLE SLIT. Two sources side by side on the left, beating in
 * step. Their ripples cross on the right and form bright and dark stripes —
 * Young's famous two-slit experiment (Hecht ch. 9). */
static inline void seed_double_slit(int gw, int gh, float omega, float lambda) {
  float cy = (float)gh * 0.5f;
  float x = (float)gw * 0.28f;
  float sep = (float)gh * 0.22f;
  source_add(x, cy - sep * 0.5f, omega, lambda, 0.f);
  source_add(x, cy + sep * 0.5f, omega, lambda, 0.f);
}

/* Preset 2 — RIPPLE TANK. One source in each corner, all in step. Their
 * ripples overlap into a criss-cross grid — like the water tanks used to teach
 * waves (Crawford ch. 4). */
static inline void seed_ripple_tank(int gw, int gh, float omega, float lambda) {
  float cx = (float)gw * 0.5f, cy = (float)gh * 0.5f;
  float rx = (float)gw * 0.32f, ry = (float)gh * 0.30f;
  source_add(cx - rx, cy - ry, omega, lambda, 0.f);
  source_add(cx + rx, cy - ry, omega, lambda, 0.f);
  source_add(cx - rx, cy + ry, omega, lambda, 0.f);
  source_add(cx + rx, cy + ry, omega, lambda, 0.f);
}

/* Preset 3 — BEAT. Two sources cycling at slightly different speeds. They drift
 * in and out of step, so the combined wave swells loud then fades quiet over
 * and over — the "beat" you hear from two almost-tuned strings (Crawford ch. 1). */
static inline void seed_beat(int gw, int gh, float omega, float lambda) {
  float cy = (float)gh * 0.5f;
  float x0 = (float)gw * 0.30f, x1 = (float)gw * 0.70f;
  source_add(x0, cy, omega, lambda, 0.f);
  source_add(x1, cy, omega * 1.15f, lambda, 0.f);
}

/* Preset 4 — RADIAL STAR. Six sources spaced evenly around a ring, each given a
 * small head-start so the pattern spins. The result is a six-armed star of
 * standing ripples (Born & Wolf §8.6). The ring is squashed vertically because
 * terminal cells are tall, so it ends up looking round. */
static inline void seed_radial_star(int gw, int gh, float omega, float lambda) {
  float cx = (float)gw * 0.5f, cy = (float)gh * 0.5f;
  float R = fminf(cx, cy) * 0.55f;
  float Ry = R * 0.5f; /* squash vertically so the ring looks round */
  const int N = 6;
  for (int i = 0; i < N; i++) {
    float a = (float)i * 2.f * (float)M_PI / (float)N;
    source_add(cx + R * cosf(a), cy + Ry * sinf(a), omega, lambda,
               (float)i * 2.f * (float)M_PI / (float)N);
  }
}

/* Preset 5 — FIVE OSCILLATOR. Four corners plus the centre, each cycling at a
 * slightly different speed. Every pair beats slowly against its neighbours, so
 * the whole pool shifts around and never quite repeats. */
static inline void seed_five_oscillator_grid(int gw, int gh, float lambda) {
  const float fx[5] = {0.22f, 0.78f, 0.50f, 0.22f, 0.78f};
  const float fy[5] = {0.25f, 0.25f, 0.50f, 0.75f, 0.75f};
  const float freq[5] = {0.220f, 0.260f, 0.190f, 0.240f, 0.210f};
  for (int i = 0; i < 5; i++)
    source_add(fx[i] * (float)gw, fy[i] * (float)gh, freq[i], lambda, 0.f);
}

/* Wipe the sources and lay out one of the five named setups. */
static void preset_apply(int p, int gw, int gh) {
  source_clear_all();
  const float o = OMEGA_DEF;
  const float lam = LAMBDA_DEF;

  switch (p) {
  case 1:
    seed_double_slit(gw, gh, o, lam);
    break;
  case 2:
    seed_ripple_tank(gw, gh, o, lam);
    break;
  case 3:
    seed_beat(gw, gh, o, lam);
    break;
  case 4:
    seed_radial_star(gw, gh, o, lam);
    break;
  default:
    seed_five_oscillator_grid(gw, gh, lam);
    break;
  }
}

/* ── §6 analytic field ── */
/* The shortcut engine. Instead of simulating water, it just adds up the sine
 * waves coming from each source at every cell. No grid, instant, always stable.
 * AnalyticTable below saves the slow part of that sum so each frame is cheap. */

/*
 * AnalyticTable — a saved-answers table for the shortcut engine.
 *
 * For each source and cell, the wave height is a sine of two things added: a
 * part that changes with time (how far the source has cycled) and a part that
 * doesn't (how far the cell is from the source, plus the source's head-start).
 * Only the time part changes between frames, so we work out the unchanging part
 * once and stash it here. Then drawing a frame is just one sine per source per
 * cell — fast. (Adding waves up like this is Born & Wolf §8.6.)
 *
 * We rebuild the table only when a source actually moves or changes, which only
 * happens when you press a key — so the rebuild cost is invisible.
 *
 * The table is a plain global array (about 960 KB) rather than something we
 * malloc, because the project rule is "no allocating once running" and a global
 * costs nothing to set aside.
 */
typedef struct {
  /* The saved unchanging part of each source's wave at each cell.
   * Laid out [source][row][col] so scanning a row reads straight through
   * memory, matching how the field is drawn. */
  float kphase[N_SRC_MAX][GRID_H_MAX][GRID_W_MAX];

  /* Set when a source changed, telling the next frame to rebuild the table
   * before it draws. One flag is enough because any change rebuilds it all. */
  bool dirty;
} AnalyticTable;

static AnalyticTable g_at;

static void analytic_recompute(int gw, int gh) {
  g_at.dirty = false;
  for (int s = 0; s < N_SRC_MAX; s++) {
    if (!g_src[s].active)
      continue;
    float k = 2.f * (float)M_PI / (g_src[s].lambda * (float)CELL_W);
    float sx = g_src[s].x, sy = g_src[s].y;
    float ph = g_src[s].phase_init;
    for (int r = 0; r < gh; r++) {
      float dy = ((float)r - sy) * (float)CELL_H;
      for (int c = 0; c < gw; c++) {
        float dx = ((float)c - sx) * (float)CELL_W;
        g_at.kphase[s][r][c] = k * sqrtf(dx * dx + dy * dy) - ph;
      }
    }
  }
}

static float analytic_field(int r, int c, float t) {
  float u = 0.f;
  for (int s = 0; s < N_SRC_MAX; s++) {
    if (!g_src[s].active)
      continue;
    u += sinf(g_src[s].omega * t - g_at.kphase[s][r][c]);
  }
  return u;
}

/* ── §7 scene ── */
/* Scene ties everything above together and owns the engine switch. scene_tick
 * runs whichever engine is active; scene_draw_field turns the wave heights into
 * glyphs. This is where the sources, the simulation grid, and the shortcut
 * engine's table all meet. */

/*
 * Engine — which of the two ways we compute the water this frame.
 *
 *   ENGINE_FDTD     The honest simulation: step a grid forward in time, ripples
 *                   spreading and bouncing. Can blow up if pushed too fast,
 *                   which is why the wave speed is kept low (Yee 1966; Taflove).
 *   ENGINE_ANALYTIC The shortcut: add up the sine waves from each source. No
 *                   grid, no stepping, never blows up (Hecht; Born & Wolf).
 *
 * 'm' flips between them. The flip wipes the grid and resets each source's
 * running clock so both engines start the picture from the same place.
 */
typedef enum { ENGINE_FDTD = 0, ENGINE_ANALYTIC = 1 } Engine;

static const char *engine_name(Engine e) {
  return e == ENGINE_FDTD ? "FDTD" : "ANALYTIC";
}

/*
 * Scene — the one place that owns everything you see on screen.
 *
 * It sits between the wave math (the grid, the source list, the shortcut table
 * — none of which know a terminal exists) and the drawing (which doesn't know
 * any wave physics). The two sides never talk to each other directly; they meet
 * here. Anything that doesn't belong to the grid, a source, the table, or the
 * screen lives in Scene.
 *
 * The fields are grouped on purpose by who reads them — the simulation, the
 * drawing, or both — because mixing them up causes real bugs. If a draw-only
 * value (like the colour theme) crept into the simulation, switching engines or
 * resizing the window could change the physics, which it must never do: the
 * same source list should give the same waves no matter how they're coloured.
 */
typedef struct {
  /* ── Size, read by both sides ──
   * The water area, gw columns by gh rows of cells. Set once at startup and on
   * resize. The math uses it to know how big the field is; the drawing uses it
   * to know where to stop. */
  int gw;
  int gh;

  /* ── Simulation, read by scene_tick ──
   * The actual wave state plus the knobs that steer it. preset is really a
   * control, but it lives here because it survives a colour change. */
  Engine engine;
  Grid grid;
  float t;        /* frame counter, ticks up once per drawn frame */
  int preset;     /* which setup (1..5); reset reloads this one   */
  int fdtd_steps; /* physics steps per drawn frame (sim only)     */
  bool paused;    /* when true, the simulation holds still        */

  /* ── Drawing, read by the draw and HUD functions ──
   * Just look-and-feel choices. Changing the theme only recolours; it must not
   * touch the simulation. needs_redraw is a one-shot "wipe the screen next
   * frame" flag so old glyphs don't linger after a theme or engine change. */
  int theme;         /* which colour scheme (0 .. N_THEMES-1) */
  bool needs_redraw; /* force a full wipe on the next frame   */
} Scene;

static void scene_init(Scene *s, int term_cols, int term_rows) {
  memset(s, 0, sizeof *s);
  s->engine = ENGINE_FDTD;
  s->preset = 1;
  s->theme = 2; /* OCEANIC by default — fits the wave aesthetic  */
  s->fdtd_steps = FDTD_STEPS_DEF;
  s->paused = false;
  s->needs_redraw = true;

  int gw = term_cols;
  int gh = term_rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  if (gw > GRID_W_MAX)
    gw = GRID_W_MAX;
  if (gh > GRID_H_MAX)
    gh = GRID_H_MAX;
  if (gw < 1)
    gw = 1;
  if (gh < 1)
    gh = 1;
  s->gw = gw;
  s->gh = gh;

  grid_alloc(&s->grid, gw, gh);
  s->grid.damping = DAMPING_DEF;
  theme_apply(s->theme);

  preset_apply(s->preset, gw, gh);
  g_at.dirty = true;
}

static void scene_free(Scene *s) { grid_free(&s->grid); }

static void scene_resize(Scene *s, int term_cols, int term_rows) {
  int gw = term_cols;
  int gh = term_rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  if (gw > GRID_W_MAX)
    gw = GRID_W_MAX;
  if (gh > GRID_H_MAX)
    gh = GRID_H_MAX;
  if (gw < 1)
    gw = 1;
  if (gh < 1)
    gh = 1;
  s->gw = gw;
  s->gh = gh;
  grid_resize(&s->grid, gw, gh);
  preset_apply(s->preset, gw, gh);
  g_at.dirty = true;
  s->needs_redraw = true;
}

static void scene_toggle_engine(Scene *s) {
  s->engine = (s->engine == ENGINE_FDTD) ? ENGINE_ANALYTIC : ENGINE_FDTD;
  grid_clear(&s->grid);
  for (int i = 0; i < N_SRC_MAX; i++)
    g_src[i].phase_run = g_src[i].phase_init;
  s->t = 0.f;
  g_at.dirty = true;
  s->needs_redraw = true;
}

/* Tap each source's cell to keep its waves going, then nudge the source a
 * little further along its cycle. The nudge is sized so the source covers the
 * same amount of cycle per drawn frame however many physics steps run — that's
 * what keeps the wave speed matched to the shortcut engine. */
static void scene_fdtd_inject(Scene *s) {
  float dphase = 1.f / (float)s->fdtd_steps;
  for (int i = 0; i < N_SRC_MAX; i++) {
    Source *src = &g_src[i];
    if (!src->active)
      continue;
    int sx = (int)src->x;
    int sy = (int)src->y;
    if (sx < 1 || sx >= s->grid.cols - 1)
      continue;
    if (sy < 1 || sy >= s->grid.rows - 1)
      continue;
    s->grid.cur[sy * s->grid.cols + sx] += SOURCE_AMP * sinf(src->phase_run);
    src->phase_run += src->omega * dphase;
  }
}

/* One drawn frame of the honest simulation: run several physics steps, each
 * one tapping the sources then advancing the water. Running more steps per
 * frame makes the waves look faster without speeding up any single step — which
 * matters because a single step that's too fast makes the simulation blow up. */
static void tick_fdtd_engine(Scene *s) {
  for (int i = 0; i < s->fdtd_steps; i++) {
    scene_fdtd_inject(s);
    grid_tick(&s->grid);
  }
}

/* The shortcut engine has no water to step — it works out each cell straight
 * from the sources when it draws. So all this does is rebuild the saved-answers
 * table when a source has moved. See the AnalyticTable note. */
static void tick_analytic_engine(Scene *s) {
  if (g_at.dirty)
    analytic_recompute(s->gw, s->gh);
}

/* Tick the frame counter that both engines use to know how far time has moved. */
static inline void advance_render_frame_clock(Scene *s) { s->t += 1.f; }

/* One frame of wave work: run whichever engine is active, then move the clock.
 * Does nothing while paused. */
static void scene_tick(Scene *s) {
  if (s->paused)
    return;

  if (s->engine == ENGINE_FDTD)
    tick_fdtd_engine(s);
  else
    tick_analytic_engine(s);

  advance_render_frame_clock(s);
}

/* Draw one cell: pick a glyph and colour for its height, or leave it blank if
 * the water there is basically flat (so the calm lines show). The row is shifted
 * down so it doesn't draw over the HUD at the top. (Glyph ramp: Bourke.) */
static inline void paint_field_cell(int field_r, int field_c, float u_norm) {
  int lv = amplitude_level(u_norm);
  if (lv < 0)
    return;
  attr_t at = level_attr(lv);
  attron(at);
  mvaddch(field_r + HUD_ROWS_TOP, field_c,
          (chtype)(unsigned char)k_field_chars[lv]);
  attroff(at);
}

/* Draw the simulation's water: read each cell's height and scale it down to the
 * -1..1 range the glyph picker wants (the simulation's heights can grow large). */
static void paint_field_from_fdtd(Scene *s) {
  const float *cu = s->grid.cur;
  const int cols = s->grid.cols;
  for (int r = 0; r < s->gh; r++) {
    int row = r * cols;
    for (int c = 0; c < s->gw; c++)
      paint_field_cell(r, c, cu[row + c] / MAX_AMP);
  }
}

/* Draw the shortcut engine's water: for each cell add up the wave from every
 * source, then divide by the source count so the total never overshoots the
 * -1..1 range no matter how many sources are switched on. */
static void paint_field_from_analytic(Scene *s) {
  float norm = (g_nactive > 0) ? 1.f / (float)g_nactive : 1.f;
  for (int r = 0; r < s->gh; r++) {
    for (int c = 0; c < s->gw; c++)
      paint_field_cell(r, c, analytic_field(r, c, s->t) * norm);
  }
}

/* Draw the whole water area, letting the active engine supply the heights.
 * Both engines hand off to the same glyph-and-colour code. */
static void scene_draw_field(Scene *s) {
  if (s->engine == ENGINE_FDTD)
    paint_field_from_fdtd(s);
  else
    paint_field_from_analytic(s);
}

static void scene_draw_sources(Scene *s) {
  for (int i = 0; i < N_SRC_MAX; i++) {
    if (!g_src[i].active)
      continue;
    int dr = (int)g_src[i].y + HUD_ROWS_TOP;
    int dc = (int)g_src[i].x;
    if (dr < HUD_ROWS_TOP || dr >= HUD_ROWS_TOP + s->gh)
      continue;
    if (dc < 0 || dc >= s->gw)
      continue;
    bool sel = (i == g_selected);
    int cp = sel ? CP_SRC_SEL : CP_SRC_OFF;
    chtype gl = sel ? 'X' : 'o';
    attron(COLOR_PAIR(cp) | A_BOLD);
    mvaddch(dr, dc, gl);
    attroff(COLOR_PAIR(cp) | A_BOLD);
  }
}

/* ── §8 screen ── */
/* The terminal size, the three HUD rows, and the one call per frame that pushes
 * everything to the terminal. */

/*
 * Screen — just the terminal's width and height in cells.
 *
 * It's tiny on purpose. ncurses keeps its own copy of the screen and works out
 * what changed each frame, so we don't keep our own — we only need the size, to
 * place the HUD rows and to know where the water area ends.
 *
 * Each frame goes: wipe the in-memory screen, draw the water, draw the source
 * markers, then draw the HUD last so it sits on top — and finally one call hands
 * it all to ncurses, which redraws only the cells that actually changed (less
 * flicker; see the ncurses HOWTO §11).
 */
typedef struct {
  int cols; /* terminal width  in cells */
  int rows; /* terminal height in cells */
} Screen;

static void screen_init(Screen *sc, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

static const char *preset_name(int p) {
  switch (p) {
  case 1:
    return "DBL-SLIT";
  case 2:
    return "RIPPLE";
  case 3:
    return "BEAT";
  case 4:
    return "RADIAL";
  default:
    return "FIVE-OSC";
  }
}

/* Draw the three info rows: top-right shows engine, fps, and paused/running;
 * the line below it shows the current settings; the bottom row lists the keys.
 * Everything in between is the water. */
static void hud_draw(Screen *sc, Scene *s, double fps) {
  char status[80];
  snprintf(status, sizeof status, " %s  %5.1f fps  %s ", engine_name(s->engine),
           fps, s->paused ? "PAUSED " : "running");

  attr_t a_status = COLOR_PAIR(CP_HUD) | A_BOLD;
  attron(a_status);
  mvprintw(0, sc->cols - (int)strlen(status), "%s", status);
  attroff(a_status);

  char params[160];
  Source *sel = g_src[g_selected].active ? &g_src[g_selected] : NULL;
  if (s->engine == ENGINE_FDTD) {
    snprintf(params, sizeof params,
             " preset:%d %-8s  theme:%-7s  src:%d/%d sel:%d  "
             "damp:%.3f  steps:%d  ",
             s->preset, preset_name(s->preset), k_themes[s->theme].name,
             g_nactive, N_SRC_MAX, g_selected, s->grid.damping, s->fdtd_steps);
  } else if (sel) {
    snprintf(params, sizeof params,
             " preset:%d %-8s  theme:%-7s  src:%d/%d sel:%d  "
             "lam:%.0f  omega:%.3f  ",
             s->preset, preset_name(s->preset), k_themes[s->theme].name,
             g_nactive, N_SRC_MAX, g_selected, sel->lambda, sel->omega);
  } else {
    snprintf(params, sizeof params,
             " preset:%d %-8s  theme:%-7s  src:0/%d  (no source) ", s->preset,
             preset_name(s->preset), k_themes[s->theme].name, N_SRC_MAX);
  }
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(1, 0, "%s", params);
  attroff(COLOR_PAIR(CP_HUD));

  attr_t a_hint = COLOR_PAIR(CP_HINT) | A_BOLD;
  attron(a_hint);
  mvprintw(sc->rows - 1, 0,
           " q:quit  m:engine  spc:pause  r:reset  1-5:preset  t/T:theme  "
           "TAB:select  arrows:move  n:add  x:del  +/-:lam|steps  "
           "f/F:omega  d/D:damp  p:impulse ");
  attroff(a_hint);
}

static void screen_draw(Screen *sc, Scene *s, double fps) {
  if (s->needs_redraw) {
    erase();
    s->needs_redraw = false;
  } else {
    erase(); /* simplest correct path; ncurses diffs internally */
  }
  scene_draw_field(s);
  scene_draw_sources(s);
  hud_draw(sc, s, fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app ── */
/* The main loop, the signal handlers, and the key handling — where your
 * keystrokes meet the simulation. Each pass reads a key, ticks the scene,
 * sleeps to hold a steady frame rate, then draws. */

/*
 * App — the whole program in one box, kept as a single global (g_app).
 *
 * It's global because the signal handlers below need to reach it, and a handler
 * can only touch global state. The two flags are written by handlers and read
 * by the loop: one says "time to quit," the other says "the window resized." A
 * handler can do almost nothing safely, so it just flips a flag and the loop
 * does the real work next time around.
 *
 * The flags are `volatile sig_atomic_t` for two reasons: that type is the only
 * kind a signal handler is allowed to write, and `volatile` stops the compiler
 * from caching the value so the loop always sees the latest one.
 */
typedef struct {
  Scene scene;                       /* the whole world           */
  Screen screen;                     /* terminal size             */
  volatile sig_atomic_t running;     /* cleared when asked to quit */
  volatile sig_atomic_t need_resize; /* set when the window resizes */
} App;

static App g_app;
static void on_exit_sig(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

/* Handle one keypress. A few keys mean different things per engine: +/- change
 * the source's wavelength in the shortcut engine but the step count in the
 * simulation; 'p' (drop a pebble) and d/D (fade rate) only do anything in the
 * simulation, since the shortcut engine has no water to disturb. */
static bool app_handle_key(App *a, int ch) {
  Scene *sc = &a->scene;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    sc->paused = !sc->paused;
    break;
  case 'm':
  case 'M':
    scene_toggle_engine(sc);
    break;

  case 'r':
  case 'R':
    preset_apply(sc->preset, sc->gw, sc->gh);
    grid_clear(&sc->grid);
    sc->t = 0.f;
    g_at.dirty = true;
    sc->needs_redraw = true;
    break;

  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
    sc->preset = ch - '0';
    preset_apply(sc->preset, sc->gw, sc->gh);
    grid_clear(&sc->grid);
    sc->t = 0.f;
    g_at.dirty = true;
    sc->needs_redraw = true;
    break;

  case 't':
    sc->theme = (sc->theme + 1) % N_THEMES;
    theme_apply(sc->theme);
    sc->needs_redraw = true;
    break;
  case 'T':
    sc->theme = (sc->theme + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->theme);
    sc->needs_redraw = true;
    break;

  case '\t':
    source_select_step(+1);
    break;
  case KEY_BTAB:
    source_select_step(-1);
    break;

  case KEY_UP:
    source_move(0, -1, sc->gw, sc->gh);
    g_at.dirty = true;
    break;
  case KEY_DOWN:
    source_move(0, +1, sc->gw, sc->gh);
    g_at.dirty = true;
    break;
  case KEY_LEFT:
    source_move(-1, 0, sc->gw, sc->gh);
    g_at.dirty = true;
    break;
  case KEY_RIGHT:
    source_move(+1, 0, sc->gw, sc->gh);
    g_at.dirty = true;
    break;

  case 'n':
  case 'N':
    source_add((float)sc->gw * 0.5f, (float)sc->gh * 0.5f, OMEGA_DEF,
               LAMBDA_DEF, 0.f);
    g_at.dirty = true;
    break;
  case 'x':
  case 'X':
    if (g_nactive > 0) {
      source_delete(g_selected);
      g_at.dirty = true;
    }
    break;

  case '+':
  case '=':
    if (sc->engine == ENGINE_ANALYTIC && g_src[g_selected].active) {
      float *l = &g_src[g_selected].lambda;
      *l += LAMBDA_STEP;
      if (*l > LAMBDA_MAX)
        *l = LAMBDA_MAX;
      g_at.dirty = true;
    } else {
      sc->fdtd_steps++;
      if (sc->fdtd_steps > FDTD_STEPS_MAX)
        sc->fdtd_steps = FDTD_STEPS_MAX;
    }
    break;
  case '-':
    if (sc->engine == ENGINE_ANALYTIC && g_src[g_selected].active) {
      float *l = &g_src[g_selected].lambda;
      *l -= LAMBDA_STEP;
      if (*l < LAMBDA_MIN)
        *l = LAMBDA_MIN;
      g_at.dirty = true;
    } else {
      sc->fdtd_steps--;
      if (sc->fdtd_steps < FDTD_STEPS_MIN)
        sc->fdtd_steps = FDTD_STEPS_MIN;
    }
    break;

  case 'f':
    if (g_src[g_selected].active) {
      g_src[g_selected].omega -= OMEGA_STEP;
      if (g_src[g_selected].omega < 0.01f)
        g_src[g_selected].omega = 0.01f;
    }
    break;
  case 'F':
    if (g_src[g_selected].active)
      g_src[g_selected].omega += OMEGA_STEP;
    break;

  case 'd':
    if (sc->engine == ENGINE_FDTD) {
      sc->grid.damping -= DAMPING_STEP;
      if (sc->grid.damping < DAMPING_MIN)
        sc->grid.damping = DAMPING_MIN;
    }
    break;
  case 'D':
    if (sc->engine == ENGINE_FDTD) {
      sc->grid.damping += DAMPING_STEP;
      if (sc->grid.damping > DAMPING_MAX)
        sc->grid.damping = DAMPING_MAX;
    }
    break;

  case 'p':
  case 'P':
    if (sc->engine == ENGINE_FDTD)
      grid_impulse(&sc->grid, sc->grid.cols / 2, sc->grid.rows / 2);
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_sig);
  signal(SIGTERM, on_exit_sig);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  /* screen first so COLORS is initialised before scene_init touches it */
  int initial_theme = 2; /* OCEANIC */
  screen_init(&app->screen, initial_theme);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);
  app->scene.theme = initial_theme;
  theme_apply(initial_theme);

  int64_t ft = clock_ns(), fa = 0;
  int fc = 0;
  double fps = 0.;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      ft = clock_ns();
      fa = 0;
      fc = 0;
    }

    int64_t now = clock_ns(), dt = now - ft;
    ft = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    fc++;
    fa += dt;
    if (fa >= 500 * NS_PER_MS) {
      fps = (double)fc / ((double)fa / (double)NS_PER_SEC);
      fc = 0;
      fa = 0;
    }

    scene_tick(&app->scene);

    int64_t elapsed = clock_ns() - ft + dt;
    clock_sleep_ns(RENDER_NS - elapsed);

    screen_draw(&app->screen, &app->scene, fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
