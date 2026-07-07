/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * wave_interference.c — a ripple tank: overlapping circular waves that add up.
 * A few point sources send out expanding rings; every cell shows the SUM of all
 * the rings, so crests reinforce into bright spots and a crest meeting a trough
 * cancels into the still "nodal lines" — the interference pattern.
 *
 * Sisters: worley_cellular_noise.c (another "distance to scattered points" field),
 * magnetic_fields.c (a vector field, where this is a scalar one).
 *
 * Keys:  q/ESC quit   space/p pause   r reset   t theme   +/- wavelength   [/] sources
 * Build: gcc -std=c11 -O2 -Wall -Wextra procedural/fields/wave_interference.c -o wave_interference -lncurses -lm
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

/* ── §1 CONFIG — constants, colour ramp, themes ── */

enum {
  TARGET_FPS = 60,
};

enum {
  /* How many point sources can be live at once, and the default on startup.
   * Two is the textbook two-source interference; more makes a busier weave. */
  MIN_SOURCES = 1,
  MAX_SOURCES = 6,
  DEFAULT_SOURCES = 2,

  /* Brightness bands from deepest trough to tallest crest. */
  N_SHADES = 8,
};

#define TWO_PI (2.0f * (float)M_PI)

/* A terminal cell is about twice as tall as it is wide, so one row-step covers
 * this many column-widths of real distance. Without it the circular rings would
 * render as tall ellipses. */
#define CELL_ASPECT 2.0f

/* Ring spacing: columns between one crest and the next. Smaller = tighter, more
 * rings. Adjusted live with +/-. */
#define WAVELENGTH_MIN 5.0f
#define WAVELENGTH_MAX 28.0f
#define WAVELENGTH_DEFAULT 10.0f
#define WAVELENGTH_STEP 1.0f

/* How many times a second each point bobs up and down. Crests travel outward at
 * frequency × wavelength cells per second; kept gentle so the eye can follow. */
#define FREQUENCY_HZ 0.5f

/* Longest frame we'll trust. A hiccup (window drag, debugger) can make one frame
 * huge; clamping it stops the wave clock from lurching forward. */
#define DT_CAP_SEC 0.10f

#define NS_PER_SEC 1000000000LL
#define HUD_BUF_LEN 80

/* ncurses colour-pair slots: N_SHADES tinted bands first, then the HUD pairs
 * (which never change with the theme). */
enum {
  SHADE_BASE = 1, /* bands occupy SHADE_BASE .. SHADE_BASE + N_SHADES - 1 */
  PAIR_HUD = SHADE_BASE + N_SHADES,
  PAIR_HINT,
};

/* Bands this bright or brighter are drawn bold, so crests pop on 8-colour
 * terminals that can't show a fine ramp. */
#define BOLD_FROM_BAND 6

/* Glyphs for the bands, flat trough → tall crest. Band 0 is a space (deepest
 * troughs read as empty water), so it's never actually drawn. */
static const char k_shade_glyph[N_SHADES] = {' ', '.', ':', '-',
                                             '=', '+', '*', '#'};

/* True when the terminal has the 256-colour palette. Set once at startup. */
static bool g_has_256 = false;

/*
 * Theme — one named colour scheme for the water, dark trough → bright crest.
 *
 * height_to_band (§4) picks which of the N_SHADES bands a cell falls in; this
 * table just says what colour each band is. 't' cycles the presets and re-tints
 * the pairs in place, so the water on screen recolours instantly.
 *
 *   name    label shown in the HUD; never NULL.
 *   fg[]    the N_SHADES band colours for a 256-colour terminal, dim → bright,
 *           all in the bright half of the palette (>= 24) so even the low bands
 *           stay visible (see CLAUDE.md "Theme Palette Brightness").
 *   fg_8[]  the same bands for a plain 8-colour terminal.
 */
typedef struct {
  const char *name;
  int fg[N_SHADES];
  int fg_8[N_SHADES];
} Theme;

static const Theme k_themes[] = {
    {"ocean",
     {24, 25, 31, 38, 45, 51, 123, 195},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
      COLOR_WHITE, COLOR_WHITE}},
    {"fire",
     {52, 88, 124, 160, 196, 202, 214, 226},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_WHITE, COLOR_WHITE}},
    {"plasma",
     {54, 92, 128, 164, 200, 206, 213, 219},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE}},
    {"forest",
     {24, 28, 34, 70, 76, 82, 118, 191},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_WHITE, COLOR_WHITE}},
    {"mono",
     {240, 243, 246, 248, 250, 252, 254, 255},
     {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* ── §2 STATE — the domain types (Source, WaveField) + the run harness (Scene, App) ── */

/*
 * Source — one point where the water is bobbing, sending out rings.
 *
 *   x, y    where the ripple starts, in screen cells.
 *   phase   a head-start on the up/down bob, in radians. Random per source so
 *           they don't all peak in lockstep, which shifts where the nodal lines
 *           fall.
 */
typedef struct {
  float x, y;
  float phase;
} Source;

/*
 * WaveField — the whole pond: the live sources plus the shared settings every
 * source obeys. The surface height isn't stored anywhere; it's recomputed from
 * these each frame (see wave_height_at).
 *
 *   sources[]    every possible source, all pre-placed; only the first
 *                n_sources of them count.
 *   n_sources    how many are live right now, 1..MAX_SOURCES ([ and ] change it).
 *   wavelength   ring spacing in columns (+/- change it).
 *   time         the shared wave clock in seconds; advances with real time and
 *                drives every source's bob.
 */
typedef struct {
  Source sources[MAX_SOURCES];
  int n_sources;
  float wavelength;
  float time;
} WaveField;

/*
 * Scene — the whole run as a table of contents:
 *   WHAT  — the wave field (its sources and settings).
 *   run   — the pause toggle.
 *   look  — which colour theme is drawn (render-only; the physics ignores it).
 */
typedef struct {
  WaveField field;
  bool paused;
  int theme_idx;
} Scene;

/* Terminal size in cells. Row 0 holds the status line; the last row the hints. */
typedef struct {
  int cols, rows;
} Screen;

/*
 * FpsCounter — a smoothed frames-per-second readout. One frame at a time jumps
 * around, so this averages over a short window.
 *
 *   frame_count   frames seen so far this window.
 *   window_ns     time elapsed so far this window, in nanoseconds.
 *   display       the last finished average, shown in the HUD.
 */
typedef struct {
  int frame_count;
  int64_t window_ns;
  double display;
} FpsCounter;

/*
 * App — everything kept alive for one run.
 *
 * g_app is the only global: the signal handlers need to reach the two flags and
 * can't be handed a pointer, so the flags are sig_atomic_t.
 */
typedef struct {
  Scene scene;
  Screen screen;
  FpsCounter fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

/* ── §3 PERFORMANCE — clock + smoothed fps (frame cap is in §7 main) ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec req = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&req, NULL);
}

static void fps_counter_init(FpsCounter *f) {
  f->frame_count = 0;
  f->window_ns = 0;
  f->display = 0.0;
}

static void fps_counter_tick(FpsCounter *f, int64_t dt_ns) {
  const int64_t FPS_WINDOW_NS = NS_PER_SEC / 2; /* 500 ms */
  f->frame_count++;
  f->window_ns += dt_ns;
  if (f->window_ns < FPS_WINDOW_NS)
    return;
  f->display =
      (double)f->frame_count * (double)NS_PER_SEC / (double)f->window_ns;
  f->frame_count = 0;
  f->window_ns = 0;
}

/* ── §4 LOGIC — pure decisions: no mutation, no I/O; read only the args ── */

/* One source's ripple at a cell: a travelling ring — the sine of how far along
 * the wave the cell is (distance × wavenumber) minus how long we've been running
 * (ω × time), offset by the source's own phase. Distance is stretched vertically
 * so the rings stay round. */
static float source_wave_at(const Source *s, float k, float omega, float time,
                            float cx, float cy) {
  float dx = cx - s->x;
  float dy = (cy - s->y) * CELL_ASPECT;
  float dist = sqrtf(dx * dx + dy * dy);
  return sinf(k * dist - omega * time + s->phase);
}

/* The combined surface height at one cell: add up every source's ripple and
 * squash to roughly −1..+1. This one function IS the simulation — waves just
 * sum, and the interference pattern is what that sum looks like. */
static float wave_height_at(const WaveField *wf, float cx, float cy) {
  float k = TWO_PI / wf->wavelength;   /* radians of wave per column */
  float omega = TWO_PI * FREQUENCY_HZ; /* radians of bob per second */
  float sum = 0.0f;
  for (int i = 0; i < wf->n_sources; i++)
    sum += source_wave_at(&wf->sources[i], k, omega, wf->time, cx, cy);
  return sum / (float)wf->n_sources;
}

/* Turn a height in −1..+1 into a brightness band 0..N_SHADES-1: deepest trough
 * flat/dark, tallest crest bright. */
static int height_to_band(float h) {
  float t = (h + 1.0f) * 0.5f; /* −1..+1 → 0..1 */
  int band = (int)(t * (float)N_SHADES);
  if (band < 0)
    band = 0;
  if (band >= N_SHADES)
    band = N_SHADES - 1;
  return band;
}

/* ── §5 SIMULATION — place sources, advance the wave clock ── */

static float urand01(void) { return (float)rand() / (float)RAND_MAX; }

/* Drop a source at a random spot with a random head-start on its bob. */
static void source_place(Source *s, int cols, int rows) {
  s->x = urand01() * (float)cols;
  s->y = urand01() * (float)rows;
  s->phase = urand01() * TWO_PI;
}

/* Scatter every source afresh. Used at startup, on reset (r), and on resize —
 * a resized window would otherwise leave sources hanging off the edge. */
static void field_scatter_sources(WaveField *wf, int cols, int rows) {
  for (int i = 0; i < MAX_SOURCES; i++)
    source_place(&wf->sources[i], cols, rows);
}

/* Build the field from scratch: default count and spacing, sources scattered,
 * clock at zero. Runs at startup and on resize. */
static void scene_init(Scene *s, int cols, int rows) {
  s->field.n_sources = DEFAULT_SOURCES;
  s->field.wavelength = WAVELENGTH_DEFAULT;
  s->field.time = 0.0f;
  field_scatter_sources(&s->field, cols, rows);
  s->paused = false;
  s->theme_idx = 0;
}

/* Re-scatter the sources for a fresh pattern (the 'r' key), keeping the count,
 * spacing, and clock. */
static void scene_reset(Scene *s, int cols, int rows) {
  field_scatter_sources(&s->field, cols, rows);
}

/* Advance the shared wave clock by the real time elapsed. While paused it does
 * nothing, but the screen still redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->field.time += dt;
}

/* ── §6 RENDER — field → screen; reads state, writes only the terminal ── */

/* Re-tint the band pairs for the chosen theme. HUD pairs are left alone so they
 * keep their colour whatever theme is up. */
static void theme_apply(int idx) {
  const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
  for (int b = 0; b < N_SHADES; b++)
    init_pair(SHADE_BASE + b, fg[b], COLOR_BLACK);
}

/* Set up the HUD colours once. The -1 background sits the HUD on the terminal's
 * real background instead of a black box. */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* Colour + weight for a shade band: brighter bands are bold so crests pop even on
 * 8-colour terminals. */
static attr_t band_attr(int band) {
  attr_t attr = COLOR_PAIR(SHADE_BASE + band);
  if (band >= BOLD_FROM_BAND)
    attr |= A_BOLD;
  return attr;
}

/* Draw the whole pond: one glyph per cell, coloured by its summed height. Band 0
 * (deepest trough) is left blank so the stillest water reads as empty. */
static void field_draw(const WaveField *wf, int cols, int rows) {
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      int band = height_to_band(wave_height_at(wf, (float)col, (float)row));
      if (band == 0)
        continue;
      attr_t attr = band_attr(band);
      attron(attr);
      mvaddch(row, col, (chtype)(unsigned char)k_shade_glyph[band]);
      attroff(attr);
    }
  }
}

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* don't let stdin interrupt frame writes */
  start_color();
  use_default_colors(); /* lets HUD pairs use the terminal's own background */
  getmaxyx(stdscr, s->rows, s->cols);
}

static void screen_free(Screen *s) {
  (void)s;
  endwin();
}

static void screen_resize(Screen *s) {
  endwin();
  refresh();
  getmaxyx(stdscr, s->rows, s->cols);
}

/* Print one coloured, bold line — shared by both HUD rows. */
static void hud_paint_text(int row, int col, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Build the top-row status text: fps, sources, wavelength, theme, paused. */
static void format_hud_status(const Scene *s, double fps, char *buf,
                              size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  src:%d  wav:%2.0f  [%s] %s ", fps,
           s->field.n_sources, (double)s->field.wavelength,
           k_themes[s->theme_idx].name, s->paused ? "PAUSED " : "running");
}

/* Paint the status flush against the right edge of row 0. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
  char buf[HUD_BUF_LEN];
  format_hud_status(s, fps, buf, sizeof buf);
  int right_col = sc->cols - (int)strlen(buf);
  if (right_col < 0)
    right_col = 0;
  hud_paint_text(0, right_col, PAIR_HUD, buf);
}

/* Paint the key-list strip along the bottom row. */
static void draw_hud_hint(const Screen *sc) {
  static const char *KEY_HINT =
      " q:quit  spc:pause  r:reset  t:theme  +/-:wavelen  [/]:sources ";
  hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

static void screen_draw_hud(const Screen *sc, double fps, const Scene *s) {
  draw_hud_status(sc, s, fps);
  draw_hud_hint(sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §7 APP — events (mutate state OUTSIDE the tick) + main loop ── */

static App g_app;

static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void scene_cycle_theme(Scene *s) {
  s->theme_idx = (s->theme_idx + 1) % THEME_COUNT;
  theme_apply(s->theme_idx);
}

/* Nudge the ring spacing, clamped to a sane range. */
static void field_adjust_wavelength(WaveField *wf, float delta) {
  wf->wavelength += delta;
  if (wf->wavelength < WAVELENGTH_MIN)
    wf->wavelength = WAVELENGTH_MIN;
  if (wf->wavelength > WAVELENGTH_MAX)
    wf->wavelength = WAVELENGTH_MAX;
}

/* Add or drop a source, clamped to 1..MAX. The extra sources are always pre-
 * placed, so adding one just reveals the next already-scattered spot. */
static void field_adjust_sources(WaveField *wf, int delta) {
  wf->n_sources += delta;
  if (wf->n_sources < MIN_SOURCES)
    wf->n_sources = MIN_SOURCES;
  if (wf->n_sources > MAX_SOURCES)
    wf->n_sources = MAX_SOURCES;
}

/* Rebuild the field at the new size but keep the user's theme and pause state,
 * which a plain scene_init would have reset. */
static void app_do_resize(App *app) {
  int saved_theme = app->scene.theme_idx;
  bool saved_paused = app->scene.paused;
  int saved_sources = app->scene.field.n_sources;
  float saved_wavelength = app->scene.field.wavelength;

  screen_resize(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  app->scene.theme_idx = saved_theme;
  app->scene.paused = saved_paused;
  app->scene.field.n_sources = saved_sources;
  app->scene.field.wavelength = saved_wavelength;
  app->need_resize = 0;
}

/* Act on one keypress. Returns false only when the user wants to quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;

  case ' ':
  case 'p':
  case 'P':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reset(s, app->screen.cols, app->screen.rows);
    break;
  case 't':
  case 'T':
    scene_cycle_theme(s);
    break;

  case '+':
  case '=':
    field_adjust_wavelength(&s->field, WAVELENGTH_STEP);
    break;
  case '-':
  case '_':
    field_adjust_wavelength(&s->field, -WAVELENGTH_STEP);
    break;
  case ']':
    field_adjust_sources(&s->field, +1);
    break;
  case '[':
    field_adjust_sources(&s->field, -1);
    break;

  default:
    break;
  }
  return true;
}

/* The main loop: each pass measures real elapsed time, reads keys, advances the
 * wave clock by that much, draws, then sleeps to hold ~60 fps. */
int main(void) {
  srand((unsigned int)clock_ns());
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  Scene *scene = &app->scene;
  app->running = 1;
  fps_counter_init(&app->fps);

  screen_init(&app->screen);
  g_has_256 = (COLORS >= 256);
  scene_init(scene, app->screen.cols, app->screen.rows);
  theme_apply(scene->theme_idx);
  hud_pairs_init();

  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
  int64_t last_ns = clock_ns();

  while (app->running) {
    if (app->need_resize) {
      app_do_resize(app);
      last_ns = clock_ns();
    }

    int64_t frame_start_ns = clock_ns();
    int64_t dt_ns = frame_start_ns - last_ns;
    last_ns = frame_start_ns;
    float dt = (float)dt_ns / (float)NS_PER_SEC;
    if (dt > DT_CAP_SEC)
      dt = DT_CAP_SEC;

    for (int ch; (ch = getch()) != ERR;) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }

    scene_tick(scene, dt);
    fps_counter_tick(&app->fps, dt_ns);

    erase();
    field_draw(&scene->field, app->screen.cols, app->screen.rows);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
