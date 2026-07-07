/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * barkley_spiral_waves.c — a rotating spiral wave in an excitable medium.
 * A sheet of cells (think heart muscle, or the Belousov–Zhabotinsky reaction)
 * carries a wave that curls around a fixed centre and rotates forever: each cell
 * rests, fires when a neighbour fires, then goes numb for a while — and that numb
 * tail is what winds the wave into a spiral instead of letting it collapse.
 *
 * Sisters: reaction_diffusion_gray_scott.c (also two fields on a grid, but Turing
 * spots/stripes not travelling waves), wave_interference.c (waves that pass through
 * each other — here they annihilate on contact).
 *
 * Barkley 1991, "A model for fast computer simulation of waves in excitable media",
 * Physica D 49, 61–70.  http://www.scholarpedia.org/article/Barkley_model
 *
 * Keys:  q/ESC quit   space/p pause   r reseed   t theme   +/- speed
 * Build: gcc -std=c11 -O2 -Wall -Wextra procedural/fields/barkley_spiral_waves.c -o barkley_spiral_waves -lncurses
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 CONFIG — constants, Barkley parameters, colour ramp, themes ── */

enum {
  TARGET_FPS = 60,
};

enum {
  /* Brightness bands from resting (dark) to freshly excited (bright). */
  N_SHADES = 8,
};

/*
 * Barkley parameters. These three set the character of the medium; the defaults
 * are a well-behaved spiral. (See the paper / Scholarpedia for the map of what
 * different a,b,ε give — spirals, meandering tips, breakup.)
 *
 *   BARKLEY_A    excitation strength — roughly how tall the excited plateau is.
 *   BARKLEY_B    threshold offset — how hard it is to fire from rest (bigger =
 *                harder, so a wider resting gap between waves).
 *   BARKLEY_EPS  the fast/slow split — small ε makes u fire much faster than v
 *                recovers, which is what keeps a clean travelling front.
 */
#define BARKLEY_A 0.75f
#define BARKLEY_B 0.01f
#define BARKLEY_EPS 0.02f

/* How far excitation spreads per step, and the fixed time step. Bigger diffusion
 * makes fatter, better-separated spiral arms (easier to read at low resolution).
 * dt is kept well under the stability limit (~0.2 with these values) so the
 * explicit update never blows up. */
#define DIFFUSION 2.0f
#define SIM_DT 0.020f

/* A terminal cell is about twice as tall as wide. Diffusing vertically by this
 * factor less (dividing the vertical term by its square) undoes the squish, so
 * the spiral renders round instead of stretched. */
#define CELL_ASPECT 2.0f
#define INV_ASPECT_SQ (1.0f / (CELL_ASPECT * CELL_ASPECT))

/* Recovery pre-loaded into the top half of the grid at seed time. It raises the
 * firing threshold there, blocking that part of the wavefront so the other end
 * is left free to curl — the break that makes a spiral instead of rings. */
#define SEED_BLOCK_V 0.6f

/* Barkley substeps run per rendered frame = the sim speed knob (+/-). Fixed per
 * frame (not tied to frame time) so the explicit update stays stable and a slow
 * terminal can't trigger a runaway catch-up. */
#define SUBSTEPS_MIN 1
#define SUBSTEPS_MAX 14
#define SUBSTEPS_DEFAULT 8

#define NS_PER_SEC 1000000000LL
#define HUD_BUF_LEN 80

/* ncurses colour-pair slots: N_SHADES tinted bands, then the HUD pairs (which
 * never change with the theme). */
enum {
  SHADE_BASE = 1, /* bands occupy SHADE_BASE .. SHADE_BASE + N_SHADES - 1 */
  PAIR_HUD = SHADE_BASE + N_SHADES,
  PAIR_HINT,
};

/* Bands this bright or brighter are drawn bold, so the excited front pops on
 * 8-colour terminals that can't show a fine ramp. */
#define BOLD_FROM_BAND 6

/* Glyphs for the bands, resting → freshly excited. Band 0 is a space (resting
 * cells read as empty), so it's never actually drawn. */
static const char k_shade_glyph[N_SHADES] = {' ', '.', ':', '-',
                                             '=', '+', '*', '#'};

/* True when the terminal has the 256-colour palette. Set once at startup. */
static bool g_has_256 = false;

/*
 * Theme — one named colour scheme for the medium, dark rest → bright excitation.
 *
 * medium_draw (§6) picks a band 0..N_SHADES-1 from a cell's excitation; this
 * table says what colour each band is. 't' cycles the presets and re-tints in
 * place, so the sheet recolours instantly.
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
    {"ember",
     {52, 88, 124, 160, 196, 202, 214, 226},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_WHITE, COLOR_WHITE}},
    {"bz",
     {24, 28, 34, 70, 112, 154, 191, 229},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE}},
    {"ocean",
     {24, 25, 31, 38, 45, 51, 123, 195},
     {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
      COLOR_WHITE, COLOR_WHITE}},
    {"plasma",
     {54, 92, 128, 164, 200, 206, 213, 219},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE}},
    {"mono",
     {240, 243, 246, 248, 250, 252, 254, 255},
     {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* ── §2 STATE — the domain type (ExcitableMedium) + the run harness (Scene, App) ── */

/*
 * ExcitableMedium — the sheet the waves live on: two values per cell over a grid
 * the size of the screen.
 *
 *   cols, rows   grid dimensions in cells (= the terminal size).
 *   u            excitation per cell, 0 (rest) .. 1 (fully lit). This is the one
 *                that diffuses and that we colour by.
 *   u_next       scratch for u: the diffusion step reads old neighbours, so the
 *                new u is built here and swapped in (see medium_step).
 *   v            recovery per cell, 0 .. 1. Rises behind the front and raises the
 *                firing threshold; does not diffuse.
 *
 * The three arrays are malloc'd once when the size is known (and again on
 * resize). The per-frame update never allocates.
 */
typedef struct {
  int cols, rows;
  float *u;
  float *u_next;
  float *v;
} ExcitableMedium;

/*
 * Scene — the whole run as a table of contents:
 *   WHAT   — the excitable medium (the grid of u, v).
 *   HOW    — substeps: how many Barkley updates per frame (the speed knob).
 *   run    — the pause toggle.
 *   look   — which colour theme is drawn (render-only; the physics ignores it).
 */
typedef struct {
  ExcitableMedium medium;
  int substeps;
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

static inline float clamp01(float x) {
  if (x < 0.0f)
    return 0.0f;
  if (x > 1.0f)
    return 1.0f;
  return x;
}

/* Weighted Laplacian of u at cell (c,r): how much more excited the neighbours
 * are than this cell. Zero-flux edges (a missing neighbour reads as this cell,
 * contributing nothing) reflect the wave off the walls. Vertical neighbours are
 * weighted down to cancel the tall-cell squish. */
static inline float laplacian_u(const ExcitableMedium *m, int c, int r) {
  int cols = m->cols, i = r * cols + c;
  float uc = m->u[i];
  float uW = (c > 0) ? m->u[i - 1] : uc;
  float uE = (c < cols - 1) ? m->u[i + 1] : uc;
  float uN = (r > 0) ? m->u[i - cols] : uc;
  float uS = (r < m->rows - 1) ? m->u[i + cols] : uc;
  return (uE + uW - 2.0f * uc) + (uN + uS - 2.0f * uc) * INV_ASPECT_SQ;
}

/* The fast excitation switch: positive drives u toward "on" (fired), negative
 * toward "off" (rest). The (v+b)/a threshold rises as recovery v builds, which is
 * what forces a fired cell back down and holds it there. */
static inline float barkley_react(float u, float v) {
  float threshold = (v + BARKLEY_B) / BARKLEY_A;
  return u * (1.0f - u) * (u - threshold) / BARKLEY_EPS;
}

/* Excitation 0..1 → brightness band 0..N_SHADES-1: resting cells dark, freshly
 * fired cells bright. */
static int excitation_to_band(float u) {
  int band = (int)(u * (float)N_SHADES);
  if (band < 0)
    band = 0;
  if (band >= N_SHADES)
    band = N_SHADES - 1;
  return band;
}

/* ── §5 SIMULATION — allocate + seed the grid, step it, run the substeps ── */

/* Documented allocation exception: the grid is sized to the screen, so it can't
 * be static; it's malloc'd here at init and on resize, never in the hot loop. */
static bool medium_alloc(ExcitableMedium *m, int cols, int rows) {
  m->cols = cols;
  m->rows = rows;
  size_t n = (size_t)cols * (size_t)rows;
  m->u = calloc(n, sizeof(float));
  m->u_next = calloc(n, sizeof(float));
  m->v = calloc(n, sizeof(float));
  return m->u && m->u_next && m->v;
}

static void medium_free(ExcitableMedium *m) {
  free(m->u);
  free(m->u_next);
  free(m->v);
  m->u = m->u_next = m->v = NULL;
}

/* Lay down the broken wavefront that curls into a spiral: the left half starts
 * fully excited, and the top half starts pre-loaded with recovery so its bit of
 * the front can't advance — leaving one free end to wind around. */
static void medium_seed_spiral(ExcitableMedium *m) {
  for (int r = 0; r < m->rows; r++) {
    for (int c = 0; c < m->cols; c++) {
      int i = r * m->cols + c;
      m->u[i] = (c < m->cols / 2) ? 1.0f : 0.0f;
      m->v[i] = (r < m->rows / 2) ? SEED_BLOCK_V : 0.0f;
    }
  }
}

/* Build the scene from scratch: allocate the grid, seed the spiral, defaults.
 * Runs at startup and on resize. Returns false if the grid couldn't allocate. */
static bool scene_init(Scene *s, int cols, int rows) {
  if (!medium_alloc(&s->medium, cols, rows))
    return false;
  medium_seed_spiral(&s->medium);
  s->substeps = SUBSTEPS_DEFAULT;
  s->paused = false;
  s->theme_idx = 0;
  return true;
}

/* Re-lay the broken front for a fresh spiral (the 'r' key), keeping the grid,
 * speed, and theme. */
static void scene_reset(Scene *s) { medium_seed_spiral(&s->medium); }

/* Advance one cell one time step: new excitation from its own reaction plus what
 * diffuses in from the neighbours, and new recovery chasing the excitation. New u
 * goes into the scratch grid (diffusion still needs the old u); v updates in place
 * since it reads only its own cell. */
static void barkley_update_cell(ExcitableMedium *m, int c, int r) {
  int i = r * m->cols + c;
  float u = m->u[i], v = m->v[i];

  float du = barkley_react(u, v) + DIFFUSION * laplacian_u(m, c, r);
  m->u_next[i] = clamp01(u + SIM_DT * du);
  m->v[i] = clamp01(v + SIM_DT * (u - v));
}

/* Swap the freshly-built u grid in for the old one — the double-buffer flip that
 * let every cell this step read from the same old snapshot. */
static void medium_swap_u(ExcitableMedium *m) {
  float *tmp = m->u;
  m->u = m->u_next;
  m->u_next = tmp;
}

/* Advance the whole grid one fixed time step: update every cell from the old
 * grids, then swap the new excitation grid in. */
static void medium_step(ExcitableMedium *m) {
  for (int r = 0; r < m->rows; r++)
    for (int c = 0; c < m->cols; c++)
      barkley_update_cell(m, c, r);
  medium_swap_u(m);
}

/* Advance the medium by this frame's worth of substeps. While paused it does
 * nothing, but the screen still redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s) {
  if (s->paused)
    return;
  for (int i = 0; i < s->substeps; i++)
    medium_step(&s->medium);
}

/* ── §6 RENDER — grid → screen; reads state, writes only the terminal ── */

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

/* Colour + weight for a shade band: brighter bands are bold so the excited front
 * pops even on 8-colour terminals. */
static attr_t band_attr(int band) {
  attr_t attr = COLOR_PAIR(SHADE_BASE + band);
  if (band >= BOLD_FROM_BAND)
    attr |= A_BOLD;
  return attr;
}

/* Draw the sheet: one glyph per cell, coloured by how excited it is. Resting
 * cells (band 0) are left blank so only the wave shows. */
static void medium_draw(const ExcitableMedium *m) {
  for (int r = 0; r < m->rows; r++) {
    for (int c = 0; c < m->cols; c++) {
      int band = excitation_to_band(m->u[r * m->cols + c]);
      if (band == 0)
        continue;
      attr_t attr = band_attr(band);
      attron(attr);
      mvaddch(r, c, (chtype)(unsigned char)k_shade_glyph[band]);
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

/* Build the top-row status text: fps, speed, theme, paused. */
static void format_hud_status(const Scene *s, double fps, char *buf,
                              size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  speed:%d  [%s] %s ", fps, s->substeps,
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
      " q:quit  spc:pause  r:reseed  t:theme  +/-:speed ";
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

/* Nudge the sim speed (substeps per frame), clamped to a sane range. */
static void scene_adjust_speed(Scene *s, int delta) {
  s->substeps += delta;
  if (s->substeps < SUBSTEPS_MIN)
    s->substeps = SUBSTEPS_MIN;
  if (s->substeps > SUBSTEPS_MAX)
    s->substeps = SUBSTEPS_MAX;
}

/* Rebuild the grid at the new size but keep the user's theme, pause, and speed,
 * which a plain scene_init would have reset. */
static void app_do_resize(App *app) {
  int saved_theme = app->scene.theme_idx;
  bool saved_paused = app->scene.paused;
  int saved_speed = app->scene.substeps;

  medium_free(&app->scene.medium);
  screen_resize(&app->screen);
  if (!scene_init(&app->scene, app->screen.cols, app->screen.rows)) {
    app->running = 0; /* out of memory — bail cleanly */
    return;
  }

  app->scene.theme_idx = saved_theme;
  app->scene.paused = saved_paused;
  app->scene.substeps = saved_speed;
  theme_apply(app->scene.theme_idx);
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
    scene_reset(s);
    break;
  case 't':
  case 'T':
    scene_cycle_theme(s);
    break;

  case '+':
  case '=':
    scene_adjust_speed(s, +1);
    break;
  case '-':
  case '_':
    scene_adjust_speed(s, -1);
    break;

  default:
    break;
  }
  return true;
}

/* The main loop: read keys, advance the medium by this frame's substeps, draw,
 * then sleep to hold ~60 fps. */
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
  if (!scene_init(scene, app->screen.cols, app->screen.rows)) {
    screen_free(&app->screen);
    fprintf(stderr, "wave grid allocation failed\n");
    return 1;
  }
  theme_apply(scene->theme_idx);
  hud_pairs_init();

  const int64_t FRAME_BUDGET_NS = NS_PER_SEC / TARGET_FPS;
  int64_t last_ns = clock_ns();

  while (app->running) {
    if (app->need_resize)
      app_do_resize(app);
    if (!app->running)
      break;

    int64_t frame_start_ns = clock_ns();
    int64_t dt_ns = frame_start_ns - last_ns;
    last_ns = frame_start_ns;

    for (int ch; (ch = getch()) != ERR;) {
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
    }

    scene_tick(scene);
    fps_counter_tick(&app->fps, dt_ns);

    erase();
    medium_draw(&scene->medium);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  medium_free(&scene->medium);
  screen_free(&app->screen);
  return 0;
}
