/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * shader_gallery.c — a fragment-shader playground: every character cell is a pixel,
 * coloured by a pure formula shade(u,v,time). A table of eight shaders (plasma,
 * tunnel, kaleidoscope, swirl, rings, spiral, moiré, warp) gives eight looks;
 * palettes recolour any of them. No geometry — just a formula per cell.
 *
 * Sisters: sun_solar.c (one fixed screen-space effect), wave_interference.c (a
 * per-cell scalar field driven by wave sources instead of a closed-form shader).
 *
 * Keys:  q/ESC quit  space/p pause  [/] shader  t palette  +/- speed  r reset
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/shader_gallery.c -o shader_gallery -lncurses -lm
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

/* ── §1 CONFIG — constants, glyph ramp, palettes ── */

enum {
  TARGET_FPS = 60,
};

enum {
  /* Brightness bands from dark to bright — the resolution of the colour ramp. */
  N_SHADES = 10,
};

#define TAU (2.0f * (float)M_PI)

/* A terminal cell is about twice as tall as wide, so one row-step covers this many
 * column-widths. Stretching the vertical coordinate by it keeps circles round. */
#define CELL_ASPECT 2.0f

/* A tiny nudge on the radius so shaders that divide by it (tunnel, spiral) stay
 * finite at the exact centre instead of blowing up. */
#define CENTER_EPSILON 1e-3f

/* How fast the clock runs (a multiplier on real time), and its adjustable range. */
#define SPEED_DEFAULT 1.0f
#define SPEED_MIN 0.0f
#define SPEED_MAX 4.0f
#define SPEED_STEP 0.25f

/* Longest frame we'll trust. A hiccup (window drag, debugger) can make one frame
 * huge; clamping it stops the animation from lurching forward. */
#define DT_CAP_SEC 0.10f

#define NS_PER_SEC 1000000000LL
#define HUD_BUF_LEN 80

/* ncurses colour-pair slots: N_SHADES ramp bands, then the HUD pairs. */
enum {
  SHADE_BASE = 1, /* bands occupy SHADE_BASE .. SHADE_BASE + N_SHADES - 1 */
  PAIR_HUD = SHADE_BASE + N_SHADES,
  PAIR_HINT,
};

/* Bands this bright or brighter are drawn bold, so highlights pop even on
 * 8-colour terminals that can't show a fine ramp. */
#define BOLD_FROM_BAND 7

/* Glyphs from faint to dense, one per band. Every band has a visible glyph (no
 * blank), so the shader fills the whole screen. */
static const char k_shade_glyph[N_SHADES] = {'.', ':', '-', '~', '+',
                                             '=', '*', 'x', '#', '@'};

/* True when the terminal has the 256-colour palette. Set once at startup. */
static bool g_has_256 = false;

/*
 * Palette — a named colour ramp, dark band → bright band, applied to whichever
 * shader is running (it colours by brightness alone, so any palette fits any
 * shader). 't' cycles the presets and re-tints the pairs in place.
 *
 *   name    label shown in the HUD; never NULL.
 *   fg[]    the N_SHADES ramp colours for a 256-colour terminal, dim → bright, all
 *           in the bright half of the palette (>= 24) so even band 0 stays visible
 *           (see CLAUDE.md "Theme Palette Brightness").
 *   fg_8    one plain ANSI colour for an 8-colour terminal; there the glyph density
 *           carries the gradient and this just sets the hue.
 */
typedef struct {
  const char *name;
  int fg[N_SHADES];
  short fg_8;
} Palette;

static const Palette k_palettes[] = {
    {"magma",
     {24, 53, 90, 126, 163, 199, 205, 211, 217, 231},
     COLOR_MAGENTA},
    {"ocean",
     {24, 25, 31, 38, 45, 51, 87, 123, 159, 195},
     COLOR_CYAN},
    {"jet",
     {24, 27, 39, 45, 51, 48, 46, 190, 214, 196},
     COLOR_BLUE},
    {"heat",
     {52, 88, 124, 160, 196, 202, 208, 214, 220, 231},
     COLOR_RED},
    {"mono",
     {240, 243, 245, 247, 249, 250, 252, 253, 254, 255},
     COLOR_WHITE},
};

#define PALETTE_COUNT (int)(sizeof k_palettes / sizeof k_palettes[0])

/* ── §2 STATE — the domain types (Shader, Palette) + the run harness (Scene, App) ── */

/*
 * Scene — the whole run: which shader is showing, which palette colours it, and the
 * animation clock.
 *
 *   shader_idx    index into the shader table (§4), cycled by [ and ].
 *   palette_idx   index into the palette table, cycled by t.
 *   speed         clock multiplier (0 = frozen animation), nudged by +/-.
 *   time          the animation clock in seconds; every shader reads it.
 *   paused        when true the clock stops (the screen still redraws).
 */
typedef struct {
  int shader_idx;
  int palette_idx;
  float speed;
  float time;
  bool paused;
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

/* ── §4 LOGIC — pure per-cell shaders + band mapping: no mutation, no I/O ── */

/* Distance of a point from the screen centre — the polar radius most shaders use. */
static float radius(float u, float v) { return sqrtf(u * u + v * v); }

/* Angle of a point around the screen centre, in radians — the polar angle. */
static float angle(float u, float v) { return atan2f(v, u); }

/* Sum of four sliding sine waves — the classic demoscene "plasma". The waves run
 * along x, along y, along the diagonal, and outward in rings; added together they
 * make soft blobby colour that flows. */
static float sh_plasma(float u, float v, float t) {
  float s = sinf(u * 2.5f + t);
  s += sinf(v * 2.5f - t * 0.9f);
  s += sinf((u + v) * 1.8f + t * 1.3f);
  s += sinf(radius(u, v) * 3.0f - t * 1.7f);
  return s * 0.125f + 0.5f; /* four sines span −4..4 → 0..1 */
}

/* A flight down a textured tunnel. Working in polar coordinates, 1/radius acts like
 * depth (the centre is infinitely far), so sliding it inward pulls the wall texture
 * toward you; angle stripes wrap it around. */
static float sh_tunnel(float u, float v, float t) {
  float r = radius(u, v) + CENTER_EPSILON;
  float a = angle(u, v);
  float depth = 0.6f / r + t * 0.6f;
  float wall = sinf(depth * TAU) * sinf(a * 5.0f + depth);
  float bright = 0.5f + 0.5f * wall;
  return bright * fminf(r * 1.5f, 1.0f); /* fade the blown-out centre */
}

/* A kaleidoscope: fold the angle into one mirrored wedge so whatever pattern we draw
 * repeats symmetrically around the centre, like a mandala. */
static float sh_kaleido(float u, float v, float t) {
  float r = radius(u, v);
  float a = angle(u, v);
  float wedge = TAU / 6.0f;
  a = fabsf(fmodf(a + TAU * 8.0f, wedge) - wedge * 0.5f); /* fold + mirror */
  float fu = cosf(a) * r, fv = sinf(a) * r;
  float s = sinf(fu * 6.0f + t) + sinf(fv * 6.0f - t * 0.7f);
  return s * 0.25f + 0.5f;
}

/* A whirlpool: twist the coordinates by an angle that grows with distance from the
 * centre, then draw a checker in the twisted space — straight lines come out as
 * spirals. */
static float sh_swirl(float u, float v, float t) {
  float r = radius(u, v);
  float a = angle(u, v) + r * 3.0f - t; /* more twist further out */
  float fu = cosf(a) * r, fv = sinf(a) * r;
  return sinf(fu * 7.0f) * sinf(fv * 7.0f) * 0.5f + 0.5f;
}

/* Sonar: concentric rings that pulse outward from the centre — brightness is just a
 * sine of the distance, sliding with the clock. */
static float sh_rings(float u, float v, float t) {
  float r = radius(u, v);
  return 0.5f + 0.5f * sinf(r * TAU * 3.0f - t * 3.0f);
}

/* A rotating spiral galaxy. Log of the radius turns the arms into a logarithmic
 * spiral (arms that keep the same tightness as they wind out); combined with the
 * angle and the clock they sweep around. */
static float sh_spiral(float u, float v, float t) {
  float r = radius(u, v) + CENTER_EPSILON;
  float a = angle(u, v);
  return 0.5f + 0.5f * sinf(a * 4.0f + logf(r) * 5.0f - t * 2.0f);
}

/* Moiré: two fine grids laid over each other, one slowly rotating. Where their lines
 * nearly line up you get big soft interference bands that crawl as it turns. */
static float sh_moire(float u, float v, float t) {
  float g1 = sinf(u * 12.0f) * sinf(v * 12.0f);
  float ca = cosf(t * 0.3f), sa = sinf(t * 0.3f);
  float ru = u * ca - v * sa, rv = u * sa + v * ca; /* rotate the 2nd grid */
  float g2 = sinf(ru * 12.0f) * sinf(rv * 12.0f);
  return (g1 + g2) * 0.25f + 0.5f;
}

/* Domain warp: shove the coordinates around with a couple of sine waves BEFORE
 * feeding them to the plasma. Distorting a pattern's input like this is what gives
 * that liquid, marbled look. */
static float sh_warp(float u, float v, float t) {
  float wu = sinf(v * 2.0f + t);
  float wv = sinf(u * 2.0f - t * 0.8f);
  return sh_plasma(u + wu, v + wv, t * 0.7f);
}

/* A shader is a pure function of position + time; a table of them is the gallery. */
typedef float (*ShaderFn)(float u, float v, float t);

typedef struct {
  const char *name;
  ShaderFn fn;
} Shader;

static const Shader k_shaders[] = {
    {"plasma", sh_plasma}, {"tunnel", sh_tunnel}, {"kaleido", sh_kaleido},
    {"swirl", sh_swirl},   {"rings", sh_rings},   {"spiral", sh_spiral},
    {"moire", sh_moire},   {"warp", sh_warp},
};

#define SHADER_COUNT (int)(sizeof k_shaders / sizeof k_shaders[0])

/* Brightness 0..1 → glyph/colour band 0..N_SHADES-1 (clamped: a few shaders can
 * push a little past the ends). */
static int value_to_band(float value) {
  int band = (int)(value * (float)N_SHADES);
  if (band < 0)
    band = 0;
  if (band >= N_SHADES)
    band = N_SHADES - 1;
  return band;
}

/* ── §5 SIMULATION — set up the scene, advance the clock ── */

static void scene_init(Scene *s) {
  s->shader_idx = 0;
  s->palette_idx = 0;
  s->speed = SPEED_DEFAULT;
  s->time = 0.0f;
  s->paused = false;
}

/* Advance the animation clock. While paused it does nothing, but the screen still
 * redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  s->time += dt * s->speed;
}

/* ── §6 RENDER — run the shader over the grid, colour it, HUD ── */

/* Colour + weight for a band: brighter bands are bold so highlights pop even on
 * 8-colour terminals. */
static attr_t band_attr(int band) {
  attr_t attr = COLOR_PAIR(SHADE_BASE + band);
  if (band >= BOLD_FROM_BAND)
    attr |= A_BOLD;
  return attr;
}

/* Map a cell column to a centred shader x: 0 in the middle, growing to the edges. */
static float cell_to_u(int col, float cx, float inv_half) {
  return ((float)col - cx) * inv_half;
}

/* Map a cell row to a centred shader y, stretched by CELL_ASPECT so a shader's
 * circles come out round on the 2×-tall character grid. */
static float cell_to_v(int row, float cy, float inv_half) {
  return ((float)row - cy) * inv_half * CELL_ASPECT;
}

/* Run the current shader at every cell and stamp its brightness. */
static void shader_draw(const Scene *s, int cols, int rows) {
  ShaderFn shade = k_shaders[s->shader_idx].fn;
  float cx = cols * 0.5f, cy = rows * 0.5f;
  float inv_half = 1.0f / cy;
  for (int row = 0; row < rows; row++) {
    float v = cell_to_v(row, cy, inv_half);
    for (int col = 0; col < cols; col++) {
      float u = cell_to_u(col, cx, inv_half);
      int band = value_to_band(shade(u, v, s->time));
      attr_t attr = band_attr(band);
      attron(attr);
      mvaddch(row, col, (chtype)(unsigned char)k_shade_glyph[band]);
      attroff(attr);
    }
  }
}

/* Re-tint the ramp bands for the chosen palette. HUD pairs are left alone so they
 * keep their colour whatever palette is up. */
static void palette_apply(int idx) {
  const Palette *p = &k_palettes[idx];
  for (int b = 0; b < N_SHADES; b++)
    init_pair(SHADE_BASE + b, g_has_256 ? p->fg[b] : p->fg_8, COLOR_BLACK);
}

/* Set up the HUD colours once. The -1 background sits the HUD on the terminal's
 * real background instead of a black box. */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
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

/* Build the top-row status text: fps, shader, palette, speed, paused. */
static void format_hud_status(const Scene *s, double fps, char *buf,
                              size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  [%s]  pal:%s  x%.2f  %s ", fps,
           k_shaders[s->shader_idx].name, k_palettes[s->palette_idx].name,
           (double)s->speed, s->paused ? "PAUSED " : "running");
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
      " q:quit  spc:pause  [/]:shader  t:palette  +/-:speed  r:reset ";
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

/* Step the shader selection, wrapping both ways. */
static void scene_cycle_shader(Scene *s, int delta) {
  s->shader_idx = (s->shader_idx + delta + SHADER_COUNT) % SHADER_COUNT;
}

static void scene_cycle_palette(Scene *s) {
  s->palette_idx = (s->palette_idx + 1) % PALETTE_COUNT;
  palette_apply(s->palette_idx);
}

/* Nudge the clock speed, clamped to a sane range. */
static void scene_adjust_speed(Scene *s, float delta) {
  s->speed += delta;
  if (s->speed < SPEED_MIN)
    s->speed = SPEED_MIN;
  if (s->speed > SPEED_MAX)
    s->speed = SPEED_MAX;
}

/* Re-read the size on resize. The picture is recomputed from scratch each frame, so
 * there is no grid to rebuild — the shader just fills the new dimensions. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
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
    s->time = 0.0f;
    break;
  case 't':
  case 'T':
    scene_cycle_palette(s);
    break;

  case ']':
    scene_cycle_shader(s, +1);
    break;
  case '[':
    scene_cycle_shader(s, -1);
    break;

  case '+':
  case '=':
    scene_adjust_speed(s, SPEED_STEP);
    break;
  case '-':
  case '_':
    scene_adjust_speed(s, -SPEED_STEP);
    break;

  default:
    break;
  }
  return true;
}

/* The main loop: each pass measures real elapsed time, reads keys, advances the
 * clock by that much, draws, then sleeps to hold ~60 fps. */
int main(void) {
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
  scene_init(scene);
  palette_apply(scene->palette_idx);
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
    shader_draw(scene, app->screen.cols, app->screen.rows);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
