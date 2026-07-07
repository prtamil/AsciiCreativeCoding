/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * matrix_vortex.c — Matrix-style green glyphs spiralling down a whirlpool.
 * Each stream falls inward while turning around a central '@' drain — doing
 * both at once is what bends its straight path into a spiral — and the whole
 * picture rotates slowly.
 *
 * Sisters: sun_rain.c (streams fly straight out), hyperspace_rain.c (streams
 * rush at you with depth). All three share the same stream + shimmer code.
 *
 * Keys:  q/ESC quit   space/p pause   r reset   t theme
 * Build: gcc -std=c11 -O2 -Wall -Wextra matrix_rain/matrix_vortex.c -o matrix_vortex -lncurses -lm
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

/* ── §1 CONFIG — constants, colour slots, the theme + glyph tables ── */

enum {
  TARGET_FPS = 60,
};

enum {
  /* How many streams swirl into the drain. A few hundred reads as a solid
   * whirlpool; fewer looks like distinct spiral arms. */
  N_STREAMS = 160,

  /* How many glyphs long each stream's tail is. */
  STREAM_TRAIL = 18,

  /* Where the brightness bands fall along a tail (see dist_attr): the first few
   * cells are bright, the middle normal, the rest dim. */
  TRAIL_HOT_END = 2,
  TRAIL_WARM_END = STREAM_TRAIL / 2,
};

/* Terminal cells are about twice as tall as they are wide, so a circle drawn
 * naively comes out as a tall ellipse. We squish the vertical direction by this
 * much to keep the whirlpool round. */
#define ASPECT 0.45f

/* How many full turns a stream winds through on its way from the rim to the
 * centre. This is the "swirliness": 0 = falls straight in, bigger = tighter
 * spiral arms. */
#define TWIST_TURNS 1.6f

/* How fast the whole whirlpool rotates, in turns per second. Slow on purpose —
 * the falling motion carries the eye; the spin just keeps it alive. */
#define SPIN_RPS 0.05f

/* Cells kept clear around the centre so streams never paint over the '@' drain,
 * which sits at distance 0. */
#define CORE_RESERVED_RADIUS 1.5f

/* Each stream picks a random inward speed in this range when it spawns, so some
 * dive fast while others drift, and they don't all arrive together. */
#define SPEED_MIN_CPS 22.0f
#define SPEED_MAX_CPS 55.0f

/* How far past the rim a recycled stream starts, as a fraction of max_r. Starting
 * beyond the edge (and staggered) means the whirlpool refills smoothly instead of
 * every stream popping in at once. */
#define STAGGER_FRAC 0.60f

/* Each frame, every glyph in a tail has a 1-in-4 chance of staying put; the rest
 * reroll. That flicker is the Matrix-rain shimmer. */
#define SHIMMER_KEEP_ONE_IN 4

/* ncurses colour-pair slots. 1..5 are the theme-tinted tail bands; head and core
 * are always white; HUD pairs never change with theme. */
enum {
  SHADE_FADE = 1,
  SHADE_DARK,
  SHADE_MID,
  SHADE_BRIGHT,
  SHADE_HOT,
  SHADE_HEAD,
  SHADE_CORE,
  PAIR_HUD,
  PAIR_HINT,
};

/* Longest frame we'll trust. A hiccup (window drag, debugger pause) can make one
 * frame huge; clamping it stops the sim from lurching forward. */
#define DT_CAP_SEC 0.10f

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

#define HUD_BUF_LEN 72

/*
 * Theme — one named colour scheme for the stream trails.
 *
 * A tail fades from a bright head to a dim tip across 5 brightness bands.
 * dist_attr (§4) decides which band each cell falls in; this struct just holds
 * the colour for each band, brightest last. The 't' key cycles the presets and
 * theme_apply re-tints the pairs in place, so streams already on screen adopt the
 * new look instantly.
 *
 * Two palettes: rich terminals get a fine 256-colour ramp, old 8-colour ones fall
 * back to fg_8 (the nearest plain ANSI colour). The head and core aren't here —
 * they're always white so they stay sharp against any theme.
 *
 *   name    label shown in the HUD ("green", "nova", …); never NULL.
 *   fg[]    5 colours for a 256-colour terminal, dimmest to brightest, all in the
 *           bright half of the palette (>= 24) so even the dimmest band stays
 *           visible on black (see CLAUDE.md "Theme Palette Brightness").
 *   fg_8[]  same 5 bands for an 8-colour terminal.
 */
typedef struct {
  const char *name;
  int fg[5];
  int fg_8[5];
} Theme;

static const Theme k_themes[] = {
    {"green",
     {28, 34, 40, 46, 82},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN}},
    {"nova",
     {24, 33, 51, 159, 255},
     {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE}},
    {"plasma",
     {53, 57, 93, 129, 201},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA}},
    {"fire",
     {52, 88, 124, 160, 196},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED}},
    {"solar",
     {130, 166, 202, 214, 220},
     {COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* True when the terminal has the 256-colour palette. Set once at startup, then
 * read by the colour setup to pick the fine ramp or the 8-colour fallback. */
static bool g_has_256 = false;

/* The pool of glyphs a stream can show. */
static const char k_glyphs[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                               "abcdefghijklmnopqrstuvwxyz"
                               "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

/* ── §2 STATE — the domain types (Stream → Vortex) + the run harness (Scene, App) ── */

/*
 * VortexGeometry — where the drain sits and the shape of the spiral, worked out
 * from the screen size at startup (and again on resize).
 *
 *   cx, cy   the drain's centre, in screen cells (middle of the screen).
 *   max_r    the rim: how far out a stream starts, set generously past the real
 *            corner so streams always sweep in from off-screen.
 *   twist    how sharply the arm curls — how much its angle turns for each cell
 *            the head moves inward, set so it makes TWIST_TURNS full turns all
 *            the way from the rim to the centre.
 */
typedef struct {
  int cx, cy;
  float max_r;
  float twist;
} VortexGeometry;

/*
 * Stream — one glowing head spiralling in toward the drain, trailing glyphs.
 *
 * It lives on a spiral arm fixed at birth: `phase` says where on the rim the arm
 * starts, and everything else about the arm's shape comes from the shared twist.
 * Only `r` (how far the head still is from the centre) changes as it falls. When
 * the whole tail has drained past the centre, the stream is recycled onto a fresh
 * random arm, so the whirlpool flows forever with no visible loop.
 *
 *   phase    the arm's starting angle on the rim, in radians. Fixed for one life,
 *            re-rolled on recycle so arms don't repeat.
 *   r        the head's distance from the centre, in cells (a float, for smooth
 *            sub-cell motion). Shrinks by speed × dt each frame; goes slightly
 *            negative as the tail finishes draining.
 *   speed    inward cells per second; fixed for one life, re-rolled on recycle so
 *            the streams don't all pulse in unison.
 *   cache[]  the glyphs this stream currently shows; cache[0] is the bright head,
 *            the rest trail behind it. About 75% reroll each frame (the shimmer).
 *
 * The whole array of streams lives in fixed storage — no malloc anywhere.
 */
typedef struct {
  float phase;
  float r;
  float speed;
  char cache[STREAM_TRAIL];
} Stream;

/*
 * Vortex — the whirlpool itself: the WHAT the simulation advances. It bundles
 * the whirling streams, the space they spiral through, and the one rotation they
 * all share.
 *
 *   streams   the streams spiralling into the drain.
 *   geom      the drain's centre and the spiral's shape.
 *   spin      the shared rotation angle, nudged a little every frame.
 */
typedef struct {
  Stream streams[N_STREAMS];
  VortexGeometry geom;
  float spin;
} Vortex;

/*
 * Scene — the whole run, read as a table of contents:
 *   WHAT  — the vortex (its streams, geometry, and spin).
 *   run   — the pause toggle.
 *   look  — which colour theme is drawn (render-only; the physics ignores it).
 * Orchestrators (init / reset / tick) take the whole Scene; everything else
 * takes the narrowest piece (a Stream, the Vortex, the geometry).
 */
typedef struct {
  Vortex vortex;
  bool paused;   /* frozen? toggled by space/p */
  int theme_idx; /* colour scheme, cycled by t */
} Scene;

/*
 * Screen — the terminal's size in cells, plus the ncurses lifecycle. The
 * geometry is derived from this size; the drawing code uses it to clip glyphs
 * that fall off the edge. Refreshed on every resize.
 *
 *   cols   width in cells.
 *   rows   height in cells. Row 0 holds the status line; row rows-1 the hints.
 */
typedef struct {
  int cols, rows;
} Screen;

/*
 * FpsCounter — a smoothed frames-per-second readout. Measuring one frame at a
 * time would jump around, so this averages over a short window.
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
 * App — everything the program keeps alive for one run.
 *
 *   scene         the streams and their state.
 *   screen        the terminal size and ncurses handle.
 *   fps           the smoothed fps readout for the HUD.
 *   running       set to 0 to quit (by 'q' or Ctrl-C).
 *   need_resize   set when the window changed; handled next frame.
 *
 * g_app is the only global; the signal handlers need to reach these two flags,
 * and a signal handler can't be handed a pointer. The flags are sig_atomic_t for
 * the same reason — they're poked from a signal.
 */
typedef struct {
  Scene scene;
  Screen screen;
  FpsCounter fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

/* ── §3 PERFORMANCE — clock + fps counter (the frame cap is in §8 main) ── */

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

/* Picks the colour + boldness for a tail cell from how far back it is. The head
 * (i=0) is white and bright; cells get dimmer toward the tip. */
static attr_t dist_attr(int i) {
  if (i == 0)
    return COLOR_PAIR(SHADE_HEAD) | A_BOLD;
  if (i == 1)
    return COLOR_PAIR(SHADE_HOT) | A_BOLD;
  if (i <= TRAIL_HOT_END)
    return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
  if (i <= TRAIL_WARM_END)
    return COLOR_PAIR(SHADE_MID);
  if (i <= STREAM_TRAIL - 2)
    return COLOR_PAIR(SHADE_DARK);
  return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* True while any part of the stream is still short of fully draining. Once the
 * head has gone a whole tail-length past the centre, the tail is gone too. */
static inline bool stream_still_draining(const Stream *s) {
  return s->r > -(float)STREAM_TRAIL;
}

/* The angle a point at `radius` on this stream's arm is drawn at: the arm's own
 * winding plus the shared whirlpool spin. Winds more the closer to the centre. */
static inline float stream_angle_at(const Stream *s, const VortexGeometry *g,
                                    float radius, float spin) {
  return s->phase + g->twist * (g->max_r - radius) + spin;
}

/* Turns "this far out, at this angle" into a screen cell. The ·ASPECT on the row
 * squishes the vertical so the swirl stays round, not egg-shaped. */
static inline void polar_to_screen_cell(const VortexGeometry *g, float radius,
                                        float angle, int *out_col, int *out_row) {
  *out_col = g->cx + (int)roundf(radius * cosf(angle));
  *out_row = g->cy + (int)roundf(radius * sinf(angle) * ASPECT);
}

/* ── §5 EFFECTS — the cosmetic glyph shimmer (never affects motion) ── */

/* A random glyph from the pool. Purely cosmetic — which character shows never
 * changes how a stream moves. */
static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }

/* Rolls fresh glyphs into roughly 75% of the tail; the rest stay put. That mix
 * of changing and steady cells is what makes it shimmer. Called from the tick. */
static inline void shimmer_reroll(Stream *s) {
  for (int i = 0; i < STREAM_TRAIL; i++)
    if (rand() % SHIMMER_KEEP_ONE_IN != 0)
      s->cache[i] = rand_glyph();
}

/* ── §6 SIMULATION — advances the streams + the spin (the per-frame tick) ── */

static float urand01(void) { return (float)rand() / (float)RAND_MAX; }

/* Works out the drain's centre, the rim distance, and the per-cell twist from the
 * screen size. Called at startup and on every resize. */
static void compute_vortex_geometry(VortexGeometry *g, int cols, int rows) {
  g->cx = cols / 2;
  g->cy = rows / 2;
  g->max_r = (float)cols + (float)rows / ASPECT;
  g->twist = TWIST_TURNS * 2.0f * (float)M_PI / g->max_r;
}

/* Sends a stream back out to the rim with a fresh arm, speed, and glyphs. It
 * starts a little past the edge (staggered) so it sweeps in smoothly. */
static void stream_reset(Stream *s, const VortexGeometry *g) {
  s->phase = urand01() * 2.0f * (float)M_PI;
  s->r = g->max_r + urand01() * STAGGER_FRAC * g->max_r;
  s->speed = SPEED_MIN_CPS + urand01() * (SPEED_MAX_CPS - SPEED_MIN_CPS);
  for (int i = 0; i < STREAM_TRAIL; i++)
    s->cache[i] = rand_glyph();
}

/* Like stream_reset, but spreads the head somewhere across the whole radius so
 * the very first frame already shows a full whirlpool instead of an empty rim. */
static void stream_spawn_spread(Stream *s, const VortexGeometry *g) {
  stream_reset(s, g);
  s->r = urand01() * g->max_r;
}

/* Moves a stream one frame: head falls inward, glyphs shimmer. Returns false once
 * it has fully drained (caller recycles it). */
static bool stream_tick(Stream *s, float dt) {
  s->r -= s->speed * dt;
  shimmer_reroll(s);
  return stream_still_draining(s);
}

/* Moves one stream forward, and if it has fully drained, sends it back to the
 * rim (fresh arm, speed, and delay). */
static inline void advance_one_stream_with_recycle(Stream *s,
                                                   const VortexGeometry *g,
                                                   float dt) {
  if (!stream_tick(s, dt))
    stream_reset(s, g);
}

/* Builds the whole scene from scratch: geometry, all the streams spread across
 * the whirlpool, and the default toggles. Runs at startup and on resize. */
static void scene_init(Scene *s, int cols, int rows) {
  compute_vortex_geometry(&s->vortex.geom, cols, rows);
  for (int i = 0; i < N_STREAMS; i++)
    stream_spawn_spread(&s->vortex.streams[i], &s->vortex.geom);
  s->vortex.spin = 0.0f;
  s->paused = false;
  s->theme_idx = 0;
}

/* Restarts every stream (the 'r' key) for a fresh whirlpool, without touching the
 * geometry. */
static void scene_reset(Scene *s) {
  for (int i = 0; i < N_STREAMS; i++)
    stream_spawn_spread(&s->vortex.streams[i], &s->vortex.geom);
  s->vortex.spin = 0.0f;
}

/* Turns the whole whirlpool a little further this frame, wrapped back under one
 * full turn so the angle can't grow without bound (and lose float precision)
 * over a long run. */
static inline void vortex_spin_advance(Vortex *v, float dt) {
  v->spin += SPIN_RPS * 2.0f * (float)M_PI * dt;
  if (v->spin > 2.0f * (float)M_PI)
    v->spin -= 2.0f * (float)M_PI;
}

/* Advances the whole whirlpool one frame: turn the shared spin a little, then
 * move every stream. While paused it does nothing, but the screen still redraws
 * so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  vortex_spin_advance(&s->vortex, dt);

  for (int i = 0; i < N_STREAMS; i++)
    advance_one_stream_with_recycle(&s->vortex.streams[i], &s->vortex.geom, dt);
}

/* ── §7 RENDER — Scene → screen; reads state, writes only the terminal ── */

/* Re-tints the tail bands plus head/core for the chosen theme. The HUD pairs are
 * left alone on purpose — they must stay the same colour whatever theme is up. */
static void theme_apply(int idx) {
  const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
  init_pair(SHADE_FADE, fg[0], COLOR_BLACK);
  init_pair(SHADE_DARK, fg[1], COLOR_BLACK);
  init_pair(SHADE_MID, fg[2], COLOR_BLACK);
  init_pair(SHADE_BRIGHT, fg[3], COLOR_BLACK);
  init_pair(SHADE_HOT, fg[4], COLOR_BLACK);
  init_pair(SHADE_HEAD, COLOR_WHITE, COLOR_BLACK);
  init_pair(SHADE_CORE, COLOR_WHITE, COLOR_BLACK);
}

/* Sets up the HUD colours once at startup. The -1 background means the HUD sits
 * on the terminal's real background instead of a black box. */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* Draws one tail cell, skipping it quietly if it lands off-screen. */
static inline void paint_trail_cell(const Stream *s, int slot_i, int col, int row,
                                    int cols, int rows) {
  if (col < 0 || col >= cols || row < 0 || row >= rows)
    return;
  attr_t attr = dist_attr(slot_i);
  attron(attr);
  mvaddch(row, col, (chtype)(unsigned char)s->cache[slot_i]);
  attroff(attr);
}

/* Draws one stream head-first along its spiral arm. Cell i sits one step farther
 * out than the head; cells inside the reserved core are skipped so the '@' stays
 * clean, and cells past the rim end the loop. */
static void stream_draw(const Stream *s, const VortexGeometry *g, float spin,
                        int cols, int rows) {
  for (int i = 0; i < STREAM_TRAIL; i++) {
    float radius = s->r + (float)i;
    if (radius < CORE_RESERVED_RADIUS)
      continue; /* skip cells inside the drain */
    if (radius > g->max_r)
      break; /* the rest of the tail is out past the rim */

    float angle = stream_angle_at(s, g, radius, spin);
    int col, row;
    polar_to_screen_cell(g, radius, angle, &col, &row);
    paint_trail_cell(s, i, col, row, cols, rows);
  }
}

/* Stamps the '@' drain at the centre. Drawn after the streams so nothing covers
 * it. Needs only where the centre is, so it takes the geometry. */
static void vortex_draw_core(const VortexGeometry *g, int cols, int rows) {
  int cx = g->cx, cy = g->cy;
  if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
    return;
  attron(COLOR_PAIR(SHADE_CORE) | A_BOLD);
  mvaddch(cy, cx, '@');
  attroff(COLOR_PAIR(SHADE_CORE) | A_BOLD);
}

/* Draws the whole whirlpool — every stream, then the '@' drain last so it always
 * shows. Takes just the Vortex; it never needs the pause flag or the theme. */
static void vortex_draw(const Vortex *v, int cols, int rows) {
  for (int i = 0; i < N_STREAMS; i++)
    stream_draw(&v->streams[i], &v->geom, v->spin, cols, rows);

  vortex_draw_core(&v->geom, cols, rows);
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

/* Prints one coloured, bold line — shared by both HUD rows. */
static inline void hud_paint_text(int row, int col, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  mvprintw(row, col, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Builds the top-row status text: fps, stream count, theme, paused state. */
static void format_hud_status(const Scene *s, double fps, char *buf,
                              size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  streams:%d  [%s] %s ", fps, N_STREAMS,
           k_themes[s->theme_idx].name, s->paused ? "PAUSED " : "running");
}

/* Paints the status text flush against the right edge of row 0. */
static void draw_hud_status(const Screen *sc, const Scene *s, double fps) {
  enum { HUD_TOP_ROW = 0 };
  char buf[HUD_BUF_LEN];
  format_hud_status(s, fps, buf, sizeof buf);
  int right_col = sc->cols - (int)strlen(buf);
  if (right_col < 0)
    right_col = 0;
  hud_paint_text(HUD_TOP_ROW, right_col, PAIR_HUD, buf);
}

/* Paints the key-list strip along the bottom row. */
static void draw_hud_hint(const Screen *sc) {
  static const char *KEY_HINT = " q:quit  spc:pause  r:reset  t:theme ";
  hud_paint_text(sc->rows - 1, 0, PAIR_HINT, KEY_HINT);
}

/* Draws both HUD rows: status up top, key hints along the bottom. */
static void screen_draw_hud(const Screen *sc, double fps, const Scene *s) {
  draw_hud_status(sc, s, fps);
  draw_hud_hint(sc);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 APP — key & resize handling, plus the main loop ── */

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

/* Rebuilds the scene at the new size but keeps the user's theme and pause state,
 * which a plain scene_init would have reset. */
static void scene_reinit_preserving_knobs(Scene *s, int new_cols, int new_rows) {
  int saved_theme = s->theme_idx;
  bool saved_paused = s->paused;

  scene_init(s, new_cols, new_rows);

  s->theme_idx = saved_theme;
  s->paused = saved_paused;
}

/* Handles a window resize: re-read the size, rebuild the scene to fit. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_reinit_preserving_knobs(&app->scene, app->screen.cols,
                                app->screen.rows);
  app->need_resize = 0;
}

/* Acts on one keypress. Returns false only when the user wants to quit. */
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

  default:
    break;
  }
  return true;
}

/* The main loop: each pass measures real elapsed time, reads keys, advances the
 * whirlpool by that much, draws, then sleeps to hold ~60 fps. */
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

    /* Handle a resize first so the rest of the frame uses the new size. */
    if (app->need_resize) {
      app_do_resize(app);
      last_ns = clock_ns();
    }

    /* How long the last frame really took (clamped so a long stall doesn't make
     * the streams jump). */
    int64_t now_ns = clock_ns();
    int64_t dt_ns = now_ns - last_ns;
    last_ns = now_ns;
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
    vortex_draw(&scene->vortex, app->screen.cols, app->screen.rows);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    /* Sleep out the rest of this frame's time budget so we don't spin. */
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
