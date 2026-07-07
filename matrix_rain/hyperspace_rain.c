/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * hyperspace_rain.c — Matrix-style glyphs flying at you through a warp tunnel.
 * Streaks start as faint specks at the centre and rush out to the edges as they
 * "come closer" — dividing by distance makes near ones move fast and spread wide
 * while far ones bunch up small in the middle, which reads as 3-D depth.
 *
 * Sisters: sun_rain.c (flat, no depth), matrix_vortex.c (streams spiral inward).
 * All three share the same stream + shimmer code.
 *
 * Keys:  q/ESC quit   space/p pause   r reset   t theme
 * Build: gcc -std=c11 -O2 -Wall -Wextra matrix_rain/hyperspace_rain.c -o hyperspace_rain -lncurses -lm
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
  /* How many streaks fill the tunnel. A couple hundred reads as a dense flight
   * through code without choking a slow terminal. */
  N_STREAKS = 220,

  /* How many glyphs long each streak is. */
  STREAK_TRAIL = 14,

  /* Where the brightness bands fall along a streak (see dist_attr): the first
   * few cells are bright, the middle normal, the rest dim. */
  TRAIL_HOT_END = 2,
  TRAIL_WARM_END = STREAK_TRAIL / 2,
};

/* Terminal cells are about twice as tall as wide, so a circle drawn naively
 * comes out as a tall ellipse. Squish the vertical to keep the tunnel round. */
#define ASPECT 0.45f

/* The "focal length": on-screen distance = PROJ_SCALE / depth. Bigger opens the
 * tunnel mouth wider (everything spreads out sooner). */
#define PROJ_SCALE 2.5f

/* Depth a streak spawns at (far away). It ticks down toward the near limit. */
#define DEPTH_FAR 1.0f

/* Depth gap between one glyph and the next along a streak. Because on-screen
 * distance is scale / depth, a fixed depth gap spreads the glyphs farther apart
 * the nearer the streak is — which is what makes a streak stretch as it flies in. */
#define DEPTH_STEP 0.02f

/* How many brightness bands a streak at the FAR distance is dimmed by; it
 * brightens back to a white head as it approaches. Keeps the crowded centre
 * from turning into one bright blur. */
#define DEPTH_DIM_CELLS 6

/* Each streak picks a random approach speed (depth units per second) at spawn,
 * so some rush and some drift and they don't all reach the edge together. */
#define SPEED_MIN 0.16f
#define SPEED_MAX 0.50f

/* Each frame, every glyph has a 1-in-4 chance of staying put; the rest reroll.
 * That flicker is the Matrix-rain shimmer. */
#define SHIMMER_KEEP_ONE_IN 4

/* ncurses colour-pair slots. 1..5 are the theme-tinted bands; the head is always
 * white; HUD pairs never change with theme. */
enum {
  SHADE_FADE = 1,
  SHADE_DARK,
  SHADE_MID,
  SHADE_BRIGHT,
  SHADE_HOT,
  SHADE_HEAD,
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
 * Theme — one named colour scheme for the streaks.
 *
 * A streak fades from a bright head to a dim tip across 5 brightness bands.
 * dist_attr (§4) decides which band each cell falls in; this struct just holds
 * the colour for each band, brightest last. The 't' key cycles the presets and
 * theme_apply re-tints the pairs in place, so streaks already on screen adopt the
 * new look instantly.
 *
 * Two palettes: rich terminals get a fine 256-colour ramp, old 8-colour ones fall
 * back to fg_8 (the nearest plain ANSI colour). The head isn't here — it's always
 * white so it stays sharp against any theme.
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

/* The pool of glyphs a streak can show. */
static const char k_glyphs[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                               "abcdefghijklmnopqrstuvwxyz"
                               "0123456789!@#$%^&*()-_=+[]{}|;:,.<>?/~`";

#define GLYPHS_LEN (int)(sizeof k_glyphs - 1)

/* ── §2 STATE — the domain types (Streak → Starfield) + the run harness (Scene, App) ── */

/*
 * TunnelGeometry — the vanishing point and the near limit, worked out from the
 * screen size at startup (and again on resize).
 *
 *   cx, cy       the vanishing point, in screen cells (middle of the screen).
 *   depth_near   once a streak's depth drops below this, its head is already off
 *                the screen edge and it gets recycled. Set from PROJ_SCALE and
 *                the screen size, so it retires right as it leaves the corner.
 */
typedef struct {
  int cx, cy;
  float depth_near;
} TunnelGeometry;

/*
 * Streak — one line of glyphs flying out from the centre toward the viewer.
 *
 * Its direction is baked at birth (cos/sin, with the cell-squish folded into
 * sin), so the draw loop is just multiply-add — no trig per cell. Only `depth`
 * changes over its life; the on-screen distance is worked out from it. When it
 * flies off the edge it's recycled onto a new random direction at the far depth,
 * so the tunnel flows forever with no visible loop.
 *
 *   cos_a, sin_a   which way this streak points, set once at birth. sin_a has
 *                  the ASPECT squish folded in.
 *   depth          how far away the head is (a float). Big = far (near centre),
 *                  small = near (out at the edge). Shrinks by speed × dt.
 *   speed          depth units per second; fixed for one life, re-rolled on
 *                  recycle so streaks don't all pulse in unison.
 *   cache[]        the glyphs this streak shows; cache[0] is the bright head,
 *                  the rest trail back toward the centre. ~75% reroll each frame.
 *
 * The whole array of streaks lives in fixed storage — no malloc anywhere.
 */
typedef struct {
  float cos_a, sin_a;
  float depth;
  float speed;
  char cache[STREAK_TRAIL];
} Streak;

/*
 * Starfield — the flight itself: the WHAT the simulation advances. It bundles
 * the flying streaks and the tunnel they project through. (A "starfield" is the
 * demoscene name for this perspective-streak flight.)
 *
 *   streaks   the streaks flying out toward the viewer.
 *   geom      the vanishing point and the near limit they retire at.
 */
typedef struct {
  Streak streaks[N_STREAKS];
  TunnelGeometry geom;
} Starfield;

/*
 * Scene — the whole run, read as a table of contents:
 *   WHAT  — the starfield (its streaks and tunnel geometry).
 *   run   — the pause toggle.
 *   look  — which colour theme is drawn (render-only; the physics ignores it).
 * Orchestrators (init / reset / tick) take the whole Scene; everything else
 * takes the narrowest piece (a Streak, the Starfield, the geometry).
 */
typedef struct {
  Starfield starfield;
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
 *   scene         the streaks and their state.
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

/* Picks the colour + boldness for a streak cell from how far back it is. The
 * head (band 0) is white and bright; cells get dimmer toward the tail. Far
 * streaks pass in a raised band, so their whole body starts out dim. */
static attr_t dist_attr(int i) {
  if (i == 0)
    return COLOR_PAIR(SHADE_HEAD) | A_BOLD;
  if (i == 1)
    return COLOR_PAIR(SHADE_HOT) | A_BOLD;
  if (i <= TRAIL_HOT_END)
    return COLOR_PAIR(SHADE_BRIGHT) | A_BOLD;
  if (i <= TRAIL_WARM_END)
    return COLOR_PAIR(SHADE_MID);
  if (i <= STREAK_TRAIL - 2)
    return COLOR_PAIR(SHADE_DARK);
  return COLOR_PAIR(SHADE_FADE) | A_DIM;
}

/* On-screen distance from the centre for a point at this depth — the perspective
 * divide. Closer (smaller depth) lands farther out. */
static inline float depth_to_radius(float depth) { return PROJ_SCALE / depth; }

/* How many brightness bands to push a streak down by, from how far it still is:
 * a streak at the far distance is dimmed the most and brightens as it nears. */
static inline int streak_depth_dim(const Streak *s) {
  return (int)((s->depth / DEPTH_FAR) * (float)DEPTH_DIM_CELLS);
}

/* Depth of the i-th glyph back along a streak: each cell sits one DEPTH_STEP
 * deeper than the one ahead of it, so the tail trails back toward the centre. */
static inline float streak_cell_depth(const Streak *s, int slot_i) {
  return s->depth + (float)slot_i * DEPTH_STEP;
}

/* Turns "this far out, in this baked direction" into a screen cell. No sin/cos
 * here — the direction (with the ASPECT squish folded into sin_a) was baked at
 * birth, so it's just multiply, add, round. */
static inline void polar_to_screen_cell(int cx, int cy, float radius,
                                        float cos_a, float sin_a, int *out_col,
                                        int *out_row) {
  *out_col = cx + (int)roundf(radius * cos_a);
  *out_row = cy + (int)roundf(radius * sin_a);
}

/* ── §5 EFFECTS — the cosmetic glyph shimmer (never affects motion) ── */

/* A random glyph from the pool. Purely cosmetic — which character shows never
 * changes how a streak moves. */
static char rand_glyph(void) { return k_glyphs[rand() % GLYPHS_LEN]; }

/* Rolls fresh glyphs into roughly 75% of the streak; the rest stay put. That mix
 * of changing and steady cells is what makes it shimmer. Called from the tick. */
static inline void shimmer_reroll(Streak *s) {
  for (int i = 0; i < STREAK_TRAIL; i++)
    if (rand() % SHIMMER_KEEP_ONE_IN != 0)
      s->cache[i] = rand_glyph();
}

/* ── §6 SIMULATION — advances the streaks (the per-frame tick) ── */

static float urand01(void) { return (float)rand() / (float)RAND_MAX; }

/* Works out the vanishing point and the near limit from the screen size. The
 * near limit is the depth at which a streak's head just reaches the far corner,
 * so it retires right as it leaves the screen. Called at startup and on resize. */
static void compute_tunnel_geometry(TunnelGeometry *g, int cols, int rows) {
  g->cx = cols / 2;
  g->cy = rows / 2;
  float max_r = (float)cols + (float)rows / ASPECT;
  g->depth_near = PROJ_SCALE / max_r;
}

/* Sends a streak back to the far distance, pointing a fresh random way, with a
 * new speed and glyphs. */
static void streak_reset(Streak *s) {
  float angle = urand01() * 2.0f * (float)M_PI;
  s->cos_a = cosf(angle);
  s->sin_a = sinf(angle) * ASPECT; /* squish the vertical here, once */
  s->depth = DEPTH_FAR;
  s->speed = SPEED_MIN + urand01() * (SPEED_MAX - SPEED_MIN);
  for (int i = 0; i < STREAK_TRAIL; i++)
    s->cache[i] = rand_glyph();
}

/* Like streak_reset, but drops the streak somewhere across the whole depth range
 * so the first frame already shows a full tunnel instead of an empty centre. */
static void streak_spawn_spread(Streak *s, const TunnelGeometry *g) {
  streak_reset(s);
  s->depth = g->depth_near + urand01() * (DEPTH_FAR - g->depth_near);
}

/* Moves a streak one frame: it comes a little closer, glyphs shimmer. Returns
 * false once it has passed the near limit (flown off the edge). */
static bool streak_tick(Streak *s, const TunnelGeometry *g, float dt) {
  s->depth -= s->speed * dt;
  shimmer_reroll(s);
  return s->depth > g->depth_near;
}

/* Moves one streak forward, and if it has flown off the edge, sends it back to
 * the far distance (new direction, speed, and glyphs). */
static inline void advance_one_streak_with_recycle(Streak *s,
                                                   const TunnelGeometry *g,
                                                   float dt) {
  if (!streak_tick(s, g, dt))
    streak_reset(s);
}

/* Builds the whole scene from scratch: geometry, all the streaks spread through
 * the tunnel, and the default toggles. Runs at startup and on resize. */
static void scene_init(Scene *s, int cols, int rows) {
  compute_tunnel_geometry(&s->starfield.geom, cols, rows);
  for (int i = 0; i < N_STREAKS; i++)
    streak_spawn_spread(&s->starfield.streaks[i], &s->starfield.geom);
  s->paused = false;
  s->theme_idx = 0;
}

/* Restarts every streak (the 'r' key) for a fresh tunnel, without touching the
 * geometry. */
static void scene_reset(Scene *s) {
  for (int i = 0; i < N_STREAKS; i++)
    streak_spawn_spread(&s->starfield.streaks[i], &s->starfield.geom);
}

/* Advances every streak one frame. While paused it does nothing, but the screen
 * still redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;

  for (int i = 0; i < N_STREAKS; i++)
    advance_one_streak_with_recycle(&s->starfield.streaks[i], &s->starfield.geom,
                                    dt);
}

/* ── §7 RENDER — Scene → screen; reads state, writes only the terminal ── */

/* Re-tints the streak bands plus the head for the chosen theme. The HUD pairs
 * are left alone — they must stay the same colour whatever theme is up. */
static void theme_apply(int idx) {
  const int *fg = g_has_256 ? k_themes[idx].fg : k_themes[idx].fg_8;
  init_pair(SHADE_FADE, fg[0], COLOR_BLACK);
  init_pair(SHADE_DARK, fg[1], COLOR_BLACK);
  init_pair(SHADE_MID, fg[2], COLOR_BLACK);
  init_pair(SHADE_BRIGHT, fg[3], COLOR_BLACK);
  init_pair(SHADE_HOT, fg[4], COLOR_BLACK);
  init_pair(SHADE_HEAD, COLOR_WHITE, COLOR_BLACK);
}

/* Sets up the HUD colours once at startup. The -1 background means the HUD sits
 * on the terminal's real background instead of a black box. */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
}

/* Draws one streak cell, skipping it quietly if it lands off-screen. The depth
 * dim shifts the whole streak down the brightness bands, so a far streak starts
 * dim and brightens as it flies in. */
static inline void paint_streak_cell(const Streak *s, int slot_i, int dim,
                                     int col, int row, int cols, int rows) {
  if (col < 0 || col >= cols || row < 0 || row >= rows)
    return;
  attr_t attr = dist_attr(slot_i + dim);
  attron(attr);
  mvaddch(row, col, (chtype)(unsigned char)s->cache[slot_i]);
  attroff(attr);
}

/* Draws one streak: head-first, one glyph per depth step back toward the centre.
 * Off-screen cells are skipped quietly (a near streak's head is often past the
 * edge while its tail is still in view). */
static void streak_draw(const Streak *s, const TunnelGeometry *g, int cols,
                        int rows) {
  int dim = streak_depth_dim(s);
  for (int i = 0; i < STREAK_TRAIL; i++) {
    float radius = depth_to_radius(streak_cell_depth(s, i));
    int col, row;
    polar_to_screen_cell(g->cx, g->cy, radius, s->cos_a, s->sin_a, &col, &row);
    paint_streak_cell(s, i, dim, col, row, cols, rows);
  }
}

/* Draws the whole flight — every streak. Takes just the Starfield; it never
 * needs the pause flag or the theme. */
static void starfield_draw(const Starfield *sf, int cols, int rows) {
  for (int i = 0; i < N_STREAKS; i++)
    streak_draw(&sf->streaks[i], &sf->geom, cols, rows);
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

/* Builds the top-row status text: fps, streak count, theme, paused state. */
static void format_hud_status(const Scene *s, double fps, char *buf,
                              size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  streaks:%d  [%s] %s ", fps, N_STREAKS,
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
 * streaks by that much, draws, then sleeps to hold ~60 fps. */
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
     * the streaks jump). */
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
    starfield_draw(&scene->starfield, app->screen.cols, app->screen.rows);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    /* Sleep out the rest of this frame's time budget so we don't spin. */
    int64_t elapsed = clock_ns() - now_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
