/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * electric_potential_field.c — the voltage map around drifting point charges.
 * A few + and − charges float around; every cell is tinted by the voltage there —
 * warm near +, cool near −, dark where they cancel. The tint steps in bands, so the
 * band edges trace the equipotential contours, writhing as the charges drift.
 *
 * Sisters: magnetic_fields.c (same "sum every source" idea drawn as vector field
 * lines; here it's a filled scalar map), marching_squares_isocontours_showcase.c
 * (contour lines of a scalar field).
 *
 * Keys:  q/ESC quit   space/p pause   r reset   t theme   +/- charges
 * Build: gcc -std=c11 -O2 -Wall -Wextra procedural/fields/electric_potential_field.c -o electric_potential_field -lncurses -lm
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

/* ── §1 CONFIG — constants, colour ramps, themes ── */

enum {
  TARGET_FPS = 60,
};

enum {
  /* How many charges can float at once, and the default on startup. */
  MIN_CHARGES = 2,
  MAX_CHARGES = 8,
  DEFAULT_CHARGES = 4,

  /* Shade bands per polarity: how many rings of brightness from near-zero
   * voltage out to a charge's bright core. */
  N_LEVELS = 6,
};

#define TWO_PI (2.0f * (float)M_PI)

/* A terminal cell is about twice as tall as wide, so one row-step covers this
 * many column-widths of real distance. Without it the round contour rings would
 * render as tall ellipses. */
#define CELL_ASPECT 2.0f

/* Distance floor: right on a charge the true 1/distance is infinite, so we never
 * let distance drop below this — it caps the core to a finite bright spot. */
#define SOFTEN 1.5f

/* Sets how quickly voltage ramps to full brightness. Voltage is squashed by
 * V/(|V|+V_HALF); smaller = the field lights up further from the charges (fills
 * more of the screen). Tuned so the equipotential rings reach well out without
 * washing the cores or closing up the dark cancellation gaps. */
#define V_HALF 0.25f

/* Charge strengths are ±1 or ±2 (alternating sign, random magnitude), giving a
 * mix of stronger and weaker hills and wells. */
#define CHARGE_MAG_MIN 1.0f
#define CHARGE_MAG_MAX 2.0f

/* How fast charges drift, in cells per second. Slow on purpose so the field
 * morphs gently and stays readable. */
#define DRIFT_MIN_CPS 1.5f
#define DRIFT_MAX_CPS 4.0f

/* Longest frame we'll trust. A hiccup (window drag, debugger) can make one frame
 * huge; clamping it stops the charges from jumping. */
#define DT_CAP_SEC 0.10f

#define NS_PER_SEC 1000000000LL
#define HUD_BUF_LEN 80

/* ncurses colour-pair slots: a cool ramp for negative voltage, a warm ramp for
 * positive, then the fixed HUD + charge-marker pairs. */
enum {
  NEG_BASE = 1,                    /* NEG_BASE .. NEG_BASE + N_LEVELS - 1 */
  POS_BASE = NEG_BASE + N_LEVELS,  /* POS_BASE .. POS_BASE + N_LEVELS - 1 */
  PAIR_HUD = POS_BASE + N_LEVELS,
  PAIR_HINT,
  PAIR_MARK, /* the + / − markers stamped on each charge */
};

/* Levels this bright or brighter are drawn bold, so cores pop on 8-colour
 * terminals that can't show a fine ramp. */
#define BOLD_FROM_LEVEL 4

/* Glyphs by level, near-zero → charge core. Level 0 is a space (voltage ≈ 0 reads
 * as empty), so it's never actually drawn. */
static const char k_level_glyph[N_LEVELS] = {' ', '.', ':', '+', '*', '#'};

/* True when the terminal has the 256-colour palette. Set once at startup. */
static bool g_has_256 = false;

/*
 * Theme — a diverging colour scheme: a cool ramp for negative voltage and a warm
 * one for positive, each running dim (near zero) → bright (charge core).
 *
 *   name     label shown in the HUD; never NULL.
 *   neg[]    the N_LEVELS cool-ramp colours for a 256-colour terminal.
 *   pos[]    the N_LEVELS warm-ramp colours for a 256-colour terminal.
 *            All entries sit in the bright half of the palette (>= 24) so even the
 *            dim end stays visible (see CLAUDE.md "Theme Palette Brightness").
 *   neg_8    one plain ANSI colour for negative voltage on an 8-colour terminal;
 *   pos_8    ditto for positive. Brightness there comes from bold on high levels.
 */
typedef struct {
  const char *name;
  int neg[N_LEVELS];
  int pos[N_LEVELS];
  short neg_8;
  short pos_8;
} Theme;

static const Theme k_themes[] = {
    {"redblue",
     {24, 25, 27, 33, 39, 45},
     {52, 88, 124, 160, 196, 203},
     COLOR_BLUE,
     COLOR_RED},
    {"coolwarm",
     {24, 31, 38, 44, 51, 123},
     {94, 130, 166, 202, 214, 220},
     COLOR_CYAN,
     COLOR_YELLOW},
    {"greenmagenta",
     {53, 90, 127, 164, 201, 207},
     {24, 28, 34, 70, 76, 82},
     COLOR_MAGENTA,
     COLOR_GREEN},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* ── §2 STATE — the domain types (Charge, ChargeField) + the run harness (Scene, App) ── */

/*
 * Charge — one point charge floating across the field.
 *
 *   x, y     position in screen cells.
 *   vx, vy   drift velocity in cells per second; bounces off the screen edges.
 *   q        signed strength: positive charges raise the voltage nearby, negative
 *            ones lower it. Magnitude 1..2.
 */
typedef struct {
  float x, y;
  float vx, vy;
  float q;
} Charge;

/*
 * ChargeField — the whole set of charges. The voltage isn't stored anywhere; it's
 * recomputed from these each frame (see potential_at).
 *
 *   charges[]    every possible charge, all pre-placed; only the first n_charges
 *                of them count.
 *   n_charges    how many are live right now, MIN_CHARGES..MAX_CHARGES (+/- change
 *                it).
 */
typedef struct {
  Charge charges[MAX_CHARGES];
  int n_charges;
} ChargeField;

/*
 * Scene — the whole run as a table of contents:
 *   WHAT  — the charge field.
 *   run   — the pause toggle.
 *   look  — which diverging theme is drawn (render-only; the physics ignores it).
 */
typedef struct {
  ChargeField field;
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

/* One charge's contribution at a cell: its strength ÷ distance (plus charges add,
 * minus subtract). Distance is stretched vertically so the rings stay round, and
 * floored at SOFTEN so the charge's own cell doesn't blow up to infinity. */
static float charge_potential_at(const Charge *ch, float cx, float cy) {
  float dx = cx - ch->x;
  float dy = (cy - ch->y) * CELL_ASPECT;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < SOFTEN)
    dist = SOFTEN;
  return ch->q / dist;
}

/* The summed voltage at one cell: add up every charge's contribution. This one
 * function IS the field — the potential is just the sum over the charges. */
static float potential_at(const ChargeField *f, float cx, float cy) {
  float v = 0.0f;
  for (int i = 0; i < f->n_charges; i++)
    v += charge_potential_at(&f->charges[i], cx, cy);
  return v;
}

/* Squash a voltage to −1..+1 so the picture auto-scales with the charge count. */
static float potential_normalized(float v) { return v / (fabsf(v) + V_HALF); }

/* How far from zero a normalized voltage sits, as a shade band 0..N_LEVELS-1:
 * near-zero → 0 (blank), near a charge → the bright top band. */
static int potential_level(float n) {
  int level = (int)(fabsf(n) * (float)N_LEVELS);
  if (level < 0)
    level = 0;
  if (level >= N_LEVELS)
    level = N_LEVELS - 1;
  return level;
}

/* ── §5 SIMULATION — place and drift the charges ── */

static float urand01(void) { return (float)rand() / (float)RAND_MAX; }

/* Place charge i at a random spot, drifting a random way, with an ALTERNATING
 * sign (even index +, odd −) so the field always has a mix of hills and wells. */
static void charge_place(Charge *c, int index, int cols, int rows) {
  c->x = urand01() * (float)cols;
  c->y = urand01() * (float)rows;

  float angle = urand01() * TWO_PI;
  float speed = DRIFT_MIN_CPS + urand01() * (DRIFT_MAX_CPS - DRIFT_MIN_CPS);
  c->vx = cosf(angle) * speed;
  c->vy = sinf(angle) * speed;

  float mag = CHARGE_MAG_MIN + urand01() * (CHARGE_MAG_MAX - CHARGE_MAG_MIN);
  c->q = (index % 2 == 0) ? mag : -mag;
}

/* Re-place every charge. Used at startup, on reset (r), and on resize (a smaller
 * window would otherwise leave charges hanging off the edge). */
static void field_scatter_charges(ChargeField *f, int cols, int rows) {
  for (int i = 0; i < MAX_CHARGES; i++)
    charge_place(&f->charges[i], i, cols, rows);
}

/* Reflect a coordinate off a [lo,hi] wall: if it crossed, pin it back to the wall
 * and flip its velocity so it bounces straight back in. */
static void reflect_off_wall(float *pos, float *vel, float lo, float hi) {
  if (*pos < lo) {
    *pos = lo;
    *vel = -*vel;
  }
  if (*pos > hi) {
    *pos = hi;
    *vel = -*vel;
  }
}

/* Move one charge a frame and bounce it off the screen edges. */
static void charge_drift(Charge *c, float dt, int cols, int rows) {
  c->x += c->vx * dt;
  c->y += c->vy * dt;
  reflect_off_wall(&c->x, &c->vx, 0.0f, (float)cols);
  reflect_off_wall(&c->y, &c->vy, 0.0f, (float)rows);
}

/* Build the field from scratch: default count, charges scattered, defaults. Runs
 * at startup and on resize. */
static void scene_init(Scene *s, int cols, int rows) {
  s->field.n_charges = DEFAULT_CHARGES;
  field_scatter_charges(&s->field, cols, rows);
  s->paused = false;
  s->theme_idx = 0;
}

/* Re-scatter the charges for a fresh field (the 'r' key), keeping the count and
 * theme. */
static void scene_reset(Scene *s, int cols, int rows) {
  field_scatter_charges(&s->field, cols, rows);
}

/* Drift every live charge by this frame's elapsed time. While paused it does
 * nothing, but the screen still redraws so the HUD keeps ticking. */
static void scene_tick(Scene *s, float dt, int cols, int rows) {
  if (s->paused)
    return;
  for (int i = 0; i < s->field.n_charges; i++)
    charge_drift(&s->field.charges[i], dt, cols, rows);
}

/* ── §6 RENDER — field → screen; reads state, writes only the terminal ── */

/* Re-tint both voltage ramps for the chosen theme. HUD/marker pairs are left
 * alone so they keep their colour whatever theme is up. */
static void theme_apply(int idx) {
  const Theme *th = &k_themes[idx];
  for (int i = 0; i < N_LEVELS; i++) {
    init_pair(NEG_BASE + i, g_has_256 ? th->neg[i] : th->neg_8, COLOR_BLACK);
    init_pair(POS_BASE + i, g_has_256 ? th->pos[i] : th->pos_8, COLOR_BLACK);
  }
}

/* Set up the HUD + marker colours once. The -1 background sits the HUD on the
 * terminal's real background instead of a black box. */
static void hud_pairs_init(void) {
  init_pair(PAIR_HUD, g_has_256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, g_has_256 ? 51 : COLOR_CYAN, -1);
  init_pair(PAIR_MARK, COLOR_WHITE, -1);
}

/* Colour + weight for a cell: the warm ramp for positive voltage, the cool ramp
 * for negative, and bold on the bright levels so the cores pop. */
static attr_t voltage_attr(float n, int level) {
  int base = (n >= 0.0f) ? POS_BASE : NEG_BASE;
  attr_t attr = COLOR_PAIR(base + level);
  if (level >= BOLD_FROM_LEVEL)
    attr |= A_BOLD;
  return attr;
}

/* Draw the voltage map: one glyph per cell, warm ramp for positive voltage, cool
 * for negative. Near-zero cells (level 0) are left blank so only the field shows. */
static void field_draw(const ChargeField *f, int cols, int rows) {
  for (int row = 0; row < rows; row++) {
    for (int col = 0; col < cols; col++) {
      float n = potential_normalized(potential_at(f, (float)col, (float)row));
      int level = potential_level(n);
      if (level == 0)
        continue;
      attr_t attr = voltage_attr(n, level);
      attron(attr);
      mvaddch(row, col, (chtype)(unsigned char)k_level_glyph[level]);
      attroff(attr);
    }
  }
}

/* Stamp a + or − on each charge's cell so the sources are easy to spot. Drawn on
 * top of the field. */
static void charge_markers_draw(const ChargeField *f, int cols, int rows) {
  attron(COLOR_PAIR(PAIR_MARK) | A_BOLD);
  for (int i = 0; i < f->n_charges; i++) {
    int cx = (int)(f->charges[i].x + 0.5f);
    int cy = (int)(f->charges[i].y + 0.5f);
    if (cx < 0 || cx >= cols || cy < 0 || cy >= rows)
      continue;
    mvaddch(cy, cx, (f->charges[i].q >= 0.0f) ? '+' : '-');
  }
  attroff(COLOR_PAIR(PAIR_MARK) | A_BOLD);
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

/* Build the top-row status text: fps, charges, theme, paused. */
static void format_hud_status(const Scene *s, double fps, char *buf,
                              size_t buflen) {
  snprintf(buf, buflen, " %5.1f fps  charges:%d  [%s] %s ", fps,
           s->field.n_charges, k_themes[s->theme_idx].name,
           s->paused ? "PAUSED " : "running");
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
      " q:quit  spc:pause  r:reset  t:theme  +/-:charges ";
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

/* Add or drop a charge, clamped to the allowed range. Extra charges are always
 * pre-placed, so adding one just reveals the next already-scattered charge. */
static void field_adjust_charges(ChargeField *f, int delta) {
  f->n_charges += delta;
  if (f->n_charges < MIN_CHARGES)
    f->n_charges = MIN_CHARGES;
  if (f->n_charges > MAX_CHARGES)
    f->n_charges = MAX_CHARGES;
}

/* Rebuild the field at the new size but keep the user's theme, pause, and charge
 * count, which a plain scene_init would have reset. */
static void app_do_resize(App *app) {
  int saved_theme = app->scene.theme_idx;
  bool saved_paused = app->scene.paused;
  int saved_charges = app->scene.field.n_charges;

  screen_resize(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  app->scene.theme_idx = saved_theme;
  app->scene.paused = saved_paused;
  app->scene.field.n_charges = saved_charges;
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
    field_adjust_charges(&s->field, +1);
    break;
  case '-':
  case '_':
    field_adjust_charges(&s->field, -1);
    break;

  default:
    break;
  }
  return true;
}

/* The main loop: each pass measures real elapsed time, reads keys, drifts the
 * charges by that much, draws, then sleeps to hold ~60 fps. */
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

    scene_tick(scene, dt, app->screen.cols, app->screen.rows);
    fps_counter_tick(&app->fps, dt_ns);

    erase();
    field_draw(&scene->field, app->screen.cols, app->screen.rows);
    charge_markers_draw(&scene->field, app->screen.cols, app->screen.rows);
    screen_draw_hud(&app->screen, app->fps.display, scene);
    screen_present();

    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(FRAME_BUDGET_NS - elapsed);
  }

  screen_free(&app->screen);
  return 0;
}
