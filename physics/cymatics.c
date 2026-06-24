/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cymatics.c — Chladni figures: the patterns sand makes on a vibrating plate.
 *
 * Sprinkle sand on a metal plate and play a tone through it. The sand slides
 * off the parts that shake hardest and piles up along the still lines, drawing
 * a clean pattern. This program draws those patterns directly from the formula
 * and slowly morphs from one to the next.
 */

/*
 * How it works
 *
 * There's no physics simulation here — no springs, no time-stepping. The
 * whole pattern comes from one formula. For a plate, a "mode" is one of the
 * shapes it can stand and shake in, named by two whole numbers (m, n) that
 * count how many waves fit across the plate in each direction. The formula
 * gives a number Z at every point:
 *
 *     Z(x, y) = cos(m·pi·x)·cos(n·pi·y) − cos(n·pi·x)·cos(m·pi·y)
 *
 * with x and y running 0..1 across the plate. Where Z is near zero the plate
 * is barely moving, so the sand settles there (those are the lines you see).
 * Where Z is far from zero — strongly positive or strongly negative — the
 * plate is heaving up or down, and sand bounces away. We just colour each
 * cell by how close its Z is to zero; the eye reads the still lines as curves.
 *
 * Two notes on the formula. m must differ from n: if m == n the two cosine
 * products cancel and Z is zero everywhere (a blank plate). And (m, n) and
 * (n, m) draw the same picture, so the mode table lists each pair only once.
 * The pitch a real plate would ring at for mode (m, n) grows with the square
 * root of (m² + n²) — the (3, 4) figure rings at the "5" of a 3-4-5 triangle.
 *
 * The animation is a slow cross-fade: blend the current mode's Z with the
 * next mode's Z as a fade value t walks from 0 (all current) to 1 (all next),
 * then snap to the next mode. Between fades it just holds on one figure.
 *
 * References (the formula here is the simple textbook approximation; real
 * plates obey a stiffer equation, so frequencies differ a little):
 *   • Chladni, E. F. F. (1787) "Entdeckungen über die Theorie des Klanges" —
 *     the original sand-on-plate experiments that named these figures.
 *   • Kirchhoff, G. (1850), J. Reine Angew. Math. 40, 51-88 — the exact
 *     bending equation for a vibrating plate that this formula approximates.
 *   • Rayleigh, "The Theory of Sound" (1894-96), Vol. I §195-227 — plate
 *     modes in depth; every Chladni figure is one such mode.
 *   • Leissa, A. W. (1969) "Vibration of Plates", NASA SP-160 — lookup tables
 *     of mode shapes for every plate shape and edge condition.
 *   • Graff, K. F. (1991) "Wave Motion in Elastic Solids", Dover — clean
 *     derivation of the mode-shape formula used here.
 *   • Morse & Ingard (1968) "Theoretical Acoustics" — where the cosine
 *     building blocks of the formula come from.
 *   • Jenny, H. (1967) "Cymatics" — the photo book that named the field;
 *     every figure on screen has a real-world twin in it.
 *   • Gander & Kwok (2012), SIAM Review 54(3), 573-596 — modern teaching paper
 *     linking Chladni figures to the underlying eigenvalue problem.
 *   • Tuan et al. (2015), J. Acoust. Soc. Am. 137(4), 2113-2123 — modern
 *     experiments checking this approximation against real measurements.
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config ───────────────────────────────────────────────────────── */

#define TICK_NS 33333333LL  /* one tick = ~1/30 s, so the demo runs at 30 fps */
#define HOLD_DEFAULT 120   /* how long to sit on a figure before fading: ~4 s */
#define HOLD_MIN 0         /* shortest hold (+/-): no pause, just keep morphing */
#define HOLD_MAX 300       /* longest hold (+/-): ~10 s to study one figure   */
#define HOLD_STEP 15       /* how much each +/- press changes the hold: ~0.5 s */
#define MORPH_SPEED 0.025f /* fade speed: 0..1 in ~40 ticks, about 1.3 s      */
#define NODAL_THRESH 0.04f /* "near zero" cutoff: closer than this draws the core */

enum { ST_HOLD = 0, ST_MORPH };
enum { CP_POS = 1, CP_NEG, CP_NODE, CP_HUD, CP_HINT };

/*
 * MODES — the list of figures the demo walks through, one per (m, n) pair.
 *
 * Each pair (m, n) picks one figure: m counts how many waves fit across the
 * plate left-to-right, n the same top-to-bottom. We only list pairs where
 * m is smaller than n, for two reasons. If m and n are equal the formula
 * cancels to zero everywhere (a blank plate). And (m, n) and (n, m) draw the
 * exact same lines, so listing both would just repeat the figure.
 *
 * The table holds 40 figures: every pair with m < n up to n = 9 (that's 36),
 * plus four from n = 10 to add some fine, lacy patterns at the end.
 *
 * What they look like as you step through with n/p:
 *   small n (2-4)  : simple, easy to read — (1,2) is a plain cross,
 *                    (3,4) is the famous "star".
 *   middle n (5-7) : more arcs, starts to look woven — close to the figures
 *                    on Chladni's own 1787 plates.
 *   large n (8-10) : a dense lattice of many fine curves. Drop the hold time
 *                    with '+' to morph smoothly through these.
 */
static const int MODES[][2] = {
    /* simple, classic figures */
    {1, 2},
    {1, 3},
    {2, 3},
    {1, 4},
    {2, 4},
    {3, 4},
    {1, 5},
    {2, 5},
    {3, 5},
    {4, 5},
    {1, 6},
    {2, 6},
    {3, 6},
    {4, 6},
    {5, 6},
    /* busier, woven mid-range figures */
    {1, 7},
    {2, 7},
    {3, 7},
    {4, 7},
    {5, 7},
    {6, 7},
    {1, 8},
    {2, 8},
    {3, 8},
    {4, 8},
    {5, 8},
    {6, 8},
    {7, 8},
    {1, 9},
    {2, 9},
    {3, 9},
    {4, 9},
    {5, 9},
    {6, 9},
    {7, 9},
    {8, 9},
    /* dense, lacy showcase figures */
    {1, 10},
    {2, 10},
    {3, 10},
    {4, 10},
};
#define N_MODES (int)(sizeof(MODES) / sizeof(MODES[0]))

#define N_THEMES 4
/* Each theme is two colours: one for the up-shaking areas, one for the down. */
static const short THEMES_256[N_THEMES][2] = {
    {51, 196},  /* Classic:  cyan / red       */
    {45, 30},   /* Ocean:    blue / teal      */
    {202, 160}, /* Ember:  orange / dark-red  */
    {82, 201},  /* Neon:    green / magenta   */
};
static const short THEMES_8[N_THEMES][2] = {
    {COLOR_CYAN, COLOR_RED},
    {COLOR_BLUE, COLOR_CYAN},
    {COLOR_YELLOW, COLOR_RED},
    {COLOR_GREEN, COLOR_MAGENTA},
};
static const char *THEME_NAMES[N_THEMES] = {"Classic", "Ocean", "Ember",
                                            "Neon"};

/* Glyphs from heaviest to lightest: closest to the still line → farthest. */
static const char NODAL_CHARS[] = "@#*+.";
#define N_NCHARS (int)(sizeof(NODAL_CHARS) - 1)

/* ── §2 clock ────────────────────────────────────────────────────────── */

static long long clock_ns(void) {
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

/* ── §3 color ────────────────────────────────────────────────────────── */

/*
 * Set up the colours for one theme. Only the two field colours change with
 * the theme; the white still-lines and the yellow/cyan status bars stay the
 * same no matter which theme is picked, so the text always stays readable.
 */
static void color_apply(int t) {
  short pos, neg;
  if (COLORS >= 256) {
    pos = THEMES_256[t][0];
    neg = THEMES_256[t][1];
  } else {
    pos = THEMES_8[t][0];
    neg = THEMES_8[t][1];
  }
  init_pair(CP_POS, pos, -1);
  init_pair(CP_NEG, neg, -1);
  init_pair(CP_NODE, (COLORS >= 256) ? 231 : COLOR_WHITE, -1);
  init_pair(CP_HUD, (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, (COLORS >= 256) ? 51 : COLOR_CYAN, -1);
}

/* ── §4 field math (pure functions) ──────────────────────────────────── */

/*
 * The core formula: how high the plate stands at point (x, y) for mode (m, n).
 * Near zero means the plate is still there, so sand piles up. See the top of
 * the file for what the formula means.
 */
static float chladni_z(float x, float y, int m, int n) {
  return cosf((float)m * (float)M_PI * x) * cosf((float)n * (float)M_PI * y) -
         cosf((float)n * (float)M_PI * x) * cosf((float)m * (float)M_PI * y);
}

/* ── §5 scene — state struct + mutators ──────────────────────────────── */

/*
 * Scene — everything the running demo needs to know, in one place.
 *
 * Pretty much every function takes a Scene* and works on it. The only state
 * that lives outside is the two signal flags below, which have to be globals
 * because the OS hands a signal to a plain function with no way to pass it
 * the Scene.
 *
 * The fields fall into three groups, ordered by what they do, not by type:
 *   1. which figure is showing and how it fades to the next;
 *   2. the three on/off switches the user controls (auto, pause, theme);
 *   3. the terminal size, re-read when the window is resized.
 *
 * The animation flips between two states: HOLD (sit on the current figure)
 * and MORPH (cross-fade into the next one). Auto-advance decides whether
 * HOLD ever leaves on its own; once a fade finishes, it moves to the next
 * figure and drops back to HOLD.
 */
typedef struct {
  /* Which figure, and the fade to the next.
   *
   * A figure is just an entry in MODES[]; mode_idx says which one. When the
   * demo is in HOLD it sits still; if auto-advance is on, hold_ctr counts up
   * to hold_ticks and then it switches to MORPH. In MORPH the fade value t
   * walks from 0 to 1, blending this figure into the next; at 1 it picks the
   * next figure and goes back to HOLD. The n/p keys also jump mode_idx by
   * hand. hold_ticks is what +/- changes, from "no pause" up to a long dwell. */
  int mode_idx;   /* which figure: 0..N_MODES-1, an index into MODES[]   */
  int state;      /* ST_HOLD (sitting still) or ST_MORPH (fading)        */
  int hold_ctr;   /* how long we've held so far, in ticks; resets each HOLD */
  int hold_ticks; /* how long to hold before fading; kept in [HOLD_MIN, HOLD_MAX] */
  float t;        /* fade progress 0..1: 0 = all current figure, 1 = all next */

  /* The three user switches.
   *
   * Only the keypress handler changes these (through the scene_* helpers),
   * which keeps the key handling a clean table. The tick reads paused and
   * auto_advance; the drawing reads theme. */
  int auto_advance; /* 'a': on = move to the next figure by itself; off = stay put */
  int paused;       /* space: on = freeze everything, even a fade in progress */
  int theme;        /* 't': which colour theme, 0..N_THEMES-1               */

  /* Terminal size.
   *
   * Full terminal size in character cells, status bars included. The field
   * is drawn on the rows in between, leaving the top row for the status bar
   * and the bottom row for the key hints. Re-read only when the window is
   * resized. */
  int rows; /* total rows, including the status/hint rows */
  int cols; /* total columns                             */
} Scene;

/* Starting state: first figure, holding, auto-advance on, first theme. */
static void scene_init(Scene *sc) {
  sc->mode_idx = 0;
  sc->state = ST_HOLD;
  sc->hold_ctr = 0;
  sc->hold_ticks = HOLD_DEFAULT;
  sc->t = 0.0f;
  sc->auto_advance = 1;
  sc->paused = 0;
  sc->theme = 0;
  sc->rows = 0;
  sc->cols = 0;
}

/* Pick up the terminal's current size after a resize. */
static void scene_resize(Scene *sc) {
  int rows, cols;
  getmaxyx(stdscr, rows, cols);
  sc->rows = rows;
  sc->cols = cols;
  erase();
}

/*
 * Move the animation forward by one tick: count down a hold, or push a fade
 * a little further along (and pick the next figure when the fade finishes).
 * Does nothing while paused.
 */
static void scene_tick(Scene *sc) {
  if (sc->paused)
    return;

  if (sc->state == ST_HOLD) {
    if (sc->auto_advance) {
      if (++sc->hold_ctr >= sc->hold_ticks) {
        sc->state = ST_MORPH;
        sc->t = 0.0f;
      }
    }
    return;
  }

  /* ST_MORPH */
  sc->t += MORPH_SPEED;
  if (sc->t >= 1.0f) {
    sc->t = 0.0f;
    sc->mode_idx = (sc->mode_idx + 1) % N_MODES;
    sc->state = ST_HOLD;
    sc->hold_ctr = 0;
  }
}

/* Jump straight to a neighbouring figure (n/p keys), cancelling any fade. */
static void scene_advance_mode(Scene *sc, int dir) {
  sc->mode_idx = ((sc->mode_idx + dir) % N_MODES + N_MODES) % N_MODES;
  sc->state = ST_HOLD;
  sc->hold_ctr = 0;
  sc->t = 0.0f;
}

static void scene_toggle_pause(Scene *sc) { sc->paused ^= 1; }
static void scene_toggle_auto(Scene *sc) { sc->auto_advance ^= 1; }

/* Switch to the next theme and load its colours. */
static void scene_cycle_theme(Scene *sc) {
  sc->theme = (sc->theme + 1) % N_THEMES;
  color_apply(sc->theme);
}

/*
 * Make the pause on each figure longer or shorter (the +/- keys). A positive
 * delta lingers more; negative hurries. Kept inside its limits, where the
 * lowest setting means no pause at all — the figures just keep morphing.
 */
static void scene_adjust_hold(Scene *sc, int delta) {
  sc->hold_ticks += delta;
  if (sc->hold_ticks < HOLD_MIN)
    sc->hold_ticks = HOLD_MIN;
  if (sc->hold_ticks > HOLD_MAX)
    sc->hold_ticks = HOLD_MAX;
}

/*
 * How high the plate stands at one screen cell. During a fade this mixes the
 * current figure with the next one, weighted by how far the fade has gone. If
 * the fade hasn't started we skip the next figure entirely and just use this
 * one, which saves a formula call per cell.
 */
static float z_at_cell(const Scene *sc, int r, int c) {
  float x = (sc->cols > 1) ? (float)c / (float)(sc->cols - 1) : 0.5f;
  float y = (sc->rows > 1) ? (float)r / (float)(sc->rows - 1) : 0.5f;

  int cm = MODES[sc->mode_idx][0], cn = MODES[sc->mode_idx][1];
  float z1 = chladni_z(x, y, cm, cn);
  if (sc->t <= 0.0f)
    return z1;

  int nm = MODES[(sc->mode_idx + 1) % N_MODES][0];
  int nn = MODES[(sc->mode_idx + 1) % N_MODES][1];
  float z2 = chladni_z(x, y, nm, nn);
  return (1.0f - sc->t) * z1 + sc->t * z2;
}

/* ── §6 rendering — band-threshold pipeline + composers ──────────────── */

/*
 * How still does a cell have to be to count as part of a line? We sort cells
 * into five rings by how close they sit to a still line. The first ring is the
 * line itself, drawn brightest; each ring out is a touch livelier and fades a
 * little. Anything past the last ring is shaking too hard and stays blank.
 *   ring 0 (under 0.04): the still line itself — a bright '@'
 *   ring 1 (under 0.10): hugging the line     — a bright '#'
 *   ring 2 (under 0.18): just off it          — a dim '*'
 *   ring 3 (under 0.28): a faint halo         — a dim '+'
 *   ring 4 (under 0.40): the last hint        — a dim '.'
 */
static const float BAND_THRESH[] = {0.04f, 0.10f, 0.18f, 0.28f, 0.40f};

/* Which ring a cell falls in by how far it shakes, or -1 for blank. */
static int band_for_amplitude(float az) {
  for (int b = 0; b < N_NCHARS; b++)
    if (az < BAND_THRESH[b])
      return b;
  return -1;
}

/* The still line is white; everywhere else is coloured by whether the plate
 * there is pushing up or down. */
static int pair_for_band(int band, float z) {
  if (band == 0)
    return CP_NODE;
  return (z > 0.0f) ? CP_POS : CP_NEG;
}

/* The two innermost rings glow bright, like packed sand; the outer ones dim. */
static attr_t attr_for_band(int band, int pair) {
  return (band <= 1) ? (COLOR_PAIR(pair) | A_BOLD) : (COLOR_PAIR(pair) | A_DIM);
}

/* Draw the right character for one cell; leave it blank if it's shaking too
 * hard to belong to any line. */
static void paint_field_cell(const Scene *sc, int r, int c) {
  float z = z_at_cell(sc, r, c);
  float az = fabsf(z);

  int band = band_for_amplitude(az);
  if (band < 0) {
    mvaddch(r, c, ' ');
    return;
  }
  int pair = pair_for_band(band, z);
  attr_t attr = attr_for_band(band, pair);
  attron(attr);
  mvaddch(r, c, (chtype)(unsigned char)NODAL_CHARS[band]);
  attroff(attr);
}

/* Draw the whole pattern, but skip the top and bottom rows so the status bars
 * there stay readable. */
static void render_field(const Scene *sc) {
  for (int r = 1; r < sc->rows - 1; r++)
    for (int c = 0; c < sc->cols - 1; c++)
      paint_field_cell(sc, r, c);
}

/* The top status bar: which figure is showing, what it's fading toward, and
 * all the current settings. */
static void render_hud_top(const Scene *sc) {
  int cm = MODES[sc->mode_idx][0], cn = MODES[sc->mode_idx][1];
  int nm = MODES[(sc->mode_idx + 1) % N_MODES][0];
  int nn = MODES[(sc->mode_idx + 1) % N_MODES][1];
  const char *state_str = (sc->state == ST_MORPH) ? "morphing" : "hold";

  /* Hold time in seconds (TICK_NS is 33.3 ms → 30 fps). */
  double hold_sec = (double)sc->hold_ticks / 30.0;

  char buf[200];
  snprintf(buf, sizeof buf,
           " mode(%d,%d)->(%d,%d)  [%d/%d]  %s t=%.2f  hold:%.1fs  "
           "theme:%s  %s  %s ",
           cm, cn, nm, nn, sc->mode_idx + 1, N_MODES, state_str, (double)sc->t,
           hold_sec, THEME_NAMES[sc->theme], sc->auto_advance ? "auto" : "man.",
           sc->paused ? "PAUSED" : "running");
  int len = (int)strlen(buf);
  int col = sc->cols - len;
  if (col < 0)
    col = 0;

  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, col, buf, sc->cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* The bottom bar listing the keys; falls back to a shorter list on narrow
 * terminals so it doesn't run off the edge. */
static void render_hud_bottom(const Scene *sc) {
  const char *hint_full =
      " q:quit  spc:pause  n/p:mode  t:theme  a:auto  +/-:hold ";
  const char *hint_short = " q:quit  n:mode  t:theme  +/-:hold ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= sc->cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(sc->rows - 1, 0, hint, sc->cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §7 screen + app ─────────────────────────────────────────────────── */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  start_color();
  use_default_colors();
  color_apply(0); /* boot palette; scene_cycle_theme rotates it later */
}

/* Signal-handler flags must be globals — the C signal API has no
 * context pointer, so any per-Scene state can't be reached from here.  */
static volatile sig_atomic_t g_running = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void sig_handler(int sig) {
  if (sig == SIGWINCH)
    g_need_resize = 1;
  else
    g_running = 0;
}
static void cleanup(void) { endwin(); }

/*
 * app_handle_key — route one keypress to the matching scene mutator.
 * Returns 0 on quit, 1 otherwise.
 */
static int app_handle_key(Scene *sc, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
    return 0;
  case ' ':
    scene_toggle_pause(sc);
    break;
  case 'a':
  case 'A':
    scene_toggle_auto(sc);
    break;
  case 'n':
    scene_advance_mode(sc, +1);
    break;
  case 'p':
    scene_advance_mode(sc, -1);
    break;
  case 't':
  case 'T':
    scene_cycle_theme(sc);
    break;
  case '+':
  case '=':
    scene_adjust_hold(sc, -HOLD_STEP);
    break;
  case '-':
  case '_':
    scene_adjust_hold(sc, +HOLD_STEP);
    break;
  }
  return 1;
}

int main(void) {
  signal(SIGINT, sig_handler);
  signal(SIGTERM, sig_handler);
  signal(SIGWINCH, sig_handler);
  atexit(cleanup);

  screen_init();

  Scene scene;
  scene_init(&scene);
  scene_resize(&scene);

  long long next = clock_ns();

  while (g_running) {
    if (g_need_resize) {
      g_need_resize = 0;
      endwin();
      refresh();
      scene_resize(&scene);
    }

    int ch;
    while ((ch = getch()) != ERR) {
      if (!app_handle_key(&scene, ch)) {
        g_running = 0;
        break;
      }
    }

    scene_tick(&scene);
    erase();
    render_field(&scene);
    render_hud_top(&scene);
    render_hud_bottom(&scene);
    wnoutrefresh(stdscr);
    doupdate();

    next += TICK_NS;
    clock_sleep_ns(next - clock_ns());
  }
  return 0;
}
