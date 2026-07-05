/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * kelvin_helmholtz.c — the "breaking wave" instability, in the terminal.
 *
 * Two streams of fluid slide past each other; the seam between them is unstable,
 * so the tiniest ripple curls up into a row of spirals — the "cat's-eyes" you
 * see in breaking waves and long rolling cloud bands. We track the seam as a
 * chain of little whirlpools and let them swirl each other; the roll-up is all
 * their own doing.
 *
 * Sister file: fluid/vorticity_streamfunction_solver.c (the Karman street, on a
 * full grid). The ideas: Rosenhead 1931 (the seam as a row of point vortices)
 * and Krasny 1986 (the "blob" softening that keeps the roll-up smooth).
 *
 * Keys:  q quit  space pause  r reset  n/p preset  t/T theme  +/- speed  ]/[ sim-Hz
 * Build: gcc -std=c11 -O2 -Wall -Wextra fluid/kelvin_helmholtz.c -o kelvin_helmholtz -lncurses -lm
 */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 CONFIG — the tweakable numbers, the presets, the colour themes ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,

  SPEED_MIN = 1,
  SPEED_DEF = 8,
  SPEED_MAX = 64,

  MAX_VORTICES = 400, /* the fixed pool size; presets use up to this many */

  FPS_UPDATE_MS = 500,

  /* ncurses numbers each colour pair. 1 and 2 are the HUD's — same in every
   * demo in this project, so the HUD always looks alike. */
  PAIR_HUD = 1,
  PAIR_HINT = 2,
  PAIR_SHEET_BASE = 3, /* eight sheet colours 0..7 — cycles once per billow */
  PAIR_SKY = 11,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

#define TWO_PI (2.0f * (float)M_PI)

/* Each tick is split into this many tiny hops. The whirlpools pull hard on each
 * other, so a couple of smaller hops keeps the motion smooth instead of jerky. */
#define SUBSTEPS 2

/* Neighbours this close on screen (in cells) get joined by a drawn line, so the
 * sheet reads as one continuous curve. Farther apart = a stretched or wrapped
 * gap, which we leave as separate dots. */
#define CONNECT_MAX_CELLS 3.0f

/* A hair of random wobble on the seed ripple, so the billows aren't identical
 * clones — real ones never are. */
#define SEED_NOISE_CELLS 0.08f

/* A gentle pull toward the middle row, per second. On its own a rolled-up sheet
 * grows forever (spirals wind ever taller) and drifts off-screen. Real Kelvin-
 * Helmholtz billows don't: buoyancy (heavier fluid below) pulls the interface
 * back, and the rolls settle into steady, endlessly-spinning cat's-eyes. This is
 * that pull — enough to keep the billows on screen, weak enough to let them roll
 * up first. Set it to 0 and the sheet grows without end. */
#define CONFINE_RATE 0.4f

/* Safety net only: because of CONFINE_RATE the billows stay put, so this reset
 * essentially never fires. If the sheet somehow grew past this fraction of the
 * screen height, we'd restart it from a fresh flat line. */
#define RESET_SPREAD_FRAC 0.90f

/* Frame timing (main loop):
 *   MAX_CATCHUP_MS — if a frame took ages (paused in a debugger, laptop asleep),
 *                    pretend only this much time passed, so the sim doesn't try
 *                    to replay the whole gap at once and lock up.
 *   FRAME_CAP_FPS  — how many frames a second we draw, separate from the sim's
 *                    own Hz setting. */
#define MAX_CATCHUP_MS 100
#define FRAME_CAP_FPS 60

/* Which of the three presets the player has picked — an index into the
 * shear_layers table below (scene_seed turns the chosen one into a live sheet). */
typedef enum {
  PATTERN_GENTLE = 0, /* two big lazy rolls */
  PATTERN_CLASSIC = 1,
  PATTERN_CHOPPY = 2, /* many small quick rolls */
  N_PATTERNS = 3,
} Pattern;

/*
 * ShearLayer — the recipe for one roll-up: the physical setup plus how finely we
 * model it. scene_seed() turns one of these into a live VortexSheet; the three
 * presets differ only in these numbers.
 *
 *   n_vortices : how many points make up the sheet. More = a smoother, finer
 *                curve that can wind into tighter spirals before it looks dotty.
 *   billows    : how many waves we bend the starting line into — and so how many
 *                spirals you end up with.
 *   shear      : how fast the two streams slide past each other, in cells/sec.
 *                Bigger = the rolls form faster and spin harder.
 *   seed_amp   : how tall the starting ripple is, in cells. A tiny bump is
 *                enough; bigger just means it curls sooner.
 *   blob       : the softening radius, in cells (see add_vortex_swirl). Small =
 *                tighter, sharper spirals but twitchier; large = fat and calm.
 */
typedef struct {
  int n_vortices;
  int billows;
  float shear;
  float seed_amp;
  float blob;
} ShearLayer;

static const ShearLayer shear_layers[N_PATTERNS] = {
    /* GENTLE  */ {200, 2, 9.0f, 0.6f, 1.4f},
    /* CLASSIC */ {260, 4, 12.0f, 0.5f, 1.2f},
    /* CHOPPY  */ {320, 6, 15.0f, 0.4f, 1.0f},
};

/*
 * Theme — one colour scheme for the sheet. sheet[8] runs faint to vivid, and the
 * eight colours cycle once per billow, so a finished spiral shows colour rings.
 * Every colour sits in the bright half of the palette, so even the faint end
 * stays visible against a black terminal.
 */
typedef struct {
  const char *name;
  short sheet[8]; /* faint to vivid */
  short sky;
} Theme;

#define N_THEMES 6

static const Theme themes[N_THEMES] = {
    /* name, sheet colours (faint to vivid), sky */
    {"OCEAN", {24, 31, 38, 45, 51, 87, 159, 195}, 233},
    {"PLASMA", {54, 91, 128, 165, 201, 207, 213, 219}, 233},
    {"FIRE", {52, 88, 130, 166, 202, 208, 214, 226}, 233},
    {"ICE", {60, 66, 73, 110, 152, 159, 195, 231}, 233},
    {"FOREST", {22, 28, 34, 70, 106, 148, 184, 226}, 233},
    {"MONO", {242, 245, 247, 249, 251, 252, 254, 255}, 232},
};

/* The characters the sheet is drawn with, faint tiers to bright — a heavier
 * blob of ink for the vivid rings, a light speck for the faint ones. */
static const char SHEET_GLYPHS[8] = {'.', ':', '-', '+', '*', 'o', '#', '@'};

/* ── §2 STATE — the data types the rest of the file works on ── */

/*
 * Vortex — one idealised whirlpool: a point on the seam that spins the fluid
 * around itself. Its array index is its place along the seam, so point i is the
 * neighbour of point i+1 — that ordering is what lets us draw the seam as one
 * connected curve.
 *
 *   x, y  : position in cells. x runs 0..width and wraps around (the flow repeats
 *           left-right); y grows downward from the middle row. Stored un-wrapped
 *           so neighbours stay numerically next to each other; only wrapped when
 *           drawing.
 *   phase : a 0..1 label set once at birth, from where the point started along
 *           the seam. It picks the colour and cycles once per billow, so each
 *           finished spiral shows colour rings. Never changes.
 */
typedef struct {
  float x, y;
  float phase;
} Vortex;

/*
 * VortexSheet — the whole seam, stored as a chain of point vortices: the points
 * themselves plus the three constants that set how they swirl one another. The
 * array order is the curve — point i next to point i+1 — so it also draws as one
 * connected line. (This "seam as a row of vortices" is Rosenhead's 1931 method.)
 *
 *   v, n   : the whirlpools, and how many are in use.
 *   gamma  : how strongly each whirlpool spins the fluid, set at seed time so the
 *            two streams slide past at the preset's shear speed.
 *   blob2  : the softening radius squared, keeping close-range pulls finite.
 *   width  : the left-right repeat distance (the flow tiles across this).
 */
typedef struct {
  Vortex v[MAX_VORTICES];
  int n;
  float gamma;
  float blob2;
  float width;
} VortexSheet;

/*
 * Scene — the whole little world, laid out like a table of contents:
 *   WHAT  is simulated  — the sheet rolling up.
 *   HOW   you drive it   — the preset, speed, and pause knobs.
 *   WHERE we are         — the play-area size and a running clock.
 *   plus the seed rng and one look-only theme knob.
 * Only the big coordinating functions (seed, init, reset, tick) get the whole
 * Scene; everything else is handed just the piece it needs.
 */
typedef struct {
  /* WHAT — the sheet we roll up */
  VortexSheet sheet;

  /* HOW — the knobs the keyboard drives */
  Pattern current_pattern; /* which preset is active  (n/p)   */
  int speed;               /* time multiplier         (+/-)   */
  bool paused;             /* freeze the update       (space) */

  /* WHERE — the play area and a running clock */
  int rows, cols;   /* terminal size the sim reads for its edges */
  float time_accum; /* seconds elapsed */

  /* random-number state, used once at seed time for the tiny ripple wobble */
  uint32_t rng;

  /* look-only — the physics never reads this */
  int current_theme; /* colour scheme (t/T) */
} Scene;

/* The terminal's current width and height, refreshed on every resize. */
typedef struct {
  int cols, rows;
} Screen;

/*
 * App — everything the program owns: the world, the terminal size, and the flags
 * the signal handlers flip to ask the main loop to quit or resize.
 */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;                       /* simulation steps per second (]/[ ) */
  volatile sig_atomic_t running;     /* a signal handler sets this to 0 to quit */
  volatile sig_atomic_t need_resize; /* SIGWINCH sets this; handled next frame */
} App;

/* ── §3 PERFORMANCE — a steady stopwatch (the frame loop that uses it is in §7) ── */

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

/* ── §4 LOGIC — little helpers that just work things out; they change nothing ── */

static const char *pattern_to_name(Pattern p) {
  switch (p) {
  case PATTERN_GENTLE:
    return "GENTLE ";
  case PATTERN_CLASSIC:
    return "CLASSIC";
  case PATTERN_CHOPPY:
    return "CHOPPY ";
  default:
    return "?      ";
  }
}

/*
 * How much one whirlpool (centred at cx,cy) swirls the fluid at (px,py): a push
 * that circles around it, strong up close and fading with distance. The +blob2
 * softens the very-close pull so two whirlpools that touch can't yank on each
 * other infinitely hard and blow the sim up. Adds its bit into *u, *v. (This is
 * the Biot-Savart law.)
 */
static inline void add_vortex_swirl(float px, float py, float cx, float cy,
                                    float gamma, float blob2, float *u,
                                    float *v) {
  float dx = px - cx;
  float dy = py - cy;
  float r2 = dx * dx + dy * dy + blob2;
  float swirl = gamma / r2;
  *u += -dy * swirl; /* the push is at right-angles to the line to the vortex */
  *v += dx * swirl;
}

/*
 * The flow speed the whole sheet makes at one spot (px,py): add up the swirl
 * from every whirlpool, plus one wrapped copy on each side so the flow tiles
 * left-right. Writes through out_u / out_v; reads the sheet, changes nothing.
 */
static void sheet_velocity_at(const VortexSheet *sheet, float px, float py,
                              float *out_u, float *out_v) {
  float u = 0.0f, vel = 0.0f;
  for (int j = 0; j < sheet->n; j++)
    for (int m = -1; m <= 1; m++) /* the point, plus its left/right copies */
      add_vortex_swirl(px, py, sheet->v[j].x + (float)m * sheet->width,
                       sheet->v[j].y, sheet->gamma, sheet->blob2, &u, &vel);
  *out_u = u;
  *out_v = vel;
}

/* Keep x between 0 and width, so a point that slides off one side comes back on
 * the other. y is left alone (the sheet stays near the middle). */
static inline float wrap_x(float x, float width) {
  x = fmodf(x, width);
  if (x < 0.0f)
    x += width;
  return x;
}

/* Has the sheet rolled up until the billows reach near the top/bottom edges? */
static bool sheet_rolled_up(const VortexSheet *sheet, int rows) {
  float mid = (float)rows * 0.5f;
  float limit = RESET_SPREAD_FRAC * (float)rows;
  for (int i = 0; i < sheet->n; i++)
    if (fabsf(sheet->v[i].y - mid) > limit)
      return true;
  return false;
}

/* Colour/glyph tier 0..7 for a whirlpool, from its fixed phase label. */
static inline int phase_to_tier(float phase) {
  int t = (int)(phase * 8.0f);
  if (t < 0)
    t = 0;
  if (t > 7)
    t = 7;
  return t;
}

/* ── §5 SIMULATION — the actual rolling-up: this is what moves the whirlpools ── */

/* A cheap random-number generator. Its state lives in the Scene rather than a
 * global, so a run is repeatable and each reset gets a fresh seed. */
static inline uint32_t lcg_next(uint32_t *st) {
  *st = *st * 1664525u + 1013904223u;
  return *st;
}
static inline float lcg_signed(uint32_t *st) {
  /* a number in [-1, 1) */
  return (float)(lcg_next(st) >> 8) / (float)(1u << 23) - 1.0f;
}

/*
 * sheet_place_vortices — lay the whirlpools evenly along one line, bent into
 * `billows` gentle sine waves (plus a hair of random wobble). That ripple is the
 * seed the swirl rule grows into that many spirals.
 */
static void sheet_place_vortices(VortexSheet *sheet, const ShearLayer *sl,
                                 int rows, uint32_t *rng) {
  float mid = (float)rows * 0.5f;
  for (int i = 0; i < sheet->n; i++) {
    float along = ((float)i + 0.5f) / (float)sheet->n; /* 0..1 along the sheet */
    float ripple = sl->seed_amp * sinf(TWO_PI * (float)sl->billows * along);
    float wobble = SEED_NOISE_CELLS * lcg_signed(rng);
    sheet->v[i].x = along * sheet->width;
    sheet->v[i].y = mid + ripple + wobble;
    sheet->v[i].phase = fmodf((float)sl->billows * along, 1.0f); /* colour ring */
  }
}

/*
 * scene_seed — build a fresh sheet for the current preset: fix its constants,
 * then lay out the rippled starting line. That starting shape is the whole
 * input; from here the swirl rule does the rest.
 */
static void scene_seed(Scene *s) {
  const ShearLayer *sl = &shear_layers[s->current_pattern];
  VortexSheet *sheet = &s->sheet;

  int n = sl->n_vortices;
  if (n > MAX_VORTICES)
    n = MAX_VORTICES;
  sheet->n = n;
  sheet->width = (float)s->cols;
  sheet->blob2 = sl->blob * sl->blob;

  /* Each whirlpool stands for this slice of the seam; its strength is how fast
   * the streams shear times that slice (the /TWO_PI is just the swirl law's
   * constant, folded in here so the velocity code stays tidy). */
  float per_vortex_span = sheet->width / (float)n;
  sheet->gamma = sl->shear * per_vortex_span / TWO_PI;

  sheet_place_vortices(sheet, sl, s->rows, &s->rng);
}

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->paused = false;
  s->speed = SPEED_DEF;
  s->current_theme = 0;
  s->current_pattern = PATTERN_CLASSIC;
  s->rng = (uint32_t)clock_ns();
  s->cols = cols;
  s->rows = rows;
  s->time_accum = 0.0f;
  scene_seed(s);
}

static void scene_resize(Scene *s, int cols, int rows) {
  s->cols = cols;
  s->rows = rows;
  scene_seed(s); /* the sheet is laid out to fit the width, so re-seed on resize */
}

/* Fresh start ('r' key): new random seed, then rebuild the sheet. */
static void scene_reset(Scene *s) {
  s->rng = (uint32_t)clock_ns() ^ 0xC0FFEEu;
  scene_seed(s);
}

/*
 * sheet_step — move the whole sheet one small hop. Do it in two passes: first
 * work out every whirlpool's velocity from where they ALL are now, then move
 * them. If we moved them one at a time, later ones would react to already-moved
 * neighbours and the flow would come out lopsided. `rows` is just so the gentle
 * pull knows which row is the middle.
 */
static void sheet_step(VortexSheet *sheet, int rows, float h) {
  static float vu[MAX_VORTICES];
  static float vv[MAX_VORTICES];

  for (int i = 0; i < sheet->n; i++)
    sheet_velocity_at(sheet, sheet->v[i].x, sheet->v[i].y, &vu[i], &vv[i]);

  float mid = (float)rows * 0.5f;
  for (int i = 0; i < sheet->n; i++) {
    sheet->v[i].x = wrap_x(sheet->v[i].x + vu[i] * h, sheet->width);
    /* move by the swirl, plus a gentle pull back to mid so the billows settle */
    sheet->v[i].y += (vv[i] - CONFINE_RATE * (sheet->v[i].y - mid)) * h;
  }
}

/* One step of the whole sim — the only place the whirlpools actually move. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  dt *= (float)s->speed / (float)SPEED_DEF;
  s->time_accum += dt;

  float h = dt / (float)SUBSTEPS;
  for (int sub = 0; sub < SUBSTEPS; sub++)
    sheet_step(&s->sheet, s->rows, h);

  if (sheet_rolled_up(&s->sheet, s->rows)) /* fully rolled — loop to a flat sheet */
    scene_reset(s);
}

/* ── §6 RENDER — turn the whirlpools into characters on screen; only reads ── */

/* Load one theme's colours into ncurses. Falls back to plain cyan if the
 * terminal only has the basic 8 colours. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &themes[idx];
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_SHEET_BASE + i), t->sheet[i], -1);
    init_pair(PAIR_SKY, t->sky, -1);
  } else {
    for (int i = 0; i < 8; i++)
      init_pair((short)(PAIR_SHEET_BASE + i), COLOR_CYAN, -1);
    init_pair(PAIR_SKY, COLOR_BLACK, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1);
    init_pair(PAIR_HINT, 51, -1);
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0);
}

static void screen_init(Screen *sc) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1); /* stop ncurses peeking at input mid-draw — that causes tearing */
  color_init();
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}
/* The endwin()/refresh() two-step is how you make ncurses notice a new size. */
static void screen_resize_curses(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static inline void sheet_put(int y, int x, int cols, int rows_eff, int tier) {
  if (x < 0 || x >= cols || y < 0 || y >= rows_eff)
    return;
  int pair = PAIR_SHEET_BASE + tier;
  int attr = (tier >= 6) ? A_BOLD : (tier <= 1) ? A_DIM : A_NORMAL;
  attron(COLOR_PAIR(pair) | attr);
  mvaddch(y, x, (chtype)(unsigned char)SHEET_GLYPHS[tier]);
  attroff(COLOR_PAIR(pair) | attr);
}

/* Draw a short straight run of glyphs from (x0,y0) to (x1,y1) so two close
 * neighbours read as one continuous curve instead of two separate dots. */
static void sheet_segment(int x0, int y0, int x1, int y1, int cols, int rows_eff,
                          int tier) {
  int dx = abs(x1 - x0), dy = abs(y1 - y0);
  int steps = dx > dy ? dx : dy;
  if (steps == 0) {
    sheet_put(y0, x0, cols, rows_eff, tier);
    return;
  }
  for (int k = 0; k <= steps; k++) {
    float t = (float)k / (float)steps;
    int x = (int)lroundf((float)x0 + ((float)x1 - (float)x0) * t);
    int y = (int)lroundf((float)y0 + ((float)y1 - (float)y0) * t);
    sheet_put(y, x, cols, rows_eff, tier);
  }
}

/* Where a vortex lands on screen: wrap its x into the width, round y to a cell. */
static inline void vortex_cell(const Vortex *v, float width, int *cx, int *cy) {
  *cx = (int)wrap_x(v->x, width);
  *cy = (int)lroundf(v->y);
}

/* Walk the sheet in order, drawing each whirlpool and joining it to the next
 * with a short segment. The array order is the order along the curve, so this
 * draws one continuous rolled-up line. rows_eff is the height minus the HUD row. */
static void sheet_draw(const VortexSheet *sheet, int cols, int rows_eff) {
  for (int i = 0; i < sheet->n; i++) {
    int xa, ya, xb, yb;
    vortex_cell(&sheet->v[i], sheet->width, &xa, &ya);
    vortex_cell(&sheet->v[(i + 1) % sheet->n], sheet->width, &xb, &yb);
    int tier = phase_to_tier(sheet->v[i].phase);

    /* Join to the next point only if it lands nearby on screen. A big gap means
     * the pair straddles the wrap seam (or the sheet is stretched thin there) —
     * draw a plain dot instead of a line all the way across the screen. */
    float gap = fabsf((float)(xb - xa)) + fabsf((float)(yb - ya));
    if (gap <= CONNECT_MAX_CELLS)
      sheet_segment(xa, ya, xb, yb, cols, rows_eff, tier);
    else
      sheet_put(ya, xa, cols, rows_eff, tier);
  }
}

/* Paint one full-width HUD bar: flood the whole row with the pair's colour, then
 * lay the text over it, so no sheet shows through the bar. */
static void draw_status_bar(int row, int cols, int pair, const char *text) {
  attron(COLOR_PAIR(pair) | A_BOLD);
  for (int x = 0; x < cols; x++)
    mvaddch(row, x, ' ');
  mvprintw(row, 0, "%s", text);
  attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Draw the sheet, then the two HUD bars on top (status line, key list). */
static void screen_draw(Screen *sc, const Scene *s, double fps, int sim_fps) {
  erase();
  sheet_draw(&s->sheet, s->cols, s->rows - 1); /* bottom row kept for HUD */

  const char *state_str =
      s->paused ? "PAUSED " : pattern_to_name(s->current_pattern);

  char status[220];
  snprintf(status, sizeof status,
           " KELVIN-HELMHOLTZ  %s   theme:%-7s   vortices:%4d   "
           "%5.1f fps  %3d Hz  speed:%-3d ",
           state_str, themes[s->current_theme].name, s->sheet.n, fps, sim_fps,
           s->speed);
  const char *hints = " q:quit  spc:pause  r:reset  n/p:preset  t/T:theme  "
                      "+/-:speed  ]/[:Hz ";

  draw_status_bar(0, sc->cols, PAIR_HUD, status);
  draw_status_bar(sc->rows - 1, sc->cols, PAIR_HINT, hints);
}

/* Push our drawing to the terminal in one write, which avoids flicker. */
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §7 APP — keypresses, resizing, and main(): where it all comes together ── */

static App g_app;

/* A signal handler can safely do almost nothing, so these just flip a flag and
 * let the main loop act on it next time around. */
static void on_exit_signal(int sig) {
  (void)sig;
  g_app.running = 0;
}
static void on_resize_signal(int sig) {
  (void)sig;
  g_app.need_resize = 1;
}
/* Runs at exit (via atexit) so the terminal is restored even if we crash out. */
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize_curses(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/* Handle one keypress. Returns false only when the user asked to quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    s->paused = !s->paused;
    break;
  case 'r':
  case 'R':
    scene_reset(s);
    break;

  case '=':
  case '+':
    if (s->speed < SPEED_MAX)
      s->speed *= 2;
    if (s->speed > SPEED_MAX)
      s->speed = SPEED_MAX;
    break;
  case '-':
    s->speed /= 2;
    if (s->speed < SPEED_MIN)
      s->speed = SPEED_MIN;
    break;

  case ']':
    app->sim_fps += SIM_FPS_STEP;
    if (app->sim_fps > SIM_FPS_MAX)
      app->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    app->sim_fps -= SIM_FPS_STEP;
    if (app->sim_fps < SIM_FPS_MIN)
      app->sim_fps = SIM_FPS_MIN;
    break;

  case 't':
    s->current_theme = (s->current_theme + 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;
  case 'T':
    s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(s->current_theme);
    break;

  case 'n':
  case 'N':
    s->current_pattern = (Pattern)(((int)s->current_pattern + 1) % N_PATTERNS);
    scene_seed(s);
    break;
  case 'p':
  case 'P':
    s->current_pattern =
        (Pattern)(((int)s->current_pattern + N_PATTERNS - 1) % N_PATTERNS);
    scene_seed(s);
    break;

  default:
    break;
  }
  return true;
}

/* How long since the last frame, in nanoseconds, and set frame_time to now. If
 * it was a really long gap (paused in a debugger, laptop asleep), we cap it —
 * otherwise the sim would try to catch that whole gap up at once and lock up. */
static int64_t frame_elapsed_ns(int64_t *frame_time) {
  int64_t now = clock_ns();
  int64_t dt = now - *frame_time;
  *frame_time = now;
  if (dt > MAX_CATCHUP_MS * NS_PER_MS)
    dt = MAX_CATCHUP_MS * NS_PER_MS;
  return dt;
}

/* Run one fixed-size sim step for each whole step's worth of time that's built
 * up. A fixed step size means the physics behaves the same at any frame rate. */
static void run_due_ticks(Scene *s, int64_t *sim_accum, int64_t tick_ns,
                          float dt_sec) {
  while (*sim_accum >= tick_ns) {
    scene_tick(s, dt_sec);
    *sim_accum -= tick_ns;
  }
}

/* Sleep out the rest of the frame so every frame takes about the same time —
 * a steady frame rate no matter how little work this one needed. */
static void cap_frame_rate(int64_t frame_start, int64_t dt) {
  int64_t elapsed = clock_ns() - frame_start + dt;
  clock_sleep_ns(NS_PER_SEC / FRAME_CAP_FPS - elapsed);
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* measure elapsed time (clamped), then run every sim step it buys */
    int64_t dt = frame_elapsed_ns(&frame_time);
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;
    sim_accum += dt;
    run_due_ticks(&app->scene, &sim_accum, tick_ns, dt_sec);

    /* refresh the on-screen fps reading about twice a second */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    cap_frame_rate(frame_time, dt);

    /* draw the frame, then handle one keypress */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
