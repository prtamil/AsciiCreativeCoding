/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * sand.c — a falling-sand toy for the terminal.
 *
 * Sand pours from a spout at the top, falls, and piles up into slopes,
 * just like a sand timer. Each grain remembers how long it has sat
 * still; the longer it's buried, the darker and more "packed" it looks.
 * Wind can blow the falling grains sideways.
 *
 * The rules come straight from the classic sandpile model:
 *   Bak, Tang & Wiesenfeld (1987), "Self-Organized Criticality",
 *     Phys. Rev. Lett. 59(4): 381-384 — the original sandpile.
 *   Wolfram (1984), "Cellular Automata as Models of Complexity",
 *     Nature 311: 419-424 — the shuffled-scan trick that keeps the
 *     pile from leaning to one side.
 * For why real sand piles at a gentler angle than this toy does, see
 *   Jaeger, Nagel & Behringer (1996), Rev. Mod. Phys. 68(4): 1259.
 * For why only light, freshly-fallen grains get blown by wind, see
 *   Bagnold (1941), "The Physics of Blown Sand and Desert Dunes".
 */

/* ── §N section map ──
 *   §1  config
 *   §2  clock
 *   §3  color
 *   §4  grid   — the sand rules, grain age, and wind
 *   §5  source — the spout that drops new grains
 *   §6  scene
 *   §7  screen
 *   §8  app
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

/* ── §1 config ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  HUD_COLS = 54,
  FPS_UPDATE_MS = 500,

  SOURCE_ROW = 1,
  SOURCE_W_DEFAULT = 3,
  SOURCE_W_MIN = 1,
  SOURCE_W_MAX = 30,

  WIND_MAX = 3, /* 1 = gentle breeze, 3 = strong gale */

  /* How many ticks a grain must sit still before it looks one step older
   * (and darker). At 30 ticks/sec these work out to roughly:
   *   AGE_DOT   3   ≈ 0.1s   just landed
   *   AGE_SMALL 12  ≈ 0.4s   starting to settle
   *   AGE_MID   30  ≈ 1.0s   settled at the surface
   *   AGE_PACK  60  ≈ 2.0s   packed in the middle
   *   AGE_DENSE 120 ≈ 4.0s   dense, buried at the base
   * The deeper steps take longer on purpose: surface grains still shuffle
   * around, but grains buried deep are locked in place. */
  AGE_DOT = 3,
  AGE_SMALL = 12,
  AGE_MID = 30,
  AGE_PACK = 60,
  AGE_DENSE = 120,
  AGE_MAX = 200, /* stop counting here so the byte age can't overflow (< 255) */
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Chance a young grain gets blown one cell sideways = |wind| / 4. */
#define WIND_PROB_DEN 4

/* ── §2 clock ── */

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

/* ── §3 color ── */

/* The six grain colours go brightest (just-fallen) to darkest (deeply
 * buried). The last four pairs are fixed UI colours, the same in every theme. */
enum {
  CP_NEW = 1,     /* brightest — a grain in the air or just spawned */
  CP_GRAIN = 2,   /* freshly landed                                 */
  CP_LIGHT = 3,   /* starting to settle                             */
  CP_MID = 4,     /* settled at the surface                         */
  CP_PACK = 5,    /* packed in the middle                           */
  CP_DENSE = 6,   /* darkest — dense, buried at the base            */
  CP_SOURCE = 7,  /* the spout marker (bright yellow)               */
  CP_WIND = 8,    /* the wind gauge (light blue)                    */
  PAIR_HUD = 9,   /* top status bar (bright yellow)                 */
  PAIR_HINT = 10, /* bottom key-hint bar (bright cyan)              */
};

/*
 * Theme — a palette of six colours running bright to dark, used to tint
 * grains by how settled they are: ramp[0] for fresh grains down to ramp[5]
 * for deeply buried ones. Picking a theme only changes how the sand looks;
 * the spout, wind gauge, and HUD keep their own colours so they stay readable.
 *
 *   name : what the theme is called, shown in the status bar.
 *   ramp : the six colours, brightest first. Every value sits in the bright
 *          half of the 256-colour palette so even the darkest grain stays
 *          visible against a black background (see the brightness rule in
 *          CLAUDE.md).
 */
typedef struct {
  const char *name;
  short ramp[6];
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    {"MATRIX", {193, 157, 121, 84, 46, 28}},
    {"FIRE", {229, 220, 208, 196, 124, 88}},
    {"OCEANIC", {195, 159, 87, 38, 31, 24}},
    {"NEON", {225, 219, 213, 207, 165, 91}},
    {"MONO", {255, 252, 248, 246, 244, 240}},
    {"ICE", {231, 195, 153, 117, 67, 24}},
    {"NOVA", {231, 226, 220, 208, 202, 130}},
    {"FOREST", {192, 156, 112, 70, 64, 28}},
    {"DESERT", {230, 229, 220, 178, 136, 130}},
    {"ECLIPSE", {217, 209, 173, 167, 95, 52}},
};

/* Switch the six grain colours to a different theme. Fine to call while
 * running — the new colours show up on the next frame. Plain 8-colour
 * terminals skip themes, since the subtle gradient needs 256 colours. */
static void theme_apply(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS < 256)
    return;
  const Theme *t = &themes[idx];
  init_pair(CP_NEW, t->ramp[0], COLOR_BLACK);
  init_pair(CP_GRAIN, t->ramp[1], COLOR_BLACK);
  init_pair(CP_LIGHT, t->ramp[2], COLOR_BLACK);
  init_pair(CP_MID, t->ramp[3], COLOR_BLACK);
  init_pair(CP_PACK, t->ramp[4], COLOR_BLACK);
  init_pair(CP_DENSE, t->ramp[5], COLOR_BLACK);
}

static void color_init(void) {
  start_color();
  if (COLORS >= 256) {
    /* The six grain colours come from theme_apply below. */
    init_pair(CP_SOURCE, 226, COLOR_BLACK);
    init_pair(CP_WIND, 117, COLOR_BLACK);
    init_pair(PAIR_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(PAIR_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    init_pair(CP_NEW, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_GRAIN, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_LIGHT, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_MID, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_PACK, COLOR_RED, COLOR_BLACK);
    init_pair(CP_DENSE, COLOR_RED, COLOR_BLACK);
    init_pair(CP_SOURCE, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_WIND, COLOR_CYAN, COLOR_BLACK);
    init_pair(PAIR_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_HINT, COLOR_CYAN, COLOR_BLACK);
  }
  theme_apply(0); /* start on MATRIX, the first theme */
}

/* ── §4 grid — the sand rules, grain age, and wind ── */

typedef uint8_t Cell;

/*
 * Grid — the whole sand world: which cells hold sand, how long each grain
 * has sat still, and the current wind. Everything is one flat block of
 * memory, allocated once at startup (and again on resize) and never touched
 * by malloc during play.
 *
 * The trick that makes the sand behave: each tick reads the OLD picture and
 * writes a fresh NEW one, then swaps them. If we edited the picture in place
 * while scanning it, a grain could fall, get scanned again one row down, and
 * fall again — dropping straight to the floor in a single tick. So we keep
 * two copies (cur/nxt and age/nxt_age) and a "moved" stamp that marks each
 * grain we've already handled this tick, so it moves at most one cell.
 *
 *   cur     : the current picture. 0 = empty, 1 = sand. Read-only while a
 *             tick is running. This is also what the renderer draws.
 *
 *   nxt     : the picture we're building for the next tick. We write here,
 *             then swap it into cur when the tick finishes.
 *
 *   age     : how many ticks each grain has stayed put, one byte per cell,
 *             counting up to AGE_MAX. This is what makes grains darken: a
 *             fresh grain (age 0) is bright, a long-buried one is dark. The
 *             count snaps back to 0 the instant a grain moves, so only the
 *             undisturbed inside of a pile ages and darkens.
 *
 *   nxt_age : the next-tick ages, swapped in alongside nxt.
 *
 *   moved   : a one-tick stamp, wiped clean at the start of every tick. We
 *             stamp each grain once we've dealt with it (left it or moved it)
 *             so the scan won't pick it up again from its new spot. This is
 *             what limits a grain to one cell of movement per tick.
 *
 *   cols    : grid width in cells. Matches the terminal width, set when the
 *             grid is allocated.
 *
 *   rows    : grid height in cells. Same idea, vertical.
 *
 *   wind    : how hard and which way the wind blows, from -WIND_MAX to
 *             +WIND_MAX. Positive blows right, negative blows left, zero is
 *             calm. Set with the w / W / 0 keys and kept across pause and
 *             resize. It does two things: it leans the spout's stream into
 *             the wind, and it gives young grains a chance to drift sideways.
 *             Older, buried grains are too heavy to lift (Bagnold 1941).
 */
typedef struct {
  Cell *cur;
  Cell *nxt;
  uint8_t *age;
  uint8_t *nxt_age;
  bool *moved;
  int cols;
  int rows;
  int wind;
} Grid;

static void grid_alloc(Grid *g, int cols, int rows) {
  g->cols = cols;
  g->rows = rows;
  g->cur = calloc((size_t)(cols * rows), sizeof(Cell));
  g->nxt = calloc((size_t)(cols * rows), sizeof(Cell));
  g->age = calloc((size_t)(cols * rows), sizeof(uint8_t));
  g->nxt_age = calloc((size_t)(cols * rows), sizeof(uint8_t));
  g->moved = calloc((size_t)(cols * rows), sizeof(bool));
  g->wind = 0;
}
static void grid_free(Grid *g) {
  free(g->cur);
  free(g->nxt);
  free(g->age);
  free(g->nxt_age);
  free(g->moved);
  *g = (Grid){0};
}
static void grid_clear(Grid *g) {
  size_t n = (size_t)(g->cols * g->rows);
  memset(g->cur, 0, n * sizeof(Cell));
  memset(g->nxt, 0, n * sizeof(Cell));
  memset(g->age, 0, n * sizeof(uint8_t));
  memset(g->nxt_age, 0, n * sizeof(uint8_t));
  memset(g->moved, 0, n * sizeof(bool));
}

static inline bool gin(const Grid *g, int x, int y) {
  return x >= 0 && x < g->cols && y >= 0 && y < g->rows;
}
static inline int gidx(const Grid *g, int x, int y) { return y * g->cols + x; }
static inline Cell gget(const Grid *g, int x, int y) {
  if (!gin(g, x, y))
    return 1;
  return g->cur[gidx(g, x, y)];
}
static inline void gset_cur(Grid *g, int x, int y, Cell v) {
  if (gin(g, x, y))
    g->cur[gidx(g, x, y)] = v;
}
static inline bool gmoved(const Grid *g, int x, int y) {
  return gin(g, x, y) && g->moved[gidx(g, x, y)];
}
static inline void gmark(Grid *g, int x, int y) {
  if (gin(g, x, y))
    g->moved[gidx(g, x, y)] = true;
}

/* Slide a grain from one cell to another: empty the old spot, fill the new
 * one, reset its age to 0, and stamp both as handled this tick. */
static void gmove(Grid *g, int sx, int sy, int dx, int dy) {
  g->nxt[gidx(g, sx, sy)] = 0;
  g->nxt_age[gidx(g, sx, sy)] = 0;
  g->nxt[gidx(g, dx, dy)] = 1;
  g->nxt_age[gidx(g, dx, dy)] = 0;
  gmark(g, sx, sy);
  gmark(g, dx, dy);
}

static void grid_update_cell(Grid *g, int x, int y) {
  if (gget(g, x, y) != 1)
    return;
  if (gmoved(g, x, y))
    return;

  int i = gidx(g, x, y);

  /* 1. Try to fall straight down. */
  if (gin(g, x, y + 1) && gget(g, x, y + 1) == 0 && !gmoved(g, x, y + 1)) {
    gmove(g, x, y, x, y + 1);
    return;
  }

  /* 2. Blocked below? Try sliding down to one side. Pick which side
   *    first at random so piles don't always lean the same way. */
  int a = (rand() & 1) ? -1 : 1, b = -a;
  if (gin(g, x + a, y + 1) && gget(g, x + a, y + 1) == 0 &&
      !gmoved(g, x + a, y + 1)) {
    gmove(g, x, y, x + a, y + 1);
    return;
  }
  if (gin(g, x + b, y + 1) && gget(g, x + b, y + 1) == 0 &&
      !gmoved(g, x + b, y + 1)) {
    gmove(g, x, y, x + b, y + 1);
    return;
  }

  /* 3. Can't fall? If there's wind, a young (still light) grain might
   *    get blown one cell sideways. Settled grains are too heavy. */
  if (g->wind != 0 && g->age[i] < AGE_SMALL) {
    int dir = (g->wind > 0) ? 1 : -1;
    int wabs = abs(g->wind);
    if ((rand() % WIND_PROB_DEN) < wabs) {
      int wx = x + dir;
      if (gin(g, wx, y) && gget(g, wx, y) == 0 && !gmoved(g, wx, y)) {
        gmove(g, x, y, wx, y);
        return;
      }
    }
  }

  /* 4. Nowhere to go — the grain stays put and gets one tick older. */
  g->nxt[i] = 1;
  g->nxt_age[i] = (g->age[i] < AGE_MAX) ? g->age[i] + 1 : AGE_MAX;
  gmark(g, x, y);
}

/* Advance the whole grid one step. We scan bottom rows first so a grain
 * that just fell isn't seen again higher up and dropped twice. Within each
 * row we visit columns in a freshly-shuffled order so the sand doesn't
 * always favour one diagonal and lean the pile to one side. */
static void grid_tick(Grid *g) {
  int cols = g->cols, rows = g->rows;
  size_t n = (size_t)(cols * rows);

  memset(g->nxt, 0, n * sizeof(Cell));
  memset(g->nxt_age, 0, n * sizeof(uint8_t));
  memset(g->moved, 0, n * sizeof(bool));

  int *order = malloc((size_t)cols * sizeof(int));
  for (int x = 0; x < cols; x++)
    order[x] = x;

  for (int y = rows - 1; y >= 0; y--) {
    for (int i = cols - 1; i > 0; i--) {
      int j = rand() % (i + 1);
      int t = order[i];
      order[i] = order[j];
      order[j] = t;
    }
    for (int i = 0; i < cols; i++)
      grid_update_cell(g, order[i], y);
  }
  free(order);

  Cell *tc = g->cur;
  g->cur = g->nxt;
  g->nxt = tc;
  uint8_t *ta = g->age;
  g->age = g->nxt_age;
  g->nxt_age = ta;
}

static int grid_neighbors(const Grid *g, int x, int y) {
  int n = 0;
  for (int dy = -1; dy <= 1; dy++)
    for (int dx = -1; dx <= 1; dx++)
      if ((dx || dy) && gin(g, x + dx, y + dy))
        n += g->cur[gidx(g, x + dx, y + dy)];
  return n;
}

/*
 * Pick the character and colour for one grain from its age and how many
 * neighbours surround it. Age sets the basic look (older = darker). The
 * neighbour count is a nudge: a grain packed inside a pile is forced to look
 * at least a bit settled, so a fresh grain dropped into a hole doesn't glow
 * bright among its dark neighbours.
 */
static void grain_visual(uint8_t age, int nb, char *ch_out, attr_t *attr_out) {
  int eff = (int)age;
  if (nb >= 5 && eff < AGE_MID)
    eff = AGE_MID;
  if (nb >= 3 && eff < AGE_SMALL)
    eff = AGE_SMALL;

  char ch;
  attr_t attr;
  if (eff < AGE_DOT) {
    ch = '`';
    attr = COLOR_PAIR(CP_NEW) | A_BOLD;
  } else if (eff < AGE_SMALL) {
    ch = '.';
    attr = COLOR_PAIR(CP_GRAIN) | A_BOLD;
  } else if (eff < AGE_MID) {
    ch = 'o';
    attr = COLOR_PAIR(CP_LIGHT) | A_BOLD;
  } else if (eff < AGE_PACK) {
    ch = 'O';
    attr = COLOR_PAIR(CP_MID);
  } else if (eff < AGE_DENSE) {
    ch = '0';
    attr = COLOR_PAIR(CP_PACK);
  } else {
    ch = '#';
    attr = COLOR_PAIR(CP_DENSE);
  }
  *ch_out = ch;
  *attr_out = attr;
}

/* ── §5 source — the spout that drops new grains ── */

/*
 * Source — the spout at the top that drips new sand. Each tick (when it's
 * on) it drops one grain into every cell across its width, centred on its
 * column and nudged sideways by the wind so the stream leans the way the
 * wind blows. The spout isn't a grain itself; it just plants fresh sand in
 * the grid and lets the falling rules take over from there.
 *
 *   col : the centre column of the spout. Starts in the middle and moves
 *         with the left/right arrows. Kept one cell in from each edge so the
 *         whole stream stays on screen even at full width.
 *
 *   w   : how wide the stream is, in cells. The +/- keys widen and narrow
 *         it between SOURCE_W_MIN and SOURCE_W_MAX.
 *
 *   on  : is the spout dripping? Space toggles it. When off, the spout's
 *         marker still shows where it would drip, but no new sand falls —
 *         handy for watching a pile settle without more arriving.
 */
typedef struct {
  int col, w;
  bool on;
} Source;

static void source_init(Source *s, int cols) {
  s->col = cols / 2;
  s->w = SOURCE_W_DEFAULT;
  s->on = true;
}

static void source_emit(const Source *s, Grid *g) {
  if (!s->on)
    return;
  int half = s->w / 2;
  int wind_offset = g->wind; /* lean the stream the way the wind blows */
  for (int dx = -half; dx <= half; dx++) {
    int x = s->col + dx + wind_offset;
    if (x < 0)
      x = 0;
    if (x >= g->cols)
      x = g->cols - 1;
    if (gin(g, x, SOURCE_ROW) && gget(g, x, SOURCE_ROW) == 0) {
      gset_cur(g, x, SOURCE_ROW, 1);
      g->age[gidx(g, x, SOURCE_ROW)] = 0;
    }
  }
}

/* ── §6 scene ── */

/*
 * Scene — all the changeable state of the simulation in one place, split
 * into the part the sand rules touch and the part only drawing cares about.
 * The scene never talks to the terminal itself: the rules write into the
 * grid, the drawing code reads it. That split means the sand could run with
 * no screen at all.
 *
 *   grid    : the sand world (cells, grain ages, wind). Allocated at
 *             startup and rebuilt to fit the new size on resize, keeping the
 *             current wind. See the Grid notes above for the details.
 *
 *   source  : the spout that drops new grains. Moved and resized by the
 *             arrow and +/- keys, turned on and off with space.
 *
 *   paused  : when set, the sand stops updating (space-bar's sibling, the
 *             p key). Drawing keeps going, so you can freeze the pile and
 *             study its shape mid-avalanche.
 *
 *   current_theme : which palette in themes[] is active, cycled with t / T.
 *             Purely cosmetic — the sand behaves the same whatever colours
 *             are showing.
 */
typedef struct {
  Grid grid;
  Source source;
  bool paused;
  int current_theme;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  grid_alloc(&s->grid, cols, rows);
  source_init(&s->source, cols);
}
static void scene_free(Scene *s) { grid_free(&s->grid); }
static void scene_resize(Scene *s, int cols, int rows) {
  int wind = s->grid.wind;
  grid_free(&s->grid);
  grid_alloc(&s->grid, cols, rows);
  s->grid.wind = wind;
  if (s->source.col >= cols)
    s->source.col = cols / 2;
}
static void scene_tick(Scene *s) {
  if (s->paused)
    return;
  source_emit(&s->source, &s->grid);
  grid_tick(&s->grid);
}

/* ── per-cell render helpers ── */

/* Draw one grain. The caller has already checked this cell holds sand. */
static inline void grain_paint_one(WINDOW *w, const Grid *g, int x, int y) {
  uint8_t age = g->age[gidx(g, x, y)];
  int nb = grid_neighbors(g, x, y);
  char ch;
  attr_t attr;
  grain_visual(age, nb, &ch, &attr);

  wattron(w, attr);
  mvwaddch(w, y, x, (chtype)(unsigned char)ch);
  wattroff(w, attr);
}

/* ── layer renderers ── */

/* Draw all the sand: walk every cell and paint the ones that hold a grain. */
static void grid_render_grains(WINDOW *w, const Grid *g) {
  for (int y = 0; y < g->rows; y++) {
    for (int x = 0; x < g->cols; x++) {
      if (g->cur[gidx(g, x, y)] == 0)
        continue;
      grain_paint_one(w, g, x, y);
    }
  }
}

/* Draw the spout: a little row of 'v' arrows just above where the sand
 * drops, nudged sideways by the wind to match where grains will actually land. */
static void source_render_marker(WINDOW *w, const Source *src, const Grid *g) {
  int half = src->w / 2;
  int wo = g->wind;

  wattron(w, COLOR_PAIR(CP_SOURCE) | A_BOLD);
  for (int dx = -half; dx <= half; dx++) {
    int mx = src->col + dx + wo;
    if (mx >= 0 && mx < g->cols)
      mvwaddch(w, SOURCE_ROW - 1, mx, 'v');
  }
  wattroff(w, COLOR_PAIR(CP_SOURCE) | A_BOLD);
}

/* Draw the wind gauge: a few '>' or '<' arrows in a bottom corner, one per
 * unit of wind strength, pointing the way it blows. Right-blowing wind sits
 * on the right edge, left-blowing on the left. We never draw more than a
 * quarter of the width so it can't cover the bottom hint bar. */
static void wind_render_indicator(WINDOW *w, const Grid *g) {
  char wc = (g->wind > 0) ? '>' : '<';
  int wabs = abs(g->wind);
  int cap = g->cols / 4;

  wattron(w, COLOR_PAIR(CP_WIND) | A_BOLD);
  for (int i = 0; i < wabs && i < cap; i++) {
    int wx = (g->wind > 0) ? g->cols - 1 - i : i;
    if (wx >= 0 && wx < g->cols && g->rows > 1)
      mvwaddch(w, g->rows - 1, wx, (chtype)(unsigned char)wc);
  }
  wattroff(w, COLOR_PAIR(CP_WIND) | A_BOLD);
}

/* ── draw the whole scene ── */

/* Lay down the sand first, then the spout marker and wind gauge on top. */
static void scene_draw(const Scene *s, WINDOW *w) {
  grid_render_grains(w, &s->grid);

  if (s->source.on)
    source_render_marker(w, &s->source, &s->grid);

  if (s->grid.wind != 0)
    wind_render_indicator(w, &s->grid);
}

/* ── §7 screen ── */

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *s) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
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

/*
 * Draw the sand, then lay two status bars over the top and bottom rows:
 * the top one shows live state (paused, spout on/off, position, wind, fps),
 * the bottom one lists every key. We fill each bar with its colour across the
 * full width and draw them last so no grain pokes through.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps) {
  erase();
  scene_draw(sc, stdscr);

  /* ── top row: live status ── */
  const char *wstr = sc->grid.wind > 0   ? ">>>"
                     : sc->grid.wind < 0 ? "<<<"
                                         : "---";

  char status[200];
  snprintf(status, sizeof status,
           " SAND   %s   theme:%-7s   emit:%s   src col:%3d  w:%d   "
           "wind:%s (%+d)   %5.1f fps  %3d Hz ",
           sc->paused ? "PAUSED " : "running", themes[sc->current_theme].name,
           sc->source.on ? "ON " : "OFF", sc->source.col, sc->source.w, wstr,
           sc->grid.wind, fps, sim_fps);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── bottom row: the keys you can press ── */
  const char *hints =
      " q:quit  spc:emit  p:pause  r:clear  </>:move  +/-:width  "
      "w/W:wind  0:calm  t/T:theme  ]/[:Hz ";

  int hint_row = s->rows - 1;
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(hint_row, x, ' ');
  mvprintw(hint_row, 0, "%s", hints);
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running, need_resize;
} App;

static App g_app;
static void on_exit_signal(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize_signal(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *sc = &app->scene;
  Source *src = &sc->source;
  Grid *g = &sc->grid;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    src->on = !src->on;
    break;
  case 'p':
  case 'P':
    sc->paused = !sc->paused;
    break;
  case 'r':
  case 'R':
    grid_clear(g);
    break;

  case KEY_LEFT:
    src->col--;
    if (src->col < 1)
      src->col = 1;
    break;
  case KEY_RIGHT:
    src->col++;
    if (src->col >= g->cols - 1)
      src->col = g->cols - 2;
    break;

  case '=':
  case '+':
    src->w++;
    if (src->w > SOURCE_W_MAX)
      src->w = SOURCE_W_MAX;
    break;
  case '-':
    src->w--;
    if (src->w < SOURCE_W_MIN)
      src->w = SOURCE_W_MIN;
    break;

  case 'w':
    g->wind++;
    if (g->wind > WIND_MAX)
      g->wind = WIND_MAX;
    break;
  case 'W':
    g->wind--;
    if (g->wind < -WIND_MAX)
      g->wind = -WIND_MAX;
    break;
  case '0':
    g->wind = 0;
    break;

  case 't':
    sc->current_theme = (sc->current_theme + 1) % N_THEMES;
    theme_apply(sc->current_theme);
    break;
  case 'T':
    sc->current_theme = (sc->current_theme + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->current_theme);
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

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)clock_ns());
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

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene);
      sim_accum -= tick_ns;
    }

    /*
     * How far we are into the next not-yet-fired tick, as a fraction from
     * 0 up to 1. Sand snaps to whole cells so we don't actually use this to
     * draw — it's kept for shape and for any future smooth-motion work.
     */
    float alpha = (float)sim_accum / (float)tick_ns;
    (void)alpha;

    /* ── frames-per-second counter ── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Cap the frame rate. We sleep before drawing, not after, so the time
     * spent writing to the terminal doesn't make the rate drift. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
