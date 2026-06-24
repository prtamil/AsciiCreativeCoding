/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * constellation.c — drifting stars that draw lines to nearby neighbours.
 *
 * Stars wander slowly; every frame we check all pairs and draw a faint
 * line between any two that are close enough. The lines aren't stored,
 * just recomputed each frame from where the stars currently are.
 *
 * References (the things the code alone can't tell you):
 *   Bresenham (1965), IBM Systems J. 4(1): 25 — the thin-line drawing trick
 *   Uhlenbeck & Ornstein (1930), Phys. Rev. 36: 823 — the wandering-walk idea
 *   bounce_ball.c (this repo) — origin of the pixel/cell split used in §4
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

/* ── §1 config ── */

enum {
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  HUD_COLS = 64, /* wide enough to fit the theme name on the end */
  FPS_UPDATE_MS = 500,

  STARS_DEFAULT = 30,
  STARS_MIN = 5,
  STARS_MAX = 80,

  N_STAR_COLORS = 6, /* stars use colour slots 1..6        */
  CONN_PAIR = 7,     /* every connection line uses slot 7   */
  HUD_PAIR = 8,      /* status text, top-right (yellow)     */
  HINT_PAIR = 9,     /* key hints, bottom-left (cyan)       */
};

/* We do the motion math in tiny "pixels" and only turn them into screen
 * cells when drawing. A cell is 8 pixels wide and 16 tall, which matches
 * the roughly 1:2 shape of a real terminal character — so a star moving
 * the same speed sideways and downways looks the same speed on screen. */
#define CELL_W 8
#define CELL_H 16

/* How fast a star starts out (pixels per second). Kept slow on purpose:
 * fast stars draw an ugly staircase on near-straight paths. */
#define SPEED_MIN 50.0f
#define SPEED_MAX 120.0f

/* The wander limits. WANDER_ACCEL caps how hard the random nudge each
 * second can be; SPEED_CAP caps the top speed it can build up to. */
#define WANDER_ACCEL 20.0f
#define SPEED_CAP 130.0f

/* How close two stars must be to get a line, in pixels. The 'c' key
 * cycles these: tighter draws fewer lines, wider draws more. */
static const float k_connect_presets[] = {120.0f, 200.0f, 280.0f};
static const char *k_connect_names[] = {"tight", "normal", "wide"};
enum { N_CONNECT_PRESETS = 3 };

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

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

/*
 * ConstTheme — one named colour scheme: six colours for the stars plus
 * one for the connection lines. Picking a theme is purely cosmetic.
 *
 *   name        the label shown in the status bar (cycled by t/T)
 *   star_fg256  the six star colours on a modern 256-colour terminal
 *   star_fg8    the same six, dumbed down for an old 8-colour terminal
 *   conn_fg256  the line colour on a 256-colour terminal
 *   conn_fg8    the line colour on an 8-colour terminal
 *
 * The table never changes. Switching theme just re-points colour slots
 * 1..7 at a different row (see theme_apply); already-drawn characters
 * pick up the new colour on their next repaint, so there's no flicker.
 * The status/hint colours (slots 8,9) sit outside this range on purpose
 * so the on-screen text stays the same colour no matter the theme.
 */
typedef struct {
  const char *name;
  int star_fg256[6]; /* star colours, 256-colour terminal   */
  int star_fg8[6];   /* star colours, 8-colour terminal     */
  int conn_fg256;    /* line colour, 256-colour terminal    */
  int conn_fg8;      /* line colour, 8-colour terminal      */
} ConstTheme;

#define N_THEMES 7

static const ConstTheme k_themes[N_THEMES] = {
    /* The original look, from before themes existed. */
    {"night",
     {15, 51, 39, 201, 147, 159},
     {COLOR_WHITE, COLOR_CYAN, COLOR_BLUE, COLOR_MAGENTA, COLOR_CYAN,
      COLOR_WHITE},
     24,
     COLOR_BLUE},

    /* Greens and teals — northern-lights vibe. */
    {"aurora",
     {46, 51, 87, 119, 156, 158},
     {COLOR_GREEN, COLOR_CYAN, COLOR_CYAN, COLOR_GREEN, COLOR_GREEN,
      COLOR_CYAN},
     30,
     COLOR_GREEN},

    /* Saturated magentas / pinks — a star-forming region. */
    {"nebula",
     {201, 207, 213, 219, 225, 159},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_CYAN},
     91,
     COLOR_MAGENTA},

    /* Cool blues + whites — clear winter sky. */
    {"winter",
     {195, 159, 153, 117, 75, 39},
     {COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_BLUE, COLOR_BLUE},
     24,
     COLOR_BLUE},

    /* Warm reds → oranges → yellows — campfire / brazier sky. */
    {"ember",
     {226, 220, 214, 208, 202, 196},
     {COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_RED, COLOR_RED,
      COLOR_RED},
     130,
     COLOR_RED},

    /* Deep indigos + cool whites — empty-space void. */
    {"void",
     {252, 248, 141, 105, 99, 63},
     {COLOR_WHITE, COLOR_WHITE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE},
     53,
     COLOR_BLUE},

    /* Whites + light greys — minimalist / B&W photograph. */
    {"mono",
     {255, 253, 252, 250, 248, 246},
     {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE},
     244,
     COLOR_WHITE},
};

/* Point the seven drawing colour slots at the chosen theme's colours. */
static void theme_apply(int theme) {
  const ConstTheme *th = &k_themes[theme];
  if (COLORS >= 256) {
    for (int i = 0; i < 6; i++)
      init_pair(1 + i, th->star_fg256[i], COLOR_BLACK);
    init_pair(CONN_PAIR, th->conn_fg256, COLOR_BLACK);
  } else {
    for (int i = 0; i < 6; i++)
      init_pair(1 + i, th->star_fg8[i], COLOR_BLACK);
    init_pair(CONN_PAIR, th->conn_fg8, COLOR_BLACK);
  }
}

/* Turn colour on, load the starting theme, and fix the status/hint colours. */
static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);
  if (COLORS >= 256) {
    init_pair(HUD_PAIR, 226, -1); /* bright yellow */
    init_pair(HINT_PAIR, 51, -1); /* bright cyan   */
  } else {
    init_pair(HUD_PAIR, COLOR_YELLOW, -1);
    init_pair(HINT_PAIR, COLOR_CYAN, -1);
  }
}

/* ── §4 coords ── */
/* The one place we convert between motion-pixels and screen-cells. */

/* Turn a count of cells into a count of pixels. */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

/* Turn a pixel position into the cell it lands in (rounding to nearest). */
static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 star ── */

/*
 * Star — one drifting star. It only knows where it is and where it's
 * going; the lines between stars are worked out fresh each frame, never
 * stored on the star itself.
 *
 *   px, py            where it is right now        (pixels)
 *   prev_px, prev_py  where it was last step       (pixels)
 *   vx, vy            how fast it's moving         (pixels/sec)
 *   color             which colour slot, 1..N_STAR_COLORS
 *   ch                which glyph it draws as, one of "*+o@."
 *
 * We keep the previous position so the renderer can draw the star part
 * way between its old and new spot for smooth motion (it slides along the
 * line between the two). That's exact no matter how the star accelerated;
 * guessing ahead from the current position would drift off course.
 *
 * Stars are never freed. A live count (Scene.n) marks how many of the
 * fixed pool are in use; "remove a star" just lowers that count.
 */
typedef struct {
  float px, py;           /* current position  (pixels)          */
  float prev_px, prev_py; /* previous position (pixels)          */
  float vx, vy;           /* velocity (pixels/sec)               */
  int color;              /* colour slot, 1..N_STAR_COLORS       */
  char ch;                /* glyph, one of k_star_chars          */
} Star;

/* The glyphs stars can be drawn as; picked by index so a crowd varies. */
static const char k_star_chars[] = "*+o@.";
static const int k_n_star_chars = (int)(sizeof k_star_chars - 1);

/* Drop one star at a random spot with a random direction at random speed. */
static void star_spawn(Star *s, int idx, int cols, int rows) {
  int pxw = pw(cols);
  int pxh = ph(rows);

  s->px = (float)(CELL_W + rand() % (pxw - 2 * CELL_W));
  s->py = (float)(CELL_H + rand() % (pxh - 2 * CELL_H));
  s->prev_px = s->px;
  s->prev_py = s->py;

  /* Pick a random direction by throwing darts at a square and keeping
   * only the ones inside a circle, so every angle is equally likely. */
  float dx, dy, len;
  do {
    dx = (float)(rand() % 2001 - 1000) / 1000.0f;
    dy = (float)(rand() % 2001 - 1000) / 1000.0f;
    len = dx * dx + dy * dy;
  } while (len < 0.01f || len > 1.0f);

  float mag = sqrtf(len);
  float speed = SPEED_MIN + (float)(rand() % (int)(SPEED_MAX - SPEED_MIN + 1));
  s->vx = (dx / mag) * speed;
  s->vy = (dy / mag) * speed;

  s->color = (idx % N_STAR_COLORS) + 1;
  s->ch = k_star_chars[idx % k_n_star_chars];
}

/*
 * Move one star forward by one step: nudge it in a random direction,
 * keep its speed in check, slide it along, and bounce it off the edges.
 *
 *   s              the star to move (changed in place)
 *   dt             how much time this step covers, in seconds
 *   max_px, max_py the far edges of the play area, in pixels
 *
 * The order is deliberate. We save the old position first so the smooth-
 * motion blend has both endpoints to slide between. We cap the speed
 * before moving, so one big random nudge can't fling a star clear through
 * a wall in a single step. We check the walls last, against the position
 * it just landed on.
 */
static void star_tick(Star *s, float dt, float max_px, float max_py) {
  /* Step 1 — remember where it was, for the smooth-motion blend. */
  s->prev_px = s->px;
  s->prev_py = s->py;

  /* Step 2 — give it a small random shove so it wanders. */
  float ax = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
  float ay = ((float)(rand() % 2001) - 1000.0f) / 1000.0f * WANDER_ACCEL;
  s->vx += ax * dt;
  s->vy += ay * dt;

  /* Step 3 — if it's going too fast, scale it back to the speed limit. */
  float spd = sqrtf(s->vx * s->vx + s->vy * s->vy);
  if (spd > SPEED_CAP) {
    float inv = SPEED_CAP / spd;
    s->vx *= inv;
    s->vy *= inv;
  }

  /* Step 4 — move it: position += velocity * time. */
  s->px += s->vx * dt;
  s->py += s->vy * dt;

  /* Step 5 — if it hit an edge, pin it back inside and bounce it off. */
  if (s->px < 0.0f) {
    s->px = 0.0f;
    s->vx = -s->vx;
  }
  if (s->px > max_px) {
    s->px = max_px;
    s->vx = -s->vx;
  }
  if (s->py < 0.0f) {
    s->py = 0.0f;
    s->vy = -s->vy;
  }
  if (s->py > max_py) {
    s->py = max_py;
    s->vy = -s->vy;
  }
}

/* ── §6 scene ── */

/*
 * Scene — the whole world in one struct: the stars plus the few things
 * the user can tweak at runtime. Everything is allocated up front; no
 * memory is grabbed or freed while it's running.
 *
 *   stars            a fixed pool of room for STARS_MAX stars
 *   n                how many are actually in use right now
 *   paused           when true, the stars stop moving
 *   connect_preset   which line-distance from k_connect_presets is active
 *   current_theme    which colour scheme from k_themes is active
 *
 * The live stars are always the first n in the pool. Stars are all alike
 * once placed, so there's no need to track which slot is which:
 *   add one    → fill slot n, then n++
 *   remove one → n--   (the slot's memory just sits unused)
 */
typedef struct {
  Star stars[STARS_MAX];
  int n; /* the first n stars are the live ones */
  bool paused;
  int connect_preset; /* which entry of k_connect_presets */
  int current_theme;  /* which entry of k_themes          */
} Scene;

/* Clear everything, set the defaults, and scatter the opening stars. */
static void scene_init(Scene *sc, int cols, int rows) {
  memset(sc, 0, sizeof *sc);
  sc->n = STARS_DEFAULT;
  sc->connect_preset = 1; /* "normal" */
  sc->current_theme = 0;  /* "night"  */
  for (int i = 0; i < sc->n; i++)
    star_spawn(&sc->stars[i], i, cols, rows);
}

/*
 * Move every live star forward one step (unless paused). Stars don't
 * notice each other here — the lines between them are purely a drawing
 * thing, handled later. This does no screen work at all, just the math.
 */
static void scene_tick(Scene *sc, float dt, int cols, int rows) {
  if (sc->paused)
    return;

  float max_px = (float)(pw(cols) - 1);
  float max_py = (float)(ph(rows) - 1);

  for (int i = 0; i < sc->n; i++)
    star_tick(&sc->stars[i], dt, max_px, max_py);
}

/*
 * Draw a thin straight line between two cells using one character per
 * step, picking - | / or \ to match the slope. `stipple` lets us skip
 * cells to fade a line out. `used` marks cells already drawn this frame
 * so the first line to reach a cell keeps it (stops glyphs clashing).
 */
static void draw_line(WINDOW *w, int x0, int y0, int x1, int y1, chtype attr,
                      int stipple, int cols, int rows, bool *used) {
  int adx = abs(x1 - x0), sx = (x0 < x1) ? 1 : -1;
  int ady = abs(y1 - y0), sy = (y0 < y1) ? 1 : -1;
  int step = 0;
  char diag = (sx * sy > 0) ? '\\' : '/';

  if (adx == 0 && ady == 0) {
    if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows && !used[y0 * cols + x0]) {
      used[y0 * cols + x0] = true;
      wattron(w, attr);
      mvwaddch(w, y0, x0, (chtype)(unsigned char)diag);
      wattroff(w, attr);
    }
    return;
  }

  if (adx >= ady) {
    /* mostly horizontal: step one column at a time */
    int err = adx / 2;
    for (int x = x0; x != x1 + sx; x += sx) {
      int next_err = err - ady;
      char ch = (next_err < 0) ? diag : '-';
      if (x >= 0 && x < cols && y0 >= 0 && y0 < rows && step % stipple == 0 &&
          !used[y0 * cols + x]) {
        used[y0 * cols + x] = true;
        wattron(w, attr);
        mvwaddch(w, y0, x, (chtype)(unsigned char)ch);
        wattroff(w, attr);
      }
      step++;
      err = next_err;
      if (err < 0) {
        y0 += sy;
        err += adx;
      }
    }
  } else {
    /* mostly vertical: step one row at a time */
    int err = ady / 2;
    for (int y = y0; y != y1 + sy; y += sy) {
      int next_err = err - adx;
      char ch = (next_err < 0) ? diag : '|';
      if (x0 >= 0 && x0 < cols && y >= 0 && y < rows && step % stipple == 0 &&
          !used[y * cols + x0]) {
        used[y * cols + x0] = true;
        wattron(w, attr);
        mvwaddch(w, y, x0, (chtype)(unsigned char)ch);
        wattroff(w, attr);
      }
      step++;
      err = next_err;
      if (err < 0) {
        x0 += sx;
        err += ady;
      }
    }
  }
}

/* Keep a cell index from falling off the screen edges. */
static inline int clamp_cell_x(int v, int cols) {
  if (v < 0)
    return 0;
  if (v >= cols)
    return cols - 1;
  return v;
}
static inline int clamp_cell_y(int v, int rows) {
  if (v < 0)
    return 0;
  if (v >= rows)
    return rows - 1;
  return v;
}

/*
 * Work out where to actually draw each star this frame: blend between its
 * old and new spot (alpha says how far along) for smooth motion, in both
 * pixels (dpx/dpy, for measuring distance) and cells (dcx/dcy, for drawing).
 */
static void compute_lerp_positions(const Scene *sc, float alpha, int cols,
                                   int rows, float *dpx, float *dpy, int *dcx,
                                   int *dcy) {
  for (int i = 0; i < sc->n; i++) {
    const Star *s = &sc->stars[i];
    dpx[i] = s->prev_px + (s->px - s->prev_px) * alpha;
    dpy[i] = s->prev_py + (s->py - s->prev_py) * alpha;
    dcx[i] = clamp_cell_x(px_to_cell_x(dpx[i]), cols);
    dcy[i] = clamp_cell_y(px_to_cell_y(dpy[i]), rows);
  }
}

/*
 * Choose how a line looks from how stretched it is (ratio: 0 = stars
 * touching, near 1 = barely in range). Close pairs get a bold solid line;
 * the farther apart, the dimmer and more dotted it gets.
 */
static void pick_connect_style(float ratio, chtype *out_attr,
                               int *out_stipple) {
  if (ratio < 0.50f) {
    *out_attr = COLOR_PAIR(CONN_PAIR) | A_BOLD;
    *out_stipple = 1;
  } else if (ratio < 0.75f) {
    *out_attr = COLOR_PAIR(CONN_PAIR);
    *out_stipple = 1;
  } else {
    *out_attr = COLOR_PAIR(CONN_PAIR);
    *out_stipple = 2;
  }
}

/*
 * Check every pair of stars and draw a line between the close ones. We
 * compare squared distances to skip the costly square root for pairs that
 * are too far apart, and only take the real distance for the few that draw.
 */
static void scene_draw_connections(const Scene *sc, WINDOW *w, int cols,
                                   int rows, const float *dpx, const float *dpy,
                                   const int *dcx, const int *dcy,
                                   float connect_dist, bool *cell_used) {
  float cdist_sq = connect_dist * connect_dist;

  for (int i = 0; i < sc->n - 1; i++) {
    for (int j = i + 1; j < sc->n; j++) {
      float dx_px = dpx[j] - dpx[i];
      float dy_px = dpy[j] - dpy[i];
      float dist_sq = dx_px * dx_px + dy_px * dy_px;
      if (dist_sq >= cdist_sq)
        continue; /* too far apart — skip without the square root */

      float dist = sqrtf(dist_sq);
      float ratio = dist / connect_dist; /* 0 = touching, 1 = at the limit */

      chtype attr;
      int stipple;
      pick_connect_style(ratio, &attr, &stipple);

      draw_line(w, dcx[i], dcy[i], dcx[j], dcy[j], attr, stipple, cols, rows,
                cell_used);
    }
  }
}

/* Draw the star glyphs. Done last so a star always sits on top of a line. */
static void scene_draw_stars(const Scene *sc, WINDOW *w, const int *dcx,
                             const int *dcy) {
  for (int i = 0; i < sc->n; i++) {
    const Star *s = &sc->stars[i];
    wattron(w, COLOR_PAIR(s->color) | A_BOLD);
    mvwaddch(w, dcy[i], dcx[i], (chtype)(unsigned char)s->ch);
    wattroff(w, COLOR_PAIR(s->color) | A_BOLD);
  }
}

/*
 * Draw one full frame: figure out where everything is, draw the lines,
 * then draw the stars on top. Lines first and stars last so a star is
 * never hidden by a line crossing the same spot. `alpha` is how far we
 * are between the last and next movement step (0 = just stepped).
 */
static void scene_draw(const Scene *sc, WINDOW *w, int cols, int rows,
                       float alpha) {
  /* Step 1 — work out the on-screen spot for every star, once. */
  float dpx[STARS_MAX], dpy[STARS_MAX];
  int dcx[STARS_MAX], dcy[STARS_MAX];
  compute_lerp_positions(sc, alpha, cols, rows, dpx, dpy, dcx, dcy);

  /* Step 2 — draw the lines. cell_used starts blank each frame and lets
   * the first line into a cell win, so overlapping lines don't clash. */
  bool cell_used[rows][cols];
  memset(cell_used, 0, sizeof cell_used);

  float connect_dist = k_connect_presets[sc->connect_preset];
  scene_draw_connections(sc, w, cols, rows, dpx, dpy, dcx, dcy, connect_dist,
                         &cell_used[0][0]);

  /* Step 3 — draw the stars on top of the lines. */
  scene_draw_stars(sc, w, dcx, dcy);
}

/* ── §7 screen ── */

/* The current terminal size, in cells. */
typedef struct {
  int cols;
  int rows;
} Screen;

static void screen_init(Screen *s, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
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

static void format_hud_text(char *buf, size_t n, const Scene *sc, double fps,
                            int sim_fps) {
  snprintf(buf, n, " %5.1f fps  stars:%-2d  conn:%-6s  spd:%-2d  [%s]  %s ",
           fps, sc->n, k_connect_names[sc->connect_preset], sim_fps,
           k_themes[sc->current_theme].name,
           sc->paused ? "PAUSED " : "running");
}

static void paint_hud_topright(int row, int cols, const char *text) {
  int hud_x = cols - (int)strlen(text);
  if (hud_x < 0)
    hud_x = 0;
  attron(COLOR_PAIR(HUD_PAIR) | A_BOLD);
  mvprintw(row, hud_x, "%s", text);
  attroff(COLOR_PAIR(HUD_PAIR) | A_BOLD);
}

static void paint_hint_bottomleft(int row, const char *text) {
  attron(COLOR_PAIR(HINT_PAIR) | A_BOLD);
  mvprintw(row, 0, "%s", text);
  attroff(COLOR_PAIR(HINT_PAIR) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha);

  char hud_text[HUD_COLS + 1];
  format_hud_text(hud_text, sizeof hud_text, sc, fps, sim_fps);
  paint_hud_topright(0, s->cols, hud_text);

  paint_hint_bottomleft(s->rows - 1,
                        " q/ESC:quit  spc:pause  +/-:stars  c:connect"
                        "  r:randomise  ]/[:speed  t/T:theme ");
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §8 app ── */

/*
 * App — everything the program holds together while it runs: the world,
 * the screen size, the chosen update rate, and two flags that the OS
 * signal handlers flip. They're sig_atomic_t because a signal can change
 * them at any instant, even mid-instruction.
 *
 *   sim_fps      how many movement steps per second
 *   running      cleared when it's time to quit
 *   need_resize  set when the terminal window changed size
 */
typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

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

static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  int cols = app->screen.cols;
  int rows = app->screen.rows;
  float mpx = (float)(pw(cols) - 1);
  float mpy = (float)(ph(rows) - 1);
  for (int i = 0; i < app->scene.n; i++) {
    Star *s = &app->scene.stars[i];
    if (s->px > mpx)
      s->px = mpx;
    if (s->py > mpy)
      s->py = mpy;
    if (s->prev_px > mpx)
      s->prev_px = mpx;
    if (s->prev_py > mpy)
      s->prev_py = mpy;
  }
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *sc = &app->scene;
  int cols = app->screen.cols;
  int rows = app->screen.rows;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    sc->paused = !sc->paused;
    break;

  case 'r':
  case 'R':
    for (int i = 0; i < sc->n; i++)
      star_spawn(&sc->stars[i], i, cols, rows);
    break;

  case '=':
  case '+':
    if (sc->n < STARS_MAX) {
      star_spawn(&sc->stars[sc->n], sc->n, cols, rows);
      sc->n++;
    }
    break;

  case '-':
    if (sc->n > STARS_MIN)
      sc->n--;
    break;

  case 'c':
  case 'C':
    sc->connect_preset = (sc->connect_preset + 1) % N_CONNECT_PRESETS;
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

static void app_install_signals(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);
}

int main(void) {
  srand((unsigned int)clock_ns());
  app_install_signals();

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen, app->scene.current_theme);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* Handle a window resize before anything else this frame. */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* How much real time passed since the last frame. */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* if we stalled, don't try to catch up forever */

    /* Run as many fixed-size movement steps as the elapsed time earns. */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    /*
     * How far we are between the last movement step and the next one.
     * The renderer uses it to draw stars part way along for smooth motion:
     * 0.0 = right on the last step, 0.9 = almost at the next one.
     */
    float alpha = (float)sim_accum / (float)tick_ns;

    /* Update the on-screen frames-per-second figure now and then. */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* Sleep just enough to hold the draw rate near 60 frames a second. */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* Draw the frame and flush it to the terminal. */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha);
    screen_present();

    /* Read a key if one's waiting (this doesn't block). */
    int key = getch();
    if (key != ERR && !app_handle_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
