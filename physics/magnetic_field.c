/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * magnetic_field.c — magnetic field lines from a handful of bar magnets.
 *
 * Each magnet is treated as two opposite "magnetic charges" at its ends.
 * We add up the pull of every charge to get the field at any point, then
 * trace the curving lines that follow it — the iron-filing pattern you'd
 * see around real magnets. Four presets show classic arrangements.
 *
 * The physics (two-charge magnet model, fields just add) is in Griffiths,
 * Introduction to Electrodynamics, ch.5-6, and Purcell, Electricity and
 * Magnetism, ch.6. The line-following math (RK4 streamline tracing) is in
 * Numerical Recipes ch.17. The "where the field is zero" spots the lines
 * stop at are the critical points of Helman & Hesselink (1991).
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
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

enum {
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MIN = 5,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  MAX_MONOPOLES = 8,    /* most charges we hold (2 per magnet)       */
  MAX_DIPOLES = 4,      /* most bar magnets on screen at once        */
  N_SEEDS = 16,         /* how many lines start around each N pole   */
  MAX_LINE_STEPS = 600, /* hard cap on how long one line can get     */
  MAX_LINES = MAX_DIPOLES * N_SEEDS, /* every line we might trace    */

  N_PRESETS = 4,
  N_THEMES = 5,

  LINES_PER_TICK_DEF = 2, /* lines revealed each tick by default     */
  LINES_PER_TICK_MIN = 1,
  LINES_PER_TICK_MAX = 8,

  ROWS_MAX = 128,
  COLS_MAX = 512,
};

#define NS_PER_SEC 1000000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* How far each line step moves, measured in cells. */
#define RK4_H 0.35f
/* Stop a line once the field gets this weak (we're near a dead spot). */
#define B_MIN 1e-6f
/* Smooths the field right next to a pole so it doesn't blow up to infinity. */
#define SOFT 0.8f
/* Radius of the ring of starting points placed around each N pole, in cells. */
#define SEED_R 1.2f
/* Drop a direction arrow every this-many steps along a line. */
#define ARR_STRIDE 18
/* Terminal cells are about twice as tall as they are wide; this corrects for that. */
#define ASPECT_R 2.0f

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

/* ── §3 color / theme ── */

/*
 * Names for the colour slots we paint with. The first six change with the
 * chosen theme; the last two (the on-screen text) always stay bright
 * yellow and cyan so they stay readable no matter what the field looks
 * like behind them.
 */
enum {
  CP_LINE_DIM = 1,
  CP_LINE_MID = 2,
  CP_LINE_BRT = 3,
  CP_NORTH = 4,
  CP_SOUTH = 5,
  CP_BODY = 6,
  CP_HUD = 7,
  CP_HINT = 8,
};

/* We keep one text row at the top and one at the bottom for the on-screen
 * readout; the field draws in the band between them. */
#define HUD_TOP_ROWS 1
#define HUD_BOT_ROWS 1

/*
 * Theme — one complete colour scheme for the whole picture.
 *
 * Each theme carries two full sets of colours: a rich one for modern
 * terminals (256 colours) and a plain one for old or bare-bones terminals
 * (only 8 colours). At startup we pick whichever set the terminal can
 * actually show, so the same program looks right everywhere.
 *
 * The three line colours go from dim to bright on purpose: a line is
 * brightest where it leaves a pole and fades as it travels, so brightness
 * reads as "how far along this line am I". Keeping all of a theme's colours
 * together means you can't accidentally mix one theme's bright with
 * another's dim while editing. (The dim/mid/bright fade idea follows Ware,
 * Information Visualization, ch.4 on sequential colour.)
 *
 * The two text colours (yellow status, cyan keys) are NOT here on purpose:
 * they stay fixed across every theme so the readout never vanishes against
 * a busy background.
 */
typedef struct {
  /* Colours for a 256-colour terminal, listed the way you'd see them
   * along a line: dim at the fading tail, bright at the pole it left. */
  short line_dim; /* faded tail of the line, near a dead spot or the far pole */
  short line_mid; /* middle stretch of the line                               */
  short line_brt; /* freshest part, right where the line leaves its pole       */
  short north;    /* the 'N' marker — a bold accent that should pop            */
  short south;    /* the 'S' marker — a contrasting accent, opposite of north  */
  short body;     /* the '=' bar drawn between a magnet's two poles; kept quiet */

  /* The same six roles again, for an 8-colour terminal. With only 8
   * colours there isn't room for a dim/mid/bright fade, so the three line
   * slots reuse one accent and we lean on line density to show depth. */
  short line_dim8, line_mid8, line_brt8;
  short north8, south8, body8;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /* Electric — cyan field on black */
    {51, 87, 231, 196, 21, 250, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_RED,
     COLOR_BLUE, COLOR_WHITE},
    /* Plasma — magenta/violet */
    {93, 165, 207, 226, 57, 248, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE,
     COLOR_YELLOW, COLOR_BLUE, COLOR_WHITE},
    /* Fire — red/orange field */
    {124, 208, 226, 231, 21, 250, COLOR_RED, COLOR_RED, COLOR_YELLOW,
     COLOR_WHITE, COLOR_BLUE, COLOR_WHITE},
    /* Ocean — blue/teal */
    {25, 39, 123, 196, 226, 250, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE, COLOR_RED,
     COLOR_YELLOW, COLOR_WHITE},
    /* Matrix — green */
    {22, 46, 118, 196, 21, 250, COLOR_GREEN, COLOR_GREEN, COLOR_WHITE,
     COLOR_RED, COLOR_BLUE, COLOR_WHITE},
};

/* Bind the six theme colours for the chosen theme (the two text colours
 * are set once in color_init and never touched here). */
static void theme_apply(int t) {
  const Theme *th = &k_themes[t];
  if (COLORS >= 256) {
    init_pair(CP_LINE_DIM, th->line_dim, COLOR_BLACK);
    init_pair(CP_LINE_MID, th->line_mid, COLOR_BLACK);
    init_pair(CP_LINE_BRT, th->line_brt, COLOR_BLACK);
    init_pair(CP_NORTH, th->north, COLOR_BLACK);
    init_pair(CP_SOUTH, th->south, COLOR_BLACK);
    init_pair(CP_BODY, th->body, COLOR_BLACK);
  } else {
    init_pair(CP_LINE_DIM, th->line_dim8, COLOR_BLACK);
    init_pair(CP_LINE_MID, th->line_mid8, COLOR_BLACK);
    init_pair(CP_LINE_BRT, th->line_brt8, COLOR_BLACK);
    init_pair(CP_NORTH, th->north8, COLOR_BLACK);
    init_pair(CP_SOUTH, th->south8, COLOR_BLACK);
    init_pair(CP_BODY, th->body8, COLOR_BLACK);
  }
}

/* Set up colours once at startup: lock in the fixed yellow/cyan text
 * colours, then load the starting theme. */
static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }

  theme_apply(0);
}

/* ── §4 magnetic physics ── */

/*
 * Monopole — a single magnetic "charge", one end of a bar magnet.
 *
 * We pretend each bar magnet is two opposite charges sitting at its ends: a
 * north (lines flow out of it) and a south (lines flow into it). Each charge
 * tugs on the field everywhere, and the real field is just all those tugs
 * added together. That adding-up is field_at(), the busiest part of the
 * program.
 *
 * Why two separate charges instead of one neat magnet formula? Because a flat
 * list of charges makes the adding-up trivial — one loop handles any number
 * of magnets at any angle — and the neat formula actually misbehaves up close
 * between two poles, which is exactly where this demo gets interesting (the
 * dead spots and X-crossings). Purcell's textbook uses the same trick.
 *
 * (Lone magnetic charges have never actually been found, but from far enough
 * away a real magnet looks just like this two-charge stand-in.)
 */
typedef struct {
  float cx, cy; /* where this charge sits, in grid cells. The math and the
                 * drawing share one grid, so no conversion is ever needed.
                 * Floats, because lines step in fractions of a cell. */
  float q;      /* the charge: +1 for north (lines flow out), -1 for south
                 * (lines flow in). Only the sign and the balance between poles
                 * matter for the line shapes — the actual size cancels out
                 * when we trace, so we just use +1 / -1. */
} Monopole;

/*
 * Dipole — a whole bar magnet, used only for drawing.
 *
 * Each magnet is stored two ways. The math sees it as its two charges in the
 * charge list; the drawing sees it as one magnet here. That looks like
 * duplication, but it's on purpose: the math just wants a plain list of
 * charges to add up (a "magnet body" means nothing to it — the body has no
 * charge), while the drawing needs to know which north and which south belong
 * together so it can draw the bar and the two letters as one shape. Both
 * copies are written together when the magnet is added, so they can't fall
 * out of step.
 */
typedef struct {
  float nx, ny; /* north pole position, in cells. Same spot as this magnet's
                 * +1 charge — kept here so the drawing doesn't have to go
                 * hunting for it in the charge list. */
  float sx, sy; /* south pole position, in cells (the -1 charge). */
  int color;    /* colour of the '=' bar. Always CP_BODY today; kept as a
                 * field so a future preset could tint magnets individually
                 * (say, red for the active one). */
} Dipole;

/*
 * The field at one point: add up the pull of every charge.
 *
 * Each charge tugs the field toward or away from itself, harder when it's
 * close. We sum those tugs to get the combined arrow (bx, by) at (px, py).
 * The SOFT fudge in r2 keeps things sane right on top of a charge, where
 * the raw formula would otherwise blow up to infinity. The y stretch by
 * ASPECT_R is because terminal cells are taller than wide, so a "round"
 * field needs the vertical distance scaled to match.
 */
static void field_at(const Monopole *mp, int nm, float px, float py, float *bx,
                     float *by) {
  *bx = 0.0f;
  *by = 0.0f;
  for (int i = 0; i < nm; i++) {
    float dx = px - mp[i].cx;
    float dy = (py - mp[i].cy) * ASPECT_R; /* stretch y so cells look square */
    float r2 = dx * dx + dy * dy + SOFT * SOFT;
    float r = sqrtf(r2);
    float inv3 = mp[i].q / (r2 * r);
    *bx += inv3 * dx;
    *by += inv3 * dy / ASPECT_R;
  }
}

/* ── §5 scene — field lines & dipoles ── */

/*
 * FieldLine — one finished field line, already turned into cells to draw.
 *
 * A field line is the curving path you'd walk if you always stepped in the
 * direction the field points — the same shape iron filings make around a
 * magnet. We start near a north pole and creep along the field, dropping a
 * character in each cell we pass, until one of three things happens: we
 * walk off the screen, the field fades to nothing (a "dead spot" where it
 * has no direction to follow), or we hit the step limit. Each step is the
 * same small length no matter how strong the field is, so a line near a
 * pole doesn't leap ahead while a faraway one barely crawls.
 *
 * We trace each line ONCE and store the finished cells here, rather than
 * re-walking the field every frame. Tracing is slow (hundreds of steps,
 * four field samples each); redrawing stored cells is just a tight loop of
 * "put this character here", with no math. So a line redraws for free
 * every frame until the scene changes.
 *
 * The four arrays (col, row, ch, cp) are kept side by side, one per cell,
 * sharing the same index. It reads clearly — "this is the column array" —
 * and keeps each kind of value packed together for fast scanning.
 *
 * (The step-along-the-field idea is classic streamline tracing, Numerical
 * Recipes ch.17; the dead spots are the critical points of Helman &
 * Hesselink 1991.)
 *
 * Cost: about 4.8 KB per line, ~310 KB across the whole scene. It all
 * lives inside Scene with no run-time allocation.
 */
typedef struct {
  /* All four arrays line up by index: 0 is the seed cell near the north
   * pole, len-1 is the last cell. Don't read past len — those slots are
   * garbage. */
  int col[MAX_LINE_STEPS]; /* screen column of each cell */
  int row[MAX_LINE_STEPS]; /* screen row of each cell    */
  char ch[MAX_LINE_STEPS]; /* the character drawn there: usually a slanted
                            * line piece ('- | / \') showing the line's
                            * tilt, but every so often an arrow
                            * ('> v < ^') so you can see which way the
                            * field flows, not just the shape. */
  int cp[MAX_LINE_STEPS];  /* colour of each cell — bright near the pole,
                            * fading along the line. Filled in after the
                            * whole line is traced, since the fade is
                            * split by total length. */
  int len;                 /* how many cells this line actually used.
                            * 0 means the seed started on a dead spot or
                            * off-screen; the line is still "done". */
  bool done;               /* set true once tracing finishes. Lets the
                            * draw pass know the line is safe to paint. */
} FieldLine;

/*
 * Scene — everything the program needs to show one picture.
 *
 * A picture is one preset (which magnets, where) shown in one theme at one
 * terminal size. Change any of those and we throw the whole thing away and
 * rebuild it from scratch, so every field below lives and dies together.
 *
 * The fields are grouped by who uses them, in the order each frame touches
 * them — physics first, then the things we draw, then the user's settings,
 * then the screen size. Each group is filled at build time and then mostly
 * left alone, so it's easy to see what's frozen and what changes per frame.
 *
 * It's one big struct on purpose. Since everything rebuilds together,
 * keeping it in one place makes building it a single clear-and-fill, makes
 * every field's lifetime obvious, and means no run-time allocation at all
 * (the whole thing is roughly 310 KB, almost all of it the line cells).
 */
typedef struct {
  /* The magnetic charges — what the field math reads. Just a flat list, in
   * any order, since the field is the same however you add up the pulls.
   * Two charges per magnet, so nm is always even. */
  Monopole mp[MAX_MONOPOLES]; /* the charges, 2 per magnet */
  int nm;                     /* how many charges are live */

  /* The magnet bodies — what the drawing code reads to place the '=' bar
   * and the N/S letters. Kept apart from the charges above so each side
   * stays simple: the math sees a plain list of charges, the drawing sees
   * whole magnets. */
  Dipole dp[MAX_DIPOLES]; /* the magnet bodies */
  int nd;                 /* how many magnets are live */

  FieldLine lines[MAX_LINES]; /* all the field lines, traced once at build */
  int n_lines;                /* how many lines this preset traced */
  int lines_traced;           /* how many we've shown so far. We reveal a few
                               * more each tick instead of all at once, so the
                               * lines appear to grow rather than pop in. */

  /* The user's live settings — what the keys change. Nothing in the field
   * math depends on these; changing the preset or theme rebuilds the scene
   * anyway. */
  int lines_per_tick; /* how many new lines to reveal each tick — pure
                       * pacing, the tracing is already done */
  bool paused;        /* freeze the reveal. The field never moves, so all
                       * "pause" does is stop revealing more lines */
  int preset;         /* which arrangement of magnets is showing */
  int theme;          /* which colour scheme is showing — swapping it just
                       * recolours, it doesn't re-trace the lines */

  /* The drawing area, in cells. This is the band BETWEEN the two HUD rows,
   * not the whole terminal: we trim a row off the top and bottom for the
   * status and key-hint lines. Set once per build/resize. */
  int cols, rows;
} Scene;

/* Pick an arrow ('> v < ^') for which way the field points here. We only
 * need the four compass directions — that's enough for the eye to read the
 * flow along a line. */
static char direction_arrow(float dx, float dy) {
  float angle = atan2f(dy * ASPECT_R, dx);
  if (angle < 0)
    angle += 2.0f * (float)M_PI;
  int sector = (int)(angle / ((float)M_PI / 4.0f) + 0.5f) % 8;
  static const char sym[4] = {'>', 'v', '<', '^'};
  return sym[sector % 4];
}

/* Pick a slanted line piece ('- \ | /') that matches the field's tilt here,
 * so the trail looks like a smooth curve. */
static char line_char(float dx, float dy) {
  float angle = atan2f(dy * ASPECT_R, dx);
  if (angle < 0)
    angle += (float)M_PI;
  if (angle >= (float)M_PI)
    angle -= (float)M_PI;
  float deg = angle * 180.0f / (float)M_PI;
  if (deg < 22.5f || deg >= 157.5f)
    return '-';
  if (deg < 67.5f)
    return '\\';
  if (deg < 112.5f)
    return '|';
  return '/';
}

/* Brightest where a line leaves its pole, fading along the way. We split
 * the line into thirds and colour each third. */
static int line_color_pair(int step, int total) {
  if (total <= 0)
    return CP_LINE_DIM;
  int third = total / 3;
  if (step < third)
    return CP_LINE_BRT;
  if (step < 2 * third)
    return CP_LINE_MID;
  return CP_LINE_DIM;
}

/* ── Helpers for trace_line — one small step of following a line ── */

/* Are we still on screen? We walk in sub-cell amounts, so check both the
 * raw position and the rounded cell (rounding can nudge a borderline point
 * over the edge). Hands back the rounded cell so the caller doesn't redo it. */
static bool cell_in_grid(float px, float py, int cols, int rows, int *out_col,
                         int *out_row) {
  if (px < 0 || px >= (float)cols || py < 0 || py >= (float)rows)
    return false;
  int col = (int)(px + 0.5f);
  int row = (int)(py + 0.5f);
  if (col < 0 || col >= cols || row < 0 || row >= rows)
    return false;
  *out_col = col;
  *out_row = row;
  return true;
}

/* Work out which way to step next, accurately. Instead of just sampling the
 * field once, we sample it four times around the next little stretch and
 * average them — a standard recipe (RK4, Numerical Recipes ch.17) that
 * follows curves much more faithfully than a single sample would. We then
 * shrink the answer to a fixed length so every step covers the same
 * distance; otherwise a strong field near a pole would fling the line too
 * far. Returns false at a dead spot (field too weak to point anywhere),
 * which tells the caller to stop the line. */
static bool rk4_unit_tangent(const Monopole *mp, int nm, float px, float py,
                             float h, float *tx, float *ty) {
  float k1x, k1y, k2x, k2y, k3x, k3y, k4x, k4y;
  field_at(mp, nm, px, py, &k1x, &k1y);
  field_at(mp, nm, px + 0.5f * h * k1x, py + 0.5f * h * k1y, &k2x, &k2y);
  field_at(mp, nm, px + 0.5f * h * k2x, py + 0.5f * h * k2y, &k3x, &k3y);
  field_at(mp, nm, px + h * k3x, py + h * k3y, &k4x, &k4y);

  float bx = (k1x + 2.0f * k2x + 2.0f * k3x + k4x) / 6.0f;
  float by = (k1y + 2.0f * k2y + 2.0f * k3y + k4y) / 6.0f;
  float bmag = sqrtf(bx * bx + by * by);
  if (bmag < B_MIN)
    return false; /* dead spot — nowhere to step, stop */

  *tx = bx / bmag; /* shrink to a fixed step length */
  *ty = by / bmag;
  return true;
}

/* Most cells get a line piece for shape; every so often we drop an arrow
 * instead so you can see which way the field flows. Never on the very first
 * cell — that sits on the 'N' marker and would clash with it. */
static char glyph_for_step(int step, float tx, float ty) {
  bool is_arrow_step = (step > 0 && step % ARR_STRIDE == 0);
  return is_arrow_step ? direction_arrow(tx, ty) : line_char(tx, ty);
}

/* Tack one more cell onto the line. Colour is left blank for now; we fill
 * the fade in afterwards, once we know how long the line ended up. */
static void record_traced_cell(FieldLine *fl, int col, int row, char ch) {
  fl->col[fl->len] = col;
  fl->row[fl->len] = row;
  fl->ch[fl->len] = ch;
  fl->cp[fl->len] = 0;
  fl->len++;
}

/* Colour the finished line so it's bright at the pole and fades toward the
 * end. Done after tracing, since the fade is measured against the line's
 * final length. */
static void apply_distance_brightness_ramp(FieldLine *fl) {
  for (int i = 0; i < fl->len; i++) {
    fl->cp[i] = line_color_pair(i, fl->len);
  }
}

/* Trace one whole field line: start at the seed and keep stepping in the
 * field's direction, dropping a character each step, until we leave the
 * screen, hit a dead spot, or run out of steps. Then colour the trail. */
static void trace_line(FieldLine *fl, const Monopole *mp, int nm, float sx,
                       float sy, int cols, int rows) {
  fl->len = 0;
  fl->done = true;

  float px = sx, py = sy;

  for (int step = 0; step < MAX_LINE_STEPS; step++) {
    int col, row;
    if (!cell_in_grid(px, py, cols, rows, &col, &row))
      break;

    float tx, ty;
    if (!rk4_unit_tangent(mp, nm, px, py, RK4_H, &tx, &ty))
      break;

    record_traced_cell(fl, col, row, glyph_for_step(step, tx, ty));

    px += RK4_H * tx;
    py += RK4_H * ty;
  }

  apply_distance_brightness_ramp(fl);
}

/* Add one magnet: record its two charges (for the math) and its body
 * geometry (for drawing) together, so the two views can't drift apart. */
static void scene_add_dipole(Scene *s, float nx, float ny, float sx, float sy) {
  if (s->nd >= MAX_DIPOLES || s->nm + 2 > MAX_MONOPOLES)
    return;

  s->mp[s->nm] = (Monopole){nx, ny, +1.0f};
  s->nm++;
  s->mp[s->nm] = (Monopole){sx, sy, -1.0f};
  s->nm++;

  s->dp[s->nd] = (Dipole){nx, ny, sx, sy, CP_BODY};
  s->nd++;
}

/* Start the field lines. Around each north pole we sprinkle a ring of
 * starting points and trace one line out of each. We do all the tracing now;
 * the per-tick reveal later just decides how many to show. */
static void scene_seed_lines(Scene *s) {
  s->n_lines = 0;
  s->lines_traced = 0;

  for (int i = 0; i < s->nm && s->n_lines < MAX_LINES; i++) {
    if (s->mp[i].q < 0)
      continue; /* lines start at north poles only */

    for (int k = 0; k < N_SEEDS && s->n_lines < MAX_LINES; k++) {
      float angle = (float)k / (float)N_SEEDS * 2.0f * (float)M_PI;
      float sx = s->mp[i].cx + SEED_R * cosf(angle);
      float sy = s->mp[i].cy + SEED_R * sinf(angle) / ASPECT_R;

      trace_line(&s->lines[s->n_lines], s->mp, s->nm, sx, sy, s->cols, s->rows);
      s->n_lines++;
    }
  }
}

/* ── Preset layouts — one helper per magnet arrangement ──
 *
 * Each one places its magnets relative to the centre (cx, cy) using offsets
 * scaled to the terminal size, so the picture fits any window. The names are
 * the textbook names for these classic setups (Purcell ch.6).
 */

/* One bar magnet lying flat. The plainest case: lines loop out of N, curve
 * around, and come back into S. */
static void preset_single_bar_magnet(Scene *s, float cx, float cy, float dx) {
  scene_add_dipole(s, cx - dx, cy, /* north pole — left */
                   cx + dx, cy);   /* south pole — right */
}

/* Two magnets crossing at the centre, one upright and one flat. The four
 * poles together leave an X-shaped dead spot in the middle where the lines
 * split apart. */
static void preset_quadrupole(Scene *s, float cx, float cy, float dx,
                              float dy) {
  scene_add_dipole(s, cx, cy - dy * 0.8f, /* upright magnet: N on top */
                   cx, cy + dy * 0.8f);   /*                 S on bottom */
  scene_add_dipole(s, cx - dx * 1.4f, cy, /* flat magnet: N far-left */
                   cx + dx * 1.4f, cy);   /*              S far-right */
}

/* Two magnets in a row with opposite poles facing (N-S  N-S). They attract,
 * and act like one long magnet: lines run straight through from the far-left
 * N to the far-right S. */
static void preset_attract_pair(Scene *s, float cx, float cy, float dx) {
  scene_add_dipole(s, cx - dx * 1.8f,
                   cy, /* left magnet:  N far-left,  S near-centre */
                   cx - dx * 0.4f, cy);
  scene_add_dipole(s, cx + dx * 0.4f,
                   cy, /* right magnet: N near-centre, S far-right */
                   cx + dx * 1.8f, cy);
}

/* Two magnets in a row with like poles facing (N-N). They repel: the two
 * inner N poles shove each other away, leaving a dead spot between them and
 * lines that swerve outward without crossing the middle. */
static void preset_repel_pair(Scene *s, float cx, float cy, float dx) {
  scene_add_dipole(s, cx - dx * 0.4f,
                   cy, /* left magnet:  N near-centre, S far-left  */
                   cx - dx * 1.8f, cy);
  scene_add_dipole(s, cx + dx * 0.4f,
                   cy, /* right magnet: N near-centre, S far-right */
                   cx + dx * 1.8f, cy);
}

/* Lay out the magnets for whichever preset is selected. */
static void scene_build_preset(Scene *s) {
  /* Centre of the screen and how far out to push the magnets, both scaled to
   * the window so the layout fits any size. */
  float cx = (float)s->cols * 0.5f;
  float cy = (float)s->rows * 0.5f;
  float dx = (float)s->cols * 0.18f;
  float dy = (float)s->rows * 0.28f;

  s->nm = 0;
  s->nd = 0;

  switch (s->preset) {
  case 0:
    preset_single_bar_magnet(s, cx, cy, dx);
    break;
  case 1:
    preset_quadrupole(s, cx, cy, dx, dy);
    break;
  case 2:
    preset_attract_pair(s, cx, cy, dx);
    break;
  case 3:
    preset_repel_pair(s, cx, cy, dx);
    break;
  }
}

static void scene_init(Scene *s, int cols, int rows, int preset, int theme) {
  memset(s, 0, sizeof *s);
  s->cols = cols;
  s->rows = rows;
  s->preset = preset;
  s->theme = theme;
  s->lines_per_tick = LINES_PER_TICK_DEF;

  scene_build_preset(s);
  scene_seed_lines(s);
}

/* One tick: show a few more of the already-traced lines, unless paused. */
static void scene_tick(Scene *s) {
  if (s->paused)
    return;
  int to_reveal = s->lines_per_tick;
  while (to_reveal-- > 0 && s->lines_traced < s->n_lines) {
    s->lines_traced++;
  }
}

/* ── Render helpers — one per thing we draw ──
 *
 * We paint back to front: field lines first, then the magnets on top, so the
 * bold N/S letters never get buried under a line. Everything works in the
 * drawing band's own coordinates; paint_cell nudges each row down by one to
 * leave the top HUD row clear.
 */

/* Put one character in one cell, ignoring anything off the drawing area. */
static void paint_cell(int row, int col, char ch, int attr, int grid_cols,
                       int grid_rows) {
  if (row < 0 || row >= grid_rows || col < 0 || col >= grid_cols)
    return;
  mvaddch(row + HUD_TOP_ROWS, col, (chtype)(unsigned char)ch | attr);
}

/* Draw one field line, cell by cell. The colours were worked out when the
 * line was traced, so this is just placing characters — no math. */
static void draw_field_line(const FieldLine *fl, int grid_cols, int grid_rows) {
  for (int i = 0; i < fl->len; i++) {
    int cp = fl->cp[i] ? fl->cp[i] : CP_LINE_DIM;
    paint_cell(fl->row[i], fl->col[i], fl->ch[i], COLOR_PAIR(cp), grid_cols,
               grid_rows);
  }
}

/* Draw only the lines revealed so far; the rest are still waiting their turn
 * in the grow-in animation. */
static void draw_revealed_field_lines(const Scene *s) {
  for (int li = 0; li < s->lines_traced; li++) {
    draw_field_line(&s->lines[li], s->cols, s->rows);
  }
}

/* Draw the '=' bar joining a magnet's two poles. We step along it twice per
 * cell so it stays unbroken at any angle, and keep it dim so it sits quietly
 * behind the N/S letters. */
static void draw_magnet_body_line(const Dipole *dp, int grid_cols,
                                  int grid_rows) {
  float dx = dp->sx - dp->nx;
  float dy = dp->sy - dp->ny;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist <= 0.5f)
    return; /* poles on the same spot — nothing to draw */

  int steps = (int)(dist * 2);
  int attr = COLOR_PAIR(CP_BODY) | A_DIM;
  for (int k = 0; k <= steps; k++) {
    float t = (float)k / (float)steps; /* 0 at N, 1 at S */
    int c = (int)(dp->nx + t * dx + 0.5f);
    int r = (int)(dp->ny + t * dy + 0.5f);
    paint_cell(r, c, '=', attr, grid_cols, grid_rows);
  }
}

/* Draw a bold 'N' or 'S' on a pole. Painted last so the letter always shows
 * clearly over the bar. */
static void draw_pole_marker(float px, float py, char glyph, int cp,
                             int grid_cols, int grid_rows) {
  int r = (int)(py + 0.5f);
  int c = (int)(px + 0.5f);
  paint_cell(r, c, glyph, COLOR_PAIR(cp) | A_BOLD, grid_cols, grid_rows);
}

/* Draw one whole magnet: the bar, then both pole letters. */
static void draw_magnet(const Dipole *dp, int grid_cols, int grid_rows) {
  draw_magnet_body_line(dp, grid_cols, grid_rows);
  draw_pole_marker(dp->nx, dp->ny, 'N', CP_NORTH, grid_cols, grid_rows);
  draw_pole_marker(dp->sx, dp->sy, 'S', CP_SOUTH, grid_cols, grid_rows);
}

/* Paint the whole picture: field lines first, then the magnets on top. */
static void scene_draw(const Scene *s) {
  draw_revealed_field_lines(s);

  for (int i = 0; i < s->nd; i++) {
    draw_magnet(&s->dp[i], s->cols, s->rows);
  }
}

/* ── §6 screen / HUD ── */

static void screen_init(void) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
}

/* Draw the two text rows: a status line up top (what's showing, how far
 * along, fps) and the key reminders along the bottom. Both stay bright so
 * they read over any field colour.
 *
 * It takes the full terminal height separately because the scene's own row
 * count is just the band between these two lines, not the whole screen. */
static void screen_draw_hud(const Scene *s, int cols, int term_rows, int fps) {
  static const char *preset_names[N_PRESETS] = {"Dipole", "Quadrupole",
                                                "Attract", "Repel"};
  static const char *theme_names[N_THEMES] = {"Electric", "Plasma", "Fire",
                                              "Ocean", "Matrix"};

  int pct = s->n_lines > 0 ? s->lines_traced * 100 / s->n_lines : 100;

  /* ── top status line ── */
  move(0, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, 0,
           " MagField  preset:%-10s  theme:%-8s  speed:%d"
           "  progress:%3d%%  %2dfps  %s",
           preset_names[s->preset], theme_names[s->theme], s->lines_per_tick,
           pct, fps, s->paused ? "PAUSED " : "running");
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── bottom key-hint line ── */
  int bot = term_rows - 1;
  move(bot, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(bot, 0,
           " q:quit  r:reset  n/N:preset  t/T:theme  [/]:speed"
           "  p|spc:pause ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);

  (void)cols; /* unused here, kept for a consistent signature */
}

/* ── §7 signal handling ── */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit = 0;

static void handle_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void handle_sigterm(int s) {
  (void)s;
  g_quit = 1;
}

/* ── §8 main loop ── */

/* ── Bootstrap helpers ── */

/* Catch the quit and resize signals. The handlers only set a flag; the real
 * work happens back in the main loop, where it's safe. */
static void install_signal_handlers(void) {
  signal(SIGWINCH, handle_sigwinch);
  signal(SIGTERM, handle_sigterm);
  signal(SIGINT, handle_sigterm);
}

/* Build the scene to fit the current terminal, reserving the top and bottom
 * rows for the HUD. The one place that subtracts those rows. */
static void rebuild_scene_for_terminal(Scene *scene, int term_rows,
                                       int term_cols, int preset, int theme) {
  scene_init(scene, term_cols, term_rows - HUD_TOP_ROWS - HUD_BOT_ROWS, preset,
             theme);
}

/* Step a value forward or back by one and wrap around the ends. Named so the
 * key handlers read clearly instead of spelling out the wrap arithmetic. */
static void cycle_index(int *value, int step, int n) {
  *value = (*value + n + step) % n;
}

/* ── Per-frame stage helpers — one phase of the loop each ── */

/* Handle a window resize. If one happened, grab the new size and rebuild the
 * scene, since the lines are traced to fit the old grid. */
static void consume_resize_event(Scene *scene, int *rows, int *cols, int preset,
                                 int theme) {
  if (!g_resize)
    return;
  g_resize = 0;
  endwin();
  refresh();
  getmaxyx(stdscr, *rows, *cols);
  rebuild_scene_for_terminal(scene, *rows, *cols, preset, theme);
}

/* Handle every key waiting in the queue. */
static void process_input(Scene *scene, int *cur_preset, int *cur_theme,
                          int *sim_fps, int rows, int cols) {
  int ch;
  while ((ch = getch()) != ERR) {
    switch (ch) {
    case 'q':
    case 27: /* 27 = ESC */
      g_quit = 1;
      break;
    case 'r': /* reset current preset */
      rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, *cur_theme);
      break;
    case 'n': /* next preset */
      cycle_index(cur_preset, +1, N_PRESETS);
      rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, *cur_theme);
      break;
    case 'N': /* previous preset */
      cycle_index(cur_preset, -1, N_PRESETS);
      rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, *cur_theme);
      break;
    case 't': /* next theme (no rebuild) */
      cycle_index(cur_theme, +1, N_THEMES);
      scene->theme = *cur_theme;
      theme_apply(*cur_theme);
      break;
    case 'T': /* previous theme */
      cycle_index(cur_theme, -1, N_THEMES);
      scene->theme = *cur_theme;
      theme_apply(*cur_theme);
      break;
    case ']': /* reveal faster */
      if (scene->lines_per_tick < LINES_PER_TICK_MAX)
        scene->lines_per_tick++;
      break;
    case '[': /* reveal slower */
      if (scene->lines_per_tick > LINES_PER_TICK_MIN)
        scene->lines_per_tick--;
      break;
    case 'p':
    case ' ': /* pause toggle */
      scene->paused = !scene->paused;
      break;
    case '+':
    case '=': /* sim Hz up */
      if (*sim_fps < SIM_FPS_MAX)
        *sim_fps += SIM_FPS_STEP;
      break;
    case '-': /* sim Hz down */
      if (*sim_fps > SIM_FPS_MIN)
        *sim_fps -= SIM_FPS_STEP;
      break;
    }
  }
}

/* Draw one frame. The wnoutrefresh + doupdate pair sends a single update
 * instead of a full redraw, which avoids flicker on slow terminals. */
static void render_frame(const Scene *scene, int rows, int cols, int fps_disp) {
  erase();
  scene_draw(scene);
  screen_draw_hud(scene, cols, rows, fps_disp);
  wnoutrefresh(stdscr);
  doupdate();
}

/* Hold the frame rate steady. Sleep off whatever time is left in this frame's
 * budget, then read the clock again to start the next frame's. Also reports
 * how long the work took, so the fps counter can reuse that number. */
static int64_t pace_frame_to_fps(int64_t t_last, int sim_fps,
                                 int64_t *out_work_ns) {
  int64_t t_now = clock_ns();
  int64_t t_work = t_now - t_last;
  int64_t t_tick = TICK_NS(sim_fps);
  clock_sleep_ns(t_tick - t_work); /* a negative gap just means "don't sleep" */
  *out_work_ns = t_work;
  return clock_ns();
}

/* Measure the frame rate. Add up time across frames and, every half second,
 * report how many frames fit in that window (doubled to get per-second). */
static void update_fps_counter(int64_t work_ns, int sim_fps, int64_t *fps_acc,
                               int *fps_cnt, int *fps_disp) {
  int64_t t_tick = TICK_NS(sim_fps);
  int64_t slack = t_tick - work_ns;
  *fps_acc += work_ns + (slack > 0 ? slack : 0);
  (*fps_cnt)++;
  if (*fps_acc >= NS_PER_SEC / 2) {
    *fps_disp = *fps_cnt * 2; /* counted over half a second, so double it */
    *fps_acc = 0;
    *fps_cnt = 0;
  }
}

/* When the current preset has fully drawn and isn't paused, pause on it for
 * about three seconds, then move to the next one. Makes the demo play itself;
 * the user can still flip presets by hand at any time. */
static void auto_advance_preset_if_complete(Scene *scene, int *cur_preset,
                                            int cur_theme, int sim_fps,
                                            int rows, int cols) {
  if (!(scene->lines_traced >= scene->n_lines && !scene->paused))
    return;

  static int hold_ticks = 0;
  hold_ticks++;
  if (hold_ticks >= sim_fps * 3) {
    hold_ticks = 0;
    cycle_index(cur_preset, +1, N_PRESETS);
    rebuild_scene_for_terminal(scene, rows, cols, *cur_preset, cur_theme);
  }
}

/* Set everything up, then loop: handle resize and keys, reveal a few more
 * lines, draw, wait out the rest of the frame, and auto-advance when a preset
 * finishes. */
int main(void) {
  install_signal_handlers();
  screen_init();
  color_init();

  int rows, cols;
  getmaxyx(stdscr, rows, cols);

  int sim_fps = SIM_FPS_DEFAULT;
  int cur_preset = 0;
  int cur_theme = 0;

  Scene scene;
  rebuild_scene_for_terminal(&scene, rows, cols, cur_preset, cur_theme);

  int64_t t_last = clock_ns();
  int64_t fps_acc = 0;
  int fps_cnt = 0;
  int fps_disp = 0;

  while (!g_quit) {
    consume_resize_event(&scene, &rows, &cols, cur_preset, cur_theme);
    process_input(&scene, &cur_preset, &cur_theme, &sim_fps, rows, cols);
    scene_tick(&scene);
    render_frame(&scene, rows, cols, fps_disp);

    int64_t work_ns;
    t_last = pace_frame_to_fps(t_last, sim_fps, &work_ns);
    update_fps_counter(work_ns, sim_fps, &fps_acc, &fps_cnt, &fps_disp);

    auto_advance_preset_if_complete(&scene, &cur_preset, cur_theme, sim_fps,
                                    rows, cols);
  }

  endwin();
  return 0;
}
