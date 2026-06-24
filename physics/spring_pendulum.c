/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * spring_pendulum.c  —  a weight on a stretchy spring, drawn in ASCII
 *
 * A weight hangs from a fixed point at the top from a spring that can both
 * swing side to side (like a pendulum) and bounce up and down (like a
 * spring). When the bounce is about twice as fast as the swing, the two
 * motions trade energy back and forth and the weight traces a looping
 * flower-like path before slowly settling.
 *
 * Ten built-in presets go from a plain swinging pendulum to wild,
 * never-repeating chaos. The math is the classic "elastic pendulum"; for
 * the full story see Lynch 2002 and Goldstein's "Classical Mechanics".
 *
 * Sister file: bounce_ball.c shares the pixel-vs-cell coordinate setup.
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

/* ── §1 config ── */

enum {
  SIM_FPS_DEFAULT = 120, /* the spring math needs tiny time steps to stay stable */
  SIM_FPS_MIN = 30,
  SIM_FPS_MAX = 120,

  FPS_UPDATE_MS = 500,

  N_COILS = 8, /* how many spring coils we draw between the top and the weight */

  N_THEMES = 10,
  N_PRESETS = 10,

  THEME_DEFAULT = 2,  /* OCEANIC */
  PRESET_DEFAULT = 4, /* RES-2:1, the classic looping-flower orbit */

  /* Two rows are reserved at the top for the readout, one at the bottom for
   * the key hint. The horizontal bar sits on the top reserved row; the point
   * the spring hangs from is one cell below it. */
  HUD_ROWS_TOP = 2,
  HUD_ROWS_BOT = 1,
};

/*
 * How many sub-pixels make up one terminal cell. A cell is about twice as
 * tall as it is wide, so we split it 8 wide by 16 tall and do all the physics
 * in these sub-pixels; only drawing rounds back to whole cells.
 */
#define CELL_W 8
#define CELL_H 16

/* How far the spring zigzag swings out to either side of the centre line. */
#define COIL_SPREAD (CELL_W * 2)

/*
 * How strongly the motion is slowed by drag. Each preset picks its own
 * starting value; [ and ] nudge it live, kept inside these limits.
 */
#define DAMPING_STEP 0.02f
#define DAMPING_MIN 0.00f
#define DAMPING_MAX 0.80f

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
 * The colour slots the program paints with. The four scene slots (the bar,
 * the straight wire, the spring, the weight) get repainted when you switch
 * themes. The two readout slots never change so the text always stays easy
 * to read on any animation: bright yellow for the status and parameters,
 * bright cyan for the bottom key hint.
 */
enum {
  CP_BAR = 1,
  CP_WIRE,
  CP_SPRING,
  CP_BALL,
  CP_HUD,
  CP_HINT,
};

/*
 * One colour scheme for the whole picture — picked from 10 named looks like
 * MATRIX or FIRE. Each scheme just names a colour for the four things that
 * change colour: the top bar, the straight wire, the spring, and the weight.
 * The readout colours are deliberately left out here so they never change.
 *
 * Two colours are listed for each thing because terminals differ. The 256
 * ones are used on modern terminals; the plain 8 (red, green, …) are the
 * fallback so it still looks right on an old 8-colour terminal. Every 256
 * colour was chosen from the bright half of the palette so even the dimmest
 * one stays visible against a black background.
 */
typedef struct {
  const char *name; /* short label shown in the readout */

  /* preferred colours on a 256-colour terminal */
  short bar256;    /* the horizontal bar across the top */
  short wire256;   /* the straight wire stubs joining bar, spring, and weight */
  short spring256; /* the spring coils and their connecting lines */
  short ball256;   /* the weight, drawn as "(@)" */

  /* fallback colours on an 8-colour terminal, same order */
  short bar8;
  short wire8;
  short spring8;
  short ball8;
} Theme;

/* The ten colour schemes you cycle through with t / T. */
static const Theme k_themes[N_THEMES] = {
    {"MATRIX", /* 256 */ 46, 28, 82, 154,
     /* 8   */ COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN},
    {"FIRE", /* 256 */ 220, 124, 208, 231,
     /* 8   */ COLOR_YELLOW, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
    {"OCEANIC", /* 256 */ 195, 31, 51, 231,
     /* 8   */ COLOR_CYAN, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE},
    {"NEON", /* 256 */ 207, 93, 200, 231,
     /* 8   */ COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
    {"MONO", /* 256 */ 231, 243, 250, 231,
     /* 8   */ COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
    {"ICE", /* 256 */ 195, 39, 51, 231,
     /* 8   */ COLOR_CYAN, COLOR_BLUE, COLOR_CYAN, COLOR_WHITE},
    {"NOVA", /* 256 */ 207, 92, 200, 231,
     /* 8   */ COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
    {"FOREST", /* 256 */ 154, 28, 100, 184,
     /* 8   */ COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW, COLOR_YELLOW},
    {"DESERT", /* 256 */ 222, 130, 208, 231,
     /* 8   */ COLOR_YELLOW, COLOR_RED, COLOR_YELLOW, COLOR_WHITE},
    {"ECLIPSE", /* 256 */ 219, 89, 197, 231,
     /* 8   */ COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
};

static void theme_apply(int ti) {
  const Theme *t = &k_themes[ti];
  bool full = (COLORS >= 256);
  init_pair(CP_BAR, full ? t->bar256 : t->bar8, -1);
  init_pair(CP_WIRE, full ? t->wire256 : t->wire8, -1);
  init_pair(CP_SPRING, full ? t->spring256 : t->spring8, -1);
  init_pair(CP_BALL, full ? t->ball256 : t->ball8, -1);
  init_pair(CP_HUD, full ? 226 : COLOR_YELLOW, -1);
  init_pair(CP_HINT, full ? 51 : COLOR_CYAN, -1);
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);
}

/* ── §4 coords ── */

/*
 * The one place we deal with cells being taller than they are wide. Physics
 * runs in fine sub-pixels; px_to_cell_x/y are the only spots that round back
 * to whole terminal cells. Same setup as bounce_ball.c.
 */
static inline int pw(int cols) { return cols * CELL_W; }
static inline int ph(int rows) { return rows * CELL_H; }

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* Forward declaration — preset table defined in §6, used by §5. */
typedef struct Preset Preset;

/* ── §5 pendulum ── */

/*
 * Pendulum — everything we know about the weight right now.
 *
 * We describe where the weight is using two numbers measured from the
 * pivot point at the top: how long the spring is (r) and which way it
 * leans (theta, the angle off straight-down). This pairing is natural
 * because those two things — stretch and swing — are exactly the two
 * motions a stretchy pendulum has. The screen position falls right out:
 *   x = pivot_x + r·sin(theta)
 *   y = pivot_y + r·cos(theta)   (y grows downward on a terminal)
 *
 * We keep speeds as well as positions because the motion equations are
 * the kind that need both to step forward in time. Storing the speeds
 * directly (rather than guessing them from the last position) is what
 * keeps the simulation's energy from quietly drifting off over minutes.
 *
 * prev_r / prev_theta are only a drawing aid — the physics never reads
 * them. Each step stashes the old position so the drawing code can slide
 * smoothly between the last step and the new one, so motion looks fluid
 * even though the physics ticks faster than the screen redraws.
 *
 * The spring constants live inside each weight (not as global #defines)
 * because every preset sets its own stiffness, gravity, and so on. That
 * way the step function only ever reads from the weight handed to it.
 *
 * For the physics of the swinging spring see Lynch 2002 and Goldstein's
 * Classical Mechanics, §1-2.
 */
typedef struct {
  /* Where the weight is, measured from the pivot. */
  float r;     /* spring length in pixels, always > 0                */
  float theta; /* lean angle off straight-down, radians (+ = right)  */

  /* How fast those two are changing right now. */
  float r_dot;  /* stretch speed, pixels per second                  */
  float th_dot; /* swing speed, radians per second                   */

  /* Last step's position, kept only so drawing can blend between
   * frames. The physics never touches these. */
  float prev_r;
  float prev_theta;

  /* The fixed point at the top the spring hangs from, in pixels.
   * Only recomputed when the terminal is resized. */
  float pivot_px; /* centre column of the field                     */
  float pivot_py; /* one cell below the top bar                     */

  /* The spring's character — each preset fills these in (weight = 1). */
  float r0;       /* the spring's relaxed length in pixels           */
  float spring_k; /* stiffness: bigger = snappier bounce             */
  float gravity;  /* downward pull, pixels per second squared        */
  float damping;  /* drag that slowly bleeds off both motions        */
} Pendulum;

static void pendulum_init(Pendulum *p, int cols, int rows, const Preset *pr);

/* Remember where the weight is before we move it, so drawing can slide
 * smoothly from the old spot to the new one. */
static inline void snapshot_state_for_interpolation(Pendulum *p) {
  p->prev_r = p->r;
  p->prev_theta = p->theta;
}

/* How fast the spring's stretch is changing — the up/down "bounce" push.
 * Four pulls add up: the swing trying to fling the weight outward, gravity
 * pulling along the spring, the spring pulling back toward its rest length,
 * and drag slowing the stretch. See Goldstein, Classical Mechanics §1-2. */
static inline float compute_radial_acceleration(const Pendulum *p) {
  return p->r * p->th_dot * p->th_dot + p->gravity * cosf(p->theta) -
         p->spring_k * (p->r - p->r0) - p->damping * p->r_dot;
}

/* How fast the swing is changing — the side-to-side push. Gravity tries to
 * pull the weight back toward straight-down, and stretching while swinging
 * speeds up or slows the swing (a skater pulling their arms in spins faster).
 * Dividing by the spring length is why a very short spring would misbehave,
 * which is what the length clamp below guards against. */
static inline float compute_angular_acceleration(const Pendulum *p) {
  return -(p->gravity * sinf(p->theta) + 2.f * p->r_dot * p->th_dot) / p->r -
         p->damping * p->th_dot;
}

/* Nudge the two speeds forward a tiny step using the pushes we just found.
 * Updating speeds BEFORE positions (next function) is the trick that keeps
 * the simulation's energy from slowly drifting off over long runs — the
 * weight keeps swinging believably for minutes. See Hairer, Geometric
 * Numerical Integration §VI. */
static inline void integrate_velocities_symplectic(Pendulum *p, float r_ddot,
                                                   float th_ddot, float dt) {
  p->r_dot += r_ddot * dt;
  p->th_dot += th_ddot * dt;
}

/* Now move the weight using the just-updated speeds. Using the new speeds
 * (not the old ones) is the second half of the energy-stable trick above. */
static inline void integrate_positions_from_new_velocities(Pendulum *p,
                                                           float dt) {
  p->r += p->r_dot * dt;
  p->theta += p->th_dot * dt;
}

/* Don't let the spring get too short or stretch too far. The swing math
 * blows up if the spring shrinks to nothing, so we pin the length between
 * sane limits. When the weight hits a limit we also stop it pushing further
 * into the wall, otherwise it would keep pumping energy every frame and the
 * wild presets (like SLINGSHOT) would spin out of control. */
static inline void enforce_radial_clamp_with_inelastic_stop(Pendulum *p) {
  const float r_min = p->r0 * 0.20f;
  const float r_max = p->r0 * 3.5f;
  if (p->r < r_min) {
    p->r = r_min;
    if (p->r_dot < 0.f)
      p->r_dot = 0.f;
  }
  if (p->r > r_max) {
    p->r = r_max;
    if (p->r_dot > 0.f)
      p->r_dot = 0.f;
  }
}

/*
 * Move the weight forward by one tiny time step. The order matters:
 *   remember where we are (so drawing can blend frames)
 *   work out the bounce push and the swing push
 *   update both speeds first, then the position (keeps energy stable)
 *   keep the spring length sane so the swing math doesn't blow up
 */
static void pendulum_tick(Pendulum *p, float dt) {
  snapshot_state_for_interpolation(p);

  float r_ddot = compute_radial_acceleration(p);
  float th_ddot = compute_angular_acceleration(p);

  integrate_velocities_symplectic(p, r_ddot, th_ddot, dt);
  integrate_positions_from_new_velocities(p, dt);

  enforce_radial_clamp_with_inelastic_stop(p);
}

/* ── §6 presets ── */

/*
 * Preset — one ready-made "flavour" of motion the user picks with n/p.
 *
 * A preset isn't just a starting position. It bundles the whole setup that
 * makes its motion look different: where the weight starts and how fast it's
 * already moving, plus the spring's character (stiffness, gravity, drag, rest
 * length). That full bundle matters — the difference between, say, the ELLIPSE
 * and CIRCLE flavours isn't only the starting spin, it's also how stiff the
 * spring is, which decides whether it springs back or stays nearly rigid.
 *
 * The table is ordered simplest → wildest so tapping n walks you through a
 * little tour: a plain swing, a plain bounce, then the two motions mixing in
 * ever more interesting ways, ending in chaos and a slow settle.
 *
 * The classic study of this "swinging spring" and its flavours is
 * Lynch, "The Swinging Spring" (2002).
 */
struct Preset {
  const char *name; /* short label shown in the readout          */

  /* Where the weight starts and how it's already moving. */
  float theta0_deg; /* starting lean angle, degrees off straight-down */
  float r_stretch;  /* starting length as a multiple of rest length (1.0 = relaxed) */
  float r_dot0;     /* starting bounce speed,  pixels per second  */
  float th_dot0;    /* starting swing speed,   radians per second */

  /* The spring's character (handed straight to the physics step). */
  float spring_k;      /* stiffness: bigger = snappier, faster bounce */
  float gravity;       /* downward pull, pixels per second squared    */
  float damping;       /* drag that slowly bleeds off the motion      */
  float rest_len_frac; /* relaxed length as a fraction of the field height */
};

static const Preset k_presets[N_PRESETS] = {
    /* 1 PENDULUM — the baseline. A very stiff spring barely stretches, so
     * almost all the motion is a clean side-to-side swing, like a clock
     * pendulum. The simplest thing to watch; everything else is a variation. */
    {"PENDULUM", 25.0f, 1.00f, 0.0f, 0.0f, 400.0f, 2000.0f, 0.05f, 0.40f},

    /* 2 SPRING — the opposite of PENDULUM. It starts hanging dead straight
     * with no sideways push, so it only bounces straight up and down and
     * never swings. Just a weight bobbing on a spring. */
    {"SPRING", 0.0f, 1.60f, 0.0f, 0.0f, 30.0f, 2000.0f, 0.05f, 0.40f},

    /* 3 ELLIPSE — give it a small sideways nudge on a fairly stiff spring and
     * the weight traces a slowly-turning oval, like a marble circling the rim
     * of a bowl. The simplest mix of swinging and a touch of stretch. */
    {"ELLIPSE", 30.0f, 1.00f, 0.0f, 2.0f, 120.0f, 2000.0f, 0.00f, 0.40f},

    /* 4 CIRCLE — spin it fast enough and the spring's pull-back exactly
     * balances the spin, so the weight whirls around in a steady circle, like
     * a ball on a string swung overhead. No drag, so it keeps going. */
    {"CIRCLE", 0.0f, 1.20f, 0.0f, 5.0f, 220.0f, 2000.0f, 0.00f, 0.30f},

    /* 5 RES-2:1 — the star of the show. The bounce here runs about twice as
     * fast as the swing, and at that magic ratio the two motions feed each
     * other: energy sloshes back and forth so the path keeps tightening into a
     * flower, reversing, and opening back up every ten seconds or so. */
    {"RES-2:1", 40.0f, 1.15f, 0.0f, 0.0f, 25.0f, 2000.0f, 0.06f, 0.40f},

    /* 6 RES-3:1 — same idea as RES-2:1 but the bounce runs three times as
     * fast as the swing. The two motions still trade energy, but more weakly,
     * so the flower is denser and the breathing in and out is gentler. */
    {"RES-3:1", 40.0f, 1.10f, 0.0f, 0.0f, 56.0f, 2000.0f, 0.06f, 0.40f},

    /* 7 PRECESS — a spinning oval whose long axis slowly rotates, so the whole
     * pattern turns like the hands of a clock. The tiny leftover bounce in a
     * stiff spring is what nudges it around, little by little. */
    {"PRECESS", 30.0f, 1.00f, 0.0f, 3.0f, 300.0f, 2000.0f, 0.00f, 0.35f},

    /* 8 SLINGSHOT — starts pulled way out with the spring loaded like a
     * catapult. It whips inward fast, gets caught by the short-length limit,
     * then bounces and swings as the stored energy spreads into the swing. */
    {"SLINGSHOT", 30.0f, 2.50f, 0.0f, 0.0f, 30.0f, 2000.0f, 0.04f, 0.35f},

    /* 9 CHAOS — start it almost upside-down with a spin and no drag. The
     * motion never repeats and never settles into a pattern; the smallest
     * change in the start would send it somewhere completely different. */
    {"CHAOS", 120.0f, 1.40f, 0.0f, 1.0f, 25.0f, 2000.0f, 0.00f, 0.40f},

    /* 10 DECAY — heavy drag. Whatever it starts doing, friction wins quickly
     * and the weight spirals to a dead stop. A calm "watch it settle" ending
     * to the tour. */
    {"DECAY", 80.0f, 1.50f, 0.0f, 1.5f, 25.0f, 2000.0f, 0.50f, 0.40f},
};

static void pendulum_init(Pendulum *p, int cols, int rows, const Preset *pr) {
  int field_rows = rows - HUD_ROWS_TOP - HUD_ROWS_BOT;
  if (field_rows < 4)
    field_rows = 4;

  p->pivot_px = (float)pw(cols) * 0.5f;
  p->pivot_py = (float)CELL_H * (float)(HUD_ROWS_TOP + 1);

  p->r0 = (float)ph(field_rows) * pr->rest_len_frac;
  p->r = p->r0 * pr->r_stretch;
  p->theta = (float)(pr->theta0_deg * M_PI / 180.0);
  p->r_dot = pr->r_dot0;
  p->th_dot = pr->th_dot0;
  p->spring_k = pr->spring_k;
  p->gravity = pr->gravity;
  p->damping = pr->damping;

  p->prev_r = p->r;
  p->prev_theta = p->theta;
}

/* ── §7 scene ── */

/*
 * Scene — the one place the physics and the drawing meet.
 *
 * The Pendulum doesn't know what a terminal is, and the drawing code doesn't
 * know any physics. Scene holds both plus the few odds and ends (which preset,
 * which theme, paused or not) that don't belong to either. The fields are
 * grouped by who reads them so it stays obvious which side touches what — if a
 * drawing-only field ever leaked into the physics step, the motion would start
 * depending on the window size, which is exactly the bug this layout prevents.
 *
 * Things that live elsewhere on purpose: frame timing is in main(), the
 * signal flags and sim rate are in App, the pivot point is in Pendulum, and
 * ncurses owns the actual screen buffer.
 */
typedef struct {
  /* Window size in cells — read by both the physics (to place the pivot and
   * size the spring) and the drawing (to clip and position the readout).
   * Only changes when the terminal is resized. */
  int cols;
  int rows;

  /* The physics state and the flags that gate it. preset_idx is really a user
   * choice, but it's grouped here because it picks which preset 'r' reloads
   * and it survives a theme change untouched. */
  Pendulum pend;
  int preset_idx; /* which preset, 0 .. N_PRESETS-1            */
  bool paused;    /* when true the physics step does nothing   */

  /* Pure drawing choice — changing the theme only repaints colours, it never
   * touches the physics. Survives a resize untouched. */
  int theme_idx; /* which colour scheme, 0 .. N_THEMES-1      */
} Scene;

static void scene_init(Scene *s, int cols, int rows, int preset_idx,
                       int theme_idx) {
  s->cols = cols;
  s->rows = rows;
  s->preset_idx = preset_idx;
  s->theme_idx = theme_idx;
  s->paused = false;
  pendulum_init(&s->pend, cols, rows, &k_presets[preset_idx]);
}

static void scene_tick(Scene *s, float dt) {
  if (!s->paused)
    pendulum_tick(&s->pend, dt);
}

/* ── drawing helpers ── */

/*
 * Bookkeeping for drawing a straight line one cell at a time, using only
 * integer math (Bresenham's line algorithm, 1965). It tracks how far apart
 * the endpoints are, which way to step on each axis, and a running tally that
 * decides when to step sideways versus down.
 */
typedef struct {
  int dx, dy; /* horizontal and vertical distance between the endpoints */
  int sx, sy; /* step direction on each axis, +1 or -1                  */
  int err;    /* running tally that triggers the next step              */
} BresenhamState;

/* Set up the line-drawing state for a line from (x0,y0) to (x1,y1).
 * Bresenham 1965, IBM Systems Journal. */
static inline void bresenham_init(BresenhamState *b, int x0, int y0, int x1,
                                  int y1) {
  b->dx = abs(x1 - x0);
  b->dy = abs(y1 - y0);
  b->sx = (x0 < x1) ? 1 : -1;
  b->sy = (y0 < y1) ? 1 : -1;
  b->err = b->dx - b->dy;
}

/* Pick the character for this step so the line looks connected at any angle:
 * a slash when moving diagonally, a dash when moving sideways, a bar when
 * moving straight down. */
static inline chtype slope_glyph(const BresenhamState *b) {
  int e2 = 2 * b->err;
  bool step_x = (e2 > -b->dy);
  bool step_y = (e2 < b->dx);
  if (step_x && step_y)
    return (b->sx == b->sy) ? '\\' : '/';
  if (step_x)
    return '-';
  (void)step_y;
  return '|';
}

/* Draw only inside the play area so the spring never paints over the readout
 * rows at the top or the key hint at the bottom. */
static inline void plot_clipped(int x, int y, chtype glyph, attr_t attr,
                                int cols, int rows) {
  int ymin = HUD_ROWS_TOP;
  int ymax = rows - HUD_ROWS_BOT;
  if (x < 0 || x >= cols || y < ymin || y >= ymax)
    return;
  attron(attr);
  mvaddch(y, x, glyph);
  attroff(attr);
}

/* Step one cell toward the endpoint — sideways, down, or both for a diagonal,
 * whichever the running tally calls for. */
static inline void bresenham_advance(BresenhamState *b, int *x, int *y) {
  int e2 = 2 * b->err;
  if (e2 > -b->dy) {
    b->err -= b->dy;
    *x += b->sx;
  }
  if (e2 < b->dx) {
    b->err += b->dx;
    *y += b->sy;
  }
}

/* Draw a straight line between two cells, stepping one cell at a time and
 * stopping at the endpoint, kept inside the play area. */
static void draw_line(int x0, int y0, int x1, int y1, int cols, int rows,
                      attr_t attr) {
  BresenhamState b;
  bresenham_init(&b, x0, y0, x1, y1);

  for (;;) {
    plot_clipped(x0, y0, slope_glyph(&b), attr, cols, rows);
    if (x0 == x1 && y0 == y1)
      break;
    bresenham_advance(&b, &x0, &y0);
  }
}

/* A length-and-angle pair — the smoothed position the drawing code uses. */
typedef struct {
  float r, theta;
} PolarSample;

/* Two directions for laying out the spring at a given lean: "axis" points
 * straight down the spring from the top to the weight, and "perp" points
 * sideways across it. We zigzag the coils along axis, offset by perp. */
typedef struct {
  float ax, ay;
  float perp_x, perp_y;
} SpringBasis;

/* ── geometry helpers ── */

/* Blend last step's position toward this step's, so motion looks smooth even
 * though the physics ticks faster than we redraw. alpha goes 0 → 1 as the next
 * tick approaches. See Fiedler, "Fix Your Timestep". */
static inline PolarSample lerp_polar_state(const Pendulum *p, float alpha) {
  PolarSample s;
  s.r = p->prev_r + (p->r - p->prev_r) * alpha;
  s.theta = p->prev_theta + (p->theta - p->prev_theta) * alpha;
  return s;
}

/* Turn a length-and-angle from the pivot into a cell on screen — basic
 * trigonometry, remembering that rows count downward. */
static inline void polar_to_cell(float pivot_px, float pivot_py, float r,
                                 float theta, int *cx, int *cy) {
  *cx = px_to_cell_x(pivot_px + r * sinf(theta));
  *cy = px_to_cell_y(pivot_py + r * cosf(theta));
}

/* Keep the drawn weight inside the play area. This only nudges where it's
 * drawn — the real physics position is left alone. */
static inline void clamp_bob_to_field(int *cx, int *cy, int cols, int rows) {
  int ymin = HUD_ROWS_TOP + 1;
  int ymax = rows - HUD_ROWS_BOT - 1;
  if (*cx < 1)
    *cx = 1;
  if (*cx > cols - 2)
    *cx = cols - 2;
  if (*cy < ymin)
    *cy = ymin;
  if (*cy > ymax)
    *cy = ymax;
}

/* Work out the down-the-spring and across-the-spring directions for the
 * current lean, ready for laying out the coils. */
static inline SpringBasis compute_spring_basis(float theta) {
  SpringBasis b;
  b.ax = sinf(theta);
  b.ay = cosf(theta);
  b.perp_x = -b.ay;
  b.perp_y = b.ax;
  return b;
}

/* Place the zigzag points of the spring evenly down its length, kicking each
 * one alternately left and right to make the coils. The very ends are left
 * just short so the short straight wires can join up to the pivot and weight. */
static inline void lay_out_zigzag_coils(const Pendulum *p, float draw_r,
                                        SpringBasis basis, float *node_px,
                                        float *node_py, int n_nodes) {
  for (int i = 0; i < n_nodes; i++) {
    float t = (float)(i + 1) / (float)(n_nodes + 1);
    float bx = p->pivot_px + t * draw_r * basis.ax;
    float by = p->pivot_py + t * draw_r * basis.ay;
    float sign = (i % 2 == 0) ? 1.f : -1.f;
    node_px[i] = bx + sign * COIL_SPREAD * basis.perp_x;
    node_py[i] = by + sign * COIL_SPREAD * basis.perp_y;
  }
}

/* ── layer painters ── */
/* Painted back to front, so each layer can cover up the one beneath it. */

/* The bar across the top the spring hangs from, with a 'v' marking the exact
 * spot the spring attaches. */
static inline void draw_pivot_bar(int cols, int pivot_cx) {
  attron(COLOR_PAIR(CP_BAR) | A_BOLD);
  for (int x = 0; x < cols; x++)
    mvaddch(HUD_ROWS_TOP, x, '=');
  if (pivot_cx >= 0 && pivot_cx < cols)
    mvaddch(HUD_ROWS_TOP, pivot_cx, 'v');
  attroff(COLOR_PAIR(CP_BAR) | A_BOLD);
}

/* A short straight wire — one joins the top bar to the spring, the other joins
 * the spring to the weight. Same routine, just different endpoints. */
static inline void draw_wire_stub(int x0, int y0, int x1, int y1, int cols,
                                  int rows) {
  draw_line(x0, y0, x1, y1, cols, rows, COLOR_PAIR(CP_WIRE));
}

/* Draw the zigzag legs of the spring by joining each coil point to the next. */
static inline void draw_coil_segments(const float *node_px,
                                      const float *node_py, int n_nodes,
                                      int cols, int rows) {
  for (int i = 0; i < n_nodes - 1; i++) {
    int x0 = px_to_cell_x(node_px[i]);
    int y0 = px_to_cell_y(node_py[i]);
    int x1 = px_to_cell_x(node_px[i + 1]);
    int y1 = px_to_cell_y(node_py[i + 1]);
    draw_line(x0, y0, x1, y1, cols, rows, COLOR_PAIR(CP_SPRING));
  }
}

/* Mark each coil point with a '*'. Drawn after the legs so the corners stay
 * clear even where several legs cross the same cell. */
static inline void draw_coil_nodes(const float *node_px, const float *node_py,
                                   int n_nodes, int cols, int rows) {
  int ymin = HUD_ROWS_TOP + 1;
  int ymax = rows - HUD_ROWS_BOT;
  attron(COLOR_PAIR(CP_SPRING) | A_BOLD);
  for (int i = 0; i < n_nodes; i++) {
    int cx = px_to_cell_x(node_px[i]);
    int cy = px_to_cell_y(node_py[i]);
    if (cx >= 0 && cx < cols && cy >= ymin && cy < ymax)
      mvaddch(cy, cx, '*');
  }
  attroff(COLOR_PAIR(CP_SPRING) | A_BOLD);
}

/* The weight itself, drawn as "(@)". Drawn last so it sits cleanly on top of
 * any wire or spring underneath. */
static inline void draw_iron_bob(int bob_cx, int bob_cy, int cols) {
  attron(COLOR_PAIR(CP_BALL) | A_BOLD);
  if (bob_cx > 0 && bob_cx < cols - 1) {
    mvaddch(bob_cy, bob_cx - 1, '(');
    mvaddch(bob_cy, bob_cx, '@');
    mvaddch(bob_cy, bob_cx + 1, ')');
  } else {
    mvaddch(bob_cy, bob_cx, '@');
  }
  attroff(COLOR_PAIR(CP_BALL) | A_BOLD);
}

/*
 * Draw one whole frame: figure out the smoothed position, work out where the
 * pivot, weight, and coils land on screen, then paint the bar, wires, spring,
 * and weight from back to front.
 */
static void scene_draw(const Scene *s, float alpha) {
  const Pendulum *p = &s->pend;
  const int cols = s->cols, rows = s->rows;

  PolarSample draw = lerp_polar_state(p, alpha);

  int pivot_cx = px_to_cell_x(p->pivot_px);
  int pivot_cy = px_to_cell_y(p->pivot_py);
  if (pivot_cy < HUD_ROWS_TOP + 1)
    pivot_cy = HUD_ROWS_TOP + 1;

  int bob_cx, bob_cy;
  polar_to_cell(p->pivot_px, p->pivot_py, draw.r, draw.theta, &bob_cx, &bob_cy);
  clamp_bob_to_field(&bob_cx, &bob_cy, cols, rows);

  SpringBasis basis = compute_spring_basis(draw.theta);

  const int N_NODES = N_COILS * 2;
  float node_px[N_COILS * 2];
  float node_py[N_COILS * 2];
  lay_out_zigzag_coils(p, draw.r, basis, node_px, node_py, N_NODES);

  draw_pivot_bar(cols, pivot_cx);
  {
    int nx0 = px_to_cell_x(node_px[0]);
    int ny0 = px_to_cell_y(node_py[0]);
    int nxN = px_to_cell_x(node_px[N_NODES - 1]);
    int nyN = px_to_cell_y(node_py[N_NODES - 1]);
    draw_wire_stub(pivot_cx, pivot_cy, nx0, ny0, cols, rows);
    draw_coil_segments(node_px, node_py, N_NODES, cols, rows);
    draw_wire_stub(nxN, nyN, bob_cx, bob_cy, cols, rows);
  }
  draw_coil_nodes(node_px, node_py, N_NODES, cols, rows);
  draw_iron_bob(bob_cx, bob_cy, cols);
}

/* ── §8 screen ── */

/*
 * Screen — just the terminal's size in cells.
 *
 * It's deliberately tiny. ncurses keeps the actual off-screen copy of the
 * picture for us; all we need to remember is how wide and tall the terminal
 * is, so the rest of the code knows where to clip and where to put the
 * readout rows.
 *
 * A frame goes: erase the off-screen copy, draw the spring and weight onto it,
 * draw the readout last so it always wins, then let ncurses send only the
 * cells that actually changed to the real terminal. Sending only the changes
 * is what keeps the display from flickering.
 */
typedef struct {
  int cols; /* terminal width in cells   */
  int rows; /* terminal height in cells  */
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

/*
 * Draw the readout: frame rate, preset, and paused state across the top right;
 * the current angle, stretch, and spring settings on the line below; and the
 * full list of keys along the bottom.
 */
static void hud_draw(Screen *s, const Scene *sc, double fps) {
  const Pendulum *p = &sc->pend;
  float deg = p->theta * (float)(180.0 / M_PI);
  /* Wrap the shown angle into -180..180 so the number stays sane even when the
   * weight spins round and round (as in CHAOS or CIRCLE). */
  while (deg > 180.f)
    deg -= 360.f;
  while (deg <= -180.f)
    deg += 360.f;
  float stretch = (p->r - p->r0) / p->r0 * 100.0f;

  /* top right: frame rate, preset, paused state */
  char status[80];
  int pn = sc->preset_idx + 1;
  snprintf(status, sizeof status, " %5.1f fps  PRESET %2d %-10s  %s ", fps, pn,
           k_presets[sc->preset_idx].name, sc->paused ? "PAUSED " : "running");
  int hx = s->cols - (int)strlen(status);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", status);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* second line: angle, stretch, and the spring settings */
  char params[160];
  snprintf(params, sizeof params,
           " theta:%+6.1f deg  dr:%+6.1f%%  damp:%.2f  "
           "k:%5.1f  g:%6.0f  theme:%s ",
           deg, stretch, p->damping, p->spring_k, p->gravity,
           k_themes[sc->theme_idx].name);
  attron(COLOR_PAIR(CP_HUD));
  mvprintw(1, 0, "%s", params);
  attroff(COLOR_PAIR(CP_HUD));

  /* bottom line: every key you can press */
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q:quit  spc:pause  r:reset  n/p:preset  t/T:theme  [/]:damp ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, const Scene *sc, double fps, float alpha) {
  erase();
  scene_draw(sc, alpha);
  hud_draw(s, sc, fps);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app ── */

/*
 * App — the one top-level box holding the whole program. There's a single
 * global copy so the signal handlers (which can't be handed a pointer) and the
 * main loop can both reach it.
 *
 * The two flags are how a signal talks to the loop: a handler flips a bit, the
 * loop checks it next time round and does the real work. They're typed and
 * marked the special way the C standard demands so a handler can safely touch
 * them and the loop always reads the fresh value instead of a stale copy.
 *
 * sim_fps lives here rather than in Scene because it's about how often we run
 * the physics, which is the loop's job, not the physics step's. The redraw
 * rate stays fixed at 60; only the physics rate is adjustable.
 */
typedef struct {
  Scene scene;                       /* the world plus user choices   */
  Screen screen;                     /* terminal size                 */
  int sim_fps;                       /* physics steps per second, 30..120 */
  volatile sig_atomic_t running;     /* quit signals clear this       */
  volatile sig_atomic_t need_resize; /* a resize signal sets this     */
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
  int preset = app->scene.preset_idx;
  int theme = app->scene.theme_idx;
  scene_init(&app->scene, app->screen.cols, app->screen.rows, preset, theme);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Pendulum *p = &app->scene.pend;
  Scene *sc = &app->scene;

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
    pendulum_init(p, sc->cols, sc->rows, &k_presets[sc->preset_idx]);
    break;

  /* Ignore repeats within 200 ms so holding n or p doesn't blur through every
   * preset at once; single taps always go through. */
  case 'n':
  case 'N': {
    static int64_t last_ns = 0;
    int64_t now = clock_ns();
    if (now - last_ns < 200 * NS_PER_MS)
      break;
    last_ns = now;
    sc->preset_idx = (sc->preset_idx + 1) % N_PRESETS;
    pendulum_init(p, sc->cols, sc->rows, &k_presets[sc->preset_idx]);
    break;
  }
  case 'p':
  case 'P': {
    static int64_t last_ns = 0;
    int64_t now = clock_ns();
    if (now - last_ns < 200 * NS_PER_MS)
      break;
    last_ns = now;
    sc->preset_idx = (sc->preset_idx + N_PRESETS - 1) % N_PRESETS;
    pendulum_init(p, sc->cols, sc->rows, &k_presets[sc->preset_idx]);
    break;
  }

  case 't':
    sc->theme_idx = (sc->theme_idx + 1) % N_THEMES;
    theme_apply(sc->theme_idx);
    break;
  case 'T':
    sc->theme_idx = (sc->theme_idx + N_THEMES - 1) % N_THEMES;
    theme_apply(sc->theme_idx);
    break;

  case ']':
    p->damping -= DAMPING_STEP;
    if (p->damping < DAMPING_MIN)
      p->damping = DAMPING_MIN;
    break;

  case '[':
    p->damping += DAMPING_STEP;
    if (p->damping > DAMPING_MAX)
      p->damping = DAMPING_MAX;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen, THEME_DEFAULT);
  scene_init(&app->scene, app->screen.cols, app->screen.rows, PRESET_DEFAULT,
             THEME_DEFAULT);

  int64_t frame_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* handle a pending resize */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* time since last frame, capped so a hiccup can't make us catch up forever */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    /* run the physics in fixed little steps, however many fit in this frame */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }
    float alpha = (float)sim_accum / (float)tick_ns;

    /* update the frame-rate number a couple of times a second */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* hold a steady 60 fps; sleep before drawing so write time doesn't drift it */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    /* draw the frame and send it to the terminal */
    screen_draw(&app->screen, &app->scene, fps_display, alpha);
    screen_present();

    /* check for a keypress */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
