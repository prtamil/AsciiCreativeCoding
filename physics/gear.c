/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * gear.c — a spinning wireframe gear that throws sparks off its teeth.
 *
 * The gear isn't a stored shape: every frame, each screen cell asks
 * "am I on the hub, a spoke, a tooth, or empty?" and draws itself. The
 * sparks are the opposite — real particles flung off the tooth tips,
 * carrying the rim's speed, then falling and fading. Spin faster and the
 * sparks fly faster and there are more of them.
 *
 * References (the code can't tell you these):
 *   Particle systems — Reeves 1983, ACM ToG 2(2):91–108. The emit/age/
 *     move/kill loop that gear_tick and draw_sparks follow.
 *   Gear tooth shape — Buckingham, Analytical Mechanics of Gears (1949).
 *   Drawing a shape by testing each cell against a curve — Bloomenthal,
 *     Introduction to Implicit Surfaces (1997).
 *
 * §1 config  §2 clock  §3 theme  §4 color  §5 coords
 * §6 entity  §7 draw   §8 screen §9 app
 */

#define _POSIX_C_SOURCE 200809L
#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1 config ── */

#define CELL_W 8
#define CELL_H 16

/* Gear geometry (pixels) */
#define GEAR_R_OUTER 88.0f
#define GEAR_R_INNER 64.0f
#define GEAR_R_HUB 22.0f
#define N_TEETH 10
#define TOOTH_DUTY 0.42f

/* Wireframe thresholds (pixels) */
#define THRESH_CIRC 7.5f
#define THRESH_SIDE 4.0f
#define THRESH_SPOKE 3.8f

/* Gear rotation */
#define GEAR_ROT_BASE 1.3f
#define GEAR_ROT_STEP 0.6f
#define GEAR_ROT_MAX 20.0f

/* Spark physics */
#define MAX_SPARKS 1500
#define SPARK_BASE_RATE 80.0f
#define TANG_SCALE 0.5f
#define SPARK_KICK_MIN 35.0f
#define SPARK_KICK_MAX 120.0f
#define SPARK_SCATTER 25.0f /* keep low so streaks stay directional */
#define SPARK_TURB 15.0f    /* keep low so trails stay clean */
#define SPARK_GRAVITY 28.0f
#define SPARK_DRAG 0.4f
#define SPARK_LIFE 1.9f

#define DENSITY_DEFAULT 1.0f
#define DENSITY_STEP 0.3f
#define DENSITY_MAX 6.0f

#define TAU 6.28318530f
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

enum { TARGET_FPS = 60, FPS_UPDATE_MS = 500 };

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

/* ── §3 themes ── */

/*
 * Theme — one complete colour-and-glyph look for the gear and its sparks.
 * Switching themes (t/T) only swaps the look; the motion is identical.
 *
 * As a spark cools from fresh to dead it walks through 7 stages, and the
 * three [7] arrays give its colour, character, and brightness at each one.
 *
 *   name         label shown in the HUD
 *   gear_fg      256-colour index for the bright gear lines
 *   gear_dim_fg  256-colour index for the faint gear lines
 *   spark_fg[7]  colour at each cooling stage (freshest first, dead last)
 *   spark_ch[7]  character at each stage
 *   spark_at[7]  brightness at each stage, encoded 0=normal 1=bold 2=dim
 */
typedef struct {
  const char *name;
  int gear_fg;
  int gear_dim_fg;
  int spark_fg[7];
  char spark_ch[7];
  int spark_at[7];
} Theme;

#define N_THEMES 10

static const Theme THEMES[N_THEMES] = {

    /* 0 FIRE — white-hot → yellow → amber → orange → red → ember */
    {"FIRE",
     153,
     67,
     {231, 226, 220, 214, 202, 196, 160},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 1 MATRIX — white flash → lime → green → dark; digital-rain look */
    {"MATRIX",
     34,
     22,
     {231, 118, 82, 46, 40, 34, 22},
     {'@', '#', '*', '+', ';', ':', '.'},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 2 PLASMA — white → pink → magenta → purple → violet; electric arc */
    {"PLASMA",
     99,
     57,
     {231, 207, 201, 165, 129, 93, 57},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 3 NOVA — white → cyan → sky-blue → blue → deep-blue; stellar */
    {"NOVA",
     69,
     27,
     {231, 159, 123, 87, 51, 39, 27},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 4 POISON — white → yellow → yellow-green → lime → green; toxic */
    {"POISON",
     64,
     22,
     {231, 226, 190, 154, 118, 82, 64},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 5 OCEAN — white → ice → cyan → ocean → deep-blue; bioluminescent */
    {"OCEAN",
     31,
     23,
     {231, 159, 123, 81, 45, 33, 24},
     {'~', 'o', '~', '+', '.', ',', '.'},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 6 GOLD — white → pale-gold → gold → ochre → bronze → copper */
    {"GOLD",
     136,
     94,
     {231, 228, 220, 178, 136, 130, 94},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 7 NEON — white → light-pink → hot-pink → magenta → deep-pink */
    {"NEON",
     201,
     164,
     {231, 219, 213, 207, 201, 200, 164},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},

    /* 8 ARCTIC — white → pale-blue → ice → steel → cornflower → grey */
    {"ARCTIC",
     153,
     67,
     {231, 195, 159, 153, 117, 75, 67},
     {'*', '*', '+', '.', '.', ',', '`'},
     {1, 1, 1, 0, 2, 2, 2}},

    /* 9 LAVA — white → amber → orange → red → dark-red → crimson */
    {"LAVA",
     88,
     52,
     {231, 220, 208, 202, 196, 124, 52},
     {'*', '*', '+', '+', '.', '.', ','},
     {1, 1, 1, 0, 0, 2, 2}},
};

/* A spark cools through 7 stages as its life ticks down; these are the
 * life cut-offs for each stage, brightest first. */
static const float STAGE_THRESH[7] = {0.85f, 0.70f, 0.55f, 0.38f,
                                      0.22f, 0.10f, 0.00f};

static inline int spark_stage(float life) {
  for (int i = 0; i < 6; i++)
    if (life > STAGE_THRESH[i])
      return i;
  return 6;
}

/* ── §4 color ── */

enum {
  CP_GEAR = 1,     /* bright gear lines */
  CP_GEAR_DIM = 2, /* faint gear lines */
  CP_S0 = 3,       /* freshest spark; stages 1..6 follow at 4..9 */
  CP_HUD = 10,
};

/* Turn a theme's brightness code (0/1/2) into the matching ncurses flag. */
static const attr_t ATTR_DEC[3] = {A_NORMAL, A_BOLD, A_DIM};

static void color_apply_theme(int idx) {
  const Theme *th = &THEMES[idx];
  if (COLORS >= 256) {
    init_pair(CP_GEAR, th->gear_fg, -1);
    init_pair(CP_GEAR_DIM, th->gear_dim_fg, -1);
    for (int i = 0; i < 7; i++)
      init_pair(CP_S0 + i, th->spark_fg[i], -1);
    init_pair(CP_HUD, 226, -1);
  } else {
    /* Plain 8-colour terminal: white gear, yellow-to-red sparks. */
    init_pair(CP_GEAR, COLOR_WHITE, -1);
    init_pair(CP_GEAR_DIM, COLOR_WHITE, -1);
    int fb[7] = {COLOR_WHITE, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
                 COLOR_RED,   COLOR_RED,    COLOR_RED};
    for (int i = 0; i < 7; i++)
      init_pair(CP_S0 + i, fb[i], -1);
    init_pair(CP_HUD, COLOR_WHITE, -1);
  }
}

static void color_init(int theme) {
  start_color();
  use_default_colors();
  color_apply_theme(theme);
}

/* ── §5 coords ── */

/* Physics runs in fine pixels; these turn a pixel into its terminal cell. */
static inline int px_col(float px) { return (int)floorf(px / CELL_W); }
static inline int px_row(float py) { return (int)floorf(py / CELL_H); }

/* ── §6 entity ── */

/*
 * Spark — one glowing chip thrown off the gear's rim.
 *
 * Each spark just remembers where it is, how fast it's going, and how
 * much life it has left. Sparks live in one big fixed array (the "pool")
 * that we sweep through every frame.
 *
 * The life field doubles as a clock: instead of remembering when the
 * spark was born and doing subtraction later, we count it down from ~1
 * toward 0. When it reaches 0 the spark is dead and its slot is free for
 * the next one. A fresh spark starts near 1.0 and lives about SPARK_LIFE
 * (1.9 s); along the way it passes through 7 colour stages.
 *
 *   px, py  position, in fine pixels (not whole cells, so slow sparks
 *           don't freeze on a grid); we only round to a cell when drawing
 *   vx, vy  velocity in pixels per second; pushed around by gravity,
 *           drag, and turbulence
 *   life    1.0 = just born, 0.0 = dead; also picks the colour stage
 */
typedef struct {
  float px, py;
  float vx, vy;
  float life;
} Spark;

/*
 * Gear — the spinning ring plus the whole pool of sparks it throws.
 *
 * Gear and sparks live together in one struct on purpose: sparks are born
 * from the teeth and inherit the rim's speed, so they belong to the gear.
 * Reset just zeroes the whole thing in one go.
 *
 * rot_speed is the one knob the user turns (+/- and 1..5), and it drives
 * three things at once: how fast the gear visibly spins, how many sparks
 * fly per second, and how fast each spark is thrown. That's deliberate —
 * a faster gear that also sprays more, faster sparks reads as a machine
 * working harder, not just a slider moving.
 *
 * The sparks live in a plain fixed array, never grown or freed (the
 * project forbids allocating mid-run). To find a free slot we just scan
 * for a dead one; dead sparks cluster near the front so it's usually quick.
 *
 *   cx, cy         gear centre in pixels (middle of the screen)
 *   angle          current rotation in radians; wrapped back under TAU
 *                  each tick so it never grows large enough to lose
 *                  precision
 *   rot_speed      spin speed in rad/s, range [0.2, 20.0]
 *   spark_density  extra spray multiplier (] / [ keys), range [0.2, 6.0];
 *                  separate from rot_speed so you can add sparks without
 *                  changing the visible spin
 *   sparks         the pool; a spark is alive while life > 0, free at <= 0
 *   emit_acc       runs the spark "tap": we add the per-second rate times
 *                  the timestep, and every time it crosses 1.0 we release
 *                  one spark, carrying the leftover fraction to next frame
 */
typedef struct {
  float cx, cy;
  float angle;
  float rot_speed;
  float spark_density;
  Spark sparks[MAX_SPARKS];
  float emit_acc;
} Gear;

static float randf(void) { return (float)rand() / (float)RAND_MAX; }

static void gear_init(Gear *g, float max_px, float max_py) {
  g->cx = max_px * 0.5f;
  g->cy = max_py * 0.5f;
  g->angle = 0.0f;
  g->rot_speed = GEAR_ROT_BASE;
  g->spark_density = DENSITY_DEFAULT;
  g->emit_acc = 0.0f;
  for (int i = 0; i < MAX_SPARKS; i++)
    g->sparks[i].life = 0.0f;
}

/* ── §6 spark pool ── */

/* Find a free spot in the spark pool by scanning for a dead one. Returns
 * nothing if the pool is full — the caller just skips that spark, and the
 * spawn rate quietly settles down on its own. */
static Spark *acquire_free_spark_slot(Spark *pool, int n) {
  for (int i = 0; i < n; i++)
    if (pool[i].life <= 0.0f)
      return &pool[i];
  return NULL;
}

/* ── §6 building a new spark's velocity ── *
 *
 * A new spark's speed and direction are built up one push at a time.
 * First we place it and set its speed to zero, then each helper below adds
 * one more shove. Read spark_emit from top to bottom and you're reading
 * the recipe in order. (Particle systems, Reeves 1983 [1].) */

/* Put the spark at the tip of the tooth and start it at a standstill,
 * ready for the helpers below to push it around. */
static void spawn_spark_at_tooth_tip(Spark *s, float cx, float cy,
                                     float tip_ang) {
  s->px = cx + GEAR_R_OUTER * cosf(tip_ang);
  s->py = cy + GEAR_R_OUTER * sinf(tip_ang);
  s->vx = 0.0f;
  s->vy = 0.0f;
}

/* Give the spark the speed of the spinning rim it's leaving, pointed the
 * way the rim is moving (sideways, along the spin). This is what makes a
 * spark shoot off following the gear's turn rather than straight out. */
static void apply_rim_tangential_velocity(Spark *s, float rot_speed,
                                          float tip_ang) {
  float v = rot_speed * GEAR_R_OUTER * TANG_SCALE;
  s->vx += -sinf(tip_ang) * v;
  s->vy += cosf(tip_ang) * v;
}

/* Give the spark a shove straight outward from the centre, with a random
 * strength so they don't all leave at the same speed. This is the "flung
 * off the edge" kick. */
static void apply_radial_outward_kick(Spark *s, float tip_ang) {
  float kick = SPARK_KICK_MIN + randf() * (SPARK_KICK_MAX - SPARK_KICK_MIN);
  s->vx += cosf(tip_ang) * kick;
  s->vy += sinf(tip_ang) * kick;
}

/* Nudge the spark a little in a random direction. Without this, every
 * spark from one tooth would fly along the exact same line; with it, they
 * fan out into a spray. */
static void apply_uniform_jitter(Spark *s, float spread) {
  s->vx += (randf() - 0.5f) * spread;
  s->vy += (randf() - 0.5f) * spread;
}

/* Start the spark's life clock just under full, so every newborn begins
 * in the brightest stage and lives for about SPARK_LIFE seconds. */
static void spark_seed_lifetime(Spark *s) { s->life = 0.85f + randf() * 0.15f; }

/*
 * Make one new spark: grab a free slot, place it at a tooth tip, build up
 * its velocity push by push, and start its life clock. If the pool is
 * full we just don't make one.
 */
static void spark_emit(Gear *g, float tip_ang) {
  Spark *s = acquire_free_spark_slot(g->sparks, MAX_SPARKS);
  if (!s)
    return;

  spawn_spark_at_tooth_tip(s, g->cx, g->cy, tip_ang);
  apply_rim_tangential_velocity(s, g->rot_speed, tip_ang);
  apply_radial_outward_kick(s, tip_ang);
  apply_uniform_jitter(s, SPARK_SCATTER);
  spark_seed_lifetime(s);
}

/* ── §6 one simulation step ── */

/* Turn the gear a little for this frame, and wrap the angle back around
 * once it passes a full turn so it never grows large enough to lose
 * accuracy over a long run. */
static void gear_advance_rotation(Gear *g, float dt) {
  g->angle += g->rot_speed * dt;
  if (g->angle > TAU)
    g->angle -= TAU;
}

/* How many sparks to throw off per second right now. A faster spin and a
 * higher density both make more. Capped at 1200/s so cranking both to the
 * max doesn't flood the frame and slow everything down. */
static float emission_rate_hz(const Gear *g) {
  float speed_norm = g->rot_speed / GEAR_ROT_BASE;
  float rate = SPARK_BASE_RATE * speed_norm * g->spark_density;
  return rate > 1200.0f ? 1200.0f : rate;
}

/* Which direction tooth number t is pointing right now. The half-tooth
 * offset aims at the middle of the tooth, so sparks come off the centre of
 * the tip instead of its leading corner. */
static float tooth_tip_angle(const Gear *g, int t) {
  return g->angle + (t + TOOTH_DUTY * 0.5f) * TAU / N_TEETH;
}

/* Release the sparks owed this frame. The rate might say "3.4 sparks this
 * frame," so we keep a running tally: add what's owed, hand out whole
 * sparks, and carry the leftover fraction to next frame. Over time the
 * average comes out exactly right. */
static void emit_via_dda_accumulator(Gear *g, float dt) {
  g->emit_acc += emission_rate_hz(g) * dt;
  while (g->emit_acc >= 1.0f) {
    g->emit_acc -= 1.0f;
    int t = rand() % N_TEETH;
    spark_emit(g, tooth_tip_angle(g, t));
  }
}

/*
 * Move one spark forward by a single frame. We age it first and bail if
 * it just died, then let gravity pull it down, add a little random wobble,
 * slow it with drag, and finally slide it to its new spot. If it ends up
 * off-screen we kill it so its slot frees up. The order matters — aging
 * first avoids doing work on a spark that's already gone.
 */
static void integrate_spark(Spark *s, float dt, float max_px, float max_py) {
  s->life -= dt / SPARK_LIFE;
  if (s->life <= 0.0f) {
    s->life = 0.0f;
    return;
  }

  /* gravity pulls the spark down */
  s->vy += SPARK_GRAVITY * dt;

  /* a little random wobble — weaker up-and-down than side-to-side, so the
   * trails stay readable as streaks instead of dissolving into noise */
  s->vx += (randf() - 0.5f) * SPARK_TURB * dt;
  s->vy += (randf() - 0.5f) * SPARK_TURB * 0.4f * dt;

  /* drag — shave a fraction off the speed each frame so sparks ease to a
   * stop; this form stays well-behaved no matter how long the frame was */
  float damp = expf(-SPARK_DRAG * dt);
  s->vx *= damp;
  s->vy *= damp;

  /* move to the new position */
  s->px += s->vx * dt;
  s->py += s->vy * dt;

  /* a spark that left the screen dies, freeing its slot */
  if (s->px < 0 || s->px > max_px || s->py < 0 || s->py > max_py)
    s->life = 0.0f;
}

/*
 * One step of the whole simulation: turn the gear, throw off any new
 * sparks it owes, then move every live spark forward.
 */
static void gear_tick(Gear *g, float dt, float max_px, float max_py) {
  gear_advance_rotation(g, dt);
  emit_via_dda_accumulator(g, dt);
  for (int i = 0; i < MAX_SPARKS; i++) {
    if (g->sparks[i].life <= 0.0f)
      continue;
    integrate_spark(&g->sparks[i], dt, max_px, max_py);
  }
}

/* ── §7 draw ── */

/* Put one character on the screen, but only if it's actually on-screen.
 * Every draw helper goes through here, so the bounds check lives in one
 * place. Defined first so the gear and spark drawers below can call it. */
static void paint_cell(WINDOW *win, int r, int c, chtype ch, int cp, attr_t at,
                       int cols, int rows) {
  if (c < 0 || c >= cols || r < 0 || r >= rows)
    return;
  wattron(win, COLOR_PAIR(cp) | at);
  mvwaddch(win, r, c, ch);
  wattroff(win, COLOR_PAIR(cp) | at);
}

/*
 * Pick the character that best looks like a line pointing in direction
 * `ang`. We only have four to work with: '-' '\' '|' '/'. The circle is
 * split into eight slices, and opposite slices reuse the same glyph since
 * a line looks the same drawn either way along it.
 */
static chtype line_char(float ang) {
  float a = fmodf(ang + TAU, TAU);
  if (a < TAU / 16 || a >= TAU * 15 / 16)
    return '-';
  if (a < TAU * 3 / 16)
    return '\\';
  if (a < TAU * 5 / 16)
    return '|';
  if (a < TAU * 7 / 16)
    return '/';
  if (a < TAU * 9 / 16)
    return '-';
  if (a < TAU * 11 / 16)
    return '\\';
  if (a < TAU * 13 / 16)
    return '|';
  return '/';
}

/* ── §7 gear cell tests ── */

/*
 * Work out the box of cells the gear could possibly touch, trimmed to
 * what's on screen. draw_gear only loops over this box instead of the
 * whole window, so we skip the expensive distance math for the big empty
 * space around the gear.
 */
static void clipped_gear_bbox(const Gear *g, int cols, int rows, int *r_lo,
                              int *r_hi, int *c_lo, int *c_hi) {
  *r_lo = px_row(g->cy - GEAR_R_OUTER) - 1;
  if (*r_lo < 0)
    *r_lo = 0;
  *r_hi = px_row(g->cy + GEAR_R_OUTER) + 1;
  if (*r_hi >= rows)
    *r_hi = rows - 1;
  *c_lo = px_col(g->cx - GEAR_R_OUTER) - 1;
  if (*c_lo < 0)
    *c_lo = 0;
  *c_hi = px_col(g->cx + GEAR_R_OUTER) + 1;
  if (*c_hi >= cols)
    *c_hi = cols - 1;
}

/*
 * For one cell, find how far it is from the gear's centre and in which
 * direction. We measure from the cell's centre point so the gear stays
 * crisp at sub-cell detail. Two angles come out:
 *   ang_g  the direction as the viewer sees it — used to pick the line
 *          character, which doesn't care how the gear is turned.
 *   ang_l  the same direction but as the gear sees it (with its spin
 *          subtracted out) — used to test for teeth and spokes, which
 *          are fixed to the gear and turn with it.
 */
static void cell_to_gear_polar(int r, int c, const Gear *g, float *rad,
                               float *ang_g, float *ang_l) {
  float dx = c * CELL_W + CELL_W * 0.5f - g->cx;
  float dy = r * CELL_H + CELL_H * 0.5f - g->cy;
  *rad = sqrtf(dx * dx + dy * dy);
  *ang_g = atan2f(dy, dx);
  *ang_l = *ang_g - g->angle;
}

/*
 * Where we are inside one tooth-and-gap slice, as a fraction from 0 to 1.
 * The first chunk (up to TOOTH_DUTY) is the tooth; the rest is the gap
 * between teeth.
 */
static float tooth_phase(float ang_l) {
  float phase = fmodf(ang_l * N_TEETH / TAU, 1.0f);
  if (phase < 0.0f)
    phase += 1.0f;
  return phase;
}

/*
 * How far (in pixels) the sample point is from the nearest edge of a
 * tooth, measured along the circle. A tooth has two side edges; we take
 * the closer one. Used to know when a cell sits right on a tooth's side
 * so we can draw that edge.
 */
static float arclen_to_tooth_side(float phase, float rad) {
  float d0 = phase;
  float dT = fabsf(phase - TOOTH_DUTY);
  if (d0 > 0.5f)
    d0 = 1.0f - d0; /* the gap wraps around, so measure the short way */
  float nearest = (d0 < dT) ? d0 : dT;
  return nearest * (TAU / N_TEETH) * rad;
}

/*
 * How far (in pixels) the sample point is from the nearest spoke, along
 * the circle. There's one spoke per tooth, lined up with the middle of
 * each slice. Used to know when a cell sits on a spoke.
 */
static float arclen_to_spoke(float ang_l, float rad) {
  float period = TAU / N_TEETH;
  float m = fmodf(ang_l, period);
  if (m < 0.0f)
    m += period;
  float ang_d = (m < period * 0.5f) ? m : period - m;
  return ang_d * rad;
}

/*
 * CellPaint — the answer to "what should I draw at this one cell?"
 *
 * For every cell near the gear we ask which part of the gear it lands on,
 * and the answer comes back as one of these little bundles: a character,
 * how bright to draw it, and which colour to use. Bundling the three
 * together keeps the deciding ("what's here?") cleanly separate from the
 * drawing ("put it on screen") — one function decides and hands back a
 * CellPaint, the other just paints it.
 *
 * Brighter parts of the gear sit on top of dimmer ones. The bright
 * outline (hub, spokes, tooth edges) is checked first and wins; the dim
 * fills (tooth interior, body disc) only show through where there's no
 * outline. So brightness here also doubles as draw order.
 *
 *   ch  the character to draw. One of the line shapes ('-' '\' '|' '/')
 *       for edges, or a fill ('.' for the body, ':' inside teeth).
 *       A value of 0 is the special "draw nothing here" marker — the
 *       cell is empty space. 0 is safe to mean this because no real
 *       printable character has that value.
 *   at  how bright: A_BOLD (bright outline), A_NORMAL (medium), or
 *       A_DIM (the faint interior fills).
 *   cp  which colour pair to use: CP_GEAR (the theme's bright gear
 *       colour) or CP_GEAR_DIM (its muted one). Switching theme changes
 *       what these colours actually look like, but this choice between
 *       bright and muted stays the same.
 */
typedef struct {
  chtype ch;
  attr_t at;
  int cp;
} CellPaint;

/*
 * Given where a cell sits relative to the gear (how far out, and which
 * way), figure out which part of the gear it's on and what to draw there.
 *
 * The checks run in order and the first match wins. Order matters: the
 * bright outline parts are checked before the dim fills, so the outline
 * always paints on top instead of being buried under a fill. The order is:
 *   1. hub ring         the small circle in the middle
 *   2. spokes           the lines from hub out to the inner ring
 *   3. tooth sides      the straight edges of each tooth
 *   4. inner ring       only across the gaps between teeth
 *   5. outer ring       only across the tips of the teeth
 *   6. tooth interior   ':' filling the body of each tooth
 *   7. body disc        '.' filling the flat plate inside the teeth
 */
static CellPaint classify_gear_cell(float rad, float ang_g, float ang_l) {
  float phase = tooth_phase(ang_l);
  bool in_tooth = (phase < TOOTH_DUTY);
  float arc_side = arclen_to_tooth_side(phase, rad);
  float arc_spoke = arclen_to_spoke(ang_l, rad);

  /* ── bright outline (checked first, drawn on top) ── */
  if (fabsf(rad - GEAR_R_HUB) < THRESH_CIRC) /* hub ring */
    return (CellPaint){line_char(ang_g + TAU * 0.25f), A_BOLD, CP_GEAR};

  if (rad > GEAR_R_HUB && rad < GEAR_R_INNER /* spoke */
      && arc_spoke < THRESH_SPOKE)
    return (CellPaint){line_char(ang_g), A_BOLD, CP_GEAR};

  if (rad > GEAR_R_INNER - THRESH_SIDE * 0.5f /* tooth side */
      && rad < GEAR_R_OUTER && arc_side < THRESH_SIDE)
    return (CellPaint){line_char(ang_g), A_BOLD, CP_GEAR};

  if (!in_tooth && fabsf(rad - GEAR_R_INNER) < THRESH_CIRC) /* inner arc */
    return (CellPaint){line_char(ang_g + TAU * 0.25f), A_NORMAL, CP_GEAR_DIM};

  if (in_tooth && fabsf(rad - GEAR_R_OUTER) < THRESH_CIRC) /* outer arc */
    return (CellPaint){line_char(ang_g + TAU * 0.25f), A_BOLD, CP_GEAR};

  /* ── dim fills (only where no outline matched) ── */
  if (in_tooth && rad > GEAR_R_INNER && rad < GEAR_R_OUTER) /* tooth fill */
    return (CellPaint){':', A_DIM, CP_GEAR};

  if (rad > GEAR_R_HUB + THRESH_CIRC * 0.5f /* body disc */
      && rad < GEAR_R_INNER - THRESH_CIRC * 0.5f)
    return (CellPaint){'.', A_DIM, CP_GEAR_DIM};

  return (CellPaint){0, A_NORMAL, CP_GEAR}; /* transparent */
}

/*
 * Draw the whole gear by walking the cells around it and asking each one
 * which part of the gear it belongs to, then painting the answer. Cells
 * too far out to be any part of the gear are skipped.
 */
static void draw_gear(WINDOW *win, const Gear *g, int cols, int rows) {
  int r_lo, r_hi, c_lo, c_hi;
  clipped_gear_bbox(g, cols, rows, &r_lo, &r_hi, &c_lo, &c_hi);

  for (int r = r_lo; r <= r_hi; r++) {
    for (int c = c_lo; c <= c_hi; c++) {
      float rad, ang_g, ang_l;
      cell_to_gear_polar(r, c, g, &rad, &ang_g, &ang_l);
      if (rad > GEAR_R_OUTER + THRESH_CIRC)
        continue;

      CellPaint p = classify_gear_cell(rad, ang_g, ang_l);
      if (!p.ch)
        continue;
      paint_cell(win, r, c, p.ch, p.cp, p.at, cols, rows);
    }
  }
}

/*
 * Pick the line character that points the way a spark is moving, so a
 * spark heading sideways looks like '-', one falling looks like '|', and
 * so on. Same idea as line_char, just driven by velocity instead of an
 * angle.
 */
static chtype direction_glyph(float vx, float vy) {
  float a = atan2f(vy, vx);
  if (a < 0.0f)
    a += TAU;
  if (a < TAU / 16.0f || a >= TAU * 15.0f / 16.0f)
    return '-';
  if (a < TAU * 3.0f / 16.0f)
    return '\\';
  if (a < TAU * 5.0f / 16.0f)
    return '|';
  if (a < TAU * 7.0f / 16.0f)
    return '/';
  if (a < TAU * 9.0f / 16.0f)
    return '-';
  if (a < TAU * 11.0f / 16.0f)
    return '\\';
  if (a < TAU * 13.0f / 16.0f)
    return '|';
  return '/';
}

/*
 * Draw a fresh spark as a little streak, like a comet: a bright head where
 * the spark is, plus a few dim cells trailing behind it along the way it
 * came from. The fading tail reads as motion, so the screen looks like a
 * spray of streaks rather than a scatter of dots.
 */
static void draw_spark_streak(WINDOW *win, const Spark *s, int st, int cp,
                              attr_t head_attr, int n_tail, int cols,
                              int rows) {
  chtype glyph = direction_glyph(s->vx, s->vy);
  paint_cell(win, px_row(s->py), px_col(s->px), glyph, cp, head_attr, cols,
             rows);

  /* A streak only makes sense if the spark is moving — a still spark has
   * no direction to trail behind it. */
  float speed = sqrtf(s->vx * s->vx + s->vy * s->vy);
  if (speed < 1.0f)
    return;
  float ux = s->vx / speed;
  float uy = s->vy / speed;
  for (int k = 1; k <= n_tail; k++) {
    float tx = s->px - ux * CELL_W * (float)k;
    float ty = s->py - uy * CELL_H * (float)k;
    paint_cell(win, px_row(ty), px_col(tx), glyph, cp, A_DIM, cols, rows);
  }
  (void)st; /* kept for future per-stage tail tuning */
}

/* Draw a cooled-down spark as a single dot — the theme says which
 * character and colour to use for how faded it is. */
static void draw_spark_dot(WINDOW *win, const Spark *s, const Theme *th, int st,
                           int cols, int rows) {
  chtype ch = (chtype)(unsigned char)th->spark_ch[st];
  attr_t at = ATTR_DEC[th->spark_at[st]];
  int cp = CP_S0 + st;
  paint_cell(win, px_row(s->py), px_col(s->px), ch, cp, at, cols, rows);
}

/*
 * Draw all the live sparks. The freshest ones get drawn as streaks with a
 * trailing tail; as a spark cools it loses its tail and finally becomes a
 * plain fading dot, so it quietly settles into the background.
 */
static void draw_sparks(WINDOW *win, const Gear *g, int cols, int rows,
                        int theme_idx) {
  const Theme *th = &THEMES[theme_idx];
  static const int TAIL_PER_STAGE[3] = {2, 1, 0};

  for (int i = 0; i < MAX_SPARKS; i++) {
    const Spark *s = &g->sparks[i];
    if (s->life <= 0.0f)
      continue;

    int st = spark_stage(s->life);
    if (st <= 2) {
      int cp = CP_S0 + st;
      attr_t head_attr = ATTR_DEC[th->spark_at[st]];
      draw_spark_streak(win, s, st, cp, head_attr, TAIL_PER_STAGE[st], cols,
                        rows);
    } else {
      draw_spark_dot(win, s, th, st, cols, rows);
    }
  }
}

/* ── §8 screen / scene ── */

/*
 * Scene — everything the simulation and the drawing need, in one place.
 *
 * The fields fall into two groups, and the split is there to keep the
 * reader honest about which knobs change the picture and which change the
 * physics:
 *
 *   Simulation — anything that affects how the gear and sparks behave.
 *   Changed by the keys that touch the motion: reset, pause, speed,
 *   density, and resize.
 *
 *   Rendering — purely how it looks. Changing these while paused leaves
 *   every spark exactly where it was; only the colours and characters
 *   change. Changed by the theme keys.
 *
 * Watch out: max_px / max_py LOOK like screen size, but they belong with
 * the simulation, because gear_tick kills any spark that drifts past them.
 * Change them and you silently change which sparks survive. So when you
 * add a field, ask: does it change how the gear behaves? If yes it's a
 * simulation field; if it's only about looks it's a rendering field.
 *
 *   gear            the gear and its whole pool of sparks (see §6)
 *   paused          when true, scene_tick does nothing and everything
 *                   freezes in place
 *   max_px, max_py  the size of the play area in pixels; sparks that
 *                   leave it die, so this is simulation state, not just
 *                   screen geometry
 *   theme           which look from THEMES[] (§3) is active; the theme
 *                   keys cycle it, and it never changes the physics
 *
 * The quit/resize flags and the terminal size live on App (§9) instead of
 * here: the flags are written from signal handlers and must stay there to
 * be safe, and keeping the window size out of Scene lets the simulation
 * stay unaware of the terminal. There's only ever one Scene, tucked inside
 * the single App, and it's big (the spark pool alone is tens of KB) so
 * it's never copied around by value.
 */
typedef struct {
  /* ── simulation (read & written by gear_tick) ── */
  Gear gear;
  bool paused;
  float max_px, max_py;

  /* ── rendering (no effect on the physics) ── */
  int theme;
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  s->max_px = (float)(cols * CELL_W);
  s->max_py = (float)(rows * CELL_H);
  s->paused = false;
  /* keep current theme on reset */
  gear_init(&s->gear, s->max_px, s->max_py);
}

static void scene_tick(Scene *s, float dt) {
  if (s->paused)
    return;
  gear_tick(&s->gear, dt, s->max_px, s->max_py);
}

/* Count how many sparks are currently alive, for the HUD readout. */
static int count_live_sparks(const Gear *g) {
  int n = 0;
  for (int i = 0; i < MAX_SPARKS; i++)
    if (g->sparks[i].life > 0.0f)
      n++;
  return n;
}

/* Draw the sparks first, then the gear on top, so the bright gear is
 * never hidden behind its own spray. */
static void compose_scene_layers(WINDOW *win, const Scene *s, int cols,
                                 int rows) {
  draw_sparks(win, &s->gear, cols, rows, s->theme);
  draw_gear(win, &s->gear, cols, rows);
}

/* Top row of the readout: the live numbers — fps, spin speed, density,
 * how many sparks are alive, and the theme. */
static void hud_draw_top(WINDOW *win, const Scene *s, double fps, int live) {
  wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
  mvwprintw(win, 0, 0,
            " %.0f fps  speed:%.1f rad/s  density:%.1fx  sparks:%d"
            "  theme:[%d] %s ",
            fps, s->gear.rot_speed, s->gear.spark_density, live, s->theme,
            THEMES[s->theme].name);
  wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Bottom row of the readout: the list of keys you can press. */
static void hud_draw_bottom(WINDOW *win, int rows) {
  wattron(win, COLOR_PAIR(CP_HUD) | A_BOLD);
  mvwprintw(win, rows - 1, 0,
            " q:quit  spc:pause  r:reset  +/-:speed  ]/[:density"
            "  t/T:theme  1-5:speed presets ");
  wattroff(win, COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Draw one whole frame: the scene, then the two readout rows. */
static void scene_draw(const Scene *s, WINDOW *win, int cols, int rows,
                       double fps) {
  compose_scene_layers(win, s, cols, rows);
  hud_draw_top(win, s, fps, count_live_sparks(&s->gear));
  hud_draw_bottom(win, rows);
}

typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *sc, int theme) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init(theme);
  getmaxyx(stdscr, sc->rows, sc->cols);
}
static void screen_free(Screen *sc) {
  (void)sc;
  endwin();
}

static void screen_resize(Screen *sc) {
  endwin();
  refresh();
  getmaxyx(stdscr, sc->rows, sc->cols);
}

static void screen_render(Screen *sc, const Scene *s, double fps) {
  erase();
  scene_draw(s, stdscr, sc->cols, sc->rows, fps);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §9 app ── */

/*
 * App — the one top-level bundle the whole program runs on.
 *
 *   scene        the gear, sparks, and view settings (§8)
 *   screen       the current terminal size (§8)
 *   running      set to 0 to ask the main loop to stop; written from the
 *                quit/interrupt signal handler, so it's marked volatile
 *                sig_atomic_t to be safe to touch from a signal
 *   need_resize  set to 1 by the window-resize signal so the loop knows
 *                to re-measure the terminal; volatile for the same reason
 */
typedef struct {
  Scene scene;
  Screen screen;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_app.need_resize = 1;
  else
    g_app.running = 0;
}
static void cleanup(void) { endwin(); }

static bool app_key(App *app, int ch) {
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
    scene_init(sc, app->screen.cols, app->screen.rows);
    break;
  case '+':
  case '=':
    sc->gear.rot_speed += GEAR_ROT_STEP;
    if (sc->gear.rot_speed > GEAR_ROT_MAX)
      sc->gear.rot_speed = GEAR_ROT_MAX;
    break;
  case '-':
    sc->gear.rot_speed -= GEAR_ROT_STEP;
    if (sc->gear.rot_speed < 0.2f)
      sc->gear.rot_speed = 0.2f;
    break;
  case ']':
    sc->gear.spark_density += DENSITY_STEP;
    if (sc->gear.spark_density > DENSITY_MAX)
      sc->gear.spark_density = DENSITY_MAX;
    break;
  case '[':
    sc->gear.spark_density -= DENSITY_STEP;
    if (sc->gear.spark_density < 0.2f)
      sc->gear.spark_density = 0.2f;
    break;
  case 't':
  case 'T': {
    int dir = (ch == 't') ? 1 : -1;
    sc->theme = (sc->theme + dir + N_THEMES) % N_THEMES;
    color_apply_theme(sc->theme);
    break;
  }
  case '1':
    sc->gear.rot_speed = 0.4f;
    break;
  case '2':
    sc->gear.rot_speed = 2.0f;
    break;
  case '3':
    sc->gear.rot_speed = 6.0f;
    break;
  case '4':
    sc->gear.rot_speed = 12.0f;
    break;
  case '5':
    sc->gear.rot_speed = 20.0f;
    break;
  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
  atexit(cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  App *app = &g_app;
  app->running = 1;
  app->scene.theme = 0; /* start with FIRE */

  screen_init(&app->screen, app->scene.theme);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t prev = clock_ns(), fps_acc = 0;
  int fps_count = 0;
  double fps_disp = 0.0;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      app->scene.max_px = (float)(app->screen.cols * CELL_W);
      app->scene.max_py = (float)(app->screen.rows * CELL_H);
      app->scene.gear.cx = app->scene.max_px * 0.5f;
      app->scene.gear.cy = app->scene.max_py * 0.5f;
      app->need_resize = 0;
      prev = clock_ns();
    }

    int64_t now = clock_ns();
    int64_t dt_ns = now - prev;
    prev = now;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;
    float dt = (float)dt_ns / (float)NS_PER_SEC;

    scene_tick(&app->scene, dt);

    fps_count++;
    fps_acc += dt_ns;
    if (fps_acc >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (double)fps_count / ((double)fps_acc / NS_PER_SEC);
      fps_count = 0;
      fps_acc = 0;
    }

    screen_render(&app->screen, &app->scene, fps_disp);

    int64_t elapsed = clock_ns() - now;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    int key = getch();
    if (key != ERR && !app_key(app, key))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
