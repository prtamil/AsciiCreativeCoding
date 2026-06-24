/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * stroke_engine.c — animated ASCII cutaway of three piston engines.
 *
 * Watch a 2-, 4-, or 6-stroke engine run in cross-section and switch
 * between them live. All three move the piston the same way (a spinning
 * crank pulls it up and down through a rod); what changes is the timing
 * of intake/burn/exhaust and how fresh and spent gas get in and out:
 * the 2-stroke uses holes in the cylinder wall, the 4-stroke uses valves
 * in the head, and the 6-stroke adds a water spray that flashes to steam
 * for one bonus power stroke off the leftover heat.
 *
 * Engine facts come from Heywood, "Internal Combustion Engine
 * Fundamentals" (2nd ed., 2018). The cutaway-drawing conventions trace
 * back to Reuleaux, "The Kinematics of Machinery" (1876).
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
  SIM_FPS_DEFAULT = 120,
  SIM_FPS_MIN = 20,
  SIM_FPS_MAX = 300,

  RPM_DEFAULT = 120,
  RPM_MIN = 30,
  RPM_MAX = 600,
  RPM_STEP = 30,

  FPS_UPDATE_MS = 500,
};

/*
 * Engine size, in character cells (y grows downward). These few numbers
 * fix where every part lives; the rest of the layout is derived from
 * them. The piston's travel (its "stroke") is twice the crank radius,
 * so it slides 8 cells between top and bottom.
 */
#define CYL_IHW 6  /* half the cylinder's inner width            */
#define CYL_WALL 1 /* how thick the cylinder wall is             */
#define HEAD_H 1   /* height of the lid on top (the head)        */
#define PISTON_H 3 /* how tall the piston is                     */
#define CRANK_R 4  /* how far the crank pin swings from center   */
#define CONROD_L 9 /* length of the rod from piston to crank     */
/* Where the crank's center sits below the engine's top, stacked up from
 * all the parts above it so the piston just kisses the head at the top. */
#define CRANK_CENTER_OFF (HEAD_H + (PISTON_H - 1) + CONROD_L + CRANK_R)

/* The two holes in the 2-stroke's wall, as rows below the engine top.
 * A hole counts as open once the piston has slid down past it. */
#define EX_PORT_OFF 6
#define TR_PORT_OFF 7

/* The crankcase — the box at the bottom housing the spinning crank. */
#define CASE_TOP_OFF 12
#define CASE_BOT_OFF 21
#define CASE_HW 9
#define ENGINE_H (CASE_BOT_OFF + 2) /* whole engine is 23 rows tall */

/* How far left/right of center the two valves sit in the 4/6-stroke. */
#define VALVE_OFF 3

/* The spark is instant, so we flash it for a small slice of crank angle
 * (in radians) on each side of the firing point instead of one frame. */
#define IGNITE_WINDOW 0.30f

/* Same idea for the 6-stroke's water spray: brief in reality, but we
 * give it a wider slice so you can actually see it happen. */
#define WATER_WINDOW 0.45f

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

/* ── §3 themes & color ── */

/* The named color slots we draw with. Each engine part and each gas
 * effect picks one of these; a theme then decides the actual color. */
typedef enum {
  CP_WALL = 1,    /* walls, head, crankcase            */
  CP_PISTON = 2,  /* the piston                        */
  CP_CONROD = 3,  /* the connecting rod                */
  CP_CRANK = 4,   /* the crankshaft                    */
  CP_FIRE = 5,    /* burning / hot gas                 */
  CP_EXHAUST = 6, /* spent gas and the exhaust valve   */
  CP_INTAKE = 7,  /* fresh fuel-air, intake valve, water */
  CP_SPARK = 8,   /* the spark flash and steam         */
  CP_HUD = 9,     /* top status bar (bright yellow)    */
  CP_HINT = 10,   /* bottom key hints (bright cyan)    */
  CP_PHASE = 11,  /* the current-phase label           */
} ColorPair;

/*
 * One color scheme. Each theme assigns a color to every engine part and
 * gas effect, so you can re-skin the whole engine with t/T. We keep all
 * colors in the bright half of the palette so nothing vanishes against
 * the black background. The status-bar colors aren't themed — they stay
 * fixed so they're always readable.
 */
typedef struct {
  const char *name;
  short wall, piston, conrod, crank;
  short fire, exhaust, intake, spark;
} Theme;

static const Theme THEMES[] = {
    /* name       wall pist  rod crank  fire exha  int  spark */
    {"MATRIX", 244, 46, 40, 82, 46, 28, 118, 231},
    {"FIRE", 244, 208, 214, 220, 196, 240, 226, 231},
    {"OCEANIC", 251, 68, 110, 87, 45, 244, 159, 231},
    {"NEON", 51, 201, 219, 82, 199, 141, 51, 231},
    {"MONO", 251, 248, 246, 255, 244, 242, 250, 255},
    {"ICE", 255, 153, 159, 51, 159, 251, 87, 231},
    {"NOVA", 255, 177, 213, 231, 201, 141, 219, 231},
    {"FOREST", 144, 70, 108, 178, 130, 240, 148, 231},
    {"DESERT", 180, 215, 222, 178, 166, 180, 228, 231},
    {"ECLIPSE", 250, 240, 244, 208, 160, 240, 214, 231},
};
#define N_THEMES ((int)(sizeof(THEMES) / sizeof(THEMES[0])))

static void color_apply_theme(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &THEMES[idx];
    init_pair(CP_WALL, t->wall, COLOR_BLACK);
    init_pair(CP_PISTON, t->piston, COLOR_BLACK);
    init_pair(CP_CONROD, t->conrod, COLOR_BLACK);
    init_pair(CP_CRANK, t->crank, COLOR_BLACK);
    init_pair(CP_FIRE, t->fire, COLOR_BLACK);
    init_pair(CP_EXHAUST, t->exhaust, COLOR_BLACK);
    init_pair(CP_INTAKE, t->intake, COLOR_BLACK);
    init_pair(CP_SPARK, t->spark, COLOR_BLACK);
  } else {
    init_pair(CP_WALL, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_PISTON, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_CONROD, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_CRANK, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_FIRE, COLOR_RED, COLOR_BLACK);
    init_pair(CP_EXHAUST, COLOR_WHITE, COLOR_BLACK);
    init_pair(CP_INTAKE, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_SPARK, COLOR_WHITE, COLOR_BLACK);
  }
}

static void color_init(int theme_idx) {
  start_color();
  if (COLORS >= 256) {
    init_pair(CP_HUD, 226, COLOR_BLACK);
    init_pair(CP_HINT, 51, COLOR_BLACK);
    init_pair(CP_PHASE, 214, COLOR_BLACK);
  } else {
    init_pair(CP_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_PHASE, COLOR_YELLOW, COLOR_BLACK);
  }
  color_apply_theme(theme_idx);
}

/* ── §4 kinematics — slider-crank ── */

/*
 * Where the moving parts are, this instant, in cell coordinates.
 *
 * A spinning crank pulls the piston up and down through a rigid rod —
 * the same "slider-crank" linkage in every car engine. From one crank
 * angle we work out four points and hand them back together. They must
 * stay in sync: if we computed them separately and they drifted, the
 * rod would stop touching the piston and the engine would look broken.
 *
 * The four points (cells, y increases downward):
 *
 *      crown_row    ┌───── top of the piston (the face the gas pushes)
 *                   │ piston, PISTON_H rows tall
 *      wrist_row    └─── wrist pin: where the rod joins the piston
 *                   │
 *                   │  the rod (length CONROD_L)
 *                   │
 *      crank_pin_r  o── crank pin: the rod's other end, which
 *      crank_pin_c     circles the crank center at radius CRANK_R
 *
 * Two things stay true no matter the angle: the rod never stretches
 * (rod end to wrist pin is always CONROD_L), and the crank pin always
 * circles the center at radius CRANK_R.
 *
 * We solve it directly with one formula instead of looping — see Norton,
 * "Design of Machinery" §4.5. The numbers get rounded to whole cells
 * once, here, so every part that should touch lines up exactly on the
 * grid instead of jittering half a cell.
 */
typedef struct {
  int crown_row;   /* top of the piston                        */
  int wrist_row;   /* wrist pin: where piston meets rod         */
  int crank_pin_r; /* crank pin (rod's far end), row            */
  int crank_pin_c; /* crank pin (rod's far end), column         */
} Kinematics;

/* Same math for every engine; only the timing around it differs. theta
 * is the crank angle, 0 = piston all the way up, growing clockwise. */
static Kinematics solve_slider_crank(float theta, float cc_row, float cc_col) {
  float cr = (float)CRANK_R;
  float rod = (float)CONROD_L;
  float cp_dy = -cr * cosf(theta);
  float cp_dx = cr * sinf(theta);
  float rod_v = sqrtf(rod * rod - cp_dx * cp_dx);
  float wp_row = (cc_row + cp_dy) - rod_v;

  Kinematics k;
  k.crank_pin_r = (int)roundf(cc_row + cp_dy);
  k.crank_pin_c = (int)roundf(cc_col + cp_dx);
  k.wrist_row = (int)roundf(wp_row);
  k.crown_row = (int)roundf(wp_row - (float)(PISTON_H - 1));
  return k;
}

/* ── §5 engine — type, cycle state machine, tick ── */

typedef enum {
  ENG_2STROKE = 0,
  ENG_4STROKE = 1,
  ENG_6STROKE = 2,
} EngineType;
#define N_ENGINES 3

static const char *engine_name(EngineType t) {
  switch (t) {
  case ENG_2STROKE:
    return "2-STROKE";
  case ENG_4STROKE:
    return "4-STROKE";
  case ENG_6STROKE:
    return "6-STROKE";
  default:
    return "?";
  }
}

static int engine_strokes(EngineType t) {
  return (t == ENG_2STROKE) ? 2 : (t == ENG_4STROKE) ? 4 : 6;
}

/* How far the cycle angle travels before everything repeats. The crank
 * turns once every two strokes, so a full cycle is half a turn per
 * stroke (pi radians each). */
static float engine_cycle_total(EngineType t) {
  return (float)engine_strokes(t) * (float)M_PI;
}

/* The friendly name shown in the status bar, so you can tell at a glance
 * which engine is running and how many crank turns one cycle takes. */
static const char *engine_subtitle(EngineType t) {
  switch (t) {
  case ENG_2STROKE:
    return "Schnurle scavenging (1 rev/cycle)";
  case ENG_4STROKE:
    return "Otto cycle (2 rev/cycle)";
  case ENG_6STROKE:
    return "Crower water-injection (3 rev/cycle)";
  default:
    return "";
  }
}

typedef enum {
  PHASE_INTAKE,
  PHASE_COMPRESS,
  PHASE_IGNITE,
  PHASE_POWER,
  PHASE_EXHAUST,
  PHASE_SCAVENGE,
  PHASE_WATER_INJ,
  PHASE_STEAM_POWER,
  PHASE_STEAM_EXHAUST,
} Phase;

static const char *phase_name(Phase p) {
  switch (p) {
  case PHASE_INTAKE:
    return "INTAKE       ";
  case PHASE_COMPRESS:
    return "COMPRESSION  ";
  case PHASE_IGNITE:
    return "IGNITION     ";
  case PHASE_POWER:
    return "POWER        ";
  case PHASE_EXHAUST:
    return "EXHAUST      ";
  case PHASE_SCAVENGE:
    return "SCAVENGING   ";
  case PHASE_WATER_INJ:
    return "WATER INJECT ";
  case PHASE_STEAM_POWER:
    return "STEAM POWER  ";
  case PHASE_STEAM_EXHAUST:
    return "STEAM EXHAUST";
  default:
    return "             ";
  }
}

/*
 * Engine — everything we know about the one cylinder right now.
 *
 * It holds two kinds of fields, and the split matters:
 *
 *   (a) The real state — where the engine actually is. The cycle angle
 *       creeps forward a little each step, and the keys change the type,
 *       speed, and pause flag. This is the only state that survives from
 *       one frame to the next.
 *
 *   (b) Handy answers worked out from (a) — which valves and ports are
 *       open and which phase we're in. We figure these out once at the
 *       top of each frame and stash them here, because several drawing
 *       routines (the gas, the head valves, the wall ports) all need the
 *       same answers. Working them out once keeps them in agreement; if
 *       each routine recomputed them, one could easily drift from another.
 *
 * The phase boundaries (when intake ends, when it fires, and so on) follow
 * the classic four-stroke "Otto" cycle in Heywood §6; the 2-stroke's ports
 * open purely by where the piston sits, also from Heywood §7.
 *
 * Always true: the cycle angle stays inside one full cycle, the speed
 * stays within its allowed range, and the handy answers (b) match the
 * cycle angle as of the current frame.
 */
typedef struct {
  /* ── (a) The real state ── */
  EngineType type;   /* which engine we're running (2-, 4-, 6-stroke)    */
  float cycle_angle; /* how far through one full cycle, in radians;
                      *   runs 0 up to strokes*pi, then wraps to 0       */
  int rpm;           /* target crank speed in revs/min; sets how fast
                      *   cycle_angle advances each tick                 */
  bool paused;       /* when true, the engine stops advancing           */

  /* ── (b) Handy answers, refreshed each frame ──
   * Filled in by engine_derive_state() at the start of every frame.
   * Don't set these anywhere else — the drawing code only reads them,
   * and they're only good for the current frame. */
  bool intake_valve_open;  /* 4/6-stroke: fresh fuel-air coming in       */
  bool exhaust_valve_open; /* 4/6-stroke: spent gas (or steam) going out */
  bool water_inj_open;     /* 6-stroke: water being sprayed in           */
  bool ex_port_open;       /* 2-stroke: piston has uncovered exhaust hole*/
  bool tr_port_open;       /* 2-stroke: piston has uncovered transfer hole*/
  Phase phase;             /* which phase we're in; picks the gas to draw*/
} Engine;

static float wrap_to_2pi(float a) {
  float two_pi = 2.0f * (float)M_PI;
  while (a < 0.0f)
    a += two_pi;
  while (a >= two_pi)
    a -= two_pi;
  return a;
}

static float engine_crank_angle(const Engine *e) {
  return wrap_to_2pi(e->cycle_angle);
}

static void engine_reset(Engine *e, EngineType type) {
  e->type = type;
  e->cycle_angle = (float)M_PI; /* start with the piston near the bottom */
  e->rpm = RPM_DEFAULT;
  e->paused = false;
  e->intake_valve_open = false;
  e->exhaust_valve_open = false;
  e->water_inj_open = false;
  e->ex_port_open = false;
  e->tr_port_open = false;
  e->phase = PHASE_COMPRESS;
}

/* Switch engine type but preserve RPM and pause state. */
static void engine_set_type(Engine *e, EngineType type) {
  e->type = type;
  e->cycle_angle = (float)M_PI;
}

static void engine_tick(Engine *e, float dt) {
  if (e->paused)
    return;
  float omega = 2.0f * (float)M_PI * (float)e->rpm / 60.0f;
  e->cycle_angle += omega * dt;
  float total = engine_cycle_total(e->type);
  if (e->cycle_angle >= total)
    e->cycle_angle -= total;
  if (e->cycle_angle < 0.0f)
    e->cycle_angle += total;
}

/*
 * From how far we are into the cycle, work out the current phase and
 * which valves and holes are open. The 2-stroke also needs the piston's
 * top row, because its wall holes open simply when the piston slides
 * past them, not on a timed schedule.
 */
static void engine_derive_state(Engine *e, int crown_row, int engine_top) {
  const float TWO_PI = 2.0f * (float)M_PI;
  const float THREE_PI = 3.0f * (float)M_PI;
  const float FOUR_PI = 4.0f * (float)M_PI;
  const float FIVE_PI = 5.0f * (float)M_PI;
  float c = e->cycle_angle;

  e->intake_valve_open = false;
  e->exhaust_valve_open = false;
  e->water_inj_open = false;
  e->ex_port_open = false;
  e->tr_port_open = false;

  switch (e->type) {
  case ENG_2STROKE: {
    bool exo = (crown_row > engine_top + EX_PORT_OFF);
    bool tro = (crown_row > engine_top + TR_PORT_OFF);
    e->ex_port_open = exo;
    e->tr_port_open = tro;
    if (tro)
      e->phase = PHASE_SCAVENGE;
    else if (exo)
      e->phase = PHASE_EXHAUST;
    else if (c < IGNITE_WINDOW || c > TWO_PI - IGNITE_WINDOW)
      e->phase = PHASE_IGNITE;
    else if (c < (float)M_PI)
      e->phase = PHASE_POWER;
    else
      e->phase = PHASE_COMPRESS;
    break;
  }
  case ENG_4STROKE: {
    if (c < (float)M_PI) {
      e->intake_valve_open = true;
      e->phase = PHASE_INTAKE;
    } else if (c < TWO_PI) {
      e->phase = (c > TWO_PI - IGNITE_WINDOW) ? PHASE_IGNITE : PHASE_COMPRESS;
    } else if (c < TWO_PI + IGNITE_WINDOW) {
      e->phase = PHASE_IGNITE;
    } else if (c < THREE_PI) {
      e->phase = PHASE_POWER;
    } else {
      e->exhaust_valve_open = true;
      e->phase = PHASE_EXHAUST;
    }
    break;
  }
  case ENG_6STROKE: {
    if (c < (float)M_PI) {
      e->intake_valve_open = true;
      e->phase = PHASE_INTAKE;
    } else if (c < TWO_PI) {
      e->phase = (c > TWO_PI - IGNITE_WINDOW) ? PHASE_IGNITE : PHASE_COMPRESS;
    } else if (c < TWO_PI + IGNITE_WINDOW) {
      e->phase = PHASE_IGNITE;
    } else if (c < THREE_PI) {
      e->phase = PHASE_POWER;
    } else if (c < FOUR_PI) {
      e->exhaust_valve_open = true;
      e->phase = PHASE_EXHAUST;
    } else if (c < FOUR_PI + WATER_WINDOW) {
      e->water_inj_open = true;
      e->phase = PHASE_WATER_INJ;
    } else if (c < FIVE_PI) {
      e->phase = PHASE_STEAM_POWER;
    } else {
      e->exhaust_valve_open = true;
      e->phase = PHASE_STEAM_EXHAUST;
    }
    break;
  }
  }
}

/* ── §6 draw ── */

static void safeaddch(WINDOW *w, int r, int c, chtype ch) {
  int rows, cols;
  getmaxyx(w, rows, cols);
  if (r >= 0 && r < rows && c >= 0 && c < cols)
    mvwaddch(w, r, c, ch);
}

static void safeaddstr(WINDOW *w, int r, int c, const char *s) {
  int rows, cols;
  getmaxyx(w, rows, cols);
  if (r < 0 || r >= rows)
    return;
  for (int i = 0; s[i] && c + i < cols; i++)
    if (c + i >= 0)
      mvwaddch(w, r, c + i, (unsigned char)s[i]);
}

/* Bresenham line — draws character ch along the line (r0,c0)->(r1,c1). */
static void draw_line_ch(WINDOW *w, int r0, int c0, int r1, int c1, chtype ch) {
  int dr = abs(r1 - r0), dc = abs(c1 - c0);
  int sr = (r0 < r1) ? 1 : -1, sc = (c0 < c1) ? 1 : -1;
  int err = dr - dc;
  int rows, cols;
  getmaxyx(w, rows, cols);
  for (int i = 0; i < 400; i++) {
    if (r0 >= 0 && r0 < rows && c0 >= 0 && c0 < cols)
      mvwaddch(w, r0, c0, ch);
    if (r0 == r1 && c0 == c1)
      break;
    int e2 = 2 * err;
    if (e2 > -dc) {
      err -= dc;
      r0 += sr;
    }
    if (e2 < dr) {
      err += dr;
      c0 += sc;
    }
  }
}

/* ── §6.1 walls ── */
/* The 2-stroke breathes through holes in the cylinder wall, so its
 * inner wall has gaps cut at two fixed rows. */
static void draw_walls_2stroke(WINDOW *w, int engine_top, int center_col,
                               const Engine *e) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  int cyl_top = engine_top + HEAD_H;
  int case_top = engine_top + CASE_TOP_OFF;

  wattron(w, COLOR_PAIR(CP_WALL));
  for (int r = cyl_top; r < case_top; r++) {
    bool at_ex = (r == engine_top + EX_PORT_OFF);
    bool at_tr = (r == engine_top + TR_PORT_OFF);
    safeaddch(w, r, lo, '|');
    safeaddch(w, r, ro, '|');
    if (!(at_ex && e->ex_port_open))
      safeaddch(w, r, li, '|');
    if (!(at_tr && e->tr_port_open))
      safeaddch(w, r, ri, '|');
  }
  wattroff(w, COLOR_PAIR(CP_WALL));
}

/* The 4/6-stroke walls are solid; fresh and spent gas move through
 * valves in the head instead, so nothing is cut into the wall. */
static void draw_walls_solid(WINDOW *w, int engine_top, int center_col) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  int cyl_top = engine_top + HEAD_H;
  int case_top = engine_top + CASE_TOP_OFF;

  wattron(w, COLOR_PAIR(CP_WALL));
  for (int r = cyl_top; r < case_top; r++) {
    safeaddch(w, r, lo, '|');
    safeaddch(w, r, ro, '|');
    safeaddch(w, r, li, '|');
    safeaddch(w, r, ri, '|');
  }
  wattroff(w, COLOR_PAIR(CP_WALL));
}

/* ── §6.2 head ── */
static void draw_head_2stroke(WINDOW *w, int engine_top, int center_col,
                              bool spark) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;

  wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
  for (int c = lo; c <= ro; c++)
    safeaddch(w, engine_top, c, (c == lo || c == ro) ? '+' : '-');
  wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);

  if (spark) {
    wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    safeaddch(w, engine_top, center_col - 2, '-');
    safeaddch(w, engine_top, center_col - 1, '[');
    safeaddch(w, engine_top, center_col, '*');
    safeaddch(w, engine_top, center_col + 1, ']');
    safeaddch(w, engine_top, center_col + 2, '-');
    wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
  } else {
    wattron(w, COLOR_PAIR(CP_WALL));
    safeaddch(w, engine_top, center_col - 1, '[');
    safeaddch(w, engine_top, center_col, 'i');
    safeaddch(w, engine_top, center_col + 1, ']');
    wattroff(w, COLOR_PAIR(CP_WALL));
  }
}

/*
 * The 4-stroke lid: a spark plug in the middle, an exhaust valve to its
 * left and an intake valve to its right. A valve shows as '#' when shut
 * and 'v' when open.
 */
static void draw_head_4stroke(WINDOW *w, int engine_top, int center_col,
                              bool spark, bool intake_open, bool exhaust_open) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  int v_left = center_col - VALVE_OFF;
  int v_right = center_col + VALVE_OFF;

  /* Head fill — leave the valve and spark-plug slots for later. */
  wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
  for (int c = lo; c <= ro; c++) {
    chtype ch = '-';
    if (c == lo || c == ro) {
      ch = '+';
    } else if (c == v_left || c == v_right) {
      continue;
    } else if (c >= center_col - 1 && c <= center_col + 1) {
      continue;
    }
    safeaddch(w, engine_top, c, ch);
  }
  wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);

  /* Spark plug. */
  if (spark) {
    wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    safeaddch(w, engine_top, center_col - 1, '[');
    safeaddch(w, engine_top, center_col, '*');
    safeaddch(w, engine_top, center_col + 1, ']');
    wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
  } else {
    wattron(w, COLOR_PAIR(CP_WALL));
    safeaddch(w, engine_top, center_col - 1, '[');
    safeaddch(w, engine_top, center_col, 'i');
    safeaddch(w, engine_top, center_col + 1, ']');
    wattroff(w, COLOR_PAIR(CP_WALL));
  }

  /* Exhaust valve (left). */
  wattron(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);
  safeaddch(w, engine_top, v_left, exhaust_open ? 'v' : '#');
  wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);

  /* Intake valve (right). */
  wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
  safeaddch(w, engine_top, v_right, intake_open ? 'v' : '#');
  wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
}

/* 6-stroke head: 4-stroke layout plus a water injector above the head. */
static void draw_head_6stroke(WINDOW *w, int engine_top, int center_col,
                              bool spark, bool intake_open, bool exhaust_open,
                              bool water_open) {
  draw_head_4stroke(w, engine_top, center_col, spark, intake_open,
                    exhaust_open);

  int wr = engine_top - 1;
  if (wr < 0)
    return;
  wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
  safeaddch(w, wr, center_col, water_open ? 'w' : 'W');
  wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
}

/* ── §6.3 2-stroke port stubs ── */
/*
 * Little pipe stubs poking out of the wall holes, drawn even when the
 * piston covers the hole so you can still see where it is. When a hole
 * is shut we draw a dim pipe outline; when it's open we draw bright
 * gas streaming out (spent gas leaves on the left, fresh gas enters on
 * the right) with a faint trail behind it.
 */
static void draw_ports_2stroke(WINDOW *w, int engine_top, int center_col,
                               const Engine *e) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  int er = engine_top + EX_PORT_OFF;
  int tr = engine_top + TR_PORT_OFF;

  /* Exhaust manifold (left). */
  if (e->ex_port_open) {
    wattron(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);
    safeaddch(w, er - 1, lo - 1, '/');
    safeaddch(w, er + 1, lo - 1, '\\');
    for (int c = lo - 5; c < lo; c++)
      safeaddch(w, er, c, '~');
    wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_BOLD);
    wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    for (int c = lo - 9; c < lo - 5; c++)
      safeaddch(w, er, c, '.');
    wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
  } else {
    wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    safeaddch(w, er - 1, lo - 1, '/');
    safeaddch(w, er + 1, lo - 1, '\\');
    for (int c = lo - 5; c < lo; c++)
      safeaddch(w, er, c, '-');
    wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
  }

  /* Transfer duct (right). */
  if (e->tr_port_open) {
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
    safeaddch(w, tr - 1, ro + 1, '\\');
    safeaddch(w, tr + 1, ro + 1, '/');
    for (int c = ro + 1; c <= ro + 5; c++)
      safeaddch(w, tr, c, '>');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    for (int c = ro + 6; c <= ro + 9; c++)
      safeaddch(w, tr, c, '+');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
  } else {
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    safeaddch(w, tr - 1, ro + 1, '\\');
    safeaddch(w, tr + 1, ro + 1, '/');
    for (int c = ro + 1; c <= ro + 5; c++)
      safeaddch(w, tr, c, '-');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
  }
}

/* ── §6.3b 4/6-stroke head pipes ── */
/*
 * The intake and exhaust pipes that run sideways out of the head. We
 * always show them: a dim '-' when the valve is shut, bright gas chars
 * when it's open. They're what makes a valve engine look different from
 * the 2-stroke, whose breathing happens at the wall instead.
 */
static void draw_manifolds_valves(WINDOW *w, int engine_top, int center_col,
                                  const Engine *e) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  int row = engine_top;

  /* Exhaust manifold extending left. */
  chtype ex_attr = e->exhaust_valve_open ? (COLOR_PAIR(CP_EXHAUST) | A_BOLD)
                                         : (COLOR_PAIR(CP_EXHAUST) | A_DIM);
  char ex_ch = e->exhaust_valve_open ? '~' : '-';
  wattron(w, ex_attr);
  for (int c = lo - 5; c < lo; c++)
    safeaddch(w, row, c, ex_ch);
  wattroff(w, ex_attr);

  /* Intake manifold extending right. */
  chtype in_attr = e->intake_valve_open ? (COLOR_PAIR(CP_INTAKE) | A_BOLD)
                                        : (COLOR_PAIR(CP_INTAKE) | A_DIM);
  char in_ch = e->intake_valve_open ? '>' : '-';
  wattron(w, in_attr);
  for (int c = ro + 1; c <= ro + 5; c++)
    safeaddch(w, row, c, in_ch);
  wattroff(w, in_attr);
}

/* ── §6.4 gas above piston ── */
/* Fill the space above the piston with a different look for each phase,
 * so you can read what the gas is doing at a glance: a spark flash, hot
 * burning gas, smoky exhaust, fresh charge, swirling steam, and so on. */
static void draw_gas_above_piston(WINDOW *w, int engine_top, int center_col,
                                  int crown_row, Phase phase) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int cyl_top = engine_top + HEAD_H;

  switch (phase) {
  case PHASE_IGNITE:
    wattron(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, ((r + c) & 1) ? '*' : '^');
    wattroff(w, COLOR_PAIR(CP_SPARK) | A_BOLD);
    break;

  case PHASE_POWER: {
    static const char pch[] = "^~`";
    wattron(w, COLOR_PAIR(CP_FIRE));
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, pch[(r + c) % 3]);
    wattroff(w, COLOR_PAIR(CP_FIRE));
    break;
  }

  case PHASE_EXHAUST:
    wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, '~');
    wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
    break;

  case PHASE_INTAKE:
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, '+');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    break;

  case PHASE_SCAVENGE: {
    int mid = (li + ri) / 2;
    for (int r = cyl_top; r < crown_row; r++) {
      wattron(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
      for (int c = li + 1; c <= mid; c++)
        safeaddch(w, r, c, '~');
      wattroff(w, COLOR_PAIR(CP_EXHAUST) | A_DIM);
      wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
      for (int c = mid + 1; c < ri; c++)
        safeaddch(w, r, c, '+');
      wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    }
    break;
  }

  case PHASE_WATER_INJ:
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, ((r + c) & 1) ? '.' : ',');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_BOLD);
    break;

  case PHASE_STEAM_POWER:
    wattron(w, COLOR_PAIR(CP_SPARK));
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, ((r + c) % 3) == 0 ? '%' : '"');
    wattroff(w, COLOR_PAIR(CP_SPARK));
    break;

  case PHASE_STEAM_EXHAUST:
    wattron(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    for (int r = cyl_top; r < crown_row; r++)
      for (int c = li + 1; c < ri; c++)
        safeaddch(w, r, c, '"');
    wattroff(w, COLOR_PAIR(CP_INTAKE) | A_DIM);
    break;

  case PHASE_COMPRESS:
  default:
    /* Compressed charge intentionally invisible. */
    break;
  }
}

/* ── §6.5 piston / conrod / crank / case ── */
static void draw_piston(WINDOW *w, int center_col, int crown_row) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  wattron(w, COLOR_PAIR(CP_PISTON) | A_BOLD);
  for (int r = crown_row; r < crown_row + PISTON_H; r++) {
    safeaddch(w, r, li, '[');
    for (int c = li + 1; c < ri; c++)
      safeaddch(w, r, c, (r == crown_row) ? '=' : '#');
    safeaddch(w, r, ri, ']');
  }
  wattroff(w, COLOR_PAIR(CP_PISTON) | A_BOLD);
}

static void draw_conrod(WINDOW *w, int center_col, int wp_row, int cp_row,
                        int cp_col) {
  wattron(w, COLOR_PAIR(CP_CONROD));
  draw_line_ch(w, wp_row, center_col, cp_row, cp_col, ':');
  safeaddch(w, wp_row, center_col, 'o');
  wattroff(w, COLOR_PAIR(CP_CONROD));
}

static void draw_crank(WINDOW *w, int cc_row, int center_col, int cp_row,
                       int cp_col) {
  /* A ring of dots tracing the crank pin's path. Cells are taller than
   * they are wide, so we squash the height to make the ring look round. */
  wattron(w, COLOR_PAIR(CP_CRANK) | A_DIM);
  for (int deg = 0; deg < 360; deg += 12) {
    float a = (float)deg * (float)M_PI / 180.0f;
    int er = cc_row + (int)roundf((float)CRANK_R * 0.5f * sinf(a));
    int ec = center_col + (int)roundf((float)CRANK_R * cosf(a));
    safeaddch(w, er, ec, '.');
  }
  wattroff(w, COLOR_PAIR(CP_CRANK) | A_DIM);

  wattron(w, COLOR_PAIR(CP_CRANK));
  draw_line_ch(w, cc_row, center_col, cp_row, cp_col, '-');
  safeaddch(w, cc_row, center_col, 'O');
  wattron(w, A_BOLD);
  safeaddch(w, cp_row, cp_col, 'o');
  wattroff(w, A_BOLD);
  wattroff(w, COLOR_PAIR(CP_CRANK));
}

static void draw_case(WINDOW *w, int engine_top, int center_col, int cc_r) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  int case_top = engine_top + CASE_TOP_OFF;
  int case_bot = engine_top + CASE_BOT_OFF;
  int case_lw = center_col - CASE_HW;
  int case_rw = center_col + CASE_HW;

  wattron(w, COLOR_PAIR(CP_WALL));
  /* Top bar — leave the cylinder bore open. */
  for (int c = case_lw; c <= case_rw; c++) {
    chtype ch;
    if (c == case_lw || c == case_rw)
      ch = '+';
    else if (c >= lo && c <= ro)
      ch = ' ';
    else
      ch = '-';
    safeaddch(w, case_top, c, ch);
  }
  for (int r = case_top + 1; r < case_bot; r++) {
    safeaddch(w, r, case_lw, '|');
    safeaddch(w, r, case_rw, '|');
  }
  for (int c = case_lw; c <= case_rw; c++)
    safeaddch(w, case_bot, c, (c == case_lw || c == case_rw) ? '+' : '-');
  /* Power-takeoff stub on the right. */
  safeaddch(w, cc_r, case_rw, '>');
  safeaddch(w, cc_r, case_rw + 1, '=');
  safeaddch(w, cc_r, case_rw + 2, '>');
  wattroff(w, COLOR_PAIR(CP_WALL));
}

/* ── §6.6 stroke timeline ── */
/*
 * A little strip drawn under the engine that lays the whole cycle out
 * as a row of labelled blocks (one per stroke) with a '^' marker
 * pointing at where we are right now. More strokes means more blocks,
 * so the strip's busyness hints at how involved the cycle is.
 *
 * It's split into small named helpers (work out the layout, find the
 * current block, look up labels, draw the opening bracket, draw a
 * block, draw the marker) so the top-level routine reads like a recipe.
 */

/*
 * TimelineLayout — where the strip sits and how big its blocks are,
 * worked out once before we draw anything.
 *
 * Longer cycles get narrower blocks so the whole strip stays about the
 * same width and stays centered under the cylinder.
 */
typedef struct {
  int row;       /* which row the labelled blocks go on              */
  int start_col; /* the strip's leftmost column                     */
  int block_w;   /* one block's width, including its separator       */
  int total_w;   /* full strip width: strokes*block_w + closing ']'  */
  bool fits;     /* false if the strip won't fit on screen           */
} TimelineLayout;

static TimelineLayout timeline_layout_compute(WINDOW *w, int engine_top,
                                              int center_col, int strokes) {
  int rows, cols;
  getmaxyx(w, rows, cols);

  TimelineLayout L;
  L.row = engine_top + ENGINE_H + 1;
  L.block_w = (strokes == 2) ? 22 : (strokes == 4) ? 11 : 9;
  L.total_w = strokes * L.block_w + 1;
  L.start_col = center_col - L.total_w / 2;
  L.fits = (L.row + 1 < rows - 1) && (L.start_col >= 0) &&
           (L.start_col + L.total_w <= cols);
  return L;
}

/* Which stroke are we in right now? Each stroke takes up an equal slice
 * of the cycle, so we just see how many slices we've gone past. */
static int current_stroke_from_cycle_angle(float cycle_angle, int strokes) {
  int s = (int)(cycle_angle / (float)M_PI);
  if (s < 0)
    s = 0;
  if (s >= strokes)
    s = strokes - 1;
  return s;
}

/* The short text labels shown on the blocks, one set per engine. */
static const char **stroke_labels_for_type(EngineType t) {
  static const char *lbl_2s[] = {"POWER + EXH/SCAV", "COMPRESSION"};
  static const char *lbl_4s[] = {"INTAKE", "COMPRESS", "POWER", "EXHAUST"};
  static const char *lbl_6s[] = {"INTAKE",  "COMPRES", "POWER",
                                 "EXHAUST", "STM PWR", "STM EXH"};
  return (t == ENG_2STROKE) ? lbl_2s : (t == ENG_4STROKE) ? lbl_4s : lbl_6s;
}

/* Draw one block with its centered label and the divider after it.
 * The block we're currently in is drawn bright; the rest stay dim. */
static void render_stroke_block(WINDOW *w, int row, int block_start,
                                int inner_w, const char *label, bool is_active,
                                bool is_last) {
  int lbl_len = (int)strlen(label);
  if (lbl_len > inner_w)
    lbl_len = inner_w;
  int padding = (inner_w - lbl_len) / 2;

  chtype attr = is_active ? (COLOR_PAIR(CP_PHASE) | A_BOLD)
                          : (COLOR_PAIR(CP_WALL) | A_DIM);
  wattron(w, attr);
  for (int k = 0; k < inner_w; k++) {
    int lp = k - padding;
    char ch = (lp >= 0 && lp < lbl_len) ? label[lp] : ' ';
    safeaddch(w, row, block_start + k, ch);
  }
  wattroff(w, attr);

  wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
  safeaddch(w, row, block_start + inner_w, is_last ? ']' : '|');
  wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);
}

static void render_ribbon_opener(WINDOW *w, const TimelineLayout *L) {
  wattron(w, COLOR_PAIR(CP_WALL) | A_BOLD);
  safeaddch(w, L->row, L->start_col, '[');
  wattroff(w, COLOR_PAIR(CP_WALL) | A_BOLD);
}

/* Draw the '^' marker under the strip, placed left-to-right in step
 * with how far through the cycle we are (0 = start, 1 = end). It's
 * nudged so it never lands on top of a divider. */
static void render_cycle_position_caret(WINDOW *w, int row,
                                        const TimelineLayout *L,
                                        float cycle_fraction) {
  int c = L->start_col + 1 + (int)(cycle_fraction * (L->total_w - 2));
  if (c < L->start_col + 1)
    c = L->start_col + 1;
  if (c > L->start_col + L->total_w - 2)
    c = L->start_col + L->total_w - 2;
  wattron(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
  safeaddch(w, row, c, '^');
  wattroff(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
}

/* Put the whole strip together from the pieces above. */
static void draw_stroke_timeline(WINDOW *w, const Engine *e, int engine_top,
                                 int center_col) {
  int strokes = engine_strokes(e->type);
  TimelineLayout L =
      timeline_layout_compute(w, engine_top, center_col, strokes);
  if (!L.fits)
    return;

  int cur_stroke = current_stroke_from_cycle_angle(e->cycle_angle, strokes);
  const char **labels = stroke_labels_for_type(e->type);

  /* "[ block_0 | block_1 | ... | block_{N-1} ]" */
  render_ribbon_opener(w, &L);
  for (int s = 0; s < strokes; s++) {
    int block_start = L.start_col + 1 + s * L.block_w;
    render_stroke_block(w, L.row, block_start, L.block_w - 1, labels[s],
                        s == cur_stroke, s == strokes - 1);
  }

  float cycle_fraction = e->cycle_angle / engine_cycle_total(e->type);
  render_cycle_position_caret(w, L.row + 1, &L, cycle_fraction);
}

/* ── §6.7 scene_draw ── */
/*
 * Draw one frame in five steps, in the order things sit from back to
 * front so nearer parts paint over farther ones:
 *
 *   1. Work out where the moving parts are this instant.
 *   2. Work out the phase and which valves and holes are open.
 *   3. Draw the fixed parts: walls first, then the gas, then the head
 *      on top so its valves and injector show over the gas.
 *   4. Draw the moving parts (piston, rod, crank), then the case box
 *      around them.
 *   5. Draw the read-out text and the cycle strip on top.
 *
 * Each step is a single helper call so this reads top to bottom like a
 * recipe; the helpers below say why each one exists.
 */

/* Step 1: find the four key positions together in one go, so the rod
 * always meets the piston and crank exactly (see the Kinematics doc). */
static Kinematics solve_frame_kinematics(const Engine *e, int engine_top,
                                         int center_col) {
  float crank_th = engine_crank_angle(e);
  float cc_row_f = (float)(engine_top + CRANK_CENTER_OFF);
  float cc_col_f = (float)center_col;
  return solve_slider_crank(crank_th, cc_row_f, cc_col_f);
}

/* Step 3 (back): the cylinder walls. The 2-stroke's wall has holes in
 * it; the others are solid because they breathe through the head. */
static void render_cylinder_walls(WINDOW *w, int engine_top, int center_col,
                                  const Engine *e) {
  if (e->type == ENG_2STROKE)
    draw_walls_2stroke(w, engine_top, center_col, e);
  else
    draw_walls_solid(w, engine_top, center_col);
}

/* Step 3 (front): the head on top of the cylinder, drawn after the gas
 * so its valves and injector stay visible. This is where the three
 * engines look most different:
 *   2-stroke -> just a spark plug, with the wall-hole pipe stubs.
 *   4-stroke -> spark plug plus two valves and side pipes.
 *   6-stroke -> same as the 4-stroke plus a water sprayer above it.
 */
static void render_cylinder_head(WINDOW *w, int engine_top, int center_col,
                                 const Engine *e) {
  bool spark = (e->phase == PHASE_IGNITE);
  switch (e->type) {
  case ENG_2STROKE:
    draw_head_2stroke(w, engine_top, center_col, spark);
    draw_ports_2stroke(w, engine_top, center_col, e);
    break;
  case ENG_4STROKE:
    draw_head_4stroke(w, engine_top, center_col, spark, e->intake_valve_open,
                      e->exhaust_valve_open);
    draw_manifolds_valves(w, engine_top, center_col, e);
    break;
  case ENG_6STROKE:
    draw_head_6stroke(w, engine_top, center_col, spark, e->intake_valve_open,
                      e->exhaust_valve_open, e->water_inj_open);
    draw_manifolds_valves(w, engine_top, center_col, e);
    break;
  }
}

/* Step 4: the moving column that turns the push of the gas into a
 * spinning shaft: piston, then rod, then crank. We draw them in this
 * order so the rod sits on top of the crank where they meet, like a
 * real cutaway drawing. */
static void render_reciprocating_train(WINDOW *w, int center_col, int cc_r,
                                       const Kinematics *k) {
  draw_piston(w, center_col, k->crown_row);
  draw_conrod(w, center_col, k->wrist_row, k->crank_pin_r, k->crank_pin_c);
  draw_crank(w, cc_r, center_col, k->crank_pin_r, k->crank_pin_c);
}

/* Step 5: the read-out text beside the cylinder — the current phase
 * name and the two angle numbers. Just for the viewer; it doesn't
 * touch the engine. */
static void render_phase_telemetry(WINDOW *w, int engine_top, int center_col,
                                   const Engine *e) {
  int ri = center_col + CYL_IHW;
  int ro = ri + CYL_WALL;

  wattron(w, COLOR_PAIR(CP_PHASE) | A_BOLD);
  safeaddstr(w, engine_top + 3, ro + 3, phase_name(e->phase));
  wattroff(w, COLOR_PAIR(CP_PHASE) | A_BOLD);

  char buf[40];
  float crank_th = engine_crank_angle(e);
  int deg_crank = (int)(crank_th * 180.0f / (float)M_PI) % 360;
  int deg_cycle = (int)(e->cycle_angle * 180.0f / (float)M_PI);
  wattron(w, COLOR_PAIR(CP_HUD));
  snprintf(buf, sizeof(buf), "crank %3d deg", deg_crank);
  safeaddstr(w, engine_top + 5, ro + 3, buf);
  snprintf(buf, sizeof(buf), "cycle %4d deg", deg_cycle);
  safeaddstr(w, engine_top + 6, ro + 3, buf);
  wattroff(w, COLOR_PAIR(CP_HUD));
}

/* Step 5: little "EX" / "TR" tags next to the 2-stroke's wall holes
 * that pop up while each hole is open. */
static void render_port_state_overlay(WINDOW *w, int engine_top, int center_col,
                                      const Engine *e) {
  int li = center_col - CYL_IHW, ri = center_col + CYL_IHW;
  int lo = li - CYL_WALL, ro = ri + CYL_WALL;
  wattron(w, COLOR_PAIR(CP_HUD));
  safeaddstr(w, engine_top + EX_PORT_OFF, lo - 3,
             e->ex_port_open ? "EX" : "  ");
  safeaddstr(w, engine_top + TR_PORT_OFF, ro + 3,
             e->tr_port_open ? "TR" : "  ");
  wattroff(w, COLOR_PAIR(CP_HUD));
}

static void scene_draw(WINDOW *w, Engine *e, int engine_top, int center_col) {
  /* 1. Find where the piston, rod, and crank are this instant. */
  Kinematics k = solve_frame_kinematics(e, engine_top, center_col);
  int cc_r = engine_top + CRANK_CENTER_OFF;

  /* 2. Work out the phase and which valves and holes are open now. */
  engine_derive_state(e, k.crown_row, engine_top);

  /* 3. Fixed parts, back to front: walls, then gas, then the head on
   *    top so its valves and injector show over the gas. */
  render_cylinder_walls(w, engine_top, center_col, e);
  draw_gas_above_piston(w, engine_top, center_col, k.crown_row, e->phase);
  render_cylinder_head(w, engine_top, center_col, e);

  /* 4. Moving parts, then the case box around them. */
  render_reciprocating_train(w, center_col, cc_r, &k);
  draw_case(w, engine_top, center_col, cc_r);

  /* 5. Read-out text and the cycle strip on top. */
  render_phase_telemetry(w, engine_top, center_col, e);
  if (e->type == ENG_2STROKE)
    render_port_state_overlay(w, engine_top, center_col, e);
  draw_stroke_timeline(w, e, engine_top, center_col);
}

/* ── §7 scene ── */

/*
 * Scene — everything the main loop needs, in one bundle it can pass
 * around as a single pointer.
 *
 * The fields fall into two groups, kept apart on purpose:
 *
 *   What the engine does — the engine itself and how fast we step it.
 *     These survive resizes, theme changes, and engine switches, and
 *     are changed by reset, the RPM keys, pause, and the engine keys.
 *
 *   What the screen looks like — just the chosen color theme. Changing
 *     it (the t/T keys) must never change the engine: the same engine
 *     under a different theme runs exactly the same.
 *
 * The split is here so the next person knows which side a new field
 * belongs on. Putting one on the wrong side is how you'd end up with
 * "pause" quietly swapping the theme, or theme-cycling nudging the RPM.
 */
typedef struct {
  /* ── what the engine does ── */
  Engine engine; /* the whole engine: type, where it is, RPM, paused */
  int sim_fps;   /* how many times a second we step the engine; set
                  *   apart from the ~60 fps drawing rate             */

  /* ── what the screen looks like ── */
  int theme_idx; /* which color theme is active; index into THEMES[],
                  *   cycled by t/T. The engine never reads this.     */
} Scene;

static void scene_init(Scene *s) {
  engine_reset(&s->engine, ENG_2STROKE);
  s->sim_fps = SIM_FPS_DEFAULT;
  s->theme_idx = 0;
}

/* ── §8 screen ── */

static volatile sig_atomic_t g_resize = 0;
static volatile sig_atomic_t g_quit = 0;

static void on_sigwinch(int s) {
  (void)s;
  g_resize = 1;
}
static void on_sigterm(int s) {
  (void)s;
  g_quit = 1;
}

static void screen_init(int theme_idx) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  color_init(theme_idx);
}

static void screen_cleanup(void) {
  curs_set(1);
  endwin();
}

/*
 * Two-bar HUD per project convention:
 *   row 0       — engine type / fps / sim rate / RPM / paused state,
 *                 bright yellow + bold
 *   row 1       — theme readout, bright yellow (no bold)
 *   row rows-1  — interactive key hints, bright cyan + bold
 */
static void draw_hud(WINDOW *w, const Scene *s, float fps) {
  int rows, cols;
  getmaxyx(w, rows, cols);

  char top[100];
  snprintf(top, sizeof(top), " %s  %5.1f fps  sim:%3d Hz  RPM:%3d  %s ",
           engine_name(s->engine.type), (double)fps, s->sim_fps, s->engine.rpm,
           s->engine.paused ? "PAUSED " : "running");
  int top_len = (int)strlen(top);
  wattron(w, COLOR_PAIR(CP_HUD) | A_BOLD);
  safeaddstr(w, 0, cols - top_len, top);
  wattroff(w, COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Row 1 left: cycle subtitle naming the thermodynamic cycle. */
  char sub_left[80];
  snprintf(sub_left, sizeof(sub_left), " %s ", engine_subtitle(s->engine.type));
  wattron(w, COLOR_PAIR(CP_HUD));
  safeaddstr(w, 1, 0, sub_left);
  wattroff(w, COLOR_PAIR(CP_HUD));

  /* Row 1 right: theme readout. */
  char sub[40];
  snprintf(sub, sizeof(sub), " theme: %-7s ", THEMES[s->theme_idx].name);
  int sub_len = (int)strlen(sub);
  wattron(w, COLOR_PAIR(CP_HUD));
  safeaddstr(w, 1, cols - sub_len, sub);
  wattroff(w, COLOR_PAIR(CP_HUD));

  const char *hint =
      " q:quit  spc:pause  r:reset  ]/[:RPM  n/p:engine  t/T:theme ";
  wattron(w, COLOR_PAIR(CP_HINT) | A_BOLD);
  safeaddstr(w, rows - 1, 0, hint);
  wattroff(w, COLOR_PAIR(CP_HINT) | A_BOLD);
}

/* ── §9 app ── */

int main(void) {
  signal(SIGWINCH, on_sigwinch);
  signal(SIGTERM, on_sigterm);
  signal(SIGINT, on_sigterm);

  Scene scene;
  scene_init(&scene);

  screen_init(scene.theme_idx);

  int64_t last_time = clock_ns();
  int64_t sim_accum = 0;
  int64_t fps_accum = 0;
  int fps_frames = 0;
  float fps_disp = 0.0f;
  int64_t frame_ns = TICK_NS(60); /* render cap ~60 fps */

  int scr_rows, scr_cols;
  getmaxyx(stdscr, scr_rows, scr_cols);

  for (;;) {
    if (g_resize) {
      g_resize = 0;
      endwin();
      refresh();
      getmaxyx(stdscr, scr_rows, scr_cols);
    }
    if (g_quit)
      break;

    int64_t now = clock_ns();
    int64_t dt_ns = now - last_time;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;
    last_time = now;
    sim_accum += dt_ns;
    fps_accum += dt_ns;
    fps_frames++;

    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_disp = (float)fps_frames * 1e9f / (float)fps_accum;
      fps_accum = 0;
      fps_frames = 0;
    }

    int64_t tick = TICK_NS(scene.sim_fps);
    while (sim_accum >= tick) {
      float dt = (float)tick / (float)NS_PER_SEC;
      engine_tick(&scene.engine, dt);
      sim_accum -= tick;
    }

    /* Reserve rows 0-1 for HUD; row 2 holds the 6-stroke water
     * injector marker above the head without overlapping HUD. */
    int engine_top = (scr_rows - ENGINE_H) / 2;
    if (engine_top < 3)
      engine_top = 3;
    int center_col = scr_cols / 2;

    erase();
    scene_draw(stdscr, &scene.engine, engine_top, center_col);
    draw_hud(stdscr, &scene, fps_disp);
    wnoutrefresh(stdscr);
    doupdate();

    int ch;
    while ((ch = getch()) != ERR) {
      switch (ch) {
      case 'q':
      case 27:
        g_quit = 1;
        break;
      case ' ':
        scene.engine.paused = !scene.engine.paused;
        break;
      case 'r':
        engine_reset(&scene.engine, scene.engine.type);
        break;
      case ']':
        if (scene.engine.rpm < RPM_MAX)
          scene.engine.rpm += RPM_STEP;
        break;
      case '[':
        if (scene.engine.rpm > RPM_MIN)
          scene.engine.rpm -= RPM_STEP;
        break;
      case 'n':
        engine_set_type(&scene.engine,
                        (EngineType)((scene.engine.type + 1) % N_ENGINES));
        break;
      case 'p':
        engine_set_type(
            &scene.engine,
            (EngineType)((scene.engine.type + N_ENGINES - 1) % N_ENGINES));
        break;
      case 't':
        scene.theme_idx = (scene.theme_idx + 1) % N_THEMES;
        color_apply_theme(scene.theme_idx);
        break;
      case 'T':
        scene.theme_idx = (scene.theme_idx + N_THEMES - 1) % N_THEMES;
        color_apply_theme(scene.theme_idx);
        break;
      case KEY_RESIZE:
        g_resize = 1;
        break;
      }
    }

    int64_t elapsed = clock_ns() - now;
    clock_sleep_ns(frame_ns - elapsed);
  }

  screen_cleanup();
  return 0;
}
