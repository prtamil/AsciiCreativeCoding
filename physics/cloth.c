/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cloth.c — a hanging sheet of cloth, simulated as a grid of little weights
 * joined by springs.  Gravity pulls it down, wind pushes it sideways, and the
 * springs fight to keep their shape; every thread is coloured by how hard it's
 * being stretched, so you watch tension ripple through the fabric.  Ten preset
 * setups (flag, sail, hammock, ...) change which corners are pinned and how the
 * wind blows; 14 colour themes cycle with t/T.
 *
 * The classic recipe for this is Provot 1995, "Deformation Constraints in a
 * Mass-Spring Model to Describe Rigid Cloth Behaviour" (Graphics Interface
 * '95) — it's where the three spring kinds (structural / shear / bend) come
 * from.  physics/chain.c is a sibling demo that solves the same problem a
 * different way (position-based dynamics instead of forces).
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
  SIM_FPS_MIN = 10,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 10,
  FPS_UPDATE_MS = 500,

  CLOTH_W = 30,  /* how many weights across the sheet         */
  CLOTH_H = 18,  /* how many weights down the sheet           */
  NODE_GAP = 2,  /* terminal cells between neighbouring weights */
  SUB_STEPS = 8, /* split each physics tick into this many tiny steps */
  N_PRESETS = 10,
  N_THEMES = 14,

  WIND_MIN = 0,   /* px/s² — weakest wind we allow             */
  WIND_MAX = 150, /* px/s² — strongest wind we allow           */
  WIND_STEP = 10, /* px/s² — how much +/- changes wind by      */
};

#define CLOTH_N (CLOTH_W * CLOTH_H)

/* Each weight links to at most 6 springs (right/down × 3 spring kinds), so
 * this is a safe upper bound on how many springs we'll ever build.        */
#define MAX_SPRINGS (CLOTH_W * CLOTH_H * 6)

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* Physics runs in "pixels", with each terminal cell worth CELL_W × CELL_H of
 * them, so motion stays smooth even though the screen is coarse.          */
#define CELL_W 8
#define CELL_H 16

/* How far apart neighbouring weights sit when the cloth is relaxed (pixels). */
#define REST_H (CELL_W * NODE_GAP)
#define REST_V (CELL_H * NODE_GAP)

/* How hard gravity pulls everything down (px/s²).  Gentler than chain.c's 380
 * — the cloth has many weights sharing the load, so a big value makes the top
 * springs snap and overshoot before damping calms them.  200 drapes nicely.  */
#define GRAVITY 200.0f

/* How fast the swaying wind oscillates (cycles per second).  0.40 Hz is one
 * gentle swing every ~2.5 s — a flutter, not a shake.                      */
#define WIND_FREQ 0.40f

/* How much speed each tiny step keeps (the rest leaks away as drag).  Just shy
 * of 1, so motion barely loses energy per step but the cloth still settles
 * after a few seconds of accumulated steps.                                */
#define DAMP 0.9993f

/* The three kinds of spring, ordered stiffest to softest.  K is how hard a
 * spring pulls back per pixel of stretch; KD is how strongly it resists being
 * stretched/squashed *quickly* (it eats the bouncy oscillation energy).
 * Structural springs hold the weave; shear ones stop diagonal skew; bend ones
 * keep the sheet from creasing.  All are far below the value that would make
 * the simulation explode.                                                  */
#define K_STRUCT 400.0f
#define K_SHEAR 100.0f
#define K_BEND 40.0f
#define KD_STRUCT 4.0f
#define KD_SHEAR 2.0f
#define KD_BEND 0.5f

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

/* ── §3 color — themes + HUD chrome ── */

/*
 * The colour slots we use.  The four ROW colours form a gradient from slack
 * threads to taut ones; PIN highlights the points holding the cloth up; the
 * two HUD slots are deliberately fixed (not theme-driven) so the status text
 * stays readable no matter which cloth palette is on screen.
 */
enum {
  CP_ROW_0 = 1,
  CP_ROW_1 = 2,
  CP_ROW_2 = 3,
  CP_ROW_3 = 4,
  CP_PIN = 5,
  CP_HUD = 6,
  CP_HINT = 7,
};

/*
 * Theme — one named colour scheme for the cloth.
 *
 *   name  — what the t/T menu shows.
 *   ramp  — four colours laid along the strain gradient, ramp[0] for the
 *           slackest threads up to ramp[3] for the tautest.  Each theme
 *           keeps a consistent feel (some cool→warm, some dark→bright).
 *   pin   — a bright accent for the anchor points holding the cloth up.
 *
 * Colours are kept in the bright half of the 256-colour space on purpose;
 * the very dark indices vanish against a black terminal background (see
 * CLAUDE.md's brightness rule).  The last four entries are single-hue
 * "unicolor" themes — one colour ramped from dim to bright — for a classic
 * glowing-CRT look without any rainbow.
 */
typedef struct {
  const char *name;
  short ramp[4];
  short pin;
} Theme;

static const Theme k_themes[N_THEMES] = {
    /*  name        ramp[0]  ramp[1]  ramp[2]  ramp[3]   pin                */
    {"Matrix", {28, 34, 40, 46}, 82},       /* cyber green   */
    {"Fire", {130, 208, 202, 196}, 226},    /* warm → red    */
    {"Oceanic", {24, 31, 39, 51}, 195},     /* teal → cyan   */
    {"Neon", {129, 165, 201, 213}, 51},     /* purple → pink */
    {"Mono", {240, 247, 250, 255}, 231},    /* grayscale     */
    {"Ice", {153, 117, 159, 195}, 231},     /* light blues   */
    {"Nova", {129, 141, 177, 213}, 226},    /* stellar       */
    {"Forest", {58, 100, 142, 190}, 226},   /* leaves/bark   */
    {"Desert", {130, 178, 214, 220}, 226},  /* sand/gold     */
    {"Eclipse", {240, 244, 124, 196}, 226}, /* gray + red    */
    /* ── single-hue themes (one colour from dim to bright) ── */
    {"Amber", {130, 172, 214, 220}, 226},  /* CRT amber     */
    {"Crimson", {88, 124, 160, 196}, 231}, /* deep red      */
    {"Azure", {25, 32, 39, 45}, 195},      /* solid blue    */
    {"Violet", {54, 91, 129, 165}, 213},   /* solid purple  */
};

static void theme_apply(int t) {
  const Theme *th = &k_themes[t % N_THEMES];
  if (COLORS >= 256) {
    init_pair(CP_ROW_0, th->ramp[0], -1);
    init_pair(CP_ROW_1, th->ramp[1], -1);
    init_pair(CP_ROW_2, th->ramp[2], -1);
    init_pair(CP_ROW_3, th->ramp[3], -1);
    init_pair(CP_PIN, th->pin, -1);
    /* Status bars are the same on every theme so they stay readable. */
    init_pair(CP_HUD, 226, -1); /* bright yellow */
    init_pair(CP_HINT, 51, -1); /* bright cyan   */
  } else {
    /* On terminals with only 8 colours, fall back to fixed picks. */
    init_pair(CP_ROW_0, COLOR_CYAN, -1);
    init_pair(CP_ROW_1, COLOR_GREEN, -1);
    init_pair(CP_ROW_2, COLOR_YELLOW, -1);
    init_pair(CP_ROW_3, COLOR_RED, -1);
    init_pair(CP_PIN, COLOR_WHITE, -1);
    init_pair(CP_HUD, COLOR_YELLOW, -1);
    init_pair(CP_HINT, COLOR_CYAN, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();
  theme_apply(0);
}

/* ── §4 coords — pixel ↔ cell ── */

static inline int px_to_cell_x(float px) {
  return (int)floorf(px / (float)CELL_W + 0.5f);
}
static inline int px_to_cell_y(float py) {
  return (int)floorf(py / (float)CELL_H + 0.5f);
}

/* ── §5 entity — Node, Spring, Cloth ── */

/*
 * Node — one little weight in the cloth.  We don't simulate continuous
 * fabric; we sprinkle a CLOTH_W × CLOTH_H grid of these weights and let the
 * springs (next struct) pull them around.  Each weight carries three kinds of
 * info, grouped below by who touches them and when:
 *
 *   - position and speed: where the weight is and how fast it's moving. This
 *     is the part physics keeps rewriting, many tiny steps per frame.
 *   - a render snapshot (rx, ry): where the weight sat at the start of the
 *     current tick. Drawing blends from this toward the live position so the
 *     motion looks smooth even though physics jumps in 8 discrete sub-steps;
 *     without it you'd see the cloth stutter.
 *   - pinned: whether this weight is nailed in place. Pinned weights never
 *     move — gravity and wind ignore them. Which weights are pinned is exactly
 *     what makes a flag a flag and a hammock a hammock.
 *
 * Reference: Provot 1995 §2 (treating cloth as a grid of weights and springs).
 */
typedef struct {
  /* where it is and how fast it's moving (pixels; +y points down, matching
   * the screen).  Rewritten every tiny physics step.                     */
  float x, y;
  float vx, vy;

  /* where it sat at the start of this tick; drawing blends from here to the
   * live (x, y) so motion stays smooth between physics steps.            */
  float rx, ry;

  /* true means nailed in place — never moves, set once when a preset loads. */
  bool pinned;
} Node;

/*
 * Spring — one stretchy connection between two weights.  When it's stretched
 * past its relaxed length it pulls the two ends together; when squashed it
 * pushes them apart (the classic spring law, Hooke).  It also has a little
 * "shock absorber" that fights quick stretching or squashing, so the cloth
 * settles instead of bouncing forever.
 *
 * The three "kinds" of spring aren't stored as a type — they're just springs
 * built with different reach and stiffness when the cloth is wired up:
 *
 *   structural  : joins next-door weights (across / down)       — stiffest
 *   shear       : joins diagonal weights, stops the grid skewing — medium
 *   bend        : reaches over one weight, stops sharp creases   — softest
 *
 * Each kind is about 10× softer than the last (Provot 1995 §3).  Soften the
 * bend springs and the cloth wrinkles freely; stiffen them and you get rigid
 * canvas.
 */
typedef struct {
  /* the two weights this spring connects (indices into Cloth::nodes[]). */
  int a, b;

  /* relaxed length in pixels — no force when the ends are exactly this far
   * apart.  Set once when the cloth is built, then left alone.           */
  float rest;
  /* stiffness: how hard it pulls back per pixel of stretch.              */
  float k;
  /* the shock absorber: how strongly it resists being stretched quickly.
   * Needed because a plain spring would store energy and bounce forever. */
  float kd;
} Spring;

/* ── Wind modes ── */

/*
 * WindMode — how the wind blows for a given preset.  Each preset picks the
 * one that suits its pinning, giving it a recognisable look:
 *
 *   WIND_SIN      swings smoothly side to side (a sheet swaying)
 *   WIND_CONST_R  steady push to the right (flies a flag out)
 *   WIND_CONST_L  steady push to the left (fills a sail)
 *   WIND_GUST     mostly one way, with quick pulses on top
 *   WIND_NONE     no wind at all — just gravity
 */
typedef enum {
  WIND_SIN = 0,
  WIND_CONST_R,
  WIND_CONST_L,
  WIND_GUST,
  WIND_NONE,
} WindMode;

/*
 * Cloth — everything the simulation needs, in one struct.  There's no global
 * state: the physics and drawing functions all take a Cloth* and work on it,
 * which keeps the line between "moving the cloth" and "drawing the cloth" just
 * one function call wide.
 *
 * The weight and spring arrays are fixed-size, filled in once at startup and
 * never resized (this demo never allocates memory while running).  Fields are
 * grouped below by what touches them, not by type.
 *
 * Reference: Baraff & Witkin 1998 §6 (what state a cloth simulation needs).
 */
typedef struct {
  /* The cloth body: the weights and the springs joining them.  nodes[] is
   * stored row by row (node_idx maps col,row → index).  Only the first
   * n_springs entries of springs[] are real; the rest is spare room.      */
  Node nodes[CLOTH_N];
  Spring springs[MAX_SPRINGS];
  int n_springs;

  /* The wind.  It keeps its own little clock (wind_phase) that ticks forward
   * every frame; wind_mode picks how that clock turns into a push direction,
   * wind_strength scales it, and wind_on is the on/off switch.             */
  float wind_phase;    /* where we are in the wind's swing cycle (radians) */
  float wind_strength; /* how hard the wind pushes (px/s²); +/- adjusts it */
  bool wind_on;        /* the w key's on/off toggle                        */
  WindMode wind_mode;  /* which wind pattern this preset uses              */

  /* Run state the user controls with keys. */
  bool paused; /* space bar; true means the cloth holds still      */
  int preset;  /* which of the ten scenarios is currently loaded   */
} Cloth;

/* ── Helpers ── */

static inline int node_idx(int col, int row) { return row * CLOTH_W + col; }

static void cloth_add_spring(Cloth *c, int a, int b, float rest, float k,
                             float kd) {
  if (c->n_springs >= MAX_SPRINGS)
    return;
  Spring *sp = &c->springs[c->n_springs++];
  sp->a = a;
  sp->b = b;
  sp->rest = rest;
  sp->k = k;
  sp->kd = kd;
}

/* ── Preset initialisation ── */

/* Lay the weights out in a neat relaxed grid, with the top-left corner at
 * pixel (ox0, oy0) and the spacing taken from REST_H / REST_V.            */
static void cloth_reset_positions(Cloth *c, float ox0, float oy0) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      Node *n = &c->nodes[node_idx(col, row)];
      n->x = n->rx = ox0 + (float)col * REST_H;
      n->y = n->ry = oy0 + (float)row * REST_V;
      n->vx = n->vy = 0.0f;
      n->pinned = false;
    }
  }
}

/* The stiff springs that hold the weave together: each weight gets one to its
 * right neighbour and one below it, so the cloth resists being pulled apart
 * like the warp and weft of real fabric.                                    */
static void add_structural_springs(Cloth *c) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      int idx = node_idx(col, row);
      if (col + 1 < CLOTH_W)
        cloth_add_spring(c, idx, node_idx(col + 1, row), (float)REST_H,
                         K_STRUCT, KD_STRUCT);
      if (row + 1 < CLOTH_H)
        cloth_add_spring(c, idx, node_idx(col, row + 1), (float)REST_V,
                         K_STRUCT, KD_STRUCT);
    }
  }
}

/* The medium springs that stop the cloth from skewing: each little square in
 * the grid gets both of its diagonals, so the squares can't slump sideways
 * into slanted diamonds.                                                     */
static void add_shear_springs(Cloth *c) {
  float diag = sqrtf((float)(REST_H * REST_H + REST_V * REST_V));
  for (int row = 0; row + 1 < CLOTH_H; row++) {
    for (int col = 0; col + 1 < CLOTH_W; col++) {
      cloth_add_spring(c, node_idx(col, row), node_idx(col + 1, row + 1), diag,
                       K_SHEAR, KD_SHEAR);
      cloth_add_spring(c, node_idx(col + 1, row), node_idx(col, row + 1), diag,
                       K_SHEAR, KD_SHEAR);
    }
  }
}

/* The soft springs that keep the sheet from creasing: each weight links to the
 * one two steps away (across and down), reaching over its neighbour.  The cloth
 * can still curve gently, but it won't fold into a sharp knife-edge.          */
static void add_bend_springs(Cloth *c) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      int idx = node_idx(col, row);
      if (col + 2 < CLOTH_W)
        cloth_add_spring(c, idx, node_idx(col + 2, row), (float)(REST_H * 2),
                         K_BEND, KD_BEND);
      if (row + 2 < CLOTH_H)
        cloth_add_spring(c, idx, node_idx(col, row + 2), (float)(REST_V * 2),
                         K_BEND, KD_BEND);
    }
  }
}

/* Build all three kinds of spring that hold the cloth together.  The order
 * doesn't affect the result — the forces add up the same either way — it just
 * follows the paper (Provot 1995) so it's easy to compare.                    */
static void cloth_build_springs(Cloth *c) {
  c->n_springs = 0;
  add_structural_springs(c);
  add_shear_springs(c);
  add_bend_springs(c);
}

/*
 * Each preset is responsible for:
 *   1. Calling cloth_reset_positions(c, ox0, oy0) with a sensible origin
 *   2. Marking pinned nodes via c->nodes[...].pinned = true
 *   3. Setting wind_strength, wind_on, wind_mode (and optionally wind_phase)
 *
 * All ten preset functions share this shape so cycling through them with
 * n/p produces ten visually distinct cloth scenarios.
 */

/*
 * preset 0 — Hanging Cloth ★ default boot preset
 * Top row fully pinned; gravity drapes the body downward; gentle
 * sinusoidal wind sways the whole sheet side-to-side.
 */
static void preset_hanging(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  for (int col = 0; col < CLOTH_W; col++)
    c->nodes[node_idx(col, 0)].pinned = true;

  /* Start at quarter-period so wind is at full strength immediately. */
  c->wind_phase = (float)M_PI * 0.5f;
  c->wind_strength = 30.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
}

/*
 * preset 1 — Flag
 * Left column fully pinned (a flagpole); constant rightward wind keeps
 * the flag flying out to the right.
 */
static void preset_flag(Cloth *c, int cols, int rows) {
  float ox0 = (float)(CELL_W * 3);
  float oy0 = (float)(rows * CELL_H) * 0.15f;

  cloth_reset_positions(c, ox0, oy0);
  for (int row = 0; row < CLOTH_H; row++)
    c->nodes[node_idx(0, row)].pinned = true;

  c->wind_strength = 40.0f;
  c->wind_on = true;
  c->wind_mode = WIND_CONST_R;
  (void)cols;
}

/*
 * preset 2 — Hammock
 * Only the two top corners pinned; the body hangs in a deep catenary
 * curve under gravity, with sinusoidal gusts rocking it.
 */
static void preset_hammock(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.08f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;

  c->wind_strength = 50.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
  (void)rows;
}

/*
 * preset 3 — Curtain
 * Top row pinned every 4 columns (plus the rightmost column), creating
 * scalloped pleats that sway between pins.  Gentle sin wind.
 */
static void preset_curtain(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  for (int col = 0; col < CLOTH_W; col += 4)
    c->nodes[node_idx(col, 0)].pinned = true;
  /* Always pin the last column so the right edge is anchored too. */
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;

  c->wind_phase = (float)M_PI * 0.5f;
  c->wind_strength = 25.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
}

/*
 * preset 4 — Banner
 * A single pin at the top-left corner; strong constant rightward wind
 * sweeps the entire cloth out into a streaming banner.
 */
static void preset_banner(Cloth *c, int cols, int rows) {
  float ox0 = (float)(CELL_W * 3);
  float oy0 = (float)(rows * CELL_H) * 0.15f;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;

  c->wind_strength = 60.0f;
  c->wind_on = true;
  c->wind_mode = WIND_CONST_R;
  (void)cols;
}

/*
 * preset 5 — Tapestry
 * Both top and bottom rows pinned — the cloth is stretched between two
 * horizontal rails like a wall hanging.  Gusty wind makes the middle
 * billow in and out unpredictably.
 */
static void preset_tapestry(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  for (int col = 0; col < CLOTH_W; col++) {
    c->nodes[node_idx(col, 0)].pinned = true;
    c->nodes[node_idx(col, CLOTH_H - 1)].pinned = true;
  }

  c->wind_strength = 50.0f;
  c->wind_on = true;
  c->wind_mode = WIND_GUST;
  (void)rows;
}

/*
 * preset 6 — Tablecloth
 * A single pin at the centre of the top row — the cloth drapes radially
 * around one point.  Gentle sin wind sways the entire sheet as one body.
 */
static void preset_tablecloth(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)(rows * CELL_H) * 0.05f + (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(CLOTH_W / 2, 0)].pinned = true;

  c->wind_strength = 15.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
}

/*
 * preset 7 — Sail
 * Three corners pinned (top-left, top-right, bottom-right) — a
 * triangular sail configuration.  Constant leftward wind fills the sail
 * by pushing the unconstrained bottom-left toward the bottom-right.
 */
static void preset_sail(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, CLOTH_H - 1)].pinned = true;

  c->wind_strength = 70.0f;
  c->wind_on = true;
  c->wind_mode = WIND_CONST_L;
  (void)rows;
}

/*
 * preset 8 — Trampoline
 * All four corners pinned and wind off — gravity pulls the centre into
 * a clean dish shape so you can see the springs settle.
 */
static void preset_trampoline(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, 0)].pinned = true;
  c->nodes[node_idx(0, CLOTH_H - 1)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, CLOTH_H - 1)].pinned = true;

  c->wind_strength = 0.0f;
  c->wind_on = false;
  c->wind_mode = WIND_NONE;
  (void)rows;
}

/*
 * preset 9 — Sash
 * Diagonal corner pins (top-left + bottom-right) — the cloth twists
 * between them, sagging on the unconstrained diagonal.  Sin wind makes
 * the twisted form sway.
 */
static void preset_sash(Cloth *c, int cols, int rows) {
  int cloth_px_w = (CLOTH_W - 1) * REST_H;
  float ox0 = (float)((cols * CELL_W) - cloth_px_w) * 0.5f;
  float oy0 = (float)CELL_H * 2;

  cloth_reset_positions(c, ox0, oy0);
  c->nodes[node_idx(0, 0)].pinned = true;
  c->nodes[node_idx(CLOTH_W - 1, CLOTH_H - 1)].pinned = true;

  c->wind_strength = 30.0f;
  c->wind_on = true;
  c->wind_mode = WIND_SIN;
  (void)rows;
}

static const char *preset_names[N_PRESETS] = {
    "Hanging Cloth", "Flag",       "Hammock", "Curtain",    "Banner",
    "Tapestry",      "Tablecloth", "Sail",    "Trampoline", "Sash",
};

static void cloth_init(Cloth *c, int preset, int cols, int rows) {
  memset(c, 0, sizeof *c);
  c->paused = false;
  c->preset = preset;
  c->wind_phase = 0.0f;

  switch (preset) {
  default:
  case 0:
    preset_hanging(c, cols, rows);
    break;
  case 1:
    preset_flag(c, cols, rows);
    break;
  case 2:
    preset_hammock(c, cols, rows);
    break;
  case 3:
    preset_curtain(c, cols, rows);
    break;
  case 4:
    preset_banner(c, cols, rows);
    break;
  case 5:
    preset_tapestry(c, cols, rows);
    break;
  case 6:
    preset_tablecloth(c, cols, rows);
    break;
  case 7:
    preset_sail(c, cols, rows);
    break;
  case 8:
    preset_trampoline(c, cols, rows);
    break;
  case 9:
    preset_sash(c, cols, rows);
    break;
  }

  cloth_build_springs(c);
}

/* ── Physics tick ── */

/* Turn the current wind mode and its little clock into one number from -1
 * (full push left) to +1 (full push right).  The whole cloth feels this same
 * push; the modes only differ in how this one number swings over time.       */
static float wind_dir_from_mode(const Cloth *c) {
  switch (c->wind_mode) {
  case WIND_CONST_R:
    return 1.0f;
  case WIND_CONST_L:
    return -1.0f;
  case WIND_GUST:
    return 0.6f + 0.4f * sinf(c->wind_phase * 2.0f);
  case WIND_NONE:
    return 0.0f;
  case WIND_SIN:
  default:
    return sinf(c->wind_phase);
  }
}

/* How hard the wind pushes one weight sideways.  The push grows stronger toward
 * the bottom of the sheet, so the free lower edge billows out more than the
 * pinned top — just like a real sheet flapping in the breeze.                 */
static inline float scalar_wind_force(const Cloth *c, float y_frac,
                                      float wind_dir) {
  return c->wind_strength * wind_dir * (0.4f + 0.6f * y_frac);
}

/* The outside pushes on every free weight: gravity pulling down and wind
 * pushing sideways.  These act on each weight on its own, no neighbours
 * involved.  Pinned weights get zero — they never move.                       */
static void accumulate_external_forces(const Cloth *c, float *ax, float *ay,
                                       float wind_dir) {
  for (int i = 0; i < CLOTH_N; i++) {
    if (c->nodes[i].pinned) {
      ax[i] = ay[i] = 0.0f;
      continue;
    }
    ax[i] = 0.0f;
    ay[i] = GRAVITY;
    if (!c->wind_on)
      continue;

    int row = i / CLOTH_W;
    float y_frac = (float)row / (float)(CLOTH_H - 1);
    ax[i] = scalar_wind_force(c, y_frac, wind_dir);
  }
}

/* Work out the pull one spring puts on its first weight.  Two parts add up: the
 * spring pulls harder the more it's stretched past its relaxed length, plus a
 * shock-absorber part that fights quick stretching so it doesn't bounce forever.
 * The other weight feels the exact opposite pull (the caller handles that).
 *
 * If the two weights land on top of each other there's no direction to push, so
 * we return zero rather than divide by zero and spit out garbage.             */
static void hooke_damped_force(const Node *na, const Node *nb, float rest,
                               float k, float kd, float *out_fx,
                               float *out_fy) {
  float dx = nb->x - na->x;
  float dy = nb->y - na->y;
  float dist = sqrtf(dx * dx + dy * dy);
  if (dist < 1e-6f) {
    *out_fx = 0.0f;
    *out_fy = 0.0f;
    return;
  }
  float inv = 1.0f / dist;
  float ex = dx * inv;
  float ey = dy * inv;
  float vrel = (nb->vx - na->vx) * ex + (nb->vy - na->vy) * ey;
  float fmag = k * (dist - rest) + kd * vrel;
  *out_fx = fmag * ex;
  *out_fy = fmag * ey;
}

/* Add up every spring's pull on the weights it joins.  Each spring tugs its two
 * ends in opposite directions; a pinned end just ignores the tug.             */
static void accumulate_spring_forces(const Cloth *c, float *ax, float *ay) {
  for (int s = 0; s < c->n_springs; s++) {
    const Spring *sp = &c->springs[s];
    const Node *na = &c->nodes[sp->a];
    const Node *nb = &c->nodes[sp->b];

    float fx, fy;
    hooke_damped_force(na, nb, sp->rest, sp->k, sp->kd, &fx, &fy);

    if (!na->pinned) {
      ax[sp->a] += fx;
      ay[sp->a] += fy;
    }
    if (!nb->pinned) {
      ax[sp->b] -= fx;
      ay[sp->b] -= fy;
    }
  }
}

/* Push every free weight forward a tiny step in time.  The trick is the order:
 * update the speed first using the forces, then move the weight using that NEW
 * speed.  Doing it this way keeps the cloth from slowly gaining fake energy and
 * blowing up, which the obvious order would.  DAMP bleeds a sliver of speed off
 * each step so the cloth eventually comes to rest.
 *
 * This ordering is the "symplectic Euler" method (Hairer, Lubich & Wanner 2006,
 * ch. VI) — symplectic just means it keeps the energy honest over time.       */
static void integrate_symplectic_euler(Cloth *c, const float *ax,
                                       const float *ay, float dt) {
  for (int i = 0; i < CLOTH_N; i++) {
    Node *n = &c->nodes[i];
    if (n->pinned)
      continue;
    n->vx = (n->vx + ax[i] * dt) * DAMP;
    n->vy = (n->vy + ay[i] * dt) * DAMP;
    n->x += n->vx * dt;
    n->y += n->vy * dt;
  }
}

/* One tiny step of the simulation: figure out the wind, add up the outside
 * forces (gravity + wind) and the spring forces, then move everything forward a
 * hair.  The body below reads as those four steps in order.                   */
static void cloth_step(Cloth *c, float dt) {
  float ax[CLOTH_N], ay[CLOTH_N]; /* per-node acceleration accumulators */

  float wind_dir = wind_dir_from_mode(c);
  accumulate_external_forces(c, ax, ay, wind_dir);
  accumulate_spring_forces(c, ax, ay);
  integrate_symplectic_euler(c, ax, ay, dt);
}

static void cloth_tick(Cloth *c, float dt) {
  if (c->paused)
    return;

  /* Remember where every weight is right now, before physics moves it.  Drawing
   * blends from this saved spot toward the new one, so the cloth glides between
   * physics steps instead of jumping.                                         */
  for (int i = 0; i < CLOTH_N; i++) {
    c->nodes[i].rx = c->nodes[i].x;
    c->nodes[i].ry = c->nodes[i].y;
  }

  float sub_dt = dt / (float)SUB_STEPS;
  c->wind_phase += WIND_FREQ * 2.0f * (float)M_PI * dt;
  if (c->wind_phase > 2.0f * (float)M_PI)
    c->wind_phase -= 2.0f * (float)M_PI;

  for (int s = 0; s < SUB_STEPS; s++) {
    cloth_step(c, sub_dt);
  }
}

/* ── Drawing ── */

/* Draw a straight line of characters from one cell to another, picking the
 * character that best matches the line's slant: '-' for mostly flat, '|' for
 * mostly upright, and '\' or '/' for the two diagonals.                       */
static void draw_segment(WINDOW *w, int x0, int y0, int x1, int y1, int cols,
                         int rows, chtype attr) {
  int dx = x1 - x0;
  int dy = y1 - y0;
  int steps = abs(dx) > abs(dy) ? abs(dx) : abs(dy);
  if (steps == 0) {
    if (x0 >= 0 && x0 < cols && y0 >= 1 && y0 < rows - 1) {
      wattron(w, attr);
      mvwaddch(w, y0, x0, '+');
      wattroff(w, attr);
    }
    return;
  }

  int adx = abs(dx), ady = abs(dy);
  char ch;
  if (adx >= 2 * ady)
    ch = '-';
  else if (ady >= 2 * adx)
    ch = '|';
  else if (dx * dy > 0)
    ch = '\\';
  else
    ch = '/';

  wattron(w, attr);
  for (int k = 0; k <= steps; k++) {
    int cx = x0 + k * dx / steps;
    int cy = y0 + k * dy / steps;
    if (cx >= 0 && cx < cols && cy >= 1 && cy < rows - 1)
      mvwaddch(w, cy, cx, ch);
  }
  wattroff(w, attr);
}

/* Find where a weight should be drawn this frame and which terminal cell that
 * lands in.  alpha (0 to 1) says how far we are between the last physics step
 * and the next, so we blend between the saved spot and the live one.          */
static inline void node_lerp_cell(const Node *n, float alpha, int *out_cx,
                                  int *out_cy) {
  float draw_x = n->rx + (n->x - n->rx) * alpha;
  float draw_y = n->ry + (n->y - n->ry) * alpha;
  *out_cx = px_to_cell_x(draw_x);
  *out_cy = px_to_cell_y(draw_y);
}

/* Pick a colour for one thread based on how stretched it is.  The four colours
 * run dim/cool for slack threads up to bright/hot for taut ones, so the picture
 * works as a live tension map — and because every theme is built that way, the
 * effect survives no matter which palette is on.  This is what makes the cloth
 * feel alive: waves of colour ripple through it as it swings.
 *
 *   squashed a bit      → ROW_0 (the coolest colour)
 *   near its rest length → ROW_1
 *   stretched, holding load → ROW_2
 *   very taut            → ROW_3 (the hottest colour)
 *
 * The upright threads near the top of a hanging sheet hold the weight of
 * everything below, so they glow hottest; slack folds sit cool.              */
static inline int strain_tier(const Node *a, const Node *b, float rest) {
  float dx = b->x - a->x;
  float dy = b->y - a->y;
  float dist = sqrtf(dx * dx + dy * dy);
  float s = (dist - rest) / rest;
  if (s < -0.02f)
    return CP_ROW_0;
  else if (s < 0.02f)
    return CP_ROW_1;
  else if (s < 0.08f)
    return CP_ROW_2;
  else
    return CP_ROW_3;
}

/* Draw the thread between two neighbouring weights, coloured by how stretched
 * it is right now.                                                            */
static void draw_strained_edge(WINDOW *w, const Node *na, const Node *nb,
                               float rest, float alpha, int cols, int rows) {
  int ax, ay, bx, by;
  node_lerp_cell(na, alpha, &ax, &ay);
  node_lerp_cell(nb, alpha, &bx, &by);
  int tier = strain_tier(na, nb, rest);
  draw_segment(w, ax, ay, bx, by, cols, rows, (chtype)COLOR_PAIR(tier));
}

/* Draw the whole sheet as a mesh of threads.  For each weight we draw the thread
 * to its right neighbour and the one below it, colouring each by how stretched it
 * is — so the picture reads as a live tension map that ripples as the cloth
 * swings.                                                                      */
static void draw_strain_weave(const Cloth *c, WINDOW *w, int cols, int rows,
                              float alpha) {
  for (int row = 0; row < CLOTH_H; row++) {
    for (int col = 0; col < CLOTH_W; col++) {
      const Node *n = &c->nodes[node_idx(col, row)];
      if (col + 1 < CLOTH_W) {
        const Node *nr = &c->nodes[node_idx(col + 1, row)];
        draw_strained_edge(w, n, nr, (float)REST_H, alpha, cols, rows);
      }
      if (row + 1 < CLOTH_H) {
        const Node *nd = &c->nodes[node_idx(col, row + 1)];
        draw_strained_edge(w, n, nd, (float)REST_V, alpha, cols, rows);
      }
    }
  }
}

/* Mark the pinned weights — the points holding the cloth up — with a bright '#'.
 * Drawn last so they sit on top of any thread crossing them.  Free weights get
 * no marker on purpose; letting only the threads show keeps the picture reading
 * as fabric instead of dotted graph paper.                                    */
static void draw_pinned_anchors(const Cloth *c, WINDOW *w, int cols, int rows,
                                float alpha) {
  for (int i = 0; i < CLOTH_N; i++) {
    const Node *n = &c->nodes[i];
    if (!n->pinned)
      continue;
    int cx, cy;
    node_lerp_cell(n, alpha, &cx, &cy);
    if (cx < 0 || cx >= cols || cy < 1 || cy >= rows - 1)
      continue;
    wattron(w, COLOR_PAIR(CP_PIN) | A_BOLD);
    mvwaddch(w, cy, cx, '#');
    wattroff(w, COLOR_PAIR(CP_PIN) | A_BOLD);
  }
}

/* Draw the cloth in two passes: the mesh of threads first, then the pins on
 * top.                                                                        */
static void cloth_draw(const Cloth *c, WINDOW *w, int cols, int rows,
                       float alpha) {
  draw_strain_weave(c, w, cols, rows, alpha);
  draw_pinned_anchors(c, w, cols, rows, alpha);
}

/* ── §6 scene ── */

typedef struct {
  Cloth cloth;
  int theme; /* active index into k_themes[] */
} Scene;

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  cloth_init(&s->cloth, 0, cols, rows);
  s->theme = 0;
  theme_apply(s->theme);
}

static void scene_tick(Scene *s, float dt, int cols, int rows) {
  (void)cols;
  (void)rows;
  cloth_tick(&s->cloth, dt);
}

static void scene_draw(const Scene *s, WINDOW *w, int cols, int rows,
                       float alpha, float dt_sec) {
  (void)dt_sec;
  cloth_draw(&s->cloth, w, cols, rows, alpha);
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
 * Two-bar HUD per CLAUDE.md convention:
 *   row 0  right (CP_HUD  bright yellow + bold)  — live status
 *   row -1 left  (CP_HINT bright cyan   + bold)  — actions / keys
 * If the full key list overflows the terminal width, a shorter hint is
 * substituted so the bar always fits on one line.
 */
static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, s->cols, s->rows, alpha, dt_sec);

  const Cloth *c = &sc->cloth;

  /* ── top: status (row 0, right-aligned) ─────────────────────────── */
  char windbuf[16];
  if (c->wind_on)
    snprintf(windbuf, sizeof windbuf, "%3.0f", c->wind_strength);
  else
    snprintf(windbuf, sizeof windbuf, "OFF");

  char top[180];
  snprintf(top, sizeof top,
           " preset:%s  theme:%s  wind:%s  %s  sim:%3d Hz  %5.1f fps ",
           preset_names[c->preset], k_themes[sc->theme].name, windbuf,
           c->paused ? "PAUSED " : "running", sim_fps, fps);
  int len = (int)strlen(top);
  int hx = s->cols - len;
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, hx, top, s->cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* ── bottom: actions (row -1, left-aligned) ─────────────────────── */
  const char *hint_full =
      " q:quit  spc:pause  r:restart  n/p:preset  t/T:theme  "
      "w:wind  +/-:strength  ]/[:Hz ";
  const char *hint_short = " q:quit  spc:pause  n:preset  t:theme  +/-:wind ";
  const char *hint = hint_full;
  if ((int)strlen(hint_full) >= s->cols - 1)
    hint = hint_short;

  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(s->rows - 1, 0, hint, s->cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
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
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *scn = &app->scene;
  Cloth *c = &scn->cloth;
  Screen *sc = &app->screen;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    c->paused = !c->paused;
    break;

  case 'r':
  case 'R':
    cloth_init(c, c->preset, sc->cols, sc->rows);
    break;

  case 'n':
    cloth_init(c, (c->preset + 1) % N_PRESETS, sc->cols, sc->rows);
    break;

  case 'p': {
    int p = c->preset - 1;
    if (p < 0)
      p = N_PRESETS - 1;
    cloth_init(c, p, sc->cols, sc->rows);
    break;
  }

  case 't':
    scn->theme = (scn->theme + 1) % N_THEMES;
    theme_apply(scn->theme);
    break;
  case 'T':
    scn->theme = (scn->theme + N_THEMES - 1) % N_THEMES;
    theme_apply(scn->theme);
    break;

  case 'w':
  case 'W':
    c->wind_on = !c->wind_on;
    break;

  case '+':
  case '=': /* '=' is shift-less '+' on US layouts */
    c->wind_strength += (float)WIND_STEP;
    if (c->wind_strength > (float)WIND_MAX)
      c->wind_strength = (float)WIND_MAX;
    c->wind_on = true; /* turning up wind implies it's on   */
    break;
  case '-':
  case '_':
    c->wind_strength -= (float)WIND_STEP;
    if (c->wind_strength < (float)WIND_MIN)
      c->wind_strength = (float)WIND_MIN;
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

    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec, app->screen.cols, app->screen.rows);
      sim_accum -= tick_ns;
    }

    float alpha = (float)sim_accum / (float)tick_ns;

    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
