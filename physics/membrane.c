/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * physics/membrane.c — a vibrating drumhead in the terminal.
 *
 * Strike the membrane and watch ripples spread out, bounce off the
 * edges, cross through each other, and slowly fade. The terminal grid
 * IS the drum: one character cell = one point on the surface. Warm
 * colours show points pushed up (crests), cool colours show points
 * pushed down (troughs). The maths is the classic damped wave equation
 * solved by stepping forward in small time slices.
 *
 * References worth keeping (the code can't hand you these):
 *   [1] Rayleigh, *The Theory of Sound*, Vol. I (1877) — the standing-wave
 *       shapes the 'm' key plays.
 *   [4] Courant, Friedrichs, Lewy (1928) — the stability limit shown as
 *       "CFL_2D" in the overlay; the sim blows up if it exceeds 1.
 *   [5] LeVeque, *Finite Difference Methods* (2007) — the 5-point stencil
 *       and where the √2 in the 2-D limit comes from.
 *   [7] Hairer/Lubich/Wanner, *Geometric Numerical Integration* (2006) —
 *       why updating velocity before position keeps energy from drifting.
 *   [8] Chladni, *Entdeckungen über die Theorie des Klanges* (1787) — sand
 *       on a plate revealing the still "nodal lines" the 'l' key draws.
 *   [9] Ware, *Information Visualization* (2020) — the warm/cool split so
 *       the eye reads "up vs down" before it reads "how much".
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

/* ── loop / display ── */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 60,
  SIM_FPS_MAX = 120,
  SIM_FPS_STEP = 5,

  TARGET_FPS = 60,
  FPS_UPDATE_MS = 500,
  HUD_COLS = 80,
};

/* ── grid limits ── */
/*
 * The drum surface IS the character grid: the value at g_grid.u[row][col]
 * is drawn directly at terminal cell (col, row). No separate pixel space
 * like the moving-particle demos use. The two fields at max size take
 * about 240 KB, which lives quietly in zeroed static memory.
 */
#define GRID_MAX_W 300
#define GRID_MAX_H 100

/* ── wave physics defaults ── */
/*
 * Wave speed is in cells per second. Push it too high for the chosen
 * frame rate and the sim goes unstable; at 60 Hz the ceiling is about
 * 42. The default of 25 sits comfortably below that.
 */
#define WAVE_SPEED_DEFAULT 25.0f
#define WAVE_SPEED_MIN 5.0f
#define WAVE_SPEED_MAX 42.0f /* above this the sim goes unstable at 60 Hz */
#define WAVE_SPEED_STEP 3.0f

#define DAMPING_DEFAULT 0.003f /* gentle fade — a strike rings for ~5 s */
#define DAMPING_MIN 0.000f
#define DAMPING_MAX 0.060f
#define DAMPING_STEP 0.003f

/* ── excitation (how a strike is shaped) ── */
#define EXCITE_AMP 1.2f    /* how hard a strike pushes the surface */
#define EXCITE_RADIUS 3.5f /* how wide the dimple is, in cells     */
#define RESONANCE_AMP 1.0f /* strength of a played standing-wave shape */

/* ── rendering ── */
#define NODAL_SIGN_THRESH 0.015f /* below this |u| counts as "basically zero" */
#define DISPLAY_RANGE 1.5f       /* clip displayed values to this range */
#define DISPLAY_MAX_FLOOR                                                      \
  0.05f /* smallest brightness scale we'll use, so a near-silent      \
         * surface doesn't divide by something tiny                   */

/* ── named numbers the formulas below lean on ── */
#define HALF 0.5f /* the ½ in energy = ½·v² and ½·c²·(slope)² */
#define CDIFF_FACTOR                                                           \
  0.5f /* slope = (value ahead − value behind) / 2, since cells   \
        * are one apart [LeVeque ref 5]                           */
#define SQRT2 1.41421356f /* shows up in the stability limit c·dt·√2 [ref 4] */
#define INV_SQRT2 0.70710678f
/* 1/√2 — starts a played mode halfway through its swing (both
 * position and speed already non-zero) so it animates on frame one */

/* ── where each strike preset lands ── *
 * Positions are fractions of the surface: 0.5 is dead centre, 0.0 the
 * top/left edge, 1.0 the bottom/right.                                   */
#define STRIKE_X_CENTRE 0.50f
#define STRIKE_Y_CENTRE 0.50f
#define STRIKE_X_LEFT_EDGE                                                     \
  0.15f /* in from the left, off-centre enough to wake up many       \
         * overtones but not so close it touches the wall            */
#define STRIKE_X_DOUBLE_L 0.30f /* left hit of the double strike  */
#define STRIKE_Y_DOUBLE_L 0.35f
#define STRIKE_X_DOUBLE_R 0.70f /* right hit — mirrored left/right but the  */
#define STRIKE_Y_DOUBLE_R                                                      \
  0.65f /* vertical offsets tilt the pair off-centre, giving a       \
         * livelier, lopsided interference pattern                   */

/* Speed used to set the starting "kick" for a played standing wave. The
 * real wave_speed may differ — this just picks a tasteful initial swing;
 * the actual speed takes over from there.                               */
#define RESONANCE_KICK_REF_C WAVE_SPEED_DEFAULT

/* ── boundary conditions: what the edges do to a wave ── */
enum {
  BC_DIRICHLET = 0, /* pinned rim — wave bounces back flipped upside down */
  BC_NEUMANN = 1,   /* free rim   — wave bounces back the same way up     */
  BC_PERIODIC = 2,  /* wrap-around — leaves one side, enters the other    */
  BC_COUNT = 3,
};
static const char *const k_bc_names[BC_COUNT] = {"dirichlet", "neumann",
                                                 "periodic"};

/* ── presets (the canned strikes the keys trigger) ── */
enum {
  PRESET_CENTER = 0,
  PRESET_EDGE = 1,
  PRESET_DOUBLE = 2,
  PRESET_RESONANCE = 3,
  PRESET_COUNT = 4,
};
static const char *const k_preset_names[PRESET_COUNT] = {"center", "edge",
                                                         "double", "resonance"};

/* ── the standing-wave shapes 'm' cycles through ── *
 * Each (nx, ny) names how many half-waves span the width and height. The
 * plain (1,1) fundamental is last because it's the dullest to look at —
 * starting on (2,2) gives an obvious shape on the very first press.    */
static const int k_mode_table[][2] = {
    {2, 2}, {3, 2}, {2, 3}, {3, 3}, {4, 3},
    {3, 4}, {4, 4}, {1, 2}, {2, 1}, {1, 1},
};
#define MODE_TABLE_LEN ((int)(sizeof k_mode_table / sizeof k_mode_table[0]))

/* ── timing ── */
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
  struct timespec r = {
      .tv_sec = (time_t)(ns / NS_PER_SEC),
      .tv_nsec = (long)(ns % NS_PER_SEC),
  };
  nanosleep(&r, NULL);
}

/* ── §3 theme — colours and characters for the wave height ── */

/*
 * The character ramp: from blank at rest to solid at full push, the glyph
 * gets denser the harder the surface moves.  The same characters serve
 * both up and down — colour is what tells crest from trough.
 *
 *   ' '  at rest          '*'  strong
 *   '.'  barely moving     'X'  near-peak
 *   ':'  slight ripple     '#'  peak
 *   '+'  medium             '@'  maxed out
 *   'x'  strong
 */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1) /* 9 levels */

/* How far up the brightness scale [0..1] each character takes over. */
static const float k_breaks[RAMP_N] = {
    0.000f, 0.030f, 0.090f, 0.200f, 0.340f, 0.500f, 0.660f, 0.820f, 0.940f,
};

/* Pick which ramp character a 0..1 brightness maps to.  The little power
 * curve spreads the steps so they look evenly spaced to the eye rather
 * than evenly spaced in the raw numbers. */
static int lut_index(float v) {
  if (v <= 0.0f)
    return 0;
  if (v >= 1.0f)
    return RAMP_N - 1;
  float g = powf(v, 1.0f / 2.2f);
  for (int i = RAMP_N - 1; i >= 0; i--)
    if (g >= k_breaks[i])
      return i;
  return 0;
}

/*
 * Three colour schemes, each with a warm set for crests (up) and a cool
 * set for troughs (down):
 *   "wave"    — blue down,   red/yellow up
 *   "thermal" — violet down, yellow/white up
 *   "ocean"   — deep-blue down, cyan/white up
 *
 * ncurses can't colour a character directly; it makes you register each
 * foreground/background combination in a numbered slot first.  The macros
 * below work out the slot number for a given theme, up/down sign, and
 * brightness, so we register every combination once at startup and just
 * switch by theme afterward.  The HUD and nodal-line colours sit past the
 * themed slots and never change. */
#define N_THEMES 3
#define CP_POS(t, i) (1 + (t) * (RAMP_N * 2) + (i))
#define CP_NEG(t, i) (1 + (t) * (RAMP_N * 2) + RAMP_N + (i))
#define CP_NODAL (1 + N_THEMES * (RAMP_N * 2))
#define CP_HUD (CP_NODAL + 1)
#define CP_HINT (CP_NODAL + 2)

/*
 * WaveTheme — one colour scheme: a warm set of shades for crests (up) and
 * a cool set for troughs (down), getting brighter as the surface moves
 * harder.  Splitting warm vs cool this way lets the eye read up-or-down
 * from the colour and how-much from the brightness, without checking a
 * legend. (Ware [ref 9].)
 *
 * Why one struct holds all four arrays: the up set and down set must stay
 * in step — same number of brightness steps, same dark-to-bright march —
 * so a half-up crest looks as strong as a half-down trough.  Bundling them
 * makes it hard to edit one out of sync with the other.
 *
 * Two copies of each set: a rich 256-colour version for modern terminals,
 * and a coarse 8-colour fallback for plain ones.  color_init() checks what
 * the terminal supports and uses whichever fits.  The one thing both
 * guarantee is that up and down never look alike.
 *
 * The HUD and nodal-line colours live OUTSIDE this struct so that cycling
 * themes never disturbs them.
 */
typedef struct {
  const char *name; /* label shown in the HUD when cycling themes */

  /* the rich palette, used on 256-colour terminals.  Index 0 is the
   * faintest shade, RAMP_N-1 the brightest; the brightest couple also
   * get A_BOLD (added in wave_attr) so peaks pop against any background. */
  int pos256[RAMP_N]; /* crest shades — the warm (up) set   */
  int neg256[RAMP_N]; /* trough shades — the cool (down) set */

  /* the plain 8-colour fallback.  Coarser — several brightness steps may
   * share one colour — but up still never looks like down. */
  int pos8[RAMP_N];
  int neg8[RAMP_N];
} WaveTheme;

static const WaveTheme k_themes[N_THEMES] = {
    {
        /* 0  wave — blue troughs, red/yellow crests */
        "wave",
        /* pos: dark red → orange → yellow → white */
        {52, 88, 124, 160, 196, 202, 208, 220, 231},
        /* neg: dark blue → blue → cyan → white     */
        {17, 19, 21, 27, 33, 39, 45, 51, 231},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
         COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
    },
    {
        /* 1  thermal — violet troughs, yellow crests */
        "thermal",
        /* pos: dark orange → amber → yellow → white */
        {52, 94, 130, 166, 202, 208, 214, 220, 231},
        /* neg: dark violet → magenta → pink         */
        {53, 54, 91, 92, 129, 165, 201, 207, 231},
        {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
         COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
         COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
    },
    {
        /* 2  ocean — deep-blue troughs, cyan/white crests */
        "ocean",
        /* pos: teal → cyan → white */
        {23, 29, 36, 43, 51, 87, 123, 159, 231},
        /* neg: navy → midnight blue → indigo        */
        {17, 18, 19, 20, 21, 27, 33, 39, 45},
        {COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
         COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
        {COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE,
         COLOR_BLUE, COLOR_BLUE, COLOR_CYAN},
    },
};

static void color_init(void) {
  start_color();
  use_default_colors();
  for (int t = 0; t < N_THEMES; t++) {
    for (int i = 0; i < RAMP_N; i++) {
      if (COLORS >= 256) {
        init_pair(CP_POS(t, i), k_themes[t].pos256[i], COLOR_BLACK);
        init_pair(CP_NEG(t, i), k_themes[t].neg256[i], COLOR_BLACK);
      } else {
        init_pair(CP_POS(t, i), k_themes[t].pos8[i], COLOR_BLACK);
        init_pair(CP_NEG(t, i), k_themes[t].neg8[i], COLOR_BLACK);
      }
    }
  }
  /* nodal-line dots: neutral grey, same in every theme */
  if (COLORS >= 256)
    init_pair(CP_NODAL, 240, COLOR_BLACK);
  else
    init_pair(CP_NODAL, COLOR_WHITE, COLOR_BLACK);
  /* HUD status line — bright yellow (project standard) */
  if (COLORS >= 256)
    init_pair(CP_HUD, 226, -1);
  else
    init_pair(CP_HUD, COLOR_YELLOW, -1);

  /* HUD key-hints line — bright cyan (project standard) */
  if (COLORS >= 256)
    init_pair(CP_HINT, 51, -1);
  else
    init_pair(CP_HINT, COLOR_CYAN, -1);
}

/* Pick the colour for a cell from its theme, up/down sign, and brightness
 * step.  The brightest steps also get bold so peaks really stand out. */
static attr_t wave_attr(int theme, bool positive, int level) {
  attr_t a = positive ? COLOR_PAIR(CP_POS(theme, level))
                      : COLOR_PAIR(CP_NEG(theme, level));
  if (level >= RAMP_N - 2)
    a |= A_BOLD;
  return a;
}

/* ── §4 grid — the membrane's state: two fields plus its live size ── */

/*
 * Grid — everything that describes the drum surface right now: how high
 *        each point is, how fast it's moving, and how big the grid is.
 *
 * The wave equation is naturally about acceleration (how the height's
 * rate-of-change itself changes), which is awkward to step forward
 * directly.  So we track TWO things per point instead: its height (u)
 * and its up/down speed (v).  Each tick we nudge the speed using the
 * heights around it, then move the height by that speed.  Two simple
 * steps replace one second-order one.  (Rayleigh [ref 1], and the
 * energy-friendly ordering is from Hairer/Lubich/Wanner [ref 7].)
 *
 * Why bundle u, v, cols, rows into one struct: they're always used
 * together, so packing them means a function can never accidentally
 * pair the fields with a stale width or height — resizing is one
 * atomic update, and every helper just takes a Grid* and reads its
 * own dimensions.
 *
 * The fields are plain fixed-size arrays, not pointers-to-pointers.
 * That keeps them in zeroed static memory: the program starts with a
 * membrane already at rest (all zeros) with no setup code, and nothing
 * on the hot path ever calls malloc.  At max size the two arrays are
 * about 240 KB, which sits quietly in that static memory.
 *
 * Layout is row-major (u[row][col]) to match how the renderer and the
 * edge loops walk the grid — rows outside, columns inside — so each
 * inner scan stays friendly to the CPU cache.
 *
 * The grid owns the four edge rows/columns (the rim) but doesn't decide
 * how they behave on a bounce — that choice (the boundary condition) is
 * a user setting and lives in Scene, not here.
 */
typedef struct {
  /* height of each point, in cells.  Positive means pushed up (a
   * crest), negative means pushed down (a trough).  Read by every
   * draw pass; written by the solver, the edge rules, and strikes. */
  float u[GRID_MAX_H][GRID_MAX_W];
  /* up/down speed of each point — how fast its height is changing.
   * Updated first each tick, before the heights move.  Positive means
   * moving up right now, negative means moving down. */
  float v[GRID_MAX_H][GRID_MAX_W];
  int cols; /* live width  — one cell per terminal column (≤ GRID_MAX_W) */
  int rows; /* live height — one cell per terminal row    (≤ GRID_MAX_H) */
} Grid;

/* The one and only grid.  Lives in static memory, never malloc'd.  Every
 * physics and render helper reaches it through g_grid.u / .v / .cols /
 * .rows. */
static Grid g_grid;

/* ── §5 solver ── */

static void init_grid(int bc, int cols, int rows); /* defined below */

/* Reset every point to flat and motionless across the live grid.  Cells
 * past the current width/height are left alone — nothing ever reads them. */
static void grid_zero(void) {
  for (int r = 0; r < g_grid.rows; r++) {
    memset(g_grid.u[r], 0, (size_t)g_grid.cols * sizeof(float));
    memset(g_grid.v[r], 0, (size_t)g_grid.cols * sizeof(float));
  }
}

/* Pinned rim: hold all four edges flat and still.  Like a drumhead glued
 * to a rigid frame — the edge can't move, so a wave hitting it bounces
 * back flipped upside down (a crest returns as a trough). */
static void apply_dirichlet_bc(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                               int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    u[0][c] = 0.0f;
    v[0][c] = 0.0f;
    u[rows - 1][c] = 0.0f;
    v[rows - 1][c] = 0.0f;
  }
  for (int r = 0; r < rows; r++) {
    u[r][0] = 0.0f;
    v[r][0] = 0.0f;
    u[r][cols - 1] = 0.0f;
    v[r][cols - 1] = 0.0f;
  }
}

/* Free rim: let the edges float instead of pinning them.  We copy each
 * edge cell from its inner neighbour so there's no slope across the rim.
 * A wave bounces back the same way up (a crest returns as a crest). */
static void apply_neumann_bc(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                             int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    u[0][c] = u[1][c];
    v[0][c] = v[1][c];
    u[rows - 1][c] = u[rows - 2][c];
    v[rows - 1][c] = v[rows - 2][c];
  }
  for (int r = 0; r < rows; r++) {
    u[r][0] = u[r][1];
    v[r][0] = v[r][1];
    u[r][cols - 1] = u[r][cols - 2];
    v[r][cols - 1] = v[r][cols - 2];
  }
}

/* Wrap-around rim: no bouncing at all.  A wave leaving the right edge
 * comes back in on the left, and top wraps to bottom.  We copy from the
 * far INNER cell (cols-2, not cols-1) so the cell just across the seam
 * sees the same neighbours it would on a seamless loop. */
static void apply_periodic_bc(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                              int cols, int rows) {
  for (int c = 0; c < cols; c++) {
    u[0][c] = u[rows - 2][c];
    v[0][c] = v[rows - 2][c];
    u[rows - 1][c] = u[1][c];
    v[rows - 1][c] = v[1][c];
  }
  for (int r = 0; r < rows; r++) {
    u[r][0] = u[r][cols - 2];
    v[r][0] = v[r][cols - 2];
    u[r][cols - 1] = u[r][1];
    v[r][cols - 1] = v[r][1];
  }
}

/* Pick the edge rule the user has selected and run it. */
static void apply_bc(int bc) {
  float(*u)[GRID_MAX_W] = g_grid.u;
  float(*v)[GRID_MAX_W] = g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  switch (bc) {
  case BC_DIRICHLET:
    apply_dirichlet_bc(u, v, cols, rows);
    break;
  case BC_NEUMANN:
    apply_neumann_bc(u, v, cols, rows);
    break;
  case BC_PERIODIC:
    apply_periodic_bc(u, v, cols, rows);
    break;
  }
}

/* How out-of-step a cell is with its four nearest neighbours: add up
 * each neighbour's height and compare to this cell's.  Zero means the
 * cell already matches its surroundings (no push); otherwise the sign
 * tells the wave which way to nudge it back toward its neighbours'
 * average.  This is what spreads a bump outward.  (LeVeque [ref 5].)
 * Inlined so it folds straight into the tight speed loop. */
static inline float laplacian5(const float (*u)[GRID_MAX_W], int r, int c) {
  return u[r - 1][c] + u[r + 1][c] + u[r][c - 1] + u[r][c + 1] - 4.0f * u[r][c];
}

/* Step 1 of the tick: update each inner cell's up/down speed.  Two
 * pushes combine — one pulling it toward its neighbours (the wave
 * spreading) and a gentle drag that bleeds off energy so strikes fade.
 * Doing speed first, off the current heights, is the trick that keeps
 * the sim from slowly gaining or losing energy [ref 7]. */
static void velocity_half_step(float (*u)[GRID_MAX_W], float (*v)[GRID_MAX_W],
                               int cols, int rows, float c2, float damping,
                               float dt) {
  for (int r = 1; r < rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      float lap = laplacian5((const float(*)[GRID_MAX_W])u, r, c);
      v[r][c] += (c2 * lap - damping * v[r][c]) * dt;
    }
  }
}

/* Step 2 of the tick: move each height by the speed we just computed.
 * This has to be its own loop — if we moved heights while still updating
 * speeds, a cell we already moved would feed wrong values into its
 * neighbours' speed calculation.  One extra pass buys correctness. */
static void displacement_half_step(float (*u)[GRID_MAX_W],
                                   const float (*v)[GRID_MAX_W], int cols,
                                   int rows, float dt) {
  for (int r = 1; r < rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      u[r][c] += v[r][c] * dt;
    }
  }
}

/* Move the whole membrane forward by one time slice: nudge the speeds,
 * move the heights, then fix up the edges so they obey the chosen rim
 * rule. */
static void update_wave(float dt, float wave_speed, float damping, int bc) {
  float c2 = wave_speed * wave_speed;
  float(*u)[GRID_MAX_W] = g_grid.u;
  float(*v)[GRID_MAX_W] = g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  velocity_half_step(u, v, cols, rows, c2, damping, dt);
  displacement_half_step(u, (const float(*)[GRID_MAX_W])v, cols, rows, dt);
  apply_bc(bc);
}

/* One sweep that gathers three numbers the HUD wants:
 *   • the tallest crest/trough (used to scale the colours)
 *   • energy of motion — how fast everything is moving
 *   • energy of tension — how stretched/bent the surface is
 * Both energies add up to the total the demo watches decay as a strike
 * fades.  We do it in a single pass so each cell is read once a frame. */
static void scan_peak_and_energy(const float (*u)[GRID_MAX_W],
                                 const float (*v)[GRID_MAX_W], int cols,
                                 int rows, float c2, float *out_max_amp,
                                 double *out_ke, double *out_pe) {
  float mx = 0.0f;
  double ke = 0.0, pe = 0.0;

  for (int r = 1; r < rows - 1; r++) {
    for (int c = 1; c < cols - 1; c++) {
      float uu = u[r][c];
      float vv = v[r][c];
      float a = fabsf(uu);
      if (a > mx)
        mx = a;

      /* energy of motion at this cell — grows with speed squared */
      ke += (double)HALF * (double)(vv * vv);

      /* energy of tension — grows with how steeply the surface tilts here */
      float gx = (u[r][c + 1] - u[r][c - 1]) * CDIFF_FACTOR;
      float gy = (u[r + 1][c] - u[r - 1][c]) * CDIFF_FACTOR;
      pe += (double)HALF * (double)c2 * (double)(gx * gx + gy * gy);
    }
  }
  *out_max_amp = mx;
  *out_ke = ke;
  *out_pe = pe;
}

/* Guess the standing-wave shape by counting how many times the surface
 * crosses zero along the middle row and middle column.  A clean (nx, ny)
 * pattern crosses zero nx times across and ny times down, so the counts
 * read straight off as the shape numbers shown in the HUD.  Floored at 1
 * so a flat membrane never reads (0, 0).  (Rayleigh [ref 1].) */
static void estimate_mode_numbers(const float (*u)[GRID_MAX_W], int cols,
                                  int rows, int *out_nx, int *out_ny) {
  int cr = rows / 2;
  int cc = cols / 2;
  int crossings_x = 0;
  int crossings_y = 0;

  for (int c = 1; c < cols - 1; c++)
    if (u[cr][c - 1] * u[cr][c] < 0.0f)
      crossings_x++;
  for (int r = 1; r < rows - 1; r++)
    if (u[r - 1][cc] * u[r][cc] < 0.0f)
      crossings_y++;

  *out_nx = (crossings_x < 1) ? 1 : crossings_x;
  *out_ny = (crossings_y < 1) ? 1 : crossings_y;
}

/* A health number for the sim: how far a wave travels in one time slice,
 * scaled so 1.0 is the danger line.  Stay under it and the simulation is
 * stable; cross it and the numbers blow up.  The overlay shows it as a
 * green/yellow/red light.  (Courant-Friedrichs-Lewy [ref 4].) */
static float cfl_2d_stability(float wave_speed, float dt_sec) {
  return wave_speed * dt_sec * SQRT2;
}

/* Work out all the numbers the side panel shows, once per tick. */
static void compute_stats(float wave_speed, float dt_sec, float *max_amplitude,
                          float *energy_est, int *mode_nx, int *mode_ny,
                          float *cfl_2d) {
  const float(*ufld)[GRID_MAX_W] = (const float(*)[GRID_MAX_W])g_grid.u;
  const float(*vfld)[GRID_MAX_W] = (const float(*)[GRID_MAX_W])g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;
  float c2 = wave_speed * wave_speed;

  double ke, pe;
  scan_peak_and_energy(ufld, vfld, cols, rows, c2, max_amplitude, &ke, &pe);
  estimate_mode_numbers(ufld, cols, rows, mode_nx, mode_ny);

  *energy_est = (float)((ke + pe) / ((rows - 2) * (cols - 2)));
  *cfl_2d = cfl_2d_stability(wave_speed, dt_sec);
}

/* Resize the grid, flatten it, and set the edges per the chosen rim rule.
 * Used at startup and on every terminal resize. */
static void init_grid(int bc, int cols, int rows) {
  g_grid.cols = cols;
  g_grid.rows = rows;
  grid_zero();
  apply_bc(bc);
}

/* ── §6 excitation ── */

/* Press a smooth round dimple into the surface — a drumstick hit.  The
 * bump is tallest at the centre and fades out with distance.  We ADD to
 * whatever's already there, so two strikes can overlap.  Only heights are
 * touched, not speeds — like pushing the surface down and letting go. */
static void apply_excitation(float cx, float cy, float amp, float radius) {
  float(*u)[GRID_MAX_W] = g_grid.u;
  int cols = g_grid.cols;
  int rows = g_grid.rows;
  float inv_2r2 = 1.0f / (2.0f * radius * radius);
  for (int r = 0; r < rows; r++) {
    float dy = (float)r - cy;
    float dy2 = dy * dy;
    for (int c = 0; c < cols; c++) {
      float dx = (float)c - cx;
      float d2 = dx * dx + dy2;
      u[r][c] += amp * expf(-d2 * inv_2r2);
    }
  }
}

/* How tightly the (nx, ny) standing-wave pattern ripples across the
 * grid — more half-waves across the width or height means tighter
 * ripples.  Feeds the sine shapes used to stamp a clean mode. */
static void mode_wavenumbers(int nx, int ny, int cols, int rows, float *out_kx,
                             float *out_ky) {
  *out_kx = (float)nx * (float)M_PI / (float)(cols - 1);
  *out_ky = (float)ny * (float)M_PI / (float)(rows - 1);
}

/* How fast a given standing-wave shape swings up and down.  Tighter
 * ripples swing faster.  We only use it to pick a tasteful starting
 * speed for a played mode (see preset_resonance).  (Rayleigh [ref 1].) */
static float mode_eigen_frequency(float c, float kx, float ky) {
  return c * sqrtf(kx * kx + ky * ky);
}

/* Paint a clean standing-wave pattern onto the surface, both its shape
 * and its motion.  The shape is two sine ripples multiplied together.
 * Instead of starting it frozen at full height (which would look static
 * for a moment), we start it halfway through its swing — already part
 * way down and moving — so it animates on the very first frame.
 * (Rayleigh [ref 1].) */
static void stamp_standing_wave_state(float (*u)[GRID_MAX_W],
                                      float (*v)[GRID_MAX_W], int cols,
                                      int rows, float kx, float ky, float amp,
                                      float omega) {
  float u_scale = INV_SQRT2 * amp;
  float v_scale = INV_SQRT2 * amp * omega;

  for (int r = 0; r < rows; r++) {
    float sy = sinf(ky * (float)r);
    for (int c = 0; c < cols; c++) {
      float sx = sinf(kx * (float)c);
      float psi = sx * sy;
      u[r][c] = u_scale * psi;
      v[r][c] = v_scale * psi;
    }
  }
}

/* Set the whole surface to one clean (nx, ny) standing-wave shape, caught
 * mid-swing so it's already moving. */
static void preset_resonance(int nx, int ny, float amp) {
  float(*u)[GRID_MAX_W] = g_grid.u;
  float(*v)[GRID_MAX_W] = g_grid.v;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  float kx, ky;
  mode_wavenumbers(nx, ny, cols, rows, &kx, &ky);

  /* Use a fixed reference speed here, not the user's current one — this
   * only picks a nice starting swing.  The real speed takes over from the
   * next tick, so the visible motion still follows the user's setting. */
  float omega = mode_eigen_frequency(RESONANCE_KICK_REF_C, kx, ky);

  stamp_standing_wave_state(u, v, cols, rows, kx, ky, amp, omega);
}

typedef struct Scene Scene; /* defined in §8; presets take it by pointer */

static void preset_apply(Scene *s, int preset_id);

/* ── §7 render ── */

/* Is this cell sitting on a "nodal line" — a still seam where up and down
 * regions meet?  Two things must be true: the cell itself is basically
 * flat, AND it has a raised neighbour on one side and a sunken one on the
 * other.  The second test matters — without it every calm background cell
 * would count.  These seams are the modern echo of Chladni's old trick of
 * sprinkling sand on a vibrating plate. (Chladni [ref 8].) */
static bool cell_is_nodal_line(const float (*u)[GRID_MAX_W], int r, int c,
                               int cols, int rows, float near_zero_thresh) {
  if (fabsf(u[r][c]) >= near_zero_thresh)
    return false;

  bool near_pos = false, near_neg = false;
  if (r > 0) {
    near_pos |= (u[r - 1][c] > near_zero_thresh);
    near_neg |= (u[r - 1][c] < -near_zero_thresh);
  }
  if (r < rows - 1) {
    near_pos |= (u[r + 1][c] > near_zero_thresh);
    near_neg |= (u[r + 1][c] < -near_zero_thresh);
  }
  if (c > 0) {
    near_pos |= (u[r][c - 1] > near_zero_thresh);
    near_neg |= (u[r][c - 1] < -near_zero_thresh);
  }
  if (c < cols - 1) {
    near_pos |= (u[r][c + 1] > near_zero_thresh);
    near_neg |= (u[r][c + 1] < -near_zero_thresh);
  }
  return near_pos && near_neg;
}

/* Mark one nodal-line cell with a dim dot.  Always the same neutral grey
 * whatever theme is active, so the seams don't fight the wave colours. */
static void paint_nodal_cell(WINDOW *w, int r, int c) {
  wattron(w, COLOR_PAIR(CP_NODAL) | A_DIM);
  mvwaddch(w, r, c, '.');
  wattroff(w, COLOR_PAIR(CP_NODAL) | A_DIM);
}

/* Draw one cell coloured by its height.  Pushed up gets a warm colour,
 * pushed down a cool one; how far it's moved picks how bright and how
 * solid the character is.  Cells barely off flat are skipped entirely,
 * left as black background — which is exactly what makes the moving wave
 * fronts stand out. (Ware [ref 9].) */
static void paint_amplitude_cell(WINDOW *w, int r, int c, float u, int theme,
                                 float inv_max) {
  bool positive = (u >= 0.0f);
  float norm = fabsf(u) * inv_max;
  if (norm > 1.0f)
    norm = 1.0f;

  int lvl = lut_index(norm);
  if (lvl == 0)
    return; /* too faint to bother drawing — leave it black */

  attr_t attr = wave_attr(theme, positive, lvl);
  wattron(w, attr);
  mvwaddch(w, r, c, k_ramp[lvl]);
  wattroff(w, attr);
}

/* Draw the whole membrane: each cell shows either a nodal-seam dot (if the
 * overlay is on and it's on a seam) or its height as a warm/cool colour.
 * display_max is the tallest point right now, used to scale the colours;
 * we floor it so a nearly-silent surface doesn't divide by almost zero. */
static void render_membrane(WINDOW *w, int theme, bool show_nodal,
                            float display_max) {
  const float(*u)[GRID_MAX_W] = (const float(*)[GRID_MAX_W])g_grid.u;
  int cols = g_grid.cols;
  int rows = g_grid.rows;

  if (display_max < DISPLAY_MAX_FLOOR)
    display_max = DISPLAY_MAX_FLOOR;
  float inv_max = 1.0f / display_max;
  float near_zero_thresh = NODAL_SIGN_THRESH * display_max;

  for (int r = 0; r < rows; r++) {
    for (int c = 0; c < cols; c++) {
      if (show_nodal &&
          cell_is_nodal_line(u, r, c, cols, rows, near_zero_thresh)) {
        paint_nodal_cell(w, r, c);
        continue;
      }
      paint_amplitude_cell(w, r, c, u[r][c], theme, inv_max);
    }
  }
}

/* Draw the little stats box in the bottom-left: tallest point, leftover
 * energy, guessed wave shape, the stability light, and the current
 * settings. */
static void render_overlay(WINDOW *w, int cols, int rows, float max_amp,
                           float energy, int mode_nx, int mode_ny, float cfl,
                           float wave_speed, float damping, int bc,
                           float sim_time, bool paused, bool show_nodal,
                           int preset_id) {
  int pw = 30; /* panel width in characters */
  int ph = 13; /* panel height (rows) */
  int ox = 1;
  int oy = rows - ph - 1;
  if (oy < 0)
    oy = 0;
  if (ox + pw > cols)
    return;

  /* stability light: green when safe, yellow when close, red when risky */
  int cfl_color;
  const char *cfl_label;
  if (cfl < 0.70f) {
    cfl_color = CP_NEG(0, 5);
    cfl_label = "STABLE  ";
  } else if (cfl < 0.90f) {
    cfl_color = CP_HUD;
    cfl_label = "MARGINAL";
  } else {
    cfl_color = CP_POS(0, 7);
    cfl_label = "UNSTABLE";
  }

  wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
  mvwprintw(w, oy + 0, ox, "+--- MEMBRANE WAVE --------+");
  mvwprintw(w, oy + 1, ox, "| max_amp  %8.4f          |", max_amp);
  mvwprintw(w, oy + 2, ox, "| energy   %8.4f          |", energy);
  mvwprintw(w, oy + 3, ox, "| mode    (%3d, %3d)         |", mode_nx, mode_ny);
  wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);

  /* the stability row, tinted by the light chosen above */
  wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
  mvwprintw(w, oy + 4, ox, "| CFL_2D  ");
  wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
  wattron(w, COLOR_PAIR(cfl_color) | A_BOLD);
  wprintw(w, "%5.3f %-8s", cfl, cfl_label);
  wattroff(w, COLOR_PAIR(cfl_color) | A_BOLD);
  wattron(w, COLOR_PAIR(CP_HUD) | A_DIM);
  wprintw(w, "|");

  mvwprintw(w, oy + 5, ox, "| BC       %-10s        |", k_bc_names[bc]);
  mvwprintw(w, oy + 6, ox, "| speed   %6.1f cells/s    |", wave_speed);
  mvwprintw(w, oy + 7, ox, "| damping %8.4f           |", damping);
  mvwprintw(w, oy + 8, ox, "| sim_t   %8.2f s          |", sim_time);
  mvwprintw(w, oy + 9, ox, "| preset  %-10s       |",
            k_preset_names[preset_id]);
  mvwprintw(w, oy + 10, ox, "| nodal   %-3s  %s            |",
            show_nodal ? "ON " : "OFF", paused ? "PAUSED " : "running");
  mvwprintw(w, oy + 11, ox, "+---------------------------+");
  wattroff(w, COLOR_PAIR(CP_HUD) | A_DIM);
}

/* ── §8 scene ── */

/*
 * Scene — all the loose, changeable state for one run: the physics knobs,
 * the UI flags, the visual choices, and the latest stats for the HUD.  It
 * does NOT hold the height/speed fields or the grid size — those live in
 * the §4 Grid (g_grid).  Splitting them keeps each struct small: Grid is
 * the membrane itself, Scene is everything around it.
 *
 * Scene doesn't set up ncurses; it just steps the physics and draws into a
 * window it's handed.  That keeps the solver easy to run headless and
 * makes resizing a one-liner (scene_resize rebuilds Grid at the new size).
 *
 * Fields are grouped into four blocks, in the order a frame touches them:
 *   (A) physics knobs — wave speed, damping, edge rule
 *   (B) flow & presets — paused, single-step, which strike, which mode
 *   (C) visuals — theme, nodal overlay
 *   (D) stats — the numbers compute_stats fills in each tick for the HUD
 *
 * It's one struct rather than three because a reset is then just
 * memset-to-zero plus a few writes; splitting would risk a resize
 * rebuilding the physics but forgetting to clear the stats.
 */
struct Scene {
  /* (A) physics knobs — read every tick by the solver, changed by the
   * c/C, d/D, n keys.  These don't rewrite the field; they just change
   * the forces the next tick produces. */
  float wave_speed; /* how fast disturbances travel, in cells/sec.  Higher
                     * = faster ripples and faster swinging, but push it
                     * too high for the frame rate and the sim goes
                     * unstable (watch the stability light). */
  float damping;    /* how quickly strikes fade.  0 rings forever; the
                     * default ~0.003 lets a hit fade over about 5 sec. */
  int bc;           /* which edge rule is active: 0 pinned (flips on
                     * bounce), 1 free (no flip), 2 wrap-around.  Cycled
                     * by 'n'. */

  /* (B) flow & preset state — owned by the keyboard handler.  scene_tick
   * checks paused / step_requested to decide whether to advance. */
  bool paused;         /* freeze the physics; HUD shows "PAUSED".  Toggled
                        * by space. */
  bool step_requested; /* one-shot: while paused, run exactly one tick
                        * then clear.  Set by 's' for frame-by-frame. */
  int preset_id;       /* the last canned strike triggered; 'r' replays it
                        * and the HUD shows its name. */
  int mode_idx;        /* where we are in the standing-wave list; 'm'
                        * advances it so each press shows a new shape.
                        * -1 until the first 'm'. */

  /* (C) visuals — read by the renderer, changed by t and l.  No physics
   * touches these. */
  int theme;       /* which colour scheme; cycles on 't'. */
  bool show_nodal; /* draw the dim dots on the still seams so the wave
                    * shape is visible — the modern echo of Chladni's
                    * sand trick [ref 8].  Toggled by 'l'. */

  /* (D) stats — compute_stats fills these at the end of each tick, the
   * HUD reads them every frame.  Caching here saves re-scanning the
   * field just to draw the panel. */
  float max_amplitude;   /* tallest crest/trough right now — also used to
                          * scale the colours so they stay vivid as the
                          * wave fades. */
  float energy_est;      /* leftover energy per cell — drifts down toward
                          * zero as damping eats the strike. */
  int mode_nx, mode_ny;  /* guessed wave shape, from the zero-crossing
                          * counts; exact for a clean standing wave. */
  float cfl_2d;          /* the stability number — green/yellow/red in the
                          * overlay [ref 4]. */
  float simulation_time; /* seconds of sim time since the last reset. */
  float dt_sec;          /* the last tick's time slice, shown in the HUD. */

  /* grid width/height live in §4 Grid, reached via g_grid.cols/.rows. */
};

static void scene_init(Scene *s, int cols, int rows) {
  memset(s, 0, sizeof *s);
  s->wave_speed = WAVE_SPEED_DEFAULT;
  s->damping = DAMPING_DEFAULT;
  s->bc = BC_DIRICHLET;
  s->theme = 0;
  s->show_nodal = true;
  s->mode_idx = -1; /* first 'm' press → k_mode_table[0] */
  init_grid(s->bc, cols, rows);
  preset_apply(s, PRESET_CENTER);
}

static void scene_reset(Scene *s) {
  init_grid(s->bc, g_grid.cols, g_grid.rows);
  s->simulation_time = 0.0f;
  s->max_amplitude = 0.0f;
  s->energy_est = 0.0f;
}

static void scene_resize(Scene *s, int cols, int rows) {
  init_grid(s->bc, cols, rows);
}

/* Advance the sim by one tick: move the wave forward, then refresh the
 * HUD numbers.  While paused (and not single-stepping) it does nothing,
 * so the picture freezes but still keeps drawing. */
static void scene_tick(Scene *s, float dt) {
  if (s->paused && !s->step_requested)
    return;
  s->step_requested = false;

  s->dt_sec = dt;
  s->simulation_time += dt;

  update_wave(dt, s->wave_speed, s->damping, s->bc);

  compute_stats(s->wave_speed, dt, &s->max_amplitude, &s->energy_est,
                &s->mode_nx, &s->mode_ny, &s->cfl_2d);
}

static void scene_draw(const Scene *s, WINDOW *w, float alpha, float dt_sec) {
  (void)alpha;
  (void)dt_sec; /* nothing slides smoothly here, so no between-frame blend */

  render_membrane(w, s->theme, s->show_nodal, s->max_amplitude);

  render_overlay(w, g_grid.cols, g_grid.rows, s->max_amplitude, s->energy_est,
                 s->mode_nx, s->mode_ny, s->cfl_2d, s->wave_speed, s->damping,
                 s->bc, s->simulation_time, s->paused, s->show_nodal,
                 s->preset_id);
}

/* Turn a "fraction across, fraction down" position into actual cell
 * coordinates, so each strike can just say "30% from the left" and not
 * repeat the size arithmetic. */
static void cell_at_fraction(float fx, float fy, float *out_x, float *out_y) {
  *out_x = (float)(g_grid.cols - 1) * fx;
  *out_y = (float)(g_grid.rows - 1) * fy;
}

/* One hit dead centre.  Makes a clean ring that spreads out and bounces
 * off all four walls. */
static void strike_centre_pulse(int bc) {
  float x, y;
  cell_at_fraction(STRIKE_X_CENTRE, STRIKE_Y_CENTRE, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  apply_bc(bc);
}

/* One hit near the left wall.  Being off to the side wakes up lots of
 * overlapping wave shapes at once, giving a busier, lopsided pattern. */
static void strike_left_edge_pulse(int bc) {
  float x, y;
  cell_at_fraction(STRIKE_X_LEFT_EDGE, STRIKE_Y_CENTRE, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  apply_bc(bc);
}

/* Two hits at once, off-centre and not lined up through the middle.  The
 * two spreading rings cross and interfere, building a richer pattern of
 * reinforcing and cancelling ripples. */
static void strike_double_offcentre_pulses(int bc) {
  float x, y;
  cell_at_fraction(STRIKE_X_DOUBLE_L, STRIKE_Y_DOUBLE_L, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  cell_at_fraction(STRIKE_X_DOUBLE_R, STRIKE_Y_DOUBLE_R, &x, &y);
  apply_excitation(x, y, EXCITE_AMP, EXCITE_RADIUS);
  apply_bc(bc);
}

/* Paint the next clean standing-wave shape from the cycle list onto the
 * surface.  See preset_resonance for why it starts mid-swing. */
static void strike_next_standing_wave_mode(const Scene *s, int bc) {
  int idx = s->mode_idx;
  if (idx < 0 || idx >= MODE_TABLE_LEN)
    idx = 0;
  preset_resonance(k_mode_table[idx][0], k_mode_table[idx][1], RESONANCE_AMP);
  apply_bc(bc);
}

/* Run one of the canned strikes by id: clear the surface, remember which
 * preset this was, then hand off to the matching strike helper. */
static void preset_apply(Scene *s, int id) {
  scene_reset(s);
  s->preset_id = id;

  switch (id) {
  case PRESET_CENTER:
    strike_centre_pulse(s->bc);
    break;
  case PRESET_EDGE:
    strike_left_edge_pulse(s->bc);
    break;
  case PRESET_DOUBLE:
    strike_double_offcentre_pulses(s->bc);
    break;
  case PRESET_RESONANCE:
    strike_next_standing_wave_mode(s, s->bc);
    break;
  }
}

/* ── §9 screen — drawing through ncurses ── */

/*
 * Screen — just how big the terminal is right now.  Everything else the
 * drawing needs (the window itself, the colour slots) lives inside
 * ncurses, set up once by color_init, so there's nothing else to keep
 * here.
 *
 * Why wrap two ints in a struct: width and height always belong
 * together, so passing them as one Screen means a resize updates both
 * at once — you can't accidentally change one and forget the other.
 *
 * These two are NOT the same as the grid's cols/rows.  The grid's size
 * is the membrane's size; Screen's size is whatever ncurses says the
 * terminal is.  For this demo they happen to match, and app_do_resize
 * is the one place that keeps them in step on a resize.
 */
typedef struct {
  int cols; /* terminal width  in characters (from getmaxyx) */
  int rows; /* terminal height in characters (from getmaxyx) */
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

static void screen_draw(Screen *s, const Scene *sc, double fps, int sim_fps,
                        float alpha, float dt_sec) {
  erase();
  scene_draw(sc, stdscr, alpha, dt_sec);

  /* Top status line: fps, settings, and paused/running.  We wipe the
   * whole row first so the wave underneath doesn't show through, and
   * draw it bright so it stays readable over any colours. */
  char status[HUD_COLS + 1];
  snprintf(status, sizeof status,
           " membrane  %5.1f fps  sim:%3d Hz  c=%.0f  "
           "g=%.4f  BC=%-10s  %s ",
           fps, sim_fps, sc->wave_speed, sc->damping, k_bc_names[sc->bc],
           sc->paused ? "PAUSED " : "running");
  move(0, 0);
  clrtoeol();
  int hx = s->cols - (int)strlen(status);
  if (hx < 0)
    hx = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvprintw(0, hx, "%s", status);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Bottom key-hints line.  Wipe the row first so the wave doesn't show
   * through, and draw it bright (not dim — dim text disappears against
   * the moving colours). */
  int hint_row = s->rows - 1;
  move(hint_row, 0);
  clrtoeol();
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvprintw(hint_row, 0,
           " q:quit  spc:pause  s:step  r:reset  b:center  e:edge"
           "  f:double  m:mode  c/C:speed  d/D:damp  n:BC"
           "  l:nodal  t:theme  [/]:Hz ");
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §10 app — signals, resize, input, main loop ── */

/*
 * App — everything the running program keeps alive from start to finish:
 * the simulation (Scene), the terminal size (Screen), how many physics
 * steps to run per second, and two little flags the signal handlers use
 * to talk back to the main loop.
 *
 * Why one struct: it gives the program a single home for its state, so
 * main() can grab one pointer and reach everything through it.
 *
 * The sim-steps-per-second knob lives here rather than in Scene because
 * it's about pacing the loop, not about the wave itself.
 *
 * About the two flags: a signal can fire at any instant, even mid-line.
 * The only safe thing a handler may do is flip a simple flag of this
 * exact type (volatile sig_atomic_t) — anything fancier is undefined.
 * So the handlers just set a flag and return; the main loop notices on
 * its next pass and does the real work (quitting, or rebuilding for a
 * new terminal size) at a safe moment.
 *
 * g_app sits at file scope because the signal handlers get no argument
 * and have no other way to reach these flags.
 */
typedef struct {
  Scene scene;   /* the whole simulation        */
  Screen screen; /* current terminal size       */
  int sim_fps;   /* physics steps per second; set apart from the render
                  * frame rate so the wave advances the same amount of
                  * sim time per second no matter how fast frames draw. */
  volatile sig_atomic_t running;     /* set to 0 to make the loop quit on
                                      * its next pass; flipped by the
                                      * quit signal or the q/ESC key. */
  volatile sig_atomic_t need_resize; /* set to 1 when the terminal was
                                      * resized; the loop rebuilds for
                                      * the new size on its next pass. */
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
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/* All keyboard handling in one place: each key either changes a setting,
 * strikes the membrane, or controls the loop.  Returns false only on
 * quit. */
static bool app_handle_key(App *app, int ch) {
  Scene *s = &app->scene;

  switch (ch) {
  /* ── flow control ── */
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    s->paused = !s->paused;
    break;

  case 's':
  case 'S':
    /* Pause, then ask for exactly one tick — lets you walk the wave
     * forward a frame at a time. */
    s->paused = true;
    s->step_requested = true;
    break;

  /* ── manual strikes ── */
  case 'b':
  case 'B':
    /* Hit the centre — works even while paused. */
    apply_excitation((float)(g_grid.cols - 1) * 0.5f,
                     (float)(g_grid.rows - 1) * 0.5f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_bc(s->bc);
    break;

  case 'e':
  case 'E':
    apply_excitation((float)(g_grid.cols - 1) * 0.15f,
                     (float)(g_grid.rows - 1) * 0.5f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_bc(s->bc);
    break;

  case 'f':
  case 'F':
    apply_excitation((float)(g_grid.cols - 1) * 0.30f,
                     (float)(g_grid.rows - 1) * 0.35f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_excitation((float)(g_grid.cols - 1) * 0.70f,
                     (float)(g_grid.rows - 1) * 0.65f, EXCITE_AMP,
                     EXCITE_RADIUS);
    apply_bc(s->bc);
    break;

  case 'm':
  case 'M':
    /* Step to the next standing-wave shape, then paint it.  We move the
     * list pointer first so the very first press already shows a shape,
     * and we un-pause so it actually animates even if you'd been
     * single-stepping. */
    s->mode_idx = (s->mode_idx + 1) % MODE_TABLE_LEN;
    s->paused = false;
    preset_apply(s, PRESET_RESONANCE);
    break;

  /* ── wave speed ── *
   * Faster waves swing every shape quicker — but push too fast for the
   * frame rate and it goes unstable; watch the stability light. */
  case 'c':
    s->wave_speed += WAVE_SPEED_STEP;
    if (s->wave_speed > WAVE_SPEED_MAX)
      s->wave_speed = WAVE_SPEED_MAX;
    break;
  case 'C':
    s->wave_speed -= WAVE_SPEED_STEP;
    if (s->wave_speed < WAVE_SPEED_MIN)
      s->wave_speed = WAVE_SPEED_MIN;
    break;

  /* ── damping ── *
   * More damping makes strikes fade faster; at zero the wave rings on
   * forever, like a perfect drumhead with no friction. */
  case 'd':
    s->damping += DAMPING_STEP;
    if (s->damping > DAMPING_MAX)
      s->damping = DAMPING_MAX;
    break;
  case 'D':
    s->damping -= DAMPING_STEP;
    if (s->damping < DAMPING_MIN)
      s->damping = DAMPING_MIN;
    break;

  /* ── edge rule ── *
   * Cycle how the rim behaves on a bounce.  We replay the current
   * strike on a clean grid so the new rule starts from a tidy state. */
  case 'n':
  case 'N':
    s->bc = (s->bc + 1) % BC_COUNT;
    preset_apply(s, s->preset_id);
    break;

  /* ── reset / presets ── */
  case 'r':
  case 'R':
    preset_apply(s, s->preset_id);
    break;

  case 'p':
    preset_apply(s, (s->preset_id + 1) % PRESET_COUNT);
    break;
  case 'P':
    preset_apply(s, (s->preset_id + PRESET_COUNT - 1) % PRESET_COUNT);
    break;

  /* ── visual ── */
  case 'l':
  case 'L':
    s->show_nodal = !s->show_nodal;
    break;

  case 't':
  case 'T':
    s->theme = (s->theme + 1) % N_THEMES;
    break;

  /* ── simulation rate ── *
   * More steps per second use a smaller time slice: steadier and safer,
   * but more work.  Fewer steps use a bigger slice and can go unstable. */
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

/* The main loop.  It keeps sim time and real time in sync: each pass it
 * runs as many fixed physics steps as the elapsed time calls for, draws
 * a frame, then handles a keypress.  Same shape as framework.c's loop. */
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

    /* ── rebuild on resize ── */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time = clock_ns();
      sim_accum = 0;
    }

    /* ── how much real time passed since last frame ── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS; /* cap it so a long pause can't fast-forward */

    /* ── run physics in fixed steps to match that time ── */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accum += dt;
    while (sim_accum >= tick_ns) {
      scene_tick(&app->scene, dt_sec);
      sim_accum -= tick_ns;
    }

    /* leftover fraction of a step — would blend frames if anything slid
     * smoothly here, but nothing does, so it's unused by the draw */
    float alpha = (float)sim_accum / (float)tick_ns;

    /* ── fps counter, averaged over half a second ── */
    frame_count++;
    fps_accum += dt;
    if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display =
          (double)frame_count / ((double)fps_accum / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accum = 0;
    }

    /* ── hold the frame rate by sleeping before we draw ── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / TARGET_FPS - elapsed);

    /* ── draw the frame ── */
    screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps, alpha,
                dt_sec);
    screen_present();

    /* ── read one keypress, if any ── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
