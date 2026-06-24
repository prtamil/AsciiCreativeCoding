/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * acoustic_wavesolver.c — sound waves rippling through a room, in ASCII.
 *
 * Up to four point sources hum out pressure waves; the waves spread as
 * expanding rings, bounce off walls (or get soaked up by a soft border),
 * and overlap to make standing-wave and interference patterns. Three
 * display modes (v/i/w) and eight colour themes (t/T) just change how the
 * field is drawn — the physics underneath is the same.
 *
 * The method is FDTD: chop the room into a grid and step it forward in
 * tiny time-steps, each cell nudged by its neighbours. See Bilbao,
 * "Numerical Sound Synthesis" (2009) and Taflove & Hagness,
 * "Computational Electrodynamics: the FDTD Method" (2005) for the scheme;
 * Kuttruff, "Room Acoustics" (2017) for the standing-wave behaviour.
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

/*
 * Wave speed, in grid cells crossed per time-step. It's kept well under 1
 * on purpose: if the wave tries to jump more than about one cell per step
 * the simulation goes unstable and blows up to nonsense. DEFAULT is a
 * comfortable speed to watch; MIN/MAX bound what +/- can dial it to; STEP
 * is how much each keypress moves it.
 */
#define WAVE_C_DEFAULT 0.35f
#define WAVE_C_MIN 0.10f
#define WAVE_C_MAX 0.70f
#define WAVE_C_STEP 0.05f

/*
 * Terminal cells are about twice as tall as they are wide. Without
 * correcting for that, round wavefronts would come out squashed into
 * ellipses. We tell the physics that one row of cells covers twice the
 * real distance of one column, so rings end up looking circular.
 */
#define ASPECT_Y 2.0f

/*
 * Safety margin on the stability limit (must stay below 1). The time-step
 * is sized so the wave never moves quite as far as it safely could each
 * step. 0.90 leaves a 10% cushion; going to 1.0 or above lets rounding
 * errors snowball and the whole field explodes to red/blue garbage.
 */
#define CFL 0.90f

/*
 * How wide we want each wave's ripples to be, measured in grid cells.
 * Source pitches are tuned to hit this. Around 10 cells keeps the rings
 * easy to see: much tighter and they blur into grey, much wider and only
 * one ring fits on screen.
 */
#define LAMBDA_CELLS 10.0f

/*
 * How many physics steps to run per drawn frame. Each step is tiny, so we
 * batch a few to make the waves move at a watchable pace. Higher = faster
 * waves but more CPU per frame; 1 = slow motion.
 */
#define STEPS_PER_FRAME 4

/*
 * Hard ceiling on grid size, even on a huge terminal. Work per step grows
 * with cell count, so capping it keeps every frame inside the time budget.
 */
#define GRID_MAX_X 160
#define GRID_MAX_Y 48

/*
 * How many point sources you can have humming at once. Two or more
 * overlapping waves interfere — bright where crests meet, dark nodal lines
 * where a crest meets a trough.
 */
#define MAX_SOURCES 4

/*
 * The absorbing border (the "sponge"). Rather than a single absorbing
 * wall — which would itself bounce waves back — it's a soft strip a few
 * cells deep that fades the wave out gradually. SPONGE_WIDTH is how deep
 * the strip is; SPONGE_DAMP is how much it eats per step at the very edge
 * (10%). The fade ramps up smoothly from nothing at the inner edge so
 * there's no sudden jump for waves to reflect off.
 */
#define SPONGE_WIDTH 8    /* cells from each wall */
#define SPONGE_DAMP 0.10f /* max damping coefficient at wall */

/*
 * Where the four sources sit and how their pitches relate.
 *
 * SRC_NX / SRC_NY: positions as fractions of the room (0..1), so they land
 * in the same relative spot at any terminal size. The layout is lopsided
 * on purpose (no mirror symmetry) to keep the interference lively.
 *
 * SRC_FMUL: each source's pitch as a multiple of the base pitch. The
 * slightly-off ratios make the waves drift in and out of sync ("beating")
 * instead of locking into one repeating pattern.
 */
static const float SRC_NX[MAX_SOURCES] = {0.25f, 0.75f, 0.50f, 0.25f};
static const float SRC_NY[MAX_SOURCES] = {0.50f, 0.50f, 0.25f, 0.75f};
static const float SRC_FMUL[MAX_SOURCES] = {1.00f, 1.33f, 0.75f, 1.60f};

/* Wall behaviour: bounce waves back, or soak them up. */
#define BC_REFLECT 0
#define BC_ABSORB 1

/* The three ways to draw the field. */
#define VIS_PRESSURE 0
#define VIS_INTENSITY 1
#define VIS_WAVEFRONT 2
#define VIS_COUNT 3

/* Colour-pair slots. Pressure runs blue (low) through grey to red (high);
 * intensity is a dim-to-bright ramp; wavefront is six contour bands. */
#define CP_PN2 1 /* pressure: strong negative (deep blue)   */
#define CP_PN1 2
#define CP_PN0 3
#define CP_PZERO 4 /* near zero (dim grey)                    */
#define CP_PP0 5
#define CP_PP1 6
#define CP_PP2 7  /* pressure: strong positive (deep red)    */
#define CP_INT0 8 /* intensity: darkest                      */
#define CP_INT1 9
#define CP_INT2 10
#define CP_INT3 11
#define CP_INT4 12
#define CP_INT5 13
#define CP_INT6 14
#define CP_INT7 15 /* intensity: brightest                    */
#define CP_WF0 16  /* wavefront band 0 (lowest)               */
#define CP_WF1 17  /* wavefront band 1                        */
#define CP_WF2 18  /* wavefront band 2                        */
#define CP_WF3 19  /* wavefront band 3                        */
#define CP_WF4 20  /* wavefront band 4                        */
#define CP_WF5 21  /* wavefront band 5 (highest)              */
#define CP_SRC 22  /* source marker (yellow)                  */
#define CP_HUD 23  /* top HUD status bar (bright yellow)      */
#define CP_HINT 24 /* bottom hint bar (bright cyan)           */

static const char DENS_CHARS[8] = {' ', '.', ':', '+', 'x', 'X', '#', '@'};
static const char *VIS_NAMES[VIS_COUNT] = {"PRESSURE", "INTENSITY",
                                           "WAVEFRONT"};
static const char *BC_NAMES[2] = {"REFLECTING", "ABSORBING "};

/*
 * One colour scheme for the field. Picking a theme only changes how the
 * waves are painted, never the physics; t/T cycle through them.
 *
 *   name        what shows in the HUD.
 *   pressure[7] colours for the signed view, running strong-negative ->
 *               near-zero -> strong-positive. Two-hue themes show the sign
 *               by colour; single-hue ones lean on the character ramp.
 *   intensity[8] dim-to-bright colours for the loudness view.
 *   wavefront[6] six contour-band colours, faintest to brightest.
 *
 * Every colour is chosen from the safe, visible part of the 256-colour
 * range so nothing vanishes against a black background.
 */
typedef struct {
  const char *name;
  short pressure[7];
  short intensity[8];
  short wavefront[6];
} Theme;

static const Theme THEMES[] = {
    {"AURORA",
     {33, 39, 117, 240, 217, 203, 196},
     {24, 28, 34, 70, 76, 118, 154, 231},
     {240, 31, 39, 51, 117, 159}},
    {"MATRIX",
     {28, 34, 70, 240, 70, 118, 154},
     {24, 28, 34, 40, 76, 118, 154, 231},
     {240, 34, 70, 76, 118, 154}},
    {"FIRE",
     {130, 166, 215, 240, 215, 202, 196},
     {52, 88, 124, 160, 196, 202, 214, 231},
     {240, 88, 124, 160, 196, 214}},
    {"OCEAN",
     {24, 31, 87, 240, 117, 45, 51},
     {24, 31, 38, 45, 51, 87, 159, 231},
     {240, 31, 38, 45, 51, 159}},
    {"NEON",
     {51, 87, 159, 240, 219, 213, 201},
     {53, 89, 125, 161, 197, 199, 213, 231},
     {240, 53, 87, 159, 201, 219}},
    {"MONO",
     {252, 248, 244, 240, 244, 248, 252},
     {240, 244, 246, 248, 250, 252, 254, 255},
     {240, 244, 247, 250, 253, 255}},
    {"ICE",
     {39, 51, 159, 240, 195, 159, 51},
     {24, 31, 38, 45, 87, 159, 195, 231},
     {240, 39, 51, 87, 159, 195}},
    {"ECLIPSE",
     {130, 166, 215, 240, 215, 208, 196},
     {240, 88, 124, 160, 196, 208, 214, 231},
     {240, 52, 88, 130, 208, 214}},
};
#define N_THEMES ((int)(sizeof(THEMES) / sizeof(THEMES[0])))

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL

/* ── §2 clock ── */

static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}
static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 color ── */

/* Sets just the theme-driven colours; the HUD and source colours are left
 * alone. Called once at startup and again every time the theme changes. */
static void color_apply_theme(int idx) {
  if (idx < 0 || idx >= N_THEMES)
    idx = 0;
  if (COLORS >= 256) {
    const Theme *t = &THEMES[idx];
    init_pair(CP_PN2, t->pressure[0], COLOR_BLACK);
    init_pair(CP_PN1, t->pressure[1], COLOR_BLACK);
    init_pair(CP_PN0, t->pressure[2], COLOR_BLACK);
    init_pair(CP_PZERO, t->pressure[3], COLOR_BLACK);
    init_pair(CP_PP0, t->pressure[4], COLOR_BLACK);
    init_pair(CP_PP1, t->pressure[5], COLOR_BLACK);
    init_pair(CP_PP2, t->pressure[6], COLOR_BLACK);
    for (int i = 0; i < 8; i++)
      init_pair(CP_INT0 + i, t->intensity[i], COLOR_BLACK);
    for (int i = 0; i < 6; i++)
      init_pair(CP_WF0 + i, t->wavefront[i], COLOR_BLACK);
  } else {
    /* On a plain 8-colour terminal we can't honour the themes, so we pin a
     * fixed blue->grey->red palette; the theme name still cycles in the HUD. */
    init_pair(CP_PN2, COLOR_BLUE, COLOR_BLACK);
    init_pair(CP_PN1, COLOR_BLUE, COLOR_BLACK);
    init_pair(CP_PN0, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_PZERO, COLOR_BLACK, COLOR_BLACK);
    init_pair(CP_PP0, COLOR_MAGENTA, COLOR_BLACK);
    init_pair(CP_PP1, COLOR_RED, COLOR_BLACK);
    init_pair(CP_PP2, COLOR_RED, COLOR_BLACK);
    for (int i = 0; i < 8; i++)
      init_pair(CP_INT0 + i, COLOR_GREEN, COLOR_BLACK);
    init_pair(CP_WF0, COLOR_BLACK, COLOR_BLACK);
    init_pair(CP_WF1, COLOR_BLUE, COLOR_BLACK);
    init_pair(CP_WF2, COLOR_BLUE, COLOR_BLACK);
    init_pair(CP_WF3, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_WF4, COLOR_CYAN, COLOR_BLACK);
    init_pair(CP_WF5, COLOR_WHITE, COLOR_BLACK);
  }
}

static void color_init(int theme_idx) {
  start_color();
  use_default_colors();
  /* Source and HUD colours stay fixed and bright so they read clearly no
   * matter which theme is active. */
  if (COLORS >= 256) {
    init_pair(CP_SRC, 226, COLOR_BLACK); /* bright yellow */
    init_pair(CP_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(CP_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    init_pair(CP_SRC, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(CP_HINT, COLOR_CYAN, COLOR_BLACK);
  }
  color_apply_theme(theme_idx);
}

/* ── §4 grid ── */

/*
 * One point source — a single spot that hums out waves.
 *
 *   nx_f, ny_f : where it sits, as fractions of the room (0..1). Stored
 *                this way so the source lands in the right relative spot
 *                at any grid size; converted to a cell index each step.
 *   freq       : its pitch, in cycles per unit of sim time. Set from the
 *                base pitch times this source's SRC_FMUL.
 *   amp        : how hard it pushes the field each step (loudness).
 *   active     : whether it's currently humming. Toggled by keys 1-4;
 *                an inactive source adds no energy at all.
 */
typedef struct {
  float nx_f, ny_f; /* position as a fraction of the room, 0..1 */
  float freq;       /* pitch, cycles per unit of sim time       */
  float amp;        /* push strength added each step            */
  bool active;
} Source;

/*
 * Wave — everything the simulation needs to know, in one place.
 *
 * The three pressure buffers (p_old, p, p_new) are the heart of it. Each
 * holds the whole room's pressure at one moment in time:
 *   p_old  = how it looked last step
 *   p      = how it looks now
 *   p_new  = what we're computing for next step
 * Stepping forward needs both "now" and "last step", so we keep three and
 * just relabel the pointers each step (rotate, don't copy) — the oldest
 * buffer gets recycled as scratch. That's why it's three and not two.
 *
 * dx is the spacing between cells, sized so the room measures 1 unit
 * across its shorter side; the row spacing is ASPECT_Y times that (see the
 * aspect note in §1). dt is the time-step, resized whenever the wave speed
 * changes so the simulation stays stable.
 *
 * rx2 and ry2 are the per-step "how much each neighbour pulls on a cell"
 * factors for the two directions. They depend only on c, dt and spacing,
 * so we work them out once and reuse them in the tight inner loop instead
 * of recomputing per cell.
 *
 * damp[] is the sponge map: 0 in the open interior, ramping up toward the
 * edges, telling the absorbing border how much to fade each cell. Built
 * once per grid size and reused.
 *
 * The diagnostics drive the HUD readouts: p_max is a slowly-smoothed
 * "loudest cell right now" used to scale the colours (smoothed so one
 * brief spike doesn't wash everything else out); energy is the total push
 * in the field; and the zero-crossing fields estimate the pitch you'd hear
 * at the room's centre by timing how often the pressure there flips sign.
 */
typedef struct {
  int nx, ny; /* grid dimensions                                 */
  float dx;   /* cell spacing across the short side (rows = ASPECT_Y*dx) */
  float dt;   /* time-step, resized with the wave speed          */
  float c;    /* wave speed                                      */
  float time; /* sim time elapsed so far                         */

  float *p;     /* pressure now        [ny*nx]                    */
  float *p_old; /* pressure last step  [ny*nx]                    */
  float *p_new; /* next step / scratch [ny*nx]                    */
  float *damp;  /* sponge fade per cell, 0 inside [ny*nx]         */

  /* neighbour-pull factors, worked out once per speed change */
  float rx2; /* horizontal coupling                             */
  float ry2; /* vertical coupling                               */

  Source srcs[MAX_SOURCES];

  /* HUD diagnostics */
  float p_max;      /* smoothed loudest cell, used to scale colours    */
  float energy;     /* total push in the field (Σ p²)                  */
  float freq_est;   /* pitch guessed from centre-probe sign flips      */
  float probe_prev; /* pressure at the centre probe last step          */
  int zc_count;     /* sign flips counted so far this window           */
  float zc_timer;   /* sim time elapsed in the current window          */

  int vis_mode;
  int bc_mode;
  int theme_idx; /* which THEMES[] entry; cycled with t / T */
  bool paused;
} Wave;

#define IDX(w, x, y) ((y) * (w)->nx + (x))

static int wave_alloc(Wave *w, int nx, int ny) {
  w->nx = nx;
  w->ny = ny;
  int n = nx * ny;
  w->p = calloc(n, sizeof(float));
  w->p_old = calloc(n, sizeof(float));
  w->p_new = calloc(n, sizeof(float));
  w->damp = calloc(n, sizeof(float));
  if (!w->p || !w->p_old || !w->p_new || !w->damp)
    return -1;
  return 0;
}

static void wave_free(Wave *w) {
  free(w->p);
  free(w->p_old);
  free(w->p_new);
  free(w->damp);
}

/* How far into the soft border a cell sits along one axis (0 if it's out
 * in the open interior). Same logic for rows and columns. */
static int sponge_depth_along_axis(int c, int n) {
  if (c < SPONGE_WIDTH)
    return SPONGE_WIDTH - c;
  if (c > n - 1 - SPONGE_WIDTH)
    return c - (n - 1 - SPONGE_WIDTH);
  return 0;
}

/* How much a cell at this depth should fade. It eases in (depth squared,
 * not straight-line) so there's no abrupt edge for waves to bounce off. */
static float sponge_damping_at_depth(int depth) {
  if (depth <= 0)
    return 0.0f;
  float t = (float)depth / (float)SPONGE_WIDTH;
  return SPONGE_DAMP * t * t;
}

static void build_sponge(Wave *w) {
  for (int y = 0; y < w->ny; y++) {
    for (int x = 0; x < w->nx; x++) {
      int ex = sponge_depth_along_axis(x, w->nx);
      int ey = sponge_depth_along_axis(y, w->ny);

      /* Use whichever direction puts the cell deeper in the border, so
       * corners (deep in both) fade hardest. */
      int e = ex;
      if (ey > e)
        e = ey;

      w->damp[IDX(w, x, y)] = sponge_damping_at_depth(e);
    }
  }
}

/* Picks the largest time-step that still keeps the simulation stable for
 * the current wave speed, then caches the neighbour-pull factors. Call this
 * whenever the speed changes. */
static void wave_recompute_dt(Wave *w) {
  float r = CFL / (w->c * sqrtf(1.0f + 1.0f / (ASPECT_Y * ASPECT_Y)));
  w->dt = r * w->dx;
  float rx = w->c * w->dt / w->dx;
  float ry = w->c * w->dt / (ASPECT_Y * w->dx);
  w->rx2 = rx * rx;
  w->ry2 = ry * ry;
}

static void wave_init(Wave *w, int nx, int ny) {
  w->c = WAVE_C_DEFAULT;
  /* Size cells so the room's shorter side measures 1 unit across. */
  int min_dim = nx;
  if (ny < min_dim)
    min_dim = ny;
  w->dx = 1.0f / (float)(min_dim - 1);
  w->time = 0.0f;
  w->p_max = 1e-6f;
  w->energy = 0.0f;
  w->freq_est = 0.0f;
  w->probe_prev = 0.0f;
  w->zc_count = 0;
  w->zc_timer = 0.0f;
  w->vis_mode = VIS_PRESSURE;
  w->bc_mode = BC_ABSORB; /* open up first so you see clean rings */
  w->theme_idx = 0;       /* AURORA */
  w->paused = false;

  wave_recompute_dt(w);

  memset(w->p, 0, nx * ny * sizeof(float));
  memset(w->p_old, 0, nx * ny * sizeof(float));
  memset(w->p_new, 0, nx * ny * sizeof(float));
  build_sponge(w);

  /* Pick a base pitch that makes each ring about LAMBDA_CELLS cells wide,
   * so the rings look the same size no matter the terminal size or speed.
   * Each source then scales this by its own SRC_FMUL. */
  float base_freq = w->c / (LAMBDA_CELLS * w->dx);
  for (int i = 0; i < MAX_SOURCES; i++) {
    w->srcs[i].nx_f = SRC_NX[i];
    w->srcs[i].ny_f = SRC_NY[i];
    w->srcs[i].freq = base_freq * SRC_FMUL[i];
    w->srcs[i].amp = 1.0f;
    w->srcs[i].active = (i == 0 || i == 1); /* sources 1+2: interference */
  }
}

/* ── §5 update_wave() — one time-step of the wave physics ── */

/*
 * The heart of the simulation: figure out each cell's next pressure from
 * its own value, the value it had last step, and a small nudge from its
 * four neighbours. Do that everywhere, add the sources' push, then roll
 * the buffers forward. The references in the file header derive the exact
 * scheme; the four helpers below carry it out one step at a time.
 *
 * Each cell looks at where it was last step and where its neighbours are
 * now, and leans toward them — that pulling-toward-the-neighbours is what
 * makes a bump spread outward as a ring. rx2/ry2 set how strong that pull
 * is in each direction.
 */
static void apply_fdtd_stencil(Wave *w) {
  int nx = w->nx, ny = w->ny;
  float rx2 = w->rx2, ry2 = w->ry2;

  for (int y = 1; y < ny - 1; y++) {
    for (int x = 1; x < nx - 1; x++) {
      float pc = w->p[IDX(w, x, y)];
      float pe = w->p[IDX(w, x + 1, y)];
      float pw_ = w->p[IDX(w, x - 1, y)];
      float pn = w->p[IDX(w, x, y + 1)];
      float ps = w->p[IDX(w, x, y - 1)];
      float po = w->p_old[IDX(w, x, y)];
      w->p_new[IDX(w, x, y)] = 2.0f * pc - po + rx2 * (pe + pw_ - 2.0f * pc) +
                               ry2 * (pn + ps - 2.0f * pc);
    }
  }
}

/*
 * Each active source gives its cell a gentle push that swings up and down
 * like a speaker cone, so waves spread out evenly in every direction. We
 * add the push to whatever the physics already computed rather than
 * overwriting the cell — overwriting would make the source act like a
 * little wall and dent the rings. Sources are kept one cell inside the
 * edge so they always have neighbours to push against.
 */
static void inject_monopole_sources(Wave *w) {
  int nx = w->nx, ny = w->ny;
  for (int i = 0; i < MAX_SOURCES; i++) {
    if (!w->srcs[i].active)
      continue;
    int sx = (int)(w->srcs[i].nx_f * (float)(nx - 1) + 0.5f);
    int sy = (int)(w->srcs[i].ny_f * (float)(ny - 1) + 0.5f);
    if (sx < 1)
      sx = 1;
    if (sx > nx - 2)
      sx = nx - 2;
    if (sy < 1)
      sy = 1;
    if (sy > ny - 2)
      sy = ny - 2;
    float drive =
        w->srcs[i].amp * sinf(2.0f * (float)M_PI * w->srcs[i].freq * w->time);
    w->p_new[IDX(w, sx, sy)] += drive;
  }
}

/*
 * Slide the three buffers along one notch: "now" becomes "last step",
 * the freshly computed buffer becomes "now", and the buffer we no longer
 * need gets reused as next step's scratch. We just swap the labels
 * (pointers) instead of copying any data, so this is instant no matter
 * how big the grid is.
 */
static void rotate_pressure_levels(Wave *w) {
  float *recycled = w->p_old;
  w->p_old = w->p;
  w->p = w->p_new;
  w->p_new = recycled;
}

/* One full step, in plain order: work out next pressure, add the
 * sources' push, slide the buffers, advance the clock. */
static void update_wave(Wave *w) {
  apply_fdtd_stencil(w);
  inject_monopole_sources(w);
  rotate_pressure_levels(w);
  w->time += w->dt;
}

/* ── §6 apply_boundary() — what happens at the walls, plus HUD readouts ── */

/*
 * Two things happen here every step: deal with the room's edges (bounce
 * waves back, or soak them up), and take a few measurements for the HUD.
 * The wall choice is what makes the difference between watching rings
 * fly out forever and watching them echo into a standing-wave pattern.
 */

/*
 * Hard wall: force the pressure to zero all along the four edges. A wall
 * that can't move flips an incoming wave over as it bounces it back — a
 * pushed-in crest comes back as a pulled-out trough. Bounce it off all
 * four walls enough times and the room settles into a fixed standing
 * pattern instead of expanding rings.
 *
 * We zero both the "now" and the "last step" buffers because the next
 * physics step reads both; leaving the old one non-zero at the edge
 * would leak that energy straight back inside.
 */
static void apply_dirichlet_walls(Wave *w) {
  int nx = w->nx, ny = w->ny;
  /* Top and bottom rows. */
  for (int x = 0; x < nx; x++) {
    w->p[IDX(w, x, 0)] = 0.0f;
    w->p_old[IDX(w, x, 0)] = 0.0f;
    w->p[IDX(w, x, ny - 1)] = 0.0f;
    w->p_old[IDX(w, x, ny - 1)] = 0.0f;
  }
  /* Left and right columns. */
  for (int y = 0; y < ny; y++) {
    w->p[IDX(w, 0, y)] = 0.0f;
    w->p_old[IDX(w, 0, y)] = 0.0f;
    w->p[IDX(w, nx - 1, y)] = 0.0f;
    w->p_old[IDX(w, nx - 1, y)] = 0.0f;
  }
}

/*
 * Soft border: instead of bouncing, fade the wave out as it nears the
 * edge so it just quietly disappears (like the foam walls of a recording
 * booth). Each cell in the border strip is shrunk a little, gently in the
 * interior and harder right at the wall, using the fade map built earlier.
 * Both buffers are faded for the same reason the hard wall zeroes both.
 */
static void apply_sponge_damping(Wave *w) {
  int n = w->nx * w->ny;
  for (int i = 0; i < n; i++) {
    float d = w->damp[i];
    if (d <= 0.0f)
      continue;
    float s = 1.0f - d;
    w->p[i] *= s;
    w->p_old[i] *= s;
  }
}

/*
 * One pass over the grid to measure two things for the HUD. p_max is a
 * slowly-smoothed "loudest cell right now" used to scale the colours —
 * we ease it along instead of snapping to each frame's peak so one brief
 * spike doesn't wash the rest of the picture out. energy is the total
 * loudness in the room: it climbs as a reflecting room fills up and fades
 * away once the sponge is soaking everything up.
 */
static void compute_field_norms(Wave *w) {
  int n = w->nx * w->ny;
  float pmax = 1e-6f;
  float energy = 0.0f;
  for (int i = 0; i < n; i++) {
    float ap = fabsf(w->p[i]);
    if (ap > pmax)
      pmax = ap;
    energy += w->p[i] * w->p[i];
  }
  w->p_max = w->p_max * 0.97f + pmax * 0.03f;
  if (w->p_max < 1e-6f)
    w->p_max = 1e-6f;
  w->energy = energy;
}

/*
 * Guess the pitch you'd "hear" at the centre of the room by watching one
 * cell there and counting how often its pressure flips from push to pull.
 * A wave flips sign twice per full cycle, so the flips-per-second tells us
 * the frequency. We tally over a short window and report it on the HUD. If
 * several sources are humming, this reflects whichever one dominates at the
 * centre (usually source 1, which sits closest).
 */
static void track_zero_crossings(Wave *w) {
  int px = w->nx / 2;
  int py = w->ny / 2;
  float pc = w->p[IDX(w, px, py)];

  bool crossed_neg_to_pos = (w->probe_prev < 0.0f && pc >= 0.0f);
  bool crossed_pos_to_neg = (w->probe_prev >= 0.0f && pc < 0.0f);
  if (crossed_neg_to_pos || crossed_pos_to_neg)
    w->zc_count++;

  w->probe_prev = pc;
  w->zc_timer += w->dt;

  /* Recompute estimate every 0.5 sim-time-units; reset accumulators. */
  if (w->zc_timer >= 0.5f) {
    w->freq_est = (float)w->zc_count / (2.0f * w->zc_timer);
    w->zc_count = 0;
    w->zc_timer = 0.0f;
  }
}

/* Handle the walls (bounce or soak, depending on the mode), then take
 * the measurements the HUD shows. */
static void apply_boundary(Wave *w) {
  if (w->bc_mode == BC_REFLECT)
    apply_dirichlet_walls(w);
  else
    apply_sponge_damping(w);

  compute_field_norms(w);
  track_zero_crossings(w);
}

/* ── §7 render_field() — three ways to paint the waves ── */

/*
 * The three render functions below each draw the same pressure field a
 * different way, picked with v/i/w. Two details they all share:
 *
 * - They only change ncurses' colour when the next cell actually needs a
 *   different one. Setting the colour is the slow part, and big stretches
 *   of the field share a colour, so skipping the repeats keeps drawing fast.
 *
 * - The screen counts rows from the top down, but the physics grid counts
 *   from the bottom up, so each draw flips the row (gy = ny-1-sy). Without
 *   the flip the whole picture would be upside-down.
 */

/* PRESSURE: show push and pull directly — red where the air is squeezed
 * (a crest), blue where it's stretched thin (a trough), faint grey where
 * it's calm. Best for seeing the actual shape of the waves and how two
 * wavefronts line up. */
static void render_pressure(const Wave *w, int cols, int rows) {
  int nx = w->nx, ny = w->ny;
  float inv = 1.0f / w->p_max;
  chtype cur_attr = A_NORMAL;
  attrset(cur_attr);

  for (int sy = 0; sy < ny && sy < rows; sy++) {
    int gy = ny - 1 - sy;
    for (int x = 0; x < nx && x < cols; x++) {
      float pn = w->p[IDX(w, x, gy)] * inv;
      if (pn > 1.0f)
        pn = 1.0f;
      if (pn < -1.0f)
        pn = -1.0f;

      /* Split the -1..+1 range into seven bands. The cutoffs match on
       * the push and pull sides, and the calm middle band is kept wide
       * so quiet areas fade into the background. */
      int cp;
      char ch;
      attr_t at;
      if (pn < -0.65f) {
        cp = CP_PN2;
        ch = '#';
        at = A_BOLD;
      } else if (pn < -0.30f) {
        cp = CP_PN1;
        ch = 'x';
        at = A_NORMAL;
      } else if (pn < -0.15f) {
        cp = CP_PN0;
        ch = ':';
        at = A_NORMAL;
      } else if (pn < 0.15f) {
        cp = CP_PZERO;
        ch = ' ';
        at = A_DIM;
      } else if (pn < 0.30f) {
        cp = CP_PP0;
        ch = ':';
        at = A_NORMAL;
      } else if (pn < 0.65f) {
        cp = CP_PP1;
        ch = 'x';
        at = A_NORMAL;
      } else {
        cp = CP_PP2;
        ch = '#';
        at = A_BOLD;
      }

      chtype a = (chtype)COLOR_PAIR(cp) | at;
      if (a != cur_attr) {
        attrset(a);
        cur_attr = a;
      }
      mvaddch(sy + 1, x, (chtype)ch);
    }
  }
  attrset(A_NORMAL);
}

/* INTENSITY: show loudness only, ignoring push-vs-pull. Dark where the
 * waves cancel out and it's quiet, bright where they pile up and it's
 * loud. The fixed quiet and loud spots of a standing wave stand out best
 * in this view. */
static void render_intensity(const Wave *w, int cols, int rows) {
  int nx = w->nx, ny = w->ny;
  float inv = 7.0f / w->p_max;
  chtype cur_attr = A_NORMAL;
  attrset(cur_attr);

  for (int sy = 0; sy < ny && sy < rows; sy++) {
    int gy = ny - 1 - sy;
    for (int x = 0; x < nx && x < cols; x++) {
      float ap = fabsf(w->p[IDX(w, x, gy)]) * inv;
      int lv = (int)ap;
      if (lv < 0)
        lv = 0;
      if (lv > 7)
        lv = 7;
      attr_t at = A_NORMAL;
      if (lv >= 5)
        at = A_BOLD;
      chtype a = (chtype)COLOR_PAIR(CP_INT0 + lv) | at;
      if (a != cur_attr) {
        attrset(a);
        cur_attr = a;
      }
      mvaddch(sy + 1, x, (chtype)DENS_CHARS[lv]);
    }
  }
  attrset(A_NORMAL);
}

/*
 * WAVEFRONT: sort loudness into a handful of bands, each a different
 * character and brightness. The sharp jumps between neighbouring bands
 * read as crisp rings that travel outward with the wave, so you can
 * watch it move. With several sources going, their rings cross and you
 * see the bright/dark interference pattern where they meet.
 */
#define N_WF_BANDS 6

static void render_wavefront(const Wave *w, int cols, int rows) {
  int nx = w->nx, ny = w->ny;
  float inv = (float)N_WF_BANDS / w->p_max;

  /* The character, colour, and brightness for each band. Each band
   * already has its own colour, so the dim/bold mix is just extra
   * contrast on top. */
  static const char WF_CH[N_WF_BANDS] = {' ', '.', 'o', 'O', '#', '@'};
  static const int WF_CP[N_WF_BANDS] = {CP_WF0, CP_WF1, CP_WF2,
                                        CP_WF3, CP_WF4, CP_WF5};
  static const attr_t WF_AT[N_WF_BANDS] = {A_DIM,    A_NORMAL, A_NORMAL,
                                           A_NORMAL, A_BOLD,   A_BOLD};

  chtype cur_attr = A_NORMAL;
  attrset(cur_attr);

  for (int sy = 0; sy < ny && sy < rows; sy++) {
    int gy = ny - 1 - sy;
    for (int x = 0; x < nx && x < cols; x++) {
      float ap = fabsf(w->p[IDX(w, x, gy)]) * inv;
      int b = (int)ap;
      if (b < 0)
        b = 0;
      if (b >= N_WF_BANDS)
        b = N_WF_BANDS - 1;
      chtype a = (chtype)COLOR_PAIR(WF_CP[b]) | WF_AT[b];
      if (a != cur_attr) {
        attrset(a);
        cur_attr = a;
      }
      mvaddch(sy + 1, x, (chtype)WF_CH[b]);
    }
  }
  attrset(A_NORMAL);
}

static void render_field(const Wave *w, int cols, int rows) {
  switch (w->vis_mode) {
  case VIS_PRESSURE:
    render_pressure(w, cols, rows);
    break;
  case VIS_INTENSITY:
    render_intensity(w, cols, rows);
    break;
  case VIS_WAVEFRONT:
    render_wavefront(w, cols, rows);
    break;
  }

  /* Mark each active source with its number, and let it pulse in time
   * with its own push so you can see it's the thing making the waves:
   * bright on the push, faint on the pull. */
  for (int i = 0; i < MAX_SOURCES; i++) {
    if (!w->srcs[i].active)
      continue;
    int sx = (int)(w->srcs[i].nx_f * (float)(w->nx - 1) + 0.5f);
    int sg = (int)(w->srcs[i].ny_f * (float)(w->ny - 1) + 0.5f);
    int ss = (w->ny - 1 - sg) + 1; /* screen row (offset by HUD row) */
    if (sx >= 0 && sx < cols && ss >= 1 && ss <= rows) {
      float ph = sinf(2.0f * (float)M_PI * w->srcs[i].freq * w->time);
      attr_t at = A_NORMAL;
      if (ph > 0.5f)
        at = A_BOLD; /* positive peak */
      else if (ph < -0.5f)
        at = A_DIM; /* negative peak */
      attron(COLOR_PAIR(CP_SRC) | at);
      mvaddch(ss, sx, (chtype)('1' + i));
      attroff(COLOR_PAIR(CP_SRC) | at);
    }
  }
}

/* ── §8 render_overlay() — the HUD ── */

/*
 * Draws the two info bars that frame the animation. The top bar (right
 * side, bright yellow) shows the current mode and theme, the wave speed,
 * how much energy is in the room, the pitch we measured at the centre,
 * which sources are on, the wall mode, and the frame rate. The bottom
 * bar (bright cyan) lists every key you can press. If paused, a red
 * PAUSED sits in the middle.
 */
static void render_overlay(const Wave *w, int cols, int rows, double fps) {
  /* Build compact source-list string e.g. "1,2" or "-" if none. */
  char src_str[16] = {0};
  int si = 0;
  for (int i = 0; i < MAX_SOURCES; i++) {
    if (w->srcs[i].active) {
      if (si > 0)
        src_str[si++] = ',';
      src_str[si++] = '1' + i;
    }
  }
  if (si == 0) {
    src_str[0] = '-';
    src_str[1] = '\0';
  } else {
    src_str[si] = '\0';
  }

  /* Top status — right-aligned. Theme name first so it's eye-catching. */
  char top[200];
  snprintf(top, sizeof top,
           " [%s] %s  c=%.2f  E=%.1e  f=%.2fHz  bc:%s  srcs:%s  %.0ffps ",
           VIS_NAMES[w->vis_mode], THEMES[w->theme_idx].name, w->c,
           (double)w->energy, (double)w->freq_est, BC_NAMES[w->bc_mode],
           src_str, fps);
  int top_len = (int)strlen(top);
  int top_col = cols - top_len;
  if (top_col < 0)
    top_col = 0;
  attron(COLOR_PAIR(CP_HUD) | A_BOLD);
  mvaddnstr(0, top_col, top, cols);
  attroff(COLOR_PAIR(CP_HUD) | A_BOLD);

  /* Bottom hint — left-aligned. */
  const char *hint = " q:quit  spc:pause  v/i/w:mode  1-4:src  b:BC"
                     "  +/-:speed  p:impulse  r:reset  t/T:theme ";
  attron(COLOR_PAIR(CP_HINT) | A_BOLD);
  mvaddnstr(rows - 1, 0, hint, cols);
  attroff(COLOR_PAIR(CP_HINT) | A_BOLD);

  if (w->paused) {
    attron(COLOR_PAIR(CP_PP2) | A_BOLD);
    mvprintw(rows / 2, cols / 2 - 4, " PAUSED ");
    attroff(COLOR_PAIR(CP_PP2) | A_BOLD);
  }
}

/* ── §9 scene ── */

/* Ties the whole simulation together. Just the wave for now, but the
 * wrapper keeps the app loop from poking at Wave internals directly and
 * leaves room to add more to a "scene" later. */
typedef struct {
  Wave wave;
} Scene;

static void scene_init(Scene *sc, int cols, int rows) {
  Wave *w = &sc->wave;
  /* Clamp grid to GRID_MAX_X / GRID_MAX_Y; leave 2 rows for HUD bars. */
  int nx = cols;
  if (nx > GRID_MAX_X)
    nx = GRID_MAX_X;
  int ny = rows - 2;
  if (ny > GRID_MAX_Y)
    ny = GRID_MAX_Y;
  if (ny < 4)
    ny = 4;
  if (wave_alloc(w, nx, ny) != 0)
    return;
  wave_init(w, nx, ny);
}

static void scene_free(Scene *sc) { wave_free(&sc->wave); }

static void scene_resize(Scene *sc, int cols, int rows) {
  Wave *w = &sc->wave;
  int vm = w->vis_mode, bc = w->bc_mode, th = w->theme_idx;
  Source saved[MAX_SOURCES];
  memcpy(saved, w->srcs, sizeof saved);

  wave_free(w);
  memset(w, 0, sizeof *w);

  int nx = cols;
  if (nx > GRID_MAX_X)
    nx = GRID_MAX_X;
  int ny = rows - 2;
  if (ny > GRID_MAX_Y)
    ny = GRID_MAX_Y;
  if (ny < 4)
    ny = 4;
  if (wave_alloc(w, nx, ny) != 0)
    return;
  wave_init(w, nx, ny);
  w->vis_mode = vm;
  w->bc_mode = bc;
  w->theme_idx = th;
  /* Restore active flags (positions/freqs recomputed for new grid size) */
  for (int i = 0; i < MAX_SOURCES; i++)
    w->srcs[i].active = saved[i].active;
}

static void scene_tick(Scene *sc) {
  Wave *w = &sc->wave;
  if (w->paused || !w->p)
    return;
  update_wave(w);
  apply_boundary(w);
}

/*
 * Draws a thin border around the room so you can see where the walls are.
 * It sits right on the edge cells, which are always near-silent anyway
 * (the walls pin or fade them), so the frame covers nothing important.
 * Dim yellow keeps it looking like a border, not part of the waves.
 */
static void render_room_frame(const Wave *w) {
  int nx = w->nx, ny = w->ny;
  int top_row = 1;  /* one row below the HUD     */
  int bot_row = ny; /* last screen row of physics */
  int left_col = 0;
  int right_col = nx - 1;

  attron(COLOR_PAIR(CP_HUD) | A_DIM);
  for (int x = left_col; x <= right_col; x++) {
    chtype ch = '-';
    if (x == left_col || x == right_col)
      ch = '+';
    mvaddch(top_row, x, ch);
    mvaddch(bot_row, x, ch);
  }
  for (int y = top_row + 1; y < bot_row; y++) {
    mvaddch(y, left_col, '|');
    mvaddch(y, right_col, '|');
  }
  attroff(COLOR_PAIR(CP_HUD) | A_DIM);
}

static void scene_draw(const Scene *sc, int cols, int rows, double fps) {
  erase();
  render_field(&sc->wave, cols, rows - 2);
  render_room_frame(&sc->wave);
  render_overlay(&sc->wave, cols, rows, fps);
}

/* ── §10 screen ── */

/* Current terminal size in characters, refreshed on startup and resize. */
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
  color_init(0); /* default theme; replaced per Wave state on t/T */
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

/* ── §11 app ── */

/*
 * The whole program's state in one place. running and need_resize are
 * flipped from inside signal handlers, so they're marked volatile
 * sig_atomic_t — the one type the standard promises is safe to touch
 * there. The main loop reads them each pass to know when to quit or
 * rebuild for a new terminal size.
 */
typedef struct {
  Scene scene;
  Screen screen;
  volatile sig_atomic_t running;
  volatile sig_atomic_t need_resize;
} App;

static App g_app;
static void on_exit(int s) {
  (void)s;
  g_app.running = 0;
}
static void on_resize(int s) {
  (void)s;
  g_app.need_resize = 1;
}
static void cleanup(void) { endwin(); }

static bool handle_key(App *app, int ch) {
  Wave *w = &app->scene.wave;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    w->paused = !w->paused;
    break;
  case 'v':
  case 'V':
    w->vis_mode = VIS_PRESSURE;
    break;
  case 'i':
  case 'I':
    w->vis_mode = VIS_INTENSITY;
    break;
  case 'w':
  case 'W':
    w->vis_mode = VIS_WAVEFRONT;
    break;
  case '1':
    w->srcs[0].active = !w->srcs[0].active;
    break;
  case '2':
    w->srcs[1].active = !w->srcs[1].active;
    break;
  case '3':
    w->srcs[2].active = !w->srcs[2].active;
    break;
  case '4':
    w->srcs[3].active = !w->srcs[3].active;
    break;
  case 'b':
  case 'B':
    w->bc_mode = (w->bc_mode + 1) % 2;
    break;
  case '+':
  case '=':
    w->c += WAVE_C_STEP;
    if (w->c > WAVE_C_MAX)
      w->c = WAVE_C_MAX;
    wave_recompute_dt(w);
    /* Faster waves need higher pitches to keep the rings the same width. */
    {
      float bf = w->c / (LAMBDA_CELLS * w->dx);
      for (int i = 0; i < MAX_SOURCES; i++)
        w->srcs[i].freq = bf * SRC_FMUL[i];
    }
    break;
  case '-':
    w->c -= WAVE_C_STEP;
    if (w->c < WAVE_C_MIN)
      w->c = WAVE_C_MIN;
    wave_recompute_dt(w);
    {
      float bf = w->c / (LAMBDA_CELLS * w->dx);
      for (int i = 0; i < MAX_SOURCES; i++)
        w->srcs[i].freq = bf * SRC_FMUL[i];
    }
    break;
  case 'p':
  case 'P': {
    /* Drop a soft round "ping" in the middle — a single tap that kicks
     * the whole room ringing at once, fading out from the centre. */
    int cx = w->nx / 2, cy = w->ny / 2;
    int R = (int)(LAMBDA_CELLS * 0.5f);
    if (R < 2)
      R = 2;
    for (int y = cy - R; y <= cy + R; y++) {
      if (y < 1 || y >= w->ny - 1)
        continue;
      for (int x = cx - R; x <= cx + R; x++) {
        if (x < 1 || x >= w->nx - 1)
          continue;
        float ddx = (float)(x - cx), ddy = (float)(y - cy);
        float r2 = (ddx * ddx + ddy * ddy) / (float)(R * R);
        w->p[IDX(w, x, y)] += 2.5f * expf(-r2 * 4.0f);
      }
    }
    break;
  }
  case 'r':
  case 'R':
    scene_resize(&app->scene, app->screen.cols, app->screen.rows);
    break;
  case 't':
    w->theme_idx = (w->theme_idx + 1) % N_THEMES;
    color_apply_theme(w->theme_idx);
    break;
  case 'T':
    w->theme_idx = (w->theme_idx + N_THEMES - 1) % N_THEMES;
    color_apply_theme(w->theme_idx);
    break;
  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

  int64_t frame_time = clock_ns();
  int64_t fps_accum = 0;
  int fps_count = 0;
  double fps_disp = 0.0;

  while (app->running) {

    /* ── resize ──────────────────────────────────── */
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      frame_time = clock_ns();
    }

    /* ── wall-clock dt ───────────────────────────── */
    int64_t now = clock_ns();
    int64_t dt = now - frame_time;
    frame_time = now;
    if (dt > 200 * NS_PER_MS)
      dt = 200 * NS_PER_MS;

    /* ── physics steps ───────────────────────────── */
    for (int s = 0; s < STEPS_PER_FRAME; s++)
      scene_tick(&app->scene);

    /* ── fps tracking ────────────────────────────── */
    fps_count++;
    fps_accum += dt;
    if (fps_accum >= 500 * NS_PER_MS) {
      fps_disp = (double)fps_count / ((double)fps_accum / (double)NS_PER_SEC);
      fps_count = 0;
      fps_accum = 0;
    }

    /* ── sleep to target 30 fps ──────────────────── */
    int64_t elapsed = clock_ns() - frame_time + dt;
    clock_sleep_ns(NS_PER_SEC / 30 - elapsed);

    /* ── render ──────────────────────────────────── */
    scene_draw(&app->scene, app->screen.cols, app->screen.rows, fps_disp);
    wnoutrefresh(stdscr);
    doupdate();

    /* ── input ───────────────────────────────────── */
    int key;
    while ((key = getch()) != ERR)
      if (!handle_key(app, key)) {
        app->running = 0;
        break;
      }
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
