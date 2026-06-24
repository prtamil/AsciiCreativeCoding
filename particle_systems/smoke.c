/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * smoke.c — ASCII smoke in the terminal, five ways.
 *
 * One shared grid of "how much smoke is here" (0 = clear, 1 = thick).
 * Five interchangeable methods fill that grid each tick, then one shared
 * renderer turns the numbers into soft round glyphs.  Press 'a' to cycle:
 *   0 particle — fly little puffs upward and dab them onto the grid
 *   1 vortex   — stir with a few whirlpools (you see curls and swirls)
 *   2 curl     — stir with smooth random flow (no obvious centres)
 *   3 buoyancy — the smoke lifts itself because thick = hot = rises
 *   4 breeze   — a swaying side-wind bends the column like a curtain
 *
 * Methods 1..4 share one trick (semi-Lagrangian advection): they only
 * differ in how they compute the wind at each cell — everything else is
 * shared in §4.  Sister file fire.c uses the same dithered renderer.
 *
 * References (the ideas behind the code, where the code can't tell you):
 *   Reeves (1983), "Particle Systems", ACM TOG 2(2):91-108 — method 0.
 *   Stam (1999), "Stable Fluids", SIGGRAPH '99:121-128 — methods 1..4.
 *   Floyd & Steinberg (1976), "An Adaptive Algorithm for Spatial
 *     Greyscale", Proc. SID 17(2):75-77 — the dithering in scene_draw.
 *   Bridson (2015), "Fluid Simulation for Computer Graphics", 2nd ed,
 *     ch.3 (advection) and §2.3 (point vortices).
 *   Akenine-Moller et al. (2018), "Real-Time Rendering", 4th ed,
 *     §5.6 (gamma) and §13.7 (soft particle splatting).
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

/* ── §1 presets — every tunable number, grouped by what it controls ── */

/* ── loop / display ── */
enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,

  HUD_COLS = 64,
  FPS_UPDATE_MS = 500,

  N_ALGOS = 5,
  MAX_PARTS = 400, /* how many smoke puffs can be alive at once  */
  N_VORTS = 3,     /* how many whirlpools method 1 uses          */
};

#define WIND_MAX 3 /* most the wind can shift the smoke, cells per tick */

/* ── the smoke source, shared by all five methods ──
 * Smoke is fed in along the bottom row, strongest in the middle and fading
 * to nothing at the edges (an arch shape).  These shape and warm-up it.
 *   ARCH_MARGIN_FRAC : how much of each side edge stays empty
 *   SRC_JITTER_BASE  : smallest flicker multiplier, so the base never goes flat
 *   SRC_JITTER_RANGE : extra random flicker added on top
 *   WARMUP_TICKS     : ease the smoke in over this many ticks at startup
 *   WARMUP_CAP       : stop counting here so the counter never overflows and
 *                      the warm-up multiplier stays pinned at 1.0 afterwards */
#define ARCH_MARGIN_FRAC 0.06f
#define SRC_JITTER_BASE 0.80f
#define SRC_JITTER_RANGE 0.20f
#define WARMUP_TICKS 80
#define WARMUP_CAP 200

/* ── method 0: particle puffs ──
 *   PART_LIFE_MIN/RANGE : each puff lives MIN + random*RANGE ticks
 *   PART_VY_BASE/RANGE  : upward speed at birth (vy is negative = up)
 *   PART_VX_SPREAD      : random sideways kick at birth
 *   PART_TURB_STEP      : random wobble added to sideways speed each tick
 *   PART_VX_DAMP        : sideways speed bleeds off a little each tick
 *   SPAWN_PER_TICK      : new puffs born each tick */
#define PART_LIFE_MIN 35.f
#define PART_LIFE_RANGE 35.f
#define PART_VY_BASE 0.25f
#define PART_VY_RANGE 0.30f
#define PART_VX_SPREAD 0.4f
#define PART_TURB_STEP 0.12f
#define PART_VX_DAMP 0.97f
#define SPAWN_PER_TICK 5

/* ── shared settings for methods 1..4 (the advection methods) ──
 *   ADV_DT          : how far back in time we look each step (keep small)
 *   ADV_VEL_CAP     : cap on any wind so the look-back stays nearby (~2 cells)
 *   VORT_REACH_FRAC : how tall the smoke column should climb, as a fraction
 *                     of screen height; the fade rate is derived from it
 *   VORT_DECAY_SCALE: tuning knob multiplied into that fade rate
 *   VORT_DECAY_MIN  : smallest fade rate, so even tiny terminals clear out */
#define ADV_DT 0.8f
#define ADV_VEL_CAP 2.0f
#define VORT_REACH_FRAC 0.55f
#define VORT_DECAY_SCALE 0.9f
#define VORT_DECAY_MIN 0.010f

/* ── method 1: vortex (whirlpool) advection ──
 *   VORT_EPS : softening so the wind doesn't blow up at a whirlpool's centre
 * The four arrays describe the N_VORTS=3 whirlpools, one entry each:
 *   VORT_ORB_FRACS   : how far each one circles from screen centre (× cols)
 *   VORT_ORB_SPDS    : how fast it circles (radians/tick); sign = which way
 *   VORT_STRENGTHS   : how hard it spins the smoke; sign = clockwise or not.
 *                      Mixing signs gives counter-rotating swirls, not one
 *                      big spin
 *   VORT_INIT_ANGLES : where on its circle each one starts */
#define VORT_EPS 6.0f

static const float VORT_ORB_FRACS[N_VORTS] = {0.20f, 0.30f, 0.18f};
static const float VORT_ORB_SPDS[N_VORTS] = {0.018f, -0.011f, 0.025f};
static const float VORT_STRENGTHS[N_VORTS] = {2.5f, -1.8f, 1.4f};
static const float VORT_INIT_ANGLES[N_VORTS] = {0.0f, 2.1f, 4.3f};

/* ── method 2: curl-noise advection ──
 *   CURL_SCALE       : noise zoom; higher = smaller, tighter swirls
 *   CURL_AMP         : how strong the swirling flow is
 *   CURL_TIME_RATE   : how fast the flow pattern slowly reshapes
 *   CURL_UPWARD_BIAS : steady upward nudge so the smoke actually rises */
#define CURL_SCALE 0.10f
#define CURL_AMP 3.5f
#define CURL_TIME_RATE 0.012f
#define CURL_UPWARD_BIAS 0.5f

/* ── method 3: buoyancy plume ──
 *   BUOY_RISE       : how fast thick smoke lifts itself (thick = hot = rises)
 *   BUOY_TURB_AMP   : how much it also wobbles sideways
 *   BUOY_TURB_SCALE : zoom of that sideways wobble pattern
 *   BUOY_TURB_RATE  : how fast the wobble pattern reshapes */
#define BUOY_RISE 2.5f
#define BUOY_TURB_AMP 0.6f
#define BUOY_TURB_SCALE 0.18f
#define BUOY_TURB_RATE 0.020f

/* ── method 4: breeze advection ──
 *   BREEZE_AMP  : how far the side-sway pushes
 *   BREEZE_K    : how quickly the sway angle changes going up the screen
 *   BREEZE_RATE : how fast the whole sway shifts over time
 *   BREEZE_RISE : steady upward drift */
#define BREEZE_AMP 1.4f
#define BREEZE_K 0.25f
#define BREEZE_RATE 0.05f
#define BREEZE_RISE 0.6f

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
  struct timespec r = {(time_t)(ns / NS_PER_SEC), (long)(ns % NS_PER_SEC)};
  nanosleep(&r, NULL);
}

/* ── §3 theme + rendering pipeline ── */

/* The glyphs we draw smoke with, faintest first, densest last.  Round soft
 * shapes were picked on purpose so the smoke reads as smoke. */
static const char k_ramp[] = " .,:coO0#";
#define RAMP_N (int)(sizeof k_ramp - 1) /* 9 */

/* Colour-pair numbering: the theme ramp takes pairs 1..9, then the two HUD
 * bars sit just above it.  The HUD colours never change with the theme. */
#define CP_BASE 1
#define PAIR_HUD (CP_BASE + RAMP_N)      /* 10 — top status bar  */
#define PAIR_HINT (CP_BASE + RAMP_N + 1) /* 11 — bottom key hints */

/* How thick the smoke must be to earn each glyph.  The steps are bunched
 * in the middle of the range so the mid-thickness billows — the part you
 * look at most — get the widest variety of characters. */
static const float k_lut_breaks[RAMP_N] = {
    0.000f, /* ' '  empty        */
    0.060f, /* '.'  wisp         */
    0.150f, /* ','  thin         */
    0.260f, /* ':'  light        */
    0.370f, /* 'c'  billow edge  */
    0.480f, /* 'o'  billow mid   */
    0.600f, /* 'O'  dense        */
    0.740f, /* '0'  opaque       */
    0.880f, /* '#'  thick        */
};

static int lut_index(float v) {
  int idx = 0;
  for (int i = RAMP_N - 1; i >= 0; i--)
    if (v >= k_lut_breaks[i]) {
      idx = i;
      break;
    }
  return idx;
}

static float lut_midpoint(int idx) {
  if (idx <= 0)
    return 0.f;
  if (idx >= RAMP_N - 1)
    return 1.f;
  return (k_lut_breaks[idx] + k_lut_breaks[idx + 1]) * 0.5f;
}

/*
 * SmokeTheme — one colour scheme for the smoke.
 *
 * Each theme is a 9-colour ramp from the faintest wisp (ramp[0]) to the
 * brightest dense core (ramp[8]); the renderer picks a colour by how thick
 * the smoke is.  Every colour sits in the bright half of the palette (the
 * CLAUDE.md brightness rule) so even the faintest wisp shows up on a dark
 * terminal.  Cycle with t / T; MATRIX is the default.
 *
 *   name  : shown in the HUD
 *   fg256 : the 9 colours on a 256-colour terminal
 *   fg8   : fallback 9 colours when only 8 colours are available
 *   attr8 : per-step bold/dim flags for that fallback, to fake brightness
 *           levels the 8-colour set can't express on its own
 *
 *   0 MATRIX  green to cream     5 ICE     navy to white
 *   1 FIRE    red to yellow      6 NOVA    blue to white to yellow
 *   2 OCEANIC teal to white      7 FOREST  green to cream
 *   3 NEON    violet to white    8 DESERT  wine to cream
 *   4 MONO    gray to white      9 ECLIPSE red to peach
 */
typedef struct {
  const char *name;
  int fg256[RAMP_N];
  int fg8[RAMP_N];
  attr_t attr8[RAMP_N];
} SmokeTheme;

static const SmokeTheme k_themes[] = {
    {/* 0  MATRIX — dark green → lime → cream */
     "MATRIX",
     {28, 34, 40, 46, 82, 118, 154, 190, 230},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE},
     {A_DIM, A_DIM, A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 1  FIRE — dark red → orange → yellow */
     "FIRE",
     {88, 124, 130, 166, 202, 208, 214, 220, 226},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 2  OCEANIC — deep teal → cyan → white */
     "OCEANIC",
     {24, 31, 38, 44, 51, 87, 123, 159, 231},
     {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 3  NEON — violet → pink → white */
     "NEON",
     {53, 91, 134, 165, 201, 207, 213, 219, 225},
     {COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_MAGENTA, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 4  MONO — gray ramp → white */
     "MONO",
     {242, 244, 245, 247, 248, 250, 251, 253, 255},
     {COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_DIM, A_DIM, A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 5  ICE — navy → pale blue → white */
     "ICE",
     {24, 31, 67, 75, 117, 153, 195, 230, 231},
     {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE,
      COLOR_WHITE, COLOR_WHITE, COLOR_WHITE},
     {A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 6  NOVA — deep blue → white → yellow */
     "NOVA",
     {60, 75, 117, 159, 195, 219, 220, 226, 231},
     {COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 7  FOREST — dark green → gold → cream */
     "FOREST",
     {28, 64, 70, 112, 148, 154, 184, 220, 230},
     {COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 8  DESERT — wine → tan → cream */
     "DESERT",
     {94, 130, 137, 173, 179, 215, 222, 229, 230},
     {COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_NORMAL, A_BOLD, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {/* 9  ECLIPSE — dark red → peach */
     "ECLIPSE",
     {52, 88, 95, 131, 167, 173, 209, 215, 217},
     {COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_RED, COLOR_RED, COLOR_WHITE},
     {A_DIM, A_NORMAL, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
};

#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

static void theme_apply(int t) {
  const SmokeTheme *th = &k_themes[t];
  for (int i = 0; i < RAMP_N; i++) {
    if (COLORS >= 256)
      init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
    else
      init_pair(CP_BASE + i, th->fg8[i], COLOR_BLACK);
  }
}

static void color_init(int theme) {
  start_color();
  theme_apply(theme);
  /* The two HUD bars keep the same bright colours no matter which theme is
   * active, so they stay readable over any smoke. */
  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, COLOR_BLACK); /* bright yellow */
    init_pair(PAIR_HINT, 51, COLOR_BLACK); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, COLOR_BLACK);
    init_pair(PAIR_HINT, COLOR_CYAN, COLOR_BLACK);
  }
}

static attr_t ramp_attr(int i, int theme) {
  attr_t a = COLOR_PAIR(CP_BASE + i);
  if (COLORS >= 256) {
    if (i >= RAMP_N - 2)
      a |= A_BOLD;
  } else {
    a |= k_themes[theme].attr8[i];
  }
  return a;
}

/* ── §4 shared helpers ── */

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/* Ease the smoke in at startup: returns a fade-in factor that climbs from
 * 0 to 1 over the first WARMUP_TICKS, then stays at 1.  Also advances the
 * shared counter (stored in the Scene) and stops it before it can overflow.
 * Call exactly once per tick. */
static float warmup_scale(int *warmup) {
  float s =
      (*warmup < WARMUP_TICKS) ? (float)*warmup / (float)WARMUP_TICKS : 1.f;
  (*warmup)++;
  if (*warmup > WARMUP_CAP)
    *warmup = WARMUP_CAP;
  return s;
}

/* Read the grid at an in-between spot like (3.4, 7.8) by blending the four
 * cells around it, weighted by how close the spot is to each — so the value
 * changes smoothly instead of jumping cell to cell.  Off the edge, it just
 * reuses the nearest edge cell. */
static float bilinear_sample(const float *grid, float sx, float sy, int cols,
                             int rows) {
  int x0 = (int)sx, y0 = (int)sy;
  int x1 = x0 + 1, y1 = y0 + 1;

  if (x0 < 0)
    x0 = 0;
  if (x0 >= cols)
    x0 = cols - 1;
  if (x1 < 0)
    x1 = 0;
  if (x1 >= cols)
    x1 = cols - 1;
  if (y0 < 0)
    y0 = 0;
  if (y0 >= rows)
    y0 = rows - 1;
  if (y1 < 0)
    y1 = 0;
  if (y1 >= rows)
    y1 = rows - 1;

  float tx = sx - (float)(int)sx;
  float ty = sy - (float)(int)sy;
  if (tx < 0.f)
    tx = 0.f;
  if (tx > 1.f)
    tx = 1.f;
  if (ty < 0.f)
    ty = 0.f;
  if (ty > 1.f)
    ty = 1.f;

  float v00 = grid[y0 * cols + x0], v10 = grid[y0 * cols + x1];
  float v01 = grid[y1 * cols + x0], v11 = grid[y1 * cols + x1];

  return (1.f - tx) * (1.f - ty) * v00 + tx * (1.f - ty) * v10 +
         (1.f - tx) * ty * v01 + tx * ty * v11;
}

/* ── §4a the shared stir-the-grid kernel (methods 1..4) ──
 *
 * Methods 1..4 all move smoke the same clever way, called semi-Lagrangian
 * advection (Stam, "Stable Fluids", 1999).  Instead of pushing each cell's
 * smoke forward along the wind (which can blow up), we ask the reverse
 * question for every cell: "given the wind here, where was this smoke one
 * step ago?"  Step backward to that spot, read the smoke that was there,
 * and that becomes the new value here.  Reading the past instead of writing
 * the future is what keeps it stable no matter how strong the wind.
 *
 * What every cell does each tick:
 *   - work out the wind at this cell      (the ONLY part each method changes)
 *   - cap the wind so the look-back stays nearby and the read stays accurate
 *   - step backward along the wind one short step
 *   - read the smoke that was there (blended, via bilinear_sample)
 *   - fade it a touch, and if this is the bottom row add fresh source smoke
 *
 * So each method below is just "compute the wind, hand it to sl_step_cell" —
 * the difference between vortex / curl / buoyancy / breeze IS the wind, and
 * everything else lives here.
 *
 * SLCtx — the bundle of values the per-cell step needs.
 *
 * These are all the same for every cell in a tick, so we compute them once
 * (sl_make_ctx) and pass the bundle by pointer rather than 10 loose
 * arguments per cell.
 *
 *   density   : the previous frame's smoke, read-only here — this is what
 *               the backward look-up samples from
 *   work      : where this tick's new smoke is written; the caller copies
 *               it back over `density` afterward.  (Same buffer the renderer
 *               later borrows as scratch — the tick finishes first, so they
 *               never collide.)
 *   cols, rows: grid size, cached so we don't re-ask ncurses every cell
 *   fy        : the bottom row's index; only that row gets fresh source smoke
 *   margin    : how many columns at each side stay empty (no source)
 *   span      : the width of the source region between the margins
 *   wind_acc  : how far sideways wind has pushed the source so far, so a
 *               steady wind makes the whole smoke base actually slide along,
 *               not just lean.  Advanced once per tick by scene_tick.
 *   intensity : the source-strength knob from the g / G keys (0.1 .. 1.0)
 *   wscale    : the startup fade-in factor from warmup_scale (0 .. 1)
 *   decay     : how much smoke each cell loses per tick, tuned from screen
 *               height so the column climbs to about VORT_REACH_FRAC of the
 *               screen before fading out
 * ──────────────────────────────────────────────────────────────────── */
typedef struct {
  const float *density;
  float *work;
  int cols, rows;
  int fy;
  float margin;
  float span;
  int wind_acc;
  float intensity;
  float wscale;
  float decay;
} SLCtx;

/* Pick a fade rate from the screen height so the smoke column climbs to
 * roughly VORT_REACH_FRAC of the screen and no higher.  Taller screens fade
 * more slowly so the smoke can reach the same relative height. */
static inline float sl_decay_for_grid(int rows) {
  float target = (float)rows * VORT_REACH_FRAC;
  float d = (target > 1.f) ? (1.f / target) * VORT_DECAY_SCALE : VORT_DECAY_MIN;
  if (d < VORT_DECAY_MIN)
    d = VORT_DECAY_MIN;
  return d;
}

/* Fill the shared bundle once at the start of a tick. */
static inline SLCtx sl_make_ctx(const float *density, float *work, int cols,
                                int rows, float intensity, int wind_acc,
                                float wscale) {
  SLCtx c;
  c.density = density;
  c.work = work;
  c.cols = cols;
  c.rows = rows;
  c.fy = rows - 1;
  c.margin = (float)cols * ARCH_MARGIN_FRAC;
  c.span = (float)cols - 2.f * c.margin;
  c.wind_acc = wind_acc;
  c.intensity = intensity;
  c.wscale = wscale;
  c.decay = sl_decay_for_grid(rows);
  return c;
}

/* Cap the wind so the backward look-up never lands more than a cell or two
 * away — close enough that the blended read stays accurate and the sim
 * stays stable. */
static inline void sl_clamp_velocity(float *vx, float *vy) {
  if (*vx > ADV_VEL_CAP)
    *vx = ADV_VEL_CAP;
  if (*vx < -ADV_VEL_CAP)
    *vx = -ADV_VEL_CAP;
  if (*vy > ADV_VEL_CAP)
    *vy = ADV_VEL_CAP;
  if (*vy < -ADV_VEL_CAP)
    *vy = -ADV_VEL_CAP;
}

/* How much fresh smoke to feed into this cell.  Only the bottom row gets
 * any; across it the amount peaks in the middle and fades to nothing at the
 * edges (the arch shape), with a little random flicker so the base looks
 * alive rather than a flat line.  Returns 0 for every other cell. */
static inline float sl_source_at(const SLCtx *c, int x, int y) {
  if (y != c->fy)
    return 0.f;
  float ts = ((float)x - c->margin - (float)c->wind_acc) /
             (c->span > 0.f ? c->span : 1.f);
  if (ts < 0.f || ts > 1.f)
    return 0.f;
  float edge = (ts < 0.5f) ? ts : 1.f - ts;
  float arch = (edge * 2.f) * (edge * 2.f);
  float jit = SRC_JITTER_BASE + SRC_JITTER_RANGE * ((float)rand() / RAND_MAX);
  return c->intensity * arch * jit * c->wscale;
}

/* Update one cell once the method has worked out the wind here: cap the
 * wind, step backward, read the old smoke, fade it, add any source, and
 * write the result.  This is the shared body of all four advection methods. */
static inline void sl_step_cell(const SLCtx *c, int x, int y, float vx,
                                float vy) {
  sl_clamp_velocity(&vx, &vy);

  float sx = (float)x - vx * ADV_DT;
  float sy = (float)y - vy * ADV_DT;
  float adv = bilinear_sample(c->density, sx, sy, c->cols, c->rows);
  float src = sl_source_at(c, x, y);

  float v = adv * (1.f - c->decay) + src;
  if (v < 0.f)
    v = 0.f;
  if (v > 1.f)
    v = 1.f;
  c->work[y * c->cols + x] = v;
}

/* ── §5 method 0 — particle puffs ── */

/*
 * Particle — one little smoke puff.  Method 0 only; the other methods
 * ignore these entirely and fill the grid directly.
 *
 * The idea (Reeves, "Particle Systems", 1983): instead of working on the
 * grid, keep a crowd of tiny puffs.  Each is born at the base, drifts up
 * with a bit of random wobble, and fades over a fixed lifetime.  Every tick
 * we wipe the grid and re-stamp each living puff onto it, so the smoke you
 * see is the overlap of the whole crowd.
 *
 * One nice touch: a puff stamps its brightness as life squared, not life.
 * Squaring means a fresh puff looks solid right away but trails off gently
 * at the end, instead of popping out of existence (Reeves §4.2).
 *
 * Life: a puff is born, drifts and fades each tick, and when its life runs
 * out or it leaves the screen its slot is freed for the next new puff.
 *
 *   x, y   : where the puff is, in grid cells, kept as decimals so it can
 *            sit between cells — that's what lets the stamp spread softly
 *            instead of snapping cell to cell
 *   vx, vy : how fast it's moving, cells per tick.  vy is negative going up
 *            (screen rows count downward).  Sideways speed starts near zero
 *            and picks up small random nudges that slowly bleed off
 *   life   : counts down from 1 to 0; the stamp uses life squared as
 *            brightness, so a half-spent puff (0.5) draws at 0.25
 *   decay  : how much life is lost each tick, set at birth to 1/lifetime so
 *            no division is needed every tick
 *   active : is this slot in use?  Empty slots are skipped and reused
 */
typedef struct {
  float x, y;
  float vx, vy;
  float life;
  float decay;
  bool active;
} Particle;

/* Birth one puff along the bottom.  We pick its column by rolling dice and
 * accepting more often near the middle, so most puffs come from the centre
 * of the base (matching the arch source shape).  After 8 tries we just use
 * the centre, so a busy frame can't get stuck here. */
static void particle_spawn(Particle *p, int cols, int rows, float intensity,
                           int wind_acc, int warmup) {
  float wscale =
      (warmup < WARMUP_TICKS) ? (float)warmup / (float)WARMUP_TICKS : 1.f;
  float margin = (float)cols * ARCH_MARGIN_FRAC;
  float span = (float)cols - 2.f * margin;

  float bx = (float)cols * 0.5f;
  for (int attempt = 0; attempt < 8; attempt++) {
    float t = (float)rand() / RAND_MAX;
    float cx = margin + t * span + (float)wind_acc;
    float edge = (t < 0.5f) ? t : 1.f - t;
    float arch = (edge * 2.f) * (edge * 2.f);
    float accept = arch * intensity * wscale;
    if (((float)rand() / RAND_MAX) < accept) {
      bx = cx;
      break;
    }
  }

  p->x = bx;
  p->y = (float)(rows - 1) - 0.5f;
  p->vx = ((float)rand() / RAND_MAX - 0.5f) * PART_VX_SPREAD;
  p->vy = -(PART_VY_BASE + ((float)rand() / RAND_MAX) * PART_VY_RANGE);
  float life_ticks =
      PART_LIFE_MIN + ((float)rand() / RAND_MAX) * PART_LIFE_RANGE;
  p->life = 1.0f;
  p->decay = 1.0f / life_ticks;
  p->active = true;
}

/* ── moving one puff ── */

/* Advance a single puff by one tick: nudge its sideways speed randomly and
 * let a little of it bleed off, slide it by its speed, and tick its life
 * down. */
static inline void particle_integrate(Particle *p) {
  p->vx += ((float)rand() / RAND_MAX - 0.5f) * PART_TURB_STEP;
  p->vx *= PART_VX_DAMP;
  p->x += p->vx;
  p->y += p->vy;
  p->life -= p->decay;
}

/* True if the puff still has life left and hasn't drifted off the screen.
 * When this turns false the caller frees the puff's slot. */
static inline bool particle_still_alive(const Particle *p, int cols, int rows) {
  return p->life > 0.f && p->x >= 0.f && p->x < (float)cols && p->y >= 0.f &&
         p->y < (float)rows;
}

/* Stamp one puff's brightness onto the grid, spread across the four cells
 * around it.  Each cell gets a share based on how close the puff sits to it,
 * and the four shares add up to the whole.  Spreading over four cells (and
 * the next puff doing the same) is what makes puffs look soft and fuzzy
 * instead of a hard single-cell dot. */
static inline void particle_splat_bilinear(const Particle *p, float *density,
                                           int cols, int rows) {
  float pd = p->life * p->life; /* brightness = life squared (gentle fade) */
  int x0 = (int)p->x, y0 = (int)p->y;
  int x1 = x0 + 1, y1 = y0 + 1;
  float tx = p->x - (float)x0;
  float ty = p->y - (float)y0;

  if (x0 >= 0 && x0 < cols && y0 >= 0 && y0 < rows)
    density[y0 * cols + x0] += pd * (1.f - tx) * (1.f - ty);
  if (x1 >= 0 && x1 < cols && y0 >= 0 && y0 < rows)
    density[y0 * cols + x1] += pd * tx * (1.f - ty);
  if (x0 >= 0 && x0 < cols && y1 >= 0 && y1 < rows)
    density[y1 * cols + x0] += pd * (1.f - tx) * ty;
  if (x1 >= 0 && x1 < cols && y1 >= 0 && y1 < rows)
    density[y1 * cols + x1] += pd * tx * ty;
}

/* ── sweeping the whole crowd ── */

/* Move every living puff one step and retire the ones that just died. */
static void particles_integrate_all(Particle *parts, int cols, int rows) {
  for (int i = 0; i < MAX_PARTS; i++) {
    Particle *p = &parts[i];
    if (!p->active)
      continue;
    particle_integrate(p);
    if (!particle_still_alive(p, cols, rows))
      p->active = false;
  }
}

/* Birth a few new puffs, walking forward from the last slot we used to find
 * empty ones.  If the crowd is completely full we just skip the birth rather
 * than spin forever. */
static void particles_spawn_burst(Particle *parts, int *next_idx, int cols,
                                  int rows, float intensity, int wind_acc,
                                  int warmup) {
  for (int s = 0; s < SPAWN_PER_TICK; s++) {
    for (int tries = 0; tries < MAX_PARTS; tries++) {
      *next_idx = (*next_idx + 1) % MAX_PARTS;
      if (!parts[*next_idx].active) {
        particle_spawn(&parts[*next_idx], cols, rows, intensity, wind_acc,
                       warmup);
        break;
      }
    }
  }
}

/* Wipe the grid and re-stamp every living puff.  We rebuild from scratch
 * each tick rather than adding on, so puffs that just died leave no ghost
 * smoke behind. */
static void particles_rebuild_density(Particle *parts, float *density, int cols,
                                      int rows) {
  memset(density, 0, (size_t)(cols * rows) * sizeof(float));
  for (int i = 0; i < MAX_PARTS; i++) {
    if (!parts[i].active)
      continue;
    particle_splat_bilinear(&parts[i], density, cols, rows);
  }
  /* Where puffs piled up a cell can read above 1; pin it back to 1 so the
   * renderer stays in range. */
  for (int i = 0; i < cols * rows; i++)
    if (density[i] > 1.f)
      density[i] = 1.f;
}

/* One tick of method 0: move the crowd, retire the dead, birth a few new
 * ones, advance the startup fade-in, then rebuild the grid from whatever's
 * still alive.  What you see is a cloud of soft little blobs you can almost
 * count, each fading over its own short life — great for thin, detailed
 * smoke (the grid-based methods look better when the smoke gets dense). */
static void particle_tick(Particle *parts, int *next_idx, float *density,
                          int cols, int rows, float intensity, int wind_acc,
                          int *warmup) {
  particles_integrate_all(parts, cols, rows);
  particles_spawn_burst(parts, next_idx, cols, rows, intensity, wind_acc,
                        *warmup);
  warmup_scale(warmup); /* just advancing the counter; the value isn't used */
  particles_rebuild_density(parts, density, cols, rows);
}

/* ── §6 method 1 — vortex (whirlpool) advection ── */

/*
 * Vortex — one spinning whirlpool that stirs the smoke.  Method 1 only.
 *
 * A point vortex is the classic textbook whirlpool (Lamb, "Hydrodynamics",
 * 1932): it sets up a circular flow around itself that spins fast up close
 * and trails off with distance.  The vortex is never drawn — it only
 * decides which way the wind blows at each cell, and method 1 sums up the
 * pull of all three to get the wind everywhere.
 *
 * A tiny softening term (VORT_EPS) keeps the flow from blowing up to
 * infinity right at the centre, which is both physically truer (real
 * whirlpools have a calm eye) and stops the sim reading garbage there
 * (Bridson §2.3).
 *
 * Each whirlpool also circles slowly around the screen centre on its own
 * little orbit.  Giving them mixed spin directions and orbit directions is
 * what produces several counter-rotating swirls instead of one big spin.
 *
 *   cx, cy   : where this whirlpool is right now, in grid cells; moved
 *              along its orbit each tick
 *   strength : how hard it spins the smoke, and which way (sign).  Around
 *              1.4 to 2.5 gives clear curls without overpowering the wind cap
 *   orb_r    : how far from screen centre it circles, in cells.  Bigger
 *              sweeps across more of the screen; smaller stays central and
 *              makes tighter local swirls
 *   orb_a    : where it is on that circle right now (an angle), nudged
 *              forward each tick; the three start at different angles so
 *              they don't bunch up
 *   orb_spd  : how fast it circles, and which way (sign).  A whirlpool can
 *              spin one way while circling the other, which adds visual
 *              interest
 */
typedef struct {
  float cx, cy;
  float strength;
  float orb_r;
  float orb_a;
  float orb_spd;
} Vortex;

/* Place the three whirlpools using the preset arrays from §1, turning each
 * orbit fraction into an actual cell distance. */
static void vortex_init(Vortex vorts[N_VORTS], int cols, int rows) {
  float cx = (float)cols * 0.5f;
  float cy = (float)rows * 0.5f;

  for (int i = 0; i < N_VORTS; i++) {
    vorts[i].orb_r = VORT_ORB_FRACS[i] * (float)cols;
    vorts[i].orb_spd = VORT_ORB_SPDS[i];
    vorts[i].strength = VORT_STRENGTHS[i];
    vorts[i].orb_a = VORT_INIT_ANGLES[i];
    vorts[i].cx = cx + vorts[i].orb_r * cosf(vorts[i].orb_a);
    vorts[i].cy = cy + vorts[i].orb_r * sinf(vorts[i].orb_a);
  }
}

static void vortex_advance_orbits(Vortex vorts[N_VORTS], int cols, int rows) {
  float cx = (float)cols * 0.5f;
  float cy = (float)rows * 0.5f;

  for (int i = 0; i < N_VORTS; i++) {
    vorts[i].orb_a += vorts[i].orb_spd;
    vorts[i].cx = cx + vorts[i].orb_r * cosf(vorts[i].orb_a);
    vorts[i].cy = cy + vorts[i].orb_r * sinf(vorts[i].orb_a);
  }
}

/* The wind at one cell is the sum of the pull from all three whirlpools.
 * Each pulls in a circle around its own centre, strong up close and weaker
 * with distance.  Where their mixed spins agree they reinforce; where they
 * fight they cancel — that's what carves out the separate swirls. */
static inline void vortex_velocity_at(int x, int y, const Vortex vorts[N_VORTS],
                                      float *out_vx, float *out_vy) {
  float vx = 0.f, vy = 0.f;
  for (int i = 0; i < N_VORTS; i++) {
    float dx = (float)x - vorts[i].cx;
    float dy = (float)y - vorts[i].cy;
    float r2 = dx * dx + dy * dy + VORT_EPS;
    vx += vorts[i].strength * (-dy) / r2;
    vy += vorts[i].strength * (dx) / r2;
  }
  *out_vx = vx;
  *out_vy = vy;
}

/* One tick of method 1: nudge each whirlpool along its orbit, then for every
 * cell work out the combined whirlpool wind and hand it to the shared step.
 * What you see is curls and swirls — the smoke simply follows the spinning
 * flow the whirlpools set up. */
static void vortex_tick(float *density, float *work, Vortex vorts[N_VORTS],
                        int cols, int rows, float intensity, int wind_acc,
                        int *warmup) {
  float wscale = warmup_scale(warmup);
  vortex_advance_orbits(vorts, cols, rows);

  SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      float vx, vy;
      vortex_velocity_at(x, y, vorts, &vx, &vy);
      sl_step_cell(&c, x, y, vx, vy);
    }
  }

  memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ── §7 method 2 — curl-noise advection ── */

/*
 * Curl-noise smoke: stir with a smooth, swirly random flow instead of a few
 * sharp whirlpools.  We start from a soft blobby random field and take its
 * "curl" — the swirl direction at each point.  A handy property of curl is
 * that the flow neither piles smoke up nor drains it away anywhere, so the
 * total amount of smoke is preserved on its own (Bridson §4).  Compared to
 * the vortex method, the flow is gentle and spread out with no obvious
 * centres.
 */

/* Scramble two grid coordinates into a repeatable random value in [0, 1).
 * Same input always gives the same number, which is what makes the noise
 * stable from frame to frame. */
static inline float curl_hash01(int ix, int iy, int seed) {
  uint32_t h = (uint32_t)(ix * 374761393 + iy * 668265263 + seed * 1274126177);
  h = (h ^ (h >> 13)) * 1274126177u;
  h ^= h >> 16;
  return (float)(h & 0xFFFFFFu) / (float)0x1000000; /* [0, 1) */
}

/* Smooth random value at any spot: grab the four random corners around it
 * and blend them with an easing curve, so the field rolls gently instead of
 * jumping between corners. */
static inline float curl_noise2d(float x, float y, int seed) {
  int ix = (int)floorf(x);
  int iy = (int)floorf(y);
  float fx = x - (float)ix;
  float fy = y - (float)iy;
  float sx = fx * fx * (3.f - 2.f * fx); /* S-shaped ease, no sharp corners */
  float sy = fy * fy * (3.f - 2.f * fy);
  float v00 = curl_hash01(ix, iy, seed);
  float v10 = curl_hash01(ix + 1, iy, seed);
  float v01 = curl_hash01(ix, iy + 1, seed);
  float v11 = curl_hash01(ix + 1, iy + 1, seed);
  float a = v00 + (v10 - v00) * sx;
  float b = v01 + (v11 - v01) * sx;
  return a + (b - a) * sy;
}

/* Read the swirl direction of the noise field at this cell, by comparing the
 * noise just to each side of the cell.  We slide the noise slowly with time
 * (t) so the flow keeps reshaping, and add a steady upward nudge so the smoke
 * climbs instead of just swirling on the spot. */
static inline void curl_velocity_at(int x, int y, float t, float *out_vx,
                                    float *out_vy) {
  float h = 0.5f;
  float scale = CURL_SCALE;
  float fx = (float)x, fy = (float)y;
  float yp = curl_noise2d(fx * scale, (fy + h) * scale + t, 0);
  float ym = curl_noise2d(fx * scale, (fy - h) * scale + t, 0);
  float xp = curl_noise2d((fx + h) * scale, fy * scale + t, 0);
  float xm = curl_noise2d((fx - h) * scale, fy * scale + t, 0);
  *out_vx = CURL_AMP * (yp - ym) / (2.f * h);
  *out_vy = -CURL_AMP * (xp - xm) / (2.f * h) - CURL_UPWARD_BIAS;
}

/* One tick of method 2: advance the noise's slow drift, then for each cell
 * read the swirly noise wind and hand it to the shared step.  Because this
 * kind of flow neither gathers nor drains smoke, the column holds together
 * and meanders through the frame with no obvious centres — closer to real
 * turbulence than the sharp whirlpools of method 1. */
static void curl_tick(float *density, float *work, int cols, int rows,
                      float intensity, int wind_acc, int *warmup) {
  float wscale = warmup_scale(warmup);
  SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);
  float t = (float)*warmup * CURL_TIME_RATE;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      float vx, vy;
      curl_velocity_at(x, y, t, &vx, &vy);
      sl_step_cell(&c, x, y, vx, vy);
    }
  }
  memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ── §8 method 3 — buoyancy plume ── */

/*
 * Buoyancy plume: the smoke lifts itself because thicker smoke counts as
 * hotter, and hot things rise.  There's no outside stirring here — the
 * upward push at each cell just comes from how much smoke is sitting there
 * (the Boussinesq idea, where smoke thickness stands in for temperature).
 * The look: a tall thin column that flattens into a mushroom cap near the
 * top, where the smoke has thinned enough that it stops rising.
 */
/* The wind at one cell for the plume: a strong upward pull where the smoke
 * is thick and none where it's empty, plus a little sideways wobble so the
 * column doesn't rise perfectly straight (which never looks natural). */
static inline void buoy_velocity_at(int x, int y, float t, const float *density,
                                    int cols, float *out_vx, float *out_vy) {
  float d_here = density[y * cols + x];
  float n = curl_noise2d((float)x * BUOY_TURB_SCALE,
                         (float)y * BUOY_TURB_SCALE + t, 7);
  *out_vx = (n - 0.5f) * 2.f * BUOY_TURB_AMP;
  *out_vy = -BUOY_RISE * d_here;
}

/* One tick of method 3: for each cell, read how thick the smoke is there,
 * turn that into an upward pull (plus a little sideways wobble), and hand it
 * to the shared step.  Thick smoke rises fast and leaves an even thicker
 * pocket below, so the column shoots up in a narrow plume, then mushrooms
 * out near the top where it has thinned and the lift gives out. */
static void buoy_tick(float *density, float *work, int cols, int rows,
                      float intensity, int wind_acc, int *warmup) {
  float wscale = warmup_scale(warmup);
  SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);
  float t = (float)*warmup * BUOY_TURB_RATE;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      float vx, vy;
      buoy_velocity_at(x, y, t, density, cols, &vx, &vy);
      sl_step_cell(&c, x, y, vx, vy);
    }
  }
  memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ── §9 method 4 — breeze advection ── */

/*
 * Breeze advection: a gentle side-wind that sways back and forth and slowly
 * shifts over time, plus a steady upward drift.  The side-push is the same
 * everywhere across a row, but the rows are slightly out of step with each
 * other, so the column bends like a curtain in a draught.  No noise tables
 * here — the wind is just a sine wave, the cheapest of the four methods.
 */
/* The side-sway for a whole row: it depends only on which row and on time,
 * never on the column, so every cell in a row gets the same push (that's why
 * the flow stays smooth).  Neighbouring rows are slightly out of step, which
 * is what gives the curtain-bend look.  Plus a steady upward drift. */
static inline void breeze_velocity_at(int y, float t, float *out_vx,
                                      float *out_vy) {
  *out_vx = BREEZE_AMP * sinf(t + (float)y * BREEZE_K);
  *out_vy = -BREEZE_RISE;
}

/* One tick of method 4: advance the sway over time, then for each row work
 * out its single side-push once and apply it across the whole row.  You see
 * the smoke rise as a swaying curtain — a clean back-and-forth in any one
 * row, and a snaking bend reading up a column, with no turbulence or curls. */
static void breeze_tick(float *density, float *work, int cols, int rows,
                        float intensity, int wind_acc, int *warmup) {
  float wscale = warmup_scale(warmup);
  SLCtx c = sl_make_ctx(density, work, cols, rows, intensity, wind_acc, wscale);
  float t = (float)*warmup * BREEZE_RATE;

  for (int y = 0; y < rows; y++) {
    /* This row's side-push, found once and reused across the row. */
    float vx, vy;
    breeze_velocity_at(y, t, &vx, &vy);

    for (int x = 0; x < cols; x++)
      sl_step_cell(&c, x, y, vx, vy);
  }
  memcpy(density, work, (size_t)(cols * rows) * sizeof(float));
}

/* ── §10 scene ── */

/*
 * Scene — all the state the simulation owns, in one place.
 *
 * Almost everything that changes at runtime lives here: the smoke grid and
 * its two scratch copies, the particle crowd, the whirlpools, plus the knobs
 * (which method, wind, source strength, theme, paused).  Keeping it in one
 * struct makes setup, teardown, and resize one-liners.
 *
 * It's split into two halves on purpose.  The simulation half is everything
 * the physics tick reads and writes; the render half is just the look (theme
 * and a redraw flag) and the physics never touches it — changing the theme
 * mid-plume only repaints, it doesn't change how the smoke moves.  The Scene
 * knows nothing about the terminal: physics fills the grid, the renderer
 * reads it, so the sim could run with no screen at all.
 *
 * Most fields serve one method only: the particle crowd is method 0's, the
 * whirlpools are method 1's; methods 2..4 carry no extra state and just use
 * the shared grid.
 */
typedef struct {
  /* ── simulation half — the physics tick reads and writes these ── */

  /* The smoke grid: how thick the smoke is in each cell, 0 to 1.  Every
   * method writes here and the renderer reads it.  (The renderer briefly
   * stuffs -1 into a working copy to mark "empty cell"; the real grid stays
   * 0 to 1.) */
  float *density;

  /* Last frame's grid, saved at the end of drawing.  The next frame compares
   * against it to find cells that just went empty, so it only needs to erase
   * those rather than the whole screen. */
  float *prev_density;

  /* A scratch buffer borrowed for two jobs in turn each frame: first the
   * advection methods write their new grid here before copying it back, then
   * the renderer reuses it as dither scratch.  The tick always runs before
   * the draw, so the two never overlap. */
  float *work;

  /* Grid size, remembered here so the inner loops don't have to ask ncurses
   * every cell. */
  int cols, rows;

  /* Which method is running: 0 particle, 1 vortex, 2 curl, 3 buoyancy,
   * 4 breeze.  Cycled by 'a'; they all fill the same grid. */
  int algo;

  /* Startup fade-in counter, shared by every method.  Climbs each tick then
   * holds; while it climbs the source is scaled down so the smoke eases in
   * instead of popping up full.  Reset to 0 when the method changes or the
   * window resizes so the new start looks clean. */
  int warmup;

  /* Source strength knob (0.1 to 1.0), set by g / G.  Low feeds a thin wisp
   * that barely climbs; full feeds a thick column that reaches the top. */
  float source;

  /* Wind step per tick: positive blows right, set by w / W, '0' clears it.
   * Read once a tick to push wind_acc along. */
  int wind;

  /* How far the wind has pushed the smoke base sideways so far.  scene_tick
   * adds `wind` to this exactly once per tick (wrapping around the screen);
   * the methods only read it.  If a method advanced it too, the wind would
   * count twice. */
  int wind_acc;

  /* Paused?  When set the tick does nothing, but drawing continues so you see
   * the frozen frame.  The whirlpools keep their positions, so resuming picks
   * up right where it left off. */
  bool paused;

  /* The puff crowd — method 0 only; the others leave it alone.  See Particle. */
  Particle parts[MAX_PARTS];

  /* Where the next puff search should start looking for a free slot.  Walking
   * forward from here beats scanning the whole crowd every time. */
  int part_idx;

  /* The three whirlpools — method 1 only; see Vortex.  Set up with mixed spin
   * directions so the smoke gets several counter-rotating swirls, not one big
   * spin. */
  Vortex vorts[N_VORTS];

  /* ── render half — only the drawing reads these; physics ignores them ── */

  /* Which colour scheme is active, an index into k_themes; cycled by t / T.
   * Purely cosmetic — the smoke moves the same whatever theme is chosen. */
  int theme;

  /* Set this to force the next frame to wipe the whole screen (after a method
   * change, theme change, or resize).  Normally drawing only touches cells
   * that changed; this flag overrides that for the one frame after a jump. */
  bool needs_clear;
} Scene;

static void scene_alloc(Scene *sc) {
  sc->density = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
  sc->prev_density = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
  sc->work = calloc((size_t)(sc->cols * sc->rows), sizeof(float));
}

static void scene_free_bufs(Scene *sc) {
  free(sc->density);
  sc->density = NULL;
  free(sc->prev_density);
  sc->prev_density = NULL;
  free(sc->work);
  sc->work = NULL;
}

static void scene_init(Scene *sc, int cols, int rows, int algo, int theme) {
  memset(sc, 0, sizeof *sc);
  sc->cols = cols;
  sc->rows = rows;
  sc->algo = algo;
  sc->theme = theme;
  sc->source = 0.85f;
  sc->wind = 0;
  sc->wind_acc = 0;
  sc->warmup = 0;
  sc->part_idx = 0;
  scene_alloc(sc);
  vortex_init(sc->vorts, cols, rows);
}

static void scene_resize(Scene *sc, int cols, int rows) {
  int algo = sc->algo;
  int theme = sc->theme;
  float src = sc->source;
  int wind = sc->wind;
  scene_free_bufs(sc);
  sc->cols = cols;
  sc->rows = rows;
  sc->algo = algo;
  sc->theme = theme;
  sc->source = src;
  sc->wind = wind;
  sc->wind_acc = 0;
  sc->warmup = 0;
  sc->needs_clear = true;
  scene_alloc(sc);
  vortex_init(sc->vorts, cols, rows);
  memset(sc->parts, 0, sizeof sc->parts);
  sc->part_idx = 0;
}

/* One step of the whole sim: push the wind along once (this is the only
 * place that does so), then run whichever method is active. */
static void scene_tick(Scene *sc) {
  if (sc->paused)
    return;

  sc->wind_acc += sc->wind;
  if (sc->wind_acc >= sc->cols || sc->wind_acc <= -sc->cols)
    sc->wind_acc = 0;

  switch (sc->algo) {
  case 0:
    particle_tick(sc->parts, &sc->part_idx, sc->density, sc->cols, sc->rows,
                  sc->source, sc->wind_acc, &sc->warmup);
    break;
  case 1:
    vortex_tick(sc->density, sc->work, sc->vorts, sc->cols, sc->rows,
                sc->source, sc->wind_acc, &sc->warmup);
    break;
  case 2:
    curl_tick(sc->density, sc->work, sc->cols, sc->rows, sc->source,
              sc->wind_acc, &sc->warmup);
    break;
  case 3:
    buoy_tick(sc->density, sc->work, sc->cols, sc->rows, sc->source,
              sc->wind_acc, &sc->warmup);
    break;
  case 4:
    breeze_tick(sc->density, sc->work, sc->cols, sc->rows, sc->source,
                sc->wind_acc, &sc->warmup);
    break;
  }
}

/* ── turning the grid into glyphs ── */

/* Step 1: nudge the brightness so it matches how the eye sees it (raw smoke
 * thickness looks too dark in the middle).  Empty cells are flagged with -1
 * so the next step knows to leave the background alone and not sprinkle stray
 * dots into it. */
static void render_density_to_gamma(const float *density, float *scratch,
                                    int cols, int rows) {
  for (int i = 0; i < cols * rows; i++) {
    float v = density[i];
    scratch[i] = (v <= 0.f) ? -1.f : powf(fminf(1.f, v), 1.f / 2.2f);
  }
}

/* Dithering, part of step 2.  Picking a glyph rounds the brightness off; this
 * spreads the leftover rounding error onto the neighbours not yet drawn, so
 * the smoke shades smoothly instead of showing hard bands.  The 7/3/5/1-over-16
 * shares are the classic Floyd-Steinberg pattern (1976).  Empty (-1) cells are
 * skipped so they stay part of the background. */
static inline void floyd_steinberg_diffuse(float *d, int i, int x, int y,
                                           int cols, int rows, float err) {
  if (x + 1 < cols && d[i + 1] >= 0.f)
    d[i + 1] += err * (7.f / 16.f);
  if (y + 1 < rows) {
    if (x - 1 >= 0 && d[i + cols - 1] >= 0.f)
      d[i + cols - 1] += err * (3.f / 16.f);
    if (d[i + cols] >= 0.f)
      d[i + cols] += err * (5.f / 16.f);
    if (x + 1 < cols && d[i + cols + 1] >= 0.f)
      d[i + cols + 1] += err * (1.f / 16.f);
  }
}

/* Step 2 for one cell: pick the glyph for this brightness, push the rounding
 * error onto the neighbours, and draw it in the theme colour.  An empty cell
 * draws a blank only if it was lit last frame — that's how we erase smoke that
 * just cleared without repainting the whole background. */
static inline void render_cell_emit(int x, int y, int i, float v,
                                    float *scratch, const float *prev, int cols,
                                    int rows, int theme) {
  if (v < 0.f) {
    if (prev[i] > 0.f)
      mvaddch(y, x, ' ');
    return;
  }
  int idx = lut_index(v);
  float qv = lut_midpoint(idx);
  float err = v - qv;
  floyd_steinberg_diffuse(scratch, i, x, y, cols, rows, err);

  attr_t attr = ramp_attr(idx, theme);
  attron(attr);
  mvaddch(y, x, (chtype)(unsigned char)k_ramp[idx]);
  attroff(attr);
}

/* Step 3: remember this frame's grid as "last frame" for the next erase pass.
 * We swap the two buffers, then copy the values back so the next tick still
 * has the current smoke to work from. */
static void render_swap_density_snapshot(Scene *sc) {
  int cols = sc->cols, rows = sc->rows;
  float *tmp = sc->prev_density;
  sc->prev_density = sc->density;
  sc->density = tmp;
  memcpy(sc->density, sc->prev_density, (size_t)(cols * rows) * sizeof(float));
}

/* Draw the whole smoke field (same renderer as fire.c).  Three steps:
 * fix the brightness, then for each visible cell pick a glyph and dither it,
 * then save the grid for next frame's erase pass.  The brightness fix gives
 * the mid-thick smoke the most glyph variety, and the dithering smooths the
 * shading so the nine glyphs don't show as hard bands. */
static void scene_draw(Scene *sc, int tcols, int trows) {
  int cols = sc->cols, rows = sc->rows;
  float *scratch = sc->work; /* the method already copied its result out */
  float *prev = sc->prev_density;

  render_density_to_gamma(sc->density, scratch, cols, rows);

  for (int y = 0; y < rows && y < trows; y++) {
    for (int x = 0; x < cols && x < tcols; x++) {
      int i = y * cols + x;
      float v = scratch[i];
      render_cell_emit(x, y, i, v, scratch, prev, cols, rows, sc->theme);
    }
  }

  render_swap_density_snapshot(sc);
}

/* ── §11 screen — the ncurses layer and the HUD ── */

typedef struct {
  int cols, rows;
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

static const char *algo_name(int a) {
  switch (a) {
  case 0:
    return "particle";
  case 1:
    return "vortex";
  case 2:
    return "curl";
  case 3:
    return "buoy";
  case 4:
    return "breeze";
  default:
    return "?";
  }
}

/* Draw the smoke, then lay the two HUD bars on top: the status line across
 * the top (paused/running, method, theme, source, wind, fps) and the key
 * list across the bottom.  Both bars fill their whole row with colour and go
 * down after the smoke, so no smoke shows through them. */
static void screen_draw(Screen *s, Scene *sc, double fps, int sfps) {
  if (sc->needs_clear) {
    erase();
    sc->needs_clear = false;
  }
  scene_draw(sc, s->cols, s->rows);

  /* ── top row: live status ── */
  const char *wstr = sc->wind > 0 ? ">>>" : sc->wind < 0 ? "<<<" : "---";

  char status[200];
  snprintf(status, sizeof status,
           " SMOKE   %s   algo:%-7s   theme:%-7s   src:%.2f   "
           "wind:%s (%+d)   %5.1f fps  %3d Hz ",
           sc->paused ? "PAUSED " : "running", algo_name(sc->algo),
           k_themes[sc->theme].name, sc->source, wstr, sc->wind, fps, sfps);

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  for (int x = 0; x < s->cols; x++)
    mvaddch(0, x, ' ');
  mvprintw(0, 0, "%s", status);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* ── bottom row: every key you can press ── */
  const char *hints = " q:quit  spc:pause  a/A:algo  t/T:theme  g/G:source  "
                      "w/W:wind  0:calm  ]/[:Hz ";

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

/* ── §12 app — the main loop ── */

typedef struct {
  Scene scene;
  Screen screen;
  int sim_fps;
  volatile sig_atomic_t running, need_resize;
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

static bool app_handle_key(App *a, int ch) {
  Scene *sc = &a->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    sc->paused = !sc->paused;
    break;

  case 'a':
  case 'A':
    sc->algo = (sc->algo + 1) % N_ALGOS;
    sc->warmup = 0;
    sc->wind_acc = 0;
    sc->needs_clear = true;
    memset(sc->parts, 0, sizeof sc->parts);
    sc->part_idx = 0;
    break;

  case 't':
  case 'T':
    sc->theme = (sc->theme + 1) % THEME_COUNT;
    theme_apply(sc->theme);
    sc->needs_clear = true;
    break;

  case 'g':
    sc->source += 0.05f;
    if (sc->source > 1.0f)
      sc->source = 1.0f;
    break;
  case 'G':
    sc->source -= 0.05f;
    if (sc->source < 0.1f)
      sc->source = 0.1f;
    break;

  case 'w':
    sc->wind++;
    if (sc->wind > WIND_MAX)
      sc->wind = WIND_MAX;
    break;
  case 'W':
    sc->wind--;
    if (sc->wind < -WIND_MAX)
      sc->wind = -WIND_MAX;
    break;
  case '0':
    sc->wind = 0;
    sc->wind_acc = 0;
    break;

  case ']':
    a->sim_fps += SIM_FPS_STEP;
    if (a->sim_fps > SIM_FPS_MAX)
      a->sim_fps = SIM_FPS_MAX;
    break;
  case '[':
    a->sim_fps -= SIM_FPS_STEP;
    if (a->sim_fps < SIM_FPS_MIN)
      a->sim_fps = SIM_FPS_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned int)clock_ns());
  atexit(cleanup);
  signal(SIGINT, on_exit);
  signal(SIGTERM, on_exit);
  signal(SIGWINCH, on_resize);

  App *app = &g_app;
  app->running = 1;
  app->sim_fps = SIM_FPS_DEFAULT;

  screen_init(&app->screen, 0);
  scene_init(&app->scene, app->screen.cols, app->screen.rows, 0, 0);

  int64_t ft = clock_ns(), sa = 0, fa = 0;
  int fc = 0;
  double fpsd = 0.0;

  while (app->running) {
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      ft = clock_ns();
      sa = 0;
    }

    int64_t now = clock_ns(), dt = now - ft;
    ft = now;
    if (dt > 100 * NS_PER_MS)
      dt = 100 * NS_PER_MS;

    int64_t tick = TICK_NS(app->sim_fps);
    sa += dt;
    while (sa >= tick) {
      scene_tick(&app->scene);
      sa -= tick;
    }

    fc++;
    fa += dt;
    if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
      fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
      fc = 0;
      fa = 0;
    }

    int64_t el = clock_ns() - ft + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - el);

    screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
    screen_present();

    int ch;
    while ((ch = getch()) != ERR)
      if (!app_handle_key(app, ch)) {
        app->running = 0;
        break;
      }
  }

  scene_free_bufs(&app->scene);
  screen_free(&app->screen);
  return 0;
}
