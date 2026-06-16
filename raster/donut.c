/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * donut.c — a spinning ASCII donut (Andy Sloane's classic torus), lit by a fixed
 * light. It samples thousands of points straight off the donut's surface with a
 * formula (no triangles, no ray-tracing), projects each onto the screen, and a
 * depth check keeps only the nearest dot at each cell.
 *
 * Keys: q quit · space pause · ]/[ faster/slower · =/- bigger/smaller
 *       t/T theme · d/D debug overlay (normal / depth / wire / no-zbuf)
 * The math and the original walkthrough: Andy Sloane, "Donut math"
 *   https://www.a1k0n.net/2011/07/20/donut-math.html
 * Build: gcc -std=c11 -O2 -Wall -Wextra raster/donut.c -o donut -lncurses -lm
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

/* ── §1 config — all the tunable numbers in one place ──────────────────── */

/* §1.1 — frame rate + UI layout. */
enum {
  SIM_FPS_DEFAULT = 30,
  FPS_UPDATE_MS = 500,
  HUD_STATUS_COLS = 90,
};

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(fps) (NS_PER_SEC / (fps))

/* §1.2 — angles and π helpers. */
#define TWO_PI (2.0f * (float)M_PI)

/* §1.3 — how fast it turns (radians per second). It tumbles forward (angle A)
 * and spins in place (angle B) at the same time. ]/[ scale both together, so
 * the tumble-to-spin look stays the same. */
#define ROT_RATE_A_DEFAULT 1.2f
#define ROT_RATE_B_DEFAULT 0.6f
#define SPEED_SCALE 1.3f
#define SPEED_MIN 0.05f
#define SPEED_MAX 12.0f

/* §1.4 — the donut's shape.
 *   TORUS_R1  how fat the tube is
 *   TORUS_R2  how far the tube's centre is from the middle — this sets the hole:
 *             the hole is R2−R1 across, the whole donut is R2+R1
 *   TORUS_K2  how far away it sits, so it's in front of the camera (never behind) */
#define TORUS_R1 1.0f
#define TORUS_R2 2.0f
#define TORUS_K2 5.0f

/* §1.5 — how big it's drawn. The scale is worked out each frame so the donut
 * fills a comfortable fraction of the screen; the +/- keys nudge it bigger or
 * smaller around that. */
#define TORUS_TARGET_FILL 0.42f
#define TORUS_SIZE_SCALE 1.15f
#define TORUS_SIZE_MIN 0.30f
#define TORUS_SIZE_MAX 5.00f

/* §1.6 — how densely we sample the surface.
 *
 * We trace points around the tube and around the big ring. The big ring is
 * longer, so it needs more points for the same on-screen density (its step is
 * the finer one). These are the COARSEST steps, fine enough when the donut is
 * small. As the donut grows, points spread apart and gaps would speckle through;
 * torus_render shrinks the step to keep the on-screen gap about SAMPLE_SPACING
 * cells, but never finer than the floors below (which cap the work at huge zoom).
 */
#define SAMPLE_SPACING 0.7f   /* aim for this gap between points, in cells (<1 = no holes) */
#define THETA_STEP_MIN 0.020f /* finest tube step — caps the work when zoomed way in */
#define PHI_STEP_MIN 0.007f   /* finest ring step */
#define THETA_STEP 0.07f
#define PHI_STEP 0.02f

/* §1.7 — a terminal cell is about twice as tall as it is wide, so we squash the
 * vertical by this much to keep the donut round instead of a tall oval. Use 1.0
 * on a square-cell terminal. */
#define Y_ASPECT 0.5f

/* §1.8 — the brightness-to-character ladder, 12 characters dim → bright. */
static const char LUMI_RAMP[] = ".,-~:;=!*#$@";
#define LUMI_RAMP_LEN ((int)(sizeof LUMI_RAMP - 1)) /* = 12 */

/* §1.8b — lighting, tuned for the coarse terminal grid.
 *
 * Plain lighting would leave everything facing away from the light pitch black —
 * about half the donut would vanish, which looks bad this coarse. So we wrap the
 * light a bit past the edge of shadow: the far side keeps a faint gradient and
 * the whole donut reads as one solid shape. (It also means every covered cell
 * competes for the depth test, so a nearer-but-darker point can't be replaced by
 * a farther lit one showing through.)
 *   INV_SQRT2     scales the raw lighting number into a tidy -1..1 range
 *   LIGHT_AMBIENT a brightness floor, so even the darkest spot isn't fully black */
#define INV_SQRT2 0.70710678f
#define LIGHT_AMBIENT 0.07f

/* §1.8c — a fixed 4×4 nudge pattern added to each cell's brightness before
 * picking a character. With only 12 characters a smooth gradient would show
 * banding; the nudge makes neighbours straddle the band edge so the eye blends
 * them. (This smooths the shading — it does NOT fill coverage gaps; that's the
 * adaptive sampling above.) The nudge is about one character-step wide. */
static const float DITHER_BAYER[4][4] = {
    {0 / 16.f, 8 / 16.f, 2 / 16.f, 10 / 16.f},
    {12 / 16.f, 4 / 16.f, 14 / 16.f, 6 / 16.f},
    {3 / 16.f, 11 / 16.f, 1 / 16.f, 9 / 16.f},
    {15 / 16.f, 7 / 16.f, 13 / 16.f, 5 / 16.f},
};
#define DITHER_STRENGTH (1.0f / (float)LUMI_RAMP_LEN)

/* §1.9 — our colour slots: 8 brightness shades (their actual colours come from
 * the theme), plus a yellow for the status bar and a cyan for the hint line. */
enum {
  LUMI_LEVELS = 8,
  PAIR_LUMI_BASE = 1,                      /* +0..+7 */
  PAIR_HUD = PAIR_LUMI_BASE + LUMI_LEVELS, /* = 9  */
  PAIR_HINT = PAIR_HUD + 1,                /* = 10 */
};

/* §1.10 — colour themes. A theme is just 8 colours, dim → bright; switching
 * theme (t/T) just re-points the 8 brightness slots at a new set of colours,
 * nothing else. Every colour is kept bright enough to show on a black
 * background, even the "darkest" one. CLASSIC is the default. */
typedef struct {
  const char *display_name;    /* the HUD label, padded to a fixed width  */
  short ramp_256[LUMI_LEVELS]; /* 8 colours, dimmest slot → brightest slot */
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    /* CLASSIC — eight greys in the bright half (240..255).
     * The original donut.c look — neutral, professional. */
    {"CLASSIC ", {240, 243, 246, 248, 250, 252, 254, 255}},

    /* AMBER — old phosphor-monitor amber, deep brown → bright gold.
     * Channels the 1980s green-screen-but-orange aesthetic. */
    {"AMBER   ", {130, 136, 166, 172, 178, 208, 214, 220}},

    /* MATRIX — eight green shades, deep moss → lime.  Cyberpunk. */
    {"MATRIX  ", {28, 34, 40, 46, 82, 118, 154, 190}},

    /* NEON — magenta → pink → fuchsia → cream-pink.  Synthwave. */
    {"NEON    ", {53, 91, 129, 165, 201, 207, 213, 227}},

    /* ICE — navy → bright cyan.  Cool / cold-storage look. */
    {"ICE     ", {25, 31, 38, 45, 51, 87, 123, 159}},

    /* COPPER — bronze → orange → amber.  Warm metallic. */
    {"COPPER  ", {94, 130, 136, 166, 172, 208, 214, 220}},
};

/* §1.11 — the drawing grid is a fixed size; a terminal bigger than this just
 * gets quietly clipped (512×256 covers any realistic one, no malloc needed). */
#define TORUS_MAX_COLS 512
#define TORUS_MAX_ROWS 256
#define TORUS_MAX_CELLS (TORUS_MAX_COLS * TORUS_MAX_ROWS)

/* A depth cell at or below this counts as empty (cleared to 0; any real point is
 * above it). The depth overlay uses it to skip empty cells. */
#define ZBUF_EMPTY_EPS 1e-6f

/* §1.12 — the debug overlays (press 'd' to cycle). Each one shows a different
 * inside view of how the donut is drawn:
 *   NORMAL   the real lit donut
 *   DEPTH    colour by nearness — shows what the depth check is keeping
 *   WIRE     draw only every Nth point — shows the dotted sampling grid
 *   NO_ZBUF  skip the depth check — the back shows through the front
 * Numbered from 0 in a row so 'd'/'D' can step through and they double as labels. */
typedef enum {
  DEBUG_NORMAL = 0,
  DEBUG_DEPTH = 1,
  DEBUG_WIRE = 2,
  DEBUG_NO_ZBUF = 3,
  DEBUG_MODE_COUNT = 4, /* how many there are (for cycling) */
} DebugMode;

/* HUD label per overlay, indexed by DebugMode (space-padded to equal width). */
static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL ",
    "DEPTH  ",
    "WIRE   ",
    "NO_ZBUF",
};

#define DEBUG_WIRE_STRIDE 10 /* paint 1 in every 10 samples */

/* ── §2 clock — reading the time and sleeping ──────────────────────────── */
/* A steady clock that only ever counts up (no jumps from the system clock being
 * changed) — what you want for "how long since the last frame". */
static int64_t clock_ns(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t nanoseconds) {
  if (nanoseconds <= 0)
    return;
  struct timespec request = {
      .tv_sec = (time_t)(nanoseconds / NS_PER_SEC),
      .tv_nsec = (long)(nanoseconds % NS_PER_SEC),
  };
  nanosleep(&request, NULL);
}

/* ── §3 color — the brightness slots, themes, and HUD colours ──────────── */

/* Points the 8 brightness slots at the chosen theme's colours. Called at
 * startup and on each t/T. On an old 8-colour terminal themes do nothing — we
 * fake the brightness levels with dim/bold instead (see lumi_attr). */
static void theme_apply(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *theme = &THEMES[theme_index];

  if (COLORS >= 256) {
    for (int slot = 0; slot < LUMI_LEVELS; slot++)
      init_pair((short)(PAIR_LUMI_BASE + slot), theme->ramp_256[slot], -1);
  } else {
    /* 8-colour fallback — themes have no effect; we layer
     * A_DIM/A_BOLD via lumi_attr to fake brightness levels. */
    for (int slot = 0; slot < LUMI_LEVELS; slot++)
      init_pair((short)(PAIR_LUMI_BASE + slot), COLOR_WHITE, -1);
  }
}

static void color_init(void) {
  start_color();
  use_default_colors();

  if (COLORS >= 256) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
  theme_apply(0); /* default to CLASSIC */
}

/* The ncurses style for one brightness slot. On a 256-colour terminal each slot
 * is its own colour; on an 8-colour one we layer dim/normal/bold on white so the
 * 8 slots still come out as three tellable-apart brightnesses. */
static attr_t lumi_attr(int slot) {
  if (slot < 0)
    slot = 0;
  if (slot > LUMI_LEVELS - 1)
    slot = LUMI_LEVELS - 1;

  attr_t attr = COLOR_PAIR(PAIR_LUMI_BASE + slot);
  if (COLORS < 256) {
    if (slot < 3)
      attr |= A_DIM;
    else if (slot > 5)
      attr |= A_BOLD;
  }
  return attr;
}

/* Keeps a character index inside the ramp. */
static inline int clamp_ramp_slot(int slot) {
  if (slot < 0)
    return 0;
  if (slot >= LUMI_RAMP_LEN)
    return LUMI_RAMP_LEN - 1;
  return slot;
}

/* Picks a character (0..11) for a brightness 0..1. */
static inline int ramp_slot(float brightness) {
  return clamp_ramp_slot((int)(brightness * (float)LUMI_RAMP_LEN));
}

/* Maps a character index (12 of them) down to a colour slot (8 of them), so the
 * 12 characters share the 8 colours. */
static inline int lumi_slot_for(int glyph_slot) {
  return (glyph_slot * LUMI_LEVELS) / LUMI_RAMP_LEN;
}

/* ── §4 math — small point types and rotation helper ───────────────────── */

typedef struct {
  float x, y;
} V2;
typedef struct {
  float x, y, z;
} V3;

/*
 * Rot — the donut's current tilt, boiled down to four numbers: the sines and
 * cosines of the two turn angles (the forward tumble and the in-place spin).
 * These are the only things that change frame to frame in the surface and
 * lighting formulas. We work them out ONCE per frame and pass them along,
 * because the inner loop runs tens of thousands of times and recomputing sin/cos
 * inside it would be a huge waste.
 */
typedef struct {
  float sin_A, cos_A; /* the forward-tumble angle */
  float sin_B, cos_B; /* the in-place-spin angle  */
} Rot;

static inline V2 v2(float x, float y) { return (V2){x, y}; }

static inline V3 v3(float x, float y, float z) { return (V3){x, y, z}; }

static inline Rot rot_make(float angle_A, float angle_B) {
  return (Rot){
      .sin_A = sinf(angle_A),
      .cos_A = cosf(angle_A),
      .sin_B = sinf(angle_B),
      .cos_B = cosf(angle_B),
  };
}

static inline float clampf(float x, float lo, float hi) {
  return x < lo ? lo : x > hi ? hi : x;
}

/* ── §5 framebuffer — where we draw before showing it ──────────────────── */

/*
 * Framebuffer — our own off-screen page, the size of the terminal. Three grids,
 * one entry per cell:
 *   zbuf   how near the nearest point landed here so far (0 = empty)
 *   glyph  the character to print (' ' = empty)
 *   luma   which colour slot, stored alongside so the painter needn't re-derive it
 *
 * zbuf is a depth check ("z-buffer") that hides farther points behind nearer
 * ones (Catmull 1974). One twist: it stores NEARNESS, not distance — bigger
 * means closer, and 0 means empty — because the projection step hands us nearness
 * for free. So the test keeps a point only if its nearness beats what's stored.
 * The grids are a fixed size; fb_set_dims clamps the live size to fit.
 */
typedef struct {
  int cols, rows;             /* live size in cells (clamped to fit)        */
  float zbuf[TORUS_MAX_CELLS];   /* nearness per cell: bigger = closer        */
  char glyph[TORUS_MAX_CELLS];   /* the character to print (' ' = empty)     */
  uint8_t luma[TORUS_MAX_CELLS]; /* colour slot per cell                     */
} Framebuffer;

/*
 * Torus — everything about the spinning donut that changes, in three groups:
 *   how it's turned : the two angles and how fast they grow — this IS the whole
 *                     "simulation" (§7 advances it)
 *   display knobs   : what the keys change about how it's drawn
 *   fb              : the page it draws into
 * The donut's shape (R1/R2/K2) is fixed back in §1, so the only thing that
 * actually changes here is how it's turned.
 */
typedef struct {
  /* how it's turned (advanced by torus_tick) */
  float angle_A;    /* how far it's tumbled forward         */
  float angle_B;    /* how far it's spun in place           */
  float rot_rate_A; /* tumble speed ('[' / ']')             */
  float rot_rate_B; /* spin speed                           */
  bool paused;      /* space freezes it                     */

  /* display knobs (don't affect the turning) */
  float k1_user_scale;  /* size multiplier ('=' / '-')      */
  int theme_index;      /* which colour theme (t/T)         */
  DebugMode debug_mode; /* which debug overlay (d/D)        */

  Framebuffer fb;
} Torus;

/* Stores the live size, clamped so it can't exceed the fixed grid — without this
 * a terminal bigger than the grid would run the writes off the end. The donut
 * just stops growing past that size. */
static void fb_set_dims(Framebuffer *fb, int cols, int rows) {
  fb->cols = cols > TORUS_MAX_COLS ? TORUS_MAX_COLS : cols;
  fb->rows = rows > TORUS_MAX_ROWS ? TORUS_MAX_ROWS : rows;
}

/* Wipes the page for a new frame: every cell empty and "infinitely far". */
static void fb_clear(Framebuffer *fb) {
  int total_cells = fb->cols * fb->rows;
  memset(fb->zbuf, 0, sizeof(float) * (size_t)total_cells);
  memset(fb->glyph, ' ', sizeof(char) * (size_t)total_cells);
  memset(fb->luma, 0, sizeof(uint8_t) * (size_t)total_cells);
}

/* Sets the donut's starting angles, speeds, and knobs, and sizes the page. */
static void torus_init(Torus *torus, int cols, int rows) {
  memset(torus, 0, sizeof *torus);
  torus->rot_rate_A = ROT_RATE_A_DEFAULT;
  torus->rot_rate_B = ROT_RATE_B_DEFAULT;
  torus->k1_user_scale = 1.0f;
  torus->theme_index = 0; /* CLASSIC */
  torus->debug_mode = DEBUG_NORMAL;
  fb_set_dims(&torus->fb, cols, rows);
}

/* ── §6 perspective — work out how big to draw the donut ───────────────── */

/* Picks the on-screen scale so the donut fills a comfortable fraction of the
 * shorter side of the window, times the user's size knob. Redone every frame, so
 * resizing the window resizes the donut automatically. */
static float compute_k1(const Torus *torus) {
  float half_width = (float)torus->fb.cols * 0.5f;
  float full_height = (float)torus->fb.rows;
  float min_half_dim = (half_width < full_height ? half_width : full_height);
  float target_pixels = min_half_dim * TORUS_TARGET_FILL;

  float k1 =
      target_pixels * (TORUS_K2 + TORUS_R2 + TORUS_R1) / (TORUS_R2 + TORUS_R1);
  return k1 * torus->k1_user_scale;
}

/* ── §7 tick — turn the donut a little ─────────────────────────────────── */

/* The entire animation: nudge both angles forward by their speed × elapsed time.
 * (Frozen while paused — the donut still draws, just held still.) */
static void torus_tick(Torus *torus, float dt_seconds) {
  if (torus->paused)
    return;
  torus->angle_A += torus->rot_rate_A * dt_seconds;
  torus->angle_B += torus->rot_rate_B * dt_seconds;
}

/* ── §8 tube point — a point around the tube's cross-section ───────────── */

/* Picks one point on the small circle that forms the tube, before it's wrapped
 * around into 3-D. The caller passes the angle's cos/sin since it reuses the same
 * tube angle for many points around the ring. */
static V2 tube_point(float cos_theta, float sin_theta) {
  return v2(TORUS_R2 + TORUS_R1 * cos_theta, TORUS_R1 * sin_theta);
}

/* ── §9 surface point — that point's full 3-D spot, turned and placed ───── */

/* Takes a tube point and works out where it actually sits in space: wrap it
 * around the big ring, then apply the donut's two turns (the forward tumble and
 * the in-place spin), then push it out in front of the camera. The long
 * expression is those steps multiplied out into one (same as Sloane's original).
 * cos/sin of the ring angle are passed in because they're reused for the
 * lighting too. */
static V3 surface_point(V2 tube, float cos_phi, float sin_phi, Rot rot) {
  return v3(tube.x * (rot.cos_B * cos_phi + rot.sin_A * rot.sin_B * sin_phi) -
                tube.y * rot.cos_A * rot.sin_B,

            tube.x * (rot.sin_B * cos_phi - rot.sin_A * rot.cos_B * sin_phi) +
                tube.y * rot.cos_A * rot.cos_B,

            TORUS_K2 + rot.cos_A * tube.x * sin_phi + tube.y * rot.sin_A);
}

/* ── §10 project to screen — drop a 3-D point onto the cell grid ───────── */

/* ScreenPos — where a point landed: its cell, a nearness value (1 over distance,
 * so bigger = closer), and whether it's even on screen. Returned together so the
 * caller gets the spot and its nearness in one go. */
typedef struct {
  int col;          /* column it lands in (valid when in_bounds)   */
  int row;          /* row it lands in (valid when in_bounds)      */
  float one_over_z; /* nearness — bigger = closer (the depth check uses it) */
  bool in_bounds;   /* false = off-screen; caller skips it          */
} ScreenPos;

/* Divides by distance so far things land smaller, then shifts to screen cells.
 * The row is flipped (screen rows count down, but up should be up) and squashed
 * by Y_ASPECT so the donut stays round on tall cells. */
static ScreenPos project_to_screen(V3 world, int cols, int rows, float K1) {
  float one_over_z = 1.0f / world.z;
  int col = (int)((float)cols * 0.5f + K1 * one_over_z * world.x);
  int row = (int)((float)rows * 0.5f - K1 * one_over_z * world.y * Y_ASPECT);
  bool in_bounds = (col >= 0 && col < cols && row >= 0 && row < rows);
  return (ScreenPos){col, row, one_over_z, in_bounds};
}

/* ── §11 surface luminance — how lit each point is ─────────────────────── */

/* How brightly a point is lit: how much the surface there faces the light. The
 * light sits above and behind the camera, fixed. Because we know the donut's
 * shape exactly, this collapses to one tidy expression (no per-point matrix
 * work). Comes out roughly -1.4..1.4 — negatives mean facing away; shade_brightness
 * turns it into a 0..1 display brightness. */
static float surface_luminance(float cos_theta, float sin_theta, float cos_phi,
                               float sin_phi, Rot rot) {
  return cos_phi * cos_theta * rot.sin_B - rot.cos_A * cos_theta * sin_phi -
         rot.sin_A * sin_theta +
         rot.cos_B * (rot.cos_A * sin_theta - cos_theta * rot.sin_A * sin_phi);
}

/* Turns the raw lighting number into a 0..1 brightness, tuned for the coarse
 * grid: scale it into range, wrap the light a bit past the shadow edge so the
 * far side keeps a gradient (instead of going black), square it for punch, and
 * floor it so the darkest spot is still faintly lit. The whole donut then has a
 * smooth gradient and reads as solid rather than a half-lit crescent. */
static float shade_brightness(float raw_dot) {
  float ndl = raw_dot * INV_SQRT2;
  float wrap = 0.5f * ndl + 0.5f;
  float lit = wrap * wrap;
  return LIGHT_AMBIENT + (1.0f - LIGHT_AMBIENT) * lit;
}

/* ── §12 emit one point — depth check, then store the character ────────── */

/* Tries to place one lit point into its cell. Skips it if it's off-screen, or if
 * a nearer point already owns the cell. Otherwise it shades it, nudges the
 * brightness for dithering, picks a character, and stores it as the new nearest.
 * (No light-based skip — even the dim far side fills its cell, so the nearest
 * point always wins.) force_overwrite is only for the no-depth debug overlay. */
static void try_emit_pixel(Framebuffer *fb, ScreenPos sp, float L,
                           bool force_overwrite) {
  if (!sp.in_bounds)
    return;

  int cell_index = sp.row * fb->cols + sp.col;
  if (!force_overwrite && sp.one_over_z <= fb->zbuf[cell_index])
    return; /* a nearer point already owns this cell */

  /* shade it, nudge for dithering, then pick a character */
  float brightness = shade_brightness(L);
  brightness += (DITHER_BAYER[sp.row & 3][sp.col & 3] - 0.5f) * DITHER_STRENGTH;
  int glyph_slot = ramp_slot(brightness);

  /* this point wins the cell: store its nearness, character, and colour slot */
  fb->zbuf[cell_index] = sp.one_over_z;
  fb->glyph[cell_index] = LUMI_RAMP[glyph_slot];
  fb->luma[cell_index] = (uint8_t)lumi_slot_for(glyph_slot);
}

/* ── §13 render — walk the whole surface and draw every point ──────────── */

/* Picks how big a step to take around the tube and around the ring so the points
 * land about SAMPLE_SPACING cells apart on screen — which is what stops gaps from
 * speckling through as the donut grows. The ring is bigger, so it gets the finer
 * step. Kept between the coarse default and a fine floor (the floor caps the work
 * when zoomed way in). */
static void sample_steps(float K1, float *theta_step, float *phi_step) {
  float closest_z = TORUS_K2 - (TORUS_R2 + TORUS_R1);
  float cells_per_unit = K1 / closest_z;

  float ts = SAMPLE_SPACING / (TORUS_R1 * cells_per_unit);
  float ps = SAMPLE_SPACING / ((TORUS_R2 + TORUS_R1) * cells_per_unit);

  /* Clamp to [finest cost cap, coarsest base step]. */
  *theta_step = clampf(ts, THETA_STEP_MIN, THETA_STEP);
  *phi_step = clampf(ps, PHI_STEP_MIN, PHI_STEP);
}

/* Draws the donut into the page: walk around the tube and around the ring, and
 * for every point work out its 3-D spot, where it lands on screen, how lit it is,
 * and try to place it. (wire_stride > 1 draws only every Nth point, for the wire
 * overlay; force_overwrite skips the depth check, for the no-depth overlay.) */
static void torus_render(Torus *torus, int wire_stride, bool force_overwrite) {
  fb_clear(&torus->fb);

  Rot rot = rot_make(torus->angle_A, torus->angle_B);
  float K1 = compute_k1(torus);

  float theta_step, phi_step;
  sample_steps(K1, &theta_step, &phi_step);

  int sample_index = 0;
  for (float theta = 0.f; theta < TWO_PI; theta += theta_step) {
    float cos_theta = cosf(theta);
    float sin_theta = sinf(theta);
    V2 tube = tube_point(cos_theta, sin_theta);

    for (float phi = 0.f; phi < TWO_PI; phi += phi_step) {
      sample_index++;

      /* wire overlay: skip most points to expose the dotted grid */
      if (wire_stride > 1 && (sample_index % wire_stride) != 0)
        continue;

      float cos_phi = cosf(phi);
      float sin_phi = sinf(phi);

      V3 world = surface_point(tube, cos_phi, sin_phi, rot);
      ScreenPos sp =
          project_to_screen(world, torus->fb.cols, torus->fb.rows, K1);
      float L = surface_luminance(cos_theta, sin_theta, cos_phi, sin_phi, rot);

      try_emit_pixel(&torus->fb, sp, L, force_overwrite);
    }
  }
}

/* ── §14 draw — copy the page to the terminal ──────────────────────────── */

/* Walks every cell and prints the non-empty ones in their stored colour. */
static void fb_draw(const Framebuffer *fb, WINDOW *window) {
  int cols = fb->cols;
  int total_cells = cols * fb->rows;

  for (int idx = 0; idx < total_cells; idx++) {
    char glyph = fb->glyph[idx];
    if (glyph == ' ')
      continue;

    attr_t attr = lumi_attr(fb->luma[idx]);
    wattron(window, attr);
    mvwaddch(window, idx / cols, idx % cols, (chtype)(unsigned char)glyph);
    wattroff(window, attr);
  }
}

/* ── §15 debug overlays — inside views of how the donut is drawn ───────── */

/* The closest point's nearness in the whole page (the biggest value), used to
 * scale the depth overlay. Returns 0 for an empty page. */
static float fb_max_depth(const Framebuffer *fb) {
  int total_cells = fb->cols * fb->rows;
  float max_one_over_z = 0.0f;
  for (int idx = 0; idx < total_cells; idx++)
    if (fb->zbuf[idx] > max_one_over_z)
      max_one_over_z = fb->zbuf[idx];
  return max_one_over_z;
}

/* DEPTH overlay: a normal render, then re-colour each cell by how near it is —
 * so you literally see what the depth check kept. */
static void render_debug_depth(Torus *torus, WINDOW *window) {
  torus_render(torus, 1, false); /* a normal render fills the nearness grid for us */
  const Framebuffer *fb = &torus->fb;

  float max_depth = fb_max_depth(fb);
  if (max_depth < ZBUF_EMPTY_EPS)
    return; /* empty frame */

  /* paint each filled cell brighter the nearer it is */
  int cols = fb->cols;
  int total_cells = cols * fb->rows;
  for (int idx = 0; idx < total_cells; idx++) {
    if (fb->zbuf[idx] < ZBUF_EMPTY_EPS)
      continue; /* empty cell */

    float closeness = fb->zbuf[idx] / max_depth; /* 0 = far … 1 = closest */
    int slot = clamp_ramp_slot((int)(closeness * (float)(LUMI_RAMP_LEN - 1) + 0.5f));

    char glyph = LUMI_RAMP[slot];
    attr_t attr = lumi_attr(lumi_slot_for(slot));
    wattron(window, attr);
    mvwaddch(window, idx / cols, idx % cols, (chtype)(unsigned char)glyph);
    wattroff(window, attr);
  }
}

/* WIRE overlay: draw only every Nth point, so you see the dotted sampling grid. */
static void render_debug_wire(Torus *torus, WINDOW *window) {
  torus_render(torus, DEBUG_WIRE_STRIDE, false);
  fb_draw(&torus->fb, window);
}

/* NO-DEPTH overlay: skip the depth check, so front and back both draw and the
 * back shows through — which is exactly what the depth check normally prevents. */
static void render_debug_no_zbuf(Torus *torus, WINDOW *window) {
  torus_render(torus, 1, true);
  fb_draw(&torus->fb, window);
}

/* Draws whichever overlay is currently selected. */
static void draw_active_view(Torus *torus, WINDOW *window) {
  switch (torus->debug_mode) {
  case DEBUG_NORMAL:
    torus_render(torus, 1, false);
    fb_draw(&torus->fb, window);
    break;
  case DEBUG_DEPTH:
    render_debug_depth(torus, window);
    break;
  case DEBUG_WIRE:
    render_debug_wire(torus, window);
    break;
  case DEBUG_NO_ZBUF:
    render_debug_no_zbuf(torus, window);
    break;
  default:
    torus_render(torus, 1, false);
    fb_draw(&torus->fb, window);
    break;
  }
}

/* ── §16 screen — ncurses setup, resize, HUD, present ──────────────────── */

/* Screen — the terminal's full size in cells. Separate from the page's size
 * (which is this clamped to fit the fixed grid): the HUD lines up against the
 * real terminal, while the donut draws into the possibly-smaller page. */
typedef struct {
  int cols, rows;
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  color_init();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_free(Screen *screen) {
  (void)screen;
  endwin();
}

static void screen_resize(Screen *screen) {
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

/* Draws a whole frame: the donut, then the overlay — a yellow status line across
 * the top and a cyan key reminder along the bottom. */
static void screen_draw(Screen *screen, Torus *torus, double fps) {
  erase();
  draw_active_view(torus, stdscr);

  /* top row — yellow status */
  char status[HUD_STATUS_COLS + 1];
  snprintf(status, sizeof status,
           " %5.1f fps  spd:%4.2f  size:%4.2f  theme:%s  debug:%s%s ", fps,
           (double)torus->rot_rate_A, (double)torus->k1_user_scale,
           THEMES[torus->theme_index].display_name,
           DEBUG_MODE_NAMES[torus->debug_mode],
           torus->paused ? "  PAUSED" : "");
  int slen = (int)strlen(status);
  if (slen > screen->cols)
    slen = screen->cols;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, screen->cols - slen, "%s", status);
  mvprintw(0, 0, " DONUT ");
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

  /* bottom row — cyan key hint */
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(screen->rows - 1, 0,
           " q:quit  spc:pause  ]/[:speed  +/-:size  t/T:theme  d/D:debug ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §17 app — setup, the main loop, and keypresses ────────────────────── */

/*
 * App — the whole running program: the donut, the terminal size, the fixed
 * tick rate, and two flags the signal handlers set. One shared instance (g_app)
 * so the handlers, which take no arguments, can reach it. The two flags are
 * marked volatile sig_atomic_t because a signal can set them at any moment, so
 * the compiler must always read/write them for real.
 */
typedef struct {
  Torus torus;   /* the spinning donut + its page              */
  Screen screen; /* terminal size                              */
  int sim_fps;   /* how many times per second it turns        */
  volatile sig_atomic_t running;     /* cleared to stop the loop          */
  volatile sig_atomic_t need_resize; /* set when the window was resized   */
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

/* Re-reads the new window size and resizes the page to match. */
static void app_do_resize(App *app) {
  screen_resize(&app->screen);
  fb_set_dims(&app->torus.fb, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

/* Handles one keypress; returns false to quit. Keys are listed in the header. */
static bool app_handle_key(App *app, int ch) {
  Torus *torus = &app->torus;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    torus->paused = !torus->paused;
    break;

  case ']': /* faster — speed both up, keeping their ratio */
    torus->rot_rate_A = clampf(torus->rot_rate_A * SPEED_SCALE, SPEED_MIN, SPEED_MAX);
    torus->rot_rate_B = clampf(torus->rot_rate_B * SPEED_SCALE, SPEED_MIN, SPEED_MAX);
    break;

  case '[': /* slower */
    torus->rot_rate_A = clampf(torus->rot_rate_A / SPEED_SCALE, SPEED_MIN, SPEED_MAX);
    torus->rot_rate_B = clampf(torus->rot_rate_B / SPEED_SCALE, SPEED_MIN, SPEED_MAX);
    break;

  case '=':
  case '+': /* grow */
    torus->k1_user_scale =
        clampf(torus->k1_user_scale * TORUS_SIZE_SCALE, TORUS_SIZE_MIN, TORUS_SIZE_MAX);
    break;

  case '-': /* shrink */
    torus->k1_user_scale =
        clampf(torus->k1_user_scale / TORUS_SIZE_SCALE, TORUS_SIZE_MIN, TORUS_SIZE_MAX);
    break;

  case 't':
    torus->theme_index = (torus->theme_index + 1) % THEME_COUNT;
    theme_apply(torus->theme_index);
    break;
  case 'T':
    torus->theme_index = (torus->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    theme_apply(torus->theme_index);
    break;

  case 'd':
    torus->debug_mode = (DebugMode)((torus->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    torus->debug_mode = (DebugMode)((torus->debug_mode + DEBUG_MODE_COUNT - 1) %
                                    DEBUG_MODE_COUNT);
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

  screen_init(&app->screen);
  torus_init(&app->torus, app->screen.cols, app->screen.rows);

  int64_t frame_time_ns = clock_ns();
  int64_t sim_accumulator_ns = 0;
  int64_t fps_accumulator_ns = 0;
  int frame_count = 0;
  double fps_display = 0.0;

  while (app->running) {

    /* handle a window resize if one happened */
    if (app->need_resize) {
      app_do_resize(app);
      frame_time_ns = clock_ns();
      sim_accumulator_ns = 0;
    }

    /* how long since the last frame (capped, so a hiccup doesn't lurch it) */
    int64_t now_ns = clock_ns();
    int64_t dt_ns = now_ns - frame_time_ns;
    frame_time_ns = now_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    /* turn the donut in fixed steps — catch up however many fit in dt, so the
     * spin speed is the same regardless of frame rate */
    int64_t tick_ns = TICK_NS(app->sim_fps);
    float dt_sec = (float)tick_ns / (float)NS_PER_SEC;

    sim_accumulator_ns += dt_ns;
    while (sim_accumulator_ns >= tick_ns) {
      torus_tick(&app->torus, dt_sec);
      sim_accumulator_ns -= tick_ns;
    }

    /* update the fps readout */
    frame_count++;
    fps_accumulator_ns += dt_ns;
    if (fps_accumulator_ns >= FPS_UPDATE_MS * NS_PER_MS) {
      fps_display = (double)frame_count /
                    ((double)fps_accumulator_ns / (double)NS_PER_SEC);
      frame_count = 0;
      fps_accumulator_ns = 0;
    }

    /* wait out the rest of the frame (before drawing, so output doesn't drift) */
    int64_t elapsed_ns = clock_ns() - frame_time_ns + dt_ns;
    clock_sleep_ns(NS_PER_SEC / 60 - elapsed_ns);

    /* draw the frame and show it */
    screen_draw(&app->screen, &app->torus, fps_display);
    screen_present();

    /* handle a keypress */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  screen_free(&app->screen);
  return 0;
}
