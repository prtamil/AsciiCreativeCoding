/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * aafire_port.c — aalib 1.4 fire cellular automaton, rebuilt on the
 *                 ncurses framework with a gamma → Floyd-Steinberg →
 *                 perceptual-LUT rendering pipeline.
 *
 * Heat lives in a uint8 grid; each cell reads a 5-neighbour sum from
 * the cells BELOW it (3 from one row below, 2 from two rows below — no
 * centre on the deeper row), passes it through a precomputed decay
 * table, and writes the cooled result back.  Two extra rows beneath
 * the visible area are FUEL — reseeded each tick in an arch-shaped
 * sweep so the flame stands tall in the middle.  The heat field is
 * rendered by gamma-correcting each byte, dithering with Floyd-
 * Steinberg error diffusion, looking the dithered float up in a
 * perceptual LUT to pick one of nine glyphs from ' ' to '@', then
 * painting that glyph in the active theme's colour pair.  10 themes;
 * four debug overlays expose raw heat / fuel-only / no-dither.
 *
 * Section map
 *   §1  config             constants — sim, fuel, timing, palette
 *   §2  clock              monotonic-ns timer + sleep
 *   §3  LUT                perceptual heat → ramp bucket
 *   §4  themes             10 colour palettes + theme_apply
 *   §5  bitmap state       clamp helpers + Bitmap struct
 *   §6  decay table        gentable() builds the per-row cooling LUT
 *   §7  propagation        firemain() applies the 5-neighbour stencil
 *   §8  fuel seeding       drawfire() seeds the bottom 2 rows in an arch
 *   §9  bitmap lifecycle   alloc / free / init / per-tick orchestrator
 *   §10 render pipeline    gamma → FS dither → LUT → paint
 *   §11 scene              orchestrator: pause, debug-mode, tick, draw
 *   §12 debug overlay      alternative render modes (raw / fuel / no-dither)
 *   §13 screen + HUD       ncurses init + HUD strip + hint strip
 *   §14 app                App struct + signal handlers + key dispatch
 *   §15 main               fixed-step main loop
 */

/* ── CONCEPTS & ALGORITHMS ───────────────────────────────────────────── *
 *
 *   Cellular automaton    5-neighbour stencil reading cells BELOW; one
 *                         cooled cell per step; rows propagate upward.
 *                         → Wolfram (1983), Rev. Mod. Phys. 55: 601
 *
 *   aalib fire CA         The exact stencil used by aalib 1.4 aafire.c.
 *                         3 cells from y+1, 2 from y+2 (no centre).
 *                         → Hubička, aalib 1.4 (1997-2001), aafire.c
 *
 *   Floyd-Steinberg       Error-diffusion dithering: 7/16, 3/16, 5/16,
 *                         1/16 distribution to forward neighbours.
 *                         → Floyd & Steinberg (1976), Proc SID 17(2): 75
 *
 *   Gamma correction      sRGB-style luma·^2.2 power-law transfer.
 *                         → ITU-R BT.709; Stevens power law for perception
 *
 *   Perceptual LUT        9 glyphs ' .:+x*X#@' partitioned by visual
 *                         density steps; non-uniform breakpoints.
 *                         → Stevens (1957), Psych. Rev. 64: 153
 *
 *   Decay table           Per-row cooling LUT precomputed once; indexed
 *                         by 5-neighbour sum (0..1275) → cooled byte.
 *                         → Toffoli & Margolus, Cellular Automata Machines
 *
 *   Fixed-step loop       sim_accum + dt-cap; deterministic physics
 *                         regardless of render Hz.
 *                         → Glenn Fiedler, "Fix Your Timestep!" (2004)
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── OVERALL PSEUDOCODE ──────────────────────────────────────────────── *
 *
 *   init:
 *     install signals; start ncurses (themes + HUD/HINT pairs)
 *     bitmap_init: alloc grid (rows + 2 fuel rows below); gentable
 *
 *   loop while running:
 *     if need_resize:        endwin → refresh → bitmap_realloc
 *     dt = clock_now − last_frame_time      (capped at 100 ms)
 *     while sim_accum ≥ tick_ns:
 *       scene_tick:           drawfire (seed fuel) + firemain (propagate)
 *                              + cycle theme every CYCLE_TICKS
 *       sim_accum -= tick_ns
 *     scene_draw:             dispatch on debug_mode →
 *                              bitmap_draw (full gamma+FS+LUT+paint)  OR
 *                              scene_draw_debug (raw / fuel / no-dither)
 *     screen_draw_hud_top_right + screen_draw_hint_bottom + present
 *     getch + app_handle_key  q/space/t/d/g/]
 *     sleep to ~60 fps
 *
 *   cleanup:
 *     atexit endwin restores the terminal
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── DRIVER PSEUDOCODE ───────────────────────────────────────────────── *
 *
 * bitmap_tick(b):                                       (CA per-step driver)
 *   drawfire(b)           seed bottom 2 fuel rows with arch-sweep bursts
 *   firemain(b)           5-neighbour-below propagation + decay table
 *   cycle_tick++;  if reached CYCLE_TICKS:
 *     theme = (theme + 1) mod THEME_COUNT;  theme_apply(theme)
 *     return true                          tells caller to clear screen
 *   return false
 *
 *   Order matters: drawfire must run BEFORE firemain so the new fuel
 *   bytes are visible to the propagation stencil's y+1 / y+2 reads.
 *
 *
 * firemain(b):                                          (the headline
 * algorithm) for each cell (y, x), top-left to bottom-right: sum =
 * sample_five_neighbours_below(bmap, cols, x, y) = bmap[y+1][x-1] +
 * bmap[y+1][x] + bmap[y+1][x+1]      (3 cells)
 *           + bmap[y+2][x-1]                + bmap[y+2][x+1]      (2 cells, no
 * centre) idx = min(sum, MAXTABLE - 1)                                paranoia
 * clamp bmap[y][x] = b->table[idx]                                  cooled byte
 *
 *   Skipping the centre cell on y+2 biases the stencil toward the SIDES
 *   of the cell directly below — that's what gives aafire its rounded
 *   blob shape rather than vertical streaks.  Edge columns clamp the
 *   horizontal neighbours to keep reads in-bounds.
 *
 *
 * scene_draw(s, cols, rows):                            (render dispatcher)
 *   if debug_mode == DEBUG_OFF:  bitmap_draw (full gamma+FS+LUT+paint)
 *   else:                        scene_draw_debug (raw heat / fuel / no-dither)
 *
 *   Two-line dispatcher; the heavy work lives in §10 (full pipeline) or
 *   §12 (debug overlays).  Splitting this way means turning on a debug
 *   mode never touches the production render code.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/*
 * Keys
 *   q | Q | ESC      quit                space      pause / resume
 *   t / T            next / prev theme
 *   d / D            next / prev debug mode
 *   g / G            fuel up / fuel down
 *   ] / [            sim Hz up / down
 *
 * Build
 *   gcc -std=c11 -O2 -Wall -Wextra particle_systems/aafire_port.c \
 *       -o aafire_port -lncurses -lm
 */

/* ── REFERENCES ──────────────────────────────────────────────────────── *
 *
 * PAPERS
 *   Floyd, R. W. & Steinberg, L. (1976)
 *     "An Adaptive Algorithm for Spatial Greyscale"
 *     Proceedings of SID 17(2): 75–77.   (Error-diffusion dither.)
 *   Wolfram, S. (1983)
 *     "Statistical mechanics of cellular automata"
 *     Reviews of Modern Physics 55: 601–644.
 *   Stevens, S. S. (1957)
 *     "On the psychophysical law"
 *     Psychological Review 64(3): 153–181.   (Perceptual power law.)
 *
 * BOOKS
 *   Toffoli, T. & Margolus, N. — "Cellular Automata Machines: A New
 *     Environment for Modeling"  (MIT Press, 1987)
 *     Ch. 4 covers neighbourhood stencils + table-driven CAs.
 *   Foley, J. et al. — "Computer Graphics: Principles and Practice"
 *     (Addison-Wesley, 2nd ed. 1990)
 *     §13.3.3 derives Floyd-Steinberg + alternative kernels.
 *   Press et al. — "Numerical Recipes in C"  (3rd ed., 2007)
 *     §17.x fixed-step integration; §15.x dithering primitives.
 *
 * ─────────────────────────────────────────────────────────────────────── */

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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
  SIM_FPS_MIN = 5,
  SIM_FPS_DEFAULT = 30,
  SIM_FPS_MAX = 60,
  SIM_FPS_STEP = 5,
  HUD_COLS = 52,
  FPS_UPDATE_MS = 500,
  CYCLE_TICKS = 300, /* ticks before auto-cycling theme          */
};

/*
 * aalib uses a 256-entry uint8 bitmap.
 * MAXTABLE must cover the maximum possible sum of 5 neighbours × 255 = 1275.
 * Original: MAXTABLE = 256*5 = 1280.  We keep the same.
 */
#define MAXTABLE 1280

/* Fuel intensity scale [0.1, 1.0] — multiplied against the base heat */
#define FUEL_DEFAULT 1.0f
#define FUEL_STEP 0.05f
#define FUEL_MIN 0.1f
#define FUEL_MAX 1.0f

#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(f) (NS_PER_SEC / (f))

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

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

/* ===================================================================== */
/* §3  LUT — perceptual heat → ramp bucket                                */
/* ===================================================================== */

/* k_ramp — 9 glyphs cold → hot.  Same alphabet as fire.c. */
static const char k_ramp[] = " .:+x*X#@";
#define RAMP_N (int)(sizeof k_ramp - 1)

/* HUD/HINT pairs sit ABOVE the theme palette so theme cycling
 * (CP_BASE..CP_BASE+RAMP_N-1) never clobbers them. */
#define PAIR_HUD (CP_BASE + RAMP_N)
#define PAIR_HINT (CP_BASE + RAMP_N + 1)

/* k_lut_breaks — perceptual thresholds; each bucket ≈ equal perceived
 * brightness. */
static const float k_lut_breaks[RAMP_N] = {
    0.000f, 0.080f, 0.180f, 0.290f, 0.390f, 0.500f, 0.620f, 0.750f, 0.900f,
};

/* lut_index — float in [0, 1] → bucket index in [0, RAMP_N). */
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

/* ===================================================================== */
/* §4  themes — 10 colour palettes                                        */
/* ===================================================================== */

/*
 * FireTheme — one palette: 9-entry ramp from cold (bucket 0) to hot (8).
 *
 *   name     cycled label, shown in HUD
 *   fg256    256-cube colour index per ramp slot (rich terminals)
 *   fg8      8-colour fallback per ramp slot
 *   attr8    A_DIM / A_NORMAL / A_BOLD per slot — expands the visible
 *            brightness range on basic-8 terminals
 *
 * Lifecycle: const table indexed by `Bitmap.theme`.  theme_apply()
 *   rebinds colour-pair IDs CP_BASE+0..+RAMP_N-1 to the chosen entry.
 *   HUD/HINT pairs (PAIR_HUD / PAIR_HINT) sit OUTSIDE that range so
 *   the HUD never changes colour as themes cycle.
 */
typedef struct {
  const char *name;
  int fg256[RAMP_N];
  int fg8[RAMP_N];
  attr_t attr8[RAMP_N];
} FireTheme;

#define CP_BASE 1

/* k_themes — 10 palettes (matrix / neon / nova / ocean / fire / toxic /
 * gold / ice / aurora / plasma).  fg256 indices 1..8 sit in the bright
 * half of the 256-cube (≥ 24); index 0 is the cold/empty slot whose
 * colour is never visible (k_ramp[0] == ' '). */
static const FireTheme k_themes[] = {
    {"matrix",
     {232, 22, 28, 34, 40, 46, 82, 118, 231},
     {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_GREEN, COLOR_GREEN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"neon",
     {232, 53, 89, 125, 161, 197, 213, 219, 231},
     {COLOR_BLACK, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_CYAN, COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_NORMAL, A_BOLD,
      A_BOLD}},
    {"nova",
     {232, 52, 88, 124, 160, 196, 208, 220, 231},
     {COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"ocean",
     {232, 17, 19, 21, 27, 33, 39, 123, 231},
     {COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN,
      COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_BOLD, A_BOLD,
      A_BOLD}},
    {"fire",
     {232, 52, 88, 124, 160, 196, 202, 214, 231},
     {COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_RED, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_BOLD,
      A_BOLD}},
    {"toxic",
     {232, 22, 28, 34, 106, 154, 190, 226, 231},
     {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD,
      A_BOLD}},
    {"gold",
     {232, 94, 130, 136, 172, 178, 214, 220, 231},
     {COLOR_BLACK, COLOR_RED, COLOR_RED, COLOR_YELLOW, COLOR_YELLOW,
      COLOR_YELLOW, COLOR_YELLOW, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"ice",
     {232, 17, 19, 21, 27, 33, 51, 123, 231},
     {COLOR_BLACK, COLOR_BLUE, COLOR_BLUE, COLOR_BLUE, COLOR_CYAN, COLOR_CYAN,
      COLOR_CYAN, COLOR_WHITE, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_DIM, A_NORMAL, A_BOLD, A_BOLD,
      A_BOLD}},
    {"aurora",
     {232, 22, 28, 34, 121, 159, 207, 219, 231},
     {COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_GREEN, COLOR_CYAN,
      COLOR_CYAN, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_BOLD, A_BOLD, A_BOLD, A_BOLD, A_BOLD,
      A_BOLD}},
    {"plasma",
     {232, 53, 91, 129, 165, 207, 213, 219, 231},
     {COLOR_BLACK, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA, COLOR_MAGENTA,
      COLOR_MAGENTA, COLOR_CYAN, COLOR_CYAN, COLOR_WHITE},
     {A_NORMAL, A_DIM, A_NORMAL, A_NORMAL, A_BOLD, A_BOLD, A_DIM, A_BOLD,
      A_BOLD}},
};
#define THEME_COUNT (int)(sizeof k_themes / sizeof k_themes[0])

/* theme_apply — rebind the RAMP_N ramp pair IDs to k_themes[t]. */
static void theme_apply(int t) {
  const FireTheme *th = &k_themes[t];
  for (int i = 0; i < RAMP_N; i++) {
    if (COLORS >= 256)
      init_pair(CP_BASE + i, th->fg256[i], COLOR_BLACK);
    else
      init_pair(CP_BASE + i, th->fg8[i], COLOR_BLACK);
  }
}

/* color_init — start_color + apply initial theme + pin HUD/HINT pairs. */
static void color_init(int theme) {
  start_color();
  use_default_colors();
  theme_apply(theme);

  /* HUD pairs are theme-independent — init ONCE, theme_apply leaves them alone.
   */
  init_pair(PAIR_HUD, COLORS >= 256 ? 226 : COLOR_YELLOW, -1);
  init_pair(PAIR_HINT, COLORS >= 256 ? 51 : COLOR_CYAN, -1);
}

/* ramp_attr — colour pair + (256-mode) bold for top buckets / (8-mode) attr8.
 */
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

/* ===================================================================== */
/* §5  bitmap state — clamp helpers + the Bitmap struct                   */
/* ===================================================================== */

/* clamp_int — clamp v to closed interval [lo, hi]. */
static inline int clamp_int(int v, int lo, int hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}
/* clamp_uchar_int — uint8 saturation: clamp v to [0, 255]. */
static inline int clamp_uchar_int(int v) { return clamp_int(v, 0, 255); }

/*
 * Bitmap — the simulation's single mutable struct.  All per-tick code
 * operates on this; nothing else holds CA state.
 *
 *   bmap[cols × (rows+2)]   uint8 heat grid; bottom 2 rows are fuel
 *   prev[cols × rows]       last drawn frame — for diff-based clearing
 *   dither[cols × rows]     float working buffer for Floyd-Steinberg
 *   table[MAXTABLE=1280]    decay LUT built by gentable() at init/resize
 *
 *   cols, rows              terminal extent in cells (rows = visible only)
 *   height                  frame counter; warms up until full brightness
 *   loop                    fuel-burst countdown (aalib original logic)
 *   sloop                   fuel-sweep cycle counter
 *   fuel                    user-adjustable intensity, ∈ [0.1, 1.0]
 *   theme, cycle_tick       active palette index + auto-cycle counter
 *
 * Lifecycle:
 *   bitmap_alloc / bitmap_init at startup and on SIGWINCH (re-alloc).
 *   bitmap_free at exit.  Every other operation in §6–§10 reads or
 *   mutates this struct in place.
 */
typedef struct {
  unsigned char *bmap; /* [cols * (rows+2)] heat uint8             */
  unsigned char *prev; /* [cols * rows]     last drawn frame        */
  float *dither;       /* [cols * rows]     dither working buffer   */
  unsigned int table[MAXTABLE];
  int cols;
  int rows;
  int height; /* frame counter for warm-up                 */
  int loop;   /* fuel burst countdown                      */
  int sloop;  /* fuel sweep counter                        */
  float fuel; /* intensity scale                           */
  int theme;
  int cycle_tick;
} Bitmap;

/* ===================================================================== */
/* §6  decay table — gentable() builds the per-row cooling LUT            */
/* ===================================================================== */

/*
 * decay_table_entry — one entry of the cooling lookup table.
 *   sum ≤ cooling_per_row:  return 0                 (cell goes cold)
 *   else:                   return (sum - cooling_per_row) / 5
 * Subtracting BEFORE dividing concentrates cooling on low sums.
 */
static unsigned int decay_table_entry(int neighbour_sum, int cooling_per_row) {
  if (neighbour_sum <= cooling_per_row)
    return 0;
  return (unsigned int)(neighbour_sum - cooling_per_row) / 5;
}

/* gentable — build the MAXTABLE decay LUT.  Called at init + on resize. */
static void gentable(Bitmap *b) {
  int cooling_per_row = 800 / b->rows;
  if (cooling_per_row == 0)
    cooling_per_row = 1;

  for (int neighbour_sum = 0; neighbour_sum < MAXTABLE; neighbour_sum++)
    b->table[neighbour_sum] = decay_table_entry(neighbour_sum, cooling_per_row);
}

/* ===================================================================== */
/* §7  propagation — firemain() applies the 5-neighbour stencil           */
/* ===================================================================== */

/*
 * sample_five_neighbours_below — read the 5-cell aafire stencil at (x, y).
 *
 *      .   .   .          ← y    (cell being written)
 *      a   b   c          ← y+1  (3 neighbours)
 *      d       e          ← y+2  (2 neighbours; NO centre — gives rounded
 * blobs)
 *
 * Edge columns clamp horizontally; the (rows+2) buffer keeps y+2 in bounds.
 */
static unsigned int sample_five_neighbours_below(const unsigned char *bmap,
                                                 int cols, int x, int y) {
  int left_column = clamp_int(x - 1, 0, cols - 1);
  int right_column = clamp_int(x + 1, 0, cols - 1);
  int row_below = y + 1;
  int row_deeper = y + 2;

  unsigned int below_left = bmap[row_below * cols + left_column];
  unsigned int below_centre = bmap[row_below * cols + x];
  unsigned int below_right = bmap[row_below * cols + right_column];
  unsigned int deeper_left = bmap[row_deeper * cols + left_column];
  unsigned int deeper_right = bmap[row_deeper * cols + right_column];

  return below_left + below_centre + below_right + deeper_left + deeper_right;
}

/*
 * firemain — DRIVER.  The headline algorithm.  5-neighbour-below
 *            propagation + decay table; one full top-down sweep per tick.
 *
 *   for each cell (y, x), top-left → bottom-right:
 *     sum ← sample_five_neighbours_below(bmap, cols, x, y)
 *     idx ← min(sum, MAXTABLE - 1)                paranoia clamp;
 *                                                  max real sum = 5·255 = 1275
 *     bmap[y][x] ← b->table[idx]                   cooled byte
 *
 * INPUTS / UNITS
 *   b   Bitmap mutated in place.  Reads from b->bmap, writes back to it.
 *
 * WHY TOP-DOWN
 *   Bottom-up sweep would re-read freshly-written hot cells in the SAME
 *   tick (the y+1 neighbour might be one we already wrote this pass),
 *   polluting propagation with same-tick state and visibly speeding the
 *   flame upward.  Top-down keeps each tick's reads against the PREVIOUS
 *   tick's bmap state where it matters.
 *
 * WHY THIS STENCIL (not a 9-neighbour Moore or 4-neighbour von Neumann)
 *   3 cells from y+1 + 2 cells from y+2 with NO centre on y+2 biases
 *   the average toward the SIDES of the immediately-lower row.  That
 *   side bias is what gives aafire its rounded blob shape rather than
 *   vertical streaks (which Doom-style 3-cell stencils produce).
 *
 * COST
 *   O(cols × rows) bytes touched per tick.  No allocations, no division
 *   in the hot loop (the LUT pre-divides everything at init/resize).
 */
static void firemain(Bitmap *b) {
  int cols = b->cols;
  int rows = b->rows;
  unsigned char *bmap = b->bmap;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      unsigned int neighbour_sum =
          sample_five_neighbours_below(bmap, cols, x, y);
      unsigned int table_index =
          (neighbour_sum < MAXTABLE) ? neighbour_sum : MAXTABLE - 1;
      bmap[y * cols + x] = (unsigned char)b->table[table_index];
    }
  }
}

/* ===================================================================== */
/* §8  fuel seeding — drawfire() seeds the bottom 2 rows in an arch       */
/* ===================================================================== */

/*
 * ArchSweep — three counters traversed together across the fuel row.
 *
 *   column   current column being seeded (0..cols)
 *   i1       grows  from 1, +4 per cell           small at LEFT edge
 *   i2       shrinks from 4·cols+1, -4 per cell   small at RIGHT edge
 *
 * Per-column arch ceiling = min(i1, i2, height): low at both edges,
 * peaks in the middle, capped by the warm-up counter.  Both i1 and i2
 * step by 4 per cell — an aalib choice that gives the arch's edges a
 * sharper curve than a linear ramp would.
 */
typedef struct {
  int column;
  int i1;
  int i2;
} ArchSweep;

/* arch_sweep_cap — per-column heat ceiling = min(i1, i2, warmup); floor at 1.
 */
static int arch_sweep_cap(const ArchSweep *sweep, int height) {
  int cap = (sweep->i1 < sweep->i2) ? sweep->i1 : sweep->i2;
  if (height < cap)
    cap = height;
  if (cap < 1)
    cap = 1;
  return cap;
}

/* arch_sweep_advance_one_cell — column++; i1 += 4; i2 -= 4. */
static void arch_sweep_advance_one_cell(ArchSweep *sweep) {
  sweep->column += 1;
  sweep->i1 += 4;
  sweep->i2 -= 4;
}

/*
 * seed_one_burst — write up to 6 consecutive fuel cells with a ±2 walk.
 * The walk creates micro-flicker along the fuel line; bursts pick
 * independent bases so neighbouring bursts can differ a lot.
 */
static void seed_one_burst(Bitmap *b, ArchSweep *sweep, unsigned char *fuel_row,
                           unsigned char *fuel_row2) {
  int cap = arch_sweep_cap(sweep, b->height);
  int seed_value = (int)((float)(rand() % cap) * b->fuel);
  int burst_length = rand() % 6;

  for (int j = 0; j <= burst_length && sweep->column < b->cols; j++) {
    seed_value = clamp_uchar_int(seed_value);
    fuel_row[sweep->column] = (unsigned char)seed_value;
    seed_value = clamp_uchar_int(seed_value + rand() % 6 - 2);
    fuel_row2[sweep->column] = (unsigned char)seed_value;
    seed_value += rand() % 6 - 2;
    arch_sweep_advance_one_cell(sweep);
  }
}

/* drawfire — seed the 2 fuel rows: burst → gap → burst → gap → … sweep. */
static void drawfire(Bitmap *b) {
  int cols = b->cols;
  int rows = b->rows;
  unsigned char *fuel_row = b->bmap + rows * cols;         /* row `rows`   */
  unsigned char *fuel_row_2 = b->bmap + (rows + 1) * cols; /* row `rows+1` */

  /* Step 1 — advance frame counter and burst countdown. */
  b->height++;
  b->loop--;
  if (b->loop < 0) {
    b->loop = rand() % 3;
    b->sloop++;
  }

  /* Step 2 — sweep across columns: burst → gap → burst → gap → … */
  ArchSweep sweep = {.column = 0, .i1 = 1, .i2 = 4 * cols + 1};
  while (sweep.column < cols) {
    seed_one_burst(b, &sweep, fuel_row, fuel_row_2);
    arch_sweep_advance_one_cell(&sweep); /* one-cell gap between bursts */
  }
}

/* ===================================================================== */
/* §9  bitmap lifecycle — alloc / free / init / per-tick                  */
/* ===================================================================== */

/* bitmap_alloc — calloc bmap (cols × rows+2), prev, dither. */
static void bitmap_alloc(Bitmap *b, int cols, int rows) {
  b->cols = cols;
  b->rows = rows;
  /* +2 extra rows for the fuel rows that firemain reads from */
  b->bmap = calloc((size_t)(cols * (rows + 2)), sizeof(unsigned char));
  b->prev = calloc((size_t)(cols * rows), sizeof(unsigned char));
  b->dither = calloc((size_t)(cols * rows), sizeof(float));
}

static void bitmap_free(Bitmap *b) {
  free(b->bmap);
  free(b->prev);
  free(b->dither);
  memset(b, 0, sizeof *b);
}

/* bitmap_init — alloc buffers + reset scalars + gentable.  Called at startup +
 * on resize. */
static void bitmap_init(Bitmap *b, int cols, int rows, int theme) {
  bitmap_alloc(b, cols, rows);
  b->height = 0;
  b->loop = 0;
  b->sloop = 0;
  b->fuel = FUEL_DEFAULT;
  b->theme = theme;
  b->cycle_tick = 0;
  gentable(b);
}

/*
 * bitmap_tick — DRIVER.  One simulation step.
 *
 *   drawfire(b)               seed bottom 2 fuel rows (arch-sweep bursts)
 *   firemain(b)               5-neighbour-below propagation + decay LUT
 *   cycle_tick++; if reached CYCLE_TICKS:
 *     theme = (theme + 1) mod THEME_COUNT;  theme_apply(theme)
 *     return true                        signals caller to clear screen
 *   return false
 *
 * INPUTS / UNITS
 *   b   Bitmap mutated; both buffers (bmap, prev) advance one frame.
 *
 * WHY drawfire BEFORE firemain
 *   The propagation stencil reads y+1 and y+2 — those rows are the fuel
 *   rows.  Seeding fuel AFTER firemain would mean the new fuel doesn't
 *   visibly propagate until the NEXT tick (a 1-frame stale-flame lag).
 *
 * RETURN VALUE CONTRACT
 *   true  → theme just changed; the caller should clear the screen on
 *           the next render so old-theme glyphs don't ghost.
 *   false → theme unchanged; caller can render normally.
 */
static bool bitmap_tick(Bitmap *b) {
  drawfire(b);
  firemain(b);

  b->cycle_tick++;
  if (b->cycle_tick >= CYCLE_TICKS) {
    b->cycle_tick = 0;
    b->theme = (b->theme + 1) % THEME_COUNT;
    theme_apply(b->theme);
    return true;
  }
  return false;
}

/* ===================================================================== */
/* §10  render pipeline — gamma + Floyd-Steinberg dither + paint          */
/* ===================================================================== */

/* gamma_correct_byte — uint8 linear heat → perceptual float (heat/255)^(1/2.2).
 */
static inline float gamma_correct_byte(unsigned char heat) {
  float linear = (float)heat / 255.f;
  return powf(linear, 1.f / 2.2f);
}

/* COLD_SENTINEL — dither-buffer value for heat == 0; negative so a single
 * (>= 0.f) check distinguishes "skip" from "process". */
#define COLD_SENTINEL (-1.0f)

/* pipeline_pass1_gamma_correct — fill dither_buffer with perceptual heat
 * (or COLD_SENTINEL where heat == 0). */
static void pipeline_pass1_gamma_correct(Bitmap *b) {
  int total_cells = b->cols * b->rows;
  const unsigned char *heat_bitmap = b->bmap;
  float *dither_buffer = b->dither;

  for (int i = 0; i < total_cells; i++) {
    unsigned char linear_heat = heat_bitmap[i];
    if (linear_heat == 0) {
      dither_buffer[i] = COLD_SENTINEL;
      continue;
    }
    dither_buffer[i] = gamma_correct_byte(linear_heat);
  }
}

/* pipeline_diffuse_quant_error — Floyd-Steinberg: 7/16 right, 3/16 down-left,
 * 5/16 down, 1/16 down-right.  Cold-tagged neighbours skip. */
static void pipeline_diffuse_quant_error(float *dither_buffer, int cols,
                                         int rows, int x, int y,
                                         float quant_error) {
  int i = y * cols + x;

  if (x + 1 < cols && dither_buffer[i + 1] >= 0.f)
    dither_buffer[i + 1] += quant_error * (7.f / 16.f);

  if (y + 1 < rows) {
    if (x - 1 >= 0 && dither_buffer[i + cols - 1] >= 0.f)
      dither_buffer[i + cols - 1] += quant_error * (3.f / 16.f);
    if (dither_buffer[i + cols] >= 0.f)
      dither_buffer[i + cols] += quant_error * (5.f / 16.f);
    if (x + 1 < cols && dither_buffer[i + cols + 1] >= 0.f)
      dither_buffer[i + cols + 1] += quant_error * (1.f / 16.f);
  }
}

/* pipeline_draw_lit_cell — quantise one cell, diffuse error, paint glyph. */
static void pipeline_draw_lit_cell(Bitmap *b, int x, int y, float perceptual) {
  int bucket = lut_index(perceptual);
  float bucket_midpoint = lut_midpoint(bucket);
  float quant_error = perceptual - bucket_midpoint;

  pipeline_diffuse_quant_error(b->dither, b->cols, b->rows, x, y, quant_error);

  attr_t glyph_attr = ramp_attr(bucket, b->theme);
  attron(glyph_attr);
  mvaddch(y, x, (chtype)(unsigned char)k_ramp[bucket]);
  attroff(glyph_attr);
}

/* pipeline_pass2_quantise_and_draw — for each cell: erase stale ' ' if
 * cold-now-but-was-lit; otherwise quantise + diffuse + paint via
 * pipeline_draw_lit_cell. */
static void pipeline_pass2_quantise_and_draw(Bitmap *b, int tcols, int trows) {
  int cols = b->cols;
  int rows = b->rows;
  const float *dither_buffer = b->dither;
  const unsigned char *previous_heat = b->prev;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;

      int i = y * cols + x;
      float perceptual = dither_buffer[i];

      if (perceptual < 0.f) {
        bool was_lit_last_frame = (previous_heat[i] > 0);
        if (was_lit_last_frame)
          mvaddch(y, x, ' ');
        continue;
      }

      pipeline_draw_lit_cell(b, x, y, perceptual);
    }
  }
}

/*
 * pipeline_pass3_archive_current_frame() — record current heat as previous.
 *
 *   Next frame's pass 2 will compare prev[i] > 0 against the new heat to
 *   decide whether a cold cell needs a literal ' ' erase.
 */
static void pipeline_pass3_archive_current_frame(Bitmap *b) {
  int total_cells = b->cols * b->rows;
  memcpy(b->prev, b->bmap, (size_t)total_cells * sizeof(unsigned char));
}

/* bitmap_draw — three-pass render: gamma → quantise+dither+paint → archive. */
static void bitmap_draw(Bitmap *b, int tcols, int trows) {
  pipeline_pass1_gamma_correct(b);
  pipeline_pass2_quantise_and_draw(b, tcols, trows);
  pipeline_pass3_archive_current_frame(b);
}

/* ===================================================================== */
/* §11  scene — orchestrator over Bitmap, pause, debug mode               */
/* ===================================================================== */

typedef enum {
  DEBUG_OFF = 0,      /* normal gamma + dither + LUT pipeline (§10)        */
  DEBUG_RAW = 1,      /* show each cell's raw heat byte as a hex digit     */
  DEBUG_FUEL = 2,     /* paint only the 2 fuel rows; rest stays as-was     */
  DEBUG_NODITHER = 3, /* gamma + LUT + paint, no Floyd-Steinberg dither    */
  DEBUG_COUNT = 4,
} DebugMode;

/* debug_mode_name — HUD label for current debug mode. */
static const char *debug_mode_name(DebugMode m) {
  switch (m) {
  case DEBUG_OFF:
    return "off     ";
  case DEBUG_RAW:
    return "raw heat";
  case DEBUG_FUEL:
    return "fuel    ";
  case DEBUG_NODITHER:
    return "nodither";
  default:
    return "?       ";
  }
}

/*
 * Scene — orchestrator over Bitmap.  Three small scalars on top of bmap.
 *
 *   bmap           the CA state (everything in §5–§10 reads/writes this)
 *   paused         set by space; freezes scene_tick early-return
 *   needs_clear    set on theme cycle / resize / debug-mode change so
 *                  the next render does a full erase() (the diff-render
 *                  in §10 can't undo glyphs the new path won't touch)
 *   debug_mode     dispatch knob for scene_draw (§12 alternative renders)
 *
 * Lifecycle: scene_init → scene_tick / scene_draw per frame → scene_free.
 *   scene_resize on SIGWINCH preserves theme + fuel + debug across realloc.
 */
typedef struct {
  Bitmap bmap;
  bool paused;
  bool needs_clear;
  DebugMode debug_mode;
} Scene;

/* Forward declaration: §12 owns the debug render functions. */
static void scene_draw_debug(Scene *s, int cols, int rows);

/* scene_init — zero Scene, init Bitmap, default debug_mode = OFF. */
static void scene_init(Scene *s, int cols, int rows, int theme) {
  memset(s, 0, sizeof *s);
  bitmap_init(&s->bmap, cols, rows, theme);
  s->debug_mode = DEBUG_OFF;
}

/* scene_free — release Bitmap buffers. */
static void scene_free(Scene *s) { bitmap_free(&s->bmap); }

/* scene_resize — re-alloc Bitmap for new extent; preserve theme/fuel/debug. */
static void scene_resize(Scene *s, int cols, int rows) {
  int t = s->bmap.theme;
  float fuel = s->bmap.fuel;
  DebugMode debug_mode = s->debug_mode;

  bitmap_free(&s->bmap);
  bitmap_init(&s->bmap, cols, rows, t);

  s->bmap.fuel = fuel;
  s->debug_mode = debug_mode;
  s->needs_clear = true;
  gentable(&s->bmap); /* rebuild decay table for new rows */
}

/* scene_cycle_debug_mode — d/D selector; forces needs_clear because the
 * previous render path may have left glyphs the new one won't overpaint. */
static void scene_cycle_debug_mode(Scene *s, int step) {
  int next = ((int)s->debug_mode + step + DEBUG_COUNT) % DEBUG_COUNT;
  s->debug_mode = (DebugMode)next;
  s->needs_clear = true;
}

/* scene_tick — pause-gated wrapper over bitmap_tick; flips needs_clear on theme
 * cycle. */
static void scene_tick(Scene *s) {
  if (!s->paused) {
    if (bitmap_tick(&s->bmap))
      s->needs_clear = true;
  }
}

/*
 * scene_draw — DRIVER.  Render dispatcher.
 *
 *   if debug_mode == DEBUG_OFF:  bitmap_draw  (full §10 pipeline)
 *   else:                        scene_draw_debug  (alternative §12 render)
 *
 * INPUTS / UNITS
 *   s              Scene to render (const-ish; bmap.dither + bmap.prev mutate
 *                  inside the §10 pipeline as scratch).
 *   cols, rows     terminal extent in cells.
 *
 * Two-line dispatcher; the heavy work lives in §10 (full pipeline) or §12
 * (debug overlays).  Splitting this way means turning on a debug mode
 * never touches the production render code.
 */
static void scene_draw(Scene *s, int cols, int rows) {
  if (s->debug_mode == DEBUG_OFF)
    bitmap_draw(&s->bmap, cols, rows);
  else
    scene_draw_debug(s, cols, rows);
}

/* ===================================================================== */
/* §12  debug overlay — alternative render modes for inspection           */
/* ===================================================================== */

/* hex_digit_for_heat — top nibble → '0'..'F'.  Used by DEBUG_RAW. */
static inline char hex_digit_for_heat(unsigned char heat) {
  static const char k_hex[] = "0123456789ABCDEF";
  return k_hex[heat >> 4];
}

/* debug_render_raw_heat — DEBUG_RAW: paint hex(heat) per cell; colour by top 3
 * bits. */
static void debug_render_raw_heat(Scene *s, int tcols, int trows) {
  Bitmap *b = &s->bmap;
  int cols = b->cols;
  int rows = b->rows;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;
      unsigned char heat = b->bmap[y * cols + x];

      int bucket = heat >> 5; /* 0..7 from top 3 bits */
      if (bucket >= RAMP_N)
        bucket = RAMP_N - 1;
      char glyph = hex_digit_for_heat(heat);
      attr_t a = ramp_attr(bucket, b->theme);

      attron(a);
      mvaddch(y, x, (chtype)(unsigned char)glyph);
      attroff(a);
    }
  }
}

/* debug_render_fuel_only — DEBUG_FUEL: erase + paint only the 2 fuel rows. */
static void debug_render_fuel_only(Scene *s, int tcols, int trows) {
  Bitmap *b = &s->bmap;
  int cols = b->cols;
  int rows = b->rows;

  erase();

  for (int row_offset = 0; row_offset < 2; row_offset++) {
    int fuel_row = rows + row_offset;       /* bmap row */
    int screen_row = rows - 2 + row_offset; /* screen row */
    if (screen_row < 0 || screen_row >= trows)
      continue;

    for (int x = 0; x < cols && x < tcols; x++) {
      unsigned char heat = b->bmap[fuel_row * cols + x];
      float perceptual = (heat == 0) ? 0.f : gamma_correct_byte(heat);
      int bucket = lut_index(perceptual);
      char glyph = k_ramp[bucket];
      attr_t a = ramp_attr(bucket, b->theme);
      attron(a);
      mvaddch(screen_row, x, (chtype)(unsigned char)glyph);
      attroff(a);
    }
  }
}

/* debug_render_no_dither — DEBUG_NODITHER: gamma + LUT + paint, no FS
 * diffusion. */
static void debug_render_no_dither(Scene *s, int tcols, int trows) {
  Bitmap *b = &s->bmap;
  int cols = b->cols;
  int rows = b->rows;

  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      if (x >= tcols || y >= trows)
        continue;
      int i = y * cols + x;

      unsigned char heat = b->bmap[i];
      if (heat == 0) {
        if (b->prev[i] > 0)
          mvaddch(y, x, ' ');
        continue;
      }
      float perceptual = gamma_correct_byte(heat);
      int bucket = lut_index(perceptual);
      char glyph = k_ramp[bucket];
      attr_t a = ramp_attr(bucket, b->theme);
      attron(a);
      mvaddch(y, x, (chtype)(unsigned char)glyph);
      attroff(a);
    }
  }
  memcpy(b->prev, b->bmap, (size_t)(cols * rows) * sizeof(unsigned char));
}

/* scene_draw_debug — dispatch on debug_mode; default falls through to
 * bitmap_draw. */
static void scene_draw_debug(Scene *s, int cols, int rows) {
  switch (s->debug_mode) {
  case DEBUG_RAW:
    debug_render_raw_heat(s, cols, rows);
    break;
  case DEBUG_FUEL:
    debug_render_fuel_only(s, cols, rows);
    break;
  case DEBUG_NODITHER:
    debug_render_no_dither(s, cols, rows);
    break;
  default:
    bitmap_draw(&s->bmap, cols, rows);
    break;
  }
}

/* ===================================================================== */
/* §13  screen + HUD                                                      */
/* ===================================================================== */

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

static void screen_draw_hud_top_right(const Screen *s, const Scene *sc,
                                      double fps, int sim_fps) {
  char buf[HUD_COLS + 1];
  snprintf(buf, sizeof buf, " %4.1f fps  [%s]  fuel:%.2f  dbg:%s  sim:%d ", fps,
           k_themes[sc->bmap.theme].name, sc->bmap.fuel,
           debug_mode_name(sc->debug_mode), sim_fps);

  int hud_x = s->cols - (int)strlen(buf);
  if (hud_x < 0)
    hud_x = 0;

  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, hud_x, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void screen_draw_hint_bottom(const Screen *s) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(s->rows - 1, 0,
           " q/ESC:quit  space:pause  t/T:theme  d/D:debug  g/G:fuel  ]/[:Hz ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *s, Scene *sc, double fps, int sim_fps) {
  if (sc->needs_clear) {
    erase();
    sc->needs_clear = false;
  }
  scene_draw(sc, s->cols, s->rows);
  screen_draw_hud_top_right(s, sc, fps, sim_fps);
  screen_draw_hint_bottom(s);
}
static void screen_present(void) {
  wnoutrefresh(stdscr);
  doupdate();
}

/* ===================================================================== */
/* §14  app — signals + key dispatch                                      */
/* ===================================================================== */

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

static void app_cycle_theme(App *a, int step) {
  Bitmap *b = &a->scene.bmap;
  int next = (b->theme + step + THEME_COUNT) % THEME_COUNT;
  b->theme = next;
  b->cycle_tick = 0;
  theme_apply(next);
  a->scene.needs_clear = true;
}

static void app_nudge_fuel(App *a, float step) {
  Bitmap *b = &a->scene.bmap;
  float v = b->fuel + step;
  if (v > FUEL_MAX)
    v = FUEL_MAX;
  if (v < FUEL_MIN)
    v = FUEL_MIN;
  b->fuel = v;
}

static void app_nudge_sim_fps(App *a, int step) {
  int v = a->sim_fps + step;
  if (v > SIM_FPS_MAX)
    v = SIM_FPS_MAX;
  if (v < SIM_FPS_MIN)
    v = SIM_FPS_MIN;
  a->sim_fps = v;
}

static bool app_handle_key(App *a, int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27 /* ESC */:
    return false;
  case ' ':
    a->scene.paused = !a->scene.paused;
    break;

  case 't':
    app_cycle_theme(a, +1);
    break;
  case 'T':
    app_cycle_theme(a, -1);
    break;

  case 'd':
    scene_cycle_debug_mode(&a->scene, +1);
    break;
  case 'D':
    scene_cycle_debug_mode(&a->scene, -1);
    break;

  case 'g':
    app_nudge_fuel(a, +FUEL_STEP);
    break;
  case 'G':
    app_nudge_fuel(a, -FUEL_STEP);
    break;

  case ']':
    app_nudge_sim_fps(a, +SIM_FPS_STEP);
    break;
  case '[':
    app_nudge_sim_fps(a, -SIM_FPS_STEP);
    break;

  default:
    break;
  }
  return true;
}

/* ===================================================================== */
/* §15  main — fixed-step loop                                            */
/* ===================================================================== */

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
  scene_init(&app->scene, app->screen.cols, app->screen.rows, 0);

  int64_t ft = clock_ns(), sa = 0, fa = 0;
  int fc = 0;
  double fpsd = 0.;

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
    float alpha = (float)sa / (float)tick;
    (void)alpha;

    fc++;
    fa += dt;
    if (fa >= FPS_UPDATE_MS * NS_PER_MS) {
      fpsd = (double)fc / ((double)fa / (double)NS_PER_SEC);
      fc = 0;
      fa = 0;
    }

    /* ── frame cap (sleep BEFORE render so I/O doesn't drift) ── */
    int64_t el = clock_ns() - ft + dt;
    clock_sleep_ns(NS_PER_SEC / 60 - el);

    screen_draw(&app->screen, &app->scene, fpsd, app->sim_fps);
    screen_present();

    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch))
      app->running = 0;
  }

  scene_free(&app->scene);
  screen_free(&app->screen);
  return 0;
}
