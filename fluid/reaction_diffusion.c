/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_diffusion.c — Gray-Scott "Turing patterns" on a terminal grid.
 *
 * Two chemicals, U and V, sit on every cell of the grid.  U is fed in
 * everywhere and spreads fast; V grows by eating U, spreads slow, and
 * dies off.  That mismatch is enough for the grid to organise itself
 * into spots, stripes, coral, mazes, and drifting blobs — no global
 * plan, just each cell reacting to its neighbours.  Seven presets each
 * pick a different feed/kill pair, and tiny changes give wildly
 * different looks.
 *
 * Companion files: procedural/fields/reaction_diffusion_gray_scott.c
 * (same maths, lighter), fluid/cfl_stability_explorer.c (the timestep
 * limit we run just under), procedural/diffusion/heat_diffusion.c
 * (diffusion with no reaction — no patterns).
 *
 * References: Turing 1952 (the original idea); Gray & Scott 1984 (this
 * chemistry); Pearson 1993 (the feed/kill map our presets come from);
 * Bourke 1997 (the brightness-to-character ramp).
 */

#define _POSIX_C_SOURCE 200809L

#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ── §1  config — every tunable in one place ── */

/* How fast each chemical spreads.  U must spread faster than V, or the
 * two just smear together into a flat field and no pattern forms. */
#define DIFFUSION_U 0.20f
#define DIFFUSION_V 0.10f

/* Size of one time-step.  There's a hard ceiling (~1.25 here) above
 * which the maths blows up into garbage; 1.0 keeps a safe margin. */
#define EULER_DT 1.00f

/* How many simulation steps we run per drawn frame, and its tuning range
 * — more steps = faster-moving pattern. */
#define STEPS_PER_FRAME_DEFAULT 16
#define STEPS_PER_FRAME_MIN 1
#define STEPS_PER_FRAME_MAX 64
#define STEPS_PER_FRAME_STEP 4

/* Steps to run unseen before the first frame, so the user opens onto a
 * grown pattern instead of a few lonely dots. */
#define WARMUP_TICK_COUNT 600

/* Half-width of each square starter blob, in cells.  3 → 7×7 block. */
#define SEED_BLOB_HALF_WIDTH 3

/* In a settled pattern V tops out around 0.3-0.5, so we multiply it up
 * before choosing a character — otherwise only the dim end of the ramp
 * would ever show. */
#define CATALYST_DISPLAY_SCALE 2.2f

/* How many steps the brightness ramp has (must match the glyph and
 * threshold tables below). */
#define RAMP_SLOT_COUNT 8

/* How long before the colour theme rotates on its own (~26 s at 30 fps). */
#define AUTO_THEME_CYCLE_FRAMES 800

/* Colour-pair slot numbers.  The ramp takes the first 8; HUD and hint
 * follow it. */
enum {
  PAIR_RAMP_FIRST = 1,
  PAIR_HUD = PAIR_RAMP_FIRST + RAMP_SLOT_COUNT,
  PAIR_HINT,
};

/* Frame-timing constants. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_FPS_CAP 30
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS_CAP)
#define FPS_RECOMPUTE_MS 500

#define THEME_COUNT 4

/* ── §2  clock — wall-clock in nanoseconds + sleep ── */

static int64_t clock_now_ns(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (int64_t)ts.tv_sec * NS_PER_SEC + ts.tv_nsec;
}

static void clock_sleep_ns(int64_t ns) {
  if (ns <= 0)
    return;
  struct timespec ts = {ns / NS_PER_SEC, ns % NS_PER_SEC};
  nanosleep(&ts, NULL);
}

/* ── §3  rng — thin wrapper over stdlib rand ── */
/* Only used to scatter the starter blobs; everything else is exactly
 * repeatable. */
static int rand_in_range(int lo, int hi_exclusive) {
  if (hi_exclusive <= lo)
    return lo;
  return lo + rand() % (hi_exclusive - lo);
}

/* ── §4  ramp — characters that stand in for brightness ── */
/* Eight characters from "nothing here" to "packed solid".  A cell picks
 * the heaviest one its scaled value reaches.  The cut-offs are spaced a
 * bit tighter in the middle, where most of the pattern lives. */

static const char ramp_glyph_table[RAMP_SLOT_COUNT] = {
    ' ', /* empty — no V here, left blank                  */
    '.', /* barely there — edge of the reaction front       */
    ':',
    '-',
    '+',
    '*',
    '#',
    '@', /* peak V — the bright core of a spot or stripe    */
};

static const float ramp_threshold_table[RAMP_SLOT_COUNT] = {
    0.00f, 0.10f, 0.24f, 0.38f, 0.52f, 0.65f, 0.78f, 0.90f,
};

static int glyph_slot_for(float scaled_v) {
  for (int i = RAMP_SLOT_COUNT - 1; i >= 0; i--)
    if (scaled_v >= ramp_threshold_table[i])
      return i;
  return 0;
}

/* ── §5  themes — four colour palettes ── */

/*
 * ColourTheme — one named colour scheme for the brightness ramp.
 *
 * A cell's V amount is bucketed into 8 levels, and the level picks a
 * colour from this list — so a theme is just 8 foreground colours
 * (dark to bright) plus a short name to show in the HUD.
 *
 * We use a single dark-to-bright ramp rather than a two-ended one
 * because the amounts are always between 0 and 1 — there's no negative
 * side to highlight, and "brighter = more stuff" is what the eye reads
 * naturally (Bourke 1997).
 *
 * Colour 0 sits at the dark end and matches the blank character that
 * never gets drawn, so its exact value barely matters; the rest are
 * kept in the bright half of the palette so even faint patterns stay
 * visible.
 *
 *   display_name  short label shown in the HUD
 *   fg256[]       the 8 foreground colours, darkest first
 */
typedef struct {
    const char *display_name;
    short       fg256[RAMP_SLOT_COUNT];
} ColourTheme;

static const ColourTheme theme_table[THEME_COUNT] = {
    {"ocean", {232, 17, 19, 21, 27, 33, 51, 231}},
    {"forest", {232, 22, 28, 34, 40, 46, 118, 231}},
    {"magma", {232, 52, 88, 124, 160, 196, 214, 231}},
    {"violet", {232, 54, 56, 93, 129, 165, 201, 231}},
};

/* ── §6  colors — set up colour pairs, switch themes ── */

static bool terminal_has_256_colours = false;

static void apply_theme(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const ColourTheme *theme = &theme_table[theme_index];

  if (terminal_has_256_colours) {
    for (int i = 0; i < RAMP_SLOT_COUNT; i++)
      init_pair((short)(PAIR_RAMP_FIRST + i), theme->fg256[i], -1);
  } else {
    /* Coarse fall-back for terminals with only the basic 8 colours. */
    static const short FALLBACK[RAMP_SLOT_COUNT] = {
        COLOR_BLACK, COLOR_BLUE,  COLOR_BLUE,  COLOR_CYAN,
        COLOR_CYAN,  COLOR_WHITE, COLOR_WHITE, COLOR_WHITE,
    };
    for (int i = 0; i < RAMP_SLOT_COUNT; i++)
      init_pair((short)(PAIR_RAMP_FIRST + i), FALLBACK[i], -1);
  }
}

static void colors_init(int theme_index) {
  start_color();
  use_default_colors();
  terminal_has_256_colours = (COLORS >= 256);
  apply_theme(theme_index);

  /* HUD colours: bright so they read over any animation. */
  if (terminal_has_256_colours) {
    init_pair(PAIR_HUD, 226, -1); /* bright yellow */
    init_pair(PAIR_HINT, 51, -1); /* bright cyan   */
  } else {
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* Colour for a ramp level, with a bold kick on the top two so peaks
 * pop a little brighter. */
static attr_t attribute_for_slot(int slot) {
  attr_t a = COLOR_PAIR(PAIR_RAMP_FIRST + slot);
  if (slot >= RAMP_SLOT_COUNT - 2)
    a |= A_BOLD;
  return a;
}

/* ── §7  presets — the feed/kill pairs that name each look ── */

/*
 * GrayScottPreset — one named pattern, picked by two numbers.
 *
 * The whole character of the pattern comes down to two knobs: the feed
 * rate (how fast U is topped up) and the kill rate (how fast V dies).
 * Pearson 1993 mapped out which (feed, kill) pairs give which look —
 * spots, stripes, worms, drifting blobs — and the borders between them
 * are razor-thin.  Compare Stripes (0.0600, 0.0620) and Worms (0.0620,
 * 0.0610): a third-decimal nudge and you're in a different world.
 *
 * Each preset is one landmark on that map, plus how many starter blobs
 * to drop — some looks fill in from just a few, others need more to get
 * going at a watchable speed.
 *
 *   display_name      label shown in the HUD
 *   feed_rate         how fast U is replenished
 *   kill_rate         how fast V decays
 *   seed_blob_count   number of starter blobs to scatter
 */
typedef struct {
    const char *display_name;
    float       feed_rate;
    float       kill_rate;
    int         seed_blob_count;
} GrayScottPreset;

static const GrayScottPreset preset_table[] = {
    {"Mitosis  ", 0.0367f, 0.0649f, 4}, /* spots that periodically divide */
    {"Coral    ", 0.0545f, 0.0630f, 5}, /* branching coral-like growth    */
    {"Stripes  ", 0.0600f, 0.0620f, 3}, /* labyrinthine stripe patterns   */
    {"Worms    ", 0.0620f, 0.0610f, 6}, /* winding worm tendrils          */
    {"Maze     ", 0.0290f, 0.0570f, 8}, /* fine-grained maze texture      */
    {"Bubbles  ", 0.0940f, 0.0590f, 3}, /* stable round bubble lattice    */
    {"Solitons ", 0.0250f, 0.0500f, 4}, /* slowly drifting soliton blobs  */
};

#define PRESET_COUNT ((int)(sizeof preset_table / sizeof preset_table[0]))

/* ── §8  grid_buffers — make, free, and resize the four arrays ── */

/*
 * Grid — the whole simulation state.
 *
 * Two chemicals, U and V, each stored as one flat array of cells laid
 * out row by row (cell at row r, column c lives at index r*cols + c).
 *
 * Each step reads every cell's neighbours to work out the next value,
 * so we can't overwrite cells as we go — a neighbour we haven't reached
 * yet would see the new value instead of the old one and the whole thing
 * quietly breaks.  The fix is two arrays per chemical: read from the
 * current one, write into the spare, then just swap which is which.
 * Swapping is instant; copying the whole grid every step would not be.
 *
 * The arrays are sized to the terminal at start-up (and re-made on
 * resize), so they're allocated once on the heap rather than reserved
 * for a worst-case window.  Nothing is allocated while the sim runs.
 *
 *   cols, rows         grid size in cells
 *   substrate_u        current U field
 *   catalyst_v         current V field
 *   substrate_u_next   spare for U — becomes current after the swap
 *   catalyst_v_next    spare for V — becomes current after the swap
 *   active_preset_*    which preset / theme is live (survives resize)
 *   active_theme_index   "
 *   auto_theme_frame_counter  ticks toward the next automatic theme swap
 */
typedef struct {
    int cols;
    int rows;

    float *substrate_u;
    float *catalyst_v;
    float *substrate_u_next;
    float *catalyst_v_next;

    int active_preset_index;
    int active_theme_index;
    int auto_theme_frame_counter;
} Grid;

static void grid_buffers_alloc(Grid *grid, int cols, int rows) {
  size_t cell_count = (size_t)cols * (size_t)rows;
  grid->cols = cols;
  grid->rows = rows;
  grid->substrate_u = malloc(cell_count * sizeof(float));
  grid->catalyst_v = malloc(cell_count * sizeof(float));
  grid->substrate_u_next = malloc(cell_count * sizeof(float));
  grid->catalyst_v_next = malloc(cell_count * sizeof(float));
}

static void grid_buffers_free(Grid *grid) {
  free(grid->substrate_u);
  free(grid->catalyst_v);
  free(grid->substrate_u_next);
  free(grid->catalyst_v_next);
  memset(grid, 0, sizeof *grid);
}

static void grid_buffers_resize(Grid *grid, int cols, int rows) {
  int saved_preset = grid->active_preset_index;
  int saved_theme = grid->active_theme_index;
  grid_buffers_free(grid);
  grid_buffers_alloc(grid, cols, rows);
  grid->active_preset_index = saved_preset;
  grid->active_theme_index = saved_theme;
}

/* ── §9  laplacian — how much a cell differs from its neighbours ── */
/* This is the "spreading" term: it measures whether a cell sits in a dip
 * or on a bump compared with the eight cells around it, which tells the
 * chemical which way to flow.  We weight the four straight neighbours
 * more than the four corners (4:1); that blend keeps spots round instead
 * of growing into diamonds. */

#define LAPLACIAN_CARDINAL_WEIGHT 0.20f
#define LAPLACIAN_DIAGONAL_WEIGHT 0.05f

/* ── §10  reaction_step — advance the whole grid by one step ── */

/*
 * ToroidalNeighbours — the four nearest cells, with the edges stitched.
 *
 * The grid wraps: fall off the right edge and you come back on the left,
 * off the top and you reappear at the bottom — like a map of a globe.
 * That means no special handling at the borders; every cell has four
 * real neighbours.
 *
 *   x_left, x_right   columns either side, wrapped
 *   y_above, y_below  rows above and below, wrapped
 */
typedef struct {
    int x_left, x_right;
    int y_above, y_below;
} ToroidalNeighbours;

static inline ToroidalNeighbours toroidal_neighbours_of(int x, int y,
                                                         int cols, int rows) {
    ToroidalNeighbours n;
    n.x_left  = (x == 0)        ? cols - 1 : x - 1;
    n.x_right = (x == cols - 1) ? 0        : x + 1;
    n.y_above = (y == 0)        ? rows - 1 : y - 1;
    n.y_below = (y == rows - 1) ? 0        : y + 1;
    return n;
}

/* The spreading term for one cell: a weighted average of the eight
 * neighbours minus the cell itself (see §9 for why the weights). */
static inline float laplacian_9point_at(const float *field, int cols,
                                         int x, int y,
                                         ToroidalNeighbours n) {
    int   idx = y * cols + x;
    float cardinal_sum =
        field[y         * cols + n.x_right] +
        field[y         * cols + n.x_left ] +
        field[n.y_below * cols + x        ] +
        field[n.y_above * cols + x        ];
    float diagonal_sum =
        field[n.y_below * cols + n.x_right] +
        field[n.y_below * cols + n.x_left ] +
        field[n.y_above * cols + n.x_right] +
        field[n.y_above * cols + n.x_left ];
    return LAPLACIAN_CARDINAL_WEIGHT * cardinal_sum
         + LAPLACIAN_DIAGONAL_WEIGHT * diagonal_sum
         - field[idx];
}

/* The reaction: where U meets V, V eats U and makes more of itself.
 * This one quantity drives both chemicals (U loses it, V gains it), so
 * we work it out once per cell and reuse it. */
static inline float gray_scott_reaction_term(float u, float v) {
    return u * v * v;
}

/* New U for one cell: spread it around, subtract what the reaction ate,
 * and top it back up (the feed slows as U approaches full). */
static inline float euler_step_u(float u, float laplacian_u,
                                  float reaction_term, float feed_rate) {
    return u + EULER_DT * (DIFFUSION_U * laplacian_u
                            - reaction_term
                            + feed_rate * (1.0f - u));
}

/* New V for one cell: spread it around, add what the reaction made, and
 * let some of it die off (feed washes it out, kill decays it). */
static inline float euler_step_v(float v, float laplacian_v,
                                  float reaction_term,
                                  float feed_rate, float kill_rate) {
    return v + EULER_DT * (DIFFUSION_V * laplacian_v
                            + reaction_term
                            - (feed_rate + kill_rate) * v);
}

/* Keep an amount within 0..1.  Each step can nudge slightly past the
 * ends; this just trims that overshoot. */
static inline float clamp_unit_interval(float x) {
    if (x < 0.0f) return 0.0f;
    if (x > 1.0f) return 1.0f;
    return x;
}

/* Make the freshly-written spare arrays the current ones, by swapping
 * pointers — no copying (see the Grid notes). */
static inline void swap_in_next_buffers(Grid *grid) {
    float *tmp;
    tmp = grid->substrate_u;
    grid->substrate_u      = grid->substrate_u_next;
    grid->substrate_u_next = tmp;
    tmp = grid->catalyst_v;
    grid->catalyst_v       = grid->catalyst_v_next;
    grid->catalyst_v_next  = tmp;
}

/* Work out every cell's next U and V into the spare arrays, then swap
 * them in.  This is the core loop the whole demo is built around. */
static void reaction_step(Grid *grid) {
    const int   cols = grid->cols, rows = grid->rows;
    const float feed_rate = preset_table[grid->active_preset_index].feed_rate;
    const float kill_rate = preset_table[grid->active_preset_index].kill_rate;

    for (int y = 0; y < rows; y++) {
        for (int x = 0; x < cols; x++) {
            ToroidalNeighbours n   = toroidal_neighbours_of(x, y, cols, rows);
            int                idx = y * cols + x;
            float              u   = grid->substrate_u[idx];
            float              v   = grid->catalyst_v [idx];

            float lap_u    = laplacian_9point_at(grid->substrate_u, cols, x, y, n);
            float lap_v    = laplacian_9point_at(grid->catalyst_v,  cols, x, y, n);
            float reaction = gray_scott_reaction_term(u, v);

            float u_new = clamp_unit_interval(
                            euler_step_u(u, lap_u, reaction, feed_rate));
            float v_new = clamp_unit_interval(
                            euler_step_v(v, lap_v, reaction, feed_rate, kill_rate));

            grid->substrate_u_next[idx] = u_new;
            grid->catalyst_v_next [idx] = v_new;
        }
    }

    swap_in_next_buffers(grid);
}

/* ── §11  seed — set the starting conditions + drop a blob ── */
/* Start from a flat field (all U, no V), then drop a few squares of V to
 * kick things off — the pattern grows outward from those. */

/* Wrap a coordinate onto the grid, handling negatives.  Plain `%` in C
 * goes negative for negative inputs, so we add n once to fix it. */
static inline int wrap_toroidal_modulo(int x, int n) {
    return ((x % n) + n) % n;
}

/* Turn one cell into a starter cell: no U, full V.  This little excess
 * of V is the spark every pattern grows from. */
static inline void seed_one_cell_to_catalyst(Grid *grid, int x, int y) {
    grid->substrate_u[y * grid->cols + x] = 0.0f;
    grid->catalyst_v [y * grid->cols + x] = 1.0f;
}

/* Drop a small square of V centred on a point (wrapping at the edges). */
static void place_seed_blob(Grid *grid, int centre_x, int centre_y) {
    for (int dy = -SEED_BLOB_HALF_WIDTH; dy <= SEED_BLOB_HALF_WIDTH; dy++) {
        for (int dx = -SEED_BLOB_HALF_WIDTH; dx <= SEED_BLOB_HALF_WIDTH; dx++) {
            int x = wrap_toroidal_modulo(centre_x + dx, grid->cols);
            int y = wrap_toroidal_modulo(centre_y + dy, grid->rows);
            seed_one_cell_to_catalyst(grid, x, y);
        }
    }
}

/* Wipe the grid to its flat resting state: U everywhere, no V.  Left
 * like this it never changes — it needs a bit of V (the seed blobs) to
 * get anything going. */
static inline void reset_grid_to_equilibrium(Grid *grid) {
    int cell_count = grid->cols * grid->rows;
    for (int i = 0; i < cell_count; i++) {
        grid->substrate_u[i] = 1.0f;
        grid->catalyst_v [i] = 0.0f;
    }
}

/* Split the grid into one column-slot per blob and work out how much
 * each blob may wander from its slot's centre, so they end up spread out
 * but not perfectly regular.  The clamps keep some wiggle room on tiny
 * grids. */
static inline void compute_blob_slot_geometry(int cols, int rows, int blob_count,
                                               int *out_slot_width,
                                               int *out_x_jitter_max,
                                               int *out_y_jitter_max,
                                               int *out_y_centre) {
    *out_slot_width   = cols / blob_count;
    *out_x_jitter_max = *out_slot_width / 2;
    if (*out_x_jitter_max < 1) *out_x_jitter_max = 1;
    *out_y_centre     = rows / 2;
    *out_y_jitter_max = rows / 4;
    if (*out_y_jitter_max < 1) *out_y_jitter_max = 1;
}

/* Pick a randomly nudged spot for one blob inside its slot, around the
 * vertical middle of the grid.  Coords are wrapped so a nudge past the
 * edge stays on the grid. */
static inline void pick_jittered_blob_centre(int slot_index,
                                              int cols, int rows,
                                              int slot_width,
                                              int x_jitter_max,
                                              int y_jitter_max,
                                              int y_centre,
                                              int *out_cx, int *out_cy) {
    int x_base   = slot_index * slot_width + slot_width / 2;
    int x_jitter = rand_in_range(-x_jitter_max, x_jitter_max + 1);
    int y_jitter = rand_in_range(-y_jitter_max, y_jitter_max + 1);
    *out_cx = wrap_toroidal_modulo(x_base   + x_jitter, cols);
    *out_cy = wrap_toroidal_modulo(y_centre + y_jitter, rows);
}

/* Wipe to the flat state, then scatter the preset's starter blobs. */
static void grid_seed_initial_conditions(Grid *grid) {
    reset_grid_to_equilibrium(grid);

    int blob_count = preset_table[grid->active_preset_index].seed_blob_count;
    if (blob_count < 1) blob_count = 1;

    int slot_width, x_jitter_max, y_jitter_max, y_centre;
    compute_blob_slot_geometry(grid->cols, grid->rows, blob_count,
                               &slot_width, &x_jitter_max, &y_jitter_max,
                               &y_centre);

    for (int s = 0; s < blob_count; s++) {
        int cx, cy;
        pick_jittered_blob_centre(s, grid->cols, grid->rows,
                                   slot_width, x_jitter_max, y_jitter_max,
                                   y_centre, &cx, &cy);
        place_seed_blob(grid, cx, cy);
    }
}

/* ── §12  warmup — run ahead before the first frame ── */
/* Right after seeding the grid is just a few dots.  Run a stretch of
 * steps unseen so the user opens onto a grown pattern. */
static void grid_warmup(Grid *grid) {
  for (int i = 0; i < WARMUP_TICK_COUNT; i++)
    reaction_step(grid);
}

/* Start fresh: re-seed and warm up again. */
static void grid_reseed(Grid *grid) {
  grid_seed_initial_conditions(grid);
  grid_warmup(grid);
}

static void grid_init(Grid *grid, int cols, int rows, int preset_index,
                      int theme_index) {
  grid_buffers_alloc(grid, cols, rows);
  grid->active_preset_index = preset_index;
  grid->active_theme_index = theme_index;
  grid->auto_theme_frame_counter = 0;
  grid_reseed(grid);
}

/* ── §13  glyph_picker — turn a cell's V into how to draw it ── */

/*
 * CellRender — what to draw for one cell, worked out from its V amount.
 *
 * Bundling the character, its colour, and a "skip" flag together keeps
 * the drawing loop short.  The skip flag marks near-empty cells we leave
 * blank — letting the background show through makes the pattern's edges
 * read more clearly, and it saves drawing work since most cells are
 * empty.
 *
 *   glyph  the character to print
 *   attr   its colour (and bold on the brightest cells)
 *   skip   true means leave this cell blank
 */
typedef struct {
    char   glyph;
    attr_t attr;
    bool   skip;
} CellRender;

/* Stretch a small raw V into the 0..1 display range.  V rarely climbs
 * far, so we scale it up to reach the bright end of the ramp, then trim
 * anything that overshoots 1. */
static inline float contrast_stretch_v_to_display(float v) {
    float scaled = v * CATALYST_DISPLAY_SCALE;
    if (scaled > 1.0f) return 1.0f;
    if (scaled < 0.0f) return 0.0f;
    return scaled;
}

/* Build the draw instruction for a ramp level.  Level 0 means "leave it
 * blank" so the background shows through. */
static inline CellRender cell_render_for_slot(int slot) {
    if (slot == 0) return (CellRender){ .skip = true };
    return (CellRender){
        .glyph = ramp_glyph_table[slot],
        .attr  = attribute_for_slot(slot),
        .skip  = false,
    };
}

/* From a cell's V amount, decide the character and colour to draw. */
static CellRender pick_cell_render(float v) {
    float v_display = contrast_stretch_v_to_display(v);
    int   slot      = glyph_slot_for(v_display);
    return cell_render_for_slot(slot);
}

/* ── §14  render_grid — paint the V field to the terminal ── */

/* Count down to the next automatic theme change; when it fires, switch
 * palette and report back true so the caller can wipe the screen first
 * (otherwise the old theme's leftover cells linger). */
static inline bool maybe_advance_auto_theme(Grid *grid) {
    grid->auto_theme_frame_counter++;
    if (grid->auto_theme_frame_counter < AUTO_THEME_CYCLE_FRAMES)
        return false;
    grid->auto_theme_frame_counter = 0;
    grid->active_theme_index = (grid->active_theme_index + 1) % THEME_COUNT;
    apply_theme(grid->active_theme_index);
    return true;
}

/* Draw one cell from its V amount.  Near-empty cells are skipped
 * entirely — they're the majority, so skipping them keeps things fast. */
static inline void paint_one_v_cell(int y, int x, float v) {
    CellRender cr = pick_cell_render(v);
    if (cr.skip) return;
    attron(cr.attr);
    mvaddch(y, x, (chtype)(unsigned char)cr.glyph);
    attroff(cr.attr);
}

/* Draw the whole field, clipped to whichever is smaller — the grid or
 * the terminal — so a shrunk window just shows part of the field rather
 * than spilling over. */
static inline void paint_v_field(const Grid *grid, int term_cols, int term_rows) {
    int cols = grid->cols, rows = grid->rows;
    for (int y = 0; y < rows && y < term_rows; y++)
        for (int x = 0; x < cols && x < term_cols; x++)
            paint_one_v_cell(y, x, grid->catalyst_v[y * cols + x]);
}

/* Maybe rotate the theme, then paint the field.  Returns true if the
 * theme just changed, so the caller knows to wipe the screen. */
static bool render_grid(Grid *grid, int term_cols, int term_rows) {
    bool theme_just_changed = maybe_advance_auto_theme(grid);
    paint_v_field(grid, term_cols, term_rows);
    return theme_just_changed;
}

/* ── §15  scene — the live demo state and per-frame update ── */

/*
 * Scene — everything the running demo needs in one place.
 *
 * It holds the simulation (the grid of U and V) plus a few control
 * flags the main loop checks each frame.
 *
 * needs_clear is deliberately a draw-only flag: it asks for a screen
 * wipe before the next paint (after a theme change or reset) and has
 * nothing to do with the chemistry — wiping the screen never touches the
 * U/V values, so theme changes leave the pattern untouched.
 *
 *   grid                        the U/V fields + which preset/theme is on
 *   simulation_paused           true freezes the simulation
 *   simulation_steps_per_frame  sim steps per drawn frame (the speed knob)
 *   needs_clear                 ask for a full screen wipe next frame
 */
typedef struct {
    Grid grid;
    bool simulation_paused;
    int  simulation_steps_per_frame;
    bool needs_clear;
} Scene;

static void scene_init(Scene *scene, int cols, int rows, int preset_index,
                       int theme_index) {
  memset(scene, 0, sizeof *scene);
  scene->simulation_steps_per_frame = STEPS_PER_FRAME_DEFAULT;
  grid_init(&scene->grid, cols, rows, preset_index, theme_index);
}

static void scene_free(Scene *scene) { grid_buffers_free(&scene->grid); }

static void scene_set_preset(Scene *scene, int new_preset_index) {
  new_preset_index =
      (new_preset_index % PRESET_COUNT + PRESET_COUNT) % PRESET_COUNT;
  scene->grid.active_preset_index = new_preset_index;
  scene->grid.auto_theme_frame_counter = 0;
  grid_reseed(&scene->grid);
  scene->needs_clear = true;
}

static void scene_cycle_theme(Scene *scene) {
  scene->grid.active_theme_index =
      (scene->grid.active_theme_index + 1) % THEME_COUNT;
  scene->grid.auto_theme_frame_counter = 0;
  apply_theme(scene->grid.active_theme_index);
  scene->needs_clear = true;
}

static void scene_resize(Scene *scene, int cols, int rows) {
  grid_buffers_resize(&scene->grid, cols, rows);
  grid_reseed(&scene->grid);
  scene->needs_clear = true;
}

static void scene_tick(Scene *scene) {
  if (scene->simulation_paused)
    return;
  for (int i = 0; i < scene->simulation_steps_per_frame; i++)
    reaction_step(&scene->grid);
}

/* ── §16  hud — status line on top, key hints on the bottom ── */

static void hud_paint_status(int term_cols, const Scene *scene, double fps) {
  const Grid *grid = &scene->grid;
  char buf[200];
  snprintf(buf, sizeof buf,
           " Reaction-Diffusion  %s  theme:%-6s  steps/frame:%2d  "
           "%5.1f fps  %s ",
           preset_table[grid->active_preset_index].display_name,
           theme_table[grid->active_theme_index].display_name,
           scene->simulation_steps_per_frame, fps,
           scene->simulation_paused ? "PAUSED " : "running");
  int slen = (int)strlen(buf);
  int sx = term_cols - slen;
  if (sx < 0)
    sx = 0;
  attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
  mvprintw(0, sx, "%s", buf);
  attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void hud_paint_hint(int term_rows) {
  attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0,
           " q:quit  spc:pause  r:reseed  s:seed  n/p:preset  "
           "t:theme  +/-:speed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* ── §17  screen — ncurses set-up, teardown, and present ── */

/*
 * Screen — just the terminal's size in cells.  ncurses keeps the actual
 * pixels; we only need the dimensions to place the HUD and clip the
 * field.
 *
 *   cols, rows  terminal width and height in character cells
 */
typedef struct {
    int cols;
    int rows;
} Screen;

static void screen_init(Screen *screen, int theme_index) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
  typeahead(-1);
  colors_init(theme_index);
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *screen) {
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, Scene *scene, double fps) {
  if (scene->needs_clear) {
    erase();
    scene->needs_clear = false;
  }

  bool theme_changed = render_grid(&scene->grid, screen->cols, screen->rows);
  if (theme_changed)
    scene->needs_clear = true;

  hud_paint_status(screen->cols, scene, fps);
  hud_paint_hint(screen->rows);

  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §18  app — the main loop, signals, and keys ── */

/*
 * App — the top-level state, kept as one global so the signal handlers
 * can reach it.  When a signal arrives, a handler just flips a flag here
 * and the main loop notices on its next pass.
 *
 * The two flags are the special sig_atomic_t type because that's the
 * only kind a signal handler is allowed to touch safely; volatile makes
 * sure the loop re-reads them rather than caching a stale copy.
 *
 *   scene        the simulation + control state
 *   screen       the terminal size
 *   running      cleared on quit signals (Ctrl-C, kill)
 *   need_resize  set when the terminal is resized
 */
typedef struct {
    Scene  scene;
    Screen screen;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App app_state;

static void on_signal_quit(int sig) {
  (void)sig;
  app_state.running = 0;
}
static void on_signal_resize(int sig) {
  (void)sig;
  app_state.need_resize = 1;
}

static bool app_handle_key(App *app, int ch) {
  Scene *scene = &app->scene;
  Grid *grid = &scene->grid;

  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case ' ':
    scene->simulation_paused = !scene->simulation_paused;
    break;

  case 'n':
  case 'N':
    scene_set_preset(scene, grid->active_preset_index + 1);
    break;
  case 'p':
  case 'P':
    scene_set_preset(scene, grid->active_preset_index + PRESET_COUNT - 1);
    break;

  case 't':
  case 'T':
    scene_cycle_theme(scene);
    break;

  case 'r':
  case 'R':
    grid_reseed(grid);
    scene->needs_clear = true;
    break;

  case 's':
  case 'S':
    place_seed_blob(grid, grid->cols / 2, grid->rows / 2);
    break;

  case '+':
  case '=':
    scene->simulation_steps_per_frame += STEPS_PER_FRAME_STEP;
    if (scene->simulation_steps_per_frame > STEPS_PER_FRAME_MAX)
      scene->simulation_steps_per_frame = STEPS_PER_FRAME_MAX;
    break;
  case '-':
    scene->simulation_steps_per_frame -= STEPS_PER_FRAME_STEP;
    if (scene->simulation_steps_per_frame < STEPS_PER_FRAME_MIN)
      scene->simulation_steps_per_frame = STEPS_PER_FRAME_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  srand((unsigned)time(NULL));
  atexit(screen_cleanup);
  signal(SIGINT, on_signal_quit);
  signal(SIGTERM, on_signal_quit);
  signal(SIGWINCH, on_signal_resize);

  App *app = &app_state;
  app->running = 1;

  screen_init(&app->screen, 0);
  scene_init(&app->scene, app->screen.cols, app->screen.rows, 0, 0);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;

  while (app->running) {
    int64_t frame_start_ns = clock_now_ns();

    /* ── input ── */
    int ch = getch();
    if (ch != ERR && !app_handle_key(app, ch)) {
      app->running = 0;
      break;
    }

    /* ── resize ── */
    if (app->need_resize) {
      screen_resize(&app->screen);
      scene_resize(&app->scene, app->screen.cols, app->screen.rows);
      app->need_resize = 0;
      prev_frame_ns = clock_now_ns();
    }

    /* ── dt + fps window ── */
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= FPS_RECOMPUTE_MS * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    /* ── physics + render ── */
    scene_tick(&app->scene);
    screen_present_frame(&app->screen, &app->scene, measured_fps);

    /* ── frame cap ── */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  scene_free(&app->scene);
  return 0;
}
