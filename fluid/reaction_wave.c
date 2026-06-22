/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_wave.c — FitzHugh-Nagumo excitable medium on a terminal grid
 *
 * Each cell is a tiny excitable spot, like a patch of nerve or heart
 * muscle: it rests quietly until a kick pushes it past a threshold,
 * then it "fires", spreads the firing to its neighbours, and needs a
 * recovery beat before it can fire again.  That recovery beat is why
 * the waves only travel outward — they form expanding rings, colliding
 * fronts, and rotating spirals (the Belousov-Zhabotinsky look).
 *
 * Two numbers per cell drive it: u, the fast "voltage" that fires, and
 * v, the slow "recovery" that drags it back to rest.  Only u spreads to
 * neighbours; v stays put.
 *
 * Sister file: fluid/reaction_diffusion.c — Gray-Scott patterns, same
 *   numerical shape (grid + Laplacian + forward Euler), different rules.
 * References: FitzHugh (1961) Biophys. J. 1(6) and Nagumo et al. (1962)
 *   Proc. IRE 50(10) define the model; LeVeque, "Finite Difference
 *   Methods for ODEs and PDEs" (2007) for the Euler + Laplacian update;
 *   Bourke (1997) for the brightness-ramp glyph idea.
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

/* §1  config — every tunable lives here, named, so nothing below is magic */

/* The four physical knobs of the model.  Together they sit in the
 * "excitable" sweet spot: one quiet rest state, fires only when kicked
 * past threshold, then recovers cleanly. */
#define FN_THRESHOLD_SHIFT 0.70f    /* a — sets where the rest state sits */
#define FN_INHIBITOR_FEEDBACK 0.80f /* b — how hard recovery pulls u down */
#define FN_TIMESCALE_RATIO 0.08f    /* how slow recovery v is vs u (~12x) */
#define ACTIVATOR_DIFFUSION 0.10f   /* how fast/far firing spreads; only u */

/* Time step for the simple Euler integrator.  Kept tiny — well under
 * the diffusion stability limit — because the cubic firing term blows
 * up at larger steps and leaves ugly ringing at wave fronts. */
#define EULER_DT 0.04f

/* The quiet rest state every cell starts in (the system's fixed point
 * for the a,b above). */
#define ACTIVATOR_REST (-1.20f)
#define INHIBITOR_REST (-0.625f)

/* The jolt a kick injects.  Set well past the firing threshold (~0) so
 * a tap always lights the cell up; a gentle nudge would just fade. */
#define SUPRATHRESHOLD_KICK_VALUE 2.00f

/* How many physics steps to run per drawn frame.  More = waves move
 * faster on screen (the +/- speed knob); one step per frame would crawl. */
#define SUBSTEPS_PER_FRAME_DEFAULT 8
#define SUBSTEPS_PER_FRAME_MIN 1
#define SUBSTEPS_PER_FRAME_MAX 20

#define RENDER_FPS_CAP 30
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define RENDER_TICK_NS (NS_PER_SEC / RENDER_FPS_CAP)

#define GRID_COLS_MAX 320
#define GRID_ROWS_MAX 100
#define HUD_RESERVED_ROWS_TOP 1    /* row 0 — status line */
#define HUD_RESERVED_ROWS_BOTTOM 1 /* bottom row — key hint */

/* Cutoffs that turn a cell's u value into one of five brightness bands
 * (see §11): below the first → rest/blank, above the last → bright front. */
#define U_BAND_REST_HIGH (-0.80f)
#define U_BAND_RECOVERY_HIGH (-0.20f)
#define U_BAND_RISING_HIGH 0.50f
#define U_BAND_WAVE_HIGH 1.20f

#define IMPULSE_RADIUS_CELLS 4  /* size of a SPACE-key kick */
#define SPIRAL_PRIMER_RADIUS 5  /* size of the ring the spiral preset breaks */

/* Colour pairs, dim rest through bright firing front, plus the two HUD
 * colours. */
enum {
  PAIR_BAND_REST = 1, /* dark blue */
  PAIR_BAND_RECOVERY, /* medium blue */
  PAIR_BAND_RISING,   /* cyan */
  PAIR_BAND_WAVE,     /* pale cyan-white */
  PAIR_BAND_FRONT,    /* bright white + bold */
  PAIR_HUD,           /* bright yellow + bold (top) */
  PAIR_HINT,          /* bright cyan + bold (bottom) */
};

#define PRESET_COUNT 4

/* §2  clock — read the time and sleep, in nanoseconds */

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

/* §3  colors — the five-band ramp plus HUD pairs (8-colour fallback) */

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_BAND_REST, 25, -1);     /* dark blue        */
    init_pair(PAIR_BAND_RECOVERY, 27, -1); /* medium blue      */
    init_pair(PAIR_BAND_RISING, 51, -1);   /* cyan             */
    init_pair(PAIR_BAND_WAVE, 195, -1);    /* pale cyan-white  */
    init_pair(PAIR_BAND_FRONT, 231, -1);   /* bright white     */
    init_pair(PAIR_HUD, 226, -1);          /* bright yellow    */
    init_pair(PAIR_HINT, 51, -1);          /* bright cyan      */
  } else {
    init_pair(PAIR_BAND_REST, COLOR_BLUE, -1);
    init_pair(PAIR_BAND_RECOVERY, COLOR_BLUE, -1);
    init_pair(PAIR_BAND_RISING, COLOR_CYAN, -1);
    init_pair(PAIR_BAND_WAVE, COLOR_CYAN, -1);
    init_pair(PAIR_BAND_FRONT, COLOR_WHITE, -1);
    init_pair(PAIR_HUD, COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, COLOR_CYAN, -1);
  }
}

/* §4  scene — everything the demo's live state lives in one struct */
/*
 * Scene — all of this demo's running state, grouped in one place.
 *
 * The model tracks two values per cell, so we keep two grids: u (the
 * fast firing "voltage") and v (the slow "recovery").  Each gets a
 * twin scratch grid because a step has to read the OLD values of every
 * cell while writing the new ones — if we overwrote in place, a cell's
 * update would leak into its neighbours' within the same sweep.  So we
 * write the new values into the *_next grids, then copy them back.
 *
 * We copy rather than swap pointers because these are plain fixed-size
 * 2-D arrays, not pointers you can rotate; a swap would mean heap
 * allocation, which this project avoids after startup.  The copy is the
 * same cost order as the step itself, so it doesn't matter.
 *
 * One struct, one global instance (g_scene): the inner loop touches the
 * whole field set every step, so threading pointers through it would
 * just slow the hot path.  Fields are grouped by who uses them — the
 * physics step, the renderer, or the keyboard controls — so it's clear
 * which side owns each one.  (V = membrane voltage, W = recovery, in
 * FitzHugh's original two-variable form.)
 */
typedef struct {
    /* The simulation state — the two value grids and their scratch twins. */
    float activator_voltage      [GRID_ROWS_MAX][GRID_COLS_MAX]; /* u: fires */
    float inhibitor_recovery     [GRID_ROWS_MAX][GRID_COLS_MAX]; /* v: recovers */
    float activator_voltage_next [GRID_ROWS_MAX][GRID_COLS_MAX]; /* next u */
    float inhibitor_recovery_next[GRID_ROWS_MAX][GRID_COLS_MAX]; /* next v */

    /* How much of the grid is actually in use (clipped to the terminal). */
    int grid_active_rows;
    int grid_active_cols;

    /* Controls: which preset is loaded, and whether the sim is frozen. */
    int  active_preset_index;
    bool simulation_paused;

    /* Speed knob: physics steps per drawn frame (doesn't affect stability). */
    int substeps_per_frame;
} Scene;

static Scene g_scene = {
    .substeps_per_frame = SUBSTEPS_PER_FRAME_DEFAULT,
};

/* §5  neighbours — picking a neighbour, but stopping at the edges */
/*
 * At the grid edge a neighbour would fall off the grid; we hand back the
 * edge cell itself instead.  Practical effect: waves bounce off the
 * walls rather than vanishing or wrapping around.
 */

static inline int row_above_clamped(int row) { return (row > 0) ? row - 1 : 0; }

static inline int row_below_clamped(int row) {
  return (row < g_scene.grid_active_rows - 1) ? row + 1 : g_scene.grid_active_rows - 1;
}

static inline int col_left_clamped(int col) { return (col > 0) ? col - 1 : 0; }

static inline int col_right_clamped(int col) {
  return (col < g_scene.grid_active_cols - 1) ? col + 1 : g_scene.grid_active_cols - 1;
}

/* §6  laplacian — how much a cell differs from its four neighbours */
/*
 * This number says whether a cell is a bump or a dip relative to the
 * cells around it; that's what makes firing spread outward.  It's the
 * four side neighbours added up, minus four times the centre.
 */

#define LAPLACIAN_5POINT_CENTRE_WEIGHT 4.0f

/* §7  reaction_step — advance every cell one tiny time step */

/* The four neighbour row/column indices for one cell, already pinned to
 * the grid edges so reading them never falls off (see §5). */
typedef struct {
    int row_above, row_below;
    int col_left,  col_right;
} ClampedNeighbours;

static inline ClampedNeighbours clamped_neighbours_of(int r, int c) {
    ClampedNeighbours n;
    n.row_above = row_above_clamped(r);
    n.row_below = row_below_clamped(r);
    n.col_left  = col_left_clamped (c);
    n.col_right = col_right_clamped(c);
    return n;
}

/* Sum the four side neighbours and subtract four times the centre — how
 * much this cell stands out from its surroundings.  Only the four sides
 * count (no diagonals), so growing rings look a touch diamond-shaped near
 * the source; harmless here.  LeVeque (2007) §10. */
static inline float laplacian_5point_at(const float field[GRID_ROWS_MAX][GRID_COLS_MAX],
                                         int r, int c, ClampedNeighbours n) {
    return field[n.row_above][c           ]
         + field[n.row_below][c           ]
         + field[r           ][n.col_left ]
         + field[r           ][n.col_right]
         - LAPLACIAN_5POINT_CENTRE_WEIGHT * field[r][c];
}

/* How fast u (the firing value) is changing right now.  The u³ term is
 * the trick: below threshold u settles back down, above it u runs away
 * and the cell fires — until recovery v catches up and drags it back.
 * FitzHugh (1961). */
static inline float fitzhugh_nagumo_du_dt(float u, float v, float laplacian_u) {
    return u - (u * u * u) / 3.0f - v + ACTIVATOR_DIFFUSION * laplacian_u;
}

/* How fast v (recovery) is changing.  It slowly chases u upward, and
 * that lag is what gives each cell its rest-then-fire-then-recover beat.
 * FitzHugh (1961), Nagumo et al. (1962). */
static inline float fitzhugh_nagumo_dv_dt(float u, float v) {
    return FN_TIMESCALE_RATIO * (u + FN_THRESHOLD_SHIFT
                                   - FN_INHIBITOR_FEEDBACK * v);
}

/* Take one small step: nudge the value by (rate of change × time step).
 * The simplest way to advance in time; works because EULER_DT is small. */
static inline float forward_euler_step(float value, float derivative) {
    return value + EULER_DT * derivative;
}

/* Move the freshly-computed next-grids back into the live grids (see the
 * Scene note for why we copy instead of swapping pointers). */
static inline void copy_scratches_to_live_fields(void) {
    memcpy(g_scene.activator_voltage,    g_scene.activator_voltage_next,
           sizeof g_scene.activator_voltage);
    memcpy(g_scene.inhibitor_recovery,   g_scene.inhibitor_recovery_next,
           sizeof g_scene.inhibitor_recovery);
}

/* Advance the whole grid one time step: for each cell work out how u and
 * v are changing, nudge both, write into the next-grids, then swap them in. */
static void reaction_step(void) {
    for (int r = 0; r < g_scene.grid_active_rows; r++) {
        for (int c = 0; c < g_scene.grid_active_cols; c++) {
            ClampedNeighbours n = clamped_neighbours_of(r, c);
            float u             = g_scene.activator_voltage  [r][c];
            float v             = g_scene.inhibitor_recovery [r][c];

            float lap_u         = laplacian_5point_at(g_scene.activator_voltage,
                                                      r, c, n);
            float du_dt         = fitzhugh_nagumo_du_dt(u, v, lap_u);
            float dv_dt         = fitzhugh_nagumo_dv_dt(u, v);

            g_scene.activator_voltage_next [r][c] = forward_euler_step(u, du_dt);
            g_scene.inhibitor_recovery_next[r][c] = forward_euler_step(v, dv_dt);
        }
    }

    copy_scratches_to_live_fields();
}

/* §8  reset — set the whole grid back to the quiet rest state */
/*
 * Every cell goes to its resting values; the grid then sits still until
 * a kick or a preset injects something.
 */

static void grid_reset_to_rest(void) {
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols; c++) {
      g_scene.activator_voltage[r][c] = ACTIVATOR_REST;
      g_scene.inhibitor_recovery[r][c] = INHIBITOR_REST;
    }
  }
}

/* §9  impulse — light up a round patch of cells (the "kick") */
/*
 * Set a disc of cells to the firing value; they ignite at once and ripple
 * outward as a ring, like dropping a stone in water.  We leave v (recovery)
 * alone so each cell starts its rest-fire-recover beat naturally.
 */

static inline bool cell_in_active_grid(int r, int c) {
    return r >= 0 && r < g_scene.grid_active_rows
        && c >= 0 && c < g_scene.grid_active_cols;
}

/* Inside the round patch?  Compares squared distances, which dodges a
 * square root and is still an exact circle (unlike a diamond test). */
static inline bool cell_inside_disc(int r, int c, int rc, int cc,
                                     int radius_squared) {
    int dr = r - rc;
    int dc = c - cc;
    return dr * dr + dc * dc <= radius_squared;
}

static inline void fire_one_cell(int r, int c) {
    g_scene.activator_voltage[r][c] = SUPRATHRESHOLD_KICK_VALUE;
}

/* Fire every cell inside a disc of the given radius around (rc, cc). */
static void inject_circular_impulse(int rc, int cc, int radius) {
    int radius_squared = radius * radius;
    for (int r = rc - radius; r <= rc + radius; r++) {
        for (int c = cc - radius; c <= cc + radius; c++) {
            if (!cell_in_active_grid (r, c))                          continue;
            if (!cell_inside_disc    (r, c, rc, cc, radius_squared))  continue;
            fire_one_cell(r, c);
        }
    }
}

/* §10  presets — the four starting scenes the number keys load */
/*
 * Preset — one named starting scene (rings, two sources, spiral, plane).
 *
 * Pressing a number key calls that preset's loader, which paints a
 * starting pattern into the grid.  Each is just a different shape of
 * initial firing — so instead of sharing numeric parameters, each preset
 * carries its own loader function that draws whatever it needs.
 * Murray ch.5-7 and Tyson & Keener (1988) for the spiral behaviour.
 */
typedef struct {
    const char *display_name;        /* short label shown in the HUD */
    void      (*loader)(void);       /* paints this preset's starting pattern */
} Preset;

static void preset_target_rings(void);
static void preset_double_source(void);
static void preset_spiral_wave(void);
static void preset_plane_wave(void);

static const Preset preset_table[PRESET_COUNT] = {
    {"TARGET RINGS  ", preset_target_rings},
    {"DOUBLE SOURCE ", preset_double_source},
    {"SPIRAL WAVE   ", preset_spiral_wave},
    {"PLANE WAVE    ", preset_plane_wave},
};

/* One kick at the centre — expands into concentric rings. */
static void preset_target_rings(void) {
  grid_reset_to_rest();
  inject_circular_impulse(g_scene.grid_active_rows / 2, g_scene.grid_active_cols / 2,
                          IMPULSE_RADIUS_CELLS);
}

/* Two kicks side by side — their fronts meet in the middle and cancel
 * out (each front is trailed by cells that can't fire yet). */
static void preset_double_source(void) {
  grid_reset_to_rest();
  int row = g_scene.grid_active_rows / 2;
  inject_circular_impulse(row, g_scene.grid_active_cols / 3, IMPULSE_RADIUS_CELLS);
  inject_circular_impulse(row, g_scene.grid_active_cols * 2 / 3, IMPULSE_RADIUS_CELLS);
}

/* A ring with one side wiped off.  The two loose ends left behind curl
 * back on themselves and settle into a steadily rotating spiral — the
 * classic excitable-medium pattern. */
static void preset_spiral_wave(void) {
  grid_reset_to_rest();
  inject_circular_impulse(g_scene.grid_active_rows / 2, g_scene.grid_active_cols / 2,
                          SPIRAL_PRIMER_RADIUS);
  /* Wipe the left half so the ring is broken open. */
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    for (int c = 0; c < g_scene.grid_active_cols / 2; c++) {
      if (g_scene.activator_voltage[r][c] > ACTIVATOR_REST + 0.5f)
        g_scene.activator_voltage[r][c] = ACTIVATOR_REST;
    }
  }
}

/* Light the whole left edge — a flat front that sweeps across the grid. */
static void preset_plane_wave(void) {
  grid_reset_to_rest();
  for (int r = 0; r < g_scene.grid_active_rows; r++) {
    g_scene.activator_voltage[r][0] = SUPRATHRESHOLD_KICK_VALUE;
    g_scene.activator_voltage[r][1] = SUPRATHRESHOLD_KICK_VALUE;
  }
}

static void preset_load(int preset_index) {
  if (preset_index < 0 || preset_index >= PRESET_COUNT)
    preset_index = 0;
  g_scene.active_preset_index = preset_index;
  preset_table[preset_index].loader();
}

/* §11  glyph_picker — turn a cell's u value into a character + colour */

/*
 * CellRender — how to draw one cell: which character, which colour, and
 * whether to draw it at all.
 *
 * A cell's u value (its firing level) decides everything here — quiet
 * cells get skipped so the background shows through and the wave fronts
 * stand out; brighter cells get denser, bolder glyphs.  Bundling the
 * choice into one struct keeps the draw loop a simple "pick, then draw
 * unless skip".  Brightness-ramp idea from Bourke (1997).
 */
typedef struct {
    chtype glyph;        /* the character to draw */
    int    pair;         /* its colour pair */
    attr_t extra_attr;   /* A_BOLD on the brightest band, else nothing */
    bool   skip;         /* true → leave the cell blank */
} CellRender;

/* Resting cells: draw nothing, let the background show through. */
static inline CellRender cell_render_rest_tier(void) {
    return (CellRender){ .skip = true };
}

/* Just-cooled cells, still recovering — faint '.'. */
static inline CellRender cell_render_recovery_tier(void) {
    return (CellRender){ .glyph = '.', .pair = PAIR_BAND_RECOVERY,
                         .extra_attr = A_NORMAL, .skip = false };
}

/* Climbing toward firing — ':'. */
static inline CellRender cell_render_rising_tier(void) {
    return (CellRender){ .glyph = ':', .pair = PAIR_BAND_RISING,
                         .extra_attr = A_NORMAL, .skip = false };
}

/* The body of the wave — '+'. */
static inline CellRender cell_render_wave_tier(void) {
    return (CellRender){ .glyph = '+', .pair = PAIR_BAND_WAVE,
                         .extra_attr = A_NORMAL, .skip = false };
}

/* The bright leading edge — bold '#'. */
static inline CellRender cell_render_front_tier(void) {
    return (CellRender){ .glyph = '#', .pair = PAIR_BAND_FRONT,
                         .extra_attr = A_BOLD,   .skip = false };
}

/* Sort a cell into one of five brightness bands by its u value, dim rest
 * through bright firing front. */
static CellRender pick_cell_render(float u_value) {
    if (u_value < U_BAND_REST_HIGH    ) return cell_render_rest_tier();
    if (u_value < U_BAND_RECOVERY_HIGH) return cell_render_recovery_tier();
    if (u_value < U_BAND_RISING_HIGH  ) return cell_render_rising_tier();
    if (u_value < U_BAND_WAVE_HIGH    ) return cell_render_wave_tier();
    return                                     cell_render_front_tier();
}

/* §12  render — draw the firing field, skipping the quiet cells */

/* Work out the rectangle the field gets to use: leave a row for the HUD
 * at top and bottom, and never reach past either the terminal or the
 * active grid (the smaller one wins).  Hands back the top row to start at
 * and how many rows/columns to paint. */
static inline void compute_activator_draw_rect(int term_rows, int term_cols,
                                                int *out_draw_top,
                                                int *out_max_rows,
                                                int *out_max_cols) {
    int draw_top    = HUD_RESERVED_ROWS_TOP;
    int draw_bottom = term_rows - HUD_RESERVED_ROWS_BOTTOM;
    int avail_rows  = draw_bottom - draw_top;

    *out_draw_top = draw_top;
    *out_max_rows = (g_scene.grid_active_rows < avail_rows)
                  ?  g_scene.grid_active_rows
                  :  avail_rows;
    *out_max_cols = (g_scene.grid_active_cols < term_cols)
                  ?  g_scene.grid_active_cols
                  :  term_cols;
}

/* Draw one cell at the given screen spot — unless it's a resting cell,
 * which we leave blank. */
static inline void paint_one_activator_cell(int display_row, int display_col,
                                             float u_value) {
    CellRender cr = pick_cell_render(u_value);
    if (cr.skip) return;
    attr_t a = COLOR_PAIR(cr.pair) | cr.extra_attr;
    attron(a);
    mvaddch(display_row, display_col, cr.glyph);
    attroff(a);
}

/* Paint the whole firing field, cell by cell, within the drawing area. */
static void render_activator_field(int term_rows, int term_cols) {
    int draw_top, max_rows, max_cols;
    compute_activator_draw_rect(term_rows, term_cols,
                                 &draw_top, &max_rows, &max_cols);

    for (int r = 0; r < max_rows; r++) {
        for (int c = 0; c < max_cols; c++) {
            paint_one_activator_cell(draw_top + r, c,
                                      g_scene.activator_voltage[r][c]);
        }
    }
}

/* §13  scene — set up, resize, and step the simulation */

static void scene_init(int term_rows, int term_cols) {
  g_scene.grid_active_rows = (term_rows < GRID_ROWS_MAX) ? term_rows : GRID_ROWS_MAX;
  g_scene.grid_active_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;
  g_scene.simulation_paused = false;
  g_scene.substeps_per_frame = SUBSTEPS_PER_FRAME_DEFAULT;
  preset_load(0);
}

static void scene_resize(int term_rows, int term_cols) {
  g_scene.grid_active_rows = (term_rows < GRID_ROWS_MAX) ? term_rows : GRID_ROWS_MAX;
  g_scene.grid_active_cols = (term_cols < GRID_COLS_MAX) ? term_cols : GRID_COLS_MAX;
  if (g_scene.grid_active_rows < 4)
    g_scene.grid_active_rows = 4;
  if (g_scene.grid_active_cols < 4)
    g_scene.grid_active_cols = 4;
  preset_load(g_scene.active_preset_index);
}

static void scene_tick(void) {
  if (g_scene.simulation_paused)
    return;
  for (int s = 0; s < g_scene.substeps_per_frame; s++)
    reaction_step();
}

/* §14  hud — status line up top, key hints along the bottom */

static void hud_paint_status(int term_cols, double measured_fps) {
  char buf[200];
  snprintf(buf, sizeof buf,
           " FitzHugh-Nagumo  preset:%s  speed:%dx  "
           "a=%.2f b=%.2f e=%.2f D=%.2f dt=%.3f  %5.1f fps  %s ",
           preset_table[g_scene.active_preset_index].display_name, g_scene.substeps_per_frame,
           (double)FN_THRESHOLD_SHIFT, (double)FN_INHIBITOR_FEEDBACK,
           (double)FN_TIMESCALE_RATIO, (double)ACTIVATOR_DIFFUSION,
           (double)EULER_DT, measured_fps,
           g_scene.simulation_paused ? "PAUSED " : "running");
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
           " q:quit  spc:impulse  p:pause  r:reset  "
           "1:rings  2:double  3:spiral  4:plane  +/-:speed ");
  attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

/* §15  screen — ncurses setup, teardown, and pushing a frame out */

/*
 * Screen — just the terminal's size in character cells.  ncurses holds
 * the actual draw buffers; we only need width and height to place the
 * HUD and clip the field.
 */
typedef struct {
    int rows;   /* terminal height in cells */
    int cols;   /* terminal width in cells */
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  cbreak();
  noecho();
  keypad(stdscr, TRUE);
  nodelay(stdscr, TRUE);
  curs_set(0);
  typeahead(-1);
  colors_init();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_cleanup(void) { endwin(); }

static void screen_resize(Screen *screen) {
  endwin();
  refresh();
  getmaxyx(stdscr, screen->rows, screen->cols);
}

static void screen_present_frame(Screen *screen, double measured_fps) {
  erase();
  render_activator_field(screen->rows, screen->cols);
  hud_paint_status(screen->cols, measured_fps);
  hud_paint_hint(screen->rows);
  wnoutrefresh(stdscr);
  doupdate();
}

/* §16  app — the main loop, signal handling, and keyboard input */

static volatile sig_atomic_t g_should_quit = 0;
static volatile sig_atomic_t g_resize_pending = 0;

static void on_signal(int sig) {
  if (sig == SIGWINCH)
    g_resize_pending = 1;
  else
    g_should_quit = 1;
}

static bool app_handle_key(int ch) {
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;

  case 'p':
  case 'P':
    g_scene.simulation_paused = !g_scene.simulation_paused;
    break;

  case ' ':
    inject_circular_impulse(g_scene.grid_active_rows / 2, g_scene.grid_active_cols / 2,
                            IMPULSE_RADIUS_CELLS);
    break;

  case 'r':
  case 'R':
    grid_reset_to_rest();
    break;

  case '1':
    preset_load(0);
    break;
  case '2':
    preset_load(1);
    break;
  case '3':
    preset_load(2);
    break;
  case '4':
    preset_load(3);
    break;

  case '+':
  case '=':
    g_scene.substeps_per_frame++;
    if (g_scene.substeps_per_frame > SUBSTEPS_PER_FRAME_MAX)
      g_scene.substeps_per_frame = SUBSTEPS_PER_FRAME_MAX;
    break;
  case '-':
    g_scene.substeps_per_frame--;
    if (g_scene.substeps_per_frame < SUBSTEPS_PER_FRAME_MIN)
      g_scene.substeps_per_frame = SUBSTEPS_PER_FRAME_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(screen_cleanup);
  signal(SIGINT, on_signal);
  signal(SIGTERM, on_signal);
  signal(SIGWINCH, on_signal);

  Screen screen;
  screen_init(&screen);
  scene_init(screen.rows, screen.cols);

  int64_t prev_frame_ns = clock_now_ns();
  int64_t fps_window_ns = 0;
  int frames_in_window = 0;
  double measured_fps = 0.0;

  while (!g_should_quit) {
    int64_t frame_start_ns = clock_now_ns();

    int ch = getch();
    if (ch != ERR && !app_handle_key(ch)) {
      g_should_quit = 1;
      break;
    }

    if (g_resize_pending) {
      g_resize_pending = 0;
      screen_resize(&screen);
      scene_resize(screen.rows, screen.cols);
      prev_frame_ns = clock_now_ns();
    }

    /* Time since last frame, capped so a long stall doesn't blow up the
     * fps average. */
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;

    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= 500 * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    scene_tick();
    screen_present_frame(&screen, measured_fps);

    /* Sleep off the rest of the frame's time budget to hold the fps cap. */
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < RENDER_TICK_NS)
      clock_sleep_ns(RENDER_TICK_NS - spent);
  }

  return 0;
}
