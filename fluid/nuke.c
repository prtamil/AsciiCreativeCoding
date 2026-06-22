/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * nuke.c — a rising mushroom cloud, in the terminal.
 *
 * Two pieces work together.  A small 2-D fluid simulation (Jos Stam's
 * "Stable Fluids") tracks how hot gas climbs, drifts, and cools.  A
 * volume raymarcher then spins that flat slice around its vertical
 * axis to draw a 3-D-looking cloud — real mushroom clouds are nearly
 * round, so one slice is enough.  Pick a blast size, a colour theme,
 * or a debug overlay to peek at the raw fluid fields.
 *
 * Sister files: fluid/navier_stokes.c (same solver, user-driven dye),
 * raymarcher/raymarcher.c (surface — not volume — raymarching),
 * fluid/fluid_sph.c (particle-based fluid for comparison).
 */

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

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ── §1 config — every tunable lives here, no magic numbers later ── */

/* §1.1 frame rate + UI layout. */
enum {
  TARGET_RENDER_FPS_MIN = 10,
  TARGET_RENDER_FPS_DEFAULT = 60,
  TARGET_RENDER_FPS_MAX = 120,
  TARGET_RENDER_FPS_STEP = 10,
  FPS_DISPLAY_UPDATE_MS = 500,
  HUD_RESERVED_ROWS = 2, /* row 0 status + last row hint */
};

/* §1.2 colour-pair IDs. */
enum {
  PAIR_HUD_STATUS = 1, /* top status row (yellow + bold)            */
  PAIR_HUD_HINT = 2,   /* bottom hint row (cyan + bold)             */
  PAIR_RAMP_BASE = 3,  /* +0..+7 = the smoke-to-fire colour ramp    */
};

/* §1.3 time helpers. */
#define NS_PER_SEC 1000000000LL
#define NS_PER_MS 1000000LL
#define TICK_NS(target_fps) (NS_PER_SEC / (target_fps))

/* §1.4 terminal cell aspect. */
#define TERMINAL_CELL_ASPECT 2.0f /* physical h / w */

/* §1.5 fluid grid — the flat radius-by-altitude slice we simulate. */
#define GRID_RADIAL_CELLS 56
#define GRID_VERTICAL_CELLS 96
#define GRID_CELL_SIZE 0.125f
#define GRID_RADIAL_EXTENT ((float)GRID_RADIAL_CELLS * GRID_CELL_SIZE)
#define GRID_VERTICAL_EXTENT ((float)GRID_VERTICAL_CELLS * GRID_CELL_SIZE)
#define GRID_INV_CELL_SIZE (1.0f / GRID_CELL_SIZE)

#define POISSON_JACOBI_ITERATIONS 40 /* relaxation passes per projection */

/* §1.6 physics constants. */
#define TEMPERATURE_AMBIENT 1.0f
#define TEMPERATURE_PEAK 8.0f
#define DENSITY_PEAK 4.0f
#define BUOYANCY_COEFFICIENT 2.4f /* how hard hot air pushes upward   */
#define COOL_RATE 0.06f           /* how fast heat fades to ambient   */
#define DENSITY_DECAY 0.009f      /* how fast smoke thins out per sec */

/* §1.7 simulation timing. */
#define SIM_DT 0.025f          /* fixed sim step (sim seconds) */
#define SIM_DT_MAX_REAL 0.080f /* cap per real frame           */
#define SIM_RATE_DEFAULT 1.0f
#define SIM_RATE_MIN 0.10f
#define SIM_RATE_MAX 6.0f
#define SIM_RATE_STEP_FACTOR 1.30f

/* §1.8 camera. */
#define CAM_DISTANCE_DEFAULT 28.0f
#define CAM_DISTANCE_MIN 8.0f /* keeps camera outside MEGATON cap */
#define CAM_DISTANCE_MAX 56.0f
#define CAM_DISTANCE_STEP 2.0f
#define CAM_HEIGHT 5.0f
#define CAM_LOOK_AT_HEIGHT 6.0f
#define CAM_FIELD_OF_VIEW_DEG 52.0f

/* §1.9 volume raymarcher — how the cloud gets drawn. */
#define RM_RAY_NEAR 0.5f
#define RM_RAY_FAR 32.0f
#define RM_STEP 0.18f
#define RM_MAX_STEPS 130
#define RM_OPAQUE_TRANSMITTANCE_EPS 0.01f
#define RM_DENSITY_GAIN 1.30f
#define RM_EMISSION_GAIN 4.5f
#define RM_AMBIENT_FLOOR 0.06f
#define RM_EMPTY_DENSITY_EPS 0.001f
#define RM_EMPTY_SKIP_FACTOR 2.0f /* take bigger steps through empty air */

/* §1.10 turning a pixel's brightness into a glyph + colour. */
#define PIXEL_LUMINANCE_CLAMP 1.10f
#define PIXEL_VISIBLE_LUM_EPS 0.002f
#define GLYPH_SLOT_COUNT 8
#define GLYPH_SLOT_FLOAT 7.999f /* just under 8, so the top slot is 7 */

/* §1.11 blast presets — the five things '1'..'5' / n drop in. */
typedef enum {
  BLAST_TACTICAL = 0,
  BLAST_STANDARD = 1,
  BLAST_MEGATON = 2,
  BLAST_AIR_BURST = 3,
  BLAST_GROUND = 4,
  BLAST_TYPE_COUNT = 5,
} BlastType;

/*
 * BlastParameters — the recipe for one blast (one of five presets).
 *
 * Each preset is just a few numbers describing the initial fireball:
 * how hot, how dense, how big, how high off the ground, and how hard
 * it pushes outward at the instant of detonation.  Pressing a number
 * stamps this blob into the fluid grid; physics takes it from there.
 * The values were hand-tuned to feel like real yield classes
 * (tactical / standard / megaton / air-burst / ground).
 */
typedef struct {
    const char *display_name;       /* short HUD label                  */
    float       sigma;              /* size of the fireball (world units) */
    float       peak_temperature;   /* hottest temperature at the core  */
    float       peak_density;       /* thickest smoke at the core       */
    float       detonation_altitude;/* height of the blast centre       */
    float       initial_outflow;    /* outward shove speed at t=0       */
} BlastParameters;

static const BlastParameters BLAST_PRESETS[BLAST_TYPE_COUNT] = {
    /* TACTICAL  — small, low-altitude, brief mushroom */
    {"TACTICAL  ", 0.35f, 5.0f, 2.5f, 1.0f, 2.0f},
    /* STANDARD  — canonical mid-yield mushroom */
    {"STANDARD  ", 0.55f, 8.0f, 4.0f, 1.6f, 3.0f},
    /* MEGATON   — huge yield, very tall column, long-lasting cap */
    {"MEGATON   ", 0.85f, 12.0f, 5.5f, 2.2f, 4.5f},
    /* AIR_BURST — high-altitude detonation, no ground stem */
    {"AIR_BURST ", 0.55f, 8.0f, 4.0f, 4.5f, 3.0f},
    /* GROUND    — surface burst, modest heat, heavy dust load */
    {"GROUND    ", 0.45f, 7.0f, 7.0f, 0.6f, 3.5f},
};

/* §1.12 themes — six colour looks for the same cloud. */
/*
 * Theme — one colour palette, picked with t/T.
 *
 * The cloud's brightness is sorted into eight tiers, from faint smoke
 * (slot 0) up to the white-hot fire core (slot 7).  A theme just says
 * which eight colours those tiers use, so the same physics can look
 * realistic, matrix-green, ocean-blue, and so on.  Slot 0 stays a bit
 * bright (index >= 24) so the faintest smoke is still visible against
 * a black terminal.  A couple of themes flip to a bright background
 * instead of black — inverted_background tells the renderer to clear
 * the screen to that colour first.
 */
typedef struct {
    const char *display_name;            /* short HUD label              */
    short       ramp_256[GLYPH_SLOT_COUNT];  /* 8 colours, smoke -> fire */
    bool        inverted_background;     /* true = use a bright backdrop */
} Theme;

#define THEME_COUNT 6

static const Theme THEMES[THEME_COUNT] = {
    /* REALISTIC: light grey smoke climbing into orange-yellow fire. */
    {"REALISTIC", {248, 250, 252, 254, 130, 166, 208, 220}, false},

    /* MATRIX: lime-green smoke + lime fire. */
    {"MATRIX   ", {64, 70, 112, 113, 119, 154, 190, 226}, false},

    /* OCEAN: deep teal smoke + cyan fire. */
    {"OCEAN    ", {37, 44, 45, 74, 81, 117, 159, 195}, false},

    /* NOVA: violet-lavender smoke + ice-blue fire. */
    {"NOVA     ", {97, 98, 99, 105, 111, 147, 153, 195}, false},

    /* TOXIC: chartreuse smoke + acid green-yellow fire. */
    {"TOXIC    ", {107, 113, 149, 155, 191, 192, 226, 228}, false},

    /* NEGATIVE: white background, dark foreground (photographic
     * negative).  See decorate_volume_pixel() for attr handling. */
    {"NEGATIVE ", {253, 250, 245, 240, 237, 234, 232, 16}, true},
};

/* §1.13 the eight glyphs, faintest to densest.  Slot 0 is '.', not a
 * space, so even a thin wisp of smoke shows up; cells dimmer than
 * PIXEL_VISIBLE_LUM_EPS are left blank. */
static const char LUMINANCE_GLYPHS[GLYPH_SLOT_COUNT] = {'.', ',', ':', ';',
                                                        '+', '*', '#', '@'};

/* §1.14 debug overlays — d / D cycles between them. */
typedef enum {
  DEBUG_NORMAL = 0,      /* full 3-D volumetric raymarch */
  DEBUG_DENSITY = 1,     /* raw 2-D density map          */
  DEBUG_TEMPERATURE = 2, /* raw 2-D temperature map      */
  DEBUG_VELOCITY = 3,    /* raw 2-D velocity arrows      */
  DEBUG_MODE_COUNT = 4,
} DebugMode;

static const char *DEBUG_MODE_NAMES[DEBUG_MODE_COUNT] = {
    "NORMAL    ",
    "DENSITY 2D",
    "TEMP 2D   ",
    "VELOCITY  ",
};

/* ── §2 clock — a steady timer and a sleep ── */

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

/* ── §3 vec3 — 3-D vector math, used only by the renderer ── */

typedef struct {
  float x, y, z;
} Vec3;

static inline Vec3 v3(float x, float y, float z) { return (Vec3){x, y, z}; }
static inline Vec3 v3add(Vec3 a, Vec3 b) {
  return v3(a.x + b.x, a.y + b.y, a.z + b.z);
}
static inline Vec3 v3sub(Vec3 a, Vec3 b) {
  return v3(a.x - b.x, a.y - b.y, a.z - b.z);
}
static inline Vec3 v3scale(float s, Vec3 a) {
  return v3(s * a.x, s * a.y, s * a.z);
}
static inline float v3dot(Vec3 a, Vec3 b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}
static inline Vec3 v3cross(Vec3 a, Vec3 b) {
  return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z,
            a.x * b.y - a.y * b.x);
}
static inline float v3length(Vec3 a) { return sqrtf(v3dot(a, a)); }
static inline Vec3 v3normalise(Vec3 a) {
  float length = v3length(a);
  return (length > 1e-12f) ? v3scale(1.0f / length, a) : v3(0, 0, 1);
}

/* ── §4 grid helpers — small utilities shared by sim and renderer ── */

static inline float clampf(float value, float lower, float upper) {
  if (value < lower)
    return lower;
  if (value > upper)
    return upper;
  return value;
}

/* Fold an index that fell off the edge back inside the grid.  This is
 * how we make solid walls: a neighbour just past the boundary reads
 * back as a cell just inside, so nothing flows through. */
static inline int mirror_index(int index, int grid_size) {
  if (index < 0)
    return 1;
  if (index >= grid_size)
    return grid_size - 2;
  return index;
}

/* Turn a 0..1 value into one of the eight slot numbers. */
static inline int to_slot(float value_01) {
  int slot = (int)(value_01 * GLYPH_SLOT_FLOAT);
  if (slot < 0)
    slot = 0;
  if (slot >= GLYPH_SLOT_COUNT)
    slot = GLYPH_SLOT_COUNT - 1;
  return slot;
}

/* Pin the sample point inside the grid so reading the cell AND its
 * up-and-right neighbour can't run off the end of the array. */
static inline void clamp_sample_to_grid_bounds(float *fi, float *fj) {
    *fi = clampf(*fi, 0.0f, (float)(GRID_RADIAL_CELLS - 1));
    *fj = clampf(*fj, 0.0f, (float)(GRID_VERTICAL_CELLS - 1));
}

/* Split a fractional position into a whole cell (i, j) — the
 * bottom-left of a 2x2 patch — and how far into that patch we are
 * (frac_i, frac_j), used as the blend weights below. */
static inline void integer_cell_and_subcell(float fi, float fj,
                                             int *out_i, int *out_j,
                                             float *out_frac_i, float *out_frac_j) {
    int i = (int)floorf(fi);
    int j = (int)floorf(fj);
    if (i >= GRID_RADIAL_CELLS   - 1) i = GRID_RADIAL_CELLS   - 2;
    if (j >= GRID_VERTICAL_CELLS - 1) j = GRID_VERTICAL_CELLS - 2;
    *out_i = i;
    *out_j = j;
    *out_frac_i = fi - (float)i;
    *out_frac_j = fj - (float)j;
}

/* Smoothly blend the four corner values by their weights.  The result
 * always lands between the smallest and largest corner, so this can
 * never blow up — that's what keeps the whole simulation stable. */
static inline float bilinear_blend_corners(float bottom_left, float bottom_right,
                                            float top_left,    float top_right,
                                            float frac_i,      float frac_j) {
    float bottom = bottom_left * (1.0f - frac_i) + bottom_right * frac_i;
    float top    = top_left    * (1.0f - frac_i) + top_right    * frac_i;
    return bottom * (1.0f - frac_j) + top * frac_j;
}

/* Read a smooth value from the grid at a fractional spot (fi, fj) by
 * blending the four cells around it. */
static float
sample_field_bilinear(const float field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
                      float fi, float fj) {
    clamp_sample_to_grid_bounds(&fi, &fj);

    int   i, j;
    float frac_i, frac_j;
    integer_cell_and_subcell(fi, fj, &i, &j, &frac_i, &frac_j);

    return bilinear_blend_corners(field[i    ][j    ], field[i + 1][j    ],
                                   field[i    ][j + 1], field[i + 1][j + 1],
                                   frac_i, frac_j);
}

/* §5 themes live in §1.12, alongside the rest of the config. */

/* ── §6 colours — set up the colour pairs, swap palettes ── */

static void apply_theme(int theme_index) {
  if (theme_index < 0 || theme_index >= THEME_COUNT)
    theme_index = 0;
  const Theme *theme = &THEMES[theme_index];

  short background_256 = theme->inverted_background ? 231 : -1;
  short background_8 = theme->inverted_background ? COLOR_WHITE : -1;

  if (COLORS >= 256) {
    for (int slot = 0; slot < GLYPH_SLOT_COUNT; slot++)
      init_pair((short)(PAIR_RAMP_BASE + slot), theme->ramp_256[slot],
                background_256);
  } else {
    /* 8-colour fallback — coarse approximation. */
    static const short FALLBACK_RAMP_8[GLYPH_SLOT_COUNT] = {
        COLOR_BLACK, COLOR_BLACK, COLOR_WHITE,  COLOR_WHITE,
        COLOR_RED,   COLOR_RED,   COLOR_YELLOW, COLOR_WHITE,
    };
    for (int slot = 0; slot < GLYPH_SLOT_COUNT; slot++)
      init_pair((short)(PAIR_RAMP_BASE + slot),
                theme->inverted_background ? COLOR_BLACK
                                           : FALLBACK_RAMP_8[slot],
                background_8);
  }
}

static void colors_init(void) {
  start_color();
  use_default_colors();
  if (COLORS >= 256) {
    init_pair(PAIR_HUD_STATUS, 226, -1); /* bright yellow */
    init_pair(PAIR_HUD_HINT, 51, -1);    /* bright cyan   */
  } else {
    init_pair(PAIR_HUD_STATUS, COLOR_YELLOW, -1);
    init_pair(PAIR_HUD_HINT, COLOR_CYAN, -1);
  }
  apply_theme(0);
}

/* ── §7 fluid state — everything the simulation tracks ── */
/*
 * Fluid — the whole simulation, as eight grids stored together.
 *
 * Because a mushroom cloud is round, we only simulate one flat slice
 * of it — a strip running from the centre axis outward (i) and from
 * the ground upward (j).  The renderer spins this slice back into a
 * full 3-D cloud.  So index i = 0 means "on the centre axis" and
 * j = 0 means "on the ground".  All eight grids together are ~168 KB
 * and live in static memory — we never call malloc.
 *
 * The grids:
 *   velocity is kept as two separate grids (sideways and up/down)
 *   because each one hits a different wall — no sideways flow through
 *   the centre axis, no up/down flow through the ground — and reading
 *   them apart keeps the inner loops simple.  Temperature and density
 *   are the stuff being carried along.  The pressure/divergence pair
 *   and the scratch_a/scratch_b pair are workspace: the solvers need
 *   somewhere to write next-step values while still reading the
 *   current ones.
 */
typedef struct {
    /* velocity, split into sideways and up/down */
    float velocity_radial   [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float velocity_vertical [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* what the flow carries: heat and smoke */
    float temperature       [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float density           [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* workspace for the projection step */
    float pressure          [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float divergence        [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];

    /* spare scratch grids the solvers write into */
    float scratch_a         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
    float scratch_b         [GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS];
} Fluid;

static Fluid g_fluid;

/* ── §8 walls — keep flow from passing through the axis and ground ── */
/*
 * The slice has two solid walls: the centre axis (nothing flows
 * sideways through it) and the ground (nothing flows up or down
 * through it).  Call this after anything that changes velocity —
 * skipping it is the classic way these solvers leak energy at the
 * edges and slowly fall apart.
 */
static void enforce_velocity_boundaries(Fluid *fluid) {
  for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
    fluid->velocity_radial[0][j] = 0.0f;

  for (int i = 0; i < GRID_RADIAL_CELLS; i++)
    fluid->velocity_vertical[i][0] = 0.0f;
}

/* ── §9 buoyancy — hot air rises ── */
/*
 * The one force that lifts the cloud: cells hotter than the
 * surrounding air get a nudge upward each tick.  Cooler cells get a
 * (gentle) nudge down.  Turn this off and the cloud just drifts
 * limply — nothing climbs.  Ref: Fedkiw, Stam & Jensen 2001.
 */
/* How much hotter than ambient this cell is — positive means it wants
 * to rise. */
static inline float temperature_excess_at(const Fluid *fluid, int i, int j) {
    return fluid->temperature[i][j] - TEMPERATURE_AMBIENT;
}

/* Give this cell its upward push for one tick, scaled by how hot it is. */
static inline void add_boussinesq_vertical_impulse(Fluid *fluid, int i, int j,
                                                    float step_seconds) {
    fluid->velocity_vertical[i][j] +=
        BUOYANCY_COEFFICIENT * temperature_excess_at(fluid, i, j) * step_seconds;
}

static void apply_buoyancy(Fluid *fluid, float step_seconds) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            add_boussinesq_vertical_impulse(fluid, i, j, step_seconds);
    enforce_velocity_boundaries(fluid);
}

/* ── §10 advect — let the flow carry a field along ── */
/*
 * Moving stuff with the flow.  The trick (Stam's "stable fluids"):
 * instead of pushing each cell's value forward — which can overshoot
 * and blow up — we look backward.  For every cell we ask "where was
 * this bit of fluid one step ago?", then copy the old value from
 * there.  Because we read by blending neighbours, the answer is
 * always tame, so the timestep can be as big as we like.  The same
 * routine carries velocity, heat, and smoke.
 */
/* Step backward from cell (i, j) along the flow to find where this
 * fluid came from one tick ago.  Velocities are in world units per
 * second; multiplying by 1/cell-size turns the offset into grid cells. */
static inline void trace_velocity_backward_to_departure(
        int i, int j,
        const float vr[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float vy[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float dt_in_grid_units,
        float *out_source_i, float *out_source_j) {
    *out_source_i = (float)i - vr[i][j] * dt_in_grid_units;
    *out_source_j = (float)j - vy[i][j] * dt_in_grid_units;
}

/* Carry one whole grid along the flow: for each cell, trace back to
 * where its fluid came from and copy the old value to here. */
static void advect_field(
        float destination_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float source_field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_radial[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        const float velocity_vertical[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float step_seconds) {
    float dt_in_grid_units = step_seconds * GRID_INV_CELL_SIZE;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float source_i, source_j;
            trace_velocity_backward_to_departure(
                i, j, velocity_radial, velocity_vertical, dt_in_grid_units,
                &source_i, &source_j);
            destination_field[i][j] =
                sample_field_bilinear(source_field, source_i, source_j);
        }
    }
}

/* ── §11 projection — keep the flow from squashing or stretching ── */
/*
 * After buoyancy gives some cells a shove, the flow stops being
 * "balanced": fluid would pile up in some spots and vanish from
 * others.  This step fixes that so the fluid neither compresses nor
 * tears.  It works in three passes: measure how out-of-balance each
 * cell is, solve for a pressure field that would cancel it, then
 * subtract that pressure's push from the velocity.  Afterward the
 * flow is balanced everywhere.
 */

/* Grab the four neighbour indices for a cell, with off-grid ones
 * folded back in (the solid-wall trick from §4).  Done once per cell
 * so the passes below don't each repeat the fold. */
static inline void neighbour_indices_mirrored(int i, int j,
                                              int *out_i_left, int *out_i_right,
                                              int *out_j_below, int *out_j_above) {
    *out_i_left  = mirror_index(i - 1, GRID_RADIAL_CELLS);
    *out_i_right = mirror_index(i + 1, GRID_RADIAL_CELLS);
    *out_j_below = mirror_index(j - 1, GRID_VERTICAL_CELLS);
    *out_j_above = mirror_index(j + 1, GRID_VERTICAL_CELLS);
}

/* How out-of-balance one cell is: are more fluid arrows pointing out
 * of it than into it?  Positive means fluid is leaving (would thin
 * out), negative means piling up.  Projection drives this to zero. */
static inline float centred_divergence_at(const Fluid *fluid, int i, int j) {
    int i_left, i_right, j_below, j_above;
    neighbour_indices_mirrored(i, j, &i_left, &i_right, &j_below, &j_above);
    float dvr_dr = fluid->velocity_radial  [i_right][j]    -
                   fluid->velocity_radial  [i_left ][j];
    float dvy_dy = fluid->velocity_vertical[i      ][j_above] -
                   fluid->velocity_vertical[i      ][j_below];
    return (dvr_dr + dvy_dy) * 0.5f * GRID_INV_CELL_SIZE;
}

/* Measure the imbalance everywhere, and start the pressure guess at
 * zero ready for the solver below. */
static void compute_divergence(Fluid *fluid) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            fluid->divergence[i][j] = centred_divergence_at(fluid, i, j);
            fluid->pressure  [i][j] = 0.0f;
        }
    }
}

/* New pressure for one cell: roughly the average of its neighbours,
 * tugged by how out-of-balance the cell is.  Reads only old pressure
 * values (this sweep writes to scratch), so the whole grid updates
 * from a consistent snapshot. */
static inline float jacobi_pressure_update_at(const Fluid *fluid, int i, int j,
                                              float h_squared) {
    int i_left, i_right, j_below, j_above;
    neighbour_indices_mirrored(i, j, &i_left, &i_right, &j_below, &j_above);
    float neighbour_sum =
        fluid->pressure[i_right][j      ] + fluid->pressure[i_left ][j      ] +
        fluid->pressure[i      ][j_above] + fluid->pressure[i      ][j_below];
    return (neighbour_sum - h_squared * fluid->divergence[i][j]) * 0.25f;
}

/* One pass over the whole grid: compute every cell's new pressure
 * into scratch, then copy it back.  Repeating this many times lets
 * the pressure settle into a consistent answer. */
static inline void jacobi_pressure_sweep(Fluid *fluid, float h_squared) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            fluid->scratch_a[i][j] = jacobi_pressure_update_at(fluid, i, j,
                                                                h_squared);
        }
    }
    memcpy(fluid->pressure, fluid->scratch_a, sizeof fluid->pressure);
}

/* Run enough sweeps for the pressure field to settle down. */
static void solve_pressure_poisson(Fluid *fluid) {
    float h_squared = GRID_CELL_SIZE * GRID_CELL_SIZE;
    for (int iter = 0; iter < POISSON_JACOBI_ITERATIONS; iter++)
        jacobi_pressure_sweep(fluid, h_squared);
}

/* Which way pressure is rising at this cell, and how steeply — the
 * sideways and up/down parts of that slope.  That slope is the push
 * we subtract from the flow next. */
static inline void centred_pressure_gradient_at(const Fluid *fluid, int i, int j,
                                                float *out_dp_dr,
                                                float *out_dp_dy) {
    int i_left, i_right, j_below, j_above;
    neighbour_indices_mirrored(i, j, &i_left, &i_right, &j_below, &j_above);
    *out_dp_dr = (fluid->pressure[i_right][j      ] -
                  fluid->pressure[i_left ][j      ]) * 0.5f * GRID_INV_CELL_SIZE;
    *out_dp_dy = (fluid->pressure[i      ][j_above] -
                  fluid->pressure[i      ][j_below]) * 0.5f * GRID_INV_CELL_SIZE;
}

/* Push this cell's flow the opposite way from rising pressure — high
 * pressure shoves fluid toward low.  Once every cell is done, the
 * flow is balanced. */
static inline void subtract_pressure_gradient_at(Fluid *fluid, int i, int j) {
    float dp_dr, dp_dy;
    centred_pressure_gradient_at(fluid, i, j, &dp_dr, &dp_dy);
    fluid->velocity_radial  [i][j] -= dp_dr;
    fluid->velocity_vertical[i][j] -= dp_dy;
}

/* Apply that pressure-driven correction to every cell. */
static void subtract_pressure_gradient(Fluid *fluid) {
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            subtract_pressure_gradient_at(fluid, i, j);
}

static void project_to_incompressible(Fluid *fluid) {
  compute_divergence(fluid);
  solve_pressure_poisson(fluid);
  subtract_pressure_gradient(fluid);
  enforce_velocity_boundaries(fluid);
}

/* ── §12 cool & fade — let the cloud wind down ── */
/*
 * Two slow losses applied at the end of every tick: heat eases back
 * toward the surrounding temperature, and smoke gradually thins out.
 * Without these the cloud would rise and persist forever.
 */
static void apply_cool_and_decay(Fluid *fluid, float step_seconds) {
  float cool_factor = expf(-COOL_RATE * step_seconds);
  float density_factor = 1.0f - DENSITY_DECAY * step_seconds;
  if (density_factor < 0.0f)
    density_factor = 0.0f;

  for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
    for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
      float excess = fluid->temperature[i][j] - TEMPERATURE_AMBIENT;
      fluid->temperature[i][j] = TEMPERATURE_AMBIENT + excess * cool_factor;
      fluid->density[i][j] *= density_factor;
    }
  }
}

/* ── §13 one tick — the whole solver, top to bottom ── */

/* The flow carries itself along.  Both velocity grids are written to
 * scratch first and swapped back at the end, so the second one isn't
 * reading a half-updated copy of the first. */
static inline void advect_velocity_self(Fluid *fluid, float step_seconds) {
    advect_field(fluid->scratch_a, fluid->velocity_radial,
                 fluid->velocity_radial, fluid->velocity_vertical, step_seconds);
    advect_field(fluid->scratch_b, fluid->velocity_vertical,
                 fluid->velocity_radial, fluid->velocity_vertical, step_seconds);
    memcpy(fluid->velocity_radial,   fluid->scratch_a,
           sizeof fluid->velocity_radial);
    memcpy(fluid->velocity_vertical, fluid->scratch_b,
           sizeof fluid->velocity_vertical);
    enforce_velocity_boundaries(fluid);
}

/* Carry one carried-along grid (heat or smoke) with the current flow,
 * using the same write-to-scratch-then-swap pattern. */
static inline void advect_scalar_by_velocity(
        Fluid *fluid,
        float field[GRID_RADIAL_CELLS][GRID_VERTICAL_CELLS],
        float step_seconds) {
    advect_field(fluid->scratch_a, field,
                 fluid->velocity_radial, fluid->velocity_vertical, step_seconds);
    memcpy(field, fluid->scratch_a,
           sizeof(float) * GRID_RADIAL_CELLS * GRID_VERTICAL_CELLS);
}

/*
 * One full step of physics, in the order it runs: hot cells get their
 * upward push, rebalance the flow, let the flow carry itself, rebalance
 * again, carry the heat and smoke along, then cool and fade.  We
 * rebalance twice because carrying the flow nudges it slightly out of
 * balance again — Jos Stam's classic advect-project-advect cycle.
 */
static void fluid_step(Fluid *fluid, float step_seconds) {
    apply_buoyancy             (fluid, step_seconds);
    project_to_incompressible  (fluid);

    advect_velocity_self       (fluid, step_seconds);
    project_to_incompressible  (fluid);  /* carrying the flow unbalanced it a touch */

    advect_scalar_by_velocity  (fluid, fluid->temperature, step_seconds);
    advect_scalar_by_velocity  (fluid, fluid->density,     step_seconds);

    apply_cool_and_decay       (fluid, step_seconds);
}

/* ── §14 detonate — the one scripted moment ── */
/*
 * The only thing we stage by hand.  Stamp a soft round blob of heat
 * and smoke at the blast centre — brightest in the middle, fading
 * with distance — and give it an outward shove.  That shove stands in
 * for the shock wave of the first second or two; without it the cloud
 * just rises like a candle flame instead of expanding first.  After
 * this, physics runs everything.
 */
/* Wipe the grid to a clean start.  Zeroing handles velocity, smoke,
 * and scratch, but temperature is reset to the ambient baseline (not
 * absolute zero) so the cooling step has a sensible target. */
static inline void reset_field_to_ambient(Fluid *fluid) {
    memset(fluid, 0, sizeof *fluid);
    for (int i = 0; i < GRID_RADIAL_CELLS; i++)
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++)
            fluid->temperature[i][j] = TEMPERATURE_AMBIENT;
}

/* Where a cell sits in the world.  The +0.5 aims at the cell's middle,
 * not its corner, so the blob stays centred on the detonation point. */
static inline void cell_centre_world_coords(int i, int j,
                                             float *out_radius, float *out_altitude) {
    *out_radius   = ((float)i + 0.5f) * GRID_CELL_SIZE;
    *out_altitude = ((float)j + 0.5f) * GRID_CELL_SIZE;
}

/* How strong the blob is at this cell: full strength at the centre,
 * smoothly fading to nothing with distance (a bell curve).  Also hands
 * back the offset from the centre, reused for the outward shove. */
static inline float gaussian_at_cell_distance(int i, int j,
                                               float detonation_altitude,
                                               float two_sigma_squared,
                                               float *out_dr, float *out_dy,
                                               float *out_distance_squared) {
    float radius, altitude;
    cell_centre_world_coords(i, j, &radius, &altitude);
    *out_dr               = radius;
    *out_dy               = altitude - detonation_altitude;
    *out_distance_squared = (*out_dr) * (*out_dr) + (*out_dy) * (*out_dy);
    return expf(- *out_distance_squared / two_sigma_squared);
}

/* Set this cell's heat and smoke from the blob's strength here:
 * fully hot and thick at the centre, ambient and clear far out. */
static inline void seed_gaussian_scalars_at(Fluid *fluid, int i, int j,
                                             float gaussian,
                                             const BlastParameters *blast) {
    fluid->temperature[i][j] =
        TEMPERATURE_AMBIENT +
        (blast->peak_temperature - TEMPERATURE_AMBIENT) * gaussian;
    fluid->density[i][j] = blast->peak_density * gaussian;
}

/* Shove this cell outward from the centre — the stand-in shock wave.
 * Only cells well inside the blob (strength > 0.05) get a kick.  The
 * tiny-distance guard skips the exact centre, where "outward" has no
 * direction and we'd divide by zero. */
static inline void seed_radial_outflow_at(Fluid *fluid, int i, int j,
                                           float gaussian,
                                           float dr, float dy,
                                           float distance_squared,
                                           float initial_outflow) {
    if (gaussian <= 0.05f) return;
    float distance = sqrtf(distance_squared);
    if (distance <= 1e-3f) return;
    float blast_speed = initial_outflow * gaussian;
    fluid->velocity_radial  [i][j] = (dr / distance) * blast_speed;
    fluid->velocity_vertical[i][j] = (dy / distance) * blast_speed;
}

/* Stamp the whole blast into the grid: clear it, then for every cell
 * set its heat/smoke and its outward shove from the blob.  The round
 * blob shape isn't derived from real physics — it's just the simplest
 * starting blob that grows into a believable mushroom once buoyancy
 * takes over. */
static void detonate_at_origin(Fluid *fluid, const BlastParameters *blast) {
    reset_field_to_ambient(fluid);

    float two_sigma_squared = 2.0f * blast->sigma * blast->sigma;

    for (int i = 0; i < GRID_RADIAL_CELLS; i++) {
        for (int j = 0; j < GRID_VERTICAL_CELLS; j++) {
            float dr, dy, distance_squared;
            float gauss = gaussian_at_cell_distance(
                i, j, blast->detonation_altitude, two_sigma_squared,
                &dr, &dy, &distance_squared);

            seed_gaussian_scalars_at(fluid, i, j, gauss, blast);
            seed_radial_outflow_at  (fluid, i, j, gauss, dr, dy,
                                     distance_squared, blast->initial_outflow);
        }
    }
    enforce_velocity_boundaries(fluid);
}

/* ── §15 sampling — the bridge from renderer back to the slice ── */
/*
 * The renderer asks "what's the smoke/heat at this 3-D point?" and
 * these answer by looking up the flat slice.  Since the cloud is
 * round, all that matters is how far the point is from the centre
 * axis and how high it is.  Points outside the simulated region get a
 * harmless default (clear air / ambient temperature).
 */
static inline float sample_density_at_world(float radius, float altitude) {
  if (radius < 0.0f || radius >= GRID_RADIAL_EXTENT)
    return 0.0f;
  if (altitude < 0.0f || altitude >= GRID_VERTICAL_EXTENT)
    return 0.0f;
  return sample_field_bilinear(g_fluid.density, radius * GRID_INV_CELL_SIZE,
                               altitude * GRID_INV_CELL_SIZE);
}

static inline float sample_temperature_at_world(float radius, float altitude) {
  if (radius < 0.0f || radius >= GRID_RADIAL_EXTENT)
    return TEMPERATURE_AMBIENT;
  if (altitude < 0.0f || altitude >= GRID_VERTICAL_EXTENT)
    return TEMPERATURE_AMBIENT;
  return sample_field_bilinear(g_fluid.temperature, radius * GRID_INV_CELL_SIZE,
                               altitude * GRID_INV_CELL_SIZE);
}

/* ── §16 raymarch — walk a sightline through the cloud ── */
/*
 * To colour a pixel we shoot a line from the camera and step along
 * it, adding up the light.  Glowing-hot bits add brightness; thick
 * smoke blocks whatever is behind it (so the far side of a dense
 * cloud stays hidden).  Each ray reports two numbers: total
 * brightness, and how much of it came from glowing heat — the second
 * decides whether the pixel reads as smoke or fire.  Empty stretches
 * are stepped over quickly, which is most of the work when the cloud
 * is small.
 */
/* Distance of a 3-D point from the centre axis — the first half of
 * turning a world point back into a spot on the flat slice. */
static inline float world_radius_at(Vec3 sample_point) {
    return sqrtf(sample_point.x * sample_point.x +
                 sample_point.z * sample_point.z);
}

/* Is this point inside the region we actually simulate?  If not, the
 * ray can take a bigger stride — there's no cloud out there to miss. */
static inline bool sample_inside_simulation_domain(Vec3 sample_point,
                                                    float *out_radius) {
    if (sample_point.y < 0.0f || sample_point.y > GRID_VERTICAL_EXTENT)
        return false;
    *out_radius = world_radius_at(sample_point);
    return *out_radius <= GRID_RADIAL_EXTENT;
}

/* Turn a temperature into a glow amount from 0 (no glow) to 1 (full
 * fire).  Capped at 1, so a brief over-heat right after detonation
 * just glows fully rather than blowing past the top. */
static inline float temperature_to_emission_normalised(float temperature,
                                                        float inverse_temp_range) {
    float emission = (temperature - TEMPERATURE_AMBIENT) * inverse_temp_range;
    return clampf(emission, 0.0f, 1.0f);
}

/* Add what this one step contributes to the pixel.  Each bit of light
 * is dimmed by how much smoke is already in front of it (transmittance
 * — the fraction of light still getting through).  We tally total
 * brightness and, separately, just the glowing-heat part, which later
 * picks smoke vs fire colour. */
static inline void accumulate_emission_absorption_step(
        float transmittance, float optical_depth_step,
        float emission, float source,
        float *running_total_luminance, float *running_hot_luminance) {
    *running_total_luminance += transmittance * optical_depth_step * source;
    *running_hot_luminance   += transmittance * optical_depth_step *
                                 emission * RM_EMISSION_GAIN;
}

/* Dim the remaining light after passing through this much smoke.  The
 * thicker the smoke in the step, the less gets through — the same law
 * that makes deep fog go dark. */
static inline float beer_lambert_attenuate(float transmittance,
                                            float optical_depth_step) {
    return transmittance * expf(-optical_depth_step);
}

/* March one sightline from the camera through the cloud, adding up
 * light as it goes.  Empty air is skipped quickly; once the smoke in
 * front is thick enough to hide everything behind, we stop early.
 * Reports total brightness and the glowing-heat part of it. */
static void raymarch_volume(Vec3 origin, Vec3 direction,
                            float *out_total_luminance,
                            float *out_hot_luminance) {
    float t                   = RM_RAY_NEAR;
    float transmittance       = 1.0f;
    float total_luminance     = 0.0f;
    float hot_luminance       = 0.0f;
    float inverse_temp_range  = 1.0f / (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);

    for (int step = 0; step < RM_MAX_STEPS; step++) {
        Vec3  sample_point = v3add(origin, v3scale(t, direction));
        float radius;

        /* OUTSIDE the simulation cylinder — skip ahead in big steps. */
        if (!sample_inside_simulation_domain(sample_point, &radius)) {
            t += RM_STEP * RM_EMPTY_SKIP_FACTOR;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* INSIDE the cylinder but the cloud is sparse here — small step. */
        float density = sample_density_at_world(radius, sample_point.y);
        if (density < RM_EMPTY_DENSITY_EPS) {
            t += RM_STEP;
            if (t > RM_RAY_FAR) break;
            continue;
        }

        /* INSIDE the cloud — read temperature and integrate one step. */
        float temperature        = sample_temperature_at_world(radius,
                                                                sample_point.y);
        float emission           = temperature_to_emission_normalised(
                                       temperature, inverse_temp_range);
        float source             = emission * RM_EMISSION_GAIN + RM_AMBIENT_FLOOR;
        float optical_depth_step = density * RM_STEP * RM_DENSITY_GAIN;

        accumulate_emission_absorption_step(transmittance, optical_depth_step,
                                             emission, source,
                                             &total_luminance, &hot_luminance);
        transmittance = beer_lambert_attenuate(transmittance, optical_depth_step);

        if (transmittance < RM_OPAQUE_TRANSMITTANCE_EPS) break;
        if (t > RM_RAY_FAR)                              break;
        t += RM_STEP;
    }

    *out_total_luminance = total_luminance;
    *out_hot_luminance   = hot_luminance;
}

/* ── §17 camera — where we look from, and the ray for each pixel ── */

/*
 * Camera — the viewpoint, plus three axes that orient the picture.
 *
 * To draw a pixel we need a ray pointing into the scene.  The camera
 * holds where it sits (origin) and which way is forward, right, and
 * up; those three axes are worked out once and reused for every pixel
 * so the per-pixel loop stays cheap.  aspect_factor corrects for
 * terminal cells being about twice as tall as they are wide, so the
 * cloud looks proportioned instead of stretched.
 */
typedef struct {
    Vec3  origin;          /* where the camera sits                   */
    Vec3  forward;         /* the way it's looking                    */
    Vec3  right;           /* screen-right direction                  */
    Vec3  up;              /* screen-up direction                     */
    float fov_tangent;     /* how wide the view is                    */
    float aspect_factor;   /* fix for tall terminal cells             */
} Camera;

static Camera build_camera_basis(float distance_behind, int visible_rows,
                                 int visible_cols) {
  Camera cam;
  cam.origin = v3(0.0f, CAM_HEIGHT, -distance_behind);
  Vec3 look_at = v3(0.0f, CAM_LOOK_AT_HEIGHT, 0.0f);

  cam.forward = v3normalise(v3sub(look_at, cam.origin));
  Vec3 world_up = v3(0.0f, 1.0f, 0.0f);
  cam.right = v3normalise(v3cross(cam.forward, world_up));
  cam.up = v3cross(cam.right, cam.forward);

  cam.fov_tangent = tanf(CAM_FIELD_OF_VIEW_DEG * (float)M_PI / 180.0f * 0.5f);
  cam.aspect_factor =
      ((float)visible_rows * TERMINAL_CELL_ASPECT) / (float)visible_cols;
  return cam;
}

static Vec3 ray_for_pixel(int col, int row, int cols, int visible_rows,
                          const Camera *cam) {
  float u = ((float)col + 0.5f) / (float)cols * 2.0f - 1.0f;
  float v = -(((float)row + 0.5f) / (float)visible_rows * 2.0f - 1.0f);

  Vec3 ray =
      v3add(cam->forward,
            v3add(v3scale(u * cam->fov_tangent, cam->right),
                  v3scale(v * cam->fov_tangent * cam->aspect_factor, cam->up)));
  return v3normalise(ray);
}

/* ── §18 decorate — turn a pixel's light into a glyph + colour ── */

/*
 * Cell — what to draw at one screen position.
 *
 * The raymarcher gives two numbers per pixel (total brightness, and
 * how much was glowing heat).  This bundles the final drawing
 * decision: which character, which colour, and any extra emphasis —
 * plus a skip flag for fully see-through pixels, which we just leave
 * blank so the terminal doesn't bother drawing them.  Returning one
 * struct lets the decorator hand all of that back at once.
 */
typedef struct {
    char   glyph;     /* the character to draw                      */
    int    pair;      /* which themed colour                        */
    attr_t attr;      /* extra emphasis (bold for the hottest)      */
    bool   skip;      /* true = leave blank (see-through)           */
} Cell;

/* Pick which of the eight glyphs to use from how bright the pixel is.
 * Very bright pixels just cap at the densest glyph. */
static inline int luminance_to_glyph_slot(float total_luminance) {
    float lum_normalised = total_luminance / PIXEL_LUMINANCE_CLAMP;
    if (lum_normalised > 1.0f) lum_normalised = 1.0f;
    return to_slot(lum_normalised);
}

/* Pick the colour from how much of the brightness was glowing heat:
 * none of it = smoke colour, all of it = fire colour.  The small +
 * keeps us from dividing by zero on a barely-visible pixel. */
static inline int hot_fraction_to_palette_slot(float hot_luminance,
                                                float total_luminance) {
    float hot_fraction = hot_luminance / (total_luminance + 0.001f);
    if (hot_fraction > 1.0f) hot_fraction = 1.0f;
    if (hot_fraction < 0.0f) hot_fraction = 0.0f;
    return to_slot(hot_fraction);
}

/* Add emphasis: bold on the brightest/hottest pixels for punch, dim
 * on the faintest wisps.  Skip both on bright-background themes, where
 * bold and dim would read backwards against the light backdrop. */
static inline attr_t pick_themed_cell_attribute(int slot_lum, int slot_hot,
                                                 bool inverted_theme) {
    if (inverted_theme) return A_NORMAL;
    if (slot_hot >= 6 || slot_lum >= 6) return A_BOLD;
    if (slot_lum <= 1)                  return A_DIM;
    return A_NORMAL;
}

/* Turn one pixel's two numbers into a full drawing decision.  The
 * brightness picks the character (so thicker cloud reads denser) and
 * the heat picks the colour (so smoke and fire look different even
 * when they're equally bright).  Fully see-through pixels are skipped. */
static Cell decorate_volume_pixel(float total_luminance, float hot_luminance,
                                  bool inverted_theme) {
    if (total_luminance < PIXEL_VISIBLE_LUM_EPS)
        return (Cell){ .skip = true };

    int    slot_lum = luminance_to_glyph_slot(total_luminance);
    int    slot_hot = hot_fraction_to_palette_slot(hot_luminance, total_luminance);
    attr_t attr     = pick_themed_cell_attribute(slot_lum, slot_hot, inverted_theme);

    return (Cell){
        .glyph = LUMINANCE_GLYPHS[slot_lum],
        .pair  = PAIR_RAMP_BASE + slot_hot,
        .attr  = attr,
        .skip  = false,
    };
}

/* Draw one cell, only switching colour/emphasis when it actually
 * changes from the last cell — cheaper across runs of the same look. */
static void emit_cell(int row, int col, Cell cell, int *last_pair,
                      attr_t *last_attr) {
  if (cell.skip)
    return;
  if (cell.pair != *last_pair || cell.attr != *last_attr) {
    if (*last_pair >= 0)
      attroff(COLOR_PAIR(*last_pair) | *last_attr);
    attron(COLOR_PAIR(cell.pair) | cell.attr);
    *last_pair = cell.pair;
    *last_attr = cell.attr;
  }
  mvaddch(row, col, (chtype)(unsigned char)cell.glyph);
}

/* ── §19 render — raymarch every screen cell and paint it ── */

/*
 * Scene — all the live state for one running demo.
 *
 * This holds the user's choices (which blast, theme, and debug view),
 * the simulation clock, and how far back the camera sits.  The fluid
 * grids themselves live in g_fluid at file scope, not here.  The
 * fields are grouped so it's clear which belong to the physics and
 * which only affect how things look — changing the theme, for
 * instance, must never touch the simulation, so the same blast always
 * plays out identically whatever colours you pick.
 */
typedef struct {
    bool      paused;                /* freezes the sim, shows in HUD */

    /* look only — changing these never touches the physics */
    int       theme_index;
    DebugMode debug_mode;

    BlastType blast_type;            /* which preset the next blast uses */

    int       cols, rows;            /* terminal size in cells   */

    /* simulation clock */
    float     simulation_time_seconds;
    float     simulation_rate;          /* how fast sim time runs */
    float     simulation_step_accumulator;

    float     camera_distance;       /* zoom in / out            */
} Scene;

/* On bright-background themes, paint the whole area with the backdrop
 * first, so the see-through cells we later skip show the right colour. */
static inline void prefill_inverted_background(int visible_rows, int cols,
                                                int y_offset) {
    attron(COLOR_PAIR(PAIR_RAMP_BASE));
    for (int row = 0; row < visible_rows; row++)
        for (int col = 0; col < cols; col++)
            mvaddch(row + y_offset, col, ' ');
    attroff(COLOR_PAIR(PAIR_RAMP_BASE));
}

/* Shoot the ray for one pixel and decide what to draw there. */
static inline Cell raymarch_and_decorate_pixel(int col, int row,
                                                int cols, int visible_rows,
                                                const Camera *cam,
                                                bool inverted_theme) {
    Vec3  ray = ray_for_pixel(col, row, cols, visible_rows, cam);
    float total_lum, hot_lum;
    raymarch_volume(cam->origin, ray, &total_lum, &hot_lum);
    return decorate_volume_pixel(total_lum, hot_lum, inverted_theme);
}

/* Walk every visible cell, raymarch it, and draw it. */
static inline void paint_raymarched_field(const Camera *cam,
                                           int visible_rows, int cols,
                                           int y_offset, bool inverted,
                                           int *last_pair, attr_t *last_attr) {
    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < cols; col++) {
            Cell cell = raymarch_and_decorate_pixel(col, row, cols,
                                                    visible_rows, cam, inverted);
            emit_cell(y_offset + row, col, cell, last_pair, last_attr);
        }
    }
}

/* Draw the full cloud: set up the camera, prep the backdrop if needed,
 * then raymarch and paint every cell. */
static void render_volume_view(const Scene *scene) {
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return;

    Camera cam = build_camera_basis(scene->camera_distance,
                                     visible_rows, scene->cols);
    bool   inverted = THEMES[scene->theme_index].inverted_background;
    int    y_offset = 1;          /* leave row 0 for status */

    if (inverted)
        prefill_inverted_background(visible_rows, scene->cols, y_offset);

    int    last_pair = inverted ? PAIR_RAMP_BASE : -1;
    attr_t last_attr = 0;
    paint_raymarched_field(&cam, visible_rows, scene->cols, y_offset, inverted,
                            &last_pair, &last_attr);

    if (last_pair >= 0)
        attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* ── §20 debug views — show the raw fluid grids straight up ── */
/*
 * Instead of the 3-D cloud, these draw one fluid grid flat on screen:
 * the smoke, the heat, or the flow shown as little arrows.  Handy for
 * watching what the simulation is actually doing.
 */

/* Map a screen cell to a grid cell, flipping top-to-bottom so the
 * ground ends up at the bottom of the screen, not the top. */
static inline void screen_cell_to_fluid_grid(int row, int col,
                                              int visible_rows, int cols,
                                              int *out_grid_i, int *out_grid_j) {
    *out_grid_j = (visible_rows - 1 - row) * GRID_VERTICAL_CELLS / visible_rows;
    *out_grid_i =  col                     * GRID_RADIAL_CELLS  / cols;
}

/* Shared setup for the overlays: how many rows are drawable (0 if the
 * window is too short), and the offset that keeps row 0 for the HUD. */
static inline int begin_debug_overlay(const Scene *scene, int *out_y_offset) {
    int visible_rows = scene->rows - HUD_RESERVED_ROWS;
    if (visible_rows < 1) return 0;
    *out_y_offset = 1;        /* leave row 0 for status */
    return visible_rows;
}

/* Turn off whatever colour/emphasis the last drawn cell left on. */
static inline void end_debug_overlay(int last_pair, attr_t last_attr) {
    if (last_pair >= 0) attroff(COLOR_PAIR(last_pair) | last_attr);
}

/* Show the smoke grid flat: each cell's thickness as a glyph + colour. */
static void render_debug_density(const Scene *scene) {
    int y_offset;
    int visible_rows = begin_debug_overlay(scene, &y_offset);
    if (!visible_rows) return;

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            int grid_i, grid_j;
            screen_cell_to_fluid_grid(row, col, visible_rows, scene->cols,
                                       &grid_i, &grid_j);
            float density   = g_fluid.density[grid_i][grid_j];
            float normalise = clampf(density / DENSITY_PEAK, 0.0f, 1.0f);
            int   slot      = to_slot(normalise);

            if (slot == 0 && density < 0.01f) continue;

            Cell c = { .glyph = LUMINANCE_GLYPHS[slot],
                       .pair  = PAIR_RAMP_BASE + slot,
                       .attr  = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    end_debug_overlay(last_pair, last_attr);
}

/* Show the heat grid flat: each cell's temperature as a glyph + colour. */
static void render_debug_temperature(const Scene *scene) {
    int y_offset;
    int visible_rows = begin_debug_overlay(scene, &y_offset);
    if (!visible_rows) return;

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            int grid_i, grid_j;
            screen_cell_to_fluid_grid(row, col, visible_rows, scene->cols,
                                       &grid_i, &grid_j);
            float temperature = g_fluid.temperature[grid_i][grid_j];
            float normalise   = (temperature - TEMPERATURE_AMBIENT) /
                                (TEMPERATURE_PEAK - TEMPERATURE_AMBIENT);
            normalise         = clampf(normalise, 0.0f, 1.0f);
            int   slot        = to_slot(normalise);

            if (slot == 0 && normalise < 0.01f) continue;

            Cell c = { .glyph = LUMINANCE_GLYPHS[slot],
                       .pair  = PAIR_RAMP_BASE + slot,
                       .attr  = A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    end_debug_overlay(last_pair, last_attr);
}

/* Pick an arrow character pointing the way the flow goes (one of eight
 * directions).  Nearly-still cells get a blank instead of an arrow. */
static char arrow_for_velocity(float vx, float vy) {
    float magnitude = sqrtf(vx * vx + vy * vy);
    if (magnitude < 0.05f) return ' ';
    float angle  = atan2f(vy, vx);
    int   octant = (int)((angle + (float)M_PI) / ((float)M_PI / 4.0f) + 0.5f) % 8;
    static const char ARROWS[8] = { '<', '/', 'v', '\\', '>', '/', '^', '\\' };
    return ARROWS[octant];
}

/* Colour an arrow by how fast the flow is, capping at a reference
 * speed of 4 so the rare fast cell just shows brightest. */
static inline int velocity_magnitude_to_slot(float vx, float vy) {
    float magnitude = sqrtf(vx * vx + vy * vy);
    float normalise = clampf(magnitude / 4.0f, 0.0f, 1.0f);
    return to_slot(normalise);
}

/* Show the flow grid as a field of arrows, coloured by speed. */
static void render_debug_velocity(const Scene *scene) {
    int y_offset;
    int visible_rows = begin_debug_overlay(scene, &y_offset);
    if (!visible_rows) return;

    int    last_pair = -1;
    attr_t last_attr = 0;

    for (int row = 0; row < visible_rows; row++) {
        for (int col = 0; col < scene->cols; col++) {
            int grid_i, grid_j;
            screen_cell_to_fluid_grid(row, col, visible_rows, scene->cols,
                                       &grid_i, &grid_j);

            float vr    = g_fluid.velocity_radial  [grid_i][grid_j];
            float vy    = g_fluid.velocity_vertical[grid_i][grid_j];
            char  glyph = arrow_for_velocity(vr, vy);
            if (glyph == ' ') continue;

            int slot = velocity_magnitude_to_slot(vr, vy);
            Cell c   = { .glyph = glyph,
                         .pair  = PAIR_RAMP_BASE + slot,
                         .attr  = (slot >= 5) ? A_BOLD : A_NORMAL };
            emit_cell(y_offset + row, col, c, &last_pair, &last_attr);
        }
    }
    end_debug_overlay(last_pair, last_attr);
}

/* ── §21 dispatch — draw the cloud, or one of the debug views ── */

static void render_active_view(const Scene *scene) {
  switch (scene->debug_mode) {
  case DEBUG_NORMAL:
    render_volume_view(scene);
    break;
  case DEBUG_DENSITY:
    render_debug_density(scene);
    break;
  case DEBUG_TEMPERATURE:
    render_debug_temperature(scene);
    break;
  case DEBUG_VELOCITY:
    render_debug_velocity(scene);
    break;
  default:
    render_volume_view(scene);
    break;
  }
}

/* ── §22 HUD — status line on top, key hints along the bottom ── */

static void hud_draw(int term_rows, int term_cols, const Scene *scene,
                     double real_fps, int target_render_fps) {
  char status_text[200];
  snprintf(status_text, sizeof status_text,
           " %5.1f fps  %3d Hz  blast:%s  theme:%s  debug:%s  "
           "t:%6.2fs  rate:%4.2f  dist:%4.1f  %s ",
           real_fps, target_render_fps,
           BLAST_PRESETS[scene->blast_type].display_name,
           THEMES[scene->theme_index].display_name,
           DEBUG_MODE_NAMES[scene->debug_mode],
           (double)scene->simulation_time_seconds,
           (double)scene->simulation_rate, (double)scene->camera_distance,
           scene->paused ? "PAUSED" : "running");
  int slen = (int)strlen(status_text);
  if (slen > term_cols)
    slen = term_cols;

  attron(COLOR_PAIR(PAIR_HUD_STATUS) | A_BOLD);
  mvprintw(0, term_cols - slen, "%s", status_text);
  mvprintw(0, 0, " NUKE · FLUID ");
  attroff(COLOR_PAIR(PAIR_HUD_STATUS) | A_BOLD);

  attron(COLOR_PAIR(PAIR_HUD_HINT) | A_BOLD);
  mvprintw(term_rows - 1, 0,
           " q:quit  spc:pause  r:detonate  n/N:blast  t/T:theme  "
           "d/D:debug  +/-:rate  z/Z:zoom ");
  attroff(COLOR_PAIR(PAIR_HUD_HINT) | A_BOLD);
}

/* ── §23 screen — ncurses setup, teardown, and frame flush ── */

/*
 * Screen — just the terminal's current size in cells.  ncurses owns
 * the actual buffers; we only need the dimensions to place the HUD and
 * shape the camera.
 */
typedef struct {
    int rows;   /* terminal height in cells                        */
    int cols;   /* terminal width  in cells                        */
} Screen;

static void screen_init(Screen *screen) {
  initscr();
  noecho();
  cbreak();
  curs_set(0);
  nodelay(stdscr, TRUE);
  keypad(stdscr, TRUE);
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

static void screen_present_frame(Screen *screen, const Scene *scene,
                                 double real_fps, int target_render_fps) {
  erase();
  render_active_view(scene);
  hud_draw(screen->rows, screen->cols, scene, real_fps, target_render_fps);
  wnoutrefresh(stdscr);
  doupdate();
}

/* ── §24 scene — set up, reset, and advance the world each frame ── */

static void scene_init(Scene *scene, int cols, int rows) {
  memset(scene, 0, sizeof *scene);
  scene->paused = false;
  scene->theme_index = 0;
  scene->blast_type = BLAST_STANDARD;
  scene->debug_mode = DEBUG_NORMAL;
  scene->cols = cols;
  scene->rows = rows;
  scene->simulation_time_seconds = 0.0f;
  scene->simulation_rate = SIM_RATE_DEFAULT;
  scene->simulation_step_accumulator = 0.0f;
  scene->camera_distance = CAM_DISTANCE_DEFAULT;

  detonate_at_origin(&g_fluid, &BLAST_PRESETS[scene->blast_type]);
}

static void scene_resize(Scene *scene, int cols, int rows) {
  scene->cols = cols;
  scene->rows = rows;
}

static void scene_redetonate(Scene *scene) {
  scene->simulation_time_seconds = 0.0f;
  scene->simulation_step_accumulator = 0.0f;
  detonate_at_origin(&g_fluid, &BLAST_PRESETS[scene->blast_type]);
}

static void scene_cycle_blast(Scene *scene, int direction) {
  int new_index = (int)scene->blast_type + direction;
  while (new_index < 0)
    new_index += BLAST_TYPE_COUNT;
  scene->blast_type = (BlastType)(new_index % BLAST_TYPE_COUNT);
  scene_redetonate(scene);
}

/* Advance the simulation.  We bank up elapsed time and run the sim in
 * fixed little steps, so the cloud evolves the same way no matter what
 * frame rate the terminal manages. */
static void scene_tick(Scene *scene, float dt_real_seconds) {
  if (scene->paused)
    return;

  float dt_sim = dt_real_seconds * scene->simulation_rate;
  if (dt_sim > SIM_DT_MAX_REAL)
    dt_sim = SIM_DT_MAX_REAL;

  scene->simulation_step_accumulator += dt_sim;
  scene->simulation_time_seconds += dt_sim;

  while (scene->simulation_step_accumulator >= SIM_DT) {
    fluid_step(&g_fluid, SIM_DT);
    scene->simulation_step_accumulator -= SIM_DT;
  }
}

/* ── §25 app — the main loop, signals, and key handling ── */

/*
 * App — everything the program holds at the top level, in one global.
 *
 * A global lets the signal handlers reach the state the main loop
 * watches.  The two flags are volatile sig_atomic_t because that's the
 * only kind of variable a signal handler is allowed to set safely, and
 * volatile makes sure the loop re-reads them instead of caching a stale
 * copy.  target_render_fps lives here rather than in Scene because it's
 * a frame-pacing detail, not part of the simulation.
 */
typedef struct {
    Scene  scene;                          /* world + control state   */
    Screen screen;                         /* terminal size           */
    int    target_render_fps;              /* frame-rate cap          */
    volatile sig_atomic_t running;         /* cleared on quit signal  */
    volatile sig_atomic_t need_resize;     /* set on terminal resize  */
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

static void app_handle_resize(App *app) {
  screen_resize(&app->screen);
  scene_resize(&app->scene, app->screen.cols, app->screen.rows);
  app->need_resize = 0;
}

static bool app_handle_key(App *app, int ch) {
  Scene *scene = &app->scene;
  switch (ch) {
  case 'q':
  case 'Q':
  case 27:
    return false;
  case ' ':
    scene->paused = !scene->paused;
    break;
  case 'r':
  case 'R':
    scene_redetonate(scene);
    break;

  case 'n':
    scene_cycle_blast(scene, +1);
    break;
  case 'N':
    scene_cycle_blast(scene, -1);
    break;

  case 't':
    scene->theme_index = (scene->theme_index + 1) % THEME_COUNT;
    apply_theme(scene->theme_index);
    break;
  case 'T':
    scene->theme_index = (scene->theme_index + THEME_COUNT - 1) % THEME_COUNT;
    apply_theme(scene->theme_index);
    break;

  case 'd':
    scene->debug_mode = (DebugMode)((scene->debug_mode + 1) % DEBUG_MODE_COUNT);
    break;
  case 'D':
    scene->debug_mode = (DebugMode)((scene->debug_mode + DEBUG_MODE_COUNT - 1) %
                                    DEBUG_MODE_COUNT);
    break;

  case '=':
  case '+':
    scene->simulation_rate *= SIM_RATE_STEP_FACTOR;
    if (scene->simulation_rate > SIM_RATE_MAX)
      scene->simulation_rate = SIM_RATE_MAX;
    break;
  case '-':
    scene->simulation_rate /= SIM_RATE_STEP_FACTOR;
    if (scene->simulation_rate < SIM_RATE_MIN)
      scene->simulation_rate = SIM_RATE_MIN;
    break;

  case 'z':
    scene->camera_distance -= CAM_DISTANCE_STEP;
    if (scene->camera_distance < CAM_DISTANCE_MIN)
      scene->camera_distance = CAM_DISTANCE_MIN;
    break;
  case 'Z':
    scene->camera_distance += CAM_DISTANCE_STEP;
    if (scene->camera_distance > CAM_DISTANCE_MAX)
      scene->camera_distance = CAM_DISTANCE_MAX;
    break;

  case ']':
    app->target_render_fps += TARGET_RENDER_FPS_STEP;
    if (app->target_render_fps > TARGET_RENDER_FPS_MAX)
      app->target_render_fps = TARGET_RENDER_FPS_MAX;
    break;
  case '[':
    app->target_render_fps -= TARGET_RENDER_FPS_STEP;
    if (app->target_render_fps < TARGET_RENDER_FPS_MIN)
      app->target_render_fps = TARGET_RENDER_FPS_MIN;
    break;

  default:
    break;
  }
  return true;
}

int main(void) {
  atexit(screen_cleanup);
  signal(SIGINT, on_exit_signal);
  signal(SIGTERM, on_exit_signal);
  signal(SIGWINCH, on_resize_signal);

  App *app = &g_app;
  app->running = 1;
  app->target_render_fps = TARGET_RENDER_FPS_DEFAULT;

  screen_init(&app->screen);
  scene_init(&app->scene, app->screen.cols, app->screen.rows);

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
      app_handle_resize(app);
      prev_frame_ns = clock_now_ns();
    }

    /* ── dt ── */
    int64_t dt_ns = frame_start_ns - prev_frame_ns;
    prev_frame_ns = frame_start_ns;
    if (dt_ns > 100 * NS_PER_MS)
      dt_ns = 100 * NS_PER_MS;
    float dt_real_seconds = (float)dt_ns / (float)NS_PER_SEC;

    /* ── physics ── */
    scene_tick(&app->scene, dt_real_seconds);

    /* ── fps window ── */
    frames_in_window++;
    fps_window_ns += dt_ns;
    if (fps_window_ns >= FPS_DISPLAY_UPDATE_MS * NS_PER_MS) {
      measured_fps = (double)frames_in_window /
                     ((double)fps_window_ns / (double)NS_PER_SEC);
      frames_in_window = 0;
      fps_window_ns = 0;
    }

    /* ── render ── */
    screen_present_frame(&app->screen, &app->scene, measured_fps,
                         app->target_render_fps);

    /* ── frame cap ── */
    int64_t target_frame_ns = TICK_NS(app->target_render_fps);
    int64_t spent = clock_now_ns() - frame_start_ns;
    if (spent < target_frame_ns)
      clock_sleep_ns(target_frame_ns - spent);
  }

  return 0;
}
