/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * reaction_diffusion_gray_scott.c
 *   — Two chemicals spread and react on a grid and grow patterns on
 *     their own: spots, stripes, mazes, coral, worms. 30 presets to
 *     flip between with n/p. Watch each one grow from a tiny seed.
 *
 * The idea is Alan Turing's: chemicals that spread at different speeds
 * and feed off each other can settle into the kinds of textures you
 * see on animal coats. The specific recipe here is Gray-Scott.
 *
 * References (things the code can't tell you):
 *   Gray & Scott (1983), Chem. Eng. Sci. 38(1):29-43 — the chemistry.
 *   Pearson (1993), Science 261:189-192 — the map of which feed/kill
 *     settings produce which pattern; the named presets come from here.
 *   Munafo's xmorphia (https://mrob.com/pub/comp/xmorphia/) and Karl
 *     Sims' interactive explorer — sources of the named presets.
 *
 * Sister files:
 *   ../../fluid/reaction_diffusion.c — same algorithm, taught as a
 *     deeper math/numerical-methods lesson. This file is the lighter
 *     pattern-watching demo.
 *   ./perin_noise_flow_showcase.c — also paints a value over a grid,
 *     but that value is sampled from a fixed noise function instead of
 *     growing over time like this one does.
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (re-seed)
 *   n / N      next / previous... n=next, p=previous preset
 *   p / P      previous preset
 *   t / T      next / previous theme
 *   + / =      faster (more sim steps per frame)
 *   -          slower
 *   ] / [      raise / lower tick rate
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra reaction_diffusion_gray_scott.c \
 *       -o gray_scott -lncurses -lm
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

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* How many simulation steps to run per drawn frame. More steps =
     * the pattern grows faster on screen. 6 lets one settle in about
     * ten seconds. */
    SUB_STEPS_MIN     =   1,
    SUB_STEPS_DEF     =   6,
    SUB_STEPS_MAX     = 128,

    /* Color slots. First two are reserved for the HUD across all demos. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BAND_BASE    =   3,    /* 4 colour bands live at BASE..BASE+3 */
    PAIR_FLASH        =   7,    /* unused here; kept so themes match sister files */
};

/* How fast each chemical spreads, and the time step size. These are the
 * standard Gray-Scott numbers; V (the slower one) must spread slower than
 * U for any pattern to form at all. */
#define GS_DU               0.16f
#define GS_DV               0.08f
#define GS_DT               1.0f

/* The amount of V in a cell is usually small (peaks around 0.4), so we
 * scale it up before turning it into brightness, otherwise everything
 * looks faint. */
#define V_RENDER_GAIN       2.5f

/* Below this brightness a cell is left blank. */
#define GLOW_THRESHOLD      0.05f

/* Rows we keep clear for the HUD: two at the top (title + status), one
 * at the bottom (key hints). The pattern fills everything in between. */
#define HUD_TOP_ROWS             2
#define HUD_BOTTOM_ROWS          1
#define HUD_BAND_RESERVED_ROWS   (HUD_TOP_ROWS + HUD_BOTTOM_ROWS)
#define HUD_LEFT_MARGIN          1

/* Brightness cutoffs for picking a character: above HIGH = '#',
 * above MID = '*', otherwise '.'. */
#define GLYPH_HIGH_THRESH   0.65f
#define GLYPH_MID_THRESH    0.30f

/*
 * The 30 presets. Each is one feed/kill setting (F, k) that grows a
 * different pattern; cycle with n/p. All 30 run on the exact same
 * simulation code — only these two numbers change. To add one: add an
 * enum value, a name in pattern_name, and a row in gs_presets.
 *
 * They're grouped into five loose tiers, easiest to wildest:
 *   1 — the five most recognisable patterns
 *   2 — Pearson's lettered regions (alpha through epsilon)
 *   3 — named patterns from Karl Sims and Munafo's catalogue
 *   4 — ones that keep moving instead of settling
 *   5 — extreme / chaotic edges of the map
 */
typedef enum {
    /* Tier 1 — classic stable patterns */
    PATTERN_SPOTS    =  0,
    PATTERN_STRIPES  =  1,
    PATTERN_MAZES    =  2,
    PATTERN_CORAL    =  3,
    PATTERN_WORMS    =  4,

    /* Tier 2 — Pearson 1993 letter regions */
    PATTERN_ALPHA    =  5,
    PATTERN_BETA     =  6,
    PATTERN_GAMMA    =  7,
    PATTERN_DELTA    =  8,
    PATTERN_EPSILON  =  9,

    /* Tier 3 — named patterns (from Karl Sims / Munafo) */
    PATTERN_MITOSIS  = 10,
    PATTERN_SOLITON  = 11,
    PATTERN_FINGERS  = 12,
    PATTERN_SKATE    = 13,
    PATTERN_PSI      = 14,

    /* Tier 4 — dynamic / travelling patterns */
    PATTERN_WAVES    = 15,
    PATTERN_PULSES   = 16,
    PATTERN_NEBULA   = 17,
    PATTERN_NEURONS  = 18,
    PATTERN_CHAOS    = 19,

    /* Tier 5 — extreme / complex regions */
    PATTERN_HOLES    = 20,
    PATTERN_BUBBLES  = 21,
    PATTERN_SLUDGE   = 22,
    PATTERN_PLANKTON = 23,
    PATTERN_CRYSTAL  = 24,
    PATTERN_THICKET  = 25,
    PATTERN_DRIFT    = 26,
    PATTERN_ZEBRA    = 27,
    PATTERN_BANDS    = 28,
    PATTERN_BARNACLE = 29,

    N_PATTERNS       = 30,
} Pattern;

static const char *pattern_name(Pattern p)
{
    switch (p) {
    case PATTERN_SPOTS:    return "SPOTS    ";
    case PATTERN_STRIPES:  return "STRIPES  ";
    case PATTERN_MAZES:    return "MAZES    ";
    case PATTERN_CORAL:    return "CORAL    ";
    case PATTERN_WORMS:    return "WORMS    ";
    case PATTERN_ALPHA:    return "ALPHA    ";
    case PATTERN_BETA:     return "BETA     ";
    case PATTERN_GAMMA:    return "GAMMA    ";
    case PATTERN_DELTA:    return "DELTA    ";
    case PATTERN_EPSILON:  return "EPSILON  ";
    case PATTERN_MITOSIS:  return "MITOSIS  ";
    case PATTERN_SOLITON:  return "SOLITON  ";
    case PATTERN_FINGERS:  return "FINGERS  ";
    case PATTERN_SKATE:    return "SKATE    ";
    case PATTERN_PSI:      return "PSI      ";
    case PATTERN_WAVES:    return "WAVES    ";
    case PATTERN_PULSES:   return "PULSES   ";
    case PATTERN_NEBULA:   return "NEBULA   ";
    case PATTERN_NEURONS:  return "NEURONS  ";
    case PATTERN_CHAOS:    return "CHAOS    ";
    case PATTERN_HOLES:    return "HOLES    ";
    case PATTERN_BUBBLES:  return "BUBBLES  ";
    case PATTERN_SLUDGE:   return "SLUDGE   ";
    case PATTERN_PLANKTON: return "PLANKTON ";
    case PATTERN_CRYSTAL:  return "CRYSTAL  ";
    case PATTERN_THICKET:  return "THICKET  ";
    case PATTERN_DRIFT:    return "DRIFT    ";
    case PATTERN_ZEBRA:    return "ZEBRA    ";
    case PATTERN_BANDS:    return "BANDS    ";
    case PATTERN_BARNACLE: return "BARNACLE ";
    default:               return "?        ";
    }
}

/*
 * GSPreset — the two dials that decide which pattern grows. Just a pair
 * of numbers; every preset in the table below is one (F, k) setting.
 *
 *   F — feed rate. How fast fresh U is topped up where it ran low.
 *       Useful values run roughly 0.010 to 0.075.
 *   k — kill rate. How fast V fades away where it isn't being made.
 *       Useful values run roughly 0.040 to 0.070.
 *
 * Stay inside those ranges. Push F+k too high (past about 0.135) and V
 * dies off completely, leaving an empty screen. All 30 presets sit in
 * the sweet spot.
 *
 * The map of which setting gives which pattern is from Pearson (1993),
 * Science 261:189-192, Figure 1.
 */
typedef struct {
    float F;     /* feed rate — tops U back up where it ran low      */
    float k;     /* kill rate — fades V away where it isn't being made */
} GSPreset;

static const GSPreset gs_presets[N_PATTERNS] = {
    /*  F        k */

    /* Tier 1 — classic stable patterns */
    { 0.035f, 0.065f },   /* SPOTS    — mitosis-like dividing spots (Pearson γ) */
    { 0.022f, 0.051f },   /* STRIPES  — stable parallel stripes                  */
    { 0.029f, 0.057f },   /* MAZES    — labyrinth (Pearson θ)                    */
    { 0.054f, 0.063f },   /* CORAL    — branching coral / dendrites              */
    { 0.046f, 0.063f },   /* WORMS    — slowly-travelling worms (Pearson κ)      */

    /* Tier 2 — Pearson 1993 letter regions */
    { 0.010f, 0.048f },   /* ALPHA    — slow / quiescent waves (Pearson α)       */
    { 0.014f, 0.045f },   /* BETA     — travelling-wave fingerprints (Pearson β) */
    { 0.026f, 0.055f },   /* GAMMA    — large-cell mitosis (Pearson γ proper)    */
    { 0.042f, 0.059f },   /* DELTA    — fingerprint-like (Pearson δ)             */
    { 0.018f, 0.055f },   /* EPSILON  — labyrinth+spots (Pearson ε)              */

    /* Tier 3 — named patterns (from Karl Sims / Munafo) */
    { 0.0367f, 0.0649f }, /* MITOSIS  — rapid spot division (Sims preset)        */
    { 0.014f,  0.054f },  /* SOLITON  — solitary travelling waves                */
    { 0.046f,  0.065f },  /* FINGERS  — finger-growth fronts (Pearson μ)         */
    { 0.062f,  0.0609f }, /* SKATE    — U-skate world (Munafo)                   */
    { 0.044f,  0.063f },  /* PSI      — ψ region (Munafo)                        */

    /* Tier 4 — dynamic / travelling patterns */
    { 0.014f,  0.045f },  /* WAVES    — travelling wave fronts                   */
    { 0.025f,  0.060f },  /* PULSES   — periodic radial pulses                   */
    { 0.018f,  0.051f },  /* NEBULA   — diffuse cloudy blobs                     */
    { 0.041f,  0.0635f }, /* NEURONS  — neuron-like network                      */
    { 0.046f,  0.0594f }, /* CHAOS    — chaotic mixing (Pearson ι)               */

    /* Tier 5 — extreme / complex regions */
    { 0.039f,  0.058f },  /* HOLES    — inverted spots (V dominates)             */
    { 0.012f,  0.052f },  /* BUBBLES  — large round blobs                        */
    { 0.018f,  0.053f },  /* SLUDGE   — slow dense gel-like                      */
    { 0.022f,  0.0585f }, /* PLANKTON — drifting spot fields                     */
    { 0.040f,  0.0608f }, /* CRYSTAL  — crystalline / sharp-edged                */
    { 0.060f,  0.062f },  /* THICKET  — dense branching network                  */
    { 0.020f,  0.057f },  /* DRIFT    — slowly drifting maze                     */
    { 0.026f,  0.052f },  /* ZEBRA    — tight high-contrast stripes              */
    { 0.034f,  0.060f },  /* BANDS    — loose broad stripes                      */
    { 0.0376f, 0.06277f },/* BARNACLE — clustered bumpy spots (Munafo frogspawn) */
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* If a frame takes a long time (window resize, machine stall), cap how
 * much time we let the sim catch up so it can't get stuck. And don't
 * redraw the terminal more than 60 times a second. */
#define DT_MAX_NS       (100 * NS_PER_MS)
#define FRAME_CAP_FPS   60

/* We sort each cell's brightness into one of four colour bands. The
 * scale (just under 4) maps a 0..1 brightness onto bands 0..3. */
#define N_PALETTE_BANDS      4
#define GLOW_COL_SCALE       3.999f

/* Widths of the three fixed-width chunks on the status row, so each one
 * knows where the next begins. */
#define HUD_PATTERN_FIELD_W   20    /* " pattern:XXXXXXXXX " slot */
#define HUD_THEME_FIELD_W     17    /* " theme:XXXXXXXX "   slot */
#define HUD_PALETTE_LABEL_W    9    /* " palette:"          slot */

/*
 * Theme — one colour scheme for the display. There are ten; cycle them
 * with t/T.
 *
 * band[] is four colours from dim to bright. We sort each cell into one
 * of those four by how much V it holds, so the faintest cells get
 * band[0] and the strongest get band[3]. Design a theme as a smooth
 * dark-to-light run of four colours.
 *
 * The numbers are xterm 256-colour codes, not RGB. On an old terminal
 * with fewer colours, theme_apply falls back to a fixed set of basic
 * colours so the demo still runs.
 *
 * flash is unused here — kept only so this theme layout matches the
 * sister field demos.
 */
typedef struct {
    const char *name;          /* short label shown in the HUD       */
    short       band[4];       /* four colours, dim (0) to bright (3) */
    short       flash;         /* unused; kept to match sister files  */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      band0 band1 band2 band3 flash */
    { "DEFAULT", {  17,   33,  220,  231 }, 226 },
    { "MATRIX",  {  22,   34,   46,  118 }, 226 },
    { "NOVA",    {  53,  129,  201,  219 }, 226 },
    { "MONO",    { 234,  244,  250,  254 }, 226 },
    { "OCEAN",   {  17,   33,   39,   51 }, 226 },
    { "FIRE",    {  52,  124,  208,  226 }, 196 },
    { "EARTH",   {  58,  100,  173,  230 }, 226 },
    { "FOREST",  {  22,   28,   64,  144 }, 226 },
    { "DESERT",  {  94,  130,  173,  222 }, 226 },
    { "ARCTIC",  {  18,   39,  159,  231 }, 226 },
};

/* ===================================================================== */
/* §2  clock                                                              */
/* ===================================================================== */

static int64_t clock_ns(void)
{
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (int64_t)t.tv_sec * NS_PER_SEC + t.tv_nsec;
}

static void clock_sleep_ns(int64_t ns)
{
    if (ns <= 0) return;
    struct timespec req = {
        .tv_sec  = (time_t)(ns / NS_PER_SEC),
        .tv_nsec = (long)  (ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, t->band[i], -1);
        init_pair(PAIR_FLASH, t->flash, -1);
    } else {
        static const short fallback[4] = {
            COLOR_BLUE, COLOR_CYAN, COLOR_MAGENTA, COLOR_YELLOW,
        };
        for (int i = 0; i < 4; i++)
            init_pair(PAIR_BAND_BASE + i, fallback[i], -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,   226, -1);
        init_pair(PAIR_HINT,   51, -1);
    } else {
        init_pair(PAIR_HUD,  COLOR_YELLOW, -1);
        init_pair(PAIR_HINT, COLOR_CYAN,   -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  rd — Gray-Scott reaction-diffusion solver                          */
/* ===================================================================== */

/*
 * Grid — just the size of the playing field, in cells. Nearly everything
 * needs the width and height, so they live here at the top.
 *
 * w * h never exceeds CELLS_MAX (200 x 56) — app_pick_map_size keeps it
 * in bounds, since that's how big the chemical arrays are.
 */
typedef struct {
    int w, h;            /* width and height, in cells          */
    int total_cells;     /* w * h, kept here so loops skip the multiply */
} Grid;

static inline int  grid_idx       (const Grid *g, int x, int y) { return y * g->w + x; }
static inline bool grid_in_bounds (const Grid *g, int x, int y)
{
    return x >= 0 && x < g->w && y >= 0 && y < g->h;
}

/*
 * ReactionField — how much of each chemical sits in every cell, plus the
 * feed/kill dials for the current preset. This is the whole simulation
 * state; the solver reads and writes only this struct.
 *
 * The two chemicals are U and V. Think of U as the plentiful background
 * and V as the stuff that eats U and multiplies — V is what you see on
 * screen. They spread at different speeds, and that difference is the
 * whole trick: it's what lets patterns form instead of a smooth blur.
 *
 * Edges are sealed, like a petri dish — nothing leaks off the grid.
 * (See clamp_neumann_lo/hi for how that's done.)
 */
typedef struct {
    /* How much of the plentiful chemical is in each cell. Sits near 1
     * wherever V isn't eating it. */
    float U     [CELLS_MAX];

    /* How much of the active chemical is in each cell — this is what
     * gets drawn. Starts as a tiny central blob and spreads from there. */
    float V     [CELLS_MAX];

    /* Scratch copies. Each step writes the new amounts here, then we copy
     * them back. We need separate copies because every cell's new value
     * depends on its neighbours' OLD values — overwrite in place and
     * you'd corrupt neighbours you haven't visited yet. */
    float U_next[CELLS_MAX];
    float V_next[CELLS_MAX];

    /* The current preset's two dials. install_kinetics sets these; they
     * stay fixed until you switch presets. See GSPreset for the ranges. */
    float F;     /* feed rate */
    float k;     /* kill rate */
} ReactionField;

/* ── the four small pieces of one simulation step ──────────────── *
 * Each does one job. step_reaction_field below stitches them together. */

/* When a neighbour would fall off the grid, pretend the edge cell is its
 * own neighbour instead. That seals the edges so nothing leaks out. */
static inline int clamp_neumann_lo(int c)        { return (c > 0)     ? c - 1 : c; }
static inline int clamp_neumann_hi(int c, int n) { return (c < n - 1) ? c + 1 : c; }

/* How fast a chemical is spreading at this cell: compare it to its four
 * neighbours. Positive means neighbours have more, so it flows in. This
 * is what "diffusion" means in one number. */
static inline float laplacian_5pt(const float *f,
                                   int idx, int il, int ir, int iu, int id)
{
    return f[il] + f[ir] + f[iu] + f[id] - 4.0f * f[idx];
}

/* The change in U for this cell over one step: it spreads in from
 * neighbours, gets eaten where V is present, and is topped back up. */
static inline float gray_scott_dU(float u, float v, float lap_U,
                                   float F, float Du)
{
    return Du * lap_U - u * v * v + F * (1.0f - u);
}

/* The change in V: it spreads in, grows wherever it can eat U, and fades
 * out everywhere at the kill rate. */
static inline float gray_scott_dV(float u, float v, float lap_V,
                                   float F, float k, float Dv)
{
    return Dv * lap_V + u * v * v - (F + k) * v;
}

/* Make the freshly-computed values the current ones, ready for the next
 * step. */
static void swap_pde_buffers(ReactionField *rxn, int total_cells)
{
    size_t bytes = (size_t)total_cells * sizeof(float);
    memcpy(rxn->U, rxn->U_next, bytes);
    memcpy(rxn->V, rxn->V_next, bytes);
}

/* ── setting up the starting state ─────────────────────────────── */

/* The starting seed: a small blob of V dropped in the middle. Its size is
 * a fraction of the grid; the U/V amounts are what's inside it. The jitter
 * is a tiny random nudge to U everywhere (see below for why). */
#define SEED_BLOB_RADIUS_DENOM  12         /* blob is 1/12 of the shorter edge */
#define SEED_BLOB_MIN_RADIUS    3
#define SEED_U_BLOB             0.5f
#define SEED_V_BLOB             0.25f
#define JITTER_RANGE            100        /* random pick from 0..99 */
#define JITTER_HALFRANGE         50        /* shift it to -50..+49   */
#define JITTER_GAIN             0.0002f    /* then scale it way down  */
#define LAPLACIAN_CENTRE_WEIGHT 4.0f
#define U_FIXED_POINT           1.0f       /* the "nothing happening" state */

/* Reset the whole grid to the calm starting state: U full everywhere, no
 * V. On its own this just sits there frozen — the seed and jitter below
 * are what get things moving. */
static void clear_concentrations(ReactionField *rxn, int total_cells)
{
    for (int i = 0; i < total_cells; i++) {
        rxn->U[i] = U_FIXED_POINT;
        rxn->V[i] = 0.0f;
    }
}

/* Drop a small disc of V in the centre. Without this first patch of V,
 * nothing ever happens — the grid stays calm forever. */
static void seed_central_blob(ReactionField *rxn, const Grid *g)
{
    int cx = g->w / 2;
    int cy = g->h / 2;
    int rad = (g->w < g->h ? g->w : g->h) / SEED_BLOB_RADIUS_DENOM;
    if (rad < SEED_BLOB_MIN_RADIUS) rad = SEED_BLOB_MIN_RADIUS;
    int rad_sq = rad * rad;
    for (int dy = -rad; dy <= rad; dy++) {
        for (int dx = -rad; dx <= rad; dx++) {
            int x = cx + dx, y = cy + dy;
            if (!grid_in_bounds(g, x, y))   continue;
            if (dx * dx + dy * dy > rad_sq) continue;
            int idx = grid_idx(g, x, y);
            rxn->U[idx] = SEED_U_BLOB;
            rxn->V[idx] = SEED_V_BLOB;
        }
    }
}

/* Sprinkle a tiny bit of randomness onto U everywhere. A perfectly
 * symmetric start would only ever grow perfectly symmetric rings; the
 * interesting patterns need some unevenness to feed on. */
static void salt_with_symmetry_breaking_jitter(ReactionField *rxn, int total_cells)
{
    for (int i = 0; i < total_cells; i++) {
        int   centred = rand() % JITTER_RANGE - JITTER_HALFRANGE;
        float jitter  = (float)centred * JITTER_GAIN;
        rxn->U[i] += jitter;
    }
}

static void seed_reaction_field(ReactionField *rxn, const Grid *g)
{
    clear_concentrations                 (rxn, g->total_cells);
    seed_central_blob                    (rxn, g);
    salt_with_symmetry_breaking_jitter   (rxn, g->total_cells);
}

/* Advance the whole grid by one step: for every cell, see how each
 * chemical is spreading, work out its change, and store the new amounts. */
static void step_reaction_field(ReactionField *rxn, const Grid *g)
{
    const float Du = GS_DU, Dv = GS_DV, dt = GS_DT;
    const float F  = rxn->F, k = rxn->k;
    const int   w  = g->w,   h = g->h;

    for (int y = 0; y < h; y++) {
        int yu = clamp_neumann_lo(y);
        int yd = clamp_neumann_hi(y, h);
        for (int x = 0; x < w; x++) {
            int xl = clamp_neumann_lo(x);
            int xr = clamp_neumann_hi(x, w);

            int idx = y  * w + x;
            int il  = y  * w + xl;
            int ir  = y  * w + xr;
            int iu  = yu * w + x;
            int id  = yd * w + x;

            float u = rxn->U[idx];
            float v = rxn->V[idx];

            float lap_U = laplacian_5pt(rxn->U, idx, il, ir, iu, id);
            float lap_V = laplacian_5pt(rxn->V, idx, il, ir, iu, id);

            rxn->U_next[idx] = u + dt * gray_scott_dU(u, v, lap_U, F, Du);
            rxn->V_next[idx] = v + dt * gray_scott_dV(u, v, lap_V, F, k, Dv);
        }
    }
    swap_pde_buffers(rxn, g->total_cells);
}

/* ===================================================================== */
/* §6  patterns — preset (F, k) plug-ins                                  */
/* ===================================================================== */

/* Load a preset's two dials into the field. After calling this you must
 * re-seed, since switching presets means starting the pattern over. */
static void install_kinetics(ReactionField *rxn, Pattern p)
{
    if (p < 0 || p >= N_PATTERNS) p = PATTERN_SPOTS;
    rxn->F = gs_presets[p].F;
    rxn->k = gs_presets[p].k;
}

/* ===================================================================== */
/* §7  scene                                                              */
/* ===================================================================== */

/*
 * The whole program state is one Scene made of four small parts, each
 * holding one kind of thing. Splitting it up this way means a function's
 * arguments tell you what it can touch: take a `const Grid *` and you
 * clearly can't change the chemistry, and so on.
 */

/* The one number the simulation itself updates each step: how many steps
 * have run since the last reset. Shown in the HUD so you can compare how
 * long different patterns take to settle. */
typedef struct {
    int step_count;
} SimState;

/* The knobs the user turns with the keyboard. Only the key handler writes
 * these; the rest of the code just reads them. */
typedef struct {
    bool    paused;
    int     sub_steps;              /* sim steps per drawn frame */
    int     current_theme;
    Pattern current_pattern;
} Controls;

/* Everything the program needs, in one place. Reading it top to bottom is
 * the quickest way to understand the whole thing. */
typedef struct {
    Grid          grid;       /* size of the field                  */
    ReactionField reaction;   /* the chemicals + current preset     */
    SimState      sim;        /* step counter                       */
    Controls      ctrl;       /* user's keyboard settings           */
} Scene;

/* ── starting (or restarting) the simulation ─────────────────────── */

static void apply_grid_dimensions(Grid *g, int w, int h)
{
    g->w           = w;
    g->h           = h;
    g->total_cells = w * h;
}

static void reset_sim_state(SimState *sim)
{
    sim->step_count = 0;
}

static void scene_reset(Scene *s, int w, int h)
{
    apply_grid_dimensions  (&s->grid, w, h);
    install_kinetics       (&s->reaction, s->ctrl.current_pattern);
    seed_reaction_field    (&s->reaction, &s->grid);
    reset_sim_state        (&s->sim);
}

static void scene_init(Scene *s, int w, int h)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused           = false;
    s->ctrl.sub_steps        = SUB_STEPS_DEF;
    s->ctrl.current_theme    = 0;
    s->ctrl.current_pattern  = PATTERN_SPOTS;
    scene_reset(s, w, h);
}

/* ── one frame of simulation ─────────────────────────────────────── */

static void run_sub_steps(Scene *s)
{
    int n = s->ctrl.sub_steps;
    for (int i = 0; i < n; i++) {
        step_reaction_field(&s->reaction, &s->grid);
        s->sim.step_count++;
    }
}

static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;
    (void)dt;     /* sim runs on its own fixed step, not real elapsed time */

    run_sub_steps(s);
}

/* ===================================================================== */
/* §8  screen                                                             */
/* ===================================================================== */

/* The terminal's current size. We re-read it whenever the window is
 * resized. That's all we need to track ourselves — ncurses keeps the
 * rest. It's kept separate from the simulation because the sim doesn't
 * care how big the window is. */
typedef struct {
    int cols;   /* width  in characters */
    int rows;   /* height in characters */
} Screen;

static void screen_init(Screen *s)
{
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
static void screen_free(Screen *s) { (void)s; endwin(); }
static void screen_resize(Screen *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, s->rows, s->cols);
}

/* What to paint in one cell: which colour, bold or not, which character,
 * and whether to skip it entirely. Deciding (classify_v_cell) is kept
 * apart from drawing (paint_cell) so the decision has no terminal calls
 * mixed in. */
typedef struct {
    int  pair;
    int  attr;
    char glyph;
    bool skip;
} CellDraw;

/* Turn a cell's raw V amount into a 0..1 brightness. V is usually small,
 * so we scale it up first, then clamp to the 0..1 range. */
static inline float normalize_v_to_glow(float v_raw)
{
    float g = v_raw * V_RENDER_GAIN;
    if (g < 0.0f) g = 0.0f;
    if (g > 1.0f) g = 1.0f;
    return g;
}

/* Pick which of the four colour bands a brightness falls into. */
static inline int glow_to_band(float glow)
{
    return (int)(glow * GLOW_COL_SCALE) & (N_PALETTE_BANDS - 1);
}

/* Given a cell's brightness, decide how to draw it: pick a character and
 * colour, or skip it if it's too faint to bother with. Brighter cells get
 * heavier characters ('.', '*', '#'). */
static CellDraw classify_v_cell(float glow)
{
    if (glow > GLYPH_HIGH_THRESH) {
        int pair = PAIR_BAND_BASE + glow_to_band(glow);
        return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '#' };
    }
    if (glow > GLYPH_MID_THRESH) {
        int pair = PAIR_BAND_BASE + glow_to_band(glow);
        return (CellDraw){ .pair = pair, .attr = A_BOLD,   .glyph = '*' };
    }
    if (glow > GLOW_THRESHOLD) {
        int pair = PAIR_BAND_BASE + glow_to_band(glow);
        return (CellDraw){ .pair = pair, .attr = A_NORMAL, .glyph = '.' };
    }
    return (CellDraw){ .skip = true };
}

/* The single place where the pattern actually gets drawn to the screen. */
static void paint_cell(int sy, int sx, CellDraw c)
{
    if (c.skip) return;
    attron (COLOR_PAIR(c.pair) | c.attr);
    mvaddch(sy, sx, (chtype)(unsigned char)c.glyph);
    attroff(COLOR_PAIR(c.pair) | c.attr);
}

/* Work out where to put the top-left corner of the pattern so it sits
 * centred, leaving room for the HUD rows. */
static void compute_centred_origin(const Grid *g, int cols, int rows,
                                    int *out_gx0, int *out_gy0)
{
    int gx0 = (cols - g->w) / 2;
    int gy0 = ((rows - HUD_BAND_RESERVED_ROWS) - g->h) / 2 + HUD_TOP_ROWS;
    if (gx0 < 0)            gx0 = 0;
    if (gy0 < HUD_TOP_ROWS) gy0 = HUD_TOP_ROWS;
    *out_gx0 = gx0;
    *out_gy0 = gy0;
}

/* Draw the whole pattern: for every cell, turn its V into a brightness,
 * decide how to draw it, and paint it. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Grid          *g   = &s->grid;
    const ReactionField *rxn = &s->reaction;
    int gx0, gy0;
    compute_centred_origin(g, cols, rows, &gx0, &gy0);

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;
            int idx = grid_idx(g, x, y);
            paint_cell(sy, sx,
                        classify_v_cell(normalize_v_to_glow(rxn->V[idx])));
        }
    }
}

/* ── the on-screen text overlay ──────────────────────────────────── *
 * The top two rows show what's happening (fps, current preset, the
 * numbers behind it). The bottom row lists the keys you can press. */

static void draw_hud_state_bar(const Screen *sc, const Scene *s,
                                double fps, int sim_fps)
{
    const Controls *c = &s->ctrl;
    const char *state_str = c->paused ? "PAUSED   "
                                      : pattern_name(c->current_pattern);
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  %s [%d/%d]  sub:%-3d  step:%-7d ",
             fps, sim_fps, state_str,
             (int)c->current_pattern + 1, N_PATTERNS,
             c->sub_steps, s->sim.step_count);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

static void draw_hud_title(void)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, HUD_LEFT_MARGIN, " GRAY-SCOTT REACTION-DIFFUSION ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
}

/* ── the status row, drawn one chunk at a time ───────────────────── *
 * Each of these paints its chunk and returns where the next one starts. */

static int draw_status_pattern_field(int row, int x, Pattern p)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " pattern:%-9s ", pattern_name(p));
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_PATTERN_FIELD_W;
}

static int draw_status_theme_field(int row, int x, int theme_idx)
{
    attron (COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(row, x, " theme:%-8s ", themes[theme_idx].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    return x + HUD_THEME_FIELD_W;
}

static int draw_status_palette_label(int row, int x)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    return x + HUD_PALETTE_LABEL_W;
}

static int draw_palette_swatch(int row, int x)
{
    for (int i = 0; i < N_PALETTE_BANDS; i++) {
        int pair = PAIR_BAND_BASE + i;
        attron (COLOR_PAIR(pair) | A_BOLD);
        mvaddch(row, x, '#');
        attroff(COLOR_PAIR(pair) | A_BOLD);
        x++;
    }
    return x;
}

/* Last chunk: the actual numbers behind the current pattern (feed, kill,
 * spread rates) and the grid size. They change as you cycle presets. */
static void draw_kinetics_readout(int row, int x, const Scene *s)
{
    attron (COLOR_PAIR(PAIR_HUD));
    mvprintw(row, x, "  F=%.4f  k=%.4f  Du=%.2f  Dv=%.2f  map:%dx%d ",
             (double)s->reaction.F, (double)s->reaction.k,
             (double)GS_DU, (double)GS_DV,
             s->grid.w, s->grid.h);
    attroff(COLOR_PAIR(PAIR_HUD));
}

static void draw_hud_status_line(const Scene *s)
{
    const Controls *c = &s->ctrl;
    int x = HUD_LEFT_MARGIN;
    x = draw_status_pattern_field (1, x, c->current_pattern);
    x = draw_status_theme_field   (1, x, c->current_theme);
    x = draw_status_palette_label (1, x);
    x = draw_palette_swatch       (1, x);
        draw_kinetics_readout     (1, x, s);
}

/* The bottom row: the list of keys you can press. */
static void draw_bottom_hint(const Screen *sc)
{
    attron (COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " n/p:pattern  t/T:theme  r:reset  spc:pause  +/-:sub-steps  ]/[:Hz  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw           (s, sc->cols, sc->rows);
    draw_hud_state_bar   (sc, s, fps, sim_fps);
    draw_hud_title       ();
    draw_hud_status_line (s);
    draw_bottom_hint     (sc);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §9  app                                                                */
/* ===================================================================== */

/*
 * App — the top-level state for the whole program. There's just one,
 * g_app, kept global so the signal handlers can reach it.
 *
 * The two flags are written by signal handlers (when you press Ctrl-C or
 * resize the window) and read by the main loop. The handlers only flip a
 * flag; the actual work happens back in the main loop, because doing real
 * work inside a signal handler isn't safe. The volatile + sig_atomic_t on
 * those flags is what makes that hand-off safe and reliable.
 */
typedef struct {
    Scene                 scene;     /* the simulation              */
    Screen                screen;    /* terminal size               */

    int                   sim_fps;   /* tick rate; '[' and ']' change it */
    int                   map_w;     /* grid width  (capped)        */
    int                   map_h;     /* grid height (capped)        */

    volatile sig_atomic_t running;       /* set to 0 to quit        */
    volatile sig_atomic_t need_resize;   /* set when the window resized */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_BAND_RESERVED_ROWS;
    if (mw < 16) mw = 16;
    if (mh < 8)  mh = 8;
    if (mw > MAP_W_MAX) mw = MAP_W_MAX;
    if (mh > MAP_H_MAX) mh = MAP_H_MAX;
    app->map_w = mw;
    app->map_h = mh;
}

static void app_do_resize(App *app)
{
    screen_resize(&app->screen);
    app_pick_map_size(app);
    scene_reset(&app->scene, app->map_w, app->map_h);
    app->need_resize = 0;
}

/* ── keyboard ─────────────────────────────────────────────────────── */

/* +/- speed: doubles or halves the steps-per-frame, so each press makes a
 * bigger jump than the last instead of nudging by a fixed amount. */
static void bump_sub_steps(Controls *c, int dir)
{
    if (dir > 0) {
        if (c->sub_steps < SUB_STEPS_MAX) c->sub_steps *= 2;
        if (c->sub_steps > SUB_STEPS_MAX) c->sub_steps = SUB_STEPS_MAX;
    } else {
        c->sub_steps /= 2;
        if (c->sub_steps < SUB_STEPS_MIN) c->sub_steps = SUB_STEPS_MIN;
    }
}

static void bump_sim_fps(App *app, int delta)
{
    app->sim_fps += delta;
    if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
    if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
}

static void cycle_theme(Controls *c, int dir)
{
    c->current_theme = (c->current_theme + dir + N_THEMES) % N_THEMES;
    theme_apply(c->current_theme);
}

/* Switch presets with n/p, and start the new one over from scratch so you
 * watch it grow — every preset has its own settings and its own seed. */
static void cycle_pattern(App *app, int dir)
{
    Controls *c = &app->scene.ctrl;
    c->current_pattern = (Pattern)(
        ((int)c->current_pattern + dir + N_PATTERNS) % N_PATTERNS);
    scene_reset(&app->scene, app->map_w, app->map_h);
}

static bool app_handle_key(App *app, int ch)
{
    Controls *c = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':           c->paused = !c->paused;                              break;
    case 'r': case 'R': scene_reset(&app->scene, app->map_w, app->map_h);    break;
    case '=': case '+': bump_sub_steps (c,   +1);                            break;
    case '-':           bump_sub_steps (c,   -1);                            break;
    case ']':           bump_sim_fps   (app, +SIM_FPS_STEP);                 break;
    case '[':           bump_sim_fps   (app, -SIM_FPS_STEP);                 break;
    case 't':           cycle_theme    (c,   +1);                            break;
    case 'T':           cycle_theme    (c,   -1);                            break;
    case 'n': case 'N': cycle_pattern  (app, +1);                            break;
    case 'p': case 'P': cycle_pattern  (app, -1);                            break;
    default: break;
    }
    return true;
}

/* ── the main loop's helpers ────────────────────────────────────────── */

static void install_signal_handlers(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* How much real time passed since the last frame, with a ceiling so a long
 * pause (a stall, a resize) can't make the sim try to catch up forever. */
static int64_t advance_frame_clock(int64_t *frame_time)
{
    int64_t now = clock_ns();
    int64_t dt  = now - *frame_time;
    *frame_time = now;
    if (dt > DT_MAX_NS) dt = DT_MAX_NS;
    return dt;
}

/* Run as many sim ticks as the elapsed time has earned, so the simulation
 * keeps a steady pace no matter how fast the screen redraws. */
static void simulate_pending_ticks(App *app, int64_t *sim_accum,
                                    int64_t tick_ns, float dt_sec)
{
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/* Update the displayed frames-per-second roughly twice a second, rather
 * than recomputing it every single frame (which would jitter). */
static double maybe_update_fps_counter(int64_t *fps_accum,
                                        int *frame_count,
                                        double previous)
{
    if (*fps_accum < FPS_UPDATE_MS * NS_PER_MS) return previous;
    double fps = (double)(*frame_count) /
                  ((double)(*fps_accum) / (double)NS_PER_SEC);
    *frame_count = 0;
    *fps_accum   = 0;
    return fps;
}

/* Wait out the rest of the frame so we don't redraw faster than needed. */
static void cap_frame_rate(int64_t work_done_ns, int target_fps)
{
    int64_t budget = NS_PER_SEC / target_fps;
    clock_sleep_ns(budget - work_done_ns);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signal_handlers();

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init      (&app->screen);
    app_pick_map_size(app);
    scene_init       (&app->scene, app->map_w, app->map_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t dt      = advance_frame_clock(&frame_time);
        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        simulate_pending_ticks(app, &sim_accum, tick_ns, dt_sec);

        frame_count++;
        fps_accum  += dt;
        fps_display = maybe_update_fps_counter(&fps_accum, &frame_count, fps_display);

        cap_frame_rate((clock_ns() - frame_time) + dt, FRAME_CAP_FPS);

        screen_draw   (&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
