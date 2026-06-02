/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * poission_disk_sampling_showcase.c
 *   — Bridson's Fast Poisson Disk Sampling, animated.
 *
 * DEMO: A single seed point appears at the map centre. From it, new
 *       points sprout — each placed at random in the annulus between
 *       r and 2r from a parent, but only accepted if it's at least r
 *       away from EVERY existing point. The accepted set grows
 *       outward in waves: glowing 'O' active points along the
 *       frontier try to spawn neighbours, '*' flashes mark fresh
 *       additions, and 'o' resting points fill the interior. After
 *       a couple of seconds the frontier collapses and the screen
 *       is covered in a "blue-noise" point cloud — uniformly
 *       distributed but with no two points closer than r, coloured as a
 *       radial gradient from the seed.  Holds, then resets and loops.
 *
 * Study alongside: ./drunkards_walk_cave_showcase.c — both grow from
 *       a centre seed but the math is opposite. Drunkard's walk is a
 *       random PROCESS (one walker, random steps); Poisson disk is
 *       a constraint-driven SAMPLER (no two points within r). The
 *       point clouds Poisson produces are what graphics people call
 *       "blue noise" — visually pleasing because the spacing is
 *       even but never aligned to a grid.
 *
 * Section map:
 *   §1 config+types — sizes, radius, attempts, themes, and all struct types
 *   §2 performance  — monotonic clock + frame cap
 *   §3 logic        — pure grid-index + candidate-distance checks
 *   §4 simulation   — Bridson step, reset, and the per-tick combine
 *   §5 render       — theme palette, ASCII point cloud, HUD
 *   §6 app          — signals, resize, key events, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (preserves theme)
 *   t / T      next / previous theme
 *   + / =      faster (more attempts/tick)
 *   -          slower
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra poission_disk_sampling_showcase.c \
 *       -o poisson_disk -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Bridson 2007 — "Fast Poisson Disk Sampling in Arbitrary
 *                  Dimensions". Maintains an "active list" of recently-
 *                  added points. Each iteration:
 *                    1. Pick a uniformly random active point P.
 *                    2. Generate a candidate at random angle θ ∈ [0, 2π)
 *                       and random distance d ∈ [r, 2r] from P.
 *                    3. Check the candidate against the BACKGROUND GRID:
 *                       only the points in the 5×5 cell box around the
 *                       candidate's grid cell can possibly be within r
 *                       (because each grid cell is sized r/√2, so any
 *                       cell holds at most one sample, and any point
 *                       beyond 2 cells away is > r away in Euclidean
 *                       distance).
 *                    4. If no existing point is within r → accept the
 *                       candidate. Add it to samples[], to the bg grid,
 *                       and to the active list.
 *                    5. After K = 30 unsuccessful attempts on point P,
 *                       remove P from the active list (its surroundings
 *                       are saturated).
 *                  Continue until the active list is empty. The result
 *                  is a Poisson-disk distribution: every pair of points
 *                  is ≥ r apart, the spacing is "blue noise" (no
 *                  visible structure), and the points fill space
 *                  efficiently.
 *
 * Data-structure : Continuous-position samples (float x, y) + a flat
 *                  background grid that maps each grid cell to either
 *                  an index in samples[] or -1 (empty). The grid
 *                  reduces distance queries from O(N) to O(1) — a
 *                  fixed 5×5 box of cells around any candidate.
 *                  Active queue is a flat int array; swap-remove for
 *                  O(1) deletion when a point is exhausted.
 *
 * Rendering      : ASCII only. 'o' for resting samples, 'O' for
 *                  samples still on the active list (the frontier),
 *                  '*' for the just-added flash. Continuous (x, y)
 *                  positions round to integer cell coordinates for
 *                  display; with radius ≥ 4 cells, no two visible
 *                  glyphs ever land on the same cell.
 *
 * Performance    : O(N · K) total work, where N is the final point
 *                  count and K = 30 attempts per active point. With
 *                  a 200×56 map and r=4, N ≈ 950 and total attempts
 *                  ≈ 14 000 — finishes in ~3 s at the default
 *                  ops_per_tick. The key trick is the bg-grid
 *                  distance query: each candidate checks at most
 *                  25 cells, not every existing point.
 *
 * References     : Concept —
 *                  [1] Bridson, R. (2007) — "Fast Poisson Disk Sampling in
 *                      Arbitrary Dimensions" (SIGGRAPH sketches).  The paper
 *                      this file animates; source of the r/√2 grid + K=30:
 *                      https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
 *                  [2] Wikipedia — "Supersampling" (Poisson-disc section):
 *                      https://en.wikipedia.org/wiki/Supersampling#Poisson_disc
 *                  [3] Bostock, Mike — "Visualizing Algorithms" (animated
 *                      Poisson-disk + blue-noise explainer):
 *                      https://bost.ocks.org/mike/algorithms/
 *                  [4] Quilez, Inigo — "Voronoi distances" (related blue-noise
 *                      / point-distribution techniques):
 *                      https://iquilezles.org/articles/voronoilines/
 *                  Rendering —
 *                  [5] Bourke, Paul — "Colour Ramping for Data Visualisation",
 *                      the model for colouring points by distance from the
 *                      seed: paulbourke.net/texture_colour/colourramp
 *                  [6] Padala — "NCURSES Programming HOWTO", TLDP: colour
 *                      pairs, glyph output, non-blocking input, resize.
 *                  [7] xterm 256-colour palette — the index the Theme ramps
 *                      draw from: https://jonasjacek.github.io/colors/
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Drop random points on a surface, BUT enforce a minimum distance r
 * between any pair. The result is "blue noise" — the points look
 * randomly placed at a casual glance, but they're never clumped or
 * aligned. Bridson's trick is to grow the cloud from existing points
 * outward (each new point spawned in the annulus around an old one),
 * with a tiny lookup grid so distance checks are constant-time.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine the seeded point throwing darts at random angles, each
 * landing somewhere in a ring around it (between r and 2r away).
 * Each landed dart is checked against neighbours; if it's far enough
 * from all of them, it sticks and starts throwing its own darts. If
 * it can't find a clear spot after 30 throws, it stops trying.
 *
 * Three layers in the visible:
 *   1. RESTING POINTS 'o' (theme gradient, coloured by distance from the
 *      seed) — accepted samples that have exhausted their attempts.
 *   2. ACTIVE FRONTIER 'O' (theme active colour, brighter) — points
 *      still on the active list, each still throwing darts.
 *   3. FLASH '*' (theme flash colour, bold) — the bright pop of a
 *      fresh acceptance.
 *
 * The active frontier shrinks over time: as the cloud fills in,
 * points lose their elbow room and stop accepting candidates,
 * dropping from the list. Eventually the frontier is empty and the
 * algorithm halts.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Pick one seed point near the centre. Add to samples[],
 *     to the bg grid, and to the active queue. n_samples = 1,
 *     n_active = 1.
 *  2. STEP (one operation):
 *     a. If n_active == 0 → done.
 *     b. Pick a uniformly random index qi ∈ [0, n_active).
 *        Let si = active_queue[qi], P = samples[si].
 *     c. Decrement P.attempts_left.
 *     d. Generate candidate (cx, cy):
 *          θ = uniform [0, 2π)
 *          d = uniform [r, 2r]
 *          cx = P.x + d·cos(θ),  cy = P.y + d·sin(θ)
 *     e. If (cx, cy) out of bounds → reject silently, fall through
 *        to the exhaustion check.
 *     f. Bg-grid lookup: gx = ⌊cx / cell⌋, gy = ⌊cy / cell⌋.
 *        For each (dx, dy) ∈ [-2, 2]² check if a sample lives in
 *        the cell (gx+dx, gy+dy). If any such sample is within r
 *        of (cx, cy) → reject.
 *     g. If no rejection: ADD candidate. samples[n_samples++] =
 *        new sample with attempts_left = K. bg_grid[(gy)(gw)+gx] =
 *        new_idx. active_queue[n_active++] = new_idx.
 *        Paint glow flash.
 *     h. If P.attempts_left == 0 → swap-remove from active_queue:
 *        active_queue[qi] = active_queue[--n_active].
 *  3. Repeat 2 until done.
 *  4. HOLD on the cloud, then reset and goto 1 with a new seed.
 *
 * KEY FORMULAS
 * ────────────
 *  Background grid cell size    : c = r / √2  (so each cell holds at
 *                                  most one sample)
 *  Bg grid dimensions           : gw = ⌈W / c⌉,  gh = ⌈H / c⌉
 *  Random annulus point         : θ = 2π · rand(),
 *                                  d = r + r · rand(),
 *                                  candidate = P + d · (cos θ, sin θ)
 *  Distance check (squared)     : Σ (Δx² + Δy²) < r²  → reject
 *  Bg-cell lookup window        : (Δgx, Δgy) ∈ [-2, 2]²
 *  Saturation density (2-D)     : N ≈ Area · 2 / (π · r²)
 *                                  (≈ 950 points on 200×56, r=4)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • CELL-SIZE CHOICE. Bridson's r/√2 is the maximum size where a
 *    cell can hold at most one sample. A larger cell could hold two
 *    points within r of each other (and the bg grid would lose its
 *    "one cell, one sample" invariant). Don't be tempted to make
 *    cells r-sized — you'd need to handle multi-occupancy.
 *
 *  • LOOKUP WINDOW. With cell = r/√2, any sample > 2 cells away is
 *    > r√2/√2 = r away → can't conflict. So [-2, 2]² is the EXACT
 *    minimum window. Larger windows are correct but waste cycles;
 *    smaller windows miss conflicts at cell-corner positions.
 *
 *  • OUT-OF-BOUNDS CANDIDATES. Reject silently — don't try to clamp
 *    them back in; clamping creates a density bias near the borders.
 *    The K=30 attempt budget handles edge points correctly: they
 *    just exhaust faster because their annulus is partly outside.
 *
 *  • SEED CHOICE. Any point works — the resulting cloud is
 *    statistically the same regardless of seed location. Putting it
 *    near the centre just means the GROWTH is visible from the
 *    middle outward, which looks better than starting from a corner.
 *
 *  • K = 30 IS BRIDSON'S DEFAULT. Higher K means denser packing
 *    (closer to the theoretical maximum of N = Area / (π · r² / 4)),
 *    but with diminishing returns past 30. Lower K is faster but
 *    leaves visible gaps.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • At init: exactly one sample exists (the seed), n_active = 1.
 *    If the screen shows multiple points at t=0, the init is wrong.
 *  • Every visible pair of 'o'/'O' points is at LEAST r cells apart.
 *    Eyeball-test: pick any two points, count the distance — should
 *    be ≥ 4 cells with default settings.
 *  • The active frontier is always a "shell" around the cloud's
 *    growing edge. Interior 'o' points are never active. (Brief
 *    visual confirmation: pause once growth is well underway and
 *    look for 'O's clustered at the cloud boundary.)
 *  • Final density: N ≈ Area · 2 / (π · r²). For 200×56 = 11200
 *    cells with r=4: expected ≈ 11200 · 2 / (π · 16) ≈ 446. Actual
 *    count typically lands within ±15% (Bridson achieves ~70% of
 *    theoretical max). HUD shows the count for verification.
 *  • Different themes change colours; the cloud SHAPE is identical
 *    for the same seed. Algorithm is theme-independent.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * All data lives in the types in §1 (Sample, Poisson, Scene, Screen, App).
 * Every other section is functions for ONE concern:
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the clock / sleeps
 *   LOGIC        §3       NOTHING — pure reads (poisson_grid_idx,
 *                         poisson_candidate_valid): no mutation, no I/O
 *   SIMULATION   §4       Poisson (samples, bg grid, active queue, counts)
 *                         + Scene state / hold timer; the per-tick combine
 *   RENDER       §5       the terminal only — reads the Scene, never writes
 *   APP          §6       App (sim_fps, map size, flags) + Scene knobs; events
 *
 *   EFFECTS — the per-sample `glow` (the fresh-add '*' flash).  Cosmetic
 *     only: read by the renderer, never by a sim/logic decision.  Set to 1.0
 *     when a sample is accepted (poisson_step) and decayed each tick at the
 *     top of scene_tick.  (The old full-screen "supernova" flash was removed.)
 *   DELAYS — the HOLD phase: scene_tick counts hold_timer down from
 *     HOLD_SECONDS, then resets.  `paused` freezes the whole tick.  Both live
 *     inside scene_tick; no separate timer machinery.
 *
 * PER-TICK COMBINE — scene_tick (§4) is the ONE place state advances:
 *     1. EFFECTS    : decay every sample's glow by exp(-rate·dt)
 *     2. SIMULATION : GROWING → ops_per_tick × poisson_step; at active-empty
 *                     enter HOLD.  HOLD → count hold_timer down; at 0, reset.
 *   RENDER (scene_draw + HUD) then PERFORMANCE (frame cap) run every frame.
 *
 * USER EVENTS are NOT the tick: keys (app_handle_key) and resize
 * (app_do_resize) mutate App/Scene directly in §6 — reset, refit, theme /
 * speed / Hz changes — but none of them call scene_tick().
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config + types                                                     */
/* ===================================================================== */

enum {
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,

    /* Maximum samples and bg-grid cells. With r=4 on 200×56 we expect
     * ~450 samples and ~5500 grid cells. Sized generously so smaller
     * radii (denser clouds) also fit. */
    MAX_SAMPLES       = 8192,
    MAX_GRID          = 16384,

    /* Bridson's K = attempts per active point. 30 is the textbook
     * value; higher = denser packing with diminishing returns. */
    ATTEMPTS_PER_POINT = 30,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Attempts processed per scene_tick. Default 64 ⇒ ~3800
     * attempts/sec at 60 Hz, finishes a 200×56 r=4 cloud in ~4 s. */
    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =  64,
    OPS_PER_TICK_MAX  = 4096,

    RENDER_CAP_FPS    =  60,        /* hard cap on rendered frames/sec (sim ticks run at sim_fps) */
    MAX_FRAME_MS      = 100,        /* clamp one frame's dt — spiral-of-death guard after a stall */
    FPS_UPDATE_MS     = 500,

    /* Colour-pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md.
     * Resting points are drawn along a RAMP_LEN-step gradient
     * (PAIR_RAMP_0 .. PAIR_RAMP_0+RAMP_LEN-1); the active frontier and the
     * fresh-add flash get one accent pair each. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_ACTIVE       =   3,        /* 'O' active frontier samples */
    PAIR_FLASH        =   4,        /* '*' fresh-add flash         */
    PAIR_RAMP_0       =   5,        /* 'o' resting gradient base   */
};

#define RAMP_LEN      6            /* resting-point gradient steps (seed → edge) */
#define PAIR_RAMP(k)  (PAIR_RAMP_0 + (k))

/* Glow decay rate for the fresh-add '*' flash (~0.7 s). */
#define POINT_GLOW_DECAY    2.5f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

/* Exclusion radius in cell units. r=4 gives ~450 samples on 200×56
 * (sparse, easy to read individual points). Smaller r → denser cloud
 * but harder to distinguish individual points. */
#define POISSON_R           4.0f
#define SQRT2               1.41421356f   /* bg-grid cell = r / √2 (Bridson) */

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Theme — the cloud's colour scheme, cycled with t/T.
 *
 * WHY a `ramp` for resting points: a finished Poisson-disk cloud is otherwise
 * a flat field of identical dots.  Colouring each resting point by its
 * distance from the seed turns the cloud into expanding colour rings that
 * reveal the growth order (scalar→colour ramp, Bourke [5]).  `active`/`flash`
 * are bright accents drawn on top.  Every index sits in the bright half of the
 * 256-colour cube (palette rule); indices follow the xterm-256 chart [7].
 */
typedef struct {
    const char *name;             /* HUD label, e.g. "AURORA"                 */
    short       ramp[RAMP_LEN];   /* resting 'o' gradient, seed centre → edge */
    short       active;           /* 'O' active frontier (bright accent)      */
    short       flash;            /* '*' fresh-add pop (brightest)            */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*  name        ramp: seed ───────────────────► edge      active flash */
    { "AURORA", {  33,  39,  45,  51, 123, 195 },              226,  231 },
    { "MATRIX", {  28,  34,  40,  46,  82, 120 },              190,  231 },
    { "NOVA",   {  54,  92, 128, 164, 200, 219 },              213,  231 },
    { "MONO",   { 240, 244, 247, 250, 253, 255 },              231,  255 },
    { "OCEAN",  {  24,  31,  38,  45,  51, 123 },              159,  231 },
    { "FIRE",   {  88, 124, 160, 202, 214, 226 },              214,  231 },
    { "EARTH",  {  94, 130, 136, 179, 180, 230 },              222,  231 },
    { "FOREST", {  28,  64,  70, 106, 148, 190 },              226,  231 },
    { "DESERT", { 137, 143, 179, 185, 221, 230 },              223,  231 },
    { "ARCTIC", {  25,  31,  39,  45, 123, 195 },              159,  231 },
};

/*
 * Sample — one accepted point of the blue-noise set (Bridson [1]).
 *
 * WHY positions are CONTINUOUS (float), not grid cells: the algorithm reasons
 * in real coordinates and only rounds to a character cell at render time, so
 * the "≥ r apart" guarantee is exact, not quantised to the display grid.
 */
typedef struct {
    float x, y;          /* continuous position, in cell units                  */
    int   attempts_left; /* Bridson's K budget: counts K→0, then this point
                          * leaves the active queue (its neighbourhood is full) */
    float glow;          /* EFFECTS: fresh-add flash, 1.0 on accept → 0 (~0.7s) */
} Sample;

/*
 * Poisson — the sampler: the accepted point set plus the two structures
 * Bridson's algorithm needs to grow it fast (Bridson [1]).
 *
 * WHY the BACKGROUND GRID: the costly step is "is this candidate ≥ r from
 * every existing point?".  Sizing each cell r/√2 means a cell can hold at most
 * one sample, so that O(N) query collapses to scanning a fixed 5×5 cell box —
 * O(1).  bg_grid[cell] is the sample index living there, or -1.
 *
 * WHY the ACTIVE QUEUE: candidates only ever spawn near recently-added points,
 * so the algorithm keeps a list of points "still throwing darts"; a point that
 * misses K times in a row is swap-removed (O(1)).  When the queue empties the
 * cloud is complete.
 */
typedef struct {
    /* ── geometry + spatial-acceleration grid ── */
    int    w, h;                  /* map size in cell units                       */
    float  radius;                /* exclusion distance r between any two points  */
    float  cell_size;             /* bg-grid cell = r/√2 (⇒ ≤ 1 sample per cell)  */
    int    gw, gh;                /* bg-grid dimensions in cells                  */
    int    bg_grid[MAX_GRID];     /* per cell: the sample index there, or -1      */

    /* ── the accepted point set, in arrival order ── */
    Sample samples[MAX_SAMPLES];
    int    n_samples;

    /* ── Bridson active list: indices of points still spawning candidates ── */
    int    active_queue[MAX_SAMPLES];
    int    n_active;              /* 0 ⇒ the algorithm has finished               */
} Poisson;

/*
 * Scene state machine:
 *
 *   GROWING — call poisson_step up to ops_per_tick times. Transitions
 *             to HOLD when the active queue is empty.
 *   HOLD    — wait HOLD_SECONDS, then poisson_reset and back to GROWING.
 */
typedef enum {
    SCENE_GROWING = 0,
    SCENE_HOLD    = 1,
} SceneState;

/* Scene — the animated sampling run, as a table of contents:
 *   WHAT      : p — the Poisson sampler being grown.
 *   HOW       : ops_per_tick — Bridson attempts advanced per frame.
 *   run-state : state + hold_timer (the GROWING→HOLD→reset lifecycle), paused.
 *   RENDER    : current_theme — the active palette. */
typedef struct {
    Poisson     p;              /* WHAT: the sampler being grown            */
    SceneState  state;          /* run-state: GROWING / HOLD                */
    float       hold_timer;     /* run-state: HOLD-phase countdown, seconds */
    bool        paused;         /* run-state: freeze the tick               */
    int         ops_per_tick;   /* HOW: Bridson attempts per frame          */
    int         current_theme;  /* RENDER: index into themes[]              */
} Scene;

/* Screen — the terminal viewport (size in character cells).  The receiver for
 * terminal setup/resize and the renderers, so they take a Screen (not the
 * whole App) and stay decoupled from app-level state. */
typedef struct { int cols, rows; } Screen;

/* App — the whole program: the animated Scene plus the terminal and the
 * app-level loop state.
 *   subsystems : the sampling scene + the terminal viewport.
 *   tuning     : sim_fps (tick rate).   geometry : map_w/map_h, fitted to fit.
 *   lifecycle  : signal-driven quit / resize flags. */
typedef struct {
    /* subsystems */
    Scene                 scene;
    Screen                screen;
    /* tuning + derived geometry */
    int                   sim_fps;       /* simulation ticks per second    */
    int                   map_w, map_h;  /* cells, fitted to the terminal  */
    /* lifecycle flags (set by signal handlers) */
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

/* ===================================================================== */
/* §2  performance   (monotonic clock + frame-cap sleep)                  */
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
/* §3  logic   (pure decisions: read-only, no I/O — cannot be corrupted)  */
/* ===================================================================== */

/*
 * poisson_grid_idx — linear index into bg_grid[] for a continuous (x, y).
 * Caller is responsible for bounds checking.
 */
static inline int poisson_grid_idx(const Poisson *p, float x, float y)
{
    int gx = (int)(x / p->cell_size);
    int gy = (int)(y / p->cell_size);
    if (gx < 0) gx = 0;
    if (gy < 0) gy = 0;
    if (gx >= p->gw) gx = p->gw - 1;
    if (gy >= p->gh) gy = p->gh - 1;
    return gy * p->gw + gx;
}

/*
 * poisson_candidate_valid — for a candidate (cx, cy), is there any
 * existing sample within radius r? Reads from the bg grid.
 *
 * Returns true if the candidate is FAR ENOUGH from all existing
 * samples (i.e. valid for acceptance). Returns false if too close.
 *
 * The 5×5 cell window is the minimum that guarantees correctness;
 * see MENTAL MODEL "LOOKUP WINDOW" for the geometry.
 */
static bool poisson_candidate_valid(const Poisson *p, float cx, float cy)
{
    if (cx < 0 || cx >= (float)p->w) return false;
    if (cy < 0 || cy >= (float)p->h) return false;

    int gx = (int)(cx / p->cell_size);
    int gy = (int)(cy / p->cell_size);
    float r2 = p->radius * p->radius;

    for (int dy = -2; dy <= 2; dy++) {
        for (int dx = -2; dx <= 2; dx++) {
            int nx = gx + dx, ny = gy + dy;
            if (nx < 0 || nx >= p->gw || ny < 0 || ny >= p->gh) continue;
            int idx = p->bg_grid[ny * p->gw + nx];
            if (idx < 0) continue;
            float ex = p->samples[idx].x - cx;
            float ey = p->samples[idx].y - cy;
            if (ex * ex + ey * ey < r2) return false;
        }
    }
    return true;
}

/*
 * ramp_index — gradient tier (0..RAMP_LEN-1) for a resting point, by its
 * distance from the seed normalised to the map half-diagonal (`inv_maxd` =
 * 1/half-diagonal).  This is the radial-gradient mapping the renderer uses.
 */
static int ramp_index(const Sample *sm, const Sample *seed, float inv_maxd)
{
    float dx = sm->x - seed->x, dy = sm->y - seed->y;
    int k = (int)(sqrtf(dx * dx + dy * dy) * inv_maxd * RAMP_LEN);
    if (k < 0) k = 0;
    if (k >= RAMP_LEN) k = RAMP_LEN - 1;
    return k;
}

/* ===================================================================== */
/* §4  simulation   (advances the Poisson state + the per-tick combine)   */
/* ===================================================================== */

static inline float rand_unit(void) { return (float)rand() / (float)RAND_MAX; }

/* A uniformly random candidate in the annulus [r, 2r] around `seed` —
 * Bridson's spawn: uniform angle, uniform distance in [r, 2r]. */
static void annulus_candidate(const Sample *seed, float r, float *cx, float *cy)
{
    float a  = 2.0f * (float)M_PI * rand_unit();
    float dr = r + r * rand_unit();
    *cx = seed->x + cosf(a) * dr;
    *cy = seed->y + sinf(a) * dr;
}

/* Add an accepted candidate to the point set, the bg grid, and the active
 * queue (fresh K budget + a flash glow). */
static void poisson_accept(Poisson *p, float cx, float cy)
{
    if (p->n_samples >= MAX_SAMPLES) return;
    int new_idx = p->n_samples++;
    p->samples[new_idx] = (Sample){
        .x = cx, .y = cy,
        .attempts_left = ATTEMPTS_PER_POINT,
        .glow = 1.0f,
    };
    p->bg_grid[poisson_grid_idx(p, cx, cy)] = new_idx;
    p->active_queue[p->n_active++] = new_idx;
}

/*
 * poisson_step — one Bridson attempt: pick a random active point, throw a dart
 * into its annulus, accept it if it clears every neighbour, and retire the
 * point if its attempt budget is exhausted.
 *
 * Returns true if work was done; false when the active queue is empty
 * (algorithm finished).
 */
static bool poisson_step(Poisson *p)
{
    if (p->n_active <= 0)
        return false;

    int qi = rand() % p->n_active;
    Sample *seed = &p->samples[p->active_queue[qi]];
    seed->attempts_left--;

    float cx, cy;
    annulus_candidate(seed, p->radius, &cx, &cy);
    if (poisson_candidate_valid(p, cx, cy))
        poisson_accept(p, cx, cy);

    if (seed->attempts_left <= 0)              /* neighbourhood saturated → retire */
        p->active_queue[qi] = p->active_queue[--p->n_active];  /* O(1) swap-remove */
    return true;
}

/* Size the background grid (cell = r/√2, so ≤ 1 sample per cell) for a w×h
 * map and clear every cell to -1 (empty). */
static void poisson_init_grid(Poisson *p, int w, int h)
{
    p->w = w;
    p->h = h;
    p->radius = POISSON_R;
    p->cell_size = p->radius / SQRT2;
    p->gw = (int)ceilf((float)w / p->cell_size) + 1;
    p->gh = (int)ceilf((float)h / p->cell_size) + 1;
    if (p->gw * p->gh > MAX_GRID) {
        /* Defensive — should never happen with our caps. Cap at MAX_GRID. */
        p->gw = MAX_GRID / p->gh;
    }
    for (int i = 0; i < p->gw * p->gh; i++) p->bg_grid[i] = -1;
}

/* Drop the single seed point near the map centre (small jitter so runs don't
 * all start on the same pixel) and prime the sample set + active queue. */
static void poisson_seed_center(Poisson *p)
{
    float sx = (float)p->w / 2.0f + ((float)(rand() % 7) - 3.0f);
    float sy = (float)p->h / 2.0f + ((float)(rand() % 5) - 2.0f);
    if (sx < 1.0f)                 sx = 1.0f;
    if (sx >= (float)p->w - 1.0f)  sx = (float)p->w - 1.5f;
    if (sy < 1.0f)                 sy = 1.0f;
    if (sy >= (float)p->h - 1.0f)  sy = (float)p->h - 1.5f;

    p->samples[0] = (Sample){
        .x = sx, .y = sy,
        .attempts_left = ATTEMPTS_PER_POINT,
        .glow = 1.0f,
    };
    p->bg_grid[poisson_grid_idx(p, sx, sy)] = 0;
    p->n_samples = 1;
    p->active_queue[0] = 0;
    p->n_active = 1;
}

/*
 * poisson_reset — size a fresh grid, clear the counts, and drop one seed.
 */
static void poisson_reset(Poisson *p, int w, int h)
{
    poisson_init_grid(p, w, h);
    p->n_samples = 0;
    p->n_active  = 0;
    poisson_seed_center(p);     /* → n_samples = 1, n_active = 1 */
}

static void scene_reset(Scene *s, int mw, int mh)
{
    poisson_reset(&s->p, mw, mh);
    s->state      = SCENE_GROWING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->paused        = false;
    s->ops_per_tick  = OPS_PER_TICK_DEF;
    s->current_theme = 0;
    scene_reset(s, mw, mh);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    /* Decay each fresh-add flash individually (~0.7 s). */
    float decay_pt = expf(-POINT_GLOW_DECAY * dt);
    for (int i = 0; i < s->p.n_samples; i++)
        s->p.samples[i].glow *= decay_pt;

    switch (s->state) {

    case SCENE_GROWING:
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!poisson_step(&s->p)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->p.w, s->p.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §5  render   (state → screen: reads the Scene, mutates only the terminal) */
/* ===================================================================== */

/* Install one theme's palette: the resting-point ramp + the active/flash
 * accents (the HUD pairs are fixed, set in color_init). */
static void theme_apply(const Theme *t)
{
    if (COLORS >= 256) {
        for (int k = 0; k < RAMP_LEN; k++) init_pair(PAIR_RAMP(k), t->ramp[k], -1);
        init_pair(PAIR_ACTIVE, t->active, -1);
        init_pair(PAIR_FLASH,  t->flash,  -1);
    } else {
        for (int k = 0; k < RAMP_LEN; k++) init_pair(PAIR_RAMP(k), COLOR_BLUE, -1);
        init_pair(PAIR_ACTIVE, COLOR_CYAN,   -1);
        init_pair(PAIR_FLASH,  COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    init_pair(PAIR_HUD,  (COLORS >= 256) ? 226 : COLOR_YELLOW, -1);
    init_pair(PAIR_HINT, (COLORS >= 256) ?  51 : COLOR_CYAN,   -1);
    theme_apply(&themes[0]);
}

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

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Poisson *p = &s->p;

    int gx0 = (cols - p->w) / 2;
    int gy0 = ((rows - 3) - p->h) / 2 + 2;   /* rows 0+1 HUD, last row hint */
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    /* Resting points are coloured by their distance from the seed, so the
     * blue-noise cloud reads as a radial gradient expanding outward.  The
     * active frontier and the fresh-add flash render on top in accent colours. */
    const Sample *seed = &p->samples[0];
    float inv_maxd = 1.0f / (0.5f * sqrtf((float)(p->w * p->w + p->h * p->h)) + 1.0f);

    for (int i = 0; i < p->n_samples; i++) {
        const Sample *sm = &p->samples[i];
        int sx = gx0 + (int)(sm->x + 0.5f);
        int sy = gy0 + (int)(sm->y + 0.5f);
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;

        int  pair, attr;
        char glyph;

        if (sm->glow > GLOW_THRESHOLD) {            /* fresh acceptance pop */
            pair  = PAIR_FLASH;
            attr  = A_BOLD;
            glyph = '*';
        } else if (sm->attempts_left > 0) {         /* still on the frontier */
            pair  = PAIR_ACTIVE;
            attr  = A_BOLD;
            glyph = 'O';
        } else {                                    /* resting → gradient by radius */
            pair  = PAIR_RAMP(ramp_index(sm, seed, inv_maxd));
            attr  = A_NORMAL;
            glyph = 'o';
        }

        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

/* Draw one HUD row left-aligned at `row`, clipped to the terminal width. */
static void draw_hud_row(const Screen *sc, int row, int pair, int attr, const char *text)
{
    char buf[256];
    snprintf(buf, sizeof buf, "%s", text);
    if ((int)strlen(buf) > sc->cols) buf[sc->cols] = '\0';
    attron(COLOR_PAIR(pair) | attr);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | attr);
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Poisson *p = &s->p;
    const char *state_str =
        s->paused                     ? "PAUSED " :
        (s->state == SCENE_GROWING)   ? "GROWING" :
                                        "HOLD   ";

    char line[256];

    /* top row 0: data — title, theme, state, live counts, frame/sim rates */
    snprintf(line, sizeof line,
             " Poisson Disk  %s  %s  pts:%d  active:%d  %.1f fps  %d Hz ",
             themes[s->current_theme].name, state_str,
             p->n_samples, p->n_active, fps, sim_fps);
    draw_hud_row(sc, 0, PAIR_HUD, A_BOLD, line);

    /* top row 1: algorithm parameters */
    snprintf(line, sizeof line,
             " r:%.1f  K:%d  ops/tick:%d  map:%dx%d ",
             p->radius, ATTEMPTS_PER_POINT, s->ops_per_tick, p->w, p->h);
    draw_hud_row(sc, 1, PAIR_HUD, A_NORMAL, line);

    /* bottom row: actions — every interactive key */
    draw_hud_row(sc, sc->rows - 1, PAIR_HINT, A_BOLD,
                 " q:quit  spc:pause  r:reset  t:theme  +/-:speed  [/]:Hz ");
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §6  app   (signals, user events, and the main loop)                    */
/* ===================================================================== */

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - 3;
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

static bool app_handle_key(App *app, int ch)
{
    Scene *s = &app->scene;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     s->paused = !s->paused; break;
    case 'r': case 'R':
        scene_reset(s, app->map_w, app->map_h);
        break;
    case '=': case '+':
        if (s->ops_per_tick < OPS_PER_TICK_MAX) s->ops_per_tick *= 2;
        if (s->ops_per_tick > OPS_PER_TICK_MAX) s->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        s->ops_per_tick /= 2;
        if (s->ops_per_tick < OPS_PER_TICK_MIN) s->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        app->sim_fps += SIM_FPS_STEP;
        if (app->sim_fps > SIM_FPS_MAX) app->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        app->sim_fps -= SIM_FPS_STEP;
        if (app->sim_fps < SIM_FPS_MIN) app->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        s->current_theme = (s->current_theme + 1) % N_THEMES;
        theme_apply(&themes[s->current_theme]);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(&themes[s->current_theme]);
        break;

    default: break;
    }
    return true;
}

static void install_signals(void)
{
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* One-time setup: flags + tick rate, bring up the terminal, fit the map to it,
 * and build the first sampler. */
static void app_init(App *app)
{
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;
    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    install_signals();

    App *app = &g_app;
    app_init(app);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* USER EVENT: resize (handled outside the tick) */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        /* PER-TICK COMBINE: fixed-timestep accumulator drives scene_tick */
        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* PERFORMANCE: frame cap to 60 fps */
        /* PERFORMANCE: cap rendered frames to RENDER_CAP_FPS */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        /* RENDER (reads state only) */
        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        /* USER EVENT: keys */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
