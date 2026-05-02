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
 *       distributed but with no two points closer than r. HOLD;
 *       supernova reset; loop forever.
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
 *   §1 config   — map size, radius, attempts, themes, glow rates
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 cave themes
 *   §5 poisson  — Sample, Poisson, background grid, step
 *   §6 scene    — GROWING / HOLD state machine
 *   §7 screen   — ASCII render: o, O, * point glyphs
 *   §8 app      — signals, resize, main loop
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
 * References     : • Bridson, R. (2007) — "Fast Poisson Disk Sampling
 *                    in Arbitrary Dimensions". The original paper:
 *                    https://www.cs.ubc.ca/~rbridson/docs/bridson-siggraph07-poissondisk.pdf
 *                  • Wikipedia — "Poisson distribution sampling
 *                    (Poisson disk)":
 *                    https://en.wikipedia.org/wiki/Supersampling#Poisson_disc
 *                  • Inigo Quilez — "Voronoi distances on a regular
 *                    grid" (related blue-noise techniques):
 *                    https://iquilezles.org/articles/voronoilines/
 *                  • Mike Bostock — "Visualizing Algorithms"
 *                    (animated Poisson disk explainer):
 *                    https://bost.ocks.org/mike/algorithms/
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
 *   1. RESTING POINTS 'o' (theme point colour) — accepted samples
 *      that have already exhausted their attempts.
 *   2. ACTIVE FRONTIER 'O' (theme active colour, brighter) — points
 *      still on the active list, each still throwing darts.
 *   3. FLASH '*' (theme flash colour, gold-bold) — the bright pop of
 *      a fresh acceptance.
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
 *  4. HOLD on the cloud, supernova reset, goto 1 with new seed.
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
/* §1  config                                                             */
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

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_BG           =   3,        /* dim background hint (unused glyph) */
    PAIR_POINT        =   4,        /* 'o' resting samples                */
    PAIR_ACTIVE       =   5,        /* 'O' active frontier samples        */
    PAIR_FLASH        =   6,        /* '*' fresh acceptance flash         */
    PAIR_FLASHHUD     =   7,        /* HUD glow accent                    */
    PAIR_SUPERNOVA    =   8,        /* yellow reset flash                 */
};

/* Glow decay rates. */
#define POINT_GLOW_DECAY    2.5f    /* fresh-add flash duration ~0.7 s */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

/* Exclusion radius in cell units. r=4 gives ~450 samples on 200×56
 * (sparse, easy to read individual points). Smaller r → denser cloud
 * but harder to distinguish individual points. */
#define POISSON_R           4.0f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — same 10 names as the other procedural showcases. Each
 * theme defines four colours: bg/point/active/flash. PAIR_BG is
 * reserved but currently unused (no background pattern is drawn).
 */
typedef struct {
    const char *name;
    short       bg, point, active, flash;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      bg point active flash */
    { "DEFAULT",  240,   67,  117,  220 },   /* grey / blue / cyan / gold     */
    { "MATRIX",    22,   34,  118,   46 },   /* greens                        */
    { "NOVA",      53,  129,  213,  219 },   /* purples                       */
    { "MONO",     234,  244,  250,  254 },   /* greyscale                     */
    { "OCEAN",     17,   33,   51,   39 },   /* navy / cyan                   */
    { "FIRE",      52,  124,  208,  226 },   /* dark red / orange / yellow    */
    { "EARTH",     58,  137,  173,  230 },   /* brown / cream                 */
    { "FOREST",    22,   64,   82,  144 },   /* greens to tan                 */
    { "DESERT",    94,  222,  178,  230 },   /* sandy                         */
    { "ARCTIC",    18,   39,  159,  231 },   /* navy / ice / white            */
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
        init_pair(PAIR_BG,     t->bg,     -1);
        init_pair(PAIR_POINT,  t->point,  -1);
        init_pair(PAIR_ACTIVE, t->active, -1);
        init_pair(PAIR_FLASH,  t->flash,  -1);
    } else {
        init_pair(PAIR_BG,     COLOR_WHITE,   -1);
        init_pair(PAIR_POINT,  COLOR_BLUE,    -1);
        init_pair(PAIR_ACTIVE, COLOR_CYAN,    -1);
        init_pair(PAIR_FLASH,  COLOR_YELLOW,  -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_FLASHHUD,   220, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_FLASHHUD,  COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  poisson                                                            */
/* ===================================================================== */

/*
 * Sample — one accepted point.
 *
 *   x, y         : continuous position in cell units
 *   attempts_left: K → 0; when 0 the sample falls off the active queue
 *   glow         : flash-on-add, decays over ~0.7 s
 */
typedef struct {
    float x, y;
    int   attempts_left;
    float glow;
} Sample;

/*
 * Poisson — the simulation heart.
 *
 *   w, h         : map dims in cell units
 *   radius       : minimum distance r between any two samples
 *   cell_size    : background grid cell size = r / √2
 *   gw, gh       : background grid dims in cells
 *   bg_grid[]    : per bg cell, the sample index living there or -1
 *
 *   samples[]    : every accepted sample, in arrival order
 *   n_samples    : sample count
 *
 *   active_queue[]: indices into samples[] of points still attempting
 *   n_active     : queue length
 *
 *   attempts_total / attempts_succ : HUD stats
 *   done         : true once the active queue empties
 *   supernova_glow_t : a single-float global supernova fade
 */
typedef struct {
    int    w, h;
    float  radius;
    float  cell_size;
    int    gw, gh;
    int    bg_grid[MAX_GRID];

    Sample samples[MAX_SAMPLES];
    int    n_samples;

    int    active_queue[MAX_SAMPLES];
    int    n_active;

    int    attempts_total;
    int    attempts_succ;
    bool   done;

    float  supernova_glow_t;
} Poisson;

static inline float rand_unit(void) { return (float)rand() / (float)RAND_MAX; }

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
 * poisson_step — one Bridson attempt. Picks a random active point,
 * generates one annulus candidate, accepts it if valid. Removes the
 * picked point from the active queue if its attempt budget is exhausted.
 *
 * Returns true if work was done; false when the active queue is empty
 * (algorithm finished).
 */
static bool poisson_step(Poisson *p)
{
    if (p->n_active <= 0) {
        p->done = true;
        return false;
    }

    p->attempts_total++;

    int qi = rand() % p->n_active;
    int si = p->active_queue[qi];
    Sample *seed = &p->samples[si];
    seed->attempts_left--;

    /* Generate candidate in annulus [r, 2r] around the seed. */
    float a  = 2.0f * (float)M_PI * rand_unit();
    float dr = p->radius + p->radius * rand_unit();
    float cx = seed->x + cosf(a) * dr;
    float cy = seed->y + sinf(a) * dr;

    if (poisson_candidate_valid(p, cx, cy)) {
        if (p->n_samples < MAX_SAMPLES) {
            int new_idx = p->n_samples++;
            p->samples[new_idx] = (Sample){
                .x = cx, .y = cy,
                .attempts_left = ATTEMPTS_PER_POINT,
                .glow = 1.0f,
            };
            p->bg_grid[poisson_grid_idx(p, cx, cy)] = new_idx;
            p->active_queue[p->n_active++] = new_idx;
            p->attempts_succ++;
        }
    }

    if (seed->attempts_left <= 0) {
        /* Swap-remove from active queue — O(1). */
        p->active_queue[qi] = p->active_queue[--p->n_active];
    }
    return true;
}

/*
 * poisson_reset — clear everything, drop one seed point near the
 * centre, paint a global supernova fade.
 */
static void poisson_reset(Poisson *p, int w, int h)
{
    p->w = w;
    p->h = h;
    p->radius = POISSON_R;
    p->cell_size = p->radius / 1.41421356f;
    p->gw = (int)ceilf((float)w / p->cell_size) + 1;
    p->gh = (int)ceilf((float)h / p->cell_size) + 1;
    if (p->gw * p->gh > MAX_GRID) {
        /* Defensive — should never happen with our caps. Cap at MAX_GRID. */
        p->gw = MAX_GRID / p->gh;
    }
    for (int i = 0; i < p->gw * p->gh; i++) p->bg_grid[i] = -1;

    p->n_samples = 0;
    p->n_active = 0;
    p->attempts_total = 0;
    p->attempts_succ  = 0;
    p->done = false;
    p->supernova_glow_t = 1.0f;

    /* Seed with one random point near the centre — small jitter so
     * runs don't all start on the same exact pixel. */
    float sx = (float)w / 2.0f + ((float)(rand() % 7) - 3.0f);
    float sy = (float)h / 2.0f + ((float)(rand() % 5) - 2.0f);
    if (sx < 1.0f)              sx = 1.0f;
    if (sx >= (float)w - 1.0f)  sx = (float)w - 1.5f;
    if (sy < 1.0f)              sy = 1.0f;
    if (sy >= (float)h - 1.0f)  sy = (float)h - 1.5f;

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

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

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

typedef struct {
    Poisson     p;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         ops_per_tick;
    int         current_theme;
} Scene;

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

    /* Decay glows. The supernova is a single global value; per-sample
     * glows decay individually. */
    float decay_pt = expf(-POINT_GLOW_DECAY * dt);
    float decay_nv = expf(-SUPERNOVA_DECAY  * dt);
    for (int i = 0; i < s->p.n_samples; i++) {
        s->p.samples[i].glow *= decay_pt;
    }
    s->p.supernova_glow_t *= decay_nv;

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
/* §7  screen                                                             */
/* ===================================================================== */

typedef struct { int cols, rows; } Screen;

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
    int gy0 = ((rows - 3) - p->h) / 2 + 2;   /* row 0+1 HUD, last row hint */
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    /* Supernova flash — paint ONLY when active so we don't iterate
     * the full screen most frames. */
    if (p->supernova_glow_t > GLOW_THRESHOLD) {
        attron(COLOR_PAIR(PAIR_SUPERNOVA) | A_BOLD);
        for (int y = 0; y < p->h; y++) {
            int sy = gy0 + y;
            if (sy < 0 || sy >= rows) continue;
            for (int x = 0; x < p->w; x++) {
                int sx = gx0 + x;
                if (sx < 0 || sx >= cols) continue;
                /* Sparse pattern so it looks like a flash, not solid. */
                if (((x ^ y) & 3) == 0) mvaddch(sy, sx, '*');
            }
        }
        attroff(COLOR_PAIR(PAIR_SUPERNOVA) | A_BOLD);
    }

    /* Render samples. Each sample at (x, y) rounds to a cell; with
     * radius ≥ 4 cells, no two samples ever round to the same cell. */
    for (int i = 0; i < p->n_samples; i++) {
        const Sample *sm = &p->samples[i];
        int sx = gx0 + (int)(sm->x + 0.5f);
        int sy = gy0 + (int)(sm->y + 0.5f);
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;

        int  pair, attr;
        char glyph;

        if (sm->glow > GLOW_THRESHOLD) {
            pair  = PAIR_FLASH;
            attr  = A_BOLD;
            glyph = '*';
        } else if (sm->attempts_left > 0) {
            pair  = PAIR_ACTIVE;
            attr  = A_BOLD;
            glyph = 'O';
        } else {
            pair  = PAIR_POINT;
            attr  = A_NORMAL;
            glyph = 'o';
        }

        attron(COLOR_PAIR(pair) | attr);
        mvaddch(sy, sx, (chtype)(unsigned char)glyph);
        attroff(COLOR_PAIR(pair) | attr);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Poisson *p = &s->p;
    const char *state_str =
        s->paused                     ? "PAUSED " :
        (s->state == SCENE_GROWING)   ? "GROWING" :
                                        "HOLD   ";

    /* Acceptance rate — useful debugging stat showing how saturated
     * the cloud is (drops from ~70% early to <5% near the end). */
    int accept_pct = (p->attempts_total > 0)
                   ? (100 * p->attempts_succ / p->attempts_total)
                   : 0;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops:%-3d  %s  pts:%-4d  active:%-3d ",
             fps, sim_fps, s->ops_per_tick, state_str,
             p->n_samples, p->n_active);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " POISSON DISK SAMPLING ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — theme + algorithm parameters. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             " r:%.1f  K:%d  attempts:%d  accept:%d%%  map:%dx%d ",
             p->radius, ATTEMPTS_PER_POINT,
             p->attempts_total, accept_pct, p->w, p->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " o:done  O:active  *:flash | t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
    attroff(COLOR_PAIR(PAIR_HINT) | A_BOLD);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

typedef struct {
    Scene                 scene;
    Screen                screen;
    int                   sim_fps;
    int                   map_w, map_h;
    volatile sig_atomic_t running;
    volatile sig_atomic_t need_resize;
} App;

static App g_app;

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
        theme_apply(s->current_theme);
        break;
    case 'T':
        s->current_theme = (s->current_theme + N_THEMES - 1) % N_THEMES;
        theme_apply(s->current_theme);
        break;

    default: break;
    }
    return true;
}

int main(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);

    App *app     = &g_app;
    app->running = 1;
    app->sim_fps = SIM_FPS_DEFAULT;

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

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

        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > 100 * NS_PER_MS) dt = 100 * NS_PER_MS;

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

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

        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(NS_PER_SEC / 60 - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
