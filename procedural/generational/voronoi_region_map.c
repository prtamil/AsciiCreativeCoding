/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * voronoi_region_map.c — Voronoi region map, animated.
 *
 * DEMO: Eight seed points drop onto the map one at a time. Then for
 *       every cell on screen, the algorithm computes which seed is
 *       closest (Euclidean distance) and paints the cell in that
 *       seed's colour. Cells reveal in DISTANCE ORDER — closest
 *       cells first — so you watch coloured waves grow outward from
 *       every seed simultaneously, meeting at the boundaries between
 *       regions. Each region settles into a solid block of '#'
 *       characters in its theme colour. HOLD on the diagram;
 *       supernova reset; new seeds; loop forever.
 *
 * Study alongside: ./poission_disk_sampling_showcase.c — both algo's
 *       work with seed POINTS but they answer opposite questions.
 *       Poisson asks "how many points fit, given a minimum spacing?"
 *       Voronoi asks "given some points, who owns each cell?". The
 *       two compose well: take a Poisson sample, then run Voronoi —
 *       you get evenly-spaced regions with no clumps.
 *
 * Section map:
 *   §1 config   — map size, seed count, palette, themes, glow rates
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 themes (8 region colours each)
 *   §5 voronoi  — Seed, Voronoi, compute_distances, reveal_step
 *   §6 scene    — PLACING / REVEALING / HOLD state machine
 *   §7 screen   — ASCII render: '#' regions, '@' seeds, '*' flash
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (preserves theme)
 *   t / T      next / previous theme
 *   + / =      faster reveal (more cells/tick)
 *   -          slower reveal
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra voronoi_region_map.c \
 *       -o voronoi -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Voronoi diagram via direct nearest-seed search. Given
 *                  N seed points {S_0, …, S_{N-1}} in 2-D space, the
 *                  Voronoi region R_i for seed i is the set of all
 *                  points whose CLOSEST seed (under some distance
 *                  metric) is S_i. Boundaries between regions are loci
 *                  where two seeds are equally distant.
 *
 *                  We compute it by brute force — for every cell,
 *                  iterate every seed and pick the closest. O(W·H·N).
 *                  For our showcase (200×56 cells, 8 seeds) that's
 *                  90 K distance comparisons — trivial. Faster
 *                  algorithms exist (Fortune's sweep, jump-flood, etc)
 *                  but the brute-force version is the simplest one to
 *                  understand and animates the same way.
 *
 *                  The animated reveal sorts cells by distance to their
 *                  owner seed, then unveils them in that order. Closer
 *                  cells appear first; the result is visible "waves"
 *                  growing from every seed simultaneously, meeting at
 *                  the region boundaries.
 *
 *                  Distance metric: EUCLIDEAN (squared, no sqrt
 *                  needed for comparisons). Manhattan and Chebyshev
 *                  distances also yield Voronoi-like diagrams with
 *                  diamond and square regions respectively.
 *
 * Data-structure : One owner[] array (uint8_t per cell, seed index),
 *                  one dist2[] array (int squared distance), one
 *                  cell_order[] array of (idx, dist2) pairs sorted by
 *                  distance for the reveal sweep. Plus per-cell glow
 *                  floats. No allocation post-init.
 *
 * Rendering      : ASCII only. '#' for revealed region cells (in the
 *                  region's theme colour), '@' for seed positions
 *                  (in the seed's region colour, BOLD), '*' for the
 *                  fresh-reveal wave_glow flash and supernova reset.
 *                  Unrevealed cells stay blank — that's how the
 *                  reveal animation reads as "growing".
 *
 * Performance    : O(W·H·N) once at PLACING→REVEALING transition,
 *                  plus O(W·H · log(W·H)) for the qsort. 11K cells
 *                  with 8 seeds = ~250 K ops total. Reveal sweep is
 *                  O(W·H) total, throttled by ops_per_tick so it
 *                  unfolds over ~3 s.
 *
 * References     : • Wikipedia — "Voronoi diagram":
 *                    https://en.wikipedia.org/wiki/Voronoi_diagram
 *                  • Inigo Quilez — "Voronoi distances":
 *                    https://iquilezles.org/articles/voronoilines/
 *                  • Red Blob Games — "Hexagonal grids" (mentions
 *                    Voronoi for irregular-cell maps):
 *                    https://www.redblobgames.com/grids/hexagons/
 *                  • Fortune, S. (1987) — "A sweepline algorithm for
 *                    Voronoi diagrams" (the asymptotically faster
 *                    O(N log N) algorithm). Not used here; brute
 *                    force is faster to read.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Take a few scattered points. Ask "who owns this cell?" and answer
 * "whichever seed is closest". That's a Voronoi diagram. The
 * boundaries between regions are perpendicular bisectors of the
 * lines between adjacent seeds; the regions are convex polygons.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Imagine each seed is a stone dropped in still water; ripples spread
 * out at the same speed. Wherever two ripples meet, you have a
 * boundary between two domains. The earliest-arrived ripple owns the
 * cell. After the ripples have covered everywhere, you have a Voronoi
 * map of regions. Our reveal animation literally renders this:
 * cells closer to their seed (= earliest-arrived ripple) appear first;
 * far cells appear later; boundaries are where waves from different
 * seeds approach the same cell from opposite sides at similar times.
 *
 * Visible layers:
 *   1. 8 seed points '@' (bright, in the region colour) — the static
 *      anchors, dropped in PLACING phase.
 *   2. '#' region cells in region colour — the resolved territory,
 *      revealing in distance order.
 *   3. '*' wave-glow flash on cells that reveal THIS frame — fades
 *      over ~0.4 s.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Drop N random seeds (with min-spacing rejection so they
 *     don't pile up). Each seed gets a distinct theme colour from
 *     its slot in themes[t].region[].
 *  2. COMPUTE. For each cell (x, y) and each seed i:
 *     d²_i = (x − seed_i.x)² + (y − seed_i.y)²
 *     owner[cell] = argmin_i d²_i
 *     dist2[cell] = min_i d²_i
 *  3. SORT cell_order[] by dist2 ascending. Tied distances are fine
 *     — the reveal animation accepts any tie-break.
 *  4. REVEAL. cells appear in sorted order. Per scene_tick advance
 *     reveal_progress by ops_per_tick. Each freshly-revealed cell
 *     gets wave_glow = 1.0 (a brief gold flash before settling to
 *     the region colour).
 *  5. When reveal_progress reaches W·H → HOLD for HOLD_SECONDS.
 *  6. Reset, supernova flash, goto 1.
 *
 * KEY FORMULAS
 * ────────────
 *  Squared Euclidean distance    : d² = Δx² + Δy²
 *                                  (avoid sqrt — only used for
 *                                   COMPARISON, which preserves order)
 *  Owner of cell                 : argmin_i d²(cell, seed_i)
 *  Min seed-seed spacing         : require dist(seed_a, seed_b) ≥
 *                                  MIN_SEED_DIST so the diagram is
 *                                  visually balanced
 *  Reveal order                  : cells sorted ascending by dist2
 *  Region count                  : N_SEEDS (= number of seeds)
 *  Visible boundary              : where adjacent cells have
 *                                  different owners (we don't draw
 *                                  the boundary explicitly — it's
 *                                  implicit in the colour change)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • SQUARED DISTANCE for comparison. Use d² = Δx² + Δy² instead of
 *    sqrt(Δx² + Δy²) — sqrt is monotone so order is preserved, and
 *    you avoid a libm call per cell. Both ints and floats work.
 *
 *  • TIES. When two seeds are exactly equidistant from a cell, the
 *    algorithm picks WHICHEVER COMES FIRST in the seeds[] array
 *    (stable argmin). This is deterministic but NOT centred — for a
 *    centred tie-break, replace argmin with "min, tie → average index".
 *    For our showcase, the asymmetry is invisible.
 *
 *  • TERMINAL ASPECT. Cells are ~2× taller than wide in pixels. We
 *    use raw cell distances; the resulting regions are slightly
 *    squashed vertically. To get visually-correct circular regions,
 *    multiply (Δy)² by ~4 (because each row is ~2 columns tall in
 *    pixels and we need the SQUARE of that ratio). We don't bother
 *    here; the showcase is recognisably Voronoi either way.
 *
 *  • SEED MIN SPACING. Without min-spacing, two seeds can appear
 *    nearly on top of each other, producing a tiny pinched region
 *    between them. Set MIN_SEED_DIST to something like W/(2N) for a
 *    well-balanced diagram. Use rejection sampling on placement.
 *
 *  • REVEAL ORDER. Sorting by raw dist2 (ints) is stable and fast.
 *    qsort works fine; for ~10 K elements it's ~150 µs. If you need
 *    O(N) reveal-order, use radix-sort or bucketed reveal — but
 *    quicksort is simpler.
 *
 *  • COLOUR CYCLING. Each seed gets the next colour from
 *    themes[t].region[]. With N_SEEDS > 8 you'd start cycling — two
 *    seeds share a colour. We default to N_SEEDS = 8 to match the
 *    palette size exactly.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Initial state: 8 seed positions visible as '@' glyphs.
 *  • After REVEALING phase completes, every cell has an owner. Total
 *    revealed cell count == W · H.
 *  • Each region is contiguous (all cells of one colour form one
 *    connected blob with the seed at its rough centre). If you see
 *    a region split into two disconnected pieces, the distance
 *    metric is wrong (e.g. Manhattan being computed as Chebyshev).
 *  • Region boundaries are roughly straight lines (perpendicular
 *    bisectors of seed pairs). Curved boundaries indicate a bug in
 *    the distance computation.
 *  • Each region's seed sits inside its own region (an obvious
 *    correctness check — if a seed's '@' falls in a different
 *    colour, owner[] computation is broken).
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
    MAP_W_MAX         = 200,
    MAP_H_MAX         =  56,
    CELLS_MAX         = MAP_W_MAX * MAP_H_MAX,

    /* Number of seeds = number of regions = palette size. 8 is the
     * sweet spot — enough variety to look like a real cellular map,
     * not so many that adjacent regions blend visually. */
    N_SEEDS           =   8,

    /* Seed-placement rejection: a candidate seed is accepted only if
     * it's at least this many cells away from every previously-placed
     * seed. Prevents lopsided diagrams with two seeds nearly on top. */
    MIN_SEED_DIST     =  10,
    SEED_PLACE_TRIES  = 100,

    /* Number of seeds to drop per scene_tick during PLACING phase.
     * 1 per tick at 60 Hz means each seed appears for ~17 ms before
     * the next one — fast but the drop-in is still visible. */
    PLACE_PER_TICK    =   1,

    /* Slow-down: hold 0.3 s between successive seed drops. */
    PLACE_HOLD_TICKS  =  18,        /* 18/60 = 0.3 s @ 60 Hz */

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Cells revealed per scene_tick during REVEALING phase. */
    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  =  64,
    OPS_PER_TICK_MAX  = 4096,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md.
     * 8 region pairs, plus seed/flash/supernova accents. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_REGION_BASE  =   3,        /* PAIR_REGION_BASE..+7 = 8 region pairs */
    PAIR_SEED         =  11,        /* '@' seed glyph                */
    PAIR_FLASH        =  12,        /* '*' wave-reveal flash         */
    PAIR_SUPERNOVA    =  13,        /* yellow reset flash            */
};

/* Glow decay rates. */
#define WAVE_GLOW_DECAY     2.5f    /* fresh-reveal flash duration ~0.7 s */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — 10 named palettes. Each defines 8 region colours (one per
 * seed) plus seed-glyph and flash colours. The 10 names are shared
 * with the other procedural showcases for consistency.
 */
typedef struct {
    const char *name;
    short       region[N_SEEDS];    /* 8 region colours              */
    short       seed_fg;            /* '@' glyph colour              */
    short       flash_fg;           /* '*' wave-reveal colour        */
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /* DEFAULT: rainbow mix that maximises adjacent-region distinction */
    { "DEFAULT", {  33,  67, 132, 165, 220,  34, 178, 244 }, 231, 226 },
    /* MATRIX: shades of green */
    { "MATRIX",  {  22,  28,  34,  40,  46,  82, 118, 154 }, 231, 226 },
    /* NOVA: purple to magenta to pink */
    { "NOVA",    {  53,  92, 129, 165, 201, 213, 219, 225 }, 231, 226 },
    /* MONO: greyscale gradient */
    { "MONO",    { 236, 240, 244, 247, 250, 252, 254, 255 }, 226, 226 },
    /* OCEAN: navy to bright cyan */
    { "OCEAN",   {  17,  18,  20,  27,  33,  39,  51, 117 }, 231, 226 },
    /* FIRE: dark red to yellow */
    { "FIRE",    {  52,  88, 124, 160, 196, 208, 220, 226 }, 231, 196 },
    /* EARTH: dark brown to cream */
    { "EARTH",   {  58,  94, 100, 137, 173, 215, 222, 229 }, 231, 226 },
    /* FOREST: dark green to tan */
    { "FOREST",  {  22,  28,  64, 100, 130, 144, 178, 187 }, 231, 226 },
    /* DESERT: brown to sand */
    { "DESERT",  {  94, 130, 137, 173, 215, 222, 229, 230 }, 231, 226 },
    /* ARCTIC: navy to white */
    { "ARCTIC",  {  17,  18,  24,  39,  51, 159, 195, 231 }, 226, 226 },
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

/*
 * theme_apply — install one of the 10 named palettes. Re-initialises
 * the 8 region pairs, the seed pair, and the flash pair. HUD/HINT/
 * SUPERNOVA pairs stay theme-independent.
 */
static void theme_apply(int idx)
{
    if (idx < 0 || idx >= N_THEMES) idx = 0;
    if (COLORS >= 256) {
        const Theme *t = &themes[idx];
        for (int i = 0; i < N_SEEDS; i++)
            init_pair(PAIR_REGION_BASE + i, t->region[i], -1);
        init_pair(PAIR_SEED,  t->seed_fg,  -1);
        init_pair(PAIR_FLASH, t->flash_fg, -1);
    } else {
        /* 8-colour fallback. */
        static const short fallback[N_SEEDS] = {
            COLOR_BLUE,  COLOR_CYAN,    COLOR_GREEN,  COLOR_YELLOW,
            COLOR_MAGENTA, COLOR_RED,   COLOR_WHITE,  COLOR_BLUE,
        };
        for (int i = 0; i < N_SEEDS; i++)
            init_pair(PAIR_REGION_BASE + i, fallback[i], -1);
        init_pair(PAIR_SEED,  COLOR_WHITE,  -1);
        init_pair(PAIR_FLASH, COLOR_YELLOW, -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  voronoi                                                            */
/* ===================================================================== */

/*
 * Seed — one anchor point for a Voronoi region.
 *
 *   x, y  : integer cell position (we don't bother with sub-cell
 *           precision — the showcase doesn't need it).
 */
typedef struct {
    int x, y;
} Seed;

/*
 * CellOrd — used for the reveal-order sort. Pairs a cell index with
 * its squared distance to its owner seed; qsort orders by dist2.
 */
typedef struct {
    int idx;
    int dist2;
} CellOrd;

/*
 * Voronoi — the simulation heart.
 *
 *   owner[]      : per-cell seed index 0..N_SEEDS-1 (after compute)
 *   dist2[]      : per-cell squared distance to owner seed
 *   wave_glow[]  : per-cell flash on first reveal
 *   supernova_glow[] : per-cell reset flash
 *   revealed[]   : per-cell bool — true once the reveal sweep has
 *                  visited this cell
 *
 *   seeds[]      : seed positions (filled during PLACING)
 *   n_seeds      : seed count, grows from 0 to N_SEEDS during PLACING
 *
 *   cell_order[] : sorted (cell idx, dist2) by ascending dist2 —
 *                  drives the REVEALING sweep
 *   reveal_progress: index into cell_order during REVEAL
 *
 *   computed     : true once owner[] / dist2[] / cell_order[] filled
 *   place_cooldown: ticks until the next seed drops during PLACING
 */
typedef struct {
    int     w, h;
    int     total_cells;
    int8_t  owner[CELLS_MAX];
    int     dist2[CELLS_MAX];
    float   wave_glow    [CELLS_MAX];
    float   supernova_glow[CELLS_MAX];
    bool    revealed[CELLS_MAX];

    Seed    seeds[N_SEEDS];
    int     n_seeds;

    CellOrd cell_order[CELLS_MAX];
    int     reveal_progress;

    bool    computed;
    int     place_cooldown;
} Voronoi;

static inline int v_idx(const Voronoi *v, int x, int y) { return y * v->w + x; }
static inline bool v_in_bounds(const Voronoi *v, int x, int y)
{
    return x >= 0 && x < v->w && y >= 0 && y < v->h;
}

/*
 * v_place_one_seed — pick a random position with min-distance rejection
 * against existing seeds. Returns true if a seed was placed; false if
 * we couldn't find a valid spot in SEED_PLACE_TRIES attempts (in which
 * case we accept the last candidate even if too close — degrades
 * gracefully on small maps).
 */
static bool v_place_one_seed(Voronoi *v)
{
    if (v->n_seeds >= N_SEEDS) return false;

    int best_x = 0, best_y = 0;
    bool good = false;
    for (int attempt = 0; attempt < SEED_PLACE_TRIES; attempt++) {
        int x = rand() % v->w;
        int y = rand() % v->h;
        bool ok = true;
        for (int i = 0; i < v->n_seeds; i++) {
            int dx = x - v->seeds[i].x;
            int dy = y - v->seeds[i].y;
            if (dx * dx + dy * dy < MIN_SEED_DIST * MIN_SEED_DIST) {
                ok = false;
                break;
            }
        }
        best_x = x; best_y = y;
        if (ok) { good = true; break; }
    }
    (void)good;     /* If we never found a fully-good spot, the last
                     * candidate is used — at worst slightly close. */
    v->seeds[v->n_seeds].x = best_x;
    v->seeds[v->n_seeds].y = best_y;
    v->n_seeds++;
    return true;
}

/*
 * v_compute_distances — for every cell, find the nearest seed and
 * record its index + squared distance. Brute force, O(W·H·N_seeds).
 *
 * Called once at PLACING → REVEALING transition.
 */
static void v_compute_distances(Voronoi *v)
{
    for (int y = 0; y < v->h; y++) {
        for (int x = 0; x < v->w; x++) {
            int idx = v_idx(v, x, y);
            int best_d2 = -1;
            int best_i  = 0;
            for (int i = 0; i < v->n_seeds; i++) {
                int dx = x - v->seeds[i].x;
                int dy = y - v->seeds[i].y;
                int d2 = dx * dx + dy * dy;
                if (best_d2 < 0 || d2 < best_d2) {
                    best_d2 = d2;
                    best_i  = i;
                }
            }
            v->owner[idx] = (int8_t)best_i;
            v->dist2[idx] = best_d2;
        }
    }
}

/*
 * cmp_cellord — qsort comparator on dist2 ascending. Standard idiom;
 * subtraction works because both values are non-negative ints fitting
 * into int range with room to spare.
 */
static int cmp_cellord(const void *a, const void *b)
{
    const CellOrd *ca = (const CellOrd *)a;
    const CellOrd *cb = (const CellOrd *)b;
    if (ca->dist2 < cb->dist2) return -1;
    if (ca->dist2 > cb->dist2) return  1;
    return 0;
}

/*
 * v_compute_full — fill owner[] and dist2[] via brute force, then
 * sort cell_order[] by ascending dist2 to drive the REVEAL sweep.
 */
static void v_compute_full(Voronoi *v)
{
    v_compute_distances(v);

    int n = v->total_cells;
    for (int i = 0; i < n; i++) {
        v->cell_order[i].idx   = i;
        v->cell_order[i].dist2 = v->dist2[i];
    }
    qsort(v->cell_order, n, sizeof(CellOrd), cmp_cellord);
    v->reveal_progress = 0;
    v->computed = true;
}

/*
 * v_reveal_step — reveal one more cell from cell_order[]. Returns
 * true if a cell was revealed; false when the sweep is complete.
 */
static bool v_reveal_step(Voronoi *v)
{
    if (v->reveal_progress >= v->total_cells) return false;
    int idx = v->cell_order[v->reveal_progress++].idx;
    v->revealed[idx]  = true;
    v->wave_glow[idx] = 1.0f;
    return true;
}

/*
 * v_reset — clear everything, set up for a fresh PLACING phase.
 */
static void v_reset(Voronoi *v, int w, int h)
{
    v->w = w;
    v->h = h;
    v->total_cells = w * h;
    v->n_seeds = 0;
    v->reveal_progress = 0;
    v->computed = false;
    v->place_cooldown = 0;

    for (int i = 0; i < v->total_cells; i++) {
        v->owner[i]          = -1;
        v->dist2[i]          = 0;
        v->wave_glow[i]      = 0.0f;
        v->supernova_glow[i] = 1.0f;
        v->revealed[i]       = false;
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   PLACING   — drop one seed every PLACE_HOLD_TICKS frames. When all
 *               N_SEEDS are placed, run v_compute_full and transition
 *               to REVEALING.
 *   REVEALING — reveal cells in cell_order. When complete, transition
 *               to HOLD.
 *   HOLD      — wait HOLD_SECONDS, then v_reset and back to PLACING.
 */
typedef enum {
    SCENE_PLACING   = 0,
    SCENE_REVEALING = 1,
    SCENE_HOLD      = 2,
} SceneState;

typedef struct {
    Voronoi     v;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         ops_per_tick;
    int         current_theme;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    v_reset(&s->v, mw, mh);
    s->state      = SCENE_PLACING;
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

    /* Decay glows. */
    float wave_d = expf(-WAVE_GLOW_DECAY * dt);
    float nova_d = expf(-SUPERNOVA_DECAY * dt);
    for (int i = 0; i < s->v.total_cells; i++) {
        s->v.wave_glow[i]      *= wave_d;
        s->v.supernova_glow[i] *= nova_d;
    }

    switch (s->state) {

    case SCENE_PLACING:
        /* Drop one seed when the cooldown expires. */
        if (s->v.place_cooldown > 0) {
            s->v.place_cooldown--;
        } else {
            v_place_one_seed(&s->v);
            s->v.place_cooldown = PLACE_HOLD_TICKS;
            if (s->v.n_seeds >= N_SEEDS) {
                v_compute_full(&s->v);
                s->state = SCENE_REVEALING;
            }
        }
        break;

    case SCENE_REVEALING:
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!v_reveal_step(&s->v)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->v.w, s->v.h);
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
    const Voronoi *v = &s->v;

    int gx0 = (cols - v->w) / 2;
    int gy0 = ((rows - 3) - v->h) / 2 + 2;
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    /* Pass 1 — region cells (skip seed cells; they're drawn after). */
    for (int y = 0; y < v->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < v->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = v_idx(v, x, y);
            float ng = v->supernova_glow[idx];
            float wg = v->wave_glow[idx];

            int  pair, attr;
            char glyph;

            if (ng > GLOW_THRESHOLD) {
                pair = PAIR_SUPERNOVA;
                attr = A_BOLD;
                glyph = '*';
            } else if (wg > GLOW_THRESHOLD) {
                /* Just-revealed flash — bright theme accent before
                 * settling to the region's own colour. */
                pair = PAIR_FLASH;
                attr = A_BOLD;
                glyph = '*';
            } else if (v->revealed[idx]) {
                /* Solid region colour. */
                int8_t owner = v->owner[idx];
                if (owner < 0) continue;
                pair = PAIR_REGION_BASE + (owner % N_SEEDS);
                attr = A_NORMAL;
                glyph = '#';
            } else {
                continue;       /* unrevealed → blank */
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }

    /* Pass 2 — seeds on top, in their region's colour, BOLD '@'. */
    for (int i = 0; i < v->n_seeds; i++) {
        int sx = gx0 + v->seeds[i].x;
        int sy = gy0 + v->seeds[i].y;
        if (sx < 0 || sx >= cols) continue;
        if (sy < 0 || sy >= rows) continue;
        int pair = PAIR_REGION_BASE + (i % N_SEEDS);
        attron(COLOR_PAIR(pair) | A_BOLD);
        mvaddch(sy, sx, (chtype)(unsigned char)'@');
        attroff(COLOR_PAIR(pair) | A_BOLD);
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Voronoi *v = &s->v;
    const char *state_str =
        s->paused                         ? "PAUSED   " :
        (s->state == SCENE_PLACING)       ? "PLACING  " :
        (s->state == SCENE_REVEALING)     ? "REVEALING" :
                                            "HOLD     ";

    int reveal_pct = (v->total_cells > 0)
                   ? (100 * v->reveal_progress / v->total_cells)
                   : 0;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops:%-3d  %s  seeds:%d/%d  %3d%% ",
             fps, sim_fps, s->ops_per_tick, state_str,
             v->n_seeds, N_SEEDS, reveal_pct);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " VORONOI REGION MAP ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — theme + parameters + per-region swatches. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, " palette:");
    attroff(COLOR_PAIR(PAIR_HUD));
    x += 9;
    /* 8 region-colour swatches as bold '#' in each pair's colour. */
    for (int i = 0; i < N_SEEDS; i++) {
        int p = PAIR_REGION_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(1, x, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
        x += 1;
    }
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x, "  metric:euclid  map:%dx%d ", v->w, v->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " #:region  @:seed  *:flash | t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
