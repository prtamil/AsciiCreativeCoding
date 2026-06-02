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
 * Section map (layers — see ARCHITECTURE block below):
 *   §1 config       — map size, seed count, palette table, glow/timer rates
 *   §2 performance  — monotonic clock + sleep (frame-cap helpers)
 *   §3 sim data     — Seed, CellOrd, Voronoi, Scene state-machine types
 *   §4 logic        — pure queries: v_idx, dist2_cells, nearest_seed, cmp_cellord
 *   §5 simulation   — seed placement, distance compute, reveal sweep, scene_tick
 *   §6 render       — palette install, scene_draw, HUD, terminal I/O
 *   §7 app          — signals, resize, key handling, fixed-timestep main loop
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
 * References     : Concept —
 *                  • Aurenhammer, F. (1991) — "Voronoi Diagrams: A Survey
 *                    of a Fundamental Geometric Data Structure", ACM
 *                    Computing Surveys 23(3). The canonical survey: what a
 *                    Voronoi diagram is, its properties, and how to build it.
 *                  • Okabe, Boots, Sugihara & Chiu (2000) — "Spatial
 *                    Tessellations: Concepts and Applications of Voronoi
 *                    Diagrams", 2nd ed. The definitive book-length treatment.
 *                  • de Berg, Cheong, van Kreveld & Overmars —
 *                    "Computational Geometry: Algorithms and Applications",
 *                    3rd ed., ch. 7. Clean derivation of the facts the MENTAL
 *                    MODEL leans on: regions are convex, boundaries are the
 *                    perpendicular bisectors between adjacent seeds.
 *                  • Fortune, S. (1987) — "A sweepline algorithm for Voronoi
 *                    diagrams". The asymptotically faster O(N log N) method.
 *                    Not used here; brute force is faster to *read*.
 *                  • Wikipedia — "Voronoi diagram", incl. the distance-metric
 *                    variants (Manhattan/Chebyshev → diamond/square cells):
 *                    https://en.wikipedia.org/wiki/Voronoi_diagram
 *                  Rendering —
 *                  • Inigo Quilez — "Voronoi distances" / "Voronoi edges":
 *                    drawing the cell *boundaries* from the distance field.
 *                    https://iquilezles.org/articles/voronoilines/
 *                  • Rong & Tan (2006) — "Jump Flooding in GPU with
 *                    Applications to Voronoi Diagram and Distance Transform".
 *                    The JFA the CONCEPTS note cites as a faster alternative
 *                    to the brute-force fill, and how it's rendered.
 *                  • Padala — "NCURSES Programming HOWTO", TLDP: colour pairs,
 *                    glyph output, non-blocking input, resize handling.
 *                  • xterm 256-colour palette — the indices the per-region
 *                    themes draw from: https://jonasjacek.github.io/colors/
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

/* ── ARCHITECTURE (layer separation) ──────────────────────────────────── *
 *
 * The program splits into labelled layers; this table says what each one
 * mutates.  (State is still passed via plain struct pointers — the
 * const-pointer convention lands in step 5.)
 *
 *   Layer        Section  Mutates
 *   ─────────────────────────────────────────────────────────────────────
 *   PERFORMANCE  §2       nothing — reads the monotonic clock / sleeps
 *   LOGIC        §4       nothing — pure index / bounds / order queries
 *   SIMULATION   §3+§5    the Voronoi (seeds / owner / dist2 / cell_order /
 *                         reveal_progress / computed / place_cooldown) and the
 *                         Scene (state / hold_timer), plus the EFFECTS glows
 *   RENDER       §6       the terminal + Screen dims only — reads sim state,
 *                         never writes it
 *   APP          §7       run knobs (paused / ops_per_tick / current_theme /
 *                         sim_fps), map size, signal flags; drives resize+keys
 *
 *   No LOGIC corruptibility: the §4 helpers (v_idx / dist2_cells / nearest_seed
 *     / cmp_cellord) take only their arguments, do no mutation and no I/O, so
 *     reordering or deleting any RENDER/EFFECTS work cannot change what they
 *     return.
 *   No EFFECTS layer — wave_glow[] / supernova_glow[] are cosmetic per-cell
 *     flashes stored on the Voronoi; set to 1.0 on reveal/reset and decayed
 *     exponentially in scene_tick's first loop.  No dedicated functions, so no
 *     section: the one decay loop lives at the top of the tick.
 *   No DELAYS layer — the HOLD pause (Scene.hold_timer) and the inter-seed gap
 *     (Voronoi.place_cooldown) are countdowns inside scene_tick's state
 *     machine, not standalone waits.  The only real sleep is the PERFORMANCE
 *     frame cap in main.
 *
 * PER-TICK COMBINE — scene_tick (§5) is the ONE place sim state advances,
 * in this order:
 *     1. EFFECTS decay — multiply both glow arrays by their per-dt factor.
 *     2. STATE MACHINE —
 *          PLACING   : drop a seed when place_cooldown hits 0; once all
 *                      N_SEEDS are down, v_compute_full → REVEALING.
 *          REVEALING : v_reveal_step up to ops_per_tick times; when the sweep
 *                      finishes, arm hold_timer → HOLD.
 *          HOLD      : count hold_timer down; at 0, scene_reset → PLACING.
 *   RENDER (screen_draw) then PERFORMANCE (frame cap) run once per frame in
 *   main, OUTSIDE the tick.
 *
 * USER EVENTS are NOT the tick: app_handle_key (pause/reset/theme/speed/fps)
 * and app_do_resize (re-fit map + scene_reset) mutate state directly in §7;
 * the fixed-timestep loop calls scene_tick only for the simulation step.
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

    /* Smallest usable map — below this the diagram is too cramped to read.
     * The terminal is clamped into [MIN, MAX] before a map is built. */
    MAP_W_MIN         =  16,
    MAP_H_MIN         =   8,

    /* Number of seeds = number of regions = palette size. 8 is the
     * sweet spot — enough variety to look like a real cellular map,
     * not so many that adjacent regions blend visually. */
    N_SEEDS           =   8,

    /* Seed-placement rejection: a candidate seed is accepted only if
     * it's at least this many cells away from every previously-placed
     * seed. Prevents lopsided diagrams with two seeds nearly on top. */
    MIN_SEED_DIST     =  10,
    SEED_PLACE_TRIES  = 100,

    /* Hold this many ticks between successive seed drops during PLACING, so
     * each seed is visibly placed one at a time. */
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

    /* HUD occupies the top HUD_TOP_ROWS (title + params) and one bottom hint
     * row; HUD_ROWS is the total the map must leave clear. */
    HUD_TOP_ROWS      =   2,
    HUD_ROWS          =   3,

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

/* Render frame cap: the wall-clock loop never spins faster than this,
 * independent of the (possibly higher) simulation tick rate. */
#define RENDER_CAP_FPS  60
/* Spiral-of-death guard: if a frame stalls (debugger, swap), clamp the
 * elapsed time fed to the accumulator so we never try to catch up forever. */
#define MAX_FRAME_NS  (100 * NS_PER_MS)

/*
 * Theme — one named palette for the map.
 *
 * WHY a struct: a Voronoi cell carries only an owner index (which seed won);
 * the ONLY way the region structure becomes visible is by mapping that index
 * to a colour.  So the palette is the rendering core, and `t`/`T` must be able
 * to swap the whole look atomically — name + all colours travel together.
 *
 * VALUE LOGIC: every entry is an xterm-256 index (see the Rendering refs in the
 * CONCEPTS block: xterm palette + Bourke on colour choice).
 *   region[i]  — colour of region i.  There are exactly N_SEEDS=8 because one
 *                colour serves one region; with >8 seeds the renderer cycles
 *                (owner % N_SEEDS) and two regions would share a colour, which
 *                is why N_SEEDS is pinned to the palette size.  Two design
 *                styles appear in the table: DEFAULT spreads hues to MAXIMISE
 *                adjacent-region contrast (so neighbouring Voronoi cells never
 *                blur together), while the rest are gradients (e.g. OCEAN
 *                navy→cyan) that trade some contrast for a mood.
 *   seed_fg    — the '@' glyph colour; 231 (bright white) on most themes so a
 *                seed stays visible whatever region colour sits under it.
 *   flash_fg   — the '*' wave-reveal flash; a hot accent (226 yellow / 196 red)
 *                so a freshly-revealed cell pops before settling to its region.
 * The HUD/HINT/SUPERNOVA pairs are deliberately NOT here — they are fixed in
 * color_init() so the status line reads the same on every theme.
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
/* §2  performance — monotonic clock + sleep (the frame-cap helpers).     */
/*     The fixed-timestep accumulator + 60 fps cap live in main (§7).     */
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
/* §3  simulation — data structures                                       */
/*     (the types the SIMULATION layer reads & mutates; behaviour is §5)  */
/* ===================================================================== */

/*
 * Seed — one generator point of the diagram.  Called a "site" or "generator"
 * in the Voronoi literature (Aurenhammer 1991; Okabe et al.): the diagram is
 * DEFINED as "assign every point to its nearest site", so the seed set is the
 * entire input — N_SEEDS of these produce N_SEEDS regions.
 *
 * VALUE LOGIC:
 *   x, y  : integer cell position.  Sub-cell precision is pointless here
 *           because the output is rasterised one character per cell and the
 *           nearest-seed test is evaluated at cell centres — fractional seed
 *           coordinates could not move a region boundary by less than a cell.
 */
typedef struct {
    int x, y;
} Seed;

/*
 * CellOrd — one entry of the reveal schedule: a cell paired with its sort key.
 *
 * WHY it exists: the Voronoi diagram is computed all at once, but we UNVEIL it
 * in order of distance-to-owner so the audience sees coloured "ripples" grow
 * from every seed (the ripple metaphor in the MENTAL MODEL).  Ordering cells by
 * their distance-to-nearest-site is exactly a distance-transform sweep
 * (cf. Rong & Tan 2006, Rendering refs); CellOrd is the sortable record that
 * sweep runs on.  It belongs to the VISUALISATION, not the Voronoi algorithm
 * itself — the diagram is identical whatever order cells reveal in.
 *
 * VALUE LOGIC:
 *   idx   : flat cell index (y*w + x) into owner[]/dist2[]/revealed[].
 *   dist2 : SQUARED Euclidean distance to the owner seed.  Squared, not the
 *           true distance, because sqrt is monotone — it cannot change the sort
 *           order — so we skip W·H sqrt calls and keep an exact integer key
 *           (ties on equal dist2 are fine; the reveal accepts any tie-break).
 */
typedef struct {
    int idx;
    int dist2;
} CellOrd;

/*
 * Voronoi — the diagram itself: the seeds, the per-cell nearest-seed result,
 * and the bookkeeping the animated reveal needs.
 *
 * INTENT / METHOD: this is the diagram built by direct nearest-site search —
 * owner[cell] = argmin_i dist2(cell, seed_i) over every cell (Aurenhammer 1991;
 * de Berg et al. ch. 7).  Brute force, O(W·H·N): chosen over Fortune's
 * O(N log N) sweep because at this scale (~11 K cells, 8 seeds) the simple
 * version is faster to *read* and animates identically.  The result is stored
 * explicitly per cell rather than as polygon edges, which is what lets the
 * reveal colour cells one at a time.
 *
 * Field groups by concept:
 *
 *   THE DIAGRAM (the answer "who owns each cell?")
 *   owner[]      : per-cell seed index 0..N_SEEDS-1 (after compute)
 *   dist2[]      : per-cell squared distance to owner seed
 *
 *   THE SEEDS (the inputs)
 *   seeds[]      : seed positions (filled during PLACING)
 *   n_seeds      : seed count, grows from 0 to N_SEEDS during PLACING
 *
 *   THE REVEAL SWEEP (how the answer is unveiled in distance order)
 *   cell_order[] : sorted (cell idx, dist2) by ascending dist2
 *   reveal_progress: index into cell_order during REVEAL
 *   revealed[]   : per-cell bool — true once the sweep has visited this cell
 *   computed     : true once owner[] / dist2[] / cell_order[] filled
 *   place_cooldown: ticks until the next seed drops during PLACING
 *
 *   THE COSMETIC LAYER (EFFECTS — purely visual, never steers the algorithm)
 *   wave_glow[]      : per-cell flash on first reveal, decays to 0
 *   supernova_glow[] : per-cell reset flash
 *   These live here rather than in a separate type because they are per-cell
 *   and share owner[]'s indexing (one flash float per diagram cell) — the
 *   cohesive owner is the cell grid, so they ride along with it.
 *
 *   w, h, total_cells: the cell-grid dimensions every array above is sized by.
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

/*
 * Scene state machine — wraps the Voronoi with the run knobs and the phase
 * we're in:
 *
 *   PLACING   — drop one seed every PLACE_HOLD_TICKS frames. When all
 *               N_SEEDS are placed, run v_compute_full and transition
 *               to REVEALING.
 *   REVEALING — reveal cells in cell_order. When complete, transition
 *               to HOLD.
 *   HOLD      — wait HOLD_SECONDS, then v_reset and back to PLACING.
 */
/*
 * SceneState — which phase of the showcase loop we are in.  This is not part
 * of the Voronoi algorithm; it is the three-beat narrative the demo plays on
 * repeat — build the inputs, reveal the answer, admire it — so the values are
 * ordered in the sequence they actually occur (PLACING→REVEALING→HOLD→reset).
 *   SCENE_PLACING   — seeds drop in one at a time (the inputs appear).
 *   SCENE_REVEALING — the precomputed diagram unveils in distance order.
 *   SCENE_HOLD      — the finished map is held still for HOLD_SECONDS.
 */
typedef enum {
    SCENE_PLACING   = 0,
    SCENE_REVEALING = 1,
    SCENE_HOLD      = 2,
} SceneState;

/*
 * Scene — the table of contents: the diagram being built plus the phase we're
 * in and the few things the user can turn.  Grouped by concept, NOT by which
 * key changes them:
 *   WHAT     — v: the Voronoi diagram (the simulation proper).
 *   PHASE    — state + hold_timer + paused: where we are in
 *              PLACING→REVEALING→HOLD, the HOLD countdown, and the freeze flag.
 *   SIM KNOB — ops_per_tick: cells revealed per tick (the reveal speed).
 *   RENDER   — current_theme: palette index. A render concept even though a
 *              key drives it, so it is grouped apart from the sim knob.
 */
typedef struct {
    Voronoi     v;              /* WHAT:     the diagram being built       */
    SceneState  state;          /* PHASE:    PLACING / REVEALING / HOLD    */
    float       hold_timer;     /* PHASE:    seconds left in HOLD          */
    bool        paused;         /* PHASE:    tick frozen?                  */
    int         ops_per_tick;   /* SIM KNOB: cells revealed per tick       */
    int         current_theme;  /* RENDER:   palette index into themes[]   */
} Scene;

/* ===================================================================== */
/* §4  logic — pure queries: no mutation, no I/O, readable in isolation.  */
/*     Take only their arguments, so RENDER/EFFECTS order cannot affect   */
/*     what they return.                                                  */
/* ===================================================================== */

static inline int v_idx(const Voronoi *v, int x, int y) { return y * v->w + x; }

/* Squared Euclidean distance between two cells. Squared (not sqrt'd) because
 * the result is only ever compared, and sqrt is monotone — see CellOrd. */
static inline int dist2_cells(int ax, int ay, int bx, int by)
{
    int dx = ax - bx, dy = ay - by;
    return dx * dx + dy * dy;
}

/* Rejection-sampling predicate: is (x,y) within MIN_SEED_DIST of any seed
 * already placed? Comparing squared distances avoids a sqrt. */
static bool seed_too_close(const Voronoi *v, int x, int y)
{
    for (int i = 0; i < v->n_seeds; i++)
        if (dist2_cells(x, y, v->seeds[i].x, v->seeds[i].y)
                < MIN_SEED_DIST * MIN_SEED_DIST)
            return true;
    return false;
}

/* The Voronoi rule itself: argmin_i dist2(cell, seed_i). Returns the winning
 * seed index and writes its squared distance to *out_d2. Pure — the mutation
 * (writing owner[]/dist2[]) happens in the caller. */
static int nearest_seed(const Voronoi *v, int x, int y, int *out_d2)
{
    int best_d2 = -1, best_i = 0;
    for (int i = 0; i < v->n_seeds; i++) {
        int d2 = dist2_cells(x, y, v->seeds[i].x, v->seeds[i].y);
        if (best_d2 < 0 || d2 < best_d2) {
            best_d2 = d2;
            best_i  = i;
        }
    }
    *out_d2 = best_d2;
    return best_i;
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

/* ===================================================================== */
/* §5  simulation — state advance: seed placement, distance compute,      */
/*     the reveal sweep, and scene_tick (the ONE per-tick combine).       */
/* ===================================================================== */

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

    /* Rejection sampling: try random spots, keep the first that clears the
     * min-spacing test. */
    int best_x = 0, best_y = 0;
    for (int attempt = 0; attempt < SEED_PLACE_TRIES; attempt++) {
        best_x = rand() % v->w;
        best_y = rand() % v->h;
        if (!seed_too_close(v, best_x, best_y)) break;
    }
    /* If none cleared in SEED_PLACE_TRIES, best_* holds the last candidate and
     * is accepted as-is — at worst slightly close (degrades on tiny maps). */

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
            int d2;
            v->owner[idx] = (int8_t)nearest_seed(v, x, y, &d2);
            v->dist2[idx] = d2;
        }
    }
}

/*
 * build_reveal_order — tag every cell with its dist2, then sort ascending so
 * the reveal sweep unveils each region nearest-to-its-seed first (the ripple).
 */
static void build_reveal_order(Voronoi *v)
{
    int n = v->total_cells;
    for (int i = 0; i < n; i++) {
        v->cell_order[i].idx   = i;
        v->cell_order[i].dist2 = v->dist2[i];
    }
    qsort(v->cell_order, n, sizeof(CellOrd), cmp_cellord);
}

/*
 * v_compute_full — run the diagram, then build its reveal schedule. Called
 * once at the PLACING → REVEALING transition.
 */
static void v_compute_full(Voronoi *v)
{
    v_compute_distances(v);
    build_reveal_order(v);
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

/* EFFECTS: exponentially fade every per-cell flash toward 0 this tick. */
static void decay_glows(Voronoi *v, float dt)
{
    float wave_d = expf(-WAVE_GLOW_DECAY * dt);
    float nova_d = expf(-SUPERNOVA_DECAY * dt);
    for (int i = 0; i < v->total_cells; i++) {
        v->wave_glow[i]      *= wave_d;
        v->supernova_glow[i] *= nova_d;
    }
}

/* PLACING: drop one seed when the cooldown expires; once all seeds are down,
 * compute the diagram and switch to REVEALING. */
static void advance_placing(Scene *s)
{
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
}

/* REVEALING: unveil up to ops_per_tick cells; when the sweep runs dry, arm the
 * hold timer and switch to HOLD. */
static void advance_revealing(Scene *s)
{
    for (int i = 0; i < s->ops_per_tick; i++) {
        if (!v_reveal_step(&s->v)) {
            s->state      = SCENE_HOLD;
            s->hold_timer = HOLD_SECONDS;
            break;
        }
    }
}

/* HOLD: count the admire-it timer down, then reset for a fresh diagram. */
static void advance_hold(Scene *s, float dt)
{
    s->hold_timer -= dt;
    if (s->hold_timer <= 0.0f)
        scene_reset(s, s->v.w, s->v.h);
}

static void scene_tick(Scene *s, float dt)
{
    if (s->paused) return;

    decay_glows(&s->v, dt);                 /* EFFECTS: fade reveal/reset flashes */

    switch (s->state) {                     /* advance the current phase */
    case SCENE_PLACING:   advance_placing(s);    break;
    case SCENE_REVEALING: advance_revealing(s);  break;
    case SCENE_HOLD:      advance_hold(s, dt);   break;
    }
}

/* ===================================================================== */
/* §6  render — state → screen. Reads sim state, never mutates it; the    */
/*     only writes are to the terminal and to Screen's own dimensions.    */
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

/*
 * Screen — the terminal viewport: its current size in cells.  A render-target
 * concept, deliberately kept as its own narrow type so the §6 functions can
 * take Screen* alone and never reach for the whole App (keeps render decoupled
 * from app/simulation).
 */
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

/* Top-left screen cell of the map: centre the w×h grid in the terminal,
 * leaving the HUD rows clear (HUD_TOP_ROWS on top, one hint row at bottom). */
static void map_screen_origin(const Voronoi *v, int cols, int rows,
                              int *gx0, int *gy0)
{
    int x0 = (cols - v->w) / 2;
    int y0 = ((rows - HUD_ROWS) - v->h) / 2 + HUD_TOP_ROWS;
    if (x0 < 0)            x0 = 0;
    if (y0 < HUD_TOP_ROWS) y0 = HUD_TOP_ROWS;
    *gx0 = x0;
    *gy0 = y0;
}

/* Pick a cell's glyph + colour from its state. Glow flashes (reset, then
 * just-revealed) win over the settled region colour. Returns false if the cell
 * draws nothing (unrevealed, or revealed but ownerless). */
static bool cell_appearance(const Voronoi *v, int idx,
                            int *pair, int *attr, char *glyph)
{
    if (v->supernova_glow[idx] > GLOW_THRESHOLD) {        /* reset flash */
        *pair = PAIR_SUPERNOVA; *attr = A_BOLD; *glyph = '*';
    } else if (v->wave_glow[idx] > GLOW_THRESHOLD) {      /* just-revealed flash */
        *pair = PAIR_FLASH;     *attr = A_BOLD; *glyph = '*';
    } else if (v->revealed[idx]) {                        /* settled region colour */
        int8_t owner = v->owner[idx];
        if (owner < 0) return false;
        *pair = PAIR_REGION_BASE + (owner % N_SEEDS); *attr = A_NORMAL; *glyph = '#';
    } else {
        return false;                                     /* unrevealed → blank */
    }
    return true;
}

/* Paint every on-screen region cell in its current appearance. */
static void draw_region_cells(const Voronoi *v, int gx0, int gy0,
                              int cols, int rows)
{
    for (int y = 0; y < v->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < v->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int pair, attr; char glyph;
            if (!cell_appearance(v, v_idx(v, x, y), &pair, &attr, &glyph))
                continue;

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

/* Draw the seed anchors on top, each a BOLD '@' in its own region's colour. */
static void draw_seeds(const Voronoi *v, int gx0, int gy0, int cols, int rows)
{
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

static void scene_draw(const Scene *s, int cols, int rows)
{
    const Voronoi *v = &s->v;
    int gx0, gy0;
    map_screen_origin(v, cols, rows, &gx0, &gy0);
    draw_region_cells(v, gx0, gy0, cols, rows);   /* settled colours + flashes */
    draw_seeds(v, gx0, gy0, cols, rows);          /* '@' anchors on top */
}

/* Phase → fixed-width label for the HUD status line. */
static const char *state_label(const Scene *s)
{
    if (s->paused)                   return "PAUSED   ";
    if (s->state == SCENE_PLACING)   return "PLACING  ";
    if (s->state == SCENE_REVEALING) return "REVEALING";
    return "HOLD     ";
}

/* The 8-colour region legend: bold '#' swatches left-to-right from (row,x). */
static void draw_palette_swatches(int row, int x)
{
    for (int i = 0; i < N_SEEDS; i++) {
        int p = PAIR_REGION_BASE + i;
        attron(COLOR_PAIR(p) | A_BOLD);
        mvaddch(row, x + i, '#');
        attroff(COLOR_PAIR(p) | A_BOLD);
    }
}

static void screen_draw(const Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Voronoi *v = &s->v;
    const char *state_str = state_label(s);

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
    draw_palette_swatches(1, x);
    x += N_SEEDS;
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
/* §7  app — orchestration. Owns run knobs + signal flags; drives resize  */
/*     and key events (NOT part of the tick) and the fixed-timestep loop. */
/* ===================================================================== */

/*
 * App — the running program: the Scene being animated, the terminal it draws
 * to, and the loop/lifecycle state that is neither simulation nor render.
 * Kept distinct from Scene so the render layer can take Screen* alone, and so
 * the signal handler can reach the run flags via the single g_app instance.
 *   DOMAIN  — scene + screen: the simulation and its draw target.
 *   PACING  — sim_fps: fixed-timestep rate (the loop's pacing knob; distinct
 *             from Scene.ops_per_tick, which paces work *within* a tick).
 *   GEOMETRY— map_w/map_h: chosen map dimensions, re-applied on reset/resize.
 *   LIFECYCLE — running/need_resize: signal-written flags (sig_atomic_t).
 */
typedef struct {
    Scene                 scene;        /* DOMAIN:    the simulation         */
    Screen                screen;       /* DOMAIN:    its draw target        */
    int                   sim_fps;      /* PACING:    fixed-timestep rate    */
    int                   map_w, map_h; /* GEOMETRY:  chosen map size        */
    volatile sig_atomic_t running;      /* LIFECYCLE: clear to exit          */
    volatile sig_atomic_t need_resize;  /* LIFECYCLE: set by SIGWINCH        */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_ROWS;   /* leave HUD rows clear */
    if (mw < MAP_W_MIN) mw = MAP_W_MIN;
    if (mh < MAP_H_MIN) mh = MAP_H_MIN;
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
        if (dt > MAX_FRAME_NS) dt = MAX_FRAME_NS;   /* spiral-of-death guard */

        int64_t tick_ns = TICK_NS(app->sim_fps);
        float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

        sim_accum += dt;
        while (sim_accum >= tick_ns) {
            scene_tick(&app->scene, dt_sec);
            sim_accum -= tick_ns;
        }

        /* Rolling FPS average over FPS_UPDATE_MS windows. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* Sleep out the rest of this render frame (capped at RENDER_CAP_FPS). */
        int64_t elapsed = clock_ns() - frame_time + dt;
        clock_sleep_ns(TICK_NS(RENDER_CAP_FPS) - elapsed);

        screen_draw(&app->screen, &app->scene, fps_display, app->sim_fps);
        screen_present();

        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
