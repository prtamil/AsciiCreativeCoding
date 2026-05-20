/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * voronoi.c — Animated Voronoi Diagram
 *
 * N_SEEDS seed points drift with Langevin Brownian motion and bounce off
 * the screen boundary.  Each frame every terminal cell is coloured by its
 * nearest seed (brute-force O(cells × seeds) nearest-neighbour search).
 *
 * PHYSICS
 * ───────
 * Langevin equation for each seed:
 *   dv/dt = −γ·v + σ·ξ      (ξ = uniform random in [−1,1])
 * Discrete update per tick:
 *   v += (−DAMP·v + NOISE·ξ) · dt
 *   p += v · dt
 * This gives self-limiting Brownian motion (terminal speed ≈ NOISE/DAMP).
 *
 * DRAWING
 * ───────
 * For each cell centre (px, py) in pixel space:
 *   • find d1 (nearest seed distance) and d2 (second nearest)
 *   • if d2 − d1 < BORDER_PX → border cell → draw '+' at normal brightness
 *   • if d1 < SEED_PX         → seed centre  → draw 'O' bold
 *   • otherwise                → interior    → draw '.' dim
 * All cells coloured with their nearest seed's colour pair.
 *
 * Keys:
 *   q/ESC quit   space pause   r reset seeds   ] / [  sim Hz up / down
 *
 * Section map:
 *   §1 config  — sim/render fps, cell pixel size, Langevin params, magic
 *                literals named with context (SEED_SPAWN_*, BOUNCE_*, …)
 *   §2 clock   — monotonic ns clock + nanosleep
 *   §3 color   — seed palette (1..N_COLORS) + CP_HUD / CP_HINT pairs
 *   §4 coords  — pixel ↔ cell conversion helpers (pw, ph)
 *   §5 entity  — Seed + Voronoi structs; spawn / colour shuffle;
 *                Langevin step + wall bounce; the brute-force renderer
 *   §6 scene   — SimControls + Scene structs; scene_init / tick / draw
 *   §7 screen  — ncurses init/teardown + canonical HUD (top status,
 *                bottom action hints)
 *   §8 app     — POSIX signal flags, input handler, fixed-timestep
 *                main loop with FPS meter + frame-rate cap
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra voronoi.c -o voronoi -lncurses -lm
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Brute-force Voronoi diagram — O(cells × seeds) per frame.
 *                  For each cell, scan all seeds and find the nearest.
 *                  Efficient for small N (N_SEEDS ≤ 30); for larger N, a
 *                  Fortune's sweep-line algorithm gives O(N log N).
 *
 * Math           : The Voronoi diagram partitions the plane into regions where
 *                  each region is the set of points closer to one seed than all
 *                  others.  The dual graph of the Voronoi diagram is the Delaunay
 *                  triangulation (every Voronoi edge connects the circumcentres
 *                  of two Delaunay triangles).
 *                  Border detection: cell is a border cell when the distance to
 *                  the nearest seed and second-nearest differ by less than BORDER_PX.
 *                  This approximates the Voronoi edge without exact line computation.
 *
 * Physics        : Seeds move under the Langevin equation:
 *                    dv/dt = −γ·v + σ·ξ  (ξ = white noise)
 *                  This is Ornstein-Uhlenbeck process — Brownian motion with
 *                  mean-reverting velocity (terminal speed = NOISE/DAMP).
 *
 * References
 * ──────────
 *   ── Original algorithm papers (Voronoi diagrams) ───────────────
 *   [1] Voronoi, G. (1908), "Nouvelles applications des paramètres
 *       continus à la théorie des formes quadratiques", J. reine
 *       angew. Math. 134, pp. 198-287 — the FOUNDATIONAL paper that
 *       gives the diagram its name; defines V(i) as the locus of
 *       points closer to seed i than any other seed.
 *   [2] Delaunay, B. (1934), "Sur la sphère vide", Bull. Acad. Sci.
 *       URSS, Class. Sci. Mat. 7, pp. 793-800 — introduces the
 *       Delaunay triangulation, the dual graph of the Voronoi
 *       diagram (every Voronoi vertex is a Delaunay circumcentre).
 *   [3] Shamos, M. I. & Hoey, D. (1975), "Closest-point problems",
 *       16th FOCS, pp. 151-162 — earliest O(N log N) divide-and-
 *       conquer construction; the first computational-geometry
 *       paper to give Voronoi a polynomial bound.
 *   [4] Fortune, S. (1987), "A sweepline algorithm for Voronoi
 *       diagrams", Algorithmica 2, pp. 153-174 — the modern O(N log N)
 *       sweep-line algorithm (NOT used here; this file uses naive
 *       O(cells × N) per frame because N ≤ 30).
 *   [5] Lloyd, S. P. (1982), "Least squares quantization in PCM",
 *       IEEE Trans. Inf. Theory 28(2), pp. 129-137 — the iterative
 *       "Lloyd's algorithm" for centroidal Voronoi tessellations
 *       (k-means).  Relevant if you want to relax seed positions
 *       toward their cell centroids.
 *
 *   ── Canonical textbooks ────────────────────────────────────────
 *   [6] Aurenhammer, F. (1991), "Voronoi diagrams — A survey of a
 *       fundamental geometric data structure", ACM Computing Surveys
 *       23(3), pp. 345-405 — the comprehensive survey: algorithms,
 *       applications, generalisations.  Read this first if approaching
 *       Voronoi seriously.
 *   [7] Preparata, F. P. & Shamos, M. I. (1985), "Computational
 *       Geometry: An Introduction", Springer — Ch. 5 covers Voronoi
 *       diagrams in the first textbook treatment of the field.
 *   [8] de Berg, M., Cheong, O., van Kreveld, M. & Overmars, M.
 *       (2008), "Computational Geometry: Algorithms and Applications"
 *       (3rd ed.), Springer — Ch. 7 (Voronoi) + Ch. 9 (Delaunay).
 *       The standard graduate text.
 *   [9] Okabe, A., Boots, B., Sugihara, K. & Chiu, S. N. (2000),
 *       "Spatial Tessellations: Concepts and Applications of Voronoi
 *       Diagrams" (2nd ed.), Wiley — the encyclopaedic monograph;
 *       every variant and application surveyed in depth.
 *
 *   ── Langevin / Brownian-motion physics (§5 seed dynamics) ──────
 *  [10] Langevin, P. (1908), "Sur la théorie du mouvement brownien",
 *       C. R. Acad. Sci. Paris 146, pp. 530-533 — the original
 *       Langevin equation: dv/dt = −γv + σξ.
 *  [11] Uhlenbeck, G. E. & Ornstein, L. S. (1930), "On the theory of
 *       the Brownian motion", Phys. Rev. 36(5), pp. 823-841 — the
 *       OU process: mean-reverting velocity with terminal speed
 *       σ/γ.  Exactly the dynamics §5 implements.
 *
 *   ── Algorithm visualisation / map generation ───────────────────
 *  [12] Patel, A., "Polygonal Map Generation for Games", Red Blob
 *       Games — https://www.redblobgames.com/maps/mapgen2/ —
 *       walkthrough of Voronoi diagrams as a procedural-generation
 *       primitive; visual presentation of cells, edges, and the
 *       Voronoi-Delaunay duality.
 *
 *   ── Companion files in this project ────────────────────────────
 *   See also:  procedural/generational/voronoi_region_map.c (static
 *     Voronoi for region mapping); procedural/generational/
 *     delaunay_triangulation.c (the dual triangulation).
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Pin N seeds in the plane.  For every other point in the
 * plane, ask "which seed am I closest to?"  The points that
 * share an answer form a CELL — one Voronoi region per seed.
 * The cell boundaries are the locus of points equidistant from
 * two seeds (perpendicular bisectors).  Brute-force compute by
 * asking the question per terminal cell.
 *
 * ALGORITHM IN STEPS  (per frame)
 * ───────────────────────────────
 *  1. Move seeds (Langevin: damped Brownian, mean-reverting).
 *  2. For each terminal CELL (col, row):
 *     a. Compute d1 = nearest seed distance, d2 = second
 *        nearest, by scanning all N seeds.
 *     b. If d2 - d1 < BORDER_PX: cell is on a Voronoi edge →
 *        paint '+' in the nearest seed's colour.
 *        (The border is "where two seeds are nearly tied.")
 *     c. Else if d1 < SEED_PX: cell is a seed centre → paint 'O'.
 *     d. Else: cell is interior → paint '.' dim.
 *  3. HUD + present.
 *
 * KEY FORMULAS
 * ────────────
 *   Voronoi cell of seed i:
 *     V(i) = { p : ∀j ≠ i, dist(p, seed_i) ≤ dist(p, seed_j) }
 *
 *   Border test (approximate):
 *     point p is "on the boundary" if
 *       d_2nd_nearest(p) - d_1st_nearest(p) < BORDER_PX
 *
 *   Langevin (Ornstein-Uhlenbeck velocity):
 *     v += (-DAMP · v + NOISE · ξ) · dt    where ξ ∈ [-1, 1] random
 *     p += v · dt
 *     terminal speed ≈ NOISE / DAMP
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── HOW TO READ THIS FILE ───────────────────────────────────────────── *
 *
 * Reading order
 * ─────────────
 *   1. CONCEPTS + MENTAL MODEL above — read first as prose.
 *   2. §5 entity — THE HEART of this file.  Three named pipelines:
 *        • voronoi_reset / voronoi_init — random-spawn + colour shuffle.
 *        • voronoi_tick                 — Langevin velocity step,
 *                                         integrate, bounce-off-walls.
 *        • voronoi_draw                 — per-cell brute-force nearest-
 *                                         seed scan + classify + paint.
 *      Helpers above each pipeline read top-to-bottom in algorithm order.
 *   3. §6 scene — SimControls + Scene structs, scene_init / tick / draw.
 *      Trivial wrappers around §5 once you understand the data flow.
 *   4. §7 screen — canonical project HUD (top=data, bottom=actions).
 *   5. §8 app   — main loop with handle_resize / fixed-timestep sim /
 *                 fps meter / frame-rate cap.  Each step is one helper.
 *
 * Variable-naming convention
 * ──────────────────────────
 *   Almost everything lives in `Scene scene` on main()'s stack and
 *   threads through every function as `Scene *s` (mutators) or
 *   `const Scene *s` (observers).  Only the POSIX signal flags
 *   (g_running, g_need_resize) remain global; signal handlers can
 *   only touch sig_atomic_t globals safely.
 *
 *   s->voronoi.seeds[N_SEEDS]   moving generator points (px, py, vx, vy, pair).
 *   s->sim.paused / s->sim.fps  user-controlled playback knobs.
 *   s->scene_cols / scene_rows  terminal extent (cells); voronoi_draw
 *                               iterates rows in [HUD_TOP_ROWS,
 *                               scene_rows − HUD_BOTTOM_ROWS).
 *
 *   d1_sq, d2_sq                squared distances from a cell to its
 *                               nearest / second-nearest seed (kept
 *                               squared inside find_two_nearest_seeds
 *                               to skip per-seed sqrt; one sqrt at end).
 *   BORDER_PX = 15              border detection threshold (in pixels).
 *   SEED_PX   = 12              seed-centre detection radius (in pixels).
 *   DAMP, NOISE                 Langevin coefficients γ and σ.
 *
 * Background you need
 * ───────────────────
 *   - Distance: |p - q| = sqrt((px - qx)² + (py - qy)²).
 *   - Brute-force search: scan all N candidates for min/second-min.
 *
 * Background you DON'T need
 * ─────────────────────────
 *   - Fortune's sweep-line algorithm (O(N log N) Voronoi).
 *   - Spatial indexing (kd-tree, etc.) for accelerated nearest-
 *     neighbour search.  Algorithms/kd_tree.c covers that
 *     separately.
 *   - Voronoi-Delaunay duality.  Mentioned in CONCEPTS; see
 *     procedural/generational/delaunay_triangulation.c for the
 *     dual structure.
 *
 * ─────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <stdio.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    SIM_FPS_MIN     = 10,
    SIM_FPS_DEFAULT = 60,
    SIM_FPS_MAX     = 120,
    SIM_FPS_STEP    = 10,
    FPS_UPDATE_MS   = 500,
    N_COLORS        = 7,
    N_SEEDS         = 24,
};

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/* Pixel cell dimensions (logical sub-pixel spacing) */
#define CELL_W  8
#define CELL_H  16

/* Langevin motion parameters (Ornstein-Uhlenbeck terminal speed = NOISE/DAMP). */
#define DAMP     2.0f    /* velocity damping coefficient γ (s⁻¹)            */
#define NOISE   60.0f    /* random force amplitude σ (px/s per √s)           */

/* Drawing thresholds in pixels. */
#define BORDER_PX  15.0f  /* d2−d1 threshold for border cell ('+')         */
#define SEED_PX    12.0f  /* d1 threshold for seed-centre cell ('O')       */

/*
 * Seed spawn / bounce geometry: margins are expressed as a multiple of
 * the pixel cell size, so they scale with terminal text dimensions.
 *   SEED_SPAWN_MARGIN_*  : a Voronoi reset draws each seed uniformly in
 *                          the rectangle inset by this many cells.
 *   BOUNCE_MARGIN_*      : voronoi_tick clamps + reflects positions that
 *                          drift past this many cells from the edge.
 *   SEED_INIT_VEL_SCALE  : initial velocity spread (px/s).  The Langevin
 *                          step drags |v| toward NOISE/DAMP within a few
 *                          frames regardless of this value.
 */
#define SEED_SPAWN_MARGIN_COLS   3
#define SEED_SPAWN_MARGIN_ROWS   2
#define BOUNCE_MARGIN_COLS       2
#define BOUNCE_MARGIN_ROWS       2
#define SEED_INIT_VEL_SCALE     20.0f

/* Voronoi-draw sentinel: any real squared distance is far below this. */
#define DIST_SQ_INFINITY  1e18f

/* Cell-center sub-pixel offset: ray tests use the centre of the cell,
 * not its top-left corner, so visibility decisions match what the user
 * sees painted in the middle of the glyph. */
#define CELL_CENTER_OFFSET_FRAC  0.5f

/* HUD rows reserved at top (status) and bottom (action hints). */
#define HUD_TOP_ROWS     1
#define HUD_BOTTOM_ROWS  1

/* Render-loop frame budget: cap the loop at 60 Hz regardless of sim fps. */
#define RENDER_TARGET_FPS  60
#define RENDER_FRAME_NS    (NS_PER_SEC / RENDER_TARGET_FPS)

/* FPS-meter sampling window: how often the HUD's displayed fps refreshes. */
#define FPS_UPDATE_PERIOD_NS  ((int64_t)FPS_UPDATE_MS * NS_PER_MS)

/* Spiral-of-death guard: if a single frame measures longer than this,
 * clamp dt so the fixed-timestep accumulator can never run away. */
#define MAX_FRAME_DT_NS  (100 * NS_PER_MS)

/* ESC key code — POSIX terminals send 27 for both ESC and the start of
 * many escape sequences; treated as quit here. */
#define KEY_ESC   27

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
        .tv_nsec = (long)(ns % NS_PER_SEC),
    };
    nanosleep(&req, NULL);
}

/* ===================================================================== */
/* §3  color                                                              */
/* ===================================================================== */

/*
 * Colour pair IDs.
 *   1..N_COLORS are the SEED palette (each Voronoi cell paints in one of
 *   these); the bottom of CP_HUD / CP_HINT are RESERVED across every
 *   demo in the project so the top/bottom HUD strips have stable IDs.
 */
enum {
    CP_HUD  = 8,    /* top status row:  bright yellow + A_BOLD */
    CP_HINT = 9,    /* bottom action row: bright cyan + A_BOLD */
};

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(1,       196, COLOR_BLACK);     /* seed palette */
        init_pair(2,       208, COLOR_BLACK);
        init_pair(3,       226, COLOR_BLACK);
        init_pair(4,        46, COLOR_BLACK);
        init_pair(5,        51, COLOR_BLACK);
        init_pair(6,        75, COLOR_BLACK);
        init_pair(7,       201, COLOR_BLACK);
        init_pair(CP_HUD,  226, -1);              /* bright yellow */
        init_pair(CP_HINT,  51, -1);              /* bright cyan   */
    } else {
        init_pair(1,       COLOR_RED,     COLOR_BLACK);
        init_pair(2,       COLOR_RED,     COLOR_BLACK);
        init_pair(3,       COLOR_YELLOW,  COLOR_BLACK);
        init_pair(4,       COLOR_GREEN,   COLOR_BLACK);
        init_pair(5,       COLOR_CYAN,    COLOR_BLACK);
        init_pair(6,       COLOR_BLUE,    COLOR_BLACK);
        init_pair(7,       COLOR_MAGENTA, COLOR_BLACK);
        init_pair(CP_HUD,  COLOR_YELLOW,  -1);
        init_pair(CP_HINT, COLOR_CYAN,    -1);
    }
}

/* ===================================================================== */
/* §4  coords — pixel ↔ cell                                              */
/* ===================================================================== */

static inline float pw(int cols) { return (float)cols * CELL_W; }
static inline float ph(int rows) { return (float)rows * CELL_H; }

/* ===================================================================== */
/* §5  entity — Seed, Voronoi                                             */
/* ===================================================================== */

/*
 * Seed — one generator point of the Voronoi diagram.
 *
 * Intent
 *   The atomic unit of the simulation.  Every cell in the terminal
 *   asks "which Seed am I closest to?" each frame; the answers
 *   partition the plane into Voronoi regions, one per Seed.
 *
 * Why position + velocity (not just position)
 *   Static seeds would give a fixed diagram.  Velocity lets the
 *   Langevin tick (see §5 voronoi_tick) integrate motion smoothly:
 *     v_{t+1} = v_t + (−γ·v + σ·ξ) · dt    (Ornstein-Uhlenbeck)
 *     p_{t+1} = p_t + v · dt
 *   The diagram then RE-PARTITIONS itself each frame as seeds drift,
 *   which is what gives the running file its life.
 *
 * Why pair (the colour) lives on the Seed
 *   Each Voronoi region is painted in its seed's colour.  Storing the
 *   palette index on the Seed avoids a per-frame lookup by seed-index
 *   in the inner cell loop (which is already O(cells × N_SEEDS)).
 *   The palette index is fixed for the seed's lifetime (assigned by
 *   voronoi_reset + Fisher-Yates shuffled so adjacent indices don't
 *   share a colour).
 *
 * Members
 *   px, py     position in PIXEL space (sub-cell precision).  Cell
 *              coordinates are recovered as col = px / CELL_W,
 *              row = py / CELL_H.  Distances are computed in pixel
 *              space so that Voronoi regions have CORRECT geometric
 *              proportions, undistorted by the 8×16 terminal cell
 *              aspect ratio.
 *   vx, vy     velocity in px/s, evolved by the Langevin step.
 *   pair       palette index in [1, N_COLORS].  Shuffled at reset to
 *              minimise the chance of two adjacent seeds sharing a
 *              colour (which would make their shared border invisible).
 *
 * Invariants
 *   px ∈ [margin_l, W − margin_l]    after voronoi_tick wall-bounce.
 *   py ∈ [margin_t, H − margin_t]    after voronoi_tick wall-bounce.
 *   |v| converges to NOISE / DAMP    (Ornstein-Uhlenbeck terminal speed).
 *   pair ∈ [1, N_COLORS]             never reassigned after voronoi_reset.
 *
 * References
 *   [1] Voronoi 1908 — the seed point is the original "centre" of a
 *       Voronoi region.
 *   [10] Langevin 1908 / [11] Uhlenbeck-Ornstein 1930 — the dynamics
 *       used to evolve (px, py, vx, vy) each tick.
 */
typedef struct {
    float px, py;   /* position in pixel space                            */
    float vx, vy;   /* velocity (px/s)                                    */
    int   pair;     /* colour pair 1–7                                    */
} Seed;

/*
 * Voronoi — the population of N_SEEDS Brownian-drifting seeds whose
 * per-frame nearest-neighbour partition IS the rendered diagram.
 *
 * Intent
 *   Owns the seeds, nothing else.  The diagram itself is NOT stored
 *   here — it is re-computed by voronoi_draw() every frame as a
 *   brute-force scan over all seeds for each cell.  This struct is
 *   the INPUT to that computation; its output is the painted screen.
 *
 * Why a fixed-size array (not a dynamic list)
 *   Project rule: no allocation after init.  N_SEEDS is a compile-time
 *   constant chosen so that the per-frame cost
 *      O(scene_cols × scene_rows × N_SEEDS)
 *   stays under a millisecond on commodity terminals (e.g. 80×24×24
 *   ≈ 46k distance comparisons / frame).  Anything beyond a few
 *   dozen seeds wants Fortune 1987's sweep-line algorithm.
 *
 * Why UI state (paused/fps) is NOT here
 *   The earlier shape carried `bool paused` inside Voronoi, mixing
 *   "the simulation's data model" with "user toggles".  After the
 *   prompt-3 refactor those knobs live in SimControls (§6) and the
 *   pause-gate decision is made in scene_tick before voronoi_tick is
 *   called.  Voronoi is now pure: nothing but seeds.
 *
 * Members
 *   seeds[N_SEEDS]   the moving generator points.  Index in this array
 *                    is meaningless to the user (seeds are
 *                    indistinguishable except by `pair`); only their
 *                    spatial arrangement matters.
 *
 * Invariants
 *   Every entry's (px, py) is kept inside the scene by the wall-bounce
 *   step in voronoi_tick.  This guarantees the nearest-seed search in
 *   voronoi_draw always finds at least one in-bounds candidate.
 *
 * References
 *   [1] Voronoi 1908 — the diagram these seeds generate.
 *   [4] Fortune 1987 — the O(N log N) alternative we explicitly
 *       chose NOT to implement because N is small.
 *   [6] Aurenhammer 1991 §1 — definition of the Voronoi diagram as
 *       the partition induced by a discrete set of generator points.
 *   [9] Okabe et al. 2000 Ch. 2 — moving-seed (kinematic) Voronoi
 *       diagrams; the regime this file animates.
 */
typedef struct {
    Seed seeds[N_SEEDS];
} Voronoi;

/* ── §5 RNG helpers ──────────────────────────────────────────────────── */

/* Uniform float in [0, 1] — wrapper around rand() / RAND_MAX. */
static float rand_unit(void) { return (float)rand() / (float)RAND_MAX; }

/* Uniform float in [−1, 1] — the symmetric noise ξ used by Langevin. */
static float rand_signed(void) { return rand_unit() * 2.0f - 1.0f; }

/* Uniform float in [lo, hi]. */
static float rand_in_range(float lo, float hi)
{
    return lo + rand_unit() * (hi - lo);
}

/* ── §5 seed spawn / colour assignment ────────────────────────────────── */

/*
 * spawn_seed_uniform — place ONE seed at a random position inside the
 * scene rectangle (with margins kept clear), with a small random
 * starting velocity that the Langevin step will dominate within a
 * couple of frames.
 *
 *   Pseudocode:
 *     px := uniform in [margin_x, W − margin_x]
 *     py := uniform in [margin_y, H − margin_y]
 *     vx := signed-uniform × initial velocity scale
 *     vy := signed-uniform × initial velocity scale
 */
static void spawn_seed_uniform(
    Seed *s, float W, float H, float margin_x, float margin_y
) {
    s->px = rand_in_range(margin_x, W - margin_x);
    s->py = rand_in_range(margin_y, H - margin_y);
    s->vx = rand_signed() * SEED_INIT_VEL_SCALE;
    s->vy = rand_signed() * SEED_INIT_VEL_SCALE;
}

/* Round-robin colour assignment so the first N_COLORS slots get one
 * each, then the cycle repeats.  Followed by Fisher-Yates shuffle so
 * adjacent indices don't share a colour by construction. */
static void assign_round_robin_colors(Voronoi *v)
{
    for (int i = 0; i < N_SEEDS; i++)
        v->seeds[i].pair = (i % N_COLORS) + 1;
}

/* Fisher-Yates shuffle of colour assignments only (Knuth TAOCP Vol. 2
 * §3.4.2 Algorithm P).  Each Seed's spatial position stays fixed; only
 * its `pair` field is permuted, breaking the round-robin correlation
 * between seed index and palette index. */
static void shuffle_seed_colors(Voronoi *v)
{
    for (int i = N_SEEDS - 1; i > 0; i--) {
        int j = rand() % (i + 1);
        int t = v->seeds[i].pair;
        v->seeds[i].pair = v->seeds[j].pair;
        v->seeds[j].pair = t;
    }
}

/*
 * voronoi_reset — re-roll seed positions, velocities, and colours at
 * the current terminal extent.
 *
 *   Pseudocode:
 *     (W, H)        := scene extent in PIXEL space
 *     (mx, my)      := spawn margins (cells × cell size)
 *     for each seed: spawn_seed_uniform(scene rect, margins)
 *     assign_round_robin_colors(v)
 *     shuffle_seed_colors(v)
 */
static void voronoi_reset(Voronoi *v, int cols, int rows)
{
    float W  = pw(cols);
    float H  = ph(rows);
    float mx = (float)CELL_W * SEED_SPAWN_MARGIN_COLS;
    float my = (float)CELL_H * SEED_SPAWN_MARGIN_ROWS;

    for (int i = 0; i < N_SEEDS; i++)
        spawn_seed_uniform(&v->seeds[i], W, H, mx, my);

    assign_round_robin_colors(v);
    shuffle_seed_colors(v);
}

/* voronoi_init — zero the struct, then delegate to reset. */
static void voronoi_init(Voronoi *v, int cols, int rows)
{
    memset(v, 0, sizeof *v);
    voronoi_reset(v, cols, rows);
}

/* ── §5 Langevin integrator + wall bounce ─────────────────────────────── */

/*
 * langevin_velocity_step — one Euler-Maruyama step of
 *   dv/dt = −γ·v + σ·ξ      (Ornstein-Uhlenbeck, ξ ∈ [−1, 1])
 *
 * Discretised as
 *   v_{t+1} = v_t + (−γ·v_t + σ·ξ) · dt
 *
 * Drives |v| toward terminal speed NOISE / DAMP regardless of starting
 * velocity.  Each component (vx, vy) is updated independently with its
 * own random noise sample.
 */
static void langevin_velocity_step(Seed *s, float dt)
{
    s->vx += (-DAMP * s->vx + NOISE * rand_signed()) * dt;
    s->vy += (-DAMP * s->vy + NOISE * rand_signed()) * dt;
}

/* integrate_position — first-order Euler: p_{t+1} = p_t + v · dt. */
static void integrate_position(Seed *s, float dt)
{
    s->px += s->vx * dt;
    s->py += s->vy * dt;
}

/*
 * bounce_seed_off_walls — reflective collision with the scene rectangle.
 *
 *   For each side:
 *     if seed strayed past the wall:
 *       clamp its position to the wall, AND
 *       flip its velocity component so it moves AWAY from the wall.
 *
 *   The fabsf() form is robust: it doesn't matter if the seed was
 *   already moving outward when we caught it — the next velocity is
 *   guaranteed to push it back into the interior.
 */
static void bounce_seed_off_walls(
    Seed *s, float mxL, float mxR, float myT, float myB
) {
    if (s->px < mxL) { s->px = mxL; s->vx =  fabsf(s->vx); }
    if (s->px > mxR) { s->px = mxR; s->vx = -fabsf(s->vx); }
    if (s->py < myT) { s->py = myT; s->vy =  fabsf(s->vy); }
    if (s->py > myB) { s->py = myB; s->vy = -fabsf(s->vy); }
}

/*
 * voronoi_tick — advance every seed one Langevin step.
 *
 *   Pseudocode:
 *     compute wall-bounce rectangle (margins inset from scene)
 *     for each seed:
 *       langevin_velocity_step(s, dt)        ← v += (−γv + σξ) dt
 *       integrate_position(s, dt)            ← p += v dt
 *       bounce_seed_off_walls(s, rect)       ← clamp + reflect
 */
static void voronoi_tick(Voronoi *v, float dt, int cols, int rows)
{
    float W   = pw(cols);
    float H   = ph(rows);
    float mxL = (float)CELL_W * BOUNCE_MARGIN_COLS;
    float myT = (float)CELL_H * BOUNCE_MARGIN_ROWS;
    float mxR = W - mxL;
    float myB = H - myT;

    for (int i = 0; i < N_SEEDS; i++) {
        Seed *s = &v->seeds[i];
        langevin_velocity_step(s, dt);
        integrate_position   (s, dt);
        bounce_seed_off_walls(s, mxL, mxR, myT, myB);
    }
}

/* ── §5 Voronoi diagram rendering ────────────────────────────────────── */

/*
 * cell_center_in_pixel_space — convert cell coords (col, row) to the
 * sub-pixel CENTRE of the cell, so distance tests probe the middle of
 * the glyph rather than its top-left corner.
 *   cx = (col + ½) · CELL_W
 *   cy = (row + ½) · CELL_H
 */
static void cell_center_in_pixel_space(
    int col, int row, float *out_cx, float *out_cy
) {
    *out_cx = ((float)col + CELL_CENTER_OFFSET_FRAC) * (float)CELL_W;
    *out_cy = ((float)row + CELL_CENTER_OFFSET_FRAC) * (float)CELL_H;
}

/*
 * find_two_nearest_seeds — brute-force scan of all N_SEEDS, returning
 * the index of the nearest seed plus the squared distances to the
 * nearest and second-nearest.
 *
 *   Pseudocode:
 *     d1, d2 := +∞
 *     best   := 0
 *     for each seed k:
 *       d := |cell − seed[k]|²                  (squared distance — no sqrt)
 *       if d < d1: d2 := d1; d1 := d; best := k
 *       else if d < d2: d2 := d
 *     return (best, d1, d2)
 *
 *   Both distances kept in SQUARED form throughout the comparison so
 *   we save N_SEEDS × W × H square-root operations per frame.  The
 *   caller takes one sqrtf at the end for the classifier thresholds.
 */
static void find_two_nearest_seeds(
    const Voronoi *v, float cx, float cy,
    int *out_best, float *out_d1_sq, float *out_d2_sq
) {
    float d1_sq = DIST_SQ_INFINITY;
    float d2_sq = DIST_SQ_INFINITY;
    int   best  = 0;
    for (int k = 0; k < N_SEEDS; k++) {
        float dx  = cx - v->seeds[k].px;
        float dy  = cy - v->seeds[k].py;
        float dsq = dx * dx + dy * dy;
        if (dsq < d1_sq) {
            d2_sq = d1_sq;
            d1_sq = dsq;
            best  = k;
        } else if (dsq < d2_sq) {
            d2_sq = dsq;
        }
    }
    *out_best  = best;
    *out_d1_sq = d1_sq;
    *out_d2_sq = d2_sq;
}

/*
 * classify_voronoi_cell — three-way triage on the cell's distances.
 *
 *   d1 < SEED_PX                  → SEED-CENTRE   'O' bold
 *   d2 − d1 < BORDER_PX           → BORDER        '+' normal
 *   otherwise                     → INTERIOR      '.' dim
 *
 *   The border test "second-nearest is close to the nearest" is the
 *   approximate-Voronoi-edge trick from Patel/Red Blob — much simpler
 *   than computing analytical perpendicular bisectors, and adequate
 *   for visualisation since the eye smooths the result.
 */
static void classify_voronoi_cell(
    float d1, float d2, char *out_ch, chtype *out_attr
) {
    if (d1 < SEED_PX) {
        *out_ch   = 'O';
        *out_attr = A_BOLD;
    } else if (d2 - d1 < BORDER_PX) {
        *out_ch   = '+';
        *out_attr = 0;
    } else {
        *out_ch   = '.';
        *out_attr = A_DIM;
    }
}

/* Paint ONE cell at (row, col) with the chosen glyph + colour pair. */
static void paint_voronoi_cell(
    WINDOW *w, int row, int col, int pair, char ch, chtype attr
) {
    wattron(w, COLOR_PAIR(pair) | attr);
    mvwaddch(w, row, col, (chtype)(unsigned char)ch);
    wattroff(w, COLOR_PAIR(pair) | attr);
}

/*
 * voronoi_draw — paint the Voronoi diagram for the current seed pool.
 *
 *   Pseudocode (per terminal cell, skipping HUD rows):
 *     (cx, cy)  := cell_center_in_pixel_space(col, row)
 *     (best, d1_sq, d2_sq) := find_two_nearest_seeds(seeds, cx, cy)
 *     classify cell (seed centre / border / interior) → (glyph, attr)
 *     paint_voronoi_cell with the nearest seed's colour
 *
 *   Cost per frame: O(scene_cells × N_SEEDS) for the nearest-seed scan;
 *   the inner work uses squared distances to avoid N_SEEDS × W × H
 *   sqrtf calls.  Each visited cell takes ONE final sqrtf for the
 *   classifier — orders of magnitude cheaper than the previous shape.
 */
static void voronoi_draw(const Voronoi *v, WINDOW *w, int cols, int rows)
{
    for (int row = HUD_TOP_ROWS; row < rows - HUD_BOTTOM_ROWS; row++) {
        for (int col = 0; col < cols; col++) {
            float cx, cy;
            cell_center_in_pixel_space(col, row, &cx, &cy);

            int   best;
            float d1_sq, d2_sq;
            find_two_nearest_seeds(v, cx, cy, &best, &d1_sq, &d2_sq);
            float d1 = sqrtf(d1_sq);
            float d2 = sqrtf(d2_sq);

            char   ch;
            chtype attr;
            classify_voronoi_cell(d1, d2, &ch, &attr);
            paint_voronoi_cell(w, row, col, v->seeds[best].pair, ch, attr);
        }
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * SimControls — playback knobs the user adjusts at runtime.
 *
 * Intent
 *   Two scalars the input handler writes and the main loop / HUD read.
 *   Kept together so input handlers + HUD touch ONE bundle instead of
 *   scattered fields.  Decouples the fixed-timestep accumulator in
 *   main() from the underlying simulation rate.
 *
 * Why a sub-struct for just two scalars
 *   The boundary "user-controlled run parameters" is conceptually
 *   distinct from "simulation state" — `paused` and `fps` are written
 *   by handle_input and read by main()'s tick loop and the HUD;
 *   everything else in Scene is simulation output.  Naming them as a
 *   bundle signals that distinction to the reader and makes future
 *   extensions (target_fps, speed_multiplier, time_scale) churn-free
 *   at Scene's top level.
 *
 * Members
 *   paused      true → scene_tick is a no-op; HUD shows "PAUSED".
 *               The user toggles via SPACE; programs that pause on
 *               focus-loss could set it from a UI hook.
 *   fps         simulation tick rate (Hz).  '[' / ']' adjust by
 *               SIM_FPS_STEP, clamped to [SIM_FPS_MIN, SIM_FPS_MAX].
 *               Drives the fixed-timestep accumulator's TICK_NS(fps)
 *               period — smaller fps = larger dt per tick = faster
 *               but jerkier seed motion.
 *
 * Invariants
 *   SIM_FPS_MIN ≤ fps ≤ SIM_FPS_MAX  (handle_input clamps).
 *
 * References
 *   Fiedler, G., "Fix Your Timestep!" — gafferongames.com/post/
 *       fix_your_timestep — the canonical "accumulator-driven discrete
 *       ticks" pattern that main()'s loop implements using SimControls.
 *   Nystrom, R. (2014), "Game Programming Patterns", Ch. 9 (Game Loop) —
 *       same pattern presented as a classic loop architecture.
 */
typedef struct {
    bool paused;
    int  fps;
} SimControls;

/*
 * Scene — owns ALL simulation + render state for one run.
 *
 * Layered ownership
 *
 *     Scene
 *       ├── voronoi  : Voronoi              ← N moving seeds (§5 entity)
 *       │     └── seeds[N_SEEDS] : Seed[]   (px, py, vx, vy, pair)
 *       │
 *       ├── sim      : SimControls          ← paused + fps
 *       │
 *       └── scene_cols, scene_rows          ← terminal extent (cells)
 *
 *   Every persistent value the program needs is reachable from a
 *   single Scene*.  No file-scope simulation state exists; only the
 *   two POSIX signal flags (g_running, g_need_resize) remain global
 *   in §8 because signal handlers cannot receive a context pointer.
 *
 * Intent
 *   One instance lives on main()'s stack and is passed by pointer to
 *   every tick, renderer, and input handler.  Pure observers take
 *   `const Scene *`; mutators take `Scene *`.  The signature alone
 *   communicates which kind of access each callee performs.
 *
 * Why a SCENE struct (not file-scope globals)
 *   • Replaceability: future variants (split-screen, recorded replay)
 *     can allocate multiple Scenes — impossible with globals.
 *   • Testability: scene_tick becomes a pure transformation s → s';
 *     no hidden inputs.
 *   • Reading aid: every helper begins with `Scene *s` (or const),
 *     giving the reader ONE place to look for "what state exists".
 *
 * Why terminal extent (scene_cols, scene_rows) is a Scene field
 *   The earlier shape carried these in a separate `Screen` struct.
 *   Folding them into Scene gives a single argument signature
 *   `scene_tick(Scene*, dt)` instead of `scene_tick(Scene*, dt, cols,
 *   rows)`, and the resize handler updates ONE bundle.
 *
 * Members
 *   voronoi           Voronoi — the N moving seeds (pure data + dynamics).
 *   sim               SimControls — paused + fps.
 *   scene_cols        drawable columns (= terminal cols).
 *   scene_rows        drawable rows   (= terminal rows).  Row 0 is the
 *                     HUD status; row R−1 is the action hint strip;
 *                     voronoi_draw() paints rows 1..R−2.
 *
 * Invariants
 *   scene_cols > 0  AND  scene_rows > 0       (set by screen_init).
 *   Every seed in voronoi.seeds[] lies inside the scene rectangle
 *   (enforced by the wall-bounce step in voronoi_tick).
 *   g_running / g_need_resize are NOT here — POSIX signal handlers
 *   need sig_atomic_t globals, so they live in §8.
 *
 * References
 *   [9] Okabe et al. 2000 — "spatial tessellation" framing; the Scene
 *       holds the generator set, the tick advances them, the draw
 *       step partitions space.
 *   Nystrom 2014, "Game Programming Patterns" — single-Scene + fixed-
 *       timestep accumulator design that main() implements.
 */
typedef struct {
    Voronoi     voronoi;
    SimControls sim;
    int         scene_cols;
    int         scene_rows;
} Scene;

static void scene_init(Scene *s, int cols, int rows)
{
    memset(s, 0, sizeof *s);
    s->scene_cols = cols;
    s->scene_rows = rows;
    s->sim.paused = false;
    s->sim.fps    = SIM_FPS_DEFAULT;
    voronoi_init(&s->voronoi, cols, rows);
}

/* Re-roll seed positions / velocities at the current terminal extent.
 * Called by the 'r' key handler and after a SIGWINCH resize. */
static void scene_reset_seeds(Scene *s)
{
    voronoi_reset(&s->voronoi, s->scene_cols, s->scene_rows);
}

/* Advance simulation by one fixed-timestep tick.  paused = no-op. */
static void scene_tick(Scene *s, float dt)
{
    if (s->sim.paused) return;
    voronoi_tick(&s->voronoi, dt, s->scene_cols, s->scene_rows);
}

/* Render the Voronoi diagram for the current seed positions. */
static void scene_draw(const Scene *s, WINDOW *w)
{
    voronoi_draw(&s->voronoi, w, s->scene_cols, s->scene_rows);
}

/* ===================================================================== */
/* §7  screen                                                             */
/* ===================================================================== */

/* Bring up ncurses, then read the terminal extent into the Scene. */
static void screen_init(Scene *s)
{
    initscr(); noecho(); cbreak(); curs_set(0);
    nodelay(stdscr, TRUE); keypad(stdscr, TRUE); typeahead(-1);
    color_init();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);
}

static void screen_free(void) { endwin(); }

/* One-word run-state indicator used in the top status string. */
static const char *run_state_label(const Scene *s)
{
    return s->sim.paused ? "PAUSED " : "running";
}

/* Format the right-aligned top status line into `buf`. */
static void format_hud_status(
    const Scene *s, double fps, char *buf, size_t bufsz
) {
    snprintf(buf, bufsz,
        " %5.1f fps  sim:%3d Hz  seeds:%d  %s ",
        fps, s->sim.fps, N_SEEDS, run_state_label(s));
}

/* Paint the right-aligned status line on row 0 (CP_HUD + A_BOLD). */
static void draw_top_status(const char *status, int term_cols)
{
    int pad = term_cols - (int)strlen(status);
    if (pad < 0) pad = 0;
    attron(COLOR_PAIR(CP_HUD) | A_BOLD);
    mvprintw(0, pad, "%s", status);
    attroff(COLOR_PAIR(CP_HUD) | A_BOLD);
}

/* Paint the bottom action-hint strip listing every interactive key. */
static void draw_bottom_hint(int term_rows)
{
    attron(COLOR_PAIR(CP_HINT) | A_BOLD);
    mvprintw(term_rows - 1, 0,
             " q:quit  spc:pause  r:reset  [/]:Hz ");
    attroff(COLOR_PAIR(CP_HINT) | A_BOLD);
}

/*
 * draw_hud — canonical project HUD.
 *
 *   Pseudocode:
 *     format right-aligned status line
 *     draw_top_status                (row 0,         CP_HUD,  A_BOLD)
 *     draw_bottom_hint               (row R−1,       CP_HINT, A_BOLD)
 */
static void draw_hud(const Scene *s, double fps)
{
    char status[80];
    format_hud_status(s, fps, status, sizeof status);
    draw_top_status(status, s->scene_cols);
    draw_bottom_hint(s->scene_rows);
}

/* frame_render — erase, paint scene, paint HUD, flush via one doupdate. */
static void frame_render(const Scene *s, double fps)
{
    erase();
    scene_draw(s, stdscr);
    draw_hud(s, fps);
    wnoutrefresh(stdscr);
    doupdate();
}

/* ===================================================================== */
/* §8  app                                                                */
/* ===================================================================== */

/*
 * POSIX signal flags — must be file-scope `volatile sig_atomic_t` so
 * the async signal handlers can touch them safely.  Set here, polled
 * by the main loop, cleared by main once acted on.
 */
static volatile sig_atomic_t g_running     = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Toggle the paused flag. */
static void sim_toggle_pause(Scene *s) { s->sim.paused = !s->sim.paused; }

/* Bump simulation fps up by SIM_FPS_STEP, capped at SIM_FPS_MAX. */
static void sim_fps_increase(Scene *s)
{
    s->sim.fps += SIM_FPS_STEP;
    if (s->sim.fps > SIM_FPS_MAX) s->sim.fps = SIM_FPS_MAX;
}

/* Bump simulation fps down by SIM_FPS_STEP, floored at SIM_FPS_MIN. */
static void sim_fps_decrease(Scene *s)
{
    s->sim.fps -= SIM_FPS_STEP;
    if (s->sim.fps < SIM_FPS_MIN) s->sim.fps = SIM_FPS_MIN;
}

/*
 * handle_input — translate one key code into a Scene mutation.
 *
 *   q / Q / ESC  → request quit (clear g_running)
 *   SPACE        → toggle pause
 *   r / R        → re-roll seed positions at current extent
 *   ]            → faster simulation tempo (+SIM_FPS_STEP)
 *   [            → slower simulation tempo (−SIM_FPS_STEP)
 */
static void handle_input(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case KEY_ESC: g_running = 0;            break;
    case ' ':                         sim_toggle_pause(s);      break;
    case 'r': case 'R':               scene_reset_seeds(s);     break;
    case ']':                         sim_fps_increase(s);      break;
    case '[':                         sim_fps_decrease(s);      break;
    default: break;
    }
}

/* On SIGWINCH, tear ncurses down + back up so stdscr matches the new
 * terminal extent, then re-seed the scene at the new size. */
static void handle_resize_if_pending(Scene *s)
{
    if (!g_need_resize) return;
    g_need_resize = 0;
    endwin(); refresh();
    getmaxyx(stdscr, s->scene_rows, s->scene_cols);
    scene_reset_seeds(s);
}

/* ── §8 main-loop helpers ────────────────────────────────────────────── */

/* Install POSIX signal handlers + atexit teardown. */
static void signals_install(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

/* Seed the RNG from the monotonic clock (low 32 bits give enough entropy
 * for a visualisation; no cryptographic claim here). */
static void rng_seed_from_clock(void)
{
    srand((unsigned int)(clock_ns() & 0xFFFFFFFF));
}

/*
 * clamp_dt_to_prevent_spiral — spiral-of-death guard.
 *
 *   If a single frame measures longer than MAX_FRAME_DT_NS (program
 *   suspended, slow terminal, debugger break), cap dt so the
 *   fixed-timestep accumulator never tries to run hundreds of ticks
 *   back-to-back trying to "catch up".  Beyond this cap the simulation
 *   slows down rather than freezing the UI.
 */
static int64_t clamp_dt_to_prevent_spiral(int64_t dt_ns)
{
    return (dt_ns > MAX_FRAME_DT_NS) ? MAX_FRAME_DT_NS : dt_ns;
}

/*
 * advance_fixed_timestep_sim — Fiedler's accumulator pattern.
 *
 *   Pseudocode:
 *     accum += dt
 *     while accum ≥ tick_period:
 *       scene_tick(s, dt_sec)
 *       accum -= tick_period
 *
 *   Keeps physics deterministic regardless of render frame rate.
 *   tick_period is recomputed every frame from sim.fps so the user can
 *   change the tempo mid-run with '[' / ']'.
 */
static void advance_fixed_timestep_sim(
    Scene *s, int64_t *sim_accum_ns, int64_t dt_ns
) {
    int64_t tick_ns = TICK_NS(s->sim.fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;
    *sim_accum_ns += dt_ns;
    while (*sim_accum_ns >= tick_ns) {
        scene_tick(s, dt_sec);
        *sim_accum_ns -= tick_ns;
    }
}

/*
 * sample_fps — running average of recent frame rate, refreshed every
 * FPS_UPDATE_PERIOD_NS so the HUD doesn't flicker.
 *
 *   Pseudocode:
 *     frame_count += 1
 *     accum_ns    += dt_ns
 *     if accum_ns ≥ FPS_UPDATE_PERIOD_NS:
 *       display := frame_count / (accum_ns / 1 s)
 *       reset frame_count + accum_ns
 */
static void sample_fps(
    int64_t dt_ns, int *frame_count, int64_t *accum_ns, double *display
) {
    (*frame_count)++;
    *accum_ns += dt_ns;
    if (*accum_ns >= FPS_UPDATE_PERIOD_NS) {
        *display = (double)(*frame_count)
                 / ((double)(*accum_ns) / (double)NS_PER_SEC);
        *frame_count = 0;
        *accum_ns    = 0;
    }
}

/* Sleep just long enough to keep the render loop at RENDER_TARGET_FPS. */
static void wait_until_next_frame(int64_t frame_start_ns)
{
    int64_t elapsed = clock_ns() - frame_start_ns;
    clock_sleep_ns(RENDER_FRAME_NS - elapsed);
}

/*
 * main — wire signals + ncurses + Scene, then run the canonical loop:
 *
 *   Pseudocode:
 *     seed RNG from clock
 *     install signal handlers + atexit cleanup
 *     bring up terminal (ncurses + colour)
 *     allocate Scene on stack, seed at current terminal extent
 *
 *     while running:
 *       handle resize if pending
 *       dt := clock_ns() − last_frame_start, clamped to MAX_FRAME_DT_NS
 *       advance_fixed_timestep_sim(scene, &sim_accum, dt)
 *       sample_fps(dt, &meter)
 *       frame_render(scene, fps_display)
 *       drain input
 *       wait_until_next_frame(frame_start)
 */
int main(void)
{
    rng_seed_from_clock();
    signals_install();

    Scene scene;
    screen_init(&scene);
    scene_init(&scene, scene.scene_cols, scene.scene_rows);

    int64_t frame_start = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (g_running) {
        if (g_need_resize) {
            handle_resize_if_pending(&scene);
            frame_start = clock_ns();
            sim_accum   = 0;
        }

        int64_t now      = clock_ns();
        int64_t dt_raw   = now - frame_start;
        int64_t dt       = clamp_dt_to_prevent_spiral(dt_raw);
        frame_start      = now;

        advance_fixed_timestep_sim(&scene, &sim_accum, dt);
        sample_fps(dt, &frame_count, &fps_accum, &fps_display);
        frame_render(&scene, fps_display);

        int ch = getch();
        if (ch != ERR) handle_input(&scene, ch);

        wait_until_next_frame(now);
    }

    screen_free();
    return 0;
}
