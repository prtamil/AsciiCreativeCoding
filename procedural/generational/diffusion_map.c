/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * diffusion_map.c  —  Diffusion-Limited Aggregation without symmetry
 *
 * Particles random-walk from a launch circle until they touch the growing
 * cluster and stick.  Without the D6 symmetry constraint of snowflake.c,
 * the aggregate is free to grow in any direction, producing natural fractal
 * branching arms and organic, asymmetric dendrites — the classic DLA
 * morphology described by Witten & Sander (1981).
 *
 * A second "Eden" mode skips the random walk entirely: a random frontier
 * cell (any empty cell adjacent to the cluster) is chosen uniformly and
 * added.  Eden growth is much faster but produces rounder, less fractal
 * shapes (no diffusion bias toward tips).
 *
 * Color is by age_delta = current_frame - cell_join_frame:
 *   newest cells = bright / bold; oldest = dim
 *
 * Themes cycle across 5 color palettes (5 age levels each).
 *
 * Keys:
 *   q / ESC   quit
 *   p / spc   pause / resume
 *   r         reset (clear grid, re-seed)
 *   t / T     next / prev theme
 *   +         more walkers per frame (max 10)
 *   -         fewer walkers per frame (min 1)
 *   n         toggle DLA / Eden mode
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra diffusion_map.c -o diffusion_map -lncurses -lm
 *
 * Sections (cut by concern — see ARCHITECTURE block)
 * --------
 *   §1  config
 *   §2  rng
 *   §3  state              (AggregateGrid + Scene types)
 *   §4  logic              (pure decisions — const AggregateGrid*)
 *   §5  simulation         (advances state; scene_tick = combine point)
 *   §6  effects            (none — fade derived in render)
 *   §7  render             (state → screen, read-only)
 *   §8  performance/delays (clock primitives, frame cap)
 *   §9  platform/app       (ncurses, signals, main loop)
 */

/* ── CONCEPTS ─────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Two aggregation modes in one file:
 *                  DLA (Diffusion-Limited Aggregation, Witten & Sander 1981):
 *                    Particles launched from a ring, random-walk until they
 *                    touch the cluster.  Tip-screening effect: tips extend
 *                    further from the centre and capture walkers preferentially,
 *                    creating fractal branching with D ≈ 1.7 in 2D.
 *                  Eden model: directly attach a random frontier cell.
 *                    No diffusion → no tip screening → compact, rounder shapes
 *                    (D → 2 as cluster grows; no fractal structure at large scales).
 *
 * Math           : DLA fractal dimension D ≈ 1.71 in 2D.
 *                  Cluster radius R ~ N^(1/D) where N = number of particles.
 *                  Comparison between modes in the same code illustrates how
 *                  diffusion (randomness in the approach path) is necessary for
 *                  fractal self-similar morphology.
 *
 * Performance    : DLA walker cost: O(R²) expected random-walk steps per particle
 *                  (hitting probability from radius 2R to R ≈ 1/(log R) in 2D).
 *                  Eden mode: O(frontier size) per particle — much faster.
 *
 * References     : Concepts —
 *                   [1] Witten & Sander, "Diffusion-Limited Aggregation, a Kinetic
 *                       Critical Phenomenon", Phys. Rev. Lett. 47, 1400 (1981).
 *                       The founding DLA paper.
 *                   [2] Witten & Sander, "Diffusion-limited aggregation",
 *                       Phys. Rev. B 27, 5686 (1983). Full theory; D ≈ 1.71.
 *                   [3] Meakin, "Fractals, Scaling and Growth Far from Equilibrium"
 *                       (Cambridge, 1998). Definitive treatment of DLA & Eden, and
 *                       the launch-circle / kill-radius walker algorithm used here.
 *                   [4] Vicsek, "Fractal Growth Phenomena", 2nd ed. (World
 *                       Scientific, 1992). Broad survey of DLA, Eden, and kin.
 *                   [5] Eden, "A two-dimensional growth process", Proc. 4th Berkeley
 *                       Symp. Math. Stat. Prob. (1961). Origin of the Eden model.
 *                   [6] Mandelbrot, "The Fractal Geometry of Nature" (Freeman,
 *                       1982). Fractal dimension as a measure of branching.
 *                   [7] Bourke, "Constructing a diffusion limited aggregate",
 *                       paulbourke.net/fractals/dla. Practical implementation recipe.
 *                  Rendering —
 *                   [8] Bourke, "Character representation of grey scale images",
 *                       paulbourke.net/dataformats/asciiart. The intensity→glyph
 *                       ramp behind age_to_char()'s @ # + : . mapping.
 *                   [9] Ben-Halim & Raymond, "Writing Programs with NCURSES"
 *                       (NCURSES HOWTO). Color pairs, erase/refresh, the terminal
 *                       rendering model used in §7 and §9.
 * ─────────────────────────────────────────────────────────────────────── */

/* ── ARCHITECTURE (layers + types) ───────────────────────────────────────
 *
 * One real domain type — AggregateGrid — carries the simulated object AND its
 * growth clock (joined[] timestamps are meaningless without frame, so they
 * live together).  Everything else the user touches is a handful of loose
 * fields on Scene; none forms a concept a textbook would name, so none gets a
 * struct of its own.  Each tick, scene_tick() (§5) is the ONE state advance.
 *
 *   LAYER          §   READS / MUTATES                    KEY FUNCTIONS
 *   ------------  --   ------------------------------     -------------------------
 *   CONFIG         1   (constants only)                  —
 *   RNG            2   g_lcg                             lcg_f, lcg_i, seed_rng
 *   STATE          3   defines AggregateGrid, Scene      (types + the one Scene)
 *   LOGIC          4   const AggregateGrid* (read)       aggregate_has_neighbor, cell_age,
 *                                                        chebyshev_from_centre,
 *                                                        collect_frontier, age_to_pair/char
 *   SIMULATION     5   AggregateGrid* / Scene* (mutate)  aggregate_reset/add_cell, launch_on_ring,
 *                                                        random_walk_step, dla_step, eden_step,
 *                                                        scene_tick <-- combine point
 *   EFFECTS        6   (none — see note)                 —
 *   RENDER         7   const sub-types → terminal        aggregate_draw, color_init_theme,
 *                                                        screen_draw_hud, render_frame
 *   PERF/DELAYS    8   (local timing only)               clock_ns, clock_sleep_ns
 *   PLATFORM/APP   9   Scene*, g_scr_*, signal flags     main, app_init, app_resize,
 *                                                        app_handle_key, install_signals
 *
 * Contracts:
 *   (a) Signature convention IN FORCE: a pure read takes a const pointer
 *       (const AggregateGrid*); a mutator takes a non-const pointer. Workers take
 *       the narrowest sub-type; only whole-scene orchestrators — scene_init,
 *       scene_tick, and the app_* drivers (app_init / app_resize / app_handle_key)
 *       — take Scene*, so hanging data off Scene never re-couples the layers.
 *   (b) LOGIC (§4) never mutates and does no I/O; RENDER (§7) writes only to
 *       the terminal. Neither can corrupt the other's inputs.
 *   (c) scene_tick() is the single per-tick combine point. reset/resize/key in
 *       §9 also mutate state, but are USER EVENTS, not part of the tick.
 *   (d) EFFECTS is empty: the age fade is derived at RENDER time from
 *       joined[][] (a SIMULATION field), not stored as cosmetic state.
 * ──────────────────────────────────────────────────────────────────────── */

#define _POSIX_C_SOURCE 200809L

#include <math.h>
#include <ncurses.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

/* ===================================================================== */
/* §1  config                                                             */
/* ===================================================================== */

enum {
    ROWS_MAX        =  80,
    COLS_MAX        = 300,

    RENDER_FPS      =  30,

    WALKER_MIN      =   1,
    WALKER_DEFAULT  =   3,
    WALKER_MAX      =  10,

    MAX_STEPS       = 500,   /* max random-walk steps before aborting     */
    LAUNCH_PAD      =   3,   /* launch circle = radius + LAUNCH_PAD       */

    N_THEMES        =   5,
    N_AGE_LEVELS    =   5,   /* color pairs CP_A0 … CP_A4                 */
    CP_HUD          =   1,   /* data line (row 0)                         */
    CP_A0           =   2,   /* newest cells (index offset 0)             */
    /* CP_A1 = 3, CP_A2 = 4, CP_A3 = 5, CP_A4 = 6                        */
    CP_HINT         =   7,   /* action line (bottom row)                  */

    FPS_UPDATE_MS   = 500,
    MAX_FRAME_MS    = 200,   /* clamp dt to this — avoids the spiral-of-death
                                after a stall (slow terminal / suspend)    */
};

/* age_delta thresholds: 0-5, 6-20, 21-80, 81-300, 300+ */
static const int AGE_THRESH[N_AGE_LEVELS] = { 5, 20, 80, 300, INT32_MAX };

#define RENDER_NS   (1000000000LL / RENDER_FPS)
#define NS_PER_SEC  1000000000LL
#define NS_PER_MS   1000000LL

/* one full turn in radians; kept at this exact literal (not 2·M_PI) so the
   RNG-driven launch angles — and thus the exact cluster — are unchanged */
static const float TWO_PI = 6.28318f;

/* ===================================================================== */
/* §2  rng                                                               */
/* ===================================================================== */

static uint32_t g_lcg;

static float lcg_f(void)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (float)(g_lcg >> 8) / (float)(1u << 24);
}

/* integer in [0, n) */
static int lcg_i(int n)
{
    g_lcg = g_lcg * 1664525u + 1013904223u;
    return (int)((g_lcg >> 8) % (uint32_t)n);
}

/* Seed the generator from the monotonic clock so every run differs */
static void seed_rng(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    g_lcg = (uint32_t)(ts.tv_nsec ^ ts.tv_sec);
}

/* ===================================================================== */
/* §3  state types                                                        */
/* ===================================================================== */

/*
 * AggregateGrid — the cluster grown by diffusion-limited aggregation, stored on
 * a square lattice (one cell per terminal character).
 *
 * WHY a grid (not just a point list): on-lattice DLA is *defined* on a lattice —
 * particles occupy discrete sites, "stick" by 4-neighbor adjacency, and the
 * cluster's reach is measured in lattice steps. The grid is the model, not an
 * implementation detail (Witten & Sander 1981 [1]; Meakin 1998 [3]).
 *
 * WHY the clock lives here: each cell records the frame it joined (joined[]),
 * and RENDER fades a cell by age = frame - joined. Those timestamps are
 * meaningless without the clock that stamped them, so `frame` belongs WITH the
 * grid rather than in a separate "lifecycle" struct.
 *
 * All buffers are fixed-size — the hot path never allocates (project memory rule).
 */
typedef struct {
    /* the lattice — SIMULATION writes, LOGIC + RENDER only read */
    uint8_t  cell  [ROWS_MAX][COLS_MAX];  /* membership: 0 = empty, 1 = aggregate.
                                             a compact byte grid that memset() clears
                                             in one shot on reset.                  */
    uint16_t joined[ROWS_MAX][COLS_MAX];  /* frame each cell stuck, low 16 bits.
                                             uint16_t halves memory vs int; past
                                             ~65k frames the stored value wraps, so
                                             an age computed across a wrap is only
                                             approximate — harmless, as such cells
                                             already sit in the oldest tier. Drives
                                             the age→colour/glyph ramp (Bourke [8]). */

    /* geometry — emerges from growth; sizes the walker launch ring */
    int rows, cols;   /* ACTIVE extent (≤ ROWS_MAX/COLS_MAX); taken from the
                         terminal size so the cluster fills the visible screen.     */
    int cx, cy;       /* seed centre. Growth starts from one central cell so the
                         aggregate can branch radially in every direction.          */
    int radius;       /* largest Chebyshev distance max(|dr|,|dc|) of any cell from
                         the centre. Chebyshev — not Euclidean — so the launch ring
                         and kill radius are cheap box bounds: walkers spawn at
                         radius+LAUNCH_PAD and die past ~2·radius (the launch-circle
                         / kill-radius scheme of Meakin [3]). Grows as tips extend. */
    int size;         /* occupied-cell count = N particles. Feeds the HUD and is the
                         N in cluster radius R ~ N^(1/D), D≈1.71 (Vicsek [4];
                         Mandelbrot [6]).                                            */

    /* growth clock — the time axis joined[] is measured against */
    int frame;        /* monotonically increasing tick counter, reset to 1 on clear.
                         age_delta = frame - joined, clamped ≥ 0.                    */
} AggregateGrid;

/*
 * Scene — the whole program in one place, read like a table of contents.
 *
 * Only `aggregate` earns a type of its own; the remaining fields are loose knobs
 * that no aggregation textbook would name as a concept, so they stay flat —
 * grouped by what they BELONG to, not by the key that happens to toggle them
 * (hence `theme`, a render choice, sits apart from the simulation knobs).
 */
typedef struct {
    AggregateGrid aggregate;  /* WHAT is simulated, plus its growth clock.        */

    /* HOW the user drives the simulation (SIMULATION knobs) */
    bool eden_mode;  /* which growth model is running:
                        false = DLA — diffusion-limited random walk; tip-screening
                                yields fractal dendrites, D≈1.71 (Witten&Sander [1]).
                        true  = Eden — attach a uniformly-random frontier cell; no
                                diffusion → no screening → compact blob, D→2
                                (Eden 1961 [5]).
                        Both modes grow on the SAME grid so the morphologies can be
                        compared directly — that contrast is the point of the demo. */
    int  n_walkers;  /* particles deposited per tick (WALKER_MIN..WALKER_MAX). A
                        throughput knob only — it sets how FAST the cluster grows,
                        not its shape. Bounded because each DLA particle costs an
                        O(R²) random walk, so large values stall the frame rate.    */
    bool paused;     /* run-state: true freezes growth while RENDER keeps drawing,
                        so the current aggregate stays on screen for inspection.    */

    /* RENDER: which palette — a view choice, deliberately kept off the sim knobs
       (it changes how cells are coloured, nothing about how they grow). */
    int  theme;      /* index into THEME_FG / THEME_NAME, 0..N_THEMES-1.            */
} Scene;

static Scene g_scene;

/* terminal geometry — owned by PLATFORM (§9), read by RENDER (§7) */
static int g_scr_rows, g_scr_cols;

/* ===================================================================== */
/* §4  logic  —  pure decisions: no mutation, no I/O, readable in isolation */
/* ===================================================================== */

/* True if any 4-neighbor of (r,c) is part of the aggregate */
static bool aggregate_has_neighbor(const AggregateGrid *agg, int r, int c)
{
    if (r > 0              && agg->cell[r-1][c]) return true;
    if (r < agg->rows - 1  && agg->cell[r+1][c]) return true;
    if (c > 0              && agg->cell[r][c-1]) return true;
    if (c < agg->cols - 1  && agg->cell[r][c+1]) return true;
    return false;
}

/* Map age_delta to color pair index (CP_A0 … CP_A4) */
static int age_to_pair(int age_delta)
{
    for (int i = 0; i < N_AGE_LEVELS; i++)
        if (age_delta <= AGE_THRESH[i])
            return CP_A0 + i;
    return CP_A0 + N_AGE_LEVELS - 1;
}

/* Map age_delta to display character */
static chtype age_to_char(int age_delta)
{
    if (age_delta <= AGE_THRESH[0]) return (chtype)'@';
    if (age_delta <= AGE_THRESH[1]) return (chtype)'#';
    if (age_delta <= AGE_THRESH[2]) return (chtype)'+';
    if (age_delta <= AGE_THRESH[3]) return (chtype)':';
    return (chtype)'.';
}

/* Chebyshev (chessboard) distance from the seed centre to (r,c). The launch
 * ring and kill radius use this metric, not Euclidean, so they stay cheap
 * square box-bounds (Meakin [3]). */
static int chebyshev_from_centre(const AggregateGrid *agg, int r, int c)
{
    int dr = abs(r - agg->cy);
    int dc = abs(c - agg->cx);
    return (dr > dc) ? dr : dc;
}

/* Age of a cell in frames: how long ago it joined, clamped ≥ 0 (a join stamp
 * can read "ahead" only after the 16-bit wrap; see the joined[] note). */
static int cell_age(const AggregateGrid *agg, int r, int c)
{
    int age = agg->frame - (int)agg->joined[r][c];
    return (age < 0) ? 0 : age;
}

/* Gather the growth frontier — every empty cell touching the aggregate — into
 * the caller's arrays; returns how many were found. */
static int collect_frontier(const AggregateGrid *agg, int *fr, int *fc)
{
    int n = 0;
    for (int r = 0; r < agg->rows; r++) {
        for (int c = 0; c < agg->cols; c++) {
            if (agg->cell[r][c]) continue;
            if (aggregate_has_neighbor(agg, r, c)) {
                fr[n] = r;
                fc[n] = c;
                n++;
            }
        }
    }
    return n;
}

/* ===================================================================== */
/* §5  simulation  —  the only layer that advances state                  */
/* ===================================================================== */

static const int DR4[4] = { -1, 1,  0, 0 };
static const int DC4[4] = {  0, 0, -1, 1 };

static void aggregate_set_size(AggregateGrid *agg, int cols, int rows)
{
    agg->cols = (cols < COLS_MAX) ? cols : COLS_MAX;
    agg->rows = (rows < ROWS_MAX) ? rows : ROWS_MAX;
}

/* Clear the lattice, restart the clock, and re-seed a single cell at the centre */
static void aggregate_reset(AggregateGrid *agg)
{
    memset(agg->cell,   0, sizeof agg->cell);
    memset(agg->joined, 0, sizeof agg->joined);
    agg->cx     = agg->cols / 2;
    agg->cy     = agg->rows / 2;
    agg->radius = 0;
    agg->frame  = 1;
    agg->size   = 0;

    /* Seed: single cell at center, stamped at the initial frame */
    agg->cell  [agg->cy][agg->cx] = 1;
    agg->joined[agg->cy][agg->cx] = 1;
    agg->size  = 1;
}

/* Add cell (r,c) to the aggregate, stamping it with the current frame */
static void aggregate_add_cell(AggregateGrid *agg, int r, int c)
{
    agg->cell  [r][c] = 1;
    agg->joined[r][c] = (uint16_t)(agg->frame & 0xFFFF);
    agg->size++;

    /* extend the tracked cluster extent if this cell reaches further out */
    int reach = chebyshev_from_centre(agg, r, c);
    if (reach > agg->radius) agg->radius = reach;
}

/* Spawn a walker at a random point on the launch ring (radius + LAUNCH_PAD),
 * clamped inside the grid. The 0.5 on the row term squashes the ring vertically
 * to correct for ~2:1 terminal character aspect, so it reads as a circle. */
static void launch_on_ring(const AggregateGrid *agg, int *r, int *c)
{
    float angle = lcg_f() * TWO_PI;
    float dist  = (float)(agg->radius + LAUNCH_PAD);

    *r = agg->cy + (int)roundf(dist * sinf(angle) * 0.5f);
    *c = agg->cx + (int)roundf(dist * cosf(angle));

    if (*r < 0) *r = 0;
    if (*r >= agg->rows) *r = agg->rows - 1;
    if (*c < 0) *c = 0;
    if (*c >= agg->cols) *c = agg->cols - 1;
}

/* Move (r,c) one unit in a uniformly random 4-direction (one random-walk step) */
static void random_walk_step(int *r, int *c)
{
    int dir = lcg_i(4);
    *r += DR4[dir];
    *c += DC4[dir];
}

/*
 * dla_step — advance growth by one DLA particle:
 *   1. Launch on the ring around the cluster.
 *   2. Random-walk until it sticks, wanders past the kill radius, or
 *      exhausts MAX_STEPS.
 *   3. Stick where it first touches the aggregate.
 * Returns true if the particle stuck.
 */
static bool dla_step(AggregateGrid *agg)
{
    int r, c;
    launch_on_ring(agg, &r, &c);

    int kill_radius = agg->radius * 2 + LAUNCH_PAD * 3;

    for (int step = 0; step < MAX_STEPS; step++) {
        if (r < 0 || r >= agg->rows || c < 0 || c >= agg->cols)
            return false;                                  /* walked off-grid */

        if (chebyshev_from_centre(agg, r, c) > kill_radius)
            return false;                                  /* wandered too far — abandon */

        if (aggregate_has_neighbor(agg, r, c)) {
            if (agg->cell[r][c] == 0) {
                aggregate_add_cell(agg, r, c);             /* touched cluster → stick */
                return true;
            }
            /* already occupied — keep walking */
        }

        random_walk_step(&r, &c);
    }
    return false;                                          /* never stuck */
}

/*
 * eden_step — advance growth by one Eden particle: pick a uniformly random cell
 * from the growth frontier and add it. No diffusion, so growth has no tip bias.
 * Returns true if a cell was added.
 */
static bool eden_step(AggregateGrid *agg)
{
    /* scratch frontier buffers — static so they are not re-allocated per call */
    static int fr[ROWS_MAX * COLS_MAX];
    static int fc[ROWS_MAX * COLS_MAX];

    int n = collect_frontier(agg, fr, fc);
    if (n == 0) return false;                  /* aggregate fully enclosed */

    int idx = lcg_i(n);                        /* uniform pick over the frontier */
    aggregate_add_cell(agg, fr[idx], fc[idx]);
    return true;
}

/* Orchestrator: set the user-tunable knobs to their defaults */
static void scene_init(Scene *s)
{
    s->eden_mode = false;
    s->n_walkers = WALKER_DEFAULT;
    s->paused    = false;
}

/*
 * scene_tick — THE single per-tick combine point (contract c).
 * Order each tick: (1) DELAY gate (paused), (2) advance the growth clock,
 * (3) SIMULATION step (DLA or Eden). LOGIC is consulted inside the step calls;
 * RENDER and EFFECTS are never touched here.
 */
static void scene_tick(Scene *s)
{
    if (s->paused) return;

    s->aggregate.frame++;

    if (s->eden_mode) {
        for (int i = 0; i < s->n_walkers; i++)
            eden_step(&s->aggregate);
    } else {
        for (int i = 0; i < s->n_walkers; i++)
            dla_step(&s->aggregate);
    }
}

/* ===================================================================== */
/* §6  effects                                                            */
/* ===================================================================== */

/*
 * No EFFECTS layer.  There is no cosmetic-only state (no glow/trail/flash
 * buffer).  The bright→dim age fade is derived at RENDER time (§7
 * aggregate_draw) from age_delta = frame - joined[r][c]; since it is computed,
 * not stored, there is nothing for an EFFECTS layer to own or mutate.
 */

/* ===================================================================== */
/* §7  render  —  state → screen, reads only, never mutates state         */
/* ===================================================================== */

/*
 * 5 themes × 5 age levels (newest → oldest):
 *
 *  Coral:  231 226 208 196 124  white→yellow→orange→red→dark red
 *  Ice:    231 123  51  27  17  white→cyan→blue→dark
 *  Lava:   231 220 208 196  88  white→yellow→orange→red→dark
 *  Plasma: 231 213 165  93  54  white→pink→purple→dark
 *  Mono:   255 251 244 238 235  bright→dark grays
 */
static const short THEME_FG[N_THEMES][N_AGE_LEVELS] = {
    { 231, 226, 208, 196, 124 },  /* Coral  */
    { 231, 123,  51,  27,  17 },  /* Ice    */
    { 231, 220, 208, 196,  88 },  /* Lava   */
    { 231, 213, 165,  93,  54 },  /* Plasma */
    { 255, 251, 244, 238, 235 },  /* Mono   */
};

static const char *THEME_NAME[N_THEMES] = {
    "Coral", "Ice", "Lava", "Plasma", "Mono"
};

/* Fallback 8-color equivalents for each age level */
static const short THEME_FG8[N_AGE_LEVELS] = {
    COLOR_WHITE, COLOR_CYAN, COLOR_CYAN, COLOR_RED, COLOR_RED
};

static void color_init_theme(int theme)
{
    /* HUD pairs: bright yellow data line, bright cyan action line */
    if (COLORS >= 256) {
        init_pair(CP_HUD,  226, COLOR_BLACK);
        init_pair(CP_HINT,  51, COLOR_BLACK);
    } else {
        init_pair(CP_HUD,  COLOR_YELLOW, COLOR_BLACK);
        init_pair(CP_HINT, COLOR_CYAN,   COLOR_BLACK);
    }

    /* Age level pairs CP_A0 … CP_A4 */
    for (int i = 0; i < N_AGE_LEVELS; i++) {
        if (COLORS >= 256)
            init_pair((short)(CP_A0 + i),
                      THEME_FG[theme][i], COLOR_BLACK);
        else
            init_pair((short)(CP_A0 + i),
                      THEME_FG8[i], COLOR_BLACK);
    }
}

static void aggregate_draw(const AggregateGrid *agg)
{
    for (int r = 0; r < agg->rows; r++) {
        for (int c = 0; c < agg->cols; c++) {
            if (!agg->cell[r][c]) continue;

            int age = cell_age(agg, r, c);
            int pair  = age_to_pair(age);
            chtype ch = age_to_char(age);
            attr_t attr = (attr_t)COLOR_PAIR(pair);
            if (age <= AGE_THRESH[0]) attr |= A_BOLD;     /* newest tier glows bold */

            attron(attr);
            mvaddch(r, c, ch);
            attroff(attr);
        }
    }
}

/* Draw a HUD line at row, clipped to the terminal width so it never overflows */
static void hud_line(int row, int pair, const char *s)
{
    char buf[128];
    int n = snprintf(buf, sizeof buf, "%s", s);
    if (n > g_scr_cols) buf[g_scr_cols] = '\0';   /* clip to screen width */

    attron(COLOR_PAIR(pair) | A_BOLD);
    mvprintw(row, 0, "%s", buf);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

/* Status line summarises the whole scene; read-only, so const Scene* is fine */
static void screen_draw_hud(const Scene *s, double fps)
{
    /* Top: live data */
    char data[128];
    snprintf(data, sizeof data,
             " %s | %s | walkers:%d | size:%d | %.1f fps ",
             s->eden_mode ? "Eden" : "DLA",
             THEME_NAME[s->theme],
             s->n_walkers,
             s->aggregate.size,
             fps);
    hud_line(0, CP_HUD, data);

    /* Bottom: interactive keys */
    hud_line(g_scr_rows - 1, CP_HINT,
             " q:quit  spc:pause  r:reset  t/T:theme  +/-:walkers  n:mode ");
}

static void screen_present(void)
{
    wnoutrefresh(stdscr);
    doupdate();
}

/* Compose one frame: clear, draw the aggregate, overlay the HUD, flush */
static void render_frame(const Scene *s, double fps)
{
    erase();
    aggregate_draw(&s->aggregate);
    screen_draw_hud(s, fps);
    screen_present();
}

/* ===================================================================== */
/* §8  performance / delays  —  timing primitives + frame cap             */
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
/* §9  platform / app  —  ncurses setup, signals, input, main loop        */
/* ===================================================================== */

static volatile sig_atomic_t g_running    = 1;
static volatile sig_atomic_t g_need_resize = 0;

static void on_exit_signal(int sig)   { (void)sig; g_running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_need_resize = 1; }
static void cleanup(void)             { endwin(); }

/* Restore the terminal on exit and route quit/resize signals to flags */
static void install_signals(void)
{
    atexit(cleanup);
    signal(SIGINT,   on_exit_signal);
    signal(SIGTERM,  on_exit_signal);
    signal(SIGWINCH, on_resize_signal);
}

static void screen_init_ncurses(int theme)
{
    initscr();
    noecho();
    cbreak();
    curs_set(0);
    nodelay(stdscr, TRUE);
    keypad(stdscr, TRUE);
    typeahead(-1);
    start_color();
    color_init_theme(theme);
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
}

static void app_resize(Scene *s)
{
    endwin();
    refresh();
    getmaxyx(stdscr, g_scr_rows, g_scr_cols);
    aggregate_set_size(&s->aggregate, g_scr_cols, g_scr_rows - 1);
    aggregate_reset(&s->aggregate);
    g_need_resize = 0;
}

static bool app_handle_key(Scene *s, int ch)
{
    switch (ch) {
    case 'q': case 'Q': case 27:
        return false;

    case 'p': case 'P': case ' ':
        s->paused = !s->paused;
        break;

    case 'r': case 'R':
        aggregate_reset(&s->aggregate);
        break;

    case 't':
        s->theme = (s->theme + 1) % N_THEMES;
        color_init_theme(s->theme);
        break;

    case 'T':
        s->theme = (s->theme + N_THEMES - 1) % N_THEMES;
        color_init_theme(s->theme);
        break;

    case '+': case '=':
        if (s->n_walkers < WALKER_MAX)
            s->n_walkers++;
        break;

    case '-':
        if (s->n_walkers > WALKER_MIN)
            s->n_walkers--;
        break;

    case 'n': case 'N':
        s->eden_mode = !s->eden_mode;
        break;

    default: break;
    }
    return true;
}

/* Bring up the terminal and seed the simulation to its starting state */
static void app_init(Scene *s)
{
    s->theme = 0;
    screen_init_ncurses(s->theme);
    aggregate_set_size(&s->aggregate, g_scr_cols, g_scr_rows - 1);
    aggregate_reset(&s->aggregate);
    scene_init(s);
}

int main(void)
{
    seed_rng();
    install_signals();
    app_init(&g_scene);

    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;
    int64_t last_frame  = clock_ns();

    while (g_running) {

        if (g_need_resize) {
            app_resize(&g_scene);
            scene_init(&g_scene);
            last_frame = clock_ns();
            fps_accum  = 0;
            frame_count = 0;
        }

        scene_tick(&g_scene);

        /* measure frame time and refresh the FPS readout twice a second */
        int64_t now = clock_ns();
        int64_t dt  = now - last_frame;
        last_frame  = now;
        if (dt > MAX_FRAME_MS * NS_PER_MS) dt = MAX_FRAME_MS * NS_PER_MS;
        fps_accum += dt;
        frame_count++;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            fps_accum   = 0;
            frame_count = 0;
        }

        render_frame(&g_scene, fps_display);

        /* read one key; a quit request ends the loop */
        int ch = getch();
        if (ch != ERR && !app_handle_key(&g_scene, ch))
            g_running = 0;

        /* throttle to RENDER_FPS: sleep off whatever time the frame left */
        int64_t elapsed = clock_ns() - last_frame + dt;
        clock_sleep_ns(RENDER_NS - elapsed);
    }

    endwin();
    return 0;
}
