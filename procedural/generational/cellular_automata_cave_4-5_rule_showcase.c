/* Copyright (c) 2026 Tamilselvan R  SPDX-License-Identifier: MIT */
/*
 * cellular_automata_cave_4-5_rule_showcase.c
 *   — Cave generation by the cellular-automata 4-5 rule, animated.
 *
 * DEMO: A solid grid of random salt-and-pepper noise (~45% walls)
 *       passes through 4 iterations of a "smoothing" cellular
 *       automaton. Each iteration is animated cell-by-cell as a
 *       sweep from top-left to bottom-right; cells already processed
 *       show their new state, cells not yet processed show their
 *       previous state, so you see a clear wavefront crawl across
 *       the map. Cells that flip (wall→floor or vice versa) flash in
 *       the theme's accent colour. After 4 sweeps the noise has
 *       organised itself into rounded, organic-looking caverns. HOLD
 *       on the cave; supernova reset; loop forever with a fresh seed.
 *
 * Study alongside: ./drunkards_walk_cave_showcase.c — another way to
 *       carve organic caves. Drunkard's walk carves bottom-up
 *       (walkers eat through walls one cell at a time); CA caves
 *       carve top-down (start with noise, smooth it). The visual is
 *       very different: drunkard's walk grows from a centre,
 *       cellular automata reorganises the entire grid in parallel.
 *
 * Section map:
 *   §1 config   — map size, fill ratio, iterations, themes, glow rates
 *   §2 clock    — monotonic timer + sleep
 *   §3 color    — HUD reserved + 10 cave themes
 *   §5 ca       — CACave struct, neighbour counter, sweep step
 *   §6 scene    — ITERATING / HOLD state machine
 *   §7 screen   — ASCII render: wavefront sweep, # walls, . floors
 *   §8 app      — signals, resize, main loop
 *
 * Keys:
 *   q / ESC    quit
 *   space      pause / resume
 *   r          reset (preserves theme)
 *   t / T      next / previous theme
 *   + / =      faster sweep (more cells/tick)
 *   -          slower sweep
 *   ] / [      raise / lower tick Hz
 *
 * Build:
 *   gcc -std=c11 -O2 -Wall -Wextra cellular_automata_cave_4-5_rule_showcase.c \
 *       -o ca_cave -lncurses -lm
 */

/* ── CONCEPTS ──────────────────────────────────────────────────────────── *
 *
 * Algorithm      : Cellular-automata cave generation, B5678/S45678 — also
 *                  known as the "4-5 rule" because a cell:
 *                    becomes WALL  if ≥ 5 of its 8 Moore neighbours are walls
 *                    becomes FLOOR if ≤ 4 of its 8 Moore neighbours are walls
 *                  Out-of-grid neighbours count as WALL — this keeps the
 *                  map border solid and prevents caves from leaking out.
 *
 *                  Procedure:
 *                    1. Randomly fill ~45% of cells as WALL.
 *                    2. Apply the 4-5 rule for ~4 iterations using a
 *                       double buffer (read from tiles[], write to
 *                       next_tiles[], swap at end of pass).
 *                    3. The resulting grid has rounded, organic
 *                       caverns where the noise stabilised — corridors,
 *                       chambers, isolated islands.
 *                  No corners, no straight lines — that's the property
 *                  cellular automata excel at.
 *
 *                  Each iteration smooths the previous: tiny noise
 *                  islands fill in, ragged edges round off, and after
 *                  ~4 passes the cave shape stops changing meaningfully.
 *                  Running too many iterations dissolves small features
 *                  back into uniform stone or open space.
 *
 * Data-structure : Two flat tile grids (current + double-buffer), one
 *                  bool array tracking "is this cell at the wavefront",
 *                  per-cell change_glow + supernova_glow floats. The
 *                  sweep_idx counter walks through cells in row-major
 *                  order during each iteration. No allocation post-init.
 *
 * Rendering      : ASCII only. The display rule is the wavefront trick:
 *                  cells with index < sweep_idx render their NEW state
 *                  (next_tiles); cells ≥ sweep_idx render their OLD state
 *                  (tiles). The user sees a sharp boundary crawl across
 *                  the map row by row as each iteration progresses.
 *                  Cells that FLIP between states get change_glow
 *                  painted; that decays over ~0.4 s as a bright accent
 *                  flash. '#' walls render only adjacent to '.' floors
 *                  for the standard roguelike look.
 *
 * Performance    : O(N · I) where N is cells and I is iterations (~4).
 *                  We throttle to ops_per_tick (default 128) so the full
 *                  generation plays out over ~5–8 s on a typical
 *                  200×53 map.
 *
 * References     : • RogueBasin — "Cellular Automata Method for Generating
 *                    Random Cave-Like Levels":
 *                    http://www.roguebasin.com/index.php?title=Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels
 *                  • Stephenson, Wesley M. — "Procedural Cave Generation
 *                    using Cellular Automata":
 *                    https://web.archive.org/web/20180614050304/http://pcg.wikidot.com/pcg-algorithm:cellular-automata
 *                  • Wikipedia — "Cellular automaton":
 *                    https://en.wikipedia.org/wiki/Cellular_automaton
 *                  • Compare — Conway's Game of Life, B3/S23 — same
 *                    Moore-neighbourhood machinery, different rule.
 *
 * ─────────────────────────────────────────────────────────────────────── */

/* ── MENTAL MODEL ─────────────────────────────────────────────────────── *
 *
 * CORE IDEA
 * ─────────
 * Caves don't have to be DESIGNED. They can be GROWN by repeatedly
 * applying the same simple smoothing rule to noise. The 4-5 rule is
 * a "majority filter" — every cell looks at its 8 neighbours and
 * agrees with the majority. Run that on random noise a few times
 * and the noise organises itself into smooth blob-like regions.
 * The blobs are your caves.
 *
 * HOW TO THINK ABOUT IT
 * ─────────────────────
 * Picture a salt-and-pepper photograph. Now apply a "smoothing"
 * filter that says: each pixel becomes the colour the majority of
 * its neighbours are. After one pass, the speckles start clumping.
 * After two passes, the clumps have rounder edges. After four
 * passes, the noise has fully reorganised into smooth, irregular
 * regions. Identical math drives both image-processing median
 * filters and cellular automaton cave generators.
 *
 * What you see on screen, as the wavefront crawls:
 *   • TOP-LEFT (already processed): the NEW state for this iteration.
 *   • BOTTOM-RIGHT (not yet processed): the OLD state from the
 *     previous iteration.
 *   • The line between them is the SWEEP — typically a partial row.
 *   • Cells that FLIPPED between iterations flash bright; cells that
 *     remained the same show no flash.
 *
 * ALGORITHM IN STEPS
 * ──────────────────
 *  1. INIT. Allocate two grids: tiles[] and next_tiles[]. For each
 *     cell, randomly set tiles[i] = WALL with probability FILL_RATIO
 *     (≈ 0.45). Mark iteration = 0 and sweep_idx = 0.
 *  2. SWEEP STEP (one cell):
 *     a. Look at tiles[sweep_idx]'s 8 Moore neighbours via tiles[]
 *        (NOT next_tiles[] — we're using a double buffer).
 *     b. Count wall neighbours; out-of-grid neighbours count as WALL.
 *     c. next_tiles[sweep_idx] = (count ≥ 5) ? WALL : FLOOR.
 *     d. If next_tiles[sweep_idx] != tiles[sweep_idx], paint
 *        change_glow at this cell.
 *     e. Increment sweep_idx.
 *  3. END-OF-ITERATION: when sweep_idx == total_cells, copy
 *     next_tiles → tiles, reset sweep_idx = 0, increment iteration.
 *  4. Goto 2 until iteration == N_ITERATIONS.
 *  5. HOLD on the finished cave for HOLD_SECONDS; supernova reset
 *     and goto 1 with a new random seed.
 *
 * KEY FORMULAS
 * ────────────
 *  Tile flat index               : idx = y · w + x
 *  Wall-neighbour count          : N = Σ_{(dx,dy) ≠ (0,0)} W(x+dx, y+dy)
 *                                  where W(.) returns 1 for WALL or oob,
 *                                  0 for FLOOR.
 *  Rule (4-5)                    : next = (N ≥ 5) ? WALL : FLOOR
 *  Display state during sweep    : show next_tiles[idx] if idx < sweep_idx
 *                                  else show tiles[idx]
 *  Glow decay (per frame)        : glow *= exp(-rate · dt)
 *
 * EDGE CASES TO WATCH
 * ───────────────────
 *  • DOUBLE BUFFERING IS ESSENTIAL. If you read and write the same
 *    grid during the sweep, later cells in the sweep see the NEW
 *    values of earlier cells, which biases the result toward the
 *    sweep order (top-left smoother than bottom-right). Always read
 *    from tiles[], write to next_tiles[], swap at end.
 *
 *  • OUT-OF-BOUNDS = WALL. Treating them as WALL keeps the border
 *    solid (no caves leak off the map). Treating them as FLOOR or
 *    skipping them produces caves that touch the edge — usually not
 *    what you want for a playable level.
 *
 *  • FILL_RATIO TUNING. Below ~0.40, the iterations dissolve the
 *    walls and you get a mostly-open map. Above ~0.55, walls
 *    dominate and the caves shrink to thin corridors. 0.45 is the
 *    common sweet spot for "lots of cave, with some structure".
 *
 *  • ITERATION COUNT. Below 3, the result is still noisy. Above 6,
 *    small features dissolve. 4 is the textbook value. Increase
 *    only if you want very smooth, blob-only caves with no detail.
 *
 *  • DISCONNECTED COMPONENTS. CA caves are NOT guaranteed connected
 *    — you'll often see isolated chambers. Real roguelikes either
 *    add a flood-fill pass to keep only the largest region, or run
 *    a corridor-carving pass to connect components. We don't here:
 *    showing the raw output is the showcase.
 *
 *  • THE WAVEFRONT IS A VISUAL TRICK. The algorithm itself does
 *    NOT process cells in row-major order conceptually — that's just
 *    how we choose to schedule the work. Any traversal order yields
 *    the same final state because the double buffer separates reads
 *    from writes.
 *
 * HOW TO VERIFY
 * ─────────────
 *  • Initial state: ~45% of cells are WALL (matches FILL_RATIO).
 *    A randomly-seeded grid should look like uniform salt-and-pepper.
 *  • After iteration 1: small isolated FLOOR cells get filled in,
 *    small isolated WALL cells get carved away. Visible "grain
 *    coalescing" effect.
 *  • After iteration 4: the cave shape stops changing meaningfully.
 *    Running a 5th iteration should flip very few cells (low
 *    change_glow density). If many cells still flip on the 5th
 *    iteration, the rule or buffer-swap is buggy.
 *  • Wall fraction at convergence is typically 50–55% — slightly
 *    higher than the initial seed because oob-as-wall biases the
 *    edges toward stone.
 *  • Different themes change colours but the cave shape is identical
 *    for the same seed (algorithm is theme-independent).
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

    /* How many smoothing passes. 4 is the textbook value; below 3
     * looks noisy, above 6 dissolves small features. */
    N_ITERATIONS      =   4,

    SIM_FPS_MIN       =  10,
    SIM_FPS_DEFAULT   =  60,
    SIM_FPS_MAX       = 240,
    SIM_FPS_STEP      =  10,

    /* Cells processed per scene_tick. Default 128 ⇒ ~7700 cells/sec
     * at 60 Hz, finishing one iteration over a 10K-cell map in ~1.3
     * seconds. 4 iterations ≈ 5–6 s — long enough to see each pass
     * smooth the noise. */
    OPS_PER_TICK_MIN  =   1,
    OPS_PER_TICK_DEF  = 128,
    OPS_PER_TICK_MAX  = 2048,

    HUD_COLS          =  72,
    FPS_UPDATE_MS     = 500,

    /* Color pair indices — PAIR_HUD/PAIR_HINT reserved per CLAUDE.md. */
    PAIR_HUD          =   1,
    PAIR_HINT         =   2,
    PAIR_WALL         =   3,        /* '#' walls — theme wall colour    */
    PAIR_FLOOR        =   4,        /* '.' floors — theme floor colour  */
    PAIR_CHANGE       =   5,        /* cell that just flipped — bright  */
    PAIR_SCAN         =   6,        /* unused here — reserved for theme */
    PAIR_FLASH        =   7,        /* iteration-boundary flash         */
    PAIR_SUPERNOVA    =   8,        /* yellow reset flash               */
};

/* Glow decay rates. */
#define CHANGE_GLOW_DECAY   3.0f    /* cell-flipped flash duration ~0.6 s */
#define SUPERNOVA_DECAY     4.0f
#define GLOW_THRESHOLD      0.05f

#define HOLD_SECONDS        2.5f

/* Random fill density at iteration 0. 0.45 is the classic value. */
#define FILL_RATIO          0.45f

/* Tile kinds. */
enum { TILE_FLOOR = 0, TILE_WALL = 1 };

#define NS_PER_SEC  1000000000LL
#define NS_PER_MS      1000000LL
#define TICK_NS(f)  (NS_PER_SEC / (f))

/*
 * Themes — 10 named palettes. Same 10 names as the other procedural
 * showcases for consistency. Each theme defines four colours:
 *   wall   — the resting stone (dim, low contrast)
 *   floor  — carved cells (brighter than wall)
 *   change — cells that just flipped state (bright accent)
 *   scan   — the wavefront sweep highlight (currently unused;
 *            reserved so themes stay swappable with cave_walk file)
 */
typedef struct {
    const char *name;
    short       wall, floor, change, scan;
} Theme;

#define N_THEMES 10

static const Theme themes[N_THEMES] = {
    /*           name      wall floor change scan */
    { "DEFAULT",  240,   67,  226,  220 },   /* grey / blue / yellow / gold  */
    { "MATRIX",    22,   34,   46,  118 },   /* dark green → bright          */
    { "NOVA",      53,  129,  201,  219 },   /* purple → magenta accent      */
    { "MONO",     234,  244,  254,  250 },   /* greyscale gradient           */
    { "OCEAN",     17,   33,   51,   39 },   /* navy → cyan flash            */
    { "FIRE",      52,  124,  226,  196 },   /* dark red → yellow flash      */
    { "EARTH",     58,  137,  230,  173 },   /* brown → cream                */
    { "FOREST",    22,   64,  118,   28 },   /* dark green → bright accent   */
    { "DESERT",    94,  222,  230,  178 },   /* brown → sand                 */
    { "ARCTIC",    18,   39,  231,  159 },   /* navy → white flash           */
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
        init_pair(PAIR_WALL,   t->wall,   -1);
        init_pair(PAIR_FLOOR,  t->floor,  -1);
        init_pair(PAIR_CHANGE, t->change, -1);
        init_pair(PAIR_SCAN,   t->scan,   -1);
    } else {
        init_pair(PAIR_WALL,   COLOR_WHITE,   -1);
        init_pair(PAIR_FLOOR,  COLOR_BLUE,    -1);
        init_pair(PAIR_CHANGE, COLOR_YELLOW,  -1);
        init_pair(PAIR_SCAN,   COLOR_CYAN,    -1);
    }
}

static void color_init(void)
{
    start_color();
    use_default_colors();
    if (COLORS >= 256) {
        init_pair(PAIR_HUD,        226, -1);
        init_pair(PAIR_HINT,        51, -1);
        init_pair(PAIR_FLASH,      220, -1);
        init_pair(PAIR_SUPERNOVA,  226, -1);
    } else {
        init_pair(PAIR_HUD,       COLOR_YELLOW,  -1);
        init_pair(PAIR_HINT,      COLOR_CYAN,    -1);
        init_pair(PAIR_FLASH,     COLOR_YELLOW,  -1);
        init_pair(PAIR_SUPERNOVA, COLOR_YELLOW,  -1);
    }
    theme_apply(0);
}

/* ===================================================================== */
/* §5  ca — Cellular Automata cave                                        */
/* ===================================================================== */

/*
 * CACave — the simulation heart.
 *
 *   tiles[]      : current state (read from during the sweep)
 *   next_tiles[] : next-iteration state (written to during the sweep)
 *   change_glow[]: per-cell accent flash on cells that flipped
 *   supernova_glow[]: per-cell reset flash
 *
 *   iteration  : 0..N_ITERATIONS, the smoothing pass we're on
 *   sweep_idx  : 0..total_cells, position within current iteration
 *   wall_count : number of WALL cells in the current state — updated
 *                incrementally during the sweep, recomputed at swap
 *   done       : true once iteration == N_ITERATIONS
 */
typedef struct {
    int     w, h;
    int     total_cells;

    uint8_t tiles     [CELLS_MAX];
    uint8_t next_tiles[CELLS_MAX];
    float   change_glow   [CELLS_MAX];
    float   supernova_glow[CELLS_MAX];

    int     iteration;
    int     sweep_idx;
    int     wall_count;
    bool    done;
} CACave;

static inline int ca_idx(const CACave *c, int x, int y) { return y * c->w + x; }

/*
 * ca_count_wall_neighbours — count WALL cells in the 8-cell Moore
 * neighbourhood of (x, y). Reads from tiles[] (the current state, not
 * next_tiles[]). Out-of-grid neighbours count as WALL — keeps the
 * border solid so caves don't reach the map edge.
 */
static int ca_count_wall_neighbours(const CACave *c, int x, int y)
{
    int n = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= c->w || ny < 0 || ny >= c->h) {
                n++;                    /* out-of-grid → WALL */
            } else if (c->tiles[ca_idx(c, nx, ny)] == TILE_WALL) {
                n++;
            }
        }
    }
    return n;
}

/*
 * ca_step_one_cell — apply the 4-5 rule to a single cell at sweep_idx
 * and advance sweep_idx. Caller loops to do many cells per scene_tick.
 *
 * Returns true if a cell was processed; false when the algorithm has
 * fully converged (all iterations done).
 */
static bool ca_step_one_cell(CACave *c)
{
    if (c->done) return false;

    /* End of current iteration — swap buffers and advance. */
    if (c->sweep_idx >= c->total_cells) {
        memcpy(c->tiles, c->next_tiles, c->total_cells);
        c->iteration++;
        c->sweep_idx = 0;
        if (c->iteration >= N_ITERATIONS) {
            c->done = true;
            return false;
        }
        return true;                    /* "did work" — bumped iteration */
    }

    int idx = c->sweep_idx;
    int x   = idx % c->w;
    int y   = idx / c->w;

    int n = ca_count_wall_neighbours(c, x, y);
    uint8_t was     = c->tiles[idx];
    uint8_t becomes = (n >= 5) ? TILE_WALL : TILE_FLOOR;
    c->next_tiles[idx] = becomes;

    if (becomes != was) {
        c->change_glow[idx] = 1.0f;
        /* wall_count maintenance — accurate for the displayed mix
         * (which uses next for processed cells, tiles for the rest). */
        if (becomes == TILE_WALL) c->wall_count++;
        else                       c->wall_count--;
    }
    c->sweep_idx++;
    return true;
}

/*
 * ca_reset — random seed at FILL_RATIO walls, supernova flash on every
 * cell. Also resets iteration counters.
 */
static void ca_reset(CACave *c, int w, int h)
{
    c->w = w;
    c->h = h;
    c->total_cells = w * h;
    c->iteration   = 0;
    c->sweep_idx   = 0;
    c->done        = false;
    c->wall_count  = 0;

    for (int i = 0; i < c->total_cells; i++) {
        bool wall = (((float)rand() / (float)RAND_MAX) < FILL_RATIO);
        c->tiles[i]          = wall ? TILE_WALL : TILE_FLOOR;
        c->next_tiles[i]     = c->tiles[i];
        c->change_glow[i]    = 0.0f;
        c->supernova_glow[i] = 1.0f;
        if (wall) c->wall_count++;
    }
}

/* ===================================================================== */
/* §6  scene                                                              */
/* ===================================================================== */

/*
 * Scene state machine:
 *
 *   ITERATING — call ca_step_one_cell up to ops_per_tick times. When
 *               ca_step_one_cell returns false (done==true), transition
 *               to HOLD.
 *   HOLD      — wait HOLD_SECONDS, then ca_reset and back to ITERATING.
 */
typedef enum {
    SCENE_ITERATING = 0,
    SCENE_HOLD      = 1,
} SceneState;

typedef struct {
    CACave      c;
    SceneState  state;
    float       hold_timer;
    bool        paused;
    int         ops_per_tick;
    int         current_theme;
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    ca_reset(&s->c, mw, mh);
    s->state      = SCENE_ITERATING;
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
    float change_d = expf(-CHANGE_GLOW_DECAY * dt);
    float nova_d   = expf(-SUPERNOVA_DECAY   * dt);
    int n = s->c.total_cells;
    for (int i = 0; i < n; i++) {
        s->c.change_glow[i]    *= change_d;
        s->c.supernova_glow[i] *= nova_d;
    }

    switch (s->state) {

    case SCENE_ITERATING:
        for (int i = 0; i < s->ops_per_tick; i++) {
            if (!ca_step_one_cell(&s->c)) {
                s->state      = SCENE_HOLD;
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->c.w, s->c.h);
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

/*
 * tile_state_for_render — for cell idx, return the tile state that
 * should be displayed: NEW (next_tiles) if the sweep has already
 * processed it this iteration, OLD (tiles) otherwise.
 *
 * This is the wavefront trick: during a sweep, the user sees a clear
 * boundary between processed and unprocessed regions. After the swap
 * (end of iteration), tiles == next_tiles so the distinction is moot
 * until the next iteration starts.
 */
static inline uint8_t tile_state_for_render(const CACave *c, int idx)
{
    return (idx < c->sweep_idx) ? c->next_tiles[idx] : c->tiles[idx];
}

/*
 * tile_has_floor_neighbour_using_view — 8-connected check using the
 * "current visible state" (mid-sweep mix of tiles[] and next_tiles[]).
 * Used to decide whether a wall cell should render as '#' (next to a
 * floor) or as ' ' (interior void).
 */
static bool tile_has_floor_neighbour_using_view(const CACave *c, int x, int y)
{
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= c->w || ny < 0 || ny >= c->h) continue;
            if (tile_state_for_render(c, ca_idx(c, nx, ny)) == TILE_FLOOR)
                return true;
        }
    }
    return false;
}

static void scene_draw(const Scene *s, int cols, int rows)
{
    const CACave *c = &s->c;

    int gx0 = (cols - c->w) / 2;
    int gy0 = ((rows - 3) - c->h) / 2 + 2;   /* row 0+1 HUD, last row hint */
    if (gx0 < 0) gx0 = 0;
    if (gy0 < 2) gy0 = 2;

    for (int y = 0; y < c->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < c->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            int idx = ca_idx(c, x, y);
            float ng = c->supernova_glow[idx];
            float cg = c->change_glow[idx];
            uint8_t k = tile_state_for_render(c, idx);

            int  pair = -1, attr = A_NORMAL;
            char glyph = ' ';

            if (ng > GLOW_THRESHOLD) {
                pair = PAIR_SUPERNOVA;
                attr = A_BOLD;
                glyph = '*';
            } else if (cg > GLOW_THRESHOLD) {
                /* Cell flipped during this iteration — bright accent. */
                pair = PAIR_CHANGE;
                attr = A_BOLD;
                glyph = (k == TILE_FLOOR) ? '.' : '#';
            } else if (k == TILE_FLOOR) {
                pair = PAIR_FLOOR;
                attr = A_DIM;
                glyph = '.';
            } else {
                /* WALL — only render where adjacent to floor (using
                 * the current view, not raw tiles[]). */
                if (tile_has_floor_neighbour_using_view(c, x, y)) {
                    pair = PAIR_WALL;
                    attr = A_NORMAL;
                    glyph = '#';
                } else {
                    continue;
                }
            }

            attron(COLOR_PAIR(pair) | attr);
            mvaddch(sy, sx, (chtype)(unsigned char)glyph);
            attroff(COLOR_PAIR(pair) | attr);
        }
    }
}

static void screen_draw(Screen *sc, const Scene *s,
                        double fps, int sim_fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const CACave *c = &s->c;
    const char *state_str =
        s->paused                       ? "PAUSED   " :
        (s->state == SCENE_ITERATING)   ? "ITERATING" :
                                          "HOLD     ";

    /* Sweep progress within current iteration. */
    int sweep_pct = (c->total_cells > 0)
                  ? (100 * c->sweep_idx / c->total_cells)
                  : 0;

    /* Row 0 right — primary state. */
    char buf[HUD_COLS + 1];
    snprintf(buf, sizeof buf,
             " %5.1f fps  %3d Hz  ops:%-3d  %s  iter %d/%d  %3d%% ",
             fps, sim_fps, s->ops_per_tick, state_str,
             c->iteration, N_ITERATIONS, sweep_pct);
    int hx = sc->cols - (int)strlen(buf);
    if (hx < 0) hx = 0;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, hx, "%s", buf);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 0 left — title. */
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(0, 1, " CA CAVE 4-5 RULE ");
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);

    /* Row 1 left — theme + algorithm parameters. */
    int x = 1;
    attron(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    mvprintw(1, x, " theme:%-8s ", themes[s->current_theme].name);
    attroff(COLOR_PAIR(PAIR_HUD) | A_BOLD);
    x += 17;

    /* Wall percentage of TOTAL cells — shows how the rule shifts the
     * mix away from FILL_RATIO with each iteration. */
    int wall_pct = (c->total_cells > 0)
                 ? (100 * c->wall_count / c->total_cells)
                 : 0;
    attron(COLOR_PAIR(PAIR_HUD));
    mvprintw(1, x,
             " rule:B5678/S45678  iters:%d  fill-init:%d%%  walls:%d%%  map:%dx%d ",
             N_ITERATIONS, (int)(FILL_RATIO * 100.0f), wall_pct, c->w, c->h);
    attroff(COLOR_PAIR(PAIR_HUD));

    /* Bottom hint. */
    attron(COLOR_PAIR(PAIR_HINT) | A_BOLD);
    mvprintw(sc->rows - 1, 0,
             " #:wall  .:floor  *:flip-flash | t/T:theme  r:reset  spc:pause  +/-:speed  q:quit ");
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
