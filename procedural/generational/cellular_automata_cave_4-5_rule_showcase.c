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
 * Section map (layered — see ARCHITECTURE below):
 *   §1 config   — map size, fill ratio, iterations, threshold, themes
 *   §2 clock    — monotonic timer + sleep (PERFORMANCE / DELAYS infra)
 *   §3 color    — HUD pairs + 10 cave themes (RENDER setup)
 *   §5 model    — STATE · pure LOGIC (4-5 rule) · EFFECTS glow · SIMULATION
 *   §6 scene    — orchestration: the one place the layers combine + HOLD
 *   §7 render   — state → ASCII: wavefront sweep, # walls, . floors, HUD
 *   §8 app      — PERFORMANCE loop, signals, resize, input
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
 * References     : ALGORITHM — the 4-5 rule and CA cave generation
 *                  • Johnson, Yannakakis & Togelius (2010) — "Cellular
 *                    Automata for Real-time Generation of Infinite Cave
 *                    Levels", Proc. Workshop on PCG in Games (PCGames'10),
 *                    ACM. The seminal paper: the exact birth/survival
 *                    smoothing rule used here, applied to roguelike caves.
 *                  • RogueBasin — "Cellular Automata Method for Generating
 *                    Random Cave-Like Levels" — the practical, code-first
 *                    tutorial this demo follows most closely:
 *                    http://www.roguebasin.com/index.php?title=Cellular_Automata_Method_for_Generating_Random_Cave-Like_Levels
 *                  • Shaker, Togelius & Nelson (2016) — "Procedural
 *                    Content Generation in Games", Springer (free at
 *                    pcgbook.com). The PCG textbook; CA caves sit in the
 *                    wider map-generation landscape here.
 *                  • Wolfram, Stephen (2002) — "A New Kind of Science",
 *                    Wolfram Media (free at wolframscience.com). The
 *                    foundational study of why simple local rules on
 *                    grids produce complex global structure.
 *                  • Gardner, Martin (1970) — "Mathematical Games: The
 *                    fantastic combinations of John Conway's new solitaire
 *                    game 'life'", Scientific American 223. Origin of the
 *                    Moore-neighbourhood birth/survival machinery; Life's
 *                    B3/S23 is the same engine, a different rule.
 *
 *                  RENDERING — terminal / ASCII output
 *                  • Padala, Pradeep — "NCURSES Programming HOWTO" (TLDP).
 *                    The reference for the erase→draw→doupdate frame model,
 *                    colour pairs, and non-blocking input this file uses.
 *                  • Wikipedia — "Cellular automaton" — quick reference for
 *                    neighbourhood definitions and rule notation:
 *                    https://en.wikipedia.org/wiki/Cellular_automaton
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

/* ── ARCHITECTURE ─────────────────────────────────────────────────────── *
 *
 * This program is built from SIX layers, kept in separate labelled places
 * so each can be read without the others — and so a change to one (say the
 * glow) provably cannot break another (the cave).
 *
 *   LAYER         WHAT IT IS                          WHERE      MUTATES
 *   ─────────────────────────────────────────────────────────────────────
 *   LOGIC         the pure 4-5 rule + neighbour       §5.2       nothing
 *                 count — a decision, no state                   (const in)
 *   SIMULATION    advancing the cave: seed, sweep,    §5.4 §6    tiles[],
 *                 double-buffer swap, progress                   progress
 *   EFFECTS       cosmetic glow (flip flash, reset    §5.3 §6    glow[] only
 *                 supernova) + its exponential decay
 *   RENDER        state → glyph/colour → ncurses,     §3  §7     screen only
 *                 pure reads of the simulation
 *   DELAYS        pause, post-convergence HOLD, and   §6  §8     timers
 *                 the inter-frame sleep
 *   PERFORMANCE   ops_per_tick work throttle +        §6  §8     —
 *                 fixed-timestep accumulator + dt cap
 *
 * SIGNATURE CONVENTION — a call site tells you whether state changes:
 *   • const Grid * / const Cave * / const Scene *  → a pure read (LOGIC
 *                                       or RENDER).
 *   • non-const pointer               → an EFFECT: the function mutates
 *                                       what it points at.
 *
 * THE ONE COMBINING POINT is scene_tick() (§6): every per-step concern
 * appears there once, in order — DELAY guard → EFFECTS decay → SIMULATION
 * (under the PERFORMANCE throttle) → DELAY state machine. Nothing else
 * advances state; the render path (§7) only reads.
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

    /* Smallest playable cave; below this a tiny terminal would give a
     * degenerate grid. app_pick_map_size clamps to [MIN, MAX]. */
    MAP_W_MIN         =  16,
    MAP_H_MIN         =   8,

    /* How many smoothing passes. 4 is the textbook value; below 3
     * looks noisy, above 6 dissolves small features. */
    N_ITERATIONS      =   4,

    /* The rule's only knob: a cell becomes WALL when at least this many
     * of its 8 Moore neighbours are walls (else FLOOR). 5 = "wall is the
     * local majority". Lower → walls spread; higher → walls erode. */
    WALL_THRESHOLD    =   5,

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

    FPS_UPDATE_MS     = 500,

    /* Rows reserved for the HUD: one data bar (top) + one action bar
     * (bottom). The cave is centred in the band between them. */
    HUD_ROW_TOP       =   1,
    HUD_ROW_BOTTOM    =   1,

    /* Frame pacing. RENDER_FPS_CAP throttles screen redraws and is
     * independent of the simulation tick rate (Control.sim_fps). DT_CAP_MS
     * clamps one frame's measured dt so a long stall (debugger, resize)
     * can't accumulate into a spiral of death. */
    RENDER_FPS_CAP    =  60,
    DT_CAP_MS         = 100,

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
 * Theme — one named colour palette for the cave, cycled live with t/T.
 *
 * WHY a struct (not loose colour constants): the algorithm is wholly
 * colour-independent (same seed ⇒ same cave shape under any theme), so the
 * palette is data, swapped at runtime without touching a line of logic.
 * Keeping the four roles together as one record is what makes themes[]
 * below a flat, eyeball-able table.
 *
 * VALUE LOGIC: every short is an xterm-256 colour index (0..255), fed to
 * init_pair() in theme_apply(). Per the project's palette-brightness rule
 * (CLAUDE.md), even the darkest tier must sit in the bright half of the
 * cube — indices < ~24 / grays < ~240 vanish against the default-black
 * background under A_DIM. The four roles form a low→high contrast ramp.
 *
 * Ref: 256-colour cube layout & escape-time/density colouring —
 *      documentation/COLOR.md.
 */
typedef struct {
    const char *name;   /* HUD label shown as "theme:NAME (i/N)"            */
    short       wall;    /* resting stone — lowest contrast, the bulk tier  */
    short       floor;   /* carved/open cells — one step brighter than wall */
    short       change;  /* freshly-flipped cells — bright accent flash     */
    short       scan;    /* wavefront-sweep highlight; UNUSED here, kept so
                          * these palettes stay drop-in with the sibling
                          * drunkards_walk_cave_showcase.c theme table      */
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
/* §5  model — STATE · LOGIC · EFFECTS · SIMULATION                       */
/* ===================================================================== */
/*
 * The heart of the program, split into four layers (see ARCHITECTURE).
 * Reading order is dependency order: each layer uses only the ones above.
 *   5.1 STATE       the data the cave lives in
 *   5.2 LOGIC       the pure rule — no state, no side effects
 *   5.3 EFFECTS     the cosmetic glow — never read by the rule
 *   5.4 SIMULATION  the machine that advances STATE using LOGIC + EFFECTS
 */

/* ── 5.1 STATE — the data structures the cave is built from ──────────── *
 * Three small abstractions, each named for its concept and readable on its
 * own, then composed into one Cave:
 *
 *   Grid  — a DOUBLE BUFFER of tiles: the rule reads read[] and writes
 *           write[], then grid_commit() makes write[] the new read[].
 *           Separating reads from writes is what lets a whole generation
 *           update "at once" (MENTAL MODEL: double buffering).
 *   Glow  — a DECAYING per-cell scalar field in [0,1]. Pure cosmetics,
 *           used twice (flip accent, reset supernova). The rule never
 *           reads a Glow — that is what makes the EFFECTS layer safe.
 *   Sweep — the WAVEFRONT CURSOR: which pass we are on, and how far the
 *           top-left → bottom-right scan has reached within it.
 */
/*
 * Grid — the cave's tiles held as a DOUBLE BUFFER, the defining data
 * structure of a synchronous cellular automaton.
 *
 * WHY two buffers: the 4-5 rule must see the WHOLE previous generation
 * while computing the next one. If it wrote back into the same array, a
 * cell processed early would corrupt the neighbour counts of cells
 * processed later, biasing the result toward traversal order. Reading
 * read[] and writing write[], then committing, makes every cell update as
 * if "simultaneously" — order no longer matters (MENTAL MODEL: double
 * buffering; EDGE CASES: "double buffering is essential").
 *
 * WHY memcpy commit (grid_commit), not a pointer swap: the renderer shows
 * a mid-sweep MIX of both buffers (write[] for cells already processed,
 * read[] for the rest), so both must stay valid contiguous arrays; a swap
 * of pointers would also work but flat arrays keep indexing trivial.
 *
 * Ref: Johnson, Yannakakis & Togelius (2010); RogueBasin CA caves — see
 *      the References block at the top of the file.
 */
typedef struct {
    int     w, h, n;               /* width, height, n = w*h (cached count)   */
    uint8_t read [CELLS_MAX];      /* current generation  — LOGIC reads this  */
    uint8_t write[CELLS_MAX];      /* next generation     — SIMULATION writes */
} Grid;

/*
 * Glow — a per-cell scalar "brightness" field in [0,1] that fades over time.
 *
 * WHY a whole field: the flash is per-cell and time-based, so each cell
 * carries its own intensity. WHY exponential decay (glow_decay multiplies
 * by exp(-rate·dt) each tick): it is frame-rate independent and never
 * reaches exactly 0, so the renderer uses a small GLOW_THRESHOLD cutoff.
 * 1.0 = just lit, 0 = fully faded. Purely cosmetic — the rule never reads
 * it, which is precisely what lets the EFFECTS layer stay decoupled.
 */
typedef struct {
    float v[CELLS_MAX];            /* per-cell intensity in [0,1], decays → 0 */
} Glow;

/*
 * Sweep — the WAVEFRONT CURSOR: a serial scan position used to ANIMATE the
 * (conceptually parallel) generation update one cell at a time.
 *
 * WHY it exists: the algorithm itself has no traversal order (the double
 * buffer guarantees that). The sweep is a presentation choice — walking
 * cells in row-major order makes a visible boundary crawl across the map
 * so the viewer watches the smoothing happen instead of seeing it pop.
 */
typedef struct {
    int  iteration;                /* 0..N_ITERATIONS — which smoothing pass  */
    int  idx;                      /* 0..Grid.n       — cell within this pass */
    bool done;                     /* true once every pass has run            */
} Sweep;

/*
 * Cave — the simulation model: the double-buffered Grid, the two cosmetic
 * Glow fields, the Sweep cursor, and the live wall tally. Each member is a
 * named abstraction, so "what is being simulated" reads at a glance. The
 * split mirrors the layers: grid+sweep+wall_count are SIMULATION/PROGRESS;
 * flip+nova are EFFECTS.
 */
typedef struct {
    Grid  grid;          /* tiles (double buffer) — SIMULATION              */
    Glow  flip;          /* EFFECTS: accent on a freshly-flipped cell       */
    Glow  nova;          /* EFFECTS: supernova flash, lit on every re-seed  */
    Sweep sweep;         /* PROGRESS: wavefront cursor                      */
    int   wall_count;    /* walls in the DISPLAYED mix; drives HUD walls%.
                          * Maintained incrementally as cells flip so the
                          * readout is correct mid-sweep, not just at end. */
} Cave;

/* Grid ops — indexing and the double-buffer commit (end-of-pass swap). */
static inline int  grid_idx   (const Grid *g, int x, int y) { return y * g->w + x; }
static inline void grid_commit(Grid *g) { memcpy(g->read, g->write, g->n); }

/* Glow ops — the decaying-field primitives, shared by both Glow instances. */
static inline void  glow_light(Glow *f, int idx)       { f->v[idx] = 1.0f; }
static inline float glow_at   (const Glow *f, int idx) { return f->v[idx]; }
static void glow_fill (Glow *f, int n, float value) { for (int i = 0; i < n; i++) f->v[i] = value; }
static void glow_decay(Glow *f, int n, float rate, float dt)
{
    float k = expf(-rate * dt);
    for (int i = 0; i < n; i++) f->v[i] *= k;
}

/* ── 5.2 LOGIC (pure — const Grid*, mutates nothing) ─────────────────── */

/*
 * moore_wall_count — count WALL cells in the 8-cell MOORE neighbourhood of
 * (x, y), reading the CURRENT generation (grid->read, never write[]).
 * Out-of-grid neighbours count as WALL so the border stays solid. Pure
 * read: takes a const pointer, changes nothing.
 */
static int moore_wall_count(const Grid *g, int x, int y)
{
    int n = 0;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h)
                n++;                              /* out-of-grid → WALL */
            else if (g->read[grid_idx(g, nx, ny)] == TILE_WALL)
                n++;
        }
    }
    return n;
}

/*
 * ca_rule — THE rule, as a pure function: a cell's next kind is decided
 * solely by how many of its 8 neighbours are walls. Walls in the local
 * majority (>= WALL_THRESHOLD of 8) → WALL, else FLOOR. This one
 * expression IS the algorithm; everything else is scheduling and
 * presentation. Read it in isolation and check it against the spec.
 *
 * Note: the canonical RogueBasin "4-5 rule" uses an easier survival
 * threshold than birth (a wall survives on >=4, a floor is born on >=5).
 * This file applies a single >=5 threshold to both, independent of the
 * current state — simpler, and what these visuals were tuned against.
 */
static inline uint8_t ca_rule(int wall_neighbours)
{
    return (wall_neighbours >= WALL_THRESHOLD) ? TILE_WALL : TILE_FLOOR;
}

/* ── 5.3 EFFECTS (cosmetic — writes ONLY the cave's Glow fields) ─────── *
 * Cave-level effect verbs, each composing the Glow primitives above. LOGIC
 * never reads a Glow, so a bug here can dim or brighten the screen but can
 * never corrupt the cave. That isolation is why they live in one place.
 */

/* cave_flash_flip — light the flip accent on one cell (fired by SIMULATION
 * when a cell changes kind). */
static inline void cave_flash_flip(Cave *cave, int idx) { glow_light(&cave->flip, idx); }

/* cave_flash_supernova — clear stale flip glow and light the reset flash on
 * every cell. Fired once per re-seed. */
static void cave_flash_supernova(Cave *cave)
{
    glow_fill(&cave->flip, cave->grid.n, 0.0f);
    glow_fill(&cave->nova, cave->grid.n, 1.0f);
}

/* cave_decay_glow — fade both cosmetic fields by one sim tick, each at its
 * own rate. Pure cosmetic time-evolution; dt is the fixed sim timestep. */
static void cave_decay_glow(Cave *cave, float dt)
{
    glow_decay(&cave->flip, cave->grid.n, CHANGE_GLOW_DECAY, dt);
    glow_decay(&cave->nova, cave->grid.n, SUPERNOVA_DECAY,   dt);
}

/* ── 5.4 SIMULATION (state advance — mutates Grid / Sweep / tally) ───── */

/* roll_wall — one Bernoulli trial: returns true with probability
 * FILL_RATIO. Each cell draws independently — this is the salt-and-pepper
 * noise the smoothing rule then organises into caverns. */
static inline bool roll_wall(void)
{
    return ((float)rand() / (float)RAND_MAX) < FILL_RATIO;
}

/*
 * cave_seed — start a fresh cave: random salt-and-pepper fill at FILL_RATIO
 * walls, Sweep and wall tally reset. SIMULATION state ONLY (Grid + Sweep +
 * count) — it does not touch the Glow fields (the caller fires
 * cave_flash_supernova for the flash), keeping the layers separable.
 */
static void cave_seed(Cave *cave, int w, int h)
{
    Grid *g = &cave->grid;
    g->w = w;
    g->h = h;
    g->n = w * h;

    cave->sweep.iteration = 0;
    cave->sweep.idx       = 0;
    cave->sweep.done      = false;
    cave->wall_count      = 0;

    for (int i = 0; i < g->n; i++) {
        bool wall = roll_wall();
        g->read[i]  = wall ? TILE_WALL : TILE_FLOOR;
        g->write[i] = g->read[i];
        if (wall) cave->wall_count++;
    }
}

/*
 * cave_advance — advance the simulation by one unit of work; return true if
 * work was done, false once fully converged. A unit is either:
 *   • one cell — apply LOGIC (ca_rule on the Moore count), write the
 *     grid's write buffer, keep wall_count in step, and on a flip fire
 *     the EFFECTS flash; then step the Sweep cursor; or
 *   • one iteration boundary — grid_commit() the double buffer and bump
 *     the pass (the double buffer is why traversal order doesn't matter).
 * The caller loops this ops_per_tick times (the PERFORMANCE throttle).
 */
static bool cave_advance(Cave *cave)
{
    Grid  *g  = &cave->grid;
    Sweep *sw = &cave->sweep;

    if (sw->done) return false;

    /* Iteration boundary: commit the buffer, start the next pass. */
    if (sw->idx >= g->n) {
        grid_commit(g);
        sw->iteration++;
        sw->idx = 0;
        if (sw->iteration >= N_ITERATIONS) sw->done = true;
        return !sw->done;               /* still work left? */
    }

    int x = sw->idx % g->w;
    int y = sw->idx / g->w;

    uint8_t was     = g->read[sw->idx];
    uint8_t becomes = ca_rule(moore_wall_count(g, x, y));   /* LOGIC */
    g->write[sw->idx] = becomes;

    if (becomes != was) {
        /* wall_count tracks the displayed mix (write[] for processed
         * cells, read[] for the rest) so the HUD reads true mid-sweep. */
        cave->wall_count += (becomes == TILE_WALL) ? +1 : -1;   /* SIMULATION */
        cave_flash_flip(cave, sw->idx);                         /* EFFECT     */
    }
    sw->idx++;
    return true;
}

/* ===================================================================== */
/* §6  scene — orchestration: where the layers combine                    */
/* ===================================================================== */

/*
 * SceneState — the two phases of the show, i.e. the DELAYS layer's outer
 * shape. WHY a state machine at all: generation is finite (it converges
 * after N_ITERATIONS), so the program alternates "build a cave" with
 * "show it off, then start over" rather than running forever.
 */
typedef enum {
    SCENE_ITERATING = 0,  /* building: advance SIMULATION ops_per_tick cells
                           * per tick; on cave_advance convergence → HOLD   */
    SCENE_HOLD      = 1,  /* finished: linger HOLD_SECONDS on the result,
                           * then re-seed and return to ITERATING            */
} SceneState;

/*
 * Control — the user-tunable knobs, gathered in one place: every field is
 * something a key changes, nothing the algorithm decides on its own.
 * Separating these from the Cave makes one fact obvious — turning any knob
 * changes the PACE or the COLOUR of the show, never the cave it produces.
 * Bounds for each live in §1 config; app_handle_key clamps to them.
 */
typedef struct {
    bool paused;          /* space — freeze: scene_tick early-returns       */
    int  ops_per_tick;    /* +/-   — PERFORMANCE throttle, cells per tick;
                           *         OPS_PER_TICK_MIN..MAX, doubles/halves   */
    int  sim_fps;         /* [ ]   — sim tick rate Hz; SIM_FPS_MIN..MAX.
                           *         Sets the fixed timestep, NOT render fps */
    int  theme;           /* t/T   — index 0..N_THEMES-1 into themes[]       */
} Control;

/*
 * Scene — the whole animated showcase in one structure, the natural home
 * for what would otherwise be scattered globals. Three concerns, kept
 * apart so each reads cleanly: WHAT is simulated, HOW the user drives it,
 * and WHERE we are in the build/show cycle.
 */
typedef struct {
    Cave        cave;        /* WHAT is simulated: Grid + Glow + Sweep + tally */
    Control     ctrl;        /* HOW the user drives it: the knobs              */
    SceneState  state;       /* ITERATING or HOLD                              */
    float       hold_timer;  /* seconds left in HOLD; counts down by dt, and
                              * at <=0 triggers a re-seed (unused in ITERATING) */
} Scene;

static void scene_reset(Scene *s, int mw, int mh)
{
    cave_seed(&s->cave, mw, mh);        /* SIMULATION: fresh random cave      */
    cave_flash_supernova(&s->cave);     /* EFFECTS:    reset flash everywhere */
    s->state      = SCENE_ITERATING;
    s->hold_timer = 0.0f;
}

static void scene_init(Scene *s, int mw, int mh)
{
    memset(s, 0, sizeof *s);
    s->ctrl.paused       = false;
    s->ctrl.ops_per_tick = OPS_PER_TICK_DEF;
    s->ctrl.sim_fps      = SIM_FPS_DEFAULT;
    s->ctrl.theme        = 0;
    scene_reset(s, mw, mh);
}

/*
 * scene_tick — THE one place the layers combine, in fixed order. Read top
 * to bottom and you have the whole per-step behaviour, each line a single
 * concern: DELAY guard → EFFECTS → SIMULATION (under the PERFORMANCE
 * throttle) → DELAY state machine. dt is the fixed sim timestep.
 */
static void scene_tick(Scene *s, float dt)
{
    if (s->ctrl.paused) return;          /* DELAY: frozen — nothing advances */

    cave_decay_glow(&s->cave, dt);       /* EFFECTS: fade both glow layers   */

    switch (s->state) {

    case SCENE_ITERATING:
        /* PERFORMANCE throttle: bounded work per tick so the sweep plays
         * out over seconds rather than finishing in a single frame. */
        for (int i = 0; i < s->ctrl.ops_per_tick; i++) {
            if (!cave_advance(&s->cave)) {       /* SIMULATION */
                s->state      = SCENE_HOLD;      /* DELAY: linger on result */
                s->hold_timer = HOLD_SECONDS;
                break;
            }
        }
        break;

    case SCENE_HOLD:
        s->hold_timer -= dt;                     /* DELAY countdown */
        if (s->hold_timer <= 0.0f) {
            scene_reset(s, s->cave.grid.w, s->cave.grid.h);
        }
        break;
    }
}

/* ===================================================================== */
/* §7  render — state → ASCII (pure reads of the simulation)              */
/* ===================================================================== */

/*
 * Screen — the terminal's current size, cached from getmaxyx(). WHY cache
 * it rather than query per draw: the dimensions only change on a resize
 * (SIGWINCH → screen_resize re-reads them), and every frame needs them to
 * centre the map and right-/bottom-pin the HUD. cols/rows are in character
 * cells — this is cell-space rendering, no sub-pixel coordinates.
 */
typedef struct {
    int cols;   /* terminal width  in character columns */
    int rows;   /* terminal height in character rows    */
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

/*
 * cave_view_tile — the tile to DISPLAY at idx: the NEW state (grid.write)
 * if the sweep has already passed this cell, else the OLD state (grid.read).
 *
 * This mid-sweep mix IS the wavefront: a clear boundary between processed
 * and unprocessed cells crawls across the map. After grid_commit (end of
 * pass) read == write, so the distinction is moot until the next pass.
 */
static inline uint8_t cave_view_tile(const Cave *cave, int idx)
{
    return (idx < cave->sweep.idx) ? cave->grid.write[idx] : cave->grid.read[idx];
}

/*
 * cave_view_has_floor_neighbour — does (x,y) touch a FLOOR in the currently
 * displayed mix? Decides whether a wall renders as '#' (a cave face) or is
 * left blank (interior rock). Pure read of the view, not the raw buffers.
 */
static bool cave_view_has_floor_neighbour(const Cave *cave, int x, int y)
{
    const Grid *g = &cave->grid;
    for (int dy = -1; dy <= 1; dy++) {
        for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= g->w || ny < 0 || ny >= g->h) continue;
            if (cave_view_tile(cave, grid_idx(g, nx, ny)) == TILE_FLOOR)
                return true;
        }
    }
    return false;
}

/*
 * CellLook — how one cell should appear: colour pair, attributes, glyph,
 * and whether to draw it at all. Separating "decide appearance" (cell_look,
 * pure — it READS the Glow fields) from "stamp a glyph" (scene_draw, dumb
 * ncurses blit) keeps the EFFECTS↔RENDER boundary crisp.
 */
typedef struct {
    short pair;    /* COLOR_PAIR index (PAIR_WALL/FLOOR/CHANGE/SUPERNOVA)   */
    int   attr;    /* ncurses attribute mask: A_BOLD / A_DIM / A_NORMAL     */
    char  glyph;   /* ASCII tile: '#' wall, '.' floor, '*' flash, ' ' void  */
    bool  draw;    /* false ⇒ skip entirely (interior rock); leaves the
                    * cell blank so caverns read as empty, not solid grey   */
} CellLook;

/* cell_look — appearance priority: supernova flash > flip accent > floor >
 * cave-face wall. Interior rock (a wall with no floor neighbour) is skipped. */
static CellLook cell_look(const Cave *cave, int x, int y)
{
    int     idx = grid_idx(&cave->grid, x, y);
    uint8_t k   = cave_view_tile(cave, idx);

    if (glow_at(&cave->nova, idx) > GLOW_THRESHOLD)
        return (CellLook){ PAIR_SUPERNOVA, A_BOLD, '*', true };

    if (glow_at(&cave->flip, idx) > GLOW_THRESHOLD)
        return (CellLook){ PAIR_CHANGE, A_BOLD, (k == TILE_FLOOR) ? '.' : '#', true };

    if (k == TILE_FLOOR)
        return (CellLook){ PAIR_FLOOR, A_DIM, '.', true };

    if (cave_view_has_floor_neighbour(cave, x, y))
        return (CellLook){ PAIR_WALL, A_NORMAL, '#', true };

    return (CellLook){ 0, A_NORMAL, ' ', false };   /* interior rock — skip */
}

/* scene_draw — pure positioning + blit: centre the grid, then stamp each
 * cell's CellLook. No appearance logic here; that all lives in cell_look. */
static void scene_draw(const Scene *s, int cols, int rows)
{
    const Cave *cave = &s->cave;
    const Grid *g    = &cave->grid;

    /* Centre the grid in the band between the two HUD bars. */
    int draw_rows = rows - HUD_ROW_TOP - HUD_ROW_BOTTOM;
    int gx0 = (cols - g->w) / 2;
    int gy0 = HUD_ROW_TOP + (draw_rows - g->h) / 2;
    if (gx0 < 0)           gx0 = 0;
    if (gy0 < HUD_ROW_TOP) gy0 = HUD_ROW_TOP;

    for (int y = 0; y < g->h; y++) {
        int sy = gy0 + y;
        if (sy < 0 || sy >= rows) continue;
        for (int x = 0; x < g->w; x++) {
            int sx = gx0 + x;
            if (sx < 0 || sx >= cols) continue;

            CellLook look = cell_look(cave, x, y);
            if (!look.draw) continue;

            attron(COLOR_PAIR(look.pair) | look.attr);
            mvaddch(sy, sx, (chtype)(unsigned char)look.glyph);
            attroff(COLOR_PAIR(look.pair) | look.attr);
        }
    }
}

/*
 * hud_bar — paint one full-width status bar on `row`: fill the row with
 * spaces in `pair`, then write `buf` clipped to the terminal width with
 * mvaddnstr. Clipping (not mvprintw) is what keeps an over-long string
 * from wrapping down onto the cave below. One helper drives both the
 * top data bar and the bottom action bar.
 */
static void hud_bar(int row, int cols, int pair, const char *buf)
{
    if (row < 0 || cols < 1) return;
    attron(COLOR_PAIR(pair) | A_BOLD);
    for (int x = 0; x < cols; x++) mvaddch(row, x, ' ');
    mvaddnstr(row, 0, buf, cols);
    attroff(COLOR_PAIR(pair) | A_BOLD);
}

static void screen_draw(Screen *sc, const Scene *s, double fps)
{
    erase();
    scene_draw(s, sc->cols, sc->rows);

    const Cave    *cave = &s->cave;
    const Control *ctrl = &s->ctrl;
    int n = cave->grid.n;
    const char *state_str =
        ctrl->paused                  ? "PAUSED"    :
        (s->state == SCENE_ITERATING) ? "ITERATING" :
                                        "HOLD";
    int sweep_pct = (n > 0) ? (100 * cave->sweep.idx  / n) : 0;
    int wall_pct  = (n > 0) ? (100 * cave->wall_count / n) : 0;

    /* Row 0 — DATA: identity, live state, and every readout that used to
     * be split across two rows + the action bar, on one clipped line. */
    char data[200];
    snprintf(data, sizeof data,
             " CA_CAVE B5678/S45678  %-9s  theme:%s (%d/%d)  iter:%d/%d  "
             "sweep:%3d%%  walls:%3d%%  ops:%-4d  %5.1f fps  %3d Hz ",
             state_str, themes[ctrl->theme].name,
             ctrl->theme + 1, N_THEMES,
             cave->sweep.iteration, N_ITERATIONS,
             sweep_pct, wall_pct, ctrl->ops_per_tick, fps, ctrl->sim_fps);

    /* Last row — ACTIONS only: every interactive key, nothing else. */
    static const char *keys =
        " q:quit  spc:pause  r:reset  t/T:theme  +/-:speed  [/]:Hz ";

    hud_bar(0,            sc->cols, PAIR_HUD,  data);
    hud_bar(sc->rows - 1, sc->cols, PAIR_HINT, keys);
}

static void screen_present(void) { wnoutrefresh(stdscr); doupdate(); }

/* ===================================================================== */
/* §8  app — PERFORMANCE loop (fixed timestep) + signals + input          */
/* ===================================================================== */

/*
 * App — the top-level program object: the simulation, the screen, the
 * derived map size, and the two signal flags. WHY a single g_app global:
 * POSIX signal handlers take no user pointer, so the handlers below reach
 * the program through this one well-known instance — the only global, by
 * design, so everything else stays passed-by-pointer.
 */
typedef struct {
    Scene                 scene;          /* the showcase (model + control)   */
    Screen                screen;         /* cached terminal size             */
    int                   map_w, map_h;   /* cave size chosen to fit screen,
                                           * clamped to MAP_W/H_MAX (§1)       */
    volatile sig_atomic_t running;        /* 0 ⇒ exit main loop (SIGINT/TERM).
                                           * volatile sig_atomic_t: the only
                                           * type safe to touch in a handler   */
    volatile sig_atomic_t need_resize;    /* 1 ⇒ re-read size next frame
                                           * (set by SIGWINCH handler)         */
} App;

static App g_app;

static void on_exit_signal  (int sig) { (void)sig; g_app.running     = 0; }
static void on_resize_signal(int sig) { (void)sig; g_app.need_resize = 1; }
static void cleanup(void)             { endwin(); }

static void app_pick_map_size(App *app)
{
    /* Cave fills the width, and the height left after the two HUD rows. */
    int mw = app->screen.cols;
    int mh = app->screen.rows - HUD_ROW_TOP - HUD_ROW_BOTTOM;
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
    Control *ctrl = &app->scene.ctrl;
    switch (ch) {
    case 'q': case 'Q': case 27 /* ESC */: return false;
    case ' ':     ctrl->paused = !ctrl->paused; break;
    case 'r': case 'R':
        scene_reset(&app->scene, app->map_w, app->map_h);
        break;
    case '=': case '+':
        ctrl->ops_per_tick *= 2;
        if (ctrl->ops_per_tick > OPS_PER_TICK_MAX) ctrl->ops_per_tick = OPS_PER_TICK_MAX;
        break;
    case '-':
        ctrl->ops_per_tick /= 2;
        if (ctrl->ops_per_tick < OPS_PER_TICK_MIN) ctrl->ops_per_tick = OPS_PER_TICK_MIN;
        break;
    case ']':
        ctrl->sim_fps += SIM_FPS_STEP;
        if (ctrl->sim_fps > SIM_FPS_MAX) ctrl->sim_fps = SIM_FPS_MAX;
        break;
    case '[':
        ctrl->sim_fps -= SIM_FPS_STEP;
        if (ctrl->sim_fps < SIM_FPS_MIN) ctrl->sim_fps = SIM_FPS_MIN;
        break;

    case 't':
        ctrl->theme = (ctrl->theme + 1) % N_THEMES;
        theme_apply(ctrl->theme);
        break;
    case 'T':
        ctrl->theme = (ctrl->theme + N_THEMES - 1) % N_THEMES;
        theme_apply(ctrl->theme);
        break;

    default: break;
    }
    return true;
}

/*
 * app_step_simulation — FIXED-TIMESTEP update: bank the frame's real
 * elapsed time, then spend it one whole TICK at a time (rate = sim_fps),
 * leaving the sub-tick remainder in *sim_accum for next frame. This is
 * what decouples simulation speed from render speed: the cave advances the
 * same amount per wall-clock second no matter the frame rate.
 */
static void app_step_simulation(App *app, int64_t dt, int64_t *sim_accum)
{
    int64_t tick_ns = TICK_NS(app->scene.ctrl.sim_fps);
    float   dt_sec  = (float)tick_ns / (float)NS_PER_SEC;

    *sim_accum += dt;
    while (*sim_accum >= tick_ns) {
        scene_tick(&app->scene, dt_sec);
        *sim_accum -= tick_ns;
    }
}

/*
 * app_pace_frame — sleep so each rendered frame lasts about one
 * RENDER_FPS_CAP period, regardless of how long this frame's work took.
 * frame_start = when this iteration began; frame_dt = the previous frame's
 * measured length. Sleeping the leftover budget holds a steady cap and
 * keeps the process off a busy-spin.
 */
static void app_pace_frame(int64_t frame_start, int64_t frame_dt)
{
    int64_t budget_ns  = NS_PER_SEC / RENDER_FPS_CAP;
    int64_t elapsed_ns = clock_ns() - frame_start + frame_dt;
    clock_sleep_ns(budget_ns - elapsed_ns);
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

    screen_init(&app->screen);
    app_pick_map_size(app);
    scene_init(&app->scene, app->map_w, app->map_h);

    int64_t frame_time  = clock_ns();
    int64_t sim_accum   = 0;
    int64_t fps_accum   = 0;
    int     frame_count = 0;
    double  fps_display = 0.0;

    while (app->running) {

        /* 1. Adopt a new terminal size if SIGWINCH asked for one. */
        if (app->need_resize) {
            app_do_resize(app);
            frame_time = clock_ns();
            sim_accum  = 0;
        }

        /* 2. Measure this frame's real elapsed time, capped so a long
         *    stall can't trigger a spiral of death; advance the clock. */
        int64_t now = clock_ns();
        int64_t dt  = now - frame_time;
        frame_time  = now;
        if (dt > DT_CAP_MS * NS_PER_MS) dt = DT_CAP_MS * NS_PER_MS;

        /* 3. Advance the simulation by that much real time. */
        app_step_simulation(app, dt, &sim_accum);

        /* 4. Update the FPS readout once per FPS_UPDATE_MS window. */
        frame_count++;
        fps_accum += dt;
        if (fps_accum >= FPS_UPDATE_MS * NS_PER_MS) {
            fps_display = (double)frame_count
                        / ((double)fps_accum / (double)NS_PER_SEC);
            frame_count = 0;
            fps_accum   = 0;
        }

        /* 5. Sleep to the frame cap, THEN draw (steady pacing). */
        app_pace_frame(frame_time, dt);
        screen_draw(&app->screen, &app->scene, fps_display);
        screen_present();

        /* 6. Drain one input event. */
        int ch = getch();
        if (ch != ERR && !app_handle_key(app, ch))
            app->running = 0;
    }

    screen_free(&app->screen);
    return 0;
}
